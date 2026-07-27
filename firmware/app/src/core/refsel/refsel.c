/*
 * STS1000 "Meridian" — core/refsel implementation.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See refsel.h for the guard policy and the handoff contract.
 */

#include "refsel/refsel.h"

#include <errno.h>
#include <string.h>

/* ------------------------------------------------------------------ config */

int refsel_cfg_defaults(refsel_cfg_t *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}

	memset(cfg, 0, sizeof(*cfg));
	cfg->extref_nominal_hz = 10000000u;
	cfg->extref_band_hz = 20u;       /* +-2 ppm of 10 MHz */
	cfg->hysteresis_ms = 30000u;     /* §3.5 debounce before engaging B */
	cfg->lockout_ms = 300000u;       /* 5 min inhibit after any revert */
	cfg->settle_ms = 10u;            /* mux + HSE restart window */
	cfg->flap_window_ms = 3600000u;  /* 1 h */
	cfg->flap_alarm_n = 3u;
	cfg->extref_require_rb_lock = false;
	return 0;
}

static bool cfg_valid(const refsel_cfg_t *c)
{
	if (c->extref_nominal_hz == 0u) {
		return false;
	}
	if (c->extref_band_hz == 0u || c->extref_band_hz >= c->extref_nominal_hz) {
		return false;
	}
	if (c->flap_alarm_n == 0u || c->flap_alarm_n > REFSEL_FLAP_HISTORY) {
		return false;
	}
	if (c->flap_window_ms == 0u) {
		return false;
	}
	/*
	 * Bound the timing parameters. settle_ms is handed to the glue as a
	 * blocking WAIT_SETTLE_MS during which SYSCLK is on HSI, so an absurd
	 * value would stall the changeover and free-wheel the PTP clock far
	 * longer than intended — cap it hard at 1 s (a mux flip plus HSE
	 * restart is milliseconds). hysteresis_ms and lockout_ms are debounce
	 * windows; a week is a generous ceiling that still rejects garbage.
	 */
	if (c->settle_ms > REFSEL_MAX_SETTLE_MS) {
		return false;
	}
	if (c->hysteresis_ms > REFSEL_MAX_WINDOW_MS ||
	    c->lockout_ms > REFSEL_MAX_WINDOW_MS) {
		return false;
	}
	return true;
}

int refsel_init(refsel_ctx_t *ctx, const refsel_cfg_t *cfg)
{
	refsel_cfg_t local;

	if (ctx == NULL) {
		return -EINVAL;
	}
	if (cfg == NULL) {
		(void)refsel_cfg_defaults(&local);
		cfg = &local;
	}
	if (!cfg_valid(cfg)) {
		return -EINVAL;
	}

	memset(ctx, 0, sizeof(*ctx));
	ctx->cfg = *cfg;
	/* R188 forces MUX_SEL low before firmware runs, so the machine starts
	 * where the hardware already is. */
	ctx->state = REFSEL_OCXO_ACTIVE;
	ctx->tracked_target = REFSEL_OCXO_ACTIVE;
	ctx->initialised = true;
	return 0;
}

/* ------------------------------------------------------------------- utils */

/* Monotonic difference that cannot underflow if a caller ever hands us a
 * timestamp older than the one it gave last time. */
static uint64_t ms_since(uint64_t now, uint64_t then)
{
	return (now >= then) ? (now - then) : 0u;
}

static bool extref_in_band(const refsel_ctx_t *ctx, const refsel_in_t *in)
{
	uint32_t nom = ctx->cfg.extref_nominal_hz;
	uint32_t band = ctx->cfg.extref_band_hz;
	uint32_t f = in->extref_freq_hz;

	if (!in->extref_valid || !in->extref_edges_advancing) {
		return false;
	}
	if (f > nom) {
		return (f - nom) <= band;
	}
	return (nom - f) <= band;
}

/* Guard set for a candidate/held state. The OCXO is always permissible. */
static bool guards_for(const refsel_ctx_t *ctx, refsel_state_t target,
		       bool extref_ok, bool rb_ok)
{
	switch (target) {
	case REFSEL_RB_ACTIVE:
		return extref_ok && rb_ok;
	case REFSEL_EXTREF_ACTIVE:
		return extref_ok &&
		       (!ctx->cfg.extref_require_rb_lock || rb_ok);
	default:
		return true;
	}
}

static void emit_actions(refsel_ctx_t *ctx, refsel_state_t target)
{
	uint32_t mux = (target == REFSEL_OCXO_ACTIVE) ? REFSEL_MUX_A
						      : REFSEL_MUX_B;

	/* Park the discipline loop first so the DAC holds its last-good Vc
	 * across the whole disturbance, and unpark only once the PLL is verified
	 * — the glue must not leave the loop steering into a free-wheeling
	 * clock. See the refsel_action_t doc for why the bracket exists. */
	ctx->steps[0].act = REFSEL_ACT_PARK_DISCIPLINE;
	ctx->steps[0].arg = 0u;
	ctx->steps[1].act = REFSEL_ACT_BRIDGE_TO_HSI;
	ctx->steps[1].arg = 0u;
	ctx->steps[2].act = REFSEL_ACT_SET_MUX;
	ctx->steps[2].arg = mux;
	ctx->steps[3].act = REFSEL_ACT_WAIT_SETTLE_MS;
	ctx->steps[3].arg = ctx->cfg.settle_ms;
	ctx->steps[4].act = REFSEL_ACT_RESELECT_HSE;
	ctx->steps[4].arg = 0u;
	ctx->steps[5].act = REFSEL_ACT_VERIFY_PLL;
	ctx->steps[5].arg = 0u;
	ctx->steps[6].act = REFSEL_ACT_UNPARK_DISCIPLINE;
	ctx->steps[6].arg = 0u;
	ctx->steps[7].act = REFSEL_ACT_DONE;
	ctx->steps[7].arg = 0u;
	ctx->n_steps = REFSEL_MAX_STEPS;
}

static void start_lockout(refsel_ctx_t *ctx, uint64_t now)
{
	ctx->lockout = true;
	ctx->lockout_until_ms = now + (uint64_t)ctx->cfg.lockout_ms;
}

/* Record a fail-safe revert and re-evaluate the flap alarm. */
static void note_revert(refsel_ctx_t *ctx, uint64_t now)
{
	uint8_t i;
	uint8_t recent = 0u;

	ctx->revert_ms[ctx->revert_head] = now;
	ctx->revert_head = (uint8_t)((ctx->revert_head + 1u) %
				     REFSEL_FLAP_HISTORY);
	if (ctx->revert_n < REFSEL_FLAP_HISTORY) {
		ctx->revert_n++;
	}
	if (ctx->revert_total < UINT16_MAX) {
		ctx->revert_total++;
	}

	for (i = 0u; i < ctx->revert_n; i++) {
		if (ms_since(now, ctx->revert_ms[i]) <=
		    (uint64_t)ctx->cfg.flap_window_ms) {
			recent++;
		}
	}
	if (recent >= ctx->cfg.flap_alarm_n) {
		ctx->flap_alarm = true;
	}
}

/* Hand the mux back to the OCXO. @p failsafe distinguishes a guard/handoff
 * failure (lockout + flap accounting) from an operator-requested change. */
static void revert_to_ocxo(refsel_ctx_t *ctx, uint64_t now, bool failsafe,
			   refsel_out_t *out)
{
	out->transition = true;
	out->from = ctx->state;
	ctx->state = REFSEL_OCXO_ACTIVE;
	ctx->guards_have_since = false;
	ctx->tracked_target = REFSEL_OCXO_ACTIVE;
	emit_actions(ctx, REFSEL_OCXO_ACTIVE);

	if (failsafe) {
		note_revert(ctx, now);
		start_lockout(ctx, now);
	}
}

/* --------------------------------------------------------------------- API */

int refsel_step(refsel_ctx_t *ctx, const refsel_in_t *in, refsel_out_t *out)
{
	refsel_out_t o;
	bool extref_ok;
	bool rb_ok;
	refsel_state_t want;

	if (ctx == NULL || in == NULL || !ctx->initialised) {
		return -EINVAL;
	}

	memset(&o, 0, sizeof(o));
	ctx->n_steps = 0u;

	extref_ok = extref_in_band(ctx, in);
	rb_ok = in->rb_lock;

	if (ctx->lockout && in->mono_ms >= ctx->lockout_until_ms) {
		ctx->lockout = false;
	}

	/* What the operator is asking for. */
	switch (in->request) {
	case REFSEL_REQ_EXTREF:
		want = REFSEL_EXTREF_ACTIVE;
		break;
	case REFSEL_REQ_FORCE_OCXO:
		want = REFSEL_OCXO_ACTIVE;
		break;
	default:
		want = REFSEL_RB_ACTIVE;
		break;
	}

	/*
	 * 1. A handoff the glue could not complete outranks everything: the
	 *    system clock's health is not something to debounce.
	 */
	if (in->switch_failed) {
		ctx->switch_failed_latched = true;
		if (ctx->state != REFSEL_OCXO_ACTIVE) {
			revert_to_ocxo(ctx, in->mono_ms, true, &o);
		} else {
			/* Already on the OCXO; there is nothing to switch away
			 * from, but do not immediately retry either. */
			note_revert(ctx, in->mono_ms);
			start_lockout(ctx, in->mono_ms);
			ctx->guards_have_since = false;
		}
	} else if (ctx->state != REFSEL_OCXO_ACTIVE) {
		/*
		 * 2. Holding input B. §3.5: revert the same tick either guard
		 *    drops — no debounce on the fail-safe direction.
		 */
		if (!guards_for(ctx, ctx->state, extref_ok, rb_ok)) {
			revert_to_ocxo(ctx, in->mono_ms, true, &o);
		} else if (want != ctx->state) {
			/* Operator changed their mind. Input B is shared, so
			 * the mux position may be unchanged, but the guard set
			 * is not — go through the OCXO and re-qualify under
			 * the new request. Not a fault, so no lockout and no
			 * flap accounting. */
			revert_to_ocxo(ctx, in->mono_ms, false, &o);
		}
	} else if (want != REFSEL_OCXO_ACTIVE) {
		/*
		 * 3. On the OCXO and something is wanted. Both guards must hold
		 *    continuously for the hysteresis window, and no lockout may
		 *    be running.
		 */
		bool ok = guards_for(ctx, want, extref_ok, rb_ok);

		if (ctx->tracked_target != want) {
			ctx->tracked_target = want;
			ctx->guards_have_since = false;
		}
		if (!ok) {
			ctx->guards_have_since = false;
		} else if (!ctx->guards_have_since) {
			ctx->guards_have_since = true;
			ctx->guards_since_ms = in->mono_ms;
		}

		if (ok && ctx->guards_have_since &&
		    ms_since(in->mono_ms, ctx->guards_since_ms) >=
			    (uint64_t)ctx->cfg.hysteresis_ms) {
			if (!ctx->lockout) {
				o.transition = true;
				o.from = ctx->state;
				ctx->state = want;
				ctx->guards_have_since = false;
				emit_actions(ctx, want);
			}
		}
	} else {
		/* On the OCXO and pinned there. */
		ctx->guards_have_since = false;
		ctx->tracked_target = REFSEL_OCXO_ACTIVE;
	}

	/* ---- report ---- */
	o.state = ctx->state;
	if (extref_ok) {
		o.flags |= REFSEL_FLAG_EXTREF_OK;
	}
	if (rb_ok) {
		o.flags |= REFSEL_FLAG_RB_OK;
	}
	if (ctx->guards_have_since &&
	    ms_since(in->mono_ms, ctx->guards_since_ms) >=
		    (uint64_t)ctx->cfg.hysteresis_ms) {
		o.flags |= REFSEL_FLAG_GUARDS_STABLE;
	}
	if (ctx->lockout) {
		o.flags |= REFSEL_FLAG_LOCKOUT;
		o.lockout_remain_ms =
			(uint32_t)ms_since(ctx->lockout_until_ms, in->mono_ms);
	}
	if (ctx->flap_alarm) {
		o.flags |= REFSEL_FLAG_FLAP_ALARM;
	}
	if (ctx->switch_failed_latched) {
		o.flags |= REFSEL_FLAG_SWITCH_FAILED;
	}
	if (in->request == REFSEL_REQ_FORCE_OCXO) {
		o.flags |= REFSEL_FLAG_FORCED_OCXO;
	}
	o.revert_count = ctx->revert_total;

	if (out != NULL) {
		*out = o;
	}
	return 0;
}

int refsel_actions(const refsel_ctx_t *ctx, const refsel_step_t **steps,
		   size_t *n)
{
	if (ctx == NULL || steps == NULL || n == NULL || !ctx->initialised) {
		return -EINVAL;
	}
	*steps = ctx->steps;
	*n = ctx->n_steps;
	return 0;
}

int refsel_clear_flap(refsel_ctx_t *ctx)
{
	if (ctx == NULL || !ctx->initialised) {
		return -EINVAL;
	}
	ctx->flap_alarm = false;
	ctx->switch_failed_latched = false;
	ctx->revert_n = 0u;
	ctx->revert_head = 0u;
	ctx->revert_total = 0u;
	return 0;
}

refsel_state_t refsel_state(const refsel_ctx_t *ctx)
{
	if (ctx == NULL || !ctx->initialised) {
		return REFSEL_STATE__COUNT;
	}
	return ctx->state;
}

const char *refsel_state_name(refsel_state_t s)
{
	static const char *const names[REFSEL_STATE__COUNT] = {
		"ocxo", "rb", "extref",
	};

	if ((unsigned int)s >= (unsigned int)REFSEL_STATE__COUNT) {
		return "invalid";
	}
	return names[s];
}

const char *refsel_action_name(refsel_action_t a)
{
	static const char *const names[REFSEL_ACT__COUNT] = {
		"none",		"park_discipline", "bridge_to_hsi",
		"set_mux",	"wait_settle_ms",  "reselect_hse",
		"verify_pll",	"unpark_discipline", "done",
	};

	if ((unsigned int)a >= (unsigned int)REFSEL_ACT__COUNT) {
		return "invalid";
	}
	return names[a];
}
