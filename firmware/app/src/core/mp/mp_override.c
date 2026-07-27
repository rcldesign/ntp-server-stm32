/*
 * STS1000 "Meridian" — core/mp: override leases, guards, interlocks (FMT §5).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See mp_override.h for the contract and the dead-man timing argument.
 */

#include "mp/mp_override.h"

#include <errno.h>
#include <string.h>

/* ------------------------------------------------------------------ names */

static const char *const ev_names[MP_OVR_EV_COUNT] = {
	"grant", "release", "expire", "deadman", "veto", "verify-fail",
};

const char *mp_ovr_ev_name(uint8_t ev)
{
	return (ev < (uint8_t)MP_OVR_EV_COUNT) ? ev_names[ev] : "unknown";
}

static const char *const gc_names[MP_GC_COUNT] = {
	"ok",          "need-session", "stale",       "need-serial",
	"need-phrase", "armed",        "need-hold",   "arm-mismatch",
};

const char *mp_gc_name(uint8_t gc)
{
	return (gc < (uint8_t)MP_GC_COUNT) ? gc_names[gc] : "unknown";
}

/* ------------------------------------------------------------------ utils */

/** Wrap-safe elapsed time. */
static uint32_t since(uint32_t now, uint32_t then)
{
	return now - then;
}

/** Wrap-safe "has @p deadline passed?". */
static bool reached(uint32_t now, uint32_t deadline)
{
	return (int32_t)(now - deadline) >= 0;
}

static void copy_str(char *dst, size_t cap, const char *src)
{
	size_t n;

	if ((src == NULL) || (cap == 0U)) {
		if (cap != 0U) {
			dst[0] = '\0';
		}
		return;
	}
	n = strlen(src);
	if (n >= cap) {
		n = cap - 1U;
	}
	(void)memcpy(dst, src, n);
	dst[n] = '\0';
}

/**
 * Constant-time-ish string compare for confirmation strings.
 *
 * The device serial and the G3 phrase are not secrets — both are printed on the
 * chassis or in the manual — so this is not a timing-attack defence. It is here
 * so a length mismatch cannot be distinguished from a content mismatch in the
 * *reply*, which keeps the guard's error surface uniform.
 */
static bool confirm_eq(const char *expect, const char *got)
{
	size_t i;
	size_t n;
	uint8_t diff = 0U;

	if ((expect == NULL) || (got == NULL)) {
		return false;
	}
	if (expect[0] == '\0') {
		return false; /* unprovisioned: never matches */
	}
	n = strlen(expect);
	if (strlen(got) != n) {
		return false;
	}
	for (i = 0U; i < n; i++) {
		diff |= (uint8_t)((uint8_t)expect[i] ^ (uint8_t)got[i]);
	}
	return diff == 0U;
}

static void emit(mp_ovr_ctx_t *c, uint8_t ev, size_t obj, uint32_t sid,
		 const char *reason)
{
	if (c->evt != NULL) {
		c->evt(c->evt_user, ev, obj, sid, reason);
	}
}

/* =============================================================== interlocks */

int mp_ilk_eval(size_t obj, int32_t req, const mp_ilk_state_t *st,
		uint32_t now_ms, mp_ilk_res_t *out)
{
	const mp_obj_t *o = mp_obj_at(obj);
	uint32_t mask;

	if ((o == NULL) || (out == NULL)) {
		return -EINVAL;
	}

	(void)memset(out, 0, sizeof(*out));
	out->value = req;
	mask = o->ilk;

	if (mask == 0U) {
		return 0;
	}

	/*
	 * A NULL state means the glue could not produce a snapshot. Every
	 * interlock below has a positive precondition, so the fail-safe reading
	 * is "refuse", reported against the lowest declared bit.
	 */
	if (st == NULL) {
		uint32_t bit;

		for (bit = 1U; bit <= MP_ILK_ALL; bit <<= 1) {
			if ((mask & bit) != 0U) {
				out->failed = bit;
				return -EPERM;
			}
		}
		return -EPERM;
	}

	/* --- refusals first: a refused request must not be reported as clamped */

	if ((mask & MP_ILK_RB_OV) != 0U) {
		/* While the autonomous 26 V latch is set, only de-assertion is
		 * permitted: turning the rubidium chain *off* is always safe. */
		if (st->rb_ov_latched && (req != 0)) {
			out->failed = MP_ILK_RB_OV;
			return -EPERM;
		}
	}

	if ((mask & MP_ILK_RB_WARM) != 0U) {
		if ((req != 0) && !(st->ocxo_warm && st->supercaps_ok)) {
			out->failed = MP_ILK_RB_WARM;
			return -EPERM;
		}
	}

	if ((mask & MP_ILK_RELAY_OK) != 0U) {
		/* K2 is normally energized; forcing it energized over a live
		 * fault would annunciate healthy service that does not exist. */
		if ((req != 0) && st->relay_blocked) {
			out->failed = MP_ILK_RELAY_OK;
			return -EPERM;
		}
	}

	if ((mask & MP_ILK_DISP_OFF) != 0U) {
		if ((req != 0) && !st->disp_on &&
		    (since(now_ms, st->disp_changed_ms) < MP_DISP_MIN_OFF_MS)) {
			out->failed = MP_ILK_DISP_OFF;
			return -EPERM;
		}
	}

	if ((mask & MP_ILK_WDT_LIVE) != 0U) {
		if ((req != 0) && !st->liveness_ok) {
			out->failed = MP_ILK_WDT_LIVE;
			return -EPERM;
		}
	}

	if ((mask & MP_ILK_MUX_GUARD) != 0U) {
		/* Selecting the OCXO (0) is the fail-safe direction and always
		 * allowed; selecting input B needs both guards (spec §3.5). */
		if ((req != 0) && !(st->extref_ok && st->rb_lock)) {
			out->failed = MP_ILK_MUX_GUARD;
			return -EPERM;
		}
	}

	if ((mask & MP_ILK_DAC_PARK) != 0U) {
		if (!st->disc_parked) {
			out->failed = MP_ILK_DAC_PARK;
			return -EPERM;
		}
	}

	/* --- clamps ------------------------------------------------------- */

	if ((mask & MP_ILK_RB_VMAX) != 0U) {
		if (o->kind == (uint8_t)MP_KIND_CODE) {
			int32_t lo = (int32_t)st->rb_code_min;
			int32_t hi = (int32_t)st->rb_code_max;

			if (hi < lo) {
				/* An inverted window means the glue could not
				 * compute one; refuse rather than guess. */
				out->failed = MP_ILK_RB_VMAX;
				return -EPERM;
			}
			if (out->value > hi) {
				out->value = hi;
				out->clamped = true;
			}
			if (out->value < lo) {
				out->value = lo;
				out->clamped = true;
			}
		} else {
			int32_t hi = (int32_t)st->rb_vmax_mv;

			if (hi <= 0) {
				out->failed = MP_ILK_RB_VMAX;
				return -EPERM;
			}
			if (out->value > hi) {
				out->value = hi;
				out->clamped = true;
			}
			if (out->value < o->min) {
				out->value = o->min;
				out->clamped = true;
			}
		}
	}

	if ((mask & MP_ILK_FAN_FLOOR) != 0U) {
		if (out->value < (int32_t)st->fan_floor_pct) {
			out->value = (int32_t)st->fan_floor_pct;
			out->clamped = true;
		}
	}

	/* --- consequences ------------------------------------------------- */

	if ((mask & MP_ILK_TUNNEL) != 0U) {
		out->tunnel = (req != 0);
	}
	if ((mask & MP_ILK_RB_VERIFY) != 0U) {
		out->verify = true;
	}

	return 0;
}

/* ================================================================ lifecycle */

int mp_ovr_init(mp_ovr_ctx_t *c, mp_ovr_apply_fn apply, void *apply_user,
		mp_ovr_evt_fn evt, void *evt_user, const char *serial)
{
	if ((c == NULL) || (apply == NULL)) {
		return -EINVAL;
	}

	(void)memset(c, 0, sizeof(*c));
	c->apply = apply;
	c->apply_user = apply_user;
	c->evt = evt;
	c->evt_user = evt_user;
	copy_str(c->serial, sizeof(c->serial), serial);
	copy_str(c->phrase, sizeof(c->phrase), MP_G3_PHRASE_DEFAULT);
	c->next_sid = 1U;
	return 0;
}

int mp_ovr_set_phrase(mp_ovr_ctx_t *c, const char *phrase)
{
	if (c == NULL) {
		return -EINVAL;
	}
	if ((phrase == NULL) || (phrase[0] == '\0')) {
		copy_str(c->phrase, sizeof(c->phrase), MP_G3_PHRASE_DEFAULT);
	} else {
		copy_str(c->phrase, sizeof(c->phrase), phrase);
	}
	return 0;
}

/* ---------------------------------------------------------- lease plumbing */

static mp_lease_t *lease_of(mp_ovr_ctx_t *c, size_t obj)
{
	size_t i;

	for (i = 0U; i < MP_LEASE_MAX; i++) {
		if (c->lease[i].obj1 == (uint16_t)(obj + 1U)) {
			return &c->lease[i];
		}
	}
	return NULL;
}

static mp_lease_t *lease_free(mp_ovr_ctx_t *c)
{
	size_t i;

	for (i = 0U; i < MP_LEASE_MAX; i++) {
		if (c->lease[i].obj1 == 0U) {
			return &c->lease[i];
		}
	}
	return NULL;
}

/** Release one lease: call the glue with NULL, clear the slot, emit @p ev. */
static void lease_drop(mp_ovr_ctx_t *c, mp_lease_t *l, uint8_t ev,
		       const char *reason)
{
	size_t obj = (size_t)(l->obj1 - 1U);
	uint32_t sid = l->sid;
	int rc;

	l->obj1 = 0U;
	l->verify_at_ms = 0U;

	rc = c->apply(c->apply_user, obj, NULL);
	if (rc < 0) {
		c->release_errors++;
	}
	emit(c, ev, obj, sid, reason);
}

int mp_ovr_revert_all(mp_ovr_ctx_t *c, uint8_t ev, const char *reason,
		      uint32_t now_ms)
{
	size_t i;
	int n = 0;

	(void)now_ms;

	if (c == NULL) {
		return -EINVAL;
	}
	for (i = 0U; i < MP_LEASE_MAX; i++) {
		if (c->lease[i].obj1 != 0U) {
			lease_drop(c, &c->lease[i], ev, reason);
			n++;
		}
	}
	return n;
}

/* ------------------------------------------------------------------ session */

int mp_ovr_set_link(mp_ovr_ctx_t *c, bool up, uint32_t now_ms)
{
	if (c == NULL) {
		return -EINVAL;
	}
	if (c->link_up == up) {
		return 0;
	}
	c->link_up = up;
	if (!up) {
		/*
		 * §5.3: the link *is* the dead-man's outermost condition. A loss
		 * is unambiguous, so revert here rather than waiting for a tick.
		 */
		int n = mp_ovr_revert_all(c, (uint8_t)MP_OVR_EV_DEADMAN,
					  "link down", now_ms);

		if (n > 0) {
			c->deadman_reverts++;
		}
		(void)memset(&c->sess, 0, sizeof(c->sess));
	}
	return 0;
}

int mp_ovr_session_open(mp_ovr_ctx_t *c, uint32_t ttl_ms, uint32_t now_ms,
			uint32_t *out_sid)
{
	if ((c == NULL) || (out_sid == NULL)) {
		return -EINVAL;
	}
	if (!c->link_up) {
		return -ENOLINK;
	}

	if (c->sess.id != 0U) {
		/* A new tool taking over must not inherit the old one's
		 * commanded state. */
		(void)mp_ovr_revert_all(c, (uint8_t)MP_OVR_EV_RELEASE,
					"session replaced", now_ms);
	}

	(void)memset(&c->sess, 0, sizeof(c->sess));
	if (ttl_ms == 0U) {
		ttl_ms = MP_KEEPALIVE_TTL_MS;
	}
	if (ttl_ms > MP_KEEPALIVE_TTL_MS) {
		ttl_ms = MP_KEEPALIVE_TTL_MS;
	}

	c->sess.id = c->next_sid;
	c->next_sid++;
	if (c->next_sid == 0U) {
		c->next_sid = 1U; /* never hand out 0 */
	}
	c->sess.opened_ms = now_ms;
	c->sess.keepalive_ms = now_ms;
	c->sess.ttl_ms = ttl_ms;

	*out_sid = c->sess.id;
	return 0;
}

int mp_ovr_keepalive(mp_ovr_ctx_t *c, uint32_t sid, uint32_t now_ms)
{
	if (c == NULL) {
		return -EINVAL;
	}
	if ((sid == 0U) || (c->sess.id != sid)) {
		return -ENOENT;
	}
	/*
	 * A keepalive arriving after the session has already gone stale does not
	 * resurrect it: the overrides are already gone, and pretending otherwise
	 * would let a tool that froze for ten seconds carry on as if it had not.
	 */
	if (since(now_ms, c->sess.keepalive_ms) > c->sess.ttl_ms) {
		return -ENOENT;
	}
	c->sess.keepalive_ms = now_ms;
	return 0;
}

int mp_ovr_session_close(mp_ovr_ctx_t *c, uint32_t sid, uint32_t now_ms)
{
	if (c == NULL) {
		return -EINVAL;
	}
	if ((sid == 0U) || (c->sess.id != sid)) {
		return -ENOENT;
	}
	(void)mp_ovr_revert_all(c, (uint8_t)MP_OVR_EV_RELEASE,
				"session closed", now_ms);
	(void)memset(&c->sess, 0, sizeof(c->sess));
	return 0;
}

bool mp_ovr_session_valid(const mp_ovr_ctx_t *c, uint32_t sid, uint32_t now_ms)
{
	if ((c == NULL) || (sid == 0U) || (c->sess.id != sid)) {
		return false;
	}
	if (!c->link_up) {
		return false;
	}
	return since(now_ms, c->sess.keepalive_ms) <= c->sess.ttl_ms;
}

uint32_t mp_ovr_session_remaining(const mp_ovr_ctx_t *c, uint32_t now_ms)
{
	uint32_t age;

	if ((c == NULL) || (c->sess.id == 0U)) {
		return 0U;
	}
	age = since(now_ms, c->sess.keepalive_ms);
	if (age >= c->sess.ttl_ms) {
		return 0U;
	}
	return c->sess.ttl_ms - age;
}

/* ------------------------------------------------------------------ guards */

/**
 * Cheap arming nonce.
 *
 * Not a secret: the nonce exists to bind the second phase of a G3 action to the
 * first, so that "confirm" cannot be replayed against a *different* action. An
 * attacker who can issue the first phase can read the reply, so unpredictability
 * buys nothing; what matters is that it changes every time.
 */
static uint32_t make_nonce(mp_ovr_ctx_t *c, uint32_t now_ms, uint16_t obj1,
			   uint8_t tag)
{
	uint32_t n = now_ms;

	n = (n * 2654435761U) ^ (c->sess.id * 40503U);
	n ^= ((uint32_t)obj1 << 16) ^ (uint32_t)tag ^ (c->grants * 2246822519U);
	if (n == 0U) {
		n = 0xA5A5A5A5U;
	}
	return n;
}

int mp_ovr_guard(mp_ovr_ctx_t *c, uint8_t guard, uint32_t sid, uint16_t obj1,
		 uint8_t tag, const char *confirm, uint32_t nonce,
		 uint32_t now_ms, mp_gc_arm_t *arm)
{
	if (c == NULL) {
		return -EINVAL;
	}

	if (guard == (uint8_t)MP_GUARD_G0) {
		return (int)MP_GC_OK;
	}

	/* G1: an open session with a fresh keepalive. */
	if ((sid == 0U) || (c->sess.id != sid) || !c->link_up) {
		c->refusals++;
		return (int)MP_GC_NEED_SESSION;
	}
	if (since(now_ms, c->sess.keepalive_ms) > c->sess.ttl_ms) {
		c->refusals++;
		return (int)MP_GC_STALE;
	}

	if (guard == (uint8_t)MP_GUARD_G1) {
		c->sess.keepalive_ms = now_ms;
		return (int)MP_GC_OK;
	}

	/* G2: typed device serial. Interlocks are checked by the caller at
	 * apply time, where the value is known. */
	if (!confirm_eq(c->serial, confirm)) {
		c->refusals++;
		return (int)MP_GC_NEED_SERIAL;
	}
	c->sess.serial_ok = true;

	if (guard == (uint8_t)MP_GUARD_G2) {
		c->sess.keepalive_ms = now_ms;
		return (int)MP_GC_OK;
	}

	/* G3: typed phrase + hold. Two phases. */
	if (nonce == 0U) {
		/* Phase 1: arm. The phrase is required to arm, so an accidental
		 * request never reaches the hold at all. */
		if (!confirm_eq(c->phrase, confirm)) {
			c->refusals++;
			return (int)MP_GC_NEED_PHRASE;
		}
		c->sess.arm_nonce = make_nonce(c, now_ms, obj1, tag);
		c->sess.arm_obj1 = obj1;
		c->sess.arm_tag = tag;
		c->sess.arm_ready_ms = now_ms + MP_G3_HOLD_MS;
		c->sess.arm_expire_ms = now_ms + MP_G3_WINDOW_MS;
		c->sess.keepalive_ms = now_ms;
		if (arm != NULL) {
			arm->nonce = c->sess.arm_nonce;
			arm->hold_ms = MP_G3_HOLD_MS;
			arm->expires_ms = MP_G3_WINDOW_MS;
		}
		return (int)MP_GC_ARMED;
	}

	/* Phase 2: complete. */
	if ((c->sess.arm_nonce == 0U) || (c->sess.arm_nonce != nonce) ||
	    (c->sess.arm_obj1 != obj1) ||
	    ((obj1 == 0U) && (c->sess.arm_tag != tag))) {
		c->refusals++;
		return (int)MP_GC_ARM_MISMATCH;
	}
	if (reached(now_ms, c->sess.arm_expire_ms)) {
		mp_ovr_disarm(c);
		c->refusals++;
		return (int)MP_GC_ARM_MISMATCH;
	}
	if (!reached(now_ms, c->sess.arm_ready_ms)) {
		c->refusals++;
		return (int)MP_GC_NEED_HOLD;
	}

	mp_ovr_disarm(c);
	c->sess.keepalive_ms = now_ms;
	return (int)MP_GC_OK;
}

void mp_ovr_disarm(mp_ovr_ctx_t *c)
{
	if (c == NULL) {
		return;
	}
	c->sess.arm_nonce = 0U;
	c->sess.arm_obj1 = 0U;
	c->sess.arm_tag = 0U;
	c->sess.arm_ready_ms = 0U;
	c->sess.arm_expire_ms = 0U;
}

/* ------------------------------------------------------------------ leases */

/** Value-envelope check against the manifest. */
static int check_envelope(const mp_obj_t *o, int32_t v)
{
	switch (o->kind) {
	case MP_KIND_BOOL:
		return ((v == 0) || (v == 1)) ? 0 : -ERANGE;
	case MP_KIND_ENUM:
	case MP_KIND_PCT:
	case MP_KIND_MV:
	case MP_KIND_CODE:
	case MP_KIND_PULSE:
	case MP_KIND_SCALAR:
		return ((v >= o->min) && (v <= o->max)) ? 0 : -ERANGE;
	default:
		/* REAL/RAIL/BITS/TEXT are not settable through this path. */
		return -ENOTSUP;
	}
}

int mp_ovr_grant(mp_ovr_ctx_t *c, size_t obj, int32_t value, uint32_t ttl_ms,
		 uint32_t sid, const mp_ilk_state_t *st, uint32_t now_ms,
		 mp_ilk_res_t *res)
{
	const mp_obj_t *o = mp_obj_at(obj);
	mp_ilk_res_t local;
	mp_lease_t *l;
	int rc;

	if ((c == NULL) || (o == NULL)) {
		return -EINVAL;
	}
	if (res == NULL) {
		res = &local;
	}
	if ((o->flags & MP_OF_OVERRIDE) == 0U) {
		return -ENOTSUP;
	}
	if (!mp_ovr_session_valid(c, sid, now_ms)) {
		return -ENOLINK;
	}

	rc = check_envelope(o, value);
	if (rc != 0) {
		(void)memset(res, 0, sizeof(*res));
		res->value = value;
		return rc;
	}

	rc = mp_ilk_eval(obj, value, st, now_ms, res);
	if (rc != 0) {
		c->refusals++;
		return rc;
	}

	/* A clamp must still land inside the published envelope. */
	rc = check_envelope(o, res->value);
	if (rc != 0) {
		return rc;
	}

	l = lease_of(c, obj);
	if (l == NULL) {
		l = lease_free(c);
		if (l == NULL) {
			return -ENOSPC;
		}
	}

	rc = c->apply(c->apply_user, obj, &res->value);
	if (rc < 0) {
		/* Firmware refused the value outright: that is a veto, and the
		 * slot must not be left half-claimed. */
		if (l->obj1 == (uint16_t)(obj + 1U)) {
			l->obj1 = 0U;
			l->verify_at_ms = 0U;
		}
		c->vetoes++;
		emit(c, (uint8_t)MP_OVR_EV_VETO, obj, sid, "apply refused");
		return -EACCES;
	}

	if (ttl_ms == 0U) {
		ttl_ms = MP_LEASE_TTL_DEFAULT_MS;
	}
	if (ttl_ms > MP_LEASE_TTL_MAX_MS) {
		ttl_ms = MP_LEASE_TTL_MAX_MS;
	}

	l->obj1 = (uint16_t)(obj + 1U);
	l->sid = sid;
	l->value = res->value;
	l->granted_ms = now_ms;
	l->deadline_ms = now_ms + ttl_ms;

	if (res->verify) {
		l->verify_at_ms = now_ms + MP_RB_VERIFY_DELAY_MS;
		if (l->verify_at_ms == 0U) {
			l->verify_at_ms = 1U; /* 0 means "nothing pending" */
		}
		l->verify_expect_mv = (o->kind == (uint8_t)MP_KIND_MV)
					      ? res->value
					      : ((st != NULL)
							 ? st->rb_expected_mv
							 : 0);
	} else {
		l->verify_at_ms = 0U;
	}

	c->grants++;
	emit(c, (uint8_t)MP_OVR_EV_GRANT, obj, sid,
	     res->clamped ? "clamped by interlock" : NULL);
	return 0;
}

int mp_ovr_release(mp_ovr_ctx_t *c, size_t obj, uint32_t sid, uint32_t now_ms)
{
	mp_lease_t *l;

	(void)now_ms;

	if ((c == NULL) || (mp_obj_at(obj) == NULL)) {
		return -EINVAL;
	}
	l = lease_of(c, obj);
	if (l == NULL) {
		return -ENOENT;
	}
	if (l->sid != sid) {
		return -EACCES;
	}
	lease_drop(c, l, (uint8_t)MP_OVR_EV_RELEASE, NULL);
	c->releases++;
	return 0;
}

int mp_ovr_veto(mp_ovr_ctx_t *c, size_t obj, const char *reason,
		uint32_t now_ms)
{
	mp_lease_t *l;

	(void)now_ms;

	if ((c == NULL) || (mp_obj_at(obj) == NULL)) {
		return -EINVAL;
	}
	l = lease_of(c, obj);
	if (l == NULL) {
		return -ENOENT;
	}
	lease_drop(c, l, (uint8_t)MP_OVR_EV_VETO,
		   (reason != NULL) ? reason : "firmware veto");
	c->vetoes++;
	return 0;
}

const mp_lease_t *mp_ovr_lease(const mp_ovr_ctx_t *c, size_t obj)
{
	size_t i;

	if (c == NULL) {
		return NULL;
	}
	for (i = 0U; i < MP_LEASE_MAX; i++) {
		if (c->lease[i].obj1 == (uint16_t)(obj + 1U)) {
			return &c->lease[i];
		}
	}
	return NULL;
}

size_t mp_ovr_active(const mp_ovr_ctx_t *c)
{
	size_t i;
	size_t n = 0U;

	if (c == NULL) {
		return 0U;
	}
	for (i = 0U; i < MP_LEASE_MAX; i++) {
		if (c->lease[i].obj1 != 0U) {
			n++;
		}
	}
	return n;
}

/* -------------------------------------------------------------------- tick */

/** True when the dead-man's conditions all hold. */
static bool deadman_ok(const mp_ovr_ctx_t *c, uint32_t now_ms)
{
	if (!c->link_up) {
		return false;
	}
	if (c->sess.id == 0U) {
		return false;
	}
	return since(now_ms, c->sess.keepalive_ms) <= c->sess.ttl_ms;
}

/** Judge one pending VCC_RB read-back. Returns true when the lease was dropped. */
static bool verify_one(mp_ovr_ctx_t *c, mp_lease_t *l, const mp_ilk_state_t *st,
		       uint32_t now_ms)
{
	int32_t expect = l->verify_expect_mv;
	int32_t tol;
	int32_t delta;

	if (l->verify_at_ms == 0U) {
		return false;
	}
	if (!reached(now_ms, l->verify_at_ms)) {
		return false;
	}

	/* "Cannot verify" and "verified wrong" get the same answer: the whole
	 * point of the read-back is that the rail is not trusted without it. */
	if ((st == NULL) || !st->vcc_rb_valid) {
		c->verify_failures++;
		lease_drop(c, l, (uint8_t)MP_OVR_EV_VERIFY_FAIL,
			   "VCC_RB read-back unavailable");
		return true;
	}

	if (expect <= 0) {
		expect = st->rb_expected_mv;
	}
	tol = (expect * (int32_t)MP_RB_VERIFY_TOL_PCT) / 100;
	if (tol < 100) {
		tol = 100; /* never tighter than 100 mV on a 4.5-24 V rail */
	}
	delta = st->vcc_rb_mv - expect;
	if (delta < 0) {
		delta = -delta;
	}

	if (delta > tol) {
		c->verify_failures++;
		lease_drop(c, l, (uint8_t)MP_OVR_EV_VERIFY_FAIL,
			   "VCC_RB out of window");
		return true;
	}

	l->verify_at_ms = 0U;
	return false;
}

int mp_ovr_tick(mp_ovr_ctx_t *c, const mp_ilk_state_t *st, uint32_t now_ms)
{
	size_t i;
	int n = 0;

	if (c == NULL) {
		return -EINVAL;
	}

	if (c->ticked && (since(now_ms, c->last_tick_ms) > MP_TICK_MAX_MS)) {
		c->late_ticks++;
	}
	c->last_tick_ms = now_ms;
	c->ticked = true;

	/* Dead-man first: it subsumes every per-lease reason. */
	if (!deadman_ok(c, now_ms)) {
		if (mp_ovr_active(c) > 0U) {
			n = mp_ovr_revert_all(c, (uint8_t)MP_OVR_EV_DEADMAN,
					      "keepalive or link lost", now_ms);
			c->deadman_reverts++;
		}
		if ((c->sess.id != 0U) &&
		    (since(now_ms, c->sess.keepalive_ms) > c->sess.ttl_ms)) {
			(void)memset(&c->sess, 0, sizeof(c->sess));
		}
		return n;
	}

	/* Lease expiry, then the read-back verifications. */
	for (i = 0U; i < MP_LEASE_MAX; i++) {
		mp_lease_t *l = &c->lease[i];

		if (l->obj1 == 0U) {
			continue;
		}
		if (reached(now_ms, l->deadline_ms)) {
			lease_drop(c, l, (uint8_t)MP_OVR_EV_EXPIRE,
				   "lease expired");
			c->expiries++;
			n++;
			continue;
		}
		if (verify_one(c, l, st, now_ms)) {
			n++;
		}
	}

	/* An armed but unfulfilled G3 action must not sit there forever. */
	if ((c->sess.arm_nonce != 0U) &&
	    reached(now_ms, c->sess.arm_expire_ms)) {
		mp_ovr_disarm(c);
	}

	return n;
}
