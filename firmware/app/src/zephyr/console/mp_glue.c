/*
 * STS1000 "Meridian" — Maintenance Protocol glue (console area).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See mp_glue.h for the mode-entry design. This file is the wire, the shell
 * hand-off, and the provider callbacks.
 *
 * ---------------------------------------------------------------------------
 * TODO — two hooks this area cannot install by itself
 *
 * 1. Autobaud entry magic. sts_mp_shell_tap() implements it, but something has
 *    to feed it every console byte while the shell owns the port. That means one
 *    line in the shell's RX path (src/zephyr/console/sts_shell.c) or a
 *    cross-area accessor. Until then MP mode is entered with `mp enter`, which
 *    covers every case except a host that cannot type.
 *
 * 2. Panel mirror. sts_mp_mirror_publish() is ready; the UI area has to call it
 *    after each ui_render(). It cannot include this private header
 *    (ARCHITECTURE.md §2 forbids cross-area private includes), so the
 *    declaration belongs in src/zephyr/sts_app.h — one prototype plus a __weak
 *    no-op, matching how sts_ui_post_input() already crosses the same seam.
 *
 * Neither gap is silent: `mirror.get` answers MP_E_NOTSUP and `mp status`
 * reports the mirror as unwired.
 *
 * A third, larger gap is the object write path. Most control objects live on
 * GPIO/PWM/DAC that the *platform* area owns, and sts_app.h exposes only
 * sts_panel_led_set(). Every other write therefore answers MP_E_NOTSUP today;
 * the manifest still publishes the object, its guard and its interlocks, so the
 * tool discovers the surface and the safety model is already enforced. Wiring
 * the rest is a platform-area change: one setter that takes a manifest object
 * index, or a small table of per-object accessors in sts_app.h.
 * ---------------------------------------------------------------------------
 */

#include <zephyr/kernel.h>

#ifdef CONFIG_STS1000_CONSOLE

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include "console/mp_glue.h"
#include "console/sts_console.h"
#include "fault/fault.h"
#include "ina228/ina228.h"
/*
 * net/sts_aaa.h is included across areas on that header's own invitation: it
 * exists so "the web server, the MCP console channel and the shell" share one
 * credential store, one role map and one lockout table instead of growing three.
 * This adds a console -> net edge to ARCHITECTURE.md §4.
 */
#include "net/sts_aaa.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_mp, CONFIG_STS1000_LOG_LEVEL);

/* --------------------------------------------------------------- tunables */

/*
 * Tunables. Each has an #if defined() fallback so the area is correct whether or
 * not app/Kconfig sources src/zephyr/console/Kconfig.mp — the same convention
 * the rest of the console area uses (see console/Kconfig).
 */
#if defined(CONFIG_STS1000_MP_SCRATCH)
#define MP_SCRATCH_BYTES CONFIG_STS1000_MP_SCRATCH
#else
#define MP_SCRATCH_BYTES 4096
#endif

#if defined(CONFIG_STS1000_MP_FW_VERSION)
#define MP_FW_VERSION CONFIG_STS1000_MP_FW_VERSION
#else
#define MP_FW_VERSION "0.1.0-dev"
#endif

#if defined(CONFIG_STS1000_MP_REASM)
#define MP_REASM_BYTES CONFIG_STS1000_MP_REASM
#else
#define MP_REASM_BYTES 3072
#endif

/* The as-built panel is a 480x320 ST7796 with an 8x16 font: 60x20 cells. */
#define MP_MIRROR_ROWS 20
#define MP_MIRROR_COLS 60
#define MP_MIRROR_CELLS (MP_MIRROR_ROWS * MP_MIRROR_COLS)

BUILD_ASSERT(MP_SCRATCH_BYTES >= (int)MP_SCRATCH_MIN,
	     "CONFIG_STS1000_MP_SCRATCH cannot hold one mirror keyframe");

/*
 * core/mp carries its own MP_ROLE_* constants because its dependency set does
 * not include core/auth (ARCHITECTURE.md §4). This file is the seam where both
 * are visible, so it is where the numeric identity is proved. If auth_role_t is
 * ever reordered, the build stops here instead of silently promoting a viewer.
 */
BUILD_ASSERT((int)AUTH_ROLE_NONE == (int)MP_ROLE_NONE, "role enum drift");
BUILD_ASSERT((int)AUTH_ROLE_VIEWER == (int)MP_ROLE_VIEWER, "role enum drift");
BUILD_ASSERT((int)AUTH_ROLE_OPERATOR == (int)MP_ROLE_OPERATOR, "role enum drift");
BUILD_ASSERT((int)AUTH_ROLE_ADMIN == (int)MP_ROLE_ADMIN, "role enum drift");
BUILD_ASSERT(MP_USER_MAX >= AUTH_USER_MAX,
	     "a session cannot record the longest user AAA accepts");
BUILD_ASSERT(MP_SECRET_MAX >= AUTH_SECRET_MAX,
	     "the control plane would truncate a credential AAA would accept");

/* ------------------------------------------------------------------ state */

static const struct device *const mp_uart =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));

static mp_ctx_t mp;
static bool mp_started;

static uint8_t mp_scratch[MP_SCRATCH_BYTES];
static uint8_t mp_reasm_buf[2][MP_REASM_BYTES / 2];
static mp_reasm_t mp_reasm[2];

static char mirror_prev_ch[MP_MIRROR_CELLS];
static uint8_t mirror_prev_attr[MP_MIRROR_CELLS];

/* The published panel frame, double-buffered under a short mutex. */
static K_MUTEX_DEFINE(mirror_lock);
static mp_mirror_in_t mirror_frame;
static char mirror_ch[MP_MIRROR_CELLS];
static uint8_t mirror_attr[MP_MIRROR_CELLS];
static ui_hint_t mirror_hints[UI_SURF_MAX_HINTS];
static bool mirror_valid;

static char mp_serial[MP_CONFIRM_MAX];
static const struct shell *mp_shell;

/* ------------------------------------------------------------------ ports */

static uint32_t mono_ms(void *user)
{
	ARG_UNUSED(user);
	return (uint32_t)k_uptime_get_32();
}

/**
 * Transmit one framed message.
 *
 * Polled output: MP owns the port exclusively while it is active (the shell is
 * bypassed), the frames are small, and a poll loop cannot desynchronise the COBS
 * stream the way a partially-accepted ring write could.
 */
static int mp_tx(void *user, const uint8_t *wire, size_t len)
{
	size_t i;

	ARG_UNUSED(user);
	if (!device_is_ready(mp_uart)) {
		return -ENODEV;
	}
	for (i = 0U; i < len; i++) {
		uart_poll_out(mp_uart, wire[i]);
	}
	return 0;
}

/* ------------------------------------------------------------- providers */

/** Map a health snapshot's INA array onto the record's rail array. */
static void fill_rails(mp_health_t *h, const sts_health_t *s)
{
	size_t i;

	for (i = 0U; i < MP_RAIL_COUNT; i++) {
		if (i >= s->ina_count) {
			break;
		}
		h->rail[i].bus_mv = s->ina[i].bus_mv;
		h->rail[i].current_ua = s->ina[i].current_ua;
		h->rail[i].power_uw = s->ina[i].power_uw / 1000U;
		h->rail[i].diag = s->ina[i].diag_alrt;
		h->rail[i].valid = s->ina[i].valid;
	}
}

static void fill_health(mp_health_t *h, const sts_health_t *s)
{
	memset(h, 0, sizeof(*h));
	fill_rails(h, s);
	h->tmp_osc_mc = s->tmp_osc_mc;
	h->tmp_osc_valid = s->tmp_osc_valid;
	h->tmp_amb_mc = s->tmp_amb_mc;
	h->tmp_amb_valid = s->tmp_amb_valid;
	h->die_mc = s->die_mc;
	h->die_valid = s->die_valid;
	h->humidity_mpct = s->humidity_mpct;
	h->humidity_valid = s->humidity_valid;
	h->fan_rpm = s->fan_rpm;
	h->fan_duty_pct = s->fan_duty_pct;
	h->poe_class = s->poe_class;
	h->poe_draw_mw = s->poe_draw_mw;
	h->poe_budget_mw = s->poe_budget_mw;
	h->bkp_stm_pg = s->bkp_stm_pg;
	h->bkp_gps_pg = s->bkp_gps_pg;
}

static int prov_telem(void *user, mp_telem_t *out)
{
	sts_health_t hs;

	ARG_UNUSED(user);
	memset(out, 0, sizeof(*out));

	if (sts_quality_snapshot(&out->q) != 0) {
		quality_block_init(&out->q);
	}
	if (sts_health_snapshot(&hs) == 0) {
		fill_health(&out->h, &hs);
	}
	out->alarms = sts_alarms_active();
	out->uptime_s = (uint32_t)(k_uptime_get() / 1000);
	(void)sts_time_tai_ns(&out->tai_ns);
	out->time_fallback = sts_time_is_fallback();

	/*
	 * The remaining fields (pwrseq stage, gnssmgr state, survey progress,
	 * RB_LOCK, EXTREF_MON) live in the platform area behind no accessor. They
	 * stay zero rather than being guessed; see the TODO block above.
	 */
	out->rb_lock = (out->q.active_ref == (uint8_t)QUALITY_REF_RB);
	out->extref_ok = (out->q.active_ref == (uint8_t)QUALITY_REF_EXTREF);
	return 0;
}

static int prov_pps(void *user, mp_pps_t *out)
{
	quality_block_t q;

	ARG_UNUSED(user);
	memset(out, 0, sizeof(*out));
	if (sts_quality_snapshot(&q) != 0) {
		return -EIO;
	}

	/*
	 * What the quality block carries. The per-capture raw phases and the
	 * pre-correction residual live in `disc`, which publishes only the
	 * corrected result; pa0_ns/pc6_ns/qerr therefore stay zero and
	 * pc6_valid false until the timing glue offers a capture accessor.
	 */
	out->residual_corr_ns = q.last_pps_off_ns;
	out->vc_cmd_mv = q.vc_cmd_mv;
	out->vc_sense_mv = q.vc_sense_mv;
	out->dac_code = q.dac_code;
	out->loop_state = q.lock_state;
	out->active_ref = q.active_ref;
	out->ref_flags = q.flags;
	out->accepted = ((q.flags & (QUALITY_FLAG_PPS_REJECT |
				     QUALITY_FLAG_NO_PPS)) == 0U);
	return 0;
}

/**
 * Interlock state.
 *
 * Fail-safe by construction: anything this area cannot observe is left in the
 * state that makes the interlock *refuse*. A missing accessor must never read as
 * "permission granted".
 */
static int prov_ilk(void *user, mp_ilk_state_t *out)
{
	sts_health_t hs;
	quality_block_t q;
	uint64_t alarms = sts_alarms_active();
	uint64_t vmax = 15000U;

	ARG_UNUSED(user);
	memset(out, 0, sizeof(*out));

	if (sts_quality_snapshot(&q) == 0) {
		out->ocxo_warm = ((q.flags & QUALITY_FLAG_OCXO_WARM) != 0U);
		out->disc_parked = ((q.flags & QUALITY_FLAG_PARKED) != 0U);
		out->rb_lock = (q.active_ref == (uint8_t)QUALITY_REF_RB);
		out->extref_ok = (q.active_ref == (uint8_t)QUALITY_REF_EXTREF);
	}

	if (sts_health_snapshot(&hs) == 0) {
		/* Both supercap rails good is the §5.5 precondition for the
		 * rubidium chain, alongside a warm oven. */
		out->supercaps_ok = hs.bkp_stm_pg && hs.bkp_gps_pg;
		if (hs.ina_count > (size_t)INA228_RAIL_VCC_RB) {
			out->vcc_rb_mv = hs.ina[INA228_RAIL_VCC_RB].bus_mv;
			out->vcc_rb_valid = hs.ina[INA228_RAIL_VCC_RB].valid;
		}
		out->fan_floor_pct = (uint8_t)MIN(hs.fan_duty_pct, 100U);
	}

	/* The configured ceiling, from cfg — this is what bounds every VCC_RB
	 * request (the manifest publishes the wider electrical range). */
	if ((sts_cfg() != NULL) &&
	    (cfg_get_u64(sts_cfg(), 0x0703U, &vmax) == 0)) {
		out->rb_vmax_mv = (uint16_t)vmax;
	} else {
		out->rb_vmax_mv = 0U; /* unknown -> rb.vmax refuses */
	}
	out->rb_expected_mv = (int32_t)out->rb_vmax_mv;

	/*
	 * The digipot code window would come from pwrseq's own VCC_RB transfer
	 * function. Without an accessor the window is left inverted
	 * (min > max), which mp_ilk_eval() treats as "cannot compute" and
	 * refuses — the correct answer for a raw wiper write.
	 */
	out->rb_code_min = 1U;
	out->rb_code_max = 0U;

	out->rb_ov_latched = (alarms & FAULT_ALARM_BIT(FAULT_ALARM_RB_OV)) != 0U;
	out->relay_blocked = (alarms & FAULT_RELAY_DISQUALIFY_DEFAULT) != 0U;

	/* DISP_EN and the supervisor's liveness gate are platform state. Leaving
	 * disp_on false with a fresh change stamp makes the off-time interlock
	 * hold, and liveness_ok false makes the WDT interlock refuse. */
	out->disp_on = false;
	out->disp_changed_ms = (uint32_t)k_uptime_get_32();
	out->liveness_ok = false;
	return 0;
}

static int prov_mirror(void *user, mp_mirror_in_t *out)
{
	ARG_UNUSED(user);

	if (!mirror_valid) {
		return -ENOTSUP;
	}
	(void)k_mutex_lock(&mirror_lock, K_MSEC(20));
	*out = mirror_frame;
	out->ch = mirror_ch;
	out->attr = mirror_attr;
	out->hint = mirror_hints;
	(void)k_mutex_unlock(&mirror_lock);
	return 0;
}

static int prov_bundle(void *user, mp_bundle_t *out)
{
	static uint8_t i2c_map[16];

	ARG_UNUSED(user);
	memset(out, 0, sizeof(*out));

	out->fault_active = sts_alarms_active();
	if (sts_diag_i2c_scan(i2c_map) >= 0) {
		out->i2c_scan = i2c_map;
	}
	out->notes = "mp diag.snapshot";
	return 0;
}

static int prov_time(void *user, uint64_t *tai_ns, bool *fallback)
{
	ARG_UNUSED(user);
	*fallback = sts_time_is_fallback();
	return sts_time_tai_ns(tai_ns);
}

static int prov_cfg_commit(void *user, cfg_commit_res_t *res)
{
	ARG_UNUSED(user);
	/* sts_cfg_commit(), not cfg_commit(): the group appliers must run. */
	return sts_cfg_commit(res);
}

/**
 * The maintenance credential check (FMT §5.3).
 *
 * One line of substance, and that is the point: the throttle, the hard lockout
 * that reconnecting does not reset, the positive cache and the role mapping all
 * live in sts_aaa_check(), shared with the web and shell planes. Every non-zero
 * return is a denial — including -EHOSTUNREACH, which means no authority could
 * answer and is never an allow-on-failure (sts_aaa.h).
 *
 * Only -EBUSY is passed through with its identity intact, because core reports a
 * lockout distinctly; every other refusal reaches the host as one
 * indistinguishable answer, so this is not an account oracle.
 */
static int prov_auth(void *user, const char *user_name, const char *secret,
		     uint8_t *out_role)
{
	int rc;

	ARG_UNUSED(user);

	*out_role = (uint8_t)AUTH_ROLE_NONE;
	if ((user_name == NULL) || (secret == NULL)) {
		return -EACCES;
	}

	rc = sts_aaa_check(user_name, secret, out_role);
	if (rc != 0) {
		*out_role = (uint8_t)AUTH_ROLE_NONE;
		return (rc == -EBUSY) ? -EBUSY : -EACCES;
	}
	return 0;
}

/* ------------------------------------------------------- object accessors */

static int obj_read(void *user, size_t obj, mp_val_t *out)
{
	const mp_obj_t *o = mp_obj_at(obj);
	sts_health_t hs;
	quality_block_t q;

	ARG_UNUSED(user);
	if (o == NULL) {
		return -EINVAL;
	}
	out->kind = o->kind;
	out->valid = false;

	/* Rails come straight from the housekeeping sweep. */
	if (o->kind == (uint8_t)MP_KIND_RAIL) {
		size_t rail = (size_t)o->min;

		if (sts_health_snapshot(&hs) != 0) {
			return -EIO;
		}
		if (rail >= hs.ina_count) {
			return -EINVAL;
		}
		out->rail[0] = hs.ina[rail].bus_mv;
		out->rail[1] = hs.ina[rail].current_ua;
		out->rail[2] = (int32_t)(hs.ina[rail].power_uw / 1000U);
		out->rail[3] = (int32_t)hs.ina[rail].diag_alrt;
		out->valid = hs.ina[rail].valid;
		return 0;
	}

	if (strcmp(o->id, "sensor.alarms") == 0) {
		out->u = sts_alarms_active();
		out->valid = true;
		return 0;
	}
	if (strcmp(o->id, "sensor.uptime_s") == 0) {
		out->i = (int32_t)(k_uptime_get() / 1000);
		out->valid = true;
		return 0;
	}
	if (strcmp(o->id, "sensor.serial") == 0) {
		(void)strncpy(out->text, mp_serial, sizeof(out->text) - 1U);
		out->text[sizeof(out->text) - 1U] = '\0';
		out->valid = true;
		return 0;
	}
	if (strcmp(o->id, "sensor.fw_version") == 0) {
		(void)strncpy(out->text, MP_FW_VERSION,
			      sizeof(out->text) - 1U);
		out->text[sizeof(out->text) - 1U] = '\0';
		out->valid = true;
		return 0;
	}
	if (strcmp(o->id, "ui.panel.duty") == 0) {
		out->i = (int32_t)sts_panel_led_get();
		out->valid = true;
		return 0;
	}

	if (sts_health_snapshot(&hs) == 0) {
		if (strcmp(o->id, "sensor.temp.osc") == 0) {
			out->i = hs.tmp_osc_mc;
			out->valid = hs.tmp_osc_valid;
			return 0;
		}
		if (strcmp(o->id, "sensor.temp.amb") == 0) {
			out->i = hs.tmp_amb_mc;
			out->valid = hs.tmp_amb_valid;
			return 0;
		}
		if (strcmp(o->id, "sensor.temp.die") == 0) {
			out->i = hs.die_mc;
			out->valid = hs.die_valid;
			return 0;
		}
		if (strcmp(o->id, "sensor.humidity") == 0) {
			out->i = hs.humidity_mpct;
			out->valid = hs.humidity_valid;
			return 0;
		}
		if (strcmp(o->id, "sensor.fan.rpm") == 0) {
			out->i = (int32_t)hs.fan_rpm;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.fan.duty") == 0) {
			out->i = (int32_t)hs.fan_duty_pct;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.poe.class") == 0) {
			out->i = (int32_t)hs.poe_class;
			out->valid = (hs.poe_class != 0U);
			return 0;
		}
		if (strcmp(o->id, "sensor.poe.draw_mw") == 0) {
			out->i = (int32_t)hs.poe_draw_mw;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.bkp.pg") == 0) {
			out->u = (hs.bkp_stm_pg ? 1U : 0U) |
				 (hs.bkp_gps_pg ? 2U : 0U);
			out->valid = true;
			return 0;
		}
	}

	if (sts_quality_snapshot(&q) == 0) {
		if (strcmp(o->id, "sensor.timing.stratum") == 0) {
			out->i = (int32_t)q.stratum;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.lock") == 0) {
			out->i = (int32_t)q.lock_state;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.ref") == 0) {
			out->i = (int32_t)q.active_ref;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.pps_ns") == 0) {
			out->i = q.last_pps_off_ns;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.freq_ppb") == 0) {
			out->f = q.freq_err_ppb;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.adev_1s") == 0) {
			out->f = q.adev_1s;
			out->valid = (q.adev_1s != 0.0f);
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.adev_10s") == 0) {
			out->f = q.adev_10s;
			out->valid = (q.adev_10s != 0.0f);
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.adev_100s") == 0) {
			out->f = q.adev_100s;
			out->valid = (q.adev_100s != 0.0f);
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.holdover_s") == 0) {
			out->i = (int32_t)q.holdover_elapsed_s;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.root_disp_ns") == 0) {
			out->i = (int32_t)quality_ns_from_ntp_short(
				q.root_disp_q16);
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.gnss.fix") == 0) {
			out->i = (int32_t)q.gnss_fix;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.gnss.sv_used") == 0) {
			out->i = (int32_t)q.gnss_sv_used;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.gnss.sv_visible") == 0) {
			out->i = (int32_t)q.gnss_sv_visible;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.gnss.tacc_ns") == 0) {
			out->i = (int32_t)q.gnss_tacc_ns;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.ocxo.vc") == 0) {
			out->i = q.vc_sense_mv;
			out->valid = ((q.flags &
				       QUALITY_FLAG_VC_SENSE_VALID) != 0U);
			return 0;
		}
	}

	/* Everything else needs a platform accessor that does not exist yet. */
	return -ENOTSUP;
}

static int obj_apply(void *user, size_t obj, const int32_t *value)
{
	const mp_obj_t *o = mp_obj_at(obj);

	ARG_UNUSED(user);
	if (o == NULL) {
		return -EINVAL;
	}

	/* The panel dimmer is the one output sts_app.h exposes today. */
	if (strcmp(o->id, "ui.panel.duty") == 0) {
		uint8_t duty = (value != NULL)
				       ? (uint8_t)CLAMP(*value, 0, 100)
				       : 0U;

		/* A release restores the UI's own brightness policy; with no
		 * accessor for it, 0 %% is the safe resting state. */
		return sts_panel_led_set(duty);
	}

	if (strcmp(o->id, "gnss.tunnel") == 0) {
		return sts_mp_tunnel_set_gnss((value != NULL) && (*value != 0));
	}
	if (strcmp(o->id, "ref.rb.tunnel") == 0) {
		return sts_mp_tunnel_set_rb((value != NULL) && (*value != 0));
	}

	/*
	 * A release of an object we never applied must succeed, or the dead-man
	 * would count a release error on every revert.
	 */
	if (value == NULL) {
		return 0;
	}
	return -ENOTSUP;
}

static int obj_pulse(void *user, size_t obj, uint32_t ms)
{
	ARG_UNUSED(user);
	ARG_UNUSED(obj);
	ARG_UNUSED(ms);
	/* Every pulsable object is a platform-owned pin; see the TODO block. */
	return -ENOTSUP;
}

static int diag_action(void *user, uint8_t test, uint8_t step,
		       mp_diag_step_res_t *out)
{
	ARG_UNUSED(user);
	ARG_UNUSED(step);

	if (test == (uint8_t)MP_DIAG_I2C_SCAN) {
		uint8_t found[16];
		int n = sts_diag_i2c_scan(found);

		if (n < 0) {
			return n;
		}
		out->verdict = (n >= 10) ? (uint8_t)MP_DIAG_PASS
					 : (uint8_t)MP_DIAG_FAIL;
		out->value = n;
		(void)snprintf(out->text, sizeof(out->text), "%d devices", n);
		return 0;
	}

	/* The rest need platform-area actuation. */
	return -ENOTSUP;
}

/* ------------------------------------------------------------------ mirror */

void sts_mp_mirror_publish(const mp_mirror_in_t *frame)
{
	size_t cells;

	if ((frame == NULL) || (frame->ch == NULL) || (frame->attr == NULL)) {
		return;
	}
	cells = (size_t)frame->rows * (size_t)frame->cols;
	if ((cells == 0U) || (cells > MP_MIRROR_CELLS)) {
		return;
	}
	if (k_mutex_lock(&mirror_lock, K_MSEC(5)) != 0) {
		return; /* the tick has it; drop this frame rather than block ui */
	}

	mirror_frame = *frame;
	memcpy(mirror_ch, frame->ch, cells);
	memcpy(mirror_attr, frame->attr, cells);
	mirror_frame.hint_count = 0U;
	if (frame->hint != NULL) {
		uint8_t n = frame->hint_count;

		if (n > UI_SURF_MAX_HINTS) {
			n = (uint8_t)UI_SURF_MAX_HINTS;
		}
		memcpy(mirror_hints, frame->hint, (size_t)n * sizeof(ui_hint_t));
		mirror_frame.hint_count = n;
	}
	mirror_frame.ch = mirror_ch;
	mirror_frame.attr = mirror_attr;
	mirror_frame.hint = mirror_hints;
	mirror_valid = true;

	(void)k_mutex_unlock(&mirror_lock);
}

/* --------------------------------------------------------- shell hand-off */

static void mp_bypass(const struct shell *sh, uint8_t *data, size_t len)
{
	ARG_UNUSED(sh);

	(void)mp_input(&mp, data, len);

	if (mp_mode(&mp) != (uint8_t)MP_MODE_MP) {
		/* The exit magic or sys.mode brought us back. */
		shell_set_bypass(mp_shell, NULL);
		LOG_INF("MP mode left; shell restored");
	}
}

bool sts_mp_active(void)
{
	return mp_started && (mp_mode(&mp) == (uint8_t)MP_MODE_MP);
}

bool sts_mp_shell_tap(uint8_t b)
{
	if (!mp_started) {
		return false;
	}
	if (mp_shell_byte(&mp, b) == 1) {
		shell_set_bypass(mp_shell, mp_bypass);
		LOG_INF("MP mode entered by magic");
		return true;
	}
	return false;
}

void sts_mp_notify_link(bool up)
{
	if (!mp_started) {
		return;
	}
	(void)mp_set_link(&mp, up);
	if (!up && (mp_shell != NULL)) {
		shell_set_bypass(mp_shell, NULL);
	}
}

void sts_mp_notify_break(void)
{
	if (!mp_started) {
		return;
	}
	(void)mp_mode_exit(&mp);
	if (mp_shell != NULL) {
		shell_set_bypass(mp_shell, NULL);
	}
}

const mp_ctx_t *sts_mp_ctx(void)
{
	return mp_started ? &mp : NULL;
}

void sts_mp_tick(void)
{
	if (mp_started) {
		(void)mp_tick(&mp);
	}
}

/* ------------------------------------------------------------ shell command */

static int cmd_mp_enter(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mp_started) {
		shell_error(sh, "MP engine is not running");
		return -ENODEV;
	}
	mp_shell = sh;
	(void)mp_set_link(&mp, true);
	(void)mp_mode_enter(&mp);
	shell_print(sh, "MP mode: proto %u, manifest %u objects, hash 0x%08x",
		    (unsigned int)MP_PROTO_VER, (unsigned int)mp_obj_count(),
		    (unsigned int)mp_manifest_hash_cached(&mp));
	shell_print(sh, "send \\x01MP0\\x02 or BREAK to return to the shell");
	shell_set_bypass(sh, mp_bypass);
	return 0;
}

static int cmd_mp_exit(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mp_started) {
		return -ENODEV;
	}
	(void)mp_mode_exit(&mp);
	shell_set_bypass(sh, NULL);
	shell_print(sh, "MP mode left");
	return 0;
}

static int cmd_mp_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mp_started) {
		shell_error(sh, "MP engine is not running");
		return -ENODEV;
	}
	shell_print(sh, "mode         %s",
		    sts_mp_active() ? "mp" : "shell");
	shell_print(sh, "manifest     %u objects, hash 0x%08x, ver %u",
		    (unsigned int)mp_obj_count(),
		    (unsigned int)mp_manifest_hash_cached(&mp),
		    (unsigned int)MP_MANIFEST_VER);
	shell_print(sh, "requests     %u (errors %u, notifications %u)",
		    mp.requests, mp.rpc_errors, mp.notifications);
	shell_print(sh, "frames       rx %u, msgs %u, crc-err %u, cobs-err %u",
		    mp.rx.frames, mp.rx.msgs, mp.rx.crc_errors,
		    mp.rx.cobs_errors);
	shell_print(sh, "records      %u sent (tx errors %u)", mp.records_sent,
		    mp.tx_errors);
	shell_print(sh, "session      %u, overrides %u/%u",
		    mp.ovr.sess.id, (unsigned int)mp_ovr_active(&mp.ovr),
		    (unsigned int)MP_LEASE_MAX);
	{
		const char *who = mp_ovr_session_user(&mp.ovr);

		shell_print(sh, "auth         role %s, user %s",
			    mp_role_name(mp_ovr_session_role(&mp.ovr)),
			    (who[0] != '\0') ? who : "-");
	}
	shell_print(sh, "safety       grants %u, vetoes %u, deadman %u, "
			"verify-fail %u, refusals %u",
		    mp.ovr.grants, mp.ovr.vetoes, mp.ovr.deadman_reverts,
		    mp.ovr.verify_failures, mp.ovr.refusals);
	shell_print(sh, "mirror       %s (%u frames, %u keyframes)",
		    mirror_valid ? "wired" : "UNWIRED (ui area must publish)",
		    mp.mirror.frames, mp.mirror.keyframes);
	shell_print(sh, "events       %u queued, %u dropped",
		    (unsigned int)mp_stream_event_count(&mp.st),
		    mp_stream_event_dropped(&mp.st));
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	mp_sub,
	SHELL_CMD(enter, NULL, "Hand the console to the Maintenance Protocol",
		  cmd_mp_enter),
	SHELL_CMD(exit, NULL, "Return the console to the shell", cmd_mp_exit),
	SHELL_CMD(status, NULL, "Maintenance Protocol counters", cmd_mp_status),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(mp, &mp_sub, "Field Maintenance Tool protocol", NULL);

/* --------------------------------------------------------------------- init */

/** Derive the G2 confirmation string from the board identity. */
static void fill_serial(void)
{
	uint8_t uid[12];
	ssize_t n = hwinfo_get_device_id(uid, sizeof(uid));
	size_t i;
	size_t w = 0U;

	(void)strcpy(mp_serial, "STS1000-");
	w = strlen(mp_serial);
	if (n <= 0) {
		(void)strcpy(&mp_serial[w], "UNPROVISIONED");
		return;
	}
	for (i = 0U; (i < (size_t)n) && ((w + 2U) < sizeof(mp_serial)); i++) {
		static const char hexd[] = "0123456789ABCDEF";

		mp_serial[w++] = hexd[(uid[i] >> 4) & 0x0FU];
		mp_serial[w++] = hexd[uid[i] & 0x0FU];
	}
	mp_serial[w] = '\0';
}

int sts_mp_start(void)
{
	mp_wiring_t w;
	unsigned int i;
	int rc;

	if (mp_started) {
		return 0;
	}
	if (!device_is_ready(mp_uart)) {
		LOG_ERR("console UART not ready; MP unavailable");
		return -ENODEV;
	}

	fill_serial();

	for (i = 0U; i < 2U; i++) {
		mp_reasm[i].buf = mp_reasm_buf[i];
		mp_reasm[i].cap = sizeof(mp_reasm_buf[i]);
	}

	memset(&w, 0, sizeof(w));
	w.tx = mp_tx;
	w.mono_ms = mono_ms;
	w.model = "STS1000";
	w.serial = mp_serial;
	w.fw_version = MP_FW_VERSION;
	w.boot_version = "mcuboot";
	w.board_id = mp_serial;
	w.telem = prov_telem;
	w.pps = prov_pps;
	w.ilk = prov_ilk;
	w.mirror = prov_mirror;
	w.bundle = prov_bundle;
	w.time_get = prov_time;
	w.apply = obj_apply;
	w.obj_read = obj_read;
	w.pulse = obj_pulse;
	w.diag = diag_action;
	w.cfg_commit = prov_cfg_commit;
	w.auth = prov_auth;
	w.img = sts_dfu_port();
	w.cfg = sts_cfg();
	w.log = sts_logring();
	w.scratch = mp_scratch;
	w.scratch_len = sizeof(mp_scratch);
	w.reasm = mp_reasm;
	w.reasm_n = 2U;
	w.mirror_prev_ch = mirror_prev_ch;
	w.mirror_prev_attr = mirror_prev_attr;
	w.mirror_cells = MP_MIRROR_CELLS;

	rc = mp_init(&mp, &w);
	if (rc != 0) {
		LOG_ERR("mp_init failed (%d)", rc);
		return rc;
	}

	mp_started = true;
	LOG_INF("MP ready: %u objects, hash 0x%08x, serial %s",
		(unsigned int)mp_obj_count(),
		(unsigned int)mp_manifest_hash_cached(&mp), mp_serial);
	return 0;
}

#endif /* CONFIG_STS1000_CONSOLE */
