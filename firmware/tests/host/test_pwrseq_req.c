/*
 * STS1000 "Meridian" — the parameterised sequencer request mailbox: from an
 * `obj.override` on the wire to a pin the housekeeping thread moved, and back.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * WHAT THIS SUITE EXISTS TO CATCH, stated so it cannot be diluted later.
 *
 * pwrseq is the single writer of every rail pin. pwrseq_exec.c's own
 * "operator requests" block says why: emit() writes the action ring's head and
 * pwrseq_drain() reads and EXECUTES it with no lock anywhere, so a console
 * thread that called in would be appending to a queue whose consumer is
 * mid-drain. The three existing operator entry points respect that by posting
 * single BITS of one atomic word — which works only because each is idempotent
 * and therefore coalesces harmlessly.
 *
 * A general object setter cannot use that mechanism: it must carry
 * (object, value), which a bitmask cannot express. That is the whole reason 31
 * mutable manifest objects answered MP_E_NOTSUP. This suite covers the
 * mechanism that replaces it, and the ONE object wired onto it.
 *
 * The three properties, and why a lesser test would pass without them:
 *
 *   1. THE REPLY MUST NOT LIE. Everything through the mailbox is up to 250 ms
 *      late, but `obj.override`'s reply has always asserted the value took
 *      effect and started a lease clock. A test that only checked "the pin
 *      eventually moved" would pass against a reply that claimed it had moved
 *      already. So the reply is read at the instant it is produced — BEFORE any
 *      drain — and `verify_pending` is asserted true, then false once the drain
 *      has confirmed it.
 *
 *   2. THE COLLISION MUST BE DEFINED. Two values for one object before a drain
 *      is a real case. Silently dropping one is not acceptable and neither is
 *      unbounded growth; the answer here is last-writer-wins per object, and
 *      the test asserts BOTH halves — the surviving value is the second, and
 *      the replacement was counted rather than lost.
 *
 *   3. A REFUSAL MUST REACH SOMEONE. A drained request the sequencer rejects
 *      goes to mp_ovr_veto(), so it lands on channel 0x09 and in the audit log
 *      exactly as a shed does. A request that vanishes silently is the same
 *      defect in a new place.
 *
 * TWO MIRRORS of Zephyr code live here and are deliberately the ONLY routes in
 * and out — hk_pass() for pwrseq_exec.c's pwrseq_service_mailbox() and
 * console_pass() for mp_glue.c's mp_req_drain_locked(). Both are a handful of
 * marshalling lines around decisions that live in headers this file compiles
 * for real (sts_pwrseq_req.h) or in core it links for real (mp_override.c,
 * mp_rpc.c). The half a mirror cannot check — that those two .c files still
 * make the call at all — is checked by reading their sources, brace-matched and
 * function-scoped, the way test_mp_veto.c and test_mp_deferred.c do.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "mp/mp.h"
#include "mp/mp_manifest.h"
#include "mp/mp_override.h"
#include "pwrseq/pwrseq.h"
#include "zephyr/platform/sts_pwrseq_req.h"
/* For sts_rb_code_for_mv(): the millivolt-to-wiper inverse the VCC_RB setpoint
 * row computes with. Compiled for real here, not modelled — it is the
 * arithmetic that stands between a maintenance setpoint and a destroyed FE. */
#include "zephyr/platform/sts_rbguard.h"

#ifndef STS_APP_SRC_DIR
#error "STS_APP_SRC_DIR must be defined by tests/host/CMakeLists.txt"
#endif

#define SERIAL "STS1000-000042"
#define ADMIN_USER "root"
#define ADMIN_PW "correct horse"

/** The object under test: the one rail wired onto the mailbox. */
#define PANEL_ID "pwr.panel.led.en"

/* The board's real cadences, so a test cannot model a faster one. */
#define HK_TICK_MS 250U

/* ============================================================ the fixture == */

static mp_ctx_t g_c;
static uint8_t g_scratch[8192];
static mp_reasm_t g_slots[2];
static uint8_t g_slot_buf[2][2048];
static char g_prev_ch[20 * 60];
static uint8_t g_prev_attr[20 * 60];

static cfg_ctx_t g_cfg;
static logr_t g_log;
static logr_rec_t g_log_slots[32];

static uint32_t g_now;
static uint8_t g_wire[32768];
static size_t g_wire_len;

/* ------------------------------------------------------------- the board -- */

/*
 * The pin, and the two pwrseq facts the drain consults.
 *
 * `g_duty` stands in for panel_pwm.c's programmed duty. PANEL_LED_EN (PC0) is
 * asserted exactly when it is non-zero — that is sts_panel_led_set()'s
 * documented behaviour and the reason `pwr.panel.led.en` and `ui.panel.duty`
 * share one setter, which is in turn why the veto policy lists both ids under
 * one subject.
 */
static uint8_t g_duty;
static bool g_panel_stage_done; /* pwrseq_status_t::panel_led_on */
static uint8_t g_shed;          /* pwrseq_status_t::shed */
static bool g_hw_fails;         /* the actuator returns an error */

/** True when PANEL_LED_EN is asserted. The pin, not anyone's belief about it. */
static bool pin_on(void)
{
	return g_duty != 0U;
}

static int panel_led_set(uint8_t duty)
{
	if (g_hw_fails) {
		return -EIO;
	}
	g_duty = duty;
	return 0;
}

/* ----------------------------------------------------------- the rails ----- */

/*
 * pwrseq_exec.c's pwrseq_rail_t, modelled: the PIN and, separately, the level
 * FIRMWARE commands.
 *
 * Keeping them apart is the whole design under test. A mailbox request moves
 * `pin` and never touches `automatic`, so the sequencer's own level survives an
 * override and is what a release restores; if the two were one field, "ON only
 * where firmware also wants it on" and "release goes back to firmware" would
 * both be tautologies and this suite would prove nothing.
 */
typedef struct {
	bool pin;
	bool automatic;
	uint8_t shed_at; /* pwrseq_shed_level_t; 0 = never shed */
} rail_t;

static rail_t g_rail[STS_PWRSEQ_REQ_COUNT];

/* The GNSS-reset notification the GPS row owes gnssmgr on a re-power. */
static unsigned int g_gnss_reset_notes;

/* ------------------------------------------------------------ the digipot -- */

/*
 * The as-built VCC_RB transfer function, out of core/pwrseq's own defaults, so
 * the codes this suite computes are the codes the board would compute.
 */
static pwrseq_rb_xfer_t g_xfer;
static uint32_t g_rb_vmax_mv;
static uint16_t g_dp_code;      /* what the part holds */
static uint16_t g_dp_auto_code; /* what firmware last commanded */
static bool g_rb_gated;         /* pwrseq_status_t::rb_gated */
static bool g_in_stage8;        /* pwrseq_status_t::stage == PWRSEQ_STAGE_8_RB */

/* ------------------------------------------------------------ the mailbox -- */

/*
 * The real queue out of the real header. No spinlock: the header is written so
 * the lock is the .c's, which is exactly what makes the collision rule and the
 * counters testable here, where there are no threads to race.
 */
static sts_pwrseq_reqq_t g_mbox;

/* ============================================== mirror: the platform side == */

/*
 * pwrseq_exec.c's pwrseq_mbox_apply_panel(), reproduced.
 *
 * The refusal policy is the part worth mirroring: OFF and release are never
 * refused (taking a load down cannot be contrary to firmware, which is what
 * makes a lapsed lease reliable), and ON is refused with a REASON that
 * distinguishes a shed load from a stage the sequencer has not reached.
 */
static bool apply_panel(bool release, int32_t value, uint8_t *out_err,
			int32_t *out_val)
{
	uint8_t duty;

	*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_NONE;
	*out_val = 0;

	if (release || (value == 0)) {
		if (panel_led_set(0) != 0) {
			*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_HW;
			return false;
		}
		return true;
	}

	if (g_shed >= 2U /* PWRSEQ_SHED_PANEL_LED */) {
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_SHED;
		return false;
	}
	if (!g_panel_stage_done) {
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_STAGE;
		return false;
	}

	duty = g_duty;
	if (duty == 0U) {
		duty = 100U;
	}
	if (panel_led_set(duty) != 0) {
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_HW;
		return false;
	}

	*out_val = 1;
	return true;
}

/*
 * pwrseq_exec.c's pwrseq_mbox_apply_duty(), reproduced.
 *
 * The refusal is the migration's whole point: written from the console thread,
 * as `ui.panel.duty` was, a duty simply landed and re-lit a panel the ladder had
 * shed. Dark is still never refused.
 */
static bool apply_duty(bool release, int32_t value, uint8_t *out_err,
		       int32_t *out_val)
{
	uint8_t duty;

	*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_NONE;
	*out_val = 0;

	duty = release ? 0U
		       : (uint8_t)((value < 0) ? 0 : ((value > 100) ? 100 : value));

	if (duty != 0U) {
		if (g_shed >= 2U /* PWRSEQ_SHED_PANEL_LED */) {
			*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_SHED;
			return false;
		}
		if (!g_panel_stage_done) {
			*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_STAGE;
			return false;
		}
	}

	if (panel_led_set(duty) != 0) {
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_HW;
		return false;
	}

	*out_val = (int32_t)duty;
	return true;
}

/*
 * pwrseq_exec.c's pwrseq_mbox_apply_rail(), reproduced.
 *
 * Three rules and nothing else: ON only where firmware also wants it on, OFF
 * never refused, release back to firmware's own level.
 */
static bool apply_rail(uint8_t req, bool release, int32_t value,
		       uint8_t *out_err, int32_t *out_val)
{
	rail_t *r = &g_rail[req];
	bool want;
	bool was;

	*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_NONE;
	*out_val = 0;

	if (g_hw_fails) {
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_HW;
		return false;
	}

	want = release ? r->automatic : (value != 0);

	if (want && !r->automatic) {
		*out_err = ((r->shed_at != 0U) && (g_shed >= r->shed_at))
				   ? (uint8_t)STS_PWRSEQ_REQ_ERR_SHED
				   : (uint8_t)STS_PWRSEQ_REQ_ERR_STAGE;
		return false;
	}

	was = r->pin;
	r->pin = want;

	if (want && !was && (req == (uint8_t)STS_PWRSEQ_REQ_GPS_EN)) {
		g_gnss_reset_notes++;
	}

	*out_val = want ? 1 : 0;
	return true;
}

/*
 * pwrseq_exec.c's pwrseq_mbox_apply_vset(), reproduced — including both
 * refusals, which are what keep a maintenance setpoint off a live FE-5680A.
 */
static bool apply_vset(bool release, int32_t value, uint8_t *out_err,
		       int32_t *out_val)
{
	uint16_t code;

	*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_NONE;
	*out_val = 0;

	if (release) {
		code = g_dp_auto_code;
	} else {
		if (g_in_stage8) {
			*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_STAGE;
			return false;
		}
		if (g_rb_gated) {
			*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_GATED;
			return false;
		}
		code = sts_rb_code_for_mv(&g_xfer, value, g_rb_vmax_mv);
	}

	if (g_hw_fails) {
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_HW;
		return false;
	}

	g_dp_code = code;
	*out_val = pwrseq_rb_expected_mv(&g_xfer, code);
	return true;
}

/**
 * One housekeeping pass: pwrseq_exec.c's pwrseq_service_mailbox().
 *
 * Claim under the lock, execute outside it, settle under it again — the
 * claim/execute/settle shape the header requires and the .c implements.
 */
static void hk_pass(void)
{
	uint8_t req;

	g_now += HK_TICK_MS;

	for (req = (uint8_t)STS_PWRSEQ_REQ_NONE + 1U;
	     req < (uint8_t)STS_PWRSEQ_REQ_COUNT; req++) {
		uint8_t err = (uint8_t)STS_PWRSEQ_REQ_ERR_NONE;
		int32_t applied = 0;
		int32_t value = 0;
		bool release = false;
		bool ok;

		if (!sts_pwrseq_reqq_claim(&g_mbox, req, &release, &value)) {
			continue;
		}

		switch ((sts_pwrseq_req_id_t)req) {
		case STS_PWRSEQ_REQ_PANEL_LED_EN:
			ok = apply_panel(release, value, &err, &applied);
			break;
		case STS_PWRSEQ_REQ_PANEL_DUTY:
			ok = apply_duty(release, value, &err, &applied);
			break;
		case STS_PWRSEQ_REQ_RB_VSET_MV:
			ok = apply_vset(release, value, &err, &applied);
			break;
		default:
			ok = apply_rail(req, release, value, &err, &applied);
			break;
		}

		TEST_ASSERT_EQUAL_INT(0,
			sts_pwrseq_reqq_settle(
				&g_mbox, req,
				ok ? (uint8_t)STS_PWRSEQ_REQ_OUT_APPLIED
				   : (uint8_t)STS_PWRSEQ_REQ_OUT_REFUSED,
				err, applied, g_now));
	}
}

/* =============================================== mirror: the console side == */

/*
 * The manifest ids each mailbox row actuates — mp_glue.c's mp_req_objects[],
 * resolved through mp_obj_find() exactly as the shipped table is, so a renamed
 * object fails here too.
 */
static const char *const g_req_objects[STS_PWRSEQ_REQ_COUNT] = {
	[STS_PWRSEQ_REQ_NONE] = NULL,
	[STS_PWRSEQ_REQ_PANEL_LED_EN] = PANEL_ID,
	[STS_PWRSEQ_REQ_PANEL_DUTY] = "ui.panel.duty",
	[STS_PWRSEQ_REQ_LAMP_TEST] = "ui.lamp.test",
	[STS_PWRSEQ_REQ_GPS_EN] = "pwr.gps.en",
	[STS_PWRSEQ_REQ_ANT_BIAS_EN] = "pwr.ant.bias.en",
	[STS_PWRSEQ_REQ_DISP_EN] = "pwr.disp.en",
	[STS_PWRSEQ_REQ_RB_GATE] = "pwr.rb.gate",
	[STS_PWRSEQ_REQ_RB_VSET_MV] = "pwr.rb.vset_mv",
	[STS_PWRSEQ_REQ_REF_TERM_EN] = "ref.term.en",
	[STS_PWRSEQ_REQ_FAN_DUTY] = "sys.fan.duty",
	[STS_PWRSEQ_REQ_RGB_MODE] = "ui.rgb.mode",
	[STS_PWRSEQ_REQ_RGB_R] = "ui.rgb.r",
	[STS_PWRSEQ_REQ_RGB_G] = "ui.rgb.g",
	[STS_PWRSEQ_REQ_RGB_B] = "ui.rgb.b",
	[STS_PWRSEQ_REQ_DISP_BL] = "ui.disp.bl",
};

/** The mailbox row that actuates @p id, or STS_PWRSEQ_REQ_NONE. */
static uint8_t req_of(const char *id)
{
	uint8_t req;

	for (req = (uint8_t)STS_PWRSEQ_REQ_NONE + 1U;
	     req < (uint8_t)STS_PWRSEQ_REQ_COUNT; req++) {
		if (strcmp(g_req_objects[req], id) == 0) {
			return req;
		}
	}
	TEST_FAIL_MESSAGE(id);
	return (uint8_t)STS_PWRSEQ_REQ_NONE;
}

/** mp_glue.c's mp_req_drain_locked(). */
static void req_drain(void)
{
	uint8_t req;

	for (req = (uint8_t)STS_PWRSEQ_REQ_NONE + 1U;
	     req < (uint8_t)STS_PWRSEQ_REQ_COUNT; req++) {
		sts_pwrseq_req_result_t r;
		int obj;

		if (!sts_pwrseq_reqq_take(&g_mbox, req, &r)) {
			continue;
		}
		obj = mp_obj_find(g_req_objects[req]);
		TEST_ASSERT_TRUE_MESSAGE(obj >= 0, g_req_objects[req]);

		if (r.outcome == (uint8_t)STS_PWRSEQ_REQ_OUT_APPLIED) {
			int rc = mp_ovr_settled(&g_c.ovr, (size_t)obj, g_now);

			TEST_ASSERT_TRUE((rc == 0) || (rc == -ENOENT));
			continue;
		}
		(void)mp_veto(&g_c, (size_t)obj,
			      sts_pwrseq_req_reason_of(r.err));
	}
}

/** One console-supervisor pass: the outcome drain, then the engine tick. */
static void console_pass(void)
{
	req_drain();
	(void)mp_tick(&g_c);
}

/* -------------------------------------------------------------- callbacks -- */

static uint32_t clock_cb(void *user)
{
	(void)user;
	return g_now;
}

static int tx_cb(void *user, const uint8_t *wire, size_t len)
{
	(void)user;
	TEST_ASSERT_TRUE(g_wire_len + len <= sizeof(g_wire));
	memcpy(&g_wire[g_wire_len], wire, len);
	g_wire_len += len;
	return 0;
}

/*
 * mp_glue.c's obj_apply(), for the one branch under test.
 *
 * The load-bearing line is the return: MP_APPLY_PENDING, never 0. Returning 0
 * would tell mp_ovr_grant() the pin had moved, and the reply would then say so.
 */
static unsigned int g_apply_calls;
static unsigned int g_release_calls;
/** Force every apply to answer MP_APPLY_PENDING, changing nothing else. */
static bool g_force_pending;

static int apply_cb(void *user, size_t obj, const int32_t *value)
{
	const mp_obj_t *o = mp_obj_at(obj);
	uint8_t req;

	(void)user;
	TEST_ASSERT_NOT_NULL(o);

	if (g_force_pending) {
		/* Any object, asynchronous, with nothing else changed. */
		return MP_APPLY_PENDING;
	}

	for (req = (uint8_t)STS_PWRSEQ_REQ_NONE + 1U;
	     req < (uint8_t)STS_PWRSEQ_REQ_COUNT; req++) {
		int32_t v;
		int rc;

		if (strcmp(o->id, g_req_objects[req]) != 0) {
			continue;
		}

		/* Bool rows normalise; the two scalar rows pass the value the
		 * interlocks already clamped, exactly as obj_apply() does. */
		if ((req == (uint8_t)STS_PWRSEQ_REQ_PANEL_DUTY) ||
		    (req == (uint8_t)STS_PWRSEQ_REQ_RB_VSET_MV)) {
			v = (value != NULL) ? *value : 0;
		} else {
			v = ((value != NULL) && (*value != 0)) ? 1 : 0;
		}

		g_apply_calls++;
		if (value == NULL) {
			g_release_calls++;
		}
		rc = sts_pwrseq_reqq_post(&g_mbox, req, (value == NULL), v,
					  g_now);
		return (rc == 0) ? MP_APPLY_PENDING : rc;
	}
	return 0;
}

/** mp_glue.c's obj_read() branch: the PIN, not the sequencer's belief. */
static int obj_read_cb(void *user, size_t obj, mp_val_t *out)
{
	const mp_obj_t *o = mp_obj_at(obj);

	(void)user;
	TEST_ASSERT_NOT_NULL(o);
	out->kind = o->kind;
	out->valid = true;
	if (strcmp(o->id, PANEL_ID) == 0) {
		out->i = pin_on() ? 1 : 0;
		return 0;
	}
	if (strcmp(o->id, "ui.panel.duty") == 0) {
		out->i = (int32_t)g_duty;
		return 0;
	}
	if (strcmp(o->id, "pwr.rb.vset_mv") == 0) {
		out->i = pwrseq_rb_expected_mv(&g_xfer, g_dp_code);
		return 0;
	}
	{
		uint8_t req;

		for (req = (uint8_t)STS_PWRSEQ_REQ_NONE + 1U;
		     req < (uint8_t)STS_PWRSEQ_REQ_COUNT; req++) {
			if (strcmp(o->id, g_req_objects[req]) == 0) {
				/* The PIN, never `automatic` — that is the
				 * distinction sts_pwrseq_rail_on() exists for. */
				out->i = g_rail[req].pin ? 1 : 0;
				return 0;
			}
		}
	}
	out->i = 0;
	return 0;
}

static mp_ilk_state_t g_ilk;

static int ilk_cb(void *user, mp_ilk_state_t *out)
{
	(void)user;
	*out = g_ilk;
	return 0;
}

static int auth_cb(void *user, const char *user_name, const char *secret,
		   uint8_t *out_role)
{
	(void)user;
	*out_role = (uint8_t)MP_ROLE_NONE;
	if ((user_name == NULL) || (secret == NULL)) {
		return -EACCES;
	}
	if ((strcmp(user_name, ADMIN_USER) == 0) &&
	    (strcmp(secret, ADMIN_PW) == 0)) {
		*out_role = (uint8_t)MP_ROLE_ADMIN;
		return 0;
	}
	return -EACCES;
}

void setUp(void)
{
	mp_wiring_t w;
	unsigned int i;

	g_now = 10000U;
	g_wire_len = 0U;
	g_duty = 0U;
	g_panel_stage_done = true;
	g_shed = 0U;
	g_hw_fails = false;
	g_apply_calls = 0U;
	g_release_calls = 0U;
	g_gnss_reset_notes = 0U;
	g_force_pending = false;
	memset(&g_mbox, 0, sizeof(g_mbox));

	/*
	 * A healthy, fully-brought-up board: the sequencer commands every rail
	 * it owns ON. That is the state in which an override is interesting —
	 * with `automatic` false the only reachable answer is a refusal.
	 */
	memset(g_rail, 0, sizeof(g_rail));
	for (i = (unsigned int)STS_PWRSEQ_REQ_NONE + 1U;
	     i < (unsigned int)STS_PWRSEQ_REQ_COUNT; i++) {
		g_rail[i].pin = true;
		g_rail[i].automatic = true;
	}
	g_rail[STS_PWRSEQ_REQ_DISP_EN].shed_at = 1U;  /* PWRSEQ_SHED_DISPLAY */
	g_rail[STS_PWRSEQ_REQ_RB_GATE].shed_at = 3U;  /* PWRSEQ_SHED_RB */

	{
		pwrseq_cfg_t c;

		pwrseq_cfg_default(&c);
		g_xfer = c.rb_xfer;
		g_rb_vmax_mv = c.rb_vmax_mv;
		g_dp_auto_code = c.digipot_operating_code;
		g_dp_code = c.digipot_operating_code;
	}
	g_rb_gated = false;
	g_in_stage8 = false;

	memset(&g_ilk, 0, sizeof(g_ilk));
	g_ilk.ocxo_warm = true;
	g_ilk.supercaps_ok = true;
	g_ilk.vcc_rb_mv = 15000;
	g_ilk.vcc_rb_valid = true;
	/* What prov_ilk() now reports: the rail the PROGRAMMED code implies, not
	 * the configured ceiling. MP_ILK_RB_VERIFY compares INA228 0x47 against
	 * it for `pwr.rb.gate`. */
	g_ilk.rb_expected_mv = pwrseq_rb_expected_mv(&g_xfer, g_dp_code);
	g_ilk.vcc_rb_mv = g_ilk.rb_expected_mv;
	g_ilk.rb_vmax_mv = 15000U;
	g_ilk.rb_code_max = 255U;
	g_ilk.liveness_ok = true;

	for (i = 0U; i < 2U; i++) {
		g_slots[i].buf = g_slot_buf[i];
		g_slots[i].cap = sizeof(g_slot_buf[i]);
	}
	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, NULL));
	TEST_ASSERT_EQUAL_INT(0, logr_init(&g_log, g_log_slots, 32U));

	memset(&w, 0, sizeof(w));
	w.tx = tx_cb;
	w.mono_ms = clock_cb;
	w.model = "STS1000";
	w.serial = SERIAL;
	w.fw_version = "1.2.3";
	w.boot_version = "0.9.0";
	w.board_id = SERIAL;
	w.apply = apply_cb;
	w.obj_read = obj_read_cb;
	w.ilk = ilk_cb;
	w.auth = auth_cb;
	w.cfg = &g_cfg;
	w.log = &g_log;
	w.scratch = g_scratch;
	w.scratch_len = sizeof(g_scratch);
	w.reasm = g_slots;
	w.reasm_n = 2U;
	w.mirror_prev_ch = g_prev_ch;
	w.mirror_prev_attr = g_prev_attr;
	w.mirror_cells = sizeof(g_prev_ch);

	TEST_ASSERT_EQUAL_INT(0, mp_init(&g_c, &w));
	TEST_ASSERT_EQUAL_INT(0, mp_set_link(&g_c, true));
	TEST_ASSERT_EQUAL_INT(0, mp_mode_enter(&g_c));
}

void tearDown(void)
{
}

/* --------------------------------------------------------------- RPC helper */

#define TOKS 512U
static mp_json_t g_rp;
static mp_json_tok_t g_rtok[TOKS];

static void call(const char *req)
{
	const char *out = NULL;
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_rpc_handle(&g_c, (const uint8_t *)req,
					       strlen(req), &out, &len));
	TEST_ASSERT_NOT_NULL_MESSAGE(out, req);
	TEST_ASSERT_TRUE(mp_json_parse(&g_rp, out, len, g_rtok, TOKS, 0U) > 0);
}

static int result(void)
{
	int r = mp_json_obj_get(&g_rp, 0, "result");

	TEST_ASSERT_TRUE_MESSAGE(r >= 0, "expected a result, got an error");
	return r;
}

static int64_t res_i(const char *key)
{
	int64_t v = 0;

	TEST_ASSERT_EQUAL_INT(0,
			      mp_json_i64(&g_rp,
					  mp_json_obj_get(&g_rp, result(), key),
					  &v));
	return v;
}

static bool res_b(const char *key)
{
	bool v = false;

	TEST_ASSERT_EQUAL_INT(0,
			      mp_json_bool(&g_rp,
					   mp_json_obj_get(&g_rp, result(), key),
					   &v));
	return v;
}

static int64_t err_code(void)
{
	int e = mp_json_obj_get(&g_rp, 0, "error");
	int64_t code = 0;

	TEST_ASSERT_TRUE_MESSAGE(e >= 0, "expected an error reply");
	TEST_ASSERT_EQUAL_INT(0,
			      mp_json_i64(&g_rp,
					  mp_json_obj_get(&g_rp, e, "code"),
					  &code));
	return code;
}

static uint32_t session(void)
{
	call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"session.open\","
	     "\"params\":{\"client\":\"fmt\",\"user\":\"" ADMIN_USER "\","
	     "\"secret\":\"" ADMIN_PW "\"}}");
	return (uint32_t)res_i("sid");
}

/** `obj.override pwr.panel.led.en = @p on`, through the real RPC layer. */
static void override_panel(uint32_t sid, bool on)
{
	char req[256];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"" PANEL_ID "\",\"value\":%s,"
		       "\"sid\":%u}}",
		       on ? "true" : "false", (unsigned int)sid);
	call(req);
}

static size_t panel_obj(void)
{
	int obj = mp_obj_find(PANEL_ID);

	TEST_ASSERT_TRUE(obj >= 0);
	return (size_t)obj;
}

/* ================================================= 1. end to end, one object */

/**
 * THE ASSERTION THE WHOLE SUITE EXISTS FOR.
 *
 * A real `obj.override` over the RPC layer; the pin has NOT moved when the reply
 * is produced and the reply says so; the housekeeping drain moves it; the
 * console pass confirms it; and only then does `verify_pending` go false.
 */
static void test_the_override_reaches_the_pin_through_the_drain(void)
{
	uint32_t sid = session();

	TEST_ASSERT_FALSE(pin_on());

	override_panel(sid, true);

	/*
	 * The reply, read at the instant it is produced. The lease exists and
	 * the value is what was asked for — both true — but the pin has NOT
	 * moved, and `verify_pending` is the field that says so. This is the
	 * assertion a "the pin eventually moved" test would miss entirely.
	 */
	TEST_ASSERT_EQUAL_INT64(1, res_i("value"));
	TEST_ASSERT_TRUE_MESSAGE(res_b("verify_pending"),
				 "the reply claimed the pin had moved before "
				 "the sequencer had touched it");
	TEST_ASSERT_NOT_NULL(mp_ovr_lease(&g_c.ovr, panel_obj()));
	TEST_ASSERT_TRUE(mp_ovr_pending(&g_c.ovr, panel_obj()));
	TEST_ASSERT_FALSE_MESSAGE(pin_on(),
				  "the console thread wrote the rail pin "
				  "itself; that is the single-writer violation "
				  "this mailbox exists to prevent");

	/* The owning thread performs it. */
	hk_pass();
	TEST_ASSERT_TRUE_MESSAGE(pin_on(), "the drain did not move the pin");

	/* ...and the console learns that it landed. */
	console_pass();
	TEST_ASSERT_FALSE(mp_ovr_pending(&g_c.ovr, panel_obj()));
	TEST_ASSERT_NOT_NULL(mp_ovr_lease(&g_c.ovr, panel_obj()));
	TEST_ASSERT_EQUAL_UINT32(1U, g_c.ovr.settled);

	/*
	 * A read-back now reports the pin, so the host can confirm it too. A
	 * MP_KIND_BOOL object emits a JSON boolean here, unlike `obj.override`'s
	 * reply, which carries the leased value as a number.
	 */
	call("{\"jsonrpc\":\"2.0\",\"id\":9,\"method\":\"obj.get\","
	     "\"params\":{\"id\":\"" PANEL_ID "\"}}");
	TEST_ASSERT_TRUE(res_b("value"));
	TEST_ASSERT_TRUE(mp_json_streq(&g_rp,
				       mp_json_obj_get(&g_rp, result(), "source"),
				       "override"));
}

/**
 * The dead-man reverts it, and the revert goes through the SAME mailbox.
 *
 * Not a separate path: lease_drop() calls the same apply callback with NULL,
 * which posts a release. If the revert wrote the pin directly it would be the
 * single-writer violation again, on the safety path this time — so the
 * assertion is that the pin is still lit until the drain runs.
 */
static void test_the_deadman_reverts_the_pin_through_the_same_mailbox(void)
{
	uint32_t sid = session();

	override_panel(sid, true);
	hk_pass();
	console_pass();
	TEST_ASSERT_TRUE(pin_on());

	/* The cable is pulled. FMT §5.4: the link is the outermost condition. */
	TEST_ASSERT_EQUAL_INT(0, mp_set_link(&g_c, false));

	/* The lease is gone immediately — a lost link is unambiguous — but the
	 * PIN cannot be, because this is not the thread that owns it. */
	TEST_ASSERT_NULL(mp_ovr_lease(&g_c.ovr, panel_obj()));
	TEST_ASSERT_EQUAL_UINT(1U, g_release_calls);
	TEST_ASSERT_TRUE_MESSAGE(pin_on(),
				 "the revert wrote the rail pin from the "
				 "console thread");
	TEST_ASSERT_TRUE(sts_pwrseq_reqq_pending(
		&g_mbox, (uint8_t)STS_PWRSEQ_REQ_PANEL_LED_EN));

	hk_pass();
	TEST_ASSERT_FALSE_MESSAGE(pin_on(),
				  "the dead-man's release never reached the "
				  "pin: the panel stays lit after the tool "
				  "walks away");
}

/**
 * A lease that is never drained is dropped rather than left standing.
 *
 * Silence is failure. Without this the mailbox would have a mode in which a
 * wedged housekeeping thread leaves a lease asserting a pin state that never
 * happened, for the whole 60 s TTL.
 */
static void test_a_request_that_never_drains_drops_the_lease(void)
{
	uint32_t sid = session();
	unsigned int i;

	override_panel(sid, true);
	TEST_ASSERT_TRUE(mp_ovr_pending(&g_c.ovr, panel_obj()));

	/* Housekeeping never runs. The console keeps ticking and keeps the
	 * session fresh, so the dead-man is NOT what drops this. */
	for (i = 0U; i < 16U; i++) {
		g_now += HK_TICK_MS;
		TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c.ovr, sid, g_now));
		console_pass();
	}

	TEST_ASSERT_NULL_MESSAGE(mp_ovr_lease(&g_c.ovr, panel_obj()),
				 "a lease whose write never reached the pin "
				 "stood anyway");
	TEST_ASSERT_EQUAL_UINT32(1U, g_c.ovr.settle_failures);
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.ovr.deadman_reverts);
}

/**
 * The settle deadline is longer than the worst latency the shipped glue can
 * produce, so a merely busy board cannot lose a working lease.
 *
 * mp_glue.h budgets `(STS_MP_TICK_MISS_MAX + 1) * (250 + 50)` = 1800 ms between
 * two successful console ticks, on top of one 250 ms sequencer pass. A deadline
 * at or below that would revert leases on a contended board — a failure that
 * would look exactly like a hardware fault.
 */
static void test_the_settle_deadline_clears_the_worst_console_latency(void)
{
	TEST_ASSERT_TRUE_MESSAGE(MP_APPLY_SETTLE_MS > (HK_TICK_MS + 1800U),
				 "MP_APPLY_SETTLE_MS is inside the tick "
				 "budget mp_glue.h already permits");
	/* ...and still far inside the shortest lease it could cut short. */
	TEST_ASSERT_TRUE(MP_APPLY_SETTLE_MS < MP_LEASE_TTL_DEFAULT_MS);
}

/* ============================================== 2. the collision definition = */

/**
 * TWO REQUESTS, ONE DRAIN: last writer wins, and the loser is counted.
 *
 * Both halves matter. Without the value assertion the rule is unproven; without
 * the counter assertion "last writer wins" is indistinguishable from "the first
 * request was silently dropped", which is the outcome the design forbids.
 */
static void test_two_requests_for_one_object_coalesce_to_the_last(void)
{
	uint32_t sid = session();

	/* on, then off, with no drain in between. */
	override_panel(sid, true);
	TEST_ASSERT_TRUE(res_b("verify_pending"));
	override_panel(sid, false);
	TEST_ASSERT_TRUE(res_b("verify_pending"));

	TEST_ASSERT_EQUAL_UINT(2U, g_apply_calls);
	TEST_ASSERT_EQUAL_UINT32(2U, g_mbox.posted);
	TEST_ASSERT_EQUAL_UINT32_MESSAGE(
		1U, g_mbox.coalesced,
		"the replaced request was not counted, so a dropped request "
		"and a revised one look the same");

	/* One drain, and the pin lands on the SECOND value. */
	hk_pass();
	TEST_ASSERT_FALSE_MESSAGE(pin_on(),
				  "the drain applied the superseded value");
	TEST_ASSERT_EQUAL_UINT32(1U, g_mbox.applied);

	/* And exactly one drain happened: the queue is empty, not backlogged. */
	TEST_ASSERT_FALSE(sts_pwrseq_reqq_pending(
		&g_mbox, (uint8_t)STS_PWRSEQ_REQ_PANEL_LED_EN));

	console_pass();
	TEST_ASSERT_FALSE(mp_ovr_pending(&g_c.ovr, panel_obj()));
}

/**
 * Requests for DIFFERENT objects never displace each other, and the table
 * cannot overflow.
 *
 * This is the property that makes "no bounded-queue drop rule" true rather than
 * merely untested: capacity is the number of requestable objects, so every id
 * has a slot by construction. Written over the whole enum so it keeps holding
 * as stage 2 adds rows.
 */
static void test_every_request_id_has_its_own_slot(void)
{
	uint8_t req;
	unsigned int n = 0U;

	for (req = (uint8_t)STS_PWRSEQ_REQ_NONE + 1U;
	     req < (uint8_t)STS_PWRSEQ_REQ_COUNT; req++) {
		TEST_ASSERT_EQUAL_INT(0, sts_pwrseq_reqq_post(&g_mbox, req,
							      false, 1, g_now));
		n++;
	}
	TEST_ASSERT_TRUE_MESSAGE(n > 0U, "no request ids to test");

	/* Every one of them is still pending: none displaced another. */
	for (req = (uint8_t)STS_PWRSEQ_REQ_NONE + 1U;
	     req < (uint8_t)STS_PWRSEQ_REQ_COUNT; req++) {
		TEST_ASSERT_TRUE(sts_pwrseq_reqq_pending(&g_mbox, req));
	}
	TEST_ASSERT_EQUAL_UINT32(0U, g_mbox.coalesced);
	TEST_ASSERT_EQUAL_UINT32((uint32_t)n, g_mbox.posted);
}

/** Slot 0 is never a request, and neither is anything past the table. */
static void test_the_mailbox_refuses_a_non_request_id(void)
{
	sts_pwrseq_req_result_t r;

	TEST_ASSERT_FALSE(sts_pwrseq_req_id_ok((uint8_t)STS_PWRSEQ_REQ_NONE));
	TEST_ASSERT_FALSE(sts_pwrseq_req_id_ok((uint8_t)STS_PWRSEQ_REQ_COUNT));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      sts_pwrseq_reqq_post(&g_mbox,
						   (uint8_t)STS_PWRSEQ_REQ_NONE,
						   false, 1, g_now));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      sts_pwrseq_reqq_post(&g_mbox,
						   (uint8_t)STS_PWRSEQ_REQ_COUNT,
						   false, 1, g_now));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      sts_pwrseq_reqq_post(NULL,
						   (uint8_t)STS_PWRSEQ_REQ_PANEL_LED_EN,
						   false, 1, g_now));
	TEST_ASSERT_FALSE(sts_pwrseq_reqq_take(&g_mbox,
					       (uint8_t)STS_PWRSEQ_REQ_NONE, &r));
	TEST_ASSERT_EQUAL_UINT32(0U, g_mbox.posted);
}

/**
 * A drain cannot settle a request into nothing.
 *
 * STS_PWRSEQ_REQ_OUT_NONE means "nothing unread"; accepting it as an outcome
 * would let a drain consume a request and report neither success nor failure,
 * which is precisely the silent disappearance the mailbox exists to prevent.
 */
static void test_a_drain_cannot_settle_a_request_into_silence(void)
{
	TEST_ASSERT_EQUAL_INT(0,
		sts_pwrseq_reqq_post(&g_mbox,
				     (uint8_t)STS_PWRSEQ_REQ_PANEL_LED_EN,
				     false, 1, g_now));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
		sts_pwrseq_reqq_settle(&g_mbox,
				       (uint8_t)STS_PWRSEQ_REQ_PANEL_LED_EN,
				       (uint8_t)STS_PWRSEQ_REQ_OUT_NONE,
				       (uint8_t)STS_PWRSEQ_REQ_ERR_NONE, 0,
				       g_now));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		sts_pwrseq_reqq_settle(&g_mbox,
				       (uint8_t)STS_PWRSEQ_REQ_PANEL_LED_EN,
				       (uint8_t)STS_PWRSEQ_REQ_OUT_COUNT,
				       (uint8_t)STS_PWRSEQ_REQ_ERR_NONE, 0,
				       g_now));
}

/** An outcome is taken once: reporting it twice would withdraw two leases. */
static void test_an_outcome_is_taken_exactly_once(void)
{
	sts_pwrseq_req_result_t r;

	TEST_ASSERT_EQUAL_INT(0,
		sts_pwrseq_reqq_settle(&g_mbox,
				       (uint8_t)STS_PWRSEQ_REQ_PANEL_LED_EN,
				       (uint8_t)STS_PWRSEQ_REQ_OUT_APPLIED,
				       (uint8_t)STS_PWRSEQ_REQ_ERR_NONE, 1,
				       g_now));

	TEST_ASSERT_TRUE(sts_pwrseq_reqq_take(
		&g_mbox, (uint8_t)STS_PWRSEQ_REQ_PANEL_LED_EN, &r));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PWRSEQ_REQ_OUT_APPLIED, r.outcome);
	TEST_ASSERT_EQUAL_INT32(1, r.value);

	TEST_ASSERT_FALSE(sts_pwrseq_reqq_take(
		&g_mbox, (uint8_t)STS_PWRSEQ_REQ_PANEL_LED_EN, &r));
}

/* ==================================================== 3. refusal reaches out */

/**
 * A REFUSAL REACHES THE CALLER, through the veto path a shed already uses.
 *
 * The sequencer has shed the panel rung. The override is granted (the guard and
 * the interlocks permit it) and reported pending — and then the drain refuses
 * it, the console maps that onto mp_ovr_veto(), the lease goes, and the event
 * carries the sequencer's own sentence rather than a generic failure.
 */
static void test_a_refused_request_withdraws_the_lease_with_its_reason(void)
{
	uint32_t sid = session();
	uint32_t vetoes_before;

	TEST_ASSERT_EQUAL_INT(0, mp_stream_sub(&g_c.st, (uint8_t)MP_CH_EVENT, 0U,
					       g_now));

	g_shed = 2U; /* PWRSEQ_SHED_PANEL_LED: the load is off, by decision. */
	vetoes_before = g_c.ovr.vetoes;

	override_panel(sid, true);
	TEST_ASSERT_TRUE(res_b("verify_pending"));
	TEST_ASSERT_NOT_NULL(mp_ovr_lease(&g_c.ovr, panel_obj()));

	hk_pass();
	TEST_ASSERT_FALSE_MESSAGE(pin_on(),
				  "a shed load was re-energised by an override");
	TEST_ASSERT_EQUAL_UINT32(1U, g_mbox.refused);

	/*
	 * The outcome drain on its own, so the event it raises can be observed
	 * before mp_tick() encodes the queue onto the wire and empties it.
	 */
	req_drain();
	TEST_ASSERT_NULL_MESSAGE(mp_ovr_lease(&g_c.ovr, panel_obj()),
				 "the refusal never reached the lease table");
	TEST_ASSERT_EQUAL_UINT32(vetoes_before + 1U, g_c.ovr.vetoes);
	/* The refusal is an EVENT, not merely a counter: a technician watching
	 * channel 0x09 is told why the override went away. */
	TEST_ASSERT_TRUE_MESSAGE(mp_stream_event_count(&g_c.st) > 0U,
				 "the refusal withdrew the lease silently");

	/* ...and it survives the encode, so it genuinely reaches the host. */
	(void)mp_tick(&g_c);
	TEST_ASSERT_TRUE(g_wire_len > 0U);
}

/** The stage refusal is a DIFFERENT sentence from the shed refusal. */
static void test_the_refusal_reasons_are_distinct_and_fit_the_event_record(void)
{
	static const uint8_t errs[] = {
		(uint8_t)STS_PWRSEQ_REQ_ERR_STAGE,
		(uint8_t)STS_PWRSEQ_REQ_ERR_SHED,
		(uint8_t)STS_PWRSEQ_REQ_ERR_HW,
	};
	size_t i;
	size_t j;

	for (i = 0U; i < (sizeof(errs) / sizeof(errs[0])); i++) {
		const char *a = sts_pwrseq_req_reason_of(errs[i]);

		TEST_ASSERT_NOT_NULL_MESSAGE(a, "a refusal with no sentence");
		TEST_ASSERT_TRUE_MESSAGE(
			strlen(a) <= (size_t)(MP_EV_TEXT_MAX - 1U),
			"a refusal reason is truncated on channel 0x09");
		for (j = i + 1U; j < (sizeof(errs) / sizeof(errs[0])); j++) {
			TEST_ASSERT_TRUE_MESSAGE(
				strcmp(a, sts_pwrseq_req_reason_of(errs[j])) != 0,
				"two refusal reasons read the same, so a "
				"technician cannot tell them apart");
		}
	}

	/* Not-a-reason has no sentence, so a bad enum cannot be reported as one. */
	TEST_ASSERT_NULL(sts_pwrseq_req_reason_of(
		(uint8_t)STS_PWRSEQ_REQ_ERR_NONE));
	TEST_ASSERT_NULL(sts_pwrseq_req_reason_of(
		(uint8_t)STS_PWRSEQ_REQ_ERR_COUNT));
}

/**
 * The OFF direction is never refused, even from a shed or an unreached stage.
 *
 * This is what makes a lapsed lease reliable: the dead-man's release must land
 * whatever the board is doing. A drain that refused an off would leave the
 * panel lit with no lease and nothing left to turn it off.
 */
static void test_the_fail_safe_direction_is_never_refused(void)
{
	uint8_t err = 0U;
	int32_t val = 0;

	g_duty = 100U;
	g_shed = 3U;              /* everything shed */
	g_panel_stage_done = false; /* and the stage never reached */

	TEST_ASSERT_TRUE(apply_panel(true, 0, &err, &val));
	TEST_ASSERT_FALSE(pin_on());
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PWRSEQ_REQ_ERR_NONE, err);

	g_duty = 100U;
	TEST_ASSERT_TRUE(apply_panel(false, 0, &err, &val));
	TEST_ASSERT_FALSE(pin_on());
}

/** An actuator that fails is a refusal, not a silent success. */
static void test_an_actuator_error_is_reported_as_a_refusal(void)
{
	uint8_t err = 0U;
	int32_t val = 0;

	g_hw_fails = true;
	TEST_ASSERT_FALSE(apply_panel(false, 1, &err, &val));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PWRSEQ_REQ_ERR_HW, err);

	/* Including on the release path, where it matters most. */
	TEST_ASSERT_FALSE(apply_panel(true, 0, &err, &val));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PWRSEQ_REQ_ERR_HW, err);
}

/* ================================== 4. the manifest contract for this object */

/**
 * `pwr.panel.led.en` is lease-only: overridable, never writable.
 *
 * Two independent reasons, both in its manifest row. An `obj.set` creates no
 * lease, so nothing — not the dead-man, not a link drop — could put the rail
 * back; and the apply is asynchronous, so `obj.set`'s reply, which has no
 * `verify_pending` field, could only report an effect that had not happened.
 */
static void test_every_mailbox_object_is_lease_only(void)
{
	uint32_t sid = session();
	uint8_t req;

	for (req = (uint8_t)STS_PWRSEQ_REQ_NONE + 1U;
	     req < (uint8_t)STS_PWRSEQ_REQ_COUNT; req++) {
		const char *id = g_req_objects[req];
		int idx = mp_obj_find(id);
		const mp_obj_t *o;
		char msg[128];
		char rq[256];

		TEST_ASSERT_TRUE_MESSAGE(idx >= 0, id);
		o = mp_obj_at((size_t)idx);

		(void)snprintf(msg, sizeof(msg), "%s", id);
		TEST_ASSERT_EQUAL_UINT_MESSAGE(
			0U, (unsigned int)(o->flags & MP_OF_WRITE), msg);
		TEST_ASSERT_TRUE_MESSAGE((o->flags & MP_OF_OVERRIDE) != 0U, msg);
		TEST_ASSERT_EQUAL_UINT_MESSAGE(
			0U, (unsigned int)(o->flags & MP_OF_DEFERRED), msg);
		/* Readable, or the asynchronous write could not be confirmed. */
		TEST_ASSERT_TRUE_MESSAGE((o->flags & MP_OF_READ) != 0U, msg);

		/* And the method is really refused on the wire, not merely
		 * unadvertised. G2 rows would fail the guard first, so the
		 * serial is supplied and the answer must still be NOTSUP. */
		(void)snprintf(rq, sizeof(rq),
			       "{\"jsonrpc\":\"2.0\",\"id\":3,"
			       "\"method\":\"obj.set\",\"params\":{\"id\":\"%s\","
			       "\"value\":1,\"sid\":%u,\"confirm\":\"%s\"}}",
			       id, (unsigned int)sid, SERIAL);
		call(rq);
		TEST_ASSERT_EQUAL_INT64_MESSAGE(MP_E_NOTSUP, err_code(), msg);
	}

	/* Not vacuous: the loop has to have covered the whole mailbox. */
	TEST_ASSERT_EQUAL_UINT(15U, (unsigned int)STS_PWRSEQ_REQ_COUNT - 1U);
}

/**
 * A glue that answers MP_APPLY_PENDING on a WRITABLE object is refused rather
 * than believed.
 *
 * The forcing function for stage 2: wiring an asynchronous actuator behind an
 * object that kept MP_OF_WRITE fails loudly at the bench instead of quietly
 * reporting success. `ui.panel.duty` is writable and its apply here returns 0,
 * so it still works — the refusal is about the pending answer, not the method.
 */
static void test_a_pending_apply_is_refused_on_obj_set(void)
{
	uint32_t sid = session();
	char req[256];

	/*
	 * `ui.identify` is G1, still WRITABLE, and its apply here answers 0 —
	 * so `obj.set` works on it and the method itself is not what is being
	 * refused elsewhere.
	 *
	 * The vehicle changed with this stage: it used to be `ui.panel.duty`,
	 * which was writable precisely because it wrote PE0 from the console
	 * thread. That is the defect the migration closed, so the object can no
	 * longer play the "synchronous apply" role.
	 */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"ui.identify\",\"value\":1,"
		       "\"sid\":%u}}",
		       (unsigned int)sid);
	call(req);
	TEST_ASSERT_EQUAL_INT64(1, res_i("value"));

	/*
	 * And the refusal that IS about the pending answer: `g_force_pending`
	 * makes the same object's apply answer MP_APPLY_PENDING with nothing
	 * else changed, so the only difference between the two calls is the
	 * return value. m_obj_set() must refuse it by name rather than report
	 * an effect that has not happened.
	 */
	g_force_pending = true;
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"ui.identify\",\"value\":0,"
		       "\"sid\":%u}}",
		       (unsigned int)sid);
	call(req);
	TEST_ASSERT_EQUAL_INT64_MESSAGE(
		MP_E_NOTSUP, err_code(),
		"obj.set answered an asynchronous apply instead of refusing it");
}

/* ================================ 4b. every object stage 2 wired, end to end */

/** `obj.override <id> = <on>` with the device serial, through the real RPC. */
static void override_obj(uint32_t sid, const char *id, int value)
{
	char req[320];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":7,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"%s\",\"value\":%d,\"sid\":%u,"
		       "\"confirm\":\"%s\"}}",
		       id, value, (unsigned int)sid, SERIAL);
	call(req);
}

static size_t obj_of(const char *id)
{
	int obj = mp_obj_find(id);

	TEST_ASSERT_TRUE_MESSAGE(obj >= 0, id);
	return (size_t)obj;
}

/**
 * THE STAGE-2 ASSERTION, for every GPIO rail this change wired.
 *
 * One loop, four objects, and the same four facts each time: the reply is
 * pending and the PIN HAS NOT MOVED when it is produced; the housekeeping drain
 * moves it; the console pass confirms it; and a release puts the pin back to
 * FIRMWARE'S OWN level rather than to off.
 *
 * That last one is the property a panel-shaped test would miss entirely. The
 * panel's release resolves to dark and is safe there; a rail's must not, or a
 * dead-man on `pwr.gps.en` would leave the receiver unpowered and nothing in
 * the system would ever turn it back on.
 */
static void test_every_wired_rail_reaches_its_pin_and_comes_back(void)
{
	static const char *const ids[] = {
		"pwr.gps.en",
		"pwr.ant.bias.en",
		"pwr.disp.en",
		"pwr.rb.gate",
	};
	size_t k;

	for (k = 0U; k < (sizeof(ids) / sizeof(ids[0])); k++) {
		const char *id = ids[k];
		uint8_t req = req_of(id);
		size_t obj = obj_of(id);
		uint32_t sid;

		setUp();
		sid = session();

		/* The board starts with firmware commanding the rail on. */
		TEST_ASSERT_TRUE_MESSAGE(g_rail[req].pin, id);

		/* --- the OFF direction, which is the diagnostic one --------- */
		override_obj(sid, id, 0);
		TEST_ASSERT_EQUAL_INT64_MESSAGE(0, res_i("value"), id);
		TEST_ASSERT_TRUE_MESSAGE(
			res_b("verify_pending"),
			"the reply claimed the pin had moved before the "
			"sequencer had touched it");
		TEST_ASSERT_TRUE_MESSAGE(
			g_rail[req].pin,
			"the console thread wrote the rail pin itself; that is "
			"the single-writer violation this mailbox prevents");

		hk_pass();
		TEST_ASSERT_FALSE_MESSAGE(g_rail[req].pin,
					  "the drain did not move the pin");
		/* Firmware's own level is untouched by the override — that is
		 * what makes the release below meaningful. */
		TEST_ASSERT_TRUE_MESSAGE(g_rail[req].automatic, id);

		console_pass();
		TEST_ASSERT_FALSE(mp_ovr_pending(&g_c.ovr, obj));
		TEST_ASSERT_NOT_NULL(mp_ovr_lease(&g_c.ovr, obj));

		/* The read-back reports the PIN, so the host can confirm it. */
		{
			char rq[192];

			(void)snprintf(rq, sizeof(rq),
				       "{\"jsonrpc\":\"2.0\",\"id\":8,"
				       "\"method\":\"obj.get\",\"params\":"
				       "{\"id\":\"%s\"}}", id);
			call(rq);
			TEST_ASSERT_FALSE_MESSAGE(res_b("value"), id);
		}

		/* --- the release returns it to FIRMWARE, not to off --------- */
		TEST_ASSERT_EQUAL_INT(0, mp_ovr_release(&g_c.ovr, obj, sid,
							g_now));
		hk_pass();
		TEST_ASSERT_TRUE_MESSAGE(
			g_rail[req].pin,
			"a released lease left the rail off; a dead-man that "
			"unpowers the receiver has broken the box it protects");
	}
}

/**
 * Re-powering the GPS rail tells gnssmgr its receiver restarted.
 *
 * PWRSEQ_ACT_GPS_RST_RELEASE says why for the sequencer's own path: the
 * receiver's configuration is gone with the power, and gnssmgr must know before
 * it believes NAV messages from the new session. A rail an override dropped and
 * restored is the same event, and nothing else in the system would notice it.
 */
static void test_restoring_the_gps_rail_notifies_the_receiver(void)
{
	uint32_t sid = session();
	uint8_t req = req_of("pwr.gps.en");

	override_obj(sid, "pwr.gps.en", 0);
	hk_pass();
	console_pass();
	TEST_ASSERT_FALSE(g_rail[req].pin);
	TEST_ASSERT_EQUAL_UINT(0U, g_gnss_reset_notes);

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_release(&g_c.ovr, obj_of("pwr.gps.en"),
						sid, g_now));
	hk_pass();
	TEST_ASSERT_TRUE(g_rail[req].pin);
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, g_gnss_reset_notes,
		"the receiver was re-powered without gnssmgr being told, so it "
		"keeps trusting a configuration that no longer exists");

	/* And not on a no-op: a request that does not move the pin is not a
	 * reset, or every drain would invalidate the receiver's state. */
	override_obj(sid, "pwr.gps.en", 1);
	hk_pass();
	TEST_ASSERT_EQUAL_UINT(1U, g_gnss_reset_notes);
}

/**
 * AN OVERRIDE MAY NOT RAISE A LOAD FIRMWARE HAS TAKEN DOWN — the refusal path.
 *
 * The shed ladder has dropped the display. The guard and the interlocks permit
 * the override (MP_ILK_DISP_OFF is satisfied — the rail has been off long
 * enough), the lease is granted and reported pending, and then the drain
 * refuses it with the SHED sentence rather than the STAGE one, the console maps
 * that onto mp_ovr_veto(), and the lease goes.
 */
static void test_a_shed_rail_refuses_an_override_with_the_shed_reason(void)
{
	uint32_t sid = session();
	uint8_t req = req_of("pwr.disp.en");
	size_t obj = obj_of("pwr.disp.en");
	uint32_t vetoes_before;

	TEST_ASSERT_EQUAL_INT(0, mp_stream_sub(&g_c.st, (uint8_t)MP_CH_EVENT, 0U,
					       g_now));

	/* Rung 1: firmware has dropped the display and no longer commands it. */
	g_shed = 1U; /* PWRSEQ_SHED_DISPLAY */
	g_rail[req].automatic = false;
	g_rail[req].pin = false;
	/* The off-time interlock is satisfied, so the refusal under test is the
	 * sequencer's and not the interlock's. */
	g_ilk.disp_on = false;
	g_ilk.disp_changed_ms = g_now - (MP_DISP_MIN_OFF_MS + 1000U);

	vetoes_before = g_c.ovr.vetoes;

	override_obj(sid, "pwr.disp.en", 1);
	TEST_ASSERT_TRUE(res_b("verify_pending"));
	TEST_ASSERT_NOT_NULL(mp_ovr_lease(&g_c.ovr, obj));

	hk_pass();
	TEST_ASSERT_FALSE_MESSAGE(g_rail[req].pin,
				  "a shed load was re-energised by an override");
	TEST_ASSERT_EQUAL_UINT32(1U, g_mbox.refused);

	req_drain();
	TEST_ASSERT_NULL_MESSAGE(mp_ovr_lease(&g_c.ovr, obj),
				 "the refusal never reached the lease table");
	TEST_ASSERT_EQUAL_UINT32(vetoes_before + 1U, g_c.ovr.vetoes);
	TEST_ASSERT_TRUE_MESSAGE(mp_stream_event_count(&g_c.st) > 0U,
				 "the refusal withdrew the lease silently");
}

/**
 * A DEAD-MAN REVERT ON A RAIL RESTORES FIRMWARE'S LEVEL, through the mailbox.
 *
 * The cable is pulled while a technician holds the antenna bias off. The lease
 * goes immediately — a lost link is unambiguous — but the PIN cannot, because
 * this is not the thread that owns it; and when the drain does run, the bias
 * comes back, because that is what firmware commands.
 */
static void test_the_deadman_restores_a_rail_through_the_mailbox(void)
{
	uint32_t sid = session();
	uint8_t req = req_of("pwr.ant.bias.en");

	override_obj(sid, "pwr.ant.bias.en", 0);
	hk_pass();
	console_pass();
	TEST_ASSERT_FALSE(g_rail[req].pin);

	/* FMT §5.4: the link is the dead-man's outermost condition. */
	TEST_ASSERT_EQUAL_INT(0, mp_set_link(&g_c, false));
	TEST_ASSERT_NULL(mp_ovr_lease(&g_c.ovr, obj_of("pwr.ant.bias.en")));
	TEST_ASSERT_FALSE_MESSAGE(g_rail[req].pin,
				  "the revert wrote the rail pin from the "
				  "console thread");
	TEST_ASSERT_TRUE(sts_pwrseq_reqq_pending(&g_mbox, req));

	hk_pass();
	TEST_ASSERT_TRUE_MESSAGE(
		g_rail[req].pin,
		"the dead-man left the antenna unbiased: the tool walked away "
		"and took the GNSS reference with it");
}

/**
 * The antenna supervisor's own cut is what a release restores, not pwrseq's
 * stale belief.
 *
 * gnssmgr latches a short and cuts the bias. pwrseq's `ant_bias_on` does not
 * move — it never sees that decision — so a release that restored pwrseq's
 * model would re-energise the bias into the short. `automatic` is the record
 * the supervisor updates, and it is what the drain reads.
 */
static void test_a_release_after_a_latched_short_leaves_the_bias_off(void)
{
	uint32_t sid = session();
	uint8_t req = req_of("pwr.ant.bias.en");

	override_obj(sid, "pwr.ant.bias.en", 1);
	hk_pass();
	console_pass();
	TEST_ASSERT_TRUE(g_rail[req].pin);

	/* sts_pwrseq_ant_bias_request(false): the supervisor's decision, which
	 * updates firmware's commanded level as well as the pin. */
	g_rail[req].automatic = false;
	g_rail[req].pin = false;

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_release(&g_c.ovr,
						obj_of("pwr.ant.bias.en"), sid,
						g_now));
	hk_pass();
	TEST_ASSERT_FALSE_MESSAGE(
		g_rail[req].pin,
		"a lapsed lease re-energised the antenna bias into a latched "
		"short");
}

/**
 * `ui.panel.duty`, migrated: the same object, a later write, and one refusal it
 * did not have.
 */
static void test_the_panel_dimmer_is_now_pending_and_refuses_a_shed_panel(void)
{
	uint32_t sid = session();
	size_t obj = obj_of("ui.panel.duty");

	override_obj(sid, "ui.panel.duty", 60);
	TEST_ASSERT_EQUAL_INT64(60, res_i("value"));
	TEST_ASSERT_TRUE_MESSAGE(
		res_b("verify_pending"),
		"the dimmer's reply still claims the duty took effect on the "
		"console thread");
	TEST_ASSERT_EQUAL_UINT8_MESSAGE(0U, g_duty,
					"the console thread wrote PE0 itself");

	hk_pass();
	TEST_ASSERT_EQUAL_UINT8(60U, g_duty);
	console_pass();
	TEST_ASSERT_FALSE(mp_ovr_pending(&g_c.ovr, obj));

	/* The refusal it gained: a shed panel is not re-lit by a dimmer write.
	 * Before the migration this landed on PE0 unconditionally. */
	g_shed = 2U; /* PWRSEQ_SHED_PANEL_LED */
	override_obj(sid, "ui.panel.duty", 90);
	hk_pass();
	TEST_ASSERT_EQUAL_UINT8_MESSAGE(
		60U, g_duty,
		"a shed panel was re-lit by a dimmer override");
	req_drain();
	TEST_ASSERT_NULL(mp_ovr_lease(&g_c.ovr, obj));
}

/**
 * THE VCC_RB SETPOINT, and the two refusals that make it offerable at all.
 *
 * The root CLAUDE.md: a digipot in this buck's feedback node can destroy the
 * FE-5680A. So the setpoint is permitted only where it cannot reach the FE and
 * cannot fight the sequencer — and both refusals are asserted here against the
 * REAL transfer function, not a modelled one.
 */
static void test_the_vcc_rb_setpoint_is_refused_while_the_fe_is_connected(void)
{
	uint32_t sid = session();
	size_t obj = obj_of("pwr.rb.vset_mv");
	uint16_t code_before;

	/* The FE is gated onto the rail. */
	g_rb_gated = true;
	code_before = g_dp_code;

	override_obj(sid, "pwr.rb.vset_mv", 12000);
	TEST_ASSERT_TRUE(res_b("verify_pending"));

	hk_pass();
	TEST_ASSERT_EQUAL_UINT16_MESSAGE(
		code_before, g_dp_code,
		"a maintenance setpoint moved the wiper with the FE-5680A "
		"connected to the rail");
	TEST_ASSERT_EQUAL_UINT32(1U, g_mbox.refused);

	req_drain();
	TEST_ASSERT_NULL(mp_ovr_lease(&g_c.ovr, obj));

	/* And the sentence is its own, not the stage one: they send a
	 * technician to different places. */
	TEST_ASSERT_EQUAL_STRING(
		"FE is connected to this rail",
		sts_pwrseq_req_reason_of((uint8_t)STS_PWRSEQ_REQ_ERR_GATED));

	/* The guarded sequence owns the wiper too. */
	setUp();
	sid = session();
	g_in_stage8 = true;
	code_before = g_dp_code;
	override_obj(sid, "pwr.rb.vset_mv", 12000);
	hk_pass();
	TEST_ASSERT_EQUAL_UINT16_MESSAGE(
		code_before, g_dp_code,
		"a setpoint landed inside the guarded rubidium sequence");
}

/**
 * With the FE disconnected it works, lands on a code that does NOT overshoot,
 * and a release puts firmware's own code back.
 */
static void test_the_vcc_rb_setpoint_lands_below_the_request_and_reverts(void)
{
	uint32_t sid = session();
	size_t obj = obj_of("pwr.rb.vset_mv");
	uint16_t auto_code = g_dp_auto_code;
	int32_t landed;

	g_rb_gated = false;

	override_obj(sid, "pwr.rb.vset_mv", 12000);
	TEST_ASSERT_TRUE(res_b("verify_pending"));
	TEST_ASSERT_EQUAL_UINT16_MESSAGE(auto_code, g_dp_code,
					 "the console thread wrote the digipot");

	hk_pass();
	landed = pwrseq_rb_expected_mv(&g_xfer, g_dp_code);
	TEST_ASSERT_TRUE_MESSAGE(landed <= 12000,
				 "the wiper overshot the requested rail");
	TEST_ASSERT_TRUE_MESSAGE(landed > 11900,
				 "the wiper undershot by more than one step");

	/* The buck follows the wiper, so MP_ILK_RB_VERIFY's read-back — which
	 * is due by now — confirms rather than reverts. The next test is the
	 * case where it does not. */
	g_ilk.vcc_rb_mv = landed;
	console_pass();
	TEST_ASSERT_FALSE(mp_ovr_pending(&g_c.ovr, obj));

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_release(&g_c.ovr, obj, sid, g_now));
	hk_pass();
	TEST_ASSERT_EQUAL_UINT16_MESSAGE(
		auto_code, g_dp_code,
		"a lapsed setpoint lease left the rubidium rail at the "
		"technician's voltage");
}

/**
 * MP_ILK_RB_VERIFY still auto-reverts a setpoint the rail does not follow.
 *
 * The brief's standing requirement, and the reason the setpoint object can be
 * offered where the raw wiper code cannot: the rail is read back on INA228 0x47
 * and a mismatch drops the lease. Nothing in this change may weaken it.
 */
static void test_a_setpoint_the_rail_does_not_follow_is_reverted(void)
{
	uint32_t sid = session();
	size_t obj = obj_of("pwr.rb.vset_mv");

	g_rb_gated = false;

	override_obj(sid, "pwr.rb.vset_mv", 12000);
	TEST_ASSERT_NOT_NULL(mp_ovr_lease(&g_c.ovr, obj));

	/*
	 * The drain writes the wiper but the buck does NOT follow it: VCC_RB
	 * stays where it was. Nothing in the mailbox path can detect that — the
	 * SPI write and its read-back both succeed — so the only thing standing
	 * between a technician and a rail that is not where they set it is
	 * MP_ILK_RB_VERIFY reading INA228 0x47 back.
	 */
	hk_pass();
	TEST_ASSERT_EQUAL_INT32_MESSAGE(
		pwrseq_rb_expected_mv(&g_xfer, g_dp_auto_code), g_ilk.vcc_rb_mv,
		"the fixture moved the rail, so this proves nothing");

	console_pass();

	TEST_ASSERT_NULL_MESSAGE(mp_ovr_lease(&g_c.ovr, obj),
				 "the VCC_RB read-back no longer reverts a "
				 "setpoint the rail did not follow");
	TEST_ASSERT_EQUAL_UINT32(1U, g_c.ovr.verify_failures);

	/* And the revert put firmware's own code back at the part. */
	hk_pass();
	TEST_ASSERT_EQUAL_UINT16(g_dp_auto_code, g_dp_code);
}

/**
 * The inverse never returns a code above the ceiling — swept, not sampled.
 *
 * This is the one arithmetic error in this change that could destroy hardware,
 * so it is checked across the whole published envelope rather than at the two
 * or three points a hand-written case would pick.
 */
static void test_no_requested_setpoint_can_exceed_the_configured_ceiling(void)
{
	static const uint32_t ceilings[] = { 5000U, 6000U, 12000U, 15000U };
	size_t c;
	int32_t mv;

	for (c = 0U; c < (sizeof(ceilings) / sizeof(ceilings[0])); c++) {
		for (mv = 0; mv <= 30000; mv += 37) {
			uint16_t code = sts_rb_code_for_mv(&g_xfer, mv,
							   ceilings[c]);
			int32_t got = pwrseq_rb_expected_mv(&g_xfer, code);
			char msg[128];

			(void)snprintf(msg, sizeof(msg),
				       "ceiling %u mV, request %d mV -> code %u "
				       "= %d mV",
				       (unsigned int)ceilings[c], (int)mv,
				       (unsigned int)code, (int)got);
			TEST_ASSERT_TRUE_MESSAGE(
				got <= (int32_t)ceilings[c], msg);
			/* ...and never above the request either. */
			if (mv >= pwrseq_rb_expected_mv(&g_xfer, 0U)) {
				TEST_ASSERT_TRUE_MESSAGE(got <= mv, msg);
			}
		}
	}

	/* A degenerate transfer function answers terminal B, the minimum rail —
	 * the same fail-safe pwrseq_digipot_vctrl_mv() applies to a bad code. */
	{
		pwrseq_rb_xfer_t bad = g_xfer;

		bad.steps = 0U;
		TEST_ASSERT_EQUAL_UINT16(0U,
					 sts_rb_code_for_mv(&bad, 15000, 15000U));
		TEST_ASSERT_EQUAL_UINT16(0U,
					 sts_rb_code_for_mv(NULL, 15000, 15000U));
	}
}

/* ===================================================== 5. the shipped source */

/*
 * mp_glue.c, pwrseq_exec.c and mp_rpc.c are read below, brace-matched and
 * function-scoped. A behavioural test cannot reach them: mp_glue.c and
 * pwrseq_exec.c are Zephyr translation units no host suite links, and the
 * mirrors above are this file's own code. What has to be true of the IMAGE is
 * that those files still make these calls.
 */

#define SRC_MAX (256U * 1024U)

static char g_src[SRC_MAX];
static size_t g_src_len;
static char g_src_name[128];

typedef struct {
	size_t begin;
	size_t end;
} span_t;

/** Read @p rel into g_src, blanking comments in place so offsets stay 1:1. */
static void load_source(const char *rel)
{
	char path[512];
	FILE *f;
	size_t n;
	size_t i;
	enum { CODE, BLOCK, LINE, STR, CHR } st = CODE;

	(void)snprintf(path, sizeof(path), "%s/%s", STS_APP_SRC_DIR, rel);
	f = fopen(path, "rb");
	TEST_ASSERT_NOT_NULL_MESSAGE(f, path);
	n = fread(g_src, 1U, sizeof(g_src) - 1U, f);
	(void)fclose(f);
	TEST_ASSERT_TRUE_MESSAGE(n < (sizeof(g_src) - 1U),
				 "source did not fit the scan buffer");
	TEST_ASSERT_TRUE_MESSAGE(n > 4096U, "source is implausibly small");
	g_src[n] = '\0';
	g_src_len = n;
	(void)snprintf(g_src_name, sizeof(g_src_name), "%s", rel);

	for (i = 0U; i < n; i++) {
		char c = g_src[i];
		char d = ((i + 1U) < n) ? g_src[i + 1U] : '\0';

		switch (st) {
		case CODE:
			if ((c == '/') && (d == '*')) {
				st = BLOCK;
				g_src[i] = ' ';
				g_src[i + 1U] = ' ';
				i++;
			} else if ((c == '/') && (d == '/')) {
				st = LINE;
				g_src[i] = ' ';
				g_src[i + 1U] = ' ';
				i++;
			} else if (c == '"') {
				st = STR;
			} else if (c == '\'') {
				st = CHR;
			}
			break;
		case BLOCK:
			g_src[i] = (c == '\n') ? '\n' : ' ';
			if ((c == '*') && (d == '/')) {
				g_src[i + 1U] = ' ';
				i++;
				st = CODE;
			}
			break;
		case LINE:
			if (c == '\n') {
				st = CODE;
			} else {
				g_src[i] = ' ';
			}
			break;
		case STR:
		case CHR:
			if (c == '\\') {
				i++;
				break;
			}
			if (((st == STR) && (c == '"')) ||
			    ((st == CHR) && (c == '\''))) {
				st = CODE;
			}
			break;
		}
	}
}

static unsigned int count_in(const span_t *s, const char *needle)
{
	size_t k = strlen(needle);
	unsigned int hits = 0U;
	size_t i;

	for (i = s->begin; (k != 0U) && ((i + k) <= s->end); i++) {
		if (memcmp(&g_src[i], needle, k) == 0) {
			hits++;
		}
	}
	return hits;
}

static size_t offset_in(const span_t *s, const char *needle)
{
	size_t k = strlen(needle);
	size_t i;
	char msg[256];

	for (i = s->begin; (i + k) <= s->end; i++) {
		if (memcmp(&g_src[i], needle, k) == 0) {
			return i;
		}
	}
	(void)snprintf(msg, sizeof(msg),
		       "%s: `%s` is gone — fix this scan, do not delete the "
		       "assertion it feeds",
		       g_src_name, needle);
	TEST_FAIL_MESSAGE(msg);
	return 0U;
}

/** The braced body of the function whose definition begins with @p sig. */
static span_t fn_body(const char *sig)
{
	size_t k = strlen(sig);
	span_t s = { 0U, 0U };
	size_t at = 0U;
	unsigned int hits = 0U;
	int depth = 0;
	size_t i;
	char msg[256];

	(void)snprintf(msg, sizeof(msg),
		       "%s: cannot locate exactly one body of `%s` — fix this "
		       "scan, do not delete the assertion it feeds",
		       g_src_name, sig);

	for (i = 0U; (i + k) <= g_src_len; i++) {
		if (memcmp(&g_src[i], sig, k) == 0) {
			hits++;
			at = i;
		}
	}
	TEST_ASSERT_EQUAL_UINT_MESSAGE(1U, hits, msg);

	while ((at < g_src_len) && (g_src[at] != '{')) {
		at++;
	}
	TEST_ASSERT_TRUE_MESSAGE(at < g_src_len, msg);
	s.begin = at + 1U;

	for (i = at; i < g_src_len; i++) {
		if (g_src[i] == '{') {
			depth++;
		} else if (g_src[i] == '}') {
			depth--;
			if (depth == 0) {
				s.end = i;
				break;
			}
		}
	}
	TEST_ASSERT_TRUE_MESSAGE(s.end > s.begin, msg);
	return s;
}

/**
 * obj_apply() POSTS the panel rail; it does not write it.
 *
 * The defect this forbids is a one-line "fix": calling sts_panel_led_set() here
 * would work on a bench and would violate the single-writer rule every time the
 * sequencer touched the same rail.
 */
static void test_the_glue_posts_the_panel_rail_and_never_writes_it(void)
{
	span_t b;

	load_source("zephyr/console/mp_glue.c");
	b = fn_body("static int obj_apply(void *user, size_t obj, const int32_t *value)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "strcmp(o->id, \"" PANEL_ID "\")"),
		"obj_apply() no longer dispatches the panel rail");
	/*
	 * One post per mailbox row, and the count is exact on purpose: a row
	 * that lost its branch would still leave the manifest advertising the
	 * object as wired, and test_mp_deferred.c cannot see the difference
	 * between "posted" and "fell through to the release short-circuit".
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		(unsigned int)STS_PWRSEQ_REQ_COUNT - 1U,
		count_in(&b, "sts_pwrseq_req_post("),
		"obj_apply() does not post exactly one request per mailbox row");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		(unsigned int)STS_PWRSEQ_REQ_COUNT - 1U,
		count_in(&b, "MP_APPLY_PENDING"),
		"a posted object no longer reports its write as pending, so the "
		"reply now claims a pin moved when it has not");
	/*
	 * The whole point, stated as an absence: the console thread does not
	 * touch the panel setter at all any more. `ui.panel.duty` was the last
	 * caller and it is now posted like the rest — this is the assertion
	 * that would fail if anyone "fixed" the 250 ms latency by writing PE0
	 * here, which works on a bench and violates the single-writer rule
	 * every time the sequencer touches the same pin.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "sts_panel_led_set("),
		"obj_apply() writes the panel setter from the console thread");
}

/**
 * Every mailbox row is dispatched by obj_apply() and named by mp_req_objects[].
 *
 * The two halves are separate failures with separate consequences: a row with
 * no obj_apply() branch is an object that silently answers -ENOTSUP, and a row
 * with no id is an outcome that can never be reported — mp_req_drain_locked()
 * logs it and drops it. Derived from the enum, so adding a row without wiring
 * both ends fails here rather than at a bench.
 */
static void test_every_mailbox_row_is_wired_at_both_ends(void)
{
	span_t ap;
	uint8_t req;

	load_source("zephyr/console/mp_glue.c");
	ap = fn_body("static int obj_apply(void *user, size_t obj, const int32_t *value)");

	for (req = (uint8_t)STS_PWRSEQ_REQ_NONE + 1U;
	     req < (uint8_t)STS_PWRSEQ_REQ_COUNT; req++) {
		const char *id = g_req_objects[req];
		char needle[128];
		char msg[192];

		TEST_ASSERT_NOT_NULL_MESSAGE(
			id, "a mailbox row names no manifest object, so its "
			    "outcome could never be reported");
		TEST_ASSERT_TRUE_MESSAGE(mp_obj_find(id) >= 0, id);

		(void)snprintf(needle, sizeof(needle),
			       "strcmp(o->id, \"%s\")", id);
		(void)snprintf(msg, sizeof(msg),
			       "obj_apply() has no branch for `%s`, which has a "
			       "mailbox row", id);
		TEST_ASSERT_EQUAL_UINT_MESSAGE(1U, count_in(&ap, needle), msg);
	}
}

/**
 * The drain routes the antenna bias through the PIN, never through
 * sts_pwrseq_ant_bias_request().
 *
 * That entry point raises STS_MP_VETO_ANT_BIAS on its off path — correctly, it
 * is the antenna supervisor's own withdrawal. Routing an override through it
 * would fire that veto on the operator's OWN request and withdraw the lease
 * that had just been granted: one event, two vetoes, the second aimed at the
 * technician who caused the first. The drain therefore calls the shared drive
 * helper, and this asserts the veto entry point is not on that path.
 */
static void test_the_mailbox_drain_does_not_re_veto_the_antenna_bias(void)
{
	span_t b;
	span_t d;

	load_source("zephyr/platform/pwrseq_exec.c");

	b = fn_body("static bool pwrseq_mbox_apply_rail(uint8_t req, bool release, int32_t value,");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "sts_pwrseq_ant_bias_request("),
		"the mailbox drain calls the antenna supervisor's entry point, "
		"so an override of the bias vetoes its own lease");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "sts_mp_veto("),
		"the mailbox drain raises a firmware veto for a request the "
		"operator made");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "pwrseq_rail_drive("),
		"the drain no longer moves the pin through the shared helper");
	/*
	 * And the GPS row still tells gnssmgr when it re-powers the receiver.
	 * Modelled in this suite's fixture, so only the source says whether the
	 * shipped drain makes the call — without it gnssmgr keeps its ACK
	 * bookkeeping across a receiver that cold-booted.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_gnss_notify_reset("),
		"re-powering the GPS rail no longer notifies the receiver");

	/*
	 * THE RAIL POLICY ITSELF, pinned in the shipped source rather than only
	 * in this file's mirror of it.
	 *
	 * The mirror above (apply_rail) is this suite's own code: a mutation to
	 * pwrseq_exec.c's copy of these two lines leaves every behavioural
	 * assertion in this file green, because none of them links that
	 * translation unit. Both lines are decisions, not mechanism:
	 *
	 *   the release destination — `auto_on`, not false. A release that
	 *   resolved to off would turn the dead-man into the thing that unpowers
	 *   the GNSS receiver, and nothing in the system would turn it back on.
	 *   the ON precondition — an override may not raise a load firmware has
	 *   shed, has not reached, or has taken down as a fail-action.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "want = release ? r->auto_on : (value != 0);"),
		"a rail release no longer restores firmware's own level");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "if (want && !r->auto_on) {"),
		"an override may now raise a rail firmware has taken down");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&b, "if (want && !r->auto_on) {") <
			offset_in(&b, "pwrseq_rail_drive("),
		"the ON precondition moved below the pin write, so the refusal "
		"happens after the rail has already come up");

	/* And the whole drain raises none either — the veto belongs to the
	 * sequencer's actions, which run before it. */
	d = fn_body("static void pwrseq_service_mailbox(uint32_t now_ms)\n{");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&d, "sts_mp_veto("),
		"the mailbox drain raises a firmware veto");
}

/**
 * The setpoint row's two refusals are in the shipped source and in this order.
 *
 * Both are load-bearing and neither is inferable from the mailbox mechanism:
 * stage 8 owns the wiper, and a gated FE must never see a commanded step on its
 * supply. A release must reach neither check, or a lapsed lease could not put
 * firmware's own code back.
 */
static void test_the_setpoint_row_refuses_stage_eight_and_a_gated_fe(void)
{
	span_t b;

	load_source("zephyr/platform/pwrseq_exec.c");
	b = fn_body("static bool pwrseq_mbox_apply_vset(bool release, int32_t value, uint8_t *out_err,");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "st.stage == PWRSEQ_STAGE_8_RB"),
		"the setpoint may now be written inside the guarded rubidium "
		"sequence, which owns the wiper");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "st.rb_gated"),
		"the setpoint may now be written with the FE-5680A connected to "
		"the rail");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "STS_PWRSEQ_REQ_ERR_GATED"),
		"the gated refusal lost its own sentence");
	/* The platform's own bound, not the console's clamped value. */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_rb_code_for_mv(&pwrseq_cfg.rb_xfer, value,"),
		"the setpoint no longer goes through the bounded inverse");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "pwrseq_cfg.rb_vmax_mv"),
		"the platform no longer bounds the setpoint against the ceiling "
		"the sequencer is running");
	/* The release is above both refusals, so it cannot be refused. */
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&b, "if (release)") <
			offset_in(&b, "st.stage == PWRSEQ_STAGE_8_RB"),
		"a release of the setpoint can now be refused, so a lapsed "
		"lease would strand the rail at a maintenance voltage");
	/* And the read-back is refreshed for the interlock that judges it. */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_hk_request_ina("),
		"the setpoint no longer asks for a fresh VCC_RB reading, so "
		"MP_ILK_RB_VERIFY may judge a reading older than the write");
}

/**
 * The rail read-backs report the PIN, and the display interlock samples it too.
 *
 * sts_pwrseq_snap_t's `gps_on` / `ant_bias_on` / `display_on` / `rb_gated` are
 * pwrseq's record of what IT commanded, and a lease moves these pins without
 * telling pwrseq. Reading the snapshot would hand a host its own request back,
 * and — worse for the display — would make MP_ILK_DISP_OFF blind to the very
 * off it measures, so a technician could cycle DISP_EN with no minimum
 * off-time.
 *
 * `ref.term.en` is the fifth read-back and reaches the same rule from the other
 * direction: the snapshot has no ref-term field at all, so there is no belief
 * for it to answer from — only the pin, or nothing.
 */
static void test_the_rail_readbacks_and_the_disp_interlock_read_the_pin(void)
{
	span_t rd;
	span_t ob;

	load_source("zephyr/console/mp_glue.c");

	rd = fn_body("static int obj_read(void *user, size_t obj, mp_val_t *out)");
	/*
	 * FIVE, and each one is named below so a future reader can check the
	 * number instead of trusting it. The fifth is `ref.term.en` — the
	 * STS_PWRSEQ_REQ_REF_TERM_EN row, REF_TERM_EN on PC10 — which joined
	 * the other four when the SMA termination became leasable (F_LEASE in
	 * mp_manifest.c) and so acquired the same pin/belief divergence. It is
	 * a genuine measurement on exactly the same path as the four: it calls
	 * sts_pwrseq_rail_on(), which ends in `gpio_pin_get_dt(r->gpio) == 1`,
	 * and pwrseq_rails[STS_PWRSEQ_REQ_REF_TERM_EN].gpio is &ref_term_en,
	 * which sts_pwrseq_start() configures behind a gpio_is_ready_dt()
	 * guard — not a NULL row that would make the call always answer false.
	 *
	 * It is the one row with no companion `ps.` assertion, and that is not
	 * an omission: see the note above this function.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		5U, count_in(&rd, "sts_pwrseq_rail_on("),
		"a rail read-back stopped reporting the pin");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&rd, "STS_PWRSEQ_REQ_GPS_EN"),
		"the GPS rail is not one of the pin read-backs counted above");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&rd, "STS_PWRSEQ_REQ_ANT_BIAS_EN"),
		"the antenna-bias rail is not one of the pin read-backs "
		"counted above");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&rd, "STS_PWRSEQ_REQ_DISP_EN"),
		"the display rail is not one of the pin read-backs counted "
		"above");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&rd, "STS_PWRSEQ_REQ_RB_GATE"),
		"the rubidium gate is not one of the pin read-backs counted "
		"above");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&rd, "STS_PWRSEQ_REQ_REF_TERM_EN"),
		"the SMA termination is not one of the pin read-backs counted "
		"above");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&rd, "ps.gps_on"),
		"the GPS read-back answers from pwrseq's belief again");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&rd, "ps.display_on"),
		"the display read-back answers from pwrseq's belief again");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&rd, "ps.rb_gated"),
		"the gate read-back answers from pwrseq's belief again");

	ob = fn_body("static void disp_observe_locked(const sts_pwrseq_snap_t *ps)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&ob, "sts_pwrseq_rail_on("),
		"the display off-time interlock no longer samples the pin, so "
		"an override's own off is invisible to it");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&ob, "ps->display_on"),
		"the display stamp follows the sequencer's belief again");
}

/**
 * MP_ILK_RB_VERIFY's expectation is a measurement, not the configured ceiling.
 *
 * `rb_expected_mv` is defined as "rail voltage the programmed code implies".
 * Filled from cfg `pwr.rb.vmax.mv` it said "the rail should read whatever the
 * ceiling is set to", which would auto-revert every `pwr.rb.gate` lease on a
 * unit whose ceiling sits more than MP_RB_VERIFY_TOL_PCT from its operating
 * setpoint — an interlock refusing correct hardware. It was dormant only
 * because the object could not be granted.
 */
static void test_the_rb_verify_expectation_is_the_programmed_rail(void)
{
	span_t b;

	load_source("zephyr/console/mp_glue.c");
	b = fn_body("static int prov_ilk(void *user, mp_ilk_state_t *out)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "out->rb_expected_mv = sts_pwrseq_rb_expected_mv();"),
		"the VCC_RB read-back expectation is not the programmed rail");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "out->rb_expected_mv = (int32_t)out->rb_vmax_mv;"),
		"the read-back expectation is the configured ceiling again, so "
		"a correctly-running rail fails its own verification");
	/* The permanent refusals beside it are untouched. */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "out->rb_code_min = 1U;"),
		"the raw wiper code's permanent refusal changed shape");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "out->rb_code_max = 0U;"),
		"the raw wiper code's window is no longer inverted, so a code "
		"is now accepted against an uncomputed bound");
}

/** The outcome drain exists, is bound into the tick, and runs in the right order. */
static void test_the_console_drains_outcomes_before_the_engine_tick(void)
{
	span_t b;
	span_t t;

	load_source("zephyr/console/mp_glue.c");

	b = fn_body("static void mp_req_drain_locked(uint32_t now_ms)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_pwrseq_req_take("),
		"the outcome drain no longer takes outcomes");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "mp_ovr_settled("),
		"an applied request no longer confirms its lease, so every "
		"working override is reverted for never landing");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "mp_veto("),
		"a refused request no longer withdraws its lease");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_pwrseq_req_reason("),
		"a refusal no longer carries the sequencer's reason");

	t = fn_body("void sts_mp_tick(void)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&t, "mp_req_drain_locked("),
		"the tick no longer drains sequencer outcomes, so a settled "
		"override is never confirmed and a refused one never withdrawn");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&t, "mp_veto_drain_locked()") <
			offset_in(&t, "mp_req_drain_locked("),
		"the outcome drain moved above the firmware vetoes");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&t, "mp_req_drain_locked(") <
			offset_in(&t, "mp_tick(&mp)"),
		"the outcome drain moved below mp_tick(), so a confirmation "
		"arrives after the lease it confirms has been reverted");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&t, "k_mutex_lock(&mp_lock") <
			offset_in(&t, "mp_req_drain_locked("),
		"the outcome drain moved outside the engine lock");
}

/** The read-back reports the pin, not the sequencer's belief about it. */
static void test_the_glue_reads_the_panel_pin_and_not_the_snapshot(void)
{
	span_t b;

	load_source("zephyr/console/mp_glue.c");
	b = fn_body("static int obj_read(void *user, size_t obj, mp_val_t *out)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "strcmp(o->id, \"" PANEL_ID "\")"),
		"the panel rail lost its read-back, so an asynchronous "
		"override can no longer be confirmed");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "ps.panel_led_on"),
		"the panel read-back answers from pwrseq's belief again; the "
		"PANEL_LED_EN action is a no-op in the executor, so that bit "
		"can be true while the string is dark");
}

/** The platform drain runs on the sequencer pass, after the actions. */
static void test_the_sequencer_drains_the_mailbox_after_its_own_actions(void)
{
	span_t b;
	span_t st;

	load_source("zephyr/platform/pwrseq_exec.c");

	/* The trailing brace pins this to the DEFINITION: the file also carries
	 * a forward declaration, which fn_body() would otherwise count as a
	 * second body and fail on. */
	b = fn_body("static void pwrseq_service_mailbox(uint32_t now_ms)\n{");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_pwrseq_reqq_claim("),
		"the drain no longer claims requests");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_pwrseq_reqq_settle("),
		"the drain no longer settles what it claimed, so a request can "
		"be consumed and reported nowhere");
	/*
	 * Claim and settle are each taken under the lock and the ACTUATION is
	 * NOT. This is the header's claim/execute/settle contract, and it is
	 * asserted as ADJACENCY rather than as a count of lock calls — a count
	 * is satisfied by a drain that holds the lock straight through the
	 * actuation, which is exactly the mistake worth catching. Holding a
	 * spinlock across sts_panel_led_set() would put a register write and a
	 * GPIO write inside a critical section the 1 kHz io_scan and the
	 * priority-4 discipline thread can be behind.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		2U, count_in(&b, "k_spin_lock(&pwrseq_mbox_lock)"),
		"the mailbox drain no longer takes the lock exactly twice");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		2U, count_in(&b, "k_spin_unlock(&pwrseq_mbox_lock, key)"),
		"the mailbox drain's lock/unlock pairs no longer balance");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U,
		count_in(&b,
			 "ok = sts_pwrseq_reqq_claim(&pwrseq_mbox, req, "
			 "&release, &value);\n\t\tk_spin_unlock("
			 "&pwrseq_mbox_lock, key);"),
		"the claim's critical section no longer ends immediately after "
		"the claim, so the actuation may now run inside the spinlock");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U,
		count_in(&b,
			 "key = k_spin_lock(&pwrseq_mbox_lock);\n\t\t(void)"
			 "sts_pwrseq_reqq_settle("),
		"the settle is no longer the first thing under its lock");
	/* ...and the actuation itself sits between the two, textually. */
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&b, "k_spin_unlock(&pwrseq_mbox_lock, key)") <
			offset_in(&b, "pwrseq_mbox_apply_panel("),
		"the actuation moved above the claim's unlock");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&b, "pwrseq_mbox_apply_panel(") <
			offset_in(&b, "sts_pwrseq_reqq_settle("),
		"the actuation moved below the settle that reports it");

	st = fn_body("void sts_pwrseq_step(uint32_t now_ms)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		2U, count_in(&st, "pwrseq_service_mailbox(now_ms)"),
		"the mailbox is no longer drained on BOTH exits of the "
		"sequencer pass; a bad input would strand an override's "
		"release, including the dead-man's");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&st, "pwrseq_drain()") <
			offset_in(&st, "pwrseq_service_mailbox(now_ms)"),
		"the mailbox drain moved above pwrseq_drain(), so a request can "
		"now outrank this tick's shed or fail-action");
}

/** The post entry point takes the lock and refuses when nothing would drain. */
static void test_the_post_entry_point_is_locked_and_refuses_when_dead(void)
{
	span_t b;

	load_source("zephyr/platform/pwrseq_exec.c");
	b = fn_body("int sts_pwrseq_req_post(uint8_t req, const int32_t *value)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "k_spin_lock(&pwrseq_mbox_lock)"),
		"the producer no longer serialises against the drain");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "k_spin_unlock(&pwrseq_mbox_lock, key)"),
		"the producer's lock is not released");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "return -ENODEV;"),
		"a request posted before the sequencer starts is now queued "
		"with nothing to drain it and no outcome to report");
	/*
	 * No mutex anywhere on this path. A k_mutex here would put the console
	 * thread and the housekeeping thread on each other's critical path,
	 * which is the arrangement mp_glue.h's tick budget forbids.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "k_mutex_lock"),
		"the mailbox producer takes a mutex");
}

/** The RPC reply reports a pending apply, and refuses one on `obj.set`. */
static void test_the_rpc_tells_the_truth_about_a_pending_apply(void)
{
	span_t ov;
	span_t se;

	load_source("core/mp/mp_rpc.c");

	ov = fn_body("static int m_obj_override(mp_ctx_t *c, const mp_json_t *p, "
		     "int params,");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&ov, "res.verify || res.pending"),
		"the override reply no longer reports a pending apply, so it "
		"asserts the value took effect before the pin moved");

	se = fn_body("static int m_obj_set(mp_ctx_t *c, const mp_json_t *p, int "
		     "params, mp_jw_t *w)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&se, "if (rc > 0) {"),
		"obj.set no longer refuses an asynchronous apply; its reply has "
		"no field that could admit the pin has not moved");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&se, "if (rc > 0) {") <
			offset_in(&se, "mp_jw_kv_i64(w, \"value\""),
		"the pending refusal moved below the reply it exists to "
		"prevent");
}

/** The settle deadline is enforced, and before the VCC_RB read-back. */
static void test_core_drops_a_lease_whose_write_never_landed(void)
{
	span_t b;

	load_source("core/mp/mp_override.c");
	b = fn_body("int mp_ovr_tick(mp_ovr_ctx_t *c, const mp_ilk_state_t *st, "
		    "uint32_t now_ms)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "settle_one(c, l, now_ms)"),
		"mp_ovr_tick() no longer judges unsettled applies, so a lease "
		"whose write never reached the pin stands for its whole TTL");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&b, "settle_one(c, l, now_ms)") <
			offset_in(&b, "verify_one(c, l, st, now_ms)"),
		"the settle check moved below the VCC_RB read-back, which "
		"would judge an unmoved pin against the rubidium's rail");
}

/* ------------------------------------------------------------------- runner */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_the_override_reaches_the_pin_through_the_drain);
	RUN_TEST(test_the_deadman_reverts_the_pin_through_the_same_mailbox);
	RUN_TEST(test_a_request_that_never_drains_drops_the_lease);
	RUN_TEST(test_the_settle_deadline_clears_the_worst_console_latency);

	RUN_TEST(test_two_requests_for_one_object_coalesce_to_the_last);
	RUN_TEST(test_every_request_id_has_its_own_slot);
	RUN_TEST(test_the_mailbox_refuses_a_non_request_id);
	RUN_TEST(test_a_drain_cannot_settle_a_request_into_silence);
	RUN_TEST(test_an_outcome_is_taken_exactly_once);

	RUN_TEST(test_a_refused_request_withdraws_the_lease_with_its_reason);
	RUN_TEST(test_the_refusal_reasons_are_distinct_and_fit_the_event_record);
	RUN_TEST(test_the_fail_safe_direction_is_never_refused);
	RUN_TEST(test_an_actuator_error_is_reported_as_a_refusal);

	RUN_TEST(test_every_mailbox_object_is_lease_only);
	RUN_TEST(test_a_pending_apply_is_refused_on_obj_set);

	RUN_TEST(test_every_wired_rail_reaches_its_pin_and_comes_back);
	RUN_TEST(test_restoring_the_gps_rail_notifies_the_receiver);
	RUN_TEST(test_a_shed_rail_refuses_an_override_with_the_shed_reason);
	RUN_TEST(test_the_deadman_restores_a_rail_through_the_mailbox);
	RUN_TEST(test_a_release_after_a_latched_short_leaves_the_bias_off);
	RUN_TEST(test_the_panel_dimmer_is_now_pending_and_refuses_a_shed_panel);
	RUN_TEST(test_the_vcc_rb_setpoint_is_refused_while_the_fe_is_connected);
	RUN_TEST(test_the_vcc_rb_setpoint_lands_below_the_request_and_reverts);
	RUN_TEST(test_a_setpoint_the_rail_does_not_follow_is_reverted);
	RUN_TEST(test_no_requested_setpoint_can_exceed_the_configured_ceiling);

	RUN_TEST(test_the_glue_posts_the_panel_rail_and_never_writes_it);
	RUN_TEST(test_every_mailbox_row_is_wired_at_both_ends);
	RUN_TEST(test_the_mailbox_drain_does_not_re_veto_the_antenna_bias);
	RUN_TEST(test_the_setpoint_row_refuses_stage_eight_and_a_gated_fe);
	RUN_TEST(test_the_rail_readbacks_and_the_disp_interlock_read_the_pin);
	RUN_TEST(test_the_rb_verify_expectation_is_the_programmed_rail);
	RUN_TEST(test_the_console_drains_outcomes_before_the_engine_tick);
	RUN_TEST(test_the_glue_reads_the_panel_pin_and_not_the_snapshot);
	RUN_TEST(test_the_sequencer_drains_the_mailbox_after_its_own_actions);
	RUN_TEST(test_the_post_entry_point_is_locked_and_refuses_when_dead);
	RUN_TEST(test_the_rpc_tells_the_truth_about_a_pending_apply);
	RUN_TEST(test_core_drops_a_lease_whose_write_never_landed);

	return UNITY_END();
}
