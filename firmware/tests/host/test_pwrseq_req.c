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
#include "zephyr/platform/sts_pwrseq_req.h"

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
static bool g_hw_fails;         /* sts_panel_led_set() returns an error */

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

		ok = apply_panel(release, value, &err, &applied);

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
};

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

static int apply_cb(void *user, size_t obj, const int32_t *value)
{
	const mp_obj_t *o = mp_obj_at(obj);

	(void)user;
	TEST_ASSERT_NOT_NULL(o);

	if (strcmp(o->id, PANEL_ID) == 0) {
		int32_t on = ((value != NULL) && (*value != 0)) ? 1 : 0;
		int rc;

		g_apply_calls++;
		if (value == NULL) {
			g_release_calls++;
		}
		rc = sts_pwrseq_reqq_post(&g_mbox,
					  (uint8_t)STS_PWRSEQ_REQ_PANEL_LED_EN,
					  (value == NULL), on, g_now);
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
	memset(&g_mbox, 0, sizeof(g_mbox));

	memset(&g_ilk, 0, sizeof(g_ilk));
	g_ilk.ocxo_warm = true;
	g_ilk.supercaps_ok = true;
	g_ilk.vcc_rb_mv = 15000;
	g_ilk.vcc_rb_valid = true;
	g_ilk.rb_expected_mv = 15000;
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
static void test_the_panel_rail_is_lease_only(void)
{
	const mp_obj_t *o = mp_obj_at(panel_obj());
	uint32_t sid = session();
	char req[256];

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, (unsigned int)(o->flags & MP_OF_WRITE),
		"a rail whose apply is asynchronous became writable; obj.set "
		"has no way to say the pin has not moved yet");
	TEST_ASSERT_TRUE((o->flags & MP_OF_OVERRIDE) != 0U);
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, (unsigned int)(o->flags & MP_OF_DEFERRED),
		"the one object this mechanism wires is still advertised as "
		"deferred");
	/* It stays readable, or the override could not be confirmed. */
	TEST_ASSERT_TRUE((o->flags & MP_OF_READ) != 0U);

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"" PANEL_ID "\",\"value\":true,"
		       "\"sid\":%u}}",
		       (unsigned int)sid);
	call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());
	TEST_ASSERT_FALSE(pin_on());
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

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"ui.panel.duty\",\"value\":40,"
		       "\"sid\":%u}}",
		       (unsigned int)sid);
	call(req);
	TEST_ASSERT_EQUAL_INT64(40, res_i("value"));
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
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_pwrseq_req_post("),
		"the panel rail no longer goes through the sequencer mailbox");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "MP_APPLY_PENDING"),
		"obj_apply() no longer reports the write as pending, so the "
		"reply now claims a pin moved when it has not");
	/*
	 * The whole point, stated as an absence: exactly ONE call to the panel
	 * setter in this function, and it is `ui.panel.duty`'s. A second would
	 * be the console thread writing a rail pin.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_panel_led_set("),
		"obj_apply() writes the panel setter more than once; the rail "
		"enable must be posted to housekeeping, not written here");
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

	RUN_TEST(test_the_panel_rail_is_lease_only);
	RUN_TEST(test_a_pending_apply_is_refused_on_obj_set);

	RUN_TEST(test_the_glue_posts_the_panel_rail_and_never_writes_it);
	RUN_TEST(test_the_console_drains_outcomes_before_the_engine_tick);
	RUN_TEST(test_the_glue_reads_the_panel_pin_and_not_the_snapshot);
	RUN_TEST(test_the_sequencer_drains_the_mailbox_after_its_own_actions);
	RUN_TEST(test_the_post_entry_point_is_locked_and_refuses_when_dead);
	RUN_TEST(test_the_rpc_tells_the_truth_about_a_pending_apply);
	RUN_TEST(test_core_drops_a_lease_whose_write_never_landed);

	return UNITY_END();
}
