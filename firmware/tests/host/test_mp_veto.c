/*
 * STS1000 "Meridian" — the firmware veto: from the board's condition to the
 * withdrawn lease, the wire record and the audit line.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * WHAT THIS SUITE EXISTS TO CATCH, stated so it cannot be diluted later.
 *
 * test_mp_override.c already proved mp_ovr_veto(): call it, the lease goes, the
 * apply callback sees a release, the event sink sees MP_OVR_EV_VETO.
 * test_mp_rpc.c already proved mp_veto(). Both passed for as long as the defect
 * existed, because the defect was in neither — it was that **nothing raised
 * one**. `mp_veto` and `mp_ovr_veto` were absent from zephyr.elf; the only
 * in-app relocation to mp_ovr_veto sat inside .text.mp_veto, a dead subtree
 * citing itself. An override firmware had decided was unsafe stood until its
 * keepalive lapsed, with no veto on channel 0x09 and no audit line. FMT §5.4.
 *
 * So this suite starts at the BOARD CONDITION. The rubidium case runs the real
 * core/pwrseq stage machine up to a live rail, raises RB_OV_DET on its real
 * input struct, lets the real pwrseq_step() decide to shut the rubidium down,
 * and takes the actions it emitted through the real
 * sts_mp_veto_of_action()/sts_mp_veto_objects() policy into a real mp_ctx_t
 * whose lease table, event stream, frame encoder and log ring are all the
 * shipped ones. A test that called mp_ovr_veto() with a hand-picked index would
 * have passed before this change and would prove nothing — that is exactly the
 * hole the event-channel work left.
 *
 * Two mirrors of Zephyr code exist here and are deliberately the ONLY routes in
 * and out — exec_pass() for pwrseq_exec.c's pwrseq_drain()/pwrseq_exec_action()
 * and veto_drain_pass() for mp_glue.c's mp_veto_drain_locked(). Both are a
 * handful of marshalling lines around decisions that live in headers this file
 * compiles for real. The half a mirror cannot check — that those two .c files
 * still make the call at all, which is the defect above — is checked by reading
 * their sources, brace-matched and guard-scoped, the way test_mp_events.c does.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "ina228/ina228.h"
#include "mp/mp.h"
#include "pwrseq/pwrseq.h"
#include "zephyr/console/sts_mp_veto_policy.h"

#ifndef STS_APP_SRC_DIR
#error "STS_APP_SRC_DIR must be defined by tests/host/CMakeLists.txt"
#endif

#define SERIAL "STS1000-000042"
#define USER "tech"

/* The board's real cadences, so a test cannot model a faster one (test_pwrseq.c). */
#define HK_TICK_MS 250U
#define HK_SWEEP_MS 1000U
#define BUCK_SETTLE_MS 80U

/* ====================================================== the engine fixture == */

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

/* The wire, and a second decoder to read it back. */
static uint8_t g_wire[32768];
static size_t g_wire_len;
static mp_frame_rx_t g_drx;
static mp_reasm_t g_dslots[2];
static uint8_t g_dslot_buf[2][2048];
static uint8_t g_msg[MP_CH_COUNT][1024];
static size_t g_msg_len[MP_CH_COUNT];
static unsigned int g_msg_n[MP_CH_COUNT];

/* Every apply() the engine performed, so "reverted to firmware-automatic" is an
 * observed release and not an inference from a flag. */
#define APPLY_MAX 64U
typedef struct {
	size_t obj;
	bool release;
	int32_t value;
} apply_rec_t;
static apply_rec_t g_apply[APPLY_MAX];
static unsigned int g_apply_n;

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

static int apply_cb(void *user, size_t obj, const int32_t *value)
{
	(void)user;
	if (g_apply_n < APPLY_MAX) {
		g_apply[g_apply_n].obj = obj;
		g_apply[g_apply_n].release = (value == NULL);
		g_apply[g_apply_n].value = (value != NULL) ? *value : 0;
		g_apply_n++;
	}
	return 0;
}

/*
 * The interlock state the engine sees, and the state every grant below is
 * judged against. A healthy rubidium chain: the oven is warm, the backup rails
 * are good, the 26 V latch is clear and VCC_RB reads what its code implies. It
 * has to be a real, permitting state — `pwr.rb.en` carries MP_ILK_RB_WARM and
 * `pwr.rb.gate` MP_ILK_RB_OV | MP_ILK_RB_VERIFY, so a NULL state would refuse
 * the grant outright and a missing wiring callback would make mp_ovr_tick()
 * fail the pending read-back for want of a reading rather than because
 * anything was wrong. Neither would be the veto under test.
 */
static mp_ilk_state_t g_ilk;

static int ilk_cb(void *user, mp_ilk_state_t *out)
{
	(void)user;
	*out = g_ilk;
	return 0;
}

static int decoded_msg(void *user, uint8_t ch, const uint8_t *msg, size_t len)
{
	(void)user;
	TEST_ASSERT_TRUE(ch < MP_CH_COUNT);
	TEST_ASSERT_TRUE(len <= sizeof(g_msg[0]));
	memcpy(g_msg[ch], msg, len);
	g_msg_len[ch] = len;
	g_msg_n[ch]++;
	return 0;
}

void setUp(void)
{
	mp_wiring_t w;
	unsigned int i;

	g_now = 10000U;
	g_wire_len = 0U;
	g_apply_n = 0U;

	memset(&g_ilk, 0, sizeof(g_ilk));
	g_ilk.ocxo_warm = true;
	g_ilk.supercaps_ok = true;
	g_ilk.rb_ov_latched = false;
	g_ilk.vcc_rb_mv = 15000;
	g_ilk.vcc_rb_valid = true;
	g_ilk.rb_expected_mv = 15000;
	g_ilk.rb_vmax_mv = 15000U;
	g_ilk.rb_code_max = 255U;
	g_ilk.liveness_ok = true;
	g_ilk.rb_lock = true;
	g_ilk.extref_ok = true;
	memset(g_msg_len, 0, sizeof(g_msg_len));
	memset(g_msg_n, 0, sizeof(g_msg_n));

	for (i = 0U; i < 2U; i++) {
		g_slots[i].buf = g_slot_buf[i];
		g_slots[i].cap = sizeof(g_slot_buf[i]);
		g_dslots[i].buf = g_dslot_buf[i];
		g_dslots[i].cap = sizeof(g_dslot_buf[i]);
	}
	TEST_ASSERT_EQUAL_INT(0, mp_frame_rx_init(&g_drx, g_dslots, 2U,
						  decoded_msg, NULL));
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
	w.ilk = ilk_cb;
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
	TEST_ASSERT_EQUAL_INT(0, mp_mode_enter(&g_c));
	TEST_ASSERT_EQUAL_INT(0, mp_set_link(&g_c, true));

	/* mp_mode_enter() queues MP_EV_MODE; start from a drained queue so the
	 * counts below describe this change and nothing else. */
	{
		uint8_t sink[512];

		(void)mp_enc_events(&g_c.st, 0U, 0U, 0U, sink, sizeof(sink));
		TEST_ASSERT_EQUAL_size_t(0U, mp_stream_event_count(&g_c.st));
	}
}

void tearDown(void)
{
}

/** Open an admin session and return its id. */
static uint32_t session(void)
{
	uint32_t sid = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_session_open(&g_c.ovr, 5000U,
						     (uint8_t)MP_ROLE_ADMIN,
						     USER, g_now, &sid));
	TEST_ASSERT_NOT_EQUAL_UINT32(0U, sid);
	return sid;
}

/** Grant a lease on @p id against the healthy interlock state above. */
static size_t hold(const char *id, int32_t value, uint32_t sid)
{
	int obj = mp_obj_find(id);

	TEST_ASSERT_TRUE_MESSAGE(obj >= 0, id);
	TEST_ASSERT_EQUAL_INT_MESSAGE(0,
				      mp_ovr_grant(&g_c.ovr, (size_t)obj, value,
						   0U, sid, &g_ilk, g_now,
						   NULL),
				      id);
	TEST_ASSERT_NOT_NULL(mp_ovr_lease(&g_c.ovr, (size_t)obj));
	return (size_t)obj;
}

static void subscribe_event(void)
{
	TEST_ASSERT_EQUAL_INT(0, mp_stream_sub(&g_c.st, (uint8_t)MP_CH_EVENT,
					       0U, g_now));
}

/* ======================================================= the board fixture == */

/*
 * core/pwrseq driven the way pwrseq_exec.c drives it, with the same glue model
 * test_pwrseq.c uses: a buck that takes time to move, a digipot readback that
 * follows the last code written, and an INA cache that only refreshes when
 * something refreshes it. Trimmed to what the rubidium path needs — this file
 * is about the veto, and test_pwrseq.c owns the sequencer's own properties.
 */
#define LOG_MAX 64U

typedef struct {
	pwrseq_ctx_t ctx;
	pwrseq_in_t in;
	pwrseq_act_t log[LOG_MAX];
	size_t log_len;

	bool rb_powered;
	uint16_t rb_cmd;
	int32_t rail_mv;
	int32_t rail_target_mv;
	uint32_t rail_settle_at_ms;
	int32_t cache_mv;
	uint32_t cache_stamp_ms;
	uint32_t cache_next_ms;
	bool cache_requested;
} board_t;

static void in_healthy(pwrseq_in_t *in)
{
	int i;

	memset(in, 0, sizeof(*in));
	in->mono_ms = 0U;
	in->nor_ready = true;
	in->i2c_probe_ok = true;
	in->cal_loaded = true;
	in->shunt_trims_applied = true;

	for (i = 0; i < (int)INA228_RAIL_COUNT; i++) {
		in->ina_valid[i] = true;
		in->ina_vbus_mv[i] = ina228_rail_tbl[i].nominal_mv;
		in->ina_current_ma[i] = 10;
		in->ina_age_ms[i] = 0U;
	}
	in->ina_vbus_mv[INA228_RAIL_VCC_RB] = 4515;
	in->ina_current_ma[INA228_RAIL_OCXO] = 300;

	in->pg_mask = 0xFFU;
	in->phy_refclk_stable = true;
	in->phy_id_ok = true;
	in->gnss_cfg_ack = true;
	in->ocxo_temp_stable = true;
	in->rb_wanted = true;
	in->supercaps_charged = true;
	in->digipot_readback_valid = true;
	in->rb_lock = true;
	in->extref_in_band = true;
	in->poe_granted_mw = 51000U;
	in->poe_measured_mw = 14000U;
	in->ui_wanted = true;
	in->liveness_ok = true;
}

static void glue_apply(board_t *b, size_t from, uint32_t ms)
{
	size_t i;
	int32_t target;

	for (i = from; i < b->log_len; i++) {
		switch ((pwrseq_action_t)b->log[i].action) {
		case PWRSEQ_ACT_DIGIPOT_WRITE:
		case PWRSEQ_ACT_DIGIPOT_WRITE_OP:
			b->rb_cmd = (uint16_t)b->log[i].arg;
			b->cache_requested = true;
			break;
		case PWRSEQ_ACT_RB_PWR_EN:
			b->rb_powered = true;
			b->cache_requested = true;
			break;
		case PWRSEQ_ACT_RB_PWR_DIS:
			b->rb_powered = false;
			break;
		default:
			break;
		}
	}

	b->in.digipot_readback = b->rb_cmd;
	b->in.digipot_readback_valid = true;

	target = b->rb_powered
			 ? pwrseq_rb_expected_mv(&b->ctx.cfg.rb_xfer, b->rb_cmd)
			 : 0;
	if (target != b->rail_target_mv) {
		b->rail_target_mv = target;
		b->rail_settle_at_ms = ms + BUCK_SETTLE_MS;
	}
	if ((int32_t)(ms - b->rail_settle_at_ms) >= 0) {
		b->rail_mv = b->rail_target_mv;
	}

	{
		bool refresh = false;

		if ((int32_t)(ms - b->cache_next_ms) >= 0) {
			b->cache_next_ms = ms + HK_SWEEP_MS;
			refresh = true;
		}
		if (b->cache_requested) {
			b->cache_requested = false;
			refresh = true;
		}
		if (refresh) {
			b->cache_mv = b->rail_mv;
			b->cache_stamp_ms = ms;
		}
	}
	b->in.ina_vbus_mv[INA228_RAIL_VCC_RB] = b->cache_mv;
	b->in.ina_age_ms[INA228_RAIL_VCC_RB] = ms - b->cache_stamp_ms;
}

/* ------------------------------------------- the two mirrors, and only these */

/*
 * Pending veto subjects, exactly as mp_glue.c stages them: one bit per subject,
 * OR-ed by the producer and swapped to zero by the drain. A bitmap and not a
 * queue, so two requests for the same subject coalesce and nothing can be
 * dropped.
 */
static uint32_t g_veto_pending;
static uint32_t g_veto_raised;

/** mp_glue.c's sts_mp_veto(): the producer, off the engine lock. */
static void veto_raise(sts_mp_veto_t s)
{
	if ((s <= STS_MP_VETO_NONE) || (s >= STS_MP_VETO_COUNT)) {
		return;
	}
	g_veto_pending |= (uint32_t)STS_MP_VETO_BIT(s);
	g_veto_raised++;
}

/**
 * pwrseq_exec.c's pwrseq_drain() + the veto half of pwrseq_exec_action().
 *
 * The DECISION — which subject an action raises, and that the over-voltage
 * latch is what separates "the hardware killed the rubidium" from "the
 * sequencer took it down" — is the real sts_mp_veto_of_action(). What is
 * mirrored is only the loop around it.
 */
static void exec_pass(board_t *b)
{
	pwrseq_act_t a;

	while (pwrseq_action_get(&b->ctx, &a) == 0) {
		sts_mp_veto_t subj = sts_mp_veto_of_action(
			a.action, pwrseq_ov_latched(&b->ctx));

		if (subj != STS_MP_VETO_NONE) {
			veto_raise(subj);
		}
		TEST_ASSERT_TRUE_MESSAGE(b->log_len < LOG_MAX,
					 "action log overflowed");
		b->log[b->log_len++] = a;
	}
}

/** One sequencer pass: step, execute (raising vetoes), model the hardware. */
static void board_tick(board_t *b, uint32_t dt_ms)
{
	size_t before = b->log_len;

	b->in.mono_ms += dt_ms;
	TEST_ASSERT_EQUAL_INT(0, pwrseq_step(&b->ctx, &b->in));
	exec_pass(b);
	glue_apply(b, before, b->in.mono_ms);
}

static void board_init(board_t *b)
{
	memset(b, 0, sizeof(*b));
	TEST_ASSERT_EQUAL_INT(0, pwrseq_init(&b->ctx, NULL));
	in_healthy(&b->in);
	b->rb_cmd = b->ctx.cfg.digipot_safe_code;
	b->cache_next_ms = HK_SWEEP_MS;
	TEST_ASSERT_EQUAL_INT(0, pwrseq_start(&b->ctx, 0U));
	g_veto_pending = 0U;
	g_veto_raised = 0U;
}

/** Run the real bring-up until the rubidium rail is live. */
static void board_run_to_rb(board_t *b)
{
	unsigned int i;

	board_tick(b, 0U);
	for (i = 0U; i < 400U; i++) {
		if (b->ctx.rb_enabled) {
			return;
		}
		TEST_ASSERT_FALSE_MESSAGE(b->ctx.halted, "bring-up halted");
		board_tick(b, HK_TICK_MS);
	}
	TEST_FAIL_MESSAGE("the rubidium rail never came up");
}

/**
 * mp_glue.c's mp_veto_drain_locked(), which runs inside sts_mp_tick()'s lock.
 *
 * Same order of operations: claim the mask with a swap so a subject raised
 * during the drain belongs to the next pass, then map each subject through the
 * real policy and call the real mp_veto() on each manifest id it names.
 */
static int veto_drain_pass(void)
{
	uint32_t pend = g_veto_pending;
	unsigned int s;
	int withdrawn = 0;

	g_veto_pending = 0U;
	if (pend == 0U) {
		return 0;
	}

	for (s = (unsigned int)STS_MP_VETO_NONE + 1U;
	     s < (unsigned int)STS_MP_VETO_COUNT; s++) {
		const char *const *ids;
		const char *reason;
		size_t n = 0U;
		size_t i;

		if ((pend & (uint32_t)STS_MP_VETO_BIT(s)) == 0U) {
			continue;
		}
		reason = sts_mp_veto_reason((sts_mp_veto_t)s);
		ids = sts_mp_veto_objects((sts_mp_veto_t)s, &n);

		for (i = 0U; i < n; i++) {
			int obj = mp_obj_find(ids[i]);
			int rc;

			TEST_ASSERT_TRUE_MESSAGE(obj >= 0, ids[i]);
			rc = mp_veto(&g_c, (size_t)obj, reason);
			TEST_ASSERT_TRUE_MESSAGE((rc == 0) || (rc == -ENOENT),
						 ids[i]);
			if (rc == 0) {
				withdrawn++;
			}
		}
	}
	return withdrawn;
}

/** The drain, then the tick that frames what it produced. */
static int service(void)
{
	size_t from = g_wire_len;
	int n = veto_drain_pass();

	(void)mp_tick(&g_c);
	if (g_wire_len > from) {
		TEST_ASSERT_TRUE(mp_frame_rx_input(&g_drx, &g_wire[from],
						   g_wire_len - from) >= 0);
	}
	return n;
}

/* ------------------------------------------------------------ record readers */

typedef struct {
	uint64_t kind;
	uint64_t sub;
	uint64_t id;
	uint64_t edge;
	int64_t value;
	uint64_t mono_ms;
	char text[MP_EV_TEXT_MAX];
} ev_out_t;

/** Decode every event in the last record on channel 0x09. */
static size_t read_events(ev_out_t *out, size_t max)
{
	mp_cbor_rd_t r;
	size_t cnt = 0U;
	size_t i;

	TEST_ASSERT_TRUE_MESSAGE(g_msg_len[MP_CH_EVENT] > 0U,
				 "no event record reached the wire");
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_msg[MP_CH_EVENT],
						 g_msg_len[MP_CH_EVENT]));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, 9U));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
	TEST_ASSERT_TRUE(cnt <= max);

	for (i = 0U; i < cnt; i++) {
		const char *tp = NULL;
		size_t tn = 0U;
		size_t f = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &f));
		TEST_ASSERT_EQUAL_size_t(7U, f);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &out[i].kind));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &out[i].sub));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &out[i].id));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &out[i].edge));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_int(&r, &out[i].value));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &out[i].mono_ms));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_text(&r, &tp, &tn));
		TEST_ASSERT_TRUE(tn < MP_EV_TEXT_MAX);
		memcpy(out[i].text, tp, tn);
		out[i].text[tn] = '\0';
	}
	return cnt;
}

/** Index of the MP_EV_OVERRIDE/VETO record for @p obj, or SIZE_MAX. */
static size_t find_veto(const ev_out_t *ev, size_t n, size_t obj)
{
	size_t i;

	for (i = 0U; i < n; i++) {
		if ((ev[i].kind == (uint64_t)MP_EV_OVERRIDE) &&
		    (ev[i].sub == (uint64_t)MP_OVR_EV_VETO) &&
		    (ev[i].id == (uint64_t)obj)) {
			return i;
		}
	}
	return (size_t)-1;
}

static unsigned int releases_of(size_t obj)
{
	unsigned int i;
	unsigned int n = 0U;

	for (i = 0U; i < g_apply_n; i++) {
		if (g_apply[i].release && (g_apply[i].obj == obj)) {
			n++;
		}
	}
	return n;
}

/** The most recent log record whose text contains @p needle, or NULL. */
static const logr_rec_t *log_find(const char *needle)
{
	static logr_rec_t recs[32];
	logr_filter_t f;
	uint16_t n = 0U;
	uint32_t next = 0U;
	uint32_t gap = 0U;
	int i;

	logr_filter_all(&f);
	TEST_ASSERT_EQUAL_INT(0, logr_tail(&g_log, 0U, &f, recs, 32U, &n, &next,
					   &gap));
	for (i = (int)n - 1; i >= 0; i--) {
		if (strstr(recs[i].msg, needle) != NULL) {
			return &recs[i];
		}
	}
	return NULL;
}

/* ================================================== 1. the rubidium, OV ==== */

/**
 * THE CASE THE WHOLE CHANGE IS FOR.
 *
 * A technician holds `pwr.rb.en` on. The autonomous 26 V over-voltage latch
 * trips — the hardware has already killed the buck — the real sequencer decides
 * to drop RB_VCC_GATE and RB_PWR_EN, and the override must not survive it.
 *
 * Nothing here calls mp_ovr_veto() or names a subject by hand: the only input is
 * `rb_ov_det` on a real pwrseq_in_t.
 */
static void test_the_ov_latch_withdraws_a_rubidium_override(void)
{
	board_t b;
	uint32_t sid;
	size_t obj_en;
	size_t obj_gate;
	ev_out_t ev[8];
	size_t n;
	size_t k;
	const logr_rec_t *rec;

	subscribe_event();
	sid = session();

	board_init(&b);
	board_run_to_rb(&b);
	/* Bring-up emitted no veto: every action it ran was an ON. */
	TEST_ASSERT_EQUAL_UINT32(0U, g_veto_raised);

	obj_en = hold("pwr.rb.en", 1, sid);
	obj_gate = hold("pwr.rb.gate", 1, sid);
	TEST_ASSERT_EQUAL_size_t(2U, mp_ovr_active(&g_c.ovr));
	g_apply_n = 0U;

	/* The condition, and the only thing this test injects. */
	b.in.rb_ov_det = true;
	board_tick(&b, HK_TICK_MS);

	/* The sequencer really did decide to drop the rail... */
	TEST_ASSERT_TRUE(pwrseq_ov_latched(&b.ctx));
	TEST_ASSERT_FALSE(b.ctx.rb_enabled);
	/* ...and the executor really did raise the OV subject for it. */
	TEST_ASSERT_TRUE(g_veto_raised >= 1U);
	TEST_ASSERT_EQUAL_UINT32((uint32_t)STS_MP_VETO_BIT(STS_MP_VETO_RB_OV),
				 g_veto_pending);

	/* One console pass later, both leases are gone. */
	TEST_ASSERT_EQUAL_INT(2, service());
	TEST_ASSERT_NULL(mp_ovr_lease(&g_c.ovr, obj_en));
	TEST_ASSERT_NULL(mp_ovr_lease(&g_c.ovr, obj_gate));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c.ovr));

	/* Reverted to firmware-automatic through the apply callback, once each. */
	TEST_ASSERT_EQUAL_UINT(1U, releases_of(obj_en));
	TEST_ASSERT_EQUAL_UINT(1U, releases_of(obj_gate));

	/* On the wire, as MP_OVR_EV_VETO, naming the cause. */
	n = read_events(ev, 8U);
	k = find_veto(ev, n, obj_en);
	TEST_ASSERT_NOT_EQUAL_size_t((size_t)-1, k);
	TEST_ASSERT_EQUAL_STRING(sts_mp_veto_reason(STS_MP_VETO_RB_OV),
				 ev[k].text);
	TEST_ASSERT_NOT_EQUAL_size_t((size_t)-1, find_veto(ev, n, obj_gate));

	/* And audited, at WARN, naming the object, the user and the cause
	 * (FMT §5.3) — not merely streamed to whoever happened to subscribe. */
	rec = log_find("pwr.rb.en");
	TEST_ASSERT_NOT_NULL_MESSAGE(rec, "no audit line for the veto");
	TEST_ASSERT_EQUAL_UINT8((uint8_t)LOGR_WARN, rec->level);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)LOGR_SUB_MCP, rec->subsys);
	TEST_ASSERT_NOT_NULL(strstr(rec->msg, "veto"));
	TEST_ASSERT_NOT_NULL(strstr(rec->msg, USER));
	TEST_ASSERT_NOT_NULL(strstr(rec->msg,
				    sts_mp_veto_reason(STS_MP_VETO_RB_OV)));
}

/**
 * The same rail, taken down by the sequencer rather than by the hardware, is a
 * different sentence.
 *
 * Withdrawing the intent (`rb_wanted` false) is pwrseq's HIGH-1 path: a
 * deliberate shutdown, no alarm, no OV. The lease must still go — firmware is
 * turning the rail off underneath it — but a technician must not be told a 26 V
 * latch tripped when none did.
 */
static void test_a_deliberate_rb_shutdown_reads_differently(void)
{
	board_t b;
	uint32_t sid;
	size_t obj_en;
	ev_out_t ev[8];
	size_t n;
	size_t k;

	subscribe_event();
	sid = session();

	board_init(&b);
	board_run_to_rb(&b);

	obj_en = hold("pwr.rb.en", 1, sid);

	b.in.rb_wanted = false;
	board_tick(&b, HK_TICK_MS);

	TEST_ASSERT_FALSE(pwrseq_ov_latched(&b.ctx));
	TEST_ASSERT_EQUAL_UINT32((uint32_t)STS_MP_VETO_BIT(STS_MP_VETO_RB_RAIL),
				 g_veto_pending);

	TEST_ASSERT_EQUAL_INT(1, service());
	TEST_ASSERT_NULL(mp_ovr_lease(&g_c.ovr, obj_en));

	n = read_events(ev, 8U);
	k = find_veto(ev, n, obj_en);
	TEST_ASSERT_NOT_EQUAL_size_t((size_t)-1, k);
	TEST_ASSERT_EQUAL_STRING(sts_mp_veto_reason(STS_MP_VETO_RB_RAIL),
				 ev[k].text);
	/* The distinction is the point: the OV sentence must not appear. */
	TEST_ASSERT_NULL(strstr(ev[k].text, "OV"));
}

/* ==================================================== 2. the shed ladder ==== */

/**
 * The PoE/thermal shed, and the one object a lease can actually be held on
 * today.
 *
 * mp_glue.c's obj_apply() answers -ENOTSUP for every platform-owned pin, so
 * `ui.panel.duty` — the LPTIM2_CH2 dimmer — is the only object in the veto
 * table the shipped image can currently grant. PWRSEQ_ACT_PANEL_LED_DIS is one
 * call to sts_panel_led_set(0), i.e. firmware writing the very register the
 * lease holds. This is the trigger that is live on the board as built.
 */
static void test_the_shed_ladder_withdraws_the_panel_dimmer(void)
{
	board_t b;
	uint32_t sid;
	size_t obj_duty;
	ev_out_t ev[8];
	size_t n;
	size_t k;

	subscribe_event();
	sid = session();

	board_init(&b);
	board_run_to_rb(&b);

	obj_duty = hold("ui.panel.duty", 80, sid);
	g_apply_n = 0U;

	/* Rung 1 sheds the display; rung 2 the panel LEDs. Real entry point. */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_shed_step(&b.ctx, b.in.mono_ms));
	TEST_ASSERT_EQUAL_INT(0, pwrseq_shed_step(&b.ctx, b.in.mono_ms));
	exec_pass(&b);

	TEST_ASSERT_EQUAL_INT(PWRSEQ_SHED_PANEL_LED,
			      (int)pwrseq_shed_level(&b.ctx));
	TEST_ASSERT_EQUAL_UINT32(
		(uint32_t)STS_MP_VETO_BIT(STS_MP_VETO_DISPLAY) |
			(uint32_t)STS_MP_VETO_BIT(STS_MP_VETO_PANEL_LED),
		g_veto_pending);

	/* Only the dimmer had a lease; `pwr.disp.en` and `pwr.panel.led.en`
	 * answer -ENOENT, which is the ordinary outcome and not an error. */
	TEST_ASSERT_EQUAL_INT(1, service());
	TEST_ASSERT_NULL(mp_ovr_lease(&g_c.ovr, obj_duty));
	TEST_ASSERT_EQUAL_UINT(1U, releases_of(obj_duty));

	n = read_events(ev, 8U);
	k = find_veto(ev, n, obj_duty);
	TEST_ASSERT_NOT_EQUAL_size_t((size_t)-1, k);
	TEST_ASSERT_EQUAL_STRING(sts_mp_veto_reason(STS_MP_VETO_PANEL_LED),
				 ev[k].text);
}

/* ================================================= 3. idempotence, safety === */

/** A veto with no lease is -ENOENT, is not an error, and changes nothing. */
static void test_a_veto_with_no_lease_is_a_normal_outcome(void)
{
	board_t b;

	subscribe_event();
	(void)session();

	board_init(&b);
	board_run_to_rb(&b);

	b.in.rb_ov_det = true;
	board_tick(&b, HK_TICK_MS);

	/* No lease anywhere; the drain still runs and withdraws nothing. */
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c.ovr));
	TEST_ASSERT_EQUAL_INT(0, service());
	TEST_ASSERT_EQUAL_UINT(0U, g_apply_n);
	TEST_ASSERT_EQUAL_size_t(0U, mp_stream_event_count(&g_c.st));
}

/**
 * Repeats coalesce, and that is why the seam is a bitmap and not a queue.
 *
 * The sequencer re-raises RB_VCC_GATE_DIS and RB_PWR_DIS on the same tick, and
 * further ticks with the latch still set produce more. One bit absorbs them all;
 * nothing can overflow and nothing can be dropped, which a bounded queue cannot
 * promise — and a dropped veto is an override left standing against a dead rail.
 */
static void test_repeat_requests_coalesce_and_never_overflow(void)
{
	board_t b;
	uint32_t sid;
	unsigned int i;

	sid = session();
	board_init(&b);
	board_run_to_rb(&b);

	b.in.rb_ov_det = true;
	for (i = 0U; i < 50U; i++) {
		board_tick(&b, HK_TICK_MS);
	}

	TEST_ASSERT_TRUE(g_veto_raised >= 2U);
	TEST_ASSERT_EQUAL_UINT32((uint32_t)STS_MP_VETO_BIT(STS_MP_VETO_RB_OV),
				 g_veto_pending);

	/* And the second drain of the same subject is a no-op, not a fault. The
	 * session is untouched throughout: a veto withdraws a lease, it does
	 * not evict the technician. */
	TEST_ASSERT_EQUAL_INT(0, service());
	TEST_ASSERT_EQUAL_INT(0, service());
	TEST_ASSERT_TRUE(mp_ovr_session_valid(&g_c.ovr, sid, g_now));
}

/** The drain claims the mask, so a consumed subject cannot fire twice. */
static void test_the_drain_claims_the_mask_before_it_works(void)
{
	board_t b;
	uint32_t sid;
	size_t obj_en;

	subscribe_event();
	sid = session();
	board_init(&b);
	board_run_to_rb(&b);
	obj_en = hold("pwr.rb.en", 1, sid);

	b.in.rb_ov_det = true;
	board_tick(&b, HK_TICK_MS);
	TEST_ASSERT_EQUAL_INT(1, service());
	TEST_ASSERT_NULL(mp_ovr_lease(&g_c.ovr, obj_en));
	TEST_ASSERT_EQUAL_UINT32(0U, g_veto_pending);

	/* Nothing left over: a second pass with no new action withdraws
	 * nothing and raises nothing. */
	board_tick(&b, HK_TICK_MS);
	TEST_ASSERT_EQUAL_UINT32(0U, g_veto_pending);
	TEST_ASSERT_EQUAL_INT(0, service());
}

/**
 * The veto is the EDGE; the interlock is the LEVEL. Both are needed and the
 * division of labour is deliberate.
 *
 * Once the rubidium is down, pwrseq emits no further RB actions — the
 * supervision block is gated on `rb_enabled` — so nothing raises another veto,
 * and a technician who simply re-issues the override would otherwise get it
 * back. What stops that is MP_ILK_RB_OV, evaluated at grant time from the state
 * mp_glue.c's prov_ilk() builds out of the RB_OV alarm.
 *
 * Asserting it here is what keeps someone from "fixing" the gap by making the
 * veto level-triggered, which would raise a subject on every 4 Hz sequencer
 * pass for as long as a rail stayed down.
 */
static void test_the_latch_then_refuses_a_regrant(void)
{
	board_t b;
	uint32_t sid;
	size_t obj_en;
	int obj;

	subscribe_event();
	sid = session();
	board_init(&b);
	board_run_to_rb(&b);
	obj_en = hold("pwr.rb.en", 1, sid);

	b.in.rb_ov_det = true;
	board_tick(&b, HK_TICK_MS);
	TEST_ASSERT_EQUAL_INT(1, service());
	TEST_ASSERT_NULL(mp_ovr_lease(&g_c.ovr, obj_en));

	/* The rail stays down, so the executor emits nothing more and raises
	 * nothing more. The veto is an edge and does not repeat. */
	board_tick(&b, HK_TICK_MS);
	TEST_ASSERT_EQUAL_UINT32(0U, g_veto_pending);

	/* The glue now reports the latch (prov_ilk() reads it from the RB_OV
	 * alarm), and the interlock refuses the re-grant outright. */
	TEST_ASSERT_TRUE(pwrseq_ov_latched(&b.ctx));
	g_ilk.rb_ov_latched = true;

	obj = mp_obj_find("pwr.rb.en");
	TEST_ASSERT_TRUE(obj >= 0);
	TEST_ASSERT_EQUAL_INT(-EPERM,
			      mp_ovr_grant(&g_c.ovr, (size_t)obj, 1, 0U, sid,
					   &g_ilk, g_now, NULL));
	TEST_ASSERT_NULL(mp_ovr_lease(&g_c.ovr, (size_t)obj));

	/* Turning the chain OFF stays permitted while latched — the interlock
	 * refuses assertion, not de-assertion. */
	TEST_ASSERT_EQUAL_INT(0,
			      mp_ovr_grant(&g_c.ovr, (size_t)obj, 0, 0U, sid,
					   &g_ilk, g_now, NULL));
}

/* ===================================================== 4. the policy table == */

/** Every id the table names resolves in the shipped manifest. */
static void test_every_vetoed_object_exists(void)
{
	unsigned int s;
	unsigned int mapped = 0U;

	for (s = (unsigned int)STS_MP_VETO_NONE + 1U;
	     s < (unsigned int)STS_MP_VETO_COUNT; s++) {
		const char *const *ids;
		size_t n = 0U;
		size_t i;

		ids = sts_mp_veto_objects((sts_mp_veto_t)s, &n);
		TEST_ASSERT_TRUE_MESSAGE(n > 0U,
					 "a subject that withdraws nothing is "
					 "a veto that does nothing");
		TEST_ASSERT_NOT_NULL(ids);
		for (i = 0U; i < n; i++) {
			int obj = mp_obj_find(ids[i]);

			TEST_ASSERT_TRUE_MESSAGE(obj >= 0, ids[i]);
			/* And it must be overridable, or no lease could ever
			 * exist to withdraw. */
			TEST_ASSERT_TRUE_MESSAGE(
				(mp_obj_at((size_t)obj)->flags &
				 MP_OF_OVERRIDE) != 0U,
				ids[i]);
			mapped++;
		}
	}
	TEST_ASSERT_TRUE(mapped >= 6U);
}

/** Every reason survives MP_EV_TEXT_MAX, so the wire carries the whole sentence. */
static void test_every_reason_fits_the_wire(void)
{
	unsigned int s;

	for (s = (unsigned int)STS_MP_VETO_NONE + 1U;
	     s < (unsigned int)STS_MP_VETO_COUNT; s++) {
		const char *r = sts_mp_veto_reason((sts_mp_veto_t)s);

		TEST_ASSERT_NOT_NULL(r);
		TEST_ASSERT_TRUE_MESSAGE(sts_mp_veto_reason_fits(
						 (sts_mp_veto_t)s),
					 r);
	}
	TEST_ASSERT_NULL(sts_mp_veto_reason(STS_MP_VETO_NONE));
	TEST_ASSERT_NULL(sts_mp_veto_reason(STS_MP_VETO_COUNT));
}

/**
 * The ON direction raises nothing, and the exclusion is load-bearing.
 *
 * Stages 6 and 7 emit GPS_PWR_EN, ANT_BIAS_EN, DISP_EN and PANEL_LED_EN long
 * before a session can exist. Mapping them would fire a veto on every boot and
 * bury the ones that mean something.
 */
static void test_only_the_off_direction_raises_a_veto(void)
{
	static const uint16_t on_actions[] = {
		PWRSEQ_ACT_GPS_PWR_EN,	  PWRSEQ_ACT_ANT_BIAS_EN,
		PWRSEQ_ACT_DISP_EN,	  PWRSEQ_ACT_PANEL_LED_EN,
		PWRSEQ_ACT_PANEL_LED_PWM, PWRSEQ_ACT_RB_PWR_EN,
		PWRSEQ_ACT_RB_VCC_GATE_EN,
	};
	unsigned int i;

	for (i = 0U; i < (sizeof(on_actions) / sizeof(on_actions[0])); i++) {
		TEST_ASSERT_EQUAL_INT_MESSAGE(
			STS_MP_VETO_NONE,
			sts_mp_veto_of_action(on_actions[i], false),
			pwrseq_action_name((pwrseq_action_t)on_actions[i]));
		TEST_ASSERT_EQUAL_INT(
			STS_MP_VETO_NONE,
			sts_mp_veto_of_action(on_actions[i], true));
	}
}

/** Every OFF action that touches an override object is mapped — none missed. */
static void test_every_off_action_on_an_override_object_is_mapped(void)
{
	TEST_ASSERT_EQUAL_INT(STS_MP_VETO_RB_OV,
			      sts_mp_veto_of_action(PWRSEQ_ACT_RB_PWR_DIS, true));
	TEST_ASSERT_EQUAL_INT(
		STS_MP_VETO_RB_OV,
		sts_mp_veto_of_action(PWRSEQ_ACT_RB_VCC_GATE_DIS, true));
	TEST_ASSERT_EQUAL_INT(
		STS_MP_VETO_RB_RAIL,
		sts_mp_veto_of_action(PWRSEQ_ACT_RB_PWR_DIS, false));
	TEST_ASSERT_EQUAL_INT(
		STS_MP_VETO_RB_RAIL,
		sts_mp_veto_of_action(PWRSEQ_ACT_RB_VCC_GATE_DIS, false));
	TEST_ASSERT_EQUAL_INT(
		STS_MP_VETO_ANT_BIAS,
		sts_mp_veto_of_action(PWRSEQ_ACT_ANT_BIAS_DIS, false));
	TEST_ASSERT_EQUAL_INT(STS_MP_VETO_DISPLAY,
			      sts_mp_veto_of_action(PWRSEQ_ACT_DISP_DIS, false));
	TEST_ASSERT_EQUAL_INT(
		STS_MP_VETO_PANEL_LED,
		sts_mp_veto_of_action(PWRSEQ_ACT_PANEL_LED_DIS, false));
	TEST_ASSERT_EQUAL_INT(
		STS_MP_VETO_GPS_RAIL,
		sts_mp_veto_of_action(PWRSEQ_ACT_GPS_PWR_DIS, false));

	/* Actions that drive no override object stay unmapped. */
	TEST_ASSERT_EQUAL_INT(STS_MP_VETO_NONE,
			      sts_mp_veto_of_action(PWRSEQ_ACT_POE_KILL, true));
	TEST_ASSERT_EQUAL_INT(STS_MP_VETO_NONE,
			      sts_mp_veto_of_action(PWRSEQ_ACT_PARK_DAC, true));
	TEST_ASSERT_EQUAL_INT(STS_MP_VETO_NONE,
			      sts_mp_veto_of_action(PWRSEQ_ACT_NONE, true));
	TEST_ASSERT_EQUAL_INT(STS_MP_VETO_NONE,
			      sts_mp_veto_of_action(0xFFFFU, true));
}

/**
 * The power-fail park list raises the rubidium subject through the same
 * executor, and is deliberately given no synchronous path.
 *
 * ~4.8 ms of hold-up against a 250 ms drain: the bits are staged and the board
 * stops before they can be consumed. Asserting it here records the decision as
 * a decision — nobody should later "fix" this by calling into the engine from
 * the PFI path.
 */
static void test_the_park_list_stages_a_veto_it_will_never_drain(void)
{
	board_t b;
	uint32_t sid;
	size_t obj_en;

	sid = session();
	board_init(&b);
	board_run_to_rb(&b);
	obj_en = hold("pwr.rb.en", 1, sid);

	TEST_ASSERT_EQUAL_INT(0, pwrseq_pfi(&b.ctx, b.in.mono_ms));
	exec_pass(&b);

	TEST_ASSERT_TRUE((g_veto_pending &
			  (uint32_t)STS_MP_VETO_BIT(STS_MP_VETO_RB_RAIL)) != 0U);
	/* Staged, not applied: no console pass has run, so the lease is still
	 * there — which is the honest state of a board that is losing power. */
	TEST_ASSERT_NOT_NULL(mp_ovr_lease(&g_c.ovr, obj_en));
}

/* ============================================ the shipped calls and guards == */

/*
 * The half the mirrors cannot check.
 *
 * exec_pass() and veto_drain_pass() prove the DECISIONS are right. They cannot
 * prove that platform/pwrseq_exec.c and console/mp_glue.c still make them,
 * because both are Zephyr translation units no host suite links — and "the code
 * is all there and nothing calls it" is precisely the defect this change
 * repairs.
 *
 * A substring scan is not enough. Asserting that `sts_mp_veto(` appears in
 * pwrseq_exec.c says the call exists and says nothing about what gates it;
 * moving it inside the wrong branch, or dropping the `pwrseq_ov_latched()`
 * term, would leave every test above green while the OV case silently reported
 * the generic sentence. So the scans below brace-match a span and assert the
 * guard, exactly as test_mp_events.c does — comments and string literals are
 * blanked in place so offsets stay 1:1 and a name quoted in prose can never
 * satisfy a claim about the code.
 */

#define SRC_MAX (192U * 1024U)

static char g_src[SRC_MAX];
static size_t g_src_len;
static char g_src_name[128];

typedef struct {
	size_t begin;
	size_t end;
} span_t;

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
				g_src[i] = ' ';
			} else if (c == '\'') {
				st = CHR;
				g_src[i] = ' ';
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
				g_src[i] = ' ';
				if ((i + 1U) < n) {
					g_src[i + 1U] = ' ';
					i++;
				}
				break;
			}
			if (((st == STR) && (c == '"')) ||
			    ((st == CHR) && (c == '\''))) {
				st = CODE;
			}
			g_src[i] = ' ';
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

/**
 * Offset of the @p nth (1-based) @p needle in @p s. Fails when there are fewer.
 *
 * Needed because string literals are blanked by load_source() — deliberately, so
 * a name quoted in prose or in a message cannot satisfy a claim about code — and
 * two LOG_ERR() calls in one function are therefore indistinguishable by text.
 * Ordinal is what is left, and it is enough: the claim is about which of them
 * the -ENOENT filter guards.
 */
static size_t offset_in_nth(const span_t *s, const char *needle, unsigned int nth)
{
	size_t k = strlen(needle);
	unsigned int seen = 0U;
	size_t i;
	char msg[256];

	for (i = s->begin; (k != 0U) && ((i + k) <= s->end); i++) {
		if (memcmp(&g_src[i], needle, k) == 0) {
			seen++;
			if (seen == nth) {
				return i;
			}
		}
	}
	(void)snprintf(msg, sizeof(msg),
		       "%s: fewer than %u occurrences of `%s` — fix this scan, "
		       "do not delete the assertion it feeds",
		       g_src_name, nth, needle);
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

/** The header of the innermost braced block enclosing @p at, within @p outer. */
static size_t guard_of(const span_t *outer, size_t at, size_t *out_len)
{
	size_t stack[32];
	size_t depth = 0U;
	size_t open;
	size_t i;

	TEST_ASSERT_TRUE((at >= outer->begin) && (at < outer->end));

	for (i = outer->begin; i < at; i++) {
		if (g_src[i] == '{') {
			TEST_ASSERT_TRUE(depth <
					 (sizeof(stack) / sizeof(stack[0])));
			stack[depth] = i;
			depth++;
		} else if (g_src[i] == '}') {
			TEST_ASSERT_TRUE_MESSAGE(depth > 0U, "brace underflow");
			depth--;
		}
	}

	if (depth == 0U) {
		open = outer->begin - 1U;
	} else {
		open = stack[depth - 1U];
	}

	i = open;
	while (i > 0U) {
		char c = g_src[i - 1U];

		if ((c == ';') || (c == '{') || (c == '}')) {
			break;
		}
		i--;
	}
	*out_len = open - i;
	return i;
}

/** Assert that what gates @p needle inside @p outer mentions @p term. */
static void guard_must_test(const span_t *outer, const char *needle,
			    const char *term)
{
	size_t at = offset_in(outer, needle);
	size_t len = 0U;
	size_t g = guard_of(outer, at, &len);
	span_t hdr;
	char msg[320];

	hdr.begin = g;
	hdr.end = g + len;

	(void)snprintf(msg, sizeof(msg),
		       "%s: `%s` is no longer gated on `%s` — an unguarded call "
		       "here is the defect, not a style change",
		       g_src_name, needle, term);
	TEST_ASSERT_TRUE_MESSAGE(count_in(&hdr, term) >= 1U, msg);
}

/** guard_must_test() for the @p nth occurrence of @p needle. */
static void guard_must_test_nth(const span_t *outer, const char *needle,
				unsigned int nth, const char *term)
{
	size_t at = offset_in_nth(outer, needle, nth);
	size_t len = 0U;
	size_t g = guard_of(outer, at, &len);
	span_t hdr;
	char msg[320];

	hdr.begin = g;
	hdr.end = g + len;

	(void)snprintf(msg, sizeof(msg),
		       "%s: occurrence %u of `%s` is no longer gated on `%s`",
		       g_src_name, nth, needle, term);
	TEST_ASSERT_TRUE_MESSAGE(count_in(&hdr, term) >= 1U, msg);
}

/* ------------------------------------------------------------ pwrseq_exec.c */

/**
 * The executor raises the veto, before the pin moves, on the OV term.
 *
 * Three separate claims, because three separate regressions are possible: the
 * call disappearing, the call losing its STS_MP_VETO_NONE filter (which would
 * hand the console a stream of no-op subjects on every action), and the
 * `pwrseq_ov_latched()` term being dropped — the last of which is invisible in
 * behaviour except that a technician is told the wrong cause.
 */
static void test_the_action_executor_still_raises_the_veto(void)
{
	span_t b;

	load_source("zephyr/platform/pwrseq_exec.c");
	b = fn_body("static void pwrseq_exec_action(const pwrseq_act_t *a)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_mp_veto_of_action("),
		"pwrseq_exec_action() no longer consults the veto policy");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "pwrseq_ov_latched(&pwrseq)"),
		"the over-voltage term is gone: an OV drop would report the "
		"sequencer's generic reason instead of the latch");
	guard_must_test(&b, "sts_mp_veto(subj)", "STS_MP_VETO_NONE");

	/* Raised BEFORE the pin moves, so the record describes an action the
	 * board is about to take rather than one it has forgotten. */
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&b, "sts_mp_veto(subj)") < offset_in(&b, "switch ("),
		"the veto moved below the action switch");
}

/**
 * The antenna-bias writer carries its own veto, on the OFF path only.
 *
 * It is the one writer of a rail an override can hold that does not pass
 * through the action queue, so it cannot inherit the executor's call. The early
 * return is what keeps gnssmgr_ant_reenable() — which re-enters this function
 * with `on` true — from withdrawing an override that wanted the bias on.
 */
static void test_the_antenna_bias_writer_vetoes_only_on_the_drop(void)
{
	span_t b;

	load_source("zephyr/platform/pwrseq_exec.c");
	b = fn_body("void sts_pwrseq_ant_bias_request(bool on)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_mp_veto(STS_MP_VETO_ANT_BIAS)"),
		"the antenna-bias drop no longer withdraws an override");
	/* At function scope, i.e. NOT inside the `on` branch... */
	guard_must_test(&b, "sts_mp_veto(STS_MP_VETO_ANT_BIAS)",
			"sts_pwrseq_ant_bias_request");
	/* ...and reached only after the enable path has returned. */
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&b, "if (on)") <
			offset_in(&b, "sts_mp_veto(STS_MP_VETO_ANT_BIAS)"),
		"the enable branch no longer precedes the veto");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&b, "pwrseq_rail_drive(") <
			offset_in(&b, "sts_mp_veto(STS_MP_VETO_ANT_BIAS)"),
		"the veto is raised before the pin is dropped");
	/* Exactly two: the not-started guard and the enable path's own. A third
	 * would mean a new path reaches the end of the function, and a first
	 * would mean the enable path now falls through into the veto. */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		2U, count_in(&b, "return;"),
		"the enable path's early return is gone, so a re-enable would "
		"now withdraw the override that asked for it");
	/*
	 * The supervisor's decision IS firmware's commanded level for PC9, and
	 * it is recorded as such.
	 *
	 * pwrseq's own `ant_bias_on` does not move when gnssmgr cuts the bias,
	 * so without this record a released `pwr.ant.bias.en` lease would
	 * restore the level pwrseq still believed in and re-energise the bias
	 * into a latched short. The drain reads exactly this field.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "pwrseq_rail_auto("),
		"the antenna supervisor no longer records firmware's commanded "
		"level, so a lapsed lease would restore pwrseq's stale belief");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&b, "pwrseq_rail_auto(") <
			offset_in(&b, "pwrseq_rail_drive("),
		"the level is recorded after the pin moves, so a drain between "
		"the two would restore the old one");
}

/* ---------------------------------------------------------------- mp_glue.c */

/**
 * The tick drains the veto, inside the lock it already holds, before mp_tick().
 *
 * The lock claim is the budget: sts_console.c's BUILD_ASSERT allows exactly one
 * timed lock wait per supervisor pass and it is the tick's own. A drain that
 * took a second one would break that assertion's arithmetic — so the scan pins
 * both that the drain is inside the existing wait and that there is still only
 * one k_mutex_lock in the function.
 */
static void test_the_tick_still_drains_the_veto_inside_its_own_lock(void)
{
	span_t b;
	span_t d;

	load_source("zephyr/console/mp_glue.c");
	b = fn_body("void sts_mp_tick(void)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "mp_veto_drain_locked()"),
		"sts_mp_tick() no longer drains the firmware veto");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "k_mutex_lock("),
		"a second timed lock wait appeared in the supervisor's pass; "
		"sts_console.c's BUILD_ASSERT budgets exactly one");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&b, "k_mutex_lock(&mp_lock") <
			offset_in(&b, "mp_veto_drain_locked()"),
		"the veto drain is no longer inside the engine lock");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&b, "mp_veto_drain_locked()") <
			offset_in(&b, "mp_tick(&mp)"),
		"the veto drain moved after mp_tick(), so a withdrawn lease "
		"reaches the wire a pass late");

	/* And the drain itself resolves ids through the manifest and calls the
	 * engine, rather than having grown a table of indices. */
	d = fn_body("static void mp_veto_drain_locked(void)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&d, "mp_veto(&mp,"),
		"the drain no longer calls mp_veto()");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&d, "mp_obj_find("),
		"the drain no longer resolves manifest ids by name");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&d, "atomic_set(&mp_veto_pending"),
		"the drain no longer claims the pending mask with a swap, so a "
		"subject raised during the drain can be lost");
	/*
	 * -ENOENT must stay a normal outcome, not a logged error. It is the
	 * ordinary answer — the board sheds loads far more often than a
	 * technician overrides one — so an unfiltered LOG_ERR here would put a
	 * line in the operator's log on every bring-up and every shed, which is
	 * how a real veto becomes invisible. The second LOG_ERR in this function
	 * is the mp_veto() one; the first belongs to the manifest-lookup miss.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(2U, count_in(&d, "LOG_ERR("),
				       "the drain's error reporting changed "
				       "shape; re-check which call the ENOENT "
				       "filter guards");
	guard_must_test_nth(&d, "LOG_ERR(", 2U, "ENOENT");
}

/** The producer never waits, and is not gated on anyone watching channel 9. */
static void test_the_producer_is_lock_free_and_always_armed(void)
{
	span_t b;

	load_source("zephyr/console/mp_glue.c");
	b = fn_body("void sts_mp_veto(sts_mp_veto_t subject)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "atomic_or(&mp_veto_pending"),
		"the veto producer no longer stages with an atomic OR");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "k_mutex_lock"),
		"the veto producer takes the engine mutex; io_scan and the "
		"discipline loop may not wait on it");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "k_spin_lock"),
		"the veto producer took a spinlock it does not need");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "sts_mp_ch_armed"),
		"the veto was gated on a channel subscription: firmware must "
		"withdraw an override whether or not anyone is watching");
}

/* --------------------------------------------------------------------- main */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_the_ov_latch_withdraws_a_rubidium_override);
	RUN_TEST(test_a_deliberate_rb_shutdown_reads_differently);
	RUN_TEST(test_the_shed_ladder_withdraws_the_panel_dimmer);

	RUN_TEST(test_a_veto_with_no_lease_is_a_normal_outcome);
	RUN_TEST(test_repeat_requests_coalesce_and_never_overflow);
	RUN_TEST(test_the_drain_claims_the_mask_before_it_works);
	RUN_TEST(test_the_latch_then_refuses_a_regrant);

	RUN_TEST(test_every_vetoed_object_exists);
	RUN_TEST(test_every_reason_fits_the_wire);
	RUN_TEST(test_only_the_off_direction_raises_a_veto);
	RUN_TEST(test_every_off_action_on_an_override_object_is_mapped);
	RUN_TEST(test_the_park_list_stages_a_veto_it_will_never_drain);

	RUN_TEST(test_the_action_executor_still_raises_the_veto);
	RUN_TEST(test_the_antenna_bias_writer_vetoes_only_on_the_drop);
	RUN_TEST(test_the_tick_still_drains_the_veto_inside_its_own_lock);
	RUN_TEST(test_the_producer_is_lock_free_and_always_armed);

	return UNITY_END();
}
