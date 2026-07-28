/*
 * STS1000 "Meridian" — the board's own edges, driven onto MP channel 0x09.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * WHAT THIS SUITE EXISTS TO CATCH, stated so it cannot be diluted later.
 *
 * `test_mp_stream.c` already proved the *encoder*: queue an mp_ev_t, call
 * mp_enc_events(), decode the CBOR. `test_mp_rpc.c` already proved the *seam*:
 * call mp_post_event() and the record comes out of mp_tick(). Both passed for
 * as long as the defect existed, because the defect was in neither — it was
 * that **nothing produced**. mp_post_event() was the only route from outside
 * core into mp_stream_eventf(), nothing called it, and --gc-sections dropped it
 * out of zephyr.elf. Five of the channel's eight event kinds — MP_EV_FAULT,
 * MP_EV_BUTTON, MP_EV_PROX, MP_EV_TOUCH, MP_EV_ALARM — had no producer anywhere
 * in the image. A technician subscribed, got an open channel that reported
 * diagnostic progress and their own override grants back to them, and sat in
 * silence through a power-good drop, an INA228 alert, an alarm, a button press
 * and a door opening. Silence was indistinguishable from a healthy box.
 *
 * So this suite starts at the PORT PINS. Every event below originates in a
 * GPIOF/GPIOG word handed to the real core/fault, is classified by the real
 * sts_io_dispatch_plan(), is staged in the real sts_mp_evq_t, is drained into a
 * real mp_ctx_t and comes back off the wire through a real mp_frame decoder. A
 * test that called mp_post_event() directly would have passed before this
 * change and would prove nothing.
 *
 * Two mirrors of Zephyr code exist here and are deliberately the ONLY routes
 * into the queue and out of it — scan_pass() for io_scan.c's dispatch and
 * drain_pass() for mp_glue.c's mp_event_drain_locked(). Both are a handful of
 * marshalling lines around decisions that live in headers this file compiles
 * for real. The half a mirror cannot check — that the .c files still make the
 * call at all, which is exactly the defect above — is checked by reading those
 * sources, the same way test_factory_policy.c checks its three planes.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "fault/fault.h"
#include "mp/mp.h"
#include "zephyr/console/sts_mp_evq.h"
#include "zephyr/platform/sts_io_policy.h"

#ifndef STS_APP_SRC_DIR
#error "STS_APP_SRC_DIR must be defined by tests/host/CMakeLists.txt"
#endif

#define SERIAL "STS1000-000042"

/** Staging depth under test. The firmware default is CONFIG_STS1000_MP_EVQ_STAGE
 *  (32); the property that matters is the drop rule, not the number, so this is
 *  small enough to fill from a test and still above FAULT_EVT_QUEUE_LEN's
 *  argument being about the *firmware's* value. */
#define STAGE_LEN 8U

/* ------------------------------------------------------------------ fixture */

static mp_ctx_t g_c;
static uint8_t g_scratch[8192];
static mp_reasm_t g_slots[2];
static uint8_t g_slot_buf[2][2048];
static char g_prev_ch[20 * 60];
static uint8_t g_prev_attr[20 * 60];

static cfg_ctx_t g_cfg;
static logr_t g_log;
static logr_rec_t g_log_slots[16];

static uint32_t g_now;

/* The board. Real core/fault, driven from raw port words. */
static fault_ctx_t g_f;
static uint16_t g_portf;
static uint16_t g_portg;

/* The staging queue, and the drain's bookkeeping (mp_events.c's ev_drops_
 * reported). */
static mp_ev_t g_stage[STAGE_LEN];
static sts_mp_evq_t g_q;
static uint32_t g_drops_reported;

/* The wire, and a second frame decoder to read it back. */
static uint8_t g_wire[32768];
static size_t g_wire_len;

static mp_frame_rx_t g_drx;
static mp_reasm_t g_dslots[2];
static uint8_t g_dslot_buf[2][2048];

/** Last message decoded per channel, and how many arrived. */
static uint8_t g_msg[MP_CH_COUNT][1024];
static size_t g_msg_len[MP_CH_COUNT];
static unsigned int g_msg_n[MP_CH_COUNT];

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
	(void)obj;
	(void)value;
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
	g_drops_reported = 0U;
	memset(g_msg_len, 0, sizeof(g_msg_len));
	memset(g_msg_n, 0, sizeof(g_msg_n));

	/* Idle board: every one of the 32 signals is active-low, so all pins
	 * high is "nothing asserted" (fault.h, FAULT_ACTIVE_LOW_MASK). */
	g_portf = 0xFFFFU;
	g_portg = 0xFFFFU;
	TEST_ASSERT_EQUAL_INT(0, fault_init(&g_f, NULL));

	TEST_ASSERT_EQUAL_INT(0, sts_mp_evq_init(&g_q, g_stage, STAGE_LEN));

	for (i = 0U; i < 2U; i++) {
		g_slots[i].buf = g_slot_buf[i];
		g_slots[i].cap = sizeof(g_slot_buf[i]);
		g_dslots[i].buf = g_dslot_buf[i];
		g_dslots[i].cap = sizeof(g_dslot_buf[i]);
	}
	TEST_ASSERT_EQUAL_INT(0, mp_frame_rx_init(&g_drx, g_dslots, 2U,
						  decoded_msg, NULL));

	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, NULL));
	TEST_ASSERT_EQUAL_INT(0, logr_init(&g_log, g_log_slots, 16U));

	memset(&w, 0, sizeof(w));
	w.tx = tx_cb;
	w.mono_ms = clock_cb;
	w.model = "STS1000";
	w.serial = SERIAL;
	w.fw_version = "1.2.3";
	w.boot_version = "0.9.0";
	w.board_id = SERIAL;
	w.apply = apply_cb;
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

	/*
	 * mp_mode_enter() queues an MP_EV_MODE record. It is a real event and
	 * one of the three kinds that already had a producer; here it would only
	 * add a second entry to every batch and blur the counts, so the fixture
	 * starts from a drained engine queue.
	 */
	{
		uint8_t sink[512];

		(void)mp_enc_events(&g_c.st, 0U, 0U, 0U, sink, sizeof(sink));
		TEST_ASSERT_EQUAL_size_t(0U, mp_stream_event_count(&g_c.st));
	}
}

void tearDown(void)
{
}

/** Subscribe a host to the event channel. */
static void subscribe_event(void)
{
	TEST_ASSERT_EQUAL_INT(0, mp_stream_sub(&g_c.st, (uint8_t)MP_CH_EVENT,
					       0U, g_now));
}

/* ------------------------------------------------- the producer's own route */

/**
 * One pass of platform/io_scan.c: sample both ports, let core/fault debounce,
 * then run every committed event through sts_io_dispatch_plan() and stage what
 * it says to stage.
 *
 * This is the ONLY route from a pin state to the queue in this file. A test
 * that reached sts_mp_evq_stage() with hand-built arguments would be back to
 * testing the queue, which is not where the defect was.
 */
static void scan_pass(uint32_t at_ms)
{
	fault_evt_t evt;

	g_now = at_ms;
	TEST_ASSERT_EQUAL_INT(0, fault_scan_input(&g_f, g_portf, g_portg, at_ms));

	while (fault_evt_get(&g_f, &evt) == 0) {
		sts_io_dispatch_t plan;

		sts_io_dispatch_plan(evt.type, evt.id, evt.edge, &plan);
		if (!plan.post_mp) {
			continue;
		}
		(void)sts_mp_evq_stage(&g_q, plan.mp_kind, plan.mp_sub, evt.id,
				       plan.mp_edge, 0, evt.mono_ms,
				       fault_sig_name((fault_sig_t)evt.id));
	}
}

/** Hold the current port state for @p ms milliseconds of 1 kHz scans. */
static void scan_hold(uint32_t ms)
{
	uint32_t i;

	for (i = 0U; i < ms; i++) {
		scan_pass(g_now + 1U);
	}
}

/**
 * sts_app.c's sts_alarm_set(), which is where an alarm's active state
 * transitions and therefore the only place MP_EV_ALARM can honestly come from.
 *
 * The edge test is the mirror; everything it is a mirror OF — fault_alarm_get()
 * before, fault_alarm_set() after — is the real module.
 */
static int alarm_set(uint8_t alarm_id, bool active)
{
	const fault_alarm_t *a;
	bool changed;
	int rc;

	if ((alarm_id < FAULT_SIG_COUNT) ||
	    (alarm_id >= (uint8_t)FAULT_ALARM_COUNT)) {
		return -EINVAL;
	}

	a = fault_alarm_get(&g_f, (fault_alarm_id_t)alarm_id);
	changed = (a != NULL) && (a->active != active);
	rc = fault_alarm_set(&g_f, (fault_alarm_id_t)alarm_id, active, g_now);

	if ((rc == 0) && changed) {
		(void)sts_mp_evq_stage(&g_q, (uint8_t)MP_EV_ALARM, 0U,
				       (uint16_t)alarm_id, active ? 1U : 0U, 0,
				       g_now, NULL);
	}
	return rc;
}

/* --------------------------------------------------------- the drain's route */

/**
 * mp_glue.c's mp_event_drain_locked(), followed by the tick it runs inside.
 *
 * Same order of operations: purge when nobody is subscribed, fold the staged
 * drop count into the engine's counter *before* the batch it belongs to, then
 * peek/post/pop so an event the engine has no room for stays staged.
 */
static void drain_pass(void)
{
	unsigned int n;
	uint32_t lost;

	if (!mp_stream_is_sub(&g_c.st, (uint8_t)MP_CH_EVENT)) {
		sts_mp_evq_reset(&g_q);
		g_drops_reported = g_q.dropped;
		return;
	}

	lost = g_q.dropped - g_drops_reported;
	g_drops_reported = g_q.dropped;
	if (lost != 0U) {
		TEST_ASSERT_EQUAL_INT(0,
				      mp_stream_event_drop_note(&g_c.st, lost));
	}

	for (n = 0U; n < (unsigned int)MP_EVQ_LEN; n++) {
		mp_ev_t ev;

		if (mp_stream_event_count(&g_c.st) >= (size_t)MP_EVQ_LEN) {
			break;
		}
		if (!sts_mp_evq_peek(&g_q, &ev)) {
			break;
		}
		if (mp_post_event(&g_c, ev.kind, ev.sub, ev.id, ev.edge,
				  ev.value, ev.mono_ms,
				  (ev.text[0] != '\0') ? ev.text : NULL) != 0) {
			break;
		}
		sts_mp_evq_pop(&g_q);
	}
}

/** Drain, tick, and decode whatever reached the wire. */
static void service(void)
{
	size_t from = g_wire_len;

	drain_pass();
	(void)mp_tick(&g_c);
	if (g_wire_len > from) {
		/* Returns the number of complete messages delivered. */
		TEST_ASSERT_TRUE(mp_frame_rx_input(&g_drx, &g_wire[from],
						   g_wire_len - from) >= 0);
	}
}

/* ------------------------------------------------------------ record readers */

/** Position @p r on the value of key @p key in the decoded event record. */
static void seek_event_key(mp_cbor_rd_t *r, uint64_t key)
{
	TEST_ASSERT_TRUE_MESSAGE(g_msg_len[MP_CH_EVENT] > 0U,
				 "no event record reached the wire");
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(r, g_msg[MP_CH_EVENT],
						 g_msg_len[MP_CH_EVENT]));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(r, key));
}

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

	seek_event_key(&r, 9U);
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

/** The record's key 8: events lost before the batch. */
static uint64_t read_dropped(void)
{
	mp_cbor_rd_t r;
	uint64_t v = 0U;

	seek_event_key(&r, 8U);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &v));
	return v;
}

/** Index of the first event of @p kind with @p id, or SIZE_MAX. */
static size_t find_ev(const ev_out_t *ev, size_t n, uint8_t kind, uint16_t id)
{
	size_t i;

	for (i = 0U; i < n; i++) {
		if ((ev[i].kind == (uint64_t)kind) &&
		    (ev[i].id == (uint64_t)id)) {
			return i;
		}
	}
	return (size_t)-1;
}

/* ============================================================ the wire ===== */

/**
 * A power-good collapse on PG7 (PoE) reaches the host as MP_EV_FAULT.
 *
 * The whole chain: a pin goes low, core/fault debounces it over its 5 ms
 * power-good window, sts_io_dispatch_plan() classifies it, the queue stages it,
 * the drain posts it, mp_tick() frames it, and a frame decoder reads the kind
 * and the signal number back out.
 */
static void test_pg_drop_reaches_the_wire(void)
{
	ev_out_t ev[8];
	size_t n;
	size_t k;

	subscribe_event();

	scan_pass(20000U);
	g_portg &= (uint16_t)~(1U << (FAULT_SIG_PG_POE - 16U));
	scan_hold(10U);

	TEST_ASSERT_EQUAL_UINT16(1U, sts_mp_evq_count(&g_q));
	service();

	TEST_ASSERT_EQUAL_UINT(1U, g_msg_n[MP_CH_EVENT]);
	n = read_events(ev, 8U);
	TEST_ASSERT_EQUAL_size_t(1U, n);

	k = find_ev(ev, n, (uint8_t)MP_EV_FAULT, (uint16_t)FAULT_SIG_PG_POE);
	TEST_ASSERT_NOT_EQUAL_size_t((size_t)-1, k);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)FAULT_EVT_PG_FAULT, ev[k].sub);
	TEST_ASSERT_EQUAL_UINT64(1U, ev[k].edge);
	/* The identity a technician reads, carried as text as well as a
	 * number, so a host with no signal table still names it. */
	TEST_ASSERT_EQUAL_STRING(fault_sig_name(FAULT_SIG_PG_POE), ev[k].text);
}

/** ...and its recovery, which is the half the log deliberately does not carry. */
static void test_pg_recovery_reaches_the_wire(void)
{
	ev_out_t ev[8];
	size_t n;

	subscribe_event();

	scan_pass(20000U);
	g_portg &= (uint16_t)~(1U << (FAULT_SIG_PG_POE - 16U));
	scan_hold(10U);
	service();
	TEST_ASSERT_EQUAL_UINT(1U, g_msg_n[MP_CH_EVENT]);

	g_portg |= (uint16_t)(1U << (FAULT_SIG_PG_POE - 16U));
	scan_hold(10U);
	service();

	TEST_ASSERT_EQUAL_UINT(2U, g_msg_n[MP_CH_EVENT]);
	n = read_events(ev, 8U);
	TEST_ASSERT_EQUAL_size_t(1U, n);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)MP_EV_FAULT, ev[0].kind);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)FAULT_SIG_PG_POE, ev[0].id);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)FAULT_EVT_PG_RECOVER, ev[0].sub);
	TEST_ASSERT_EQUAL_UINT64(0U, ev[0].edge);
}

/** A panel button press and release reach the host as MP_EV_BUTTON. */
static void test_button_edges_reach_the_wire(void)
{
	ev_out_t ev[8];
	size_t n;

	subscribe_event();

	scan_pass(20000U);
	g_portf &= (uint16_t)~(1U << FAULT_SIG_BUTTON_3);
	scan_hold(30U); /* button_debounce_ms = 25 */
	service();

	TEST_ASSERT_EQUAL_UINT(1U, g_msg_n[MP_CH_EVENT]);
	n = read_events(ev, 8U);
	TEST_ASSERT_EQUAL_size_t(1U, n);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)MP_EV_BUTTON, ev[0].kind);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)FAULT_SIG_BUTTON_3, ev[0].id);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)FAULT_EVT_BUTTON, ev[0].sub);
	TEST_ASSERT_EQUAL_UINT64(1U, ev[0].edge);

	g_portf |= (uint16_t)(1U << FAULT_SIG_BUTTON_3);
	scan_hold(30U);
	service();

	TEST_ASSERT_EQUAL_UINT(2U, g_msg_n[MP_CH_EVENT]);
	n = read_events(ev, 8U);
	TEST_ASSERT_EQUAL_size_t(1U, n);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)MP_EV_BUTTON, ev[0].kind);
	TEST_ASSERT_EQUAL_UINT64(0U, ev[0].edge);
}

/**
 * A held button's long-press and auto-repeat arrive as MP_EV_BUTTON too, told
 * apart by `sub` — which is why `sub` carries the fault_evt_type_t rather than
 * being left zero.
 */
static void test_long_press_and_repeat_are_distinguishable(void)
{
	ev_out_t ev[16];
	size_t n;
	size_t i;
	bool saw_long = false;
	bool saw_repeat = false;

	subscribe_event();

	scan_pass(20000U);
	g_portf &= (uint16_t)~(1U << FAULT_SIG_BUTTON_1);
	scan_hold(1400U); /* long_press 800, repeat_delay 1000, period 250 */
	service();

	n = read_events(ev, 16U);
	TEST_ASSERT_TRUE(n >= 3U);
	for (i = 0U; i < n; i++) {
		TEST_ASSERT_EQUAL_UINT64((uint64_t)MP_EV_BUTTON, ev[i].kind);
		TEST_ASSERT_EQUAL_UINT64((uint64_t)FAULT_SIG_BUTTON_1,
					 ev[i].id);
		saw_long |= (ev[i].sub == (uint64_t)FAULT_EVT_BUTTON_LONG);
		saw_repeat |= (ev[i].sub == (uint64_t)FAULT_EVT_BUTTON_REPEAT);
	}
	TEST_ASSERT_TRUE_MESSAGE(saw_long, "no long-press on the event channel");
	TEST_ASSERT_TRUE_MESSAGE(saw_repeat, "no auto-repeat on the event channel");
}

/** The reed switch reaches the host as MP_EV_PROX, both edges. */
static void test_reed_switch_reaches_the_wire(void)
{
	ev_out_t ev[8];
	size_t n;

	subscribe_event();

	scan_pass(20000U);
	g_portf &= (uint16_t)~(1U << FAULT_SIG_PROX_WAKE);
	scan_hold(60U); /* prox_debounce_ms = 50 */
	service();

	n = read_events(ev, 8U);
	TEST_ASSERT_EQUAL_size_t(1U, n);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)MP_EV_PROX, ev[0].kind);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)FAULT_SIG_PROX_WAKE, ev[0].id);
	TEST_ASSERT_EQUAL_UINT64(1U, ev[0].edge);

	g_portf |= (uint16_t)(1U << FAULT_SIG_PROX_WAKE);
	scan_hold(60U);
	service();
	n = read_events(ev, 8U);
	TEST_ASSERT_EQUAL_size_t(1U, n);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)MP_EV_PROX, ev[0].kind);
	TEST_ASSERT_EQUAL_UINT64(0U, ev[0].edge);
}

/**
 * The touch INT reaches the host as MP_EV_TOUCH — on the assert, and ONLY on
 * the assert, because core/fault never emits a release.
 *
 * This is the one kind whose coverage of the channel is narrower than "both
 * edges", and it is asserted here rather than glossed: fault.c's
 * FAULT_CLASS_TOUCH dispatch pushes an event only when `asserted` is true, so a
 * release never becomes a fault_evt_t and there is nothing for any consumer to
 * carry. The dispatcher's DEASSERT arm is still correct — it would stage a
 * release the day core/fault produces one — and the second half of this test
 * pins that, so the day the upstream behaviour changes this suite says what
 * changed rather than silently starting to emit a new record.
 */
static void test_touch_int_reaches_the_wire_on_assert(void)
{
	ev_out_t ev[8];
	sts_io_dispatch_t plan;
	size_t n;

	subscribe_event();

	scan_pass(20000U);
	g_portf &= (uint16_t)~(1U << FAULT_SIG_TOUCH_INT);
	scan_hold(10U);
	g_portf |= (uint16_t)(1U << FAULT_SIG_TOUCH_INT);
	scan_hold(10U);
	service();

	n = read_events(ev, 8U);
	TEST_ASSERT_EQUAL_size_t(1U, n);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)MP_EV_TOUCH, ev[0].kind);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)FAULT_SIG_TOUCH_INT, ev[0].id);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)FAULT_EVT_TOUCH, ev[0].sub);
	TEST_ASSERT_EQUAL_UINT64(1U, ev[0].edge);

	/*
	 * The release is missing upstream, not here: the plan still says to
	 * stage one, and still diverges from the UI's suppression of it.
	 */
	sts_io_dispatch_plan((uint8_t)FAULT_EVT_TOUCH,
			     (uint8_t)FAULT_SIG_TOUCH_INT,
			     (uint8_t)FAULT_EDGE_DEASSERT, &plan);
	TEST_ASSERT_FALSE(plan.post_ui);
	TEST_ASSERT_TRUE(plan.post_mp);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_EV_TOUCH, plan.mp_kind);
	TEST_ASSERT_EQUAL_UINT8(0U, plan.mp_edge);
}

/** An INA228 ALERT reaches the host as MP_EV_FAULT with the monitor's signal. */
static void test_ina_alert_reaches_the_wire(void)
{
	ev_out_t ev[8];
	size_t n;

	subscribe_event();

	scan_pass(20000U);
	g_portg &= (uint16_t)~(1U << (FAULT_SIG_INA_ALERT_OCXO - 16U));
	scan_hold(10U); /* ina_debounce_samples = 2 */
	service();

	n = read_events(ev, 8U);
	TEST_ASSERT_EQUAL_size_t(1U, n);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)MP_EV_FAULT, ev[0].kind);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)FAULT_SIG_INA_ALERT_OCXO, ev[0].id);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)FAULT_EVT_INA_ALERT, ev[0].sub);
	TEST_ASSERT_EQUAL_UINT64(1U, ev[0].edge);
}

/**
 * A software alarm's assert and clear reach the host as MP_EV_ALARM, and a
 * repeated set at the same level does NOT.
 *
 * The second half is the point: almost every caller of sts_alarm_set() is a
 * level evaluation on a periodic loop, so an implementation that posted on
 * every call would put a 1 Hz stream of "gnss-lost is still lost" on a channel
 * whose value is that something on it means something changed.
 */
static void test_alarm_transitions_reach_the_wire_once(void)
{
	ev_out_t ev[8];
	size_t n;

	subscribe_event();

	g_now = 20000U;
	TEST_ASSERT_EQUAL_INT(0, alarm_set((uint8_t)FAULT_ALARM_GNSS_LOST, true));
	TEST_ASSERT_EQUAL_UINT16(1U, sts_mp_evq_count(&g_q));

	/* Four more level evaluations at the same level: no new events. */
	TEST_ASSERT_EQUAL_INT(0, alarm_set((uint8_t)FAULT_ALARM_GNSS_LOST, true));
	TEST_ASSERT_EQUAL_INT(0, alarm_set((uint8_t)FAULT_ALARM_GNSS_LOST, true));
	TEST_ASSERT_EQUAL_INT(0, alarm_set((uint8_t)FAULT_ALARM_GNSS_LOST, true));
	TEST_ASSERT_EQUAL_INT(0, alarm_set((uint8_t)FAULT_ALARM_GNSS_LOST, true));
	TEST_ASSERT_EQUAL_UINT16(1U, sts_mp_evq_count(&g_q));

	service();
	n = read_events(ev, 8U);
	TEST_ASSERT_EQUAL_size_t(1U, n);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)MP_EV_ALARM, ev[0].kind);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)FAULT_ALARM_GNSS_LOST, ev[0].id);
	TEST_ASSERT_EQUAL_UINT64(1U, ev[0].edge);
	TEST_ASSERT_EQUAL_UINT64(20000U, ev[0].mono_ms);

	g_now = 25000U;
	TEST_ASSERT_EQUAL_INT(0, alarm_set((uint8_t)FAULT_ALARM_GNSS_LOST, false));
	service();
	n = read_events(ev, 8U);
	TEST_ASSERT_EQUAL_size_t(1U, n);
	TEST_ASSERT_EQUAL_UINT64((uint64_t)MP_EV_ALARM, ev[0].kind);
	TEST_ASSERT_EQUAL_UINT64(0U, ev[0].edge);
	TEST_ASSERT_EQUAL_UINT64(25000U, ev[0].mono_ms);
}

/** Scanned-signal ids are refused by the alarm producer, so they cannot be
 *  reported twice — once as MP_EV_FAULT and again as MP_EV_ALARM. */
static void test_scanned_signals_are_not_alarm_events(void)
{
	subscribe_event();
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      alarm_set((uint8_t)FAULT_SIG_PG_POE, true));
	TEST_ASSERT_EQUAL_UINT16(0U, sts_mp_evq_count(&g_q));
}

/**
 * The record carries the SCAN's timestamp, not the drain's.
 *
 * The drain runs on the console supervisor up to 250 ms after the edge. An
 * event stream stamped at drain time cannot order a button press against the
 * rail collapse it was a response to, which is most of what the channel is for.
 */
static void test_timestamp_is_the_edge_not_the_drain(void)
{
	ev_out_t ev[8];
	size_t n;

	subscribe_event();

	scan_pass(20000U);
	g_portg &= (uint16_t)~(1U << (FAULT_SIG_PG_5V_PSU - 16U));
	scan_hold(10U);

	/* The supervisor gets to it a quarter of a second later. */
	g_now += 250U;
	service();

	n = read_events(ev, 8U);
	TEST_ASSERT_EQUAL_size_t(1U, n);
	TEST_ASSERT_TRUE(ev[0].mono_ms <= 20010U);
	TEST_ASSERT_TRUE(ev[0].mono_ms >= 20000U);
	TEST_ASSERT_NOT_EQUAL_UINT64((uint64_t)g_now, ev[0].mono_ms);
}

/* ============================================================= overflow ==== */

/**
 * Overflow drops the NEWEST record, counts it, and the count reaches the host.
 *
 * An event channel that silently loses the event a technician is waiting for is
 * the original defect in a new place, so this asserts all three: the oldest
 * events survive (a cascade keeps its root cause), the counter moves, and key 8
 * of the record on the wire says a gap happened.
 */
static void test_overflow_drops_newest_and_is_counted(void)
{
	ev_out_t ev[16];
	size_t n;
	unsigned int i;

	subscribe_event();
	scan_pass(20000U);

	/* Nine buttons' worth of edges into an eight-deep queue: seven panel
	 * buttons plus the encoder switch is eight, so the ninth event is the
	 * encoder's release. */
	for (i = 0U; i < 7U; i++) {
		g_portf &= (uint16_t)~(1U << (FAULT_SIG_BUTTON_1 + i));
	}
	g_portf &= (uint16_t)~(1U << FAULT_SIG_ENC_BUTTON);
	scan_hold(30U);
	TEST_ASSERT_EQUAL_UINT16(STAGE_LEN, sts_mp_evq_count(&g_q));
	TEST_ASSERT_EQUAL_UINT32(0U, g_q.dropped);

	g_portf |= (uint16_t)(1U << FAULT_SIG_ENC_BUTTON);
	scan_hold(30U);

	/* The newest went, the eight oldest stayed. */
	TEST_ASSERT_EQUAL_UINT16(STAGE_LEN, sts_mp_evq_count(&g_q));
	TEST_ASSERT_EQUAL_UINT32(1U, g_q.dropped);

	service();

	TEST_ASSERT_EQUAL_UINT64(1U, read_dropped());
	n = read_events(ev, 16U);
	TEST_ASSERT_EQUAL_size_t((size_t)STAGE_LEN, n);
	for (i = 0U; i < (unsigned int)n; i++) {
		/* Every surviving record is a press, not the lost release. */
		TEST_ASSERT_EQUAL_UINT64((uint64_t)MP_EV_BUTTON, ev[i].kind);
		TEST_ASSERT_EQUAL_UINT64(1U, ev[i].edge);
	}
}

/** The drop count is reported once, not on every subsequent record. */
static void test_drop_count_is_not_re_reported(void)
{
	subscribe_event();
	scan_pass(20000U);

	{
		unsigned int i;

		for (i = 0U; i < 7U; i++) {
			g_portf &= (uint16_t)~(1U << (FAULT_SIG_BUTTON_1 + i));
		}
		g_portf &= (uint16_t)~(1U << FAULT_SIG_ENC_BUTTON);
		scan_hold(30U);
		g_portf |= (uint16_t)(1U << FAULT_SIG_ENC_BUTTON);
		scan_hold(30U);
	}
	service();
	TEST_ASSERT_EQUAL_UINT64(1U, read_dropped());

	/* A later, unrelated edge must not re-announce the same loss. */
	g_portf &= (uint16_t)~(1U << FAULT_SIG_ENC_BUTTON);
	scan_hold(30U);
	service();
	TEST_ASSERT_EQUAL_UINT64(0U, read_dropped());
}

/**
 * An engine queue with no room leaves the remainder STAGED rather than losing
 * it between the two queues.
 *
 * MP_EVQ_LEN is 24 and the staging queue here is 8, so the case is reached by
 * filling the engine's queue with a host that is not being ticked.
 */
static void test_engine_backpressure_keeps_events_staged(void)
{
	subscribe_event();
	scan_pass(20000U);

	/*
	 * Fill the engine's queue without draining it to the wire. mp_mode_enter()
	 * has already put one MP_EV_MODE in it, so top it up rather than assuming
	 * it started empty.
	 */
	while (mp_stream_event_count(&g_c.st) < (size_t)MP_EVQ_LEN) {
		TEST_ASSERT_EQUAL_INT(0,
				      mp_post_event(&g_c, (uint8_t)MP_EV_MODE,
						    0U, 0U, 1U, 0, g_now,
						    NULL));
	}
	TEST_ASSERT_EQUAL_size_t((size_t)MP_EVQ_LEN,
				 mp_stream_event_count(&g_c.st));

	g_portf &= (uint16_t)~(1U << FAULT_SIG_BUTTON_2);
	scan_hold(30U);
	TEST_ASSERT_EQUAL_UINT16(1U, sts_mp_evq_count(&g_q));

	/* The drain cannot place it, so it must still be staged, and nothing
	 * may be counted as dropped — nothing was lost. */
	drain_pass();
	TEST_ASSERT_EQUAL_UINT16(1U, sts_mp_evq_count(&g_q));
	TEST_ASSERT_EQUAL_UINT32(0U, g_q.dropped);
	TEST_ASSERT_EQUAL_UINT32(0U, mp_stream_event_dropped(&g_c.st));

	/* Once the engine has room the staged event goes out. */
	service();
	service();
	TEST_ASSERT_EQUAL_UINT16(0U, sts_mp_evq_count(&g_q));
}

/**
 * With nobody subscribed the backlog is discarded rather than delivered stale,
 * and discarding it is not counted as a loss.
 */
static void test_unsubscribed_backlog_is_purged_not_counted(void)
{
	subscribe_event();
	scan_pass(20000U);
	g_portf &= (uint16_t)~(1U << FAULT_SIG_BUTTON_4);
	scan_hold(30U);
	TEST_ASSERT_EQUAL_UINT16(1U, sts_mp_evq_count(&g_q));

	TEST_ASSERT_EQUAL_INT(0, mp_stream_unsub(&g_c.st, (uint8_t)MP_CH_EVENT));
	service();

	TEST_ASSERT_EQUAL_UINT16(0U, sts_mp_evq_count(&g_q));
	TEST_ASSERT_EQUAL_UINT32(0U, g_q.dropped);
	TEST_ASSERT_EQUAL_UINT(0U, g_msg_n[MP_CH_EVENT]);
	TEST_ASSERT_EQUAL_UINT32(0U, mp_stream_event_dropped(&g_c.st));
}

/** A kind core/mp would not accept is refused at the producer, not at the
 *  drain, so a caller's mistake cannot occupy a slot a real event needs. */
static void test_bad_kind_is_refused_at_the_producer(void)
{
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      sts_mp_evq_stage(&g_q,
					       (uint8_t)MP_EV_KIND_COUNT, 0U,
					       1U, 1U, 0, 1000U, NULL));
	TEST_ASSERT_EQUAL_UINT16(0U, sts_mp_evq_count(&g_q));
	TEST_ASSERT_EQUAL_UINT32(0U, g_q.dropped);
	TEST_ASSERT_EQUAL_UINT32(0U, g_q.staged);
}

/** Long text is truncated, not overrun, and stays NUL-terminated on the wire. */
static void test_text_is_truncated_not_overrun(void)
{
	ev_out_t ev[4];
	static const char *const longtext =
		"a signal name far longer than the record allows";

	subscribe_event();
	TEST_ASSERT_EQUAL_INT(0,
			      sts_mp_evq_stage(&g_q, (uint8_t)MP_EV_FAULT, 0U,
					       7U, 1U, 0, 1000U, longtext));
	service();
	TEST_ASSERT_EQUAL_size_t(1U, read_events(ev, 4U));
	TEST_ASSERT_EQUAL_size_t((size_t)(MP_EV_TEXT_MAX - 1U),
				 strlen(ev[0].text));
	TEST_ASSERT_EQUAL_UINT8_ARRAY(longtext, ev[0].text, MP_EV_TEXT_MAX - 1U);
}

/* ============================================ the shipped calls and guards == */

/*
 * The half a behavioural mirror cannot check.
 *
 * scan_pass(), alarm_set() and drain_pass() above prove the DECISIONS are
 * right. They cannot prove that platform/io_scan.c, sts_app.c and
 * console/mp_glue.c still make those decisions, because all three are Zephyr
 * translation units no host suite links — and "the code is all there and
 * nothing calls it" is precisely the defect this whole change repairs.
 *
 * A SUBSTRING SCAN IS NOT ENOUGH, and the first version of this section was
 * one. Asserting that `sts_mp_post_event((uint8_t)MP_EV_ALARM` appears in
 * sts_app.c says the call exists; it says nothing about the condition that
 * gates it. Changing `if ((rc == 0) && changed)` to `if ((rc == 0) && true)`
 * left every test in this file green, and the consequence is not cosmetic:
 * almost every caller of sts_alarm_set() is a level evaluation on a periodic
 * loop, so an unguarded post emits one ALARM record per pass per active alarm,
 * floods the 32-record staging queue and drives the drop counter — turning the
 * channel a technician relies on into noise at exactly the moment the board is
 * unhealthy.
 *
 * So the scans below brace-match a span and assert the guard, not the call:
 *
 *   fn_body()          the braced body of one named function, so a decision
 *                      that moved out from under a claim stops satisfying it;
 *   guard_of()         the header of the innermost block enclosing an offset —
 *                      what actually gates a statement, independent of layout;
 *   count_in/offset_in scoped occurrence and ordering.
 *
 * Comments and string literals are blanked in place, one space per character
 * with newlines preserved, so byte offsets stay 1:1 with the file and a brace
 * inside a comment cannot close a body early — the same false green by a
 * different route. It also means a function name quoted in prose can never
 * satisfy a claim about the code.
 *
 * The model is test_mp_fw.c's section 9, which reads fwupd_glue.c the same way
 * and for the same reason.
 */

#define SRC_MAX (192U * 1024U)

static char g_src[SRC_MAX];
static size_t g_src_len;
static char g_src_name[128];

/** A half-open [begin, end) octet range of g_src. */
typedef struct {
	size_t begin;
	size_t end;
} span_t;

/**
 * Read @p rel (relative to app/src), blanking comments and literals.
 *
 * Absent, truncated or implausibly small is a FAILURE and never a tolerated
 * default: a scan that cannot find its subject must not report agreement
 * with it.
 */
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

/** The whole loaded file, as a span. */
static span_t whole_file(void)
{
	span_t s = { 0U, g_src_len };

	return s;
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

/** Offset of the first @p needle in @p s. Fails when absent. */
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
	(void)snprintf(msg, sizeof(msg), "%s: `%s` is gone — fix this scan, do "
				         "not delete the assertion it feeds",
		       g_src_name, needle);
	TEST_FAIL_MESSAGE(msg);
	return 0U;
}

/**
 * The braced body of the function whose definition begins with @p sig.
 *
 * Scoped on purpose: a guard found somewhere else in the file must not satisfy
 * a claim about this function, or the scan goes on agreeing after the decision
 * has been moved out from under it.
 */
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
 * The header of the innermost braced block enclosing @p at, within @p outer.
 *
 * "Header" is the source between the previous statement boundary and the `{`
 * that opens the block — i.e. `if (...) `, `for (...) `, or the function
 * signature when nothing else encloses the statement. Asserting on THIS rather
 * than on a literal `if (...) {` string is what makes the check about the
 * condition rather than about the formatting: reindent it, split it across
 * lines, add a clause, and the claim still holds; drop the term the guard rests
 * on and it does not.
 *
 * @p out_len receives the header length; the return is its offset.
 */
static size_t guard_of(const span_t *outer, size_t at, size_t *out_len)
{
	size_t stack[32];
	size_t depth = 0U;
	size_t open;
	size_t i;

	TEST_ASSERT_TRUE((at >= outer->begin) && (at < outer->end));

	for (i = outer->begin; i < at; i++) {
		if (g_src[i] == '{') {
			TEST_ASSERT_TRUE(depth < (sizeof(stack) / sizeof(stack[0])));
			stack[depth] = i;
			depth++;
		} else if (g_src[i] == '}') {
			TEST_ASSERT_TRUE_MESSAGE(depth > 0U, "brace underflow");
			depth--;
		}
	}

	/* Nothing inside the span encloses it: the function's own body does. */
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

/* --------------------------------------------------------------- sts_app.c */

/**
 * MP_EV_ALARM is posted, and posted ONLY on a transition.
 *
 * The guard is the whole value of the kind. Without it every periodic
 * re-assertion of a live alarm becomes a record, and a board with three alarms
 * up produces twelve records a second on a channel whose meaning is that
 * something changed.
 */
static void test_alarm_set_posts_only_on_a_transition(void)
{
	span_t b;

	load_source("zephyr/sts_app.c");
	b = fn_body("int sts_alarm_set(uint8_t alarm_id, bool active)");

	/* The edge is computed from the module that owns it, under the same
	 * lock as the write, or it is two racing samples rather than an edge. */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "fault_alarm_get(sts_fault()"),
		"sts_alarm_set() no longer reads the previous alarm state");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "a->active != active"),
		"sts_alarm_set() no longer derives the edge from the alarm table");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_mp_post_event((uint8_t)MP_EV_ALARM"),
		"sts_alarm_set() no longer posts MP_EV_ALARM exactly once");

	guard_must_test(&b, "sts_mp_post_event((uint8_t)MP_EV_ALARM", "changed");

	/* And the post is outside the fault mutex: a caller may already hold
	 * another lock (mp_tunnel.c raises TAMPER inside the engine lock), so
	 * nesting a second one under it is a cycle waiting for a third caller. */
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&b, "sts_fault_unlock();") <
			offset_in(&b, "sts_mp_post_event((uint8_t)MP_EV_ALARM"),
		"MP_EV_ALARM is now posted inside the fault lock");
}

/* -------------------------------------------------------------- io_scan.c */

/** The 1 kHz scan posts what the plan says to post, gated on the plan. */
static void test_io_scan_posts_what_the_plan_decided(void)
{
	span_t b;

	load_source("zephyr/platform/io_scan.c");
	b = fn_body("static void io_scan_dispatch(const fault_evt_t *evt)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_mp_post_event(plan.mp_kind"),
		"io_scan_dispatch() no longer posts the MP event exactly once");

	guard_must_test(&b, "sts_mp_post_event(plan.mp_kind", "plan.post_mp");

	/*
	 * The scan's timestamp, not the drain's. A record stamped when the
	 * console supervisor got to it cannot order a button press against the
	 * rail collapse it was a response to, and nothing at runtime would say
	 * so — the events still arrive, just uncorrelatable.
	 */
	{
		size_t at = offset_in(&b, "sts_mp_post_event(plan.mp_kind");
		span_t call;

		call.begin = at;
		call.end = at + 240U;
		if (call.end > b.end) {
			call.end = b.end;
		}
		TEST_ASSERT_EQUAL_UINT_MESSAGE(
			1U, count_in(&call, "evt->mono_ms"),
			"the MP event no longer carries the scan's timestamp");
	}
}

/* -------------------------------------------------------------- mp_glue.c */

/**
 * The tick drains the staging queue, before mp_tick(), inside its own lock.
 *
 * Ordering is not cosmetic: draining after mp_tick() would put every event a
 * console pass late for no reason. And the lock count is the supervisor's
 * budget — sts_console.c's BUILD_ASSERT is built on exactly one timed wait per
 * pass, so a second one here silently breaks the override dead-man's revert
 * deadline rather than failing the build.
 */
static void test_the_tick_drains_before_it_ticks_and_waits_once(void)
{
	span_t t;
	span_t d;

	load_source("zephyr/console/mp_glue.c");
	t = fn_body("void sts_mp_tick(void)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&t, "mp_event_drain_locked();"),
		"sts_mp_tick() no longer drains the event queue exactly once");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&t, "mp_event_drain_locked();") <
			offset_in(&t, "(void)mp_tick(&mp);"),
		"the event drain no longer runs before mp_tick()");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&t, "k_mutex_lock("),
		"sts_mp_tick() no longer takes exactly one lock — the console "
		"pass budget in sts_console.c's BUILD_ASSERT assumes one");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U,
		count_in(&t, "k_mutex_lock(&mp_lock, K_MSEC(STS_MP_TICK_LOCK_MS))"),
		"sts_mp_tick()'s one lock wait is no longer the budgeted one");

	d = fn_body("static void mp_event_drain_locked(void)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&d, "k_mutex_lock("),
		"the event drain now takes a lock of its own; it runs inside "
		"the tick's and must not add a second wait");
}

/**
 * The drain's three guards, each of which is silent when it is wrong.
 *
 *   - the purge is gated on nobody being subscribed. Ungated, it discards the
 *     backlog it was about to deliver;
 *   - room is checked BEFORE the post. mp_stream_event() counts a drop on
 *     -ENOSPC, so offering an event to a full engine queue and then keeping it
 *     staged reports a loss on key 8 for an event that was never lost — and the
 *     one counter a technician has to be able to believe is the one that says
 *     "there is a gap here";
 *   - the staged drop count is folded in before the batch it belongs to.
 */
static void test_the_drain_keeps_its_guards(void)
{
	span_t d;

	load_source("zephyr/console/mp_glue.c");
	d = fn_body("static void mp_event_drain_locked(void)");

	guard_must_test(&d, "sts_mp_event_purge();", "mp_stream_is_sub");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U,
		count_in(&d, "mp_stream_event_count(&mp.st) >= (size_t)MP_EVQ_LEN"),
		"the drain no longer checks the engine has room");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&d, "mp_stream_event_count(&mp.st) >= (size_t)MP_EVQ_LEN") <
			offset_in(&d, "mp_post_event(&mp,"),
		"the drain now posts before checking for room, which counts a "
		"drop for an event it goes on to keep");

	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&d, "mp_stream_event_drop_note(&mp.st") <
			offset_in(&d, "mp_post_event(&mp,"),
		"the staged drop count is no longer folded in before its batch");

	/* Peek/post/pop, not pop/post: the pop must follow the successful
	 * post or a full engine queue loses the event between the two. */
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&d, "sts_mp_event_peek(&ev)") <
			offset_in(&d, "sts_mp_event_pop();"),
		"the drain no longer peeks before it pops");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&d, "mp_post_event(&mp,") <
			offset_in(&d, "sts_mp_event_pop();"),
		"the drain now pops before the engine has accepted the event");
}

/**
 * The event channel is in the lock-free armed mask.
 *
 * Scoped to mp_armed_refresh(): MP_CH_EVENT appears elsewhere in the file (the
 * drain, `mp status`), and a file-wide substring check would keep passing with
 * the channel removed from the array — at which case the producers' arming test
 * reads false forever and NOTHING is ever staged. That is the original defect
 * restored in full, with every behavioural test in this suite still green.
 */
static void test_the_event_channel_is_armed(void)
{
	span_t a;

	load_source("zephyr/console/mp_glue.c");
	a = fn_body("static void mp_armed_refresh(void)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&a, "(uint8_t)MP_CH_EVENT,"),
		"MP_CH_EVENT is no longer in the armed-channel array");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&a, "mp_mode(&mp) == (uint8_t)MP_MODE_MP"),
		"the armed mask is no longer gated on MP mode");
}

/** The producer is bound before anything can be armed. */
static void test_the_staging_queue_is_bound_at_start(void)
{
	span_t s;

	load_source("zephyr/console/mp_glue.c");
	s = fn_body("int sts_mp_start(void)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&s, "sts_mp_event_init();"),
		"sts_mp_start() no longer binds the event staging queue");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&s, "sts_mp_event_init();") <
			offset_in(&s, "mp_armed_refresh();"),
		"the staging queue is bound after the mask that arms its "
		"producers");
}

/** The scanner itself must be able to fail. A blanked comment must not
 *  satisfy a claim, and a missing subject must not read as agreement. */
static void test_the_scanner_ignores_prose(void)
{
	span_t f;

	load_source("zephyr/sts_app.c");
	f = whole_file();

	/* sts_app.c's own comment block above the post names MP_EV_ALARM and
	 * sts_alarm_set() in prose; after blanking, only the code counts. */
	TEST_ASSERT_EQUAL_UINT(1U,
			       count_in(&f, "sts_mp_post_event((uint8_t)MP_EV_ALARM"));
	TEST_ASSERT_EQUAL_UINT(0U, count_in(&f, "MP_EV_ALARM on the Field"));
}

/* ------------------------------------------------------------------- runner */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_pg_drop_reaches_the_wire);
	RUN_TEST(test_pg_recovery_reaches_the_wire);
	RUN_TEST(test_button_edges_reach_the_wire);
	RUN_TEST(test_long_press_and_repeat_are_distinguishable);
	RUN_TEST(test_reed_switch_reaches_the_wire);
	RUN_TEST(test_touch_int_reaches_the_wire_on_assert);
	RUN_TEST(test_ina_alert_reaches_the_wire);
	RUN_TEST(test_alarm_transitions_reach_the_wire_once);
	RUN_TEST(test_scanned_signals_are_not_alarm_events);
	RUN_TEST(test_timestamp_is_the_edge_not_the_drain);

	RUN_TEST(test_overflow_drops_newest_and_is_counted);
	RUN_TEST(test_drop_count_is_not_re_reported);
	RUN_TEST(test_engine_backpressure_keeps_events_staged);
	RUN_TEST(test_unsubscribed_backlog_is_purged_not_counted);
	RUN_TEST(test_bad_kind_is_refused_at_the_producer);
	RUN_TEST(test_text_is_truncated_not_overrun);

	RUN_TEST(test_alarm_set_posts_only_on_a_transition);
	RUN_TEST(test_io_scan_posts_what_the_plan_decided);
	RUN_TEST(test_the_tick_drains_before_it_ticks_and_waits_once);
	RUN_TEST(test_the_drain_keeps_its_guards);
	RUN_TEST(test_the_event_channel_is_armed);
	RUN_TEST(test_the_staging_queue_is_bound_at_start);
	RUN_TEST(test_the_scanner_ignores_prose);

	return UNITY_END();
}
