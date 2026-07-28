/*
 * STS1000 "Meridian" — the console link sampler, driven into the real engine.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * WHAT THIS SUITE EXISTS TO CATCH, stated so it cannot be diluted later.
 *
 * `test_mp_rpc.c:test_link_loss_leaves_mp_mode` already proved the *seam*: call
 * mp_set_link(c, false) and the mode drops and the leases revert. It passed for
 * as long as the defect existed, because the defect was never in the seam — it
 * was that **nothing decided to call it**. sts_usb.c polled CDC-ACM #0's DTR for
 * sts_usb_configured() and tracked VBUS for the attach gate, and told MP about
 * neither. A technician who pulled the cable left the board with `mp_bypass`
 * installed on the shell UART and the overrides asserted; the port came back
 * with the Zephyr shell unreachable.
 *
 * So this suite starts one step earlier. Every link notification below is
 * emitted **only when sts_console_link_step() says a transition happened**, from
 * a sequence of (VBUS, DTR) samples of the shape sts_usb_poll() actually sees.
 * The seam is then driven with the real mp_ctx_t — a live admin session holding
 * a real override lease — so "the caller decided to notify" and "the notify
 * reverted the lease" are one chain and not two facts that happen to be true
 * separately.
 *
 * The mirror of sts_usb_poll() is in one place, drive_pass(), and it is
 * deliberately the *only* route from a sample to the engine in this file: a test
 * that reached mp_set_link() directly would be back to testing the seam.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "mp/mp.h"
#include "zephyr/console/sts_console_link_policy.h"

#define SERIAL "STS1000-000042"
#define ADMIN_USER "root"
#define ADMIN_PW "correct horse"

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

/* Releases seen by the apply callback — the board actually being stood down. */
static unsigned int g_releases;

static mp_ilk_state_t g_ilk;

/* The sampler under test, and the engine's view of the link it drives. */
static sts_link_state_t g_link;

static uint32_t clock_cb(void *user)
{
	(void)user;
	return g_now;
}

static int tx_cb(void *user, const uint8_t *wire, size_t len)
{
	(void)user;
	(void)wire;
	(void)len;
	return 0;
}

static int apply_cb(void *user, size_t obj, const int32_t *value)
{
	(void)user;
	(void)obj;
	if (value == NULL) {
		g_releases++;
	}
	return 0;
}

static int obj_read_cb(void *user, size_t obj, mp_val_t *out)
{
	const mp_obj_t *o = mp_obj_at(obj);

	(void)user;
	TEST_ASSERT_NOT_NULL(o);
	memset(out, 0, sizeof(*out));
	out->kind = o->kind;
	out->valid = true;
	out->i = o->min;
	return 0;
}

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
	g_releases = 0U;

	memset(&g_ilk, 0, sizeof(g_ilk));
	g_ilk.ocxo_warm = true;
	g_ilk.supercaps_ok = true;
	g_ilk.vcc_rb_mv = 15000;
	g_ilk.vcc_rb_valid = true;
	g_ilk.rb_expected_mv = 15000;
	g_ilk.rb_vmax_mv = 15000U;
	g_ilk.rb_code_max = 200U;
	g_ilk.liveness_ok = true;
	g_ilk.disc_parked = true;
	g_ilk.extref_ok = true;
	g_ilk.rb_lock = true;
	g_ilk.disp_on = true;

	for (i = 0U; i < 2U; i++) {
		g_slots[i].buf = g_slot_buf[i];
		g_slots[i].cap = sizeof(g_slot_buf[i]);
	}

	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, NULL));
	TEST_ASSERT_EQUAL_INT(0, logr_init(&g_log, g_log_slots, 16U));

	memset(&w, 0, sizeof(w));
	w.tx = tx_cb;
	w.mono_ms = clock_cb;
	w.model = "STS1000";
	w.serial = SERIAL;
	w.fw_version = "1.2.3";
	w.boot_version = "0.9.0";
	w.board_id = "0011223344556677";
	w.ilk = ilk_cb;
	w.apply = apply_cb;
	w.obj_read = obj_read_cb;
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

	/*
	 * The sampler starts where sts_usb_start() leaves it: VBUS present (the
	 * technician is plugged in), DTR not yet believed. The very first pass
	 * therefore publishes the link as an edge, which is how the engine's
	 * link_up comes to be true at all in this file — nothing here sets it
	 * directly.
	 */
	sts_console_link_seed(&g_link, true);
}

/* ------------------------------------------------------- the caller's mirror */

/**
 * One pass of sts_usb_poll(), with the same order of operations.
 *
 * The engine call is inside the `switch` on purpose. sts_usb.c reaches
 * sts_mp_notify_link(), whose uncontended path is mp_link_apply_locked() ->
 * mp_set_link(); the mutex and the deferral it wraps that in are Zephyr and are
 * not modelled here — what is modelled is that **no notification is emitted
 * unless the policy produced a transition**, which is the half that was missing
 * from the image.
 *
 * @return the verdict, so a test can assert on the decision as well as its effect.
 */
static sts_link_act_t drive_pass(bool vbus, bool dtr)
{
	sts_link_act_t act = sts_console_link_step(&g_link, vbus, dtr);

	switch (act.mp) {
	case STS_LINK_MP_UP:
		TEST_ASSERT_EQUAL_INT(0, mp_set_link(&g_c, true));
		break;
	case STS_LINK_MP_DOWN:
		TEST_ASSERT_EQUAL_INT(0, mp_set_link(&g_c, false));
		break;
	case STS_LINK_MP_NONE:
	default:
		break;
	}
	return act;
}

/** Open MP mode with an admin session holding one real override lease. */
static void enter_with_a_lease(void)
{
	const char *out = NULL;
	size_t len = 0U;
	char req[256];
	uint32_t sid;
	mp_json_t rp;
	static mp_json_tok_t tok[512];
	int64_t v = 0;

	TEST_ASSERT_EQUAL_INT(0, mp_mode_enter(&g_c));

	TEST_ASSERT_EQUAL_INT(
		0, mp_rpc_handle(&g_c,
				 (const uint8_t *)"{\"jsonrpc\":\"2.0\",\"id\":1,"
				 "\"method\":\"session.open\",\"params\":"
				 "{\"client\":\"fmt\",\"user\":\"" ADMIN_USER
				 "\",\"secret\":\"" ADMIN_PW "\"}}",
				 strlen("{\"jsonrpc\":\"2.0\",\"id\":1,"
					"\"method\":\"session.open\",\"params\":"
					"{\"client\":\"fmt\",\"user\":\"" ADMIN_USER
					"\",\"secret\":\"" ADMIN_PW "\"}}"),
				 &out, &len));
	TEST_ASSERT_TRUE(mp_json_parse(&rp, out, len, tok, 512U, 0U) > 0);
	TEST_ASSERT_EQUAL_INT(
		0, mp_json_i64(&rp,
			       mp_json_obj_get(&rp, mp_json_obj_get(&rp, 0,
								    "result"),
					       "sid"),
			       &v));
	sid = (uint32_t)v;
	TEST_ASSERT_TRUE(sid != 0U);

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"ui.disp.bl\",\"value\":40,"
		       "\"sid\":%u}}",
		       sid);
	TEST_ASSERT_EQUAL_INT(0, mp_rpc_handle(&g_c, (const uint8_t *)req,
					       strlen(req), &out, &len));
	TEST_ASSERT_TRUE(mp_json_parse(&rp, out, len, tok, 512U, 0U) > 0);
	TEST_ASSERT_TRUE_MESSAGE(mp_json_obj_get(&rp, 0, "result") >= 0,
				 "obj.override was refused");

	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_MODE_MP, mp_mode(&g_c));
	TEST_ASSERT_EQUAL_size_t(1U, mp_ovr_active(&g_c.ovr));
	g_releases = 0U;
}

/** The state a technician is in: plugged in, port open, override held. */
static void a_live_session(void)
{
	sts_link_act_t act = drive_pass(true, true);

	TEST_ASSERT_EQUAL_INT(STS_LINK_MP_UP, act.mp);
	TEST_ASSERT_TRUE(g_c.ovr.link_up);
	enter_with_a_lease();
}

/* ===================================================================== *
 *  the escape: what the defect report said did not work
 * ===================================================================== */

/**
 * THE REGRESSION TEST. Closing the terminal drops DTR; that is a link drop; the
 * lease reverts and the console comes back.
 *
 * Every assertion below failed to be *reachable* before this change, because
 * nothing turned the second half of a DTR sample into a notification.
 */
static void test_closing_the_port_reverts_the_override_and_leaves_mp_mode(void)
{
	sts_link_act_t act;

	a_live_session();

	/* The host closes the port. VBUS stays: the cable is still in. */
	act = drive_pass(true, false);

	TEST_ASSERT_EQUAL_INT_MESSAGE(STS_LINK_MP_DOWN, act.mp,
				      "a DTR drop is a link drop");
	TEST_ASSERT_FALSE(g_c.ovr.link_up);
	TEST_ASSERT_EQUAL_UINT8_MESSAGE((uint8_t)MP_MODE_SHELL, mp_mode(&g_c),
					"the shell must own the port again");
	TEST_ASSERT_EQUAL_size_t_MESSAGE(0U, mp_ovr_active(&g_c.ovr),
					 "the override must not outlive its link");
	/* Not merely forgotten: the object was actually released. */
	TEST_ASSERT_EQUAL_UINT(1U, g_releases);
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.ovr.sess.id);
	/* §5.3 accounting: a link loss is a dead-man revert, not an expiry. */
	TEST_ASSERT_EQUAL_UINT32(1U, g_c.ovr.deadman_reverts);
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.ovr.expiries);
}

/**
 * THE CABLE PULL, which is the case the defect report opened with — and the one
 * a DTR-only sampler gets wrong.
 *
 * CDC-ACM's DTR is `line_state`, a host-written byte the class driver clears
 * only on a controller-reported USB_DC_DISCONNECTED. A VBUS-gated board calls
 * usb_disable() itself, so that callback may never run and the last DTR sample
 * can still read asserted across the pull. The sampler must not consult it.
 */
static void test_a_cable_pull_is_a_link_drop_even_with_dtr_still_asserted(void)
{
	sts_link_act_t act;

	a_live_session();

	/* VBUS gone, DTR still reading 1 — the stale line_state case. */
	act = drive_pass(false, true);

	TEST_ASSERT_EQUAL_INT_MESSAGE(STS_LINK_MP_DOWN, act.mp,
				      "no VBUS means no port, whatever DTR says");
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_MODE_SHELL, mp_mode(&g_c));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c.ovr));
	TEST_ASSERT_EQUAL_UINT(1U, g_releases);

	/* And the USB gate still does its own job in the same pass. */
	TEST_ASSERT_TRUE(act.detach);
	TEST_ASSERT_TRUE(act.mcp_down);
	TEST_ASSERT_FALSE(act.attach);
}

/**
 * The revert happens BEFORE the stack is torn down.
 *
 * sts_usb_poll() runs the MP notification ahead of usb_detach(); the verdict is
 * a struct, so the only thing pinning that order is the caller. Here it is
 * pinned as a property of the pass rather than of the field order: by the time
 * anything could act on `detach`, the engine has already released the object.
 */
static void test_the_revert_precedes_the_detach(void)
{
	sts_link_act_t act;

	a_live_session();
	act = drive_pass(false, false);

	TEST_ASSERT_TRUE(act.detach);
	TEST_ASSERT_EQUAL_UINT_MESSAGE(1U, g_releases,
				       "the board must be stood down while the "
				       "diagnostic surface still exists");
}

/* ===================================================================== *
 *  the sampler's own rules
 * ===================================================================== */

/** Edge, not level: a held-open port must not re-notify on every 250 ms pass. */
static void test_a_steady_link_produces_no_further_notifications(void)
{
	unsigned int i;

	a_live_session();

	for (i = 0U; i < 20U; i++) {
		TEST_ASSERT_EQUAL_INT(STS_LINK_MP_NONE, drive_pass(true, true).mp);
	}
	/* Nothing moved: still in MP mode, still holding the lease. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_MODE_MP, mp_mode(&g_c));
	TEST_ASSERT_EQUAL_size_t(1U, mp_ovr_active(&g_c.ovr));
	TEST_ASSERT_EQUAL_UINT(0U, g_releases);
}

/** And a steady *absent* port is equally quiet, from the seeded state. */
static void test_a_steady_absence_produces_no_notifications(void)
{
	unsigned int i;

	sts_console_link_seed(&g_link, false);
	for (i = 0U; i < 20U; i++) {
		TEST_ASSERT_EQUAL_INT(STS_LINK_MP_NONE, drive_pass(false, true).mp);
	}
	TEST_ASSERT_FALSE(g_c.ovr.link_up);
}

/**
 * A stale DTR must not raise the link while VBUS is absent.
 *
 * The inverse of the cable-pull case and the one that would quietly re-satisfy
 * the dead-man's outermost condition on a board with nothing plugged in.
 */
static void test_dtr_alone_never_raises_the_link(void)
{
	unsigned int i;

	sts_console_link_seed(&g_link, false);
	for (i = 0U; i < 5U; i++) {
		TEST_ASSERT_EQUAL_INT(STS_LINK_MP_NONE, drive_pass(false, true).mp);
	}
	TEST_ASSERT_FALSE(g_c.ovr.link_up);

	/* Plugging in with the host's DTR already asserted is one edge, up. */
	TEST_ASSERT_EQUAL_INT(STS_LINK_MP_UP, drive_pass(true, true).mp);
	TEST_ASSERT_TRUE(g_c.ovr.link_up);
}

/** Reconnecting is a fresh link, and the reverted lease does not come back. */
static void test_the_link_comes_back_up_and_the_lease_does_not(void)
{
	a_live_session();
	TEST_ASSERT_EQUAL_INT(STS_LINK_MP_DOWN, drive_pass(false, false).mp);
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c.ovr));

	TEST_ASSERT_EQUAL_INT(STS_LINK_MP_UP, drive_pass(true, true).mp);
	TEST_ASSERT_TRUE(g_c.ovr.link_up);
	/* Link up is not session up: MP mode and the lease are gone for good. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_MODE_SHELL, mp_mode(&g_c));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c.ovr));
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.ovr.sess.id);
}

/**
 * The attach gate is unchanged by any of this.
 *
 * sts_usb_poll()'s VBUS edges were already correct and are now produced by the
 * same function that decides the link; a regression here would take USB away
 * from a board that has no other management port when the network is down.
 */
static void test_the_vbus_gate_still_attaches_and_detaches_on_edges(void)
{
	sts_link_act_t act;

	sts_console_link_seed(&g_link, false);

	act = drive_pass(false, false);
	TEST_ASSERT_FALSE(act.attach);
	TEST_ASSERT_FALSE(act.detach);

	act = drive_pass(true, false);
	TEST_ASSERT_TRUE(act.attach);
	TEST_ASSERT_FALSE(act.detach);
	TEST_ASSERT_FALSE(act.mcp_down);

	/* Level, not edge, would attach again. */
	act = drive_pass(true, false);
	TEST_ASSERT_FALSE(act.attach);

	act = drive_pass(false, false);
	TEST_ASSERT_TRUE(act.detach);
	TEST_ASSERT_TRUE(act.mcp_down);
}

/** The seed is what stops sts_usb_start()'s attach being repeated by pass one. */
static void test_the_seed_absorbs_the_boot_time_edge(void)
{
	sts_console_link_seed(&g_link, true);
	TEST_ASSERT_FALSE_MESSAGE(drive_pass(true, false).attach,
				  "sts_usb_start() already attached");
	/* But the link is still unseeded, so DTR arrives as a real edge. */
	TEST_ASSERT_EQUAL_INT(STS_LINK_MP_UP, drive_pass(true, true).mp);
}

/**
 * No sequence of samples leaves an override held without a port.
 *
 * The conjunction FMT §5.4 states, checked exhaustively over every reachable
 * two-sample path rather than over the three the tests above happen to walk.
 */
static void test_no_sample_sequence_holds_an_override_without_a_port(void)
{
	unsigned int a;
	unsigned int b;

	for (a = 0U; a < 4U; a++) {
		for (b = 0U; b < 4U; b++) {
			bool va = (a & 2U) != 0U;
			bool da = (a & 1U) != 0U;
			bool vb = (b & 2U) != 0U;
			bool db = (b & 1U) != 0U;

			setUp();
			a_live_session();

			(void)drive_pass(va, da);
			(void)drive_pass(vb, db);

			if (vb && db) {
				continue; /* port present: holding is correct */
			}
			TEST_ASSERT_EQUAL_size_t_MESSAGE(
				0U, mp_ovr_active(&g_c.ovr),
				"an override outlived the port it was held over");
			TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_MODE_SHELL,
						mp_mode(&g_c));
		}
	}
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_closing_the_port_reverts_the_override_and_leaves_mp_mode);
	RUN_TEST(test_a_cable_pull_is_a_link_drop_even_with_dtr_still_asserted);
	RUN_TEST(test_the_revert_precedes_the_detach);

	RUN_TEST(test_a_steady_link_produces_no_further_notifications);
	RUN_TEST(test_a_steady_absence_produces_no_notifications);
	RUN_TEST(test_dtr_alone_never_raises_the_link);
	RUN_TEST(test_the_link_comes_back_up_and_the_lease_does_not);

	RUN_TEST(test_the_vbus_gate_still_attaches_and_detaches_on_edges);
	RUN_TEST(test_the_seed_absorbs_the_boot_time_edge);
	RUN_TEST(test_no_sample_sequence_holds_an_override_without_a_port);

	return UNITY_END();
}
