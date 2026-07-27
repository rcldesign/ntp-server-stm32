/*
 * STS1000 "Meridian" — arming IEEE 1588-2019 Annex P integrity.
 *
 * sts_ptp_icv_policy.h owns the decision the glue was missing: core/ptp had a
 * complete Annex-P implementation and ptp_port.c called into it on both paths,
 * but nothing ever created a ptp_icv_ctx_t, so ptp_port_ctx_t::icv was
 * permanently NULL and the feature could not be switched on at all. That is the
 * shape of every defect this planner can carry — invisible, and in the direction
 * of "authenticated" being a word rather than a property.
 *
 * Four invariants, and the suite is organised around them:
 *
 *   1. OFF IS THE SHIPPED DEFAULT. `ptp.icv.policy` defaults to 0 and 0 detaches.
 *      A grandmaster that starts signing to peers that cannot verify is worse
 *      than one that does not, so the feature is opt-in even though core's own
 *      ptp_icv_cfg_defaults() starts at VERIFY_IF_PRESENT — core's default is
 *      where a caller who has already chosen the feature begins, which is a
 *      different question and is asserted here as a difference on purpose.
 *
 *   2. ARMING NEEDS A KEY, AND A MISSING KEY DOES NOT DETACH. With
 *      `ptp.icv.key` empty the context is still attached and the engine is
 *      inert: TX emits nothing, RX accepts unsigned under VERIFY_IF_PRESENT and
 *      rejects everything under REQUIRE. Detaching would be quieter and would
 *      silently downgrade a REQUIRE deployment to unauthenticated operation. A
 *      dead PTP port is loud; a forged Announce is not.
 *
 *   3. A REFUSAL DISARMS. The produced cfg is the *disarmed* one on every
 *      non-arming path, so pushing it into a live context zeroizes rather than
 *      leaving the previous key in service. The header exposes
 *      sts_ptp_icv_cfg_is_keyless() precisely so this is asserted directly
 *      rather than inferred from the state enum — and it is the same lesson
 *      net/sts_ntp_keys.h records, because a rejected rotation that leaves the
 *      old key authenticating is worse than one that leaves the box
 *      unauthenticated: the rotation was probably prompted by that key's
 *      compromise.
 *
 *   4. THE RULES ARE CORE'S. The planner defers to ptp_icv_key_set() and
 *      ptp_icv_cfg_validate() instead of re-deriving what a usable association
 *      looks like, so the suite drives the real core/ptp rather than a model:
 *      a glue that installs something core would silently reinterpret is
 *      exactly the disagreement the indirection exists to prevent.
 *
 * The sweep at the end states all four over the whole input domain, so a new
 * branch cannot widen the arming conjunction without failing here.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "ptp/ptp_icv.h"
#include "zephyr/net/sts_ptp_icv_policy.h"

/* ------------------------------------------------------------------ fixture */

/* File-scope, so setUp() has to clear them: a plan left over from the previous
 * test would make every assertion after it read somebody else's answer. */
static sts_ptp_icv_want_t g_w;
static ptp_icv_cfg_t g_cfg;
static sts_ptp_icv_plan_t g_plan;

/** A configuration that would arm: policy, suite, key, everything in range. */
static void want_armable(void)
{
	(void)memset(&g_w, 0, sizeof(g_w));
	g_w.policy = (uint8_t)PTP_ICV_POLICY_VERIFY_IF_PRESENT;
	g_w.suite = (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128;
	g_w.spp = 7U;
	g_w.key_id = 0x1234ABCDUL;
	g_w.key_len = 16U;
	(void)memset(g_w.key, 0xA5, g_w.key_len);
	g_w.replay_protect = true;
	g_w.replay_window = 16U;
	g_w.mask_mutable = true;
}

void setUp(void)
{
	(void)memset(&g_w, 0, sizeof(g_w));
	(void)memset(&g_cfg, 0, sizeof(g_cfg));
	(void)memset(&g_plan, 0, sizeof(g_plan));
}

void tearDown(void)
{
}

static void plan(void)
{
	sts_ptp_icv_plan(&g_w, &g_cfg, &g_plan);
}

/** The association the plan installed for (spp, key_id), or NULL. */
static const ptp_icv_key_t *assoc(uint8_t spp, uint32_t key_id)
{
	size_t i;

	for (i = 0U; i < (size_t)PTP_ICV_MAX_KEYS; i++) {
		if (g_cfg.key[i].in_use && (g_cfg.key[i].spp == spp) &&
		    (g_cfg.key[i].key_id == key_id)) {
			return &g_cfg.key[i];
		}
	}
	return NULL;
}

/** Every disarming path owes this: policy OFF, no attach, no key material. */
static void assert_disarmed(const char *why)
{
	TEST_ASSERT_FALSE_MESSAGE(g_plan.attach, why);
	TEST_ASSERT_EQUAL_UINT8_MESSAGE((uint8_t)PTP_ICV_POLICY_OFF,
					g_cfg.policy, why);
	TEST_ASSERT_TRUE_MESSAGE(sts_ptp_icv_cfg_is_keyless(&g_cfg), why);
}

/* ===================================================================== */
/* 1. OFF is the shipped default                                          */
/* ===================================================================== */

/**
 * `ptp.icv.policy` = 0 is what a unit ships with, and 0 means detached.
 *
 * The cfg schema's own default for the key is 0, so this is also the state of
 * every box that has never been configured: no context, and the 1588 engine
 * behaves exactly as it did before Annex P existed.
 */
static void test_policy_off_is_the_default_and_detaches(void)
{
	g_w.policy = (uint8_t)PTP_ICV_POLICY_OFF;
	plan();

	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_OFF, g_plan.state);
	TEST_ASSERT_EQUAL_INT(0, g_plan.rc); /* OFF is not a refusal */
	assert_disarmed("policy OFF");
}

/**
 * OFF wins over a provisioned key.
 *
 * This is the disarm path an operator uses to switch the feature off without
 * deleting the key from the config tree, and it is also what a factory reset
 * produces once the applier re-runs: the key must not reach the produced cfg,
 * or ptp_icv_set_cfg() would leave the association live in RAM.
 */
static void test_policy_off_drops_a_provisioned_key(void)
{
	want_armable();
	g_w.policy = (uint8_t)PTP_ICV_POLICY_OFF;
	plan();

	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_OFF, g_plan.state);
	assert_disarmed("policy OFF with a key provisioned");
	TEST_ASSERT_NULL(assoc(g_w.spp, g_w.key_id));
}

/**
 * The planner's OFF is deliberately *not* core's default.
 *
 * ptp_icv_cfg_defaults() starts at VERIFY_IF_PRESENT because it answers "where
 * should a caller who wants this feature begin". The shipped policy answers a
 * different question, and a planner that simply forwarded core's default would
 * quietly attach a context on every unit.
 */
static void test_the_shipped_default_differs_from_cores_default(void)
{
	ptp_icv_cfg_t core;

	(void)memset(&core, 0, sizeof(core));
	ptp_icv_cfg_defaults(&core);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)PTP_ICV_POLICY_VERIFY_IF_PRESENT,
				core.policy);

	g_w.policy = (uint8_t)PTP_ICV_POLICY_OFF;
	plan();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)PTP_ICV_POLICY_OFF, g_cfg.policy);
}

/* ===================================================================== */
/* 2. attached but unkeyed                                                */
/* ===================================================================== */

/**
 * A policy without a key attaches and stays inert.
 *
 * ptp_icv_cfg_validate() permits an empty table on purpose — it is legal and
 * inert — so this is a plan, not a refusal, and the distinction the header draws
 * between *attach* and *arm* is exactly this state.
 */
static void test_verify_if_present_without_a_key_attaches_inert(void)
{
	want_armable();
	g_w.key_len = 0U;
	plan();

	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_NO_KEY, g_plan.state);
	TEST_ASSERT_TRUE(g_plan.attach);
	TEST_ASSERT_EQUAL_INT(0, g_plan.rc);
	/* The policy is loaded — the engine is attached, just unkeyed. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)PTP_ICV_POLICY_VERIFY_IF_PRESENT,
				g_cfg.policy);
	TEST_ASSERT_TRUE(sts_ptp_icv_cfg_is_keyless(&g_cfg));
}

/**
 * REQUIRE without a key attaches too, and that is the deliberate choice.
 *
 * Every message is then rejected and PTP stops working. Detaching would be
 * quieter and would silently downgrade a REQUIRE deployment to unauthenticated
 * operation — an unauthenticated grandmaster that an operator believes is
 * authenticated is the precise failure this feature exists to prevent. So the
 * loud outcome is the correct one, and this test is what stops a future "fix"
 * from making it quiet.
 */
static void test_require_without_a_key_attaches_rather_than_downgrading(void)
{
	want_armable();
	g_w.policy = (uint8_t)PTP_ICV_POLICY_REQUIRE;
	g_w.key_len = 0U;
	plan();

	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_NO_KEY, g_plan.state);
	TEST_ASSERT_TRUE_MESSAGE(g_plan.attach,
				 "REQUIRE with no key must NOT detach: that "
				 "silently serves unauthenticated PTP to an "
				 "operator who asked for authenticated PTP");
	TEST_ASSERT_EQUAL_UINT8((uint8_t)PTP_ICV_POLICY_REQUIRE, g_cfg.policy);
	TEST_ASSERT_TRUE(sts_ptp_icv_cfg_is_keyless(&g_cfg));
}

/* ===================================================================== */
/* 3. armed                                                               */
/* ===================================================================== */

static void test_a_policy_and_a_key_arms(void)
{
	want_armable();
	plan();

	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_ARMED, g_plan.state);
	TEST_ASSERT_TRUE(g_plan.attach);
	TEST_ASSERT_EQUAL_INT(0, g_plan.rc);
	TEST_ASSERT_FALSE(sts_ptp_icv_cfg_is_keyless(&g_cfg));

	/* The transmit association names the key that was installed, or TX
	 * signs with nothing and core answers -ENOENT at validate time. */
	TEST_ASSERT_EQUAL_UINT8(g_w.spp, g_cfg.tx_spp);
	TEST_ASSERT_EQUAL_UINT32(g_w.key_id, g_cfg.tx_key_id);
	TEST_ASSERT_NOT_NULL(assoc(g_w.spp, g_w.key_id));
	TEST_ASSERT_EQUAL_UINT8(g_w.suite, assoc(g_w.spp, g_w.key_id)->suite);
	TEST_ASSERT_EQUAL_UINT8(g_w.key_len,
				assoc(g_w.spp, g_w.key_id)->key_len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_w.key, assoc(g_w.spp, g_w.key_id)->key,
				     g_w.key_len);
}

static void test_both_policies_and_both_suites_arm(void)
{
	uint8_t pol;
	uint8_t suite;

	for (pol = (uint8_t)PTP_ICV_POLICY_VERIFY_IF_PRESENT;
	     pol < (uint8_t)PTP_ICV_POLICY_COUNT; pol++) {
		for (suite = 0U; suite < (uint8_t)PTP_ICV_SUITE_COUNT;
		     suite++) {
			char msg[64];

			want_armable();
			g_w.policy = pol;
			g_w.suite = suite;
			plan();

			(void)snprintf(msg, sizeof(msg),
				       "policy %u suite %u -> %s",
				       (unsigned int)pol, (unsigned int)suite,
				       sts_ptp_icv_state_name(g_plan.state));
			TEST_ASSERT_EQUAL_UINT8_MESSAGE(
				(uint8_t)STS_PTP_ICV_ARMED, g_plan.state, msg);
			TEST_ASSERT_EQUAL_UINT8_MESSAGE(pol, g_cfg.policy, msg);
			TEST_ASSERT_EQUAL_UINT8_MESSAGE(
				suite, assoc(g_w.spp, g_w.key_id)->suite, msg);
		}
	}
}

/** The key-length domain is core's: 1..PTP_ICV_KEY_MAX, and 65 is not a key. */
static void test_the_key_length_boundary_is_cores(void)
{
	want_armable();
	g_w.key_len = 1U;
	plan();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_ARMED, g_plan.state);

	want_armable();
	g_w.key_len = (size_t)PTP_ICV_KEY_MAX;
	(void)memset(g_w.key, 0x5A, g_w.key_len);
	plan();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_ARMED, g_plan.state);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)PTP_ICV_KEY_MAX,
				assoc(g_w.spp, g_w.key_id)->key_len);

	/* One over. ptp_icv_key_set() rejects the length before it copies, so
	 * the planner never has a truncated association to install. */
	want_armable();
	g_w.key_len = (size_t)PTP_ICV_KEY_MAX + 1U;
	plan();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_REFUSED, g_plan.state);
	TEST_ASSERT_EQUAL_INT(-EINVAL, g_plan.rc);
	assert_disarmed("an over-long key");
}

/* ===================================================================== */
/* 4. refusals, and that every one of them disarms                        */
/* ===================================================================== */

static void test_a_null_request_is_a_refusal_not_an_off(void)
{
	ptp_icv_cfg_t cfg;
	sts_ptp_icv_plan_t out;

	(void)memset(&cfg, 0xEE, sizeof(cfg));
	(void)memset(&out, 0xEE, sizeof(out));
	sts_ptp_icv_plan(NULL, &cfg, &out);

	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_REFUSED, out.state);
	TEST_ASSERT_EQUAL_INT(-EINVAL, out.rc);
	TEST_ASSERT_FALSE(out.attach);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)PTP_ICV_POLICY_OFF, cfg.policy);
	TEST_ASSERT_TRUE_MESSAGE(sts_ptp_icv_cfg_is_keyless(&cfg),
				 "a refusal must still overwrite the cfg it "
				 "was handed, or it is not a zeroization step");
}

static void test_an_unknown_policy_is_refused(void)
{
	unsigned int pol;

	for (pol = (unsigned int)PTP_ICV_POLICY_COUNT; pol <= 0xFFU; pol++) {
		char msg[64];

		want_armable();
		g_w.policy = (uint8_t)pol;
		plan();

		(void)snprintf(msg, sizeof(msg), "policy %u -> %s", pol,
			       sts_ptp_icv_state_name(g_plan.state));
		TEST_ASSERT_EQUAL_UINT8_MESSAGE((uint8_t)STS_PTP_ICV_REFUSED,
						g_plan.state, msg);
		TEST_ASSERT_EQUAL_INT_MESSAGE(-EINVAL, g_plan.rc, msg);
		assert_disarmed(msg);
	}
}

static void test_an_unknown_suite_is_refused(void)
{
	want_armable();
	g_w.suite = (uint8_t)PTP_ICV_SUITE_COUNT;
	plan();

	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_REFUSED, g_plan.state);
	TEST_ASSERT_EQUAL_INT(-EINVAL, g_plan.rc);
	assert_disarmed("an unknown suite");
}

/**
 * A window outside core's range is refused rather than clamped.
 *
 * `ptp.icv.window` is bounded 1..32 by the schema, so this arrives only from a
 * corrupted store or a future caller that skips cfg — which is exactly when the
 * planner must not invent a value. Both the keyed and the unkeyed path go
 * through ptp_icv_cfg_validate(), so both refuse.
 */
static void test_a_window_out_of_range_is_refused_on_both_paths(void)
{
	want_armable();
	g_w.replay_window = (uint8_t)PTP_ICV_REPLAY_WINDOW_MAX + 1U;
	plan();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_REFUSED, g_plan.state);
	TEST_ASSERT_EQUAL_INT(-EINVAL, g_plan.rc);
	assert_disarmed("an over-wide replay window, keyed");

	want_armable();
	g_w.key_len = 0U;
	g_w.replay_window = (uint8_t)PTP_ICV_REPLAY_WINDOW_MAX + 1U;
	plan();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_REFUSED, g_plan.state);
	TEST_ASSERT_EQUAL_INT(-EINVAL, g_plan.rc);
	assert_disarmed("an over-wide replay window, unkeyed");
}

/**
 * A refusal wipes a cfg that already held a key.
 *
 * The rotation case, and the reason the planner writes @p cfg on every path:
 * the glue pushes the produced cfg into the live context, so a refusal that
 * left the buffer alone would leave the *previous* key authenticating — after
 * the operator has been told the new one was rejected, and usually because the
 * old one was compromised.
 */
static void test_a_refusal_overwrites_a_previously_keyed_cfg(void)
{
	static const uint8_t old_key[16] = {
		0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
		0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
	};

	/* Arm for real first, so the buffer genuinely holds key material. */
	want_armable();
	(void)memcpy(g_w.key, old_key, sizeof(old_key));
	g_w.key_len = sizeof(old_key);
	plan();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_ARMED, g_plan.state);
	TEST_ASSERT_FALSE(sts_ptp_icv_cfg_is_keyless(&g_cfg));

	/* Now hand the same buffer a rotation core will not accept. */
	want_armable();
	g_w.suite = (uint8_t)PTP_ICV_SUITE_COUNT;
	plan();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_REFUSED, g_plan.state);
	assert_disarmed("a rejected rotation over a live key");

	/* Byte-level: the old key is not merely unreferenced, it is gone. */
	{
		size_t i;
		size_t j;

		for (i = 0U; i < (size_t)PTP_ICV_MAX_KEYS; i++) {
			for (j = 0U; j + sizeof(old_key) <=
				     (size_t)PTP_ICV_KEY_MAX; j++) {
				TEST_ASSERT_NOT_EQUAL_INT(
					0, memcmp(&g_cfg.key[i].key[j], old_key,
						  sizeof(old_key)));
			}
		}
	}
}

/** And so does an OFF, which is the same path a factory reset takes. */
static void test_switching_off_overwrites_a_previously_keyed_cfg(void)
{
	want_armable();
	plan();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_ARMED, g_plan.state);

	/*
	 * A factory reset clears `ptp.icv.key` (CFG_F_SECRET|CFG_F_NOEXPORT)
	 * and the PTP group's applier re-runs the planner. Policy is 0 again by
	 * then, so the plan is OFF with an empty table — and ptp_icv_set_cfg()
	 * overwrites the context's whole ptp_icv_cfg_t with it, key material
	 * included, rather than waiting for the reboot.
	 */
	(void)memset(&g_w, 0, sizeof(g_w));
	plan();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_OFF, g_plan.state);
	assert_disarmed("factory-reset defaults over a live key");
}

/* ===================================================================== */
/* 5. the window clamp                                                    */
/* ===================================================================== */

/**
 * 0 is not "no window": ptp_icv.h treats it as 1 so a zeroed config fails
 * closed, and the planner clamps here as well so the value the glue logs is the
 * value the engine enforces. A log line that says 0 while the engine enforces 1
 * is how an operator concludes replay protection is off when it is not.
 */
static void test_a_zero_window_is_clamped_to_one(void)
{
	want_armable();
	g_w.replay_window = 0U;
	plan();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_ARMED, g_plan.state);
	TEST_ASSERT_EQUAL_UINT8(1U, g_cfg.replay_window);

	want_armable();
	g_w.key_len = 0U;
	g_w.replay_window = 0U;
	plan();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_NO_KEY, g_plan.state);
	TEST_ASSERT_EQUAL_UINT8(1U, g_cfg.replay_window);
}

/** Every other in-range width passes through untouched. */
static void test_an_in_range_window_passes_through(void)
{
	unsigned int w;

	for (w = 1U; w <= (unsigned int)PTP_ICV_REPLAY_WINDOW_MAX; w++) {
		want_armable();
		g_w.replay_window = (uint8_t)w;
		plan();
		TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_ARMED,
					g_plan.state);
		TEST_ASSERT_EQUAL_UINT8((uint8_t)w, g_cfg.replay_window);
	}
}

/** The two booleans reach the cfg unaltered, in both senses. */
static void test_the_flags_reach_the_engine(void)
{
	unsigned int bits;

	for (bits = 0U; bits < 4U; bits++) {
		want_armable();
		g_w.replay_protect = (bits & 1U) != 0U;
		g_w.mask_mutable = (bits & 2U) != 0U;
		plan();

		TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_PTP_ICV_ARMED,
					g_plan.state);
		TEST_ASSERT_EQUAL_INT((int)g_w.replay_protect,
				      (int)g_cfg.replay_protect);
		TEST_ASSERT_EQUAL_INT((int)g_w.mask_mutable,
				      (int)g_cfg.mask_mutable);
	}
}

/* ===================================================================== */
/* 6. purity                                                              */
/* ===================================================================== */

/**
 * No globals, no clock, and @p w is not modified.
 *
 * The glue calls this from the CFG_G_PTP applier with a stack copy of the
 * config values; a planner that wrote back through @p w, or that carried state
 * between calls, would make the second commit of a session differ from the
 * first for no visible reason.
 */
static void test_the_planner_is_pure(void)
{
	sts_ptp_icv_want_t before;
	ptp_icv_cfg_t first;
	sts_ptp_icv_plan_t first_plan;

	want_armable();
	before = g_w;
	plan();
	TEST_ASSERT_EQUAL_HEX8_ARRAY(&before, &g_w, sizeof(before));

	first = g_cfg;
	first_plan = g_plan;

	/*
	 * Same input, output buffers full of junk: the same answer, and — for
	 * the cfg — the same answer *byte for byte*. Every field of the
	 * produced ptp_icv_cfg_t is written, so none of the junk survives into
	 * a structure that is about to be memcpy'd into a live context.
	 *
	 * The plan struct is compared field by field rather than as bytes: it
	 * is { uint8_t, bool, int } and its two padding bytes are nobody's to
	 * write.
	 */
	(void)memset(&g_cfg, 0x77, sizeof(g_cfg));
	(void)memset(&g_plan, 0x77, sizeof(g_plan));
	plan();
	TEST_ASSERT_EQUAL_HEX8_ARRAY(&first, &g_cfg, sizeof(first));
	TEST_ASSERT_EQUAL_UINT8(first_plan.state, g_plan.state);
	TEST_ASSERT_EQUAL_INT((int)first_plan.attach, (int)g_plan.attach);
	TEST_ASSERT_EQUAL_INT(first_plan.rc, g_plan.rc);
}

/** A NULL output pointer is ignored rather than dereferenced. */
static void test_null_outputs_are_tolerated(void)
{
	sts_ptp_icv_plan_t out;
	ptp_icv_cfg_t cfg;

	want_armable();

	(void)memset(&out, 0x33, sizeof(out));
	sts_ptp_icv_plan(&g_w, NULL, &out);
	/* Nothing was written: the sentinel survives. */
	TEST_ASSERT_EQUAL_UINT8(0x33U, out.state);

	(void)memset(&cfg, 0x33, sizeof(cfg));
	sts_ptp_icv_plan(&g_w, &cfg, NULL);
	TEST_ASSERT_EQUAL_UINT8(0x33U, cfg.policy);

	sts_ptp_icv_plan(NULL, NULL, NULL);
}

/* ===================================================================== */
/* 7. the names                                                           */
/* ===================================================================== */

static void test_every_state_has_a_distinct_name(void)
{
	unsigned int i;
	unsigned int j;

	for (i = 0U; i < (unsigned int)STS_PTP_ICV_STATE_COUNT; i++) {
		const char *ni = sts_ptp_icv_state_name((uint8_t)i);

		TEST_ASSERT_NOT_NULL(ni);
		TEST_ASSERT_TRUE(strlen(ni) > 0U);
		TEST_ASSERT_NOT_EQUAL_INT(0, strcmp(ni, "?"));
		for (j = i + 1U; j < (unsigned int)STS_PTP_ICV_STATE_COUNT;
		     j++) {
			TEST_ASSERT_NOT_EQUAL_INT(
				0, strcmp(ni, sts_ptp_icv_state_name(
						      (uint8_t)j)));
		}
	}

	/*
	 * "armed-no-key" rather than "no-key": the log line has to say the
	 * context IS attached, because that is the surprising half — an
	 * operator reading "no-key" would reasonably conclude nothing is
	 * enforcing anything, which under REQUIRE is the opposite of the truth.
	 */
	TEST_ASSERT_EQUAL_STRING("off",
				 sts_ptp_icv_state_name(
					 (uint8_t)STS_PTP_ICV_OFF));
	TEST_ASSERT_EQUAL_STRING("armed",
				 sts_ptp_icv_state_name(
					 (uint8_t)STS_PTP_ICV_ARMED));
	TEST_ASSERT_EQUAL_STRING("refused",
				 sts_ptp_icv_state_name(
					 (uint8_t)STS_PTP_ICV_REFUSED));
	TEST_ASSERT_NOT_NULL(strstr(sts_ptp_icv_state_name(
					    (uint8_t)STS_PTP_ICV_NO_KEY),
				    "no-key"));
	TEST_ASSERT_EQUAL_STRING("?", sts_ptp_icv_state_name(0xFFU));
}

/** A keyless cfg is keyless; a NULL one is treated as such. */
static void test_the_keyless_predicate_sees_a_single_stray_byte(void)
{
	ptp_icv_cfg_t cfg;

	TEST_ASSERT_TRUE(sts_ptp_icv_cfg_is_keyless(NULL));

	(void)memset(&cfg, 0, sizeof(cfg));
	TEST_ASSERT_TRUE(sts_ptp_icv_cfg_is_keyless(&cfg));

	/* Each of the three things that count as "key material left behind". */
	cfg.key[PTP_ICV_MAX_KEYS - 1U].in_use = true;
	TEST_ASSERT_FALSE(sts_ptp_icv_cfg_is_keyless(&cfg));

	(void)memset(&cfg, 0, sizeof(cfg));
	cfg.key[1].key_len = 1U;
	TEST_ASSERT_FALSE(sts_ptp_icv_cfg_is_keyless(&cfg));

	(void)memset(&cfg, 0, sizeof(cfg));
	cfg.key[2].key[PTP_ICV_KEY_MAX - 1U] = 0x01U;
	TEST_ASSERT_FALSE_MESSAGE(sts_ptp_icv_cfg_is_keyless(&cfg),
				  "a freed slot still holding its bytes is not "
				  "keyless");
}

/* ===================================================================== */
/* 8. the whole domain                                                    */
/* ===================================================================== */

/**
 * Every invariant at once, over the cross product the header describes.
 *
 * The positive half matters as much as the negative: a planner that refused
 * everything would satisfy "nothing arms that should not" perfectly.
 */
static void test_the_arming_conjunction_over_the_whole_domain(void)
{
	static const size_t lens[] = { 0U, 1U, 16U, 32U, 64U, 65U, 200U };
	static const uint8_t suites[] = { 0U, 1U, 2U, 200U };
	static const uint8_t windows[] = { 0U, 1U, 16U, 32U, 33U, 255U };
	unsigned int pol;
	size_t li;
	size_t si;
	size_t wi;
	unsigned int armed = 0U;
	unsigned int no_key = 0U;
	unsigned int off = 0U;
	unsigned int refused = 0U;

	for (pol = 0U; pol < 5U; pol++) {
		for (li = 0U; li < (sizeof(lens) / sizeof(lens[0])); li++) {
			for (si = 0U;
			     si < (sizeof(suites) / sizeof(suites[0])); si++) {
				for (wi = 0U;
				     wi < (sizeof(windows) / sizeof(windows[0]));
				     wi++) {
					bool want_arm;
					char msg[144];

					want_armable();
					g_w.policy = (uint8_t)pol;
					g_w.key_len = lens[li];
					g_w.suite = suites[si];
					g_w.replay_window = windows[wi];
					plan();

					(void)snprintf(
						msg, sizeof(msg),
						"policy=%u len=%zu suite=%u "
						"window=%u -> %s rc=%d",
						pol, lens[li],
						(unsigned int)suites[si],
						(unsigned int)windows[wi],
						sts_ptp_icv_state_name(
							g_plan.state),
						g_plan.rc);

					/* attach is exactly the two attached
					 * states, never a fourth answer. */
					TEST_ASSERT_EQUAL_INT_MESSAGE(
						(int)((g_plan.state ==
						       (uint8_t)STS_PTP_ICV_NO_KEY) ||
						      (g_plan.state ==
						       (uint8_t)STS_PTP_ICV_ARMED)),
						(int)g_plan.attach, msg);

					/* rc is non-zero exactly on REFUSED. */
					TEST_ASSERT_EQUAL_INT_MESSAGE(
						(int)(g_plan.state ==
						      (uint8_t)STS_PTP_ICV_REFUSED),
						(int)(g_plan.rc != 0), msg);

					/* OFF is policy 0 and nothing else. */
					TEST_ASSERT_EQUAL_INT_MESSAGE(
						(int)(pol == 0U),
						(int)(g_plan.state ==
						      (uint8_t)STS_PTP_ICV_OFF),
						msg);

					/* Anything not armed holds no key and
					 * loads no policy. */
					if (g_plan.state !=
					    (uint8_t)STS_PTP_ICV_ARMED) {
						TEST_ASSERT_TRUE_MESSAGE(
							sts_ptp_icv_cfg_is_keyless(
								&g_cfg),
							msg);
					}
					if ((g_plan.state ==
					     (uint8_t)STS_PTP_ICV_OFF) ||
					    (g_plan.state ==
					     (uint8_t)STS_PTP_ICV_REFUSED)) {
						TEST_ASSERT_EQUAL_UINT8_MESSAGE(
							(uint8_t)PTP_ICV_POLICY_OFF,
							g_cfg.policy, msg);
					}

					/* And the positive statement: a usable
					 * configuration always arms. */
					want_arm = (pol >= 1U) && (pol <= 2U) &&
						   (lens[li] >= 1U) &&
						   (lens[li] <=
						    (size_t)PTP_ICV_KEY_MAX) &&
						   (suites[si] <
						    (uint8_t)PTP_ICV_SUITE_COUNT) &&
						   (windows[wi] <=
						    (uint8_t)PTP_ICV_REPLAY_WINDOW_MAX);
					TEST_ASSERT_EQUAL_INT_MESSAGE(
						(int)want_arm,
						(int)(g_plan.state ==
						      (uint8_t)STS_PTP_ICV_ARMED),
						msg);

					/*
					 * The planner can only produce the two
					 * errors it can cause. -ENOSPC and
					 * -ENOENT are reachable in core but not
					 * from here: it always plans into a
					 * fresh table and always names the
					 * association it just installed. A
					 * different errno means one of those
					 * two properties broke.
					 */
					TEST_ASSERT_TRUE_MESSAGE(
						(g_plan.rc == 0) ||
							(g_plan.rc == -EINVAL),
						msg);

					switch (g_plan.state) {
					case (uint8_t)STS_PTP_ICV_ARMED:
						armed++;
						break;
					case (uint8_t)STS_PTP_ICV_NO_KEY:
						no_key++;
						break;
					case (uint8_t)STS_PTP_ICV_OFF:
						off++;
						break;
					default:
						refused++;
						break;
					}
				}
			}
		}
	}

	/* Not vacuous: the sweep really visited all four states. */
	TEST_ASSERT_TRUE(armed > 0U);
	TEST_ASSERT_TRUE(no_key > 0U);
	TEST_ASSERT_TRUE(off > 0U);
	TEST_ASSERT_TRUE(refused > 0U);
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_policy_off_is_the_default_and_detaches);
	RUN_TEST(test_policy_off_drops_a_provisioned_key);
	RUN_TEST(test_the_shipped_default_differs_from_cores_default);

	RUN_TEST(test_verify_if_present_without_a_key_attaches_inert);
	RUN_TEST(test_require_without_a_key_attaches_rather_than_downgrading);

	RUN_TEST(test_a_policy_and_a_key_arms);
	RUN_TEST(test_both_policies_and_both_suites_arm);
	RUN_TEST(test_the_key_length_boundary_is_cores);

	RUN_TEST(test_a_null_request_is_a_refusal_not_an_off);
	RUN_TEST(test_an_unknown_policy_is_refused);
	RUN_TEST(test_an_unknown_suite_is_refused);
	RUN_TEST(test_a_window_out_of_range_is_refused_on_both_paths);
	RUN_TEST(test_a_refusal_overwrites_a_previously_keyed_cfg);
	RUN_TEST(test_switching_off_overwrites_a_previously_keyed_cfg);

	RUN_TEST(test_a_zero_window_is_clamped_to_one);
	RUN_TEST(test_an_in_range_window_passes_through);
	RUN_TEST(test_the_flags_reach_the_engine);

	RUN_TEST(test_the_planner_is_pure);
	RUN_TEST(test_null_outputs_are_tolerated);

	RUN_TEST(test_every_state_has_a_distinct_name);
	RUN_TEST(test_the_keyless_predicate_sees_a_single_stray_byte);

	RUN_TEST(test_the_arming_conjunction_over_the_whole_domain);

	return UNITY_END();
}
