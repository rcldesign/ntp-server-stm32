/*
 * STS1000 "Meridian" — the supervisor's fail-safe decisions, tested away from
 * Zephyr.
 *
 * platform/sts_super_policy.h holds what supervisor.c decides before it touches
 * WDT_KICK, K2 and the status RGB. Each of the three is a fail-safe output,
 * which is exactly why a defect here is invisible: the safe direction looks
 * like a working box until the day the unsafe one is needed.
 *
 *   - The liveness AND-gate is only exercised when something has actually gone
 *     quiet, which never happens on a bench. An inverted or over-permissive
 *     gate feeds the TPS3430 through a wedge and the external watchdog stops
 *     being a watchdog. Nothing annunciates that.
 *   - The relay's three gates each vote only for energizing, so dropping one
 *     produces a relay that says "service OK" a few minutes early — during
 *     warm-up, or before pwrseq reaches stage 9. Downstream that is a monitoring
 *     system told the grandmaster is traceable when it is not.
 *   - The identify expiry is signed 32-bit wrap arithmetic. Unsigned, it is
 *     wrong for 24 days out of every 49.7 — and only for units that have been
 *     up that long, i.e. exactly the ones nobody reboots to test.
 *   - The §8.3 self-confirm gate is the moment MCUboot's automatic revert stops
 *     protecting a remote box. Confirming one term too early throws that away
 *     with no symptom at all until a bad image ships.
 *
 * The TPS3430 cadence arithmetic itself is core/pwrseq's and is covered by the
 * `pwrseq` suite; this file only pins the permission and the annunciation.
 */

#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "zephyr/platform/sts_super_policy.h"

/* --------------------------------------------------------------- fixtures - */

static quality_block_t q_serving(void)
{
	quality_block_t q;

	memset(&q, 0, sizeof(q));
	q.stratum = QUALITY_STRATUM_PRIMARY;
	q.lock_state = (uint8_t)QUALITY_LOCK_LOCKED;
	q.flags = QUALITY_FLAG_OCXO_WARM;

	return q;
}

/* --------------------------------------------------------- liveness gate -- */

/*
 * The AND-gate. An empty stale mask is the only input that may permit a kick;
 * ANY set bit withholds, whichever participant it belongs to. The registry has
 * up to 12 slots and pwrseq's word has 3 bits, so the mapping is deliberately
 * all-or-nothing rather than bit-for-bit.
 */
static void test_a_single_stale_participant_withholds_the_kick(void)
{
	TEST_ASSERT_EQUAL_UINT32(PWRSEQ_LIVE_ALL, sts_super_wdt_liveness(0U));

	for (unsigned int i = 0; i < 12U; i++) {
		char msg[64];

		snprintf(msg, sizeof(msg), "participant %u stale still permitted a kick",
			 i);
		TEST_ASSERT_EQUAL_UINT32_MESSAGE(
			0U, sts_super_wdt_liveness(1U << i), msg);
	}

	/* And a participant beyond pwrseq's own three bits still withholds —
	 * the registry, not the word width, is the authority on who must live. */
	TEST_ASSERT_EQUAL_UINT32(0U, sts_super_wdt_liveness(1U << 11));
	TEST_ASSERT_EQUAL_UINT32(0U, sts_super_wdt_liveness(0xFFFFFFFFU));
}

/* Permitting a kick means permitting ALL of pwrseq's bits; a partial word would
 * be refused by pwrseq_wdt_service() and silently withhold forever. */
static void test_a_permitted_kick_sets_every_pwrseq_liveness_bit(void)
{
	uint32_t live = sts_super_wdt_liveness(0U);

	TEST_ASSERT_TRUE((live & (uint32_t)PWRSEQ_LIVE_TIMING) != 0U);
	TEST_ASSERT_TRUE((live & (uint32_t)PWRSEQ_LIVE_NETWORK) != 0U);
	TEST_ASSERT_TRUE((live & (uint32_t)PWRSEQ_LIVE_HOUSEKEEPING) != 0U);
}

/* Liveness loss is logged at CRIT from a 4 Hz tick, so it must be edge-driven
 * and armed-gated or it buries its own cause in the ring. */
static void test_stale_logging_is_edge_driven_and_armed_gated(void)
{
	/* Not armed: a participant that has not started yet is not news. */
	TEST_ASSERT_FALSE(sts_super_stale_log(false, 0x4U, 0U));

	/* First appearance. */
	TEST_ASSERT_TRUE(sts_super_stale_log(true, 0x4U, 0U));
	/* Same mask next tick — silence. */
	TEST_ASSERT_FALSE(sts_super_stale_log(true, 0x4U, 0x4U));
	/* A second participant joining it is a different picture. */
	TEST_ASSERT_TRUE(sts_super_stale_log(true, 0x6U, 0x4U));
	/* Recovery is not logged here (the kick resuming is the evidence). */
	TEST_ASSERT_FALSE(sts_super_stale_log(true, 0U, 0x6U));
}

/* A window violation means a second writer is on WDI and the board is
 * cold-cycling itself; the report must survive a counter wrap. */
static void test_violation_reporting_is_change_driven(void)
{
	TEST_ASSERT_FALSE(sts_super_violation_new(0U, 0U));
	TEST_ASSERT_TRUE(sts_super_violation_new(1U, 0U));
	TEST_ASSERT_FALSE(sts_super_violation_new(7U, 7U));
	TEST_ASSERT_TRUE(sts_super_violation_new(0U, UINT32_MAX));
}

/*
 * EARLY is "below the window" — the runaway direction, which is the one a
 * cadence bug actually produces (a kick per housekeeping tick). Reporting it as
 * "above" points the reader at a stall and away from the real cause.
 */
static void test_violation_word_names_the_right_side_of_the_window(void)
{
	TEST_ASSERT_EQUAL_STRING(
		"below",
		sts_super_violation_word((uint8_t)PWRSEQ_WDT_INTERVAL_EARLY));
	TEST_ASSERT_EQUAL_STRING(
		"above",
		sts_super_violation_word((uint8_t)PWRSEQ_WDT_INTERVAL_LATE));
	TEST_ASSERT_EQUAL_STRING(
		"above", sts_super_violation_word((uint8_t)PWRSEQ_WDT_INTERVAL_OK));
}

/* ---------------------------------------------------------------- relay --- */

/* All three gates required. Each one removed on its own must de-energize. */
static void test_relay_needs_all_three_gates(void)
{
	quality_block_t q = q_serving();

	TEST_ASSERT_TRUE(sts_super_relay_want(true, true, &q));

	TEST_ASSERT_FALSE_MESSAGE(
		sts_super_relay_want(false, true, &q),
		"K2 energized before pwrseq reached stage 9");
	TEST_ASSERT_FALSE_MESSAGE(sts_super_relay_want(true, false, &q),
				  "K2 energized with a disqualifying alarm active");
}

/*
 * "Serving" is two facts, not one. A locked loop that is not advertising
 * stratum 1, and a stratum-1 advertisement without a lock, are both boxes whose
 * relay must stay in ALARM — and each is one dropped conjunct away.
 */
static void test_relay_requires_both_halves_of_serving(void)
{
	quality_block_t q = q_serving();

	q.lock_state = (uint8_t)QUALITY_LOCK_HOLDOVER;
	TEST_ASSERT_FALSE_MESSAGE(sts_super_relay_want(true, true, &q),
				  "K2 energized in holdover");

	q = q_serving();
	q.lock_state = (uint8_t)QUALITY_LOCK_LOCKING;
	TEST_ASSERT_FALSE(sts_super_relay_want(true, true, &q));

	q = q_serving();
	q.stratum = QUALITY_STRATUM_UNSYNC;
	TEST_ASSERT_FALSE_MESSAGE(sts_super_relay_want(true, true, &q),
				  "K2 energized while unsynchronised");

	/* A never-published block: stratum 0, lock UNKNOWN. */
	memset(&q, 0, sizeof(q));
	TEST_ASSERT_FALSE(sts_super_relay_want(true, true, &q));
}

/* De-energized is the safe state, so a missing quality block must not energize. */
static void test_relay_treats_a_missing_quality_block_as_not_serving(void)
{
	TEST_ASSERT_FALSE(sts_super_relay_want(true, true, NULL));
	TEST_ASSERT_FALSE(sts_super_serving(NULL));
}

/* ------------------------------------------------------------------ RGB --- */

/*
 * The identify deadline is a monotonic uint32 that wraps every 49.7 days. An
 * identify started 30 s before the wrap has a deadline numerically *below*
 * `now`, which an unsigned comparison ends immediately; the mirror case keeps
 * it running for 24 days. Both are silent — nobody watches an LED for a month.
 */
static void test_identify_expiry_survives_the_32_bit_wrap(void)
{
	/* Normal case: 5 s remaining. */
	TEST_ASSERT_TRUE(sts_super_identify_active(15000U, 10000U));
	/* Exactly expired, and past. */
	TEST_ASSERT_FALSE(sts_super_identify_active(10000U, 10000U));
	TEST_ASSERT_FALSE(sts_super_identify_active(10000U, 10001U));

	/* Started 1 s before the wrap, 29 s still to run: deadline has wrapped
	 * to a small number while `now` is still huge. */
	TEST_ASSERT_TRUE_MESSAGE(
		sts_super_identify_active(29000U, UINT32_MAX - 1000U),
		"identify ended at the uptime wrap");

	/* The mirror: a deadline 30 s in the past, straddling the wrap. */
	TEST_ASSERT_FALSE_MESSAGE(
		sts_super_identify_active(UINT32_MAX - 1000U, 29000U),
		"an expired identify was still running after the wrap");
}

/* 0 is the "nothing pending" sentinel and must never read as active, whatever
 * `now` happens to be. */
static void test_identify_zero_deadline_is_the_off_sentinel(void)
{
	TEST_ASSERT_FALSE(sts_super_identify_active(0U, 0U));
	TEST_ASSERT_FALSE(sts_super_identify_active(0U, 1U));
	TEST_ASSERT_FALSE(sts_super_identify_active(0U, UINT32_MAX));
}

/* A serving box is green: locked, not warming, not in holdover. */
static void test_rgb_inputs_for_a_serving_box(void)
{
	quality_block_t q = q_serving();
	fault_rgb_in_t in;

	sts_super_rgb_in(&q, false, false, &in);

	TEST_ASSERT_TRUE(in.locked);
	TEST_ASSERT_FALSE(in.warming);
	TEST_ASSERT_FALSE(in.holdover);
	TEST_ASSERT_FALSE(in.any_fault);
	TEST_ASSERT_FALSE(in.identify);
	TEST_ASSERT_EQUAL_INT(FAULT_RGB_GREEN, fault_rgb_state(&in));
}

/*
 * "Warming" is an OR of three conditions and each one alone must set it. A cold
 * oven with a converged loop is not green, and a warm oven that is still
 * acquiring is not green either — dropping either disjunct produces a box that
 * shows green while it is still settling.
 */
static void test_warming_is_raised_by_any_of_its_three_causes(void)
{
	quality_block_t q;
	fault_rgb_in_t in;

	q = q_serving();
	q.flags &= (uint32_t)~QUALITY_FLAG_OCXO_WARM;
	sts_super_rgb_in(&q, false, false, &in);
	TEST_ASSERT_TRUE_MESSAGE(in.warming, "a cold oven did not read as warming");

	q = q_serving();
	q.lock_state = (uint8_t)QUALITY_LOCK_ACQUIRING;
	sts_super_rgb_in(&q, false, false, &in);
	TEST_ASSERT_TRUE(in.warming);
	TEST_ASSERT_FALSE(in.locked);

	q = q_serving();
	q.lock_state = (uint8_t)QUALITY_LOCK_LOCKING;
	sts_super_rgb_in(&q, false, false, &in);
	TEST_ASSERT_TRUE(in.warming);
}

/* Holdover comes straight from the block and must reach the encoder. */
static void test_holdover_reaches_the_rgb_encoder(void)
{
	quality_block_t q = q_serving();
	fault_rgb_in_t in;

	q.lock_state = (uint8_t)QUALITY_LOCK_HOLDOVER;
	q.holdover = true;
	sts_super_rgb_in(&q, false, false, &in);

	TEST_ASSERT_TRUE(in.holdover);
	TEST_ASSERT_FALSE(in.locked);
	TEST_ASSERT_EQUAL_INT(FAULT_RGB_AMBER, fault_rgb_state(&in));
}

/* With nothing published, the oven cannot be assumed warm. Reporting a
 * never-published block as "not warming" would show green-adjacent state for a
 * box that has published nothing at all. */
static void test_rgb_inputs_without_a_quality_block_assume_warming(void)
{
	fault_rgb_in_t in;

	sts_super_rgb_in(NULL, false, false, &in);

	TEST_ASSERT_FALSE(in.locked);
	TEST_ASSERT_TRUE(in.warming);
	TEST_ASSERT_FALSE(in.holdover);
}

/* Fault and identify pass through untouched; the priority between them is
 * core/fault's, and it puts identify on top deliberately. */
static void test_fault_and_identify_pass_through(void)
{
	quality_block_t q = q_serving();
	fault_rgb_in_t in;

	sts_super_rgb_in(&q, true, false, &in);
	TEST_ASSERT_TRUE(in.any_fault);
	TEST_ASSERT_EQUAL_INT(FAULT_RGB_RED, fault_rgb_state(&in));

	sts_super_rgb_in(&q, true, true, &in);
	TEST_ASSERT_TRUE(in.identify);
	TEST_ASSERT_EQUAL_INT(FAULT_RGB_BLUE_PULSE, fault_rgb_state(&in));
}

/* --------------------------------------------------------- self-confirm --- */

/*
 * Every term required. Doing nothing IS the revert, so a term that has quietly
 * stopped being checked costs the whole protection — and the only way to notice
 * is to ship a bad image to a remote site.
 */
static void test_self_confirm_needs_every_term(void)
{
	const uint8_t locked = (uint8_t)QUALITY_LOCK_LOCKED;

	TEST_ASSERT_TRUE(sts_super_confirm_now(true, 0U, false, locked));

	TEST_ASSERT_FALSE_MESSAGE(sts_super_confirm_now(false, 0U, false, locked),
				  "confirmed an image that was not pending");
	TEST_ASSERT_FALSE_MESSAGE(sts_super_confirm_now(true, 0x1U, false, locked),
				  "confirmed with a liveness participant stale");
	TEST_ASSERT_FALSE_MESSAGE(sts_super_confirm_now(true, 0U, true, locked),
				  "confirmed with an alarm active");
}

/*
 * LOCKED specifically — not "anything better than unknown". An image whose
 * discipline loop is broken still boots, enumerates USB and answers the shell,
 * so a gate that accepted ACQUIRING or HOLDOVER would confirm exactly the
 * update the revert exists to undo.
 */
static void test_self_confirm_accepts_only_a_locked_clock(void)
{
	static const uint8_t not_locked[] = {
		(uint8_t)QUALITY_LOCK_UNKNOWN,    (uint8_t)QUALITY_LOCK_ACQUIRING,
		(uint8_t)QUALITY_LOCK_LOCKING,    (uint8_t)QUALITY_LOCK_HOLDOVER,
		(uint8_t)QUALITY_LOCK_RECOVERING, (uint8_t)QUALITY_LOCK_PARKED,
	};

	for (size_t i = 0; i < sizeof(not_locked) / sizeof(not_locked[0]); i++) {
		char msg[72];

		snprintf(msg, sizeof(msg), "confirmed with lock_state %u",
			 not_locked[i]);
		TEST_ASSERT_FALSE_MESSAGE(
			sts_super_confirm_now(true, 0U, false, not_locked[i]), msg);
	}
}

/* -------------------------------------------------------------------- main - */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_a_single_stale_participant_withholds_the_kick);
	RUN_TEST(test_a_permitted_kick_sets_every_pwrseq_liveness_bit);
	RUN_TEST(test_stale_logging_is_edge_driven_and_armed_gated);
	RUN_TEST(test_violation_reporting_is_change_driven);
	RUN_TEST(test_violation_word_names_the_right_side_of_the_window);

	RUN_TEST(test_relay_needs_all_three_gates);
	RUN_TEST(test_relay_requires_both_halves_of_serving);
	RUN_TEST(test_relay_treats_a_missing_quality_block_as_not_serving);

	RUN_TEST(test_identify_expiry_survives_the_32_bit_wrap);
	RUN_TEST(test_identify_zero_deadline_is_the_off_sentinel);
	RUN_TEST(test_rgb_inputs_for_a_serving_box);
	RUN_TEST(test_warming_is_raised_by_any_of_its_three_causes);
	RUN_TEST(test_holdover_reaches_the_rgb_encoder);
	RUN_TEST(test_rgb_inputs_without_a_quality_block_assume_warming);
	RUN_TEST(test_fault_and_identify_pass_through);

	RUN_TEST(test_self_confirm_needs_every_term);
	RUN_TEST(test_self_confirm_accepts_only_a_locked_clock);

	return UNITY_END();
}
