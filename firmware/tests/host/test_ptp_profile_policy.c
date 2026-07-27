/*
 * STS1000 "Meridian" — the PTP profile/operator precedence rule, tested away
 * from Zephyr.
 *
 * sts_ptp_profile_policy.h resolves one question per profile-owned field: does
 * the value come from the profile descriptor or from the operator's cfg key?
 * It is a Zephyr-free header for the reason all of them are — the defect it
 * replaces was invisible at runtime.
 *
 * What that defect looked like, because it is what these tests are guarding
 * against coming back: sts_ptp_start() read every profile-owned field from its
 * own cfg key, whose schema defaults are the *Default profile's* values, and
 * then set `.profile` last. Selecting G.8275.1 therefore applied none of
 * G.8275.1 — and because ptp_cfg_validate() enforces that profile's normative
 * ranges, the resulting mismatch was rejected and the rejection path called
 * ptp_cfg_defaults(), which erases `.profile` too. The unit came up as a
 * Default-profile grandmaster and said so only as "falling back to defaults".
 *
 * Two of these tests are therefore about a log line rather than a value
 * (`report_discard`). That is deliberate: the silent part of the failure was
 * the expensive part.
 */

#include <stdbool.h>
#include <stdint.h>

#include "unity.h"

#include "zephyr/net/sts_ptp_profile_policy.h"

/*
 * The real numbers, so a test that passes is saying something about this board
 * rather than about arithmetic. Schema defaults are core/cfg/cfg_schema.h's;
 * the G.8275.1 column is core/ptp/ptp_profile.c's descriptor.
 *
 *   field            schema default   G.8275.1
 *   domain                        0         24
 *   log_sync                      0         -4
 *   log_announce                  1         -3
 *   transport      UDP_IPV4 (0)         L2 (2)
 */
#define SCHEMA_DOMAIN 0U
#define G82751_DOMAIN 24U
#define SCHEMA_LOG_SYNC 0
#define G82751_LOG_SYNC (-4)

/* PTP_PROFILE_* as core/ptp declares them; passed in rather than included so
 * the header under test stays free of the engine. */
#define PROFILE_DEFAULT 0U
#define PROFILE_G82751 1U
#define PROFILE_COUNT 4U

void setUp(void) {}
void tearDown(void) {}

/* ===================================================================== *
 *  pick_u64 — the precedence rule itself
 * ===================================================================== */

static void test_an_untouched_key_takes_the_profiles_value(void)
{
	/* The whole bug in one assertion: domain still at the schema default
	 * must become 24 under G.8275.1, not stay 0. */
	TEST_ASSERT_EQUAL_UINT64(
		G82751_DOMAIN,
		sts_ptp_prof_pick_u64(SCHEMA_DOMAIN, SCHEMA_DOMAIN,
				      G82751_DOMAIN));
}

static void test_an_operator_set_key_beats_the_profile(void)
{
	/* An operator running G.8275.1 on domain 30 (inside 24..43) keeps 30. */
	TEST_ASSERT_EQUAL_UINT64(
		30U, sts_ptp_prof_pick_u64(30U, SCHEMA_DOMAIN, G82751_DOMAIN));
}

static void test_the_default_profile_is_a_no_op_for_untouched_keys(void)
{
	/* Under Default, profile value == schema default, so either branch
	 * gives the same answer — the rule must not perturb the Default path. */
	TEST_ASSERT_EQUAL_UINT64(SCHEMA_DOMAIN,
				 sts_ptp_prof_pick_u64(SCHEMA_DOMAIN,
						       SCHEMA_DOMAIN,
						       SCHEMA_DOMAIN));
	TEST_ASSERT_EQUAL_UINT64(7U, sts_ptp_prof_pick_u64(7U, SCHEMA_DOMAIN,
							   SCHEMA_DOMAIN));
}

static void test_the_documented_cost_is_what_the_header_says(void)
{
	/*
	 * Pinning the known limitation rather than pretending it is absent: an
	 * operator who deliberately types the schema default under a
	 * non-Default profile gets the profile's value. If someone later adds
	 * a real "is set" bit to cfg, this test is the one that should fail and
	 * be deleted — which is why it is here rather than only in a comment.
	 */
	TEST_ASSERT_EQUAL_UINT64(
		G82751_DOMAIN,
		sts_ptp_prof_pick_u64(SCHEMA_DOMAIN, SCHEMA_DOMAIN,
				      G82751_DOMAIN));
}

static void test_zero_is_not_special(void)
{
	/* A stored 0 against a non-zero schema default is an operator decision
	 * like any other — the rule must compare, not test for truthiness. */
	TEST_ASSERT_EQUAL_UINT64(0U, sts_ptp_prof_pick_u64(0U, 128U, 200U));
}

static void test_the_full_unsigned_range_round_trips(void)
{
	TEST_ASSERT_EQUAL_UINT64(UINT64_MAX,
				 sts_ptp_prof_pick_u64(UINT64_MAX, 0U, 5U));
	/* stored == schema_def at the top of the range still yields profile. */
	TEST_ASSERT_EQUAL_UINT64(
		5U, sts_ptp_prof_pick_u64(UINT64_MAX, UINT64_MAX, 5U));
}

/* ===================================================================== *
 *  pick_i32 — the three log intervals, where the values are negative
 * ===================================================================== */

static void test_a_negative_profile_interval_is_applied(void)
{
	/* G.8275.1's 16/s Sync is log -4. A sign bug here would silently serve
	 * 16 s Sync instead of 1/16 s, which is the same field and 256x wrong. */
	TEST_ASSERT_EQUAL_INT32(
		G82751_LOG_SYNC,
		sts_ptp_prof_pick_i32(SCHEMA_LOG_SYNC, SCHEMA_LOG_SYNC,
				      G82751_LOG_SYNC));
}

static void test_an_operator_interval_beats_a_negative_profile_one(void)
{
	TEST_ASSERT_EQUAL_INT32(-3, sts_ptp_prof_pick_i32(-3, SCHEMA_LOG_SYNC,
							  G82751_LOG_SYNC));
}

static void test_a_negative_schema_default_is_compared_not_assumed(void)
{
	/* schema_def itself negative: stored equal to it must still yield the
	 * profile's value. */
	TEST_ASSERT_EQUAL_INT32(2, sts_ptp_prof_pick_i32(-1, -1, 2));
	TEST_ASSERT_EQUAL_INT32(-1, sts_ptp_prof_pick_i32(-1, 0, 2));
}

static void test_the_signed_extremes_round_trip(void)
{
	TEST_ASSERT_EQUAL_INT32(INT32_MIN,
				sts_ptp_prof_pick_i32(INT32_MIN, 0, 4));
	TEST_ASSERT_EQUAL_INT32(INT32_MAX,
				sts_ptp_prof_pick_i32(INT32_MAX, 0, 4));
}

/* ===================================================================== *
 *  clamp — a profile id from a firmware that is not this one
 * ===================================================================== */

static void test_every_known_profile_survives_the_clamp(void)
{
	unsigned int i;

	for (i = 0U; i < PROFILE_COUNT; i++) {
		TEST_ASSERT_EQUAL_UINT8((uint8_t)i,
					sts_ptp_prof_clamp((uint64_t)i,
							   PROFILE_COUNT,
							   PROFILE_DEFAULT));
	}
}

static void test_a_profile_from_a_newer_firmware_clamps_to_default(void)
{
	/* The downgrade case: a stored id this build does not know must boot as
	 * Default rather than index off the end of the descriptor table. */
	TEST_ASSERT_EQUAL_UINT8(PROFILE_DEFAULT,
				sts_ptp_prof_clamp((uint64_t)PROFILE_COUNT,
						   PROFILE_COUNT,
						   PROFILE_DEFAULT));
	TEST_ASSERT_EQUAL_UINT8(PROFILE_DEFAULT,
				sts_ptp_prof_clamp(UINT64_MAX, PROFILE_COUNT,
						   PROFILE_DEFAULT));
}

static void test_an_empty_profile_table_clamps_rather_than_divides(void)
{
	TEST_ASSERT_EQUAL_UINT8(PROFILE_DEFAULT,
				sts_ptp_prof_clamp(0U, 0U, PROFILE_DEFAULT));
}

/* ===================================================================== *
 *  report_discard — the log line that was missing
 * ===================================================================== */

static void test_a_discarded_telecom_profile_is_reported(void)
{
	TEST_ASSERT_TRUE(
		sts_ptp_prof_report_discard(PROFILE_G82751, PROFILE_DEFAULT));
}

static void test_a_default_profile_falling_back_is_not_news(void)
{
	/* Default rejected and "falling back to Default" is noise, not a
	 * discarded selection. */
	TEST_ASSERT_FALSE(
		sts_ptp_prof_report_discard(PROFILE_DEFAULT, PROFILE_DEFAULT));
}

static void test_every_non_default_profile_reports(void)
{
	unsigned int i;

	for (i = 1U; i < PROFILE_COUNT; i++) {
		TEST_ASSERT_TRUE_MESSAGE(
			sts_ptp_prof_report_discard((uint8_t)i,
						    PROFILE_DEFAULT),
			"a discarded non-Default profile must be named");
	}
}

/* ===================================================================== *
 *  The composite each field actually goes through
 * ===================================================================== */

static void test_a_fresh_unit_selecting_g82751_gets_g82751(void)
{
	/*
	 * End to end on the two fields whose wrong values were the visible
	 * symptom: a unit whose cfg has never been touched, selecting
	 * G.8275.1, must come out with domain 24 and log_sync -4 — not the
	 * schema's 0 and 0.
	 */
	uint8_t prof = sts_ptp_prof_clamp((uint64_t)PROFILE_G82751,
					  PROFILE_COUNT, PROFILE_DEFAULT);
	uint64_t domain;
	int32_t sync;

	TEST_ASSERT_EQUAL_UINT8(PROFILE_G82751, prof);

	domain = sts_ptp_prof_pick_u64(SCHEMA_DOMAIN, SCHEMA_DOMAIN,
				       G82751_DOMAIN);
	sync = sts_ptp_prof_pick_i32(SCHEMA_LOG_SYNC, SCHEMA_LOG_SYNC,
				     G82751_LOG_SYNC);

	TEST_ASSERT_EQUAL_UINT64(G82751_DOMAIN, domain);
	TEST_ASSERT_EQUAL_INT32(G82751_LOG_SYNC, sync);
	TEST_ASSERT_TRUE_MESSAGE(domain >= 24U && domain <= 43U,
				 "G.8275.1 permits only domains 24..43");
}

static void test_a_commissioned_unit_keeps_what_the_operator_typed(void)
{
	/* Same selection, but the operator has set domain 30 and Sync 1/8 s. */
	uint64_t domain = sts_ptp_prof_pick_u64(30U, SCHEMA_DOMAIN,
						G82751_DOMAIN);
	int32_t sync = sts_ptp_prof_pick_i32(-3, SCHEMA_LOG_SYNC,
					     G82751_LOG_SYNC);

	TEST_ASSERT_EQUAL_UINT64(30U, domain);
	TEST_ASSERT_EQUAL_INT32(-3, sync);
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_an_untouched_key_takes_the_profiles_value);
	RUN_TEST(test_an_operator_set_key_beats_the_profile);
	RUN_TEST(test_the_default_profile_is_a_no_op_for_untouched_keys);
	RUN_TEST(test_the_documented_cost_is_what_the_header_says);
	RUN_TEST(test_zero_is_not_special);
	RUN_TEST(test_the_full_unsigned_range_round_trips);

	RUN_TEST(test_a_negative_profile_interval_is_applied);
	RUN_TEST(test_an_operator_interval_beats_a_negative_profile_one);
	RUN_TEST(test_a_negative_schema_default_is_compared_not_assumed);
	RUN_TEST(test_the_signed_extremes_round_trip);

	RUN_TEST(test_every_known_profile_survives_the_clamp);
	RUN_TEST(test_a_profile_from_a_newer_firmware_clamps_to_default);
	RUN_TEST(test_an_empty_profile_table_clamps_rather_than_divides);

	RUN_TEST(test_a_discarded_telecom_profile_is_reported);
	RUN_TEST(test_a_default_profile_falling_back_is_not_news);
	RUN_TEST(test_every_non_default_profile_reports);

	RUN_TEST(test_a_fresh_unit_selecting_g82751_gets_g82751);
	RUN_TEST(test_a_commissioned_unit_keeps_what_the_operator_typed);

	return UNITY_END();
}
