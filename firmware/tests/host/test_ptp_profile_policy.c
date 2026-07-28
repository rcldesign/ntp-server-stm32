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

/* ===================================================================== *
 *  An override the profile forbids — the case that used to cost the
 *  operator their whole profile
 * ===================================================================== */

/*
 * G.8275.1's real ranges (core/ptp/ptp_profile.c desc_g8275_1), against the
 * real schema defaults. Most are single values, which is exactly why the old
 * behaviour was not a corner case.
 *
 *   field           schema default   profile default   profile range
 *   domain                       0                24         24..43
 *   priority1                  128               128       128..128
 *   log_announce                 1                -3         -3..-3
 *   log_sync                     0                -4         -4..-4
 */
#define G82751_DOMAIN_LO 24U
#define G82751_DOMAIN_HI 43U
#define SCHEMA_LOG_ANNOUNCE 1
#define G82751_LOG_ANNOUNCE (-3)

static void test_an_out_of_profile_domain_is_refused_not_fatal(void)
{
	sts_ptp_pick_t how = STS_PTP_PICK_OPERATOR;
	/* Operator set domain 5: differs from the schema default, so under the
	 * old rule it won — and then failed validation, and took G.8275.1 with
	 * it. Now the field alone loses and the profile survives. */
	uint64_t v = sts_ptp_prof_pick_u64_r(5U, SCHEMA_DOMAIN, G82751_DOMAIN,
					     G82751_DOMAIN_LO, G82751_DOMAIN_HI,
					     &how);

	TEST_ASSERT_EQUAL_UINT64(G82751_DOMAIN, v);
	TEST_ASSERT_EQUAL_INT(STS_PTP_PICK_REFUSED, how);
	TEST_ASSERT_TRUE(sts_ptp_pick_notable(how));
}

static void test_an_in_profile_domain_is_still_the_operators(void)
{
	sts_ptp_pick_t how = STS_PTP_PICK_REFUSED;
	uint64_t v = sts_ptp_prof_pick_u64_r(30U, SCHEMA_DOMAIN, G82751_DOMAIN,
					     G82751_DOMAIN_LO, G82751_DOMAIN_HI,
					     &how);

	TEST_ASSERT_EQUAL_UINT64(30U, v);
	TEST_ASSERT_EQUAL_INT(STS_PTP_PICK_OPERATOR, how);
	TEST_ASSERT_FALSE(sts_ptp_pick_notable(how));
}

static void test_the_range_is_inclusive_at_both_ends(void)
{
	TEST_ASSERT_EQUAL_UINT64(24U, sts_ptp_prof_pick_u64_r(
					      24U, SCHEMA_DOMAIN, G82751_DOMAIN,
					      G82751_DOMAIN_LO,
					      G82751_DOMAIN_HI, NULL));
	TEST_ASSERT_EQUAL_UINT64(43U, sts_ptp_prof_pick_u64_r(
					      43U, SCHEMA_DOMAIN, G82751_DOMAIN,
					      G82751_DOMAIN_LO,
					      G82751_DOMAIN_HI, NULL));
	/* One past each end goes to the profile. */
	TEST_ASSERT_EQUAL_UINT64(G82751_DOMAIN,
				 sts_ptp_prof_pick_u64_r(23U, SCHEMA_DOMAIN,
							 G82751_DOMAIN,
							 G82751_DOMAIN_LO,
							 G82751_DOMAIN_HI,
							 NULL));
	TEST_ASSERT_EQUAL_UINT64(G82751_DOMAIN,
				 sts_ptp_prof_pick_u64_r(44U, SCHEMA_DOMAIN,
							 G82751_DOMAIN,
							 G82751_DOMAIN_LO,
							 G82751_DOMAIN_HI,
							 NULL));
}

static void test_a_single_value_range_refuses_everything_else(void)
{
	sts_ptp_pick_t how = STS_PTP_PICK_OPERATOR;
	/* log_announce: schema default 1, profile demands exactly -3. A unit
	 * commissioned under the Default profile, where the interval is free,
	 * hits this the moment somebody selects G.8275.1. */
	int32_t v = sts_ptp_prof_pick_i32_r(0, SCHEMA_LOG_ANNOUNCE,
					    G82751_LOG_ANNOUNCE,
					    G82751_LOG_ANNOUNCE,
					    G82751_LOG_ANNOUNCE, &how);

	TEST_ASSERT_EQUAL_INT32(G82751_LOG_ANNOUNCE, v);
	TEST_ASSERT_EQUAL_INT(STS_PTP_PICK_REFUSED, how);

	/* The one permitted value is accepted as the operator's. */
	v = sts_ptp_prof_pick_i32_r(G82751_LOG_ANNOUNCE, SCHEMA_LOG_ANNOUNCE,
				    G82751_LOG_ANNOUNCE, G82751_LOG_ANNOUNCE,
				    G82751_LOG_ANNOUNCE, &how);
	TEST_ASSERT_EQUAL_INT32(G82751_LOG_ANNOUNCE, v);
	TEST_ASSERT_EQUAL_INT(STS_PTP_PICK_OPERATOR, how);
}

static void test_an_untouched_key_is_never_reported_as_refused(void)
{
	sts_ptp_pick_t how = STS_PTP_PICK_REFUSED;

	/* Schema default 0 is outside G.8275.1's 24..43, but the operator never
	 * set it — reporting that as a refusal would put a warning in every
	 * boot log of every correctly-configured unit. */
	(void)sts_ptp_prof_pick_u64_r(SCHEMA_DOMAIN, SCHEMA_DOMAIN,
				      G82751_DOMAIN, G82751_DOMAIN_LO,
				      G82751_DOMAIN_HI, &how);
	TEST_ASSERT_EQUAL_INT(STS_PTP_PICK_PROFILE, how);
	TEST_ASSERT_FALSE(sts_ptp_pick_notable(how));
}

static void test_the_substituted_value_always_satisfies_the_profile(void)
{
	/*
	 * The property that makes this safe: whatever branch is taken, the
	 * result is inside the profile's range, so ptp_cfg_validate() accepts it
	 * by construction and this can never mask an invalid configuration.
	 */
	uint64_t probes[] = { 0U, 1U, 23U, 24U, 33U, 43U, 44U, 255U, UINT64_MAX };
	size_t i;

	for (i = 0U; i < (sizeof(probes) / sizeof(probes[0])); i++) {
		uint64_t v = sts_ptp_prof_pick_u64_r(probes[i], SCHEMA_DOMAIN,
						     G82751_DOMAIN,
						     G82751_DOMAIN_LO,
						     G82751_DOMAIN_HI, NULL);

		TEST_ASSERT_TRUE_MESSAGE(v >= G82751_DOMAIN_LO &&
						 v <= G82751_DOMAIN_HI,
					 "pick returned a value the profile "
					 "forbids");
	}
}

static void test_a_null_how_is_accepted(void)
{
	TEST_ASSERT_EQUAL_UINT64(G82751_DOMAIN,
				 sts_ptp_prof_pick_u64_r(5U, SCHEMA_DOMAIN,
							 G82751_DOMAIN,
							 G82751_DOMAIN_LO,
							 G82751_DOMAIN_HI,
							 NULL));
	TEST_ASSERT_EQUAL_INT32(G82751_LOG_SYNC,
				sts_ptp_prof_pick_i32_r(2, SCHEMA_LOG_SYNC,
							G82751_LOG_SYNC,
							G82751_LOG_SYNC,
							G82751_LOG_SYNC, NULL));
}

/* --- transport: a mask, not a range --------------------------------- */

/* PTP_TRANSPORT_UDP_IPV4 = 0, UDP_IPV6 = 1, L2 = 2 (core/ptp/ptp.h). */
#define XPORT_L2 2U
#define XPORT_UDP4 0U
#define MASK_L2_ONLY (1U << XPORT_L2)

static void test_a_transport_the_profile_forbids_is_refused(void)
{
	sts_ptp_pick_t how = STS_PTP_PICK_OPERATOR;
	/* G.8275.1 is L2-only; an operator carrying UDPv6 from the Default
	 * profile must not silently demote the unit. */
	uint64_t v = sts_ptp_prof_pick_xport(1U, XPORT_UDP4, XPORT_L2,
					     MASK_L2_ONLY, &how);

	TEST_ASSERT_EQUAL_UINT64(XPORT_L2, v);
	TEST_ASSERT_EQUAL_INT(STS_PTP_PICK_REFUSED, how);
}

static void test_a_permitted_transport_is_kept(void)
{
	sts_ptp_pick_t how = STS_PTP_PICK_REFUSED;
	uint64_t v = sts_ptp_prof_pick_xport(XPORT_L2, XPORT_UDP4, XPORT_L2,
					     MASK_L2_ONLY | (1U << XPORT_UDP4),
					     &how);

	TEST_ASSERT_EQUAL_UINT64(XPORT_L2, v);
	TEST_ASSERT_EQUAL_INT(STS_PTP_PICK_OPERATOR, how);
}

static void test_a_transport_index_past_the_mask_cannot_shift_out_of_range(void)
{
	sts_ptp_pick_t how = STS_PTP_PICK_OPERATOR;

	/* 32 and above would be undefined behaviour as a shift count. The width
	 * test has to come first, and this is what says so. */
	TEST_ASSERT_EQUAL_UINT64(XPORT_L2,
				 sts_ptp_prof_pick_xport(32U, XPORT_UDP4,
							 XPORT_L2, MASK_L2_ONLY,
							 &how));
	TEST_ASSERT_EQUAL_INT(STS_PTP_PICK_REFUSED, how);
	TEST_ASSERT_EQUAL_UINT64(XPORT_L2,
				 sts_ptp_prof_pick_xport(UINT64_MAX, XPORT_UDP4,
							 XPORT_L2, MASK_L2_ONLY,
							 NULL));
}

static void test_an_out_of_range_transport_does_not_alias_onto_a_valid_one(void)
{
	sts_ptp_pick_t how = STS_PTP_PICK_OPERATOR;
	/*
	 * The case that distinguishes a width GUARD from a width MASK. With the
	 * mask permitting UDPv4 (bit 0), a stored 32 masked to 5 bits becomes
	 * bit 0 and would be accepted as a valid transport — 32 is not a
	 * transport at all. `stored & 31` is a plausible-looking way to make the
	 * shift defined and it is wrong; refusing outright is right.
	 *
	 * Written after a mutation that replaced the guard with `& 31U`
	 * survived every other test in this file.
	 */
	uint64_t v = sts_ptp_prof_pick_xport(32U, XPORT_L2, XPORT_L2,
					     (1U << XPORT_UDP4), &how);

	TEST_ASSERT_EQUAL_INT(STS_PTP_PICK_REFUSED, how);
	TEST_ASSERT_EQUAL_UINT64(XPORT_L2, v);

	/* 64 aliases to bit 0 as well; 33 would alias to bit 1. */
	how = STS_PTP_PICK_OPERATOR;
	(void)sts_ptp_prof_pick_xport(64U, XPORT_L2, XPORT_L2,
				      (1U << XPORT_UDP4), &how);
	TEST_ASSERT_EQUAL_INT(STS_PTP_PICK_REFUSED, how);
}

static void test_an_empty_transport_mask_refuses_every_override(void)
{
	TEST_ASSERT_EQUAL_UINT64(XPORT_L2,
				 sts_ptp_prof_pick_xport(1U, XPORT_UDP4,
							 XPORT_L2, 0U, NULL));
}

static void test_the_commissioned_g82751_unit_now_stays_on_g82751(void)
{
	/*
	 * End to end on the scenario that motivated all of this. A unit
	 * commissioned under Default with priority1 128, log_announce 0 and
	 * UDPv6, switched to G.8275.1. Under the old code every one of those
	 * survived the pick, ptp_cfg_validate() then returned -ERANGE, and the
	 * unit came up on the Default profile. Now each offending field loses
	 * individually and the profile stands.
	 */
	int32_t ann = sts_ptp_prof_pick_i32_r(0, SCHEMA_LOG_ANNOUNCE,
					      G82751_LOG_ANNOUNCE,
					      G82751_LOG_ANNOUNCE,
					      G82751_LOG_ANNOUNCE, NULL);
	uint64_t xport = sts_ptp_prof_pick_xport(1U, XPORT_UDP4, XPORT_L2,
						 MASK_L2_ONLY, NULL);
	uint64_t dom = sts_ptp_prof_pick_u64_r(SCHEMA_DOMAIN, SCHEMA_DOMAIN,
					       G82751_DOMAIN, G82751_DOMAIN_LO,
					       G82751_DOMAIN_HI, NULL);

	TEST_ASSERT_EQUAL_INT32(G82751_LOG_ANNOUNCE, ann);
	TEST_ASSERT_EQUAL_UINT64(XPORT_L2, xport);
	TEST_ASSERT_EQUAL_UINT64(G82751_DOMAIN, dom);
}

/* ===================================================================== *
 *  The Default profile is not range-constrained
 *
 *  This block exists because the range-refusal shipped as a regression and
 *  no test in this file caught it: sts_ptp_prof_pick_i32_r() was correct in
 *  isolation, and the defect was in WHICH range sts_ptp.c handed it. These
 *  tests pin the numbers from both sides of that decision, so a future edit
 *  that reintroduces the descriptor's range for Default fails here.
 *
 *  ptp_port.c's profile_range_check() returns 0 immediately for
 *  PTP_PROFILE_DEFAULT — Annex I.3's intervals are recommendations, and
 *  enforcing them "would reject configurations the standard allows". The
 *  schema is deliberately wider than desc_default for exactly that reason:
 *
 *      key                 schema (cfg_schema.h)   desc_default
 *      ptp.log.sync              -7 .. 1             -1 .. 1
 *      ptp.log.announce          -3 .. 4              0 .. 4
 *      ptp.log.delayreq          -7 .. 5              0 .. 5
 * ===================================================================== */

#define SCHEMA_LOG_SYNC_MIN (-7)
#define SCHEMA_LOG_SYNC_MAX 1
#define DESC_DEFAULT_LOG_SYNC_MIN (-1)
#define DESC_DEFAULT_LOG_SYNC_MAX 1
#define DESC_DEFAULT_LOG_SYNC 0

static void test_default_takes_the_schema_range_not_the_descriptors(void)
{
	int32_t lo = 999, hi = 999;

	sts_ptp_prof_range_i32(PROFILE_DEFAULT, PROFILE_DEFAULT,
			       DESC_DEFAULT_LOG_SYNC_MIN,
			       DESC_DEFAULT_LOG_SYNC_MAX, SCHEMA_LOG_SYNC_MIN,
			       SCHEMA_LOG_SYNC_MAX, &lo, &hi);

	TEST_ASSERT_EQUAL_INT32_MESSAGE(SCHEMA_LOG_SYNC_MIN, lo,
					"Default must be held to the schema, "
					"which cfg already enforces");
	TEST_ASSERT_EQUAL_INT32(SCHEMA_LOG_SYNC_MAX, hi);
}

static void test_a_named_profile_takes_the_descriptor_range(void)
{
	int32_t lo = 999, hi = 999;

	sts_ptp_prof_range_i32(PROFILE_G82751, PROFILE_DEFAULT,
			       G82751_LOG_SYNC, G82751_LOG_SYNC,
			       SCHEMA_LOG_SYNC_MIN, SCHEMA_LOG_SYNC_MAX, &lo,
			       &hi);

	TEST_ASSERT_EQUAL_INT32(G82751_LOG_SYNC, lo);
	TEST_ASSERT_EQUAL_INT32(G82751_LOG_SYNC, hi);
}

static void test_the_range_choice_composes_with_the_pick(void)
{
	/*
	 * The two halves together, which is the path sts_ptp.c actually takes
	 * and the one the earlier tests could not reach: choosing the range and
	 * then applying it. Under Default, 8 Sync/s survives; under G.8275.1 the
	 * same value is refused for the profile's own.
	 */
	int32_t lo, hi, v;

	sts_ptp_prof_range_i32(PROFILE_DEFAULT, PROFILE_DEFAULT,
			       DESC_DEFAULT_LOG_SYNC_MIN,
			       DESC_DEFAULT_LOG_SYNC_MAX, SCHEMA_LOG_SYNC_MIN,
			       SCHEMA_LOG_SYNC_MAX, &lo, &hi);
	v = sts_ptp_prof_pick_i32_r(-3, SCHEMA_LOG_SYNC, DESC_DEFAULT_LOG_SYNC,
				    lo, hi, NULL);
	TEST_ASSERT_EQUAL_INT32_MESSAGE(-3, v,
					"Default: the operator's 8 Sync/s must "
					"survive");

	sts_ptp_prof_range_i32(PROFILE_G82751, PROFILE_DEFAULT, G82751_LOG_SYNC,
			       G82751_LOG_SYNC, SCHEMA_LOG_SYNC_MIN,
			       SCHEMA_LOG_SYNC_MAX, &lo, &hi);
	v = sts_ptp_prof_pick_i32_r(-3, SCHEMA_LOG_SYNC, G82751_LOG_SYNC, lo,
				    hi, NULL);
	TEST_ASSERT_EQUAL_INT32_MESSAGE(G82751_LOG_SYNC, v,
					"G.8275.1: -4 is normative and -3 is "
					"not it");
}

static void test_the_unsigned_range_choice_matches(void)
{
	uint64_t lo = 999U, hi = 999U;

	sts_ptp_prof_range_u64(PROFILE_DEFAULT, PROFILE_DEFAULT, 24U, 43U, 0U,
			       255U, &lo, &hi);
	TEST_ASSERT_EQUAL_UINT64(0U, lo);
	TEST_ASSERT_EQUAL_UINT64(255U, hi);

	sts_ptp_prof_range_u64(PROFILE_G82751, PROFILE_DEFAULT, 24U, 43U, 0U,
			       255U, &lo, &hi);
	TEST_ASSERT_EQUAL_UINT64(24U, lo);
	TEST_ASSERT_EQUAL_UINT64(43U, hi);
}

static void test_only_a_named_profile_owns_the_transport(void)
{
	unsigned int i;

	TEST_ASSERT_FALSE_MESSAGE(
		sts_ptp_prof_owns_transport(PROFILE_DEFAULT, PROFILE_DEFAULT),
		"Default must not move a unit off its stored transport");
	for (i = 1U; i < PROFILE_COUNT; i++) {
		TEST_ASSERT_TRUE(sts_ptp_prof_owns_transport(
			(uint8_t)i, PROFILE_DEFAULT));
	}
}

static void test_default_profile_keeps_a_fast_sync_the_schema_allows(void)
{
	sts_ptp_pick_t how = STS_PTP_PICK_REFUSED;
	/*
	 * 8 Sync/s. Legal per the schema, stored, and honoured by every build
	 * before profiles were applied. Under the SCHEMA's range it must survive
	 * as the operator's value.
	 */
	int32_t v = sts_ptp_prof_pick_i32_r(-3, SCHEMA_LOG_SYNC,
					    DESC_DEFAULT_LOG_SYNC,
					    SCHEMA_LOG_SYNC_MIN,
					    SCHEMA_LOG_SYNC_MAX, &how);

	TEST_ASSERT_EQUAL_INT32(-3, v);
	TEST_ASSERT_EQUAL_INT(STS_PTP_PICK_OPERATOR, how);
}

static void test_the_descriptor_range_would_have_broken_it(void)
{
	sts_ptp_pick_t how = STS_PTP_PICK_OPERATOR;
	/*
	 * The regression, pinned as the thing NOT to do. Handing the same value
	 * desc_default's -1..1 refuses it and substitutes 0 — 1 Sync/s where the
	 * operator asked for 8. If someone reverts sts_ptp.c to pass the
	 * descriptor range for Default, the test above goes red and this one
	 * documents why.
	 */
	int32_t v = sts_ptp_prof_pick_i32_r(-3, SCHEMA_LOG_SYNC,
					    DESC_DEFAULT_LOG_SYNC,
					    DESC_DEFAULT_LOG_SYNC_MIN,
					    DESC_DEFAULT_LOG_SYNC_MAX, &how);

	TEST_ASSERT_EQUAL_INT32(DESC_DEFAULT_LOG_SYNC, v);
	TEST_ASSERT_EQUAL_INT(STS_PTP_PICK_REFUSED, how);
	TEST_ASSERT_TRUE_MESSAGE(
		SCHEMA_LOG_SYNC_MIN < DESC_DEFAULT_LOG_SYNC_MIN,
		"the schema must stay wider than desc_default here, or the "
		"Default-profile exemption has nothing to protect");
}

static void test_a_schema_range_can_never_refuse(void)
{
	/*
	 * The property that makes the Default path a no-op rather than a second
	 * gate: cfg_set() already refuses anything outside the schema bounds, so
	 * every value that can reach the pick is inside them.
	 */
	int32_t probes[] = { SCHEMA_LOG_SYNC_MIN, -3, -1, 0, SCHEMA_LOG_SYNC_MAX };
	size_t i;

	for (i = 0U; i < (sizeof(probes) / sizeof(probes[0])); i++) {
		sts_ptp_pick_t how = STS_PTP_PICK_REFUSED;

		(void)sts_ptp_prof_pick_i32_r(probes[i], SCHEMA_LOG_SYNC,
					      DESC_DEFAULT_LOG_SYNC,
					      SCHEMA_LOG_SYNC_MIN,
					      SCHEMA_LOG_SYNC_MAX, &how);
		TEST_ASSERT_NOT_EQUAL_MESSAGE(
			STS_PTP_PICK_REFUSED, how,
			"a value the schema accepts must never be refused "
			"under the Default profile");
	}
}

static void test_a_telecom_profile_still_gets_its_normative_range(void)
{
	/* The exemption is Default-only: G.8275.1's -4..-4 still binds. */
	sts_ptp_pick_t how = STS_PTP_PICK_OPERATOR;
	int32_t v = sts_ptp_prof_pick_i32_r(-3, SCHEMA_LOG_SYNC,
					    G82751_LOG_SYNC, G82751_LOG_SYNC,
					    G82751_LOG_SYNC, &how);

	TEST_ASSERT_EQUAL_INT32(G82751_LOG_SYNC, v);
	TEST_ASSERT_EQUAL_INT(STS_PTP_PICK_REFUSED, how);
}

static void test_the_default_transport_is_the_stored_one(void)
{
	/*
	 * ptp.transport's schema default is 1 (UDPv6); desc_default's is 0
	 * (UDPv4). Under the plain rule an untouched key resolves to the
	 * profile's UDPv4 and every client on ff0e::181 loses the grandmaster,
	 * silently — STS_PTP_PICK_PROFILE is not notable. sts_ptp.c therefore
	 * bypasses the pick entirely for Default; this pins the collision that
	 * makes the bypass necessary, so narrowing the schema default later
	 * makes the reason visible rather than mysterious.
	 */
	const uint64_t schema_default_transport = 1U;  /* UDPv6, cfg_schema.h */
	const uint64_t desc_default_transport = XPORT_UDP4; /* ptp_profile.c */
	sts_ptp_pick_t how = STS_PTP_PICK_OPERATOR;
	uint64_t v;

	TEST_ASSERT_TRUE_MESSAGE(
		schema_default_transport != desc_default_transport,
		"if these ever agree, the Default-transport bypass in "
		"sts_ptp.c is dead code and should go");

	/* What the generic rule WOULD do, which is why sts_ptp.c bypasses it:
	 * an untouched key silently becomes the profile's UDPv4, reported as
	 * PICK_PROFILE, which sts_ptp_pick_notable() does not log. */
	v = sts_ptp_prof_pick_xport(schema_default_transport,
				    schema_default_transport,
				    desc_default_transport,
				    (1U << XPORT_UDP4) | (1U << 1) |
					    (1U << XPORT_L2),
				    &how);
	TEST_ASSERT_EQUAL_UINT64(desc_default_transport, v);
	TEST_ASSERT_EQUAL_INT(STS_PTP_PICK_PROFILE, how);
	TEST_ASSERT_FALSE_MESSAGE(
		sts_ptp_pick_notable(how),
		"the flip was silent, which is what made it dangerous");
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

	RUN_TEST(test_an_out_of_profile_domain_is_refused_not_fatal);
	RUN_TEST(test_an_in_profile_domain_is_still_the_operators);
	RUN_TEST(test_the_range_is_inclusive_at_both_ends);
	RUN_TEST(test_a_single_value_range_refuses_everything_else);
	RUN_TEST(test_an_untouched_key_is_never_reported_as_refused);
	RUN_TEST(test_the_substituted_value_always_satisfies_the_profile);
	RUN_TEST(test_a_null_how_is_accepted);

	RUN_TEST(test_a_transport_the_profile_forbids_is_refused);
	RUN_TEST(test_a_permitted_transport_is_kept);
	RUN_TEST(test_a_transport_index_past_the_mask_cannot_shift_out_of_range);
	RUN_TEST(test_an_out_of_range_transport_does_not_alias_onto_a_valid_one);
	RUN_TEST(test_an_empty_transport_mask_refuses_every_override);
	RUN_TEST(test_the_commissioned_g82751_unit_now_stays_on_g82751);

	RUN_TEST(test_default_takes_the_schema_range_not_the_descriptors);
	RUN_TEST(test_a_named_profile_takes_the_descriptor_range);
	RUN_TEST(test_the_range_choice_composes_with_the_pick);
	RUN_TEST(test_the_unsigned_range_choice_matches);
	RUN_TEST(test_only_a_named_profile_owns_the_transport);
	RUN_TEST(test_default_profile_keeps_a_fast_sync_the_schema_allows);
	RUN_TEST(test_the_descriptor_range_would_have_broken_it);
	RUN_TEST(test_a_schema_range_can_never_refuse);
	RUN_TEST(test_a_telecom_profile_still_gets_its_normative_range);
	RUN_TEST(test_the_default_transport_is_the_stored_one);

	return UNITY_END();
}
