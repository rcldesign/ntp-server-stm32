/*
 * STS1000 "Meridian" — the operator-recovery surface's three decisions.
 *
 * sts_recovery_policy.h holds them because each is silent when it is wrong, and
 * all three sit on paths a technician reaches only when something has already
 * gone wrong on the board. This suite is written against the failure modes:
 *
 *   1. THE CLEAR SCOPE IS NOT COPIED FROM sts_alarm_set()'S. That function
 *      refuses scanned-signal ids because the 1 kHz scan owns them; clearing
 *      one touches only the latch and must be allowed. And "nothing was
 *      latched" is not a failure while "still active" is — collapsing the two
 *      either makes an idempotent command fail, or reports a live fault as
 *      acknowledged.
 *
 *   2. A TEMPCO FIT IS REFUSED RATHER THAN PRINTED WHEN THE DATA CANNOT SUPPORT
 *      IT. Ordinary least squares returns a slope for any two points, with
 *      R^2 = 1.0 by construction, and that slope goes into cal.tempco, which
 *      the discipline loop applies as FEED-FORWARD. A wrong value does not read
 *      wrong anywhere; it steers the oscillator whenever the box warms up. The
 *      real disc_tempco_fit() is driven here rather than a model, because the
 *      acceptance rule is only as good as the numbers it is judging.
 *
 *   3. AN MP OBJECT ID NAMES THE ACTION IT IS SUPPOSED TO. One string compare
 *      separates a board cold-cycle from an LED. The router is asserted against
 *      the REAL manifest, guard classes included, so a renamed object or a
 *      downgraded guard breaks this suite rather than the field: FMT §5.2 puts
 *      POE_KILL in G3 and mp_ovr_guard() maps G2/G3 to an admin role floor.
 */

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "unity.h"

#include "disc/disc.h"
#include "fault/fault.h"
#include "mp/mp_manifest.h"
#include "mp/mp_override.h"

#include "zephyr/console/sts_recovery_policy.h"

/* ===================================================================== */
/* 1. clearing a latched alarm                                           */
/* ===================================================================== */

/**
 * The policy header states the id space as a plain number so it is a claim
 * about core/fault rather than a restatement of it. If FAULT_ALARM_COUNT or
 * FAULT_SIG_COUNT moves, this is what says so.
 */
static void test_the_alarm_id_space_matches_core_fault(void)
{
	TEST_ASSERT_EQUAL_UINT((unsigned int)FAULT_ALARM_COUNT,
			       STS_ALARM_ID_COUNT);
	TEST_ASSERT_EQUAL_UINT((unsigned int)FAULT_SIG_COUNT,
			       STS_ALARM_FIRST_SOFTWARE);
}

/**
 * The asymmetry with sts_alarm_set(): a scanned-signal latch IS clearable.
 *
 * sts_app.c refuses ids 0..31 for sts_alarm_set() because the scan owns the
 * signal's active state. The latch is a historical record the scan never
 * writes, so refusing here too would leave a technician unable to acknowledge a
 * power-good glitch that has long since gone away.
 */
static void test_a_scanned_signal_latch_may_be_cleared(void)
{
	for (unsigned int id = 0U; id < STS_ALARM_FIRST_SOFTWARE; id++) {
		TEST_ASSERT_EQUAL_INT(STS_ACLR_TARGET_ONE,
				      sts_aclr_target(false, id));
	}
}

static void test_a_software_alarm_id_may_be_cleared(void)
{
	for (unsigned int id = STS_ALARM_FIRST_SOFTWARE;
	     id < STS_ALARM_ID_COUNT; id++) {
		TEST_ASSERT_EQUAL_INT(STS_ACLR_TARGET_ONE,
				      sts_aclr_target(false, id));
	}
}

static void test_an_id_past_the_table_is_refused(void)
{
	TEST_ASSERT_EQUAL_INT(STS_ACLR_TARGET_BAD,
			      sts_aclr_target(false, STS_ALARM_ID_COUNT));
	TEST_ASSERT_EQUAL_INT(STS_ACLR_TARGET_BAD,
			      sts_aclr_target(false, 64U));
	/* A 64-bit token must not be truncated into range. */
	TEST_ASSERT_EQUAL_INT(STS_ACLR_TARGET_BAD,
			      sts_aclr_target(false, 0x100000000ULL));
}

/** `all` wins over whatever id came with it, and is never out of range. */
static void test_all_is_accepted_whatever_the_id_says(void)
{
	TEST_ASSERT_EQUAL_INT(STS_ACLR_TARGET_ALL, sts_aclr_target(true, 0U));
	TEST_ASSERT_EQUAL_INT(STS_ACLR_TARGET_ALL,
			      sts_aclr_target(true, 999999U));
}

/**
 * -ENOENT is a success and -EBUSY is not.
 *
 * Reporting "not latched" as a failure breaks a script that clears twice;
 * reporting "still active" as a success tells an operator a live fault has been
 * dealt with. Both readings are one `case` away from each other.
 */
static void test_not_latched_is_success_and_still_active_is_not(void)
{
	TEST_ASSERT_EQUAL_INT(STS_ACLR_CLEARED, sts_aclr_outcome(0));
	TEST_ASSERT_EQUAL_INT(STS_ACLR_NOT_LATCHED, sts_aclr_outcome(-ENOENT));
	TEST_ASSERT_EQUAL_INT(STS_ACLR_STILL_ACTIVE, sts_aclr_outcome(-EBUSY));
	TEST_ASSERT_EQUAL_INT(STS_ACLR_REJECTED, sts_aclr_outcome(-EINVAL));

	TEST_ASSERT_FALSE(sts_aclr_is_failure(STS_ACLR_CLEARED));
	TEST_ASSERT_FALSE_MESSAGE(sts_aclr_is_failure(STS_ACLR_NOT_LATCHED),
				  "clearing an unlatched alarm is idempotent, "
				  "not an error");
	TEST_ASSERT_TRUE_MESSAGE(sts_aclr_is_failure(STS_ACLR_STILL_ACTIVE),
				 "a live fault was reported as cleared");
	TEST_ASSERT_TRUE(sts_aclr_is_failure(STS_ACLR_REJECTED));
}

/** Anything core/fault might grow later is a refusal, not a silent success. */
static void test_an_unknown_errno_is_a_rejection(void)
{
	TEST_ASSERT_EQUAL_INT(STS_ACLR_REJECTED, sts_aclr_outcome(-EPERM));
	TEST_ASSERT_EQUAL_INT(STS_ACLR_REJECTED, sts_aclr_outcome(-EIO));
	TEST_ASSERT_TRUE(sts_aclr_is_failure(sts_aclr_outcome(-EIO)));
}

static void test_every_clear_outcome_has_a_distinct_name(void)
{
	static const sts_aclr_outcome_t all[] = {
		STS_ACLR_CLEARED, STS_ACLR_NOT_LATCHED, STS_ACLR_STILL_ACTIVE,
		STS_ACLR_REJECTED,
	};

	for (size_t i = 0U; i < (sizeof(all) / sizeof(all[0])); i++) {
		TEST_ASSERT_NOT_NULL(sts_aclr_outcome_name(all[i]));
		for (size_t j = i + 1U; j < (sizeof(all) / sizeof(all[0])); j++) {
			TEST_ASSERT_TRUE(strcmp(sts_aclr_outcome_name(all[i]),
						sts_aclr_outcome_name(all[j])) !=
					 0);
		}
	}
	TEST_ASSERT_EQUAL_STRING("?", sts_aclr_outcome_name(
					      (sts_aclr_outcome_t)99));
}

/**
 * core/fault really does answer the way the outcome map assumes.
 *
 * The map is a translation, and a translation is only correct against the thing
 * it translates. Driving the real fault context is what keeps this from
 * agreeing with a remembered contract.
 */
static void test_core_fault_produces_the_three_outcomes(void)
{
	fault_ctx_t ctx;

	TEST_ASSERT_EQUAL_INT(0, fault_init(&ctx, NULL));

	/* Nothing latched yet. */
	TEST_ASSERT_EQUAL_INT(STS_ACLR_NOT_LATCHED,
			      sts_aclr_outcome(fault_alarm_clear(
				      &ctx, FAULT_ALARM_RB_OV)));

	/* Raise it: now latched AND active, so the clear must be refused. */
	TEST_ASSERT_EQUAL_INT(0, fault_alarm_set(&ctx, FAULT_ALARM_RB_OV, true,
						 1000U));
	TEST_ASSERT_EQUAL_INT(STS_ACLR_STILL_ACTIVE,
			      sts_aclr_outcome(fault_alarm_clear(
				      &ctx, FAULT_ALARM_RB_OV)));

	/* Condition gone, latch remains: now it clears. */
	TEST_ASSERT_EQUAL_INT(0, fault_alarm_set(&ctx, FAULT_ALARM_RB_OV, false,
						 2000U));
	TEST_ASSERT_EQUAL_INT(STS_ACLR_CLEARED,
			      sts_aclr_outcome(fault_alarm_clear(
				      &ctx, FAULT_ALARM_RB_OV)));

	/* Out of range is the rejection branch. */
	TEST_ASSERT_EQUAL_INT(STS_ACLR_REJECTED,
			      sts_aclr_outcome(fault_alarm_clear(
				      &ctx, (fault_alarm_id_t)
						    FAULT_ALARM_COUNT)));
}

/* ===================================================================== */
/* 2. accepting an OCXO tempco fit                                        */
/* ===================================================================== */

/** Build a clean linear sweep: y = slope*t + offset over [t0, t0+span]. */
static size_t make_sweep(float *t, float *y, size_t n, float t0, float span,
			 float slope, float offset)
{
	for (size_t i = 0U; i < n; i++) {
		t[i] = t0 + ((span * (float)i) / (float)(n - 1U));
		y[i] = (slope * t[i]) + offset;
	}
	return n;
}

static void test_the_span_helper_is_max_minus_min(void)
{
	float t[5] = { 10.0f, -5.0f, 40.0f, 0.0f, 12.5f };

	TEST_ASSERT_FLOAT_WITHIN(1e-4f, 45.0f, sts_tempco_span(t, 5U));
	/* Fewer than two points has no span, and must not read uninitialised
	 * memory to say so. */
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, sts_tempco_span(t, 1U));
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, sts_tempco_span(t, 0U));
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, sts_tempco_span(NULL, 5U));
}

/** A real sweep through the real fit is accepted, and the slope is recovered. */
static void test_a_clean_sweep_is_accepted(void)
{
	float t[12];
	float y[12];
	float slope = 0.0f;
	float offset = 0.0f;
	float r2 = 0.0f;
	size_t n = make_sweep(t, y, 12U, -10.0f, 60.0f, 0.42f, -3.0f);

	TEST_ASSERT_EQUAL_INT(0, disc_tempco_fit(t, y, n, &slope, &offset, &r2));
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.42f, slope);
	TEST_ASSERT_FLOAT_WITHIN(1e-2f, -3.0f, offset);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, r2);

	TEST_ASSERT_EQUAL_INT(STS_TEMPCO_OK,
			      sts_tempco_check(n, sts_tempco_span(t, n), r2,
					       slope));
}

/**
 * Two points fit perfectly and mean nothing.
 *
 * This is the case the sample floor exists for and the R^2 floor cannot catch:
 * a two-point least squares has R^2 = 1.0 by construction, so scatter says the
 * data is flawless however wrong it is.
 */
static void test_two_points_are_refused_despite_a_perfect_r2(void)
{
	float t[2] = { 0.0f, 50.0f };
	float y[2] = { 0.0f, 21.0f };
	float slope = 0.0f;
	float r2 = 0.0f;

	TEST_ASSERT_EQUAL_INT(0, disc_tempco_fit(t, y, 2U, &slope, NULL, &r2));
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, r2);
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.42f, slope);

	TEST_ASSERT_EQUAL_INT_MESSAGE(
		STS_TEMPCO_ERR_SAMPLES,
		sts_tempco_check(2U, sts_tempco_span(t, 2U), r2, slope),
		"a two-point fit was accepted because R^2 was 1.0");
}

/** Enough points, but over a sweep too narrow to extrapolate from. */
static void test_a_narrow_sweep_is_refused(void)
{
	float t[10];
	float y[10];
	float slope = 0.0f;
	float r2 = 0.0f;
	size_t n = make_sweep(t, y, 10U, 20.0f, 4.0f, 0.5f, 0.0f);

	TEST_ASSERT_EQUAL_INT(0, disc_tempco_fit(t, y, n, &slope, NULL, &r2));
	TEST_ASSERT_EQUAL_INT(STS_TEMPCO_ERR_SPAN,
			      sts_tempco_check(n, sts_tempco_span(t, n), r2,
					       slope));

	/* Exactly at the floor is accepted: the bound is inclusive, and an
	 * off-by-one here silently rejects a legitimate sweep. */
	n = make_sweep(t, y, 10U, 20.0f, STS_TEMPCO_MIN_SPAN_C, 0.5f, 0.0f);
	TEST_ASSERT_EQUAL_INT(0, disc_tempco_fit(t, y, n, &slope, NULL, &r2));
	TEST_ASSERT_EQUAL_INT(STS_TEMPCO_OK,
			      sts_tempco_check(n, sts_tempco_span(t, n), r2,
					       slope));
}

/** Scatter that no line describes is refused, not averaged into one. */
static void test_a_noisy_data_set_is_refused(void)
{
	/* A wide, well-populated sweep whose y values are unrelated to t. */
	float t[8] = { 0.0f, 8.0f, 16.0f, 24.0f, 32.0f, 40.0f, 48.0f, 56.0f };
	float y[8] = { 5.0f, -4.0f, 6.0f, -5.0f, 4.0f, -6.0f, 5.0f, -4.0f };
	float slope = 0.0f;
	float r2 = 0.0f;

	TEST_ASSERT_EQUAL_INT(0, disc_tempco_fit(t, y, 8U, &slope, NULL, &r2));
	TEST_ASSERT_TRUE_MESSAGE(r2 < STS_TEMPCO_MIN_R2,
				 "the fixture is not actually noisy");
	TEST_ASSERT_EQUAL_INT(STS_TEMPCO_ERR_SCATTER,
			      sts_tempco_check(8U, sts_tempco_span(t, 8U), r2,
					       slope));
}

/**
 * A slope cal.tempco cannot hold is refused rather than offered.
 *
 * The schema bound on 0x0C0C is +-100 ppb/C, so printing a larger value as a
 * result to paste would be handing the operator something the very next command
 * rejects — and a clamp would be worse, because it would silently store a
 * different coefficient from the one measured.
 */
static void test_a_slope_outside_the_cfg_bound_is_refused(void)
{
	float t[8];
	float y[8];
	float slope = 0.0f;
	float r2 = 0.0f;
	size_t n = make_sweep(t, y, 8U, 0.0f, 40.0f, 250.0f, 0.0f);

	TEST_ASSERT_EQUAL_INT(0, disc_tempco_fit(t, y, n, &slope, NULL, &r2));
	TEST_ASSERT_EQUAL_INT(STS_TEMPCO_ERR_SLOPE,
			      sts_tempco_check(n, sts_tempco_span(t, n), r2,
					       slope));

	/* Symmetric: a large negative df/dT is just as unstorable. */
	n = make_sweep(t, y, 8U, 0.0f, 40.0f, -250.0f, 0.0f);
	TEST_ASSERT_EQUAL_INT(0, disc_tempco_fit(t, y, n, &slope, NULL, &r2));
	TEST_ASSERT_EQUAL_INT(STS_TEMPCO_ERR_SLOPE,
			      sts_tempco_check(n, sts_tempco_span(t, n), r2,
					       slope));

	/* Exactly at the bound is storable, so it is accepted. */
	n = make_sweep(t, y, 8U, 0.0f, 40.0f, STS_TEMPCO_SLOPE_ABS_MAX, 0.0f);
	TEST_ASSERT_EQUAL_INT(0, disc_tempco_fit(t, y, n, &slope, NULL, &r2));
	TEST_ASSERT_EQUAL_INT(STS_TEMPCO_OK,
			      sts_tempco_check(n, sts_tempco_span(t, n), r2,
					       slope));
}

/** A NaN R^2 refuses. Written as a negated >=, so this is a real branch. */
static void test_a_nan_r2_is_refused(void)
{
	TEST_ASSERT_EQUAL_INT(STS_TEMPCO_ERR_SCATTER,
			      sts_tempco_check(10U, 40.0f, NAN, 1.0f));
	TEST_ASSERT_EQUAL_INT(STS_TEMPCO_ERR_SPAN,
			      sts_tempco_check(10U, NAN, 1.0f, 1.0f));
	TEST_ASSERT_EQUAL_INT(STS_TEMPCO_ERR_SLOPE,
			      sts_tempco_check(10U, 40.0f, 1.0f, NAN));
}

/**
 * The refusal names the FIRST thing wrong, so an operator with four points is
 * not told about R^2 as well.
 */
static void test_the_first_defect_is_the_one_reported(void)
{
	/* Too few AND too narrow AND too noisy AND out of range. */
	TEST_ASSERT_EQUAL_INT(STS_TEMPCO_ERR_SAMPLES,
			      sts_tempco_check(2U, 0.5f, 0.1f, 5000.0f));
	/* Enough points; span is now the first defect. */
	TEST_ASSERT_EQUAL_INT(STS_TEMPCO_ERR_SPAN,
			      sts_tempco_check(STS_TEMPCO_MIN_SAMPLES, 0.5f,
					       0.1f, 5000.0f));
	/* Span fixed; scatter is next. */
	TEST_ASSERT_EQUAL_INT(STS_TEMPCO_ERR_SCATTER,
			      sts_tempco_check(STS_TEMPCO_MIN_SAMPLES, 40.0f,
					       0.1f, 5000.0f));
	/* Scatter fixed; only the slope is left. */
	TEST_ASSERT_EQUAL_INT(STS_TEMPCO_ERR_SLOPE,
			      sts_tempco_check(STS_TEMPCO_MIN_SAMPLES, 40.0f,
					       0.99f, 5000.0f));
}

static void test_the_sample_floor_is_below_the_buffer(void)
{
	/* A buffer that cannot hold a believable data set is a dead feature. */
	TEST_ASSERT_TRUE(STS_TEMPCO_MAX_SAMPLES >= STS_TEMPCO_MIN_SAMPLES);
	/* And the floor is above the two points that fit anything. */
	TEST_ASSERT_TRUE(STS_TEMPCO_MIN_SAMPLES > 2U);
}

static void test_every_tempco_verdict_has_a_distinct_name(void)
{
	static const sts_tempco_verdict_t all[] = {
		STS_TEMPCO_OK, STS_TEMPCO_ERR_SAMPLES, STS_TEMPCO_ERR_SPAN,
		STS_TEMPCO_ERR_SCATTER, STS_TEMPCO_ERR_SLOPE,
	};

	for (size_t i = 0U; i < (sizeof(all) / sizeof(all[0])); i++) {
		TEST_ASSERT_NOT_NULL(sts_tempco_verdict_name(all[i]));
		for (size_t j = i + 1U; j < (sizeof(all) / sizeof(all[0])); j++) {
			TEST_ASSERT_TRUE(
				strcmp(sts_tempco_verdict_name(all[i]),
				       sts_tempco_verdict_name(all[j])) != 0);
		}
	}
	TEST_ASSERT_EQUAL_STRING("?", sts_tempco_verdict_name(
					      (sts_tempco_verdict_t)99));
}

/* ===================================================================== */
/* 3. which recovery action an MP object names                            */
/* ===================================================================== */

static void test_the_pulse_router_names_the_right_pin(void)
{
	TEST_ASSERT_EQUAL_INT(STS_RECOV_POE_KILL,
			      sts_recov_pulse_action(STS_RECOV_ID_POE_KILL));
	TEST_ASSERT_EQUAL_INT(STS_RECOV_RB_OV_RESET,
			      sts_recov_pulse_action(STS_RECOV_ID_RB_OV_RESET));
}

/**
 * The catastrophic confusion, asserted in both directions.
 *
 * Clearing an over-voltage latch and cold-cycling the board are one string
 * compare apart, and the operator reaches for the first one precisely when the
 * second would be worst.
 */
static void test_the_ov_reset_never_reaches_poe_kill(void)
{
	TEST_ASSERT_NOT_EQUAL(STS_RECOV_POE_KILL,
			      sts_recov_pulse_action(STS_RECOV_ID_RB_OV_RESET));
	TEST_ASSERT_NOT_EQUAL(STS_RECOV_RB_OV_RESET,
			      sts_recov_pulse_action(STS_RECOV_ID_POE_KILL));
	TEST_ASSERT_TRUE(strcmp(STS_RECOV_ID_POE_KILL,
				STS_RECOV_ID_RB_OV_RESET) != 0);
}

/** Nothing else pulses into a recovery action, including NULL. */
static void test_an_unknown_pulse_id_does_nothing(void)
{
	TEST_ASSERT_EQUAL_INT(STS_RECOV_NONE, sts_recov_pulse_action(NULL));
	TEST_ASSERT_EQUAL_INT(STS_RECOV_NONE, sts_recov_pulse_action(""));
	TEST_ASSERT_EQUAL_INT(STS_RECOV_NONE,
			      sts_recov_pulse_action("sys.wdt.kick"));
	TEST_ASSERT_EQUAL_INT(STS_RECOV_NONE,
			      sts_recov_pulse_action("gnss.reset"));
	/* A prefix of a known id is not that id. */
	TEST_ASSERT_EQUAL_INT(STS_RECOV_NONE, sts_recov_pulse_action("pwr.poe"));
	TEST_ASSERT_EQUAL_INT(STS_RECOV_NONE,
			      sts_recov_pulse_action("pwr.poe.kill2"));
}

/**
 * The two routers are separate tables, and must stay separate.
 *
 * One table serving both RPCs would let `obj.set` drive `pwr.poe.kill` — an
 * object the manifest does not even mark writable — which is a cold cycle from
 * a request that was never meant to reach a pin.
 */
static void test_the_bool_router_cannot_reach_a_pulse_action(void)
{
	TEST_ASSERT_EQUAL_INT(STS_RECOV_IDENTIFY,
			      sts_recov_bool_action(STS_RECOV_ID_IDENTIFY));
	TEST_ASSERT_EQUAL_INT(STS_RECOV_NONE,
			      sts_recov_bool_action(STS_RECOV_ID_POE_KILL));
	TEST_ASSERT_EQUAL_INT(STS_RECOV_NONE,
			      sts_recov_bool_action(STS_RECOV_ID_RB_OV_RESET));
	TEST_ASSERT_EQUAL_INT(STS_RECOV_NONE, sts_recov_bool_action(NULL));
	TEST_ASSERT_EQUAL_INT(STS_RECOV_NONE,
			      sts_recov_pulse_action(STS_RECOV_ID_IDENTIFY));
}

/** Every id the router knows exists in the shipped manifest. */
static void test_every_routed_id_is_a_real_manifest_object(void)
{
	TEST_ASSERT_TRUE_MESSAGE(mp_obj_find(STS_RECOV_ID_POE_KILL) >= 0,
				 "pwr.poe.kill is not in the manifest");
	TEST_ASSERT_TRUE_MESSAGE(mp_obj_find(STS_RECOV_ID_RB_OV_RESET) >= 0,
				 "pwr.rb.ov.reset is not in the manifest");
	TEST_ASSERT_TRUE_MESSAGE(mp_obj_find(STS_RECOV_ID_IDENTIFY) >= 0,
				 "ui.identify is not in the manifest");
}

/**
 * FMT §5.2 puts POE_KILL in G3, and mp_ovr_guard() gives G2/G3 an ADMIN role
 * floor. Both halves are the reason the shell carries no POE_KILL command: this
 * is the only plane that can enforce them.
 */
static void test_poe_kill_is_g3_with_an_admin_floor(void)
{
	const mp_obj_t *o = mp_obj_at((size_t)mp_obj_find(STS_RECOV_ID_POE_KILL));

	TEST_ASSERT_NOT_NULL(o);
	TEST_ASSERT_EQUAL_INT_MESSAGE(
		MP_GUARD_G3, o->guard,
		"POE_KILL was downgraded below the FMT 5.2 class");
	TEST_ASSERT_TRUE_MESSAGE((o->flags & MP_OF_PULSE) != 0U,
				 "POE_KILL is no longer reachable by obj.pulse");
	/* Not writable: a `set` must not be able to cold-cycle the board. */
	TEST_ASSERT_TRUE_MESSAGE((o->flags & MP_OF_WRITE) == 0U,
				 "POE_KILL became writable by obj.set");
}

/** The OV reset is G2 — typed device serial, admin floor. */
static void test_the_ov_reset_is_g2_and_pulsable(void)
{
	const mp_obj_t *o =
		mp_obj_at((size_t)mp_obj_find(STS_RECOV_ID_RB_OV_RESET));

	TEST_ASSERT_NOT_NULL(o);
	TEST_ASSERT_EQUAL_INT(MP_GUARD_G2, o->guard);
	TEST_ASSERT_TRUE((o->flags & MP_OF_PULSE) != 0U);
}

/**
 * The locate beacon is G1 and overridable, which is what makes it a LEASE — the
 * dead-man reverts it, so a tool that walks away does not leave a board
 * blinking at nobody.
 */
static void test_identify_is_g1_and_leased(void)
{
	const mp_obj_t *o = mp_obj_at((size_t)mp_obj_find(STS_RECOV_ID_IDENTIFY));

	TEST_ASSERT_NOT_NULL(o);
	TEST_ASSERT_EQUAL_INT_MESSAGE(
		MP_GUARD_G1, o->guard,
		"ui.identify at G0 would let an unauthenticated session take a "
		"lease and suppress the fault colour");
	TEST_ASSERT_TRUE((o->flags & MP_OF_OVERRIDE) != 0U);
}

/**
 * The beacon's fail-safe outlives the longest lease that can ask for it.
 *
 * A release stops it immediately; this bound only covers a release that never
 * arrives. Shorter than MP_LEASE_TTL_MAX_MS and a long lease would stop
 * blinking while the object still read "on".
 */
static void test_the_identify_fail_safe_covers_the_longest_lease(void)
{
	TEST_ASSERT_TRUE(STS_RECOV_IDENTIFY_MS >= MP_LEASE_TTL_MAX_MS);
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_the_alarm_id_space_matches_core_fault);
	RUN_TEST(test_a_scanned_signal_latch_may_be_cleared);
	RUN_TEST(test_a_software_alarm_id_may_be_cleared);
	RUN_TEST(test_an_id_past_the_table_is_refused);
	RUN_TEST(test_all_is_accepted_whatever_the_id_says);
	RUN_TEST(test_not_latched_is_success_and_still_active_is_not);
	RUN_TEST(test_an_unknown_errno_is_a_rejection);
	RUN_TEST(test_every_clear_outcome_has_a_distinct_name);
	RUN_TEST(test_core_fault_produces_the_three_outcomes);

	RUN_TEST(test_the_span_helper_is_max_minus_min);
	RUN_TEST(test_a_clean_sweep_is_accepted);
	RUN_TEST(test_two_points_are_refused_despite_a_perfect_r2);
	RUN_TEST(test_a_narrow_sweep_is_refused);
	RUN_TEST(test_a_noisy_data_set_is_refused);
	RUN_TEST(test_a_slope_outside_the_cfg_bound_is_refused);
	RUN_TEST(test_a_nan_r2_is_refused);
	RUN_TEST(test_the_first_defect_is_the_one_reported);
	RUN_TEST(test_the_sample_floor_is_below_the_buffer);
	RUN_TEST(test_every_tempco_verdict_has_a_distinct_name);

	RUN_TEST(test_the_pulse_router_names_the_right_pin);
	RUN_TEST(test_the_ov_reset_never_reaches_poe_kill);
	RUN_TEST(test_an_unknown_pulse_id_does_nothing);
	RUN_TEST(test_the_bool_router_cannot_reach_a_pulse_action);
	RUN_TEST(test_every_routed_id_is_a_real_manifest_object);
	RUN_TEST(test_poe_kill_is_g3_with_an_admin_floor);
	RUN_TEST(test_the_ov_reset_is_g2_and_pulsable);
	RUN_TEST(test_identify_is_g1_and_leased);
	RUN_TEST(test_the_identify_fail_safe_covers_the_longest_lease);

	return UNITY_END();
}
