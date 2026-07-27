/*
 * STS1000 "Meridian" — core/cal unit tests.
 *
 * The property under test is refusal, not arithmetic. The ratio itself is three
 * lines borrowed from core/ina228 (already covered by test_ina228.c); what this
 * module exists for is deciding when NOT to write a calibration constant.
 *
 * Root CLAUDE.md on why that matters: the per-board SHUNT_CAL trim "silently
 * absorbs such an error and then lets it drift with temperature". A SHUNT_CAL
 * written from a mistyped reference current is well-formed, produces no error
 * flag anywhere, and rescales every current reading on that rail until someone
 * repeats the bench procedure. So every test below drives a plausible-looking
 * operator mistake at cal_ina_trim() and insists it is rejected with the reason
 * that names the mistake — a generic failure would send the operator to the
 * wrong bench.
 *
 * Currents are microamps throughout, matching sts_app.h's STS_CAL_INA_ARG().
 * The worked numbers use the PoE monitor (R30 25 mOhm, full scale 1.6384 A) and
 * the rubidium monitor (R159 7 mOhm, full scale 5.8514 A) from the interface
 * ref §4.2 table, so the ranges exercised are the board's real ones.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "cal/cal.h"
#include "ina228/ina228.h"

/* ---------------------------------------------------------------- helpers */

/* Full scale of two real monitors, from ina228_rail_tbl[]. */
#define FS_POE_UA    1638400u /* U10 0x40, R30 25 mOhm */
#define FS_VCC_RB_UA 5851400u /* U44 0x47, R159 7 mOhm */

static void in_default(cal_ina_in_t *in, uint32_t i_ref_ua, int32_t i_meas_ua)
{
	memset(in, 0, sizeof(*in));
	in->base_shunt_cal = (uint16_t)INA228_SHUNT_CAL_DEFAULT;
	in->i_ref_ua = i_ref_ua;
	in->i_meas_ua = i_meas_ua;
	in->meas_valid = true;
	in->meas_age_ms = 250u;
	in->max_age_ms = 2000u;
	in->fs_current_ua = FS_POE_UA;
	in->trim_min = (uint16_t)CAL_INA_TRIM_MIN;
	in->trim_max = (uint16_t)CAL_INA_TRIM_MAX;
}

static uint8_t run_expect(const cal_ina_in_t *in, int expect_rc,
			  cal_ina_out_t *out)
{
	TEST_ASSERT_EQUAL_INT(expect_rc, cal_ina_trim(in, out));
	return out->reason;
}

/* ------------------------------------------------------------ happy path */

/*
 * The whole point of the module in one case: a 1 % shunt reading 1 % high is
 * corrected by scaling SHUNT_CAL down 1 %.
 *
 * CURRENT is proportional to SHUNT_CAL, so a monitor reporting 1.0100 A for a
 * true 1.0000 A needs 4096 * 1.0000/1.0100 = 4055.4 -> 4055.
 */
static void test_one_percent_high_is_trimmed_down(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	in_default(&in, 1000000u, 1010000);
	TEST_ASSERT_EQUAL_INT(0, cal_ina_trim(&in, &out));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_OK, out.reason);
	TEST_ASSERT_EQUAL_UINT16(4055u, out.shunt_cal);
	TEST_ASSERT_EQUAL_INT32(10000, out.error_ppm); /* +1 % */
}

static void test_one_percent_low_is_trimmed_up(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	in_default(&in, 1000000u, 990000);
	TEST_ASSERT_EQUAL_INT(0, cal_ina_trim(&in, &out));
	TEST_ASSERT_EQUAL_UINT16(4137u, out.shunt_cal); /* 4096/0.99 = 4137.4 */
	TEST_ASSERT_EQUAL_INT32(-10000, out.error_ppm);
}

/* A perfect monitor must come back with the calibration it already has, not
 * with an off-by-one that would walk the constant every time the bench
 * procedure is repeated. */
static void test_exact_agreement_is_a_no_op(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	in_default(&in, 1234567u, 1234567);
	TEST_ASSERT_EQUAL_INT(0, cal_ina_trim(&in, &out));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)INA228_SHUNT_CAL_DEFAULT, out.shunt_cal);
	TEST_ASSERT_EQUAL_INT32(0, out.error_ppm);
}

/*
 * Re-running the procedure against the calibration ALREADY in force must
 * converge, not oscillate. This is why cal_ina_in_t::base_shunt_cal is
 * documented as the in-force value: trimming from the nominal 4096 after a
 * previous trim throws the previous correction away.
 */
static void test_trim_is_idempotent_against_the_in_force_value(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	/* First pass: 1 % high -> 4055. */
	in_default(&in, 1000000u, 1010000);
	TEST_ASSERT_EQUAL_INT(0, cal_ina_trim(&in, &out));
	TEST_ASSERT_EQUAL_UINT16(4055u, out.shunt_cal);

	/*
	 * Second pass with the part now calibrated: it reports the truth, so the
	 * answer must be the value it is already using.
	 */
	in_default(&in, 1000000u, 1000000);
	in.base_shunt_cal = out.shunt_cal;
	TEST_ASSERT_EQUAL_INT(0, cal_ina_trim(&in, &out));
	TEST_ASSERT_EQUAL_UINT16(4055u, out.shunt_cal);
}

/* The band edges are inclusive, and a rail with a completely different full
 * scale exercises the same policy. */
static void test_band_edges_are_accepted(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	/* +20 %: i_ref/i_meas = 4915/4096 = 1.19995... */
	in_default(&in, 4915000u, 4096000);
	in.fs_current_ua = FS_VCC_RB_UA;
	TEST_ASSERT_EQUAL_INT(0, cal_ina_trim(&in, &out));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)CAL_INA_TRIM_MAX, out.shunt_cal);

	/* -20 %: 3277/4096. */
	in_default(&in, 3277000u, 4096000);
	in.fs_current_ua = FS_VCC_RB_UA;
	TEST_ASSERT_EQUAL_INT(0, cal_ina_trim(&in, &out));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)CAL_INA_TRIM_MIN, out.shunt_cal);
}

/* --------------------------------------------------------------- refusals */

/*
 * The headline case. The operator means 1.0 A and types 1000 — milliamps into a
 * microamp field, three orders of magnitude low. ina228_shunt_cal_trim() would
 * happily return the 15-bit clamp; this must refuse, and must say the REFERENCE
 * was wrong rather than blaming the board.
 */
static void test_reference_typed_in_the_wrong_unit_is_refused(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	in_default(&in, 1000u, 1000000);
	TEST_ASSERT_EQUAL_INT(-ERANGE, cal_ina_trim(&in, &out));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_REF_RANGE, out.reason);
	TEST_ASSERT_EQUAL_UINT16(0u, out.shunt_cal);
}

/* Just under 10 % of full scale is rejected; just over is not. The boundary is
 * load-bearing — it is what separates "small load" from "wrong unit". */
static void test_reference_floor_is_ten_percent_of_full_scale(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;
	uint32_t floor_ua = FS_POE_UA / 10u; /* 163840 uA */

	in_default(&in, floor_ua - 1u, (int32_t)(floor_ua - 1u));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_REF_RANGE,
				run_expect(&in, -ERANGE, &out));

	in_default(&in, floor_ua, (int32_t)floor_ua);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_OK, run_expect(&in, 0, &out));
}

/* A reference above the monitor's own full scale cannot have been measured
 * through this shunt at all. */
static void test_reference_above_full_scale_is_refused(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	in_default(&in, FS_POE_UA + 1u, (int32_t)FS_POE_UA);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_REF_RANGE,
				run_expect(&in, -ERANGE, &out));
}

static void test_zero_reference_is_refused(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	in_default(&in, 0u, 1000000);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_REF_RANGE,
				run_expect(&in, -ERANGE, &out));
}

/*
 * A negative CURRENT means the load is on the far side of the shunt from the
 * sense polarity: a wiring or rail-selection error. Trimming against it would
 * produce a nonsense ratio from a real-looking number.
 */
static void test_negative_measurement_is_refused(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	in_default(&in, 1000000u, -1000000);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_MEAS_RANGE,
				run_expect(&in, -ERANGE, &out));
	TEST_ASSERT_EQUAL_INT32(-2000000, out.error_ppm);
}

static void test_zero_measurement_is_refused(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	in_default(&in, 1000000u, 0);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_MEAS_RANGE,
				run_expect(&in, -ERANGE, &out));
}

/*
 * Right rail, right units, wrong load — the two currents differ by 30 %, which
 * no 1 % metal-strip shunt can produce. The trim (4096/1.3 = 3151) is well
 * inside the register's 15-bit field and outside the schema's band, so only the
 * band catches it.
 */
static void test_disagreement_beyond_the_band_is_refused(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	in_default(&in, 1000000u, 1300000);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_DISAGREE,
				run_expect(&in, -ERANGE, &out));
	TEST_ASSERT_EQUAL_UINT16(0u, out.shunt_cal);
	/* The error is still reported: it is what tells the operator whether
	 * they mistyped or the board is genuinely wrong. */
	TEST_ASSERT_EQUAL_INT32(300000, out.error_ppm);
}

/*
 * The band is symmetric in SHUNT_CAL and therefore ASYMMETRIC in the
 * measurement error it tolerates, because the trim is a reciprocal: a monitor
 * may read 25.0 % high (4096/3277) but only 16.7 % low (4096/4915). Reading
 * "4096 +-20 %" as a bound on the error is wrong by 5 percentage points on one
 * side, so both ends are asserted rather than commented.
 */
static void test_tolerated_error_window_is_asymmetric(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	/* +24.99 % high: 4096 * 1000000/1249900 = 3277.06 -> 3277, accepted. */
	in_default(&in, 1000000u, 1249900);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_OK, run_expect(&in, 0, &out));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)CAL_INA_TRIM_MIN, out.shunt_cal);

	/* +25.1 % is past it: 4096/1.251 = 3274.2. */
	in_default(&in, 1000000u, 1251000);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_DISAGREE,
				run_expect(&in, -ERANGE, &out));

	/* -16.6 % low: 4096/0.834 = 4911.3, accepted. */
	in_default(&in, 1000000u, 834000);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_OK, run_expect(&in, 0, &out));
	TEST_ASSERT_EQUAL_UINT16(4911u, out.shunt_cal);

	/* -17 % is past it: 4096/0.83 = 4934.9. */
	in_default(&in, 1000000u, 830000);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_DISAGREE,
				run_expect(&in, -ERANGE, &out));
}

static void test_disagreement_edges_are_exclusive(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	/* One microamp past the ratio used in test_band_edges_are_accepted():
	 * 4096 * 4915001/4096000 = 4915.001, which still rounds to the edge. */
	in_default(&in, 4915001u, 4096000);
	in.fs_current_ua = FS_VCC_RB_UA;
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_OK, run_expect(&in, 0, &out));

	/* And a ratio that genuinely rounds past it. */
	in_default(&in, 4916000u, 4096000);
	in.fs_current_ua = FS_VCC_RB_UA;
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_DISAGREE,
				run_expect(&in, -ERANGE, &out));
}

/*
 * A ratio big enough to reach the register's own 15-bit clamp still refuses,
 * and refuses as a disagreement rather than surfacing the codec's -ERANGE as a
 * success with a clamped value.
 */
static void test_ratio_beyond_the_register_field_is_refused(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	/* 4096 * 1638400/100 would be ~67 million: clamped to 0x7FFF by
	 * ina228_shunt_cal_trim(), still far outside the band. */
	in_default(&in, FS_POE_UA, 100);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_DISAGREE,
				run_expect(&in, -ERANGE, &out));
	TEST_ASSERT_EQUAL_UINT16(0u, out.shunt_cal);
}

/*
 * error_ppm is reported on refusals as well as successes, so it has to survive
 * inputs three orders of magnitude apart without wrapping. At a reference near
 * the floor and a measurement near INT32_MAX the true figure is ~1.3e10 ppm,
 * which does not fit int32 — it saturates rather than wrapping to a small or
 * negative number that would read as a healthy board.
 */
static void test_error_ppm_saturates_rather_than_wrapping(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	in_default(&in, FS_POE_UA / 10u, 2000000000);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_DISAGREE,
				run_expect(&in, -ERANGE, &out));
	TEST_ASSERT_EQUAL_INT32(2000000000, out.error_ppm);

	in_default(&in, FS_POE_UA / 10u, -2000000000);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_MEAS_RANGE,
				run_expect(&in, -ERANGE, &out));
	TEST_ASSERT_EQUAL_INT32(-2000000000, out.error_ppm);
}

/*
 * The 10 %-of-full-scale floor is integer arithmetic, so a full scale under
 * 10 uA would compute a floor of zero and re-admit the zero reference the
 * MEAS/REF checks exist to reject. No rail on this board is anywhere near that,
 * which is exactly why the guard needs a test rather than a reviewer.
 */
static void test_tiny_full_scale_still_rejects_a_zero_reference(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	in_default(&in, 0u, 5);
	in.fs_current_ua = 5u;
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_REF_RANGE,
				run_expect(&in, -ERANGE, &out));

	/* 1 uA is the clamped floor and is accepted. */
	in_default(&in, 1u, 1);
	in.fs_current_ua = 5u;
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_OK, run_expect(&in, 0, &out));
}

/* ------------------------------------------------------- reading validity */

static void test_invalid_reading_is_refused(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	in_default(&in, 1000000u, 1000000);
	in.meas_valid = false;
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_NO_READING,
				run_expect(&in, -ENODATA, &out));
}

/*
 * A stale reading is a calibration against whatever the board was doing before
 * the operator applied the reference load. The boundary is inclusive: a reading
 * exactly at max_age_ms is still accepted.
 */
static void test_stale_reading_is_refused_at_the_boundary(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	in_default(&in, 1000000u, 1000000);
	in.meas_age_ms = in.max_age_ms;
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_OK, run_expect(&in, 0, &out));

	in.meas_age_ms = in.max_age_ms + 1u;
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_NO_READING,
				run_expect(&in, -ENODATA, &out));
}

/* The reading is checked before the reference, so an unusable monitor is
 * reported as such even when the operator's number is also wrong. */
static void test_no_reading_outranks_a_bad_reference(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	in_default(&in, 1u, 1000000);
	in.meas_valid = false;
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_NO_READING,
				run_expect(&in, -ENODATA, &out));
}

/* ------------------------------------------------------ structural errors */

static void test_null_arguments(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	in_default(&in, 1000000u, 1000000);
	TEST_ASSERT_EQUAL_INT(-EINVAL, cal_ina_trim(&in, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cal_ina_trim(NULL, &out));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_ARG, out.reason);
	TEST_ASSERT_EQUAL_UINT16(0u, out.shunt_cal);
}

/*
 * A zero base calibration would compute a trim of zero, which the INA228
 * accepts silently and then reads 0 A forever with no error flag (ina228.h).
 * A zero full scale would make the reference plausibility check vacuous.
 */
static void test_structurally_impossible_inputs(void)
{
	cal_ina_in_t in;
	cal_ina_out_t out;

	in_default(&in, 1000000u, 1000000);
	in.base_shunt_cal = 0u;
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_ARG,
				run_expect(&in, -EINVAL, &out));

	in_default(&in, 1000000u, 1000000);
	in.fs_current_ua = 0u;
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_ARG,
				run_expect(&in, -EINVAL, &out));

	in_default(&in, 1000000u, 1000000);
	in.trim_min = 0u;
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_ARG,
				run_expect(&in, -EINVAL, &out));

	in_default(&in, 1000000u, 1000000);
	in.trim_min = 5000u;
	in.trim_max = 4000u;
	TEST_ASSERT_EQUAL_UINT8((uint8_t)CAL_INA_ERR_ARG,
				run_expect(&in, -EINVAL, &out));
}

/* ------------------------------------------------------------- the table */

/*
 * The band this module enforces must be the cfg schema's own bound on
 * cal.ina.0..8 (cfg_schema.h 0x0C01..0x0C09, "4096, 3277, 4915"). If the two
 * ever drift apart the procedure computes a value cfg_set() then rejects, and
 * the operator gets -EIO from a trim that was actually fine.
 */
static void test_band_matches_the_schema_bound(void)
{
	TEST_ASSERT_EQUAL_UINT32(3277u, (uint32_t)CAL_INA_TRIM_MIN);
	TEST_ASSERT_EQUAL_UINT32(4915u, (uint32_t)CAL_INA_TRIM_MAX);
	/* 4096 +-20 %, rounded as the schema rows round: 3276.8 -> 3277 and
	 * 4915.2 -> 4915, both toward the nominal value. */
	TEST_ASSERT_EQUAL_UINT32(
		((uint32_t)INA228_SHUNT_CAL_DEFAULT * 8u + 9u) / 10u,
		(uint32_t)CAL_INA_TRIM_MIN);
	TEST_ASSERT_EQUAL_UINT32((uint32_t)INA228_SHUNT_CAL_DEFAULT * 12u / 10u,
				 (uint32_t)CAL_INA_TRIM_MAX);
}

/* Every rail on the board must be trimmable: the 10 % floor has to sit inside
 * each monitor's usable range, and full scale must be non-zero so the checks
 * are not vacuous. */
static void test_every_rail_can_be_trimmed(void)
{
	size_t r;

	for (r = 0u; r < (size_t)INA228_RAIL_COUNT; r++) {
		const ina228_rail_info_t *info = &ina228_rail_tbl[r];
		cal_ina_in_t in;
		cal_ina_out_t out;
		uint32_t i;

		TEST_ASSERT_TRUE(info->fs_current_ua > 0u);

		/* The rail's design maximum is the load the bench applies, and
		 * it must clear the reference floor. */
		i = info->design_max_ma * 1000u;
		TEST_ASSERT_TRUE(i >= (info->fs_current_ua / 10u));
		TEST_ASSERT_TRUE(i <= info->fs_current_ua);

		in_default(&in, i, (int32_t)i);
		in.fs_current_ua = info->fs_current_ua;
		TEST_ASSERT_EQUAL_INT(0, cal_ina_trim(&in, &out));
		TEST_ASSERT_EQUAL_UINT16((uint16_t)INA228_SHUNT_CAL_DEFAULT,
					 out.shunt_cal);
	}
}

static void test_reason_names(void)
{
	uint8_t r;

	for (r = 0u; r < (uint8_t)CAL_INA_REASON__COUNT; r++) {
		TEST_ASSERT_NOT_NULL(cal_ina_reason_name(r));
		TEST_ASSERT_TRUE(cal_ina_reason_name(r)[0] != '\0');
	}
	TEST_ASSERT_EQUAL_STRING("?",
				 cal_ina_reason_name((uint8_t)CAL_INA_REASON__COUNT));
	TEST_ASSERT_EQUAL_STRING("?", cal_ina_reason_name(255u));
	TEST_ASSERT_EQUAL_STRING("ok", cal_ina_reason_name((uint8_t)CAL_INA_OK));
}

/* ------------------------------------------------------------------- main */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_one_percent_high_is_trimmed_down);
	RUN_TEST(test_one_percent_low_is_trimmed_up);
	RUN_TEST(test_exact_agreement_is_a_no_op);
	RUN_TEST(test_trim_is_idempotent_against_the_in_force_value);
	RUN_TEST(test_band_edges_are_accepted);

	RUN_TEST(test_reference_typed_in_the_wrong_unit_is_refused);
	RUN_TEST(test_reference_floor_is_ten_percent_of_full_scale);
	RUN_TEST(test_reference_above_full_scale_is_refused);
	RUN_TEST(test_zero_reference_is_refused);
	RUN_TEST(test_negative_measurement_is_refused);
	RUN_TEST(test_zero_measurement_is_refused);
	RUN_TEST(test_disagreement_beyond_the_band_is_refused);
	RUN_TEST(test_tolerated_error_window_is_asymmetric);
	RUN_TEST(test_disagreement_edges_are_exclusive);
	RUN_TEST(test_ratio_beyond_the_register_field_is_refused);
	RUN_TEST(test_error_ppm_saturates_rather_than_wrapping);
	RUN_TEST(test_tiny_full_scale_still_rejects_a_zero_reference);

	RUN_TEST(test_invalid_reading_is_refused);
	RUN_TEST(test_stale_reading_is_refused_at_the_boundary);
	RUN_TEST(test_no_reading_outranks_a_bad_reference);

	RUN_TEST(test_null_arguments);
	RUN_TEST(test_structurally_impossible_inputs);

	RUN_TEST(test_band_matches_the_schema_bound);
	RUN_TEST(test_every_rail_can_be_trimmed);
	RUN_TEST(test_reason_names);

	return UNITY_END();
}
