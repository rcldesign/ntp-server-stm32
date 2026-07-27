/*
 * STS1000 "Meridian" — core/quality unit tests.
 *
 * Provenance of the expectations (ARCHITECTURE.md §9 wants values derived
 * independently of the code under test, not self-round-trips):
 *
 *  - NTP short format is RFC 5905 §6: unsigned 16.16 fixed-point seconds, so
 *    one second is exactly 65536 units. Every conversion expectation below is
 *    hand-computed from that definition and written out longhand in the
 *    comment next to it, including the two values that straddle the
 *    round-to-nearest boundary of the least significant unit.
 *
 *  - The holdover expectations are hand-evaluations of the closed form printed
 *    in quality.h, and the quadratic inverse is additionally checked by
 *    substituting the returned time back into the forward model — the inverse
 *    cannot agree with the forward function by accident.
 *
 *  - quality_sqrtf() is checked against a double-precision Newton iteration
 *    written here in the test. That reference shares no code, no table and no
 *    seed with the implementation; it is the definition of a square root
 *    (y such that y*y == x) iterated to convergence.
 *
 *  - The seqlock protocol tests drive the sequence counter by hand and through
 *    the documented race hook, so the retry and -EAGAIN paths are exercised
 *    deterministically in a single-threaded binary.
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include "unity.h"

#include "quality/quality.h"

/* ---------------------------------------------------------------- helpers */

/*
 * Reference square root: Newton–Raphson in double, iterated far past
 * convergence. Deliberately not sharing the seed trick of the implementation,
 * and deliberately not calling libm (the host harness links none).
 */
static double ref_sqrt(double x)
{
	double y = x;
	int i;

	if (!(x > 0.0)) {
		return 0.0;
	}
	for (i = 0; i < 80; i++) {
		y = 0.5 * (y + x / y);
	}
	return y;
}

static double rel_err(double got, double want)
{
	double d = got - want;

	if (d < 0.0) {
		d = -d;
	}
	if (want == 0.0) {
		return d;
	}
	return d / (want < 0.0 ? -want : want);
}

/* Compare against the reference taken of the same float value. */
static void check_sqrt_close(float x)
{
	TEST_ASSERT_TRUE(rel_err((double)quality_sqrtf(x), ref_sqrt((double)x)) <
			 1.0e-6);
}

/* ------------------------------------------------------------ block basics */

static void test_block_init_is_the_unsynchronised_state(void)
{
	quality_block_t b;

	memset(&b, 0xA5, sizeof(b));
	quality_block_init(&b);

	TEST_ASSERT_EQUAL_UINT16(QUALITY_BLOCK_VER, b.ver);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)sizeof(b), b.size);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_STRATUM_UNSYNC, b.stratum);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_LOCK_UNKNOWN, b.lock_state);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_REF_NONE, b.active_ref);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_GNSS_NO_FIX, b.gnss_fix);
	TEST_ASSERT_FALSE(b.holdover);
	TEST_ASSERT_FALSE(b.utc_valid);
	TEST_ASSERT_EQUAL_UINT32(0u, b.flags);
	TEST_ASSERT_EQUAL_UINT32(0u, b.root_disp_q16);
	/* "no demotion on the horizon", not "demote immediately". */
	TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, b.holdover_t_demote_s);
}

static void test_block_init_tolerates_null(void)
{
	quality_block_init(NULL); /* must not fault */
}

static void test_state_init_rejects_null(void)
{
	TEST_ASSERT_EQUAL_INT(-EINVAL, quality_state_init(NULL));
}

/* --------------------------------------------------------- publish/snapshot */

static void test_publish_snapshot_round_trip(void)
{
	quality_state_t qs;
	quality_block_t in;
	quality_block_t out;

	TEST_ASSERT_EQUAL_INT(0, quality_state_init(&qs));

	quality_block_init(&in);
	in.stratum = 1u;
	in.lock_state = (uint8_t)QUALITY_LOCK_LOCKED;
	in.active_ref = (uint8_t)QUALITY_REF_RB;
	in.last_pps_off_ns = -37;
	in.freq_err_ppb = 1.25f;
	in.refid = quality_refid('G', 'P', 'S', '\0');
	in.root_disp_q16 = 1234u;
	in.leap_pending = -1;
	in.leap_current_s = 37;
	in.leap_at_tai_s = 0x1122334455667788ULL;
	in.adev_10s = 3.5e-11f;
	in.flags = QUALITY_FLAG_OCXO_WARM | QUALITY_FLAG_GNSS_TIME_LOCKED;

	TEST_ASSERT_EQUAL_INT(0, quality_publish(&qs, &in));

	memset(&out, 0, sizeof(out));
	TEST_ASSERT_EQUAL_INT(0, quality_snapshot(&qs, &out));

	TEST_ASSERT_EQUAL_UINT8(1u, out.stratum);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_LOCK_LOCKED, out.lock_state);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_REF_RB, out.active_ref);
	TEST_ASSERT_EQUAL_INT32(-37, out.last_pps_off_ns);
	TEST_ASSERT_EQUAL_FLOAT(1.25f, out.freq_err_ppb);
	TEST_ASSERT_EQUAL_HEX32(0x47505300u, out.refid);
	TEST_ASSERT_EQUAL_UINT32(1234u, out.root_disp_q16);
	TEST_ASSERT_EQUAL_INT8(-1, out.leap_pending);
	TEST_ASSERT_EQUAL_INT16(37, out.leap_current_s);
	TEST_ASSERT_EQUAL_HEX64(0x1122334455667788ULL, out.leap_at_tai_s);
	TEST_ASSERT_EQUAL_FLOAT(3.5e-11f, out.adev_10s);
	TEST_ASSERT_EQUAL_UINT32(QUALITY_FLAG_OCXO_WARM |
					 QUALITY_FLAG_GNSS_TIME_LOCKED,
				 out.flags);
}

static void test_publish_stamps_identity_over_a_scribbled_block(void)
{
	quality_state_t qs;
	quality_block_t in;
	quality_block_t out;

	TEST_ASSERT_EQUAL_INT(0, quality_state_init(&qs));

	/* A caller that memset() its scratch block must still publish a
	 * well-formed one — ver/size/tick belong to publish, not the caller. */
	memset(&in, 0, sizeof(in));
	in.ver = 0xDEADu;
	in.size = 0x1234u;
	in.tick = 999u;

	TEST_ASSERT_EQUAL_INT(0, quality_publish(&qs, &in));
	TEST_ASSERT_EQUAL_INT(0, quality_snapshot(&qs, &out));

	TEST_ASSERT_EQUAL_UINT16(QUALITY_BLOCK_VER, out.ver);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)sizeof(out), out.size);
	TEST_ASSERT_EQUAL_UINT32(1u, out.tick);
}

static void test_tick_counts_publications(void)
{
	quality_state_t qs;
	quality_block_t b;
	quality_block_t out;
	unsigned int i;

	TEST_ASSERT_EQUAL_INT(0, quality_state_init(&qs));
	quality_block_init(&b);

	for (i = 1u; i <= 5u; i++) {
		TEST_ASSERT_EQUAL_INT(0, quality_publish(&qs, &b));
		TEST_ASSERT_EQUAL_INT(0, quality_snapshot(&qs, &out));
		TEST_ASSERT_EQUAL_UINT32(i, out.tick);
	}
}

static void test_sequence_advances_by_two_per_publish(void)
{
	quality_state_t qs;
	quality_block_t b;

	TEST_ASSERT_EQUAL_INT(0, quality_state_init(&qs));
	quality_block_init(&b);

	TEST_ASSERT_EQUAL_UINT32(0u, qs.seq);
	TEST_ASSERT_EQUAL_INT(0, quality_publish(&qs, &b));
	TEST_ASSERT_EQUAL_UINT32(2u, qs.seq);
	TEST_ASSERT_EQUAL_INT(0, quality_publish(&qs, &b));
	TEST_ASSERT_EQUAL_UINT32(4u, qs.seq);
	/* Even at rest: a reader that arrives between publications succeeds. */
	TEST_ASSERT_EQUAL_UINT32(0u, qs.seq & 1u);
}

static void test_fresh_slot_snapshots_without_waiting(void)
{
	quality_state_t qs;
	quality_block_t out;

	/* Nothing published yet must still read as a valid "nothing known"
	 * block rather than -EAGAIN: NTP starts answering before the first
	 * discipline tick. */
	TEST_ASSERT_EQUAL_INT(0, quality_state_init(&qs));
	TEST_ASSERT_EQUAL_INT(0, quality_snapshot(&qs, &out));
	TEST_ASSERT_EQUAL_UINT8(QUALITY_STRATUM_UNSYNC, out.stratum);
}

static void test_snapshot_refuses_an_in_progress_write(void)
{
	quality_state_t qs;
	quality_block_t b;
	quality_block_t out;

	TEST_ASSERT_EQUAL_INT(0, quality_state_init(&qs));
	quality_block_init(&b);
	b.stratum = 1u;
	TEST_ASSERT_EQUAL_INT(0, quality_publish(&qs, &b));

	/* Odd sequence == writer inside the copy. Every attempt must bail out
	 * before copying, and the call must fail rather than return a block
	 * that may be half old and half new. */
	qs.seq |= 1u;
	memset(&out, 0xC3, sizeof(out));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, quality_snapshot(&qs, &out));
	TEST_ASSERT_EQUAL_UINT8(0xC3, ((const uint8_t *)&out)[0]);
}

/* The race hook stands in for a concurrent writer; see quality.h. */
struct racer {
	quality_state_t *qs;
	unsigned int fires_left;
	unsigned int fired;
	uint8_t stratum_to_write;
};

static void racer_hook(void *ctx)
{
	struct racer *r = (struct racer *)ctx;

	r->fired++;
	if (r->fires_left == 0u) {
		return;
	}
	r->fires_left--;

	/* Exactly what quality_publish() does, so the reader observes a real
	 * torn window rather than a synthetic sequence bump. */
	r->qs->seq += 1u;
	r->qs->blk.stratum = r->stratum_to_write;
	r->qs->seq += 1u;
}

static void test_snapshot_retries_a_torn_read_and_returns_the_new_value(void)
{
	quality_state_t qs;
	quality_block_t b;
	quality_block_t out;
	struct racer r;

	TEST_ASSERT_EQUAL_INT(0, quality_state_init(&qs));
	quality_block_init(&b);
	b.stratum = 1u;
	TEST_ASSERT_EQUAL_INT(0, quality_publish(&qs, &b));

	r.qs = &qs;
	r.fires_left = 2u; /* lose two races, win the third */
	r.fired = 0u;
	r.stratum_to_write = 7u;
	qs.race_hook = racer_hook;
	qs.hook_ctx = &r;

	TEST_ASSERT_EQUAL_INT(0, quality_snapshot(&qs, &out));
	TEST_ASSERT_EQUAL_UINT(3u, r.fired);
	/* The retry must have re-copied: the value written by the interfering
	 * writer is the one that comes back. */
	TEST_ASSERT_EQUAL_UINT8(7u, out.stratum);
}

static void test_snapshot_gives_up_against_an_endless_writer(void)
{
	quality_state_t qs;
	quality_block_t b;
	quality_block_t out;
	struct racer r;

	TEST_ASSERT_EQUAL_INT(0, quality_state_init(&qs));
	quality_block_init(&b);
	TEST_ASSERT_EQUAL_INT(0, quality_publish(&qs, &b));

	r.qs = &qs;
	r.fires_left = 1000u;
	r.fired = 0u;
	r.stratum_to_write = 4u;
	qs.race_hook = racer_hook;
	qs.hook_ctx = &r;

	TEST_ASSERT_EQUAL_INT(-EAGAIN, quality_snapshot(&qs, &out));
	TEST_ASSERT_EQUAL_UINT(QUALITY_SNAPSHOT_RETRIES, r.fired);
}

static void test_publish_and_snapshot_reject_null(void)
{
	quality_state_t qs;
	quality_block_t b;

	TEST_ASSERT_EQUAL_INT(0, quality_state_init(&qs));
	quality_block_init(&b);

	TEST_ASSERT_EQUAL_INT(-EINVAL, quality_publish(NULL, &b));
	TEST_ASSERT_EQUAL_INT(-EINVAL, quality_publish(&qs, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, quality_snapshot(NULL, &b));
	TEST_ASSERT_EQUAL_INT(-EINVAL, quality_snapshot(&qs, NULL));
}

/* -------------------------------------------------------- NTP short format */

static void test_ntp_short_known_values(void)
{
	/* 1 s = 2^16 units by definition (RFC 5905 §6). */
	TEST_ASSERT_EQUAL_UINT32(0u, quality_ntp_short_from_ns(0));
	TEST_ASSERT_EQUAL_UINT32(65536u, quality_ntp_short_from_ns(1000000000));
	TEST_ASSERT_EQUAL_UINT32(32768u, quality_ntp_short_from_ns(500000000));
	/* 1 ms * 65536 = 65.536 units -> 66 to nearest. */
	TEST_ASSERT_EQUAL_UINT32(66u, quality_ntp_short_from_ns(1000000));
	/* 1 us * 65536 / 1e9 = 0.0655 -> 0. */
	TEST_ASSERT_EQUAL_UINT32(0u, quality_ntp_short_from_ns(1000));
	/* 65535 s * 65536 = 4294901760, the largest whole second that fits. */
	TEST_ASSERT_EQUAL_HEX32(0xFFFF0000u,
				quality_ntp_short_from_ns(65535LL * 1000000000LL));
}

static void test_ntp_short_rounds_to_nearest_at_the_lsb_boundary(void)
{
	/* One unit is 1e9/65536 = 15258.789 ns, so the half-way point is
	 * 7629.39 ns: 7629 must round down and 7630 must round up. */
	TEST_ASSERT_EQUAL_UINT32(0u, quality_ntp_short_from_ns(7629));
	TEST_ASSERT_EQUAL_UINT32(1u, quality_ntp_short_from_ns(7630));
	TEST_ASSERT_EQUAL_UINT32(1u, quality_ntp_short_from_ns(15259));
}

static void test_ntp_short_saturates_and_floors(void)
{
	TEST_ASSERT_EQUAL_UINT32(0u, quality_ntp_short_from_ns(-1));
	TEST_ASSERT_EQUAL_UINT32(0u, quality_ntp_short_from_ns(-1000000000));
	TEST_ASSERT_EQUAL_HEX32(UINT32_MAX,
				quality_ntp_short_from_ns(65536LL * 1000000000LL));
	TEST_ASSERT_EQUAL_HEX32(UINT32_MAX,
				quality_ntp_short_from_ns(INT64_MAX));
}

static void test_ntp_short_inverse(void)
{
	TEST_ASSERT_EQUAL_INT64(0, quality_ns_from_ntp_short(0u));
	TEST_ASSERT_EQUAL_INT64(1000000000, quality_ns_from_ntp_short(65536u));
	TEST_ASSERT_EQUAL_INT64(500000000, quality_ns_from_ntp_short(32768u));
	/* 1e9/65536 = 15258.789 -> 15259 to nearest. */
	TEST_ASSERT_EQUAL_INT64(15259, quality_ns_from_ntp_short(1u));
	/* 66 units = 66 * 15258.789 = 1007080 ns. */
	TEST_ASSERT_EQUAL_INT64(1007080, quality_ns_from_ntp_short(66u));
}

static void test_ntp_short_round_trips_whole_units(void)
{
	uint32_t units;

	/* Exact at every representable point: converting a whole number of
	 * units to ns and back must be the identity. */
	for (units = 0u; units <= 200000u; units += 997u) {
		int64_t ns = quality_ns_from_ntp_short(units);

		TEST_ASSERT_EQUAL_UINT32(units, quality_ntp_short_from_ns(ns));
	}
}

static void test_refid_packs_left_justified(void)
{
	/* RFC 5905 §7.3: a four-octet ASCII string, left justified and zero
	 * padded, first character in the most significant octet. */
	TEST_ASSERT_EQUAL_HEX32(0x47505300u, quality_refid('G', 'P', 'S', '\0'));
	TEST_ASSERT_EQUAL_HEX32(0x474E5353u, quality_refid('G', 'N', 'S', 'S'));
	TEST_ASSERT_EQUAL_HEX32(0x00000000u,
				quality_refid('\0', '\0', '\0', '\0'));
}

/* -------------------------------------------------------------- holdover */

static void test_holdover_linear_growth(void)
{
	/* err(t) = 100 + 2*t. Hand-evaluated at three points. */
	quality_holdover_model_t m = {
		.base_ns = 100.0f,
		.drift_ns_per_s = 2.0f,
		.aging_ns_per_s2 = 0.0f,
		.temp_ns_per_s_per_c = 0.0f,
	};

	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f,
				 quality_holdover_err_ns(&m, 0.0f, 0.0f));
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 200.0f,
				 quality_holdover_err_ns(&m, 50.0f, 0.0f));
	TEST_ASSERT_FLOAT_WITHIN(1e-2f, 7300.0f,
				 quality_holdover_err_ns(&m, 3600.0f, 0.0f));
}

static void test_holdover_temperature_term_uses_magnitude(void)
{
	/* rate = drift + temp*|dT| = 2 + 0.5*4 = 4, so err(50) = 100 + 200. */
	quality_holdover_model_t m = {
		.base_ns = 100.0f,
		.drift_ns_per_s = 2.0f,
		.aging_ns_per_s2 = 0.0f,
		.temp_ns_per_s_per_c = 0.5f,
	};

	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 300.0f,
				 quality_holdover_err_ns(&m, 50.0f, 4.0f));
	/* A fall of 4 °C is as much of an excursion as a rise of 4 °C. */
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 300.0f,
				 quality_holdover_err_ns(&m, 50.0f, -4.0f));
}

static void test_holdover_ageing_term(void)
{
	/* err(50) = 100 + 2*50 + 0.5*0.01*2500 = 100 + 100 + 12.5. */
	quality_holdover_model_t m = {
		.base_ns = 100.0f,
		.drift_ns_per_s = 2.0f,
		.aging_ns_per_s2 = 0.01f,
		.temp_ns_per_s_per_c = 0.0f,
	};

	TEST_ASSERT_FLOAT_WITHIN(1e-2f, 212.5f,
				 quality_holdover_err_ns(&m, 50.0f, 0.0f));
}

static void test_holdover_never_predicts_an_improving_oscillator(void)
{
	/* A negative ageing coefficient would bend the estimate downwards and
	 * under-report dispersion to clients. It must read as zero. */
	quality_holdover_model_t m = {
		.base_ns = 100.0f,
		.drift_ns_per_s = 2.0f,
		.aging_ns_per_s2 = -0.01f,
		.temp_ns_per_s_per_c = 0.0f,
	};

	TEST_ASSERT_FLOAT_WITHIN(1e-2f, 200.0f,
				 quality_holdover_err_ns(&m, 50.0f, 0.0f));
	/* And the inverse must agree with the clamped forward model. */
	TEST_ASSERT_FLOAT_WITHIN(1e-2f, 450.0f,
				 quality_holdover_time_to_ns(&m, 0.0f, 1000.0f));
}

static void test_holdover_edge_inputs(void)
{
	quality_holdover_model_t m = {
		.base_ns = 100.0f,
		.drift_ns_per_s = 2.0f,
		.aging_ns_per_s2 = 0.0f,
		.temp_ns_per_s_per_c = 0.0f,
	};

	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f,
				 quality_holdover_err_ns(NULL, 10.0f, 0.0f));
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f,
				 quality_holdover_err_ns(&m, -5.0f, 0.0f));
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f,
				 quality_holdover_err_ns(&m, NAN, 0.0f));
	/* A model whose base is itself nonsense must still not go negative. */
	m.base_ns = -50.0f;
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f,
				 quality_holdover_err_ns(&m, 0.0f, 0.0f));
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f,
				 quality_holdover_err_ns(&m, 10.0f, 0.0f));
}

static void test_holdover_inverse_linear(void)
{
	quality_holdover_model_t m = {
		.base_ns = 100.0f,
		.drift_ns_per_s = 2.0f,
		.aging_ns_per_s2 = 0.0f,
		.temp_ns_per_s_per_c = 0.0f,
	};

	/* (1000 - 100)/2 = 450 s. */
	TEST_ASSERT_FLOAT_WITHIN(1e-2f, 450.0f,
				 quality_holdover_time_to_ns(&m, 0.0f, 1000.0f));
	/* Already past the budget at t = 0. */
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,
				 quality_holdover_time_to_ns(&m, 0.0f, 50.0f));
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,
				 quality_holdover_time_to_ns(&m, 0.0f, 100.0f));
}

static void test_holdover_inverse_quadratic_agrees_with_the_forward_model(void)
{
	quality_holdover_model_t m = {
		.base_ns = 100.0f,
		.drift_ns_per_s = 2.0f,
		.aging_ns_per_s2 = 0.01f,
		.temp_ns_per_s_per_c = 0.0f,
	};
	float t;

	/* 0.005 t^2 + 2 t = 900  =>  t = (-2 + sqrt(4 + 18)) / 0.01 = 269.0416 */
	t = quality_holdover_time_to_ns(&m, 0.0f, 1000.0f);
	TEST_ASSERT_FLOAT_WITHIN(0.05f, 269.0416f, t);

	/* Substituting the answer back must land on the threshold. */
	TEST_ASSERT_FLOAT_WITHIN(0.5f, 1000.0f,
				 quality_holdover_err_ns(&m, t, 0.0f));
}

static void test_holdover_inverse_never_and_null(void)
{
	quality_holdover_model_t still = {
		.base_ns = 10.0f,
		.drift_ns_per_s = 0.0f,
		.aging_ns_per_s2 = 0.0f,
		.temp_ns_per_s_per_c = 0.0f,
	};
	quality_holdover_model_t backwards = {
		.base_ns = 10.0f,
		.drift_ns_per_s = -1.0f,
		.aging_ns_per_s2 = 0.0f,
		.temp_ns_per_s_per_c = 0.0f,
	};

	TEST_ASSERT_TRUE(isinf(quality_holdover_time_to_ns(NULL, 0.0f, 1.0f)));
	TEST_ASSERT_TRUE(isinf(quality_holdover_time_to_ns(&still, 0.0f, 1000.0f)));
	/* A negative drift is clamped to zero, so it never reaches either. */
	TEST_ASSERT_TRUE(
		isinf(quality_holdover_time_to_ns(&backwards, 0.0f, 1000.0f)));
	/* A NaN threshold must not read as "plenty of time left". */
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,
				 quality_holdover_time_to_ns(&still, 0.0f, NAN));
}

/* ------------------------------------------------------------------ sqrtf */

static void test_sqrtf_known_values(void)
{
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, quality_sqrtf(0.0f));
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, quality_sqrtf(1.0f));
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, quality_sqrtf(4.0f));
	TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.41421356f, quality_sqrtf(2.0f));
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100.0f, quality_sqrtf(10000.0f));
	TEST_ASSERT_FLOAT_WITHIN(1e-9f, 1.0e-5f, quality_sqrtf(1.0e-10f));
}

static void test_sqrtf_matches_a_double_newton_reference(void)
{
	/* Sweep the mantissa densely at one exponent, then sweep exponents, so
	 * both the seed constant and the iteration count are exercised across
	 * the whole normal range. */
	int i;
	double worst = 0.0;

	for (i = 1; i < 200000; i++) {
		float x = (float)i * 1.0e-2f;
		double got = (double)quality_sqrtf(x);
		double want = ref_sqrt((double)x);
		double e = rel_err(got, want);

		if (e > worst) {
			worst = e;
		}
	}
	TEST_ASSERT_TRUE(worst < 1.0e-6);

	worst = 0.0;
	for (i = -44; i <= 38; i++) {
		float x = 1.0f;
		int k;
		double got;
		double want;
		double e;

		/* Build 10^i without libm. */
		if (i >= 0) {
			for (k = 0; k < i; k++) {
				x *= 10.0f;
			}
		} else {
			for (k = 0; k < -i; k++) {
				x *= 0.1f;
			}
		}
		if (!(x > 0.0f) || isinf(x)) {
			continue;
		}
		got = (double)quality_sqrtf(x);
		want = ref_sqrt((double)x);
		e = rel_err(got, want);
		if (e > worst) {
			worst = e;
		}
	}
	TEST_ASSERT_TRUE(worst < 1.0e-6);
}

static void test_sqrtf_edges_never_trap(void)
{
	TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, quality_sqrtf(-1.0f));
	TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, quality_sqrtf(-0.0f));
	TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, quality_sqrtf(NAN));
	TEST_ASSERT_TRUE(isinf(quality_sqrtf(INFINITY)));

	/* Subnormals take the rescaling path. The reference is taken of the
	 * *float* value, because a literal below FLT_MIN does not survive the
	 * conversion exactly and the difference would otherwise be charged to
	 * the algorithm. */
	check_sqrt_close(1.4012985e-45f); /* FLT_TRUE_MIN */
	check_sqrt_close(1.0e-40f);
	check_sqrt_close(1.0e-38f);
	check_sqrt_close(3.0e38f); /* near FLT_MAX */
}

/* ------------------------------------------------------------------ names */

static void test_names(void)
{
	TEST_ASSERT_EQUAL_STRING("unknown",
				 quality_lock_state_name(QUALITY_LOCK_UNKNOWN));
	TEST_ASSERT_EQUAL_STRING("locked",
				 quality_lock_state_name(QUALITY_LOCK_LOCKED));
	TEST_ASSERT_EQUAL_STRING("holdover",
				 quality_lock_state_name(QUALITY_LOCK_HOLDOVER));
	TEST_ASSERT_EQUAL_STRING("parked",
				 quality_lock_state_name(QUALITY_LOCK_PARKED));
	TEST_ASSERT_EQUAL_STRING("invalid", quality_lock_state_name(200u));

	TEST_ASSERT_EQUAL_STRING("none", quality_ref_name(QUALITY_REF_NONE));
	TEST_ASSERT_EQUAL_STRING("ocxo", quality_ref_name(QUALITY_REF_OCXO));
	TEST_ASSERT_EQUAL_STRING("rb", quality_ref_name(QUALITY_REF_RB));
	TEST_ASSERT_EQUAL_STRING("extref", quality_ref_name(QUALITY_REF_EXTREF));
	TEST_ASSERT_EQUAL_STRING("invalid", quality_ref_name(99u));
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_block_init_is_the_unsynchronised_state);
	RUN_TEST(test_block_init_tolerates_null);
	RUN_TEST(test_state_init_rejects_null);

	RUN_TEST(test_publish_snapshot_round_trip);
	RUN_TEST(test_publish_stamps_identity_over_a_scribbled_block);
	RUN_TEST(test_tick_counts_publications);
	RUN_TEST(test_sequence_advances_by_two_per_publish);
	RUN_TEST(test_fresh_slot_snapshots_without_waiting);
	RUN_TEST(test_snapshot_refuses_an_in_progress_write);
	RUN_TEST(test_snapshot_retries_a_torn_read_and_returns_the_new_value);
	RUN_TEST(test_snapshot_gives_up_against_an_endless_writer);
	RUN_TEST(test_publish_and_snapshot_reject_null);

	RUN_TEST(test_ntp_short_known_values);
	RUN_TEST(test_ntp_short_rounds_to_nearest_at_the_lsb_boundary);
	RUN_TEST(test_ntp_short_saturates_and_floors);
	RUN_TEST(test_ntp_short_inverse);
	RUN_TEST(test_ntp_short_round_trips_whole_units);
	RUN_TEST(test_refid_packs_left_justified);

	RUN_TEST(test_holdover_linear_growth);
	RUN_TEST(test_holdover_temperature_term_uses_magnitude);
	RUN_TEST(test_holdover_ageing_term);
	RUN_TEST(test_holdover_never_predicts_an_improving_oscillator);
	RUN_TEST(test_holdover_edge_inputs);
	RUN_TEST(test_holdover_inverse_linear);
	RUN_TEST(test_holdover_inverse_quadratic_agrees_with_the_forward_model);
	RUN_TEST(test_holdover_inverse_never_and_null);

	RUN_TEST(test_sqrtf_known_values);
	RUN_TEST(test_sqrtf_matches_a_double_newton_reference);
	RUN_TEST(test_sqrtf_edges_never_trap);

	RUN_TEST(test_names);

	return UNITY_END();
}
