/*
 * STS1000 "Meridian" — the phase-record export path, end to end on the host.
 *
 * WHAT IS UNDER TEST, AND WHY IT NEEDS ITS OWN SUITE
 *
 * core/disc deliberately absorbs short PPS gaps rather than resetting its ADEV
 * ring (the ADEV_MAX_GAP_S banner in disc.c). That trade is right for
 * telemetry — a pedantic reset means the tau = 100 s point never exists on a
 * real antenna — and inadmissible for offline analysis, because core/stats
 * takes a bare phase array with one tau0 and cannot see that a second went
 * missing in the middle of it.
 *
 * The export therefore has to say so, and that is a property no existing suite
 * covers: test_disc.c asserts what the loop DOES with a gap, and test_stats.c
 * asserts the arithmetic over an array someone else supplied. Nothing asserted
 * that the number crossing between them is right.
 *
 * So these cases drive the real disc_tick_pps() with real intervals, then take
 * the real snapshot and encode the real record, and assert on what a bench
 * consumer would read. The two mutations the design is guarded against are
 * named in the cases: a counter that never increments
 * (test_an_absorbed_gap_is_counted) and a record whose declared tau0 disagrees
 * with the series (test_exported_tau0_matches_the_spacing_the_loop_used).
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include "unity.h"

#include "disc/disc.h"
#include "stats/phase_rec.h"

#define REC_CAP 8192u

static uint8_t rec[REC_CAP];
static disc_phase_snap_t snap;
static int64_t x_ps[DISC_ADEV_CAP];

/* ------------------------------------------------------------- scaffolding */

static void env_defaults(disc_env_t *env, uint64_t ms)
{
	memset(env, 0, sizeof(*env));
	env->mono_ms = ms;
	env->gnss_time_locked = true;
	env->osc_temp_mc = 45000;
	env->osc_temp_valid = true;
	env->ocxo_current_ua = 450000;
	env->ocxo_current_valid = true;
	env->vc_sense_valid = false;
	env->anc.active_ref = (uint8_t)QUALITY_REF_OCXO;
	env->anc.gnss_fix = (uint8_t)QUALITY_GNSS_TIME_ONLY;
	env->anc.gnss_sv_used = 14u;
	env->anc.gnss_sv_visible = 20u;
	env->anc.gnss_tacc_ns = 12u;
	env->anc.utc_valid = true;
	env->anc.leap_current_s = 37;
	env->timebase_traceable = true;
}

/* disc.h: e = (expected - captured) * ns_per_count, so captured follows. */
static void pps_from_error(disc_pps_t *p, double e_ns, uint32_t expected)
{
	int64_t d = (int64_t)((e_ns < 0.0) ? (e_ns - 0.5) : (e_ns + 0.5));

	memset(p, 0, sizeof(*p));
	p->primary_expected = expected;
	p->primary_count = (uint32_t)(expected - (uint32_t)d);
	p->primary_ns_per_count = 1.0f;
	p->primary_valid = true;
}

/*
 * One accepted PPS second at an arbitrary monotonic instant.
 *
 * The interval between accepted samples is what the gap classifier looks at,
 * so the caller drives `ms` directly rather than a fixed cadence. Acceptance is
 * asserted, not assumed: a sample the §3.2 gate rejected never reaches the ADEV
 * ring at all, and a test that silently lost its samples would pass for the
 * wrong reason.
 */
static void tick_at(disc_ctx_t *ctx, uint64_t ms, double e_ns)
{
	disc_in_t in;
	disc_out_t out;
	static uint32_t expected = 1000000u;

	env_defaults(&in.env, ms);
	pps_from_error(&in.pps, e_ns, expected);
	expected += 1000000u;

	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(ctx, &in, NULL, &out));
	TEST_ASSERT_TRUE_MESSAGE(out.sample_accepted,
				 "sample was rejected; it never reached the ring");
}

static void ctx_init(disc_ctx_t *ctx)
{
	disc_cfg_t cfg;

	TEST_ASSERT_EQUAL_INT(0, disc_cfg_defaults(&cfg));
	TEST_ASSERT_EQUAL_INT(0, disc_init(ctx, &cfg));
}

/* Snapshot + encode, exactly as the discipline thread's exporter does. */
static size_t export_record(const disc_ctx_t *ctx)
{
	phase_rec_meta_t m;
	size_t len = 0u;
	size_t i;

	TEST_ASSERT_EQUAL_INT(0, disc_phase_snapshot(ctx, &snap));

	for (i = 0u; i < (size_t)snap.n; i++) {
		x_ps[i] = phase_rec_ns_f_to_ps(snap.x_ns[i]);
	}

	memset(&m, 0, sizeof(m));
	m.ver = PHASE_REC_VER;
	m.flags = snap.full ? PHASE_REC_F_FULL : 0u;
	m.n = snap.n;
	m.tau0_ns = snap.tau0_ns;
	m.gaps = snap.gaps;
	m.mono_ms = 0u;

	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_ps, snap.gapmap, rec,
						  REC_CAP, &len));
	return len;
}

/* --------------------------------------------------------------- the count */

static void test_an_absorbed_gap_is_counted(void)
{
	disc_ctx_t ctx;
	phase_rec_meta_t got;
	size_t len;
	uint64_t ms = 1000u;
	unsigned int i;

	ctx_init(&ctx);

	/* Ten uniform seconds. */
	for (i = 0u; i < 10u; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}
	len = export_record(&ctx);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_EQUAL_UINT16(10u, got.n);
	TEST_ASSERT_EQUAL_UINT32(0u, got.gaps);

	/*
	 * Now miss one pulse: the next accepted sample arrives 2 s after the
	 * last. dt = 2.0 is under ADEV_MAX_GAP_S (3.0), so the loop ABSORBS it
	 * — the ring is not reset and the sample count keeps climbing. That is
	 * the runtime behaviour this change must not alter, and it is precisely
	 * why the count has to exist: nothing about the resulting phase array
	 * distinguishes it from eleven consecutive seconds.
	 */
	ms += 1000u; /* one second skipped */
	tick_at(&ctx, ms, 0.0);
	ms += 1000u;

	len = export_record(&ctx);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));

	/* The ring was NOT reset — the absorb decision is unchanged. */
	TEST_ASSERT_EQUAL_UINT16(11u, got.n);
	/* And the record says the series is no longer uniformly spaced. */
	TEST_ASSERT_EQUAL_UINT32(1u, got.gaps);
	/* On the sample that arrived late, and on no other. */
	for (i = 0u; i < 11u; i++) {
		TEST_ASSERT_EQUAL_INT((int)(i == 10u),
				      (int)phase_rec_gap_at(rec, len, i));
	}

	/* Further uniform seconds do not add to the count. */
	for (i = 0u; i < 5u; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}
	len = export_record(&ctx);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_EQUAL_UINT16(16u, got.n);
	TEST_ASSERT_EQUAL_UINT32(1u, got.gaps);
}

static void test_a_gap_beyond_the_absorb_window_resets_and_stays_clean(void)
{
	disc_ctx_t ctx;
	phase_rec_meta_t got;
	size_t len;
	uint64_t ms = 1000u;
	unsigned int i;

	ctx_init(&ctx);
	for (i = 0u; i < 10u; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}

	/*
	 * A 4-second interval exceeds ADEV_MAX_GAP_S, so the loop discards the
	 * ring. The series restarts, and it restarts CLEAN: the first sample of
	 * a new series has no interval behind it, so counting a gap there would
	 * permanently taint every record taken after a real discontinuity and a
	 * bench operator could never obtain a usable capture.
	 */
	ms += 3000u;
	tick_at(&ctx, ms, 0.0);
	ms += 1000u;

	len = export_record(&ctx);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_EQUAL_UINT16(1u, got.n);
	TEST_ASSERT_EQUAL_UINT32(0u, got.gaps);

	for (i = 0u; i < 5u; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}
	len = export_record(&ctx);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_EQUAL_UINT16(6u, got.n);
	TEST_ASSERT_EQUAL_UINT32(0u, got.gaps);
}

static void test_several_gaps_are_each_counted_and_located(void)
{
	disc_ctx_t ctx;
	phase_rec_meta_t got;
	size_t len;
	uint64_t ms = 1000u;
	unsigned int i;
	size_t start = 0u;

	ctx_init(&ctx);
	/* Samples 0..4 uniform. */
	for (i = 0u; i < 5u; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}
	/* Sample 5 arrives 2 s late. */
	ms += 1000u;
	tick_at(&ctx, ms, 0.0);
	ms += 1000u;
	/* Samples 6..9 uniform. */
	for (i = 0u; i < 4u; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}
	/*
	 * Sample 10 also arrives 2 s late.
	 *
	 * Deliberately 2 s and not 3 s: dt is computed as
	 * (mono_ms delta) * 0.001f, and 3000 * 0.001f rounds to 3.0000002 in
	 * binary32, so an exactly-three-second interval falls on the RESET side
	 * of `dt > ADEV_MAX_GAP_S` rather than the absorb side. That is existing
	 * runtime behaviour and this change does not touch it; the case under
	 * test here is a second ABSORBED gap, so it uses an interval that is
	 * unambiguously inside the window.
	 */
	ms += 1000u;
	tick_at(&ctx, ms, 0.0);
	ms += 1000u;
	/* Samples 11..20 uniform. */
	for (i = 0u; i < 10u; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}

	len = export_record(&ctx);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_EQUAL_UINT16(21u, got.n);
	TEST_ASSERT_EQUAL_UINT32(2u, got.gaps);
	TEST_ASSERT_TRUE(phase_rec_gap_at(rec, len, 5u));
	TEST_ASSERT_TRUE(phase_rec_gap_at(rec, len, 10u));

	/* The salvageable window: [10, 21) is 11 samples. */
	TEST_ASSERT_EQUAL_UINT32(11u, (uint32_t)phase_rec_longest_clean(
					      rec, len, &start));
	TEST_ASSERT_EQUAL_UINT32(10u, (uint32_t)start);
}

static void test_a_gap_scrolls_out_of_the_window_with_its_sample(void)
{
	disc_ctx_t ctx;
	phase_rec_meta_t got;
	size_t len;
	uint64_t ms = 1000u;
	unsigned int i;

	ctx_init(&ctx);

	/* One absorbed gap at sample 1, then fill the ring exactly. */
	tick_at(&ctx, ms, 0.0);
	ms += 2000u; /* a missed second */
	tick_at(&ctx, ms, 0.0);
	ms += 1000u;
	for (i = 2u; i < DISC_ADEV_CAP; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}

	len = export_record(&ctx);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)DISC_ADEV_CAP, got.n);
	TEST_ASSERT_EQUAL_UINT32(1u, got.gaps);
	TEST_ASSERT_TRUE(phase_rec_gap_at(rec, len, 1u));
	TEST_ASSERT_EQUAL_UINT8(PHASE_REC_F_FULL,
				(uint8_t)(got.flags & PHASE_REC_F_FULL));

	/*
	 * One more second evicts sample 0; the gap moves to index 0. Two more
	 * evict it entirely. A count that did not follow the eviction would
	 * accumulate forever and no record on a long-running unit would ever
	 * read clean — which would make the field useless in exactly the
	 * deployment it exists for.
	 */
	tick_at(&ctx, ms, 0.0);
	ms += 1000u;
	len = export_record(&ctx);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_EQUAL_UINT32(1u, got.gaps);
	TEST_ASSERT_TRUE(phase_rec_gap_at(rec, len, 0u));

	tick_at(&ctx, ms, 0.0);
	ms += 1000u;
	len = export_record(&ctx);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)DISC_ADEV_CAP, got.n);
	TEST_ASSERT_EQUAL_UINT32(0u, got.gaps);
}

/* ---------------------------------------------------------------- the tau0 */

static void test_exported_tau0_matches_the_spacing_the_loop_used(void)
{
	disc_ctx_t ctx;
	phase_rec_meta_t got;
	size_t len;
	uint64_t ms = 1000u;
	unsigned int i;
	uint64_t first_ms = 0u;
	uint64_t last_ms = 0u;

	ctx_init(&ctx);

	/*
	 * Drive a known number of uniformly spaced seconds and measure the
	 * spacing independently of the module: the elapsed wall time divided by
	 * the number of intervals IS tau0, and the record must declare that
	 * number. An export that reported any other value would scale every tau
	 * on the resulting plot by the ratio — a wrong answer that looks
	 * completely ordinary, which is the whole reason this is asserted
	 * against measured time rather than against the module's own constant.
	 */
	for (i = 0u; i < 20u; i++) {
		tick_at(&ctx, ms, 0.0);
		if (i == 0u) {
			first_ms = ms;
		}
		last_ms = ms;
		ms += 1000u;
	}

	len = export_record(&ctx);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_EQUAL_UINT16(20u, got.n);

	{
		uint64_t span_ms = last_ms - first_ms;
		uint64_t intervals = (uint64_t)got.n - 1u;
		uint64_t measured_tau0_ns =
			(span_ms * UINT64_C(1000000)) / intervals;

		TEST_ASSERT_TRUE(intervals == 19u);
		TEST_ASSERT_TRUE(measured_tau0_ns ==
				 (uint64_t)got.tau0_ns);
	}

	/* And it is never zero, which would make every plotted tau zero. */
	TEST_ASSERT_TRUE(got.tau0_ns != 0u);
}

/* ------------------------------------------------------------- the samples */

static void test_the_record_carries_the_phase_series_itself(void)
{
	disc_ctx_t ctx;
	size_t len;
	size_t n_out = 0u;
	uint64_t ms = 1000u;
	unsigned int i;
	static const double e[6] = { 0.0, 4.0, -3.0, 7.0, -2.0, 1.0 };

	ctx_init(&ctx);
	for (i = 0u; i < 6u; i++) {
		tick_at(&ctx, ms, e[i]);
		ms += 1000u;
	}

	len = export_record(&ctx);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_samples(rec, len, x_ps,
							  DISC_ADEV_CAP,
							  &n_out));
	TEST_ASSERT_EQUAL_UINT32(6u, (uint32_t)n_out);

	/*
	 * The decoded picoseconds must be the ring's nanoseconds, scaled. The
	 * loop's own corrections (sawtooth, cable delay) are not applied here,
	 * so the sample the ring holds is the measured error as fed in.
	 */
	for (i = 0u; i < 6u; i++) {
		TEST_ASSERT_TRUE(x_ps[i] ==
				 phase_rec_ns_f_to_ps(snap.x_ns[i]));
		TEST_ASSERT_FLOAT_WITHIN(0.5f, (float)e[i], snap.x_ns[i]);
	}
}

static void test_snapshot_rejects_null(void)
{
	disc_ctx_t ctx;

	ctx_init(&ctx);
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_phase_snapshot(NULL, &snap));
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_phase_snapshot(&ctx, NULL));
}

static void test_an_empty_ring_exports_a_valid_empty_record(void)
{
	disc_ctx_t ctx;
	phase_rec_meta_t got;
	size_t len;

	ctx_init(&ctx);
	len = export_record(&ctx);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_EQUAL_UINT16(0u, got.n);
	TEST_ASSERT_EQUAL_UINT32(0u, got.gaps);
	TEST_ASSERT_TRUE(got.tau0_ns != 0u);
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_an_absorbed_gap_is_counted);
	RUN_TEST(test_a_gap_beyond_the_absorb_window_resets_and_stays_clean);
	RUN_TEST(test_several_gaps_are_each_counted_and_located);
	RUN_TEST(test_a_gap_scrolls_out_of_the_window_with_its_sample);

	RUN_TEST(test_exported_tau0_matches_the_spacing_the_loop_used);

	RUN_TEST(test_the_record_carries_the_phase_series_itself);
	RUN_TEST(test_snapshot_rejects_null);
	RUN_TEST(test_an_empty_ring_exports_a_valid_empty_record);

	return UNITY_END();
}
