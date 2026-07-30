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
#include "stats/phase_join.h"
#include "stats/phase_rec.h"

#define REC_CAP 16384u
#define JOIN_CAP (4u * DISC_ADEV_CAP)

static uint8_t rec[REC_CAP];
static uint8_t rec_b[REC_CAP];
static disc_phase_snap_t snap;
static int64_t x_ps[DISC_ADEV_CAP];
static int64_t raw_ps[DISC_ADEV_CAP];

static int64_t join_x[JOIN_CAP];
static int64_t join_raw[JOIN_CAP];
static uint8_t join_map[(JOIN_CAP + 7u) / 8u];

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

/*
 * @p epoch_seed is what the platform draws from the TRNG; 0 is the "no seed"
 * case, in which the loop must publish NO identity rather than a reproducible
 * one (see disc_cfg_t::adev_epoch_seed).
 */
static void ctx_init_seeded(disc_ctx_t *ctx, uint32_t epoch_seed)
{
	disc_cfg_t cfg;

	TEST_ASSERT_EQUAL_INT(0, disc_cfg_defaults(&cfg));
	TEST_ASSERT_EQUAL_UINT32(0u, cfg.adev_epoch_seed);
	cfg.adev_epoch_seed = epoch_seed;
	TEST_ASSERT_EQUAL_INT(0, disc_init(ctx, &cfg));
}

static void ctx_init(disc_ctx_t *ctx)
{
	ctx_init_seeded(ctx, 0u);
}

/*
 * Snapshot + encode, exactly as the discipline thread's exporter does —
 * including the identity, so that what these cases decode is byte-for-byte the
 * shape disc_publish_phase() produces. @p buf is a parameter because the join
 * cases need two records alive at once.
 */
static size_t export_record_into(const disc_ctx_t *ctx, uint8_t *buf,
				 size_t cap, uint64_t mono_ms)
{
	phase_rec_meta_t m;
	size_t len = 0u;
	size_t i;

	TEST_ASSERT_EQUAL_INT(0, disc_phase_snapshot(ctx, &snap));

	for (i = 0u; i < (size_t)snap.n; i++) {
		x_ps[i] = phase_rec_ns_f_to_ps(snap.x_ns[i]);
		raw_ps[i] = phase_rec_ns_f_to_ps(snap.x_raw_ns[i]);
	}

	memset(&m, 0, sizeof(m));
	m.ver = PHASE_REC_VER;
	m.flags = snap.full ? PHASE_REC_F_FULL : 0u;
	m.n = snap.n;
	m.tau0_ns = snap.tau0_ns;
	m.gaps = snap.gaps;
	m.mono_ms = mono_ms;
	m.has_ident = snap.has_ident;
	m.epoch = snap.epoch;
	m.seq0 = snap.seq0;

	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode_pair(&m, x_ps, raw_ps,
						       snap.gapmap, buf, cap,
						       &len));
	return len;
}

static size_t export_record(const disc_ctx_t *ctx)
{
	return export_record_into(ctx, rec, REC_CAP, 0u);
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

/* ---------------------------------------------------- sample identity */

/*
 * THE COUNTER. The whole reason the record numbers its samples is that the ring
 * is a 512-sample window on a longer series, and a bench operator reaching
 * tau > 128 s has to concatenate several windows. That is only safe if the
 * numbering ADVANCES with the window — a counter that never incremented would
 * make every poll claim to be samples [0, 512) and turn a concatenation into a
 * silent overwrite.
 *
 * So this drives the real loop past the ring's capacity and asserts the number
 * the real snapshot reports, then joins the two real records with the real
 * joiner. It is the case that fails if disc's adev_seq stops counting.
 */
static void test_the_exported_sample_index_advances_with_the_window(void)
{
	disc_ctx_t ctx;
	phase_rec_meta_t got;
	phase_join_t j;
	size_t len_a;
	size_t len_b;
	uint64_t ms = 1000u;
	uint32_t epoch;
	unsigned int i;

	/*
	 * The epoch is NOT asserted to equal the seed: disc_tick_pps() resets
	 * the ring on the first sample (there is no previous interval to judge
	 * it against), so the first live generation is one past the seed. What
	 * the record must carry is a non-zero generation that is STABLE across
	 * polls of one run and different between runs — which is what is
	 * asserted here and in the two cases below.
	 */
	ctx_init_seeded(&ctx, 0x1234ABCDu);

	/* Fill the ring exactly: samples 0 .. CAP-1, nothing evicted yet. */
	for (i = 0u; i < DISC_ADEV_CAP; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}
	len_a = export_record_into(&ctx, rec, REC_CAP, 11u);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len_a, &got));
	TEST_ASSERT_TRUE(got.has_ident);
	TEST_ASSERT_TRUE(got.epoch != 0u);
	epoch = got.epoch;
	TEST_ASSERT_EQUAL_UINT16(DISC_ADEV_CAP, got.n);
	TEST_ASSERT_TRUE_MESSAGE(got.seq0 == 0u,
				 "the first window must start at sample 0");

	/* Slide it by 200 seconds. The ring now holds [200, 200+CAP), and the
	 * record must say so — this is the assertion a non-incrementing
	 * counter fails. */
	for (i = 0u; i < 200u; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}
	len_b = export_record_into(&ctx, rec_b, REC_CAP, 22u);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec_b, len_b, &got));
	TEST_ASSERT_EQUAL_UINT16(DISC_ADEV_CAP, got.n);
	TEST_ASSERT_TRUE_MESSAGE(got.seq0 == 200u,
				 "the window slid 200 samples; the index must "
				 "have advanced by 200");
	/* Same generation: nothing reset between the two polls. */
	TEST_ASSERT_EQUAL_UINT32(epoch, got.epoch);

	/*
	 * And the two real records join into the real underlying series:
	 * CAP + 200 samples, with CAP - 200 of them verified against what the
	 * first record already said. If the index did not advance, this
	 * overlap check is what would fire.
	 */
	TEST_ASSERT_EQUAL_INT(0, phase_join_init(&j, join_x, join_raw,
						 join_map, JOIN_CAP));
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec, len_a));
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_b, len_b));
	TEST_ASSERT_EQUAL_UINT32(DISC_ADEV_CAP + 200u, (uint32_t)j.n);
	TEST_ASSERT_EQUAL_UINT32(DISC_ADEV_CAP - 200u, (uint32_t)j.overlap);
	TEST_ASSERT_TRUE(j.seq0 == 0u);
}

/*
 * THE EPOCH. A ring reset restarts the numbering, so records from either side
 * of one must be un-joinable however their indices read — and after a reset the
 * indices restart at 0, which is exactly the shape that would otherwise abut or
 * overlap a previous record plausibly.
 *
 * The reset here is the real one: a PPS interval beyond ADEV_MAX_GAP_S, which
 * disc.c handles by discarding the ring. Nothing in the test reaches into the
 * loop to set an epoch.
 */
static void test_a_ring_reset_makes_the_records_unjoinable(void)
{
	disc_ctx_t ctx;
	phase_rec_meta_t before;
	phase_rec_meta_t after;
	phase_join_t j;
	size_t len_a;
	size_t len_b;
	uint64_t ms = 1000u;
	unsigned int i;

	ctx_init_seeded(&ctx, 0x00C0FFEEu);

	for (i = 0u; i < 64u; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}
	len_a = export_record_into(&ctx, rec, REC_CAP, 1u);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len_a, &before));
	TEST_ASSERT_TRUE(before.epoch != 0u);
	TEST_ASSERT_TRUE(before.seq0 == 0u);
	TEST_ASSERT_EQUAL_UINT16(64u, before.n);

	/* Five seconds missing: past ADEV_MAX_GAP_S, so the ring resets. */
	ms += 5000u;
	for (i = 0u; i < 64u; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}
	len_b = export_record_into(&ctx, rec_b, REC_CAP, 2u);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec_b, len_b, &after));

	/* The ring really did reset — 64 samples again, not 128. */
	TEST_ASSERT_EQUAL_UINT16(64u, after.n);
	/* The numbering restarted, which is exactly why the epoch had to. */
	TEST_ASSERT_TRUE(after.seq0 == 0u);
	TEST_ASSERT_TRUE_MESSAGE(after.epoch != before.epoch,
				 "a ring reset must change the epoch, or the "
				 "restarted numbering silently overlaps the "
				 "record taken before it");

	/*
	 * Both records claim samples [0, 64) of their own epoch. Without the
	 * epoch this would look like the same window twice — and the samples
	 * would very nearly agree, because the loop is settled. With it, the
	 * join is refused outright.
	 */
	TEST_ASSERT_EQUAL_INT(0, phase_join_init(&j, join_x, join_raw,
						 join_map, JOIN_CAP));
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec, len_a));
	TEST_ASSERT_EQUAL_INT(-EPROTO, phase_join_add(&j, rec_b, len_b));
	TEST_ASSERT_EQUAL_UINT32(64u, (uint32_t)j.n);
}

/*
 * THE SEEDLESS CASE. A platform that cannot draw entropy must publish NO
 * identity, because the only value it could otherwise publish is one the next
 * boot would reproduce — and two boots numbering from the same origin is
 * precisely the silent splice the epoch exists to prevent. Refusing to join is
 * a bench inconvenience; joining across a reboot is a wrong plot.
 */
static void test_without_a_seed_the_export_publishes_no_identity(void)
{
	disc_ctx_t ctx;
	phase_rec_meta_t got;
	phase_join_t j;
	size_t len;
	uint64_t ms = 1000u;
	unsigned int i;

	ctx_init_seeded(&ctx, 0u);
	for (i = 0u; i < 32u; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}
	len = export_record_into(&ctx, rec, REC_CAP, 0u);

	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_FALSE(got.has_ident);
	TEST_ASSERT_EQUAL_UINT32(0u, got.flags & PHASE_REC_F_IDENT);
	/* Still a perfectly valid record — it simply cannot be joined. */
	TEST_ASSERT_EQUAL_UINT16(32u, got.n);

	TEST_ASSERT_EQUAL_INT(0, phase_join_init(&j, join_x, join_raw,
						 join_map, JOIN_CAP));
	TEST_ASSERT_EQUAL_INT(-ENOENT, phase_join_add(&j, rec, len));

	/* A reset does not manufacture one either: 0 is sticky, because
	 * incrementing out of it would invent a generation this platform
	 * cannot make unique across a reboot. */
	ms += 5000u;
	for (i = 0u; i < 8u; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}
	len = export_record_into(&ctx, rec, REC_CAP, 0u);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_FALSE(got.has_ident);
	TEST_ASSERT_EQUAL_UINT32(0u, got.epoch);
}

/*
 * An absorbed gap survives the join at the right ABSOLUTE index. This is the
 * union property asserted against the real producer rather than a synthetic
 * one: the loop puts the gap at a ring-local index that shifts as the window
 * slides, and the joined record has to end up with it in one place.
 */
static void test_an_absorbed_gap_keeps_its_place_across_a_join(void)
{
	disc_ctx_t ctx;
	phase_join_t j;
	size_t len_a;
	size_t len_b;
	size_t i;
	uint64_t ms = 1000u;
	size_t marked = 0u;

	ctx_init_seeded(&ctx, 0x5A5A5A5Au);

	/* 20 uniform seconds, then one missing (absorbed: dt = 2 s), then 20
	 * more. The late sample is absolute index 20. */
	for (i = 0u; i < 20u; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}
	ms += 1000u;
	for (i = 0u; i < 21u; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}
	len_a = export_record_into(&ctx, rec, REC_CAP, 1u);
	TEST_ASSERT_TRUE(phase_rec_gap_at(rec, len_a, 20u));

	for (i = 0u; i < 30u; i++) {
		tick_at(&ctx, ms, 0.0);
		ms += 1000u;
	}
	len_b = export_record_into(&ctx, rec_b, REC_CAP, 2u);

	TEST_ASSERT_EQUAL_INT(0, phase_join_init(&j, join_x, join_raw,
						 join_map, JOIN_CAP));
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec, len_a));
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_b, len_b));
	TEST_ASSERT_EQUAL_UINT32(71u, (uint32_t)j.n);

	/* Exactly one gap, at absolute 20, counted once despite both records
	 * carrying it. */
	TEST_ASSERT_EQUAL_UINT32(1u, (uint32_t)j.gaps);
	for (i = 0u; i < j.n; i++) {
		bool set = (join_map[i >> 3] & (uint8_t)(1u << (i & 7u))) != 0u;

		if (set) {
			TEST_ASSERT_EQUAL_UINT32(20u, (uint32_t)i);
			marked++;
		}
	}
	TEST_ASSERT_EQUAL_UINT32(1u, (uint32_t)marked);
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

	RUN_TEST(test_the_exported_sample_index_advances_with_the_window);
	RUN_TEST(test_a_ring_reset_makes_the_records_unjoinable);
	RUN_TEST(test_without_a_seed_the_export_publishes_no_identity);
	RUN_TEST(test_an_absorbed_gap_keeps_its_place_across_a_join);

	return UNITY_END();
}
