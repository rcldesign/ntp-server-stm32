/*
 * STS1000 "Meridian" — spec §14's sawtooth proof, end to end on the host.
 *
 * WHAT IS UNDER TEST
 *
 * §14 wants "PPS residual histograms with/without sawtooth correction (prove
 * the qErr path)". Three pieces have to hold for that to be possible at all,
 * and no existing suite covered any of them:
 *
 *   1. core/disc must CAPTURE the uncorrected residual. Before this, the ADEV
 *      ring held only the corrected value and the pre-correction one was
 *      unrecoverable one line later.
 *   2. stats/phase_rec must carry both series and refuse a record whose header
 *      claims the pair over a buffer that holds one.
 *   3. stats/saw must turn the pair into the verdict the operator acts on, and
 *      the verdict must SEPARATE a working qErr path from a sign-inverted one.
 *
 * THE ASSERTION THAT MATTERS
 *
 * test_a_sign_inversion_in_the_qerr_path_is_caught() drives the real
 * disc_tick_pps() with a real sawtooth and the real matching qErr, exports
 * through the real encoder, and requires the verdict to be CORRECTED — then
 * does it again with the qErr sign flipped in the INPUT and requires
 * SIGN-INVERTED. A firmware that inverts the sign internally turns the first
 * into SIGN-INVERTED and the second into CORRECTED, so the pair fails in both
 * directions. That is the whole point of the feature: a single corrected
 * histogram from an inverted path is plausible, single-peaked, and wrong.
 *
 * test_a_duplicated_uncorrected_series_is_not_a_pass() is the other half — if
 * the "uncorrected" series were ever wired up as a copy of the corrected one,
 * every capture would show a perfect zero-width difference. The verdict for
 * that is NOT-APPLIED, never CORRECTED.
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include "unity.h"

#include "disc/disc.h"
#include "stats/phase_rec.h"
#include "stats/saw.h"

#define REC_CAP 16384u
#define SYN_N   512u

static uint8_t rec[REC_CAP];
static disc_phase_snap_t snap;
static int64_t g_corr[SYN_N];
static int64_t g_raw[SYN_N];

/* ------------------------------------------------------------- scaffolding */

/*
 * The two independent components of a PPS residual, in picoseconds.
 *
 * `syn_w` is the receiver's quantisation artefact — the sawtooth — and `syn_t`
 * is the underlying error the correction cannot touch.
 *
 * WHY THE SHAPES ARE THESE AND NOT SOMETHING PRETTIER
 *
 * saw.h's landmarks (z = -1 correct, +3 inverted) are derived assuming t and w
 * are uncorrelated. That is an assumption about the real signal, and over a
 * finite record a merely plausible-looking pair only satisfies it
 * approximately: two coprime strides gave a sample correlation of -0.0024 here,
 * which moves z off -1 by 2e-4. Asserting the closed form to 1e-9 against that
 * input would have been asserting something untrue, and loosening the tolerance
 * to absorb it would have been fitting the test to the code.
 *
 * So the input is built to make the assumption EXACT rather than nearly true:
 *
 *   * syn_w has period 8 and sums to exactly zero over each period;
 *   * syn_t is constant across each aligned block of 8.
 *
 * Then cov(t, w) = (1/n) * SUM_blocks (t_b - mean_t) * SUM_{i in block} w_i,
 * and the inner sum is 0 by construction — so the covariance is exactly zero,
 * for any record length that is a whole number of blocks. Every count these
 * cases use (512, 256, 128, 64) is. Physically this is a per-second sawtooth
 * riding on a wander that moves more slowly than the sawtooth does, which is
 * what the real pair looks like.
 *
 * The magnitudes are the real ones: +-2450 ps of sawtooth (a ZED-F9T's
 * TIMEPULSE quantisation is this order) on +-105 ps of residual.
 */
#define SYN_PERIOD 8u

static int64_t syn_t(size_t i)
{
	return (int64_t)((((i / SYN_PERIOD) * 37u) % 211u)) - 105;
}

static int64_t syn_w(size_t i)
{
	return ((int64_t)(i % SYN_PERIOD) * 700) - 2450;
}

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

/*
 * One accepted PPS second whose MEASURED error is @p e_ps picoseconds and which
 * carries @p qerr_ps.
 *
 * disc.h: e = (expected - captured) * ns_per_count, so `captured` follows from
 * the error the caller wants. ns_per_count is 1 ps rather than the round 1 ns
 * the other disc suites use, because a sawtooth IS a sub-nanosecond quantity —
 * a 1 ns capture grid would quantise away the very signal these cases are
 * about, and would do it silently. At 1 ps per count every value fed in is a
 * whole number of counts and survives the float pipeline exactly (2.5 ns and
 * every intermediate here are dyadic), so the assertions below are about the
 * arithmetic and not about rounding.
 */
static void tick(disc_ctx_t *ctx, uint64_t ms, int64_t e_ps, int32_t qerr_ps,
		 bool qerr_valid)
{
	static uint32_t expected = 1000000u;
	disc_in_t in;
	disc_out_t out;

	env_defaults(&in.env, ms);
	memset(&in.pps, 0, sizeof(in.pps));
	in.pps.primary_expected = expected;
	in.pps.primary_count = (uint32_t)(expected - (uint32_t)e_ps);
	in.pps.primary_ns_per_count = 0.001f;
	in.pps.primary_valid = true;
	in.pps.qerr_ps = qerr_ps;
	in.pps.qerr_valid = qerr_valid;
	expected += 1000000u;

	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(ctx, &in, NULL, &out));
	TEST_ASSERT_TRUE_MESSAGE(out.sample_accepted,
				 "sample was rejected; it never reached the ring");
}

/* Unity is built without double-precision asserts; compare explicitly. */
static void assert_close(double want, double got, double eps, const char *what)
{
	TEST_ASSERT_TRUE_MESSAGE(fabs(got - want) <= eps, what);
}

static void ctx_init(disc_ctx_t *ctx)
{
	disc_cfg_t cfg;

	TEST_ASSERT_EQUAL_INT(0, disc_cfg_defaults(&cfg));
	TEST_ASSERT_EQUAL_INT(0, disc_init(ctx, &cfg));
}

/*
 * Snapshot and encode exactly as disc_thread.c's exporter does — through
 * phase_rec_encode_f() off the float rings, with no int64 staging array, so the
 * conversion path under test is the one the device runs.
 */
static size_t export_pair(const disc_ctx_t *ctx)
{
	phase_rec_meta_t m;
	size_t len = 0u;

	TEST_ASSERT_EQUAL_INT(0, disc_phase_snapshot(ctx, &snap));

	memset(&m, 0, sizeof(m));
	m.flags = snap.full ? PHASE_REC_F_FULL : 0u;
	m.n = snap.n;
	m.tau0_ns = snap.tau0_ns;
	m.gaps = snap.gaps;
	m.mono_ms = 0u;

	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode_f(&m, snap.x_ns,
						    snap.x_raw_ns, snap.gapmap,
						    rec, sizeof(rec), &len));
	return len;
}

/* Decode both series out of a record into g_corr/g_raw. Returns the count. */
static size_t decode_pair(size_t len)
{
	size_t n_c = 0u;
	size_t n_r = 0u;

	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_samples(rec, len, g_corr,
							  SYN_N, &n_c));
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_raw(rec, len, g_raw, SYN_N,
						      &n_r));
	TEST_ASSERT_EQUAL_size_t(n_c, n_r);
	return n_c;
}

/*
 * Run `n` seconds of disciplined PPS carrying the sawtooth, then export and
 * judge. @p qerr_scale is applied to the qErr the receiver reports: +1 is a
 * receiver behaving normally, -1 is the same board with the sign of the
 * reported quantisation error reversed.
 */
static void run_and_judge(int qerr_scale, size_t n, stats_saw_t *out)
{
	static disc_ctx_t ctx;
	size_t len;
	size_t got;
	size_t i;

	ctx_init(&ctx);
	for (i = 0u; i < n; i++) {
		/* Cancelling the artefact means adding -w, so the receiver
		 * reports qErr = -w and the loop adds qerr_sign * qErr. */
		int32_t q = (int32_t)(-syn_w(i) * qerr_scale);

		tick(&ctx, 1000ull * (uint64_t)(i + 1u),
		     syn_t(i) + syn_w(i), q, true);
	}

	len = export_pair(&ctx);
	got = decode_pair(len);
	TEST_ASSERT_EQUAL_size_t(n, got);
	TEST_ASSERT_EQUAL_INT(0, stats_saw_verdict(g_corr, g_raw, got, out));
}

/* ------------------------------------------- the verdict, on synthetic pairs */

/*
 * The three signatures, built the way the defects occur rather than by handing
 * stats_saw_verdict() precooked variances. Same t and w in all three; only what
 * the correction did to them changes.
 */
static void fill_syn(int mode)
{
	size_t i;

	for (i = 0u; i < SYN_N; i++) {
		int64_t t = syn_t(i);
		int64_t w = syn_w(i);

		g_raw[i] = t + w;
		switch (mode) {
		case 0: /* correct: the artefact is cancelled */
			g_corr[i] = t;
			break;
		case 1: /* sign inverted: the artefact is applied twice */
			g_corr[i] = t + (2 * w);
			break;
		case 2: /* nothing applied */
			g_corr[i] = g_raw[i];
			break;
		default: /* mispaired: a real correction, wrong pulse */
			g_corr[i] = g_raw[i] - syn_w(i + 2u);
			break;
		}
	}
}

static void test_a_working_correction_collapses_the_spread(void)
{
	stats_saw_t v;

	fill_syn(0);
	TEST_ASSERT_EQUAL_INT(0, stats_saw_verdict(g_corr, g_raw, SYN_N, &v));

	TEST_ASSERT_EQUAL_INT(STATS_SAW_CORRECTED, v.verdict);
	/*
	 * z = -1 EXACTLY. saw.h derives it for an uncorrelated t and w, and
	 * fill_syn() makes them exactly uncorrelated; every accumulation is
	 * over integers well inside a double's 53 bits, and n is a power of two
	 * so the means are exact too. The only inexact step is the final
	 * division of two identical magnitudes. 1e-12 is orders of magnitude
	 * below any real deviation (an approximately-uncorrelated input moved
	 * this by 2e-4) and orders above the floating-point floor.
	 */
	assert_close(-1.0, v.z, 1e-12, "z must be -1 for an exact correction");
	/* And the headline the operator reads: the spread collapsed. */
	TEST_ASSERT_TRUE(v.spread_ratio < 0.5);
	TEST_ASSERT_TRUE(v.sdev_corr_ps < v.sdev_raw_ps);
}

static void test_an_inverted_sign_roughly_doubles_the_spread(void)
{
	stats_saw_t v;

	fill_syn(1);
	TEST_ASSERT_EQUAL_INT(0, stats_saw_verdict(g_corr, g_raw, SYN_N, &v));

	TEST_ASSERT_EQUAL_INT(STATS_SAW_INVERTED, v.verdict);
	/* +3 exactly, for test_a_working_correction...'s reasons. */
	assert_close(3.0, v.z, 1e-12, "z must be +3 for an inverted correction");
	/*
	 * "Roughly doubles" is the brief's own words and the number a bench
	 * operator is told to look for: with the sawtooth dominating, sd(c)
	 * should be about 2x sd(r), NOT below it.
	 */
	TEST_ASSERT_TRUE(v.spread_ratio > 1.8);
	TEST_ASSERT_TRUE(v.spread_ratio < 2.2);
}

static void test_a_mispaired_qerr_is_not_reported_as_working(void)
{
	stats_saw_t v;

	fill_syn(3);
	TEST_ASSERT_EQUAL_INT(0, stats_saw_verdict(g_corr, g_raw, SYN_N, &v));

	/*
	 * A correction of the right magnitude applied to the wrong pulse adds
	 * noise instead of removing it. Unlike the two cases above there is no
	 * single closed-form z here: saw.h's +1 landmark assumes the misapplied
	 * correction is uncorrelated with the artefact, and a sawtooth shifted
	 * against itself is not (here rho = -1/7, giving z = 1 + 2/7). What is
	 * asserted is the property that matters — it lands strictly between the
	 * two failure landmarks and is NOT reported as working.
	 */
	TEST_ASSERT_EQUAL_INT(STATS_SAW_INEFFECTIVE, v.verdict);
	TEST_ASSERT_TRUE(v.z > 0.0);
	TEST_ASSERT_TRUE(v.z < 2.0);
	TEST_ASSERT_TRUE(v.spread_ratio > 1.0);
}

static void test_a_duplicated_uncorrected_series_is_not_a_pass(void)
{
	stats_saw_t v;

	fill_syn(2);
	TEST_ASSERT_EQUAL_INT(0, stats_saw_verdict(g_corr, g_raw, SYN_N, &v));

	/*
	 * If the uncorrected series were ever wired up as a copy of the
	 * corrected one — the single most plausible way to fake this feature —
	 * every difference is zero and the pair carries no information. The
	 * answer is NOT-APPLIED. Anything that reports CORRECTED here would
	 * pass every board on earth, including one with the sign inverted.
	 */
	TEST_ASSERT_EQUAL_INT(STATS_SAW_NONE, v.verdict);
	TEST_ASSERT_EQUAL_size_t(0u, v.n_applied);
	assert_close(0.0, v.sdev_saw_ps, 1e-12, "a duplicate series applies nothing");
	assert_close(1.0, v.spread_ratio, 1e-12, "a duplicate series cannot change the spread");
}

static void test_verdict_rejects_bad_arguments(void)
{
	stats_saw_t v;

	fill_syn(0);
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      stats_saw_verdict(NULL, g_raw, SYN_N, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      stats_saw_verdict(g_corr, NULL, SYN_N, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      stats_saw_verdict(g_corr, g_raw, SYN_N, NULL));
	TEST_ASSERT_EQUAL_INT(-ENODATA,
			      stats_saw_verdict(g_corr, g_raw, 1u, &v));
}

/* ------------------------------------------------- the record carries a pair */

static void test_the_exported_record_carries_both_series(void)
{
	static disc_ctx_t ctx;
	size_t len;
	size_t i;
	phase_rec_meta_t m;
	size_t n_c = 0u;
	size_t n_r = 0u;

	ctx_init(&ctx);
	for (i = 0u; i < 64u; i++) {
		tick(&ctx, 1000ull * (uint64_t)(i + 1u), syn_t(i) + syn_w(i),
		     (int32_t)(-syn_w(i)), true);
	}

	len = export_pair(&ctx);

	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &m));
	TEST_ASSERT_EQUAL_UINT8(PHASE_REC_VER, m.ver);
	TEST_ASSERT_TRUE((m.flags & PHASE_REC_F_RAW) != 0u);
	TEST_ASSERT_EQUAL_size_t(phase_rec_size_flags(m.n, m.flags), len);
	/* Two series is exactly 8 more bytes per sample than one. */
	TEST_ASSERT_EQUAL_size_t(phase_rec_size(m.n) + ((size_t)m.n * 8u), len);

	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_samples(rec, len, g_corr,
							  SYN_N, &n_c));
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_raw(rec, len, g_raw, SYN_N,
						      &n_r));
	TEST_ASSERT_EQUAL_size_t(64u, n_c);
	TEST_ASSERT_EQUAL_size_t(64u, n_r);

	/*
	 * The two series must actually DIFFER, and differ by the sawtooth. A
	 * capture where they are equal would satisfy every structural check
	 * above and prove nothing.
	 */
	for (i = 0u; i < n_c; i++) {
		int64_t applied = g_corr[i] - g_raw[i];

		TEST_ASSERT_EQUAL_INT64(-syn_w(i), applied);
	}
}

static void test_a_gated_qerr_leaves_the_two_series_equal_for_that_sample(void)
{
	static disc_ctx_t ctx;
	size_t len;
	size_t i;
	size_t n;

	/*
	 * qerr_valid false means the loop applied nothing that second. The
	 * uncorrected series must equal the corrected one for exactly those
	 * samples — which is why the record has to carry the captured value
	 * rather than a qErr the exporter re-adds: an exporter re-adding qErr
	 * has no record of the gate and would invent a correction that never
	 * happened.
	 */
	ctx_init(&ctx);
	for (i = 0u; i < 64u; i++) {
		bool valid = ((i % 4u) != 0u);

		tick(&ctx, 1000ull * (uint64_t)(i + 1u), syn_t(i) + syn_w(i),
		     (int32_t)(-syn_w(i)), valid);
	}

	len = export_pair(&ctx);
	n = decode_pair(len);
	TEST_ASSERT_EQUAL_size_t(64u, n);

	for (i = 0u; i < n; i++) {
		if ((i % 4u) == 0u) {
			TEST_ASSERT_EQUAL_INT64(g_raw[i], g_corr[i]);
		} else {
			TEST_ASSERT_EQUAL_INT64(-syn_w(i),
						g_corr[i] - g_raw[i]);
		}
	}
}

/* ------------------------------------------------ the end-to-end §14 proof */

static void test_a_sign_inversion_in_the_qerr_path_is_caught(void)
{
	stats_saw_t good;
	stats_saw_t bad;

	run_and_judge(+1, 256u, &good);
	TEST_ASSERT_EQUAL_INT_MESSAGE(STATS_SAW_CORRECTED, good.verdict,
				      "a correctly-signed qErr path must read CORRECTED");
	TEST_ASSERT_TRUE_MESSAGE(good.spread_ratio < 0.6,
				 "a working correction must collapse the spread");

	run_and_judge(-1, 256u, &bad);
	TEST_ASSERT_EQUAL_INT_MESSAGE(STATS_SAW_INVERTED, bad.verdict,
				      "an inverted qErr must read SIGN-INVERTED, not CORRECTED");
	TEST_ASSERT_TRUE_MESSAGE(bad.spread_ratio > 1.7,
				 "an inverted correction must roughly double the spread");

	/*
	 * The pair is the proof: the two runs differ ONLY in the sign of the
	 * qErr, so a verdict that came out the same for both would be reading
	 * something other than the sawtooth.
	 */
	TEST_ASSERT_TRUE(good.verdict != bad.verdict);
}

static void test_the_correction_moves_the_spread_not_the_mean(void)
{
	stats_saw_t v;

	/*
	 * §14's joint signature. The sawtooth is zero-mean by construction, so
	 * removing it must leave the mean essentially where it was — a
	 * correction that shifts the mean is doing something other than
	 * cancelling a quantisation artefact.
	 */
	run_and_judge(+1, 256u, &v);

	TEST_ASSERT_EQUAL_INT(STATS_SAW_CORRECTED, v.verdict);
	TEST_ASSERT_TRUE_MESSAGE(fabs(v.mean_shift_ps) < 0.15 * v.sdev_raw_ps,
				 "correction must not move the mean");
	TEST_ASSERT_TRUE_MESSAGE(v.sdev_corr_ps < 0.6 * v.sdev_raw_ps,
				 "correction must collapse the spread");
}

static void test_no_qerr_at_all_reads_as_not_applied(void)
{
	static disc_ctx_t ctx;
	stats_saw_t v;
	size_t len;
	size_t n;
	size_t i;

	ctx_init(&ctx);
	for (i = 0u; i < 128u; i++) {
		tick(&ctx, 1000ull * (uint64_t)(i + 1u), syn_t(i) + syn_w(i), 0,
		     false);
	}

	len = export_pair(&ctx);
	n = decode_pair(len);
	TEST_ASSERT_EQUAL_INT(0, stats_saw_verdict(g_corr, g_raw, n, &v));

	/* Nothing was applied, so the record proves nothing — and must say so
	 * rather than reporting a clean bill of health. */
	TEST_ASSERT_EQUAL_INT(STATS_SAW_NONE, v.verdict);
	TEST_ASSERT_EQUAL_size_t(0u, v.n_applied);
}

/* ------------------------------------------------------- format guards */

static void test_a_record_claiming_the_raw_series_without_it_is_refused(void)
{
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;
	size_t i;

	for (i = 0u; i < 64u; i++) {
		g_corr[i] = (int64_t)i;
	}
	memset(&m, 0, sizeof(m));
	m.n = 64u;
	m.tau0_ns = 1000000000u;
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, g_corr, NULL, rec,
						  sizeof(rec), &len));
	TEST_ASSERT_EQUAL_size_t(phase_rec_size(64u), len);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_TRUE((got.flags & PHASE_REC_F_RAW) == 0u);

	/*
	 * Flip the flag on. The record is now exactly long enough for the base
	 * layout and 512 bytes short of what it claims; a decoder that checked
	 * only the base length would hand back the header and the bitmap as if
	 * they were the uncorrected samples.
	 */
	rec[5] |= (uint8_t)PHASE_REC_F_RAW;
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE,
			      phase_rec_decode_raw(rec, len, g_raw, SYN_N, &i));
}

static void test_the_raw_series_is_absent_not_empty_on_a_single_record(void)
{
	phase_rec_meta_t m;
	size_t len = 0u;
	size_t n = 0u;

	memset(&m, 0, sizeof(m));
	m.n = 8u;
	m.tau0_ns = 1000000000u;
	memset(g_corr, 0, sizeof(int64_t) * 8u);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, g_corr, NULL, rec,
						  sizeof(rec), &len));

	/* -ENOENT, not a silently zeroed series: a consumer that got zeros
	 * would compute a verdict from a series that does not exist. */
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      phase_rec_decode_raw(rec, len, g_raw, SYN_N, &n));
}

static void test_a_v1_record_cannot_claim_the_raw_series(void)
{
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;

	memset(&m, 0, sizeof(m));
	m.n = 8u;
	m.tau0_ns = 1000000000u;
	memset(g_corr, 0, sizeof(int64_t) * 8u);
	memset(g_raw, 0, sizeof(int64_t) * 8u);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode_pair(&m, g_corr, g_raw, NULL,
						       rec, sizeof(rec), &len));

	/*
	 * A well-formed pair, at whatever version this build writes. The
	 * precondition the rest of the test needs is not "version 2" but "a
	 * version at which F_RAW is legal", so that is what is asserted —
	 * pinning the literal made this test fail on the v3 bump for a reason
	 * that had nothing to do with the property it exists to prove.
	 */
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_EQUAL_UINT8(PHASE_REC_VER, got.ver);
	TEST_ASSERT_TRUE(got.ver >= 2u);
	TEST_ASSERT_TRUE((got.flags & PHASE_REC_F_RAW) != 0u);

	/* Say it is v1 and the claim becomes impossible: F_RAW did not exist. */
	rec[4] = 1u;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, phase_rec_decode_hdr(rec, len, &got));

	/* A genuine v1 record — the version this build no longer writes — is
	 * still readable, because its layout is a strict prefix of v2's. */
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, g_corr, NULL, rec,
						  sizeof(rec), &len));
	rec[4] = 1u;
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_EQUAL_UINT8(1u, got.ver);
}

static void test_an_unknown_flag_bit_is_refused(void)
{
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;

	memset(&m, 0, sizeof(m));
	m.n = 8u;
	m.tau0_ns = 1000000000u;
	memset(g_corr, 0, sizeof(int64_t) * 8u);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, g_corr, NULL, rec,
						  sizeof(rec), &len));

	rec[5] |= 0x40u;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, phase_rec_decode_hdr(rec, len, &got));

	/* And the encoder will not create one, so the pair cannot disagree. */
	m.flags = 0x40u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, phase_rec_encode(&m, g_corr, NULL, rec,
							sizeof(rec), &len));
}

static void test_the_flag_records_what_was_written_not_what_was_asked(void)
{
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;

	memset(&m, 0, sizeof(m));
	m.n = 8u;
	m.tau0_ns = 1000000000u;
	m.flags = PHASE_REC_F_RAW; /* a caller asserting the pair it has not got */
	memset(g_corr, 0, sizeof(int64_t) * 8u);

	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode_pair(&m, g_corr, NULL, NULL,
						       rec, sizeof(rec), &len));
	TEST_ASSERT_EQUAL_size_t(phase_rec_size(8u), len);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec, len, &got));
	TEST_ASSERT_TRUE_MESSAGE((got.flags & PHASE_REC_F_RAW) == 0u,
				 "F_RAW must follow the data, not the caller's flags");
}

/* ------------------------------------------------------------------ runner */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_a_working_correction_collapses_the_spread);
	RUN_TEST(test_an_inverted_sign_roughly_doubles_the_spread);
	RUN_TEST(test_a_mispaired_qerr_is_not_reported_as_working);
	RUN_TEST(test_a_duplicated_uncorrected_series_is_not_a_pass);
	RUN_TEST(test_verdict_rejects_bad_arguments);

	RUN_TEST(test_the_exported_record_carries_both_series);
	RUN_TEST(test_a_gated_qerr_leaves_the_two_series_equal_for_that_sample);

	RUN_TEST(test_a_sign_inversion_in_the_qerr_path_is_caught);
	RUN_TEST(test_the_correction_moves_the_spread_not_the_mean);
	RUN_TEST(test_no_qerr_at_all_reads_as_not_applied);

	RUN_TEST(test_a_record_claiming_the_raw_series_without_it_is_refused);
	RUN_TEST(test_the_raw_series_is_absent_not_empty_on_a_single_record);
	RUN_TEST(test_a_v1_record_cannot_claim_the_raw_series);
	RUN_TEST(test_an_unknown_flag_bit_is_refused);
	RUN_TEST(test_the_flag_records_what_was_written_not_what_was_asked);

	return UNITY_END();
}
