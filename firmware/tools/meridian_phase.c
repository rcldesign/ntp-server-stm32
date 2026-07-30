/*
 * STS1000 "Meridian" — meridian_phase: the bench stability analyser.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Reads a phase record exported from the device (MCP PHASE_EXPORT 0x24, fetched
 * with `meridian_ctl.py phase-export`) and prints the spec §14 analysis: ADEV
 * and MDEV against tau, and the PPS residual histograms with and without
 * sawtooth correction, with a verdict on the qErr path.
 *
 * IT CALLS core/stats DIRECTLY, AND THAT IS THE POINT
 *
 * The arithmetic here is not reimplemented. This file links app/src/core/stats
 * and calls stats_adev() / stats_mdev() / stats_hist() — the same functions
 * tests/host/test_stats.c pins against closed forms. A second implementation in
 * a scripting language would be free to disagree with the proved one, and a
 * disagreement between two stability plots is worse than having no tool: the
 * operator has no way to know which is lying. Likewise the record is decoded
 * with core/stats' own phase_rec.c, the same code the device encoded it with.
 *
 * WHY IT REFUSES A RECORD WITH ABSORBED GAPS
 *
 * The estimators assume uniform sampling; a phase array carries no timestamps,
 * so they cannot check. The device's discipline loop deliberately absorbs short
 * PPS gaps rather than resetting its ring (core/disc's ADEV_MAX_GAP_S), which
 * is the right trade for telemetry and inadmissible here. An absorbed gap
 * mis-times every later second difference and produces a plot that looks
 * entirely ordinary and is wrong.
 *
 * The record states how many samples arrived after a non-uniform interval. If
 * that count is non-zero this tool REFUSES by default, and says what to do
 * about it: --clean-run analyses the longest uniformly spaced stretch instead,
 * which is usually most of the capture, and --allow-gaps forces the whole thing
 * for someone who has decided the error does not matter for what they are
 * looking at. Defaulting to "analyse it anyway with a warning" was rejected —
 * warnings on stderr get lost in a terminal and the number still gets copied
 * into a report.
 *
 * THE SAWTOOTH VERDICT IS THE DELIVERABLE, NOT THE TWO TABLES
 *
 * Spec §14 asks for the residual histogram "with/without sawtooth correction
 * (prove the qErr path)". A record from this firmware carries both series
 * (PHASE_REC_F_RAW), so this tool prints both histograms — and then states what
 * they mean, because the thing being proved is a JOINT property of the pair and
 * nobody should have to derive it by eye from two tables of counts.
 *
 * Correction should leave the mean essentially where it was and collapse the
 * spread. The failure it is being proved against is a SIGN INVERSION in the
 * qErr path, which roughly DOUBLES the spread instead of removing it — and
 * which, seen on the corrected histogram alone, looks like an ordinary
 * single-peaked distribution that happens to be wide. So the check is not "did
 * the spread shrink" but "did it shrink rather than roughly double", and that
 * question has no answer without the uncorrected series. stats_saw_verdict()
 * (core/stats/saw.h) is the arithmetic; this file prints it.
 *
 * Usage:
 *   meridian_phase [options] <record.phr>
 *   meridian_phase --self-test
 *
 * Options:
 *   --clean-run       analyse the longest gap-free run instead of refusing
 *   --allow-gaps      analyse the whole record anyway (prints a warning)
 *   --hist-bin <ps>   histogram bin width, picoseconds (default 1000 = 1 ns)
 *   --hist-bins <n>   number of histogram bins, odd, centred (default 41)
 *   --no-hist         skip the histograms (the verdict is still printed)
 *   --require-sawtooth
 *                     exit non-zero unless the verdict is CORRECTED, for a
 *                     bench script that wants the proof to gate something
 *   --self-test       analyse a synthetic exactly-linear record and check that
 *                     ADEV is exactly 0, then check that the three sawtooth
 *                     signatures produce the three verdicts, then exit. Proves
 *                     the analysis chain on this machine before it is trusted
 *                     on real data.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stats/adev.h"
#include "stats/phase_rec.h"
#include "stats/saw.h"

/* The device ring is 512 samples; allow well beyond it so a concatenated or
 * future longer record still loads rather than being silently truncated. The
 * byte budget carries TWO series per sample, because a paired record does. */
#define MAX_SAMPLES 262144u
#define MAX_FILE_BYTES (32u + (MAX_SAMPLES / 8u) + (2u * MAX_SAMPLES * 8u))

#define MAX_OCTAVES 64u
#define MAX_BINS 1024u

static int64_t g_x[MAX_SAMPLES];
static int64_t g_raw[MAX_SAMPLES];
static uint8_t g_buf[MAX_FILE_BYTES];
static uint32_t g_m[MAX_OCTAVES];
static double g_adev[MAX_OCTAVES];
static double g_mdev[MAX_OCTAVES];
static uint32_t g_bins[MAX_BINS];

/* ------------------------------------------------------------------- io */

static int read_file(const char *path, size_t *out_len)
{
	FILE *f = fopen(path, "rb");
	size_t n;

	if (f == NULL) {
		fprintf(stderr, "meridian_phase: %s: %s\n", path,
			strerror(errno));
		return -1;
	}

	n = fread(g_buf, 1u, sizeof(g_buf), f);
	if (ferror(f) != 0) {
		fprintf(stderr, "meridian_phase: %s: read error\n", path);
		(void)fclose(f);
		return -1;
	}
	/* A file that exactly fills the buffer is indistinguishable from one
	 * that overflowed it, and analysing a truncated record is the failure
	 * this whole tool exists to avoid. */
	if (n == sizeof(g_buf)) {
		fprintf(stderr,
			"meridian_phase: %s: larger than the %zu-byte limit\n",
			path, sizeof(g_buf));
		(void)fclose(f);
		return -1;
	}
	(void)fclose(f);

	*out_len = n;
	return 0;
}

/* -------------------------------------------------------------- analysis */

static void print_header(const phase_rec_meta_t *m, const char *path)
{
	printf("record       : %s\n", path);
	printf("samples      : %u\n", (unsigned int)m->n);
	printf("tau0         : %u ns (%.6f s)\n", (unsigned int)m->tau0_ns,
	       (double)m->tau0_ns / 1.0e9);
	printf("device mono  : %llu ms\n", (unsigned long long)m->mono_ms);
	printf("ring full    : %s\n",
	       ((m->flags & PHASE_REC_F_FULL) != 0u)
		       ? "yes (older samples were evicted)"
		       : "no");
	printf("absorbed gaps: %u\n", (unsigned int)m->gaps);
}

static void print_dev_table(const int64_t *x, size_t n, uint64_t tau0_ns)
{
	size_t n_oct;
	size_t n_a = 0u;
	size_t n_m = 0u;
	size_t i;
	int rc;

	n_oct = stats_m_octaves(n, STATS_DEV_ADEV, g_m, MAX_OCTAVES);
	if (n_oct > MAX_OCTAVES) {
		n_oct = MAX_OCTAVES;
	}
	if (n_oct == 0u) {
		printf("\n(too few samples for any tau)\n");
		return;
	}

	rc = stats_dev_sweep(STATS_DEV_ADEV, x, n, tau0_ns, g_m, n_oct, g_adev,
			     &n_a);
	if (rc != 0) {
		fprintf(stderr, "meridian_phase: ADEV sweep failed (%d)\n", rc);
		return;
	}
	rc = stats_dev_sweep(STATS_DEV_MDEV, x, n, tau0_ns, g_m, n_oct, g_mdev,
			     &n_m);
	if (rc != 0) {
		fprintf(stderr, "meridian_phase: MDEV sweep failed (%d)\n", rc);
		return;
	}

	printf("\n%12s %10s %14s %14s\n", "tau (s)", "m", "ADEV", "MDEV");
	for (i = 0u; i < n_a; i++) {
		double tau = (double)g_m[i] * (double)tau0_ns / 1.0e9;

		printf("%12.3f %10u %14.4e ", tau, (unsigned int)g_m[i],
		       g_adev[i]);
		if (i < n_m) {
			printf("%14.4e\n", g_mdev[i]);
		} else {
			/* MDEV needs 3m samples where ADEV needs 2m+1, so the
			 * MDEV column ends first. Saying so beats printing a
			 * placeholder that could be read as a value. */
			printf("%14s\n", "-");
		}
	}
}

static void print_hist(const char *label, const int64_t *x, size_t n,
		       int64_t bin_ps, size_t n_bins)
{
	stats_hist_t s;
	int64_t lo;
	size_t i;
	uint32_t peak = 0u;
	int rc;

	/* Two-pass auto-range, as adev.h prescribes: summary first for the
	 * centre, then the binned pass around it. */
	rc = stats_hist(x, n, 0, 1, NULL, 0u, &s);
	if (rc != 0) {
		fprintf(stderr, "meridian_phase: histogram failed (%d)\n", rc);
		return;
	}

	lo = (int64_t)((double)s.mean_ps) - ((int64_t)(n_bins / 2u) * bin_ps);
	rc = stats_hist(x, n, lo, bin_ps, g_bins, n_bins, &s);
	if (rc != 0) {
		fprintf(stderr, "meridian_phase: histogram failed (%d)\n", rc);
		return;
	}

	printf("\nPPS residual summary — %s (picoseconds)\n", label);
	printf("  n=%zu  min=%lld  max=%lld  p2p=%lld\n", s.n,
	       (long long)s.min_ps, (long long)s.max_ps, (long long)s.p2p_ps);
	printf("  mean=%.1f  rms=%.1f  sdev=%.1f\n", s.mean_ps, s.rms_ps,
	       s.sdev_ps);
	printf("  in_range=%zu under=%zu over=%zu\n", s.in_range, s.under,
	       s.over);

	for (i = 0u; i < n_bins; i++) {
		if (g_bins[i] > peak) {
			peak = g_bins[i];
		}
	}
	if (peak == 0u) {
		return;
	}

	printf("\n%14s %8s\n", "bin lo (ps)", "count");
	for (i = 0u; i < n_bins; i++) {
		int bar = (int)(((uint64_t)g_bins[i] * 40u) / peak);
		int k;

		if (g_bins[i] == 0u) {
			continue;
		}
		printf("%14lld %8u  ", (long long)(lo + (int64_t)i * bin_ps),
		       g_bins[i]);
		for (k = 0; k < bar; k++) {
			(void)putchar('#');
		}
		(void)putchar('\n');
	}
}

/* ------------------------------------------------------ sawtooth verdict */

/*
 * The §14 proof, stated. Returns the verdict so main() can gate an exit code on
 * it, and so the self-test can assert on it.
 *
 * Everything printed here comes from stats_saw_verdict(); nothing is
 * recomputed. The prose exists because "z = 3.02" is not an answer a bench
 * operator can act on, and because the actionable part of each verdict is
 * different: a collapsed spread is done, a doubled one is a firmware sign to
 * flip, and an absent correction is a receiver or pairing question that has not
 * reached the arithmetic yet.
 */
static stats_saw_verdict_t print_saw(const int64_t *corr, const int64_t *raw,
				     size_t n)
{
	stats_saw_t v;
	int rc;

	rc = stats_saw_verdict(corr, raw, n, &v);
	if (rc != 0) {
		fprintf(stderr,
			"meridian_phase: sawtooth verdict failed (%d)\n", rc);
		return STATS_SAW_NONE;
	}

	printf("\nSAWTOOTH CORRECTION (spec §14 — proving the qErr path)\n");
	printf("  %-14s %14s %14s\n", "", "uncorrected", "corrected");
	printf("  %-14s %14.1f %14.1f\n", "mean (ps)", v.mean_raw_ps,
	       v.mean_corr_ps);
	printf("  %-14s %14.1f %14.1f\n", "sdev (ps)", v.sdev_raw_ps,
	       v.sdev_corr_ps);
	if (v.spread_ratio >= 0.0) {
		printf("  spread ratio   : %.3f  (corrected / uncorrected)\n",
		       v.spread_ratio);
	} else {
		printf("  spread ratio   : n/a (the uncorrected spread is 0)\n");
	}
	printf("  mean shift     : %+.1f ps\n", v.mean_shift_ps);
	printf("  correction     : applied to %zu of %zu samples, sdev %.1f ps\n",
	       v.n_applied, v.n, v.sdev_saw_ps);
	if (v.verdict != STATS_SAW_NONE) {
		printf("  z              : %+.3f +- %.3f  "
		       "(-1 correct, +1 mispaired, +3 sign-inverted)\n",
		       v.z, v.z_sigma);
	}

	printf("\n  VERDICT: %s — ", stats_saw_str(v.verdict));
	switch (v.verdict) {
	case STATS_SAW_CORRECTED:
		printf("spread collapsed.\n");
		printf("    The qErr path is removing the sawtooth. The mean "
		       "moved by %+.1f ps,\n"
		       "    which is the residual cable/board bias the "
		       "correction does not touch.\n",
		       v.mean_shift_ps);
		break;
	case STATS_SAW_INVERTED:
		printf("spread roughly DOUBLED.\n");
		printf("    This is the signature of a SIGN INVERSION in the "
		       "qErr path: the\n"
		       "    correction is being subtracted where it should be "
		       "added, so the\n"
		       "    sawtooth is applied twice instead of cancelled. "
		       "Check disc_cfg_t\n"
		       "    ::qerr_sign and the sign of the qErr term in "
		       "core/disc's\n"
		       "    condition_sample(). The corrected histogram above "
		       "is NOT usable.\n");
		break;
	case STATS_SAW_INEFFECTIVE:
		printf("spread did not collapse.\n");
		printf("    A correction was applied and it did not remove the "
		       "sawtooth. z near\n"
		       "    +1 means the qErr is uncorrelated with the residual "
		       "— the magnitude\n"
		       "    is plausible but it belongs to a different pulse. "
		       "Check the UBX-TIM-TP\n"
		       "    pairing (core/disc disc_name_pulse) before "
		       "suspecting the sign.\n");
		break;
	case STATS_SAW_NONE:
	default:
		printf("no correction is present in this capture.\n");
		printf("    Every sample's corrected and uncorrected residual "
		       "is identical, so\n"
		       "    this record proves nothing about the qErr path "
		       "either way. Either the\n"
		       "    receiver supplied no usable qErr (UBX-TIM-TP "
		       "missing, or unpaired) or\n"
		       "    the correction is disabled. Capture again with the "
		       "receiver locked.\n");
		break;
	}

	if (v.marginal && (v.verdict != STATS_SAW_NONE)) {
		printf("    MARGINAL: z is within 2 sigma of a verdict "
		       "boundary. The sawtooth is\n"
		       "    small next to the rest of the residual, or the "
		       "capture is short.\n"
		       "    Take a longer record before acting on this.\n");
	}

	return v.verdict;
}

/* ------------------------------------------------------------- self-test */

/*
 * A record that is exactly linear in i is a pure frequency offset with no
 * instability at all, so its Allan deviation is EXACTLY zero — adev.h states
 * that as a guarantee of the implementation, not an approximation, because the
 * second differences are formed in int64. That makes it the sharpest possible
 * end-to-end check: any sign slip, stride error, dropped factor or float
 * contamination anywhere between the record and the printed number turns an
 * exact 0 into something that is not 0.
 */
/*
 * A deterministic sawtooth and a deterministic underlying residual, pushed
 * through the encoder, the decoder and the verdict — three times, once per
 * signature. This is the check that the tool's headline output is wired to the
 * arithmetic and not to a constant: the SAME t[] and w[] produce three
 * different verdicts, so a printer stuck on any one of them fails here.
 *
 * The three cases are constructed the way the defects occur, not by feeding
 * stats_saw_verdict() precooked variances:
 *
 *   correct  : c = t          (the applied delta is -w, cancelling the artefact)
 *   inverted : c = t + 2w     (the delta is +w, applying the artefact twice)
 *   absent   : c = r          (no delta at all — also the shape a "raw series is
 *                              a copy of the corrected one" bug produces)
 */
static int saw_case(const char *name, int use_inverted, int use_absent,
		    stats_saw_verdict_t want)
{
	const size_t n = 512u;
	phase_rec_meta_t m;
	stats_saw_t v;
	size_t len = 0u;
	size_t n_c = 0u;
	size_t n_r = 0u;
	size_t i;
	int rc;

	for (i = 0u; i < n; i++) {
		/* Underlying residual: a slow, sawtooth-independent wander. */
		int64_t t = (int64_t)(((i * 37u) % 211u)) - 105;
		/* The receiver's quantisation artefact, +-2500 ps, stepping
		 * through the interval on a stride coprime with 211 so it does
		 * not correlate with t. */
		int64_t w = (int64_t)(((i * 97u) % 101u) * 50u) - 2500;

		g_raw[i] = t + w;
		if (use_absent) {
			g_x[i] = g_raw[i];
		} else if (use_inverted) {
			g_x[i] = t + (2 * w);
		} else {
			g_x[i] = t;
		}
	}

	memset(&m, 0, sizeof(m));
	m.n = (uint16_t)n;
	m.tau0_ns = 1000000000u;
	rc = phase_rec_encode_pair(&m, g_x, g_raw, NULL, g_buf, sizeof(g_buf),
				   &len);
	if (rc != 0) {
		printf("self-test: saw %s encode failed (%d)\n", name, rc);
		return 1;
	}
	if (len != phase_rec_size_flags((uint16_t)n, PHASE_REC_F_RAW)) {
		printf("self-test: saw %s FAIL paired length %zu\n", name, len);
		return 1;
	}

	memset(g_x, 0, sizeof(int64_t) * n);
	memset(g_raw, 0, sizeof(int64_t) * n);
	rc = phase_rec_decode_samples(g_buf, len, g_x, MAX_SAMPLES, &n_c);
	if (rc == 0) {
		rc = phase_rec_decode_raw(g_buf, len, g_raw, MAX_SAMPLES, &n_r);
	}
	if ((rc != 0) || (n_c != n) || (n_r != n)) {
		printf("self-test: saw %s decode failed (%d, %zu/%zu)\n", name,
		       rc, n_c, n_r);
		return 1;
	}

	rc = stats_saw_verdict(g_x, g_raw, n, &v);
	if (rc != 0) {
		printf("self-test: saw %s verdict failed (%d)\n", name, rc);
		return 1;
	}
	printf("self-test: saw %-8s -> %-13s z=%+.3f ratio=%.3f\n", name,
	       stats_saw_str(v.verdict), v.z, v.spread_ratio);
	if (v.verdict != want) {
		printf("self-test: saw %s FAIL expected %s\n", name,
		       stats_saw_str(want));
		return 1;
	}
	return 0;
}

static int saw_self_test(void)
{
	int fails = 0;

	fails += saw_case("correct", 0, 0, STATS_SAW_CORRECTED);
	fails += saw_case("inverted", 1, 0, STATS_SAW_INVERTED);
	fails += saw_case("absent", 0, 1, STATS_SAW_NONE);

	/*
	 * And the record-level guard: a header claiming the raw series over a
	 * buffer that only holds one must be refused. Without this the whole
	 * pair can be faked by flipping one bit.
	 */
	{
		const size_t n = 64u;
		phase_rec_meta_t m;
		phase_rec_meta_t got;
		size_t len = 0u;
		size_t i;
		int rc;

		for (i = 0u; i < n; i++) {
			g_x[i] = (int64_t)i;
		}
		memset(&m, 0, sizeof(m));
		m.n = (uint16_t)n;
		m.tau0_ns = 1000000000u;
		rc = phase_rec_encode(&m, g_x, NULL, g_buf, sizeof(g_buf),
				      &len);
		if ((rc != 0) ||
		    (len != phase_rec_size_flags((uint16_t)n, 0u))) {
			printf("self-test: saw single-series encode failed "
			       "(%d)\n", rc);
			return fails + 1;
		}
		g_buf[5] |= (uint8_t)PHASE_REC_F_RAW;
		rc = phase_rec_decode_hdr(g_buf, len, &got);
		if (rc != -EMSGSIZE) {
			printf("self-test: saw FAIL a record claiming F_RAW "
			       "without the bytes decoded as %d\n", rc);
			fails++;
		}
	}

	return fails;
}

static int self_test(void)
{
	const size_t n = 512u;
	const uint64_t tau0_ns = 1000000000u;
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;
	size_t n_out = 0u;
	size_t i;
	double adev = -1.0;
	double mdev = -1.0;
	int rc;
	int fails = 0;

	for (i = 0u; i < n; i++) {
		/* x[i] = a + b*i, in picoseconds. */
		g_x[i] = INT64_C(-12345) + ((int64_t)i * INT64_C(6789));
	}

	memset(&m, 0, sizeof(m));
	m.ver = PHASE_REC_VER;
	m.n = (uint16_t)n;
	m.tau0_ns = (uint32_t)tau0_ns;
	m.gaps = 0u;
	m.mono_ms = 0u;

	rc = phase_rec_encode(&m, g_x, NULL, g_buf, sizeof(g_buf), &len);
	if (rc != 0) {
		printf("self-test: encode failed (%d)\n", rc);
		return 1;
	}

	/* Go back through the decoder, so the check covers the container the
	 * device actually produces and not just the estimator. */
	memset(g_x, 0, sizeof(int64_t) * n);
	rc = phase_rec_decode_hdr(g_buf, len, &got);
	if (rc != 0) {
		printf("self-test: decode header failed (%d)\n", rc);
		return 1;
	}
	rc = phase_rec_decode_samples(g_buf, len, g_x, MAX_SAMPLES, &n_out);
	if ((rc != 0) || (n_out != n)) {
		printf("self-test: decode samples failed (%d, n=%zu)\n", rc,
		       n_out);
		return 1;
	}
	if (got.tau0_ns != (uint32_t)tau0_ns) {
		printf("self-test: FAIL tau0 %u != %llu\n",
		       (unsigned int)got.tau0_ns,
		       (unsigned long long)tau0_ns);
		fails++;
	}
	if (got.gaps != 0u) {
		printf("self-test: FAIL gaps %u != 0\n",
		       (unsigned int)got.gaps);
		fails++;
	}

	rc = stats_adev(g_x, n_out, got.tau0_ns, 1u, &adev);
	if (rc != 0) {
		printf("self-test: stats_adev failed (%d)\n", rc);
		return 1;
	}
	rc = stats_mdev(g_x, n_out, got.tau0_ns, 1u, &mdev);
	if (rc != 0) {
		printf("self-test: stats_mdev failed (%d)\n", rc);
		return 1;
	}

	printf("self-test: linear phase, n=%zu tau0=%u ns\n", n_out,
	       (unsigned int)got.tau0_ns);
	printf("self-test: ADEV(m=1) = %.17g\n", adev);
	printf("self-test: MDEV(m=1) = %.17g\n", mdev);

	if (adev != 0.0) {
		printf("self-test: FAIL ADEV of a pure frequency offset must be exactly 0\n");
		fails++;
	}
	if (mdev != 0.0) {
		printf("self-test: FAIL MDEV of a pure frequency offset must be exactly 0\n");
		fails++;
	}

	/* A larger tau must still be exactly 0 — the property is about the
	 * second difference, not about m. */
	rc = stats_adev(g_x, n_out, got.tau0_ns, 64u, &adev);
	if ((rc != 0) || (adev != 0.0)) {
		printf("self-test: FAIL ADEV(m=64) rc=%d value=%.17g\n", rc,
		       adev);
		fails++;
	}

	/* And the refusal gate itself: a record carrying a gap must be
	 * recognised as such, or the tool's one safety property is absent. */
	{
		static uint8_t map[MAX_SAMPLES / 8u];
		size_t start = 0u;

		memset(map, 0, sizeof(map));
		map[1] |= (uint8_t)(1u << 2); /* sample 10 */
		m.gaps = 1u;
		rc = phase_rec_encode(&m, g_x, map, g_buf, sizeof(g_buf), &len);
		if (rc != 0) {
			printf("self-test: gap encode failed (%d)\n", rc);
			return 1;
		}
		rc = phase_rec_decode_hdr(g_buf, len, &got);
		if ((rc != 0) || (got.gaps != 1u)) {
			printf("self-test: FAIL gap record rc=%d gaps=%u\n", rc,
			       (unsigned int)got.gaps);
			fails++;
		}
		if (phase_rec_longest_clean(g_buf, len, &start) != (n - 10u)) {
			printf("self-test: FAIL longest clean run\n");
			fails++;
		}
	}

	fails += saw_self_test();

	printf("self-test: %s\n", (fails == 0) ? "PASS" : "FAIL");
	return (fails == 0) ? 0 : 1;
}

/* ------------------------------------------------------------------ main */

static void usage(void)
{
	fprintf(stderr,
		"usage: meridian_phase [options] <record.phr>\n"
		"       meridian_phase --self-test\n"
		"\n"
		"  --clean-run       analyse the longest gap-free run\n"
		"  --allow-gaps      analyse the whole record anyway\n"
		"  --hist-bin <ps>   histogram bin width (default 1000)\n"
		"  --hist-bins <n>   histogram bins, odd (default 41)\n"
		"  --no-hist         skip the histograms (verdict still shown)\n"
		"  --require-sawtooth\n"
		"                    exit non-zero unless the §14 sawtooth "
		"verdict is CORRECTED\n"
		"  --self-test       check the analysis chain and exit\n");
}

int main(int argc, char **argv)
{
	const char *path = NULL;
	bool clean_run = false;
	bool allow_gaps = false;
	bool want_hist = true;
	bool require_saw = false;
	int64_t bin_ps = 1000;
	size_t n_bins = 41u;
	phase_rec_meta_t m;
	stats_saw_verdict_t verdict;
	size_t len = 0u;
	size_t n_out = 0u;
	size_t n_raw = 0u;
	size_t off = 0u;
	size_t n_use;
	int i;
	int rc;

	for (i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--self-test") == 0) {
			return self_test();
		} else if (strcmp(argv[i], "--clean-run") == 0) {
			clean_run = true;
		} else if (strcmp(argv[i], "--allow-gaps") == 0) {
			allow_gaps = true;
		} else if (strcmp(argv[i], "--no-hist") == 0) {
			want_hist = false;
		} else if (strcmp(argv[i], "--require-sawtooth") == 0) {
			require_saw = true;
		} else if ((strcmp(argv[i], "--hist-bin") == 0) &&
			   ((i + 1) < argc)) {
			bin_ps = (int64_t)strtoll(argv[++i], NULL, 10);
		} else if ((strcmp(argv[i], "--hist-bins") == 0) &&
			   ((i + 1) < argc)) {
			n_bins = (size_t)strtoull(argv[++i], NULL, 10);
		} else if ((strcmp(argv[i], "-h") == 0) ||
			   (strcmp(argv[i], "--help") == 0)) {
			usage();
			return 0;
		} else if (argv[i][0] == '-') {
			fprintf(stderr, "meridian_phase: unknown option %s\n",
				argv[i]);
			usage();
			return 2;
		} else if (path == NULL) {
			path = argv[i];
		} else {
			fprintf(stderr, "meridian_phase: one record at a time\n");
			return 2;
		}
	}

	if (path == NULL) {
		usage();
		return 2;
	}
	if ((bin_ps <= 0) || (n_bins == 0u) || (n_bins > MAX_BINS)) {
		fprintf(stderr, "meridian_phase: bad histogram parameters\n");
		return 2;
	}

	if (read_file(path, &len) != 0) {
		return 1;
	}

	rc = phase_rec_decode_hdr(g_buf, len, &m);
	if (rc != 0) {
		fprintf(stderr,
			"meridian_phase: %s is not a valid phase record (%d)\n",
			path, rc);
		if (rc == -EBADMSG) {
			fprintf(stderr,
				"  (bad magic/version, or the gap count "
				"disagrees with the gap bitmap)\n");
		}
		return 1;
	}

	print_header(&m, path);

	if (m.n == 0u) {
		fprintf(stderr,
			"meridian_phase: the record is empty — the device had "
			"no phase samples yet\n");
		return 1;
	}

	rc = phase_rec_decode_samples(g_buf, len, g_x, MAX_SAMPLES, &n_out);
	if (rc != 0) {
		fprintf(stderr, "meridian_phase: cannot read samples (%d)\n",
			rc);
		return 1;
	}

	n_use = n_out;

	if (m.gaps != 0u) {
		size_t run;
		size_t start = 0u;

		run = phase_rec_longest_clean(g_buf, len, &start);

		fprintf(stderr,
			"\nmeridian_phase: WARNING — this record contains %u "
			"absorbed sample gap(s).\n"
			"  The device's discipline loop tolerates short PPS "
			"gaps rather than\n"
			"  discarding its ring, so these samples are NOT one "
			"tau0 apart. ADEV\n"
			"  and MDEV assume uniform spacing and cannot detect "
			"the difference:\n"
			"  the whole-record answer would be wrong and would "
			"look correct.\n"
			"  Longest uniformly spaced run: %zu samples starting "
			"at index %zu.\n",
			(unsigned int)m.gaps, run, start);

		/*
		 * Say WHERE. "two gaps somewhere in 512 samples" leaves the
		 * operator no way to correlate them with what was happening on
		 * the bench; two sample indices do. Capped so a badly broken
		 * capture cannot bury the advice below it.
		 */
		{
			size_t shown = 0u;
			size_t i;

			fprintf(stderr, "  gap at sample:");
			for (i = 0u; i < n_out; i++) {
				if (!phase_rec_gap_at(g_buf, len, i)) {
					continue;
				}
				if (shown == 16u) {
					fprintf(stderr, " ...");
					break;
				}
				fprintf(stderr, " %zu", i);
				shown++;
			}
			(void)fputc('\n', stderr);
		}

		if (clean_run) {
			if (run < 3u) {
				fprintf(stderr,
					"meridian_phase: the longest clean run "
					"is too short to analyse\n");
				return 1;
			}
			off = start;
			n_use = run;
			fprintf(stderr,
				"  --clean-run: analysing samples [%zu, %zu).\n",
				start, start + run);
		} else if (allow_gaps) {
			fprintf(stderr,
				"  --allow-gaps: analysing the whole record "
				"anyway. The numbers below\n"
				"  are NOT a valid stability estimate.\n");
		} else {
			fprintf(stderr,
				"\nmeridian_phase: refusing to analyse. Re-run "
				"with --clean-run to use\n"
				"  the longest uniformly spaced stretch, or "
				"--allow-gaps to override.\n");
			return 1;
		}
	}

	printf("analysed     : %zu samples", n_use);
	if (off != 0u) {
		printf(" from index %zu", off);
	}
	(void)putchar('\n');

	print_dev_table(&g_x[off], n_use, (uint64_t)m.tau0_ns);

	if ((m.flags & PHASE_REC_F_RAW) == 0u) {
		if (want_hist) {
			print_hist("sawtooth-corrected", &g_x[off], n_use,
				   bin_ps, n_bins);
		}
		fprintf(stderr,
			"\nmeridian_phase: this record carries only the "
			"corrected series, so the\n"
			"  spec §14 sawtooth proof cannot be made from it. A "
			"single histogram\n"
			"  cannot distinguish a working qErr path from one "
			"whose sign is inverted:\n"
			"  both are single-peaked and the difference is a width "
			"with nothing to be\n"
			"  compared against. Re-export from firmware that "
			"writes PHASE_REC_F_RAW.\n");
		return require_saw ? 1 : 0;
	}

	rc = phase_rec_decode_raw(g_buf, len, g_raw, MAX_SAMPLES, &n_raw);
	if ((rc != 0) || (n_raw != n_out)) {
		fprintf(stderr,
			"meridian_phase: cannot read the uncorrected series "
			"(%d, n=%zu vs %zu)\n",
			rc, n_raw, n_out);
		return 1;
	}

	if (want_hist) {
		/*
		 * Uncorrected first: it is the "before". Both use the same bin
		 * width so the two tables are read against each other rather
		 * than each against its own auto-scale — a comparison drawn on
		 * two different axes is exactly the way to miss a doubling.
		 */
		print_hist("WITHOUT sawtooth correction", &g_raw[off], n_use,
			   bin_ps, n_bins);
		print_hist("WITH sawtooth correction", &g_x[off], n_use, bin_ps,
			   n_bins);
	}

	verdict = print_saw(&g_x[off], &g_raw[off], n_use);

	if (require_saw && (verdict != STATS_SAW_CORRECTED)) {
		fprintf(stderr,
			"\nmeridian_phase: --require-sawtooth: verdict is %s, "
			"not CORRECTED\n",
			stats_saw_str(verdict));
		return 1;
	}

	return 0;
}
