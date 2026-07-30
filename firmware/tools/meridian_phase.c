/*
 * STS1000 "Meridian" — meridian_phase: the bench stability analyser.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Reads a phase record exported from the device (MCP PHASE_EXPORT 0x24, fetched
 * with `meridian_ctl.py phase-export`) and prints the spec §14 analysis: ADEV
 * and MDEV against tau, and the PPS residual histogram.
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
 * Usage:
 *   meridian_phase [options] <record.phr>
 *   meridian_phase --self-test
 *
 * Options:
 *   --clean-run       analyse the longest gap-free run instead of refusing
 *   --allow-gaps      analyse the whole record anyway (prints a warning)
 *   --hist-bin <ps>   histogram bin width, picoseconds (default 1000 = 1 ns)
 *   --hist-bins <n>   number of histogram bins, odd, centred (default 41)
 *   --no-hist         skip the histogram
 *   --self-test       analyse a synthetic exactly-linear record and check that
 *                     ADEV is exactly 0, then exit. Proves the analysis chain
 *                     on this machine before it is trusted on real data.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stats/adev.h"
#include "stats/phase_rec.h"

/* The device ring is 512 samples; allow well beyond it so a concatenated or
 * future longer record still loads rather than being silently truncated. */
#define MAX_SAMPLES 262144u
#define MAX_FILE_BYTES (32u + (MAX_SAMPLES / 8u) + (MAX_SAMPLES * 8u))

#define MAX_OCTAVES 64u
#define MAX_BINS 1024u

static int64_t g_x[MAX_SAMPLES];
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

static void print_hist(const int64_t *x, size_t n, int64_t bin_ps,
		       size_t n_bins)
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

	printf("\nPPS residual summary (picoseconds)\n");
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
		"  --no-hist         skip the histogram\n"
		"  --self-test       check the analysis chain and exit\n");
}

int main(int argc, char **argv)
{
	const char *path = NULL;
	bool clean_run = false;
	bool allow_gaps = false;
	bool want_hist = true;
	int64_t bin_ps = 1000;
	size_t n_bins = 41u;
	phase_rec_meta_t m;
	size_t len = 0u;
	size_t n_out = 0u;
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
	if (want_hist) {
		print_hist(&g_x[off], n_use, bin_ps, n_bins);
	}

	return 0;
}
