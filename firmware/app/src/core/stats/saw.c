/*
 * STS1000 "Meridian" — core/stats: the spec §14 sawtooth-correction verdict.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See saw.h for the contract and the derivation of z.
 */

#include "stats/saw.h"

#include <errno.h>
#include <string.h>

#include "stats/adev.h" /* stats_sqrt() — core/ has no libm */

/*
 * Verdict boundaries on z, placed at the midpoints between the three
 * theoretical landmarks (-1 correct, +1 uncorrelated, +3 inverted) rather than
 * at tuned values. Midpoints are the only choice that does not privilege one
 * hypothesis over another, and they need no board-specific calibration.
 */
#define Z_CORRECTED_MAX 0.0 /* midpoint of -1 and +1 */
#define Z_INVERTED_MIN  2.0 /* midpoint of +1 and +3 */

const char *stats_saw_str(stats_saw_verdict_t v)
{
	switch (v) {
	case STATS_SAW_NONE:
		return "NOT-APPLIED";
	case STATS_SAW_CORRECTED:
		return "CORRECTED";
	case STATS_SAW_INEFFECTIVE:
		return "INEFFECTIVE";
	case STATS_SAW_INVERTED:
		return "SIGN-INVERTED";
	default:
		return "UNKNOWN";
	}
}

int stats_saw_verdict(const int64_t *corr_ps, const int64_t *raw_ps, size_t n,
		      stats_saw_t *out)
{
	double mean_c;
	double mean_r;
	double mean_s;
	double var_c;
	double var_r;
	double var_s;
	double cov_rs = 0.0;
	double beta;
	double resid = 0.0;
	size_t i;

	if ((corr_ps == NULL) || (raw_ps == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (n < 2u) {
		return -ENODATA;
	}

	memset(out, 0, sizeof(*out));
	out->n = n;
	out->spread_ratio = -1.0;

	/*
	 * Accumulate in double straight off the int64 series. Both are bounded
	 * by phase_rec.c's PS_SAT_LIMIT (1e15), so the difference cannot
	 * overflow int64 — but it is formed in double anyway, because that is
	 * the type every statistic below is computed in and one conversion
	 * point is easier to reason about than three.
	 */
	{
		double sum_c = 0.0;
		double sum_r = 0.0;
		double sum_s = 0.0;

		for (i = 0u; i < n; i++) {
			double c = (double)corr_ps[i];
			double r = (double)raw_ps[i];

			sum_c += c;
			sum_r += r;
			sum_s += c - r;
			if (corr_ps[i] != raw_ps[i]) {
				out->n_applied++;
			}
		}
		mean_c = sum_c / (double)n;
		mean_r = sum_r / (double)n;
		mean_s = sum_s / (double)n;
	}

	var_c = 0.0;
	var_r = 0.0;
	var_s = 0.0;
	for (i = 0u; i < n; i++) {
		double dc = (double)corr_ps[i] - mean_c;
		double dr = (double)raw_ps[i] - mean_r;
		double ds = ((double)corr_ps[i] - (double)raw_ps[i]) - mean_s;

		var_c += dc * dc;
		var_r += dr * dr;
		var_s += ds * ds;
		cov_rs += dr * ds;
	}
	var_c /= (double)n;
	var_r /= (double)n;
	var_s /= (double)n;
	cov_rs /= (double)n;

	out->mean_corr_ps = mean_c;
	out->mean_raw_ps = mean_r;
	out->mean_shift_ps = mean_c - mean_r;
	out->sdev_corr_ps = stats_sqrt(var_c);
	out->sdev_raw_ps = stats_sqrt(var_r);
	out->sdev_saw_ps = stats_sqrt(var_s);
	if (var_r > 0.0) {
		out->spread_ratio = out->sdev_corr_ps / out->sdev_raw_ps;
	}

	/*
	 * No correction to judge. n_applied == 0 and var_s == 0 are not the
	 * same condition — a constant non-zero correction on every sample moves
	 * the mean without changing any spread — so both are checked, and both
	 * mean there is no evidence here about the qErr path's sign.
	 *
	 * This is also the branch that catches the "uncorrected series is a
	 * copy of the corrected one" failure: a duplicated series makes every
	 * s[i] exactly 0, and the answer is NOT-APPLIED rather than a clean
	 * bill of health.
	 */
	if ((out->n_applied == 0u) || (var_s <= 0.0)) {
		out->verdict = STATS_SAW_NONE;
		return 0;
	}

	/* z = (var(c) - var(r)) / var(s). Identically 1 + 2*cov(r,s)/var(s);
	 * computed from the variances so it is the quantity the operator can
	 * check against the printed spreads. */
	out->z = (var_c - var_r) / var_s;

	/*
	 * Uncertainty. Regress r on s: the part of r that s does not explain is
	 * what makes z noisy, and its size relative to var(s) is exactly the
	 * standard error of the slope. z = 1 + 2*beta, so sigma_z = 2*sigma_beta.
	 *
	 * n - 2 degrees of freedom (mean and slope both estimated). n >= 2 is
	 * guaranteed above; at n == 2 the fit is exact, the residual is 0 and
	 * sigma is 0 — which is honest: two points say nothing, and `marginal`
	 * is not the field that reports that. n is.
	 */
	beta = cov_rs / var_s;
	for (i = 0u; i < n; i++) {
		double dr = (double)raw_ps[i] - mean_r;
		double ds = ((double)corr_ps[i] - (double)raw_ps[i]) - mean_s;
		double e = dr - (beta * ds);

		resid += e * e;
	}
	if (n > 2u) {
		double s2 = resid / (double)(n - 2u);
		double denom = (double)n * var_s;

		out->z_sigma = 2.0 * stats_sqrt(s2 / denom);
	}

	if (out->z < Z_CORRECTED_MAX) {
		out->verdict = STATS_SAW_CORRECTED;
	} else if (out->z < Z_INVERTED_MIN) {
		out->verdict = STATS_SAW_INEFFECTIVE;
	} else {
		out->verdict = STATS_SAW_INVERTED;
	}

	{
		double d0 = out->z - Z_CORRECTED_MAX;
		double d2 = out->z - Z_INVERTED_MIN;
		double two = 2.0 * out->z_sigma;

		if (d0 < 0.0) {
			d0 = -d0;
		}
		if (d2 < 0.0) {
			d2 = -d2;
		}
		out->marginal = (d0 < two) || (d2 < two);
	}

	return 0;
}
