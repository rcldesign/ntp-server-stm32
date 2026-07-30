/*
 * STS1000 "Meridian" — core/stats implementation.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See adev.h for the contract, the arithmetic rationale and the reachability
 * classification.
 */

#include "stats/adev.h"

#include <errno.h>
#include <string.h>

/* Picoseconds per second, as the double the estimators divide by. */
#define PS_PER_S 1.0e12

/* Nanoseconds per second. */
#define NS_PER_S 1.0e9

/* --------------------------------------------------------------- sqrt */

double stats_sqrt(double x)
{
	uint64_t bits;
	double y;
	double scale = 1.0;
	int i;

	/* Written as !(x > 0) so NaN and -0.0 land here too, matching
	 * quality_sqrtf(). */
	if (!(x > 0.0)) {
		return 0.0;
	}

	/* isinf() without <math.h>: an all-ones exponent field. NaN shares that
	 * encoding and has already been rejected above, so what reaches here is
	 * +infinity, whose square root is itself. */
	memcpy(&bits, &x, sizeof(bits));
	if ((bits & UINT64_C(0x7FF0000000000000)) ==
	    UINT64_C(0x7FF0000000000000)) {
		return x;
	}

	/* Subnormals defeat the exponent-halving seed, so scale them into the
	 * normal range first and undo the scale afterwards. 2^600 lifts even
	 * DBL_TRUE_MIN (4.9e-324) to 1.9e-143. */
	if (x < 1.0e-300) {
		x *= 0x1p600;
		scale = 0x1p-300;
	}

	/* Seed: halving the biased exponent approximates the square root to a
	 * few per cent. Adding (1023 << 51) restores the bias that the shift
	 * halved — (e - 1023)/2 + 1023 = e/2 + 511.5, and 511.5 in exponent
	 * units is 1023 at bit 51. */
	memcpy(&bits, &x, sizeof(bits));
	bits = (bits >> 1) + (UINT64_C(1023) << 51);
	memcpy(&y, &bits, sizeof(y));

	/* Newton–Raphson: each step squares the relative error.
	 * 6e-2 -> 1.8e-3 -> 1.6e-6 -> 1.3e-12 -> ulp. The fifth step is
	 * margin, and costs nothing anyone will measure — this runs once per
	 * plotted point, not once per sample. */
	for (i = 0; i < 5; i++) {
		y = 0.5 * (y + x / y);
	}

	return y * scale;
}

/* ------------------------------------------------------------- internals */

/**
 * Reject a record that would break the exactness guarantee in adev.h.
 *
 * Checked once per estimator call rather than assumed. The scan is O(n) against
 * an estimator that is also O(n), so it costs a constant factor and buys the
 * right to form every difference in int64 without a per-sample overflow test.
 */
static int phase_range_ok(const int64_t *x_ps, size_t n)
{
	size_t i;

	for (i = 0u; i < n; i++) {
		if (x_ps[i] > STATS_PHASE_MAX_PS ||
		    x_ps[i] < -STATS_PHASE_MAX_PS) {
			return -ERANGE;
		}
	}
	return 0;
}

/**
 * Second difference x[i+2m] - 2x[i+m] + x[i], exact in int64.
 *
 * Bounded by 4 * STATS_PHASE_MAX_PS = 4e12 once phase_range_ok() has passed,
 * which is inside int64 and inside the 2^53 exactly-representable double range.
 */
static int64_t second_diff(const int64_t *x_ps, size_t i, size_t m)
{
	return x_ps[i + 2u * m] - 2 * x_ps[i + m] + x_ps[i];
}

/* Shared argument checks for both estimators. */
static int dev_args_ok(const int64_t *x_ps, uint64_t tau0_ns, uint32_t m,
		       const double *out)
{
	if (x_ps == NULL || out == NULL) {
		return -EINVAL;
	}
	if (m == 0u || m > STATS_MAX_M) {
		return -EINVAL;
	}
	if (tau0_ns == 0u) {
		return -EINVAL;
	}
	return 0;
}

/* ------------------------------------------------------------ estimators */

uint32_t stats_max_m(size_t n, stats_dev_t kind)
{
	size_t m;

	if (kind == STATS_DEV_ADEV) {
		/* n >= 2m + 1 */
		if (n < 3u) {
			return 0u;
		}
		m = (n - 1u) / 2u;
	} else {
		/* n >= 3m */
		if (n < 3u) {
			return 0u;
		}
		m = n / 3u;
	}

	if (m > (size_t)STATS_MAX_M) {
		m = (size_t)STATS_MAX_M;
	}
	return (uint32_t)m;
}

int stats_adev(const int64_t *x_ps, size_t n, uint64_t tau0_ns, uint32_t m,
	       double *out)
{
	size_t mz;
	size_t terms;
	size_t i;
	double acc = 0.0;
	double md;
	double tau0_s;
	double denom;
	int rc;

	rc = dev_args_ok(x_ps, tau0_ns, m, out);
	if (rc != 0) {
		return rc;
	}

	mz = (size_t)m;
	if (n < 2u * mz + 1u) {
		return -ENODATA;
	}

	rc = phase_range_ok(x_ps, n);
	if (rc != 0) {
		return rc;
	}

	terms = n - 2u * mz;
	for (i = 0u; i < terms; i++) {
		/* Overlapping: the stride is 1. A stride of m here would be the
		 * non-overlapping estimator — same units, same slope, a factor
		 * of m fewer terms and error bars wide enough to hide the
		 * OCXO/rubidium difference at large tau. */
		double d = (double)second_diff(x_ps, i, mz);

		acc += d * d;
	}

	md = (double)m;
	tau0_s = (double)tau0_ns / NS_PER_S;
	denom = 2.0 * md * md * tau0_s * tau0_s * (double)terms;

	/* Phase is in picoseconds, so the dimensionless deviation carries the
	 * 1e-12. Applied to the deviation, not the variance, so the scaling
	 * cannot underflow a small variance to zero. */
	*out = stats_sqrt(acc / denom) / PS_PER_S;
	return 0;
}

int stats_mdev(const int64_t *x_ps, size_t n, uint64_t tau0_ns, uint32_t m,
	       double *out)
{
	size_t mz;
	size_t outer;
	size_t i;
	size_t j;
	int64_t win = 0;
	double acc = 0.0;
	double md;
	double md2;
	double tau0_s;
	double denom;
	int rc;

	rc = dev_args_ok(x_ps, tau0_ns, m, out);
	if (rc != 0) {
		return rc;
	}

	mz = (size_t)m;
	if (n < 3u * mz) {
		return -ENODATA;
	}

	rc = phase_range_ok(x_ps, n);
	if (rc != 0) {
		return rc;
	}

	/* Outer index j runs 0 .. n-3m, so there are n-3m+1 window sums; the
	 * window [j, j+m-1] of second differences needs j+m-1 <= n-2m-1, which
	 * is the same bound. */
	outer = n - 3u * mz + 1u;

	/* Seed the first window, then slide it: S(j+1) = S(j) + d[j+m] - d[j].
	 * O(n) rather than O(n*m), and exact — STATS_MAX_M is chosen so that
	 * |S| <= m * 4 * STATS_PHASE_MAX_PS stays inside int64. */
	for (i = 0u; i < mz; i++) {
		win += second_diff(x_ps, i, mz);
	}
	acc = (double)win * (double)win;

	for (j = 1u; j < outer; j++) {
		win += second_diff(x_ps, j + mz - 1u, mz);
		win -= second_diff(x_ps, j - 1u, mz);
		acc += (double)win * (double)win;
	}

	md = (double)m;
	md2 = md * md;
	tau0_s = (double)tau0_ns / NS_PER_S;

	/* m^4: m^2 from tau^2 = (m*tau0)^2, and m^2 from the m-sample phase
	 * average that the inner sum leaves unnormalised. At m == 1 both
	 * collapse and this is exactly the Allan variance, as it must be. */
	denom = 2.0 * md2 * md2 * tau0_s * tau0_s * (double)outer;

	*out = stats_sqrt(acc / denom) / PS_PER_S;
	return 0;
}

int stats_dev_sweep(stats_dev_t kind, const int64_t *x_ps, size_t n,
		    uint64_t tau0_ns, const uint32_t *m_list, size_t n_m,
		    double *out_dev, size_t *out_n)
{
	size_t k;
	int rc;

	if (out_n == NULL) {
		return -EINVAL;
	}
	*out_n = 0u;

	if (x_ps == NULL || m_list == NULL || out_dev == NULL || n_m == 0u) {
		return -EINVAL;
	}
	if (tau0_ns == 0u) {
		return -EINVAL;
	}
	if (kind != STATS_DEV_ADEV && kind != STATS_DEV_MDEV) {
		return -EINVAL;
	}

	/* Validate the whole factor list before computing anything: a caller
	 * that passes a malformed axis should learn that from the return code,
	 * not from a partly-filled output array it has already started
	 * plotting. */
	for (k = 0u; k < n_m; k++) {
		if (m_list[k] == 0u || m_list[k] > STATS_MAX_M) {
			return -EINVAL;
		}
		if (k > 0u && m_list[k] <= m_list[k - 1u]) {
			return -EINVAL;
		}
	}

	rc = phase_range_ok(x_ps, n);
	if (rc != 0) {
		return rc;
	}

	for (k = 0u; k < n_m; k++) {
		double v;

		if (kind == STATS_DEV_ADEV) {
			rc = stats_adev(x_ps, n, tau0_ns, m_list[k], &v);
		} else {
			rc = stats_mdev(x_ps, n, tau0_ns, m_list[k], &v);
		}

		if (rc == -ENODATA) {
			/* The factors ascend, so nothing after this one fits
			 * either. Stop and report the prefix. */
			break;
		}
		if (rc != 0) {
			return rc;
		}

		out_dev[k] = v;
		*out_n = k + 1u;
	}

	return 0;
}

size_t stats_m_octaves(size_t n, stats_dev_t kind, uint32_t *out_m, size_t cap)
{
	uint32_t max_m = stats_max_m(n, kind);
	uint32_t m = 1u;
	size_t count = 0u;

	if (max_m == 0u) {
		return 0u;
	}

	while (m <= max_m) {
		if (out_m != NULL && count < cap) {
			out_m[count] = m;
		}
		count++;

		/* max_m <= STATS_MAX_M < 2^31, so this cannot wrap. */
		m *= 2u;
	}

	return count;
}

/* ------------------------------------------------------------- histogram */

int stats_hist(const int64_t *resid_ps, size_t n, int64_t lo_ps, int64_t bin_ps,
	       uint32_t *bins, size_t n_bins, stats_hist_t *out)
{
	size_t i;
	int64_t sum = 0;
	int64_t span = 0;
	int64_t hi_ps = 0;
	double sum_sq = 0.0;
	double nd;
	double mean;
	double meansq;
	double var;

	if (resid_ps == NULL || out == NULL || n == 0u) {
		return -EINVAL;
	}
	if (bins != NULL && (n_bins == 0u || bin_ps <= 0)) {
		return -EINVAL;
	}

	/* The mean is accumulated in int64 so that a record with an exactly
	 * representable mean reports it exactly. That holds only while
	 * n * STATS_PHASE_MAX_PS fits, which is 9.2e6 samples — 106 days of
	 * 1 Hz PPS. Beyond that the caller must split the record rather than be
	 * handed a wrapped sum. */
	if (n > (size_t)(INT64_MAX / STATS_PHASE_MAX_PS)) {
		return -ERANGE;
	}

	if (bins != NULL) {
		/* |resid - lo| must stay inside int64; bounding lo by the same
		 * limit as the samples caps it at 2e12. */
		if (lo_ps > STATS_PHASE_MAX_PS || lo_ps < -STATS_PHASE_MAX_PS) {
			return -ERANGE;
		}
		if (n_bins > (size_t)INT64_MAX ||
		    (int64_t)n_bins > INT64_MAX / bin_ps) {
			return -ERANGE;
		}
		span = (int64_t)n_bins * bin_ps;
		if (lo_ps > INT64_MAX - span) {
			return -ERANGE;
		}
		hi_ps = lo_ps + span;

		memset(bins, 0, n_bins * sizeof(bins[0]));
	}

	memset(out, 0, sizeof(*out));
	out->min_ps = INT64_MAX;
	out->max_ps = INT64_MIN;

	for (i = 0u; i < n; i++) {
		int64_t v = resid_ps[i];
		double vd;

		if (v > STATS_PHASE_MAX_PS || v < -STATS_PHASE_MAX_PS) {
			return -ERANGE;
		}

		if (v < out->min_ps) {
			out->min_ps = v;
		}
		if (v > out->max_ps) {
			out->max_ps = v;
		}

		sum += v;
		vd = (double)v;
		sum_sq += vd * vd;

		if (bins == NULL) {
			continue;
		}

		/* Out-of-window samples are counted separately but still feed
		 * every statistic above: the summary describes the data, not
		 * the window chosen to draw it. */
		if (v < lo_ps) {
			out->under++;
		} else if (v >= hi_ps) {
			out->over++;
		} else {
			size_t idx = (size_t)((uint64_t)(v - lo_ps) /
					      (uint64_t)bin_ps);

			bins[idx]++;
			out->in_range++;
		}
	}

	out->n = n;
	out->p2p_ps = out->max_ps - out->min_ps;

	nd = (double)n;
	mean = (double)sum / nd;
	meansq = sum_sq / nd;

	/* var = E[x^2] - E[x]^2 is the cheap form and the numerically poor one
	 * when the mean dominates the spread — which is exactly the §14 case, a
	 * few-hundred-picosecond jitter riding a cable-delay bias. It is safe
	 * here only because both moments are accumulated exactly in the integer
	 * domain first: sum is int64-exact, and sum_sq is a sum of exactly
	 * representable products. What remains is a single cancellation at the
	 * end, clamped at zero so a last-ULP negative can never reach sqrt(). */
	var = meansq - mean * mean;
	if (var < 0.0) {
		var = 0.0;
	}

	out->mean_ps = mean;
	out->rms_ps = stats_sqrt(meansq);
	out->sdev_ps = stats_sqrt(var);
	return 0;
}
