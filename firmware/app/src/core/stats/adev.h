/*
 * STS1000 "Meridian" — core/stats: frequency-stability estimators for the
 * spec §14 timing-validation record.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation, no platform headers, no globals.
 *
 * WHAT THIS IS, AND WHAT IT IS NOT A DUPLICATE OF
 *
 * core/disc already ships an overlapping Allan deviation: disc_adev_overlapping()
 * runs once a second over a bounded float ring and publishes adev_1s/10s/100s
 * into the quality block. That is *telemetry* — three fixed averaging factors,
 * single precision, a window sized by the RAM the discipline thread can spare.
 *
 * Spec §14 asks for something different: "ADEV/MDEV vs tau for OCXO and Rb
 * (locked and holdover)" and "PPS residual histograms with/without sawtooth
 * correction". That is *validation* — an arbitrary tau decade sweep over a
 * captured record of arbitrary length, MDEV as well as ADEV (the runtime has no
 * MDEV at all), and enough arithmetic headroom that the answer is limited by the
 * measurement rather than by the estimator. This module is that estimator. It
 * deliberately does not touch core/disc: the runtime path is on a 1 Hz thread
 * budget and changing it to serve a bench plot would be the wrong trade.
 *
 * The measurement half of §14 needs a board and a better reference. The
 * arithmetic half does not, and it is the half that can be wrong silently: an
 * off-by-one in the overlap stride or a dropped factor of two produces a plot
 * that looks entirely plausible and is wrong by 40 %. Everything here therefore
 * has a closed-form answer that tests/host/test_stats.c checks.
 *
 * ARITHMETIC CHOICE — INTEGER DIFFERENCES, DOUBLE ACCUMULATION
 *
 * The rest of core/ is integer arithmetic because it runs in the servo and ISR
 * paths, where determinism and the absence of an FPU-dependent code path matter.
 * That reasoning does not transfer here, and the opposite reasoning applies:
 *
 *   * The second difference x[i+2m] - 2x[i+m] + x[i] is computed in **int64**,
 *     so it is exact. This is what makes the sharpest test possible: for a pure
 *     frequency offset (x linear in i) every second difference is *identically*
 *     zero, so the deviation is exactly 0.0 with no tolerance — a float
 *     subtraction of large nearly-equal phases would leave cancellation noise
 *     and force the test to accept a small non-zero answer, which is exactly the
 *     tolerance a sign error or a stride error hides behind.
 *
 *   * The sums of squares accumulate in **double**. AVAR is a ratio of sums of
 *     squares over a record that can be 10^6 samples long and a tau sweep that
 *     spans five decades; the variance therefore spans ten. A phase difference
 *     of 4e12 ps squares to 1.6e25 and sums to ~1e31 — outside int64 (9.2e18)
 *     and outside float (24-bit mantissa loses a summand once the accumulator is
 *     2^24 times larger than it). double's 53-bit mantissa and 10^308 range
 *     cover it with room to spare. core/disc uses double accumulators for its
 *     offline tempco fit for the same reason (disc.c, disc_tempco_fit).
 *
 *   * MDEV's inner window sum runs as an **int64 running sum** (O(N) instead of
 *     O(N*m)) and is therefore also exact; STATS_MAX_M bounds it away from
 *     overflow rather than assuming it cannot happen.
 *
 * The price is a soft-float dependency if this ever ran on the target: the
 * Cortex-M33 FPU here is FPv5-*SP*, so double is emulated. That price is not
 * paid, because nothing on the target calls this — see REACHABILITY below.
 *
 * NO libm. core/quality states the rule ("There is deliberately no libm
 * dependency anywhere in core/") and the host harness enforces it by not linking
 * -lm. quality_sqrtf() is the float precedent; stats_sqrt() below is the double
 * one, and it is public rather than static so its accuracy is asserted directly
 * instead of inferred.
 *
 * REACHABILITY
 *
 * Every symbol here is absent from zephyr.elf and carries a `no-caller` entry in
 * scripts/reachability.allow. That is a deliberate classification, not an
 * oversight: this is bench-validation arithmetic driven from the host against an
 * exported record, and the runtime's own stability telemetry is core/disc's.
 * Binding these into the image would put a double-precision multi-decade sweep
 * on a 250 MHz Cortex-M33 with no FPU support for it, to compute a number no
 * on-board consumer reads.
 *
 * UNITS
 *
 *   phase / residual   int64 picoseconds. Matches disc_pulse_t::qerr_ps, so a
 *                      sawtooth-corrected record needs no rescaling.
 *   tau0               uint64 nanoseconds (1 PPS => 1000000000).
 *   deviation          dimensionless (fractional frequency), as ADEV always is.
 */

#ifndef STS1000_CORE_STATS_ADEV_H_
#define STS1000_CORE_STATS_ADEV_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- limits */

/**
 * Largest phase magnitude accepted, picoseconds (1 s).
 *
 * The bound is what makes the int64 arithmetic provably exact rather than
 * probably exact. |x| <= 1e12 ps gives |x[i+2m] - 2x[i+m] + x[i]| <= 4e12, which
 * is both inside int64 and inside the 2^53 range that converts to double without
 * rounding.
 *
 * It is not a restriction in practice: a phase record whose residual exceeds one
 * whole second is not a stability record, it is a record of a clock that is not
 * running. Records are refused rather than silently truncated, because a
 * saturated sample produces a plausible-looking ADEV.
 */
#define STATS_PHASE_MAX_PS INT64_C(1000000000000)

/**
 * Largest averaging factor accepted.
 *
 * Bounded by MDEV's exact int64 window sum: |sum| <= m * 4 * STATS_PHASE_MAX_PS
 * must stay inside INT64_MAX, so m <= 2^63-1 / 4e12 = 2305843. ADEV needs no
 * such bound but shares it, so one limit describes the module.
 *
 * At tau0 = 1 s this is tau = 26.7 days, and MDEV at that tau needs 3m = 80 days
 * of record. The limit therefore never binds on a real capture; it exists so
 * that "never binds" is checked rather than asserted.
 */
#define STATS_MAX_M UINT32_C(2305843)

/* ------------------------------------------------------------------ types */

/** Which deviation a sweep computes. */
typedef enum {
	STATS_DEV_ADEV = 0, /**< overlapping Allan deviation */
	STATS_DEV_MDEV,     /**< modified Allan deviation */
} stats_dev_t;

/**
 * PPS residual histogram result (spec §14 "PPS residual histograms").
 *
 * `rms_ps` and `sdev_ps` are both present and both wanted: they are related by
 * rms^2 = mean^2 + sdev^2, so the pair separates a *bias* from a *spread*. That
 * separation is the point of the §14 comparison — sawtooth correction removes a
 * spread the receiver knows about and leaves any residual cable/antenna bias
 * untouched, so a qErr path that works shows up as sdev collapsing while mean
 * holds still, and a qErr path with an inverted sign shows up as sdev growing.
 */
typedef struct {
	size_t n;         /**< samples examined */
	size_t in_range;  /**< samples that landed in a bin */
	size_t under;     /**< samples below the first bin's lower edge */
	size_t over;      /**< samples at or above the last bin's upper edge */
	int64_t min_ps;   /**< smallest residual seen */
	int64_t max_ps;   /**< largest residual seen */
	int64_t p2p_ps;   /**< max_ps - min_ps */
	double mean_ps;   /**< arithmetic mean */
	double rms_ps;    /**< sqrt(mean(x^2)) — about zero */
	double sdev_ps;   /**< sqrt(mean((x-mean)^2)) — population, about the mean */
} stats_hist_t;

/* ------------------------------------------------------------------- math */

/**
 * Square root, double precision, with no libm dependency.
 *
 * Same construction as quality_sqrtf(): seed from the halved biased exponent,
 * then Newton–Raphson, each step squaring the relative error. Accurate to within
 * a few ULP across the whole normal range, which is far beyond what a deviation
 * needs — it is written this way so that the estimators' asserted coefficients
 * (see test_stats.c) are testing the estimator and not the square root.
 *
 * @param x  Radicand.
 * @return   sqrt(x); 0.0 for x <= 0, -0.0 and NaN; +inf for +inf.
 */
double stats_sqrt(double x);

/* ------------------------------------------------------------- estimators */

/**
 * Largest averaging factor a record of @p n samples supports.
 *
 * ADEV needs n >= 2m+1 (the last second difference must exist), MDEV needs
 * n >= 3m (the last window of m second differences must exist). Both are also
 * capped at STATS_MAX_M.
 *
 * @param n     Sample count.
 * @param kind  Which estimator.
 * @return      Largest usable m, or 0 if the record supports none.
 */
uint32_t stats_max_m(size_t n, stats_dev_t kind);

/**
 * Overlapping Allan deviation at tau = m * tau0 (NIST SP 1065 §5.2.4).
 *
 *   sigma_y^2(tau) = SUM(i=0..n-2m-1) (x[i+2m] - 2x[i+m] + x[i])^2
 *                    / (2 * m^2 * tau0^2 * (n - 2m))
 *
 * "Overlapping" is the stride: i advances by 1, not by m. Every sample
 * participates in every estimate, which is what buys the confidence that makes a
 * large-tau point worth plotting at all — the non-overlapping estimator throws
 * away a factor of m of the record and its error bars at the right-hand end of a
 * §14 plot are wide enough to hide a real difference between the OCXO and the
 * rubidium.
 *
 * The second difference annihilates any constant phase offset and any constant
 * frequency offset, so what is measured is the residual instability, not the
 * (steered) rate. A record that is exactly linear in i therefore returns exactly
 * 0.0, and that is a guarantee of this implementation, not an approximation:
 * the differences are formed in int64.
 *
 * @param x_ps     Uniformly spaced phase samples, picoseconds, oldest first.
 * @param n        Number of samples.
 * @param tau0_ns  Sample interval, nanoseconds. Must be non-zero.
 * @param m        Averaging factor; tau = m * tau0. Must be non-zero.
 * @param out      Receives the dimensionless deviation.
 *
 * @retval 0        Success.
 * @retval -EINVAL  NULL argument, m == 0, m > STATS_MAX_M, or tau0_ns == 0.
 * @retval -ENODATA Fewer than 2m+1 samples — the estimate is undefined.
 * @retval -ERANGE  Some |x_ps[i]| exceeds STATS_PHASE_MAX_PS.
 */
int stats_adev(const int64_t *x_ps, size_t n, uint64_t tau0_ns, uint32_t m,
	       double *out);

/**
 * Modified Allan deviation at tau = m * tau0 (NIST SP 1065 §5.2.5).
 *
 *   Mod sigma_y^2(tau) = SUM(j=0..n-3m) [ SUM(i=j..j+m-1)
 *                            (x[i+2m] - 2x[i+m] + x[i]) ]^2
 *                        / (2 * m^4 * tau0^2 * (n - 3m + 1))
 *
 * The extra phase average over m samples is a low-pass on the *measurement*
 * bandwidth, and it is the whole reason §14 names MDEV separately from ADEV:
 * white PM and flicker PM share an ADEV slope of tau^-1 and are indistinguishable
 * on an ADEV plot, whereas MDEV separates them cleanly (tau^-3/2 against
 * tau^-1). On this board that is the difference between "the PPS input stage is
 * quantisation-limited" and "something is flickering", which are different
 * repairs.
 *
 * MDEV and ADEV coincide at m == 1 by construction — the m^4 in the denominator
 * is m^2 for tau^2 and m^2 for the averaging, and at m = 1 both are 1.
 *
 * The inner window sum is carried as an O(1) running update, so cost is O(n)
 * rather than O(n*m); a five-decade sweep over a week of 1 Hz data stays
 * interactive.
 *
 * @param x_ps     Uniformly spaced phase samples, picoseconds, oldest first.
 * @param n        Number of samples.
 * @param tau0_ns  Sample interval, nanoseconds. Must be non-zero.
 * @param m        Averaging factor; tau = m * tau0. Must be non-zero.
 * @param out      Receives the dimensionless deviation.
 *
 * @retval 0        Success.
 * @retval -EINVAL  NULL argument, m == 0, m > STATS_MAX_M, or tau0_ns == 0.
 * @retval -ENODATA Fewer than 3m samples — the estimate is undefined.
 * @retval -ERANGE  Some |x_ps[i]| exceeds STATS_PHASE_MAX_PS.
 */
int stats_mdev(const int64_t *x_ps, size_t n, uint64_t tau0_ns, uint32_t m,
	       double *out);

/**
 * Sweep one estimator over an ascending list of averaging factors — the §14
 * "ADEV/MDEV vs tau" plot in one call.
 *
 * @p m_list must be strictly ascending. The sweep stops at the first factor the
 * record cannot support and reports how many points it produced, because that is
 * the honest shape of the answer: a stability plot's right-hand end is limited by
 * record length, and silently omitting an interior point (or emitting a
 * placeholder) would let a caller plot a tau axis that does not match its data.
 *
 * @param kind     Which deviation.
 * @param x_ps     Phase samples, picoseconds, oldest first.
 * @param n        Number of samples.
 * @param tau0_ns  Sample interval, nanoseconds.
 * @param m_list   Strictly ascending averaging factors.
 * @param n_m      Length of @p m_list.
 * @param out_dev  Receives up to @p n_m deviations.
 * @param out_n    Receives the number actually written. Required.
 *
 * @retval 0        Success — @p *out_n points written (possibly 0).
 * @retval -EINVAL  NULL argument, n_m == 0, a zero or non-ascending factor, or a
 *                  factor above STATS_MAX_M.
 * @retval -ERANGE  Some |x_ps[i]| exceeds STATS_PHASE_MAX_PS.
 */
int stats_dev_sweep(stats_dev_t kind, const int64_t *x_ps, size_t n,
		    uint64_t tau0_ns, const uint32_t *m_list, size_t n_m,
		    double *out_dev, size_t *out_n);

/**
 * Fill @p out_m with the octave-spaced averaging factors (1, 2, 4, 8, ...) a
 * record of @p n samples supports — the conventional x-axis of a stability plot,
 * and a ready-made argument for stats_dev_sweep().
 *
 * @param n      Sample count.
 * @param kind   Which estimator the factors are for (they differ: ADEV reaches
 *               (n-1)/2, MDEV only n/3).
 * @param out_m  Receives the factors, ascending. May be NULL to count only.
 * @param cap    Capacity of @p out_m; ignored when @p out_m is NULL.
 * @return       Number of factors the record supports, which may exceed @p cap
 *               (in which case only @p cap were written).
 */
size_t stats_m_octaves(size_t n, stats_dev_t kind, uint32_t *out_m, size_t cap);

/* ------------------------------------------------------------- histogram */

/**
 * PPS residual histogram plus the summary a bench operator reads off it
 * (spec §14).
 *
 * The input is a plain residual series, deliberately: §14 wants this computed
 * twice, once on sawtooth-corrected residuals and once on uncorrected ones, and
 * the *difference* between the two summaries is the evidence that the UBX-TIM-TP
 * qErr path is wired up and has the right sign. Folding the correction in here
 * would make this function unable to produce the "without" half of that pair.
 *
 * Bin k covers [lo_ps + k*bin_ps, lo_ps + (k+1)*bin_ps). Samples outside the
 * bins are counted in `under`/`over` and still contribute to every summary
 * statistic — the summary describes the data, not the window chosen to draw it,
 * so a badly chosen window cannot quietly change the reported RMS.
 *
 * A two-pass auto-range is the caller's to make: run once with @p bins NULL to
 * get min/max, then choose lo_ps and bin_ps and run again.
 *
 * @param resid_ps  Residual series, picoseconds.
 * @param n         Number of residuals; at least 1.
 * @param lo_ps     Lower edge of bin 0. Ignored when @p bins is NULL.
 * @param bin_ps    Bin width, picoseconds; positive. Ignored when @p bins NULL.
 * @param bins      Receives the counts, zeroed first. NULL for summary only.
 * @param n_bins    Number of bins; non-zero when @p bins is non-NULL.
 * @param out       Receives the summary. Required.
 *
 * @retval 0        Success.
 * @retval -EINVAL  NULL required argument, n == 0, or (with @p bins non-NULL)
 *                  n_bins == 0 or bin_ps <= 0.
 * @retval -ERANGE  Some |resid_ps[i]| exceeds STATS_PHASE_MAX_PS; @p lo_ps does
 *                  too; the bin window's upper edge is not representable; or
 *                  @p n exceeds INT64_MAX / STATS_PHASE_MAX_PS (9.2e6 samples,
 *                  106 days of 1 Hz PPS), past which the exact integer sum
 *                  behind `mean_ps` could wrap. Split the record rather than be
 *                  handed a wrapped mean.
 */
int stats_hist(const int64_t *resid_ps, size_t n, int64_t lo_ps, int64_t bin_ps,
	       uint32_t *bins, size_t n_bins, stats_hist_t *out);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_STATS_ADEV_H_ */
