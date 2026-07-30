/*
 * STS1000 "Meridian" — core/stats: the spec §14 sawtooth-correction verdict.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation, no platform headers, no globals,
 * no libm (stats_sqrt() from adev.h).
 *
 * WHAT §14 ACTUALLY ASKS FOR
 *
 * "PPS residual histograms with/without sawtooth correction (prove the qErr
 * path)". stats_hist() computes either histogram. Neither one proves anything
 * on its own: a residual histogram from a working qErr path and one from a
 * qErr path with an inverted sign are both single-peaked, both centred in the
 * same place, and differ only in a width that has nothing to be compared
 * against. The proof is in the PAIR.
 *
 * THE SIGNATURE
 *
 * Write the corrected residual as c[i] and the uncorrected one as r[i]. The
 * correction the loop actually applied is exactly s[i] = c[i] - r[i] — not a
 * qErr re-derived here, the real applied delta, including the seconds where it
 * was gated off and s[i] is 0.
 *
 * Let w be the sawtooth artefact in the raw measurement, so r = t + w for some
 * underlying residual t that the correction cannot touch. A correct
 * implementation applies s = -w and lands on c = t. Then:
 *
 *   var(c) = var(r) - var(s)          spread COLLAPSES by var(s)
 *
 * An implementation with the sign inverted applies s = +w and lands on
 * c = t + 2w:
 *
 *   var(c) = var(r) + 3*var(s)        spread GROWS; sd roughly DOUBLES when
 *                                     the sawtooth dominates the raw residual
 *
 * And one that applies a qErr belonging to some other pulse — the pairing
 * failure core/disc's disc_name_pulse() exists to prevent — applies an s
 * uncorrelated with w:
 *
 *   var(c) = var(r) + var(s)          spread grows slightly; the correction is
 *                                     adding noise rather than removing it
 *
 * The three cases separate cleanly in one number:
 *
 *   z = (var(c) - var(r)) / var(s)  =  -1 correct, +1 uncorrelated, +3 inverted
 *
 * z is used rather than the bare spread ratio sd(c)/sd(r) because the ratio's
 * meaning depends on how large the sawtooth is relative to everything else:
 * with a small sawtooth an inverted sign gives a ratio of 1.05, not 2, and a
 * threshold picked for one board is wrong on the next. z is normalised by the
 * correction that was actually applied, so its landmarks are fixed. The spread
 * ratio is still reported — it is what a bench operator was told to look at —
 * but it is not what the verdict is decided on.
 *
 * WHY THE UNCERTAINTY IS PART OF THE ANSWER
 *
 * z is a sample statistic. Its noise is set by how much of r is NOT explained
 * by s: when the sawtooth is buried under a large residual, or the record is
 * short, z wanders and a confident verdict would be a lie. @ref stats_saw_t
 * carries z_sigma and @ref stats_saw_t::marginal, which is set when z is within
 * two sigma of a band boundary. A marginal verdict means "take a longer
 * capture", not "the qErr path is broken".
 */

#ifndef STS1000_CORE_STATS_SAW_H_
#define STS1000_CORE_STATS_SAW_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** The four things a with/without pair can say about the qErr path. */
typedef enum {
	/**
	 * No correction is present: every sample's corrected and uncorrected
	 * residual is identical, so s is identically zero. Either the receiver
	 * supplied no usable qErr for any pulse, or the correction is disabled.
	 * NOT a statement that the path is broken — and not a pass either.
	 */
	STATS_SAW_NONE = 0,
	/** z < 0: the spread collapsed. The qErr path is doing its job. */
	STATS_SAW_CORRECTED,
	/**
	 * 0 <= z < 2: a correction was applied and the spread did not collapse.
	 * z near +1 is the signature of a qErr paired to the wrong pulse — the
	 * magnitude is right, the alignment is not.
	 */
	STATS_SAW_INEFFECTIVE,
	/**
	 * z >= 2: the spread grew by about what an inverted sign predicts
	 * (+3*var(s), i.e. roughly double the standard deviation once the
	 * sawtooth dominates). This is the defect §14's pair exists to catch.
	 */
	STATS_SAW_INVERTED,
} stats_saw_verdict_t;

/** Result of stats_saw_verdict(). All spreads are population, picoseconds. */
typedef struct {
	stats_saw_verdict_t verdict;
	/** Samples compared. */
	size_t n;
	/** Samples where the correction was non-zero, i.e. c[i] != r[i]. */
	size_t n_applied;
	double mean_corr_ps;
	double mean_raw_ps;
	double sdev_corr_ps;
	double sdev_raw_ps;
	/** Spread of the applied correction s = c - r. */
	double sdev_saw_ps;
	/** mean(c) - mean(r): the bias the correction introduced. */
	double mean_shift_ps;
	/**
	 * sdev_corr_ps / sdev_raw_ps, the headline "did the spread collapse or
	 * roughly double" number. -1.0 when sdev_raw_ps is 0 and the ratio has
	 * no value.
	 */
	double spread_ratio;
	/** (var(c) - var(r)) / var(s). -1 correct, +1 uncorrelated, +3 inverted. */
	double z;
	/** One-sigma uncertainty of @ref z from this record. */
	double z_sigma;
	/** @ref z is within 2 sigma of a verdict boundary (0 or +2). */
	bool marginal;
} stats_saw_t;

/**
 * Judge a with/without sawtooth pair (spec §14).
 *
 * @param corr_ps  n sawtooth-corrected residuals, picoseconds.
 * @param raw_ps   the SAME n samples uncorrected, same order. Must come from
 *                 the device's own pre-correction capture; a series
 *                 reconstructed by adding a qErr back to @p corr_ps cannot
 *                 disprove a sign error, because it inherits it.
 * @param n        sample count; at least 2.
 * @param out      Receives the verdict and its supporting statistics.
 *
 * @retval 0         Success — including a STATS_SAW_NONE verdict, which is a
 *                   real answer about the capture, not a failure to compute.
 * @retval -EINVAL   NULL argument.
 * @retval -ENODATA  Fewer than 2 samples: no spread is defined.
 */
int stats_saw_verdict(const int64_t *corr_ps, const int64_t *raw_ps, size_t n,
		      stats_saw_t *out);

/** Stable one-word name for @p v, for logs and CLI output. Never NULL. */
const char *stats_saw_str(stats_saw_verdict_t v);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_STATS_SAW_H_ */
