/*
 * STS1000 "Meridian" — core/stats: joining successive phase records.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation, no platform headers, no globals.
 *
 * WHY
 *
 * The device's ADEV phase ring is DISC_ADEV_CAP = 512 samples at tau0 = 1 s, so
 * one exported record is 8 min 32 s and the largest averaging factor it can
 * support is m = (n-1)/2 = 255 — an octave axis that stops at tau = 128 s. Spec
 * §14 wants ADEV/MDEV against tau for the OCXO and the rubidium, and the part
 * of that curve which decides whether an oscillator is fit to hold over runs to
 * tau of many hundreds or thousands of seconds. 128 s does not reach it.
 *
 * Enlarging the ring is not the answer. Each extra sample costs 4 B in
 * disc_ctx_t's corrected ring, 4 B in its uncorrected ring, 4 B + 4 B again in
 * the disc_phase_snap_t the exporter stages through, and 16 B in the export
 * blob — about 32 B per sample, plus two bitmaps. A 2048-sample ring (enough
 * for tau = 1024 s and no further) is roughly 65 KB against the ~73 KB of SRAM
 * this build has left, and it triples the per-second memmove on the timing
 * thread. A ring for tau = 8192 s is not expressible on the part at all.
 *
 * So the record stays a WINDOW and the long record is built by polling
 * repeatedly and concatenating. This file is the concatenation, and the reason
 * it is a module rather than three lines of memcpy in the analyser is that a
 * concatenation which cannot be PROVED contiguous is silent corruption: two
 * polls that overlap splice duplicated seconds into the series, and two that
 * straddle a dropped stretch splice a hole out of existence. Either produces a
 * phase series with a wrong time axis, from which stats_adev() returns a
 * confident wrong number that looks entirely ordinary. That is the same failure
 * the record's absorbed-gap count exists to stop, one level up.
 *
 * WHAT IT PROVES BEFORE IT JOINS
 *
 *  1. **Identity exists.** A record without PHASE_REC_F_IDENT (a v1/v2 capture,
 *     or a device that could not draw an epoch) says nothing about which
 *     samples it holds. It is refused: -ENOENT.
 *
 *  2. **The epoch matches.** The epoch changes on every event that restarts the
 *     numbering — a PPS gap past ADEV_MAX_GAP_S, entry to holdover,
 *     disc_restart(), a reboot. Records from either side of one describe
 *     different series and are not joinable however their indices read:
 *     -EPROTO.
 *
 *  3. **The overlap agrees.** Polls taken faster than the ring empties overlap,
 *     which is not a nuisance to be tolerated but the mechanism by which the
 *     join is CHECKED: samples the two records share come from the same ring
 *     slots, so they must be bit-identical, in both series and in their gap
 *     bits. A disagreement means the epoch has been reused or a record is
 *     corrupt, and it is a hard failure: -EILSEQ.
 *
 *  4. **There is no hole.** A record starting after the accumulated span ends
 *     is refused (-ERANGE) rather than joined with the missing stretch marked
 *     as a gap. The gap bitmap records that ONE interval was non-uniform; it
 *     has no field for how long the discontinuity was. Marking a 4000-second
 *     hole as a single gap bit would leave --allow-gaps computing over a series
 *     whose time axis is short by more than an hour, and would leave the record
 *     claiming a duration it does not have. Two chains that cannot be joined
 *     are two records, and analysing them separately is correct where analysing
 *     their concatenation is not.
 *
 * THE GAP BITMAP COMPOSES AS A UNION
 *
 * The joined bitmap is the union of the parts', each mapped to ABSOLUTE sample
 * index (bit `seq - seq0_join`) rather than to its own record-local index. The
 * offsets are not multiples of 8 in general — the ring slides one sample a
 * second, so a poll two minutes later starts 120 samples along — so the union
 * is taken bit by bit; a byte-wise copy would displace every gap in the part by
 * up to seven samples and quietly relocate the discontinuities the whole field
 * exists to name. `gaps` is maintained as the population count of that union,
 * so the joined record satisfies the same header/bitmap cross-check
 * phase_rec_decode_hdr() applies to a device record.
 *
 * ORDER
 *
 * Records must be added oldest first (non-decreasing seq0). The accumulator
 * only ever extends forward, because prepending would mean shifting the whole
 * accumulated series and bit-shifting the whole bitmap on every out-of-order
 * arrival — cost and risk paid on every join to spare the caller a sort it can
 * do once over the record headers. A record older than the accumulated span is
 * refused: -EALREADY.
 */

#ifndef STS1000_CORE_STATS_PHASE_JOIN_H_
#define STS1000_CORE_STATS_PHASE_JOIN_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stats/phase_rec.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Accumulator for a joined phase series.
 *
 * The destination arrays are the CALLER'S. A join can reach 65535 samples —
 * 512 KB per series — which is not a thing core/ may allocate, and not a thing
 * the device would ever want; the only consumer is the host analyser, which
 * knows its own bound. Fields below the arrays are state and must not be
 * written by the caller.
 */
typedef struct {
	/* ---- caller-owned destination ---- */
	int64_t *x;       /**< corrected series, @ref cap entries */
	int64_t *x_raw;   /**< uncorrected series, @ref cap entries, or NULL */
	uint8_t *gapmap;  /**< (@ref cap + 7) / 8 bytes, zeroed by init */
	size_t cap;       /**< capacity of the destination, in samples */

	/* ---- accumulated state ---- */
	bool started;     /**< at least one record has been added */
	bool have_raw;    /**< the parts carry the uncorrected series */
	bool full;        /**< the earliest part had PHASE_REC_F_FULL */
	uint32_t epoch;   /**< the epoch every part must carry */
	uint64_t seq0;    /**< absolute index of x[0] */
	uint32_t tau0_ns; /**< the tau0 every part must carry */
	uint64_t mono_ms; /**< mono_ms of the part that contributed x[0] */
	size_t n;         /**< samples accumulated */
	size_t gaps;      /**< popcount of @ref gapmap over [0, n) */
	size_t parts;     /**< records accepted */
	size_t overlap;   /**< samples verified against an earlier part */
} phase_join_t;

/**
 * Prepare @p j to accumulate into the caller's arrays.
 *
 * @param x       Corrected series destination, @p cap entries. Required.
 * @param x_raw   Uncorrected series destination, @p cap entries, or NULL to
 *                join records that carry only the corrected series. A record
 *                carrying PHASE_REC_F_RAW is refused when this is NULL rather
 *                than silently losing the series §14's sawtooth proof needs.
 * @param gapmap  (@p cap + 7) / 8 bytes. Required; zeroed here.
 * @param cap     Destination capacity, in samples. Must be non-zero.
 *
 * @retval 0        Success.
 * @retval -EINVAL  NULL @p j, @p x or @p gapmap, or @p cap == 0.
 */
int phase_join_init(phase_join_t *j, int64_t *x, int64_t *x_raw,
		    uint8_t *gapmap, size_t cap);

/**
 * Validate one encoded record against the accumulated span and splice it in.
 *
 * The first accepted record establishes the epoch, tau0, the presence of the
 * uncorrected series, and the origin of the index axis. Every later one must
 * agree, must not start before the span, must not start after it ends, and must
 * match it wherever the two overlap.
 *
 * @retval 0          Joined. @p j->n, gaps and overlap are updated.
 * @retval -EINVAL    NULL argument, or phase_join_init() was not called.
 * @retval -EBADMSG   The record failed phase_rec_decode_hdr(), or carries no
 *                    samples (n == 0), which cannot extend anything.
 * @retval -EMSGSIZE  The record is shorter than its header declares.
 * @retval -ENOENT    The record carries no identity (PHASE_REC_F_IDENT clear),
 *                    so nothing about it can be proved contiguous.
 * @retval -EPROTO    The record's epoch differs from the accumulated one: it
 *                    is from the other side of a ring reset or a reboot.
 * @retval -ENOTSUP   tau0 differs between parts, or one part carries the
 *                    uncorrected series and another does not, or the record
 *                    carries it and phase_join_init() got no @p x_raw.
 * @retval -EALREADY  The record starts before the accumulated span; records
 *                    must be added oldest first.
 * @retval -ERANGE    The record starts after the accumulated span ends: the
 *                    samples between are missing and the join would fabricate
 *                    a continuous time axis across a hole.
 * @retval -EILSEQ    The record and the span disagree about a sample they both
 *                    claim to hold. The epoch is being reused or one of them
 *                    is corrupt; nothing here can tell which, and joining
 *                    either version would be a guess.
 * @retval -ENOSPC    The joined series would exceed @p j->cap.
 *
 * On any error the accumulator is left exactly as it was: a refused record
 * contributes nothing, so a caller may report the reason and continue with the
 * records it has.
 */
int phase_join_add(phase_join_t *j, const uint8_t *buf, size_t len);

/**
 * Fill @p out with a phase_rec_meta_t describing the joined series, ready for
 * phase_rec_encode_pair() to write it back out as one record.
 *
 * The joined record keeps the FIRST part's mono_ms and PHASE_REC_F_FULL,
 * because both describe the capture that contributed sample 0 — the same
 * capture seq0 names. Its identity is (epoch, seq0) of that sample, so a joined
 * record is itself joinable with a later poll of the same epoch.
 *
 * @retval 0           Success.
 * @retval -EINVAL     NULL argument.
 * @retval -ENODATA    Nothing has been joined.
 * @retval -EOVERFLOW  The join holds more than UINT16_MAX samples, which the
 *                     record's 16-bit count cannot express. The in-memory
 *                     series is still valid and still analysable; only writing
 *                     it back as one record is refused.
 */
int phase_join_meta(const phase_join_t *j, phase_rec_meta_t *out);

/**
 * One sentence naming what a phase_join_add() return value means, for an
 * operator who is looking at two files and a refusal.
 *
 * Never NULL; an unrecognised code gets a generic sentence rather than a crash.
 */
const char *phase_join_strerror(int rc);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_STATS_PHASE_JOIN_H_ */
