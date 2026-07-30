/*
 * STS1000 "Meridian" — core/stats: the bench phase-record container.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation, no platform headers, no globals.
 *
 * WHY A CONTAINER, AND WHY IT LIVES HERE
 *
 * adev.h is the spec §14 arithmetic: it takes a phase array, a sample count and
 * one tau0, and returns ADEV/MDEV/histograms. It has no input path — a bench
 * operator with a board in front of them cannot get a phase series out of the
 * device and into it.
 *
 * This is that path's on-the-wire and on-disk form. It sits in core/stats rather
 * than in a protocol module because the device encodes it and the host analyser
 * decodes it, and those two must agree byte for byte: one definition, compiled
 * into both, is the only version of that which cannot drift. It adds no
 * dependency — the module's edge set stays empty, so the closed-form assertions
 * in tests/host/test_stats.c still concern nothing but the estimator.
 *
 * THE FIELD THAT MATTERS: `gaps`
 *
 * An estimator over a phase array assumes the samples are uniformly spaced,
 * because a bare array carries no timestamps. The runtime that produced the
 * series does not guarantee that: core/disc deliberately absorbs sample gaps up
 * to ADEV_MAX_GAP_S so that a rejected sample every few minutes does not stop
 * the tau = 100 s telemetry point from ever existing. That trade is correct for
 * telemetry and inadmissible for validation — an absorbed gap mis-times every
 * later second difference, and the resulting plot looks entirely plausible.
 *
 * So the record states it. @ref phase_rec_meta_t::gaps counts the samples that
 * followed a non-uniform interval and the bitmap says which. A record with
 * gaps != 0 is not fit for ADEV as a whole. Consumers must act on it:
 * phase_rec_longest_clean() exists so that acting on it can mean "analyse the
 * longest uniformly spaced run" rather than "throw the capture away".
 *
 * THE SECOND SERIES: `x_raw_ps`
 *
 * Spec §14 asks for PPS residual histograms *with and without* sawtooth
 * correction, to prove the qErr path. One series proves nothing: the pair has a
 * joint signature — correction leaves the mean essentially where it was and
 * collapses the spread — and the failure the proof exists to catch is a SIGN
 * INVERSION in the qErr path, which roughly DOUBLES the spread instead. "Did
 * the spread shrink?" and "did the spread roughly double?" are answerable only
 * against the uncorrected series, so the record carries both.
 *
 * The uncorrected series is CAPTURED, not reconstructed. core/disc snapshots
 * the phase error immediately before it adds the sawtooth term and keeps that
 * value in its own ring; nothing here re-derives one series from the other by
 * adding a qErr back. That distinction is the whole point — the correction path
 * gates on qerr_valid and rounds in float, so a reconstruction would differ
 * from the real uncorrected series exactly where the proof is being made, and
 * would agree with a broken qErr path by construction.
 *
 * WIRE FORMAT (little-endian throughout, no padding, no alignment requirement)
 *
 *   off  size  field
 *   ----------------------------------------------------------------------
 *    0    4    magic     PHASE_REC_MAGIC ('P','H','R','1')
 *    4    1    ver       PHASE_REC_VER
 *    5    1    flags     PHASE_REC_F_*
 *    6    2    n         sample count
 *    8    4    tau0_ns   nominal sample interval, nanoseconds; non-zero
 *   12    4    gaps      non-uniform-interval samples within [0, n)
 *   16    8    mono_ms   device monotonic time at capture (provenance only)
 *   ----------------------------------------------------------------------
 *   24        gb   gapmap    (n + 7) / 8 bytes, LSB-first; bit i describes
 *                            sample i
 *   24+gb     8n   x_ps      n * int64 sawtooth-CORRECTED phase samples,
 *                            picoseconds, oldest first
 *   24+gb+8n  8n   x_raw_ps  n * int64 UNCORRECTED phase samples, same units,
 *                            same order, same samples — present if and only if
 *                            PHASE_REC_F_RAW is set in `flags`
 *
 * Picoseconds because that is exactly what stats_adev()/stats_mdev() take: a
 * decoded record is handed to the estimator with no conversion step that could
 * introduce a scale error between the two sides.
 *
 * VERSIONING, AND WHY A STALE DECODER CANNOT MISREAD A PAIRED RECORD
 *
 * The second series is appended, so the leading bytes of a v2 record are
 * byte-identical to the v1 record of the same samples. That is exactly the
 * shape a decoder can get wrong: a v1 reader would find its expected length
 * satisfied, read the corrected series, and report a single-series analysis
 * from a record whose whole purpose is the pair. Two independent guards stop
 * that:
 *
 *   1. `ver` is 2. The v1 decoder tests `ver != 1` and refuses outright, so it
 *      cannot silently analyse half of a paired record.
 *   2. `flags` declares the contents and the length must match the declaration:
 *      PHASE_REC_F_RAW is what makes the extra 8n bytes REQUIRED, and
 *      phase_rec_decode_hdr() rejects a record that claims the raw series but
 *      is not long enough to hold it. Unknown flag bits are refused for the
 *      same reason — an unrecognised bit means unrecognised content, and a
 *      decoder that ignores it is guessing at the layout.
 *
 * This decoder still reads v1 records: their layout is a strict prefix, they
 * cannot set F_RAW (guard 3 below), and a capture taken before the pair existed
 * is still a valid single-series capture. What it will not do is treat one as
 * if it carried the pair.
 */

#ifndef STS1000_CORE_STATS_PHASE_REC_H_
#define STS1000_CORE_STATS_PHASE_REC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 'P','H','R','1' as a little-endian u32. */
#define PHASE_REC_MAGIC UINT32_C(0x31524850)

/**
 * Format version written by this build.
 *
 * 2 adds the appended uncorrected series (PHASE_REC_F_RAW). The bump is what
 * stops a v1 decoder from reading a paired record as a single-series one; see
 * the versioning note in the banner.
 */
#define PHASE_REC_VER 2u

/** Oldest format version this decoder accepts. */
#define PHASE_REC_VER_MIN 1u

/** Fixed header length, in bytes. */
#define PHASE_REC_HDR_LEN 24u

/**
 * The producing ring was full, so samples older than x_ps[0] were evicted.
 * Informational: it says the record is a window on a longer series, not that
 * anything is wrong with the window.
 */
#define PHASE_REC_F_FULL 0x01u

/**
 * The record carries the uncorrected (pre-sawtooth) series after the corrected
 * one, so it holds 16 bytes per sample rather than 8. Requires ver >= 2.
 *
 * This bit is load-bearing, not descriptive: it is what makes the trailing 8n
 * bytes mandatory, and phase_rec_decode_hdr() refuses a record that sets it
 * without the length to back it.
 */
#define PHASE_REC_F_RAW 0x02u

/**
 * Every flag bit this build understands. A record setting anything outside this
 * mask is refused — an unknown bit may declare content whose length this
 * decoder cannot account for, and guessing is how a truncated record gets
 * analysed as a whole one.
 */
#define PHASE_REC_F_KNOWN (PHASE_REC_F_FULL | PHASE_REC_F_RAW)

/** Header fields, host-side. */
typedef struct {
	uint8_t ver;
	uint8_t flags;
	/** Samples in the record. */
	uint16_t n;
	/** Nominal sample interval, nanoseconds. Never 0 in a valid record. */
	uint32_t tau0_ns;
	/**
	 * Samples within [0, n) that followed a non-uniform interval.
	 *
	 * 0 == the whole record is uniformly spaced and fit for ADEV/MDEV.
	 * Non-zero == it is not, and an estimator run over the whole array
	 * returns a confident wrong answer.
	 */
	uint32_t gaps;
	/** Device monotonic milliseconds at capture. Provenance only. */
	uint64_t mono_ms;
} phase_rec_meta_t;

/**
 * Encoded size of a single-series record holding @p n samples, in bytes.
 *
 * Header + bitmap + one series. Equivalent to
 * phase_rec_size_flags(@p n, 0) — kept because most callers mean exactly this.
 */
size_t phase_rec_size(uint16_t n);

/**
 * Encoded size of a record holding @p n samples with @p flags, in bytes.
 *
 * The only flag that changes the length is PHASE_REC_F_RAW, which appends a
 * second series of the same shape. This is the function that ties the header's
 * declaration to the record's length, and it is what phase_rec_decode_hdr()
 * validates against — so a record cannot claim contents it does not carry.
 */
size_t phase_rec_size_flags(uint16_t n, uint8_t flags);

/**
 * Encode a record into @p buf.
 *
 * @param m        Header fields. @p m->n is the sample count; ver is written
 *                 as PHASE_REC_VER regardless of what the caller set.
 * @param x_ps     @p m->n phase samples, picoseconds, oldest first.
 * @param gapmap   (@p m->n + 7) / 8 bytes of LSB-first per-sample gap bits, or
 *                 NULL for a record with no gap information — permitted only
 *                 when @p m->gaps is 0, because a count with no map that
 *                 disagrees would be unresolvable.
 * @param buf      Destination.
 * @param cap      Capacity of @p buf.
 * @param out_len  Receives the encoded length. Required.
 *
 * @retval 0         Success.
 * @retval -EINVAL   NULL argument, tau0_ns == 0, gaps > n, or a non-zero gaps
 *                   with a NULL gapmap.
 * @retval -EMSGSIZE @p cap is smaller than phase_rec_size(m->n).
 */
int phase_rec_encode(const phase_rec_meta_t *m, const int64_t *x_ps,
		     const uint8_t *gapmap, uint8_t *buf, size_t cap,
		     size_t *out_len);

/**
 * Encode a record carrying both the corrected and the uncorrected series.
 *
 * @param x_ps      @p m->n sawtooth-corrected samples, picoseconds.
 * @param x_raw_ps  @p m->n uncorrected samples, same order — or NULL, which
 *                  encodes a single-series record identical to
 *                  phase_rec_encode()'s.
 *
 * PHASE_REC_F_RAW is set in the encoded header if and only if @p x_raw_ps is
 * non-NULL; @p m->flags cannot assert it and cannot suppress it, because the
 * flag describes what was written and only this function knows that.
 *
 * @retval 0         Success.
 * @retval -EINVAL   As phase_rec_encode().
 * @retval -EMSGSIZE @p cap is smaller than the paired record's length.
 */
int phase_rec_encode_pair(const phase_rec_meta_t *m, const int64_t *x_ps,
			  const int64_t *x_raw_ps, const uint8_t *gapmap,
			  uint8_t *buf, size_t cap, size_t *out_len);

/**
 * Encode from the device's float-nanosecond rings, converting in place.
 *
 * The producing side holds both series as single-precision nanoseconds (see
 * disc_phase_snap_t). Converting through phase_rec_ns_f_to_ps() inside the
 * encoder rather than in the caller is not a convenience: it removes the
 * caller's int64 staging array, which for a 512-sample ring is 4 KiB of RAM
 * per series on a part that has none to spare.
 *
 * @param x_ns      @p m->n corrected samples, float nanoseconds.
 * @param x_raw_ns  @p m->n uncorrected samples, or NULL for a single-series
 *                  record.
 *
 * Semantics are otherwise phase_rec_encode_pair()'s exactly.
 */
int phase_rec_encode_f(const phase_rec_meta_t *m, const float *x_ns,
		       const float *x_raw_ns, const uint8_t *gapmap,
		       uint8_t *buf, size_t cap, size_t *out_len);

/**
 * Decode and validate the header of a record.
 *
 * Validation is total: magic, version, the declared length against @p len, and
 * the internal consistency of gaps against both n and the bitmap's actual
 * population count. The last of those is the point — a truncated or tampered
 * record must not be able to present itself as clean by carrying gaps = 0 over
 * a bitmap with bits set.
 *
 * @retval 0         Success.
 * @retval -EINVAL   NULL argument.
 * @retval -EBADMSG  Bad magic, unsupported version, an unknown flag bit,
 *                   PHASE_REC_F_RAW on a pre-v2 record, tau0_ns == 0,
 *                   gaps > n, or gaps disagreeing with the bitmap's
 *                   population count.
 * @retval -EMSGSIZE @p len is shorter than the record the header declares —
 *                   including when the shortfall is only the raw series the
 *                   flags claim.
 */
int phase_rec_decode_hdr(const uint8_t *buf, size_t len, phase_rec_meta_t *out);

/**
 * Copy the phase samples out of a validated record.
 *
 * @param out_x  Receives min(n, cap) samples, picoseconds, oldest first.
 * @param cap    Capacity of @p out_x, in samples.
 * @param out_n  Receives the number written. Required.
 *
 * @retval 0         Success.
 * @retval -EINVAL   NULL argument.
 * @retval -EBADMSG  Header failed validation (see phase_rec_decode_hdr()).
 * @retval -EMSGSIZE @p cap is smaller than the record's sample count. Nothing
 *                   is written: a silently truncated phase series would change
 *                   the answer without changing its appearance.
 */
int phase_rec_decode_samples(const uint8_t *buf, size_t len, int64_t *out_x,
			     size_t cap, size_t *out_n);

/**
 * Copy the UNCORRECTED (pre-sawtooth) series out of a validated record.
 *
 * Same samples, same order and same count as phase_rec_decode_samples(); the
 * pair is what spec §14's sawtooth proof is computed from (see stats/saw.h).
 *
 * @retval 0         Success.
 * @retval -EINVAL   NULL argument.
 * @retval -ENOENT   The record does not carry the uncorrected series
 *                   (PHASE_REC_F_RAW clear). Not an error in the record — a
 *                   v1 capture, or an exporter that had only one series.
 * @retval -EBADMSG  Header failed validation (see phase_rec_decode_hdr()).
 * @retval -EMSGSIZE @p cap is smaller than the record's sample count. Nothing
 *                   is written, for phase_rec_decode_samples()' reason.
 */
int phase_rec_decode_raw(const uint8_t *buf, size_t len, int64_t *out_x,
			 size_t cap, size_t *out_n);

/** True when sample @p i of a validated record followed a non-uniform interval. */
bool phase_rec_gap_at(const uint8_t *buf, size_t len, size_t i);

/**
 * Longest run of consecutive samples containing no absorbed gap.
 *
 * A gap bit on sample i means the interval *ending* at i is wrong, so i breaks
 * the run and starts the next one — [start, start + run) is uniformly spaced
 * and safe to hand to an estimator.
 *
 * Returns 0 (and leaves @p out_start alone) for an invalid or empty record.
 * For a clean record the answer is the whole thing: run = n, start = 0.
 *
 * @param out_start  Receives the run's first sample index. May be NULL.
 */
size_t phase_rec_longest_clean(const uint8_t *buf, size_t len,
			       size_t *out_start);

/**
 * Convert a phase sample from float nanoseconds to int64 picoseconds.
 *
 * The device holds its ring in single-precision nanoseconds; the record and the
 * estimators are integer picoseconds. Rounds to nearest, and saturates rather
 * than invoking the undefined behaviour a float-to-integer conversion has when
 * the value does not fit — including for a NaN, which is mapped to 0 because a
 * saturated NaN is a silent 1000-second phase error.
 */
int64_t phase_rec_ns_f_to_ps(float ns);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_STATS_PHASE_REC_H_ */
