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
 * THE THIRD THING: SAMPLE IDENTITY (`epoch`, `seq0`)
 *
 * The producing ring is 512 samples at tau0 = 1 s (DISC_ADEV_CAP), so one
 * record is 8.5 minutes and the largest averaging factor it supports is
 * m = (n-1)/2 = 255 — a tau axis that stops at 128 s. Spec §14's interesting
 * region for an OCXO and a rubidium runs to tau of many hundreds or thousands
 * of seconds, which the ring cannot hold and the part cannot afford to enlarge
 * (a 2048-sample ring costs ~65 KB across the ring, the snapshot and the export
 * blob, against ~73 KB free).
 *
 * The record is therefore a WINDOW on a longer series, and the way to a longer
 * tau is to poll repeatedly and concatenate. That is only safe if the host can
 * PROVE the pieces are contiguous, and a record carrying nothing but samples
 * cannot be proved anything about: two polls that overlap, or that straddle a
 * dropped stretch, splice into a series with a wrong time axis and the plot
 * looks entirely ordinary. So the record numbers its samples.
 *
 *   `seq0`  is the monotonic index of x_ps[0] within the current epoch, so
 *           sample i is index seq0 + i. Two records overlap where their index
 *           ranges intersect, abut when one ends where the other starts, and
 *           are separated by a hole otherwise — all three distinguishable.
 *
 *   `epoch` is the numbering GENERATION. It changes on every event that
 *           restarts the numbering: a ring reset (a PPS gap beyond
 *           ADEV_MAX_GAP_S, entry to holdover, disc_restart()) and a reboot.
 *           Records from either side of one are not joinable at all, whatever
 *           their indices say, and a differing epoch is what says so.
 *
 * The epoch's initial value is a random 32-bit draw supplied by the platform,
 * not a constant, and that is load-bearing rather than decorative: with a
 * constant seed a reboot would restart BOTH the epoch and the index, so a
 * pre-reboot record covering [512, 1024) and a post-reboot record covering
 * [0, 512) would present as a perfectly abutting pair with no overlap to check
 * — a silent splice across a discontinuity, which is the exact failure this
 * field exists to stop. A platform that cannot draw one supplies 0, and the
 * loop then reports NO epoch: the records carry no identity and refuse to join
 * rather than joining wrongly.
 *
 * Overlap is normal and useful. The ring slides one sample a second, so polls
 * taken faster than it empties always overlap, and the overlap is where a join
 * is VERIFIED: the shared samples must be identical. They come from the same
 * ring slots, so they are bit-identical or something is wrong — a mismatch
 * means the epoch is being reused or a record is corrupt, and it is a hard
 * failure. See stats/phase_join.h for the joiner.
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
 *   ...+12    12   ident     u32 epoch, u64 seq0 — present if and only if
 *                            PHASE_REC_F_IDENT is set in `flags`. Appended,
 *                            like the raw series, so the leading bytes stay
 *                            byte-identical to the older record of the same
 *                            samples and the gapmap/series offsets do not move
 *                            with the version.
 *
 * Picoseconds because that is exactly what stats_adev()/stats_mdev() take: a
 * decoded record is handed to the estimator with no conversion step that could
 * introduce a scale error between the two sides.
 *
 * VERSIONING, AND WHY A STALE DECODER CANNOT MISREAD A LATER RECORD
 *
 * Every addition is APPENDED, so the leading bytes of a v3 record are
 * byte-identical to the v2 record of the same samples, which are in turn
 * byte-identical to the v1 record. That is exactly the shape a decoder can get
 * wrong: an older reader would find its expected length satisfied, read what it
 * knows, and report an analysis from a record whose whole purpose is the part
 * it skipped — a single-series analysis of a paired record, or an unverifiable
 * join of records whose identity it never saw. Two independent guards stop
 * that, and each addition arms both:
 *
 *   1. `ver` is 3. A v1 decoder tests `ver != 1` and a v2 decoder tests
 *      `ver > 2`; both refuse outright, so neither can silently analyse the
 *      part of a v3 record it happens to understand.
 *   2. `flags` declares the contents and the length must match the declaration.
 *      PHASE_REC_F_RAW is what makes the second series' 8n bytes REQUIRED and
 *      PHASE_REC_F_IDENT what makes the identity block's 12 bytes required;
 *      phase_rec_decode_hdr() rejects a record that claims either and is not
 *      long enough to hold it. Unknown flag bits are refused for the same
 *      reason — an unrecognised bit means unrecognised content, and a decoder
 *      that ignores it is guessing at the layout. A v2 decoder therefore
 *      refuses a v3 record on the unknown F_IDENT bit even if its version test
 *      were somehow bypassed.
 *
 * This decoder still reads v1 and v2 records: their layouts are strict
 * prefixes, they cannot set the flags their versions predate (the version/flag
 * cross-checks below), and a capture taken before the pair or the identity
 * existed is still a valid capture of what it does hold. What it will not do is
 * treat one as if it carried more than it does — in particular a v1/v2 record
 * has no identity, so stats/phase_join.h refuses to join it rather than
 * assuming its samples abut anything.
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
 * 2 adds the appended uncorrected series (PHASE_REC_F_RAW). 3 adds the appended
 * sample-identity block (PHASE_REC_F_IDENT). Each bump is what stops the
 * previous build's decoder from reading a later record as one of its own; see
 * the versioning note in the banner.
 */
#define PHASE_REC_VER 3u

/** Oldest format version this decoder accepts. */
#define PHASE_REC_VER_MIN 1u

/** Fixed header length, in bytes. */
#define PHASE_REC_HDR_LEN 24u

/** Length of the appended identity block: u32 epoch + u64 seq0. */
#define PHASE_REC_IDENT_LEN 12u

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
 * The record carries the 12-byte sample-identity block after the series, so its
 * samples can be located within a longer capture. Requires ver >= 3.
 *
 * Load-bearing in the same way F_RAW is: it makes the trailing 12 bytes
 * mandatory, and phase_rec_decode_hdr() refuses a record that sets it without
 * the length to back it. Its absence is not an error — a v1/v2 capture is a
 * valid record — but stats/phase_join.h will not join a record that lacks it,
 * because there is then nothing to prove contiguity with.
 */
#define PHASE_REC_F_IDENT 0x04u

/**
 * Every flag bit this build understands. A record setting anything outside this
 * mask is refused — an unknown bit may declare content whose length this
 * decoder cannot account for, and guessing is how a truncated record gets
 * analysed as a whole one.
 */
#define PHASE_REC_F_KNOWN \
	(PHASE_REC_F_FULL | PHASE_REC_F_RAW | PHASE_REC_F_IDENT)

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
	/**
	 * @ref epoch and @ref seq0 are present and meaningful.
	 *
	 * On encode this is the caller's assertion that it has identity to
	 * write, and it is what sets PHASE_REC_F_IDENT. On decode it mirrors
	 * that flag; the two fields read back 0 when it is false, and a
	 * consumer must not read them as "epoch 0, sample 0".
	 */
	bool has_ident;
	/**
	 * Sample-numbering generation. Records with different epochs describe
	 * different, unrelated numberings and can never be joined.
	 */
	uint32_t epoch;
	/** Monotonic index of sample 0 within @ref epoch; sample i is seq0+i. */
	uint64_t seq0;
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
 * Two flags change the length: PHASE_REC_F_RAW appends a second series of the
 * same shape, and PHASE_REC_F_IDENT appends PHASE_REC_IDENT_LEN bytes after it.
 * This is the function that ties the header's declaration to the record's
 * length, and it is what phase_rec_decode_hdr() validates against — so a record
 * cannot claim contents it does not carry.
 */
size_t phase_rec_size_flags(uint16_t n, uint8_t flags);

/**
 * Encode a record into @p buf.
 *
 * @param m        Header fields. @p m->n is the sample count; ver is written
 *                 as PHASE_REC_VER regardless of what the caller set.
 *                 PHASE_REC_F_IDENT is written if and only if @p m->has_ident,
 *                 whatever @p m->flags says — the flag records what was
 *                 written, and only the encoder knows that.
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
 *                   PHASE_REC_F_RAW on a pre-v2 record, PHASE_REC_F_IDENT on a
 *                   pre-v3 record, tau0_ns == 0, gaps > n, or gaps disagreeing
 *                   with the bitmap's population count.
 * @retval -EMSGSIZE @p len is shorter than the record the header declares —
 *                   including when the shortfall is only the raw series or the
 *                   identity block the flags claim.
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

/**
 * Pointer to series @p which inside a record whose header @p m came from a
 * successful phase_rec_decode_hdr() over @p buf.
 *
 * @param which  0 corrected, 1 uncorrected.
 * @return       Start of the series, or NULL for a record that does not carry
 *               @p which (F_RAW clear) or for a NULL argument.
 *
 * This exists so that a consumer which must compare two records SAMPLE BY
 * SAMPLE — stats/phase_join.c, verifying that an overlap agrees — can do it
 * without a staging array the size of the record. phase_rec_decode_samples()
 * would need up to 512 KB of caller buffer per record for that, on top of the
 * joined series itself. The returned bytes are raw little-endian int64;
 * phase_rec_sample() is the reader, so no consumer re-implements the byte
 * order.
 */
const uint8_t *phase_rec_series(const uint8_t *buf, const phase_rec_meta_t *m,
				unsigned int which);

/** Read sample @p i from a pointer returned by phase_rec_series(). */
int64_t phase_rec_sample(const uint8_t *series, size_t i);

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
