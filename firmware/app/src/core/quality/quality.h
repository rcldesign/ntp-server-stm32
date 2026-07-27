/*
 * STS1000 "Meridian" — core/quality: the §3.8 service-quality block.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation, no platform headers, no globals.
 *
 * Spec §3.8 makes `discipline` the single publisher of one authoritative
 * quality block, consumed by NTP, PTP, SNMP, the web/console and the local UI.
 * Services never compute their own view of clock health; they read this. The
 * metric set is spec §10.5.
 *
 * Concurrency (ARCHITECTURE.md §10 invariant 10): management threads must never
 * take a timing lock. Publication is therefore a **seqlock**: the writer bumps
 * an odd sequence, copies, and bumps it even again; readers copy and re-read the
 * sequence, retrying while it is odd or has moved. Readers never block the
 * discipline thread and never block each other.
 *
 *   writer (discipline, 1 Hz, sole writer):  quality_publish()
 *   readers (any thread, any rate):          quality_snapshot()
 *
 * The backing memory is caller-owned (`quality_state_t` lives in the glue's
 * BSS), matching the no-allocation rule.
 *
 * Seqlock caveat, stated plainly: while a reader is copying, the writer may be
 * overwriting the same bytes. The payload copy is therefore a data race in the
 * strict C11 model, resolved the way every production seqlock resolves it — the
 * torn copy is *detected* by the sequence check and discarded, and the two
 * `atomic_thread_fence()` calls plus the `volatile` sequence stop the compiler
 * from hoisting the copy out of the guarded region. This is the standard
 * pattern (Linux `seqcount_t`); it is chosen over a mutex because a mutex would
 * violate the §1.2 rule above, and over double-buffering because the block is
 * ~150 B and readers are far more frequent than writers.
 *
 * There is deliberately no libm dependency anywhere in `core/` — see
 * quality_sqrtf().
 */

#ifndef STS1000_CORE_QUALITY_QUALITY_H_
#define STS1000_CORE_QUALITY_QUALITY_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ types */

/** Layout version of quality_block_t. Bump on any field add/remove/reorder. */
#define QUALITY_BLOCK_VER 1u

/** RFC 5905 "unsynchronised" stratum. Served whenever we are not stratum-1. */
#define QUALITY_STRATUM_UNSYNC 16u

/** Stratum served when the clock is disciplined and inside policy. */
#define QUALITY_STRATUM_PRIMARY 1u

/**
 * Discipline-loop state (spec §11 "Oscillator" / §3.3 / §3.6).
 *
 * `disc` owns the transitions; everything else only reads them. The numeric
 * values are part of the telemetry contract (SNMP/MCP/UI) — append only.
 */
typedef enum {
	QUALITY_LOCK_UNKNOWN = 0,  /**< nothing published yet */
	QUALITY_LOCK_ACQUIRING,    /**< coarse pull-in, FLL leading */
	QUALITY_LOCK_LOCKING,      /**< PI engaged, lock criteria not yet met */
	QUALITY_LOCK_LOCKED,       /**< §3.3 locked criteria satisfied */
	QUALITY_LOCK_HOLDOVER,     /**< §3.6, actuator frozen / flywheeling */
	QUALITY_LOCK_RECOVERING,   /**< §3.6 rate-limited pull-in, never a step */
	QUALITY_LOCK_PARKED,       /**< PFI park: actuator held, loop stopped */
	QUALITY_LOCK__COUNT
} quality_lock_state_t;

/**
 * Which reference the clock mux is forwarding to PH0 (spec §3.5).
 * Owned by `refsel`; `disc` copies it into the block for publication.
 */
typedef enum {
	QUALITY_REF_NONE = 0,
	QUALITY_REF_OCXO,          /**< mux input A, MUX_SEL=0 (boot default) */
	QUALITY_REF_RB,            /**< mux input B, FE-5680A rubidium */
	QUALITY_REF_EXTREF,        /**< mux input B, external house standard */
	QUALITY_REF__COUNT
} quality_ref_t;

/** GNSS fix classification (spec §3.7), reduced to what services need. */
typedef enum {
	QUALITY_GNSS_NO_FIX = 0,
	QUALITY_GNSS_2D,
	QUALITY_GNSS_3D,
	QUALITY_GNSS_TIME_ONLY,    /**< fixed-position timing mode */
	QUALITY_GNSS__COUNT
} quality_gnss_fix_t;

/* Published condition flags. Set by `disc` each tick unless noted sticky. */
#define QUALITY_FLAG_OCXO_WARM        (1u << 0)  /**< oven at temperature */
#define QUALITY_FLAG_GNSS_TIME_LOCKED (1u << 1)  /**< receiver time is usable */
#define QUALITY_FLAG_HOLDOVER         (1u << 2)  /**< §3.6 holdover in effect */
#define QUALITY_FLAG_NO_PPS           (1u << 3)  /**< no capture this second */
#define QUALITY_FLAG_PPS_REJECT       (1u << 4)  /**< median/MAD gate rejected */
#define QUALITY_FLAG_PPS_DIVERGE      (1u << 5)  /**< PA0 vs PC6 disagreement */
#define QUALITY_FLAG_DAC_SAT          (1u << 6)  /**< pull range exhausted */
#define QUALITY_FLAG_DAC_SLEW         (1u << 7)  /**< slew limiter is binding */
#define QUALITY_FLAG_DAC_FAULT        (1u << 8)  /**< Vc cmd vs PA3 divergence */
#define QUALITY_FLAG_TEMPCO           (1u << 9)  /**< tempco feed-forward live */
#define QUALITY_FLAG_PARKED           (1u << 10) /**< PFI park latched */
#define QUALITY_FLAG_DEMOTED          (1u << 11) /**< holdover past policy */
/* Validity of the fields whose zero is otherwise ambiguous (0 mV / 0 m°C are
 * legal readings). A consumer must treat vc_sense_mv / osc_temp_mc as unknown
 * unless the corresponding flag is set. */
#define QUALITY_FLAG_VC_SENSE_VALID   (1u << 12) /**< vc_sense_mv is a reading */
#define QUALITY_FLAG_OSC_TEMP_VALID   (1u << 13) /**< osc_temp_mc is a reading */

/**
 * The one authoritative view of clock quality (spec §3.8, metrics §10.5).
 *
 * This is an in-RAM structure, not a wire format: it is memcpy'd under the
 * seqlock and re-encoded by whichever service needs it (MCP `STATUS_GET`, SNMP,
 * the UI). @ref ver / @ref size exist so a mismatched consumer — a PC tool
 * built against an older header, a log record replayed from NOR — can detect
 * the mismatch instead of misreading the bytes.
 */
typedef struct {
	uint16_t ver;   /**< QUALITY_BLOCK_VER */
	uint16_t size;  /**< sizeof(quality_block_t) as built */

	/** Monotonic ms at publication; consumers use it to detect staleness. */
	uint64_t updated_mono_ms;
	/** Publication ordinal, stamped by quality_publish(): 1 for the first
	 *  published block and +1 for each after it. A consumer that keeps the
	 *  previous value sees exactly how many updates it missed. */
	uint32_t tick;

	uint8_t stratum;     /**< 1 when serving primary, else 16 */
	uint8_t lock_state;  /**< quality_lock_state_t */
	uint8_t active_ref;  /**< quality_ref_t */
	uint8_t gnss_fix;    /**< quality_gnss_fix_t */

	bool holdover;       /**< mirror of QUALITY_FLAG_HOLDOVER, for cheap tests */
	bool utc_valid;      /**< UTC (leap offset) is known and applied */
	uint8_t gnss_sv_used;
	uint8_t gnss_sv_visible;

	/** NTP reference identifier, host byte order (see quality_refid()). */
	uint32_t refid;
	/** RFC 5905 root delay, NTP short format (16.16 s). 0 for a GPS primary. */
	uint32_t root_delay_q16;
	/** RFC 5905 root dispersion, NTP short format (16.16 s). Grows in holdover. */
	uint32_t root_disp_q16;

	/** §3.6 estimated time error accumulated since holdover entry. */
	int64_t holdover_est_err_ns;
	/** Seconds elapsed since holdover entry (0 when not in holdover). */
	uint32_t holdover_elapsed_s;
	/** Seconds until stratum demotion; UINT32_MAX when not on the horizon. */
	uint32_t holdover_t_demote_s;

	int32_t last_pps_off_ns;    /**< last accepted, fully corrected PPS offset */
	float pps_off_mean_ns;      /**< rolling mean over the lock window */
	float pps_off_sigma_ns;     /**< rolling standard deviation, same window */
	float freq_err_ppb;         /**< filtered residual fractional frequency */

	int32_t vc_cmd_mv;          /**< commanded OCXO Vc (DAC1_OUT1, PA4) */
	int32_t vc_sense_mv;        /**< sensed OCXO Vc (OCXO_V, PA3) */
	uint16_t dac_code;          /**< raw 12-bit DAC code held on PA4 */

	float adev_1s;              /**< overlapping ADEV, tau = 1 s (0 = n/a) */
	float adev_10s;             /**< overlapping ADEV, tau = 10 s */
	float adev_100s;            /**< overlapping ADEV, tau = 100 s */

	uint32_t gnss_tacc_ns;      /**< receiver time accuracy estimate */

	int8_t leap_pending;        /**< +1 insert, 0 none, -1 delete */
	int16_t leap_current_s;     /**< current TAI-UTC offset, seconds */
	/** TAI seconds (PTP epoch, see port_time.h) at which @ref leap_pending
	 *  takes effect. Meaningless when @ref leap_pending is 0. */
	uint64_t leap_at_tai_s;

	int32_t osc_temp_mc;        /**< TMP117 #1 oscillator temperature, m°C */
	uint32_t flags;             /**< QUALITY_FLAG_* */
} quality_block_t;

/**
 * Optional observer invoked by quality_snapshot() between the payload copy and
 * the closing sequence read.
 *
 * Production code leaves this NULL — the Zephyr glue never sets it. It exists
 * so the seqlock's retry path is reachable from a single-threaded host test:
 * the hook stands in for the concurrent writer that a test binary cannot have.
 */
typedef void (*quality_race_hook_t)(void *hook_ctx);

/**
 * Seqlock-published quality state. Caller-owned memory (no allocation in core).
 *
 * All fields are private; use the quality_* functions.
 */
typedef struct {
	/** Even = stable, odd = write in progress. C11 atomic so concurrent
	 *  access from the writer and any reader is a well-defined operation
	 *  rather than a data race, and so the two reads in quality_snapshot()
	 *  cannot be folded into one. Accessed relaxed under the release/acquire
	 *  fences in quality.c. */
	_Atomic uint32_t seq;
	quality_race_hook_t race_hook;
	void *hook_ctx;
	quality_block_t blk;
} quality_state_t;

/** Snapshot attempts before quality_snapshot() gives up with -EAGAIN. */
#define QUALITY_SNAPSHOT_RETRIES 8u

/* --------------------------------------------------------------- lifecycle */

/**
 * Initialise a block to safe "nothing known yet" values: stratum 16, lock state
 * UNKNOWN, reference NONE, dispersion 0, all counters zero.
 *
 * @param b  Block to initialise. NULL is a no-op.
 */
void quality_block_init(quality_block_t *b);

/**
 * Initialise a publication slot. The embedded block is quality_block_init()'d
 * and the sequence starts even, so an early reader gets a valid "nothing known"
 * snapshot rather than -EAGAIN.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p qs is NULL.
 */
int quality_state_init(quality_state_t *qs);

/* ------------------------------------------------------------ publish/read */

/**
 * Publish a new block (single writer only — the `discipline` thread).
 *
 * @ref quality_block_t::ver, @ref quality_block_t::size and
 * @ref quality_block_t::tick are stamped by this function (the tick from the
 * slot's own publication count), so a caller that memset() its scratch block
 * still publishes a well-formed one.
 *
 * @param qs   Publication slot.
 * @param blk  Block to publish; copied, not retained.
 *
 * @retval 0        Published.
 * @retval -EINVAL  @p qs or @p blk is NULL.
 */
int quality_publish(quality_state_t *qs, const quality_block_t *blk);

/**
 * Take a consistent snapshot (any reader thread, lock-free).
 *
 * @param qs   Publication slot.
 * @param out  Receives the block. Untouched on failure.
 *
 * @retval 0        @p out holds a consistent snapshot.
 * @retval -EINVAL  @p qs or @p out is NULL.
 * @retval -EAGAIN  The writer won QUALITY_SNAPSHOT_RETRIES races in a row.
 *                  The caller should retry later or keep its previous value;
 *                  at a 1 Hz writer this cannot happen in practice.
 */
int quality_snapshot(const quality_state_t *qs, quality_block_t *out);

/* ------------------------------------------------------------- conversions */

/**
 * Nanoseconds → NTP short format (RFC 5905 §6: unsigned 16.16 fixed-point
 * seconds), the encoding of root delay and root dispersion.
 *
 * Saturating and non-negative: a negative input yields 0 (a delay or dispersion
 * is never negative), and anything at or beyond 65536 s yields 0xFFFFFFFF.
 * Rounds to nearest.
 */
uint32_t quality_ntp_short_from_ns(int64_t ns);

/** NTP short format → nanoseconds, rounded to nearest. Exact inverse at the
 *  representable points (65536 units == 1 s == 1e9 ns). */
int64_t quality_ns_from_ntp_short(uint32_t q16);

/**
 * Pack four ASCII characters into an NTP reference identifier (RFC 5905 §7.3:
 * a left-justified, zero-padded four-octet string for stratum 1).
 *
 * Host byte order, first character in the most significant octet, matching how
 * the field is serialised big-endian on the wire.
 */
uint32_t quality_refid(char a, char b, char c, char d);

/* ------------------------------------------------------- holdover growth */

/**
 * Holdover error-growth model (spec §3.6, characterised per §10.4).
 *
 *     err(t) = base + (drift + temp * |dT|) * t + 0.5 * aging * t^2
 *
 * The linear form asked for by §3.6 ("increment root dispersion per the active
 * oscillator's characterised drift", plus a temperature term) is the
 * @ref aging_ns_per_s2 == 0 case. The quadratic term carries oscillator ageing,
 * which dominates multi-hour holdover on an OCXO.
 *
 * A negative @ref aging_ns_per_s2 is clamped to zero by both helpers: the
 * estimator must never predict an oscillator that improves with time, because
 * that would under-report dispersion to NTP clients.
 */
typedef struct {
	float base_ns;             /**< error already present at holdover entry */
	float drift_ns_per_s;      /**< characterised linear drift (1 ns/s = 1 ppb) */
	float aging_ns_per_s2;     /**< ageing acceleration; 0 for the linear model */
	float temp_ns_per_s_per_c; /**< extra drift per °C of excursion from entry */
} quality_holdover_model_t;

/**
 * Evaluate the holdover model.
 *
 * @param m     Model. NULL yields 0.
 * @param t_s   Elapsed holdover time, seconds. Negative/NaN is treated as 0.
 * @param dt_c  Temperature excursion from holdover entry, °C (sign ignored).
 * @return      Estimated time error in nanoseconds, never negative.
 */
float quality_holdover_err_ns(const quality_holdover_model_t *m, float t_s,
			      float dt_c);

/**
 * Invert the holdover model: how long until the error reaches @p threshold_ns.
 *
 * @param m             Model. NULL yields INFINITY.
 * @param dt_c          Temperature excursion, °C (sign ignored).
 * @param threshold_ns  Error budget being tested against.
 * @return              Seconds until the threshold is crossed; 0 if it is
 *                      already exceeded at t=0; INFINITY if the model never
 *                      reaches it (no drift, no ageing, no temperature term).
 */
float quality_holdover_time_to_ns(const quality_holdover_model_t *m, float dt_c,
				  float threshold_ns);

/* --------------------------------------------------------------- leap smear */

/**
 * Leap-second smear model (spec §15.2) — the *shape*, not the policy.
 *
 * This is the leap-second counterpart of the holdover model above: a pure
 * function of the published §3.8 block, computing no policy of its own. Whether
 * a smear is applied at all, and what a service gives up while it is, is
 * decided by the consuming service — today only `core/ntp`. The model lives
 * here for the same reason the holdover growth curve does: it is derived from
 * the block's own fields (@ref quality_block_t::leap_pending,
 * @ref quality_block_t::leap_at_tai_s), so every management surface can
 * recompute exactly what the server is doing from the snapshot it already
 * reads, with no second publisher and no field for `disc` to fill.
 *
 * **Shape: a linear ramp over a window that ENDS at the leap instant.**
 *
 * Written out, with `L` the leap instant, `W` the window and `u = (t − (L−W))/W`
 * the ramp phase, the served timescale during the window is
 *
 *     served(t) = TAI(t) − leap_current_s − sign · u
 *
 * where `sign` is +1 for an insert and −1 for a delete. At `u = 0` that is
 * exactly the pre-leap UTC; at `u → 1` it is exactly the post-leap UTC, which
 * is what @ref quality_block_t::leap_current_s becomes the moment the event
 * fires. The served timescale is therefore continuous across the leap — the
 * whole point — and back on true UTC from the leap instant onwards.
 *
 * Three properties follow, and all three are load-bearing:
 *
 *  1. **Monotonic, by construction.** The correction is `elapsed_ns / W_s`, an
 *     integer division: non-decreasing in elapsed time, and bounded above by
 *     1e9 − 1 ns. Served time therefore advances at `1 ∓ 1/W` s/s, which over
 *     the permitted window range is between 0.99993 and 1.00007 — a client can
 *     never read an earlier instant than one it has already read. No table, no
 *     polynomial, no floating point, nothing to round the wrong way.
 *
 *     Stated exactly, because the difference matters at the nanosecond the
 *     division ticks: during an *insert* the served instant is non-decreasing
 *     and **stalls for exactly one nanosecond every W nanoseconds** — t gains
 *     1 ns and the correction gains 1 ns with it. That is quantisation, not a
 *     defect: a timescale running at 0.99993 ns per ns cannot be expressed in
 *     integer nanoseconds except by occasionally skipping one, and any other
 *     shape has the same floor. A *delete* runs the correction the other way
 *     and is strictly increasing throughout. Neither ever reverses, which is
 *     the property clients depend on; test_quality.c pins both, including the
 *     stall count, at nanosecond resolution.
 *  2. **It terminates exactly.** At `elapsed = W` the correction is exactly
 *     1e9 ns, so the ramp has absorbed precisely one second by the instant
 *     `leap_current_s` moves. There is no residue to step away afterwards.
 *  3. **It is computable from a snapshot alone.** A window *centred* on the
 *     leap (the Google/AWS convention) spends half its length after the event,
 *     by which time `leap_pending` is 0 and `leap_at_tai_s` is meaningless —
 *     reconstructing the post-half would need `disc` to retain the past leap
 *     and publish it. A leading window needs nothing that is not already in
 *     the block, so the NTP thread, SNMP, the web UI and the console all derive
 *     the identical state and cannot disagree.
 *
 * The price of (3) is the peak deviation from true UTC: a leading window drifts
 * to 1 s just before the event, where a centred window stays inside ±0.5 s. The
 * trade is deliberate — a leading window is wrong for the same *total* number
 * of second-seconds, is finished the moment the leap fires rather than half an
 * announce-window later, and never depends on state that has already expired.
 *
 * A linear ramp, rather than a raised cosine, for the same reason: it is what
 * every deployed smearing service emits, so client servos are tuned for it; a
 * constant `1/W` frequency offset (11.574 ppm at the default window) is a
 * number an operator can verify with one `chronyc tracking` reading; and it is
 * exactly monotone in integer arithmetic, where a cosine would need a table
 * and a rounding argument to make the same guarantee.
 */

/**
 * Shortest permitted smear window, seconds (4 h).
 *
 * The bound is a client-tolerance one, not an arbitrary round number. The ramp
 * presents itself to every client as a constant frequency offset of `1/W`;
 * at 14400 s that is 69.4 ppm, about a seventh of the 500 ppm at which ntpd
 * (`NTP_MAXFREQ`) and chrony (`maxdrift`) stop believing a source, leaving the
 * rest of the budget for the client's own oscillator error. A much shorter
 * window would be monotone and would still terminate exactly, and would still
 * push some clients out of capture range.
 */
#define QUALITY_SMEAR_WINDOW_MIN_S UINT32_C(14400)

/**
 * Longest permitted smear window, seconds (24 h).
 *
 * Equal to the NTP leap-announcement window (NTP_LEAP_ANNOUNCE_WINDOW_S), which
 * is not a coincidence: at the maximum the smear occupies exactly the interval
 * over which the server would otherwise have been announcing the leap, so
 * "smearing" and "would have warned" are one interval rather than two. It also
 * bounds how long a smearing server spends off its stratum-1 claim.
 */
#define QUALITY_SMEAR_WINDOW_MAX_S UINT32_C(86400)

/** Default window, seconds: 24 h, the industry convention, 11.574 ppm. */
#define QUALITY_SMEAR_WINDOW_DEFAULT_S UINT32_C(86400)

/** Live smear state derived by quality_leap_smear(). */
typedef struct {
	/** A smear is in progress right now. All other fields describe it. */
	bool active;
	/** +1 for an insert, −1 for a delete, 0 when not active. */
	int8_t direction;
	/**
	 * The correction, nanoseconds, in [−(1e9 − 1), +(1e9 − 1)].
	 *
	 * Sign convention: **subtract this from a TAI instant** before applying
	 * the TAI−UTC offset. Positive for an insert (the served timescale is
	 * held back), negative for a delete (it is run ahead). 0 when inactive,
	 * so an unconditional subtraction is a no-op outside the window.
	 */
	int32_t offset_ns;
	/** Seconds elapsed into the window. */
	uint32_t elapsed_s;
	/** Seconds remaining until the leap, i.e. until the ramp completes. */
	uint32_t remaining_s;
	/**
	 * Effective window after clamping, seconds; 0 means smearing is not
	 * configured at all. Set even when @ref active is false, so a caller can
	 * distinguish "configured, not yet in the window" from "switched off" —
	 * a distinction NTP needs, because a server configured to smear must not
	 * announce the leap even before the ramp starts.
	 */
	uint32_t window_s;
} quality_smear_t;

/**
 * Clamp a requested smear window into the permitted band.
 *
 * @param window_s  Requested window; 0 means "no smearing".
 * @return          0 for 0, otherwise @p window_s clamped into
 *                  [QUALITY_SMEAR_WINDOW_MIN_S, QUALITY_SMEAR_WINDOW_MAX_S].
 *                  A non-zero request never clamps to 0: an operator who asked
 *                  for a smear gets a safe one, never a silent step.
 */
uint32_t quality_smear_window_clamp(uint32_t window_s);

/**
 * Evaluate the smear model against a §3.8 snapshot.
 *
 * Pure: no state, no side effects, safe to call from any thread on a snapshot
 * it already holds (ARCHITECTURE.md §10 invariant 10 — no timing lock is taken,
 * because none is needed).
 *
 * @param b           Snapshot to evaluate. Not retained.
 * @param window_s    Requested window, clamped internally; 0 disables.
 * @param now_tai_ns  Current TAI nanoseconds; 0 means "no time", which yields
 *                    an inactive result rather than an invented ramp phase.
 * @param out         Receives the state; zeroed first, so every field is
 *                    defined on every return path.
 *
 * @retval 0        @p out is populated (possibly inactive).
 * @retval -EINVAL  @p out is NULL, or @p b is NULL.
 */
int quality_leap_smear(const quality_block_t *b, uint32_t window_s,
		       uint64_t now_tai_ns, quality_smear_t *out);

/* ------------------------------------------------------------------- misc */

/**
 * Square root of a non-negative float, computed without libm.
 *
 * `core/` must not depend on a math library: the host test harness links none,
 * and on target the only float square roots in the timing path are these, at
 * 1 Hz. The implementation is an exponent-halving seed followed by four
 * Newton–Raphson steps, which reaches ~1 ulp across the whole normal range
 * (measured worst-case relative error 9e-8) and needs no table.
 *
 * It lives here, rather than in a module of its own, because `quality` is the
 * lowest module that both the holdover inverse and `disc` (ADEV, sigma) share.
 *
 * @param x  Radicand.
 * @return   sqrt(@p x) for x > 0; 0 for zero, negative or NaN input; +inf for
 *           +inf. Never raises, never sets errno, never traps.
 */
float quality_sqrtf(float x);

/** Human-readable lock-state name, for logs and the UI. Never NULL. */
const char *quality_lock_state_name(uint8_t state);

/** Human-readable reference name, for logs and the UI. Never NULL. */
const char *quality_ref_name(uint8_t ref);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_QUALITY_QUALITY_H_ */
