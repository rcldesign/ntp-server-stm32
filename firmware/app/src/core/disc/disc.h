/*
 * STS1000 "Meridian" — core/disc: the OCXO disciplining engine.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation, no platform headers, no globals,
 * no libm (see quality_sqrtf()). All state lives in a caller-owned disc_ctx_t.
 *
 * Implements spec §3.2 (PPS capture conditioning), §3.3 (FLL+PI loop, actuator
 * scaling, tempco feed-forward, lock criteria) and §3.6 (holdover engine), and
 * publishes the §3.8 quality block. Hardware contract:
 * docs/sts1000_firmware_hardware_interface.md §3.
 *
 * ARCHITECTURE.md §10 invariant 2: the DAC on PA4 is written by the discipline
 * thread and nobody else. This module never touches hardware — it returns the
 * code to write — but that invariant is why disc_out_t::dac_code has exactly
 * one legitimate consumer.
 *
 * ==========================================================================
 * SIGN CONVENTIONS — get these wrong and the loop runs away
 * ==========================================================================
 *
 * Phase error `e`:  e = t_true - t_local, in nanoseconds.
 *
 *   e > 0  =>  the local clock reads BEHIND true UTC: it is LATE and must
 *              speed up.
 *   e < 0  =>  the local clock is EARLY and must slow down.
 *
 * It is derived from the capture as
 *
 *       e_raw = (int32_t)(expected_count - captured_count) * ns_per_count
 *
 * where `captured_count` is the free-running timer value latched by the GNSS
 * PPS edge and `expected_count` is the count at which the local clock predicted
 * the top of second. The subtraction is done in uint32 and reinterpreted as
 * int32, so it is correct across the ~17.18 s wrap of a 32-bit counter at
 * 250 MHz. Worked example: the true second arrives *before* the local clock
 * reaches its predicted mark, so captured < expected, so e > 0 — the local
 * clock has not got there yet, i.e. it is late. Correct.
 *
 * Corrections (§3.2 steps 2 and 3) all move the *measured* edge earlier, and
 * therefore all add to e:
 *
 *       e = e_raw + qErr_ns + cable_delay_ns + board_delay_ns
 *
 *   - qErr: UBX-TIM-TP reports the receiver's quantisation error for the pulse
 *     it has just tagged. The pulse fires qErr *after* the ideal instant, so
 *     the true second is at (capture - qErr) and e gains +qErr. This matches
 *     the convention used by gpsd and chrony. disc_cfg_t::qerr_sign exists
 *     because the polarity is worth confirming on the bench against a
 *     reference PPS before it is trusted (bench item, §10.4 "PPS / board
 *     routing offset").
 *   - Cable and board delay: the edge arrives late by the propagation delay,
 *     so the same +correction refers it back to the antenna phase centre.
 *
 * Frequency `y`, in ppb (parts per 10^9). Because 1 ppb of a 1 s interval is
 * exactly 1 ns, ppb and ns/s are the same number here:
 *
 *   y > 0  =>  the local clock runs FAST.       de/dt = -y.
 *
 * The command sent to the OCXO has the same sign: a positive command raises the
 * OCXO frequency (positive Vc slope). An oscillator with an inverting tuning
 * slope is handled by a negative disc_cfg_t::dac_full_scale_ppb, not by
 * flipping anything else.
 *
 * ==========================================================================
 * CONTROLLER
 * ==========================================================================
 *
 * The plant from frequency command to phase error is a pure integrator with
 * unity gain in these units (de/dt = -y). With a PI controller
 *
 *       y = Kp*e + Ki*integral(e dt)
 *
 * the closed loop is  e'' + Kp*e' + Ki*e = 0, i.e. a standard second-order
 * system with wn = sqrt(Ki) and 2*zeta*wn = Kp. Parameterising by the loop time
 * constant tau and a damping constant a:
 *
 *       Kp = 1 / tau                 [ppb per ns]      = 1/s
 *       Ki = 1 / (a * tau^2)         [ppb per ns per s] = 1/s^2
 *
 * gives wn = 1/(tau*sqrt(a)) and zeta = sqrt(a)/2. The default a = 2 is
 * therefore zeta = 0.707 (Butterworth, ~4 % overshoot, fastest settle without
 * ringing); a = 4 is critically damped. tau is the §3.3 configurable
 * 10..1000 s.
 *
 * During ACQUIRING the same integrator is driven by an FLL instead of by Ki*e,
 * which is what makes the handover bumpless — there is only ever one frequency
 * state. The FLL estimates the oscillator's own offset from what actually
 * happened over the last interval:
 *
 *       f_obs   = -(e[k] - e[k-1]) / dt        (measured local frequency error)
 *       f_osc   = y_applied - f_obs            (the oscillator's contribution)
 *       I      += g * ((f_osc - ff) - I)
 *
 * `y_applied` is the command that was really in effect (post clamp, post slew),
 * so saturation does not corrupt the estimate: with the phase term railed at
 * +400 ppb, y_applied - f_obs still isolates the oscillator term exactly.
 *
 * ==========================================================================
 */

#ifndef STS1000_CORE_DISC_DISC_H_
#define STS1000_CORE_DISC_DISC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "quality/quality.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------ dimensioning */

/** Maximum median/MAD window. disc_cfg_t::mad_window must not exceed it. */
#define DISC_MAD_WIN_MAX 16u

/** Maximum lock dwell / rolling-variance window, in samples (= seconds). */
#define DISC_LOCK_WIN_MAX 64u

/**
 * Phase-sample ring feeding the ADEV estimator, in samples (= seconds).
 *
 * Memory bound: 512 * sizeof(float) = 2048 B, the dominant term in
 * sizeof(disc_ctx_t). 512 is chosen from the largest tau reported: an
 * overlapping estimate at tau = 100 s needs at least 201 samples and gets
 * 512 - 200 = 312 averaging terms here, which is a usable confidence interval.
 * Halving it to 256 would leave 56.
 */
#define DISC_ADEV_CAP 512u

/**
 * Nominal spacing of the ADEV phase ring, in nanoseconds.
 *
 * The ring is fed one sample per accepted PPS second, and the runtime
 * estimator is called with tau0 = 1.0 s. This constant is the same number
 * spelled for consumers that must state the record's sample interval
 * explicitly — @ref disc_phase_snapshot exports it so an offline analyser
 * never has to assume it.
 */
#define DISC_ADEV_TAU0_NS 1000000000u

/** Bytes in the per-sample gap bitmap that shadows the ADEV ring. */
#define DISC_ADEV_GAPMAP_BYTES (DISC_ADEV_CAP / 8u)

/* ------------------------------------------------------------------ states */

/**
 * Discipline state machine (spec §3.3 lock criteria, §3.6 holdover).
 *
 *   ACQUIRING --|e| small--> LOCKING --criteria met--> LOCKED
 *       ^                       |                        |
 *       |                       v                        v
 *       +---|e| large-----------+                    HOLDOVER
 *                               ^                        |
 *                               |                        v
 *                               +----criteria met---- RECOVERING
 *
 * HOLDOVER is entered only from LOCKING/LOCKED/RECOVERING: from ACQUIRING there
 * is no frequency estimate to fly on, so PPS loss just freezes the actuator and
 * stays in ACQUIRING. PARKED is entered only by disc_park() and left only by
 * disc_unpark().
 */
typedef enum {
	DISC_STATE_ACQUIRING = 0,
	DISC_STATE_LOCKING,
	DISC_STATE_LOCKED,
	DISC_STATE_HOLDOVER,
	DISC_STATE_RECOVERING,
	DISC_STATE_PARKED,
	DISC_STATE__COUNT
} disc_state_t;

/* ------------------------------------------------------------------ inputs */

/**
 * One second's PPS capture pair (spec §3.2).
 *
 * The primary is TIMEPULSE on PA0/TIM2_CH1, the secondary TIMEPULSE2 on
 * PC6/TIM3_CH1 (interface ref §3). Both are free-running 32-bit captures; the
 * caller supplies, per channel, the count its own timebase predicted for the
 * top of second and the nanoseconds-per-count scale of that timer.
 */
typedef struct {
	uint32_t primary_count;      /**< PA0/TIM2_CH1 capture value */
	uint32_t primary_expected;   /**< predicted count at top of second */
	float primary_ns_per_count;  /**< e.g. 4.0 for TIM2 at 250 MHz */
	bool primary_valid;

	uint32_t secondary_count;    /**< PC6/TIM3_CH1 capture value */
	uint32_t secondary_expected;
	float secondary_ns_per_count;
	bool secondary_valid;

	/** UBX-TIM-TP quantisation error for the tagged pulse, picoseconds. */
	int32_t qerr_ps;
	bool qerr_valid;
} disc_pps_t;

/**
 * Fields the discipline thread copies straight into the quality block but does
 * not own: they come from `gnssmgr` (§3.7) and `refsel` (§3.5).
 */
typedef struct {
	uint8_t active_ref;       /**< quality_ref_t, from refsel */
	uint8_t gnss_fix;         /**< quality_gnss_fix_t */
	uint8_t gnss_sv_used;
	uint8_t gnss_sv_visible;
	uint32_t gnss_tacc_ns;
	int8_t leap_pending;      /**< +1 insert, 0 none, -1 delete */
	int16_t leap_current_s;   /**< current TAI-UTC offset */
	uint64_t leap_at_tai_s;   /**< epoch of the pending leap */
	bool utc_valid;
	uint32_t refid;           /**< 0 selects the default, "GPS\0" */
	int32_t root_delay_ns;    /**< 0 for a GNSS primary (RFC 5905) */
} disc_anc_t;

/**
 * Everything the loop needs every second regardless of whether a PPS arrived.
 */
typedef struct {
	uint64_t mono_ms;         /**< monotonic milliseconds (port_time.h) */

	/** UBX-NAV-* says receiver time is usable. False forces holdover. */
	bool gnss_time_locked;

	/**
	 * The served timescale has a valid absolute epoch.
	 *
	 * PPS lock proves *rate*, not *epoch*: a disciplined oscillator tracking a
	 * 1 Hz edge says nothing about which second it is. Until the served clock
	 * has actually been set — on this board, the ETH PTP hardware clock loaded
	 * from GNSS and its servo reporting synchronised — a primary stratum would
	 * advertise a wrong absolute time with a small dispersion, which is worse
	 * than advertising nothing (RFC 5905 §11.1). served_stratum() therefore
	 * returns UNSYNC while this is false, however well the loop is locked.
	 *
	 * Defaults false: a caller that does not know must not claim traceability.
	 */
	bool timebase_traceable;

	int32_t osc_temp_mc;      /**< TMP117 #1 (0x49), milli-°C */
	bool osc_temp_valid;

	int32_t ocxo_current_ua;  /**< INA228 0x46 rail current, microamps */
	bool ocxo_current_valid;

	int32_t vc_sense_mv;      /**< OCXO_V on PA3, millivolts */
	bool vc_sense_valid;

	disc_anc_t anc;
} disc_env_t;

/** A second with a PPS capture. */
typedef struct {
	disc_env_t env;
	disc_pps_t pps;
} disc_in_t;

/* ------------------------------------------------------------------ config */

/**
 * Loop configuration. disc_cfg_defaults() fills sane, documented values; the
 * `timing` config group (ARCHITECTURE.md §8, group 0x06) overrides them.
 *
 * Values marked BENCH must be characterised on hardware before the numbers
 * mean anything — they are placeholders chosen to be safe, not correct.
 */
typedef struct {
	/* ---- §3.2 sample conditioning ---- */
	int32_t cable_delay_ns;    /**< BENCH §10.4 antenna cable delay */
	int32_t board_delay_ns;    /**< BENCH §10.4 board PPS routing delay */
	int8_t qerr_sign;          /**< +1 (default) or -1; see header banner */
	uint32_t xcheck_tol_ns;    /**< PA0 vs PC6 divergence tolerance (250) */
	uint8_t mad_window;        /**< median/MAD window, 3..DISC_MAD_WIN_MAX */
	float mad_k;               /**< MAD multiplier (5.0) */
	float mad_floor_ns;        /**< minimum acceptance half-width (100) */
	uint8_t max_consec_reject; /**< force-accept after this many rejects (5) */

	/* ---- §3.3 controller ---- */
	float tau_s;               /**< loop time constant, 10..1000 (150) */
	float pi_damping_a;        /**< zeta = sqrt(a)/2; 2.0 => 0.707 */
	float fll_gain;            /**< ACQUIRING FLL gain, 0..1 (0.25) */
	float acq_exit_ns;         /**< ACQUIRING -> LOCKING below this (10000) */
	float acq_reentry_ns;      /**< LOCKING -> ACQUIRING above this (50000) */
	uint16_t acq_min_ticks;    /**< minimum accepted samples in ACQUIRING (8) */
	float acq_ramp_max_ppb;    /**< phase-term clamp while ACQUIRING (400) */
	float recover_ramp_max_ppb;/**< §3.6 pull-in rate limit, ns/s (50) */

	/* ---- actuator (interface ref §3) ---- */
	float dac_full_scale_ppb;  /**< BENCH pull at full scale; +-0.4 ppm => 400 */
	uint16_t dac_center_code;  /**< code for 1.65 V (2048) */
	uint16_t dac_max_code;     /**< 4095 for the 12-bit DAC1_OUT1 */
	uint16_t dac_vref_mv;      /**< DAC reference, 3300 */
	/** Slew limit while serving a primary stratum, LSB/s (5). */
	float slew_lsb_per_s;
	/** Slew limit while converging (ACQUIRING/LOCKING), LSB/s (256). */
	float slew_acq_lsb_per_s;

	/* ---- §3.3 tempco feed-forward ---- */
	float tempco_ppb_per_c;    /**< BENCH oscillator df/dT; see disc_tempco_fit */
	bool tempco_enable;

	/* ---- §3.3 lock criteria ---- */
	float lock_phase_ns;       /**< |e| threshold (500) */
	float lock_var_ns2;        /**< rolling variance threshold (40000 = 200 ns) */
	uint8_t lock_dwell_s;      /**< consecutive good seconds, <= 64 (30) */
	uint8_t unlock_dwell_s;    /**< consecutive bad seconds to drop lock (10) */

	/* ---- OCXO warm detection (INA228 0x46 + TMP117 0x49) ---- */
	int32_t warm_temp_mc;      /**< BENCH case temperature when warm (40000) */
	int32_t warm_current_ua;   /**< BENCH steady current ceiling (700000) */
	uint16_t warm_dwell_s;     /**< both conditions held this long (60) */
	uint16_t warm_timeout_s;   /**< sensorless fallback after this long (600) */

	/* ---- §3.6 holdover ---- */
	uint8_t pps_loss_ticks;    /**< missing/rejected seconds before holdover (3) */
	quality_holdover_model_t holdover; /**< BENCH §10.4 drift model */
	float demote_threshold_ns; /**< error budget before demotion (1e6 = 1 ms) */
	uint8_t holdover_stratum;  /**< stratum once demoted (16) */

	/* ---- Vc sense cross-check (PA3) ---- */
	int32_t vc_tol_mv;         /**< |cmd - sense| tolerance (100) */
	uint8_t vc_fault_dwell_s;  /**< consecutive seconds before fault (3) */

	/* ---- served dispersion ---- */
	float base_disp_ns;        /**< dispersion floor while locked (1000) */
} disc_cfg_t;

/* ----------------------------------------------------------------- context */

/** Loop state. Caller-owned, opaque in practice — use the disc_* functions. */
typedef struct {
	disc_cfg_t cfg;

	/* Derived from cfg at init, cached to keep the tick divide-free. */
	float kp;                  /* ppb per ns */
	float ki;                  /* ppb per ns per s */
	float ppb_per_code;        /* signed: tracks dac_full_scale_ppb */
	float y_range_lo;          /* ppb at code 0 or dac_max_code */
	float y_range_hi;

	bool initialised;
	disc_state_t state;

	/* Actuator */
	uint16_t dac_code;
	float y_int;               /* integrator == frequency command, ppb */
	float y_eff_prev;          /* command actually applied last interval */
	float ff_ppb;              /* tempco feed-forward currently applied */

	/* Sample history */
	bool have_prev;
	float e_prev;
	uint64_t prev_mono_ms;
	uint64_t first_mono_ms;
	bool have_first;

	/* Median/MAD gate */
	float mad_buf[DISC_MAD_WIN_MAX];
	uint8_t mad_n;
	uint8_t mad_head;
	uint8_t consec_reject;

	/* Lock dwell + rolling variance */
	float lock_buf[DISC_LOCK_WIN_MAX];
	uint8_t lock_n;
	uint8_t lock_head;
	uint16_t lock_run;
	uint16_t unlock_run;

	/* ADEV */
	float adev_buf[DISC_ADEV_CAP];
	uint16_t adev_n;
	uint16_t adev_head;
	/**
	 * Per-sample "this sample was appended after a non-uniform interval"
	 * bitmap, one bit per adev_buf[] slot (LSB-first: sample i is bit i&7
	 * of byte i>>3). It shadows adev_buf exactly, including the shift-down
	 * eviction, so a gap scrolls out of the window with the sample that
	 * carried it.
	 */
	uint8_t adev_gapmap[DISC_ADEV_GAPMAP_BYTES];
	/** Population count of adev_gapmap over the live window [0, adev_n). */
	uint16_t adev_gaps;
	float adev_1s;
	float adev_10s;
	float adev_100s;

	/* Warm-up */
	uint16_t warm_run;
	bool ocxo_warm;

	/* Vc cross-check */
	uint8_t vc_bad_run;
	bool vc_fault;

	/* Holdover. The error estimate is ACCUMULATED, not recomputed from the
	 * closed form each tick: RFC 5905 requires root dispersion to grow
	 * monotonically through an outage, and a from-scratch recompute drops
	 * when the temperature returns toward its entry value or a sensor
	 * reading is lost. holdover_last_dt_c holds the most recent known
	 * temperature excursion so a dropped TMP117 read freezes the growth
	 * rate instead of zeroing it. */
	uint64_t holdover_start_ms;
	uint64_t holdover_prev_ms;      /* last update, for the incremental dt */
	int32_t holdover_start_temp_mc;
	bool holdover_start_temp_valid;
	float holdover_last_dt_c;       /* last known excursion from entry, °C */
	uint32_t holdover_elapsed_s;
	float holdover_est_ns;          /* monotonic non-decreasing while held */
	uint32_t t_demote_s;

	/*
	 * The accumulated error the loop is still carrying, retained across
	 * RECOVERING and PARKED and cleared only when LOCKED is genuinely
	 * re-achieved.
	 *
	 * Without it, one accepted PPS after a 16-hour holdover — or any refsel
	 * park/unpark bracket — moved the loop to RECOVERING, where the estimate
	 * was abandoned and the served dispersion collapsed to |last_e_ns|. The
	 * clock advertised stratum 1 with a dispersion two orders of magnitude
	 * below its actual error, while the recovery ramp had corrected almost
	 * none of the phase. RFC 5905 §11.1 requires dispersion to *bound* the
	 * error, so the retained value is the floor for both the dispersion and
	 * the demotion decision until the phase is demonstrably back.
	 */
	float holdover_retained_ns;
	/* Elapsed seconds already banked by earlier legs of this outage, so the
	 * published holdover_elapsed_s is monotonic across the whole episode
	 * rather than restarting on every brief return of the reference. */
	uint32_t holdover_elapsed_base_s;

	/* Tempco reference temperature, captured on first lock */
	bool tref_valid;
	int32_t tref_mc;

	/* Set the first time LOCKED is reached; gates the primary stratum in
	 * holdover and recovery. Never cleared. */
	bool ever_locked;

	/* Latest disc_env_t::timebase_traceable. Held here because the stratum
	 * policy is a function of the context alone. */
	bool timebase_traceable;

	/* Accepted samples since ACQUIRING was last entered. */
	uint32_t acq_ticks;

	/* Telemetry */
	float last_e_ns;
	bool last_e_valid;
	float freq_err_ppb;
	float pps_mean_ns;
	float pps_sigma_ns;
	int32_t vc_cmd_mv;
	int32_t vc_sense_mv;
	bool vc_sense_valid;
	uint32_t flags;
	uint32_t miss_run;

	/* Counters, for DIAG and the console */
	uint32_t n_accept;
	uint32_t n_reject;
	uint32_t n_diverge;
	uint32_t n_miss;
} disc_ctx_t;

/* ----------------------------------------------------------------- outputs */

/** Result of one tick. */
typedef struct {
	disc_state_t state;
	uint16_t dac_code;         /**< value to write to DAC1_OUT1 (PA4) */
	int32_t vc_cmd_mv;         /**< commanded Vc, for telemetry */

	bool sample_accepted;      /**< a PPS sample passed the §3.2 gate */
	bool phase_valid;          /**< phase_err_ns is meaningful this tick */
	float phase_err_ns;        /**< corrected phase error, sign per banner */

	float applied_ppb;         /**< frequency command actually applied */
	float freq_err_ppb;        /**< filtered residual frequency error */

	float adev_1s;             /**< 0 when not enough data yet */
	float adev_10s;
	float adev_100s;

	bool holdover;
	int64_t holdover_est_err_ns;
	uint32_t holdover_t_demote_s; /**< UINT32_MAX when not on the horizon */

	uint8_t stratum;
	uint32_t flags;            /**< QUALITY_FLAG_* */
} disc_out_t;

/* --------------------------------------------------------------------- API */

/**
 * Fill @p cfg with the documented defaults.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p cfg is NULL.
 */
int disc_cfg_defaults(disc_cfg_t *cfg);

/**
 * Initialise the loop. The actuator starts at disc_cfg_t::dac_center_code
 * (1.65 V, never floating — interface ref §3) and the state at ACQUIRING.
 *
 * @param ctx  Context to initialise.
 * @param cfg  Configuration, or NULL for disc_cfg_defaults().
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p ctx is NULL, or @p cfg fails validation (tau outside
 *                  10..1000, window sizes out of range, zero DAC span,
 *                  qerr_sign not +-1, ...).
 */
int disc_init(disc_ctx_t *ctx, const disc_cfg_t *cfg);

/**
 * Run one second with a PPS capture.
 *
 * Pipeline: cross-check and condition the captures (§3.2), gate them through
 * the median/MAD filter, update the FLL or PI, apply the tempco feed-forward,
 * clamp and slew-limit the actuator, run the lock/holdover state machine, and
 * publish the §3.8 block.
 *
 * A tick whose sample is rejected, whose captures are all invalid, or that
 * arrives while GNSS time is not locked, is handled exactly like a missed PPS.
 *
 * @param ctx  Context.
 * @param in   Environment plus this second's captures.
 * @param qs   Quality slot to publish into, or NULL to skip publication.
 * @param out  Receives the tick result, or NULL if not wanted.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p ctx or @p in is NULL, or the context is not initialised.
 */
int disc_tick_pps(disc_ctx_t *ctx, const disc_in_t *in, quality_state_t *qs,
		  disc_out_t *out);

/**
 * Run one second with no PPS capture (the 1.2 s PPS-semaphore timeout in the
 * discipline thread). Freezes the actuator and drives the holdover transitions.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p ctx or @p env is NULL, or the context is not initialised.
 */
int disc_tick_no_pps(disc_ctx_t *ctx, const disc_env_t *env, quality_state_t *qs,
		     disc_out_t *out);

/**
 * PFI park (§10.3): stop steering and hold the actuator where it is.
 *
 * The returned code is the value the PFI handler persists to NVS as "last good
 * Vc" and the value the DAC must keep driving — PA4 must never be tri-stated
 * (interface ref §10 caution 12). Subsequent ticks are no-ops that keep
 * returning this code until disc_unpark().
 *
 * @param ctx       Context.
 * @param out_code  Receives the held DAC code; may be NULL.
 *
 * @retval 0        Parked (idempotent).
 * @retval -EINVAL  @p ctx is NULL or not initialised.
 */
int disc_park(disc_ctx_t *ctx, uint16_t *out_code);

/**
 * Leave the PFI park after the rail recovers without a reset.
 *
 * Resumes in RECOVERING, not ACQUIRING: the frequency estimate is still valid,
 * so the correct behaviour is §3.6's rate-limited pull-in with no step. The
 * retained error estimate survives the park, so the served dispersion after an
 * unpark still bounds an error the park did nothing to correct.
 *
 * @retval 0        Resumed (no-op if not parked).
 * @retval -EINVAL  @p ctx is NULL or not initialised.
 */
int disc_unpark(disc_ctx_t *ctx);

/**
 * The actuator's code<->millivolt transfer, both directions, in ONE place.
 *
 * `ref.ocxo.vc_mv` and `ref.ocxo.dac_code` are two views of DAC1_OUT1, so the
 * console converts between them on every write and every read-back; the loop's
 * own disc_out_t::vc_cmd_mv telemetry is the same conversion. Two copies of it
 * would eventually disagree by a rounding rule and a technician would see a
 * value read back as a different one from the one written.
 *
 * Free functions on (vref, max_code) rather than methods on disc_ctx_t, because
 * the discipline thread must answer both while the context is being stepped —
 * these take no state and cannot race with a tick.
 *
 * @param vref_mv   disc_cfg_t::dac_vref_mv (3300 on this board).
 * @param max_code  disc_cfg_t::dac_max_code (4095, 12-bit).
 */
int32_t disc_code_to_mv(uint16_t vref_mv, uint16_t max_code, uint16_t code);

/**
 * The inverse, rounded to NEAREST code. @p mv is clamped to [0, vref_mv].
 *
 * @retval 0        @p out_code written.
 * @retval -EINVAL  @p out_code is NULL, or a zero vref/max_code.
 */
int disc_mv_to_code(uint16_t vref_mv, uint16_t max_code, int32_t mv,
		    uint16_t *out_code);

/** Current state. DISC_STATE__COUNT for a NULL/uninitialised context. */
disc_state_t disc_state(const disc_ctx_t *ctx);

/**
 * Error the loop is still carrying, nanoseconds: the holdover estimate retained
 * across RECOVERING and PARKED. Zero once LOCKED has been re-achieved.
 *
 * Exposed so the glue and the tests can assert the property that matters — the
 * served dispersion bounds this — rather than inferring it from the state name.
 */
float disc_retained_error_ns(const disc_ctx_t *ctx);

/* ------------------------------------------------------- sawtooth pairing */

/**
 * Evidence for deciding whether a UBX-TIM-TP record describes the PPS edge that
 * was just captured.
 *
 * Pairing is the whole difficulty of the sawtooth correction. The receiver emits
 * TIM-TP in the second *before* the pulse it describes, so "the newest record"
 * is one second early; and TIM-TP reports its ToW on the pulse's own timebase,
 * which for this board's UTC-aligned TIMEPULSE differs from NAV-PVT's GPS iTOW by
 * the leap-second offset. Applying a qErr to the wrong second is worse than
 * applying none — it injects the sawtooth instead of removing it — so the match
 * has to be positive, not assumed.
 */
typedef struct {
	bool record_valid;        /**< a TIM-TP has been decoded at all */
	bool qerr_valid;          /**< receiver flagged it good AND the ToW is on
				    *  GPS; false when the leap offset needed for
				    *  the UTC->GPS conversion is still unknown */
	int32_t qerr_ps;
	uint32_t target_tow_ms;   /**< GPS ToW of the pulse the record describes */
	uint32_t pulse_tow_ms;    /**< GPS ToW the caller believes it captured */
	bool pulse_tow_valid;
	uint64_t record_rx_mono_ms; /**< when the record was decoded */
	uint64_t capture_mono_ms;   /**< when the edge was captured */
} disc_qerr_match_t;

/** Milliseconds in a GPS week; the ToW wrap modulus. */
#define DISC_GPS_WEEK_MS 604800000u

/**
 * Does @p m describe the captured pulse?
 *
 * Requires a valid record with a usable qErr, a known pulse ToW, an exact ToW
 * match, and the record to have arrived before the capture and within one second
 * of it — the last check catches a stale record whose ToW happens to alias after
 * a week wrap or a receiver restart.
 */
bool disc_qerr_matches_pulse(const disc_qerr_match_t *m);

/**
 * GPS ToW of the pulse captured at @p cap_mono_ms, derived from the last NAV-PVT.
 *
 * NAV-PVT for epoch N is transmitted just after epoch N, so the elapsed time
 * between its arrival and a later capture, rounded to whole seconds, is the
 * number of epochs to add. Wraps the week.
 *
 * CALLER OBLIGATIONS — both are preconditions, not merely conditions under
 * which the return value happens to be -EAGAIN:
 *
 *  1. **@p pvt_rx_mono_ms MUST NOT be newer than @p cap_mono_ms.** This routine
 *     only ever extrapolates *forward* from a NAV-PVT to a later pulse. A
 *     caller holding "the newest NAV-PVT" and "the newest capture" does NOT
 *     satisfy this for most of a second: the receiver decodes epoch N's NAV-PVT
 *     *after* pulse N has already been captured, so from the moment of that
 *     decode until the next pulse the newest record is newer than the newest
 *     capture and no naming is possible. That is a property of the pairing the
 *     caller must design around — by keeping the previous observation and
 *     offering it here (disc_name_pulse() below does exactly that) — and not
 *     something this function can fix. A caller that ignores it does not get
 *     wrong answers; it gets -EAGAIN for whatever fraction of the second its
 *     own polling phase happens to fall in, which if that phase is stable is
 *     the same fraction forever.
 *
 *  2. **The NAV-PVT must have arrived within 500 ms of the epoch it names.**
 *     The epoch count is rounded to nearest, so a receiver whose NAV-PVT lands
 *     more than half a second after its own epoch aliases onto the neighbouring
 *     second. At 1 Hz and this board's configured baud the message lands within
 *     a couple of hundred milliseconds; a violation is caught downstream by
 *     disc_qerr_matches_pulse()'s exact ToW comparison, which refuses rather
 *     than mis-names.
 *
 * @retval 0        @p out_tow_ms written.
 * @retval -EINVAL  @p out_tow_ms is NULL, or @p pvt_itow_ms is not a valid ToW.
 * @retval -EAGAIN  The capture predates the NAV-PVT (obligation 1 unmet), or is
 *                  too far past it to attribute (more than @p max_age_ms), so
 *                  no honest answer exists.
 */
int disc_pulse_tow_ms(uint32_t pvt_itow_ms, uint64_t pvt_rx_mono_ms,
		      uint64_t cap_mono_ms, uint32_t max_age_ms,
		      uint32_t *out_tow_ms);

/* ------------------------------------------------ naming a captured pulse */

/**
 * One NAV-PVT arrival, offered as evidence of which second a pulse belongs to.
 *
 * @p rx_mono_ms is 64-bit deliberately: it is compared against a capture
 * timestamp on the same monotonic clock, and a 32-bit millisecond counter wraps
 * after 49.71 days — on an appliance built to run for years, a silent and
 * permanent loss of the comparison.
 */
typedef struct {
	uint32_t itow_ms;     /**< NAV-PVT iTOW, GPS ToW milliseconds */
	uint64_t rx_mono_ms;  /**< monotonic ms when the message was decoded */
	bool     valid;       /**< this slot holds a real observation */
} disc_pvt_obs_t;

/** One UBX-TIM-TP record, offered as evidence naming the pulse it describes. */
typedef struct {
	uint32_t target_tow_ms; /**< GPS ToW of the pulse the record describes */
	uint64_t rx_mono_ms;    /**< monotonic ms when the record was decoded */
	uint16_t week;          /**< GPS week of that pulse, after normalisation */
	int32_t  qerr_ps;       /**< sawtooth quantisation error to subtract */
	bool     qerr_valid;    /**< the qErr is usable AND the ToW is on GPS */
	bool     valid;         /**< this slot holds a real record */
} disc_qerr_obs_t;

/** What a captured pulse turned out to be. */
typedef struct {
	uint32_t tow_ms;   /**< GPS ToW of the captured pulse */
	uint16_t week;     /**< GPS week of the captured pulse */
	int32_t  qerr_ps;  /**< the sawtooth belonging to THAT pulse */
	bool     valid;    /**< a record was positively matched */
} disc_pulse_name_t;

/**
 * Name the pulse captured at @p cap_mono_ms: which GPS second it was, and which
 * sawtooth belongs to it.
 *
 * This is the *join* — the step that turns a hardware capture with no date into
 * an absolutely-placed second. It exists as one function, here, because it was
 * previously open-coded in two glue files against two different callers' timing
 * assumptions, and the assumption the second caller violated was stated nowhere.
 *
 * The caller offers a short history rather than a single record, newest first,
 * because BOTH pieces of receiver evidence are, for most of each second, newer
 * than the capture they would have to describe:
 *
 *   NAV-PVT for epoch N is decoded *after* pulse N (obligation 1 above).
 *   UBX-TIM-TP names the NEXT pulse, so the record decoded during second N
 *   describes pulse N+1, not the pulse that has already been captured.
 *
 * Offer depth 1 and the join therefore succeeds only while the caller's own
 * polling phase happens to fall between the pulse and that second's messages.
 * Offer depth 2 — the record in force at the pulse, plus the one that replaced
 * it — and there is always exactly one candidate on the correct side of the
 * capture, whatever the phase. Depth 2 is the minimum; more is harmless.
 *
 * Selection is newest-first and every candidate must still prove itself:
 * disc_pulse_tow_ms() derives an independent belief of the captured ToW from a
 * NAV-PVT, and disc_qerr_matches_pulse() requires a TIM-TP record whose own ToW
 * equals it exactly and which arrived before the capture and within a second of
 * it. A candidate that cannot be proved is skipped, never guessed at: naming the
 * wrong second is a one-second error in every timestamp the appliance emits.
 *
 * @param pvt          NAV-PVT observations, newest first. May be NULL if n is 0.
 * @param n_pvt        Number of slots in @p pvt (invalid slots are skipped).
 * @param qerr         TIM-TP records, newest first. May be NULL if n is 0.
 * @param n_qerr       Number of slots in @p qerr.
 * @param cap_mono_ms  Monotonic ms the edge was captured at.
 * @param max_pvt_age_ms  Oldest NAV-PVT still allowed to name a pulse.
 * @param out          Always fully written; zeroed on failure. Never NULL.
 *
 * @return true when @p out is a positively-matched name, false when no
 *         combination of the offered evidence could prove one.
 */
bool disc_name_pulse(const disc_pvt_obs_t *pvt, size_t n_pvt,
		     const disc_qerr_obs_t *qerr, size_t n_qerr,
		     uint64_t cap_mono_ms, uint32_t max_pvt_age_ms,
		     disc_pulse_name_t *out);

/**
 * Whole reference seconds between two iterations of the discipline loop.
 *
 * The phase-prediction accumulator in the discipline thread must advance by the
 * number of *reference seconds* that actually elapsed, not by one per loop
 * iteration: a second with no PPS costs the loop its whole 1.2 s timeout, so
 * advancing by exactly one second each time drifts 200 ms per missed second and
 * after three misses forces a re-anchor that throws away the first returning
 * sample.
 *
 * Rounding to the nearest whole second — rather than using the raw elapsed
 * milliseconds — is deliberate: the prediction has to stay on the reference's
 * one-second grid, and feeding the loop's own scheduling jitter into `expected`
 * would inject that jitter straight into the measured phase error.
 *
 * Never returns 0 for a forward-going clock: a stalled prediction is not a valid
 * answer, and the caller has no other way to keep the accumulator moving.
 */
uint32_t disc_expected_advance_s(uint64_t prev_ms, uint64_t now_ms);

/* ------------------------------------------------------- phase-record export */

/**
 * A coherent copy of the ADEV phase ring, for offline stability analysis.
 *
 * Why this exists, and why it carries @ref gaps
 * --------------------------------------------
 * The runtime estimator deliberately absorbs short sample gaps: the tick
 * resets the ring only when the interval exceeds ADEV_MAX_GAP_S (3 s), so one
 * or two missed PPS seconds are tolerated rather than throwing away a window
 * that takes 201 samples to refill. That trade is right for telemetry — see
 * the ADEV_MAX_GAP_S banner in disc.c — and it is wrong for validation.
 *
 * An offline estimator (core/stats) takes a bare phase array and a single
 * tau0. It has no way to see that sample 313 arrived two seconds after sample
 * 312, so an absorbed gap silently mis-times every later second difference and
 * the resulting plot is wrong in a way that looks perfectly plausible.
 *
 * So the record states it. @ref gaps is the number of samples in the exported
 * window that were appended after a non-uniform interval; @ref gapmap says
 * which ones. A record with gaps != 0 is NOT fit for ADEV as a whole, and a
 * consumer that ignores the field is producing a wrong answer confidently.
 * The bitmap additionally lets a consumer recover the longest uniformly
 * spaced run instead of discarding the record.
 *
 * The runtime behaviour is unchanged: nothing here alters what the loop
 * absorbs, only what an exporter can see about it.
 */
typedef struct {
	/** Samples in @ref x_ns, oldest first. 0 when the ring is empty. */
	uint16_t n;
	/** Non-uniform-interval samples within [0, n). 0 == fit for ADEV. */
	uint16_t gaps;
	/** Nominal sample interval; always DISC_ADEV_TAU0_NS. */
	uint32_t tau0_ns;
	/** True once the ring has been full and is evicting oldest samples. */
	bool full;
	/** Phase samples, nanoseconds, oldest first. */
	float x_ns[DISC_ADEV_CAP];
	/** Bit i set == x_ns[i] followed a non-uniform interval. LSB-first. */
	uint8_t gapmap[DISC_ADEV_GAPMAP_BYTES];
} disc_phase_snap_t;

/**
 * Copy the ADEV phase ring and its gap accounting into @p out.
 *
 * Pure read; the loop state is untouched. The caller is responsible for
 * excluding a concurrent disc_tick_pps() — on the device the discipline
 * thread owns disc_ctx_t, so the export runs under that thread's snapshot
 * mutex.
 *
 * @retval 0        Success (including n == 0, an empty but valid record).
 * @retval -EINVAL  NULL argument.
 */
int disc_phase_snapshot(const disc_ctx_t *ctx, disc_phase_snap_t *out);

/* ------------------------------------------------------- offline utilities */

/**
 * Overlapping Allan deviation from phase samples (spec §3.3 telemetry).
 *
 * Standard estimator over phase data (NIST SP 1065 §5.2.4):
 *
 *   sigma_y^2(m*tau0) = SUM (x[i+2m] - 2x[i+m] + x[i])^2
 *                       / (2 * m^2 * tau0^2 * (n - 2m))
 *
 * The second difference removes any constant frequency offset, so a disciplined
 * clock's residual is measured rather than its (steered) rate.
 *
 * Exposed publicly because it is a pure function: the loop calls it on its own
 * ring, and DIAG/console and the unit tests call it on supplied data.
 *
 * @param phase_ns  Uniformly spaced phase samples, nanoseconds, oldest first.
 * @param n         Number of samples.
 * @param m         Averaging factor; tau = m * tau0.
 * @param tau0_s    Sample interval, seconds (1.0 for a 1 Hz PPS).
 * @param out       Receives the dimensionless deviation.
 *
 * @retval 0        Success.
 * @retval -EINVAL  NULL argument, m == 0, or tau0_s not positive.
 * @retval -ENODATA Fewer than 2m+1 samples — the estimate is undefined.
 */
int disc_adev_overlapping(const float *phase_ns, size_t n, uint32_t m,
			  float tau0_s, float *out);

/**
 * Offline tempco fit (spec §10.4 "OCXO tuning characterization ... learn tempco
 * vs TMP117 #1"). Ordinary least squares of @p osc_ppb against @p temp_c.
 *
 * Learning is deliberately offline: the runtime only ever *applies* the stored
 * coefficient, so a bad data set can never destabilise the live loop.
 *
 * @p osc_ppb is the oscillator's own fractional frequency offset, not the
 * command. From a disciplined log it is the negated applied correction:
 * `osc_ppb[i] = -applied_ppb[i]` over an interval where the loop was LOCKED.
 * The returned slope is therefore df/dT, and disc_cfg_t::tempco_ppb_per_c takes
 * it unchanged — the runtime applies the negative as feed-forward.
 *
 * @param temp_c          Oscillator temperature per sample, °C.
 * @param osc_ppb         Oscillator frequency offset per sample, ppb.
 * @param n               Sample count; at least 2.
 * @param out_ppb_per_c   Receives the slope (df/dT). Required.
 * @param out_offset_ppb  Receives the intercept, or NULL.
 * @param out_r2          Receives the coefficient of determination, or NULL.
 *
 * @retval 0        Success.
 * @retval -EINVAL  NULL required argument or n < 2.
 * @retval -EDOM    The temperature does not vary — the slope is unidentifiable.
 */
int disc_tempco_fit(const float *temp_c, const float *osc_ppb, size_t n,
		    float *out_ppb_per_c, float *out_offset_ppb, float *out_r2);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_DISC_DISC_H_ */
