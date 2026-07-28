/*
 * STS1000 Meridian — cross-area application API (Zephyr glue layer).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * The glue layer is split into four ownership areas (platform, net, console,
 * ui). This header is the ONLY interface between them; areas must not include
 * each other's private headers. Everything declared here is implemented by the
 * platform area unless the comment says otherwise.
 *
 * Threading: unless stated otherwise, functions here are callable from any
 * cooperative/preemptive thread, not from ISRs. The registration functions
 * (sts_time_register_source, sts_cfg_register_applier,
 * sts_status_register_encoder, sts_liveness_register) are init-time only —
 * call them from the area's start function, before it spawns its threads.
 */
#ifndef STS1000_ZEPHYR_STS_APP_H_
#define STS1000_ZEPHYR_STS_APP_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "quality/quality.h"
#include "cfg/cfg.h"
#include "logring/logring.h"
/* mp_mirror_in_t crosses the ui -> console seam below (sts_mp_mirror_publish).
 * It is a plain data struct; pulling it in here does not couple any area to
 * core/mp's engine. */
#include "mp/mp_mirror.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- area entry points (called from main.c in bring-up order) ---------- */
int sts_platform_init(void);   /* platform area: stages 2-3, buses, io_scan */
int sts_net_start(void);       /* net area: eth up, services (guarded by Kconfig) */
int sts_console_start(void);   /* console area: usb, shell, mcp, storage */
int sts_ui_start(void);        /* ui area: display, touch, encoder, panel */

/* ---- timing / quality --------------------------------------------------- */
/* Read-consistent snapshot of the §3.8 quality block (seqlock under the
 * hood; never blocks the discipline thread). Returns 0 on success. */
int sts_quality_snapshot(quality_block_t *out);

/* Canonical system time: TAI nanoseconds since PTP epoch, from the ETH PTP
 * clock once the net area has registered it, else from a monotonic-derived
 * fallback that is flagged unsynchronized. Returns 0 on success. */
int sts_time_tai_ns(uint64_t *out_ns);

/* Net area registers its PTP-clock-backed time source here (once). */
typedef int (*sts_tai_source_fn)(void *ctx, uint64_t *out_ns);
void sts_time_register_source(sts_tai_source_fn fn, void *ctx);

/* True while sts_time_tai_ns() is answering from the monotonic fallback,
 * i.e. the returned time is not traceable and must not be served. */
bool sts_time_is_fallback(void);

/* True when the served timescale has a valid ABSOLUTE EPOCH — the PTP hardware
 * clock has been set from GNSS and its servo reports synchronised.
 *
 * Distinct from PPS lock, and from sts_time_is_fallback(). A disciplined
 * oscillator tracking a 1 Hz edge proves *rate*; it says nothing about which
 * second it is, and a registered time source is not the same as a correct one.
 * core/disc serves stratum UNSYNC while this is false, because a confidently
 * wrong timestamp at stratum 1 is worse than serving none.
 *
 * Defaults FALSE. The net area calls sts_time_set_traceable() when its servo
 * reaches (and when it leaves) the synchronised state. */
bool sts_time_is_traceable(void);
void sts_time_set_traceable(bool traceable);

/* Monotonic milliseconds since boot; the timebase every core module's
 * mono_ms argument expects. */
uint64_t sts_mono_ms(void);

/* TAI - GPS, fixed at 19 s since the GPS epoch (1980-01-06). The receiver
 * reports GPS-UTC; every timescale field in this firmware is TAI-UTC, so the
 * platform adds this when publishing. Not a leap second and never changes. */
#define STS_TAI_MINUS_GPS_S 19

/* ---- PPS capture seam (platform publishes, net correlates) -------------- */
/*
 * The path that makes the served time absolutely accurate rather than merely
 * stable (spec §2.3/§15).
 *
 * The ETH MAC's 1588 counter is what actually stamps NTP and PTP packets, and
 * it is syntonized to the disciplined reference in hardware — but its absolute
 * offset has to be set by something. Placing it from the receiver's civil time
 * over USART3 is good to tens of milliseconds and no better: NAV-PVT describes a
 * second that has already passed and the decode-to-publish latency is neither
 * constant nor measured.
 *
 * TIM2 already latches the PPS edge in hardware, to one count. Pairing that
 * capture with a PTP-counter read taken back-to-back with interrupts locked
 * places the counter to a few hundred nanoseconds instead. This seam is what
 * lets the net area do that: the platform area owns TIM2 and the receiver, so it
 * publishes the capture together with the GPS week/ToW naming the pulse and the
 * UBX-TIM-TP sawtooth belonging to it; the net area supplies the PTP counter and
 * does the arithmetic (src/zephyr/net/sts_ppscorr.h).
 *
 * Nothing is computed in the capture ISR. The ISR latches, and both entry points
 * below are read paths.
 */
typedef struct {
	uint32_t seq;         /* capture sequence; 0 = nothing captured yet */
	uint32_t tim2_cnt;    /* TIM2_CCR1 latch of the PA0 rising edge */
	uint32_t timer_hz;    /* TIM2 count rate, from the live RCC tree */
	uint64_t mono_ms;     /* sts_mono_ms() latched in the capture ISR */
	uint16_t gps_week;    /* GPS week of the pulse; valid iff epoch_valid */
	uint32_t gps_tow_ms;  /* GPS time-of-week of the pulse, ms */
	int32_t  qerr_ps;     /* sawtooth quantisation error of THIS pulse */
	bool     epoch_valid; /* a UBX-TIM-TP was positively paired with this edge */
	bool     overcapture; /* an earlier edge was lost before this one */
} sts_pps_epoch_t;

/* Copy the latest PPS capture, with the receiver's naming of that pulse.
 *
 * Non-consuming: it does not touch the discipline thread's capture semaphore,
 * so calling it never costs core/disc a second. @p out is always fully written.
 *
 * epoch_valid is false — leaving gps_week/gps_tow_ms/qerr_ps zero — whenever the
 * UBX-TIM-TP record cannot be proved to describe this edge. It is not a
 * best-effort field: applying a sawtooth to the wrong second injects the
 * sawtooth instead of removing it, and naming the wrong second is a one-second
 * error in the served time.
 *
 * The naming is resolved in the CALLER's context from the published
 * sts_gnss_pulse_evidence_t below, by disc_name_pulse(). It therefore does not
 * depend on when the caller happens to ask: see that structure's comment for
 * why a naive "newest record" pairing would instead work or not work according
 * to a phase fixed at boot.
 *
 * Takes the GNSS snapshot mutex for a struct copy. Callable from any
 * cooperative thread; NOT from an ISR.
 *
 * @retval 0        @p out written (check ::seq and ::epoch_valid).
 * @retval -EINVAL  @p out is NULL.
 * @retval -ENODEV  PPS capture has not been initialised.
 */
int sts_pps_epoch_get(sts_pps_epoch_t *out);

/* The free-running TIM2 counter, right now.
 *
 * One register read and nothing else — it exists to be called immediately
 * either side of a PTP-counter read inside a single irq_lock(), so that the two
 * timebases are sampled as close together as the CPU allows. Anything more in
 * here would widen the very interval it is measuring.
 *
 * ISR-safe. Meaningless before sts_pps_epoch_get() reports a capture. */
uint32_t sts_pps_counter_now(void);

/* ---- receiver-side evidence for naming a captured pulse ----------------- */
/*
 * What sts_pps_epoch_get() needs from the receiver in order to fill in
 * sts_pps_epoch_t's week/ToW/qErr, published by the GNSS thread.
 *
 * Separate from the discipline snapshot (platform.h sts_gnss_snap_t) because
 * the two views answer to different callers with different timing, and the
 * difference is the whole point of this structure:
 *
 *   The discipline thread runs ON the PPS edge. At that instant the receiver's
 *   newest NAV-PVT still describes the PREVIOUS epoch and its newest UBX-TIM-TP
 *   still describes the pulse that has just fired, so one record of each is all
 *   it can use and all it needs.
 *
 *   The PTP servo runs on a free-running 1 Hz k_timer with no phase relationship
 *   to the PPS at all. Once that second's messages have been decoded — which
 *   happens a couple of hundred milliseconds after the pulse and holds for the
 *   rest of the second — the newest NAV-PVT is newer than the latched capture
 *   and the newest TIM-TP describes the NEXT pulse. Both are then unusable, and
 *   a servo whose phase sits there gets no correlated reference at all. Because
 *   both clocks come off the same PLL and that PLL is disciplined to this very
 *   PPS, the servo's phase relative to the pulse is frozen at boot to about a
 *   part in 1e9: it does not drift out of a bad phase in any useful lifetime.
 *
 * So this carries a short history — the record in force at the pulse, and the
 * one that replaced it. With both available there is always exactly one
 * candidate on the correct side of the capture, whatever the polling phase, and
 * disc_name_pulse() still has to prove which by ToW before either is believed.
 */

/** History depth. Two: the record in force at the pulse, plus its successor. */
#define STS_GNSS_PULSE_OBS 2

/**
 * Period of the receiver messages the history above is built from.
 *
 * NAV-PVT and UBX-TIM-TP are both configured at 1 Hz (spec §3.7), so a new
 * record of each kind lands once a second.
 */
#define STS_GNSS_EVIDENCE_SRC_PERIOD_MS 1000U

/**
 * Longest publish period STS_GNSS_PULSE_OBS still covers.
 *
 * THE DEPTH AND THE CADENCE ARE ONE ASSUMPTION, NOT TWO. The snapshot is a
 * SAMPLE of gnssmgr's retained records, not a queue: each publish overwrites
 * both slots with whatever the two newest records are at that instant. Depth 2
 * is sufficient only because a publish happens at least as often as a new
 * record arrives, so no record can be born and evicted between two publishes.
 *
 * Let the publish period be P and the message period S. Between publishes
 * floor(P/S) new records land; the snapshot keeps N of them, so the record that
 * was in force at a pulse survives to be published only while P <= (N-1)*S.
 * With N = 2 that is one second. Raise the publish period past it and pulses
 * silently stop being named — the exact symptom sts_gnss_pulse_evidence_t was
 * introduced to cure, returning without a single failing assertion.
 *
 * platform/gnss.c holds a BUILD_ASSERT against this, next to the tick it
 * publishes on, so changing the cadence is a compile error rather than a
 * regression found on a scope.
 */
#define STS_GNSS_EVIDENCE_MAX_PERIOD_MS \
	(((unsigned int)STS_GNSS_PULSE_OBS - 1U) * STS_GNSS_EVIDENCE_SRC_PERIOD_MS)

/** One NAV-PVT arrival: which epoch it named, and when it was decoded. */
typedef struct {
	uint32_t itow_ms;    /* NAV-PVT iTOW, GPS ToW milliseconds */
	uint64_t rx_mono_ms; /* sts_mono_ms() when the message was DECODED */
	bool     valid;
} sts_gnss_pvt_obs_t;

/** One UBX-TIM-TP record, already normalised onto the GPS timescale. */
typedef struct {
	uint32_t target_tow_ms; /* GPS ToW of the pulse the record describes */
	uint64_t rx_mono_ms;    /* sts_mono_ms() when the record was DECODED */
	uint16_t week;          /* GPS week of that pulse, after normalisation */
	int32_t  qerr_ps;       /* sawtooth quantisation error of that pulse */
	bool     qerr_valid;    /* receiver flagged it good AND the ToW is on GPS */
	bool     valid;
} sts_gnss_qerr_obs_t;

/** Both histories, newest first. Index 0 is always the most recent. */
typedef struct {
	sts_gnss_pvt_obs_t  pvt[STS_GNSS_PULSE_OBS];
	sts_gnss_qerr_obs_t qerr[STS_GNSS_PULSE_OBS];
} sts_gnss_pulse_evidence_t;

/* Copy the published pulse-naming evidence.
 *
 * Takes the GNSS snapshot mutex for a struct copy; callable from any
 * cooperative thread, NOT from an ISR. @p out is always fully written.
 *
 * @retval 0        @p out written (individual slots may still be invalid).
 * @retval -EINVAL  @p out is NULL.
 * @retval -ENODEV  The GNSS thread never started.
 */
int sts_gnss_pulse_evidence(sts_gnss_pulse_evidence_t *out);

/* ---- reference-selection override (operator intent) --------------------- */
/*
 * The operator's standing request to core/refsel, read by the discipline
 * thread once a second as refsel_in_t::request.
 *
 * A single atomic word, not a mutex. The discipline thread is priority 4 and
 * owns the DAC and the reference state machine; nothing it touches may be
 * held by a priority-12 web worker, so the request crosses as one aligned
 * store with no lock on either side (ARCHITECTURE.md §10 invariant 10).
 *
 * refsel decides; this only expresses intent. AUTO is the normal setting and
 * the boot default: refsel engages the rubidium whenever its guards allow.
 * OCXO pins the OCXO and reverts immediately if input B is live. EXTREF is the
 * explicit selection of input B as a house standard — refsel.h states it is
 * "never entered automatically", and the automatic path cannot reach it here
 * either, because refsel_step() maps AUTO to REFSEL_RB_ACTIVE and only an
 * explicit REFSEL_REQ_EXTREF names REFSEL_EXTREF_ACTIVE.
 *
 * The guards are NOT bypassed by any of these: selecting a reference whose
 * 10 MHz is out of band, or whose lock line is not asserted, leaves the machine
 * on the OCXO until the guards hold for the hysteresis window. There is no
 * force.
 *
 * RUNTIME-ONLY. The request resets to AUTO on every boot. There is no cfg key
 * for it — group 0x06 (timing) has no reference-request row — and a pinned
 * reference surviving a reboot is the failure mode where a box repaired months
 * ago is still running on its OCXO because nobody remembered.
 */
typedef enum {
	STS_REF_REQ_AUTO = 0,  /**< engage the Rb whenever the guards allow */
	STS_REF_REQ_OCXO,      /**< pin the OCXO; revert now if not already */
	STS_REF_REQ_EXTREF,    /**< explicit selection of input B */
	STS_REF_REQ__COUNT,
} sts_ref_req_t;

/**
 * Set the standing reference request.
 *
 * Idempotent: setting the mode already in force changes nothing and does not
 * disturb the state machine. That matters — refsel.h notes that changing the
 * request restarts the guard-debounce window, so a management plane polling or
 * re-POSTing a control must not be able to hold the machine off its reference
 * indefinitely.
 *
 * Non-blocking, callable from any thread (not an ISR).
 *
 * @retval 0        Stored (or already in force).
 * @retval -EINVAL  @p mode is not an sts_ref_req_t.
 */
int sts_ref_override_set(uint8_t mode);

/**
 * The standing request currently in force (sts_ref_req_t).
 *
 * This is INTENT, not state: the reference actually feeding PH0 is
 * quality_block_t::active_ref from sts_quality_snapshot(). A management plane
 * showing an override control needs both — "requested rb, running ocxo" is the
 * normal reading while the guards debounce.
 */
uint8_t sts_ref_override_get(void);

/* ---- bench calibration procedures ---------------------------------------- */
/*
 * One entry point, one genuinely automatable procedure. See core/cal/cal.h for
 * why the other three are refused rather than faked.
 */
typedef enum {
	STS_CAL_HOLDOVER = 0, /**< §10.4 holdover characterisation run */
	STS_CAL_OCXO_TUNE,    /**< OCXO pull-range / tempco sweep */
	STS_CAL_INA_TRIM,     /**< per-board SHUNT_CAL trim against a load */
	STS_CAL_COMPASS,      /**< e-compass hard/soft-iron */
	STS_CAL__COUNT,
} sts_cal_proc_t;

/**
 * Pack an STS_CAL_INA_TRIM argument: rail index in the top byte, the reference
 * instrument's current in the low 24 bits, in MICROAMPS.
 *
 * Microamps because the trim's whole purpose is to remove a 1 % error: at
 * milliamp granularity the argument's own quantisation would be 0.7 % of the
 * antenna rail's design current and the calibration would be measuring the
 * operator's rounding. 24 bits reaches 16.777 A, comfortably past the largest
 * rail full scale on the board (VCC_RB, 5.8514 A).
 */
#define STS_CAL_INA_ARG(rail, i_ref_ua)                                        \
	((((uint32_t)(rail) & 0xFFU) << 24) | ((uint32_t)(i_ref_ua) & 0xFFFFFFU))

/**
 * Run a bench calibration procedure.
 *
 * Runs to completion on the CALLING thread and takes no timing-state lock. The
 * only bounded wait is the housekeeping sensor-cache mutex, held for a struct
 * copy by a priority-14 thread; nothing here touches the gnss or discipline
 * contexts.
 *
 * STS_CAL_INA_TRIM (@p arg built with STS_CAL_INA_ARG):
 *   compares the named rail's INA228 reading against the operator's reference
 *   current, computes a bounded SHUNT_CAL trim (core/cal), writes it to
 *   cal.ina.<rail>, commits, and re-applies the calibration to the part.
 *
 * @retval 0        The procedure ran and its result was persisted.
 * @retval -ENOTSUP @p proc is a bench operation this device cannot perform on
 *                  itself. Its result is entered through the group 0x0C cfg
 *                  keys; there is no procedure to start.
 * @retval -EINVAL  @p proc is out of range, or the packed argument is malformed
 *                  (unknown rail, zero reference).
 * @retval -ENODATA The monitor has no fresh, calibrated reading to trim against.
 * @retval -ERANGE  The reference and the measurement are not consistent with a
 *                  1 % shunt: refused rather than persisted.
 * @retval -EBUSY   Uncommitted config changes are staged; this procedure
 *                  commits, and committing would sweep them in.
 * @retval -EIO     The trim was computed but could not be staged or persisted.
 */
int sts_cal_run(uint8_t proc, uint32_t arg);

/* ---- GNSS wall-clock (absolute epoch source for the PTP clock) ----------- */
/* The receiver's civil time, used ONCE per boot (and after any step) to place
 * the ETH PTP counter's absolute epoch via ptp_clock_set(). The discipline loop
 * steers rate from PPS; nothing in that path knows which second it is, so this
 * is the only thing that makes a served timestamp traceable.
 *
 * Implemented by the platform area (GNSS thread). The net area's sts_ptpclk
 * carries a __weak fallback returning -ENODATA so a build without the GNSS
 * thread links and stays honestly unsynchronised rather than serving a
 * power-on-relative timestamp as stratum 1.
 *
 * Consumers MUST reject the reading unless utc_valid && leap_valid &&
 * time_locked, and MUST age it by (now - mono_ms) before comparing: this is
 * when the receiver's time was decoded, not when the getter was called. */
typedef struct {
	int64_t  utc_unix_s;    /* UTC seconds since 1970-01-01, receiver-reported */
	int32_t  utc_nano_ns;   /* signed sub-second correction, (-1e9, +1e9) */
	int16_t  tai_minus_utc; /* TAI - UTC, seconds; valid iff leap_valid */
	uint32_t tacc_ns;       /* receiver time-accuracy estimate; 0 = unknown */
	uint64_t mono_ms;       /* sts_mono_ms() when this time was DECODED */
	bool     utc_valid;     /* date and time fully resolved */
	bool     leap_valid;    /* tai_minus_utc is a real receiver value */
	bool     time_locked;   /* the fix is usable for timing */
} sts_gnss_wallclock_t;

/* Fill @p out from the receiver. 0 on success, -ENODATA when no GNSS time
 * source exists in this build/boot, -EINVAL when @p out is NULL. */
int sts_gnss_wallclock(sts_gnss_wallclock_t *out);

/* ---- GNSS USART3 seam (receiver flash sessions and MP passthrough) ------- */
/* Hand USART3 over to something that is not the GNSS manager.
 *
 * The platform area (src/zephyr/platform/gnss.c) owns USART3: the RX ISR, the
 * UBX parser and the gnssmgr instance. Two consumers need the port instead —
 * core/fwupd's receiver flash loader and the Maintenance Protocol's GNSS
 * passthrough tunnel (FMT §5.5) — and both need firmware to stop driving it
 * first, or two writers interleave frames onto one wire and gnssmgr keeps
 * trusting bytes that are no longer its receiver's.
 *
 * While SUSPENDED: the RX ISR still fills the ring, so raw_rx() can drain it,
 * but nothing feeds the UBX parser or gnssmgr, and gnssmgr sits in
 * GNSSMGR_ST_FW_UPDATE so a configuration retry can never be aimed at a flash
 * loader. raw_tx() and set_baud() are refused (-EPERM) outside a suspended
 * session. resume() re-runs gnssmgr's configuration walk.
 *
 * Both suspend() and resume() are idempotent: calling either when already in
 * that state returns 0 without touching anything, so callers can be balanced by
 * construction. -ENODEV when the GNSS thread never started.
 *
 * The console area (fwupd_glue.c) carries __weak no-ops returning -ENOTSUP, so a
 * build without the platform GNSS thread links and honestly reports that the
 * transport does not exist. */
int sts_gnss_uart_suspend(void);
int sts_gnss_uart_resume(void);
int sts_gnss_uart_raw_tx(const uint8_t *buf, size_t len);
int sts_gnss_uart_raw_rx(uint8_t *buf, size_t cap);
int sts_gnss_uart_set_baud(uint32_t baud);

/* ---- GNSS operator requests (management planes -> the gnss thread) ------- */
/*
 * Two operator actions change what the receiver is doing: re-survey, and adopt
 * a fixed antenna position. Both are REQUESTS, not calls.
 *
 * gnssmgr.h is explicit that a gnssmgr_t is owned by one thread and is not
 * internally synchronised. Calling gnssmgr_request_survey() from a web worker
 * would mutate the manager's state machine and emit a UBX frame underneath the
 * gnss thread's own parse and ACK bookkeeping. So these post into a
 * single-slot mailbox that the gnss thread drains on its next wake.
 *
 * Non-blocking by construction. The post is a k_spinlock critical section of a
 * handful of stores — no k_mutex, so a priority-12 web worker can never park
 * the priority-6 gnss thread, and no management thread ever waits on a timing
 * thread (firmware/CLAUDE.md hard rule, ARCHITECTURE.md §10 invariant 10).
 * Callable from any cooperative thread, not from an ISR.
 *
 * The mailbox holds ONE request. A second arriving before the first is drained
 * is refused with -EBUSY rather than queued: these are mutually exclusive
 * reconfigurations of the same TMODE setting, and silently applying both in
 * sequence is never what the operator meant.
 */

/**
 * Start survey-in.
 *
 * The stored position is discarded when the request is applied and the receiver
 * surveys for `gnss.survey.dur` seconds (default 1 h). Returning 0 means the
 * request was ACCEPTED, never that the survey finished; progress arrives on
 * UBX-NAV-SVIN and is read back with gnssmgr_svin().
 *
 * @param start  Must be true; see -ENOTSUP.
 *
 * @retval 0        Queued; the gnss thread applies it within one of its ticks.
 * @retval -ENOTSUP @p start is false. core/gnssmgr exposes no survey abort —
 *                  gnssmgr_request_survey() has no inverse — and there is no
 *                  honest local substitute: the position surveyed so far has by
 *                  definition not met its accuracy limit, and the position that
 *                  preceded the survey was discarded when it started. Refused
 *                  and reported rather than silently ignored.
 * @retval -ENODEV  The gnss thread never started.
 * @retval -EBUSY   A previous request has not been drained yet.
 */
int sts_gnss_request_survey(bool start);

/**
 * Adopt an operator-supplied fixed ECEF antenna position.
 *
 * Units are centimetres, matching the REST contract (rest.h gnss_fixed) and
 * gnssmgr_ecef_t::x_cm — NOT the 0.1 mm of gnssmgr_ecef_t::acc_0p1mm, which is
 * derived from `gnss.survey.acc` because the caller supplies no accuracy.
 *
 * Applying it re-runs the configuration walk so the TMODE step goes out in its
 * fixed-position form; the receiver leaves survey-in and starts fixed-position
 * timing mode. That costs the receiver's measured state for one navigation
 * epoch (gnssmgr_notify_reset() invalidates it), which is why this is an
 * operator action and not something anything does on its own.
 *
 * NOT PERSISTED — see the note in src/zephyr/platform/gnss.c. The cfg schema
 * has no key for a site position (group 0x05 stops at 0x0507), so a cold boot
 * surveys again. Position error is a fixed delay error, so the cost is survey
 * time, not accuracy.
 *
 * @retval 0        Queued.
 * @retval -EINVAL  A coordinate does not fit int32 centimetres, or the triple
 *                  is not a point on or near the Earth's surface.
 * @retval -ENODEV  The gnss thread never started.
 * @retval -EBUSY   A previous request has not been drained yet.
 */
int sts_gnss_set_fixed_ecef(int64_t x_cm, int64_t y_cm, int64_t z_cm);

/* ---- GNSS sky view (per-satellite az/el/CN0 for the §6.3 skyplot) -------- */
/*
 * core/gnssmgr deliberately keeps no per-satellite records — it reduces
 * UBX-NAV-SAT to a counted summary (gnssmgr.h), because the discipline loop and
 * the quality block only ever need the counts. The skyplot needs the records
 * themselves, so the GNSS thread caches the last frame here and this is where
 * it crosses the area seam.
 *
 * A snapshot, not a pointer: the gnss thread (priority 6) rewrites the cache
 * once a second and the UI renders at 10 Hz from priority 15, so a reader
 * holding a pointer would tear. The copy is under a bounded mutex, like
 * sts_health_snapshot().
 *
 * `mono_ms` is when the frame was DECODED. A consumer that cares whether the
 * sky is current MUST age it; a receiver that has stopped talking leaves the
 * last good frame in place, which is the right thing to draw for a second or
 * two and the wrong thing to draw for an hour.
 */
#define STS_GNSS_SKY_MAX_SV 32u

typedef struct {
	uint8_t gnss_id;  /* UBX: 0 GPS, 1 SBAS, 2 GAL, 3 BDS, 5 QZSS, 6 GLO */
	uint8_t sv_id;    /* receiver-scoped satellite id */
	uint8_t cno_dbhz;
	int8_t  elev_deg; /* -90..90; 0 when the receiver does not know */
	int16_t azim_deg; /* 0..360; 0 when the receiver does not know */
	bool    used;     /* in the navigation/timing solution */
} sts_gnss_sv_t;

typedef struct {
	uint8_t       count;   /* valid entries in sv[] */
	sts_gnss_sv_t sv[STS_GNSS_SKY_MAX_SV];
	uint32_t      itow_ms; /* iTOW of the NAV-SAT frame */
	uint64_t      mono_ms; /* sts_mono_ms() at decode; 0 = never received */

	/* Last known geodetic position, for the skyplot's declination model.
	 * Carried here rather than in a second getter because the two are read
	 * together, once per rendered frame. */
	bool    pos_valid;
	int32_t lat_1e7;
	int32_t lon_1e7;
} sts_gnss_sky_t;

/* Fill @p out from the GNSS thread's cache.
 *
 * @retval 0        Written. `count` may be 0 and `mono_ms` may be 0.
 * @retval -EINVAL  @p out is NULL.
 * @retval -EBUSY   The cache lock was held past the caller's patience; @p out
 *                  is zeroed. A render tick may simply skip the frame. */
int sts_gnss_sky(sts_gnss_sky_t *out);

/* ---- GNSS receiver detail (survey-in, stored position, RF front end) ----- */
/*
 * Spec §7.1 asks the `gnss` shell group for "survey-in start/stop, ... antenna
 * status"; the survey progress, the stored ECEF and the MON-RF view are what
 * answer it, and none of them belongs in the discipline snapshot above — the
 * timing path never reads them and copying them onto the priority-6 thread's
 * hot path would be pure cost.
 *
 * Flat scalars rather than core/gnssmgr's own structs on purpose: sts_app.h is
 * the seam between four areas and must not drag core/gnssmgr's header into all
 * of them (ARCHITECTURE.md §2). The gnss thread reads gnssmgr_svin(),
 * gnssmgr_position() and gnssmgr_rf() on its own thread — which is where the
 * manager may be read at all — and publishes the result here.
 *
 * A snapshot under the same bounded mutex as sts_gnss_snapshot(), refreshed
 * once a second. Callable from any cooperative thread, NOT from an ISR.
 */
typedef struct {
	/* core/gnssmgr's own lifecycle state (gnssmgr_state_t): IDLE, CONFIG,
	 * SURVEY_IN, FIXED, CONFIG_FAILED, FW_UPDATE. sts_gnss_snap_t carries
	 * only the two booleans the sequencer needs (cfg_ack / cfg_failed),
	 * which cannot distinguish "still walking the config" from "surveying"
	 * from "the receiver belongs to the firmware updater" — and those are
	 * three very different answers to "why is this unit not serving". */
	uint8_t  mgr_state;

	/* Survey-in progress, UBX-NAV-SVIN. */
	bool     svin_seen;      /* a NAV-SVIN has been decoded this session */
	bool     svin_active;    /* a survey is running right now */
	bool     svin_ok;        /* the surveyed position met its limits */
	uint32_t svin_dur_s;
	uint32_t svin_obs;
	uint32_t svin_acc_0p1mm; /* mean accuracy so far */

	/* Stored antenna position: surveyed, seeded from NVS, or operator-set. */
	bool     pos_valid;
	int32_t  pos_x_cm;
	int32_t  pos_y_cm;
	int32_t  pos_z_cm;
	uint32_t pos_acc_0p1mm;

	/* RF front end, UBX-MON-RF. */
	bool     rf_valid;
	bool     rf_ant_short;
	bool     rf_ant_open;
	uint8_t  rf_ant_power;   /* UBX_ANT_POWER_* */
	uint8_t  rf_jamming;     /* worst UBX_JAMMING_* across the blocks */

	/* Antenna supervisor verdict and the manager's own alarm set. */
	uint8_t  ant_state;         /* gnssmgr_ant_state_t */
	bool     ant_short_latched; /* bias cut; only sts_gnss_ant_reenable() undoes it */
	uint32_t alarms;            /* bit n = gnssmgr_alarm_t n active */
} sts_gnss_detail_t;

/* Fill @p out from the gnss thread's published view.
 *
 * Takes the gnss thread's snapshot mutex, which that thread holds only for
 * three struct copies once a second — no I/O, no logging, no allocation — so
 * the wait is bounded by a memcpy rather than by any device. Same shape as
 * sts_health_snapshot(). Not from an ISR.
 *
 * @retval 0        Written; individual `*_valid` flags say what is populated.
 * @retval -EINVAL  @p out is NULL.
 * @retval -ENODEV  The gnss thread never started; @p out is zeroed. */
int sts_gnss_detail(sts_gnss_detail_t *out);

/* Convert a UBX 0.1 mm accuracy to millimetres, rounded to nearest.
 *
 * Lives here, beside the sts_gnss_detail_t members that carry the 0.1 mm unit,
 * because every plane that reports survey or position accuracy reports it in
 * millimetres and a factor of ten on the one number an operator compares
 * against `gnss.survey.acc` looks entirely plausible either way. One
 * implementation, three consumers (web, MP telemetry, panel). */
static inline uint32_t sts_gnss_acc_0p1mm_to_mm(uint32_t v)
{
	/* Not (v + 5) / 10: v may be UINT32_MAX and the add would wrap, turning
	 * the widest possible accuracy into 0 mm — "perfectly surveyed". */
	return (v / 10U) + (((v % 10U) >= 5U) ? 1U : 0U);
}

/* ---- e-compass (IIS2MDC magnetometer + LIS2DH12 accelerometer) ----------- */
/*
 * Sampled by the housekeeping sweep and consumed by the UI area's skyplot to
 * rotate the plot to true north (spec §6.3, §10.4). Raw sensor output with no
 * hard/soft-iron correction applied — the calibration lives in cfg group 0x0C
 * and is applied by the consumer, so a recalibration takes effect without the
 * sampling path knowing anything about it.
 *
 * Axes are the board's: +x right, +y up the front panel, +z out of the board
 * face. The board is mounted vertically, so +z is horizontal in service and
 * tilt compensation is mandatory rather than optional.
 */
typedef struct {
	int32_t  mag_mgauss[3]; /* IIS2MDC, milligauss */
	int32_t  acc_mg[3];     /* LIS2DH12, milli-g */
	bool     valid;         /* both parts answered on the last sweep */
	uint32_t mono_ms;       /* sts_mono_ms() of that sweep */
} sts_ecompass_t;

/* Fill @p out from the housekeeping cache. Returns 0, or -EINVAL for NULL.
 * `valid` is false — not an error — when the parts are absent or the sweep has
 * not run yet, which is what puts the skyplot's "north unverified" badge up. */
int sts_ecompass(sts_ecompass_t *out);

/* ---- config ------------------------------------------------------------- */
/* The single live cfg context (loaded before any area starts). Never NULL
 * once sts_platform_init() has returned.
 *
 * MUTEX-GUARDED. A cfg_ctx_t is not internally locked (cfg.h) and this one is
 * shared by the MCP engine, the Zephyr shell backend and the ui_local thread.
 * Every cfg_set/cfg_revert/cfg_import/cfg_factory_reset call on it, and every
 * read that must not see a half-applied commit, MUST be bracketed by
 * sts_cfg_lock()/sts_cfg_unlock(). sts_cfg_commit() takes the lock itself. */
cfg_ctx_t *sts_cfg(void);

/* Enter/leave mutual exclusion over sts_cfg(). Callable from any thread, not
 * from an ISR. Hold it across a whole multi-step operation (stage several keys,
 * feed an import chunk) so another cfg user cannot commit half of it.
 *
 * Do NOT hold it across sts_cfg_commit(): that call takes the mutex itself and
 * then dispatches config appliers with it RELEASED, and an applier may block on
 * sockets, DNS or display I/O. The lock is recursive (Zephyr k_mutex counts
 * ownership) so nesting would not deadlock — it would quietly run those appliers
 * inside the config critical section, where every other cfg user waits on them.
 * Release, commit, re-acquire if you still need the section. */
void sts_cfg_lock(void);
void sts_cfg_unlock(void);

/* The console area owns persistence (Zephyr settings/NVS). It registers its
 * store here during sts_console_start(); until then the platform area runs
 * cfg on a RAM-backed store holding schema defaults, so every area can read
 * config from its own start function without ordering games.
 *
 * Registration re-runs cfg_load_all() against the real store, so values
 * persisted on a previous boot appear at this point. Appliers registered for
 * every group are then invoked once with the group id, exactly as after a
 * commit. Call at most once. */
int sts_cfg_register_store(const port_store_t *store);

/* True once a persistent store has been registered. While false, config
 * changes are accepted but will not survive a reboot. */
bool sts_cfg_is_persistent(void);

/* Areas register appliers invoked after a successful cfg commit for the
 * groups they own (group = high byte of key id). Called from the committing
 * thread with sts_cfg_mutex released; must be quick or defer to the area's own
 * thread.
 *
 * A group may have SEVERAL subscribers and every one of them is called, in
 * registration order. That is not a convenience: group 0x09 (log) is genuinely
 * shared — the console area pushes log.level into the ring and the net area
 * reloads the syslog sender — and a one-slot-per-group registry silently let
 * whichever area started second delete the other's applier.
 *
 * Registering the same (fn, ctx) pair twice for a group is idempotent and
 * returns 0.
 *
 * @retval 0        Registered (or already present).
 * @retval -EINVAL  fn is NULL, or group is out of range.
 * @retval -ENOSPC  That group already has STS_CFG_GROUP_SUBS_MAX subscribers. */
typedef void (*sts_cfg_apply_fn)(void *ctx, uint8_t group);
int sts_cfg_register_applier(uint8_t group, sts_cfg_apply_fn fn, void *ctx);

/* Commit the staged config set and run the appliers for every group touched.
 * Areas must use this rather than calling cfg_commit() directly, so the
 * appliers actually run. Takes sts_cfg_mutex for the commit and releases it
 * before dispatching appliers.
 *
 * Returns cfg_commit()'s result. -EIO means "applied to the live tree, but N
 * keys did not reach the store" (res->persist_errors) — the appliers still run,
 * because the running system really did change. Only a validation or
 * cross-field rejection (nothing applied) skips them. */
int sts_cfg_commit(cfg_commit_res_t *res);

/* Factory-reset the config tree and run the appliers for every registered
 * group, so no subsystem keeps running pre-reset configuration.
 *
 * Areas must use this rather than calling cfg_factory_reset() directly, for the
 * same reason they must use sts_cfg_commit() rather than cfg_commit(): the bare
 * core call rewrites the tree and erases the store while every subsystem carries
 * on with the configuration it was handed before the reset, until the next
 * reboot. A factory reset touches every key, so EVERY registered group is
 * dispatched, not just the ones a commit would have staged.
 *
 * Takes sts_cfg_mutex for the reset and releases it before dispatching, exactly
 * as sts_cfg_commit() does — so this must NOT be called with the lock held.
 * Groups whose keys are flagged reboot-required stay reboot-required; the point
 * is that the runtime-apply groups take effect immediately.
 *
 * Returns cfg_factory_reset()'s result. -EIO means "the live tree was reset but
 * at least one store erase failed"; the appliers still run, because the running
 * system really did change. */
int sts_cfg_factory_reset(void);

/* ---- logging ------------------------------------------------------------ */
logr_t *sts_logring(void);
/* Convenience structured-log emit (logr_sub_t subsystem, logr_level_t level).
 * Safe from any thread; not from an ISR. */
void sts_log(uint8_t subsys, uint8_t level, const char *fmt, ...)
	__attribute__((format(printf, 3, 4)));

/* ---- status / telemetry for MCP + shell + SNMP -------------------------- */
/* Encode the versioned status struct for an MCP STATUS_GET group
 * (mcp_wire.h mcp_group_t ids). Returns encoded length or negative error.
 * Implemented by the platform area, aggregating all areas' state.
 *
 * The platform area encodes SUMMARY, TIMING, GNSS, POWER and ALARMS itself.
 * NET and PTP have no platform-side source, so they return -ENOTSUP until the
 * net area registers a sub-encoder for them. */
int sts_status_encode(uint8_t group, uint8_t *buf, size_t cap);

/* Sub-encoder signature; same contract as sts_status_encode() for one group.
 * buf[0] must be the group struct's own version byte (mcp_wire.h §STATUS_GET). */
typedef int (*sts_status_encode_fn)(void *ctx, uint8_t group, uint8_t *buf,
				    size_t cap);

/* Register (or replace) the encoder for one group. An area may only register
 * groups it owns. Returns 0, or -EINVAL for an out-of-range group. */
int sts_status_register_encoder(uint8_t group, sts_status_encode_fn fn,
				void *ctx);

/* ---- health snapshot (rails, temps, fan, PoE, backup) ------------------- */
/* Aggregate environmental/power health, published by the platform's
 * housekeeping thread and read by the UI Power/Health page, the MCP
 * STATUS_GET POWER group and the SNMP agent — one source so all three agree.
 * Non-timing data; snapshot is a bounded mutex copy of the housekeeping cache,
 * safe from any management thread (not an ISR). */
#define STS_HEALTH_VER 1u
#define STS_HEALTH_INA_COUNT 9u /* == INA228_RAIL_COUNT, ina228_rail_t order */

typedef struct {
	int32_t  bus_mv;
	int32_t  current_ua;
	uint32_t power_uw;
	uint16_t diag_alrt;
	bool     valid;   /* read succeeded AND SHUNT_CAL confirmed present */
} sts_health_ina_t;

typedef struct {
	uint16_t ver;       /* STS_HEALTH_VER */
	uint16_t ina_count; /* STS_HEALTH_INA_COUNT */

	sts_health_ina_t ina[STS_HEALTH_INA_COUNT];

	int32_t tmp_osc_mc;   bool tmp_osc_valid;   /* TMP117 0x49 (oscillator) */
	int32_t tmp_amb_mc;   bool tmp_amb_valid;   /* TMP117 0x48 (enclosure) */
	int32_t die_mc;       bool die_valid;       /* STM32 internal sensor */
	int32_t humidity_mpct; bool humidity_valid; /* SHT45 0x44, milli-%RH */

	uint32_t fan_rpm;
	uint16_t fan_duty_pct;

	uint8_t  poe_class;      /* negotiated class, 0 = unknown (see note) */
	uint32_t poe_draw_mw;    /* measured, INA228 0x40 */
	uint32_t poe_budget_mw;  /* granted budget (cfg pwr.poe.mw) */

	bool bkp_stm_pg;         /* STM supercap backup rail good */
	bool bkp_gps_pg;         /* GPS V_BCKP backup rail good */

	uint32_t mono_ms;        /* age of the underlying sweep */
} sts_health_t;

/* Fill @p out from the housekeeping cache. Returns 0, or -EINVAL for NULL. */
int sts_health_snapshot(sts_health_t *out);

/* ---- power sequencer (platform area, pwrseq_exec.c) --------------------- */
/*
 * What the stage machine decided, and what it decided it from.
 *
 * core/pwrseq owns every gated load on the board — GPS power, the display, the
 * guarded rubidium sequence, the PoE shed ladder, the 26 V over-voltage latch,
 * the watchdog arm and the holdover relay — and until this snapshot existed
 * nothing outside the platform area could see any of it. A technician on the
 * Field Maintenance Tool read stage 0, shed level 0 and an empty alarm word on
 * a unit that was in fact halted at stage 8 with the rubidium deferred, which
 * is precisely the state you need to see when a unit will not come up.
 *
 * Flat scalars rather than core/pwrseq's own structs, for the reason
 * sts_gnss_detail_t states: sts_app.h is the seam between four areas and must
 * not drag core/pwrseq's header into all of them (ARCHITECTURE.md §2).
 *
 * Two halves, and the distinction matters when reading a unit that is wrong:
 *
 *   DECIDED   core/pwrseq's own flattened status (pwrseq_status()). These are
 *             the sequencer's beliefs and its commanded pin states.
 *   OBSERVED  the inputs it was handed on that same tick (pwrseq_in_t). These
 *             are the hardware's answers. `rb_lock_pin` is RB_LOCK (PB13) with
 *             the per-unit polarity applied — NOT the same signal as
 *             quality_block_t::active_ref, which is what the discipline loop
 *             *selected*; a unit whose FE has dropped lock but has not yet been
 *             switched away differs between the two, and that difference is the
 *             diagnosis.
 *
 * Published by the housekeeping thread's 4 Hz sequencer pass through a
 * single-writer seqlock, so sts_pwrseq_snapshot() never blocks that thread and
 * never blocks on it. Safe from any thread; not from an ISR.
 */
typedef struct {
	/* False = the sequencer never started; every field below is zero and
	 * means nothing. Distinct from stage 0 (PWRSEQ_STAGE_IDLE). */
	bool started;

	/* -------------------------------------------------- decided -------- */
	uint8_t  stage;            /* pwrseq_stage_t */
	uint32_t stage_entered_ms;
	uint8_t  retries;          /* attempts at the current step */
	uint8_t  shed;             /* pwrseq_shed_level_t */
	uint32_t alarms;           /* bit n = pwrseq_alarm_t n active */
	bool     halted;
	bool     rb_deferred;      /* stage 8 skipped; retry is an operator act */
	bool     rb_enabled;       /* RB_PWR_EN commanded */
	bool     rb_gated;         /* RB_VCC_GATE commanded */
	bool     rb_locked;        /* pwrseq's fused verdict: gated AND the pin
				    * reads locked AND EXTREF_MON is in band */
	uint8_t  rb_auto_retries;
	bool     ov_latched;       /* the firmware-side 26 V latch stands */
	bool     wdt_armed;
	bool     relay_eligible;
	bool     display_on;
	bool     panel_led_on;
	bool     gps_on;
	bool     ant_bias_on;
	bool     disc_started;
	bool     phy_released;
	bool     pfi_seen;         /* a power-fail early warning has fired */
	bool     pfi_expected;     /* a commanded POE_KILL was already armed */

	/* -------------------------------------------------- observed ------- */
	bool     rb_wanted;        /* cfg pwr.rb.policy: this unit has an FE */
	bool     rb_lock_pin;      /* RB_LOCK (PB13), polarity applied */
	bool     rb_ov_det;        /* RB_OV_DET (PE3) reads high right now */
	uint32_t extref_hz;        /* EXTREF_MON (PB14/TIM12) measurement */
	bool     extref_valid;     /* that measurement is fresh and trustworthy */
	bool     extref_in_band;   /* ...and inside the 10 MHz acceptance band */
	uint32_t tick_mono_ms;     /* when this observation was taken */
} sts_pwrseq_snap_t;

/* Copy the published sequencer view.
 *
 * Lock-free and O(1): a bounded seqlock retry against a single writer, so it
 * cannot park the caller behind the housekeeping thread and cannot delay that
 * thread either. Callable from any thread, not from an ISR.
 *
 * @retval 0        Written.
 * @retval -EINVAL  @p out is NULL.
 * @retval -ENODEV  The sequencer never started; @p out is zeroed.
 * @retval -EAGAIN  The publisher won every retry; @p out is zeroed. Only
 *                  reachable if the reader is preempted repeatedly inside the
 *                  copy, which needs eight consecutive 250 ms ticks to land in
 *                  the same few microseconds. */
int sts_pwrseq_snapshot(sts_pwrseq_snap_t *out);

/* ---- alarms / health ---------------------------------------------------- */
/* Bitmask view of active alarms (fault-module alarm ids, FAULT_ALARM_BIT). */
uint64_t sts_alarms_active(void);

/* Raise or clear a software alarm (fault_alarm_id_t >= 32). Scanned signals
 * 0..31 are owned by the io_scan thread and must not be set this way. */
int sts_alarm_set(uint8_t alarm_id, bool active);

/* Bitmask view of LATCHED alarms — every alarm that has fired since the last
 * clear, whether or not its condition still holds. The expected-off mask does
 * not apply: the latch is a historical record. */
uint64_t sts_alarms_latched(void);

/* Acknowledge one latch, so an alarm that has gone away stops being reported.
 *
 * Unlike sts_alarm_set(), scanned-signal ids (0..31) ARE accepted: the io_scan
 * thread owns those signals' active state, not their latch. A latch whose
 * condition is still true is refused — acknowledging a live fault would make
 * the alarm page read clean over it.
 *
 * @retval 0        The latch is gone.
 * @retval -EINVAL  @p alarm_id is not an alarm id.
 * @retval -ENOENT  Nothing was latched.
 * @retval -EBUSY   The condition is still active; the latch stands. */
int sts_alarm_clear(uint8_t alarm_id);

/* Acknowledge every latch whose condition has gone away. Still-active alarms
 * keep theirs. Returns 0; read sts_alarms_latched() for the result. */
int sts_alarm_clear_all(void);

/* ---- operator recovery actions ------------------------------------------ */
/*
 * The field-service set: everything a technician standing at the board can do
 * to a subsystem that has latched itself off. Each is implemented by the area
 * that owns the state, and every one of them is a REQUEST rather than a call
 * wherever the owning subsystem is single-threaded — the same arrangement, and
 * for the same reason, as the GNSS operator requests above: a management thread
 * must never mutate a timing or sequencing context underneath the thread that
 * owns it (ARCHITECTURE.md §10 invariant 10).
 */

/* Restore GNSS antenna bias after a latched short (spec §3.7: there is no
 * automatic recovery, by design — a self-restoring bias would flap into a
 * genuine fault at the debounce period forever).
 *
 * Queued for the gnss thread, which applies it within one of its ticks and logs
 * the result. Idempotent: safe when nothing is latched.
 *
 * @retval 0        Queued.
 * @retval -ENODEV  The gnss thread never started.
 * @retval -EBUSY   A previous GNSS operator request has not been drained. */
int sts_gnss_ant_reenable(void);

/* Clear core/refsel's sticky reference-flap latch (REFSEL_FLAG_FLAP_ALARM) and
 * the switch-failed latch with it.
 *
 * Queued for the discipline thread, which owns the refsel context. Non-blocking
 * and idempotent.
 *
 * @retval 0        Queued.
 * @retval -ENODEV  The discipline thread never started. */
int sts_ref_clear_flap(void);

/* Clock-security-system events since boot: the count of times the STM32 CSS
 * fired because the HSE feeding PH0 stopped. A non-zero value that keeps
 * growing is the signature of a marginal reference or a mux handoff that is not
 * settling. Read-only and cheap. */
uint32_t sts_clock_css_events(void);

/* Clear the firmware-side rubidium over-voltage latch and pulse RB_OV_RESET
 * (PD3). The autonomous 26 V hardware latch is the real backstop; this is the
 * acknowledgement that lets pwrseq leave PWRSEQ_ALARM_RB_OV.
 *
 * Queued for the housekeeping thread's 4 Hz sequencer pass, which owns the
 * pwrseq context and its action queue. The outcome is logged there.
 *
 * @retval 0        Queued.
 * @retval -ENODEV  The sequencer never started.
 * @retval -EBUSY   The same request is already pending. */
int sts_pwrseq_ov_clear(void);

/* Operator-initiated retry of the guarded rubidium sequence, for instance once
 * the PoE budget frees up. Clears the deferral and the rubidium alarms and
 * re-enters stage 8 at its first step, so the full guarded sequence runs again.
 * Refused by core/pwrseq while the sequencer is halted.
 *
 * Queued exactly as sts_pwrseq_ov_clear() is; same return values. */
int sts_pwrseq_rb_retry(void);

/* Assert POE_KILL (PE15): drop the PD's own supply so the PSE re-powers the
 * board. A cold cycle, not a reboot — everything volatile is lost and recovery
 * depends on the PSE.
 *
 * FMT §5.2 lists POE_KILL under G3, and the MP object `pwr.poe.kill` carries
 * that class; this entry point performs no confirmation of its own, so a caller
 * that is not behind a G3 guard must not use it.
 *
 * Queued exactly as sts_pwrseq_ov_clear() is; same return values. The board
 * therefore stays up long enough for the caller's reply to reach the wire. */
int sts_pwrseq_poe_kill(void);

/* Blink the status RGB blue for @p duration_ms (operator locate, the flag
 * MP_MIRROR_F_IDENTIFY reports). 0 stops it. Safe from any thread. */
void sts_supervisor_identify(uint32_t duration_ms);

/* ---- FE-5680A serial link (UART7 + the K1 RS-232/CMOS relay) ------------- */
/*
 * The Rb housekeeping port, as an operator sees it. The FE-5680A variant fitted
 * decides whether it speaks RS-232 through U46 or CMOS directly
 * (docs/rb_rs232_interface.md), and getting that wrong is silent: the link just
 * never answers. So the position is commissionable and observable rather than
 * assumed.
 */
typedef struct {
	uint8_t  mode;        /* 0 = RS-232 through U46, 1 = direct CMOS */
	bool     locked;      /* RB_LOCK (PB13), variant polarity applied */
	bool     rail_up;     /* RB_PWR_EN asserted, so U46 has a supply */
	bool     tunnel_open; /* a maintenance raw tunnel holds the port */
	uint32_t tx_bytes;
	uint32_t rx_bytes;
	uint32_t overruns;
} sts_rb_serial_t;

/* Snapshot the Rb serial link. @retval 0 / -EINVAL for NULL. Fields read as
 * zero/false when the platform never brought UART7 up. */
int sts_rb_serial_status(sts_rb_serial_t *out);

/* Move the K1 DPDT relay. Blocks for the contact settling time.
 *
 * @retval 0        Moved (or already there).
 * @retval -EINVAL  @p mode is not 0 or 1.
 * @retval -EBUSY   A raw tunnel holds the port.
 * @retval -ENODEV  UART7 was never initialised. */
int sts_rb_serial_set_mode(uint8_t mode);

/* Liveness: each area calls this periodically; the supervisor ANDs all
 * registered bits before kicking the external watchdog. id is allocated
 * via sts_liveness_register at init.
 *
 * A registered participant that stops feeding for its deadline stops the WDT
 * kick, so register only from a thread that will genuinely run periodically. */
int  sts_liveness_register(const char *name);      /* -> id or negative */
void sts_liveness_feed(int id);

/* ---- local UI input ----------------------------------------------------- */
/* Front-panel input events. Buttons, the encoder switch, the capacitive-touch
 * INT and the reed switch all arrive from the 1 kHz GPIOF/GPIOG scan
 * (interface ref §5) and are dispatched to the UI area through
 * sts_ui_post_input(). The encoder *rotation* does not come from the scan —
 * TIM1 decodes it in hardware — so the UI area posts those itself if it wants
 * them in the same queue. */
typedef enum {
	STS_INPUT_BUTTON = 0,   /* value 1 = press, 0 = release */
	STS_INPUT_BUTTON_LONG,  /* held past the long-press threshold */
	STS_INPUT_BUTTON_REPEAT,/* auto-repeat tick while held */
	STS_INPUT_TOUCH,        /* FT6336 INT asserted; service over I2C */
	STS_INPUT_PROX,         /* reed switch; value 1 = magnet present */
	STS_INPUT_ENCODER,      /* value = signed detent delta */
	STS_INPUT_TYPE_COUNT,
} sts_input_type_t;

typedef struct {
	uint8_t  type;    /* sts_input_type_t */
	uint8_t  id;      /* fault_sig_t of the source pin; 0 for ENCODER */
	int16_t  value;
	uint32_t mono_ms; /* scan timestamp the event was committed at */
} sts_input_evt_t;

/* Implemented by the UI area; __weak no-op drop when the area is absent.
 * Called from the io_scan thread (priority 11) — must not block. */
void sts_ui_post_input(const sts_input_evt_t *evt);

/* ---- panel LED backlight (platform owns the LPTIM2 PWM) ----------------- */
/* PANEL_LED_PWM (PE0, LPTIM2_CH2 AF3) duty, 0..100 %. 0 also drops
 * PANEL_LED_EN (PC0); any non-zero duty asserts it. Returns 0 on success. */
int sts_panel_led_set(uint8_t duty_pct);

/* Last commanded duty, 0..100. */
uint8_t sts_panel_led_get(void);

/* ---- panel mirror (ui area produces, console area consumes) -------------- */
/* Publish the front-panel frame the Maintenance Protocol mirrors: the rendered
 * text-tile surface, the indicator state and the live input echo (channel 0x0A
 * and the `mirror.get` RPC).
 *
 * The UI area calls this after each ui_render(); the console area implements it
 * (src/zephyr/console/mp_glue.c). It crosses the seam here rather than through
 * mp_glue.h because that header is private to the console area
 * (ARCHITECTURE.md §2) — the same arrangement as sts_ui_post_input() in the
 * other direction, with a __weak no-op so a CONFIG_STS1000_CONSOLE=n image
 * links and simply mirrors nothing.
 *
 * The frame is DEEP-COPIED: @p frame->ch / ->attr / ->hint are read before this
 * returns and never retained, so the caller may reuse its render surface
 * immediately and may pass an mp_mirror_in_t living on its stack.
 *
 * Non-blocking by contract. The implementation takes a short mutex shared with
 * the MP tick and DROPS the frame rather than waiting, because the ui thread is
 * the lowest-priority thread in the system and must never be parked behind the
 * console. Callable from any cooperative thread, not from an ISR. */
void sts_mp_mirror_publish(const mp_mirror_in_t *frame);

/* ---- DFU / image state (console area implements, others read) ----------- */
bool sts_update_pending_confirm(void);  /* true while running unconfirmed */
int  sts_update_self_confirm(void);     /* called by supervisor when healthy */

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_STS_APP_H_ */
