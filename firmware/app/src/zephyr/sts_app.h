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

/* ---- alarms / health ---------------------------------------------------- */
/* Bitmask view of active alarms (fault-module alarm ids, FAULT_ALARM_BIT). */
uint64_t sts_alarms_active(void);

/* Raise or clear a software alarm (fault_alarm_id_t >= 32). Scanned signals
 * 0..31 are owned by the io_scan thread and must not be set this way. */
int sts_alarm_set(uint8_t alarm_id, bool active);

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
