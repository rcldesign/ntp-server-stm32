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

/* ---- DFU / image state (console area implements, others read) ----------- */
bool sts_update_pending_confirm(void);  /* true while running unconfirmed */
int  sts_update_self_confirm(void);     /* called by supervisor when healthy */

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_STS_APP_H_ */
