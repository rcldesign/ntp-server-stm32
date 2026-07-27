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

/* Monotonic milliseconds since boot; the timebase every core module's
 * mono_ms argument expects. */
uint64_t sts_mono_ms(void);

/* ---- config ------------------------------------------------------------- */
/* The single live cfg context (loaded before any area starts). Never NULL
 * once sts_platform_init() has returned. */
cfg_ctx_t *sts_cfg(void);

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
 * thread; must be quick or defer to the area's own thread. */
typedef void (*sts_cfg_apply_fn)(void *ctx, uint8_t group);
int sts_cfg_register_applier(uint8_t group, sts_cfg_apply_fn fn, void *ctx);

/* Commit the staged config set and run the appliers for every group touched.
 * Areas must use this rather than calling cfg_commit() directly, so the
 * appliers actually run. Returns cfg_commit()'s result. */
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
