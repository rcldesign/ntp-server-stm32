/*
 * STS1000 Meridian — cross-area application API (Zephyr glue layer).
 *
 * The glue layer is split into four ownership areas (platform, net, console,
 * ui). This header is the ONLY interface between them. Initial version is
 * authored by the architect; the platform integrator owns it afterwards.
 * Areas must not include each other's private headers.
 *
 * Threading: unless stated otherwise, functions here are callable from any
 * cooperative/preemptive thread, not from ISRs.
 */
#ifndef STS1000_STS_APP_H_
#define STS1000_STS_APP_H_

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

/* ---- config ------------------------------------------------------------- */
/* The single live cfg context (loaded before any area starts). */
cfg_ctx_t *sts_cfg(void);

/* Areas register appliers invoked after a successful cfg commit for the
 * groups they own (group = high byte of key id). Called from the committing
 * thread; must be quick or defer to the area's own thread. */
typedef void (*sts_cfg_apply_fn)(void *ctx, uint8_t group);
int sts_cfg_register_applier(uint8_t group, sts_cfg_apply_fn fn, void *ctx);

/* ---- logging ------------------------------------------------------------ */
logring_ctx_t *sts_logring(void);
/* Convenience structured-log emit (maps to logring subsys/level ids). */
void sts_log(uint8_t subsys, uint8_t level, const char *fmt, ...);

/* ---- status / telemetry for MCP + shell + SNMP -------------------------- */
/* Encode the versioned status struct for an MCP STATUS_GET group
 * (mcp_wire.h group ids). Returns encoded length or negative error.
 * Implemented by the platform area, aggregating all areas' state. */
int sts_status_encode(uint8_t group, uint8_t *buf, size_t cap);

/* ---- alarms / health ---------------------------------------------------- */
/* Bitmask view of active alarms (fault-module alarm ids). */
uint32_t sts_alarms_active(void);

/* Liveness: each area calls this periodically; the supervisor ANDs all
 * registered bits before kicking the external watchdog. id is allocated
 * via sts_liveness_register at init. */
int  sts_liveness_register(const char *name);      /* -> id or negative */
void sts_liveness_feed(int id);

/* ---- DFU / image state (console area implements, others read) ----------- */
bool sts_update_pending_confirm(void);  /* true while running unconfirmed */
int  sts_update_self_confirm(void);     /* called by supervisor when healthy */

#ifdef __cplusplus
}
#endif

#endif /* STS1000_STS_APP_H_ */
