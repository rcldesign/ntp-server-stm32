/*
 * STS1000 "Meridian" — console-area internal header.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/console/. The platform, net and ui areas talk to this
 * area through src/zephyr/sts_app.h and nothing else (ARCHITECTURE.md §2).
 * src/zephyr/storage/sts_store.h is the one exception: it is a deliberately
 * public façade so the platform area can hand core/cfg a persistent store.
 */

#ifndef STS1000_ZEPHYR_CONSOLE_STS_CONSOLE_H_
#define STS1000_ZEPHYR_CONSOLE_STS_CONSOLE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mcp/mcp.h"
#include "fwupd/fwupd.h"
#include "fwupd/rb_fwupd.h"
#include "mp/mp.h"
#include "port/port_image.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* sts_usb.c — composite USB device, VBUS-gated                              */
/* ------------------------------------------------------------------------- */

/** Configure USB_VBUS_SENSE (PE2) and attach if VBUS is already present. */
int sts_usb_start(void);

/** Re-evaluate the VBUS gate; called from the console supervisor thread. */
void sts_usb_poll(void);

/** True while VBUS is present on the USB-C port. */
bool sts_usb_vbus_present(void);

/** True once the host has driven the device to USB_DC_CONFIGURED. */
bool sts_usb_configured(void);

/* ------------------------------------------------------------------------- */
/* sts_dfu.c — port_image_t over flash_map + MCUboot                         */
/* ------------------------------------------------------------------------- */

/** The staging-slot / bootloader port handed to core/mcp. Never NULL. */
const port_image_t *sts_dfu_port(void);

/** Erase granularity of the staging slot, in bytes. */
uint32_t sts_dfu_erase_granularity(void);

/**
 * Flash write-block size of the staging slot, in bytes (16 on STM32H5).
 *
 * Passed to core/mcp as mcp_wiring_t::dfu_write_block so it keeps every
 * non-final FW_DATA chunk a multiple of this; staging_write then buffers only
 * the final short block.
 */
uint32_t sts_dfu_write_block(void);

/**
 * Read one slot's image table entry.
 *
 * Shared by the MCP FW_INFO handler (through the port) and the `sts fw info`
 * shell command.
 *
 * @param slot  0 = running (slot0), 1 = staged (slot1).
 */
int sts_dfu_image_info(uint8_t slot, port_image_info_t *out);

/** Human-readable MCUboot swap type for the next boot, e.g. "test". */
const char *sts_dfu_swap_type_name(void);

/* ------------------------------------------------------------------------- */
/* sts_selfconfirm.c — spec §8.3 test-image self-confirmation                */
/* ------------------------------------------------------------------------- */

/** Announce the image state at boot. No-op when the image is confirmed. */
int sts_selfconfirm_start(void);

/**
 * Report on the health gate, and abandon the attempt once the deadline passes.
 *
 * Called from the console supervisor thread. It does NOT confirm: the only
 * confirming path is sts_update_self_confirm() (sts_app.h), driven by the
 * platform supervisor. A second, weaker gate racing the strict one is how an
 * undisciplined image used to confirm itself sixty seconds after boot — see the
 * header of sts_selfconfirm.c.
 */
void sts_selfconfirm_poll(void);

/** Latest gate evaluation, for the shell and for the log line. */
typedef struct {
	bool image_confirmed;  /* boot_is_img_confirmed() */
	bool min_age_met;      /* uptime past CONFIG_..._SELF_CONFIRM_MIN_S */
	bool cfg_loaded;       /* the platform area published a cfg context */
	bool store_ready;      /* NVS mounted, so cfg is genuinely persistent */
	bool link_ok;          /* USB configured or a network interface is up */
	bool clock_locked;     /* quality lock_state == QUALITY_LOCK_LOCKED */
	bool serving_primary;  /* quality stratum == QUALITY_STRATUM_PRIMARY */
	bool deadline_passed;  /* gave up; MCUboot will revert on next boot */
	uint32_t uptime_s;
	uint32_t deadline_s;   /* derived from tim.lock.hold, not a constant */
} sts_selfconfirm_status_t;

void sts_selfconfirm_status(sts_selfconfirm_status_t *out);

/* ------------------------------------------------------------------------- */
/* sts_mcp.c — the binary console channel on CDC-ACM #1                      */
/* ------------------------------------------------------------------------- */

/** Create the `mcp` thread (ARCHITECTURE.md §6, priority 14). */
int sts_mcp_start(void);

/** Engine counters, for `sts diag` and the shell. NULL before sts_mcp_start(). */
const mcp_stats_t *sts_mcp_stats(void);

/**
 * Tell the MCP thread the physical link went away (VBUS loss).
 *
 * Safe from any context. The thread performs the actual mcp_reset_session().
 */
void sts_mcp_notify_link_down(void);

/** Serial-link counters kept by the glue rather than the engine. */
typedef struct {
	uint32_t rx_bytes;
	uint32_t tx_bytes;
	uint32_t rx_overruns;   /* bytes dropped because the RX ring was full */
	uint32_t tx_backpressure; /* frames the TX ring could not take at once */
	uint32_t link_drops;    /* DTR transitions that reset the session */
	bool     dtr;
} sts_mcp_link_stats_t;

void sts_mcp_link_stats(sts_mcp_link_stats_t *out);

/* ------------------------------------------------------------------------- */
/* sts_diag.c — MCP DIAG sub-functions                                       */
/* ------------------------------------------------------------------------- */

/** mcp_diag_fn: sub 1 = I²C scan, sub 3 = thread/CPU stats. */
int sts_diag_encode(void *user, uint8_t sub, uint8_t *buf, size_t cap);

/**
 * Scan I²C1 for responding addresses.
 *
 * @param found  Receives a 128-bit map, one bit per 7-bit address, LSB-first.
 * @return Number of devices that answered, or a negative errno.
 */
int sts_diag_i2c_scan(uint8_t found[16]);

/* ------------------------------------------------------------------------- */
/* sts_logspool.c — logger thread (ARCHITECTURE.md §6, priority 16)          */
/* ------------------------------------------------------------------------- */

int sts_logspool_start(void);

/** Log-spool counters, for `sts log` and DIAG. */
typedef struct {
	uint32_t records;     /* records drained from the ring */
	uint32_t spooled;     /* records written to /lfs */
	uint32_t spool_errors;
	uint32_t bridge_in;   /* Zephyr LOG messages forwarded into the ring */
	uint32_t bridge_dropped;
	uint32_t cursor;      /* next logring sequence to drain */
	bool     spool_open;
} sts_logspool_stats_t;

void sts_logspool_stats(sts_logspool_stats_t *out);

/* ------------------------------------------------------------------------- */
/* sts_shell.c                                                                */
/* ------------------------------------------------------------------------- */

/** Announce the shell command set once the console area is up. */
void sts_shell_announce(void);

#ifdef __cplusplus
}
#endif


/* ------------------------------------------------------------------------- */
/* fwupd_glue.c — core/fwupd bound to the board's three updatable ICs        */
/* ------------------------------------------------------------------------- */

/**
 * Wire core/fwupd's targets and register the whole component inventory.
 *
 * Only FWUPD_COMP_STM32_APP is permitted by default: it is the one path with a
 * signature check and an automatic revert behind it. The GNSS and Rb paths
 * destroy a peripheral with no way back and must be enabled explicitly.
 *
 * @retval 0          Ready.
 * @retval -EALREADY  Already initialised.
 * @retval other      A client or the orchestrator failed to initialise.
 */
int sts_fwupd_init(void);

/** The orchestrator context, or NULL before sts_fwupd_init(). */
fwupd_ctx_t *sts_fwupd_ctx(void);

/** The FE-5680A client context, or NULL before sts_fwupd_init(). */
rb_ctx_t *sts_fwupd_rb_ctx(void);

/**
 * Pump an open update session and enforce core/fwupd's timeouts.
 *
 * The console supervisor calls it every pass. It is bounded by construction:
 * with no session open it is one predicate, and none of this board's targets
 * registers a `poll` callback, so the work is timeout arithmetic plus — on the
 * one path that can fire from here — core/fwupd's guaranteed RESTORE.
 *
 * Tries the orchestrator lock with **K_NO_WAIT** and skips the pass on
 * contention, because sts_console.c's BUILD_ASSERT budgets exactly one lock wait
 * per supervisor pass and sts_mp_tick() already spends it.
 *
 * @retval 0        Stepped (or there was nothing to do).
 * @retval -ENODEV  sts_fwupd_init() has not run.
 * @retval -EBUSY   A console request holds the orchestrator; skipped.
 */
int sts_fwupd_step(void);

/**
 * The orchestrator as the MP control plane's port (mp.h), or NULL before
 * sts_fwupd_init().
 *
 * Every entry point takes this area's orchestrator mutex, which is why core gets
 * a port and not a `fwupd_ctx_t *`: the `fw.*` handlers run on the console RX
 * thread and sts_fwupd_step() on the supervisor, and `fwupd_ctx_t` is not
 * internally synchronised. See the threading block in fwupd_glue.c.
 */
const mp_fwupd_t *sts_fwupd_mp_port(void);

/**
 * Whole component inventory, for the shell. Locked; thread context only.
 *
 * @retval 0        Written; @p n rows.
 * @retval -ENOSPC  @p max was below FWUPD_COMP__COUNT; @p n rows were still
 *                  written, so a short buffer truncates rather than fails.
 * @retval -ENODEV  Not initialised.
 */
int sts_fwupd_inventory(fwupd_inv_row_t *out, size_t max, size_t *n);

/** One component's running version/identity, for the shell. Locked. */
int sts_fwupd_query(uint8_t comp, char *out, size_t cap);

/** Orchestrator progress, policy and counters, for the shell. Locked. */
int sts_fwupd_status(mp_fw_status_t *out);

#endif /* STS1000_ZEPHYR_CONSOLE_STS_CONSOLE_H_ */
