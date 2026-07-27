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
#include "port/port_image.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* sts_usb.c — composite USB device, VBUS-gated                              */
/* ------------------------------------------------------------------------- */

/** Configure USB_VBUS_SENSE (PE2) and start the VBUS gate poller. */
int sts_usb_start(void);

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

/** Start the health-gate poller. No-op when the running image is confirmed. */
int sts_selfconfirm_start(void);

/** Latest gate evaluation, for the shell and for the log line. */
typedef struct {
	bool image_confirmed;  /* boot_is_img_confirmed() */
	bool min_age_met;      /* uptime past CONFIG_..._SELF_CONFIRM_MIN_S */
	bool cfg_loaded;       /* the platform area published a cfg context */
	bool store_ready;      /* NVS mounted, so cfg is genuinely persistent */
	bool link_ok;          /* USB configured or a network interface is up */
	bool deadline_passed;  /* gave up; MCUboot will revert on next boot */
	uint32_t uptime_s;
} sts_selfconfirm_status_t;

void sts_selfconfirm_status(sts_selfconfirm_status_t *out);

/* ------------------------------------------------------------------------- */
/* sts_mcp.c — the binary console channel on CDC-ACM #1                      */
/* ------------------------------------------------------------------------- */

/** Create the `mcp` thread (ARCHITECTURE.md §6, priority 14). */
int sts_mcp_start(void);

/** Engine counters, for `sts diag` and the shell. NULL before sts_mcp_start(). */
const mcp_stats_t *sts_mcp_stats(void);

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

#endif /* STS1000_ZEPHYR_CONSOLE_STS_CONSOLE_H_ */
