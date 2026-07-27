/*
 * STS1000 "Meridian" — console area: persistent storage façade.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Two independent stores live behind this header (ARCHITECTURE.md §3,
 * spec §12):
 *
 *   internal flash / NVS on `storage_partition` — the port_store_t that
 *       core/cfg persists through, plus the PFI fast-save record. Small,
 *       always present, survives a dead SPI-NOR.
 *
 *   external SPI-NOR / LittleFS on `/lfs` — the log spool and bulk assets.
 *       ARCHITECTURE.md §3: "NOR is not required to boot", so every entry
 *       point here degrades instead of failing.
 *
 * The NVS backend mounts from a SYS_INIT hook at APPLICATION level, i.e.
 * before main() runs the bring-up sequencer, so sts_port_store() is already
 * answerable by the time the platform area builds its cfg context.
 *
 * Threading: every function is callable from any thread. Only
 * sts_store_critical_flush_from_isr() is ISR-safe; everything else may block
 * on the NVS mutex or on flash.
 */

#ifndef STS1000_ZEPHYR_STORAGE_STS_STORE_H_
#define STS1000_ZEPHYR_STORAGE_STS_STORE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "port/port_crypto.h"
#include "port/port_store.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------ NVS backend */

/**
 * @brief The persistent key/value store backing core/cfg.
 *
 * Keys are the cfg numeric IDs (0xGGII). The ops honour the port_store_t
 * contract exactly, including the -2 ("no such key") load result.
 *
 * INTEGRATION NOTE (platform area / Wave 3a): sts_app.h has no store
 * registration API, so the platform area must pass this pointer to
 * cfg_init() when it builds the context returned by sts_cfg():
 *
 *     #include "storage/sts_store.h"
 *     cfg_init(&ctx, sts_port_store());
 *     cfg_load_all(&ctx, &corrupt);
 *
 * @return Never NULL. When the NVS mount failed the ops are still wired but
 *         report -EIO on save/erase and -2 on load, so cfg falls back to its
 *         schema defaults and the box still boots.
 */
const port_store_t *sts_port_store(void);

/** True once the NVS backend has mounted `storage_partition`. */
bool sts_store_ready(void);

/* ------------------------------------------------- PFI fast-save (spec §12) */

/** Magic word of a valid critical record. */
#define STS_CRITICAL_MAGIC 0x53544331U /* "STC1" */
/** Layout version of sts_critical_t. Bump on any field change. */
#define STS_CRITICAL_VER   1U

/**
 * The volatile-critical set: state that is cheap to write, expensive to
 * re-acquire, and worthless if it arrives late. Written on the PFI
 * early-warning edge (spec §2, docs/sts1000_power_fail_input.md) and, so a
 * hard power cut without PFI still leaves something useful, once a minute
 * whenever it is dirty.
 */
typedef struct {
	uint32_t magic;        /* STS_CRITICAL_MAGIC */
	uint16_t ver;          /* STS_CRITICAL_VER */
	uint16_t dac_code;     /* last DAC1_OUT1 code written by `discipline` */
	int16_t  leap_current; /* current TAI-UTC offset, seconds */
	int16_t  leap_pending; /* announced offset after the next event */
	uint8_t  leap_valid;   /* non-zero when the leap fields are trustworthy */
	uint8_t  reason;       /* sts_critical_reason_t of the write */
	uint16_t reserved;
	uint32_t log_cursor;   /* logring sequence the NOR spool reached */
	uint64_t mono_ms;      /* uptime at the write */
} sts_critical_t;

/** Why a critical record was written. */
typedef enum {
	STS_CRITICAL_REASON_PERIODIC = 0,
	STS_CRITICAL_REASON_PFI      = 1,
	STS_CRITICAL_REASON_SHUTDOWN = 2,
} sts_critical_reason_t;

/**
 * Stage the last DAC code written to OCXO_VC.
 *
 * ISR-safe and lock-free; only marks the RAM shadow dirty. Nothing is written
 * to flash until a flush.
 */
void sts_store_note_dac_code(uint16_t code);

/** Stage the leap-second state tracked from UBX-NAV-TIMELS. ISR-safe. */
void sts_store_note_leap(int16_t current, int16_t pending, bool valid);

/** Stage the log-spool cursor. ISR-safe. */
void sts_store_note_log_cursor(uint32_t cursor);

/**
 * Read back the record saved by the previous power cycle.
 *
 * @retval 0        @p out holds a valid record.
 * @retval -ENOENT  Nothing was ever saved, or the record is unreadable.
 * @retval -EINVAL  @p out is NULL.
 */
int sts_store_critical_load(sts_critical_t *out);

/**
 * Write the staged record now. Thread context only — this programs flash.
 *
 * @param reason  Recorded in the saved record.
 * @retval 0        Written, or nothing was dirty.
 * @retval -EIO     The NVS backend is not mounted or refused the write.
 */
int sts_store_critical_flush(sts_critical_reason_t reason);

/**
 * ISR-safe PFI hook: schedule sts_store_critical_flush(PFI) on the system
 * workqueue, which is cooperative and therefore runs ahead of every
 * application thread.
 *
 * INTEGRATION NOTE (platform area / Wave 3a): call this from the PFI EXTI8
 * callback (PE8, docs/sts1000_power_fail_input.md). The hold-up window is
 * milliseconds; three NVS records fit comfortably.
 */
void sts_store_critical_flush_from_isr(void);

/* ------------------------------------------------------- LittleFS on /lfs */

/** Mount point of the SPI-NOR volume. */
#define STS_FS_MOUNT_POINT "/lfs"

/** True once `/lfs` is mounted. */
bool sts_fs_ready(void);

/**
 * Attempt (or re-attempt) the `/lfs` mount.
 *
 * Idempotent: returns 0 immediately when already mounted. A blank NOR is
 * formatted on the first attempt. Attempts are capped
 * (CONFIG_STS1000_LFS_MOUNT_ATTEMPTS) so a dead device cannot turn the caller
 * into an erase loop.
 *
 * @retval 0        Mounted.
 * @retval -ENODEV  The SPI-NOR is absent or its driver never initialised.
 * @retval -EPERM   The attempt budget is spent; the volume is given up on.
 * @retval <0       Whatever fs_mount() returned.
 */
int sts_fs_try_mount(void);

/* ------------------------------------------------------------ crypto ports */

/** mbedTLS-backed one-shot primitives (AES-ECB, SHA-256, HMAC, CSPRNG). */
const port_crypto_t *sts_port_crypto(void);

/** mbedTLS-backed incremental SHA-256, used by the DFU image verify. */
const port_sha256_stream_t *sts_port_sha256_stream(void);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_STORAGE_STS_STORE_H_ */
