/*
 * STS1000 "Meridian" — LittleFS on the external SPI-NOR, mounted at /lfs.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * ARCHITECTURE.md §3: "External SPI-NOR (MX25L25645, 32 MB, SPI4): LittleFS
 * mount `/lfs` — bulk config export, calibration tables, log spool, GNSS cache,
 * staged assets. NOR is **not** required to boot."
 *
 * That last sentence is the whole design of this file. The mount is:
 *
 *   lazy   — never attempted from an init hook, because the NOR node is
 *            `zephyr,deferred-init` and main() only releases NOR_RST_N (PE10)
 *            and calls device_init() during Stage 2 (interface ref §2). An
 *            init hook could therefore run while the part is still in reset;
 *   retried — a first attempt that fails is retried by the logger thread with
 *            a backoff, because a slow rail or a late reset release is a
 *            transient, not a verdict;
 *   capped — after CONFIG_STS1000_LFS_MOUNT_ATTEMPTS failures the volume is
 *            given up on. Zephyr's LittleFS auto-formats a volume it cannot
 *            mount, so an uncapped retry against a dead or mis-wired part
 *            would be an erase loop on 32 MB of flash;
 *   optional — every failure is a WARN and a degraded feature (no log spool,
 *            no bulk export), never a boot failure.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

#include "storage/sts_store.h"

LOG_MODULE_REGISTER(sts_lfs, CONFIG_STS1000_LOG_LEVEL);

#if defined(CONFIG_STS1000_LFS_MOUNT_ATTEMPTS)
#define LFS_MOUNT_ATTEMPTS CONFIG_STS1000_LFS_MOUNT_ATTEMPTS
#else
#define LFS_MOUNT_ATTEMPTS 3
#endif

#define LFS_PARTITION lfs_partition
#define NOR_NODE      DT_NODELABEL(nor_flash)

BUILD_ASSERT(FIXED_PARTITION_EXISTS(LFS_PARTITION),
	     "lfs_partition is missing from the board devicetree");

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(sts_lfs_data);

static struct fs_mount_t sts_lfs_mnt = {
	.type = FS_LITTLEFS,
	.fs_data = &sts_lfs_data,
	.storage_dev = (void *)FIXED_PARTITION_ID(LFS_PARTITION),
	.mnt_point = STS_FS_MOUNT_POINT,
};

static K_MUTEX_DEFINE(lfs_lock);
static bool lfs_mounted;
static unsigned int lfs_attempts;

bool sts_fs_ready(void)
{
	return lfs_mounted;
}

int sts_fs_try_mount(void)
{
	const struct device *nor = DEVICE_DT_GET(NOR_NODE);
	struct fs_statvfs st;
	int rc;

	k_mutex_lock(&lfs_lock, K_FOREVER);

	if (lfs_mounted) {
		rc = 0;
		goto out;
	}
	if (lfs_attempts >= (unsigned int)LFS_MOUNT_ATTEMPTS) {
		rc = -EPERM;
		goto out;
	}

	/*
	 * device_is_ready() is false both when the part is absent and when
	 * Stage 2's device_init() failed. Do not count that as an attempt: the
	 * sequencer may simply not have reached Stage 2 yet.
	 */
	if (!device_is_ready(nor)) {
		rc = -ENODEV;
		goto out;
	}

	lfs_attempts++;

	rc = fs_mount(&sts_lfs_mnt);
	if (rc != 0) {
		LOG_WRN("/lfs mount attempt %u/%u failed (%d)", lfs_attempts,
			(unsigned int)LFS_MOUNT_ATTEMPTS, rc);
		if (lfs_attempts >= (unsigned int)LFS_MOUNT_ATTEMPTS) {
			LOG_ERR("/lfs given up on: log spool, bulk config "
				"export and the calibration store are "
				"unavailable this boot");
		}
		goto out;
	}

	lfs_mounted = true;

	if (fs_statvfs(STS_FS_MOUNT_POINT, &st) == 0) {
		LOG_INF("/lfs mounted: %lu blocks of %lu B, %lu free",
			(unsigned long)st.f_blocks, (unsigned long)st.f_frsize,
			(unsigned long)st.f_bfree);
	} else {
		LOG_INF("/lfs mounted");
	}

out:
	k_mutex_unlock(&lfs_lock);
	return rc;
}
