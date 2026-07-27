/*
 * STS1000 "Meridian" — port_store_t over Zephyr NVS on `storage_partition`.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * ARCHITECTURE.md §3 gives the application 64 KB of internal flash at
 * 0x1E0000 (eight 8 KB sectors). Spec §12 wants three things in it: the
 * configuration tree, the PFI fast-save record, and the calibration deltas
 * (which are cfg keys in group 0x0C, so they come along with the tree).
 *
 * Why raw NVS and not the settings subsystem
 * ------------------------------------------
 * core/cfg already owns the schema, the typing, the bounds and the
 * validate-then-commit transaction. The settings subsystem would add a second
 * naming scheme (dotted strings), a second serialisation, and a load callback
 * that has to re-enter cfg — all to reach the same NVS underneath. port_store_t
 * is a numeric-key map and nvs_read/nvs_write is a numeric-key map, so they are
 * wired straight together: one NVS record per cfg key, which also makes every
 * write atomic at exactly the granularity cfg reasons about.
 *
 * CONFIG_SETTINGS_NVS is enabled in prj.conf and would target this same
 * partition, but its backend only touches flash from settings_subsys_init().
 * Nothing in this application calls that, so the two cannot collide. Should a
 * later wave need the settings subsystem (for a Zephyr subsystem that insists
 * on it), give it its own partition rather than sharing this one.
 *
 * ID space
 * --------
 *   0x0100 .. 0xFDFF   cfg keys, mapped identically (group 0x01 is the lowest
 *                      group in cfg_schema.h, so nothing collides with 0x00xx)
 *   0xFE00 .. 0xFEFF   reserved for future internal records
 *   0xFF00             the PFI critical record
 *   0xFFFF             reserved by NVS itself (sector-close ATE)
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/util.h>

#include "storage/sts_store.h"

LOG_MODULE_REGISTER(sts_store, CONFIG_STS1000_LOG_LEVEL);

BUILD_ASSERT(ENOENT == 2,
	     "port_store_t spells its not-found result -2; this libc disagrees");

/* ------------------------------------------------------------------------- */
/* Geometry                                                                  */
/* ------------------------------------------------------------------------- */

#define STORE_PARTITION storage_partition

BUILD_ASSERT(FIXED_PARTITION_EXISTS(STORE_PARTITION),
	     "storage_partition is missing from the board devicetree");

/** Lowest NVS id the cfg key space may use. */
#define STORE_ID_CFG_MIN 0x0100U
/** Highest NVS id the cfg key space may use. */
#define STORE_ID_CFG_MAX 0xFDFFU
/** The PFI critical record. */
#define STORE_ID_CRITICAL 0xFF00U

static struct nvs_fs store_fs;
static bool store_mounted;

bool sts_store_ready(void)
{
	return store_mounted;
}

/* ------------------------------------------------------------------------- */
/* port_store_t                                                              */
/* ------------------------------------------------------------------------- */

static bool cfg_id_ok(uint16_t id)
{
	return (id >= STORE_ID_CFG_MIN) && (id <= STORE_ID_CFG_MAX);
}

static int store_load(void *ctx, uint16_t id, void *buf, size_t cap)
{
	ssize_t rc;

	ARG_UNUSED(ctx);

	if ((buf == NULL) || (cap == 0U) || !cfg_id_ok(id)) {
		return -EINVAL;
	}
	if (!store_mounted) {
		/* An unmounted store holds nothing; cfg then defaults the key
		 * and counts it missing rather than corrupt. */
		return -ENOENT;
	}

	rc = nvs_read(&store_fs, id, buf, cap);
	if (rc == -ENOENT) {
		return -ENOENT;
	}
	if (rc < 0) {
		return (int)rc;
	}
	if ((size_t)rc > cap) {
		/*
		 * NVS reports the stored length even when only `cap` bytes were
		 * copied. A record that does not fit the caller's buffer cannot
		 * be decoded, so report it as unreadable instead of handing back
		 * a length the caller never received.
		 */
		LOG_WRN("cfg 0x%04x: stored record %d B > buffer %zu B", id,
			(int)rc, cap);
		return -EIO;
	}
	return (int)rc;
}

static int store_save(void *ctx, uint16_t id, const void *buf, size_t len)
{
	ssize_t rc;

	ARG_UNUSED(ctx);

	if (((buf == NULL) && (len != 0U)) || !cfg_id_ok(id)) {
		return -EINVAL;
	}
	if (!store_mounted) {
		return -EIO;
	}

	rc = nvs_write(&store_fs, id, buf, len);
	if (rc < 0) {
		LOG_ERR("cfg 0x%04x: NVS write failed (%d)", id, (int)rc);
		return (int)rc;
	}
	return 0;
}

static int store_erase(void *ctx, uint16_t id)
{
	int rc;

	ARG_UNUSED(ctx);

	if (!cfg_id_ok(id)) {
		return -EINVAL;
	}
	if (!store_mounted) {
		return -EIO;
	}

	rc = nvs_delete(&store_fs, id);
	return (rc < 0) ? rc : 0;
}

static const port_store_t store_ops = {
	.load = store_load,
	.save = store_save,
	.erase = store_erase,
	.ctx = NULL,
};

const port_store_t *sts_port_store(void)
{
	return &store_ops;
}

/* ------------------------------------------------------------------------- */
/* PFI fast-save                                                             */
/* ------------------------------------------------------------------------- */

/*
 * The shadow is updated from producer threads and, for the DAC code, possibly
 * from a high-priority context, so the setters take a spinlock rather than a
 * mutex. Everything under the lock is a handful of stores; flash never happens
 * there.
 */
static struct k_spinlock crit_lock;
static sts_critical_t crit_shadow = {
	.magic = STS_CRITICAL_MAGIC,
	.ver = STS_CRITICAL_VER,
};
static bool crit_dirty;

void sts_store_note_dac_code(uint16_t code)
{
	k_spinlock_key_t key = k_spin_lock(&crit_lock);

	if (crit_shadow.dac_code != code) {
		crit_shadow.dac_code = code;
		crit_dirty = true;
	}
	k_spin_unlock(&crit_lock, key);
}

void sts_store_note_leap(int16_t current, int16_t pending, bool valid)
{
	k_spinlock_key_t key = k_spin_lock(&crit_lock);
	uint8_t v = valid ? 1U : 0U;

	if ((crit_shadow.leap_current != current) ||
	    (crit_shadow.leap_pending != pending) ||
	    (crit_shadow.leap_valid != v)) {
		crit_shadow.leap_current = current;
		crit_shadow.leap_pending = pending;
		crit_shadow.leap_valid = v;
		crit_dirty = true;
	}
	k_spin_unlock(&crit_lock, key);
}

void sts_store_note_log_cursor(uint32_t cursor)
{
	k_spinlock_key_t key = k_spin_lock(&crit_lock);

	if (crit_shadow.log_cursor != cursor) {
		crit_shadow.log_cursor = cursor;
		crit_dirty = true;
	}
	k_spin_unlock(&crit_lock, key);
}

int sts_store_critical_load(sts_critical_t *out)
{
	sts_critical_t rec;
	ssize_t rc;

	if (out == NULL) {
		return -EINVAL;
	}
	if (!store_mounted) {
		return -ENOENT;
	}

	rc = nvs_read(&store_fs, STORE_ID_CRITICAL, &rec, sizeof(rec));
	if (rc != (ssize_t)sizeof(rec)) {
		return -ENOENT;
	}
	if ((rec.magic != STS_CRITICAL_MAGIC) || (rec.ver != STS_CRITICAL_VER)) {
		return -ENOENT;
	}

	*out = rec;
	return 0;
}

int sts_store_critical_flush(sts_critical_reason_t reason)
{
	k_spinlock_key_t key;
	sts_critical_t rec;
	ssize_t rc;
	bool dirty;

	key = k_spin_lock(&crit_lock);
	dirty = crit_dirty;
	crit_dirty = false;
	rec = crit_shadow;
	k_spin_unlock(&crit_lock, key);

	if (!dirty) {
		return 0;
	}
	if (!store_mounted) {
		return -EIO;
	}

	rec.magic = STS_CRITICAL_MAGIC;
	rec.ver = STS_CRITICAL_VER;
	rec.reason = (uint8_t)reason;
	rec.reserved = 0U;
	rec.mono_ms = (uint64_t)k_uptime_get();

	rc = nvs_write(&store_fs, STORE_ID_CRITICAL, &rec, sizeof(rec));
	if (rc < 0) {
		/* Put the dirty flag back so the next attempt retries. */
		key = k_spin_lock(&crit_lock);
		crit_dirty = true;
		k_spin_unlock(&crit_lock, key);
		LOG_ERR("critical record write failed (%d)", (int)rc);
		return -EIO;
	}

	LOG_DBG("critical record saved (reason %u, dac %u, cursor %u)",
		(unsigned int)reason, (unsigned int)rec.dac_code,
		(unsigned int)rec.log_cursor);
	return 0;
}

static void crit_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	(void)sts_store_critical_flush(STS_CRITICAL_REASON_PFI);
}

static K_WORK_DEFINE(crit_work, crit_work_handler);

void sts_store_critical_flush_from_isr(void)
{
	/*
	 * The system workqueue runs at a cooperative priority, so it pre-empts
	 * every application thread. That is what the PFI hold-up window needs:
	 * the record must reach flash before the rail collapses, and a
	 * millisecond of stalled telemetry is irrelevant at that point.
	 */
	(void)k_work_submit(&crit_work);
}

/* ------------------------------------------------------------------------- */
/* Mount                                                                     */
/* ------------------------------------------------------------------------- */

static int store_init(void)
{
	const struct device *flash = FIXED_PARTITION_DEVICE(STORE_PARTITION);
	struct flash_pages_info page;
	size_t part_size = FIXED_PARTITION_SIZE(STORE_PARTITION);
	int rc;

	if (!device_is_ready(flash)) {
		LOG_ERR("internal flash %s not ready: configuration will not "
			"persist",
			(flash != NULL) ? flash->name : "<null>");
		return 0; /* never fail the boot over this */
	}

	store_fs.flash_device = flash;
	store_fs.offset = FIXED_PARTITION_OFFSET(STORE_PARTITION);

	rc = flash_get_page_info_by_offs(flash, store_fs.offset, &page);
	if (rc != 0) {
		LOG_ERR("storage_partition page info failed (%d)", rc);
		return 0;
	}

	/*
	 * NVS needs at least two sectors (one is always reserved for garbage
	 * collection). 64 KB of 8 KB sectors gives eight.
	 */
	store_fs.sector_size = (uint16_t)page.size;
	store_fs.sector_count = (uint16_t)(part_size / page.size);
	if (store_fs.sector_count < 2U) {
		LOG_ERR("storage_partition holds %u sector(s); NVS needs 2",
			(unsigned int)store_fs.sector_count);
		return 0;
	}

	rc = nvs_mount(&store_fs);
	if (rc != 0) {
		LOG_ERR("NVS mount failed (%d): configuration will not persist",
			rc);
		return 0;
	}

	store_mounted = true;
	LOG_INF("NVS mounted: %u x %u B at 0x%08lx",
		(unsigned int)store_fs.sector_count,
		(unsigned int)store_fs.sector_size,
		(unsigned long)store_fs.offset);
	return 0;
}

/*
 * APPLICATION level runs after every driver and before main(), which is where
 * the bring-up sequencer builds the cfg context. sts_port_store() is therefore
 * always answerable by the time cfg_init() wants it.
 */
SYS_INIT(store_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
