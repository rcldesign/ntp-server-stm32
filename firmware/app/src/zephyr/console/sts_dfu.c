/*
 * STS1000 "Meridian" — port_image_t over Zephyr flash_map + MCUboot.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * ARCHITECTURE.md §3 upgrade path 1: the PC tool streams a signed MCUboot
 * image into `slot1_partition` over the MCP console channel, core/mcp verifies
 * the SHA-256 and the image magic, and this file does the flash and bootloader
 * work underneath. Nothing here decides policy — core/mcp owns the session
 * state machine and is unit-tested against a RAM-backed version of exactly
 * this interface.
 *
 * Write buffering
 * ---------------
 * STM32H5 internal flash programs in 16-byte quad-words, while FW_DATA chunks
 * are arbitrary lengths up to 1024 B. Writes are therefore accumulated in an
 * aligned buffer and pushed out a whole buffer at a time; the FINAL short block
 * is padded to the write-block size with 0xFF (the erased value) when core/mcp
 * sets `flush`. The stream is then *sealed*: any further write without an
 * intervening erase would have to re-program a partially-written block, which
 * NOR flash cannot do, so it is refused rather than silently corrupting the
 * image.
 *
 * The write frontier is strictly sequential because core/mcp guarantees it —
 * a FW_DATA below the frontier is absorbed as a retransmit without reaching
 * this file, and one above it is refused with the expected offset. A
 * non-sequential write arriving here is a contract violation and returns
 * -EINVAL rather than being papered over.
 *
 * Trailer handling
 * ----------------
 * The last erase page of the staging slot is *excluded from staging_size()*
 * and reserved for the MCUboot image trailer. That matters for correctness,
 * not just tidiness: boot_request_upgrade() writes the boot magic into that
 * trailer with a plain flash write and no erase of its own, so the region has
 * to be blank first. It is erased here immediately before the upgrade request,
 * and erasing it is also exactly what cancels a pending swap, which is how
 * request_revert() is implemented.
 */

#include <zephyr/kernel.h>

#ifdef CONFIG_STS1000_CONSOLE

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>

#include "console/sts_console.h"
#include "port/port.h"

LOG_MODULE_REGISTER(sts_dfu, CONFIG_STS1000_LOG_LEVEL);

#define SLOT0_ID FIXED_PARTITION_ID(slot0_partition)
#define SLOT1_ID FIXED_PARTITION_ID(slot1_partition)

BUILD_ASSERT(FIXED_PARTITION_EXISTS(slot0_partition) &&
		     FIXED_PARTITION_EXISTS(slot1_partition),
	     "the A/B slot partitions are missing from the board devicetree");

/** Bytes accumulated before a flash program. Must be a multiple of the
 *  write-block size, which is checked against the live geometry at init. */
#if defined(CONFIG_STS1000_DFU_WRITE_BUF)
#define DFU_WBUF CONFIG_STS1000_DFU_WRITE_BUF
#else
#define DFU_WBUF 256
#endif

/** Fallback erase granularity if the flash driver cannot describe the slot. */
#define DFU_ERASE_GRAN_FALLBACK 8192U

struct dfu_ctx {
	const struct flash_area *fa; /* slot1, opened once at init */
	uint32_t erase_gran;         /* flash erase page, bytes */
	uint32_t usable;             /* staging bytes offered to core/mcp */
	uint32_t trailer_off;        /* start of the reserved trailer page */
	uint32_t align;              /* flash write-block size, bytes */
	uint8_t  erased_val;         /* what an erased byte reads back as */

	uint32_t buf_off;            /* flash offset of buf[0] */
	size_t   buf_n;              /* valid bytes in buf */
	uint32_t stream_end;         /* == buf_off + buf_n, the accepted frontier */
	bool     sealed;             /* a padded flush closed the stream */
	bool     ready;
	bool     open_failed;        /* do not retry a hard geometry failure */

	uint8_t buf[DFU_WBUF] __aligned(4);
};

static K_MUTEX_DEFINE(dfu_lock);

static struct dfu_ctx dfu = {
	.erase_gran = DFU_ERASE_GRAN_FALLBACK,
	.erased_val = 0xFFU,
};

/* ------------------------------------------------------------------------- */
/* Init                                                                      */
/* ------------------------------------------------------------------------- */

/* Zephyr mutexes are recursive, so callers already holding dfu_lock are fine. */
static int dfu_open(void)
{
	struct flash_pages_info page;
	const struct device *dev;
	int rc;

	k_mutex_lock(&dfu_lock, K_FOREVER);

	if (dfu.ready) {
		rc = 0;
		goto out;
	}
	if (dfu.open_failed) {
		rc = -ENODEV;
		goto out;
	}

	rc = flash_area_open(SLOT1_ID, &dfu.fa);
	if (rc != 0) {
		LOG_ERR("slot1 flash_area_open failed (%d): DFU unavailable", rc);
		dfu.open_failed = true;
		goto out;
	}

	dfu.align = flash_area_align(dfu.fa);
	if ((dfu.align == 0U) || ((DFU_WBUF % dfu.align) != 0U)) {
		LOG_ERR("flash write-block %u B does not divide the %u B DFU "
			"buffer",
			(unsigned int)dfu.align, (unsigned int)DFU_WBUF);
		flash_area_close(dfu.fa);
		dfu.fa = NULL;
		dfu.open_failed = true;
		rc = -EINVAL;
		goto out;
	}

	dfu.erased_val = flash_area_erased_val(dfu.fa);

	dev = flash_area_get_device(dfu.fa);
	if ((dev != NULL) &&
	    (flash_get_page_info_by_offs(dev, dfu.fa->fa_off, &page) == 0) &&
	    (page.size != 0U)) {
		dfu.erase_gran = (uint32_t)page.size;
	} else {
		LOG_WRN("slot1 page geometry unavailable; assuming %u B pages",
			(unsigned int)DFU_ERASE_GRAN_FALLBACK);
		dfu.erase_gran = DFU_ERASE_GRAN_FALLBACK;
	}

	if ((uint32_t)dfu.fa->fa_size <= dfu.erase_gran) {
		LOG_ERR("slot1 is only %u B; no room for an image plus trailer",
			(unsigned int)dfu.fa->fa_size);
		flash_area_close(dfu.fa);
		dfu.fa = NULL;
		dfu.open_failed = true;
		rc = -EINVAL;
		goto out;
	}

	/* Reserve the final erase page for the MCUboot trailer. */
	dfu.trailer_off = (uint32_t)dfu.fa->fa_size - dfu.erase_gran;
	dfu.usable = dfu.trailer_off;
	dfu.ready = true;

	LOG_INF("DFU staging: slot1 %u B, %u B usable, %u B pages, %u B writes",
		(unsigned int)dfu.fa->fa_size, (unsigned int)dfu.usable,
		(unsigned int)dfu.erase_gran, (unsigned int)dfu.align);

out:
	k_mutex_unlock(&dfu_lock);
	return rc;
}

uint32_t sts_dfu_erase_granularity(void)
{
	(void)dfu_open();
	return dfu.erase_gran;
}

/* ------------------------------------------------------------------------- */
/* Staging slot                                                              */
/* ------------------------------------------------------------------------- */

static uint32_t dfu_staging_size(void *ctx)
{
	ARG_UNUSED(ctx);

	if (dfu_open() != 0) {
		return 0U;
	}
	return dfu.usable;
}

/* Caller holds dfu_lock. */
static void dfu_stream_reset(void)
{
	dfu.buf_off = 0U;
	dfu.buf_n = 0U;
	dfu.stream_end = 0U;
	dfu.sealed = false;
}

static int dfu_staging_erase(void *ctx, uint32_t off, uint32_t len)
{
	int rc;

	ARG_UNUSED(ctx);

	if (dfu_open() != 0) {
		return PORT_ENOTSUP;
	}
	if (len == 0U) {
		return 0;
	}
	if ((off > dfu.usable) || (len > (dfu.usable - off))) {
		return PORT_EINVAL;
	}
	if (((off % dfu.erase_gran) != 0U) || ((len % dfu.erase_gran) != 0U)) {
		LOG_ERR("unaligned erase [0x%x, +0x%x)", (unsigned int)off,
			(unsigned int)len);
		return PORT_EINVAL;
	}

	k_mutex_lock(&dfu_lock, K_FOREVER);

	/*
	 * core/mcp only ever erases ahead of the write frontier, so an erase
	 * that reaches back into the buffered tail means the tool restarted the
	 * transfer. Drop whatever the previous attempt left behind.
	 */
	if (off < dfu.stream_end) {
		dfu_stream_reset();
	}

	rc = flash_area_erase(dfu.fa, (off_t)off, (size_t)len);

	k_mutex_unlock(&dfu_lock);

	if (rc != 0) {
		LOG_ERR("slot1 erase [0x%x, +0x%x) failed (%d)",
			(unsigned int)off, (unsigned int)len, rc);
	}
	return rc;
}

/* Caller holds dfu_lock. Programs the whole buffer, which is align-sized. */
static int dfu_buf_program(size_t n)
{
	int rc = flash_area_write(dfu.fa, (off_t)dfu.buf_off, dfu.buf, n);

	if (rc != 0) {
		LOG_ERR("slot1 write at 0x%x (%zu B) failed (%d)",
			(unsigned int)dfu.buf_off, n, rc);
		return rc;
	}
	dfu.buf_off += (uint32_t)n;
	dfu.buf_n = 0U;
	return 0;
}

static int dfu_staging_write(void *ctx, uint32_t off, const uint8_t *data,
			     size_t len, bool flush)
{
	int rc = 0;

	ARG_UNUSED(ctx);

	if ((data == NULL) && (len != 0U)) {
		return PORT_EINVAL;
	}
	if (dfu_open() != 0) {
		return PORT_ENOTSUP;
	}
	if ((off > dfu.usable) || (len > (size_t)(dfu.usable - off))) {
		return PORT_ENOSPC;
	}

	k_mutex_lock(&dfu_lock, K_FOREVER);

	/*
	 * Invariant: stream_end == buf_off + buf_n. A fresh (or just-erased)
	 * stream has all three at zero, so the first chunk must start at 0 —
	 * which is exactly what core/mcp does after FW_BEGIN.
	 */
	if (dfu.sealed || (off != dfu.stream_end)) {
		LOG_ERR("non-sequential staging write: got 0x%x, expected 0x%x%s",
			(unsigned int)off, (unsigned int)dfu.stream_end,
			dfu.sealed ? " (stream sealed)" : "");
		k_mutex_unlock(&dfu_lock);
		return PORT_EINVAL;
	}

	while (len > 0U) {
		size_t n = MIN(len, sizeof(dfu.buf) - dfu.buf_n);

		memcpy(&dfu.buf[dfu.buf_n], data, n);
		dfu.buf_n += n;
		dfu.stream_end += (uint32_t)n;
		data += n;
		len -= n;

		if (dfu.buf_n == sizeof(dfu.buf)) {
			rc = dfu_buf_program(sizeof(dfu.buf));
			if (rc != 0) {
				goto out;
			}
		}
	}

	if (flush) {
		if (dfu.buf_n > 0U) {
			size_t padded = ROUND_UP(dfu.buf_n, (size_t)dfu.align);

			memset(&dfu.buf[dfu.buf_n], dfu.erased_val,
			       padded - dfu.buf_n);
			rc = dfu_buf_program(padded);
			if (rc != 0) {
				goto out;
			}
			/* buf_off advanced by the padded length; wind it back
			 * to the real end of data so the invariant holds. */
			dfu.buf_off = dfu.stream_end;
		}
		dfu.sealed = true;
	}

out:
	k_mutex_unlock(&dfu_lock);
	return rc;
}

static int dfu_staging_read(void *ctx, uint32_t off, uint8_t *data, size_t len)
{
	int rc;

	ARG_UNUSED(ctx);

	if ((data == NULL) && (len != 0U)) {
		return PORT_EINVAL;
	}
	if (dfu_open() != 0) {
		return PORT_ENOTSUP;
	}
	if ((off > dfu.usable) || (len > (size_t)(dfu.usable - off))) {
		return PORT_EINVAL;
	}
	if (len == 0U) {
		return 0;
	}

	k_mutex_lock(&dfu_lock, K_FOREVER);

	rc = flash_area_read(dfu.fa, (off_t)off, data, len);

	/*
	 * Overlay anything still sitting in the write buffer. core/mcp only
	 * reads back after a flush, so this should never fire — but a read that
	 * silently returned erased flash for bytes the caller had already
	 * handed us would turn a correct image into a hash mismatch, and that
	 * failure is far too expensive to leave to convention.
	 */
	if ((rc == 0) && (dfu.buf_n > 0U)) {
		uint32_t bs = dfu.buf_off;
		uint32_t be = dfu.buf_off + (uint32_t)dfu.buf_n;
		uint32_t rs = off;
		uint32_t re = off + (uint32_t)len;

		if ((rs < be) && (re > bs)) {
			uint32_t s = MAX(rs, bs);
			uint32_t e = MIN(re, be);

			memcpy(&data[s - rs], &dfu.buf[s - bs], (size_t)(e - s));
		}
	}

	k_mutex_unlock(&dfu_lock);

	if (rc != 0) {
		LOG_ERR("slot1 read at 0x%x (%zu B) failed (%d)",
			(unsigned int)off, len, rc);
	}
	return rc;
}

/* ------------------------------------------------------------------------- */
/* Bootloader interface                                                      */
/* ------------------------------------------------------------------------- */

int sts_dfu_image_info(uint8_t slot, port_image_info_t *out)
{
	struct mcuboot_img_header hdr;
	uint8_t area_id;
	int rc;

	if ((out == NULL) || (slot > 1U)) {
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));
	out->slot = slot;
	area_id = (slot == 0U) ? SLOT0_ID : SLOT1_ID;

	rc = boot_read_bank_header(area_id, &hdr, sizeof(hdr));
	if ((rc == 0) && (hdr.mcuboot_version == 1U)) {
		out->valid = true;
		out->size = hdr.h.v1.image_size;
		out->version[0] = hdr.h.v1.sem_ver.major;
		out->version[1] = hdr.h.v1.sem_ver.minor;
		out->version[2] = hdr.h.v1.sem_ver.revision;
		out->version[3] = hdr.h.v1.sem_ver.build_num;
	}

	if (slot == 0U) {
		out->active = true;
		out->confirmed = boot_is_img_confirmed();
	} else {
		int swap = mcuboot_swap_type();

		out->pending = (swap == BOOT_SWAP_TYPE_TEST) ||
			       (swap == BOOT_SWAP_TYPE_PERM);
	}

	return 0;
}

const char *sts_dfu_swap_type_name(void)
{
	switch (mcuboot_swap_type()) {
	case BOOT_SWAP_TYPE_NONE:
		return "none";
	case BOOT_SWAP_TYPE_TEST:
		return "test";
	case BOOT_SWAP_TYPE_PERM:
		return "permanent";
	case BOOT_SWAP_TYPE_REVERT:
		return "revert";
	case BOOT_SWAP_TYPE_FAIL:
		return "fail";
	default:
		return "unknown";
	}
}

static int dfu_image_info(void *ctx, uint8_t slot, port_image_info_t *out)
{
	ARG_UNUSED(ctx);
	return sts_dfu_image_info(slot, out);
}

/* Erase the reserved trailer page so MCUboot can write the boot magic. */
static int dfu_erase_trailer(void)
{
	int rc;

	k_mutex_lock(&dfu_lock, K_FOREVER);
	rc = flash_area_erase(dfu.fa, (off_t)dfu.trailer_off,
			      (size_t)dfu.erase_gran);
	k_mutex_unlock(&dfu_lock);

	if (rc != 0) {
		LOG_ERR("slot1 trailer erase failed (%d)", rc);
	}
	return rc;
}

static int dfu_mark_pending(void *ctx)
{
	int rc;

	ARG_UNUSED(ctx);

	if (dfu_open() != 0) {
		return PORT_ENOTSUP;
	}

	rc = dfu_erase_trailer();
	if (rc != 0) {
		return rc;
	}

	rc = boot_request_upgrade(BOOT_UPGRADE_TEST);
	if (rc != 0) {
		LOG_ERR("boot_request_upgrade failed (%d)", rc);
		return rc;
	}

	LOG_WRN("slot1 marked pending: next boot runs it as a TEST image and "
		"reverts unless the health gate confirms it");
	return 0;
}

static int dfu_confirm_active(void *ctx)
{
	int rc;

	ARG_UNUSED(ctx);

	if (boot_is_img_confirmed()) {
		return 0;
	}

	rc = boot_write_img_confirmed();
	if (rc != 0) {
		LOG_ERR("boot_write_img_confirmed failed (%d)", rc);
		return rc;
	}

	LOG_INF("running image confirmed");
	return 0;
}

static int dfu_request_revert(void *ctx)
{
	int rc;

	ARG_UNUSED(ctx);

	if (dfu_open() != 0) {
		return PORT_ENOTSUP;
	}

	/*
	 * Two distinct meanings share this command, and both are served by
	 * blanking the staging trailer:
	 *
	 *   a pending swap is cancelled, because MCUboot decides what to do on
	 *   the next boot purely from that trailer;
	 *
	 *   an unconfirmed running image is left unconfirmed, which is already
	 *   an automatic revert on the next reboot. Nothing to do but say so.
	 */
	rc = dfu_erase_trailer();
	if (rc != 0) {
		return rc;
	}

	k_mutex_lock(&dfu_lock, K_FOREVER);
	dfu_stream_reset();
	k_mutex_unlock(&dfu_lock);

	if (!boot_is_img_confirmed()) {
		LOG_WRN("revert requested: the running image is unconfirmed, so "
			"the next reboot returns to the previous one");
	} else {
		LOG_INF("revert requested: any staged image is no longer pending");
	}
	return 0;
}

static void dfu_reboot(void *ctx, int mode)
{
	ARG_UNUSED(ctx);

	switch (mode) {
	case 1:
		/*
		 * MCUboot on this board enters serial recovery from a held GPIO
		 * only (sysbuild/mcuboot.conf: CONFIG_BOOT_SERIAL_ENTRANCE_GPIO
		 * with the `mcuboot-button0` alias on BUTTON_1/PF0, 1 s detect
		 * delay). There is no retention area and no boot-mode flag in
		 * this build, so firmware cannot arm recovery for the next
		 * boot; only the operator can, by holding the button through
		 * reset. Say so loudly and reset normally rather than pretend.
		 */
		LOG_ERR("recovery reboot is not firmware-armable on this build: "
			"hold front-panel BUTTON_1 (PF0) through reset and for "
			"1 s afterwards to enter MCUboot serial recovery");
		break;
	case 2:
		/* MCP "halt-to-test": the port contract defines modes 0 and 1
		 * only, so there is nothing to do but leave the box running. */
		LOG_WRN("halt-to-test requested: not implemented, application "
			"left running");
		return;
	default:
		break;
	}

	LOG_WRN("rebooting on console request");
	/* Give the deferred log backend a moment to push the lines out. */
	k_sleep(K_MSEC(50));
	sys_reboot(SYS_REBOOT_COLD);
}

/* ------------------------------------------------------------------------- */

static const port_image_t dfu_port = {
	.staging_size = dfu_staging_size,
	.staging_erase = dfu_staging_erase,
	.staging_write = dfu_staging_write,
	.staging_read = dfu_staging_read,
	.image_info = dfu_image_info,
	.mark_pending = dfu_mark_pending,
	.confirm_active = dfu_confirm_active,
	.request_revert = dfu_request_revert,
	.reboot = dfu_reboot,
	.ctx = NULL,
};

const port_image_t *sts_dfu_port(void)
{
	(void)dfu_open();
	return &dfu_port;
}

#endif /* CONFIG_STS1000_CONSOLE */
