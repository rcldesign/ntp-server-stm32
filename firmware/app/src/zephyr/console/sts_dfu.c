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
 * STM32H5 internal flash programs in 16-byte quad-words. This port reports that
 * write-block size to core/mcp via mcp_wiring_t::dfu_write_block, and core then
 * guarantees every non-final FW_DATA chunk is a multiple of it. So the port is
 * a write-block aligner that writes the aligned bulk straight through to flash
 * (the H5 driver tolerates an unaligned source, so no bounce copy) and buffers
 * only the FINAL short block, padding it to the write-block size with the
 * erased value when core/mcp sets `flush`. The stream is then *sealed*: any
 * further write without an intervening erase would have to re-program a
 * partially-written block, which flash cannot do, so it is refused rather than
 * silently corrupting the image.
 *
 * The write frontier is strictly sequential because core/mcp guarantees it —
 * a FW_DATA below the frontier is absorbed as a retransmit without reaching
 * this file, and one above it is refused with the expected offset. A
 * non-sequential write arriving here is a contract violation and returns
 * -EINVAL rather than being papered over.
 *
 * Trailer handling and the size ceiling
 * -------------------------------------
 * Two distinct regions at the top of the staging slot are excluded from
 * staging_size():
 *
 *   the MCUboot image trailer, because boot_request_upgrade() writes the boot
 *   magic into it with a plain flash write and no erase of its own, so the
 *   region has to be blank first. It is erased here immediately before the
 *   upgrade request, and erasing it is also exactly what cancels a pending swap,
 *   which is how request_revert() is implemented;
 *
 *   one further erase sector, because swap-using-move begins by shifting the
 *   primary image UP by one sector and needs somewhere to shift it to. Omitting
 *   it over-advertised the slot by exactly 8 KiB: an image in that window
 *   uploaded, verified, was marked pending, and was then silently declined by
 *   swap_move() on every subsequent boot, inside a bootloader with logging
 *   compiled out and no console.
 *
 * sts_stage_geom.h derives the resulting ceiling from MCUboot's own arithmetic
 * and carries the full derivation; for this board it is 901120 B of a 917504 B
 * slot. dfu.trailer_off is still the start of the last sector, because that is
 * the page dfu_erase_trailer() has to blank.
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
#include "console/sts_rollback.h"
#include "console/sts_stage_geom.h"
#include "port/port.h"

LOG_MODULE_REGISTER(sts_dfu, CONFIG_STS1000_LOG_LEVEL);

#define SLOT0_ID FIXED_PARTITION_ID(slot0_partition)
#define SLOT1_ID FIXED_PARTITION_ID(slot1_partition)

BUILD_ASSERT(FIXED_PARTITION_EXISTS(slot0_partition) &&
		     FIXED_PARTITION_EXISTS(slot1_partition),
	     "the A/B slot partitions are missing from the board devicetree");

/** Upper bound on the flash write-block this aligner buffers. STM32H5 internal
 *  flash is 16 B (a 128-bit quad-word); the headroom covers other NOR/flash
 *  parts without making the buffer meaningfully larger. Checked at open. */
#define DFU_ALIGN_MAX 32U

/** Fallback erase granularity if the flash driver cannot describe the slot. */
#define DFU_ERASE_GRAN_FALLBACK 8192U

struct dfu_ctx {
	const struct flash_area *fa; /* slot1, opened once at init */
	uint32_t erase_gran;         /* flash erase page, bytes */
	uint32_t usable;             /* staging bytes offered to core/mcp */
	uint32_t trailer_off;        /* start of the reserved trailer page */
	uint32_t align;              /* flash write-block size (16 on STM32H5) */
	uint8_t  erased_val;         /* what an erased byte reads back as */

	/*
	 * Write-block aligner state. `flushed` is the flash write frontier: a
	 * multiple of `align` (until the final padded write). `tail_n` (< align)
	 * is the sub-block remainder held in `tail`, not yet committed. The
	 * accepted frontier — the next offset core may write — is
	 * `flushed + tail_n`.
	 */
	uint32_t flushed;
	uint32_t tail_n;
	bool     sealed;             /* a padded flush closed the stream */
	bool     ready;
	bool     open_failed;        /* do not retry a hard geometry failure */

	uint8_t tail[DFU_ALIGN_MAX] __aligned(4);
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
	if ((dfu.align == 0U) || (dfu.align > DFU_ALIGN_MAX)) {
		LOG_ERR("flash write-block %u B is 0 or exceeds the %u B aligner "
			"buffer",
			(unsigned int)dfu.align, (unsigned int)DFU_ALIGN_MAX);
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

	/*
	 * The MCUboot swap-using-move ceiling, not "everything but the last
	 * page": the trailer AND one free move sector are both excluded. See
	 * sts_stage_geom.h for the derivation against MCUboot's own arithmetic.
	 */
	dfu.usable = sts_staging_usable((uint32_t)dfu.fa->fa_size, dfu.erase_gran,
					dfu.align);
	if (dfu.usable == 0U) {
		LOG_ERR("slot1 is only %u B with %u B pages; no room for an image "
			"plus the MCUboot trailer and move sector",
			(unsigned int)dfu.fa->fa_size,
			(unsigned int)dfu.erase_gran);
		flash_area_close(dfu.fa);
		dfu.fa = NULL;
		dfu.open_failed = true;
		rc = -EINVAL;
		goto out;
	}

	/* The page dfu_erase_trailer() blanks so boot_request_upgrade() can write
	 * the boot magic. Always the last one, whatever the size ceiling is. */
	dfu.trailer_off = (uint32_t)dfu.fa->fa_size - dfu.erase_gran;
	dfu.ready = true;

	LOG_INF("DFU staging: slot1 %u B, %u B usable (trailer + move sector "
		"reserved), %u B pages, %u B writes",
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

uint32_t sts_dfu_write_block(void)
{
	(void)dfu_open();
	/* 16 on STM32H5. Reported to core/mcp as mcp_wiring_t::dfu_write_block
	 * so it keeps every non-final FW_DATA chunk a multiple of this — the
	 * guarantee that lets staging_write buffer only the final short block. */
	return (dfu.align != 0U) ? dfu.align : 16U;
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
	dfu.flushed = 0U;
	dfu.tail_n = 0U;
	dfu.sealed = false;
}

/* Accepted frontier: the next offset core may write. Caller holds dfu_lock. */
static uint32_t dfu_accepted(void)
{
	return dfu.flushed + dfu.tail_n;
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
	 * that reaches back into the accepted region means the tool restarted
	 * the transfer. Drop whatever the previous attempt left behind.
	 */
	if (off < dfu_accepted()) {
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

/* Caller holds dfu_lock. Programs `n` bytes (a multiple of align) at the flash
 * frontier from `src`, advancing the frontier. */
static int dfu_program(const uint8_t *src, size_t n)
{
	int rc = flash_area_write(dfu.fa, (off_t)dfu.flushed, src, n);

	if (rc != 0) {
		LOG_ERR("slot1 write at 0x%x (%zu B) failed (%d)",
			(unsigned int)dfu.flushed, n, rc);
		return rc;
	}
	dfu.flushed += (uint32_t)n;
	return 0;
}

/*
 * Write-block aligner (port_image.h: "impl handles write-block alignment
 * buffering for the FINAL short block when flush=true").
 *
 * core/mcp reports the flash write-block via mcp_wiring_t::dfu_write_block and
 * then guarantees every non-final FW_DATA chunk is a multiple of it, so in
 * normal operation the only thing ever buffered is the last short block. The
 * top-up branch below still handles a sub-block mid-stream remainder, so the
 * port stays correct even if that guarantee is ever relaxed — it is just never
 * exercised in the aligned-chunk hot path.
 *
 * The STM32H5 flash driver reads the source with UNALIGNED_GET, so the bulk is
 * written straight from core's payload buffer with no bounce copy; only offset
 * and length must be align-multiples, which they are by construction.
 */
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
	 * Strictly sequential: the accepted frontier (flushed + tail_n) is the
	 * only offset accepted. A fresh or just-erased stream has it at 0, which
	 * is exactly where core/mcp starts after FW_BEGIN.
	 */
	if (dfu.sealed || (off != dfu_accepted())) {
		LOG_ERR("non-sequential staging write: got 0x%x, expected 0x%x%s",
			(unsigned int)off, (unsigned int)dfu_accepted(),
			dfu.sealed ? " (stream sealed)" : "");
		k_mutex_unlock(&dfu_lock);
		return PORT_EINVAL;
	}

	/* 1. Top up a pending sub-block tail to a full write block. */
	if ((dfu.tail_n > 0U) && (len > 0U)) {
		size_t n = MIN((size_t)(dfu.align - dfu.tail_n), len);

		memcpy(&dfu.tail[dfu.tail_n], data, n);
		dfu.tail_n += (uint32_t)n;
		data += n;
		len -= n;

		if (dfu.tail_n == dfu.align) {
			rc = dfu_program(dfu.tail, dfu.align);
			if (rc != 0) {
				goto out;
			}
			dfu.tail_n = 0U;
		}
	}

	/* 2. Write the write-block-aligned bulk directly from core's buffer. */
	if (len >= dfu.align) {
		size_t bulk = (len / dfu.align) * (size_t)dfu.align;

		rc = dfu_program(data, bulk);
		if (rc != 0) {
			goto out;
		}
		data += bulk;
		len -= bulk;
	}

	/* 3. Buffer the sub-block remainder. Only the FINAL chunk leaves one
	 *    here when core honours the alignment guarantee. */
	if (len > 0U) {
		memcpy(&dfu.tail[dfu.tail_n], data, len);
		dfu.tail_n += (uint32_t)len;
	}

	/* 4. Flush: pad the buffered tail to a full write block and commit it,
	 *    then advance the frontier by the real byte count so the accepted
	 *    frontier equals the image size, not the padded block boundary. */
	if (flush) {
		if (dfu.tail_n > 0U) {
			uint32_t real = dfu.tail_n;

			memset(&dfu.tail[real], dfu.erased_val, dfu.align - real);
			rc = flash_area_write(dfu.fa, (off_t)dfu.flushed,
					      dfu.tail, dfu.align);
			if (rc != 0) {
				LOG_ERR("slot1 final write at 0x%x failed (%d)",
					(unsigned int)dfu.flushed, rc);
				goto out;
			}
			dfu.flushed += real;
			dfu.tail_n = 0U;
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
	 * Overlay the buffered sub-block tail. core/mcp only reads back after a
	 * flush (tail_n == 0 then), so this normally does nothing — but a read
	 * that returned erased flash for bytes the caller had already handed us
	 * would turn a correct image into a hash mismatch, and that failure is
	 * far too expensive to leave to convention.
	 */
	if ((rc == 0) && (dfu.tail_n > 0U)) {
		uint32_t bs = dfu.flushed;
		uint32_t be = dfu.flushed + dfu.tail_n;
		uint32_t rs = off;
		uint32_t re = off + (uint32_t)len;

		if ((rs < be) && (re > bs)) {
			uint32_t s = MAX(rs, bs);
			uint32_t e = MIN(re, be);

			memcpy(&data[s - rs], &dfu.tail[s - bs], (size_t)(e - s));
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

	/*
	 * Refuse here what the bootloader would refuse silently.
	 *
	 * MCUboot re-checks the security counter at the next boot, so skipping
	 * this costs nothing in security — but its logging is compiled out, so
	 * the operator's experience of a rolled-back image is "upload OK",
	 * "staged, pending", reboot, and the same version still running, with
	 * nothing anywhere saying why. Checking before the trailer is written
	 * turns that into an error at the moment the decision is made.
	 *
	 * Deliberately before boot_request_upgrade() and after the erase: the
	 * erase is idempotent, whereas a trailer already marked pending would
	 * have to be un-marked.
	 */
	rc = sts_rollback_check_staged();
	if (rc != 0) {
		return (rc == -EPERM) ? PORT_ENOTSUP : rc;
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
		 * reset. Say so loudly and reset normally rather than pretend:
		 * a cold reset while the button is already held still enters
		 * recovery, so this is best-effort rather than a lie.
		 */
		LOG_ERR("recovery reboot is not firmware-armable on this build: "
			"hold front-panel BUTTON_1 (PF0) through reset and for "
			"1 s afterwards to enter MCUboot serial recovery");
		break;
	case 2:
		/*
		 * halt-to-test (port_image.h): reboot into a staged TEST image
		 * without confirming it. FW_END already called mark_pending,
		 * i.e. boot_request_upgrade(TEST), so a plain cold reset makes
		 * MCUboot swap in the staged image and run it unconfirmed —
		 * which is exactly this. If nothing is staged the swap type is
		 * "none" and it is an ordinary reboot; warn so the operator is
		 * not surprised.
		 */
		if (mcuboot_swap_type() != BOOT_SWAP_TYPE_TEST) {
			LOG_WRN("halt-to-test with no TEST image staged "
				"(swap=%s): rebooting normally",
				sts_dfu_swap_type_name());
		} else {
			LOG_WRN("halt-to-test: rebooting into the staged image "
				"unconfirmed");
		}
		break;
	default:
		break;
	}

	LOG_WRN("rebooting on console request (mode %d)", mode);
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
