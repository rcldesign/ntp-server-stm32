/*
 * STS1000 "Meridian" — how large an image the staging slot may actually hold.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/console/, and deliberately free of every Zephyr and
 * MCUboot dependency so tests/host can compile it and check the arithmetic
 * against an independent model of MCUboot's swap.
 *
 * Why this is not "slot size minus one sector"
 * --------------------------------------------
 * port_image_t::staging_size() is a hard promise: core/mcp accepts any FW_BEGIN
 * up to that number and rejects everything above it, and NOTHING downstream
 * re-checks. `MCUBOOT_SERIAL_IMG_GRP_SLOT_INFO` and `MCUBOOT_DATA_SHARING` are
 * both off in this build, so the bootloader never tells the application its real
 * ceiling, and boot_slots_compatible() passes regardless because both slots have
 * the same sector count. An over-advertised size therefore produces the worst
 * possible failure mode: the image uploads, verifies its SHA-256, is marked
 * pending, and then swap_move() silently declines the upgrade on every
 * subsequent boot — into a bootloader built with CONFIG_MCUBOOT_LOG_LEVEL_OFF
 * and no console. Not a brick (the old image keeps running) but an update that
 * never applies and produces no diagnostic anywhere.
 *
 * MCUboot's actual arithmetic (bootloader/mcuboot, swap-using-move)
 * ----------------------------------------------------------------
 *   bootutil_misc.c:
 *     boot_status_sz(w)   = BOOT_STATUS_MAX_ENTRIES * BOOT_STATUS_STATE_COUNT * w
 *     boot_trailer_sz(w)  = boot_status_sz(w) + BOOT_MAX_ALIGN * 4
 *                                            + BOOT_MAGIC_ALIGN_SIZE
 *   bootutil_priv.h (MCUBOOT_SWAP_USING_MOVE):
 *     BOOT_STATUS_STATE_COUNT = BOOT_STATUS_MOVE_STATE_COUNT (1)
 *                             + BOOT_STATUS_SWAP_STATE_COUNT (2)   = 3
 *     BOOT_STATUS_MAX_ENTRIES = BOOT_MAX_IMG_SECTORS
 *   bootutil_public.h:
 *     BOOT_MAX_ALIGN        = flash write-block size when > 8      (16 on H5)
 *     BOOT_MAGIC_ALIGN_SIZE = ALIGN_UP(BOOT_MAGIC_SZ = 16, BOOT_MAX_ALIGN)
 *   swap_move.c swap_run():
 *     first_trailer_idx starts at n_sectors - 1 and is decremented until the
 *     sectors from it to the top of the slot cover trailer_sz; the upgrade is
 *     abandoned (swap_type = NONE) when
 *         find_last_idx(copy_size) >= first_trailer_idx,
 *     and find_last_idx() is ceil(copy_size / sector_sz), minimum 1.
 *
 * Solving that for the largest accepted size gives
 *
 *     trailer_sectors = ceil(trailer_sz / sector_sz)          (at least 1)
 *     usable_sectors  = n_sectors - trailer_sectors - 1       (the -1 is the
 *                                                             free sector the
 *                                                             move algorithm
 *                                                             shifts the image
 *                                                             up into)
 *     usable          = usable_sectors * sector_sz
 *
 * For this board — slot1 = 896 KiB, 8 KiB sectors, 16 B write block, so
 * n_sectors = 112 and BOOT_MAX_IMG_SECTORS = 112 (derived by
 * BOOT_MAX_IMG_SECTORS_AUTO from the partition table) —
 *
 *     trailer_sz      = 112*3*16 + 16*4 + 16 = 5376 + 64 + 16 = 5456 B
 *     trailer_sectors = ceil(5456 / 8192)                     = 1
 *     usable_sectors  = 112 - 1 - 1                           = 110
 *     usable          = 110 * 8192                            = 901120 B
 *
 * which is also what MCUboot's own app_max_sectors() reports (n_sectors - 2 for
 * this geometry). The previous "fa_size - erase_gran" = 909312 over-advertised by
 * exactly the one move sector, so images of 901121..909312 B were accepted and
 * then silently never applied.
 */

#ifndef STS1000_ZEPHYR_CONSOLE_STS_STAGE_GEOM_H_
#define STS1000_ZEPHYR_CONSOLE_STS_STAGE_GEOM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** MCUboot BOOT_STATUS_STATE_COUNT for swap-using-move: MOVE(1) + SWAP(2). */
#define STS_MCUBOOT_STATUS_STATE_COUNT 3U

/** MCUboot BOOT_MAGIC_SZ. */
#define STS_MCUBOOT_MAGIC_SZ 16U

/** MCUboot's BOOT_MAX_ALIGN: the flash write block, floored at 8. */
static inline uint32_t sts_mcuboot_max_align(uint32_t write_block)
{
	return (write_block > 8U) ? write_block : 8U;
}

/**
 * MCUboot's boot_trailer_sz() for a swap-using-move slot, in bytes.
 *
 * @param max_img_sectors  BOOT_MAX_IMG_SECTORS as the bootloader was built with.
 *                         With BOOT_MAX_IMG_SECTORS_AUTO that is the slot's own
 *                         sector count; passing a larger value only over-reserves.
 * @param write_block      Flash write-block size (flash_area_align()).
 */
static inline uint32_t sts_mcuboot_trailer_sz(uint32_t max_img_sectors,
					      uint32_t write_block)
{
	uint32_t align = sts_mcuboot_max_align(write_block);
	uint32_t magic = ((STS_MCUBOOT_MAGIC_SZ + align - 1U) / align) * align;

	return (max_img_sectors * STS_MCUBOOT_STATUS_STATE_COUNT * write_block) +
	       (align * 4U) + magic;
}

/**
 * Largest IMAGE the staging slot may advertise, in bytes.
 *
 * @param slot_size    Physical slot size (flash_area::fa_size).
 * @param erase_gran   Erase sector size, in bytes. Must divide @p slot_size for
 *                     the result to be meaningful; a remainder is discarded,
 *                     which is the safe direction.
 * @param write_block  Flash write-block size, in bytes.
 *
 * @return The ceiling, or 0 when the slot cannot hold an image plus the trailer
 *         plus the move sector at all. 0 makes core/mcp answer MCP_ERR_NOSPC to
 *         every FW_BEGIN, which is the correct behaviour for a slot that geometry
 *         says can never be swapped.
 */
static inline uint32_t sts_staging_usable(uint32_t slot_size,
					  uint32_t erase_gran,
					  uint32_t write_block)
{
	uint32_t sectors;
	uint32_t trailer_sz;
	uint32_t trailer_sectors;

	if ((erase_gran == 0U) || (write_block == 0U) ||
	    (slot_size < erase_gran)) {
		return 0U;
	}

	sectors = slot_size / erase_gran;

	/* BOOT_MAX_IMG_SECTORS_AUTO derives the bound from the partition table,
	 * so the slot's own sector count is what the bootloader was built with. */
	trailer_sz = sts_mcuboot_trailer_sz(sectors, write_block);
	trailer_sectors = (trailer_sz + erase_gran - 1U) / erase_gran;
	if (trailer_sectors == 0U) {
		trailer_sectors = 1U;
	}

	/* trailer_sectors + 1: the reserved trailer region, plus the one free
	 * sector at the top of the slot that swap-using-move shifts into. */
	if (sectors <= (trailer_sectors + 1U)) {
		return 0U;
	}

	return (sectors - trailer_sectors - 1U) * erase_gran;
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_CONSOLE_STS_STAGE_GEOM_H_ */
