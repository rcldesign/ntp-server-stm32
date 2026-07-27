/*
 * STS1000 "Meridian" — SPI4 arbitration and the MCP41U83 digipot.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPI4 (PE12 SCK / PE13 MISO / PE14 MOSI) carries three devices on distinct
 * chip-selects (interface ref §9):
 *
 *   cs-gpios[0]  PE11  NOR U62 MX25L25645G   — Zephyr jedec,spi-nor driver
 *   cs-gpios[1]  PD2   digipot U43 MCP41U83  — this file
 *   cs-gpios[2]  PE9   display ST7796        — ui area
 *
 * ---------------------------------------------------------------------------
 * Locking: what the Zephyr SPI driver already guarantees, and what it does not
 * ---------------------------------------------------------------------------
 * Verified against zephyr/drivers/spi/spi_context.h (v4.2.2):
 *
 *   spi_context_lock() takes a per-controller binary semaphore (ctx->lock) at
 *   the top of every transceive and spi_context_release() gives it back at the
 *   bottom, unless SPI_LOCK_ON is set in the caller's spi_config, in which case
 *   the same spi_config keeps ownership until spi_release().
 *
 * So one spi_transceive() call — including a multi-buffer set, which keeps CS
 * asserted throughout — is atomic against every other user of SPI4, whichever
 * driver or thread issues it. That covers the interleaving that matters:
 *
 *   - The NOR driver issues each flash command as a single transceive
 *     (spi_nor_access() builds one tx buffer set), so a digipot or display
 *     access can only land *between* flash commands, never inside one. Between
 *     commands the NOR's CS is high and it ignores the bus; an erase or program
 *     in flight is a state of the NOR die, not of the bus, and is unaffected.
 *   - Conversely, a flash command cannot land inside a digipot transceive.
 *
 * What the driver does NOT give us is atomicity across *several* transceives
 * by one caller. The digipot write-then-verify below is exactly that, so it
 * takes an application mutex. The mutex is scoped to sequences that this file
 * owns; it is deliberately not wrapped around flash or display access, because
 * doing so would add a second lock ordering on top of the driver's without
 * buying anything, and would let a slow LittleFS operation block a pwrseq
 * digipot action for the duration of a 4 KB erase.
 *
 * CPOL/CPHA and speed are per-transfer properties of spi_config, so no
 * reconfiguration handshake between devices is needed — each caller passes its
 * own and the driver applies it while holding the controller lock.
 *
 * ---------------------------------------------------------------------------
 * MCP41U83 command format (DS20007000B §7.0)
 * ---------------------------------------------------------------------------
 * Every access is 16 bits, MSB first, SPI mode 0,0:
 *
 *     bit  15 14 13 12 | 11 10 | 9 8 7 6 5 4 3 2 1 0
 *          A3 A2 A1 A0 | C1 C0 | D9 .. D0
 *
 *   AAAA  register address: 0x0 volatile wiper 0, 0x2 non-volatile wiper 0
 *   CC    00 write, 11 read, 01 increment, 10 decrement
 *   DD..  10-bit data field; the 8-bit part uses D7..D0 and ignores D9/D8
 *
 * On a read the device returns the address/command nibble with bit 1 as the
 * CMDERR flag (1 = command accepted) followed by the data.
 *
 * ARCHITECTURE.md §10.3: code 0 = terminal B = the safe-low VCC_RB. That is
 * what pwrseq writes before RB_PWR_EN, and what the NV wiper is programmed to
 * so a POR never comes up mid-scale.
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "zephyr/platform/platform.h"

LOG_MODULE_REGISTER(sts_spi4, CONFIG_STS1000_LOG_LEVEL);

#define SPI4_NODE DT_NODELABEL(spi4)

/* Digipot chip-select: SPI_DPOT_CS on PD2, cs-gpios index 1. */
#define DIGIPOT_CS_IDX 1

#define MCP41U83_ADDR_WIPER0    0x0U
#define MCP41U83_ADDR_NV_WIPER0 0x2U
#define MCP41U83_CMD_WRITE      0x0U
#define MCP41U83_CMD_READ       0x3U

/** The MCP41U83 is a 10-bit potentiometer: wiper codes are 0..1023. */
#define MCP41U83_CODE_MAX 0x3FFU

/*
 * Read response: the device echoes the address/command in the high bits and
 * returns the 10-bit wiper value in [9:0]. Bit 1 of the echoed command nibble
 * is CMDERR (1 = accepted). Data D9/D8 land in [9:8], so the CMDERR bit is read
 * from the high byte with D9/D8 masked off first.
 */
#define MCP41U83_CMDERR_BIT 0x02U

/** Non-volatile writes need tWC to complete (DS20007000B §1.2, 10 ms max). */
#define MCP41U83_NV_WRITE_MS 12

static const struct device *const spi4_dev = DEVICE_DT_GET(SPI4_NODE);

static const struct gpio_dt_spec digipot_cs =
	GPIO_DT_SPEC_GET_BY_IDX(SPI4_NODE, cs_gpios, DIGIPOT_CS_IDX);

/*
 * Built by hand rather than with SPI_DT_SPEC_GET: the digipot has no
 * devicetree node of its own because no Zephyr driver binds it, and inventing
 * one would need a private binding for a device this file already fully
 * describes. The CS spec still comes from devicetree.
 */
static struct spi_config digipot_cfg;

static K_MUTEX_DEFINE(spi4_seq_mutex);

static bool digipot_ready;

static int digipot_xfer(uint16_t cmd, uint16_t *rsp)
{
	uint8_t tx[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFFU) };
	uint8_t rx[2] = { 0U, 0U };
	const struct spi_buf tx_buf = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf rx_buf = { .buf = rx, .len = sizeof(rx) };
	const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };
	int rc;

	rc = spi_transceive(spi4_dev, &digipot_cfg, &tx_set, &rx_set);
	if (rc != 0) {
		return rc;
	}

	if (rsp != NULL) {
		*rsp = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
	}

	return 0;
}

static uint16_t digipot_cmd(uint8_t addr, uint8_t cmd, uint16_t data)
{
	return (uint16_t)(((uint16_t)(addr & 0x0FU) << 12) |
			  ((uint16_t)(cmd & 0x03U) << 10) | (data & 0x03FFU));
}

static int digipot_read_reg(uint8_t addr, uint16_t *code)
{
	uint16_t rsp = 0;
	int rc;

	rc = digipot_xfer(digipot_cmd(addr, MCP41U83_CMD_READ, 0U), &rsp);
	if (rc != 0) {
		return rc;
	}

	/*
	 * CMDERR must read back set. A part that is absent, held in reset or
	 * wired to a floating MISO returns all-zero or all-one; the CMDERR
	 * check plus the 10-bit readback compare in the callers rejects both,
	 * which matters because this gates whether pwrseq may energise the Rb
	 * rail. D9/D8 share the high byte with the echoed command, so mask
	 * them off before testing CMDERR.
	 */
	if ((((rsp >> 8) & ~0x03U) & MCP41U83_CMDERR_BIT) == 0U) {
		return -EIO;
	}

	*code = (uint16_t)(rsp & MCP41U83_CODE_MAX);
	return 0;
}

int sts_digipot_get(uint16_t *code)
{
	int rc;

	if (code == NULL) {
		return -EINVAL;
	}
	if (!digipot_ready) {
		return -ENODEV;
	}

	(void)k_mutex_lock(&spi4_seq_mutex, K_FOREVER);
	rc = digipot_read_reg(MCP41U83_ADDR_WIPER0, code);
	(void)k_mutex_unlock(&spi4_seq_mutex);

	return rc;
}

static int digipot_write_verify(uint8_t addr, uint16_t code, bool nv)
{
	uint16_t readback = 0;
	int rc;

	if (code > MCP41U83_CODE_MAX) {
		return -EINVAL;
	}

	(void)k_mutex_lock(&spi4_seq_mutex, K_FOREVER);

	rc = digipot_xfer(digipot_cmd(addr, MCP41U83_CMD_WRITE, code), NULL);
	if (rc == 0) {
		if (nv) {
			k_msleep(MCP41U83_NV_WRITE_MS);
		}
		rc = digipot_read_reg(addr, &readback);
	}

	(void)k_mutex_unlock(&spi4_seq_mutex);

	if (rc != 0) {
		LOG_ERR("digipot: %swrite %u failed (%d)", nv ? "NV " : "", code, rc);
		return rc;
	}

	if (readback != code) {
		/*
		 * A wiper that did not take the commanded code is the failure
		 * mode that can over-volt the FE-5680A, so it is an error, not
		 * a warning: pwrseq must not proceed to RB_PWR_EN on it.
		 */
		LOG_ERR("digipot: %swrote %u, read back %u", nv ? "NV " : "", code,
			readback);
		return -EIO;
	}

	return 0;
}

int sts_digipot_set(uint16_t code)
{
	if (!digipot_ready) {
		return -ENODEV;
	}

	return digipot_write_verify(MCP41U83_ADDR_WIPER0, code, false);
}

int sts_digipot_set_nv(uint16_t code)
{
	int rc;

	if (!digipot_ready) {
		return -ENODEV;
	}

	rc = digipot_write_verify(MCP41U83_ADDR_NV_WIPER0, code, true);
	if (rc == 0) {
		LOG_INF("digipot: NV wiper programmed to %u (POR value)", code);
	}

	return rc;
}

int sts_spi4_init(void)
{
	if (!device_is_ready(spi4_dev)) {
		LOG_ERR("SPI4 not ready");
		return -ENODEV;
	}
	if (!gpio_is_ready_dt(&digipot_cs)) {
		LOG_ERR("SPI_DPOT_CS not ready");
		return -ENODEV;
	}

	digipot_cfg.frequency = CONFIG_STS1000_DIGIPOT_SPI_HZ;
	/*
	 * Mode 0,0 per DS20007000B: CPOL and CPHA both 0, which is the absence
	 * of SPI_MODE_CPOL and SPI_MODE_CPHA rather than a flag to set. 8-bit
	 * words, MSB first; CS polarity comes from the devicetree spec.
	 */
	digipot_cfg.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
	digipot_cfg.slave = DIGIPOT_CS_IDX;
	digipot_cfg.cs.gpio = digipot_cs;
	digipot_cfg.cs.delay = 0U;

	digipot_ready = true;

	return 0;
}
