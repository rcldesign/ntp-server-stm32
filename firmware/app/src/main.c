/*
 * STS1000 "Meridian" — bring-up sequencer entry.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Implements the early part of the power-up order of operations in
 * docs/sts1000_firmware_hardware_interface.md §2. The fail-safe principle of
 * that section is that every gated load and every resettable device sits in its
 * safe/off state at reset, held there by an external pull, and firmware
 * releases them deliberately and in order.
 *
 * Wave 1a covers Stage 2 (release held-in-reset digital devices) and Stage 4
 * (Ethernet PHY) only. Stages 3 and 5-9 — rail verification via the nine
 * INA228 monitors, GNSS, panel, OCXO discipline, the guarded rubidium sequence
 * and the watchdog/alarm arming — are owned by core/pwrseq (ARCHITECTURE.md §5)
 * and land in later waves. Nothing here enables a gated load.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zephyr/app_version.h>

#if defined(CONFIG_MCUBOOT_IMG_MANAGER)
#include <zephyr/dfu/mcuboot.h>
#endif

#include "zephyr/threads/threads.h"

LOG_MODULE_REGISTER(sts1000_main, CONFIG_STS1000_LOG_LEVEL);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

/* Stage 2: SPI-NOR U62 reset. Boots asserted via R205 10k pull-down. */
static const struct gpio_dt_spec nor_rst =
	GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, nor_rst_gpios);

/* Stage 4: LAN8742AI reset. Boots asserted via R35 10k pull-down. */
static const struct gpio_dt_spec lan_rst =
	GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, lan_rst_gpios);

#define NOR_FLASH_NODE DT_NODELABEL(nor_flash)

/* ------------------------------------------------------------------------ */
/* Stage 4 — Ethernet PHY reset                                             */
/* ------------------------------------------------------------------------ */

/*
 * LAN_RST_N has to be released before the MDIO, PHY and Ethernet drivers
 * probe, and those run at POST_KERNEL priorities 60/70/80 — i.e. before
 * main(). A PHY still held in reset reads back an all-ones ID and the
 * interface never links, so the release is done from an init hook ordered
 * ahead of them (CONFIG_STS1000_EARLY_RESET_INIT_PRIORITY, default 45) rather
 * than from the sequencer body below.
 *
 * TODO(wave-2): core/pwrseq must gate this release on Stage 3 rail
 * verification, which means &mac/&mdio have to become `zephyr,deferred-init`
 * so the sequencer can call device_init() at the right moment. This hook is
 * deleted at that point.
 *
 * The SPI-NOR needs no such hook: its node is `zephyr,deferred-init`, so no
 * bus traffic can reach it before Stage 2 runs in main().
 */
static int sts1000_early_reset_release(void)
{
	int rc;

	if (!gpio_is_ready_dt(&lan_rst)) {
		LOG_ERR("LAN_RST_N: GPIO port %s not ready", lan_rst.port->name);
		return -ENODEV;
	}

	/* Drive the asserted state explicitly; do not rely on the pull-down. */
	rc = gpio_pin_configure_dt(&lan_rst, GPIO_OUTPUT_ACTIVE);
	if (rc != 0) {
		LOG_ERR("LAN_RST_N: configure failed (%d)", rc);
		return rc;
	}

	k_busy_wait(CONFIG_STS1000_PHY_RESET_ASSERT_US);

	rc = gpio_pin_set_dt(&lan_rst, 0);
	if (rc != 0) {
		LOG_ERR("LAN_RST_N: release failed (%d)", rc);
		return rc;
	}

	k_busy_wait(CONFIG_STS1000_PHY_RESET_RECOVERY_US);

	return 0;
}

SYS_INIT(sts1000_early_reset_release, POST_KERNEL,
	 CONFIG_STS1000_EARLY_RESET_INIT_PRIORITY);

/* ------------------------------------------------------------------------ */
/* Boot banner                                                              */
/* ------------------------------------------------------------------------ */

static void log_boot_banner(void)
{
	uint32_t cause = 0U;
	int rc;

	LOG_INF("STS1000 \"Meridian\" %s (build %s) on %s", APP_VERSION_EXTENDED_STRING,
		STRINGIFY(BUILD_VERSION), CONFIG_BOARD_TARGET);
	LOG_INF("SYSCLK %u Hz from HSE-bypass PH0 (clock mux -> OCXO or Rb)",
		(unsigned int)sys_clock_hw_cycles_per_sec());

	rc = hwinfo_get_reset_cause(&cause);
	if (rc == 0) {
		LOG_INF("reset cause: 0x%08x", cause);
		/*
		 * Clear so the next boot reports only its own cause; the STM32
		 * RCC reset flags are sticky across resets otherwise.
		 */
		(void)hwinfo_clear_reset_cause();
	} else if (rc != -ENOSYS) {
		LOG_WRN("reset cause unavailable (%d)", rc);
	}

#if defined(CONFIG_MCUBOOT_IMG_MANAGER)
	if (!boot_is_img_confirmed()) {
		/*
		 * Running a TEST image. ARCHITECTURE.md §3 requires the
		 * application to call boot_write_img_confirmed() only after its
		 * own health checks pass; until that wave exists, MCUboot will
		 * correctly revert on the next boot.
		 * TODO(wave-2): confirm from the supervisor once timing,
		 * network and housekeeping liveness all pass.
		 */
		LOG_WRN("running an unconfirmed image: MCUboot will revert on reboot");
	}
#endif
}

/* ------------------------------------------------------------------------ */
/* Stage 2 — release held-in-reset digital devices                          */
/* ------------------------------------------------------------------------ */

static int stage2_release_resets(void)
{
	const struct device *nor = DEVICE_DT_GET(NOR_FLASH_NODE);
	int rc;

	LOG_INF("stage 2: releasing held-in-reset devices");

	if (!gpio_is_ready_dt(&nor_rst)) {
		LOG_ERR("NOR_RST_N: GPIO port %s not ready", nor_rst.port->name);
		return -ENODEV;
	}

	/*
	 * GPIO_OUTPUT_INACTIVE on an active-low line drives the pin HIGH, i.e.
	 * releases the reset. Interface ref §2 Stage 2.1: this must happen
	 * before any SPI4 access to the NOR (ARCHITECTURE.md §10.4).
	 */
	rc = gpio_pin_configure_dt(&nor_rst, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		LOG_ERR("NOR_RST_N: release failed (%d)", rc);
		return rc;
	}

	k_busy_wait(CONFIG_STS1000_NOR_RESET_RECOVERY_US);

	/*
	 * The NOR node is `zephyr,deferred-init`, so this is its first and only
	 * initialisation — and the first traffic on SPI4. A missing NOR is not
	 * fatal: ARCHITECTURE.md §3 states the board boots without it, and the
	 * consequence is only that /lfs, the calibration record and the log
	 * spool are unavailable.
	 */
	rc = device_init(nor);
	if (rc != 0) {
		LOG_ERR("SPI-NOR %s init failed (%d): /lfs, calibration and log "
			"spool unavailable",
			nor->name, rc);
	} else {
		LOG_INF("SPI-NOR %s ready", nor->name);
	}

	/*
	 * DISP_RST (PA10) is deliberately left asserted. Interface ref §2
	 * Stage 2.3 releases the display controller only when the UI is wanted,
	 * and the display rail (DISP_EN/PC11) is still off; the panel subsystem
	 * is a later wave.
	 */

	return 0;
}

/* ------------------------------------------------------------------------ */
/* Stage 4 — network                                                        */
/* ------------------------------------------------------------------------ */

static void stage4_report_phy(void)
{
	const struct device *mac = DEVICE_DT_GET(DT_NODELABEL(mac));
	const struct device *phy = DEVICE_DT_GET(DT_NODELABEL(eth_phy));

	LOG_INF("stage 4: LAN_RST_N released (%u us assert, %u us recovery)",
		(unsigned int)CONFIG_STS1000_PHY_RESET_ASSERT_US,
		(unsigned int)CONFIG_STS1000_PHY_RESET_RECOVERY_US);

	LOG_INF("stage 4: MAC %s %s, PHY %s %s (addr 0)", mac->name,
		device_is_ready(mac) ? "ready" : "NOT READY", phy->name,
		device_is_ready(phy) ? "ready" : "NOT READY");

	/*
	 * TODO(wave-2): the net_mgmt thread owns DHCPv4 start, link-event
	 * handling and the MDIO link poll (there is no PHY interrupt line on
	 * this board — interface ref §1.1, PA2).
	 */
}

/* ------------------------------------------------------------------------ */

int main(void)
{
	size_t threads;
	int rc;

	log_boot_banner();

	rc = stage2_release_resets();
	if (rc != 0) {
		LOG_ERR("stage 2 failed (%d)", rc);
		return rc;
	}

	/*
	 * TODO(wave-2): stage 3 — verify 3V3 (INA228 0x43 + PG4), 3V3_STM
	 * (0x41), the PoE input (0x40 + PG7), then apply the stored per-board
	 * shunt_cal[] trim to all nine monitors BEFORE any current reading is
	 * used for a budget or alarm decision (interface ref §2, §4.2).
	 */

	stage4_report_phy();

	threads = sts1000_threads_start();
	LOG_INF("bring-up complete: %zu application threads started", threads);

	return 0;
}
