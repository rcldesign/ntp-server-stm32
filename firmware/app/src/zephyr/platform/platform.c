/*
 * STS1000 "Meridian" — platform-area bring-up.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * sts_platform_init() runs the stages of
 * docs/sts1000_firmware_hardware_interface.md §2 that belong to the platform:
 * 2 (release held-in-reset devices), 3 (verify rails, calibrate the INA228s),
 * 7 (start the discipline loop) and the parts of 9 that do not depend on the
 * other areas. Stages 4, 6 and 8 are the net area, the ui area and pwrseq.
 *
 * The fail-safe principle of §2 is that every gated load and every resettable
 * device sits in its safe/off state at reset, held there by an external pull.
 * Nothing here enables a load whose rail has not been verified, and nothing
 * un-gates the rubidium — that is stage 8 and it is guarded.
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zephyr/app_version.h>

#include "zephyr/platform/platform.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_platform, CONFIG_STS1000_LOG_LEVEL);

/* Stage 2: SPI-NOR U62 reset. Boots asserted via R205 10k pull-down. */
static const struct gpio_dt_spec nor_rst = STS_USER_GPIO(nor_rst_gpios);

/* Stage 4: LAN8742AI reset. Boots asserted via R35 10k pull-down. */
static const struct gpio_dt_spec lan_rst = STS_USER_GPIO(lan_rst_gpios);

#define NOR_FLASH_NODE DT_NODELABEL(nor_flash)

/* ------------------------------------------------------------------------ */
/* Early: Ethernet PHY reset release                                        */
/* ------------------------------------------------------------------------ */

/*
 * LAN_RST_N has to be released before the MDIO, PHY and Ethernet drivers
 * probe, and those run at POST_KERNEL priorities 60/70/80 — i.e. before
 * main(). A PHY still held in reset reads back an all-ones ID and the
 * interface never links, so the release is done from an init hook ordered
 * ahead of them rather than from the stage body below.
 *
 * This inverts the §2 ordering (stage 4 before stage 3's rail verification),
 * which is acceptable only because the PHY's rail is 3V3 — the same always-on
 * rail the MCU itself is running from, so if it were not good this code would
 * not be executing. It is called out rather than hidden because the same
 * shortcut would be wrong for any gated rail.
 *
 * TODO(wave-3b): make &mac/&mdio `zephyr,deferred-init` so the net area can
 * call device_init() after stage 3, and delete this hook.
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
		/* Clear so the next boot reports only its own cause; the STM32
		 * RCC reset flags are sticky otherwise. */
		(void)hwinfo_clear_reset_cause();
	} else if (rc != -ENOSYS) {
		LOG_WRN("reset cause unavailable (%d)", rc);
	}

	if (sts_update_pending_confirm()) {
		LOG_WRN("running an unconfirmed image: the supervisor will confirm "
			"it once the clock is locked and no alarm is active");
	}
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
		(void)sts_alarm_set(FAULT_ALARM_NOR_FAULT, true);
	} else {
		LOG_INF("SPI-NOR %s ready", nor->name);
	}

	/*
	 * DISP_RST (PA10) is deliberately left asserted. Interface ref §2
	 * Stage 2.3 releases the display controller only when the UI is wanted,
	 * and the display rail (DISP_EN/PC11) is still off; that is the ui
	 * area's call, not the platform's.
	 */

	return 0;
}

/* ------------------------------------------------------------------------ */
/* Stage 3 — verify main rails via telemetry + PG lines                     */
/* ------------------------------------------------------------------------ */

/*
 * The rails that must be good before anything else is allowed to proceed.
 * These are the ones the MCU, the housekeeping bus and the timing chain are
 * already running from; a monitor that disagrees means the telemetry is wrong,
 * not that the board is dead, and that is worth saying loudly.
 */
static const struct {
	ina228_rail_t rail;
	fault_sig_t pg;
	int32_t min_mv;
	int32_t max_mv;
} stage3_rails[] = {
	{ INA228_RAIL_3V3_MAIN, FAULT_SIG_PG_3V3_PSU, 3100, 3500 },
	{ INA228_RAIL_3V3_STM, FAULT_SIG_PG_3V3_PSU, 3100, 3500 },
	{ INA228_RAIL_POE, FAULT_SIG_PG_POE, 37000, 57000 },
};

static int stage3_verify_rails(void)
{
	sts_hk_snapshot_t hk;
	int bad = 0;
	int rc;

	LOG_INF("stage 3: verifying rails and calibrating the INA228 monitors");

	/*
	 * Configuration first, reading second. A monitor whose SHUNT_CAL has
	 * not been applied reports current against POR calibration, and
	 * interface ref §2 stage 3 is explicit that no current reading may be
	 * used for a budget or alarm decision before the trim is in place.
	 */
	rc = sts_hk_ina_configure_all();
	if (rc != 0) {
		LOG_ERR("stage 3: not all INA228 monitors responded");
		(void)sts_alarm_set(FAULT_ALARM_I2C_WEDGE, true);
	}

	/*
	 * Read synchronously, on this thread. sts_hk_request_ina() queues the read
	 * for the housekeeping thread, which does not exist yet at this point in
	 * bring-up — so every reading came back valid == false and this function
	 * logged three errors and returned -EIO on every single boot, which main()
	 * reported as "running degraded". The rails were fine; the check was asking
	 * an empty cache.
	 *
	 * A settle wait first: the INA228s were configured a few microseconds ago
	 * and their averaged continuous conversions need a cycle to produce a real
	 * value rather than the POR register contents.
	 */
	k_msleep(CONFIG_STS1000_INA228_SETTLE_MS);

	for (size_t i = 0; i < ARRAY_SIZE(stage3_rails); i++) {
		(void)sts_hk_read_ina_now((uint8_t)stage3_rails[i].rail);
	}

	if (sts_hk_read(&hk) != 0) {
		return -EIO;
	}

	for (size_t i = 0; i < ARRAY_SIZE(stage3_rails); i++) {
		const ina228_rail_info_t *info = &ina228_rail_tbl[stage3_rails[i].rail];
		const sts_ina_reading_t *r = &hk.ina[stage3_rails[i].rail];
		int32_t mv;

		if (!r->valid) {
			LOG_ERR("stage 3: %s (%s) unreadable", info->name,
				info->designator);
			bad++;
			continue;
		}

		mv = r->bus_uv / 1000;
		if (mv < stage3_rails[i].min_mv || mv > stage3_rails[i].max_mv) {
			LOG_ERR("stage 3: %s = %d mV, outside %d..%d mV", info->name,
				mv, stage3_rails[i].min_mv, stage3_rails[i].max_mv);
			bad++;
			continue;
		}

		LOG_INF("stage 3: %s = %d mV, %d uA", info->name, mv, r->current_ua);
	}

	return (bad == 0) ? 0 : -EIO;
}

/* ------------------------------------------------------------------------ */
/* Stage 7 — timing engine                                                  */
/* ------------------------------------------------------------------------ */

static int stage7_timing(void)
{
	int rc;

	LOG_INF("stage 7: timing engine");

	rc = sts_clkmux_init();
	if (rc != 0) {
		return rc;
	}

	rc = sts_pps_init();
	if (rc != 0) {
		return rc;
	}

	rc = sts_extref_mon_init();
	if (rc != 0) {
		return rc;
	}

	return sts_discipline_start();
}

/* ------------------------------------------------------------------------ */

int sts_platform_init(void)
{
	int first_err = 0;
	int rc;

#define STEP(what, call)                                                       \
	do {                                                                   \
		rc = (call);                                                   \
		if (rc != 0) {                                                 \
			LOG_ERR("%s failed (%d)", (what), rc);                 \
			if (first_err == 0) {                                  \
				first_err = rc;                                \
			}                                                      \
		}                                                              \
	} while (0)

	/*
	 * Order is load-bearing. sts_app_early_init() has to be first because
	 * everything below it logs through the ring and reads config; the
	 * fault context has to exist before any alarm can be raised; and the
	 * supervisor's outputs have to be driven to their safe states before a
	 * thread can start changing them.
	 */
	rc = sts_app_early_init();
	if (rc != 0) {
		/* No log ring, no config: nothing further can be trusted. */
		LOG_ERR("sts_app_early_init failed (%d)", rc);
		return rc;
	}

	STEP("fault init", sts_fault_init());
	STEP("supervisor init", sts_supervisor_init());

	log_boot_banner();

	STEP("stage 2", stage2_release_resets());
	STEP("SPI4 init", sts_spi4_init());
	STEP("stage 3", stage3_verify_rails());

	STEP("PFI init", sts_pfi_init());
	STEP("panel LED PWM init", sts_panel_led_init());

	STEP("io_scan", sts_io_scan_start());
	STEP("housekeeping", sts_hk_start());

	/*
	 * The watchdog kicker before pwrseq, because pwrseq's stage 9 asserts
	 * WDT_EN and sts_supervisor_arm() refuses to do that with no kicker
	 * running — arming a windowed watchdog nothing is feeding is a guaranteed
	 * cold cycle 1.4 s later.
	 */
	STEP("WDT kicker", sts_supervisor_wdt_start());

	STEP("stage 7", stage7_timing());

	/*
	 * The GNSS link. Started here rather than from stage 5 because the thread
	 * only parses whatever arrives on USART3; the *configuration walk* is
	 * stage 5.5's PWRSEQ_ACT_GNSS_CONFIG_REQUEST, which runs after GPS_PWR_EN
	 * and the reset release, so the F9T is powered and listening before any
	 * VALSET is sent and its ACK timeout is not spent on a dark receiver.
	 */
	STEP("gnss", sts_gnss_start());

	/*
	 * Stage 5 onward (GPS, display, guarded Rb, watchdog, relay) plus the
	 * runtime fault responses are driven by the pwrseq stage machine, run
	 * from the housekeeping tick. It observes the infrastructure this
	 * function just brought up through its inputs and picks up from there.
	 */
	STEP("pwrseq", sts_pwrseq_start(k_uptime_get_32()));

#undef STEP

	if (first_err != 0) {
		LOG_WRN("platform bring-up completed with errors (first %d)",
			first_err);
	} else {
		LOG_INF("platform bring-up complete");
	}

	return first_err;
}
