/*
 * STS1000 "Meridian" — bring-up sequencer entry.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * This file is deliberately thin. It owns the *order* of bring-up — the stage
 * sequence of docs/sts1000_firmware_hardware_interface.md §2 — and nothing
 * else. The work of each stage belongs to one of the four glue areas:
 *
 *   platform  stages 2-3 and 5-9: buses, rail verification, GNSS power, the
 *             timing engine, the 1 kHz scan, housekeeping and the supervisor.
 *   console   USB, shell, MCP, settings/NVS, LittleFS.
 *   net       stage 4 onwards: link, DHCP, NTP/NTS, PTP, SNMP.
 *   ui        stage 6: display, touch, encoder, panel LEDs.
 *
 * The fail-safe principle of §2 is that every gated load and every resettable
 * device sits in its safe/off state at reset, held there by an external pull,
 * and firmware releases them deliberately and in order. Ordering here reflects
 * that: the platform area verifies rails before any area is allowed to enable
 * a load, and the console area comes up next so that a failure in net or ui is
 * observable on the shell.
 *
 * The three area entry points have __weak no-op fallbacks
 * (src/zephyr/platform/sts_area_weak.c), so the image links and boots with any
 * subset of CONFIG_STS1000_{NET,CONSOLE,UI} disabled or unimplemented.
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts1000_main, CONFIG_STS1000_LOG_LEVEL);

/*
 * An area that fails to start is reported and stepped over: none of the three
 * is required for the timing engine, which is the product's reason to exist. A
 * platform failure is different — it means rails or buses are unusable — but
 * even that must not spin here, because the console area is the only way to
 * diagnose it and the external watchdog is not armed until Stage 9.
 */
static void run_area(const char *name, int (*start)(void))
{
	int rc = start();

	if (rc != 0) {
		LOG_ERR("%s area failed to start (%d); continuing degraded", name, rc);
	}
}

int main(void)
{
	int rc;

	LOG_INF("=== stage 2-3: platform bring-up ===");
	rc = sts_platform_init();
	if (rc != 0) {
		LOG_ERR("platform init failed (%d): running degraded", rc);
	}

	if (IS_ENABLED(CONFIG_STS1000_CONSOLE)) {
		LOG_INF("=== console area: usb, shell, mcp, storage ===");
		run_area("console", sts_console_start);
	}

	if (IS_ENABLED(CONFIG_STS1000_NET)) {
		LOG_INF("=== stage 4: network area ===");
		run_area("net", sts_net_start);
	}

	if (IS_ENABLED(CONFIG_STS1000_UI)) {
		LOG_INF("=== stage 6: local ui area ===");
		run_area("ui", sts_ui_start);
	}

	LOG_INF("=== bring-up complete ===");

	/*
	 * Every subsystem runs on its own static thread; main has no further
	 * work. Returning from main() in Zephyr terminates only this thread.
	 */
	return 0;
}
