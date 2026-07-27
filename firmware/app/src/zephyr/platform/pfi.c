/*
 * STS1000 "Meridian" — PFI power-fail early warning (PE8, EXTI8).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * docs/sts1000_power_fail_input.md: comparator U2A watches VOUT_P and pulls
 * PFI low while the PoE input is still above the point where the bulk
 * capacitance can hold the 3V3 rails up. The hold-up window that buys is short
 * — single-digit milliseconds — so the handler does the minimum that must
 * happen before the rails collapse and defers everything else.
 *
 * What happens in the ISR, in order of how badly it is needed:
 *   1. Freeze the OCXO actuator. disc_park() is a pure state change on the
 *      discipline context; the DAC keeps holding its last code, which is what
 *      makes the Vc value recoverable on the next boot (interface ref §2).
 *   2. Latch the event so the housekeeping thread can run pwrseq's PFI
 *      actions and the console area can fast-save the volatile-critical set
 *      (last Vc, leap, log cursor) to NVS.
 *
 * The ISR does not log, does not touch I2C, and does not take a mutex. A
 * spurious PFI edge therefore costs one parked loop, which the recovery path
 * un-parks; a real one costs nothing that had to be done anyway.
 *
 * EXTI line 8 belongs to PE8 exclusively (interface ref §6) — no other port's
 * pin 8 is wired as an interrupt on this board, so there is no shared-line
 * demultiplexing to do.
 */

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "zephyr/platform/platform.h"
#include "storage/sts_store.h"

LOG_MODULE_REGISTER(sts_pfi, CONFIG_STS1000_LOG_LEVEL);

static const struct gpio_dt_spec pfi = STS_USER_GPIO(pfi_gpios);

static struct gpio_callback pfi_cb;
static atomic_t pfi_latched = ATOMIC_INIT(0);

static void pfi_handler(const struct device *port, struct gpio_callback *cb,
			gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if (atomic_set(&pfi_latched, 1) != 0) {
		return; /* already handled; the rails are on their way down */
	}

	/*
	 * Freeze the OCXO actuator first (holds the last Vc), then kick the
	 * console area's fast-save. sts_store_critical_flush_from_isr() only
	 * schedules the flush onto the system work queue — it does not touch
	 * flash here — so it is safe in this ISR and completes in the hold-up
	 * window if the queue drains in time.
	 */
	sts_discipline_park();
	sts_store_critical_flush_from_isr();
}

bool sts_pfi_fired(void)
{
	return atomic_get(&pfi_latched) != 0;
}

int sts_pfi_init(void)
{
	int rc;

	if (!gpio_is_ready_dt(&pfi)) {
		LOG_ERR("PFI: GPIO port not ready");
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&pfi, GPIO_INPUT);
	if (rc != 0) {
		LOG_ERR("PFI: configure failed (%d)", rc);
		return rc;
	}

	/*
	 * PFI is active-low in devicetree, so GPIO_INT_EDGE_TO_ACTIVE is the
	 * falling edge on the pin — the comparator tripping, not releasing.
	 */
	rc = gpio_pin_interrupt_configure_dt(&pfi, GPIO_INT_EDGE_TO_ACTIVE);
	if (rc != 0) {
		LOG_ERR("PFI: interrupt configure failed (%d)", rc);
		return rc;
	}

	gpio_init_callback(&pfi_cb, pfi_handler, BIT(pfi.pin));

	rc = gpio_add_callback_dt(&pfi, &pfi_cb);
	if (rc != 0) {
		LOG_ERR("PFI: callback add failed (%d)", rc);
		return rc;
	}

	/*
	 * If PFI is already asserted at bring-up the board is running on a
	 * supply that is on its way out; say so rather than arming and waiting
	 * for an edge that has already happened.
	 */
	if (gpio_pin_get_dt(&pfi) == 1) {
		LOG_WRN("PFI already asserted at init: input supply is low or absent");
		atomic_set(&pfi_latched, 1);
	}

	LOG_INF("PFI armed on PE8 (EXTI8)");
	return 0;
}
