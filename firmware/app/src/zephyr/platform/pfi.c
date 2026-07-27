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
 * What happens in the ISR, in order of how badly it is needed
 * (docs/sts1000_power_fail_input.md §5):
 *   1. Freeze the OCXO actuator. disc_park() is a pure state change on the
 *      discipline context; the DAC keeps holding its last code, which is what
 *      makes the Vc value recoverable on the next boot (interface ref §2).
 *   2. Kick the NVS fast-save onto the system work queue.
 *   3. Quiesce the rubidium at the pins — RB_VCC_GATE then RB_PWR_EN. The FE's
 *      warm-up surge is the largest concurrent load on the board, so shedding it
 *      is what *extends* the hold-up window everything else has to finish in.
 *      Two GPIO register writes; nothing else in the list is cheaper per joule
 *      saved.
 *   4. Latch the event so the housekeeping thread can run pwrseq's ordered park
 *      list (which also sets the clean-shutdown flag for the next boot).
 *
 * Step 4 is deliberately not "call pwrseq_pfi() here". That function purges and
 * rebuilds the action queue with no internal locking (pwrseq.h §pwrseq_pfi), and
 * from ISR context the purge can race a mid-flight pwrseq_action_get() in the
 * housekeeping thread and delete the rest of the park list. Steps 1-3 are the
 * parts that must happen within the ~4.8 ms hold-up, and they are all
 * lock-free — so the queue work is left to the thread that owns the queue.
 *
 * The ISR does not log, does not touch I2C, and does not take a mutex.
 *
 * ---------------------------------------------------------------------------
 * Recovery
 * ---------------------------------------------------------------------------
 * A PFI edge that is not followed by a power failure — a spurious comparator
 * trip, or a brown-out the supply rode out — must not cost the board its clock
 * for good. The latch used to be write-once, so a single edge left the DAC
 * frozen, the stratum at UNSYNC and FAULT_ALARM_PFI asserted (which is in the
 * relay-disqualify set, so K2 stayed released and the RGB stayed red) until
 * someone power-cycled the box.
 *
 * sts_pfi_service() re-samples PE8 from the housekeeping tick and, once the
 * comparator has read de-asserted continuously for PFI_RECOVER_DWELL_MS, clears
 * the latch, clears the alarm and asks the discipline thread to unpark.
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
#include "zephyr/sts_app.h"
#include "storage/sts_store.h"

LOG_MODULE_REGISTER(sts_pfi, CONFIG_STS1000_LOG_LEVEL);

/*
 * How long PE8 must read de-asserted before the park is released. The comparator
 * has built-in hysteresis (power_fail_input §3), so this only has to outlast the
 * bulk capacitance settling after the input recovers; two seconds is many
 * hold-up windows and still fast enough that a transient costs one recovery ramp
 * rather than a service call.
 */
#define PFI_RECOVER_DWELL_MS 2000U

static const struct gpio_dt_spec pfi = STS_USER_GPIO(pfi_gpios);

static struct gpio_callback pfi_cb;
static atomic_t pfi_latched = ATOMIC_INIT(0);

static struct {
	bool alarm_raised;   /* FAULT_ALARM_PFI asserted by this module */
	bool park_list_done; /* pwrseq's park list has been run for this latch */
	bool good;           /* PE8 currently reads de-asserted */
	uint32_t good_since_ms;
} pfi_state;

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
	sts_pwrseq_rb_quiesce_from_isr();
}

bool sts_pfi_fired(void)
{
	return atomic_get(&pfi_latched) != 0;
}

void sts_pfi_service(uint32_t now_ms)
{
	bool asserted;

	if (!gpio_is_ready_dt(&pfi)) {
		return;
	}

	if (atomic_get(&pfi_latched) == 0) {
		pfi_state.good = false;
		return;
	}

	/* Raise the alarm and run the ordered park list once per latch. */
	if (!pfi_state.alarm_raised) {
		(void)sts_alarm_set(FAULT_ALARM_PFI, true);
		pfi_state.alarm_raised = true;
	}
	if (!pfi_state.park_list_done) {
		/*
		 * power_fail_input §5 items 2-4: persist the volatile-critical
		 * set, re-drive the rubidium off (idempotent — the ISR already
		 * did the pins) and set the clean-shutdown marker so the next
		 * boot can tell a line drop from a crash.
		 */
		sts_pwrseq_pfi(now_ms);
		pfi_state.park_list_done = true;
	}

	/* PFI is active-low in devicetree, so a logical 1 is "asserted". */
	asserted = gpio_pin_get_dt(&pfi) == 1;
	if (asserted) {
		pfi_state.good = false;
		return;
	}

	if (!pfi_state.good) {
		pfi_state.good = true;
		pfi_state.good_since_ms = now_ms;
		return;
	}
	if ((now_ms - pfi_state.good_since_ms) < PFI_RECOVER_DWELL_MS) {
		return;
	}

	/*
	 * The supply came back. Release everything the latch held down, in the
	 * order that keeps the fault picture honest: clear the alarm (so K2 and the
	 * RGB can recover), then resume the loop in RECOVERING — which is a
	 * rate-limited pull-in carrying the retained error, not a step.
	 */
	atomic_set(&pfi_latched, 0);
	pfi_state.alarm_raised = false;
	pfi_state.park_list_done = false;
	pfi_state.good = false;

	(void)sts_alarm_set(FAULT_ALARM_PFI, false);
	sts_discipline_unpark_request();

	sts_log(LOGR_SUB_PWR, LOGR_NOTICE,
		"PFI released for %u ms: discipline unparked, alarm cleared",
		PFI_RECOVER_DWELL_MS);
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
