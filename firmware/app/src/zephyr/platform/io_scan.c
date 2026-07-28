/*
 * STS1000 "Meridian" — the 1 kHz GPIOF/GPIOG scan thread.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Interface ref §5. There is no I/O expander on this board: all 32 aggregated
 * inputs — seven buttons, the encoder switch, the touch INT, the reed switch,
 * three RT9742 fault flags, ten power-good lines and nine INA228 ALERTs — sit
 * on their own pin across GPIOF and GPIOG. This thread reads both input data
 * registers once a millisecond and hands the pair to core/fault, which owns the
 * diff, the per-class debounce, the event queue and the alarm table.
 *
 * Reading the whole port at once (gpio_port_get_raw) rather than 32 individual
 * pin reads is what makes the scan a *snapshot*: every signal in one sample
 * shares a timestamp, so a rail collapse that takes its power-good, its INA
 * alert and a load-switch flag with it produces one coherent picture rather
 * than a sequence that depends on read order.
 *
 * Pacing is a k_timer rather than k_sleep so a scan that runs long does not
 * push the next one out — the period stays anchored to the timer, and a missed
 * slot is counted instead of silently stretching the debounce windows that
 * core/fault measures in wall time.
 *
 * Everything this file *decides* — the signal-to-port/bit split, the ALERT
 * signal-to-monitor lookup, what each debounced event turns into, the overrun
 * and dropped-event tests — lives in sts_io_policy.h, which is Zephyr-free and
 * unit-tested on the host (tests/host/test_io_policy.c). What is left here is
 * the devicetree, the ports, the thread and the pacing.
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "console/mp_glue.h"
#include "mp/mp_stream.h"
#include "zephyr/platform/platform.h"
#include "zephyr/platform/sts_io_policy.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_io_scan, CONFIG_STS1000_LOG_LEVEL);

#define IO_SCAN_STACK_SIZE 2048
#define IO_SCAN_PRIO       11  /* ARCHITECTURE.md §6 */
#define IO_SCAN_PERIOD_MS  STS_IO_SCAN_PERIOD_MS

/*
 * sts_io_policy.h re-declares the UI input types so it can stay free of
 * sts_app.h (which pulls core/mp and core/cfg in for types nothing in the
 * policy uses). These are what stop the copy drifting.
 */
BUILD_ASSERT((int)STS_IO_UI_BUTTON == (int)STS_INPUT_BUTTON);
BUILD_ASSERT((int)STS_IO_UI_BUTTON_LONG == (int)STS_INPUT_BUTTON_LONG);
BUILD_ASSERT((int)STS_IO_UI_BUTTON_REPEAT == (int)STS_INPUT_BUTTON_REPEAT);
BUILD_ASSERT((int)STS_IO_UI_TOUCH == (int)STS_INPUT_TOUCH);
BUILD_ASSERT((int)STS_IO_UI_PROX == (int)STS_INPUT_PROX);

/*
 * The same guard for the MP event kinds, which sts_io_policy.h mirrors for the
 * same reason. A drift here would not fail: it would report every power-good
 * drop to the Field Maintenance Tool as a button press, on a channel whose
 * whole purpose is telling a technician which of those happened.
 */
BUILD_ASSERT((int)STS_IO_MP_FAULT == (int)MP_EV_FAULT);
BUILD_ASSERT((int)STS_IO_MP_BUTTON == (int)MP_EV_BUTTON);
BUILD_ASSERT((int)STS_IO_MP_PROX == (int)MP_EV_PROX);
BUILD_ASSERT((int)STS_IO_MP_TOUCH == (int)MP_EV_TOUCH);

static const struct device *const gpiof = DEVICE_DT_GET(DT_NODELABEL(gpiof));
static const struct device *const gpiog = DEVICE_DT_GET(DT_NODELABEL(gpiog));

K_THREAD_STACK_DEFINE(io_scan_stack, IO_SCAN_STACK_SIZE);
static struct k_thread io_scan_tcb;
static K_SEM_DEFINE(io_scan_sem, 0, 1);
static struct k_timer io_scan_timer;

static struct {
	uint32_t scans;
	uint32_t overruns;
	int liveness_id;
} io_scan;

/* --------------------------------------------------------------------------
 * Pin configuration
 *
 * Every scanned pin must be an input before the first read, or the IDR reflects
 * an output latch instead of the board. The /zephyr,user node carries the
 * polarity and the pull for the ones that need it (FAN_TACH's internal pull-up,
 * the NCP1095 open-drain trio); the scanned set below is plain GPIO_INPUT
 * because each line has an external pull sized in the schematic.
 * -------------------------------------------------------------------------- */

#define IO_SCAN_USER_INPUT(prop)                                                 \
	{ .spec = STS_USER_GPIO(prop), .name = #prop }

static const struct {
	struct gpio_dt_spec spec;
	const char *name;
} io_scan_inputs[] = {
	/* GPIOF — HMI, load-switch flags, backup power-good */
	IO_SCAN_USER_INPUT(disp_touch_int_gpios),
	IO_SCAN_USER_INPUT(ant_en_fault_gpios),
	IO_SCAN_USER_INPUT(disp_en_fault_gpios),
	IO_SCAN_USER_INPUT(prox_wake_gpios),
	IO_SCAN_USER_INPUT(panel_led_fault_gpios),
	IO_SCAN_USER_INPUT(ina_alert_5v_panel_gpios),
	IO_SCAN_USER_INPUT(bkp_stm_pg_gpios),
	IO_SCAN_USER_INPUT(bkp_gps_pg_gpios),

	/* GPIOG[0:7] — rail power-good */
	IO_SCAN_USER_INPUT(pg_3v3_gps_gpios),
	IO_SCAN_USER_INPUT(pg_ocxo_ldo_gpios),
	IO_SCAN_USER_INPUT(pg_3v0_rf_gpios),
	IO_SCAN_USER_INPUT(pg_5v_psu_gpios),
	IO_SCAN_USER_INPUT(pg_3v3_psu_gpios),
	IO_SCAN_USER_INPUT(pg_ocxo_psu_gpios),
	IO_SCAN_USER_INPUT(pg_rb_psu_gpios),
	IO_SCAN_USER_INPUT(pg_poe_gpios),

	/* GPIOG[8:15] — INA228 ALERT */
	IO_SCAN_USER_INPUT(ina_alert_poe_gpios),
	IO_SCAN_USER_INPUT(ina_alert_3v3_stm_gpios),
	IO_SCAN_USER_INPUT(ina_alert_5v_disp_gpios),
	IO_SCAN_USER_INPUT(ina_alert_3v3_gpios),
	IO_SCAN_USER_INPUT(ina_alert_3v3_gps_gpios),
	IO_SCAN_USER_INPUT(ina_alert_v_ant_gpios),
	IO_SCAN_USER_INPUT(ina_alert_ocxo_gpios),
	IO_SCAN_USER_INPUT(ina_alert_vcc_rb_gpios),
};

/*
 * The seven buttons and the encoder switch live in the disabled `gpio-keys`
 * node — disabled precisely so the input driver does not claim EXTI0..6/11 for
 * them (interface ref §6) — so their specs come from there rather than from
 * /zephyr,user.
 */
#define IO_SCAN_BUTTON(nodelabel) GPIO_DT_SPEC_GET(DT_NODELABEL(nodelabel), gpios)

static const struct gpio_dt_spec io_scan_buttons[] = {
	IO_SCAN_BUTTON(button_1), IO_SCAN_BUTTON(button_2), IO_SCAN_BUTTON(button_3),
	IO_SCAN_BUTTON(button_4), IO_SCAN_BUTTON(button_5), IO_SCAN_BUTTON(button_6),
	IO_SCAN_BUTTON(button_7), IO_SCAN_BUTTON(encoder_button),
};

static int io_scan_configure_pins(void)
{
	int rc;

	for (size_t i = 0; i < ARRAY_SIZE(io_scan_inputs); i++) {
		if (!gpio_is_ready_dt(&io_scan_inputs[i].spec)) {
			LOG_ERR("io_scan: %s port not ready", io_scan_inputs[i].name);
			return -ENODEV;
		}

		rc = gpio_pin_configure_dt(&io_scan_inputs[i].spec, GPIO_INPUT);
		if (rc != 0) {
			LOG_ERR("io_scan: %s configure failed (%d)",
				io_scan_inputs[i].name, rc);
			return rc;
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(io_scan_buttons); i++) {
		if (!gpio_is_ready_dt(&io_scan_buttons[i])) {
			return -ENODEV;
		}

		rc = gpio_pin_configure_dt(&io_scan_buttons[i], GPIO_INPUT);
		if (rc != 0) {
			LOG_ERR("io_scan: button %u configure failed (%d)",
				(unsigned int)i, rc);
			return rc;
		}
	}

	return 0;
}

/* --------------------------------------------------------------------------
 * Event dispatch
 * -------------------------------------------------------------------------- */

static void io_scan_dispatch(const fault_evt_t *evt)
{
	sts_io_dispatch_t plan;

	sts_io_dispatch_plan(evt->type, evt->id, evt->edge, &plan);

	if (plan.post_ui) {
		sts_input_evt_t ui = {
			.type = (uint8_t)plan.ui_type,
			.id = evt->id,
			.value = plan.ui_value,
			.mono_ms = evt->mono_ms,
		};

		sts_ui_post_input(&ui);
	}

	if (plan.ina_reread) {
		sts_hk_request_ina(plan.ina_rail);
	}

	if (plan.log) {
		/*
		 * One call site for every event's log line: the message and its
		 * severity are chosen together in the policy, so a recovery can
		 * never acquire a failure's wording or level. Every template
		 * takes exactly one %s, the signal name.
		 */
		sts_log(plan.log_sub, plan.log_level, sts_io_msg_fmt(plan.msg),
			fault_sig_name((fault_sig_t)evt->id));
	}

	if (plan.post_mp) {
		/*
		 * The Field Maintenance Tool's event channel (FMT §7.5). This
		 * costs one atomic read when no technician is subscribed and a
		 * ~48-byte copy under a spinlock when one is: mp_glue.h's
		 * contract is that the call never takes the engine mutex, never
		 * touches the console UART and never blocks, which is what lets
		 * it sit inside a 1 ms scan slot at all.
		 *
		 * evt->mono_ms, not "now": the drain runs on the console
		 * supervisor up to 250 ms later, and an event stream whose
		 * timestamps are drain times cannot be used to order a button
		 * press against the rail collapse it was a response to.
		 */
		sts_mp_post_event(plan.mp_kind, plan.mp_sub, evt->id,
				  plan.mp_edge, 0, evt->mono_ms,
				  fault_sig_name((fault_sig_t)evt->id));
	}
}

/* --------------------------------------------------------------------------
 * The scan
 * -------------------------------------------------------------------------- */

static void io_scan_timer_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	/*
	 * A binary semaphore, so a scan that overruns its slot is coalesced
	 * rather than queued: the next scan reads the *current* port state,
	 * which is what a level-triggered snapshot model wants. The miss is
	 * visible as a jump in the mono_ms core/fault sees.
	 */
	k_sem_give(&io_scan_sem);
}

static void io_scan_entry(void *p1, void *p2, void *p3)
{
	gpio_port_value_t f_val = 0;
	gpio_port_value_t g_val = 0;
	uint32_t last_ms;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	last_ms = k_uptime_get_32();

	for (;;) {
		sts_io_drop_action_t drop;
		fault_evt_t evt;
		uint32_t now_ms;
		uint32_t dropped;
		int rc_f;
		int rc_g;

		(void)k_sem_take(&io_scan_sem, K_FOREVER);

		now_ms = k_uptime_get_32();
		if (sts_io_scan_overrun(now_ms, last_ms, IO_SCAN_PERIOD_MS)) {
			io_scan.overruns++;
		}
		last_ms = now_ms;

		rc_f = gpio_port_get_raw(gpiof, &f_val);
		rc_g = gpio_port_get_raw(gpiog, &g_val);
		if (rc_f != 0 || rc_g != 0) {
			/* A port read cannot fail on this SoC (it is an IDR
			 * load), but a driver that has been de-initialised
			 * would return an error rather than a stale sample. */
			continue;
		}

		sts_fault_lock();
		(void)fault_scan_input(sts_fault(), (uint16_t)f_val, (uint16_t)g_val,
				       now_ms);
		sts_fault_unlock();

		io_scan.scans++;

		/*
		 * Drain outside the lock: dispatch calls into the UI queue and
		 * the log ring, neither of which should run with the fault
		 * context held.
		 */
		for (;;) {
			int rc;

			sts_fault_lock();
			rc = fault_evt_get(sts_fault(), &evt);
			sts_fault_unlock();

			if (rc != 0) {
				break;
			}

			io_scan_dispatch(&evt);
		}

		sts_fault_lock();
		dropped = fault_evt_dropped(sts_fault());
		sts_io_drop_action(dropped, &drop);
		if (drop.clear) {
			fault_evt_clear_dropped(sts_fault());
		}
		sts_fault_unlock();

		if (drop.log) {
			sts_log(LOGR_SUB_PWR, LOGR_WARN,
				"io_scan: %u fault events dropped", dropped);
		}

		sts_liveness_feed(io_scan.liveness_id);
	}
}

void sts_io_scan_counters(uint32_t *scans, uint32_t *overruns)
{
	if (scans != NULL) {
		*scans = io_scan.scans;
	}
	if (overruns != NULL) {
		*overruns = io_scan.overruns;
	}
}

int sts_io_scan_start(void)
{
	k_tid_t tid;
	int rc;

	if (!device_is_ready(gpiof) || !device_is_ready(gpiog)) {
		LOG_ERR("io_scan: GPIOF/GPIOG not ready");
		return -ENODEV;
	}

	rc = io_scan_configure_pins();
	if (rc != 0) {
		return rc;
	}

	io_scan.liveness_id = sts_liveness_register("io_scan");

	k_timer_init(&io_scan_timer, io_scan_timer_expiry, NULL);

	tid = k_thread_create(&io_scan_tcb, io_scan_stack, IO_SCAN_STACK_SIZE,
			      io_scan_entry, NULL, NULL, NULL, IO_SCAN_PRIO, 0,
			      K_NO_WAIT);
	k_thread_name_set(tid, "io_scan");

	k_timer_start(&io_scan_timer, K_MSEC(IO_SCAN_PERIOD_MS),
		      K_MSEC(IO_SCAN_PERIOD_MS));

	LOG_INF("io_scan up: %u Hz GPIOF/GPIOG snapshot, %u signals",
		1000U / IO_SCAN_PERIOD_MS, (unsigned int)FAULT_SIG_COUNT);

	return 0;
}
