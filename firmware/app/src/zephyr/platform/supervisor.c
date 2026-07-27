/*
 * STS1000 "Meridian" — supervisor: watchdog kick, holdover relay, status RGB.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Called at 4 Hz from the housekeeping thread. Three outputs, each with a
 * fail-safe direction that firmware has to work to move away from:
 *
 *   WDT_KICK (PB2)   TPS3430 windowed watchdog. Kicked only when every
 *                    registered liveness participant has fed inside its
 *                    deadline. Silence is the safe answer: the watchdog times
 *                    out, WDO_N drives POE_KILL, and the board cold-cycles.
 *   WDT_EN (PC12)    Armed last, at stage 9, and never de-asserted.
 *   K2 relay (PA6)   Normally de-energized = ALARM. Driven high only when no
 *                    disqualifying alarm is active AND the clock is actually
 *                    serving (ARCHITECTURE.md §10.6).
 *   RGB D5           TIM4_CH1..3, common anode with low-side NPN per colour,
 *                    so a duty of 0 is off and the idle state is dark.
 *
 * ---------------------------------------------------------------------------
 * Why the kick is a level decision, not a heartbeat
 * ---------------------------------------------------------------------------
 * The TPS3430 window means a kick that is too *early* is as fatal as one that
 * is too late (docs/sts1000_external_wdt.md). The cadence is therefore set by
 * this function's own 4 Hz call rate — one kick per call, never more — and the
 * liveness AND-gate only decides whether that kick happens. Feeding the
 * watchdog from each participant directly would produce exactly the burst of
 * early kicks the window exists to catch.
 */

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "zephyr/platform/platform.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_super, CONFIG_STS1000_LOG_LEVEL);

static const struct gpio_dt_spec wdt_kick = STS_USER_GPIO(wdt_kick_gpios);
static const struct gpio_dt_spec wdt_en = STS_USER_GPIO(wdt_en_gpios);
static const struct gpio_dt_spec relay = STS_USER_GPIO(holdover_relay_gpios);

#define RGB_PWM_NODE DT_NODELABEL(rgb_pwm)
static const struct device *const rgb_pwm = DEVICE_DT_GET(RGB_PWM_NODE);

/* TIM4 prescaler 249 -> 1 MHz tick, so a 1 kHz LED PWM is 1000 steps. */
#define RGB_PERIOD_NS (1000000000U / 1000U)
#define RGB_CH_RED   1  /* TIM4_CH1, PD12 */
#define RGB_CH_GREEN 2  /* TIM4_CH2, PD13 */
#define RGB_CH_BLUE  3  /* TIM4_CH3, PD14 */

static struct {
	bool armed;         /* WDT_EN asserted (stage 9 complete) */
	bool seq_eligible;  /* pwrseq reached RELAY_ELIGIBLE (stage 9) */
	bool relay_on;
	uint32_t kicks;
	uint32_t kicks_withheld;
	uint32_t last_stale_mask;
	fault_rgb_state_t rgb;
	uint32_t identify_until_ms;
	bool ready;
} super;

void sts_supervisor_set_seq_eligible(bool eligible)
{
	super.seq_eligible = eligible;
}

/* ------------------------------------------------------------------ RGB -- */

static void rgb_set(uint8_t r_pct, uint8_t g_pct, uint8_t b_pct)
{
	const struct {
		uint32_t ch;
		uint8_t pct;
	} out[] = {
		{ RGB_CH_RED, r_pct }, { RGB_CH_GREEN, g_pct }, { RGB_CH_BLUE, b_pct },
	};

	for (size_t i = 0; i < ARRAY_SIZE(out); i++) {
		uint32_t pulse = ((uint32_t)out[i].pct * RGB_PERIOD_NS) / 100U;

		(void)pwm_set(rgb_pwm, out[i].ch, RGB_PERIOD_NS, pulse, 0);
	}
}

static void rgb_apply(fault_rgb_state_t state, uint32_t now_ms)
{
	/*
	 * Brightness is deliberately not 100 %: the ballast table in
	 * docs/sts1000_rgb_indicator.md sizes each colour's series resistor for
	 * a matched apparent brightness at full duty, and the indicator is read
	 * across a rack aisle, not stared at.
	 */
	switch (state) {
	case FAULT_RGB_GREEN:
		rgb_set(0U, 60U, 0U);
		break;
	case FAULT_RGB_AMBER:
		rgb_set(70U, 35U, 0U);
		break;
	case FAULT_RGB_RED:
		rgb_set(70U, 0U, 0U);
		break;
	case FAULT_RGB_BLUE_PULSE:
		/* 1 Hz square pulse, phase taken from the monotonic clock so
		 * multiple units in a rack do not have to agree on anything. */
		rgb_set(0U, 0U, ((now_ms / 500U) % 2U == 0U) ? 80U : 0U);
		break;
	case FAULT_RGB_OFF:
	default:
		rgb_set(0U, 0U, 0U);
		break;
	}
}

void sts_supervisor_identify(uint32_t duration_ms)
{
	super.identify_until_ms = k_uptime_get_32() + duration_ms;
}

/* ---------------------------------------------------------------- relay -- */

static void relay_apply(bool eligible, const quality_block_t *q)
{
	/*
	 * Two independent gates. fault_relay_eligible() says no disqualifying
	 * alarm is active; the quality block says the clock is actually
	 * serving. Both must hold — an alarm-free box that has never locked is
	 * not a healthy grandmaster, and a locked box with a dead OCXO rail is
	 * not one either.
	 */
	bool serving = (q->stratum == QUALITY_STRATUM_PRIMARY) &&
		       (q->lock_state == QUALITY_LOCK_LOCKED);
	/* Three gates: pwrseq has reached stage 9 (seq_eligible), no
	 * disqualifying alarm is active (eligible), and the clock is actually
	 * serving. The sequence gate is what keeps K2 de-energized through
	 * bring-up even if the OCXO locks early. */
	bool want = super.seq_eligible && eligible && serving;

	if (want == super.relay_on) {
		return;
	}

	if (gpio_pin_set_dt(&relay, want ? 1 : 0) != 0) {
		return;
	}

	super.relay_on = want;
	sts_log(LOGR_SUB_FAULT, want ? LOGR_NOTICE : LOGR_WARN,
		"holdover relay %s", want ? "energized (service OK)" : "released");
}

/* ------------------------------------------------------------ watchdog -- */

static void wdt_kick_once(void)
{
	/*
	 * A pulse, not a toggle: the TPS3430 edge-triggers on the WDI
	 * transition, and a toggle would halve the effective cadence and put
	 * the alternate kicks in a different part of the window.
	 */
	(void)gpio_pin_set_dt(&wdt_kick, 1);
	k_busy_wait(CONFIG_STS1000_WDT_KICK_WIDTH_US);
	(void)gpio_pin_set_dt(&wdt_kick, 0);

	super.kicks++;
}

int sts_supervisor_arm(void)
{
	int rc;

	if (!super.ready) {
		return -ENODEV;
	}
	if (super.armed) {
		return 0;
	}

	/* Kick once immediately before arming so the first window opens with
	 * the watchdog already fed. */
	wdt_kick_once();

	rc = gpio_pin_set_dt(&wdt_en, 1);
	if (rc != 0) {
		LOG_ERR("WDT_EN assert failed (%d)", rc);
		return rc;
	}

	super.armed = true;
	sts_log(LOGR_SUB_SYS, LOGR_NOTICE,
		"stage 9: external watchdog armed (%u liveness participants)",
		sts_liveness_count());

	return 0;
}

/* --------------------------------------------------------------- step --- */

void sts_supervisor_step(uint32_t now_ms)
{
	quality_block_t q;
	fault_rgb_in_t rgb_in;
	fault_rgb_state_t rgb_state;
	uint32_t stale;
	bool eligible;
	bool any_fault;

	if (!super.ready) {
		return;
	}

	stale = sts_liveness_stale_mask(now_ms);

	/* No kick until pwrseq has armed the watchdog (WDT_EN high). Before
	 * that the TPS3430 is not watching, and kicking into a closed window
	 * once armed is what a premature kick would risk. */
	if (!super.armed) {
		super.last_stale_mask = stale;
		goto relay_rgb;
	}

	if (stale == 0U) {
		wdt_kick_once();
	} else {
		super.kicks_withheld++;

		if (stale != super.last_stale_mask) {
			for (uint32_t i = 0; i < sts_liveness_count(); i++) {
				if ((stale & BIT(i)) != 0U) {
					sts_log(LOGR_SUB_SYS, LOGR_CRIT,
						"liveness lost: %s — WDT kick withheld",
						sts_liveness_name(i));
				}
			}
		}
	}
	super.last_stale_mask = stale;

	(void)sts_quality_snapshot(&q);

	sts_fault_lock();
	eligible = fault_relay_eligible(sts_fault());
	any_fault = fault_any_active(sts_fault());
	sts_fault_unlock();

	relay_apply(eligible, &q);

	rgb_in.locked = (q.lock_state == QUALITY_LOCK_LOCKED) &&
			(q.stratum == QUALITY_STRATUM_PRIMARY);
	rgb_in.holdover = q.holdover;
	rgb_in.warming = ((q.flags & QUALITY_FLAG_OCXO_WARM) == 0U) ||
			 (q.lock_state == QUALITY_LOCK_ACQUIRING) ||
			 (q.lock_state == QUALITY_LOCK_LOCKING);
	rgb_in.any_fault = any_fault;
	rgb_in.identify = (super.identify_until_ms != 0U) &&
			  ((int32_t)(super.identify_until_ms - now_ms) > 0);

	if (!rgb_in.identify) {
		super.identify_until_ms = 0U;
	}

	rgb_state = fault_rgb_state(&rgb_in);
	if (rgb_state != super.rgb || rgb_state == FAULT_RGB_BLUE_PULSE) {
		super.rgb = rgb_state;
		rgb_apply(rgb_state, now_ms);
	}

	/*
	 * Self-confirm the running image once the box is genuinely healthy.
	 * ARCHITECTURE.md §3: an unconfirmed image is reverted by MCUboot on
	 * the next boot, so confirming too early defeats the whole mechanism
	 * and confirming never means one reboot loses the update.
	 */
	if (sts_update_pending_confirm() && stale == 0U && !any_fault &&
	    q.lock_state == QUALITY_LOCK_LOCKED) {
		if (sts_update_self_confirm() == 0) {
			sts_log(LOGR_SUB_SYS, LOGR_NOTICE,
				"running image confirmed (healthy)");
		}
	}
}

int sts_supervisor_init(void)
{
	int rc;

	if (!gpio_is_ready_dt(&wdt_kick) || !gpio_is_ready_dt(&wdt_en) ||
	    !gpio_is_ready_dt(&relay)) {
		LOG_ERR("supervisor: GPIO not ready");
		return -ENODEV;
	}
	if (!device_is_ready(rgb_pwm)) {
		LOG_ERR("supervisor: RGB PWM not ready");
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&wdt_kick, GPIO_OUTPUT_INACTIVE);
	if (rc == 0) {
		/* WDT_EN stays low until stage 9. Arming the watchdog before
		 * the kick cadence exists would cold-cycle the board. */
		rc = gpio_pin_configure_dt(&wdt_en, GPIO_OUTPUT_INACTIVE);
	}
	if (rc == 0) {
		/* K2 de-energized = ALARM, which is the correct state until
		 * service quality has been demonstrated. */
		rc = gpio_pin_configure_dt(&relay, GPIO_OUTPUT_INACTIVE);
	}
	if (rc != 0) {
		LOG_ERR("supervisor: GPIO configure failed (%d)", rc);
		return rc;
	}

	super.rgb = FAULT_RGB_OFF;
	rgb_apply(FAULT_RGB_OFF, 0U);

	super.ready = true;
	return 0;
}

void sts_supervisor_counters(uint32_t *kicks, uint32_t *withheld, bool *armed)
{
	if (kicks != NULL) {
		*kicks = super.kicks;
	}
	if (withheld != NULL) {
		*withheld = super.kicks_withheld;
	}
	if (armed != NULL) {
		*armed = super.armed;
	}
}
