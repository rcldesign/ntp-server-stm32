/*
 * STS1000 "Meridian" — PPS input capture (TIM2_CH1 / PA0, TIM3_CH1 / PC6).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Interface ref §3. TIMEPULSE from the ZED-F9T lands on PA0 (TIM2_CH1, AF1)
 * and TIMEPULSE2 on PC6 (TIM3_CH1, AF2). PA0 is the primary; PC6 exists to
 * cross-check it (core/disc, cfg.xcheck_tol_ns = 250 ns).
 *
 * ---------------------------------------------------------------------------
 * Clocking
 * ---------------------------------------------------------------------------
 * Both timers hang off APB1. The board .dts sets ahb-prescaler = 1 and
 * apb1-prescaler = 1 with SYSCLK = 250 MHz, so PCLK1 = 250 MHz and — because
 * the APB prescaler is 1, which is the case where the x2 timer multiplier does
 * NOT apply — the timer input clock is 250 MHz. `st,prescaler = <0>` on both
 * nodes leaves CK_CNT at the full rate:
 *
 *     4.000 ns per count; TIM2 (32-bit) wraps every 17.18 s
 *                         TIM3 (16-bit) wraps every 262.14 us
 *
 * That derivation is not hardcoded. sts_pps_init() reads the *live* rate back
 * through pwm_get_cycles_per_sec() on the bound st,stm32-pwm device, which
 * computes it from the running RCC tree, and logs it. Everything downstream —
 * disc_pps_t::primary_ns_per_count, the one-second prediction step — is
 * derived from that number, so a clock-tree change cannot silently mis-scale
 * the loop.
 *
 * ---------------------------------------------------------------------------
 * Why the st,stm32-pwm child node stays enabled
 * ---------------------------------------------------------------------------
 * Zephyr has no input-capture driver for these timers, so the capture
 * registers are programmed directly here. The `pwm` child of each timer node
 * is still left bound, because pwm_stm32_init() does exactly the three things
 * this file would otherwise have to duplicate: clock_control_on() for the
 * timer, reset_line_toggle_dt(), and pinctrl_apply_state() for the AF mux. It
 * leaves ARR at 0 (LL_TIM_Init with Autoreload = 0) and the counter enabled;
 * sts_pps_init() therefore sets ARR to the timer's full width before enabling
 * capture. CONFIG_PWM_CAPTURE stays off so the driver installs no ISR of its
 * own on these IRQ lines.
 *
 * ---------------------------------------------------------------------------
 * Pairing PC6 with PA0 without a 3.8 kHz interrupt
 * ---------------------------------------------------------------------------
 * TIM3 is 16-bit and wraps 3815 times a second, so a software 32-bit extension
 * would need an update ISR at that rate. It is not needed. Both timers run
 * from the same 250 MHz clock, so a TIM3 capture can be re-expressed on the
 * TIM2 timebase from inside the TIM2 capture ISR:
 *
 *     age16 = (uint16_t)(TIM3->CNT - TIM3->CCR1)   // counts since the capture
 *     sec32 = TIM2->CNT - age16                    // same instant, TIM2 domain
 *
 * The TIM2->CNT and TIM3->CNT reads are a few instructions apart, so `sec32`
 * carries a small constant bias. It cancels exactly: the phase error handed to
 * core/disc is wrap_diff(expected, count) where `expected` was itself built
 * from a previous `sec32`, and a constant common to both cancels in the
 * difference. Only the read-gap *jitter* (a few nanoseconds) survives, against
 * a 250 ns cross-check tolerance.
 *
 * The pairing is gated on `age16 < 32768`, i.e. the two edges must be within
 * 131 us of each other. A stale CC1IF left over from the previous second
 * fails that gate by construction: 250e6 mod 65536 = 45696 counts, and the
 * OCXO would have to be ~52 ppm off for that to alias into the accepted
 * window — two orders of magnitude outside its +-0.4 ppm pull range.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <stm32_ll_tim.h>

#include "zephyr/platform/platform.h"

LOG_MODULE_REGISTER(sts_pps, CONFIG_STS1000_LOG_LEVEL);

#define PPS_PRIMARY_TIMER_NODE   DT_NODELABEL(timers2)
#define PPS_SECONDARY_TIMER_NODE DT_NODELABEL(timers3)
#define PPS_PRIMARY_PWM_NODE     DT_NODELABEL(pps_primary)
#define PPS_SECONDARY_PWM_NODE   DT_NODELABEL(pps_secondary)

BUILD_ASSERT(DT_NODE_HAS_STATUS(PPS_PRIMARY_PWM_NODE, okay),
	     "pps_primary must be enabled: pwm_stm32 owns the TIM2 clock and PA0 pinmux");
BUILD_ASSERT(DT_NODE_HAS_STATUS(PPS_SECONDARY_PWM_NODE, okay),
	     "pps_secondary must be enabled: pwm_stm32 owns the TIM3 clock and PC6 pinmux");

/* The capture ISRs must not be delayed by an ordinary driver ISR. 0 is the
 * highest configurable Zephyr IRQ priority (above it are the zero-latency
 * levels, which cannot call kernel APIs). */
#define PPS_IRQ_PRIO 0

static TIM_TypeDef *const pps_tim2 = (TIM_TypeDef *)DT_REG_ADDR(PPS_PRIMARY_TIMER_NODE);
static TIM_TypeDef *const pps_tim3 = (TIM_TypeDef *)DT_REG_ADDR(PPS_SECONDARY_TIMER_NODE);

/** Ceiling on the TIM3->TIM2 re-expression: 32768 counts = 131 us. */
#define PPS_XCHECK_MAX_AGE 32768U

static K_SEM_DEFINE(pps_sem, 0, 1);

static struct {
	sts_pps_capture_t cap; /* written by the ISR, read under irq_lock */
	uint32_t timer_hz;
	uint32_t captures;
	uint32_t lost;
	bool started;
} pps;

static void pps_isr(const void *arg)
{
	uint32_t t2_now;
	uint32_t c3_cap;
	uint16_t age16;

	ARG_UNUSED(arg);

	if (!LL_TIM_IsActiveFlag_CC1(pps_tim2)) {
		/* Not ours — nothing else on this line is enabled, but a
		 * spurious entry must not fabricate a capture. */
		return;
	}

	/*
	 * Reading CCR1 clears CC1IF. Read the overcapture flag first: it says
	 * a second edge arrived before this one was read, i.e. a capture was
	 * lost, which is the one condition that invalidates the running
	 * one-second prediction.
	 */
	pps.cap.tim2_overcapture = LL_TIM_IsActiveFlag_CC1OVR(pps_tim2);
	if (pps.cap.tim2_overcapture) {
		LL_TIM_ClearFlag_CC1OVR(pps_tim2);
		pps.lost++;
	}

	pps.cap.tim2_cnt = LL_TIM_IC_GetCaptureCH1(pps_tim2);

	/* Re-express the PC6 capture on the TIM2 timebase — see the banner. */
	if (LL_TIM_IsActiveFlag_CC1(pps_tim3)) {
		c3_cap = LL_TIM_IC_GetCaptureCH1(pps_tim3); /* clears CC1IF */
		age16 = (uint16_t)((uint16_t)LL_TIM_GetCounter(pps_tim3) -
				   (uint16_t)c3_cap);
		t2_now = LL_TIM_GetCounter(pps_tim2);

		pps.cap.tim3_cnt = t2_now - (uint32_t)age16;
		pps.cap.tim3_wraps = 0U;
		pps.cap.tim3_valid = (age16 < PPS_XCHECK_MAX_AGE);
		LL_TIM_ClearFlag_CC1OVR(pps_tim3);
	} else {
		pps.cap.tim3_valid = false;
	}

	pps.cap.mono_ms = (uint64_t)k_uptime_get();
	pps.cap.seq++;
	pps.captures++;

	k_sem_give(&pps_sem);
}

static void pps_configure_capture(TIM_TypeDef *tim, uint32_t arr)
{
	/*
	 * pwm_stm32_init() left ARR at 0 and the counter running. Widen it to
	 * the full counter width so the timebase free-runs, then set up CH1 as
	 * a direct rising-edge input capture with the timer's own digital
	 * filter at fDTS/1, N = 8 — eight consecutive agreeing samples at
	 * 250 MHz, i.e. 32 ns of glitch rejection on a signal whose edge we
	 * care about to 4 ns.
	 */
	LL_TIM_DisableCounter(tim);
	LL_TIM_SetCounter(tim, 0U);
	LL_TIM_SetAutoReload(tim, arr);
	LL_TIM_SetClockDivision(tim, LL_TIM_CLOCKDIVISION_DIV1);

	LL_TIM_IC_SetActiveInput(tim, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
	LL_TIM_IC_SetPrescaler(tim, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
	LL_TIM_IC_SetFilter(tim, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1_N8);
	LL_TIM_IC_SetPolarity(tim, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);

	LL_TIM_CC_EnableChannel(tim, LL_TIM_CHANNEL_CH1);
	LL_TIM_ClearFlag_CC1(tim);
	LL_TIM_ClearFlag_CC1OVR(tim);

	/* Force an update so PSC/ARR latch, then clear the flag it raises. */
	LL_TIM_GenerateEvent_UPDATE(tim);
	LL_TIM_ClearFlag_UPDATE(tim);

	LL_TIM_EnableCounter(tim);
}

int sts_pps_init(void)
{
	const struct device *pwm_primary = DEVICE_DT_GET(PPS_PRIMARY_PWM_NODE);
	const struct device *pwm_secondary = DEVICE_DT_GET(PPS_SECONDARY_PWM_NODE);
	uint64_t cycles = 0;
	int rc;

	if (pps.started) {
		return -EALREADY;
	}

	if (!device_is_ready(pwm_primary) || !device_is_ready(pwm_secondary)) {
		LOG_ERR("PPS timers not ready (TIM2 %d, TIM3 %d)",
			(int)device_is_ready(pwm_primary),
			(int)device_is_ready(pwm_secondary));
		return -ENODEV;
	}

	/* The live timer rate from the RCC tree, not a compile-time constant. */
	rc = pwm_get_cycles_per_sec(pwm_primary, 0, &cycles);
	if (rc != 0 || cycles == 0U || cycles > UINT32_MAX) {
		LOG_ERR("PPS: cannot determine TIM2 clock (rc %d, %llu Hz)", rc,
			(unsigned long long)cycles);
		return (rc != 0) ? rc : -EIO;
	}
	pps.timer_hz = (uint32_t)cycles;

	/*
	 * The one-second prediction step is exactly `timer_hz` counts, which
	 * only works if the rate is an integer number of counts per second.
	 * At 250 MHz with prescaler 0 it is; a clock tree that made it
	 * otherwise would put a fractional-count error into every second.
	 */
	if (pps.timer_hz < 1000000U) {
		LOG_ERR("PPS: timer clock %u Hz is implausibly low", pps.timer_hz);
		return -EIO;
	}

	pps_configure_capture(pps_tim2, UINT32_MAX);
	pps_configure_capture(pps_tim3, UINT16_MAX);

	IRQ_CONNECT(DT_IRQ_BY_NAME(PPS_PRIMARY_TIMER_NODE, global, irq), PPS_IRQ_PRIO,
		    pps_isr, NULL, 0);
	irq_enable(DT_IRQ_BY_NAME(PPS_PRIMARY_TIMER_NODE, global, irq));

	LL_TIM_EnableIT_CC1(pps_tim2);
	/* TIM3 raises no interrupt: its capture is harvested by the TIM2 ISR. */

	pps.started = true;

	LOG_INF("PPS capture up: TIM2_CH1/PA0 + TIM3_CH1/PC6, %u Hz (%u.%03u ns/count)",
		pps.timer_hz, (unsigned int)(1000000000U / pps.timer_hz),
		(unsigned int)((1000000000ULL * 1000U / pps.timer_hz) % 1000U));

	return 0;
}

int sts_pps_wait(sts_pps_capture_t *out, uint32_t timeout_ms)
{
	unsigned int key;
	int rc;

	if (out == NULL) {
		return -EINVAL;
	}
	if (!pps.started) {
		return -ENODEV;
	}

	rc = k_sem_take(&pps_sem, K_MSEC(timeout_ms));
	if (rc != 0) {
		return -EAGAIN;
	}

	key = irq_lock();
	*out = pps.cap;
	irq_unlock(key);

	return 0;
}

uint32_t sts_pps_timer_hz(void)
{
	return pps.timer_hz;
}

void sts_pps_counters(uint32_t *captures, uint32_t *lost)
{
	unsigned int key = irq_lock();

	if (captures != NULL) {
		*captures = pps.captures;
	}
	if (lost != NULL) {
		*lost = pps.lost;
	}

	irq_unlock(key);
}
