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
 *
 * ---------------------------------------------------------------------------
 * Two consumers, one capture
 * ---------------------------------------------------------------------------
 * The discipline thread consumes captures through sts_pps_wait(), which blocks
 * on the semaphore the ISR gives. The net area needs the SAME capture for a
 * different purpose — correlating it against the ETH PTP counter so the served
 * timescale has an absolute epoch good to nanoseconds instead of the tens of
 * milliseconds the receiver's civil time can offer (sts_app.h "PPS capture
 * seam", net/sts_ppscorr.h).
 *
 * It gets it through sts_pps_epoch_get(), which copies the latched capture
 * WITHOUT taking the semaphore, so a second reader can never cost core/disc a
 * PPS tick. Nothing was added to the ISR for it: everything the correlation
 * needs is already latched there, and the ISR's budget is microseconds
 * (firmware/CLAUDE.md thread table). The receiver-side naming of the pulse — its
 * GPS week/ToW and the UBX-TIM-TP sawtooth — is resolved in the *caller's*
 * context, with the same positive ToW match that the discipline thread uses.
 *
 * That naming is the one thing this second consumer cannot inherit unchanged
 * from the first. The discipline thread runs ON the edge, so "the receiver's
 * newest records" and "the records describing the captured pulse" are the same
 * thing; a caller on an unrelated timer sees them diverge for most of every
 * second, and its phase relative to the PPS is frozen at boot because both
 * clocks come off the PLL that this same PPS disciplines. The evidence the
 * naming reads (sts_app.h sts_gnss_pulse_evidence_t) therefore carries an epoch
 * of history, and pps_name_pulse() below picks the candidate on the correct side
 * of the capture rather than assuming the newest one is.
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
#include "zephyr/sts_app.h"

#include "disc/disc.h"

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

uint32_t sts_pps_counter_now(void)
{
	/*
	 * Deliberately unguarded. This is called from inside an irq_lock() that
	 * also holds an ETH PTP register read, and the whole point of the pairing
	 * is that the two timebases are sampled as close together as the CPU
	 * allows — a `pps.started` load and branch here would widen exactly the
	 * interval being measured. pwm_stm32_init() has enabled the TIM2 clock
	 * long before any thread runs, so the read cannot fault; before
	 * sts_pps_init() the value is simply not on a known timebase, which the
	 * caller detects from sts_pps_epoch_get() reporting no capture.
	 */
	return LL_TIM_GetCounter(pps_tim2);
}

/** Two navigation epochs of slack, matching disc_thread.c. */
#define PPS_MAX_PVT_AGE_MS 2000U

BUILD_ASSERT(STS_GNSS_PULSE_OBS >= 2,
	     "naming a pulse from a non-phase-locked caller needs an epoch of "
	     "history; see sts_app.h sts_gnss_pulse_evidence_t");

/*
 * Name the pulse a capture belongs to, and the sawtooth that belongs to it.
 *
 * The rule is core/disc's disc_name_pulse() — the same positive ToW match the
 * discipline thread applies before it corrects a phase sample, for the same
 * reason: UBX-TIM-TP is emitted in the second BEFORE the pulse it describes, so
 * "the newest record" is one second early, and a qErr applied to the wrong
 * second injects the sawtooth instead of removing it. gnssmgr has already
 * normalised the record's ToW onto the GPS timescale and cleared qerr_valid when
 * it could not; what is left is to work out which ToW was captured and insist on
 * an exact match.
 *
 * The consequence here is stronger than a mis-corrected phase sample, which is
 * why nothing is assumed: the record's week + target_tow_ms is what names the
 * ABSOLUTE second the served timescale is placed on. Matching the wrong record
 * is a one-second error in every timestamp this appliance emits.
 *
 * What differs from the discipline thread is not the rule but the evidence. That
 * thread runs ON the edge, so the receiver's newest records are still the ones
 * that describe the captured pulse. This runs whenever its caller asks — the PTP
 * servo is a free-running 1 Hz k_timer with no phase relationship to the PPS at
 * all — and for most of every second the newest NAV-PVT is newer than the
 * latched capture while the newest TIM-TP already names the NEXT pulse. Pairing
 * against "the newest" therefore succeeds only inside the window between the
 * edge and that second's messages, and since both clocks derive from the same
 * PLL and that PLL is disciplined to this very PPS, the caller's phase is frozen
 * at boot: a unit that boots into the dead region stays there. So the evidence
 * carries an epoch of history and disc_name_pulse() picks the candidate on the
 * correct side of the capture, whichever that is.
 */
static void pps_name_pulse(sts_pps_epoch_t *out, const sts_pps_capture_t *cap)
{
	sts_gnss_pulse_evidence_t ev;
	disc_pvt_obs_t pvt[STS_GNSS_PULSE_OBS];
	disc_qerr_obs_t qerr[STS_GNSS_PULSE_OBS];
	disc_pulse_name_t name;
	size_t i;

	if (sts_gnss_pulse_evidence(&ev) != 0) {
		return;
	}

	for (i = 0; i < STS_GNSS_PULSE_OBS; i++) {
		pvt[i].itow_ms = ev.pvt[i].itow_ms;
		pvt[i].rx_mono_ms = ev.pvt[i].rx_mono_ms;
		pvt[i].valid = ev.pvt[i].valid;

		qerr[i].target_tow_ms = ev.qerr[i].target_tow_ms;
		qerr[i].rx_mono_ms = ev.qerr[i].rx_mono_ms;
		qerr[i].week = ev.qerr[i].week;
		qerr[i].qerr_ps = ev.qerr[i].qerr_ps;
		qerr[i].qerr_valid = ev.qerr[i].qerr_valid;
		qerr[i].valid = ev.qerr[i].valid;
	}

	if (!disc_name_pulse(pvt, STS_GNSS_PULSE_OBS, qerr, STS_GNSS_PULSE_OBS,
			     cap->mono_ms, PPS_MAX_PVT_AGE_MS, &name)) {
		return;
	}

	out->gps_week = name.week;
	out->gps_tow_ms = name.tow_ms;
	out->qerr_ps = name.qerr_ps;
	out->epoch_valid = true;
}

int sts_pps_epoch_get(sts_pps_epoch_t *out)
{
	sts_pps_capture_t cap;
	unsigned int key;

	if (out == NULL) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));

	if (!pps.started) {
		return -ENODEV;
	}

	/*
	 * The ISR writes pps.cap field by field, so the copy has to exclude it —
	 * exactly as sts_pps_wait() does. This one does NOT take the semaphore:
	 * the discipline thread is the semaphore's owner and must keep every tick
	 * it is given.
	 */
	key = irq_lock();
	cap = pps.cap;
	irq_unlock(key);

	out->seq = cap.seq;
	out->tim2_cnt = cap.tim2_cnt;
	out->timer_hz = pps.timer_hz;
	out->mono_ms = cap.mono_ms;
	out->overcapture = cap.tim2_overcapture;

	if (cap.seq != 0U) {
		pps_name_pulse(out, &cap);
	}

	return 0;
}
