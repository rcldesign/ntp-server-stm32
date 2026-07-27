/*
 * STS1000 "Meridian" — clock-mux executor and external-reference monitor.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Two jobs, both bound to the 74LVC1G157 clock mux (U52) that feeds PH0:
 *
 *   1. Execute the refsel action list — the HSI-bridge sequence that is the
 *      ONLY sanctioned way to write MUX_SEL/PB6 (ARCHITECTURE.md §10.1).
 *   2. Measure the frequency present on mux input B (EXTREF_MON, PB14/TIM12)
 *      so refsel's in-band guard has something to guard on.
 *
 * ---------------------------------------------------------------------------
 * The handoff, and why it is shaped this way
 * ---------------------------------------------------------------------------
 * Interface ref §3: "firmware bridges the system clock to HSI, changes MUX_SEL
 * (PB6), and relies on the STM32 CSS to catch a dead source". A dedicated
 * glitch-free mux IC is deliberately not used because it completes its handoff
 * on the *outgoing* clock's edges and hangs when that source has stopped —
 * exactly the Rb-failure case this machine exists to survive
 * (docs/sts1000_clock_mux.md §0).
 *
 * Sequence, all of it with interrupts locked:
 *
 *   HSI on -> SYSCLK = HSI -> PLL1 off -> flip PB6 -> settle
 *          -> HSE off/on (bypass) + HSERDY -> PLL1 on + PLL1RDY
 *          -> SYSCLK = PLL1 -> CSS armed
 *
 * HSE is cycled rather than left running across the mux flip: the bypass input
 * sees a runt pulse when the selector switches, and re-qualifying HSERDY after
 * the fact is the only way to know the new source is actually oscillating.
 *
 * Interrupts are locked for the whole sequence because SYSCLK — and with it
 * every APB kernel clock — drops from 250 MHz to HSI 64 MHz and back. An ISR
 * that ran in that window would drive its peripheral at 3.9x the wrong rate.
 * Two consequences, both accepted and bounded:
 *
 *   - Kernel monotonic time under-counts for the duration. SysTick keeps
 *     counting real HCLK cycles while Zephyr scales them by the compile-time
 *     250 MHz, so a handoff of D milliseconds loses about D*(1 - 64/250) =
 *     0.74*D ms of uptime. With the default 10 ms settle that is ~11 ms, once
 *     per handoff. Nothing in the timing path measures phase through
 *     k_uptime_get() — phase comes from the TIM2 capture — so this costs
 *     scheduling accuracy, not time accuracy.
 *   - The settle delay cannot use k_busy_wait(), whose calibration assumes the
 *     nominal clock. It is timed off TIM2's free-running counter instead,
 *     scaled by the SYSCLK actually in effect.
 *
 * CONFIG_STS1000_CLKMUX_MAX_SETTLE_MS caps how long refsel can ask us to hold
 * the system there.
 *
 * ---------------------------------------------------------------------------
 * CSS
 * ---------------------------------------------------------------------------
 * HSECSSON is set-only in hardware — nothing but a reset clears it — so once
 * armed, a genuine reference failure raises an NMI. Zephyr's default NMI
 * handler resets the SoC, which would turn "the rubidium died" into "the
 * grandmaster rebooted". CONFIG_RUNTIME_NMI lets us install a handler that
 * instead clears HSECSSF and latches the event; the hardware has already
 * switched SYSCLK to HSI and stopped the HSE-fed PLL, so the box keeps running
 * (slowly) and the recorded failure is fed to refsel, which reverts to the
 * OCXO and triggers a full re-lock through the same executor.
 */

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <cmsis_core.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_tim.h>

#include "zephyr/platform/platform.h"
#include "zephyr/sts_app.h"

#include "refsel/refsel.h"

LOG_MODULE_REGISTER(sts_clkmux, CONFIG_STS1000_LOG_LEVEL);

#define EXTREF_TIMER_NODE DT_NODELABEL(timers12)
#define EXTREF_PWM_NODE   DT_NODELABEL(extref_mon)
#define PPS_TIMER_NODE    DT_NODELABEL(timers2)

BUILD_ASSERT(DT_NODE_HAS_STATUS(EXTREF_PWM_NODE, okay),
	     "extref_mon must be enabled: pwm_stm32 owns the TIM12 clock and PB14 pinmux");

static TIM_TypeDef *const extref_tim = (TIM_TypeDef *)DT_REG_ADDR(EXTREF_TIMER_NODE);
static TIM_TypeDef *const pps_tim = (TIM_TypeDef *)DT_REG_ADDR(PPS_TIMER_NODE);

static const struct gpio_dt_spec mux_sel = STS_USER_GPIO(mux_sel_gpios);

/* Register-poll budgets. Deliberately iteration counts, not wall time: they
 * run with interrupts locked and at whichever clock the SoC is on. At 64 MHz
 * an iteration is a handful of cycles, so 2 000 000 is tens of milliseconds —
 * far longer than any HSE or PLL start-up, short enough not to hang. */
#define CLKMUX_POLL_LIMIT 2000000U

static atomic_t clkmux_css_events = ATOMIC_INIT(0);

/* ========================================================================= */
/* CSS / NMI                                                                 */
/* ========================================================================= */

#if defined(CONFIG_RUNTIME_NMI)
static void clkmux_nmi_handler(void)
{
	if (LL_RCC_IsActiveFlag_HSECSS()) {
		LL_RCC_ClearFlag_HSECSS();
		atomic_inc(&clkmux_css_events);
	}
	/*
	 * Deliberately no logging, no k_* call and no recovery here. The
	 * hardware has already failed SYSCLK over to HSI; recovery is a
	 * refsel decision taken by the discipline thread, which polls
	 * sts_clkmux_css_fired().
	 */
}
#endif

uint32_t sts_clkmux_css_events(void)
{
	return (uint32_t)atomic_get(&clkmux_css_events);
}

bool sts_clkmux_css_fired(void)
{
	return atomic_set(&clkmux_css_events, 0) != 0;
}

/* ========================================================================= */
/* delays that survive a SYSCLK change                                       */
/* ========================================================================= */

/** SYSCLK right now, straight from the RCC tree. */
static uint32_t clkmux_sysclk_hz(void)
{
	LL_RCC_ClocksTypeDef clocks;

	LL_RCC_GetSystemClocksFreq(&clocks);
	return clocks.SYSCLK_Frequency;
}

/*
 * Busy-wait using TIM2's free-running 32-bit counter. Both AHB and all APB
 * prescalers are 1 on this board, so the timer input clock equals SYSCLK and a
 * count is one SYSCLK cycle — correct at 250 MHz and at HSI 64 MHz alike.
 * TIM2 wraps every 17 s at 250 MHz, so the unsigned difference is safe for any
 * delay this function is asked for.
 */
static void clkmux_delay_us(uint32_t us)
{
	uint32_t hz = clkmux_sysclk_hz();
	uint32_t start = LL_TIM_GetCounter(pps_tim);
	uint32_t need;

	if (hz == 0U) {
		return;
	}

	need = (uint32_t)(((uint64_t)us * (uint64_t)hz) / 1000000ULL);

	while ((uint32_t)(LL_TIM_GetCounter(pps_tim) - start) < need) {
		/* spin */
	}
}

/* ========================================================================= */
/* MUX_SEL / HSI bridge                                                      */
/* ========================================================================= */

static int clkmux_wait(bool (*ready)(void), bool want)
{
	for (uint32_t i = 0; i < CLKMUX_POLL_LIMIT; i++) {
		if (ready() == want) {
			return 0;
		}
	}

	return -ETIMEDOUT;
}

static bool hse_ready(void)
{
	return LL_RCC_HSE_IsReady() != 0U;
}

static bool hsi_ready(void)
{
	return LL_RCC_HSI_IsReady() != 0U;
}

static bool pll1_ready(void)
{
	return LL_RCC_PLL1_IsReady() != 0U;
}

static bool sysclk_is_pll1(void)
{
	return LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_PLL1;
}

static bool sysclk_is_hsi(void)
{
	return LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_HSI;
}

/** Park SYSCLK on HSI and stop PLL1. Returns 0 when SYSCLK is on HSI. */
static int clkmux_bridge_to_hsi(void)
{
	int rc;

	LL_RCC_HSI_Enable();
	rc = clkmux_wait(hsi_ready, true);
	if (rc != 0) {
		return rc;
	}

	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
	rc = clkmux_wait(sysclk_is_hsi, true);
	if (rc != 0) {
		return rc;
	}

	LL_RCC_PLL1_Disable();
	return clkmux_wait(pll1_ready, false);
}

/** Re-qualify HSE in bypass mode against whatever the mux now forwards. */
static int clkmux_reselect_hse(void)
{
	int rc;

	LL_RCC_HSE_Disable();
	rc = clkmux_wait(hse_ready, false);
	if (rc != 0) {
		return rc;
	}

	/* Bypass must be (re)asserted while HSEON is clear: HSEBYP is
	 * write-protected once the oscillator is enabled. */
	LL_RCC_HSE_EnableBypass();
	LL_RCC_HSE_Enable();

	return clkmux_wait(hse_ready, true);
}

/** Re-lock PLL1 on the new HSE and put SYSCLK back on it. */
static int clkmux_relock_pll(void)
{
	int rc;

	LL_RCC_PLL1_Enable();
	rc = clkmux_wait(pll1_ready, true);
	if (rc != 0) {
		return rc;
	}

	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);
	rc = clkmux_wait(sysclk_is_pll1, true);
	if (rc != 0) {
		return rc;
	}

	/* Arm the hardware failsafe only once the reference has proven itself
	 * good enough to lock a PLL. Set-only in hardware; re-arming an
	 * already-armed CSS is a no-op. */
	LL_RCC_HSE_EnableCSS();

	return 0;
}

int sts_clkmux_get(void)
{
	int val;

	if (!gpio_is_ready_dt(&mux_sel)) {
		return -ENODEV;
	}

	val = gpio_pin_get_dt(&mux_sel);
	return val;
}

/*
 * Recovery path. Called with interrupts already locked when the forward
 * sequence failed, and standalone after a CSS event. Puts the mux back on the
 * OCXO — which is always powered and always oscillating — and rebuilds the
 * clock tree around it.
 */
static int clkmux_fallback_to_ocxo(void)
{
	int rc;

	(void)gpio_pin_set_dt(&mux_sel, 0);
	clkmux_delay_us(1000U);

	rc = clkmux_reselect_hse();
	if (rc != 0) {
		return rc;
	}

	return clkmux_relock_pll();
}

int sts_clkmux_execute(const void *steps_v, size_t n_steps)
{
	const refsel_step_t *steps = steps_v;
	unsigned int key;
	uint32_t settle_ms = 0;
	int target = -1;
	bool want_park = false;
	bool want_unpark = false;
	int rc = 0;

	if (steps == NULL || n_steps == 0U) {
		return -EINVAL;
	}
	if (!gpio_is_ready_dt(&mux_sel)) {
		return -ENODEV;
	}

	/*
	 * Read the whole list before touching anything. refsel emits a fixed
	 * shape (PARK, BRIDGE, SET_MUX, WAIT, RESELECT_HSE, VERIFY_PLL, UNPARK,
	 * DONE); pre-scanning it means the interrupt-locked region contains
	 * only register work, and an unexpected action is rejected before the
	 * clock tree is disturbed rather than halfway through it.
	 */
	for (size_t i = 0; i < n_steps; i++) {
		switch (steps[i].act) {
		case REFSEL_ACT_BRIDGE_TO_HSI:
		case REFSEL_ACT_RESELECT_HSE:
		case REFSEL_ACT_VERIFY_PLL:
		case REFSEL_ACT_DONE:
		case REFSEL_ACT_NONE:
			break;
		case REFSEL_ACT_PARK_DISCIPLINE:
			want_park = true;
			break;
		case REFSEL_ACT_UNPARK_DISCIPLINE:
			want_unpark = true;
			break;
		case REFSEL_ACT_SET_MUX:
			target = (steps[i].arg == REFSEL_MUX_B) ? 1 : 0;
			break;
		case REFSEL_ACT_WAIT_SETTLE_MS:
			settle_ms = steps[i].arg;
			break;
		default:
			LOG_ERR("clkmux: unknown action %u", (unsigned int)steps[i].act);
			return -ENOTSUP;
		}
	}

	if (target < 0) {
		LOG_ERR("clkmux: action list has no SET_MUX");
		return -EINVAL;
	}

	if (settle_ms > CONFIG_STS1000_CLKMUX_MAX_SETTLE_MS) {
		LOG_WRN("clkmux: settle %u ms clamped to %u ms", settle_ms,
			(unsigned int)CONFIG_STS1000_CLKMUX_MAX_SETTLE_MS);
		settle_ms = CONFIG_STS1000_CLKMUX_MAX_SETTLE_MS;
	}

	/*
	 * Park the discipline loop before the bridge and unpark after it
	 * (ARCHITECTURE.md §3.5). Both are pure disc-context state changes done
	 * outside the irq_lock: the DAC holds its last code through the flip,
	 * and disc_unpark() resumes in RECOVERING so the phase realignment the
	 * bridge introduces is ramped in, never stepped. Skipping this would
	 * demote the server on a healthy handoff.
	 */
	if (want_park) {
		sts_disc_handoff_park();
	}

	key = irq_lock();

	rc = clkmux_bridge_to_hsi();
	if (rc == 0) {
		(void)gpio_pin_set_dt(&mux_sel, target);
		clkmux_delay_us(settle_ms * 1000U);

		rc = clkmux_reselect_hse();
		if (rc == 0) {
			rc = clkmux_relock_pll();
		}
	}

	if (rc != 0 && target != 0) {
		/* The new reference did not come up. The OCXO is the one
		 * source that is always there; go back to it before releasing
		 * interrupts so the rest of the system never runs on HSI. */
		if (clkmux_fallback_to_ocxo() == 0) {
			rc = -EIO; /* handoff failed, clock tree recovered */
		} else {
			rc = -ENODEV; /* still on HSI: everything runs slow */
		}
	}

	irq_unlock(key);

	/*
	 * Unpark whenever the list asked for it, on success or failure: a
	 * handoff that fell back to the OCXO still left the loop parked, and
	 * leaving it parked would freeze the DAC indefinitely. disc_unpark()
	 * resumes in RECOVERING either way.
	 */
	if (want_unpark) {
		sts_disc_handoff_unpark();
	}

	if (rc == 0) {
		LOG_INF("clkmux: MUX_SEL -> %s, SYSCLK re-locked at %u Hz",
			(target != 0) ? "B (ext/Rb)" : "A (OCXO)", clkmux_sysclk_hz());
	} else {
		LOG_ERR("clkmux: handoff to %s failed (%d); SYSCLK %u Hz",
			(target != 0) ? "B" : "A", rc, clkmux_sysclk_hz());
	}

	return rc;
}

int sts_clkmux_recover(void)
{
	unsigned int key;
	int rc;

	if (!gpio_is_ready_dt(&mux_sel)) {
		return -ENODEV;
	}

	key = irq_lock();
	rc = clkmux_fallback_to_ocxo();
	irq_unlock(key);

	LOG_WRN("clkmux: CSS recovery to OCXO %s (SYSCLK %u Hz)",
		(rc == 0) ? "succeeded" : "FAILED", clkmux_sysclk_hz());

	return rc;
}

/* ========================================================================= */
/* EXTREF_MON — TIM12_CH1 (PB14) frequency measurement                       */
/* ========================================================================= */

/*
 * Method: TIM12 runs in external-clock mode 1 with TI1FP1 as the clock, so its
 * counter increments once per rising edge on PB14. The counter is 16-bit and
 * therefore wraps every 6.55 ms at 10 MHz; the update interrupt (152 Hz at
 * nominal, ~0.005 % CPU) extends it to 32 bits.
 *
 * The gate is measured with TIM2, not with the kernel tick. A 1 s gate timed
 * by k_uptime_get() at the default 100 Hz tick would carry +-10 ms of gate
 * error, i.e. +-1 %, or +-100 kHz on a 10 MHz signal — three orders of
 * magnitude worse than the +-20 Hz band refsel guards on. TIM2 resolves the
 * gate to 4 ns, so the measurement is limited by the +-1 edge count instead:
 * about 1 Hz over a 1 s gate.
 *
 * What is being measured is input B *relative to the current system
 * reference*, since TIM2 is clocked from it. That is the physically meaningful
 * comparison: refsel is deciding whether B is close enough to the reference
 * already in use to hand over to.
 */

#define EXTREF_IRQ_PRIO 2

static struct {
	uint32_t wraps;       /* TIM12 update events, ISR-owned */
	uint32_t last_count;  /* extended TIM12 count at the last sample */
	uint32_t last_gate;   /* TIM2 count at the last sample */
	uint32_t freq_hz;
	bool have_last;
	bool valid;
	bool edges;
	bool started;
} extref;

static void extref_isr(const void *arg)
{
	ARG_UNUSED(arg);

	if (LL_TIM_IsActiveFlag_UPDATE(extref_tim)) {
		LL_TIM_ClearFlag_UPDATE(extref_tim);
		extref.wraps++;
	}
}

/** Extended 32-bit edge count. Must be called with interrupts locked. */
static uint32_t extref_count_locked(void)
{
	uint32_t wraps = extref.wraps;
	uint32_t cnt = (uint16_t)LL_TIM_GetCounter(extref_tim);

	/*
	 * If an update is pending but the ISR has not run (we are inside an
	 * irq_lock), the wrap it represents is not yet in `wraps` while `cnt`
	 * has already rolled over. Fold it in by hand rather than returning a
	 * count that goes backwards.
	 */
	if (LL_TIM_IsActiveFlag_UPDATE(extref_tim) && cnt < 0x8000U) {
		wraps++;
	}

	return (wraps << 16) | cnt;
}

int sts_extref_mon_init(void)
{
	const struct device *pwm_dev = DEVICE_DT_GET(EXTREF_PWM_NODE);

	if (extref.started) {
		return -EALREADY;
	}

	if (!device_is_ready(pwm_dev)) {
		LOG_ERR("EXTREF_MON: TIM12 not ready");
		return -ENODEV;
	}

	LL_TIM_DisableCounter(extref_tim);
	LL_TIM_SetPrescaler(extref_tim, 0U);
	LL_TIM_SetAutoReload(extref_tim, UINT16_MAX);
	LL_TIM_SetCounter(extref_tim, 0U);

	/*
	 * CH1 as a direct input with the maximum digital filter (fDTS/32,
	 * N = 8). The slicer output is a clean CMOS edge, but PB14 sits next
	 * to the RMII TX pair and the filter costs nothing at a 10 MHz input
	 * that is being counted, not timestamped.
	 */
	LL_TIM_IC_SetActiveInput(extref_tim, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
	LL_TIM_IC_SetPrescaler(extref_tim, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
	LL_TIM_IC_SetFilter(extref_tim, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
	LL_TIM_IC_SetPolarity(extref_tim, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);

	LL_TIM_SetTriggerInput(extref_tim, LL_TIM_TS_TI1FP1);
	LL_TIM_SetClockSource(extref_tim, LL_TIM_CLOCKSOURCE_EXT_MODE1);

	LL_TIM_GenerateEvent_UPDATE(extref_tim);
	LL_TIM_ClearFlag_UPDATE(extref_tim);

	IRQ_CONNECT(DT_IRQ_BY_NAME(EXTREF_TIMER_NODE, global, irq), EXTREF_IRQ_PRIO,
		    extref_isr, NULL, 0);
	irq_enable(DT_IRQ_BY_NAME(EXTREF_TIMER_NODE, global, irq));

	LL_TIM_EnableIT_UPDATE(extref_tim);
	LL_TIM_EnableCounter(extref_tim);

	extref.started = true;
	LOG_INF("EXTREF_MON up: TIM12_CH1/PB14 external-clock mode, TIM2-gated");

	return 0;
}

/**
 * Close the current gate and open the next one. Call at a steady ~1 Hz from
 * the housekeeping thread; the gate length is measured, not assumed, so
 * jitter in the call rate costs nothing.
 */
void sts_extref_mon_sample(void)
{
	uint32_t count;
	uint32_t gate;
	uint32_t d_count;
	uint32_t d_gate;
	uint32_t tim_hz = sts_pps_timer_hz();
	unsigned int key;

	if (!extref.started || tim_hz == 0U) {
		return;
	}

	key = irq_lock();
	count = extref_count_locked();
	gate = LL_TIM_GetCounter(pps_tim);
	irq_unlock(key);

	if (!extref.have_last) {
		extref.last_count = count;
		extref.last_gate = gate;
		extref.have_last = true;
		return;
	}

	d_count = count - extref.last_count;
	d_gate = gate - extref.last_gate;

	extref.last_count = count;
	extref.last_gate = gate;

	/*
	 * A gate shorter than 100 ms cannot resolve the +-20 Hz band (the
	 * +-1-count quantisation alone would be 10 Hz), and one longer than
	 * 8 s risks a TIM2 wrap being mistaken for a short gate. Outside that
	 * window the sample is discarded rather than reported as a frequency.
	 */
	if (d_gate < (tim_hz / 10U) || d_gate > (tim_hz * 8U)) {
		extref.valid = false;
		return;
	}

	extref.edges = (d_count != 0U);
	extref.freq_hz = (uint32_t)(((uint64_t)d_count * (uint64_t)tim_hz) / (uint64_t)d_gate);
	extref.valid = true;
}

void sts_extref_mon_read(uint32_t *hz, bool *valid, bool *edges)
{
	unsigned int key = irq_lock();

	if (hz != NULL) {
		*hz = extref.freq_hz;
	}
	if (valid != NULL) {
		*valid = extref.valid;
	}
	if (edges != NULL) {
		*edges = extref.edges;
	}

	irq_unlock(key);
}

/* ========================================================================= */

int sts_clkmux_init(void)
{
	int rc;

	if (!gpio_is_ready_dt(&mux_sel)) {
		LOG_ERR("MUX_SEL: GPIO port not ready");
		return -ENODEV;
	}

	/*
	 * Drive the boot default explicitly. R188 (100 k pull-down) has held
	 * PB6 at input A since reset; taking ownership of the pin without
	 * changing its level keeps the OCXO selected.
	 */
	rc = gpio_pin_configure_dt(&mux_sel, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		LOG_ERR("MUX_SEL: configure failed (%d)", rc);
		return rc;
	}

#if defined(CONFIG_RUNTIME_NMI)
	z_arm_nmi_set_handler(clkmux_nmi_handler);
#else
#warning "CONFIG_RUNTIME_NMI is off: a CSS event will reset the board"
#endif

	/*
	 * Arm CSS on the boot reference. Everything downstream assumes a live
	 * PH0; if the OCXO ever stops, the hardware failover to HSI plus the
	 * NMI latch is what turns that into a reported fault instead of a hang.
	 */
	LL_RCC_HSE_EnableCSS();

	LOG_INF("clock mux: MUX_SEL=A (OCXO), CSS armed, SYSCLK %u Hz",
		clkmux_sysclk_hz());

	return 0;
}
