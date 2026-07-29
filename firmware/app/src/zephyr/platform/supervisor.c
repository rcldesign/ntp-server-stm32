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
 * The watchdog cadence, and why it has its own thread
 * ---------------------------------------------------------------------------
 * The TPS3430 window is 920-1360 ms between WDI falling edges: a kick that is
 * too *early* is as fatal as one that is too late (docs/sts1000_external_wdt.md
 * §4). Anything faster than tWDL(min) = 680 ms is a RUNAWAY fault, which drives
 * WDO_N for ~200 ms, which drives POE_KILL, which drops the board's PoE port.
 *
 * Two consequences shape this file:
 *
 *   1. The cadence must be enforced by the *decision*, never by the call rate.
 *      pwrseq_wdt_service() owns it; this file only drives the pin when that
 *      says so. A previous version kicked once per housekeeping tick, which at
 *      HK_PERIOD_MS = 250 ms is a runaway fault on every single tick — the board
 *      cold-cycled itself a quarter-second after arming, forever.
 *   2. The kicker must not be the lowest-priority thread on the board. It used
 *      to run inside housekeeping (priority 14) — which is also a liveness
 *      *participant* — so a housekeeping tick that merely ran slow could miss
 *      the late boundary and cause the hardware power cycle the watchdog exists
 *      to trigger only on a genuine hang. The kicker therefore runs on its own
 *      thread at a cooperative priority, where no application thread can
 *      preempt it, and it *consumes* the liveness mask instead of feeding it.
 *
 * Liveness stays an AND-gate: silence is the safe answer. If any registered
 * participant is late the kick is withheld and the watchdog is allowed to do
 * its job.
 */

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "zephyr/platform/platform.h"
#include "zephyr/platform/sts_super_policy.h"
#include "zephyr/sts_app.h"

#include "pwrseq/pwrseq.h"

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

/* ---------------------------------------------------------------- kicker -- */

#define WDT_STACK_SIZE 768
/*
 * Cooperative priority: unpreemptable by every application thread, so no amount
 * of network or UI load can delay a kick past the window's late boundary. Above
 * the system work queue (-1) because that queue runs the NVS fast-save.
 */
#define WDT_PRIO       (-8)
/* Poll rate. Far finer than the ~1.1 s cadence so the decision's own latency is
 * a rounding error against the 440 ms-wide window. */
#define WDT_TICK_MS    50

K_THREAD_STACK_DEFINE(wdt_stack, WDT_STACK_SIZE);
static struct k_thread wdt_tcb;

static struct {
	bool armed;         /* WDT_EN asserted (stage 9 complete) */
	bool seq_eligible;  /* pwrseq reached RELAY_ELIGIBLE (stage 9) */
	bool relay_on;
	uint32_t last_stale_mask;
	fault_rgb_state_t rgb;
	uint32_t identify_until_ms;
	bool ready;
	bool wdt_thread_started;

	/*
	 * Owned by the kicker thread once it starts; written before that only by
	 * sts_supervisor_arm()/_kick_start(), both of which run in the
	 * housekeeping thread on the pwrseq action drain. The kicker only ever
	 * reads `armed` through pwrseq_wdt_service(), and a 32-bit aligned store
	 * on Cortex-M33 is atomic, so no lock is needed for a struct one thread
	 * mutates at arm time and another mutates thereafter.
	 */
	pwrseq_wdt_t wdt;
	/* Snapshot of the kicker's violation counter, so the housekeeping thread can
	 * log it (the kicker must not, it holds a cooperative priority). */
	uint32_t logged_violations;

	/*
	 * The D5 maintenance override, in two independently-leased layers.
	 *
	 * `rgb_ovr.mode` forces a whole PATTERN (STS_RGB_MODE_AUTO = none held,
	 * hand D5 back to core/fault's priority encode); the leg entries hold
	 * individual duties AFTER the pattern has resolved to a colour, so one leg
	 * can be held without taking the indicator off the fault policy. How the
	 * two layers resolve is sts_super_policy.h's, not this file's — a decision
	 * taken here is one no host suite can execute.
	 *
	 * These are STATE the 4 Hz pass consults, not writes anyone else performs:
	 * this file is the single writer of PD12/PD13/PD14 and re-drives them on
	 * its own pass, so a foreign write would be undone within 250 ms. The
	 * setters are housekeeping-thread-only for the same reason.
	 *
	 * `leg_out` and `shown_mode` are the other direction — what was ACTUALLY
	 * programmed, so a read-back reports the indicator rather than the request.
	 */
	sts_super_rgb_ovr_t rgb_ovr;
	uint8_t leg_out[STS_RGB_LEG_COUNT];
	uint8_t shown_mode;
} super;

void sts_supervisor_set_seq_eligible(bool eligible)
{
	super.seq_eligible = eligible;
}

/* ------------------------------------------------------------------ RGB -- */

/*
 * The manifest's `ui.rgb.mode` values are the wire enum; sts_super_policy.h
 * restates them so the policy — which owns the fault-encode-to-pattern map, the
 * colour table and the leg composition — keeps a leaf dependency set. The two
 * are one enum in two places, so they are asserted equal here: drift stops the
 * build instead of turning amber into red on a technician's screen.
 */
BUILD_ASSERT((int)STS_RGB_MODE_AUTO == (int)STS_SUPER_RGB_MODE_AUTO, "rgb enum drift");
BUILD_ASSERT((int)STS_RGB_MODE_OFF == (int)STS_SUPER_RGB_MODE_OFF, "rgb enum drift");
BUILD_ASSERT((int)STS_RGB_MODE_GREEN == (int)STS_SUPER_RGB_MODE_GREEN, "rgb enum drift");
BUILD_ASSERT((int)STS_RGB_MODE_AMBER == (int)STS_SUPER_RGB_MODE_AMBER, "rgb enum drift");
BUILD_ASSERT((int)STS_RGB_MODE_RED == (int)STS_SUPER_RGB_MODE_RED, "rgb enum drift");
BUILD_ASSERT((int)STS_RGB_MODE_BLUE_PULSE == (int)STS_SUPER_RGB_MODE_BLUE_PULSE,
	     "rgb enum drift");
BUILD_ASSERT((int)STS_RGB_MODE_COUNT == (int)STS_SUPER_RGB_MODE_COUNT, "rgb enum drift");
BUILD_ASSERT((int)STS_RGB_LEG_R == (int)STS_SUPER_RGB_LEG_R, "rgb leg drift");
BUILD_ASSERT((int)STS_RGB_LEG_G == (int)STS_SUPER_RGB_LEG_G, "rgb leg drift");
BUILD_ASSERT((int)STS_RGB_LEG_B == (int)STS_SUPER_RGB_LEG_B, "rgb leg drift");
BUILD_ASSERT((int)STS_RGB_LEG_COUNT == (int)STS_SUPER_RGB_LEG_COUNT, "rgb leg drift");

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
	super.leg_out[STS_RGB_LEG_R] = r_pct;
	super.leg_out[STS_RGB_LEG_G] = g_pct;
	super.leg_out[STS_RGB_LEG_B] = b_pct;
}

/**
 * Resolve, then program D5. Housekeeping-thread context.
 *
 * The resolution itself — forced pattern over the fault encode, that pattern's
 * three duties, then the leg holds on top — is sts_super_policy.h's and is
 * covered by tests/host/test_super_policy.c. `shown_mode` records the pattern
 * that was actually resolved — never STS_RGB_MODE_AUTO, because "auto" is a
 * source and not something D5 can show.
 */
static void rgb_program(uint32_t now_ms)
{
	sts_super_rgb_out_t r;

	sts_super_rgb_resolve(&super.rgb_ovr, super.rgb, now_ms, &r);

	super.shown_mode = r.mode;
	rgb_set(r.duty[STS_RGB_LEG_R], r.duty[STS_RGB_LEG_G], r.duty[STS_RGB_LEG_B]);
}

int sts_supervisor_rgb_mode(uint8_t mode)
{
	if (mode >= STS_RGB_MODE_COUNT) {
		return -EINVAL;
	}
	if (!super.ready) {
		return -ENODEV;
	}

	/* STS_RGB_MODE_AUTO is the release: the stored override goes and the next
	 * resolve falls through to core/fault's encode. Applied immediately so
	 * the caller's "applied" outcome is true of the pin, not of the next
	 * 4 Hz pass. */
	super.rgb_ovr.mode = mode;
	rgb_program(k_uptime_get_32());

	return 0;
}

int sts_supervisor_rgb_leg(uint8_t leg, bool active, uint8_t pct)
{
	if (leg >= STS_RGB_LEG_COUNT) {
		return -EINVAL;
	}
	if (!super.ready) {
		return -ENODEV;
	}

	(void)sts_super_rgb_leg_set(&super.rgb_ovr, leg, active, pct);
	rgb_program(k_uptime_get_32());

	return 0;
}

uint8_t sts_supervisor_rgb_mode_get(void)
{
	/* Before the first pass the honest answer is OFF, which is what
	 * sts_supervisor_init() drives — not AUTO, which is not a colour. */
	if (!super.ready || (super.shown_mode == STS_RGB_MODE_AUTO)) {
		return (uint8_t)STS_RGB_MODE_OFF;
	}
	return super.shown_mode;
}

uint8_t sts_supervisor_rgb_leg_get(uint8_t leg)
{
	if (leg >= STS_RGB_LEG_COUNT) {
		return 0U;
	}
	return super.leg_out[leg];
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
	/* Three gates: pwrseq has reached stage 9 (seq_eligible), no
	 * disqualifying alarm is active (eligible), and the clock is actually
	 * serving. The sequence gate is what keeps K2 de-energized through
	 * bring-up even if the OCXO locks early. sts_super_policy.h holds the
	 * conjunction so it is asserted rather than reasoned about. */
	bool want = sts_super_relay_want(super.seq_eligible, eligible, q);

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
}

/*
 * The liveness AND-gate, mapped onto pwrseq's three-bit model.
 *
 * The platform's registry is the authority on who must be alive: every area
 * registers its own participants by name, so the count is not fixed at three.
 * A single late participant therefore clears the whole mask — which is the
 * AND-gate the window exists to enforce — and pwrseq_wdt_service() withholds.
 */
static uint32_t wdt_liveness(uint32_t now_ms)
{
	return sts_super_wdt_liveness(sts_liveness_stale_mask(now_ms));
}

static void wdt_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		pwrseq_wdt_tick_t t;
		uint32_t now;

		k_msleep(WDT_TICK_MS);

		if (!super.armed) {
			continue;
		}

		now = k_uptime_get_32();
		if (pwrseq_wdt_service(&super.wdt, wdt_liveness(now), now, &t) != 0) {
			continue;
		}
		if (t.kick) {
			wdt_kick_once();
		}
	}
}

int sts_supervisor_wdt_start(void)
{
	k_tid_t tid;

	if (!super.ready) {
		return -ENODEV;
	}
	if (super.wdt_thread_started) {
		return -EALREADY;
	}

	/* Configured now, armed later: the thread spins harmlessly until stage 9
	 * asserts WDT_EN, and the cadence is validated here rather than at the
	 * first kick. */
	if (pwrseq_wdt_init(&super.wdt, PWRSEQ_WDT_KICK_PERIOD_MS) != 0) {
		LOG_ERR("WDT cadence %u ms is outside the TPS3430 window %u-%u ms",
			PWRSEQ_WDT_KICK_PERIOD_MS, PWRSEQ_WDT_WINDOW_MIN_MS,
			PWRSEQ_WDT_WINDOW_MAX_MS);
		return -EINVAL;
	}

	tid = k_thread_create(&wdt_tcb, wdt_stack, WDT_STACK_SIZE, wdt_entry, NULL,
			      NULL, NULL, WDT_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(tid, "wdt_kick");

	super.wdt_thread_started = true;

	LOG_INF("WDT kicker up: %u ms cadence, %u ms poll, priority %d",
		PWRSEQ_WDT_KICK_PERIOD_MS, WDT_TICK_MS, WDT_PRIO);

	return 0;
}

void sts_supervisor_kick_start(uint32_t now_ms)
{
	/*
	 * PWRSEQ_ACT_WDT_KICK_START. sts_supervisor_arm() already seeded the
	 * cadence from its own pre-arm kick; this only re-seeds if the two actions
	 * did not land in the same drain, and re-seeding can only ever make the
	 * next kick later, never earlier.
	 */
	if (super.armed && !pwrseq_wdt_is_armed(&super.wdt)) {
		(void)pwrseq_wdt_arm(&super.wdt, now_ms);
	}
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
	if (!super.wdt_thread_started) {
		LOG_ERR("refusing to arm WDT_EN with no kicker thread");
		return -ENODEV;
	}

	/*
	 * Kick once immediately before arming so the first window opens with the
	 * watchdog already fed, and seed the cadence from that same instant — so
	 * the kicker's first serviced kick lands one full period later rather than
	 * on its next 50 ms poll, which would be a runaway fault.
	 */
	wdt_kick_once();
	(void)pwrseq_wdt_arm(&super.wdt, k_uptime_get_32());

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
/*
 * ---------------------------------------------------------------------------
 * The maintenance seam: `sys.wdt.en` and `sys.wdt.kick`
 * ---------------------------------------------------------------------------
 * Both exist so a technician can scope the PB2 edge, and both are written here
 * rather than as raw gpio_pin_set_dt() calls from the console thread, because
 * the pin is only half of what has to move.
 *
 * WDT_EN carries a CADENCE with it. sts_supervisor_arm() above kicks once and
 * seeds pwrseq's window from that same instant precisely so the first window
 * opens already fed; a bare gpio_pin_set_dt(&wdt_en, 1) on override release
 * would raise WDT_EN mid-cadence with no seed, and the kicker's next serviced
 * kick would land on its 50 ms poll rather than a full period later — a runaway
 * fault, the same class the file banner describes. So re-enabling REPLAYS that
 * sequence.
 *
 * Disabling clears `armed` as well as dropping the pin. That is not bookkeeping:
 * wdt_entry() polls `armed` and pwrseq_wdt_service() is what decides a kick, so
 * leaving it set would have the kicker keep driving WDI at cadence into a
 * TPS3430 that is not watching — harmless in itself, but it also means the
 * re-arm's own seed would be fighting a window pwrseq still believed was open.
 */

int sts_supervisor_wdt_state(bool *armed)
{
	if (armed == NULL) {
		return -EINVAL;
	}

	/*
	 * Reported unconditionally, including on the failure path: an
	 * uninitialised supervisor has no PC12 to read and must still answer
	 * "watching", so a caller that reads the out-param without the errno
	 * cannot be told the watchdog is off. sts_super_wdt_state_armed() holds
	 * that reasoning and is the only place the direction is written down.
	 */
	*armed = sts_super_wdt_state_armed(super.ready, super.armed);

	return super.ready ? 0 : -ENODEV;
}

int sts_supervisor_wdt_enable(bool enable)
{
	int rc;

	if (!super.ready) {
		return -ENODEV;
	}

	if (!enable) {
		/*
		 * Order matters: stop the kicker first, then drop the pin. The
		 * reverse leaves a window in which WDI is still being driven at
		 * cadence with WDT_EN already low, which is the state the
		 * violation counter cannot distinguish from a second writer.
		 */
		super.armed = false;
		rc = gpio_pin_set_dt(&wdt_en, 0);
		if (rc != 0) {
			LOG_ERR("WDT_EN de-assert failed (%d)", rc);
			return rc;
		}
		sts_log(LOGR_SUB_SYS, LOGR_WARN,
			"external watchdog DISABLED by maintenance override");
		return 0;
	}

	if (super.armed) {
		return 0; /* idempotent, and re-seeding here would be a second
			   * cadence origin for a window already open */
	}
	if (!super.wdt_thread_started) {
		LOG_ERR("refusing to arm WDT_EN with no kicker thread");
		return -ENODEV;
	}

	/* The arm sequence, replayed — kick, seed from that same instant, then
	 * assert. See sts_supervisor_arm(). */
	wdt_kick_once();
	(void)pwrseq_wdt_arm(&super.wdt, k_uptime_get_32());

	rc = gpio_pin_set_dt(&wdt_en, 1);
	if (rc != 0) {
		LOG_ERR("WDT_EN assert failed (%d)", rc);
		return rc;
	}

	super.armed = true;
	sts_log(LOGR_SUB_SYS, LOGR_NOTICE,
		"external watchdog re-armed after maintenance override");
	return 0;
}

int sts_supervisor_wdt_kick(void)
{
	/*
	 * A single maintenance edge on WDI. Refused unless the watchdog is not
	 * watching — sts_super_wdi_pulse_allowed() holds the reasoning, and the
	 * console's MP_ILK_WDT_OFF refuses the same request one seam earlier so
	 * a technician gets a named interlock rather than an errno.
	 *
	 * The duplication is deliberate. This entry point is reachable from
	 * anything inside the image, and a permission that lives only in the
	 * caller is a permission a second caller does not have.
	 */
	if (!sts_super_wdi_pulse_allowed(super.ready, super.armed)) {
		return -EPERM;
	}

	wdt_kick_once();
	sts_log(LOGR_SUB_SYS, LOGR_NOTICE,
		"WDI pulsed by maintenance override (watchdog disabled)");
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

	/*
	 * The kick itself belongs to the dedicated thread; this only annunciates.
	 * Logging here rather than there is deliberate: the kicker holds a
	 * cooperative priority and sts_log() is not something to run from one.
	 */
	if (sts_super_stale_log(super.armed, stale, super.last_stale_mask)) {
		for (uint32_t i = 0; i < sts_liveness_count(); i++) {
			if ((stale & BIT(i)) != 0U) {
				sts_log(LOGR_SUB_SYS, LOGR_CRIT,
					"liveness lost: %s — WDT kick withheld",
					sts_liveness_name(i));
			}
		}
	}
	super.last_stale_mask = stale;

	/*
	 * A kick observed outside 920-1360 ms means the board is being cold-cycled
	 * by its own supervisor. That must be impossible by construction, so if it
	 * ever happens say so at CRIT with the measured interval rather than
	 * leaving an unexplained reboot loop for someone to reverse-engineer.
	 */
	if (sts_super_violation_new(super.wdt.violations, super.logged_violations)) {
		super.logged_violations = super.wdt.violations;
		sts_log(LOGR_SUB_SYS, LOGR_CRIT,
			"WDT kick %s the %u-%u ms window: %u total — the TPS3430 "
			"will drive WDO_N and POE_KILL",
			sts_super_violation_word(super.wdt.last_verdict),
			PWRSEQ_WDT_WINDOW_MIN_MS, PWRSEQ_WDT_WINDOW_MAX_MS,
			super.wdt.violations);
	}

	(void)sts_quality_snapshot(&q);

	sts_fault_lock();
	eligible = fault_relay_eligible(sts_fault());
	any_fault = fault_any_active(sts_fault());
	sts_fault_unlock();

	relay_apply(eligible, &q);

	sts_super_rgb_in(&q, any_fault,
			 sts_super_identify_active(super.identify_until_ms, now_ms),
			 &rgb_in);

	if (!rgb_in.identify) {
		super.identify_until_ms = 0U;
	}

	/*
	 * The fault policy's answer is recorded on every pass but only re-driven
	 * when it changed or when the pattern actually on D5 is the pulse, which
	 * has to be re-driven to pulse at all. An override's own writes happen in
	 * its setter, so a held mode or leg needs nothing here — and a change of
	 * fault state under a held mode simply resolves to the same duties.
	 */
	rgb_state = fault_rgb_state(&rgb_in);
	if (rgb_state != super.rgb) {
		super.rgb = rgb_state;
		rgb_program(now_ms);
	} else if (sts_super_rgb_mode_now(super.rgb_ovr.mode, super.rgb) ==
		   STS_SUPER_RGB_MODE_BLUE_PULSE) {
		rgb_program(now_ms);
	}

	/*
	 * Self-confirm the running image once the box is genuinely healthy.
	 * ARCHITECTURE.md §3: an unconfirmed image is reverted by MCUboot on
	 * the next boot, so confirming too early defeats the whole mechanism
	 * and confirming never means one reboot loses the update.
	 */
	if (sts_super_confirm_now(sts_update_pending_confirm(), stale, any_fault,
				  q.lock_state)) {
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
	super.rgb_ovr.mode = (uint8_t)STS_RGB_MODE_AUTO;
	rgb_program(0U);

	super.ready = true;
	return 0;
}

void sts_supervisor_counters(uint32_t *kicks, uint32_t *withheld, bool *armed)
{
	if (kicks != NULL) {
		*kicks = super.wdt.kicks;
	}
	if (withheld != NULL) {
		*withheld = super.wdt.withheld;
	}
	if (armed != NULL) {
		*armed = super.armed;
	}
}
