/*
 * STS1000 "Meridian" — the fan actuator and the thermal ladder's actuator side.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/platform/, and deliberately free of every Zephyr
 * dependency so tests/host can compile it — the same arrangement as
 * net/sts_ppscorr.h, for the same reason.
 *
 * ---------------------------------------------------------------------------
 * The invariant this file exists to make testable
 * ---------------------------------------------------------------------------
 * ARCHITECTURE.md §10.9 and firmware/CLAUDE.md both state it as a hard rule:
 *
 *     "Fan fail-safe to cooling: 4-wire fan runs full speed on float/100 % PWM.
 *      Keep FAN_PWM (PE5/TIM15_CH1) resting state = max airflow so a hung MCU
 *      can't cook the box."
 *
 * Three separate paths have to honour it — the pre-loop state at hk start, the
 * loop's own refusal path, and the duty-to-pulse conversion that turns 100 %
 * into a pulse width. The rule was asserted by no test at all, and the third
 * path is the quiet one: TIM15_CH1 drives a fan whose speed rises with duty, so
 * a conversion that clamps, rounds or scales 100 % to anything short of the
 * full period is a box that runs permanently slightly cool-of-target and only
 * discovers it at 80 °C. STS_FAN_DUTY_RESTING_PCT and sts_fan_pulse_ns() are
 * here so one test pins all three.
 *
 * ---------------------------------------------------------------------------
 * What is decided here and what is not
 * ---------------------------------------------------------------------------
 * core/thermal owns the PI loop, the ladder thresholds and the stall detector
 * (tests/host/test_thermal.c). This header owns what housekeeping does with the
 * answer: the actuator conversion, the refusal path, the alarm mapping, the
 * maintenance override's resolution against the loop, and the latching of rungs
 * 2 and 3 for the 4 Hz pwrseq tick to pick up. That last part is the one with
 * history — nothing consumed request_rb_shed at all, so a stalled fan past the
 * kill threshold lit an LED and sent a trap while the ~12 W rubidium kept
 * running and the board never cold-cycled.
 *
 * The override resolution is here for the same reason as the resting duty: it
 * carries the §10.9 invariant on the one path that has no other carrier. While
 * it lived in hk.c — Zephyr glue, which does not link into a host suite — the
 * release rule could be deleted outright and the whole suite stayed green.
 *
 * pwm_set(), the tach ISR and the k_mutex-protected cache stay in hk.c.
 */

#ifndef STS1000_ZEPHYR_PLATFORM_STS_FAN_POLICY_H_
#define STS1000_ZEPHYR_PLATFORM_STS_FAN_POLICY_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "fault/fault.h"
#include "thermal/thermal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * PWM carrier, 25 kHz.
 *
 * Above audibility and inside the Intel 4-wire fan specification's 21-28 kHz
 * band, so the fan's own controller reads the duty rather than chopping its
 * commutation.
 */
#define STS_FAN_PWM_HZ 25000U

/** Carrier period in nanoseconds. Exact at 25 kHz: 40 000 ns. */
#define STS_FAN_PWM_PERIOD_NS (1000000000U / STS_FAN_PWM_HZ)

/**
 * The resting duty: full airflow.
 *
 * Not "the maximum the loop may command" — the state the pin is left in
 * whenever firmware has no live opinion, which is at start-up before the loop
 * has run and on every path where the loop declines to answer.
 */
#define STS_FAN_DUTY_RESTING_PCT 100U

/** Tach pulses per revolution on a standard 4-wire fan. */
#define STS_FAN_TACH_PULSES_PER_REV 2U

/**
 * Duty percent to TIM15_CH1 pulse width.
 *
 * Clamps rather than rejects: a caller that computes 137 % from a broken sensor
 * must get maximum airflow, not a rejected write that leaves the previous duty
 * in the compare register.
 */
static inline uint32_t sts_fan_pulse_ns(uint8_t duty_pct)
{
	if (duty_pct > 100U) {
		duty_pct = 100U;
	}

	return ((uint32_t)duty_pct * STS_FAN_PWM_PERIOD_NS) / 100U;
}

/* ------------------------------------------------------- ladder actuation -- */

/** What one 1 Hz thermal step asks housekeeping to actuate. */
typedef struct {
	/** Program this duty into TIM15_CH1. Always set. */
	uint8_t duty_pct;

	/**
	 * The loop declined to answer, so @p duty_pct is the fail-safe rather
	 * than a regulated value, the alarms below are not to be applied, and
	 * the cached telemetry must not be updated with a value nothing
	 * measured.
	 */
	bool loop_failed;

	/** Apply the three alarm bits. False when @p loop_failed. */
	bool apply_alarms;
	bool alarm_thermal_warn;     /**< FAULT_ALARM_THERMAL_WARN, rung 1 */
	bool alarm_thermal_critical; /**< FAULT_ALARM_THERMAL_CRITICAL, rung 3 */
	bool alarm_fan_fault;        /**< FAULT_ALARM_FAN_FAULT */

	/** Latch the escalation requests for the 4 Hz pwrseq tick. */
	bool latch_escalation;
	bool request_rb_shed;  /**< rung 2 — pwrseq drops RB_PWR_EN */
	bool request_poe_kill; /**< rung 3 — pwrseq asserts POE_KILL */
} sts_fan_action_t;

/**
 * Turn one thermal_step_1hz() result into an actuator plan.
 *
 * @param step_rc  thermal_step_1hz()'s return value.
 * @param out_t    Its output. May be NULL only when @p step_rc is non-zero.
 * @param act      Never NULL; always fully written.
 *
 * On a refusal the fan goes to the resting duty and NOTHING else changes: the
 * alarms keep whatever the last successful step set, and the escalation latches
 * keep whatever pwrseq is already acting on. Clearing them here would be
 * strictly worse than leaving them — a loop that will not step has no evidence
 * the box has cooled, and dropping request_poe_kill on the way to full speed
 * would cancel an escalation that a real over-temperature raised.
 */
static inline void sts_fan_policy_eval(int step_rc, const thermal_out_t *out_t,
				       sts_fan_action_t *act)
{
	memset(act, 0, sizeof(*act));

	if (step_rc != 0 || out_t == NULL) {
		/* A loop that will not step is a loop that is not cooling. */
		act->duty_pct = STS_FAN_DUTY_RESTING_PCT;
		act->loop_failed = true;
		return;
	}

	act->duty_pct = out_t->duty_pct;

	act->apply_alarms = true;
	act->alarm_thermal_warn = out_t->alarm_overtemp;
	/*
	 * THERMAL_CRITICAL tracks rung 3, the POE_KILL request — not rung 2.
	 * Rung 2 (shed the rubidium) is a load-management action that leaves the
	 * grandmaster serving on the OCXO, so annunciating it as critical would
	 * disqualify the holdover relay (it is in FAULT_RELAY_DISQUALIFY_DEFAULT)
	 * for a box that is still perfectly traceable.
	 */
	act->alarm_thermal_critical = out_t->request_poe_kill;
	act->alarm_fan_fault = out_t->fan_stall;

	act->latch_escalation = true;
	act->request_rb_shed = out_t->request_rb_shed;
	act->request_poe_kill = out_t->request_poe_kill;
}

/** One alarm the plan raises. @p id is a fault_alarm_id_t. */
typedef struct {
	uint8_t id;
	bool active;
} sts_fan_alarm_t;

/**
 * The three alarms an actuator plan raises, in a fixed order.
 *
 * @param act  A plan from sts_fan_policy_eval().
 * @param out  Array of at least 3 entries.
 * @return Number written: 3 when @p act->apply_alarms, else 0.
 */
static inline size_t sts_fan_alarms(const sts_fan_action_t *act,
				    sts_fan_alarm_t *out)
{
	if (!act->apply_alarms) {
		return 0U;
	}

	out[0].id = (uint8_t)FAULT_ALARM_THERMAL_WARN;
	out[0].active = act->alarm_thermal_warn;
	out[1].id = (uint8_t)FAULT_ALARM_THERMAL_CRITICAL;
	out[1].active = act->alarm_thermal_critical;
	out[2].id = (uint8_t)FAULT_ALARM_FAN_FAULT;
	out[2].active = act->alarm_fan_fault;

	return 3U;
}

/* ------------------------------------------------- maintenance override -- */

/**
 * A maintenance lease held over the fan.
 *
 * `pct` is a FLOOR and not a level. A technician can raise the fan to prove an
 * airflow path or to pre-cool a box before a long test, and can never hold it
 * below what the thermal loop is asking for as the enclosure heats up. Zero and
 * inactive are the same thing to sts_fan_resolve(); `pct` is cleared on release
 * anyway so a lapsed lease cannot leave a stale floor behind it.
 */
typedef struct {
	bool active;
	uint8_t pct;
} sts_fan_ovr_t;

/**
 * The duty TIM15_CH1 must hold: max(thermal loop, override floor).
 *
 * @param loop_pct  The loop's OWN last answer — including the fail-safe duty
 *                  sts_fan_policy_eval() commands on a step it declined, which
 *                  is what keeps a stalled loop from being quietly undercut by
 *                  a lease taken out before it stalled.
 * @param ovr       The held lease, or NULL for "none".
 */
static inline uint8_t sts_fan_resolve(uint8_t loop_pct, const sts_fan_ovr_t *ovr)
{
	if (ovr != NULL && ovr->active && ovr->pct > loop_pct) {
		return ovr->pct;
	}

	return loop_pct;
}

/**
 * Take or release the lease, and return the duty to program right now.
 *
 * Storing the lease and resolving the duty are ONE decision and are taken here
 * together, because the two halves are only correct with respect to each other:
 *
 *  - A TAKE resolves to max(loop, floor) — the lease raises the fan and never
 *    lowers it. The requested level is clamped to full airflow rather than
 *    rejected, on the same reasoning as sts_fan_pulse_ns().
 *
 *  - A RELEASE resolves to STS_FAN_DUTY_RESTING_PCT — full airflow — and NOT
 *    to the loop's last answer. ARCHITECTURE.md §10.9 fixes the resting state
 *    at maximum airflow for every moment firmware has no live opinion, and a
 *    lease that has just lapsed is exactly such a moment: the loop's answer is
 *    up to a second stale and, on a step the loop declined, is not a
 *    measurement of anything. The next 1 Hz step re-establishes the regulated
 *    duty, so the cost is bounded at one second of a fan running too fast,
 *    against the alternative of a fan running too slow in a box nobody is
 *    watching.
 *
 * @param ovr       Lease state, updated in place. Never NULL.
 * @param active    True to take/replace the lease, false to release it.
 * @param pct       Requested floor. Ignored when @p active is false.
 * @param loop_pct  The thermal loop's own last answer.
 * @return The duty to program into TIM15_CH1.
 */
static inline uint8_t sts_fan_ovr_apply(sts_fan_ovr_t *ovr, bool active, uint8_t pct,
					uint8_t loop_pct)
{
	ovr->active = active;
	ovr->pct = active ? ((pct > 100U) ? 100U : pct) : 0U;

	if (!active) {
		return STS_FAN_DUTY_RESTING_PCT;
	}

	return sts_fan_resolve(loop_pct, ovr);
}

/* ------------------------------------------------------------------ tach -- */

/**
 * Tach edges over an interval, as RPM.
 *
 * @param edges  Falling edges counted since the last call.
 * @param dt_ms  Elapsed milliseconds. Zero yields zero — a zero-length interval
 *               carries no rate, and dividing by it would be the fault report
 *               rather than the measurement.
 *
 * Two pulses per revolution and one counted edge per pulse, so
 * RPM = edges * 60000 / (dt_ms * 2). Reported as 0 rather than saturating on an
 * implausible count: core/thermal's stall detector treats a low reading as a
 * stall, which is the fail-safe direction.
 */
static inline uint32_t sts_fan_tach_rpm(uint32_t edges, uint32_t dt_ms)
{
	if (dt_ms == 0U) {
		return 0U;
	}

	return (uint32_t)(((uint64_t)edges * 60000ULL) /
			  ((uint64_t)dt_ms * (uint64_t)STS_FAN_TACH_PULSES_PER_REV));
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_PLATFORM_STS_FAN_POLICY_H_ */
