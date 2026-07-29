/*
 * STS1000 "Meridian" — the fan actuator and the thermal ladder's actuator side,
 * tested away from Zephyr.
 *
 * core/thermal decides the duty and the ladder rungs, and test_thermal.c covers
 * that. platform/sts_fan_policy.h decides what the box *does* with the answer,
 * and until this file existed nothing asserted any of it — including the one
 * rule both ARCHITECTURE.md §10.9 and firmware/CLAUDE.md state as hard:
 *
 *     "Keep FAN_PWM (PE5/TIM15_CH1) resting state = max airflow so a hung MCU
 *      can't cook the box."
 *
 * That invariant has three separate carriers — the start-up duty, the loop's
 * refusal path, and the duty-to-pulse conversion — and every one of them fails
 * silently. A fan running at 99 % instead of 100 % is indistinguishable from a
 * healthy box right up to the point where the enclosure passes 80 °C and the
 * board cold-cycles itself.
 *
 * The escalation half is here for the opposite reason: rungs 2 and 3 were once
 * annunciated and never actuated, so the ladder had no bottom. These tests pin
 * that each rung reaches a latch, and that the latch is not silently dropped by
 * the fail-safe path on its way past.
 *
 * The override half is here because it was measured to be uncovered: deleting
 * the release rule from hk.c outright — resolving a lapsed maintenance lease to
 * the thermal loop's last answer instead of to full airflow — left the entire
 * host suite green, because a decision taken in Zephyr glue is a decision no
 * host suite links. It is a fourth carrier of the same §10.9 invariant, and it
 * now lives in the policy header where these tests execute it.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "zephyr/platform/sts_fan_policy.h"

/* --------------------------------------------------- the resting invariant - */

/*
 * 100 % duty must be the FULL period, exactly. TIM15_CH1 feeds a 4-wire fan
 * whose speed rises with duty, so any short-fall here is airflow the thermal
 * design assumed it had. 40 000 ns is asserted as a number because the fan
 * specification's 21-28 kHz band is what makes the fan read the duty at all
 * rather than chopping its own commutation.
 */
static void test_full_duty_is_the_whole_period(void)
{
	TEST_ASSERT_EQUAL_UINT32(40000U, STS_FAN_PWM_PERIOD_NS);
	TEST_ASSERT_EQUAL_UINT32(STS_FAN_PWM_PERIOD_NS, sts_fan_pulse_ns(100U));
	TEST_ASSERT_EQUAL_UINT32(STS_FAN_PWM_PERIOD_NS,
				 sts_fan_pulse_ns(STS_FAN_DUTY_RESTING_PCT));
}

/* The resting duty IS maximum airflow, not merely "some duty". */
static void test_resting_duty_is_maximum(void)
{
	TEST_ASSERT_EQUAL_UINT8(100U, STS_FAN_DUTY_RESTING_PCT);
}

/* A duty computed out of range from a broken sensor must clamp UP to full
 * airflow, never wrap to a small pulse. */
static void test_out_of_range_duty_clamps_to_full_airflow(void)
{
	TEST_ASSERT_EQUAL_UINT32(STS_FAN_PWM_PERIOD_NS, sts_fan_pulse_ns(101U));
	TEST_ASSERT_EQUAL_UINT32(STS_FAN_PWM_PERIOD_NS, sts_fan_pulse_ns(200U));
	TEST_ASSERT_EQUAL_UINT32(STS_FAN_PWM_PERIOD_NS, sts_fan_pulse_ns(255U));
}

/* Monotone, with 0 genuinely off (the loop's own min_duty floor is core's
 * business, not the actuator's) and no arithmetic overflow anywhere between. */
static void test_pulse_width_is_monotone_and_bounded(void)
{
	uint32_t prev = 0;

	TEST_ASSERT_EQUAL_UINT32(0U, sts_fan_pulse_ns(0U));

	for (unsigned int d = 0; d <= 100U; d++) {
		uint32_t p = sts_fan_pulse_ns((uint8_t)d);

		TEST_ASSERT_LESS_OR_EQUAL_UINT32(STS_FAN_PWM_PERIOD_NS, p);
		TEST_ASSERT_GREATER_OR_EQUAL_UINT32(prev, p);
		prev = p;
	}

	/* 50 % is exactly half a period; a rounding slip would show here first. */
	TEST_ASSERT_EQUAL_UINT32(20000U, sts_fan_pulse_ns(50U));
	TEST_ASSERT_EQUAL_UINT32(8000U, sts_fan_pulse_ns(20U));
}

/* ---------------------------------------------------------- the refusal ---- */

/*
 * The path that matters most and runs least. thermal_step_1hz() refuses on a
 * NULL or uninitialised context, which is a firmware fault — and the response
 * to a firmware fault in the cooling loop must be cooling.
 */
static void test_a_refused_step_commands_full_airflow(void)
{
	sts_fan_action_t act;
	thermal_out_t out;

	memset(&out, 0, sizeof(out));
	out.duty_pct = 20U; /* whatever happened to be in the struct */

	sts_fan_policy_eval(-EINVAL, &out, &act);

	TEST_ASSERT_EQUAL_UINT8_MESSAGE(STS_FAN_DUTY_RESTING_PCT, act.duty_pct,
					"a refused thermal step did not go to full airflow");
	TEST_ASSERT_TRUE(act.loop_failed);
	TEST_ASSERT_EQUAL_UINT32(STS_FAN_PWM_PERIOD_NS,
				 sts_fan_pulse_ns(act.duty_pct));
}

/* A NULL output is the same fault by another route. */
static void test_a_null_output_commands_full_airflow(void)
{
	sts_fan_action_t act;

	sts_fan_policy_eval(0, NULL, &act);

	TEST_ASSERT_EQUAL_UINT8(STS_FAN_DUTY_RESTING_PCT, act.duty_pct);
	TEST_ASSERT_TRUE(act.loop_failed);
}

/*
 * A refusal must not touch the alarms or the escalation latches. The loop
 * declining to step is not evidence the box cooled down, and clearing
 * request_poe_kill on the way past would cancel an escalation a real
 * over-temperature raised — while the fan spins up and hides the symptom.
 */
static void test_a_refusal_touches_neither_alarms_nor_escalation(void)
{
	sts_fan_action_t act;
	sts_fan_alarm_t alarms[3];

	sts_fan_policy_eval(-EINVAL, NULL, &act);

	TEST_ASSERT_FALSE_MESSAGE(act.apply_alarms,
				  "a refused step rewrote the thermal alarms");
	TEST_ASSERT_FALSE_MESSAGE(act.latch_escalation,
				  "a refused step cleared the escalation latches");
	TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)sts_fan_alarms(&act, alarms));
}

/* ------------------------------------------------------------ the ladder --- */

static thermal_out_t rung(bool alarm, bool shed, bool kill, bool stall,
			  uint8_t duty)
{
	thermal_out_t o;

	memset(&o, 0, sizeof(o));
	o.duty_pct = duty;
	o.alarm_overtemp = alarm;
	o.request_rb_shed = shed;
	o.request_poe_kill = kill;
	o.fan_stall = stall;

	return o;
}

/* Rung 0: nothing wrong. The commanded duty passes through untouched — the
 * actuator must not have an opinion of its own about the regulated value. */
static void test_a_healthy_step_passes_the_duty_through(void)
{
	thermal_out_t out = rung(false, false, false, false, 37U);
	sts_fan_action_t act;

	sts_fan_policy_eval(0, &out, &act);

	TEST_ASSERT_EQUAL_UINT8(37U, act.duty_pct);
	TEST_ASSERT_FALSE(act.loop_failed);
	TEST_ASSERT_TRUE(act.apply_alarms);
	TEST_ASSERT_FALSE(act.alarm_thermal_warn);
	TEST_ASSERT_FALSE(act.alarm_thermal_critical);
	TEST_ASSERT_FALSE(act.alarm_fan_fault);
	TEST_ASSERT_TRUE(act.latch_escalation);
	TEST_ASSERT_FALSE(act.request_rb_shed);
	TEST_ASSERT_FALSE(act.request_poe_kill);
}

/*
 * Rung 2 must NOT read as thermal-critical.
 *
 * FAULT_ALARM_THERMAL_CRITICAL is in FAULT_RELAY_DISQUALIFY_DEFAULT, so
 * annunciating rung 2 there releases the holdover relay for a box that is still
 * serving traceable time on the OCXO — the shed is a load-management action,
 * not a loss of service. Wiring CRITICAL to request_rb_shed instead of
 * request_poe_kill is a one-word change that nothing else would catch.
 */
static void test_rung_two_sheds_the_rubidium_without_claiming_critical(void)
{
	thermal_out_t out = rung(true, true, false, false, 100U);
	sts_fan_action_t act;

	sts_fan_policy_eval(0, &out, &act);

	TEST_ASSERT_TRUE(act.alarm_thermal_warn);
	TEST_ASSERT_FALSE_MESSAGE(act.alarm_thermal_critical,
				  "rung 2 raised THERMAL_CRITICAL and would drop the relay");
	TEST_ASSERT_TRUE_MESSAGE(act.request_rb_shed,
				 "rung 2 did not reach the shed latch");
	TEST_ASSERT_FALSE(act.request_poe_kill);
}

/* Rung 3 latches both: the shed stays asserted and POE_KILL joins it. */
static void test_rung_three_reaches_the_kill_latch(void)
{
	thermal_out_t out = rung(true, true, true, false, 100U);
	sts_fan_action_t act;

	sts_fan_policy_eval(0, &out, &act);

	TEST_ASSERT_TRUE(act.alarm_thermal_warn);
	TEST_ASSERT_TRUE(act.alarm_thermal_critical);
	TEST_ASSERT_TRUE(act.request_rb_shed);
	TEST_ASSERT_TRUE_MESSAGE(act.request_poe_kill,
				 "rung 3 did not reach the POE_KILL latch");
}

/* A stalled fan is its own alarm and is independent of temperature: a fan that
 * has stopped at 25 °C is still a fan that has stopped. */
static void test_fan_stall_is_independent_of_the_ladder(void)
{
	thermal_out_t out = rung(false, false, false, true, 100U);
	sts_fan_action_t act;

	sts_fan_policy_eval(0, &out, &act);

	TEST_ASSERT_TRUE(act.alarm_fan_fault);
	TEST_ASSERT_FALSE(act.alarm_thermal_warn);
	TEST_ASSERT_FALSE(act.alarm_thermal_critical);
}

/* The three alarm ids, in order, with the values the plan carries. Asserted by
 * id so a reordering of the emitted array cannot swap two alarms' states. */
static void test_alarm_list_pairs_each_id_with_its_own_state(void)
{
	thermal_out_t out = rung(true, false, true, false, 100U);
	sts_fan_alarm_t alarms[3];
	sts_fan_action_t act;
	size_t n;

	sts_fan_policy_eval(0, &out, &act);
	n = sts_fan_alarms(&act, alarms);

	TEST_ASSERT_EQUAL_UINT32(3U, (uint32_t)n);

	for (size_t i = 0; i < n; i++) {
		switch (alarms[i].id) {
		case (uint8_t)FAULT_ALARM_THERMAL_WARN:
			TEST_ASSERT_TRUE(alarms[i].active);
			break;
		case (uint8_t)FAULT_ALARM_THERMAL_CRITICAL:
			TEST_ASSERT_TRUE(alarms[i].active);
			break;
		case (uint8_t)FAULT_ALARM_FAN_FAULT:
			TEST_ASSERT_FALSE(alarms[i].active);
			break;
		default:
			TEST_FAIL_MESSAGE("unexpected alarm id in the fan plan");
			break;
		}
	}
}

/* ------------------------------------------------------ override lease ---- */

/*
 * The release rule, and the reason this half of the file exists.
 *
 * A lease that has just lapsed is a moment when firmware has no live opinion of
 * what the fan should be doing — which is precisely the case ARCHITECTURE.md
 * §10.9 fixes at maximum airflow. Resolving a release to the loop's last answer
 * instead looks harmless and is not: that answer is up to a second stale, and on
 * a step the loop declined it is not a measurement of anything at all.
 *
 * The loop is deliberately parked at 30 % here. That is the only value at which
 * the two candidate rules — "resting" and "whatever the loop last said" — give
 * different answers, so this assertion is the one that fails if the release rule
 * is ever dropped.
 */
static void test_release_resolves_to_resting_not_to_the_loop(void)
{
	sts_fan_ovr_t ovr = { 0 };

	TEST_ASSERT_EQUAL_UINT32(40U, (uint32_t)sts_fan_ovr_apply(&ovr, true, 40U, 30U));

	TEST_ASSERT_EQUAL_UINT32(STS_FAN_DUTY_RESTING_PCT,
				 (uint32_t)sts_fan_ovr_apply(&ovr, false, 0U, 30U));
	TEST_ASSERT_EQUAL_UINT32(100U,
				 (uint32_t)sts_fan_ovr_apply(&ovr, false, 0U, 30U));
}

/* A release with no lease held is still a release, and still rests at full
 * airflow: pwrseq_exec drains a release for a lease that already expired. */
static void test_release_without_a_lease_still_rests_at_full(void)
{
	sts_fan_ovr_t ovr = { 0 };

	TEST_ASSERT_EQUAL_UINT32(STS_FAN_DUTY_RESTING_PCT,
				 (uint32_t)sts_fan_ovr_apply(&ovr, false, 0U, 0U));
	TEST_ASSERT_FALSE(ovr.active);
}

/* A release leaves no floor behind. The next 1 Hz step must resolve to the
 * loop's answer alone, or a lapsed lease would go on holding the fan up. */
static void test_release_clears_the_floor(void)
{
	sts_fan_ovr_t ovr = { 0 };

	(void)sts_fan_ovr_apply(&ovr, true, 90U, 30U);
	(void)sts_fan_ovr_apply(&ovr, false, 0U, 30U);

	TEST_ASSERT_FALSE(ovr.active);
	TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)ovr.pct);
	TEST_ASSERT_EQUAL_UINT32(30U, (uint32_t)sts_fan_resolve(30U, &ovr));
}

/*
 * The override is a FLOOR, not a level. A technician who asks for 30 % on a box
 * whose loop is already calling for 80 % gets 80 % — a maintenance request must
 * never be able to slow the fan down on a box that is heating.
 */
static void test_an_override_below_the_loop_cannot_lower_the_fan(void)
{
	sts_fan_ovr_t ovr = { 0 };

	TEST_ASSERT_EQUAL_UINT32(80U, (uint32_t)sts_fan_ovr_apply(&ovr, true, 30U, 80U));
	TEST_ASSERT_EQUAL_UINT32(80U, (uint32_t)sts_fan_resolve(80U, &ovr));

	/* Equality is not "above": a floor at the loop's own level changes
	 * nothing, and must not be read as an override winning. */
	TEST_ASSERT_EQUAL_UINT32(80U, (uint32_t)sts_fan_resolve(80U, &ovr));
	TEST_ASSERT_EQUAL_UINT32(30U, (uint32_t)ovr.pct);
}

/* Above the loop, the lease is the point: it raises the fan. */
static void test_an_override_above_the_loop_raises_it(void)
{
	sts_fan_ovr_t ovr = { 0 };

	TEST_ASSERT_EQUAL_UINT32(90U, (uint32_t)sts_fan_ovr_apply(&ovr, true, 90U, 30U));
	TEST_ASSERT_EQUAL_UINT32(90U, (uint32_t)sts_fan_resolve(30U, &ovr));
	TEST_ASSERT_EQUAL_UINT32(90U, (uint32_t)sts_fan_resolve(0U, &ovr));
}

/*
 * The loop always wins upward. A lease held at 60 % while the enclosure heats
 * must not cap the ladder at 60 %: the 1 Hz step re-resolves against the same
 * floor every second, and every rung above it has to get through.
 */
static void test_the_loop_rising_past_a_held_override_wins(void)
{
	sts_fan_ovr_t ovr = { 0 };

	(void)sts_fan_ovr_apply(&ovr, true, 60U, 20U);

	TEST_ASSERT_EQUAL_UINT32(60U, (uint32_t)sts_fan_resolve(20U, &ovr));
	TEST_ASSERT_EQUAL_UINT32(60U, (uint32_t)sts_fan_resolve(59U, &ovr));
	TEST_ASSERT_EQUAL_UINT32(61U, (uint32_t)sts_fan_resolve(61U, &ovr));
	TEST_ASSERT_EQUAL_UINT32(85U, (uint32_t)sts_fan_resolve(85U, &ovr));
	/* Including the fail-safe duty a declined step commands. */
	TEST_ASSERT_EQUAL_UINT32(STS_FAN_DUTY_RESTING_PCT,
				 (uint32_t)sts_fan_resolve(STS_FAN_DUTY_RESTING_PCT, &ovr));
}

/* An out-of-range request clamps to full airflow rather than being rejected —
 * the same direction as sts_fan_pulse_ns(), for the same reason. */
static void test_an_out_of_range_override_clamps_to_full_airflow(void)
{
	sts_fan_ovr_t ovr = { 0 };

	TEST_ASSERT_EQUAL_UINT32(100U, (uint32_t)sts_fan_ovr_apply(&ovr, true, 200U, 30U));
	TEST_ASSERT_EQUAL_UINT32(100U, (uint32_t)ovr.pct);
	TEST_ASSERT_EQUAL_UINT32(STS_FAN_PWM_PERIOD_NS, sts_fan_pulse_ns(ovr.pct));
}

/* No lease at all resolves to the loop, and a NULL lease is the same thing —
 * the 1 Hz step must never be made conditional on an override existing. */
static void test_no_override_resolves_to_the_loop(void)
{
	sts_fan_ovr_t ovr = { 0 };

	TEST_ASSERT_EQUAL_UINT32(45U, (uint32_t)sts_fan_resolve(45U, &ovr));
	TEST_ASSERT_EQUAL_UINT32(45U, (uint32_t)sts_fan_resolve(45U, NULL));

	/* An inactive lease carrying a stale level is still inactive. */
	ovr.pct = 99U;
	TEST_ASSERT_EQUAL_UINT32(45U, (uint32_t)sts_fan_resolve(45U, &ovr));
}

/* ------------------------------------------------------------------ tach --- */

/*
 * Two pulses per revolution. Getting the divisor wrong doubles or halves every
 * reported RPM, which core/thermal then compares against its rpm_at_full model
 * — so a factor-of-two error either invents a permanent stall alarm or masks a
 * real one. 6000 RPM at 2 pulses/rev over one second is 200 edges.
 */
static void test_tach_uses_two_pulses_per_revolution(void)
{
	TEST_ASSERT_EQUAL_UINT32(2U, STS_FAN_TACH_PULSES_PER_REV);
	TEST_ASSERT_EQUAL_UINT32(6000U, sts_fan_tach_rpm(200U, 1000U));
	TEST_ASSERT_EQUAL_UINT32(3000U, sts_fan_tach_rpm(100U, 1000U));
	/* Same rate measured over a quarter second must give the same RPM. */
	TEST_ASSERT_EQUAL_UINT32(6000U, sts_fan_tach_rpm(50U, 250U));
}

/* A stopped fan reads zero, which is what the stall detector wants to see. */
static void test_tach_reports_zero_for_no_edges(void)
{
	TEST_ASSERT_EQUAL_UINT32(0U, sts_fan_tach_rpm(0U, 1000U));
}

/* A zero-length interval carries no rate. Dividing by it would be the fault
 * report rather than the measurement. */
static void test_tach_zero_interval_is_zero_not_a_divide(void)
{
	TEST_ASSERT_EQUAL_UINT32(0U, sts_fan_tach_rpm(1234U, 0U));
}

/* An implausible edge count must not overflow into a plausible RPM. The 64-bit
 * intermediate is what stops edges * 60000 wrapping in 32 bits at ~71 600. */
static void test_tach_does_not_overflow_on_an_absurd_edge_count(void)
{
	TEST_ASSERT_EQUAL_UINT32(3000000U, sts_fan_tach_rpm(100000U, 1000U));
	TEST_ASSERT_EQUAL_UINT32(2148000U, sts_fan_tach_rpm(71600U, 1000U));
}

/* -------------------------------------------------------------------- main - */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_full_duty_is_the_whole_period);
	RUN_TEST(test_resting_duty_is_maximum);
	RUN_TEST(test_out_of_range_duty_clamps_to_full_airflow);
	RUN_TEST(test_pulse_width_is_monotone_and_bounded);

	RUN_TEST(test_a_refused_step_commands_full_airflow);
	RUN_TEST(test_a_null_output_commands_full_airflow);
	RUN_TEST(test_a_refusal_touches_neither_alarms_nor_escalation);

	RUN_TEST(test_a_healthy_step_passes_the_duty_through);
	RUN_TEST(test_rung_two_sheds_the_rubidium_without_claiming_critical);
	RUN_TEST(test_rung_three_reaches_the_kill_latch);
	RUN_TEST(test_fan_stall_is_independent_of_the_ladder);
	RUN_TEST(test_alarm_list_pairs_each_id_with_its_own_state);

	RUN_TEST(test_release_resolves_to_resting_not_to_the_loop);
	RUN_TEST(test_release_without_a_lease_still_rests_at_full);
	RUN_TEST(test_release_clears_the_floor);
	RUN_TEST(test_an_override_below_the_loop_cannot_lower_the_fan);
	RUN_TEST(test_an_override_above_the_loop_raises_it);
	RUN_TEST(test_the_loop_rising_past_a_held_override_wins);
	RUN_TEST(test_an_out_of_range_override_clamps_to_full_airflow);
	RUN_TEST(test_no_override_resolves_to_the_loop);

	RUN_TEST(test_tach_uses_two_pulses_per_revolution);
	RUN_TEST(test_tach_reports_zero_for_no_edges);
	RUN_TEST(test_tach_zero_interval_is_zero_not_a_divide);
	RUN_TEST(test_tach_does_not_overflow_on_an_absurd_edge_count);

	return UNITY_END();
}
