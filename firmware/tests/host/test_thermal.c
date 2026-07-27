/*
 * STS1000 "Meridian" — core/thermal unit tests.
 *
 * These are closed-loop scenario tests, not API smoke tests. A fan controller
 * that returns plausible numbers for a single call can still chatter, wind up,
 * or fail open; each case below drives the loop second by second against a
 * plant model and asserts the property spec §10.2 actually asks for.
 *
 * Provenance of the expectations: every duty figure is hand-derived from the
 * control law printed in thermal.h/thermal.c (duty = min_duty + Kp*e' + I,
 * where e' is the dead-banded error) using the documented default gains, and
 * every ladder threshold is read straight from spec §10.3.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "thermal/thermal.h"

/* ---------------------------------------------------------------- helpers */

static void in_defaults(thermal_in_t *in, uint64_t ms, int32_t enc_mc)
{
	memset(in, 0, sizeof(*in));
	in->mono_ms = ms;
	in->enclosure_mc = enc_mc;
	in->enclosure_valid = true;
	in->osc_mc = 55000;
	in->osc_valid = true;
	in->die_mc = 50000;
	in->die_valid = true;
	/* A fan comfortably above any model expectation, so the stall detector
	 * stays quiet unless a case deliberately provokes it. */
	in->fan_rpm = 6000u;
	in->rpm_valid = true;
}

/* Run n seconds at a fixed temperature; returns the final output. */
static thermal_out_t soak(thermal_ctx_t *ctx, uint64_t *ms, int32_t enc_mc,
			  unsigned int n)
{
	thermal_in_t in;
	thermal_out_t out;
	unsigned int i;

	memset(&out, 0, sizeof(out));
	for (i = 0u; i < n; i++) {
		*ms += 1000u;
		in_defaults(&in, *ms, enc_mc);
		TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(ctx, &in, &out));
	}
	return out;
}

/*
 * First-order enclosure plant, used for the convergence test.
 *
 *   dT/dt = (heat - cooling(duty)) / C
 *
 * with cooling proportional to duty and a slow passive relaxation towards
 * ambient. The numbers are not a model of any particular box; they are chosen
 * so that holding 45 °C requires about 40 % duty — off both actuator rails,
 * which is what makes the test able to fail if the integral term is wrong.
 * Solving 0.060 - 0.001*d - 0.001*(T - 25) = 0 at T = 46 gives d = 39.
 */
struct plant {
	double temp_c;
	double ambient_c;
	double heat_c_per_s;      /* rise rate with the fan stopped */
	double cool_c_per_s_full; /* extra fall rate at 100 % duty */
	double relax_per_s;       /* passive loss coefficient towards ambient */
};

static void plant_step(struct plant *p, uint8_t duty, double dt)
{
	double cool = p->cool_c_per_s_full * ((double)duty / 100.0);
	double relax = (p->temp_c - p->ambient_c) * p->relax_per_s;

	p->temp_c += (p->heat_c_per_s - cool - relax) * dt;
}

/* ------------------------------------------------------------------- setup */

static void test_defaults_and_init(void)
{
	thermal_cfg_t cfg;
	thermal_ctx_t ctx;

	TEST_ASSERT_EQUAL_INT(-EINVAL, thermal_cfg_defaults(NULL));
	TEST_ASSERT_EQUAL_INT(0, thermal_cfg_defaults(&cfg));

	/* Spec §10.2/§10.3 values. */
	TEST_ASSERT_EQUAL_INT32(45000, cfg.setpoint_mc);
	TEST_ASSERT_EQUAL_UINT8(20u, cfg.min_duty_pct);
	TEST_ASSERT_EQUAL_UINT8(100u, cfg.max_duty_pct);
	TEST_ASSERT_EQUAL_INT32(60000, cfg.alarm_mc);
	TEST_ASSERT_EQUAL_INT32(70000, cfg.shed_rb_mc);
	TEST_ASSERT_EQUAL_INT32(80000, cfg.kill_mc);
	TEST_ASSERT_EQUAL_UINT16(500u, cfg.stall_rpm_floor);
	TEST_ASSERT_EQUAL_UINT8(30u, cfg.stall_duty_pct);
	TEST_ASSERT_EQUAL_UINT8(5u, cfg.stall_dwell_s);

	TEST_ASSERT_EQUAL_INT(-EINVAL, thermal_init(NULL, &cfg));
	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, &cfg));
	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, NULL));

	/* Init leaves the actuator where the hardware already is: PE5 undriven
	 * is full speed (ARCHITECTURE.md §10 invariant 9). */
	TEST_ASSERT_EQUAL_UINT8(100u, ctx.duty_pct);
}

static void test_init_rejects_bad_config(void)
{
	thermal_cfg_t cfg;
	thermal_ctx_t ctx;

	/* Duty range inverted. */
	(void)thermal_cfg_defaults(&cfg);
	cfg.min_duty_pct = 90u;
	cfg.max_duty_pct = 50u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, thermal_init(&ctx, &cfg));

	(void)thermal_cfg_defaults(&cfg);
	cfg.max_duty_pct = 150u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, thermal_init(&ctx, &cfg));

	/* Non-positive proportional gain would open the loop. */
	(void)thermal_cfg_defaults(&cfg);
	cfg.kp_pct_per_c = 0.0f;
	TEST_ASSERT_EQUAL_INT(-EINVAL, thermal_init(&ctx, &cfg));

	(void)thermal_cfg_defaults(&cfg);
	cfg.ki_pct_per_c_s = -1.0f;
	TEST_ASSERT_EQUAL_INT(-EINVAL, thermal_init(&ctx, &cfg));

	(void)thermal_cfg_defaults(&cfg);
	cfg.deadband_mc = -1;
	TEST_ASSERT_EQUAL_INT(-EINVAL, thermal_init(&ctx, &cfg));

	(void)thermal_cfg_defaults(&cfg);
	cfg.ladder_hyst_mc = -1;
	TEST_ASSERT_EQUAL_INT(-EINVAL, thermal_init(&ctx, &cfg));

	/* A ladder that does not escalate would make a rung unreachable. */
	(void)thermal_cfg_defaults(&cfg);
	cfg.shed_rb_mc = 50000;
	TEST_ASSERT_EQUAL_INT(-EINVAL, thermal_init(&ctx, &cfg));

	(void)thermal_cfg_defaults(&cfg);
	cfg.kill_mc = 65000;
	TEST_ASSERT_EQUAL_INT(-EINVAL, thermal_init(&ctx, &cfg));

	(void)thermal_cfg_defaults(&cfg);
	cfg.rpm_model_pct = 101u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, thermal_init(&ctx, &cfg));

	(void)thermal_cfg_defaults(&cfg);
	cfg.stall_duty_pct = 101u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, thermal_init(&ctx, &cfg));

	(void)thermal_cfg_defaults(&cfg);
	cfg.stall_dwell_s = 0u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, thermal_init(&ctx, &cfg));
}

static void test_step_rejects_null_and_uninitialised(void)
{
	thermal_ctx_t ctx;
	thermal_in_t in;
	thermal_out_t out;

	memset(&ctx, 0, sizeof(ctx)); /* initialised == false */
	in_defaults(&in, 1000u, 40000);
	TEST_ASSERT_EQUAL_INT(-EINVAL, thermal_step_1hz(&ctx, &in, &out));

	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, thermal_step_1hz(NULL, &in, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, thermal_step_1hz(&ctx, NULL, &out));
	/* A NULL out is legal — a caller that only wants the side effects. */
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, NULL));
}

/* ------------------------------------------------------------ the PI loop */

static void test_cold_box_sits_on_the_minimum_duty(void)
{
	thermal_ctx_t ctx;
	uint64_t ms = 0u;
	thermal_out_t out;

	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, NULL));
	/* 20 °C is 25 °C below setpoint: the loop must floor, not stop. A fan
	 * that stops has a bearing life problem (§10.2). */
	out = soak(&ctx, &ms, 20000, 30u);

	TEST_ASSERT_EQUAL_UINT8(20u, out.duty_pct);
	TEST_ASSERT_TRUE((out.flags & THERMAL_FLAG_MIN_DUTY) != 0u);
	TEST_ASSERT_FALSE(out.failsafe);
	TEST_ASSERT_FALSE(out.alarm_overtemp);
}

static void test_proportional_response_matches_the_control_law(void)
{
	thermal_ctx_t ctx;
	thermal_cfg_t cfg;
	thermal_in_t in;
	thermal_out_t out;

	/* Integral disabled so the first sample isolates the P term:
	 * duty = 20 + 15 * (50 - 45 - 1) = 20 + 60 = 80. */
	(void)thermal_cfg_defaults(&cfg);
	cfg.ki_pct_per_c_s = 0.0f;
	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, &cfg));

	in_defaults(&in, 1000u, 50000);
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
	TEST_ASSERT_EQUAL_UINT8(80u, out.duty_pct);

	/* 3 °C over, minus the 1 °C dead band: 20 + 15*2 = 50. */
	in_defaults(&in, 2000u, 48000);
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
	TEST_ASSERT_EQUAL_UINT8(50u, out.duty_pct);

	/* Far over: clamped to the ceiling, and flagged as such. */
	in_defaults(&in, 3000u, 58000);
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
	TEST_ASSERT_EQUAL_UINT8(100u, out.duty_pct);
	TEST_ASSERT_TRUE((out.flags & THERMAL_FLAG_MAX_DUTY) != 0u);
}

static void test_pi_converges_on_a_first_order_plant(void)
{
	thermal_ctx_t ctx;
	thermal_in_t in;
	thermal_out_t out;
	struct plant p = {
		.temp_c = 30.0,
		.ambient_c = 25.0,
		.heat_c_per_s = 0.060,
		.cool_c_per_s_full = 0.100,
		.relax_per_s = 0.001,
	};
	uint64_t ms = 0u;
	unsigned int i;
	double worst_after_settle = 0.0;

	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, NULL));

	for (i = 0u; i < 4000u; i++) {
		int32_t mc = (int32_t)(p.temp_c * 1000.0);

		ms += 1000u;
		in_defaults(&in, ms, mc);
		TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
		plant_step(&p, out.duty_pct, 1.0);

		if (i > 3000u) {
			double e = p.temp_c - 45.0;

			if (e < 0.0) {
				e = -e;
			}
			if (e > worst_after_settle) {
				worst_after_settle = e;
			}
		}
	}

	/* Regulated to inside the dead band plus a little quantisation, and
	 * the actuator is off both rails — i.e. the integral is doing work. */
	TEST_ASSERT_TRUE(worst_after_settle < 1.5);
	TEST_ASSERT_TRUE(out.duty_pct > 20u);
	TEST_ASSERT_TRUE(out.duty_pct < 100u);
}

static void test_deadband_suppresses_chatter(void)
{
	thermal_ctx_t ctx;
	thermal_cfg_t cfg;
	thermal_in_t in;
	thermal_out_t out;
	uint64_t ms = 0u;
	unsigned int i;
	uint8_t first = 0u;

	/* Integral off so any duty movement can only come from the dead band
	 * failing to absorb the dither. */
	(void)thermal_cfg_defaults(&cfg);
	cfg.ki_pct_per_c_s = 0.0f;
	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, &cfg));

	/* Dither +-0.6 °C around the 45 °C setpoint, inside the +-1 °C band.
	 * Without hysteresis, 15 %/°C would swing the duty by 18 points every
	 * second and the fan would audibly hunt. */
	for (i = 0u; i < 40u; i++) {
		int32_t mc = 45000 + (((i % 2u) == 0u) ? 600 : -600);

		ms += 1000u;
		in_defaults(&in, ms, mc);
		TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
		TEST_ASSERT_TRUE((out.flags & THERMAL_FLAG_DEADBAND) != 0u);
		if (i == 0u) {
			first = out.duty_pct;
		} else {
			TEST_ASSERT_EQUAL_UINT8(first, out.duty_pct);
		}
	}
}

static void test_deadband_is_continuous_at_the_band_edge(void)
{
	thermal_ctx_t ctx;
	thermal_cfg_t cfg;
	thermal_in_t in;
	thermal_out_t at_edge;
	thermal_out_t just_past;

	(void)thermal_cfg_defaults(&cfg);
	cfg.ki_pct_per_c_s = 0.0f;
	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, &cfg));

	/* Exactly one dead band over setpoint -> zero effective error. */
	in_defaults(&in, 1000u, 46000);
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &at_edge));
	TEST_ASSERT_EQUAL_UINT8(20u, at_edge.duty_pct);

	/* A tenth of a degree past it -> 1.5 points, not a Kp*deadband step.
	 * A hard zeroing dead band would have jumped straight to 35 %. */
	in_defaults(&in, 2000u, 46100);
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &just_past));
	TEST_ASSERT_TRUE(just_past.duty_pct <= 23u);
	TEST_ASSERT_TRUE(just_past.duty_pct >= 21u);
}

static void test_integral_does_not_wind_up(void)
{
	thermal_ctx_t ctx;
	thermal_in_t in;
	thermal_out_t out;
	uint64_t ms = 0u;
	unsigned int i;

	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, NULL));

	/* An hour pinned far over setpoint saturates the output. */
	out = soak(&ctx, &ms, 75000, 3600u);
	TEST_ASSERT_EQUAL_UINT8(100u, out.duty_pct);

	/* The moment the box is cold again the fan must come back down. With a
	 * wound-up integrator it would sit at 100 % for hours. */
	for (i = 0u; i < 5u; i++) {
		ms += 1000u;
		in_defaults(&in, ms, 30000);
		TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
	}
	TEST_ASSERT_EQUAL_UINT8(20u, out.duty_pct);
}

static void test_irregular_cadence_is_tolerated(void)
{
	thermal_ctx_t ctx;
	thermal_in_t in;
	thermal_out_t out;

	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, NULL));

	/* A duplicated timestamp, a very long gap and a backwards clock must
	 * all be survivable: the interval falls back to the nominal second
	 * rather than integrating a wild step. */
	in_defaults(&in, 1000u, 50000);
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
	in_defaults(&in, 1000u, 50000);
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
	in_defaults(&in, 900000u, 50000);
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
	in_defaults(&in, 800000u, 50000);
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));

	TEST_ASSERT_TRUE(out.duty_pct >= 20u);
	TEST_ASSERT_TRUE(out.duty_pct <= 100u);
}

/* ---------------------------------------------------------------- failsafe */

static void test_invalid_sensor_goes_to_full_speed(void)
{
	thermal_ctx_t ctx;
	thermal_in_t in;
	thermal_out_t out;
	uint64_t ms = 0u;

	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, NULL));

	/* Settle cold first, so a stale duty would be visible as 20 %. */
	out = soak(&ctx, &ms, 25000, 20u);
	TEST_ASSERT_EQUAL_UINT8(20u, out.duty_pct);

	ms += 1000u;
	in_defaults(&in, ms, 25000);
	in.enclosure_valid = false;
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));

	TEST_ASSERT_EQUAL_UINT8(100u, out.duty_pct);
	TEST_ASSERT_TRUE(out.failsafe);
	TEST_ASSERT_TRUE((out.flags & THERMAL_FLAG_FAILSAFE) != 0u);
	TEST_ASSERT_TRUE((out.flags & THERMAL_FLAG_MAX_DUTY) != 0u);

	/* And it recovers when the sensor comes back. */
	out = soak(&ctx, &ms, 25000, 5u);
	TEST_ASSERT_FALSE(out.failsafe);
	TEST_ASSERT_EQUAL_UINT8(20u, out.duty_pct);
}

static void test_failsafe_holds_the_ladder_rather_than_escalating(void)
{
	thermal_ctx_t ctx;
	thermal_in_t in;
	thermal_out_t out;
	uint64_t ms = 0u;

	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, NULL));

	/* Latch the alarm and the Rb shed. */
	out = soak(&ctx, &ms, 72000, 3u);
	TEST_ASSERT_TRUE(out.alarm_overtemp);
	TEST_ASSERT_TRUE(out.request_rb_shed);
	TEST_ASSERT_FALSE(out.request_poe_kill);

	/* Losing the sensor must neither clear those (the box is still hot)
	 * nor escalate to a cold cycle on no evidence. */
	ms += 1000u;
	in_defaults(&in, ms, 72000);
	in.enclosure_valid = false;
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
	TEST_ASSERT_TRUE(out.alarm_overtemp);
	TEST_ASSERT_TRUE(out.request_rb_shed);
	TEST_ASSERT_FALSE(out.request_poe_kill);
	TEST_ASSERT_TRUE((out.flags & THERMAL_FLAG_ALARM) != 0u);
	TEST_ASSERT_TRUE((out.flags & THERMAL_FLAG_SHED_RB) != 0u);
	TEST_ASSERT_TRUE((out.flags & THERMAL_FLAG_POE_KILL) == 0u);

	/* A cold cycle already requested stays requested when the sensor
	 * disappears: pwrseq must not see the request retracted while the
	 * enclosure is, as far as anyone knows, still at 81 C. */
	out = soak(&ctx, &ms, 81000, 2u);
	TEST_ASSERT_TRUE(out.request_poe_kill);

	ms += 1000u;
	in_defaults(&in, ms, 81000);
	in.enclosure_valid = false;
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
	TEST_ASSERT_TRUE(out.request_poe_kill);
	TEST_ASSERT_TRUE((out.flags & THERMAL_FLAG_POE_KILL) != 0u);
}

/* ------------------------------------------------------------- the ladder */

static void test_ladder_rungs(void)
{
	thermal_ctx_t ctx;
	uint64_t ms = 0u;
	thermal_out_t out;

	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, NULL));

	out = soak(&ctx, &ms, 59000, 2u);
	TEST_ASSERT_FALSE(out.alarm_overtemp);

	out = soak(&ctx, &ms, 61000, 2u);
	TEST_ASSERT_TRUE(out.alarm_overtemp);
	TEST_ASSERT_FALSE(out.request_rb_shed);
	TEST_ASSERT_FALSE(out.request_poe_kill);

	out = soak(&ctx, &ms, 71000, 2u);
	TEST_ASSERT_TRUE(out.alarm_overtemp);
	TEST_ASSERT_TRUE(out.request_rb_shed);
	TEST_ASSERT_FALSE(out.request_poe_kill);

	out = soak(&ctx, &ms, 81000, 2u);
	TEST_ASSERT_TRUE(out.alarm_overtemp);
	TEST_ASSERT_TRUE(out.request_rb_shed);
	TEST_ASSERT_TRUE(out.request_poe_kill);
	TEST_ASSERT_TRUE((out.flags & THERMAL_FLAG_POE_KILL) != 0u);
}

static void test_ladder_releases_only_with_hysteresis(void)
{
	thermal_ctx_t ctx;
	uint64_t ms = 0u;
	thermal_out_t out;

	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, NULL));

	out = soak(&ctx, &ms, 61000, 2u);
	TEST_ASSERT_TRUE(out.alarm_overtemp);

	/* 59.5 °C is below the 60 °C trip but inside the 2 °C release band —
	 * a latched rung must not drop out on a fraction of a degree. */
	out = soak(&ctx, &ms, 59500, 2u);
	TEST_ASSERT_TRUE(out.alarm_overtemp);

	out = soak(&ctx, &ms, 58500, 2u);
	TEST_ASSERT_TRUE(out.alarm_overtemp);

	/* Below 58 °C it finally clears. */
	out = soak(&ctx, &ms, 57500, 2u);
	TEST_ASSERT_FALSE(out.alarm_overtemp);
}

/* ------------------------------------------------------------ stall detect */

static void test_stall_detection(void)
{
	thermal_ctx_t ctx;
	thermal_in_t in;
	thermal_out_t out;
	uint64_t ms = 0u;
	unsigned int i;

	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, NULL));

	/* Hot enough to command full duty; the model then expects
	 * 6000 * 100/100 * 50/100 = 3000 RPM. */
	for (i = 0u; i < 4u; i++) {
		ms += 1000u;
		in_defaults(&in, ms, 70000);
		in.fan_rpm = 200u;
		TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
		TEST_ASSERT_EQUAL_UINT16(3000u, out.expected_rpm);
		/* Below the 5 s dwell it must not fire — one bad tach reading
		 * is not a stall. */
		TEST_ASSERT_FALSE(out.fan_stall);
	}

	ms += 1000u;
	in_defaults(&in, ms, 70000);
	in.fan_rpm = 200u;
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
	TEST_ASSERT_TRUE(out.fan_stall);
	TEST_ASSERT_TRUE((out.flags & THERMAL_FLAG_FAN_STALL) != 0u);

	/* A healthy fan clears it immediately. */
	ms += 1000u;
	in_defaults(&in, ms, 70000);
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
	TEST_ASSERT_FALSE(out.fan_stall);
}

static void test_no_stall_expectation_below_the_duty_threshold(void)
{
	thermal_ctx_t ctx;
	thermal_in_t in;
	thermal_out_t out;
	uint64_t ms = 0u;
	unsigned int i;

	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, NULL));

	/* Cold box: duty floors at 20 %, which is below stall_duty_pct. A
	 * 4-wire fan may legitimately be barely turning there. */
	for (i = 0u; i < 20u; i++) {
		ms += 1000u;
		in_defaults(&in, ms, 20000);
		in.fan_rpm = 0u;
		TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
	}
	TEST_ASSERT_EQUAL_UINT8(20u, out.duty_pct);
	TEST_ASSERT_EQUAL_UINT16(0u, out.expected_rpm);
	TEST_ASSERT_FALSE(out.fan_stall);
}

static void test_rpm_floor_applies_at_modest_duty(void)
{
	thermal_ctx_t ctx;
	thermal_cfg_t cfg;
	thermal_in_t in;
	thermal_out_t out;
	uint64_t ms = 0u;

	/* A fan whose linear model predicts almost nothing at 32 % duty:
	 * 1000 * 32/100 * 50/100 = 160 RPM, below the 500 RPM floor. The floor
	 * is what must be enforced. */
	(void)thermal_cfg_defaults(&cfg);
	cfg.rpm_at_full = 1000u;
	cfg.ki_pct_per_c_s = 0.0f;
	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, &cfg));

	/* 20 + 15*(46.8 - 45 - 1) = 20 + 12 = 32 % duty. */
	ms += 1000u;
	in_defaults(&in, ms, 46800);
	in.fan_rpm = 400u;
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
	TEST_ASSERT_EQUAL_UINT8(32u, out.duty_pct);
	TEST_ASSERT_EQUAL_UINT16(500u, out.expected_rpm);

	out = soak(&ctx, &ms, 46800, 1u);
	TEST_ASSERT_EQUAL_UINT16(500u, out.expected_rpm);
}

static void test_missing_tach_is_reported_but_is_not_a_stall(void)
{
	thermal_ctx_t ctx;
	thermal_in_t in;
	thermal_out_t out;
	uint64_t ms = 0u;
	unsigned int i;

	TEST_ASSERT_EQUAL_INT(0, thermal_init(&ctx, NULL));

	for (i = 0u; i < 4u; i++) {
		ms += 1000u;
		in_defaults(&in, ms, 70000);
		in.rpm_valid = false;
		TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
		TEST_ASSERT_TRUE((out.flags & THERMAL_FLAG_TACH_FAULT) == 0u);
	}

	ms += 1000u;
	in_defaults(&in, ms, 70000);
	in.rpm_valid = false;
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
	/* A dead tachometer is annunciated, but the module must not claim a
	 * stall it cannot observe — the fan may well be spinning. */
	TEST_ASSERT_TRUE((out.flags & THERMAL_FLAG_TACH_FAULT) != 0u);
	TEST_ASSERT_FALSE(out.fan_stall);

	ms += 1000u;
	in_defaults(&in, ms, 70000);
	TEST_ASSERT_EQUAL_INT(0, thermal_step_1hz(&ctx, &in, &out));
	TEST_ASSERT_TRUE((out.flags & THERMAL_FLAG_TACH_FAULT) == 0u);
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_defaults_and_init);
	RUN_TEST(test_init_rejects_bad_config);
	RUN_TEST(test_step_rejects_null_and_uninitialised);

	RUN_TEST(test_cold_box_sits_on_the_minimum_duty);
	RUN_TEST(test_proportional_response_matches_the_control_law);
	RUN_TEST(test_pi_converges_on_a_first_order_plant);
	RUN_TEST(test_deadband_suppresses_chatter);
	RUN_TEST(test_deadband_is_continuous_at_the_band_edge);
	RUN_TEST(test_integral_does_not_wind_up);
	RUN_TEST(test_irregular_cadence_is_tolerated);

	RUN_TEST(test_invalid_sensor_goes_to_full_speed);
	RUN_TEST(test_failsafe_holds_the_ladder_rather_than_escalating);

	RUN_TEST(test_ladder_rungs);
	RUN_TEST(test_ladder_releases_only_with_hysteresis);

	RUN_TEST(test_stall_detection);
	RUN_TEST(test_no_stall_expectation_below_the_duty_threshold);
	RUN_TEST(test_rpm_floor_applies_at_modest_duty);
	RUN_TEST(test_missing_tach_is_reported_but_is_not_a_stall);

	return UNITY_END();
}
