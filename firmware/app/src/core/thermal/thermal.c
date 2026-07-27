/*
 * STS1000 "Meridian" — core/thermal implementation.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See thermal.h for the contract and the fail-safe reasoning.
 */

#include "thermal/thermal.h"

#include <errno.h>
#include <math.h>
#include <string.h>

/* Bounds on the measured iteration interval. A stalled or wildly late caller
 * must not integrate a huge step; a duplicated timestamp must not integrate
 * zero and stall the loop either. */
#define DT_MIN_S 0.1f
#define DT_MAX_S 10.0f
#define DT_NOMINAL_S 1.0f

int thermal_cfg_defaults(thermal_cfg_t *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}

	memset(cfg, 0, sizeof(*cfg));
	cfg->setpoint_mc = 45000;
	cfg->deadband_mc = 1000;
	/* BENCH §10.2: sized for an enclosure time constant of minutes.
	 * Kp = 15 %/°C puts the fan at full 5 °C over setpoint; the integral
	 * time constant Kp/Ki = 300 s keeps the loop slower than the plant. */
	cfg->kp_pct_per_c = 15.0f;
	cfg->ki_pct_per_c_s = 0.05f;
	cfg->min_duty_pct = 20u;
	cfg->max_duty_pct = 100u;

	cfg->alarm_mc = 60000;
	cfg->shed_rb_mc = 70000;
	cfg->kill_mc = 80000;
	cfg->ladder_hyst_mc = 2000;

	cfg->rpm_at_full = 6000u;
	cfg->rpm_model_pct = 50u;
	cfg->stall_rpm_floor = 500u;
	cfg->stall_duty_pct = 30u;
	cfg->stall_dwell_s = 5u;
	return 0;
}

static bool cfg_valid(const thermal_cfg_t *c)
{
	if (c->max_duty_pct > 100u || c->min_duty_pct > c->max_duty_pct) {
		return false;
	}
	if (!(c->kp_pct_per_c > 0.0f) || !isfinite(c->kp_pct_per_c)) {
		return false;
	}
	if (!(c->ki_pct_per_c_s >= 0.0f) || !isfinite(c->ki_pct_per_c_s)) {
		return false;
	}
	if (c->deadband_mc < 0) {
		return false;
	}
	if (c->ladder_hyst_mc < 0) {
		return false;
	}
	/* The ladder must escalate, or a rung would be unreachable. */
	if (!(c->alarm_mc < c->shed_rb_mc) || !(c->shed_rb_mc < c->kill_mc)) {
		return false;
	}
	if (c->rpm_model_pct > 100u) {
		return false;
	}
	if (c->stall_duty_pct > 100u || c->stall_dwell_s == 0u) {
		return false;
	}
	return true;
}

int thermal_init(thermal_ctx_t *ctx, const thermal_cfg_t *cfg)
{
	thermal_cfg_t local;

	if (ctx == NULL) {
		return -EINVAL;
	}
	if (cfg == NULL) {
		(void)thermal_cfg_defaults(&local);
		cfg = &local;
	}
	if (!cfg_valid(cfg)) {
		return -EINVAL;
	}

	memset(ctx, 0, sizeof(*ctx));
	ctx->cfg = *cfg;
	/* Start where the hardware already is: PE5 undriven == full speed. */
	ctx->duty_pct = cfg->max_duty_pct;
	ctx->initialised = true;
	return 0;
}

/* One rung of the escalation ladder: latch on above the threshold, release
 * only after falling ladder_hyst_mc below it. */
static bool rung(bool latched, int32_t t_mc, int32_t on_mc, int32_t hyst_mc)
{
	if (!latched) {
		return t_mc > on_mc;
	}
	return t_mc > (on_mc - hyst_mc);
}

static uint8_t clamp_duty(const thermal_cfg_t *c, float duty)
{
	if (!(duty > (float)c->min_duty_pct)) {
		return c->min_duty_pct;
	}
	if (duty >= (float)c->max_duty_pct) {
		return c->max_duty_pct;
	}
	return (uint8_t)(duty + 0.5f);
}

/*
 * Expected RPM at the commanded duty: a linear fan model derated by
 * rpm_model_pct, with an absolute floor once the duty is high enough that the
 * fan must certainly be turning. Below stall_duty_pct no expectation is made —
 * a 4-wire fan may legitimately be at or near a standstill there.
 */
static uint16_t expected_rpm(const thermal_cfg_t *c, uint8_t duty)
{
	uint32_t model;

	if (duty <= c->stall_duty_pct) {
		return 0u;
	}

	model = ((uint32_t)c->rpm_at_full * (uint32_t)duty *
		 (uint32_t)c->rpm_model_pct) / 10000u;
	if (model < (uint32_t)c->stall_rpm_floor) {
		model = c->stall_rpm_floor;
	}
	return (uint16_t)model;
}

int thermal_step_1hz(thermal_ctx_t *ctx, const thermal_in_t *in,
		     thermal_out_t *out)
{
	thermal_out_t o;
	const thermal_cfg_t *c;
	float dt = DT_NOMINAL_S;
	int32_t t_mc;
	uint16_t want_rpm;

	if (ctx == NULL || in == NULL || !ctx->initialised) {
		return -EINVAL;
	}
	c = &ctx->cfg;

	memset(&o, 0, sizeof(o));

	if (ctx->have_prev && in->mono_ms > ctx->prev_mono_ms) {
		float d = (float)(in->mono_ms - ctx->prev_mono_ms) * 0.001f;

		if (d >= DT_MIN_S && d <= DT_MAX_S) {
			dt = d;
		}
	}
	ctx->prev_mono_ms = in->mono_ms;
	ctx->have_prev = true;

	if (!in->enclosure_valid) {
		/*
		 * §10.2 fail-safe. No usable enclosure reading means no basis
		 * for running the fan slowly. Hold the integral where it is so
		 * a transient sensor dropout does not reset the loop, hold the
		 * ladder rungs (escalating on missing data would be as wrong as
		 * releasing on it), and go to maximum airflow.
		 */
		ctx->duty_pct = c->max_duty_pct;
		o.duty_pct = ctx->duty_pct;
		o.failsafe = true;
		o.temp_mc = 0;
		o.flags = THERMAL_FLAG_FAILSAFE | THERMAL_FLAG_MAX_DUTY;
		o.alarm_overtemp = ctx->alarm;
		o.request_rb_shed = ctx->shed_rb;
		o.request_poe_kill = ctx->poe_kill;
		if (ctx->alarm) {
			o.flags |= THERMAL_FLAG_ALARM;
		}
		if (ctx->shed_rb) {
			o.flags |= THERMAL_FLAG_SHED_RB;
		}
		if (ctx->poe_kill) {
			o.flags |= THERMAL_FLAG_POE_KILL;
		}
	} else {
		float err_c;
		float duty_f;

		t_mc = in->enclosure_mc;
		o.temp_mc = t_mc;

		/*
		 * Dead-band around the setpoint. The error is shrunk towards
		 * zero rather than zeroed outright, so the control law stays
		 * continuous at the band edge — zeroing it would trade
		 * dithering chatter for a step of Kp*deadband every time the
		 * temperature crossed the boundary.
		 */
		err_c = (float)(t_mc - c->setpoint_mc) * 0.001f;
		if (err_c > 0.0f) {
			err_c -= (float)c->deadband_mc * 0.001f;
			if (err_c < 0.0f) {
				err_c = 0.0f;
			}
		} else {
			err_c += (float)c->deadband_mc * 0.001f;
			if (err_c > 0.0f) {
				err_c = 0.0f;
			}
		}
		if (err_c == 0.0f) {
			o.flags |= THERMAL_FLAG_DEADBAND;
		}

		/* Conditional-integration anti-windup against the duty rails. */
		{
			float i_delta = c->ki_pct_per_c_s * err_c * dt;
			float span = (float)c->max_duty_pct -
				     (float)c->min_duty_pct;
			float post = ctx->integ_pct + i_delta;

			if (post < 0.0f) {
				post = 0.0f;
			} else if (post > span) {
				post = span;
			}
			ctx->integ_pct = post;
		}

		duty_f = (float)c->min_duty_pct + c->kp_pct_per_c * err_c +
			 ctx->integ_pct;
		ctx->duty_pct = clamp_duty(c, duty_f);
		o.duty_pct = ctx->duty_pct;

		if (ctx->duty_pct == c->min_duty_pct) {
			o.flags |= THERMAL_FLAG_MIN_DUTY;
		}
		if (ctx->duty_pct == c->max_duty_pct) {
			o.flags |= THERMAL_FLAG_MAX_DUTY;
		}

		/* §10.3 escalation ladder, each rung latched with hysteresis. */
		ctx->alarm = rung(ctx->alarm, t_mc, c->alarm_mc,
				  c->ladder_hyst_mc);
		ctx->shed_rb = rung(ctx->shed_rb, t_mc, c->shed_rb_mc,
				    c->ladder_hyst_mc);
		ctx->poe_kill = rung(ctx->poe_kill, t_mc, c->kill_mc,
				     c->ladder_hyst_mc);

		o.alarm_overtemp = ctx->alarm;
		o.request_rb_shed = ctx->shed_rb;
		o.request_poe_kill = ctx->poe_kill;
		if (ctx->alarm) {
			o.flags |= THERMAL_FLAG_ALARM;
		}
		if (ctx->shed_rb) {
			o.flags |= THERMAL_FLAG_SHED_RB;
		}
		if (ctx->poe_kill) {
			o.flags |= THERMAL_FLAG_POE_KILL;
		}
	}

	/* ---- stall / under-speed, against the duty just commanded ---- */
	want_rpm = expected_rpm(c, ctx->duty_pct);
	o.expected_rpm = want_rpm;

	if (!in->rpm_valid) {
		/* A dead tachometer is a distinct fault from a stalled fan:
		 * the fan may well be spinning. Annunciate it, but do not
		 * claim a stall we cannot observe. */
		if (ctx->tach_bad_run < UINT8_MAX) {
			ctx->tach_bad_run++;
		}
		if (ctx->tach_bad_run >= c->stall_dwell_s) {
			o.flags |= THERMAL_FLAG_TACH_FAULT;
		}
		ctx->stall_run = 0u;
	} else {
		ctx->tach_bad_run = 0u;
		if (want_rpm != 0u && in->fan_rpm < want_rpm) {
			if (ctx->stall_run < UINT8_MAX) {
				ctx->stall_run++;
			}
		} else {
			ctx->stall_run = 0u;
		}
	}

	ctx->fan_stall = (ctx->stall_run >= c->stall_dwell_s);
	o.fan_stall = ctx->fan_stall;
	if (ctx->fan_stall) {
		o.flags |= THERMAL_FLAG_FAN_STALL;
	}

	if (out != NULL) {
		*out = o;
	}
	return 0;
}
