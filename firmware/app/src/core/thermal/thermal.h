/*
 * STS1000 "Meridian" — core/thermal: fan control and the over-temperature
 * shed ladder.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation, no platform headers, no globals,
 * no libm.
 *
 * Implements spec §10.2: a PI loop on enclosure temperature (TMP117 #2, 0x48)
 * driving `FAN_PWM` (PE5, TIM15_CH1, 25 kHz), tachometer feedback from
 * `FAN_TACH` (PA15), hysteresis and a minimum duty for bearing life, fail-safe
 * full speed, and stall/under-speed detection. The escalation ladder is §10.3:
 * alarm, then shed the rubidium, then request `POE_KILL`.
 *
 * Fail-safe direction, twice over. In hardware an undriven PE5 is full speed
 * (interface ref §1.1, ARCHITECTURE.md §10 invariant 9), so a hung MCU cooks
 * nothing. In software this module answers a missing or invalid enclosure
 * reading with 100 % duty rather than with its last good value: a sensor that
 * has stopped reporting is not evidence that the box is cool.
 *
 * All temperatures are milli-degrees Celsius, matching the TMP117 driver and
 * the rest of the telemetry path.
 */

#ifndef STS1000_CORE_THERMAL_THERMAL_H_
#define STS1000_CORE_THERMAL_THERMAL_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Published condition flags. */
#define THERMAL_FLAG_FAILSAFE   (1u << 0) /**< no usable enclosure reading */
#define THERMAL_FLAG_MIN_DUTY   (1u << 1) /**< output sitting on the floor */
#define THERMAL_FLAG_MAX_DUTY   (1u << 2) /**< output sitting on the ceiling */
#define THERMAL_FLAG_DEADBAND   (1u << 3) /**< inside the setpoint hysteresis */
#define THERMAL_FLAG_ALARM      (1u << 4) /**< over the alarm rung */
#define THERMAL_FLAG_SHED_RB    (1u << 5) /**< over the rubidium-shed rung */
#define THERMAL_FLAG_POE_KILL   (1u << 6) /**< over the cold-cycle rung */
#define THERMAL_FLAG_FAN_STALL  (1u << 7) /**< measured RPM below the model */
#define THERMAL_FLAG_TACH_FAULT (1u << 8) /**< no usable tachometer reading */

/* ------------------------------------------------------------------ config */

/**
 * Loop and ladder configuration. thermal_cfg_defaults() fills the documented
 * values; the `power`/`ui` config groups may override them.
 *
 * The PI gains are placeholders sized for a several-minute enclosure time
 * constant and MUST be trimmed on the bench (§10.2) — the plant is the
 * enclosure, not the die.
 */
typedef struct {
	int32_t setpoint_mc;      /**< regulation target (45000 = 45 °C) */
	int32_t deadband_mc;      /**< hysteresis half-width (1000 = +-1 °C) */
	float kp_pct_per_c;       /**< proportional gain, % duty per °C (15) */
	float ki_pct_per_c_s;     /**< integral gain, % duty per °C per s (0.05) */
	uint8_t min_duty_pct;     /**< floor for bearing life (20) */
	uint8_t max_duty_pct;     /**< ceiling (100) */

	int32_t alarm_mc;         /**< rung 1: annunciate (60000) */
	int32_t shed_rb_mc;       /**< rung 2: request Rb shed (70000) */
	int32_t kill_mc;          /**< rung 3: request POE_KILL (80000) */
	int32_t ladder_hyst_mc;   /**< release hysteresis on all rungs (2000) */

	uint16_t rpm_at_full;     /**< fan RPM at 100 % duty (6000) */
	uint8_t rpm_model_pct;    /**< fraction of the model that is acceptable (50) */
	uint16_t stall_rpm_floor; /**< absolute floor above stall_duty_pct (500) */
	uint8_t stall_duty_pct;   /**< stall detection applies above this duty (30) */
	uint8_t stall_dwell_s;    /**< consecutive seconds before the alarm (5) */
} thermal_cfg_t;

/* ----------------------------------------------------------------- context */

typedef struct {
	thermal_cfg_t cfg;
	bool initialised;

	float integ_pct;      /* integral term, percent of duty */
	uint8_t duty_pct;     /* last commanded duty */

	bool alarm;           /* latched ladder rungs, released with hysteresis */
	bool shed_rb;
	bool poe_kill;

	uint8_t stall_run;    /* consecutive seconds under the expected RPM */
	uint8_t tach_bad_run; /* consecutive seconds without a usable tach */
	bool fan_stall;

	uint64_t prev_mono_ms;
	bool have_prev;
} thermal_ctx_t;

/* ------------------------------------------------------------- input/output */

typedef struct {
	uint64_t mono_ms;

	int32_t enclosure_mc;  /**< TMP117 #2 (0x48) — the controlled variable */
	bool enclosure_valid;

	int32_t osc_mc;        /**< TMP117 #1 (0x49), reported not controlled */
	bool osc_valid;
	int32_t die_mc;        /**< STM32 internal sensor, reported */
	bool die_valid;

	uint16_t fan_rpm;      /**< FAN_TACH (PA15) */
	bool rpm_valid;
} thermal_in_t;

typedef struct {
	uint8_t duty_pct;         /**< value to program into TIM15_CH1 */
	bool failsafe;            /**< duty forced to maximum by a bad sensor */
	bool alarm_overtemp;      /**< rung 1 */
	bool request_rb_shed;     /**< rung 2 — pwrseq drops RB_PWR_EN */
	bool request_poe_kill;    /**< rung 3 — assert POE_KILL (PE15) */
	bool fan_stall;           /**< under-speed for stall_dwell_s */
	int32_t temp_mc;          /**< the temperature the loop acted on */
	uint16_t expected_rpm;    /**< model RPM the stall test used */
	uint32_t flags;           /**< THERMAL_FLAG_* */
} thermal_out_t;

/* --------------------------------------------------------------------- API */

/**
 * Fill @p cfg with the documented defaults.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p cfg is NULL.
 */
int thermal_cfg_defaults(thermal_cfg_t *cfg);

/**
 * Initialise the loop. The output starts at the configured maximum, which is
 * the fail-safe state the hardware already sits in before PE5 is driven.
 *
 * @param ctx  Context.
 * @param cfg  Configuration, or NULL for thermal_cfg_defaults().
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p ctx is NULL, or @p cfg fails validation (rung ordering,
 *                  duty range, non-positive gains, ...).
 */
int thermal_init(thermal_ctx_t *ctx, const thermal_cfg_t *cfg);

/**
 * Run one loop iteration. Nominally 1 Hz from `housekeeping`; the real elapsed
 * time is taken from @p in->mono_ms, so a slower or jittery cadence only
 * changes the integral step, not the behaviour.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p ctx or @p in is NULL, or the context is not initialised.
 */
int thermal_step_1hz(thermal_ctx_t *ctx, const thermal_in_t *in,
		     thermal_out_t *out);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_THERMAL_THERMAL_H_ */
