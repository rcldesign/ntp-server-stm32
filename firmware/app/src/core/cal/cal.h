/*
 * STS1000 "Meridian" — core/cal: bench-calibration arithmetic and policy.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11 (ARCHITECTURE.md §4). No dynamic allocation, no globals,
 * no platform headers. Dependency edge: cal -> ina228, util.
 *
 * Scope, deliberately narrow: the ONE calibration a running unit can perform on
 * itself. The per-board INA228 SHUNT_CAL trim (root CLAUDE.md bench item, spec
 * §10.4, interface ref §4.2) is automatable because the operator supplies a
 * reference current from an external instrument and the board already measures
 * the same load — everything after that is arithmetic, and arithmetic belongs
 * here rather than in a glue file where it cannot be tested.
 *
 * The other three procedures the REST contract names are NOT here, because they
 * are not procedures a device can run on itself:
 *
 *   holdover characterisation   hours of GNSS-denied operation against a
 *                               reference clock (spec §10.4)
 *   OCXO pull-range / tempco    a thermal-chamber sweep correlating TMP117
 *                               against frequency error
 *   e-compass hard/soft-iron    the operator must rotate the enclosure through
 *                               a sphere while the magnetometer is sampled
 *
 * Each is a bench operation whose *result* is entered through the cfg keys in
 * group 0x0C. A firmware entry point that wrote such a key would be a config
 * set wearing a procedure's name.
 *
 * ---------------------------------------------------------------------------
 * Why the trim is bounded rather than merely computed
 * ---------------------------------------------------------------------------
 * ina228_shunt_cal_trim() does the ratio and clamps it to the register's 15-bit
 * field — 0x7FFF, eight times the nominal 4096. That is the right contract for a
 * codec and the wrong one for an operator-facing control: a mistyped reference
 * current produces a perfectly well-formed SHUNT_CAL that silently rescales
 * every current reading on that rail, with no error flag anywhere, until someone
 * re-runs the bench procedure. The root CLAUDE.md says exactly this about tap
 * asymmetry ("the per-board SHUNT_CAL trim silently absorbs such an error and
 * then lets it drift with temperature").
 *
 * So this module refuses rather than clamps. The band is the cfg schema's own
 * bound on cal.ina.0..8 (3277..4915 = 4096 ±20 %), which is far wider than the
 * 1 % shunt tolerance the trim exists to remove: a pair of currents implying
 * more than that is not a shunt tolerance, it is the wrong load, the wrong rail
 * or the wrong number of zeroes.
 *
 * Note the band is symmetric in SHUNT_CAL and therefore ASYMMETRIC in the error
 * it tolerates, because the trim is a reciprocal: SHUNT_CAL scales by
 * i_ref/i_meas, so ±20 % of 4096 accepts a monitor reading between 16.7 % LOW
 * (4096/4915) and 25.0 % HIGH (4096/3277). That is a property of the schema
 * row, not of this module, and it is stated here because "±20 %" reads as if it
 * were a bound on the measurement error, which it is not.
 */

#ifndef STS1000_CORE_CAL_CAL_H_
#define STS1000_CORE_CAL_CAL_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Schema bound on cal.ina.0..8 (cfg_schema.h 0x0C01..0x0C09): 4096 -20 %. */
#define CAL_INA_TRIM_MIN 3277u
/** Schema bound on cal.ina.0..8: 4096 +20 %. */
#define CAL_INA_TRIM_MAX 4915u

/**
 * Smallest reference load accepted, in per-mille of the rail's full scale.
 *
 * A trim is a ratio of two currents, so its precision is that of the smaller
 * one. At 10 % of full scale the INA228's own quantisation is ~0.01 % of the
 * reading (20-bit ADC over the ±40.96 mV range), which is two orders below the
 * 1 % the trim is removing; below that the ratio starts measuring the ADC. It
 * also catches the commonest operator error by far — a reference current typed
 * in the wrong unit, which lands three orders of magnitude low.
 */
#define CAL_INA_MIN_REF_PERMILLE_FS 100u

/** Why cal_ina_trim() refused, or CAL_INA_OK. */
typedef enum {
	CAL_INA_OK = 0,
	/** Bad argument: NULL, or a zero full scale / trim band. */
	CAL_INA_ERR_ARG,
	/** The monitor's own reading is unusable: invalid, uncalibrated, or
	 *  older than @p max_age_ms. Nothing to trim against. */
	CAL_INA_ERR_NO_READING,
	/** The reference current is zero, above the rail's full scale, or below
	 *  CAL_INA_MIN_REF_PERMILLE_FS of it. */
	CAL_INA_ERR_REF_RANGE,
	/** The measured current is zero or negative: no ratio exists, and a
	 *  negative reading means the load is on the wrong side of the shunt. */
	CAL_INA_ERR_MEAS_RANGE,
	/** Reference and measurement disagree by more than the trim band allows.
	 *  A 1 % shunt cannot produce this; the inputs are wrong. */
	CAL_INA_ERR_DISAGREE,
	CAL_INA_REASON__COUNT,
} cal_ina_reason_t;

/**
 * One INA228 trim measurement.
 *
 * @p base_shunt_cal MUST be the SHUNT_CAL that was actually in force when
 * @p i_meas_ua was measured, not the nominal 4096. The reported current is
 * proportional to the calibration in the part, so trimming from the nominal
 * value after a previous trim would throw the previous correction away and
 * converge on nothing. Re-running against the in-force value is idempotent.
 */
typedef struct {
	uint16_t base_shunt_cal; /**< calibration in force at measurement time */
	uint32_t i_ref_ua;       /**< the operator's reference instrument */
	int32_t  i_meas_ua;      /**< what the INA228 reported for the same load */
	bool     meas_valid;     /**< reading succeeded AND SHUNT_CAL confirmed */
	uint32_t meas_age_ms;    /**< age of that reading */
	uint32_t max_age_ms;     /**< oldest reading still accepted */
	uint32_t fs_current_ua;  /**< rail full scale, ina228_rail_info_t */
	uint16_t trim_min;       /**< inclusive band, normally CAL_INA_TRIM_MIN */
	uint16_t trim_max;       /**< inclusive band, normally CAL_INA_TRIM_MAX */
} cal_ina_in_t;

/** Result of cal_ina_trim(). */
typedef struct {
	/** The trimmed SHUNT_CAL. Meaningful only when @p reason is CAL_INA_OK;
	 *  zero otherwise, so a caller that ignores the return value writes an
	 *  obviously-wrong calibration rather than a plausible one. */
	uint16_t shunt_cal;
	/** Measured error before the trim, (i_meas - i_ref) / i_ref in parts per
	 *  million. Filled whenever both currents are usable, including on a
	 *  CAL_INA_ERR_DISAGREE refusal — that number is what tells the operator
	 *  whether they mistyped or the board is genuinely wrong. Saturates at
	 *  +-2e9 ppm rather than overflowing. */
	int32_t  error_ppm;
	uint8_t  reason; /**< cal_ina_reason_t */
} cal_ina_out_t;

/**
 * Compute a bounded per-board SHUNT_CAL trim.
 *
 *     shunt_cal = round(base_shunt_cal * i_ref / i_meas)
 *
 * accepted only when every plausibility condition above holds.
 *
 * @retval 0        Trim computed; @p out->reason is CAL_INA_OK.
 * @retval -EINVAL  @p in or @p out is NULL, or an input is structurally bad
 *                  (CAL_INA_ERR_ARG). @p out is still filled when non-NULL.
 * @retval -ENODATA The monitor has no usable reading (CAL_INA_ERR_NO_READING).
 * @retval -ERANGE  A current is outside its plausible range, or the pair
 *                  disagrees by more than the band (CAL_INA_ERR_REF_RANGE,
 *                  CAL_INA_ERR_MEAS_RANGE, CAL_INA_ERR_DISAGREE).
 */
int cal_ina_trim(const cal_ina_in_t *in, cal_ina_out_t *out);

/** Human-readable @p reason (cal_ina_reason_t), for logs. Never NULL. */
const char *cal_ina_reason_name(uint8_t reason);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_CAL_CAL_H_ */
