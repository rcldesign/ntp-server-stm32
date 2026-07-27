/*
 * STS1000 "Meridian" — core/cal implementation.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See cal.h for what this module is and, more importantly, what it refuses to
 * pretend to be.
 */

#include <errno.h>
#include <stdint.h>

#include "cal/cal.h"
#include "ina228/ina228.h"

/** Ceiling on the reported error, so a tiny reference cannot overflow int32. */
#define CAL_ERR_PPM_SAT 2000000000

static void fill_error_ppm(cal_ina_out_t *out, uint32_t i_ref_ua, int32_t i_meas_ua)
{
	int64_t ppm;

	if (i_ref_ua == 0U) {
		out->error_ppm = 0;
		return;
	}

	ppm = (((int64_t)i_meas_ua - (int64_t)i_ref_ua) * 1000000LL) /
	      (int64_t)i_ref_ua;

	if (ppm > (int64_t)CAL_ERR_PPM_SAT) {
		ppm = (int64_t)CAL_ERR_PPM_SAT;
	} else if (ppm < -(int64_t)CAL_ERR_PPM_SAT) {
		ppm = -(int64_t)CAL_ERR_PPM_SAT;
	}

	out->error_ppm = (int32_t)ppm;
}

/** Set the refusal and return the errno the header documents for it. */
static int refuse(cal_ina_out_t *out, cal_ina_reason_t reason, int err)
{
	out->shunt_cal = 0U;
	out->reason = (uint8_t)reason;
	return err;
}

int cal_ina_trim(const cal_ina_in_t *in, cal_ina_out_t *out)
{
	uint32_t min_ref_ua;
	uint16_t trimmed = 0U;
	int rc;

	if (out == NULL) {
		return -EINVAL;
	}

	out->shunt_cal = 0U;
	out->error_ppm = 0;
	out->reason = (uint8_t)CAL_INA_ERR_ARG;

	if (in == NULL) {
		return -EINVAL;
	}
	/*
	 * A zero full scale or an empty/inverted band would make every check
	 * below vacuous, and a zero base calibration would compute a trim of 0 —
	 * the value the INA228 accepts silently and then reads 0 A forever
	 * (ina228.h). None of these can come from the rail table or the schema;
	 * they mean the caller assembled the request wrong.
	 */
	if ((in->fs_current_ua == 0U) || (in->base_shunt_cal == 0U) ||
	    (in->trim_min == 0U) || (in->trim_max < in->trim_min)) {
		return -EINVAL;
	}

	if (!in->meas_valid || (in->meas_age_ms > in->max_age_ms)) {
		return refuse(out, CAL_INA_ERR_NO_READING, -ENODATA);
	}

	/*
	 * Reference plausibility comes before the measurement so the operator is
	 * told about their own input first: a mistyped reference is far more
	 * likely than a broken monitor, and reporting the monitor would send them
	 * to the wrong bench.
	 */
	min_ref_ua = (uint32_t)(((uint64_t)in->fs_current_ua *
				 (uint64_t)CAL_INA_MIN_REF_PERMILLE_FS) /
				1000ULL);
	if (min_ref_ua == 0U) {
		min_ref_ua = 1U;
	}
	if ((in->i_ref_ua < min_ref_ua) || (in->i_ref_ua > in->fs_current_ua)) {
		fill_error_ppm(out, in->i_ref_ua, in->i_meas_ua);
		return refuse(out, CAL_INA_ERR_REF_RANGE, -ERANGE);
	}

	/*
	 * CURRENT is signed: the INA228 reads negative when the load sits on the
	 * far side of the shunt from the sense polarity. Zero gives no ratio at
	 * all. Either way there is nothing to trim against, and both are wiring
	 * or rail-selection errors rather than calibration errors.
	 */
	if (in->i_meas_ua <= 0) {
		fill_error_ppm(out, in->i_ref_ua, in->i_meas_ua);
		return refuse(out, CAL_INA_ERR_MEAS_RANGE, -ERANGE);
	}

	fill_error_ppm(out, in->i_ref_ua, in->i_meas_ua);

	/*
	 * One implementation of the arithmetic: core/ina228 owns the formula and
	 * the register's own 15-bit clamp, this module owns the policy band. The
	 * -ERANGE that clamp returns is deliberately not distinguished here — a
	 * ratio big enough to reach 0x7FFF is 8x nominal and fails the band check
	 * on the very next line anyway, so it gets the one refusal that describes
	 * what actually happened.
	 */
	rc = ina228_shunt_cal_trim(in->base_shunt_cal, in->i_ref_ua,
				   (uint32_t)in->i_meas_ua, &trimmed);
	if ((rc != 0) && (rc != -ERANGE)) {
		return refuse(out, CAL_INA_ERR_ARG, rc);
	}

	if ((trimmed < in->trim_min) || (trimmed > in->trim_max)) {
		return refuse(out, CAL_INA_ERR_DISAGREE, -ERANGE);
	}

	out->shunt_cal = trimmed;
	out->reason = (uint8_t)CAL_INA_OK;
	return 0;
}

const char *cal_ina_reason_name(uint8_t reason)
{
	static const char *const names[CAL_INA_REASON__COUNT] = {
		[CAL_INA_OK] = "ok",
		[CAL_INA_ERR_ARG] = "bad-argument",
		[CAL_INA_ERR_NO_READING] = "no-usable-reading",
		[CAL_INA_ERR_REF_RANGE] = "reference-out-of-range",
		[CAL_INA_ERR_MEAS_RANGE] = "measurement-out-of-range",
		[CAL_INA_ERR_DISAGREE] = "disagreement-beyond-trim-band",
	};

	if (reason >= (uint8_t)CAL_INA_REASON__COUNT) {
		return "?";
	}

	return names[reason];
}
