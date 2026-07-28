/*
 * STS1000 "Meridian" — console area: the operator-recovery surface's decisions.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/console/, and deliberately free of every Zephyr
 * dependency so tests/host can compile it — the same arrangement as
 * console/sts_fwupd_seam_policy.h and net/sts_secops_policy.h, and for the same
 * reason: each decision below is silent when it is wrong.
 *
 * sts_shell.c and mp_glue.c own the wires. This header owns the three
 * judgements those wires are steered by.
 *
 *   1. WHICH LATCH AN OPERATOR MAY CLEAR, AND WHAT THE ANSWER MEANS.
 *      fault_alarm_clear() has an asymmetry with sts_alarm_set() that is easy
 *      to copy wrongly: sts_alarm_set() REFUSES ids 0..31 because those mirror
 *      scanned pins and the 1 kHz scan owns them, whereas clearing one of those
 *      ids touches only the LATCH — a historical record the scan does not write
 *      — so it is legal and useful. Collapsing the two rules would leave a
 *      technician unable to acknowledge a power-good glitch that has long since
 *      gone away, which is most of what latches are for.
 *
 *      The outcomes need separating too. -ENOENT ("nothing was latched") is an
 *      idempotent success: the operator asked for a clean slate and has one.
 *      -EBUSY ("the condition is still true") is the safety property firing and
 *      must reach the operator as a refusal, because the alternative reading —
 *      "cleared" — is a display that says healthy over a live fault.
 *
 *   2. WHETHER A TEMPCO FIT MAY BE BELIEVED. disc_tempco_fit() is ordinary
 *      least squares and will happily return a slope for two points, for a
 *      2 degree sweep, or for a data set that is pure noise. R^2 = 1.0 for any
 *      two-point fit by construction. The number it returns goes into
 *      cal.tempco, which the discipline loop applies as FEED-FORWARD — so a
 *      wrong slope does not read wrong anywhere, it steers the oscillator the
 *      wrong way whenever the enclosure temperature moves. Refuse rather than
 *      print, exactly as core/cal refuses rather than clamps an INA trim.
 *
 *   3. WHICH RECOVERY ACTION AN MP OBJECT NAMES. Three manifest objects drive
 *      irreversible or service-affecting hardware, and the difference between
 *      them is one string compare: `pwr.poe.kill` cold-cycles the board,
 *      `pwr.rb.ov.reset` clears a latch, `ui.identify` blinks an LED. A router
 *      that pairs the wrong id with the wrong action is not a mis-render, it is
 *      a power cycle in the field. The mapping is a pure function here so the
 *      suite can assert it against the real manifest, guard classes included.
 */

#ifndef STS1000_ZEPHYR_CONSOLE_STS_RECOVERY_POLICY_H_
#define STS1000_ZEPHYR_CONSOLE_STS_RECOVERY_POLICY_H_

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================== *
 *  1. clearing a latched alarm
 * ===================================================================== */

/**
 * Highest alarm id the console may name, exclusive.
 *
 * Mirrors FAULT_ALARM_COUNT. Stated as a plain number rather than by including
 * fault.h so this header stays a statement of policy that the suite checks
 * AGAINST core/fault, instead of one that agrees with it by construction.
 */
#define STS_ALARM_ID_COUNT 49U

/** First software-raised alarm id; below this the ids mirror scanned pins. */
#define STS_ALARM_FIRST_SOFTWARE 32U

/** What `sts alarms clear <token>` was asked to do. */
typedef enum {
	/** Clear one named latch. */
	STS_ACLR_TARGET_ONE = 0,
	/** Clear every latch whose condition has gone away. */
	STS_ACLR_TARGET_ALL,
	/** The id is not an alarm; nothing is attempted. */
	STS_ACLR_TARGET_BAD,
} sts_aclr_target_t;

/**
 * Classify a clear request.
 *
 * @param all  The operator typed `all`.
 * @param id   The alarm id, when @p all is false.
 *
 * Scanned-signal ids (0..31) are ACCEPTED. That is the asymmetry with
 * sts_alarm_set() spelled out in this file's header: the scan owns the signal,
 * it does not own the latch.
 */
static inline sts_aclr_target_t sts_aclr_target(bool all, uint64_t id)
{
	if (all) {
		return STS_ACLR_TARGET_ALL;
	}
	if (id >= (uint64_t)STS_ALARM_ID_COUNT) {
		return STS_ACLR_TARGET_BAD;
	}
	return STS_ACLR_TARGET_ONE;
}

/** What fault_alarm_clear() actually did, in operator terms. */
typedef enum {
	/** The latch is gone. */
	STS_ACLR_CLEARED = 0,
	/** There was no latch. The operator's intent is already satisfied. */
	STS_ACLR_NOT_LATCHED,
	/** The condition is still true; the latch stands. */
	STS_ACLR_STILL_ACTIVE,
	/** Bad id, or no fault context. */
	STS_ACLR_REJECTED,
} sts_aclr_outcome_t;

/** Map fault_alarm_clear()'s return value onto an outcome. */
static inline sts_aclr_outcome_t sts_aclr_outcome(int rc)
{
	switch (rc) {
	case 0:
		return STS_ACLR_CLEARED;
	case -ENOENT:
		return STS_ACLR_NOT_LATCHED;
	case -EBUSY:
		return STS_ACLR_STILL_ACTIVE;
	default:
		return STS_ACLR_REJECTED;
	}
}

/**
 * True when the outcome must be reported as a command failure.
 *
 * NOT_LATCHED is deliberately not a failure: `clear` is idempotent and a script
 * that clears an alarm twice has not done anything wrong. STILL_ACTIVE is,
 * because the operator asked for something the device refused to do.
 */
static inline bool sts_aclr_is_failure(sts_aclr_outcome_t o)
{
	return (o == STS_ACLR_STILL_ACTIVE) || (o == STS_ACLR_REJECTED);
}

/** Short description of an outcome, for the console. Never NULL. */
static inline const char *sts_aclr_outcome_name(sts_aclr_outcome_t o)
{
	switch (o) {
	case STS_ACLR_CLEARED:
		return "cleared";
	case STS_ACLR_NOT_LATCHED:
		return "not latched";
	case STS_ACLR_STILL_ACTIVE:
		return "still active - not cleared";
	case STS_ACLR_REJECTED:
		return "rejected";
	default:
		return "?";
	}
}

/* ===================================================================== *
 *  2. accepting an OCXO tempco fit
 * ===================================================================== */

/**
 * Samples the console keeps for one tempco sweep.
 *
 * A chamber sweep of the enclosure's service range in 5 degree steps is about
 * twenty points; 24 is that with headroom, and 192 bytes of the console area's
 * static footprint is what it costs. Larger buys nothing — least squares over
 * 24 well-spread points is not noticeably worse than over 200.
 */
#define STS_TEMPCO_MAX_SAMPLES 24U

/** Fewest points a fit may be believed over. */
#define STS_TEMPCO_MIN_SAMPLES 6U

/**
 * Narrowest temperature sweep a fit may be believed over, in degrees Celsius.
 *
 * The slope is applied as feed-forward across the whole service range, so a fit
 * taken over a couple of degrees is an extrapolation by a factor of twenty. Ten
 * degrees is the point at which an oven-controlled oscillator's df/dT is large
 * enough against the measurement noise to be what the line is describing.
 */
#define STS_TEMPCO_MIN_SPAN_C 10.0f

/**
 * Least of the variance the line must explain.
 *
 * Below this the samples are not describing a temperature coefficient: they are
 * describing a loop that was still disciplining, a chamber that had not settled,
 * or a transcription error. Note R^2 is 1.0 for ANY two-point fit, which is why
 * the sample floor above exists as well and not instead.
 */
#define STS_TEMPCO_MIN_R2 0.80f

/**
 * Bound on the fitted slope, ppb per degree Celsius.
 *
 * This is the cfg schema's own bound on `cal.tempco` (0x0C0C, -100..+100). A
 * fit outside it cannot be stored, so printing it as a result to paste would be
 * offering the operator a value the very next command rejects.
 */
#define STS_TEMPCO_SLOPE_ABS_MAX 100.0f

/** Why a fit was refused, or STS_TEMPCO_OK. */
typedef enum {
	STS_TEMPCO_OK = 0,
	/** Fewer than STS_TEMPCO_MIN_SAMPLES points. */
	STS_TEMPCO_ERR_SAMPLES,
	/** The sweep is narrower than STS_TEMPCO_MIN_SPAN_C. */
	STS_TEMPCO_ERR_SPAN,
	/** The line explains less than STS_TEMPCO_MIN_R2 of the variance. */
	STS_TEMPCO_ERR_SCATTER,
	/** The slope is outside what cal.tempco can hold. */
	STS_TEMPCO_ERR_SLOPE,
} sts_tempco_verdict_t;

/** Short description of a verdict, for the console. Never NULL. */
static inline const char *sts_tempco_verdict_name(sts_tempco_verdict_t v)
{
	switch (v) {
	case STS_TEMPCO_OK:
		return "accepted";
	case STS_TEMPCO_ERR_SAMPLES:
		return "too few samples";
	case STS_TEMPCO_ERR_SPAN:
		return "temperature sweep too narrow";
	case STS_TEMPCO_ERR_SCATTER:
		return "scatter too large for a linear tempco";
	case STS_TEMPCO_ERR_SLOPE:
		return "slope outside the cal.tempco range";
	default:
		return "?";
	}
}

/**
 * Temperature span of a sample set, in degrees Celsius.
 *
 * @return max - min, or 0.0f for fewer than two samples.
 */
static inline float sts_tempco_span(const float *temp_c, size_t n)
{
	float lo;
	float hi;
	size_t i;

	if ((temp_c == NULL) || (n < 2U)) {
		return 0.0f;
	}
	lo = temp_c[0];
	hi = temp_c[0];
	for (i = 1U; i < n; i++) {
		if (temp_c[i] < lo) {
			lo = temp_c[i];
		}
		if (temp_c[i] > hi) {
			hi = temp_c[i];
		}
	}
	return hi - lo;
}

/**
 * Decide whether a completed fit may be offered as a calibration value.
 *
 * Ordered so the refusal names the FIRST thing wrong with the data set rather
 * than the last: an operator with four points does not need to be told about
 * R^2 as well.
 *
 * @param n        Points the fit ran over.
 * @param span_c   sts_tempco_span() of those points.
 * @param r2       Coefficient of determination from disc_tempco_fit().
 * @param slope    Fitted df/dT, ppb per degree Celsius.
 */
static inline sts_tempco_verdict_t sts_tempco_check(size_t n, float span_c,
						    float r2, float slope)
{
	float mag = (slope < 0.0f) ? -slope : slope;

	if (n < (size_t)STS_TEMPCO_MIN_SAMPLES) {
		return STS_TEMPCO_ERR_SAMPLES;
	}
	if (!(span_c >= STS_TEMPCO_MIN_SPAN_C)) {
		return STS_TEMPCO_ERR_SPAN;
	}
	/* Written as a negated >= so a NaN R^2 — which a degenerate fit can
	 * produce — refuses instead of comparing false and passing. */
	if (!(r2 >= STS_TEMPCO_MIN_R2)) {
		return STS_TEMPCO_ERR_SCATTER;
	}
	if (!(mag <= STS_TEMPCO_SLOPE_ABS_MAX)) {
		return STS_TEMPCO_ERR_SLOPE;
	}
	return STS_TEMPCO_OK;
}

/* ===================================================================== *
 *  3. which recovery action an MP object names
 * ===================================================================== */

/** A platform recovery action an MP manifest object is wired to. */
typedef enum {
	/** No recovery action; the caller answers MP_E_NOTSUP as before. */
	STS_RECOV_NONE = 0,
	/** Assert POE_KILL (PE15): board cold-cycle, PSE-driven recovery. */
	STS_RECOV_POE_KILL,
	/** Pulse RB_OV_RESET (PD3): clear the autonomous 26 V latch. */
	STS_RECOV_RB_OV_RESET,
	/** Blink the status RGB blue: operator locate. */
	STS_RECOV_IDENTIFY,
} sts_recov_act_t;

/** Manifest id of the board cold-cycle pulse. FMT §5.2 lists it under G3. */
#define STS_RECOV_ID_POE_KILL "pwr.poe.kill"
/** Manifest id of the rubidium over-voltage latch reset pulse (G2). */
#define STS_RECOV_ID_RB_OV_RESET "pwr.rb.ov.reset"
/** Manifest id of the locate beacon (G1). */
#define STS_RECOV_ID_IDENTIFY "ui.identify"

/**
 * The action `obj.pulse` on @p obj_id performs.
 *
 * Exhaustive by construction: anything not named here is STS_RECOV_NONE, so a
 * renamed or newly-added pulsable object cannot fall through into one of these
 * two by accident.
 */
static inline sts_recov_act_t sts_recov_pulse_action(const char *obj_id)
{
	if (obj_id == NULL) {
		return STS_RECOV_NONE;
	}
	if (strcmp(obj_id, STS_RECOV_ID_POE_KILL) == 0) {
		return STS_RECOV_POE_KILL;
	}
	if (strcmp(obj_id, STS_RECOV_ID_RB_OV_RESET) == 0) {
		return STS_RECOV_RB_OV_RESET;
	}
	return STS_RECOV_NONE;
}

/**
 * The action `obj.set` / `obj.override` on @p obj_id performs.
 *
 * Separate from the pulse router on purpose: the two RPCs reach different
 * manifest flags, and one table serving both would let a pulse id be driven by
 * a set — which for `pwr.poe.kill` would mean a cold cycle from an object the
 * manifest does not mark writable.
 */
static inline sts_recov_act_t sts_recov_bool_action(const char *obj_id)
{
	if (obj_id == NULL) {
		return STS_RECOV_NONE;
	}
	if (strcmp(obj_id, STS_RECOV_ID_IDENTIFY) == 0) {
		return STS_RECOV_IDENTIFY;
	}
	return STS_RECOV_NONE;
}

/**
 * Duration the locate beacon runs for when `ui.identify` is asserted, in
 * milliseconds.
 *
 * sts_supervisor_identify() takes a DEADLINE, not a flag, so an object whose
 * value is a boolean has to name one. Releasing the lease (or setting it to 0)
 * stops the beacon immediately, so this is only the fail-safe for a release
 * that never arrives — and it is set to MP_LEASE_TTL_MAX_MS, the longest a
 * lease can stand, so the beacon never stops before the lease that asked for it
 * while still being bounded without one.
 */
#define STS_RECOV_IDENTIFY_MS 600000U

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_CONSOLE_STS_RECOVERY_POLICY_H_ */
