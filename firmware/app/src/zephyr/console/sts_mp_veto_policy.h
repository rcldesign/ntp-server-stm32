/*
 * STS1000 "Meridian" — the firmware-veto translation table (console area).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/console/ except for the enum, which mp_glue.h re-exports
 * so the platform area can name a subject. Zephyr-free so tests/host can drive
 * it, the same arrangement as sts_recovery_policy.h and sts_mp_evq.h beside it.
 *
 * ---------------------------------------------------------------------------
 * The layering problem this file solves
 * ---------------------------------------------------------------------------
 * FMT §5.1.2: "firmware remains the authority. An override is a lease, not a
 * transfer: fault supervision keeps running and may veto or revert any
 * override". core/mp implements that — mp_ovr_veto() drops the lease, calls the
 * apply callback to revert the pin, raises MP_OVR_EV_VETO on channel 0x09 and
 * writes the audit line — and it takes a MANIFEST OBJECT INDEX.
 *
 * The platform area, where every unsafe condition is actually detected, does not
 * know the manifest. It knows rails, fault_sig_t, pwrseq actions and alarm bits.
 * The console area owns mp_ctx_t and the object table. So neither side can raise
 * a veto on its own, and pushing the manifest down into platform would put a
 * console data structure on the 4 Hz sequencer's path.
 *
 * Resolved the way every other seam here is: **platform publishes, console
 * binds.** Platform raises a subject naming what it just did to the board;
 * mp_glue.c maps the subject to manifest indices and calls mp_veto(). This file
 * is the map, and it holds BOTH columns on purpose — the pwrseq action on one
 * side and the manifest object ids on the other. Split across two headers, a row
 * goes missing on one side and nothing says so; here the two are read together
 * and one host suite proves both.
 *
 * ---------------------------------------------------------------------------
 * Scope: the OFF direction, and only where firmware has already acted
 * ---------------------------------------------------------------------------
 * A veto is raised exactly where firmware is about to drive a pin an override
 * may be holding the other way — i.e. at pwrseq's action executor, which is the
 * single choke point for every rail pwrseq owns, plus the one out-of-band writer
 * that does not go through the action queue (the antenna-bias supervisor).
 *
 * Only the OFF/shed direction is mapped, and the asymmetry is deliberate. When
 * firmware takes a load DOWN it is because the hardware has already acted (the
 * autonomous 26 V latch), or because it must act (a shed ladder, a latched
 * short); the lease cannot be honoured and standing until its keepalive lapses
 * is a lie about the board. Firmware bringing a load UP under a lease that wants
 * it off is the weaker, symmetrical case, nobody has asked for it, and mapping
 * it would fire a veto on every bring-up (stage 6/7 emit DISP_EN and
 * PANEL_LED_EN before any session can exist). If that case ever needs covering
 * it is a row here, not a redesign.
 *
 * KNOWN CONSEQUENCE OF THAT ASYMMETRY, now that the rails are grantable: a
 * lease holding a rail OFF is not withdrawn when a stage brings that rail up.
 * The pin follows firmware — it always does, because a lease is applied once at
 * grant and once at revert and never re-asserts itself — so the board is right
 * and the lease's claim is stale until it lapses or is released, at which point
 * the drain re-drives firmware's own level anyway. It is visible rather than
 * hidden: the read-backs report the PIN (mp_glue.c obj_read), not the lease.
 *
 * NOT mapped, with the evidence, so these are recognisable as decisions:
 *
 *   - THE POWER-FAIL PARK LIST is not given a trigger of its own.
 *     sts_pwrseq_pfi() runs the park list through the very same executor, so its
 *     RB_VCC_GATE_DIS / RB_PWR_DIS *do* raise STS_MP_VETO_RB_*; what it does not
 *     get is a synchronous path. It could not use one: PFI leaves ~4.8 ms of
 *     hold-up (docs/sts1000_power_fail_input.md §3.5, and the reason
 *     pwrseq_pfi() discards the queue), the drain is on the 250 ms console
 *     supervisor, and the engine mutex may be held by the shell thread — so a
 *     synchronous veto would either miss the window or park the park list behind
 *     a maintenance request. The staged bits simply never drain, which is
 *     honest: nothing was withdrawn because the board stopped. Nothing is lost
 *     by that. A lease does not re-assert itself — obj_apply() is called once at
 *     grant and once at revert — so an override cannot fight the park list at
 *     the pin, and FMT §5.1.3 reverts everything at the next boot anyway.
 *
 *   - THE VCC_RB SETPOINT objects are not in the RUBIDIUM RAIL row, which is a
 *     narrower statement than it used to be here. pwrseq's rail drop is
 *     RB_PWR_EN and RB_VCC_GATE; it does not touch the digipot, so a setpoint
 *     lease is not contrary to THAT action, and `pwr.rb.vset_mv` additionally
 *     carries MP_ILK_RB_VERIFY, so mp_ovr_tick()'s read-back drops it on its
 *     own ("VCC_RB out of window") once the rail goes away. What it IS contrary
 *     to is pwrseq writing the wiper itself, in stage 8 — and that is
 *     DELIBERATELY still not mapped, on the rule stated above rather than by
 *     omission: PWRSEQ_ACT_DIGIPOT_WRITE runs on every bring-up, so a subject
 *     for it would raise a veto on every boot and make `mp status`'s veto
 *     counters mean nothing. The residue is the same shape as the ON-direction
 *     caveat and is covered the same way — the read-back reports the code the
 *     part actually holds (sts_pwrseq_rb_expected_mv), and if the rewrite moves
 *     the RAIL, MP_ILK_RB_VERIFY drops the lease on its own. `pwr.rb.pot.code`
 *     is in neither: it is refused outright (mp_glue.c prov_ilk), so no lease
 *     on it can exist to withdraw.
 *
 * ---------------------------------------------------------------------------
 * Why the reason strings are constants here and not passed by the caller
 * ---------------------------------------------------------------------------
 * The reason reaches two places: MP_EV_TEXT_MAX (32) bytes of the channel-0x09
 * record, and the LOGR_WARN audit line ovr_evt() writes. Keeping it a constant
 * per subject means the seam carries no pointer across a thread boundary and no
 * copy — the staging side is one atomic OR of one bit — and it makes the
 * strings reviewable in one place against that 31-character budget, which
 * sts_mp_veto_reason_fits() asserts and the host suite checks for every subject.
 */

#ifndef STS1000_ZEPHYR_CONSOLE_STS_MP_VETO_POLICY_H_
#define STS1000_ZEPHYR_CONSOLE_STS_MP_VETO_POLICY_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "mp/mp_stream.h"
#include "pwrseq/pwrseq.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * What firmware just did to the board, in the platform area's own terms.
 *
 * One value per *actuation*, not per cause, with the single exception of the
 * rubidium: the autonomous 26 V latch is the case FMT §5.5 calls out by name and
 * a technician reading "pwrseq dropped the Rb rail" when the hardware killed it
 * would be told the wrong thing. Everything else has one honest sentence.
 *
 * STS_MP_VETO_NONE is 0 so that "no veto" is the falsy default of a lookup, and
 * so bit 0 of the pending mask is never used.
 */
typedef enum {
	STS_MP_VETO_NONE = 0,
	/** RB_PWR_EN / RB_VCC_GATE dropped by the autonomous 26 V OV latch. */
	STS_MP_VETO_RB_OV,
	/** RB_PWR_EN / RB_VCC_GATE dropped by the sequencer for any other
	 *  reason: intent withdrawn, rail out of window, shed rung 3. */
	STS_MP_VETO_RB_RAIL,
	/** ANT_BIAS_EN dropped by the antenna supervisor's short latch. */
	STS_MP_VETO_ANT_BIAS,
	/** DISP_EN dropped: shed rung 1, or a stage fail-action. */
	STS_MP_VETO_DISPLAY,
	/** PANEL_LED_EN / PANEL_LED_PWM dropped: shed rung 2, or a fail-action. */
	STS_MP_VETO_PANEL_LED,
	/** GPS_PWR_EN dropped by a stage fail-action. */
	STS_MP_VETO_GPS_RAIL,
	STS_MP_VETO_COUNT
} sts_mp_veto_t;

/** Bit position of @p s in the pending mask. Bit 0 is never set. */
#define STS_MP_VETO_BIT(s) (1UL << (unsigned int)(s))

/** Every subject bit, for the drain's bounds check. */
#define STS_MP_VETO_MASK_ALL                                                   \
	((1UL << (unsigned int)STS_MP_VETO_COUNT) - 2UL)

/**
 * The subject a pwrseq action raises, or STS_MP_VETO_NONE.
 *
 * @param action     pwrseq_action_t, as carried in pwrseq_act_t::action.
 * @param ov_latched pwrseq_ov_latched() at the moment the action is executed.
 *                   It is what separates "the hardware killed the rubidium"
 *                   from "the sequencer took it down", and both are true
 *                   statements about the same two actions — so it is read at
 *                   the call site rather than guessed here.
 */
static inline sts_mp_veto_t sts_mp_veto_of_action(uint16_t action,
						 bool ov_latched)
{
	switch ((pwrseq_action_t)action) {
	case PWRSEQ_ACT_RB_PWR_DIS:
	case PWRSEQ_ACT_RB_VCC_GATE_DIS:
		/* rb_shutdown() emits both, always in that order; one subject
		 * covering both objects makes the pair coalesce into a single
		 * veto rather than two that race for the same lease. */
		return ov_latched ? STS_MP_VETO_RB_OV : STS_MP_VETO_RB_RAIL;
	case PWRSEQ_ACT_ANT_BIAS_DIS:
		return STS_MP_VETO_ANT_BIAS;
	case PWRSEQ_ACT_DISP_DIS:
		return STS_MP_VETO_DISPLAY;
	case PWRSEQ_ACT_PANEL_LED_DIS:
		return STS_MP_VETO_PANEL_LED;
	case PWRSEQ_ACT_GPS_PWR_DIS:
		return STS_MP_VETO_GPS_RAIL;
	default:
		return STS_MP_VETO_NONE;
	}
}

/**
 * The sentence a technician reads, on channel 0x09 and in the audit log.
 *
 * Never NULL for a valid subject: mp_ovr_veto() substitutes "firmware veto" for
 * a NULL reason, and a veto that cannot say what happened is the shape of the
 * defect this whole path exists to close.
 */
static inline const char *sts_mp_veto_reason(sts_mp_veto_t s)
{
	switch (s) {
	case STS_MP_VETO_RB_OV:
		return "Rb 26 V OV latch tripped";
	case STS_MP_VETO_RB_RAIL:
		return "pwrseq dropped the Rb rail";
	case STS_MP_VETO_ANT_BIAS:
		return "antenna bias short latched";
	case STS_MP_VETO_DISPLAY:
		return "display shed (power/thermal)";
	case STS_MP_VETO_PANEL_LED:
		return "panel LED shed (power/thermal)";
	case STS_MP_VETO_GPS_RAIL:
		return "pwrseq dropped the GPS rail";
	case STS_MP_VETO_NONE:
	case STS_MP_VETO_COUNT:
	default:
		return NULL;
	}
}

/** True when @p s's reason survives MP_EV_TEXT_MAX intact. */
static inline bool sts_mp_veto_reason_fits(sts_mp_veto_t s)
{
	const char *r = sts_mp_veto_reason(s);

	return (r != NULL) && (strlen(r) <= (size_t)(MP_EV_TEXT_MAX - 1U));
}

/**
 * The manifest ids @p s withdraws.
 *
 * By id and not by index: the manifest is a build-time table whose ordering is
 * nobody's contract, and mp_obj_find() is the lookup that exists for exactly
 * this. A renamed object fails loudly at the lookup (and in the host suite,
 * which resolves every id below against the shipped manifest) instead of
 * silently vetoing whatever now sits at a hard-coded index.
 *
 * @param n_out Receives the count; set to 0 and NULL returned for a subject
 *              with no objects, so a caller that ignores the return still
 *              iterates zero times rather than off the end.
 */
static inline const char *const *sts_mp_veto_objects(sts_mp_veto_t s,
						    size_t *n_out)
{
	static const char *const rb[] = {
		"pwr.rb.en",   /* RB_PWR_EN    PB7 */
		"pwr.rb.gate", /* RB_VCC_GATE  PB1 */
	};
	static const char *const ant[] = {
		"pwr.ant.bias.en", /* ANT_BIAS_EN PC9 */
	};
	static const char *const disp[] = {
		"pwr.disp.en", /* DISP_EN PC11 */
	};
	/*
	 * Both panel objects, because PWRSEQ_ACT_PANEL_LED_DIS is one call to
	 * sts_panel_led_set(0) and that is the write BOTH objects name:
	 * `ui.panel.duty` is the LPTIM2_CH2 duty a dimmer lease sets, and the
	 * rail enable is folded into the same setter (pwrseq_exec.c's
	 * PANEL_LED_EN case says so). A lease on either is contrary to a dark
	 * panel — and `ui.panel.duty` is, today, the one object in this table
	 * that mp_glue.c's obj_apply() can actually grant a lease on.
	 */
	static const char *const panel[] = {
		"pwr.panel.led.en", /* PANEL_LED_EN  PC0 */
		"ui.panel.duty",    /* PANEL_LED_PWM PE0 */
	};
	static const char *const gps[] = {
		"pwr.gps.en", /* GPS_PWR_EN PC8 */
	};
	const char *const *ids = NULL;
	size_t n = 0U;

	switch (s) {
	case STS_MP_VETO_RB_OV:
	case STS_MP_VETO_RB_RAIL:
		ids = rb;
		n = sizeof(rb) / sizeof(rb[0]);
		break;
	case STS_MP_VETO_ANT_BIAS:
		ids = ant;
		n = sizeof(ant) / sizeof(ant[0]);
		break;
	case STS_MP_VETO_DISPLAY:
		ids = disp;
		n = sizeof(disp) / sizeof(disp[0]);
		break;
	case STS_MP_VETO_PANEL_LED:
		ids = panel;
		n = sizeof(panel) / sizeof(panel[0]);
		break;
	case STS_MP_VETO_GPS_RAIL:
		ids = gps;
		n = sizeof(gps) / sizeof(gps[0]);
		break;
	case STS_MP_VETO_NONE:
	case STS_MP_VETO_COUNT:
	default:
		break;
	}

	if (n_out != NULL) {
		*n_out = n;
	}
	return ids;
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_CONSOLE_STS_MP_VETO_POLICY_H_ */
