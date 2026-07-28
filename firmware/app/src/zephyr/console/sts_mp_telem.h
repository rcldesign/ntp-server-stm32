/*
 * STS1000 "Meridian" — the Maintenance Protocol telemetry record's platform
 * bindings.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * WHAT THIS IS FOR
 * ----------------
 * mp_telem_t declares keys 45-51 — core/fault's debounced signal bitmap, the
 * power sequencer's stage/shed/alarms, the GNSS manager's state and antenna
 * verdict, survey progress, and the reference bitmap with EXTREF_MON's
 * frequency — and mp_stream.c dutifully encodes all of them onto the wire. For
 * the whole life of the record so far, nothing wrote any of them. prov_telem()
 * memset the struct and filled the timing and health halves, and the rest went
 * out as zeros: a Field Maintenance Tool showed a power sequencer permanently
 * at stage 0 with no alarms and no shed level, and a receiver permanently in
 * state 0 with no survey progress, on every unit, including the ones that were
 * in trouble.
 *
 * The gap was never in the encoder or in the platform's state — it was in the
 * BINDING, which is one flat assignment block and therefore exactly the kind of
 * code that loses a field without anything noticing. So the binding lives here,
 * Zephyr-free, and tests/host/test_mp_telem.c drives real platform state
 * through it, encodes a record and decodes it back. A test that only proved an
 * accessor returned something would have passed against the defect.
 *
 * Every key of the record is now bound, here or in prov_telem(). `scan_state`
 * (key 45) was the last hold-out and it is bound below; what it is and why it
 * arrives through the sequencer's snapshot is stated on that assignment.
 */

#ifndef STS1000_ZEPHYR_CONSOLE_STS_MP_TELEM_H_
#define STS1000_ZEPHYR_CONSOLE_STS_MP_TELEM_H_

#include <stdbool.h>
#include <stdint.h>

#include "mp/mp_stream.h"
#include "quality/quality.h"
#include "refsel/refsel.h"
#include "zephyr/sts_app.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Recover core/refsel's selected state from the published quality block.
 *
 * NOT a derivation in the sense the RB_LOCK comment below warns about: the
 * discipline thread writes quality_block_t::active_ref *from* refsel_state()
 * with a total map (disc_thread.c, disc_build_env), so inverting it recovers
 * the selector's state exactly rather than approximating it. The refsel context
 * itself belongs to the discipline thread and has no published accessor, and a
 * second one would have to be kept in step with this one for no gain.
 *
 * QUALITY_REF_NONE is the pre-publication value — the discipline loop has not
 * run yet. REFSEL_OCXO_ACTIVE is the right answer there too and not a guess:
 * refsel's own initial state is OCXO_ACTIVE and MUX_SEL boots to input A.
 */
static inline uint8_t sts_mp_telem_refsel(uint8_t active_ref)
{
	switch (active_ref) {
	case (uint8_t)QUALITY_REF_RB:
		return (uint8_t)REFSEL_RB_ACTIVE;
	case (uint8_t)QUALITY_REF_EXTREF:
		return (uint8_t)REFSEL_EXTREF_ACTIVE;
	case (uint8_t)QUALITY_REF_OCXO:
	case (uint8_t)QUALITY_REF_NONE:
	default:
		return (uint8_t)REFSEL_OCXO_ACTIVE;
	}
}

/**
 * Bind keys 45, 46 and 49-51 from the power sequencer's published view.
 *
 * @param t           Record under construction; only the fields named below are
 *                    touched, so this composes with prov_telem()'s other fills.
 * @param p           sts_pwrseq_snapshot(), or NULL when it could not answer
 *                    (-ENODEV: the sequencer never started). NULL leaves the
 *                    sequencer fields zero, which is the truth for an image
 *                    whose sequencer is not running.
 * @param active_ref  quality_block_t::active_ref, for key 51 only.
 *
 * KEY 45 (scan_state) AND WHY IT COMES FROM THE SEQUENCER
 * ------------------------------------------------------
 * It is core/fault's debounced asserted bitmap — bit n = fault_sig_t n, so the
 * seven panel buttons, the encoder switch, touch and proximity, the three
 * RT9742 nFLG flags, the two backup power-goods, the eight rail power-goods and
 * the nine INA228 ALERTs, all in one word. Keys 43/44 carry the ALARM view of
 * the same evidence, which is not a substitute: alarms are aggregated, latched
 * and suppressed by the expected-off mask, so a rail firmware has deliberately
 * gated off vanishes from key 43 while still reading asserted here. Telling
 * "off because we switched it off" from "off because it failed" needs both.
 *
 * The bitmap lives behind the platform's fault lock and this header is
 * Zephyr-free, so it could not be read from here directly. It does not need to
 * be: the sequencer pass already reads it once per 4 Hz tick under that lock —
 * pwrseq_in_t::pg_mask and ::supercaps_charged are derived from it — and now
 * publishes it in the snapshot's observed half. No second accessor, no second
 * lock acquisition, and the bitmap a technician reads is provably the one the
 * stage machine decided from on that tick rather than a later scan that merely
 * arrived at the same time.
 *
 * `started` gates it with the rest of the observed half: an unstarted sequencer
 * publishes no observation at all, and a bitmap with no tick behind it would be
 * indistinguishable from "every signal clear".
 *
 * RB_LOCK AND EXTREF, AND WHY THEY ARE NOT active_ref
 * ---------------------------------------------------
 * This binding used to read
 *
 *     out->rb_lock   = (q.active_ref == QUALITY_REF_RB);
 *     out->extref_ok = (q.active_ref == QUALITY_REF_EXTREF);
 *
 * which is not the same signal and is wrong in the one case that matters.
 * active_ref is what the discipline loop SELECTED. RB_LOCK (PB13, through the
 * FE's opto and the per-unit polarity bit) and EXTREF_MON (PB14/TIM12) are what
 * the hardware REPORTS. A unit whose rubidium has dropped lock but has not yet
 * been switched away reported `rb_lock = true` — the reading that would have
 * told a technician what happened was the one reading it could not produce. And
 * `extref_ok` was false on every unit that was not actively running on the
 * external input, including every unit with a perfectly good house standard
 * connected.
 *
 * Both now come from sts_pwrseq_snap_t's OBSERVED half, which is the pin and
 * the TIM12 measurement as the sequencer read them on its last 4 Hz tick.
 * `rb_powered` is RB_PWR_EN as commanded, and `pfi` is the sequencer's record
 * of the power-fail early warning having fired.
 */
static inline void sts_mp_telem_bind_pwrseq(mp_telem_t *t,
					    const sts_pwrseq_snap_t *p,
					    uint8_t active_ref)
{
	if (t == NULL) {
		return;
	}

	t->refsel_state = sts_mp_telem_refsel(active_ref);

	if ((p == NULL) || !p->started) {
		return;
	}

	t->scan_state = p->scan_state;

	t->pwrseq_stage = p->stage;
	t->pwrseq_shed = p->shed;
	t->pwrseq_alarms = p->alarms;

	t->rb_lock = p->rb_lock_pin;
	t->rb_powered = p->rb_enabled;
	t->extref_ok = p->extref_in_band;
	t->extref_hz = p->extref_hz;
	t->pfi = p->pfi_seen;
}

/**
 * Bind keys 47 and 48 from the GNSS receiver's published detail.
 *
 * @param t  Record under construction.
 * @param g  sts_gnss_detail(), or NULL when it could not answer (-ENODEV: no
 *           GNSS thread). NULL leaves the receiver fields zero.
 *
 * `gnss_state` carries core/gnssmgr's own gnssmgr_state_t and `gnss_ant` its
 * gnssmgr_ant_state_t, unmapped. The host renders the names; inventing a
 * separate wire enumeration would mean two definitions of the same fact that
 * nothing keeps in step. The encodings are stated in the mp_stream.h key list.
 *
 * SURVEY ACCURACY follows the same rule as the REST plane's
 * (sts_web_sky_survey): while a survey runs, NAV-SVIN's own meanAcc is the live
 * figure; once it has finished the survey block stops advancing, so the
 * accuracy of the position actually IN FORCE is the stored position's.
 * Reporting the last survey's number for a position seeded from NVS or set by
 * an operator would attribute an accuracy to it that nothing measured — and the
 * two planes disagreeing about the number a technician compares against
 * `gnss.survey.acc` is its own defect. The rule is pinned against the web
 * plane's in tests/host/test_mp_telem.c.
 */
static inline void sts_mp_telem_bind_gnss(mp_telem_t *t,
					  const sts_gnss_detail_t *g)
{
	if ((t == NULL) || (g == NULL)) {
		return;
	}

	t->gnss_state = g->mgr_state;
	t->gnss_ant = g->ant_state;
	t->survey_dur_s = g->svin_dur_s;
	t->survey_acc_mm = sts_gnss_acc_0p1mm_to_mm(
		g->svin_active ? g->svin_acc_0p1mm : g->pos_acc_0p1mm);
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_CONSOLE_STS_MP_TELEM_H_ */
