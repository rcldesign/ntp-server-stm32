/*
 * STS1000 "Meridian" — the web plane's GNSS-detail decisions, as pure logic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/net/, and deliberately free of every Zephyr, socket and
 * logging dependency so tests/host can compile it — the same arrangement as
 * net/sts_ppscorr.h, net/sts_ntp_keys.h and net/sts_ldap_ca.h.
 *
 * ---------------------------------------------------------------------------
 * What this is for
 * ---------------------------------------------------------------------------
 *
 * Spec §336 requires the web skyplot to mirror the local-UI renderer of §6.3,
 * and §371 requires identical data to feed both so the two views agree. The
 * source of that data is sts_gnss_sky() — the per-satellite NAV-SAT records the
 * GNSS thread caches because core/gnssmgr reduces the frame to counts.
 *
 * Three decisions sit between that cache and the REST document, and every one
 * of them is silent when it is wrong, which is why they live here rather than
 * inline in sts_web.c:
 *
 *   FRESHNESS   sts_app.h is explicit that `mono_ms` is when the frame was
 *               DECODED and that a receiver which has stopped talking leaves
 *               the last good frame in the cache. An unaged read therefore
 *               serves an hour-old sky as current, with a full satellite list
 *               and no indication at all that the receiver is gone. This is the
 *               defect the ui area already guards against; the web plane needs
 *               its own guard because it is a different consumer (below).
 *
 *   ADMISSION   UBX reports azimuth 0 and elevation 0 for a satellite it knows
 *               only from the almanac, so serving every NAV-SAT record paints a
 *               stack of phantom markers at due north on the horizon — exactly
 *               where an operator looks for an obstruction. The rule is stated
 *               once here and pinned against the panel's by tests/host.
 *
 *   EVIDENCE    `detail_available` must mean what it says: TRUE when a fresh
 *               per-SV list was served (even if it is empty, which is what an
 *               indoor unit with a disconnected antenna legitimately reports),
 *               FALSE when there is no list to serve. A caller that cannot tell
 *               "no satellites visible" from "no data" cannot tell a working
 *               receiver under a metal roof from a dead one.
 *
 * ---------------------------------------------------------------------------
 * Why the staleness window is the net area's own constant
 * ---------------------------------------------------------------------------
 *
 * ui/sts_ui.c has SKY_STALE_MS for the panel. This is deliberately NOT that
 * constant, shared or included: the two are different consumers of the same
 * producer, and coupling them would make one view's cache policy the other's
 * problem — a change to the panel's render cadence would silently redefine what
 * an HTTPS client is told, in a file no web author reads.
 *
 * The web's own arithmetic, from the web's own numbers:
 *
 *   producer   UBX-NAV-SAT is configured at 1 Hz (spec §3.7), so a healthy
 *              receiver refreshes the cache every 1000 ms.
 *   consumer   the SPA polls REST at 1000 ms (web/app.js pollTimer) and the WSS
 *              telemetry push runs at the subscriber's rate_hz, 1 Hz by
 *              default. A backgrounded browser tab is throttled to >= 1 s and
 *              in practice much more.
 *   transport  unlike the panel, the answer crosses TLS, a network and a worker
 *              that serialises behind api_lock, and the client cannot re-ask
 *              instantly if it disbelieves what it got.
 *
 * A window shorter than producer + consumer + transport makes a healthy unit
 * intermittently report "no data", which is the worst possible failure for a
 * field that exists to distinguish absence from emptiness. 1000 + 1000 + jitter
 * puts the floor near 3 s; 5000 ms clears it with room for four consecutive
 * missed NAV-SAT frames.
 *
 * That it lands on the same number the panel uses is a result, not an input,
 * and it is a welcome one: the two views change their verdict at the same age,
 * so an operator with the panel in front of them and a laptop in their hand
 * does not see one view showing satellites the other has already blanked. If a
 * future argument moves either constant, §371 is what has to be re-argued —
 * tests/host/test_web_sky.c states that dependency as an assertion rather than
 * leaving it to a comment.
 *
 * The document also carries the AGE, which the panel cannot: an HTTP client is
 * remote and entitled to apply its own policy on top of this one. See
 * rest_gnss_t::sat_age_ms.
 */

#ifndef STS1000_ZEPHYR_NET_STS_WEB_SKY_H_
#define STS1000_ZEPHYR_NET_STS_WEB_SKY_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "fault/fault.h"
#include "quality/quality.h"
#include "web/rest.h"
#include "zephyr/sts_app.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Oldest NAV-SAT frame the web plane will serve as current, milliseconds.
 *
 * See the header comment for the derivation. This is the net area's number; the
 * ui area has its own.
 */
#define STS_WEB_SKY_STALE_MS 5000U

/*
 * The REST satellite array and the platform's sky cache must be able to hold
 * each other's worst case, or a full frame is silently clipped by whichever is
 * smaller. Both are 32 today (REST_SAT_MAX, STS_GNSS_SKY_MAX_SV, UI_MAX_SV);
 * this makes a divergence a compile error in the one file that would otherwise
 * absorb it. Not a BUILD_ASSERT: this header is Zephyr-free by construction.
 */
typedef char sts_web_sky_bound_check_t
	[(REST_SAT_MAX >= STS_GNSS_SKY_MAX_SV) ? 1 : -1];

/**
 * Is a cached NAV-SAT frame fresh enough to serve as the current sky?
 *
 * @param now_ms         sts_mono_ms() at the moment of the request.
 * @param frame_mono_ms  sts_gnss_sky_t::mono_ms; 0 means never received.
 *
 * A frame stamped in the future is refused rather than clamped. It cannot
 * happen from one monotonic clock, so if it is seen the snapshot is torn or the
 * caller passed the wrong clock — neither is evidence about the sky, and an
 * unsigned subtraction would turn it into an enormous age that passes no test
 * or a tiny one that passes every test depending on which way it wrapped.
 */
static inline bool sts_web_sky_fresh(uint64_t now_ms, uint64_t frame_mono_ms)
{
	if (frame_mono_ms == 0U) {
		return false;
	}
	if (now_ms < frame_mono_ms) {
		return false;
	}
	return (now_ms - frame_mono_ms) <= (uint64_t)STS_WEB_SKY_STALE_MS;
}

/**
 * Convert one UBX-NAV-SAT record into a REST satellite entry.
 *
 * @return true when the record should be served.
 *
 * THE ADMISSION RULE IS THE PANEL'S, RESTATED IN THE REST SHAPE — spec §371.
 * ui/sts_sky_policy.h sts_sky_sv_from_ubx() is the same decision producing a
 * ui_sv_t, and the two are pinned together by tests/host/test_web_sky.c over
 * the whole input space. It is restated rather than shared because
 * sts_app.h's area rule forbids the net area including a ui-private header,
 * and because the outputs genuinely differ: the panel maps gnssId onto a
 * constellation COLOUR (a lossy, display-only mapping), while REST carries the
 * receiver's own gnssId so a client can name a constellation the firmware has
 * never heard of.
 *
 * Rejected: a record the receiver is not actually measuring (C/N0 zero and not
 * in the solution). A satellite that IS in the timing solution is always
 * admitted regardless of its reported C/N0, because being used is stronger
 * evidence than a momentary zero.
 *
 * Normalised: azimuth is folded into 0..359. UBX may report 360, and a negative
 * value would be a receiver fault; both are wrapped rather than dropped, so a
 * marker lands at the right bearing instead of vanishing. Elevation below the
 * horizon is left alone — the renderers clamp it, and dropping it here would
 * hide a receiver reporting nonsense.
 */
static inline bool sts_web_sky_admit(uint8_t gnss_id, uint8_t sv_id,
				     int8_t elev_deg, int16_t azim_deg,
				     uint8_t cno_dbhz, bool used,
				     rest_sat_t *out)
{
	int32_t az;

	if (out == NULL) {
		return false;
	}
	if ((cno_dbhz == 0U) && !used) {
		return false;
	}

	az = (int32_t)azim_deg % 360;
	if (az < 0) {
		az += 360;
	}

	(void)memset(out, 0, sizeof(*out));
	out->gnss_id = gnss_id;
	out->sv_id = sv_id;
	out->cno = cno_dbhz;
	out->elev_deg = elev_deg;
	out->azim_deg = (int16_t)az;
	out->used = used;
	return true;
}

/**
 * Fill the satellite list, its age and `detail_available` from a sky snapshot.
 *
 * @param sky     The platform area's cache, or NULL (treated as "never").
 * @param now_ms  sts_mono_ms() at the moment of the request.
 * @param out     Written: sat[], n_sats, sat_age_ms, sat_age_valid,
 *                detail_available. Every other member is left alone, so the
 *                caller may fill the quality-derived fields in either order.
 *
 * The three states a caller must be able to tell apart, and how they encode:
 *
 *   never received   detail_available=false, sat_age_valid=false
 *   stale            detail_available=false, sat_age_valid=true, age > window
 *   fresh            detail_available=true,  sat_age_valid=true, n_sats >= 0
 *
 * A stale frame's SATELLITES are not served — serving them with a flag saying
 * not to trust them invites exactly the client that ignores the flag — but its
 * AGE is, because "the receiver last spoke 400 seconds ago" is the single most
 * useful thing the document can say when the list is empty.
 */
static inline void sts_web_sky_fill(const sts_gnss_sky_t *sky, uint64_t now_ms,
				    rest_gnss_t *out)
{
	uint8_t n = 0U;
	uint8_t i;

	if (out == NULL) {
		return;
	}
	out->n_sats = 0U;
	out->sat_age_ms = 0U;
	out->sat_age_valid = false;
	out->detail_available = false;
	if (sky == NULL) {
		return;
	}

	if ((sky->mono_ms != 0U) && (now_ms >= sky->mono_ms)) {
		uint64_t age = now_ms - sky->mono_ms;

		out->sat_age_ms = (age > (uint64_t)UINT32_MAX)
					  ? UINT32_MAX
					  : (uint32_t)age;
		out->sat_age_valid = true;
	}

	if (!sts_web_sky_fresh(now_ms, sky->mono_ms)) {
		return;
	}

	/* Fresh: the list is authoritative even when it is empty. */
	out->detail_available = true;

	for (i = 0U; (i < sky->count) && (i < (uint8_t)STS_GNSS_SKY_MAX_SV) &&
		     (n < (uint8_t)REST_SAT_MAX);
	     i++) {
		const sts_gnss_sv_t *s = &sky->sv[i];

		if (sts_web_sky_admit(s->gnss_id, s->sv_id, s->elev_deg,
				      s->azim_deg, s->cno_dbhz, s->used,
				      &out->sat[n])) {
			n++;
		}
	}
	out->n_sats = n;
}

/**
 * The antenna verdict.
 *
 * @param alarms         sts_alarms_active().
 * @param quality_flags  quality_block_t::flags.
 * @param short_latched  sts_gnss_detail_t::ant_short_latched — the supervisor
 *                       has cut the bias and only an operator undoes it. Pass
 *                       false when sts_gnss_detail() could not answer.
 * @param bias_on        the `gnss.ant.bias` key, i.e. whether bias is COMMANDED
 *                       on — not whether it is flowing.
 *
 * core/gnssmgr owns the real five-state supervisor (gnssmgr_ant_state_t), fused
 * from UBX-MON-RF, GPS_ANT_OFF_MON and the antenna INA228. Two of its outputs
 * cross the area seam and are used here; a third deliberately is not:
 *
 *   USED  the pair of alarms it raises, FAULT_ALARM_ANTENNA_OPEN and
 *         FAULT_ALARM_ANTENNA_SHORT, posted through sts_alarm_set() by
 *         platform/gnss.c's alarm callback. These are the DEBOUNCED verdicts —
 *         the same ones the alarm page and the front panel see.
 *   USED  `ant_short_latched`, which is a latch and therefore debounced by
 *         construction: it survives the fault clearing, because the bias stays
 *         cut until sts_gnss_ant_reenable().
 *   NOT   sts_gnss_detail_t::rf_ant_short / rf_ant_open, the raw UBX-MON-RF
 *         antStatus bits. The supervisor exists precisely to debounce those,
 *         and consuming them here would make the web view flap on transients
 *         the alarm page and the panel both ride out — a §371 divergence
 *         introduced in the name of freshness.
 *
 * gnssmgr_ant_state_t itself is not consumed either: sts_app.h publishes it as
 * a bare uint8_t precisely so the seam does not drag core/gnssmgr's header into
 * every area (ARCHITECTURE.md §2), and decoding the number here would put that
 * enum's values in a second file with nothing to keep them in step.
 *
 * Precedence, and why:
 *
 *   1. SHORT beats everything, alarm or latch. It is the destructive fault, and
 *      gnssmgr CUTS THE BIAS when it latches one. Testing `bias_on` first would
 *      therefore erase the reason the bias is off the instant the supervisor
 *      acted — the operator would see "unknown" for the one fault that needs a
 *      hand, and "re-enable bias" is the action that clears it.
 *   2. OPEN next: a real, reported fault outranks any inference.
 *   3. Bias commanded off -> UNKNOWN. With no bias there is no current to
 *      measure, so "ok" would be a claim about an unpowered LNA.
 *   4. Otherwise the receiver's own time lock is the evidence: an antenna
 *      delivering a usable timing solution is working, whatever else is true.
 *   5. No lock and no fault -> UNKNOWN, not OPEN. A cold start under a roof is
 *      not a cabling fault, and REST_ANT_OPEN is what the SPA paints red.
 */
static inline uint8_t sts_web_ant_state(uint64_t alarms, uint32_t quality_flags,
					bool short_latched, bool bias_on)
{
	if (short_latched ||
	    ((alarms & FAULT_ALARM_BIT(FAULT_ALARM_ANTENNA_SHORT)) != 0U)) {
		return (uint8_t)REST_ANT_SHORT;
	}
	if ((alarms & FAULT_ALARM_BIT(FAULT_ALARM_ANTENNA_OPEN)) != 0U) {
		return (uint8_t)REST_ANT_OPEN;
	}
	if (!bias_on) {
		return (uint8_t)REST_ANT_UNKNOWN;
	}
	if ((quality_flags & QUALITY_FLAG_GNSS_TIME_LOCKED) != 0U) {
		return (uint8_t)REST_ANT_OK;
	}
	return (uint8_t)REST_ANT_UNKNOWN;
}

/** Convert a UBX 0.1 mm accuracy to millimetres, rounded to nearest. */
static inline uint32_t sts_web_0p1mm_to_mm(uint32_t v)
{
	/* Not (v + 5) / 10: v may be UINT32_MAX and the add would wrap, turning
	 * the widest possible accuracy into 0 mm — "perfectly surveyed". */
	return (v / 10U) + (((v % 10U) >= 5U) ? 1U : 0U);
}

/**
 * Fill the survey-in and stored-position members from the receiver detail.
 *
 * @param d    sts_gnss_detail(), or NULL when it could not answer (-ENODEV: no
 *             GNSS thread). NULL leaves everything idle/invalid, which is the
 *             truth for an image with no receiver.
 * @param out  Written: survey_*, ecef_*, position_valid. Nothing else.
 *
 * State mapping, in this order:
 *
 *   svin_active            -> ACTIVE. A survey in progress outranks whatever
 *                             position is still stored, because the receiver is
 *                             no longer using it.
 *   otherwise pos_valid    -> FIXED. A stored antenna position IS fixed-position
 *                             timing mode (spec §3.7); it does not matter
 *                             whether it was surveyed, seeded or operator-set.
 *   otherwise              -> IDLE.
 *
 * The quality block's `gnss_fix` is NOT used for this and must not be:
 * disc_thread.c returns QUALITY_GNSS_TIME_ONLY whenever the receiver reports
 * time lock, including mid-survey and in 3D nav mode, so deriving FIXED from it
 * would report a surveying unit as already fixed.
 *
 * UNITS. UBX reports survey and position accuracy in 0.1 mm; the REST contract
 * carries millimetres. A missed conversion here is a factor of ten on the one
 * number an operator compares against `gnss.survey.acc` to decide whether the
 * site is commissioned — and it looks entirely plausible either way.
 */
static inline void sts_web_sky_survey(const sts_gnss_detail_t *d,
				      rest_gnss_t *out)
{
	if (out == NULL) {
		return;
	}
	out->survey_state = (uint8_t)REST_SURVEY_IDLE;
	out->survey_dur_s = 0U;
	out->survey_obs = 0U;
	out->survey_acc_mm = 0U;
	out->position_valid = false;
	out->ecef_x_cm = 0;
	out->ecef_y_cm = 0;
	out->ecef_z_cm = 0;
	if (d == NULL) {
		return;
	}

	if (d->svin_active) {
		out->survey_state = (uint8_t)REST_SURVEY_ACTIVE;
	} else if (d->pos_valid) {
		out->survey_state = (uint8_t)REST_SURVEY_FIXED;
	}

	out->survey_dur_s = d->svin_dur_s;
	out->survey_obs = d->svin_obs;
	/*
	 * While a survey runs, NAV-SVIN's own meanAcc is the live figure. Once
	 * it has finished the survey block stops advancing, so the accuracy of
	 * the position actually IN FORCE is the stored position's — reporting
	 * the last survey's number for a position seeded from NVS or set by an
	 * operator would attribute an accuracy to it that nothing measured.
	 */
	out->survey_acc_mm = sts_web_0p1mm_to_mm(
		d->svin_active ? d->svin_acc_0p1mm : d->pos_acc_0p1mm);
	if (d->svin_active && (d->svin_acc_0p1mm == 0U)) {
		/* A survey that has not yet produced a meanAcc: fall back to the
		 * survey block rather than reporting the stale stored figure. */
		out->survey_acc_mm = 0U;
	}

	out->position_valid = d->pos_valid;
	out->ecef_x_cm = (int64_t)d->pos_x_cm;
	out->ecef_y_cm = (int64_t)d->pos_y_cm;
	out->ecef_z_cm = (int64_t)d->pos_z_cm;
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_NET_STS_WEB_SKY_H_ */
