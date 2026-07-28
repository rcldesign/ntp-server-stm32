/*
 * STS1000 "Meridian" — the panel's reference, antenna and survey bindings.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * The Clocks page prints "Rb power ON / Rb lock LOCKED / Ext ref ... MHz" and
 * the Sky page prints "Antenna OK   Survey 1832 s acc 41 mm". Until the
 * platform published a sequencer view and a receiver detail, build_health()
 * had nothing to fill any of them from, and rather than leave them blank it
 * inferred them from quality_block_t::active_ref:
 *
 *     if (q->active_ref == QUALITY_REF_RB) {
 *             h->rb_present = true; h->rb_powered = true; h->rb_locked = true;
 *     } else if (q->active_ref == QUALITY_REF_EXTREF) {
 *             h->extref_ok = true;
 *     }
 *
 * Three of those five assignments are wrong in the case an operator walks up to
 * the box for. active_ref is what the discipline loop SELECTED; RB_LOCK (PB13)
 * and EXTREF_MON (PB14/TIM12) are what the hardware REPORTS. A unit whose FE
 * has dropped lock but has not yet been switched away printed "Rb lock LOCKED"
 * on its own front panel. A unit with a good house standard connected but not
 * selected printed "Ext ref not in band". And a unit running on the OCXO
 * printed "Rb power ABSENT" whether or not a rubidium was fitted and powered —
 * the Rb page said nothing at all about the Rb.
 *
 * Zephyr-free, beside sts_sky_policy.h and for the same reason: every one of
 * these mappings is silent when it is wrong (a panel does not fail, it just
 * reads the wrong thing) and none of them is reachable through sts_ui.c, which
 * is threads, displays and devicetree.
 */

#ifndef STS1000_ZEPHYR_UI_STS_UI_REFS_H_
#define STS1000_ZEPHYR_UI_STS_UI_REFS_H_

#include <stdbool.h>
#include <stdint.h>

#include "gnssmgr/gnssmgr.h"
#include "ui/ui.h"
#include "zephyr/sts_app.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Map core/gnssmgr's antenna verdict onto the panel's.
 *
 * The two enumerations carry the same five verdicts in DIFFERENT ORDER —
 * gnssmgr is {UNKNOWN, OFF, OK, OPEN, SHORT} and ui.h is {UNKNOWN, OK, OPEN,
 * SHORT, OFF} — so a cast compiles, runs, and renders a commanded-off antenna
 * as "OK" and an open one as "SHORT". Hence a switch with no default fallthrough
 * to a plausible value: anything unrecognised is UNKNOWN, which is the only
 * honest answer for a verdict this table does not know.
 */
static inline uint8_t sts_ui_ant_state(uint8_t gnssmgr_ant)
{
	switch (gnssmgr_ant) {
	case (uint8_t)GNSSMGR_ANT_OFF:
		return (uint8_t)UI_ANT_OFF;
	case (uint8_t)GNSSMGR_ANT_OK:
		return (uint8_t)UI_ANT_OK;
	case (uint8_t)GNSSMGR_ANT_OPEN:
		return (uint8_t)UI_ANT_OPEN;
	case (uint8_t)GNSSMGR_ANT_SHORT:
		return (uint8_t)UI_ANT_SHORT;
	case (uint8_t)GNSSMGR_ANT_UNKNOWN:
	default:
		return (uint8_t)UI_ANT_UNKNOWN;
	}
}

/**
 * Fill the panel's RUBIDIUM / EXTERNAL block from the sequencer's view.
 *
 * @param h  Health block under construction; only the reference members are
 *           touched.
 * @param p  sts_pwrseq_snapshot(), or NULL when it could not answer. NULL
 *           leaves the block as it was — for a memset() caller that is
 *           "absent, off, unlocked, no external reference", which is the truth
 *           for an image whose sequencer never started.
 *
 * `rb_present` is the configured intent (cfg `pwr.rb.policy`, carried as
 * sts_pwrseq_snap_t::rb_wanted), because nothing on this board can sense an FE
 * that is not powered. So the three panel states read: ABSENT = no rubidium
 * configured for this unit; OFF = configured, rail down; ON = rail up. Which is
 * what a technician means by the question.
 */
static inline void sts_ui_refs_from_pwrseq(ui_health_t *h,
					   const sts_pwrseq_snap_t *p)
{
	if ((h == NULL) || (p == NULL) || !p->started) {
		return;
	}

	h->rb_present = p->rb_wanted;
	h->rb_powered = p->rb_enabled;
	h->rb_locked = p->rb_lock_pin;
	h->extref_ok = p->extref_in_band;
	h->extref_hz = p->extref_hz;
}

/**
 * Fill the panel's antenna verdict and survey progress from the receiver.
 *
 * @param h  Health block under construction.
 * @param g  sts_gnss_detail(), or NULL when it could not answer (-ENODEV: no
 *           GNSS thread). NULL leaves the antenna UNKNOWN and the survey idle.
 *
 * Survey accuracy uses the same selection as the web and MP planes: the live
 * meanAcc while a survey runs, the stored position's accuracy once it has
 * finished. See sts_web_sky_survey() for the argument; the three are pinned
 * against each other in tests/host.
 */
static inline void sts_ui_refs_from_gnss(ui_health_t *h,
					 const sts_gnss_detail_t *g)
{
	if ((h == NULL) || (g == NULL)) {
		return;
	}

	h->ant_state = sts_ui_ant_state(g->ant_state);
	h->survey_active = g->svin_active;
	h->survey_dur_s = g->svin_dur_s;
	h->survey_acc_mm = sts_gnss_acc_0p1mm_to_mm(
		g->svin_active ? g->svin_acc_0p1mm : g->pos_acc_0p1mm);
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_UI_STS_UI_REFS_H_ */
