/*
 * STS1000 "Meridian" — the 1 kHz GPIOF/GPIOG scan's decisions, as pure logic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/platform/, and deliberately free of every Zephyr,
 * devicetree and logging dependency so tests/host can compile it — the same
 * arrangement as net/sts_ppscorr.h and console/sts_confirm_gate.h, for the same
 * reason: what it decides is invisible at runtime when it is wrong.
 *
 * ---------------------------------------------------------------------------
 * Why this particular seam
 * ---------------------------------------------------------------------------
 * io_scan.c reads two 16-bit input data registers once a millisecond and hands
 * the pair to core/fault, which owns the diff, the debounce and the event queue
 * — all of that is already covered by tests/host/test_fault.c. What was NOT
 * covered is everything io_scan.c does with the result:
 *
 *   1. Which signal number means which physical pin. core/fault's contract is
 *      "GPIOF is bits 0..15, GPIOG bits 16..31"; io_scan.c re-derives that split
 *      to look an ALERT signal up in ina228_rail_tbl[]. An off-by-one there does
 *      not fail — it silently attributes the OCXO monitor's alert to the
 *      rubidium rail, and housekeeping then re-reads the wrong part's DIAG_ALRT
 *      while the real fault goes unread.
 *
 *   2. What each debounced event turns into: a UI input, an I2C re-read
 *      request, a log line, an MP event-channel record, or nothing. Two of
 *      those pairings are invertible in a way nothing would notice — an
 *      EN_FAULT recovery logged at ERR reads as a second failure, and a
 *      backup-rail "good" logged as "not good" is a field diagnosis pointed at
 *      the wrong end of the board. And one of them was simply MISSING: nothing
 *      staged an MP event at all, so a technician subscribed to channel 0x09
 *      sat in silence through a power-good drop, an INA228 alert, a button
 *      press and a door opening, with silence indistinguishable from health.
 *
 *   3. When a dropped-event burst is worth annunciating. The queue drops the
 *      newest event and counts it (fault.h); the counter is cleared once the
 *      consumer has reported it, so "has anything been dropped" is the whole
 *      condition and any latch on top of it double-reports.
 *
 * None of these are reachable from a host test through io_scan.c itself: that
 * file is a devicetree GPIO table, a k_timer and a thread. They are reachable
 * here.
 *
 * ---------------------------------------------------------------------------
 * What stays behind in the .c
 * ---------------------------------------------------------------------------
 * The gpio_dt_spec tables, gpio_port_get_raw(), the k_timer pacing, the
 * semaphore, the fault-context lock and the actual sts_log()/sts_ui_post_input()
 * calls. This header decides; io_scan.c does.
 */

#ifndef STS1000_ZEPHYR_PLATFORM_STS_IO_POLICY_H_
#define STS1000_ZEPHYR_PLATFORM_STS_IO_POLICY_H_

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "fault/fault.h"
#include "ina228/ina228.h"
#include "logring/logring.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Scan period, milliseconds. One sample of both ports per tick. */
#define STS_IO_SCAN_PERIOD_MS 1U

/**
 * Port letters as ina228_rail_info_t records them.
 *
 * The rail table spells the ALERT location as a port letter plus a bit inside
 * that port's IDR, precisely so a consumer never has to hand-maintain a second
 * signal->rail table. These constants exist so the comparison below reads as a
 * lookup rather than as two magic characters.
 */
#define STS_IO_PORT_F 'F'
#define STS_IO_PORT_G 'G'

/** Signals per port in the combined fault_sig_t space. */
#define STS_IO_SIGS_PER_PORT 16U

/**
 * Split a scanned signal into its (port, bit) location.
 *
 * fault_sig_t is the concatenation of the two input data registers — GPIOF in
 * 0..15, GPIOG in 16..31 (fault.h). This is the only place io_scan re-derives
 * that, and it is the mapping every downstream attribution rests on.
 *
 * @retval 0        @p port and @p bit written.
 * @retval -EINVAL  @p sig is out of range, or an output pointer is NULL.
 */
static inline int sts_io_sig_port_bit(fault_sig_t sig, uint8_t *port, uint8_t *bit)
{
	unsigned int n = (unsigned int)sig;

	if (port == NULL || bit == NULL || n >= (unsigned int)FAULT_SIG_COUNT) {
		return -EINVAL;
	}

	if (n < STS_IO_SIGS_PER_PORT) {
		*port = (uint8_t)STS_IO_PORT_F;
		*bit = (uint8_t)n;
	} else {
		*port = (uint8_t)STS_IO_PORT_G;
		*bit = (uint8_t)(n - STS_IO_SIGS_PER_PORT);
	}

	return 0;
}

/**
 * Which of the nine monitors owns the ALERT on @p sig.
 *
 * Resolved against ina228_rail_tbl[]'s own alert_port/alert_bit fields rather
 * than a second hand-written table, so the scan and the register map cannot
 * drift apart — the GPS monitor being at 0x4A and the panel monitor's alert
 * being the one line on GPIOF are both facts this lookup inherits for free.
 *
 * @retval 0        @p out holds the rail.
 * @retval -EINVAL  bad @p sig or NULL @p out.
 * @retval -ENOENT  @p sig is not an INA228 ALERT line.
 */
static inline int sts_io_ina_rail_for_sig(fault_sig_t sig, ina228_rail_t *out)
{
	uint8_t port;
	uint8_t bit;
	int rc;

	if (out == NULL) {
		return -EINVAL;
	}

	rc = sts_io_sig_port_bit(sig, &port, &bit);
	if (rc != 0) {
		return rc;
	}

	for (size_t i = 0; i < (size_t)INA228_RAIL_COUNT; i++) {
		if (ina228_rail_tbl[i].alert_port == port &&
		    ina228_rail_tbl[i].alert_bit == bit) {
			*out = (ina228_rail_t)i;
			return 0;
		}
	}

	return -ENOENT;
}

/**
 * Did the previous scan slot overrun?
 *
 * Unsigned difference, so the 49.7-day k_uptime_get_32() rollover needs no
 * special case. One millisecond of slack is allowed because the scan is paced
 * by a 1 ms k_timer against a 1 ms-resolution clock: adjacent samples routinely
 * read 1 or 2 ms apart with nothing wrong, and counting those as overruns would
 * make the counter useless as a "the scan is being starved" indicator.
 */
static inline bool sts_io_scan_overrun(uint32_t now_ms, uint32_t last_ms,
				       uint32_t period_ms)
{
	return (uint32_t)(now_ms - last_ms) > (period_ms + 1U);
}

/* ------------------------------------------------------------- dispatch --- */

/**
 * UI input types this dispatcher can raise.
 *
 * Numerically identical to sts_app.h's sts_input_evt_t type enum, and duplicated
 * rather than included so this header stays free of the cross-area API (which
 * drags in core/mp and core/cfg for types nothing here uses). io_scan.c carries
 * BUILD_ASSERTs that the two agree, so the copy cannot drift — the same
 * arrangement sts_ppscorr.h uses for the TAI-GPS constant.
 */
typedef enum {
	STS_IO_UI_NONE = -1,
	STS_IO_UI_BUTTON = 0,
	STS_IO_UI_BUTTON_LONG = 1,
	STS_IO_UI_BUTTON_REPEAT = 2,
	STS_IO_UI_TOUCH = 3,
	STS_IO_UI_PROX = 4,
} sts_io_ui_type_t;

/**
 * MP event-channel kinds this dispatcher can raise (FMT §7.5, channel 0x09).
 *
 * Numerically identical to core/mp's mp_ev_kind_t and duplicated for the same
 * reason the UI types above are: this header stays free of the cross-area API
 * and of core/mp, whose stream header drags in the frame layer and core/quality
 * for types nothing here uses. io_scan.c carries BUILD_ASSERTs against the real
 * enum, so the copy cannot drift.
 *
 * The four kinds below are the ones a *scanned signal* can produce. MP_EV_ALARM
 * is deliberately absent: an alarm is the debounced signal's entry in the alarm
 * table, not the edge itself, and its assert/clear transition is decided in
 * sts_alarm_set() — which is also the only producer of the software-raised ids
 * (>= 32) that have no scanned signal at all. Raising it here as well would
 * report the same transition twice under two kinds, and would do it without the
 * expected-off masking that decides whether a scanned signal is a fault or a
 * rail firmware deliberately gated off.
 */
typedef enum {
	STS_IO_MP_FAULT = 0,  /**< PG drop/recover, INA alert, EN fault, BKP PG */
	STS_IO_MP_BUTTON = 1, /**< panel button / encoder switch */
	STS_IO_MP_PROX = 2,   /**< reed switch (door/magnet) */
	STS_IO_MP_TOUCH = 3,  /**< touch controller INT */
} sts_io_mp_kind_t;

/**
 * Which log line an event produces.
 *
 * The wording is part of the decision, not decoration. "load switch X faulted"
 * at ERR and "load switch X recovered" at NOTICE are the same event type with
 * opposite edges, and swapping either half — the severity or the verb — yields
 * a log that reads exactly like the opposite condition. Pairing them here means
 * one test pins both.
 */
typedef enum {
	STS_IO_MSG_NONE = 0,
	STS_IO_MSG_INA_ALERT,     /**< "INA228 ALERT: %s" */
	STS_IO_MSG_PG_LOST,       /**< "power-good lost: %s" */
	STS_IO_MSG_PG_RESTORED,   /**< "power-good restored: %s" */
	STS_IO_MSG_EN_FAULTED,    /**< "load switch %s faulted" */
	STS_IO_MSG_EN_RECOVERED,  /**< "load switch %s recovered" */
	STS_IO_MSG_BKP_NOT_GOOD,  /**< "backup supply %s not good" */
	STS_IO_MSG_BKP_GOOD,      /**< "backup supply %s good" */
	STS_IO_MSG__COUNT,
} sts_io_msg_t;

/** Everything one debounced event asks the scan thread to do. */
typedef struct {
	/** Post a UI input event. */
	bool post_ui;
	/** sts_io_ui_type_t; only with @p post_ui. */
	int8_t ui_type;
	/** sts_input_evt_t::value; only with @p post_ui. */
	int16_t ui_value;

	/**
	 * Ask housekeeping to re-read this monitor's DIAG_ALRT.
	 *
	 * The ALERT pin says "something tripped"; only DIAG_ALRT says what.
	 * Reading it in the scan slot would put an I2C transaction inside the
	 * 1 ms budget, so the request is deferred.
	 */
	bool ina_reread;
	/** ina228_rail_t; only with @p ina_reread. */
	uint8_t ina_rail;

	/** Emit a log line. */
	bool log;
	/** logr_sub_t; only with @p log. */
	uint8_t log_sub;
	/** logr_level_t; only with @p log. */
	uint8_t log_level;
	/** sts_io_msg_t; only with @p log. */
	uint8_t msg;

	/**
	 * Stage a record on the MP event channel (FMT §7.5).
	 *
	 * Separate from @p post_ui and @p log because the three consumers want
	 * different things and have historically been conflated:
	 *
	 *   - the UI wants only what it can act on, which is why a touch INT
	 *     *release* is not posted to it (the FIFO has just been drained, so
	 *     re-entering the touch path would read nothing);
	 *   - the log wants the failures, which is why an INA228 ALERT is logged
	 *     on assert only;
	 *   - the event channel is an OBSERVATION stream for a technician with a
	 *     Field Maintenance Tool attached, and for it the missing half of a
	 *     pair is diagnosis: an ALERT that never clears is a rail still out
	 *     of limits, and a power-good that never recovers is a rail that is
	 *     gone rather than glitching. So it carries every edge core/fault
	 *     hands over, both directions, for every event type.
	 *
	 * ONE ASYMMETRY IS NOT THIS DISPATCHER'S TO FIX, and it is stated here
	 * so nobody documents an event the wire cannot carry: core/fault never
	 * *emits* a touch release. fault.c's FAULT_CLASS_TOUCH dispatch pushes
	 * an event only on the assert ("its release says nothing"), so
	 * MP_EV_TOUCH is assert-only on the wire — not because the mapping below
	 * suppresses it, but because there is nothing to map. The DEASSERT arm
	 * of the mapping is correct and would carry a release the day core/fault
	 * produces one; until then a stuck-low INT is diagnosed from the absence
	 * of further assertions rather than from a missing release.
	 */
	bool post_mp;
	/** sts_io_mp_kind_t; only with @p post_mp. */
	uint8_t mp_kind;
	/** mp_ev_t::sub — the fault_evt_type_t that produced it. */
	uint8_t mp_sub;
	/** mp_ev_t::edge; 1 = assert/press, 0 = deassert/release. */
	uint8_t mp_edge;
} sts_io_dispatch_t;

/**
 * Which MP event kind a debounced fault event belongs to.
 *
 * A total map over fault_evt_type_t, written as one switch with no default so
 * that adding an event type to core/fault is a compiler diagnostic here rather
 * than a signal that silently stops reaching the technician's tool. That is the
 * exact failure this whole seam exists to fix.
 *
 * @retval true   @p out holds an sts_io_mp_kind_t.
 * @retval false  @p type is not a scanned-signal event; nothing is staged.
 */
static inline bool sts_io_mp_kind_for(uint8_t type, uint8_t *out)
{
	switch ((fault_evt_type_t)type) {
	case FAULT_EVT_BUTTON:
	case FAULT_EVT_BUTTON_LONG:
	case FAULT_EVT_BUTTON_REPEAT:
		*out = (uint8_t)STS_IO_MP_BUTTON;
		return true;
	case FAULT_EVT_TOUCH:
		*out = (uint8_t)STS_IO_MP_TOUCH;
		return true;
	case FAULT_EVT_PROX:
		*out = (uint8_t)STS_IO_MP_PROX;
		return true;
	case FAULT_EVT_EN_FAULT:
	case FAULT_EVT_PG_FAULT:
	case FAULT_EVT_PG_RECOVER:
	case FAULT_EVT_INA_ALERT:
	case FAULT_EVT_BKP_PG:
		*out = (uint8_t)STS_IO_MP_FAULT;
		return true;
	case FAULT_EVT_TYPE_COUNT:
		break;
	}
	return false;
}

/**
 * Decide what a debounced fault event turns into.
 *
 * @p out is always fully written, so an event type this dispatcher does not
 * handle produces a plan that does nothing rather than a stale one.
 *
 * @param type  fault_evt_type_t.
 * @param id    fault_sig_t the event is about.
 * @param edge  fault_edge_t.
 * @param out   Never NULL.
 */
static inline void sts_io_dispatch_plan(uint8_t type, uint8_t id, uint8_t edge,
					sts_io_dispatch_t *out)
{
	bool assert_edge = (edge == (uint8_t)FAULT_EDGE_ASSERT);

	memset(out, 0, sizeof(*out));
	out->ui_type = (int8_t)STS_IO_UI_NONE;

	switch ((fault_evt_type_t)type) {
	case FAULT_EVT_BUTTON:
		out->post_ui = true;
		out->ui_type = (int8_t)STS_IO_UI_BUTTON;
		out->ui_value = assert_edge ? 1 : 0;
		break;
	case FAULT_EVT_BUTTON_LONG:
		out->post_ui = true;
		out->ui_type = (int8_t)STS_IO_UI_BUTTON_LONG;
		out->ui_value = 1;
		break;
	case FAULT_EVT_BUTTON_REPEAT:
		out->post_ui = true;
		out->ui_type = (int8_t)STS_IO_UI_BUTTON_REPEAT;
		out->ui_value = 1;
		break;
	case FAULT_EVT_TOUCH:
		/* Only the assertion is actionable: the FT6336 INT going away
		 * means the controller's FIFO was drained, which is what the UI
		 * just did. Posting the release would re-enter the touch path
		 * with nothing to read. */
		if (assert_edge) {
			out->post_ui = true;
			out->ui_type = (int8_t)STS_IO_UI_TOUCH;
			out->ui_value = 1;
		}
		break;
	case FAULT_EVT_PROX:
		out->post_ui = true;
		out->ui_type = (int8_t)STS_IO_UI_PROX;
		out->ui_value = assert_edge ? 1 : 0;
		break;

	case FAULT_EVT_INA_ALERT:
		if (assert_edge) {
			ina228_rail_t rail;

			if (sts_io_ina_rail_for_sig((fault_sig_t)id, &rail) == 0) {
				out->ina_reread = true;
				out->ina_rail = (uint8_t)rail;
			}
			out->log = true;
			out->log_sub = (uint8_t)LOGR_SUB_PWR;
			out->log_level = (uint8_t)LOGR_WARN;
			out->msg = (uint8_t)STS_IO_MSG_INA_ALERT;
		}
		break;

	case FAULT_EVT_PG_FAULT:
		out->log = true;
		out->log_sub = (uint8_t)LOGR_SUB_PWR;
		out->log_level = (uint8_t)LOGR_ERR;
		out->msg = (uint8_t)STS_IO_MSG_PG_LOST;
		break;
	case FAULT_EVT_PG_RECOVER:
		out->log = true;
		out->log_sub = (uint8_t)LOGR_SUB_PWR;
		out->log_level = (uint8_t)LOGR_NOTICE;
		out->msg = (uint8_t)STS_IO_MSG_PG_RESTORED;
		break;
	case FAULT_EVT_EN_FAULT:
		out->log = true;
		out->log_sub = (uint8_t)LOGR_SUB_PWR;
		out->log_level = assert_edge ? (uint8_t)LOGR_ERR
					     : (uint8_t)LOGR_NOTICE;
		out->msg = assert_edge ? (uint8_t)STS_IO_MSG_EN_FAULTED
				       : (uint8_t)STS_IO_MSG_EN_RECOVERED;
		break;
	case FAULT_EVT_BKP_PG:
		/*
		 * The supercap rails back the RTC and the GNSS ephemeris, not
		 * the served time, so neither edge is an error — but the polarity
		 * still has to be right, because "not good" is the one that
		 * predicts a cold GNSS start after the next power interruption.
		 */
		out->log = true;
		out->log_sub = (uint8_t)LOGR_SUB_PWR;
		out->log_level = (uint8_t)LOGR_NOTICE;
		out->msg = assert_edge ? (uint8_t)STS_IO_MSG_BKP_NOT_GOOD
				       : (uint8_t)STS_IO_MSG_BKP_GOOD;
		break;

	default:
		break;
	}

	/*
	 * The MP event-channel decision, deliberately outside the switch above.
	 *
	 * Everything the scan debounces goes to the technician's tool, both
	 * edges, with the fault_evt_type_t carried in `sub` so the host can tell
	 * a press from a long-press from an auto-repeat and a power-good loss
	 * from its recovery. The switch above is where the three *local*
	 * consumers disagree with each other; there is no such disagreement
	 * here, and expressing "all of them" as nine more identical case labels
	 * would be nine more places for one to be forgotten — which is precisely
	 * how four of this channel's eight event kinds came to have no producer.
	 */
	{
		uint8_t kind = 0U;

		if (sts_io_mp_kind_for(type, &kind)) {
			out->post_mp = true;
			out->mp_kind = kind;
			out->mp_sub = type;
			out->mp_edge = assert_edge ? 1U : 0U;
		}
	}
}

/** The format string @p msg selects. Never NULL; "" for STS_IO_MSG_NONE. */
static inline const char *sts_io_msg_fmt(uint8_t msg)
{
	switch ((sts_io_msg_t)msg) {
	case STS_IO_MSG_INA_ALERT:
		return "INA228 ALERT: %s";
	case STS_IO_MSG_PG_LOST:
		return "power-good lost: %s";
	case STS_IO_MSG_PG_RESTORED:
		return "power-good restored: %s";
	case STS_IO_MSG_EN_FAULTED:
		return "load switch %s faulted";
	case STS_IO_MSG_EN_RECOVERED:
		return "load switch %s recovered";
	case STS_IO_MSG_BKP_NOT_GOOD:
		return "backup supply %s not good";
	case STS_IO_MSG_BKP_GOOD:
		return "backup supply %s good";
	case STS_IO_MSG_NONE:
	default:
		return "";
	}
}

/* ---------------------------------------------------------- dropped events - */

/** What to do about the queue's dropped-event counter this scan. */
typedef struct {
	bool clear; /**< call fault_evt_clear_dropped() */
	bool log;   /**< annunciate the loss */
} sts_io_drop_action_t;

/**
 * Decide whether a dropped-event burst is worth reporting.
 *
 * The counter is cumulative and is cleared by the consumer once reported
 * (fault.h), so "is it non-zero" is the entire condition and the report must
 * take the counter back to zero in the same breath. Any additional latch on the
 * last-reported value double-reports: the scan after a real report reads the
 * freshly-cleared zero, sees it differ from the latch, and emits a second
 * warning claiming zero events were dropped.
 */
static inline void sts_io_drop_action(uint32_t dropped, sts_io_drop_action_t *out)
{
	out->clear = (dropped != 0U);
	out->log = (dropped != 0U);
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_PLATFORM_STS_IO_POLICY_H_ */
