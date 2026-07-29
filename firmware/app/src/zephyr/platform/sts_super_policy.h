/*
 * STS1000 "Meridian" — the supervisor's three fail-safe decisions, as pure logic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/platform/, and deliberately free of every Zephyr
 * dependency so tests/host can compile it — the same arrangement as
 * net/sts_ppscorr.h, for the same reason.
 *
 * ---------------------------------------------------------------------------
 * Why these three, and what is NOT here
 * ---------------------------------------------------------------------------
 * supervisor.c drives three outputs whose wrong state is invisible in normal
 * operation and severe when it happens:
 *
 *   WDT_KICK (PB2)  The TPS3430 window is 920-1360 ms and a kick that is too
 *                   EARLY is as fatal as one that is too late — either drives
 *                   WDO_N -> POE_KILL and the board's PoE port drops. The
 *                   cadence arithmetic itself belongs to core/pwrseq
 *                   (pwrseq_wdt_service, covered by the `pwrseq` suite); what
 *                   lives here is the *permission*: an AND-gate over the
 *                   platform's liveness registry. Get that inverted and a
 *                   wedged box keeps being fed; get it over-strict and a
 *                   healthy grandmaster cold-cycles. Neither shows up on a
 *                   bench that never wedges.
 *
 *   K2 relay (PA6)  Normally de-energized = ALARM. Three independent gates have
 *                   to hold before it is driven, and dropping any one of them
 *                   makes the relay lie in the direction that matters:
 *                   annunciating "service OK" for a box that is warming up, or
 *                   for one whose sequencer has not reached stage 9.
 *
 *   RGB D5          The priority encode is core/fault's (fault_rgb_state); the
 *                   *inputs* are assembled here, and the identify expiry is
 *                   32-bit-wrap arithmetic that is wrong for 49.7 days at a
 *                   time if the comparison is unsigned. The maintenance
 *                   override resolves here too — which pattern wins, what each
 *                   pattern's three duties are, and the order the leg holds
 *                   compose in — because a decision taken in supervisor.c is a
 *                   decision no host suite links, and an indicator that shows
 *                   the wrong colour is wrong in exactly the silent way this
 *                   file exists to stop.
 *
 * Plus the §8.3 self-confirm trigger, which is the moment MCUboot's automatic
 * revert stops protecting a remote box. Confirming one condition too early
 * throws that protection away silently.
 *
 * The pins, the PWM, the log calls and the k_thread stay in supervisor.c.
 */

#ifndef STS1000_ZEPHYR_PLATFORM_STS_SUPER_POLICY_H_
#define STS1000_ZEPHYR_PLATFORM_STS_SUPER_POLICY_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "fault/fault.h"
#include "pwrseq/pwrseq.h"
#include "quality/quality.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------- liveness gate -- */

/**
 * Map the platform's stale-participant mask onto pwrseq's liveness word.
 *
 * An AND-gate, and deliberately an all-or-nothing one: the platform registry is
 * the authority on who must be alive — every area registers its own
 * participants by name, so the count is not fixed at pwrseq's three bits — and
 * a single late participant must withhold the kick.
 *
 * Silence is the safe answer. Withholding lets the TPS3430 time out, drive
 * WDO_N, drive POE_KILL and cold-cycle a board that is genuinely wedged. The
 * failure mode this function exists to prevent is the opposite one: returning
 * PWRSEQ_LIVE_ALL for a partially-dead board, which feeds the watchdog forever
 * and turns the whole external-watchdog design into decoration.
 */
static inline uint32_t sts_super_wdt_liveness(uint32_t stale_mask)
{
	return (stale_mask == 0U) ? PWRSEQ_LIVE_ALL : 0U;
}

/**
 * May a MAINTENANCE WDI edge be driven right now?
 *
 * This is not the cadence question — pwrseq_wdt_service() owns that for the
 * supervisor's own kicks. This is the console's `sys.wdt.kick`, an edge whose
 * PHASE relative to the supervisor's last kick is arbitrary by construction,
 * because nothing synchronises a technician's keystroke to a 920-1360 ms
 * window. Land it inside tWDL(min) = 680 ms of the last kick and the TPS3430
 * calls it a RUNAWAY fault: WDO_N asserts for ~200 ms, drives POE_KILL, and the
 * board drops its own PoE port (docs/sts1000_external_wdt.md §4).
 *
 * So there is exactly one state in which the edge is harmless — WDT_EN
 * de-asserted, no window being watched, no cadence to collide with — and the
 * answer for every other state, INCLUDING "the supervisor never initialised and
 * cannot say", is no. `ready` is therefore a term rather than an assumption:
 * an uninitialised supervisor has not configured PB2 either, and refusing is
 * the only reading of an unknown pin that cannot cold-cycle the board.
 *
 * The console-side interlock MP_ILK_WDT_OFF says the same thing one seam out,
 * so a caller reaching sts_supervisor_wdt_kick() directly is refused too.
 */
static inline bool sts_super_wdi_pulse_allowed(bool ready, bool armed)
{
	return ready && !armed;
}

/**
 * What does sts_supervisor_wdt_state() publish through its out-param?
 *
 * The mirror of the predicate above, and deliberately the same two terms in the
 * same order: this decides what a caller is TOLD about WDT_EN, that one decides
 * what may be DRIVEN on WDI. Both must fail the same way, because a state
 * report that says "not watching" is the premise the pulse permission is
 * granted on one seam out.
 *
 * So an uninitialised supervisor reports ARMED. There is no pin to read — PC12
 * has not been configured — and of the two things that can be said about an
 * unknown watchdog, "watching" is the one that costs a refused maintenance
 * pulse and "not watching" is the one that costs the board its PoE port. The
 * report is paired with -ENODEV, but the errno is the caller's to ignore and
 * the out-param is the headline output; this term is what makes ignoring it
 * safe rather than merely unlikely.
 *
 * That is belt-and-braces TODAY, and stating so is the point of this comment.
 * All three in-tree callers (console/mp_glue.c) check the return code first and
 * discard the out-param on failure, so the value below reaches no live
 * decision: prov_ilk() computes `(rc == 0) && !armed`, whose short-circuit
 * decides MP_ILK_WDT_OFF before this value is consulted at all. Neither read
 * that as dead code nor as the guard that is holding the hazard shut — it is
 * the contract a fourth caller gets for free, written once so the safe
 * direction is a tested fact rather than an inline literal in Zephyr glue no
 * host suite links.
 *
 * `ready` is a term rather than an assumption for the same reason it is one
 * above, and the ternary keeps both parameters live so dropping either is an
 * -Werror=unused-parameter build failure rather than a silent fail-open.
 */
static inline bool sts_super_wdt_state_armed(bool ready, bool armed)
{
	return ready ? armed : true;
}

/**
 * Should this tick log the liveness loss?
 *
 * Only while armed (before that a stale participant is just a thread that has
 * not started), only when something is actually stale, and only on a change —
 * the supervisor runs at 4 Hz and a permanently-late participant would
 * otherwise emit four CRIT lines a second until the ring wrapped and took the
 * cause with it.
 */
static inline bool sts_super_stale_log(bool armed, uint32_t stale_mask,
				       uint32_t last_stale_mask)
{
	return armed && (stale_mask != 0U) && (stale_mask != last_stale_mask);
}

/**
 * Has the kicker recorded a new window violation since @p logged?
 *
 * A validated cadence makes this impossible from pwrseq_wdt_service() alone, so
 * a non-zero count means a second writer is on the pin and the board is being
 * cold-cycled by its own supervisor. Unsigned inequality rather than `>` so a
 * counter wrap still reports.
 */
static inline bool sts_super_violation_new(uint32_t violations, uint32_t logged)
{
	return violations != logged;
}

/** "below" or "above" for the violation log line. Never NULL. */
static inline const char *sts_super_violation_word(uint8_t verdict)
{
	return (verdict == (uint8_t)PWRSEQ_WDT_INTERVAL_EARLY) ? "below" : "above";
}

/* ---------------------------------------------------------------- relay --- */

/**
 * Is the clock actually serving?
 *
 * Both halves are required. Stratum 1 without a lock is a box that is
 * advertising traceability it does not have; a lock without stratum 1 is a
 * disciplined oscillator that is not serving anybody.
 */
static inline bool sts_super_serving(const quality_block_t *q)
{
	return (q != NULL) && (q->stratum == QUALITY_STRATUM_PRIMARY) &&
	       (q->lock_state == (uint8_t)QUALITY_LOCK_LOCKED);
}

/**
 * Drive K2?
 *
 * Three gates, all required:
 *   @p seq_eligible   pwrseq has reached stage 9. This is what keeps K2
 *                     de-energized through bring-up even if the OCXO happens to
 *                     lock early.
 *   @p alarm_eligible fault_relay_eligible(): no disqualifying alarm is active.
 *   serving           the quality block says the clock is serving.
 *
 * De-energized is ALARM and is the safe state, so every gate votes only for
 * energizing. A NULL quality block is treated as not serving.
 */
static inline bool sts_super_relay_want(bool seq_eligible, bool alarm_eligible,
					const quality_block_t *q)
{
	return seq_eligible && alarm_eligible && sts_super_serving(q);
}

/* ------------------------------------------------------------------ RGB --- */

/**
 * Is an operator-commanded identify still running?
 *
 * @p until_ms of 0 means "no identify pending" — the sentinel, which is why a
 * bare interval test is not enough.
 *
 * The comparison is a SIGNED difference of two uint32 monotonic timestamps.
 * k_uptime_get_32() wraps every 49.7 days, and an identify started just before
 * the wrap has a deadline that is numerically *smaller* than `now`; an unsigned
 * `until > now` test would end it instantly, and an unsigned `now - until`
 * test would keep it running for the next 24 days. The signed difference is
 * correct for any interval shorter than 24.8 days, which every identify is.
 */
static inline bool sts_super_identify_active(uint32_t until_ms, uint32_t now_ms)
{
	return (until_ms != 0U) && ((int32_t)(until_ms - now_ms) > 0);
}

/**
 * Assemble the status-RGB inputs from the published quality block.
 *
 * core/fault owns the priority encode (fault_rgb_state); this owns what each
 * input *means*:
 *
 *   locked    serving, by the same two-part test the relay uses.
 *   warming   the OCXO oven is not up to temperature, OR the loop has not
 *             finished converging. Either is amber, and the OR matters: a warm
 *             oven that is still acquiring is not green, and a converged loop on
 *             a cold oven is not green either.
 *   holdover  straight from the block.
 *
 * @param q         Published quality. NULL is treated as "nothing known", which
 *                  is not warm and not locked.
 * @param any_fault fault_any_active().
 * @param identify  sts_super_identify_active().
 * @param out       Never NULL; always fully written.
 */
static inline void sts_super_rgb_in(const quality_block_t *q, bool any_fault,
				    bool identify, fault_rgb_in_t *out)
{
	memset(out, 0, sizeof(*out));

	out->any_fault = any_fault;
	out->identify = identify;

	if (q == NULL) {
		/* No published block: the oven cannot be assumed warm. */
		out->warming = true;
		return;
	}

	out->locked = sts_super_serving(q);
	out->holdover = q->holdover;
	out->warming = ((q->flags & QUALITY_FLAG_OCXO_WARM) == 0U) ||
		       (q->lock_state == (uint8_t)QUALITY_LOCK_ACQUIRING) ||
		       (q->lock_state == (uint8_t)QUALITY_LOCK_LOCKING);
}

/* ------------------------------------------------- RGB override + colour --- */

/*
 * The `ui.rgb.mode` values, restated here rather than included from sts_app.h.
 * This header's dependency set is a leaf one (fault, pwrseq, quality) and the
 * `super_policy` suite's link set follows it; pulling in the app's public
 * surface would drag cfg, logring and mp_mirror behind it for six integers.
 * supervisor.c BUILD_ASSERTs the two sets identical, so drift is a build
 * failure rather than an amber that shows up red on a technician's screen.
 */
#define STS_SUPER_RGB_MODE_AUTO       0U /* no pattern held — use the encode */
#define STS_SUPER_RGB_MODE_OFF        1U
#define STS_SUPER_RGB_MODE_GREEN      2U
#define STS_SUPER_RGB_MODE_AMBER      3U
#define STS_SUPER_RGB_MODE_RED        4U
#define STS_SUPER_RGB_MODE_BLUE_PULSE 5U
#define STS_SUPER_RGB_MODE_COUNT      6U

/** Legs, in the order TIM4 CH1/CH2/CH3 are programmed. */
#define STS_SUPER_RGB_LEG_R     0U
#define STS_SUPER_RGB_LEG_G     1U
#define STS_SUPER_RGB_LEG_B     2U
#define STS_SUPER_RGB_LEG_COUNT 3U

/*
 * The pattern duties. Deliberately not 100 %: docs/sts1000_rgb_indicator.md
 * sizes each colour's ballast resistor for a matched apparent brightness at
 * full duty, and the indicator is read across a rack aisle rather than stared
 * at. Amber is a RATIO of two legs — 70/35 — so scaling one without the other
 * turns it orange-red, which is the colour red is supposed to own.
 */
#define STS_SUPER_RGB_DUTY_GREEN   60U
#define STS_SUPER_RGB_DUTY_AMBER_R 70U
#define STS_SUPER_RGB_DUTY_AMBER_G 35U
#define STS_SUPER_RGB_DUTY_RED     70U
#define STS_SUPER_RGB_DUTY_BLUE    80U

/** Identify pulses at 1 Hz, so a half period is 500 ms. */
#define STS_SUPER_RGB_PULSE_HALF_MS 500U

/**
 * The D5 maintenance override, in two independently-leased layers.
 *
 * `mode` forces a whole PATTERN (STS_SUPER_RGB_MODE_AUTO = none held); the leg
 * entries hold individual duties AFTER the pattern has resolved to a colour, so
 * one leg can be held without taking the indicator off the fault policy.
 */
typedef struct {
	uint8_t mode;
	bool leg_held[STS_SUPER_RGB_LEG_COUNT];
	uint8_t leg_pct[STS_SUPER_RGB_LEG_COUNT];
} sts_super_rgb_ovr_t;

/** What D5 is to be driven to. */
typedef struct {
	/** The pattern actually resolved. NEVER STS_SUPER_RGB_MODE_AUTO — "auto"
	 *  names where the answer came from, not something D5 can show. */
	uint8_t mode;
	uint8_t duty[STS_SUPER_RGB_LEG_COUNT];
} sts_super_rgb_out_t;

/**
 * core/fault's priority-encode result as a `ui.rgb.mode` pattern.
 *
 * An explicit map and not `(uint8_t)state + 1`: the two enums happen to be one
 * apart today, and an arithmetic bridge turns any future reordering of
 * fault_rgb_state_t into amber shown as red, silently, on a pin nobody is
 * asserting against.
 */
static inline uint8_t sts_super_rgb_mode_of(fault_rgb_state_t state)
{
	switch (state) {
	case FAULT_RGB_GREEN:
		return STS_SUPER_RGB_MODE_GREEN;
	case FAULT_RGB_AMBER:
		return STS_SUPER_RGB_MODE_AMBER;
	case FAULT_RGB_RED:
		return STS_SUPER_RGB_MODE_RED;
	case FAULT_RGB_BLUE_PULSE:
		return STS_SUPER_RGB_MODE_BLUE_PULSE;
	case FAULT_RGB_OFF:
	default:
		return STS_SUPER_RGB_MODE_OFF;
	}
}

/**
 * The pattern D5 is to show: the forced one, or the fault policy's own.
 *
 * @param mode        The held override, STS_SUPER_RGB_MODE_AUTO for none.
 * @param auto_state  core/fault's encode, used only when nothing is held.
 */
static inline uint8_t sts_super_rgb_mode_now(uint8_t mode, fault_rgb_state_t auto_state)
{
	if (mode != STS_SUPER_RGB_MODE_AUTO && mode < STS_SUPER_RGB_MODE_COUNT) {
		return mode;
	}

	return sts_super_rgb_mode_of(auto_state);
}

/** One pattern's three duties, before any leg override. */
static inline void sts_super_rgb_colour(uint8_t mode, uint32_t now_ms,
					uint8_t out[STS_SUPER_RGB_LEG_COUNT])
{
	out[STS_SUPER_RGB_LEG_R] = 0U;
	out[STS_SUPER_RGB_LEG_G] = 0U;
	out[STS_SUPER_RGB_LEG_B] = 0U;

	switch (mode) {
	case STS_SUPER_RGB_MODE_GREEN:
		out[STS_SUPER_RGB_LEG_G] = STS_SUPER_RGB_DUTY_GREEN;
		break;
	case STS_SUPER_RGB_MODE_AMBER:
		out[STS_SUPER_RGB_LEG_R] = STS_SUPER_RGB_DUTY_AMBER_R;
		out[STS_SUPER_RGB_LEG_G] = STS_SUPER_RGB_DUTY_AMBER_G;
		break;
	case STS_SUPER_RGB_MODE_RED:
		out[STS_SUPER_RGB_LEG_R] = STS_SUPER_RGB_DUTY_RED;
		break;
	case STS_SUPER_RGB_MODE_BLUE_PULSE:
		/* Phase taken from the monotonic clock, so multiple units in a
		 * rack do not have to agree on anything. */
		out[STS_SUPER_RGB_LEG_B] =
			((now_ms / STS_SUPER_RGB_PULSE_HALF_MS) % 2U == 0U)
				? STS_SUPER_RGB_DUTY_BLUE
				: 0U;
		break;
	case STS_SUPER_RGB_MODE_OFF:
	default:
		break;
	}
}

/**
 * Hold one leg's duty, or release it.
 *
 * Clamps rather than rejects, on the same reasoning as sts_fan_pulse_ns(): a
 * request the caller has already accepted must land somewhere defined. A
 * release zeroes the stored level too, so a lapsed lease leaves nothing behind
 * for the next resolve to pick up.
 *
 * @return false if @p leg is not a leg — the state is then untouched.
 */
static inline bool sts_super_rgb_leg_set(sts_super_rgb_ovr_t *ovr, uint8_t leg,
					 bool active, uint8_t pct)
{
	if (leg >= STS_SUPER_RGB_LEG_COUNT) {
		return false;
	}

	ovr->leg_held[leg] = active;
	ovr->leg_pct[leg] = active ? ((pct > 100U) ? 100U : pct) : 0U;

	return true;
}

/**
 * Resolve pattern, then legs.
 *
 * The two layers compose in this order and not the other: the legs override
 * individual duties AFTER the pattern has become a colour. That is what lets a
 * technician hold one leg — to prove a dead colour channel, say — without
 * taking the whole indicator off the fault policy, and it is why a leg hold at
 * 0 % is a real command (dark) and not "no override".
 *
 * @param ovr         The held override. NULL is the same as nothing held.
 * @param auto_state  core/fault's encode.
 * @param now_ms      Monotonic milliseconds, for the identify pulse phase.
 * @param out         Never NULL; always fully written.
 */
static inline void sts_super_rgb_resolve(const sts_super_rgb_ovr_t *ovr,
					 fault_rgb_state_t auto_state, uint32_t now_ms,
					 sts_super_rgb_out_t *out)
{
	memset(out, 0, sizeof(*out));

	out->mode = sts_super_rgb_mode_now(
		(ovr != NULL) ? ovr->mode : (uint8_t)STS_SUPER_RGB_MODE_AUTO, auto_state);

	sts_super_rgb_colour(out->mode, now_ms, out->duty);

	if (ovr == NULL) {
		return;
	}

	for (uint8_t i = 0U; i < STS_SUPER_RGB_LEG_COUNT; i++) {
		if (ovr->leg_held[i]) {
			out->duty[i] = ovr->leg_pct[i];
		}
	}
}

/* --------------------------------------------------------- self-confirm --- */

/**
 * Confirm the running image now?
 *
 * ARCHITECTURE.md §3: an unconfirmed image is reverted by MCUboot on the next
 * boot. That automatic revert is the only unattended protection a remote
 * grandmaster has against a bad update, and it is worth exactly as much as this
 * gate is strict — so every term has to hold at once:
 *
 *   @p pending      there is something to confirm.
 *   @p stale_mask   every liveness participant is feeding. An image whose GNSS
 *                   or discipline thread died still boots, enumerates USB and
 *                   answers the shell.
 *   @p any_fault    no alarm is active.
 *   @p lock_state   the clock is LOCKED, not merely converging. This is the term
 *                   that makes the gate mean what §8.2 says: "network up, clock
 *                   disciplining, services answering".
 *
 * Doing nothing IS the revert, so a false negative costs one boot and a false
 * positive costs the protection entirely.
 */
static inline bool sts_super_confirm_now(bool pending, uint32_t stale_mask,
					 bool any_fault, uint8_t lock_state)
{
	return pending && (stale_mask == 0U) && !any_fault &&
	       (lock_state == (uint8_t)QUALITY_LOCK_LOCKED);
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_PLATFORM_STS_SUPER_POLICY_H_ */
