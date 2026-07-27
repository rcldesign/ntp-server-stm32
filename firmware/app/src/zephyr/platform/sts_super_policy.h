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
 *                   time if the comparison is unsigned.
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
