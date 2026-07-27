/*
 * STS1000 "Meridian" — the spec §8.3 self-confirm decision, as pure arithmetic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/console/, with one exception: it is deliberately free of
 * every Zephyr and hardware dependency so tests/host can compile it and pin the
 * decision down. The policy in here is the only thing standing between a broken
 * image and a remote grandmaster that has to be recovered by hand, so it is
 * tested rather than reasoned about.
 *
 * What the gate is for
 * --------------------
 * MCUboot boots a freshly-staged image in TEST mode: unless the application
 * calls boot_write_img_confirmed() before the next reset, the previous image is
 * swapped back. That automatic revert is the ONLY unattended protection against
 * shipping a bad update, and it is worth exactly as much as the gate is strict.
 * Spec §8.2 names the bar: "the app must self-confirm after passing post-update
 * health (network up, clock disciplining, services answering)".
 *
 * "Clock disciplining" is not optional decoration here. An image whose GNSS
 * UART, PPS capture, DAC path or discipline loop is broken still boots, still
 * enumerates USB and still mounts NVS — so a gate made only of uptime, cfg and
 * link liveness confirms it, and the revert never fires. The timing terms below
 * are what make the gate mean what §8.3 says.
 *
 * The deadline
 * ------------
 * Abandoning the attempt is safe (doing nothing IS the revert) but it is also
 * final for this boot, so the window has to be longer than the slowest legitimate
 * path to a first lock: GNSS time-to-first-fix, OCXO warm-up, loop convergence,
 * and then tim.lock.hold seconds of continuously meeting the lock criteria before
 * QUALITY_LOCK_LOCKED can even be declared. A hardcoded 600 s is *below*
 * tim.lock.hold + anything, so it would abandon a perfectly healthy slow start.
 * sts_confirm_deadline_s() therefore derives the window from the configured hold
 * time; the Kconfig value is only a floor.
 */

#ifndef STS1000_ZEPHYR_CONSOLE_STS_CONFIRM_GATE_H_
#define STS1000_ZEPHYR_CONSOLE_STS_CONFIRM_GATE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * One evaluation of the §8.3 health gate.
 *
 * Every field is an observation except @p min_age_s and @p deadline_s, which are
 * the policy the observations are judged against.
 */
typedef struct {
	/** boot_is_img_confirmed(): nothing to do when already true. */
	bool image_confirmed;
	/** The platform area published a cfg context. */
	bool cfg_loaded;
	/** NVS mounted, so configuration genuinely persists. */
	bool store_ready;
	/** USB reached CONFIGURED, or a network interface is up. */
	bool link_ok;
	/** quality lock_state == QUALITY_LOCK_LOCKED. */
	bool clock_locked;
	/** quality stratum == QUALITY_STRATUM_PRIMARY, i.e. serving traceably. */
	bool serving_primary;

	uint32_t uptime_s;
	uint32_t min_age_s;
	uint32_t deadline_s;
} sts_confirm_gate_t;

/** Uptime proxy for "every thread has registered and fed its liveness slot". */
static inline bool sts_confirm_min_age_met(const sts_confirm_gate_t *g)
{
	return g->uptime_s >= g->min_age_s;
}

/**
 * The timing half of the gate: the clock is disciplined AND we are serving
 * stratum 1.
 *
 * Both terms, even though on the current quality state machine primary implies
 * locked. They are separate spec statements ("clock disciplining" and "services
 * answering"), and asserting both means a later change that decouples them
 * cannot silently weaken this gate.
 */
static inline bool sts_confirm_timing_ok(const sts_confirm_gate_t *g)
{
	return g->clock_locked && g->serving_primary;
}

/**
 * The whole gate. True means the running image has demonstrated §8.3 health and
 * may be confirmed.
 *
 * Note what is NOT here: the active-alarm mask. The fault module belongs to the
 * platform area and the supervisor — the authoritative caller — already refuses
 * to confirm while any fault is active. Duplicating that here with the coarser
 * console-side view would mean a board with one permanently-active benign alarm
 * could never accept an update at all, because every candidate image would fail
 * the same way.
 */
static inline bool sts_confirm_gate_pass(const sts_confirm_gate_t *g)
{
	return sts_confirm_min_age_met(g) && g->cfg_loaded && g->store_ready &&
	       g->link_ok && sts_confirm_timing_ok(g);
}

/** True once the window has closed and the attempt must be abandoned. */
static inline bool sts_confirm_deadline_passed(const sts_confirm_gate_t *g)
{
	return g->uptime_s >= g->deadline_s;
}

/**
 * Derive the abandon deadline.
 *
 * @param min_age_s    The uptime floor the gate already imposes.
 * @param lock_hold_s  cfg key tim.lock.hold: seconds the lock criteria must hold
 *                     continuously before QUALITY_LOCK_LOCKED is declared. The
 *                     deadline has to be strictly beyond it or a legitimate slow
 *                     first fix is abandoned before it can possibly succeed.
 * @param margin_s     Allowance for everything ahead of that hold window: GNSS
 *                     time-to-first-fix, OCXO warm-up, loop convergence, PHY
 *                     autonegotiation and a DHCP lease.
 * @param floor_s      Configured minimum (CONFIG_STS1000_SELF_CONFIRM_DEADLINE_S).
 *
 * @return Seconds of uptime after which the attempt is abandoned. Saturates
 *         rather than wrapping, so no combination of configured values can
 *         produce a deadline in the past.
 */
static inline uint32_t sts_confirm_deadline_s(uint32_t min_age_s,
					      uint32_t lock_hold_s,
					      uint32_t margin_s,
					      uint32_t floor_s)
{
	uint64_t derived = (uint64_t)min_age_s + (uint64_t)lock_hold_s +
			   (uint64_t)margin_s;

	if (derived > (uint64_t)UINT32_MAX) {
		derived = (uint64_t)UINT32_MAX;
	}
	if (derived < (uint64_t)floor_s) {
		derived = (uint64_t)floor_s;
	}

	return (uint32_t)derived;
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_CONSOLE_STS_CONFIRM_GATE_H_ */
