/*
 * STS1000 "Meridian" — the AAA offload gate, as pure logic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/net/, and deliberately free of every Zephyr dependency
 * so tests/host can compile it — the same arrangement as net/sts_ppscorr.h and
 * platform/sts_fan_policy.h, for the same reason: what it decides is invisible
 * at runtime when it is wrong.
 *
 * sts_secops.c owns the primitives (a semaphore for the gate, a mutex for the
 * hand-off critical section, two semaphores for the worker rendezvous). This
 * header owns the three decisions those primitives are steered by, because each
 * of them fails silently:
 *
 *   1. how long the next liveness-fed sleep may be — get it wrong and either
 *      the watchdog is starved (a reboot) or the loop never terminates (a
 *      permanent wedge, which is worse: see below);
 *   2. who releases the gate token when a submitter gives up and the worker is
 *      still inside the lookup — release it twice and two lookups run
 *      concurrently in sts_aaa.c's shared static buffers; release it never and
 *      the management plane is denied for the rest of the uptime;
 *   3. how much wall clock an UNAUTHENTICATED caller may buy.
 *
 * None of the three produces a crash, a log line or a failing request when it
 * is wrong. All three are therefore here, as functions with one call site each
 * in sts_secops.c, and pinned by tests/host/test_secops_policy.c.
 *
 * ---------------------------------------------------------------------------
 * THE TRADE THIS FILE EXISTS TO RECORD
 * ---------------------------------------------------------------------------
 *
 * sts_secops.h explains why a management-plane AAA lookup runs on a worker
 * thread while the caller sleeps feeding sts_liveness_feed(): a bare
 * sts_aaa_check() withholds the watchdog kick for the operator's whole
 * configured backend chain (~540 s worst case), and the TPS3430 cold-cycles a
 * running grandmaster over one failed login.
 *
 * That fix has a cost, and the cost is not obvious. Feeding liveness through
 * the wait converts a DETECTABLE hang into an UNDETECTABLE one. The watchdog is
 * the mechanism that recovers a wedged box; a wait that satisfies it while
 * making no progress has disabled exactly the recovery designed for this case.
 * So the wrapper is only safe if every wait inside it is bounded — an
 * unbounded, liveness-fed wait is the worst outcome available here, strictly
 * worse than either the reboot or an honest denial.
 *
 * Both halves matter and neither is optional:
 *
 *   feed liveness   so a slow RADIUS server cannot reboot the appliance;
 *   bound the wait  so a dead or stuck worker degrades to a denial instead of
 *                   silencing the watchdog forever.
 *
 * The budgets below are where the second half is decided.
 */

#ifndef STS1000_ZEPHYR_NET_STS_SECOPS_POLICY_H_
#define STS1000_ZEPHYR_NET_STS_SECOPS_POLICY_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------- the budgets */

/**
 * How long the caller sleeps between liveness feeds while it waits.
 *
 * Must be comfortably under CONFIG_STS1000_LIVENESS_DEADLINE_MS (5 000) with
 * room for the caller to be descheduled by higher-priority work; 250 ms gives a
 * 20x margin and costs four wakeups a second during a lookup that is, by
 * definition, already slow. sts_secops.c BUILD_ASSERTs it against the real
 * Kconfig value, which this Zephyr-free header cannot see.
 */
#define STS_AAA_FED_SLICE_MS 250U

/**
 * Total wall clock an UNAUTHENTICATED lookup may occupy — the answer to the
 * "one POST /api/login stalls the whole HTTP plane" problem.
 *
 * ---------------------------------------------------------------------------
 * The problem
 * ---------------------------------------------------------------------------
 * A web worker holds `api_lock` across rest_dispatch(), and POST /api/login
 * reaches the AAA chain from inside it. Nobody has authenticated yet — that is
 * what the route is for. Before this budget existed, one such request parked a
 * worker inside the fed wait for the operator's whole configured chain timeout
 * (~540 s, sts_secops.h) with `api_lock` held, so every other REST request,
 * every WebSocket upgrade and every DFU step queued behind an anonymous
 * caller's failed login — and the liveness feeding meant nothing recovered the
 * box, because nothing had noticed.
 *
 * ---------------------------------------------------------------------------
 * The three candidate fixes, and why this one
 * ---------------------------------------------------------------------------
 *
 *  (a) Bound the pre-auth path far more tightly than the post-auth one.
 *      CHOSEN. See the number below.
 *
 *  (b) Drop `api_lock` across the lookup, as the maintenance plane drops
 *      `mp_lock`. REJECTED. sts_web.c's web_remote_auth() sets out why in
 *      full: `api_lock` is the only mutual exclusion for auth_web_ctx_t AND
 *      rest_ctx_t, neither of which is internally locked, and the release
 *      window is not "between operations" the way MP's is — this thread is
 *      *inside* auth_web_login(). A second worker entering auth_web_login(),
 *      auth_web_reload() or auth_web_set_password() through the window
 *      corrupts the credential store. Trading a bounded stall for memory
 *      corruption in the thing that decides who is an admin is not a trade.
 *
 *  (c) Cap the number of concurrent in-flight unauthenticated lookups.
 *      REJECTED as a no-op: the gate already caps them at one, board-wide, and
 *      `api_lock` independently caps the web plane's at one. A cap of one on a
 *      thing already capped at one changes nothing; the damage was never
 *      concurrency, it was duration.
 *
 * ---------------------------------------------------------------------------
 * Why 10 s
 * ---------------------------------------------------------------------------
 * The bound has to clear a working backend and fall well short of a stalled
 * one, and the two are three orders of magnitude apart:
 *
 *   a reachable RADIUS/LDAP server on the management LAN answers in single-
 *   digit milliseconds; the cfg schema's *defaults* give a single backend
 *   3 000 x 3 = 9 s (RADIUS), 5 000 ms (TACACS+) or 5 000 ms (LDAP) before it
 *   is written off; a blackholed one burns the whole chain, ~540 s.
 *
 * 10 s sits above every default single-backend budget except RADIUS-with-all-
 * retries, which it matches, and 54x below the unbounded case. It is also 2x
 * CONFIG_STS1000_LIVENESS_DEADLINE_MS, so it is unambiguously a duration that
 * needs this wrapper rather than one that could have been run inline.
 *
 * The asymmetry is what settles it. Too tight costs one retryable 401 on a
 * login against a pathologically slow chain, and the operator can still reach
 * the box over the console, over SNMP, and through a local account (the chain
 * tries `local` first at the shipped default, so a local admin never reaches
 * this path at all). Too loose costs the entire HTTPS management plane, to an
 * anonymous caller, undetectably. Err tight.
 *
 * A caller that legitimately needs the long form — the physical-access console
 * planes, which stall nothing but their own session — passes
 * STS_AAA_FED_CEILING_MS instead.
 */
#define STS_AAA_FED_PREAUTH_MS 10000U

/**
 * The absolute ceiling on any fed lookup, and the reason no wait in
 * sts_secops.c is unbounded.
 *
 * Its job is NOT to cut a slow chain short — 540 s is the documented worst case
 * for a fully-configured local->radius->tacacs->ldap chain at the schema's
 * maxima, and this sits above it so no legitimate configuration is truncated.
 * Its job is to put a finite number on "the worker never came back", so that a
 * worker which has died or is stuck in a server's one-octet-per-timeout dribble
 * degrades to a denial rather than parking every management plane forever while
 * feeding the watchdog that would otherwise have recovered the board.
 *
 * A thread that has actually terminated is caught up front and answered
 * immediately (sts_secops.c, worker_gone()); this covers the case where it is
 * alive and making no progress, which no liveness flag can detect.
 */
#define STS_AAA_FED_CEILING_MS 600000U

/* The documented worst-case chain, from sts_secops.h's arithmetic. */
#define STS_AAA_FED_CHAIN_WORST_MS 540000U

_Static_assert(STS_AAA_FED_PREAUTH_MS > STS_AAA_FED_SLICE_MS * 4U,
	       "a budget under a few feed slices makes the wrapper pointless");
_Static_assert(STS_AAA_FED_PREAUTH_MS < STS_AAA_FED_CEILING_MS,
	       "an unauthenticated caller must not buy the console's budget");
_Static_assert(STS_AAA_FED_CEILING_MS >= STS_AAA_FED_CHAIN_WORST_MS,
	       "the ceiling must not truncate a legally-configured chain");

/* -------------------------------------------------------------- the states */

/**
 * The request slot, which is also the gate token's whereabouts.
 *
 * Exactly one party may release the gate per acquisition. Which one depends on
 * whether the submitter is still waiting when the worker finishes, and that is
 * the whole content of this enum.
 */
typedef enum {
	/** No lookup in flight. The gate is free, or about to be. */
	STS_AAA_SLOT_IDLE = 0,
	/** The worker owns the slot and a submitter is waiting for the answer. */
	STS_AAA_SLOT_RUNNING,
	/** The worker owns the slot and the submitter has given up on it. */
	STS_AAA_SLOT_ABANDONED,
} sts_aaa_slot_t;

/** What the worker must do when its lookup returns. */
typedef struct {
	uint8_t next;    /**< slot state to store */
	bool publish;    /**< signal the waiting submitter; it owns the answer */
	bool release;    /**< release the gate: nobody is waiting to do it */
	bool discard;    /**< wipe the whole request: nobody will read it */
} sts_aaa_fed_finish_t;

/** What a submitter must do when its budget runs out. */
typedef struct {
	uint8_t next;    /**< slot state to store */
	bool take;       /**< the answer landed after all: read and return it */
	bool release;    /**< release the gate here; the worker will not */
} sts_aaa_fed_expiry_t;

/* ----------------------------------------------------------- the decisions */

/**
 * How long the next liveness-fed sleep may be.
 *
 * @param now_ms       monotonic milliseconds now.
 * @param deadline_ms  monotonic milliseconds at which the budget is spent.
 *
 * @return 1..STS_AAA_FED_SLICE_MS while there is budget left, 0 once there is
 *         not. Never negative, never longer than one slice, and never 0 while
 *         time remains — so a caller looping on it both terminates and keeps
 *         feeding, which are the two ways this can be wrong.
 */
static inline int32_t sts_aaa_fed_slice_ms(int64_t now_ms, int64_t deadline_ms)
{
	int64_t left = deadline_ms - now_ms;

	if (left <= 0) {
		return 0;
	}
	if (left > (int64_t)STS_AAA_FED_SLICE_MS) {
		return (int32_t)STS_AAA_FED_SLICE_MS;
	}
	return (int32_t)left;
}

/**
 * May a submitter keep waiting for the gate?
 *
 * IDLE and RUNNING both mean "the holder is inside somebody's budget and will
 * release" — wait. ABANDONED means the holder already blew a deadline, so
 * waiting for it is waiting for a thing known to be slow; deny immediately
 * instead.
 *
 * This is what stops a *sustained* attack from doing what a single request no
 * longer can. Without it, an attacker who keeps a lookup wedged simply pays the
 * pre-auth budget again on every subsequent request and `api_lock` is held
 * continuously; with it, the first request stalls the plane once and every
 * later one is refused in microseconds.
 */
static inline bool sts_aaa_fed_wait_ok(uint8_t slot)
{
	return slot != (uint8_t)STS_AAA_SLOT_ABANDONED;
}

/**
 * The worker's disposition, decided from the slot state under the hand-off
 * lock.
 *
 * The IDLE row is unreachable — the worker only runs after a submitter set
 * RUNNING — and is defined anyway, because the defensive answer is not the
 * obvious one: it discards the credential but does NOT release the gate. A
 * spurious release is the one failure that breaks mutual exclusion and puts two
 * lookups inside sts_aaa.c's shared static buffers at once; a spurious
 * non-release only denies logins, loudly and recoverably by reboot.
 */
static inline sts_aaa_fed_finish_t sts_aaa_fed_finish(uint8_t slot)
{
	sts_aaa_fed_finish_t f;

	f.next = (uint8_t)STS_AAA_SLOT_IDLE;

	switch (slot) {
	case (uint8_t)STS_AAA_SLOT_RUNNING:
		f.publish = true;
		f.release = false;
		f.discard = false;
		break;
	case (uint8_t)STS_AAA_SLOT_ABANDONED:
		f.publish = false;
		f.release = true;
		f.discard = true;
		break;
	default:
		f.publish = false;
		f.release = false;
		f.discard = true;
		break;
	}
	return f;
}

/**
 * The submitter's disposition when its budget expires, decided from the slot
 * state and one last non-blocking look at the answer semaphore, both under the
 * hand-off lock.
 *
 * @param slot      the slot state.
 * @param answered  the worker signalled inside the same critical section, i.e.
 *                  it beat the deadline by less than the lock hold.
 *
 * Exactly one of `release` here and `release` in sts_aaa_fed_finish() is true
 * for any one acquisition. That is the invariant; it is why these two are
 * functions rather than inline branches.
 */
static inline sts_aaa_fed_expiry_t sts_aaa_fed_expiry(uint8_t slot, bool answered)
{
	sts_aaa_fed_expiry_t e;

	if (answered) {
		/* The worker already moved the slot to IDLE when it published. */
		e.next = (uint8_t)STS_AAA_SLOT_IDLE;
		e.take = true;
		e.release = true;
	} else if (slot == (uint8_t)STS_AAA_SLOT_RUNNING) {
		e.next = (uint8_t)STS_AAA_SLOT_ABANDONED;
		e.take = false;
		e.release = false;
	} else {
		/* No worker owns the slot and no answer is coming: this
		 * submitter is the only party that can free the gate. */
		e.next = (uint8_t)STS_AAA_SLOT_IDLE;
		e.take = false;
		e.release = true;
	}
	return e;
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_NET_STS_SECOPS_POLICY_H_ */
