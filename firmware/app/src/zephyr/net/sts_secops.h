/*
 * STS1000 "Meridian" — security operations that cross a glue-area seam.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Two things live here, both of them net-area implementations that the console
 * area also has to reach:
 *
 *   - sts_aaa_check_fed(), the watchdog-safe way to run a management-plane AAA
 *     lookup from a thread that feeds a liveness participant;
 *   - sts_sec_factory_wipe(), the key zeroization half of a factory reset.
 *
 * Both are declared __weak at every call site outside this area, exactly as
 * sts_web.c declares sts_dfu_port(), so a CONFIG_STS1000_NET=n image links with
 * the console area present and simply does without them.
 */

#ifndef STS1000_ZEPHYR_NET_STS_SECOPS_H_
#define STS1000_ZEPHYR_NET_STS_SECOPS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "net/sts_secops_policy.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * sts_aaa_check(), run somewhere it cannot stop a watchdog participant.
 *
 * ---------------------------------------------------------------------------
 * Why this exists at all
 * ---------------------------------------------------------------------------
 *
 * sts_aaa.h states its own cost: sts_aaa_check() "blocks (DNS, a UDP round
 * trip, or a TCP+TLS handshake) for up to the configured per-backend timeout".
 * From the cfg schema (cfg_schema.h, group 0x0A), per login attempt:
 *
 *   RADIUS   sec.radius.tmo.ms <= 30 000 x (sec.radius.retries <= 5)+1 -> 180 s
 *   TACACS+  sec.tacacs.tmo.ms <= 30 000, >= 4 blocking recv()s        -> 120 s
 *   LDAP     sec.ldap.tmo.ms   <= 30 000, >= 8 blocking recv()s        -> 240 s
 *
 * i.e. a configured chain of local->radius->tacacs->ldap is about 540 s in the
 * worst case and 69 s at the shipped defaults. Those are floors, not ceilings:
 * read_exact()/ldap_recv() loop per-recv, so a server that dribbles one octet
 * just inside each timeout extends them without bound.
 *
 * Both planes that need AAA call it from a thread that feeds
 * sts_liveness_feed(), and CONFIG_STS1000_LIVENESS_DEADLINE_MS is 5 000. So a
 * plain call would withhold the watchdog kick and COLD-CYCLE THE BOARD — from
 * one unauthenticated HTTP POST, on any unit whose operator configured a remote
 * backend that is currently unreachable. That is a worse outcome than the
 * failed login it came from, and it is why this wrapper is not optional:
 *
 *   - the web workers (src/zephyr/net/sts_web.c) share ONE liveness id, and
 *     they serialise on api_lock, so a single blocked worker takes the second
 *     one down with it as soon as a second request arrives;
 *   - the MCP console thread (src/zephyr/console/sts_mcp.c) is alone and owns
 *     its liveness id outright, so it needs no help to trip the watchdog.
 *
 * ---------------------------------------------------------------------------
 * What it does instead
 * ---------------------------------------------------------------------------
 *
 * The blocking call runs on a dedicated worker thread. The caller waits on a
 * semaphore in short slices and feeds @p live_id on every slice, so it is
 * asleep — not runnable, not holding the CPU — while remaining visibly alive to
 * the watchdog supervisor. The answer is identical to calling sts_aaa_check()
 * directly, up to the caller's budget; only the watchdog consequence changes.
 *
 * ---------------------------------------------------------------------------
 * ...and why every wait inside it is bounded
 * ---------------------------------------------------------------------------
 *
 * Feeding the watchdog through a wait turns a DETECTABLE hang into an
 * UNDETECTABLE one, so an unbounded fed wait is strictly the worst outcome
 * available here: it defeats the one mechanism that recovers a wedged board.
 * Every wait therefore carries a deadline, and overrunning it is a denial —
 * -EHOSTUNREACH, which every caller already treats as one. sts_secops_policy.h
 * holds the budgets, the arithmetic and the reasoning.
 *
 * One lookup at a time, board-wide — and, since 2026-07, that is true of every
 * caller: the web plane, the MCP console and the Maintenance Protocol all enter
 * through this function, so all three take the same gate. (MP used to call
 * sts_aaa_check() raw, which left this sentence false and let an MP login and a
 * web login sit inside sts_aaa.c's shared static buffers at the same time,
 * serialised only by that file's own mutex.) sts_aaa.c serialises its own
 * exchanges anyway, so nothing is lost by making it explicit here — and a
 * second caller waits in the same liveness-feeding loop rather than blocking
 * outright.
 *
 * @param user      NUL-terminated principal name.
 * @param secret    NUL-terminated password. Copied into a private buffer for
 *                  the worker and wiped before this function returns.
 * @param out_role  auth_role_t; AUTH_ROLE_NONE unless the return value is 0.
 * @param live_id   The CALLER's liveness participant, from
 *                  sts_liveness_register(). Negative means "not registered",
 *                  which is legal and simply skips the feeding.
 *
 * @retval 0               Accepted.
 * @retval -EACCES         Rejected by an authority.
 * @retval -EBUSY          Locked out by the brute-force policy.
 * @retval -EHOSTUNREACH   No backend could answer; AAA is not running; the
 *                         offload worker has died; another lookup still held
 *                         the gate when the budget ran out; or this lookup
 *                         overran it. **Treat as a denial** — never an
 *                         allow-on-failure.
 * @retval -EINVAL         Bad argument, or an over-long user/secret.
 */
int sts_aaa_check_fed(const char *user, const char *secret, uint8_t *out_role,
		      int live_id);

/**
 * sts_aaa_check_fed() with the caller's own wall-clock budget.
 *
 * Identical in every respect except how long it may take. Use it wherever the
 * caller is holding something the rest of the appliance needs — the web plane
 * holds `api_lock` across the whole of rest_dispatch(), so its budget is
 * STS_AAA_FED_PREAUTH_MS rather than the ceiling. Callers that stall nothing
 * but their own session (the physical-access console planes) use the plain
 * sts_aaa_check_fed() and get STS_AAA_FED_CEILING_MS.
 *
 * @param budget_ms  Total wall clock for the whole call, gate acquisition
 *                   included. Clamped into (0, STS_AAA_FED_CEILING_MS]; the
 *                   call returns within it plus at most one
 *                   STS_AAA_FED_SLICE_MS of granularity.
 *
 * Returns exactly what sts_aaa_check_fed() returns; a budget overrun is
 * -EHOSTUNREACH.
 */
int sts_aaa_check_fed_budgeted(const char *user, const char *secret,
			       uint8_t *out_role, int live_id,
			       uint32_t budget_ms);

/**
 * Zeroize the key material a factory reset must not leave behind.
 *
 * Called AFTER sts_cfg_factory_reset() has cleared the config tree and run
 * every group's appliers, because those appliers are what push the emptied
 * credential keys into the subsystems holding RAM copies of them.
 *
 * What this erases, and what it deliberately does not, is documented at the
 * implementation in src/zephyr/net/sts_web.c.
 *
 * @retval 0     Everything this owns is gone.
 * @retval -EIO  At least one step failed; the caller should report the reset as
 *               incomplete rather than claim a clean unit.
 */
int sts_sec_factory_wipe(void);

/**
 * Destroy the persisted TLS server credential.
 *
 * Deletes the private key and certificate from /lfs and from the Zephyr
 * credential store, and zeroizes the in-RAM PEM buffers. The ACME account key
 * goes too: it identifies this box to a CA.
 *
 * sts_cert.c persists the pair ON PURPOSE — a key that changes every boot makes
 * certificate pinning meaningless, and that intent is preserved everywhere
 * except here. A factory reset is precisely the moment the old identity must
 * not survive.
 *
 * HTTPS has no credential after this until sts_cert_init() runs again on the
 * next boot and mints a fresh self-signed pair. That is why the factory-reset
 * path reboots.
 *
 * @retval 0     Erased.
 * @retval -EIO  A filesystem delete failed; the RAM copy and the credential
 *               store are still cleared.
 */
int sts_cert_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_NET_STS_SECOPS_H_ */
