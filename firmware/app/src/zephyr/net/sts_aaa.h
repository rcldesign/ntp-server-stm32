/*
 * STS1000 "Meridian" — management AAA (spec §9.4).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * One entry point for every management surface — the web server, the MCP console
 * channel and the shell — so the lockout counters, the positive cache and the
 * role mapping are shared rather than reimplemented three times.
 *
 * Threading: sts_aaa_check() blocks (DNS, a UDP round trip, or a TCP+TLS
 * handshake) for up to the configured per-backend timeout. Call it from a
 * management thread, never from the timing path and never from an ISR.
 */

#ifndef STS1000_ZEPHYR_NET_STS_AAA_H_
#define STS1000_ZEPHYR_NET_STS_AAA_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "auth/auth.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Build the AAA context and apply the security configuration. */
int sts_aaa_start(void);

/** Re-read the 0x0A security group after a cfg commit. Keeps the lockout table. */
void sts_aaa_reapply(void);

/**
 * Authenticate and authorise.
 *
 * @param out_role  Receives an auth_role_t; AUTH_ROLE_NONE unless the return
 *                  value is 0.
 *
 * @retval 0               Accepted.
 * @retval -EACCES         Rejected by an authority.
 * @retval -EBUSY          Locked out by the brute-force policy.
 * @retval -EHOSTUNREACH   No backend could answer. **Treat as a denial** — this
 *                         is never an allow-on-failure.
 * @retval -EINVAL         Empty or over-long user/secret.
 */
int sts_aaa_check(const char *user, const char *secret, uint8_t *out_role);

/** Clear a user's lockout (an explicit administrative unlock). */
int sts_aaa_unlock(const char *user);

/** Drop every cached decision (after a credential or chain change). */
void sts_aaa_flush(void);

/** Snapshot the AAA counters, for `diag` and the SNMP/REST surfaces. */
int sts_aaa_stats(auth_stats_t *out);

/**
 * The remote-backend hook core/auth calls. Public only so the AAA context can be
 * wired to it; nothing else should call it directly.
 */
int sts_aaa_remote(void *ctx, uint8_t backend, const char *user,
		   const char *secret, uint8_t *out_role);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_NET_STS_AAA_H_ */
