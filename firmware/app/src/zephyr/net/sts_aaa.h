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

/* -------------------------------------------------- the LDAPS trust anchor */

/**
 * The installed LDAP trust anchor, as the management surfaces describe it.
 *
 * Field widths match rest_cert_t's, so the REST encoder copies rather than
 * reformats. `subject` is the FIRST certificate's subject DN when the blob
 * holds more than one — the set is registered whole, and naming the first is
 * enough for an operator to recognise which anchor is loaded.
 */
typedef struct {
	bool present;   /**< a parsed anchor is registered */
	bool persisted; /**< it survives a reboot (i.e. /lfs took it) */
	char subject[72];
	char not_after[24];
	char sha256_fp[68]; /**< lowercase hex of the first certificate's DER */
} sts_aaa_ldap_ca_t;

/**
 * Describe the installed LDAP trust anchor.
 *
 * @retval 0        @p out is filled; `present` says whether there is one.
 * @retval -EINVAL  @p out is NULL.
 * @retval -ENOTSUP This build has no TLS socket layer, so LDAPS cannot run.
 */
int sts_aaa_ldap_ca_info(sts_aaa_ldap_ca_t *out);

/**
 * Install an operator-supplied trust anchor: one or more PEM CERTIFICATE
 * blocks, validated, persisted to /lfs and registered under
 * STS_LDAP_CA_SEC_TAG.
 *
 * Fail-hard rather than fail-stale: an anchor that is refused leaves NO anchor
 * installed, so `sec.ldap.mode = 2` then refuses with "no LDAP CA installed"
 * instead of quietly continuing to trust the previous one while the operator
 * has been told the new one was rejected. Losing the anchor costs LDAP logins,
 * not access — the chain still falls through to the local admin account.
 *
 * @retval 0        Installed and live.
 * @retval -EBADMSG Not a usable PEM certificate (see sts_ldap_ca_check()).
 * @retval -EFBIG   Larger than STS_LDAP_CA_PEM_MAX.
 * @retval -EPERM   The blob carries private-key material.
 * @retval -EROFS   Accepted and live, but /lfs could not persist it.
 * @retval -ENOTSUP This build has no TLS socket layer.
 */
int sts_aaa_ldap_ca_install(const char *pem, size_t len);

/**
 * Erase the trust anchor: the credential entry, the RAM copy and the /lfs file.
 *
 * Called by the factory-reset sweep (sts_sec_factory_wipe()). @retval 0 also
 * covers "there was nothing to erase".
 */
int sts_aaa_ldap_ca_erase(void);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_NET_STS_AAA_H_ */
