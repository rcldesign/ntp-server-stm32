/*
 * STS1000 "Meridian" — the LDAPS transport decision and its trust anchor, as
 * pure logic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/net/, and deliberately free of every Zephyr dependency
 * so tests/host can compile it — the same arrangement as sts_secops_policy.h
 * and sts_ntp_keys.h, for the same reason: what it decides is invisible at
 * runtime when it is wrong.
 *
 * ---------------------------------------------------------------------------
 * WHY THIS EXISTS: LDAPS THAT CANNOT COMPLETE A HANDSHAKE
 * ---------------------------------------------------------------------------
 *
 * `sec.ldap.mode = 2` opens a Zephyr TLS socket. Zephyr's socket layer defaults
 * `options.verify_level` to -1 (sockets_tls.c) and then only calls
 * mbedtls_ssl_conf_authmode() when it is NOT -1 — so an unset TLS_PEER_VERIFY
 * leaves the mbedTLS *client* default in force, which is
 * MBEDTLS_SSL_VERIFY_REQUIRED. The same function additionally forces
 * mbedtls_ssl_set_hostname(ssl, "") when the caller set no TLS_HOSTNAME.
 *
 * The consequence is not "insecure". It is "impossible": with no
 * TLS_SEC_TAG_LIST there is no CA chain (tls_mbedtls_set_credentials() only
 * calls tls_set_ca_chain() when a tag supplied one), verification is required,
 * and the handshake can never succeed. The bind password never reaches the wire
 * — the failure is closed — but the feature does not work, and the operator is
 * left debugging a bare handshake error.
 *
 * So LDAPS needs three things that all live here or immediately behind here:
 *
 *   1. a trust anchor the operator installs (STS_LDAP_CA_SEC_TAG below);
 *   2. TLS_HOSTNAME, so the certificate's *name* is checked and not merely its
 *      chain — a chain check alone accepts any host the CA ever signed;
 *   3. a refusal, BEFORE the socket is opened, when no anchor is installed, so
 *      the log names the cause instead of the symptom.
 *
 * Point 3 is the whole reason the transport decision is a function rather than
 * an `if` at the call site: "refuse early, and say why" is a behaviour, and a
 * behaviour that only shows up in a log line is one nothing else can catch.
 *
 * ---------------------------------------------------------------------------
 * WHAT IS NOT HERE
 * ---------------------------------------------------------------------------
 *
 * TLS_PEER_VERIFY. sts_aaa.c deliberately never sets it, because Zephyr's
 * default for a client is already REQUIRED and any value this code could pass
 * would be a chance to pass the wrong one. sts_web.c:open_listener() does set
 * TLS_PEER_VERIFY_NONE, but that is the *server* socket declining to demand a
 * client certificate from a browser — the opposite direction, and not a
 * precedent for relaxing what this client demands of a directory server.
 *
 * ---------------------------------------------------------------------------
 * AND HOW THE ANCHOR IS TAKEN AWAY AGAIN
 * ---------------------------------------------------------------------------
 *
 * The last section of this header owns the *erase*, which is a separate
 * decision with a separate failure mode: the erase is the factory reset's only
 * means of retracting "which directory may tell this box who its administrators
 * are", and it used to be able to cold-cycle the board instead of running. See
 * sts_ldap_ca_erase_t.
 */

#ifndef STS1000_ZEPHYR_NET_STS_LDAP_CA_H_
#define STS1000_ZEPHYR_NET_STS_LDAP_CA_H_

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- the anchor */

/**
 * Zephyr credential security tag for the LDAP trust anchor.
 *
 * Distinct from STS_WEB_SEC_TAG (0x57454231, "WEB1", sts_web.h) and in the same
 * four-ASCII-character style. It is defined HERE, next to the AAA code that
 * binds it, rather than in sts_web.h: the web plane's server identity and the
 * AAA client's trust anchor are opposite ends of TLS and must never be mixed
 * up, and a tag that names one of them belongs with its owner.
 */
#define STS_LDAP_CA_SEC_TAG 0x4C444131 /* "LDA1" */

/**
 * Where the operator-installed anchor is persisted.
 *
 * The same /lfs/tls directory sts_cert.c uses for the server identity
 * (STS_CERT_DIR in sts_web.h), spelled out here so this header stays free of
 * that include. One directory, so the factory-reset sweep and any future
 * sealing layer have one place to look.
 */
#define STS_LDAP_CA_DIR "/lfs/tls"
#define STS_LDAP_CA_PATH STS_LDAP_CA_DIR "/ldap_ca.pem"

/**
 * Largest anchor blob accepted, bytes (excluding the NUL sts_aaa.c appends).
 *
 * Sized by the worst realistic enterprise root rather than by a round number.
 * An RSA-4096 root: 512-byte modulus + 512-byte signature + ~400 bytes of TBS
 * (DN, validity, extensions) ~= 1 500 bytes DER, base64 = 2 000 characters,
 * plus line breaks at 64 columns (~32) and the 52-byte BEGIN/END framing —
 * ~2 085 bytes for ONE such certificate. 3 072 clears that with room for a
 * two-certificate anchor set (a root plus a cross-signed root), which is the
 * case that would otherwise force an operator to choose. It also matches
 * STS_CERT_PEM_MAX, so the two PEM buffers on this board are one size.
 */
#define STS_LDAP_CA_PEM_MAX 3072U

_Static_assert(STS_LDAP_CA_PEM_MAX >= 2176U,
	       "an RSA-4096 root is ~2 085 bytes of PEM; a cap under that "
	       "silently excludes the most common enterprise CA");

/* PEM framing this header recognises. The base64 alphabet contains no '-', so
 * none of these can appear inside a certificate body. */
#define STS_LDAP_CA_BEGIN "-----BEGIN CERTIFICATE-----"
#define STS_LDAP_CA_END "-----END CERTIFICATE-----"
/** Matches BEGIN {,EC ,RSA ,ENCRYPTED }PRIVATE KEY. */
#define STS_LDAP_CA_KEYTAG "PRIVATE KEY-----"

/* ------------------------------------------------------- the mode selector */

/** `sec.ldap.mode` (cfg schema 0x0A26), which the schema bounds to 0..2. */
#define STS_LDAP_MODE_PLAIN 0U
#define STS_LDAP_MODE_STARTTLS 1U
#define STS_LDAP_MODE_LDAPS 2U

/** What do_ldap() does with the configured mode. */
typedef enum {
	/** Plain LDAP on the configured port. Open a TCP socket. */
	STS_LDAP_GO_PLAIN = 0,
	/** LDAPS. Open a TLS socket and arm it with the anchor. */
	STS_LDAP_GO_LDAPS,
	/** A TLS mode in a build with no TLS socket layer. */
	STS_LDAP_STOP_NO_TLS_BUILD,
	/** StartTLS, which Zephyr's socket-typed TLS cannot perform. */
	STS_LDAP_STOP_STARTTLS,
	/** LDAPS with no trust anchor installed. */
	STS_LDAP_STOP_NO_CA,
	/** A mode outside the schema's 0..2. */
	STS_LDAP_STOP_BAD_MODE,
} sts_ldap_go_t;

/**
 * Decide the transport before any socket is created.
 *
 * @param mode       `sec.ldap.mode`.
 * @param tls_build  CONFIG_NET_SOCKETS_SOCKOPT_TLS is set in this image.
 * @param ca_ready   a parsed trust anchor is registered under
 *                   STS_LDAP_CA_SEC_TAG.
 *
 * The order of the tests is the contract, not an implementation detail:
 *
 *   - mode 0 answers first, so the plain path is never gated on TLS state;
 *   - an out-of-range mode is refused NEXT, before the build test, because
 *     "I do not understand what you asked for" must never fall through to
 *     plaintext. (It used to: the pre-existing `if (mode == 2) TLS; else if
 *     (mode == 1) refuse;` chain sent mode 3 over a cleartext socket. The
 *     schema bounds the key to 0..2, so that needed a corrupt store to reach —
 *     but "unreachable" and "safe" are different claims.)
 *   - the build test precedes the StartTLS and anchor tests so a build without
 *     TLS reports the one thing the operator can act on;
 *   - the anchor test is LAST, so it is only reported for the mode that
 *     actually consumes an anchor.
 */
static inline sts_ldap_go_t sts_ldap_transport(uint8_t mode, bool tls_build,
					       bool ca_ready)
{
	if (mode == (uint8_t)STS_LDAP_MODE_PLAIN) {
		return STS_LDAP_GO_PLAIN;
	}
	if (mode > (uint8_t)STS_LDAP_MODE_LDAPS) {
		return STS_LDAP_STOP_BAD_MODE;
	}
	if (!tls_build) {
		return STS_LDAP_STOP_NO_TLS_BUILD;
	}
	if (mode == (uint8_t)STS_LDAP_MODE_STARTTLS) {
		return STS_LDAP_STOP_STARTTLS;
	}
	if (!ca_ready) {
		return STS_LDAP_STOP_NO_CA;
	}
	return STS_LDAP_GO_LDAPS;
}

/** True when do_ldap() may create a socket at all. */
static inline bool sts_ldap_go_opens_socket(sts_ldap_go_t go)
{
	return (go == STS_LDAP_GO_PLAIN) || (go == STS_LDAP_GO_LDAPS);
}

/** True when that socket must be a TLS socket armed with the anchor. */
static inline bool sts_ldap_go_is_tls(sts_ldap_go_t go)
{
	return go == STS_LDAP_GO_LDAPS;
}

/**
 * Why the attempt was refused, in the operator's terms.
 *
 * Every one of these is a configuration the operator can fix, so the string
 * names the missing thing rather than the code path. "no LDAP CA installed" is
 * the whole point of the exercise: without it the operator sees an mbedTLS
 * handshake error and has no way to tell a missing anchor from an unreachable
 * server, a wrong port or a clock skew.
 */
static inline const char *sts_ldap_go_reason(sts_ldap_go_t go)
{
	switch (go) {
	case STS_LDAP_GO_PLAIN:
		return "plain LDAP";
	case STS_LDAP_GO_LDAPS:
		return "LDAPS";
	case STS_LDAP_STOP_NO_TLS_BUILD:
		return "this build has no TLS socket layer";
	case STS_LDAP_STOP_STARTTLS:
		return "StartTLS cannot upgrade a socket-typed TLS stack; "
		       "use LDAPS (sec.ldap.mode=2) on 636";
	case STS_LDAP_STOP_NO_CA:
		return "no LDAP CA installed; POST the issuer PEM to "
		       "/api/v1/security/ldap-ca";
	case STS_LDAP_STOP_BAD_MODE:
		return "sec.ldap.mode is out of range (0=plain, 1=StartTLS, "
		       "2=LDAPS)";
	default:
		return "unknown";
	}
}

/* ------------------------------------------------------ the anchor's blob */

/** Why an offered trust anchor was refused. */
typedef enum {
	STS_LDAP_CA_OK = 0,
	/** Nothing at all, or a NULL pointer. */
	STS_LDAP_CA_EMPTY,
	/** Longer than STS_LDAP_CA_PEM_MAX. */
	STS_LDAP_CA_TOO_BIG,
	/** No complete BEGIN/END CERTIFICATE block. */
	STS_LDAP_CA_MALFORMED,
	/**
	 * Contains private-key material.
	 *
	 * mbedTLS would simply ignore the key block, so this refusal is not
	 * about parsing — it is about what would be WRITTEN. The anchor is
	 * persisted verbatim to /lfs, so accepting a server bundle pasted into
	 * the wrong box would leave a private key in a plain file that nothing
	 * on any normal path ever erases. A trust anchor is a public object;
	 * anything else in the blob is an accident worth naming.
	 */
	STS_LDAP_CA_HAS_KEY,
} sts_ldap_ca_verdict_t;

/** Bounded substring search. @return index, or SIZE_MAX when absent. */
static inline size_t sts_ldap_ca_find(const char *hay, size_t n,
				      const char *needle, size_t nn, size_t from)
{
	size_t i;

	if ((hay == NULL) || (needle == NULL) || (nn == 0U) || (nn > n)) {
		return (size_t)-1;
	}
	for (i = from; (i + nn) <= n; i++) {
		if (memcmp(&hay[i], needle, nn) == 0) {
			return i;
		}
	}
	return (size_t)-1;
}

/**
 * Screen an offered anchor before anything live is touched.
 *
 * This is the cheap structural gate, not the cryptographic one: sts_aaa.c
 * follows it with mbedtls_x509_crt_parse(), which is the only thing that can
 * say the DER is well-formed and the certificate is a certificate. The split is
 * deliberate — everything an operator actually gets wrong (an empty paste, a
 * truncated copy, the wrong file, a whole server bundle including its key) is
 * caught here, on the caller's buffer, with no allocation and no Zephyr.
 *
 * An embedded NUL counts as malformed rather than as a truncation to tolerate:
 * mbedTLS finds the PEM framing with strstr() over a NUL-terminated buffer, so
 * a blob with a NUL in the middle parses as whatever precedes it while /lfs
 * stores the whole thing. Two different views of one file is not a state worth
 * having.
 */
static inline sts_ldap_ca_verdict_t sts_ldap_ca_check(const char *pem, size_t len)
{
	size_t b;
	size_t e;

	if ((pem == NULL) || (len == 0U)) {
		return STS_LDAP_CA_EMPTY;
	}
	if (len > (size_t)STS_LDAP_CA_PEM_MAX) {
		return STS_LDAP_CA_TOO_BIG;
	}
	if (memchr(pem, '\0', len) != NULL) {
		return STS_LDAP_CA_MALFORMED;
	}
	if (sts_ldap_ca_find(pem, len, STS_LDAP_CA_KEYTAG,
			     sizeof(STS_LDAP_CA_KEYTAG) - 1U, 0U) != (size_t)-1) {
		return STS_LDAP_CA_HAS_KEY;
	}

	b = sts_ldap_ca_find(pem, len, STS_LDAP_CA_BEGIN,
			     sizeof(STS_LDAP_CA_BEGIN) - 1U, 0U);
	if (b == (size_t)-1) {
		return STS_LDAP_CA_MALFORMED;
	}
	e = sts_ldap_ca_find(pem, len, STS_LDAP_CA_END,
			     sizeof(STS_LDAP_CA_END) - 1U,
			     b + sizeof(STS_LDAP_CA_BEGIN) - 1U);
	if (e == (size_t)-1) {
		return STS_LDAP_CA_MALFORMED;
	}
	return STS_LDAP_CA_OK;
}

/** Why an offered anchor was refused, in the operator's terms. */
static inline const char *sts_ldap_ca_reason(sts_ldap_ca_verdict_t v)
{
	switch (v) {
	case STS_LDAP_CA_OK:
		return "ok";
	case STS_LDAP_CA_EMPTY:
		return "empty";
	case STS_LDAP_CA_TOO_BIG:
		return "larger than the 3072-byte anchor buffer";
	case STS_LDAP_CA_MALFORMED:
		return "no complete BEGIN/END CERTIFICATE block";
	case STS_LDAP_CA_HAS_KEY:
		return "contains a private key; a trust anchor is the issuer "
		       "certificate only";
	default:
		return "unknown";
	}
}

/* ----------------------------------------------------- erasing the anchor */

/**
 * What one attempt at erasing the anchor managed to do.
 *
 * ---------------------------------------------------------------------------
 * WHY THE ERASE HAS TWO HALVES AND ONLY ONE OF THEM IS THE ANSWER
 * ---------------------------------------------------------------------------
 *
 * Erasing the anchor touches two places with opposite safety requirements:
 *
 *   THE /lfs FILE (STS_LDAP_CA_PATH) is what survives a reboot, and it is the
 *   only half that has to happen. Nothing reads or writes it under sts_aaa.c's
 *   exchange mutex, so unlinking it needs no lock at all.
 *
 *   THE RAM COPY and its Zephyr credential entry do need that mutex: do_ldap()
 *   is the credential's only reader, and sts_aaa.c deletes the credential entry
 *   BEFORE zeroing the buffer the entry points at (tls_credentials.c stores the
 *   pointer and copies nothing).
 *
 * They used to be one critical section entered with K_FOREVER — and that mutex
 * is held for the WHOLE of a backend exchange. sts_secops.h prices that at up
 * to 240 s for LDAP alone and ~540 s for a configured local->radius->tacacs->
 * ldap chain, with no ceiling at all when a directory dribbles one octet just
 * inside each per-recv timeout.
 *
 * Every caller of the erase is a factory reset running INLINE on a thread that
 * feeds a liveness participant against CONFIG_STS1000_LIVENESS_DEADLINE_MS
 * (5 000): the web worker (net/sts_web.c pv_factory_reset), the panel's ui
 * thread (ui/sts_ui.c UI_ACTION_FACTORY_RESET) and the mcp console thread
 * (console/sts_mcp.c mcp_cfg_factory_reset). So a factory reset issued while an
 * unauthenticated login was parked on a slow or hostile directory withheld the
 * watchdog kick, and the external TPS3430 cold-cycled the board — AFTER
 * sts_cfg_factory_reset(), auth_web_wipe() and sts_cert_reset() had already
 * run, and BEFORE the plane could annunciate the reset as incomplete. The unit
 * came back with its credentials and TLS identity gone, looking factory-clean,
 * still trusting the previous operator's certificate authority. That is exactly
 * the outcome erasing the anchor exists to prevent, reached by the erase
 * itself, and reachable on demand by anyone who can make the box attempt a
 * bind.
 *
 * Hence the shape recorded here:
 *
 *   THE PERSISTED HALF IS UNCONDITIONAL AND LOCK-FREE.  It runs first, before
 *   any mutex is taken, so nothing an authentication exchange is doing can
 *   delay or skip it.
 *
 *   THE RAM HALF IS BEST-EFFORT UNDER A BOUNDED WAIT.  Overrunning that wait is
 *   not a failure, because the caller reboots and the reboot clears RAM.
 *
 *   THE RESULT REPORTS THE PERSISTED HALF ONLY.  A `false` from the RAM half
 *   must never become -EIO: that would report an anchor which cannot outlive
 *   the next second as a failed erase, and (via sts_factory_result()) brand a
 *   correctly-wiped unit as one that must not be treated as decommissioned.
 *
 * The third rule is sound for exactly one reason, and it is a property of the
 * CALLERS rather than of this file: all three reboot unconditionally. The panel
 * gates its sys_reboot() on sts_factory_should_reboot(), which takes the
 * outcome and ignores it by construction; the web and mcp planes each
 * k_work_reschedule() their reboot outside every conditional. If that ever
 * stops being true on any plane, this rule has to be revisited with it —
 * tests/host/test_factory_policy.c is what keeps it true.
 */
typedef struct {
	/**
	 * /lfs was mounted, so the unlink had something it could reach.
	 *
	 * A dead /lfs is NOT a failure — the same convention sts_cert_reset()
	 * uses, and for the same reason: there is then nothing persisted for
	 * this call to remove, and the volatile copies have been dealt with
	 * separately.
	 */
	bool fs_ready;
	/**
	 * Every unlink attempt this call made returned 0 or -ENOENT.
	 * Meaningless when !fs_ready; initialise it true, so "no attempt was
	 * needed" and "the attempt succeeded" are one state.
	 */
	bool unlinked;
	/**
	 * The exchange mutex was obtained within its bounded wait, so the RAM
	 * copy and the credential-store entry went too.
	 *
	 * False means they were left for the reboot. It never reaches the
	 * result — see the header comment — but it IS worth a log line, because
	 * "the reboot will finish this" is only true while the reboot is.
	 */
	bool ram_erased;
} sts_ldap_ca_erase_t;

/**
 * Normalise one fs_unlink() of the anchor.
 *
 * -ENOENT is the desired end state, not a failure: the erase is trying to reach
 * "there is no anchor on this box", and a unit that never had one is already
 * there.
 */
static inline bool sts_ldap_ca_unlink_ok(int rc)
{
	return (rc == 0) || (rc == -ENOENT);
}

/**
 * The result the erase reports to the factory-reset sweep.
 *
 * @retval 0     No anchor can survive the reboot by way of /lfs.
 * @retval -EIO  The persisted anchor is still there; the unit must NOT be
 *               described as factory-clean, because the next boot loads it
 *               straight back in.
 */
static inline int sts_ldap_ca_erase_result(const sts_ldap_ca_erase_t *e)
{
	if (e == NULL) {
		return -EIO;
	}
	if (e->fs_ready && !e->unlinked) {
		return -EIO;
	}
	return 0;
}

/**
 * Whether the in-RAM anchor outlived the call and was left for the reboot.
 *
 * True is not an error; it is a fact the caller must log, because it is only
 * harmless for as long as the reboot that follows is unconditional.
 */
static inline bool sts_ldap_ca_erase_ram_deferred(const sts_ldap_ca_erase_t *e)
{
	return (e != NULL) && !e->ram_erased;
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_NET_STS_LDAP_CA_H_ */
