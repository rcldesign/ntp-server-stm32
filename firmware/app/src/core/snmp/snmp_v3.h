/*
 * STS1000 "Meridian" — core/snmp: SNMPv3 / USM (spec §5.3, §9.5).
 *
 * Platform-neutral C11, no dynamic allocation, all state in a caller-owned
 * context (ARCHITECTURE.md §4). This is the RFC 3412 message envelope plus the
 * RFC 3414 User-based Security Model; the RFC 3416 PDU inside it is handled by
 * the same code that serves SNMPv2c (snmp_internal.h), so the two versions can
 * never give a manager different answers.
 *
 * ---------------------------------------------------------------------------
 * What is implemented
 * ---------------------------------------------------------------------------
 *
 *   auth  usmHMAC192SHA256AuthProtocol (RFC 7860) — the default and the one to
 *         deploy: HMAC-SHA-256 truncated to 24 octets.
 *         usmHMACSHAAuthProtocol (RFC 3414) — HMAC-SHA-1 truncated to 12
 *         octets, kept for interoperability with managers that speak nothing
 *         else, and because RFC 3414 Appendix A.3 is the only *published* set
 *         of key-localisation test vectors in existence. See "SHA-1" below.
 *   priv  usmAesCfb128Protocol (RFC 3826) — AES-128 in 128-bit-feedback CFB.
 *
 * DES (usmDESPrivProtocol) is deliberately absent: a 56-bit cipher on a device
 * whose whole purpose is to be trusted would be worse than no privacy at all,
 * and every manager that can do SNMPv3 at all can do AES.
 *
 * ---------------------------------------------------------------------------
 * SHA-1 is here on purpose, and only for two reasons
 * ---------------------------------------------------------------------------
 *
 * port_crypto.h offers SHA-256 and HMAC-SHA-256; SHA-1 is not a primitive core
 * code is allowed to build on, and it is not added to the port. It is
 * implemented privately inside snmp_usm.c because:
 *
 *   1. usmHMACSHAAuthProtocol is the *mandatory* USM auth protocol of RFC 3414
 *      and is what a decade of deployed managers default to. A grandmaster that
 *      cannot be polled is not a grandmaster.
 *   2. RFC 3414 Appendix A.3.2 publishes Ku and Kul for the password
 *      "maplesyrup" under SHA-1. That vector is the only external check on the
 *      1 048 576-octet key-expansion algorithm, which is the single most
 *      error-prone step in USM and the one where a private mistake would go
 *      unnoticed until a manager failed to authenticate.
 *
 * SHA-256 is the default for new users; the SHA-1 protocol must be selected
 * explicitly per user.
 *
 * ---------------------------------------------------------------------------
 * AES-128-CFB from an AES-ECB port
 * ---------------------------------------------------------------------------
 *
 * port_crypto_t provides AES-ECB only. CFB with a 128-bit feedback segment is
 * built from it exactly as NIST SP 800-38A §6.3 defines:
 *
 *     O_1 = E(K, IV)                C_1 = P_1 ⊕ MSB(O_1)
 *     O_j = E(K, C_{j-1})           C_j = P_j ⊕ MSB(O_j)
 *
 * and decryption uses the *same* forward cipher on the ciphertext feedback, so
 * no AES decryption primitive is needed. The final block may be short; its
 * keystream is truncated, so ciphertext and plaintext are the same length and
 * the scopedPDU needs no padding (RFC 3826 §3.1.4). The construction is pinned
 * by the NIST SP 800-38A F.3.13/F.3.14 known-answer vectors in
 * tests/host/test_snmpv3.c.
 *
 * The IV is `msgAuthoritativeEngineBoots ‖ msgAuthoritativeEngineTime ‖ salt`,
 * both counters big-endian, the salt 8 octets carried in
 * msgPrivacyParameters (RFC 3826 §3.1.2.1). The salt must never repeat for a
 * given key; this implementation draws it from the crypto port's CSPRNG and
 * additionally mixes in a monotonic counter, so a CSPRNG that silently repeats
 * still cannot produce a repeated IV within one boot.
 *
 * ---------------------------------------------------------------------------
 * Engine ID and time
 * ---------------------------------------------------------------------------
 *
 * @ref snmp_v3_engine_id_build follows RFC 3411 §5: four octets of enterprise
 * number with the top bit set, a format octet of 5 ("administratively assigned
 * octets"), then a device-unique value — the ATECC608B serial number when the
 * part answers, else the SoC's hardware device id.
 *
 * `snmpEngineBoots` must be persisted and must increase on every restart; the
 * glue keeps it in cfg (`sec.snmpv3.boots`) and hands it to
 * @ref snmp_v3_set_boots. `snmpEngineTime` is seconds since that boot and is
 * passed in on every call rather than read from a port, so the timeliness
 * window is deterministic in tests.
 */

#ifndef STS1000_CORE_SNMP_SNMP_V3_H_
#define STS1000_CORE_SNMP_SNMP_V3_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "port/port_crypto.h"
#include "snmp/snmp.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ limits */

/** RFC 3411 SnmpEngineID is 5..32 octets. */
#define SNMP_V3_ENGINEID_MIN 5U
#define SNMP_V3_ENGINEID_MAX 32U

/** RFC 3414: msgUserName is 0..32 octets. */
#define SNMP_V3_USER_MAX 32U

/** Users the context holds. */
#ifndef SNMP_V3_USERS
#define SNMP_V3_USERS 4U
#endif

/** Longest localised key (SHA-256 digest). */
#define SNMP_V3_KEY_MAX 32U

/** Longest msgAuthenticationParameters (RFC 7860: 24 octets for SHA-256). */
#define SNMP_V3_AUTHPARAM_MAX 24U

/** AES-128 key and block, and the RFC 3826 salt. */
#define SNMP_V3_PRIV_KEY_LEN 16U
#define SNMP_V3_AES_BLOCK 16U
#define SNMP_V3_SALT_LEN 8U

/** Shortest USM passphrase accepted (RFC 3414 §11.2). */
#define SNMP_V3_PASSPHRASE_MIN 8U
/** Longest USM passphrase accepted. */
#define SNMP_V3_PASSPHRASE_MAX 64U

/** RFC 3414 §2.2.3 timeliness window, seconds. */
#define SNMP_V3_TIME_WINDOW_S 150

/** RFC 3412: msgMaxSize is at least 484. */
#define SNMP_V3_MSG_MAX_SIZE_MIN 484U

/** msgFlags bits (RFC 3412 §6.4). */
#define SNMP_V3_FLAG_AUTH 0x01U
#define SNMP_V3_FLAG_PRIV 0x02U
#define SNMP_V3_FLAG_REPORTABLE 0x04U

/** msgSecurityModel value for USM (RFC 3411). */
#define SNMP_V3_SEC_MODEL_USM 3

/* --------------------------------------------------------------- protocols */

typedef enum {
	SNMP_AUTH_NONE = 0,
	/** RFC 3414 usmHMACSHAAuthProtocol: HMAC-SHA-1, 12-octet digest. */
	SNMP_AUTH_HMAC_SHA1_96 = 1,
	/** RFC 7860 usmHMAC192SHA256AuthProtocol: HMAC-SHA-256, 24 octets. */
	SNMP_AUTH_HMAC_SHA256_192 = 2,
} snmp_auth_proto_t;

typedef enum {
	SNMP_PRIV_NONE = 0,
	/** RFC 3826 usmAesCfb128Protocol. */
	SNMP_PRIV_AES128_CFB = 1,
} snmp_priv_proto_t;

/** Security levels, ordered so a numeric comparison is a strength comparison. */
typedef enum {
	SNMP_SEC_NOAUTH_NOPRIV = 0,
	SNMP_SEC_AUTH_NOPRIV = 1,
	SNMP_SEC_AUTH_PRIV = 2,
} snmp_sec_level_t;

/** Digest length of @p proto, or 0 for SNMP_AUTH_NONE. */
size_t snmp_usm_authparam_len(uint8_t proto);

/** Localised-key length of @p proto (the hash's digest size), or 0. */
size_t snmp_usm_key_len(uint8_t proto);

/** Name of an auth protocol, for logs and the shell. Never NULL. */
const char *snmp_auth_proto_name(uint8_t proto);

/** Name of a privacy protocol. Never NULL. */
const char *snmp_priv_proto_name(uint8_t proto);

/** Name of a security level. Never NULL. */
const char *snmp_sec_level_name(uint8_t level);

/* -------------------------------------------------------------- USM crypto */

/**
 * The primitives USM needs from the platform.
 *
 * @p crypto supplies AES-ECB (for CFB), SHA-256, HMAC-SHA-256 and the CSPRNG.
 * @p sha256_stream is needed only by @ref snmp_usm_password_to_key, which has to
 * hash 1 MiB in 64-octet chunks; without it, SHA-256 *passphrases* cannot be
 * expanded and users must be provisioned with already-localised keys. SHA-1 in
 * all its forms is internal to this module.
 */
typedef struct {
	const port_crypto_t *crypto;
	const port_sha256_stream_t *sha256_stream;
} snmp_usm_ports_t;

/**
 * RFC 3414 §2.6 password-to-key: hash 1 048 576 octets of the passphrase
 * repeated, yielding the non-localised key Ku.
 *
 * The megabyte is not a typo and not negotiable — it is the whole of the
 * algorithm's deliberate cost, and shortening it produces keys no manager will
 * agree with.
 *
 * @param out      Receives @ref snmp_usm_key_len octets.
 * @param out_len  Optional; receives that length.
 *
 * @retval 0        Derived.
 * @retval -EINVAL  NULL argument, unknown protocol, or a passphrase outside
 *                  SNMP_V3_PASSPHRASE_MIN..MAX.
 * @retval -ENOTSUP The protocol needs a port this @p p does not carry.
 * @retval -EIO     A port call failed.
 */
int snmp_usm_password_to_key(const snmp_usm_ports_t *p, uint8_t proto,
			     const char *password, uint8_t *out,
			     size_t *out_len);

/**
 * RFC 3414 §2.6 key localisation: H(Ku ‖ engineID ‖ Ku).
 *
 * @retval 0        Localised.
 * @retval -EINVAL  NULL argument, unknown protocol, wrong @p ku_len, or an
 *                  engine id outside SNMP_V3_ENGINEID_MIN..MAX.
 * @retval -ENOTSUP Missing port.
 * @retval -EIO     A port call failed.
 */
int snmp_usm_localize(const snmp_usm_ports_t *p, uint8_t proto,
		      const uint8_t *ku, size_t ku_len,
		      const uint8_t *engine_id, size_t engine_id_len,
		      uint8_t *out, size_t *out_len);

/**
 * Both steps at once: passphrase → localised key for @p engine_id.
 */
int snmp_usm_password_to_localized(const snmp_usm_ports_t *p, uint8_t proto,
				   const char *password,
				   const uint8_t *engine_id,
				   size_t engine_id_len, uint8_t *out,
				   size_t *out_len);

/**
 * Truncated HMAC of a whole message under a localised auth key.
 *
 * The caller must already have zero-filled the msgAuthenticationParameters
 * field: RFC 3414 §6.3.1 computes the digest over the serialised message with
 * that field zeroed and the same length it will finally have.
 *
 * @param out      Receives @ref snmp_usm_authparam_len octets.
 * @param out_len  Optional; receives that length.
 */
int snmp_usm_auth(const snmp_usm_ports_t *p, uint8_t proto, const uint8_t *key,
		  size_t key_len, const uint8_t *msg, size_t msg_len,
		  uint8_t *out, size_t *out_len);

/** Build the RFC 3826 §3.1.2.1 IV: boots ‖ time ‖ salt, counters big-endian. */
int snmp_usm_priv_iv(uint32_t boots, uint32_t time_s,
		     const uint8_t salt[SNMP_V3_SALT_LEN],
		     uint8_t iv[SNMP_V3_AES_BLOCK]);

/**
 * AES-128 CFB with a 128-bit feedback segment, over an AES-ECB port.
 *
 * @param encrypt  true to encrypt, false to decrypt. Both directions use the
 *                 forward cipher; only the feedback source differs.
 * @param in/out   May be the same pointer (in-place is the normal case).
 *
 * @retval 0        Done.
 * @retval -EINVAL  NULL argument.
 * @retval -ENOTSUP The port has no aes_ecb_encrypt.
 * @retval -EIO     A port call failed.
 */
int snmp_usm_aes_cfb(const snmp_usm_ports_t *p,
		     const uint8_t key[SNMP_V3_PRIV_KEY_LEN],
		     const uint8_t iv[SNMP_V3_AES_BLOCK], const uint8_t *in,
		     uint8_t *out, size_t len, bool encrypt);

/* ---------------------------------------------------------------- engine id */

/**
 * Build an RFC 3411 §5 engine id from an enterprise number and a unique value.
 *
 * Layout: `(0x80000000 | pen)` big-endian, then format octet 5, then up to 27
 * octets of @p uniq. A NULL or empty @p uniq is refused rather than silently
 * producing the same engine id on every board — two agents sharing an engine id
 * is an interoperability failure a manager diagnoses as "random
 * authentication errors" months later.
 *
 * @retval >0       Engine-id length.
 * @retval -EINVAL  NULL argument, a PEN over 2^31-1, or an empty @p uniq.
 * @retval -ENOSPC  @p cap under the resulting length.
 */
int snmp_v3_engine_id_build(uint32_t pen, const uint8_t *uniq, size_t uniq_len,
			    uint8_t *out, size_t cap);

/* ------------------------------------------------------------------ users */

/** One USM user. Private. */
typedef struct {
	char name[SNMP_V3_USER_MAX + 1U];
	uint8_t auth_proto;
	uint8_t priv_proto;
	uint8_t auth_key[SNMP_V3_KEY_MAX];
	uint8_t priv_key[SNMP_V3_KEY_MAX];
	uint8_t auth_key_len;
	uint8_t priv_key_len;
	bool used;
} snmp_v3_user_t;

/* ------------------------------------------------------------------- stats */

/**
 * USM counters.
 *
 * The first six are the RFC 3414 §5 `usmStats*` MIB objects a Report PDU
 * carries; the rest are local bookkeeping.
 */
typedef struct {
	uint64_t unsupported_sec_levels; /**< usmStatsUnsupportedSecLevels .1 */
	uint64_t not_in_time_windows;    /**< usmStatsNotInTimeWindows     .2 */
	uint64_t unknown_user_names;     /**< usmStatsUnknownUserNames     .3 */
	uint64_t unknown_engine_ids;     /**< usmStatsUnknownEngineIDs     .4 */
	uint64_t wrong_digests;          /**< usmStatsWrongDigests         .5 */
	uint64_t decryption_errors;      /**< usmStatsDecryptionErrors     .6 */

	uint64_t rx;            /**< v3 datagrams handed to snmp_v3_handle() */
	uint64_t responses;     /**< Response messages produced              */
	uint64_t reports;       /**< Report messages produced                */
	uint64_t authenticated; /**< messages whose digest verified          */
	uint64_t decrypted;     /**< messages successfully decrypted         */
	uint64_t malformed;     /**< failed to parse                         */
	uint64_t bad_sec_model; /**< msgSecurityModel other than USM         */
	uint64_t bad_context;   /**< wrong contextEngineID or contextName    */
	uint64_t notifications; /**< v3 traps and informs built              */
} snmp_v3_stats_t;

/* ------------------------------------------------------------------ config */

typedef struct {
	/** USM primitives. Required; `crypto` must carry hmac_sha256. */
	snmp_usm_ports_t ports;
	/**
	 * Persisted snmpEngineBoots. RFC 3414 §2.2.2 requires it to increase on
	 * every restart and to be at least 1 once the engine has ever run.
	 */
	uint32_t engine_boots;
	/** Engine id; see @ref snmp_v3_engine_id_build. */
	const uint8_t *engine_id;
	size_t engine_id_len;
	/** msgMaxSize this agent advertises; 0 selects SNMP_PKT_MAX. */
	uint32_t max_msg_size;
} snmp_v3_cfg_t;

/* ----------------------------------------------------------------- context */

/**
 * SNMPv3 context. Caller-owned; all fields private.
 *
 * Carries a full-datagram scratch buffer because a privacy-protected request
 * has to be decrypted somewhere and the received datagram is const. That makes
 * the struct about 1.6 KiB, which is why the glue holds one as a static rather
 * than on a thread stack — and why one context serves one thread. Two threads
 * sharing a context would share the scratch.
 */
typedef struct {
	snmp_v3_cfg_t cfg;
	uint8_t engine_id[SNMP_V3_ENGINEID_MAX];
	size_t engine_id_len;
	snmp_v3_user_t users[SNMP_V3_USERS];
	snmp_v3_stats_t stats;
	uint8_t scratch[SNMP_PKT_MAX];
	uint32_t salt_counter;
	int32_t notify_msgid;
	bool ready;
} snmp_v3_ctx_t;

/**
 * Initialise a context.
 *
 * @retval 0        Ready.
 * @retval -EINVAL  NULL argument, no crypto port, or a bad engine id.
 * @retval -ENOTSUP The crypto port lacks hmac_sha256 or sha256.
 */
int snmp_v3_init(snmp_v3_ctx_t *c, const snmp_v3_cfg_t *cfg);

/**
 * Replace the engine id.
 *
 * **Clears every configured user.** Localised keys are bound to the engine id
 * (RFC 3414 §2.6), so keeping them across a change would leave the agent unable
 * to authenticate anybody while looking configured. Re-add the users after.
 */
int snmp_v3_set_engine_id(snmp_v3_ctx_t *c, const uint8_t *id, size_t len);

/** The engine id in use. @p out must hold SNMP_V3_ENGINEID_MAX. */
int snmp_v3_get_engine_id(const snmp_v3_ctx_t *c, uint8_t *out, size_t *out_len);

/** Set snmpEngineBoots (a persisted value read back at start-up). */
int snmp_v3_set_boots(snmp_v3_ctx_t *c, uint32_t boots);

/** The snmpEngineBoots in use. */
uint32_t snmp_v3_get_boots(const snmp_v3_ctx_t *c);

/**
 * Add or replace a user.
 *
 * @param auth_proto     snmp_auth_proto_t.
 * @param auth_secret    Passphrase (NUL-terminated) when @p auth_localized is
 *                       false, else exactly @ref snmp_usm_key_len octets of
 *                       already-localised key.
 * @param auth_len       Length of @p auth_secret in octets.
 * @param auth_localized Which of the two @p auth_secret is.
 * @param priv_*         The same for privacy. A privacy protocol without an
 *                       auth protocol is refused: RFC 3414 has no such security
 *                       level, and accepting it would give an unauthenticated
 *                       peer an encryption oracle.
 *
 * Storing localised keys is the recommended provisioning route — it keeps the
 * passphrase off the device entirely, and it is the only route available when
 * the crypto port carries no streaming SHA-256. cfg stores whichever the
 * operator provided; `sec.snmpv3.uN.localized` says which.
 *
 * @retval 0        Stored.
 * @retval -EINVAL  Bad argument, unknown protocol, or a bad secret length.
 * @retval -EPERM   No engine id is set, so a passphrase cannot be localised.
 * @retval -ENOSPC  All SNMP_V3_USERS slots hold other users.
 * @retval -ENOTSUP A passphrase was given but the port cannot expand one.
 * @retval -EIO     A crypto port call failed.
 */
int snmp_v3_user_set(snmp_v3_ctx_t *c, const char *name, uint8_t auth_proto,
		     const void *auth_secret, size_t auth_len,
		     bool auth_localized, uint8_t priv_proto,
		     const void *priv_secret, size_t priv_len,
		     bool priv_localized);

/** Remove a user. @retval -ENOENT when there is no such user. */
int snmp_v3_user_clear(snmp_v3_ctx_t *c, const char *name);

/** Remove every user. */
void snmp_v3_users_clear(snmp_v3_ctx_t *c);

/** Users currently configured. */
size_t snmp_v3_user_count(const snmp_v3_ctx_t *c);

/**
 * The security level a user's configuration demands.
 *
 * A request weaker than this is refused with usmStatsUnsupportedSecLevels: a
 * read-only agent that answers a noAuthNoPriv GET for a user provisioned with
 * an auth key has no authentication at all.
 */
int snmp_v3_user_level(const snmp_v3_ctx_t *c, const char *name,
		       uint8_t *out_level);

/* ---------------------------------------------------------------- handling */

/**
 * Handle one received SNMPv3 datagram.
 *
 * @param agent         The v2c/PDU agent — supplies the OID catalogue, the
 *                      getter and the response counters.
 * @param engine_time_s snmpEngineTime: seconds since this engine booted.
 * @param uptime_cs     sysUpTime in hundredths, for the PDU's own bookkeeping.
 * @param rsp           Response buffer.
 * @param rsp_len       Receives the length to send when the return value is 0.
 *
 * A Report PDU is a normal, successful outcome: a manager discovers the engine
 * id and synchronises its clock by provoking exactly that. So a return of 0 with
 * a Report in @p rsp is not an error, and the caller sends it like any response.
 *
 * @retval 0         Send @p rsp_len octets (a Response *or* a Report).
 * @retval -EBADMSG  Malformed beyond the point where a Report could be built.
 * @retval -EPROTO   Not an SNMPv3 message, or not msgSecurityModel USM.
 * @retval -EACCES   Authentication failed and the request was not reportable.
 * @retval -EINVAL   NULL argument or an uninitialised context.
 * @retval -ENOSPC   @p rsp_cap cannot hold even a Report.
 */
int snmp_v3_handle(snmp_ctx_t *agent, snmp_v3_ctx_t *c, const uint8_t *req,
		   size_t req_len, uint32_t engine_time_s, uint32_t uptime_cs,
		   uint8_t *rsp, size_t rsp_cap, size_t *rsp_len);

/**
 * Route a datagram to the v2c or the v3 handler by its version field.
 *
 * @param c  May be NULL, which makes every v3 datagram a counted drop.
 *
 * @retval 0        Send @p rsp_len octets.
 * @retval -EPROTO  A version neither handler serves (or v3 with @p c NULL).
 * @retval other    Whatever the selected handler returned.
 */
int snmp_dispatch(snmp_ctx_t *agent, snmp_v3_ctx_t *c, const uint8_t *req,
		  size_t req_len, uint32_t engine_time_s, uint32_t uptime_cs,
		  uint8_t *rsp, size_t rsp_cap, size_t *rsp_len);

/* ------------------------------------------------------- traps and informs */

/**
 * Build an SNMPv3 notification.
 *
 * @param user       USM user to send as; must exist and, for @p level above
 *                   noAuthNoPriv, must carry the matching keys.
 * @param level      snmp_sec_level_t. authNoPriv or authPriv per spec §9.5;
 *                   noAuthNoPriv is accepted but carries no protection.
 * @param inform     true builds an InformRequest (RFC 3416 tag 0xA6, which the
 *                   receiver acknowledges with a Response); false builds an
 *                   SNMPv2-Trap (0xA7).
 * @param out_msgid  Optional; the msgID and request-id used, so a caller
 *                   awaiting an inform acknowledgement can match it.
 *
 * Notifications are sent with *our* engine id, boots and time: RFC 3414 §3.1
 * makes the notification originator the authoritative engine, which is why a
 * receiver never has to discover anything to accept one.
 *
 * @retval 0        Built.
 * @retval -EINVAL  Bad argument or an unknown notification.
 * @retval -ENOENT  No such user.
 * @retval -EPERM   The user cannot satisfy @p level.
 * @retval -ENOSPC  @p cap too small.
 * @retval -EIO     A crypto port call failed.
 */
int snmp_v3_make_notification(snmp_ctx_t *agent, snmp_v3_ctx_t *c,
			      const char *user, uint8_t level, snmp_trap_t t,
			      uint32_t engine_time_s, uint32_t uptime_cs,
			      const snmp_bind_t *binds, size_t n_binds,
			      bool inform, uint8_t *out, size_t cap,
			      size_t *out_len, int32_t *out_msgid);

/* ------------------------------------------------------------------- stats */

/** Snapshot the USM counters. */
int snmp_v3_stats_get(const snmp_v3_ctx_t *c, snmp_v3_stats_t *out);

/** Zero the USM counters. */
int snmp_v3_stats_reset(snmp_v3_ctx_t *c);

/**
 * usmStats OIDs, exposed so the MIB-file and the Zabbix template can be checked
 * against the same constants the Report builder uses.
 *
 * @param which  1..6 in the order of @ref snmp_v3_stats_t's first six members.
 * @param arcs   Receives the OID; must hold SNMP_OID_MAX_LEN arcs.
 *
 * @retval >0       Arcs written.
 * @retval -EINVAL  NULL @p arcs or @p which out of range.
 */
int snmp_usm_stats_oid(unsigned int which, uint32_t *arcs);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_SNMP_SNMP_V3_H_ */
