/*
 * STS1000 "Meridian" — core/web: management-plane authentication (spec §9.4).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation; the caller owns auth_web_ctx_t.
 *
 * What this owns
 * --------------
 *   - opaque session tokens (port_crypto rand -> hex), idle AND absolute
 *     timeouts, a bounded session table;
 *   - the role model admin / operator / viewer, ordered so a route's
 *     requirement is a `>=` test;
 *   - a per-session CSRF token, required on every mutating request;
 *   - the same escalating brute-force throttle core/mcp uses, per user, with
 *     the counters surviving a connection drop (an attacker must not be able
 *     to reset the throttle by reconnecting);
 *   - constant-time verification of the stored credential.
 *
 * Credential format — deliberately IDENTICAL to core/mcp
 * -----------------------------------------------------
 * The stored blob is exactly what core/mcp stores, in exactly the same cfg key
 * (`sec.admin.pw`, CFG_F_SECRET | CFG_F_NOEXPORT, 48 bytes):
 *
 *     salt[16] || tag[32]        tag = KDF(salt, password)
 *
 * The default KDF is HMAC-SHA-256(key = salt, msg = password), matching
 * mcp_auth_make_blob() byte for byte. That is NOT password stretching, and
 * spec §9.4 asks for Argon2id.
 *
 * The upgrade path is plumbed rather than guessed at: @ref auth_kdf_t is a
 * swappable derivation whose output is the same 32-byte tag in the same
 * 48-byte envelope, so an Argon2id implementation drops in without touching
 * this module, the schema, or the blob layout. What it CANNOT do is change one
 * credential store without the other: the web plane and the MCP console read
 * the same key, so switching the KDF is a coordinated change to both (and needs
 * either a schema flag byte or a re-provisioning step, since the envelope has
 * no room to record which KDF produced the tag). That is written down here
 * because silently diverging the two stores would lock the operator out of one
 * channel while leaving the other open — a worse outcome than keeping the
 * weaker KDF until both move together.
 *
 * TODO(argon2id): land an Argon2id auth_kdf_t (m=64 MiB is impossible on this
 * part; m=16..32 KiB, t=3, p=1 is the realistic envelope), add a cfg key
 * recording the KDF id, and move core/mcp onto the same table in the same
 * change. Tracked as a firmware open item.
 *
 * Threading: not internally locked. The Zephyr glue serialises access from the
 * web worker threads with one mutex, exactly as core/cfg is serialised.
 *
 * Portable errno only
 * -------------------
 * Every code returned from this module exists in picolibc, which is the target
 * libc. That rules out the Linux-only key-management family (ENOKEY,
 * EKEYEXPIRED, ...) even where it would read better: a host build would accept
 * it and the target link would not.
 */

#ifndef STS1000_CORE_WEB_AUTH_WEB_H_
#define STS1000_CORE_WEB_AUTH_WEB_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cfg/cfg.h"
#include "logring/logring.h"
#include "port/port_crypto.h"
#include "web/web.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ limits */

/** Concurrent authenticated sessions. */
#ifndef AUTH_WEB_SESSIONS
#define AUTH_WEB_SESSIONS 4U
#endif

/** User accounts the table holds. */
#ifndef AUTH_WEB_USERS
#define AUTH_WEB_USERS 4U
#endif

/** Random bytes behind a session token (hex-encoded, so 2x on the wire). */
#define AUTH_WEB_TOKEN_BYTES 24U
/** Session-token string length, excluding the NUL. */
#define AUTH_WEB_TOKEN_LEN (AUTH_WEB_TOKEN_BYTES * 2U)

/** Random bytes behind a CSRF token. */
#define AUTH_WEB_CSRF_BYTES 16U
/** CSRF-token string length, excluding the NUL. */
#define AUTH_WEB_CSRF_LEN (AUTH_WEB_CSRF_BYTES * 2U)

/** Longest user name, excluding the NUL. */
#define AUTH_WEB_NAME_MAX 24U
/** Longest password accepted. Matches MCP_PW_MAX. */
#define AUTH_WEB_PW_MAX 64U
/** Shortest password accepted for a *new* credential. */
#define AUTH_WEB_PW_MIN 8U

/** Credential blob layout, identical to core/mcp. */
#define AUTH_WEB_SALT_LEN 16U
#define AUTH_WEB_TAG_LEN  32U
#define AUTH_WEB_BLOB_LEN (AUTH_WEB_SALT_LEN + AUTH_WEB_TAG_LEN)

/** Default idle timeout, seconds, when cfg has no `sec.session.s`. */
#define AUTH_WEB_IDLE_S_DEFAULT 600U
/** Default absolute session lifetime, seconds. */
#define AUTH_WEB_ABSOLUTE_S_DEFAULT 43200U

/*
 * Brute-force throttle. Same shape and same numbers as core/mcp (mcp.h): the
 * first AUTH_WEB_FREE_TRIES mismatches answer immediately, each further one arms
 * a doubling backoff, and AUTH_WEB_LOCK_TRIES arms the long lockout. A correct
 * password clears it; a reboot does too, which is acceptable because a reboot is
 * not something a remote attacker can trigger before authenticating.
 */
#ifndef AUTH_WEB_FREE_TRIES
#define AUTH_WEB_FREE_TRIES 3U
#endif
#ifndef AUTH_WEB_LOCK_TRIES
#define AUTH_WEB_LOCK_TRIES 8U
#endif
#ifndef AUTH_WEB_THROTTLE_MS
#define AUTH_WEB_THROTTLE_MS 2000U
#endif
#ifndef AUTH_WEB_THROTTLE_MAX_MS
#define AUTH_WEB_THROTTLE_MAX_MS 30000U
#endif
#ifndef AUTH_WEB_LOCKOUT_MS
#define AUTH_WEB_LOCKOUT_MS 60000U
#endif

/* --------------------------------------------------------------------- KDF */

/** KDF identifiers. Only HMAC-SHA-256 is implemented; see the header comment. */
#define AUTH_KDF_HMAC_SHA256 1U
#define AUTH_KDF_ARGON2ID    2U

/**
 * Password-derivation hook.
 *
 * @param out_len  Always AUTH_WEB_TAG_LEN; passed so an implementation can
 *                 assert rather than assume.
 * @retval 0   @p out holds the tag.
 * @retval <0  errno-style failure; the login is refused, not defaulted.
 */
typedef struct {
	uint8_t     id;
	const char *name;
	int (*derive)(void *user, const uint8_t *salt, size_t salt_len,
		      const uint8_t *pw, size_t pw_len, uint8_t *out,
		      size_t out_len);
	void *user;
} auth_kdf_t;

/**
 * The default KDF: HMAC-SHA-256(salt, password) over @p crypto.
 *
 * @p out is filled in and points at @p crypto, which must outlive it.
 * @retval 0        Ready.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOTSUP @p crypto has no hmac_sha256.
 */
int auth_kdf_hmac_sha256(auth_kdf_t *out, const port_crypto_t *crypto);

/* ------------------------------------------------------------------- state */

/** One account. */
typedef struct {
	char     name[AUTH_WEB_NAME_MAX + 1U];
	uint8_t  role;                       /**< web_role_t */
	uint8_t  blob[AUTH_WEB_BLOB_LEN];
	bool     has_blob;
	bool     in_use;
	/**
	 * cfg key holding the credential, or 0 for a RAM-only account.
	 *
	 * Only the admin account has a schema key today (`sec.admin.pw`).
	 * Additional roles therefore live in RAM until the schema grows keys for
	 * them — cfg_schema.h is not this module's to extend. Reported honestly
	 * by @ref auth_web_user_persistent().
	 */
	uint16_t cfg_key;

	/* throttle, per account */
	uint32_t fails;
	uint64_t lock_until_ms;
} auth_web_user_t;

/** One session. */
typedef struct {
	char     token[AUTH_WEB_TOKEN_LEN + 1U];
	char     csrf[AUTH_WEB_CSRF_LEN + 1U];
	uint8_t  user;      /**< index into auth_web_ctx_t::users */
	uint8_t  role;      /**< snapshot of the role at login */
	uint64_t created_ms;
	uint64_t last_ms;
	uint32_t requests;
	bool     active;
} auth_web_sess_t;

/** Counters, for telemetry and tests. */
typedef struct {
	uint32_t logins_ok;
	uint32_t logins_bad_pw;
	uint32_t logins_bad_user;
	uint32_t logins_throttled;
	uint32_t logins_no_slot;
	uint32_t logouts;
	uint32_t expired_idle;
	uint32_t expired_absolute;
	uint32_t csrf_rejected;
	uint32_t token_rejected;
} auth_web_stats_t;

/** Registry. Caller-owned; zeroed and populated by auth_web_init(). */
typedef struct {
	const port_crypto_t *crypto;
	const auth_kdf_t    *kdf;
	cfg_ctx_t           *cfg;
	logr_t              *log;

	uint32_t idle_s;
	uint32_t absolute_s;

	auth_web_user_t users[AUTH_WEB_USERS];
	auth_web_sess_t sess[AUTH_WEB_SESSIONS];
	auth_web_stats_t st;
} auth_web_ctx_t;

/* --------------------------------------------------------------- lifecycle */

/**
 * Initialise @p c.
 *
 * @param crypto  Required: supplies rand (tokens) and the default KDF.
 * @param kdf     Required; use auth_kdf_hmac_sha256() for the default.
 * @param cfg     Optional. When present, the admin account is bound to
 *                CFG_ID_SEC_ADMIN_PW and the idle timeout follows
 *                `sec.session.s`; when absent, everything is RAM-only.
 * @param log     Optional audit sink.
 *
 * @retval 0        Ready.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOTSUP @p crypto lacks rand.
 */
int auth_web_init(auth_web_ctx_t *c, const port_crypto_t *crypto,
		  const auth_kdf_t *kdf, cfg_ctx_t *cfg, logr_t *log);

/**
 * Create or update an account.
 *
 * @param cfg_key  cfg key holding the credential, or 0 for RAM-only.
 * @retval >=0      Account index.
 * @retval -EINVAL  Bad argument (bad name, unknown role).
 * @retval -ENOSPC  Table full.
 */
int auth_web_user_add(auth_web_ctx_t *c, const char *name, uint8_t role,
		      uint16_t cfg_key);

/** Account index for @p name, or -ENOENT. */
int auth_web_user_find(const auth_web_ctx_t *c, const char *name, size_t n);

/** Account by index, or NULL. */
const auth_web_user_t *auth_web_user_at(const auth_web_ctx_t *c, size_t idx);

/** True when @p idx's credential survives a reboot. */
bool auth_web_user_persistent(const auth_web_ctx_t *c, size_t idx);

/**
 * Re-read every cfg-backed credential into the RAM table.
 *
 * Call after init and after any config commit that could have rewritten a
 * credential. An account whose cfg value is absent or the wrong length is left
 * without a credential, which makes every login for it fail closed.
 *
 * @return Number of accounts that now hold a credential.
 */
int auth_web_reload(auth_web_ctx_t *c);

/**
 * Derive and store a new credential for account @p idx.
 *
 * A fresh 16-byte salt is drawn from @p crypto->rand. When the account carries a
 * cfg key the blob is *staged* through cfg_set() — never committed here, so the
 * caller keeps the validate-then-commit discipline and can revert.
 *
 * @retval 0        Stored (and staged, if persistent).
 * @retval -EINVAL  Bad argument, or a password outside
 *                  [AUTH_WEB_PW_MIN, AUTH_WEB_PW_MAX].
 * @retval -ENOENT  No such account.
 * @retval -EIO     A crypto or cfg call failed.
 */
int auth_web_set_password(auth_web_ctx_t *c, size_t idx, const uint8_t *pw,
			  size_t pw_len);

/** Build a credential blob without storing it (provisioning / tests). */
int auth_web_make_blob(const auth_web_ctx_t *c, const uint8_t *salt,
		       const uint8_t *pw, size_t pw_len, uint8_t *out,
		       size_t cap);

/* -------------------------------------------------------------- sessions */

/** Result of a successful login. */
typedef struct {
	const char *token;
	const char *csrf;
	uint8_t     role;
	uint32_t    idle_s;
	uint32_t    absolute_s;
} auth_web_grant_t;

/**
 * Verify a credential and open a session.
 *
 * @retval 0        Granted; @p out describes the session (pointers alias @p c).
 * @retval -EINVAL  Bad argument.
 * @retval -EACCES  Unknown user or wrong password.
 * @retval -EBUSY   Throttled — do not retry until the window expires. The
 *                  password is NOT tested in this case, so the answer carries no
 *                  information about it.
 * @retval -ENOSPC  No free session slot.
 * @retval -ENOENT  The account has no credential provisioned. (Not -ENOKEY:
 *                  picolibc, the target libc, does not define it — see the
 *                  portable-errno note at the top of this header.)
 * @retval -EIO     The KDF failed.
 */
int auth_web_login(auth_web_ctx_t *c, const char *user, size_t user_len,
		   const uint8_t *pw, size_t pw_len, uint64_t now_ms,
		   auth_web_grant_t *out);

/**
 * Resolve a bearer/cookie token to a live session and refresh its idle timer.
 *
 * @retval 0        Valid; @p out_sess (optional) receives the slot index.
 * @retval -EINVAL  Bad argument.
 * @retval -EACCES  Unknown token.
 * @retval -ETIME   The token named a session that has just expired (it is
 *                  closed by this call).
 */
int auth_web_validate(auth_web_ctx_t *c, const char *token, uint64_t now_ms,
		      size_t *out_sess);

/** Session by slot index, or NULL. */
const auth_web_sess_t *auth_web_sess_at(const auth_web_ctx_t *c, size_t idx);

/**
 * Constant-time CSRF check for session @p idx.
 *
 * @retval 0        Match.
 * @retval -EINVAL  Bad argument or dead session.
 * @retval -EACCES  Missing or wrong token.
 */
int auth_web_check_csrf(auth_web_ctx_t *c, size_t idx, const char *token,
			size_t token_len);

/** Close session @p idx. Returns 0, or -ENOENT when it was not open. */
int auth_web_logout(auth_web_ctx_t *c, size_t idx);

/** Close every session (password change, factory reset, TLS key rotation). */
void auth_web_logout_all(auth_web_ctx_t *c);

/** Expire idle/aged sessions. Returns the number closed. */
int auth_web_tick(auth_web_ctx_t *c, uint64_t now_ms);

/** Live session count. */
uint32_t auth_web_session_count(const auth_web_ctx_t *c);

/** Counters snapshot. */
const auth_web_stats_t *auth_web_stats(const auth_web_ctx_t *c);

/**
 * Milliseconds until account @p idx may attempt a login again.
 *
 * 0 when it may attempt now. Used to populate `Retry-After`.
 */
uint32_t auth_web_retry_after_ms(const auth_web_ctx_t *c, size_t idx,
				 uint64_t now_ms);

/** True when config demands authentication (cfg `sec.auth.req`, default on). */
bool auth_web_required(const auth_web_ctx_t *c);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_WEB_AUTH_WEB_H_ */
