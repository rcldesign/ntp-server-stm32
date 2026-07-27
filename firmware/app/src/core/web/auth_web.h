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
 *   - constant-time verification of the stored credential;
 *   - an OPTIONAL remote authority (@ref auth_web_remote_fn) consulted when the
 *     local table cannot accept the credential.
 *
 * The remote authority
 * --------------------
 * RADIUS / TACACS+ / LDAP live behind one function pointer, for the same reason
 * core/auth puts them behind one: this module must stay platform-neutral and
 * socket-free. The Zephyr glue wires the hook to sts_aaa_check(); a host test
 * wires a table. Leaving it NULL reproduces the pre-hook behaviour byte for
 * byte, which is what makes "an unwired hook changes nothing" testable rather
 * than merely asserted.
 *
 * Three rules the hook's callers depend on, all of them security properties:
 *
 *   1. It is consulted ONLY when the local table could not accept — the account
 *      is unknown, holds no credential, or the password did not verify. A local
 *      success never leaves the box.
 *   2. Every non-zero return is a DENIAL. -EHOSTUNREACH in particular means "no
 *      authority could answer" and is never an allow-on-failure. Only -EBUSY is
 *      distinguishable in the answer (it is a lockout, and the caller needs it
 *      to populate Retry-After); everything else collapses into -EACCES so the
 *      reply cannot tell an attacker which half of the credential was right.
 *   3. It is called with the caller's own state on the stack only, so the glue
 *      may block in it. It WILL block: sts_aaa_check() runs a DNS lookup and up
 *      to three network round trips. See the threading note below.
 *
 * Threading: not internally locked. The Zephyr glue serialises access from the
 * web worker threads with one mutex, exactly as core/cfg is serialised. That
 * mutex is held across @ref auth_web_login, and therefore across the remote
 * hook, so the glue's implementation must keep its watchdog participant fed
 * rather than simply blocking (src/zephyr/net/sts_secops.c does).
 *
 * Credential format — deliberately IDENTICAL to core/mcp
 * -----------------------------------------------------
 * The stored blob is exactly what core/mcp stores, in exactly the same cfg key
 * shape (`sec.admin.pw`, CFG_F_SECRET | CFG_F_NOEXPORT, 48 bytes):
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
 * this module, the schema, or the blob layout.
 *
 * All three accounts use that one shape and one set of key flags, so a single
 * verification path serves every role: `sec.admin.pw` (admin),
 * `sec.operator.pw` (operator) and `sec.viewer.pw` (viewer). The KDF swap
 * therefore reaches all of them at once — which is the point, because what it
 * CANNOT do is change one credential store without the other: the web plane and
 * the MCP console read the same keys, so switching the KDF is a coordinated
 * change to both (and needs either a schema flag byte or a re-provisioning step,
 * since the envelope has no room to record which KDF produced the tag). That is
 * written down here because silently diverging the two stores would lock the
 * operator out of one channel while leaving the other open — a worse outcome
 * than keeping the weaker KDF until both move together.
 *
 * TODO(argon2id): land an Argon2id auth_kdf_t (m=64 MiB is impossible on this
 * part; m=16..32 KiB, t=3, p=1 is the realistic envelope), add a cfg key
 * recording the KDF id, and move core/mcp onto the same table in the same
 * change. Tracked as a firmware open item.
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

/* --------------------------------------------------------- remote authority */

/**
 * Ask a remote authority about a credential the local table could not accept.
 *
 * Implemented by the glue over sts_aaa_check() (RADIUS / TACACS+ / LDAP behind
 * core/auth's chain, cache and lockout). Optional: a NULL hook keeps this module
 * behaving exactly as it did before the hook existed.
 *
 * @param name      NUL-terminated account name, at most AUTH_WEB_NAME_MAX bytes.
 * @param secret    NUL-terminated password, at most AUTH_WEB_PW_MAX bytes.
 * @param out_role  Set to a web_role_t on acceptance; the caller has already
 *                  set it to WEB_ROLE_NONE, and clamps whatever comes back.
 *
 * @retval 0               Accepted.
 * @retval -EBUSY          The authority is holding this principal in a lockout.
 *                         Reported distinctly so the caller can send
 *                         Retry-After; it says nothing about the password.
 * @retval <0 (any other)  Denied. -EHOSTUNREACH ("no authority could answer")
 *                         is a denial like every other — never allow-on-failure.
 */
typedef int (*auth_web_remote_fn)(void *user, const char *name,
				  const char *secret, uint8_t *out_role);

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
	 * All three shipped roles now have a schema key of their own —
	 * `sec.admin.pw`, `sec.operator.pw`, `sec.viewer.pw` — so an account
	 * created against one of them survives a reboot. An account created with
	 * 0 is still legal (tests, bring-up images) and is reported honestly by
	 * @ref auth_web_user_persistent().
	 */
	uint16_t cfg_key;

	/* throttle, per account */
	uint32_t fails;
	uint64_t lock_until_ms;
} auth_web_user_t;

/**
 * A session slot index that names no local account.
 *
 * @ref auth_web_sess_t::user carries it when the principal was authenticated by
 * the remote authority and has no entry in the local table. It is deliberately
 * out of range, so auth_web_user_at() answers NULL for it and every existing
 * caller that maps a session to an account already fails safe. Use
 * @ref auth_web_sess_name() to get the principal's name in that case.
 */
#define AUTH_WEB_NO_USER ((uint8_t)AUTH_WEB_USERS)

/** One session. */
typedef struct {
	char     token[AUTH_WEB_TOKEN_LEN + 1U];
	char     csrf[AUTH_WEB_CSRF_LEN + 1U];
	/** Index into auth_web_ctx_t::users, or @ref AUTH_WEB_NO_USER. */
	uint8_t  user;
	uint8_t  role;      /**< snapshot of the role at login */
	/** Authenticated principal, always populated (local or remote). */
	char     name[AUTH_WEB_NAME_MAX + 1U];
	/** True when a remote authority, not the local table, decided. */
	bool     remote;
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
	/** Logins the remote authority accepted. */
	uint32_t logins_remote_ok;
	/** Logins the remote authority refused (every reason, including
	 *  "no authority could answer" — which is a refusal). */
	uint32_t logins_remote_denied;
	/** Local misses/failures that were referred to the remote authority. */
	uint32_t remote_consults;
} auth_web_stats_t;

/** Registry. Caller-owned; zeroed and populated by auth_web_init(). */
typedef struct {
	const port_crypto_t *crypto;
	const auth_kdf_t    *kdf;
	cfg_ctx_t           *cfg;
	logr_t              *log;

	uint32_t idle_s;
	uint32_t absolute_s;

	/** Optional remote authority; see @ref auth_web_set_remote(). */
	auth_web_remote_fn remote;
	void              *remote_user;

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
 * Attach (or detach, with @p fn NULL) the remote authority.
 *
 * Separate from auth_web_init() on purpose: the hook is optional, and every
 * existing caller of auth_web_init() keeps working unchanged and keeps the
 * pre-hook behaviour. Call it once, from the same init path, before the worker
 * threads exist.
 *
 * @retval 0        Stored.
 * @retval -EINVAL  @p c is NULL.
 */
int auth_web_set_remote(auth_web_ctx_t *c, auth_web_remote_fn fn, void *user);

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
 * Order of authorities, and why it is this one:
 *
 *   1. the per-account lockout — a throttled account never reaches a password
 *      test NOR the network, so a brute-force attempt costs the attacker no
 *      RADIUS round trips;
 *   2. the local credential — a local success never leaves the box;
 *   3. the remote authority, if one is wired, for every case the local table
 *      could not accept: unknown account, account with no credential, wrong
 *      password.
 *
 * @retval 0        Granted; @p out describes the session (pointers alias @p c).
 * @retval -EINVAL  Bad argument.
 * @retval -EACCES  Refused. Deliberately ONE answer for an unknown user, a
 *                  wrong password, an unprovisioned account on a commissioned
 *                  box, and every remote refusal including "no authority could
 *                  answer" — so the reply cannot be used to enumerate accounts
 *                  or to learn which half of a credential was right.
 * @retval -EBUSY   Throttled, locally or by the remote authority — do not retry
 *                  until the window expires. The password is NOT tested
 *                  locally in this case, so the answer carries no information
 *                  about it.
 * @retval -ENOSPC  No free session slot.
 * @retval -ENOENT  A KNOWN account was named and NO account on this box holds a
 *                  credential — i.e. the unit has never been commissioned. The
 *                  web layer turns this into a 503 telling the operator to set
 *                  a password over the local UI or the USB console, so it is
 *                  load-bearing for bootstrap and must not be folded away.
 *                  It is a global property of the box, never a per-account one:
 *                  once ANY account is provisioned, an unprovisioned one
 *                  answers -EACCES like everything else, because otherwise the
 *                  reply would report per-account provisioning state to an
 *                  unauthenticated peer. (Not -ENOKEY: picolibc, the target
 *                  libc, does not define it — see the portable-errno note at
 *                  the top of this header.)
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
 * Name of the principal behind session @p idx, or NULL when the slot is dead.
 *
 * Works for a remote principal, which has no entry in the local account table
 * and therefore cannot be named through auth_web_user_at().
 */
const char *auth_web_sess_name(const auth_web_ctx_t *c, size_t idx);

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

/**
 * Secure-erase every secret this context holds.
 *
 * Closes every session (tokens and CSRF tokens are bearer secrets), zeroizes
 * every stored credential blob, and clears the brute-force counters. The
 * account NAMES, roles and cfg-key bindings survive, so the table is still the
 * one the box boots with — only the secrets are gone.
 *
 * Written through a volatile pointer, not memset(), for the reason the whole
 * codebase does it: a plain memset over a buffer nothing reads again is a dead
 * store the compiler may delete.
 *
 * Called from the factory-reset path. Idempotent; safe on a NULL context.
 */
void auth_web_wipe(auth_web_ctx_t *c);

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
