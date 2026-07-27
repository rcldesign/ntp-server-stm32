/*
 * STS1000 "Meridian" — core/auth: management AAA (spec §9.4).
 *
 * Platform-neutral C11, no dynamic allocation, all state in a caller-owned
 * context (ARCHITECTURE.md §4, dependency edge `auth→util`). One entry point,
 * @ref auth_check, answers "who is this and what may they do" for every
 * management surface — the web server, the MCP console channel and the shell —
 * so the lockout counters, the cache and the role mapping are shared rather
 * than reimplemented three times.
 *
 * ---------------------------------------------------------------------------
 * The backend chain
 * ---------------------------------------------------------------------------
 *
 * `auth_cfg_t::order` lists backends first-to-last (default: local only). Each
 * is tried in turn and one of three things happens:
 *
 *   accept     the chain stops and the role is returned
 *   reject     the chain stops — a definite "no" from an authority is final,
 *              because falling through to the next backend on a reject is how
 *              a mis-ordered chain turns into a privilege-escalation path
 *   unavailable the next backend is tried (network down, no config, no secret)
 *
 * That is the commercial-AAA convention and it is the safe one: only an
 * *absent* authority is skipped. If every backend is unavailable the result is
 * -EHOSTUNREACH, which the caller must treat as a denial — never as an
 * allow-on-failure.
 *
 * ---------------------------------------------------------------------------
 * Where the network lives
 * ---------------------------------------------------------------------------
 *
 * RADIUS, TACACS+ and LDAP packet construction and parsing are this module's
 * business (radius.h, tacacs.h, ldap.h); sockets, timeouts and TLS are not.
 * `auth_cfg_t::remote` is the one callback that crosses that line, and the
 * Zephyr glue (`src/zephyr/net/sts_aaa.c`) implements it. A host test
 * implements it with a table, which is how the chain, the cache and the lockout
 * are covered without a server.
 *
 * ---------------------------------------------------------------------------
 * MD5 lives here, and only because two protocols require it
 * ---------------------------------------------------------------------------
 *
 * RFC 2865 §5.2 defines User-Password hiding in terms of MD5, RFC 2865 §3
 * defines the Response Authenticator in terms of MD5, and RFC 8907 §4.5 defines
 * the TACACS+ body obfuscation in terms of MD5. There is no negotiable
 * alternative in either protocol: an implementation that used something else
 * would not interoperate. So MD5 is implemented in this module rather than
 * added to port_crypto.h — putting it in the port would advertise it as a
 * primitive core code may build on, which it is not. It is a wire-format
 * detail of two legacy protocols and must never be used for anything else.
 */

#ifndef STS1000_CORE_AUTH_AUTH_H_
#define STS1000_CORE_AUTH_AUTH_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "port/port_crypto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ limits */

/** Longest username handled, excluding the NUL (RFC 2865 caps at 253). */
#define AUTH_USER_MAX 63U

/** Longest password/secret handled, excluding the NUL. */
#define AUTH_SECRET_MAX 64U

/** Longest shared secret / bind password held by a backend config. */
#define AUTH_SHARED_MAX 64U

/** Positive-result cache entries. */
#ifndef AUTH_CACHE_SLOTS
#define AUTH_CACHE_SLOTS 8U
#endif

/** Distinct usernames the lockout table tracks concurrently. */
#ifndef AUTH_LOCK_SLOTS
#define AUTH_LOCK_SLOTS 8U
#endif

/* ------------------------------------------------------- roles & backends */

/**
 * Authorisation roles (spec §9.4), ordered so a numeric comparison is a
 * privilege comparison: a surface needing operator rights accepts
 * `role >= AUTH_ROLE_OPERATOR`.
 */
typedef enum {
	AUTH_ROLE_NONE = 0,
	AUTH_ROLE_VIEWER = 1,
	AUTH_ROLE_OPERATOR = 2,
	AUTH_ROLE_ADMIN = 3,
} auth_role_t;

/** Backends, in the order they appear in a default chain. */
typedef enum {
	AUTH_BE_LOCAL = 0,
	AUTH_BE_RADIUS = 1,
	AUTH_BE_TACACS = 2,
	AUTH_BE_LDAP = 3,
	AUTH_BE_COUNT
} auth_backend_t;

/** Name of @p r ("admin"/"operator"/"viewer"/"none"). Never NULL. */
const char *auth_role_name(uint8_t r);

/**
 * Parse a role name, case-insensitively.
 *
 * Accepts the three role names plus the common vendor synonyms an AAA server
 * is likely to return in a Filter-Id or a TACACS+ argument: "rw"/"readwrite"
 * for operator, "ro"/"readonly"/"monitor" for viewer, "root"/"superuser" for
 * admin.
 *
 * @retval >0       auth_role_t.
 * @retval -EINVAL  NULL or unrecognised.
 */
int auth_role_parse(const char *s);

/** Name of @p b ("local"/"radius"/"tacacs"/"ldap"). Never NULL. */
const char *auth_backend_name(uint8_t b);

/**
 * Parse a comma- or space-separated backend chain, e.g. "local,radius".
 *
 * Unknown names and duplicates are skipped rather than failing the whole list:
 * a chain is operator-typed configuration, and dropping one bad word is more
 * useful than refusing to authenticate anybody.
 *
 * @param out  Receives the chain.
 * @param cap  Capacity of @p out, normally AUTH_BE_COUNT.
 *
 * @retval >=0      Entries written (0 for an empty or wholly unparsable list).
 * @retval -EINVAL  NULL argument or zero capacity.
 */
int auth_order_parse(const char *s, uint8_t *out, size_t cap);

/* ------------------------------------------------------------- MD5 (legacy) */

#define AUTH_MD5_LEN 16U

/** Incremental MD5 state. See the file comment for why this exists. */
typedef struct {
	uint32_t h[4];
	uint64_t bits;
	uint8_t buf[64];
	size_t n;
} auth_md5_t;

void auth_md5_init(auth_md5_t *s);
void auth_md5_update(auth_md5_t *s, const void *data, size_t len);
void auth_md5_final(auth_md5_t *s, uint8_t out[AUTH_MD5_LEN]);

/** One-shot MD5. */
void auth_md5(const void *data, size_t len, uint8_t out[AUTH_MD5_LEN]);

/** HMAC-MD5 (RFC 2104), for the RADIUS Message-Authenticator (RFC 3579 §3.2). */
void auth_hmac_md5(const uint8_t *key, size_t key_len, const uint8_t *msg,
		   size_t msg_len, uint8_t out[AUTH_MD5_LEN]);

/**
 * Constant-time equality over @p n octets.
 *
 * Exported because every MAC comparison in this module and in core/snmp's USM
 * needs it and a `memcmp` there is a timing oracle.
 */
bool auth_ct_eq(const uint8_t *a, const uint8_t *b, size_t n);

/* ------------------------------------------------- the local credential */

/** Random salt length of the stored local credential blob. */
#define AUTH_PW_SALT_LEN 16U
/** MAC length of the stored local credential blob. */
#define AUTH_PW_MAC_LEN 32U
/**
 * Stored local credential: salt ‖ HMAC-SHA-256(key = salt, msg = password).
 *
 * This is byte-for-byte the layout core/mcp's `mcp_auth_make_blob()` writes and
 * cfg key `sec.admin.pw` (a 48-octet blob) stores, so the console and this
 * module verify the same credential. The two definitions must agree;
 * tests/host/test_auth.c pins the layout by building a blob from the primitives
 * and requiring @ref auth_local_verify to accept it.
 *
 * Spec §9.4 asks for Argon2id. This is the salted-HMAC scheme the console
 * already ships, kept identical so nothing regresses; moving both to a memory-
 * hard KDF is a single change to this constant plus a cfg migration.
 */
#define AUTH_PW_BLOB_LEN (AUTH_PW_SALT_LEN + AUTH_PW_MAC_LEN)

/**
 * Verify @p secret against a stored credential blob.
 *
 * @retval 0        Match.
 * @retval -EACCES  Mismatch.
 * @retval -ENOENT  No credential provisioned (NULL or wrong-length blob).
 * @retval -EINVAL  NULL @p crypto or @p secret, or a secret over AUTH_SECRET_MAX.
 * @retval -ENOTSUP @p crypto has no hmac_sha256.
 * @retval -EIO     The crypto port failed.
 */
int auth_local_verify(const port_crypto_t *crypto, const uint8_t *blob,
		      size_t blob_len, const char *secret);

/* -------------------------------------------------------------- remote hook */

/**
 * Ask one remote backend about a credential. Implemented by the glue.
 *
 * @param backend    auth_backend_t (never AUTH_BE_LOCAL).
 * @param out_role   On acceptance, the auth_role_t the backend's attributes
 *                   mapped to. A backend that accepts but says nothing about
 *                   authorisation must still write a role; AUTH_ROLE_VIEWER is
 *                   the safe default and what radius.c/tacacs.c/ldap.c use.
 *
 * @retval 0                 Accept.
 * @retval -EACCES           Reject — an authoritative "no".
 * @retval -EHOSTUNREACH     Not configured or unreachable; try the next backend.
 * @retval -ETIMEDOUT        No answer in time; try the next backend.
 * @retval -ENOTSUP          Backend not built in; try the next backend.
 * @retval any other error   Treated as unavailable.
 */
typedef int (*auth_remote_fn)(void *ctx, uint8_t backend, const char *user,
			      const char *secret, uint8_t *out_role);

/* ------------------------------------------------------------------ config */

typedef struct {
	/**
	 * Crypto port. Required: the local credential check needs
	 * HMAC-SHA-256, and the cache key is derived with it. `rand` is needed
	 * for the cache salt — without it caching stays off rather than
	 * falling back to a predictable salt.
	 */
	const port_crypto_t *crypto;

	/** Remote backend hook, or NULL for a local-only box. */
	auth_remote_fn remote;
	void *remote_ctx;

	/** Backend chain. An empty chain is treated as {AUTH_BE_LOCAL}. */
	uint8_t order[AUTH_BE_COUNT];
	uint8_t n_order;

	/**
	 * Stored local credential (cfg `sec.admin.pw`). Borrowed, not copied,
	 * and must outlive the context. NULL means no local credential, which
	 * makes the local backend report "unavailable" rather than reject —
	 * so a box with only remote backends configured still works.
	 */
	const uint8_t *local_blob;
	size_t local_blob_len;

	/** Role granted by the local credential. 0 selects AUTH_ROLE_ADMIN. */
	uint8_t local_role;

	/** Positive-cache lifetime, seconds. 0 disables the cache. */
	uint16_t cache_ttl_s;

	/** Consecutive failures before lockout. 0 disables lockout. */
	uint8_t lockout_fails;

	/** Lockout duration, seconds. 0 with a non-zero count means 60. */
	uint16_t lockout_s;
} auth_cfg_t;

/* ------------------------------------------------------------------- stats */

typedef struct {
	uint32_t checks;
	uint32_t accepts;
	uint32_t rejects;
	uint32_t locked;      /**< refused by the lockout window            */
	uint32_t unavailable; /**< every backend in the chain was unavailable */
	uint32_t cache_hits;
	uint32_t cache_fills;
	uint32_t by_backend[AUTH_BE_COUNT]; /**< accepts credited per backend */
} auth_stats_t;

/* ----------------------------------------------------------------- context */

/** Cache entry. Private. */
typedef struct {
	uint8_t key[16];
	uint64_t expiry_ms;
	uint8_t role;
	uint8_t backend;
	bool valid;
} auth_cache_ent_t;

/** Lockout entry. Private. */
typedef struct {
	char user[AUTH_USER_MAX + 1U];
	uint64_t lock_until_ms;
	uint64_t last_ms;
	uint16_t fails;
	bool valid;
} auth_lock_ent_t;

/** AAA context. Caller-owned; all fields private. */
typedef struct {
	auth_cfg_t cfg;
	auth_stats_t stats;
	auth_cache_ent_t cache[AUTH_CACHE_SLOTS];
	auth_lock_ent_t locks[AUTH_LOCK_SLOTS];
	uint8_t cache_salt[16];
	bool cache_ready;
	bool ready;
} auth_ctx_t;

/** Result of one @ref auth_check. */
typedef struct {
	uint8_t role;    /**< auth_role_t; AUTH_ROLE_NONE unless accepted   */
	uint8_t backend; /**< auth_backend_t that decided                   */
	bool cached;     /**< answered from the positive cache              */
	uint32_t retry_after_s; /**< seconds left on a lockout, else 0      */
} auth_out_t;

/**
 * Initialise a context.
 *
 * @retval 0        Ready.
 * @retval -EINVAL  NULL argument, or a config with no crypto port.
 * @retval -ENOTSUP The crypto port has no hmac_sha256.
 */
int auth_init(auth_ctx_t *c, const auth_cfg_t *cfg);

/**
 * Replace the configuration, keeping the lockout table.
 *
 * A cfg commit must not clear a brute-force lockout — that would make
 * "change any setting" a lockout bypass. The positive cache *is* dropped,
 * because a changed chain or credential invalidates it.
 */
int auth_reconfigure(auth_ctx_t *c, const auth_cfg_t *cfg);

/**
 * Authenticate @p user with @p secret and return the authorised role.
 *
 * @param mono_ms  Monotonic milliseconds; the caller owns the clock so the
 *                 cache and lockout windows are testable.
 * @param out      Optional result detail.
 *
 * @retval 0               Accepted; @p out->role is the authorised role.
 * @retval -EACCES         Rejected by an authority.
 * @retval -EBUSY          Locked out; @p out->retry_after_s says for how long.
 * @retval -EHOSTUNREACH   No backend in the chain could answer. **Deny.**
 * @retval -EINVAL         NULL argument, empty user, or an over-long field.
 */
int auth_check(auth_ctx_t *c, const char *user, const char *secret,
	       uint64_t mono_ms, auth_out_t *out);

/**
 * Forget any cached decision.
 *
 * @param user  A specific user, or NULL for every entry.
 */
void auth_cache_flush(auth_ctx_t *c, const char *user);

/** Clear a user's lockout (an explicit administrative unlock). */
int auth_unlock(auth_ctx_t *c, const char *user);

/** True while @p user is inside a lockout window. */
bool auth_is_locked(const auth_ctx_t *c, const char *user, uint64_t mono_ms);

/** Snapshot the counters. */
int auth_stats_get(const auth_ctx_t *c, auth_stats_t *out);

/** Zero the counters. */
int auth_stats_reset(auth_ctx_t *c);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_AUTH_AUTH_H_ */
