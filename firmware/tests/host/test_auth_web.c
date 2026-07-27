/*
 * STS1000 "Meridian" — core/web auth_web unit tests.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The important ones, since this is the gate on the whole management plane:
 *
 *   - the stored blob is byte-identical to what core/mcp writes, verified
 *     against an independent HMAC-SHA-256 rather than against the module's own
 *     derivation (test_blob_matches_mcp_format);
 *   - the escalating brute-force throttle actually escalates, and a throttled
 *     attempt does NOT test the password;
 *   - idle and absolute session timeouts both fire;
 *   - CSRF verification rejects absent, short and wrong tokens;
 *   - a credential that cfg does not hold leaves the account closed.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "cfg/cfg.h"
#include "logring/logring.h"
#include "web/auth_web.h"
#include "web/web.h"

#include "host_aes.h"
#include "host_sha256.h"
#include "test_support.h"

/* ------------------------------------------------------------------------- */
/* fixture                                                                   */
/* ------------------------------------------------------------------------- */

#define PW "correct-horse"
#define PW_LEN (sizeof(PW) - 1U)

static cfg_ctx_t       g_cfg;
static logr_t          g_log;
static logr_rec_t      g_slots[32];
static auth_web_ctx_t  g_auth;
static auth_kdf_t      g_kdf;
static host_crypto_t   g_hc;
static port_crypto_t   g_port;

/*
 * host_crypto_port() rather than host_sha256.h's host_crypto(): the fixture form
 * carries the fail_*_in injectors, which are the only way to reach this module's
 * -EIO paths (a KDF or CSPRNG that fails must never yield a session).
 */
static void fixture(bool with_cfg)
{
	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, NULL));
	TEST_ASSERT_EQUAL_INT(0, logr_init(&g_log, g_slots, 32U));
	host_crypto_init(&g_hc, 0x1234U);
	g_port = host_crypto_port(&g_hc);
	TEST_ASSERT_EQUAL_INT(0, auth_kdf_hmac_sha256(&g_kdf, &g_port));
	TEST_ASSERT_EQUAL_INT(0, auth_web_init(&g_auth, &g_port, &g_kdf,
					       with_cfg ? &g_cfg : NULL,
					       &g_log));
}

/* Provision the admin account with PW, bound to the cfg credential key. */
static void provision_admin(void)
{
	cfg_commit_res_t res;
	int idx;

	idx = auth_web_user_add(&g_auth, "admin", (uint8_t)WEB_ROLE_ADMIN,
				(uint16_t)CFG_ID_SEC_ADMIN_PW);
	TEST_ASSERT_EQUAL_INT(0, idx);
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_password(&g_auth, 0U,
						       (const uint8_t *)PW,
						       PW_LEN));
	memset(&res, 0, sizeof(res));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, &res));
	TEST_ASSERT_EQUAL_INT(1, auth_web_reload(&g_auth));
}

/* ------------------------------------------------------------------------- */
/* init / kdf                                                               */
/* ------------------------------------------------------------------------- */

static void test_init_args(void)
{
	auth_web_ctx_t c;
	auth_kdf_t k;
	host_crypto_t hc;
	port_crypto_t port;
	port_crypto_t no_rand;
	port_crypto_t no_hmac;

	host_crypto_init(&hc, 7U);
	port = host_crypto_port(&hc);
	TEST_ASSERT_EQUAL_INT(0, auth_kdf_hmac_sha256(&k, &port));
	TEST_ASSERT_EQUAL_UINT(AUTH_KDF_HMAC_SHA256, k.id);
	TEST_ASSERT_EQUAL_STRING("hmac-sha256", k.name);
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_kdf_hmac_sha256(NULL, &port));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_kdf_hmac_sha256(&k, NULL));

	no_hmac = port;
	no_hmac.hmac_sha256 = NULL;
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, auth_kdf_hmac_sha256(&k, &no_hmac));

	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_init(NULL, &port, &k, NULL,
						     NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_init(&c, NULL, &k, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_init(&c, &port, NULL, NULL,
						     NULL));
	{
		auth_kdf_t broken = k;

		broken.derive = NULL;
		TEST_ASSERT_EQUAL_INT(-EINVAL,
				      auth_web_init(&c, &port, &broken, NULL,
						    NULL));
	}
	no_rand = port;
	no_rand.rand = NULL;
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, auth_web_init(&c, &no_rand, &k, NULL,
						      NULL));

	/* With no cfg the module fails closed on the auth-required policy. */
	TEST_ASSERT_EQUAL_INT(0, auth_web_init(&c, &port, &k, NULL, NULL));
	TEST_ASSERT_TRUE(auth_web_required(&c));
	TEST_ASSERT_TRUE(auth_web_required(NULL));
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_session_count(&c));
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_session_count(NULL));
	TEST_ASSERT_NULL(auth_web_stats(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_reload(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_tick(NULL, 0U));
}

/*
 * The credential store must NOT diverge from core/mcp's. mcp_auth_make_blob()
 * writes salt[16] || HMAC-SHA-256(key = salt, msg = password); this recomputes
 * that independently and compares, so a change to either derivation shows up
 * here rather than as a field lock-out.
 */
static void test_blob_matches_mcp_format(void)
{
	uint8_t salt[AUTH_WEB_SALT_LEN];
	uint8_t blob[AUTH_WEB_BLOB_LEN];
	uint8_t want[32];
	size_t i;

	fixture(true);
	for (i = 0U; i < sizeof(salt); i++) {
		salt[i] = (uint8_t)(0x10U + i);
	}
	TEST_ASSERT_EQUAL_INT(0, auth_web_make_blob(&g_auth, salt,
						    (const uint8_t *)PW, PW_LEN,
						    blob, sizeof(blob)));
	TEST_ASSERT_EQUAL_MEMORY(salt, blob, AUTH_WEB_SALT_LEN);
	host_hmac_sha256(salt, sizeof(salt), (const uint8_t *)PW, PW_LEN, want);
	TEST_ASSERT_EQUAL_MEMORY(want, &blob[AUTH_WEB_SALT_LEN], 32U);
	TEST_ASSERT_EQUAL_UINT(48U, AUTH_WEB_BLOB_LEN);

	/* A NULL salt draws a random one and produces a different blob. */
	{
		uint8_t blob2[AUTH_WEB_BLOB_LEN];

		TEST_ASSERT_EQUAL_INT(0,
				      auth_web_make_blob(&g_auth, NULL,
							 (const uint8_t *)PW,
							 PW_LEN, blob2,
							 sizeof(blob2)));
		TEST_ASSERT_TRUE(memcmp(blob, blob2, sizeof(blob)) != 0);
	}

	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_make_blob(NULL, salt,
							  (const uint8_t *)PW,
							  PW_LEN, blob,
							  sizeof(blob)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_make_blob(&g_auth, salt, NULL,
							  PW_LEN, blob,
							  sizeof(blob)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_make_blob(&g_auth, salt,
							  (const uint8_t *)PW,
							  PW_LEN, NULL,
							  sizeof(blob)));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, auth_web_make_blob(&g_auth, salt,
							  (const uint8_t *)PW,
							  PW_LEN, blob, 4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_make_blob(&g_auth, salt,
							  (const uint8_t *)PW,
							  0U, blob,
							  sizeof(blob)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      auth_web_make_blob(&g_auth, salt,
						 (const uint8_t *)PW,
						 AUTH_WEB_PW_MAX + 1U, blob,
						 sizeof(blob)));
}

/* ------------------------------------------------------------------------- */
/* accounts                                                                  */
/* ------------------------------------------------------------------------- */

static void test_users(void)
{
	char name[AUTH_WEB_NAME_MAX + 4U];
	unsigned int i;

	fixture(true);

	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_user_add(NULL, "a",
							 WEB_ROLE_ADMIN, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_user_add(&g_auth, NULL,
							 WEB_ROLE_ADMIN, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_user_add(&g_auth, "",
							 WEB_ROLE_ADMIN, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_user_add(&g_auth, "a",
							 WEB_ROLE_NONE, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_user_add(&g_auth, "a", 99U, 0U));
	memset(name, 'x', sizeof(name) - 1U);
	name[sizeof(name) - 1U] = '\0';
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_user_add(&g_auth, name,
							 WEB_ROLE_ADMIN, 0U));

	TEST_ASSERT_EQUAL_INT(0, auth_web_user_add(&g_auth, "admin",
						   WEB_ROLE_ADMIN,
						   CFG_ID_SEC_ADMIN_PW));
	/* Re-adding the same name updates in place. */
	TEST_ASSERT_EQUAL_INT(0, auth_web_user_add(&g_auth, "admin",
						   WEB_ROLE_ADMIN,
						   CFG_ID_SEC_ADMIN_PW));
	TEST_ASSERT_EQUAL_INT(1, auth_web_user_add(&g_auth, "opr",
						   WEB_ROLE_OPERATOR, 0U));
	TEST_ASSERT_EQUAL_INT(2, auth_web_user_add(&g_auth, "view",
						   WEB_ROLE_VIEWER, 0U));

	TEST_ASSERT_EQUAL_INT(0, auth_web_user_find(&g_auth, "admin", 5U));
	TEST_ASSERT_EQUAL_INT(1, auth_web_user_find(&g_auth, "opr", 3U));
	TEST_ASSERT_EQUAL_INT(-ENOENT, auth_web_user_find(&g_auth, "nope", 4U));
	TEST_ASSERT_EQUAL_INT(-ENOENT, auth_web_user_find(&g_auth, "admin", 4U));
	TEST_ASSERT_EQUAL_INT(-ENOENT, auth_web_user_find(&g_auth, NULL, 4U));
	TEST_ASSERT_EQUAL_INT(-ENOENT, auth_web_user_find(&g_auth, "a", 0U));
	TEST_ASSERT_EQUAL_INT(-ENOENT, auth_web_user_find(NULL, "a", 1U));

	TEST_ASSERT_NOT_NULL(auth_web_user_at(&g_auth, 0U));
	TEST_ASSERT_EQUAL_STRING("admin", auth_web_user_at(&g_auth, 0U)->name);
	TEST_ASSERT_NULL(auth_web_user_at(&g_auth, AUTH_WEB_USERS));
	TEST_ASSERT_NULL(auth_web_user_at(NULL, 0U));
	TEST_ASSERT_NULL(auth_web_user_at(&g_auth, AUTH_WEB_USERS - 1U));

	/* Only the admin account has a schema key today. */
	TEST_ASSERT_TRUE(auth_web_user_persistent(&g_auth, 0U));
	TEST_ASSERT_FALSE(auth_web_user_persistent(&g_auth, 1U));
	TEST_ASSERT_FALSE(auth_web_user_persistent(&g_auth, AUTH_WEB_USERS));

	/* Table full. */
	for (i = 3U; i < AUTH_WEB_USERS; i++) {
		char nm[8];

		nm[0] = 'u';
		nm[1] = (char)('0' + i);
		nm[2] = '\0';
		TEST_ASSERT_EQUAL_INT((int)i,
				      auth_web_user_add(&g_auth, nm,
							WEB_ROLE_VIEWER, 0U));
	}
	TEST_ASSERT_EQUAL_INT(-ENOSPC, auth_web_user_add(&g_auth, "extra",
							 WEB_ROLE_VIEWER, 0U));
}

static void test_set_password_and_reload(void)
{
	cfg_commit_res_t res;
	cfg_val_t v;

	fixture(true);
	TEST_ASSERT_EQUAL_INT(0, auth_web_user_add(&g_auth, "admin",
						   WEB_ROLE_ADMIN,
						   CFG_ID_SEC_ADMIN_PW));
	/* RAM-only account, so a set must not touch cfg. */
	TEST_ASSERT_EQUAL_INT(1, auth_web_user_add(&g_auth, "opr",
						   WEB_ROLE_OPERATOR, 0U));

	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_set_password(NULL, 0U,
							     (const uint8_t *)PW,
							     PW_LEN));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_set_password(&g_auth, 0U, NULL,
							     PW_LEN));
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      auth_web_set_password(&g_auth, AUTH_WEB_USERS,
						    (const uint8_t *)PW,
						    PW_LEN));
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      auth_web_set_password(&g_auth,
						    AUTH_WEB_USERS - 1U,
						    (const uint8_t *)PW,
						    PW_LEN));
	/* Password policy. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_set_password(&g_auth, 0U,
							     (const uint8_t *)"short",
							     5U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      auth_web_set_password(&g_auth, 0U,
						    (const uint8_t *)PW,
						    AUTH_WEB_PW_MAX + 1U));

	/* Set stages, it does not commit. */
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_password(&g_auth, 0U,
						       (const uint8_t *)PW,
						       PW_LEN));
	TEST_ASSERT_TRUE(cfg_is_staged(&g_cfg, CFG_ID_SEC_ADMIN_PW));
	TEST_ASSERT_EQUAL_INT(0, cfg_get(&g_cfg, CFG_ID_SEC_ADMIN_PW, &v));
	TEST_ASSERT_EQUAL_UINT(0U, v.len); /* live value still empty */

	/* Before the commit, reload finds no stored credential and closes it. */
	TEST_ASSERT_EQUAL_INT(0, auth_web_reload(&g_auth));
	TEST_ASSERT_FALSE(auth_web_user_at(&g_auth, 0U)->has_blob);

	/* After the commit the credential is live. */
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_password(&g_auth, 0U,
						       (const uint8_t *)PW,
						       PW_LEN));
	memset(&res, 0, sizeof(res));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, &res));
	TEST_ASSERT_EQUAL_INT(1, auth_web_reload(&g_auth));
	TEST_ASSERT_TRUE(auth_web_user_at(&g_auth, 0U)->has_blob);

	/* A RAM-only account keeps its credential across a reload. */
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_password(&g_auth, 1U,
						       (const uint8_t *)"opr-password",
						       12U));
	TEST_ASSERT_EQUAL_INT(2, auth_web_reload(&g_auth));
	TEST_ASSERT_TRUE(auth_web_user_at(&g_auth, 1U)->has_blob);

	/* A stored blob of the wrong length is treated as absent. */
	memset(&v, 0, sizeof(v));
	v.type = CFG_T_BLOB;
	v.len = 8U;
	TEST_ASSERT_EQUAL_INT(0, cfg_set(&g_cfg, CFG_ID_SEC_ADMIN_PW, &v));
	memset(&res, 0, sizeof(res));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, &res));
	TEST_ASSERT_EQUAL_INT(1, auth_web_reload(&g_auth)); /* only opr */
	TEST_ASSERT_FALSE(auth_web_user_at(&g_auth, 0U)->has_blob);
}

/* ------------------------------------------------------------------------- */
/* login                                                                     */
/* ------------------------------------------------------------------------- */

static void test_login_ok(void)
{
	auth_web_grant_t g;
	size_t sess = 99U;

	fixture(true);
	provision_admin();

	memset(&g, 0, sizeof(g));
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "admin", 5U,
						(const uint8_t *)PW, PW_LEN,
						1000U, &g));
	TEST_ASSERT_EQUAL_UINT(WEB_ROLE_ADMIN, g.role);
	TEST_ASSERT_EQUAL_UINT(AUTH_WEB_TOKEN_LEN, strlen(g.token));
	TEST_ASSERT_EQUAL_UINT(AUTH_WEB_CSRF_LEN, strlen(g.csrf));
	TEST_ASSERT_EQUAL_UINT32(1U, auth_web_session_count(&g_auth));
	TEST_ASSERT_EQUAL_UINT32(1U, auth_web_stats(&g_auth)->logins_ok);

	/* The tokens are hex and distinct. */
	{
		uint8_t raw[AUTH_WEB_TOKEN_BYTES];

		TEST_ASSERT_EQUAL_INT((int)AUTH_WEB_TOKEN_BYTES,
				      web_hex_decode(g.token, strlen(g.token),
						     raw, sizeof(raw)));
		TEST_ASSERT_TRUE(strcmp(g.token, g.csrf) != 0);
	}

	/* Validate resolves the token and refreshes the idle timer. */
	{
		char tok[AUTH_WEB_TOKEN_LEN + 1U];

		memcpy(tok, g.token, sizeof(tok));
		TEST_ASSERT_EQUAL_INT(0, auth_web_validate(&g_auth, tok, 1500U,
							   &sess));
		TEST_ASSERT_EQUAL_UINT(0U, sess);
		TEST_ASSERT_NOT_NULL(auth_web_sess_at(&g_auth, sess));
		TEST_ASSERT_EQUAL_UINT32(1U,
					 auth_web_sess_at(&g_auth, sess)->requests);
		TEST_ASSERT_NULL(auth_web_sess_at(&g_auth, AUTH_WEB_SESSIONS));
		TEST_ASSERT_NULL(auth_web_sess_at(NULL, 0U));
	}

	/* Unknown tokens. */
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_validate(&g_auth, "nope", 1500U,
							 &sess));
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_validate(&g_auth, "", 1500U,
							 &sess));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_validate(NULL, "x", 0U, &sess));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_validate(&g_auth, NULL, 0U,
							 &sess));

	/* Logout. */
	TEST_ASSERT_EQUAL_INT(0, auth_web_logout(&g_auth, 0U));
	TEST_ASSERT_EQUAL_INT(-ENOENT, auth_web_logout(&g_auth, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_logout(&g_auth,
						       AUTH_WEB_SESSIONS));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_logout(NULL, 0U));
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_session_count(&g_auth));
}

static void test_login_failures(void)
{
	auth_web_grant_t g;

	fixture(true);
	provision_admin();

	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_login(NULL, "admin", 5U,
						      (const uint8_t *)PW,
						      PW_LEN, 0U, &g));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_login(&g_auth, NULL, 5U,
						      (const uint8_t *)PW,
						      PW_LEN, 0U, &g));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_login(&g_auth, "admin", 5U, NULL,
						      PW_LEN, 0U, &g));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_login(&g_auth, "admin", 5U,
						      (const uint8_t *)PW,
						      PW_LEN, 0U, NULL));

	/* An empty or over-long name/password is refused as a bad credential,
	 * never as a lookup. */
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_login(&g_auth, "admin", 0U,
						      (const uint8_t *)PW,
						      PW_LEN, 0U, &g));
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      auth_web_login(&g_auth, "admin",
					     AUTH_WEB_NAME_MAX + 1U,
					     (const uint8_t *)PW, PW_LEN, 0U,
					     &g));
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_login(&g_auth, "admin", 5U,
						      (const uint8_t *)PW, 0U,
						      0U, &g));
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      auth_web_login(&g_auth, "admin", 5U,
					     (const uint8_t *)PW,
					     AUTH_WEB_PW_MAX + 1U, 0U, &g));

	/* An unknown user looks exactly like a wrong password. */
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_login(&g_auth, "root", 4U,
						      (const uint8_t *)PW,
						      PW_LEN, 0U, &g));
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_login(&g_auth, "admin", 5U,
						      (const uint8_t *)"wrong-password",
						      14U, 0U, &g));

	/*
	 * An account with no credential on a box that HAS one elsewhere is
	 * refused with the same -EACCES as a wrong password and an unknown
	 * user. It used to answer -ENOENT, which told an unauthenticated peer
	 * "this account exists but has not been set up yet" — with three
	 * accounts whose names are fixed and public, that is which-account-to-
	 * attack reconnaissance, and core/mcp's h_auth already refuses to give
	 * out the same fact. See the -ENOENT contract in auth_web.h: the code
	 * survives only as a GLOBAL "this box has never been commissioned"
	 * signal, which test_virgin_box_still_reports_unprovisioned() pins.
	 */
	TEST_ASSERT_EQUAL_INT(1, auth_web_user_add(&g_auth, "opr",
						   WEB_ROLE_OPERATOR, 0U));
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_login(&g_auth, "opr", 3U,
						      (const uint8_t *)PW,
						      PW_LEN, 0U, &g));

	/* A failing KDF is an error, never a pass. */
	g_hc.fail_hmac_in = 1U;
	TEST_ASSERT_EQUAL_INT(-EIO, auth_web_login(&g_auth, "admin", 5U,
						   (const uint8_t *)PW, PW_LEN,
						   0U, &g));
	g_hc.fail_hmac_in = 0U;
}

static void test_throttle_escalates(void)
{
	auth_web_grant_t g;
	unsigned int i;
	uint64_t now = 10000U;

	fixture(true);
	provision_admin();

	/* The first AUTH_WEB_FREE_TRIES mismatches answer immediately. */
	for (i = 0U; i < AUTH_WEB_FREE_TRIES; i++) {
		TEST_ASSERT_EQUAL_INT(-EACCES,
				      auth_web_login(&g_auth, "admin", 5U,
						     (const uint8_t *)"bad-password",
						     12U, now, &g));
		TEST_ASSERT_EQUAL_UINT32(0U,
					 auth_web_retry_after_ms(&g_auth, 0U,
								 now));
	}

	/* The next one arms the first backoff window. */
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      auth_web_login(&g_auth, "admin", 5U,
					     (const uint8_t *)"bad-password",
					     12U, now, &g));
	TEST_ASSERT_EQUAL_UINT32(AUTH_WEB_THROTTLE_MS,
				 auth_web_retry_after_ms(&g_auth, 0U, now));

	/* While the window is armed, even the RIGHT password is refused with
	 * EBUSY — the point is that the password is not tested at all. */
	TEST_ASSERT_EQUAL_INT(-EBUSY, auth_web_login(&g_auth, "admin", 5U,
						     (const uint8_t *)PW, PW_LEN,
						     now, &g));
	TEST_ASSERT_EQUAL_UINT32(1U,
				 auth_web_stats(&g_auth)->logins_throttled);

	/* Escalation doubles, then saturates, then hits the long lockout. */
	now += AUTH_WEB_THROTTLE_MS;
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      auth_web_login(&g_auth, "admin", 5U,
					     (const uint8_t *)"bad-password",
					     12U, now, &g));
	TEST_ASSERT_EQUAL_UINT32(AUTH_WEB_THROTTLE_MS * 2U,
				 auth_web_retry_after_ms(&g_auth, 0U, now));

	while (auth_web_user_at(&g_auth, 0U)->fails < (AUTH_WEB_LOCK_TRIES - 1U)) {
		now += auth_web_retry_after_ms(&g_auth, 0U, now);
		TEST_ASSERT_EQUAL_INT(-EACCES,
				      auth_web_login(&g_auth, "admin", 5U,
						     (const uint8_t *)"bad-password",
						     12U, now, &g));
		TEST_ASSERT_TRUE(auth_web_retry_after_ms(&g_auth, 0U, now) <=
				 AUTH_WEB_THROTTLE_MAX_MS);
	}
	now += auth_web_retry_after_ms(&g_auth, 0U, now);
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      auth_web_login(&g_auth, "admin", 5U,
					     (const uint8_t *)"bad-password",
					     12U, now, &g));
	TEST_ASSERT_EQUAL_UINT32(AUTH_WEB_LOCKOUT_MS,
				 auth_web_retry_after_ms(&g_auth, 0U, now));

	/* Once the lockout expires, the right password works and clears it. */
	now += AUTH_WEB_LOCKOUT_MS;
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "admin", 5U,
						(const uint8_t *)PW, PW_LEN, now,
						&g));
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_user_at(&g_auth, 0U)->fails);
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_retry_after_ms(&g_auth, 0U, now));
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_retry_after_ms(&g_auth,
							    AUTH_WEB_USERS, now));
}

static void test_password_change_clears_throttle(void)
{
	auth_web_grant_t g;

	fixture(true);
	provision_admin();

	/* Arm a window. */
	{
		unsigned int i;

		for (i = 0U; i <= AUTH_WEB_FREE_TRIES; i++) {
			(void)auth_web_login(&g_auth, "admin", 5U,
					     (const uint8_t *)"bad-password",
					     12U, 0U, &g);
		}
	}
	TEST_ASSERT_TRUE(auth_web_retry_after_ms(&g_auth, 0U, 0U) > 0U);
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_password(&g_auth, 0U,
						       (const uint8_t *)"a-new-password",
						       14U));
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_retry_after_ms(&g_auth, 0U, 0U));
}

/* ------------------------------------------------------------------------- */
/* sessions                                                                  */
/* ------------------------------------------------------------------------- */

static void test_session_idle_timeout(void)
{
	auth_web_grant_t g;
	char tok[AUTH_WEB_TOKEN_LEN + 1U];
	uint64_t idle_ms;

	fixture(true);
	provision_admin();
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "admin", 5U,
						(const uint8_t *)PW, PW_LEN, 0U,
						&g));
	memcpy(tok, g.token, sizeof(tok));
	idle_ms = (uint64_t)g.idle_s * 1000ULL;
	/* cfg default sec.session.s is 600 s. */
	TEST_ASSERT_EQUAL_UINT32(600U, g.idle_s);

	/* Just inside the window: still alive, and the timer resets. */
	TEST_ASSERT_EQUAL_INT(0, auth_web_validate(&g_auth, tok, idle_ms - 1U,
						   NULL));
	/* One idle period after that refresh it is still alive. */
	TEST_ASSERT_EQUAL_INT(0, auth_web_validate(&g_auth, tok,
						   (idle_ms - 1U) + idle_ms - 1U,
						   NULL));
	/* Past the window: closed, and reported as expired exactly once. */
	TEST_ASSERT_EQUAL_INT(-ETIME,
			      auth_web_validate(&g_auth, tok,
						(idle_ms - 1U) + (2U * idle_ms),
						NULL));
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_validate(&g_auth, tok, 0U, NULL));
	TEST_ASSERT_EQUAL_UINT32(1U, auth_web_stats(&g_auth)->expired_idle);
}

static void test_session_absolute_timeout(void)
{
	auth_web_grant_t g;
	char tok[AUTH_WEB_TOKEN_LEN + 1U];
	uint64_t abs_ms;
	uint64_t t;

	fixture(true);
	provision_admin();
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "admin", 5U,
						(const uint8_t *)PW, PW_LEN, 0U,
						&g));
	memcpy(tok, g.token, sizeof(tok));
	abs_ms = (uint64_t)g.absolute_s * 1000ULL;
	TEST_ASSERT_TRUE(abs_ms > 0U);

	/* Keep it busy so the idle timer never fires, and watch the absolute
	 * lifetime end the session anyway. */
	for (t = 60000U; t < abs_ms; t += 60000U) {
		TEST_ASSERT_EQUAL_INT(0, auth_web_validate(&g_auth, tok, t,
							   NULL));
	}
	TEST_ASSERT_EQUAL_INT(-ETIME, auth_web_validate(&g_auth, tok, abs_ms,
							NULL));
	TEST_ASSERT_EQUAL_UINT32(1U, auth_web_stats(&g_auth)->expired_absolute);
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_stats(&g_auth)->expired_idle);
}

static void test_session_tick_and_logout_all(void)
{
	auth_web_grant_t g;

	fixture(true);
	provision_admin();
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "admin", 5U,
						(const uint8_t *)PW, PW_LEN, 0U,
						&g));
	TEST_ASSERT_EQUAL_INT(0, auth_web_tick(&g_auth, 1000U));
	TEST_ASSERT_EQUAL_INT(1, auth_web_tick(&g_auth, 60ULL * 60ULL * 1000ULL));
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_session_count(&g_auth));

	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "admin", 5U,
						(const uint8_t *)PW, PW_LEN, 0U,
						&g));
	auth_web_logout_all(&g_auth);
	auth_web_logout_all(NULL);
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_session_count(&g_auth));
	TEST_ASSERT_TRUE(auth_web_stats(&g_auth)->logouts >= 1U);
}

static void test_session_table_full_evicts_lru(void)
{
	auth_web_grant_t g;
	char first[AUTH_WEB_TOKEN_LEN + 1U];
	unsigned int i;

	fixture(true);
	provision_admin();

	for (i = 0U; i < AUTH_WEB_SESSIONS; i++) {
		TEST_ASSERT_EQUAL_INT(0,
				      auth_web_login(&g_auth, "admin", 5U,
						     (const uint8_t *)PW, PW_LEN,
						     1000U + i, &g));
		if (i == 0U) {
			memcpy(first, g.token, sizeof(first));
		}
	}
	TEST_ASSERT_EQUAL_UINT32(AUTH_WEB_SESSIONS,
				 auth_web_session_count(&g_auth));

	/* One more login evicts the least-recently-used session rather than
	 * locking the operator out of their own box. */
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "admin", 5U,
						(const uint8_t *)PW, PW_LEN,
						2000U, &g));
	TEST_ASSERT_EQUAL_UINT32(AUTH_WEB_SESSIONS,
				 auth_web_session_count(&g_auth));
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_validate(&g_auth, first, 2000U,
							 NULL));
	TEST_ASSERT_EQUAL_UINT32(1U, auth_web_stats(&g_auth)->logins_no_slot);
}

static void test_csrf(void)
{
	auth_web_grant_t g;
	char csrf[AUTH_WEB_CSRF_LEN + 1U];
	char wrong[AUTH_WEB_CSRF_LEN + 1U];

	fixture(true);
	provision_admin();
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "admin", 5U,
						(const uint8_t *)PW, PW_LEN, 0U,
						&g));
	memcpy(csrf, g.csrf, sizeof(csrf));

	TEST_ASSERT_EQUAL_INT(0, auth_web_check_csrf(&g_auth, 0U, csrf,
						     AUTH_WEB_CSRF_LEN));
	/* Absent. */
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_check_csrf(&g_auth, 0U, NULL,
							   0U));
	/* Wrong length: a prefix must never be accepted. */
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_check_csrf(&g_auth, 0U, csrf,
							   AUTH_WEB_CSRF_LEN -
								   1U));
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_check_csrf(&g_auth, 0U, csrf,
							   AUTH_WEB_CSRF_LEN +
								   1U));
	/* Right length, wrong value. */
	memcpy(wrong, csrf, sizeof(wrong));
	wrong[0] = (char)((wrong[0] == 'a') ? 'b' : 'a');
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_check_csrf(&g_auth, 0U, wrong,
							   AUTH_WEB_CSRF_LEN));
	TEST_ASSERT_TRUE(auth_web_stats(&g_auth)->csrf_rejected >= 4U);

	/* Bad arguments and dead sessions. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_check_csrf(NULL, 0U, csrf,
							   AUTH_WEB_CSRF_LEN));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      auth_web_check_csrf(&g_auth, AUTH_WEB_SESSIONS,
						  csrf, AUTH_WEB_CSRF_LEN));
	TEST_ASSERT_EQUAL_INT(0, auth_web_logout(&g_auth, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_web_check_csrf(&g_auth, 0U, csrf,
							   AUTH_WEB_CSRF_LEN));
}

static void test_auth_required_policy(void)
{
	cfg_commit_res_t res;

	fixture(true);
	/* Schema default is 1. */
	TEST_ASSERT_TRUE(auth_web_required(&g_auth));
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_SEC_AUTH_REQUIRED,
					     0U));
	memset(&res, 0, sizeof(res));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, &res));
	TEST_ASSERT_FALSE(auth_web_required(&g_auth));

	/* The idle timeout follows sec.session.s. */
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_SEC_SESSION_S, 30U));
	memset(&res, 0, sizeof(res));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, &res));
	provision_admin();
	{
		auth_web_grant_t g;

		TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "admin", 5U,
							(const uint8_t *)PW,
							PW_LEN, 0U, &g));
		TEST_ASSERT_EQUAL_UINT32(30U, g.idle_s);
		TEST_ASSERT_EQUAL_INT(-ETIME, auth_web_validate(&g_auth, g.token,
								30000U, NULL));
	}
}

/* Everything still works with no cfg and no log wired (minimal build). */
static void test_ram_only_build(void)
{
	auth_web_grant_t g;

	fixture(false);
	g_auth.log = NULL;
	TEST_ASSERT_EQUAL_INT(0, auth_web_user_add(&g_auth, "admin",
						   WEB_ROLE_ADMIN, 0U));
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_password(&g_auth, 0U,
						       (const uint8_t *)PW,
						       PW_LEN));
	TEST_ASSERT_FALSE(auth_web_user_persistent(&g_auth, 0U));
	TEST_ASSERT_EQUAL_INT(1, auth_web_reload(&g_auth));
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "admin", 5U,
						(const uint8_t *)PW, PW_LEN, 0U,
						&g));
	TEST_ASSERT_EQUAL_UINT32(AUTH_WEB_IDLE_S_DEFAULT, g.idle_s);
	/* An unknown user with no log sink must not crash. */
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_login(&g_auth, "x", 1U,
						      (const uint8_t *)PW,
						      PW_LEN, 0U, &g));
}

/* A rand failure must fail the login, never issue a predictable token. */
static void test_rand_failure(void)
{
	auth_web_grant_t g;

	fixture(true);
	provision_admin();
	g_hc.fail_rand_in = 1U;
	TEST_ASSERT_EQUAL_INT(-EIO, auth_web_login(&g_auth, "admin", 5U,
						   (const uint8_t *)PW, PW_LEN,
						   0U, &g));
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_session_count(&g_auth));
	g_hc.fail_rand_in = 0U;

	/* A failure on the SECOND draw (the CSRF token) must also close it. */
	g_hc.fail_rand_in = 2U;
	TEST_ASSERT_EQUAL_INT(-EIO, auth_web_login(&g_auth, "admin", 5U,
						   (const uint8_t *)PW, PW_LEN,
						   0U, &g));
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_session_count(&g_auth));
	g_hc.fail_rand_in = 0U;
}

/* ------------------------------------------------------------------------- */
/* the remote authority                                                      */
/* ------------------------------------------------------------------------- */

/*
 * A stand-in for sts_aaa_check(), with the same three-valued contract: 0
 * accepts and names a role, -EBUSY is a lockout, and every other code — most
 * importantly -EHOSTUNREACH, "no authority could answer" — is a refusal.
 */
static struct {
	unsigned int calls;
	char         last_name[AUTH_WEB_NAME_MAX + 1U];
	char         last_secret[AUTH_WEB_PW_MAX + 1U];
	int          rc;   /* what the authority answers */
	uint8_t      role; /* the role it names when rc == 0 */
} g_remote;

static int remote_stub(void *user, const char *name, const char *secret,
		       uint8_t *out_role)
{
	(void)user;
	g_remote.calls++;
	web_span_copy(g_remote.last_name, sizeof(g_remote.last_name), name,
		      strlen(name));
	web_span_copy(g_remote.last_secret, sizeof(g_remote.last_secret), secret,
		      strlen(secret));
	if (g_remote.rc == 0) {
		*out_role = g_remote.role;
	}
	return g_remote.rc;
}

static void remote_reset(int rc, uint8_t role)
{
	memset(&g_remote, 0, sizeof(g_remote));
	g_remote.rc = rc;
	g_remote.role = role;
}

/* A local MISS falls through to the authority; a local SUCCESS never does. */
static void test_remote_consulted_only_on_local_miss(void)
{
	auth_web_grant_t g;

	fixture(true);
	provision_admin();
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_remote(&g_auth, remote_stub, NULL));

	/* A correct local password must not put the credential on the wire. */
	remote_reset(0, (uint8_t)WEB_ROLE_ADMIN);
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "admin", 5U,
						(const uint8_t *)PW, PW_LEN, 0U,
						&g));
	TEST_ASSERT_EQUAL_UINT32(0U, g_remote.calls);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)WEB_ROLE_ADMIN, g.role);
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_stats(&g_auth)->remote_consults);

	/* An unknown account does, and is granted the role the authority named. */
	remote_reset(0, (uint8_t)WEB_ROLE_OPERATOR);
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "alice", 5U,
						(const uint8_t *)"from-radius",
						11U, 0U, &g));
	TEST_ASSERT_EQUAL_UINT32(1U, g_remote.calls);
	TEST_ASSERT_EQUAL_STRING("alice", g_remote.last_name);
	TEST_ASSERT_EQUAL_STRING("from-radius", g_remote.last_secret);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)WEB_ROLE_OPERATOR, g.role);
	TEST_ASSERT_EQUAL_UINT32(1U, auth_web_stats(&g_auth)->logins_remote_ok);

	/* A KNOWN account with the WRONG password does too. */
	remote_reset(0, (uint8_t)WEB_ROLE_VIEWER);
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "admin", 5U,
						(const uint8_t *)"not-the-local-one",
						17U, 0U, &g));
	TEST_ASSERT_EQUAL_UINT32(1U, g_remote.calls);
	/* The authority is authoritative about authorisation: the session gets
	 * the role it named, not the local account's. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)WEB_ROLE_VIEWER, g.role);
}

/* -EHOSTUNREACH is a DENIAL. Never an allow-on-failure. */
static void test_remote_unreachable_denies(void)
{
	auth_web_grant_t g;

	fixture(true);
	provision_admin();
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_remote(&g_auth, remote_stub, NULL));

	remote_reset(-EHOSTUNREACH, (uint8_t)WEB_ROLE_ADMIN);
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_login(&g_auth, "alice", 5U,
						      (const uint8_t *)"anything",
						      8U, 0U, &g));
	TEST_ASSERT_EQUAL_UINT32(1U, g_remote.calls);
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_session_count(&g_auth));
	TEST_ASSERT_EQUAL_UINT32(1U,
				 auth_web_stats(&g_auth)->logins_remote_denied);

	/* The role the authority left in *out_role must not leak into a grant
	 * either: a refused login writes no session at all. */
	TEST_ASSERT_NULL(auth_web_sess_at(&g_auth, 0U));

	/* Same for a refused-but-reachable authority, and for a broken one that
	 * answers with a code nobody documented. */
	remote_reset(-EACCES, 0U);
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_login(&g_auth, "alice", 5U,
						      (const uint8_t *)"anything",
						      8U, 0U, &g));
	remote_reset(-EPROTO, 0U);
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_login(&g_auth, "alice", 5U,
						      (const uint8_t *)"anything",
						      8U, 0U, &g));
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_session_count(&g_auth));

	/* -EBUSY is the one code that keeps its identity: it is a lockout, and
	 * the caller needs it to populate Retry-After. It says nothing about
	 * the password. */
	remote_reset(-EBUSY, 0U);
	TEST_ASSERT_EQUAL_INT(-EBUSY, auth_web_login(&g_auth, "alice", 5U,
						     (const uint8_t *)"anything",
						     8U, 0U, &g));
}

/* An authority that accepts without naming a usable role gets LEAST privilege. */
static void test_remote_role_is_clamped(void)
{
	auth_web_grant_t g;

	fixture(true);
	provision_admin();
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_remote(&g_auth, remote_stub, NULL));

	remote_reset(0, (uint8_t)WEB_ROLE_NONE);
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "alice", 5U,
						(const uint8_t *)"pw", 2U, 0U,
						&g));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)WEB_ROLE_VIEWER, g.role);

	auth_web_logout_all(&g_auth);
	remote_reset(0, 200U); /* out of range */
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "alice", 5U,
						(const uint8_t *)"pw", 2U, 0U,
						&g));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)WEB_ROLE_VIEWER, g.role);
}

/*
 * A remote principal has no local account. The session must still name it, and
 * every existing caller that maps a session index to an account must fail safe
 * rather than read a neighbouring account's row.
 */
static void test_remote_session_has_no_local_account(void)
{
	auth_web_grant_t g;
	const auth_web_sess_t *s;
	size_t sess = 0U;

	fixture(true);
	provision_admin();
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_remote(&g_auth, remote_stub, NULL));

	remote_reset(0, (uint8_t)WEB_ROLE_OPERATOR);
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "alice", 5U,
						(const uint8_t *)"pw", 2U, 0U,
						&g));
	TEST_ASSERT_EQUAL_INT(0, auth_web_validate(&g_auth, g.token, 0U, &sess));

	s = auth_web_sess_at(&g_auth, sess);
	TEST_ASSERT_NOT_NULL(s);
	TEST_ASSERT_TRUE(s->remote);
	TEST_ASSERT_EQUAL_UINT8(AUTH_WEB_NO_USER, s->user);
	/* The whole point of the sentinel: this must be NULL, not users[0]. */
	TEST_ASSERT_NULL(auth_web_user_at(&g_auth, s->user));
	TEST_ASSERT_EQUAL_STRING("alice", auth_web_sess_name(&g_auth, sess));

	/* A local login still names its account both ways. */
	remote_reset(-EHOSTUNREACH, 0U);
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "admin", 5U,
						(const uint8_t *)PW, PW_LEN, 0U,
						&g));
	TEST_ASSERT_EQUAL_INT(0, auth_web_validate(&g_auth, g.token, 0U, &sess));
	s = auth_web_sess_at(&g_auth, sess);
	TEST_ASSERT_NOT_NULL(s);
	TEST_ASSERT_FALSE(s->remote);
	TEST_ASSERT_NOT_NULL(auth_web_user_at(&g_auth, s->user));
	TEST_ASSERT_EQUAL_STRING("admin", auth_web_sess_name(&g_auth, sess));
}

/*
 * A wrong password and an unknown user must be ONE answer — with a remote
 * authority wired, which is where they could most easily diverge (an unknown
 * user could short-circuit before the authority is asked, a known one could
 * report the authority's own code).
 */
static void test_wrong_password_and_unknown_user_are_identical(void)
{
	auth_web_grant_t g;
	const auth_web_stats_t *st;
	int rc_unknown;
	int rc_wrong;
	int rc_unprovisioned;
	unsigned int calls_unknown;
	unsigned int calls_wrong;

	fixture(true);
	provision_admin();
	/* A second, deliberately UNPROVISIONED account, which is what the new
	 * operator/viewer schema keys make the normal state of a half-set-up
	 * box. */
	TEST_ASSERT_EQUAL_INT(1, auth_web_user_add(&g_auth, "operator",
						   (uint8_t)WEB_ROLE_OPERATOR,
						   0U));
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_remote(&g_auth, remote_stub, NULL));

	remote_reset(-EHOSTUNREACH, 0U);
	rc_unknown = auth_web_login(&g_auth, "nosuchuser", 10U,
				    (const uint8_t *)"guess", 5U, 0U, &g);
	calls_unknown = g_remote.calls;

	remote_reset(-EHOSTUNREACH, 0U);
	rc_wrong = auth_web_login(&g_auth, "admin", 5U,
				  (const uint8_t *)"guess", 5U, 0U, &g);
	calls_wrong = g_remote.calls;

	remote_reset(-EHOSTUNREACH, 0U);
	rc_unprovisioned = auth_web_login(&g_auth, "operator", 8U,
					  (const uint8_t *)"guess", 5U, 0U, &g);

	TEST_ASSERT_EQUAL_INT(-EACCES, rc_unknown);
	TEST_ASSERT_EQUAL_INT(rc_unknown, rc_wrong);
	TEST_ASSERT_EQUAL_INT(rc_unknown, rc_unprovisioned);

	/* Not just the same code: all three must actually reach the authority,
	 * so the answer does not differ in latency either. */
	TEST_ASSERT_EQUAL_UINT32(1U, calls_unknown);
	TEST_ASSERT_EQUAL_UINT32(1U, calls_wrong);
	TEST_ASSERT_EQUAL_UINT32(1U, g_remote.calls);

	/* And nothing was granted. */
	st = auth_web_stats(&g_auth);
	TEST_ASSERT_EQUAL_UINT32(0U, st->logins_ok);
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_session_count(&g_auth));
}

/*
 * The bootstrap signal that rest.c turns into its "commission this box" 503.
 * It is a global property — a box with NO credential at all — never a
 * per-account one.
 */
static void test_virgin_box_still_reports_unprovisioned(void)
{
	auth_web_grant_t g;

	fixture(true);
	TEST_ASSERT_EQUAL_INT(0, auth_web_user_add(&g_auth, "admin",
						   (uint8_t)WEB_ROLE_ADMIN,
						   (uint16_t)CFG_ID_SEC_ADMIN_PW));
	TEST_ASSERT_EQUAL_INT(1, auth_web_user_add(&g_auth, "operator",
						   (uint8_t)WEB_ROLE_OPERATOR,
						   0U));
	TEST_ASSERT_EQUAL_INT(0, auth_web_reload(&g_auth));

	/* No remote wired: a known name on a virgin box asks to be commissioned. */
	TEST_ASSERT_EQUAL_INT(-ENOENT, auth_web_login(&g_auth, "admin", 5U,
						      (const uint8_t *)PW,
						      PW_LEN, 0U, &g));

	/*
	 * With a remote wired it STILL does, once the authority has refused.
	 * The production glue always wires one, so a condition that tested for
	 * the hook would have deleted this message from every shipped image.
	 */
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_remote(&g_auth, remote_stub, NULL));
	remote_reset(-EHOSTUNREACH, 0U);
	TEST_ASSERT_EQUAL_INT(-ENOENT, auth_web_login(&g_auth, "admin", 5U,
						      (const uint8_t *)PW,
						      PW_LEN, 0U, &g));
	TEST_ASSERT_EQUAL_UINT32(1U, g_remote.calls);

	/* An UNKNOWN name gets nothing but -EACCES even here, so the signal
	 * cannot be probed with arbitrary names. */
	remote_reset(-EHOSTUNREACH, 0U);
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_login(&g_auth, "probe", 5U,
						      (const uint8_t *)PW,
						      PW_LEN, 0U, &g));

	/* Commission ONE account and the global signal goes away for the other. */
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_password(&g_auth, 0U,
						       (const uint8_t *)PW,
						       PW_LEN));
	remote_reset(-EHOSTUNREACH, 0U);
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_login(&g_auth, "operator", 8U,
						      (const uint8_t *)PW,
						      PW_LEN, 0U, &g));
}

/* An unwired hook must leave the pre-hook behaviour exactly as it was. */
static void test_unwired_hook_changes_nothing(void)
{
	auth_web_grant_t g;

	fixture(true);
	provision_admin();
	/* Explicitly wired to NULL — the documented "no authority" state. */
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_remote(&g_auth, NULL, NULL));

	/* Correct password still opens a session with the local role. */
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "admin", 5U,
						(const uint8_t *)PW, PW_LEN, 0U,
						&g));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)WEB_ROLE_ADMIN, g.role);

	/* Wrong password and unknown user are still -EACCES, and nothing is
	 * consulted. */
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_login(&g_auth, "admin", 5U,
						      (const uint8_t *)"nope", 4U,
						      0U, &g));
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_login(&g_auth, "ghost", 5U,
						      (const uint8_t *)PW, PW_LEN,
						      0U, &g));
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_stats(&g_auth)->remote_consults);

	/* The local throttle still escalates to a hard lockout on its own. */
	{
		unsigned int i;

		for (i = 0U; i < AUTH_WEB_LOCK_TRIES + 2U; i++) {
			(void)auth_web_login(&g_auth, "admin", 5U,
					     (const uint8_t *)"nope", 4U, 0U, &g);
		}
		TEST_ASSERT_EQUAL_INT(-EBUSY, auth_web_login(&g_auth, "admin", 5U,
							     (const uint8_t *)PW,
							     PW_LEN, 0U, &g));
	}
}

/*
 * A throttled account must not reach the network: the whole point of testing
 * the lockout first is that a brute-force attempt costs the AAA server nothing.
 */
static void test_throttled_account_never_reaches_the_authority(void)
{
	auth_web_grant_t g;
	unsigned int i;

	fixture(true);
	provision_admin();
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_remote(&g_auth, remote_stub, NULL));
	remote_reset(-EACCES, 0U);

	for (i = 0U; i < AUTH_WEB_LOCK_TRIES + 2U; i++) {
		(void)auth_web_login(&g_auth, "admin", 5U,
				     (const uint8_t *)"nope", 4U, 0U, &g);
	}
	TEST_ASSERT_EQUAL_INT(-EBUSY, auth_web_login(&g_auth, "admin", 5U,
						     (const uint8_t *)"nope", 4U,
						     0U, &g));
	{
		unsigned int before = g_remote.calls;

		TEST_ASSERT_EQUAL_INT(-EBUSY,
				      auth_web_login(&g_auth, "admin", 5U,
						     (const uint8_t *)"nope", 4U,
						     0U, &g));
		TEST_ASSERT_EQUAL_UINT32(before, g_remote.calls);
	}
}

/*
 * A password carrying an embedded NUL must be refused, not truncated at it:
 * the hook and sts_aaa_check() below it speak C strings, so "pw\0junk" would
 * otherwise authenticate as "pw".
 */
static void test_embedded_nul_password_is_refused(void)
{
	static const uint8_t split_pw[] = { 'p', 'w', 0x00, 'j', 'u', 'n', 'k' };
	auth_web_grant_t g;

	fixture(true);
	provision_admin();
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_remote(&g_auth, remote_stub, NULL));
	remote_reset(0, (uint8_t)WEB_ROLE_ADMIN); /* would accept anything */

	TEST_ASSERT_EQUAL_INT(-EACCES, auth_web_login(&g_auth, "alice", 5U,
						      split_pw, sizeof(split_pw),
						      0U, &g));
	TEST_ASSERT_EQUAL_UINT32(0U, g_remote.calls);
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_session_count(&g_auth));
}

/* A factory reset must leave no credential and no session behind. */
static void test_wipe_erases_credentials_and_sessions(void)
{
	auth_web_grant_t g;
	const auth_web_user_t *u;
	size_t i;

	fixture(true);
	provision_admin();
	TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "admin", 5U,
						(const uint8_t *)PW, PW_LEN, 0U,
						&g));
	TEST_ASSERT_EQUAL_UINT32(1U, auth_web_session_count(&g_auth));

	auth_web_wipe(&g_auth);

	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_session_count(&g_auth));
	u = auth_web_user_at(&g_auth, 0U);
	TEST_ASSERT_NOT_NULL(u);           /* the account itself survives */
	TEST_ASSERT_EQUAL_STRING("admin", u->name);
	TEST_ASSERT_FALSE(u->has_blob);
	for (i = 0U; i < AUTH_WEB_BLOB_LEN; i++) {
		TEST_ASSERT_EQUAL_UINT8(0U, u->blob[i]);
	}
	/* No stale token bytes anywhere in the session table. */
	for (i = 0U; i < AUTH_WEB_SESSIONS; i++) {
		TEST_ASSERT_EQUAL_UINT8(0U, (uint8_t)g_auth.sess[i].token[0]);
		TEST_ASSERT_EQUAL_UINT8(0U, (uint8_t)g_auth.sess[i].csrf[0]);
	}
	/* And the credential really is gone: the old password no longer works. */
	TEST_ASSERT_EQUAL_INT(-ENOENT, auth_web_login(&g_auth, "admin", 5U,
						      (const uint8_t *)PW,
						      PW_LEN, 0U, &g));
	auth_web_wipe(&g_auth); /* idempotent */
	auth_web_wipe(NULL);    /* and NULL-safe */
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_init_args);
	RUN_TEST(test_blob_matches_mcp_format);
	RUN_TEST(test_users);
	RUN_TEST(test_set_password_and_reload);

	RUN_TEST(test_login_ok);
	RUN_TEST(test_login_failures);
	RUN_TEST(test_throttle_escalates);
	RUN_TEST(test_password_change_clears_throttle);

	RUN_TEST(test_session_idle_timeout);
	RUN_TEST(test_session_absolute_timeout);
	RUN_TEST(test_session_tick_and_logout_all);
	RUN_TEST(test_session_table_full_evicts_lru);
	RUN_TEST(test_csrf);
	RUN_TEST(test_auth_required_policy);
	RUN_TEST(test_ram_only_build);
	RUN_TEST(test_rand_failure);

	RUN_TEST(test_remote_consulted_only_on_local_miss);
	RUN_TEST(test_remote_unreachable_denies);
	RUN_TEST(test_remote_role_is_clamped);
	RUN_TEST(test_remote_session_has_no_local_account);
	RUN_TEST(test_wrong_password_and_unknown_user_are_identical);
	RUN_TEST(test_virgin_box_still_reports_unprovisioned);
	RUN_TEST(test_unwired_hook_changes_nothing);
	RUN_TEST(test_throttled_account_never_reaches_the_authority);
	RUN_TEST(test_embedded_nul_password_is_refused);
	RUN_TEST(test_wipe_erases_credentials_and_sessions);

	return UNITY_END();
}
