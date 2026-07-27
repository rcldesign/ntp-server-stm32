/*
 * STS1000 "Meridian" — core/auth AAA chain + LDAP codec unit tests.
 *
 * Two modules share this suite because they share a source directory: the LDAP
 * codec has no chain of its own to drive it, and the chain's LDAP backend is
 * exactly what its parsers feed. Splitting them would compile the same objects
 * twice for no extra signal.
 *
 * Provenance of the expectations (ARCHITECTURE.md §9):
 *
 *   - **The local credential layout is pinned by construction, not by
 *     round-trip.** The blob is built here from the primitives —
 *     `salt ‖ HMAC-SHA-256(key = salt, msg = password)` using the host's own
 *     HMAC — and auth_local_verify() must accept it. That is the same layout
 *     core/mcp's `mcp_auth_make_blob()` writes and cfg key `sec.admin.pw`
 *     stores, so if either side ever changes shape this test fails rather than
 *     the console silently rejecting every login.
 *
 *   - **LDAP messages are checked against literal RFC 4511 §4 encodings.** A
 *     simple BindRequest for `cn=svc` with password `pw` is written out octet by
 *     octet from the ASN.1 in §4.2 plus X.690 §8, including the `[0]` primitive
 *     context tag (0x80) on the simple authentication choice — the one field a
 *     hand-rolled encoder gets wrong.
 *
 *   - The chain's behaviour is asserted as the policy the header states: a
 *     reject stops the chain, an unavailable backend does not, and "every
 *     backend unavailable" is a denial. Those are the properties that decide
 *     whether a mis-ordered chain is a privilege-escalation path.
 *
 *   - Every parser is fuzzed with random and mutated input.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "auth/auth.h"
#include "auth/ldap.h"
#include "host_sha256.h"
#include "test_support.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* ========================================================================= */
/* roles, backends, chain parsing                                            */
/* ========================================================================= */

static void test_role_names_and_parsing(void)
{
	TEST_ASSERT_EQUAL_STRING("none", auth_role_name(AUTH_ROLE_NONE));
	TEST_ASSERT_EQUAL_STRING("viewer", auth_role_name(AUTH_ROLE_VIEWER));
	TEST_ASSERT_EQUAL_STRING("operator", auth_role_name(AUTH_ROLE_OPERATOR));
	TEST_ASSERT_EQUAL_STRING("admin", auth_role_name(AUTH_ROLE_ADMIN));
	TEST_ASSERT_EQUAL_STRING("none", auth_role_name(99U));

	/* Roles are ordered so `>=` is a privilege test. */
	TEST_ASSERT_TRUE(AUTH_ROLE_ADMIN > AUTH_ROLE_OPERATOR);
	TEST_ASSERT_TRUE(AUTH_ROLE_OPERATOR > AUTH_ROLE_VIEWER);
	TEST_ASSERT_TRUE(AUTH_ROLE_VIEWER > AUTH_ROLE_NONE);

	TEST_ASSERT_EQUAL_INT(AUTH_ROLE_ADMIN, auth_role_parse("admin"));
	TEST_ASSERT_EQUAL_INT(AUTH_ROLE_ADMIN, auth_role_parse("ADMIN"));
	TEST_ASSERT_EQUAL_INT(AUTH_ROLE_ADMIN, auth_role_parse("Root"));
	TEST_ASSERT_EQUAL_INT(AUTH_ROLE_ADMIN, auth_role_parse("superuser"));
	TEST_ASSERT_EQUAL_INT(AUTH_ROLE_ADMIN,
			      auth_role_parse("administrative"));
	TEST_ASSERT_EQUAL_INT(AUTH_ROLE_ADMIN, auth_role_parse("rw-admin"));
	TEST_ASSERT_EQUAL_INT(AUTH_ROLE_OPERATOR, auth_role_parse("operator"));
	TEST_ASSERT_EQUAL_INT(AUTH_ROLE_OPERATOR, auth_role_parse("rw"));
	TEST_ASSERT_EQUAL_INT(AUTH_ROLE_OPERATOR, auth_role_parse("ReadWrite"));
	TEST_ASSERT_EQUAL_INT(AUTH_ROLE_OPERATOR, auth_role_parse("read-write"));
	TEST_ASSERT_EQUAL_INT(AUTH_ROLE_VIEWER, auth_role_parse("viewer"));
	TEST_ASSERT_EQUAL_INT(AUTH_ROLE_VIEWER, auth_role_parse("ro"));
	TEST_ASSERT_EQUAL_INT(AUTH_ROLE_VIEWER, auth_role_parse("readonly"));
	TEST_ASSERT_EQUAL_INT(AUTH_ROLE_VIEWER, auth_role_parse("read-only"));
	TEST_ASSERT_EQUAL_INT(AUTH_ROLE_VIEWER, auth_role_parse("monitor"));
	TEST_ASSERT_EQUAL_INT(AUTH_ROLE_VIEWER, auth_role_parse("guest"));

	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_role_parse(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_role_parse(""));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_role_parse("wizard"));
	/* A prefix must not match: "admin" is not "adm". */
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_role_parse("adm"));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_role_parse("administrator"));
}

static void test_backend_names(void)
{
	TEST_ASSERT_EQUAL_STRING("local", auth_backend_name(AUTH_BE_LOCAL));
	TEST_ASSERT_EQUAL_STRING("radius", auth_backend_name(AUTH_BE_RADIUS));
	TEST_ASSERT_EQUAL_STRING("tacacs", auth_backend_name(AUTH_BE_TACACS));
	TEST_ASSERT_EQUAL_STRING("ldap", auth_backend_name(AUTH_BE_LDAP));
	TEST_ASSERT_EQUAL_STRING("?", auth_backend_name(9U));
}

static void test_order_parsing(void)
{
	uint8_t o[AUTH_BE_COUNT];
	int n;

	n = auth_order_parse("local,radius,tacacs,ldap", o, sizeof(o));
	TEST_ASSERT_EQUAL_INT(4, n);
	TEST_ASSERT_EQUAL_UINT8(AUTH_BE_LOCAL, o[0]);
	TEST_ASSERT_EQUAL_UINT8(AUTH_BE_RADIUS, o[1]);
	TEST_ASSERT_EQUAL_UINT8(AUTH_BE_TACACS, o[2]);
	TEST_ASSERT_EQUAL_UINT8(AUTH_BE_LDAP, o[3]);

	/* Whitespace, mixed case, "tacacs+", and a repeated entry. */
	n = auth_order_parse(" LDAP  tacacs+ , ldap ,Local", o, sizeof(o));
	TEST_ASSERT_EQUAL_INT(3, n);
	TEST_ASSERT_EQUAL_UINT8(AUTH_BE_LDAP, o[0]);
	TEST_ASSERT_EQUAL_UINT8(AUTH_BE_TACACS, o[1]);
	TEST_ASSERT_EQUAL_UINT8(AUTH_BE_LOCAL, o[2]);

	/* Unknown words are skipped, not fatal. */
	n = auth_order_parse("kerberos,radius,nis", o, sizeof(o));
	TEST_ASSERT_EQUAL_INT(1, n);
	TEST_ASSERT_EQUAL_UINT8(AUTH_BE_RADIUS, o[0]);

	/* A capacity of 1 stops after the first. */
	n = auth_order_parse("radius,ldap", o, 1U);
	TEST_ASSERT_EQUAL_INT(1, n);
	TEST_ASSERT_EQUAL_UINT8(AUTH_BE_RADIUS, o[0]);

	TEST_ASSERT_EQUAL_INT(0, auth_order_parse("", o, sizeof(o)));
	TEST_ASSERT_EQUAL_INT(0, auth_order_parse(",,, ,", o, sizeof(o)));
	TEST_ASSERT_EQUAL_INT(0, auth_order_parse(NULL, o, sizeof(o)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_order_parse("local", NULL,
							sizeof(o)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_order_parse("local", o, 0U));
}

static void test_constant_time_compare(void)
{
	static const uint8_t a[4] = { 1, 2, 3, 4 };
	static const uint8_t b[4] = { 1, 2, 3, 5 };

	TEST_ASSERT_TRUE(auth_ct_eq(a, a, sizeof(a)));
	TEST_ASSERT_FALSE(auth_ct_eq(a, b, sizeof(a)));
	TEST_ASSERT_TRUE(auth_ct_eq(a, b, 3U));
	TEST_ASSERT_TRUE(auth_ct_eq(a, b, 0U));
	TEST_ASSERT_FALSE(auth_ct_eq(NULL, b, 4U));
	TEST_ASSERT_FALSE(auth_ct_eq(a, NULL, 4U));
}

/* ========================================================================= */
/* the local credential                                                      */
/* ========================================================================= */

static const char *const PW = "correct horse";

/** Build the stored blob exactly as the console's provisioning path does. */
static void make_blob(uint8_t blob[AUTH_PW_BLOB_LEN], const char *pw,
		      uint8_t salt_fill)
{
	memset(blob, salt_fill, AUTH_PW_SALT_LEN);
	host_hmac_sha256(blob, AUTH_PW_SALT_LEN, (const uint8_t *)pw, strlen(pw),
			 &blob[AUTH_PW_SALT_LEN]);
}

static void test_local_verify_layout(void)
{
	const port_crypto_t *cr = host_crypto();
	uint8_t blob[AUTH_PW_BLOB_LEN];

	/* The layout constants must be what cfg key sec.admin.pw is sized for. */
	TEST_ASSERT_EQUAL_UINT(16U, AUTH_PW_SALT_LEN);
	TEST_ASSERT_EQUAL_UINT(32U, AUTH_PW_MAC_LEN);
	TEST_ASSERT_EQUAL_UINT(48U, AUTH_PW_BLOB_LEN);

	make_blob(blob, PW, 0x5AU);
	TEST_ASSERT_EQUAL_INT(0, auth_local_verify(cr, blob, sizeof(blob), PW));
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      auth_local_verify(cr, blob, sizeof(blob),
						"correct hors"));
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      auth_local_verify(cr, blob, sizeof(blob),
						"correct horses"));

	/* The salt is part of the input: the same password under a different
	 * salt is a different blob. */
	{
		uint8_t other[AUTH_PW_BLOB_LEN];

		make_blob(other, PW, 0x11U);
		TEST_ASSERT_TRUE(memcmp(blob, other, sizeof(blob)) != 0);
		TEST_ASSERT_EQUAL_INT(0, auth_local_verify(cr, other,
							  sizeof(other), PW));
	}
}

static void test_local_verify_guards(void)
{
	const port_crypto_t *cr = host_crypto();
	port_crypto_t no_hmac = *cr;
	uint8_t blob[AUTH_PW_BLOB_LEN];
	char big[AUTH_SECRET_MAX + 2U];

	make_blob(blob, PW, 0x5AU);
	memset(big, 'x', sizeof(big) - 1U);
	big[sizeof(big) - 1U] = '\0';

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      auth_local_verify(NULL, blob, sizeof(blob), PW));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      auth_local_verify(cr, blob, sizeof(blob), NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      auth_local_verify(cr, blob, sizeof(blob), ""));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      auth_local_verify(cr, blob, sizeof(blob), big));

	/* No credential provisioned is -ENOENT, not -EACCES: the difference is
	 * what lets the chain fall through to a remote backend. */
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      auth_local_verify(cr, NULL, 0U, PW));
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      auth_local_verify(cr, blob, 47U, PW));
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      auth_local_verify(cr, blob, 49U, PW));

	no_hmac.hmac_sha256 = NULL;
	TEST_ASSERT_EQUAL_INT(-ENOTSUP,
			      auth_local_verify(&no_hmac, blob, sizeof(blob),
						PW));
}

/* ========================================================================= */
/* the chain                                                                 */
/* ========================================================================= */

/* A scripted remote backend. */
typedef struct {
	int result[AUTH_BE_COUNT];  /* 0, -EACCES, -EHOSTUNREACH, ... */
	uint8_t role[AUTH_BE_COUNT];
	unsigned int calls[AUTH_BE_COUNT];
	char last_user[AUTH_USER_MAX + 1U];
	bool write_no_role;
} remote_t;

static remote_t g_remote;

static int remote_fn(void *ctx, uint8_t be, const char *user,
		     const char *secret, uint8_t *out_role)
{
	remote_t *r = (remote_t *)ctx;

	(void)secret;
	if (be >= AUTH_BE_COUNT) {
		return -EINVAL;
	}
	r->calls[be]++;
	(void)strncpy(r->last_user, user, AUTH_USER_MAX);
	r->last_user[AUTH_USER_MAX] = '\0';
	if (r->result[be] == 0 && !r->write_no_role) {
		*out_role = r->role[be];
	}
	return r->result[be];
}

static uint8_t g_blob[AUTH_PW_BLOB_LEN];

static auth_cfg_t base_cfg(void)
{
	auth_cfg_t c;
	size_t i;

	memset(&g_remote, 0, sizeof(g_remote));
	for (i = 0U; i < AUTH_BE_COUNT; i++) {
		g_remote.result[i] = -EHOSTUNREACH;
	}
	make_blob(g_blob, PW, 0x5AU);

	memset(&c, 0, sizeof(c));
	c.crypto = host_crypto();
	c.remote = remote_fn;
	c.remote_ctx = &g_remote;
	c.local_blob = g_blob;
	c.local_blob_len = sizeof(g_blob);
	return c;
}

static void test_init_guards(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();
	port_crypto_t no_hmac = *host_crypto();

	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_init(NULL, &c));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_init(&a, NULL));
	c.crypto = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_init(&a, &c));

	no_hmac.hmac_sha256 = NULL;
	c = base_cfg();
	c.crypto = &no_hmac;
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, auth_init(&a, &c));

	c = base_cfg();
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));
	/* An empty chain defaults to local-only. */
	TEST_ASSERT_EQUAL_UINT8(1U, a.cfg.n_order);
	TEST_ASSERT_EQUAL_UINT8(AUTH_BE_LOCAL, a.cfg.order[0]);
	/* And an unset local role defaults to admin. */
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_ADMIN, a.cfg.local_role);

	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_reconfigure(NULL, &c));
	{
		auth_ctx_t fresh;

		memset(&fresh, 0, sizeof(fresh));
		TEST_ASSERT_EQUAL_INT(-EINVAL, auth_reconfigure(&fresh, &c));
	}
}

static void test_order_is_normalised(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();

	/* Out-of-range and duplicate entries are dropped. */
	c.order[0] = AUTH_BE_RADIUS;
	c.order[1] = 99U;
	c.order[2] = AUTH_BE_RADIUS;
	c.order[3] = AUTH_BE_LOCAL;
	c.n_order = 4U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));
	TEST_ASSERT_EQUAL_UINT8(2U, a.cfg.n_order);
	TEST_ASSERT_EQUAL_UINT8(AUTH_BE_RADIUS, a.cfg.order[0]);
	TEST_ASSERT_EQUAL_UINT8(AUTH_BE_LOCAL, a.cfg.order[1]);

	/* An n_order beyond the array is clamped. */
	c = base_cfg();
	c.order[0] = AUTH_BE_LDAP;
	c.n_order = 200U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));
	TEST_ASSERT_TRUE(a.cfg.n_order <= AUTH_BE_COUNT);

	/* An out-of-range local role is clamped to admin. */
	c = base_cfg();
	c.local_role = 77U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_ADMIN, a.cfg.local_role);

	/* A lockout count with no duration gets a default. */
	c = base_cfg();
	c.lockout_fails = 3U;
	c.lockout_s = 0U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));
	TEST_ASSERT_EQUAL_UINT16(60U, a.cfg.lockout_s);
}

static void test_local_only_accept_and_reject(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();
	auth_out_t out;
	auth_stats_t st;

	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));

	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "admin", PW, 1000U, &out));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_ADMIN, out.role);
	TEST_ASSERT_EQUAL_UINT8(AUTH_BE_LOCAL, out.backend);
	TEST_ASSERT_FALSE(out.cached);

	/* The local credential is the one admin credential; the username is not
	 * part of it, which is the console's behaviour today. */
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "anyone", PW, 1000U, &out));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_ADMIN, out.role);

	TEST_ASSERT_EQUAL_INT(-EACCES,
			      auth_check(&a, "admin", "wrong", 1000U, &out));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_NONE, out.role);

	/* out is optional. */
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "admin", PW, 1000U, NULL));

	TEST_ASSERT_EQUAL_INT(0, auth_stats_get(&a, &st));
	TEST_ASSERT_EQUAL_UINT32(4U, st.checks);
	TEST_ASSERT_EQUAL_UINT32(3U, st.accepts);
	TEST_ASSERT_EQUAL_UINT32(1U, st.rejects);
	TEST_ASSERT_EQUAL_UINT32(3U, st.by_backend[AUTH_BE_LOCAL]);

	TEST_ASSERT_EQUAL_INT(0, auth_stats_reset(&a));
	TEST_ASSERT_EQUAL_INT(0, auth_stats_get(&a, &st));
	TEST_ASSERT_EQUAL_UINT32(0U, st.checks);
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_stats_get(NULL, &st));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_stats_get(&a, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_stats_reset(NULL));
}

static void test_check_argument_guards(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();
	auth_out_t out;
	char big_user[AUTH_USER_MAX + 2U];
	char big_secret[AUTH_SECRET_MAX + 2U];

	memset(big_user, 'u', sizeof(big_user) - 1U);
	big_user[sizeof(big_user) - 1U] = '\0';
	memset(big_secret, 's', sizeof(big_secret) - 1U);
	big_secret[sizeof(big_secret) - 1U] = '\0';

	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));

	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_check(NULL, "u", PW, 0U, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_check(&a, NULL, PW, 0U, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_check(&a, "u", NULL, 0U, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_check(&a, "", PW, 0U, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_check(&a, "u", "", 0U, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_check(&a, big_user, PW, 0U, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      auth_check(&a, "u", big_secret, 0U, &out));
	{
		auth_ctx_t fresh;

		memset(&fresh, 0, sizeof(fresh));
		TEST_ASSERT_EQUAL_INT(-EINVAL,
				      auth_check(&fresh, "u", PW, 0U, &out));
	}
}

static void test_chain_stops_on_reject_but_not_on_unavailable(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();
	auth_out_t out;
	auth_stats_t st;

	/* radius unavailable -> tacacs rejects -> ldap must never be asked. */
	c.order[0] = AUTH_BE_RADIUS;
	c.order[1] = AUTH_BE_TACACS;
	c.order[2] = AUTH_BE_LDAP;
	c.n_order = 3U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));

	g_remote.result[AUTH_BE_RADIUS] = -ETIMEDOUT;
	g_remote.result[AUTH_BE_TACACS] = -EACCES;
	g_remote.result[AUTH_BE_LDAP] = 0;
	g_remote.role[AUTH_BE_LDAP] = AUTH_ROLE_ADMIN;

	TEST_ASSERT_EQUAL_INT(-EACCES,
			      auth_check(&a, "topher", "pw", 0U, &out));
	TEST_ASSERT_EQUAL_UINT8(AUTH_BE_TACACS, out.backend);
	TEST_ASSERT_EQUAL_UINT(1U, g_remote.calls[AUTH_BE_RADIUS]);
	TEST_ASSERT_EQUAL_UINT(1U, g_remote.calls[AUTH_BE_TACACS]);
	TEST_ASSERT_EQUAL_UINT(0U, g_remote.calls[AUTH_BE_LDAP]);
	TEST_ASSERT_EQUAL_STRING("topher", g_remote.last_user);

	/* Now make tacacs unavailable too: ldap gets its turn and accepts. */
	g_remote.result[AUTH_BE_TACACS] = -EHOSTUNREACH;
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "topher", "pw", 0U, &out));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_ADMIN, out.role);
	TEST_ASSERT_EQUAL_UINT8(AUTH_BE_LDAP, out.backend);

	TEST_ASSERT_EQUAL_INT(0, auth_stats_get(&a, &st));
	TEST_ASSERT_EQUAL_UINT32(1U, st.by_backend[AUTH_BE_LDAP]);
}

static void test_every_backend_unavailable_is_a_denial(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();
	auth_out_t out;
	auth_stats_t st;

	/* No local credential and every remote down. */
	c.local_blob = NULL;
	c.local_blob_len = 0U;
	c.order[0] = AUTH_BE_LOCAL;
	c.order[1] = AUTH_BE_RADIUS;
	c.n_order = 2U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));

	TEST_ASSERT_EQUAL_INT(-EHOSTUNREACH,
			      auth_check(&a, "u", "p", 0U, &out));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_NONE, out.role);
	TEST_ASSERT_EQUAL_INT(0, auth_stats_get(&a, &st));
	TEST_ASSERT_EQUAL_UINT32(1U, st.unavailable);

	/* A NULL remote hook makes every remote backend unavailable. */
	c = base_cfg();
	c.remote = NULL;
	c.local_blob = NULL;
	c.local_blob_len = 0U;
	c.order[0] = AUTH_BE_TACACS;
	c.n_order = 1U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));
	TEST_ASSERT_EQUAL_INT(-EHOSTUNREACH,
			      auth_check(&a, "u", "p", 0U, &out));
}

static void test_accepting_backend_with_no_role_gets_least_privilege(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();
	auth_out_t out;

	c.order[0] = AUTH_BE_RADIUS;
	c.n_order = 1U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));

	g_remote.result[AUTH_BE_RADIUS] = 0;
	g_remote.write_no_role = true;
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "u", "p", 0U, &out));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER, out.role);

	/* And so does one that returns nonsense. */
	g_remote.write_no_role = false;
	g_remote.role[AUTH_BE_RADIUS] = 200U;
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "u2", "p", 0U, &out));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER, out.role);
}

/* ========================================================================= */
/* cache                                                                     */
/* ========================================================================= */

static void test_cache_hit_and_expiry(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();
	auth_out_t out;
	auth_stats_t st;

	c.order[0] = AUTH_BE_RADIUS;
	c.n_order = 1U;
	c.cache_ttl_s = 30U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));

	g_remote.result[AUTH_BE_RADIUS] = 0;
	g_remote.role[AUTH_BE_RADIUS] = AUTH_ROLE_OPERATOR;

	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "u", "p", 1000U, &out));
	TEST_ASSERT_FALSE(out.cached);
	TEST_ASSERT_EQUAL_UINT(1U, g_remote.calls[AUTH_BE_RADIUS]);

	/* Inside the TTL: no backend call, same role, and flagged cached. */
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "u", "p", 20000U, &out));
	TEST_ASSERT_TRUE(out.cached);
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_OPERATOR, out.role);
	TEST_ASSERT_EQUAL_UINT8(AUTH_BE_RADIUS, out.backend);
	TEST_ASSERT_EQUAL_UINT(1U, g_remote.calls[AUTH_BE_RADIUS]);

	/* A different secret is a different cache key. */
	g_remote.result[AUTH_BE_RADIUS] = -EACCES;
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      auth_check(&a, "u", "other", 20000U, &out));
	TEST_ASSERT_EQUAL_UINT(2U, g_remote.calls[AUTH_BE_RADIUS]);
	/* And a different user is too. */
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      auth_check(&a, "u2", "p", 20000U, &out));
	TEST_ASSERT_EQUAL_UINT(3U, g_remote.calls[AUTH_BE_RADIUS]);

	/* Past the TTL the backend is consulted again. */
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      auth_check(&a, "u", "p", 40000U, &out));
	TEST_ASSERT_EQUAL_UINT(4U, g_remote.calls[AUTH_BE_RADIUS]);

	TEST_ASSERT_EQUAL_INT(0, auth_stats_get(&a, &st));
	TEST_ASSERT_EQUAL_UINT32(1U, st.cache_hits);
	TEST_ASSERT_EQUAL_UINT32(1U, st.cache_fills);
}

static void test_cache_is_not_poisoned_by_rejections(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();
	auth_out_t out;

	c.order[0] = AUTH_BE_RADIUS;
	c.n_order = 1U;
	c.cache_ttl_s = 300U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));

	/* A rejection must not be remembered: a password reset would otherwise
	 * keep failing for the whole TTL. */
	g_remote.result[AUTH_BE_RADIUS] = -EACCES;
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_check(&a, "u", "p", 0U, &out));
	g_remote.result[AUTH_BE_RADIUS] = 0;
	g_remote.role[AUTH_BE_RADIUS] = AUTH_ROLE_ADMIN;
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "u", "p", 100U, &out));
	TEST_ASSERT_FALSE(out.cached);
}

static void test_cache_flush_and_eviction(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();
	auth_out_t out;
	char user[16];
	unsigned int i;

	c.order[0] = AUTH_BE_RADIUS;
	c.n_order = 1U;
	c.cache_ttl_s = 600U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));
	g_remote.result[AUTH_BE_RADIUS] = 0;
	g_remote.role[AUTH_BE_RADIUS] = AUTH_ROLE_VIEWER;

	/* Fill past capacity: the oldest entries are evicted, nothing breaks. */
	for (i = 0U; i < (AUTH_CACHE_SLOTS * 2U); i++) {
		(void)snprintf(user, sizeof(user), "u%u", i);
		TEST_ASSERT_EQUAL_INT(0, auth_check(&a, user, "p",
						    1000U + (i * 10U), &out));
	}
	/* The most recent is still cached. */
	(void)snprintf(user, sizeof(user), "u%u", (AUTH_CACHE_SLOTS * 2U) - 1U);
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, user, "p", 2000U, &out));
	TEST_ASSERT_TRUE(out.cached);

	/* A flush drops everything. */
	auth_cache_flush(&a, "u1");
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, user, "p", 2000U, &out));
	TEST_ASSERT_FALSE(out.cached);
	auth_cache_flush(&a, NULL);
	auth_cache_flush(NULL, NULL);
}

static void test_cache_disabled_without_a_csprng(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();
	port_crypto_t no_rand = *host_crypto();
	auth_out_t out;

	/* No salt source means no cache, rather than a predictable cache key. */
	no_rand.rand = NULL;
	c.crypto = &no_rand;
	c.order[0] = AUTH_BE_RADIUS;
	c.n_order = 1U;
	c.cache_ttl_s = 300U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));
	TEST_ASSERT_FALSE(a.cache_ready);

	g_remote.result[AUTH_BE_RADIUS] = 0;
	g_remote.role[AUTH_BE_RADIUS] = AUTH_ROLE_ADMIN;
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "u", "p", 0U, &out));
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "u", "p", 10U, &out));
	TEST_ASSERT_FALSE(out.cached);
	TEST_ASSERT_EQUAL_UINT(2U, g_remote.calls[AUTH_BE_RADIUS]);
}

/* ========================================================================= */
/* lockout                                                                   */
/* ========================================================================= */

static void test_lockout_after_n_failures(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();
	auth_out_t out;
	auth_stats_t st;
	unsigned int i;

	c.lockout_fails = 3U;
	c.lockout_s = 60U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));

	for (i = 0U; i < 3U; i++) {
		TEST_ASSERT_EQUAL_INT(-EACCES,
				      auth_check(&a, "admin", "bad", 1000U,
						 &out));
	}
	TEST_ASSERT_EQUAL_UINT32(60U, out.retry_after_s);
	TEST_ASSERT_TRUE(auth_is_locked(&a, "admin", 1000U));

	/* Locked out: even the correct password is refused, and no backend is
	 * consulted — that is what makes a brute-force attempt cheap for us. */
	TEST_ASSERT_EQUAL_INT(-EBUSY,
			      auth_check(&a, "admin", PW, 2000U, &out));
	TEST_ASSERT_EQUAL_UINT32(59U, out.retry_after_s);

	/* A different user is unaffected. */
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "other", PW, 2000U, &out));

	/* After the window it works again. */
	TEST_ASSERT_FALSE(auth_is_locked(&a, "admin", 62000U));
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "admin", PW, 62000U, &out));

	TEST_ASSERT_EQUAL_INT(0, auth_stats_get(&a, &st));
	TEST_ASSERT_EQUAL_UINT32(1U, st.locked);

	TEST_ASSERT_FALSE(auth_is_locked(&a, "nobody", 0U));
	TEST_ASSERT_FALSE(auth_is_locked(NULL, "admin", 0U));
	TEST_ASSERT_FALSE(auth_is_locked(&a, NULL, 0U));
}

static void test_lockout_is_shared_by_every_backend(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();
	auth_out_t out;

	/* Two failures against RADIUS and one against the local credential must
	 * add up to a lockout — a per-backend counter would let an attacker get
	 * N tries per backend. */
	c.order[0] = AUTH_BE_RADIUS;
	c.order[1] = AUTH_BE_LOCAL;
	c.n_order = 2U;
	c.lockout_fails = 3U;
	c.lockout_s = 30U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));

	g_remote.result[AUTH_BE_RADIUS] = -EHOSTUNREACH;
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_check(&a, "u", "bad1", 0U, &out));
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_check(&a, "u", "bad2", 0U, &out));
	g_remote.result[AUTH_BE_RADIUS] = -EACCES;
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_check(&a, "u", "bad3", 0U, &out));
	TEST_ASSERT_EQUAL_INT(-EBUSY, auth_check(&a, "u", PW, 0U, &out));
}

static void test_success_resets_the_failure_count(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();
	auth_out_t out;

	c.lockout_fails = 3U;
	c.lockout_s = 30U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));

	TEST_ASSERT_EQUAL_INT(-EACCES, auth_check(&a, "u", "bad", 0U, &out));
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_check(&a, "u", "bad", 0U, &out));
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "u", PW, 0U, &out));
	/* The count is back to zero, so two more failures do not lock. */
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_check(&a, "u", "bad", 0U, &out));
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_check(&a, "u", "bad", 0U, &out));
	TEST_ASSERT_FALSE(auth_is_locked(&a, "u", 0U));
}

static void test_unlock_and_lock_table_pressure(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();
	auth_out_t out;
	char user[16];
	unsigned int i;

	c.lockout_fails = 1U;
	c.lockout_s = 600U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));

	/* Lock every slot, then try to lock one more. The extra user gets no
	 * tracking rather than evicting somebody else's live lock. */
	for (i = 0U; i < AUTH_LOCK_SLOTS; i++) {
		(void)snprintf(user, sizeof(user), "u%u", i);
		TEST_ASSERT_EQUAL_INT(-EACCES,
				      auth_check(&a, user, "bad", 1000U, &out));
		TEST_ASSERT_TRUE(auth_is_locked(&a, user, 1000U));
	}
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      auth_check(&a, "extra", "bad", 1000U, &out));
	/* Every earlier lock survives. */
	for (i = 0U; i < AUTH_LOCK_SLOTS; i++) {
		(void)snprintf(user, sizeof(user), "u%u", i);
		TEST_ASSERT_TRUE(auth_is_locked(&a, user, 1000U));
	}

	TEST_ASSERT_EQUAL_INT(0, auth_unlock(&a, "u0"));
	TEST_ASSERT_FALSE(auth_is_locked(&a, "u0", 1000U));
	TEST_ASSERT_EQUAL_INT(-ENOENT, auth_unlock(&a, "nobody"));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_unlock(NULL, "u0"));
	TEST_ASSERT_EQUAL_INT(-EINVAL, auth_unlock(&a, NULL));

	/* Once the locks expire the slots are reusable. */
	(void)snprintf(user, sizeof(user), "later");
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      auth_check(&a, user, "bad", 700000U, &out));
	TEST_ASSERT_TRUE(auth_is_locked(&a, user, 700000U));
}

static void test_reconfigure_keeps_lockout_drops_cache(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();
	auth_out_t out;

	c.order[0] = AUTH_BE_RADIUS;
	c.n_order = 1U;
	c.cache_ttl_s = 600U;
	c.lockout_fails = 2U;
	c.lockout_s = 300U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));

	g_remote.result[AUTH_BE_RADIUS] = 0;
	g_remote.role[AUTH_BE_RADIUS] = AUTH_ROLE_ADMIN;
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "good", "p", 1000U, &out));
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "good", "p", 1000U, &out));
	TEST_ASSERT_TRUE(out.cached);

	g_remote.result[AUTH_BE_RADIUS] = -EACCES;
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_check(&a, "bad", "p", 1000U, &out));
	TEST_ASSERT_EQUAL_INT(-EACCES, auth_check(&a, "bad", "p", 1000U, &out));
	TEST_ASSERT_TRUE(auth_is_locked(&a, "bad", 1000U));

	/* A cfg commit must not be a lockout bypass... */
	TEST_ASSERT_EQUAL_INT(0, auth_reconfigure(&a, &c));
	TEST_ASSERT_TRUE(auth_is_locked(&a, "bad", 1000U));
	/* ...but it does invalidate cached decisions. */
	g_remote.result[AUTH_BE_RADIUS] = 0;
	TEST_ASSERT_EQUAL_INT(0, auth_check(&a, "good", "p", 1000U, &out));
	TEST_ASSERT_FALSE(out.cached);
}

/* ========================================================================= */
/* LDAP — result codes                                                       */
/* ========================================================================= */

static void test_ldap_result_names_and_mapping(void)
{
	TEST_ASSERT_EQUAL_STRING("success", ldap_result_name(LDAP_RES_SUCCESS));
	TEST_ASSERT_EQUAL_STRING("operationsError",
				 ldap_result_name(LDAP_RES_OPERATIONS_ERROR));
	TEST_ASSERT_EQUAL_STRING("protocolError",
				 ldap_result_name(LDAP_RES_PROTOCOL_ERROR));
	TEST_ASSERT_EQUAL_STRING("timeLimitExceeded",
				 ldap_result_name(LDAP_RES_TIME_LIMIT_EXCEEDED));
	TEST_ASSERT_EQUAL_STRING("sizeLimitExceeded",
				 ldap_result_name(LDAP_RES_SIZE_LIMIT_EXCEEDED));
	TEST_ASSERT_EQUAL_STRING(
		"authMethodNotSupported",
		ldap_result_name(LDAP_RES_AUTH_METHOD_NOT_SUPPORTED));
	TEST_ASSERT_EQUAL_STRING(
		"strongerAuthRequired",
		ldap_result_name(LDAP_RES_STRONGER_AUTH_REQUIRED));
	TEST_ASSERT_EQUAL_STRING("referral",
				 ldap_result_name(LDAP_RES_REFERRAL));
	TEST_ASSERT_EQUAL_STRING("noSuchObject",
				 ldap_result_name(LDAP_RES_NO_SUCH_OBJECT));
	TEST_ASSERT_EQUAL_STRING("invalidDNSyntax",
				 ldap_result_name(LDAP_RES_INVALID_DN_SYNTAX));
	TEST_ASSERT_EQUAL_STRING(
		"inappropriateAuthentication",
		ldap_result_name(LDAP_RES_INAPPROPRIATE_AUTH));
	TEST_ASSERT_EQUAL_STRING("invalidCredentials",
				 ldap_result_name(LDAP_RES_INVALID_CREDENTIALS));
	TEST_ASSERT_EQUAL_STRING(
		"insufficientAccessRights",
		ldap_result_name(LDAP_RES_INSUFFICIENT_ACCESS));
	TEST_ASSERT_EQUAL_STRING("busy", ldap_result_name(LDAP_RES_BUSY));
	TEST_ASSERT_EQUAL_STRING("unavailable",
				 ldap_result_name(LDAP_RES_UNAVAILABLE));
	TEST_ASSERT_EQUAL_STRING("unwillingToPerform",
				 ldap_result_name(LDAP_RES_UNWILLING_TO_PERFORM));
	TEST_ASSERT_EQUAL_STRING("other", ldap_result_name(1234));

	/* The mapping is what decides "deny" versus "try the next backend". */
	TEST_ASSERT_EQUAL_INT(0, ldap_result_to_errno(LDAP_RES_SUCCESS));
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      ldap_result_to_errno(LDAP_RES_INVALID_CREDENTIALS));
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      ldap_result_to_errno(LDAP_RES_INAPPROPRIATE_AUTH));
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      ldap_result_to_errno(LDAP_RES_INSUFFICIENT_ACCESS));
	TEST_ASSERT_EQUAL_INT(-EHOSTUNREACH,
			      ldap_result_to_errno(LDAP_RES_BUSY));
	TEST_ASSERT_EQUAL_INT(-EHOSTUNREACH,
			      ldap_result_to_errno(LDAP_RES_UNAVAILABLE));
	TEST_ASSERT_EQUAL_INT(-EHOSTUNREACH,
			      ldap_result_to_errno(LDAP_RES_UNWILLING_TO_PERFORM));
	TEST_ASSERT_EQUAL_INT(-EHOSTUNREACH,
			      ldap_result_to_errno(LDAP_RES_REFERRAL));
	TEST_ASSERT_EQUAL_INT(-EPROTO,
			      ldap_result_to_errno(LDAP_RES_NO_SUCH_OBJECT));
}

/* ========================================================================= */
/* LDAP — builders                                                           */
/* ========================================================================= */

static void test_ldap_bind_known_answer(void)
{
	/* RFC 4511 §4.2 BindRequest for cn=svc / "pw", messageID 1:
	 *   30 14                          LDAPMessage SEQUENCE, 20 content
	 *      02 01 01                    messageID 1
	 *      60 0F                       [APPLICATION 0] BindRequest, 15
	 *         02 01 03                 version 3
	 *         04 06 63 6e 3d 73 76 63  name "cn=svc"
	 *         80 02 70 77              [0] simple "pw"
	 */
	static const uint8_t want[] = {
		0x30, 0x14, 0x02, 0x01, 0x01, 0x60, 0x0f, 0x02, 0x01, 0x03,
		0x04, 0x06, 0x63, 0x6e, 0x3d, 0x73, 0x76, 0x63, 0x80, 0x02,
		0x70, 0x77
	};
	uint8_t out[LDAP_BUF_MAX];
	size_t len = 0U;
	uint32_t id = 0U;
	uint8_t op = 0U;

	TEST_ASSERT_EQUAL_INT(0, ldap_build_bind_simple(1U, "cn=svc", "pw", out,
							sizeof(out), &len));
	TEST_ASSERT_EQUAL_UINT(sizeof(want), len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want, out, sizeof(want));

	TEST_ASSERT_EQUAL_INT((int)len, ldap_msg_len(out, len));
	TEST_ASSERT_EQUAL_INT(0, ldap_msg_peek(out, len, &id, &op));
	TEST_ASSERT_EQUAL_UINT32(1U, id);
	TEST_ASSERT_EQUAL_HEX8(LDAP_OP_BIND_REQUEST, op);

	/* An anonymous bind is an empty name and an empty password. */
	TEST_ASSERT_EQUAL_INT(0, ldap_build_bind_simple(2U, "", "", out,
							sizeof(out), &len));
	TEST_ASSERT_EQUAL_HEX8(0x04U, out[10]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, out[11]);
	TEST_ASSERT_EQUAL_HEX8(0x80U, out[12]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, out[13]);
}

static void test_ldap_bind_refuses_unauthenticated_bind(void)
{
	uint8_t out[LDAP_BUF_MAX];
	char big[LDAP_DN_MAX + 2U];
	size_t len = 0U;

	memset(big, 'd', sizeof(big) - 1U);
	big[sizeof(big) - 1U] = '\0';

	/* RFC 4513 §5.1.2: a name with an empty password is an unauthenticated
	 * bind that many servers answer with success. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ldap_build_bind_simple(1U, "cn=svc", "", out,
						     sizeof(out), &len));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ldap_build_bind_simple(1U, NULL, "pw", out,
						     sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ldap_build_bind_simple(1U, "cn=svc", NULL, out,
						     sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ldap_build_bind_simple(1U, "cn=svc", "pw", NULL,
						     sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ldap_build_bind_simple(1U, "cn=svc", "pw", out,
						     sizeof(out), NULL));
	/* messageID 0 is reserved for unsolicited notifications. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ldap_build_bind_simple(0U, "cn=svc", "pw", out,
						     sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ldap_build_bind_simple(1U, big, "pw", out,
						     sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      ldap_build_bind_simple(1U, "cn=svc", "pw", out,
						     8U, &len));
}

static void test_ldap_unbind_and_starttls(void)
{
	/* UnbindRequest is [APPLICATION 2] NULL — primitive, zero length. */
	static const uint8_t want_unbind[] = { 0x30, 0x05, 0x02, 0x01,
					      0x07, 0x42, 0x00 };
	uint8_t out[LDAP_BUF_MAX];
	size_t len = 0U;
	uint32_t id = 0U;
	uint8_t op = 0U;

	TEST_ASSERT_EQUAL_INT(0, ldap_build_unbind(7U, out, sizeof(out), &len));
	TEST_ASSERT_EQUAL_UINT(sizeof(want_unbind), len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want_unbind, out, sizeof(want_unbind));
	/* The zero-length primitive operation is the one envelope shape a
	 * length-driven peek can mistake for a truncated message. */
	TEST_ASSERT_EQUAL_INT(0, ldap_msg_peek(out, len, &id, &op));
	TEST_ASSERT_EQUAL_UINT32(7U, id);
	TEST_ASSERT_EQUAL_HEX8(LDAP_OP_UNBIND_REQUEST, op);

	TEST_ASSERT_EQUAL_INT(0, ldap_build_starttls(3U, out, sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(0, ldap_msg_peek(out, len, &id, &op));
	TEST_ASSERT_EQUAL_UINT32(3U, id);
	TEST_ASSERT_EQUAL_HEX8(LDAP_OP_EXTENDED_REQUEST, op);
	/* requestName is the [0] context-primitive form, not an OCTET STRING. */
	TEST_ASSERT_EQUAL_HEX8(LDAP_EXT_REQUEST_NAME, out[7]);
	TEST_ASSERT_EQUAL_MEMORY(LDAP_OID_STARTTLS, &out[9],
				 strlen(LDAP_OID_STARTTLS));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_unbind(1U, NULL, 16U, &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_unbind(1U, out, 16U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_unbind(0U, out, 16U, &len));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ldap_build_unbind(1U, out, 3U, &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_starttls(1U, NULL, 64U, &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_starttls(1U, out, 64U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_starttls(0U, out, 64U, &len));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ldap_build_starttls(1U, out, 10U, &len));
}

static ldap_search_req_t base_search(void)
{
	ldap_search_req_t s;

	memset(&s, 0, sizeof(s));
	s.base = "ou=groups,dc=example,dc=com";
	s.scope = LDAP_SCOPE_SUBTREE;
	s.size_limit = 8U;
	s.time_limit = 5U;
	s.attr = "member";
	s.value = "uid=topher,ou=people,dc=example,dc=com";
	return s;
}

static void test_ldap_search_layout(void)
{
	ldap_search_req_t s = base_search();
	uint8_t out[LDAP_BUF_MAX];
	size_t len = 0U;
	uint32_t id = 0U;
	uint8_t op = 0U;
	size_t off;

	TEST_ASSERT_EQUAL_INT(0, ldap_build_search(9U, &s, out, sizeof(out),
						   &len));
	TEST_ASSERT_EQUAL_INT(0, ldap_msg_peek(out, len, &id, &op));
	TEST_ASSERT_EQUAL_UINT32(9U, id);
	TEST_ASSERT_EQUAL_HEX8(LDAP_OP_SEARCH_REQUEST, op);

	/* SearchRequest fields in order (RFC 4511 §4.5.1): baseObject,
	 * scope ENUMERATED, derefAliases ENUMERATED, sizeLimit, timeLimit,
	 * typesOnly BOOLEAN, filter, attributes. */
	off = 5U;                 /* past 30 LL 02 01 09 */
	TEST_ASSERT_EQUAL_HEX8(LDAP_OP_SEARCH_REQUEST, out[off]);
	off += 2U;                /* the op header is short-form here */
	TEST_ASSERT_EQUAL_HEX8(LDAP_T_OCTET_STRING, out[off]);
	off += 2U + strlen(s.base);
	TEST_ASSERT_EQUAL_HEX8(LDAP_T_ENUMERATED, out[off]);
	TEST_ASSERT_EQUAL_HEX8(LDAP_SCOPE_SUBTREE, out[off + 2U]);
	off += 3U;
	TEST_ASSERT_EQUAL_HEX8(LDAP_T_ENUMERATED, out[off]);
	TEST_ASSERT_EQUAL_HEX8(LDAP_DEREF_NEVER, out[off + 2U]);
	off += 3U;
	TEST_ASSERT_EQUAL_HEX8(LDAP_T_INTEGER, out[off]);
	TEST_ASSERT_EQUAL_HEX8(8U, out[off + 2U]);
	off += 3U;
	TEST_ASSERT_EQUAL_HEX8(LDAP_T_INTEGER, out[off]);
	TEST_ASSERT_EQUAL_HEX8(5U, out[off + 2U]);
	off += 3U;
	TEST_ASSERT_EQUAL_HEX8(LDAP_T_BOOLEAN, out[off]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, out[off + 2U]);
	off += 3U;
	/* A single filter term is an equalityMatch, [3] constructed. */
	TEST_ASSERT_EQUAL_HEX8(LDAP_FILTER_EQUALITY, out[off]);
}

static void test_ldap_search_and_filter(void)
{
	ldap_search_req_t s = base_search();
	uint8_t out[LDAP_BUF_MAX];
	size_t len = 0U;

	/* Two terms become an `and` filter, [0] constructed. */
	s.attr2 = "objectClass";
	s.value2 = "groupOfNames";
	s.want_attr = "cn";
	TEST_ASSERT_EQUAL_INT(0, ldap_build_search(1U, &s, out, sizeof(out),
						   &len));
	{
		size_t i;
		bool found_and = false;

		for (i = 0U; i + 1U < len; i++) {
			if (out[i] == LDAP_FILTER_AND) {
				found_and = true;
			}
		}
		TEST_ASSERT_TRUE(found_and);
	}
	/* The requested attribute is the last string in the message. */
	TEST_ASSERT_EQUAL_MEMORY("cn", &out[len - 2U], 2U);

	/* A base-scoped membership check with no attributes requested. */
	s = base_search();
	s.base = "cn=admins,ou=groups,dc=example,dc=com";
	s.scope = LDAP_SCOPE_BASE;
	s.want_attr = NULL;
	TEST_ASSERT_EQUAL_INT(0, ldap_build_search(2U, &s, out, sizeof(out),
						   &len));
	/* An empty attribute list is a zero-length SEQUENCE at the end. */
	TEST_ASSERT_EQUAL_HEX8(LDAP_T_SEQUENCE, out[len - 2U]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, out[len - 1U]);

	s.scope = LDAP_SCOPE_ONELEVEL;
	TEST_ASSERT_EQUAL_INT(0, ldap_build_search(3U, &s, out, sizeof(out),
						   &len));
	s.want_attr = "";
	TEST_ASSERT_EQUAL_INT(0, ldap_build_search(4U, &s, out, sizeof(out),
						   &len));
}

static void test_ldap_search_guards(void)
{
	ldap_search_req_t s;
	uint8_t out[LDAP_BUF_MAX];
	char big[LDAP_DN_MAX + 2U];
	char bigattr[LDAP_ATTR_MAX + 2U];
	size_t len = 0U;

	memset(big, 'x', sizeof(big) - 1U);
	big[sizeof(big) - 1U] = '\0';
	memset(bigattr, 'a', sizeof(bigattr) - 1U);
	bigattr[sizeof(bigattr) - 1U] = '\0';

	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, NULL, out,
							 sizeof(out), &len));
	s = base_search();
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, NULL,
							 sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, out,
							 sizeof(out), NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(0U, &s, out,
							 sizeof(out), &len));

	s.base = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, out,
							 sizeof(out), &len));
	s = base_search();
	s.attr = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, out,
							 sizeof(out), &len));
	s = base_search();
	s.value = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, out,
							 sizeof(out), &len));
	s = base_search();
	s.attr = "";
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, out,
							 sizeof(out), &len));
	s = base_search();
	s.value = "";
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, out,
							 sizeof(out), &len));
	s = base_search();
	s.scope = 3U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, out,
							 sizeof(out), &len));
	s = base_search();
	s.base = big;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, out,
							 sizeof(out), &len));
	s = base_search();
	s.attr = bigattr;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, out,
							 sizeof(out), &len));
	s = base_search();
	s.value = big;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, out,
							 sizeof(out), &len));
	s = base_search();
	s.want_attr = bigattr;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, out,
							 sizeof(out), &len));

	/* A second filter term needs both halves. */
	s = base_search();
	s.attr2 = "objectClass";
	s.value2 = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, out,
							 sizeof(out), &len));
	s.value2 = "";
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, out,
							 sizeof(out), &len));
	s.attr2 = "";
	s.value2 = "x";
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, out,
							 sizeof(out), &len));
	s.attr2 = bigattr;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, out,
							 sizeof(out), &len));
	s.attr2 = "objectClass";
	s.value2 = big;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_build_search(1U, &s, out,
							 sizeof(out), &len));

	s = base_search();
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ldap_build_search(1U, &s, out, 20U,
							 &len));
}

/* ========================================================================= */
/* LDAP — parsers                                                            */
/* ========================================================================= */

/* Minimal independent BER writer, so the parser is not fed its own encoder. */
typedef struct {
	uint8_t buf[LDAP_BUF_MAX];
	size_t len;
} bw_t;

static void bw_reset(bw_t *w)
{
	w->len = 0U;
}

static void bw_tlv(bw_t *w, uint8_t tag, const void *val, size_t n)
{
	w->buf[w->len++] = tag;
	if (n < 0x80U) {
		w->buf[w->len++] = (uint8_t)n;
	} else if (n <= 0xFFU) {
		w->buf[w->len++] = 0x81U;
		w->buf[w->len++] = (uint8_t)n;
	} else {
		w->buf[w->len++] = 0x82U;
		w->buf[w->len++] = (uint8_t)((n >> 8) & 0xFFU);
		w->buf[w->len++] = (uint8_t)(n & 0xFFU);
	}
	if (n != 0U) {
		memcpy(&w->buf[w->len], val, n);
		w->len += n;
	}
}

static void bw_str(bw_t *w, uint8_t tag, const char *s)
{
	bw_tlv(w, tag, s, strlen(s));
}

static void bw_u8(bw_t *w, uint8_t tag, uint8_t v)
{
	bw_tlv(w, tag, &v, 1U);
}

/** Wrap @p inner in `SEQUENCE { INTEGER msgid, <tag> inner }`. */
static size_t bw_envelope(uint8_t *out, uint32_t msgid, uint8_t tag,
			  const uint8_t *inner, size_t inner_len)
{
	bw_t body;
	bw_t msg;

	bw_reset(&body);
	bw_u8(&body, LDAP_T_INTEGER, (uint8_t)msgid);
	bw_tlv(&body, tag, inner, inner_len);

	bw_reset(&msg);
	bw_tlv(&msg, LDAP_T_SEQUENCE, body.buf, body.len);
	memcpy(out, msg.buf, msg.len);
	return msg.len;
}

static void test_ldap_parse_result(void)
{
	bw_t inner;
	uint8_t msg[LDAP_BUF_MAX];
	ldap_result_t r;
	size_t len;

	bw_reset(&inner);
	bw_u8(&inner, LDAP_T_ENUMERATED, LDAP_RES_INVALID_CREDENTIALS);
	bw_str(&inner, LDAP_T_OCTET_STRING, "");
	bw_str(&inner, LDAP_T_OCTET_STRING, "80090308: bad password");
	len = bw_envelope(msg, 4U, LDAP_OP_BIND_RESPONSE, inner.buf, inner.len);

	TEST_ASSERT_EQUAL_INT(0, ldap_parse_result(msg, len,
						   LDAP_OP_BIND_RESPONSE, &r));
	TEST_ASSERT_EQUAL_UINT32(4U, r.msgid);
	TEST_ASSERT_EQUAL_HEX8(LDAP_OP_BIND_RESPONSE, r.op);
	TEST_ASSERT_EQUAL_INT32(LDAP_RES_INVALID_CREDENTIALS, r.code);
	TEST_ASSERT_EQUAL_STRING("", r.matched_dn);
	TEST_ASSERT_EQUAL_STRING("80090308: bad password", r.diagnostic);
	TEST_ASSERT_FALSE(r.truncated);

	/* want_op 0 accepts any result-bearing operation. */
	TEST_ASSERT_EQUAL_INT(0, ldap_parse_result(msg, len, 0U, &r));
	/* A specific mismatch is -ENOMSG, not a parse failure. */
	TEST_ASSERT_EQUAL_INT(-ENOMSG,
			      ldap_parse_result(msg, len, LDAP_OP_SEARCH_DONE,
						&r));

	/* SearchResultDone and ExtendedResponse use the same shape. */
	len = bw_envelope(msg, 5U, LDAP_OP_SEARCH_DONE, inner.buf, inner.len);
	TEST_ASSERT_EQUAL_INT(0, ldap_parse_result(msg, len, 0U, &r));
	len = bw_envelope(msg, 6U, LDAP_OP_EXTENDED_RESPONSE, inner.buf,
			  inner.len);
	TEST_ASSERT_EQUAL_INT(0, ldap_parse_result(msg, len, 0U, &r));

	/* An operation that carries no LDAPResult. */
	len = bw_envelope(msg, 7U, LDAP_OP_SEARCH_ENTRY, inner.buf, inner.len);
	TEST_ASSERT_EQUAL_INT(-ENOMSG, ldap_parse_result(msg, len, 0U, &r));
}

static void test_ldap_parse_result_truncation_and_errors(void)
{
	bw_t inner;
	uint8_t msg[LDAP_BUF_MAX];
	char big[LDAP_DIAG_MAX + 40U];
	ldap_result_t r;
	size_t len;

	memset(big, 'D', sizeof(big) - 1U);
	big[sizeof(big) - 1U] = '\0';

	bw_reset(&inner);
	bw_u8(&inner, LDAP_T_ENUMERATED, LDAP_RES_SUCCESS);
	bw_str(&inner, LDAP_T_OCTET_STRING, "");
	bw_str(&inner, LDAP_T_OCTET_STRING, big);
	len = bw_envelope(msg, 1U, LDAP_OP_BIND_RESPONSE, inner.buf, inner.len);
	TEST_ASSERT_EQUAL_INT(0, ldap_parse_result(msg, len, 0U, &r));
	TEST_ASSERT_TRUE(r.truncated);
	TEST_ASSERT_EQUAL_UINT(LDAP_DIAG_MAX, strlen(r.diagnostic));

	/* Non-printable octets are replaced, so a server cannot inject control
	 * characters into a log line. */
	bw_reset(&inner);
	bw_u8(&inner, LDAP_T_ENUMERATED, LDAP_RES_SUCCESS);
	bw_str(&inner, LDAP_T_OCTET_STRING, "");
	bw_tlv(&inner, LDAP_T_OCTET_STRING, "a\033[2Jb", 6U);
	len = bw_envelope(msg, 1U, LDAP_OP_BIND_RESPONSE, inner.buf, inner.len);
	TEST_ASSERT_EQUAL_INT(0, ldap_parse_result(msg, len, 0U, &r));
	TEST_ASSERT_EQUAL_STRING("a.[2Jb", r.diagnostic);

	/* Missing components. */
	bw_reset(&inner);
	bw_u8(&inner, LDAP_T_ENUMERATED, LDAP_RES_SUCCESS);
	len = bw_envelope(msg, 1U, LDAP_OP_BIND_RESPONSE, inner.buf, inner.len);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ldap_parse_result(msg, len, 0U, &r));

	bw_reset(&inner);
	bw_u8(&inner, LDAP_T_ENUMERATED, LDAP_RES_SUCCESS);
	bw_str(&inner, LDAP_T_OCTET_STRING, "");
	len = bw_envelope(msg, 1U, LDAP_OP_BIND_RESPONSE, inner.buf, inner.len);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ldap_parse_result(msg, len, 0U, &r));

	/* A resultCode that is not an ENUMERATED. */
	bw_reset(&inner);
	bw_u8(&inner, LDAP_T_INTEGER, LDAP_RES_SUCCESS);
	bw_str(&inner, LDAP_T_OCTET_STRING, "");
	bw_str(&inner, LDAP_T_OCTET_STRING, "");
	len = bw_envelope(msg, 1U, LDAP_OP_BIND_RESPONSE, inner.buf, inner.len);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ldap_parse_result(msg, len, 0U, &r));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_parse_result(NULL, len, 0U, &r));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_parse_result(msg, len, 0U, NULL));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, ldap_parse_result(msg, 1U, 0U, &r));
}

/** Build a SearchResultEntry with one attribute and up to two values. */
static size_t make_entry(uint8_t *out, uint32_t msgid, const char *dn,
			 const char *attr, const char *v1, const char *v2)
{
	bw_t vals;
	bw_t one;
	bw_t attrs;
	bw_t inner;

	bw_reset(&vals);
	if (v1 != NULL) {
		bw_str(&vals, LDAP_T_OCTET_STRING, v1);
	}
	if (v2 != NULL) {
		bw_str(&vals, LDAP_T_OCTET_STRING, v2);
	}

	bw_reset(&one);
	bw_str(&one, LDAP_T_OCTET_STRING, attr);
	bw_tlv(&one, LDAP_T_SET, vals.buf, vals.len);

	bw_reset(&attrs);
	bw_tlv(&attrs, LDAP_T_SEQUENCE, one.buf, one.len);

	bw_reset(&inner);
	bw_str(&inner, LDAP_T_OCTET_STRING, dn);
	bw_tlv(&inner, LDAP_T_SEQUENCE, attrs.buf, attrs.len);

	return bw_envelope(out, msgid, LDAP_OP_SEARCH_ENTRY, inner.buf,
			   inner.len);
}

static void test_ldap_parse_search_entry(void)
{
	uint8_t msg[LDAP_BUF_MAX];
	ldap_entry_t e;
	size_t len;

	len = make_entry(msg, 11U, "cn=admins,ou=groups,dc=example,dc=com", "cn",
			 "admins", "Administrators");

	TEST_ASSERT_EQUAL_INT(0, ldap_parse_search_entry(msg, len, "cn", &e));
	TEST_ASSERT_EQUAL_UINT32(11U, e.msgid);
	TEST_ASSERT_EQUAL_STRING("cn=admins,ou=groups,dc=example,dc=com", e.dn);
	TEST_ASSERT_EQUAL_STRING("cn", e.attr);
	TEST_ASSERT_EQUAL_UINT8(2U, e.n_values);
	TEST_ASSERT_EQUAL_STRING("admins", e.values[0]);
	TEST_ASSERT_EQUAL_STRING("Administrators", e.values[1]);

	/* A NULL want_attr keeps the first attribute whatever it is. */
	TEST_ASSERT_EQUAL_INT(0, ldap_parse_search_entry(msg, len, NULL, &e));
	TEST_ASSERT_EQUAL_STRING("cn", e.attr);

	/* An attribute that is not there leaves the DN usable and no values —
	 * which is exactly the membership-check case. */
	TEST_ASSERT_EQUAL_INT(0, ldap_parse_search_entry(msg, len,
							 "memberOf", &e));
	TEST_ASSERT_EQUAL_STRING("cn=admins,ou=groups,dc=example,dc=com", e.dn);
	TEST_ASSERT_EQUAL_UINT8(0U, e.n_values);
	TEST_ASSERT_EQUAL_STRING("", e.attr);

	/* An entry with no attributes at all. */
	{
		bw_t inner;
		bw_t empty;

		bw_reset(&empty);
		bw_reset(&inner);
		bw_str(&inner, LDAP_T_OCTET_STRING, "cn=x");
		bw_tlv(&inner, LDAP_T_SEQUENCE, empty.buf, 0U);
		len = bw_envelope(msg, 1U, LDAP_OP_SEARCH_ENTRY, inner.buf,
				  inner.len);
		TEST_ASSERT_EQUAL_INT(0, ldap_parse_search_entry(msg, len, "cn",
								 &e));
		TEST_ASSERT_EQUAL_STRING("cn=x", e.dn);
		TEST_ASSERT_EQUAL_UINT8(0U, e.n_values);
	}

	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_parse_search_entry(NULL, len, "cn",
							       &e));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_parse_search_entry(msg, len, "cn",
							       NULL));
}

static void test_ldap_parse_search_entry_bounds(void)
{
	uint8_t msg[LDAP_BUF_MAX];
	bw_t vals;
	bw_t one;
	bw_t attrs;
	bw_t inner;
	ldap_entry_t e;
	char big[LDAP_VALUE_MAX + 30U];
	size_t len;
	unsigned int i;

	memset(big, 'v', sizeof(big) - 1U);
	big[sizeof(big) - 1U] = '\0';

	/* More values than the fixed array holds, and one too long for it. */
	bw_reset(&vals);
	for (i = 0U; i < (LDAP_VALUES_MAX + 2U); i++) {
		bw_str(&vals, LDAP_T_OCTET_STRING, big);
	}
	bw_reset(&one);
	bw_str(&one, LDAP_T_OCTET_STRING, "member");
	bw_tlv(&one, LDAP_T_SET, vals.buf, vals.len);
	bw_reset(&attrs);
	bw_tlv(&attrs, LDAP_T_SEQUENCE, one.buf, one.len);
	bw_reset(&inner);
	bw_str(&inner, LDAP_T_OCTET_STRING, "cn=big");
	bw_tlv(&inner, LDAP_T_SEQUENCE, attrs.buf, attrs.len);
	len = bw_envelope(msg, 1U, LDAP_OP_SEARCH_ENTRY, inner.buf, inner.len);

	TEST_ASSERT_EQUAL_INT(0, ldap_parse_search_entry(msg, len, "member",
							 &e));
	TEST_ASSERT_EQUAL_UINT8(LDAP_VALUES_MAX, e.n_values);
	TEST_ASSERT_TRUE(e.truncated);
	TEST_ASSERT_EQUAL_UINT(LDAP_VALUE_MAX, strlen(e.values[0]));
}

static void test_ldap_parse_search_entry_malformed(void)
{
	uint8_t msg[LDAP_BUF_MAX];
	bw_t inner;
	bw_t attrs;
	bw_t one;
	ldap_entry_t e;
	size_t len;

	/* Not a SearchResultEntry. */
	bw_reset(&inner);
	bw_str(&inner, LDAP_T_OCTET_STRING, "cn=x");
	len = bw_envelope(msg, 1U, LDAP_OP_SEARCH_DONE, inner.buf, inner.len);
	TEST_ASSERT_EQUAL_INT(-ENOMSG, ldap_parse_search_entry(msg, len, NULL,
							       &e));

	/* An objectName that is not an OCTET STRING. */
	bw_reset(&inner);
	bw_u8(&inner, LDAP_T_INTEGER, 1U);
	len = bw_envelope(msg, 1U, LDAP_OP_SEARCH_ENTRY, inner.buf, inner.len);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ldap_parse_search_entry(msg, len, NULL,
								&e));

	/* A DN with no attribute list. */
	bw_reset(&inner);
	bw_str(&inner, LDAP_T_OCTET_STRING, "cn=x");
	len = bw_envelope(msg, 1U, LDAP_OP_SEARCH_ENTRY, inner.buf, inner.len);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ldap_parse_search_entry(msg, len, NULL,
								&e));

	/* An attribute list that is not a SEQUENCE. */
	bw_reset(&inner);
	bw_str(&inner, LDAP_T_OCTET_STRING, "cn=x");
	bw_u8(&inner, LDAP_T_INTEGER, 5U);
	len = bw_envelope(msg, 1U, LDAP_OP_SEARCH_ENTRY, inner.buf, inner.len);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ldap_parse_search_entry(msg, len, NULL,
								&e));

	/* A PartialAttribute whose values are not a SET. */
	bw_reset(&one);
	bw_str(&one, LDAP_T_OCTET_STRING, "cn");
	bw_u8(&one, LDAP_T_INTEGER, 3U);
	bw_reset(&attrs);
	bw_tlv(&attrs, LDAP_T_SEQUENCE, one.buf, one.len);
	bw_reset(&inner);
	bw_str(&inner, LDAP_T_OCTET_STRING, "cn=x");
	bw_tlv(&inner, LDAP_T_SEQUENCE, attrs.buf, attrs.len);
	len = bw_envelope(msg, 1U, LDAP_OP_SEARCH_ENTRY, inner.buf, inner.len);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ldap_parse_search_entry(msg, len, NULL,
								&e));

	/* A PartialAttribute that is not a SEQUENCE. */
	bw_reset(&attrs);
	bw_u8(&attrs, LDAP_T_INTEGER, 1U);
	bw_reset(&inner);
	bw_str(&inner, LDAP_T_OCTET_STRING, "cn=x");
	bw_tlv(&inner, LDAP_T_SEQUENCE, attrs.buf, attrs.len);
	len = bw_envelope(msg, 1U, LDAP_OP_SEARCH_ENTRY, inner.buf, inner.len);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ldap_parse_search_entry(msg, len, NULL,
								&e));
}

static void test_ldap_msg_framing(void)
{
	uint8_t msg[LDAP_BUF_MAX];
	uint8_t stream[LDAP_BUF_MAX];
	uint32_t id = 0U;
	uint8_t op = 0U;
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, ldap_build_bind_simple(1U, "cn=a", "pw", msg,
							sizeof(msg), &len));

	/* A stream reader learns the length before it has the whole message. */
	memcpy(stream, msg, len);
	TEST_ASSERT_EQUAL_INT((int)len, ldap_msg_len(stream, len));
	TEST_ASSERT_EQUAL_INT((int)len, ldap_msg_len(stream, len + 5U));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, ldap_msg_len(stream, len - 1U));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, ldap_msg_len(stream, 1U));

	/* Not a SEQUENCE. */
	stream[0] = 0x31U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ldap_msg_len(stream, len));
	/* An indefinite length is illegal in LDAP's BER subset. */
	stream[0] = 0x30U;
	stream[1] = 0x80U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ldap_msg_len(stream, len));
	/* A three-octet length is longer than we will ever buffer. */
	stream[1] = 0x83U;
	stream[2] = 0x01U;
	stream[3] = 0x00U;
	stream[4] = 0x00U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ldap_msg_len(stream, len));
	/* A two-octet length that exceeds LDAP_BUF_MAX. */
	stream[1] = 0x82U;
	stream[2] = 0xFFU;
	stream[3] = 0xFFU;
	TEST_ASSERT_EQUAL_INT(-EAGAIN, ldap_msg_len(stream, len));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_msg_len(NULL, 4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_msg_peek(NULL, 4U, &id, &op));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_msg_peek(msg, len, NULL, &op));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ldap_msg_peek(msg, len, &id, NULL));

	/* An envelope with no protocolOp. */
	{
		bw_t body;
		bw_t out;

		bw_reset(&body);
		bw_u8(&body, LDAP_T_INTEGER, 1U);
		bw_reset(&out);
		bw_tlv(&out, LDAP_T_SEQUENCE, body.buf, body.len);
		TEST_ASSERT_EQUAL_INT(-EBADMSG,
				      ldap_msg_peek(out.buf, out.len, &id, &op));
	}
	/* A negative messageID. */
	{
		bw_t body;
		bw_t out;

		bw_reset(&body);
		bw_u8(&body, LDAP_T_INTEGER, 0xFFU);
		bw_tlv(&body, LDAP_OP_UNBIND_REQUEST, NULL, 0U);
		bw_reset(&out);
		bw_tlv(&out, LDAP_T_SEQUENCE, body.buf, body.len);
		TEST_ASSERT_EQUAL_INT(-EBADMSG,
				      ldap_msg_peek(out.buf, out.len, &id, &op));
	}
	/* A messageID that is not an INTEGER. */
	{
		bw_t body;
		bw_t out;

		bw_reset(&body);
		bw_str(&body, LDAP_T_OCTET_STRING, "1");
		bw_tlv(&body, LDAP_OP_UNBIND_REQUEST, NULL, 0U);
		bw_reset(&out);
		bw_tlv(&out, LDAP_T_SEQUENCE, body.buf, body.len);
		TEST_ASSERT_EQUAL_INT(-EBADMSG,
				      ldap_msg_peek(out.buf, out.len, &id, &op));
	}
}

/* ========================================================================= */
/* LDAP — role mapping and DN safety                                         */
/* ========================================================================= */

static void test_ldap_role_from_group(void)
{
	ldap_role_map_t map;
	bool ex = false;

	memset(&map, 0, sizeof(map));
	map.admin = "cn=Admins,ou=groups,dc=example,dc=com";
	map.oper = "cn=Ops,ou=groups,dc=example,dc=com";
	map.viewer = "cn=Viewers,ou=groups,dc=example,dc=com";

	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_ADMIN,
				ldap_role_from_group(&map, map.admin, &ex));
	TEST_ASSERT_TRUE(ex);
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_OPERATOR,
				ldap_role_from_group(&map, map.oper, &ex));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER,
				ldap_role_from_group(&map, map.viewer, &ex));

	/* DN comparison is case-insensitive. */
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_ADMIN,
				ldap_role_from_group(&map,
						     "CN=ADMINS,OU=GROUPS,DC=EXAMPLE,DC=COM",
						     &ex));

	/* A prefix must not match. */
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_NONE,
				ldap_role_from_group(&map, "cn=Admins", &ex));
	TEST_ASSERT_FALSE(ex);
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_NONE,
				ldap_role_from_group(&map, "cn=Others", &ex));

	/* An unconfigured group never matches — including against "". */
	memset(&map, 0, sizeof(map));
	map.admin = "";
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_NONE,
				ldap_role_from_group(&map, "", &ex));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_NONE,
				ldap_role_from_group(&map, "cn=x", &ex));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_NONE,
				ldap_role_from_group(NULL, "cn=x", &ex));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_NONE,
				ldap_role_from_group(&map, NULL, &ex));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_NONE,
				ldap_role_from_group(&map, NULL, NULL));
}

static void test_ldap_dn_safety(void)
{
	TEST_ASSERT_TRUE(ldap_user_is_dn_safe("topher"));
	TEST_ASSERT_TRUE(ldap_user_is_dn_safe("first.last"));
	TEST_ASSERT_TRUE(ldap_user_is_dn_safe("a_b-c1"));

	/* Every DN metacharacter is refused rather than escaped. */
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe("a,b"));
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe("a+b"));
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe("a\"b"));
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe("a\\b"));
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe("a<b"));
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe("a>b"));
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe("a;b"));
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe("a=b"));
	/* And so are the LDAP *filter* metacharacters. */
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe("a*"));
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe("a(b"));
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe("a)b"));
	/* Leading/trailing space and a leading '#'. */
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe(" a"));
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe("a "));
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe("#a"));
	/* Control characters and non-ASCII. */
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe("a\tb"));
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe("a\x80"));
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe(""));
	TEST_ASSERT_FALSE(ldap_user_is_dn_safe(NULL));
	{
		char big[AUTH_USER_MAX + 2U];

		memset(big, 'u', sizeof(big) - 1U);
		big[sizeof(big) - 1U] = '\0';
		TEST_ASSERT_FALSE(ldap_user_is_dn_safe(big));
	}
}

static void test_ldap_dn_template(void)
{
	char out[LDAP_DN_MAX + 1U];
	int n;

	n = ldap_dn_from_template("uid=%s,ou=people,dc=example,dc=com", "topher",
				  out, sizeof(out));
	TEST_ASSERT_EQUAL_INT((int)strlen(out), n);
	TEST_ASSERT_EQUAL_STRING("uid=topher,ou=people,dc=example,dc=com", out);

	/* The substitution point may be at either end. */
	n = ldap_dn_from_template("%s@example.com", "topher", out, sizeof(out));
	TEST_ASSERT_EQUAL_STRING("topher@example.com", out);
	TEST_ASSERT_TRUE(n > 0);
	n = ldap_dn_from_template("prefix-%s", "topher", out, sizeof(out));
	TEST_ASSERT_EQUAL_STRING("prefix-topher", out);

	/* Only the first %s is substituted. */
	n = ldap_dn_from_template("a=%s,b=%s", "x", out, sizeof(out));
	TEST_ASSERT_EQUAL_STRING("a=x,b=%s", out);
	TEST_ASSERT_TRUE(n > 0);

	/* An unsafe username never reaches a DN. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ldap_dn_from_template("uid=%s,dc=x",
						    "evil,ou=admins", out,
						    sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ldap_dn_from_template("uid=x,dc=y", "topher", out,
						    sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ldap_dn_from_template(NULL, "u", out,
						    sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ldap_dn_from_template("uid=%s", NULL, out,
						    sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ldap_dn_from_template("uid=%s", "u", NULL,
						    sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ldap_dn_from_template("uid=%s", "u", out, 0U));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      ldap_dn_from_template("uid=%s,ou=people", "topher",
						    out, 8U));
}

/* ========================================================================= */
/* hostile input                                                             */
/* ========================================================================= */

static void test_ldap_random_messages_never_crash(void)
{
	test_rng_t g;
	uint8_t buf[400];
	ldap_result_t r;
	ldap_entry_t e;
	uint32_t id;
	uint8_t op;
	unsigned int i;

	test_rng_init(&g, 0x4511U);
	for (i = 0U; i < 30000U; i++) {
		size_t n = 1U + test_rng_below(&g, sizeof(buf));

		test_rng_fill(&g, buf, n);
		/* Half of them get a plausible envelope so the operation
		 * parsers see real work. */
		if ((i & 1U) != 0U && n > 8U) {
			buf[0] = LDAP_T_SEQUENCE;
			buf[1] = (uint8_t)(n - 2U);
			buf[2] = LDAP_T_INTEGER;
			buf[3] = 0x01U;
			buf[4] = 0x01U;
		}
		(void)ldap_msg_len(buf, n);
		(void)ldap_msg_peek(buf, n, &id, &op);
		(void)ldap_parse_result(buf, n, 0U, &r);
		(void)ldap_parse_search_entry(buf, n, "cn", &e);
		(void)ldap_parse_search_entry(buf, n, NULL, &e);
	}
}

static void test_ldap_mutated_messages_never_crash(void)
{
	test_rng_t g;
	uint8_t msg[LDAP_BUF_MAX];
	ldap_result_t r;
	ldap_entry_t e;
	unsigned int i;

	test_rng_init(&g, 0x1234ABCDU);
	for (i = 0U; i < 20000U; i++) {
		size_t len = make_entry(msg, 3U, "cn=g,dc=x", "member",
					"uid=a,dc=x", "uid=b,dc=x");
		size_t pos = test_rng_below(&g, len);

		msg[pos] ^= (uint8_t)(1U << test_rng_below(&g, 8U));
		(void)ldap_parse_search_entry(msg, len, "member", &e);
		(void)ldap_parse_result(msg, len, 0U, &r);
	}
}

static void test_auth_chain_random_inputs_never_crash(void)
{
	auth_ctx_t a;
	auth_cfg_t c = base_cfg();
	test_rng_t g;
	char user[AUTH_USER_MAX + 1U];
	char secret[AUTH_SECRET_MAX + 1U];
	unsigned int i;

	c.order[0] = AUTH_BE_LOCAL;
	c.order[1] = AUTH_BE_RADIUS;
	c.n_order = 2U;
	c.cache_ttl_s = 5U;
	c.lockout_fails = 4U;
	c.lockout_s = 10U;
	TEST_ASSERT_EQUAL_INT(0, auth_init(&a, &c));

	test_rng_init(&g, 0xA11AU);
	for (i = 0U; i < 20000U; i++) {
		size_t ul = 1U + test_rng_below(&g, AUTH_USER_MAX);
		size_t sl = 1U + test_rng_below(&g, AUTH_SECRET_MAX);
		size_t j;
		int rc;

		for (j = 0U; j < ul; j++) {
			user[j] = (char)('a' + (test_rng_u32(&g) % 26U));
		}
		user[ul] = '\0';
		for (j = 0U; j < sl; j++) {
			secret[j] = (char)(0x21U + (test_rng_u32(&g) % 0x5EU));
		}
		secret[sl] = '\0';

		g_remote.result[AUTH_BE_RADIUS] =
			(int)(test_rng_u32(&g) % 3U) == 0
				? 0
				: ((int)(test_rng_u32(&g) % 2U) == 0 ? -EACCES
								     : -EHOSTUNREACH);
		g_remote.role[AUTH_BE_RADIUS] =
			(uint8_t)(test_rng_u32(&g) % 5U);

		rc = auth_check(&a, user, secret, i * 100U, NULL);
		/* Only the four documented outcomes are ever produced. */
		TEST_ASSERT_TRUE(rc == 0 || rc == -EACCES || rc == -EBUSY ||
				 rc == -EHOSTUNREACH);
	}
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_role_names_and_parsing);
	RUN_TEST(test_backend_names);
	RUN_TEST(test_order_parsing);
	RUN_TEST(test_constant_time_compare);

	RUN_TEST(test_local_verify_layout);
	RUN_TEST(test_local_verify_guards);

	RUN_TEST(test_init_guards);
	RUN_TEST(test_order_is_normalised);
	RUN_TEST(test_local_only_accept_and_reject);
	RUN_TEST(test_check_argument_guards);
	RUN_TEST(test_chain_stops_on_reject_but_not_on_unavailable);
	RUN_TEST(test_every_backend_unavailable_is_a_denial);
	RUN_TEST(test_accepting_backend_with_no_role_gets_least_privilege);

	RUN_TEST(test_cache_hit_and_expiry);
	RUN_TEST(test_cache_is_not_poisoned_by_rejections);
	RUN_TEST(test_cache_flush_and_eviction);
	RUN_TEST(test_cache_disabled_without_a_csprng);

	RUN_TEST(test_lockout_after_n_failures);
	RUN_TEST(test_lockout_is_shared_by_every_backend);
	RUN_TEST(test_success_resets_the_failure_count);
	RUN_TEST(test_unlock_and_lock_table_pressure);
	RUN_TEST(test_reconfigure_keeps_lockout_drops_cache);

	RUN_TEST(test_ldap_result_names_and_mapping);
	RUN_TEST(test_ldap_bind_known_answer);
	RUN_TEST(test_ldap_bind_refuses_unauthenticated_bind);
	RUN_TEST(test_ldap_unbind_and_starttls);
	RUN_TEST(test_ldap_search_layout);
	RUN_TEST(test_ldap_search_and_filter);
	RUN_TEST(test_ldap_search_guards);
	RUN_TEST(test_ldap_parse_result);
	RUN_TEST(test_ldap_parse_result_truncation_and_errors);
	RUN_TEST(test_ldap_parse_search_entry);
	RUN_TEST(test_ldap_parse_search_entry_bounds);
	RUN_TEST(test_ldap_parse_search_entry_malformed);
	RUN_TEST(test_ldap_msg_framing);
	RUN_TEST(test_ldap_role_from_group);
	RUN_TEST(test_ldap_dn_safety);
	RUN_TEST(test_ldap_dn_template);

	RUN_TEST(test_ldap_random_messages_never_crash);
	RUN_TEST(test_ldap_mutated_messages_never_crash);
	RUN_TEST(test_auth_chain_random_inputs_never_crash);

	return UNITY_END();
}
