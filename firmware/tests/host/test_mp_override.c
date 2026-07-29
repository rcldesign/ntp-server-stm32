/*
 * STS1000 "Meridian" — core/mp override, guard and interlock unit tests.
 *
 * This is the module whose failure modes matter more than its features, so the
 * tests are organised around the properties the FMT spec puts numbers on:
 *
 *   §5.2  the guard escalation, as an exhaustive matrix over
 *         (class, session, role, keepalive freshness, confirmation, nonce, hold);
 *   §5.3  the role floor — G1 needs operator, G2/G3 need admin — including that
 *         the typed device serial does *not* substitute for a role, because the
 *         serial is printed on the chassis and returned by `hello`;
 *   §5.3  the dead-man — every override reverts to firmware-automatic within the
 *         keepalive TTL plus one tick, on *any* of: link loss, session loss,
 *         stale keepalive, session close, mode exit;
 *   §5.4  the firmware veto, and that an apply refusal never leaves a
 *         half-claimed lease;
 *   §5.5  every interlock, both refusing and permitting, with the VCC_RB and
 *         RB_PWR_EN cases pinned against the as-built hardware rules.
 *
 * The apply callback records rather than acts, so "reverted to
 * firmware-automatic" is checked as an actual `apply(obj, NULL)` call, not
 * inferred from a flag.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

/*
 * core/auth's header, for the enum only — no auth symbol is linked. core/mp
 * cannot include it (ARCHITECTURE.md §4 gives mp no `auth` edge), so it carries
 * its own MP_ROLE_* constants; this is where the two are proved numerically
 * identical on the host, mirroring the BUILD_ASSERTs in mp_glue.c.
 */
#include "auth/auth.h"
#include "mp/mp_override.h"

#define SERIAL "STS1000-000042"
#define USER "tech"

/* ------------------------------------------------------------------ fixture */

#define APPLY_LOG_MAX 64U
#define EVT_LOG_MAX 64U

typedef struct {
	size_t obj;
	bool release; /* true when value was NULL */
	int32_t value;
} apply_rec_t;

typedef struct {
	uint8_t ev;
	size_t obj;
	uint32_t sid;
	char reason[64];
} evt_rec_t;

static apply_rec_t g_apply[APPLY_LOG_MAX];
static unsigned int g_apply_n;
static int g_apply_rc; /* what the callback returns */
static evt_rec_t g_evt[EVT_LOG_MAX];
static unsigned int g_evt_n;

static mp_ovr_ctx_t g_c;

static int apply_cb(void *user, size_t obj, const int32_t *value)
{
	(void)user;
	if (g_apply_n < APPLY_LOG_MAX) {
		g_apply[g_apply_n].obj = obj;
		g_apply[g_apply_n].release = (value == NULL);
		g_apply[g_apply_n].value = (value != NULL) ? *value : 0;
		g_apply_n++;
	}
	/* A release must always be permitted, or a test that makes apply fail
	 * would also break the revert path it is trying to observe. */
	return (value == NULL) ? 0 : g_apply_rc;
}

static void evt_cb(void *user, uint8_t ev, size_t obj, uint32_t sid,
		   const char *reason)
{
	(void)user;
	if (g_evt_n < EVT_LOG_MAX) {
		g_evt[g_evt_n].ev = ev;
		g_evt[g_evt_n].obj = obj;
		g_evt[g_evt_n].sid = sid;
		g_evt[g_evt_n].reason[0] = '\0';
		if (reason != NULL) {
			size_t n = strlen(reason);

			if (n > (sizeof(g_evt[0].reason) - 1U)) {
				n = sizeof(g_evt[0].reason) - 1U;
			}
			memcpy(g_evt[g_evt_n].reason, reason, n);
			g_evt[g_evt_n].reason[n] = '\0';
		}
		g_evt_n++;
	}
}

static void logs_clear(void)
{
	g_apply_n = 0U;
	g_evt_n = 0U;
	g_apply_rc = 0;
}

void setUp(void)
{
	logs_clear();
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_init(&g_c, apply_cb, NULL, evt_cb, NULL,
					     SERIAL));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_set_link(&g_c, true, 1000U));
}

/** Count releases (apply with NULL) for @p obj in the log. */
static unsigned int releases_of(size_t obj)
{
	unsigned int i;
	unsigned int n = 0U;

	for (i = 0U; i < g_apply_n; i++) {
		if (g_apply[i].release && (g_apply[i].obj == obj)) {
			n++;
		}
	}
	return n;
}

static unsigned int events_of(uint8_t ev)
{
	unsigned int i;
	unsigned int n = 0U;

	for (i = 0U; i < g_evt_n; i++) {
		if (g_evt[i].ev == ev) {
			n++;
		}
	}
	return n;
}

/** A sane interlock state in which nothing refuses. */
static void ilk_permissive(mp_ilk_state_t *st)
{
	memset(st, 0, sizeof(*st));
	st->ocxo_warm = true;
	st->supercaps_ok = true;
	st->rb_ov_latched = false;
	st->vcc_rb_mv = 15000;
	st->vcc_rb_valid = true;
	st->rb_expected_mv = 15000;
	st->rb_vmax_mv = 15000U;
	st->rb_code_min = 0U;
	st->rb_code_max = 200U;
	st->fan_floor_pct = 0U;
	st->relay_blocked = false;
	st->disp_on = true;
	st->disp_changed_ms = 0U;
	st->liveness_ok = true;
	st->wdt_off = true;
	st->disc_parked = true;
	st->extref_ok = true;
	st->rb_lock = true;
}

/** Manifest index of @p id, asserted present. */
static size_t obj_of(const char *id)
{
	int idx = mp_obj_find(id);

	TEST_ASSERT_TRUE_MESSAGE(idx >= 0, id);
	return (size_t)idx;
}

/** Open a session with @p role. */
static uint32_t open_session_as(uint32_t now, uint8_t role, const char *user)
{
	uint32_t sid = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_session_open(&g_c, 0U, role, user, now,
						     &sid));
	TEST_ASSERT_NOT_EQUAL_UINT32(0U, sid);
	return sid;
}

/**
 * Open an **admin** session, which is what the lease and interlock tests need:
 * they are about the interlocks, not about authentication, so they run at the
 * privilege every guard class accepts. The role floor itself is tested
 * separately, below.
 */
static uint32_t open_session(uint32_t now)
{
	return open_session_as(now, (uint8_t)MP_ROLE_ADMIN, USER);
}

/* ------------------------------------------------------------------ naming */

static void test_names(void)
{
	uint8_t i;

	for (i = 0U; i < (uint8_t)MP_OVR_EV_COUNT; i++) {
		TEST_ASSERT_TRUE(strlen(mp_ovr_ev_name(i)) > 0U);
		TEST_ASSERT_TRUE(
			strcmp(mp_ovr_ev_name(i), "unknown") != 0);
	}
	TEST_ASSERT_EQUAL_STRING("unknown", mp_ovr_ev_name(MP_OVR_EV_COUNT));

	for (i = 0U; i < (uint8_t)MP_GC_COUNT; i++) {
		TEST_ASSERT_TRUE(strlen(mp_gc_name(i)) > 0U);
		TEST_ASSERT_TRUE(strcmp(mp_gc_name(i), "unknown") != 0);
	}
	TEST_ASSERT_EQUAL_STRING("unknown", mp_gc_name(MP_GC_COUNT));
	TEST_ASSERT_EQUAL_STRING("ok", mp_gc_name(MP_GC_OK));
	TEST_ASSERT_EQUAL_STRING("need-role", mp_gc_name(MP_GC_NEED_ROLE));

	TEST_ASSERT_EQUAL_STRING("none", mp_role_name(MP_ROLE_NONE));
	TEST_ASSERT_EQUAL_STRING("viewer", mp_role_name(MP_ROLE_VIEWER));
	TEST_ASSERT_EQUAL_STRING("operator", mp_role_name(MP_ROLE_OPERATOR));
	TEST_ASSERT_EQUAL_STRING("admin", mp_role_name(MP_ROLE_ADMIN));
	TEST_ASSERT_EQUAL_STRING("unknown", mp_role_name(MP_ROLE_ADMIN + 1U));
	TEST_ASSERT_EQUAL_STRING("unknown", mp_role_name(255U));
}

/**
 * The role constants core/mp carries must be the ones core/auth hands out.
 *
 * MP_ROLE_* exists because core/mp has no `auth` dependency edge; if the two
 * ever diverge, an operator's numeric role would silently satisfy an admin
 * guard. Pinned here and, on target, by BUILD_ASSERTs in mp_glue.c.
 */
static void test_roles_match_the_auth_enum(void)
{
	TEST_ASSERT_EQUAL_UINT((unsigned int)AUTH_ROLE_NONE,
			       (unsigned int)MP_ROLE_NONE);
	TEST_ASSERT_EQUAL_UINT((unsigned int)AUTH_ROLE_VIEWER,
			       (unsigned int)MP_ROLE_VIEWER);
	TEST_ASSERT_EQUAL_UINT((unsigned int)AUTH_ROLE_OPERATOR,
			       (unsigned int)MP_ROLE_OPERATOR);
	TEST_ASSERT_EQUAL_UINT((unsigned int)AUTH_ROLE_ADMIN,
			       (unsigned int)MP_ROLE_ADMIN);
	/* Ordered so a numeric compare is a privilege compare. */
	TEST_ASSERT_TRUE(MP_ROLE_NONE < MP_ROLE_VIEWER);
	TEST_ASSERT_TRUE(MP_ROLE_VIEWER < MP_ROLE_OPERATOR);
	TEST_ASSERT_TRUE(MP_ROLE_OPERATOR < MP_ROLE_ADMIN);
	/* And a session can record the longest credential name AAA accepts. */
	TEST_ASSERT_TRUE(MP_USER_MAX >= AUTH_USER_MAX);
	TEST_ASSERT_TRUE(MP_SECRET_MAX >= AUTH_SECRET_MAX);
}

static void test_init_validation(void)
{
	mp_ovr_ctx_t c;

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_ovr_init(NULL, apply_cb, NULL, NULL, NULL, ""));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_ovr_init(&c, NULL, NULL, NULL, NULL, ""));
	/* A NULL serial is legal: G2 then always refuses. */
	TEST_ASSERT_EQUAL_INT(0,
			      mp_ovr_init(&c, apply_cb, NULL, NULL, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_ovr_set_phrase(NULL, "x"));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_ovr_set_link(NULL, true, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_ovr_tick(NULL, NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_ovr_revert_all(NULL, 0U, NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_ovr_keepalive(NULL, 1U, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_ovr_session_close(NULL, 1U, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_ovr_session_open(NULL, 0U,
						  (uint8_t)MP_ROLE_ADMIN, USER,
						  0U, NULL));
	TEST_ASSERT_NULL(mp_ovr_lease(NULL, 0U));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(NULL));
	TEST_ASSERT_FALSE(mp_ovr_session_valid(NULL, 1U, 0U));
	TEST_ASSERT_EQUAL_UINT32(0U, mp_ovr_session_remaining(NULL, 0U));
	mp_ovr_disarm(NULL); /* must not fault */
}

/* ----------------------------------------------------------------- session */

static void test_session_lifecycle(void)
{
	uint32_t sid1;
	uint32_t sid2 = 0U;

	sid1 = open_session(1000U);
	TEST_ASSERT_TRUE(mp_ovr_session_valid(&g_c, sid1, 1000U));
	TEST_ASSERT_FALSE(mp_ovr_session_valid(&g_c, sid1 + 1U, 1000U));
	TEST_ASSERT_FALSE(mp_ovr_session_valid(&g_c, 0U, 1000U));
	TEST_ASSERT_EQUAL_UINT32(MP_KEEPALIVE_TTL_MS,
				 mp_ovr_session_remaining(&g_c, 1000U));

	/* Keepalive refreshes; a wrong id does not. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid1, 4000U));
	TEST_ASSERT_EQUAL_INT(-ENOENT, mp_ovr_keepalive(&g_c, sid1 + 1U, 4000U));
	TEST_ASSERT_EQUAL_INT(-ENOENT, mp_ovr_keepalive(&g_c, 0U, 4000U));
	TEST_ASSERT_TRUE(mp_ovr_session_valid(&g_c, sid1, 8000U));

	/* Past the TTL it is gone, and a late keepalive does not resurrect it. */
	TEST_ASSERT_FALSE(mp_ovr_session_valid(&g_c, sid1,
					       4000U + MP_KEEPALIVE_TTL_MS +
						       1U));
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      mp_ovr_keepalive(&g_c, sid1,
					       4000U + MP_KEEPALIVE_TTL_MS +
						       1U));
	TEST_ASSERT_EQUAL_UINT32(0U,
				 mp_ovr_session_remaining(&g_c,
							  4000U +
								  MP_KEEPALIVE_TTL_MS));

	/* Reopening yields a different id. */
	TEST_ASSERT_EQUAL_INT(0,
			      mp_ovr_session_open(&g_c, 0U,
						  (uint8_t)MP_ROLE_ADMIN, USER,
						  20000U, &sid2));
	TEST_ASSERT_NOT_EQUAL_UINT32(sid1, sid2);

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_session_close(&g_c, sid2, 21000U));
	TEST_ASSERT_EQUAL_INT(-ENOENT, mp_ovr_session_close(&g_c, sid2, 21000U));
	TEST_ASSERT_EQUAL_UINT32(0U, mp_ovr_session_remaining(&g_c, 21000U));
}

static void test_session_ttl_is_clamped(void)
{
	uint32_t sid = 0U;

	TEST_ASSERT_EQUAL_INT(0,
			      mp_ovr_session_open(&g_c, 999999U,
						  (uint8_t)MP_ROLE_ADMIN, USER,
						  1000U, &sid));
	TEST_ASSERT_EQUAL_UINT32(MP_KEEPALIVE_TTL_MS, g_c.sess.ttl_ms);

	TEST_ASSERT_EQUAL_INT(0,
			      mp_ovr_session_open(&g_c, 1500U,
						  (uint8_t)MP_ROLE_ADMIN, USER,
						  2000U, &sid));
	TEST_ASSERT_EQUAL_UINT32(1500U, g_c.sess.ttl_ms);
	TEST_ASSERT_FALSE(mp_ovr_session_valid(&g_c, sid, 3600U));
}

static void test_session_requires_a_link(void)
{
	uint32_t sid = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_set_link(&g_c, false, 1000U));
	TEST_ASSERT_EQUAL_INT(-ENOLINK,
			      mp_ovr_session_open(&g_c, 0U,
						  (uint8_t)MP_ROLE_ADMIN, USER,
						  1000U, &sid));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_ovr_session_open(&g_c, 0U,
						  (uint8_t)MP_ROLE_ADMIN, USER,
						  1000U, NULL));
}

static void test_reopening_reverts_the_previous_sessions_overrides(void)
{
	size_t obj = obj_of("ui.disp.bl");
	mp_ilk_state_t st;
	uint32_t sid1;
	uint32_t sid2 = 0U;

	ilk_permissive(&st);
	sid1 = open_session(1000U);
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 40, 0U, sid1, &st,
					      1000U, NULL));
	TEST_ASSERT_EQUAL_size_t(1U, mp_ovr_active(&g_c));

	logs_clear();
	TEST_ASSERT_EQUAL_INT(0,
			      mp_ovr_session_open(&g_c, 0U,
						  (uint8_t)MP_ROLE_ADMIN, USER,
						  2000U, &sid2));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT(1U, releases_of(obj));
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_RELEASE));
}

static void test_session_records_the_role_and_the_user(void)
{
	uint32_t sid;

	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_ROLE_NONE,
				mp_ovr_session_role(&g_c));
	TEST_ASSERT_EQUAL_STRING("", mp_ovr_session_user(&g_c));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_ROLE_NONE, mp_ovr_session_role(NULL));
	TEST_ASSERT_EQUAL_STRING("", mp_ovr_session_user(NULL));

	sid = open_session_as(1000U, (uint8_t)MP_ROLE_OPERATOR, "alice");
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_ROLE_OPERATOR,
				mp_ovr_session_role(&g_c));
	TEST_ASSERT_EQUAL_STRING("alice", mp_ovr_session_user(&g_c));

	/* Closing forgets both — an audit record must never name a session that
	 * is no longer there. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_session_close(&g_c, sid, 1100U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_ROLE_NONE,
				mp_ovr_session_role(&g_c));
	TEST_ASSERT_EQUAL_STRING("", mp_ovr_session_user(&g_c));

	/* A NULL user is legal and reads back as empty, not as a dangling
	 * pointer. */
	(void)open_session_as(2000U, (uint8_t)MP_ROLE_VIEWER, NULL);
	TEST_ASSERT_EQUAL_STRING("", mp_ovr_session_user(&g_c));

	/* An over-long name is truncated, not overflowed. */
	{
		char long_user[MP_USER_MAX + 32U];

		memset(long_user, 'u', sizeof(long_user) - 1U);
		long_user[sizeof(long_user) - 1U] = '\0';
		(void)open_session_as(3000U, (uint8_t)MP_ROLE_ADMIN, long_user);
		TEST_ASSERT_EQUAL_size_t(MP_USER_MAX,
					 strlen(mp_ovr_session_user(&g_c)));
	}
}

/**
 * A role the device does not recognise grants nothing.
 *
 * Roles compare numerically, so a backend returning 200 must not read as
 * "more than admin". The fail-safe answer for an unknown role is no privilege.
 */
static void test_an_unrecognised_role_grants_nothing(void)
{
	uint32_t sid = open_session_as(1000U, 200U, "weird");

	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_ROLE_NONE,
				mp_ovr_session_role(&g_c));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_ROLE,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, sid, 0U, 0U, NULL,
					   NULL, 0U, 1000U, NULL));
}

/**
 * A takeover does not inherit the previous session's privilege.
 *
 * The concrete attack: an admin opens a session, then an anonymous caller opens
 * another. If the role survived the reopen, the second caller would hold admin.
 */
static void test_session_takeover_does_not_inherit_the_role(void)
{
	uint32_t admin_sid;
	uint32_t anon_sid;

	admin_sid = open_session_as(1000U, (uint8_t)MP_ROLE_ADMIN, "admin");
	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, admin_sid, 0U, 0U,
					   SERIAL, NULL, 0U, 1000U, NULL));
	TEST_ASSERT_TRUE(g_c.sess.serial_ok);

	/* The next tool presents no credential at all. */
	anon_sid = open_session_as(2000U, (uint8_t)MP_ROLE_NONE, NULL);
	TEST_ASSERT_NOT_EQUAL_UINT32(admin_sid, anon_sid);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_ROLE_NONE,
				mp_ovr_session_role(&g_c));
	TEST_ASSERT_EQUAL_STRING("", mp_ovr_session_user(&g_c));
	/* The G2 confirmation does not carry over either. */
	TEST_ASSERT_FALSE(g_c.sess.serial_ok);

	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_ROLE,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, anon_sid, 0U, 0U,
					   NULL, NULL, 0U, 2000U, NULL));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_ROLE,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, anon_sid, 0U, 0U,
					   SERIAL, NULL, 0U, 2000U, NULL));

	/* And the old id is dead, so the admin cannot keep using it either. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SESSION,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, admin_sid, 0U, 0U,
					   NULL, NULL, 0U, 2000U, NULL));
}

/* ============================================== §5.2 guard escalation ==== */

/** An unauthenticated session may observe (G0) and nothing else. */
static void test_a_session_with_no_credential_is_capped_at_g0(void)
{
	uint32_t sid = open_session_as(1000U, (uint8_t)MP_ROLE_NONE, NULL);

	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G0, sid, 0U, 0U, NULL,
					   NULL, 0U, 1000U, NULL));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_ROLE,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, sid, 0U, 0U, NULL,
					   NULL, 0U, 1000U, NULL));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_ROLE,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, SERIAL,
					   NULL, 0U, 1000U, NULL));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_ROLE,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, SERIAL,
					   MP_G3_PHRASE_DEFAULT, 0U, 1000U,
					   NULL));
	/* Refused, and nothing was armed by the attempt. G0 is not a refusal, so
	 * the three guarded attempts are all that count. */
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.sess.arm_nonce);
	TEST_ASSERT_EQUAL_UINT32(3U, g_c.refusals);
}

/** A viewer is read-only: it may not perform a G1 action. */
static void test_a_viewer_cannot_perform_a_g1_action(void)
{
	uint32_t sid = open_session_as(1000U, (uint8_t)MP_ROLE_VIEWER, "obs");

	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G0, sid, 0U, 0U, NULL,
					   NULL, 0U, 1000U, NULL));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_ROLE,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, sid, 0U, 0U, NULL,
					   NULL, 0U, 1000U, NULL));
	TEST_ASSERT_EQUAL_UINT32(1U, g_c.refusals);
}

/**
 * An operator may perform G1 but not G2 — **even with the correct serial**.
 *
 * This is the property that makes the typed serial an anti-mistake interlock
 * rather than a credential: it is printed on the label and returned by `hello`,
 * so being able to type it must buy no privilege.
 */
static void test_an_operator_gets_g1_but_not_g2_even_with_the_serial(void)
{
	uint32_t sid = open_session_as(1000U, (uint8_t)MP_ROLE_OPERATOR, "op");

	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, sid, 0U, 0U, NULL,
					   NULL, 0U, 1000U, NULL));

	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_ROLE,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, SERIAL,
					   NULL, 0U, 1000U, NULL));
	/* The refusal is a role refusal, not a serial one, and the G2
	 * confirmation was never recorded. */
	TEST_ASSERT_FALSE(g_c.sess.serial_ok);

	/* G3 is refused for the same reason, and does not arm. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_ROLE,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, SERIAL,
					   MP_G3_PHRASE_DEFAULT, 0U, 1000U,
					   NULL));
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.sess.arm_nonce);
}

/** An admin with the correct serial gets G2. */
static void test_an_admin_with_the_serial_gets_g2(void)
{
	uint32_t sid = open_session_as(1000U, (uint8_t)MP_ROLE_ADMIN, "admin");

	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, sid, 0U, 0U, NULL,
					   NULL, 0U, 1000U, NULL));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, SERIAL,
					   NULL, 0U, 1000U, NULL));
	TEST_ASSERT_TRUE(g_c.sess.serial_ok);
	/* Admin is still not exempt from the serial itself. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, NULL,
					   NULL, 0U, 1000U, NULL));
	TEST_ASSERT_EQUAL_UINT32(1U, g_c.refusals);
}

/**
 * An under-privileged caller cannot hold its session open by hammering.
 *
 * The keepalive refresh is a reward for a *successful* guard check, so a viewer
 * spraying G1 requests must still go stale on schedule.
 */
static void test_a_refused_role_does_not_refresh_the_keepalive(void)
{
	uint32_t sid = open_session_as(1000U, (uint8_t)MP_ROLE_VIEWER, "obs");
	uint32_t t;

	for (t = 1000U; t < (1000U + MP_KEEPALIVE_TTL_MS); t += 1000U) {
		TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_ROLE,
				      mp_ovr_guard(&g_c, MP_GUARD_G1, sid, 0U, 0U,
						   NULL, NULL, 0U, t, NULL));
	}
	TEST_ASSERT_FALSE(mp_ovr_session_valid(&g_c, sid,
					       1000U + MP_KEEPALIVE_TTL_MS + 1U));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_STALE,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, sid, 0U, 0U, NULL,
					   NULL, 0U,
					   1000U + MP_KEEPALIVE_TTL_MS + 1U,
					   NULL));
}

static void test_g0_needs_nothing(void)
{
	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G0, 0U, 0U, 0U, NULL, NULL,
					   0U, 1000U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_ovr_guard(NULL, MP_GUARD_G0, 0U, 0U, 0U, NULL, NULL,
					   0U, 1000U, NULL));
}

static void test_g1_needs_a_fresh_session(void)
{
	uint32_t sid;

	/* No session at all. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SESSION,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, 0U, 0U, 0U, NULL, NULL,
					   0U, 1000U, NULL));
	sid = open_session(1000U);

	/* Wrong id. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SESSION,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, sid + 1U, 0U, 0U, NULL, NULL, 0U, 1000U, NULL));
	/* Right id, fresh. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, sid, 0U, 0U, NULL, NULL,
					   0U, 1000U, NULL));

	/* Stale keepalive. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_STALE,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, sid, 0U, 0U, NULL, NULL,
					   0U,
					   1000U + MP_KEEPALIVE_TTL_MS + 1U,
					   NULL));

	/* Link down. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_set_link(&g_c, false, 1000U));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SESSION,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, sid, 0U, 0U, NULL, NULL,
					   0U, 1000U, NULL));
}

static void test_a_successful_guard_refreshes_the_keepalive(void)
{
	uint32_t sid = open_session(1000U);

	/* Issuing a command demonstrates the tool is alive (FMT §5.3). */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, sid, 0U, 0U, NULL, NULL,
					   0U, 4000U, NULL));
	TEST_ASSERT_TRUE(mp_ovr_session_valid(&g_c, sid, 8500U));
}

static void test_g2_needs_the_typed_device_serial(void)
{
	uint32_t sid = open_session(1000U);

	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, NULL, NULL,
					   0U, 1000U, NULL));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, "", NULL,
					   0U, 1000U, NULL));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, "wrong", NULL, 0U, 1000U, NULL));
	/* A prefix is not the serial. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, "STS1000", NULL, 0U, 1000U, NULL));
	/* Nor is the serial plus a suffix. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, SERIAL "x", NULL, 0U, 1000U, NULL));
	/* Case matters. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, "sts1000-000042", NULL, 0U, 1000U, NULL));

	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, SERIAL, NULL, 0U, 1000U, NULL));
	TEST_ASSERT_TRUE(g_c.sess.serial_ok);

	/* G2 still requires a session first: no session outranks a good serial. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_session_close(&g_c, sid, 2000U));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SESSION,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, SERIAL, NULL, 0U, 2000U, NULL));
}

static void test_g2_on_an_unprovisioned_board_always_refuses(void)
{
	uint32_t sid = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_init(&g_c, apply_cb, NULL, evt_cb, NULL,
					     NULL));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_set_link(&g_c, true, 1000U));
	TEST_ASSERT_EQUAL_INT(0,
			      mp_ovr_session_open(&g_c, 0U,
						  (uint8_t)MP_ROLE_ADMIN, USER,
						  1000U, &sid));

	/* An empty expected string must never match, including against "". */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, "", NULL,
					   0U, 1000U, NULL));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, SERIAL, NULL, 0U, 1000U, NULL));
}

static void test_g3_arm_hold_and_complete(void)
{
	uint32_t sid = open_session(1000U);
	mp_gc_arm_t arm;
	uint32_t nonce;

	memset(&arm, 0, sizeof(arm));

	/* The phrase is required to arm; the serial alone is not enough. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, NULL, NULL,
					   0U, 1000U, &arm));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_PHRASE,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, SERIAL, NULL, 0U, 1000U, &arm));

	/* Arming needs the phrase. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARMED,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, SERIAL, MP_G3_PHRASE_DEFAULT, 0U, 1000U,
					   &arm));
	TEST_ASSERT_NOT_EQUAL_UINT32(0U, arm.nonce);
	TEST_ASSERT_EQUAL_UINT32(MP_G3_HOLD_MS, arm.hold_ms);
	TEST_ASSERT_EQUAL_UINT32(MP_G3_WINDOW_MS, arm.expires_ms);
	nonce = arm.nonce;

	/* Too early. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_HOLD,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, SERIAL, NULL, nonce,
					   1000U + MP_G3_HOLD_MS - 1U, NULL));
	/* Exactly at the hold: allowed. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, SERIAL, NULL, nonce, 1000U + MP_G3_HOLD_MS,
					   NULL));
	/* The arm is consumed: a replay fails. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARM_MISMATCH,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, SERIAL, NULL, nonce, 1000U + MP_G3_HOLD_MS,
					   NULL));
}

static void test_g3_nonce_is_bound_to_the_action(void)
{
	uint32_t sid = open_session(1000U);
	mp_gc_arm_t arm;

	memset(&arm, 0, sizeof(arm));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARMED,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 7U, 0U, SERIAL, MP_G3_PHRASE_DEFAULT, 0U, 1000U,
					   &arm));

	/* Right nonce, wrong object: refused. This is what stops an arm for one
	 * G3 action being completed as a different one. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARM_MISMATCH,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 8U, 0U, SERIAL, NULL, arm.nonce,
					   1000U + MP_G3_HOLD_MS, NULL));
	/* Wrong nonce, right object: refused. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARM_MISMATCH,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 7U, 0U, SERIAL, NULL, arm.nonce ^ 1U,
					   1000U + MP_G3_HOLD_MS, NULL));

	/* And for a non-object action the tag binds it instead. */
	memset(&arm, 0, sizeof(arm));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARMED,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 0U, 200U, SERIAL, MP_G3_PHRASE_DEFAULT, 0U, 2000U,
					   &arm));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARM_MISMATCH,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 0U, 201U, SERIAL, NULL, arm.nonce,
					   2000U + MP_G3_HOLD_MS, NULL));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 0U, 200U, SERIAL, NULL, arm.nonce,
					   2000U + MP_G3_HOLD_MS, NULL));
}

static void test_g3_arm_expires(void)
{
	uint32_t sid = open_session(1000U);
	mp_gc_arm_t arm;

	memset(&arm, 0, sizeof(arm));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARMED,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, SERIAL, MP_G3_PHRASE_DEFAULT, 0U, 1000U,
					   &arm));

	/* Keep the session alive across the arm window. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 4000U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 8000U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 12000U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 16000U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 20000U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 24000U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 28000U));

	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARM_MISMATCH,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, SERIAL, NULL, arm.nonce,
					   1000U + MP_G3_WINDOW_MS + 1U, NULL));
	/* And the arm has been cleared, so re-arming is required. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 31200U));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARM_MISMATCH,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, SERIAL, NULL, arm.nonce, 31600U, NULL));
}

static void test_g3_arm_is_dropped_by_the_tick_after_its_window(void)
{
	uint32_t sid = open_session(1000U);
	mp_ilk_state_t st;
	mp_gc_arm_t arm;

	ilk_permissive(&st);
	memset(&arm, 0, sizeof(arm));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARMED,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, SERIAL, MP_G3_PHRASE_DEFAULT, 0U, 1000U,
					   &arm));
	TEST_ASSERT_NOT_EQUAL_UINT32(0U, g_c.sess.arm_nonce);

	/* Ticks inside the window leave it armed. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 2000U));
	TEST_ASSERT_TRUE(mp_ovr_tick(&g_c, &st, 2000U) >= 0);
	TEST_ASSERT_NOT_EQUAL_UINT32(0U, g_c.sess.arm_nonce);

	/* Walk the keepalive forward so it is the arm window, not the dead-man,
	 * that expires the arm. */
	{
		uint32_t t;

		for (t = 6000U; t <= 34000U; t += 4000U) {
			TEST_ASSERT_EQUAL_INT(0,
					      mp_ovr_keepalive(&g_c, sid, t));
			TEST_ASSERT_TRUE(mp_ovr_tick(&g_c, &st, t) >= 0);
		}
	}
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.sess.arm_nonce);
}

static void test_g3_phrase_is_replaceable(void)
{
	uint32_t sid = open_session(1000U);
	mp_gc_arm_t arm;

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_set_phrase(&g_c, "DO IT"));
	memset(&arm, 0, sizeof(arm));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_PHRASE,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, SERIAL, MP_G3_PHRASE_DEFAULT, 0U, 1000U,
					   &arm));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARMED,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, SERIAL, "DO IT", 0U, 1000U, &arm));

	/* NULL and "" restore the default. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_set_phrase(&g_c, NULL));
	mp_ovr_disarm(&g_c);
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARMED,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, SERIAL, MP_G3_PHRASE_DEFAULT, 0U, 1000U,
					   &arm));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_set_phrase(&g_c, ""));
	mp_ovr_disarm(&g_c);
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARMED,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, SERIAL, MP_G3_PHRASE_DEFAULT, 0U, 1000U,
					   &arm));
}

/**
 * The whole escalation as a matrix, so a reordering cannot pass unnoticed.
 *
 * The `role` column is the §5.3 floor. Note the ordering rows: a session check
 * outranks a role check, and a role check outranks both confirmations — so an
 * operator holding a valid serial is told "need-role", never "ok".
 */
static void test_guard_escalation_matrix(void)
{
	static const struct {
		uint8_t guard;
		bool session;
		uint8_t role;
		bool fresh;
		const char *confirm;
		const char *phrase;
		int expect;
	} m[] = {
		/* G0 ignores everything else, role included. */
		{ MP_GUARD_G0, false, MP_ROLE_NONE, false, NULL, NULL,
		  MP_GC_OK },
		{ MP_GUARD_G0, false, MP_ROLE_NONE, false, "junk", "junk",
		  MP_GC_OK },
		{ MP_GUARD_G0, true, MP_ROLE_NONE, false, NULL, NULL,
		  MP_GC_OK },
		/* G1: a fresh session, operator or better. */
		{ MP_GUARD_G1, false, MP_ROLE_ADMIN, false, NULL, NULL,
		  MP_GC_NEED_SESSION },
		{ MP_GUARD_G1, true, MP_ROLE_NONE, true, NULL, NULL,
		  MP_GC_NEED_ROLE },
		{ MP_GUARD_G1, true, MP_ROLE_VIEWER, true, NULL, NULL,
		  MP_GC_NEED_ROLE },
		{ MP_GUARD_G1, true, MP_ROLE_OPERATOR, true, NULL, NULL,
		  MP_GC_OK },
		{ MP_GUARD_G1, true, MP_ROLE_ADMIN, true, NULL, NULL,
		  MP_GC_OK },
		/* Staleness outranks the role: the session is gone either way. */
		{ MP_GUARD_G1, true, MP_ROLE_ADMIN, false, NULL, NULL,
		  MP_GC_STALE },
		{ MP_GUARD_G1, true, MP_ROLE_NONE, false, NULL, NULL,
		  MP_GC_STALE },
		/* G2: G1 plus admin plus the typed serial, in that order. */
		{ MP_GUARD_G2, false, MP_ROLE_ADMIN, false, SERIAL, NULL,
		  MP_GC_NEED_SESSION },
		{ MP_GUARD_G2, true, MP_ROLE_ADMIN, false, SERIAL, NULL,
		  MP_GC_STALE },
		{ MP_GUARD_G2, true, MP_ROLE_OPERATOR, true, SERIAL, NULL,
		  MP_GC_NEED_ROLE },
		{ MP_GUARD_G2, true, MP_ROLE_VIEWER, true, SERIAL, NULL,
		  MP_GC_NEED_ROLE },
		{ MP_GUARD_G2, true, MP_ROLE_ADMIN, true, NULL, NULL,
		  MP_GC_NEED_SERIAL },
		{ MP_GUARD_G2, true, MP_ROLE_ADMIN, true, "nope", NULL,
		  MP_GC_NEED_SERIAL },
		{ MP_GUARD_G2, true, MP_ROLE_ADMIN, true, SERIAL, NULL,
		  MP_GC_OK },
		/* G3: G2 plus the typed phrase; both are required to arm. */
		{ MP_GUARD_G3, false, MP_ROLE_ADMIN, false, SERIAL,
		  MP_G3_PHRASE_DEFAULT, MP_GC_NEED_SESSION },
		{ MP_GUARD_G3, true, MP_ROLE_ADMIN, false, SERIAL,
		  MP_G3_PHRASE_DEFAULT, MP_GC_STALE },
		{ MP_GUARD_G3, true, MP_ROLE_OPERATOR, true, SERIAL,
		  MP_G3_PHRASE_DEFAULT, MP_GC_NEED_ROLE },
		{ MP_GUARD_G3, true, MP_ROLE_ADMIN, true, NULL,
		  MP_G3_PHRASE_DEFAULT, MP_GC_NEED_SERIAL },
		{ MP_GUARD_G3, true, MP_ROLE_ADMIN, true, SERIAL, NULL,
		  MP_GC_NEED_PHRASE },
		{ MP_GUARD_G3, true, MP_ROLE_ADMIN, true, SERIAL, "wrong",
		  MP_GC_NEED_PHRASE },
		{ MP_GUARD_G3, true, MP_ROLE_ADMIN, true,
		  MP_G3_PHRASE_DEFAULT, SERIAL, MP_GC_NEED_SERIAL },
		{ MP_GUARD_G3, true, MP_ROLE_ADMIN, true, SERIAL,
		  MP_G3_PHRASE_DEFAULT, MP_GC_ARMED },
	};
	size_t i;

	for (i = 0U; i < (sizeof(m) / sizeof(m[0])); i++) {
		uint32_t sid = 0U;
		uint32_t now = 1000U;
		mp_gc_arm_t arm;
		char msg[64];

		TEST_ASSERT_EQUAL_INT(0, mp_ovr_init(&g_c, apply_cb, NULL,
						    evt_cb, NULL, SERIAL));
		TEST_ASSERT_EQUAL_INT(0, mp_ovr_set_link(&g_c, true, now));
		if (m[i].session) {
			sid = open_session_as(now, m[i].role, USER);
			if (!m[i].fresh) {
				now += MP_KEEPALIVE_TTL_MS + 1U;
			}
		} else {
			sid = 1U; /* an id no session ever had */
		}

		memset(&arm, 0, sizeof(arm));
		(void)snprintf(msg, sizeof(msg), "row %u", (unsigned int)i);
		TEST_ASSERT_EQUAL_INT_MESSAGE(m[i].expect,
					      mp_ovr_guard(&g_c, m[i].guard,
							   sid, 3U, 0U,
							   m[i].confirm,
							   m[i].phrase, 0U,
							   now, &arm),
					      msg);
	}
}

/* ============================================== §5.5 interlocks ========== */

static void test_ilk_no_interlocks_passes_through(void)
{
	size_t obj = obj_of("ui.identify");
	mp_ilk_res_t res;

	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 1, NULL, 0U, &res));
	TEST_ASSERT_EQUAL_INT32(1, res.value);
	TEST_ASSERT_FALSE(res.clamped);
	TEST_ASSERT_EQUAL_UINT32(0U, res.failed);
}

static void test_ilk_null_state_refuses_every_guarded_object(void)
{
	size_t i;
	size_t n = mp_obj_count();
	unsigned int checked = 0U;

	for (i = 0U; i < n; i++) {
		const mp_obj_t *o = mp_obj_at(i);
		mp_ilk_res_t res;

		if (o->ilk == 0U) {
			continue;
		}
		checked++;
		TEST_ASSERT_EQUAL_INT_MESSAGE(-EPERM,
					      mp_ilk_eval(i, 1, NULL, 0U, &res),
					      o->id);
		/* The reported bit is one the object actually declares. */
		TEST_ASSERT_NOT_EQUAL_MESSAGE(0U, res.failed, o->id);
		TEST_ASSERT_EQUAL_UINT32_MESSAGE(res.failed,
						 res.failed & o->ilk, o->id);
		TEST_ASSERT_NOT_NULL(mp_ilk_name(res.failed));
	}
	TEST_ASSERT_TRUE(checked > 10U);
}

static void test_ilk_argument_validation(void)
{
	mp_ilk_res_t res;

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_ilk_eval(mp_obj_count(), 0, NULL, 0U, &res));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_ilk_eval(0U, 0, NULL, 0U, NULL));
}

/* --- the VCC_RB rules, in detail --------------------------------------- */

static void test_ilk_vcc_rb_is_clamped_to_the_configured_ceiling(void)
{
	size_t obj = obj_of("pwr.rb.vset_mv");
	mp_ilk_state_t st;
	mp_ilk_res_t res;

	ilk_permissive(&st);
	st.rb_vmax_mv = 15000U;

	/* Below the ceiling: untouched. */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 12000, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT32(12000, res.value);
	TEST_ASSERT_FALSE(res.clamped);
	TEST_ASSERT_TRUE(res.verify);

	/* Exactly at the ceiling: untouched. */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 15000, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT32(15000, res.value);
	TEST_ASSERT_FALSE(res.clamped);

	/* Above it: clamped down, not refused — an operator asking for 24 V on a
	 * 15 V unit gets 15 V and is told it was clamped. */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 24450, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT32(15000, res.value);
	TEST_ASSERT_TRUE(res.clamped);

	/* Below the object's own floor: clamped up to the electrical minimum. */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 0, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT32(mp_obj_at(obj)->min, res.value);
	TEST_ASSERT_TRUE(res.clamped);

	/* A zero ceiling means the glue does not know the envelope in force —
	 * sts_pwrseq_rb_vmax_mv() answering 0 for a sequencer that has not
	 * started. Refuse; do not fall back to the electrical maximum. */
	st.rb_vmax_mv = 0U;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(obj, 12000, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_RB_VMAX, res.failed);
}

static void test_ilk_vcc_rb_refuses_while_the_ov_latch_is_set(void)
{
	size_t vset = obj_of("pwr.rb.vset_mv");
	size_t en = obj_of("pwr.rb.en");
	size_t gate = obj_of("pwr.rb.gate");
	mp_ilk_state_t st;
	mp_ilk_res_t res;

	ilk_permissive(&st);
	st.rb_ov_latched = true;

	/* Any non-zero request on the rubidium chain is refused... */
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(vset, 12000, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_RB_OV, res.failed);
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(en, 1, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_RB_OV, res.failed);
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(gate, 1, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_RB_OV, res.failed);

	/* ...but turning the chain *off* is always permitted. */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(en, 0, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(gate, 0, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(vset, 0, &st, 0U, &res));
}

static void test_ilk_digipot_code_is_clamped_to_the_permitted_window(void)
{
	size_t obj = obj_of("pwr.rb.pot.code");
	mp_ilk_state_t st;
	mp_ilk_res_t res;

	ilk_permissive(&st);
	st.rb_code_min = 10U;
	st.rb_code_max = 120U;

	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 60, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT32(60, res.value);
	TEST_ASSERT_FALSE(res.clamped);

	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 255, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT32(120, res.value);
	TEST_ASSERT_TRUE(res.clamped);

	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 0, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT32(10, res.value);
	TEST_ASSERT_TRUE(res.clamped);

	/* An inverted window means the glue could not compute one: refuse. */
	st.rb_code_min = 200U;
	st.rb_code_max = 10U;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(obj, 60, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_RB_VMAX, res.failed);
}

static void test_ilk_rb_power_needs_ocxo_warm_and_supercaps(void)
{
	size_t obj = obj_of("pwr.rb.en");
	mp_ilk_state_t st;
	mp_ilk_res_t res;

	ilk_permissive(&st);

	/* Both preconditions true: permitted. */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 1, &st, 0U, &res));

	/* Either missing: refused (FMT §5.5, PoE-budget staggering). */
	st.ocxo_warm = false;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(obj, 1, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_RB_WARM, res.failed);

	ilk_permissive(&st);
	st.supercaps_ok = false;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(obj, 1, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_RB_WARM, res.failed);

	ilk_permissive(&st);
	st.ocxo_warm = false;
	st.supercaps_ok = false;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(obj, 1, &st, 0U, &res));

	/* Turning it off never needs a warm oven. */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 0, &st, 0U, &res));
}

/* --- the rest of §5.5 --------------------------------------------------- */

static void test_ilk_fan_floor_clamps_upward(void)
{
	size_t obj = obj_of("sys.fan.duty");
	mp_ilk_state_t st;
	mp_ilk_res_t res;

	ilk_permissive(&st);
	st.fan_floor_pct = 40U;

	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 0, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT32(40, res.value);
	TEST_ASSERT_TRUE(res.clamped);

	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 40, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT32(40, res.value);
	TEST_ASSERT_FALSE(res.clamped);

	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 90, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT32(90, res.value);
	TEST_ASSERT_FALSE(res.clamped);
}

static void test_ilk_relay_cannot_be_forced_over_a_live_fault(void)
{
	size_t obj = obj_of("ref.relay.hold");
	mp_ilk_state_t st;
	mp_ilk_res_t res;

	ilk_permissive(&st);
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 1, &st, 0U, &res));

	st.relay_blocked = true;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(obj, 1, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_RELAY_OK, res.failed);

	/* De-energizing it is always allowed — that is the fail-safe direction. */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 0, &st, 0U, &res));
}

static void test_ilk_display_minimum_off_time(void)
{
	size_t obj = obj_of("pwr.disp.en");
	mp_ilk_state_t st;
	mp_ilk_res_t res;

	ilk_permissive(&st);
	st.disp_on = false;
	st.disp_changed_ms = 10000U;

	/* Too soon after the rail went off. */
	TEST_ASSERT_EQUAL_INT(-EPERM,
			      mp_ilk_eval(obj, 1, &st,
					  10000U + MP_DISP_MIN_OFF_MS - 1U,
					  &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_DISP_OFF, res.failed);

	/* At the boundary: permitted. */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 1, &st,
					     10000U + MP_DISP_MIN_OFF_MS,
					     &res));

	/* Turning it off is never rate-limited. */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 0, &st, 10000U, &res));

	/* Already on: re-asserting is not an off-time question. */
	st.disp_on = true;
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 1, &st, 10000U, &res));
}

static void test_ilk_wdt_arm_needs_a_healthy_liveness_gate(void)
{
	size_t en = obj_of("sys.wdt.en");
	mp_ilk_state_t st;
	mp_ilk_res_t res;

	ilk_permissive(&st);
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(en, 1, &st, 0U, &res));

	st.liveness_ok = false;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(en, 1, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_WDT_LIVE, res.failed);

	/* Disarming is allowed regardless — and it has to be, because it is the
	 * state a WDI pulse requires and the liveness gate is not observable
	 * from the console today (mp_glue.c, prov_ilk). */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(en, 0, &st, 0U, &res));

	/* The liveness gate says nothing about the WDI pulse. */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj_of("sys.wdt.kick"), 1, &st, 0U,
					     &res));
}

/**
 * THE ASSERTION 4b EXISTS FOR: a WDI pulse is refused while the watchdog is on.
 *
 * `sys.wdt.kick` used to carry MP_ILK_WDT_LIVE, and on a PULSE object that is
 * inverted in effect. m_obj_pulse() evaluates every pulse against a hardcoded
 * request of 1 (mp_rpc.c), and MP_ILK_WDT_LIVE refuses only `req != 0` — so the
 * edge was refused precisely when the supervisor was withholding kicks and WDI
 * was idle (harmless), and granted precisely when the supervisor was kicking on
 * the TPS3430's 920-1360 ms cadence. A console edge has an arbitrary phase
 * against that cadence; one landing inside tWDL(min) = 680 ms of the last kick
 * is a RUNAWAY fault — WDO_N asserts for ~200 ms, drives POE_KILL, and the board
 * drops its own PoE port (docs/sts1000_external_wdt.md §4).
 *
 * MP_ILK_WDT_OFF is the replacement and it has no `req` term at all: a WDI edge
 * IS an assertion, there is no de-assert direction to exempt, and the only safe
 * state for one is a watchdog that is not watching.
 */
static void test_ilk_wdi_pulse_is_refused_while_the_watchdog_is_armed(void)
{
	size_t kick = obj_of("sys.wdt.kick");
	mp_ilk_state_t st;
	mp_ilk_res_t res;

	/* Armed: refused, and named as the interlock a technician can act on. */
	ilk_permissive(&st);
	st.wdt_off = false;
	TEST_ASSERT_EQUAL_INT_MESSAGE(
		-EPERM, mp_ilk_eval(kick, 1, &st, 0U, &res),
		"a WDI edge was granted against an ARMED watchdog; that is the "
		"board cold-cycling itself");
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_WDT_OFF, res.failed);
	TEST_ASSERT_EQUAL_STRING("wdt.off", mp_ilk_name(res.failed));

	/* Disabled: granted — this is the bench capability the bit buys. */
	st.wdt_off = true;
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(kick, 1, &st, 0U, &res));

	/*
	 * The liveness gate is orthogonal now. An unhealthy gate is exactly when
	 * the supervisor stops kicking, so a pulse against a DISABLED watchdog
	 * stays permitted, and a healthy gate does not buy a pulse against an
	 * ARMED one.
	 */
	st.liveness_ok = false;
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(kick, 1, &st, 0U, &res));
	st.liveness_ok = true;
	st.wdt_off = false;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(kick, 1, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_WDT_OFF, res.failed);

	/*
	 * A glue that cannot produce a snapshot refuses too: mp_ilk_eval()'s
	 * NULL-state path reports the lowest declared bit, which for this row is
	 * the only bit. An unobservable watchdog is a watching one.
	 */
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(kick, 1, NULL, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_WDT_OFF, res.failed);
}

static void test_ilk_clock_mux_guard(void)
{
	size_t obj = obj_of("ref.mux.sel");
	mp_ilk_state_t st;
	mp_ilk_res_t res;

	ilk_permissive(&st);

	/* Input B needs both guards (spec §3.5). */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 1, &st, 0U, &res));

	st.extref_ok = false;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(obj, 1, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_MUX_GUARD, res.failed);
	/* Selecting the OCXO is the fail-safe direction and never refused. */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 0, &st, 0U, &res));

	ilk_permissive(&st);
	st.rb_lock = false;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(obj, 1, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 0, &st, 0U, &res));
}

static void test_ilk_dac_needs_the_loop_parked(void)
{
	size_t mv = obj_of("ref.ocxo.vc_mv");
	size_t code = obj_of("ref.ocxo.dac_code");
	mp_ilk_state_t st;
	mp_ilk_res_t res;

	ilk_permissive(&st);
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(mv, 1650, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(code, 2048, &st, 0U, &res));

	st.disc_parked = false;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(mv, 1650, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_DAC_PARK, res.failed);
	/* Unlike the others, this one refuses in *both* directions: writing 0 mV
	 * to a live loop is exactly as wrong as writing 3300. */
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(mv, 0, &st, 0U, &res));
}

/**
 * The two Vc views may not be leased at once (MP_ILK_DAC_SOLE).
 *
 * They are two names for DAC1_OUT1. Held together with different values there
 * is no correct pin state and one of the two leases must be reporting a Vc the
 * oven has never seen — so the second grant is refused rather than resolved.
 * The refusal reads the OTHER view of the pair, which is what lets a lease be
 * RE-granted (mp_ovr_grant() replaces a lease on a second grant for the same
 * object, and that must not be refused by the object's own lease).
 */
static void test_ilk_the_two_vc_views_exclude_each_other(void)
{
	size_t mv = obj_of("ref.ocxo.vc_mv");
	size_t code = obj_of("ref.ocxo.dac_code");
	mp_ilk_state_t st;
	mp_ilk_res_t res;

	ilk_permissive(&st);
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(mv, 1650, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(code, 2048, &st, 0U, &res));

	/* The raw-code view is leased: the millivolt view is refused. */
	st.dac_code_held = true;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(mv, 1650, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_DAC_SOLE, res.failed);
	/* ...and so is 0, because the refusal is about WHO drives the pin and
	 * not about the value. */
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(mv, 0, &st, 0U, &res));
	/* But re-granting the view that already holds it is not refused. */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(code, 3000, &st, 0U, &res));

	ilk_permissive(&st);
	st.dac_mv_held = true;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(code, 2048, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_DAC_SOLE, res.failed);
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(mv, 1700, &st, 0U, &res));
}

/**
 * The park may not be RELEASED out from under a Vc override (MP_ILK_DAC_IDLE).
 *
 * Releasing it resumes the loop, and a resumed loop rewrites DAC1_OUT1 every
 * second — so a technician who still holds a Vc lease would be left with a
 * control that reports a value the pin no longer carries, and two writers
 * arguing over PA4 in between.
 *
 * Only `req == 0` is refused. TAKING the park while a Vc override stands is
 * harmless: the loop is already parked, which is that override's own
 * precondition, so refusing it would block a technician from re-arming the very
 * interlock their lease depends on.
 */
static void test_ilk_park_release_is_refused_under_a_vc_override(void)
{
	size_t park = obj_of("ref.disc.park");
	mp_ilk_state_t st;
	mp_ilk_res_t res;

	ilk_permissive(&st);
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(park, 1, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(park, 0, &st, 0U, &res));

	st.dac_mv_held = true;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(park, 0, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_DAC_IDLE, res.failed);
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(park, 1, &st, 0U, &res));

	ilk_permissive(&st);
	st.dac_code_held = true;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(park, 0, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_DAC_IDLE, res.failed);
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(park, 1, &st, 0U, &res));

	/* Both at once is still one refusal, and still only of the release. */
	st.dac_mv_held = true;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(park, 0, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(park, 1, &st, 0U, &res));
}

/**
 * The park is not gated on being parked, and the Vc objects are not gated on
 * each other's absence by MP_ILK_DAC_PARK.
 *
 * A cross-check on the manifest wiring rather than on the evaluator: if
 * `ref.disc.park` had picked up MP_ILK_DAC_PARK it would demand the very state
 * it exists to produce, which is the unsatisfiable-interlock defect this whole
 * group was blocked on.
 */
static void test_the_park_object_does_not_demand_its_own_effect(void)
{
	const mp_obj_t *park = mp_obj_at(obj_of("ref.disc.park"));
	const mp_obj_t *mv = mp_obj_at(obj_of("ref.ocxo.vc_mv"));
	const mp_obj_t *code = mp_obj_at(obj_of("ref.ocxo.dac_code"));

	TEST_ASSERT_EQUAL_UINT32(0U, park->ilk & MP_ILK_DAC_PARK);
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_DAC_IDLE, park->ilk & MP_ILK_DAC_IDLE);

	/* And both Vc views carry BOTH of their guards, not one of them. */
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_DAC_PARK | MP_ILK_DAC_SOLE,
				 mv->ilk & (MP_ILK_DAC_PARK | MP_ILK_DAC_SOLE));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_DAC_PARK | MP_ILK_DAC_SOLE,
				 code->ilk &
					 (MP_ILK_DAC_PARK | MP_ILK_DAC_SOLE));
}

static void test_ilk_tunnel_reports_a_consequence_not_a_refusal(void)
{
	size_t obj = obj_of("gnss.tunnel");
	mp_ilk_state_t st;
	mp_ilk_res_t res;

	ilk_permissive(&st);

	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 1, &st, 0U, &res));
	TEST_ASSERT_TRUE(res.tunnel);
	TEST_ASSERT_EQUAL_UINT32(0U, res.failed);

	/* Closing the tunnel is not a suspension. */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(obj, 0, &st, 0U, &res));
	TEST_ASSERT_FALSE(res.tunnel);
}

/* ============================================== leases and the dead-man == */

static void test_grant_and_release(void)
{
	size_t obj = obj_of("ui.disp.bl");
	mp_ilk_state_t st;
	mp_ilk_res_t res;
	uint32_t sid;
	const mp_lease_t *l;

	ilk_permissive(&st);
	sid = open_session(1000U);

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 55, 30000U, sid, &st,
					      1000U, &res));
	TEST_ASSERT_EQUAL_INT32(55, res.value);
	TEST_ASSERT_EQUAL_size_t(1U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT(1U, g_apply_n);
	TEST_ASSERT_FALSE(g_apply[0].release);
	TEST_ASSERT_EQUAL_INT32(55, g_apply[0].value);
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_GRANT));

	l = mp_ovr_lease(&g_c, obj);
	TEST_ASSERT_NOT_NULL(l);
	TEST_ASSERT_EQUAL_UINT32(sid, l->sid);
	TEST_ASSERT_EQUAL_INT32(55, l->value);
	TEST_ASSERT_EQUAL_UINT32(31000U, l->deadline_ms);

	/* Regranting the same object replaces the lease, not adds one. */
	logs_clear();
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 70, 0U, sid, &st,
					      1500U, &res));
	TEST_ASSERT_EQUAL_size_t(1U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_INT32(70, mp_ovr_lease(&g_c, obj)->value);
	TEST_ASSERT_EQUAL_UINT32(1500U + MP_LEASE_TTL_DEFAULT_MS,
				 mp_ovr_lease(&g_c, obj)->deadline_ms);

	/* Release returns the object to firmware. */
	logs_clear();
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_release(&g_c, obj, sid, 2000U));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT(1U, releases_of(obj));
	TEST_ASSERT_NULL(mp_ovr_lease(&g_c, obj));

	TEST_ASSERT_EQUAL_INT(-ENOENT, mp_ovr_release(&g_c, obj, sid, 2000U));
}

static void test_release_belongs_to_the_owner(void)
{
	size_t obj = obj_of("ui.disp.bl");
	mp_ilk_state_t st;
	uint32_t sid;

	ilk_permissive(&st);
	sid = open_session(1000U);
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 55, 0U, sid, &st,
					      1000U, NULL));
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      mp_ovr_release(&g_c, obj, sid + 1U, 1000U));
	TEST_ASSERT_EQUAL_size_t(1U, mp_ovr_active(&g_c));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_ovr_release(&g_c, mp_obj_count(), sid, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_ovr_release(NULL, obj, sid, 0U));
}

static void test_grant_rejections(void)
{
	mp_ilk_state_t st;
	mp_ilk_res_t res;
	uint32_t sid;
	size_t bl = obj_of("ui.disp.bl");
	size_t sensor = obj_of("sensor.rail.poe");
	size_t pulse = obj_of("gnss.reset");

	ilk_permissive(&st);
	sid = open_session(1000U);

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_ovr_grant(NULL, bl, 1, 0U, sid, &st, 1000U,
					   &res));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_ovr_grant(&g_c, mp_obj_count(), 1, 0U, sid, &st,
					   1000U, &res));
	/* A sensor is not overridable, nor is a pulse-only object. */
	TEST_ASSERT_EQUAL_INT(-ENOTSUP,
			      mp_ovr_grant(&g_c, sensor, 1, 0U, sid, &st, 1000U,
					   &res));
	TEST_ASSERT_EQUAL_INT(-ENOTSUP,
			      mp_ovr_grant(&g_c, pulse, 1, 0U, sid, &st, 1000U,
					   &res));
	/* Outside the envelope. */
	TEST_ASSERT_EQUAL_INT(-ERANGE,
			      mp_ovr_grant(&g_c, bl, 101, 0U, sid, &st, 1000U,
					   &res));
	TEST_ASSERT_EQUAL_INT(-ERANGE,
			      mp_ovr_grant(&g_c, bl, -1, 0U, sid, &st, 1000U,
					   &res));
	/* A bool takes only 0 or 1. */
	TEST_ASSERT_EQUAL_INT(-ERANGE,
			      mp_ovr_grant(&g_c, obj_of("ref.term.en"), 2, 0U,
					   sid, &st, 1000U, &res));
	/* No valid session. */
	TEST_ASSERT_EQUAL_INT(-ENOLINK,
			      mp_ovr_grant(&g_c, bl, 50, 0U, sid + 1U, &st,
					   1000U, &res));
	TEST_ASSERT_EQUAL_INT(-ENOLINK,
			      mp_ovr_grant(&g_c, bl, 50, 0U, sid, &st,
					   1000U + MP_KEEPALIVE_TTL_MS + 1U,
					   &res));
}

static void test_lease_ttl_is_clamped(void)
{
	size_t obj = obj_of("ui.disp.bl");
	mp_ilk_state_t st;
	uint32_t sid;

	ilk_permissive(&st);
	sid = open_session(1000U);

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 50, 0xFFFFFFFFU, sid,
					      &st, 1000U, NULL));
	TEST_ASSERT_EQUAL_UINT32(1000U + MP_LEASE_TTL_MAX_MS,
				 mp_ovr_lease(&g_c, obj)->deadline_ms);
}

static void test_lease_table_full(void)
{
	mp_ilk_state_t st;
	uint32_t sid;
	size_t i;
	unsigned int granted = 0U;

	ilk_permissive(&st);
	sid = open_session(1000U);

	/* Fill the table with distinct overridable objects, granting each its
	 * own minimum so no interlock refuses (the permissive state passes the
	 * clamping ones and every refusing one is satisfied at value 0). */
	for (i = 0U; (i < mp_obj_count()) && (granted < MP_LEASE_MAX); i++) {
		const mp_obj_t *o = mp_obj_at(i);

		if ((o->flags & MP_OF_OVERRIDE) == 0U) {
			continue;
		}
		if (mp_ovr_grant(&g_c, i, o->min, 0U, sid, &st, 1000U, NULL) ==
		    0) {
			granted++;
		}
	}
	TEST_ASSERT_EQUAL_UINT(MP_LEASE_MAX, granted);
	TEST_ASSERT_EQUAL_size_t(MP_LEASE_MAX, mp_ovr_active(&g_c));

	/* One more distinct object cannot fit. */
	for (; i < mp_obj_count(); i++) {
		const mp_obj_t *o = mp_obj_at(i);

		if ((o->flags & MP_OF_OVERRIDE) != 0U) {
			TEST_ASSERT_EQUAL_INT_MESSAGE(
				-ENOSPC,
				mp_ovr_grant(&g_c, i, o->min, 0U, sid, &st,
					     1000U, NULL),
				o->id);
			break;
		}
	}
	TEST_ASSERT_TRUE_MESSAGE(i < mp_obj_count(),
				 "manifest has no spare overridable object");

	/* Re-granting one that is already leased still works: it replaces. */
	{
		size_t bl = obj_of("ui.disp.bl");

		if (mp_ovr_lease(&g_c, bl) != NULL) {
			TEST_ASSERT_EQUAL_INT(0,
					      mp_ovr_grant(&g_c, bl, 33, 0U, sid,
							   &st, 1000U, NULL));
			TEST_ASSERT_EQUAL_size_t(MP_LEASE_MAX,
						 mp_ovr_active(&g_c));
		}
	}
}

static void test_apply_refusal_is_a_veto_and_leaves_no_lease(void)
{
	size_t obj = obj_of("ui.disp.bl");
	mp_ilk_state_t st;
	uint32_t sid;

	ilk_permissive(&st);
	sid = open_session(1000U);

	g_apply_rc = -EIO;
	TEST_ASSERT_EQUAL_INT(-EACCES, mp_ovr_grant(&g_c, obj, 50, 0U, sid, &st,
						    1000U, NULL));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_NULL(mp_ovr_lease(&g_c, obj));
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_VETO));
	TEST_ASSERT_EQUAL_UINT32(1U, g_c.vetoes);

	/*
	 * And a refusal on a *replacement* must not destroy the existing lease
	 * silently either. A standing lease is a COMMANDED PIN, not a
	 * half-claimed slot: freeing it without running the release left the
	 * actuator wherever the previous grant put it, unowned and invisible.
	 * So the slot is freed AND the release runs AND both facts reach the
	 * subscriber — the release that says the object went back to
	 * firmware-automatic, and the veto that says the new value was refused.
	 */
	g_apply_rc = 0;
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 50, 0U, sid, &st,
					      1000U, NULL));
	logs_clear();
	g_apply_rc = -EIO;
	TEST_ASSERT_EQUAL_INT(-EACCES, mp_ovr_grant(&g_c, obj, 60, 0U, sid, &st,
						    1000U, NULL));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, releases_of(obj),
		"a vetoed re-grant freed the slot without reverting the pin");
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_RELEASE));
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_VETO));
}

/**
 * An apply that answers -ENOTSUP is NOT a veto, and the difference is the whole
 * sentence a technician reads.
 *
 * MP_E_VETO says firmware's safety supervision refused a request it understood,
 * so it lands on the event channel, moves `vetoes`, and sends whoever is at the
 * bench looking for the interlock or the fault behind it. On an object with
 * nothing wired there is no supervisor to find: the veto counter would be
 * counting unimplemented objects and channel 0x09 — which the veto work exists
 * to make meaningful — would be carrying noise.
 *
 * Every clause below was false before this change: the return was -EACCES
 * (MP_E_VETO), one MP_OVR_EV_VETO was emitted, and `vetoes` moved. The lease
 * half is unchanged and is asserted anyway, because "not a veto" must not have
 * been bought by leaving a half-claimed slot behind.
 */
static void test_an_unwired_apply_is_notsup_and_not_a_veto(void)
{
	size_t obj = obj_of("ui.disp.bl");
	mp_ilk_state_t st;
	uint32_t sid;

	ilk_permissive(&st);
	sid = open_session(1000U);

	g_apply_rc = -ENOTSUP;
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, mp_ovr_grant(&g_c, obj, 50, 0U, sid,
						     &st, 1000U, NULL));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_NULL(mp_ovr_lease(&g_c, obj));
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, events_of((uint8_t)MP_OVR_EV_VETO),
		"an unimplemented object emitted a safety veto on channel 9");
	TEST_ASSERT_EQUAL_UINT32_MESSAGE(
		0U, g_c.vetoes,
		"an unimplemented object was counted as a firmware veto");

	/*
	 * The same refusal against a *standing* lease frees the slot without
	 * claiming a veto either — and the release runs, so the object really
	 * did go back to firmware-automatic.
	 *
	 * That second clause was asserted by nobody until now. The suite checked
	 * mp_ovr_active() and the veto counters, both of which were already true
	 * of the defect: the slot really was zeroed, in place, with no
	 * apply(obj, NULL) anywhere — the actuator stayed where the previous
	 * grant put it and nothing was left in the table to move it back. The
	 * comment claiming otherwise is why the hole survived review.
	 */
	g_apply_rc = 0;
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 50, 0U, sid, &st,
					      1000U, NULL));
	logs_clear();
	g_apply_rc = -ENOTSUP;
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, mp_ovr_grant(&g_c, obj, 60, 0U, sid,
						     &st, 1000U, NULL));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, releases_of(obj),
		"the standing lease was deleted without reverting the pin");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, events_of((uint8_t)MP_OVR_EV_RELEASE),
		"the lease vanished with nothing on channel 0x09 to say so");
	TEST_ASSERT_EQUAL_UINT(0U, events_of((uint8_t)MP_OVR_EV_VETO));
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.vetoes);

	/* And a refusal firmware *understood* is still a veto: the separation is
	 * by errno, not by "any failure is now quiet". */
	g_apply_rc = -EIO;
	TEST_ASSERT_EQUAL_INT(-EACCES, mp_ovr_grant(&g_c, obj, 60, 0U, sid, &st,
						    1000U, NULL));
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_VETO));
	TEST_ASSERT_EQUAL_UINT32(1U, g_c.vetoes);
}

/**
 * An apply that answers -EBUSY is not a veto either, and here the
 * mis-attribution accused the technician of the wrong thing entirely.
 *
 * The reachable producer is `ref.rb.serial`: rb_serial_set_mode() answers
 * -EBUSY while a raw tunnel holds UART7, because throwing the K1 relay under a
 * live passthrough would corrupt whatever the host is mid-transaction with. So
 * the sequence that hits it is hold `ref.rb.serial`, open `ref.rb.tunnel`,
 * re-command the relay — every step of which is the technician's own doing.
 *
 * Reported as -EACCES that reached the bench as MP_E_VETO: "firmware's safety
 * supervision refused you", which sends someone looking for an interlock that
 * does not exist and never names the actual remedy (close the tunnel). -EBUSY
 * carries its own identity through to MP_E_BUSY, the retryable code, which is
 * what the glue's own comment at the `ref.rb.serial` apply already promised.
 *
 * The errno IS the whole decision — mp_map_errno() maps -EACCES to MP_E_VETO
 * and -EBUSY to MP_E_BUSY with nothing in between — so it is asserted here,
 * where the engine decides it, rather than through the RPC layer.
 */
static void test_a_busy_resource_is_busy_and_not_a_veto(void)
{
	size_t obj = obj_of("ui.disp.bl");
	mp_ilk_state_t st;
	uint32_t sid;

	ilk_permissive(&st);
	sid = open_session(1000U);

	/* No lease yet: nothing was claimed, so nothing is reverted. */
	g_apply_rc = -EBUSY;
	TEST_ASSERT_EQUAL_INT(-EBUSY, mp_ovr_grant(&g_c, obj, 50, 0U, sid, &st,
						   1000U, NULL));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_NULL(mp_ovr_lease(&g_c, obj));
	TEST_ASSERT_EQUAL_UINT(0U, releases_of(obj));
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, events_of((uint8_t)MP_OVR_EV_VETO),
		"a busy resource emitted a safety veto on channel 9");
	TEST_ASSERT_EQUAL_UINT32_MESSAGE(
		0U, g_c.vetoes,
		"a busy resource was counted as a firmware veto");

	/*
	 * Over a standing lease it is the Finding-B shape as well: the lease
	 * goes, the release runs, the subscriber is told — and it is still not
	 * a veto. This is the exact sequence a technician reaches by holding
	 * ref.rb.serial and then opening ref.rb.tunnel.
	 */
	g_apply_rc = 0;
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 50, 0U, sid, &st,
					      1000U, NULL));
	logs_clear();
	g_apply_rc = -EBUSY;
	TEST_ASSERT_EQUAL_INT(-EBUSY, mp_ovr_grant(&g_c, obj, 60, 0U, sid, &st,
						   1000U, NULL));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, releases_of(obj),
		"a busy re-grant freed the slot without reverting the pin");
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_RELEASE));
	TEST_ASSERT_EQUAL_UINT(0U, events_of((uint8_t)MP_OVR_EV_VETO));
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.vetoes);

	/* And -EIO, which firmware DID understand, is still a veto. The split
	 * is by errno, not "any failure is now quiet". */
	g_apply_rc = -EIO;
	TEST_ASSERT_EQUAL_INT(-EACCES, mp_ovr_grant(&g_c, obj, 60, 0U, sid, &st,
						    1000U, NULL));
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_VETO));
	TEST_ASSERT_EQUAL_UINT32(1U, g_c.vetoes);
}

/* ================================ the other half of a release the glue refused */
/*
 * Everything above proves what THIS module does with a refused release: it
 * counts it (`release_errors`) and drops the lease anyway. That is correct — a
 * lease core could not clear because the glue was slow or busy would be worse
 * than one whose last write is still in flight — but it means the engine cannot
 * be the thing that guarantees the pin came back. Whoever refused owes it.
 *
 * On UART7 that owner is platform/rb_serial.c, and it did not pay:
 *
 *   1. `obj.override ref.rb.serial cmos`   lease slot 0, K1 -> CMOS
 *   2. `obj.override ref.rb.tunnel true`   lease slot 1, UART7 handed to the host
 *   3. USB pulled -> mp_ovr_revert_all(), which drops in SLOT ORDER, so slot 0
 *      goes first: its release lands as rb_serial_set_mode(RS232), which refuses
 *      with -EBUSY because the tunnel is still open. release_errors++, lease
 *      gone, relay still CMOS.
 *   4. Slot 1 then closes the tunnel — and closing it moved nothing.
 *
 * The board resumes with K1 in a commissioning position no lease holds and
 * FE-5680A housekeeping telemetry silently dead until somebody reboots it.
 *
 * WHY THIS IS A SOURCE SCAN. The fix belongs in the module that owns PE4, and
 * that module is Zephyr glue — devicetree GPIO specs, a UART ISR and a 20 ms
 * relay settle — so it cannot be linked into a host suite at all, and the
 * `mp_override` suite's own apply fake models the ANSWER (an errno) rather than
 * the pins. A fixture that simulated the relay would be asserting against its
 * own copy of the logic, which is how several guards in this tree went green
 * for the wrong reason. So the mechanism is read out of the source instead,
 * the way test_smear_isolation.c and test_factory_policy.c read theirs.
 *
 * It is a real scan, not a grep for a keyword: it pins BOTH halves and the
 * ordering that connects them. Deleting the latch, deleting the discharge,
 * dropping the RS-232-only guard that keeps the CMOS direction undeferred, or
 * moving the discharge above the callback retraction — where set_mode() would
 * refuse itself — each fails a different assertion below.
 */

#ifndef STS_APP_SRC_DIR
#error "STS_APP_SRC_DIR must be defined by tests/host/CMakeLists.txt"
#endif

#define SRC_MAX (64U * 1024U)

static char g_src[SRC_MAX];
static size_t g_src_len;
static char g_src_name[128];

typedef struct {
	size_t begin;
	size_t end;
} span_t;

/**
 * Read @p rel into g_src, blanking comments AND string literals in place so
 * offsets stay 1:1 with the file.
 *
 * The strings go too, unlike test_mp_deferred.c's loader: nothing quoted is
 * under test here, and this file's own LOG_WRN/LOG_INF lines name the relay
 * positions in prose. A scan that counted those would pass on a module that
 * only *talks* about restoring the relay.
 */
static void load_source(const char *rel)
{
	char path[512];
	FILE *f;
	size_t n;
	size_t i;
	enum { CODE, BLOCK, LINE, STR, CHR } state = CODE;

	(void)snprintf(path, sizeof(path), "%s/%s", STS_APP_SRC_DIR, rel);
	f = fopen(path, "rb");
	TEST_ASSERT_NOT_NULL_MESSAGE(f, path);
	n = fread(g_src, 1U, sizeof(g_src) - 1U, f);
	(void)fclose(f);
	TEST_ASSERT_TRUE_MESSAGE(n < (sizeof(g_src) - 1U),
				 "source did not fit the scan buffer");
	TEST_ASSERT_TRUE_MESSAGE(n > 4096U, "source is implausibly small");
	g_src[n] = '\0';
	g_src_len = n;
	(void)snprintf(g_src_name, sizeof(g_src_name), "%s", rel);

	for (i = 0U; i < n; i++) {
		char c = g_src[i];
		char d = ((i + 1U) < n) ? g_src[i + 1U] : '\0';

		switch (state) {
		case CODE:
			if ((c == '/') && (d == '*')) {
				state = BLOCK;
				g_src[i] = ' ';
				g_src[i + 1U] = ' ';
				i++;
			} else if ((c == '/') && (d == '/')) {
				state = LINE;
				g_src[i] = ' ';
				g_src[i + 1U] = ' ';
				i++;
			} else if (c == '"') {
				state = STR;
				g_src[i] = ' ';
			} else if (c == '\'') {
				state = CHR;
				g_src[i] = ' ';
			}
			break;
		case BLOCK:
			g_src[i] = (c == '\n') ? '\n' : ' ';
			if ((c == '*') && (d == '/')) {
				g_src[i + 1U] = ' ';
				i++;
				state = CODE;
			}
			break;
		case LINE:
			if (c == '\n') {
				state = CODE;
			} else {
				g_src[i] = ' ';
			}
			break;
		case STR:
		case CHR:
			/* The escape pair is stepped over so an escaped quote
			 * cannot end the literal early. */
			if (c == '\\') {
				g_src[i] = ' ';
				if ((i + 1U) < n) {
					g_src[i + 1U] = ' ';
				}
				i++;
				break;
			}
			if (((state == STR) && (c == '"')) ||
			    ((state == CHR) && (c == '\''))) {
				state = CODE;
			}
			g_src[i] = ' ';
			break;
		}
	}
}

static unsigned int count_in(const span_t *s, const char *needle)
{
	size_t k = strlen(needle);
	unsigned int hits = 0U;
	size_t i;

	for (i = s->begin; (k != 0U) && ((i + k) <= s->end); i++) {
		if (memcmp(&g_src[i], needle, k) == 0) {
			hits++;
		}
	}
	return hits;
}

static size_t offset_in(const span_t *s, const char *needle)
{
	size_t k = strlen(needle);
	size_t i;
	char msg[256];

	for (i = s->begin; (i + k) <= s->end; i++) {
		if (memcmp(&g_src[i], needle, k) == 0) {
			return i;
		}
	}
	(void)snprintf(msg, sizeof(msg),
		       "%s: `%s` is gone — fix this scan, do not delete the "
		       "assertion it feeds",
		       g_src_name, needle);
	TEST_FAIL_MESSAGE(msg);
	return 0U;
}

/** The braced body of the function whose definition begins with @p sig. */
static span_t fn_body(const char *sig)
{
	size_t k = strlen(sig);
	span_t s = { 0U, 0U };
	size_t at = 0U;
	unsigned int hits = 0U;
	int depth = 0;
	size_t i;
	char msg[256];

	(void)snprintf(msg, sizeof(msg),
		       "%s: cannot locate exactly one body of `%s` — fix this "
		       "scan, do not delete the assertion it feeds",
		       g_src_name, sig);

	for (i = 0U; (i + k) <= g_src_len; i++) {
		if (memcmp(&g_src[i], sig, k) == 0) {
			hits++;
			at = i;
		}
	}
	TEST_ASSERT_EQUAL_UINT_MESSAGE(1U, hits, msg);

	while ((at < g_src_len) && (g_src[at] != '{')) {
		at++;
	}
	TEST_ASSERT_TRUE_MESSAGE(at < g_src_len, msg);
	s.begin = at + 1U;

	for (i = at; i < g_src_len; i++) {
		if (g_src[i] == '{') {
			depth++;
		} else if (g_src[i] == '}') {
			depth--;
			if (depth == 0) {
				s.end = i;
				break;
			}
		}
	}
	TEST_ASSERT_TRUE_MESSAGE(s.end > s.begin, msg);
	return s;
}

/**
 * A release refused because a tunnel held UART7 must still reach the relay.
 *
 * The RULE now lives in platform/sts_rb_serial_policy.h and is EXECUTED by
 * tests/host/test_rb_serial_policy.c: which moves a live tunnel refuses, which
 * of those refusals is owed, why the RS-232 and CMOS directions are not
 * symmetric, and why an unconditional restore on close would be worse than the
 * bug it replaced. It had to move to be testable at all — rb_serial.c needs a
 * devicetree, a UART ISR and k_msleep() to link, so everything that could be
 * said about it from here was said by reading its text.
 *
 * What is left for a scan is the half no host suite can see: that rb_serial.c
 * still ASKS the policy instead of keeping a second copy of the decision, and
 * that the two steps of the close sit either side of the retraction —
 *
 *   set_mode()      hands the live tunnel state to sts_rb_move_decide() and
 *                   arms the latch under STS_RB_MOVE_OWE. The direction test
 *                   itself appears nowhere in this file: a copy here is a copy
 *                   nothing executes, and the two are then free to disagree.
 *   tunnel_close()  reads sts_rb_close_decide() BEFORE retracting `tunnel_cb`
 *                   (afterwards the policy sees no tunnel, answers
 *                   STS_RB_CLOSE_NOTHING, and the debt disappears with nothing
 *                   failing) and pays it AFTER (beforehand set_mode() refuses
 *                   itself and re-arms the latch it was called to discharge).
 */
static void test_a_release_refused_by_a_tunnel_is_paid_at_tunnel_close(void)
{
	span_t sm;
	span_t tc;
	size_t decide;
	size_t retract;

	load_source("zephyr/platform/rb_serial.c");

	/* --- half 1: the refusal asks, and honours the answer ------------- */
	sm = fn_body("int rb_serial_set_mode(uint8_t mode)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U,
		count_in(&sm, "sts_rb_move_decide(mode, rb.tunnel_cb != NULL)"),
		"rb_serial_set_mode() no longer puts the live tunnel state to "
		"the policy, so whether K1 can be thrown mid-passthrough is "
		"decided by something no suite executes");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&sm, "rb.relay.restore_rs232_on_close = true;"),
		"a release refused by the tunnel is dropped on the floor again: "
		"K1 stays where the previous grant put it, with no lease left "
		"to move it");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&sm, "case STS_RB_MOVE_OWE:") <
			offset_in(&sm,
				  "rb.relay.restore_rs232_on_close = true;"),
		"the deferred restore is being recorded outside the verdict it "
		"exists for");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&sm, "case STS_RB_MOVE_REFUSE:"),
		"the refusal that owes NOTHING is gone, so either every refused "
		"move now defers or none does — and the asymmetry is the whole "
		"design");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&sm, "mode == (uint8_t)RB_SERIAL_MODE_RS232"),
		"the RS-232-only guard was copied back into rb_serial.c, where "
		"no host suite can execute it and it is free to drift from the "
		"policy that is executed");

	/* --- half 2: the close pays it, and only once -------------------- */
	tc = fn_body("int rb_serial_tunnel_close(void)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&tc, "if (act == STS_RB_CLOSE_RESTORE) {"),
		"rb_serial_tunnel_close() no longer discharges the deferred "
		"restore, so a dead-man revert during a tunnel leaves K1 in the "
		"commissioning position");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&tc,
			     "rb_serial_set_mode((uint8_t)RB_SERIAL_MODE_RS232)"),
		"the discharge no longer commands the relay back to RS-232");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&tc, "rb.relay.restore_rs232_on_close = false;"),
		"the latch is never cleared, so one refused release arms every "
		"future tunnel close");

	/* --- the two orderings that make the discharge possible at all --- */
	decide = offset_in(&tc, "sts_rb_close_decide(rb.tunnel_cb != NULL,");
	retract = offset_in(&tc, "rb.tunnel_cb = NULL;");

	TEST_ASSERT_TRUE_MESSAGE(
		decide < retract,
		"the close asks the policy AFTER retracting tunnel_cb, where it "
		"sees no tunnel, answers STS_RB_CLOSE_NOTHING, and the debt "
		"vanishes without anything failing");
	TEST_ASSERT_TRUE_MESSAGE(
		retract < offset_in(&tc, "rb_serial_set_mode("),
		"the deferred restore moved above the tunnel_cb retraction, "
		"where set_mode() refuses itself and re-arms the latch it was "
		"called to discharge");
	TEST_ASSERT_TRUE_MESSAGE(
		retract < offset_in(&tc, "if (act == STS_RB_CLOSE_RESTORE) {"),
		"the latch is tested before the port is given back");
}

static void test_firmware_veto(void)
{
	size_t obj = obj_of("ui.disp.bl");
	mp_ilk_state_t st;
	uint32_t sid;

	ilk_permissive(&st);
	sid = open_session(1000U);
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 50, 0U, sid, &st,
					      1000U, NULL));

	logs_clear();
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_veto(&g_c, obj, "thermal shed", 1500U));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT(1U, releases_of(obj));
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_VETO));
	TEST_ASSERT_EQUAL_STRING("thermal shed", g_evt[0].reason);

	TEST_ASSERT_EQUAL_INT(-ENOENT, mp_ovr_veto(&g_c, obj, NULL, 1500U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_ovr_veto(&g_c, mp_obj_count(), NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_ovr_veto(NULL, obj, NULL, 0U));

	/* A NULL reason still names the event. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 50, 0U, sid, &st,
					      1000U, NULL));
	logs_clear();
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_veto(&g_c, obj, NULL, 1500U));
	TEST_ASSERT_EQUAL_STRING("firmware veto", g_evt[0].reason);
}

/* --- the dead-man ------------------------------------------------------- */

/** Grant one lease and return its object index. */
static size_t grant_one(uint32_t sid, uint32_t now)
{
	size_t obj = obj_of("ui.disp.bl");
	mp_ilk_state_t st;

	ilk_permissive(&st);
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 50, 0U, sid, &st, now,
					      NULL));
	return obj;
}

static void test_deadman_reverts_on_a_stale_keepalive(void)
{
	mp_ilk_state_t st;
	uint32_t sid = open_session(1000U);
	size_t obj = grant_one(sid, 1000U);

	ilk_permissive(&st);
	logs_clear();

	/* Inside the TTL: nothing happens. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_tick(&g_c, &st, 1000U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_tick(&g_c, &st,
					     1000U + MP_KEEPALIVE_TTL_MS));
	TEST_ASSERT_EQUAL_size_t(1U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT(0U, releases_of(obj));

	/* The first tick past it reverts everything, immediately. */
	TEST_ASSERT_EQUAL_INT(1, mp_ovr_tick(&g_c, &st,
					     1000U + MP_KEEPALIVE_TTL_MS + 1U));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT(1U, releases_of(obj));
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_DEADMAN));
	TEST_ASSERT_EQUAL_UINT32(1U, g_c.deadman_reverts);

	/* The session is dropped too, so a stale tool cannot keep issuing. */
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.sess.id);
}

/**
 * The spec's 2 s number, as a timing bound.
 *
 * Reversion happens on the first tick that observes the failure, so with the
 * glue ticking at MP_TICK_MAX_MS the worst case is TTL + one tick. This walks a
 * realistic tick schedule and asserts the override is gone by that deadline.
 */
static void test_deadman_revert_timing_bound(void)
{
	mp_ilk_state_t st;
	uint32_t sid = open_session(1000U);
	size_t obj = grant_one(sid, 1000U);
	const uint32_t last_keepalive = 1000U;
	const uint32_t deadline = last_keepalive + MP_KEEPALIVE_TTL_MS +
				 MP_DEADMAN_REVERT_MS;
	uint32_t now;
	uint32_t reverted_at = 0U;

	ilk_permissive(&st);
	logs_clear();

	for (now = 1000U; now <= (deadline + 4000U);
	     now += MP_TICK_MAX_MS) {
		(void)mp_ovr_tick(&g_c, &st, now);
		if ((mp_ovr_active(&g_c) == 0U) && (reverted_at == 0U)) {
			reverted_at = now;
		}
	}

	TEST_ASSERT_NOT_EQUAL_UINT32(0U, reverted_at);
	TEST_ASSERT_TRUE_MESSAGE(reverted_at <= deadline,
				 "override outlived the §5.3 revert deadline");
	/* And it is genuinely back under firmware control. */
	TEST_ASSERT_EQUAL_UINT(1U, releases_of(obj));
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.late_ticks);
}

static void test_deadman_reverts_immediately_on_link_loss(void)
{
	mp_ilk_state_t st;
	uint32_t sid = open_session(1000U);
	size_t obj = grant_one(sid, 1000U);

	ilk_permissive(&st);
	logs_clear();

	/* No tick required: a lost link is unambiguous. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_set_link(&g_c, false, 1100U));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT(1U, releases_of(obj));
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_DEADMAN));
	TEST_ASSERT_EQUAL_STRING("link down", g_evt[0].reason);
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.sess.id);

	/* Re-asserting the same state is a no-op. */
	logs_clear();
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_set_link(&g_c, false, 1200U));
	TEST_ASSERT_EQUAL_UINT(0U, g_evt_n);
}

static void test_deadman_reverts_on_session_close(void)
{
	mp_ilk_state_t st;
	uint32_t sid = open_session(1000U);
	size_t obj = grant_one(sid, 1000U);

	ilk_permissive(&st);
	logs_clear();
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_session_close(&g_c, sid, 1100U));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT(1U, releases_of(obj));
}

static void test_deadman_with_no_session_reverts(void)
{
	mp_ilk_state_t st;
	uint32_t sid = open_session(1000U);

	ilk_permissive(&st);
	(void)grant_one(sid, 1000U);

	/* Wipe the session as a link-independent failure. */
	memset(&g_c.sess, 0, sizeof(g_c.sess));
	logs_clear();
	TEST_ASSERT_EQUAL_INT(1, mp_ovr_tick(&g_c, &st, 1100U));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_DEADMAN));
}

static void test_late_ticks_are_counted(void)
{
	mp_ilk_state_t st;

	ilk_permissive(&st);
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_tick(&g_c, &st, 1000U));
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.late_ticks);
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_tick(&g_c, &st,
					     1000U + MP_TICK_MAX_MS));
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.late_ticks);
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_tick(&g_c, &st,
					     1000U + 2U * MP_TICK_MAX_MS + 1U));
	TEST_ASSERT_EQUAL_UINT32(1U, g_c.late_ticks);
}

static void test_lease_expiry(void)
{
	mp_ilk_state_t st;
	uint32_t sid = open_session(1000U);
	size_t obj = obj_of("ui.disp.bl");

	ilk_permissive(&st);
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 50, 3000U, sid, &st,
					      1000U, NULL));
	logs_clear();

	/* Keep the dead-man happy so it is the lease TTL that fires. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 3000U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_tick(&g_c, &st, 3500U));
	TEST_ASSERT_EQUAL_size_t(1U, mp_ovr_active(&g_c));

	TEST_ASSERT_EQUAL_INT(1, mp_ovr_tick(&g_c, &st, 4000U));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT(1U, releases_of(obj));
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_EXPIRE));
	TEST_ASSERT_EQUAL_UINT32(1U, g_c.expiries);
}

/* --- the post-set VCC_RB read-back ------------------------------------- */

static void test_vcc_rb_readback_verify_passes(void)
{
	size_t obj = obj_of("pwr.rb.vset_mv");
	mp_ilk_state_t st;
	mp_ilk_res_t res;
	uint32_t sid = open_session(1000U);
	const mp_lease_t *l;

	ilk_permissive(&st);
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 12000, 0U, sid, &st,
					      1000U, &res));
	TEST_ASSERT_TRUE(res.verify);
	l = mp_ovr_lease(&g_c, obj);
	TEST_ASSERT_NOT_EQUAL_UINT32(0U, l->verify_at_ms);
	TEST_ASSERT_EQUAL_INT32(12000, l->verify_expect_mv);

	/* Rail comes up where it was asked to: the verification clears. */
	st.vcc_rb_mv = 12100;
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 1200U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_tick(&g_c, &st,
					     1000U + MP_RB_VERIFY_DELAY_MS));
	TEST_ASSERT_EQUAL_size_t(1U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT32(0U, mp_ovr_lease(&g_c, obj)->verify_at_ms);
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.verify_failures);
}

static void test_vcc_rb_readback_mismatch_auto_reverts(void)
{
	size_t obj = obj_of("pwr.rb.vset_mv");
	mp_ilk_state_t st;
	uint32_t sid = open_session(1000U);

	ilk_permissive(&st);
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 12000, 0U, sid, &st,
					      1000U, NULL));
	logs_clear();

	/* The rail is nowhere near the commanded value. */
	st.vcc_rb_mv = 4600;
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 1200U));
	TEST_ASSERT_EQUAL_INT(1, mp_ovr_tick(&g_c, &st,
					     1000U + MP_RB_VERIFY_DELAY_MS));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT(1U, releases_of(obj));
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_VERIFY_FAIL));
	TEST_ASSERT_EQUAL_UINT32(1U, g_c.verify_failures);
	TEST_ASSERT_EQUAL_STRING("VCC_RB out of window", g_evt[0].reason);
}

static void test_vcc_rb_readback_unavailable_also_reverts(void)
{
	size_t obj = obj_of("pwr.rb.vset_mv");
	mp_ilk_state_t st;
	uint32_t sid = open_session(1000U);

	ilk_permissive(&st);
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 12000, 0U, sid, &st,
					      1000U, NULL));
	logs_clear();

	/* The INA read failed: "cannot verify" gets the same answer as
	 * "verified bad". */
	st.vcc_rb_valid = false;
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 1200U));
	TEST_ASSERT_EQUAL_INT(1, mp_ovr_tick(&g_c, &st,
					     1000U + MP_RB_VERIFY_DELAY_MS));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_VERIFY_FAIL));
	TEST_ASSERT_EQUAL_STRING("VCC_RB read-back unavailable",
				 g_evt[0].reason);
}

static void test_vcc_rb_readback_with_no_state_reverts(void)
{
	size_t obj = obj_of("pwr.rb.vset_mv");
	mp_ilk_state_t st;
	uint32_t sid = open_session(1000U);

	ilk_permissive(&st);
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 12000, 0U, sid, &st,
					      1000U, NULL));
	logs_clear();
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 1200U));
	/* A NULL state at tick time is the documented fail-safe path. */
	TEST_ASSERT_EQUAL_INT(1, mp_ovr_tick(&g_c, NULL,
					     1000U + MP_RB_VERIFY_DELAY_MS));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT32(1U, g_c.verify_failures);
}

static void test_rb_gate_verify_uses_the_expected_rail_voltage(void)
{
	size_t obj = obj_of("pwr.rb.gate");
	mp_ilk_state_t st;
	uint32_t sid = open_session(1000U);

	ilk_permissive(&st);
	st.rb_expected_mv = 15000;
	TEST_ASSERT_EQUAL_INT(0,
			      mp_ovr_grant(&g_c, obj, 1, 0U, sid, &st, 1000U,
					   NULL));
	/* The gate is a bool, so the expected value comes from the state, not
	 * from the commanded value. */
	TEST_ASSERT_EQUAL_INT32(15000,
				mp_ovr_lease(&g_c, obj)->verify_expect_mv);

	st.vcc_rb_mv = 15200;
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 1200U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_tick(&g_c, &st,
					     1000U + MP_RB_VERIFY_DELAY_MS));
	TEST_ASSERT_EQUAL_size_t(1U, mp_ovr_active(&g_c));
}

static void test_revert_all_counts_and_reports(void)
{
	mp_ilk_state_t st;
	uint32_t sid = open_session(1000U);

	ilk_permissive(&st);
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj_of("ui.disp.bl"), 50,
					      0U, sid, &st, 1000U, NULL));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj_of("ui.panel.duty"), 60,
					      0U, sid, &st, 1000U, NULL));
	TEST_ASSERT_EQUAL_size_t(2U, mp_ovr_active(&g_c));

	logs_clear();
	TEST_ASSERT_EQUAL_INT(2, mp_ovr_revert_all(&g_c,
						   (uint8_t)MP_OVR_EV_RELEASE,
						   "bench", 2000U));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT(2U, events_of((uint8_t)MP_OVR_EV_RELEASE));
	TEST_ASSERT_EQUAL_STRING("bench", g_evt[0].reason);

	/* Reverting nothing is 0, not an error. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_revert_all(&g_c,
						   (uint8_t)MP_OVR_EV_RELEASE,
						   NULL, 2000U));
}

/** An apply callback that accepts values but refuses every release. */
static unsigned int g_refuse_release_calls;

static int apply_refuse_release_cb(void *user, size_t obj, const int32_t *value)
{
	(void)user;
	(void)obj;
	g_refuse_release_calls++;
	return (value == NULL) ? -EIO : 0;
}

static void test_release_error_is_counted_but_the_lease_still_goes(void)
{
	/*
	 * A glue that cannot hand an object back is a real fault, but the lease
	 * must still be dropped: keeping it would leave the tool believing it
	 * owns something core has already forgotten, and the next dead-man would
	 * try to release it again forever.
	 */
	mp_ovr_ctx_t c;
	size_t obj = obj_of("ui.disp.bl");
	mp_ilk_state_t st;
	uint32_t sid = 0U;

	ilk_permissive(&st);
	g_refuse_release_calls = 0U;
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_init(&c, apply_refuse_release_cb, NULL,
					     NULL, NULL, SERIAL));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_set_link(&c, true, 1000U));
	TEST_ASSERT_EQUAL_INT(0,
			      mp_ovr_session_open(&c, 0U, (uint8_t)MP_ROLE_ADMIN,
						  USER, 1000U, &sid));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&c, obj, 50, 0U, sid, &st, 1000U,
					      NULL));

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_release(&c, obj, sid, 2000U));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&c));
	TEST_ASSERT_NULL(mp_ovr_lease(&c, obj));
	TEST_ASSERT_EQUAL_UINT32(1U, c.release_errors);
	TEST_ASSERT_EQUAL_UINT(2U, g_refuse_release_calls);

	/* A NULL event sink must also be tolerated throughout. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&c, obj, 60, 0U, sid, &st, 2000U,
					      NULL));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_veto(&c, obj, "no sink", 2100U));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&c));
}

/* ------------------------------------------------------------------- runner */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_names);
	RUN_TEST(test_init_validation);
	RUN_TEST(test_roles_match_the_auth_enum);

	RUN_TEST(test_session_lifecycle);
	RUN_TEST(test_session_ttl_is_clamped);
	RUN_TEST(test_session_requires_a_link);
	RUN_TEST(test_reopening_reverts_the_previous_sessions_overrides);
	RUN_TEST(test_session_records_the_role_and_the_user);
	RUN_TEST(test_an_unrecognised_role_grants_nothing);
	RUN_TEST(test_session_takeover_does_not_inherit_the_role);

	RUN_TEST(test_g0_needs_nothing);
	RUN_TEST(test_a_session_with_no_credential_is_capped_at_g0);
	RUN_TEST(test_a_viewer_cannot_perform_a_g1_action);
	RUN_TEST(test_an_operator_gets_g1_but_not_g2_even_with_the_serial);
	RUN_TEST(test_an_admin_with_the_serial_gets_g2);
	RUN_TEST(test_a_refused_role_does_not_refresh_the_keepalive);
	RUN_TEST(test_g1_needs_a_fresh_session);
	RUN_TEST(test_a_successful_guard_refreshes_the_keepalive);
	RUN_TEST(test_g2_needs_the_typed_device_serial);
	RUN_TEST(test_g2_on_an_unprovisioned_board_always_refuses);
	RUN_TEST(test_g3_arm_hold_and_complete);
	RUN_TEST(test_g3_nonce_is_bound_to_the_action);
	RUN_TEST(test_g3_arm_expires);
	RUN_TEST(test_g3_arm_is_dropped_by_the_tick_after_its_window);
	RUN_TEST(test_g3_phrase_is_replaceable);
	RUN_TEST(test_guard_escalation_matrix);

	RUN_TEST(test_ilk_no_interlocks_passes_through);
	RUN_TEST(test_ilk_null_state_refuses_every_guarded_object);
	RUN_TEST(test_ilk_argument_validation);
	RUN_TEST(test_ilk_vcc_rb_is_clamped_to_the_configured_ceiling);
	RUN_TEST(test_ilk_vcc_rb_refuses_while_the_ov_latch_is_set);
	RUN_TEST(test_ilk_digipot_code_is_clamped_to_the_permitted_window);
	RUN_TEST(test_ilk_rb_power_needs_ocxo_warm_and_supercaps);
	RUN_TEST(test_ilk_fan_floor_clamps_upward);
	RUN_TEST(test_ilk_relay_cannot_be_forced_over_a_live_fault);
	RUN_TEST(test_ilk_display_minimum_off_time);
	RUN_TEST(test_ilk_wdt_arm_needs_a_healthy_liveness_gate);
	RUN_TEST(test_ilk_wdi_pulse_is_refused_while_the_watchdog_is_armed);
	RUN_TEST(test_ilk_clock_mux_guard);
	RUN_TEST(test_ilk_dac_needs_the_loop_parked);
	RUN_TEST(test_ilk_the_two_vc_views_exclude_each_other);
	RUN_TEST(test_ilk_park_release_is_refused_under_a_vc_override);
	RUN_TEST(test_the_park_object_does_not_demand_its_own_effect);
	RUN_TEST(test_ilk_tunnel_reports_a_consequence_not_a_refusal);

	RUN_TEST(test_grant_and_release);
	RUN_TEST(test_release_belongs_to_the_owner);
	RUN_TEST(test_grant_rejections);
	RUN_TEST(test_lease_ttl_is_clamped);
	RUN_TEST(test_lease_table_full);
	RUN_TEST(test_apply_refusal_is_a_veto_and_leaves_no_lease);
	RUN_TEST(test_an_unwired_apply_is_notsup_and_not_a_veto);
	RUN_TEST(test_a_busy_resource_is_busy_and_not_a_veto);
	RUN_TEST(test_a_release_refused_by_a_tunnel_is_paid_at_tunnel_close);
	RUN_TEST(test_firmware_veto);

	RUN_TEST(test_deadman_reverts_on_a_stale_keepalive);
	RUN_TEST(test_deadman_revert_timing_bound);
	RUN_TEST(test_deadman_reverts_immediately_on_link_loss);
	RUN_TEST(test_deadman_reverts_on_session_close);
	RUN_TEST(test_deadman_with_no_session_reverts);
	RUN_TEST(test_late_ticks_are_counted);
	RUN_TEST(test_lease_expiry);

	RUN_TEST(test_vcc_rb_readback_verify_passes);
	RUN_TEST(test_vcc_rb_readback_mismatch_auto_reverts);
	RUN_TEST(test_vcc_rb_readback_unavailable_also_reverts);
	RUN_TEST(test_vcc_rb_readback_with_no_state_reverts);
	RUN_TEST(test_rb_gate_verify_uses_the_expected_rail_voltage);
	RUN_TEST(test_revert_all_counts_and_reports);
	RUN_TEST(test_release_error_is_counted_but_the_lease_still_goes);

	return UNITY_END();
}
