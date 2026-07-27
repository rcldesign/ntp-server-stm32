/*
 * STS1000 "Meridian" — core/mp override, guard and interlock unit tests.
 *
 * This is the module whose failure modes matter more than its features, so the
 * tests are organised around the properties the FMT spec puts numbers on:
 *
 *   §5.2  the guard escalation, as an exhaustive matrix over
 *         (class, session, keepalive freshness, confirmation, nonce, hold);
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

#include "mp/mp_override.h"

#define SERIAL "STS1000-000042"

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

/** Open a session and satisfy G2 so grants can be tested in isolation. */
static uint32_t open_session(uint32_t now)
{
	uint32_t sid = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_session_open(&g_c, 0U, now, &sid));
	TEST_ASSERT_NOT_EQUAL_UINT32(0U, sid);
	return sid;
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
	}
	TEST_ASSERT_EQUAL_STRING("unknown", mp_gc_name(MP_GC_COUNT));
	TEST_ASSERT_EQUAL_STRING("ok", mp_gc_name(MP_GC_OK));
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
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_ovr_session_open(NULL, 0U, 0U, NULL));
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
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_session_open(&g_c, 0U, 20000U, &sid2));
	TEST_ASSERT_NOT_EQUAL_UINT32(sid1, sid2);

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_session_close(&g_c, sid2, 21000U));
	TEST_ASSERT_EQUAL_INT(-ENOENT, mp_ovr_session_close(&g_c, sid2, 21000U));
	TEST_ASSERT_EQUAL_UINT32(0U, mp_ovr_session_remaining(&g_c, 21000U));
}

static void test_session_ttl_is_clamped(void)
{
	uint32_t sid = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_session_open(&g_c, 999999U, 1000U,
						    &sid));
	TEST_ASSERT_EQUAL_UINT32(MP_KEEPALIVE_TTL_MS, g_c.sess.ttl_ms);

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_session_open(&g_c, 1500U, 2000U, &sid));
	TEST_ASSERT_EQUAL_UINT32(1500U, g_c.sess.ttl_ms);
	TEST_ASSERT_FALSE(mp_ovr_session_valid(&g_c, sid, 3600U));
}

static void test_session_requires_a_link(void)
{
	uint32_t sid = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_set_link(&g_c, false, 1000U));
	TEST_ASSERT_EQUAL_INT(-ENOLINK,
			      mp_ovr_session_open(&g_c, 0U, 1000U, &sid));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_ovr_session_open(&g_c, 0U, 1000U, NULL));
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
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_session_open(&g_c, 0U, 2000U, &sid2));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT(1U, releases_of(obj));
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_RELEASE));
}

/* ============================================== §5.2 guard escalation ==== */

static void test_g0_needs_nothing(void)
{
	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G0, 0U, 0U, 0U, NULL,
					   0U, 1000U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_ovr_guard(NULL, MP_GUARD_G0, 0U, 0U, 0U, NULL,
					   0U, 1000U, NULL));
}

static void test_g1_needs_a_fresh_session(void)
{
	uint32_t sid;

	/* No session at all. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SESSION,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, 0U, 0U, 0U, NULL,
					   0U, 1000U, NULL));
	sid = open_session(1000U);

	/* Wrong id. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SESSION,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, sid + 1U, 0U, 0U,
					   NULL, 0U, 1000U, NULL));
	/* Right id, fresh. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, sid, 0U, 0U, NULL,
					   0U, 1000U, NULL));

	/* Stale keepalive. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_STALE,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, sid, 0U, 0U, NULL,
					   0U,
					   1000U + MP_KEEPALIVE_TTL_MS + 1U,
					   NULL));

	/* Link down. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_set_link(&g_c, false, 1000U));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SESSION,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, sid, 0U, 0U, NULL,
					   0U, 1000U, NULL));
}

static void test_a_successful_guard_refreshes_the_keepalive(void)
{
	uint32_t sid = open_session(1000U);

	/* Issuing a command demonstrates the tool is alive (FMT §5.3). */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G1, sid, 0U, 0U, NULL,
					   0U, 4000U, NULL));
	TEST_ASSERT_TRUE(mp_ovr_session_valid(&g_c, sid, 8500U));
}

static void test_g2_needs_the_typed_device_serial(void)
{
	uint32_t sid = open_session(1000U);

	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, NULL,
					   0U, 1000U, NULL));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, "",
					   0U, 1000U, NULL));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U,
					   "wrong", 0U, 1000U, NULL));
	/* A prefix is not the serial. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U,
					   "STS1000", 0U, 1000U, NULL));
	/* Nor is the serial plus a suffix. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U,
					   SERIAL "x", 0U, 1000U, NULL));
	/* Case matters. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U,
					   "sts1000-000042", 0U, 1000U, NULL));

	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U,
					   SERIAL, 0U, 1000U, NULL));
	TEST_ASSERT_TRUE(g_c.sess.serial_ok);

	/* G2 still requires a session first: no session outranks a good serial. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_session_close(&g_c, sid, 2000U));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SESSION,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U,
					   SERIAL, 0U, 2000U, NULL));
}

static void test_g2_on_an_unprovisioned_board_always_refuses(void)
{
	uint32_t sid = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_ovr_init(&g_c, apply_cb, NULL, evt_cb, NULL,
					     NULL));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_set_link(&g_c, true, 1000U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_session_open(&g_c, 0U, 1000U, &sid));

	/* An empty expected string must never match, including against "". */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U, "",
					   0U, 1000U, NULL));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G2, sid, 0U, 0U,
					   SERIAL, 0U, 1000U, NULL));
}

static void test_g3_arm_hold_and_complete(void)
{
	uint32_t sid = open_session(1000U);
	mp_gc_arm_t arm;
	uint32_t nonce;

	memset(&arm, 0, sizeof(arm));

	/* The phrase is required to arm; the serial alone is not enough. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_SERIAL,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U, NULL,
					   0U, 1000U, &arm));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_PHRASE,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U,
					   SERIAL, 0U, 1000U, &arm));

	/* Arming needs the phrase. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARMED,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U,
					   MP_G3_PHRASE_DEFAULT, 0U, 1000U,
					   &arm));
	TEST_ASSERT_NOT_EQUAL_UINT32(0U, arm.nonce);
	TEST_ASSERT_EQUAL_UINT32(MP_G3_HOLD_MS, arm.hold_ms);
	TEST_ASSERT_EQUAL_UINT32(MP_G3_WINDOW_MS, arm.expires_ms);
	nonce = arm.nonce;

	/* Too early. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_NEED_HOLD,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U,
					   SERIAL, nonce,
					   1000U + MP_G3_HOLD_MS - 1U, NULL));
	/* Exactly at the hold: allowed. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U,
					   SERIAL, nonce, 1000U + MP_G3_HOLD_MS,
					   NULL));
	/* The arm is consumed: a replay fails. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARM_MISMATCH,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U,
					   SERIAL, nonce, 1000U + MP_G3_HOLD_MS,
					   NULL));
}

static void test_g3_nonce_is_bound_to_the_action(void)
{
	uint32_t sid = open_session(1000U);
	mp_gc_arm_t arm;

	memset(&arm, 0, sizeof(arm));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARMED,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 7U, 0U,
					   MP_G3_PHRASE_DEFAULT, 0U, 1000U,
					   &arm));

	/* Right nonce, wrong object: refused. This is what stops an arm for one
	 * G3 action being completed as a different one. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARM_MISMATCH,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 8U, 0U,
					   SERIAL, arm.nonce,
					   1000U + MP_G3_HOLD_MS, NULL));
	/* Wrong nonce, right object: refused. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARM_MISMATCH,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 7U, 0U,
					   SERIAL, arm.nonce ^ 1U,
					   1000U + MP_G3_HOLD_MS, NULL));

	/* And for a non-object action the tag binds it instead. */
	memset(&arm, 0, sizeof(arm));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARMED,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 0U, 200U,
					   MP_G3_PHRASE_DEFAULT, 0U, 2000U,
					   &arm));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARM_MISMATCH,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 0U, 201U,
					   SERIAL, arm.nonce,
					   2000U + MP_G3_HOLD_MS, NULL));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_OK,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 0U, 200U,
					   SERIAL, arm.nonce,
					   2000U + MP_G3_HOLD_MS, NULL));
}

static void test_g3_arm_expires(void)
{
	uint32_t sid = open_session(1000U);
	mp_gc_arm_t arm;

	memset(&arm, 0, sizeof(arm));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARMED,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U,
					   MP_G3_PHRASE_DEFAULT, 0U, 1000U,
					   &arm));

	/* Keep the session alive across the arm window. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 4000U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 8000U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 12000U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 16000U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 20000U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 24000U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 28000U));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_keepalive(&g_c, sid, 31500U));

	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARM_MISMATCH,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U,
					   SERIAL, arm.nonce,
					   1000U + MP_G3_WINDOW_MS + 1U, NULL));
	/* And the arm has been cleared, so re-arming is required. */
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARM_MISMATCH,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U,
					   SERIAL, arm.nonce, 31600U, NULL));
}

static void test_g3_arm_is_dropped_by_the_tick_after_its_window(void)
{
	uint32_t sid = open_session(1000U);
	mp_ilk_state_t st;
	mp_gc_arm_t arm;

	ilk_permissive(&st);
	memset(&arm, 0, sizeof(arm));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARMED,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U,
					   MP_G3_PHRASE_DEFAULT, 0U, 1000U,
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

		for (t = 6000U; t <= 32000U; t += 4000U) {
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
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U,
					   MP_G3_PHRASE_DEFAULT, 0U, 1000U,
					   &arm));
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARMED,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U,
					   "DO IT", 0U, 1000U, &arm));

	/* NULL and "" restore the default. */
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_set_phrase(&g_c, NULL));
	mp_ovr_disarm(&g_c);
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARMED,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U,
					   MP_G3_PHRASE_DEFAULT, 0U, 1000U,
					   &arm));
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_set_phrase(&g_c, ""));
	mp_ovr_disarm(&g_c);
	TEST_ASSERT_EQUAL_INT((int)MP_GC_ARMED,
			      mp_ovr_guard(&g_c, MP_GUARD_G3, sid, 5U, 0U,
					   MP_G3_PHRASE_DEFAULT, 0U, 1000U,
					   &arm));
}

/** The whole escalation as a matrix, so a reordering cannot pass unnoticed. */
static void test_guard_escalation_matrix(void)
{
	static const struct {
		uint8_t guard;
		bool session;
		bool fresh;
		const char *confirm;
		int expect;
	} m[] = {
		{ MP_GUARD_G0, false, false, NULL, MP_GC_OK },
		{ MP_GUARD_G0, false, false, "junk", MP_GC_OK },
		{ MP_GUARD_G1, false, false, NULL, MP_GC_NEED_SESSION },
		{ MP_GUARD_G1, true, true, NULL, MP_GC_OK },
		{ MP_GUARD_G1, true, false, NULL, MP_GC_STALE },
		{ MP_GUARD_G2, false, false, SERIAL, MP_GC_NEED_SESSION },
		{ MP_GUARD_G2, true, false, SERIAL, MP_GC_STALE },
		{ MP_GUARD_G2, true, true, NULL, MP_GC_NEED_SERIAL },
		{ MP_GUARD_G2, true, true, "nope", MP_GC_NEED_SERIAL },
		{ MP_GUARD_G2, true, true, SERIAL, MP_GC_OK },
		{ MP_GUARD_G3, false, false, MP_G3_PHRASE_DEFAULT,
		  MP_GC_NEED_SESSION },
		{ MP_GUARD_G3, true, false, MP_G3_PHRASE_DEFAULT, MP_GC_STALE },
		{ MP_GUARD_G3, true, true, NULL, MP_GC_NEED_SERIAL },
		{ MP_GUARD_G3, true, true, SERIAL, MP_GC_NEED_PHRASE },
		{ MP_GUARD_G3, true, true, MP_G3_PHRASE_DEFAULT, MP_GC_ARMED },
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
			sid = open_session(now);
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
							   m[i].confirm, 0U,
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

	/* A zero ceiling means the glue could not read cfg: refuse, do not
	 * fall back to the electrical maximum. */
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
	size_t kick = obj_of("sys.wdt.kick");
	mp_ilk_state_t st;
	mp_ilk_res_t res;

	ilk_permissive(&st);
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(en, 1, &st, 0U, &res));
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(kick, 1, &st, 0U, &res));

	st.liveness_ok = false;
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(en, 1, &st, 0U, &res));
	TEST_ASSERT_EQUAL_UINT32(MP_ILK_WDT_LIVE, res.failed);
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_ilk_eval(kick, 1, &st, 0U, &res));

	/* Disarming is allowed regardless. */
	TEST_ASSERT_EQUAL_INT(0, mp_ilk_eval(en, 0, &st, 0U, &res));
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

	/* Fill the table with distinct overridable objects. */
	for (i = 0U; (i < mp_obj_count()) && (granted < MP_LEASE_MAX); i++) {
		const mp_obj_t *o = mp_obj_at(i);

		if ((o->flags & MP_OF_OVERRIDE) == 0U) {
			continue;
		}
		if (o->ilk != 0U) {
			continue; /* keep the interlocks out of this test */
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

		if (((o->flags & MP_OF_OVERRIDE) != 0U) && (o->ilk == 0U)) {
			TEST_ASSERT_EQUAL_INT(-ENOSPC,
					      mp_ovr_grant(&g_c, i, o->min, 0U,
							   sid, &st, 1000U,
							   NULL));
			break;
		}
	}
	TEST_ASSERT_TRUE_MESSAGE(i < mp_obj_count(),
				 "manifest has no spare overridable object");
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

	/* And a refusal on a *replacement* must not destroy the existing lease
	 * silently either — it is reported as a veto and the slot is freed. */
	g_apply_rc = 0;
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_grant(&g_c, obj, 50, 0U, sid, &st,
					      1000U, NULL));
	logs_clear();
	g_apply_rc = -EIO;
	TEST_ASSERT_EQUAL_INT(-EACCES, mp_ovr_grant(&g_c, obj, 60, 0U, sid, &st,
						    1000U, NULL));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c));
	TEST_ASSERT_EQUAL_UINT(1U, events_of((uint8_t)MP_OVR_EV_VETO));
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
	TEST_ASSERT_EQUAL_INT(0, mp_ovr_session_open(&c, 0U, 1000U, &sid));
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

	RUN_TEST(test_session_lifecycle);
	RUN_TEST(test_session_ttl_is_clamped);
	RUN_TEST(test_session_requires_a_link);
	RUN_TEST(test_reopening_reverts_the_previous_sessions_overrides);

	RUN_TEST(test_g0_needs_nothing);
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
	RUN_TEST(test_ilk_clock_mux_guard);
	RUN_TEST(test_ilk_dac_needs_the_loop_parked);
	RUN_TEST(test_ilk_tunnel_reports_a_consequence_not_a_refusal);

	RUN_TEST(test_grant_and_release);
	RUN_TEST(test_release_belongs_to_the_owner);
	RUN_TEST(test_grant_rejections);
	RUN_TEST(test_lease_ttl_is_clamped);
	RUN_TEST(test_lease_table_full);
	RUN_TEST(test_apply_refusal_is_a_veto_and_leaves_no_lease);
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
