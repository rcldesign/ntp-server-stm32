/*
 * STS1000 "Meridian" — core/mp JSON-RPC control-plane unit tests.
 *
 * Every request is driven the way a host drives it — as a JSON document — and
 * every reply is parsed back as JSON, so the tests assert on the wire contract
 * rather than on internal state. The guarded methods are exercised through the
 * whole escalation (no session, session, serial, phrase + hold), because the
 * interesting failure is a guard that is *declared* in the manifest but not
 * actually applied by the handler.
 *
 * The last group runs the full stack: a request is COBS-framed, fed in a byte at
 * a time, and the reply is decoded from the captured wire bytes.
 *
 * Authentication (FMT §5.3) is wired to a table-driven stand-in for
 * sts_aaa_check(), so the role floor, the indistinguishable refusal and the
 * lockout are covered without a credential store. The default session is an
 * **admin** session, because most of these tests are about the handlers rather
 * than about who is allowed to reach them; the role tests open their own.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "mp/mp.h"
#include "test_support.h"

#define SERIAL "STS1000-000042"

/* Credentials the stand-in AAA accepts. */
#define ADMIN_USER "root"
#define ADMIN_PW "correct horse"
#define OPER_USER "tech"
#define OPER_PW "operator pw"
#define VIEW_USER "guest"
#define VIEW_PW "viewer pw"
#define LOCKED_USER "locked"

/* ------------------------------------------------------------------ fixture */

static mp_ctx_t g_c;
static uint8_t g_scratch[8192];
static mp_reasm_t g_slots[2];
static uint8_t g_slot_buf[2][4096];
static char g_prev_ch[20 * 60];
static uint8_t g_prev_attr[20 * 60];

static cfg_ctx_t g_cfg;
static logr_t g_log;
static logr_rec_t g_log_slots[16];

static uint32_t g_now;

/* captured transmit */
static uint8_t g_wire[65536];
static size_t g_wire_len;
static unsigned int g_frames;

/* applied values */
typedef struct {
	size_t obj;
	bool release;
	int32_t value;
} apply_rec_t;

static apply_rec_t g_apply[32];
static unsigned int g_apply_n;
static int g_apply_rc;

static size_t g_pulse_obj;
static uint32_t g_pulse_ms;
static unsigned int g_pulse_n;
static int g_pulse_rc;

static unsigned int g_commit_n;
static int g_commit_rc;

static int g_reboot_mode;
static unsigned int g_reboot_n;

/* provider state */
static mp_ilk_state_t g_ilk;
static bool g_ilk_present = true;
static int g_ilk_rc;
static mp_telem_t g_telem;
static bool g_telem_present = true;
static mp_pps_t g_pps;
static bool g_pps_present = true;
static mp_mirror_in_t g_mirror;
static bool g_mirror_present = true;
static char g_grid_ch[20 * 60];
static uint8_t g_grid_attr[20 * 60];
static bool g_bundle_present = true;

static uint32_t clock_cb(void *user)
{
	(void)user;
	return g_now;
}

static int tx_cb(void *user, const uint8_t *wire, size_t len)
{
	(void)user;
	TEST_ASSERT_TRUE((g_wire_len + len) <= sizeof(g_wire));
	memcpy(&g_wire[g_wire_len], wire, len);
	g_wire_len += len;
	g_frames++;
	return 0;
}

static int apply_cb(void *user, size_t obj, const int32_t *value)
{
	(void)user;
	if (g_apply_n < (sizeof(g_apply) / sizeof(g_apply[0]))) {
		g_apply[g_apply_n].obj = obj;
		g_apply[g_apply_n].release = (value == NULL);
		g_apply[g_apply_n].value = (value != NULL) ? *value : 0;
		g_apply_n++;
	}
	return (value == NULL) ? 0 : g_apply_rc;
}

static int obj_read_cb(void *user, size_t obj, mp_val_t *out)
{
	const mp_obj_t *o = mp_obj_at(obj);

	(void)user;
	TEST_ASSERT_NOT_NULL(o);
	out->kind = o->kind;
	out->valid = true;
	switch (o->kind) {
	case MP_KIND_BOOL:
		out->i = 1;
		break;
	case MP_KIND_REAL:
		out->f = 1.5f;
		break;
	case MP_KIND_BITS:
		out->u = 0x1234ULL;
		break;
	case MP_KIND_TEXT:
		(void)snprintf(out->text, sizeof(out->text), "text-%u",
			       (unsigned int)obj);
		break;
	case MP_KIND_RAIL:
		out->rail[0] = 3300;
		out->rail[1] = -1500;
		out->rail[2] = 4950;
		out->rail[3] = 0x0002;
		break;
	default:
		out->i = o->min;
		break;
	}
	return 0;
}

static int pulse_cb(void *user, size_t obj, uint32_t ms)
{
	(void)user;
	g_pulse_obj = obj;
	g_pulse_ms = ms;
	g_pulse_n++;
	return g_pulse_rc;
}

static int diag_action_cb(void *user, uint8_t test, uint8_t step,
			  mp_diag_step_res_t *out)
{
	(void)user;
	(void)test;
	(void)step;
	out->verdict = (uint8_t)MP_DIAG_PASS;
	out->value = 1;
	return 0;
}

static int ilk_cb(void *user, mp_ilk_state_t *out)
{
	(void)user;
	if (g_ilk_rc != 0) {
		return g_ilk_rc;
	}
	*out = g_ilk;
	return 0;
}

static int telem_cb(void *user, mp_telem_t *out)
{
	(void)user;
	*out = g_telem;
	return 0;
}

static int pps_cb(void *user, mp_pps_t *out)
{
	(void)user;
	*out = g_pps;
	return 0;
}

static int mirror_cb(void *user, mp_mirror_in_t *out)
{
	(void)user;
	*out = g_mirror;
	return 0;
}

static int bundle_cb(void *user, mp_bundle_t *out)
{
	(void)user;
	memset(out, 0, sizeof(*out));
	out->notes = "unit test";
	return 0;
}

static int time_cb(void *user, uint64_t *tai_ns, bool *fallback)
{
	(void)user;
	*tai_ns = 1700000000000000000ULL;
	*fallback = false;
	return 0;
}

/* --- the credential store stand-in ------------------------------------- */

static bool g_auth_present = true;
static unsigned int g_auth_calls;
static char g_auth_last_user[96];
static char g_auth_last_secret[96];

/**
 * Stands in for sts_aaa_check() with the same three-valued contract: 0 accepts
 * and writes a role, -EBUSY is a lockout, everything else is a refusal that the
 * control plane must not describe.
 */
static int auth_cb(void *user, const char *user_name, const char *secret,
		   uint8_t *out_role)
{
	(void)user;
	g_auth_calls++;
	(void)snprintf(g_auth_last_user, sizeof(g_auth_last_user), "%s",
		       (user_name != NULL) ? user_name : "");
	(void)snprintf(g_auth_last_secret, sizeof(g_auth_last_secret), "%s",
		       (secret != NULL) ? secret : "");

	*out_role = (uint8_t)MP_ROLE_NONE;
	if ((user_name == NULL) || (secret == NULL)) {
		return -EACCES;
	}
	if (strcmp(user_name, LOCKED_USER) == 0) {
		return -EBUSY;
	}
	if ((strcmp(user_name, ADMIN_USER) == 0) &&
	    (strcmp(secret, ADMIN_PW) == 0)) {
		*out_role = (uint8_t)MP_ROLE_ADMIN;
		return 0;
	}
	if ((strcmp(user_name, OPER_USER) == 0) &&
	    (strcmp(secret, OPER_PW) == 0)) {
		*out_role = (uint8_t)MP_ROLE_OPERATOR;
		return 0;
	}
	if ((strcmp(user_name, VIEW_USER) == 0) &&
	    (strcmp(secret, VIEW_PW) == 0)) {
		*out_role = (uint8_t)MP_ROLE_VIEWER;
		return 0;
	}
	/* Unknown user and wrong password are the same answer here, exactly as
	 * they are in the reply. */
	return -EACCES;
}

static int cfg_commit_cb(void *user, cfg_commit_res_t *res)
{
	(void)user;
	g_commit_n++;
	if (g_commit_rc != 0) {
		return g_commit_rc;
	}
	return cfg_commit(&g_cfg, res);
}

static void reboot_cb(void *ctx, int mode)
{
	(void)ctx;
	g_reboot_mode = mode;
	g_reboot_n++;
}

static const port_image_t g_img = { .reboot = reboot_cb };

static void ilk_permissive(void)
{
	memset(&g_ilk, 0, sizeof(g_ilk));
	g_ilk.ocxo_warm = true;
	g_ilk.supercaps_ok = true;
	g_ilk.vcc_rb_mv = 15000;
	g_ilk.vcc_rb_valid = true;
	g_ilk.rb_expected_mv = 15000;
	g_ilk.rb_vmax_mv = 15000U;
	g_ilk.rb_code_max = 200U;
	g_ilk.liveness_ok = true;
	g_ilk.disc_parked = true;
	g_ilk.extref_ok = true;
	g_ilk.rb_lock = true;
	g_ilk.disp_on = true;
}

static void wire_up(mp_wiring_t *w)
{
	memset(w, 0, sizeof(*w));
	w->tx = tx_cb;
	w->mono_ms = clock_cb;
	w->model = "STS1000";
	w->serial = SERIAL;
	w->fw_version = "1.2.3";
	w->boot_version = "0.9.0";
	w->board_id = "0011223344556677";
	w->telem = g_telem_present ? telem_cb : NULL;
	w->pps = g_pps_present ? pps_cb : NULL;
	w->ilk = g_ilk_present ? ilk_cb : NULL;
	w->mirror = g_mirror_present ? mirror_cb : NULL;
	w->bundle = g_bundle_present ? bundle_cb : NULL;
	w->time_get = time_cb;
	w->apply = apply_cb;
	w->obj_read = obj_read_cb;
	w->pulse = pulse_cb;
	w->diag = diag_action_cb;
	w->cfg_commit = cfg_commit_cb;
	w->auth = g_auth_present ? auth_cb : NULL;
	w->img = &g_img;
	w->cfg = &g_cfg;
	w->log = &g_log;
	w->scratch = g_scratch;
	w->scratch_len = sizeof(g_scratch);
	w->reasm = g_slots;
	w->reasm_n = 2U;
	w->mirror_prev_ch = g_prev_ch;
	w->mirror_prev_attr = g_prev_attr;
	w->mirror_cells = sizeof(g_prev_ch);
}

void setUp(void)
{
	mp_wiring_t w;
	unsigned int i;

	g_now = 10000U;
	g_wire_len = 0U;
	g_frames = 0U;
	g_apply_n = 0U;
	g_apply_rc = 0;
	g_pulse_n = 0U;
	g_pulse_rc = 0;
	g_commit_n = 0U;
	g_commit_rc = 0;
	g_reboot_n = 0U;
	g_reboot_mode = -1;
	g_ilk_present = true;
	g_ilk_rc = 0;
	g_auth_present = true;
	g_auth_calls = 0U;
	g_auth_last_user[0] = '\0';
	g_auth_last_secret[0] = '\0';
	g_telem_present = true;
	g_pps_present = true;
	g_mirror_present = true;
	g_bundle_present = true;
	ilk_permissive();

	for (i = 0U; i < 2U; i++) {
		g_slots[i].buf = g_slot_buf[i];
		g_slots[i].cap = sizeof(g_slot_buf[i]);
	}

	memset(&g_telem, 0, sizeof(g_telem));
	g_telem.q.stratum = 1U;
	g_telem.q.lock_state = (uint8_t)QUALITY_LOCK_LOCKED;
	g_telem.q.active_ref = (uint8_t)QUALITY_REF_OCXO;
	g_telem.q.utc_valid = true;
	g_telem.q.leap_current_s = 37;
	memset(&g_pps, 0, sizeof(g_pps));

	memset(g_grid_ch, ' ', sizeof(g_grid_ch));
	memset(g_grid_attr, 0, sizeof(g_grid_attr));
	memset(&g_mirror, 0, sizeof(g_mirror));
	g_mirror.rows = 20U;
	g_mirror.cols = 60U;
	g_mirror.cell_w = 8U;
	g_mirror.cell_h = 16U;
	g_mirror.ch = g_grid_ch;
	g_mirror.attr = g_grid_attr;

	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, NULL));
	TEST_ASSERT_EQUAL_INT(0, logr_init(&g_log, g_log_slots, 16U));

	wire_up(&w);
	TEST_ASSERT_EQUAL_INT(0, mp_init(&g_c, &w));
	TEST_ASSERT_EQUAL_INT(0, mp_set_link(&g_c, true));
	TEST_ASSERT_EQUAL_INT(0, mp_mode_enter(&g_c));
}

/* --------------------------------------------------------------- RPC helper */

#define TOKS 1024U
static mp_json_t g_rp;
static mp_json_tok_t g_rtok[TOKS];

/** Issue @p req; parse the reply and leave it in g_rp. Returns the reply len. */
static size_t call(const char *req)
{
	const char *out = NULL;
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_rpc_handle(&g_c, (const uint8_t *)req,
					       strlen(req), &out, &len));
	TEST_ASSERT_NOT_NULL_MESSAGE(out, req);
	TEST_ASSERT_TRUE(len > 0U);
	TEST_ASSERT_TRUE_MESSAGE(mp_json_parse(&g_rp, out, len, g_rtok, TOKS,
					       0U) > 0,
				 out);
	/* Every reply is a JSON-RPC 2.0 envelope. */
	TEST_ASSERT_TRUE(mp_json_streq(&g_rp,
				       mp_json_obj_get(&g_rp, 0, "jsonrpc"),
				       "2.0"));
	return len;
}

/** The `result` object of the last reply; fails when it was an error. */
static int result(void)
{
	int r = mp_json_obj_get(&g_rp, 0, "result");

	if (r < 0) {
		int e = mp_json_obj_get(&g_rp, 0, "error");
		char msg[96] = "no result and no error";

		if (e >= 0) {
			int d = mp_json_obj_get(&g_rp, e, "code");
			int64_t code = 0;

			(void)mp_json_i64(&g_rp, d, &code);
			(void)snprintf(msg, sizeof(msg),
				       "unexpected error %lld", (long long)code);
		}
		TEST_FAIL_MESSAGE(msg);
	}
	return r;
}

/** The error code of the last reply. */
static int64_t err_code(void)
{
	int e = mp_json_obj_get(&g_rp, 0, "error");
	int64_t code = 0;

	TEST_ASSERT_TRUE_MESSAGE(e >= 0, "expected an error reply");
	TEST_ASSERT_EQUAL_INT(0,
			      mp_json_i64(&g_rp, mp_json_obj_get(&g_rp, e,
								 "code"),
					  &code));
	return code;
}

static int64_t res_i(const char *key)
{
	int64_t v = 0;

	TEST_ASSERT_EQUAL_INT(0,
			      mp_json_i64(&g_rp,
					  mp_json_obj_get(&g_rp, result(), key),
					  &v));
	return v;
}

static bool res_b(const char *key)
{
	bool v = false;

	TEST_ASSERT_EQUAL_INT(0,
			      mp_json_bool(&g_rp,
					   mp_json_obj_get(&g_rp, result(), key),
					   &v));
	return v;
}

static bool res_streq(const char *key, const char *want)
{
	return mp_json_streq(&g_rp, mp_json_obj_get(&g_rp, result(), key),
			     want);
}

/** Open a session as @p user / @p secret and return its id. */
static uint32_t session_as(const char *user, const char *secret)
{
	char req[256];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"session.open\","
		       "\"params\":{\"client\":\"fmt\",\"user\":\"%s\","
		       "\"secret\":\"%s\"}}",
		       user, secret);
	(void)call(req);
	return (uint32_t)res_i("sid");
}

/**
 * Open the default session: **admin**.
 *
 * Every guarded method below is reached at the privilege its guard class
 * demands, so these tests exercise the handler rather than the role floor. The
 * floor itself is covered by the §5.3 group.
 */
static uint32_t session(void)
{
	return session_as(ADMIN_USER, ADMIN_PW);
}

/** Open an anonymous session — no credential at all. */
static uint32_t session_anon(void)
{
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"session.open\","
		   "\"params\":{\"client\":\"fmt\"}}");
	return (uint32_t)res_i("sid");
}

/* ------------------------------------------------------------------- basics */

static void test_init_validation(void)
{
	mp_wiring_t w;
	mp_ctx_t c;

	wire_up(&w);
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_init(NULL, &w));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_init(&c, NULL));

	wire_up(&w);
	w.tx = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_init(&c, &w));
	wire_up(&w);
	w.mono_ms = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_init(&c, &w));
	wire_up(&w);
	w.apply = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_init(&c, &w));
	wire_up(&w);
	w.scratch = NULL;
	TEST_ASSERT_EQUAL_INT(-ENOMEM, mp_init(&c, &w));
	wire_up(&w);
	w.scratch_len = MP_SCRATCH_MIN - 1U;
	TEST_ASSERT_EQUAL_INT(-ENOMEM, mp_init(&c, &w));

	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_MODE_SHELL, mp_mode(NULL));
	TEST_ASSERT_EQUAL_UINT32(0U, mp_manifest_hash_cached(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_mode_enter(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_mode_exit(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_set_link(NULL, true));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_tick(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_shell_byte(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_input(NULL, NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_post_event(NULL, 0U, 0U, 0U, 0U, 0,
						     0U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_veto(NULL, 0U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_rpc_handle(NULL, NULL, 0U, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_rpc_handle(&g_c, NULL, 4U, NULL, NULL));

	TEST_ASSERT_NOT_EQUAL_UINT32(0U, mp_manifest_hash_cached(&g_c));
}

static void test_error_message_table(void)
{
	static const int codes[] = { 0,
				     MP_E_PARSE,
				     MP_E_INVALID_REQ,
				     MP_E_NO_METHOD,
				     MP_E_BAD_PARAMS,
				     MP_E_INTERNAL,
				     MP_E_NO_SESSION,
				     MP_E_GUARD,
				     MP_E_INTERLOCK,
				     MP_E_RANGE,
				     MP_E_BUSY,
				     MP_E_NOTSUP,
				     MP_E_STATE,
				     MP_E_HOLD,
				     MP_E_VETO,
				     MP_E_IO };
	size_t i;

	for (i = 0U; i < (sizeof(codes) / sizeof(codes[0])); i++) {
		TEST_ASSERT_TRUE(strlen(mp_err_msg(codes[i])) > 0U);
		TEST_ASSERT_TRUE(strcmp(mp_err_msg(codes[i]), "error") != 0);
	}
	TEST_ASSERT_EQUAL_STRING("error", mp_err_msg(-1));
}

/* ---------------------------------------------------------- the mode machine */

static void test_mode_entry_magic(void)
{
	size_t i;

	TEST_ASSERT_EQUAL_INT(0, mp_mode_exit(&g_c));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_MODE_SHELL, mp_mode(&g_c));

	/* Ordinary shell bytes do not trigger it. */
	for (i = 0U; i < 32U; i++) {
		TEST_ASSERT_EQUAL_INT(0, mp_shell_byte(&g_c, (uint8_t)('a' + i)));
	}
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_MODE_SHELL, mp_mode(&g_c));

	/* A partial magic followed by junk does not either. */
	(void)mp_shell_byte(&g_c, 0x01U);
	(void)mp_shell_byte(&g_c, 'M');
	(void)mp_shell_byte(&g_c, 'P');
	TEST_ASSERT_EQUAL_INT(0, mp_shell_byte(&g_c, 'X'));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_MODE_SHELL, mp_mode(&g_c));

	/* The real thing does, on the last byte. */
	for (i = 0U; i < (MP_MAGIC_LEN - 1U); i++) {
		TEST_ASSERT_EQUAL_INT(0,
				      mp_shell_byte(&g_c,
						    (uint8_t)MP_MAGIC_ENTER[i]));
	}
	TEST_ASSERT_EQUAL_INT(1,
			      mp_shell_byte(&g_c,
					    (uint8_t)MP_MAGIC_ENTER[MP_MAGIC_LEN -
								    1U]));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_MODE_MP, mp_mode(&g_c));

	/* In MP mode the shell hook is inert. */
	TEST_ASSERT_EQUAL_INT(0, mp_shell_byte(&g_c, 0x01U));

	/* A repeated first byte still starts a fresh match. */
	TEST_ASSERT_EQUAL_INT(0, mp_mode_exit(&g_c));
	(void)mp_shell_byte(&g_c, 0x01U);
	(void)mp_shell_byte(&g_c, 0x01U);
	(void)mp_shell_byte(&g_c, 'M');
	(void)mp_shell_byte(&g_c, 'P');
	(void)mp_shell_byte(&g_c, '1');
	TEST_ASSERT_EQUAL_INT(1, mp_shell_byte(&g_c, 0x02U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_MODE_MP, mp_mode(&g_c));
}

static void test_mode_exit_magic_and_cleanup(void)
{
	uint32_t sid = session();

	ilk_permissive();
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":\"telemetry\",\"rate_hz\":2,\"sid\":1}}");

	/* An override, so the exit has something to revert. */
	{
		char req[256];

		(void)snprintf(req, sizeof(req),
			       "{\"jsonrpc\":\"2.0\",\"id\":3,"
			       "\"method\":\"obj.override\",\"params\":"
			       "{\"id\":\"ui.disp.bl\",\"value\":40,\"sid\":%u}}",
			       sid);
		(void)call(req);
		TEST_ASSERT_TRUE(res_i("value") == 40);
	}
	TEST_ASSERT_EQUAL_size_t(1U, mp_ovr_active(&g_c.ovr));

	g_apply_n = 0U;
	{
		size_t i;

		for (i = 0U; i < MP_MAGIC_LEN; i++) {
			(void)mp_input(&g_c, (const uint8_t *)&MP_MAGIC_EXIT[i],
				       1U);
		}
	}
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_MODE_SHELL, mp_mode(&g_c));
	/* Leaving MP mode must not leave the board commanded. */
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c.ovr));
	TEST_ASSERT_EQUAL_UINT(1U, g_apply_n);
	TEST_ASSERT_TRUE(g_apply[0].release);
	TEST_ASSERT_FALSE(mp_stream_is_sub(&g_c.st, MP_CH_TELEMETRY));
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.ovr.sess.id);

	/* Input in shell mode is refused. */
	TEST_ASSERT_EQUAL_INT(-EPERM, mp_input(&g_c, (const uint8_t *)"x", 1U));
	/* Exiting twice is a no-op. */
	TEST_ASSERT_EQUAL_INT(0, mp_mode_exit(&g_c));
	/* Entering twice is a no-op. */
	TEST_ASSERT_EQUAL_INT(0, mp_mode_enter(&g_c));
	TEST_ASSERT_EQUAL_INT(0, mp_mode_enter(&g_c));
}

static void test_link_loss_leaves_mp_mode(void)
{
	uint32_t sid = session();
	char req[256];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"ui.disp.bl\",\"value\":40,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_size_t(1U, mp_ovr_active(&g_c.ovr));

	TEST_ASSERT_EQUAL_INT(0, mp_set_link(&g_c, false));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_MODE_SHELL, mp_mode(&g_c));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c.ovr));
}

static void test_exit_aborts_a_running_diagnostic(void)
{
	(void)session();
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"diag.run\","
		   "\"params\":{\"test\":\"ina.selftest\",\"sid\":1}}");
	TEST_ASSERT_TRUE(mp_diag_busy(&g_c.diag));
	TEST_ASSERT_EQUAL_INT(0, mp_mode_exit(&g_c));
	TEST_ASSERT_FALSE(mp_diag_busy(&g_c.diag));
}

/* ------------------------------------------------------------- envelope */

static void test_parse_and_envelope_errors(void)
{
	(void)call("not json");
	TEST_ASSERT_EQUAL_INT64(MP_E_PARSE, err_code());
	/* A parse error carries a null id (JSON-RPC 2.0 §5). */
	TEST_ASSERT_EQUAL_INT(MP_J_NULL,
			      mp_json_at(&g_rp,
					 mp_json_obj_get(&g_rp, 0, "id"))->type);

	(void)call("{\"jsonrpc\":\"2.0\"}");
	TEST_ASSERT_EQUAL_INT64(MP_E_INVALID_REQ, err_code());

	(void)call("{\"jsonrpc\":\"1.0\",\"id\":1,\"method\":\"hello\"}");
	TEST_ASSERT_EQUAL_INT64(MP_E_INVALID_REQ, err_code());

	(void)call("{\"id\":1,\"method\":\"hello\"}");
	TEST_ASSERT_EQUAL_INT64(MP_E_INVALID_REQ, err_code());

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":123}");
	TEST_ASSERT_EQUAL_INT64(MP_E_INVALID_REQ, err_code());

	/* An object id is not a legal JSON-RPC id. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":{},\"method\":\"hello\"}");
	TEST_ASSERT_EQUAL_INT64(MP_E_INVALID_REQ, err_code());

	/* A batch is refused (documented: the reply buffer is bounded). */
	(void)call("[{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"hello\"}]");
	TEST_ASSERT_EQUAL_INT64(MP_E_INVALID_REQ, err_code());

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"nope\"}");
	TEST_ASSERT_EQUAL_INT64(MP_E_NO_METHOD, err_code());

	/* Positional params are refused rather than silently ignored. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"hello\","
		   "\"params\":[1,2]}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
}

static void test_id_is_echoed_verbatim(void)
{
	int id;

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":42,\"method\":\"hello\"}");
	id = mp_json_obj_get(&g_rp, 0, "id");
	TEST_ASSERT_EQUAL_INT(MP_J_NUM, mp_json_at(&g_rp, id)->type);
	{
		int64_t v = 0;

		TEST_ASSERT_EQUAL_INT(0, mp_json_i64(&g_rp, id, &v));
		TEST_ASSERT_EQUAL_INT64(42, v);
	}

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":\"abc-1\",\"method\":\"hello\"}");
	id = mp_json_obj_get(&g_rp, 0, "id");
	TEST_ASSERT_EQUAL_INT(MP_J_STR, mp_json_at(&g_rp, id)->type);
	TEST_ASSERT_TRUE(mp_json_streq(&g_rp, id, "abc-1"));

	/* A string id with an escape survives re-quoting unchanged. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":\"a\\\"b\",\"method\":\"hello\"}");
	id = mp_json_obj_get(&g_rp, 0, "id");
	TEST_ASSERT_TRUE(mp_json_streq(&g_rp, id, "a\"b"));

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":null,\"method\":\"hello\"}");
	id = mp_json_obj_get(&g_rp, 0, "id");
	TEST_ASSERT_EQUAL_INT(MP_J_NULL, mp_json_at(&g_rp, id)->type);

	/* A negative id is still a number and comes back unchanged. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":-9,\"method\":\"hello\"}");
	id = mp_json_obj_get(&g_rp, 0, "id");
	TEST_ASSERT_EQUAL_INT(MP_J_NUM, mp_json_at(&g_rp, id)->type);
	{
		int64_t v = 0;

		TEST_ASSERT_EQUAL_INT(0, mp_json_i64(&g_rp, id, &v));
		TEST_ASSERT_EQUAL_INT64(-9, v);
	}
}

static void test_notification_gets_no_reply(void)
{
	const char *out = NULL;
	size_t len = 1U;

	{
		static const char note[] =
			"{\"jsonrpc\":\"2.0\",\"method\":\"hello\"}";

		TEST_ASSERT_EQUAL_INT(0,
				      mp_rpc_handle(&g_c, (const uint8_t *)note,
						    strlen(note), &out, &len));
	}
	TEST_ASSERT_NULL(out);
	TEST_ASSERT_EQUAL_size_t(0U, len);
	TEST_ASSERT_EQUAL_UINT32(1U, g_c.notifications);

	/* Even a failing notification is silent. */
	{
		static const char bad[] =
			"{\"jsonrpc\":\"2.0\",\"method\":\"nope\"}";

		TEST_ASSERT_EQUAL_INT(0,
				      mp_rpc_handle(&g_c, (const uint8_t *)bad,
						    strlen(bad), &out, &len));
	}
	TEST_ASSERT_NULL(out);
}

/* -------------------------------------------------------------- discovery */

static void test_hello(void)
{
	int r;
	int m;

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"hello\"}");
	r = result();

	TEST_ASSERT_EQUAL_INT64(MP_PROTO_VER, res_i("proto"));
	TEST_ASSERT_TRUE(res_streq("protocol", "meridian-mp"));
	TEST_ASSERT_TRUE(res_streq("model", "STS1000"));
	TEST_ASSERT_TRUE(res_streq("serial", SERIAL));
	TEST_ASSERT_TRUE(res_streq("fw", "1.2.3"));
	TEST_ASSERT_TRUE(res_streq("boot", "0.9.0"));

	/* The tool learns before opening anything that a credential is accepted
	 * here and that it currently holds no role. */
	TEST_ASSERT_TRUE(res_b("auth_required"));
	TEST_ASSERT_TRUE(res_streq("role", "none"));

	m = mp_json_obj_get(&g_rp, r, "manifest");
	TEST_ASSERT_TRUE(m >= 0);
	{
		int64_t hash = 0;
		int64_t objects = 0;

		TEST_ASSERT_EQUAL_INT(0,
				      mp_json_i64(&g_rp,
						  mp_json_obj_get(&g_rp, m,
								  "hash"),
						  &hash));
		{
			static char hb[MP_MANIFEST_OBJ_JSON_MAX];

			TEST_ASSERT_EQUAL_INT64((int64_t)mp_manifest_hash(hb,
									 sizeof(hb)),
						hash);
		}
		TEST_ASSERT_EQUAL_INT(0,
				      mp_json_i64(&g_rp,
						  mp_json_obj_get(&g_rp, m,
								  "objects"),
						  &objects));
		TEST_ASSERT_EQUAL_INT64((int64_t)mp_obj_count(), objects);
		/* No compressor is linked, so this is stated plainly. */
		TEST_ASSERT_TRUE(mp_json_streq(&g_rp,
					       mp_json_obj_get(&g_rp, m,
							       "encoding"),
					       "identity"));
		{
			bool gzip = true;

			TEST_ASSERT_EQUAL_INT(0,
					      mp_json_bool(&g_rp,
							   mp_json_obj_get(&g_rp,
									   m,
									   "gzip"),
							   &gzip));
			TEST_ASSERT_FALSE(gzip);
		}
	}

	/* The channel map is published so the host need not hard-code it. */
	{
		int ch = mp_json_obj_get(&g_rp, r, "channels");
		uint16_t n = mp_json_count(&g_rp, ch);
		uint16_t i;

		TEST_ASSERT_EQUAL_UINT16((uint16_t)MP_CH_MIRROR + 1U, n);
		for (i = 0U; i < n; i++) {
			int e = mp_json_arr_at(&g_rp, ch, i);
			int64_t id = -1;

			TEST_ASSERT_EQUAL_INT(0,
					      mp_json_i64(&g_rp,
							  mp_json_obj_get(&g_rp,
									  e,
									  "id"),
							  &id));
			TEST_ASSERT_EQUAL_INT64(i, id);
			TEST_ASSERT_TRUE(mp_json_streq(&g_rp,
						       mp_json_obj_get(&g_rp, e,
								       "name"),
						       mp_channel_name((uint8_t)i)));
		}
	}

	/* The safety numbers a tool must honour. */
	{
		int l = mp_json_obj_get(&g_rp, r, "limits");
		int64_t v = 0;

		TEST_ASSERT_EQUAL_INT(0,
				      mp_json_i64(&g_rp,
						  mp_json_obj_get(&g_rp, l,
								  "keepalive_ms"),
						  &v));
		TEST_ASSERT_EQUAL_INT64(MP_KEEPALIVE_TTL_MS, v);
		TEST_ASSERT_EQUAL_INT(0,
				      mp_json_i64(&g_rp,
						  mp_json_obj_get(&g_rp, l,
								  "revert_ms"),
						  &v));
		TEST_ASSERT_EQUAL_INT64(MP_DEADMAN_REVERT_MS, v);
		TEST_ASSERT_EQUAL_INT(0,
				      mp_json_i64(&g_rp,
						  mp_json_obj_get(&g_rp, l,
								  "rx_payload"),
						  &v));
		TEST_ASSERT_EQUAL_INT64(MP_PAYLOAD_MAX, v);
	}
}

static void test_manifest_paging(void)
{
	size_t from = 0U;
	unsigned int pages = 0U;
	unsigned int objects = 0U;

	for (;;) {
		char req[128];
		int r;
		int64_t next = 0;

		(void)snprintf(req, sizeof(req),
			       "{\"jsonrpc\":\"2.0\",\"id\":1,"
			       "\"method\":\"manifest.get\",\"params\":"
			       "{\"from\":%u}}",
			       (unsigned int)from);
		(void)call(req);
		r = result();

		TEST_ASSERT_EQUAL_INT64((int64_t)mp_obj_count(),
					res_i("total"));
		TEST_ASSERT_EQUAL_INT64((int64_t)from, res_i("from"));
		objects += (unsigned int)res_i("count");
		TEST_ASSERT_EQUAL_UINT16((uint16_t)res_i("count"),
					 mp_json_count(&g_rp,
						       mp_json_obj_get(&g_rp, r,
								       "objects")));
		next = res_i("next");
		TEST_ASSERT_TRUE(next > (int64_t)from);
		from = (size_t)next;
		pages++;
		TEST_ASSERT_TRUE(pages < 100U);
		if (res_b("done")) {
			break;
		}
	}
	TEST_ASSERT_EQUAL_UINT((unsigned int)mp_obj_count(), objects);
	TEST_ASSERT_TRUE(pages > 1U);

	/* Out of range. */
	{
		char req[128];

		(void)snprintf(req, sizeof(req),
			       "{\"jsonrpc\":\"2.0\",\"id\":1,"
			       "\"method\":\"manifest.get\",\"params\":"
			       "{\"from\":%u}}",
			       (unsigned int)mp_obj_count() + 5U);
		(void)call(req);
		TEST_ASSERT_EQUAL_INT64(MP_E_RANGE, err_code());
	}
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"manifest.get\","
		   "\"params\":{\"from\":\"x\"}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
}

/* ---------------------------------------------------------------- sessions */

static void test_session_methods(void)
{
	uint32_t sid;
	char req[192];

	sid = session();
	TEST_ASSERT_NOT_EQUAL_UINT32(0U, sid);
	TEST_ASSERT_TRUE(res_b("serial_required"));
	TEST_ASSERT_TRUE(res_b("auth_required"));
	TEST_ASSERT_TRUE(res_streq("role", "admin"));
	TEST_ASSERT_TRUE(res_streq("user", ADMIN_USER));
	TEST_ASSERT_EQUAL_INT64(MP_KEEPALIVE_TTL_MS, res_i("keepalive_ms"));
	/* The credential went to the hook, and only there. */
	TEST_ASSERT_EQUAL_UINT(1U, g_auth_calls);
	TEST_ASSERT_EQUAL_STRING(ADMIN_USER, g_auth_last_user);
	TEST_ASSERT_EQUAL_STRING(ADMIN_PW, g_auth_last_secret);

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,"
		       "\"method\":\"session.keepalive\",\"params\":{\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(sid, res_i("sid"));
	TEST_ASSERT_EQUAL_INT64(0, res_i("overrides"));

	/* A wrong id is refused. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,"
		       "\"method\":\"session.keepalive\",\"params\":{\"sid\":%u}}",
		       sid + 7U);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_NO_SESSION, err_code());

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,"
		       "\"method\":\"session.close\",\"params\":{\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_TRUE(res_b("closed"));

	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_NO_SESSION, err_code());
}

/* ================================== §5.3 authentication and the role floor */

/** A G1 action (`obj.set` on ui.disp.bl); returns the reply's error code, or 0. */
static int64_t try_g1(uint32_t sid)
{
	char req[256];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":90,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"ui.disp.bl\",\"value\":42,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	if (mp_json_obj_get(&g_rp, 0, "error") < 0) {
		return 0;
	}
	return err_code();
}

/** A G2 action (`obj.set` on pwr.gps.en) with the correct typed serial. */
static int64_t try_g2_with_serial(uint32_t sid)
{
	char req[256];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":91,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"pwr.gps.en\",\"value\":true,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	if (mp_json_obj_get(&g_rp, 0, "error") < 0) {
		return 0;
	}
	return err_code();
}

/**
 * A session opened with no credential is capped at G0.
 *
 * This is the defect this group exists for: before authentication was wired,
 * `session.open` with no parameters granted every G1 action on the board.
 */
static void test_a_session_with_no_credential_is_capped_at_g0(void)
{
	uint32_t sid = session_anon();

	TEST_ASSERT_NOT_EQUAL_UINT32(0U, sid);
	/* The open itself succeeds — read-only monitoring needs no credential. */
	TEST_ASSERT_TRUE(res_streq("role", "none"));
	TEST_ASSERT_TRUE(res_b("auth_required"));
	TEST_ASSERT_EQUAL_UINT(0U, g_auth_calls);

	/* G0 still answers. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.get\","
		   "\"params\":{\"id\":\"sensor.rail.poe\"}}");
	TEST_ASSERT_TRUE(res_streq("id", "sensor.rail.poe"));

	/* G1 does not, and nothing was applied. */
	TEST_ASSERT_EQUAL_INT64(MP_E_ROLE, try_g1(sid));
	TEST_ASSERT_EQUAL_UINT(0U, g_apply_n);
	TEST_ASSERT_EQUAL_INT64(MP_E_ROLE, try_g2_with_serial(sid));
	TEST_ASSERT_EQUAL_UINT(0U, g_apply_n);
}

/** A viewer may observe but not act. */
static void test_a_viewer_cannot_perform_a_g1_action(void)
{
	uint32_t sid = session_as(VIEW_USER, VIEW_PW);

	TEST_ASSERT_TRUE(res_streq("role", "viewer"));
	TEST_ASSERT_EQUAL_INT64(MP_E_ROLE, try_g1(sid));
	TEST_ASSERT_EQUAL_UINT(0U, g_apply_n);
}

/**
 * An operator gets G1 and is refused G2 **with the correct serial supplied**.
 *
 * The serial is printed on the chassis and returned by `hello`, so typing it
 * must not stand in for an admin credential.
 */
static void test_an_operator_gets_g1_but_not_g2(void)
{
	uint32_t sid = session_as(OPER_USER, OPER_PW);

	TEST_ASSERT_TRUE(res_streq("role", "operator"));

	TEST_ASSERT_EQUAL_INT64(0, try_g1(sid));
	TEST_ASSERT_EQUAL_UINT(1U, g_apply_n);
	TEST_ASSERT_EQUAL_INT32(42, g_apply[0].value);

	TEST_ASSERT_EQUAL_INT64(MP_E_ROLE, try_g2_with_serial(sid));
	TEST_ASSERT_EQUAL_UINT(1U, g_apply_n); /* nothing further applied */
}

/** An admin with the correct serial gets G2. */
static void test_an_admin_with_the_serial_gets_g2(void)
{
	uint32_t sid = session();

	TEST_ASSERT_TRUE(res_streq("role", "admin"));
	TEST_ASSERT_EQUAL_INT64(0, try_g1(sid));
	TEST_ASSERT_EQUAL_INT64(0, try_g2_with_serial(sid));
	TEST_ASSERT_EQUAL_UINT(2U, g_apply_n);
}

/**
 * With no auth hook wired, every session is capped at G0.
 *
 * The fail-closed default: an unwired hook must degrade to read-only, never to
 * full access. A build that forgot to wire it is diagnosable
 * (`auth_required:false`) rather than wide open.
 */
static void test_no_auth_hook_caps_every_session_at_g0(void)
{
	mp_wiring_t w;
	uint32_t sid;

	g_auth_present = false;
	wire_up(&w);
	TEST_ASSERT_EQUAL_INT(0, mp_init(&g_c, &w));
	TEST_ASSERT_EQUAL_INT(0, mp_set_link(&g_c, true));
	TEST_ASSERT_EQUAL_INT(0, mp_mode_enter(&g_c));

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"hello\"}");
	TEST_ASSERT_FALSE(res_b("auth_required"));

	/* Even presenting a credential that the real store would accept. */
	sid = session();
	TEST_ASSERT_NOT_EQUAL_UINT32(0U, sid);
	TEST_ASSERT_FALSE(res_b("auth_required"));
	TEST_ASSERT_TRUE(res_streq("role", "none"));
	TEST_ASSERT_EQUAL_UINT(0U, g_auth_calls);

	TEST_ASSERT_EQUAL_INT64(MP_E_ROLE, try_g1(sid));
	TEST_ASSERT_EQUAL_UINT(0U, g_apply_n);
}

/**
 * A refused credential and an unknown user are indistinguishable.
 *
 * Same code, same message, same `data.reason` — the control plane is not an
 * account oracle. An over-long field and an empty user name join them.
 */
static void test_a_refusal_reveals_nothing_about_the_account(void)
{
	char req[512];
	char first[256];
	size_t len;
	unsigned int i;
	static const char *const bad[] = {
		/* wrong password for a real account */
		"{\"user\":\"" ADMIN_USER "\",\"secret\":\"wrong\"}",
		/* an account that does not exist */
		"{\"user\":\"nosuchuser\",\"secret\":\"wrong\"}",
		/* the right password, the wrong account */
		"{\"user\":\"nosuchuser\",\"secret\":\"" ADMIN_PW "\"}",
		/* a user with no password at all */
		"{\"user\":\"" ADMIN_USER "\"}",
		/* an empty user name */
		"{\"user\":\"\",\"secret\":\"" ADMIN_PW "\"}",
		/* a secret longer than MP_SECRET_MAX */
		"{\"user\":\"" ADMIN_USER "\",\"secret\":\""
		"0123456789012345678901234567890123456789"
		"0123456789012345678901234567890123456789\"}",
		/* a user name longer than MP_USER_MAX */
		"{\"user\":\""
		"uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu"
		"uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu"
		"\",\"secret\":\"" ADMIN_PW "\"}",
	};

	first[0] = '\0';
	for (i = 0U; i < (sizeof(bad) / sizeof(bad[0])); i++) {
		(void)snprintf(req, sizeof(req),
			       "{\"jsonrpc\":\"2.0\",\"id\":1,"
			       "\"method\":\"session.open\",\"params\":%s}",
			       bad[i]);
		len = call(req);
		TEST_ASSERT_EQUAL_INT64_MESSAGE(MP_E_AUTH, err_code(), bad[i]);

		/* Byte-for-byte identical replies, not merely the same code. */
		TEST_ASSERT_TRUE(len < sizeof(first));
		if (first[0] == '\0') {
			memcpy(first, g_rp.src, len);
			first[len] = '\0';
		} else {
			TEST_ASSERT_EQUAL_STRING_MESSAGE(first, g_rp.src,
							 bad[i]);
		}

		/* And no session came into existence. */
		TEST_ASSERT_EQUAL_UINT32(0U, g_c.ovr.sess.id);
	}
}

/**
 * A lockout is reported distinctly, and only a lockout.
 *
 * Telling an operator to wait is operationally necessary and reveals nothing an
 * attacker who caused the lockout does not already know.
 */
static void test_a_lockout_is_reported_distinctly(void)
{
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"session.open\","
		   "\"params\":{\"user\":\"" LOCKED_USER
		   "\",\"secret\":\"whatever\"}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_LOCKED, err_code());
	TEST_ASSERT_EQUAL_UINT32(0U, g_c.ovr.sess.id);
}

/**
 * A failed open leaves the session that is already there alone.
 *
 * Otherwise `session.open` with a junk password would be a way to kick a working
 * tool off the board and revert its overrides — a denial of service that needs
 * no credential.
 */
static void test_a_failed_open_does_not_disturb_the_live_session(void)
{
	uint32_t sid = session();
	char req[256];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"ui.disp.bl\",\"value\":30,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(30, res_i("value"));
	TEST_ASSERT_EQUAL_size_t(1U, mp_ovr_active(&g_c.ovr));

	/* An attacker tries to take over and fails. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"session.open\","
		   "\"params\":{\"user\":\"attacker\",\"secret\":\"guess\"}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_AUTH, err_code());

	/* The original session and its lease survive untouched. */
	TEST_ASSERT_EQUAL_UINT32(sid, g_c.ovr.sess.id);
	TEST_ASSERT_EQUAL_size_t(1U, mp_ovr_active(&g_c.ovr));
	TEST_ASSERT_EQUAL_INT64(0, try_g1(sid));
}

/** A successful takeover does not inherit the previous session's role. */
static void test_a_takeover_does_not_inherit_the_role(void)
{
	uint32_t admin_sid = session();
	uint32_t anon_sid;

	TEST_ASSERT_EQUAL_INT64(0, try_g2_with_serial(admin_sid));

	anon_sid = session_anon();
	TEST_ASSERT_NOT_EQUAL_UINT32(admin_sid, anon_sid);
	TEST_ASSERT_TRUE(res_streq("role", "none"));

	g_apply_n = 0U;
	TEST_ASSERT_EQUAL_INT64(MP_E_ROLE, try_g1(anon_sid));
	TEST_ASSERT_EQUAL_INT64(MP_E_ROLE, try_g2_with_serial(anon_sid));
	TEST_ASSERT_EQUAL_UINT(0U, g_apply_n);

	/* The admin's id is dead too. */
	TEST_ASSERT_EQUAL_INT64(MP_E_NO_SESSION, try_g1(admin_sid));

	/* `hello` now reports the anonymous session's role, not the admin's. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":9,\"method\":\"hello\"}");
	TEST_ASSERT_TRUE(res_streq("role", "none"));
}

/** The credential never reaches the reply, the log or the event stream. */
static void test_the_secret_never_leaves_the_auth_hook(void)
{
	logr_rec_t recs[16];
	logr_filter_t f;
	uint16_t n = 0U;
	uint32_t next = 0U;
	uint32_t gap = 0U;
	uint16_t i;

	(void)session();

	/* Not in the reply. */
	TEST_ASSERT_NULL(strstr(g_rp.src, ADMIN_PW));
	/* Not anywhere in what has been transmitted. */
	TEST_ASSERT_TRUE(g_wire_len < sizeof(g_wire));
	g_wire[g_wire_len] = '\0';
	TEST_ASSERT_NULL(strstr((const char *)g_wire, ADMIN_PW));

	/* Not in the audit log — which does name the user, per §5.3. */
	logr_filter_all(&f);
	TEST_ASSERT_EQUAL_INT(0, logr_tail(&g_log, 0U, &f, recs, 16U, &n, &next,
					   &gap));
	TEST_ASSERT_TRUE(n > 0U);
	{
		bool named = false;

		for (i = 0U; i < n; i++) {
			TEST_ASSERT_NULL_MESSAGE(strstr(recs[i].msg, ADMIN_PW),
						 recs[i].msg);
			if (strstr(recs[i].msg, ADMIN_USER) != NULL) {
				named = true;
			}
		}
		TEST_ASSERT_TRUE_MESSAGE(named,
					 "§5.3: the session open must be audited "
					 "with the user");
	}
}

/**
 * An override audit record names the user (§5.3).
 *
 * The grant and its reversion are both logged, and both carry the name the
 * credential was granted to — an audit trail that says only "ui.disp.bl changed"
 * answers none of the questions an audit trail exists for.
 */
static void test_an_override_is_audited_with_the_user(void)
{
	uint32_t sid = session_as(OPER_USER, OPER_PW);
	logr_rec_t recs[16];
	logr_filter_t f;
	uint16_t n = 0U;
	uint32_t next = 0U;
	uint32_t gap = 0U;
	uint16_t i;
	char req[256];
	bool grant_named = false;
	bool revert_named = false;

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"ui.disp.bl\",\"value\":30,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(30, res_i("value"));

	/* Let the dead-man revert it, which is the path §5.3 requires logged. */
	g_now += MP_KEEPALIVE_TTL_MS + 1U;
	(void)mp_tick(&g_c);
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c.ovr));

	logr_filter_all(&f);
	TEST_ASSERT_EQUAL_INT(0, logr_tail(&g_log, 0U, &f, recs, 16U, &n, &next,
					   &gap));
	for (i = 0U; i < n; i++) {
		if (strstr(recs[i].msg, OPER_USER) == NULL) {
			continue;
		}
		if (strstr(recs[i].msg, "grant") != NULL) {
			grant_named = true;
		}
		if (strstr(recs[i].msg, "deadman") != NULL) {
			revert_named = true;
		}
	}
	TEST_ASSERT_TRUE_MESSAGE(grant_named, "grant not audited with the user");
	TEST_ASSERT_TRUE_MESSAGE(revert_named,
				 "dead-man revert not audited with the user");
}

/**
 * An over-long string param is *emptied*, not truncated.
 *
 * mp_json_str() returns -ENOSPC before writing the NUL, so the destination is
 * left unterminated with its last byte indeterminate; every p_str() caller here
 * then treats it as a C string (guard_or_fail() hands `confirm` straight to
 * strlen()). The `client` echo is the deterministic witness: an over-long value
 * must come back as JSON null, because the buffer was emptied. Without the fix
 * it comes back as 31 bytes of garbage — and the `confirm` path, which has no
 * echo, reads one indeterminate byte instead.
 */
static void test_an_over_long_string_param_is_emptied(void)
{
	uint32_t sid;
	char req[512];
	char big[MP_CONFIRM_MAX + 32U];
	int t;

	memset(big, 'X', sizeof(big) - 1U);
	big[sizeof(big) - 1U] = '\0';

	/* `client` is a 32-byte buffer and is echoed back verbatim. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"session.open\","
		       "\"params\":{\"user\":\"" ADMIN_USER "\",\"secret\":\""
		       ADMIN_PW "\",\"client\":\"%s\"}}",
		       big);
	(void)call(req);
	sid = (uint32_t)res_i("sid");
	t = mp_json_obj_get(&g_rp, result(), "client");
	TEST_ASSERT_TRUE(t >= 0);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_J_NULL, mp_json_at(&g_rp, t)->type);

	/* And an over-long confirmation is a clean refusal, not a comparison
	 * against whatever the stack happened to hold. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"pwr.gps.en\",\"value\":true,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, big);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_GUARD, err_code());
	TEST_ASSERT_EQUAL_UINT(0U, g_apply_n);

	/* An over-long object id is an unknown object, not a truncated match. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"obj.get\","
		       "\"params\":{\"id\":\"%s\"}}",
		       big);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());

	/* The real serial still works afterwards. */
	TEST_ASSERT_EQUAL_INT64(0, try_g2_with_serial(sid));
}

/* ----------------------------------------------------------------- objects */

static void test_obj_get(void)
{
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"obj.get\","
		   "\"params\":{\"id\":\"sensor.rail.vcc_rb\"}}");
	TEST_ASSERT_TRUE(res_streq("id", "sensor.rail.vcc_rb"));
	TEST_ASSERT_TRUE(res_streq("kind", "rail"));
	TEST_ASSERT_TRUE(res_streq("source", "auto"));
	{
		int v = mp_json_obj_get(&g_rp, result(), "value");
		int64_t mv = 0;

		TEST_ASSERT_EQUAL_INT(MP_J_OBJ, mp_json_at(&g_rp, v)->type);
		TEST_ASSERT_EQUAL_INT(0,
				      mp_json_i64(&g_rp,
						  mp_json_obj_get(&g_rp, v,
								  "mv"),
						  &mv));
		TEST_ASSERT_EQUAL_INT64(3300, mv);
	}

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"obj.get\","
		   "\"params\":{\"id\":\"sensor.rb.lock\"}}");
	TEST_ASSERT_TRUE(res_b("value"));

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"obj.get\","
		   "\"params\":{\"id\":\"sensor.alarms\"}}");
	TEST_ASSERT_TRUE(res_streq("kind", "bits"));
	TEST_ASSERT_EQUAL_INT64(0x1234, res_i("value"));
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"obj.get\","
		   "\"params\":{\"id\":\"sensor.pg\"}}");
	TEST_ASSERT_EQUAL_INT64(0x1234, res_i("value"));

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"obj.get\","
		   "\"params\":{\"id\":\"sensor.serial\"}}");
	TEST_ASSERT_TRUE(res_streq("kind", "text"));

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"obj.get\","
		   "\"params\":{\"id\":\"sensor.timing.freq_ppb\"}}");
	TEST_ASSERT_TRUE(res_streq("kind", "real"));

	/* Bad and missing ids. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"obj.get\","
		   "\"params\":{\"id\":\"no.such\"}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"obj.get\"}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"obj.get\","
		   "\"params\":{\"id\":7}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
}

static void test_obj_get_reports_an_override(void)
{
	uint32_t sid = session();
	char req[256];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"ui.disp.bl\",\"value\":35,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"obj.get\","
		   "\"params\":{\"id\":\"ui.disp.bl\"}}");
	TEST_ASSERT_TRUE(res_streq("source", "override"));
	{
		int l = mp_json_obj_get(&g_rp, result(), "lease");
		int64_t v = 0;

		TEST_ASSERT_TRUE(l >= 0);
		TEST_ASSERT_EQUAL_INT(0,
				      mp_json_i64(&g_rp,
						  mp_json_obj_get(&g_rp, l,
								  "value"),
						  &v));
		TEST_ASSERT_EQUAL_INT64(35, v);
	}
}

static void test_obj_set_guard_escalation(void)
{
	uint32_t sid;
	char req[320];

	/* G1 object with no session. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"obj.set\","
		   "\"params\":{\"id\":\"ui.disp.bl\",\"value\":50}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_NO_SESSION, err_code());

	sid = session();
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"ui.disp.bl\",\"value\":50,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(50, res_i("value"));
	TEST_ASSERT_FALSE(res_b("persistent"));
	TEST_ASSERT_EQUAL_UINT(1U, g_apply_n);
	TEST_ASSERT_EQUAL_INT32(50, g_apply[0].value);

	/* A G2 object needs the typed serial. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"pwr.gps.en\",\"value\":true,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_GUARD, err_code());

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"pwr.gps.en\",\"value\":true,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(1, res_i("value"));

	/* A wrong serial is refused. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"pwr.gps.en\",\"value\":false,"
		       "\"sid\":%u,\"confirm\":\"WRONG\"}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_GUARD, err_code());
}

static void test_obj_set_g3_arm_and_complete(void)
{
	uint32_t sid = session();
	char req[384];
	int64_t nonce;

	/* Phase 1: serial + phrase arms and returns a nonce and the hold. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"gnss.safeboot\",\"value\":true,"
		       "\"sid\":%u,\"confirm\":\"%s\",\"phrase\":\"%s\"}}",
		       sid, SERIAL, MP_G3_PHRASE_DEFAULT);
	(void)call(req);
	TEST_ASSERT_TRUE(res_b("armed"));
	TEST_ASSERT_EQUAL_INT64(MP_G3_HOLD_MS, res_i("hold_ms"));
	TEST_ASSERT_TRUE(res_streq("guard", "G3"));
	nonce = res_i("nonce");
	TEST_ASSERT_TRUE(nonce != 0);
	/* Nothing was applied yet. */
	TEST_ASSERT_EQUAL_UINT(0U, g_apply_n);

	/* Too early. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"gnss.safeboot\",\"value\":true,"
		       "\"sid\":%u,\"confirm\":\"%s\",\"nonce\":%lld}}",
		       sid, SERIAL, (long long)nonce);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_HOLD, err_code());
	TEST_ASSERT_EQUAL_UINT(0U, g_apply_n);

	/* After the hold. */
	g_now += MP_G3_HOLD_MS;
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(1, res_i("value"));
	TEST_ASSERT_EQUAL_UINT(1U, g_apply_n);

	/* Replaying the nonce fails: the arm is consumed. */
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_GUARD, err_code());
}

static void test_obj_set_bad_values(void)
{
	uint32_t sid = session();
	char req[320];

	/* No value at all. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"ui.disp.bl\",\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());

	/* A string where a number belongs. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"ui.disp.bl\",\"value\":\"x\","
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());

	/* A read-only object. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"sensor.rb.lock\",\"value\":1,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());

	/* A bool accepts an integer too — a tool that sends 1 is not wrong. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"ref.term.en\",\"value\":1,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(1, res_i("value"));
}

static void test_obj_set_persists_a_cfg_backed_object(void)
{
	uint32_t sid = session();
	char req[320];
	uint64_t v = 0U;

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"pwr.rb.vmax_mv\",\"value\":12000,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(12000, res_i("value"));
	TEST_ASSERT_TRUE(res_b("persistent"));
	TEST_ASSERT_EQUAL_UINT(1U, g_commit_n);
	/* It went through cfg, not through the apply callback. */
	TEST_ASSERT_EQUAL_UINT(0U, g_apply_n);
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, 0x0703U, &v));
	TEST_ASSERT_EQUAL_UINT64(12000U, v);

	/* The manifest's published ceiling is enforced on `set`, not only on
	 * `override` — a cfg-backed object must not be bounded by its schema row
	 * alone. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"pwr.rb.vmax_mv\",\"value\":30000,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_RANGE, err_code());
	/* And the electrical maximum, which the manifest does publish, is fine. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"pwr.rb.vmax_mv\",\"value\":24450,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(24450, res_i("value"));
}

static void test_obj_override_interlock_refusals(void)
{
	uint32_t sid = session();
	char req[384];

	/* RB_PWR_EN with a cold OCXO is refused, and the interlock is named. */
	g_ilk.ocxo_warm = false;
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"pwr.rb.en\",\"value\":true,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_INTERLOCK, err_code());
	{
		int e = mp_json_obj_get(&g_rp, 0, "error");
		int d = mp_json_obj_get(&g_rp, e, "data");

		TEST_ASSERT_TRUE(mp_json_streq(&g_rp,
					       mp_json_obj_get(&g_rp, d,
							       "reason"),
					       "rb.warm"));
	}
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c.ovr));

	/* Warm: permitted. */
	g_ilk.ocxo_warm = true;
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(1, res_i("value"));

	/* The OV latch refuses it again. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"pwr.rb.en\",\"value\":null,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_TRUE(res_b("released"));

	g_ilk.rb_ov_latched = true;
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"pwr.rb.en\",\"value\":true,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_INTERLOCK, err_code());
	{
		int e = mp_json_obj_get(&g_rp, 0, "error");
		int d = mp_json_obj_get(&g_rp, e, "data");

		TEST_ASSERT_TRUE(mp_json_streq(&g_rp,
					       mp_json_obj_get(&g_rp, d,
							       "reason"),
					       "rb.ov"));
	}
}

static void test_obj_override_vcc_rb_is_clamped_to_the_configured_ceiling(void)
{
	uint32_t sid = session();
	char req[384];

	g_ilk.rb_vmax_mv = 15000U;

	/* Asking for the electrical maximum yields the configured ceiling, and
	 * the reply says it was clamped. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"pwr.rb.vset_mv\",\"value\":24450,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(15000, res_i("value"));
	TEST_ASSERT_TRUE(res_b("clamped"));
	/* And the post-set read-back verification is armed. */
	TEST_ASSERT_TRUE(res_b("verify_pending"));
	TEST_ASSERT_EQUAL_INT32(15000, g_apply[0].value);

	/* A lower configured ceiling clamps harder. */
	g_ilk.rb_vmax_mv = 6000U;
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"pwr.rb.vset_mv\",\"value\":15000,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(6000, res_i("value"));
	TEST_ASSERT_TRUE(res_b("clamped"));

	/* Outside the published envelope entirely. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"pwr.rb.vset_mv\",\"value\":40000,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_RANGE, err_code());
}

static void test_obj_override_reports_a_tunnel_as_reference_suspect(void)
{
	uint32_t sid = session();
	char req[384];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"gnss.tunnel\",\"value\":true,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_TRUE(res_b("reference_suspect"));
}

static void test_obj_override_veto_and_not_overridable(void)
{
	uint32_t sid = session();
	char req[320];

	g_apply_rc = -EIO;
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"ui.disp.bl\",\"value\":40,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_VETO, err_code());
	g_apply_rc = 0;

	/* A pulse-only object is not overridable. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"gnss.reset\",\"value\":1,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());

	/* Releasing a lease nobody holds. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"ui.disp.bl\",\"value\":null,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
}

static void test_obj_pulse(void)
{
	uint32_t sid = session();
	char req[320];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.pulse\","
		       "\"params\":{\"id\":\"gnss.extint\",\"ms\":25,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(25, res_i("ms"));
	TEST_ASSERT_EQUAL_UINT(1U, g_pulse_n);
	TEST_ASSERT_EQUAL_UINT32(25U, g_pulse_ms);

	/* Out of the published duration range. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"obj.pulse\","
		       "\"params\":{\"id\":\"gnss.extint\",\"ms\":99999,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_RANGE, err_code());

	/* A non-pulsable object. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"obj.pulse\","
		       "\"params\":{\"id\":\"ui.disp.bl\",\"ms\":10,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());

	/* A pulse still passes the object's interlocks. */
	g_ilk.liveness_ok = false;
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"obj.pulse\","
		       "\"params\":{\"id\":\"sys.wdt.kick\",\"ms\":1,"
		       "\"sid\":%u,\"confirm\":\"%s\",\"phrase\":\"%s\"}}",
		       sid, SERIAL, MP_G3_PHRASE_DEFAULT);
	(void)call(req);
	TEST_ASSERT_TRUE(res_b("armed"));
	{
		int64_t nonce = res_i("nonce");

		g_now += MP_G3_HOLD_MS;
		(void)snprintf(req, sizeof(req),
			       "{\"jsonrpc\":\"2.0\",\"id\":6,"
			       "\"method\":\"obj.pulse\",\"params\":"
			       "{\"id\":\"sys.wdt.kick\",\"ms\":1,\"sid\":%u,"
			       "\"confirm\":\"%s\",\"nonce\":%lld}}",
			       sid, SERIAL, (long long)nonce);
		(void)call(req);
		TEST_ASSERT_EQUAL_INT64(MP_E_INTERLOCK, err_code());
	}

	/* A failing pulse callback is reported. */
	g_ilk.liveness_ok = true;
	g_pulse_rc = -EIO;
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":7,\"method\":\"obj.pulse\","
		       "\"params\":{\"id\":\"gnss.extint\",\"ms\":5,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_IO, err_code());
}

static void test_interlock_state_unavailable_is_reported(void)
{
	uint32_t sid = session();
	char req[320];

	g_ilk_rc = -EIO;
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.set\","
		       "\"params\":{\"id\":\"ui.disp.bl\",\"value\":50,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_IO, err_code());
}

/* ----------------------------------------------------------------- streams */

static void test_stream_sub_unsub(void)
{
	(void)session();

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":1,\"rate_hz\":5,\"sid\":1}}");
	TEST_ASSERT_EQUAL_INT64(1, res_i("ch"));
	TEST_ASSERT_EQUAL_INT64(5, res_i("rate_hz"));
	TEST_ASSERT_TRUE(res_b("paced"));

	/* By name too. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":\"mirror\",\"rate_hz\":10,\"sid\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_CH_MIRROR, res_i("ch"));

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":\"event\",\"sid\":1}}");
	TEST_ASSERT_FALSE(res_b("paced"));

	/* Rate out of range, unknown channel, non-subscribable channel. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":1,\"rate_hz\":50,\"sid\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_RANGE, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":6,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":\"nope\",\"sid\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":7,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":0,\"sid\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":8,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":99,\"sid\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":9,\"method\":\"stream.sub\","
		   "\"params\":{\"sid\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());

	/* A subscription needs a session. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":10,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":1,\"rate_hz\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_NO_SESSION, err_code());

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":11,\"method\":\"stream.unsub\","
		   "\"params\":{\"ch\":1}}");
	TEST_ASSERT_FALSE(res_b("subscribed"));
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":12,\"method\":\"stream.unsub\","
		   "\"params\":{\"ch\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
}

static void test_tick_pumps_subscribed_streams(void)
{
	int sent;

	(void)session();
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":\"telemetry\",\"rate_hz\":10,\"sid\":1}}");
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":\"pps\",\"rate_hz\":10,\"sid\":1}}");
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":\"log\",\"rate_hz\":10,\"sid\":1}}");
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":\"mirror\",\"rate_hz\":10,\"sid\":1}}");
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":6,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":\"event\",\"sid\":1}}");

	TEST_ASSERT_EQUAL_INT(0, logr_puts(&g_log, (uint8_t)LOGR_INFO,
					   (uint8_t)LOGR_SUB_SYS, 1U, "hello"));
	TEST_ASSERT_EQUAL_INT(0, mp_post_event(&g_c, (uint8_t)MP_EV_FAULT, 1U,
					       23U, 1U, 0, g_now, "pg"));

	g_wire_len = 0U;
	g_frames = 0U;
	sent = mp_tick(&g_c);
	/* telemetry + pps + log + mirror + events */
	TEST_ASSERT_EQUAL_INT(5, sent);
	TEST_ASSERT_TRUE(g_frames >= 5U);

	/* Nothing is due 1 ms later. */
	g_now += 1U;
	TEST_ASSERT_EQUAL_INT(0, mp_tick(&g_c));

	/* A tick in shell mode pumps nothing. */
	TEST_ASSERT_EQUAL_INT(0, mp_mode_exit(&g_c));
	g_now += 1000U;
	TEST_ASSERT_EQUAL_INT(0, mp_tick(&g_c));
}

static void test_tick_tolerates_absent_providers(void)
{
	mp_wiring_t w;

	g_telem_present = false;
	g_pps_present = false;
	g_mirror_present = false;
	g_ilk_present = false;
	wire_up(&w);
	w.log = NULL;
	TEST_ASSERT_EQUAL_INT(0, mp_init(&g_c, &w));
	TEST_ASSERT_EQUAL_INT(0, mp_set_link(&g_c, true));
	TEST_ASSERT_EQUAL_INT(0, mp_mode_enter(&g_c));

	(void)session();
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":\"telemetry\",\"rate_hz\":10,\"sid\":1}}");
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":\"pps\",\"rate_hz\":10,\"sid\":1}}");
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":\"log\",\"rate_hz\":10,\"sid\":1}}");
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":\"mirror\",\"rate_hz\":10,\"sid\":1}}");

	/* Every provider is missing: nothing is sent, nothing faults. */
	TEST_ASSERT_EQUAL_INT(0, mp_tick(&g_c));

	/* And the affected requests answer -ENOTSUP rather than crashing. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":6,\"method\":\"mirror.get\"}");
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":7,\"method\":\"log.fetch\"}");
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());
}

static void test_stream_raw_tee(void)
{
	static const uint8_t nmea[] = "$GPZDA,000000.00,01,01,2026,00,00*00\r\n";

	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      mp_stream_raw(&g_c, MP_CH_NMEA, nmea,
					    sizeof(nmea) - 1U));

	(void)session();
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"stream.sub\","
		   "\"params\":{\"ch\":\"nmea\",\"sid\":1}}");

	g_frames = 0U;
	TEST_ASSERT_EQUAL_INT((int)(sizeof(nmea) - 1U),
			      mp_stream_raw(&g_c, MP_CH_NMEA, nmea,
					    sizeof(nmea) - 1U));
	TEST_ASSERT_EQUAL_UINT(1U, g_frames);

	/* The SMP tunnel is not a subscription and is always accepted. */
	TEST_ASSERT_EQUAL_INT(2, mp_stream_raw(&g_c, MP_CH_SMP,
					       (const uint8_t *)"ab", 2U));

	/* A non-raw channel is an argument error. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_stream_raw(&g_c, MP_CH_CONTROL, nmea, 4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_stream_raw(&g_c, MP_CH_NMEA, NULL,
						     4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_stream_raw(NULL, MP_CH_NMEA, nmea,
						     4U));

	/* In shell mode nothing is teed. */
	TEST_ASSERT_EQUAL_INT(0, mp_mode_exit(&g_c));
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      mp_stream_raw(&g_c, MP_CH_NMEA, nmea, 4U));
}

/* ------------------------------------------------------------- diagnostics */

static void test_diag_list(void)
{
	int r;
	int tests;
	uint16_t i;

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"diag.list\"}");
	r = result();
	TEST_ASSERT_EQUAL_INT64(MP_DIAG_COUNT, res_i("count"));
	tests = mp_json_obj_get(&g_rp, r, "tests");
	TEST_ASSERT_EQUAL_UINT16((uint16_t)MP_DIAG_COUNT,
				 mp_json_count(&g_rp, tests));
	for (i = 0U; i < (uint16_t)MP_DIAG_COUNT; i++) {
		int e = mp_json_arr_at(&g_rp, tests, i);

		TEST_ASSERT_TRUE(mp_json_streq(&g_rp,
					       mp_json_obj_get(&g_rp, e,
							       "name"),
					       mp_diag_tests[i].name));
		TEST_ASSERT_TRUE(mp_json_streq(&g_rp,
					       mp_json_obj_get(&g_rp, e,
							       "guard"),
					       mp_guard_name(mp_diag_tests[i].guard)));
	}
}

static void test_diag_run(void)
{
	uint32_t sid = session();
	char req[320];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"diag.run\","
		       "\"params\":{\"test\":\"i2c.scan\",\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_TRUE(res_streq("test", "i2c.scan"));
	TEST_ASSERT_TRUE(res_i("run_id") > 0);
	TEST_ASSERT_EQUAL_INT64(MP_CH_EVENT, res_i("channel"));
	TEST_ASSERT_TRUE(mp_diag_busy(&g_c.diag));

	/* Only one run at a time. */
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_BUSY, err_code());

	/* Stepping it to completion queues progress and result events. */
	while (mp_diag_busy(&g_c.diag)) {
		g_now += 10U;
		TEST_ASSERT_TRUE(mp_tick(&g_c) >= 0);
	}
	TEST_ASSERT_TRUE(mp_stream_event_count(&g_c.st) >= 3U);

	/* An unknown test, and a deferred one. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"diag.run\","
		       "\"params\":{\"test\":\"no.such\",\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"diag.run\","
		       "\"params\":{\"test\":\"sec.attest\",\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"diag.run\","
		   "\"params\":{\"sid\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
}

static void test_diag_run_guards_and_interlocks(void)
{
	uint32_t sid = session();
	char req[384];

	/* A G2 diagnostic needs the serial. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"diag.run\","
		       "\"params\":{\"test\":\"relay.exercise\",\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_GUARD, err_code());

	/* And its interlock: not over a live fault. */
	g_ilk.relay_blocked = true;
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"diag.run\","
		       "\"params\":{\"test\":\"relay.exercise\",\"sid\":%u,"
		       "\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_INTERLOCK, err_code());

	g_ilk.relay_blocked = false;
	(void)call(req);
	TEST_ASSERT_TRUE(res_i("run_id") > 0);
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"diag.abort\","
		   "\"params\":{\"sid\":1}}");
	TEST_ASSERT_TRUE(res_b("aborted"));
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"diag.abort\","
		   "\"params\":{\"sid\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());

	/* The G3 watchdog test arms; its interlock is checked at completion. */
	g_ilk.liveness_ok = false;
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":6,\"method\":\"diag.run\","
		       "\"params\":{\"test\":\"wdt.test\",\"sid\":%u,"
		       "\"confirm\":\"%s\",\"phrase\":\"%s\"}}",
		       sid, SERIAL, MP_G3_PHRASE_DEFAULT);
	(void)call(req);
	TEST_ASSERT_TRUE(res_b("armed"));
	{
		int64_t nonce = res_i("nonce");

		g_now += MP_G3_HOLD_MS;
		(void)snprintf(req, sizeof(req),
			       "{\"jsonrpc\":\"2.0\",\"id\":7,"
			       "\"method\":\"diag.run\",\"params\":"
			       "{\"test\":\"wdt.test\",\"sid\":%u,"
			       "\"confirm\":\"%s\",\"nonce\":%lld}}",
			       sid, SERIAL, (long long)nonce);
		(void)call(req);
		TEST_ASSERT_EQUAL_INT64(MP_E_INTERLOCK, err_code());
	}
}

static void test_diag_snapshot(void)
{
	(void)session();

	g_frames = 0U;
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"diag.snapshot\","
		   "\"params\":{\"sid\":1}}");
	TEST_ASSERT_TRUE(res_i("bytes") > 0);
	TEST_ASSERT_EQUAL_INT64(MP_CH_TELEMETRY, res_i("channel"));
	TEST_ASSERT_EQUAL_INT64(MP_REC_BUNDLE, res_i("record"));
	TEST_ASSERT_TRUE(res_i("crc32") != 0);
	/* The archive itself went out on the telemetry channel. */
	TEST_ASSERT_TRUE(g_frames >= 1U);

	/* With no bundle provider it is -ENOTSUP, not a crash. */
	{
		mp_wiring_t w;

		g_bundle_present = false;
		wire_up(&w);
		TEST_ASSERT_EQUAL_INT(0, mp_init(&g_c, &w));
		TEST_ASSERT_EQUAL_INT(0, mp_set_link(&g_c, true));
		TEST_ASSERT_EQUAL_INT(0, mp_mode_enter(&g_c));
		(void)session();
		(void)call("{\"jsonrpc\":\"2.0\",\"id\":2,"
			   "\"method\":\"diag.snapshot\",\"params\":{\"sid\":1}}");
		TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());
	}
}

/* ------------------------------------------------------------ calibration */

static void test_cal_list_and_get(void)
{
	int r;
	int keys;
	uint16_t n;
	uint16_t i;

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"cal.list\"}");
	r = result();
	TEST_ASSERT_EQUAL_INT64(CFG_G_CAL, res_i("group"));
	keys = mp_json_obj_get(&g_rp, r, "keys");
	n = mp_json_count(&g_rp, keys);
	TEST_ASSERT_TRUE(n >= 13U); /* nine INA trims + PPS/tempco/DAC */
	for (i = 0U; i < n; i++) {
		int e = mp_json_arr_at(&g_rp, keys, i);
		int64_t key = 0;

		TEST_ASSERT_EQUAL_INT(0,
				      mp_json_i64(&g_rp,
						  mp_json_obj_get(&g_rp, e,
								  "key"),
						  &key));
		/* cal.* is scoped to group 0x0C. */
		TEST_ASSERT_EQUAL_INT(CFG_G_CAL, (int)((key >> 8) & 0xFF));
	}

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"cal.get\","
		   "\"params\":{\"key\":\"cal.ina.0\"}}");
	{
		int k = mp_json_obj_get(&g_rp, result(), "key");
		int64_t id = 0;

		TEST_ASSERT_EQUAL_INT(0,
				      mp_json_i64(&g_rp,
						  mp_json_obj_get(&g_rp, k,
								  "key"),
						  &id));
		TEST_ASSERT_EQUAL_INT64(0x0C01, id);
	}

	/* By numeric key too. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"cal.get\","
		   "\"params\":{\"key\":3073}}"); /* 0x0C01 */
	TEST_ASSERT_TRUE(result() >= 0);

	/* A key outside the cal group is refused: cal.* must not become a
	 * second, unguarded path into the whole config tree. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"cal.get\","
		   "\"params\":{\"key\":\"pwr.rb.vmax.mv\"}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"cal.get\","
		   "\"params\":{\"key\":\"no.such.key\"}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":6,\"method\":\"cal.get\"}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
}

static void test_cal_set_commit_revert(void)
{
	uint32_t sid = session();
	char req[320];
	uint64_t v = 0U;

	/* Calibration is G2. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"cal.set\","
		       "\"params\":{\"key\":\"cal.ina.0\",\"value\":4100,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_GUARD, err_code());

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"cal.set\","
		       "\"params\":{\"key\":\"cal.ina.0\",\"value\":4100,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_TRUE(res_b("staged"));
	TEST_ASSERT_EQUAL_INT64(1, res_i("pending"));
	/* Staged, not yet live. */
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, 0x0C01U, &v));
	TEST_ASSERT_EQUAL_UINT64(4096U, v);

	/* A float cal key. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"cal.set\","
		       "\"params\":{\"key\":\"cal.tempco\",\"value\":-0.25,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_TRUE(res_b("staged"));

	/* And a signed one. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"cal.set\","
		       "\"params\":{\"key\":\"cal.pps.ns\",\"value\":-137,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_TRUE(res_b("staged"));

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":6,\"method\":\"cal.commit\","
		       "\"params\":{\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_TRUE(res_b("ok"));
	TEST_ASSERT_TRUE(res_i("applied") >= 3);
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, 0x0C01U, &v));
	TEST_ASSERT_EQUAL_UINT64(4100U, v);

	/* Revert drops a staged set. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":7,\"method\":\"cal.set\","
		       "\"params\":{\"key\":\"cal.ina.1\",\"value\":4200,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":8,\"method\":\"cal.revert\","
		       "\"params\":{\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_TRUE(res_b("reverted"));
	TEST_ASSERT_EQUAL_UINT16(0U, cfg_staged_count(&g_cfg));

	/* Bad value types. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":9,\"method\":\"cal.set\","
		       "\"params\":{\"key\":\"cal.ina.0\",\"value\":\"x\","
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());

	/* A failing commit is reported. */
	g_commit_rc = -EPROTO;
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":10,\"method\":\"cal.commit\","
		       "\"params\":{\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_STATE, err_code());
}

/* ---------------------------------------------------------- cfg transfer */

static void test_cfg_export_import_round_trip(void)
{
	uint32_t sid = session();
	static char req[2048];
	static uint8_t image[8192];
	size_t image_len = 0U;
	unsigned int chunks = 0U;
	uint64_t v = 0U;

	/* Export the whole tree, chunk by chunk. */
	for (;;) {
		char b64[1024];
		int r;
		int n;

		(void)snprintf(req, sizeof(req),
			       "{\"jsonrpc\":\"2.0\",\"id\":2,"
			       "\"method\":\"cfg.export\",\"params\":{\"sid\":%u}}",
			       sid);
		(void)call(req);
		r = result();
		TEST_ASSERT_TRUE(res_streq("encoding", "base64"));
		TEST_ASSERT_EQUAL_INT(0,
				      mp_json_str(&g_rp,
						  mp_json_obj_get(&g_rp, r,
								  "data"),
						  b64, sizeof(b64)) >= 0
					      ? 0
					      : -1);
		n = mp_b64_decode(b64, strlen(b64), &image[image_len],
				  sizeof(image) - image_len);
		TEST_ASSERT_TRUE(n >= 0);
		image_len += (size_t)n;
		chunks++;
		TEST_ASSERT_TRUE(chunks < 300U);
		if (res_b("done")) {
			break;
		}
	}
	TEST_ASSERT_TRUE(chunks > 1U);
	TEST_ASSERT_TRUE(image_len > CFG_EXPORT_HDR_LEN);

	/* Change something so the import has an observable effect. */
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, 0x0C01U, 4200U));
	{
		cfg_commit_res_t cres;

		TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, &cres));
	}
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, 0x0C01U, &v));
	TEST_ASSERT_EQUAL_UINT64(4200U, v);

	/* Import it back. */
	{
		size_t off = 0U;
		bool first = true;

		while (off < image_len) {
			size_t take = image_len - off;
			static char b64[1024];

			if (take > 320U) {
				take = 320U;
			}
			TEST_ASSERT_TRUE(mp_b64_encode(&image[off], take, b64,
						       sizeof(b64)) >= 0);
			(void)snprintf(req, sizeof(req),
				       "{\"jsonrpc\":\"2.0\",\"id\":3,"
				       "\"method\":\"cfg.import\",\"params\":"
				       "{\"sid\":%u,\"confirm\":\"%s\","
				       "\"restart\":%s,\"data\":\"%s\"}}",
				       sid, SERIAL, first ? "true" : "false",
				       b64);
			(void)call(req);
			TEST_ASSERT_TRUE(result() >= 0);
			off += take;
			first = false;
			if (res_b("done")) {
				break;
			}
		}
		TEST_ASSERT_TRUE(res_b("done"));
		TEST_ASSERT_TRUE(res_b("ok"));
	}

	/* The original value is back. */
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, 0x0C01U, &v));
	TEST_ASSERT_EQUAL_UINT64(4096U, v); /* cal.ina.0 schema default */
}

static void test_cfg_transfer_errors(void)
{
	uint32_t sid = session();
	char req[512];

	/* An import is G2. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"cfg.import\","
		       "\"params\":{\"sid\":%u,\"data\":\"AAAA\"}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_GUARD, err_code());

	/* Garbage base64. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"cfg.import\","
		       "\"params\":{\"sid\":%u,\"confirm\":\"%s\","
		       "\"data\":\"!!!!\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());

	/* No data at all. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"cfg.import\","
		       "\"params\":{\"sid\":%u,\"confirm\":\"%s\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());

	/* A bad magic aborts the stream. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"cfg.import\","
		       "\"params\":{\"sid\":%u,\"confirm\":\"%s\","
		       "\"restart\":true,\"data\":\"AAAAAAAAAAAA\"}}",
		       sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_TRUE(err_code() < 0);

	/* An export needs only a session. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":6,\"method\":\"cfg.export\"}");
	TEST_ASSERT_EQUAL_INT64(MP_E_NO_SESSION, err_code());
}

static void test_cfg_methods_without_a_cfg_context(void)
{
	mp_wiring_t w;

	wire_up(&w);
	w.cfg = NULL;
	TEST_ASSERT_EQUAL_INT(0, mp_init(&g_c, &w));
	TEST_ASSERT_EQUAL_INT(0, mp_set_link(&g_c, true));
	TEST_ASSERT_EQUAL_INT(0, mp_mode_enter(&g_c));
	(void)session();

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"cal.list\"}");
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"cal.get\","
		   "\"params\":{\"key\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"cal.set\","
		   "\"params\":{\"key\":1,\"value\":1,\"sid\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"cal.commit\","
		   "\"params\":{\"sid\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":6,\"method\":\"cal.revert\","
		   "\"params\":{\"sid\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":7,\"method\":\"cfg.export\","
		   "\"params\":{\"sid\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":8,\"method\":\"cfg.import\","
		   "\"params\":{\"sid\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());
}

/* ---------------------------------------------------------------- log.fetch */

static void test_log_fetch(void)
{
	int r;
	int recs;
	unsigned int i;

	for (i = 0U; i < 5U; i++) {
		char msg[16];

		(void)snprintf(msg, sizeof(msg), "line%u", i);
		TEST_ASSERT_TRUE(logr_puts(&g_log, (uint8_t)LOGR_WARN,
					   (uint8_t)LOGR_SUB_PWR, 1000U + i,
					   msg) == 0);
	}

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"log.fetch\","
		   "\"params\":{\"cursor\":0,\"max\":8}}");
	r = result();
	TEST_ASSERT_EQUAL_INT64(5, res_i("count"));
	TEST_ASSERT_EQUAL_INT64(0, res_i("gap"));
	recs = mp_json_obj_get(&g_rp, r, "records");
	TEST_ASSERT_EQUAL_UINT16(5U, mp_json_count(&g_rp, recs));
	{
		int e = mp_json_arr_at(&g_rp, recs, 0U);
		char text[32];

		TEST_ASSERT_EQUAL_UINT16(5U, mp_json_count(&g_rp, e));
		TEST_ASSERT_TRUE(mp_json_str(&g_rp,
					     mp_json_arr_at(&g_rp, e, 4U), text,
					     sizeof(text)) > 0);
		TEST_ASSERT_EQUAL_STRING("line0", text);
	}

	/* Level filter: nothing at CRIT or above. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"log.fetch\","
		   "\"params\":{\"cursor\":0,\"level\":2}}");
	TEST_ASSERT_EQUAL_INT64(0, res_i("count"));

	/* Subsystem filter. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"log.fetch\","
		   "\"params\":{\"cursor\":0,\"subs\":128}}"); /* bit 7 = PWR */
	TEST_ASSERT_EQUAL_INT64(5, res_i("count"));
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"log.fetch\","
		   "\"params\":{\"cursor\":0,\"subs\":1}}"); /* bit 0 = SYS */
	TEST_ASSERT_EQUAL_INT64(0, res_i("count"));

	/* `max` is clamped to what one reply can carry. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"log.fetch\","
		   "\"params\":{\"cursor\":0,\"max\":9999}}");
	TEST_ASSERT_TRUE(res_i("count") <= 8);

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":6,\"method\":\"log.fetch\","
		   "\"params\":{\"level\":99}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_RANGE, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":7,\"method\":\"log.fetch\","
		   "\"params\":{\"subs\":\"x\"}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
}

/* ------------------------------------------------------------------- system */

static void test_sys_reboot_is_g3_and_deferred(void)
{
	uint32_t sid = session();
	char req[384];
	int64_t nonce;

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"sys.reboot\","
		       "\"params\":{\"mode\":0,\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_GUARD, err_code());

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"sys.reboot\","
		       "\"params\":{\"mode\":0,\"sid\":%u,\"confirm\":\"%s\","
		       "\"phrase\":\"%s\"}}",
		       sid, SERIAL, MP_G3_PHRASE_DEFAULT);
	(void)call(req);
	TEST_ASSERT_TRUE(res_b("armed"));
	nonce = res_i("nonce");
	TEST_ASSERT_EQUAL_UINT(0U, g_reboot_n);

	g_now += MP_G3_HOLD_MS;
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"sys.reboot\","
		       "\"params\":{\"mode\":0,\"sid\":%u,\"confirm\":\"%s\","
		       "\"nonce\":%lld}}",
		       sid, SERIAL, (long long)nonce);
	(void)call(req);
	TEST_ASSERT_TRUE(res_b("rebooting"));
	/* The reboot is deferred so the reply reaches the host first. */
	TEST_ASSERT_EQUAL_UINT(0U, g_reboot_n);
	TEST_ASSERT_EQUAL_UINT8(1U, g_c.pending_reboot);

	TEST_ASSERT_TRUE(mp_tick(&g_c) >= 0);
	TEST_ASSERT_EQUAL_UINT(1U, g_reboot_n);
	TEST_ASSERT_EQUAL_INT(0, g_reboot_mode);

	/* An out-of-range mode. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"sys.reboot\","
		       "\"params\":{\"mode\":9,\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_RANGE, err_code());
}

static void test_sys_reboot_arm_is_bound_to_the_mode(void)
{
	uint32_t sid = session();
	char req[384];
	int64_t nonce;

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"sys.reboot\","
		       "\"params\":{\"mode\":0,\"sid\":%u,\"confirm\":\"%s\","
		       "\"phrase\":\"%s\"}}",
		       sid, SERIAL, MP_G3_PHRASE_DEFAULT);
	(void)call(req);
	nonce = res_i("nonce");

	/* An arm for "normal" must not complete as "stay in bootloader". */
	g_now += MP_G3_HOLD_MS;
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"sys.reboot\","
		       "\"params\":{\"mode\":1,\"sid\":%u,\"confirm\":\"%s\","
		       "\"nonce\":%lld}}",
		       sid, SERIAL, (long long)nonce);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_GUARD, err_code());
	TEST_ASSERT_EQUAL_UINT8(0U, g_c.pending_reboot);
}

static void test_sys_bootloader(void)
{
	uint32_t sid = session();
	char req[384];
	int64_t nonce;

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,"
		       "\"method\":\"sys.bootloader\",\"params\":{\"sid\":%u,"
		       "\"confirm\":\"%s\",\"phrase\":\"%s\"}}",
		       sid, SERIAL, MP_G3_PHRASE_DEFAULT);
	(void)call(req);
	nonce = res_i("nonce");

	g_now += MP_G3_HOLD_MS;
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,"
		       "\"method\":\"sys.bootloader\",\"params\":{\"sid\":%u,"
		       "\"confirm\":\"%s\",\"nonce\":%lld}}",
		       sid, SERIAL, (long long)nonce);
	(void)call(req);
	TEST_ASSERT_TRUE(res_streq("target", "bootloader"));
	TEST_ASSERT_TRUE(mp_tick(&g_c) >= 0);
	TEST_ASSERT_EQUAL_INT(1, g_reboot_mode); /* stay in serial recovery */
}

static void test_sys_methods_without_an_image_port(void)
{
	mp_wiring_t w;

	wire_up(&w);
	w.img = NULL;
	TEST_ASSERT_EQUAL_INT(0, mp_init(&g_c, &w));
	TEST_ASSERT_EQUAL_INT(0, mp_set_link(&g_c, true));
	TEST_ASSERT_EQUAL_INT(0, mp_mode_enter(&g_c));
	(void)session();

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"sys.reboot\","
		   "\"params\":{\"mode\":0,\"sid\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":3,"
		   "\"method\":\"sys.bootloader\",\"params\":{\"sid\":1}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());

	/* `hello` then omits the sys feature. */
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"hello\"}");
	{
		int f = mp_json_obj_get(&g_rp, result(), "features");
		uint16_t n = mp_json_count(&g_rp, f);
		uint16_t i;
		bool found = false;

		for (i = 0U; i < n; i++) {
			if (mp_json_streq(&g_rp, mp_json_arr_at(&g_rp, f, i),
					  "sys")) {
				found = true;
			}
		}
		TEST_ASSERT_FALSE(found);
	}
}

static void test_sys_mode_returns_to_the_shell(void)
{
	uint32_t sid = session();
	char req[256];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"ui.disp.bl\",\"value\":40,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"sys.mode\","
		   "\"params\":{\"mode\":\"shell\"}}");
	TEST_ASSERT_TRUE(res_streq("mode", "shell"));
	TEST_ASSERT_TRUE(res_b("reverted"));
	/* Still in MP mode until the reply has been sent. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_MODE_MP, mp_mode(&g_c));
	TEST_ASSERT_TRUE(g_c.pending_exit);

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"sys.mode\","
		   "\"params\":{\"mode\":\"mp\"}}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"sys.mode\"}");
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
}

static void test_sys_status(void)
{
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"sys.status\"}");
	TEST_ASSERT_TRUE(res_i("requests") > 0);
	{
		int r = result();

		TEST_ASSERT_TRUE(mp_json_obj_get(&g_rp, r, "frames") >= 0);
		TEST_ASSERT_TRUE(mp_json_obj_get(&g_rp, r, "events") >= 0);
		TEST_ASSERT_TRUE(mp_json_obj_get(&g_rp, r, "mirror") >= 0);
	}
}

static void test_time_get(void)
{
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"time.get\"}");
	TEST_ASSERT_EQUAL_INT64(1700000000000000000LL, res_i("tai_ns"));
	TEST_ASSERT_TRUE(res_b("traceable"));
	TEST_ASSERT_EQUAL_INT64(1, res_i("stratum"));
	TEST_ASSERT_TRUE(res_streq("lock",
				   quality_lock_state_name((uint8_t)QUALITY_LOCK_LOCKED)));
	TEST_ASSERT_TRUE(res_streq("ref",
				   quality_ref_name((uint8_t)QUALITY_REF_OCXO)));
	TEST_ASSERT_TRUE(res_b("utc_valid"));
	{
		int l = mp_json_obj_get(&g_rp, result(), "leap");
		int64_t cur = 0;

		TEST_ASSERT_EQUAL_INT(0,
				      mp_json_i64(&g_rp,
						  mp_json_obj_get(&g_rp, l,
								  "current_s"),
						  &cur));
		TEST_ASSERT_EQUAL_INT64(37, cur);
	}
}

/* ------------------------------------------------------------------ mirror */

static void test_mirror_get(void)
{
	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"mirror.get\"}");
	TEST_ASSERT_EQUAL_INT64(MP_CH_MIRROR, res_i("channel"));
	TEST_ASSERT_EQUAL_INT64(MP_REC_MIRROR, res_i("record"));
	TEST_ASSERT_EQUAL_INT64(MP_MIRROR_VER, res_i("ver"));
	TEST_ASSERT_TRUE(res_b("keyframe"));
	TEST_ASSERT_TRUE(res_i("bytes") > 100);
	TEST_ASSERT_TRUE(res_i("seq") > 0);
	/* The keyframe itself went out on channel 0x0A. */
	TEST_ASSERT_TRUE(g_frames >= 1U);
}

/* ---------------------------------------------------- veto and event feed */

static void test_veto_and_event_feed(void)
{
	uint32_t sid = session();
	char req[256];
	size_t obj = (size_t)mp_obj_find("ui.disp.bl");

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,\"method\":\"obj.override\","
		       "\"params\":{\"id\":\"ui.disp.bl\",\"value\":40,"
		       "\"sid\":%u}}",
		       sid);
	(void)call(req);

	/* Firmware withdraws it; the reversion is queued as an event. */
	TEST_ASSERT_EQUAL_INT(0, mp_veto(&g_c, obj, "thermal shed"));
	TEST_ASSERT_EQUAL_size_t(0U, mp_ovr_active(&g_c.ovr));
	TEST_ASSERT_TRUE(mp_stream_event_count(&g_c.st) >= 1U);
	/* And it is logged, not only streamed (FMT §5.3). */
	TEST_ASSERT_TRUE(logr_count(&g_log) > 0U);

	TEST_ASSERT_EQUAL_INT(-ENOENT, mp_veto(&g_c, obj, NULL));
	TEST_ASSERT_EQUAL_INT(0, mp_post_event(&g_c, (uint8_t)MP_EV_PROX, 0U,
					       10U, 1U, 1, g_now, "door"));
}

/* ------------------------------------------------------- end to end frames */

/** Frame @p req and feed it to the device one byte at a time. */
static void call_framed(const char *req)
{
	static mp_frame_tx_t tx;
	static uint8_t copy[COBS_ENCODE_MAX(MP_TX_FRAME_MAX) + 1U];
	const uint8_t *wire = NULL;
	size_t wire_len = 0U;
	size_t i;

	TEST_ASSERT_EQUAL_INT(0, mp_frame_encode(&tx, MP_CH_CONTROL, 0U,
						 (const uint8_t *)req,
						 strlen(req), &wire,
						 &wire_len));
	TEST_ASSERT_TRUE(wire_len <= sizeof(copy));
	memcpy(copy, wire, wire_len);

	/* Reset the capture *after* encoding, so it holds only the reply. */
	g_wire_len = 0U;
	g_frames = 0U;
	for (i = 0U; i < wire_len; i++) {
		(void)mp_input(&g_c, &copy[i], 1U);
	}
}

static uint8_t g_rx_ch;
static uint8_t g_rx_msg[4096];
static size_t g_rx_len;
static unsigned int g_rx_n;

static int host_rx(void *user, uint8_t ch, const uint8_t *msg, size_t len)
{
	(void)user;
	g_rx_ch = ch;
	g_rx_len = (len <= sizeof(g_rx_msg)) ? len : sizeof(g_rx_msg);
	memcpy(g_rx_msg, msg, g_rx_len);
	g_rx_n++;
	return 0;
}

static void test_end_to_end_over_frames(void)
{
	static mp_frame_rx_t host;
	static mp_reasm_t slot;
	static uint8_t slot_buf[4096];
	static const char req[] =
		"{\"jsonrpc\":\"2.0\",\"id\":7,\"method\":\"hello\"}";

	slot.buf = slot_buf;
	slot.cap = sizeof(slot_buf);
	TEST_ASSERT_EQUAL_INT(0,
			      mp_frame_rx_init(&host, &slot, 1U, host_rx, NULL));
	g_rx_n = 0U;

	call_framed(req);

	/* The reply came back framed on the control channel. */
	TEST_ASSERT_TRUE(g_wire_len > 0U);
	(void)mp_frame_rx_input(&host, g_wire, g_wire_len);
	TEST_ASSERT_EQUAL_UINT(1U, g_rx_n);
	TEST_ASSERT_EQUAL_UINT8(MP_CH_CONTROL, g_rx_ch);

	TEST_ASSERT_TRUE(mp_json_parse(&g_rp, (const char *)g_rx_msg, g_rx_len,
				       g_rtok, TOKS, 0U) > 0);
	TEST_ASSERT_TRUE(mp_json_streq(&g_rp,
				       mp_json_obj_get(&g_rp, 0, "jsonrpc"),
				       "2.0"));
	TEST_ASSERT_TRUE(res_streq("model", "STS1000"));

	/* A frame on a channel core does not own is dropped, not guessed at. */
	{
		static mp_frame_tx_t tx;
		const uint8_t *wire = NULL;
		size_t wire_len = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_frame_encode(&tx, MP_CH_UBX, 0U,
							 (const uint8_t *)"\xB5\x62",
							 2U, &wire, &wire_len));
		TEST_ASSERT_EQUAL_INT(1, mp_input(&g_c, wire, wire_len));
		TEST_ASSERT_EQUAL_UINT32(1U, g_c.rx.sink_errors);
	}
}

static void test_fragmented_request_is_reassembled(void)
{
	static mp_frame_tx_t tx;
	static char big[MP_TX_PAYLOAD_MAX + 600U];
	size_t pad;
	size_t n;

	/* A request padded past one frame with a long ignored member, so the
	 * device has to reassemble before it can parse. */
	n = (size_t)snprintf(big, sizeof(big),
			     "{\"jsonrpc\":\"2.0\",\"id\":9,\"method\":\"hello\","
			     "\"params\":{\"pad\":\"");
	for (pad = 0U; pad < (MP_TX_PAYLOAD_MAX + 200U); pad++) {
		big[n++] = 'x';
	}
	n += (size_t)snprintf(&big[n], sizeof(big) - n, "\"}}");

	g_wire_len = 0U;
	TEST_ASSERT_EQUAL_INT(0, mp_frame_send(&tx, MP_CH_CONTROL,
					       (const uint8_t *)big, n, tx_cb,
					       NULL));
	TEST_ASSERT_TRUE(g_frames >= 2U);

	{
		static uint8_t req_wire[8192];
		size_t req_len = g_wire_len;

		memcpy(req_wire, g_wire, req_len);
		g_wire_len = 0U;
		g_frames = 0U;
		TEST_ASSERT_EQUAL_INT(1, mp_input(&g_c, req_wire, req_len));
	}
	TEST_ASSERT_TRUE(g_wire_len > 0U);
}

/* --------------------------------------------------------------- reply size */

static void test_a_reply_that_cannot_fit_is_an_error_not_a_truncation(void)
{
	/*
	 * Every reply must be valid JSON. When one cannot fit MP_REPLY_MAX the
	 * handler answers with an internal error rather than a truncated
	 * document, which a host would fail to parse with no idea why.
	 */
	char req[600];
	size_t i;
	size_t n;

	/* A very long string id inflates the envelope past the buffer. */
	n = (size_t)snprintf(req, sizeof(req),
			     "{\"jsonrpc\":\"2.0\",\"method\":\"hello\",\"id\":\"");
	for (i = 0U; i < 500U; i++) {
		req[n++] = 'A';
	}
	n += (size_t)snprintf(&req[n], sizeof(req) - n, "\"}");
	req[n] = '\0';

	(void)call(req);
	/* Either it fitted, or it is a clean error — never a broken document. */
	if (mp_json_obj_get(&g_rp, 0, "result") < 0) {
		TEST_ASSERT_EQUAL_INT64(MP_E_INTERNAL, err_code());
	}
}


/* ==================================================== diag runner (direct) */

/*
 * The runner is driven directly here rather than only through `diag.run`,
 * because the interesting paths are the ones a request cannot reach: a step that
 * never finishes, a callback that returns nonsense, an early completion, and the
 * verdict-ranking that decides what a mixed run reports.
 */

static uint8_t g_dstep_verdict;
static int g_dstep_rc;
static bool g_dstep_done;
static unsigned int g_dstep_calls;
static bool g_dstep_fail_second;

static int diag_ctl_cb(void *user, uint8_t test, uint8_t step,
		       mp_diag_step_res_t *out)
{
	(void)user;
	(void)test;
	g_dstep_calls++;
	if (g_dstep_rc != 0) {
		return g_dstep_rc;
	}
	out->verdict = g_dstep_verdict;
	if (g_dstep_fail_second && (step == 1U)) {
		out->verdict = (uint8_t)MP_DIAG_FAIL;
	}
	out->done = g_dstep_done;
	out->value = (int32_t)step;
	return 0;
}

typedef struct {
	uint8_t ev;
	uint8_t test;
	uint8_t step;
	uint8_t verdict;
	int32_t value;
	char text[MP_DIAG_TEXT_MAX];
} devt_t;

static devt_t g_devt[64];
static unsigned int g_devt_n;

static void diag_ctl_evt(void *user, uint8_t ev, uint8_t test, uint8_t step,
			 uint8_t verdict, int32_t value, const char *text)
{
	(void)user;
	if (g_devt_n < (sizeof(g_devt) / sizeof(g_devt[0]))) {
		g_devt[g_devt_n].ev = ev;
		g_devt[g_devt_n].test = test;
		g_devt[g_devt_n].step = step;
		g_devt[g_devt_n].verdict = verdict;
		g_devt[g_devt_n].value = value;
		g_devt[g_devt_n].text[0] = '\0';
		if (text != NULL) {
			(void)strncpy(g_devt[g_devt_n].text, text,
				      sizeof(g_devt[0].text) - 1U);
			g_devt[g_devt_n].text[sizeof(g_devt[0].text) - 1U] = '\0';
		}
		g_devt_n++;
	}
}

static mp_diag_ctx_t g_d;

static void diag_setup(void)
{
	g_dstep_verdict = (uint8_t)MP_DIAG_PASS;
	g_dstep_rc = 0;
	g_dstep_done = false;
	g_dstep_calls = 0U;
	g_dstep_fail_second = false;
	g_devt_n = 0U;
	TEST_ASSERT_EQUAL_INT(0, mp_diag_init(&g_d, diag_ctl_cb, NULL,
					      diag_ctl_evt, NULL));
}

/** Drive the run to completion; returns the number of steps taken. */
static unsigned int diag_drive(uint32_t *now)
{
	unsigned int steps = 0U;

	while (mp_diag_busy(&g_d)) {
		int rc = mp_diag_step(&g_d, *now);

		TEST_ASSERT_TRUE(rc >= 0);
		*now += 1U;
		steps++;
		TEST_ASSERT_TRUE(steps < 200U);
	}
	return steps;
}

static unsigned int devt_count(uint8_t ev)
{
	unsigned int i;
	unsigned int n = 0U;

	for (i = 0U; i < g_devt_n; i++) {
		if (g_devt[i].ev == ev) {
			n++;
		}
	}
	return n;
}

static const devt_t *devt_last(uint8_t ev)
{
	unsigned int i;

	for (i = g_devt_n; i > 0U; i--) {
		if (g_devt[i - 1U].ev == ev) {
			return &g_devt[i - 1U];
		}
	}
	return NULL;
}

static void test_diag_registry_lookup(void)
{
	size_t i;

	for (i = 0U; i < MP_DIAG_COUNT; i++) {
		const mp_diag_test_t *t = mp_diag_test((uint8_t)i);

		TEST_ASSERT_NOT_NULL(t);
		TEST_ASSERT_NOT_NULL(t->name);
		TEST_ASSERT_TRUE(strlen(t->name) > 3U);
		TEST_ASSERT_NOT_NULL(t->desc);
		TEST_ASSERT_TRUE(t->guard < (uint8_t)MP_GUARD_COUNT);
		TEST_ASSERT_TRUE(t->steps > 0U);
		TEST_ASSERT_TRUE(t->step_timeout_ms > 0U);
		TEST_ASSERT_EQUAL_UINT32(0U, t->ilk & ~MP_ILK_ALL);
		TEST_ASSERT_EQUAL_INT((int)i, mp_diag_find(t->name));
		/* Names are unique. */
		{
			size_t j;

			for (j = i + 1U; j < MP_DIAG_COUNT; j++) {
				TEST_ASSERT_TRUE_MESSAGE(
					strcmp(t->name,
					       mp_diag_tests[j].name) != 0,
					t->name);
			}
		}
	}
	TEST_ASSERT_NULL(mp_diag_test((uint8_t)MP_DIAG_COUNT));
	TEST_ASSERT_EQUAL_INT(-ENOENT, mp_diag_find("nope"));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_diag_find(NULL));

	/* The one deferred test is the ATECC attestation, and it says so. */
	TEST_ASSERT_TRUE(mp_diag_tests[MP_DIAG_SEC_ATTEST].deferred);
	/* wdt.test is G3 and disruptive — it cold-cycles the board. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_GUARD_G3,
				mp_diag_tests[MP_DIAG_WDT_TEST].guard);
	TEST_ASSERT_TRUE(mp_diag_tests[MP_DIAG_WDT_TEST].disruptive);
}

static void test_diag_names(void)
{
	uint8_t i;

	for (i = 0U; i < (uint8_t)MP_DIAG_VERDICT_COUNT; i++) {
		TEST_ASSERT_TRUE(strlen(mp_diag_verdict_name(i)) > 0U);
		TEST_ASSERT_TRUE(strcmp(mp_diag_verdict_name(i),
					"unknown") != 0);
	}
	TEST_ASSERT_EQUAL_STRING("unknown",
				 mp_diag_verdict_name(MP_DIAG_VERDICT_COUNT));

	for (i = 0U; i < (uint8_t)MP_DIAG_EV_COUNT; i++) {
		TEST_ASSERT_TRUE(strlen(mp_diag_ev_name(i)) > 0U);
		TEST_ASSERT_TRUE(strcmp(mp_diag_ev_name(i), "unknown") != 0);
	}
	TEST_ASSERT_EQUAL_STRING("unknown", mp_diag_ev_name(MP_DIAG_EV_COUNT));
}

static void test_diag_init_and_start_validation(void)
{
	mp_diag_ctx_t c;
	uint32_t run = 0U;

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_diag_init(NULL, diag_ctl_cb, NULL, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_diag_init(&c, NULL, NULL, NULL, NULL));

	diag_setup();
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_diag_start(NULL, 0U, 0U, 0U, &run));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_diag_start(&g_d, (uint8_t)MP_DIAG_COUNT, 0U, 0U,
					    &run));
	TEST_ASSERT_EQUAL_INT(-ENOTSUP,
			      mp_diag_start(&g_d,
					    (uint8_t)MP_DIAG_SEC_ATTEST, 0U, 0U,
					    &run));

	TEST_ASSERT_FALSE(mp_diag_busy(NULL));
	TEST_ASSERT_EQUAL_UINT16(0U, mp_diag_progress(NULL));
	TEST_ASSERT_EQUAL_UINT16(0U, mp_diag_progress(&g_d)); /* idle */
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_diag_step(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(0, mp_diag_step(&g_d, 0U)); /* idle: nothing to do */
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_diag_abort(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-ENOENT, mp_diag_abort(&g_d, 0U));
}

static void test_diag_all_steps_pass(void)
{
	uint32_t now = 1000U;
	uint32_t run = 0U;
	unsigned int steps;

	diag_setup();
	TEST_ASSERT_EQUAL_INT(0, mp_diag_start(&g_d,
					       (uint8_t)MP_DIAG_INA_SELFTEST,
					       7U, now, &run));
	TEST_ASSERT_NOT_EQUAL_UINT32(0U, run);
	TEST_ASSERT_TRUE(mp_diag_busy(&g_d));
	TEST_ASSERT_EQUAL_UINT32(7U, g_d.sid);
	TEST_ASSERT_EQUAL_UINT16(0U, mp_diag_progress(&g_d));

	/* A START event carries the step count. */
	TEST_ASSERT_EQUAL_UINT(1U, devt_count((uint8_t)MP_DIAG_EV_START));
	TEST_ASSERT_EQUAL_INT32(mp_diag_tests[MP_DIAG_INA_SELFTEST].steps,
				devt_last((uint8_t)MP_DIAG_EV_START)->value);

	steps = diag_drive(&now);
	TEST_ASSERT_EQUAL_UINT(mp_diag_tests[MP_DIAG_INA_SELFTEST].steps, steps);
	TEST_ASSERT_EQUAL_UINT(steps,
			       devt_count((uint8_t)MP_DIAG_EV_PROGRESS));
	TEST_ASSERT_EQUAL_UINT(1U, devt_count((uint8_t)MP_DIAG_EV_RESULT));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_DIAG_PASS,
				devt_last((uint8_t)MP_DIAG_EV_RESULT)->verdict);
	TEST_ASSERT_EQUAL_INT32(0, devt_last((uint8_t)MP_DIAG_EV_RESULT)->value);
	TEST_ASSERT_EQUAL_UINT32(1U, g_d.completed);

	/* A second run gets a different id. */
	{
		uint32_t run2 = 0U;

		TEST_ASSERT_EQUAL_INT(0,
				      mp_diag_start(&g_d,
						    (uint8_t)MP_DIAG_I2C_SCAN,
						    0U, now, &run2));
		TEST_ASSERT_NOT_EQUAL_UINT32(run, run2);
		TEST_ASSERT_EQUAL_INT(-EBUSY,
				      mp_diag_start(&g_d,
						    (uint8_t)MP_DIAG_I2C_SCAN,
						    0U, now, NULL));
	}
}

static void test_diag_progress_advances(void)
{
	uint32_t now = 1000U;

	diag_setup();
	TEST_ASSERT_EQUAL_INT(0, mp_diag_start(&g_d,
					       (uint8_t)MP_DIAG_INA_SELFTEST,
					       0U, now, NULL));
	TEST_ASSERT_EQUAL_UINT16(0U, mp_diag_progress(&g_d));
	TEST_ASSERT_EQUAL_INT(1, mp_diag_step(&g_d, now));
	/* 1 of 9 steps done. */
	TEST_ASSERT_EQUAL_UINT16(111U, mp_diag_progress(&g_d));
	TEST_ASSERT_EQUAL_INT(1, mp_diag_step(&g_d, now));
	TEST_ASSERT_EQUAL_UINT16(222U, mp_diag_progress(&g_d));
}

/** A failing step does not abort the run; the worst verdict is reported. */
static void test_diag_failure_does_not_stop_the_sequence(void)
{
	uint32_t now = 1000U;
	unsigned int steps;

	diag_setup();
	g_dstep_fail_second = true;
	TEST_ASSERT_EQUAL_INT(0, mp_diag_start(&g_d,
					       (uint8_t)MP_DIAG_PPS_SELFCHECK,
					       0U, now, NULL));
	steps = diag_drive(&now);

	/* Every step ran, not just up to the failure. */
	TEST_ASSERT_EQUAL_UINT(mp_diag_tests[MP_DIAG_PPS_SELFCHECK].steps,
			       steps);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_DIAG_FAIL,
				devt_last((uint8_t)MP_DIAG_EV_RESULT)->verdict);
	/* The RESULT value is the failed-step count. */
	TEST_ASSERT_EQUAL_INT32(1, devt_last((uint8_t)MP_DIAG_EV_RESULT)->value);
}

/** SKIP is worse than PASS but better than FAIL, and FAIL better than ERROR. */
static void test_diag_verdict_ranking(void)
{
	static const struct {
		int rc;
		uint8_t verdict;
		uint8_t expect;
		/* True where the *runner* synthesises the explanation, rather
		 * than the callback being free to leave it empty. */
		bool runner_text;
	} m[] = {
		{ 0, (uint8_t)MP_DIAG_PASS, (uint8_t)MP_DIAG_PASS, false },
		{ 0, (uint8_t)MP_DIAG_SKIP, (uint8_t)MP_DIAG_SKIP, false },
		{ 0, (uint8_t)MP_DIAG_FAIL, (uint8_t)MP_DIAG_FAIL, false },
		{ 0, (uint8_t)MP_DIAG_ERROR, (uint8_t)MP_DIAG_ERROR, false },
		{ -ENOTSUP, 0U, (uint8_t)MP_DIAG_SKIP, true },
		{ -EIO, 0U, (uint8_t)MP_DIAG_ERROR, true },
		/* A callback that returns success with a nonsense verdict is a
		 * glue bug; it is surfaced, not trusted. */
		{ 0, (uint8_t)MP_DIAG_VERDICT_COUNT, (uint8_t)MP_DIAG_ERROR,
		  true },
		{ 0, 200U, (uint8_t)MP_DIAG_ERROR, true },
	};
	size_t i;

	for (i = 0U; i < (sizeof(m) / sizeof(m[0])); i++) {
		uint32_t now = 1000U;
		char msg[32];

		diag_setup();
		g_dstep_rc = m[i].rc;
		g_dstep_verdict = m[i].verdict;
		TEST_ASSERT_EQUAL_INT(0,
				      mp_diag_start(&g_d,
						    (uint8_t)MP_DIAG_I2C_SCAN,
						    0U, now, NULL));
		(void)diag_drive(&now);

		(void)snprintf(msg, sizeof(msg), "row %u", (unsigned int)i);
		TEST_ASSERT_EQUAL_UINT8_MESSAGE(
			m[i].expect,
			devt_last((uint8_t)MP_DIAG_EV_RESULT)->verdict, msg);
		/* Where the runner classified the step itself, it must say why. */
		if (m[i].runner_text) {
			TEST_ASSERT_TRUE_MESSAGE(
				strlen(devt_last((uint8_t)MP_DIAG_EV_PROGRESS)
					       ->text) > 0U,
				msg);
		}
	}
}

/** A step that never finishes is bounded by the registry's timeout. */
static void test_diag_step_timeout(void)
{
	uint32_t now = 1000U;
	const mp_diag_test_t *t = &mp_diag_tests[MP_DIAG_I2C_SCAN];

	diag_setup();
	g_dstep_rc = -EAGAIN;
	TEST_ASSERT_EQUAL_INT(0, mp_diag_start(&g_d, (uint8_t)MP_DIAG_I2C_SCAN,
					       0U, now, NULL));

	/* Inside the timeout the runner just waits. */
	TEST_ASSERT_EQUAL_INT(0, mp_diag_step(&g_d, now));
	TEST_ASSERT_EQUAL_INT(0, mp_diag_step(&g_d, now + t->step_timeout_ms));
	TEST_ASSERT_TRUE(mp_diag_busy(&g_d));
	TEST_ASSERT_EQUAL_UINT32(0U, g_d.timeouts);

	/* Past it the step is recorded as an error and the run moves on. */
	TEST_ASSERT_EQUAL_INT(2,
			      mp_diag_step(&g_d,
					   now + t->step_timeout_ms + 1U));
	TEST_ASSERT_FALSE(mp_diag_busy(&g_d));
	TEST_ASSERT_EQUAL_UINT32(1U, g_d.timeouts);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MP_DIAG_ERROR,
				devt_last((uint8_t)MP_DIAG_EV_RESULT)->verdict);
	TEST_ASSERT_EQUAL_STRING("step timeout",
				 devt_last((uint8_t)MP_DIAG_EV_PROGRESS)->text);
}

/** A timeout mid-sequence continues to the next step rather than ending. */
static void test_diag_step_timeout_mid_sequence(void)
{
	uint32_t now = 1000U;
	const mp_diag_test_t *t = &mp_diag_tests[MP_DIAG_INA_SELFTEST];

	diag_setup();
	g_dstep_rc = -EAGAIN;
	TEST_ASSERT_EQUAL_INT(0, mp_diag_start(&g_d,
					       (uint8_t)MP_DIAG_INA_SELFTEST,
					       0U, now, NULL));
	TEST_ASSERT_EQUAL_INT(1,
			      mp_diag_step(&g_d,
					   now + t->step_timeout_ms + 1U));
	TEST_ASSERT_TRUE(mp_diag_busy(&g_d));
	TEST_ASSERT_EQUAL_UINT8(1U, g_d.step);
}

/** A step may end the run early by setting `done`. */
static void test_diag_early_completion(void)
{
	uint32_t now = 1000U;

	diag_setup();
	g_dstep_done = true;
	TEST_ASSERT_EQUAL_INT(0, mp_diag_start(&g_d,
					       (uint8_t)MP_DIAG_INA_SELFTEST,
					       0U, now, NULL));
	TEST_ASSERT_EQUAL_INT(2, mp_diag_step(&g_d, now));
	TEST_ASSERT_FALSE(mp_diag_busy(&g_d));
	TEST_ASSERT_EQUAL_UINT(1U, g_dstep_calls);
	TEST_ASSERT_EQUAL_UINT(1U, devt_count((uint8_t)MP_DIAG_EV_RESULT));
}

static void test_diag_abort_mid_run(void)
{
	uint32_t now = 1000U;

	diag_setup();
	TEST_ASSERT_EQUAL_INT(0, mp_diag_start(&g_d,
					       (uint8_t)MP_DIAG_INA_SELFTEST,
					       0U, now, NULL));
	TEST_ASSERT_EQUAL_INT(1, mp_diag_step(&g_d, now));
	TEST_ASSERT_EQUAL_INT(0, mp_diag_abort(&g_d, now));
	TEST_ASSERT_FALSE(mp_diag_busy(&g_d));
	TEST_ASSERT_EQUAL_UINT32(1U, g_d.aborts);
	TEST_ASSERT_EQUAL_UINT(1U, devt_count((uint8_t)MP_DIAG_EV_ABORT));
	TEST_ASSERT_EQUAL_STRING("aborted",
				 devt_last((uint8_t)MP_DIAG_EV_ABORT)->text);
	/* No RESULT: an aborted run has no verdict. */
	TEST_ASSERT_EQUAL_UINT(0U, devt_count((uint8_t)MP_DIAG_EV_RESULT));
	TEST_ASSERT_EQUAL_INT(-ENOENT, mp_diag_abort(&g_d, now));
}

static void test_diag_runs_without_an_event_sink(void)
{
	mp_diag_ctx_t c;
	uint32_t now = 1000U;

	g_dstep_verdict = (uint8_t)MP_DIAG_PASS;
	g_dstep_rc = 0;
	g_dstep_done = false;
	g_dstep_fail_second = false;
	TEST_ASSERT_EQUAL_INT(0, mp_diag_init(&c, diag_ctl_cb, NULL, NULL,
					      NULL));
	TEST_ASSERT_EQUAL_INT(0, mp_diag_start(&c, (uint8_t)MP_DIAG_I2C_SCAN, 0U,
					       now, NULL));
	while (mp_diag_busy(&c)) {
		TEST_ASSERT_TRUE(mp_diag_step(&c, now++) >= 0);
	}
	TEST_ASSERT_EQUAL_UINT32(1U, c.completed);
}

/** The run-id counter must never hand out 0, including across a wrap. */
static void test_diag_run_id_never_zero(void)
{
	uint32_t now = 1000U;
	uint32_t run = 0U;

	diag_setup();
	g_d.next_run_id = 0xFFFFFFFFU;
	TEST_ASSERT_EQUAL_INT(0, mp_diag_start(&g_d, (uint8_t)MP_DIAG_I2C_SCAN,
					       0U, now, &run));
	TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFU, run);
	TEST_ASSERT_EQUAL_UINT32(1U, g_d.next_run_id);
	(void)diag_drive(&now);
	TEST_ASSERT_EQUAL_INT(0, mp_diag_start(&g_d, (uint8_t)MP_DIAG_I2C_SCAN,
					       0U, now, &run));
	TEST_ASSERT_EQUAL_UINT32(1U, run);
}

/* ------------------------------------------------------------------- runner */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_init_validation);
	RUN_TEST(test_error_message_table);

	RUN_TEST(test_mode_entry_magic);
	RUN_TEST(test_mode_exit_magic_and_cleanup);
	RUN_TEST(test_link_loss_leaves_mp_mode);
	RUN_TEST(test_exit_aborts_a_running_diagnostic);

	RUN_TEST(test_parse_and_envelope_errors);
	RUN_TEST(test_id_is_echoed_verbatim);
	RUN_TEST(test_notification_gets_no_reply);

	RUN_TEST(test_hello);
	RUN_TEST(test_manifest_paging);
	RUN_TEST(test_session_methods);

	RUN_TEST(test_a_session_with_no_credential_is_capped_at_g0);
	RUN_TEST(test_a_viewer_cannot_perform_a_g1_action);
	RUN_TEST(test_an_operator_gets_g1_but_not_g2);
	RUN_TEST(test_an_admin_with_the_serial_gets_g2);
	RUN_TEST(test_no_auth_hook_caps_every_session_at_g0);
	RUN_TEST(test_a_refusal_reveals_nothing_about_the_account);
	RUN_TEST(test_a_lockout_is_reported_distinctly);
	RUN_TEST(test_a_failed_open_does_not_disturb_the_live_session);
	RUN_TEST(test_a_takeover_does_not_inherit_the_role);
	RUN_TEST(test_the_secret_never_leaves_the_auth_hook);
	RUN_TEST(test_an_override_is_audited_with_the_user);
	RUN_TEST(test_an_over_long_string_param_is_emptied);

	RUN_TEST(test_obj_get);
	RUN_TEST(test_obj_get_reports_an_override);
	RUN_TEST(test_obj_set_guard_escalation);
	RUN_TEST(test_obj_set_g3_arm_and_complete);
	RUN_TEST(test_obj_set_bad_values);
	RUN_TEST(test_obj_set_persists_a_cfg_backed_object);
	RUN_TEST(test_obj_override_interlock_refusals);
	RUN_TEST(test_obj_override_vcc_rb_is_clamped_to_the_configured_ceiling);
	RUN_TEST(test_obj_override_reports_a_tunnel_as_reference_suspect);
	RUN_TEST(test_obj_override_veto_and_not_overridable);
	RUN_TEST(test_obj_pulse);
	RUN_TEST(test_interlock_state_unavailable_is_reported);

	RUN_TEST(test_stream_sub_unsub);
	RUN_TEST(test_tick_pumps_subscribed_streams);
	RUN_TEST(test_tick_tolerates_absent_providers);
	RUN_TEST(test_stream_raw_tee);

	RUN_TEST(test_diag_list);
	RUN_TEST(test_diag_run);
	RUN_TEST(test_diag_run_guards_and_interlocks);
	RUN_TEST(test_diag_snapshot);

	RUN_TEST(test_cal_list_and_get);
	RUN_TEST(test_cal_set_commit_revert);
	RUN_TEST(test_cfg_export_import_round_trip);
	RUN_TEST(test_cfg_transfer_errors);
	RUN_TEST(test_cfg_methods_without_a_cfg_context);

	RUN_TEST(test_log_fetch);

	RUN_TEST(test_sys_reboot_is_g3_and_deferred);
	RUN_TEST(test_sys_reboot_arm_is_bound_to_the_mode);
	RUN_TEST(test_sys_bootloader);
	RUN_TEST(test_sys_methods_without_an_image_port);
	RUN_TEST(test_sys_mode_returns_to_the_shell);
	RUN_TEST(test_sys_status);
	RUN_TEST(test_time_get);
	RUN_TEST(test_mirror_get);

	RUN_TEST(test_veto_and_event_feed);
	RUN_TEST(test_end_to_end_over_frames);
	RUN_TEST(test_fragmented_request_is_reassembled);
	RUN_TEST(test_a_reply_that_cannot_fit_is_an_error_not_a_truncation);

	RUN_TEST(test_diag_registry_lookup);
	RUN_TEST(test_diag_names);
	RUN_TEST(test_diag_init_and_start_validation);
	RUN_TEST(test_diag_all_steps_pass);
	RUN_TEST(test_diag_progress_advances);
	RUN_TEST(test_diag_failure_does_not_stop_the_sequence);
	RUN_TEST(test_diag_verdict_ranking);
	RUN_TEST(test_diag_step_timeout);
	RUN_TEST(test_diag_step_timeout_mid_sequence);
	RUN_TEST(test_diag_early_completion);
	RUN_TEST(test_diag_abort_mid_run);
	RUN_TEST(test_diag_runs_without_an_event_sink);
	RUN_TEST(test_diag_run_id_never_zero);

	return UNITY_END();
}
