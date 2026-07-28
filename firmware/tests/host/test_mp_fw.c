/*
 * STS1000 "Meridian" — the six FMT §9 `fw.*` methods, driven against a real
 * core/fwupd through the mp_fwupd_t port.
 *
 * The port is not a seam for convenience. core/mp and core/fwupd are two state
 * machines with two drivers on two threads, and the property that has to hold
 * across the join is core/fwupd's:
 *
 *     restore() runs exactly once for every prepare() that succeeded, on every
 *     exit path (ARCHITECTURE.md §5, `fwupd`).
 *
 * A GNSS receiver left in safeboot, or a rubidium left held in reset, is a
 * board that boots into a permanent timing fault and says nothing useful about
 * why. core/fwupd guarantees the property for its own exits; what it cannot see
 * is the exits that belong to the *control plane* — a tool that walks away
 * mid-flash, a session takeover, a link drop, leaving MP mode. Those all land in
 * mp_rpc.c's fw_abandon(), and a missing call there is invisible until a
 * technician is standing in front of a dead unit. So this suite drives the real
 * orchestrator with real target drivers and keeps a ledger: every test asserts
 * restores == successful prepares, and test_the_restore_ledger_balances_on_every_exit()
 * walks every exit the two modules have between them.
 *
 * The other four things it pins, all of which fail silently:
 *
 *   THE ALLOW-LIST. fwupd_cfg_t::allow permits only FWUPD_COMP_STM32_APP out of
 *   the box (fwupd_glue.c), and that is a commissioning decision, not a
 *   convenience: the STM32 path has MCUboot's signature check and automatic
 *   revert behind it, so the worst case is a boot cycle. The GNSS and Rb paths
 *   reprogram a soldered or cabled peripheral with no signature check and no way
 *   back. Holding a G3 arm must not make a component updatable that the unit was
 *   never commissioned to update — so the refusal is asserted *after* the guard
 *   has been satisfied, which is the only ordering that proves the allow-list is
 *   doing the refusing.
 *
 *   THE GUARD CLASS PER TARGET. FMT §5.2 is not uniform: G2 (typed device
 *   serial) for the STM32 image, G3 (typed phrase + hold) for the peripherals.
 *   The arm is bound to the component, so an arm taken for the GNSS receiver
 *   cannot be spent on a rubidium flash.
 *
 *   THE OFFSET ARITHMETIC. A resumable chunked transfer refuses a gap and a
 *   partial overlap rather than trimming them, because accepting the tail of an
 *   overlap would leave the streaming SHA-256 covering octets the target never
 *   received — and the hash would then verify an image that is not the one on
 *   the device.
 *
 *   THE PAGED INVENTORY. Eleven components, four to a page, so the last page is
 *   partial; and the board is probed once per walk rather than once per page.
 *
 * Every reply is parsed back as JSON and asserted on the wire contract, exactly
 * as test_mp_rpc.c does — nothing reaches into mp_ctx_t or fwupd_ctx_t to
 * shortcut a transition.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "fwupd/fwupd.h"
#include "mp/mp.h"

#include "host_sha256.h"

#define SERIAL "STS1000-000042"

#define ADMIN_USER "root"
#define ADMIN_PW "correct horse"
#define OPER_USER "tech"
#define OPER_PW "operator pw"
#define VIEW_USER "guest"
#define VIEW_PW "viewer pw"

/* The image under test. 1200 octets is three chunks at the MP layer's 512-octet
 * ceiling, so the last one is short — the case an off-by-one in `last` misses. */
#define IMG_LEN 1200U
#define IMG_CAP 4096U

/* ========================================================================= */
/* Fake targets, and the restore ledger                                      */
/* ========================================================================= */

typedef struct {
	uint8_t comp;

	/* Counters. */
	unsigned int query_n;
	unsigned int prepare_n;
	unsigned int prepare_ok_n;
	unsigned int transfer_n;
	unsigned int verify_n;
	unsigned int restore_n;
	bool last_restore_after_failure;

	/* Injectable failures. */
	int query_rc;
	int prepare_rc;
	int verify_rc;
	int restore_rc;
	int transfer_rc;
	unsigned int transfer_fail_at; /* 1-based; 0 = never fail */

	/* 0 = no opinion, so the orchestrator's FWUPD_CHUNK_MAX applies. */
	uint32_t chunk_max_val;

	char version[FWUPD_VER_LEN];
	char version_after[FWUPD_VER_LEN];

	uint32_t written;
	uint8_t image[IMG_CAP];
} fake_t;

static fake_t g_tgt[FWUPD_COMP__COUNT];

/*
 * The ledger. Global rather than per-target because the invariant is global:
 * across every component and every exit, the two numbers are equal.
 */
static unsigned int g_prepares_ok;
static unsigned int g_restores;

static int t_query(void *user, char *out, size_t cap)
{
	fake_t *t = (fake_t *)user;

	t->query_n++;
	if (t->query_rc != 0) {
		return t->query_rc;
	}
	(void)snprintf(out, cap, "%s", t->version);
	return 0;
}

static int t_prepare(void *user, uint32_t image_size)
{
	fake_t *t = (fake_t *)user;

	(void)image_size;
	t->prepare_n++;
	if (t->prepare_rc != 0) {
		return t->prepare_rc;
	}
	t->prepare_ok_n++;
	g_prepares_ok++;
	t->written = 0U;
	return 0;
}

static int t_transfer(void *user, uint32_t off, const uint8_t *data, size_t len,
		      bool last)
{
	fake_t *t = (fake_t *)user;

	(void)last;
	t->transfer_n++;
	if ((t->transfer_fail_at != 0U) &&
	    (t->transfer_n == t->transfer_fail_at)) {
		return (t->transfer_rc != 0) ? t->transfer_rc : -EIO;
	}
	TEST_ASSERT_TRUE(((size_t)off + len) <= IMG_CAP);
	(void)memcpy(&t->image[off], data, len);
	t->written = off + (uint32_t)len;
	return 0;
}

static int t_verify(void *user, char *out, size_t cap)
{
	fake_t *t = (fake_t *)user;

	t->verify_n++;
	if (t->verify_rc != 0) {
		return t->verify_rc;
	}
	(void)snprintf(out, cap, "%s", t->version_after);
	return 0;
}

/**
 * The callback the whole suite exists for.
 *
 * Counted globally as well as per-target: fwupd's contract is that this runs
 * once per successful prepare() on every exit path, and the failure mode that
 * matters — a control-plane exit that forgets to abort — shows up as the two
 * totals drifting apart, not as anything the reply says.
 */
static int t_restore(void *user, bool after_failure)
{
	fake_t *t = (fake_t *)user;

	t->restore_n++;
	t->last_restore_after_failure = after_failure;
	g_restores++;
	return t->restore_rc;
}

static uint32_t t_chunk_max(void *user)
{
	return ((fake_t *)user)->chunk_max_val;
}

/** Everything an updatable component needs. */
static fwupd_target_ops_t ops_full(fake_t *t)
{
	fwupd_target_ops_t o;

	(void)memset(&o, 0, sizeof(o));
	o.query_version = t_query;
	o.prepare = t_prepare;
	o.transfer = t_transfer;
	o.verify = t_verify;
	o.restore = t_restore;
	o.poll = NULL; /* synchronous, like the MCUboot slot */
	o.chunk_max = t_chunk_max;
	o.user = t;
	return o;
}

/** A read-only part: a real identity read and no programming path. */
static fwupd_target_ops_t ops_readonly(fake_t *t)
{
	fwupd_target_ops_t o;

	(void)memset(&o, 0, sizeof(o));
	o.query_version = t_query;
	o.user = t;
	return o;
}

/* ========================================================================= */
/* The mp_fwupd_t port — the same shape as fwupd_glue.c's                    */
/* ========================================================================= */

static fwupd_ctx_t g_fw;
static fwupd_inv_row_t g_inv[FWUPD_COMP__COUNT];
static size_t g_inv_n;
static uint32_t g_now;

/**
 * Paged inventory with the snapshot semantics the port documents: the board is
 * probed when @p first is 0, later pages are served from that snapshot.
 */
static int mpfw_inventory(void *ctx, size_t first, fwupd_inv_row_t *out,
			  size_t max, size_t *n, size_t *total)
{
	size_t avail;
	size_t take;

	(void)ctx;
	if ((out == NULL) || (n == NULL) || (total == NULL)) {
		return -EINVAL;
	}
	*n = 0U;
	*total = (size_t)FWUPD_COMP__COUNT;

	if ((first == 0U) || (g_inv_n == 0U)) {
		size_t got = 0U;
		int rc = fwupd_inventory(&g_fw, g_inv,
					 sizeof(g_inv) / sizeof(g_inv[0]), &got);

		if (rc != 0) {
			g_inv_n = 0U;
			return rc;
		}
		g_inv_n = got;
	}

	avail = (first < g_inv_n) ? (g_inv_n - first) : 0U;
	take = (avail < max) ? avail : max;
	if (take != 0U) {
		(void)memcpy(out, &g_inv[first], take * sizeof(out[0]));
	}
	*n = take;
	*total = g_inv_n;
	return 0;
}

static int mpfw_begin(void *ctx, uint8_t comp, const fwupd_req_t *req)
{
	(void)ctx;
	return fwupd_begin(&g_fw, comp, req, (uint64_t)g_now);
}

/*
 * The same shape as fwupd_glue.c's mpfw_data(), including the duplicate
 * decision: `done` is read before the call and compared with `next_off` after
 * it, which is the only test that separates a discarded retransmit from a
 * write. Deriving it any other way here would test a different implementation
 * from the one that ships.
 */
static int mpfw_data(void *ctx, uint32_t off, const uint8_t *d, size_t len,
		     uint32_t *next_off, bool *duplicate)
{
	fwupd_event_t before;
	uint32_t local_next = 0U;
	int rc;

	(void)ctx;

	if (next_off == NULL) {
		next_off = &local_next;
	}
	if (duplicate != NULL) {
		*duplicate = false;
	}

	(void)fwupd_progress(&g_fw, &before);
	rc = fwupd_data(&g_fw, off, d, len, next_off, (uint64_t)g_now);
	if ((rc == 0) && (duplicate != NULL)) {
		*duplicate = (*next_off == before.done);
	}
	return rc;
}

static int mpfw_end(void *ctx)
{
	(void)ctx;
	return fwupd_end(&g_fw, (uint64_t)g_now);
}

static int mpfw_abort(void *ctx)
{
	int rc;

	(void)ctx;
	rc = fwupd_abort(&g_fw, (uint64_t)g_now);
	if (rc == 0) {
		(void)fwupd_reset(&g_fw);
	}
	return rc;
}

static int mpfw_reset(void *ctx)
{
	(void)ctx;
	return fwupd_reset(&g_fw);
}

static int mpfw_status(void *ctx, mp_fw_status_t *out)
{
	uint8_t comp;

	(void)ctx;
	if (out == NULL) {
		return -EINVAL;
	}
	(void)memset(out, 0, sizeof(*out));
	(void)fwupd_progress(&g_fw, &out->progress);
	(void)snprintf(out->before, sizeof(out->before), "%s",
		       fwupd_version_before(&g_fw));
	(void)snprintf(out->after, sizeof(out->after), "%s",
		       fwupd_version_after(&g_fw));
	out->allow = g_fw.cfg.allow;
	out->chunk_max = FWUPD_CHUNK_MAX;

	comp = out->progress.comp;
	if ((comp < (uint8_t)FWUPD_COMP__COUNT) && g_fw.target_set[comp] &&
	    (g_fw.target[comp].chunk_max != NULL)) {
		uint32_t m = g_fw.target[comp].chunk_max(g_fw.target[comp].user);

		if ((m != 0U) && (m < out->chunk_max)) {
			out->chunk_max = m;
		}
	}
	out->chunks_duplicate = g_fw.chunks_duplicate;
	out->chunks_rejected = g_fw.chunks_rejected;
	out->sessions = g_fw.sessions;
	out->sessions_ok = g_fw.sessions_ok;
	out->sessions_failed = g_fw.sessions_failed;
	return 0;
}

static const mp_fwupd_t g_port = {
	.inventory = mpfw_inventory,
	.begin = mpfw_begin,
	.data = mpfw_data,
	.end = mpfw_end,
	.abort = mpfw_abort,
	.reset = mpfw_reset,
	.status = mpfw_status,
	.ctx = NULL,
};

/* ========================================================================= */
/* MP engine wiring                                                          */
/* ========================================================================= */

static mp_ctx_t g_c;
static uint8_t g_scratch[8192];
static mp_reasm_t g_slots[2];
static uint8_t g_slot_buf[2][4096];

static cfg_ctx_t g_cfg;
static logr_t g_log;
static logr_rec_t g_log_slots[16];

static size_t g_wire_len;
static bool g_fwupd_wired = true;

static unsigned int g_confirm_n;
static unsigned int g_revert_n;
static int g_confirm_rc;
static int g_revert_rc;

static uint32_t clock_cb(void *user)
{
	(void)user;
	return g_now;
}

static int tx_cb(void *user, const uint8_t *wire, size_t len)
{
	(void)user;
	(void)wire;
	g_wire_len += len;
	return 0;
}

static int apply_cb(void *user, size_t obj, const int32_t *value)
{
	(void)user;
	(void)obj;
	(void)value;
	return 0;
}

static int obj_read_cb(void *user, size_t obj, mp_val_t *out)
{
	const mp_obj_t *o = mp_obj_at(obj);

	(void)user;
	TEST_ASSERT_NOT_NULL(o);
	(void)memset(out, 0, sizeof(*out));
	out->kind = o->kind;
	out->valid = true;
	return 0;
}

static int auth_cb(void *user, const char *user_name, const char *secret,
		   uint8_t *out_role)
{
	(void)user;
	*out_role = (uint8_t)MP_ROLE_NONE;
	if ((user_name == NULL) || (secret == NULL)) {
		return -EACCES;
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
	return -EACCES;
}

static int img_confirm_cb(void *ctx)
{
	(void)ctx;
	g_confirm_n++;
	return g_confirm_rc;
}

static int img_revert_cb(void *ctx)
{
	(void)ctx;
	g_revert_n++;
	return g_revert_rc;
}

static void reboot_cb(void *ctx, int mode)
{
	(void)ctx;
	(void)mode;
}

static const port_image_t g_img = {
	.confirm_active = img_confirm_cb,
	.request_revert = img_revert_cb,
	.reboot = reboot_cb,
};

/* ------------------------------------------------------------- the image -- */

static uint8_t g_image[IMG_LEN];
static char g_sha_hex[65];

static void hexify(const uint8_t d[32], char out[65])
{
	static const char hx[] = "0123456789abcdef";
	size_t i;

	for (i = 0U; i < 32U; i++) {
		out[i * 2U] = hx[d[i] >> 4];
		out[(i * 2U) + 1U] = hx[d[i] & 0x0FU];
	}
	out[64] = '\0';
}

/* ------------------------------------------------------------------ setup -- */

static void arm_targets(void)
{
	fwupd_target_ops_t o;
	size_t i;

	for (i = 0U; i < (size_t)FWUPD_COMP__COUNT; i++) {
		g_tgt[i].comp = (uint8_t)i;
	}
	(void)snprintf(g_tgt[FWUPD_COMP_STM32_APP].version,
		       sizeof(g_tgt[0].version), "1.2.3");
	(void)snprintf(g_tgt[FWUPD_COMP_STM32_APP].version_after,
		       sizeof(g_tgt[0].version_after), "1.3.0");
	(void)snprintf(g_tgt[FWUPD_COMP_GNSS_ZED_F9T].version,
		       sizeof(g_tgt[0].version), "FWVER=TIM 2.20");
	(void)snprintf(g_tgt[FWUPD_COMP_GNSS_ZED_F9T].version_after,
		       sizeof(g_tgt[0].version_after), "FWVER=TIM 2.30");
	(void)snprintf(g_tgt[FWUPD_COMP_RB_FE5680A].version,
		       sizeof(g_tgt[0].version), "FE-5680A");
	(void)snprintf(g_tgt[FWUPD_COMP_RB_FE5680A].version_after,
		       sizeof(g_tgt[0].version_after), "FE-5680A");
	(void)snprintf(g_tgt[FWUPD_COMP_PHY_LAN8742].version,
		       sizeof(g_tgt[0].version), "LAN8742A rev 1");

	o = ops_full(&g_tgt[FWUPD_COMP_STM32_APP]);
	TEST_ASSERT_EQUAL_INT(0, fwupd_set_target(&g_fw,
						  (uint8_t)FWUPD_COMP_STM32_APP,
						  &o));
	o = ops_full(&g_tgt[FWUPD_COMP_GNSS_ZED_F9T]);
	TEST_ASSERT_EQUAL_INT(0,
			      fwupd_set_target(&g_fw,
					       (uint8_t)FWUPD_COMP_GNSS_ZED_F9T,
					       &o));
	o = ops_full(&g_tgt[FWUPD_COMP_RB_FE5680A]);
	TEST_ASSERT_EQUAL_INT(0,
			      fwupd_set_target(&g_fw,
					       (uint8_t)FWUPD_COMP_RB_FE5680A,
					       &o));
	/* One read-only part, so "updatable: no" is a real row rather than a
	 * hypothesis — FMT renders it and fwupd_begin() refuses it. */
	o = ops_readonly(&g_tgt[FWUPD_COMP_PHY_LAN8742]);
	TEST_ASSERT_EQUAL_INT(0,
			      fwupd_set_target(&g_fw,
					       (uint8_t)FWUPD_COMP_PHY_LAN8742,
					       &o));
}

void setUp(void)
{
	fwupd_cfg_t cfg;
	mp_wiring_t w;
	uint8_t digest[32];
	unsigned int i;

	g_now = 10000U;
	g_wire_len = 0U;
	g_confirm_n = 0U;
	g_revert_n = 0U;
	g_confirm_rc = 0;
	g_revert_rc = 0;
	g_prepares_ok = 0U;
	g_restores = 0U;
	g_inv_n = 0U;
	g_fwupd_wired = true;

	(void)memset(g_tgt, 0, sizeof(g_tgt));
	(void)memset(g_inv, 0, sizeof(g_inv));

	for (i = 0U; i < IMG_LEN; i++) {
		g_image[i] = (uint8_t)((i * 7U) + (i >> 3));
	}
	host_sha256(g_image, sizeof(g_image), digest);
	hexify(digest, g_sha_hex);

	/*
	 * The commissioning policy fwupd_glue.c ships: only the STM32
	 * application image. The GNSS and Rb rows are updatable in principle
	 * and refused in practice until an operator enables them.
	 */
	fwupd_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, fwupd_cfg_allow(&cfg,
						 (uint8_t)FWUPD_COMP_STM32_APP,
						 true));
	TEST_ASSERT_EQUAL_INT(0, fwupd_init(&g_fw, &cfg, NULL,
					    host_sha256_stream()));
	arm_targets();

	for (i = 0U; i < 2U; i++) {
		g_slots[i].buf = g_slot_buf[i];
		g_slots[i].cap = sizeof(g_slot_buf[i]);
	}

	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, NULL));
	TEST_ASSERT_EQUAL_INT(0, logr_init(&g_log, g_log_slots, 16U));

	(void)memset(&w, 0, sizeof(w));
	w.tx = tx_cb;
	w.mono_ms = clock_cb;
	w.model = "STS1000";
	w.serial = SERIAL;
	w.fw_version = "1.2.3";
	w.boot_version = "0.9.0";
	w.board_id = "0011223344556677";
	w.apply = apply_cb;
	w.obj_read = obj_read_cb;
	w.auth = auth_cb;
	w.img = &g_img;
	w.fwupd = g_fwupd_wired ? &g_port : NULL;
	w.cfg = &g_cfg;
	w.log = &g_log;
	w.scratch = g_scratch;
	w.scratch_len = sizeof(g_scratch);
	w.reasm = g_slots;
	w.reasm_n = 2U;

	TEST_ASSERT_EQUAL_INT(0, mp_init(&g_c, &w));
	TEST_ASSERT_EQUAL_INT(0, mp_set_link(&g_c, true));
	TEST_ASSERT_EQUAL_INT(0, mp_mode_enter(&g_c));
}

void tearDown(void)
{
}

/* ========================================================================= */
/* RPC helpers                                                               */
/* ========================================================================= */

#define TOKS 1024U
static mp_json_t g_rp;
static mp_json_tok_t g_rtok[TOKS];

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
	TEST_ASSERT_TRUE(mp_json_streq(&g_rp,
				       mp_json_obj_get(&g_rp, 0, "jsonrpc"),
				       "2.0"));
	return len;
}

static int result(void)
{
	int r = mp_json_obj_get(&g_rp, 0, "result");

	if (r < 0) {
		int e = mp_json_obj_get(&g_rp, 0, "error");
		char msg[128] = "no result and no error";

		if (e >= 0) {
			int64_t code = 0;
			char reason[64] = "";
			int d = mp_json_obj_get(&g_rp, e, "data");

			(void)mp_json_i64(&g_rp,
					  mp_json_obj_get(&g_rp, e, "code"),
					  &code);
			if (d >= 0) {
				(void)mp_json_str(&g_rp,
						  mp_json_obj_get(&g_rp, d,
								  "reason"),
						  reason, sizeof(reason));
			}
			(void)snprintf(msg, sizeof(msg),
				       "unexpected error %lld (%s)",
				       (long long)code, reason);
		}
		TEST_FAIL_MESSAGE(msg);
	}
	return r;
}

static int64_t err_code(void)
{
	int e = mp_json_obj_get(&g_rp, 0, "error");
	int64_t code = 0;

	TEST_ASSERT_TRUE_MESSAGE(e >= 0, "expected an error reply");
	TEST_ASSERT_EQUAL_INT(0,
			      mp_json_i64(&g_rp,
					  mp_json_obj_get(&g_rp, e, "code"),
					  &code));
	return code;
}

/** The `data.reason` of the last error reply, or "" when there is none. */
static const char *err_reason(void)
{
	static char reason[80];
	int e = mp_json_obj_get(&g_rp, 0, "error");
	int d;

	reason[0] = '\0';
	TEST_ASSERT_TRUE(e >= 0);
	d = mp_json_obj_get(&g_rp, e, "data");
	if (d >= 0) {
		(void)mp_json_str(&g_rp, mp_json_obj_get(&g_rp, d, "reason"),
				  reason, sizeof(reason));
	}
	return reason;
}

/**
 * An integer member of the last error reply's `data`.
 *
 * -1 when `data` has no such member, so a test can assert its ABSENCE as well
 * as its value — which matters here: the rewind point is carried only by the
 * refusals that leave the transfer resumable, and adding it everywhere would be
 * telling a tool it can rewind into a session that is over.
 */
static int64_t err_data_i(const char *key)
{
	int e = mp_json_obj_get(&g_rp, 0, "error");
	int64_t v = 0;
	int d;
	int t;

	TEST_ASSERT_TRUE(e >= 0);
	d = mp_json_obj_get(&g_rp, e, "data");
	if (d < 0) {
		return -1;
	}
	t = mp_json_obj_get(&g_rp, d, key);
	if (t < 0) {
		return -1;
	}
	TEST_ASSERT_EQUAL_INT(0, mp_json_i64(&g_rp, t, &v));
	return v;
}

/** True when the last error reply's `data.state` equals @p want. */
static bool err_data_streq(const char *key, const char *want)
{
	int e = mp_json_obj_get(&g_rp, 0, "error");
	int d;

	TEST_ASSERT_TRUE(e >= 0);
	d = mp_json_obj_get(&g_rp, e, "data");
	if (d < 0) {
		return false;
	}
	return mp_json_streq(&g_rp, mp_json_obj_get(&g_rp, d, key), want);
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
					   mp_json_obj_get(&g_rp, result(),
							   key),
					   &v));
	return v;
}

static bool res_streq(const char *key, const char *want)
{
	return mp_json_streq(&g_rp, mp_json_obj_get(&g_rp, result(), key),
			     want);
}

/* The `progress` member every fw.* reply carries. */
static int progress(void)
{
	int p = mp_json_obj_get(&g_rp, result(), "progress");

	TEST_ASSERT_TRUE_MESSAGE(p >= 0, "reply carries no progress member");
	return p;
}

static int64_t prog_i(const char *key)
{
	int64_t v = 0;

	TEST_ASSERT_EQUAL_INT(0,
			      mp_json_i64(&g_rp,
					  mp_json_obj_get(&g_rp, progress(),
							  key),
					  &v));
	return v;
}

static bool prog_streq(const char *key, const char *want)
{
	return mp_json_streq(&g_rp, mp_json_obj_get(&g_rp, progress(), key),
			     want);
}

/**
 * A published `hello.limits` value.
 *
 * MP_FW_INV_PAGE and MP_FW_CHUNK_MAX are private to mp_rpc.c, and the reason
 * they are private is that a host has no way to read a C macro: it reads them
 * from `hello`. Taking them from there rather than duplicating the numbers here
 * means these tests exercise the contract a tool actually sees, and an
 * unpublished limit fails as a missing key instead of passing against a
 * hardcoded copy that has drifted.
 */
static int64_t hello_limit(const char *key)
{
	int lim;
	int64_t v = 0;
	char msg[64];

	(void)call("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"hello\"}");
	lim = mp_json_obj_get(&g_rp, result(), "limits");
	TEST_ASSERT_TRUE_MESSAGE(lim >= 0, "hello publishes no limits object");
	(void)snprintf(msg, sizeof(msg), "hello.limits.%s is not published",
		       key);
	TEST_ASSERT_EQUAL_INT_MESSAGE(0,
				      mp_json_i64(&g_rp,
						  mp_json_obj_get(&g_rp, lim,
								  key),
						  &v),
				      msg);
	TEST_ASSERT_TRUE_MESSAGE(v > 0, msg);
	return v;
}

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

static uint32_t session(void)
{
	return session_as(ADMIN_USER, ADMIN_PW);
}

/** `session.keepalive`, for tests that have to advance the clock. */
static void keepalive(uint32_t sid)
{
	char req[192];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":9,"
		       "\"method\":\"session.keepalive\",\"params\":{\"sid\":%u}}",
		       sid);
	(void)call(req);
}

/* --------------------------------------------------------- fw.* shorthands */

/** `fw.inventory` from @p from. */
static void inventory(uint32_t from)
{
	char req[192];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":2,"
		       "\"method\":\"fw.inventory\",\"params\":{\"from\":%u}}",
		       (unsigned int)from);
	(void)call(req);
}

/** The `components[]` entry for @p comp in the current reply, or -1. */
static int comp_row(uint8_t comp)
{
	int arr = mp_json_obj_get(&g_rp, result(), "components");
	uint16_t i;
	int64_t n = res_i("count");

	TEST_ASSERT_TRUE(arr >= 0);
	for (i = 0U; (int64_t)i < n; i++) {
		int row = mp_json_arr_at(&g_rp, arr, i);
		int64_t id = -1;

		TEST_ASSERT_TRUE(row >= 0);
		(void)mp_json_i64(&g_rp, mp_json_obj_get(&g_rp, row, "id"),
				  &id);
		if (id == (int64_t)comp) {
			return row;
		}
	}
	return -1;
}

static bool row_b(int row, const char *key)
{
	bool v = false;

	TEST_ASSERT_EQUAL_INT(0,
			      mp_json_bool(&g_rp,
					   mp_json_obj_get(&g_rp, row, key), &v));
	return v;
}

/** `fw.begin` at G2: a typed device serial and nothing else. */
static void begin_g2(uint32_t sid, int target, uint32_t size)
{
	char req[320];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"fw.begin\","
		       "\"params\":{\"target\":%d,\"size\":%u,"
		       "\"sha256\":\"%s\",\"sid\":%u,\"confirm\":\"%s\"}}",
		       target, (unsigned int)size, g_sha_hex, sid, SERIAL);
	(void)call(req);
}

/**
 * `fw.begin` at G3: serial + phrase to arm, then the nonce after the hold.
 *
 * Leaves the *second* reply parsed, so the caller asserts on the outcome of the
 * completed action rather than on the arming acknowledgement.
 */
static void begin_g3(uint32_t sid, int target, uint32_t size)
{
	char req[384];
	int64_t nonce;

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"fw.begin\","
		       "\"params\":{\"target\":%d,\"size\":%u,"
		       "\"sha256\":\"%s\",\"sid\":%u,\"confirm\":\"%s\","
		       "\"phrase\":\"%s\"}}",
		       target, (unsigned int)size, g_sha_hex, sid, SERIAL,
		       MP_G3_PHRASE_DEFAULT);
	(void)call(req);
	TEST_ASSERT_TRUE_MESSAGE(res_b("armed"), "G3 phase 1 did not arm");
	nonce = res_i("nonce");

	g_now += MP_G3_HOLD_MS;
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"fw.begin\","
		       "\"params\":{\"target\":%d,\"size\":%u,"
		       "\"sha256\":\"%s\",\"sid\":%u,\"confirm\":\"%s\","
		       "\"nonce\":%lld}}",
		       target, (unsigned int)size, g_sha_hex, sid, SERIAL,
		       (long long)nonce);
	(void)call(req);
}

/** `fw.data` carrying @p len octets of the image from @p off. */
static void data_at(uint32_t sid, uint32_t off, const uint8_t *raw, size_t len)
{
	char b64[MP_B64_LEN(1024U) + 8U];
	char req[2048];

	TEST_ASSERT_TRUE(mp_b64_encode(raw, len, b64, sizeof(b64)) >= 0);
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"fw.data\","
		       "\"params\":{\"sid\":%u,\"off\":%u,\"data\":\"%s\"}}",
		       sid, (unsigned int)off, b64);
	(void)call(req);
}

/** The image chunk at @p off, clipped to the end. */
static size_t chunk_len(uint32_t off, size_t want)
{
	size_t left = (size_t)(IMG_LEN - off);

	return (want < left) ? want : left;
}

static void send_chunk(uint32_t sid, uint32_t off, size_t want)
{
	data_at(sid, off, &g_image[off], chunk_len(off, want));
}

/** Stream the whole image in 512-octet chunks. */
static void send_whole_image(uint32_t sid)
{
	uint32_t off = 0U;

	while (off < IMG_LEN) {
		size_t n = chunk_len(off, 512U);

		data_at(sid, off, &g_image[off], n);
		TEST_ASSERT_EQUAL_INT64((int64_t)(off + n), res_i("next_off"));
		off += (uint32_t)n;
	}
}

static void fw_end(uint32_t sid)
{
	char req[192];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":6,\"method\":\"fw.end\","
		       "\"params\":{\"sid\":%u}}",
		       sid);
	(void)call(req);
}

static void fw_revert(uint32_t sid, int target)
{
	char req[192];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":7,\"method\":\"fw.revert\","
		       "\"params\":{\"target\":%d,\"sid\":%u}}",
		       target, sid);
	(void)call(req);
}

static void fw_confirm(uint32_t sid, int target)
{
	char req[224];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":8,"
		       "\"method\":\"fw.confirm\",\"params\":{\"target\":%d,"
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       target, sid, SERIAL);
	(void)call(req);
}

/**
 * THE invariant, checked after every exit in this file.
 *
 * restore() runs exactly once per successful prepare(), so the two totals are
 * equal whenever no session is mid-flight. They drift apart in exactly two ways:
 * an exit that forgot to abort (restore missing — a component left in whatever
 * state PREPARE put it in), or an abort that ran twice (restore doubled).
 */
static void assert_ledger(const char *where)
{
	char msg[160];

	(void)snprintf(msg, sizeof(msg),
		       "%s: %u prepare(s) succeeded but restore() ran %u "
		       "time(s) — it must run exactly once per prepare on "
		       "every exit path",
		       where, g_prepares_ok, g_restores);
	TEST_ASSERT_EQUAL_UINT_MESSAGE(g_prepares_ok, g_restores, msg);
}

/* ========================================================================= */
/* 1. fw.inventory                                                            */
/* ========================================================================= */

/**
 * FMT §5.1: observation is free.
 *
 * The inventory is what the tool renders before anyone logs in. A board whose
 * component list needs a credential is a board whose operator cannot tell what
 * is wrong with it.
 */
static void test_inventory_needs_no_session(void)
{
	inventory(0U);
	TEST_ASSERT_EQUAL_INT64((int64_t)FWUPD_COMP__COUNT, res_i("total"));
	TEST_ASSERT_TRUE(res_i("count") > 0);
}

/**
 * Eleven rows, four to a page, so the last page is partial.
 *
 * The partial page is the interesting one: `next` must still be the absolute
 * index and `done` must be true, or a tool loops forever on the tail.
 */
static void test_inventory_pages_and_the_last_page_is_partial(void)
{
	int64_t page = hello_limit("fw_inv_page");
	int64_t total = (int64_t)FWUPD_COMP__COUNT;
	int64_t from = 0;
	int64_t seen = 0;
	int64_t last = 0;
	unsigned int pages = 0U;
	bool done = false;

	/* The premise: the row count is not a multiple of the page size, so the
	 * last page really is partial. */
	TEST_ASSERT_TRUE_MESSAGE((total % page) != 0,
				 "this test needs a partial last page");

	while (!done) {
		inventory((uint32_t)from);
		TEST_ASSERT_EQUAL_INT64(total, res_i("total"));
		TEST_ASSERT_EQUAL_INT64(from, res_i("from"));
		TEST_ASSERT_TRUE(res_i("count") <= page);
		TEST_ASSERT_EQUAL_INT64(from + res_i("count"), res_i("next"));

		last = res_i("count");
		seen += last;
		done = res_b("done");
		TEST_ASSERT_EQUAL_INT((int)(seen >= total), (int)done);

		from = res_i("next");
		pages++;
		TEST_ASSERT_TRUE_MESSAGE(pages <= (unsigned int)total,
					 "inventory paging did not terminate");
	}

	TEST_ASSERT_EQUAL_INT64(total, seen);
	TEST_ASSERT_EQUAL_UINT((unsigned int)((total + page - 1) / page), pages);
	TEST_ASSERT_EQUAL_INT64_MESSAGE(total % page, last,
					"the last page did not carry the "
					"remainder");
}

/** A `first` at the end lists nothing and says so, rather than erroring. */
static void test_inventory_from_the_end_lists_nothing_and_is_done(void)
{
	inventory((uint32_t)FWUPD_COMP__COUNT);
	TEST_ASSERT_EQUAL_INT64(0, res_i("count"));
	TEST_ASSERT_EQUAL_INT64((int64_t)FWUPD_COMP__COUNT, res_i("next"));
	TEST_ASSERT_TRUE(res_b("done"));
}

/** A `first` past the end is a range error, not an empty page. */
static void test_inventory_past_the_end_is_a_range_error(void)
{
	inventory((uint32_t)FWUPD_COMP__COUNT + 1U);
	TEST_ASSERT_EQUAL_INT64(MP_E_RANGE, err_code());

	inventory(0xFFFFFFFFU);
	TEST_ASSERT_EQUAL_INT64(MP_E_RANGE, err_code());
}

/**
 * Walking the inventory reads each component once, not once per page.
 *
 * The snapshot exists because query_version() is an electrical read on four
 * different buses; re-probing per page would triple the bus traffic and, on the
 * GNSS path, three UBX round trips per page of a list nobody is acting on yet.
 */
static void test_inventory_reads_each_component_once_per_walk(void)
{
	unsigned int after_first;
	unsigned int pages = 1U;
	int64_t from;

	inventory(0U);
	after_first = g_tgt[FWUPD_COMP_STM32_APP].query_n +
		      g_tgt[FWUPD_COMP_GNSS_ZED_F9T].query_n +
		      g_tgt[FWUPD_COMP_RB_FE5680A].query_n +
		      g_tgt[FWUPD_COMP_PHY_LAN8742].query_n;
	/* Four registered drivers, probed once each by the page-0 snapshot. */
	TEST_ASSERT_EQUAL_UINT(4U, after_first);

	from = res_i("next");
	while (!res_b("done")) {
		inventory((uint32_t)from);
		from = res_i("next");
		pages++;
		/* Bounded, so a `done` flag that never latches fails here with
		 * its own name rather than as a 120-second CTest timeout. */
		TEST_ASSERT_TRUE_MESSAGE(pages <= (unsigned int)FWUPD_COMP__COUNT,
					 "inventory paging did not terminate");
	}
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		after_first,
		g_tgt[FWUPD_COMP_STM32_APP].query_n +
			g_tgt[FWUPD_COMP_GNSS_ZED_F9T].query_n +
			g_tgt[FWUPD_COMP_RB_FE5680A].query_n +
			g_tgt[FWUPD_COMP_PHY_LAN8742].query_n,
		"later pages re-probed the board instead of serving the "
		"snapshot page 0 took");

	/* And a fresh walk does re-probe, or the tool can never see a part that
	 * has come back since. */
	inventory(0U);
	TEST_ASSERT_EQUAL_UINT(2U, g_tgt[FWUPD_COMP_STM32_APP].query_n);
}

/**
 * `updatable` is what the board provides; `allowed` is what this unit was
 * commissioned to do. The tool needs both, or it offers a button that always
 * refuses.
 */
static void test_inventory_separates_updatable_from_allowed(void)
{
	int row;

	inventory(0U);

	row = comp_row((uint8_t)FWUPD_COMP_STM32_APP);
	TEST_ASSERT_TRUE(row >= 0);
	TEST_ASSERT_TRUE(row_b(row, "updatable"));
	TEST_ASSERT_TRUE(row_b(row, "allowed"));
	TEST_ASSERT_TRUE(mp_json_streq(&g_rp,
				       mp_json_obj_get(&g_rp, row, "guard"),
				       "G2"));

	row = comp_row((uint8_t)FWUPD_COMP_GNSS_ZED_F9T);
	TEST_ASSERT_TRUE(row >= 0);
	TEST_ASSERT_TRUE_MESSAGE(row_b(row, "updatable"),
				 "the receiver has an update path");
	TEST_ASSERT_FALSE_MESSAGE(row_b(row, "allowed"),
				  "the GNSS receiver must not be permitted out "
				  "of the box");
	TEST_ASSERT_TRUE_MESSAGE(mp_json_streq(&g_rp,
					       mp_json_obj_get(&g_rp, row,
							       "guard"),
					       "G3"),
				 "reprogramming the receiver is G3");

	row = comp_row((uint8_t)FWUPD_COMP_RB_FE5680A);
	TEST_ASSERT_TRUE(row >= 0);
	TEST_ASSERT_FALSE(row_b(row, "allowed"));
	TEST_ASSERT_TRUE(mp_json_streq(&g_rp,
				       mp_json_obj_get(&g_rp, row, "guard"),
				       "G3"));

	/* The read-only part is on the list on purpose: "updatable: no" next to
	 * what CAN be read is what stops the question being asked again. */
	{
		unsigned int pages = 1U;

		inventory(0U);
		while (comp_row((uint8_t)FWUPD_COMP_PHY_LAN8742) < 0) {
			TEST_ASSERT_FALSE_MESSAGE(res_b("done"),
						  "the read-only PHY is not in "
						  "the inventory at all");
			inventory((uint32_t)res_i("next"));
			pages++;
			TEST_ASSERT_TRUE_MESSAGE(
				pages <= (unsigned int)FWUPD_COMP__COUNT,
				"inventory paging did not terminate");
		}
	}
	row = comp_row((uint8_t)FWUPD_COMP_PHY_LAN8742);
	TEST_ASSERT_FALSE(row_b(row, "updatable"));
	TEST_ASSERT_FALSE(row_b(row, "allowed"));
}

/* ========================================================================= */
/* 2. the allow-list                                                          */
/* ========================================================================= */

/**
 * The commissioning refusal, and it happens AFTER the G3 arm is spent.
 *
 * That ordering is the whole point: the arm proves the guard let the request
 * through, so the MP_E_VETO can only be firmware policy refusing. A test that
 * never satisfied the guard would pass just as happily against a build with no
 * allow-list at all.
 */
static void test_the_allow_list_refuses_the_gnss_receiver_by_default(void)
{
	uint32_t sid = session();

	begin_g3(sid, (int)FWUPD_COMP_GNSS_ZED_F9T, IMG_LEN);

	TEST_ASSERT_EQUAL_INT64(MP_E_VETO, err_code());
	TEST_ASSERT_EQUAL_STRING("component not permitted", err_reason());
	/* Nothing was touched: no prepare, so nothing is owed a restore. */
	TEST_ASSERT_EQUAL_UINT(0U, g_tgt[FWUPD_COMP_GNSS_ZED_F9T].prepare_n);
	assert_ledger("allow-list refusal (GNSS)");
}

static void test_the_allow_list_refuses_the_rubidium_by_default(void)
{
	uint32_t sid = session();

	begin_g3(sid, (int)FWUPD_COMP_RB_FE5680A, IMG_LEN);

	TEST_ASSERT_EQUAL_INT64(MP_E_VETO, err_code());
	TEST_ASSERT_EQUAL_STRING("component not permitted", err_reason());
	TEST_ASSERT_EQUAL_UINT(0U, g_tgt[FWUPD_COMP_RB_FE5680A].prepare_n);
	assert_ledger("allow-list refusal (Rb)");
}

/** The one component that is permitted, so the refusals above mean something. */
static void test_the_stm32_image_is_permitted(void)
{
	uint32_t sid = session();

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);

	TEST_ASSERT_EQUAL_INT64((int64_t)FWUPD_COMP_STM32_APP,
				res_i("target"));
	TEST_ASSERT_TRUE(res_streq("guard", "G2"));
	TEST_ASSERT_EQUAL_INT64((int64_t)IMG_LEN, res_i("size"));
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].prepare_n);
	TEST_ASSERT_TRUE(prog_streq("state", "TRANSFER"));
}

/**
 * Enabling a component is what makes it startable — and a read-only part still
 * is not, because the refusal moves from policy to physics.
 */
static void test_a_component_with_no_update_path_says_so(void)
{
	uint32_t sid = session();
	fwupd_cfg_t cfg = g_fw.cfg;

	TEST_ASSERT_EQUAL_INT(0, fwupd_cfg_allow(&cfg,
						 (uint8_t)FWUPD_COMP_PHY_LAN8742,
						 true));
	TEST_ASSERT_EQUAL_INT(0, fwupd_set_cfg(&g_fw, &cfg));

	begin_g3(sid, (int)FWUPD_COMP_PHY_LAN8742, IMG_LEN);

	TEST_ASSERT_EQUAL_INT64(MP_E_NOTSUP, err_code());
	TEST_ASSERT_EQUAL_STRING("no update path", err_reason());
	assert_ledger("read-only component");
}

/** Enabling the receiver makes it startable, which proves the gate is the gate. */
static void test_enabling_a_component_makes_it_startable(void)
{
	uint32_t sid = session();
	fwupd_cfg_t cfg = g_fw.cfg;

	TEST_ASSERT_EQUAL_INT(0,
			      fwupd_cfg_allow(&cfg,
					      (uint8_t)FWUPD_COMP_GNSS_ZED_F9T,
					      true));
	TEST_ASSERT_EQUAL_INT(0, fwupd_set_cfg(&g_fw, &cfg));

	begin_g3(sid, (int)FWUPD_COMP_GNSS_ZED_F9T, IMG_LEN);

	TEST_ASSERT_EQUAL_INT64((int64_t)FWUPD_COMP_GNSS_ZED_F9T,
				res_i("target"));
	TEST_ASSERT_TRUE(res_streq("guard", "G3"));
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_GNSS_ZED_F9T].prepare_n);
}

/* ========================================================================= */
/* 3. guard class and role                                                    */
/* ========================================================================= */

/**
 * The STM32 image is G2 and the peripherals are G3, and the difference is
 * observable: the same request that starts an STM32 transfer only *arms* a GNSS
 * one.
 */
static void test_the_stm32_is_g2_and_a_peripheral_is_g3(void)
{
	uint32_t sid = session();
	char req[320];

	/* Serial alone: enough for the image. */
	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	TEST_ASSERT_TRUE(res_streq("guard", "G2"));

	fw_revert(sid, (int)FWUPD_COMP_STM32_APP);
	TEST_ASSERT_TRUE(res_b("aborted"));

	/* Serial alone against a peripheral: refused for want of the phrase. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"fw.begin\","
		       "\"params\":{\"target\":%d,\"size\":%u,"
		       "\"sha256\":\"%s\",\"sid\":%u,\"confirm\":\"%s\"}}",
		       (int)FWUPD_COMP_GNSS_ZED_F9T, (unsigned int)IMG_LEN,
		       g_sha_hex, sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_GUARD, err_code());
	TEST_ASSERT_EQUAL_UINT(0U, g_tgt[FWUPD_COMP_GNSS_ZED_F9T].prepare_n);
	assert_ledger("G3 refusal");
}

/**
 * The G3 arm is bound to the component.
 *
 * An arm taken for the GNSS receiver must not complete as a rubidium flash: the
 * two are different irreversible acts on different ICs, and the operator typed
 * the phrase for one of them.
 */
static void test_a_g3_arm_is_bound_to_its_component(void)
{
	uint32_t sid = session();
	char req[384];
	int64_t nonce;
	fwupd_cfg_t cfg = g_fw.cfg;

	/* Permit both, so only the arm binding can refuse. */
	TEST_ASSERT_EQUAL_INT(0,
			      fwupd_cfg_allow(&cfg,
					      (uint8_t)FWUPD_COMP_GNSS_ZED_F9T,
					      true));
	TEST_ASSERT_EQUAL_INT(0,
			      fwupd_cfg_allow(&cfg,
					      (uint8_t)FWUPD_COMP_RB_FE5680A,
					      true));
	TEST_ASSERT_EQUAL_INT(0, fwupd_set_cfg(&g_fw, &cfg));

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"fw.begin\","
		       "\"params\":{\"target\":%d,\"size\":%u,"
		       "\"sha256\":\"%s\",\"sid\":%u,\"confirm\":\"%s\","
		       "\"phrase\":\"%s\"}}",
		       (int)FWUPD_COMP_GNSS_ZED_F9T, (unsigned int)IMG_LEN,
		       g_sha_hex, sid, SERIAL, MP_G3_PHRASE_DEFAULT);
	(void)call(req);
	TEST_ASSERT_TRUE(res_b("armed"));
	nonce = res_i("nonce");
	g_now += MP_G3_HOLD_MS;

	/* Spend it on the rubidium instead. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"fw.begin\","
		       "\"params\":{\"target\":%d,\"size\":%u,"
		       "\"sha256\":\"%s\",\"sid\":%u,\"confirm\":\"%s\","
		       "\"nonce\":%lld}}",
		       (int)FWUPD_COMP_RB_FE5680A, (unsigned int)IMG_LEN,
		       g_sha_hex, sid, SERIAL, (long long)nonce);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_GUARD, err_code());
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, g_tgt[FWUPD_COMP_RB_FE5680A].prepare_n,
		"an arm taken for the GNSS receiver reprogrammed the rubidium");
	assert_ledger("cross-component arm");
}

/** The hold is real: completing before it elapses is MP_E_HOLD. */
static void test_the_g3_hold_must_elapse(void)
{
	uint32_t sid = session();
	char req[384];
	int64_t nonce;
	fwupd_cfg_t cfg = g_fw.cfg;

	TEST_ASSERT_EQUAL_INT(0,
			      fwupd_cfg_allow(&cfg,
					      (uint8_t)FWUPD_COMP_GNSS_ZED_F9T,
					      true));
	TEST_ASSERT_EQUAL_INT(0, fwupd_set_cfg(&g_fw, &cfg));

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"fw.begin\","
		       "\"params\":{\"target\":%d,\"size\":%u,"
		       "\"sha256\":\"%s\",\"sid\":%u,\"confirm\":\"%s\","
		       "\"phrase\":\"%s\"}}",
		       (int)FWUPD_COMP_GNSS_ZED_F9T, (unsigned int)IMG_LEN,
		       g_sha_hex, sid, SERIAL, MP_G3_PHRASE_DEFAULT);
	(void)call(req);
	nonce = res_i("nonce");
	TEST_ASSERT_EQUAL_INT64((int64_t)MP_G3_HOLD_MS, res_i("hold_ms"));

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":4,\"method\":\"fw.begin\","
		       "\"params\":{\"target\":%d,\"size\":%u,"
		       "\"sha256\":\"%s\",\"sid\":%u,\"confirm\":\"%s\","
		       "\"nonce\":%lld}}",
		       (int)FWUPD_COMP_GNSS_ZED_F9T, (unsigned int)IMG_LEN,
		       g_sha_hex, sid, SERIAL, (long long)nonce);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_HOLD, err_code());
	TEST_ASSERT_EQUAL_UINT(0U, g_tgt[FWUPD_COMP_GNSS_ZED_F9T].prepare_n);

	g_now += MP_G3_HOLD_MS;
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64((int64_t)FWUPD_COMP_GNSS_ZED_F9T,
				res_i("target"));
}

/** G2 and G3 both floor at admin; no session at all is a different refusal. */
static void test_fw_begin_needs_an_admin_session(void)
{
	char req[320];
	uint32_t sid;

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"fw.begin\","
		       "\"params\":{\"target\":0,\"size\":%u,"
		       "\"sha256\":\"%s\",\"confirm\":\"%s\"}}",
		       (unsigned int)IMG_LEN, g_sha_hex, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_NO_SESSION, err_code());

	sid = session_as(VIEW_USER, VIEW_PW);
	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	TEST_ASSERT_EQUAL_INT64(MP_E_ROLE, err_code());

	sid = session_as(OPER_USER, OPER_PW);
	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	TEST_ASSERT_EQUAL_INT64_MESSAGE(MP_E_ROLE, err_code(),
					"G2 floors at admin, not operator");

	TEST_ASSERT_EQUAL_UINT(0U, g_tgt[FWUPD_COMP_STM32_APP].prepare_n);
	assert_ledger("role refusals");
}

/**
 * `fw.revert` is the cheapest of the six to reach, on purpose.
 *
 * An operator who can see a transfer going wrong must be able to end it without
 * hunting for the device serial, and every outcome is the safe direction: abort
 * runs RESTORE, and unstaging leaves the running image alone.
 */
static void test_fw_revert_is_g1_and_an_operator_can_reach_it(void)
{
	uint32_t admin = session();
	uint32_t oper;

	begin_g2(admin, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(admin, 0U, 512U);
	TEST_ASSERT_EQUAL_UINT(1U, g_prepares_ok);

	/*
	 * A takeover abandons the transfer it inherited (§5.3) — which is
	 * itself an exit, and the ledger must already balance before the
	 * operator's stop button is even pressed.
	 */
	oper = session_as(OPER_USER, OPER_PW);
	assert_ledger("session takeover mid-transfer");

	fw_revert(oper, (int)FWUPD_COMP_STM32_APP);
	TEST_ASSERT_FALSE_MESSAGE(res_b("aborted"),
				  "the takeover already aborted it");
	TEST_ASSERT_TRUE(res_b("unstaged"));
	TEST_ASSERT_EQUAL_UINT(1U, g_revert_n);
	assert_ledger("operator revert");
}

/** `fw.confirm` is G2: a viewer cannot, an operator cannot, an admin can. */
static void test_fw_confirm_is_g2(void)
{
	uint32_t sid = session_as(OPER_USER, OPER_PW);

	fw_confirm(sid, (int)FWUPD_COMP_STM32_APP);
	TEST_ASSERT_EQUAL_INT64(MP_E_ROLE, err_code());
	TEST_ASSERT_EQUAL_UINT(0U, g_confirm_n);

	sid = session();
	fw_confirm(sid, (int)FWUPD_COMP_STM32_APP);
	TEST_ASSERT_TRUE(res_b("confirmed"));
	TEST_ASSERT_TRUE_MESSAGE(res_b("image_confirmed"),
				 "confirming the STM32 image must reach the "
				 "MCUboot self-confirm (spec §8.3)");
	TEST_ASSERT_EQUAL_UINT(1U, g_confirm_n);

	/* A peripheral has no MCUboot slot: acknowledged, but nothing to
	 * self-confirm. */
	fw_confirm(sid, (int)FWUPD_COMP_GNSS_ZED_F9T);
	TEST_ASSERT_TRUE(res_b("confirmed"));
	TEST_ASSERT_FALSE(res_b("image_confirmed"));
	TEST_ASSERT_EQUAL_UINT(1U, g_confirm_n);
}

/** Every method refuses a target outside the enum, and a missing one. */
static void test_a_bad_target_is_refused_by_every_method(void)
{
	uint32_t sid = session();
	char req[320];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"fw.begin\","
		       "\"params\":{\"target\":%d,\"size\":%u,"
		       "\"sha256\":\"%s\",\"sid\":%u,\"confirm\":\"%s\"}}",
		       (int)FWUPD_COMP__COUNT, (unsigned int)IMG_LEN,
		       g_sha_hex, sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_RANGE, err_code());

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"fw.begin\","
		       "\"params\":{\"size\":%u,\"sha256\":\"%s\",\"sid\":%u,"
		       "\"confirm\":\"%s\"}}",
		       (unsigned int)IMG_LEN, g_sha_hex, sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());

	fw_revert(sid, (int)FWUPD_COMP__COUNT);
	TEST_ASSERT_EQUAL_INT64(MP_E_RANGE, err_code());

	fw_confirm(sid, (int)FWUPD_COMP__COUNT);
	TEST_ASSERT_EQUAL_INT64(MP_E_RANGE, err_code());
}

/** The request envelope: a size and a 64-hex-digit digest, both checked. */
static void test_fw_begin_validates_its_request(void)
{
	uint32_t sid = session();
	char req[320];

	/* Zero size. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"fw.begin\","
		       "\"params\":{\"target\":0,\"size\":0,\"sha256\":\"%s\","
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       g_sha_hex, sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_RANGE, err_code());

	/* Over FWUPD_IMAGE_MAX. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"fw.begin\","
		       "\"params\":{\"target\":0,\"size\":%u,\"sha256\":\"%s\","
		       "\"sid\":%u,\"confirm\":\"%s\"}}",
		       (unsigned int)FWUPD_IMAGE_MAX + 1U, g_sha_hex, sid,
		       SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_RANGE, err_code());

	/* A digest of the wrong length is a different digest, not a short one. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"fw.begin\","
		       "\"params\":{\"target\":0,\"size\":%u,"
		       "\"sha256\":\"abcdef\",\"sid\":%u,\"confirm\":\"%s\"}}",
		       (unsigned int)IMG_LEN, sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());

	/* No digest at all. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"fw.begin\","
		       "\"params\":{\"target\":0,\"size\":%u,\"sid\":%u,"
		       "\"confirm\":\"%s\"}}",
		       (unsigned int)IMG_LEN, sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());

	TEST_ASSERT_EQUAL_UINT(0U, g_tgt[FWUPD_COMP_STM32_APP].prepare_n);
}

/* ========================================================================= */
/* 4. fw.data — the offset arithmetic                                         */
/* ========================================================================= */

static void test_fw_data_walks_the_image_in_order(void)
{
	uint32_t sid = session();

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_whole_image(sid);

	TEST_ASSERT_EQUAL_INT64((int64_t)IMG_LEN, res_i("next_off"));
	TEST_ASSERT_FALSE(res_b("duplicate"));
	TEST_ASSERT_EQUAL_INT64((int64_t)IMG_LEN, prog_i("done"));
	TEST_ASSERT_EQUAL_INT64((int64_t)IMG_LEN, prog_i("total"));
	TEST_ASSERT_EQUAL_INT64(1000, prog_i("permille"));
	TEST_ASSERT_EQUAL_UINT32(IMG_LEN, g_tgt[FWUPD_COMP_STM32_APP].written);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_image,
				     g_tgt[FWUPD_COMP_STM32_APP].image,
				     IMG_LEN);
	/* 1200 in 512s is three chunks, the last one short. */
	TEST_ASSERT_EQUAL_UINT(3U, g_tgt[FWUPD_COMP_STM32_APP].transfer_n);
}

/**
 * A retransmit wholly inside what is already written is accepted and advances
 * nothing — a tool that did not see the acknowledgement simply resends.
 */
static void test_a_duplicate_chunk_advances_nothing(void)
{
	uint32_t sid = session();

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);
	TEST_ASSERT_EQUAL_INT64(512, res_i("next_off"));
	TEST_ASSERT_FALSE(res_b("duplicate"));

	/*
	 * The same chunk again — the commonest retransmit there is, and the one
	 * `next != off + n` could never see: it ends exactly where `done`
	 * already was, so the arithmetic is identical to a fresh write's. The
	 * flag now comes from the port, which read `done` before the call inside
	 * the same lock, so it reports the truth.
	 */
	send_chunk(sid, 0U, 512U);
	TEST_ASSERT_EQUAL_INT64(512, res_i("next_off"));
	TEST_ASSERT_TRUE_MESSAGE(
		res_b("duplicate"),
		"a retransmit ending exactly at `done` reported duplicate:false");
	TEST_ASSERT_EQUAL_INT64(512, prog_i("done"));
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, g_tgt[FWUPD_COMP_STM32_APP].transfer_n,
		"a retransmit was written to the target a second time");

	/* A strict subset of what is written: unambiguously a duplicate, and
	 * reported as one. */
	data_at(sid, 100U, &g_image[100], 8U);
	TEST_ASSERT_TRUE(res_b("duplicate"));
	TEST_ASSERT_EQUAL_INT64(512, res_i("next_off"));
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].transfer_n);

	/* And the other side of the flag: the NEXT genuine write must not
	 * inherit it. Without this the whole assertion above is satisfied by a
	 * port that answers `true` unconditionally. */
	send_chunk(sid, 512U, 512U);
	TEST_ASSERT_EQUAL_INT64(1024, res_i("next_off"));
	TEST_ASSERT_FALSE_MESSAGE(res_b("duplicate"),
				  "a fresh write was reported as a duplicate");
	TEST_ASSERT_EQUAL_UINT(2U, g_tgt[FWUPD_COMP_STM32_APP].transfer_n);

	/* The transfer is untouched and still finishes — which also proves the
	 * streaming hash did not absorb the retransmitted octets. */
	send_chunk(sid, 1024U, 512U);
	fw_end(sid);
	TEST_ASSERT_TRUE_MESSAGE(res_b("ok"),
				 "a duplicate chunk was hashed twice");
	assert_ledger("duplicate chunks");
}

/**
 * A gap is refused, and the transfer stays open at the true offset.
 *
 * Resumability is the whole point of the refusal: the orchestrator will not
 * trim, so the tool has to rewind — and it must still be able to.
 */
static void test_a_gap_is_refused_and_the_transfer_survives_it(void)
{
	uint32_t sid = session();

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);

	/* One octet past where the orchestrator is. */
	data_at(sid, 513U, &g_image[513], 64U);
	TEST_ASSERT_EQUAL_INT64(MP_E_STATE, err_code());
	TEST_ASSERT_EQUAL_STRING("offset", err_reason());
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].transfer_n);

	/*
	 * The refusal itself has to say where to rewind to. Without it the tool
	 * learns only that the offset was wrong, and mp_rpc.c's comment claiming
	 * the reply "still carries next_off" described something mp_fail() never
	 * emitted.
	 */
	TEST_ASSERT_EQUAL_INT64_MESSAGE(
		512, err_data_i("next_off"),
		"a refused chunk did not carry the rewind point");
	TEST_ASSERT_TRUE_MESSAGE(
		err_data_streq("state", "TRANSFER"),
		"a refused chunk did not say the session was still open");

	/* Rewinding to the real offset works: the session was not closed. */
	send_chunk(sid, 512U, 512U);
	TEST_ASSERT_EQUAL_INT64(1024, res_i("next_off"));
	send_chunk(sid, 1024U, 512U);
	fw_end(sid);
	TEST_ASSERT_TRUE(res_b("ok"));
	assert_ledger("gap then rewind");
}

/**
 * A partial overlap is refused rather than trimmed.
 *
 * Writing only the tail would leave the streaming SHA-256 covering octets the
 * target never received, and fw.end would then be verifying a different image
 * from the one on the device — which is the one failure mode a hash check exists
 * to make impossible.
 */
static void test_a_partial_overlap_is_refused_rather_than_trimmed(void)
{
	uint32_t sid = session();

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);

	/* Starts inside what is written, ends past it. */
	data_at(sid, 256U, &g_image[256], 512U);
	TEST_ASSERT_EQUAL_INT64(MP_E_STATE, err_code());
	TEST_ASSERT_EQUAL_STRING("offset", err_reason());
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, g_tgt[FWUPD_COMP_STM32_APP].transfer_n,
		"the overlapping tail was written instead of refused");

	send_chunk(sid, 512U, 512U);
	send_chunk(sid, 1024U, 512U);
	fw_end(sid);
	TEST_ASSERT_TRUE(res_b("ok"));
	assert_ledger("partial overlap");
}

/** A chunk that would run past the declared size is refused. */
static void test_a_chunk_past_the_end_is_refused(void)
{
	uint32_t sid = session();

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);
	send_chunk(sid, 512U, 512U);

	/* 176 octets remain; offer 512 of the padding buffer. */
	{
		uint8_t big[512];

		(void)memset(big, 0x5A, sizeof(big));
		data_at(sid, 1024U, big, sizeof(big));
	}
	TEST_ASSERT_EQUAL_INT64(MP_E_BUSY, err_code());
	TEST_ASSERT_EQUAL_STRING("chunk", err_reason());
	TEST_ASSERT_EQUAL_UINT(2U, g_tgt[FWUPD_COMP_STM32_APP].transfer_n);

	/* Still open: the correct tail completes it. */
	send_chunk(sid, 1024U, 512U);
	fw_end(sid);
	TEST_ASSERT_TRUE(res_b("ok"));
	assert_ledger("chunk past the end");
}

/** An empty chunk is a bad request, and does not disturb the transfer. */
static void test_a_zero_length_chunk_is_refused(void)
{
	uint32_t sid = session();
	char req[192];

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"fw.data\","
		       "\"params\":{\"sid\":%u,\"off\":512,\"data\":\"\"}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
	TEST_ASSERT_EQUAL_STRING("empty chunk", err_reason());

	/* No `data` member at all is the same class of error. */
	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"fw.data\","
		       "\"params\":{\"sid\":%u,\"off\":512}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());

	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].transfer_n);
	send_chunk(sid, 512U, 512U);
	send_chunk(sid, 1024U, 512U);
	fw_end(sid);
	TEST_ASSERT_TRUE(res_b("ok"));
}

/** A missing or malformed `off` is refused before anything is written. */
static void test_fw_data_validates_its_offset(void)
{
	uint32_t sid = session();
	char req[256];

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"fw.data\","
		       "\"params\":{\"sid\":%u,\"data\":\"AAAA\"}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
	TEST_ASSERT_EQUAL_STRING("off", err_reason());

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":5,\"method\":\"fw.data\","
		       "\"params\":{\"sid\":%u,\"off\":-1,"
		       "\"data\":\"AAAA\"}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(MP_E_RANGE, err_code());

	TEST_ASSERT_EQUAL_UINT(0U, g_tgt[FWUPD_COMP_STM32_APP].transfer_n);
}

/**
 * The effective chunk ceiling is the smaller of the two the device publishes.
 *
 * `hello.limits.fw_chunk` (MP_FW_CHUNK_MAX) bounds what one `fw.data` request
 * can physically carry — it is the size of the handler's two stack buffers —
 * while `fw.begin`'s `chunk_max` reports the orchestrator's and the target's.
 * They are different numbers on purpose, and a tool that honours only the larger
 * one over-sends. Both edges are pinned here.
 */
static void test_the_mp_layers_own_chunk_ceiling(void)
{
	size_t fw_chunk = (size_t)hello_limit("fw_chunk");
	uint32_t sid = session();
	uint8_t big[1024];

	TEST_ASSERT_TRUE_MESSAGE(fw_chunk < (uint32_t)FWUPD_CHUNK_MAX,
				 "the transport bound is meant to be the "
				 "tighter of the two");
	TEST_ASSERT_TRUE((fw_chunk + 1U) <= sizeof(big));
	TEST_ASSERT_TRUE((fw_chunk * 2U) <= IMG_LEN);
	(void)memcpy(big, g_image, sizeof(big));

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	/* fw.begin reports the orchestrator's ceiling, which is the larger. */
	TEST_ASSERT_EQUAL_INT64((int64_t)FWUPD_CHUNK_MAX, res_i("chunk_max"));

	/* Exactly hello.limits.fw_chunk octets: accepted. */
	data_at(sid, 0U, big, fw_chunk);
	TEST_ASSERT_EQUAL_INT64((int64_t)fw_chunk, res_i("next_off"));

	/* One more than the handler's raw buffer: refused by the MP layer
	 * before core/fwupd is consulted, so nothing is written. */
	data_at(sid, (uint32_t)fw_chunk, big, fw_chunk + 1U);
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].transfer_n);

	/* Still open. */
	send_chunk(sid, (uint32_t)fw_chunk, fw_chunk);
	TEST_ASSERT_EQUAL_INT64((int64_t)(fw_chunk * 2U), res_i("next_off"));

	/* Close it out, so the ledger is asserted with nothing in flight. */
	fw_revert(sid, (int)FWUPD_COMP_STM32_APP);
	assert_ledger("MP chunk ceiling");
}

/**
 * A target with a tighter ceiling of its own is honoured, and advertised.
 *
 * The ZED-F9T's safeboot loader is the real case: the orchestrator would take
 * 1024 octets and the receiver would not.
 */
static void test_a_targets_own_chunk_ceiling_is_advertised_and_enforced(void)
{
	uint32_t sid = session();

	g_tgt[FWUPD_COMP_STM32_APP].chunk_max_val = 64U;

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	TEST_ASSERT_EQUAL_INT64_MESSAGE(
		64, res_i("chunk_max"),
		"fw.begin advertised a ceiling the target will not accept");

	data_at(sid, 0U, g_image, 64U);
	TEST_ASSERT_EQUAL_INT64(64, res_i("next_off"));

	data_at(sid, 64U, &g_image[64], 65U);
	TEST_ASSERT_EQUAL_INT64(MP_E_BAD_PARAMS, err_code());
	TEST_ASSERT_EQUAL_STRING("chunk", err_reason());
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].transfer_n);

	/* Recoverable: a chunk within the ceiling still lands. */
	data_at(sid, 64U, &g_image[64], 64U);
	TEST_ASSERT_EQUAL_INT64(128, res_i("next_off"));

	fw_revert(sid, (int)FWUPD_COMP_STM32_APP);
	assert_ledger("target chunk ceiling");
}

/** `fw.data` and `fw.end` need a transfer, and refuse legibly without one. */
static void test_fw_data_and_fw_end_need_an_open_transfer(void)
{
	uint32_t sid = session();

	data_at(sid, 0U, g_image, 16U);
	TEST_ASSERT_EQUAL_INT64(MP_E_STATE, err_code());
	TEST_ASSERT_EQUAL_STRING("no transfer", err_reason());

	fw_end(sid);
	TEST_ASSERT_EQUAL_INT64(MP_E_STATE, err_code());
	TEST_ASSERT_EQUAL_STRING("no transfer", err_reason());

	/* And after a completed one, the record is gone again. */
	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_whole_image(sid);
	fw_end(sid);
	TEST_ASSERT_TRUE(res_b("ok"));

	data_at(sid, 0U, g_image, 16U);
	TEST_ASSERT_EQUAL_INT64(MP_E_STATE, err_code());
	fw_end(sid);
	TEST_ASSERT_EQUAL_INT64(MP_E_STATE, err_code());
	assert_ledger("no open transfer");
}

/**
 * A chunk arriving after the transfer-idle timeout drops the ownership record.
 *
 * The reachable sequence, not a contrivance: the operator opens a transfer and
 * stops sending. sts_fwupd_step() on the console supervisor fires core/fwupd's
 * transfer-idle timeout, which runs finish() -> restore() and leaves the session
 * FAILED. The tool then sends its next chunk.
 *
 * fwupd_data() answers -EPERM for that chunk, because the state is no longer
 * TRANSFER — which is precisely "the session you owned has ended". `fw_forget()`
 * used to exclude -EPERM from the codes that end ownership, alongside the three
 * refusals that genuinely leave a transfer open at `next_off`. The consequence
 * is not cosmetic from the tool's side: `fw_sid` kept naming a session that no
 * longer existed, so every subsequent chunk kept answering MP_E_INTERLOCK
 * instead of MP_E_STATE and the tool was never told the transfer was over.
 */
static void test_a_chunk_after_the_idle_timeout_drops_ownership(void)
{
	uint32_t sid = session();

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);
	TEST_ASSERT_EQUAL_INT64(512, res_i("next_off"));

	/*
	 * The operator stops sending CHUNKS while the tool keeps the SESSION
	 * alive, which is the situation the transfer-idle timeout exists for: a
	 * live maintenance session sitting on a half-written component. Hence
	 * the keepalive per step — MP_KEEPALIVE_TTL_MS is far shorter than the
	 * idle budget, and a session that simply went stale would be refused by
	 * the guard before fwupd_data() were ever reached, which would test
	 * nothing.
	 */
	{
		uint32_t waited = 0U;

		while (waited <= g_fw.cfg.transfer_idle_timeout_ms) {
			g_now += MP_KEEPALIVE_TTL_MS / 2U;
			waited += MP_KEEPALIVE_TTL_MS / 2U;
			keepalive(sid);
			TEST_ASSERT_EQUAL_INT64((int64_t)sid, res_i("sid"));
		}
	}

	/* The supervisor's pump — not any RPC — is what ends the transfer. */
	TEST_ASSERT_EQUAL_INT(0, fwupd_step(&g_fw, (uint64_t)g_now));
	TEST_ASSERT_EQUAL_INT((int)FWUPD_ST_FAILED, (int)fwupd_state(&g_fw));
	assert_ledger("transfer-idle timeout");

	/* The late chunk is refused, and the reply says so. */
	send_chunk(sid, 512U, 512U);
	TEST_ASSERT_EQUAL_INT64(MP_E_INTERLOCK, err_code());

	/*
	 * And the ownership went with it. The next attempt is "no transfer" —
	 * the same answer a tool that never began one gets — rather than a
	 * second interlock against a session nobody holds.
	 */
	send_chunk(sid, 512U, 512U);
	TEST_ASSERT_EQUAL_INT64(MP_E_STATE, err_code());
	TEST_ASSERT_EQUAL_STRING("no transfer", err_reason());

	/* fw.end lands on the same answer, so the tool cannot end a session it
	 * has already been told it does not own. */
	fw_end(sid);
	TEST_ASSERT_EQUAL_INT64(MP_E_STATE, err_code());

	/* restore() ran exactly once, from the timeout, and not again. */
	assert_ledger("after the late chunks");
	TEST_ASSERT_EQUAL_UINT(1U, g_restores);
}

/** A wrong `sid` on a chunk is refused as a session, not silently accepted. */
static void test_fw_data_refuses_a_foreign_sid(void)
{
	uint32_t sid = session();

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);

	data_at(sid + 1U, 512U, &g_image[512], 512U);
	TEST_ASSERT_EQUAL_INT64(MP_E_NO_SESSION, err_code());
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].transfer_n);

	/* The real owner is unaffected. */
	send_chunk(sid, 512U, 512U);
	TEST_ASSERT_EQUAL_INT64(1024, res_i("next_off"));
}

/* ========================================================================= */
/* 5. fw.end                                                                  */
/* ========================================================================= */

static void test_fw_end_verifies_and_restores(void)
{
	uint32_t sid = session();

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_whole_image(sid);
	fw_end(sid);

	TEST_ASSERT_TRUE(res_b("ok"));
	TEST_ASSERT_TRUE(prog_streq("state", "DONE"));
	TEST_ASSERT_TRUE(prog_streq("reason", "ok"));
	TEST_ASSERT_EQUAL_INT64(0, prog_i("rc"));
	TEST_ASSERT_TRUE(mp_json_streq(&g_rp,
				       mp_json_obj_get(&g_rp, progress(),
						       "version_before"),
				       "1.2.3"));
	TEST_ASSERT_TRUE(mp_json_streq(&g_rp,
				       mp_json_obj_get(&g_rp, progress(),
						       "version_after"),
				       "1.3.0"));
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].verify_n);
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].restore_n);
	TEST_ASSERT_FALSE(g_tgt[FWUPD_COMP_STM32_APP].last_restore_after_failure);
	assert_ledger("successful end");
}

/** Fewer octets than declared: FWUPD_END_SHORT, and RESTORE has run. */
static void test_a_short_image_fails_at_end(void)
{
	uint32_t sid = session();

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);
	fw_end(sid);

	TEST_ASSERT_EQUAL_INT64(MP_E_INTERNAL, err_code());
	TEST_ASSERT_EQUAL_STRING("short-image", err_reason());
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, g_tgt[FWUPD_COMP_STM32_APP].verify_n,
		"nothing was verified: the image was never complete");
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].restore_n);
	TEST_ASSERT_TRUE(g_tgt[FWUPD_COMP_STM32_APP].last_restore_after_failure);
	assert_ledger("short image");
}

/** The declared digest is checked against the streamed one. */
static void test_a_hash_mismatch_fails_at_end(void)
{
	uint32_t sid = session();
	uint8_t wrong[IMG_LEN];
	uint32_t off = 0U;

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);

	/* Right length, one flipped bit. */
	(void)memcpy(wrong, g_image, sizeof(wrong));
	wrong[IMG_LEN / 2U] ^= 0x01U;
	while (off < IMG_LEN) {
		size_t n = chunk_len(off, 512U);

		data_at(sid, off, &wrong[off], n);
		off += (uint32_t)n;
	}

	fw_end(sid);
	TEST_ASSERT_EQUAL_INT64(MP_E_INTERNAL, err_code());
	TEST_ASSERT_EQUAL_STRING("hash-mismatch", err_reason());
	TEST_ASSERT_EQUAL_UINT(0U, g_tgt[FWUPD_COMP_STM32_APP].verify_n);
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].restore_n);
	assert_ledger("hash mismatch");
}

/**
 * "Bytes went in" and "the update took" are different claims.
 *
 * With `expect` set and not matched the session ends FWUPD_END_VERIFY even
 * though the transfer succeeded — reporting success here would be the single
 * most misleading outcome this path can produce.
 */
static void test_a_version_that_does_not_match_expect_fails(void)
{
	uint32_t sid = session();
	char req[384];

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"fw.begin\","
		       "\"params\":{\"target\":0,\"size\":%u,\"sha256\":\"%s\","
		       "\"expect\":\"9.9.9\",\"sid\":%u,\"confirm\":\"%s\"}}",
		       (unsigned int)IMG_LEN, g_sha_hex, sid, SERIAL);
	(void)call(req);
	TEST_ASSERT_EQUAL_INT64(0, res_i("target"));

	send_whole_image(sid);
	fw_end(sid);

	/*
	 * `fw.end` is not an error here, and that is the contract: fwupd_end()
	 * returns 0 once it has entered VERIFY, because on a target with a
	 * poll() the verdict arrives later. The RPC therefore succeeds and the
	 * verdict is in the reply — `ok` false, and the reason spelled out.
	 * A tool that reads only the JSON-RPC error and not `ok` would call
	 * this update a success, which is why both are pinned.
	 */
	TEST_ASSERT_FALSE_MESSAGE(res_b("ok"),
				  "the component is not running what was asked "
				  "for; this is not a successful update");
	TEST_ASSERT_TRUE(prog_streq("state", "FAILED"));
	TEST_ASSERT_TRUE(prog_streq("reason", "version-not-confirmed"));
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].verify_n);
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].restore_n);
	assert_ledger("version not confirmed");
}

/* ========================================================================= */
/* 6. abort                                                                   */
/* ========================================================================= */

/** The stop button, mid-flash. */
static void test_abort_mid_transfer_runs_restore(void)
{
	uint32_t sid = session();

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);
	TEST_ASSERT_EQUAL_UINT(0U, g_tgt[FWUPD_COMP_STM32_APP].restore_n);

	fw_revert(sid, (int)FWUPD_COMP_STM32_APP);
	TEST_ASSERT_TRUE(res_b("aborted"));
	TEST_ASSERT_FALSE_MESSAGE(res_b("unstaged"),
				  "an aborted transfer staged nothing to "
				  "unstage");
	TEST_ASSERT_EQUAL_UINT(0U, g_revert_n);
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].restore_n);
	TEST_ASSERT_TRUE(g_tgt[FWUPD_COMP_STM32_APP].last_restore_after_failure);
	TEST_ASSERT_TRUE(prog_streq("state", "IDLE"));
	assert_ledger("abort mid-transfer");

	/* And the orchestrator takes a new session afterwards. */
	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	TEST_ASSERT_EQUAL_INT64(0, res_i("target"));
	TEST_ASSERT_EQUAL_UINT(2U, g_tgt[FWUPD_COMP_STM32_APP].prepare_n);
}

/**
 * After a completed transfer there is nothing to abort — the same button
 * unstages the pending MCUboot swap instead.
 */
static void test_revert_after_end_unstages_the_pending_swap(void)
{
	uint32_t sid = session();

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_whole_image(sid);
	fw_end(sid);
	TEST_ASSERT_TRUE(res_b("ok"));

	fw_revert(sid, (int)FWUPD_COMP_STM32_APP);
	TEST_ASSERT_FALSE(res_b("aborted"));
	TEST_ASSERT_TRUE(res_b("unstaged"));
	TEST_ASSERT_EQUAL_UINT(1U, g_revert_n);
	/* One restore, from fw.end — the revert must not run a second. */
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].restore_n);
	TEST_ASSERT_TRUE(prog_streq("state", "IDLE"));
	assert_ledger("revert after end");
}

/** Leaving MP mode must not leave a component in whatever state PREPARE set. */
static void test_leaving_mp_mode_abandons_an_open_transfer(void)
{
	uint32_t sid = session();

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);
	TEST_ASSERT_EQUAL_UINT(0U, g_tgt[FWUPD_COMP_STM32_APP].restore_n);

	TEST_ASSERT_EQUAL_INT(0, mp_mode_exit(&g_c));
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, g_tgt[FWUPD_COMP_STM32_APP].restore_n,
		"a tool that walked away left the component in the state "
		"PREPARE put it in");
	assert_ledger("mp_mode_exit");
}

/** So must closing the session that authorised it. */
static void test_closing_the_session_abandons_an_open_transfer(void)
{
	uint32_t sid = session();
	char req[192];

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);

	(void)snprintf(req, sizeof(req),
		       "{\"jsonrpc\":\"2.0\",\"id\":9,"
		       "\"method\":\"session.close\",\"params\":{\"sid\":%u}}",
		       sid);
	(void)call(req);
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, g_tgt[FWUPD_COMP_STM32_APP].restore_n,
		"the transfer had nobody left to finish it and RESTORE was "
		"owed");
	assert_ledger("session.close");
}

/** And a takeover inherits nothing, least of all a half-written image. */
static void test_a_session_takeover_abandons_an_open_transfer(void)
{
	uint32_t sid = session();

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);

	(void)session_as(OPER_USER, OPER_PW);
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].restore_n);
	assert_ledger("session takeover");
}

/**
 * prepare() failing is the one exit that owes NO restore.
 *
 * The component was never taken away, so returning it would be a second,
 * unbalanced call — and on the GNSS path that means driving safeboot on a
 * receiver that was never put into it.
 */
static void test_a_failed_prepare_owes_no_restore(void)
{
	uint32_t sid = session();

	g_tgt[FWUPD_COMP_STM32_APP].prepare_rc = -EIO;

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	TEST_ASSERT_EQUAL_INT64(MP_E_IO, err_code());
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].prepare_n);
	TEST_ASSERT_EQUAL_UINT(0U, g_tgt[FWUPD_COMP_STM32_APP].prepare_ok_n);
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, g_tgt[FWUPD_COMP_STM32_APP].restore_n,
		"restore() ran for a prepare() that never succeeded");
	assert_ledger("failed prepare");

	/* And the orchestrator is usable again. */
	g_tgt[FWUPD_COMP_STM32_APP].prepare_rc = 0;
	fw_revert(sid, (int)FWUPD_COMP_STM32_APP);
	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	TEST_ASSERT_EQUAL_INT64(0, res_i("target"));
}

/** A target error mid-transfer ends the session there and then. */
static void test_a_target_error_ends_the_session_and_restores(void)
{
	uint32_t sid = session();

	g_tgt[FWUPD_COMP_STM32_APP].transfer_fail_at = 2U;
	g_tgt[FWUPD_COMP_STM32_APP].transfer_rc = -EIO;

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);
	send_chunk(sid, 512U, 512U);

	TEST_ASSERT_EQUAL_INT64(MP_E_IO, err_code());
	TEST_ASSERT_EQUAL_STRING("chunk", err_reason());
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].restore_n);
	assert_ledger("target transfer error");

	/*
	 * Unlike a gap or an over-long chunk, this is not recoverable: the
	 * session is over, so the ownership record went with it and the next
	 * chunk is refused as "no transfer" rather than accepted into a dead
	 * session.
	 */
	send_chunk(sid, 512U, 512U);
	TEST_ASSERT_EQUAL_INT64(MP_E_STATE, err_code());
	TEST_ASSERT_EQUAL_STRING("no transfer", err_reason());
}

/** verify() failing is still a completed prepare, so RESTORE is still owed. */
static void test_a_failed_verify_still_restores(void)
{
	uint32_t sid = session();

	g_tgt[FWUPD_COMP_STM32_APP].verify_rc = -EIO;

	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_whole_image(sid);
	fw_end(sid);

	/* Same shape as the expect-mismatch above: the RPC succeeded, the
	 * update did not. */
	TEST_ASSERT_FALSE(res_b("ok"));
	TEST_ASSERT_TRUE(prog_streq("state", "FAILED"));
	TEST_ASSERT_TRUE(prog_streq("reason", "target-error"));
	TEST_ASSERT_EQUAL_INT64(-EIO, prog_i("rc"));
	TEST_ASSERT_EQUAL_UINT(1U, g_tgt[FWUPD_COMP_STM32_APP].verify_n);
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, g_tgt[FWUPD_COMP_STM32_APP].restore_n,
		"prepare() succeeded, so restore() is owed even though the "
		"component never confirmed the new image");
	assert_ledger("failed verify");
}

/* ========================================================================= */
/* 7. the invariant, across every exit at once                                */
/* ========================================================================= */

/**
 * Every exit the two modules have between them, back to back in one context.
 *
 * Run individually the paths above each prove their own case; run in sequence
 * they also prove the accounting does not drift — a restore that leaked from one
 * session into the next, or an abort made idempotent by forgetting rather than
 * by checking, shows up here and nowhere else.
 */
static void test_the_restore_ledger_balances_on_every_exit(void)
{
	uint32_t sid = session();
	unsigned int expect = 0U;

	/* 1. success */
	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_whole_image(sid);
	fw_end(sid);
	expect++;
	assert_ledger("1 success");
	TEST_ASSERT_EQUAL_UINT(expect, g_restores);

	/* 2. acknowledged, then a short image */
	fw_revert(sid, (int)FWUPD_COMP_STM32_APP);
	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);
	fw_end(sid);
	expect++;
	assert_ledger("2 short image");
	TEST_ASSERT_EQUAL_UINT(expect, g_restores);

	/* 3. abort mid-transfer */
	fw_revert(sid, (int)FWUPD_COMP_STM32_APP);
	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);
	fw_revert(sid, (int)FWUPD_COMP_STM32_APP);
	expect++;
	assert_ledger("3 abort");
	TEST_ASSERT_EQUAL_UINT(expect, g_restores);

	/* 4. a double abort must not restore twice */
	fw_revert(sid, (int)FWUPD_COMP_STM32_APP);
	assert_ledger("4 double abort");
	TEST_ASSERT_EQUAL_UINT(expect, g_restores);

	/* 5. a target error */
	g_tgt[FWUPD_COMP_STM32_APP].transfer_fail_at =
		g_tgt[FWUPD_COMP_STM32_APP].transfer_n + 1U;
	g_tgt[FWUPD_COMP_STM32_APP].transfer_rc = -EIO;
	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);
	expect++;
	assert_ledger("5 target error");
	TEST_ASSERT_EQUAL_UINT(expect, g_restores);
	g_tgt[FWUPD_COMP_STM32_APP].transfer_fail_at = 0U;

	/* 6. a failed prepare, which owes nothing */
	fw_revert(sid, (int)FWUPD_COMP_STM32_APP);
	g_tgt[FWUPD_COMP_STM32_APP].prepare_rc = -EIO;
	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	assert_ledger("6 failed prepare");
	TEST_ASSERT_EQUAL_UINT(expect, g_restores);
	g_tgt[FWUPD_COMP_STM32_APP].prepare_rc = 0;

	/* 7. session takeover mid-transfer */
	fw_revert(sid, (int)FWUPD_COMP_STM32_APP);
	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);
	sid = session();
	expect++;
	assert_ledger("7 takeover");
	TEST_ASSERT_EQUAL_UINT(expect, g_restores);

	/* 8. leaving MP mode mid-transfer */
	begin_g2(sid, (int)FWUPD_COMP_STM32_APP, IMG_LEN);
	send_chunk(sid, 0U, 512U);
	TEST_ASSERT_EQUAL_INT(0, mp_mode_exit(&g_c));
	expect++;
	assert_ledger("8 mode exit");
	TEST_ASSERT_EQUAL_UINT(expect, g_restores);

	/* Every one of the eight was a real session, not a no-op. */
	TEST_ASSERT_EQUAL_UINT(expect, g_prepares_ok);
	TEST_ASSERT_TRUE(expect >= 6U);
}

/* ========================================================================= */
/* 8. the port is optional                                                    */
/* ========================================================================= */

/**
 * A build with no orchestrator answers MP_E_NOTSUP on all six rather than
 * crashing, so a partially wired image is diagnosable instead of dead.
 */
static void test_an_unwired_port_answers_notsup_on_every_method(void)
{
	static const char *const reqs[] = {
		"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"fw.inventory\","
		"\"params\":{\"from\":0}}",
		"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"fw.begin\","
		"\"params\":{\"target\":0,\"size\":16}}",
		"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"fw.data\","
		"\"params\":{\"off\":0,\"data\":\"AAAA\"}}",
		"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"fw.end\","
		"\"params\":{}}",
		"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"fw.confirm\","
		"\"params\":{\"target\":0}}",
		"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"fw.revert\","
		"\"params\":{\"target\":0}}",
	};
	mp_wiring_t w;
	size_t i;

	(void)memset(&w, 0, sizeof(w));
	w.tx = tx_cb;
	w.mono_ms = clock_cb;
	w.model = "STS1000";
	w.serial = SERIAL;
	w.apply = apply_cb;
	w.obj_read = obj_read_cb;
	w.auth = auth_cb;
	w.fwupd = NULL;
	w.scratch = g_scratch;
	w.scratch_len = sizeof(g_scratch);
	w.reasm = g_slots;
	w.reasm_n = 2U;
	TEST_ASSERT_EQUAL_INT(0, mp_init(&g_c, &w));
	TEST_ASSERT_EQUAL_INT(0, mp_set_link(&g_c, true));
	TEST_ASSERT_EQUAL_INT(0, mp_mode_enter(&g_c));

	for (i = 0U; i < (sizeof(reqs) / sizeof(reqs[0])); i++) {
		(void)call(reqs[i]);
		TEST_ASSERT_EQUAL_INT64_MESSAGE(MP_E_NOTSUP, err_code(),
						reqs[i]);
	}
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_inventory_needs_no_session);
	RUN_TEST(test_inventory_pages_and_the_last_page_is_partial);
	RUN_TEST(test_inventory_from_the_end_lists_nothing_and_is_done);
	RUN_TEST(test_inventory_past_the_end_is_a_range_error);
	RUN_TEST(test_inventory_reads_each_component_once_per_walk);
	RUN_TEST(test_inventory_separates_updatable_from_allowed);

	RUN_TEST(test_the_allow_list_refuses_the_gnss_receiver_by_default);
	RUN_TEST(test_the_allow_list_refuses_the_rubidium_by_default);
	RUN_TEST(test_the_stm32_image_is_permitted);
	RUN_TEST(test_a_component_with_no_update_path_says_so);
	RUN_TEST(test_enabling_a_component_makes_it_startable);

	RUN_TEST(test_the_stm32_is_g2_and_a_peripheral_is_g3);
	RUN_TEST(test_a_g3_arm_is_bound_to_its_component);
	RUN_TEST(test_the_g3_hold_must_elapse);
	RUN_TEST(test_fw_begin_needs_an_admin_session);
	RUN_TEST(test_fw_revert_is_g1_and_an_operator_can_reach_it);
	RUN_TEST(test_fw_confirm_is_g2);
	RUN_TEST(test_a_bad_target_is_refused_by_every_method);
	RUN_TEST(test_fw_begin_validates_its_request);

	RUN_TEST(test_fw_data_walks_the_image_in_order);
	RUN_TEST(test_a_duplicate_chunk_advances_nothing);
	RUN_TEST(test_a_gap_is_refused_and_the_transfer_survives_it);
	RUN_TEST(test_a_partial_overlap_is_refused_rather_than_trimmed);
	RUN_TEST(test_a_chunk_past_the_end_is_refused);
	RUN_TEST(test_a_zero_length_chunk_is_refused);
	RUN_TEST(test_fw_data_validates_its_offset);
	RUN_TEST(test_the_mp_layers_own_chunk_ceiling);
	RUN_TEST(test_a_targets_own_chunk_ceiling_is_advertised_and_enforced);
	RUN_TEST(test_fw_data_and_fw_end_need_an_open_transfer);
	RUN_TEST(test_a_chunk_after_the_idle_timeout_drops_ownership);
	RUN_TEST(test_fw_data_refuses_a_foreign_sid);

	RUN_TEST(test_fw_end_verifies_and_restores);
	RUN_TEST(test_a_short_image_fails_at_end);
	RUN_TEST(test_a_hash_mismatch_fails_at_end);
	RUN_TEST(test_a_version_that_does_not_match_expect_fails);

	RUN_TEST(test_abort_mid_transfer_runs_restore);
	RUN_TEST(test_revert_after_end_unstages_the_pending_swap);
	RUN_TEST(test_leaving_mp_mode_abandons_an_open_transfer);
	RUN_TEST(test_closing_the_session_abandons_an_open_transfer);
	RUN_TEST(test_a_session_takeover_abandons_an_open_transfer);
	RUN_TEST(test_a_failed_prepare_owes_no_restore);
	RUN_TEST(test_a_target_error_ends_the_session_and_restores);
	RUN_TEST(test_a_failed_verify_still_restores);

	RUN_TEST(test_the_restore_ledger_balances_on_every_exit);
	RUN_TEST(test_an_unwired_port_answers_notsup_on_every_method);

	return UNITY_END();
}
