/*
 * STS1000 "Meridian" — host unit tests for core/fwupd/fwupd.c.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The property this suite exists to prove is the one that keeps a soldered-down
 * part alive: **restore() runs exactly once for every prepare() that succeeded,
 * on every exit path**. A fake target counts its own callbacks, and every test
 * that ends a session — success, abort, hash mismatch, short image, target error
 * at each step, timeout — asserts the prepare/restore balance and that the
 * degraded flag came back down.
 */

#include <errno.h>
#include <string.h>

#include "fwupd/fwupd.h"
#include "host_sha256.h"
#include "unity.h"

/* ------------------------------------------------------------------ fake -- */

#define IMG_LEN 3000U

typedef struct {
	/* call counts */
	unsigned int n_query;
	unsigned int n_prepare;
	unsigned int n_transfer;
	unsigned int n_verify;
	unsigned int n_restore;
	unsigned int n_poll;
	/* injected failures: 0 = succeed */
	int rc_query;
	int rc_prepare;
	int rc_transfer;
	int rc_verify;
	int rc_restore;
	/* fail transfer only at/after this offset */
	uint32_t transfer_fail_at;
	/* poll returns -EAGAIN this many times before succeeding */
	unsigned int poll_busy;
	int rc_poll;
	bool have_poll;
	uint32_t chunk_limit;
	/* what verify() reports */
	const char *ver_before;
	const char *ver_after;
	bool restore_saw_failure;
	/* image reassembly, to prove the bytes actually arrived in order */
	uint8_t got[IMG_LEN + 64U];
	uint32_t got_len;
} fake_t;

static void fake_reset(fake_t *f)
{
	(void)memset(f, 0, sizeof(*f));
	f->ver_before = "1.00";
	f->ver_after = "2.00";
}

static int f_query(void *u, char *out, size_t cap)
{
	fake_t *f = u;

	f->n_query++;
	if (f->rc_query != 0) {
		return f->rc_query;
	}
	/* Before prepare() the "before" string; afterwards the new one. */
	{
		const char *s = (f->n_prepare == 0U) ? f->ver_before : f->ver_after;
		size_t n = strlen(s);

		if (n >= cap) {
			n = cap - 1U;
		}
		(void)memcpy(out, s, n);
		out[n] = '\0';
	}
	return 0;
}

static int f_prepare(void *u, uint32_t size)
{
	fake_t *f = u;

	(void)size;
	f->n_prepare++;
	return f->rc_prepare;
}

static int f_transfer(void *u, uint32_t off, const uint8_t *d, size_t len,
		      bool last)
{
	fake_t *f = u;

	(void)last;
	f->n_transfer++;
	if ((f->rc_transfer != 0) && (off >= f->transfer_fail_at)) {
		return f->rc_transfer;
	}
	if ((off + len) <= sizeof(f->got)) {
		(void)memcpy(&f->got[off], d, len);
		if ((off + (uint32_t)len) > f->got_len) {
			f->got_len = off + (uint32_t)len;
		}
	}
	return 0;
}

static int f_verify(void *u, char *out, size_t cap)
{
	fake_t *f = u;

	f->n_verify++;
	if (f->rc_verify != 0) {
		return f->rc_verify;
	}
	{
		size_t n = strlen(f->ver_after);

		if (n >= cap) {
			n = cap - 1U;
		}
		(void)memcpy(out, f->ver_after, n);
		out[n] = '\0';
	}
	return 0;
}

static int f_restore(void *u, bool after_failure)
{
	fake_t *f = u;

	f->n_restore++;
	f->restore_saw_failure = after_failure;
	return f->rc_restore;
}

static int f_poll(void *u)
{
	fake_t *f = u;

	f->n_poll++;
	if (f->poll_busy > 0U) {
		f->poll_busy--;
		return -EAGAIN;
	}
	return f->rc_poll;
}

static uint32_t f_chunk_max(void *u)
{
	fake_t *f = u;

	return f->chunk_limit;
}

static void ops_from(fwupd_target_ops_t *o, fake_t *f, bool full)
{
	(void)memset(o, 0, sizeof(*o));
	o->query_version = f_query;
	o->user = f;
	if (full) {
		o->prepare = f_prepare;
		o->transfer = f_transfer;
		o->verify = f_verify;
		o->restore = f_restore;
		o->chunk_max = f_chunk_max;
		if (f->have_poll) {
			o->poll = f_poll;
		}
	}
}

/* ------------------------------------------------------------ callbacks -- */

typedef struct {
	unsigned int events;
	unsigned int audits;
	int degraded_up;
	int degraded_down;
	bool degraded_now;
	uint8_t last_state;
	uint8_t last_reason;
} obs_t;

static obs_t obs;

static void cb_event(void *u, const fwupd_event_t *e)
{
	(void)u;
	obs.events++;
	obs.last_state = e->state;
	obs.last_reason = e->reason;
}

static void cb_audit(void *u, uint8_t comp, const char *what, int rc)
{
	(void)u;
	(void)comp;
	(void)what;
	(void)rc;
	obs.audits++;
}

static void cb_degraded(void *u, uint8_t comp, bool on)
{
	(void)u;
	(void)comp;
	obs.degraded_now = on;
	if (on) {
		obs.degraded_up++;
	} else {
		obs.degraded_down++;
	}
}

static void cbs(fwupd_cb_t *cb)
{
	(void)memset(cb, 0, sizeof(*cb));
	cb->event = cb_event;
	cb->audit = cb_audit;
	cb->degraded = cb_degraded;
}

/* ------------------------------------------------------------- fixtures -- */

static uint8_t image[IMG_LEN];
static uint8_t image_sha[32];

static void make_image(void)
{
	size_t i;

	for (i = 0U; i < sizeof(image); i++) {
		image[i] = (uint8_t)(i * 7U);
	}
	host_sha256(image, sizeof(image), image_sha);
}

static void req_init(fwupd_req_t *r, uint32_t size, const uint8_t *sha,
		     const char *expect)
{
	(void)memset(r, 0, sizeof(*r));
	r->magic = (uint32_t)FWUPD_MAGIC;
	r->size = size;
	if (sha != NULL) {
		(void)memcpy(r->sha256, sha, 32U);
	}
	if (expect != NULL) {
		size_t n = strlen(expect);

		if (n >= sizeof(r->expect_version)) {
			n = sizeof(r->expect_version) - 1U;
		}
		(void)memcpy(r->expect_version, expect, n);
	}
}

static void setup_ctx(fwupd_ctx_t *c, fake_t *f, uint8_t comp, bool full)
{
	fwupd_cfg_t cfg;
	fwupd_cb_t cb;
	fwupd_target_ops_t o;

	(void)memset(&obs, 0, sizeof(obs));
	fwupd_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, fwupd_cfg_allow(&cfg, comp, true));
	cbs(&cb);
	TEST_ASSERT_EQUAL_INT(0,
		fwupd_init(c, &cfg, &cb, host_sha256_stream()));
	ops_from(&o, f, full);
	TEST_ASSERT_EQUAL_INT(0, fwupd_set_target(c, comp, &o));
}

/* Stream the image in @p chunk-sized pieces. */
static void stream_all(fwupd_ctx_t *c, uint32_t chunk)
{
	uint32_t off = 0U;

	while (off < (uint32_t)sizeof(image)) {
		uint32_t n = (uint32_t)sizeof(image) - off;
		uint32_t next = 0U;

		if (n > chunk) {
			n = chunk;
		}
		TEST_ASSERT_EQUAL_INT(0, fwupd_data(c, off, &image[off], n, &next,
						    1000U));
		off += n;
		TEST_ASSERT_EQUAL_UINT32(off, next);
	}
}

/* The invariant, asserted after every terminal state. */
static void assert_balanced(const fake_t *f)
{
	TEST_ASSERT_EQUAL_UINT(f->n_prepare, f->n_restore);
	TEST_ASSERT_FALSE(obs.degraded_now);
	TEST_ASSERT_EQUAL_INT(obs.degraded_up, obs.degraded_down);
}

/* ===================================================================== *
 *  Inventory
 * ===================================================================== */

static void test_descriptors_cover_every_component(void)
{
	unsigned int i;
	unsigned int updatable = 0U;

	TEST_ASSERT_NULL(fwupd_comp_desc((uint8_t)FWUPD_COMP__COUNT));
	TEST_ASSERT_EQUAL_STRING("?", fwupd_comp_name(0xFFU));

	for (i = 0U; i < (unsigned int)FWUPD_COMP__COUNT; i++) {
		const fwupd_comp_desc_t *d = fwupd_comp_desc((uint8_t)i);

		TEST_ASSERT_NOT_NULL(d);
		/* Every row must be renderable by the maintenance tool. */
		TEST_ASSERT_EQUAL_UINT8((uint8_t)i, d->comp);
		TEST_ASSERT_NOT_NULL(d->name);
		TEST_ASSERT_NOT_NULL(d->designator);
		TEST_ASSERT_NOT_NULL(d->version_how);
		TEST_ASSERT_TRUE(d->name[0] != '\0');
		TEST_ASSERT_TRUE(d->designator[0] != '\0');
		/* "not updatable" is only useful with what CAN be read instead. */
		TEST_ASSERT_TRUE(d->version_how[0] != '\0');
		TEST_ASSERT_TRUE(d->xport < (uint8_t)FWUPD_XPORT__COUNT);
		TEST_ASSERT_TRUE(d->count >= 1U);
		if (d->updatable) {
			updatable++;
		}
	}

	/* Exactly three updatable components, and they are the expected three. */
	TEST_ASSERT_EQUAL_UINT(3U, updatable);
	TEST_ASSERT_TRUE(fwupd_comp_desc((uint8_t)FWUPD_COMP_STM32_APP)->updatable);
	TEST_ASSERT_TRUE(fwupd_comp_desc((uint8_t)FWUPD_COMP_GNSS_ZED_F9T)->updatable);
	TEST_ASSERT_TRUE(fwupd_comp_desc((uint8_t)FWUPD_COMP_RB_FE5680A)->updatable);

	/* And the read-only set really is flagged read-only. */
	TEST_ASSERT_FALSE(fwupd_comp_desc((uint8_t)FWUPD_COMP_PHY_LAN8742)->updatable);
	TEST_ASSERT_FALSE(fwupd_comp_desc((uint8_t)FWUPD_COMP_SE_ATECC608B)->updatable);
	TEST_ASSERT_FALSE(fwupd_comp_desc((uint8_t)FWUPD_COMP_INA228)->updatable);
	TEST_ASSERT_FALSE(fwupd_comp_desc((uint8_t)FWUPD_COMP_TMP117)->updatable);
	TEST_ASSERT_FALSE(fwupd_comp_desc((uint8_t)FWUPD_COMP_DISPLAY_ST7796)->updatable);
	TEST_ASSERT_FALSE(fwupd_comp_desc((uint8_t)FWUPD_COMP_TOUCH_FT6336U)->updatable);
	TEST_ASSERT_FALSE(fwupd_comp_desc((uint8_t)FWUPD_COMP_DIGIPOT_MCP41U83)->updatable);
	TEST_ASSERT_FALSE(fwupd_comp_desc((uint8_t)FWUPD_COMP_PD_NCP1095)->updatable);

	/* The nine INA228 devices are one row with a count, not nine rows. */
	TEST_ASSERT_EQUAL_UINT8(9U,
		fwupd_comp_desc((uint8_t)FWUPD_COMP_INA228)->count);
	TEST_ASSERT_EQUAL_UINT8(2U,
		fwupd_comp_desc((uint8_t)FWUPD_COMP_TMP117)->count);
}

static void test_inventory_lists_everything(void)
{
	fwupd_ctx_t c;
	fake_t f;
	fwupd_inv_row_t rows[FWUPD_COMP__COUNT];
	size_t n = 0U;
	unsigned int i;

	fake_reset(&f);
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_GNSS_ZED_F9T, true);

	TEST_ASSERT_EQUAL_INT(0, fwupd_inventory(&c, rows, sizeof(rows) /
						 sizeof(rows[0]), &n));
	TEST_ASSERT_EQUAL_UINT((size_t)FWUPD_COMP__COUNT, n);

	for (i = 0U; i < n; i++) {
		TEST_ASSERT_NOT_NULL(rows[i].desc);
		TEST_ASSERT_EQUAL_UINT8((uint8_t)i, rows[i].desc->comp);
	}

	/* The one component with a driver answered; the rest report -ENODEV. */
	TEST_ASSERT_TRUE(rows[FWUPD_COMP_GNSS_ZED_F9T].present);
	TEST_ASSERT_TRUE(rows[FWUPD_COMP_GNSS_ZED_F9T].version_valid);
	TEST_ASSERT_EQUAL_STRING("1.00", rows[FWUPD_COMP_GNSS_ZED_F9T].version);
	TEST_ASSERT_FALSE(rows[FWUPD_COMP_INA228].present);
	TEST_ASSERT_EQUAL_INT(-ENODEV, rows[FWUPD_COMP_INA228].last_rc);
	TEST_ASSERT_EQUAL_STRING("", rows[FWUPD_COMP_INA228].version);

	/* A truncating buffer reports -ENOSPC but still fills what it can. */
	n = 0U;
	TEST_ASSERT_EQUAL_INT(-ENOSPC, fwupd_inventory(&c, rows, 3U, &n));
	TEST_ASSERT_EQUAL_UINT(3U, n);

	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_inventory(NULL, rows, 3U, &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_inventory(&c, NULL, 3U, &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_inventory(&c, rows, 3U, NULL));
}

static void test_query_paths(void)
{
	fwupd_ctx_t c;
	fake_t f;
	char buf[FWUPD_VER_LEN];

	fake_reset(&f);
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_GNSS_ZED_F9T, true);

	TEST_ASSERT_EQUAL_INT(0, fwupd_query(&c, (uint8_t)FWUPD_COMP_GNSS_ZED_F9T,
					     buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_STRING("1.00", buf);

	/* A failing read leaves the string empty rather than half-written. */
	f.rc_query = -EIO;
	TEST_ASSERT_EQUAL_INT(-EIO, fwupd_query(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_STRING("", buf);

	TEST_ASSERT_EQUAL_INT(-ENODEV, fwupd_query(&c,
			(uint8_t)FWUPD_COMP_INA228, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_query(&c,
			(uint8_t)FWUPD_COMP__COUNT, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_query(NULL, 0U, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_query(&c, 0U, NULL, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_query(&c, 0U, buf, 0U));
}

/* ===================================================================== *
 *  Guards
 * ===================================================================== */

static void test_guards(void)
{
	fwupd_ctx_t c;
	fake_t f;
	fwupd_req_t r;
	fwupd_cfg_t cfg;

	make_image();
	fake_reset(&f);
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_GNSS_ZED_F9T, true);

	/* Guard 1: the magic. */
	req_init(&r, sizeof(image), image_sha, NULL);
	r.magic = 0U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 0U));
	TEST_ASSERT_EQUAL_UINT(0U, f.n_prepare);

	/* Size bounds. */
	req_init(&r, 0U, image_sha, NULL);
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 0U));
	req_init(&r, (uint32_t)FWUPD_IMAGE_MAX + 1U, image_sha, NULL);
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 0U));

	/* Guard 2: policy. Nothing is allowed by default. */
	fwupd_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_UINT32(0U, cfg.allow);
	TEST_ASSERT_EQUAL_INT(0, fwupd_set_cfg(&c, &cfg));
	req_init(&r, sizeof(image), image_sha, NULL);
	TEST_ASSERT_EQUAL_INT(-EACCES, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 0U));
	TEST_ASSERT_EQUAL_UINT(0U, f.n_prepare);

	/* No driver at all. */
	TEST_ASSERT_EQUAL_INT(0, fwupd_cfg_allow(&cfg,
			(uint8_t)FWUPD_COMP_INA228, true));
	TEST_ASSERT_EQUAL_INT(0, fwupd_set_cfg(&c, &cfg));
	TEST_ASSERT_EQUAL_INT(-ENODEV, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_INA228, &r, 0U));

	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP__COUNT, &r, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_begin(NULL, 0U, &r, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_begin(&c, 0U, NULL, 0U));

	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_cfg_allow(NULL, 0U, true));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_cfg_allow(&cfg,
			(uint8_t)FWUPD_COMP__COUNT, true));
	fwupd_cfg_defaults(NULL);
}

/*
 * Guard 3: a component with no programming callbacks — which is how every
 * read-only part is modelled — is refused with -ENOTSUP, not attempted.
 */
static void test_read_only_component_is_not_supported(void)
{
	fwupd_ctx_t c;
	fake_t f;
	fwupd_req_t r;
	fwupd_cfg_t cfg;
	fwupd_target_ops_t o;

	make_image();
	fake_reset(&f);
	(void)memset(&obs, 0, sizeof(obs));
	fwupd_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, fwupd_cfg_allow(&cfg,
			(uint8_t)FWUPD_COMP_INA228, true));
	{
		fwupd_cb_t cb;

		cbs(&cb);
		TEST_ASSERT_EQUAL_INT(0,
			fwupd_init(&c, &cfg, &cb, host_sha256_stream()));
	}
	/* Identity read only: no prepare/transfer/verify. */
	ops_from(&o, &f, false);
	TEST_ASSERT_EQUAL_INT(0, fwupd_set_target(&c,
			(uint8_t)FWUPD_COMP_INA228, &o));

	req_init(&r, sizeof(image), image_sha, NULL);
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_INA228, &r, 0U));
	TEST_ASSERT_EQUAL_UINT(0U, f.n_prepare);
	TEST_ASSERT_EQUAL_UINT(0U, f.n_restore);
	TEST_ASSERT_EQUAL_INT(0, obs.degraded_up);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_IDLE, fwupd_state(&c));

	/* But it is still inventoried, with a version. */
	{
		char buf[FWUPD_VER_LEN];

		TEST_ASSERT_EQUAL_INT(0, fwupd_query(&c,
				(uint8_t)FWUPD_COMP_INA228, buf, sizeof(buf)));
		TEST_ASSERT_EQUAL_STRING("1.00", buf);
	}
}

static void test_set_target_rejects(void)
{
	fwupd_ctx_t c;
	fake_t f;
	fwupd_target_ops_t o;
	fwupd_req_t r;

	make_image();
	fake_reset(&f);
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_GNSS_ZED_F9T, true);

	/* A target without an identity read cannot even be inventoried. */
	(void)memset(&o, 0, sizeof(o));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_set_target(&c, 0U, &o));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_set_target(NULL, 0U, &o));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_set_target(&c,
			(uint8_t)FWUPD_COMP__COUNT, &o));

	/* Deregistering works. */
	TEST_ASSERT_EQUAL_INT(0, fwupd_set_target(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, NULL));
	TEST_ASSERT_EQUAL_INT(-ENODEV, fwupd_query(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, (char[8]){ 0 }, 8U));

	/* Neither targets nor policy may change mid-session. */
	ops_from(&o, &f, true);
	TEST_ASSERT_EQUAL_INT(0, fwupd_set_target(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &o));
	req_init(&r, sizeof(image), image_sha, NULL);
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 0U));
	TEST_ASSERT_EQUAL_INT(-EBUSY, fwupd_set_target(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &o));
	{
		fwupd_cfg_t cfg;

		fwupd_cfg_defaults(&cfg);
		TEST_ASSERT_EQUAL_INT(-EBUSY, fwupd_set_cfg(&c, &cfg));
	}
	TEST_ASSERT_EQUAL_INT(-EBUSY, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 0U));
	TEST_ASSERT_EQUAL_INT(-EBUSY, fwupd_reset(&c));

	TEST_ASSERT_EQUAL_INT(0, fwupd_abort(&c, 1U));
	TEST_ASSERT_EQUAL_INT(0, fwupd_reset(&c));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_IDLE, fwupd_state(&c));
}

static void test_init_rejects(void)
{
	fwupd_ctx_t c;
	fwupd_cfg_t cfg;
	port_sha256_stream_t bad;

	fwupd_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_init(NULL, &cfg, NULL,
						  host_sha256_stream()));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_init(&c, NULL, NULL,
						  host_sha256_stream()));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_init(&c, &cfg, NULL, NULL));

	/* Hashing is mandatory: an unverified image is not an update path. */
	bad = *host_sha256_stream();
	bad.final = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_init(&c, &cfg, NULL, &bad));

	/* A NULL callback block is fine. */
	TEST_ASSERT_EQUAL_INT(0, fwupd_init(&c, &cfg, NULL,
					    host_sha256_stream()));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_IDLE, fwupd_state(&c));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_IDLE, fwupd_state(NULL));
	TEST_ASSERT_EQUAL_UINT8(0U, fwupd_component(NULL));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_END_NONE, fwupd_end_reason(NULL));
	TEST_ASSERT_EQUAL_STRING("", fwupd_version_before(NULL));
	TEST_ASSERT_EQUAL_STRING("", fwupd_version_after(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_reset(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_step(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_abort(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_end(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_progress(NULL, NULL));
}

/* ===================================================================== *
 *  The happy path, for each of the three updatable components
 * ===================================================================== */

static void run_happy(uint8_t comp, uint32_t chunk)
{
	fwupd_ctx_t c;
	fake_t f;
	fwupd_req_t r;
	fwupd_event_t p;

	make_image();
	fake_reset(&f);
	setup_ctx(&c, &f, comp, true);

	req_init(&r, sizeof(image), image_sha, "2.00");
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c, comp, &r, 100U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_TRANSFER, fwupd_state(&c));
	TEST_ASSERT_EQUAL_UINT8(comp, fwupd_component(&c));
	TEST_ASSERT_EQUAL_STRING("1.00", fwupd_version_before(&c));
	/* Timing is flagged degraded for the whole session. */
	TEST_ASSERT_TRUE(obs.degraded_now);

	stream_all(&c, chunk);

	TEST_ASSERT_EQUAL_INT(0, fwupd_progress(&c, &p));
	TEST_ASSERT_EQUAL_UINT32(sizeof(image), p.done);
	TEST_ASSERT_EQUAL_UINT16(1000U, p.permille);

	TEST_ASSERT_EQUAL_INT(0, fwupd_end(&c, 2000U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_DONE, fwupd_state(&c));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_END_OK, fwupd_end_reason(&c));
	TEST_ASSERT_EQUAL_STRING("2.00", fwupd_version_after(&c));

	/* The bytes arrived, in order, complete. */
	TEST_ASSERT_EQUAL_UINT32(sizeof(image), f.got_len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(image, f.got, sizeof(image));

	assert_balanced(&f);
	TEST_ASSERT_FALSE(f.restore_saw_failure);
	TEST_ASSERT_EQUAL_UINT(1U, f.n_verify);
	TEST_ASSERT_TRUE(obs.events > 0U);
	TEST_ASSERT_TRUE(obs.audits > 0U);

	TEST_ASSERT_EQUAL_INT(0, fwupd_reset(&c));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_IDLE, fwupd_state(&c));
}

static void test_happy_path_all_targets(void)
{
	run_happy((uint8_t)FWUPD_COMP_STM32_APP, 1024U);
	run_happy((uint8_t)FWUPD_COMP_GNSS_ZED_F9T, 512U);
	run_happy((uint8_t)FWUPD_COMP_RB_FE5680A, 64U);
	/* A chunk size that does not divide the image, so the last one is short. */
	run_happy((uint8_t)FWUPD_COMP_STM32_APP, 700U);
}

/* ===================================================================== *
 *  Abort / failure matrix — the restore invariant
 * ===================================================================== */

typedef enum {
	WHERE_AFTER_BEGIN = 0,
	WHERE_MID_TRANSFER,
	WHERE_BEFORE_END,
} abort_where_t;

static void run_abort(uint8_t comp, abort_where_t where)
{
	fwupd_ctx_t c;
	fake_t f;
	fwupd_req_t r;

	make_image();
	fake_reset(&f);
	setup_ctx(&c, &f, comp, true);

	req_init(&r, sizeof(image), image_sha, NULL);
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c, comp, &r, 0U));

	if (where != WHERE_AFTER_BEGIN) {
		uint32_t next = 0U;

		TEST_ASSERT_EQUAL_INT(0, fwupd_data(&c, 0U, image, 512U, &next,
						    10U));
	}
	if (where == WHERE_BEFORE_END) {
		uint32_t next = 0U;

		TEST_ASSERT_EQUAL_INT(0, fwupd_data(&c, 512U, &image[512], 512U,
						    &next, 20U));
	}

	TEST_ASSERT_EQUAL_INT(0, fwupd_abort(&c, 100U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_FAILED, fwupd_state(&c));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_END_ABORTED, fwupd_end_reason(&c));
	assert_balanced(&f);
	TEST_ASSERT_TRUE(f.restore_saw_failure);

	/* Abort is idempotent and safe from a terminal state. */
	TEST_ASSERT_EQUAL_INT(0, fwupd_abort(&c, 200U));
	TEST_ASSERT_EQUAL_UINT(f.n_prepare, f.n_restore);

	/* Nothing may be fed to a dead session. */
	TEST_ASSERT_EQUAL_INT(-EPERM, fwupd_data(&c, 0U, image, 16U, NULL, 300U));
	TEST_ASSERT_EQUAL_INT(-EPERM, fwupd_end(&c, 300U));
}

static void test_abort_matrix(void)
{
	uint8_t comps[3] = { (uint8_t)FWUPD_COMP_STM32_APP,
			     (uint8_t)FWUPD_COMP_GNSS_ZED_F9T,
			     (uint8_t)FWUPD_COMP_RB_FE5680A };
	unsigned int i;

	for (i = 0U; i < 3U; i++) {
		run_abort(comps[i], WHERE_AFTER_BEGIN);
		run_abort(comps[i], WHERE_MID_TRANSFER);
		run_abort(comps[i], WHERE_BEFORE_END);
	}
}

/* A failure injected at each step, with the invariant asserted every time. */
static void test_target_error_at_every_step(void)
{
	fwupd_ctx_t c;
	fake_t f;
	fwupd_req_t r;

	make_image();

	/* prepare() fails: restore() is NOT owed, but degraded must come down. */
	fake_reset(&f);
	f.rc_prepare = -EIO;
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_GNSS_ZED_F9T, true);
	req_init(&r, sizeof(image), image_sha, NULL);
	TEST_ASSERT_EQUAL_INT(-EIO, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 0U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_FAILED, fwupd_state(&c));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_END_TARGET_ERROR,
				fwupd_end_reason(&c));
	TEST_ASSERT_EQUAL_UINT(0U, f.n_restore);
	TEST_ASSERT_FALSE(obs.degraded_now);
	TEST_ASSERT_EQUAL_INT(obs.degraded_up, obs.degraded_down);

	/* transfer() fails mid-image. */
	fake_reset(&f);
	f.rc_transfer = -EIO;
	f.transfer_fail_at = 1024U;
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_GNSS_ZED_F9T, true);
	req_init(&r, sizeof(image), image_sha, NULL);
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 0U));
	TEST_ASSERT_EQUAL_INT(0, fwupd_data(&c, 0U, image, 512U, NULL, 1U));
	TEST_ASSERT_EQUAL_INT(0, fwupd_data(&c, 512U, &image[512], 512U, NULL, 2U));
	TEST_ASSERT_EQUAL_INT(-EIO, fwupd_data(&c, 1024U, &image[1024], 512U,
					       NULL, 3U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_FAILED, fwupd_state(&c));
	assert_balanced(&f);
	TEST_ASSERT_TRUE(f.restore_saw_failure);

	/* verify() fails. */
	fake_reset(&f);
	f.rc_verify = -EIO;
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_GNSS_ZED_F9T, true);
	req_init(&r, sizeof(image), image_sha, NULL);
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 0U));
	stream_all(&c, 1024U);
	TEST_ASSERT_EQUAL_INT(0, fwupd_end(&c, 10U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_FAILED, fwupd_state(&c));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_END_TARGET_ERROR,
				fwupd_end_reason(&c));
	assert_balanced(&f);

	/*
	 * restore() itself fails. Nothing can be done about it in software, so the
	 * session must NOT report success — a receiver that did not come out of
	 * safeboot is not a completed update.
	 */
	fake_reset(&f);
	f.rc_restore = -EIO;
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_GNSS_ZED_F9T, true);
	req_init(&r, sizeof(image), image_sha, NULL);
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 0U));
	stream_all(&c, 1024U);
	TEST_ASSERT_EQUAL_INT(0, fwupd_end(&c, 10U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_FAILED, fwupd_state(&c));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_END_TARGET_ERROR,
				fwupd_end_reason(&c));
	TEST_ASSERT_EQUAL_UINT(1U, f.n_restore);
	TEST_ASSERT_FALSE(obs.degraded_now);
}

/* ===================================================================== *
 *  Integrity and resume
 * ===================================================================== */

static void test_hash_mismatch_and_short_image(void)
{
	fwupd_ctx_t c;
	fake_t f;
	fwupd_req_t r;
	uint8_t wrong[32];

	make_image();
	(void)memcpy(wrong, image_sha, sizeof(wrong));
	wrong[0] ^= 0xFFU;

	/* Wrong hash: the transfer succeeded, so verify() must NOT be called. */
	fake_reset(&f);
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_STM32_APP, true);
	req_init(&r, sizeof(image), wrong, NULL);
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c, (uint8_t)FWUPD_COMP_STM32_APP,
					     &r, 0U));
	stream_all(&c, 1024U);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, fwupd_end(&c, 10U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_END_HASH, fwupd_end_reason(&c));
	TEST_ASSERT_EQUAL_UINT(0U, f.n_verify);
	assert_balanced(&f);

	/* Fewer bytes than declared. */
	fake_reset(&f);
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_STM32_APP, true);
	req_init(&r, sizeof(image), image_sha, NULL);
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c, (uint8_t)FWUPD_COMP_STM32_APP,
					     &r, 0U));
	TEST_ASSERT_EQUAL_INT(0, fwupd_data(&c, 0U, image, 512U, NULL, 1U));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, fwupd_end(&c, 10U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_END_SHORT, fwupd_end_reason(&c));
	TEST_ASSERT_EQUAL_UINT(0U, f.n_verify);
	assert_balanced(&f);
}

/*
 * The version was written but the component is not running it. Reporting success
 * here would be the single most misleading outcome this module could produce.
 */
static void test_version_not_confirmed(void)
{
	fwupd_ctx_t c;
	fake_t f;
	fwupd_req_t r;

	make_image();
	fake_reset(&f);
	f.ver_after = "1.99";
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_GNSS_ZED_F9T, true);
	req_init(&r, sizeof(image), image_sha, "2.00");
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 0U));
	stream_all(&c, 1024U);
	TEST_ASSERT_EQUAL_INT(0, fwupd_end(&c, 10U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_FAILED, fwupd_state(&c));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_END_VERIFY, fwupd_end_reason(&c));
	TEST_ASSERT_EQUAL_STRING("1.99", fwupd_version_after(&c));
	assert_balanced(&f);

	/* A substring match is enough, which is what the F9T's FWVER needs. */
	fake_reset(&f);
	f.ver_after = "EXT CORE 1.00 (a1b2c3) FWVER=TIM 2.30";
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_GNSS_ZED_F9T, true);
	req_init(&r, sizeof(image), image_sha, "TIM 2.30");
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 0U));
	stream_all(&c, 1024U);
	TEST_ASSERT_EQUAL_INT(0, fwupd_end(&c, 10U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_DONE, fwupd_state(&c));
}

static void test_resume_and_chunk_rules(void)
{
	fwupd_ctx_t c;
	fake_t f;
	fwupd_req_t r;
	uint32_t next = 0xFFFFFFFFUL;

	make_image();
	fake_reset(&f);
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_STM32_APP, true);
	req_init(&r, sizeof(image), image_sha, NULL);
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c, (uint8_t)FWUPD_COMP_STM32_APP,
					     &r, 0U));

	TEST_ASSERT_EQUAL_INT(0, fwupd_data(&c, 0U, image, 512U, &next, 1U));
	TEST_ASSERT_EQUAL_UINT32(512U, next);

	/* A gap is refused and next_off says where to resume. */
	next = 0U;
	TEST_ASSERT_EQUAL_INT(-EPROTO, fwupd_data(&c, 1024U, &image[1024], 512U,
						  &next, 2U));
	TEST_ASSERT_EQUAL_UINT32(512U, next);
	TEST_ASSERT_EQUAL_UINT32(1U, c.chunks_rejected);

	/* An exact duplicate of the already-written region is a no-op. */
	next = 0U;
	TEST_ASSERT_EQUAL_INT(0, fwupd_data(&c, 0U, image, 512U, &next, 3U));
	TEST_ASSERT_EQUAL_UINT32(512U, next);
	TEST_ASSERT_EQUAL_UINT32(1U, c.chunks_duplicate);
	/* ...and did not reach the target a second time. */
	TEST_ASSERT_EQUAL_UINT(1U, f.n_transfer);

	/* A partial overlap is refused, not trimmed: the streaming hash must
	 * cover exactly the octets the target was given. */
	next = 0U;
	TEST_ASSERT_EQUAL_INT(-EPROTO, fwupd_data(&c, 256U, &image[256], 512U,
						  &next, 4U));
	TEST_ASSERT_EQUAL_UINT32(512U, next);

	/* Over the module's own chunk ceiling, and over the target's. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_data(&c, 512U, &image[512],
			(size_t)FWUPD_CHUNK_MAX + 1U, &next, 6U));
	f.chunk_limit = 256U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_data(&c, 512U, &image[512], 512U,
						  &next, 7U));
	f.chunk_limit = 0U;

	/* Bad arguments. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_data(&c, 512U, NULL, 8U, &next, 8U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_data(&c, 512U, image, 0U, &next, 8U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fwupd_data(NULL, 0U, image, 8U, NULL, 8U));

	/* Resuming from the reported offset completes normally. */
	{
		uint32_t off = 512U;

		while (off < (uint32_t)sizeof(image)) {
			uint32_t n = (uint32_t)sizeof(image) - off;

			if (n > 1024U) {
				n = 1024U;
			}
			TEST_ASSERT_EQUAL_INT(0, fwupd_data(&c, off, &image[off],
							    n, &next, 100U));
			off += n;
		}
	}
	TEST_ASSERT_EQUAL_INT(0, fwupd_end(&c, 200U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_DONE, fwupd_state(&c));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(image, f.got, sizeof(image));
}

/* ===================================================================== *
 *  Polling and timeouts
 * ===================================================================== */

/*
 * A chunk that runs past the declared image size is -ENOSPC, distinct from a
 * chunk that is simply too large (-EINVAL). Needs its own session with a small
 * declared size, because with a 3000-octet image no chunk inside
 * FWUPD_CHUNK_MAX can overrun the end from an early offset.
 */
static void test_chunk_past_declared_end(void)
{
	fwupd_ctx_t c;
	fake_t f;
	fwupd_req_t r;
	uint8_t sha[32];
	uint32_t next = 0U;

	make_image();
	host_sha256(image, 700U, sha);

	fake_reset(&f);
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_STM32_APP, true);
	req_init(&r, 700U, sha, NULL);
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c, (uint8_t)FWUPD_COMP_STM32_APP,
					     &r, 0U));

	/* 1024 is a legal chunk size but 0 + 1024 > 700. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC, fwupd_data(&c, 0U, image, 1024U, &next, 1U));
	TEST_ASSERT_EQUAL_UINT32(0U, next);
	TEST_ASSERT_EQUAL_UINT(0U, f.n_transfer);

	/* The exact remainder is accepted and completes the image. */
	TEST_ASSERT_EQUAL_INT(0, fwupd_data(&c, 0U, image, 700U, &next, 2U));
	TEST_ASSERT_EQUAL_UINT32(700U, next);
	TEST_ASSERT_EQUAL_INT(0, fwupd_end(&c, 3U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_DONE, fwupd_state(&c));
	assert_balanced(&f);
}

static void test_poll_advances_prepare_and_verify(void)
{
	fwupd_ctx_t c;
	fake_t f;
	fwupd_req_t r;

	make_image();
	fake_reset(&f);
	f.have_poll = true;
	f.poll_busy = 3U;
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_GNSS_ZED_F9T, true);
	req_init(&r, sizeof(image), image_sha, NULL);

	/* prepare() returns, but poll() holds the session in PREPARE. */
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 0U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_PREPARE, fwupd_state(&c));
	TEST_ASSERT_EQUAL_INT(-EPERM, fwupd_data(&c, 0U, image, 16U, NULL, 1U));

	TEST_ASSERT_EQUAL_INT(0, fwupd_step(&c, 10U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_PREPARE, fwupd_state(&c));
	TEST_ASSERT_EQUAL_INT(0, fwupd_step(&c, 20U));
	TEST_ASSERT_EQUAL_INT(0, fwupd_step(&c, 30U));
	TEST_ASSERT_EQUAL_INT(0, fwupd_step(&c, 40U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_TRANSFER, fwupd_state(&c));

	stream_all(&c, 1024U);

	/* Same again for VERIFY. */
	f.poll_busy = 2U;
	TEST_ASSERT_EQUAL_INT(0, fwupd_end(&c, 50U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_VERIFY, fwupd_state(&c));
	TEST_ASSERT_EQUAL_INT(0, fwupd_step(&c, 60U));
	TEST_ASSERT_EQUAL_INT(0, fwupd_step(&c, 70U));
	TEST_ASSERT_EQUAL_INT(0, fwupd_step(&c, 80U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_DONE, fwupd_state(&c));
	assert_balanced(&f);

	/* step() on an idle context does nothing. */
	TEST_ASSERT_EQUAL_INT(0, fwupd_step(&c, 90U));
}

static void test_poll_failure(void)
{
	fwupd_ctx_t c;
	fake_t f;
	fwupd_req_t r;

	make_image();
	fake_reset(&f);
	f.have_poll = true;
	f.poll_busy = 1U;
	f.rc_poll = -EIO;
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_GNSS_ZED_F9T, true);
	req_init(&r, sizeof(image), image_sha, NULL);
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 0U));
	TEST_ASSERT_EQUAL_INT(0, fwupd_step(&c, 10U));
	TEST_ASSERT_EQUAL_INT(0, fwupd_step(&c, 20U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_FAILED, fwupd_state(&c));
	assert_balanced(&f);
}

static void test_timeouts(void)
{
	fwupd_ctx_t c;
	fake_t f;
	fwupd_req_t r;
	fwupd_cfg_t cfg;
	fwupd_cb_t cb;
	fwupd_target_ops_t o;

	make_image();

	/* A transfer that stalls is abandoned, and the component restored. */
	fake_reset(&f);
	setup_ctx(&c, &f, (uint8_t)FWUPD_COMP_GNSS_ZED_F9T, true);
	req_init(&r, sizeof(image), image_sha, NULL);
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 1000U));
	TEST_ASSERT_EQUAL_INT(0, fwupd_data(&c, 0U, image, 512U, NULL, 2000U));
	/* Inside the window: nothing happens. */
	TEST_ASSERT_EQUAL_INT(0, fwupd_step(&c, 2000U + 59999U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_TRANSFER, fwupd_state(&c));
	/* At the window: abandoned. */
	TEST_ASSERT_EQUAL_INT(0, fwupd_step(&c, 2000U + 60000U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_FAILED, fwupd_state(&c));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_END_TIMEOUT, fwupd_end_reason(&c));
	assert_balanced(&f);

	/* A step that never completes is abandoned too. */
	fake_reset(&f);
	f.have_poll = true;
	f.poll_busy = 0xFFFFFFFFUL;
	(void)memset(&obs, 0, sizeof(obs));
	fwupd_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, fwupd_cfg_allow(&cfg,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, true));
	cbs(&cb);
	TEST_ASSERT_EQUAL_INT(0, fwupd_init(&c, &cfg, &cb,
					    host_sha256_stream()));
	ops_from(&o, &f, true);
	TEST_ASSERT_EQUAL_INT(0, fwupd_set_target(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &o));
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 0U));
	TEST_ASSERT_EQUAL_INT(0, fwupd_step(&c, 29999U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_PREPARE, fwupd_state(&c));
	TEST_ASSERT_EQUAL_INT(0, fwupd_step(&c, 30000U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_FAILED, fwupd_state(&c));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_END_TIMEOUT, fwupd_end_reason(&c));
	assert_balanced(&f);

	/* Timeouts of 0 disable the check. */
	fake_reset(&f);
	(void)memset(&obs, 0, sizeof(obs));
	fwupd_cfg_defaults(&cfg);
	cfg.transfer_idle_timeout_ms = 0U;
	cfg.step_timeout_ms = 0U;
	TEST_ASSERT_EQUAL_INT(0, fwupd_cfg_allow(&cfg,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, true));
	TEST_ASSERT_EQUAL_INT(0, fwupd_init(&c, &cfg, &cb,
					    host_sha256_stream()));
	ops_from(&o, &f, true);
	TEST_ASSERT_EQUAL_INT(0, fwupd_set_target(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &o));
	TEST_ASSERT_EQUAL_INT(0, fwupd_begin(&c,
			(uint8_t)FWUPD_COMP_GNSS_ZED_F9T, &r, 0U));
	TEST_ASSERT_EQUAL_INT(0, fwupd_step(&c, 100000000UL));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)FWUPD_ST_TRANSFER, fwupd_state(&c));
	TEST_ASSERT_EQUAL_INT(0, fwupd_abort(&c, 100000001UL));
}

static void test_names(void)
{
	unsigned int i;

	for (i = 0U; i < (unsigned int)FWUPD_ST__COUNT; i++) {
		TEST_ASSERT_NOT_NULL(fwupd_state_name((uint8_t)i));
		TEST_ASSERT_TRUE(fwupd_state_name((uint8_t)i)[0] != '?');
	}
	for (i = 0U; i < (unsigned int)FWUPD_END__COUNT; i++) {
		TEST_ASSERT_NOT_NULL(fwupd_end_name((uint8_t)i));
	}
	TEST_ASSERT_EQUAL_STRING("?", fwupd_state_name(0xFFU));
	TEST_ASSERT_EQUAL_STRING("?", fwupd_end_name(0xFFU));
}

/* ===================================================================== *
 *  Runner
 * ===================================================================== */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_descriptors_cover_every_component);
	RUN_TEST(test_inventory_lists_everything);
	RUN_TEST(test_query_paths);

	RUN_TEST(test_guards);
	RUN_TEST(test_read_only_component_is_not_supported);
	RUN_TEST(test_set_target_rejects);
	RUN_TEST(test_init_rejects);

	RUN_TEST(test_happy_path_all_targets);

	RUN_TEST(test_abort_matrix);
	RUN_TEST(test_target_error_at_every_step);

	RUN_TEST(test_hash_mismatch_and_short_image);
	RUN_TEST(test_version_not_confirmed);
	RUN_TEST(test_resume_and_chunk_rules);
	RUN_TEST(test_chunk_past_declared_end);

	RUN_TEST(test_poll_advances_prepare_and_verify);
	RUN_TEST(test_poll_failure);
	RUN_TEST(test_timeouts);
	RUN_TEST(test_names);

	return UNITY_END();
}
