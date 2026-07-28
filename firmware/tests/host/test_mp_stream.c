/*
 * STS1000 "Meridian" — core/mp CBOR writer and stream-record unit tests.
 *
 * The CBOR primitives are checked against the encodings in RFC 8949 Appendix A
 * rather than against this writer's own output, so a head-encoding mistake
 * cannot pass by agreeing with itself. Every record is then round-tripped
 * through the reader field by field: the telemetry record in particular is the
 * device's highest-rate output and a silently mis-keyed field would show up on
 * the host as plausible-looking wrong data.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "mp/mp_stream.h"
#include "test_support.h"

static uint8_t g_buf[8192];
static mp_cbor_t g_w;
static mp_stream_ctx_t g_st;

void setUp(void)
{
	memset(g_buf, 0, sizeof(g_buf));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_init(&g_w, g_buf, sizeof(g_buf)));
	TEST_ASSERT_EQUAL_INT(0, mp_stream_init(&g_st));
}

/* ========================================================= CBOR writer === */

/** Assert the writer's bytes equal a literal encoding. */
static void expect_bytes(const uint8_t *want, size_t n)
{
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_finish(&g_w, &len));
	TEST_ASSERT_EQUAL_size_t(n, len);
	TEST_ASSERT_EQUAL_UINT8_ARRAY(want, g_buf, n);
}

static void test_cbor_uint_vectors(void)
{
	/* RFC 8949 Appendix A. */
	static const struct {
		uint64_t v;
		uint8_t enc[9];
		size_t n;
	} vec[] = {
		{ 0U, { 0x00 }, 1U },
		{ 1U, { 0x01 }, 1U },
		{ 10U, { 0x0A }, 1U },
		{ 23U, { 0x17 }, 1U },
		{ 24U, { 0x18, 0x18 }, 2U },
		{ 25U, { 0x18, 0x19 }, 2U },
		{ 100U, { 0x18, 0x64 }, 2U },
		{ 1000U, { 0x19, 0x03, 0xE8 }, 3U },
		{ 1000000U, { 0x1A, 0x00, 0x0F, 0x42, 0x40 }, 5U },
		{ 1000000000000ULL,
		  { 0x1B, 0x00, 0x00, 0x00, 0xE8, 0xD4, 0xA5, 0x10, 0x00 },
		  9U },
		{ 18446744073709551615ULL,
		  { 0x1B, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
		  9U },
		{ 255U, { 0x18, 0xFF }, 2U },
		{ 256U, { 0x19, 0x01, 0x00 }, 3U },
		{ 65535U, { 0x19, 0xFF, 0xFF }, 3U },
		{ 65536U, { 0x1A, 0x00, 0x01, 0x00, 0x00 }, 5U },
		{ 4294967295ULL, { 0x1A, 0xFF, 0xFF, 0xFF, 0xFF }, 5U },
		{ 4294967296ULL,
		  { 0x1B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00 },
		  9U },
	};
	size_t i;

	for (i = 0U; i < (sizeof(vec) / sizeof(vec[0])); i++) {
		size_t len = 0U;

		TEST_ASSERT_EQUAL_INT(0,
				      mp_cbor_init(&g_w, g_buf, sizeof(g_buf)));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_uint(&g_w, vec[i].v));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_finish(&g_w, &len));
		TEST_ASSERT_EQUAL_size_t(vec[i].n, len);
		TEST_ASSERT_EQUAL_UINT8_ARRAY(vec[i].enc, g_buf, vec[i].n);
	}
}

static void test_cbor_int_vectors(void)
{
	static const struct {
		int64_t v;
		uint8_t enc[9];
		size_t n;
	} vec[] = {
		{ 0, { 0x00 }, 1U },
		{ 1, { 0x01 }, 1U },
		{ -1, { 0x20 }, 1U },
		{ -10, { 0x29 }, 1U },
		{ -24, { 0x37 }, 1U },
		{ -25, { 0x38, 0x18 }, 2U },
		{ -100, { 0x38, 0x63 }, 2U },
		{ -1000, { 0x39, 0x03, 0xE7 }, 3U },
		{ -1000000, { 0x3A, 0x00, 0x0F, 0x42, 0x3F }, 5U },
		{ (int64_t)(-9223372036854775807LL) - 1LL,
		  { 0x3B, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
		  9U },
		{ 9223372036854775807LL,
		  { 0x1B, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
		  9U },
	};
	size_t i;

	for (i = 0U; i < (sizeof(vec) / sizeof(vec[0])); i++) {
		size_t len = 0U;

		TEST_ASSERT_EQUAL_INT(0,
				      mp_cbor_init(&g_w, g_buf, sizeof(g_buf)));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_int(&g_w, vec[i].v));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_finish(&g_w, &len));
		TEST_ASSERT_EQUAL_size_t(vec[i].n, len);
		TEST_ASSERT_EQUAL_UINT8_ARRAY(vec[i].enc, g_buf, vec[i].n);
	}
}

static void test_cbor_other_vectors(void)
{
	static const uint8_t want_false[] = { 0xF4 };
	static const uint8_t want_true[] = { 0xF5 };
	static const uint8_t want_null[] = { 0xF6 };
	static const uint8_t want_empty_bstr[] = { 0x40 };
	static const uint8_t want_bstr[] = { 0x44, 0x01, 0x02, 0x03, 0x04 };
	static const uint8_t want_empty_tstr[] = { 0x60 };
	static const uint8_t want_tstr[] = { 0x61, 0x61 };            /* "a" */
	static const uint8_t want_ietf[] = { 0x64, 0x49, 0x45, 0x54,
					     0x46 };                  /* "IETF" */
	static const uint8_t want_arr[] = { 0x83, 0x01, 0x02, 0x03 };
	static const uint8_t want_empty_arr[] = { 0x80 };
	static const uint8_t want_map[] = { 0xA2, 0x01, 0x02, 0x03, 0x04 };
	static const uint8_t want_empty_map[] = { 0xA0 };
	/* 1.0f, 100000.0f, 3.4028234663852886e+38 (RFC 8949 §A). */
	static const uint8_t want_f1[] = { 0xFA, 0x3F, 0x80, 0x00, 0x00 };
	static const uint8_t want_f2[] = { 0xFA, 0x47, 0xC3, 0x50, 0x00 };
	static const uint8_t want_f3[] = { 0xFA, 0x7F, 0x7F, 0xFF, 0xFF };

#define ONE(call, want)                                                        \
	do {                                                                   \
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_init(&g_w, g_buf,             \
						     sizeof(g_buf)));          \
		TEST_ASSERT_EQUAL_INT(0, call);                                \
		expect_bytes(want, sizeof(want));                              \
	} while (0)

	ONE(mp_cbor_bool(&g_w, false), want_false);
	ONE(mp_cbor_bool(&g_w, true), want_true);
	ONE(mp_cbor_null(&g_w), want_null);
	ONE(mp_cbor_bytes(&g_w, NULL, 0U), want_empty_bstr);
	ONE(mp_cbor_bytes(&g_w, (const uint8_t *)"\x01\x02\x03\x04", 4U),
	    want_bstr);
	ONE(mp_cbor_text(&g_w, ""), want_empty_tstr);
	ONE(mp_cbor_text(&g_w, "a"), want_tstr);
	ONE(mp_cbor_text(&g_w, "IETF"), want_ietf);
	ONE(mp_cbor_arr(&g_w, 0U), want_empty_arr);
	ONE(mp_cbor_map(&g_w, 0U), want_empty_map);
	ONE(mp_cbor_f32(&g_w, 1.0f), want_f1);
	ONE(mp_cbor_f32(&g_w, 100000.0f), want_f2);
	ONE(mp_cbor_f32(&g_w, 3.4028234663852886e+38f), want_f3);
#undef ONE

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_init(&g_w, g_buf, sizeof(g_buf)));
	(void)mp_cbor_arr(&g_w, 3U);
	(void)mp_cbor_uint(&g_w, 1U);
	(void)mp_cbor_uint(&g_w, 2U);
	(void)mp_cbor_uint(&g_w, 3U);
	expect_bytes(want_arr, sizeof(want_arr));

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_init(&g_w, g_buf, sizeof(g_buf)));
	(void)mp_cbor_map(&g_w, 2U);
	(void)mp_cbor_kv_uint(&g_w, 1U, 2U);
	(void)mp_cbor_kv_uint(&g_w, 3U, 4U);
	expect_bytes(want_map, sizeof(want_map));

	/* A NULL text is CBOR null, not an empty string. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_init(&g_w, g_buf, sizeof(g_buf)));
	(void)mp_cbor_text(&g_w, NULL);
	expect_bytes(want_null, sizeof(want_null));

	/* And a NULL byte string through the kv helper is likewise null. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_init(&g_w, g_buf, sizeof(g_buf)));
	(void)mp_cbor_kv_bytes(&g_w, 0U, NULL, 4U);
	{
		size_t len = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_finish(&g_w, &len));
		TEST_ASSERT_EQUAL_size_t(2U, len);
		TEST_ASSERT_EQUAL_UINT8(0x00, g_buf[0]);
		TEST_ASSERT_EQUAL_UINT8(0xF6, g_buf[1]);
	}
}

static void test_cbor_kv_helpers(void)
{
	size_t len = 0U;
	mp_cbor_rd_t r;
	int64_t iv = 0;
	bool bv = false;
	float fv = 0.0f;
	const char *tp = NULL;
	size_t tn = 0U;
	const uint8_t *bp = NULL;
	size_t bn = 0U;

	(void)mp_cbor_map(&g_w, 5U);
	(void)mp_cbor_kv_int(&g_w, 1U, -7);
	(void)mp_cbor_kv_bool(&g_w, 2U, true);
	(void)mp_cbor_kv_f32(&g_w, 3U, -0.5f);
	(void)mp_cbor_kv_text(&g_w, 4U, "hi");
	(void)mp_cbor_kv_bytes(&g_w, 5U, (const uint8_t *)"\xAA\xBB", 2U);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_finish(&g_w, &len));

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, 1U));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_int(&r, &iv));
	TEST_ASSERT_EQUAL_INT64(-7, iv);

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, 2U));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_bool(&r, &bv));
	TEST_ASSERT_TRUE(bv);

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, 3U));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_f32(&r, &fv));
	TEST_ASSERT_EQUAL_FLOAT(-0.5f, fv);

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, 4U));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_text(&r, &tp, &tn));
	TEST_ASSERT_EQUAL_size_t(2U, tn);
	TEST_ASSERT_EQUAL_UINT8_ARRAY("hi", tp, 2U);

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, 5U));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_bytes(&r, &bp, &bn));
	TEST_ASSERT_EQUAL_size_t(2U, bn);
	TEST_ASSERT_EQUAL_UINT8_ARRAY("\xAA\xBB", bp, 2U);

	/* A key that is not there. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, len));
	TEST_ASSERT_EQUAL_INT(-ENOENT, mp_cbor_map_find(&r, 99U));
}

static void test_cbor_writer_overflow(void)
{
	uint8_t tiny[3];
	mp_cbor_t w;
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_init(&w, tiny, sizeof(tiny)));
	(void)mp_cbor_uint(&w, 1000000U); /* needs 5 bytes */
	TEST_ASSERT_EQUAL_INT(-ENOSPC, mp_cbor_finish(&w, &len));
	/* Sticky: later writes keep failing. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC, mp_cbor_uint(&w, 0U));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, mp_cbor_bool(&w, true));

	/* A byte string longer than the buffer fails on the payload, not the head. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_init(&w, tiny, sizeof(tiny)));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      mp_cbor_bytes(&w, (const uint8_t *)"abcdef", 6U));
}

static void test_cbor_writer_argument_validation(void)
{
	mp_cbor_t w;

	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_init(NULL, g_buf, 4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_init(&w, NULL, 4U));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_init(&w, NULL, 0U));

	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_uint(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_int(NULL, 0));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_bytes(NULL, NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_textn(NULL, NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_arr(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_map(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_bool(NULL, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_null(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_f32(NULL, 0.0f));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_finish(NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_bytes(&g_w, NULL, 4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_textn(&g_w, NULL, 4U));
}

/* ------------------------------------------------------------ CBOR reader */

static void test_cbor_reader_rejections(void)
{
	mp_cbor_rd_t r;
	uint64_t u = 0U;
	int64_t i = 0;
	size_t n = 0U;
	bool b = false;
	float f = 0.0f;
	const uint8_t *p = NULL;
	static const uint8_t indefinite[] = { 0x9F, 0x01, 0xFF };
	static const uint8_t reserved[] = { 0x1C };
	static const uint8_t truncated[] = { 0x19, 0x01 };
	static const uint8_t liar[] = { 0x58, 0x40, 0x01 }; /* bstr len 64, 1 byte */
	static const uint8_t big_arr[] = { 0x9A, 0xFF, 0xFF, 0xFF, 0xFF };

	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_rd_init(NULL, g_buf, 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_rd_init(&r, NULL, 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_peek_major(NULL));

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, 0U));
	TEST_ASSERT_EQUAL_INT(-ENODATA, mp_cbor_peek_major(&r));
	TEST_ASSERT_EQUAL_INT(-ENODATA, mp_cbor_rd_uint(&r, &u));
	TEST_ASSERT_EQUAL_INT(-ENODATA, mp_cbor_rd_bool(&r, &b));
	TEST_ASSERT_EQUAL_INT(-ENODATA, mp_cbor_rd_f32(&r, &f));

	/* Indefinite length is refused: the writer never emits it. */
	TEST_ASSERT_EQUAL_INT(0,
			      mp_cbor_rd_init(&r, indefinite, sizeof(indefinite)));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, mp_cbor_rd_arr(&r, &n));

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, reserved, sizeof(reserved)));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, mp_cbor_rd_uint(&r, &u));

	TEST_ASSERT_EQUAL_INT(0,
			      mp_cbor_rd_init(&r, truncated, sizeof(truncated)));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, mp_cbor_rd_uint(&r, &u));

	/* A declared length longer than the remaining bytes is rejected, not
	 * trusted — this is the bound that keeps the reader inside the buffer. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, liar, sizeof(liar)));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, mp_cbor_rd_bytes(&r, &p, &n));

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, big_arr, sizeof(big_arr)));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, mp_cbor_skip(&r));

	/* Type mismatches. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_init(&g_w, g_buf, sizeof(g_buf)));
	(void)mp_cbor_text(&g_w, "x");
	(void)mp_cbor_finish(&g_w, &n);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, n));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, mp_cbor_rd_uint(&r, &u));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, n));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, mp_cbor_rd_int(&r, &i));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, n));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, mp_cbor_rd_map(&r, &n));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, 2U));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, mp_cbor_rd_bool(&r, &b));

	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_rd_uint(&r, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_rd_int(&r, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_rd_map(&r, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_rd_bytes(&r, NULL, &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_rd_bool(&r, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_rd_f32(&r, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_skip(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_cbor_map_find(NULL, 0U));
}

static void test_cbor_skip_walks_containers(void)
{
	mp_cbor_rd_t r;
	size_t len = 0U;
	uint64_t u = 0U;

	(void)mp_cbor_map(&g_w, 2U);
	(void)mp_cbor_uint(&g_w, 1U);
	(void)mp_cbor_arr(&g_w, 3U);
	(void)mp_cbor_uint(&g_w, 10U);
	(void)mp_cbor_map(&g_w, 1U);
	(void)mp_cbor_kv_text(&g_w, 5U, "nested");
	(void)mp_cbor_bytes(&g_w, (const uint8_t *)"\x01\x02", 2U);
	(void)mp_cbor_kv_uint(&g_w, 2U, 42U);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_finish(&g_w, &len));

	/* Finding key 2 requires skipping the whole nested array. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, 2U));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	TEST_ASSERT_EQUAL_UINT64(42U, u);

	/* Skipping every simple type is also exercised. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_init(&g_w, g_buf, sizeof(g_buf)));
	(void)mp_cbor_arr(&g_w, 6U);
	(void)mp_cbor_uint(&g_w, 1U);
	(void)mp_cbor_int(&g_w, -1);
	(void)mp_cbor_bool(&g_w, true);
	(void)mp_cbor_null(&g_w);
	(void)mp_cbor_f32(&g_w, 1.5f);
	(void)mp_cbor_text(&g_w, "z");
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_finish(&g_w, &len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_skip(&r));
	TEST_ASSERT_EQUAL_INT(-ENODATA, mp_cbor_peek_major(&r));
}

static void test_cbor_skip_depth_is_bounded(void)
{
	uint8_t deep[64];
	mp_cbor_rd_t r;
	size_t i;

	/* 40 nested one-element arrays: past the skip depth cap. */
	for (i = 0U; i < sizeof(deep) - 1U; i++) {
		deep[i] = 0x81U;
	}
	deep[sizeof(deep) - 1U] = 0x00U;

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, deep, sizeof(deep)));
	TEST_ASSERT_EQUAL_INT(-E2BIG, mp_cbor_skip(&r));
}

/* ========================================================== telemetry ==== */

/** Fill a telemetry input with distinct values so a mis-key is visible. */
static void fill_telem(mp_telem_t *t)
{
	size_t i;

	memset(t, 0, sizeof(*t));
	t->seq = 0x11223344U;
	t->mono_ms = 0x0000000123456789ULL;
	t->tai_ns = 0x0FEDCBA987654321ULL;
	t->time_fallback = true;
	t->uptime_s = 987654U;

	t->q.stratum = 1U;
	t->q.lock_state = (uint8_t)QUALITY_LOCK_LOCKED;
	t->q.active_ref = (uint8_t)QUALITY_REF_RB;
	t->q.gnss_fix = (uint8_t)QUALITY_GNSS_TIME_ONLY;
	t->q.gnss_sv_used = 14U;
	t->q.gnss_sv_visible = 21U;
	t->q.gnss_tacc_ns = 12345U;
	t->q.last_pps_off_ns = -4321;
	t->q.pps_off_mean_ns = -1.25f;
	t->q.pps_off_sigma_ns = 3.5f;
	t->q.freq_err_ppb = -0.125f;
	t->q.vc_cmd_mv = 1650;
	t->q.vc_sense_mv = 1648;
	t->q.dac_code = 2048U;
	t->q.adev_1s = 1.0e-11f;
	t->q.adev_10s = 5.0e-12f;
	t->q.adev_100s = 2.5e-12f;
	t->q.root_delay_q16 = 0U;
	t->q.root_disp_q16 = 6553U;
	t->q.holdover_elapsed_s = 42U;
	t->q.holdover_est_err_ns = -98765;
	t->q.holdover_t_demote_s = 3600U;
	t->q.flags = QUALITY_FLAG_OCXO_WARM | QUALITY_FLAG_VC_SENSE_VALID;
	t->q.osc_temp_mc = 55250;
	t->q.leap_pending = 1;
	t->q.leap_current_s = 37;
	t->q.leap_at_tai_s = 1234567890ULL;

	for (i = 0U; i < MP_RAIL_COUNT; i++) {
		t->h.rail[i].bus_mv = (int32_t)(1000 + (int32_t)i);
		t->h.rail[i].current_ua = (int32_t)(-2000 - (int32_t)i);
		t->h.rail[i].power_uw = (uint32_t)(3000U + i);
		t->h.rail[i].diag = (uint16_t)(0x0100U + i);
		t->h.rail[i].valid = ((i % 2U) == 0U);
	}
	t->h.tmp_osc_mc = 55000;
	t->h.tmp_osc_valid = true;
	t->h.tmp_amb_mc = 28000;
	t->h.tmp_amb_valid = true;
	t->h.die_mc = -1000;
	t->h.die_valid = false;
	t->h.humidity_mpct = 42500;
	t->h.humidity_valid = true;
	t->h.fan_rpm = 4200U;
	t->h.fan_duty_pct = 55U;
	t->h.poe_class = 4U;
	t->h.poe_draw_mw = 13500U;
	t->h.poe_budget_mw = 25500U;
	t->h.bkp_stm_pg = true;
	t->h.bkp_gps_pg = false;

	t->alarms = 0x00000000DEADBEEFULL;
	t->alarms_latched = 0x00000000FEEDFACEULL;
	t->scan_state = 0xA5A5A5A5U;
	t->pwrseq_stage = 7U;
	t->pwrseq_shed = 2U;
	t->pwrseq_alarms = 0x1234U;
	t->gnss_state = 3U;
	t->gnss_ant = 1U;
	t->survey_dur_s = 3600U;
	t->survey_acc_mm = 1500U;
	t->rb_lock = true;
	t->rb_powered = true;
	t->extref_ok = false;
	t->pfi = false;
	t->extref_hz = 10000000U;
	t->refsel_state = 1U;
}

/** Position a reader on key @p k of the record in @p buf. */
static void seek(mp_cbor_rd_t *r, const uint8_t *buf, size_t len, uint64_t k)
{
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(r, buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(r, k));
}

static uint64_t get_u(const uint8_t *buf, size_t len, uint64_t k)
{
	mp_cbor_rd_t r;
	uint64_t v = 0U;

	seek(&r, buf, len, k);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &v));
	return v;
}

static int64_t get_i(const uint8_t *buf, size_t len, uint64_t k)
{
	mp_cbor_rd_t r;
	int64_t v = 0;

	seek(&r, buf, len, k);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_int(&r, &v));
	return v;
}

static float get_f(const uint8_t *buf, size_t len, uint64_t k)
{
	mp_cbor_rd_t r;
	float v = 0.0f;

	seek(&r, buf, len, k);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_f32(&r, &v));
	return v;
}

static bool get_b(const uint8_t *buf, size_t len, uint64_t k)
{
	mp_cbor_rd_t r;
	bool v = false;

	seek(&r, buf, len, k);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_bool(&r, &v));
	return v;
}

static void test_telemetry_round_trip(void)
{
	mp_telem_t t;
	mp_cbor_rd_t r;
	size_t n;
	size_t pairs = 0U;
	size_t i;
	int len = 0;

	fill_telem(&t);
	len = mp_enc_telem(&t, g_buf, sizeof(g_buf));
	TEST_ASSERT_TRUE(len > 0);
	TEST_ASSERT_TRUE_MESSAGE((size_t)len <= MP_TELEM_MAX,
				 "MP_TELEM_MAX is too small for the record");
	n = (size_t)len;

	/* The record is one definite-length map with the declared pair count. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, n));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_map(&r, &pairs));
	TEST_ASSERT_EQUAL_size_t(4U + 44U, pairs);

	/* Common header. */
	TEST_ASSERT_EQUAL_UINT64(MP_REC_TELEM, get_u(g_buf, n, MP_K_TYPE));
	TEST_ASSERT_EQUAL_UINT64(1U, get_u(g_buf, n, MP_K_VER));
	TEST_ASSERT_EQUAL_UINT64(t.seq, get_u(g_buf, n, MP_K_SEQ));
	TEST_ASSERT_EQUAL_UINT64(t.mono_ms, get_u(g_buf, n, MP_K_MONO));

	/* Scalars, key by key. */
	TEST_ASSERT_EQUAL_UINT64(t.tai_ns, get_u(g_buf, n, 8U));
	TEST_ASSERT_TRUE(get_b(g_buf, n, 9U));
	TEST_ASSERT_EQUAL_UINT64(t.uptime_s, get_u(g_buf, n, 10U));
	TEST_ASSERT_EQUAL_UINT64(t.q.stratum, get_u(g_buf, n, 11U));
	TEST_ASSERT_EQUAL_UINT64(t.q.lock_state, get_u(g_buf, n, 12U));
	TEST_ASSERT_EQUAL_UINT64(t.q.active_ref, get_u(g_buf, n, 13U));
	TEST_ASSERT_EQUAL_UINT64(t.q.gnss_fix, get_u(g_buf, n, 14U));
	TEST_ASSERT_EQUAL_UINT64(t.q.gnss_sv_used, get_u(g_buf, n, 15U));
	TEST_ASSERT_EQUAL_UINT64(t.q.gnss_sv_visible, get_u(g_buf, n, 16U));
	TEST_ASSERT_EQUAL_UINT64(t.q.gnss_tacc_ns, get_u(g_buf, n, 17U));
	TEST_ASSERT_EQUAL_INT64(t.q.last_pps_off_ns, get_i(g_buf, n, 18U));
	TEST_ASSERT_EQUAL_FLOAT(t.q.pps_off_mean_ns, get_f(g_buf, n, 19U));
	TEST_ASSERT_EQUAL_FLOAT(t.q.pps_off_sigma_ns, get_f(g_buf, n, 20U));
	TEST_ASSERT_EQUAL_FLOAT(t.q.freq_err_ppb, get_f(g_buf, n, 21U));
	TEST_ASSERT_EQUAL_INT64(t.q.vc_cmd_mv, get_i(g_buf, n, 22U));
	TEST_ASSERT_EQUAL_INT64(t.q.vc_sense_mv, get_i(g_buf, n, 23U));
	TEST_ASSERT_EQUAL_UINT64(t.q.dac_code, get_u(g_buf, n, 24U));

	/* ADEV triple. */
	seek(&r, g_buf, n, 25U);
	{
		size_t cnt = 0U;
		float a = 0.0f;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
		TEST_ASSERT_EQUAL_size_t(3U, cnt);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_f32(&r, &a));
		TEST_ASSERT_EQUAL_FLOAT(t.q.adev_1s, a);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_f32(&r, &a));
		TEST_ASSERT_EQUAL_FLOAT(t.q.adev_10s, a);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_f32(&r, &a));
		TEST_ASSERT_EQUAL_FLOAT(t.q.adev_100s, a);
	}

	TEST_ASSERT_EQUAL_UINT64(t.q.root_delay_q16, get_u(g_buf, n, 26U));
	TEST_ASSERT_EQUAL_UINT64(t.q.root_disp_q16, get_u(g_buf, n, 27U));
	TEST_ASSERT_EQUAL_UINT64(t.q.holdover_elapsed_s, get_u(g_buf, n, 28U));
	TEST_ASSERT_EQUAL_INT64(t.q.holdover_est_err_ns, get_i(g_buf, n, 29U));
	TEST_ASSERT_EQUAL_UINT64(t.q.holdover_t_demote_s, get_u(g_buf, n, 30U));
	TEST_ASSERT_EQUAL_UINT64(t.q.flags, get_u(g_buf, n, 31U));
	TEST_ASSERT_EQUAL_INT64(t.q.osc_temp_mc, get_i(g_buf, n, 32U));

	/* Leap triple. */
	seek(&r, g_buf, n, 33U);
	{
		size_t cnt = 0U;
		int64_t v = 0;
		uint64_t u = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
		TEST_ASSERT_EQUAL_size_t(3U, cnt);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_int(&r, &v));
		TEST_ASSERT_EQUAL_INT64(t.q.leap_pending, v);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_int(&r, &v));
		TEST_ASSERT_EQUAL_INT64(t.q.leap_current_s, v);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(t.q.leap_at_tai_s, u);
	}

	/* Nine rails, in ina228_rail_t order, five fields each. */
	seek(&r, g_buf, n, 34U);
	{
		size_t cnt = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
		TEST_ASSERT_EQUAL_size_t(MP_RAIL_COUNT, cnt);
		for (i = 0U; i < MP_RAIL_COUNT; i++) {
			size_t f = 0U;
			int64_t v = 0;
			uint64_t u = 0U;
			bool b = false;

			TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &f));
			TEST_ASSERT_EQUAL_size_t(5U, f);
			TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_int(&r, &v));
			TEST_ASSERT_EQUAL_INT64(t.h.rail[i].bus_mv, v);
			TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_int(&r, &v));
			TEST_ASSERT_EQUAL_INT64(t.h.rail[i].current_ua, v);
			TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
			TEST_ASSERT_EQUAL_UINT64(t.h.rail[i].power_uw, u);
			TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
			TEST_ASSERT_EQUAL_UINT64(t.h.rail[i].diag, u);
			TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_bool(&r, &b));
			TEST_ASSERT_EQUAL_INT(t.h.rail[i].valid ? 1 : 0,
					      b ? 1 : 0);
		}
	}

	TEST_ASSERT_EQUAL_INT64(t.h.tmp_osc_mc, get_i(g_buf, n, 35U));
	TEST_ASSERT_EQUAL_INT64(t.h.tmp_amb_mc, get_i(g_buf, n, 36U));
	TEST_ASSERT_EQUAL_INT64(t.h.die_mc, get_i(g_buf, n, 37U));
	TEST_ASSERT_EQUAL_INT64(t.h.humidity_mpct, get_i(g_buf, n, 38U));
	/* Validity bits: osc | amb | humidity, die absent. */
	TEST_ASSERT_EQUAL_UINT64(0x0BU, get_u(g_buf, n, 39U));

	seek(&r, g_buf, n, 40U);
	{
		size_t cnt = 0U;
		uint64_t u = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
		TEST_ASSERT_EQUAL_size_t(2U, cnt);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(t.h.fan_rpm, u);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(t.h.fan_duty_pct, u);
	}

	seek(&r, g_buf, n, 41U);
	{
		size_t cnt = 0U;
		uint64_t u = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
		TEST_ASSERT_EQUAL_size_t(3U, cnt);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(t.h.poe_class, u);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(t.h.poe_draw_mw, u);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(t.h.poe_budget_mw, u);
	}

	seek(&r, g_buf, n, 42U);
	{
		size_t cnt = 0U;
		bool b = false;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
		TEST_ASSERT_EQUAL_size_t(2U, cnt);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_bool(&r, &b));
		TEST_ASSERT_TRUE(b);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_bool(&r, &b));
		TEST_ASSERT_FALSE(b);
	}

	TEST_ASSERT_EQUAL_UINT64(t.alarms, get_u(g_buf, n, 43U));
	TEST_ASSERT_EQUAL_UINT64(t.alarms_latched, get_u(g_buf, n, 44U));
	TEST_ASSERT_EQUAL_UINT64(t.scan_state, get_u(g_buf, n, 45U));

	seek(&r, g_buf, n, 46U);
	{
		size_t cnt = 0U;
		uint64_t u = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
		TEST_ASSERT_EQUAL_size_t(3U, cnt);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(t.pwrseq_stage, u);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(t.pwrseq_shed, u);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(t.pwrseq_alarms, u);
	}

	/* Reference flag bitmap: rb_lock | rb_powered, extref and PFI clear. */
	TEST_ASSERT_EQUAL_UINT64(0x03U, get_u(g_buf, n, 49U));
	TEST_ASSERT_EQUAL_UINT64(t.extref_hz, get_u(g_buf, n, 50U));
	TEST_ASSERT_EQUAL_UINT64(t.refsel_state, get_u(g_buf, n, 51U));

	/* Every declared pair is reachable and the record ends cleanly. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, n));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_skip(&r));
	TEST_ASSERT_EQUAL_INT(-ENODATA, mp_cbor_peek_major(&r));
}

static void test_telemetry_errors(void)
{
	mp_telem_t t;
	uint8_t tiny[16];

	fill_telem(&t);
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_enc_telem(NULL, g_buf, sizeof(g_buf)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_enc_telem(&t, NULL, sizeof(g_buf)));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, mp_enc_telem(&t, tiny, sizeof(tiny)));
}

/* ================================================================= PPS === */

static void test_pps_round_trip(void)
{
	mp_pps_t p;
	mp_cbor_rd_t r;
	size_t pairs = 0U;
	size_t n;
	int len;

	memset(&p, 0, sizeof(p));
	p.seq = 77U;
	p.mono_ms = 1234567ULL;
	p.pa0_ns = -123456789012345LL;
	p.pc6_ns = 123456789012345LL;
	p.pc6_valid = true;
	p.qerr_ps = -3210;
	p.residual_ns = -55;
	p.residual_corr_ns = -52;
	p.cable_delay_ns = 137;
	p.interval_ns = -9;
	p.vc_cmd_mv = 1651;
	p.vc_sense_mv = 1649;
	p.dac_code = 2049U;
	p.loop_state = (uint8_t)QUALITY_LOCK_LOCKING;
	p.active_ref = (uint8_t)QUALITY_REF_OCXO;
	p.ref_flags = QUALITY_FLAG_PPS_REJECT;
	p.accepted = false;
	p.reject_reason = 3U;

	len = mp_enc_pps(&p, g_buf, sizeof(g_buf));
	TEST_ASSERT_TRUE(len > 0);
	TEST_ASSERT_TRUE((size_t)len <= MP_PPS_MAX);
	n = (size_t)len;

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, n));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_map(&r, &pairs));
	TEST_ASSERT_EQUAL_size_t(4U + 16U, pairs);

	TEST_ASSERT_EQUAL_UINT64(MP_REC_PPS, get_u(g_buf, n, MP_K_TYPE));
	TEST_ASSERT_EQUAL_UINT64(p.seq, get_u(g_buf, n, MP_K_SEQ));
	/* Both capture channels, raw. */
	TEST_ASSERT_EQUAL_INT64(p.pa0_ns, get_i(g_buf, n, 8U));
	TEST_ASSERT_EQUAL_INT64(p.pc6_ns, get_i(g_buf, n, 9U));
	TEST_ASSERT_TRUE(get_b(g_buf, n, 10U));
	/* The sawtooth term, and the residual before and after applying it. */
	TEST_ASSERT_EQUAL_INT64(p.qerr_ps, get_i(g_buf, n, 11U));
	TEST_ASSERT_EQUAL_INT64(p.residual_ns, get_i(g_buf, n, 12U));
	TEST_ASSERT_EQUAL_INT64(p.residual_corr_ns, get_i(g_buf, n, 13U));
	TEST_ASSERT_EQUAL_INT64(p.cable_delay_ns, get_i(g_buf, n, 14U));
	TEST_ASSERT_EQUAL_INT64(p.vc_cmd_mv, get_i(g_buf, n, 15U));
	TEST_ASSERT_EQUAL_INT64(p.vc_sense_mv, get_i(g_buf, n, 16U));
	TEST_ASSERT_EQUAL_UINT64(p.dac_code, get_u(g_buf, n, 17U));
	TEST_ASSERT_EQUAL_UINT64(p.loop_state, get_u(g_buf, n, 18U));
	TEST_ASSERT_EQUAL_UINT64(p.active_ref, get_u(g_buf, n, 19U));
	TEST_ASSERT_EQUAL_UINT64(p.ref_flags, get_u(g_buf, n, 20U));
	TEST_ASSERT_FALSE(get_b(g_buf, n, 21U));
	TEST_ASSERT_EQUAL_UINT64(p.reject_reason, get_u(g_buf, n, 22U));
	TEST_ASSERT_EQUAL_INT64(p.interval_ns, get_i(g_buf, n, 23U));

	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_enc_pps(NULL, g_buf, sizeof(g_buf)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_enc_pps(&p, NULL, sizeof(g_buf)));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, mp_enc_pps(&p, g_buf, 8U));
}

/* ================================================================= log === */

static void test_log_round_trip(void)
{
	logr_rec_t recs[3];
	mp_cbor_rd_t r;
	size_t n;
	size_t cnt = 0U;
	uint16_t i;
	int len;

	memset(recs, 0, sizeof(recs));
	for (i = 0U; i < 3U; i++) {
		recs[i].seq = 100U + i;
		recs[i].mono_ms = 5000U + i;
		recs[i].level = (uint8_t)LOGR_WARN;
		recs[i].subsys = (uint8_t)LOGR_SUB_PWR;
		recs[i].len = 5U;
		memcpy(recs[i].msg, "line", 4U);
		recs[i].msg[4] = (char)('0' + (char)i);
	}

	len = mp_enc_log(9U, 6000U, recs, 3U, 103U, 7U, g_buf, sizeof(g_buf));
	TEST_ASSERT_TRUE(len > 0);
	n = (size_t)len;

	TEST_ASSERT_EQUAL_UINT64(MP_REC_LOG, get_u(g_buf, n, MP_K_TYPE));
	TEST_ASSERT_EQUAL_UINT64(9U, get_u(g_buf, n, MP_K_SEQ));
	TEST_ASSERT_EQUAL_UINT64(103U, get_u(g_buf, n, 8U));
	TEST_ASSERT_EQUAL_UINT64(7U, get_u(g_buf, n, 9U));

	seek(&r, g_buf, n, 10U);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
	TEST_ASSERT_EQUAL_size_t(3U, cnt);
	for (i = 0U; i < 3U; i++) {
		size_t f = 0U;
		uint64_t u = 0U;
		const char *tp = NULL;
		size_t tn = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &f));
		TEST_ASSERT_EQUAL_size_t(5U, f);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(recs[i].seq, u);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(recs[i].mono_ms, u);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(recs[i].level, u);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(recs[i].subsys, u);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_text(&r, &tp, &tn));
		TEST_ASSERT_EQUAL_size_t(5U, tn);
		TEST_ASSERT_EQUAL_UINT8_ARRAY(recs[i].msg, tp, 5U);
	}

	/* An empty batch is still a valid record. */
	len = mp_enc_log(1U, 1U, NULL, 0U, 0U, 0U, g_buf, sizeof(g_buf));
	TEST_ASSERT_TRUE(len > 0);

	/* An over-long `len` field is clamped, not trusted. */
	recs[0].len = 200U;
	len = mp_enc_log(1U, 1U, recs, 1U, 0U, 0U, g_buf, sizeof(g_buf));
	TEST_ASSERT_TRUE(len > 0);
	seek(&r, g_buf, (size_t)len, 10U);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
	{
		size_t f = 0U;
		uint64_t u = 0U;
		const char *tp = NULL;
		size_t tn = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &f));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_text(&r, &tp, &tn));
		TEST_ASSERT_EQUAL_size_t(LOGR_MSG_MAX, tn);
	}

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_enc_log(1U, 1U, recs, 1U, 0U, 0U, NULL, 16U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_enc_log(1U, 1U, NULL, 1U, 0U, 0U,
						  g_buf, sizeof(g_buf)));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      mp_enc_log(1U, 1U, recs, 3U, 0U, 0U, g_buf, 12U));
}

/* ============================================================== events === */

static void test_event_kind_names(void)
{
	uint8_t i;

	for (i = 0U; i < (uint8_t)MP_EV_KIND_COUNT; i++) {
		TEST_ASSERT_TRUE(strlen(mp_ev_kind_name(i)) > 0U);
		TEST_ASSERT_TRUE(strcmp(mp_ev_kind_name(i), "unknown") != 0);
	}
	TEST_ASSERT_EQUAL_STRING("unknown", mp_ev_kind_name(MP_EV_KIND_COUNT));
}

static void test_event_queue_and_drain(void)
{
	mp_cbor_rd_t r;
	size_t cnt = 0U;
	uint16_t i;
	int len;

	TEST_ASSERT_EQUAL_size_t(0U, mp_stream_event_count(&g_st));
	TEST_ASSERT_EQUAL_INT(0, mp_enc_events(&g_st, 1U, 1U, 0U, g_buf,
					       sizeof(g_buf)));

	for (i = 0U; i < 5U; i++) {
		TEST_ASSERT_EQUAL_INT(0,
				      mp_stream_eventf(&g_st,
						       (uint8_t)MP_EV_FAULT,
						       2U, (uint16_t)(20U + i),
						       1U, (int32_t)i,
						       1000U + i, "pg drop"));
	}
	TEST_ASSERT_EQUAL_size_t(5U, mp_stream_event_count(&g_st));

	len = mp_enc_events(&g_st, 3U, 2000U, 0U, g_buf, sizeof(g_buf));
	TEST_ASSERT_TRUE(len > 0);
	TEST_ASSERT_EQUAL_size_t(0U, mp_stream_event_count(&g_st));

	TEST_ASSERT_EQUAL_UINT64(MP_REC_EVENT,
				 get_u(g_buf, (size_t)len, MP_K_TYPE));
	TEST_ASSERT_EQUAL_UINT64(0U, get_u(g_buf, (size_t)len, 8U));

	seek(&r, g_buf, (size_t)len, 9U);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
	TEST_ASSERT_EQUAL_size_t(5U, cnt);
	for (i = 0U; i < 5U; i++) {
		size_t f = 0U;
		uint64_t u = 0U;
		int64_t v = 0;
		const char *tp = NULL;
		size_t tn = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &f));
		TEST_ASSERT_EQUAL_size_t(7U, f);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64((uint64_t)MP_EV_FAULT, u);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(2U, u);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(20U + i, u);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(1U, u);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_int(&r, &v));
		TEST_ASSERT_EQUAL_INT64(i, v);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_UINT64(1000U + i, u);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_text(&r, &tp, &tn));
		TEST_ASSERT_EQUAL_size_t(7U, tn);
		TEST_ASSERT_EQUAL_UINT8_ARRAY("pg drop", tp, 7U);
	}
}

/*
 * Loss that happened BEFORE the queue still has to reach the host.
 *
 * The glue stages platform events (the 1 kHz scan, the alarm table) in a queue
 * of its own because their producers may not take the engine lock, and that
 * queue can overflow too. Key 8 is the only field on the wire that says a gap
 * happened, so the drain folds its own losses into this counter — otherwise a
 * technician sees a gap indistinguishable from a quiet board, which is the
 * failure the whole channel exists to prevent.
 */
static void test_drop_note_reaches_the_wire(void)
{
	int len;

	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_stream_event_drop_note(NULL, 3U));
	/* Nothing lost is not an event. */
	TEST_ASSERT_EQUAL_INT(0, mp_stream_event_drop_note(&g_st, 0U));
	TEST_ASSERT_EQUAL_UINT32(0U, mp_stream_event_dropped(&g_st));

	TEST_ASSERT_EQUAL_INT(0, mp_stream_event_drop_note(&g_st, 3U));
	TEST_ASSERT_EQUAL_INT(0, mp_stream_event_drop_note(&g_st, 4U));
	TEST_ASSERT_EQUAL_UINT32(7U, mp_stream_event_dropped(&g_st));

	/* Saturating, not wrapping: "were events lost" must not answer 0 at the
	 * moment it matters most. */
	TEST_ASSERT_EQUAL_INT(0, mp_stream_event_drop_note(&g_st, UINT32_MAX));
	TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, mp_stream_event_dropped(&g_st));

	/* And it is published with the batch, then cleared with it. */
	g_st.evq_dropped = 5U;
	TEST_ASSERT_EQUAL_INT(0, mp_stream_eventf(&g_st, (uint8_t)MP_EV_FAULT,
						  0U, 1U, 1U, 0, 10U, NULL));
	len = mp_enc_events(&g_st, 1U, 1U, 0U, g_buf, sizeof(g_buf));
	TEST_ASSERT_TRUE(len > 0);
	TEST_ASSERT_EQUAL_UINT64(5U, get_u(g_buf, (size_t)len, 8U));
	TEST_ASSERT_EQUAL_UINT32(0U, mp_stream_event_dropped(&g_st));
}

static void test_event_queue_drops_the_newest(void)
{
	uint16_t i;

	for (i = 0U; i < MP_EVQ_LEN; i++) {
		TEST_ASSERT_EQUAL_INT(0,
				      mp_stream_eventf(&g_st,
						       (uint8_t)MP_EV_ALARM, 0U,
						       i, 1U, 0, 0U, NULL));
	}
	TEST_ASSERT_EQUAL_size_t(MP_EVQ_LEN, mp_stream_event_count(&g_st));

	/* Full: the new event is refused and counted, the head is kept — the
	 * root cause of a cascade survives, the consequences do not. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      mp_stream_eventf(&g_st, (uint8_t)MP_EV_ALARM, 0U,
					       999U, 1U, 0, 0U, NULL));
	TEST_ASSERT_EQUAL_UINT32(1U, mp_stream_event_dropped(&g_st));
	TEST_ASSERT_EQUAL_size_t(MP_EVQ_LEN, mp_stream_event_count(&g_st));

	/* The oldest is still first out. */
	{
		mp_cbor_rd_t r;
		size_t cnt = 0U;
		size_t f = 0U;
		uint64_t u = 0U;
		int len = mp_enc_events(&g_st, 1U, 1U, 1U, g_buf,
					sizeof(g_buf));

		TEST_ASSERT_TRUE(len > 0);
		/* The drop counter is published with the batch. */
		TEST_ASSERT_EQUAL_UINT64(1U, get_u(g_buf, (size_t)len, 8U));
		seek(&r, g_buf, (size_t)len, 9U);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
		TEST_ASSERT_EQUAL_size_t(1U, cnt);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &f));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u)); /* kind */
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u)); /* sub */
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u)); /* id */
		TEST_ASSERT_EQUAL_UINT64(0U, u);
	}
	TEST_ASSERT_EQUAL_size_t(MP_EVQ_LEN - 1U, mp_stream_event_count(&g_st));
}

static void test_event_batch_shrinks_to_fit(void)
{
	uint8_t small[48];
	uint16_t i;
	int len;
	size_t before;

	for (i = 0U; i < 12U; i++) {
		(void)mp_stream_eventf(&g_st, (uint8_t)MP_EV_BUTTON, 0U, i, 1U,
				       0, 0U, "a button was pressed here");
	}
	before = mp_stream_event_count(&g_st);
	TEST_ASSERT_EQUAL_size_t(12U, before);

	/* Too small for all twelve: the batch shrinks and the rest stay queued,
	 * so no event is consumed without being encoded. */
	len = mp_enc_events(&g_st, 1U, 1U, 0U, small, sizeof(small));
	TEST_ASSERT_TRUE(len > 0);
	TEST_ASSERT_TRUE((size_t)len <= sizeof(small));
	TEST_ASSERT_TRUE(mp_stream_event_count(&g_st) < before);
	TEST_ASSERT_TRUE(mp_stream_event_count(&g_st) > 0U);

	/* Drain the rest. */
	while (mp_stream_event_count(&g_st) != 0U) {
		int rc = mp_enc_events(&g_st, 1U, 1U, 0U, small, sizeof(small));

		TEST_ASSERT_TRUE(rc > 0);
	}

	/* A buffer that cannot hold even one event says so rather than looping. */
	(void)mp_stream_eventf(&g_st, (uint8_t)MP_EV_BUTTON, 0U, 0U, 1U, 0, 0U,
			       "x");
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      mp_enc_events(&g_st, 1U, 1U, 0U, small, 6U));
	TEST_ASSERT_EQUAL_size_t(1U, mp_stream_event_count(&g_st));
}

static void test_event_argument_validation(void)
{
	mp_ev_t ev;

	memset(&ev, 0, sizeof(ev));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_stream_event(NULL, &ev));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_stream_event(&g_st, NULL));
	ev.kind = (uint8_t)MP_EV_KIND_COUNT;
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_stream_event(&g_st, &ev));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_enc_events(NULL, 1U, 1U, 0U, g_buf, 64U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_enc_events(&g_st, 1U, 1U, 0U, NULL, 64U));
	TEST_ASSERT_EQUAL_size_t(0U, mp_stream_event_count(NULL));
	TEST_ASSERT_EQUAL_UINT32(0U, mp_stream_event_dropped(NULL));

	/* An over-long text is truncated, and stays NUL-terminated. */
	TEST_ASSERT_EQUAL_INT(0,
			      mp_stream_eventf(&g_st, (uint8_t)MP_EV_MODE, 0U,
					       0U, 1U, 0, 0U,
					       "0123456789012345678901234567890123456789"));
	{
		mp_cbor_rd_t r;
		size_t cnt = 0U;
		size_t f = 0U;
		uint64_t u = 0U;
		int64_t v = 0;
		const char *tp = NULL;
		size_t tn = 0U;
		int len = mp_enc_events(&g_st, 1U, 1U, 0U, g_buf,
					sizeof(g_buf));

		TEST_ASSERT_TRUE(len > 0);
		seek(&r, g_buf, (size_t)len, 9U);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &f));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_int(&r, &v));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_text(&r, &tp, &tn));
		TEST_ASSERT_EQUAL_size_t(MP_EV_TEXT_MAX - 1U, tn);
	}
}

/* ======================================================= subscriptions === */

static void test_subscribable_and_paced_channels(void)
{
	/* Control and the SMP tunnel are not subscriptions. */
	TEST_ASSERT_FALSE(mp_stream_subscribable(MP_CH_CONTROL));
	TEST_ASSERT_FALSE(mp_stream_subscribable(MP_CH_SMP));
	TEST_ASSERT_FALSE(mp_stream_subscribable(0x0BU));
	TEST_ASSERT_FALSE(mp_stream_subscribable(MP_CH_MAX));

	TEST_ASSERT_TRUE(mp_stream_subscribable(MP_CH_TELEMETRY));
	TEST_ASSERT_TRUE(mp_stream_subscribable(MP_CH_NMEA));
	TEST_ASSERT_TRUE(mp_stream_subscribable(MP_CH_UBX));
	TEST_ASSERT_TRUE(mp_stream_subscribable(MP_CH_LOG));
	TEST_ASSERT_TRUE(mp_stream_subscribable(MP_CH_PPS));
	TEST_ASSERT_TRUE(mp_stream_subscribable(MP_CH_GNSS_PASS));
	TEST_ASSERT_TRUE(mp_stream_subscribable(MP_CH_RB_PASS));
	TEST_ASSERT_TRUE(mp_stream_subscribable(MP_CH_EVENT));
	TEST_ASSERT_TRUE(mp_stream_subscribable(MP_CH_MIRROR));

	/* Only the periodic ones are paced. */
	TEST_ASSERT_TRUE(mp_stream_paced(MP_CH_TELEMETRY));
	TEST_ASSERT_TRUE(mp_stream_paced(MP_CH_PPS));
	TEST_ASSERT_TRUE(mp_stream_paced(MP_CH_LOG));
	TEST_ASSERT_TRUE(mp_stream_paced(MP_CH_MIRROR));
	TEST_ASSERT_FALSE(mp_stream_paced(MP_CH_EVENT));
	TEST_ASSERT_FALSE(mp_stream_paced(MP_CH_NMEA));
	TEST_ASSERT_FALSE(mp_stream_paced(MP_CH_CONTROL));
}

static void test_subscribe_unsubscribe(void)
{
	TEST_ASSERT_FALSE(mp_stream_is_sub(&g_st, MP_CH_TELEMETRY));
	TEST_ASSERT_EQUAL_INT(-ENOENT, mp_stream_unsub(&g_st, MP_CH_TELEMETRY));

	TEST_ASSERT_EQUAL_INT(0,
			      mp_stream_sub(&g_st, MP_CH_TELEMETRY, 4U, 1000U));
	TEST_ASSERT_TRUE(mp_stream_is_sub(&g_st, MP_CH_TELEMETRY));
	TEST_ASSERT_EQUAL_UINT8(4U, g_st.sub[MP_CH_TELEMETRY].rate_hz);

	/* Rate limits (FMT §7: 1..10 Hz). */
	TEST_ASSERT_EQUAL_INT(-ERANGE,
			      mp_stream_sub(&g_st, MP_CH_TELEMETRY, 11U, 1000U));
	TEST_ASSERT_EQUAL_INT(-ERANGE,
			      mp_stream_sub(&g_st, MP_CH_TELEMETRY, 255U,
					    1000U));
	/* 0 means "the slowest rate", not "unpaced". */
	TEST_ASSERT_EQUAL_INT(0,
			      mp_stream_sub(&g_st, MP_CH_TELEMETRY, 0U, 1000U));
	TEST_ASSERT_EQUAL_UINT8(1U, g_st.sub[MP_CH_TELEMETRY].rate_hz);

	/* An unpaced channel ignores the rate. */
	TEST_ASSERT_EQUAL_INT(0, mp_stream_sub(&g_st, MP_CH_EVENT, 99U, 1000U));
	TEST_ASSERT_EQUAL_UINT8(0U, g_st.sub[MP_CH_EVENT].rate_hz);

	TEST_ASSERT_EQUAL_INT(-ENOTSUP,
			      mp_stream_sub(&g_st, MP_CH_CONTROL, 1U, 1000U));
	TEST_ASSERT_EQUAL_INT(-ENOTSUP,
			      mp_stream_sub(&g_st, 0x40U, 1U, 1000U));
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, mp_stream_unsub(&g_st, 0x40U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_stream_sub(NULL, 1U, 1U, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_stream_unsub(NULL, 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_stream_init(NULL));
	TEST_ASSERT_FALSE(mp_stream_is_sub(NULL, 1U));

	mp_stream_unsub_all(&g_st);
	TEST_ASSERT_FALSE(mp_stream_is_sub(&g_st, MP_CH_TELEMETRY));
	TEST_ASSERT_FALSE(mp_stream_is_sub(&g_st, MP_CH_EVENT));
	mp_stream_unsub_all(NULL); /* must not fault */
}

static void test_pacing(void)
{
	TEST_ASSERT_FALSE(mp_stream_due(&g_st, MP_CH_TELEMETRY, 1000U));
	TEST_ASSERT_FALSE(mp_stream_due(NULL, MP_CH_TELEMETRY, 1000U));
	TEST_ASSERT_FALSE(mp_stream_due(&g_st, 0x40U, 1000U));

	/* 4 Hz = a 250 ms period. */
	TEST_ASSERT_EQUAL_INT(0,
			      mp_stream_sub(&g_st, MP_CH_TELEMETRY, 4U, 1000U));
	TEST_ASSERT_TRUE(mp_stream_due(&g_st, MP_CH_TELEMETRY, 1000U));
	TEST_ASSERT_FALSE(mp_stream_due(&g_st, MP_CH_TELEMETRY, 1100U));
	TEST_ASSERT_FALSE(mp_stream_due(&g_st, MP_CH_TELEMETRY, 1249U));
	TEST_ASSERT_TRUE(mp_stream_due(&g_st, MP_CH_TELEMETRY, 1250U));
	TEST_ASSERT_TRUE(mp_stream_due(&g_st, MP_CH_TELEMETRY, 1500U));
	TEST_ASSERT_FALSE(mp_stream_due(&g_st, MP_CH_TELEMETRY, 1600U));

	/* The schedule advances from the deadline, so a slightly late poll does
	 * not slow the stream. */
	TEST_ASSERT_TRUE(mp_stream_due(&g_st, MP_CH_TELEMETRY, 1760U));
	TEST_ASSERT_FALSE(mp_stream_due(&g_st, MP_CH_TELEMETRY, 1900U));

	/* More than a whole period late: resynchronise instead of bursting. */
	TEST_ASSERT_TRUE(mp_stream_due(&g_st, MP_CH_TELEMETRY, 9000U));
	TEST_ASSERT_FALSE(mp_stream_due(&g_st, MP_CH_TELEMETRY, 9100U));
	TEST_ASSERT_TRUE(mp_stream_due(&g_st, MP_CH_TELEMETRY, 9250U));

	/* An unpaced channel is never "due" — it is event-driven. */
	TEST_ASSERT_EQUAL_INT(0, mp_stream_sub(&g_st, MP_CH_EVENT, 0U, 1000U));
	TEST_ASSERT_FALSE(mp_stream_due(&g_st, MP_CH_EVENT, 99999U));

	/* At 10 Hz the period is 100 ms. */
	TEST_ASSERT_EQUAL_INT(0, mp_stream_sub(&g_st, MP_CH_PPS, 10U, 0U));
	TEST_ASSERT_TRUE(mp_stream_due(&g_st, MP_CH_PPS, 0U));
	TEST_ASSERT_FALSE(mp_stream_due(&g_st, MP_CH_PPS, 99U));
	TEST_ASSERT_TRUE(mp_stream_due(&g_st, MP_CH_PPS, 100U));
}

static void test_sequence_and_cursor(void)
{
	TEST_ASSERT_EQUAL_UINT32(1U,
				 mp_stream_next_seq(&g_st, MP_CH_TELEMETRY));
	TEST_ASSERT_EQUAL_UINT32(2U,
				 mp_stream_next_seq(&g_st, MP_CH_TELEMETRY));
	/* Per-channel, not global. */
	TEST_ASSERT_EQUAL_UINT32(1U, mp_stream_next_seq(&g_st, MP_CH_PPS));
	TEST_ASSERT_EQUAL_UINT32(0U, mp_stream_next_seq(NULL, MP_CH_PPS));
	TEST_ASSERT_EQUAL_UINT32(0U, mp_stream_next_seq(&g_st, 0x40U));

	TEST_ASSERT_EQUAL_UINT32(0U, mp_stream_cursor(&g_st, MP_CH_LOG));
	mp_stream_set_cursor(&g_st, MP_CH_LOG, 4242U);
	TEST_ASSERT_EQUAL_UINT32(4242U, mp_stream_cursor(&g_st, MP_CH_LOG));
	TEST_ASSERT_EQUAL_UINT32(0U, mp_stream_cursor(NULL, MP_CH_LOG));
	TEST_ASSERT_EQUAL_UINT32(0U, mp_stream_cursor(&g_st, 0x40U));
	mp_stream_set_cursor(NULL, MP_CH_LOG, 1U);
	mp_stream_set_cursor(&g_st, 0x40U, 1U);
}

/* ============================================== §8.1 support bundle ====== */

static void test_bundle_round_trip(void)
{
	mp_bundle_t b;
	mp_telem_t t;
	logr_rec_t log[2];
	uint8_t i2c[16];
	uint32_t hist[8] = { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U };
	uint8_t navsat[12];
	uint8_t cfg[24];
	mp_cbor_rd_t r;
	size_t n;
	int len;

	fill_telem(&t);
	memset(log, 0, sizeof(log));
	log[0].seq = 1U;
	log[0].len = 3U;
	memcpy(log[0].msg, "abc", 3U);
	log[1].seq = 2U;
	log[1].len = 3U;
	memcpy(log[1].msg, "def", 3U);
	memset(i2c, 0xA5, sizeof(i2c));
	memset(navsat, 0x5A, sizeof(navsat));
	memset(cfg, 0x33, sizeof(cfg));

	memset(&b, 0, sizeof(b));
	b.telem = &t;
	b.fw_version = "1.2.3";
	b.boot_version = "0.9.0";
	b.board_id = "abcd";
	b.serial = "STS1000-1";
	b.fault_latched = 0x1234ULL;
	b.fault_active = 0x0012ULL;
	b.i2c_scan = i2c;
	b.pps_hist = hist;
	b.pps_hist_n = 8U;
	b.pps_hist_bin_ns = 5U;
	b.log = log;
	b.log_n = 2U;
	b.navsat = navsat;
	b.navsat_n = sizeof(navsat);
	b.cfg_tlv = cfg;
	b.cfg_tlv_n = sizeof(cfg);
	b.manifest_hash = 0xDEADBEEFU;
	b.notes = "bench";

	len = mp_enc_bundle(&b, 5U, 6000U, g_buf, sizeof(g_buf));
	TEST_ASSERT_TRUE(len > 0);
	n = (size_t)len;

	TEST_ASSERT_EQUAL_UINT64(MP_REC_BUNDLE, get_u(g_buf, n, MP_K_TYPE));
	TEST_ASSERT_EQUAL_UINT64(5U, get_u(g_buf, n, MP_K_SEQ));
	TEST_ASSERT_EQUAL_UINT64(b.fault_latched, get_u(g_buf, n, 11U));
	TEST_ASSERT_EQUAL_UINT64(b.fault_active, get_u(g_buf, n, 12U));
	TEST_ASSERT_EQUAL_UINT64(b.pps_hist_bin_ns, get_u(g_buf, n, 15U));
	TEST_ASSERT_EQUAL_UINT64(b.manifest_hash, get_u(g_buf, n, 19U));

	/* The nested telemetry record is a self-contained CBOR document. */
	seek(&r, g_buf, n, 8U);
	{
		const uint8_t *p = NULL;
		size_t pn = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_bytes(&r, &p, &pn));
		TEST_ASSERT_TRUE(pn > 100U);
		TEST_ASSERT_EQUAL_UINT64(MP_REC_TELEM,
					 get_u(p, pn, MP_K_TYPE));
		TEST_ASSERT_EQUAL_UINT64(t.tai_ns, get_u(p, pn, 8U));
	}

	/* Versions, i2c map, histogram, log and blobs. */
	seek(&r, g_buf, n, 9U);
	{
		size_t cnt = 0U;
		const char *tp = NULL;
		size_t tn = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
		TEST_ASSERT_EQUAL_size_t(3U, cnt);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_text(&r, &tp, &tn));
		TEST_ASSERT_EQUAL_UINT8_ARRAY("1.2.3", tp, 5U);
	}
	seek(&r, g_buf, n, 13U);
	{
		const uint8_t *p = NULL;
		size_t pn = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_bytes(&r, &p, &pn));
		TEST_ASSERT_EQUAL_size_t(16U, pn);
		TEST_ASSERT_EQUAL_UINT8_ARRAY(i2c, p, 16U);
	}
	seek(&r, g_buf, n, 14U);
	{
		size_t cnt = 0U;
		size_t k;
		uint64_t u = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
		TEST_ASSERT_EQUAL_size_t(8U, cnt);
		for (k = 0U; k < 8U; k++) {
			TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
			TEST_ASSERT_EQUAL_UINT64(hist[k], u);
		}
	}
	seek(&r, g_buf, n, 16U);
	{
		size_t cnt = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
		TEST_ASSERT_EQUAL_size_t(2U, cnt);
	}
	seek(&r, g_buf, n, 17U);
	{
		const uint8_t *p = NULL;
		size_t pn = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_bytes(&r, &p, &pn));
		TEST_ASSERT_EQUAL_size_t(sizeof(navsat), pn);
	}
	seek(&r, g_buf, n, 20U);
	{
		const char *tp = NULL;
		size_t tn = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_text(&r, &tp, &tn));
		TEST_ASSERT_EQUAL_UINT8_ARRAY("bench", tp, 5U);
	}

	/* The whole record is one well-formed item. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, n));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_skip(&r));
	TEST_ASSERT_EQUAL_INT(-ENODATA, mp_cbor_peek_major(&r));
}

static void test_bundle_from_a_broken_board_is_still_valid(void)
{
	mp_bundle_t b;
	mp_cbor_rd_t r;
	size_t n;
	int len;

	/* Everything absent: a board with a dead NOR and no GNSS must still
	 * produce a bundle a host can parse. */
	memset(&b, 0, sizeof(b));
	len = mp_enc_bundle(&b, 1U, 1U, g_buf, sizeof(g_buf));
	TEST_ASSERT_TRUE(len > 0);
	n = (size_t)len;

	TEST_ASSERT_EQUAL_UINT64(MP_REC_BUNDLE, get_u(g_buf, n, MP_K_TYPE));
	seek(&r, g_buf, n, 8U);
	TEST_ASSERT_EQUAL_INT(7, mp_cbor_peek_major(&r)); /* null */

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, n));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_skip(&r));
	TEST_ASSERT_EQUAL_INT(-ENODATA, mp_cbor_peek_major(&r));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_enc_bundle(NULL, 1U, 1U, g_buf,
					    sizeof(g_buf)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_enc_bundle(&b, 1U, 1U, NULL, 64U));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, mp_enc_bundle(&b, 1U, 1U, g_buf, 4U));
}

static void test_bundle_reports_a_telemetry_encode_failure(void)
{
	mp_bundle_t b;
	mp_telem_t t;

	fill_telem(&t);
	memset(&b, 0, sizeof(b));
	b.telem = &t;

	/* A buffer big enough for the bundle head but not the nested record
	 * still fails cleanly rather than emitting a truncated blob. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      mp_enc_bundle(&b, 1U, 1U, g_buf, 64U));
}

/* --------------------------------------------------------------- fuzzing */

/** Random bytes into the reader: it must terminate and stay in bounds. */
static void test_fuzz_cbor_reader(void)
{
	test_rng_t rng;
	unsigned int round;

	test_rng_init(&rng, 0xCB0FU);

	for (round = 0U; round < 20000U; round++) {
		uint8_t buf[64];
		size_t n = test_rng_below(&rng, sizeof(buf)) + 1U;
		mp_cbor_rd_t r;
		uint64_t u = 0U;
		int64_t i = 0;
		size_t sz = 0U;
		bool b = false;
		float f = 0.0f;
		const uint8_t *p = NULL;
		const char *tp = NULL;

		test_rng_fill(&rng, buf, n);

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, buf, n));
		(void)mp_cbor_peek_major(&r);
		(void)mp_cbor_skip(&r);
		TEST_ASSERT_TRUE(r.pos <= r.len);

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, buf, n));
		(void)mp_cbor_map_find(&r, 0U);
		TEST_ASSERT_TRUE(r.pos <= r.len);

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, buf, n));
		(void)mp_cbor_rd_uint(&r, &u);
		(void)mp_cbor_rd_int(&r, &i);
		(void)mp_cbor_rd_map(&r, &sz);
		(void)mp_cbor_rd_arr(&r, &sz);
		(void)mp_cbor_rd_bool(&r, &b);
		(void)mp_cbor_rd_f32(&r, &f);
		if (mp_cbor_rd_bytes(&r, &p, &sz) == 0) {
			/* A successful read must point inside the buffer. */
			TEST_ASSERT_TRUE(p >= buf);
			TEST_ASSERT_TRUE((size_t)(p - buf) + sz <= n);
		}
		if (mp_cbor_rd_text(&r, &tp, &sz) == 0) {
			TEST_ASSERT_TRUE((const uint8_t *)tp >= buf);
			TEST_ASSERT_TRUE((size_t)((const uint8_t *)tp - buf) +
						 sz <=
					 n);
		}
		TEST_ASSERT_TRUE(r.pos <= r.len);
	}
}

/** Encode a record into every buffer size: it either fits or says -ENOSPC. */
static void test_encoders_never_overrun_a_short_buffer(void)
{
	mp_telem_t t;
	mp_pps_t p;
	size_t cap;

	fill_telem(&t);
	memset(&p, 0, sizeof(p));
	p.pc6_valid = true;

	for (cap = 0U; cap <= MP_TELEM_MAX; cap++) {
		uint8_t scratch[MP_TELEM_MAX + 16U];
		int rc;

		memset(scratch, 0xEE, sizeof(scratch));
		rc = mp_enc_telem(&t, scratch, cap);
		TEST_ASSERT_TRUE((rc >= 0) || (rc == -ENOSPC));
		if (rc >= 0) {
			TEST_ASSERT_TRUE((size_t)rc <= cap);
		}
		/* Nothing was written past `cap`. */
		TEST_ASSERT_EQUAL_UINT8(0xEEU, scratch[cap]);

		if (cap <= MP_PPS_MAX) {
			memset(scratch, 0xEE, sizeof(scratch));
			rc = mp_enc_pps(&p, scratch, cap);
			TEST_ASSERT_TRUE((rc >= 0) || (rc == -ENOSPC));
			TEST_ASSERT_EQUAL_UINT8(0xEEU, scratch[cap]);
		}
	}
}

/* ------------------------------------------------------------------- runner */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_cbor_uint_vectors);
	RUN_TEST(test_cbor_int_vectors);
	RUN_TEST(test_cbor_other_vectors);
	RUN_TEST(test_cbor_kv_helpers);
	RUN_TEST(test_cbor_writer_overflow);
	RUN_TEST(test_cbor_writer_argument_validation);

	RUN_TEST(test_cbor_reader_rejections);
	RUN_TEST(test_cbor_skip_walks_containers);
	RUN_TEST(test_cbor_skip_depth_is_bounded);

	RUN_TEST(test_telemetry_round_trip);
	RUN_TEST(test_telemetry_errors);
	RUN_TEST(test_pps_round_trip);
	RUN_TEST(test_log_round_trip);

	RUN_TEST(test_event_kind_names);
	RUN_TEST(test_event_queue_and_drain);
	RUN_TEST(test_event_queue_drops_the_newest);
	RUN_TEST(test_drop_note_reaches_the_wire);
	RUN_TEST(test_event_batch_shrinks_to_fit);
	RUN_TEST(test_event_argument_validation);

	RUN_TEST(test_subscribable_and_paced_channels);
	RUN_TEST(test_subscribe_unsubscribe);
	RUN_TEST(test_pacing);
	RUN_TEST(test_sequence_and_cursor);

	RUN_TEST(test_bundle_round_trip);
	RUN_TEST(test_bundle_from_a_broken_board_is_still_valid);
	RUN_TEST(test_bundle_reports_a_telemetry_encode_failure);

	RUN_TEST(test_fuzz_cbor_reader);
	RUN_TEST(test_encoders_never_overrun_a_short_buffer);

	return UNITY_END();
}
