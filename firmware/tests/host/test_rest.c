/*
 * STS1000 "Meridian" — core/web rest + web (JSON/codec) unit tests.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Drives the real router end to end: a raw HTTP request goes through
 * http_parse_request() and rest_dispatch(), and the assertions are made on the
 * JSON that comes back. Fake providers stand in for the platform, so what is
 * under test is the routing, the authorisation, the CSRF gate, the audit trail
 * and the encoders — not a mock of them.
 *
 * The security invariants from rest.h each get a dedicated case:
 *   test_secret_never_served     invariant 2
 *   test_role_enforcement        invariant 1 (role)
 *   test_csrf_enforcement        invariant 1 (CSRF), including auth-off
 *   test_audit_records           invariant 3
 *   test_missing_providers       invariant 4
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "cfg/cfg.h"
#include "logring/logring.h"
#include "quality/quality.h"
#include "web/auth_web.h"
#include "web/http_parse.h"
#include "web/rest.h"
#include "web/web.h"
#include "web/wss.h"
/*
 * The net area's response-buffer size, so the worst-case telemetry frame is
 * measured against the buffer sts_web.c actually serves it from rather than
 * against a comfortable test-local one. Header-only; nothing is linked from the
 * glue layer. test_ldap_ca.c reaches for the same header for the same reason.
 */
#include "zephyr/net/sts_web.h"
/*
 * PTP_PROFILE_DEVIATION_TEXT_MAX only — this suite links core/web, not core/ptp,
 * and references no ptp symbol. Type-only includes across a module boundary are
 * the established pattern here; see the `mp` note in tests/host/CMakeLists.txt.
 */
#include "ptp/ptp_profile.h"

#include "host_aes.h"
#include "test_support.h"

/* memmem() is a GNU extension; the harness builds with -std=c11 (no
 * _GNU_SOURCE), so the binary-needle search lives here. */
static const uint8_t *mem_find(const void *hay, size_t hn, const void *needle,
			       size_t nn)
{
	const uint8_t *h = hay;
	size_t i;

	if (nn == 0U || hn < nn) {
		return NULL;
	}
	for (i = 0U; i + nn <= hn; i++) {
		if (memcmp(&h[i], needle, nn) == 0) {
			return &h[i];
		}
	}
	return NULL;
}

/* ========================================================================= */
/* part 1: the JSON writer / reader and the small codecs (web.c)             */
/* ========================================================================= */

static void test_jw_basics(void)
{
	web_jw_t w;
	char buf[256];
	size_t len = 0U;

	web_jw_init(&w, buf, sizeof(buf));
	web_jw_obj_begin(&w);
	web_jw_kstr(&w, "s", "hi");
	web_jw_ki64(&w, "i", -42);
	web_jw_ku64(&w, "u", 18446744073709551615ULL);
	web_jw_kbool(&w, "t", true);
	web_jw_kbool(&w, "f", false);
	web_jw_knull(&w, "n");
	web_jw_kfixed(&w, "fx", -12345, 3U);
	web_jw_karr(&w, "a");
	web_jw_i64(&w, 1);
	web_jw_i64(&w, 2);
	web_jw_obj_begin(&w);
	web_jw_kstr(&w, "k", "v");
	web_jw_obj_end(&w);
	web_jw_arr_end(&w);
	web_jw_kobj(&w, "o");
	web_jw_khex(&w, "h", (const uint8_t *)"\x01\xAB", 2U);
	web_jw_obj_end(&w);
	web_jw_obj_end(&w);
	TEST_ASSERT_EQUAL_INT(0, web_jw_finish(&w, &len));
	TEST_ASSERT_EQUAL_STRING("{\"s\":\"hi\",\"i\":-42,"
				 "\"u\":18446744073709551615,\"t\":true,"
				 "\"f\":false,\"n\":null,\"fx\":-12.345,"
				 "\"a\":[1,2,{\"k\":\"v\"}],"
				 "\"o\":{\"h\":\"01ab\"}}",
				 buf);
	TEST_ASSERT_EQUAL_UINT(strlen(buf), len);
}

static void test_jw_escaping(void)
{
	web_jw_t w;
	char buf[192];

	web_jw_init(&w, buf, sizeof(buf));
	web_jw_obj_begin(&w);
	web_jw_kstr(&w, "q", "a\"b\\c\nd\re\tf\bg\fh");
	web_jw_kstrn(&w, "ctl", "x\x01y\x7F", 4U);
	web_jw_kstr(&w, "null_in", NULL);
	web_jw_obj_end(&w);
	TEST_ASSERT_EQUAL_INT(0, web_jw_finish(&w, NULL));
	TEST_ASSERT_EQUAL_STRING("{\"q\":\"a\\\"b\\\\c\\nd\\re\\tf\\bg\\fh\","
				 "\"ctl\":\"x\\u0001y\\u007f\","
				 "\"null_in\":\"\"}",
				 buf);
}

static void test_jw_numbers(void)
{
	web_jw_t w;
	char buf[128];

	/* INT64_MIN must not be negated in signed space. */
	web_jw_init(&w, buf, sizeof(buf));
	web_jw_i64(&w, INT64_MIN);
	TEST_ASSERT_EQUAL_INT(0, web_jw_finish(&w, NULL));
	TEST_ASSERT_EQUAL_STRING("-9223372036854775808", buf);

	/* Fixed-point: zero-padded fraction, decimals == 0, clamping at 9. */
	web_jw_init(&w, buf, sizeof(buf));
	web_jw_arr_begin(&w);
	web_jw_fixed(&w, 5, 3U);
	web_jw_fixed(&w, -5, 3U);
	web_jw_fixed(&w, 1000, 3U);
	web_jw_fixed(&w, 7, 0U);
	web_jw_fixed(&w, 1, 12U);
	web_jw_arr_end(&w);
	TEST_ASSERT_EQUAL_INT(0, web_jw_finish(&w, NULL));
	TEST_ASSERT_EQUAL_STRING("[0.005,-0.005,1.000,7,0.000000001]", buf);

	/* Floats go through the same integer path. */
	web_jw_init(&w, buf, sizeof(buf));
	web_jw_arr_begin(&w);
	web_jw_f32(&w, 1.5f, 3U);
	/* 0.125 and -0.125 are exact in binary32, so they pin the
	 * round-half-away-from-zero rule without a representation wobble. */
	web_jw_f32(&w, 0.125f, 2U);
	web_jw_f32(&w, -0.125f, 2U);
	web_jw_f32(&w, 0.0f, 2U);
	web_jw_f32(&w, 1.0f / 0.0f, 2U);  /* +inf -> null */
	web_jw_f32(&w, -1.0f / 0.0f, 2U); /* -inf -> null */
	web_jw_f32(&w, 0.0f / 0.0f, 2U);  /* NaN  -> null */
	web_jw_f32(&w, 1.0f, 12U);
	web_jw_arr_end(&w);
	TEST_ASSERT_EQUAL_INT(0, web_jw_finish(&w, NULL));
	TEST_ASSERT_EQUAL_STRING("[1.500,0.13,-0.13,0.00,null,null,null,"
				 "1.000000000]",
				 buf);
}

static void test_jw_errors(void)
{
	web_jw_t w;
	char buf[16];
	size_t len = 0U;
	unsigned int i;

	/* Overflow is reported, never silently truncated into valid JSON. */
	web_jw_init(&w, buf, sizeof(buf));
	web_jw_obj_begin(&w);
	web_jw_kstr(&w, "key", "a very long value indeed");
	web_jw_obj_end(&w);
	TEST_ASSERT_EQUAL_INT(-ENOSPC, web_jw_finish(&w, &len));
	TEST_ASSERT_TRUE(len < sizeof(buf));

	/* Unbalanced. */
	web_jw_init(&w, buf, sizeof(buf));
	web_jw_obj_begin(&w);
	TEST_ASSERT_EQUAL_INT(-EPROTO, web_jw_finish(&w, NULL));

	web_jw_init(&w, buf, sizeof(buf));
	web_jw_obj_end(&w);
	TEST_ASSERT_EQUAL_INT(-EPROTO, web_jw_finish(&w, NULL));

	/* A key with no value. */
	web_jw_init(&w, buf, sizeof(buf));
	web_jw_obj_begin(&w);
	web_jw_key(&w, "k");
	web_jw_obj_end(&w);
	TEST_ASSERT_EQUAL_INT(-EPROTO, web_jw_finish(&w, NULL));

	/* Two keys in a row. */
	web_jw_init(&w, buf, sizeof(buf));
	web_jw_obj_begin(&w);
	web_jw_key(&w, "a");
	web_jw_key(&w, "b");
	TEST_ASSERT_EQUAL_INT(-EPROTO, web_jw_finish(&w, NULL));

	/* A NULL key. */
	web_jw_init(&w, buf, sizeof(buf));
	web_jw_key(&w, NULL);
	TEST_ASSERT_EQUAL_INT(-EPROTO, web_jw_finish(&w, NULL));

	/* Too deep. */
	web_jw_init(&w, buf, sizeof(buf));
	for (i = 0U; i <= WEB_JSON_DEPTH_MAX; i++) {
		web_jw_arr_begin(&w);
	}
	TEST_ASSERT_EQUAL_INT(-EPROTO, web_jw_finish(&w, NULL));

	/* Bad init. */
	web_jw_init(NULL, buf, sizeof(buf));
	web_jw_init(&w, NULL, 4U);
	TEST_ASSERT_EQUAL_INT(-EPROTO, web_jw_finish(&w, NULL));
	web_jw_init(&w, buf, 0U);
	TEST_ASSERT_EQUAL_INT(-EPROTO, web_jw_finish(&w, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_jw_finish(NULL, NULL));
}

static void test_json_read(void)
{
	static const char *const doc =
		"{ \"a\" : 1 , \"b\":\"str\", \"c\":true, \"d\":false,"
		"\"e\":null, \"f\":[1,2,3], \"g\":{\"h\":-7},"
		"\"i\":\"esc\\n\\\"\\u0041\\u00e9\\u20ac\" }";
	web_json_val_t v;
	web_json_val_t k;
	int64_t s;
	uint64_t u;
	bool b;
	char out[32];
	size_t cur;

	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get(doc, strlen(doc), "a", &v));
	TEST_ASSERT_EQUAL_UINT(WEB_JSON_NUM, v.type);
	TEST_ASSERT_EQUAL_INT(0, web_json_i64(&v, &s));
	TEST_ASSERT_EQUAL_INT64(1, s);
	TEST_ASSERT_EQUAL_INT(0, web_json_u64(&v, &u));
	TEST_ASSERT_EQUAL_UINT64(1U, u);

	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get(doc, strlen(doc), "b", &v));
	TEST_ASSERT_EQUAL_UINT(WEB_JSON_STR, v.type);
	TEST_ASSERT_TRUE(web_json_str_eq(&v, "str"));
	TEST_ASSERT_FALSE(web_json_str_eq(&v, "other"));
	TEST_ASSERT_EQUAL_INT(3, web_json_str_copy(&v, out, sizeof(out)));
	TEST_ASSERT_EQUAL_STRING("str", out);

	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get(doc, strlen(doc), "c", &v));
	TEST_ASSERT_EQUAL_INT(0, web_json_bool(&v, &b));
	TEST_ASSERT_TRUE(b);
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get(doc, strlen(doc), "d", &v));
	TEST_ASSERT_EQUAL_INT(0, web_json_bool(&v, &b));
	TEST_ASSERT_FALSE(b);

	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get(doc, strlen(doc), "e", &v));
	TEST_ASSERT_EQUAL_UINT(WEB_JSON_NULL, v.type);

	/* Array iteration. */
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get(doc, strlen(doc), "f", &v));
	TEST_ASSERT_EQUAL_UINT(WEB_JSON_ARR, v.type);
	cur = 0U;
	{
		web_json_val_t el;
		int64_t sum = 0;
		int n = 0;

		while (web_json_arr_next(&v, &cur, &el) == 1) {
			TEST_ASSERT_EQUAL_INT(0, web_json_i64(&el, &s));
			sum += s;
			n++;
		}
		TEST_ASSERT_EQUAL_INT(3, n);
		TEST_ASSERT_EQUAL_INT64(6, sum);
	}

	/* Nested object, then member iteration. */
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get(doc, strlen(doc), "g", &v));
	TEST_ASSERT_EQUAL_UINT(WEB_JSON_OBJ, v.type);
	{
		web_json_val_t iv;

		TEST_ASSERT_EQUAL_INT(0, web_json_obj_get(v.p, v.n, "h", &iv));
		TEST_ASSERT_EQUAL_INT(0, web_json_i64(&iv, &s));
		TEST_ASSERT_EQUAL_INT64(-7, s);
	}
	cur = 0U;
	TEST_ASSERT_EQUAL_INT(1, web_json_obj_next(&v, &cur, &k, &v));
	TEST_ASSERT_TRUE(web_json_str_eq(&k, "h"));

	/* Escapes, including a 1-, 2- and 3-byte \u form. */
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get(doc, strlen(doc), "i", &v));
	TEST_ASSERT_EQUAL_INT(11, web_json_str_copy(&v, out, sizeof(out)));
	TEST_ASSERT_EQUAL_STRING("esc\n\"A\xC3\xA9\xE2\x82\xAC", out);

	TEST_ASSERT_EQUAL_INT(-ENOENT, web_json_obj_get(doc, strlen(doc), "zz",
							&v));
}

static void test_json_read_errors(void)
{
	web_json_val_t v;
	web_json_val_t k;
	int64_t s;
	uint64_t u;
	bool b;
	char out[8];
	size_t cur;

	TEST_ASSERT_EQUAL_INT(-EINVAL, web_json_obj_get(NULL, 2U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_json_obj_get("{}", 2U, NULL, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_json_obj_get("{}", 2U, "a", NULL));
	TEST_ASSERT_EQUAL_INT(-ENOENT, web_json_obj_get("{}", 2U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, web_json_obj_get("[]", 2U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, web_json_obj_get("", 0U, "a", &v));
	/* Trailing comma. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      web_json_obj_get("{\"a\":1,}", 8U, "b", &v));
	/* Missing colon. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      web_json_obj_get("{\"a\" 1}", 7U, "b", &v));
	/* Unterminated string, object, array. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      web_json_obj_get("{\"a\":\"x}", 8U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      web_json_obj_get("{\"a\":{\"b\":1}", 12U, "z", &v));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      web_json_obj_get("{\"a\":[1,2}", 10U, "a", &v));
	/* Mismatched bracket. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      web_json_obj_get("{\"a\":[1}", 8U, "a", &v));
	/*
	 * A nested value is BALANCED, not validated, by the top-level scan: the
	 * reader is a bounded scanner, and `[1]` here is well-formed anyway, so
	 * the absent key is simply absent.
	 */
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      web_json_obj_get("{\"a\":[1]}", 9U, "z", &v));
	/* A raw control character inside a string. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      web_json_obj_get("{\"a\":\"x\ty\"}", 11U, "a", &v));
	/* Bad literals. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      web_json_obj_get("{\"a\":tru}", 9U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      web_json_obj_get("{\"a\":fals}", 10U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      web_json_obj_get("{\"a\":nul}", 9U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      web_json_obj_get("{\"a\":?}", 7U, "a", &v));
	/* A key that is not a string. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG, web_json_obj_get("{a:1}", 5U, "a", &v));

	/* Numeric conversions. */
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":1.5}", 9U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, web_json_i64(&v, &s));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, web_json_u64(&v, &u));
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":1e3}", 9U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, web_json_i64(&v, &s));
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":+1}", 8U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, web_json_i64(&v, &s));
	TEST_ASSERT_EQUAL_INT(-ERANGE, web_json_u64(&v, &u));
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":-1}", 8U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-ERANGE, web_json_u64(&v, &u));
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":-}", 7U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, web_json_i64(&v, &s));

	/* Range limits. */
	TEST_ASSERT_EQUAL_INT(0,
			      web_json_obj_get("{\"a\":9223372036854775807}",
					       25U, "a", &v));
	TEST_ASSERT_EQUAL_INT(0, web_json_i64(&v, &s));
	TEST_ASSERT_EQUAL_INT64(INT64_MAX, s);
	TEST_ASSERT_EQUAL_INT(0,
			      web_json_obj_get("{\"a\":-9223372036854775808}",
					       26U, "a", &v));
	TEST_ASSERT_EQUAL_INT(0, web_json_i64(&v, &s));
	TEST_ASSERT_EQUAL_INT64(INT64_MIN, s);
	TEST_ASSERT_EQUAL_INT(0,
			      web_json_obj_get("{\"a\":-9223372036854775809}",
					       26U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-ERANGE, web_json_i64(&v, &s));
	TEST_ASSERT_EQUAL_INT(0,
			      web_json_obj_get("{\"a\":9223372036854775808}",
					       25U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-ERANGE, web_json_i64(&v, &s));
	TEST_ASSERT_EQUAL_INT(0,
			      web_json_obj_get("{\"a\":18446744073709551616}",
					       26U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-ERANGE, web_json_u64(&v, &u));
	TEST_ASSERT_EQUAL_INT(0,
			      web_json_obj_get("{\"a\":99999999999999999999999}",
					       29U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-ERANGE, web_json_u64(&v, &u));

	/* Type mismatches. */
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":\"x\"}", 9U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_json_i64(&v, &s));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_json_u64(&v, &u));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_json_bool(&v, &b));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_json_i64(NULL, &s));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_json_i64(&v, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_json_u64(NULL, &u));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_json_bool(NULL, &b));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_json_str_copy(NULL, out,
							 sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_json_str_copy(&v, NULL,
							 sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_json_str_copy(&v, out, 0U));
	TEST_ASSERT_FALSE(web_json_str_eq(NULL, "x"));

	/* str_copy errors. */
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":\"toolongvalue\"}",
						  20U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, web_json_str_copy(&v, out, sizeof(out)));
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":\"\\q\"}", 10U, "a",
						  &v));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, web_json_str_copy(&v, out, sizeof(out)));
	/*
	 * A span ending in a lone backslash cannot come out of a well-formed
	 * document (the scanner would have consumed the closing quote as an
	 * escape), so it is built by hand — the copier must still refuse it
	 * rather than read one byte past the span.
	 */
	{
		web_json_val_t hand = { (uint8_t)WEB_JSON_STR, "\\", 1U };

		TEST_ASSERT_EQUAL_INT(-EILSEQ, web_json_str_copy(&hand, out,
								 sizeof(out)));
	}
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      web_json_obj_get("{\"a\":\"\\\"}", 9U, "a", &v));
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":\"\\u00\"}", 12U, "a",
						  &v));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, web_json_str_copy(&v, out, sizeof(out)));
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":\"\\uZZZZ\"}", 14U, "a",
						  &v));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, web_json_str_copy(&v, out, sizeof(out)));
	/* A lone surrogate and an encoded NUL are both refused. */
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":\"\\ud800\"}", 14U, "a",
						  &v));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, web_json_str_copy(&v, out, sizeof(out)));
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":\"\\u0000\"}", 14U, "a",
						  &v));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, web_json_str_copy(&v, out, sizeof(out)));
	/* A multi-byte escape that does not fit. */
	TEST_ASSERT_EQUAL_INT(0,
			      web_json_obj_get("{\"a\":\"aaaaaa\\u00e9\"}", 20U,
					       "a", &v));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, web_json_str_copy(&v, out, sizeof(out)));
	TEST_ASSERT_EQUAL_INT(0,
			      web_json_obj_get("{\"a\":\"aaaaa\\u20ac\"}", 19U,
					       "a", &v));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, web_json_str_copy(&v, out, sizeof(out)));

	/* Iterator argument checks. */
	cur = 0U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_json_arr_next(NULL, &cur, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_json_obj_next(NULL, &cur, &k, &v));
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":1}", 7U, "a", &v));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, web_json_arr_next(&v, &cur, &k));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, web_json_obj_next(&v, &cur, &k, &v));

	/* Empty containers iterate to zero elements. */
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":[]}", 8U, "a", &v));
	cur = 0U;
	TEST_ASSERT_EQUAL_INT(0, web_json_arr_next(&v, &cur, &k));
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":{}}", 8U, "a", &v));
	cur = 0U;
	{
		web_json_val_t kv;

		TEST_ASSERT_EQUAL_INT(0, web_json_obj_next(&v, &cur, &k, &kv));
	}
	/*
	 * A trailing comma inside an array. The comma promises another element,
	 * so it is refused at the comma rather than reported as a clean end one
	 * step later.
	 */
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":[1,]}", 10U, "a", &v));
	cur = 0U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, web_json_arr_next(&v, &cur, &k));
	/* A well-formed two-element array still iterates cleanly. */
	TEST_ASSERT_EQUAL_INT(0, web_json_obj_get("{\"a\":[1,2]}", 11U, "a", &v));
	cur = 0U;
	TEST_ASSERT_EQUAL_INT(1, web_json_arr_next(&v, &cur, &k));
	TEST_ASSERT_EQUAL_INT(1, web_json_arr_next(&v, &cur, &k));
	TEST_ASSERT_EQUAL_INT(0, web_json_arr_next(&v, &cur, &k));
}

static void test_codecs(void)
{
	char hex[16];
	uint8_t raw[8];
	char b64[16];
	static const uint8_t in3[3] = { 0x01U, 0x02U, 0x03U };

	TEST_ASSERT_EQUAL_UINT(6U, web_hex_encode(in3, 3U, hex, sizeof(hex)));
	TEST_ASSERT_EQUAL_STRING("010203", hex);
	TEST_ASSERT_EQUAL_UINT(0U, web_hex_encode(in3, 3U, hex, 6U));
	TEST_ASSERT_EQUAL_UINT(0U, web_hex_encode(in3, 3U, NULL, 8U));
	TEST_ASSERT_EQUAL_UINT(0U, web_hex_encode(in3, 3U, hex, 0U));
	TEST_ASSERT_EQUAL_UINT(0U, web_hex_encode(NULL, 3U, hex, sizeof(hex)));

	TEST_ASSERT_EQUAL_INT(3, web_hex_decode("0A0b0C", 6U, raw,
						sizeof(raw)));
	TEST_ASSERT_EQUAL_HEX8(0x0AU, raw[0]);
	TEST_ASSERT_EQUAL_HEX8(0x0CU, raw[2]);
	TEST_ASSERT_EQUAL_INT(-EILSEQ, web_hex_decode("0A0", 3U, raw,
						      sizeof(raw)));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, web_hex_decode("0Az0", 4U, raw,
						      sizeof(raw)));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, web_hex_decode("0102030405060708090a",
						      20U, raw, sizeof(raw)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_hex_decode(NULL, 2U, raw,
						      sizeof(raw)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_hex_decode("00", 2U, NULL,
						      sizeof(raw)));

	/* base64, all three padding cases. */
	TEST_ASSERT_EQUAL_UINT(4U, web_b64_encode(in3, 3U, b64, sizeof(b64)));
	TEST_ASSERT_EQUAL_STRING("AQID", b64);
	TEST_ASSERT_EQUAL_UINT(4U, web_b64_encode(in3, 2U, b64, sizeof(b64)));
	TEST_ASSERT_EQUAL_STRING("AQI=", b64);
	TEST_ASSERT_EQUAL_UINT(4U, web_b64_encode(in3, 1U, b64, sizeof(b64)));
	TEST_ASSERT_EQUAL_STRING("AQ==", b64);
	TEST_ASSERT_EQUAL_UINT(0U, web_b64_encode(in3, 3U, b64, 4U));
	TEST_ASSERT_EQUAL_UINT(0U, web_b64_encode(in3, 3U, NULL, 8U));
	TEST_ASSERT_EQUAL_UINT(0U, web_b64_encode(NULL, 3U, b64, sizeof(b64)));
	TEST_ASSERT_EQUAL_UINT(0U, web_b64_encode(NULL, 0U, b64, sizeof(b64)));

	TEST_ASSERT_EQUAL_INT(3, web_b64_decode("AQID", 4U, raw, sizeof(raw)));
	TEST_ASSERT_EQUAL_HEX8(0x03U, raw[2]);
	TEST_ASSERT_EQUAL_INT(2, web_b64_decode("AQI=", 4U, raw, sizeof(raw)));
	TEST_ASSERT_EQUAL_INT(1, web_b64_decode("AQ==", 4U, raw, sizeof(raw)));
	TEST_ASSERT_EQUAL_INT(3, web_b64_decode("AQ ID\n", 6U, raw,
						sizeof(raw)));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, web_b64_decode("A", 1U, raw,
						      sizeof(raw)));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, web_b64_decode("A?ID", 4U, raw,
						      sizeof(raw)));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, web_b64_decode("AQ===", 5U, raw,
						      sizeof(raw)));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, web_b64_decode("AQ=I", 4U, raw,
						      sizeof(raw)));
	/* Non-zero padding bits. */
	TEST_ASSERT_EQUAL_INT(-EILSEQ, web_b64_decode("AR==", 4U, raw,
						      sizeof(raw)));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      web_b64_decode("AQIDBAUGBwgJ", 12U, raw, 4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_b64_decode(NULL, 4U, raw,
						      sizeof(raw)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_b64_decode("AQID", 4U, NULL,
						      sizeof(raw)));
}

static void test_ct_and_spans(void)
{
	TEST_ASSERT_EQUAL_INT(0, web_ct_memcmp("abc", "abc", 3U));
	TEST_ASSERT_EQUAL_INT(1, web_ct_memcmp("abc", "abd", 3U));
	TEST_ASSERT_EQUAL_INT(1, web_ct_memcmp(NULL, "abd", 3U));
	TEST_ASSERT_EQUAL_INT(1, web_ct_memcmp("abc", NULL, 3U));
	/*
	 * The window (8, 64) is deliberately much longer than the strings: the
	 * comparator must walk it for constant time WITHOUT reading past either
	 * terminator. ASan is what actually proves the second half — this case
	 * caught a real out-of-bounds read.
	 */
	TEST_ASSERT_EQUAL_INT(0, web_ct_streq("abc", "abc", 8U));
	TEST_ASSERT_EQUAL_INT(0, web_ct_streq("abc", "abc", 64U));
	TEST_ASSERT_EQUAL_INT(0, web_ct_streq("", "", 64U));
	TEST_ASSERT_EQUAL_INT(1, web_ct_streq("abc", "abcd", 8U));
	TEST_ASSERT_EQUAL_INT(1, web_ct_streq("abcd", "abc", 8U));
	TEST_ASSERT_EQUAL_INT(1, web_ct_streq("", "a", 64U));
	TEST_ASSERT_EQUAL_INT(1, web_ct_streq("a", "", 64U));
	TEST_ASSERT_EQUAL_INT(1, web_ct_streq("abc", "abd", 8U));
	TEST_ASSERT_EQUAL_INT(1, web_ct_streq(NULL, "abc", 8U));
	TEST_ASSERT_EQUAL_INT(1, web_ct_streq("abc", NULL, 8U));

	TEST_ASSERT_TRUE(web_span_eq_ci("AbC", 3U, "aBc"));
	TEST_ASSERT_FALSE(web_span_eq_ci("Ab", 2U, "abc"));
	TEST_ASSERT_FALSE(web_span_eq_ci("Abcd", 4U, "abc"));
	TEST_ASSERT_FALSE(web_span_eq_ci(NULL, 3U, "abc"));
	TEST_ASSERT_FALSE(web_span_eq_ci("abc", 3U, NULL));
	TEST_ASSERT_TRUE(web_span_eq("abc", 3U, "abc"));
	TEST_ASSERT_FALSE(web_span_eq("abc", 2U, "abc"));
	TEST_ASSERT_FALSE(web_span_eq(NULL, 3U, "abc"));

	{
		char out[4];

		TEST_ASSERT_EQUAL_UINT(3U, web_span_copy(out, sizeof(out),
							 "abcdef", 6U));
		TEST_ASSERT_EQUAL_STRING("abc", out);
		TEST_ASSERT_EQUAL_UINT(0U, web_span_copy(out, sizeof(out), NULL,
							 4U));
		TEST_ASSERT_EQUAL_UINT(0U, web_span_copy(NULL, 4U, "a", 1U));
		TEST_ASSERT_EQUAL_UINT(0U, web_span_copy(out, 0U, "a", 1U));
	}
}

static void test_methods_and_roles(void)
{
	uint8_t r = 0U;

	TEST_ASSERT_EQUAL_UINT(WEB_METHOD_GET, web_method_parse("GET", 3U));
	TEST_ASSERT_EQUAL_UINT(WEB_METHOD_OPTIONS,
			       web_method_parse("OPTIONS", 7U));
	TEST_ASSERT_EQUAL_UINT(WEB_METHOD_UNKNOWN, web_method_parse("get", 3U));
	TEST_ASSERT_EQUAL_UINT(WEB_METHOD_UNKNOWN, web_method_parse(NULL, 3U));
	TEST_ASSERT_EQUAL_UINT(WEB_METHOD_UNKNOWN, web_method_parse("GET", 0U));
	TEST_ASSERT_EQUAL_STRING("PUT", web_method_name(WEB_METHOD_PUT));
	TEST_ASSERT_EQUAL_STRING("?", web_method_name(WEB_METHOD_UNKNOWN));
	TEST_ASSERT_TRUE(web_method_is_mutating(WEB_METHOD_POST));
	TEST_ASSERT_TRUE(web_method_is_mutating(WEB_METHOD_PUT));
	TEST_ASSERT_TRUE(web_method_is_mutating(WEB_METHOD_DELETE));
	TEST_ASSERT_FALSE(web_method_is_mutating(WEB_METHOD_GET));
	TEST_ASSERT_FALSE(web_method_is_mutating(WEB_METHOD_HEAD));

	TEST_ASSERT_EQUAL_STRING("admin", web_role_name(WEB_ROLE_ADMIN));
	TEST_ASSERT_EQUAL_STRING("operator", web_role_name(WEB_ROLE_OPERATOR));
	TEST_ASSERT_EQUAL_STRING("viewer", web_role_name(WEB_ROLE_VIEWER));
	TEST_ASSERT_EQUAL_STRING("none", web_role_name(WEB_ROLE_NONE));
	TEST_ASSERT_EQUAL_STRING("none", web_role_name(200U));
	TEST_ASSERT_EQUAL_INT(0, web_role_parse("admin", 5U, &r));
	TEST_ASSERT_EQUAL_UINT(WEB_ROLE_ADMIN, r);
	TEST_ASSERT_EQUAL_INT(0, web_role_parse("operator", 8U, &r));
	TEST_ASSERT_EQUAL_UINT(WEB_ROLE_OPERATOR, r);
	TEST_ASSERT_EQUAL_INT(0, web_role_parse("viewer", 6U, &r));
	TEST_ASSERT_EQUAL_UINT(WEB_ROLE_VIEWER, r);
	TEST_ASSERT_EQUAL_INT(-ENOENT, web_role_parse("root", 4U, &r));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_role_parse(NULL, 4U, &r));
	TEST_ASSERT_EQUAL_INT(-EINVAL, web_role_parse("admin", 5U, NULL));
}

static void test_status_text(void)
{
	TEST_ASSERT_EQUAL_STRING("OK", rest_status_text(200));
	TEST_ASSERT_EQUAL_STRING("Not Found", rest_status_text(404));
	TEST_ASSERT_EQUAL_STRING("Error", rest_status_text(599));
	/* Every status the router can emit must have a real phrase. */
	{
		static const uint16_t used[] = { 200, 201, 202, 204, 304, 400,
						 401, 403, 404, 405, 409, 411,
						 413, 415, 422, 426, 429, 431,
						 500, 501, 503 };
		size_t i;

		for (i = 0U; i < (sizeof(used) / sizeof(used[0])); i++) {
			TEST_ASSERT_TRUE(strcmp(rest_status_text(used[i]),
						"Error") != 0);
		}
	}
}

/* ========================================================================= */
/* part 2: the router                                                        */
/* ========================================================================= */

/* ---- fake providers ----------------------------------------------------- */

static struct {
	int      quality_rc;
	int      health_rc;
	int      gnss_rc;
	int      net_rc;
	int      ptp_rc;
	int      svc_rc;
	int      survey_rc;
	int      fixed_rc;
	int      ref_rc;
	int      svc_en_rc;
	int      cal_rc;
	int      reboot_rc;
	int      factory_rc;
	int      fw_info_rc;
	int      fw_begin_rc;
	int      fw_data_rc;
	int      fw_end_rc;
	int      fw_confirm_rc;
	int      fw_revert_rc;
	int      cert_info_rc;
	int      cert_install_rc;
	int      csr_rc;

	bool     survey_started;
	int64_t  ecef[3];
	uint8_t  ref_mode;
	uint8_t  svc_id;
	bool     svc_enable;
	uint8_t  cal_proc;
	uint8_t  reboot_mode;
	uint32_t fw_size;
	uint8_t  fw_sha[32];
	uint32_t fw_off;
	uint32_t fw_len;
	uint32_t fw_next;
	bool     factory_called;
	bool     confirm_called;
	bool     revert_called;
	size_t   cert_len;
} fk;

static int pv_quality(void *u, quality_block_t *out)
{
	(void)u;
	if (fk.quality_rc != 0) {
		return fk.quality_rc;
	}
	quality_block_init(out);
	out->tick = 7U;
	out->stratum = QUALITY_STRATUM_PRIMARY;
	out->lock_state = (uint8_t)QUALITY_LOCK_LOCKED;
	out->active_ref = (uint8_t)QUALITY_REF_OCXO;
	out->gnss_fix = (uint8_t)QUALITY_GNSS_TIME_ONLY;
	out->gnss_sv_used = 12U;
	out->gnss_sv_visible = 18U;
	out->last_pps_off_ns = -3;
	out->pps_off_mean_ns = 1.25f;
	out->pps_off_sigma_ns = 4.5f;
	out->freq_err_ppb = -0.125f;
	out->vc_cmd_mv = 1650;
	out->vc_sense_mv = 1649;
	out->dac_code = 2048U;
	out->adev_1s = 1.0e-12f;
	out->root_disp_q16 = 65536U; /* 1 s */
	out->holdover_t_demote_s = UINT32_MAX;
	out->osc_temp_mc = 55000;
	out->flags = QUALITY_FLAG_OCXO_WARM;
	return 0;
}

/* A second variant with the two validity flags set, to cover both branches. */
static int pv_quality_valid(void *u, quality_block_t *out)
{
	int rc = pv_quality(u, out);

	if (rc == 0) {
		out->flags |= QUALITY_FLAG_VC_SENSE_VALID |
			      QUALITY_FLAG_OSC_TEMP_VALID;
		out->holdover = true;
		out->holdover_t_demote_s = 900U;
	}
	return rc;
}

static int pv_health(void *u, rest_health_t *out)
{
	size_t i;

	(void)u;
	if (fk.health_rc != 0) {
		return fk.health_rc;
	}
	memset(out, 0, sizeof(*out));
	out->n_rails = REST_RAIL_MAX;
	for (i = 0U; i < REST_RAIL_MAX; i++) {
		out->rail[i].name = "3V3_STM";
		out->rail[i].designator = "U31";
		out->rail[i].shunt_ref = "R106";
		out->rail[i].addr = (uint8_t)(0x40U + i);
		out->rail[i].bus_mv = 3300;
		out->rail[i].current_ua = 123456;
		out->rail[i].power_uw = 407404;
		out->rail[i].nominal_mv = 3300;
		out->rail[i].design_max_ma = 300U;
		out->rail[i].valid = (i != 2U); /* one invalid rail */
	}
	out->tmp_osc_mc = 55000;
	out->tmp_osc_valid = true;
	out->tmp_amb_mc = 30000;
	out->tmp_amb_valid = true;
	out->die_valid = false;
	out->humidity_valid = true;
	out->humidity_mpct = 42500;
	out->fan_rpm = 3200U;
	out->fan_duty_pct = 45U;
	out->poe_class = 4U;
	out->poe_draw_mw = 13500U;
	out->poe_budget_mw = 25500U;
	out->bkp_stm_pg = true;
	out->age_ms = 250U;
	return 0;
}

static int pv_gnss(void *u, rest_gnss_t *out)
{
	(void)u;
	if (fk.gnss_rc != 0) {
		return fk.gnss_rc;
	}
	memset(out, 0, sizeof(*out));
	out->detail_available = true;
	out->fix_type = (uint8_t)QUALITY_GNSS_TIME_ONLY;
	out->sv_used = 2U;
	out->sv_visible = 3U;
	out->tacc_ns = 12U;
	out->survey_state = (uint8_t)REST_SURVEY_FIXED;
	out->position_valid = true;
	out->ecef_x_cm = 123456789LL;
	out->ant_state = (uint8_t)REST_ANT_OK;
	out->ant_bias_on = true;
	out->leap_current_s = 37;
	out->utc_valid = true;
	out->n_sats = 3U;
	out->sat[0].gnss_id = 0U;
	out->sat[0].sv_id = 1U;
	out->sat[0].cno = 44U;
	out->sat[0].elev_deg = 65;
	out->sat[0].azim_deg = 180;
	out->sat[0].used = true;
	out->sat[1].gnss_id = 2U;
	out->sat[1].sv_id = 11U;
	out->sat[1].used = true;
	out->sat[2].gnss_id = 9U; /* unknown constellation id */
	(void)web_span_copy(out->sw_version, sizeof(out->sw_version), "1.2", 3U);
	(void)web_span_copy(out->hw_version, sizeof(out->hw_version), "F9T", 3U);
	return 0;
}

/* Cover the other survey / antenna enum branches. */
static int pv_gnss_alt(void *u, rest_gnss_t *out)
{
	int rc = pv_gnss(u, out);

	if (rc == 0) {
		out->survey_state = (uint8_t)REST_SURVEY_ACTIVE;
		out->ant_state = (uint8_t)REST_ANT_OPEN;
		out->detail_available = false;
	}
	return rc;
}

static int pv_gnss_alt2(void *u, rest_gnss_t *out)
{
	int rc = pv_gnss(u, out);

	if (rc == 0) {
		out->survey_state = (uint8_t)REST_SURVEY_IDLE;
		out->ant_state = (uint8_t)REST_ANT_SHORT;
	}
	return rc;
}

static int pv_gnss_alt3(void *u, rest_gnss_t *out)
{
	int rc = pv_gnss(u, out);

	if (rc == 0) {
		out->ant_state = (uint8_t)REST_ANT_UNKNOWN;
		out->fix_type = (uint8_t)QUALITY_GNSS_2D;
	}
	return rc;
}

/*
 * The satellite array at its bound, with every field at its widest.
 *
 * `n_sats` is REST_SAT_MAX here; pv_gnss_over() below sets it one past, which
 * the encoder must clamp rather than walk off the end of a fixed array with a
 * count that came, ultimately, from a GNSS receiver.
 */
static void gnss_fill_max(rest_gnss_t *out)
{
	unsigned int i;

	memset(out, 0, sizeof(*out));
	out->detail_available = true;
	out->sat_age_valid = true;
	out->sat_age_ms = UINT32_MAX;
	out->fix_type = (uint8_t)QUALITY_GNSS_TIME_ONLY;
	out->sv_used = 32U;
	out->sv_visible = 32U;
	out->tacc_ns = UINT32_MAX;
	out->survey_state = (uint8_t)REST_SURVEY_ACTIVE;
	out->survey_dur_s = UINT32_MAX;
	out->survey_obs = UINT32_MAX;
	out->survey_acc_mm = UINT32_MAX;
	out->position_valid = true;
	out->ecef_x_cm = INT64_MIN;
	out->ecef_y_cm = INT64_MIN;
	out->ecef_z_cm = INT64_MIN;
	out->ant_state = (uint8_t)REST_ANT_OK;
	out->ant_bias_on = true;
	out->leap_current_s = -32768;
	out->leap_pending = -128;
	out->utc_valid = true;
	out->n_sats = (uint8_t)REST_SAT_MAX;
	for (i = 0U; i < (unsigned int)REST_SAT_MAX; i++) {
		out->sat[i].gnss_id = 6U; /* "glonass", the longest name */
		out->sat[i].sv_id = (uint8_t)(200U + i);
		out->sat[i].cno = 255U;
		out->sat[i].elev_deg = -128;
		out->sat[i].azim_deg = 359;
		out->sat[i].used = true;
	}
	memset(out->sw_version, 'X', sizeof(out->sw_version) - 1U);
	memset(out->hw_version, 'X', sizeof(out->hw_version) - 1U);
}

static int pv_gnss_max(void *u, rest_gnss_t *out)
{
	(void)u;
	if (fk.gnss_rc != 0) {
		return fk.gnss_rc;
	}
	gnss_fill_max(out);
	return 0;
}

/* One past the array bound. Must be clamped, not served and not read. */
static int pv_gnss_over(void *u, rest_gnss_t *out)
{
	(void)u;
	if (fk.gnss_rc != 0) {
		return fk.gnss_rc;
	}
	gnss_fill_max(out);
	out->n_sats = (uint8_t)(REST_SAT_MAX + 1U);
	/* A distinguishable marker in the last legal slot, so the count of
	 * emitted objects can be checked without counting braces. */
	out->sat[REST_SAT_MAX - 1U].sv_id = 199U;
	return 0;
}

/* Fresh, and genuinely empty: the case that must not read as "no data". */
static int pv_gnss_empty(void *u, rest_gnss_t *out)
{
	(void)u;
	if (fk.gnss_rc != 0) {
		return fk.gnss_rc;
	}
	memset(out, 0, sizeof(*out));
	out->detail_available = true;
	out->sat_age_valid = true;
	out->sat_age_ms = 250U;
	out->n_sats = 0U;
	return 0;
}

/* Stale: a list existed once, and its age is the useful half of the answer. */
static int pv_gnss_stale(void *u, rest_gnss_t *out)
{
	(void)u;
	if (fk.gnss_rc != 0) {
		return fk.gnss_rc;
	}
	memset(out, 0, sizeof(*out));
	out->detail_available = false;
	out->sat_age_valid = true;
	out->sat_age_ms = 412345U;
	out->n_sats = 0U;
	return 0;
}

/* Never heard from the receiver at all. */
static int pv_gnss_never(void *u, rest_gnss_t *out)
{
	(void)u;
	if (fk.gnss_rc != 0) {
		return fk.gnss_rc;
	}
	memset(out, 0, sizeof(*out));
	return 0;
}

static int pv_net(void *u, rest_net_t *out)
{
	(void)u;
	if (fk.net_rc != 0) {
		return fk.net_rc;
	}
	memset(out, 0, sizeof(*out));
	out->link_up = true;
	out->ipv4_ok = true;
	out->dhcp_bound = true;
	out->ipv4_addr = 0xC0A80105U; /* 192.168.1.5 */
	out->ipv4_mask = 0xFFFFFF00U;
	out->ipv4_gw = 0xC0A80101U;
	out->mac[0] = 0x02U;
	out->mac[5] = 0xEFU;
	(void)web_span_copy(out->hostname, sizeof(out->hostname), "meridian", 8U);
	out->ptp_clock_ok = true;
	out->ptp_clock_off_ns = -17;
	return 0;
}

/*
 * The PTP fake is deliberately the WORST case, not a typical one.
 *
 * This provider feeds the worst-case telemetry-frame measurement below, and
 * that measurement is only worth having if the group it measures is as large as
 * it can get in the field. A fake with alarms = 0 and profile = Default emits an
 * empty `alarm_names` array and the 31-character Default deviation string —
 * 97 bytes less than a real C37.238 unit with every alarm up, which is 97 bytes
 * of guard band the test would have silently handed back.
 *
 * So: every bit of PTP_ALARM_ALL set (all six named, the longest possible
 * `alarm_names`), and the profile whose deviation text is longest. The text is
 * spelled out here rather than called from core/ptp because this suite links
 * core/web only; PTP_PROFILE_DEVIATION_TEXT_MAX is the compile-time tie, and
 * test_ptp_profile.c is what keeps that bound equal to the real longest string.
 */
#define FAKE_PTP_ALARMS 0x3FU /* PTP_ALARM_ALL: all six bits named */
#define FAKE_PTP_DEVIATION_TEXT \
	"E2E only (profile mandates peer-delay); two-step only; grandmaster-only"

/*
 * The tie that stops this fake from drifting *below* reality. If core/ptp grows
 * a longer deviation string, PTP_PROFILE_DEVIATION_TEXT_MAX rises with it (its
 * own suite enforces that), and this fails here rather than silently restoring
 * the understated measurement this fake exists to prevent.
 */
_Static_assert(sizeof(FAKE_PTP_DEVIATION_TEXT) - 1U ==
		       (size_t)PTP_PROFILE_DEVIATION_TEXT_MAX,
	       "PTP fake no longer carries the longest deviation text");

static int pv_ptp(void *u, rest_ptp_t *out)
{
	(void)u;
	if (fk.ptp_rc != 0) {
		return fk.ptp_rc;
	}
	memset(out, 0, sizeof(*out));
	out->running = true;
	out->clock_class = 6U;
	out->tx_total = 100U;
	out->alarms = FAKE_PTP_ALARMS;
	out->profile = 3U; /* PTP_PROFILE_POWER_C37_238 */
	out->deviations = 0x0007U;
	out->profile_name = "C37.238";
	out->deviation_text = FAKE_PTP_DEVIATION_TEXT;
	return 0;
}

static int pv_services(void *u, rest_services_t *out)
{
	(void)u;
	if (fk.svc_rc != 0) {
		return fk.svc_rc;
	}
	memset(out, 0, sizeof(*out));
	out->ntp_rx = 1000U;
	out->ntp_served = 999U;
	out->ntp_running = true;
	out->uptime_s = 3600U;
	out->board_id[0] = 0xDEU;
	out->board_id[7] = 0xEFU;
	(void)web_span_copy(out->fw_version, sizeof(out->fw_version), "0.4.0", 5U);
	(void)web_span_copy(out->model, sizeof(out->model), "STS1000", 7U);
	return 0;
}

static uint64_t pv_alarms(void *u)
{
	(void)u;
	return (UINT64_C(1) << 3) | (UINT64_C(1) << 33);
}

static uint64_t pv_alarms_latched(void *u)
{
	(void)u;
	return (UINT64_C(1) << 40);
}

static const char *pv_alarm_name(void *u, uint8_t bit)
{
	(void)u;
	if (bit == 33U) {
		return "GNSS_LOST";
	}
	return NULL;
}

/*
 * The PTP alarm namespace, which is NOT the fault namespace above: the same
 * small integers mean different things in each. Bit 3 is deliberately
 * "PROFILE_UNSUPPORTED" here and "ANTENNA_SHORT" over there, so a test that
 * passes with the two swapped would be the bug this callback exists to prevent.
 *
 * Bit 4 returns NULL on purpose even though it is set in FAKE_PTP_ALARMS: the
 * encoder must SKIP an unnamed set bit rather than emitting null, "", or
 * stopping the array early.
 */
static const char *pv_ptp_alarm_name(void *u, uint8_t bit)
{
	static const char *const names[] = {
		"NOT_BEST_MASTER", "FAULTY", "TX_ERROR", "PROFILE_UNSUPPORTED",
		NULL, /* bit 4: deliberately unnamed */
		"DISPLACED_WHILE_LOCKED",
	};

	(void)u;
	if ((size_t)bit >= (sizeof(names) / sizeof(names[0]))) {
		return NULL;
	}
	return names[bit];
}

static int pv_survey(void *u, bool start)
{
	(void)u;
	fk.survey_started = start;
	return fk.survey_rc;
}

static int pv_fixed(void *u, int64_t x, int64_t y, int64_t z)
{
	(void)u;
	fk.ecef[0] = x;
	fk.ecef[1] = y;
	fk.ecef[2] = z;
	return fk.fixed_rc;
}

static int pv_ref(void *u, uint8_t mode)
{
	(void)u;
	fk.ref_mode = mode;
	return fk.ref_rc;
}

static int pv_svc_en(void *u, uint8_t svc, bool enable)
{
	(void)u;
	fk.svc_id = svc;
	fk.svc_enable = enable;
	return fk.svc_en_rc;
}

static int pv_cal_run(void *u, uint8_t proc, uint32_t arg)
{
	(void)u;
	(void)arg;
	fk.cal_proc = proc;
	return fk.cal_rc;
}

static int pv_reboot(void *u, uint8_t mode)
{
	(void)u;
	fk.reboot_mode = mode;
	return fk.reboot_rc;
}

static int pv_factory(void *u)
{
	(void)u;
	fk.factory_called = true;
	return fk.factory_rc;
}

static int pv_fw_info(void *u, rest_fw_t *out)
{
	(void)u;
	if (fk.fw_info_rc != 0) {
		return fk.fw_info_rc;
	}
	memset(out, 0, sizeof(*out));
	out->n_slots = 2U;
	out->slot[0].slot = 0U;
	out->slot[0].size = 400000U;
	out->slot[0].version[0] = 0U;
	out->slot[0].version[1] = 4U;
	out->slot[0].version[2] = 0U;
	out->slot[0].version[3] = 123U;
	out->slot[0].valid = true;
	out->slot[0].active = true;
	out->slot[0].confirmed = true;
	out->slot[1].slot = 1U;
	out->chunk_max = 1024U;
	out->write_block = 16U;
	out->pending_confirm = false;
	return 0;
}

static int pv_fw_begin(void *u, uint32_t size, const uint8_t sha[32],
		       uint32_t *next)
{
	(void)u;
	fk.fw_size = size;
	memcpy(fk.fw_sha, sha, 32U);
	*next = fk.fw_next;
	return fk.fw_begin_rc;
}

static int pv_fw_data(void *u, uint32_t off, const uint8_t *d, size_t n,
		      uint32_t *next)
{
	(void)u;
	(void)d;
	fk.fw_off = off;
	fk.fw_len = (uint32_t)n;
	*next = fk.fw_next;
	return fk.fw_data_rc;
}

static int pv_fw_end(void *u, uint32_t *size)
{
	(void)u;
	*size = 400000U;
	return fk.fw_end_rc;
}

static int pv_fw_confirm(void *u)
{
	(void)u;
	fk.confirm_called = true;
	return fk.fw_confirm_rc;
}

static int pv_fw_revert(void *u)
{
	(void)u;
	fk.revert_called = true;
	return fk.fw_revert_rc;
}

static int pv_cert_info(void *u, rest_cert_t *out)
{
	(void)u;
	if (fk.cert_info_rc != 0) {
		return fk.cert_info_rc;
	}
	memset(out, 0, sizeof(*out));
	out->present = true;
	out->self_signed = true;
	out->persisted = true;
	(void)web_span_copy(out->subject, sizeof(out->subject),
			    "CN=STS1000 Meridian", 19U);
	(void)web_span_copy(out->sha256_fp, sizeof(out->sha256_fp), "aabb", 4U);
	(void)web_span_copy(out->key_type, sizeof(out->key_type), "EC", 2U);
	out->key_bits = 256U;
	out->acme_state = (uint8_t)REST_ACME_DISABLED;
	return 0;
}

static int pv_cert_install(void *u, const char *pem, size_t len)
{
	(void)u;
	(void)pem;
	fk.cert_len = len;
	return fk.cert_install_rc;
}

static int pv_csr(void *u, const char *subject, char *out, size_t cap,
		  size_t *out_len)
{
	static const char *const csr = "-----BEGIN CERTIFICATE REQUEST-----\n";

	(void)u;
	(void)subject;
	if (fk.csr_rc != 0) {
		return fk.csr_rc;
	}
	if (strlen(csr) >= cap) {
		return -ENOSPC;
	}
	memcpy(out, csr, strlen(csr));
	*out_len = strlen(csr);
	return 0;
}

static rest_providers_t g_pv;

static void providers_all(void)
{
	memset(&g_pv, 0, sizeof(g_pv));
	g_pv.quality = pv_quality;
	g_pv.health = pv_health;
	g_pv.gnss = pv_gnss;
	g_pv.net = pv_net;
	g_pv.ptp = pv_ptp;
	g_pv.services = pv_services;
	g_pv.alarms = pv_alarms;
	g_pv.alarms_latched = pv_alarms_latched;
	g_pv.alarm_name = pv_alarm_name;
	g_pv.ptp_alarm_name = pv_ptp_alarm_name;
	g_pv.gnss_survey = pv_survey;
	g_pv.gnss_fixed = pv_fixed;
	g_pv.ref_override = pv_ref;
	g_pv.service_enable = pv_svc_en;
	g_pv.cal_run = pv_cal_run;
	g_pv.reboot = pv_reboot;
	g_pv.factory_reset = pv_factory;
	g_pv.fw_info = pv_fw_info;
	g_pv.fw_begin = pv_fw_begin;
	g_pv.fw_data = pv_fw_data;
	g_pv.fw_end = pv_fw_end;
	g_pv.fw_confirm = pv_fw_confirm;
	g_pv.fw_revert = pv_fw_revert;
	g_pv.cert_info = pv_cert_info;
	g_pv.cert_install = pv_cert_install;
	g_pv.csr_make = pv_csr;
	g_pv.u = NULL;
}

/* ---- fixture ------------------------------------------------------------ */

#define PW "correct-horse"
#define PW_LEN (sizeof(PW) - 1U)
#define RESP_CAP 20480U

static cfg_ctx_t      g_cfg;
static logr_t         g_log;
static logr_rec_t     g_slots[64];
static auth_web_ctx_t g_auth;
static auth_kdf_t     g_kdf;
static host_crypto_t  g_hc;
static port_crypto_t  g_port;
static rest_ctx_t     g_rest;

static char       g_respbuf[RESP_CAP];
static web_resp_t g_resp;
static char       g_reqbuf[8192];
static http_req_t g_req;

static char g_token[AUTH_WEB_TOKEN_LEN + 1U];
static char g_csrf[AUTH_WEB_CSRF_LEN + 1U];

static int commit_hook_calls;

static int commit_hook(void *u, cfg_commit_res_t *res)
{
	(void)u;
	commit_hook_calls++;
	return cfg_commit(&g_cfg, res);
}

static void setup_rest(uint8_t role)
{
	cfg_commit_res_t res;

	memset(&fk, 0, sizeof(fk));
	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, NULL));
	TEST_ASSERT_EQUAL_INT(0, logr_init(&g_log, g_slots, 64U));
	host_crypto_init(&g_hc, 0xABCDU);
	g_port = host_crypto_port(&g_hc);
	TEST_ASSERT_EQUAL_INT(0, auth_kdf_hmac_sha256(&g_kdf, &g_port));
	TEST_ASSERT_EQUAL_INT(0, auth_web_init(&g_auth, &g_port, &g_kdf, &g_cfg,
					       &g_log));
	providers_all();
	TEST_ASSERT_EQUAL_INT(0, rest_init(&g_rest, &g_cfg, &g_log, &g_auth,
					   &g_pv));
	g_rest.commit = commit_hook;
	g_rest.now_ms = 1000U;
	commit_hook_calls = 0;

	g_token[0] = '\0';
	g_csrf[0] = '\0';
	if (role == (uint8_t)WEB_ROLE_NONE) {
		return;
	}

	TEST_ASSERT_EQUAL_INT(0, auth_web_user_add(&g_auth, "admin", role,
						   (uint16_t)CFG_ID_SEC_ADMIN_PW));
	TEST_ASSERT_EQUAL_INT(0, auth_web_set_password(&g_auth, 0U,
						       (const uint8_t *)PW,
						       PW_LEN));
	memset(&res, 0, sizeof(res));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, &res));
	TEST_ASSERT_EQUAL_INT(1, auth_web_reload(&g_auth));
	{
		auth_web_grant_t g;

		TEST_ASSERT_EQUAL_INT(0, auth_web_login(&g_auth, "admin", 5U,
							(const uint8_t *)PW,
							PW_LEN, g_rest.now_ms,
							&g));
		memcpy(g_token, g.token, sizeof(g_token));
		memcpy(g_csrf, g.csrf, sizeof(g_csrf));
	}
}

/*
 * Issue one request. @p flags: bit0 send the session cookie, bit1 send the CSRF
 * header. The body is sent with an explicit Content-Length.
 */
#define REQ_COOKIE 0x01U
#define REQ_CSRF   0x02U

static void request(const char *method, const char *target, unsigned int flags,
		    const char *ctype, const char *body, size_t body_len)
{
	size_t o = 0U;

	o += (size_t)snprintf(&g_reqbuf[o], sizeof(g_reqbuf) - o,
			      "%s %s HTTP/1.1\r\nHost: h\r\n", method, target);
	if ((flags & REQ_COOKIE) != 0U && g_token[0] != '\0') {
		o += (size_t)snprintf(&g_reqbuf[o], sizeof(g_reqbuf) - o,
				      "Cookie: sts_session=%s\r\n", g_token);
	}
	if ((flags & REQ_CSRF) != 0U && g_csrf[0] != '\0') {
		o += (size_t)snprintf(&g_reqbuf[o], sizeof(g_reqbuf) - o,
				      "X-CSRF-Token: %s\r\n", g_csrf);
	}
	if (ctype != NULL) {
		o += (size_t)snprintf(&g_reqbuf[o], sizeof(g_reqbuf) - o,
				      "Content-Type: %s\r\n", ctype);
	}
	o += (size_t)snprintf(&g_reqbuf[o], sizeof(g_reqbuf) - o,
			      "Content-Length: %u\r\n\r\n",
			      (unsigned int)body_len);
	if (body_len != 0U) {
		memcpy(&g_reqbuf[o], body, body_len);
		o += body_len;
	}

	TEST_ASSERT_EQUAL_INT(0, http_parse_request(g_reqbuf, o, &g_req));
	rest_resp_init(&g_resp, g_respbuf, sizeof(g_respbuf));
	TEST_ASSERT_EQUAL_INT(0, rest_dispatch(&g_rest, &g_req,
					       (body_len != 0U)
						       ? &g_reqbuf[o - body_len]
						       : NULL,
					       body_len, &g_resp));
}

static void get_auth(const char *target)
{
	request("GET", target, REQ_COOKIE, NULL, NULL, 0U);
}

static void post_auth(const char *target, const char *body)
{
	request("POST", target, REQ_COOKIE | REQ_CSRF, "application/json", body,
		(body == NULL) ? 0U : strlen(body));
}

/* The response body as a NUL-terminated string (it always is). */
static const char *body_str(void)
{
	TEST_ASSERT_TRUE(g_resp.body_len < RESP_CAP);
	g_respbuf[g_resp.body_len] = '\0';
	return g_respbuf;
}

static bool body_has(const char *needle)
{
	return strstr(body_str(), needle) != NULL;
}

/* How many times @p needle occurs in the body. Non-overlapping. */
static unsigned int body_count(const char *needle)
{
	const char *p = body_str();
	size_t n = strlen(needle);
	unsigned int c = 0U;

	TEST_ASSERT_TRUE(n != 0U);
	for (;;) {
		p = strstr(p, needle);
		if (p == NULL) {
			return c;
		}
		c++;
		p += n;
	}
}

/* ---- routing ------------------------------------------------------------ */

static void test_routing_basics(void)
{
	setup_rest((uint8_t)WEB_ROLE_ADMIN);

	/* Not an API path. */
	request("GET", "/index.html", REQ_COOKIE, NULL, NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(404U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("not_found"));

	/* Unknown route. */
	get_auth("/api/v1/nope");
	TEST_ASSERT_EQUAL_UINT(404U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("no_such_route"));

	/* Wrong method on a real path. */
	request("PUT", "/api/v1/status", REQ_COOKIE | REQ_CSRF, NULL, NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(405U, g_resp.status);
	TEST_ASSERT_TRUE(strstr(g_resp.extra, "Allow:") != NULL);

	/* An unknown method. */
	request("PATCH", "/api/v1/status", REQ_COOKIE, NULL, NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(405U, g_resp.status);

	/* OPTIONS answers 204 with Allow and no CORS headers. */
	request("OPTIONS", "/api/v1/status", REQ_COOKIE, NULL, NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(204U, g_resp.status);
	TEST_ASSERT_EQUAL_UINT(0U, g_resp.body_len);
	TEST_ASSERT_TRUE(strstr(g_resp.extra, "Allow:") != NULL);
	TEST_ASSERT_NULL(strstr(g_resp.extra, "Access-Control"));

	/* A prefix route must not swallow a longer word. */
	get_auth("/api/v1/statusfoo");
	TEST_ASSERT_EQUAL_UINT(404U, g_resp.status);

	/* HEAD is answered by the GET handler. */
	request("HEAD", "/api/v1/status/summary", REQ_COOKIE, NULL, NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);

	/* An exact route wins over the prefix route. */
	post_auth("/api/v1/config/revert", NULL);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);

	TEST_ASSERT_EQUAL_INT(-EINVAL, rest_dispatch(NULL, &g_req, NULL, 0U,
						     &g_resp));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rest_dispatch(&g_rest, NULL, NULL, 0U,
						     &g_resp));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rest_dispatch(&g_rest, &g_req, NULL, 0U,
						     NULL));
	TEST_ASSERT_FALSE(rest_is_api_path(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rest_init(NULL, NULL, NULL, NULL, NULL));
	rest_resp_init(NULL, NULL, 0U);
}

static void test_status_groups(void)
{
	setup_rest((uint8_t)WEB_ROLE_VIEWER);

	get_auth("/api/v1/status/summary");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"stratum\":1"));
	TEST_ASSERT_TRUE(body_has("\"lock_state\":\"locked\""));
	TEST_ASSERT_TRUE(body_has("\"model\":\"STS1000\""));
	TEST_ASSERT_TRUE(body_has("\"board_id\":\"de000000000000ef\""));

	get_auth("/api/v1/status/timing");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"last_pps_offset_ns\":-3"));
	TEST_ASSERT_TRUE(body_has("\"pps_offset_mean_ns\":1.250"));
	TEST_ASSERT_TRUE(body_has("\"freq_err_ppb\":-0.1250"));
	TEST_ASSERT_TRUE(body_has("\"root_disp_ns\":1000000000"));
	/* Neither validity flag is set in the default fake. */
	TEST_ASSERT_TRUE(body_has("\"vc_sense_mv\":null"));
	TEST_ASSERT_TRUE(body_has("\"osc_temp_c\":null"));
	TEST_ASSERT_TRUE(body_has("\"time_to_demote_s\":null"));

	/* The other branch of each validity flag. */
	g_pv.quality = pv_quality_valid;
	get_auth("/api/v1/status/timing");
	TEST_ASSERT_TRUE(body_has("\"vc_sense_mv\":1649"));
	TEST_ASSERT_TRUE(body_has("\"osc_temp_c\":55.000"));
	TEST_ASSERT_TRUE(body_has("\"time_to_demote_s\":900"));
	g_pv.quality = pv_quality;

	get_auth("/api/v1/status/gnss");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"constellation\":\"gps\""));
	TEST_ASSERT_TRUE(body_has("\"constellation\":\"galileo\""));
	TEST_ASSERT_TRUE(body_has("\"constellation\":\"other\""));
	TEST_ASSERT_TRUE(body_has("\"state\":\"fixed\""));
	TEST_ASSERT_TRUE(body_has("\"state\":\"ok\""));
	TEST_ASSERT_TRUE(body_has("\"ecef_x_cm\":123456789"));

	g_pv.gnss = pv_gnss_alt;
	get_auth("/api/v1/status/gnss");
	TEST_ASSERT_TRUE(body_has("\"state\":\"active\""));
	TEST_ASSERT_TRUE(body_has("\"state\":\"open\""));
	TEST_ASSERT_TRUE(body_has("\"detail_available\":false"));
	g_pv.gnss = pv_gnss_alt2;
	get_auth("/api/v1/status/gnss");
	TEST_ASSERT_TRUE(body_has("\"state\":\"idle\""));
	TEST_ASSERT_TRUE(body_has("\"state\":\"short\""));
	g_pv.gnss = pv_gnss_alt3;
	get_auth("/api/v1/status/gnss");
	TEST_ASSERT_TRUE(body_has("\"state\":\"unknown\""));
	TEST_ASSERT_TRUE(body_has("\"fix\":\"2d\""));
	g_pv.gnss = pv_gnss;

	get_auth("/api/v1/status/power");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"bus_v\":3.300"));
	TEST_ASSERT_TRUE(body_has("\"current_a\":0.123456"));
	TEST_ASSERT_TRUE(body_has("\"bus_v\":null")); /* the invalid rail */
	TEST_ASSERT_TRUE(body_has("\"die_c\":null"));
	TEST_ASSERT_TRUE(body_has("\"humidity_pct\":42.500"));
	TEST_ASSERT_TRUE(body_has("\"budget_w\":25.500"));

	get_auth("/api/v1/status/net");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"ipv4_addr\":\"192.168.1.5\""));
	TEST_ASSERT_TRUE(body_has("\"ipv4_mask\":\"255.255.255.0\""));
	TEST_ASSERT_TRUE(body_has("\"mac\":\"02:00:00:00:00:ef\""));
	TEST_ASSERT_TRUE(body_has("\"offset_ns\":-17"));

	get_auth("/api/v1/status/ptp");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"clock_class\":6"));

	/*
	 * The profile conformance disclosure. Before this existed an operator on
	 * a C37.238 unit got `"alarms":8` and nothing else — core/ptp's headers
	 * claimed the deviation was "reported" while the only renderer for it was
	 * absent from the linked image. These four fields ARE the report.
	 */
	TEST_ASSERT_TRUE(body_has("\"profile\":3"));
	TEST_ASSERT_TRUE(body_has("\"profile_name\":\"C37.238\""));
	TEST_ASSERT_TRUE(body_has("\"deviations\":7"));
	TEST_ASSERT_TRUE(body_has("\"deviation_text\":\""
				  FAKE_PTP_DEVIATION_TEXT "\""));

	/*
	 * alarm_names: every SET bit that HAS a name, in ascending bit order,
	 * with the unnamed bit 4 skipped rather than rendered as null or "".
	 * Asserted as one exact substring so a reordering or a stray element
	 * fails — checking membership one name at a time would not.
	 */
	TEST_ASSERT_TRUE(body_has(
		"\"alarm_names\":[\"NOT_BEST_MASTER\",\"FAULTY\",\"TX_ERROR\","
		"\"PROFILE_UNSUPPORTED\",\"DISPLACED_WHILE_LOCKED\"]"));
	/* The fault namespace must not be what named them. */
	TEST_ASSERT_FALSE(body_has("\"alarm_names\":[\"GNSS_LOST\""));

	get_auth("/api/v1/status/alarms");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"name\":\"GNSS_LOST\""));
	TEST_ASSERT_TRUE(body_has("\"id\":3"));
	TEST_ASSERT_TRUE(body_has("\"id\":40"));
	TEST_ASSERT_TRUE(body_has("\"latched\":true"));

	/* The whole document. */
	get_auth("/api/v1/status");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"summary\":{"));
	TEST_ASSERT_TRUE(body_has("\"ptp\":{"));

	/* An unknown or non-status group. */
	get_auth("/api/v1/status/bogus");
	TEST_ASSERT_EQUAL_UINT(404U, g_resp.status);
	get_auth("/api/v1/status/logs");
	TEST_ASSERT_EQUAL_UINT(404U, g_resp.status);

	/* A provider that fails yields 503, not a half-answer. */
	fk.quality_rc = -EIO;
	get_auth("/api/v1/status/timing");
	TEST_ASSERT_EQUAL_UINT(503U, g_resp.status);
	fk.quality_rc = 0;
	fk.health_rc = -EIO;
	get_auth("/api/v1/status/power");
	TEST_ASSERT_EQUAL_UINT(503U, g_resp.status);
	fk.health_rc = 0;
	fk.gnss_rc = -EIO;
	get_auth("/api/v1/status/gnss");
	TEST_ASSERT_EQUAL_UINT(503U, g_resp.status);
	fk.gnss_rc = 0;
	fk.net_rc = -EIO;
	get_auth("/api/v1/status/net");
	TEST_ASSERT_EQUAL_UINT(503U, g_resp.status);
	fk.net_rc = 0;
	fk.ptp_rc = -EIO;
	get_auth("/api/v1/status/ptp");
	TEST_ASSERT_EQUAL_UINT(503U, g_resp.status);
	fk.ptp_rc = 0;

	/* Summary degrades when only one of its two sources answers. */
	fk.svc_rc = -EIO;
	get_auth("/api/v1/status/summary");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	fk.svc_rc = 0;
	fk.quality_rc = -EIO;
	get_auth("/api/v1/status/summary");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	fk.quality_rc = 0;

	/* A response buffer too small to hold the document is a 500, never a
	 * truncated body. */
	{
		char tiny[64];
		char raw[256];
		size_t rn = (size_t)snprintf(raw, sizeof(raw),
					     "GET /api/v1/status/power "
					     "HTTP/1.1\r\nHost: h\r\n"
					     "Cookie: sts_session=%s\r\n\r\n",
					     g_token);

		TEST_ASSERT_EQUAL_INT(0, http_parse_request(raw, rn, &g_req));
		rest_resp_init(&g_resp, tiny, sizeof(tiny));
		TEST_ASSERT_EQUAL_INT(0, rest_dispatch(&g_rest, &g_req, NULL, 0U,
						       &g_resp));
		TEST_ASSERT_EQUAL_UINT(500U, g_resp.status);
	}
}

/*
 * The skyplot's data path through the encoder (spec §336, §371).
 *
 * pv_gnss() used to hardcode `detail_available = false` and n_sats = 0, so this
 * array was serialised faithfully and always empty and no test could tell. The
 * provider now fills it from sts_gnss_sky(); these cases are the encoder's half
 * of that — the decision logic behind the provider is tests/host/test_web_sky.c.
 */
static void test_gnss_satellites(void)
{
	setup_rest((uint8_t)WEB_ROLE_VIEWER);

	/* ---- a fresh list serialises every field of every entry --------- */
	g_pv.gnss = pv_gnss;
	get_auth("/api/v1/status/gnss");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"detail_available\":true"));
	TEST_ASSERT_TRUE(body_has(
		"{\"constellation\":\"gps\",\"sv\":1,\"cno\":44,\"elev\":65,"
		"\"azim\":180,\"used\":true}"));
	TEST_ASSERT_TRUE(body_has(
		"{\"constellation\":\"galileo\",\"sv\":11,\"cno\":0,\"elev\":0,"
		"\"azim\":0,\"used\":true}"));
	TEST_ASSERT_EQUAL_UINT(3U, body_count("\"constellation\":"));

	/* ---- empty but fresh is DATA, not absence ----------------------- */
	g_pv.gnss = pv_gnss_empty;
	get_auth("/api/v1/status/gnss");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"detail_available\":true"));
	TEST_ASSERT_TRUE(body_has("\"sat_age_ms\":250"));
	TEST_ASSERT_TRUE(body_has("\"satellites\":[]"));
	TEST_ASSERT_EQUAL_UINT(0U, body_count("\"constellation\":"));

	/* ---- stale: no list, but the age still tells the operator why --- */
	g_pv.gnss = pv_gnss_stale;
	get_auth("/api/v1/status/gnss");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"detail_available\":false"));
	TEST_ASSERT_TRUE(body_has("\"sat_age_ms\":412345"));
	TEST_ASSERT_TRUE(body_has("\"satellites\":[]"));

	/* ---- never heard from: age is null, not zero -------------------- */
	g_pv.gnss = pv_gnss_never;
	get_auth("/api/v1/status/gnss");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"detail_available\":false"));
	TEST_ASSERT_TRUE(body_has("\"sat_age_ms\":null"));

	/* ---- the array bound, exactly -------------------------------- */
	g_pv.gnss = pv_gnss_max;
	get_auth("/api/v1/status/gnss");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_SAT_MAX,
			       body_count("\"constellation\":"));
	TEST_ASSERT_TRUE(body_has("\"sv\":200"));
	TEST_ASSERT_TRUE(body_has("\"sv\":231")); /* 200 + 32 - 1 */
	TEST_ASSERT_TRUE(body_has("\"cno\":255"));
	TEST_ASSERT_TRUE(body_has("\"elev\":-128"));
	TEST_ASSERT_TRUE(body_has("\"azim\":359"));
	TEST_ASSERT_TRUE(body_has("\"sat_age_ms\":4294967295"));

	/* ---- and one past it: clamped, never walked past ---------------- */
	g_pv.gnss = pv_gnss_over;
	get_auth("/api/v1/status/gnss");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_SAT_MAX,
			       body_count("\"constellation\":"));
	/* The last legal entry is present, so the clamp dropped the extra one
	 * rather than the last real one. */
	TEST_ASSERT_TRUE(body_has("\"sv\":199"));

	/*
	 * ---- the boundary the JSON writer itself can get wrong -----------
	 *
	 * A full satellite list is ~2.6 KB of growth in a response the net area
	 * serves from a fixed STS_WEB_RESP_SIZE buffer. web_jw_finish() answers
	 * -ENOSPC on overflow, rest_dispatch() turns that into a 500, and
	 * ws_run() sends no telemetry frame at all — so the failure mode of
	 * getting this wrong is a management plane that goes quiet exactly when
	 * a unit finally has a full sky. Measure it against the real buffer.
	 */
	{
		static char frame[STS_WEB_RESP_SIZE];
		char tight[512];
		int n;

		g_pv.gnss = pv_gnss_max;

		n = rest_encode_telemetry(&g_rest, WSS_GRP_ALL, 1U, frame,
					  sizeof(frame));
		TEST_ASSERT_TRUE(n > 0);
		TEST_ASSERT_TRUE((size_t)n < sizeof(frame));
		frame[n] = '\0';
		TEST_ASSERT_NOT_NULL(strstr(frame, "\"gnss\":{"));
		TEST_ASSERT_NOT_NULL(strstr(frame, "\"power\":{"));
		printf("worst-case telemetry frame: %d B of %u B\n", n,
		       (unsigned int)sizeof(frame));
		/* Headroom, so the next field added to any group is not the one
		 * that silently breaks this. */
		TEST_ASSERT_TRUE((size_t)n < (sizeof(frame) - 1024U));

		/* The whole /api/v1/status document, same worst case. */
		get_auth("/api/v1/status");
		TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
		TEST_ASSERT_TRUE(g_resp.body_len < STS_WEB_RESP_SIZE);
		printf("worst-case /api/v1/status: %u B of %u B\n",
		       (unsigned int)g_resp.body_len,
		       (unsigned int)STS_WEB_RESP_SIZE);

		/* And when it genuinely does not fit, it says so rather than
		 * emitting a truncated document. */
		TEST_ASSERT_EQUAL_INT(-ENOSPC,
				      rest_encode_telemetry(&g_rest,
							    WSS_GRP_GNSS, 1U,
							    tight,
							    sizeof(tight)));
	}

	g_pv.gnss = pv_gnss;
}

static void test_missing_providers(void)
{
	setup_rest((uint8_t)WEB_ROLE_ADMIN);
	memset(&g_pv, 0, sizeof(g_pv));

	get_auth("/api/v1/status/timing");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("no_provider"));
	get_auth("/api/v1/status/summary");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	get_auth("/api/v1/status/alarms");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	get_auth("/api/v1/firmware");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	get_auth("/api/v1/security/tls");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/gnss/survey", "{\"action\":\"start\"}");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/gnss/position",
		  "{\"ecef_x_cm\":1,\"ecef_y_cm\":2,\"ecef_z_cm\":3}");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/timing/reference", "{\"mode\":\"auto\"}");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/services/ntp", "{\"enable\":true}");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/calibration/run", "{\"procedure\":\"holdover\"}");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/firmware/begin", "{\"size\":1}");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	request("POST", "/api/v1/firmware/data?offset=0",
		REQ_COOKIE | REQ_CSRF, "application/octet-stream", "x", 1U);
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/firmware/end", NULL);
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/firmware/confirm", NULL);
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/firmware/revert", NULL);
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/security/tls", "pem");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/security/csr", NULL);
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/reboot", NULL);
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/factory-reset", "{\"confirm\":\"FACTORY\"}");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);

	/* No cfg registry: every config route reports it rather than guessing. */
	g_rest.cfg = NULL;
	get_auth("/api/v1/config");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	get_auth("/api/v1/config/net.dhcp");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	request("PUT", "/api/v1/config", REQ_COOKIE | REQ_CSRF,
		"application/json", "{}", 2U);
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/config/commit", NULL);
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/config/revert", NULL);
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	get_auth("/api/v1/config/export");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/config/import", "x");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	get_auth("/api/v1/calibration");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);

	/* No log ring. */
	g_rest.log = NULL;
	get_auth("/api/v1/logs");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);

	/*
	 * No auth registry. The three RF_OPEN routes reach their handlers and
	 * report 501; the admin-gated security routes never get that far,
	 * because with no registry the caller resolves to `viewer` and the role
	 * check refuses first. That ordering is the point — a build with no
	 * credential store must not expose an admin route at all.
	 */
	g_rest.auth = NULL;
	post_auth("/api/v1/auth/login",
		  "{\"user\":\"admin\",\"password\":\"x\"}");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	post_auth("/api/v1/auth/logout", NULL);
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	get_auth("/api/v1/auth/session");
	TEST_ASSERT_EQUAL_UINT(501U, g_resp.status);
	get_auth("/api/v1/security/users");
	TEST_ASSERT_EQUAL_UINT(403U, g_resp.status);
	post_auth("/api/v1/security/password",
		  "{\"user\":\"admin\",\"password\":\"newpassword\"}");
	TEST_ASSERT_EQUAL_UINT(401U, g_resp.status);
}

/* ---- authorisation ------------------------------------------------------ */

static void test_unauthenticated_is_401(void)
{
	setup_rest((uint8_t)WEB_ROLE_ADMIN);

	/* No cookie at all. */
	request("GET", "/api/v1/status/summary", 0U, NULL, NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(401U, g_resp.status);
	TEST_ASSERT_TRUE(strstr(g_resp.extra, "WWW-Authenticate:") != NULL);

	/* A bogus cookie. */
	{
		const char *raw = "GET /api/v1/status/summary HTTP/1.1\r\n"
				  "Host: h\r\nCookie: sts_session=deadbeef\r\n"
				  "\r\n";

		TEST_ASSERT_EQUAL_INT(0, http_parse_request(raw, strlen(raw),
							    &g_req));
		rest_resp_init(&g_resp, g_respbuf, sizeof(g_respbuf));
		TEST_ASSERT_EQUAL_INT(0, rest_dispatch(&g_rest, &g_req, NULL, 0U,
						       &g_resp));
		TEST_ASSERT_EQUAL_UINT(401U, g_resp.status);
	}

	/* A bearer token works as well as a cookie. */
	{
		char raw[256];
		size_t n = (size_t)snprintf(raw, sizeof(raw),
					    "GET /api/v1/status/summary "
					    "HTTP/1.1\r\nHost: h\r\n"
					    "Authorization: Bearer %s\r\n\r\n",
					    g_token);

		TEST_ASSERT_EQUAL_INT(0, http_parse_request(raw, n, &g_req));
		rest_resp_init(&g_resp, g_respbuf, sizeof(g_respbuf));
		TEST_ASSERT_EQUAL_INT(0, rest_dispatch(&g_rest, &g_req, NULL, 0U,
						       &g_resp));
		TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	}

	/* The open routes need nothing. */
	request("GET", "/api/v1/auth/session", 0U, NULL, NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"authenticated\":false"));
	TEST_ASSERT_TRUE(body_has("\"role\":\"none\""));

	/* rest_resolve_auth() tolerates missing pieces. */
	{
		rest_authctx_t a;

		rest_resolve_auth(NULL, &g_req, &a);
		TEST_ASSERT_EQUAL_UINT(WEB_ROLE_NONE, a.role);
		rest_resolve_auth(&g_rest, NULL, &a);
		TEST_ASSERT_EQUAL_UINT(WEB_ROLE_NONE, a.role);
		rest_resolve_auth(&g_rest, &g_req, NULL);
		g_rest.auth = NULL;
		rest_resolve_auth(&g_rest, &g_req, &a);
		TEST_ASSERT_EQUAL_UINT(WEB_ROLE_NONE, a.role);
	}
}

static void test_role_enforcement(void)
{
	/* A viewer may read but not write, and may not touch admin routes. */
	setup_rest((uint8_t)WEB_ROLE_VIEWER);
	get_auth("/api/v1/status/summary");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	get_auth("/api/v1/logs");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	get_auth("/api/v1/config");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);

	post_auth("/api/v1/config/commit", NULL);
	TEST_ASSERT_EQUAL_UINT(403U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("insufficient_role"));
	post_auth("/api/v1/gnss/survey", "{\"action\":\"start\"}");
	TEST_ASSERT_EQUAL_UINT(403U, g_resp.status);
	get_auth("/api/v1/config/export");
	TEST_ASSERT_EQUAL_UINT(403U, g_resp.status);
	get_auth("/api/v1/security/users");
	TEST_ASSERT_EQUAL_UINT(403U, g_resp.status);
	post_auth("/api/v1/reboot", NULL);
	TEST_ASSERT_EQUAL_UINT(403U, g_resp.status);
	post_auth("/api/v1/firmware/begin", "{\"size\":1}");
	TEST_ASSERT_EQUAL_UINT(403U, g_resp.status);

	/* An operator may control the box but not its security or firmware. */
	setup_rest((uint8_t)WEB_ROLE_OPERATOR);
	post_auth("/api/v1/gnss/survey", "{\"action\":\"start\"}");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(fk.survey_started);
	post_auth("/api/v1/timing/reference", "{\"mode\":\"rb\"}");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	post_auth("/api/v1/config/commit", NULL);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	get_auth("/api/v1/security/users");
	TEST_ASSERT_EQUAL_UINT(403U, g_resp.status);
	post_auth("/api/v1/security/tls", "pem");
	TEST_ASSERT_EQUAL_UINT(403U, g_resp.status);
	post_auth("/api/v1/firmware/end", NULL);
	TEST_ASSERT_EQUAL_UINT(403U, g_resp.status);
	post_auth("/api/v1/factory-reset", "{\"confirm\":\"FACTORY\"}");
	TEST_ASSERT_EQUAL_UINT(403U, g_resp.status);

	/* An admin may do all of it. */
	setup_rest((uint8_t)WEB_ROLE_ADMIN);
	get_auth("/api/v1/security/users");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	post_auth("/api/v1/firmware/confirm", NULL);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
}

static void test_csrf_enforcement(void)
{
	cfg_commit_res_t res;

	setup_rest((uint8_t)WEB_ROLE_ADMIN);

	/* A mutating request with a session but no CSRF header is refused. */
	request("POST", "/api/v1/config/revert", REQ_COOKIE, NULL, NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(403U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("csrf_failed"));

	/* A wrong CSRF token, right length. */
	{
		char raw[512];
		char bad[AUTH_WEB_CSRF_LEN + 1U];
		size_t n;

		memcpy(bad, g_csrf, sizeof(bad));
		bad[0] = (char)((bad[0] == 'a') ? 'b' : 'a');
		n = (size_t)snprintf(raw, sizeof(raw),
				     "POST /api/v1/config/revert HTTP/1.1\r\n"
				     "Host: h\r\nCookie: sts_session=%s\r\n"
				     "X-CSRF-Token: %s\r\n"
				     "Content-Length: 0\r\n\r\n",
				     g_token, bad);
		TEST_ASSERT_EQUAL_INT(0, http_parse_request(raw, n, &g_req));
		rest_resp_init(&g_resp, g_respbuf, sizeof(g_respbuf));
		TEST_ASSERT_EQUAL_INT(0, rest_dispatch(&g_rest, &g_req, NULL, 0U,
						       &g_resp));
		TEST_ASSERT_EQUAL_UINT(403U, g_resp.status);
	}
	TEST_ASSERT_TRUE(g_rest.st.csrf_failures >= 2U);

	/* With the right token it goes through. */
	post_auth("/api/v1/config/revert", NULL);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);

	/*
	 * Invariant 1: turning `sec.auth.req` off opens READS, and must NOT open
	 * writes — a hostile page in the operator's browser is still a hostile
	 * page.
	 */
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_SEC_AUTH_REQUIRED,
					     0U));
	memset(&res, 0, sizeof(res));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, &res));
	TEST_ASSERT_FALSE(auth_web_required(&g_auth));

	request("GET", "/api/v1/status/summary", 0U, NULL, NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	request("GET", "/api/v1/logs", 0U, NULL, NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	/* ...but an admin-only read is still gated on the role. */
	request("GET", "/api/v1/security/users", 0U, NULL, NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(403U, g_resp.status);
	/* ...and every write still needs a session. */
	request("POST", "/api/v1/config/revert", 0U, NULL, NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(401U, g_resp.status);
	request("POST", "/api/v1/reboot", 0U, "application/json", NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(401U, g_resp.status);
	TEST_ASSERT_FALSE(fk.factory_called);
}

static void test_login_logout_routes(void)
{
	cfg_commit_res_t res;

	setup_rest((uint8_t)WEB_ROLE_NONE);
	TEST_ASSERT_EQUAL_INT(0, auth_web_user_add(&g_auth, "admin",
						   (uint8_t)WEB_ROLE_ADMIN,
						   (uint16_t)CFG_ID_SEC_ADMIN_PW));

	/* No credential provisioned yet. */
	request("POST", "/api/v1/auth/login", 0U, "application/json",
		"{\"user\":\"admin\",\"password\":\"" PW "\"}",
		strlen("{\"user\":\"admin\",\"password\":\"" PW "\"}"));
	TEST_ASSERT_EQUAL_UINT(503U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("no_credential"));

	TEST_ASSERT_EQUAL_INT(0, auth_web_set_password(&g_auth, 0U,
						       (const uint8_t *)PW,
						       PW_LEN));
	memset(&res, 0, sizeof(res));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, &res));
	TEST_ASSERT_EQUAL_INT(1, auth_web_reload(&g_auth));

	/* Malformed bodies. */
	request("POST", "/api/v1/auth/login", 0U, "application/json", NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	request("POST", "/api/v1/auth/login", 0U, "application/json",
		"{\"user\":\"admin\"}", 17U);
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	request("POST", "/api/v1/auth/login", 0U, "application/json",
		"{\"user\":7,\"password\":\"x\"}", 25U);
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	request("POST", "/api/v1/auth/login", 0U, "application/json",
		"{\"user\":\"admin\",\"password\":\"\\q\"}", 32U);
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);

	/* Wrong password. */
	request("POST", "/api/v1/auth/login", 0U, "application/json",
		"{\"user\":\"admin\",\"password\":\"nope-nope-nope\"}", 44U);
	TEST_ASSERT_EQUAL_UINT(401U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("invalid_credentials"));

	/* Right password: a Secure/HttpOnly/SameSite cookie and a CSRF token in
	 * the body, but never the session token in the body. */
	{
		const char *b = "{\"user\":\"admin\",\"password\":\"" PW "\"}";

		request("POST", "/api/v1/auth/login", 0U, "application/json", b,
			strlen(b));
	}
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(strstr(g_resp.extra, "Set-Cookie: sts_session=") !=
			 NULL);
	TEST_ASSERT_TRUE(strstr(g_resp.extra, "HttpOnly") != NULL);
	TEST_ASSERT_TRUE(strstr(g_resp.extra, "Secure") != NULL);
	TEST_ASSERT_TRUE(strstr(g_resp.extra, "SameSite=Strict") != NULL);
	TEST_ASSERT_TRUE(body_has("\"csrf_token\":\""));
	TEST_ASSERT_TRUE(body_has("\"role\":\"admin\""));
	TEST_ASSERT_FALSE(body_has("\"token\""));
	{
		/* Prove the session token really is absent from the body. */
		const auth_web_sess_t *s = auth_web_sess_at(&g_auth, 0U);

		TEST_ASSERT_NOT_NULL(s);
		TEST_ASSERT_NULL(strstr(body_str(), s->token));
		memcpy(g_token, s->token, sizeof(g_token));
		memcpy(g_csrf, s->csrf, sizeof(g_csrf));
	}

	/* The session probe now reports the session. */
	get_auth("/api/v1/auth/session");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"authenticated\":true"));
	TEST_ASSERT_TRUE(body_has("\"user\":\"admin\""));

	/* Logout clears the cookie. */
	post_auth("/api/v1/auth/logout", NULL);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(strstr(g_resp.extra, "Max-Age=0") != NULL);
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_session_count(&g_auth));
	/* Logging out with no session is not an error. */
	post_auth("/api/v1/auth/logout", NULL);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
}

static void test_login_throttle_route(void)
{
	unsigned int i;
	const char *bad = "{\"user\":\"admin\",\"password\":\"nope-nope-nope\"}";

	setup_rest((uint8_t)WEB_ROLE_ADMIN);
	for (i = 0U; i <= AUTH_WEB_FREE_TRIES; i++) {
		request("POST", "/api/v1/auth/login", 0U, "application/json",
			bad, strlen(bad));
		TEST_ASSERT_EQUAL_UINT(401U, g_resp.status);
	}
	/* Now throttled: 429 with a Retry-After. */
	request("POST", "/api/v1/auth/login", 0U, "application/json", bad,
		strlen(bad));
	TEST_ASSERT_EQUAL_UINT(429U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("throttled"));
	TEST_ASSERT_TRUE(strstr(g_resp.extra, "Retry-After: 2") != NULL);
}

/* ---- config ------------------------------------------------------------- */

static void test_secret_never_served(void)
{
	cfg_commit_res_t res;
	uint8_t blob[48];
	size_t i;

	setup_rest((uint8_t)WEB_ROLE_ADMIN);

	/* Put a recognisable pattern in both a NOEXPORT and a SECRET key. */
	for (i = 0U; i < sizeof(blob); i++) {
		blob[i] = 0xA5U;
	}
	TEST_ASSERT_EQUAL_INT(0, cfg_set_bytes(&g_cfg, CFG_ID_SEC_ADMIN_PW, blob,
					       sizeof(blob)));
	TEST_ASSERT_EQUAL_INT(0, cfg_set_bytes(&g_cfg, CFG_ID_SNMP_COMMUNITY,
					       (const uint8_t *)"s3cr3t", 6U));
	memset(&res, 0, sizeof(res));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, &res));

	/*
	 * The security group (0x0A holds the NOEXPORT credential) and the SNMP
	 * group (0x0B holds the SECRET community), as admin, without asking for
	 * secrets. A group-scoped listing is emitted whole, so this really does
	 * cover the two keys rather than whichever ones land on page one.
	 */
	get_auth("/api/v1/config?group=10");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"name\":\"sec.admin.pw\""));
	TEST_ASSERT_TRUE(body_has("\"secrets_included\":false"));
	TEST_ASSERT_NULL(strstr(body_str(), "a5a5a5"));
	TEST_ASSERT_TRUE(body_has("\"value_withheld\":true"));
	TEST_ASSERT_TRUE(body_has("\"value_set\":true"));

	get_auth("/api/v1/config?group=11");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"name\":\"snmp.community\""));
	TEST_ASSERT_NULL(strstr(body_str(), "s3cr3t"));
	TEST_ASSERT_TRUE(body_has("\"value_withheld\":true"));

	/* Asking for secrets as admin reveals the SECRET key... */
	get_auth("/api/v1/config?group=11&secrets=1");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"secrets_included\":true"));
	TEST_ASSERT_TRUE(body_has("s3cr3t"));
	/* ...but NEVER the NOEXPORT one, even with the flag and the role. */
	get_auth("/api/v1/config?group=10&secrets=1");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"secrets_included\":true"));
	TEST_ASSERT_TRUE(body_has("\"value_withheld\":true"));
	TEST_ASSERT_NULL(strstr(body_str(), "a5a5a5"));

	/* An unqualified listing pages instead of overflowing the buffer. */
	get_auth("/api/v1/config");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"count\":32"));
	TEST_ASSERT_TRUE(body_has("\"next_id\":"));
	TEST_ASSERT_FALSE(body_has("\"next_id\":0"));
	TEST_ASSERT_NULL(strstr(body_str(), "a5a5a5"));
	TEST_ASSERT_NULL(strstr(body_str(), "s3cr3t"));

	/* Single-key reads behave the same way. */
	get_auth("/api/v1/config/sec.admin.pw");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"value_withheld\":true"));
	TEST_ASSERT_NULL(strstr(body_str(), "a5a5"));
	get_auth("/api/v1/config/sec.admin.pw?secrets=1");
	TEST_ASSERT_TRUE(body_has("\"value_withheld\":true"));
	TEST_ASSERT_NULL(strstr(body_str(), "a5a5"));
	get_auth("/api/v1/config/snmp.community");
	TEST_ASSERT_TRUE(body_has("\"value_withheld\":true"));
	TEST_ASSERT_NULL(strstr(body_str(), "s3cr3t"));
	get_auth("/api/v1/config/snmp.community?secrets=1");
	TEST_ASSERT_TRUE(body_has("s3cr3t"));

	/* A TLV export must never carry the NOEXPORT blob, with or without the
	 * secrets flag. */
	get_auth("/api/v1/config/export");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(g_resp.body_is_binary);
	TEST_ASSERT_NULL(mem_find(g_respbuf, g_resp.body_len, blob, 16U));
	TEST_ASSERT_NULL(mem_find(g_respbuf, g_resp.body_len, "s3cr3t", 6U));
	get_auth("/api/v1/config/export?secrets=1");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_NULL(mem_find(g_respbuf, g_resp.body_len, blob, 16U));
	TEST_ASSERT_NOT_NULL(mem_find(g_respbuf, g_resp.body_len, "s3cr3t", 6U));

	/* A viewer cannot request secrets at all. */
	setup_rest((uint8_t)WEB_ROLE_VIEWER);
	TEST_ASSERT_EQUAL_INT(0, cfg_set_bytes(&g_cfg, CFG_ID_SNMP_COMMUNITY,
					       (const uint8_t *)"s3cr3t", 6U));
	memset(&res, 0, sizeof(res));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, &res));
	get_auth("/api/v1/config?group=11&secrets=1");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"name\":\"snmp.community\""));
	TEST_ASSERT_TRUE(body_has("\"secrets_included\":false"));
	TEST_ASSERT_NULL(strstr(body_str(), "s3cr3t"));
}

static void test_config_read(void)
{
	setup_rest((uint8_t)WEB_ROLE_VIEWER);

	get_auth("/api/v1/config");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"schema_version\":1"));
	TEST_ASSERT_TRUE(body_has("\"name\":\"net.dhcp\""));
	TEST_ASSERT_TRUE(body_has("\"id\":\"0x0101\""));

	/* Group filter. */
	get_auth("/api/v1/config?group=1");
	TEST_ASSERT_TRUE(body_has("\"name\":\"net.dhcp\""));
	TEST_ASSERT_NULL(strstr(body_str(), "\"name\":\"ntp.enable\""));

	/* Paging: an unqualified listing pages at REST_CONFIG_PAGE_DEFAULT. */
	get_auth("/api/v1/config");
	TEST_ASSERT_TRUE(body_has("\"count\":32"));
	get_auth("/api/v1/config?max=2");
	TEST_ASSERT_TRUE(body_has("\"count\":2"));
	TEST_ASSERT_TRUE(body_has("\"next_id\":259")); /* 0x0103 */
	get_auth("/api/v1/config?start=513&max=1");    /* 0x0201 */
	TEST_ASSERT_TRUE(body_has("\"name\":\"ntp.enable\""));

	/* Single key by name, by hex id and by an unknown selector. */
	get_auth("/api/v1/config/tim.tau.s");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"value\":200"));
	TEST_ASSERT_TRUE(body_has("\"min\":10"));
	get_auth("/api/v1/config/0x0601");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"name\":\"tim.tau.s\""));
	get_auth("/api/v1/config/0X0601");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	get_auth("/api/v1/config/nope");
	TEST_ASSERT_EQUAL_UINT(404U, g_resp.status);
	get_auth("/api/v1/config/0xZZZZ");
	TEST_ASSERT_EQUAL_UINT(404U, g_resp.status);
	get_auth("/api/v1/config/0x99999");
	TEST_ASSERT_EQUAL_UINT(404U, g_resp.status);
	get_auth("/api/v1/config/0xFFFF");
	TEST_ASSERT_EQUAL_UINT(404U, g_resp.status);
	get_auth("/api/v1/config/");
	TEST_ASSERT_EQUAL_UINT(404U, g_resp.status);

	/* Each value type renders. */
	get_auth("/api/v1/config/net.dhcp"); /* BOOL */
	TEST_ASSERT_TRUE(body_has("\"value\":true"));
	get_auth("/api/v1/config/net.hostname"); /* STR */
	TEST_ASSERT_TRUE(body_has("\"value\":\"meridian\""));
	TEST_ASSERT_TRUE(body_has("\"maxlen\":63"));
	get_auth("/api/v1/config/ptp.log.sync"); /* I32 */
	TEST_ASSERT_TRUE(body_has("\"value\":0"));
	TEST_ASSERT_TRUE(body_has("\"min\":-7"));
	get_auth("/api/v1/config/cal.tempco"); /* F32 */
	TEST_ASSERT_TRUE(body_has("\"value\":0.000000"));
	TEST_ASSERT_TRUE(body_has("\"min\":-100.000000"));
	get_auth("/api/v1/config/net.mgmt.acl"); /* BLOB, empty */
	TEST_ASSERT_TRUE(body_has("\"value\":\"\""));
}

static void test_config_write(void)
{
	setup_rest((uint8_t)WEB_ROLE_OPERATOR);

	/* Stage a mixed set: numeric, bool as 0/1, string, i32, float, blob. */
	request("PUT", "/api/v1/config", REQ_COOKIE | REQ_CSRF,
		"application/json",
		"{\"tim.tau.s\":300,\"0x0101\":0,\"net.hostname\":\"box1\","
		"\"ptp.log.sync\":-2,\"cal.tempco\":{\"micro\":-1500000},"
		"\"net.mgmt.acl\":\"0aff\",\"ntp.rate.burst\":16}",
		strlen("{\"tim.tau.s\":300,\"0x0101\":0,\"net.hostname\":\"box1\","
		       "\"ptp.log.sync\":-2,\"cal.tempco\":{\"micro\":-1500000},"
		       "\"net.mgmt.acl\":\"0aff\",\"ntp.rate.burst\":16}"));
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"staged\":7"));
	TEST_ASSERT_TRUE(body_has("\"rejected\":0"));
	TEST_ASSERT_TRUE(body_has("\"commit_required\":true"));

	/* The staged value is visible but not yet live. */
	get_auth("/api/v1/config/tim.tau.s");
	TEST_ASSERT_TRUE(body_has("\"staged\":true"));
	TEST_ASSERT_TRUE(body_has("\"value\":200"));
	TEST_ASSERT_TRUE(body_has("\"staged_value\":300"));

	/* Commit runs through the hook so the appliers fire. */
	post_auth("/api/v1/config/commit", NULL);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"applied\":7"));
	TEST_ASSERT_TRUE(body_has("\"persisted\":true"));
	TEST_ASSERT_EQUAL_INT(1, commit_hook_calls);
	{
		uint64_t v = 0U;
		int32_t s = 0;
		float f = 0.0f;

		TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S,
						     &v));
		TEST_ASSERT_EQUAL_UINT64(300U, v);
		TEST_ASSERT_EQUAL_INT(0, cfg_get_i32(&g_cfg,
						     CFG_ID_PTP_LOG_SYNC, &s));
		TEST_ASSERT_EQUAL_INT(-2, s);
		TEST_ASSERT_EQUAL_INT(0, cfg_get_f32(&g_cfg,
						     CFG_ID_CAL_TEMPCO_PPB_C,
						     &f));
		TEST_ASSERT_TRUE(f < -1.4f && f > -1.6f);
	}

	/* Revert drops a staged set. */
	request("PUT", "/api/v1/config", REQ_COOKIE | REQ_CSRF,
		"application/json", "{\"tim.tau.s\":400}", 17U);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	post_auth("/api/v1/config/revert", NULL);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"dropped\":1"));

	/* Per-key errors: unknown key, out of range, type mismatch. */
	request("PUT", "/api/v1/config", REQ_COOKIE | REQ_CSRF,
		"application/json",
		"{\"nope\":1,\"tim.tau.s\":99999,\"net.dhcp\":\"yes\","
		"\"net.hostname\":7,\"cal.tempco\":\"x\","
		"\"net.mgmt.acl\":\"zz\",\"ptp.log.sync\":\"x\"}",
		strlen("{\"nope\":1,\"tim.tau.s\":99999,\"net.dhcp\":\"yes\","
		       "\"net.hostname\":7,\"cal.tempco\":\"x\","
		       "\"net.mgmt.acl\":\"zz\",\"ptp.log.sync\":\"x\"}"));
	TEST_ASSERT_EQUAL_UINT(422U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("unknown_key"));
	TEST_ASSERT_TRUE(body_has("out_of_range"));
	TEST_ASSERT_TRUE(body_has("type_mismatch"));
	TEST_ASSERT_TRUE(body_has("\"staged\":0"));

	/* Over-long string / blob values report a range error. */
	{
		char big[400];
		size_t o;

		o = (size_t)snprintf(big, sizeof(big), "{\"net.hostname\":\"");
		memset(&big[o], 'x', 100U);
		o += 100U;
		o += (size_t)snprintf(&big[o], sizeof(big) - o, "\"}");
		request("PUT", "/api/v1/config", REQ_COOKIE | REQ_CSRF,
			"application/json", big, o);
		TEST_ASSERT_EQUAL_UINT(422U, g_resp.status);
		TEST_ASSERT_TRUE(body_has("out_of_range"));
	}

	/* An i32 out of int32 range and a u64 that is not a number. */
	request("PUT", "/api/v1/config", REQ_COOKIE | REQ_CSRF,
		"application/json",
		"{\"ptp.log.sync\":99999999999,\"tim.tau.s\":true}", 46U);
	TEST_ASSERT_EQUAL_UINT(422U, g_resp.status);

	/* Malformed bodies. */
	request("PUT", "/api/v1/config", REQ_COOKIE | REQ_CSRF,
		"application/json", NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("empty_body"));
	request("PUT", "/api/v1/config", REQ_COOKIE | REQ_CSRF,
		"application/json", "[1,2]", 5U);
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("not_an_object"));
	request("PUT", "/api/v1/config", REQ_COOKIE | REQ_CSRF,
		"application/json", "  ", 2U);
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	request("PUT", "/api/v1/config", REQ_COOKIE | REQ_CSRF,
		"application/json", "{\"a\":1,}", 8U);
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("malformed_json"));

}

/* A cross-field hook that refuses every candidate tree. */
static int reject_all(const cfg_ctx_t *c, void *u)
{
	(void)c;
	(void)u;
	return -EPERM;
}

static void test_config_commit_failure(void)
{
	setup_rest((uint8_t)WEB_ROLE_OPERATOR);
	TEST_ASSERT_EQUAL_INT(0, cfg_set_validate_hook(&g_cfg, reject_all,
						       NULL));
	request("PUT", "/api/v1/config", REQ_COOKIE | REQ_CSRF,
		"application/json", "{\"tim.tau.s\":300}", 17U);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	post_auth("/api/v1/config/commit", NULL);
	TEST_ASSERT_EQUAL_UINT(422U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("validation_failed"));
	/* Still staged, so the operator can fix it. */
	TEST_ASSERT_EQUAL_UINT(1U, cfg_staged_count(&g_cfg));
	TEST_ASSERT_EQUAL_INT(0, cfg_set_validate_hook(&g_cfg, NULL, NULL));

	/* Without a commit hook the router falls back to cfg_commit(). */
	g_rest.commit = NULL;
	post_auth("/api/v1/config/commit", NULL);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
}

static void test_config_import_export(void)
{
	static uint8_t blob[4096];
	size_t n;

	setup_rest((uint8_t)WEB_ROLE_ADMIN);

	/* Round trip: export, change a value, import, check it came back. */
	get_auth("/api/v1/config/export");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(strstr(g_resp.extra, "Content-Disposition:") != NULL);
	n = g_resp.body_len;
	TEST_ASSERT_TRUE(n > 0U && n <= sizeof(blob));
	memcpy(blob, g_respbuf, n);

	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 999U));
	{
		cfg_commit_res_t res;

		memset(&res, 0, sizeof(res));
		TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, &res));
	}
	request("POST", "/api/v1/config/import", REQ_COOKIE | REQ_CSRF,
		"application/octet-stream", (const char *)blob, n);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	{
		uint64_t v = 0U;

		TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S,
						     &v));
		TEST_ASSERT_EQUAL_UINT64(200U, v);
	}

	/* Import errors map onto distinct codes. */
	request("POST", "/api/v1/config/import", REQ_COOKIE | REQ_CSRF,
		"application/octet-stream", NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	request("POST", "/api/v1/config/import", REQ_COOKIE | REQ_CSRF,
		"application/octet-stream", "not a config blob at all", 24U);
	TEST_ASSERT_EQUAL_UINT(422U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("bad_magic"));

	/* Corrupt the CRC trailer. */
	blob[n - 1U] ^= 0xFFU;
	request("POST", "/api/v1/config/import", REQ_COOKIE | REQ_CSRF,
		"application/octet-stream", (const char *)blob, n);
	TEST_ASSERT_EQUAL_UINT(422U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("bad_crc"));
	blob[n - 1U] ^= 0xFFU;

	/* Bump the schema version so the import is rejected as unmigratable. */
	blob[4] = 0x63U;
	request("POST", "/api/v1/config/import", REQ_COOKIE | REQ_CSRF,
		"application/octet-stream", (const char *)blob, n);
	TEST_ASSERT_EQUAL_UINT(422U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("schema_version"));

	/* An export that does not fit is a 500, never a partial file. */
	{
		char raw[256];
		char tiny[64];
		size_t rn = (size_t)snprintf(raw, sizeof(raw),
					     "GET /api/v1/config/export "
					     "HTTP/1.1\r\nHost: h\r\n"
					     "Cookie: sts_session=%s\r\n\r\n",
					     g_token);

		TEST_ASSERT_EQUAL_INT(0, http_parse_request(raw, rn, &g_req));
		rest_resp_init(&g_resp, tiny, sizeof(tiny));
		TEST_ASSERT_EQUAL_INT(0, rest_dispatch(&g_rest, &g_req, NULL, 0U,
						       &g_resp));
		TEST_ASSERT_EQUAL_UINT(500U, g_resp.status);
	}
}

/* ---- control routes ----------------------------------------------------- */

static void test_control_routes(void)
{
	setup_rest((uint8_t)WEB_ROLE_OPERATOR);

	/* GNSS survey. */
	post_auth("/api/v1/gnss/survey", "{\"action\":\"stop\"}");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_FALSE(fk.survey_started);
	post_auth("/api/v1/gnss/survey", "{\"action\":\"bogus\"}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	post_auth("/api/v1/gnss/survey", "{}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	post_auth("/api/v1/gnss/survey", "{\"action\":7}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	post_auth("/api/v1/gnss/survey", NULL);
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	fk.survey_rc = -EBUSY;
	post_auth("/api/v1/gnss/survey", "{\"action\":\"start\"}");
	TEST_ASSERT_EQUAL_UINT(409U, g_resp.status);
	fk.survey_rc = 0;

	/* Fixed position, including the sanity bound. */
	post_auth("/api/v1/gnss/position",
		  "{\"ecef_x_cm\":100,\"ecef_y_cm\":-200,\"ecef_z_cm\":300}");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_EQUAL_INT64(100, fk.ecef[0]);
	TEST_ASSERT_EQUAL_INT64(-200, fk.ecef[1]);
	post_auth("/api/v1/gnss/position", "{\"ecef_x_cm\":1}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	post_auth("/api/v1/gnss/position",
		  "{\"ecef_x_cm\":\"a\",\"ecef_y_cm\":1,\"ecef_z_cm\":2}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	post_auth("/api/v1/gnss/position",
		  "{\"ecef_x_cm\":999999999999,\"ecef_y_cm\":1,"
		  "\"ecef_z_cm\":2}");
	TEST_ASSERT_EQUAL_UINT(422U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("coordinates_out_of_range"));
	fk.fixed_rc = -EBUSY;
	post_auth("/api/v1/gnss/position",
		  "{\"ecef_x_cm\":1,\"ecef_y_cm\":2,\"ecef_z_cm\":3}");
	TEST_ASSERT_EQUAL_UINT(409U, g_resp.status);
	fk.fixed_rc = 0;

	/* Reference override, including the guard refusal. */
	post_auth("/api/v1/timing/reference", "{\"mode\":\"auto\"}");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_EQUAL_UINT(REST_REF_AUTO, fk.ref_mode);
	post_auth("/api/v1/timing/reference", "{\"mode\":\"ocxo\"}");
	TEST_ASSERT_EQUAL_UINT(REST_REF_OCXO, fk.ref_mode);
	post_auth("/api/v1/timing/reference", "{\"mode\":\"rb\"}");
	TEST_ASSERT_EQUAL_UINT(REST_REF_RB, fk.ref_mode);
	post_auth("/api/v1/timing/reference", "{\"mode\":\"gps\"}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	post_auth("/api/v1/timing/reference", "{}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	fk.ref_rc = -EPERM;
	post_auth("/api/v1/timing/reference", "{\"mode\":\"rb\"}");
	TEST_ASSERT_EQUAL_UINT(409U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("guard_not_satisfied"));
	fk.ref_rc = -EIO;
	post_auth("/api/v1/timing/reference", "{\"mode\":\"rb\"}");
	TEST_ASSERT_EQUAL_UINT(409U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"error\":\"rejected\""));
	fk.ref_rc = 0;

	/* Service enable/disable. */
	post_auth("/api/v1/services/ntp", "{\"enable\":false}");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_EQUAL_UINT(REST_SVC_NTP, fk.svc_id);
	TEST_ASSERT_FALSE(fk.svc_enable);
	post_auth("/api/v1/services/syslog", "{\"enable\":true}");
	TEST_ASSERT_EQUAL_UINT(REST_SVC_SYSLOG, fk.svc_id);
	post_auth("/api/v1/services/nope", "{\"enable\":true}");
	TEST_ASSERT_EQUAL_UINT(404U, g_resp.status);
	post_auth("/api/v1/services/ntp", "{}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	fk.svc_en_rc = -EBUSY;
	post_auth("/api/v1/services/ntp", "{\"enable\":true}");
	TEST_ASSERT_EQUAL_UINT(409U, g_resp.status);
	fk.svc_en_rc = 0;

	/* Calibration. */
	get_auth("/api/v1/calibration");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"name\":\"cal.pps.ns\""));
	TEST_ASSERT_TRUE(body_has("\"id\":\"holdover\""));
	post_auth("/api/v1/calibration/run",
		  "{\"procedure\":\"holdover\",\"arg\":86400}");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_EQUAL_UINT(REST_CAL_HOLDOVER, fk.cal_proc);
	post_auth("/api/v1/calibration/run", "{\"procedure\":\"ocxo_tune\"}");
	TEST_ASSERT_EQUAL_UINT(REST_CAL_OCXO_TUNE, fk.cal_proc);
	post_auth("/api/v1/calibration/run", "{\"procedure\":\"ina_trim\"}");
	TEST_ASSERT_EQUAL_UINT(REST_CAL_INA_TRIM, fk.cal_proc);
	post_auth("/api/v1/calibration/run", "{\"procedure\":\"compass\"}");
	TEST_ASSERT_EQUAL_UINT(REST_CAL_COMPASS, fk.cal_proc);
	post_auth("/api/v1/calibration/run", "{\"procedure\":\"nope\"}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	post_auth("/api/v1/calibration/run", "{\"procedure\":7}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	post_auth("/api/v1/calibration/run", "{}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	fk.cal_rc = -EBUSY;
	post_auth("/api/v1/calibration/run", "{\"procedure\":\"holdover\"}");
	TEST_ASSERT_EQUAL_UINT(409U, g_resp.status);
	fk.cal_rc = 0;
}

static void test_system_routes(void)
{
	setup_rest((uint8_t)WEB_ROLE_ADMIN);

	post_auth("/api/v1/reboot", NULL);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(g_resp.close);
	TEST_ASSERT_EQUAL_UINT(0U, fk.reboot_mode);
	post_auth("/api/v1/reboot", "{\"mode\":1}");
	TEST_ASSERT_EQUAL_UINT(1U, fk.reboot_mode);
	post_auth("/api/v1/reboot", "{\"mode\":9}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	post_auth("/api/v1/reboot", "{\"mode\":\"x\"}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	fk.reboot_rc = -EBUSY;
	post_auth("/api/v1/reboot", NULL);
	TEST_ASSERT_EQUAL_UINT(409U, g_resp.status);
	fk.reboot_rc = 0;

	/* Factory reset needs the explicit magic. */
	post_auth("/api/v1/factory-reset", NULL);
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	TEST_ASSERT_FALSE(fk.factory_called);
	post_auth("/api/v1/factory-reset", "{\"confirm\":\"nope\"}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	TEST_ASSERT_FALSE(fk.factory_called);
	post_auth("/api/v1/factory-reset", "{\"confirm\":\"FACTORY\"}");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(fk.factory_called);
	/* Every session is closed by a factory reset. */
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_session_count(&g_auth));

	setup_rest((uint8_t)WEB_ROLE_ADMIN);
	fk.factory_rc = -EBUSY;
	post_auth("/api/v1/factory-reset", "{\"confirm\":\"FACTORY\"}");
	TEST_ASSERT_EQUAL_UINT(409U, g_resp.status);
}

/* ---- firmware ----------------------------------------------------------- */

static void test_firmware_routes(void)
{
	static const char *const sha_hex =
		"00112233445566778899aabbccddeeff"
		"00112233445566778899aabbccddeeff";
	char body[160];

	setup_rest((uint8_t)WEB_ROLE_ADMIN);

	get_auth("/api/v1/firmware");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"version\":\"0.4.0.123\""));
	TEST_ASSERT_TRUE(body_has("\"chunk_max\":1024"));
	fk.fw_info_rc = -EIO;
	get_auth("/api/v1/firmware");
	TEST_ASSERT_EQUAL_UINT(503U, g_resp.status);
	fk.fw_info_rc = 0;

	/* begin */
	(void)snprintf(body, sizeof(body),
		       "{\"size\":400000,\"sha256\":\"%s\"}", sha_hex);
	post_auth("/api/v1/firmware/begin", body);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_EQUAL_UINT32(400000U, fk.fw_size);
	TEST_ASSERT_EQUAL_HEX8(0x00U, fk.fw_sha[0]);
	TEST_ASSERT_EQUAL_HEX8(0xFFU, fk.fw_sha[31]);

	post_auth("/api/v1/firmware/begin", "{\"sha256\":\"aa\"}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	post_auth("/api/v1/firmware/begin", "{\"size\":0}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	post_auth("/api/v1/firmware/begin", "{\"size\":400000}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("bad_sha256"));
	post_auth("/api/v1/firmware/begin",
		  "{\"size\":400000,\"sha256\":\"tooshort\"}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	(void)snprintf(body, sizeof(body),
		       "{\"size\":400000,\"sha256\":\"%.63szz\"}", sha_hex);
	post_auth("/api/v1/firmware/begin", body);
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	post_auth("/api/v1/firmware/begin", NULL);
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);

	(void)snprintf(body, sizeof(body),
		       "{\"size\":400000,\"sha256\":\"%s\"}", sha_hex);
	fk.fw_begin_rc = -ENOSPC;
	post_auth("/api/v1/firmware/begin", body);
	TEST_ASSERT_EQUAL_UINT(413U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("image_too_large"));
	fk.fw_begin_rc = -EBUSY;
	post_auth("/api/v1/firmware/begin", body);
	TEST_ASSERT_EQUAL_UINT(409U, g_resp.status);
	fk.fw_begin_rc = 0;

	/* data */
	fk.fw_next = 1024U;
	request("POST", "/api/v1/firmware/data?offset=0", REQ_COOKIE | REQ_CSRF,
		"application/octet-stream", "abcd", 4U);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"next_offset\":1024"));
	TEST_ASSERT_EQUAL_UINT32(0U, fk.fw_off);
	TEST_ASSERT_EQUAL_UINT32(4U, fk.fw_len);
	TEST_ASSERT_EQUAL_UINT32(1U, g_rest.st.fw_chunks);

	request("POST", "/api/v1/firmware/data", REQ_COOKIE | REQ_CSRF,
		"application/octet-stream", "abcd", 4U);
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("missing_offset"));
	request("POST", "/api/v1/firmware/data?offset=0", REQ_COOKIE | REQ_CSRF,
		"application/octet-stream", NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("empty_chunk"));

	/* An offset gap returns the frontier so the client can rewind. */
	fk.fw_data_rc = -EPROTO;
	fk.fw_next = 512U;
	request("POST", "/api/v1/firmware/data?offset=4096",
		REQ_COOKIE | REQ_CSRF, "application/octet-stream", "abcd", 4U);
	TEST_ASSERT_EQUAL_UINT(409U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("offset_gap"));
	TEST_ASSERT_TRUE(body_has("\"next_offset\":512"));
	fk.fw_data_rc = -EIO;
	request("POST", "/api/v1/firmware/data?offset=0", REQ_COOKIE | REQ_CSRF,
		"application/octet-stream", "abcd", 4U);
	TEST_ASSERT_EQUAL_UINT(500U, g_resp.status);
	fk.fw_data_rc = 0;

	/* end / confirm / revert */
	post_auth("/api/v1/firmware/end", NULL);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"size\":400000"));
	fk.fw_end_rc = -EBADMSG;
	post_auth("/api/v1/firmware/end", NULL);
	TEST_ASSERT_EQUAL_UINT(422U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("verify_failed"));
	fk.fw_end_rc = -EBUSY;
	post_auth("/api/v1/firmware/end", NULL);
	TEST_ASSERT_EQUAL_UINT(409U, g_resp.status);
	fk.fw_end_rc = 0;

	post_auth("/api/v1/firmware/confirm", NULL);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(fk.confirm_called);
	fk.fw_confirm_rc = -EBUSY;
	post_auth("/api/v1/firmware/confirm", NULL);
	TEST_ASSERT_EQUAL_UINT(409U, g_resp.status);
	fk.fw_confirm_rc = 0;

	post_auth("/api/v1/firmware/revert", NULL);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(fk.revert_called);
	fk.fw_revert_rc = -EBUSY;
	post_auth("/api/v1/firmware/revert", NULL);
	TEST_ASSERT_EQUAL_UINT(409U, g_resp.status);
	fk.fw_revert_rc = 0;
}

/* ---- security ----------------------------------------------------------- */

static void test_security_routes(void)
{
	setup_rest((uint8_t)WEB_ROLE_ADMIN);
	TEST_ASSERT_EQUAL_INT(1, auth_web_user_add(&g_auth, "opr",
						   (uint8_t)WEB_ROLE_OPERATOR,
						   0U));

	get_auth("/api/v1/security/users");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"name\":\"admin\""));
	TEST_ASSERT_TRUE(body_has("\"role\":\"operator\""));
	TEST_ASSERT_TRUE(body_has("\"persistent\":true"));
	TEST_ASSERT_TRUE(body_has("\"persistent\":false"));
	TEST_ASSERT_TRUE(body_has("\"kdf\":\"hmac-sha256\""));

	/* Password change stages the credential. */
	post_auth("/api/v1/security/password",
		  "{\"user\":\"admin\",\"password\":\"a-brand-new-one\"}");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"commit_required\":true"));
	TEST_ASSERT_TRUE(cfg_is_staged(&g_cfg, CFG_ID_SEC_ADMIN_PW));
	/* The plaintext must not appear in the response. */
	TEST_ASSERT_NULL(strstr(body_str(), "a-brand-new-one"));

	post_auth("/api/v1/security/password",
		  "{\"user\":\"nobody\",\"password\":\"whatever-long\"}");
	TEST_ASSERT_EQUAL_UINT(404U, g_resp.status);
	post_auth("/api/v1/security/password",
		  "{\"user\":\"admin\",\"password\":\"short\"}");
	TEST_ASSERT_EQUAL_UINT(422U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("password_policy"));
	post_auth("/api/v1/security/password", "{\"password\":\"longenough\"}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	post_auth("/api/v1/security/password", "{\"user\":\"admin\"}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	post_auth("/api/v1/security/password",
		  "{\"user\":\"admin\",\"password\":\"\\q\"}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);

	/* TLS info + install + CSR. */
	get_auth("/api/v1/security/tls");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"self_signed\":true"));
	TEST_ASSERT_TRUE(body_has("\"key_type\":\"EC\""));
	TEST_ASSERT_TRUE(body_has("\"enabled\":false"));
	fk.cert_info_rc = -EIO;
	get_auth("/api/v1/security/tls");
	TEST_ASSERT_EQUAL_UINT(503U, g_resp.status);
	fk.cert_info_rc = 0;

	post_auth("/api/v1/security/tls", "-----BEGIN CERTIFICATE-----");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_EQUAL_UINT(27U, fk.cert_len);
	/* Installing a new key closes every session. */
	TEST_ASSERT_EQUAL_UINT32(0U, auth_web_session_count(&g_auth));

	setup_rest((uint8_t)WEB_ROLE_ADMIN);
	post_auth("/api/v1/security/tls", NULL);
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	fk.cert_install_rc = -EBADMSG;
	post_auth("/api/v1/security/tls", "junk");
	TEST_ASSERT_EQUAL_UINT(422U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("bad_pem"));
	fk.cert_install_rc = -EPERM;
	post_auth("/api/v1/security/tls", "junk");
	TEST_ASSERT_EQUAL_UINT(422U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("key_mismatch"));
	fk.cert_install_rc = -EIO;
	post_auth("/api/v1/security/tls", "junk");
	TEST_ASSERT_EQUAL_UINT(500U, g_resp.status);
	fk.cert_install_rc = 0;

	post_auth("/api/v1/security/csr", NULL);
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("BEGIN CERTIFICATE REQUEST"));
	TEST_ASSERT_TRUE(strstr(g_resp.extra, "meridian.csr") != NULL);
	post_auth("/api/v1/security/csr", "{\"subject\":\"CN=box\"}");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	post_auth("/api/v1/security/csr", "{\"subject\":\"\\q\"}");
	TEST_ASSERT_EQUAL_UINT(400U, g_resp.status);
	fk.csr_rc = -ENOSPC;
	post_auth("/api/v1/security/csr", NULL);
	TEST_ASSERT_EQUAL_UINT(500U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("csr_too_large"));
	fk.csr_rc = -EIO;
	post_auth("/api/v1/security/csr", NULL);
	TEST_ASSERT_EQUAL_UINT(500U, g_resp.status);
	fk.csr_rc = 0;
}

/* ---- logs, metrics, telemetry ------------------------------------------- */

static void test_logs_route(void)
{
	unsigned int i;

	setup_rest((uint8_t)WEB_ROLE_VIEWER);
	for (i = 0U; i < 10U; i++) {
		TEST_ASSERT_TRUE(logr_puts(&g_log, (uint8_t)LOGR_INFO,
					   (uint8_t)LOGR_SUB_NET, 100U + i,
					   "hello \"world\"") <= 1);
	}

	get_auth("/api/v1/logs");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"type\":\"logs\""));
	TEST_ASSERT_TRUE(body_has("hello \\\"world\\\""));
	TEST_ASSERT_TRUE(body_has("\"subsystem\":\"NET\""));
	TEST_ASSERT_TRUE(body_has("\"level_name\":\"info\""));

	get_auth("/api/v1/logs?cursor=5&max=2&level=6");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(body_has("\"cursor\":5"));
	get_auth("/api/v1/logs?max=0&level=99");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	get_auth("/api/v1/logs?max=999");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);

	/* A page that cannot fit is a 500 that tells the client what to do. */
	{
		char tiny[128];
		char raw[256];
		size_t rn = (size_t)snprintf(raw, sizeof(raw),
					     "GET /api/v1/logs HTTP/1.1\r\n"
					     "Host: h\r\n"
					     "Cookie: sts_session=%s\r\n\r\n",
					     g_token);

		TEST_ASSERT_EQUAL_INT(0, http_parse_request(raw, rn, &g_req));
		rest_resp_init(&g_resp, tiny, sizeof(tiny));
		TEST_ASSERT_EQUAL_INT(0, rest_dispatch(&g_rest, &g_req, NULL, 0U,
						       &g_resp));
		TEST_ASSERT_EQUAL_UINT(500U, g_resp.status);
		TEST_ASSERT_TRUE(strstr(tiny, "encode_overflow") != NULL);
	}

	/* Direct API argument checks. */
	{
		uint32_t cur = 0U;
		char out[512];

		TEST_ASSERT_EQUAL_INT(-EINVAL, rest_encode_logs(NULL, &cur, 4U,
								7U, out,
								sizeof(out)));
		TEST_ASSERT_EQUAL_INT(-EINVAL, rest_encode_logs(&g_rest, NULL,
								4U, 7U, out,
								sizeof(out)));
		TEST_ASSERT_EQUAL_INT(-EINVAL, rest_encode_logs(&g_rest, &cur,
								4U, 7U, NULL,
								sizeof(out)));
		TEST_ASSERT_EQUAL_INT(-EINVAL, rest_encode_logs(&g_rest, &cur,
								4U, 7U, out, 0U));
		TEST_ASSERT_TRUE(rest_encode_logs(&g_rest, &cur, 4U, 7U, out,
						  sizeof(out)) > 0);
		TEST_ASSERT_EQUAL_UINT32(4U, cur);
	}
}

static void test_metrics_route(void)
{
	setup_rest((uint8_t)WEB_ROLE_VIEWER);

	get_auth("/api/v1/metrics");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_EQUAL_STRING("text/plain; version=0.0.4; charset=utf-8",
				 g_resp.content_type);
	TEST_ASSERT_TRUE(body_has("sts_stratum 1\n"));
	TEST_ASSERT_TRUE(body_has("sts_pps_offset_ns -3\n"));
	TEST_ASSERT_TRUE(body_has("sts_rail_bus_mv{rail=\"3V3_STM\"} 3300\n"));
	TEST_ASSERT_TRUE(body_has("sts_rail_valid{rail=\"3V3_STM\"} 0\n"));
	TEST_ASSERT_TRUE(body_has("sts_ntp_served_total 999\n"));
	TEST_ASSERT_TRUE(body_has("sts_web_sessions 1\n"));

	/* A rail with no name still gets a label. */
	{
		char out[4096];

		TEST_ASSERT_TRUE(rest_encode_metrics(&g_rest, out,
						     sizeof(out)) > 0);
		TEST_ASSERT_EQUAL_INT(-EINVAL, rest_encode_metrics(NULL, out,
								   sizeof(out)));
		TEST_ASSERT_EQUAL_INT(-EINVAL, rest_encode_metrics(&g_rest, NULL,
								   sizeof(out)));
		TEST_ASSERT_EQUAL_INT(-EINVAL, rest_encode_metrics(&g_rest, out,
								   0U));
		TEST_ASSERT_EQUAL_INT(-ENOSPC, rest_encode_metrics(&g_rest, out,
								   32U));
	}

	/* Metrics degrade rather than fail when providers are absent. */
	memset(&g_pv, 0, sizeof(g_pv));
	g_rest.auth = NULL;
	get_auth("/api/v1/metrics");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	{
		char out[1024];

		TEST_ASSERT_TRUE(rest_encode_metrics(&g_rest, out,
						     sizeof(out)) > 0);
		TEST_ASSERT_NULL(strstr(out, "sts_stratum"));
		TEST_ASSERT_NOT_NULL(strstr(out, "sts_web_requests_total"));
	}

	/* A tiny buffer on the route surfaces as a 500. */
	setup_rest((uint8_t)WEB_ROLE_VIEWER);
	{
		char tiny[16];
		char raw[256];
		size_t rn = (size_t)snprintf(raw, sizeof(raw),
					     "GET /api/v1/metrics HTTP/1.1\r\n"
					     "Host: h\r\n"
					     "Cookie: sts_session=%s\r\n\r\n",
					     g_token);

		TEST_ASSERT_EQUAL_INT(0, http_parse_request(raw, rn, &g_req));
		rest_resp_init(&g_resp, tiny, sizeof(tiny));
		TEST_ASSERT_EQUAL_INT(0, rest_dispatch(&g_rest, &g_req, NULL, 0U,
						       &g_resp));
		TEST_ASSERT_EQUAL_UINT(500U, g_resp.status);
	}
}

static void test_telemetry_encoder(void)
{
	char out[8192];
	int n;

	setup_rest((uint8_t)WEB_ROLE_VIEWER);

	n = rest_encode_telemetry(&g_rest, WSS_GRP_ALL, 11U, out, sizeof(out));
	TEST_ASSERT_TRUE(n > 0);
	out[n] = '\0';
	TEST_ASSERT_NOT_NULL(strstr(out, "\"type\":\"telemetry\""));
	TEST_ASSERT_NOT_NULL(strstr(out, "\"seq\":11"));
	TEST_ASSERT_NOT_NULL(strstr(out, "\"summary\":{"));
	TEST_ASSERT_NOT_NULL(strstr(out, "\"alarms\":{"));

	n = rest_encode_telemetry(&g_rest, WSS_GRP_TIMING, 1U, out,
				  sizeof(out));
	TEST_ASSERT_TRUE(n > 0);
	out[n] = '\0';
	TEST_ASSERT_NOT_NULL(strstr(out, "\"timing\":{"));
	TEST_ASSERT_NULL(strstr(out, "\"power\":"));

	/* A group with no provider becomes null, so "absent" is not "zero". */
	memset(&g_pv, 0, sizeof(g_pv));
	n = rest_encode_telemetry(&g_rest, WSS_GRP_TIMING, 1U, out,
				  sizeof(out));
	TEST_ASSERT_TRUE(n > 0);
	out[n] = '\0';
	TEST_ASSERT_NOT_NULL(strstr(out, "\"timing\":null"));

	TEST_ASSERT_EQUAL_INT(-EINVAL, rest_encode_telemetry(NULL, 1U, 0U, out,
							     sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rest_encode_telemetry(&g_rest, 1U, 0U,
							     NULL,
							     sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rest_encode_telemetry(&g_rest, 1U, 0U,
							     out, 0U));
	providers_all();
	TEST_ASSERT_EQUAL_INT(-ENOSPC, rest_encode_telemetry(&g_rest,
							     WSS_GRP_ALL, 0U,
							     out, 40U));

	/* rest_encode_group() argument and range checks. */
	{
		web_jw_t w;

		web_jw_init(&w, out, sizeof(out));
		TEST_ASSERT_EQUAL_INT(-EINVAL, rest_encode_group(NULL, 1U, &w));
		TEST_ASSERT_EQUAL_INT(-EINVAL, rest_encode_group(&g_rest, 1U,
								 NULL));
		TEST_ASSERT_EQUAL_INT(-EINVAL, rest_encode_group(&g_rest,
								 WSS_GRP_LOGS,
								 &w));
		TEST_ASSERT_EQUAL_INT(-EINVAL, rest_encode_group(&g_rest, 0U,
								 &w));
	}
}

/* ---- audit -------------------------------------------------------------- */

static void test_audit_records(void)
{
	logr_rec_t recs[16];
	uint16_t got = 0U;
	uint32_t next = 0U;
	uint32_t gap = 0U;
	uint32_t before;
	unsigned int i;
	bool found = false;

	setup_rest((uint8_t)WEB_ROLE_OPERATOR);
	before = logr_head(&g_log);

	post_auth("/api/v1/gnss/survey", "{\"action\":\"start\"}");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
	TEST_ASSERT_TRUE(g_rest.st.audit_records >= 1U);

	TEST_ASSERT_EQUAL_INT(0, logr_tail(&g_log, before, NULL, recs, 16U, &got,
					   &next, &gap));
	TEST_ASSERT_TRUE(got >= 1U);
	for (i = 0U; i < got; i++) {
		if (recs[i].subsys == (uint8_t)LOGR_SUB_SEC &&
		    strstr(recs[i].msg, "survey-start") != NULL &&
		    strstr(recs[i].msg, "operator") != NULL &&
		    strstr(recs[i].msg, "POST") != NULL) {
			found = true;
		}
	}
	TEST_ASSERT_TRUE(found);

	/* A rejected action is audited as an error, not silently. */
	before = logr_head(&g_log);
	fk.survey_rc = -EBUSY;
	post_auth("/api/v1/gnss/survey", "{\"action\":\"start\"}");
	TEST_ASSERT_EQUAL_UINT(409U, g_resp.status);
	TEST_ASSERT_EQUAL_INT(0, logr_tail(&g_log, before, NULL, recs, 16U, &got,
					   &next, &gap));
	found = false;
	for (i = 0U; i < got; i++) {
		if (strstr(recs[i].msg, "=err") != NULL) {
			found = true;
		}
	}
	TEST_ASSERT_TRUE(found);
	fk.survey_rc = 0;

	/* An audit record survives a very long path without overflowing. */
	{
		char raw[512];
		size_t rn;

		rn = (size_t)snprintf(raw, sizeof(raw),
				      "POST /api/v1/services/"
				      "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
				      "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
				      " HTTP/1.1\r\nHost: h\r\n"
				      "Cookie: sts_session=%s\r\n"
				      "X-CSRF-Token: %s\r\n"
				      "Content-Length: 0\r\n\r\n",
				      g_token, g_csrf);
		TEST_ASSERT_EQUAL_INT(0, http_parse_request(raw, rn, &g_req));
		rest_resp_init(&g_resp, g_respbuf, sizeof(g_respbuf));
		TEST_ASSERT_EQUAL_INT(0, rest_dispatch(&g_rest, &g_req, NULL, 0U,
						       &g_resp));
		TEST_ASSERT_EQUAL_UINT(404U, g_resp.status);
	}

	/* With no log ring, auditing is a no-op rather than a crash. */
	g_rest.log = NULL;
	post_auth("/api/v1/gnss/survey", "{\"action\":\"stop\"}");
	TEST_ASSERT_EQUAL_UINT(200U, g_resp.status);
}

/* ---- fuzz --------------------------------------------------------------- */

/*
 * Fuzz the router itself: random targets and random JSON bodies against a real
 * session. Nothing here should ever return anything but a well-formed response.
 */
static void test_fuzz_dispatch(void)
{
	test_rng_t g;
	static char raw[1024];
	unsigned int iter;

	setup_rest((uint8_t)WEB_ROLE_ADMIN);
	test_rng_init(&g, 0x1010U);

	for (iter = 0U; iter < 20000U; iter++) {
		static const char *const paths[] = {
			"/api/v1/status", "/api/v1/status/timing",
			"/api/v1/config", "/api/v1/config/tim.tau.s",
			"/api/v1/config/commit", "/api/v1/gnss/survey",
			"/api/v1/gnss/position", "/api/v1/timing/reference",
			"/api/v1/services/ntp", "/api/v1/firmware/begin",
			"/api/v1/security/password", "/api/v1/reboot",
			"/api/v1/factory-reset", "/api/v1/logs",
			"/api/v1/metrics", "/api/v1/auth/login",
		};
		static const char *const methods[] = { "GET", "POST", "PUT",
						       "HEAD", "OPTIONS" };
		char bodybuf[128];
		size_t bn;
		size_t o;
		size_t i;

		bn = test_rng_below(&g, sizeof(bodybuf));
		for (i = 0U; i < bn; i++) {
			static const char alpha[] =
				"{}[]\":,.0123456789-truefalsnl abcdefgimopstwxyz";

			bodybuf[i] = alpha[test_rng_below(&g, sizeof(alpha) - 1U)];
		}

		o = (size_t)snprintf(raw, sizeof(raw),
				     "%s %s HTTP/1.1\r\nHost: h\r\n"
				     "Cookie: sts_session=%s\r\n"
				     "X-CSRF-Token: %s\r\n"
				     "Content-Length: %u\r\n\r\n",
				     methods[test_rng_below(&g,
							    sizeof(methods) /
								    sizeof(methods[0]))],
				     paths[test_rng_below(&g,
							  sizeof(paths) /
								  sizeof(paths[0]))],
				     g_token, g_csrf, (unsigned int)bn);
		memcpy(&raw[o], bodybuf, bn);
		o += bn;

		if (http_parse_request(raw, o, &g_req) != 0) {
			continue;
		}
		rest_resp_init(&g_resp, g_respbuf, sizeof(g_respbuf));
		TEST_ASSERT_EQUAL_INT(0, rest_dispatch(&g_rest, &g_req,
						       (bn != 0U) ? &raw[o - bn]
								  : NULL,
						       bn, &g_resp));
		TEST_ASSERT_TRUE(g_resp.status >= 200U && g_resp.status < 600U);
		TEST_ASSERT_TRUE(g_resp.body_len < RESP_CAP);
		TEST_ASSERT_TRUE(g_resp.extra_len <= REST_EXTRA_MAX);
	}
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	UNITY_BEGIN();

	/* web.c */
	RUN_TEST(test_jw_basics);
	RUN_TEST(test_jw_escaping);
	RUN_TEST(test_jw_numbers);
	RUN_TEST(test_jw_errors);
	RUN_TEST(test_json_read);
	RUN_TEST(test_json_read_errors);
	RUN_TEST(test_codecs);
	RUN_TEST(test_ct_and_spans);
	RUN_TEST(test_methods_and_roles);
	RUN_TEST(test_status_text);

	/* rest.c */
	RUN_TEST(test_routing_basics);
	RUN_TEST(test_status_groups);
	RUN_TEST(test_gnss_satellites);
	RUN_TEST(test_missing_providers);
	RUN_TEST(test_unauthenticated_is_401);
	RUN_TEST(test_role_enforcement);
	RUN_TEST(test_csrf_enforcement);
	RUN_TEST(test_login_logout_routes);
	RUN_TEST(test_login_throttle_route);
	RUN_TEST(test_secret_never_served);
	RUN_TEST(test_config_read);
	RUN_TEST(test_config_write);
	RUN_TEST(test_config_commit_failure);
	RUN_TEST(test_config_import_export);
	RUN_TEST(test_control_routes);
	RUN_TEST(test_system_routes);
	RUN_TEST(test_firmware_routes);
	RUN_TEST(test_security_routes);
	RUN_TEST(test_logs_route);
	RUN_TEST(test_metrics_route);
	RUN_TEST(test_telemetry_encoder);
	RUN_TEST(test_audit_records);
	RUN_TEST(test_fuzz_dispatch);

	return UNITY_END();
}
