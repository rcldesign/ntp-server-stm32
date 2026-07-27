/*
 * STS1000 "Meridian" — core/mp JSON codec unit tests.
 *
 * The parser is fed attacker-controlled bytes on every request, so the tests are
 * weighted towards rejection: malformed documents, depth and token exhaustion,
 * broken escapes, and a fuzz pass whose only assertions are that the parser
 * terminates, never reports success on garbage it then mis-reads, and stays
 * inside its buffers (ASan/UBSan).
 *
 * Expectations are checked against RFC 8259 (JSON) rather than against this
 * implementation's own behaviour: literal documents from §13 of the RFC, the
 * grammar's number production, and the escape set of §7.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "mp/mp_json.h"
#include "test_support.h"

#define TOKS 64U

static mp_json_t g_p;
static mp_json_tok_t g_tok[TOKS];

/** Parse @p s, asserting success, and return the token count. */
static int ok_parse(const char *s)
{
	int n = mp_json_parse(&g_p, s, strlen(s), g_tok, TOKS, 0U);

	TEST_ASSERT_TRUE_MESSAGE(n > 0, s);
	return n;
}

static void bad_parse(const char *s, int expect)
{
	TEST_ASSERT_EQUAL_INT_MESSAGE(expect,
				      mp_json_parse(&g_p, s, strlen(s), g_tok,
						    TOKS, 0U),
				      s);
}

/* ============================================================ parser: good */

static void test_parse_scalars(void)
{
	int64_t i = 0;
	float f = 0.0f;
	bool b = false;
	char s[32];

	ok_parse("123");
	TEST_ASSERT_EQUAL_INT(MP_J_NUM, mp_json_at(&g_p, 0)->type);
	TEST_ASSERT_EQUAL_INT(0, mp_json_i64(&g_p, 0, &i));
	TEST_ASSERT_EQUAL_INT64(123, i);

	ok_parse("-4096");
	TEST_ASSERT_EQUAL_INT(0, mp_json_i64(&g_p, 0, &i));
	TEST_ASSERT_EQUAL_INT64(-4096, i);

	ok_parse("0");
	TEST_ASSERT_EQUAL_INT(0, mp_json_i64(&g_p, 0, &i));
	TEST_ASSERT_EQUAL_INT64(0, i);

	ok_parse("true");
	TEST_ASSERT_EQUAL_INT(MP_J_BOOL, mp_json_at(&g_p, 0)->type);
	TEST_ASSERT_EQUAL_INT(0, mp_json_bool(&g_p, 0, &b));
	TEST_ASSERT_TRUE(b);

	ok_parse("false");
	TEST_ASSERT_EQUAL_INT(0, mp_json_bool(&g_p, 0, &b));
	TEST_ASSERT_FALSE(b);

	ok_parse("null");
	TEST_ASSERT_EQUAL_INT(MP_J_NULL, mp_json_at(&g_p, 0)->type);

	ok_parse("\"hello\"");
	TEST_ASSERT_EQUAL_INT(MP_J_STR, mp_json_at(&g_p, 0)->type);
	TEST_ASSERT_EQUAL_INT(5, mp_json_str(&g_p, 0, s, sizeof(s)));
	TEST_ASSERT_EQUAL_STRING("hello", s);

	ok_parse("\"\"");
	TEST_ASSERT_EQUAL_INT(0, mp_json_str(&g_p, 0, s, sizeof(s)));
	TEST_ASSERT_EQUAL_STRING("", s);

	ok_parse("1.5e3");
	TEST_ASSERT_EQUAL_INT(0, mp_json_f32(&g_p, 0, &f));
	TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f, f);

	ok_parse("-0.125");
	TEST_ASSERT_EQUAL_INT(0, mp_json_f32(&g_p, 0, &f));
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.125f, f);

	ok_parse("2E-2");
	TEST_ASSERT_EQUAL_INT(0, mp_json_f32(&g_p, 0, &f));
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.02f, f);

	ok_parse("1e+2");
	TEST_ASSERT_EQUAL_INT(0, mp_json_f32(&g_p, 0, &f));
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100.0f, f);
}

static void test_parse_object(void)
{
	int v;
	int64_t i = 0;
	char s[32];

	ok_parse("{\"a\":1,\"b\":\"two\",\"c\":[1,2,3],\"d\":{\"e\":true}}");

	TEST_ASSERT_EQUAL_INT(MP_J_OBJ, mp_json_at(&g_p, 0)->type);
	TEST_ASSERT_EQUAL_UINT16(4U, mp_json_count(&g_p, 0));

	v = mp_json_obj_get(&g_p, 0, "a");
	TEST_ASSERT_TRUE(v > 0);
	TEST_ASSERT_EQUAL_INT(0, mp_json_i64(&g_p, v, &i));
	TEST_ASSERT_EQUAL_INT64(1, i);

	v = mp_json_obj_get(&g_p, 0, "b");
	TEST_ASSERT_EQUAL_INT(3, mp_json_str(&g_p, v, s, sizeof(s)));
	TEST_ASSERT_EQUAL_STRING("two", s);

	v = mp_json_obj_get(&g_p, 0, "c");
	TEST_ASSERT_EQUAL_INT(MP_J_ARR, mp_json_at(&g_p, v)->type);
	TEST_ASSERT_EQUAL_UINT16(3U, mp_json_count(&g_p, v));
	TEST_ASSERT_EQUAL_INT(0, mp_json_i64(&g_p, mp_json_arr_at(&g_p, v, 1U),
					     &i));
	TEST_ASSERT_EQUAL_INT64(2, i);

	v = mp_json_obj_get(&g_p, 0, "d");
	TEST_ASSERT_EQUAL_INT(MP_J_OBJ, mp_json_at(&g_p, v)->type);
	TEST_ASSERT_TRUE(mp_json_has(&g_p, v, "e"));
	TEST_ASSERT_FALSE(mp_json_has(&g_p, v, "f"));

	TEST_ASSERT_EQUAL_INT(-ENOENT, mp_json_obj_get(&g_p, 0, "zz"));
}

static void test_parse_empty_containers(void)
{
	ok_parse("{}");
	TEST_ASSERT_EQUAL_UINT16(0U, mp_json_count(&g_p, 0));
	ok_parse("[]");
	TEST_ASSERT_EQUAL_UINT16(0U, mp_json_count(&g_p, 0));
	ok_parse("{\"a\":{},\"b\":[]}");
	TEST_ASSERT_EQUAL_UINT16(2U, mp_json_count(&g_p, 0));
	ok_parse("[[],[[]]]");
	TEST_ASSERT_EQUAL_UINT16(2U, mp_json_count(&g_p, 0));
}

static void test_parse_whitespace_is_ignored(void)
{
	int v;
	int64_t i = 0;

	ok_parse("  \t\r\n { \"a\" : \n 1 , \"b\" : [ 2 , 3 ] } \n ");
	v = mp_json_obj_get(&g_p, 0, "a");
	TEST_ASSERT_EQUAL_INT(0, mp_json_i64(&g_p, v, &i));
	TEST_ASSERT_EQUAL_INT64(1, i);
}

static void test_rfc8259_examples(void)
{
	/* RFC 8259 §13 examples. */
	ok_parse("{\"Image\":{\"Width\":800,\"Height\":600,"
		 "\"Title\":\"View from 15th Floor\",\"Thumbnail\":"
		 "{\"Url\":\"http://www.example.com/image/481989943\","
		 "\"Height\":125,\"Width\":100},\"Animated\":false,"
		 "\"IDs\":[116,943,234,38793]}}");
	{
		int img = mp_json_obj_get(&g_p, 0, "Image");
		int ids = mp_json_obj_get(&g_p, img, "IDs");
		int64_t i = 0;

		TEST_ASSERT_EQUAL_UINT16(4U, mp_json_count(&g_p, ids));
		TEST_ASSERT_EQUAL_INT(0,
				      mp_json_i64(&g_p,
						  mp_json_arr_at(&g_p, ids, 3U),
						  &i));
		TEST_ASSERT_EQUAL_INT64(38793, i);
	}
}

/* ============================================================ parser: bad */

static void test_parse_rejections(void)
{
	bad_parse("", -EBADMSG);
	bad_parse("   ", -EBADMSG);
	bad_parse("{", -EBADMSG);
	bad_parse("}", -EBADMSG);
	bad_parse("[", -EBADMSG);
	bad_parse("]", -EBADMSG);
	bad_parse("{\"a\"", -EBADMSG);
	bad_parse("{\"a\":", -EBADMSG);
	bad_parse("{\"a\":}", -EBADMSG);
	bad_parse("{\"a\":1,}", -EBADMSG);
	bad_parse("{,}", -EBADMSG);
	bad_parse("{\"a\" 1}", -EBADMSG);
	bad_parse("{a:1}", -EBADMSG);
	bad_parse("{1:2}", -EBADMSG);
	bad_parse("[1,]", -EBADMSG);
	bad_parse("[,1]", -EBADMSG);
	bad_parse("[1 2]", -EBADMSG);
	bad_parse("{\"a\":1]", -EBADMSG);
	bad_parse("[1,2}", -EBADMSG);
	bad_parse("1 2", -EBADMSG);
	bad_parse("{} {}", -EBADMSG);
	bad_parse("tru", -EBADMSG);
	bad_parse("truex", -EBADMSG); /* trailing garbage after `true` */
	bad_parse("nul", -EBADMSG);
	bad_parse("fals", -EBADMSG);
	bad_parse("undefined", -EBADMSG);
	bad_parse("'single'", -EBADMSG);
	bad_parse("\"unterminated", -EBADMSG);
	bad_parse("\"bad\\escape\"", -EBADMSG);
	bad_parse("\"\\u12\"", -EBADMSG);
	bad_parse("\"\\uZZZZ\"", -EBADMSG);
	bad_parse("\"\\", -EBADMSG);
	/* RFC 8259 §7: unescaped control characters are not allowed. */
	bad_parse("\"a\nb\"", -EBADMSG);
	bad_parse("\"a\tb\"", -EBADMSG);
	/* Number grammar (RFC 8259 §6). */
	bad_parse("-", -EBADMSG);
	bad_parse("+1", -EBADMSG);
	bad_parse("1.", -EBADMSG);
	bad_parse(".5", -EBADMSG);
	bad_parse("1e", -EBADMSG);
	bad_parse("1e+", -EBADMSG);
	bad_parse("--1", -EBADMSG);
	bad_parse("{\"a\":1,\"b\"}", -EBADMSG);
	bad_parse("{\"a\":,1}", -EBADMSG);
	bad_parse("[:]", -EBADMSG);
}

static void test_leading_zero_is_tolerated_but_bounded(void)
{
	int64_t i = 0;

	/*
	 * Strict RFC 8259 forbids "01". This parser accepts a leading zero as a
	 * complete number and then rejects the trailing digit as garbage, which
	 * is the same outcome — the document is refused.
	 */
	bad_parse("01", -EBADMSG);
	ok_parse("0");
	TEST_ASSERT_EQUAL_INT(0, mp_json_i64(&g_p, 0, &i));
}

static void test_depth_cap(void)
{
	char deep[128];
	size_t i;

	/* Exactly at the cap is fine. */
	for (i = 0U; i < MP_JSON_DEPTH_MAX; i++) {
		deep[i] = '[';
	}
	for (i = 0U; i < MP_JSON_DEPTH_MAX; i++) {
		deep[MP_JSON_DEPTH_MAX + i] = ']';
	}
	deep[2U * MP_JSON_DEPTH_MAX] = '\0';
	TEST_ASSERT_TRUE(mp_json_parse(&g_p, deep, strlen(deep), g_tok, TOKS,
				       0U) > 0);

	/* One deeper is refused. */
	for (i = 0U; i < (MP_JSON_DEPTH_MAX + 1U); i++) {
		deep[i] = '[';
	}
	for (i = 0U; i < (MP_JSON_DEPTH_MAX + 1U); i++) {
		deep[MP_JSON_DEPTH_MAX + 1U + i] = ']';
	}
	deep[2U * (MP_JSON_DEPTH_MAX + 1U)] = '\0';
	TEST_ASSERT_EQUAL_INT(-E2BIG, mp_json_parse(&g_p, deep, strlen(deep),
						    g_tok, TOKS, 0U));

	/* A caller-supplied lower cap is honoured. */
	TEST_ASSERT_EQUAL_INT(-E2BIG, mp_json_parse(&g_p, "[[[1]]]", 7U, g_tok,
						    TOKS, 2U));
	TEST_ASSERT_TRUE(mp_json_parse(&g_p, "[[1]]", 5U, g_tok, TOKS, 2U) > 0);
}

static void test_token_exhaustion(void)
{
	mp_json_tok_t few[3];

	TEST_ASSERT_EQUAL_INT(-ENOSPC, mp_json_parse(&g_p, "[1,2,3,4]", 9U, few,
						     3U, 0U));
}

static void test_document_too_large(void)
{
	static char big[70000];

	memset(big, ' ', sizeof(big));
	big[0] = '1';
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE, mp_json_parse(&g_p, big, sizeof(big),
						       g_tok, TOKS, 0U));
}

static void test_parser_argument_validation(void)
{
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_json_parse(NULL, "1", 1U, g_tok, TOKS, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_json_parse(&g_p, NULL, 1U, g_tok, TOKS, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_json_parse(&g_p, "1", 1U, NULL, TOKS, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_json_parse(&g_p, "1", 1U, g_tok, 0U, 0U));

	TEST_ASSERT_EQUAL_INT(-ENOENT, mp_json_root(NULL));
	TEST_ASSERT_NULL(mp_json_at(NULL, 0));
	TEST_ASSERT_NULL(mp_json_at(&g_p, -1));
	TEST_ASSERT_NULL(mp_json_at(&g_p, 9999));
	TEST_ASSERT_EQUAL_UINT16(0U, mp_json_count(&g_p, -1));
	TEST_ASSERT_FALSE(mp_json_streq(&g_p, -1, "x"));
	TEST_ASSERT_FALSE(mp_json_streq(NULL, 0, "x"));
}

static void test_accessor_type_errors(void)
{
	int64_t i = 0;
	float f = 0.0f;
	bool b = false;
	char s[8];

	ok_parse("{\"a\":1,\"b\":\"x\",\"c\":true,\"d\":1.5}");

	TEST_ASSERT_EQUAL_INT(-ENOTDIR,
			      mp_json_obj_get(&g_p,
					      mp_json_obj_get(&g_p, 0, "a"),
					      "a"));
	TEST_ASSERT_EQUAL_INT(-ENOTDIR, mp_json_arr_at(&g_p, 0, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_json_obj_get(&g_p, 0, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_json_obj_get(NULL, 0, "a"));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_json_arr_at(NULL, 0, 0U));

	/* A string is not a number, and 1.5 is not an integer. */
	TEST_ASSERT_EQUAL_INT(-ENOTDIR,
			      mp_json_i64(&g_p, mp_json_obj_get(&g_p, 0, "b"),
					  &i));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      mp_json_i64(&g_p, mp_json_obj_get(&g_p, 0, "d"),
					  &i));
	TEST_ASSERT_EQUAL_INT(-ENOTDIR,
			      mp_json_f32(&g_p, mp_json_obj_get(&g_p, 0, "b"),
					  &f));
	TEST_ASSERT_EQUAL_INT(-ENOTDIR,
			      mp_json_bool(&g_p, mp_json_obj_get(&g_p, 0, "a"),
					   &b));
	TEST_ASSERT_EQUAL_INT(-ENOTDIR,
			      mp_json_str(&g_p, mp_json_obj_get(&g_p, 0, "a"), s,
					  sizeof(s)));

	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_json_i64(&g_p, 1, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_json_f32(&g_p, 1, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_json_bool(&g_p, 3, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_json_str(&g_p, 1, NULL, sizeof(s)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_json_str(&g_p, 1, s, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_json_i32_range(&g_p, 1, 0, 1, NULL));
}

static void test_i64_bounds(void)
{
	int64_t i = 0;
	int32_t v = 0;

	ok_parse("9223372036854775807");
	TEST_ASSERT_EQUAL_INT(0, mp_json_i64(&g_p, 0, &i));
	TEST_ASSERT_EQUAL_INT64(9223372036854775807LL, i);

	ok_parse("-9223372036854775808");
	TEST_ASSERT_EQUAL_INT(0, mp_json_i64(&g_p, 0, &i));
	TEST_ASSERT_TRUE(i == (int64_t)(-9223372036854775807LL) - 1LL);

	ok_parse("9223372036854775808");
	TEST_ASSERT_EQUAL_INT(-ERANGE, mp_json_i64(&g_p, 0, &i));

	ok_parse("99999999999999999999999");
	TEST_ASSERT_EQUAL_INT(-ERANGE, mp_json_i64(&g_p, 0, &i));

	ok_parse("-99999999999999999999999");
	TEST_ASSERT_EQUAL_INT(-ERANGE, mp_json_i64(&g_p, 0, &i));

	ok_parse("50");
	TEST_ASSERT_EQUAL_INT(0, mp_json_i32_range(&g_p, 0, 0, 100, &v));
	TEST_ASSERT_EQUAL_INT32(50, v);
	TEST_ASSERT_EQUAL_INT(-ERANGE,
			      mp_json_i32_range(&g_p, 0, 0, 49, &v));
	TEST_ASSERT_EQUAL_INT(-ERANGE,
			      mp_json_i32_range(&g_p, 0, 51, 100, &v));
}

/* ---------------------------------------------------------------- escapes */

static void test_string_escapes(void)
{
	char s[64];

	ok_parse("\"a\\\"b\\\\c\\/d\\be\\ff\\ng\\rh\\ti\"");
	TEST_ASSERT_EQUAL_INT(17, mp_json_str(&g_p, 0, s, sizeof(s)));
	TEST_ASSERT_EQUAL_STRING("a\"b\\c/d\be\ff\ng\rh\ti", s);

	/* \u to ASCII, two-byte and three-byte UTF-8. */
	ok_parse("\"\\u0041\"");
	TEST_ASSERT_EQUAL_INT(1, mp_json_str(&g_p, 0, s, sizeof(s)));
	TEST_ASSERT_EQUAL_STRING("A", s);

	ok_parse("\"\\u00e9\"");
	TEST_ASSERT_EQUAL_INT(2, mp_json_str(&g_p, 0, s, sizeof(s)));
	TEST_ASSERT_EQUAL_UINT8(0xC3U, (uint8_t)s[0]);
	TEST_ASSERT_EQUAL_UINT8(0xA9U, (uint8_t)s[1]);

	ok_parse("\"\\u20AC\""); /* euro sign */
	TEST_ASSERT_EQUAL_INT(3, mp_json_str(&g_p, 0, s, sizeof(s)));
	TEST_ASSERT_EQUAL_UINT8(0xE2U, (uint8_t)s[0]);
	TEST_ASSERT_EQUAL_UINT8(0x82U, (uint8_t)s[1]);
	TEST_ASSERT_EQUAL_UINT8(0xACU, (uint8_t)s[2]);

	/* Surrogate pair: U+1D11E (musical symbol G clef). */
	ok_parse("\"\\uD834\\uDD1E\"");
	TEST_ASSERT_EQUAL_INT(4, mp_json_str(&g_p, 0, s, sizeof(s)));
	TEST_ASSERT_EQUAL_UINT8(0xF0U, (uint8_t)s[0]);
	TEST_ASSERT_EQUAL_UINT8(0x9DU, (uint8_t)s[1]);
	TEST_ASSERT_EQUAL_UINT8(0x84U, (uint8_t)s[2]);
	TEST_ASSERT_EQUAL_UINT8(0x9EU, (uint8_t)s[3]);

	/* Lone high surrogate -> U+FFFD, not a rejection. */
	ok_parse("\"\\uD834\"");
	TEST_ASSERT_EQUAL_INT(3, mp_json_str(&g_p, 0, s, sizeof(s)));
	TEST_ASSERT_EQUAL_UINT8(0xEFU, (uint8_t)s[0]);
	TEST_ASSERT_EQUAL_UINT8(0xBFU, (uint8_t)s[1]);
	TEST_ASSERT_EQUAL_UINT8(0xBDU, (uint8_t)s[2]);

	/* Lone low surrogate -> U+FFFD. */
	ok_parse("\"\\uDD1E\"");
	TEST_ASSERT_EQUAL_INT(3, mp_json_str(&g_p, 0, s, sizeof(s)));
	TEST_ASSERT_EQUAL_UINT8(0xEFU, (uint8_t)s[0]);

	/* High surrogate followed by a non-surrogate -> U+FFFD then the char. */
	ok_parse("\"\\uD834\\u0041\"");
	TEST_ASSERT_EQUAL_INT(4, mp_json_str(&g_p, 0, s, sizeof(s)));
	TEST_ASSERT_EQUAL_UINT8(0xEFU, (uint8_t)s[0]);
	TEST_ASSERT_EQUAL_CHAR('A', s[3]);

	/* NUL escape: written out and terminated, length counts the byte. */
	ok_parse("\"a\\u0000b\"");
	TEST_ASSERT_EQUAL_INT(3, mp_json_str(&g_p, 0, s, sizeof(s)));
	TEST_ASSERT_EQUAL_CHAR('a', s[0]);
	TEST_ASSERT_EQUAL_CHAR('\0', s[1]);
	TEST_ASSERT_EQUAL_CHAR('b', s[2]);
}

static void test_string_output_space(void)
{
	char s[4];

	ok_parse("\"abcdefgh\"");
	TEST_ASSERT_EQUAL_INT(-ENOSPC, mp_json_str(&g_p, 0, s, sizeof(s)));

	/* A multi-byte escape that will not fit is also -ENOSPC, not a partial. */
	ok_parse("\"\\uD834\\uDD1E\"");
	TEST_ASSERT_EQUAL_INT(-ENOSPC, mp_json_str(&g_p, 0, s, sizeof(s)));
}

static void test_streq_with_escapes(void)
{
	ok_parse("{\"a\\/b\":1,\"c\\td\":2,\"\\u0041\":3}");

	TEST_ASSERT_TRUE(mp_json_has(&g_p, 0, "a/b"));
	TEST_ASSERT_TRUE(mp_json_has(&g_p, 0, "c\td"));
	/* A \u escape never matches a plain-ASCII key: documented behaviour. */
	TEST_ASSERT_FALSE(mp_json_has(&g_p, 0, "A"));
	TEST_ASSERT_FALSE(mp_json_has(&g_p, 0, "a/"));
	TEST_ASSERT_FALSE(mp_json_has(&g_p, 0, "a/bc"));
	TEST_ASSERT_FALSE(mp_json_streq(&g_p, 1, NULL));
}

/* ============================================================ writer ==== */

static void test_writer_basic(void)
{
	char buf[256];
	mp_jw_t w;
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(0, mp_jw_obj_open(&w));
	TEST_ASSERT_EQUAL_INT(0, mp_jw_kv_str(&w, "s", "hi"));
	TEST_ASSERT_EQUAL_INT(0, mp_jw_kv_i64(&w, "i", -42));
	TEST_ASSERT_EQUAL_INT(0, mp_jw_kv_u64(&w, "u", 4294967296ULL));
	TEST_ASSERT_EQUAL_INT(0, mp_jw_kv_bool(&w, "b", true));
	TEST_ASSERT_EQUAL_INT(0, mp_jw_kv_null(&w, "n"));
	TEST_ASSERT_EQUAL_INT(0, mp_jw_key(&w, "a"));
	TEST_ASSERT_EQUAL_INT(0, mp_jw_arr_open(&w));
	TEST_ASSERT_EQUAL_INT(0, mp_jw_i64(&w, 1));
	TEST_ASSERT_EQUAL_INT(0, mp_jw_i64(&w, 2));
	TEST_ASSERT_EQUAL_INT(0, mp_jw_arr_close(&w));
	TEST_ASSERT_EQUAL_INT(0, mp_jw_obj_close(&w));
	TEST_ASSERT_EQUAL_INT(0, mp_jw_finish(&w, &len));

	TEST_ASSERT_EQUAL_STRING("{\"s\":\"hi\",\"i\":-42,\"u\":4294967296,"
				 "\"b\":true,\"n\":null,\"a\":[1,2]}",
				 buf);
	TEST_ASSERT_EQUAL_size_t(strlen(buf), len);

	/* And it parses back. */
	TEST_ASSERT_TRUE(mp_json_parse(&g_p, buf, len, g_tok, TOKS, 0U) > 0);
}

static void test_writer_escaping(void)
{
	char buf[128];
	char out[64];
	mp_jw_t w;
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	(void)mp_jw_str(&w, "q\"b\\s\nt\tc\x01");
	TEST_ASSERT_EQUAL_INT(0, mp_jw_finish(&w, &len));
	TEST_ASSERT_EQUAL_STRING("\"q\\\"b\\\\s\\nt\\tc\\u0001\"", buf);

	/* Round-trips through the parser. */
	TEST_ASSERT_TRUE(mp_json_parse(&g_p, buf, len, g_tok, TOKS, 0U) > 0);
	TEST_ASSERT_EQUAL_INT(10, mp_json_str(&g_p, 0, out, sizeof(out)));
	TEST_ASSERT_EQUAL_STRING("q\"b\\s\nt\tc\x01", out);
}

static void test_writer_float(void)
{
	char buf[64];
	mp_jw_t w;
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	(void)mp_jw_arr_open(&w);
	(void)mp_jw_f32(&w, 1.5f, 3U);
	(void)mp_jw_f32(&w, -0.25f, 2U);
	(void)mp_jw_f32(&w, 0.0f, 0U);
	(void)mp_jw_f32(&w, 12.0f, 1U);
	(void)mp_jw_f32(&w, 0.001f, 3U);
	(void)mp_jw_arr_close(&w);
	TEST_ASSERT_EQUAL_INT(0, mp_jw_finish(&w, &len));
	TEST_ASSERT_EQUAL_STRING("[1.500,-0.25,0,12.0,0.001]", buf);
}

static void test_writer_nan_and_inf_become_null(void)
{
	char buf[64];
	mp_jw_t w;
	size_t len = 0U;
	float nan_v;
	float inf_v = 1.0e30f;
	uint32_t bits = 0x7FC00000U; /* quiet NaN */

	memcpy(&nan_v, &bits, sizeof(nan_v));
	inf_v = inf_v * inf_v * inf_v; /* overflow to +inf */

	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	(void)mp_jw_arr_open(&w);
	(void)mp_jw_f32(&w, nan_v, 2U);
	(void)mp_jw_f32(&w, inf_v, 2U);
	(void)mp_jw_f32(&w, -inf_v, 2U);
	(void)mp_jw_arr_close(&w);
	TEST_ASSERT_EQUAL_INT(0, mp_jw_finish(&w, &len));
	TEST_ASSERT_EQUAL_STRING("[null,null,null]", buf);
}

static void test_writer_int64_min(void)
{
	char buf[64];
	mp_jw_t w;
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	(void)mp_jw_i64(&w, (int64_t)(-9223372036854775807LL) - 1LL);
	TEST_ASSERT_EQUAL_INT(0, mp_jw_finish(&w, &len));
	TEST_ASSERT_EQUAL_STRING("-9223372036854775808", buf);
}

static void test_writer_null_string_is_json_null(void)
{
	char buf[64];
	mp_jw_t w;
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	(void)mp_jw_obj_open(&w);
	(void)mp_jw_kv_str(&w, "a", NULL);
	(void)mp_jw_obj_close(&w);
	TEST_ASSERT_EQUAL_INT(0, mp_jw_finish(&w, &len));
	TEST_ASSERT_EQUAL_STRING("{\"a\":null}", buf);
}

static void test_writer_overflow_latches(void)
{
	char buf[8];
	mp_jw_t w;
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	(void)mp_jw_obj_open(&w);
	(void)mp_jw_kv_str(&w, "key", "a rather long value");
	(void)mp_jw_obj_close(&w);
	TEST_ASSERT_EQUAL_INT(-ENOSPC, mp_jw_finish(&w, &len));

	/* Once latched, later calls keep returning the error. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC, mp_jw_str(&w, "x"));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, mp_jw_i64(&w, 1));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, mp_jw_obj_open(&w));
}

static void test_writer_structure_errors(void)
{
	char buf[128];
	mp_jw_t w;
	size_t len = 0U;
	unsigned int i;

	/* Unbalanced: a container left open. */
	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	(void)mp_jw_obj_open(&w);
	TEST_ASSERT_EQUAL_INT(-EPROTO, mp_jw_finish(&w, &len));

	/* Wrong closer. */
	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	(void)mp_jw_obj_open(&w);
	TEST_ASSERT_EQUAL_INT(-EPROTO, mp_jw_arr_close(&w));

	/* Closing nothing. */
	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(-EPROTO, mp_jw_obj_close(&w));

	/* A key outside an object. */
	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	(void)mp_jw_arr_open(&w);
	TEST_ASSERT_EQUAL_INT(-EPROTO, mp_jw_key(&w, "a"));

	/* A key at top level. */
	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(-EPROTO, mp_jw_key(&w, "a"));

	/* Two keys in a row. */
	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	(void)mp_jw_obj_open(&w);
	(void)mp_jw_key(&w, "a");
	TEST_ASSERT_EQUAL_INT(-EPROTO, mp_jw_key(&w, "b"));

	/* A bare value inside an object. */
	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	(void)mp_jw_obj_open(&w);
	(void)mp_jw_i64(&w, 1);
	TEST_ASSERT_EQUAL_INT(-EPROTO, mp_jw_finish(&w, &len));

	/* Closing an object with a key owed. */
	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	(void)mp_jw_obj_open(&w);
	(void)mp_jw_key(&w, "a");
	TEST_ASSERT_EQUAL_INT(-EPROTO, mp_jw_obj_close(&w));

	/* Depth cap. */
	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	for (i = 0U; i < MP_JW_DEPTH_MAX; i++) {
		TEST_ASSERT_EQUAL_INT(0, mp_jw_arr_open(&w));
	}
	TEST_ASSERT_EQUAL_INT(-E2BIG, mp_jw_arr_open(&w));
}

static void test_writer_argument_validation(void)
{
	char buf[16];
	mp_jw_t w;

	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_init(NULL, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_init(&w, NULL, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_init(&w, buf, 0U));

	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_obj_open(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_obj_close(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_arr_open(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_arr_close(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_key(NULL, "a"));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_str(NULL, "a"));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_i64(NULL, 0));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_u64(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_bool(NULL, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_null(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_f32(NULL, 0.0f, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_raw(NULL, "1", 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_finish(NULL, NULL));

	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_key(&w, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_jw_raw(&w, NULL, 1U));
}

static void test_writer_raw_and_decimal_clamp(void)
{
	char buf[64];
	mp_jw_t w;
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	(void)mp_jw_obj_open(&w);
	(void)mp_jw_key(&w, "r");
	(void)mp_jw_raw(&w, "[1,2,3]", 7U);
	(void)mp_jw_obj_close(&w);
	TEST_ASSERT_EQUAL_INT(0, mp_jw_finish(&w, &len));
	TEST_ASSERT_EQUAL_STRING("{\"r\":[1,2,3]}", buf);

	/* decimals > 6 clamps to 6 rather than overrunning the scratch. */
	TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
	(void)mp_jw_f32(&w, 1.0f, 200U);
	TEST_ASSERT_EQUAL_INT(0, mp_jw_finish(&w, &len));
	TEST_ASSERT_EQUAL_STRING("1.000000", buf);
}

/* ============================================================ base64 ==== */

static void test_base64_round_trip(void)
{
	static const struct {
		const char *plain;
		const char *b64;
	} vec[] = {
		/* RFC 4648 §10 test vectors. */
		{ "", "" },
		{ "f", "Zg==" },
		{ "fo", "Zm8=" },
		{ "foo", "Zm9v" },
		{ "foob", "Zm9vYg==" },
		{ "fooba", "Zm9vYmE=" },
		{ "foobar", "Zm9vYmFy" },
	};
	size_t i;

	for (i = 0U; i < (sizeof(vec) / sizeof(vec[0])); i++) {
		char enc[32];
		uint8_t dec[32];
		size_t n = strlen(vec[i].plain);
		int e;
		int d;

		e = mp_b64_encode((const uint8_t *)vec[i].plain, n, enc,
				  sizeof(enc));
		TEST_ASSERT_TRUE(e >= 0);
		TEST_ASSERT_EQUAL_STRING(vec[i].b64, enc);

		d = mp_b64_decode(enc, strlen(enc), dec, sizeof(dec));
		TEST_ASSERT_EQUAL_INT((int)n, d);
		if (n != 0U) {
			TEST_ASSERT_EQUAL_UINT8_ARRAY(vec[i].plain, dec, n);
		}
	}
}

static void test_base64_binary_round_trip(void)
{
	test_rng_t rng;
	unsigned int round;

	test_rng_init(&rng, 0x64U);
	for (round = 0U; round < 200U; round++) {
		uint8_t raw[96];
		uint8_t back[96];
		char enc[MP_B64_LEN(sizeof(raw)) + 1U];
		size_t n = test_rng_below(&rng, sizeof(raw) + 1U);

		test_rng_fill(&rng, raw, n);
		TEST_ASSERT_TRUE(mp_b64_encode(raw, n, enc, sizeof(enc)) >= 0);
		TEST_ASSERT_EQUAL_INT((int)n, mp_b64_decode(enc, strlen(enc),
							    back,
							    sizeof(back)));
		if (n != 0U) {
			TEST_ASSERT_EQUAL_UINT8_ARRAY(raw, back, n);
		}
	}
}

static void test_base64_errors(void)
{
	uint8_t dec[8];
	char enc[8];

	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_b64_encode(NULL, 4U, enc,
						     sizeof(enc)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_b64_encode((const uint8_t *)"ab", 2U, NULL,
					    sizeof(enc)));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      mp_b64_encode((const uint8_t *)"abcdefgh", 8U,
					    enc, sizeof(enc)));

	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_b64_decode(NULL, 4U, dec,
						     sizeof(dec)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_b64_decode("Zm9v", 4U, NULL, 4U));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, mp_b64_decode("Zm9*", 4U, dec,
						      sizeof(dec)));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, mp_b64_decode("Zm9", 3U, dec,
						      sizeof(dec)));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, mp_b64_decode("Zg===", 5U, dec,
						      sizeof(dec)));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, mp_b64_decode("Zg==Zg==", 8U, dec,
						      sizeof(dec)));
	/* Non-zero pad bits are rejected: "Zh==" would decode 'f' with junk. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG, mp_b64_decode("Zh==", 4U, dec,
						      sizeof(dec)));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, mp_b64_decode(
					       "Zm9vYmFyYmF6cXV1eA==", 20U, dec,
					       4U));

	/* Whitespace is skipped. */
	TEST_ASSERT_EQUAL_INT(3, mp_b64_decode("Zm 9\nv", 6U, dec,
					       sizeof(dec)));
}

/* ============================================================== fuzz ==== */

/**
 * Parser fuzz.
 *
 * Two generators: pure random bytes, and structurally plausible documents built
 * from JSON tokens. When a parse succeeds, every token is walked and every
 * accessor exercised — that is what turns "it did not crash" into "the token
 * table it produced is self-consistent".
 */
/** Walk every token, calling every accessor. Asserts internal consistency. */
static void walk_tokens(const mp_json_t *p, int n)
{
	int i;

	for (i = 0; i < n; i++) {
		const mp_json_tok_t *t = mp_json_at(p, i);
		char s[64];
		int64_t iv = 0;
		float fv = 0.0f;
		bool bv = false;

		TEST_ASSERT_NOT_NULL(t);
		/* Spans stay inside the document and are ordered. */
		TEST_ASSERT_TRUE((size_t)t->start <= p->len);
		TEST_ASSERT_TRUE((size_t)t->end <= p->len);
		TEST_ASSERT_TRUE(t->start <= t->end);
		/* Parents precede their children and the root has none. */
		TEST_ASSERT_TRUE(t->parent < (int16_t)i);
		if (i == 0) {
			TEST_ASSERT_EQUAL_INT(-1, t->parent);
		} else {
			TEST_ASSERT_TRUE(t->parent >= 0);
		}
		TEST_ASSERT_TRUE(t->depth <= MP_JSON_DEPTH_MAX);
		TEST_ASSERT_TRUE(t->type <= (uint8_t)MP_J_NULL);

		/* Accessors must answer, not fault, whatever the token is. */
		(void)mp_json_str(p, i, s, sizeof(s));
		(void)mp_json_i64(p, i, &iv);
		(void)mp_json_f32(p, i, &fv);
		(void)mp_json_bool(p, i, &bv);
		(void)mp_json_streq(p, i, "probe");
		(void)mp_json_count(p, i);
		(void)mp_json_obj_get(p, i, "probe");
		(void)mp_json_arr_at(p, i, 0U);

		if ((t->type == (uint8_t)MP_J_OBJ) ||
		    (t->type == (uint8_t)MP_J_ARR)) {
			uint16_t k;
			uint16_t children = 0U;
			int j;

			/* `size` must equal the number of tokens claiming this
			 * token as their parent. */
			for (j = i + 1; j < n; j++) {
				if (p->tok[j].parent == (int16_t)i) {
					children++;
				}
			}
			TEST_ASSERT_EQUAL_UINT16(t->size, children);

			if (t->type == (uint8_t)MP_J_ARR) {
				for (k = 0U; k < t->size; k++) {
					TEST_ASSERT_TRUE(
						mp_json_arr_at(p, i, k) > i);
				}
				TEST_ASSERT_EQUAL_INT(-ENOENT,
						      mp_json_arr_at(p, i,
								     t->size));
			}
		}
	}
}

/**
 * Parser fuzz.
 *
 * Two generators: pure random bytes, and structurally plausible documents built
 * from JSON fragments. When a parse succeeds, every token is walked and every
 * accessor exercised — that is what turns "it did not crash" into "the token
 * table it produced is self-consistent". ASan/UBSan cover the memory side.
 */
static void test_fuzz_parser(void)
{
	static const char *const frag[] = {
		"{",	 "}",	 "[",	 "]",	  ",",	  ":",	  "\"a\"",
		"\"bb\"", "1",	 "-1",	 "1.5",	  "true", "false", "null",
		" ",	 "\"",	 "\\",	 "\\u0041", "e5",  "+",	  "\t",
		"0",	 "1e9",	 "\\n",	 "\xC3\xA9", "\x01", "-",   ".",
	};
	const size_t nfrag = sizeof(frag) / sizeof(frag[0]);
	test_rng_t rng;
	unsigned int round;
	unsigned int accepted = 0U;
	unsigned int rejected = 0U;

	test_rng_init(&rng, 0x1F5EEDU);

	/* Pass 1: pure random bytes. */
	for (round = 0U; round < 4000U; round++) {
		char buf[128];
		size_t n = test_rng_below(&rng, sizeof(buf)) + 1U;
		int rc;

		test_rng_fill(&rng, (uint8_t *)buf, n);
		rc = mp_json_parse(&g_p, buf, n, g_tok, TOKS, 0U);
		if (rc > 0) {
			accepted++;
			walk_tokens(&g_p, rc);
		} else {
			rejected++;
			TEST_ASSERT_TRUE(rc < 0);
		}
	}
	TEST_ASSERT_TRUE(rejected > 0U);

	/* Pass 2: fragment soup — much more likely to parse. */
	for (round = 0U; round < 20000U; round++) {
		char buf[192];
		size_t used = 0U;
		unsigned int pieces = (unsigned int)test_rng_below(&rng, 14U) + 1U;
		unsigned int k;
		int rc;

		for (k = 0U; k < pieces; k++) {
			const char *f = frag[test_rng_below(&rng, nfrag)];
			size_t fl = strlen(f);

			if ((used + fl) >= sizeof(buf)) {
				break;
			}
			memcpy(&buf[used], f, fl);
			used += fl;
		}
		if (used == 0U) {
			continue;
		}

		rc = mp_json_parse(&g_p, buf, used, g_tok, TOKS, 0U);
		if (rc > 0) {
			accepted++;
			walk_tokens(&g_p, rc);
			/* An accepted document must also survive re-parsing at a
			 * tighter token budget without inventing tokens. */
			{
				mp_json_tok_t few[8];
				int rc2 = mp_json_parse(&g_p, buf, used, few, 8U,
							0U);

				TEST_ASSERT_TRUE((rc2 == rc) ||
						 (rc2 == -ENOSPC));
			}
		} else {
			rejected++;
		}
	}

	/* The soup must exercise both outcomes, or the fuzz proves nothing. */
	TEST_ASSERT_TRUE(accepted > 100U);
	TEST_ASSERT_TRUE(rejected > 100U);
}

/** The writer must never emit a document its own parser rejects. */
static void test_fuzz_writer_output_is_parseable(void)
{
	test_rng_t rng;
	unsigned int round;

	test_rng_init(&rng, 0x2A2AU);

	for (round = 0U; round < 2000U; round++) {
		char buf[512];
		mp_jw_t w;
		size_t len = 0U;
		unsigned int items = (unsigned int)test_rng_below(&rng, 8U) + 1U;
		unsigned int k;

		TEST_ASSERT_EQUAL_INT(0, mp_jw_init(&w, buf, sizeof(buf)));
		(void)mp_jw_obj_open(&w);
		for (k = 0U; k < items; k++) {
			char key[16];
			uint8_t raw[12];
			size_t rn = test_rng_below(&rng, sizeof(raw));

			(void)snprintf(key, sizeof(key), "k%u", k & 0xFFU);
			switch (test_rng_u32(&rng) % 6U) {
			case 0U:
				test_rng_fill(&rng, raw, rn);
				(void)mp_jw_key(&w, key);
				(void)mp_jw_strn(&w, (const char *)raw, rn);
				break;
			case 1U:
				(void)mp_jw_kv_i64(
					&w, key,
					(int64_t)(int32_t)test_rng_u32(&rng));
				break;
			case 2U:
				(void)mp_jw_kv_u64(&w, key,
						   test_rng_u32(&rng));
				break;
			case 3U:
				(void)mp_jw_kv_bool(&w, key,
						    (test_rng_u32(&rng) & 1U) !=
							    0U);
				break;
			case 4U:
				(void)mp_jw_kv_null(&w, key);
				break;
			default:
				(void)mp_jw_kv_f32(
					&w, key,
					(float)(int32_t)test_rng_u32(&rng) /
						1024.0f,
					(uint8_t)(test_rng_u32(&rng) % 7U));
				break;
			}
		}
		(void)mp_jw_obj_close(&w);

		if (mp_jw_finish(&w, &len) != 0) {
			continue; /* buffer ran out; nothing to check */
		}
		TEST_ASSERT_TRUE_MESSAGE(mp_json_parse(&g_p, buf, len, g_tok,
						       TOKS, 0U) > 0,
					 buf);
	}
}

/* ------------------------------------------------------------------ runner */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_parse_scalars);
	RUN_TEST(test_parse_object);
	RUN_TEST(test_parse_empty_containers);
	RUN_TEST(test_parse_whitespace_is_ignored);
	RUN_TEST(test_rfc8259_examples);

	RUN_TEST(test_parse_rejections);
	RUN_TEST(test_leading_zero_is_tolerated_but_bounded);
	RUN_TEST(test_depth_cap);
	RUN_TEST(test_token_exhaustion);
	RUN_TEST(test_document_too_large);
	RUN_TEST(test_parser_argument_validation);
	RUN_TEST(test_accessor_type_errors);
	RUN_TEST(test_i64_bounds);

	RUN_TEST(test_string_escapes);
	RUN_TEST(test_string_output_space);
	RUN_TEST(test_streq_with_escapes);

	RUN_TEST(test_writer_basic);
	RUN_TEST(test_writer_escaping);
	RUN_TEST(test_writer_float);
	RUN_TEST(test_writer_nan_and_inf_become_null);
	RUN_TEST(test_writer_int64_min);
	RUN_TEST(test_writer_null_string_is_json_null);
	RUN_TEST(test_writer_overflow_latches);
	RUN_TEST(test_writer_structure_errors);
	RUN_TEST(test_writer_argument_validation);
	RUN_TEST(test_writer_raw_and_decimal_clamp);

	RUN_TEST(test_base64_round_trip);
	RUN_TEST(test_base64_binary_round_trip);
	RUN_TEST(test_base64_errors);

	RUN_TEST(test_fuzz_parser);
	RUN_TEST(test_fuzz_writer_output_is_parseable);

	return UNITY_END();
}
