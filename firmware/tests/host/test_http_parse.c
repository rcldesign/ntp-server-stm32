/*
 * STS1000 "Meridian" — core/web http_parse unit tests.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Two halves:
 *
 *   1. Behaviour. Every rule in http_parse.h's threat model gets a case that
 *      would pass if the rule were removed, so the tests can actually fail.
 *   2. Fuzz. The parser and the chunked decoder are the first code an
 *      unauthenticated attacker reaches, so they are driven with random bytes,
 *      with mutated valid requests, and with every truncation of a valid
 *      request. Run the suite with -fsanitize=address,undefined to make an
 *      out-of-bounds read or an overflow a failure rather than a maybe:
 *
 *        TESTS='http_parse' firmware/scripts/test.sh \
 *          -DCMAKE_C_FLAGS='-fsanitize=address,undefined -fno-sanitize-recover=all'
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "web/http_parse.h"
#include "web/web.h"

#include "test_support.h"

/* ------------------------------------------------------------------------- */
/* helpers                                                                   */
/* ------------------------------------------------------------------------- */

static const char *const GOOD_GET =
	"GET /api/v1/status/timing?cursor=7&flag HTTP/1.1\r\n"
	"Host: meridian.local\r\n"
	"User-Agent: test\r\n"
	"Accept-Encoding: br;q=1.0, gzip;q=0.8\r\n"
	"Cookie: other=1; sts_session=deadbeef; last=x\r\n"
	"X-CSRF-Token: 0123456789abcdef0123456789abcdef\r\n"
	"If-None-Match: \"abc123\"\r\n"
	"\r\n";

static int parse(const char *s, http_req_t *r)
{
	return http_parse_request(s, strlen(s), r);
}

/* ------------------------------------------------------------------------- */
/* request line + headers                                                    */
/* ------------------------------------------------------------------------- */

static void test_parse_good_get(void)
{
	http_req_t r;
	char buf[64];

	TEST_ASSERT_EQUAL_INT(0, parse(GOOD_GET, &r));
	TEST_ASSERT_EQUAL_UINT(WEB_METHOD_GET, r.method);
	TEST_ASSERT_EQUAL_UINT(1U, r.version_minor);
	TEST_ASSERT_TRUE(http_span_eq(r.path, "/api/v1/status/timing"));
	TEST_ASSERT_TRUE(http_span_eq(r.query, "cursor=7&flag"));
	TEST_ASSERT_TRUE(http_span_eq(r.host, "meridian.local"));
	TEST_ASSERT_TRUE(http_span_eq_ci(r.host, "MERIDIAN.LOCAL"));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_KEEPALIVE) != 0U);
	TEST_ASSERT_TRUE((r.flags & HTTP_F_GZIP_OK) != 0U);
	TEST_ASSERT_TRUE((r.flags & HTTP_F_HAS_BODY) == 0U);
	TEST_ASSERT_EQUAL_UINT(strlen(GOOD_GET), r.head_len);
	TEST_ASSERT_EQUAL_UINT(6U, r.n_headers);

	TEST_ASSERT_TRUE(http_span_eq(r.if_none_match, "\"abc123\""));
	TEST_ASSERT_EQUAL_INT(8, http_cookie_get(&r, "sts_session", buf,
						 sizeof(buf)));
	TEST_ASSERT_EQUAL_STRING("deadbeef", buf);
	TEST_ASSERT_EQUAL_INT(1, http_cookie_get(&r, "other", buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(1, http_cookie_get(&r, "last", buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      http_cookie_get(&r, "nope", buf, sizeof(buf)));

	TEST_ASSERT_EQUAL_INT(21, http_path_decode(&r, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_STRING("/api/v1/status/timing", buf);
	TEST_ASSERT_EQUAL_INT(32, (int)http_span_str(r.csrf, buf, sizeof(buf)));
}

static void test_parse_incomplete(void)
{
	http_req_t r;

	TEST_ASSERT_EQUAL_INT(-EAGAIN, parse("GE", &r));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, parse("GET / HTTP/1.1\r", &r));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, parse("GET / HTTP/1.1\r\n", &r));
	TEST_ASSERT_EQUAL_INT(-EAGAIN,
			      parse("GET / HTTP/1.1\r\nHost: a\r\n", &r));
	TEST_ASSERT_EQUAL_INT(-EINVAL, http_parse_request(NULL, 0U, &r));
	TEST_ASSERT_EQUAL_INT(-EINVAL, http_parse_request("x", 1U, NULL));
}

static void test_reject_bare_lf(void)
{
	http_req_t r;

	TEST_ASSERT_EQUAL_INT(-EBADMSG, parse("GET / HTTP/1.1\nHost: a\r\n\r\n",
					      &r));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      parse("GET / HTTP/1.1\r\nHost: a\nX: 1\r\n\r\n", &r));
	/* A CR that is not followed by LF is equally ambiguous. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      parse("GET / HTTP/1.1\rX\r\nHost: a\r\n\r\n", &r));
}

static void test_reject_obs_fold(void)
{
	http_req_t r;

	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      parse("GET / HTTP/1.1\r\nHost: a\r\n"
				    " continued\r\n\r\n",
				    &r));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      parse("GET / HTTP/1.1\r\nHost: a\r\n"
				    "\tcontinued\r\n\r\n",
				    &r));
}

static void test_reject_space_before_colon(void)
{
	http_req_t r;

	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      parse("GET / HTTP/1.1\r\nHost : a\r\n\r\n", &r));
	/* A header with no colon at all. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      parse("GET / HTTP/1.1\r\nHost\r\n\r\n", &r));
	/* An empty header name. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      parse("GET / HTTP/1.1\r\n: a\r\n\r\n", &r));
}

static void test_reject_ctl_in_value(void)
{
	http_req_t r;
	char buf[80];
	size_t n;

	n = (size_t)snprintf(buf, sizeof(buf),
			     "GET / HTTP/1.1\r\nHost: a%cb\r\n\r\n", 0x01);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, http_parse_request(buf, n, &r));

	/* An embedded NUL, which is how a value smuggles a second meaning. */
	memcpy(buf, "GET / HTTP/1.1\r\nHost: a\0b\r\n\r\n", 29U);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, http_parse_request(buf, 28U, &r));
}

static void test_framing_conflicts(void)
{
	http_req_t r;

	/* CL + TE is a smuggling attempt, not a preference. */
	TEST_ASSERT_EQUAL_INT(-EPROTO,
			      parse("POST /x HTTP/1.1\r\nHost: a\r\n"
				    "Content-Length: 5\r\n"
				    "Transfer-Encoding: chunked\r\n\r\n",
				    &r));
	/* Reversed order must fail identically. */
	TEST_ASSERT_EQUAL_INT(-EPROTO,
			      parse("POST /x HTTP/1.1\r\nHost: a\r\n"
				    "Transfer-Encoding: chunked\r\n"
				    "Content-Length: 5\r\n\r\n",
				    &r));
	/* Two disagreeing Content-Lengths. */
	TEST_ASSERT_EQUAL_INT(-EPROTO,
			      parse("POST /x HTTP/1.1\r\nHost: a\r\n"
				    "Content-Length: 5\r\n"
				    "Content-Length: 6\r\n\r\n",
				    &r));
	/* Two agreeing ones are merely redundant. */
	TEST_ASSERT_EQUAL_INT(0, parse("POST /x HTTP/1.1\r\nHost: a\r\n"
				       "Content-Length: 5\r\n"
				       "Content-Length: 5\r\n\r\n",
				       &r));
	TEST_ASSERT_EQUAL_UINT(5U, r.content_length);
	TEST_ASSERT_TRUE((r.flags & HTTP_F_HAS_BODY) != 0U);

	/* A coding we cannot apply. */
	TEST_ASSERT_EQUAL_INT(-ENOTSUP,
			      parse("POST /x HTTP/1.1\r\nHost: a\r\n"
				    "Transfer-Encoding: gzip, chunked\r\n\r\n",
				    &r));
	/* Non-numeric / over-wide Content-Length. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      parse("POST /x HTTP/1.1\r\nHost: a\r\n"
				    "Content-Length: 5x\r\n\r\n",
				    &r));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      parse("POST /x HTTP/1.1\r\nHost: a\r\n"
				    "Content-Length: \r\n\r\n",
				    &r));
	TEST_ASSERT_EQUAL_INT(-E2BIG,
			      parse("POST /x HTTP/1.1\r\nHost: a\r\n"
				    "Content-Length: 4294967296\r\n\r\n",
				    &r));
}

static void test_chunked_flag(void)
{
	http_req_t r;

	TEST_ASSERT_EQUAL_INT(0, parse("POST /x HTTP/1.1\r\nHost: a\r\n"
				       "Transfer-Encoding: chunked\r\n\r\n",
				       &r));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_CHUNKED) != 0U);
	TEST_ASSERT_TRUE((r.flags & HTTP_F_HAS_BODY) != 0U);
	TEST_ASSERT_EQUAL_UINT(0U, r.content_length);
}

static void test_versions_and_host(void)
{
	http_req_t r;

	/* HTTP/1.0 needs no Host and defaults to close. */
	TEST_ASSERT_EQUAL_INT(0, parse("GET / HTTP/1.0\r\n\r\n", &r));
	TEST_ASSERT_EQUAL_UINT(0U, r.version_minor);
	TEST_ASSERT_TRUE((r.flags & HTTP_F_KEEPALIVE) == 0U);

	TEST_ASSERT_EQUAL_INT(0, parse("GET / HTTP/1.0\r\n"
				       "Connection: keep-alive\r\n\r\n",
				       &r));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_KEEPALIVE) != 0U);

	/* HTTP/1.1 without Host. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG, parse("GET / HTTP/1.1\r\n\r\n", &r));

	TEST_ASSERT_EQUAL_INT(0, parse("GET / HTTP/1.1\r\nHost: a\r\n"
				       "Connection: close\r\n\r\n",
				       &r));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_KEEPALIVE) == 0U);

	TEST_ASSERT_EQUAL_INT(-ENOTSUP, parse("GET / HTTP/2.0\r\n\r\n", &r));
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, parse("GET / HTTP/1.9\r\n\r\n", &r));
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, parse("GET / HTTP/1.1x\r\n\r\n", &r));
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, parse("GET / SPDY\r\n\r\n", &r));
}

static void test_request_line_syntax(void)
{
	http_req_t r;
	static char big[HTTP_TARGET_MAX + 128U];
	size_t o;

	/* No target. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG, parse("GET\r\n\r\n", &r));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, parse("GET \r\n\r\n", &r));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, parse("GET  HTTP/1.1\r\n\r\n", &r));
	/* No version. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG, parse("GET /\r\n\r\n", &r));
	/* Empty method. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG, parse(" / HTTP/1.1\r\n\r\n", &r));
	/* Non-token method character. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG, parse("GE(T / HTTP/1.1\r\n\r\n", &r));
	/* Absolute-form (proxy) target. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      parse("GET http://h/x HTTP/1.1\r\nHost: h\r\n\r\n",
				    &r));
	/* Authority-form. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      parse("CONNECT h:443 HTTP/1.1\r\nHost: h\r\n\r\n",
				    &r));
	/* Raw control byte in the target. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      parse("GET /a\x01\x62 HTTP/1.1\r\nHost: h\r\n\r\n",
				    &r));
	/* Raw high byte in the target: must be percent-encoded. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      parse("GET /\xC3\xA9 HTTP/1.1\r\nHost: h\r\n\r\n",
				    &r));

	/* An unknown method parses, so the router can answer 405. */
	TEST_ASSERT_EQUAL_INT(0, parse("PATCH / HTTP/1.1\r\nHost: a\r\n\r\n", &r));
	TEST_ASSERT_EQUAL_UINT(WEB_METHOD_UNKNOWN, r.method);

	/* Target over the limit. */
	o = (size_t)snprintf(big, sizeof(big), "GET /");
	memset(&big[o], 'a', HTTP_TARGET_MAX + 1U);
	o += HTTP_TARGET_MAX + 1U;
	o += (size_t)snprintf(&big[o], sizeof(big) - o,
			      " HTTP/1.1\r\nHost: a\r\n\r\n");
	TEST_ASSERT_EQUAL_INT(-E2BIG, http_parse_request(big, o, &r));
}

static void test_limits(void)
{
	static char buf[HTTP_HEAD_MAX * 2U];
	http_req_t r;
	size_t o;
	unsigned int i;

	/* Too many headers. */
	o = (size_t)snprintf(buf, sizeof(buf), "GET / HTTP/1.1\r\nHost: a\r\n");
	for (i = 0U; i < HTTP_HEADERS_MAX + 4U; i++) {
		o += (size_t)snprintf(&buf[o], sizeof(buf) - o, "X-%u: v\r\n", i);
	}
	o += (size_t)snprintf(&buf[o], sizeof(buf) - o, "\r\n");
	TEST_ASSERT_EQUAL_INT(-E2BIG, http_parse_request(buf, o, &r));

	/* One header line over the limit. */
	o = (size_t)snprintf(buf, sizeof(buf), "GET / HTTP/1.1\r\nHost: a\r\nX: ");
	memset(&buf[o], 'v', HTTP_HEADER_LINE_MAX + 8U);
	o += HTTP_HEADER_LINE_MAX + 8U;
	o += (size_t)snprintf(&buf[o], sizeof(buf) - o, "\r\n\r\n");
	TEST_ASSERT_EQUAL_INT(-E2BIG, http_parse_request(buf, o, &r));

	/* No head terminator inside HTTP_HEAD_MAX of a longer buffer. */
	o = (size_t)snprintf(buf, sizeof(buf), "GET / HTTP/1.1\r\nHost: a\r\n");
	while (o < (HTTP_HEAD_MAX + 64U)) {
		o += (size_t)snprintf(&buf[o], sizeof(buf) - o, "X: v\r\n");
	}
	TEST_ASSERT_EQUAL_INT(-E2BIG, http_parse_request(buf, o, &r));

	/* A complete head early in an over-long buffer parses fine — the caller
	 * hands us its whole receive buffer, body included. */
	o = (size_t)snprintf(buf, sizeof(buf),
			     "POST /x HTTP/1.1\r\nHost: a\r\n"
			     "Content-Length: 4000\r\n\r\n");
	memset(&buf[o], 'B', 4000U);
	o += 4000U;
	TEST_ASSERT_TRUE(o > HTTP_HEAD_MAX);
	TEST_ASSERT_EQUAL_INT(0, http_parse_request(buf, o, &r));
	TEST_ASSERT_EQUAL_UINT(4000U, r.content_length);
}

static void test_websocket_upgrade(void)
{
	http_req_t r;

	static const char *const good =
		"GET /ws HTTP/1.1\r\n"
		"Host: a\r\n"
		"Connection: keep-alive, Upgrade\r\n"
		"Upgrade: websocket\r\n"
		"Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
		"Sec-WebSocket-Version: 13\r\n"
		"Sec-WebSocket-Protocol: sts.telemetry.v1\r\n"
		"\r\n";

	TEST_ASSERT_EQUAL_INT(0, parse(good, &r));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_UPGRADE_WS) != 0U);
	TEST_ASSERT_TRUE(http_span_eq(r.ws_key, "dGhlIHNhbXBsZSBub25jZQ=="));
	TEST_ASSERT_TRUE(http_span_eq(r.ws_protocol, "sts.telemetry.v1"));

	/* Each precondition removed in turn must leave it a plain GET. */
	TEST_ASSERT_EQUAL_INT(0, parse("GET /ws HTTP/1.1\r\nHost: a\r\n"
				       "Connection: Upgrade\r\n"
				       "Upgrade: websocket\r\n"
				       "Sec-WebSocket-Key: k\r\n"
				       "Sec-WebSocket-Version: 8\r\n\r\n",
				       &r));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_UPGRADE_WS) == 0U);

	TEST_ASSERT_EQUAL_INT(0, parse("GET /ws HTTP/1.1\r\nHost: a\r\n"
				       "Connection: Upgrade\r\n"
				       "Upgrade: h2c\r\n"
				       "Sec-WebSocket-Key: k\r\n"
				       "Sec-WebSocket-Version: 13\r\n\r\n",
				       &r));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_UPGRADE_WS) == 0U);

	TEST_ASSERT_EQUAL_INT(0, parse("GET /ws HTTP/1.1\r\nHost: a\r\n"
				       "Upgrade: websocket\r\n"
				       "Sec-WebSocket-Key: k\r\n"
				       "Sec-WebSocket-Version: 13\r\n\r\n",
				       &r));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_UPGRADE_WS) == 0U);

	TEST_ASSERT_EQUAL_INT(0, parse("GET /ws HTTP/1.1\r\nHost: a\r\n"
				       "Connection: Upgrade\r\n"
				       "Upgrade: websocket\r\n"
				       "Sec-WebSocket-Version: 13\r\n\r\n",
				       &r));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_UPGRADE_WS) == 0U);

	/* POST cannot upgrade. */
	TEST_ASSERT_EQUAL_INT(0, parse("POST /ws HTTP/1.1\r\nHost: a\r\n"
				       "Connection: Upgrade\r\n"
				       "Upgrade: websocket\r\n"
				       "Sec-WebSocket-Key: k\r\n"
				       "Sec-WebSocket-Version: 13\r\n\r\n",
				       &r));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_UPGRADE_WS) == 0U);

	/* HTTP/1.0 cannot upgrade either. */
	TEST_ASSERT_EQUAL_INT(0, parse("GET /ws HTTP/1.0\r\n"
				       "Connection: Upgrade\r\n"
				       "Upgrade: websocket\r\n"
				       "Sec-WebSocket-Key: k\r\n"
				       "Sec-WebSocket-Version: 13\r\n\r\n",
				       &r));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_UPGRADE_WS) == 0U);
}

static void test_expect_and_misc_headers(void)
{
	http_req_t r;
	char buf[64];

	TEST_ASSERT_EQUAL_INT(0, parse("POST /x HTTP/1.1\r\nHost: a\r\n"
				       "Expect: 100-continue\r\n"
				       "Content-Type: application/json\r\n"
				       "Content-Length: 2\r\n"
				       "Authorization: Bearer  tok123\r\n"
				       "Origin: https://a\r\n"
				       "Referer: https://a/x\r\n\r\n",
				       &r));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_EXPECT_100) != 0U);
	TEST_ASSERT_TRUE(http_span_eq(r.content_type, "application/json"));
	TEST_ASSERT_TRUE(http_span_eq(r.origin, "https://a"));
	TEST_ASSERT_TRUE(http_span_eq(r.referer, "https://a/x"));
	TEST_ASSERT_TRUE(http_bearer_token(&r, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_STRING("tok123", buf);

	/* Expect with another value must not set the flag. */
	TEST_ASSERT_EQUAL_INT(0, parse("POST /x HTTP/1.1\r\nHost: a\r\n"
				       "Expect: something\r\n\r\n",
				       &r));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_EXPECT_100) == 0U);

	/* Bearer variants. */
	TEST_ASSERT_EQUAL_INT(0, parse("GET / HTTP/1.1\r\nHost: a\r\n"
				       "Authorization: Basic abc\r\n\r\n",
				       &r));
	TEST_ASSERT_FALSE(http_bearer_token(&r, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(0, parse("GET / HTTP/1.1\r\nHost: a\r\n"
				       "Authorization: Bearer\r\n\r\n",
				       &r));
	TEST_ASSERT_FALSE(http_bearer_token(&r, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(0, parse("GET / HTTP/1.1\r\nHost: a\r\n"
				       "Authorization: Bearer   \r\n\r\n",
				       &r));
	TEST_ASSERT_FALSE(http_bearer_token(&r, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(0, parse("GET / HTTP/1.1\r\nHost: a\r\n\r\n", &r));
	TEST_ASSERT_FALSE(http_bearer_token(&r, buf, sizeof(buf)));
	TEST_ASSERT_FALSE(http_bearer_token(NULL, buf, sizeof(buf)));
	TEST_ASSERT_FALSE(http_bearer_token(&r, NULL, 4U));

	/* A bearer token that does not fit is refused, never truncated. */
	TEST_ASSERT_EQUAL_INT(0, parse("GET / HTTP/1.1\r\nHost: a\r\n"
				       "Authorization: Bearer abcdefghij\r\n\r\n",
				       &r));
	TEST_ASSERT_FALSE(http_bearer_token(&r, buf, 4U));

	/* No gzip advertised. */
	TEST_ASSERT_EQUAL_INT(0, parse("GET / HTTP/1.1\r\nHost: a\r\n"
				       "Accept-Encoding: deflate, br\r\n\r\n",
				       &r));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_GZIP_OK) == 0U);
	TEST_ASSERT_EQUAL_INT(0, parse("GET / HTTP/1.1\r\nHost: a\r\n"
				       "Accept-Encoding: gzip\r\n\r\n",
				       &r));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_GZIP_OK) != 0U);
	/* An empty value list matches nothing. */
	TEST_ASSERT_EQUAL_INT(0, parse("GET / HTTP/1.1\r\nHost: a\r\n"
				       "Accept-Encoding:\r\n\r\n",
				       &r));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_GZIP_OK) == 0U);
	/* A list element that is only a parameter must not match. */
	TEST_ASSERT_EQUAL_INT(0, parse("GET / HTTP/1.1\r\nHost: a\r\n"
				       "Accept-Encoding: ;gzip\r\n\r\n",
				       &r));
	TEST_ASSERT_TRUE((r.flags & HTTP_F_GZIP_OK) == 0U);
}

/* ------------------------------------------------------------------------- */
/* percent decoding, query, cookies                                          */
/* ------------------------------------------------------------------------- */

static void test_pct_decode(void)
{
	char out[16];

	TEST_ASSERT_EQUAL_INT(3, http_pct_decode("a%20b", 5U, out, sizeof(out)));
	TEST_ASSERT_EQUAL_STRING("a b", out);
	TEST_ASSERT_EQUAL_INT(1, http_pct_decode("%2F", 3U, out, sizeof(out)));
	TEST_ASSERT_EQUAL_STRING("/", out);
	TEST_ASSERT_EQUAL_INT(1, http_pct_decode("%2f", 3U, out, sizeof(out)));
	TEST_ASSERT_EQUAL_INT(0, http_pct_decode("", 0U, out, sizeof(out)));
	/* '+' stays literal (see the header rationale). */
	TEST_ASSERT_EQUAL_INT(3, http_pct_decode("a+b", 3U, out, sizeof(out)));
	TEST_ASSERT_EQUAL_STRING("a+b", out);

	TEST_ASSERT_EQUAL_INT(-EILSEQ, http_pct_decode("%00", 3U, out,
						       sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, http_pct_decode("%2", 2U, out,
						       sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, http_pct_decode("%", 1U, out,
						       sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, http_pct_decode("%zz", 3U, out,
						       sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, http_pct_decode("%2z", 3U, out,
						       sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, http_pct_decode("abcd", 4U, out, 4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, http_pct_decode("a", 1U, NULL, 4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, http_pct_decode("a", 1U, out, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, http_pct_decode(NULL, 1U, out,
						       sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, http_path_decode(NULL, out, sizeof(out)));
}

static void test_query(void)
{
	http_req_t r;
	char out[16];
	uint32_t v = 0U;

	TEST_ASSERT_EQUAL_INT(0, parse("GET /x?a=1&bb=%41%42&c=&d HTTP/1.1\r\n"
				       "Host: h\r\n\r\n",
				       &r));
	TEST_ASSERT_EQUAL_INT(1, http_query_get(&r, "a", out, sizeof(out)));
	TEST_ASSERT_EQUAL_STRING("1", out);
	TEST_ASSERT_EQUAL_INT(2, http_query_get(&r, "bb", out, sizeof(out)));
	TEST_ASSERT_EQUAL_STRING("AB", out);
	TEST_ASSERT_EQUAL_INT(0, http_query_get(&r, "c", out, sizeof(out)));
	TEST_ASSERT_EQUAL_INT(0, http_query_get(&r, "d", out, sizeof(out)));
	TEST_ASSERT_TRUE(http_query_has(&r, "d"));
	TEST_ASSERT_FALSE(http_query_has(&r, "e"));
	TEST_ASSERT_FALSE(http_query_has(&r, ""));
	TEST_ASSERT_FALSE(http_query_has(NULL, "a"));
	TEST_ASSERT_EQUAL_INT(-ENOENT, http_query_get(&r, "e", out, sizeof(out)));
	/* A prefix of a real name must not match. */
	TEST_ASSERT_EQUAL_INT(-ENOENT, http_query_get(&r, "b", out, sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, http_query_get(&r, "a", NULL, 4U));

	TEST_ASSERT_EQUAL_INT(0, http_query_get_u32(&r, "a", &v));
	TEST_ASSERT_EQUAL_UINT32(1U, v);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, http_query_get_u32(&r, "bb", &v));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, http_query_get_u32(&r, "c", &v));
	TEST_ASSERT_EQUAL_INT(-ENOENT, http_query_get_u32(&r, "e", &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, http_query_get_u32(&r, "a", NULL));

	TEST_ASSERT_EQUAL_INT(0, parse("GET /x?n=4294967295&o=4294967296&"
				       "p=99999999999999999999&q=%zz HTTP/1.1\r\n"
				       "Host: h\r\n\r\n",
				       &r));
	TEST_ASSERT_EQUAL_INT(0, http_query_get_u32(&r, "n", &v));
	TEST_ASSERT_EQUAL_UINT32(4294967295U, v);
	TEST_ASSERT_EQUAL_INT(-ERANGE, http_query_get_u32(&r, "o", &v));
	TEST_ASSERT_EQUAL_INT(-ERANGE, http_query_get_u32(&r, "p", &v));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, http_query_get_u32(&r, "q", &v));

	/* No query at all. */
	TEST_ASSERT_EQUAL_INT(0, parse("GET /x HTTP/1.1\r\nHost: h\r\n\r\n", &r));
	TEST_ASSERT_EQUAL_UINT(0U, r.query.n);
	TEST_ASSERT_EQUAL_INT(-ENOENT, http_query_get(&r, "a", out, sizeof(out)));
}

static void test_cookie_edges(void)
{
	http_req_t r;
	char out[8];

	TEST_ASSERT_EQUAL_INT(0, parse("GET / HTTP/1.1\r\nHost: h\r\n"
				       "Cookie: novalue; k=v; ; k2=vv\r\n\r\n",
				       &r));
	TEST_ASSERT_EQUAL_INT(1, http_cookie_get(&r, "k", out, sizeof(out)));
	TEST_ASSERT_EQUAL_STRING("v", out);
	TEST_ASSERT_EQUAL_INT(2, http_cookie_get(&r, "k2", out, sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-ENOENT, http_cookie_get(&r, "novalue", out,
						       sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, http_cookie_get(&r, "", out, sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, http_cookie_get(NULL, "k", out,
						       sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, http_cookie_get(&r, "k", NULL, 4U));

	TEST_ASSERT_EQUAL_INT(0, parse("GET / HTTP/1.1\r\nHost: h\r\n"
				       "Cookie: k=0123456789\r\n\r\n",
				       &r));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, http_cookie_get(&r, "k", out,
						       sizeof(out)));

	/* Empty cookie value. */
	TEST_ASSERT_EQUAL_INT(0, parse("GET / HTTP/1.1\r\nHost: h\r\n"
				       "Cookie: k=\r\n\r\n",
				       &r));
	TEST_ASSERT_EQUAL_INT(0, http_cookie_get(&r, "k", out, sizeof(out)));
	TEST_ASSERT_EQUAL_STRING("", out);

	/* No Cookie header. */
	TEST_ASSERT_EQUAL_INT(0, parse("GET / HTTP/1.1\r\nHost: h\r\n\r\n", &r));
	TEST_ASSERT_EQUAL_INT(-ENOENT, http_cookie_get(&r, "k", out,
						       sizeof(out)));
}

/* ------------------------------------------------------------------------- */
/* chunked decoding                                                          */
/* ------------------------------------------------------------------------- */

static void test_chunked_basic(void)
{
	http_chunked_t st;
	uint8_t out[64];
	size_t out_len = 0U;
	size_t consumed = 0U;
	static const char *const enc =
		"4\r\nWiki\r\n"
		"5;name=value\r\npedia\r\n"
		"0\r\n"
		"X-Trailer: v\r\n"
		"\r\n";

	http_chunked_init(&st, sizeof(out));
	TEST_ASSERT_EQUAL_INT(1, http_chunked_feed(&st, enc, strlen(enc),
						   &consumed, out, sizeof(out),
						   &out_len));
	TEST_ASSERT_EQUAL_UINT(strlen(enc), consumed);
	TEST_ASSERT_EQUAL_UINT(9U, out_len);
	TEST_ASSERT_EQUAL_MEMORY("Wikipedia", out, 9U);

	/* A completed decoder keeps answering 1. */
	TEST_ASSERT_EQUAL_INT(1, http_chunked_feed(&st, "junk", 4U, &consumed,
						   out, sizeof(out), &out_len));

	/* The minimal body: just the terminator. */
	http_chunked_init(&st, sizeof(out));
	out_len = 0U;
	TEST_ASSERT_EQUAL_INT(1, http_chunked_feed(&st, "0\r\n\r\n", 5U,
						   &consumed, out, sizeof(out),
						   &out_len));
	TEST_ASSERT_EQUAL_UINT(0U, out_len);
}

static void test_chunked_byte_at_a_time(void)
{
	http_chunked_t st;
	uint8_t out[64];
	size_t out_len = 0U;
	static const char *const enc = "3\r\nabc\r\n0\r\n\r\n";
	size_t i;
	int rc = 0;

	http_chunked_init(&st, sizeof(out));
	for (i = 0U; i < strlen(enc); i++) {
		size_t consumed = 0U;

		rc = http_chunked_feed(&st, &enc[i], 1U, &consumed, out,
				       sizeof(out), &out_len);
		TEST_ASSERT_TRUE(rc >= 0);
		TEST_ASSERT_EQUAL_UINT(1U, consumed);
		if (rc == 1) {
			break;
		}
	}
	TEST_ASSERT_EQUAL_INT(1, rc);
	TEST_ASSERT_EQUAL_UINT(3U, out_len);
	TEST_ASSERT_EQUAL_MEMORY("abc", out, 3U);
}

static void test_chunked_errors(void)
{
	http_chunked_t st;
	uint8_t out[16];
	size_t out_len;
	size_t consumed;

	/* Non-hex size. */
	http_chunked_init(&st, sizeof(out));
	out_len = 0U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      http_chunked_feed(&st, "z\r\n", 3U, &consumed, out,
						sizeof(out), &out_len));
	/* The error latches. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      http_chunked_feed(&st, "1\r\na\r\n", 6U, &consumed,
						out, sizeof(out), &out_len));

	/* Empty size line. */
	http_chunked_init(&st, sizeof(out));
	out_len = 0U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      http_chunked_feed(&st, "\r\n", 2U, &consumed, out,
						sizeof(out), &out_len));

	/* CR not followed by LF in the size line. */
	http_chunked_init(&st, sizeof(out));
	out_len = 0U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      http_chunked_feed(&st, "1\rx", 3U, &consumed, out,
						sizeof(out), &out_len));

	/* Extension before any size digit. */
	http_chunked_init(&st, sizeof(out));
	out_len = 0U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      http_chunked_feed(&st, ";x\r\n", 4U, &consumed, out,
						sizeof(out), &out_len));

	/* CR without LF closing an extension. */
	http_chunked_init(&st, sizeof(out));
	out_len = 0U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      http_chunked_feed(&st, "1;a\rZ", 5U, &consumed, out,
						sizeof(out), &out_len));

	/* Missing CRLF after chunk-data. */
	http_chunked_init(&st, sizeof(out));
	out_len = 0U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      http_chunked_feed(&st, "1\r\naX", 5U, &consumed, out,
						sizeof(out), &out_len));
	http_chunked_init(&st, sizeof(out));
	out_len = 0U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      http_chunked_feed(&st, "1\r\na\rX", 6U, &consumed,
						out, sizeof(out), &out_len));

	/* Control byte inside an extension. */
	http_chunked_init(&st, sizeof(out));
	out_len = 0U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      http_chunked_feed(&st, "1;\x01", 3U, &consumed, out,
						sizeof(out), &out_len));

	/* Trailer line that does not end in CRLF. */
	http_chunked_init(&st, sizeof(out));
	out_len = 0U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      http_chunked_feed(&st, "0\r\nA\rB", 6U, &consumed,
						out, sizeof(out), &out_len));

	/* Control byte in a trailer line. */
	http_chunked_init(&st, sizeof(out));
	out_len = 0U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      http_chunked_feed(&st, "0\r\nA\x01", 5U, &consumed,
						out, sizeof(out), &out_len));

	/* Bad argument handling. */
	http_chunked_init(&st, sizeof(out));
	out_len = 0U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, http_chunked_feed(&st, "0", 1U, NULL, out,
							 sizeof(out), NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, http_chunked_feed(NULL, "0", 1U, NULL, out,
							 sizeof(out), &out_len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, http_chunked_feed(&st, NULL, 1U, NULL, out,
							 sizeof(out), &out_len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, http_chunked_feed(&st, "0", 1U, NULL, NULL,
							 4U, &out_len));
	/* Zero-length feed on a fresh decoder is a no-op, not an error. */
	TEST_ASSERT_EQUAL_INT(0, http_chunked_feed(&st, NULL, 0U, &consumed, NULL,
						   0U, &out_len));
	http_chunked_init(NULL, 0U); /* must not crash */
}

static void test_chunked_size_limits(void)
{
	http_chunked_t st;
	uint8_t out[16];
	size_t out_len;
	size_t consumed;

	/* A declared chunk bigger than the ceiling is refused up front. */
	http_chunked_init(&st, 8U);
	out_len = 0U;
	TEST_ASSERT_EQUAL_INT(-E2BIG,
			      http_chunked_feed(&st, "FF\r\n", 4U, &consumed, out,
						sizeof(out), &out_len));

	/* A size field wider than 8 hex digits cannot fit a uint32. */
	http_chunked_init(&st, 0U);
	out_len = 0U;
	TEST_ASSERT_EQUAL_INT(-E2BIG,
			      http_chunked_feed(&st, "111111111\r\n", 11U,
						&consumed, out, sizeof(out),
						&out_len));

	/* Two chunks that individually fit but together exceed the total. */
	http_chunked_init(&st, 6U);
	out_len = 0U;
	TEST_ASSERT_EQUAL_INT(0, http_chunked_feed(&st, "4\r\nabcd\r\n", 9U,
						   &consumed, out, sizeof(out),
						   &out_len));
	TEST_ASSERT_EQUAL_INT(-E2BIG,
			      http_chunked_feed(&st, "4\r\nefgh\r\n", 9U,
						&consumed, out, sizeof(out),
						&out_len));

	/* Output buffer too small. */
	http_chunked_init(&st, 64U);
	out_len = 0U;
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      http_chunked_feed(&st, "8\r\nabcdefgh\r\n", 13U,
						&consumed, out, 4U, &out_len));

	/* Trailer section over the line limit. */
	{
		static char big[HTTP_HEADER_LINE_MAX + 64U];
		size_t o;

		http_chunked_init(&st, 64U);
		out_len = 0U;
		memcpy(big, "0\r\n", 3U);
		o = 3U;
		memset(&big[o], 'T', sizeof(big) - o);
		TEST_ASSERT_EQUAL_INT(-E2BIG,
				      http_chunked_feed(&st, big, sizeof(big),
							&consumed, out,
							sizeof(out), &out_len));
	}
}

/* ------------------------------------------------------------------------- */
/* fuzz                                                                      */
/* ------------------------------------------------------------------------- */

/*
 * Invariants any accepted parse must satisfy. If the parser ever reports
 * success with a span pointing outside the input, this catches it even without
 * a sanitizer; with ASan it also catches the read that produced it.
 */
static void check_req_invariants(const char *buf, size_t len,
				 const http_req_t *r)
{
	const char *end = buf + len;

	TEST_ASSERT_TRUE(r->head_len <= len);
	TEST_ASSERT_TRUE(r->target.p >= buf && r->target.p <= end);
	TEST_ASSERT_TRUE((r->target.p + r->target.n) <= end);
	TEST_ASSERT_TRUE(r->path.p >= buf && (r->path.p + r->path.n) <= end);
	TEST_ASSERT_TRUE(r->query.p >= buf && (r->query.p + r->query.n) <= end);
	TEST_ASSERT_TRUE(r->target.n <= HTTP_TARGET_MAX);
	TEST_ASSERT_TRUE(r->n_headers <= HTTP_HEADERS_MAX);
	if (r->host.p != NULL) {
		TEST_ASSERT_TRUE(r->host.p >= buf &&
				 (r->host.p + r->host.n) <= end);
	}
	if (r->cookie.p != NULL) {
		TEST_ASSERT_TRUE((r->cookie.p + r->cookie.n) <= end);
	}
	if (r->authorization.p != NULL) {
		TEST_ASSERT_TRUE((r->authorization.p + r->authorization.n) <= end);
	}
	if (r->ws_key.p != NULL) {
		TEST_ASSERT_TRUE((r->ws_key.p + r->ws_key.n) <= end);
	}
	/* Framing must never be self-contradictory on an accepted request. */
	if ((r->flags & HTTP_F_CHUNKED) != 0U) {
		TEST_ASSERT_EQUAL_UINT(0U, r->content_length);
	}
}

/* Exercise the accessors on whatever came out, to catch OOB reads there too. */
static void poke_accessors(const http_req_t *r)
{
	char out[64];
	uint32_t v;

	(void)http_path_decode(r, out, sizeof(out));
	(void)http_query_get(r, "cursor", out, sizeof(out));
	(void)http_query_get(r, "secrets", out, sizeof(out));
	(void)http_query_get_u32(r, "max", &v);
	(void)http_query_has(r, "x");
	(void)http_cookie_get(r, "sts_session", out, sizeof(out));
	(void)http_bearer_token(r, out, sizeof(out));
	(void)http_span_str(r->host, out, sizeof(out));
	(void)http_span_eq_ci(r->content_type, "application/json");
}

static void test_fuzz_random_bytes(void)
{
	test_rng_t g;
	uint8_t buf[512];
	unsigned int iter;

	test_rng_init(&g, 0xC0FFEEU);
	for (iter = 0U; iter < 40000U; iter++) {
		http_req_t r;
		size_t len = 1U + test_rng_below(&g, sizeof(buf) - 1U);
		int rc;

		test_rng_fill(&g, buf, len);
		rc = http_parse_request((const char *)buf, len, &r);
		TEST_ASSERT_TRUE(rc == 0 || rc == -EAGAIN || rc == -EBADMSG ||
				 rc == -E2BIG || rc == -ENOTSUP || rc == -EPROTO);
		if (rc == 0) {
			check_req_invariants((const char *)buf, len, &r);
			poke_accessors(&r);
		}
	}
}

/* Structured fuzz: mutate a valid request so the parser gets deep, not just
 * rejected at byte 0. */
static void test_fuzz_mutated_requests(void)
{
	static const char *const seeds[] = {
		"GET /api/v1/status HTTP/1.1\r\nHost: h\r\n\r\n",
		"POST /api/v1/config HTTP/1.1\r\nHost: h\r\n"
		"Content-Type: application/json\r\nContent-Length: 9\r\n"
		"X-CSRF-Token: abc\r\n\r\n{\"a\":true}",
		"GET /ws HTTP/1.1\r\nHost: h\r\nConnection: Upgrade\r\n"
		"Upgrade: websocket\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
		"Sec-WebSocket-Version: 13\r\n\r\n",
		"POST /api/v1/firmware/data?offset=0 HTTP/1.1\r\nHost: h\r\n"
		"Transfer-Encoding: chunked\r\n\r\n4\r\nWiki\r\n0\r\n\r\n",
		"GET /api/v1/config?group=1&secrets=1 HTTP/1.1\r\nHost: h\r\n"
		"Cookie: sts_session=abcdef; x=1\r\n"
		"Authorization: Bearer 0123456789\r\n\r\n",
	};
	test_rng_t g;
	char buf[1024];
	unsigned int iter;
	size_t s;

	test_rng_init(&g, 0x5EEDU);
	for (iter = 0U; iter < 40000U; iter++) {
		http_req_t r;
		size_t len;
		unsigned int muts;
		unsigned int m;
		int rc;

		s = test_rng_below(&g, sizeof(seeds) / sizeof(seeds[0]));
		len = strlen(seeds[s]);
		memcpy(buf, seeds[s], len);

		muts = 1U + (unsigned int)test_rng_below(&g, 6U);
		for (m = 0U; m < muts; m++) {
			size_t at = test_rng_below(&g, len);
			uint32_t how = test_rng_u32(&g) % 3U;

			if (how == 0U) {
				buf[at] = (char)(test_rng_u32(&g) & 0xFFU);
			} else if (how == 1U) {
				len = at + 1U; /* truncate */
			} else if ((len + 1U) < sizeof(buf)) {
				memmove(&buf[at + 1U], &buf[at], len - at);
				len++;
			}
		}

		rc = http_parse_request(buf, len, &r);
		TEST_ASSERT_TRUE(rc == 0 || rc == -EAGAIN || rc == -EBADMSG ||
				 rc == -E2BIG || rc == -ENOTSUP || rc == -EPROTO);
		if (rc == 0) {
			check_req_invariants(buf, len, &r);
			poke_accessors(&r);
		}
	}
}

/* Every prefix of a valid request must be EAGAIN or a clean rejection. */
static void test_fuzz_truncations(void)
{
	size_t full = strlen(GOOD_GET);
	size_t n;

	for (n = 0U; n <= full; n++) {
		http_req_t r;
		int rc = http_parse_request(GOOD_GET, n, &r);

		if (n == full) {
			TEST_ASSERT_EQUAL_INT(0, rc);
			continue;
		}
		TEST_ASSERT_TRUE(rc == -EAGAIN || rc == -EBADMSG ||
				 rc == -EINVAL);
	}
}

static void test_fuzz_chunked(void)
{
	test_rng_t g;
	uint8_t enc[256];
	uint8_t out[256];
	unsigned int iter;

	test_rng_init(&g, 0xBEEF01U);
	for (iter = 0U; iter < 40000U; iter++) {
		http_chunked_t st;
		size_t len = 1U + test_rng_below(&g, sizeof(enc) - 1U);
		size_t out_len = 0U;
		size_t consumed = 0U;
		int rc;

		test_rng_fill(&g, enc, len);
		/* Bias toward hex digits and CRLF so the decoder reaches its
		 * data path rather than dying on the first byte. */
		if ((iter & 1U) != 0U) {
			size_t i;

			for (i = 0U; i < len; i++) {
				if ((enc[i] & 0x03U) == 0U) {
					enc[i] = (uint8_t)"0123456789abcdef"
							 [enc[i] & 0x0FU];
				} else if ((enc[i] & 0x0FU) == 1U) {
					enc[i] = (uint8_t)'\r';
				} else if ((enc[i] & 0x0FU) == 2U) {
					enc[i] = (uint8_t)'\n';
				}
			}
		}

		http_chunked_init(&st, (uint32_t)sizeof(out));
		rc = http_chunked_feed(&st, (const char *)enc, len, &consumed,
				       out, sizeof(out), &out_len);
		TEST_ASSERT_TRUE(rc == 0 || rc == 1 || rc == -EBADMSG ||
				 rc == -E2BIG || rc == -ENOSPC);
		TEST_ASSERT_TRUE(consumed <= len);
		TEST_ASSERT_TRUE(out_len <= sizeof(out));
	}
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_parse_good_get);
	RUN_TEST(test_parse_incomplete);
	RUN_TEST(test_reject_bare_lf);
	RUN_TEST(test_reject_obs_fold);
	RUN_TEST(test_reject_space_before_colon);
	RUN_TEST(test_reject_ctl_in_value);
	RUN_TEST(test_framing_conflicts);
	RUN_TEST(test_chunked_flag);
	RUN_TEST(test_versions_and_host);
	RUN_TEST(test_request_line_syntax);
	RUN_TEST(test_limits);
	RUN_TEST(test_websocket_upgrade);
	RUN_TEST(test_expect_and_misc_headers);

	RUN_TEST(test_pct_decode);
	RUN_TEST(test_query);
	RUN_TEST(test_cookie_edges);

	RUN_TEST(test_chunked_basic);
	RUN_TEST(test_chunked_byte_at_a_time);
	RUN_TEST(test_chunked_errors);
	RUN_TEST(test_chunked_size_limits);

	RUN_TEST(test_fuzz_random_bytes);
	RUN_TEST(test_fuzz_mutated_requests);
	RUN_TEST(test_fuzz_truncations);
	RUN_TEST(test_fuzz_chunked);

	return UNITY_END();
}
