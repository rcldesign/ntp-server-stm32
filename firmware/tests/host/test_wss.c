/*
 * STS1000 "Meridian" — core/web wss unit tests.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Covers the SHA-1 the handshake needs against the FIPS 180-4 vectors, the
 * RFC 6455 §1.3 Sec-WebSocket-Accept example, every framing rule in wss.h, the
 * UTF-8 validator against the classic malformed-sequence battery, the send
 * queue's drop policy, and the subscription parser.
 *
 * The frame decoder is attacker-facing, so it is also fuzzed. Sanitizer run:
 *
 *   TESTS='wss' firmware/scripts/test.sh \
 *     -DCMAKE_C_FLAGS='-fsanitize=address,undefined -fno-sanitize-recover=all'
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "web/web.h"
#include "web/wss.h"

#include "test_support.h"

/* ------------------------------------------------------------------------- */
/* SHA-1 + handshake                                                         */
/* ------------------------------------------------------------------------- */

static void test_sha1_vectors(void)
{
	static const uint8_t empty[20] = {
		0xDA, 0x39, 0xA3, 0xEE, 0x5E, 0x6B, 0x4B, 0x0D, 0x32, 0x55,
		0xBF, 0xEF, 0x95, 0x60, 0x18, 0x90, 0xAF, 0xD8, 0x07, 0x09,
	};
	static const uint8_t abc[20] = {
		0xA9, 0x99, 0x3E, 0x36, 0x47, 0x06, 0x81, 0x6A, 0xBA, 0x3E,
		0x25, 0x71, 0x78, 0x50, 0xC2, 0x6C, 0x9C, 0xD0, 0xD8, 0x9D,
	};
	/* "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq" */
	static const uint8_t two_block[20] = {
		0x84, 0x98, 0x3E, 0x44, 0x1C, 0x3B, 0xD2, 0x6E, 0xBA, 0xAE,
		0x4A, 0xA1, 0xF9, 0x51, 0x29, 0xE5, 0xE5, 0x46, 0x70, 0xF1,
	};
	/* 1,000,000 x 'a' */
	static const uint8_t million_a[20] = {
		0x34, 0xAA, 0x97, 0x3C, 0xD4, 0xC4, 0xDA, 0xA4, 0xF6, 0x1E,
		0xEB, 0x2B, 0xDB, 0xAD, 0x27, 0x31, 0x65, 0x34, 0x01, 0x6F,
	};
	uint8_t d[20];
	static uint8_t big[1000000];

	wss_sha1(NULL, 0U, d);
	TEST_ASSERT_EQUAL_MEMORY(empty, d, 20U);
	wss_sha1((const uint8_t *)"", 0U, d);
	TEST_ASSERT_EQUAL_MEMORY(empty, d, 20U);
	wss_sha1((const uint8_t *)"abc", 3U, d);
	TEST_ASSERT_EQUAL_MEMORY(abc, d, 20U);
	wss_sha1((const uint8_t *)"abcdbcdecdefdefgefghfghighijhijkijkljklmklm"
				 "nlmnomnopnopq",
		 56U, d);
	TEST_ASSERT_EQUAL_MEMORY(two_block, d, 20U);

	memset(big, 'a', sizeof(big));
	wss_sha1(big, sizeof(big), d);
	TEST_ASSERT_EQUAL_MEMORY(million_a, d, 20U);

	/* A 55-byte message exercises the "length fits in this block" padding
	 * path that the 56-byte case above does not. */
	wss_sha1(big, 55U, d);
	TEST_ASSERT_EQUAL_HEX8(0xC1, d[0]);
	wss_sha1(NULL, 0U, NULL); /* must not crash */
}

static void test_accept_key_rfc6455(void)
{
	char out[WSS_ACCEPT_LEN + 2U];

	/* RFC 6455 §1.3 worked example. */
	TEST_ASSERT_EQUAL_INT(0, wss_accept_key("dGhlIHNhbXBsZSBub25jZQ==", 24U,
						out, sizeof(out)));
	TEST_ASSERT_EQUAL_STRING("s3pPLMBiTxaQ9kYGzzhZRbK+xOo=", out);

	/* RFC 6455 §4.2.2 example. */
	TEST_ASSERT_EQUAL_INT(0, wss_accept_key("x3JJHMbDL1EzLkh9GBhXDw==", 24U,
						out, sizeof(out)));
	TEST_ASSERT_EQUAL_STRING("HSmrc0sMlYUkAGmm5OPpG2HaGWk=", out);

	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_accept_key(NULL, 4U, out,
						      sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_accept_key("k", 1U, NULL,
						      sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_accept_key("k", 0U, out,
						      sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      wss_accept_key("k", WSS_KEY_MAX + 1U, out,
					     sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, wss_accept_key("k", 1U, out,
						      WSS_ACCEPT_LEN));
}

/* ------------------------------------------------------------------------- */
/* UTF-8                                                                     */
/* ------------------------------------------------------------------------- */

static void test_utf8(void)
{
	static const uint8_t ok2[] = { 0xC2, 0xA2 };             /* U+00A2 */
	static const uint8_t ok3[] = { 0xE2, 0x82, 0xAC };       /* U+20AC */
	static const uint8_t ok4[] = { 0xF0, 0x90, 0x8D, 0x88 }; /* U+10348 */
	static const uint8_t overlong2[] = { 0xC0, 0xAF };
	static const uint8_t overlong3[] = { 0xE0, 0x80, 0xAF };
	static const uint8_t overlong4[] = { 0xF0, 0x80, 0x80, 0xAF };
	static const uint8_t surrogate[] = { 0xED, 0xA0, 0x80 }; /* U+D800 */
	static const uint8_t too_big[] = { 0xF4, 0x90, 0x80, 0x80 };
	static const uint8_t lone_cont[] = { 0x80 };
	static const uint8_t five_byte[] = { 0xF8, 0x88, 0x80, 0x80, 0x80 };
	static const uint8_t truncated[] = { 0xE2, 0x82 };
	static const uint8_t bad_cont[] = { 0xE2, 0x28, 0xA1 };
	static const uint8_t last_valid[] = { 0xF4, 0x8F, 0xBF, 0xBF };

	TEST_ASSERT_TRUE(wss_utf8_valid(NULL, 0U));
	TEST_ASSERT_FALSE(wss_utf8_valid(NULL, 1U));
	TEST_ASSERT_TRUE(wss_utf8_valid((const uint8_t *)"hello", 5U));
	TEST_ASSERT_TRUE(wss_utf8_valid(ok2, sizeof(ok2)));
	TEST_ASSERT_TRUE(wss_utf8_valid(ok3, sizeof(ok3)));
	TEST_ASSERT_TRUE(wss_utf8_valid(ok4, sizeof(ok4)));
	TEST_ASSERT_TRUE(wss_utf8_valid(last_valid, sizeof(last_valid)));

	TEST_ASSERT_FALSE(wss_utf8_valid(overlong2, sizeof(overlong2)));
	TEST_ASSERT_FALSE(wss_utf8_valid(overlong3, sizeof(overlong3)));
	TEST_ASSERT_FALSE(wss_utf8_valid(overlong4, sizeof(overlong4)));
	TEST_ASSERT_FALSE(wss_utf8_valid(surrogate, sizeof(surrogate)));
	TEST_ASSERT_FALSE(wss_utf8_valid(too_big, sizeof(too_big)));
	TEST_ASSERT_FALSE(wss_utf8_valid(lone_cont, sizeof(lone_cont)));
	TEST_ASSERT_FALSE(wss_utf8_valid(five_byte, sizeof(five_byte)));
	TEST_ASSERT_FALSE(wss_utf8_valid(truncated, sizeof(truncated)));
	TEST_ASSERT_FALSE(wss_utf8_valid(bad_cont, sizeof(bad_cont)));
}

/* ------------------------------------------------------------------------- */
/* header codec                                                              */
/* ------------------------------------------------------------------------- */

static void test_hdr_parse(void)
{
	wss_hdr_t h;
	uint8_t b[WSS_HDR_MAX];

	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_hdr_parse(NULL, 2U, &h));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_hdr_parse(b, 2U, NULL));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, wss_hdr_parse(b, 1U, &h));

	/* Unmasked TEXT, 5-byte payload. */
	b[0] = 0x81U;
	b[1] = 0x05U;
	TEST_ASSERT_EQUAL_INT(0, wss_hdr_parse(b, 2U, &h));
	TEST_ASSERT_TRUE(h.fin);
	TEST_ASSERT_FALSE(h.masked);
	TEST_ASSERT_EQUAL_UINT(WSS_OP_TEXT, h.op);
	TEST_ASSERT_EQUAL_UINT64(5U, h.len);
	TEST_ASSERT_EQUAL_UINT(2U, h.hdr_len);

	/* Masked, 125-byte payload: the largest 7-bit length. */
	b[1] = 0x80U | 125U;
	b[2] = 0x01U;
	b[3] = 0x02U;
	b[4] = 0x03U;
	b[5] = 0x04U;
	TEST_ASSERT_EQUAL_INT(-EAGAIN, wss_hdr_parse(b, 5U, &h));
	TEST_ASSERT_EQUAL_INT(0, wss_hdr_parse(b, 6U, &h));
	TEST_ASSERT_TRUE(h.masked);
	TEST_ASSERT_EQUAL_UINT(6U, h.hdr_len);
	TEST_ASSERT_EQUAL_UINT64(125U, h.len);
	TEST_ASSERT_EQUAL_HEX8(0x01U, h.mask[0]);
	TEST_ASSERT_EQUAL_HEX8(0x04U, h.mask[3]);

	/* 16-bit length. */
	b[1] = 126U;
	b[2] = 0x01U;
	b[3] = 0x00U;
	TEST_ASSERT_EQUAL_INT(-EAGAIN, wss_hdr_parse(b, 3U, &h));
	TEST_ASSERT_EQUAL_INT(0, wss_hdr_parse(b, 4U, &h));
	TEST_ASSERT_EQUAL_UINT64(256U, h.len);
	TEST_ASSERT_EQUAL_UINT(4U, h.hdr_len);

	/* 16-bit length below 126: not the shortest form. */
	b[2] = 0x00U;
	b[3] = 0x7DU;
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_hdr_parse(b, 4U, &h));

	/* 64-bit length: the 8 length octets are b[2]..b[9], big-endian, so
	 * 65536 == 0x0000000000010000 puts the 0x01 at b[7]. */
	memset(b, 0, sizeof(b));
	b[0] = 0x82U;
	b[1] = 127U;
	b[7] = 0x01U;
	TEST_ASSERT_EQUAL_INT(-EAGAIN, wss_hdr_parse(b, 9U, &h));
	TEST_ASSERT_EQUAL_INT(0, wss_hdr_parse(b, 10U, &h));
	TEST_ASSERT_EQUAL_UINT64(65536U, h.len);
	TEST_ASSERT_EQUAL_UINT(10U, h.hdr_len);

	/* 64-bit length that fits in 16 bits: not the shortest form. */
	memset(&b[2], 0, 8U);
	b[8] = 0xFFU;
	b[9] = 0xFFU;
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_hdr_parse(b, 10U, &h));

	/* 64-bit length with the MSB set. */
	memset(&b[2], 0, 8U);
	b[2] = 0x80U;
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE, wss_hdr_parse(b, 10U, &h));

	/* RSV bits, each in turn. */
	b[0] = 0xC1U;
	b[1] = 0x00U;
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_hdr_parse(b, 2U, &h));
	b[0] = 0xA1U;
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_hdr_parse(b, 2U, &h));
	b[0] = 0x91U;
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_hdr_parse(b, 2U, &h));

	/* Reserved opcodes. */
	b[0] = 0x83U;
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_hdr_parse(b, 2U, &h));
	b[0] = 0x8BU;
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_hdr_parse(b, 2U, &h));

	/* A fragmented control frame. */
	b[0] = 0x09U; /* PING without FIN */
	b[1] = 0x00U;
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_hdr_parse(b, 2U, &h));

	/* An over-long control frame. */
	b[0] = 0x89U;
	b[1] = 126U;
	b[2] = 0x00U;
	b[3] = 0x7EU;
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_hdr_parse(b, 4U, &h));

	/* Mask bytes not yet available. */
	b[0] = 0x81U;
	b[1] = 0x80U | 1U;
	TEST_ASSERT_EQUAL_INT(-EAGAIN, wss_hdr_parse(b, 4U, &h));
}

static void test_unmask(void)
{
	uint8_t buf[8];
	static const uint8_t mask[4] = { 0x11U, 0x22U, 0x33U, 0x44U };
	size_t i;

	memset(buf, 0, sizeof(buf));
	wss_unmask(buf, sizeof(buf), mask, 0U);
	for (i = 0U; i < sizeof(buf); i++) {
		TEST_ASSERT_EQUAL_HEX8(mask[i & 3U], buf[i]);
	}
	/* An offset keeps the key in phase across a split payload. */
	memset(buf, 0, sizeof(buf));
	wss_unmask(buf, 4U, mask, 2U);
	TEST_ASSERT_EQUAL_HEX8(mask[2], buf[0]);
	TEST_ASSERT_EQUAL_HEX8(mask[3], buf[1]);
	TEST_ASSERT_EQUAL_HEX8(mask[0], buf[2]);
	wss_unmask(NULL, 4U, mask, 0U);
	wss_unmask(buf, 4U, NULL, 0U);
}

static void test_encode(void)
{
	static uint8_t out[70100];
	static uint8_t payload[70000];
	int n;

	memset(payload, 'x', sizeof(payload));

	/* 2-byte header. */
	n = wss_encode((uint8_t)WSS_OP_TEXT, true, (const uint8_t *)"hi", 2U, out,
		       sizeof(out));
	TEST_ASSERT_EQUAL_INT(4, n);
	TEST_ASSERT_EQUAL_HEX8(0x81U, out[0]);
	TEST_ASSERT_EQUAL_HEX8(0x02U, out[1]);
	TEST_ASSERT_EQUAL_MEMORY("hi", &out[2], 2U);

	/* 4-byte header. */
	n = wss_encode((uint8_t)WSS_OP_BIN, true, payload, 200U, out,
		       sizeof(out));
	TEST_ASSERT_EQUAL_INT(204, n);
	TEST_ASSERT_EQUAL_HEX8(0x82U, out[0]);
	TEST_ASSERT_EQUAL_HEX8(126U, out[1]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, out[2]);
	TEST_ASSERT_EQUAL_HEX8(0xC8U, out[3]);

	/* 10-byte header. */
	n = wss_encode((uint8_t)WSS_OP_BIN, true, payload, 70000U, out,
		       sizeof(out));
	TEST_ASSERT_EQUAL_INT(70010, n);
	TEST_ASSERT_EQUAL_HEX8(127U, out[1]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, out[2]);
	TEST_ASSERT_EQUAL_HEX8(0x01U, out[7]);
	TEST_ASSERT_EQUAL_HEX8(0x11U, out[8]);
	TEST_ASSERT_EQUAL_HEX8(0x70U, out[9]);

	/* A non-final fragment. */
	n = wss_encode((uint8_t)WSS_OP_CONT, false, payload, 1U, out,
		       sizeof(out));
	TEST_ASSERT_EQUAL_INT(3, n);
	TEST_ASSERT_EQUAL_HEX8(0x00U, out[0]);

	/* An empty frame. */
	n = wss_encode((uint8_t)WSS_OP_PONG, true, NULL, 0U, out, sizeof(out));
	TEST_ASSERT_EQUAL_INT(2, n);

	/* The header-only form, used for payloads too big to stage. */
	{
		uint8_t h[WSS_HDR_MAX];

		TEST_ASSERT_EQUAL_INT(2, wss_encode_header((uint8_t)WSS_OP_TEXT,
							   true, 10U, h,
							   sizeof(h)));
		TEST_ASSERT_EQUAL_HEX8(0x81U, h[0]);
		TEST_ASSERT_EQUAL_HEX8(10U, h[1]);
		TEST_ASSERT_EQUAL_INT(4, wss_encode_header((uint8_t)WSS_OP_TEXT,
							   true, 6000U, h,
							   sizeof(h)));
		TEST_ASSERT_EQUAL_HEX8(126U, h[1]);
		TEST_ASSERT_EQUAL_INT(10, wss_encode_header((uint8_t)WSS_OP_BIN,
							    true, 70000U, h,
							    sizeof(h)));
		TEST_ASSERT_EQUAL_HEX8(127U, h[1]);
		TEST_ASSERT_EQUAL_INT(-ENOSPC,
				      wss_encode_header((uint8_t)WSS_OP_TEXT,
							true, 6000U, h, 3U));
		TEST_ASSERT_EQUAL_INT(-EINVAL,
				      wss_encode_header((uint8_t)WSS_OP_TEXT,
							true, 10U, NULL,
							sizeof(h)));
		TEST_ASSERT_EQUAL_INT(-EINVAL, wss_encode_header(0x03U, true, 0U,
								 h, sizeof(h)));
		TEST_ASSERT_EQUAL_INT(-EINVAL,
				      wss_encode_header((uint8_t)WSS_OP_PING,
							true, 200U, h,
							sizeof(h)));
	}

	/* Errors. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_encode(0x03U, true, NULL, 0U, out,
						  sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_encode((uint8_t)WSS_OP_TEXT, true,
						  NULL, 4U, out, sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_encode((uint8_t)WSS_OP_TEXT, true,
						  payload, 4U, NULL,
						  sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_encode((uint8_t)WSS_OP_PING, true,
						  payload, 126U, out,
						  sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_encode((uint8_t)WSS_OP_PING, false,
						  payload, 4U, out,
						  sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, wss_encode((uint8_t)WSS_OP_TEXT, true,
						  payload, 10U, out, 5U));
}

static void test_encode_close(void)
{
	uint8_t out[160];
	int n;
	char long_reason[200];

	n = wss_encode_close(WSS_CLOSE_NORMAL, "bye", out, sizeof(out));
	TEST_ASSERT_EQUAL_INT(7, n);
	TEST_ASSERT_EQUAL_HEX8(0x88U, out[0]);
	TEST_ASSERT_EQUAL_HEX8(5U, out[1]);
	TEST_ASSERT_EQUAL_HEX8(0x03U, out[2]);
	TEST_ASSERT_EQUAL_HEX8(0xE8U, out[3]);
	TEST_ASSERT_EQUAL_MEMORY("bye", &out[4], 3U);

	/* No status code at all. */
	n = wss_encode_close(0U, "ignored", out, sizeof(out));
	TEST_ASSERT_EQUAL_INT(2, n);
	TEST_ASSERT_EQUAL_HEX8(0x00U, out[1]);

	/* NULL reason. */
	n = wss_encode_close(WSS_CLOSE_POLICY, NULL, out, sizeof(out));
	TEST_ASSERT_EQUAL_INT(4, n);

	/* A private-use code is allowed. */
	TEST_ASSERT_TRUE(wss_encode_close(4000U, NULL, out, sizeof(out)) > 0);
	TEST_ASSERT_TRUE(wss_encode_close(1010U, NULL, out, sizeof(out)) > 0);
	TEST_ASSERT_TRUE(wss_encode_close(WSS_CLOSE_TOO_BIG, NULL, out,
					  sizeof(out)) > 0);

	/* Reserved codes must never go on the wire. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_encode_close(1005U, NULL, out,
							sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_encode_close(1006U, NULL, out,
							sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_encode_close(999U, NULL, out,
							sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_encode_close(5000U, NULL, out,
							sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_encode_close(1000U, NULL, NULL,
							sizeof(out)));

	/* A long reason is truncated to fit a control frame. */
	memset(long_reason, 'r', sizeof(long_reason) - 1U);
	long_reason[sizeof(long_reason) - 1U] = '\0';
	n = wss_encode_close(WSS_CLOSE_NORMAL, long_reason, out, sizeof(out));
	TEST_ASSERT_EQUAL_INT(2 + (int)WSS_CONTROL_MAX, n);
	TEST_ASSERT_EQUAL_HEX8((uint8_t)WSS_CONTROL_MAX, out[1]);
}

/* ------------------------------------------------------------------------- */
/* reassembly                                                                */
/* ------------------------------------------------------------------------- */

/* Build a masked client frame into @p out; returns its total length. */
static size_t mk_frame(uint8_t *out, uint8_t op, bool fin, const void *payload,
		       size_t len)
{
	static const uint8_t mask[4] = { 0xA1U, 0xB2U, 0xC3U, 0xD4U };
	size_t o = 0U;
	size_t i;

	out[o++] = (uint8_t)((fin ? 0x80U : 0x00U) | (op & 0x0FU));
	if (len < 126U) {
		out[o++] = (uint8_t)(0x80U | len);
	} else if (len <= 0xFFFFU) {
		out[o++] = 0x80U | 126U;
		out[o++] = (uint8_t)((len >> 8) & 0xFFU);
		out[o++] = (uint8_t)(len & 0xFFU);
	} else {
		unsigned int k;

		out[o++] = 0x80U | 127U;
		for (k = 0U; k < 8U; k++) {
			out[o++] = (uint8_t)(((uint64_t)len >>
					      (56U - (8U * k))) & 0xFFU);
		}
	}
	memcpy(&out[o], mask, 4U);
	o += 4U;
	for (i = 0U; i < len; i++) {
		out[o + i] = (uint8_t)(((const uint8_t *)payload)[i] ^
				       mask[i & 3U]);
	}
	return o + len;
}

static void test_rx_single_text(void)
{
	wss_rx_t rx;
	uint8_t msg[128];
	uint8_t frame[128];
	size_t n;
	size_t consumed = 0U;
	uint8_t op = 0U;
	const uint8_t *p = NULL;
	size_t len = 0U;
	uint16_t code = 0U;

	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_rx_init(NULL, msg, sizeof(msg)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_rx_init(&rx, NULL, sizeof(msg)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_rx_init(&rx, msg, 0U));
	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));

	n = mk_frame(frame, (uint8_t)WSS_OP_TEXT, true, "hello", 5U);
	TEST_ASSERT_EQUAL_INT(1, wss_rx_feed(&rx, frame, n, &consumed, &op, &p,
					     &len, &code));
	TEST_ASSERT_EQUAL_UINT(n, consumed);
	TEST_ASSERT_EQUAL_UINT(WSS_OP_TEXT, op);
	TEST_ASSERT_EQUAL_UINT(5U, len);
	TEST_ASSERT_EQUAL_MEMORY("hello", p, 5U);
	TEST_ASSERT_EQUAL_UINT32(1U, rx.messages);
	TEST_ASSERT_EQUAL_UINT32(1U, rx.frames);

	/* A partial frame consumes nothing and asks for more. */
	n = mk_frame(frame, (uint8_t)WSS_OP_TEXT, true, "hello", 5U);
	TEST_ASSERT_EQUAL_INT(0, wss_rx_feed(&rx, frame, n - 2U, &consumed, &op,
					     &p, &len, &code));
	TEST_ASSERT_EQUAL_UINT(0U, consumed);

	/* A truncated header likewise. */
	TEST_ASSERT_EQUAL_INT(0, wss_rx_feed(&rx, frame, 1U, &consumed, &op, &p,
					     &len, &code));
	TEST_ASSERT_EQUAL_UINT(0U, consumed);

	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_rx_feed(NULL, frame, n, &consumed,
						   &op, &p, &len, &code));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_rx_feed(&rx, NULL, 4U, &consumed, &op,
						   &p, &len, &code));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_rx_feed(&rx, frame, n, &consumed,
						   NULL, &p, &len, &code));
	/* A zero-length feed is a no-op. */
	TEST_ASSERT_EQUAL_INT(0, wss_rx_feed(&rx, NULL, 0U, &consumed, &op, &p,
					     &len, &code));
}

static void test_rx_fragmented(void)
{
	wss_rx_t rx;
	uint8_t msg[128];
	uint8_t buf[256];
	size_t o = 0U;
	size_t consumed = 0U;
	uint8_t op = 0U;
	const uint8_t *p = NULL;
	size_t len = 0U;
	uint16_t code = 0U;

	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
	o += mk_frame(&buf[o], (uint8_t)WSS_OP_TEXT, false, "ab", 2U);
	o += mk_frame(&buf[o], (uint8_t)WSS_OP_CONT, false, "cd", 2U);
	o += mk_frame(&buf[o], (uint8_t)WSS_OP_CONT, true, "ef", 2U);

	TEST_ASSERT_EQUAL_INT(1, wss_rx_feed(&rx, buf, o, &consumed, &op, &p,
					     &len, &code));
	TEST_ASSERT_EQUAL_UINT(o, consumed);
	TEST_ASSERT_EQUAL_UINT(WSS_OP_TEXT, op);
	TEST_ASSERT_EQUAL_UINT(6U, len);
	TEST_ASSERT_EQUAL_MEMORY("abcdef", p, 6U);

	/* A control frame interleaved inside a fragmented message is legal and
	 * must be delivered without disturbing the reassembly. */
	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
	o = mk_frame(buf, (uint8_t)WSS_OP_TEXT, false, "ab", 2U);
	TEST_ASSERT_EQUAL_INT(0, wss_rx_feed(&rx, buf, o, &consumed, &op, &p,
					     &len, &code));
	o = mk_frame(buf, (uint8_t)WSS_OP_PING, true, "pi", 2U);
	TEST_ASSERT_EQUAL_INT(1, wss_rx_feed(&rx, buf, o, &consumed, &op, &p,
					     &len, &code));
	TEST_ASSERT_EQUAL_UINT(WSS_OP_PING, op);
	TEST_ASSERT_EQUAL_MEMORY("pi", p, 2U);
	o = mk_frame(buf, (uint8_t)WSS_OP_CONT, true, "cd", 2U);
	TEST_ASSERT_EQUAL_INT(1, wss_rx_feed(&rx, buf, o, &consumed, &op, &p,
					     &len, &code));
	TEST_ASSERT_EQUAL_UINT(WSS_OP_TEXT, op);
	TEST_ASSERT_EQUAL_UINT(4U, len);
	TEST_ASSERT_EQUAL_MEMORY("abcd", p, 4U);
	TEST_ASSERT_EQUAL_UINT32(1U, rx.pings);
}

static void test_rx_protocol_errors(void)
{
	wss_rx_t rx;
	uint8_t msg[16];
	uint8_t buf[256];
	size_t o;
	size_t consumed = 0U;
	uint8_t op = 0U;
	const uint8_t *p = NULL;
	size_t len = 0U;
	uint16_t code = 0U;

	/* An unmasked client frame. */
	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
	buf[0] = 0x81U;
	buf[1] = 0x02U;
	buf[2] = 'h';
	buf[3] = 'i';
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_rx_feed(&rx, buf, 4U, &consumed, &op,
						   &p, &len, &code));
	TEST_ASSERT_EQUAL_UINT16(WSS_CLOSE_PROTOCOL_ERROR, code);
	TEST_ASSERT_EQUAL_UINT32(1U, rx.protocol_errors);

	/* Continuation with nothing in progress. */
	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
	o = mk_frame(buf, (uint8_t)WSS_OP_CONT, true, "x", 1U);
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_rx_feed(&rx, buf, o, &consumed, &op,
						   &p, &len, &code));

	/* A new data frame while a message is open. */
	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
	o = mk_frame(buf, (uint8_t)WSS_OP_TEXT, false, "a", 1U);
	TEST_ASSERT_EQUAL_INT(0, wss_rx_feed(&rx, buf, o, &consumed, &op, &p,
					     &len, &code));
	o = mk_frame(buf, (uint8_t)WSS_OP_TEXT, true, "b", 1U);
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_rx_feed(&rx, buf, o, &consumed, &op,
						   &p, &len, &code));

	/* Invalid UTF-8 in a TEXT message. */
	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
	o = mk_frame(buf, (uint8_t)WSS_OP_TEXT, true, "\xC0\xAF", 2U);
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_rx_feed(&rx, buf, o, &consumed, &op,
						   &p, &len, &code));
	TEST_ASSERT_EQUAL_UINT16(WSS_CLOSE_INVALID_DATA, code);

	/* A code point split across two fragments is still valid: the check
	 * must run on the reassembled message, not per frame. */
	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
	o = mk_frame(buf, (uint8_t)WSS_OP_TEXT, false, "\xE2", 1U);
	TEST_ASSERT_EQUAL_INT(0, wss_rx_feed(&rx, buf, o, &consumed, &op, &p,
					     &len, &code));
	o = mk_frame(buf, (uint8_t)WSS_OP_CONT, true, "\x82\xAC", 2U);
	TEST_ASSERT_EQUAL_INT(1, wss_rx_feed(&rx, buf, o, &consumed, &op, &p,
					     &len, &code));
	TEST_ASSERT_EQUAL_UINT(3U, len);

	/* Binary is not UTF-8 checked. */
	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
	o = mk_frame(buf, (uint8_t)WSS_OP_BIN, true, "\xC0\xAF", 2U);
	TEST_ASSERT_EQUAL_INT(1, wss_rx_feed(&rx, buf, o, &consumed, &op, &p,
					     &len, &code));

	/* A message bigger than the reassembly buffer. */
	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
	{
		char big[64];

		memset(big, 'z', sizeof(big));
		o = mk_frame(buf, (uint8_t)WSS_OP_BIN, true, big, sizeof(big));
	}
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE, wss_rx_feed(&rx, buf, o, &consumed, &op,
						     &p, &len, &code));
	TEST_ASSERT_EQUAL_UINT16(WSS_CLOSE_TOO_BIG, code);
	TEST_ASSERT_EQUAL_UINT32(1U, rx.oversize);

	/* A declared 64-bit length with the MSB set. */
	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
	memset(buf, 0, 16U);
	buf[0] = 0x82U;
	buf[1] = 0x80U | 127U;
	buf[2] = 0x80U;
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_rx_feed(&rx, buf, 14U, &consumed, &op,
						   &p, &len, &code));
	TEST_ASSERT_EQUAL_UINT16(WSS_CLOSE_TOO_BIG, code);

	/* A header-level protocol error (RSV set). */
	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
	buf[0] = 0xC1U;
	buf[1] = 0x80U;
	memset(&buf[2], 0, 4U);
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_rx_feed(&rx, buf, 6U, &consumed, &op,
						   &p, &len, &code));
}

static void test_rx_close_frames(void)
{
	wss_rx_t rx;
	uint8_t msg[64];
	uint8_t buf[64];
	uint8_t body[8];
	size_t o;
	size_t consumed = 0U;
	uint8_t op = 0U;
	const uint8_t *p = NULL;
	size_t len = 0U;
	uint16_t code = 0U;

	/* Empty close. */
	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
	o = mk_frame(buf, (uint8_t)WSS_OP_CLOSE, true, NULL, 0U);
	TEST_ASSERT_EQUAL_INT(1, wss_rx_feed(&rx, buf, o, &consumed, &op, &p,
					     &len, &code));
	TEST_ASSERT_EQUAL_UINT(WSS_OP_CLOSE, op);
	TEST_ASSERT_EQUAL_UINT(0U, len);

	/* Close with a valid code and reason. */
	body[0] = 0x03U;
	body[1] = 0xE8U;
	body[2] = 'o';
	body[3] = 'k';
	o = mk_frame(buf, (uint8_t)WSS_OP_CLOSE, true, body, 4U);
	TEST_ASSERT_EQUAL_INT(1, wss_rx_feed(&rx, buf, o, &consumed, &op, &p,
					     &len, &code));

	/* A one-byte close payload is a truncated status code. */
	o = mk_frame(buf, (uint8_t)WSS_OP_CLOSE, true, body, 1U);
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_rx_feed(&rx, buf, o, &consumed, &op,
						   &p, &len, &code));

	/* A reserved close code. */
	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
	body[0] = 0x03U;
	body[1] = 0xEDU; /* 1005 */
	o = mk_frame(buf, (uint8_t)WSS_OP_CLOSE, true, body, 2U);
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_rx_feed(&rx, buf, o, &consumed, &op,
						   &p, &len, &code));

	/* A close reason that is not UTF-8. */
	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
	body[0] = 0x03U;
	body[1] = 0xE8U;
	body[2] = 0xC0U;
	body[3] = 0xAFU;
	o = mk_frame(buf, (uint8_t)WSS_OP_CLOSE, true, body, 4U);
	TEST_ASSERT_EQUAL_INT(-EPROTO, wss_rx_feed(&rx, buf, o, &consumed, &op,
						   &p, &len, &code));

	/* Pong is counted. */
	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
	o = mk_frame(buf, (uint8_t)WSS_OP_PONG, true, NULL, 0U);
	TEST_ASSERT_EQUAL_INT(1, wss_rx_feed(&rx, buf, o, &consumed, &op, &p,
					     &len, &code));
	TEST_ASSERT_EQUAL_UINT32(1U, rx.pongs);
}

static void test_rx_16bit_and_zero_length(void)
{
	wss_rx_t rx;
	static uint8_t msg[1024];
	static uint8_t buf[1024];
	static char payload[300];
	size_t o;
	size_t consumed = 0U;
	uint8_t op = 0U;
	const uint8_t *p = NULL;
	size_t len = 0U;
	uint16_t code = 0U;

	memset(payload, 'q', sizeof(payload));
	TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
	o = mk_frame(buf, (uint8_t)WSS_OP_BIN, true, payload, sizeof(payload));
	TEST_ASSERT_EQUAL_INT(1, wss_rx_feed(&rx, buf, o, &consumed, &op, &p,
					     &len, &code));
	TEST_ASSERT_EQUAL_UINT(sizeof(payload), len);

	/* A zero-length TEXT message is legal. */
	o = mk_frame(buf, (uint8_t)WSS_OP_TEXT, true, NULL, 0U);
	TEST_ASSERT_EQUAL_INT(1, wss_rx_feed(&rx, buf, o, &consumed, &op, &p,
					     &len, &code));
	TEST_ASSERT_EQUAL_UINT(0U, len);
}

/* ------------------------------------------------------------------------- */
/* send queue                                                                */
/* ------------------------------------------------------------------------- */

static void test_txq(void)
{
	static wss_txq_t q;
	uint8_t frame[16];
	const uint8_t *p = NULL;
	size_t n = 0U;
	unsigned int i;

	wss_txq_init(NULL);
	wss_txq_init(&q);
	TEST_ASSERT_EQUAL_UINT(0U, wss_txq_count(&q));
	TEST_ASSERT_EQUAL_UINT(0U, wss_txq_count(NULL));
	TEST_ASSERT_EQUAL_INT(-ENOENT, wss_txq_peek(&q, &p, &n));
	TEST_ASSERT_EQUAL_INT(-ENOENT, wss_txq_pop(&q));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_txq_peek(NULL, &p, &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_txq_peek(&q, NULL, &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_txq_pop(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_txq_push(NULL, frame, 4U, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_txq_push(&q, NULL, 4U, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_txq_push(&q, frame, 0U, false));
	{
		uint8_t big[WSS_TXQ_SLOT_BYTES + 4U];

		TEST_ASSERT_EQUAL_INT(-ENOSPC,
				      wss_txq_push(&q, big,
						   WSS_TXQ_SLOT_BYTES + 1U,
						   false));
	}

	/* FIFO order. */
	for (i = 0U; i < WSS_TXQ_SLOTS; i++) {
		frame[0] = (uint8_t)i;
		TEST_ASSERT_EQUAL_INT(0, wss_txq_push(&q, frame, 1U, false));
	}
	TEST_ASSERT_EQUAL_UINT(WSS_TXQ_SLOTS, wss_txq_count(&q));
	for (i = 0U; i < WSS_TXQ_SLOTS; i++) {
		TEST_ASSERT_EQUAL_INT(0, wss_txq_peek(&q, &p, &n));
		TEST_ASSERT_EQUAL_UINT(1U, n);
		TEST_ASSERT_EQUAL_HEX8((uint8_t)i, p[0]);
		TEST_ASSERT_EQUAL_INT(0, wss_txq_pop(&q));
	}
	TEST_ASSERT_EQUAL_UINT(0U, wss_txq_count(&q));
	TEST_ASSERT_EQUAL_UINT32(WSS_TXQ_SLOTS, q.sent);

	/* A full queue drops the oldest. */
	for (i = 0U; i < WSS_TXQ_SLOTS; i++) {
		frame[0] = (uint8_t)i;
		TEST_ASSERT_EQUAL_INT(0, wss_txq_push(&q, frame, 1U, false));
	}
	frame[0] = 0xEEU;
	TEST_ASSERT_EQUAL_INT(1, wss_txq_push(&q, frame, 1U, false));
	TEST_ASSERT_EQUAL_UINT32(1U, q.dropped);
	TEST_ASSERT_EQUAL_INT(0, wss_txq_peek(&q, &p, &n));
	TEST_ASSERT_EQUAL_HEX8(1U, p[0]); /* entry 0 was dropped */

	/* An urgent push jumps the queue. */
	frame[0] = 0x99U;
	TEST_ASSERT_EQUAL_INT(1, wss_txq_push(&q, frame, 1U, true));
	TEST_ASSERT_EQUAL_INT(0, wss_txq_peek(&q, &p, &n));
	TEST_ASSERT_EQUAL_HEX8(0x99U, p[0]);

	/* An urgent push into a queue with room does not drop anything. */
	wss_txq_init(&q);
	frame[0] = 0x01U;
	TEST_ASSERT_EQUAL_INT(0, wss_txq_push(&q, frame, 1U, false));
	frame[0] = 0x02U;
	TEST_ASSERT_EQUAL_INT(0, wss_txq_push(&q, frame, 1U, true));
	TEST_ASSERT_EQUAL_UINT(2U, wss_txq_count(&q));
	TEST_ASSERT_EQUAL_INT(0, wss_txq_peek(&q, &p, &n));
	TEST_ASSERT_EQUAL_HEX8(0x02U, p[0]);
	TEST_ASSERT_EQUAL_INT(0, wss_txq_pop(&q));
	TEST_ASSERT_EQUAL_INT(0, wss_txq_peek(&q, &p, &n));
	TEST_ASSERT_EQUAL_HEX8(0x01U, p[0]);
}

/* ------------------------------------------------------------------------- */
/* subscriptions                                                             */
/* ------------------------------------------------------------------------- */

static void test_groups(void)
{
	TEST_ASSERT_EQUAL_UINT(WSS_GRP_SUMMARY, wss_group_bit("summary", 7U));
	TEST_ASSERT_EQUAL_UINT(WSS_GRP_LOGS, wss_group_bit("logs", 4U));
	TEST_ASSERT_EQUAL_UINT(0U, wss_group_bit("nope", 4U));
	TEST_ASSERT_EQUAL_UINT(0U, wss_group_bit(NULL, 4U));
	TEST_ASSERT_EQUAL_UINT(0U, wss_group_bit("summary", 0U));
	TEST_ASSERT_EQUAL_STRING("power", wss_group_name(WSS_GRP_POWER));
	TEST_ASSERT_NULL(wss_group_name(0U));
	TEST_ASSERT_NULL(wss_group_name(0x03U));
}

static void test_sub_apply(void)
{
	wss_sub_t s;
	static const char *const sub_all =
		"{\"op\":\"subscribe\",\"groups\":[\"all\"],\"rate\":4}";
	static const char *const sub_two =
		"{\"op\":\"subscribe\",\"groups\":[\"timing\",\"power\","
		"\"bogus\",7],\"rate\":9,\"log_cursor\":42}";
	static const char *const unsub =
		"{\"op\":\"unsubscribe\",\"groups\":[\"power\"]}";

	wss_sub_init(NULL);
	wss_sub_init(&s);
	TEST_ASSERT_EQUAL_UINT(WSS_GRP_SUMMARY | WSS_GRP_ALARMS, s.groups);
	TEST_ASSERT_EQUAL_UINT(WSS_RATE_HZ_DEFAULT, s.rate_hz);

	TEST_ASSERT_EQUAL_INT(0, wss_sub_apply(&s, sub_all, strlen(sub_all)));
	TEST_ASSERT_EQUAL_UINT(WSS_GRP_ALL, s.groups);
	TEST_ASSERT_EQUAL_UINT(WSS_RATE_HZ_MAX, s.rate_hz);

	TEST_ASSERT_EQUAL_INT(0, wss_sub_apply(&s, sub_two, strlen(sub_two)));
	TEST_ASSERT_EQUAL_UINT(WSS_GRP_TIMING | WSS_GRP_POWER, s.groups);
	TEST_ASSERT_EQUAL_UINT(WSS_RATE_HZ_MAX, s.rate_hz); /* clamped from 9 */
	TEST_ASSERT_EQUAL_UINT32(42U, s.log_cursor);

	TEST_ASSERT_EQUAL_INT(0, wss_sub_apply(&s, unsub, strlen(unsub)));
	TEST_ASSERT_EQUAL_UINT(WSS_GRP_TIMING, s.groups);

	/* rate 0 becomes 1, and no groups given keeps the existing set. */
	{
		static const char *const r0 =
			"{\"op\":\"subscribe\",\"rate\":0}";

		TEST_ASSERT_EQUAL_INT(0, wss_sub_apply(&s, r0, strlen(r0)));
		TEST_ASSERT_EQUAL_UINT(1U, s.rate_hz);
		TEST_ASSERT_EQUAL_UINT(WSS_GRP_TIMING, s.groups);
	}

	/* A ping is accepted and changes nothing. */
	{
		static const char *const ping = "{\"op\":\"ping\"}";

		TEST_ASSERT_EQUAL_INT(0, wss_sub_apply(&s, ping, strlen(ping)));
		TEST_ASSERT_EQUAL_UINT(WSS_GRP_TIMING, s.groups);
	}

	/* A non-array "groups" is ignored, not fatal. */
	{
		static const char *const bad =
			"{\"op\":\"subscribe\",\"groups\":\"timing\"}";

		TEST_ASSERT_EQUAL_INT(0, wss_sub_apply(&s, bad, strlen(bad)));
	}

	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_sub_apply(NULL, "{}", 2U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, wss_sub_apply(&s, NULL, 2U));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, wss_sub_apply(&s, "{}", 2U));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, wss_sub_apply(&s, "not json", 8U));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      wss_sub_apply(&s, "{\"op\":\"nope\"}", 13U));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, wss_sub_apply(&s, "{\"op\":7}", 8U));
}

/* ------------------------------------------------------------------------- */
/* fuzz                                                                      */
/* ------------------------------------------------------------------------- */

static void test_fuzz_frames(void)
{
	test_rng_t g;
	static uint8_t buf[512];
	static uint8_t msg[256];
	unsigned int iter;

	test_rng_init(&g, 0x11223344U);
	for (iter = 0U; iter < 60000U; iter++) {
		wss_rx_t rx;
		size_t len = 1U + test_rng_below(&g, sizeof(buf) - 1U);
		size_t consumed = 0U;
		uint8_t op = 0U;
		const uint8_t *p = NULL;
		size_t plen = 0U;
		uint16_t code = 0U;
		int rc;

		test_rng_fill(&g, buf, len);
		/*
		 * Half the iterations start from a well-formed masked frame and
		 * corrupt one byte, so the decoder reaches its payload and
		 * fragmentation paths instead of dying on byte 0.
		 */
		if ((iter & 1U) != 0U) {
			static const uint8_t ops[] = {
				(uint8_t)WSS_OP_TEXT, (uint8_t)WSS_OP_BIN,
				(uint8_t)WSS_OP_CONT, (uint8_t)WSS_OP_PING,
				(uint8_t)WSS_OP_CLOSE,
			};
			char pl[64];
			size_t pn = test_rng_below(&g, sizeof(pl));
			size_t at;

			test_rng_fill(&g, (uint8_t *)pl, pn);
			len = mk_frame(buf, ops[test_rng_below(&g, sizeof(ops))],
				       (test_rng_u32(&g) & 1U) != 0U, pl, pn);
			at = test_rng_below(&g, len);
			buf[at] = (uint8_t)(test_rng_u32(&g) & 0xFFU);
		}

		TEST_ASSERT_EQUAL_INT(0, wss_rx_init(&rx, msg, sizeof(msg)));
		rc = wss_rx_feed(&rx, buf, len, &consumed, &op, &p, &plen,
				 &code);
		TEST_ASSERT_TRUE(rc == 0 || rc == 1 || rc == -EPROTO ||
				 rc == -EMSGSIZE);
		TEST_ASSERT_TRUE(consumed <= len);
		if (rc == 1) {
			/* The payload must lie inside one of the two buffers. */
			bool in_msg = (p >= msg) &&
				      ((p + plen) <= (msg + sizeof(msg)));
			bool in_buf = (p >= buf) && ((p + plen) <= (buf + len));

			TEST_ASSERT_TRUE(in_msg || in_buf);
			TEST_ASSERT_TRUE(plen <= sizeof(msg));
		}

		/* Also fuzz the standalone header parser. */
		{
			wss_hdr_t h;
			int hrc = wss_hdr_parse(buf, len, &h);

			TEST_ASSERT_TRUE(hrc == 0 || hrc == -EAGAIN ||
					 hrc == -EPROTO || hrc == -EMSGSIZE);
			if (hrc == 0) {
				TEST_ASSERT_TRUE(h.hdr_len <= WSS_HDR_MAX);
				TEST_ASSERT_TRUE((size_t)h.hdr_len <= len);
			}
		}
	}
}

static void test_fuzz_sub_json(void)
{
	test_rng_t g;
	char buf[192];
	unsigned int iter;

	test_rng_init(&g, 0x77U);
	for (iter = 0U; iter < 40000U; iter++) {
		wss_sub_t s;
		size_t len = 1U + test_rng_below(&g, sizeof(buf) - 1U);
		size_t i;
		int rc;

		for (i = 0U; i < len; i++) {
			static const char alpha[] =
				"{}[]\":,0123456789abcdefopsubcrieglmntwy \\\t";

			buf[i] = alpha[test_rng_below(&g, sizeof(alpha) - 1U)];
		}
		wss_sub_init(&s);
		rc = wss_sub_apply(&s, buf, len);
		TEST_ASSERT_TRUE(rc == 0 || rc == -EBADMSG);
		TEST_ASSERT_TRUE(s.rate_hz >= 1U && s.rate_hz <= WSS_RATE_HZ_MAX);
	}
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_sha1_vectors);
	RUN_TEST(test_accept_key_rfc6455);
	RUN_TEST(test_utf8);
	RUN_TEST(test_hdr_parse);
	RUN_TEST(test_unmask);
	RUN_TEST(test_encode);
	RUN_TEST(test_encode_close);

	RUN_TEST(test_rx_single_text);
	RUN_TEST(test_rx_fragmented);
	RUN_TEST(test_rx_protocol_errors);
	RUN_TEST(test_rx_close_frames);
	RUN_TEST(test_rx_16bit_and_zero_length);

	RUN_TEST(test_txq);
	RUN_TEST(test_groups);
	RUN_TEST(test_sub_apply);

	RUN_TEST(test_fuzz_frames);
	RUN_TEST(test_fuzz_sub_json);

	return UNITY_END();
}
