/*
 * STS1000 "Meridian" — core/auth TACACS+ codec unit tests.
 *
 * Provenance of the expectations (ARCHITECTURE.md §9):
 *
 *   - RFC 8907 publishes no test vectors, so the obfuscation keystream is pinned
 *     against an **independently recomputed** one: this file builds
 *     `MD5(session_id ‖ key ‖ version ‖ seq_no)` and the chained follow-on
 *     blocks with the local MD5 in a second, differently-shaped implementation
 *     (a plain loop here versus the block walk in tacacs.c) and requires the two
 *     to agree byte for byte. The MD5 underneath is separately pinned to the
 *     RFC 1321 suite in test_radius.c, so a wrong keystream cannot come from a
 *     wrong hash.
 *
 *   - Header and body layouts are asserted field by field against RFC 8907 §4.1
 *     (12-octet header, big-endian session id and length), §5.1 (authentication
 *     START), §5.2 (REPLY), §5.3 (CONTINUE) and §6.1/§6.2 (authorization
 *     REQUEST/RESPONSE), as literal offsets — not by round-tripping the encoder
 *     through its own decoder.
 *
 *   - The obfuscation is its own inverse, so the round-trip test asserts a real
 *     protocol property rather than an implementation coincidence.
 *
 *   - Every parser is fuzzed. A TACACS+ body carries four independent length
 *     fields that must exactly account for it; the property asserted is that a
 *     mismatch is always -EBADMSG and never a read past the buffer.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "auth/auth.h"
#include "auth/tacacs.h"
#include "test_support.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

static const char *const KEY = "tac-secret";
#define KEY_B ((const uint8_t *)KEY)
#define KEY_L (strlen(KEY))

/* ------------------------------------------------------------------------- */
/* header                                                                    */
/* ------------------------------------------------------------------------- */

static void test_hdr_layout(void)
{
	tacacs_hdr_t h;
	tacacs_hdr_t back;
	uint8_t buf[TACACS_HDR_LEN];

	memset(&h, 0, sizeof(h));
	h.version = TACACS_VER(TACACS_VER_MINOR_ONE);
	h.type = TACACS_TYPE_AUTHEN;
	h.seq_no = 1U;
	h.flags = TACACS_FLAG_SINGLE_CONNECT;
	h.session_id = 0x11223344U;
	h.length = 0x00000102U;

	TEST_ASSERT_EQUAL_INT(0, tacacs_hdr_build(buf, sizeof(buf), &h));
	TEST_ASSERT_EQUAL_HEX8(0xC1U, buf[0]);
	TEST_ASSERT_EQUAL_HEX8(0x01U, buf[1]);
	TEST_ASSERT_EQUAL_HEX8(0x01U, buf[2]);
	TEST_ASSERT_EQUAL_HEX8(0x04U, buf[3]);
	/* Session id and length are big-endian (RFC 8907 §4.1). */
	TEST_ASSERT_EQUAL_HEX8(0x11U, buf[4]);
	TEST_ASSERT_EQUAL_HEX8(0x22U, buf[5]);
	TEST_ASSERT_EQUAL_HEX8(0x33U, buf[6]);
	TEST_ASSERT_EQUAL_HEX8(0x44U, buf[7]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, buf[8]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, buf[9]);
	TEST_ASSERT_EQUAL_HEX8(0x01U, buf[10]);
	TEST_ASSERT_EQUAL_HEX8(0x02U, buf[11]);

	TEST_ASSERT_EQUAL_INT(0, tacacs_hdr_parse(buf, sizeof(buf), &back));
	TEST_ASSERT_EQUAL_HEX8(h.version, back.version);
	TEST_ASSERT_EQUAL_HEX8(h.type, back.type);
	TEST_ASSERT_EQUAL_HEX8(h.seq_no, back.seq_no);
	TEST_ASSERT_EQUAL_HEX8(h.flags, back.flags);
	TEST_ASSERT_EQUAL_HEX32(h.session_id, back.session_id);
	TEST_ASSERT_EQUAL_UINT32(h.length, back.length);
}

static void test_hdr_guards(void)
{
	tacacs_hdr_t h;
	uint8_t buf[TACACS_HDR_LEN];

	memset(&h, 0, sizeof(h));
	h.version = TACACS_VER(0U);
	TEST_ASSERT_EQUAL_INT(-EINVAL, tacacs_hdr_build(NULL, sizeof(buf), &h));
	TEST_ASSERT_EQUAL_INT(-EINVAL, tacacs_hdr_build(buf, sizeof(buf), NULL));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, tacacs_hdr_build(buf, 11U, &h));
	h.length = TACACS_BODY_MAX + 1U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, tacacs_hdr_build(buf, sizeof(buf), &h));

	TEST_ASSERT_EQUAL_INT(-EINVAL, tacacs_hdr_parse(NULL, 12U, &h));
	TEST_ASSERT_EQUAL_INT(-EINVAL, tacacs_hdr_parse(buf, 12U, NULL));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, tacacs_hdr_parse(buf, 11U, &h));

	/* A wrong major version is not TACACS+. */
	memset(buf, 0, sizeof(buf));
	buf[0] = 0xB0U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, tacacs_hdr_parse(buf, sizeof(buf), &h));
	/* And a body longer than we will ever accept. */
	buf[0] = 0xC0U;
	buf[8] = 0xFFU;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, tacacs_hdr_parse(buf, sizeof(buf), &h));
}

/* ------------------------------------------------------------------------- */
/* obfuscation                                                               */
/* ------------------------------------------------------------------------- */

/**
 * The RFC 8907 §4.5 keystream, recomputed here in a different shape:
 * one MD5 per block into a flat buffer, rather than tacacs.c's rolling walk.
 */
static void reference_pad(uint32_t session_id, const uint8_t *key,
			  size_t key_len, uint8_t version, uint8_t seq_no,
			  uint8_t *pad, size_t n)
{
	uint8_t sid[4];
	uint8_t prev[AUTH_MD5_LEN];
	size_t off = 0U;
	unsigned int block = 0U;

	sid[0] = (uint8_t)((session_id >> 24) & 0xFFU);
	sid[1] = (uint8_t)((session_id >> 16) & 0xFFU);
	sid[2] = (uint8_t)((session_id >> 8) & 0xFFU);
	sid[3] = (uint8_t)(session_id & 0xFFU);

	while (off < n) {
		uint8_t msg[4 + 64 + 1 + 1 + AUTH_MD5_LEN];
		uint8_t dig[AUTH_MD5_LEN];
		size_t m = 0U;
		size_t take;

		memcpy(&msg[m], sid, 4U);
		m += 4U;
		memcpy(&msg[m], key, key_len);
		m += key_len;
		msg[m++] = version;
		msg[m++] = seq_no;
		if (block != 0U) {
			memcpy(&msg[m], prev, sizeof(prev));
			m += sizeof(prev);
		}
		auth_md5(msg, m, dig);
		memcpy(prev, dig, sizeof(prev));

		take = n - off;
		if (take > AUTH_MD5_LEN) {
			take = AUTH_MD5_LEN;
		}
		memcpy(&pad[off], dig, take);
		off += take;
		block++;
	}
}

static void test_obfuscation_matches_an_independent_keystream(void)
{
	tacacs_hdr_t h;
	uint8_t body[100];
	uint8_t plain[100];
	uint8_t pad[100];
	size_t i;

	memset(&h, 0, sizeof(h));
	h.version = TACACS_VER(TACACS_VER_MINOR_ONE);
	h.type = TACACS_TYPE_AUTHEN;
	h.seq_no = 1U;
	h.session_id = 0xCAFEBABEU;
	h.length = sizeof(body);

	for (i = 0U; i < sizeof(body); i++) {
		plain[i] = (uint8_t)(i * 7U);
	}
	memcpy(body, plain, sizeof(body));

	TEST_ASSERT_EQUAL_INT(0, tacacs_obfuscate(&h, KEY_B, KEY_L, body,
						  sizeof(body)));
	reference_pad(h.session_id, KEY_B, KEY_L, h.version, h.seq_no, pad,
		      sizeof(pad));
	for (i = 0U; i < sizeof(body); i++) {
		TEST_ASSERT_EQUAL_HEX8((uint8_t)(plain[i] ^ pad[i]), body[i]);
	}

	/* And it is its own inverse. */
	TEST_ASSERT_EQUAL_INT(0, tacacs_obfuscate(&h, KEY_B, KEY_L, body,
						  sizeof(body)));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(plain, body, sizeof(plain));
}

static void test_obfuscation_depends_on_every_input(void)
{
	tacacs_hdr_t h;
	uint8_t a[32];
	uint8_t b[32];
	static const uint8_t zero[32] = { 0 };

	memset(&h, 0, sizeof(h));
	h.version = TACACS_VER(0U);
	h.seq_no = 1U;
	h.session_id = 1U;

	memcpy(a, zero, sizeof(a));
	TEST_ASSERT_EQUAL_INT(0, tacacs_obfuscate(&h, KEY_B, KEY_L, a,
						  sizeof(a)));

	/* A different sequence number gives a different keystream — that is what
	 * stops two packets in one session sharing a pad. */
	h.seq_no = 3U;
	memcpy(b, zero, sizeof(b));
	TEST_ASSERT_EQUAL_INT(0, tacacs_obfuscate(&h, KEY_B, KEY_L, b,
						  sizeof(b)));
	TEST_ASSERT_TRUE(memcmp(a, b, sizeof(a)) != 0);

	/* So does a different session id. */
	h.seq_no = 1U;
	h.session_id = 2U;
	memcpy(b, zero, sizeof(b));
	TEST_ASSERT_EQUAL_INT(0, tacacs_obfuscate(&h, KEY_B, KEY_L, b,
						  sizeof(b)));
	TEST_ASSERT_TRUE(memcmp(a, b, sizeof(a)) != 0);

	/* And a different key. */
	h.session_id = 1U;
	memcpy(b, zero, sizeof(b));
	TEST_ASSERT_EQUAL_INT(0, tacacs_obfuscate(&h, (const uint8_t *)"other",
						  5U, b, sizeof(b)));
	TEST_ASSERT_TRUE(memcmp(a, b, sizeof(a)) != 0);
}

static void test_obfuscation_guards(void)
{
	tacacs_hdr_t h;
	uint8_t body[16];

	memset(&h, 0, sizeof(h));
	memset(body, 0x5A, sizeof(body));

	TEST_ASSERT_EQUAL_INT(-EINVAL, tacacs_obfuscate(NULL, KEY_B, KEY_L, body,
							sizeof(body)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_obfuscate(&h, KEY_B, KEY_L, NULL, 4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_obfuscate(&h, KEY_B, KEY_L, body,
					       TACACS_BODY_MAX + 1U));

	/* No key and no body are both no-ops that leave the buffer alone. */
	TEST_ASSERT_EQUAL_INT(0, tacacs_obfuscate(&h, NULL, 0U, body,
						  sizeof(body)));
	TEST_ASSERT_EQUAL_HEX8(0x5AU, body[0]);
	TEST_ASSERT_EQUAL_INT(0, tacacs_obfuscate(&h, KEY_B, 0U, body,
						  sizeof(body)));
	TEST_ASSERT_EQUAL_HEX8(0x5AU, body[0]);
	TEST_ASSERT_EQUAL_INT(0, tacacs_obfuscate(&h, KEY_B, KEY_L, body, 0U));
	TEST_ASSERT_EQUAL_HEX8(0x5AU, body[0]);
	TEST_ASSERT_EQUAL_INT(0, tacacs_obfuscate(&h, KEY_B, KEY_L, NULL, 0U));
}

/* ------------------------------------------------------------------------- */
/* authentication START                                                      */
/* ------------------------------------------------------------------------- */

static tacacs_authen_start_t base_start(void)
{
	tacacs_authen_start_t s;

	memset(&s, 0, sizeof(s));
	s.session_id = 0x0BADF00DU;
	s.action = TACACS_AUTHEN_LOGIN;
	s.priv_lvl = TACACS_PRIV_USER;
	s.authen_type = TACACS_AUTHEN_TYPE_PAP;
	s.authen_service = TACACS_SVC_LOGIN;
	s.user = "topher";
	s.port = "https";
	s.rem_addr = "10.0.0.9";
	s.password = "hunter2!";
	return s;
}

static void test_authen_start_pap_layout(void)
{
	tacacs_authen_start_t s = base_start();
	uint8_t out[TACACS_PKT_MAX];
	uint8_t body[TACACS_BODY_MAX];
	tacacs_hdr_t h;
	size_t len = 0U;
	size_t off;

	s.single_connect = true;
	TEST_ASSERT_EQUAL_INT(0, tacacs_build_authen_start(&s, KEY_B, KEY_L, out,
							   sizeof(out), &len));

	TEST_ASSERT_EQUAL_INT(0, tacacs_hdr_parse(out, len, &h));
	/* PAP selects minor version 1 (RFC 8907 §5.1). */
	TEST_ASSERT_EQUAL_HEX8(TACACS_VER(TACACS_VER_MINOR_ONE), h.version);
	TEST_ASSERT_EQUAL_HEX8(TACACS_TYPE_AUTHEN, h.type);
	TEST_ASSERT_EQUAL_HEX8(1U, h.seq_no);
	TEST_ASSERT_EQUAL_HEX8(TACACS_FLAG_SINGLE_CONNECT, h.flags);
	TEST_ASSERT_EQUAL_HEX32(0x0BADF00DU, h.session_id);
	TEST_ASSERT_EQUAL_UINT32(len - TACACS_HDR_LEN, h.length);

	/* De-obfuscate and check the eight fixed octets plus the four strings. */
	memcpy(body, &out[TACACS_HDR_LEN], h.length);
	TEST_ASSERT_EQUAL_INT(0, tacacs_obfuscate(&h, KEY_B, KEY_L, body,
						  h.length));
	TEST_ASSERT_EQUAL_HEX8(TACACS_AUTHEN_LOGIN, body[0]);
	TEST_ASSERT_EQUAL_HEX8(TACACS_PRIV_USER, body[1]);
	TEST_ASSERT_EQUAL_HEX8(TACACS_AUTHEN_TYPE_PAP, body[2]);
	TEST_ASSERT_EQUAL_HEX8(TACACS_SVC_LOGIN, body[3]);
	TEST_ASSERT_EQUAL_HEX8(6U, body[4]);  /* user_len     */
	TEST_ASSERT_EQUAL_HEX8(5U, body[5]);  /* port_len     */
	TEST_ASSERT_EQUAL_HEX8(8U, body[6]);  /* rem_addr_len */
	TEST_ASSERT_EQUAL_HEX8(8U, body[7]);  /* data_len     */
	off = 8U;
	TEST_ASSERT_EQUAL_MEMORY("topher", &body[off], 6U);
	off += 6U;
	TEST_ASSERT_EQUAL_MEMORY("https", &body[off], 5U);
	off += 5U;
	TEST_ASSERT_EQUAL_MEMORY("10.0.0.9", &body[off], 8U);
	off += 8U;
	TEST_ASSERT_EQUAL_MEMORY("hunter2!", &body[off], 8U);
	off += 8U;
	TEST_ASSERT_EQUAL_UINT(h.length, off);
}

static void test_authen_start_ascii_omits_the_password(void)
{
	tacacs_authen_start_t s = base_start();
	uint8_t out[TACACS_PKT_MAX];
	uint8_t body[TACACS_BODY_MAX];
	tacacs_hdr_t h;
	size_t len = 0U;

	s.authen_type = TACACS_AUTHEN_TYPE_ASCII;
	TEST_ASSERT_EQUAL_INT(0, tacacs_build_authen_start(&s, KEY_B, KEY_L, out,
							   sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(0, tacacs_hdr_parse(out, len, &h));
	/* ASCII login uses minor version 0. */
	TEST_ASSERT_EQUAL_HEX8(TACACS_VER(TACACS_VER_MINOR_DEFAULT), h.version);

	memcpy(body, &out[TACACS_HDR_LEN], h.length);
	TEST_ASSERT_EQUAL_INT(0, tacacs_obfuscate(&h, KEY_B, KEY_L, body,
						  h.length));
	TEST_ASSERT_EQUAL_HEX8(0U, body[7]); /* data_len */
}

static void test_authen_start_unencrypted_sets_the_flag(void)
{
	tacacs_authen_start_t s = base_start();
	uint8_t out[TACACS_PKT_MAX];
	tacacs_hdr_t h;
	size_t len = 0U;

	/* No key means the body is plaintext, and the flag must say so — the
	 * two can never disagree because one function sets both. */
	TEST_ASSERT_EQUAL_INT(0, tacacs_build_authen_start(&s, NULL, 0U, out,
							   sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(0, tacacs_hdr_parse(out, len, &h));
	TEST_ASSERT_TRUE((h.flags & TACACS_FLAG_UNENCRYPTED) != 0U);
	TEST_ASSERT_EQUAL_HEX8(TACACS_AUTHEN_LOGIN, out[TACACS_HDR_LEN]);
	TEST_ASSERT_EQUAL_MEMORY("topher", &out[TACACS_HDR_LEN + 8U], 6U);
}

static void test_authen_start_guards(void)
{
	tacacs_authen_start_t s;
	uint8_t out[TACACS_PKT_MAX];
	char big[AUTH_USER_MAX + 2U];
	char big65[66];
	char bigpw[AUTH_SECRET_MAX + 2U];
	size_t len = 0U;

	memset(big, 'u', sizeof(big) - 1U);
	big[sizeof(big) - 1U] = '\0';
	/* The port and rem_addr fields cap at 64 octets, so 65 is the first
	 * rejected length — 64 must still be accepted. */
	memset(big65, 'p', sizeof(big65) - 1U);
	big65[sizeof(big65) - 1U] = '\0';
	memset(bigpw, 'p', sizeof(bigpw) - 1U);
	bigpw[sizeof(bigpw) - 1U] = '\0';

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_start(NULL, KEY_B, KEY_L, out,
							sizeof(out), &len));
	s = base_start();
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_start(&s, KEY_B, KEY_L, NULL,
							sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_start(&s, KEY_B, KEY_L, out,
							sizeof(out), NULL));
	s.user = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_start(&s, KEY_B, KEY_L, out,
							sizeof(out), &len));
	s = base_start();
	s.user = "";
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_start(&s, KEY_B, KEY_L, out,
							sizeof(out), &len));
	s = base_start();
	s.user = big;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_start(&s, KEY_B, KEY_L, out,
							sizeof(out), &len));
	s = base_start();
	s.port = big;
	/* 64 is the port cap, so a 64-octet name is accepted... */
	TEST_ASSERT_EQUAL_INT(0,
			      tacacs_build_authen_start(&s, KEY_B, KEY_L, out,
							sizeof(out), &len));
	s.port = big65; /* ...and 65 is not. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_start(&s, KEY_B, KEY_L, out,
							sizeof(out), &len));
	s = base_start();
	s.rem_addr = big65;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_start(&s, KEY_B, KEY_L, out,
							sizeof(out), &len));
	s = base_start();
	s.password = bigpw;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_start(&s, KEY_B, KEY_L, out,
							sizeof(out), &len));
	/* A PAP login with no password is not a login. */
	s = base_start();
	s.password = "";
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_start(&s, KEY_B, KEY_L, out,
							sizeof(out), &len));
	s = base_start();
	s.password = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_start(&s, KEY_B, KEY_L, out,
							sizeof(out), &len));

	s = base_start();
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      tacacs_build_authen_start(&s, KEY_B, KEY_L, out,
							20U, &len));
}

/* ------------------------------------------------------------------------- */
/* authentication CONTINUE                                                   */
/* ------------------------------------------------------------------------- */

static void test_authen_continue_layout(void)
{
	uint8_t out[TACACS_PKT_MAX];
	uint8_t body[TACACS_BODY_MAX];
	tacacs_hdr_t h;
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, tacacs_build_authen_continue(0x1234U, 3U,
							      "secret", false,
							      KEY_B, KEY_L, out,
							      sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(0, tacacs_hdr_parse(out, len, &h));
	TEST_ASSERT_EQUAL_HEX8(3U, h.seq_no);

	memcpy(body, &out[TACACS_HDR_LEN], h.length);
	TEST_ASSERT_EQUAL_INT(0, tacacs_obfuscate(&h, KEY_B, KEY_L, body,
						  h.length));
	/* user_msg_len(2) data_len(2) flags(1) then the message. */
	TEST_ASSERT_EQUAL_HEX8(0U, body[0]);
	TEST_ASSERT_EQUAL_HEX8(6U, body[1]);
	TEST_ASSERT_EQUAL_HEX8(0U, body[2]);
	TEST_ASSERT_EQUAL_HEX8(0U, body[3]);
	TEST_ASSERT_EQUAL_HEX8(0U, body[4]);
	TEST_ASSERT_EQUAL_MEMORY("secret", &body[5], 6U);
	TEST_ASSERT_EQUAL_UINT32(11U, h.length);

	/* An abort carries its reason in the data field with the abort flag. */
	TEST_ASSERT_EQUAL_INT(0, tacacs_build_authen_continue(0x1234U, 5U,
							      "user cancelled",
							      true, KEY_B, KEY_L,
							      out, sizeof(out),
							      &len));
	TEST_ASSERT_EQUAL_INT(0, tacacs_hdr_parse(out, len, &h));
	memcpy(body, &out[TACACS_HDR_LEN], h.length);
	TEST_ASSERT_EQUAL_INT(0, tacacs_obfuscate(&h, KEY_B, KEY_L, body,
						  h.length));
	TEST_ASSERT_EQUAL_HEX8(0U, body[1]);  /* user_msg_len = 0 */
	TEST_ASSERT_EQUAL_HEX8(14U, body[3]); /* data_len         */
	TEST_ASSERT_EQUAL_HEX8(TACACS_CONTINUE_FLAG_ABORT, body[4]);
	TEST_ASSERT_EQUAL_MEMORY("user cancelled", &body[5], 14U);

	/* An empty continue is legal. */
	TEST_ASSERT_EQUAL_INT(0, tacacs_build_authen_continue(1U, 1U, NULL,
							      false, KEY_B,
							      KEY_L, out,
							      sizeof(out), &len));
	TEST_ASSERT_EQUAL_UINT(TACACS_HDR_LEN + 5U, len);
}

static void test_authen_continue_guards(void)
{
	uint8_t out[TACACS_PKT_MAX];
	char big[AUTH_SECRET_MAX + 2U];
	size_t len = 0U;

	memset(big, 'x', sizeof(big) - 1U);
	big[sizeof(big) - 1U] = '\0';

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_continue(1U, 3U, "x", false,
							   KEY_B, KEY_L, NULL,
							   sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_continue(1U, 3U, "x", false,
							   KEY_B, KEY_L, out,
							   sizeof(out), NULL));
	/* Client packets are odd-numbered. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_continue(1U, 0U, "x", false,
							   KEY_B, KEY_L, out,
							   sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_continue(1U, 2U, "x", false,
							   KEY_B, KEY_L, out,
							   sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_continue(1U, 3U, big, false,
							   KEY_B, KEY_L, out,
							   sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_authen_continue(1U, 3U, big, true,
							   KEY_B, KEY_L, out,
							   sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      tacacs_build_authen_continue(1U, 3U, "hello",
							   false, KEY_B, KEY_L,
							   out, 14U, &len));
}

/* ------------------------------------------------------------------------- */
/* authentication REPLY                                                      */
/* ------------------------------------------------------------------------- */

/** Frame a server packet: header plus an obfuscated body. */
static size_t make_server_pkt(uint8_t *out, uint8_t type, uint8_t seq,
			      uint32_t sid, const uint8_t *body, size_t body_len,
			      bool encrypt)
{
	tacacs_hdr_t h;

	memset(&h, 0, sizeof(h));
	h.version = TACACS_VER(TACACS_VER_MINOR_DEFAULT);
	h.type = type;
	h.seq_no = seq;
	h.flags = encrypt ? 0U : (uint8_t)TACACS_FLAG_UNENCRYPTED;
	h.session_id = sid;
	h.length = (uint32_t)body_len;

	(void)tacacs_hdr_build(out, TACACS_HDR_LEN, &h);
	memcpy(&out[TACACS_HDR_LEN], body, body_len);
	if (encrypt) {
		(void)tacacs_obfuscate(&h, KEY_B, KEY_L, &out[TACACS_HDR_LEN],
				       body_len);
	}
	return TACACS_HDR_LEN + body_len;
}

static void test_authen_reply_parse(void)
{
	uint8_t body[64];
	uint8_t pkt[TACACS_PKT_MAX];
	tacacs_authen_reply_t r;
	size_t len;

	/* status PASS, flags 0, server_msg "welcome", data_len 2. */
	body[0] = TACACS_AUTHEN_STATUS_PASS;
	body[1] = 0U;
	body[2] = 0U;
	body[3] = 7U;
	body[4] = 0U;
	body[5] = 2U;
	memcpy(&body[6], "welcome", 7U);
	body[13] = 0xAAU;
	body[14] = 0xBBU;
	len = make_server_pkt(pkt, TACACS_TYPE_AUTHEN, 2U, 0x99U, body, 15U,
			      true);

	TEST_ASSERT_EQUAL_INT(0, tacacs_parse_authen_reply(pkt, len, 0x99U, 2U,
							   KEY_B, KEY_L, &r));
	TEST_ASSERT_EQUAL_HEX8(TACACS_AUTHEN_STATUS_PASS, r.status);
	TEST_ASSERT_EQUAL_HEX8(0U, r.flags);
	TEST_ASSERT_EQUAL_STRING("welcome", r.server_msg);
	TEST_ASSERT_EQUAL_UINT(2U, r.data_len);
	TEST_ASSERT_EQUAL_HEX8(2U, r.seq_no);

	/* expect_seq 0 accepts any even, non-zero sequence number. */
	TEST_ASSERT_EQUAL_INT(0, tacacs_parse_authen_reply(pkt, len, 0x99U, 0U,
							   KEY_B, KEY_L, &r));

	/* Wrong session, wrong sequence. */
	TEST_ASSERT_EQUAL_INT(-ENOMSG,
			      tacacs_parse_authen_reply(pkt, len, 0x98U, 2U,
							KEY_B, KEY_L, &r));
	TEST_ASSERT_EQUAL_INT(-ENOMSG,
			      tacacs_parse_authen_reply(pkt, len, 0x99U, 4U,
							KEY_B, KEY_L, &r));

	/* A client-numbered (odd) packet is not a server reply. */
	len = make_server_pkt(pkt, TACACS_TYPE_AUTHEN, 3U, 0x99U, body, 15U,
			      true);
	TEST_ASSERT_EQUAL_INT(-ENOMSG,
			      tacacs_parse_authen_reply(pkt, len, 0x99U, 0U,
							KEY_B, KEY_L, &r));

	/* Nor is an authorization packet. */
	len = make_server_pkt(pkt, TACACS_TYPE_AUTHOR, 2U, 0x99U, body, 15U,
			      true);
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      tacacs_parse_authen_reply(pkt, len, 0x99U, 2U,
							KEY_B, KEY_L, &r));
}

static void test_authen_reply_sanitises_and_bounds(void)
{
	uint8_t body[TACACS_BODY_MAX];
	uint8_t pkt[TACACS_PKT_MAX];
	tacacs_authen_reply_t r;
	size_t len;
	size_t i;

	/* A 200-octet server message with control characters in it. */
	body[0] = TACACS_AUTHEN_STATUS_GETPASS;
	body[1] = 0U;
	body[2] = 0U;
	body[3] = 200U;
	body[4] = 0U;
	body[5] = 0U;
	for (i = 0U; i < 200U; i++) {
		body[6U + i] = (uint8_t)((i % 2U) ? 0x01U : 'A');
	}
	len = make_server_pkt(pkt, TACACS_TYPE_AUTHEN, 2U, 1U, body, 206U, true);

	TEST_ASSERT_EQUAL_INT(0, tacacs_parse_authen_reply(pkt, len, 1U, 2U,
							   KEY_B, KEY_L, &r));
	TEST_ASSERT_EQUAL_UINT(TACACS_MSG_MAX, strlen(r.server_msg));
	TEST_ASSERT_EQUAL_CHAR('A', r.server_msg[0]);
	TEST_ASSERT_EQUAL_CHAR('.', r.server_msg[1]);
}

static void test_authen_reply_length_fields_must_add_up(void)
{
	uint8_t body[32];
	uint8_t pkt[TACACS_PKT_MAX];
	tacacs_authen_reply_t r;
	size_t len;

	memset(body, 0, sizeof(body));
	body[0] = TACACS_AUTHEN_STATUS_FAIL;
	body[3] = 4U; /* server_msg_len = 4 but only 2 octets follow */
	len = make_server_pkt(pkt, TACACS_TYPE_AUTHEN, 2U, 1U, body, 8U, true);
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      tacacs_parse_authen_reply(pkt, len, 1U, 2U, KEY_B,
							KEY_L, &r));

	/* A body shorter than the six fixed octets. */
	len = make_server_pkt(pkt, TACACS_TYPE_AUTHEN, 2U, 1U, body, 5U, true);
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      tacacs_parse_authen_reply(pkt, len, 1U, 2U, KEY_B,
							KEY_L, &r));

	/* A truncated packet: the header promises more than arrived. */
	len = make_server_pkt(pkt, TACACS_TYPE_AUTHEN, 2U, 1U, body, 8U, true);
	TEST_ASSERT_EQUAL_INT(-EAGAIN,
			      tacacs_parse_authen_reply(pkt, len - 2U, 1U, 2U,
							KEY_B, KEY_L, &r));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_parse_authen_reply(NULL, len, 1U, 2U, KEY_B,
							KEY_L, &r));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_parse_authen_reply(pkt, len, 1U, 2U, KEY_B,
							KEY_L, NULL));
}

static void test_authen_reply_unencrypted_body(void)
{
	uint8_t body[16];
	uint8_t pkt[TACACS_PKT_MAX];
	tacacs_authen_reply_t r;
	size_t len;

	memset(body, 0, sizeof(body));
	body[0] = TACACS_AUTHEN_STATUS_PASS;
	body[3] = 2U;
	memcpy(&body[6], "ok", 2U);
	/* A server that sets the unencrypted flag is taken at its word — a
	 * de-obfuscation pass would turn a readable body into noise. */
	len = make_server_pkt(pkt, TACACS_TYPE_AUTHEN, 2U, 1U, body, 8U, false);
	TEST_ASSERT_EQUAL_INT(0, tacacs_parse_authen_reply(pkt, len, 1U, 2U,
							   KEY_B, KEY_L, &r));
	TEST_ASSERT_EQUAL_STRING("ok", r.server_msg);
}

/* ------------------------------------------------------------------------- */
/* authorization                                                             */
/* ------------------------------------------------------------------------- */

static void test_author_request_layout(void)
{
	static const char *const args[] = { "service=sts1000", "cmd*" };
	tacacs_author_req_t q;
	uint8_t out[TACACS_PKT_MAX];
	uint8_t body[TACACS_BODY_MAX];
	tacacs_hdr_t h;
	size_t len = 0U;
	size_t off;

	memset(&q, 0, sizeof(q));
	q.session_id = 0x77U;
	q.seq_no = 1U;
	q.authen_method = TACACS_AUTHEN_METH_TACACSPLUS;
	q.priv_lvl = TACACS_PRIV_USER;
	q.authen_type = TACACS_AUTHEN_TYPE_PAP;
	q.authen_service = TACACS_SVC_LOGIN;
	q.user = "topher";
	q.port = "https";
	q.rem_addr = "";
	q.args = args;
	q.n_args = ARRAY_LEN(args);

	TEST_ASSERT_EQUAL_INT(0, tacacs_build_author_request(&q, KEY_B, KEY_L,
							     out, sizeof(out),
							     &len));
	TEST_ASSERT_EQUAL_INT(0, tacacs_hdr_parse(out, len, &h));
	TEST_ASSERT_EQUAL_HEX8(TACACS_TYPE_AUTHOR, h.type);

	memcpy(body, &out[TACACS_HDR_LEN], h.length);
	TEST_ASSERT_EQUAL_INT(0, tacacs_obfuscate(&h, KEY_B, KEY_L, body,
						  h.length));
	TEST_ASSERT_EQUAL_HEX8(TACACS_AUTHEN_METH_TACACSPLUS, body[0]);
	TEST_ASSERT_EQUAL_HEX8(TACACS_PRIV_USER, body[1]);
	TEST_ASSERT_EQUAL_HEX8(TACACS_AUTHEN_TYPE_PAP, body[2]);
	TEST_ASSERT_EQUAL_HEX8(TACACS_SVC_LOGIN, body[3]);
	TEST_ASSERT_EQUAL_HEX8(6U, body[4]);
	TEST_ASSERT_EQUAL_HEX8(5U, body[5]);
	TEST_ASSERT_EQUAL_HEX8(0U, body[6]);
	TEST_ASSERT_EQUAL_HEX8(2U, body[7]); /* arg_cnt */
	TEST_ASSERT_EQUAL_HEX8(15U, body[8]);
	TEST_ASSERT_EQUAL_HEX8(4U, body[9]);
	off = 10U;
	TEST_ASSERT_EQUAL_MEMORY("topher", &body[off], 6U);
	off += 6U;
	TEST_ASSERT_EQUAL_MEMORY("https", &body[off], 5U);
	off += 5U;
	TEST_ASSERT_EQUAL_MEMORY("service=sts1000", &body[off], 15U);
	off += 15U;
	TEST_ASSERT_EQUAL_MEMORY("cmd*", &body[off], 4U);
	off += 4U;
	TEST_ASSERT_EQUAL_UINT(h.length, off);
}

static void test_author_request_guards(void)
{
	static const char *const args[] = { "a", "b", "c", "d",
					    "e", "f", "g", "h", "i" };
	static const char *const with_null[] = { "a", NULL };
	static const char *const with_empty[] = { "" };
	tacacs_author_req_t q;
	uint8_t out[TACACS_PKT_MAX];
	char big[AUTH_USER_MAX + 2U];
	char big65[66];
	size_t len = 0U;

	memset(big, 'u', sizeof(big) - 1U);
	big[sizeof(big) - 1U] = '\0';
	memset(big65, 'p', sizeof(big65) - 1U);
	big65[sizeof(big65) - 1U] = '\0';

	memset(&q, 0, sizeof(q));
	q.seq_no = 1U;
	q.user = "u";

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_author_request(NULL, KEY_B, KEY_L,
							  out, sizeof(out),
							  &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_author_request(&q, KEY_B, KEY_L, NULL,
							  sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_author_request(&q, KEY_B, KEY_L, out,
							  sizeof(out), NULL));

	q.user = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_author_request(&q, KEY_B, KEY_L, out,
							  sizeof(out), &len));
	q.user = "";
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_author_request(&q, KEY_B, KEY_L, out,
							  sizeof(out), &len));
	q.user = big;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_author_request(&q, KEY_B, KEY_L, out,
							  sizeof(out), &len));

	q.user = "u";
	q.n_args = ARRAY_LEN(args);
	q.args = args;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_author_request(&q, KEY_B, KEY_L, out,
							  sizeof(out), &len));
	q.n_args = 2U;
	q.args = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_author_request(&q, KEY_B, KEY_L, out,
							  sizeof(out), &len));
	q.args = with_null;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_author_request(&q, KEY_B, KEY_L, out,
							  sizeof(out), &len));
	q.args = with_empty;
	q.n_args = 1U;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_author_request(&q, KEY_B, KEY_L, out,
							  sizeof(out), &len));

	q.n_args = 0U;
	q.args = NULL;
	q.seq_no = 2U;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_author_request(&q, KEY_B, KEY_L, out,
							  sizeof(out), &len));
	q.seq_no = 1U;
	q.port = big65;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_author_request(&q, KEY_B, KEY_L, out,
							  sizeof(out), &len));
	q.port = NULL;
	q.rem_addr = big65;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_build_author_request(&q, KEY_B, KEY_L, out,
							  sizeof(out), &len));
	q.rem_addr = NULL;
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      tacacs_build_author_request(&q, KEY_B, KEY_L, out,
							  16U, &len));
	q.single_connect = true;
	TEST_ASSERT_EQUAL_INT(0, tacacs_build_author_request(&q, KEY_B, KEY_L,
							     out, sizeof(out),
							     &len));
}

/** Assemble an authorization RESPONSE body. */
static size_t make_author_body(uint8_t *body, uint8_t status,
			       const char *const *args, size_t n_args,
			       const char *msg)
{
	size_t off;
	size_t i;
	size_t mlen = (msg != NULL) ? strlen(msg) : 0U;

	body[0] = status;
	body[1] = (uint8_t)n_args;
	body[2] = 0U;
	body[3] = (uint8_t)mlen;
	body[4] = 0U;
	body[5] = 0U;
	for (i = 0U; i < n_args; i++) {
		body[6U + i] = (uint8_t)strlen(args[i]);
	}
	off = 6U + n_args;
	if (mlen != 0U) {
		memcpy(&body[off], msg, mlen);
		off += mlen;
	}
	for (i = 0U; i < n_args; i++) {
		size_t n = strlen(args[i]);

		memcpy(&body[off], args[i], n);
		off += n;
	}
	return off;
}

static void test_author_response_parse(void)
{
	static const char *const args[] = { "priv-lvl=15", "service=sts1000" };
	uint8_t body[TACACS_BODY_MAX];
	uint8_t pkt[TACACS_PKT_MAX];
	tacacs_author_rsp_t r;
	size_t body_len;
	size_t len;

	body_len = make_author_body(body, TACACS_AUTHOR_STATUS_PASS_ADD, args,
				    ARRAY_LEN(args), "ok");
	len = make_server_pkt(pkt, TACACS_TYPE_AUTHOR, 2U, 0x55U, body, body_len,
			      true);

	TEST_ASSERT_EQUAL_INT(0, tacacs_parse_author_response(pkt, len, 0x55U,
							      2U, KEY_B, KEY_L,
							      &r));
	TEST_ASSERT_EQUAL_HEX8(TACACS_AUTHOR_STATUS_PASS_ADD, r.status);
	TEST_ASSERT_EQUAL_STRING("ok", r.server_msg);
	TEST_ASSERT_EQUAL_UINT8(2U, r.n_args);
	TEST_ASSERT_EQUAL_STRING("priv-lvl=15", r.args[0]);
	TEST_ASSERT_EQUAL_STRING("service=sts1000", r.args[1]);
	TEST_ASSERT_EQUAL_UINT8(0U, r.args_dropped);

	TEST_ASSERT_EQUAL_INT(-ENOMSG,
			      tacacs_parse_author_response(pkt, len, 0x56U, 2U,
							   KEY_B, KEY_L, &r));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_parse_author_response(NULL, len, 0x55U, 2U,
							   KEY_B, KEY_L, &r));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      tacacs_parse_author_response(pkt, len, 0x55U, 2U,
							   KEY_B, KEY_L, NULL));
}

static void test_author_response_drops_excess_args(void)
{
	static const char *const many[] = { "a=1", "b=2", "c=3", "d=4", "e=5",
					    "f=6", "g=7", "h=8", "i=9", "j=10" };
	static const char *const longarg[] = {
		"x=012345678901234567890123456789012345678901234567890123456789"
		"0123456789",
		"role=admin"
	};
	uint8_t body[TACACS_BODY_MAX];
	uint8_t pkt[TACACS_PKT_MAX];
	tacacs_author_rsp_t r;
	size_t body_len;
	size_t len;

	body_len = make_author_body(body, TACACS_AUTHOR_STATUS_PASS_ADD, many,
				    ARRAY_LEN(many), NULL);
	len = make_server_pkt(pkt, TACACS_TYPE_AUTHOR, 2U, 1U, body, body_len,
			      true);
	TEST_ASSERT_EQUAL_INT(0, tacacs_parse_author_response(pkt, len, 1U, 2U,
							      KEY_B, KEY_L, &r));
	TEST_ASSERT_EQUAL_UINT8(TACACS_ARGS_MAX, r.n_args);
	TEST_ASSERT_EQUAL_UINT8(ARRAY_LEN(many) - TACACS_ARGS_MAX,
				r.args_dropped);

	/* An over-long argument is dropped, and the one after it still lands —
	 * so a server padding its response cannot hide the role. */
	body_len = make_author_body(body, TACACS_AUTHOR_STATUS_PASS_ADD, longarg,
				    ARRAY_LEN(longarg), NULL);
	len = make_server_pkt(pkt, TACACS_TYPE_AUTHOR, 2U, 1U, body, body_len,
			      true);
	TEST_ASSERT_EQUAL_INT(0, tacacs_parse_author_response(pkt, len, 1U, 2U,
							      KEY_B, KEY_L, &r));
	TEST_ASSERT_EQUAL_UINT8(1U, r.n_args);
	TEST_ASSERT_EQUAL_UINT8(1U, r.args_dropped);
	TEST_ASSERT_EQUAL_STRING("role=admin", r.args[0]);
}

static void test_author_response_length_fields_must_add_up(void)
{
	uint8_t body[64];
	uint8_t pkt[TACACS_PKT_MAX];
	tacacs_author_rsp_t r;
	size_t len;

	memset(body, 0, sizeof(body));
	body[0] = TACACS_AUTHOR_STATUS_PASS_ADD;
	body[1] = 4U; /* arg_cnt 4, but no arg-length octets follow */
	len = make_server_pkt(pkt, TACACS_TYPE_AUTHOR, 2U, 1U, body, 6U, true);
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      tacacs_parse_author_response(pkt, len, 1U, 2U,
							   KEY_B, KEY_L, &r));

	/* arg_cnt 1 with a length that overruns the body. */
	body[1] = 1U;
	body[6] = 200U;
	len = make_server_pkt(pkt, TACACS_TYPE_AUTHOR, 2U, 1U, body, 7U, true);
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      tacacs_parse_author_response(pkt, len, 1U, 2U,
							   KEY_B, KEY_L, &r));

	/* Under the six fixed octets. */
	len = make_server_pkt(pkt, TACACS_TYPE_AUTHOR, 2U, 1U, body, 3U, true);
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      tacacs_parse_author_response(pkt, len, 1U, 2U,
							   KEY_B, KEY_L, &r));
}

/* ------------------------------------------------------------------------- */
/* role mapping                                                              */
/* ------------------------------------------------------------------------- */

static void set_args(tacacs_author_rsp_t *r, const char *const *args, size_t n)
{
	size_t i;

	memset(r, 0, sizeof(*r));
	for (i = 0U; i < n && i < TACACS_ARGS_MAX; i++) {
		size_t l = strlen(args[i]);

		if (l > TACACS_ARG_LEN_MAX) {
			l = TACACS_ARG_LEN_MAX;
		}
		memcpy(r->args[i], args[i], l);
		r->args[i][l] = '\0';
		r->n_args++;
	}
}

static void test_role_from_args(void)
{
	static const char *const a_role[] = { "service=x", "role=admin" };
	static const char *const a_role_star[] = { "role*operator" };
	static const char *const a_priv15[] = { "priv-lvl=15" };
	static const char *const a_priv7[] = { "priv_lvl=7" };
	static const char *const a_priv1[] = { "priv-lvl=1" };
	static const char *const a_priv0[] = { "priv-lvl=0" };
	static const char *const a_priv99[] = { "priv-lvl=99" };
	static const char *const a_both[] = { "priv-lvl=1", "role=admin" };
	static const char *const a_none[] = { "service=sts1000", "cmd=" };
	static const char *const a_bad_role[] = { "role=wizard" };
	static const char *const a_bad_priv[] = { "priv-lvl=abc" };
	static const char *const a_no_sep[] = { "rolestuff", "priv-lvlx" };
	tacacs_author_rsp_t r;
	bool ex = false;

	set_args(&r, a_role, ARRAY_LEN(a_role));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_ADMIN,
				tacacs_role_from_args(&r, &ex));
	TEST_ASSERT_TRUE(ex);

	set_args(&r, a_role_star, ARRAY_LEN(a_role_star));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_OPERATOR,
				tacacs_role_from_args(&r, &ex));

	set_args(&r, a_priv15, ARRAY_LEN(a_priv15));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_ADMIN,
				tacacs_role_from_args(&r, &ex));
	set_args(&r, a_priv7, ARRAY_LEN(a_priv7));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_OPERATOR,
				tacacs_role_from_args(&r, &ex));
	set_args(&r, a_priv1, ARRAY_LEN(a_priv1));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER,
				tacacs_role_from_args(&r, &ex));
	set_args(&r, a_priv0, ARRAY_LEN(a_priv0));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER,
				tacacs_role_from_args(&r, &ex));
	set_args(&r, a_priv99, ARRAY_LEN(a_priv99));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_ADMIN,
				tacacs_role_from_args(&r, &ex));

	/* `role=` is checked before `priv-lvl=`, so an explicit role wins. */
	set_args(&r, a_both, ARRAY_LEN(a_both));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_ADMIN,
				tacacs_role_from_args(&r, &ex));

	/* Nothing to go on: least privilege, and not explicit. */
	set_args(&r, a_none, ARRAY_LEN(a_none));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER,
				tacacs_role_from_args(&r, &ex));
	TEST_ASSERT_FALSE(ex);

	set_args(&r, a_bad_role, ARRAY_LEN(a_bad_role));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER,
				tacacs_role_from_args(&r, &ex));
	TEST_ASSERT_FALSE(ex);

	set_args(&r, a_bad_priv, ARRAY_LEN(a_bad_priv));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER,
				tacacs_role_from_args(&r, &ex));
	TEST_ASSERT_FALSE(ex);

	set_args(&r, a_no_sep, ARRAY_LEN(a_no_sep));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER,
				tacacs_role_from_args(&r, &ex));
	TEST_ASSERT_FALSE(ex);

	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER,
				tacacs_role_from_args(NULL, &ex));
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER,
				tacacs_role_from_args(NULL, NULL));
}

/* ------------------------------------------------------------------------- */
/* hostile input                                                             */
/* ------------------------------------------------------------------------- */

static void test_random_packets_never_crash(void)
{
	test_rng_t g;
	uint8_t pkt[300];
	tacacs_authen_reply_t ar;
	tacacs_author_rsp_t zr;
	unsigned int i;

	test_rng_init(&g, 0x8907U);
	for (i = 0U; i < 30000U; i++) {
		size_t n = 1U + test_rng_below(&g, sizeof(pkt));

		test_rng_fill(&g, pkt, n);
		/* Steer some of them past the version gate so the body parsers
		 * see real work rather than an early reject. */
		if ((i & 1U) != 0U && n >= TACACS_HDR_LEN) {
			pkt[0] = TACACS_VER(TACACS_VER_MINOR_DEFAULT);
			pkt[8] = 0U;
			pkt[9] = 0U;
			pkt[10] = 0U;
			pkt[11] = (uint8_t)(n - TACACS_HDR_LEN);
		}
		(void)tacacs_parse_authen_reply(pkt, n, 0U, 0U, KEY_B, KEY_L,
						&ar);
		(void)tacacs_parse_author_response(pkt, n, 0U, 0U, KEY_B, KEY_L,
						   &zr);
	}
}

static void test_mutated_valid_packets_never_crash(void)
{
	static const char *const args[] = { "priv-lvl=15", "role=operator" };
	test_rng_t g;
	uint8_t body[TACACS_BODY_MAX];
	uint8_t pkt[TACACS_PKT_MAX];
	tacacs_author_rsp_t zr;
	tacacs_authen_reply_t ar;
	unsigned int i;

	test_rng_init(&g, 0x4711U);
	for (i = 0U; i < 20000U; i++) {
		size_t body_len = make_author_body(body,
						   TACACS_AUTHOR_STATUS_PASS_ADD,
						   args, ARRAY_LEN(args), "hi");
		size_t len = make_server_pkt(pkt, TACACS_TYPE_AUTHOR, 2U, 1U,
					     body, body_len, false);
		size_t pos = test_rng_below(&g, len);

		pkt[pos] ^= (uint8_t)(1U << test_rng_below(&g, 8U));
		(void)tacacs_parse_author_response(pkt, len, 1U, 0U, KEY_B,
						   KEY_L, &zr);
		(void)tacacs_parse_authen_reply(pkt, len, 1U, 0U, KEY_B, KEY_L,
						&ar);
		if (tacacs_parse_author_response(pkt, len, 1U, 0U, KEY_B, KEY_L,
						 &zr) == 0) {
			(void)tacacs_role_from_args(&zr, NULL);
		}
	}
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_hdr_layout);
	RUN_TEST(test_hdr_guards);

	RUN_TEST(test_obfuscation_matches_an_independent_keystream);
	RUN_TEST(test_obfuscation_depends_on_every_input);
	RUN_TEST(test_obfuscation_guards);

	RUN_TEST(test_authen_start_pap_layout);
	RUN_TEST(test_authen_start_ascii_omits_the_password);
	RUN_TEST(test_authen_start_unencrypted_sets_the_flag);
	RUN_TEST(test_authen_start_guards);

	RUN_TEST(test_authen_continue_layout);
	RUN_TEST(test_authen_continue_guards);

	RUN_TEST(test_authen_reply_parse);
	RUN_TEST(test_authen_reply_sanitises_and_bounds);
	RUN_TEST(test_authen_reply_length_fields_must_add_up);
	RUN_TEST(test_authen_reply_unencrypted_body);

	RUN_TEST(test_author_request_layout);
	RUN_TEST(test_author_request_guards);
	RUN_TEST(test_author_response_parse);
	RUN_TEST(test_author_response_drops_excess_args);
	RUN_TEST(test_author_response_length_fields_must_add_up);

	RUN_TEST(test_role_from_args);

	RUN_TEST(test_random_packets_never_crash);
	RUN_TEST(test_mutated_valid_packets_never_crash);

	return UNITY_END();
}
