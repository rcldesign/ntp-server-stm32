/*
 * STS1000 "Meridian" — core/auth RADIUS codec unit tests.
 *
 * Provenance of the expectations (ARCHITECTURE.md §9):
 *
 *   - **RFC 2865 §7.1 "User Telnet to Specified Host" is transcribed literally.**
 *     Both the Access-Request and the Access-Accept from that worked example are
 *     byte arrays below, with the shared secret "xyzzy5461" and the password
 *     "arctangent". Three separate properties are checked against them:
 *
 *       1. radius_hide_password() reproduces the example's User-Password
 *          attribute value exactly — that pins MD5(secret ‖ RequestAuth) ⊕ p1
 *          and the zero padding.
 *       2. radius_response_auth() reproduces the example's Response
 *          Authenticator exactly — that pins the MD5 input order
 *          (code ‖ id ‖ length ‖ *request* authenticator ‖ attrs ‖ secret),
 *          which is the one thing implementations get wrong.
 *       3. radius_parse_response() accepts the reply and extracts its
 *          attributes.
 *
 *     Because both directions of the same published exchange are reproduced, the
 *     local MD5 is validated at the same time: a wrong MD5 could not produce
 *     either value.
 *
 *   - **RFC 1321 §A.5 MD5 test suite** is checked directly, so an MD5 fault is
 *     attributed to MD5 rather than to RADIUS. **RFC 2202 §2** supplies the
 *     HMAC-MD5 vectors that the Message-Authenticator depends on.
 *
 *   - Everything else asserts a protocol property the RFC states: a reply whose
 *     Response Authenticator does not verify is dropped, one whose Identifier
 *     does not match is not an answer, and an attribute list with a length octet
 *     below 2 is malformed rather than an infinite loop.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "auth/auth.h"
#include "auth/radius.h"
#include "test_support.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* ------------------------------------------------------------------------- */
/* RFC 2865 §7.1                                                             */
/* ------------------------------------------------------------------------- */

static const char *const SECRET = "xyzzy5461";

/* Access-Request: id 0, length 56, then User-Name "nemo", the hidden
 * User-Password, NAS-IP-Address 192.168.1.16 and NAS-Port 3. */
static const uint8_t v_request[] = {
	0x01, 0x00, 0x00, 0x38, 0x0f, 0x40, 0x3f, 0x94, 0x73, 0x97, 0x80, 0x57,
	0xbd, 0x83, 0xd5, 0xcb, 0x98, 0xf4, 0x22, 0x7a,
	/* User-Name = "nemo" */
	0x01, 0x06, 0x6e, 0x65, 0x6d, 0x6f,
	/* User-Password */
	0x02, 0x12, 0x0d, 0xbe, 0x70, 0x8d, 0x93, 0xd4, 0x13, 0xce, 0x31, 0x96,
	0xe4, 0x3f, 0x78, 0x2a, 0x0a, 0xee,
	/* NAS-IP-Address = 192.168.1.16 */
	0x04, 0x06, 0xc0, 0xa8, 0x01, 0x10,
	/* NAS-Port = 3 */
	0x05, 0x06, 0x00, 0x00, 0x00, 0x03
};

/* Access-Accept: Service-Type Login(1), Login-Service Telnet(0),
 * Login-IP-Host 192.168.1.3. */
static const uint8_t v_accept[] = {
	0x02, 0x00, 0x00, 0x26, 0x86, 0xfe, 0x22, 0x0e, 0x76, 0x24, 0xba, 0x2a,
	0x10, 0x05, 0xf6, 0xbf, 0x9b, 0x55, 0xe0, 0xb2,
	0x06, 0x06, 0x00, 0x00, 0x00, 0x01,
	0x0f, 0x06, 0x00, 0x00, 0x00, 0x00,
	0x0e, 0x06, 0xc0, 0xa8, 0x01, 0x03
};

#define REQ_AUTH (&v_request[4])

/* ------------------------------------------------------------------------- */
/* MD5 / HMAC-MD5 (RFC 1321 §A.5, RFC 2202 §2)                               */
/* ------------------------------------------------------------------------- */

static void test_md5_rfc1321_suite(void)
{
	struct {
		const char *in;
		const char *hex;
	} const v[] = {
		{ "", "d41d8cd98f00b204e9800998ecf8427e" },
		{ "a", "0cc175b9c0f1b6a831c399e269772661" },
		{ "abc", "900150983cd24fb0d6963f7d28e17f72" },
		{ "message digest", "f96b697d7cb7938d525a2f31aaf161d0" },
		{ "abcdefghijklmnopqrstuvwxyz",
		  "c3fcd3d76192e4007dfb496cca67e13b" },
		{ "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789",
		  "d174ab98d277d9f5a5611c2c9f419d9f" },
		{ "12345678901234567890123456789012345678901234567890"
		  "123456789012345678901234567890",
		  "57edf4a22be3c955ac49da2e2107b67a" },
	};
	size_t i;

	for (i = 0U; i < ARRAY_LEN(v); i++) {
		uint8_t out[AUTH_MD5_LEN];
		char hex[(AUTH_MD5_LEN * 2U) + 1U];
		size_t j;

		auth_md5(v[i].in, strlen(v[i].in), out);
		for (j = 0U; j < AUTH_MD5_LEN; j++) {
			static const char d[] = "0123456789abcdef";

			hex[j * 2U] = d[(out[j] >> 4) & 0x0FU];
			hex[(j * 2U) + 1U] = d[out[j] & 0x0FU];
		}
		hex[AUTH_MD5_LEN * 2U] = '\0';
		TEST_ASSERT_EQUAL_STRING(v[i].hex, hex);
	}

	/* Guards must be inert, not crash. */
	auth_md5_init(NULL);
	auth_md5_update(NULL, "x", 1U);
	auth_md5_final(NULL, NULL);
	{
		auth_md5_t s;

		auth_md5_init(&s);
		auth_md5_update(&s, NULL, 4U); /* ignored */
		auth_md5_final(&s, NULL);
	}
}

static void test_md5_streaming_matches_one_shot(void)
{
	uint8_t data[300];
	uint8_t a[AUTH_MD5_LEN];
	uint8_t b[AUTH_MD5_LEN];
	test_rng_t g;
	size_t split;

	test_rng_init(&g, 0x1234U);
	test_rng_fill(&g, data, sizeof(data));

	auth_md5(data, sizeof(data), a);
	for (split = 0U; split <= sizeof(data); split += 7U) {
		auth_md5_t s;

		auth_md5_init(&s);
		auth_md5_update(&s, data, split);
		auth_md5_update(&s, &data[split], sizeof(data) - split);
		auth_md5_final(&s, b);
		TEST_ASSERT_EQUAL_HEX8_ARRAY(a, b, sizeof(a));
	}
}

static void test_hmac_md5_rfc2202(void)
{
	uint8_t key[80];
	uint8_t out[AUTH_MD5_LEN];
	/* RFC 2202 §2 case 1: key = 16 x 0x0b, data = "Hi There". */
	static const uint8_t want1[] = { 0x92, 0x94, 0x72, 0x7a, 0x36, 0x38,
					 0xbb, 0x1c, 0x13, 0xf4, 0x8e, 0xf8,
					 0x15, 0x8b, 0xfc, 0x9d };
	/* Case 2: key = "Jefe", data = "what do ya want for nothing?". */
	static const uint8_t want2[] = { 0x75, 0x0c, 0x78, 0x3e, 0x6a, 0xb0,
					 0xb5, 0x03, 0xea, 0xa8, 0x6e, 0x31,
					 0x0a, 0x5d, 0xb7, 0x38 };
	/* Case 6: key = 80 x 0xaa (longer than the block), data as below. */
	static const uint8_t want6[] = { 0x6b, 0x1a, 0xb7, 0xfe, 0x4b, 0xd7,
					 0xbf, 0x8f, 0x0b, 0x62, 0xe6, 0xce,
					 0x61, 0xb9, 0xd0, 0xcd };

	memset(key, 0x0b, 16U);
	auth_hmac_md5(key, 16U, (const uint8_t *)"Hi There", 8U, out);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want1, out, sizeof(want1));

	auth_hmac_md5((const uint8_t *)"Jefe", 4U,
		      (const uint8_t *)"what do ya want for nothing?", 28U, out);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want2, out, sizeof(want2));

	memset(key, 0xaa, sizeof(key));
	auth_hmac_md5(key, sizeof(key),
		      (const uint8_t *)"Test Using Larger Than Block-Size Key - "
				       "Hash Key First",
		      54U, out);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want6, out, sizeof(want6));

	/* Inert guards. */
	auth_hmac_md5(NULL, 0U, (const uint8_t *)"x", 1U, out);
	auth_hmac_md5(key, 4U, (const uint8_t *)"x", 1U, NULL);
}

/* ------------------------------------------------------------------------- */
/* password hiding                                                           */
/* ------------------------------------------------------------------------- */

static void test_hide_password_rfc2865_example(void)
{
	uint8_t out[128];
	size_t out_len = 0U;

	TEST_ASSERT_EQUAL_INT(0, radius_hide_password((const uint8_t *)SECRET,
						      strlen(SECRET), REQ_AUTH,
						      "arctangent", out,
						      sizeof(out), &out_len));
	TEST_ASSERT_EQUAL_UINT(16U, out_len);
	/* The example's User-Password attribute value starts at offset 28. */
	TEST_ASSERT_EQUAL_HEX8_ARRAY(&v_request[28], out, 16U);
}

static void test_hide_password_chaining_and_padding(void)
{
	uint8_t out[128];
	uint8_t expect[48];
	uint8_t b[AUTH_MD5_LEN];
	auth_md5_t h;
	size_t out_len = 0U;
	/* 33 characters: three chunks, the last one two-thirds padding. */
	const char *pw = "0123456789abcdef0123456789abcdefX";
	size_t i;
	size_t chunk;

	TEST_ASSERT_EQUAL_INT(0, radius_hide_password((const uint8_t *)SECRET,
						      strlen(SECRET), REQ_AUTH,
						      pw, out, sizeof(out),
						      &out_len));
	TEST_ASSERT_EQUAL_UINT(48U, out_len);

	/* Independently recompute the chain here, in the other direction: undo
	 * each chunk and check the plaintext comes back with zero padding. */
	for (chunk = 0U; chunk < 3U; chunk++) {
		auth_md5_init(&h);
		auth_md5_update(&h, SECRET, strlen(SECRET));
		if (chunk == 0U) {
			auth_md5_update(&h, REQ_AUTH, RADIUS_AUTH_LEN);
		} else {
			auth_md5_update(&h, &out[(chunk - 1U) * 16U], 16U);
		}
		auth_md5_final(&h, b);
		for (i = 0U; i < 16U; i++) {
			expect[(chunk * 16U) + i] =
				(uint8_t)(out[(chunk * 16U) + i] ^ b[i]);
		}
	}
	TEST_ASSERT_EQUAL_HEX8_ARRAY(pw, expect, 33U);
	for (i = 33U; i < 48U; i++) {
		TEST_ASSERT_EQUAL_HEX8(0U, expect[i]);
	}

	/* An empty password still occupies exactly one chunk. */
	TEST_ASSERT_EQUAL_INT(0, radius_hide_password((const uint8_t *)SECRET,
						      strlen(SECRET), REQ_AUTH,
						      "", out, sizeof(out),
						      &out_len));
	TEST_ASSERT_EQUAL_UINT(16U, out_len);
}

static void test_hide_password_guards(void)
{
	uint8_t out[128];
	char big[AUTH_SECRET_MAX + 2U];
	size_t out_len = 0U;

	memset(big, 'x', sizeof(big) - 1U);
	big[sizeof(big) - 1U] = '\0';

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_hide_password(NULL, 9U, REQ_AUTH, "p", out,
						   sizeof(out), &out_len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_hide_password((const uint8_t *)SECRET, 0U,
						   REQ_AUTH, "p", out,
						   sizeof(out), &out_len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_hide_password((const uint8_t *)SECRET, 9U,
						   NULL, "p", out, sizeof(out),
						   &out_len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_hide_password((const uint8_t *)SECRET, 9U,
						   REQ_AUTH, NULL, out,
						   sizeof(out), &out_len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_hide_password((const uint8_t *)SECRET, 9U,
						   REQ_AUTH, "p", NULL,
						   sizeof(out), &out_len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_hide_password((const uint8_t *)SECRET, 9U,
						   REQ_AUTH, "p", out,
						   sizeof(out), NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_hide_password((const uint8_t *)SECRET, 9U,
						   REQ_AUTH, big, out,
						   sizeof(out), &out_len));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      radius_hide_password((const uint8_t *)SECRET, 9U,
						   REQ_AUTH, "p", out, 8U,
						   &out_len));
}

/* ------------------------------------------------------------------------- */
/* response authenticator                                                    */
/* ------------------------------------------------------------------------- */

static void test_response_auth_rfc2865_example(void)
{
	uint8_t out[RADIUS_AUTH_LEN];

	TEST_ASSERT_EQUAL_INT(0, radius_response_auth(v_accept,
						      sizeof(v_accept), REQ_AUTH,
						      (const uint8_t *)SECRET,
						      strlen(SECRET), out));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(&v_accept[4], out, RADIUS_AUTH_LEN);

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_response_auth(NULL, 20U, REQ_AUTH,
						   (const uint8_t *)SECRET, 9U,
						   out));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_response_auth(v_accept, 19U, REQ_AUTH,
						   (const uint8_t *)SECRET, 9U,
						   out));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_response_auth(v_accept, 20U, NULL,
						   (const uint8_t *)SECRET, 9U,
						   out));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_response_auth(v_accept, 20U, REQ_AUTH, NULL,
						   9U, out));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_response_auth(v_accept, 20U, REQ_AUTH,
						   (const uint8_t *)SECRET, 9U,
						   NULL));
}

/* ------------------------------------------------------------------------- */
/* attribute walking                                                         */
/* ------------------------------------------------------------------------- */

static void test_attr_find(void)
{
	const uint8_t *v = NULL;
	size_t vl = 0U;

	TEST_ASSERT_EQUAL_INT(0, radius_attr_find(v_request, sizeof(v_request),
						  RADIUS_AT_USER_NAME, 0U, &v,
						  &vl));
	TEST_ASSERT_EQUAL_UINT(4U, vl);
	TEST_ASSERT_EQUAL_MEMORY("nemo", v, 4U);

	TEST_ASSERT_EQUAL_INT(0, radius_attr_find(v_request, sizeof(v_request),
						  RADIUS_AT_NAS_PORT, 0U, &v,
						  &vl));
	TEST_ASSERT_EQUAL_UINT(4U, vl);
	TEST_ASSERT_EQUAL_HEX8(3U, v[3]);

	/* Skipping past the only instance. */
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      radius_attr_find(v_request, sizeof(v_request),
					       RADIUS_AT_USER_NAME, 1U, &v,
					       &vl));
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      radius_attr_find(v_request, sizeof(v_request),
					       RADIUS_AT_STATE, 0U, &v, &vl));

	TEST_ASSERT_EQUAL_INT(0, radius_attrs_check(v_request,
						    sizeof(v_request)));
	TEST_ASSERT_EQUAL_INT(0, radius_attrs_check(v_accept, sizeof(v_accept)));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_attr_find(NULL, 20U, 1U, 0U, &v, &vl));
	TEST_ASSERT_EQUAL_INT(-EINVAL, radius_attr_find(v_request, 19U, 1U, 0U,
							&v, &vl));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_attr_find(v_request, sizeof(v_request), 1U,
					       0U, NULL, &vl));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_attr_find(v_request, sizeof(v_request), 1U,
					       0U, &v, NULL));
}

static void test_attr_find_rejects_malformed_lists(void)
{
	uint8_t pkt[64];
	const uint8_t *v = NULL;
	size_t vl = 0U;

	memset(pkt, 0, sizeof(pkt));
	pkt[0] = RADIUS_CODE_ACCESS_ACCEPT;

	/* A length octet of 0 would be an infinite loop in a naive walker. */
	pkt[20] = 0x06U;
	pkt[21] = 0x00U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      radius_attr_find(pkt, 24U, 6U, 0U, &v, &vl));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, radius_attrs_check(pkt, 24U));

	/* A length of 1 is below the two header octets. */
	pkt[21] = 0x01U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      radius_attr_find(pkt, 24U, 6U, 0U, &v, &vl));

	/* An attribute that runs past the packet. */
	pkt[21] = 0x40U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      radius_attr_find(pkt, 24U, 6U, 0U, &v, &vl));

	/* A single trailing octet cannot be an attribute header. */
	pkt[20] = 0x06U;
	pkt[21] = 0x02U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      radius_attr_find(pkt, 23U, 99U, 0U, &v, &vl));
}

/* ------------------------------------------------------------------------- */
/* role mapping                                                              */
/* ------------------------------------------------------------------------- */

/** Build a minimal Access-Accept carrying @p attrs. */
static size_t make_accept(uint8_t *pkt, const uint8_t *attrs, size_t n,
			  uint8_t id, const uint8_t *req_auth)
{
	size_t len = RADIUS_HDR_LEN + n;

	pkt[0] = RADIUS_CODE_ACCESS_ACCEPT;
	pkt[1] = id;
	pkt[2] = (uint8_t)((len >> 8) & 0xFFU);
	pkt[3] = (uint8_t)(len & 0xFFU);
	memset(&pkt[4], 0, RADIUS_AUTH_LEN);
	if (n != 0U) {
		memcpy(&pkt[RADIUS_HDR_LEN], attrs, n);
	}
	(void)radius_response_auth(pkt, len, req_auth,
				   (const uint8_t *)SECRET, strlen(SECRET),
				   &pkt[4]);
	return len;
}

static void test_role_from_service_type(void)
{
	static const uint8_t admin[] = { 0x06, 0x06, 0, 0, 0,
					 RADIUS_ST_ADMINISTRATIVE };
	static const uint8_t oper[] = { 0x06, 0x06, 0, 0, 0,
					RADIUS_ST_NAS_PROMPT };
	static const uint8_t view[] = { 0x06, 0x06, 0, 0, 0, RADIUS_ST_LOGIN };
	static const uint8_t framed[] = { 0x06, 0x06, 0, 0, 0,
					  RADIUS_ST_FRAMED };
	static const uint8_t other[] = { 0x06, 0x06, 0, 0, 0, 99 };
	uint8_t pkt[64];
	size_t len;
	bool ex = false;

	len = make_accept(pkt, admin, sizeof(admin), 0U, REQ_AUTH);
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_ADMIN,
				radius_role_from_attrs(pkt, len, &ex));
	TEST_ASSERT_TRUE(ex);

	len = make_accept(pkt, oper, sizeof(oper), 0U, REQ_AUTH);
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_OPERATOR,
				radius_role_from_attrs(pkt, len, &ex));
	TEST_ASSERT_TRUE(ex);

	len = make_accept(pkt, view, sizeof(view), 0U, REQ_AUTH);
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER,
				radius_role_from_attrs(pkt, len, &ex));
	TEST_ASSERT_TRUE(ex);

	len = make_accept(pkt, framed, sizeof(framed), 0U, REQ_AUTH);
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER,
				radius_role_from_attrs(pkt, len, &ex));

	/* An unmapped Service-Type falls back to least privilege, and says so. */
	len = make_accept(pkt, other, sizeof(other), 0U, REQ_AUTH);
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER,
				radius_role_from_attrs(pkt, len, &ex));
	TEST_ASSERT_FALSE(ex);

	/* No attributes at all: viewer, not explicit. */
	len = make_accept(pkt, NULL, 0U, 0U, REQ_AUTH);
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER,
				radius_role_from_attrs(pkt, len, NULL));
}

static void test_role_from_filter_id_wins(void)
{
	/* Filter-Id "operator" alongside Service-Type Login: the string wins. */
	static const uint8_t attrs[] = {
		0x06, 0x06, 0, 0, 0, RADIUS_ST_LOGIN,
		0x0b, 0x0a, 'o', 'p', 'e', 'r', 'a', 't', 'o', 'r'
	};
	/* Two Filter-Ids, only the second of which names a role. */
	static const uint8_t attrs2[] = {
		0x0b, 0x08, 'v', 'l', 'a', 'n', '-', '9',
		0x0b, 0x07, 'a', 'd', 'm', 'i', 'n'
	};
	/* A Filter-Id longer than the scratch buffer must be skipped, not
	 * truncated into a role name. */
	static const uint8_t attrs3[] = {
		0x0b, 0x24, 'a', 'd', 'm', 'i', 'n', 'x', 'x', 'x', 'x', 'x',
		'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
		'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
		0x06, 0x06, 0, 0, 0, RADIUS_ST_ADMINISTRATIVE
	};
	/* A zero-length Filter-Id is skipped. */
	static const uint8_t attrs4[] = {
		0x0b, 0x02,
		0x06, 0x06, 0, 0, 0, RADIUS_ST_NAS_PROMPT
	};
	uint8_t pkt[128];
	size_t len;

	len = make_accept(pkt, attrs, sizeof(attrs), 0U, REQ_AUTH);
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_OPERATOR,
				radius_role_from_attrs(pkt, len, NULL));

	len = make_accept(pkt, attrs2, sizeof(attrs2), 0U, REQ_AUTH);
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_ADMIN,
				radius_role_from_attrs(pkt, len, NULL));

	len = make_accept(pkt, attrs3, sizeof(attrs3), 0U, REQ_AUTH);
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_ADMIN,
				radius_role_from_attrs(pkt, len, NULL));

	len = make_accept(pkt, attrs4, sizeof(attrs4), 0U, REQ_AUTH);
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_OPERATOR,
				radius_role_from_attrs(pkt, len, NULL));

	/* A malformed list yields the safe default rather than a crash. */
	memset(pkt, 0, sizeof(pkt));
	pkt[0] = RADIUS_CODE_ACCESS_ACCEPT;
	pkt[20] = 0x0bU;
	pkt[21] = 0x00U;
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER,
				radius_role_from_attrs(pkt, 24U, NULL));
}

/* ------------------------------------------------------------------------- */
/* request building                                                          */
/* ------------------------------------------------------------------------- */

static radius_req_t base_req(void)
{
	radius_req_t r;

	memset(&r, 0, sizeof(r));
	r.user = "nemo";
	r.secret = "arctangent";
	r.shared = (const uint8_t *)SECRET;
	r.shared_len = strlen(SECRET);
	r.id = 0U;
	r.authenticator = REQ_AUTH;
	return r;
}

static void test_build_reproduces_the_rfc_example(void)
{
	radius_req_t r = base_req();
	uint8_t out[RADIUS_BUF_MAX];
	size_t len = 0U;

	/* The example also carries NAS-IP-Address and NAS-Port. */
	r.nas_ip = 0xC0A80110U;
	r.nas_port = 3U;
	r.nas_port_set = true;

	TEST_ASSERT_EQUAL_INT(0, radius_build_access_request(&r, out,
							     sizeof(out), &len));
	TEST_ASSERT_EQUAL_UINT(sizeof(v_request), len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(v_request, out, sizeof(v_request));
}

static void test_build_optional_attributes(void)
{
	radius_req_t r = base_req();
	uint8_t out[RADIUS_BUF_MAX];
	uint8_t state[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
	const uint8_t *v = NULL;
	size_t vl = 0U;
	size_t len = 0U;

	r.nas_id = "meridian";
	r.nas_port_type = RADIUS_NPT_VIRTUAL;
	r.service_type = RADIUS_ST_ADMINISTRATIVE;
	r.state = state;
	r.state_len = sizeof(state);
	r.message_authenticator = true;

	TEST_ASSERT_EQUAL_INT(0, radius_build_access_request(&r, out,
							     sizeof(out), &len));
	TEST_ASSERT_EQUAL_INT(0, radius_attrs_check(out, len));
	TEST_ASSERT_EQUAL_UINT(len, ((size_t)out[2] << 8) | out[3]);

	TEST_ASSERT_EQUAL_INT(0, radius_attr_find(out, len,
						  RADIUS_AT_NAS_IDENTIFIER, 0U,
						  &v, &vl));
	TEST_ASSERT_EQUAL_MEMORY("meridian", v, 8U);
	TEST_ASSERT_EQUAL_INT(0, radius_attr_find(out, len,
						  RADIUS_AT_NAS_PORT_TYPE, 0U,
						  &v, &vl));
	TEST_ASSERT_EQUAL_HEX8(RADIUS_NPT_VIRTUAL, v[3]);
	TEST_ASSERT_EQUAL_INT(0, radius_attr_find(out, len, RADIUS_AT_STATE, 0U,
						  &v, &vl));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(state, v, sizeof(state));

	/* RFC 3579 §3.2: the Message-Authenticator is HMAC-MD5 over the whole
	 * datagram with its own value zeroed. Recompute it here independently. */
	TEST_ASSERT_EQUAL_INT(0, radius_attr_find(out, len,
						  RADIUS_AT_MESSAGE_AUTHENTICATOR,
						  0U, &v, &vl));
	TEST_ASSERT_EQUAL_UINT(16U, vl);
	{
		uint8_t copy[RADIUS_BUF_MAX];
		uint8_t mac[AUTH_MD5_LEN];
		size_t off = (size_t)(v - out);

		memcpy(copy, out, len);
		memset(&copy[off], 0, 16U);
		auth_hmac_md5((const uint8_t *)SECRET, strlen(SECRET), copy, len,
			      mac);
		TEST_ASSERT_EQUAL_HEX8_ARRAY(mac, v, sizeof(mac));
	}
}

static void test_build_guards(void)
{
	radius_req_t r;
	uint8_t out[RADIUS_BUF_MAX];
	uint8_t state[RADIUS_STATE_MAX + 1U];
	char big_user[AUTH_USER_MAX + 2U];
	size_t len = 0U;

	memset(state, 0, sizeof(state));
	memset(big_user, 'u', sizeof(big_user) - 1U);
	big_user[sizeof(big_user) - 1U] = '\0';

	TEST_ASSERT_EQUAL_INT(-EINVAL, radius_build_access_request(NULL, out,
								   sizeof(out),
								   &len));
	r = base_req();
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_build_access_request(&r, NULL, sizeof(out),
							  &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_build_access_request(&r, out, sizeof(out),
							  NULL));

	r = base_req();
	r.user = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_build_access_request(&r, out, sizeof(out),
							  &len));
	r = base_req();
	r.user = "";
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_build_access_request(&r, out, sizeof(out),
							  &len));
	r = base_req();
	r.user = big_user;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_build_access_request(&r, out, sizeof(out),
							  &len));
	r = base_req();
	r.secret = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_build_access_request(&r, out, sizeof(out),
							  &len));
	r = base_req();
	r.shared = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_build_access_request(&r, out, sizeof(out),
							  &len));
	r = base_req();
	r.shared_len = 0U;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_build_access_request(&r, out, sizeof(out),
							  &len));
	r = base_req();
	r.authenticator = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_build_access_request(&r, out, sizeof(out),
							  &len));
	r = base_req();
	r.state = state;
	r.state_len = sizeof(state);
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_build_access_request(&r, out, sizeof(out),
							  &len));
	r = base_req();
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      radius_build_access_request(&r, out, 19U, &len));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      radius_build_access_request(&r, out,
							  RADIUS_BUF_MAX + 1U,
							  &len));
	/* Just enough room for the header but not the attributes. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      radius_build_access_request(&r, out, 22U, &len));
}

static void test_build_capacity_edges(void)
{
	radius_req_t r;
	uint8_t out[RADIUS_BUF_MAX];
	size_t len = 0U;
	size_t cap;

	/* Walk the capacity up one octet at a time: every prefix must either
	 * fail with -ENOSPC or produce a well-formed datagram. Nothing in
	 * between, and never a buffer overrun (ASan is the assertion). */
	for (cap = 20U; cap < 120U; cap++) {
		int rc;

		r = base_req();
		r.nas_id = "meridian-observatory";
		r.nas_ip = 0x0A000001U;
		r.nas_port = 1U;
		r.nas_port_set = true;
		r.service_type = RADIUS_ST_ADMINISTRATIVE;
		r.nas_port_type = RADIUS_NPT_VIRTUAL;
		r.message_authenticator = true;

		rc = radius_build_access_request(&r, out, cap, &len);
		if (rc == 0) {
			TEST_ASSERT_TRUE(len <= cap);
			TEST_ASSERT_EQUAL_INT(0, radius_attrs_check(out, len));
		} else {
			TEST_ASSERT_EQUAL_INT(-ENOSPC, rc);
		}
	}
}

/* ------------------------------------------------------------------------- */
/* response parsing                                                          */
/* ------------------------------------------------------------------------- */

static void test_parse_accept_from_the_rfc(void)
{
	radius_rsp_t rsp;

	TEST_ASSERT_EQUAL_INT(0, radius_parse_response(v_accept,
						       sizeof(v_accept),
						       (const uint8_t *)SECRET,
						       strlen(SECRET), 0U,
						       REQ_AUTH, &rsp));
	TEST_ASSERT_EQUAL_HEX8(RADIUS_CODE_ACCESS_ACCEPT, rsp.code);
	TEST_ASSERT_EQUAL_HEX8(0U, rsp.id);
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_VIEWER, rsp.role);
	TEST_ASSERT_TRUE(rsp.role_explicit);
	TEST_ASSERT_FALSE(rsp.have_msg_auth);
	TEST_ASSERT_EQUAL_UINT(0U, rsp.state_len);
	TEST_ASSERT_EQUAL_STRING("", rsp.reply);
}

static void test_parse_rejects_a_forged_reply(void)
{
	uint8_t pkt[64];
	radius_rsp_t rsp;
	size_t len;

	len = make_accept(pkt, NULL, 0U, 7U, REQ_AUTH);
	TEST_ASSERT_EQUAL_INT(0, radius_parse_response(pkt, len,
						       (const uint8_t *)SECRET,
						       strlen(SECRET), 7U,
						       REQ_AUTH, &rsp));

	/* One flipped bit in the authenticator, and it is not our server. */
	pkt[10] ^= 0x01U;
	TEST_ASSERT_EQUAL_INT(-EBADE,
			      radius_parse_response(pkt, len,
						    (const uint8_t *)SECRET,
						    strlen(SECRET), 7U, REQ_AUTH,
						    &rsp));
	pkt[10] ^= 0x01U;

	/* The wrong shared secret is indistinguishable from a forgery. */
	TEST_ASSERT_EQUAL_INT(-EBADE,
			      radius_parse_response(pkt, len,
						    (const uint8_t *)"wrong", 5U,
						    7U, REQ_AUTH, &rsp));

	/* A reply to somebody else's request. */
	TEST_ASSERT_EQUAL_INT(-ENOMSG,
			      radius_parse_response(pkt, len,
						    (const uint8_t *)SECRET,
						    strlen(SECRET), 8U, REQ_AUTH,
						    &rsp));
}

static void test_parse_reject_challenge_and_reply_message(void)
{
	static const uint8_t attrs[] = {
		/* Reply-Message with a control character to be sanitised. */
		0x12, 0x08, 'd', 'e', 'n', 'i', 'e', 0x07,
		/* State to echo in the next round. */
		0x18, 0x06, 0xDE, 0xAD, 0xBE, 0xEF
	};
	uint8_t pkt[64];
	radius_rsp_t rsp;
	size_t len;

	len = make_accept(pkt, attrs, sizeof(attrs), 3U, REQ_AUTH);
	pkt[0] = RADIUS_CODE_ACCESS_REJECT;
	(void)radius_response_auth(pkt, len, REQ_AUTH,
				   (const uint8_t *)SECRET, strlen(SECRET),
				   &pkt[4]);
	TEST_ASSERT_EQUAL_INT(0, radius_parse_response(pkt, len,
						       (const uint8_t *)SECRET,
						       strlen(SECRET), 3U,
						       REQ_AUTH, &rsp));
	TEST_ASSERT_EQUAL_HEX8(RADIUS_CODE_ACCESS_REJECT, rsp.code);
	TEST_ASSERT_EQUAL_STRING("denie.", rsp.reply);
	/* State is only kept for a challenge — a reject has nothing to resume. */
	TEST_ASSERT_EQUAL_UINT(0U, rsp.state_len);
	/* And a rejected request grants no role. */
	TEST_ASSERT_EQUAL_UINT8(AUTH_ROLE_NONE, rsp.role);

	pkt[0] = RADIUS_CODE_ACCESS_CHALLENGE;
	(void)radius_response_auth(pkt, len, REQ_AUTH,
				   (const uint8_t *)SECRET, strlen(SECRET),
				   &pkt[4]);
	TEST_ASSERT_EQUAL_INT(0, radius_parse_response(pkt, len,
						       (const uint8_t *)SECRET,
						       strlen(SECRET), 3U,
						       REQ_AUTH, &rsp));
	TEST_ASSERT_EQUAL_UINT(4U, rsp.state_len);
	TEST_ASSERT_EQUAL_HEX8(0xDEU, rsp.state[0]);

	/* An unexpected code. */
	pkt[0] = 42U;
	(void)radius_response_auth(pkt, len, REQ_AUTH,
				   (const uint8_t *)SECRET, strlen(SECRET),
				   &pkt[4]);
	TEST_ASSERT_EQUAL_INT(-EPROTO,
			      radius_parse_response(pkt, len,
						    (const uint8_t *)SECRET,
						    strlen(SECRET), 3U, REQ_AUTH,
						    &rsp));
}

static void test_parse_message_authenticator(void)
{
	uint8_t attrs[18];
	uint8_t pkt[64];
	radius_rsp_t rsp;
	uint8_t mac[AUTH_MD5_LEN];
	uint8_t copy[64];
	size_t len;

	/* A reply carrying a correct Message-Authenticator. */
	memset(attrs, 0, sizeof(attrs));
	attrs[0] = RADIUS_AT_MESSAGE_AUTHENTICATOR;
	attrs[1] = 18U;
	len = make_accept(pkt, attrs, sizeof(attrs), 9U, REQ_AUTH);
	memcpy(copy, pkt, len);
	memcpy(&copy[4], REQ_AUTH, RADIUS_AUTH_LEN);
	auth_hmac_md5((const uint8_t *)SECRET, strlen(SECRET), copy, len, mac);
	memcpy(&pkt[RADIUS_HDR_LEN + 2U], mac, sizeof(mac));
	(void)radius_response_auth(pkt, len, REQ_AUTH,
				   (const uint8_t *)SECRET, strlen(SECRET),
				   &pkt[4]);

	TEST_ASSERT_EQUAL_INT(0, radius_parse_response(pkt, len,
						       (const uint8_t *)SECRET,
						       strlen(SECRET), 9U,
						       REQ_AUTH, &rsp));
	TEST_ASSERT_TRUE(rsp.have_msg_auth);

	/* Corrupt it: the reply is now a forgery. */
	pkt[RADIUS_HDR_LEN + 2U] ^= 0x80U;
	(void)radius_response_auth(pkt, len, REQ_AUTH,
				   (const uint8_t *)SECRET, strlen(SECRET),
				   &pkt[4]);
	TEST_ASSERT_EQUAL_INT(-EBADE,
			      radius_parse_response(pkt, len,
						    (const uint8_t *)SECRET,
						    strlen(SECRET), 9U, REQ_AUTH,
						    &rsp));

	/* A Message-Authenticator of the wrong length is malformed. */
	attrs[1] = 10U;
	len = make_accept(pkt, attrs, 10U, 9U, REQ_AUTH);
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      radius_parse_response(pkt, len,
						    (const uint8_t *)SECRET,
						    strlen(SECRET), 9U, REQ_AUTH,
						    &rsp));
}

static void test_parse_length_field_rules(void)
{
	uint8_t pkt[64];
	radius_rsp_t rsp;
	size_t len;

	/* RFC 2865 §3: octets past Length are ignored. */
	len = make_accept(pkt, NULL, 0U, 5U, REQ_AUTH);
	memset(&pkt[len], 0xFF, 8U);
	TEST_ASSERT_EQUAL_INT(0, radius_parse_response(pkt, len + 8U,
						       (const uint8_t *)SECRET,
						       strlen(SECRET), 5U,
						       REQ_AUTH, &rsp));

	/* A Length longer than what arrived is malformed. */
	pkt[3] = (uint8_t)(len + 8U);
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      radius_parse_response(pkt, len,
						    (const uint8_t *)SECRET,
						    strlen(SECRET), 5U, REQ_AUTH,
						    &rsp));
	/* And so is one below the header size. */
	pkt[3] = 19U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      radius_parse_response(pkt, len,
						    (const uint8_t *)SECRET,
						    strlen(SECRET), 5U, REQ_AUTH,
						    &rsp));
}

static void test_parse_guards(void)
{
	radius_rsp_t rsp;
	uint8_t big[RADIUS_BUF_MAX + 1U];

	memset(big, 0, sizeof(big));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_parse_response(NULL, 20U,
						    (const uint8_t *)SECRET, 9U,
						    0U, REQ_AUTH, &rsp));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_parse_response(v_accept, sizeof(v_accept),
						    NULL, 9U, 0U, REQ_AUTH,
						    &rsp));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_parse_response(v_accept, sizeof(v_accept),
						    (const uint8_t *)SECRET, 0U,
						    0U, REQ_AUTH, &rsp));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_parse_response(v_accept, sizeof(v_accept),
						    (const uint8_t *)SECRET, 9U,
						    0U, NULL, &rsp));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      radius_parse_response(v_accept, sizeof(v_accept),
						    (const uint8_t *)SECRET, 9U,
						    0U, REQ_AUTH, NULL));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      radius_parse_response(v_accept, 19U,
						    (const uint8_t *)SECRET, 9U,
						    0U, REQ_AUTH, &rsp));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      radius_parse_response(big, sizeof(big),
						    (const uint8_t *)SECRET, 9U,
						    0U, REQ_AUTH, &rsp));
}

/* ------------------------------------------------------------------------- */
/* hostile input                                                             */
/* ------------------------------------------------------------------------- */

static void test_random_datagrams_never_crash(void)
{
	test_rng_t g;
	uint8_t pkt[300];
	radius_rsp_t rsp;
	unsigned int i;

	test_rng_init(&g, 0x2865U);
	for (i = 0U; i < 30000U; i++) {
		size_t n = 1U + test_rng_below(&g, sizeof(pkt));
		int rc;

		test_rng_fill(&g, pkt, n);
		rc = radius_parse_response(pkt, n, (const uint8_t *)SECRET,
					   strlen(SECRET), pkt[1], REQ_AUTH,
					   &rsp);
		/* A random datagram cannot be authentic; the only interesting
		 * property is that a "success" is never returned on garbage. */
		TEST_ASSERT_TRUE(rc != 0);
		(void)radius_attrs_check(pkt, n);
		(void)radius_role_from_attrs(pkt, n, NULL);
	}
}

static void test_mutated_valid_replies_never_crash(void)
{
	test_rng_t g;
	uint8_t pkt[128];
	radius_rsp_t rsp;
	size_t len;
	unsigned int i;

	static const uint8_t attrs[] = {
		0x06, 0x06, 0, 0, 0, RADIUS_ST_ADMINISTRATIVE,
		0x0b, 0x07, 'a', 'd', 'm', 'i', 'n',
		0x12, 0x06, 'h', 'e', 'l', 'o',
		0x18, 0x06, 1, 2, 3, 4
	};

	test_rng_init(&g, 0x3579U);
	for (i = 0U; i < 20000U; i++) {
		size_t pos;

		len = make_accept(pkt, attrs, sizeof(attrs), 1U, REQ_AUTH);
		pos = 1U + test_rng_below(&g, len - 1U);
		pkt[pos] ^= (uint8_t)(1U << test_rng_below(&g, 8U));
		/* Re-seal so the authenticator check does not mask the parse. */
		(void)radius_response_auth(pkt, len, REQ_AUTH,
					   (const uint8_t *)SECRET,
					   strlen(SECRET), &pkt[4]);
		(void)radius_parse_response(pkt, len, (const uint8_t *)SECRET,
					    strlen(SECRET), pkt[1], REQ_AUTH,
					    &rsp);
	}
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_md5_rfc1321_suite);
	RUN_TEST(test_md5_streaming_matches_one_shot);
	RUN_TEST(test_hmac_md5_rfc2202);

	RUN_TEST(test_hide_password_rfc2865_example);
	RUN_TEST(test_hide_password_chaining_and_padding);
	RUN_TEST(test_hide_password_guards);
	RUN_TEST(test_response_auth_rfc2865_example);

	RUN_TEST(test_attr_find);
	RUN_TEST(test_attr_find_rejects_malformed_lists);

	RUN_TEST(test_role_from_service_type);
	RUN_TEST(test_role_from_filter_id_wins);

	RUN_TEST(test_build_reproduces_the_rfc_example);
	RUN_TEST(test_build_optional_attributes);
	RUN_TEST(test_build_guards);
	RUN_TEST(test_build_capacity_edges);

	RUN_TEST(test_parse_accept_from_the_rfc);
	RUN_TEST(test_parse_rejects_a_forged_reply);
	RUN_TEST(test_parse_reject_challenge_and_reply_message);
	RUN_TEST(test_parse_message_authenticator);
	RUN_TEST(test_parse_length_field_rules);
	RUN_TEST(test_parse_guards);

	RUN_TEST(test_random_datagrams_never_crash);
	RUN_TEST(test_mutated_valid_replies_never_crash);

	return UNITY_END();
}
