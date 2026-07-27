/*
 * STS1000 "Meridian" — core/nts/aes_siv unit tests.
 *
 * Provenance of every expectation (ARCHITECTURE.md §9 wants published vectors,
 * not self-round-trips). Each was additionally re-derived on the build host
 * with OpenSSL 3.0 — an implementation entirely independent of this one — and
 * agreed byte for byte:
 *
 *   AES-ECB      FIPS-197 §C.1 (AES-128) and §C.3 (AES-256) single-block KATs,
 *                plus the four-block AES-128 sequence from NIST SP 800-38A
 *                §F.1.1, which catches a key schedule that is right for one
 *                block and wrong afterwards.
 *   SHA-256      FIPS-180-4 §B / the classic NIST examples: "", "abc" and the
 *                56-octet two-block message.
 *   HMAC-SHA-256 RFC 4231 test cases 1, 2, 3, 6 and 7 — 6 and 7 are the
 *                over-length-key cases that exercise the key-hashing branch.
 *   AES-CMAC     RFC 4493 §4 examples 1–4 (0, 16, 40 and 64 octets), which
 *                cover the empty message, an exact block, and both the K1
 *                (complete final block) and K2 (padded final block) paths.
 *   AES-SIV      RFC 5297 Appendix A.1 (deterministic, one AD) and A.2
 *                (nonce-based, two ADs plus a nonce), byte-exact including the
 *                synthetic IV.
 *
 * The primitives are tested first and hardest on purpose: everything the NTS
 * cookie and NTP MAC suites conclude rests on the fixture being correct. AES
 * lives in support/host_aes.c and the hashes in its sibling support/
 * host_sha256.c; both are pinned here because both are load-bearing for this
 * module and for core/ntp's symmetric MAC, whoever owns the file.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "host_aes.h"
#include "host_sha256.h"
#include "nts/aes_siv.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

static host_crypto_t g_hc;
static port_crypto_t g_port;

void setUp(void)
{
	host_crypto_init(&g_hc, 0xC0FFEEU);
	g_port = host_crypto_port(&g_hc);
}

void tearDown(void)
{
}

/* --------------------------------------------------------------- AES-ECB */

static void test_aes128_fips197_c1(void)
{
	static const uint8_t key[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
					 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
					 0x0c, 0x0d, 0x0e, 0x0f };
	static const uint8_t pt[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55,
					0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb,
					0xcc, 0xdd, 0xee, 0xff };
	static const uint8_t want[16] = { 0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b,
					  0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80,
					  0x70, 0xb4, 0xc5, 0x5a };
	uint8_t out[16];

	TEST_ASSERT_EQUAL_INT(0, host_aes_ecb_encrypt(key, sizeof(key), pt, out,
						      sizeof(pt)));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want, out, sizeof(want));
}

static void test_aes256_fips197_c3(void)
{
	static const uint8_t key[32] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a,
		0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
		0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f
	};
	static const uint8_t pt[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55,
					0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb,
					0xcc, 0xdd, 0xee, 0xff };
	static const uint8_t want[16] = { 0x8e, 0xa2, 0xb7, 0xca, 0x51, 0x67,
					  0x45, 0xbf, 0xea, 0xfc, 0x49, 0x90,
					  0x4b, 0x49, 0x60, 0x89 };
	uint8_t out[16];

	TEST_ASSERT_EQUAL_INT(0, host_aes_ecb_encrypt(key, sizeof(key), pt, out,
						      sizeof(pt)));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want, out, sizeof(want));
}

/* NIST SP 800-38A §F.1.1, four blocks under one key. */
static const uint8_t sp38a_key[16] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae,
				       0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88,
				       0x09, 0xcf, 0x4f, 0x3c };
static const uint8_t sp38a_pt[64] = {
	0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11,
	0x73, 0x93, 0x17, 0x2a, 0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c,
	0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51, 0x30, 0xc8, 0x1c, 0x46,
	0xa3, 0x5c, 0xe4, 0x11, 0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
	0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17, 0xad, 0x2b, 0x41, 0x7b,
	0xe6, 0x6c, 0x37, 0x10
};

static void test_aes128_multiblock_sp800_38a(void)
{
	static const uint8_t want[64] = {
		0x3a, 0xd7, 0x7b, 0xb4, 0x0d, 0x7a, 0x36, 0x60, 0xa8, 0x9e, 0xca,
		0xf3, 0x24, 0x66, 0xef, 0x97, 0xf5, 0xd3, 0xd5, 0x85, 0x03, 0xb9,
		0x69, 0x9d, 0xe7, 0x85, 0x89, 0x5a, 0x96, 0xfd, 0xba, 0xaf, 0x43,
		0xb1, 0xcd, 0x7f, 0x59, 0x8e, 0xce, 0x23, 0x88, 0x1b, 0x00, 0xe3,
		0xed, 0x03, 0x06, 0x88, 0x7b, 0x0c, 0x78, 0x5e, 0x27, 0xe8, 0xad,
		0x3f, 0x82, 0x23, 0x20, 0x71, 0x04, 0x72, 0x5d, 0xd4
	};
	uint8_t buf[64];

	memcpy(buf, sp38a_pt, sizeof(buf));
	/* In place: the port contract allows in and out to alias. */
	TEST_ASSERT_EQUAL_INT(0, host_aes_ecb_encrypt(sp38a_key, sizeof(sp38a_key),
						      buf, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want, buf, sizeof(want));
}

static void test_aes_bad_arguments(void)
{
	uint8_t buf[16] = { 0 };

	TEST_ASSERT_EQUAL_INT(-1, host_aes_ecb_encrypt(sp38a_key, 24U, buf, buf, 16U));
	TEST_ASSERT_EQUAL_INT(-1, host_aes_ecb_encrypt(sp38a_key, 16U, buf, buf, 15U));
	TEST_ASSERT_EQUAL_INT(-1, host_aes_ecb_encrypt(NULL, 16U, buf, buf, 16U));
	TEST_ASSERT_EQUAL_INT(-1, host_aes_ecb_encrypt(sp38a_key, 16U, NULL, buf, 16U));
	/* Zero length is a legal no-op. */
	TEST_ASSERT_EQUAL_INT(0, host_aes_ecb_encrypt(sp38a_key, 16U, NULL, NULL, 0U));
}

/* --------------------------------------------------------------- SHA-256 */

static void expect_sha256(const char *msg, const uint8_t want[32])
{
	uint8_t out[32];

	host_sha256((const uint8_t *)msg, strlen(msg), out);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want, out, 32);
}

static void test_sha256_known_answers(void)
{
	static const uint8_t empty[32] = {
		0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14, 0x9a, 0xfb, 0xf4,
		0xc8, 0x99, 0x6f, 0xb9, 0x24, 0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b,
		0x93, 0x4c, 0xa4, 0x95, 0x99, 0x1b, 0x78, 0x52, 0xb8, 0x55
	};
	static const uint8_t abc[32] = {
		0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40,
		0xde, 0x5d, 0xae, 0x22, 0x23, 0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17,
		0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad
	};
	/* 56 octets: the length that forces the padding into a second block. */
	static const uint8_t two_block[32] = {
		0x24, 0x8d, 0x6a, 0x61, 0xd2, 0x06, 0x38, 0xb8, 0xe5, 0xc0, 0x26,
		0x93, 0x0c, 0x3e, 0x60, 0x39, 0xa3, 0x3c, 0xe4, 0x59, 0x64, 0xff,
		0x21, 0x67, 0xf6, 0xec, 0xed, 0xd4, 0x19, 0xdb, 0x06, 0xc1
	};

	expect_sha256("", empty);
	expect_sha256("abc", abc);
	expect_sha256("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
		      two_block);
}

/* ---------------------------------------------------------- HMAC-SHA-256 */

static void test_hmac_sha256_rfc4231(void)
{
	uint8_t key[131];
	uint8_t data[50];
	uint8_t out[32];

	/* TC1: 20 × 0x0b, "Hi There". */
	static const uint8_t tc1[32] = {
		0xb0, 0x34, 0x4c, 0x61, 0xd8, 0xdb, 0x38, 0x53, 0x5c, 0xa8, 0xaf,
		0xce, 0xaf, 0x0b, 0xf1, 0x2b, 0x88, 0x1d, 0xc2, 0x00, 0xc9, 0x83,
		0x3d, 0xa7, 0x26, 0xe9, 0x37, 0x6c, 0x2e, 0x32, 0xcf, 0xf7
	};
	/* TC2: key "Jefe" — shorter than the block, so the zero-pad path. */
	static const uint8_t tc2[32] = {
		0x5b, 0xdc, 0xc1, 0x46, 0xbf, 0x60, 0x75, 0x4e, 0x6a, 0x04, 0x24,
		0x26, 0x08, 0x95, 0x75, 0xc7, 0x5a, 0x00, 0x3f, 0x08, 0x9d, 0x27,
		0x39, 0x83, 0x9d, 0xec, 0x58, 0xb9, 0x64, 0xec, 0x38, 0x43
	};
	/* TC3: 20 × 0xaa key, 50 × 0xdd data — data spans blocks. */
	static const uint8_t tc3[32] = {
		0x77, 0x3e, 0xa9, 0x1e, 0x36, 0x80, 0x0e, 0x46, 0x85, 0x4d, 0xb8,
		0xeb, 0xd0, 0x91, 0x81, 0xa7, 0x29, 0x59, 0x09, 0x8b, 0x3e, 0xf8,
		0xc1, 0x22, 0xd9, 0x63, 0x55, 0x14, 0xce, 0xd5, 0x65, 0xfe
	};
	/* TC6: 131-octet key, hashed down first. */
	static const uint8_t tc6[32] = {
		0x60, 0xe4, 0x31, 0x59, 0x1e, 0xe0, 0xb6, 0x7f, 0x0d, 0x8a, 0x26,
		0xaa, 0xcb, 0xf5, 0xb7, 0x7f, 0x8e, 0x0b, 0xc6, 0x21, 0x37, 0x28,
		0xc5, 0x14, 0x05, 0x46, 0x04, 0x0f, 0x0e, 0xe3, 0x7f, 0x54
	};
	/* TC7: 131-octet key and a 152-octet message. */
	static const uint8_t tc7[32] = {
		0x9b, 0x09, 0xff, 0xa7, 0x1b, 0x94, 0x2f, 0xcb, 0x27, 0x63, 0x5f,
		0xbc, 0xd5, 0xb0, 0xe9, 0x44, 0xbf, 0xdc, 0x63, 0x64, 0x4f, 0x07,
		0x13, 0x93, 0x8a, 0x7f, 0x51, 0x53, 0x5c, 0x3a, 0x35, 0xe2
	};
	static const char *tc7_msg =
		"This is a test using a larger than block-size key and a larger "
		"than block-size data. The key needs to be hashed before being "
		"used by the HMAC algorithm.";

	memset(key, 0x0b, 20);
	host_hmac_sha256(key, 20U, (const uint8_t *)"Hi There", 8U, out);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(tc1, out, 32);

	host_hmac_sha256((const uint8_t *)"Jefe", 4U,
			 (const uint8_t *)"what do ya want for nothing?", 28U, out);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(tc2, out, 32);

	memset(key, 0xaa, 20);
	memset(data, 0xdd, sizeof(data));
	host_hmac_sha256(key, 20U, data, sizeof(data), out);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(tc3, out, 32);

	memset(key, 0xaa, sizeof(key));
	host_hmac_sha256(key, sizeof(key),
			 (const uint8_t *)"Test Using Larger Than Block-Size Key"
					  " - Hash Key First",
			 54U, out);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(tc6, out, 32);

	host_hmac_sha256(key, sizeof(key), (const uint8_t *)tc7_msg,
			 strlen(tc7_msg), out);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(tc7, out, 32);

	/* A zero-length key is legal and must not be confused with "no key". */
	host_hmac_sha256(NULL, 0U, (const uint8_t *)"x", 1U, out);
	host_hmac_sha256(key, 0U, (const uint8_t *)"x", 1U, key);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(out, key, 32);
}

/* -------------------------------------------------------------- AES-CMAC */

static void test_cmac_rfc4493(void)
{
	static const uint8_t want_empty[16] = { 0xbb, 0x1d, 0x69, 0x29, 0xe9,
						0x59, 0x37, 0x28, 0x7f, 0xa3,
						0x7d, 0x12, 0x9b, 0x75, 0x67,
						0x46 };
	static const uint8_t want_16[16] = { 0x07, 0x0a, 0x16, 0xb4, 0x6b, 0x4d,
					     0x41, 0x44, 0xf7, 0x9b, 0xdd, 0x9d,
					     0xd0, 0x4a, 0x28, 0x7c };
	static const uint8_t want_40[16] = { 0xdf, 0xa6, 0x67, 0x47, 0xde, 0x9a,
					     0xe6, 0x30, 0x30, 0xca, 0x32, 0x61,
					     0x14, 0x97, 0xc8, 0x27 };
	static const uint8_t want_64[16] = { 0x51, 0xf0, 0xbe, 0xbf, 0x7e, 0x3b,
					     0x9d, 0x92, 0xfc, 0x49, 0x74, 0x17,
					     0x79, 0x36, 0x3c, 0xfe };
	uint8_t out[16];

	TEST_ASSERT_EQUAL_INT(0, aes_cmac(&g_port, sp38a_key, NULL, 0U, out));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want_empty, out, 16);

	TEST_ASSERT_EQUAL_INT(0, aes_cmac(&g_port, sp38a_key, sp38a_pt, 16U, out));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want_16, out, 16);

	TEST_ASSERT_EQUAL_INT(0, aes_cmac(&g_port, sp38a_key, sp38a_pt, 40U, out));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want_40, out, 16);

	TEST_ASSERT_EQUAL_INT(0, aes_cmac(&g_port, sp38a_key, sp38a_pt, 64U, out));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want_64, out, 16);
}

static void test_cmac_bad_arguments(void)
{
	port_crypto_t empty;
	uint8_t out[16];

	memset(&empty, 0, sizeof(empty));

	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_cmac(NULL, sp38a_key, NULL, 0U, out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_cmac(&empty, sp38a_key, NULL, 0U, out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_cmac(&g_port, NULL, NULL, 0U, out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_cmac(&g_port, sp38a_key, NULL, 0U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_cmac(&g_port, sp38a_key, NULL, 1U, out));

	/* Port failure during subkey generation, then during absorption. */
	g_hc.fail_aes_in = 1U;
	TEST_ASSERT_EQUAL_INT(-EIO, aes_cmac(&g_port, sp38a_key, NULL, 0U, out));
	g_hc.fail_aes_in = 2U;
	TEST_ASSERT_EQUAL_INT(-EIO, aes_cmac(&g_port, sp38a_key, NULL, 0U, out));
}

/* --------------------------------------------------- RFC 5297 A.1 and A.2 */

/* A.1 — deterministic authenticated encryption, one AD component. */
static const uint8_t a1_key[32] = {
	0xff, 0xfe, 0xfd, 0xfc, 0xfb, 0xfa, 0xf9, 0xf8, 0xf7, 0xf6, 0xf5,
	0xf4, 0xf3, 0xf2, 0xf1, 0xf0, 0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5,
	0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff
};
static const uint8_t a1_ad[24] = { 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
				   0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b,
				   0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21,
				   0x22, 0x23, 0x24, 0x25, 0x26, 0x27 };
static const uint8_t a1_pt[14] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
				   0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee };
/* SIV || ciphertext. The first 16 octets are the synthetic IV the RFC prints
 * as "CMAC(final)"; the remainder is the CTR output. */
static const uint8_t a1_out[30] = {
	0x85, 0x63, 0x2d, 0x07, 0xc6, 0xe8, 0xf3, 0x7f, 0x95, 0x0a,
	0xcd, 0x32, 0x0a, 0x2e, 0xcc, 0x93, 0x40, 0xc0, 0x2b, 0x96,
	0x90, 0xc4, 0xdc, 0x04, 0xda, 0xef, 0x7f, 0x6a, 0xfe, 0x5c
};

/* A.2 — nonce-based, two AD components followed by the nonce. */
static const uint8_t a2_key[32] = {
	0x7f, 0x7e, 0x7d, 0x7c, 0x7b, 0x7a, 0x79, 0x78, 0x77, 0x76, 0x75,
	0x74, 0x73, 0x72, 0x71, 0x70, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
	0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f
};
static const uint8_t a2_ad1[40] = {
	0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb,
	0xcc, 0xdd, 0xee, 0xff, 0xde, 0xad, 0xda, 0xda, 0xde, 0xad, 0xda, 0xda,
	0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44,
	0x33, 0x22, 0x11, 0x00
};
static const uint8_t a2_ad2[10] = { 0x10, 0x20, 0x30, 0x40, 0x50,
				    0x60, 0x70, 0x80, 0x90, 0xa0 };
static const uint8_t a2_nonce[16] = { 0x09, 0xf9, 0x11, 0x02, 0x9d, 0x74,
				      0xe3, 0x5b, 0xd8, 0x41, 0x56, 0xc5,
				      0x63, 0x56, 0x88, 0xc0 };
static const char a2_pt[] = "this is some plaintext to encrypt using SIV-AES";
static const uint8_t a2_out[63] = {
	0x7b, 0xdb, 0x6e, 0x3b, 0x43, 0x26, 0x67, 0xeb, 0x06, 0xf4, 0xd1, 0x4b,
	0xff, 0x2f, 0xbd, 0x0f, 0xcb, 0x90, 0x0f, 0x2f, 0xdd, 0xbe, 0x40, 0x43,
	0x26, 0x60, 0x19, 0x65, 0xc8, 0x89, 0xbf, 0x17, 0xdb, 0xa7, 0x7c, 0xeb,
	0x09, 0x4f, 0xa6, 0x63, 0xb7, 0xa3, 0xf7, 0x48, 0xba, 0x8a, 0xf8, 0x29,
	0xea, 0x64, 0xad, 0x54, 0x4a, 0x27, 0x2e, 0x9c, 0x48, 0x5b, 0x62, 0xa3,
	0xfd, 0x5c, 0x0d
};

static void test_siv_rfc5297_a1(void)
{
	aes_siv_ctx_t c;
	aes_siv_ad_t ad[1] = { { a1_ad, sizeof(a1_ad) } };
	uint8_t out[64];
	uint8_t back[64];
	size_t n = 0U;
	size_t m = 0U;

	TEST_ASSERT_EQUAL_INT(0, aes_siv_init(&c, &g_port, a1_key, sizeof(a1_key)));
	TEST_ASSERT_EQUAL_INT(0, aes_siv_encrypt(&c, ad, 1U, a1_pt, sizeof(a1_pt),
						 out, sizeof(out), &n));
	TEST_ASSERT_EQUAL_size_t(sizeof(a1_out), n);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(a1_out, out, sizeof(a1_out));

	TEST_ASSERT_EQUAL_INT(0, aes_siv_decrypt(&c, ad, 1U, a1_out, sizeof(a1_out),
						 back, sizeof(back), &m));
	TEST_ASSERT_EQUAL_size_t(sizeof(a1_pt), m);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(a1_pt, back, sizeof(a1_pt));
}

static void test_siv_rfc5297_a2(void)
{
	aes_siv_ctx_t c;
	aes_siv_ad_t ad[3] = { { a2_ad1, sizeof(a2_ad1) },
			       { a2_ad2, sizeof(a2_ad2) },
			       { a2_nonce, sizeof(a2_nonce) } };
	const size_t pt_len = sizeof(a2_pt) - 1U; /* drop the terminator */
	uint8_t out[96];
	uint8_t back[96];
	size_t n = 0U;
	size_t m = 0U;

	TEST_ASSERT_EQUAL_size_t(47U, pt_len);
	TEST_ASSERT_EQUAL_INT(0, aes_siv_init(&c, &g_port, a2_key, sizeof(a2_key)));
	TEST_ASSERT_EQUAL_INT(0, aes_siv_encrypt(&c, ad, 3U,
						 (const uint8_t *)a2_pt, pt_len,
						 out, sizeof(out), &n));
	TEST_ASSERT_EQUAL_size_t(sizeof(a2_out), n);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(a2_out, out, sizeof(a2_out));

	TEST_ASSERT_EQUAL_INT(0, aes_siv_decrypt(&c, ad, 3U, a2_out, sizeof(a2_out),
						 back, sizeof(back), &m));
	TEST_ASSERT_EQUAL_size_t(pt_len, m);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(a2_pt, back, pt_len);
}

static void test_siv_rejects_tampering(void)
{
	aes_siv_ctx_t c;
	aes_siv_ad_t ad[1] = { { a1_ad, sizeof(a1_ad) } };
	uint8_t bad_ad[sizeof(a1_ad)];
	uint8_t buf[sizeof(a1_out)];
	uint8_t back[64];
	size_t m = 0U;

	TEST_ASSERT_EQUAL_INT(0, aes_siv_init(&c, &g_port, a1_key, sizeof(a1_key)));

	/* Every single-bit change anywhere in SIV || C must fail. Sampling one
	 * bit per octet keeps the test quick and still covers both halves. */
	for (size_t i = 0U; i < sizeof(a1_out); i++) {
		memcpy(buf, a1_out, sizeof(buf));
		buf[i] ^= (uint8_t)(1U << (i % 8U));
		TEST_ASSERT_EQUAL_INT(-EBADMSG,
				      aes_siv_decrypt(&c, ad, 1U, buf, sizeof(buf),
						      back, sizeof(back), &m));
	}

	/* A change in the associated data must fail too — that is the whole
	 * point of authenticating it. */
	memcpy(bad_ad, a1_ad, sizeof(bad_ad));
	bad_ad[0] ^= 0x01U;
	ad[0].p = bad_ad;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      aes_siv_decrypt(&c, ad, 1U, a1_out, sizeof(a1_out),
					      back, sizeof(back), &m));

	/* Dropping the AD entirely is a different message, not a shortcut. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      aes_siv_decrypt(&c, NULL, 0U, a1_out, sizeof(a1_out),
					      back, sizeof(back), &m));

	/* Truncation below the tag, and truncation of the ciphertext. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      aes_siv_decrypt(&c, ad, 1U, a1_out, 15U, back,
					      sizeof(back), &m));
	ad[0].p = a1_ad;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      aes_siv_decrypt(&c, ad, 1U, a1_out,
					      sizeof(a1_out) - 1U, back,
					      sizeof(back), &m));
}

static void test_siv_empty_and_aliased(void)
{
	aes_siv_ctx_t c;
	uint8_t buf[64];
	uint8_t back[64];
	size_t n = 0U;
	size_t m = 0U;

	TEST_ASSERT_EQUAL_INT(0, aes_siv_init(&c, &g_port, a1_key, sizeof(a1_key)));

	/* Empty plaintext, no associated data: the S2V degenerate case. The
	 * output is exactly the tag. */
	TEST_ASSERT_EQUAL_INT(0, aes_siv_encrypt(&c, NULL, 0U, NULL, 0U, buf,
						 sizeof(buf), &n));
	TEST_ASSERT_EQUAL_size_t(AES_SIV_TAG_LEN, n);
	TEST_ASSERT_EQUAL_INT(0, aes_siv_decrypt(&c, NULL, 0U, buf, n, NULL, 0U, &m));
	TEST_ASSERT_EQUAL_size_t(0U, m);

	/* Zero-length AD components are legal and are not the same as no AD. */
	{
		aes_siv_ad_t z[1] = { { NULL, 0U } };
		uint8_t with_z[64];
		size_t k = 0U;

		TEST_ASSERT_EQUAL_INT(0, aes_siv_encrypt(&c, z, 1U, NULL, 0U,
							 with_z, sizeof(with_z), &k));
		TEST_ASSERT_EQUAL_size_t(AES_SIV_TAG_LEN, k);
		TEST_ASSERT_FALSE(aes_siv_ct_memeq(with_z, buf, AES_SIV_TAG_LEN));
	}

	/* Exact aliasing: seal a plaintext where it already lies. */
	memcpy(&buf[AES_SIV_TAG_LEN], a1_pt, sizeof(a1_pt));
	{
		aes_siv_ad_t ad[1] = { { a1_ad, sizeof(a1_ad) } };

		TEST_ASSERT_EQUAL_INT(0, aes_siv_encrypt(&c, ad, 1U,
							 &buf[AES_SIV_TAG_LEN],
							 sizeof(a1_pt), buf,
							 sizeof(buf), &n));
		TEST_ASSERT_EQUAL_HEX8_ARRAY(a1_out, buf, sizeof(a1_out));

		memcpy(back, a1_out, sizeof(a1_out));
		TEST_ASSERT_EQUAL_INT(0, aes_siv_decrypt(&c, ad, 1U, back,
							 sizeof(a1_out),
							 &back[AES_SIV_TAG_LEN],
							 sizeof(back) -
								 AES_SIV_TAG_LEN,
							 &m));
		TEST_ASSERT_EQUAL_size_t(sizeof(a1_pt), m);
		TEST_ASSERT_EQUAL_HEX8_ARRAY(a1_pt, &back[AES_SIV_TAG_LEN],
					     sizeof(a1_pt));
	}
}

static void test_siv_bad_arguments(void)
{
	aes_siv_ctx_t c;
	port_crypto_t empty;
	aes_siv_ad_t ad[AES_SIV_MAX_AD + 1U];
	uint8_t out[64];
	size_t n = 0U;

	memset(&empty, 0, sizeof(empty));
	memset(&c, 0, sizeof(c));
	for (size_t i = 0U; i < ARRAY_LEN(ad); i++) {
		ad[i].p = a1_ad;
		ad[i].len = 4U;
	}

	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_init(NULL, &g_port, a1_key, 32U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_init(&c, NULL, a1_key, 32U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_init(&c, &g_port, NULL, 32U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_init(&c, &empty, a1_key, 32U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_init(&c, &g_port, a1_key, 16U));

	/* Using a context that init() rejected must not "work". */
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_encrypt(&c, NULL, 0U, NULL, 0U, out,
						       sizeof(out), &n));

	TEST_ASSERT_EQUAL_INT(0, aes_siv_init(&c, &g_port, a1_key, sizeof(a1_key)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_encrypt(NULL, NULL, 0U, NULL, 0U,
						       out, sizeof(out), &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_encrypt(&c, NULL, 0U, NULL, 0U, NULL,
						       sizeof(out), &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_encrypt(&c, NULL, 0U, NULL, 0U, out,
						       sizeof(out), NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_encrypt(&c, NULL, 0U, NULL, 4U, out,
						       sizeof(out), &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_encrypt(&c, NULL, 1U, NULL, 0U, out,
						       sizeof(out), &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_encrypt(&c, ad, ARRAY_LEN(ad), NULL,
						       0U, out, sizeof(out), &n));
	ad[0].p = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_encrypt(&c, ad, 1U, NULL, 0U, out,
						       sizeof(out), &n));
	ad[0].p = a1_ad;
	TEST_ASSERT_EQUAL_INT(-ENOSPC, aes_siv_encrypt(&c, NULL, 0U, a1_pt,
						       sizeof(a1_pt), out, 16U, &n));

	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_decrypt(NULL, NULL, 0U, a1_out, 30U,
						       out, sizeof(out), &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_decrypt(&c, NULL, 0U, NULL, 30U, out,
						       sizeof(out), &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_decrypt(&c, NULL, 0U, a1_out, 30U,
						       out, sizeof(out), NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_decrypt(&c, ad, ARRAY_LEN(ad), a1_out,
						       30U, out, sizeof(out), &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, aes_siv_decrypt(&c, NULL, 0U, a1_out, 30U,
						       NULL, 64U, &n));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, aes_siv_decrypt(&c, NULL, 0U, a1_out, 30U,
						       out, 4U, &n));
}

static void test_siv_reports_port_failure(void)
{
	aes_siv_ctx_t c;
	aes_siv_ad_t ad[1] = { { a1_ad, sizeof(a1_ad) } };
	uint8_t out[64];
	size_t n = 0U;

	TEST_ASSERT_EQUAL_INT(0, aes_siv_init(&c, &g_port, a1_key, sizeof(a1_key)));

	/*
	 * Walk the failure injection across every AES call an A.1-shaped
	 * encryption makes: subkey generation, each absorption inside each of
	 * the four CMACs, and the CTR keystream block. A failure anywhere must
	 * surface as -EIO and never as a plausible-looking tag.
	 */
	for (unsigned k = 1U; k <= 8U; k++) {
		host_crypto_init(&g_hc, 0xC0FFEEU);
		g_hc.fail_aes_in = k;
		TEST_ASSERT_EQUAL_INT(-EIO,
				      aes_siv_encrypt(&c, ad, 1U, a1_pt,
						      sizeof(a1_pt), out,
						      sizeof(out), &n));
	}
	TEST_ASSERT_EQUAL_UINT(8U, g_hc.n_aes);

	for (unsigned k = 1U; k <= 8U; k++) {
		host_crypto_init(&g_hc, 0xC0FFEEU);
		g_hc.fail_aes_in = k;
		TEST_ASSERT_EQUAL_INT(-EIO,
				      aes_siv_decrypt(&c, ad, 1U, a1_out,
						      sizeof(a1_out), out,
						      sizeof(out), &n));
	}
}

static void test_ct_memeq(void)
{
	static const uint8_t a[4] = { 1, 2, 3, 4 };
	static const uint8_t b[4] = { 1, 2, 3, 5 };
	static const uint8_t c[4] = { 9, 2, 3, 4 };

	TEST_ASSERT_TRUE(aes_siv_ct_memeq(a, a, sizeof(a)));
	TEST_ASSERT_FALSE(aes_siv_ct_memeq(a, b, sizeof(a)));
	TEST_ASSERT_FALSE(aes_siv_ct_memeq(a, c, sizeof(a)));
	TEST_ASSERT_TRUE(aes_siv_ct_memeq(a, b, 0U));
}

int main(void)
{
	UNITY_BEGIN();
	RUN_TEST(test_aes128_fips197_c1);
	RUN_TEST(test_aes256_fips197_c3);
	RUN_TEST(test_aes128_multiblock_sp800_38a);
	RUN_TEST(test_aes_bad_arguments);
	RUN_TEST(test_sha256_known_answers);
	RUN_TEST(test_hmac_sha256_rfc4231);
	RUN_TEST(test_cmac_rfc4493);
	RUN_TEST(test_cmac_bad_arguments);
	RUN_TEST(test_siv_rfc5297_a1);
	RUN_TEST(test_siv_rfc5297_a2);
	RUN_TEST(test_siv_rejects_tampering);
	RUN_TEST(test_siv_empty_and_aliased);
	RUN_TEST(test_siv_bad_arguments);
	RUN_TEST(test_siv_reports_port_failure);
	RUN_TEST(test_ct_memeq);
	return UNITY_END();
}
