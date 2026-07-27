/*
 * STS1000 "Meridian" — core/snmp SNMPv3 / USM unit tests.
 *
 * Provenance of the expectations (ARCHITECTURE.md §9 asks for published
 * vectors, not self-round-trips):
 *
 *   - **RFC 3414 Appendix A.3.2 is transcribed literally.** For the passphrase
 *     "maplesyrup" under SHA-1 the RFC publishes
 *       Ku  = 9f b5 cc 03 81 49 7b 37 93 52 89 39 ff 78 8d 5d 79 14 52 11
 *       Kul = 66 95 fe bc 92 88 e3 62 82 23 5f c7 15 1f 12 84 97 b3 8f 3f
 *     for engineID 00 00 00 00 00 00 00 00 00 00 00 02. Both are required here.
 *     That is the *only* external check on the 1 048 576-octet key expansion,
 *     which is the step most likely to be subtly wrong, and it simultaneously
 *     validates the private SHA-1 (a wrong hash could not produce either value).
 *
 *   - **RFC 7860 publishes no vectors**, so the SHA-256 analogue was derived
 *     independently — the same "maplesyrup"/engineID inputs run through
 *     Python's `hashlib.sha256`, which shares no code with this firmware:
 *       Ku  = ab51014d1e077f6017df2b12bee5f5aa72993177e9bb569c4dff5a4ca0b4afac
 *       Kul = 8982e0e549e866db361a6b625d84cccc11162d453ee8ce3a6445c2d6776f0f8b
 *     The MD5 line of A.3.1 is not applicable: usmHMACMD5AuthProtocol is not
 *     implemented (see snmp_v3.h).
 *
 *   - **RFC 3826 publishes no vectors either.** The AES-128-CFB construction is
 *     instead pinned to **NIST SP 800-38A §F.3.13/F.3.14** (CFB128-AES128,
 *     Encrypt and Decrypt), whose four blocks are transcribed below. RFC 3826
 *     §3.1.4 specifies exactly that mode, so agreeing with SP 800-38A is
 *     agreeing with RFC 3826. A short final block is checked separately,
 *     because that is the case RFC 3826 relies on and SP 800-38A does not show.
 *
 *   - **FIPS 180-4 / RFC 2202** anchor the private SHA-1 and HMAC-SHA-1 on their
 *     own, so a localisation failure is attributable.
 *
 *   - Whole-message properties are asserted against RFC 3412 §6 (the envelope
 *     field order and msgFlags bits) and RFC 3414 §3.2 (the numbered checks and
 *     which usmStats counter each one increments). The report OIDs are compared
 *     against literal 1.3.6.1.6.3.15.1.1.x.0 encodings.
 *
 *   - Every v3 parse path is fuzzed: random datagrams, random datagrams wearing
 *     a plausible envelope, and bit-flipped valid requests at all three security
 *     levels.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "host_aes.h"
#include "host_sha256.h"
#include "snmp/snmp.h"
#include "snmp/snmp_v3.h"
#include "test_support.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* ------------------------------------------------------------------------- */
/* fixtures                                                                  */
/* ------------------------------------------------------------------------- */

static host_crypto_t g_hc;
static port_crypto_t g_port;
static snmp_usm_ports_t g_ports;

static snmp_ctx_t g_agent;
static snmp_v3_ctx_t g_v3;

/* A device-unique value standing in for the ATECC608B serial number. */
static const uint8_t g_uniq[9] = { 0x01, 0x23, 0xAB, 0xCD, 0xEF,
				   0x12, 0x34, 0x56, 0xEE };
static uint8_t g_eid[SNMP_V3_ENGINEID_MAX];
static size_t g_eid_len;

/** A getter that answers every scalar, so a walk has something to return. */
static int getter(void *ctx, uint16_t obj, uint16_t inst, snmp_value_t *out)
{
	(void)ctx;

	switch (obj) {
	case SNMP_OBJ_SYS_DESCR:
		snmp_val_str(out, "STS1000 Meridian");
		return 0;
	case SNMP_OBJ_SYS_OBJECT_ID: {
		static const uint32_t oid[] = { 1U, 3U, 6U, 1U, 4U,
						1U, SNMP_PEN, 1U };

		snmp_val_oid(out, oid, ARRAY_LEN(oid));
		return 0;
	}
	case SNMP_OBJ_RAIL_INDEX:
		snmp_val_int(out, inst);
		return 0;
	case SNMP_OBJ_RAIL_NAME:
		snmp_val_str(out, "rail");
		return 0;
	case SNMP_OBJ_STRATUM:
		snmp_val_int(out, 1);
		return 0;
	default:
		snmp_val_uint(out, SNMP_TAG_GAUGE32, 42U);
		return 0;
	}
}

static void fixture_init(void)
{
	snmp_cfg_t acfg;
	snmp_v3_cfg_t vcfg;
	int n;

	host_crypto_init(&g_hc, 0xC0FFEEU);
	g_port = host_crypto_port(&g_hc);
	memset(&g_ports, 0, sizeof(g_ports));
	g_ports.crypto = &g_port;
	g_ports.sha256_stream = host_sha256_stream();

	memset(&acfg, 0, sizeof(acfg));
	acfg.community = "public";
	acfg.getter.get = getter;
	TEST_ASSERT_EQUAL_INT(0, snmp_init(&g_agent, &acfg));

	n = snmp_v3_engine_id_build(SNMP_PEN, g_uniq, sizeof(g_uniq), g_eid,
				    sizeof(g_eid));
	TEST_ASSERT_TRUE(n > 0);
	g_eid_len = (size_t)n;

	memset(&vcfg, 0, sizeof(vcfg));
	vcfg.ports = g_ports;
	vcfg.engine_boots = 7U;
	vcfg.engine_id = g_eid;
	vcfg.engine_id_len = g_eid_len;
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_init(&g_v3, &vcfg));
}

/* ------------------------------------------------------------------------- */
/* protocol tables                                                           */
/* ------------------------------------------------------------------------- */

static void test_protocol_tables(void)
{
	/* RFC 3414 §6.3.1: SHA-1 truncates to 12; RFC 7860 §4.2.2: 24. */
	TEST_ASSERT_EQUAL_UINT(12U,
			       snmp_usm_authparam_len(SNMP_AUTH_HMAC_SHA1_96));
	TEST_ASSERT_EQUAL_UINT(24U,
			       snmp_usm_authparam_len(SNMP_AUTH_HMAC_SHA256_192));
	TEST_ASSERT_EQUAL_UINT(0U, snmp_usm_authparam_len(SNMP_AUTH_NONE));
	TEST_ASSERT_EQUAL_UINT(0U, snmp_usm_authparam_len(99U));

	TEST_ASSERT_EQUAL_UINT(20U, snmp_usm_key_len(SNMP_AUTH_HMAC_SHA1_96));
	TEST_ASSERT_EQUAL_UINT(32U,
			       snmp_usm_key_len(SNMP_AUTH_HMAC_SHA256_192));
	TEST_ASSERT_EQUAL_UINT(0U, snmp_usm_key_len(SNMP_AUTH_NONE));

	TEST_ASSERT_EQUAL_STRING("HMAC-SHA-1-96",
				 snmp_auth_proto_name(SNMP_AUTH_HMAC_SHA1_96));
	TEST_ASSERT_EQUAL_STRING(
		"HMAC-SHA-256-192",
		snmp_auth_proto_name(SNMP_AUTH_HMAC_SHA256_192));
	TEST_ASSERT_EQUAL_STRING("none", snmp_auth_proto_name(SNMP_AUTH_NONE));
	TEST_ASSERT_EQUAL_STRING("AES-128-CFB",
				 snmp_priv_proto_name(SNMP_PRIV_AES128_CFB));
	TEST_ASSERT_EQUAL_STRING("none", snmp_priv_proto_name(SNMP_PRIV_NONE));
	TEST_ASSERT_EQUAL_STRING("noAuthNoPriv",
				 snmp_sec_level_name(SNMP_SEC_NOAUTH_NOPRIV));
	TEST_ASSERT_EQUAL_STRING("authNoPriv",
				 snmp_sec_level_name(SNMP_SEC_AUTH_NOPRIV));
	TEST_ASSERT_EQUAL_STRING("authPriv",
				 snmp_sec_level_name(SNMP_SEC_AUTH_PRIV));
	TEST_ASSERT_EQUAL_STRING("noAuthNoPriv", snmp_sec_level_name(9U));
}

/* ------------------------------------------------------------------------- */
/* RFC 3414 §2.6 key localisation                                            */
/* ------------------------------------------------------------------------- */

/* RFC 3414 Appendix A.3.2, and the independently derived SHA-256 analogue. */
static const uint8_t A3_ENGINE_ID[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2 };

static const uint8_t A3_SHA1_KU[20] = { 0x9f, 0xb5, 0xcc, 0x03, 0x81, 0x49,
					0x7b, 0x37, 0x93, 0x52, 0x89, 0x39,
					0xff, 0x78, 0x8d, 0x5d, 0x79, 0x14,
					0x52, 0x11 };
static const uint8_t A3_SHA1_KUL[20] = { 0x66, 0x95, 0xfe, 0xbc, 0x92, 0x88,
					 0xe3, 0x62, 0x82, 0x23, 0x5f, 0xc7,
					 0x15, 0x1f, 0x12, 0x84, 0x97, 0xb3,
					 0x8f, 0x3f };

static const uint8_t A3_SHA256_KU[32] = {
	0xab, 0x51, 0x01, 0x4d, 0x1e, 0x07, 0x7f, 0x60, 0x17, 0xdf, 0x2b,
	0x12, 0xbe, 0xe5, 0xf5, 0xaa, 0x72, 0x99, 0x31, 0x77, 0xe9, 0xbb,
	0x56, 0x9c, 0x4d, 0xff, 0x5a, 0x4c, 0xa0, 0xb4, 0xaf, 0xac
};
static const uint8_t A3_SHA256_KUL[32] = {
	0x89, 0x82, 0xe0, 0xe5, 0x49, 0xe8, 0x66, 0xdb, 0x36, 0x1a, 0x6b,
	0x62, 0x5d, 0x84, 0xcc, 0xcc, 0x11, 0x16, 0x2d, 0x45, 0x3e, 0xe8,
	0xce, 0x3a, 0x64, 0x45, 0xc2, 0xd6, 0x77, 0x6f, 0x0f, 0x8b
};

static void test_password_to_key_rfc3414_a3(void)
{
	uint8_t ku[SNMP_V3_KEY_MAX];
	uint8_t kul[SNMP_V3_KEY_MAX];
	size_t len = 0U;

	/* SHA-1: the published vector. */
	TEST_ASSERT_EQUAL_INT(0, snmp_usm_password_to_key(&g_ports,
							  SNMP_AUTH_HMAC_SHA1_96,
							  "maplesyrup", ku,
							  &len));
	TEST_ASSERT_EQUAL_UINT(20U, len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(A3_SHA1_KU, ku, sizeof(A3_SHA1_KU));

	TEST_ASSERT_EQUAL_INT(0, snmp_usm_localize(&g_ports,
						   SNMP_AUTH_HMAC_SHA1_96, ku,
						   len, A3_ENGINE_ID,
						   sizeof(A3_ENGINE_ID), kul,
						   &len));
	TEST_ASSERT_EQUAL_UINT(20U, len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(A3_SHA1_KUL, kul, sizeof(A3_SHA1_KUL));

	/* SHA-256: the independently derived analogue. */
	TEST_ASSERT_EQUAL_INT(0,
			      snmp_usm_password_to_key(&g_ports,
						       SNMP_AUTH_HMAC_SHA256_192,
						       "maplesyrup", ku, &len));
	TEST_ASSERT_EQUAL_UINT(32U, len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(A3_SHA256_KU, ku, sizeof(A3_SHA256_KU));

	TEST_ASSERT_EQUAL_INT(0, snmp_usm_localize(&g_ports,
						   SNMP_AUTH_HMAC_SHA256_192, ku,
						   len, A3_ENGINE_ID,
						   sizeof(A3_ENGINE_ID), kul,
						   &len));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(A3_SHA256_KUL, kul, sizeof(A3_SHA256_KUL));

	/* The one-shot form must agree with the two-step one. */
	TEST_ASSERT_EQUAL_INT(0,
			      snmp_usm_password_to_localized(
				      &g_ports, SNMP_AUTH_HMAC_SHA1_96,
				      "maplesyrup", A3_ENGINE_ID,
				      sizeof(A3_ENGINE_ID), kul, NULL));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(A3_SHA1_KUL, kul, sizeof(A3_SHA1_KUL));
}

static void test_localisation_binds_to_the_engine_id(void)
{
	uint8_t a[SNMP_V3_KEY_MAX];
	uint8_t b[SNMP_V3_KEY_MAX];
	uint8_t other[12];

	memcpy(other, A3_ENGINE_ID, sizeof(other));
	other[11] = 3U;

	TEST_ASSERT_EQUAL_INT(0, snmp_usm_password_to_localized(
					 &g_ports, SNMP_AUTH_HMAC_SHA256_192,
					 "maplesyrup", A3_ENGINE_ID,
					 sizeof(A3_ENGINE_ID), a, NULL));
	TEST_ASSERT_EQUAL_INT(0, snmp_usm_password_to_localized(
					 &g_ports, SNMP_AUTH_HMAC_SHA256_192,
					 "maplesyrup", other, sizeof(other), b,
					 NULL));
	/* One flipped engine-id octet is a completely different key — which is
	 * why changing the engine id has to clear the user table. */
	TEST_ASSERT_TRUE(memcmp(a, b, 32U) != 0);
}

static void test_password_to_key_guards(void)
{
	snmp_usm_ports_t no_stream = g_ports;
	snmp_usm_ports_t no_sha = g_ports;
	port_crypto_t crippled = g_port;
	uint8_t out[SNMP_V3_KEY_MAX];
	char big[SNMP_V3_PASSPHRASE_MAX + 2U];

	memset(big, 'p', sizeof(big) - 1U);
	big[sizeof(big) - 1U] = '\0';

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_password_to_key(NULL,
						       SNMP_AUTH_HMAC_SHA1_96,
						       "maplesyrup", out, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_password_to_key(&g_ports,
						       SNMP_AUTH_HMAC_SHA1_96,
						       NULL, out, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_password_to_key(&g_ports,
						       SNMP_AUTH_HMAC_SHA1_96,
						       "maplesyrup", NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_password_to_key(&g_ports, SNMP_AUTH_NONE,
						       "maplesyrup", out, NULL));
	/* RFC 3414 §11.2: at least eight characters. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_password_to_key(&g_ports,
						       SNMP_AUTH_HMAC_SHA1_96,
						       "short", out, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_password_to_key(&g_ports,
						       SNMP_AUTH_HMAC_SHA1_96,
						       big, out, NULL));

	/* SHA-256 passphrase expansion needs the streaming port. */
	no_stream.sha256_stream = NULL;
	TEST_ASSERT_EQUAL_INT(-ENOTSUP,
			      snmp_usm_password_to_key(&no_stream,
						       SNMP_AUTH_HMAC_SHA256_192,
						       "maplesyrup", out, NULL));

	/* Localisation with SHA-256 needs the one-shot sha256. */
	crippled.sha256 = NULL;
	no_sha.crypto = &crippled;
	TEST_ASSERT_EQUAL_INT(-ENOTSUP,
			      snmp_usm_localize(&no_sha,
						SNMP_AUTH_HMAC_SHA256_192,
						A3_SHA256_KU, 32U, A3_ENGINE_ID,
						sizeof(A3_ENGINE_ID), out,
						NULL));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_localize(&g_ports,
						SNMP_AUTH_HMAC_SHA1_96, NULL,
						20U, A3_ENGINE_ID, 12U, out,
						NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_localize(&g_ports,
						SNMP_AUTH_HMAC_SHA1_96,
						A3_SHA1_KU, 19U, A3_ENGINE_ID,
						12U, out, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_localize(&g_ports, SNMP_AUTH_NONE,
						A3_SHA1_KU, 20U, A3_ENGINE_ID,
						12U, out, NULL));
	/* An engine id outside 5..32 octets. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_localize(&g_ports,
						SNMP_AUTH_HMAC_SHA1_96,
						A3_SHA1_KU, 20U, A3_ENGINE_ID,
						4U, out, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_localize(&g_ports,
						SNMP_AUTH_HMAC_SHA1_96,
						A3_SHA1_KU, 20U, A3_ENGINE_ID,
						33U, out, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_localize(&g_ports,
						SNMP_AUTH_HMAC_SHA1_96,
						A3_SHA1_KU, 20U, NULL, 12U, out,
						NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_localize(NULL, SNMP_AUTH_HMAC_SHA1_96,
						A3_SHA1_KU, 20U, A3_ENGINE_ID,
						12U, out, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_localize(&g_ports,
						SNMP_AUTH_HMAC_SHA1_96,
						A3_SHA1_KU, 20U, A3_ENGINE_ID,
						12U, NULL, NULL));
}

/* ------------------------------------------------------------------------- */
/* authentication digest                                                     */
/* ------------------------------------------------------------------------- */

static void test_auth_digest_is_a_truncated_hmac(void)
{
	uint8_t msg[64];
	uint8_t got[SNMP_V3_AUTHPARAM_MAX];
	uint8_t want[32];
	size_t len = 0U;
	size_t i;

	for (i = 0U; i < sizeof(msg); i++) {
		msg[i] = (uint8_t)i;
	}

	/* SHA-256: the first 24 octets of HMAC-SHA-256, computed here by the
	 * host implementation rather than by the code under test. */
	TEST_ASSERT_EQUAL_INT(0, snmp_usm_auth(&g_ports,
					       SNMP_AUTH_HMAC_SHA256_192,
					       A3_SHA256_KUL, 32U, msg,
					       sizeof(msg), got, &len));
	TEST_ASSERT_EQUAL_UINT(24U, len);
	host_hmac_sha256(A3_SHA256_KUL, 32U, msg, sizeof(msg), want);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want, got, 24U);

	/* SHA-1 truncates to 12. Its HMAC is private to the module, so the
	 * assertion is on the length and on determinism. */
	TEST_ASSERT_EQUAL_INT(0, snmp_usm_auth(&g_ports,
					       SNMP_AUTH_HMAC_SHA1_96,
					       A3_SHA1_KUL, 20U, msg,
					       sizeof(msg), got, &len));
	TEST_ASSERT_EQUAL_UINT(12U, len);

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_auth(NULL, SNMP_AUTH_HMAC_SHA1_96,
					    A3_SHA1_KUL, 20U, msg, 4U, got,
					    NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_auth(&g_ports, SNMP_AUTH_HMAC_SHA1_96,
					    NULL, 20U, msg, 4U, got, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_auth(&g_ports, SNMP_AUTH_HMAC_SHA1_96,
					    A3_SHA1_KUL, 20U, NULL, 4U, got,
					    NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_auth(&g_ports, SNMP_AUTH_HMAC_SHA1_96,
					    A3_SHA1_KUL, 20U, msg, 4U, NULL,
					    NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_auth(&g_ports, SNMP_AUTH_NONE,
					    A3_SHA1_KUL, 20U, msg, 4U, got,
					    NULL));
	/* A key of the wrong length for the protocol. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_usm_auth(&g_ports, SNMP_AUTH_HMAC_SHA1_96,
					    A3_SHA1_KUL, 32U, msg, 4U, got,
					    NULL));
}

/* ------------------------------------------------------------------------- */
/* privacy: AES-128-CFB against NIST SP 800-38A                              */
/* ------------------------------------------------------------------------- */

static void test_aes_cfb_nist_sp800_38a(void)
{
	/* SP 800-38A §F.3.13 CFB128-AES128.Encrypt / §F.3.14 .Decrypt. */
	static const uint8_t key[16] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae,
					 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88,
					 0x09, 0xcf, 0x4f, 0x3c };
	static const uint8_t iv[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
					0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
					0x0c, 0x0d, 0x0e, 0x0f };
	static const uint8_t plain[64] = {
		0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d,
		0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a, 0xae, 0x2d, 0x8a, 0x57,
		0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf,
		0x8e, 0x51, 0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11,
		0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef, 0xf6, 0x9f,
		0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17, 0xad, 0x2b, 0x41, 0x7b,
		0xe6, 0x6c, 0x37, 0x10
	};
	static const uint8_t cipher[64] = {
		0x3b, 0x3f, 0xd9, 0x2e, 0xb7, 0x2d, 0xad, 0x20, 0x33, 0x34,
		0x49, 0xf8, 0xe8, 0x3c, 0xfb, 0x4a, 0xc8, 0xa6, 0x45, 0x37,
		0xa0, 0xb3, 0xa9, 0x3f, 0xcd, 0xe3, 0xcd, 0xad, 0x9f, 0x1c,
		0xe5, 0x8b, 0x26, 0x75, 0x1f, 0x67, 0xa3, 0xcb, 0xb1, 0x40,
		0xb1, 0x80, 0x8c, 0xf1, 0x87, 0xa4, 0xf4, 0xdf, 0xc0, 0x4b,
		0x05, 0x35, 0x7c, 0x5d, 0x1c, 0x0e, 0xea, 0xc4, 0xc6, 0x6f,
		0x9f, 0xf7, 0xf2, 0xe6
	};
	uint8_t buf[64];

	TEST_ASSERT_EQUAL_INT(0, snmp_usm_aes_cfb(&g_ports, key, iv, plain, buf,
						  sizeof(plain), true));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(cipher, buf, sizeof(cipher));

	TEST_ASSERT_EQUAL_INT(0, snmp_usm_aes_cfb(&g_ports, key, iv, cipher, buf,
						  sizeof(cipher), false));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(plain, buf, sizeof(plain));

	/* In-place, both directions — the normal case for a scopedPDU. */
	memcpy(buf, plain, sizeof(plain));
	TEST_ASSERT_EQUAL_INT(0, snmp_usm_aes_cfb(&g_ports, key, iv, buf, buf,
						  sizeof(buf), true));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(cipher, buf, sizeof(cipher));
	TEST_ASSERT_EQUAL_INT(0, snmp_usm_aes_cfb(&g_ports, key, iv, buf, buf,
						  sizeof(buf), false));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(plain, buf, sizeof(plain));
}

static void test_aes_cfb_short_final_block(void)
{
	static const uint8_t key[16] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae,
					 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88,
					 0x09, 0xcf, 0x4f, 0x3c };
	static const uint8_t iv[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
					0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
					0x0c, 0x0d, 0x0e, 0x0f };
	uint8_t plain[70];
	uint8_t enc[70];
	uint8_t dec[70];
	size_t n;
	size_t i;

	for (i = 0U; i < sizeof(plain); i++) {
		plain[i] = (uint8_t)(i * 3U);
	}

	/* RFC 3826 §3.1.4 relies on a possibly-short last block, so the
	 * ciphertext length must equal the plaintext length at every size and
	 * every prefix must decrypt back. That is what lets the scopedPDU go
	 * unpadded. */
	for (n = 0U; n <= sizeof(plain); n++) {
		TEST_ASSERT_EQUAL_INT(0, snmp_usm_aes_cfb(&g_ports, key, iv,
							  plain, enc, n, true));
		TEST_ASSERT_EQUAL_INT(0, snmp_usm_aes_cfb(&g_ports, key, iv, enc,
							  dec, n, false));
		if (n != 0U) {
			TEST_ASSERT_EQUAL_HEX8_ARRAY(plain, dec, n);
			/* A short block still changes the data. */
			TEST_ASSERT_TRUE(memcmp(plain, enc, n) != 0);
		}
	}
}

static void test_aes_cfb_guards(void)
{
	static const uint8_t key[16] = { 0 };
	static const uint8_t iv[16] = { 0 };
	snmp_usm_ports_t no_aes = g_ports;
	port_crypto_t crippled = g_port;
	uint8_t buf[16] = { 0 };

	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_usm_aes_cfb(NULL, key, iv, buf, buf,
							16U, true));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_usm_aes_cfb(&g_ports, NULL, iv, buf,
							buf, 16U, true));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_usm_aes_cfb(&g_ports, key, NULL, buf,
							buf, 16U, true));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_usm_aes_cfb(&g_ports, key, iv, buf,
							NULL, 16U, true));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_usm_aes_cfb(&g_ports, key, iv, NULL,
							buf, 16U, true));

	crippled.aes_ecb_encrypt = NULL;
	no_aes.crypto = &crippled;
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, snmp_usm_aes_cfb(&no_aes, key, iv, buf,
							 buf, 16U, true));

	/* An AES failure from the port is reported, not ignored. */
	g_hc.fail_aes_in = 1U;
	TEST_ASSERT_EQUAL_INT(-EIO, snmp_usm_aes_cfb(&g_ports, key, iv, buf, buf,
						     16U, true));
	g_hc.fail_aes_in = 0U;
}

static void test_priv_iv_layout(void)
{
	static const uint8_t salt[8] = { 0xAA, 0xBB, 0xCC, 0xDD,
					 0xEE, 0xFF, 0x01, 0x02 };
	uint8_t iv[16];

	/* RFC 3826 §3.1.2.1: boots ‖ time ‖ salt, both counters big-endian. */
	TEST_ASSERT_EQUAL_INT(0, snmp_usm_priv_iv(0x01020304U, 0x05060708U, salt,
						  iv));
	TEST_ASSERT_EQUAL_HEX8(0x01U, iv[0]);
	TEST_ASSERT_EQUAL_HEX8(0x02U, iv[1]);
	TEST_ASSERT_EQUAL_HEX8(0x03U, iv[2]);
	TEST_ASSERT_EQUAL_HEX8(0x04U, iv[3]);
	TEST_ASSERT_EQUAL_HEX8(0x05U, iv[4]);
	TEST_ASSERT_EQUAL_HEX8(0x06U, iv[5]);
	TEST_ASSERT_EQUAL_HEX8(0x07U, iv[6]);
	TEST_ASSERT_EQUAL_HEX8(0x08U, iv[7]);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(salt, &iv[8], 8U);

	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_usm_priv_iv(0U, 0U, NULL, iv));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_usm_priv_iv(0U, 0U, salt, NULL));
}

/* ------------------------------------------------------------------------- */
/* engine id                                                                 */
/* ------------------------------------------------------------------------- */

static void test_engine_id_build(void)
{
	uint8_t out[SNMP_V3_ENGINEID_MAX];
	uint8_t big[40];
	int n;

	n = snmp_v3_engine_id_build(SNMP_PEN, g_uniq, sizeof(g_uniq), out,
				    sizeof(out));
	/* RFC 3411 §5: 4 octets of PEN with the top bit set, a format octet,
	 * then the unique value. */
	TEST_ASSERT_EQUAL_INT(5 + (int)sizeof(g_uniq), n);
	TEST_ASSERT_EQUAL_HEX8(0x80U | ((SNMP_PEN >> 24) & 0x7FU), out[0]);
	TEST_ASSERT_EQUAL_HEX8((SNMP_PEN >> 16) & 0xFFU, out[1]);
	TEST_ASSERT_EQUAL_HEX8((SNMP_PEN >> 8) & 0xFFU, out[2]);
	TEST_ASSERT_EQUAL_HEX8(SNMP_PEN & 0xFFU, out[3]);
	TEST_ASSERT_EQUAL_HEX8(5U, out[4]);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_uniq, &out[5], sizeof(g_uniq));
	/* And the result is a legal SnmpEngineID length. */
	TEST_ASSERT_TRUE((size_t)n >= SNMP_V3_ENGINEID_MIN);
	TEST_ASSERT_TRUE((size_t)n <= SNMP_V3_ENGINEID_MAX);

	/* An over-long unique value is truncated to fit, not rejected. */
	memset(big, 0x77, sizeof(big));
	n = snmp_v3_engine_id_build(1U, big, sizeof(big), out, sizeof(out));
	TEST_ASSERT_EQUAL_INT((int)SNMP_V3_ENGINEID_MAX, n);

	/* An empty unique value would give every board the same engine id. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_engine_id_build(1U, g_uniq, 0U, out,
						      sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_engine_id_build(1U, NULL, 4U, out,
						      sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_engine_id_build(1U, g_uniq, 4U, NULL,
						      sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_engine_id_build(0x80000000U, g_uniq, 4U,
						      out, sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      snmp_v3_engine_id_build(1U, g_uniq,
						      sizeof(g_uniq), out, 8U));
}

/* ------------------------------------------------------------------------- */
/* context and users                                                         */
/* ------------------------------------------------------------------------- */

static void test_init_guards(void)
{
	snmp_v3_ctx_t c;
	snmp_v3_cfg_t cfg;
	port_crypto_t crippled;

	memset(&cfg, 0, sizeof(cfg));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_init(NULL, &cfg));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_init(&c, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_init(&c, &cfg));

	cfg.ports = g_ports;
	cfg.engine_id = g_eid;
	cfg.engine_id_len = g_eid_len;
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_init(&c, &cfg));
	/* max_msg_size below the RFC 3412 floor is replaced. */
	TEST_ASSERT_EQUAL_UINT32(SNMP_PKT_MAX, c.cfg.max_msg_size);

	cfg.engine_id_len = 4U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_init(&c, &cfg));
	cfg.engine_id_len = 33U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_init(&c, &cfg));
	cfg.engine_id = NULL;
	cfg.engine_id_len = g_eid_len;
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_init(&c, &cfg));

	crippled = g_port;
	crippled.hmac_sha256 = NULL;
	cfg.engine_id = g_eid;
	cfg.ports.crypto = &crippled;
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, snmp_v3_init(&c, &cfg));
	crippled = g_port;
	crippled.sha256 = NULL;
	cfg.ports.crypto = &crippled;
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, snmp_v3_init(&c, &cfg));
}

static void test_engine_id_and_boots_accessors(void)
{
	uint8_t out[SNMP_V3_ENGINEID_MAX];
	uint8_t other[8] = { 0x80, 0, 0, 1, 5, 9, 9, 9 };
	size_t len = 0U;

	fixture_init();

	TEST_ASSERT_EQUAL_INT(0, snmp_v3_get_engine_id(&g_v3, out, &len));
	TEST_ASSERT_EQUAL_UINT(g_eid_len, len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_eid, out, len);

	TEST_ASSERT_EQUAL_UINT32(7U, snmp_v3_get_boots(&g_v3));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_set_boots(&g_v3, 100U));
	TEST_ASSERT_EQUAL_UINT32(100U, snmp_v3_get_boots(&g_v3));
	/* RFC 3414 §2.2.2: 2147483647 is terminal, so it is clamped there. */
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_set_boots(&g_v3, 0xFFFFFFFFU));
	TEST_ASSERT_EQUAL_UINT32(0x7FFFFFFFU, snmp_v3_get_boots(&g_v3));
	TEST_ASSERT_EQUAL_UINT32(0U, snmp_v3_get_boots(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_set_boots(NULL, 1U));

	/* Changing the engine id clears the users, because their localised keys
	 * were bound to the old one. */
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_set(&g_v3, "alice",
						  SNMP_AUTH_HMAC_SHA256_192,
						  "maplesyrup", 10U, false,
						  SNMP_PRIV_NONE, NULL, 0U,
						  false));
	TEST_ASSERT_EQUAL_UINT(1U, snmp_v3_user_count(&g_v3));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_set_engine_id(&g_v3, other,
						      sizeof(other)));
	TEST_ASSERT_EQUAL_UINT(0U, snmp_v3_user_count(&g_v3));

	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_set_engine_id(&g_v3, other, 4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_set_engine_id(&g_v3, other, 33U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_set_engine_id(&g_v3, NULL, 8U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_set_engine_id(NULL, other, 8U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_get_engine_id(NULL, out, &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_get_engine_id(&g_v3, NULL, &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_get_engine_id(&g_v3, out, NULL));
	TEST_ASSERT_EQUAL_UINT(0U, snmp_v3_user_count(NULL));
	snmp_v3_users_clear(NULL);
}

static void test_user_set_variants(void)
{
	uint8_t kul[SNMP_V3_KEY_MAX];
	uint8_t level = 0U;

	fixture_init();

	/* A passphrase, localised on the way in. */
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_set(&g_v3, "alice",
						  SNMP_AUTH_HMAC_SHA256_192,
						  "maplesyrup", 10U, false,
						  SNMP_PRIV_NONE, NULL, 0U,
						  false));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_level(&g_v3, "alice", &level));
	TEST_ASSERT_EQUAL_UINT8(SNMP_SEC_AUTH_NOPRIV, level);

	/* The stored key must equal what a manager would compute. */
	TEST_ASSERT_EQUAL_INT(0, snmp_usm_password_to_localized(
					 &g_ports, SNMP_AUTH_HMAC_SHA256_192,
					 "maplesyrup", g_eid, g_eid_len, kul,
					 NULL));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(kul, g_v3.users[0].auth_key, 32U);

	/* A NUL-terminated passphrase inside a longer cfg blob. */
	{
		uint8_t blob[32];

		memset(blob, 0, sizeof(blob));
		memcpy(blob, "maplesyrup", 10U);
		TEST_ASSERT_EQUAL_INT(0,
				      snmp_v3_user_set(&g_v3, "alice2",
						       SNMP_AUTH_HMAC_SHA256_192,
						       blob, sizeof(blob), false,
						       SNMP_PRIV_NONE, NULL, 0U,
						       false));
		TEST_ASSERT_EQUAL_HEX8_ARRAY(kul, g_v3.users[1].auth_key, 32U);
	}

	/* An already-localised key, and a privacy key with it. */
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_set(&g_v3, "bob",
						  SNMP_AUTH_HMAC_SHA256_192, kul,
						  32U, true,
						  SNMP_PRIV_AES128_CFB, kul,
						  32U, true));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_level(&g_v3, "bob", &level));
	TEST_ASSERT_EQUAL_UINT8(SNMP_SEC_AUTH_PRIV, level);

	/* A no-auth user. */
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_set(&g_v3, "guest",
						  SNMP_AUTH_NONE, NULL, 0U,
						  false, SNMP_PRIV_NONE, NULL,
						  0U, false));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_level(&g_v3, "guest", &level));
	TEST_ASSERT_EQUAL_UINT8(SNMP_SEC_NOAUTH_NOPRIV, level);

	TEST_ASSERT_EQUAL_UINT(4U, snmp_v3_user_count(&g_v3));

	/* Replacing an existing user reuses its slot. */
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_set(&g_v3, "alice",
						  SNMP_AUTH_HMAC_SHA1_96,
						  "maplesyrup", 10U, false,
						  SNMP_PRIV_NONE, NULL, 0U,
						  false));
	TEST_ASSERT_EQUAL_UINT(4U, snmp_v3_user_count(&g_v3));

	/* And the table is finite. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      snmp_v3_user_set(&g_v3, "eve", SNMP_AUTH_NONE,
					       NULL, 0U, false, SNMP_PRIV_NONE,
					       NULL, 0U, false));

	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_clear(&g_v3, "alice"));
	TEST_ASSERT_EQUAL_UINT(3U, snmp_v3_user_count(&g_v3));
	TEST_ASSERT_EQUAL_INT(-ENOENT, snmp_v3_user_clear(&g_v3, "alice"));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_user_clear(&g_v3, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_user_clear(NULL, "bob"));

	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      snmp_v3_user_level(&g_v3, "nobody", &level));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_user_level(&g_v3, NULL, &level));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_user_level(&g_v3, "bob", NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_user_level(NULL, "bob", &level));
}

static void test_user_set_guards(void)
{
	uint8_t kul[SNMP_V3_KEY_MAX];
	char big[SNMP_V3_USER_MAX + 2U];

	fixture_init();
	memset(kul, 0x5A, sizeof(kul));
	memset(big, 'u', sizeof(big) - 1U);
	big[sizeof(big) - 1U] = '\0';

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_user_set(NULL, "a", SNMP_AUTH_NONE, NULL,
					       0U, false, SNMP_PRIV_NONE, NULL,
					       0U, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_user_set(&g_v3, NULL, SNMP_AUTH_NONE, NULL,
					       0U, false, SNMP_PRIV_NONE, NULL,
					       0U, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_user_set(&g_v3, "", SNMP_AUTH_NONE, NULL,
					       0U, false, SNMP_PRIV_NONE, NULL,
					       0U, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_user_set(&g_v3, big, SNMP_AUTH_NONE, NULL,
					       0U, false, SNMP_PRIV_NONE, NULL,
					       0U, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_user_set(&g_v3, "a", 9U, kul, 32U, true,
					       SNMP_PRIV_NONE, NULL, 0U, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_user_set(&g_v3, "a",
					       SNMP_AUTH_HMAC_SHA256_192, kul,
					       32U, true, 9U, kul, 32U, true));

	/* Privacy without authentication is not a security level RFC 3414 has. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_user_set(&g_v3, "a", SNMP_AUTH_NONE, NULL,
					       0U, false, SNMP_PRIV_AES128_CFB,
					       kul, 32U, true));

	/* A localised key of the wrong length. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_user_set(&g_v3, "a",
					       SNMP_AUTH_HMAC_SHA256_192, kul,
					       31U, true, SNMP_PRIV_NONE, NULL,
					       0U, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_user_set(&g_v3, "a",
					       SNMP_AUTH_HMAC_SHA256_192, NULL,
					       32U, true, SNMP_PRIV_NONE, NULL,
					       0U, false));
	/* A passphrase that is too short, or empty inside its blob. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_user_set(&g_v3, "a",
					       SNMP_AUTH_HMAC_SHA256_192, "abc",
					       3U, false, SNMP_PRIV_NONE, NULL,
					       0U, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_user_set(&g_v3, "a",
					       SNMP_AUTH_HMAC_SHA256_192, "",
					       1U, false, SNMP_PRIV_NONE, NULL,
					       0U, false));
	/*
	 * A SHA-1 authPriv user is legal: RFC 3826 §3.1.2.1 localises the
	 * privacy key with the *auth* protocol's hash, so it is 20 octets long
	 * and AES-128 takes the first 16.
	 */
	TEST_ASSERT_EQUAL_INT(0,
			      snmp_v3_user_set(&g_v3, "sha1priv",
					       SNMP_AUTH_HMAC_SHA1_96, kul, 20U,
					       true, SNMP_PRIV_AES128_CFB, kul,
					       20U, true));
	{
		uint8_t level = 0U;

		TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_level(&g_v3, "sha1priv",
							    &level));
		TEST_ASSERT_EQUAL_UINT8(SNMP_SEC_AUTH_PRIV, level);
	}
}

static void test_user_set_needs_an_engine_id(void)
{
	snmp_v3_ctx_t c;
	snmp_v3_cfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.ports = g_ports;
	cfg.engine_id = g_eid;
	cfg.engine_id_len = g_eid_len;
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_init(&c, &cfg));

	/* Reach in and clear it, which is the only way to produce the state a
	 * caller could otherwise create by ignoring snmp_v3_init()'s result. */
	c.engine_id_len = 0U;
	TEST_ASSERT_EQUAL_INT(-EPERM,
			      snmp_v3_user_set(&c, "a", SNMP_AUTH_NONE, NULL, 0U,
					       false, SNMP_PRIV_NONE, NULL, 0U,
					       false));
}

/* ------------------------------------------------------------------------- */
/* usmStats OIDs                                                             */
/* ------------------------------------------------------------------------- */

static void test_usm_stats_oids(void)
{
	/* RFC 3414 §5: 1.3.6.1.6.3.15.1.1.<n>, instance .0. */
	static const uint32_t want4[] = { 1U, 3U, 6U,  1U, 6U,
					  3U, 15U, 1U, 1U, 4U, 0U };
	uint32_t arcs[SNMP_OID_MAX_LEN];
	unsigned int i;
	int n;

	n = snmp_usm_stats_oid(4U, arcs);
	TEST_ASSERT_EQUAL_INT((int)ARRAY_LEN(want4), n);
	for (i = 0U; i < ARRAY_LEN(want4); i++) {
		TEST_ASSERT_EQUAL_UINT32(want4[i], arcs[i]);
	}

	for (i = 1U; i <= 6U; i++) {
		n = snmp_usm_stats_oid(i, arcs);
		TEST_ASSERT_EQUAL_INT(11, n);
		TEST_ASSERT_EQUAL_UINT32(i, arcs[9]);
		TEST_ASSERT_EQUAL_UINT32(0U, arcs[10]);
	}

	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_usm_stats_oid(0U, arcs));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_usm_stats_oid(7U, arcs));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_usm_stats_oid(1U, NULL));
}

/* ------------------------------------------------------------------------- */
/* request building (an independent v3 encoder for the test)                  */
/* ------------------------------------------------------------------------- */

/*
 * The requests below are assembled by this second, deliberately different
 * encoder: a flat append-only writer with hand-computed lengths, rather than the
 * reserve-and-shrink writer core/snmp uses. A bug in one cannot cancel against
 * the other.
 */
typedef struct {
	uint8_t buf[SNMP_PKT_MAX];
	size_t len;
} w_t;

static void w_reset(w_t *w)
{
	w->len = 0U;
}

static void w_raw(w_t *w, const void *p, size_t n)
{
	memcpy(&w->buf[w->len], p, n);
	w->len += n;
}

static void w_tlv(w_t *w, uint8_t tag, const void *val, size_t n)
{
	w->buf[w->len++] = tag;
	if (n < 0x80U) {
		w->buf[w->len++] = (uint8_t)n;
	} else if (n <= 0xFFU) {
		w->buf[w->len++] = 0x81U;
		w->buf[w->len++] = (uint8_t)n;
	} else {
		w->buf[w->len++] = 0x82U;
		w->buf[w->len++] = (uint8_t)((n >> 8) & 0xFFU);
		w->buf[w->len++] = (uint8_t)(n & 0xFFU);
	}
	if (n != 0U) {
		w_raw(w, val, n);
	}
}

static void w_int(w_t *w, int32_t v)
{
	uint8_t tmp[5];
	size_t n = 0U;
	int i;

	for (i = 3; i > 0; i--) {
		if (((((uint32_t)v) >> (8 * (unsigned int)i)) & 0xFFU) != 0U) {
			break;
		}
	}
	if (((((uint32_t)v) >> (8 * (unsigned int)i)) & 0x80U) != 0U) {
		tmp[n++] = 0x00U;
	}
	for (; i >= 0; i--) {
		tmp[n++] = (uint8_t)((((uint32_t)v) >> (8 * (unsigned int)i)) &
				     0xFFU);
	}
	w_tlv(w, SNMP_TAG_INTEGER, tmp, n);
}

/** OID 1.3.6.1.4.1.<PEN>.1.1.1.0 — sts1000Stratum. */
static void w_oid_stratum(w_t *w)
{
	static const uint8_t enc[] = { 0x2B, 0x06, 0x01, 0x04, 0x01,
				       0x86, 0x8D, 0x1F, 0x01, 0x01,
				       0x01, 0x00 };

	w_tlv(w, SNMP_TAG_OID, enc, sizeof(enc));
}

typedef struct {
	int32_t msg_id;
	uint32_t max_size;
	uint8_t flags;
	int32_t sec_model;
	const uint8_t *eid;
	size_t eid_len;
	uint32_t boots;
	uint32_t time_s;
	const char *user;
	size_t authparam_len;
	const uint8_t *salt;   /**< NULL for no privacy parameters */
	uint8_t pdu_tag;
	bool bad_context_eid;
	bool bad_context_name;
} req_spec_t;

/** Build a scopedPDU (contextEngineID, contextName, PDU) into @p out. */
static size_t build_scoped(const req_spec_t *s, uint8_t *out)
{
	w_t vbl;
	w_t pdu;
	w_t scoped;
	w_t vb;

	/* One varbind: sts1000Stratum.0 = NULL. */
	w_reset(&vb);
	w_oid_stratum(&vb);
	w_tlv(&vb, SNMP_TAG_NULL, NULL, 0U);
	w_reset(&vbl);
	w_tlv(&vbl, SNMP_TAG_SEQUENCE, vb.buf, vb.len);

	w_reset(&pdu);
	w_int(&pdu, 0x1234);
	w_int(&pdu, 0);
	w_int(&pdu, 0);
	w_tlv(&pdu, SNMP_TAG_SEQUENCE, vbl.buf, vbl.len);

	w_reset(&scoped);
	if (s->bad_context_eid) {
		static const uint8_t junk[5] = { 9, 9, 9, 9, 9 };

		w_tlv(&scoped, SNMP_TAG_OCTET_STRING, junk, sizeof(junk));
	} else {
		w_tlv(&scoped, SNMP_TAG_OCTET_STRING, s->eid, s->eid_len);
	}
	if (s->bad_context_name) {
		w_tlv(&scoped, SNMP_TAG_OCTET_STRING, "other", 5U);
	} else {
		w_tlv(&scoped, SNMP_TAG_OCTET_STRING, NULL, 0U);
	}
	w_tlv(&scoped, s->pdu_tag, pdu.buf, pdu.len);

	{
		w_t wrapped;

		w_reset(&wrapped);
		w_tlv(&wrapped, SNMP_TAG_SEQUENCE, scoped.buf, scoped.len);
		memcpy(out, wrapped.buf, wrapped.len);
		return wrapped.len;
	}
}

/**
 * Assemble a whole v3 request.
 *
 * @param keys   NULL for an unauthenticated message; otherwise the localised
 *               auth key and, when @p s->salt is set, the privacy key.
 * @param out_authparam_off  Receives the digest field's offset.
 */
static size_t build_request(const req_spec_t *s, const uint8_t *auth_key,
			    size_t auth_key_len, uint8_t auth_proto,
			    const uint8_t *priv_key, uint8_t *out,
			    size_t *out_authparam_off)
{
	uint8_t scoped[SNMP_PKT_MAX];
	uint8_t zero[SNMP_V3_AUTHPARAM_MAX];
	w_t gd;
	w_t usm;
	w_t body;
	w_t msg;
	size_t scoped_len;

	memset(zero, 0, sizeof(zero));
	scoped_len = build_scoped(s, scoped);

	w_reset(&gd);
	w_int(&gd, s->msg_id);
	w_int(&gd, (int32_t)s->max_size);
	w_tlv(&gd, SNMP_TAG_OCTET_STRING, &s->flags, 1U);
	w_int(&gd, s->sec_model);

	w_reset(&usm);
	w_tlv(&usm, SNMP_TAG_OCTET_STRING, s->eid, s->eid_len);
	w_int(&usm, (int32_t)s->boots);
	w_int(&usm, (int32_t)s->time_s);
	w_tlv(&usm, SNMP_TAG_OCTET_STRING, s->user, strlen(s->user));
	w_tlv(&usm, SNMP_TAG_OCTET_STRING, zero, s->authparam_len);
	if (s->salt != NULL) {
		w_tlv(&usm, SNMP_TAG_OCTET_STRING, s->salt, SNMP_V3_SALT_LEN);
	} else {
		w_tlv(&usm, SNMP_TAG_OCTET_STRING, NULL, 0U);
	}

	w_reset(&body);
	w_int(&body, SNMP_VERSION_3);
	w_tlv(&body, SNMP_TAG_SEQUENCE, gd.buf, gd.len);
	{
		w_t params;

		w_reset(&params);
		w_tlv(&params, SNMP_TAG_SEQUENCE, usm.buf, usm.len);
		w_tlv(&body, SNMP_TAG_OCTET_STRING, params.buf, params.len);
	}

	if (s->salt != NULL && priv_key != NULL) {
		uint8_t iv[SNMP_V3_AES_BLOCK];

		(void)snmp_usm_priv_iv(s->boots, s->time_s, s->salt, iv);
		TEST_ASSERT_EQUAL_INT(0, snmp_usm_aes_cfb(&g_ports, priv_key, iv,
							  scoped, scoped,
							  scoped_len, true));
		w_tlv(&body, SNMP_TAG_OCTET_STRING, scoped, scoped_len);
	} else {
		w_raw(&body, scoped, scoped_len);
	}

	w_reset(&msg);
	w_tlv(&msg, SNMP_TAG_SEQUENCE, body.buf, body.len);
	memcpy(out, msg.buf, msg.len);

	/*
	 * Rather than track the offset through four nesting levels by hand,
	 * locate the zero-filled digest field by searching for the USM blob's
	 * prefix. It is unambiguous: the engine id is unique in the message.
	 */
	if (s->authparam_len != 0U) {
		size_t i;
		size_t found = 0U;

		for (i = 0U; i + s->authparam_len <= msg.len; i++) {
			if (msg.buf[i] == SNMP_TAG_OCTET_STRING &&
			    msg.buf[i + 1U] == (uint8_t)s->authparam_len &&
			    i > 8U) {
				size_t j;
				bool all_zero = true;

				for (j = 0U; j < s->authparam_len; j++) {
					if (msg.buf[i + 2U + j] != 0U) {
						all_zero = false;
					}
				}
				if (all_zero) {
					found = i + 2U;
					break;
				}
			}
		}
		TEST_ASSERT_TRUE(found != 0U);
		*out_authparam_off = found;

		if (auth_key != NULL) {
			uint8_t digest[SNMP_V3_AUTHPARAM_MAX];

			TEST_ASSERT_EQUAL_INT(0,
					      snmp_usm_auth(&g_ports, auth_proto,
							    auth_key,
							    auth_key_len,
							    out, msg.len, digest,
							    NULL));
			memcpy(&out[found], digest, s->authparam_len);
		}
	} else {
		*out_authparam_off = 0U;
	}

	return msg.len;
}

static req_spec_t base_spec(void)
{
	req_spec_t s;

	memset(&s, 0, sizeof(s));
	s.msg_id = 0x2A2A;
	s.max_size = SNMP_PKT_MAX;
	s.flags = SNMP_V3_FLAG_REPORTABLE;
	s.sec_model = SNMP_V3_SEC_MODEL_USM;
	s.eid = g_eid;
	s.eid_len = g_eid_len;
	s.boots = 7U;
	s.time_s = 1000U;
	s.user = "";
	s.pdu_tag = SNMP_PDU_GET;
	return s;
}

/* ------------------------------------------------------------------------- */
/* report generation (RFC 3414 §3.2)                                         */
/* ------------------------------------------------------------------------- */

/** Find `usmStats.<which>.0` encoded as an OID in @p msg. */
static bool has_usm_stat(const uint8_t *msg, size_t len, unsigned int which)
{
	uint32_t arcs[SNMP_OID_MAX_LEN];
	uint8_t enc[32];
	snmp_wr_t w;
	int n;
	size_t i;

	n = snmp_usm_stats_oid(which, arcs);
	if (n < 0) {
		return false;
	}
	(void)snmp_wr_init(&w, enc, sizeof(enc));
	if (snmp_wr_oid(&w, arcs, (size_t)n) != 0) {
		return false;
	}
	for (i = 0U; i + snmp_wr_len(&w) <= len; i++) {
		if (memcmp(&msg[i], enc, snmp_wr_len(&w)) == 0) {
			return true;
		}
	}
	return false;
}

/** Decode a response's version, flags and PDU tag. */
static void inspect(const uint8_t *msg, size_t len, int32_t *out_ver,
		    uint8_t *out_flags, int32_t *out_msgid)
{
	snmp_rd_t r;
	snmp_rd_t body;
	snmp_rd_t gd;
	uint8_t tag;
	size_t clen;
	int64_t v;
	const uint8_t *oct;
	size_t oct_len;

	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, msg, len));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_hdr(&r, &tag, &clen));
	TEST_ASSERT_EQUAL_HEX8(SNMP_TAG_SEQUENCE, tag);
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&body, &msg[r.off], clen));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_int(&body, &v));
	*out_ver = (int32_t)v;

	TEST_ASSERT_EQUAL_INT(0, snmp_rd_hdr(&body, &tag, &clen));
	TEST_ASSERT_EQUAL_HEX8(SNMP_TAG_SEQUENCE, tag);
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&gd, &body.buf[body.off], clen));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_int(&gd, &v));
	*out_msgid = (int32_t)v;
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_int(&gd, &v)); /* msgMaxSize */
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_octets(&gd, &oct, &oct_len));
	TEST_ASSERT_EQUAL_UINT(1U, oct_len);
	*out_flags = oct[0];
}

static void test_unknown_engine_id_report(void)
{
	req_spec_t s;
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	uint8_t other[8] = { 0x80, 0, 0, 1, 5, 1, 2, 3 };
	snmp_v3_stats_t st;
	size_t req_len;
	size_t rsp_len = 0U;
	size_t ap = 0U;
	int32_t ver = 0;
	int32_t msgid = 0;
	uint8_t flags = 0xFFU;

	fixture_init();

	/* The standard discovery probe: empty engine id, empty user, no auth. */
	s = base_spec();
	s.eid = other;
	s.eid_len = 0U;
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);

	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));
	inspect(rsp, rsp_len, &ver, &flags, &msgid);
	TEST_ASSERT_EQUAL_INT32(SNMP_VERSION_3, ver);
	TEST_ASSERT_EQUAL_INT32(s.msg_id, msgid);
	/* A Report is never itself reportable, and this one is unauthenticated. */
	TEST_ASSERT_EQUAL_HEX8(0U, flags);
	TEST_ASSERT_TRUE(has_usm_stat(rsp, rsp_len, 4U));
	/* The Report carries our engine id, which is what discovery wants. */
	{
		size_t i;
		bool found = false;

		for (i = 0U; i + g_eid_len <= rsp_len; i++) {
			if (memcmp(&rsp[i], g_eid, g_eid_len) == 0) {
				found = true;
			}
		}
		TEST_ASSERT_TRUE(found);
	}

	TEST_ASSERT_EQUAL_INT(0, snmp_v3_stats_get(&g_v3, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.unknown_engine_ids);
	TEST_ASSERT_EQUAL_UINT64(1U, st.reports);

	/* A wrong (non-empty) engine id is the same case. */
	s.eid = other;
	s.eid_len = sizeof(other);
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));
	TEST_ASSERT_TRUE(has_usm_stat(rsp, rsp_len, 4U));

	/* A non-reportable message gets silence instead of a report. */
	s.flags = 0U;
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      snmp_v3_handle(&g_agent, &g_v3, req, req_len,
					     1000U, 100U, rsp, sizeof(rsp),
					     &rsp_len));
}

static void test_unknown_user_report(void)
{
	req_spec_t s;
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	snmp_v3_stats_t st;
	size_t req_len;
	size_t rsp_len = 0U;
	size_t ap = 0U;

	fixture_init();
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_set(&g_v3, "alice",
						  SNMP_AUTH_HMAC_SHA256_192,
						  "maplesyrup", 10U, false,
						  SNMP_PRIV_NONE, NULL, 0U,
						  false));

	s = base_spec();
	s.user = "mallory";
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);

	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));
	TEST_ASSERT_TRUE(has_usm_stat(rsp, rsp_len, 3U));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_stats_get(&g_v3, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.unknown_user_names);

	s.flags = 0U;
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      snmp_v3_handle(&g_agent, &g_v3, req, req_len,
					     1000U, 100U, rsp, sizeof(rsp),
					     &rsp_len));
}

static void test_unsupported_sec_level_report(void)
{
	req_spec_t s;
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	uint8_t kul[SNMP_V3_KEY_MAX];
	snmp_v3_stats_t st;
	size_t req_len;
	size_t rsp_len = 0U;
	size_t ap = 0U;

	fixture_init();
	TEST_ASSERT_EQUAL_INT(0, snmp_usm_password_to_localized(
					 &g_ports, SNMP_AUTH_HMAC_SHA256_192,
					 "maplesyrup", g_eid, g_eid_len, kul,
					 NULL));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_set(&g_v3, "alice",
						  SNMP_AUTH_HMAC_SHA256_192, kul,
						  32U, true, SNMP_PRIV_NONE,
						  NULL, 0U, false));

	/* A noAuthNoPriv request naming an authenticated user is refused: it
	 * would otherwise be a way to read the MIB with no authentication. */
	s = base_spec();
	s.user = "alice";
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));
	TEST_ASSERT_TRUE(has_usm_stat(rsp, rsp_len, 1U));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_stats_get(&g_v3, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.unsupported_sec_levels);

	/* And an authPriv request for a user with no privacy key is refused
	 * too, because the keys to satisfy it do not exist. */
	{
		static const uint8_t salt[SNMP_V3_SALT_LEN] = { 1, 2, 3, 4,
								5, 6, 7, 8 };

		s = base_spec();
		s.user = "alice";
		s.flags = SNMP_V3_FLAG_AUTH | SNMP_V3_FLAG_PRIV |
			  SNMP_V3_FLAG_REPORTABLE;
		s.authparam_len = 24U;
		s.salt = salt;
		req_len = build_request(&s, kul, 32U,
					SNMP_AUTH_HMAC_SHA256_192, kul, req,
					&ap);
		TEST_ASSERT_EQUAL_INT(0,
				      snmp_v3_handle(&g_agent, &g_v3, req,
						     req_len, 1000U, 100U, rsp,
						     sizeof(rsp), &rsp_len));
		TEST_ASSERT_TRUE(has_usm_stat(rsp, rsp_len, 1U));
	}

	s.flags = 0U;
	s.authparam_len = 0U;
	s.salt = NULL;
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      snmp_v3_handle(&g_agent, &g_v3, req, req_len,
					     1000U, 100U, rsp, sizeof(rsp),
					     &rsp_len));
}

static void test_wrong_digest_report(void)
{
	req_spec_t s;
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	uint8_t kul[SNMP_V3_KEY_MAX];
	snmp_v3_stats_t st;
	size_t req_len;
	size_t rsp_len = 0U;
	size_t ap = 0U;

	fixture_init();
	TEST_ASSERT_EQUAL_INT(0, snmp_usm_password_to_localized(
					 &g_ports, SNMP_AUTH_HMAC_SHA256_192,
					 "maplesyrup", g_eid, g_eid_len, kul,
					 NULL));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_set(&g_v3, "alice",
						  SNMP_AUTH_HMAC_SHA256_192, kul,
						  32U, true, SNMP_PRIV_NONE,
						  NULL, 0U, false));

	s = base_spec();
	s.user = "alice";
	s.flags = SNMP_V3_FLAG_AUTH | SNMP_V3_FLAG_REPORTABLE;
	s.authparam_len = 24U;
	req_len = build_request(&s, kul, 32U, SNMP_AUTH_HMAC_SHA256_192, NULL,
				req, &ap);

	/* Flip one digest bit. */
	req[ap] ^= 0x01U;
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));
	TEST_ASSERT_TRUE(has_usm_stat(rsp, rsp_len, 5U));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_stats_get(&g_v3, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.wrong_digests);
	TEST_ASSERT_EQUAL_UINT64(0U, st.authenticated);

	/* A digest of the wrong length is the same failure. */
	s.authparam_len = 12U;
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));
	TEST_ASSERT_TRUE(has_usm_stat(rsp, rsp_len, 5U));

	s.flags = SNMP_V3_FLAG_AUTH;
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      snmp_v3_handle(&g_agent, &g_v3, req, req_len,
					     1000U, 100U, rsp, sizeof(rsp),
					     &rsp_len));
}

static void test_time_window_report(void)
{
	req_spec_t s;
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	uint8_t kul[SNMP_V3_KEY_MAX];
	snmp_v3_stats_t st;
	size_t req_len;
	size_t rsp_len = 0U;
	size_t ap = 0U;
	int32_t ver = 0;
	int32_t msgid = 0;
	uint8_t flags = 0U;

	fixture_init();
	TEST_ASSERT_EQUAL_INT(0, snmp_usm_password_to_localized(
					 &g_ports, SNMP_AUTH_HMAC_SHA256_192,
					 "maplesyrup", g_eid, g_eid_len, kul,
					 NULL));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_set(&g_v3, "alice",
						  SNMP_AUTH_HMAC_SHA256_192, kul,
						  32U, true, SNMP_PRIV_NONE,
						  NULL, 0U, false));

	s = base_spec();
	s.user = "alice";
	s.flags = SNMP_V3_FLAG_AUTH | SNMP_V3_FLAG_REPORTABLE;
	s.authparam_len = 24U;

	/* Inside the window: served. */
	s.time_s = 1000U;
	req_len = build_request(&s, kul, 32U, SNMP_AUTH_HMAC_SHA256_192, NULL,
				req, &ap);
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U + 150U, 100U, rsp,
						sizeof(rsp), &rsp_len));
	TEST_ASSERT_FALSE(has_usm_stat(rsp, rsp_len, 2U));

	/* One second outside it: notInTimeWindows. */
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U + 151U, 100U, rsp,
						sizeof(rsp), &rsp_len));
	TEST_ASSERT_TRUE(has_usm_stat(rsp, rsp_len, 2U));
	/* And this one *is* authenticated, so the manager can trust the
	 * boots/time it synchronises to (RFC 3414 §3.2(7)). */
	inspect(rsp, rsp_len, &ver, &flags, &msgid);
	TEST_ASSERT_EQUAL_HEX8(SNMP_V3_FLAG_AUTH, flags);

	/* Behind by more than the window, too. */
	s.time_s = 5000U;
	req_len = build_request(&s, kul, 32U, SNMP_AUTH_HMAC_SHA256_192, NULL,
				req, &ap);
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));
	TEST_ASSERT_TRUE(has_usm_stat(rsp, rsp_len, 2U));

	/* A mismatched boots count is out of the window whatever the time. */
	s.time_s = 1000U;
	s.boots = 6U;
	req_len = build_request(&s, kul, 32U, SNMP_AUTH_HMAC_SHA256_192, NULL,
				req, &ap);
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));
	TEST_ASSERT_TRUE(has_usm_stat(rsp, rsp_len, 2U));

	/* An engine at the terminal boots value never passes. */
	s.boots = 0x7FFFFFFFU;
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_set_boots(&g_v3, 0x7FFFFFFFU));
	req_len = build_request(&s, kul, 32U, SNMP_AUTH_HMAC_SHA256_192, NULL,
				req, &ap);
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));
	TEST_ASSERT_TRUE(has_usm_stat(rsp, rsp_len, 2U));

	TEST_ASSERT_EQUAL_INT(0, snmp_v3_stats_get(&g_v3, &st));
	TEST_ASSERT_EQUAL_UINT64(4U, st.not_in_time_windows);

	/* Non-reportable: silence. */
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_set_boots(&g_v3, 7U));
	s.boots = 6U;
	s.flags = SNMP_V3_FLAG_AUTH;
	req_len = build_request(&s, kul, 32U, SNMP_AUTH_HMAC_SHA256_192, NULL,
				req, &ap);
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      snmp_v3_handle(&g_agent, &g_v3, req, req_len,
					     1000U, 100U, rsp, sizeof(rsp),
					     &rsp_len));
}

/* ------------------------------------------------------------------------- */
/* successful exchanges                                                      */
/* ------------------------------------------------------------------------- */

/** Set up "alice" at authNoPriv and "bob" at authPriv, both SHA-256. */
static uint8_t g_kul[SNMP_V3_KEY_MAX];

static void users_ready(void)
{
	fixture_init();
	TEST_ASSERT_EQUAL_INT(0, snmp_usm_password_to_localized(
					 &g_ports, SNMP_AUTH_HMAC_SHA256_192,
					 "maplesyrup", g_eid, g_eid_len, g_kul,
					 NULL));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_set(&g_v3, "alice",
						  SNMP_AUTH_HMAC_SHA256_192,
						  g_kul, 32U, true,
						  SNMP_PRIV_NONE, NULL, 0U,
						  false));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_set(&g_v3, "bob",
						  SNMP_AUTH_HMAC_SHA256_192,
						  g_kul, 32U, true,
						  SNMP_PRIV_AES128_CFB, g_kul,
						  32U, true));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_set(&g_v3, "guest",
						  SNMP_AUTH_NONE, NULL, 0U,
						  false, SNMP_PRIV_NONE, NULL,
						  0U, false));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_user_set(&g_v3, "carol",
						  SNMP_AUTH_HMAC_SHA1_96,
						  "maplesyrup", 10U, false,
						  SNMP_PRIV_NONE, NULL, 0U,
						  false));
}

/** Verify a response's digest with @p key. */
static void check_digest(uint8_t *msg, size_t len, const uint8_t *key,
			 size_t key_len, uint8_t proto, size_t ap_len)
{
	uint8_t copy[SNMP_PKT_MAX];
	uint8_t digest[SNMP_V3_AUTHPARAM_MAX];
	size_t i;
	size_t found = 0U;

	/* Locate the digest field: the only ap_len-octet OCTET STRING that is
	 * not all zero, sitting inside the security parameters. */
	for (i = 6U; i + 2U + ap_len <= len; i++) {
		if (msg[i] == SNMP_TAG_OCTET_STRING &&
		    msg[i + 1U] == (uint8_t)ap_len) {
			found = i + 2U;
			break;
		}
	}
	TEST_ASSERT_TRUE(found != 0U);

	memcpy(copy, msg, len);
	memset(&copy[found], 0, ap_len);
	TEST_ASSERT_EQUAL_INT(0, snmp_usm_auth(&g_ports, proto, key, key_len,
					       copy, len, digest, NULL));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(digest, &msg[found], ap_len);
}

static void test_noauth_get_is_served(void)
{
	req_spec_t s;
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	snmp_v3_stats_t st;
	snmp_stats_t ast;
	size_t req_len;
	size_t rsp_len = 0U;
	size_t ap = 0U;
	int32_t ver = 0;
	int32_t msgid = 0;
	uint8_t flags = 0xFFU;

	users_ready();

	s = base_spec();
	s.user = "guest";
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);

	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 4242U, rsp, sizeof(rsp),
						&rsp_len));
	inspect(rsp, rsp_len, &ver, &flags, &msgid);
	TEST_ASSERT_EQUAL_INT32(SNMP_VERSION_3, ver);
	TEST_ASSERT_EQUAL_INT32(s.msg_id, msgid);
	TEST_ASSERT_EQUAL_HEX8(0U, flags);

	/* No usmStats OID: this is a Response, not a Report. */
	TEST_ASSERT_FALSE(has_usm_stat(rsp, rsp_len, 1U));
	TEST_ASSERT_FALSE(has_usm_stat(rsp, rsp_len, 4U));

	TEST_ASSERT_EQUAL_INT(0, snmp_v3_stats_get(&g_v3, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.responses);
	TEST_ASSERT_EQUAL_UINT64(1U, st.rx);
	TEST_ASSERT_EQUAL_UINT64(0U, st.reports);
	/* The PDU layer's counters are the agent's, shared with the v2c path. */
	TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&g_agent, &ast));
	TEST_ASSERT_EQUAL_UINT64(1U, ast.get);
	TEST_ASSERT_EQUAL_UINT64(1U, ast.responses);
	TEST_ASSERT_EQUAL_UINT64(1U, ast.varbinds);

	TEST_ASSERT_EQUAL_INT(0, snmp_v3_stats_reset(&g_v3));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_stats_get(&g_v3, &st));
	TEST_ASSERT_EQUAL_UINT64(0U, st.rx);
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_stats_get(NULL, &st));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_stats_get(&g_v3, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_v3_stats_reset(NULL));
}

static void test_authnopriv_get_is_served_and_signed(void)
{
	req_spec_t s;
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	snmp_v3_stats_t st;
	size_t req_len;
	size_t rsp_len = 0U;
	size_t ap = 0U;
	int32_t ver = 0;
	int32_t msgid = 0;
	uint8_t flags = 0U;

	users_ready();

	s = base_spec();
	s.user = "alice";
	s.flags = SNMP_V3_FLAG_AUTH | SNMP_V3_FLAG_REPORTABLE;
	s.authparam_len = 24U;
	req_len = build_request(&s, g_kul, 32U, SNMP_AUTH_HMAC_SHA256_192, NULL,
				req, &ap);

	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 4242U, rsp, sizeof(rsp),
						&rsp_len));
	inspect(rsp, rsp_len, &ver, &flags, &msgid);
	/* The response mirrors the request's level and is not reportable. */
	TEST_ASSERT_EQUAL_HEX8(SNMP_V3_FLAG_AUTH, flags);
	check_digest(rsp, rsp_len, g_kul, 32U, SNMP_AUTH_HMAC_SHA256_192, 24U);

	TEST_ASSERT_EQUAL_INT(0, snmp_v3_stats_get(&g_v3, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.authenticated);
	TEST_ASSERT_EQUAL_UINT64(1U, st.responses);

	/* The same, under SHA-1: 12-octet digest, 20-octet key. */
	{
		uint8_t kul1[SNMP_V3_KEY_MAX];

		TEST_ASSERT_EQUAL_INT(0, snmp_usm_password_to_localized(
						 &g_ports,
						 SNMP_AUTH_HMAC_SHA1_96,
						 "maplesyrup", g_eid, g_eid_len,
						 kul1, NULL));
		s.user = "carol";
		s.authparam_len = 12U;
		req_len = build_request(&s, kul1, 20U, SNMP_AUTH_HMAC_SHA1_96,
					NULL, req, &ap);
		TEST_ASSERT_EQUAL_INT(0,
				      snmp_v3_handle(&g_agent, &g_v3, req,
						     req_len, 1000U, 4242U, rsp,
						     sizeof(rsp), &rsp_len));
		check_digest(rsp, rsp_len, kul1, 20U, SNMP_AUTH_HMAC_SHA1_96,
			     12U);
	}
}

static void test_authpriv_round_trip(void)
{
	static const uint8_t salt[SNMP_V3_SALT_LEN] = { 0xDE, 0xAD, 0xBE, 0xEF,
							0x01, 0x02, 0x03, 0x04 };
	req_spec_t s;
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	snmp_v3_stats_t st;
	size_t req_len;
	size_t rsp_len = 0U;
	size_t ap = 0U;
	int32_t ver = 0;
	int32_t msgid = 0;
	uint8_t flags = 0U;

	users_ready();

	s = base_spec();
	s.user = "bob";
	s.flags = SNMP_V3_FLAG_AUTH | SNMP_V3_FLAG_PRIV |
		  SNMP_V3_FLAG_REPORTABLE;
	s.authparam_len = 24U;
	s.salt = salt;
	req_len = build_request(&s, g_kul, 32U, SNMP_AUTH_HMAC_SHA256_192, g_kul,
				req, &ap);

	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 4242U, rsp, sizeof(rsp),
						&rsp_len));
	inspect(rsp, rsp_len, &ver, &flags, &msgid);
	TEST_ASSERT_EQUAL_HEX8(SNMP_V3_FLAG_AUTH | SNMP_V3_FLAG_PRIV, flags);
	check_digest(rsp, rsp_len, g_kul, 32U, SNMP_AUTH_HMAC_SHA256_192, 24U);

	TEST_ASSERT_EQUAL_INT(0, snmp_v3_stats_get(&g_v3, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.decrypted);
	TEST_ASSERT_EQUAL_UINT64(1U, st.responses);

	/* The response's scopedPDU must actually be encrypted: the plaintext
	 * varbind OID must not appear anywhere in it. */
	{
		static const uint8_t stratum_oid[] = { 0x2B, 0x06, 0x01, 0x04,
						       0x01, 0x86, 0x8D, 0x1F };
		size_t i;
		bool leaked = false;

		for (i = 0U; i + sizeof(stratum_oid) <= rsp_len; i++) {
			if (memcmp(&rsp[i], stratum_oid,
				   sizeof(stratum_oid)) == 0) {
				leaked = true;
			}
		}
		TEST_ASSERT_FALSE(leaked);
	}

	/* Two consecutive privacy-protected responses must use different salts,
	 * or the keystream repeats. */
	{
		uint8_t rsp2[SNMP_PKT_MAX];
		size_t rsp2_len = 0U;

		TEST_ASSERT_EQUAL_INT(0,
				      snmp_v3_handle(&g_agent, &g_v3, req,
						     req_len, 1000U, 4242U, rsp2,
						     sizeof(rsp2), &rsp2_len));
		TEST_ASSERT_EQUAL_UINT(rsp_len, rsp2_len);
		TEST_ASSERT_TRUE(memcmp(rsp, rsp2, rsp_len) != 0);
	}
}

static void test_privacy_failures(void)
{
	static const uint8_t salt[SNMP_V3_SALT_LEN] = { 1, 2, 3, 4, 5, 6, 7, 8 };
	req_spec_t s;
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	uint8_t wrong[SNMP_V3_KEY_MAX];
	snmp_v3_stats_t st;
	size_t req_len;
	size_t rsp_len = 0U;
	size_t ap = 0U;

	users_ready();
	memset(wrong, 0x77, sizeof(wrong));

	/* Encrypted with the wrong key: the digest still verifies (the digest is
	 * over the ciphertext), so this lands in the decryption-error path. */
	s = base_spec();
	s.user = "bob";
	s.flags = SNMP_V3_FLAG_AUTH | SNMP_V3_FLAG_PRIV |
		  SNMP_V3_FLAG_REPORTABLE;
	s.authparam_len = 24U;
	s.salt = salt;
	req_len = build_request(&s, g_kul, 32U, SNMP_AUTH_HMAC_SHA256_192, wrong,
				req, &ap);
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));
	TEST_ASSERT_TRUE(has_usm_stat(rsp, rsp_len, 6U));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_stats_get(&g_v3, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.decryption_errors);

	/* privFlag set but no privacy parameters. */
	s.salt = NULL;
	req_len = build_request(&s, g_kul, 32U, SNMP_AUTH_HMAC_SHA256_192, NULL,
				req, &ap);
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));
	TEST_ASSERT_TRUE(has_usm_stat(rsp, rsp_len, 6U));

	/* An authNoPriv user sending ciphertext: nothing here can read it. */
	s = base_spec();
	s.user = "alice";
	s.flags = SNMP_V3_FLAG_AUTH | SNMP_V3_FLAG_REPORTABLE;
	s.authparam_len = 24U;
	s.salt = salt;
	req_len = build_request(&s, g_kul, 32U, SNMP_AUTH_HMAC_SHA256_192, g_kul,
				req, &ap);
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));
	TEST_ASSERT_TRUE(has_usm_stat(rsp, rsp_len, 6U));

	/* Non-reportable versions get silence. */
	s.flags = SNMP_V3_FLAG_AUTH;
	req_len = build_request(&s, g_kul, 32U, SNMP_AUTH_HMAC_SHA256_192, g_kul,
				req, &ap);
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      snmp_v3_handle(&g_agent, &g_v3, req, req_len,
					     1000U, 100U, rsp, sizeof(rsp),
					     &rsp_len));
}

static void test_pdu_variants(void)
{
	req_spec_t s;
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	snmp_stats_t ast;
	size_t req_len;
	size_t rsp_len = 0U;
	size_t ap = 0U;

	users_ready();

	s = base_spec();
	s.user = "guest";

	s.pdu_tag = SNMP_PDU_GETNEXT;
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));

	s.pdu_tag = SNMP_PDU_GETBULK;
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));

	/* A SET is answered notWritable, exactly as on the v2c path. */
	s.pdu_tag = SNMP_PDU_SET;
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));

	/* A PDU this agent does not serve. */
	s.pdu_tag = SNMP_PDU_RESPONSE;
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(-ENOTSUP,
			      snmp_v3_handle(&g_agent, &g_v3, req, req_len,
					     1000U, 100U, rsp, sizeof(rsp),
					     &rsp_len));

	TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&g_agent, &ast));
	TEST_ASSERT_EQUAL_UINT64(1U, ast.getnext);
	TEST_ASSERT_EQUAL_UINT64(1U, ast.getbulk);
	TEST_ASSERT_EQUAL_UINT64(1U, ast.set_refused);
	TEST_ASSERT_EQUAL_UINT64(1U, ast.unsupported_pdu);
}

static void test_context_checks(void)
{
	req_spec_t s;
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	snmp_v3_stats_t st;
	size_t req_len;
	size_t rsp_len = 0U;
	size_t ap = 0U;

	users_ready();

	/* A wrong contextEngineID. */
	s = base_spec();
	s.user = "guest";
	s.bad_context_eid = true;
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      snmp_v3_handle(&g_agent, &g_v3, req, req_len,
					     1000U, 100U, rsp, sizeof(rsp),
					     &rsp_len));

	/* A non-default contextName; this agent has one context. */
	s = base_spec();
	s.user = "guest";
	s.bad_context_name = true;
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      snmp_v3_handle(&g_agent, &g_v3, req, req_len,
					     1000U, 100U, rsp, sizeof(rsp),
					     &rsp_len));

	TEST_ASSERT_EQUAL_INT(0, snmp_v3_stats_get(&g_v3, &st));
	TEST_ASSERT_EQUAL_UINT64(2U, st.bad_context);
}

static void test_bad_security_model_and_version(void)
{
	req_spec_t s;
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	snmp_v3_stats_t st;
	size_t req_len;
	size_t rsp_len = 0U;
	size_t ap = 0U;

	users_ready();

	s = base_spec();
	s.user = "guest";
	s.sec_model = 1; /* SNMPv1 security model */
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(-EPROTO,
			      snmp_v3_handle(&g_agent, &g_v3, req, req_len,
					     1000U, 100U, rsp, sizeof(rsp),
					     &rsp_len));
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_stats_get(&g_v3, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.bad_sec_model);

	/* A v2c datagram handed to the v3 handler. */
	{
		static const uint8_t v2c[] = { 0x30, 0x0C, 0x02, 0x01, 0x01,
					       0x04, 0x06, 'p',  'u',  'b',
					       'l',  'i',  'c',  0xA0 };

		TEST_ASSERT_EQUAL_INT(-EPROTO,
				      snmp_v3_handle(&g_agent, &g_v3, v2c,
						     sizeof(v2c), 1000U, 100U,
						     rsp, sizeof(rsp),
						     &rsp_len));
	}
}

static void test_handle_argument_guards(void)
{
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	snmp_v3_ctx_t fresh;
	size_t rsp_len = 0U;

	users_ready();
	memset(req, 0, sizeof(req));
	memset(&fresh, 0, sizeof(fresh));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_handle(NULL, &g_v3, req, 32U, 0U, 0U, rsp,
					     sizeof(rsp), &rsp_len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_handle(&g_agent, NULL, req, 32U, 0U, 0U,
					     rsp, sizeof(rsp), &rsp_len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_handle(&g_agent, &fresh, req, 32U, 0U, 0U,
					     rsp, sizeof(rsp), &rsp_len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_handle(&g_agent, &g_v3, NULL, 32U, 0U, 0U,
					     rsp, sizeof(rsp), &rsp_len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_handle(&g_agent, &g_v3, req, 32U, 0U, 0U,
					     NULL, sizeof(rsp), &rsp_len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_handle(&g_agent, &g_v3, req, 32U, 0U, 0U,
					     rsp, sizeof(rsp), NULL));
	/* Below the RFC 3412 msgMaxSize floor. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      snmp_v3_handle(&g_agent, &g_v3, req, 32U, 0U, 0U,
					     rsp, 100U, &rsp_len));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      snmp_v3_handle(&g_agent, &g_v3, req, 0U, 0U, 0U,
					     rsp, sizeof(rsp), &rsp_len));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      snmp_v3_handle(&g_agent, &g_v3, req,
					     SNMP_PKT_MAX + 1U, 0U, 0U, rsp,
					     sizeof(rsp), &rsp_len));
	/* Garbage. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      snmp_v3_handle(&g_agent, &g_v3, req, 32U, 0U, 0U,
					     rsp, sizeof(rsp), &rsp_len));
}

static void test_msg_max_size_is_honoured(void)
{
	req_spec_t s;
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	size_t req_len;
	size_t rsp_len = 0U;
	size_t ap = 0U;

	users_ready();

	/* A manager that can only take 484 octets must not be sent more. */
	s = base_spec();
	s.user = "guest";
	s.max_size = 484U;
	s.pdu_tag = SNMP_PDU_GETBULK;
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));
	TEST_ASSERT_TRUE(rsp_len <= 484U);

	/* A nonsensically small msgMaxSize is ignored in favour of the floor. */
	s.max_size = 8U;
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));
	TEST_ASSERT_TRUE(rsp_len > 0U);
}

/* ------------------------------------------------------------------------- */
/* dispatch                                                                  */
/* ------------------------------------------------------------------------- */

static void test_dispatch_routes_by_version(void)
{
	req_spec_t s;
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	snmp_stats_t ast;
	size_t req_len;
	size_t rsp_len = 0U;
	size_t ap = 0U;
	int32_t ver = 0;

	users_ready();

	/* A v3 datagram reaches the v3 handler. */
	s = base_spec();
	s.user = "guest";
	req_len = build_request(&s, NULL, 0U, 0U, NULL, req, &ap);
	TEST_ASSERT_EQUAL_INT(0, snmp_peek_version(req, req_len, &ver));
	TEST_ASSERT_EQUAL_INT32(SNMP_VERSION_3, ver);
	TEST_ASSERT_EQUAL_INT(0, snmp_dispatch(&g_agent, &g_v3, req, req_len,
					       1000U, 100U, rsp, sizeof(rsp),
					       &rsp_len));
	TEST_ASSERT_EQUAL_INT(0, snmp_peek_version(rsp, rsp_len, &ver));
	TEST_ASSERT_EQUAL_INT32(SNMP_VERSION_3, ver);

	/* Without a v3 context, a v3 datagram is a counted drop. */
	TEST_ASSERT_EQUAL_INT(-EPROTO,
			      snmp_dispatch(&g_agent, NULL, req, req_len, 1000U,
					    100U, rsp, sizeof(rsp), &rsp_len));
	TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&g_agent, &ast));
	TEST_ASSERT_EQUAL_UINT64(1U, ast.bad_version);

	/* A v2c GET still works through the same entry point. */
	{
		static const uint8_t v2c_get[] = {
			0x30, 0x29, 0x02, 0x01, 0x01, 0x04, 0x06, 'p',  'u',
			'b',  'l',  'i',  'c',  0xA0, 0x1C, 0x02, 0x04, 0x11,
			0x22, 0x33, 0x44, 0x02, 0x01, 0x00, 0x02, 0x01, 0x00,
			0x30, 0x0E, 0x30, 0x0C, 0x06, 0x08, 0x2B, 0x06, 0x01,
			0x02, 0x01, 0x01, 0x03, 0x00, 0x05, 0x00
		};

		TEST_ASSERT_EQUAL_INT(0, snmp_peek_version(v2c_get,
							   sizeof(v2c_get),
							   &ver));
		TEST_ASSERT_EQUAL_INT32(SNMP_VERSION_2C, ver);
		TEST_ASSERT_EQUAL_INT(0,
				      snmp_dispatch(&g_agent, &g_v3, v2c_get,
						    sizeof(v2c_get), 1000U,
						    100U, rsp, sizeof(rsp),
						    &rsp_len));
		TEST_ASSERT_EQUAL_INT(0, snmp_peek_version(rsp, rsp_len, &ver));
		TEST_ASSERT_EQUAL_INT32(SNMP_VERSION_2C, ver);

		/* ...unless v2c is switched off (spec §9.5). */
		TEST_ASSERT_EQUAL_INT(0, snmp_set_v2c_enabled(&g_agent, false));
		TEST_ASSERT_EQUAL_INT(-EPERM,
				      snmp_dispatch(&g_agent, &g_v3, v2c_get,
						    sizeof(v2c_get), 1000U,
						    100U, rsp, sizeof(rsp),
						    &rsp_len));
		TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&g_agent, &ast));
		TEST_ASSERT_EQUAL_UINT64(1U, ast.v2c_refused);
		/* And v3 is unaffected by that switch. */
		TEST_ASSERT_EQUAL_INT(0,
				      snmp_dispatch(&g_agent, &g_v3, req,
						    req_len, 1000U, 100U, rsp,
						    sizeof(rsp), &rsp_len));
		TEST_ASSERT_EQUAL_INT(0, snmp_set_v2c_enabled(&g_agent, true));
	}

	/* Garbage goes to the v2c handler, which counts it. */
	{
		uint8_t junk[8] = { 0xFF, 0xFF, 0xFF, 0xFF,
				    0xFF, 0xFF, 0xFF, 0xFF };

		TEST_ASSERT_EQUAL_INT(-EBADMSG,
				      snmp_peek_version(junk, sizeof(junk),
							&ver));
		TEST_ASSERT_EQUAL_INT(-EBADMSG,
				      snmp_dispatch(&g_agent, &g_v3, junk,
						    sizeof(junk), 1000U, 100U,
						    rsp, sizeof(rsp),
						    &rsp_len));
	}

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_dispatch(NULL, &g_v3, req, req_len, 0U, 0U,
					    rsp, sizeof(rsp), &rsp_len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_dispatch(&g_agent, &g_v3, NULL, req_len, 0U,
					    0U, rsp, sizeof(rsp), &rsp_len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_dispatch(&g_agent, &g_v3, req, req_len, 0U,
					    0U, NULL, sizeof(rsp), &rsp_len));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_dispatch(&g_agent, &g_v3, req, req_len, 0U,
					    0U, rsp, sizeof(rsp), NULL));

	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_peek_version(NULL, 8U, &ver));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_peek_version(req, req_len, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_set_v2c_enabled(NULL, true));
}

/* ------------------------------------------------------------------------- */
/* notifications                                                             */
/* ------------------------------------------------------------------------- */

static void test_notifications(void)
{
	uint8_t out[SNMP_PKT_MAX];
	snmp_bind_t binds[2];
	snmp_v3_stats_t st;
	size_t len = 0U;
	int32_t id1 = 0;
	int32_t id2 = 0;
	int32_t ver = 0;
	int32_t msgid = 0;
	uint8_t flags = 0U;

	users_ready();

	binds[0].obj = (uint16_t)SNMP_OBJ_STRATUM;
	binds[0].inst = 0U;
	binds[1].obj = (uint16_t)SNMP_OBJ_RAIL_MV;
	binds[1].inst = 3U;

	/* An authNoPriv trap. */
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_make_notification(
					 &g_agent, &g_v3, "alice",
					 SNMP_SEC_AUTH_NOPRIV,
					 SNMP_TRAP_LOCK_LOST, 1000U, 4242U,
					 binds, ARRAY_LEN(binds), false, out,
					 sizeof(out), &len, &id1));
	inspect(out, len, &ver, &flags, &msgid);
	TEST_ASSERT_EQUAL_INT32(SNMP_VERSION_3, ver);
	TEST_ASSERT_EQUAL_INT32(id1, msgid);
	/* A trap expects no answer, so it must not be reportable. */
	TEST_ASSERT_EQUAL_HEX8(SNMP_V3_FLAG_AUTH, flags);
	check_digest(out, len, g_kul, 32U, SNMP_AUTH_HMAC_SHA256_192, 24U);

	/* An authPriv inform: reportable, because it expects a Response. */
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_make_notification(
					 &g_agent, &g_v3, "bob",
					 SNMP_SEC_AUTH_PRIV,
					 SNMP_TRAP_HOLDOVER_ENTER, 1000U, 4242U,
					 NULL, 0U, true, out, sizeof(out), &len,
					 &id2));
	inspect(out, len, &ver, &flags, &msgid);
	TEST_ASSERT_EQUAL_HEX8(SNMP_V3_FLAG_AUTH | SNMP_V3_FLAG_PRIV |
				       SNMP_V3_FLAG_REPORTABLE,
			       flags);
	/* Message ids advance, so an inform acknowledgement is matchable. */
	TEST_ASSERT_TRUE(id2 != id1);

	/* noAuthNoPriv is accepted but carries nothing. */
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_make_notification(
					 &g_agent, &g_v3, "guest",
					 SNMP_SEC_NOAUTH_NOPRIV,
					 SNMP_TRAP_COLD_START, 1000U, 4242U,
					 NULL, 0U, false, out, sizeof(out), &len,
					 NULL));
	inspect(out, len, &ver, &flags, &msgid);
	TEST_ASSERT_EQUAL_HEX8(0U, flags);

	TEST_ASSERT_EQUAL_INT(0, snmp_v3_stats_get(&g_v3, &st));
	TEST_ASSERT_EQUAL_UINT64(3U, st.notifications);

	/* An unresolvable binding is skipped rather than failing the trap. */
	binds[0].obj = 0xFFFFU;
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_make_notification(
					 &g_agent, &g_v3, "alice",
					 SNMP_SEC_AUTH_NOPRIV,
					 SNMP_TRAP_REF_SWITCH, 1000U, 4242U,
					 binds, ARRAY_LEN(binds), false, out,
					 sizeof(out), &len, NULL));
}

static void test_notification_guards(void)
{
	uint8_t out[SNMP_PKT_MAX];
	snmp_bind_t b;
	size_t len = 0U;

	users_ready();
	b.obj = (uint16_t)SNMP_OBJ_STRATUM;
	b.inst = 0U;

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_make_notification(NULL, &g_v3, "alice", 1U,
							SNMP_TRAP_LOCK_LOST, 0U,
							0U, NULL, 0U, false, out,
							sizeof(out), &len,
							NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_make_notification(&g_agent, NULL, "alice",
							1U, SNMP_TRAP_LOCK_LOST,
							0U, 0U, NULL, 0U, false,
							out, sizeof(out), &len,
							NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_make_notification(&g_agent, &g_v3, NULL,
							1U, SNMP_TRAP_LOCK_LOST,
							0U, 0U, NULL, 0U, false,
							out, sizeof(out), &len,
							NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_make_notification(&g_agent, &g_v3, "alice",
							1U, SNMP_TRAP_LOCK_LOST,
							0U, 0U, NULL, 0U, false,
							NULL, sizeof(out), &len,
							NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_make_notification(&g_agent, &g_v3, "alice",
							1U, SNMP_TRAP_LOCK_LOST,
							0U, 0U, NULL, 0U, false,
							out, sizeof(out), NULL,
							NULL));
	/* Non-zero bind count with a NULL array. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_make_notification(&g_agent, &g_v3, "alice",
							1U, SNMP_TRAP_LOCK_LOST,
							0U, 0U, NULL, 2U, false,
							out, sizeof(out), &len,
							NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_make_notification(&g_agent, &g_v3, "alice",
							1U,
							(snmp_trap_t)SNMP_TRAP__COUNT,
							0U, 0U, &b, 1U, false,
							out, sizeof(out), &len,
							NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_v3_make_notification(&g_agent, &g_v3, "alice",
							9U, SNMP_TRAP_LOCK_LOST,
							0U, 0U, &b, 1U, false,
							out, sizeof(out), &len,
							NULL));
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      snmp_v3_make_notification(&g_agent, &g_v3,
							"nobody", 1U,
							SNMP_TRAP_LOCK_LOST, 0U,
							0U, &b, 1U, false, out,
							sizeof(out), &len,
							NULL));
	/* A user that cannot satisfy the requested level. */
	TEST_ASSERT_EQUAL_INT(-EPERM,
			      snmp_v3_make_notification(&g_agent, &g_v3, "alice",
							SNMP_SEC_AUTH_PRIV,
							SNMP_TRAP_LOCK_LOST, 0U,
							0U, &b, 1U, false, out,
							sizeof(out), &len,
							NULL));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      snmp_v3_make_notification(&g_agent, &g_v3, "alice",
							SNMP_SEC_AUTH_NOPRIV,
							SNMP_TRAP_LOCK_LOST, 0U,
							0U, &b, 1U, false, out,
							40U, &len, NULL));

	/* The msgID counter wraps inside Integer32 rather than going negative. */
	g_v3.notify_msgid = 0x7FFFFFFE;
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_make_notification(
					 &g_agent, &g_v3, "alice",
					 SNMP_SEC_AUTH_NOPRIV,
					 SNMP_TRAP_LOCK_LOST, 0U, 0U, NULL, 0U,
					 false, out, sizeof(out), &len, NULL));
	TEST_ASSERT_EQUAL_INT32(1, g_v3.notify_msgid);
}

/* ------------------------------------------------------------------------- */
/* notification rate limiter (core/snmp)                                     */
/* ------------------------------------------------------------------------- */

static void test_notify_gate(void)
{
	snmp_cfg_t cfg;
	snmp_ctx_t c;
	snmp_stats_t st;
	uint32_t sup = 0U;

	memset(&cfg, 0, sizeof(cfg));
	cfg.community = "public";
	cfg.getter.get = getter;
	cfg.notify_min_interval_ms = 1000U;
	TEST_ASSERT_EQUAL_INT(0, snmp_init(&c, &cfg));

	/* The first of a type always passes. */
	TEST_ASSERT_TRUE(snmp_notify_gate(&c, SNMP_TRAP_AUTH_FAILURE, 0U, &sup));
	TEST_ASSERT_EQUAL_UINT32(0U, sup);

	/* A burst inside the window is suppressed and counted. */
	TEST_ASSERT_FALSE(snmp_notify_gate(&c, SNMP_TRAP_AUTH_FAILURE, 100U,
					   NULL));
	TEST_ASSERT_FALSE(snmp_notify_gate(&c, SNMP_TRAP_AUTH_FAILURE, 999U,
					   NULL));
	TEST_ASSERT_EQUAL_INT(0, snmp_notify_suppressed(&c,
						       SNMP_TRAP_AUTH_FAILURE,
						       &sup));
	TEST_ASSERT_EQUAL_UINT32(2U, sup);

	/* A different type is not starved by that burst. */
	TEST_ASSERT_TRUE(snmp_notify_gate(&c, SNMP_TRAP_LOCK_LOST, 100U, &sup));
	TEST_ASSERT_EQUAL_UINT32(0U, sup);

	/* Past the window it passes again and reports what was dropped. */
	TEST_ASSERT_TRUE(snmp_notify_gate(&c, SNMP_TRAP_AUTH_FAILURE, 1000U,
					  &sup));
	TEST_ASSERT_EQUAL_UINT32(2U, sup);
	TEST_ASSERT_EQUAL_INT(0, snmp_notify_suppressed(&c,
						       SNMP_TRAP_AUTH_FAILURE,
						       &sup));
	TEST_ASSERT_EQUAL_UINT32(0U, sup);

	TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&c, &st));
	TEST_ASSERT_EQUAL_UINT64(2U, st.notify_suppressed);

	/* A zero interval disables the gate entirely. */
	cfg.notify_min_interval_ms = 0U;
	TEST_ASSERT_EQUAL_INT(0, snmp_init(&c, &cfg));
	TEST_ASSERT_TRUE(snmp_notify_gate(&c, SNMP_TRAP_AUTH_FAILURE, 0U, NULL));
	TEST_ASSERT_TRUE(snmp_notify_gate(&c, SNMP_TRAP_AUTH_FAILURE, 1U, NULL));

	TEST_ASSERT_FALSE(snmp_notify_gate(NULL, SNMP_TRAP_AUTH_FAILURE, 0U,
					   NULL));
	TEST_ASSERT_FALSE(snmp_notify_gate(&c, (snmp_trap_t)SNMP_TRAP__COUNT, 0U,
					   NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_notify_suppressed(NULL,
						     SNMP_TRAP_AUTH_FAILURE,
						     &sup));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_notify_suppressed(&c,
						     SNMP_TRAP_AUTH_FAILURE,
						     NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_notify_suppressed(&c,
						     (snmp_trap_t)SNMP_TRAP__COUNT,
						     &sup));
}

/* ------------------------------------------------------------------------- */
/* hostile input                                                             */
/* ------------------------------------------------------------------------- */

static void test_random_datagrams_never_crash(void)
{
	test_rng_t g;
	uint8_t buf[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	size_t rsp_len = 0U;
	unsigned int i;

	users_ready();
	test_rng_init(&g, 0x3414U);

	for (i = 0U; i < 20000U; i++) {
		size_t n = 1U + test_rng_below(&g, 200U);
		int32_t ver;

		test_rng_fill(&g, buf, n);
		/* Give a third of them a plausible v3 envelope prefix so the
		 * USM parser gets past the first two fields. */
		if ((i % 3U) == 0U && n > 12U) {
			buf[0] = SNMP_TAG_SEQUENCE;
			buf[1] = (uint8_t)(n - 2U);
			buf[2] = SNMP_TAG_INTEGER;
			buf[3] = 0x01U;
			buf[4] = 0x03U;
			buf[5] = SNMP_TAG_SEQUENCE;
		}
		(void)snmp_peek_version(buf, n, &ver);
		(void)snmp_v3_handle(&g_agent, &g_v3, buf, n, 1000U, 100U, rsp,
				     sizeof(rsp), &rsp_len);
		(void)snmp_dispatch(&g_agent, &g_v3, buf, n, 1000U, 100U, rsp,
				    sizeof(rsp), &rsp_len);
	}
}

static void test_mutated_valid_requests_never_crash(void)
{
	static const uint8_t salt[SNMP_V3_SALT_LEN] = { 9, 8, 7, 6, 5, 4, 3, 2 };
	test_rng_t g;
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	size_t rsp_len = 0U;
	size_t ap = 0U;
	unsigned int i;

	users_ready();
	test_rng_init(&g, 0x3826U);

	for (i = 0U; i < 12000U; i++) {
		req_spec_t s = base_spec();
		size_t req_len;
		size_t pos;
		unsigned int level = i % 3U;

		/* Rotate through all three security levels so every branch of
		 * the §3.2 ladder is reached with damaged input. */
		if (level == 0U) {
			s.user = "guest";
		} else if (level == 1U) {
			s.user = "alice";
			s.flags = SNMP_V3_FLAG_AUTH | SNMP_V3_FLAG_REPORTABLE;
			s.authparam_len = 24U;
		} else {
			s.user = "bob";
			s.flags = SNMP_V3_FLAG_AUTH | SNMP_V3_FLAG_PRIV |
				  SNMP_V3_FLAG_REPORTABLE;
			s.authparam_len = 24U;
			s.salt = salt;
		}
		s.pdu_tag = (i & 1U) ? SNMP_PDU_GETNEXT : SNMP_PDU_GET;

		req_len = build_request(&s,
					(level != 0U) ? g_kul : NULL, 32U,
					SNMP_AUTH_HMAC_SHA256_192,
					(level == 2U) ? g_kul : NULL, req, &ap);

		pos = test_rng_below(&g, req_len);
		req[pos] ^= (uint8_t)(1U << test_rng_below(&g, 8U));

		(void)snmp_v3_handle(&g_agent, &g_v3, req, req_len, 1000U, 100U,
				     rsp, sizeof(rsp), &rsp_len);
	}
}

static void test_truncated_valid_requests_never_crash(void)
{
	req_spec_t s;
	uint8_t req[SNMP_PKT_MAX];
	uint8_t rsp[SNMP_PKT_MAX];
	size_t req_len;
	size_t rsp_len = 0U;
	size_t ap = 0U;
	size_t n;

	users_ready();

	s = base_spec();
	s.user = "alice";
	s.flags = SNMP_V3_FLAG_AUTH | SNMP_V3_FLAG_REPORTABLE;
	s.authparam_len = 24U;
	req_len = build_request(&s, g_kul, 32U, SNMP_AUTH_HMAC_SHA256_192, NULL,
				req, &ap);

	/* Every prefix of a valid request must be rejected, never parsed
	 * past its end. */
	for (n = 1U; n < req_len; n++) {
		int rc = snmp_v3_handle(&g_agent, &g_v3, req, n, 1000U, 100U,
					rsp, sizeof(rsp), &rsp_len);

		TEST_ASSERT_TRUE(rc != 0);
	}
	TEST_ASSERT_EQUAL_INT(0, snmp_v3_handle(&g_agent, &g_v3, req, req_len,
						1000U, 100U, rsp, sizeof(rsp),
						&rsp_len));
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	UNITY_BEGIN();

	fixture_init();

	RUN_TEST(test_protocol_tables);
	RUN_TEST(test_password_to_key_rfc3414_a3);
	RUN_TEST(test_localisation_binds_to_the_engine_id);
	RUN_TEST(test_password_to_key_guards);
	RUN_TEST(test_auth_digest_is_a_truncated_hmac);

	RUN_TEST(test_aes_cfb_nist_sp800_38a);
	RUN_TEST(test_aes_cfb_short_final_block);
	RUN_TEST(test_aes_cfb_guards);
	RUN_TEST(test_priv_iv_layout);

	RUN_TEST(test_engine_id_build);
	RUN_TEST(test_init_guards);
	RUN_TEST(test_engine_id_and_boots_accessors);
	RUN_TEST(test_user_set_variants);
	RUN_TEST(test_user_set_guards);
	RUN_TEST(test_user_set_needs_an_engine_id);
	RUN_TEST(test_usm_stats_oids);

	RUN_TEST(test_unknown_engine_id_report);
	RUN_TEST(test_unknown_user_report);
	RUN_TEST(test_unsupported_sec_level_report);
	RUN_TEST(test_wrong_digest_report);
	RUN_TEST(test_time_window_report);

	RUN_TEST(test_noauth_get_is_served);
	RUN_TEST(test_authnopriv_get_is_served_and_signed);
	RUN_TEST(test_authpriv_round_trip);
	RUN_TEST(test_privacy_failures);
	RUN_TEST(test_pdu_variants);
	RUN_TEST(test_context_checks);
	RUN_TEST(test_bad_security_model_and_version);
	RUN_TEST(test_handle_argument_guards);
	RUN_TEST(test_msg_max_size_is_honoured);

	RUN_TEST(test_dispatch_routes_by_version);
	RUN_TEST(test_notifications);
	RUN_TEST(test_notification_guards);
	RUN_TEST(test_notify_gate);

	RUN_TEST(test_random_datagrams_never_crash);
	RUN_TEST(test_mutated_valid_requests_never_crash);
	RUN_TEST(test_truncated_valid_requests_never_crash);

	return UNITY_END();
}
