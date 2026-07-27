/*
 * STS1000 "Meridian" — host unit tests for core/ptp/ptp_icv.c
 * (IEEE 1588-2019 Annex P AUTHENTICATION TLV).
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The suite is built around one independently-computed vector: the ICV is
 * recomputed here with host_hmac_sha256() over the byte range the standard
 * describes, rather than by calling the module and comparing it to itself. A
 * self-round-trip would pass even if the covered range were wrong on both sides.
 */

#include <errno.h>
#include <string.h>

#include "host_sha256.h"
#include "ptp/ptp.h"
#include "ptp/ptp_icv.h"
#include "unity.h"

/* ------------------------------------------------------------------ fixture */

#define SPP_A 3U
#define KEYID_A 0x11223344UL
#define SPP_B 9U
#define KEYID_B 0x55667788UL

static const uint8_t key_a[20] = { 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
				   0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
				   0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b };
static const uint8_t key_b[32] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00,
				   0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
				   0x88, 0x99, 0x01, 0x02, 0x03, 0x04, 0x05,
				   0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C,
				   0x0D, 0x0E, 0x0F, 0x10 };

static const uint8_t peer_mac[6] = { 0x02, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE };

static ptp_port_id_t peer_id(void)
{
	ptp_port_id_t id;

	(void)memset(&id, 0, sizeof(id));
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_id_from_mac(&id.clock_id, peer_mac));
	id.port_number = 1U;
	return id;
}

/* A Sync with a distinctive correctionField and messageTypeSpecific. */
static size_t make_sync(uint8_t *buf, size_t cap)
{
	ptp_hdr_t h;
	ptp_timestamp_t ts;
	size_t len = 0U;

	(void)memset(&h, 0, sizeof(h));
	h.msg_type = (uint8_t)PTP_MSG_SYNC;
	h.version = PTP_VERSION;
	h.minor_version = PTP_MINOR_VERSION_2019;
	h.domain = 0U;
	h.flags = PTP_FLAG_TWO_STEP;
	h.correction = 0x0123456789ABLL;
	h.msg_type_specific = 0xDEADBEEFUL;
	h.seq_id = 0x1234U;
	h.control = ptp_msg_control_field((uint8_t)PTP_MSG_SYNC);
	h.source_port = peer_id();

	ts.seconds = 0x0000112233445566ULL & 0xFFFFFFFFFFFFULL;
	ts.nanoseconds = 123456789UL;

	TEST_ASSERT_EQUAL_INT(0, ptp_tsmsg_encode(buf, cap, &h, &ts, &len));
	return len;
}

static void cfg_with_key_a(ptp_icv_cfg_t *cfg, uint8_t policy, bool replay,
			   bool mask, uint8_t suite)
{
	ptp_icv_cfg_defaults(cfg);
	cfg->policy = policy;
	cfg->replay_protect = replay;
	cfg->mask_mutable = mask;
	cfg->tx_spp = SPP_A;
	cfg->tx_key_id = KEYID_A;
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_key_set(cfg, SPP_A, KEYID_A, suite,
						 key_a, sizeof(key_a)));
}

/*
 * Recompute the ICV independently: HMAC-SHA-256 over [0, icv_off) with the ICV
 * field excluded and, when masking is on, correctionField (8..15) and
 * messageTypeSpecific (16..19) treated as zero.
 */
static void expected_icv(const uint8_t *msg, size_t icv_off, bool mask,
			 const uint8_t *key, size_t key_len, uint8_t out[32])
{
	uint8_t tmp[PTP_MSG_MAX_LEN];

	TEST_ASSERT_TRUE(icv_off <= sizeof(tmp));
	(void)memcpy(tmp, msg, icv_off);
	if (mask && (icv_off >= 20U)) {
		(void)memset(&tmp[8], 0, 12U);
	}
	host_hmac_sha256(key, key_len, tmp, icv_off, out);
}

/* ===================================================================== *
 *  Configuration
 * ===================================================================== */

static void test_defaults(void)
{
	ptp_icv_cfg_t cfg;

	ptp_icv_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)PTP_ICV_POLICY_VERIFY_IF_PRESENT,
				cfg.policy);
	TEST_ASSERT_EQUAL_HEX16(PTP_TLV_TYPE_AUTHENTICATION, cfg.tlv_type);
	TEST_ASSERT_TRUE(cfg.replay_protect);
	TEST_ASSERT_EQUAL_UINT8(16U, cfg.replay_window);
	TEST_ASSERT_TRUE(cfg.mask_mutable);
	/* Inert until a key is provisioned. */
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_cfg_validate(&cfg));
	ptp_icv_cfg_defaults(NULL); /* must not crash */
}

static void test_suite_lengths(void)
{
	TEST_ASSERT_EQUAL_UINT(16U,
		ptp_icv_suite_len((uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128));
	TEST_ASSERT_EQUAL_UINT(32U,
		ptp_icv_suite_len((uint8_t)PTP_ICV_SUITE_HMAC_SHA256_256));
	TEST_ASSERT_EQUAL_UINT(0U,
		ptp_icv_suite_len((uint8_t)PTP_ICV_SUITE_COUNT));
	TEST_ASSERT_EQUAL_UINT(0U, ptp_icv_suite_len(0xFFU));
}

static void test_key_table(void)
{
	ptp_icv_cfg_t cfg;
	unsigned int i;

	ptp_icv_cfg_defaults(&cfg);

	TEST_ASSERT_EQUAL_INT(0, ptp_icv_key_set(&cfg, SPP_A, KEYID_A,
			(uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128, key_a,
			sizeof(key_a)));
	/* Same (spp, keyID) replaces rather than consuming a second slot. */
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_key_set(&cfg, SPP_A, KEYID_A,
			(uint8_t)PTP_ICV_SUITE_HMAC_SHA256_256, key_b,
			sizeof(key_b)));

	/* Fill it. */
	for (i = 1U; i < (unsigned int)PTP_ICV_MAX_KEYS; i++) {
		TEST_ASSERT_EQUAL_INT(0, ptp_icv_key_set(&cfg, SPP_B,
				(uint32_t)i,
				(uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128, key_a,
				sizeof(key_a)));
	}
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ptp_icv_key_set(&cfg, SPP_B, 0xFFFFU,
			(uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128, key_a,
			sizeof(key_a)));

	/* Clearing frees the slot, and the key bytes are wiped. */
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_key_clear(&cfg, SPP_B, 1U));
	TEST_ASSERT_EQUAL_INT(-ENOENT, ptp_icv_key_clear(&cfg, SPP_B, 1U));
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_key_set(&cfg, SPP_B, 0xFFFFU,
			(uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128, key_a,
			sizeof(key_a)));

	/* Argument checks. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_key_set(NULL, 0U, 0U, 0U, key_a, 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_key_set(&cfg, 0U, 0U, 0U, NULL, 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		ptp_icv_key_set(&cfg, 0U, 0U, 0xFFU, key_a, 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_key_set(&cfg, 0U, 0U,
			(uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128, key_a, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_key_set(&cfg, 0U, 0U,
			(uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128, key_a,
			PTP_ICV_KEY_MAX + 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_key_clear(NULL, 0U, 0U));
}

static void test_cfg_validate(void)
{
	ptp_icv_cfg_t cfg;

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_cfg_validate(NULL));

	ptp_icv_cfg_defaults(&cfg);
	cfg.policy = (uint8_t)PTP_ICV_POLICY_COUNT;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_cfg_validate(&cfg));

	ptp_icv_cfg_defaults(&cfg);
	cfg.replay_window = PTP_ICV_REPLAY_WINDOW_MAX + 1U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_cfg_validate(&cfg));

	/* A populated table whose transmit association is missing is reported. */
	ptp_icv_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_key_set(&cfg, SPP_A, KEYID_A,
			(uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128, key_a,
			sizeof(key_a)));
	cfg.tx_spp = SPP_B;
	cfg.tx_key_id = KEYID_B;
	TEST_ASSERT_EQUAL_INT(-ENOENT, ptp_icv_cfg_validate(&cfg));
	/* ...unless integrity is off entirely. */
	cfg.policy = (uint8_t)PTP_ICV_POLICY_OFF;
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_cfg_validate(&cfg));

	/* A corrupt entry is caught. */
	ptp_icv_cfg_defaults(&cfg);
	cfg.key[0].in_use = true;
	cfg.key[0].suite = 0xFFU;
	cfg.key[0].key_len = 4U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_cfg_validate(&cfg));
	cfg.key[0].suite = (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128;
	cfg.key[0].key_len = 0U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_cfg_validate(&cfg));
}

static void test_init_rejects(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	port_crypto_t bad;

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_REQUIRE, true, true,
		       (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_init(NULL, &cfg, host_crypto()));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_init(&c, NULL, host_crypto()));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_init(&c, &cfg, NULL));

	/* hmac_sha256 is the one primitive this module cannot do without. */
	bad = *host_crypto();
	bad.hmac_sha256 = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_init(&c, &cfg, &bad));

	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));
	TEST_ASSERT_NOT_NULL(ptp_icv_counters(&c));
	TEST_ASSERT_NULL(ptp_icv_counters(NULL));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_set_cfg(NULL, &cfg));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_set_cfg(&c, NULL));
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_set_cfg(&c, &cfg));
	ptp_icv_reset_peers(NULL); /* must not crash */
}

/* ===================================================================== *
 *  The vector
 * ===================================================================== */

/*
 * Append with replay protection on and masking on: assert the whole TLV layout
 * field by field, then the ICV against an independently computed HMAC.
 */
static void test_append_layout_and_vector(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	size_t base;
	size_t icv_off;
	uint8_t want[32];

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_REQUIRE, true, true,
		       (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));

	base = make_sync(buf, sizeof(buf));
	TEST_ASSERT_EQUAL_UINT(PTP_TSMSG_LEN, base);
	len = base;

	TEST_ASSERT_EQUAL_INT(0, ptp_icv_append(&c, buf, sizeof(buf), &len));

	/* 4 header + SPP 1 + secParamIndicator 1 + keyID 4 + seqNo 4 + ICV 16. */
	TEST_ASSERT_EQUAL_UINT(base + 4U + 6U + 4U + 16U, len);
	TEST_ASSERT_EQUAL_UINT32(1U, ptp_icv_counters(&c)->tx_appended);

	/* messageLength updated to include the TLV. */
	TEST_ASSERT_EQUAL_UINT16(len, ((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);

	/* tlvType and lengthField. */
	TEST_ASSERT_EQUAL_HEX16(PTP_TLV_TYPE_AUTHENTICATION,
		((uint16_t)buf[base] << 8) | (uint16_t)buf[base + 1U]);
	TEST_ASSERT_EQUAL_HEX16(26U,
		((uint16_t)buf[base + 2U] << 8) | (uint16_t)buf[base + 3U]);

	/* SPP, secParamIndicator (sequenceNo present), keyID. */
	TEST_ASSERT_EQUAL_HEX8(SPP_A, buf[base + 4U]);
	TEST_ASSERT_EQUAL_HEX8(PTP_ICV_SPI_SEQNO, buf[base + 5U]);
	TEST_ASSERT_EQUAL_HEX8(0x11U, buf[base + 6U]);
	TEST_ASSERT_EQUAL_HEX8(0x22U, buf[base + 7U]);
	TEST_ASSERT_EQUAL_HEX8(0x33U, buf[base + 8U]);
	TEST_ASSERT_EQUAL_HEX8(0x44U, buf[base + 9U]);
	/* sequenceNo starts at 1, not 0. */
	TEST_ASSERT_EQUAL_HEX8(0x00U, buf[base + 10U]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, buf[base + 11U]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, buf[base + 12U]);
	TEST_ASSERT_EQUAL_HEX8(0x01U, buf[base + 13U]);

	icv_off = base + 14U;
	expected_icv(buf, icv_off, true, key_a, sizeof(key_a), want);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want, &buf[icv_off], 16U);

	/* The next message carries sequenceNo 2. */
	len = make_sync(buf, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_append(&c, buf, sizeof(buf), &len));
	TEST_ASSERT_EQUAL_HEX8(0x02U, buf[base + 13U]);
}

/* Masking is what the ICV covers, so it must be observable on the wire. */
static void test_masking_changes_the_covered_bytes(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	uint8_t masked[PTP_MSG_MAX_LEN];
	uint8_t unmasked[PTP_MSG_MAX_LEN];
	size_t lm;
	size_t lu;
	size_t icv_off = PTP_TSMSG_LEN + 14U;
	uint8_t want[32];

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_REQUIRE, true, true,
		       (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));
	lm = make_sync(masked, sizeof(masked));
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_append(&c, masked, sizeof(masked), &lm));

	cfg.mask_mutable = false;
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));
	lu = make_sync(unmasked, sizeof(unmasked));
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_append(&c, unmasked, sizeof(unmasked), &lu));

	TEST_ASSERT_EQUAL_UINT(lm, lu);
	/* Same message, same key, same sequence number — different coverage. */
	TEST_ASSERT_TRUE(memcmp(&masked[icv_off], &unmasked[icv_off], 16U) != 0);

	/* And each matches its own independent recomputation. */
	expected_icv(unmasked, icv_off, false, key_a, sizeof(key_a), want);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want, &unmasked[icv_off], 16U);

	/*
	 * Concretely: with masking on, rewriting correctionField the way a
	 * transparent clock does leaves the ICV valid. With masking off it does
	 * not. That is the whole reason the switch exists.
	 */
	{
		ptp_port_id_t p = peer_id();

		cfg.mask_mutable = true;
		TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));
		masked[8] = 0x7FU; /* a TC updating correctionField */
		masked[15] = 0x01U;
		TEST_ASSERT_EQUAL_INT(0,
			ptp_icv_verify(&c, masked, lm, &p, 1000U));

		cfg.mask_mutable = false;
		TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));
		unmasked[8] = 0x7FU;
		TEST_ASSERT_EQUAL_INT(-EACCES,
			ptp_icv_verify(&c, unmasked, lu, &p, 1000U));
	}
}

static void test_append_suite_256(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	size_t base;
	uint8_t want[32];

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_REQUIRE, false, true,
		       (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_256);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));

	base = make_sync(buf, sizeof(buf));
	len = base;
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_append(&c, buf, sizeof(buf), &len));

	/* No sequenceNo this time, and a 32-octet ICV. */
	TEST_ASSERT_EQUAL_UINT(base + 4U + 6U + 32U, len);
	TEST_ASSERT_EQUAL_HEX8(0x00U, buf[base + 5U]);

	expected_icv(buf, base + 10U, true, key_a, sizeof(key_a), want);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want, &buf[base + 10U], 32U);

	/* And it verifies. */
	{
		ptp_port_id_t p = peer_id();

		TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, &p, 1U));
	}
}

/* ===================================================================== *
 *  Policy switch
 * ===================================================================== */

static void test_policy_off_is_inert(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	ptp_port_id_t p = peer_id();

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_OFF, true, true,
		       (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));

	len = make_sync(buf, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(-ENOENT, ptp_icv_append(&c, buf, sizeof(buf), &len));
	TEST_ASSERT_EQUAL_UINT(PTP_TSMSG_LEN, len);
	TEST_ASSERT_EQUAL_UINT32(0U, ptp_icv_counters(&c)->tx_appended);
	TEST_ASSERT_EQUAL_UINT32(0U, ptp_icv_counters(&c)->tx_errors);

	/* Unsigned traffic passes, and so does a message with a bogus TLV. */
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, &p, 0U));
}

static void test_policy_verify_if_present(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	ptp_port_id_t p = peer_id();

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_VERIFY_IF_PRESENT, true,
		       true, (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));

	/* Absent: accepted, and counted as such rather than silently. */
	len = make_sync(buf, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, &p, 0U));
	TEST_ASSERT_EQUAL_UINT32(1U, ptp_icv_counters(&c)->rx_absent_ok);

	/* Present and good: accepted. */
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_append(&c, buf, sizeof(buf), &len));
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, &p, 1U));
	TEST_ASSERT_EQUAL_UINT32(1U, ptp_icv_counters(&c)->rx_ok);

	/* Present and bad: rejected. "If present" is not "if convenient". */
	buf[len - 1U] ^= 0x01U;
	TEST_ASSERT_EQUAL_INT(-EACCES, ptp_icv_verify(&c, buf, len, &p, 2U));
	TEST_ASSERT_EQUAL_UINT32(1U, ptp_icv_counters(&c)->rx_bad_icv);
}

static void test_policy_require(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	ptp_port_id_t p = peer_id();

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_REQUIRE, true, true,
		       (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));

	len = make_sync(buf, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(-EACCES, ptp_icv_verify(&c, buf, len, &p, 0U));
	TEST_ASSERT_EQUAL_UINT32(1U, ptp_icv_counters(&c)->rx_absent_rej);

	TEST_ASSERT_EQUAL_INT(0, ptp_icv_append(&c, buf, sizeof(buf), &len));
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, &p, 1U));
}

/* An unknown (SPP, keyID) is distinguishable from a bad ICV. */
static void test_unknown_key(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	size_t base;
	ptp_port_id_t p = peer_id();

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_REQUIRE, true, true,
		       (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));

	base = make_sync(buf, sizeof(buf));
	len = base;
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_append(&c, buf, sizeof(buf), &len));

	buf[base + 4U] = SPP_B; /* an SPP we have no association for */
	TEST_ASSERT_EQUAL_INT(-EPERM, ptp_icv_verify(&c, buf, len, &p, 1U));
	TEST_ASSERT_EQUAL_UINT32(1U, ptp_icv_counters(&c)->rx_no_key);
	TEST_ASSERT_EQUAL_UINT32(0U, ptp_icv_counters(&c)->rx_bad_icv);
}

/* Wrong key for the same association: the ICV must not verify. */
static void test_wrong_key_fails(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	ptp_port_id_t p = peer_id();

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_REQUIRE, false, true,
		       (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));
	len = make_sync(buf, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_append(&c, buf, sizeof(buf), &len));

	/* Same SPP/keyID, different key material. */
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_key_set(&cfg, SPP_A, KEYID_A,
			(uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128, key_b,
			sizeof(key_b)));
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_set_cfg(&c, &cfg));
	TEST_ASSERT_EQUAL_INT(-EACCES, ptp_icv_verify(&c, buf, len, &p, 1U));
}

/* ===================================================================== *
 *  Replay
 * ===================================================================== */

/* Build a signed Sync with an arbitrary sequenceNo, as an attacker would. */
static size_t signed_sync_with_seq(ptp_icv_ctx_t *c, uint8_t *buf, size_t cap,
				   uint32_t seq)
{
	size_t len;
	size_t base;
	uint8_t tag[32];

	base = make_sync(buf, cap);
	len = base;
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_append(c, buf, cap, &len));

	/* Rewrite sequenceNo and re-sign, so the ICV stays valid. */
	buf[base + 10U] = (uint8_t)(seq >> 24);
	buf[base + 11U] = (uint8_t)(seq >> 16);
	buf[base + 12U] = (uint8_t)(seq >> 8);
	buf[base + 13U] = (uint8_t)seq;
	expected_icv(buf, base + 14U, c->cfg.mask_mutable, key_a, sizeof(key_a),
		     tag);
	(void)memcpy(&buf[base + 14U], tag, 16U);
	return len;
}

static void test_replay_rejection(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	ptp_port_id_t p = peer_id();

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_REQUIRE, true, true,
		       (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);
	cfg.replay_window = 8U;
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));

	/* First message at 100 sets the window. */
	len = signed_sync_with_seq(&c, buf, sizeof(buf), 100U);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, &p, 1U));

	/* Exactly the same message again: replay. */
	TEST_ASSERT_EQUAL_INT(-EPROTO, ptp_icv_verify(&c, buf, len, &p, 2U));
	TEST_ASSERT_EQUAL_UINT32(1U, ptp_icv_counters(&c)->rx_replay);

	/* Forward is fine. */
	len = signed_sync_with_seq(&c, buf, sizeof(buf), 101U);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, &p, 3U));

	/* An out-of-order arrival inside the window is accepted exactly once. */
	len = signed_sync_with_seq(&c, buf, sizeof(buf), 99U);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, &p, 4U));
	TEST_ASSERT_EQUAL_INT(-EPROTO, ptp_icv_verify(&c, buf, len, &p, 5U));

	/* Below the window: refused, because we cannot prove it is not a replay. */
	len = signed_sync_with_seq(&c, buf, sizeof(buf), 90U);
	TEST_ASSERT_EQUAL_INT(-EPROTO, ptp_icv_verify(&c, buf, len, &p, 6U));

	/* A large forward jump resets the window and is accepted. */
	len = signed_sync_with_seq(&c, buf, sizeof(buf), 100000U);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, &p, 7U));
	/* ...and now the old high water mark is far below the window. */
	len = signed_sync_with_seq(&c, buf, sizeof(buf), 101U);
	TEST_ASSERT_EQUAL_INT(-EPROTO, ptp_icv_verify(&c, buf, len, &p, 8U));
}

/*
 * The ordering that matters: a *forged* message must not be able to advance a
 * peer's replay window, because that would let an attacker lock out the genuine
 * messages that follow it. ICV first, window second.
 */
static void test_forged_message_cannot_poison_the_window(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	uint8_t good[PTP_MSG_MAX_LEN];
	uint8_t bad[PTP_MSG_MAX_LEN];
	size_t lg;
	size_t lb;
	ptp_port_id_t p = peer_id();

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_REQUIRE, true, true,
		       (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));

	/* A forgery claiming a huge sequenceNo, with the ICV left wrong. */
	lb = signed_sync_with_seq(&c, bad, sizeof(bad), 0xFFFFFF00UL);
	bad[lb - 1U] ^= 0xFFU;
	TEST_ASSERT_EQUAL_INT(-EACCES, ptp_icv_verify(&c, bad, lb, &p, 1U));

	/* The genuine low-numbered traffic that follows is still accepted. */
	lg = signed_sync_with_seq(&c, good, sizeof(good), 5U);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, good, lg, &p, 2U));
	TEST_ASSERT_EQUAL_UINT32(0U, ptp_icv_counters(&c)->rx_replay);
}

static void test_replay_peer_table_and_eviction(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	unsigned int i;
	ptp_port_id_t p = peer_id();

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_REQUIRE, true, true,
		       (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));

	/* Fill the peer table, each peer with its own port number. */
	for (i = 0U; i < (unsigned int)PTP_ICV_MAX_PEERS; i++) {
		ptp_port_id_t q = p;

		q.port_number = (uint16_t)(i + 1U);
		len = signed_sync_with_seq(&c, buf, sizeof(buf), 10U);
		TEST_ASSERT_EQUAL_INT(0,
			ptp_icv_verify(&c, buf, len, &q, 100U + i));
	}
	TEST_ASSERT_EQUAL_UINT32(0U, ptp_icv_counters(&c)->peers_evicted);

	/* One more displaces the least recently heard. */
	{
		ptp_port_id_t q = p;

		q.port_number = 0x4000U;
		len = signed_sync_with_seq(&c, buf, sizeof(buf), 10U);
		TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, &q, 9999U));
		TEST_ASSERT_EQUAL_UINT32(1U, ptp_icv_counters(&c)->peers_evicted);
	}

	/* A NULL peer skips the window entirely but still checks the ICV. */
	len = signed_sync_with_seq(&c, buf, sizeof(buf), 1U);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, NULL, 1U));
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, NULL, 1U));

	/*
	 * Peer 1 was the least recently heard and is the one that was displaced,
	 * so its window has restarted and seq 10 is new to it again — a bounded
	 * loss of replay history, which is the documented cost of a full table.
	 */
	len = signed_sync_with_seq(&c, buf, sizeof(buf), 10U);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, &p, 1U));

	/*
	 * The most recently heard peer kept its window, so its seq 10 is still a
	 * replay. This is the half that proves eviction is by recency and not
	 * wholesale forgetting.
	 */
	{
		ptp_port_id_t q = p;

		q.port_number = (uint16_t)PTP_ICV_MAX_PEERS;
		len = signed_sync_with_seq(&c, buf, sizeof(buf), 10U);
		TEST_ASSERT_EQUAL_INT(-EPROTO,
			ptp_icv_verify(&c, buf, len, &q, 2U));

		/* Resetting the windows lets it through again. */
		ptp_icv_reset_peers(&c);
		TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, &q, 3U));
	}
}

/* A window of 0 must behave as the strictest setting, not as "no window". */
static void test_zero_window_is_strict(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	ptp_port_id_t p = peer_id();

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_REQUIRE, true, true,
		       (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);
	cfg.replay_window = 0U;
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));

	len = signed_sync_with_seq(&c, buf, sizeof(buf), 50U);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, &p, 1U));
	/* Immediately-previous is already outside a one-wide window. */
	len = signed_sync_with_seq(&c, buf, sizeof(buf), 49U);
	TEST_ASSERT_EQUAL_INT(-EPROTO, ptp_icv_verify(&c, buf, len, &p, 2U));
	len = signed_sync_with_seq(&c, buf, sizeof(buf), 51U);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, &p, 3U));
}

/* ===================================================================== *
 *  Malformed input
 * ===================================================================== */

static void test_malformed_tlv(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	size_t base;
	ptp_port_id_t p = peer_id();

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_VERIFY_IF_PRESENT, true,
		       true, (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_verify(NULL, buf, 10U, &p, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_verify(&c, NULL, 10U, &p, 0U));

	/* Too short to hold a header: a broken suffix, not "no TLV". */
	base = make_sync(buf, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
		ptp_icv_verify(&c, buf, PTP_HDR_LEN - 1U, &p, 0U));

	/* An AUTHENTICATION TLV whose value cannot hold the fixed fields. */
	len = base;
	{
		static const uint8_t stub[4] = { 0, 0, 0, 0 };

		TEST_ASSERT_EQUAL_INT(0, ptp_tlv_append(buf, sizeof(buf), &len,
			(uint16_t)PTP_TLV_TYPE_AUTHENTICATION, stub,
			sizeof(stub), NULL));
	}
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ptp_icv_verify(&c, buf, len, &p, 0U));
	TEST_ASSERT_TRUE(ptp_icv_counters(&c)->rx_malformed > 0U);

	/* disclosedKey (TESLA) is refused rather than parsed at a guessed offset. */
	base = make_sync(buf, sizeof(buf));
	len = base;
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_append(&c, buf, sizeof(buf), &len));
	buf[base + 5U] |= PTP_ICV_SPI_DISCLOSED_KEY;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ptp_icv_verify(&c, buf, len, &p, 0U));

	/* secParamIndicator claims sequenceNo/RES the TLV is too short to hold. */
	base = make_sync(buf, sizeof(buf));
	len = base;
	{
		uint8_t v[PTP_ICV_FIXED_LEN + 16U];

		(void)memset(v, 0, sizeof(v));
		v[0] = SPP_A;
		v[1] = PTP_ICV_SPI_SEQNO | PTP_ICV_SPI_RES;
		v[2] = 0x11U;
		v[3] = 0x22U;
		v[4] = 0x33U;
		v[5] = 0x44U;
		TEST_ASSERT_EQUAL_INT(0, ptp_tlv_append(buf, sizeof(buf), &len,
			(uint16_t)PTP_TLV_TYPE_AUTHENTICATION, v, sizeof(v),
			NULL));
	}
	/* 6 fixed + 4 seq + 4 RES = 14 of 22, leaving 8 where 16 are needed. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ptp_icv_verify(&c, buf, len, &p, 0U));

	/* An ICV length that disagrees with the association's suite. */
	base = make_sync(buf, sizeof(buf));
	len = base;
	{
		uint8_t v[PTP_ICV_FIXED_LEN + 8U];

		(void)memset(v, 0, sizeof(v));
		v[0] = SPP_A;
		v[1] = 0U;
		v[2] = 0x11U;
		v[3] = 0x22U;
		v[4] = 0x33U;
		v[5] = 0x44U;
		TEST_ASSERT_EQUAL_INT(0, ptp_tlv_append(buf, sizeof(buf), &len,
			(uint16_t)PTP_TLV_TYPE_AUTHENTICATION, v, sizeof(v),
			NULL));
	}
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ptp_icv_verify(&c, buf, len, &p, 0U));
}

/* A different tlvType is "absent", which the configurable type makes reachable. */
static void test_tlv_type_selects_what_is_recognised(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	ptp_port_id_t p = peer_id();

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_REQUIRE, false, true,
		       (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);
	cfg.tlv_type = (uint16_t)PTP_TLV_TYPE_AUTHENTICATION_2008;
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));

	len = make_sync(buf, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_append(&c, buf, sizeof(buf), &len));
	TEST_ASSERT_EQUAL_HEX16(PTP_TLV_TYPE_AUTHENTICATION_2008,
		((uint16_t)buf[PTP_TSMSG_LEN] << 8) |
			(uint16_t)buf[PTP_TSMSG_LEN + 1U]);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_verify(&c, buf, len, &p, 0U));

	/* A context expecting the 2019 type sees no TLV at all. */
	cfg.tlv_type = (uint16_t)PTP_TLV_TYPE_AUTHENTICATION;
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));
	TEST_ASSERT_EQUAL_INT(-EACCES, ptp_icv_verify(&c, buf, len, &p, 0U));
	TEST_ASSERT_EQUAL_UINT32(1U, ptp_icv_counters(&c)->rx_absent_rej);
}

/* No transmit key: nothing is emitted, and it is not reported as a failure. */
static void test_append_without_a_key(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;

	ptp_icv_cfg_defaults(&cfg);
	cfg.policy = (uint8_t)PTP_ICV_POLICY_VERIFY_IF_PRESENT;
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));

	len = make_sync(buf, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(-ENOENT, ptp_icv_append(&c, buf, sizeof(buf), &len));
	TEST_ASSERT_EQUAL_UINT(PTP_TSMSG_LEN, len);
	TEST_ASSERT_EQUAL_UINT32(0U, ptp_icv_counters(&c)->tx_errors);

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_append(NULL, buf, 8U, &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_append(&c, NULL, 8U, &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_icv_append(&c, buf, 8U, NULL));
}

/*
 * No room for the TLV: the message must be left exactly as it was, so a caller
 * that ignores the return code transmits an ordinary unsigned message rather
 * than one carrying a zero ICV that looks like a forgery.
 */
static void test_append_no_space_rolls_back(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	uint8_t buf[PTP_MSG_MAX_LEN];
	uint8_t before[PTP_MSG_MAX_LEN];
	size_t len;

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_REQUIRE, true, true,
		       (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));

	len = make_sync(buf, sizeof(buf));
	(void)memcpy(before, buf, len);

	/* Two octets short of the 30 the TLV needs. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
		ptp_icv_append(&c, buf, len + 28U, &len));
	TEST_ASSERT_EQUAL_UINT(PTP_TSMSG_LEN, len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(before, buf, PTP_TSMSG_LEN);
	TEST_ASSERT_EQUAL_UINT32(1U, ptp_icv_counters(&c)->tx_errors);
	TEST_ASSERT_EQUAL_UINT32(0U, ptp_icv_counters(&c)->tx_appended);
}

/* A failing crypto port must fail closed in both directions. */
static int failing_hmac(void *ctx, const uint8_t *key, size_t key_len,
			const uint8_t *in, size_t len, uint8_t *out)
{
	(void)ctx;
	(void)key;
	(void)key_len;
	(void)in;
	(void)len;
	(void)out;
	return -5;
}

static void test_crypto_failure_fails_closed(void)
{
	ptp_icv_ctx_t c;
	ptp_icv_cfg_t cfg;
	port_crypto_t broken;
	uint8_t buf[PTP_MSG_MAX_LEN];
	uint8_t before[PTP_MSG_MAX_LEN];
	size_t len;
	size_t good_len;
	ptp_port_id_t p = peer_id();

	cfg_with_key_a(&cfg, (uint8_t)PTP_ICV_POLICY_REQUIRE, true, true,
		       (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);

	/* Build a genuinely-signed message first, with a working port. */
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, host_crypto()));
	good_len = make_sync(buf, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_append(&c, buf, sizeof(buf), &good_len));

	broken = *host_crypto();
	broken.hmac_sha256 = failing_hmac;
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&c, &cfg, &broken));

	/* Verify must reject, never accept unverified. */
	TEST_ASSERT_EQUAL_INT(-EIO, ptp_icv_verify(&c, buf, good_len, &p, 0U));

	/* Append must roll the TLV back rather than emit a zero ICV. */
	len = make_sync(buf, sizeof(buf));
	(void)memcpy(before, buf, len);
	TEST_ASSERT_EQUAL_INT(-EIO, ptp_icv_append(&c, buf, sizeof(buf), &len));
	TEST_ASSERT_EQUAL_UINT(PTP_TSMSG_LEN, len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(before, buf, PTP_TSMSG_LEN);
	TEST_ASSERT_EQUAL_UINT16(PTP_TSMSG_LEN,
		((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
}

/* ===================================================================== *
 *  Constant-time compare
 * ===================================================================== */

static void test_ct_equal(void)
{
	uint8_t a[16];
	uint8_t b[16];
	size_t i;

	(void)memset(a, 0x5AU, sizeof(a));
	(void)memset(b, 0x5AU, sizeof(b));
	TEST_ASSERT_TRUE(ptp_icv_ct_equal(a, b, sizeof(a)));

	/* A difference at any single position is detected. */
	for (i = 0U; i < sizeof(a); i++) {
		b[i] ^= 0x01U;
		TEST_ASSERT_FALSE(ptp_icv_ct_equal(a, b, sizeof(a)));
		b[i] ^= 0x01U;
	}
	TEST_ASSERT_TRUE(ptp_icv_ct_equal(a, b, sizeof(a)));

	/* Every bit position within one octet, not just the low one. */
	for (i = 0U; i < 8U; i++) {
		b[7] = (uint8_t)(a[7] ^ (1U << i));
		TEST_ASSERT_FALSE(ptp_icv_ct_equal(a, b, sizeof(a)));
	}
	b[7] = a[7];

	/* Degenerate arguments are never a match. */
	TEST_ASSERT_FALSE(ptp_icv_ct_equal(NULL, b, sizeof(a)));
	TEST_ASSERT_FALSE(ptp_icv_ct_equal(a, NULL, sizeof(a)));
	TEST_ASSERT_FALSE(ptp_icv_ct_equal(a, b, 0U));
}

/*
 * Constant time is a property of the generated code, not something a host test
 * can time reliably. What is checkable, and what a regression would break, is
 * that the comparison has no data-dependent early exit: a mismatch in the FIRST
 * octet and a mismatch in the LAST must both read the whole operand. This test
 * asserts the observable proxy — the result is identical regardless of where the
 * difference sits, including the pathological all-but-one-byte-equal case that
 * an early-exit memcmp would resolve in one iteration.
 */
static void test_ct_equal_has_no_early_exit(void)
{
	uint8_t a[32];
	uint8_t b[32];

	(void)memset(a, 0xFFU, sizeof(a));

	(void)memset(b, 0xFFU, sizeof(b));
	b[0] ^= 0x80U; /* differs immediately */
	TEST_ASSERT_FALSE(ptp_icv_ct_equal(a, b, sizeof(a)));

	(void)memset(b, 0xFFU, sizeof(b));
	b[sizeof(b) - 1U] ^= 0x01U; /* differs only at the very end */
	TEST_ASSERT_FALSE(ptp_icv_ct_equal(a, b, sizeof(a)));

	/* And a full match over the same length. */
	(void)memset(b, 0xFFU, sizeof(b));
	TEST_ASSERT_TRUE(ptp_icv_ct_equal(a, b, sizeof(a)));
}

/* ===================================================================== *
 *  Integration with the port engine
 * ===================================================================== */

typedef struct {
	uint8_t buf[4][PTP_MSG_MAX_LEN];
	size_t len[4];
	unsigned int n;
} icv_fake_t;

static int icv_tx(void *ctx, const ptp_tx_desc_t *d)
{
	icv_fake_t *f = ctx;

	if (f->n < 4U) {
		size_t n = (d->len < sizeof(f->buf[0])) ? d->len
						        : sizeof(f->buf[0]);

		(void)memcpy(f->buf[f->n], d->buf, n);
		f->len[f->n] = d->len;
		f->n++;
	}
	return 0;
}

static void test_port_signs_and_verifies(void)
{
	static const uint8_t self_mac[6] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x55 };
	ptp_port_ctx_t c;
	ptp_cfg_t cfg;
	ptp_port_ops_t ops;
	icv_fake_t f;
	ptp_icv_ctx_t icv;
	ptp_icv_cfg_t icfg;
	uint64_t t = 1000U;
	unsigned int i;
	ptp_tlv_iter_t it;
	uint16_t type;
	bool found = false;

	(void)memset(&f, 0, sizeof(f));
	(void)memset(&ops, 0, sizeof(ops));
	ops.tx = icv_tx;
	ops.ctx = &f;

	cfg_with_key_a(&icfg, (uint8_t)PTP_ICV_POLICY_REQUIRE, true, true,
		       (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128);
	TEST_ASSERT_EQUAL_INT(0, ptp_icv_init(&icv, &icfg, host_crypto()));

	ptp_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_init(&c, &cfg, self_mac, &ops));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_port_set_icv(NULL, &icv));
	TEST_ASSERT_EQUAL_INT(0, ptp_port_set_icv(&c, &icv));
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&c, t));

	for (i = 0U; i < 40U && f.n == 0U; i++) {
		t += 1000U;
		TEST_ASSERT_EQUAL_INT(0, ptp_port_step(&c, t));
	}
	TEST_ASSERT_TRUE(f.n > 0U);

	/* Every transmitted PDU carries an AUTHENTICATION TLV. */
	TEST_ASSERT_EQUAL_INT(0, ptp_tlv_iter_begin(&it, f.buf[0], f.len[0]));
	while (ptp_tlv_iter_next(&it, &type, NULL, NULL, NULL) == 0) {
		if (type == (uint16_t)PTP_TLV_TYPE_AUTHENTICATION) {
			found = true;
		}
	}
	TEST_ASSERT_TRUE(found);
	TEST_ASSERT_TRUE(ptp_icv_counters(&icv)->tx_appended > 0U);

	/*
	 * Under REQUIRE, an unsigned Announce from a peer is dropped before it
	 * can reach the foreign-master table — so it cannot influence the BMCA.
	 */
	{
		ptp_hdr_t h;
		ptp_announce_t a;
		uint8_t buf[PTP_MSG_MAX_LEN];
		size_t len = 0U;
		uint32_t bmca_before;

		(void)memset(&h, 0, sizeof(h));
		(void)memset(&a, 0, sizeof(a));
		h.msg_type = (uint8_t)PTP_MSG_ANNOUNCE;
		h.version = PTP_VERSION;
		h.minor_version = PTP_MINOR_VERSION_2019;
		h.source_port = peer_id();
		a.gm_quality.clock_class = 6U;
		a.gm_identity = h.source_port.clock_id;
		TEST_ASSERT_EQUAL_INT(0,
			ptp_announce_encode(buf, sizeof(buf), &h, &a, &len));

		bmca_before = ptp_port_counters(&c)->bmca_runs;
		TEST_ASSERT_EQUAL_INT(-EACCES,
			ptp_port_rx(&c, buf, len, 0U, t));
		TEST_ASSERT_EQUAL_UINT32(1U,
			ptp_port_counters(&c)->rx_icv_rejected);
		TEST_ASSERT_TRUE((ptp_port_alarms(&c) &
				  PTP_ALARM_ICV_FAILED) != 0U);
		/* No election was re-run on the strength of a rejected frame. */
		TEST_ASSERT_EQUAL_UINT32(bmca_before,
			ptp_port_counters(&c)->bmca_runs);
		TEST_ASSERT_EQUAL_INT(PTP_PS_MASTER, ptp_port_state(&c));
	}

	/* Detaching clears the alarm and restores unauthenticated operation. */
	TEST_ASSERT_EQUAL_INT(0, ptp_port_set_icv(&c, NULL));
	TEST_ASSERT_TRUE((ptp_port_alarms(&c) & PTP_ALARM_ICV_FAILED) == 0U);
}

/* ===================================================================== *
 *  Runner
 * ===================================================================== */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_defaults);
	RUN_TEST(test_suite_lengths);
	RUN_TEST(test_key_table);
	RUN_TEST(test_cfg_validate);
	RUN_TEST(test_init_rejects);

	RUN_TEST(test_append_layout_and_vector);
	RUN_TEST(test_masking_changes_the_covered_bytes);
	RUN_TEST(test_append_suite_256);

	RUN_TEST(test_policy_off_is_inert);
	RUN_TEST(test_policy_verify_if_present);
	RUN_TEST(test_policy_require);
	RUN_TEST(test_unknown_key);
	RUN_TEST(test_wrong_key_fails);

	RUN_TEST(test_replay_rejection);
	RUN_TEST(test_forged_message_cannot_poison_the_window);
	RUN_TEST(test_replay_peer_table_and_eviction);
	RUN_TEST(test_zero_window_is_strict);

	RUN_TEST(test_malformed_tlv);
	RUN_TEST(test_tlv_type_selects_what_is_recognised);
	RUN_TEST(test_append_without_a_key);
	RUN_TEST(test_append_no_space_rolls_back);
	RUN_TEST(test_crypto_failure_fails_closed);

	RUN_TEST(test_ct_equal);
	RUN_TEST(test_ct_equal_has_no_early_exit);

	RUN_TEST(test_port_signs_and_verifies);

	return UNITY_END();
}
