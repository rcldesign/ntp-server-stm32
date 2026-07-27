/*
 * STS1000 "Meridian" — core/ptp message codec unit tests.
 *
 * Provenance of the expectations (ARCHITECTURE.md §9 asks for known-good byte
 * vectors, not self-round-trips):
 *
 *  - Every vector below is a hex dump laid out by hand from the field tables of
 *    IEEE 1588-2019 §13 — Table 35 for the common header, Table 43 for
 *    Announce, Table 46 for Delay_Resp, Table 40 for Sync. The decode tests
 *    address the dump through independently written octet offsets, so a codec
 *    that agreed with itself but disagreed with the standard would fail here.
 *
 *  - The encode tests build the same message from a struct and compare against
 *    the identical dump, which pins the encoder to the standard rather than to
 *    the decoder.
 *
 *  - controlField values are Table 39, the legacy v1 codes. Message lengths are
 *    the sums of the field widths in those tables: 34 header, +10 Timestamp,
 *    +10 PortIdentity, and 30 for the Announce body.
 *
 *  - 0x3B9AC9FF is 999 999 999, the largest legal nanoseconds field; the 48-bit
 *    all-ones seconds field is the wire maximum.
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include "unity.h"

#include "ptp/ptp_msg.h"
#include "test_support.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* ------------------------------------------------------------- vectors --- */

/*
 * Announce, 64 octets. Grandmaster identity deliberately differs from the
 * sourcePortIdentity so a codec that read one for the other would fail.
 */
static const uint8_t announce_vec[64] = {
	/* 0x00 */ 0x0B,                          /* majorSdoId 0, messageType Announce */
	/* 0x01 */ 0x12,                          /* minorVersionPTP 1, versionPTP 2 */
	/* 0x02 */ 0x00, 0x40,                    /* messageLength 64 */
	/* 0x04 */ 0x00,                          /* domainNumber 0 */
	/* 0x05 */ 0x00,                          /* minorSdoId */
	/* 0x06 */ 0x00, 0x1C,                    /* utcOffsetValid|ptpTimescale|timeTraceable */
	/* 0x08 */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x23, 0x45, /* correctionField */
	/* 0x10 */ 0xDE, 0xAD, 0xBE, 0xEF,        /* messageTypeSpecific */
	/* 0x14 */ 0x00, 0x80, 0xE1, 0xFF, 0xFE, 0x11, 0x22, 0x33, /* source clockIdentity */
	/* 0x1C */ 0x00, 0x02,                    /* source portNumber 2 */
	/* 0x1E */ 0x04, 0xD2,                    /* sequenceId 1234 */
	/* 0x20 */ 0x05,                          /* controlField 5 (all others) */
	/* 0x21 */ 0x01,                          /* logMessageInterval 1 */
	/* 0x22 */ 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, /* originTimestamp seconds */
	/* 0x28 */ 0x06, 0x07, 0x08, 0x09,        /* originTimestamp nanoseconds */
	/* 0x2C */ 0x00, 0x25,                    /* currentUtcOffset 37 */
	/* 0x2E */ 0x00,                          /* reserved */
	/* 0x2F */ 0x80,                          /* grandmasterPriority1 128 */
	/* 0x30 */ 0x06,                          /* clockClass 6 */
	/* 0x31 */ 0x20,                          /* clockAccuracy 25 ns */
	/* 0x32 */ 0x4E, 0x5D,                    /* offsetScaledLogVariance */
	/* 0x34 */ 0x80,                          /* grandmasterPriority2 128 */
	/* 0x35 */ 0xAA, 0xBB, 0xCC, 0xFF, 0xFE, 0xDD, 0xEE, 0xFF, /* grandmasterIdentity */
	/* 0x3D */ 0x00, 0x03,                    /* stepsRemoved 3 */
	/* 0x3F */ 0x20,                          /* timeSource GNSS */
};

/*
 * Delay_Resp, 54 octets. Carries a correctionField of -1 and the largest legal
 * Timestamp, so it doubles as the sign and 48-bit-edge vector.
 */
static const uint8_t delay_resp_vec[54] = {
	/* 0x00 */ 0x09,                          /* messageType Delay_Resp */
	/* 0x01 */ 0x12,
	/* 0x02 */ 0x00, 0x36,                    /* messageLength 54 */
	/* 0x04 */ 0x2A,                          /* domainNumber 42 */
	/* 0x05 */ 0x07,                          /* minorSdoId 7 */
	/* 0x06 */ 0x00, 0x00,
	/* 0x08 */ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, /* correction -1 */
	/* 0x10 */ 0x00, 0x00, 0x00, 0x00,
	/* 0x14 */ 0x00, 0x80, 0xE1, 0xFF, 0xFE, 0x11, 0x22, 0x33,
	/* 0x1C */ 0x00, 0x01,
	/* 0x1E */ 0x00, 0x07,                    /* sequenceId 7 */
	/* 0x20 */ 0x03,                          /* controlField 3 */
	/* 0x21 */ 0x00,                          /* logMinDelayReqInterval 0 */
	/* 0x22 */ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, /* receiveTimestamp seconds 2^48-1 */
	/* 0x28 */ 0x3B, 0x9A, 0xC9, 0xFF,        /* nanoseconds 999999999 */
	/* 0x2C */ 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, /* requesting clockId */
	/* 0x34 */ 0x00, 0x63,                    /* requesting portNumber 99 */
};

/* Sync, 44 octets. minorVersionPTP 0 marks a 1588-2008 peer, which is accepted. */
static const uint8_t sync_vec[44] = {
	/* 0x00 */ 0xE0,                          /* majorSdoId 0xE, messageType Sync */
	/* 0x01 */ 0x02,                          /* minorVersionPTP 0, versionPTP 2 */
	/* 0x02 */ 0x00, 0x2C,                    /* messageLength 44 */
	/* 0x04 */ 0x00,
	/* 0x05 */ 0x00,
	/* 0x06 */ 0x02, 0x00,                    /* twoStepFlag */
	/* 0x08 */ 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, /* correction INT64_MAX */
	/* 0x10 */ 0x00, 0x00, 0x00, 0x00,
	/* 0x14 */ 0x00, 0x80, 0xE1, 0xFF, 0xFE, 0x11, 0x22, 0x33,
	/* 0x1C */ 0x00, 0x01,
	/* 0x1E */ 0x12, 0x34,                    /* sequenceId */
	/* 0x20 */ 0x00,                          /* controlField 0 */
	/* 0x21 */ 0xFD,                          /* logMessageInterval -3 */
	/* 0x22 */ 0x00, 0x00, 0x68, 0x5D, 0x3B, 0x00, /* originTimestamp seconds */
	/* 0x28 */ 0x1D, 0xCD, 0x65, 0x00,        /* nanoseconds 500000000 */
};

/* ---------------------------------------------------------- header decode -- */

static void test_hdr_decode_announce_every_field(void)
{
	ptp_hdr_t h;

	TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(announce_vec, sizeof(announce_vec), &h));

	TEST_ASSERT_EQUAL_HEX8(0x00U, h.major_sdo_id);
	TEST_ASSERT_EQUAL_HEX8(0x0BU, h.msg_type);
	TEST_ASSERT_EQUAL_HEX8(0x01U, h.minor_version);
	TEST_ASSERT_EQUAL_HEX8(0x02U, h.version);
	TEST_ASSERT_EQUAL_HEX16(64U, h.msg_length);
	TEST_ASSERT_EQUAL_HEX8(0x00U, h.domain);
	TEST_ASSERT_EQUAL_HEX8(0x00U, h.minor_sdo_id);
	TEST_ASSERT_EQUAL_HEX16(0x001CU, h.flags);
	TEST_ASSERT_EQUAL_INT64(0x12345, h.correction);
	TEST_ASSERT_EQUAL_HEX32(0xDEADBEEFU, h.msg_type_specific);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(&announce_vec[0x14], h.source_port.clock_id.id, 8);
	TEST_ASSERT_EQUAL_HEX16(2U, h.source_port.port_number);
	TEST_ASSERT_EQUAL_HEX16(1234U, h.seq_id);
	TEST_ASSERT_EQUAL_HEX8(5U, h.control);
	TEST_ASSERT_EQUAL_INT8(1, h.log_msg_interval);

	/* The individual timeProperties bits, decoded independently of the u16. */
	TEST_ASSERT_TRUE((h.flags & PTP_FLAG_UTC_OFFSET_VALID) != 0U);
	TEST_ASSERT_TRUE((h.flags & PTP_FLAG_PTP_TIMESCALE) != 0U);
	TEST_ASSERT_TRUE((h.flags & PTP_FLAG_TIME_TRACEABLE) != 0U);
	TEST_ASSERT_TRUE((h.flags & PTP_FLAG_LEAP61) == 0U);
	TEST_ASSERT_TRUE((h.flags & PTP_FLAG_LEAP59) == 0U);
	TEST_ASSERT_TRUE((h.flags & PTP_FLAG_TWO_STEP) == 0U);
}

static void test_hdr_decode_sync_sdo_and_flags(void)
{
	ptp_hdr_t h;

	TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(sync_vec, sizeof(sync_vec), &h));

	/* The high nibble of octet 0 is majorSdoId, the low nibble messageType. */
	TEST_ASSERT_EQUAL_HEX8(0x0EU, h.major_sdo_id);
	TEST_ASSERT_EQUAL_HEX8(0x00U, h.msg_type);
	/* minorVersionPTP 0 identifies a 1588-2008 peer and is still versionPTP 2. */
	TEST_ASSERT_EQUAL_HEX8(0x00U, h.minor_version);
	TEST_ASSERT_EQUAL_HEX8(0x02U, h.version);
	TEST_ASSERT_TRUE((h.flags & PTP_FLAG_TWO_STEP) != 0U);
	TEST_ASSERT_TRUE((h.flags & PTP_FLAG_UNICAST) == 0U);
	TEST_ASSERT_EQUAL_INT8(-3, h.log_msg_interval);
	TEST_ASSERT_EQUAL_HEX8(0U, h.control);
}

static void test_hdr_correction_field_sign(void)
{
	ptp_hdr_t h;
	uint8_t buf[PTP_TSMSG_LEN];
	uint8_t out[PTP_TSMSG_LEN];
	ptp_timestamp_t ts = { 0U, 0U };
	static const int64_t values[] = {
		0, 1, -1, 65536, -65536, INT64_MAX, INT64_MIN, -0x0102030405060708LL,
	};
	size_t i;

	/* The two extremes are pinned to their wire images by the vectors. */
	TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(delay_resp_vec, sizeof(delay_resp_vec), &h));
	TEST_ASSERT_EQUAL_INT64(-1, h.correction);

	TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(sync_vec, sizeof(sync_vec), &h));
	TEST_ASSERT_EQUAL_INT64(INT64_MAX, h.correction);

	/* Two's-complement round trip across the range, including INT64_MIN. */
	memcpy(buf, sync_vec, sizeof(buf));
	for (i = 0U; i < ARRAY_LEN(values); i++) {
		ptp_hdr_t enc;

		TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(buf, sizeof(buf), &enc));
		enc.correction = values[i];
		TEST_ASSERT_EQUAL_INT(0,
			ptp_tsmsg_encode(out, sizeof(out), &enc, &ts, NULL));
		TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(out, sizeof(out), &h));
		TEST_ASSERT_EQUAL_INT64(values[i], h.correction);
	}

	/* -1 must be all-ones on the wire; INT64_MIN must be 0x80 followed by zeros. */
	{
		ptp_hdr_t enc;
		static const uint8_t all_ones[8] = {
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
		};
		static const uint8_t int64_min[8] = {
			0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
		};

		TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(buf, sizeof(buf), &enc));
		enc.correction = -1;
		TEST_ASSERT_EQUAL_INT(0, ptp_tsmsg_encode(out, sizeof(out), &enc, &ts, NULL));
		TEST_ASSERT_EQUAL_HEX8_ARRAY(all_ones, &out[8], 8);

		enc.correction = INT64_MIN;
		TEST_ASSERT_EQUAL_INT(0, ptp_tsmsg_encode(out, sizeof(out), &enc, &ts, NULL));
		TEST_ASSERT_EQUAL_HEX8_ARRAY(int64_min, &out[8], 8);
	}
}

static void test_hdr_decode_rejects_bad_input(void)
{
	uint8_t buf[PTP_ANNOUNCE_LEN];
	ptp_hdr_t h;

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_hdr_decode(NULL, sizeof(announce_vec), &h));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_hdr_decode(announce_vec, sizeof(announce_vec), NULL));

	/* Shorter than the common header. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ptp_hdr_decode(announce_vec, PTP_HDR_LEN - 1U, &h));

	memcpy(buf, announce_vec, sizeof(buf));

	/* versionPTP 1 and 3 are not this protocol. */
	buf[1] = 0x11U;
	TEST_ASSERT_EQUAL_INT(-EPROTO, ptp_hdr_decode(buf, sizeof(buf), &h));
	buf[1] = 0x13U;
	TEST_ASSERT_EQUAL_INT(-EPROTO, ptp_hdr_decode(buf, sizeof(buf), &h));
	buf[1] = 0x12U;

	/* messageLength below the minimum for an Announce. */
	buf[2] = 0x00U;
	buf[3] = 0x3FU;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ptp_hdr_decode(buf, sizeof(buf), &h));

	/* messageLength claiming more than the caller holds. */
	buf[3] = 0x41U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ptp_hdr_decode(buf, sizeof(buf), &h));
	buf[3] = 0x40U;

	/* A whole Announce truncated to a Sync's length: header decode catches it. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ptp_hdr_decode(buf, PTP_TSMSG_LEN, &h));
}

static void test_hdr_decode_tolerates_trailing_octets(void)
{
	/* 802.3 pads a 44-octet Sync out to the 60-octet minimum frame. */
	uint8_t padded[60];
	ptp_hdr_t h;

	memset(padded, 0xA5, sizeof(padded));
	memcpy(padded, sync_vec, sizeof(sync_vec));

	TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(padded, sizeof(padded), &h));
	TEST_ASSERT_EQUAL_HEX16(44U, h.msg_length);
}

/* ---------------------------------------------------------- header encode -- */

static void test_hdr_encode_matches_the_vector(void)
{
	ptp_hdr_t h;
	uint8_t out[PTP_HDR_LEN];

	TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(announce_vec, sizeof(announce_vec), &h));

	memset(out, 0x5A, sizeof(out));
	TEST_ASSERT_EQUAL_INT(0, ptp_hdr_encode(out, sizeof(out), &h));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(announce_vec, out, PTP_HDR_LEN);
}

static void test_hdr_encode_packs_the_nibbles(void)
{
	ptp_hdr_t h;
	uint8_t out[PTP_HDR_LEN];

	memset(&h, 0, sizeof(h));
	h.major_sdo_id = 0x0EU;
	h.msg_type = (uint8_t)PTP_MSG_SIGNALING;   /* 0xC */
	h.minor_version = 0x01U;
	h.version = PTP_VERSION;
	h.msg_length = PTP_HDR_LEN;

	TEST_ASSERT_EQUAL_INT(0, ptp_hdr_encode(out, sizeof(out), &h));
	TEST_ASSERT_EQUAL_HEX8(0xECU, out[0]);
	TEST_ASSERT_EQUAL_HEX8(0x12U, out[1]);
}

static void test_hdr_encode_rejects_bad_input(void)
{
	ptp_hdr_t h;
	uint8_t out[PTP_HDR_LEN];

	memset(&h, 0, sizeof(h));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_hdr_encode(NULL, sizeof(out), &h));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_hdr_encode(out, sizeof(out), NULL));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ptp_hdr_encode(out, PTP_HDR_LEN - 1U, &h));
}

/* ---------------------------------------------------------------- announce -- */

static void test_announce_decode_every_field(void)
{
	ptp_announce_t a;

	TEST_ASSERT_EQUAL_INT(0, ptp_announce_decode(announce_vec, sizeof(announce_vec), &a));

	TEST_ASSERT_EQUAL_HEX64(0x000102030405ULL, a.origin_ts.seconds);
	TEST_ASSERT_EQUAL_HEX32(0x06070809U, a.origin_ts.nanoseconds);
	TEST_ASSERT_EQUAL_INT16(37, a.current_utc_offset);
	TEST_ASSERT_EQUAL_HEX8(128U, a.gm_priority1);
	TEST_ASSERT_EQUAL_HEX8(6U, a.gm_quality.clock_class);
	TEST_ASSERT_EQUAL_HEX8(0x20U, a.gm_quality.clock_accuracy);
	TEST_ASSERT_EQUAL_HEX16(0x4E5DU, a.gm_quality.offset_scaled_log_variance);
	TEST_ASSERT_EQUAL_HEX8(128U, a.gm_priority2);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(&announce_vec[0x35], a.gm_identity.id, 8);
	TEST_ASSERT_EQUAL_HEX16(3U, a.steps_removed);
	TEST_ASSERT_EQUAL_HEX8(0x20U, a.time_source);
}

static void test_announce_decode_negative_utc_offset(void)
{
	/*
	 * currentUtcOffset is Integer16 (§13.5.2.2). It is positive in reality,
	 * but the codec must sign-extend rather than wrap: 0xFFDB is -37.
	 */
	uint8_t buf[PTP_ANNOUNCE_LEN];
	ptp_announce_t a;

	memcpy(buf, announce_vec, sizeof(buf));
	buf[0x2C] = 0xFFU;
	buf[0x2D] = 0xDBU;
	TEST_ASSERT_EQUAL_INT(0, ptp_announce_decode(buf, sizeof(buf), &a));
	TEST_ASSERT_EQUAL_INT16(-37, a.current_utc_offset);

	buf[0x2C] = 0x80U;
	buf[0x2D] = 0x00U;
	TEST_ASSERT_EQUAL_INT(0, ptp_announce_decode(buf, sizeof(buf), &a));
	TEST_ASSERT_EQUAL_INT16(INT16_MIN, a.current_utc_offset);
}

static void test_announce_encode_matches_the_vector(void)
{
	ptp_hdr_t h;
	ptp_announce_t a;
	uint8_t out[PTP_ANNOUNCE_LEN];
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(announce_vec, sizeof(announce_vec), &h));
	TEST_ASSERT_EQUAL_INT(0, ptp_announce_decode(announce_vec, sizeof(announce_vec), &a));

	/* Deliberately wrong: the encoder must overwrite it with the true length. */
	h.msg_length = 0xFFFFU;

	memset(out, 0x5A, sizeof(out));
	TEST_ASSERT_EQUAL_INT(0, ptp_announce_encode(out, sizeof(out), &h, &a, &len));
	TEST_ASSERT_EQUAL_size_t(PTP_ANNOUNCE_LEN, len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(announce_vec, out, PTP_ANNOUNCE_LEN);
}

static void test_announce_encode_negative_utc_offset(void)
{
	ptp_hdr_t h;
	ptp_announce_t a;
	uint8_t out[PTP_ANNOUNCE_LEN];

	TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(announce_vec, sizeof(announce_vec), &h));
	TEST_ASSERT_EQUAL_INT(0, ptp_announce_decode(announce_vec, sizeof(announce_vec), &a));

	a.current_utc_offset = -37;
	TEST_ASSERT_EQUAL_INT(0, ptp_announce_encode(out, sizeof(out), &h, &a, NULL));
	TEST_ASSERT_EQUAL_HEX8(0xFFU, out[0x2C]);
	TEST_ASSERT_EQUAL_HEX8(0xDBU, out[0x2D]);
}

static void test_announce_rejects_bad_input(void)
{
	ptp_hdr_t h;
	ptp_announce_t a;
	uint8_t out[PTP_ANNOUNCE_LEN];

	TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(announce_vec, sizeof(announce_vec), &h));
	TEST_ASSERT_EQUAL_INT(0, ptp_announce_decode(announce_vec, sizeof(announce_vec), &a));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_announce_decode(NULL, PTP_ANNOUNCE_LEN, &a));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_announce_decode(announce_vec, PTP_ANNOUNCE_LEN, NULL));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
		ptp_announce_decode(announce_vec, PTP_ANNOUNCE_LEN - 1U, &a));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_announce_encode(NULL, sizeof(out), &h, &a, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_announce_encode(out, sizeof(out), NULL, &a, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_announce_encode(out, sizeof(out), &h, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
		ptp_announce_encode(out, PTP_ANNOUNCE_LEN - 1U, &h, &a, NULL));
}

/* -------------------------------------------------------------- delay_resp -- */

static void test_delay_resp_decode_every_field(void)
{
	ptp_hdr_t h;
	ptp_timestamp_t ts;
	ptp_port_id_t req;

	TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(delay_resp_vec, sizeof(delay_resp_vec), &h));
	TEST_ASSERT_EQUAL_HEX8(0x09U, h.msg_type);
	TEST_ASSERT_EQUAL_HEX8(42U, h.domain);
	TEST_ASSERT_EQUAL_HEX8(7U, h.minor_sdo_id);
	TEST_ASSERT_EQUAL_HEX16(54U, h.msg_length);
	TEST_ASSERT_EQUAL_HEX8(3U, h.control);
	TEST_ASSERT_EQUAL_HEX16(7U, h.seq_id);

	TEST_ASSERT_EQUAL_INT(0,
		ptp_delay_resp_decode(delay_resp_vec, sizeof(delay_resp_vec), &ts, &req));

	/* The 48-bit seconds field at its maximum, and the largest legal ns. */
	TEST_ASSERT_EQUAL_HEX64(0xFFFFFFFFFFFFULL, ts.seconds);
	TEST_ASSERT_EQUAL_UINT32(999999999U, ts.nanoseconds);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(&delay_resp_vec[0x2C], req.clock_id.id, 8);
	TEST_ASSERT_EQUAL_HEX16(99U, req.port_number);
}

static void test_delay_resp_encode_matches_the_vector(void)
{
	ptp_hdr_t h;
	ptp_timestamp_t ts;
	ptp_port_id_t req;
	uint8_t out[PTP_DELAY_RESP_LEN];
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(delay_resp_vec, sizeof(delay_resp_vec), &h));
	TEST_ASSERT_EQUAL_INT(0,
		ptp_delay_resp_decode(delay_resp_vec, sizeof(delay_resp_vec), &ts, &req));

	h.msg_length = 0U;
	memset(out, 0x5A, sizeof(out));
	TEST_ASSERT_EQUAL_INT(0,
		ptp_delay_resp_encode(out, sizeof(out), &h, &ts, &req, &len));
	TEST_ASSERT_EQUAL_size_t(PTP_DELAY_RESP_LEN, len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(delay_resp_vec, out, PTP_DELAY_RESP_LEN);
}

static void test_delay_resp_optional_outputs_and_bad_input(void)
{
	ptp_hdr_t h;
	ptp_timestamp_t ts;
	ptp_port_id_t req;
	uint8_t out[PTP_DELAY_RESP_LEN];

	TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(delay_resp_vec, sizeof(delay_resp_vec), &h));
	TEST_ASSERT_EQUAL_INT(0,
		ptp_delay_resp_decode(delay_resp_vec, sizeof(delay_resp_vec), &ts, &req));

	/* Either output may be dropped. */
	TEST_ASSERT_EQUAL_INT(0,
		ptp_delay_resp_decode(delay_resp_vec, sizeof(delay_resp_vec), NULL, NULL));
	TEST_ASSERT_EQUAL_INT(0,
		ptp_delay_resp_decode(delay_resp_vec, sizeof(delay_resp_vec), &ts, NULL));
	TEST_ASSERT_EQUAL_INT(0,
		ptp_delay_resp_decode(delay_resp_vec, sizeof(delay_resp_vec), NULL, &req));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_delay_resp_decode(NULL, 54U, &ts, &req));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
		ptp_delay_resp_decode(delay_resp_vec, PTP_DELAY_RESP_LEN - 1U, &ts, &req));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
		ptp_delay_resp_encode(NULL, sizeof(out), &h, &ts, &req, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		ptp_delay_resp_encode(out, sizeof(out), NULL, &ts, &req, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		ptp_delay_resp_encode(out, sizeof(out), &h, NULL, &req, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		ptp_delay_resp_encode(out, sizeof(out), &h, &ts, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
		ptp_delay_resp_encode(out, PTP_DELAY_RESP_LEN - 1U, &h, &ts, &req, NULL));
}

/* ------------------------------------------------------------------- sync -- */

static void test_tsmsg_decode_and_encode(void)
{
	ptp_hdr_t h;
	ptp_timestamp_t ts;
	uint8_t out[PTP_TSMSG_LEN];
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(sync_vec, sizeof(sync_vec), &h));
	TEST_ASSERT_EQUAL_INT(0, ptp_tsmsg_decode(sync_vec, sizeof(sync_vec), &ts));

	TEST_ASSERT_EQUAL_HEX64(0x0000685D3B00ULL, ts.seconds);
	TEST_ASSERT_EQUAL_UINT32(500000000U, ts.nanoseconds);

	h.msg_length = 1U;
	memset(out, 0x5A, sizeof(out));
	TEST_ASSERT_EQUAL_INT(0, ptp_tsmsg_encode(out, sizeof(out), &h, &ts, &len));
	TEST_ASSERT_EQUAL_size_t(PTP_TSMSG_LEN, len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(sync_vec, out, PTP_TSMSG_LEN);
}

static void test_tsmsg_rejects_bad_input(void)
{
	ptp_hdr_t h;
	ptp_timestamp_t ts;
	uint8_t out[PTP_TSMSG_LEN];

	TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(sync_vec, sizeof(sync_vec), &h));
	TEST_ASSERT_EQUAL_INT(0, ptp_tsmsg_decode(sync_vec, sizeof(sync_vec), &ts));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_tsmsg_decode(NULL, PTP_TSMSG_LEN, &ts));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_tsmsg_decode(sync_vec, PTP_TSMSG_LEN, NULL));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ptp_tsmsg_decode(sync_vec, PTP_TSMSG_LEN - 1U, &ts));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_tsmsg_encode(NULL, sizeof(out), &h, &ts, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_tsmsg_encode(out, sizeof(out), NULL, &ts, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_tsmsg_encode(out, sizeof(out), &h, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
		ptp_tsmsg_encode(out, PTP_TSMSG_LEN - 1U, &h, &ts, NULL));
}

/* -------------------------------------------------------------- timestamp -- */

static void test_timestamp_48_bit_seconds_edge(void)
{
	uint8_t buf[PTP_TS_LEN];
	ptp_timestamp_t ts;
	static const uint8_t all_ones[PTP_TS_LEN] = {
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
	};

	/* The wire maximum survives a round trip. */
	ts.seconds = 0xFFFFFFFFFFFFULL;
	ts.nanoseconds = 0xFFFFFFFFU;
	ptp_ts_encode(buf, &ts);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(all_ones, buf, PTP_TS_LEN);

	memset(&ts, 0, sizeof(ts));
	ptp_ts_decode(buf, &ts);
	TEST_ASSERT_EQUAL_HEX64(0xFFFFFFFFFFFFULL, ts.seconds);
	TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFU, ts.nanoseconds);

	/* Anything above 48 bits is truncated to the field, never wrapped into ns. */
	ts.seconds = 0x0001000000000000ULL;
	ts.nanoseconds = 0U;
	ptp_ts_encode(buf, &ts);
	TEST_ASSERT_EACH_EQUAL_HEX8(0x00U, buf, PTP_TS_LEN);

	ts.seconds = 0xABCD000102030405ULL;
	ptp_ts_encode(buf, &ts);
	ptp_ts_decode(buf, &ts);
	TEST_ASSERT_EQUAL_HEX64(0x000102030405ULL, ts.seconds);
}

static void test_timestamp_ns_conversion(void)
{
	ptp_timestamp_t ts;

	ts = ptp_ts_from_ns(0U);
	TEST_ASSERT_EQUAL_HEX64(0U, ts.seconds);
	TEST_ASSERT_EQUAL_UINT32(0U, ts.nanoseconds);

	ts = ptp_ts_from_ns(999999999ULL);
	TEST_ASSERT_EQUAL_HEX64(0U, ts.seconds);
	TEST_ASSERT_EQUAL_UINT32(999999999U, ts.nanoseconds);

	ts = ptp_ts_from_ns(1000000000ULL);
	TEST_ASSERT_EQUAL_HEX64(1U, ts.seconds);
	TEST_ASSERT_EQUAL_UINT32(0U, ts.nanoseconds);

	ts = ptp_ts_from_ns(1750350080123456789ULL);
	TEST_ASSERT_EQUAL_HEX64(1750350080ULL, ts.seconds);
	TEST_ASSERT_EQUAL_UINT32(123456789U, ts.nanoseconds);
	TEST_ASSERT_EQUAL_UINT64(1750350080123456789ULL, ptp_ts_to_ns(&ts));
}

static void test_timestamp_to_ns_saturates(void)
{
	ptp_timestamp_t ts;

	/* UINT64_MAX ns is 18446744073.709551615 s. */
	ts.seconds = 18446744073ULL;
	ts.nanoseconds = 709551615U;
	TEST_ASSERT_EQUAL_UINT64(UINT64_MAX, ptp_ts_to_ns(&ts));

	ts.nanoseconds = 709551616U;
	TEST_ASSERT_EQUAL_UINT64(UINT64_MAX, ptp_ts_to_ns(&ts));

	ts.seconds = 18446744074ULL;
	ts.nanoseconds = 0U;
	TEST_ASSERT_EQUAL_UINT64(UINT64_MAX, ptp_ts_to_ns(&ts));

	/* The 48-bit maximum is far past the ns range and must not wrap. */
	ts.seconds = 0xFFFFFFFFFFFFULL;
	TEST_ASSERT_EQUAL_UINT64(UINT64_MAX, ptp_ts_to_ns(&ts));

	/* One nanosecond below the ceiling is still exact. */
	ts.seconds = 18446744073ULL;
	ts.nanoseconds = 709551614U;
	TEST_ASSERT_EQUAL_UINT64(UINT64_MAX - 1ULL, ptp_ts_to_ns(&ts));
}

/* ---------------------------------------------------------- small helpers -- */

static void test_clock_id_from_mac(void)
{
	/* §7.5.2.2.2: OUI, 0xFF, 0xFE, extension. */
	static const uint8_t mac[6] = { 0x00, 0x80, 0xE1, 0x11, 0x22, 0x33 };
	static const uint8_t want[8] = {
		0x00, 0x80, 0xE1, 0xFF, 0xFE, 0x11, 0x22, 0x33
	};
	ptp_clock_id_t id;

	TEST_ASSERT_EQUAL_INT(0, ptp_clock_id_from_mac(&id, mac));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want, id.id, 8);

	/* The vectors' sourcePortIdentity is that very EUI-64. */
	TEST_ASSERT_EQUAL_HEX8_ARRAY(&announce_vec[0x14], id.id, 8);

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_clock_id_from_mac(NULL, mac));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_clock_id_from_mac(&id, NULL));
}

static void test_identity_ordering(void)
{
	ptp_port_id_t a;
	ptp_port_id_t b;

	memset(&a, 0, sizeof(a));
	memset(&b, 0, sizeof(b));
	a.port_number = 1U;
	b.port_number = 1U;

	TEST_ASSERT_EQUAL_INT(0, ptp_port_id_cmp(&a, &b));
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_id_cmp(&a.clock_id, &b.clock_id));

	/* The clock identity dominates the port number. */
	b.clock_id.id[7] = 1U;
	b.port_number = 0U;
	TEST_ASSERT_TRUE(ptp_port_id_cmp(&a, &b) < 0);
	TEST_ASSERT_TRUE(ptp_port_id_cmp(&b, &a) > 0);
	TEST_ASSERT_TRUE(ptp_clock_id_cmp(&a.clock_id, &b.clock_id) < 0);

	/* The most significant octet dominates the least. */
	memset(&b, 0, sizeof(b));
	a.clock_id.id[7] = 0xFFU;
	b.clock_id.id[0] = 0x01U;
	TEST_ASSERT_TRUE(ptp_clock_id_cmp(&a.clock_id, &b.clock_id) < 0);

	/* Equal identities fall through to the port number. */
	memset(&a, 0, sizeof(a));
	memset(&b, 0, sizeof(b));
	a.port_number = 1U;
	b.port_number = 2U;
	TEST_ASSERT_TRUE(ptp_port_id_cmp(&a, &b) < 0);
	TEST_ASSERT_TRUE(ptp_port_id_cmp(&b, &a) > 0);
}

static void test_min_len_and_control_field(void)
{
	/* Widths summed from the Table 40..49 field lists. */
	TEST_ASSERT_EQUAL_size_t(44U, ptp_msg_min_len(PTP_MSG_SYNC));
	TEST_ASSERT_EQUAL_size_t(44U, ptp_msg_min_len(PTP_MSG_DELAY_REQ));
	TEST_ASSERT_EQUAL_size_t(44U, ptp_msg_min_len(PTP_MSG_FOLLOW_UP));
	TEST_ASSERT_EQUAL_size_t(54U, ptp_msg_min_len(PTP_MSG_PDELAY_REQ));
	TEST_ASSERT_EQUAL_size_t(54U, ptp_msg_min_len(PTP_MSG_PDELAY_RESP));
	TEST_ASSERT_EQUAL_size_t(54U, ptp_msg_min_len(PTP_MSG_PDELAY_RESP_FOLLOW_UP));
	TEST_ASSERT_EQUAL_size_t(54U, ptp_msg_min_len(PTP_MSG_DELAY_RESP));
	TEST_ASSERT_EQUAL_size_t(64U, ptp_msg_min_len(PTP_MSG_ANNOUNCE));
	TEST_ASSERT_EQUAL_size_t(44U, ptp_msg_min_len(PTP_MSG_SIGNALING));
	TEST_ASSERT_EQUAL_size_t(48U, ptp_msg_min_len(PTP_MSG_MANAGEMENT));
	TEST_ASSERT_EQUAL_size_t(34U, ptp_msg_min_len(0x04U));
	TEST_ASSERT_EQUAL_size_t(34U, ptp_msg_min_len(0x0FU));

	/* Table 39, the legacy controlField codes. */
	TEST_ASSERT_EQUAL_HEX8(0U, ptp_msg_control_field(PTP_MSG_SYNC));
	TEST_ASSERT_EQUAL_HEX8(1U, ptp_msg_control_field(PTP_MSG_DELAY_REQ));
	TEST_ASSERT_EQUAL_HEX8(2U, ptp_msg_control_field(PTP_MSG_FOLLOW_UP));
	TEST_ASSERT_EQUAL_HEX8(3U, ptp_msg_control_field(PTP_MSG_DELAY_RESP));
	TEST_ASSERT_EQUAL_HEX8(4U, ptp_msg_control_field(PTP_MSG_MANAGEMENT));
	TEST_ASSERT_EQUAL_HEX8(5U, ptp_msg_control_field(PTP_MSG_ANNOUNCE));
	TEST_ASSERT_EQUAL_HEX8(5U, ptp_msg_control_field(PTP_MSG_SIGNALING));
	TEST_ASSERT_EQUAL_HEX8(5U, ptp_msg_control_field(PTP_MSG_PDELAY_REQ));
	TEST_ASSERT_EQUAL_HEX8(5U, ptp_msg_control_field(0x07U));

	/* The vectors' controlField octets agree with the table. */
	TEST_ASSERT_EQUAL_HEX8(ptp_msg_control_field(PTP_MSG_ANNOUNCE), announce_vec[0x20]);
	TEST_ASSERT_EQUAL_HEX8(ptp_msg_control_field(PTP_MSG_DELAY_RESP), delay_resp_vec[0x20]);
	TEST_ASSERT_EQUAL_HEX8(ptp_msg_control_field(PTP_MSG_SYNC), sync_vec[0x20]);
}

static void test_msg_is_event(void)
{
	TEST_ASSERT_TRUE(ptp_msg_is_event(PTP_MSG_SYNC));
	TEST_ASSERT_TRUE(ptp_msg_is_event(PTP_MSG_DELAY_REQ));
	TEST_ASSERT_TRUE(ptp_msg_is_event(PTP_MSG_PDELAY_REQ));
	TEST_ASSERT_TRUE(ptp_msg_is_event(PTP_MSG_PDELAY_RESP));
	TEST_ASSERT_FALSE(ptp_msg_is_event(PTP_MSG_FOLLOW_UP));
	TEST_ASSERT_FALSE(ptp_msg_is_event(PTP_MSG_ANNOUNCE));
	TEST_ASSERT_FALSE(ptp_msg_is_event(PTP_MSG_MANAGEMENT));
}

static void test_msg_type_names(void)
{
	TEST_ASSERT_EQUAL_STRING("Sync", ptp_msg_type_name(PTP_MSG_SYNC));
	TEST_ASSERT_EQUAL_STRING("Delay_Req", ptp_msg_type_name(PTP_MSG_DELAY_REQ));
	TEST_ASSERT_EQUAL_STRING("Pdelay_Req", ptp_msg_type_name(PTP_MSG_PDELAY_REQ));
	TEST_ASSERT_EQUAL_STRING("Pdelay_Resp", ptp_msg_type_name(PTP_MSG_PDELAY_RESP));
	TEST_ASSERT_EQUAL_STRING("Follow_Up", ptp_msg_type_name(PTP_MSG_FOLLOW_UP));
	TEST_ASSERT_EQUAL_STRING("Delay_Resp", ptp_msg_type_name(PTP_MSG_DELAY_RESP));
	TEST_ASSERT_EQUAL_STRING("Pdelay_Resp_Follow_Up",
				 ptp_msg_type_name(PTP_MSG_PDELAY_RESP_FOLLOW_UP));
	TEST_ASSERT_EQUAL_STRING("Announce", ptp_msg_type_name(PTP_MSG_ANNOUNCE));
	TEST_ASSERT_EQUAL_STRING("Signaling", ptp_msg_type_name(PTP_MSG_SIGNALING));
	TEST_ASSERT_EQUAL_STRING("Management", ptp_msg_type_name(PTP_MSG_MANAGEMENT));
	TEST_ASSERT_EQUAL_STRING("reserved", ptp_msg_type_name(0x04U));
	TEST_ASSERT_EQUAL_STRING("reserved", ptp_msg_type_name(0x0FU));
}

static void test_log_interval_ms(void)
{
	TEST_ASSERT_EQUAL_UINT32(1000U, ptp_log_interval_ms(0));
	TEST_ASSERT_EQUAL_UINT32(2000U, ptp_log_interval_ms(1));
	TEST_ASSERT_EQUAL_UINT32(500U, ptp_log_interval_ms(-1));
	TEST_ASSERT_EQUAL_UINT32(250U, ptp_log_interval_ms(-2));
	TEST_ASSERT_EQUAL_UINT32(125U, ptp_log_interval_ms(-3));
	TEST_ASSERT_EQUAL_UINT32(63U, ptp_log_interval_ms(-4));   /* 62.5 rounds up */
	TEST_ASSERT_EQUAL_UINT32(31U, ptp_log_interval_ms(-5));   /* 31.25 rounds down */
	TEST_ASSERT_EQUAL_UINT32(16U, ptp_log_interval_ms(-6));   /* 15.625 rounds up */
	TEST_ASSERT_EQUAL_UINT32(8U, ptp_log_interval_ms(-7));    /* 7.8125 rounds up */
	TEST_ASSERT_EQUAL_UINT32(128000U, ptp_log_interval_ms(7));

	/* Out-of-range exponents clamp rather than shifting off the end. */
	TEST_ASSERT_EQUAL_UINT32(8U, ptp_log_interval_ms(-8));
	TEST_ASSERT_EQUAL_UINT32(8U, ptp_log_interval_ms(INT8_MIN));
	TEST_ASSERT_EQUAL_UINT32(128000U, ptp_log_interval_ms(8));
	TEST_ASSERT_EQUAL_UINT32(128000U, ptp_log_interval_ms(PTP_LOG_INTERVAL_UNSPEC));
}

/* ------------------------------------------------------------------ fuzz -- */

/*
 * Decoders are fed straight from the network, so the contract is that no input
 * of any length can read out of bounds or trap. Under ASan (see the report) the
 * same loop proves the bound; without it, it still proves nothing crashes and
 * that a decode that reports success has produced a self-consistent header.
 */
static void test_fuzz_never_reads_out_of_bounds(void)
{
	test_rng_t rng;
	unsigned int iter;

	test_rng_init(&rng, 0x1588C0DEU);

	for (iter = 0U; iter < 20000U; iter++) {
		uint8_t buf[80];
		size_t len = test_rng_below(&rng, sizeof(buf) + 1U);
		ptp_hdr_t h;
		int rc;

		test_rng_fill(&rng, buf, len);

		/*
		 * Half the iterations force versionPTP 2 and a plausible
		 * messageLength, so the run actually reaches the body decoders
		 * instead of bouncing off the version check.
		 */
		if (((iter & 1U) != 0U) && (len >= PTP_HDR_LEN)) {
			buf[1] = (uint8_t)((buf[1] & 0xF0U) | PTP_VERSION);
			buf[2] = 0U;
			buf[3] = (uint8_t)len;
		}

		rc = ptp_hdr_decode(buf, len, &h);
		if (rc != 0) {
			TEST_ASSERT_TRUE((rc == -EBADMSG) || (rc == -EPROTO));
			continue;
		}

		/* A successful decode must be internally consistent. */
		TEST_ASSERT_EQUAL_HEX8(PTP_VERSION, h.version);
		TEST_ASSERT_LESS_OR_EQUAL_size_t(len, (size_t)h.msg_length);
		TEST_ASSERT_GREATER_OR_EQUAL_size_t(ptp_msg_min_len(h.msg_type),
						    (size_t)h.msg_length);

		switch (h.msg_type) {
		case PTP_MSG_ANNOUNCE: {
			ptp_announce_t a;

			TEST_ASSERT_EQUAL_INT(0, ptp_announce_decode(buf, len, &a));
			break;
		}
		case PTP_MSG_SYNC:
		case PTP_MSG_DELAY_REQ:
		case PTP_MSG_FOLLOW_UP: {
			ptp_timestamp_t ts;

			TEST_ASSERT_EQUAL_INT(0, ptp_tsmsg_decode(buf, len, &ts));
			break;
		}
		case PTP_MSG_DELAY_RESP: {
			ptp_timestamp_t ts;
			ptp_port_id_t req;

			TEST_ASSERT_EQUAL_INT(0,
				ptp_delay_resp_decode(buf, len, &ts, &req));
			break;
		}
		default:
			/* Management, Signaling, Pdelay: header only, ignored. */
			break;
		}
	}
}

static void test_fuzz_truncations_of_valid_vectors(void)
{
	static const struct {
		const uint8_t *buf;
		size_t len;
	} vecs[] = {
		{ announce_vec, sizeof(announce_vec) },
		{ delay_resp_vec, sizeof(delay_resp_vec) },
		{ sync_vec, sizeof(sync_vec) },
	};
	size_t v;

	for (v = 0U; v < ARRAY_LEN(vecs); v++) {
		ptp_hdr_t h;
		size_t n;

		for (n = 0U; n < vecs[v].len; n++) {
			/*
			 * Every prefix shorter than the whole message must be
			 * rejected: messageLength always exceeds the prefix.
			 */
			TEST_ASSERT_EQUAL_INT(-EBADMSG,
				ptp_hdr_decode(vecs[v].buf, n, &h));
		}
		/* The untruncated vector still decodes, so the loop proves something. */
		TEST_ASSERT_EQUAL_INT(0, ptp_hdr_decode(vecs[v].buf, vecs[v].len, &h));
	}
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_hdr_decode_announce_every_field);
	RUN_TEST(test_hdr_decode_sync_sdo_and_flags);
	RUN_TEST(test_hdr_correction_field_sign);
	RUN_TEST(test_hdr_decode_rejects_bad_input);
	RUN_TEST(test_hdr_decode_tolerates_trailing_octets);
	RUN_TEST(test_hdr_encode_matches_the_vector);
	RUN_TEST(test_hdr_encode_packs_the_nibbles);
	RUN_TEST(test_hdr_encode_rejects_bad_input);

	RUN_TEST(test_announce_decode_every_field);
	RUN_TEST(test_announce_decode_negative_utc_offset);
	RUN_TEST(test_announce_encode_matches_the_vector);
	RUN_TEST(test_announce_encode_negative_utc_offset);
	RUN_TEST(test_announce_rejects_bad_input);

	RUN_TEST(test_delay_resp_decode_every_field);
	RUN_TEST(test_delay_resp_encode_matches_the_vector);
	RUN_TEST(test_delay_resp_optional_outputs_and_bad_input);

	RUN_TEST(test_tsmsg_decode_and_encode);
	RUN_TEST(test_tsmsg_rejects_bad_input);

	RUN_TEST(test_timestamp_48_bit_seconds_edge);
	RUN_TEST(test_timestamp_ns_conversion);
	RUN_TEST(test_timestamp_to_ns_saturates);

	RUN_TEST(test_clock_id_from_mac);
	RUN_TEST(test_identity_ordering);
	RUN_TEST(test_min_len_and_control_field);
	RUN_TEST(test_msg_is_event);
	RUN_TEST(test_msg_type_names);
	RUN_TEST(test_log_interval_ms);

	RUN_TEST(test_fuzz_never_reads_out_of_bounds);
	RUN_TEST(test_fuzz_truncations_of_valid_vectors);

	return UNITY_END();
}
