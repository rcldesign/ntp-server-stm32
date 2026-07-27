/*
 * STS1000 "Meridian" — core/ubx unit tests.
 *
 * Anchors, in the order they matter:
 *
 * 1. The checksum and the frame builder are pinned to four UBX command strings
 *    that exist outside this repository — the poll and reset frames every
 *    u-blox integration guide, u-center log and forum post quotes verbatim:
 *
 *      UBX-CFG-PRT poll     B5 62 06 00 00 00 06 18
 *      UBX-MON-VER poll     B5 62 0A 04 00 00 0E 34
 *      UBX-NAV-PVT poll     B5 62 01 07 00 00 08 19
 *      UBX-CFG-RST hotstart B5 62 06 04 04 00 00 00 09 00 17 76
 *      UBX-CFG-RST coldstart B5 62 06 04 04 00 FF FF 02 00 0E 61
 *
 *    Each CK_A/CK_B pair is re-derived by hand in the comment beside it, so the
 *    test proves the implementation matches the ICD's algorithm and not merely
 *    itself.
 *
 * 2. Message-layout vectors are transcribed from the generation-9 interface
 *    description field tables, byte offset by byte offset, with the offsets in
 *    the comments. Their frames are assembled with ubx_frame(), whose checksum
 *    is already anchored by (1) — the vector is testing field placement, and
 *    hand-adding 96 bytes of Fletcher would only add a transcription risk that
 *    proves nothing further.
 *
 * 3. Both UBX-CFG-VALSET vectors are hand-computed end to end, including the
 *    checksum, because the VALSET payload layout (version/layers/reserved, then
 *    little-endian key then value) is the thing most easily got wrong.
 *
 * 4. The configuration key table is checked against the key-ID encoding rules
 *    (group field, storage-size field) — that catches a transposed digit in an
 *    ID nobody has verified against the ICD PDF yet.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"
#include "ubx/ubx.h"

#include "test_support.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

static uint8_t g_pbuf[UBX_PAYLOAD_CAP_DEFAULT];
static uint8_t g_frame[UBX_PAYLOAD_CAP_DEFAULT + UBX_FRAME_OVERHEAD];
static ubx_parser_t g_parser;

static void parser_fresh(uint16_t cap)
{
	TEST_ASSERT_EQUAL_INT(0, ubx_parser_init(&g_parser, g_pbuf, cap));
}

/* Feed a byte run; return how many frames completed. Asserts no parse error. */
static unsigned int feed_clean(const uint8_t *p, size_t n)
{
	unsigned int frames = 0U;
	size_t i;

	for (i = 0U; i < n; i++) {
		int rc = ubx_parse_byte(&g_parser, p[i]);

		TEST_ASSERT_TRUE_MESSAGE(rc >= 0, "unexpected parse error");
		frames += (rc == 1) ? 1U : 0U;
	}
	return frames;
}

/* Feed a byte run tolerating errors; return how many frames completed. */
static unsigned int feed_any(const uint8_t *p, size_t n)
{
	unsigned int frames = 0U;
	size_t i;

	for (i = 0U; i < n; i++) {
		frames += (ubx_parse_byte(&g_parser, p[i]) == 1) ? 1U : 0U;
	}
	return frames;
}

/* As feed_any(), but count only frames of the given class/id. */
static unsigned int feed_count(const uint8_t *p, size_t n, uint8_t cls, uint8_t id)
{
	unsigned int frames = 0U;
	size_t i;

	for (i = 0U; i < n; i++) {
		ubx_msg_t m;

		if (ubx_parse_byte(&g_parser, p[i]) != 1) {
			continue;
		}
		TEST_ASSERT_EQUAL_INT(0, ubx_parser_msg(&g_parser, &m));
		if (ubx_msg_is(&m, cls, id)) {
			frames++;
		}
	}
	return frames;
}

/* --------------------------------------------------- checksum + builder -- */

/*
 * UBX-CFG-PRT poll: class 06, id 00, len 0000.
 *   A: 06 06 06 06        (06 +00 +00 +00)
 *   B: 06 0C 12 18
 * -> CK_A 06, CK_B 18.
 */
static void test_checksum_cfg_prt_poll(void)
{
	static const uint8_t body[] = {0x06, 0x00, 0x00, 0x00};
	uint8_t a = 0xFFU;
	uint8_t b = 0xFFU;

	ubx_checksum(body, sizeof(body), &a, &b);
	TEST_ASSERT_EQUAL_HEX8(0x06, a);
	TEST_ASSERT_EQUAL_HEX8(0x18, b);
}

/*
 * UBX-MON-VER poll: class 0A, id 04, len 0000.
 *   A: 0A 0E 0E 0E
 *   B: 0A 18 26 34
 */
static void test_checksum_mon_ver_poll(void)
{
	static const uint8_t body[] = {0x0A, 0x04, 0x00, 0x00};
	uint8_t a = 0U;
	uint8_t b = 0U;

	ubx_checksum(body, sizeof(body), &a, &b);
	TEST_ASSERT_EQUAL_HEX8(0x0E, a);
	TEST_ASSERT_EQUAL_HEX8(0x34, b);
}

static void test_checksum_null_tolerant(void)
{
	uint8_t a = 0x5AU;
	uint8_t b = 0xA5U;

	/* NULL data with a non-zero length must not read anything. */
	ubx_checksum(NULL, 8U, &a, &b);
	TEST_ASSERT_EQUAL_HEX8(0x00, a);
	TEST_ASSERT_EQUAL_HEX8(0x00, b);

	/* NULL outputs are simply ignored. */
	ubx_checksum((const uint8_t *)"x", 1U, NULL, &b);
	ubx_checksum((const uint8_t *)"x", 1U, &a, NULL);
}

static void test_frame_known_polls(void)
{
	static const uint8_t cfg_prt[] = {0xB5, 0x62, 0x06, 0x00, 0x00, 0x00,
					  0x06, 0x18};
	static const uint8_t mon_ver[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00,
					  0x0E, 0x34};
	static const uint8_t nav_pvt[] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00,
					  0x08, 0x19};
	uint8_t buf[16];

	TEST_ASSERT_EQUAL_INT(8, ubx_poll(buf, sizeof(buf), 0x06U, 0x00U));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(cfg_prt, buf, sizeof(cfg_prt));

	TEST_ASSERT_EQUAL_INT(8, ubx_poll(buf, sizeof(buf), 0x0AU, 0x04U));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(mon_ver, buf, sizeof(mon_ver));

	TEST_ASSERT_EQUAL_INT(8, ubx_poll(buf, sizeof(buf), UBX_CLASS_NAV,
					  UBX_ID_NAV_PVT));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(nav_pvt, buf, sizeof(nav_pvt));
}

/*
 * UBX-CFG-RST with a payload. Hot start (navBbrMask 0x0000, resetMode 09):
 *   body 06 04 04 00 00 00 09 00
 *   A:   06 0A 0E 0E 0E 0E 17 17
 *   B:   06 10 1E 2C 3A 48 5F 76
 * Cold start (navBbrMask 0xFFFF, resetMode 02):
 *   body 06 04 04 00 FF FF 02 00
 *   A:   06 0A 0E 0E 0D 0C 0E 0E
 *   B:   06 10 1E 2C 39 45 53 61
 */
static void test_frame_with_payload_cfg_rst(void)
{
	static const uint8_t hot_payload[] = {0x00, 0x00, 0x09, 0x00};
	static const uint8_t hot[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00,
				      0x00, 0x00, 0x09, 0x00, 0x17, 0x76};
	static const uint8_t cold_payload[] = {0xFF, 0xFF, 0x02, 0x00};
	static const uint8_t cold[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00,
				       0xFF, 0xFF, 0x02, 0x00, 0x0E, 0x61};
	uint8_t buf[16];

	TEST_ASSERT_EQUAL_INT(12, ubx_frame(buf, sizeof(buf), 0x06U, 0x04U,
					    hot_payload, sizeof(hot_payload)));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(hot, buf, sizeof(hot));

	TEST_ASSERT_EQUAL_INT(12, ubx_frame(buf, sizeof(buf), 0x06U, 0x04U,
					    cold_payload, sizeof(cold_payload)));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(cold, buf, sizeof(cold));
}

static void test_frame_argument_errors(void)
{
	uint8_t buf[16];
	uint8_t one = 0U;

	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_frame(NULL, 16U, 1U, 2U, NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_frame(buf, sizeof(buf), 1U, 2U, NULL, 4U));
	/* Length is bounds-checked before the buffer is touched. */
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE,
			      ubx_frame(buf, sizeof(buf), 1U, 2U, &one, 0x10000U));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ubx_frame(buf, 8U, 1U, 2U, &one, 1U));
	/* Exactly enough room is enough. */
	TEST_ASSERT_EQUAL_INT(9, ubx_frame(buf, 9U, 1U, 2U, &one, 1U));
}

/* -------------------------------------------------------------- parser -- */

static void test_parser_init_errors(void)
{
	ubx_parser_t p;
	ubx_msg_t m;

	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_parser_init(NULL, g_pbuf, 16U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_parser_init(&p, NULL, 16U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_parser_init(&p, g_pbuf, 0U));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_parse_byte(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_parser_msg(NULL, &m));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_parser_stats(NULL, NULL));

	/* reset() and stats() on a NULL parser must be inert, not a crash. */
	ubx_parser_reset(NULL);

	TEST_ASSERT_EQUAL_INT(0, ubx_parser_init(&p, g_pbuf, 16U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_parser_msg(&p, NULL));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, ubx_parser_msg(&p, &m));
}

static void test_parser_zero_length_frame(void)
{
	/* An ACK has a payload; a poll response echo does not. Length 0 must
	 * skip the payload state entirely rather than stalling. */
	static const uint8_t poll[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00,
				       0x0E, 0x34};
	ubx_msg_t m;

	parser_fresh(UBX_PAYLOAD_CAP_DEFAULT);
	TEST_ASSERT_EQUAL_UINT(1U, feed_clean(poll, sizeof(poll)));
	TEST_ASSERT_EQUAL_INT(0, ubx_parser_msg(&g_parser, &m));
	TEST_ASSERT_EQUAL_HEX8(0x0A, m.cls);
	TEST_ASSERT_EQUAL_HEX8(0x04, m.id);
	TEST_ASSERT_EQUAL_UINT16(0U, m.len);
	TEST_ASSERT_TRUE(ubx_msg_is(&m, 0x0AU, 0x04U));
	TEST_ASSERT_FALSE(ubx_msg_is(&m, 0x0AU, 0x05U));
	TEST_ASSERT_FALSE(ubx_msg_is(NULL, 0x0AU, 0x04U));
}

/* B5 immediately before the real sync pair must not be swallowed. */
static void test_parser_resync_double_sync1(void)
{
	static const uint8_t stream[] = {0xB5, 0xB5, 0x62, 0x0A, 0x04, 0x00,
					 0x00, 0x0E, 0x34};
	ubx_msg_t m;
	ubx_stats_t st;

	parser_fresh(UBX_PAYLOAD_CAP_DEFAULT);
	TEST_ASSERT_EQUAL_UINT(1U, feed_clean(stream, sizeof(stream)));
	TEST_ASSERT_EQUAL_INT(0, ubx_parser_msg(&g_parser, &m));
	TEST_ASSERT_EQUAL_HEX8(0x0A, m.cls);

	TEST_ASSERT_EQUAL_INT(0, ubx_parser_stats(&g_parser, &st));
	TEST_ASSERT_EQUAL_UINT32(1U, st.frames);
	TEST_ASSERT_EQUAL_UINT32(1U, st.hunt_bytes); /* the extra B5 */
}

/*
 * A frame whose CK_B is itself 0xB5, immediately followed by the rest of a
 * valid frame. Reporting the checksum error must not consume the 0xB5 — this
 * is the "resync without losing bytes" property in its sharpest form.
 */
static void test_parser_resync_from_checksum_error(void)
{
	static const uint8_t stream[] = {
		/* corrupt ACK-ACK: CK_B forced to B5 instead of C1 */
		0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x8A, 0x98, 0xB5,
		/* ...which is the sync1 of a good ACK-ACK */
		0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x8A, 0x98, 0xC1,
	};
	ubx_msg_t m;
	ubx_stats_t st;
	unsigned int frames = 0U;
	unsigned int errors = 0U;
	size_t i;

	parser_fresh(UBX_PAYLOAD_CAP_DEFAULT);
	for (i = 0U; i < sizeof(stream); i++) {
		int rc = ubx_parse_byte(&g_parser, stream[i]);

		if (rc == 1) {
			frames++;
		} else if (rc == -EBADMSG) {
			errors++;
		} else {
			TEST_ASSERT_EQUAL_INT(0, rc);
		}
	}

	TEST_ASSERT_EQUAL_UINT(1U, errors);
	TEST_ASSERT_EQUAL_UINT(1U, frames);
	TEST_ASSERT_EQUAL_INT(0, ubx_parser_msg(&g_parser, &m));
	TEST_ASSERT_EQUAL_HEX8(UBX_CLASS_ACK, m.cls);
	TEST_ASSERT_EQUAL_HEX8(UBX_ID_ACK_ACK, m.id);

	TEST_ASSERT_EQUAL_INT(0, ubx_parser_stats(&g_parser, &st));
	TEST_ASSERT_EQUAL_UINT32(1U, st.ck_errors);
}

/* Same property on the length-bound rejection path. */
static void test_parser_resync_from_length_error(void)
{
	static const uint8_t stream[] = {
		/* declares 0xB500 payload bytes against a 32-byte cap */
		0xB5, 0x62, 0x05, 0x01, 0x00, 0xB5,
		/* the rejected 0xB5 opens this good ACK-ACK */
		0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x8A, 0x98, 0xC1,
	};
	ubx_msg_t m;
	ubx_stats_t st;
	unsigned int frames = 0U;
	unsigned int errors = 0U;
	size_t i;

	parser_fresh(32U);
	for (i = 0U; i < sizeof(stream); i++) {
		int rc = ubx_parse_byte(&g_parser, stream[i]);

		if (rc == 1) {
			frames++;
		} else if (rc == -EMSGSIZE) {
			errors++;
		} else {
			TEST_ASSERT_EQUAL_INT(0, rc);
		}
	}

	TEST_ASSERT_EQUAL_UINT(1U, errors);
	TEST_ASSERT_EQUAL_UINT(1U, frames);
	TEST_ASSERT_EQUAL_INT(0, ubx_parser_msg(&g_parser, &m));
	TEST_ASSERT_EQUAL_HEX8(UBX_ID_ACK_ACK, m.id);

	TEST_ASSERT_EQUAL_INT(0, ubx_parser_stats(&g_parser, &st));
	TEST_ASSERT_EQUAL_UINT32(1U, st.len_errors);
}

/* Sync bytes inside a payload are payload, not framing. */
static void test_parser_sync_pattern_inside_payload(void)
{
	static const uint8_t payload[] = {0xB5, 0x62, 0x01, 0x07, 0xFF, 0x00,
					  0xB5, 0x62};
	ubx_msg_t m;
	int len = ubx_frame(g_frame, sizeof(g_frame), UBX_CLASS_MON, 0x09U,
			    payload, sizeof(payload));

	TEST_ASSERT_TRUE(len > 0);
	parser_fresh(UBX_PAYLOAD_CAP_DEFAULT);
	TEST_ASSERT_EQUAL_UINT(1U, feed_clean(g_frame, (size_t)len));
	TEST_ASSERT_EQUAL_INT(0, ubx_parser_msg(&g_parser, &m));
	TEST_ASSERT_EQUAL_UINT16(sizeof(payload), m.len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, m.payload, sizeof(payload));
}

/* ----------------------------------------------------------- NAV-PVT ---- */

/*
 * UBX-NAV-PVT, 92 bytes, transcribed offset by offset from the generation-9
 * interface description. Filler 0xA5 marks every byte the decoder must NOT
 * read: velocity, heading, their accuracies, and the reserved runs. If a field
 * offset slips, an assertion lands on 0xA5A5A5A5 and says so loudly.
 */
static const uint8_t k_nav_pvt[UBX_LEN_NAV_PVT] = {
	/*  0 iTOW   = 259200000 ms */ 0x00, 0x14, 0x73, 0x0F,
	/*  4 year   = 2026 */         0xEA, 0x07,
	/*  6 month  = 7 */            0x07,
	/*  7 day    = 27 */           0x1B,
	/*  8 hour   = 12 */           0x0C,
	/*  9 min    = 34 */           0x22,
	/* 10 sec    = 56 */           0x38,
	/* 11 valid  = date|time|fullyResolved */ 0x07,
	/* 12 tAcc   = 42 ns */        0x2A, 0x00, 0x00, 0x00,
	/* 16 nano   = -1234 ns */     0x2E, 0xFB, 0xFF, 0xFF,
	/* 20 fixType= 5 TIME_ONLY */  0x05,
	/* 21 flags  = gnssFixOK | carrSoln=2 */ 0x81,
	/* 22 flags2 */                0xE0,
	/* 23 numSV  = 23 */           0x17,
	/* 24 lon    = -1220841234 */  0xEE, 0x70, 0x3B, 0xB7,
	/* 28 lat    =   473608123 */  0xBB, 0xAF, 0x3A, 0x1C,
	/* 32 height = 123456 mm */    0x40, 0xE2, 0x01, 0x00,
	/* 36 hMSL   =  98765 mm */    0xCD, 0x81, 0x01, 0x00,
	/* 40 hAcc   =   1500 mm */    0xDC, 0x05, 0x00, 0x00,
	/* 44 vAcc   =   2500 mm */    0xC4, 0x09, 0x00, 0x00,
	/* 48 velN..75 headAcc — never decoded */
	0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5,
	0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5,
	0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5,
	0xA5, 0xA5, 0xA5, 0xA5,
	/* 76 pDOP   = 145 (1.45) */   0x91, 0x00,
	/* 78 flags3 = invalidLlh */   0x01, 0x00,
	/* 80 reserved1 */             0xA5, 0xA5, 0xA5, 0xA5,
	/* 84 headVeh */               0xA5, 0xA5, 0xA5, 0xA5,
	/* 88 magDec */                0xA5, 0xA5,
	/* 90 magAcc */                0xA5, 0xA5,
};

static void make_msg(ubx_msg_t *m, uint8_t cls, uint8_t id,
		     const uint8_t *payload, uint16_t len)
{
	m->cls = cls;
	m->id = id;
	m->len = len;
	m->payload = payload;
}

static void test_nav_pvt_full_vector(void)
{
	ubx_nav_pvt_t pvt;
	ubx_msg_t m;

	make_msg(&m, UBX_CLASS_NAV, UBX_ID_NAV_PVT, k_nav_pvt, UBX_LEN_NAV_PVT);
	TEST_ASSERT_EQUAL_INT(0, ubx_parse_nav_pvt(&m, &pvt));

	TEST_ASSERT_EQUAL_UINT32(259200000UL, pvt.itow_ms);
	TEST_ASSERT_EQUAL_UINT16(2026U, pvt.year);
	TEST_ASSERT_EQUAL_UINT8(7U, pvt.month);
	TEST_ASSERT_EQUAL_UINT8(27U, pvt.day);
	TEST_ASSERT_EQUAL_UINT8(12U, pvt.hour);
	TEST_ASSERT_EQUAL_UINT8(34U, pvt.min);
	TEST_ASSERT_EQUAL_UINT8(56U, pvt.sec);
	TEST_ASSERT_EQUAL_HEX8(0x07, pvt.valid);
	TEST_ASSERT_TRUE(pvt.valid_date);
	TEST_ASSERT_TRUE(pvt.valid_time);
	TEST_ASSERT_TRUE(pvt.fully_resolved);
	TEST_ASSERT_FALSE(pvt.valid_mag);
	TEST_ASSERT_EQUAL_UINT32(42U, pvt.tacc_ns);
	TEST_ASSERT_EQUAL_INT32(-1234, pvt.nano_ns);
	TEST_ASSERT_EQUAL_UINT8(UBX_FIX_TIME_ONLY, pvt.fix_type);
	TEST_ASSERT_EQUAL_HEX8(0x81, pvt.flags);
	TEST_ASSERT_TRUE(pvt.gnss_fix_ok);
	TEST_ASSERT_FALSE(pvt.diff_soln);
	TEST_ASSERT_EQUAL_UINT8(2U, pvt.carr_soln);
	TEST_ASSERT_EQUAL_HEX8(0xE0, pvt.flags2);
	TEST_ASSERT_EQUAL_UINT8(23U, pvt.num_sv);
	TEST_ASSERT_EQUAL_INT32(-1220841234L, pvt.lon_1e7);
	TEST_ASSERT_EQUAL_INT32(473608123L, pvt.lat_1e7);
	TEST_ASSERT_EQUAL_INT32(123456L, pvt.height_mm);
	TEST_ASSERT_EQUAL_INT32(98765L, pvt.hmsl_mm);
	TEST_ASSERT_EQUAL_UINT32(1500U, pvt.hacc_mm);
	TEST_ASSERT_EQUAL_UINT32(2500U, pvt.vacc_mm);
	TEST_ASSERT_EQUAL_UINT16(145U, pvt.pdop);
	TEST_ASSERT_EQUAL_HEX16(0x0001, pvt.flags3);
	TEST_ASSERT_TRUE(pvt.invalid_llh);
}

/* The same vector through the streaming parser, so framing and decode agree. */
static void test_nav_pvt_through_parser(void)
{
	ubx_nav_pvt_t pvt;
	ubx_msg_t m;
	int len = ubx_frame(g_frame, sizeof(g_frame), UBX_CLASS_NAV,
			    UBX_ID_NAV_PVT, k_nav_pvt, sizeof(k_nav_pvt));

	TEST_ASSERT_EQUAL_INT((int)(UBX_LEN_NAV_PVT + UBX_FRAME_OVERHEAD), len);
	parser_fresh(UBX_PAYLOAD_CAP_DEFAULT);
	TEST_ASSERT_EQUAL_UINT(1U, feed_clean(g_frame, (size_t)len));
	TEST_ASSERT_EQUAL_INT(0, ubx_parser_msg(&g_parser, &m));
	TEST_ASSERT_EQUAL_INT(0, ubx_parse_nav_pvt(&m, &pvt));
	TEST_ASSERT_EQUAL_UINT32(259200000UL, pvt.itow_ms);
	TEST_ASSERT_EQUAL_UINT8(23U, pvt.num_sv);
}

static void test_nav_pvt_zero_flags(void)
{
	uint8_t payload[UBX_LEN_NAV_PVT];
	ubx_nav_pvt_t pvt;
	ubx_msg_t m;

	(void)memcpy(payload, k_nav_pvt, sizeof(payload));
	payload[11] = 0x08U; /* validMag only */
	payload[21] = 0x40U; /* carrSoln = 1, gnssFixOK clear */
	payload[78] = 0x00U;

	make_msg(&m, UBX_CLASS_NAV, UBX_ID_NAV_PVT, payload, UBX_LEN_NAV_PVT);
	TEST_ASSERT_EQUAL_INT(0, ubx_parse_nav_pvt(&m, &pvt));
	TEST_ASSERT_FALSE(pvt.valid_date);
	TEST_ASSERT_FALSE(pvt.valid_time);
	TEST_ASSERT_FALSE(pvt.fully_resolved);
	TEST_ASSERT_TRUE(pvt.valid_mag);
	TEST_ASSERT_FALSE(pvt.gnss_fix_ok);
	TEST_ASSERT_EQUAL_UINT8(1U, pvt.carr_soln);
	TEST_ASSERT_FALSE(pvt.invalid_llh);
}

static void test_nav_pvt_rejects(void)
{
	ubx_nav_pvt_t pvt;
	ubx_msg_t m;

	make_msg(&m, UBX_CLASS_NAV, UBX_ID_NAV_PVT, k_nav_pvt, UBX_LEN_NAV_PVT);
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_parse_nav_pvt(NULL, &pvt));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_parse_nav_pvt(&m, NULL));

	make_msg(&m, UBX_CLASS_NAV, UBX_ID_NAV_SAT, k_nav_pvt, UBX_LEN_NAV_PVT);
	TEST_ASSERT_EQUAL_INT(-ENOMSG, ubx_parse_nav_pvt(&m, &pvt));

	make_msg(&m, UBX_CLASS_NAV, UBX_ID_NAV_PVT, k_nav_pvt, 91U);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ubx_parse_nav_pvt(&m, &pvt));

	make_msg(&m, UBX_CLASS_NAV, UBX_ID_NAV_PVT, NULL, UBX_LEN_NAV_PVT);
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_parse_nav_pvt(&m, &pvt));
}

/* ----------------------------------------------------------- NAV-SAT ---- */

/*
 * 8-byte header + three 12-byte SV records. Record layout:
 *   0 gnssId  1 svId  2 cno  3 elev(I1)  4 azim(I2)  6 prRes(I2)  8 flags(X4)
 * flags: bits 0-2 qualityInd, bit 3 svUsed, bits 4-5 health.
 */
static const uint8_t k_nav_sat[8U + (3U * 12U)] = {
	/* iTOW */ 0x00, 0x14, 0x73, 0x0F,
	/* version 1, numSvs 3, reserved */ 0x01, 0x03, 0x00, 0x00,
	/* SV0: GPS-8, cno 45, elev -12, azim 310, prRes -25, q5 used h1 */
	0x00, 0x08, 0x2D, 0xF4, 0x36, 0x01, 0xE7, 0xFF, 0x1D, 0x00, 0x00, 0x00,
	/* SV1: GAL-27, cno 0, elev 5, azim 45, prRes 0, q4 unused h0 */
	0x02, 0x1B, 0x00, 0x05, 0x2D, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
	/* SV2: GLO-17, cno 51, elev 90, azim 359, prRes 100, q7 used h2 */
	0x06, 0x11, 0x33, 0x5A, 0x67, 0x01, 0x64, 0x00, 0x2F, 0x00, 0x00, 0x00,
};

static void test_nav_sat_iteration(void)
{
	ubx_nav_sat_iter_t it;
	ubx_nav_sat_sv_t sv;
	ubx_msg_t m;

	make_msg(&m, UBX_CLASS_NAV, UBX_ID_NAV_SAT, k_nav_sat, sizeof(k_nav_sat));
	TEST_ASSERT_EQUAL_INT(0, ubx_nav_sat_begin(&m, &it));
	TEST_ASSERT_EQUAL_UINT32(259200000UL, it.itow_ms);
	TEST_ASSERT_EQUAL_UINT8(1U, it.version);
	TEST_ASSERT_EQUAL_UINT8(3U, it.num_svs);

	TEST_ASSERT_EQUAL_INT(1, ubx_nav_sat_next(&it, &sv));
	TEST_ASSERT_EQUAL_UINT8(0U, sv.gnss_id);
	TEST_ASSERT_EQUAL_UINT8(8U, sv.sv_id);
	TEST_ASSERT_EQUAL_UINT8(45U, sv.cno_dbhz);
	TEST_ASSERT_EQUAL_INT8(-12, sv.elev_deg);
	TEST_ASSERT_EQUAL_INT16(310, sv.azim_deg);
	TEST_ASSERT_EQUAL_INT16(-25, sv.pr_res);
	TEST_ASSERT_EQUAL_UINT8(5U, sv.quality);
	TEST_ASSERT_TRUE(sv.used);
	TEST_ASSERT_EQUAL_UINT8(1U, sv.health);

	TEST_ASSERT_EQUAL_INT(1, ubx_nav_sat_next(&it, &sv));
	TEST_ASSERT_EQUAL_UINT8(2U, sv.gnss_id);
	TEST_ASSERT_EQUAL_UINT8(27U, sv.sv_id);
	TEST_ASSERT_EQUAL_UINT8(0U, sv.cno_dbhz);
	TEST_ASSERT_EQUAL_INT8(5, sv.elev_deg);
	TEST_ASSERT_EQUAL_INT16(45, sv.azim_deg);
	TEST_ASSERT_FALSE(sv.used);
	TEST_ASSERT_EQUAL_UINT8(0U, sv.health);

	TEST_ASSERT_EQUAL_INT(1, ubx_nav_sat_next(&it, &sv));
	TEST_ASSERT_EQUAL_UINT8(6U, sv.gnss_id);
	TEST_ASSERT_EQUAL_INT8(90, sv.elev_deg);
	TEST_ASSERT_EQUAL_INT16(359, sv.azim_deg);
	TEST_ASSERT_EQUAL_INT16(100, sv.pr_res);
	TEST_ASSERT_EQUAL_UINT8(7U, sv.quality);
	TEST_ASSERT_TRUE(sv.used);
	TEST_ASSERT_EQUAL_UINT8(2U, sv.health);

	TEST_ASSERT_EQUAL_INT(0, ubx_nav_sat_next(&it, &sv));
	TEST_ASSERT_EQUAL_INT(0, ubx_nav_sat_next(&it, &sv));
}

static void test_nav_sat_rejects(void)
{
	ubx_nav_sat_iter_t it;
	ubx_nav_sat_sv_t sv;
	ubx_msg_t m;

	make_msg(&m, UBX_CLASS_NAV, UBX_ID_NAV_SAT, k_nav_sat, sizeof(k_nav_sat));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_nav_sat_begin(NULL, &it));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_nav_sat_begin(&m, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_nav_sat_next(NULL, &sv));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_nav_sat_next(&it, NULL));

	make_msg(&m, UBX_CLASS_MON, UBX_ID_NAV_SAT, k_nav_sat, sizeof(k_nav_sat));
	TEST_ASSERT_EQUAL_INT(-ENOMSG, ubx_nav_sat_begin(&m, &it));

	/* Shorter than the header. */
	make_msg(&m, UBX_CLASS_NAV, UBX_ID_NAV_SAT, k_nav_sat, 4U);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ubx_nav_sat_begin(&m, &it));

	/* numSvs disagrees with the length. */
	make_msg(&m, UBX_CLASS_NAV, UBX_ID_NAV_SAT, k_nav_sat, 8U + 24U);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ubx_nav_sat_begin(&m, &it));

	/* Header-only frame with numSvs 0 is legal and simply empty. */
	{
		static const uint8_t empty[8] = {0, 0, 0, 0, 1, 0, 0, 0};

		make_msg(&m, UBX_CLASS_NAV, UBX_ID_NAV_SAT, empty, sizeof(empty));
		TEST_ASSERT_EQUAL_INT(0, ubx_nav_sat_begin(&m, &it));
		TEST_ASSERT_EQUAL_INT(0, ubx_nav_sat_next(&it, &sv));
	}
}

/* -------------------------------------------------------- NAV-TIMELS ---- */

static void test_nav_timels_vector(void)
{
	/*
	 *  0 iTOW  4 version  5..7 reserved  8 srcOfCurrLs  9 currLs
	 * 10 srcOfLsChange 11 lsChange 12 timeToLsEvent 16 wn 18 dn
	 * 20..22 reserved  23 valid
	 */
	static const uint8_t payload[UBX_LEN_NAV_TIMELS] = {
		0x00, 0x14, 0x73, 0x0F, /* iTOW */
		0x00,                   /* version */
		0xA5, 0xA5, 0xA5,       /* reserved0 */
		0x02,                   /* srcOfCurrLs = GPS */
		0x12,                   /* currLs = 18 */
		0x02,                   /* srcOfLsChange = GPS */
		0xFF,                   /* lsChange = -1 (deletion) */
		0x10, 0x0E, 0x00, 0x00, /* timeToLsEvent = 3600 */
		0xC4, 0x09,             /* dateOfLsGpsWn = 2500 */
		0x07, 0x00,             /* dateOfLsGpsDn = 7 */
		0xA5, 0xA5, 0xA5,       /* reserved1 */
		0x03,                   /* valid = currLs | timeToLsEvent */
	};
	ubx_nav_timels_t ls;
	ubx_msg_t m;

	make_msg(&m, UBX_CLASS_NAV, UBX_ID_NAV_TIMELS, payload, sizeof(payload));
	TEST_ASSERT_EQUAL_INT(0, ubx_parse_nav_timels(&m, &ls));
	TEST_ASSERT_EQUAL_UINT32(259200000UL, ls.itow_ms);
	TEST_ASSERT_EQUAL_UINT8(0U, ls.version);
	TEST_ASSERT_EQUAL_UINT8(2U, ls.src_of_curr_ls);
	TEST_ASSERT_EQUAL_INT8(18, ls.curr_ls);
	TEST_ASSERT_EQUAL_UINT8(2U, ls.src_of_ls_change);
	TEST_ASSERT_EQUAL_INT8(-1, ls.ls_change);
	TEST_ASSERT_EQUAL_INT32(3600, ls.time_to_ls_event_s);
	TEST_ASSERT_EQUAL_UINT16(2500U, ls.date_of_ls_gps_wn);
	TEST_ASSERT_EQUAL_UINT16(7U, ls.date_of_ls_gps_dn);
	TEST_ASSERT_TRUE(ls.valid_curr_ls);
	TEST_ASSERT_TRUE(ls.valid_time_to_ls_event);

	make_msg(&m, UBX_CLASS_NAV, UBX_ID_NAV_TIMELS, payload, 23U);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ubx_parse_nav_timels(&m, &ls));
}

/* ---------------------------------------------------------- NAV-SVIN ---- */

static void test_nav_svin_vector(void)
{
	/*
	 *  0 version 1..3 reserved  4 iTOW  8 dur 12 meanX 16 meanY 20 meanZ
	 * 24 meanXHP 25 meanYHP 26 meanZHP 27 reserved 28 meanAcc 32 obs
	 * 36 valid 37 active 38..39 reserved
	 */
	static const uint8_t payload[UBX_LEN_NAV_SVIN] = {
		0x00, 0xA5, 0xA5, 0xA5,
		0x00, 0x14, 0x73, 0x0F, /* iTOW */
		0x10, 0x0E, 0x00, 0x00, /* dur = 3600 s */
		0x60, 0x79, 0xFE, 0xFF, /* meanX = -100000 cm */
		0xA0, 0x86, 0x01, 0x00, /* meanY =  100000 cm */
		0x2C, 0x01, 0x00, 0x00, /* meanZ =     300 cm */
		0xF6,                   /* meanXHP = -10 */
		0x0A,                   /* meanYHP =  10 */
		0x00,                   /* meanZHP =   0 */
		0xA5,
		0xE8, 0x03, 0x00, 0x00, /* meanAcc = 1000 (0.1 mm) */
		0x39, 0x30, 0x00, 0x00, /* obs = 12345 */
		0x01,                   /* valid */
		0x00,                   /* active */
		0xA5, 0xA5,
	};
	ubx_nav_svin_t s;
	ubx_msg_t m;

	make_msg(&m, UBX_CLASS_NAV, UBX_ID_NAV_SVIN, payload, sizeof(payload));
	TEST_ASSERT_EQUAL_INT(0, ubx_parse_nav_svin(&m, &s));
	TEST_ASSERT_EQUAL_UINT8(0U, s.version);
	TEST_ASSERT_EQUAL_UINT32(259200000UL, s.itow_ms);
	TEST_ASSERT_EQUAL_UINT32(3600U, s.dur_s);
	TEST_ASSERT_EQUAL_INT32(-100000L, s.mean_x_cm);
	TEST_ASSERT_EQUAL_INT32(100000L, s.mean_y_cm);
	TEST_ASSERT_EQUAL_INT32(300L, s.mean_z_cm);
	TEST_ASSERT_EQUAL_INT8(-10, s.mean_x_hp);
	TEST_ASSERT_EQUAL_INT8(10, s.mean_y_hp);
	TEST_ASSERT_EQUAL_INT8(0, s.mean_z_hp);
	TEST_ASSERT_EQUAL_UINT32(1000U, s.mean_acc_0p1mm);
	TEST_ASSERT_EQUAL_UINT32(12345U, s.obs);
	TEST_ASSERT_TRUE(s.valid);
	TEST_ASSERT_FALSE(s.active);

	make_msg(&m, UBX_CLASS_NAV, UBX_ID_NAV_SVIN, payload, 39U);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ubx_parse_nav_svin(&m, &s));
}

/* ------------------------------------------------------------ TIM-TP ---- */

/*
 * qErr is the whole point of this message for the discipline loop, and it is
 * signed: a negative value that decodes as +4.29e9 would push the loop the
 * wrong way by nanoseconds every second. Both signs are asserted, plus the
 * qErrInvalid flag that says "do not apply this one at all".
 */
static void test_tim_tp_qerr_signs(void)
{
	static const uint8_t payload[UBX_LEN_TIM_TP] = {
		0x00, 0x14, 0x73, 0x0F, /*  0 towMS = 259200000 */
		0x00, 0x00, 0x00, 0x80, /*  4 towSubMS = 0x80000000 (half ms) */
		0x18, 0xFC, 0xFF, 0xFF, /*  8 qErr = -1000 ps */
		0xC4, 0x09,             /* 12 week = 2500 */
		0x0B,                   /* 14 flags: utc base, utc avail, raim=2 */
		0x51,                   /* 15 refInfo: gnss=1, utcStd=5 */
	};
	uint8_t buf[UBX_LEN_TIM_TP];
	ubx_tim_tp_t tp;
	ubx_msg_t m;

	make_msg(&m, UBX_CLASS_TIM, UBX_ID_TIM_TP, payload, sizeof(payload));
	TEST_ASSERT_EQUAL_INT(0, ubx_parse_tim_tp(&m, &tp));
	TEST_ASSERT_EQUAL_UINT32(259200000UL, tp.tow_ms);
	TEST_ASSERT_EQUAL_HEX32(0x80000000UL, tp.tow_sub_ms);
	TEST_ASSERT_EQUAL_INT32(-1000, tp.qerr_ps);
	TEST_ASSERT_EQUAL_UINT16(2500U, tp.week);
	TEST_ASSERT_TRUE(tp.time_base_utc);
	TEST_ASSERT_TRUE(tp.utc_available);
	TEST_ASSERT_EQUAL_UINT8(2U, tp.raim);
	TEST_ASSERT_FALSE(tp.qerr_invalid);
	TEST_ASSERT_EQUAL_UINT8(1U, tp.time_ref_gnss);
	TEST_ASSERT_EQUAL_UINT8(5U, tp.utc_standard);

	/* Positive qErr, GNSS timebase, qErrInvalid set. */
	(void)memcpy(buf, payload, sizeof(buf));
	buf[8] = 0xE8U;
	buf[9] = 0x03U;
	buf[10] = 0x00U;
	buf[11] = 0x00U; /* +1000 ps */
	buf[14] = 0x10U; /* qErrInvalid only */
	make_msg(&m, UBX_CLASS_TIM, UBX_ID_TIM_TP, buf, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(0, ubx_parse_tim_tp(&m, &tp));
	TEST_ASSERT_EQUAL_INT32(1000, tp.qerr_ps);
	TEST_ASSERT_FALSE(tp.time_base_utc);
	TEST_ASSERT_FALSE(tp.utc_available);
	TEST_ASSERT_EQUAL_UINT8(0U, tp.raim);
	TEST_ASSERT_TRUE(tp.qerr_invalid);

	/* The extremes of the signed range must survive the round trip. */
	buf[8] = 0x00U;
	buf[9] = 0x00U;
	buf[10] = 0x00U;
	buf[11] = 0x80U; /* INT32_MIN */
	make_msg(&m, UBX_CLASS_TIM, UBX_ID_TIM_TP, buf, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(0, ubx_parse_tim_tp(&m, &tp));
	TEST_ASSERT_EQUAL_INT32(INT32_MIN, tp.qerr_ps);

	buf[8] = 0xFFU;
	buf[9] = 0xFFU;
	buf[10] = 0xFFU;
	buf[11] = 0x7FU; /* INT32_MAX */
	make_msg(&m, UBX_CLASS_TIM, UBX_ID_TIM_TP, buf, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(0, ubx_parse_tim_tp(&m, &tp));
	TEST_ASSERT_EQUAL_INT32(INT32_MAX, tp.qerr_ps);

	make_msg(&m, UBX_CLASS_TIM, UBX_ID_TIM_TP, buf, 15U);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ubx_parse_tim_tp(&m, &tp));
	make_msg(&m, UBX_CLASS_TIM, 0x03U, buf, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(-ENOMSG, ubx_parse_tim_tp(&m, &tp));
}

/* ------------------------------------------------------------ MON-RF ---- */

static const uint8_t k_mon_rf[4U + (2U * 24U)] = {
	/* version 0, nBlocks 2, reserved */ 0x00, 0x02, 0x00, 0x00,
	/* block 0: id 0, jamming WARNING, antStatus OK, antPower ON */
	0x00, 0x02, 0x02, 0x01,
	0x00, 0x00, 0x00, 0x00,       /* postStatus */
	0xA5, 0xA5, 0xA5, 0xA5,       /* reserved1 */
	0x64, 0x00,                   /* noisePerMS = 100 */
	0x40, 0x1F,                   /* agcCnt = 8000 */
	0x0C,                         /* jamInd = 12 */
	0xFD,                         /* ofsI = -3 */
	0x82,                         /* magI = 130 */
	0x04,                         /* ofsQ = 4 */
	0x7D,                         /* magQ = 125 */
	0xA5, 0xA5, 0xA5,             /* reserved2 */
	/* block 1: id 1, jamming OK, antStatus SHORT, antPower ON */
	0x01, 0x01, 0x03, 0x01,
	0x78, 0x56, 0x34, 0x12,       /* postStatus */
	0xA5, 0xA5, 0xA5, 0xA5,
	0xC8, 0x00,                   /* noisePerMS = 200 */
	0x00, 0x10,                   /* agcCnt = 4096 */
	0xFF,                         /* jamInd = 255 */
	0x01, 0x40, 0xFE, 0x41,
	0xA5, 0xA5, 0xA5,
};

static void test_mon_rf_iteration(void)
{
	ubx_mon_rf_iter_t it;
	ubx_mon_rf_block_t blk;
	ubx_msg_t m;

	make_msg(&m, UBX_CLASS_MON, UBX_ID_MON_RF, k_mon_rf, sizeof(k_mon_rf));
	TEST_ASSERT_EQUAL_INT(0, ubx_mon_rf_begin(&m, &it));
	TEST_ASSERT_EQUAL_UINT8(0U, it.version);
	TEST_ASSERT_EQUAL_UINT8(2U, it.n_blocks);

	TEST_ASSERT_EQUAL_INT(1, ubx_mon_rf_next(&it, &blk));
	TEST_ASSERT_EQUAL_UINT8(0U, blk.block_id);
	TEST_ASSERT_EQUAL_UINT8(UBX_JAMMING_WARNING, blk.jamming_state);
	TEST_ASSERT_EQUAL_UINT8(UBX_ANT_STATUS_OK, blk.ant_status);
	TEST_ASSERT_EQUAL_UINT8(UBX_ANT_POWER_ON, blk.ant_power);
	TEST_ASSERT_EQUAL_UINT32(0U, blk.post_status);
	TEST_ASSERT_EQUAL_UINT16(100U, blk.noise_per_ms);
	TEST_ASSERT_EQUAL_UINT16(8000U, blk.agc_cnt);
	TEST_ASSERT_EQUAL_UINT8(12U, blk.jam_ind);
	TEST_ASSERT_EQUAL_INT8(-3, blk.ofs_i);
	TEST_ASSERT_EQUAL_UINT8(130U, blk.mag_i);
	TEST_ASSERT_EQUAL_INT8(4, blk.ofs_q);
	TEST_ASSERT_EQUAL_UINT8(125U, blk.mag_q);

	TEST_ASSERT_EQUAL_INT(1, ubx_mon_rf_next(&it, &blk));
	TEST_ASSERT_EQUAL_UINT8(1U, blk.block_id);
	TEST_ASSERT_EQUAL_UINT8(UBX_JAMMING_OK, blk.jamming_state);
	TEST_ASSERT_EQUAL_UINT8(UBX_ANT_STATUS_SHORT, blk.ant_status);
	TEST_ASSERT_EQUAL_HEX32(0x12345678UL, blk.post_status);
	TEST_ASSERT_EQUAL_UINT16(4096U, blk.agc_cnt);
	TEST_ASSERT_EQUAL_UINT8(255U, blk.jam_ind);
	TEST_ASSERT_EQUAL_INT8(-2, blk.ofs_q);

	TEST_ASSERT_EQUAL_INT(0, ubx_mon_rf_next(&it, &blk));
}

static void test_mon_rf_rejects(void)
{
	ubx_mon_rf_iter_t it;
	ubx_mon_rf_block_t blk;
	ubx_msg_t m;

	make_msg(&m, UBX_CLASS_MON, UBX_ID_MON_RF, k_mon_rf, sizeof(k_mon_rf));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_mon_rf_begin(NULL, &it));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_mon_rf_begin(&m, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_mon_rf_next(NULL, &blk));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_mon_rf_next(&it, NULL));

	make_msg(&m, UBX_CLASS_NAV, UBX_ID_MON_RF, k_mon_rf, sizeof(k_mon_rf));
	TEST_ASSERT_EQUAL_INT(-ENOMSG, ubx_mon_rf_begin(&m, &it));

	make_msg(&m, UBX_CLASS_MON, UBX_ID_MON_RF, k_mon_rf, 2U);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ubx_mon_rf_begin(&m, &it));

	make_msg(&m, UBX_CLASS_MON, UBX_ID_MON_RF, k_mon_rf, 4U + 24U);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ubx_mon_rf_begin(&m, &it));

	make_msg(&m, UBX_CLASS_MON, UBX_ID_MON_RF, NULL, sizeof(k_mon_rf));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_mon_rf_begin(&m, &it));
}

/* --------------------------------------------------------------- ACK ---- */

/*
 * ACK-ACK for CFG-VALSET, hand-derived:
 *   body 05 01 02 00 06 8A
 *   A:   05 06 08 08 0E 98
 *   B:   05 0B 13 1B 29 C1
 *   -> B5 62 05 01 02 00 06 8A 98 C1
 */
static void test_ack_vector(void)
{
	static const uint8_t frame[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00,
					0x06, 0x8A, 0x98, 0xC1};
	static const uint8_t nak_payload[] = {UBX_CLASS_CFG, UBX_ID_CFG_VALSET};
	ubx_msg_t m;
	ubx_ack_t a;
	uint8_t built[16];

	TEST_ASSERT_EQUAL_INT(10, ubx_frame(built, sizeof(built), UBX_CLASS_ACK,
					    UBX_ID_ACK_ACK, nak_payload,
					    sizeof(nak_payload)));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(frame, built, sizeof(frame));

	parser_fresh(UBX_PAYLOAD_CAP_DEFAULT);
	TEST_ASSERT_EQUAL_UINT(1U, feed_clean(frame, sizeof(frame)));
	TEST_ASSERT_EQUAL_INT(0, ubx_parser_msg(&g_parser, &m));
	TEST_ASSERT_EQUAL_INT(0, ubx_parse_ack(&m, &a));
	TEST_ASSERT_TRUE(a.ack);
	TEST_ASSERT_EQUAL_HEX8(UBX_CLASS_CFG, a.cls_id);
	TEST_ASSERT_EQUAL_HEX8(UBX_ID_CFG_VALSET, a.msg_id);

	make_msg(&m, UBX_CLASS_ACK, UBX_ID_ACK_NAK, nak_payload, 2U);
	TEST_ASSERT_EQUAL_INT(0, ubx_parse_ack(&m, &a));
	TEST_ASSERT_FALSE(a.ack);

	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_parse_ack(NULL, &a));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_parse_ack(&m, NULL));
	make_msg(&m, UBX_CLASS_ACK, 0x07U, nak_payload, 2U);
	TEST_ASSERT_EQUAL_INT(-ENOMSG, ubx_parse_ack(&m, &a));
	make_msg(&m, UBX_CLASS_NAV, UBX_ID_ACK_ACK, nak_payload, 2U);
	TEST_ASSERT_EQUAL_INT(-ENOMSG, ubx_parse_ack(&m, &a));
	make_msg(&m, UBX_CLASS_ACK, UBX_ID_ACK_ACK, nak_payload, 1U);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ubx_parse_ack(&m, &a));
	make_msg(&m, UBX_CLASS_ACK, UBX_ID_ACK_ACK, NULL, 2U);
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_parse_ack(&m, &a));
}

/* ------------------------------------------------- resync exhaustively -- */

/*
 * Truncate a NAV-PVT frame at every possible point, let the line go quiet, then
 * send a good frame. Whatever state the parser was left in — mid-header,
 * mid-payload, waiting on a checksum byte — the quiet period must return it to
 * hunting and the following frame must arrive intact, exactly once.
 *
 * The 128 zero bytes stand in for the gap between UART bursts. They are more
 * than the 94 bytes any pending NAV-PVT state can still be waiting for, so the
 * parser is always back to hunting before the good frame starts.
 *
 * The count is of ACK-ACK frames, not of frames, because a run of zeros behind
 * a dangling sync pair is itself a legal UBX frame: truncating at t = 2 leaves
 * "B5 62" pending, and "00 00 00 00 00 00" is a well-formed zero-length class-0
 * message whose Fletcher checksum is 00 00. That is the protocol, not a defect
 * — UBX defines no class 0, so gnssmgr ignores it — and a test that counted all
 * frames would be asserting something untrue about UBX.
 */
static void test_parser_resync_from_every_truncation(void)
{
	static const uint8_t good[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00,
				       0x06, 0x8A, 0x98, 0xC1};
	uint8_t quiet[128];
	int flen = ubx_frame(g_frame, sizeof(g_frame), UBX_CLASS_NAV,
			     UBX_ID_NAV_PVT, k_nav_pvt, sizeof(k_nav_pvt));
	size_t t;

	TEST_ASSERT_TRUE(flen > 0);
	(void)memset(quiet, 0, sizeof(quiet));

	for (t = 0U; t < (size_t)flen; t++) {
		unsigned int acks = 0U;
		ubx_msg_t m;
		char msg[64];

		parser_fresh(UBX_PAYLOAD_CAP_DEFAULT);
		acks += feed_count(g_frame, t, UBX_CLASS_ACK, UBX_ID_ACK_ACK);
		acks += feed_count(quiet, sizeof(quiet), UBX_CLASS_ACK,
				   UBX_ID_ACK_ACK);
		acks += feed_count(good, sizeof(good), UBX_CLASS_ACK,
				   UBX_ID_ACK_ACK);

		(void)snprintf(msg, sizeof(msg), "truncation at %u",
			       (unsigned int)t);
		TEST_ASSERT_EQUAL_UINT_MESSAGE(1U, acks, msg);
		/* The last thing parsed is the good frame, intact. */
		TEST_ASSERT_EQUAL_INT(0, ubx_parser_msg(&g_parser, &m));
		TEST_ASSERT_EQUAL_HEX8_MESSAGE(UBX_CLASS_ACK, m.cls, msg);
		TEST_ASSERT_EQUAL_HEX8_MESSAGE(UBX_ID_ACK_ACK, m.id, msg);
		TEST_ASSERT_EQUAL_UINT16(2U, m.len);
		TEST_ASSERT_EQUAL_HEX8(UBX_CLASS_CFG, m.payload[0]);
		TEST_ASSERT_EQUAL_HEX8(UBX_ID_CFG_VALSET, m.payload[1]);
	}
}

/*
 * The dangling-sync-pair case above, isolated and asserted for what it is: the
 * parser must accept "B5 62 00 00 00 00 00 00" as a valid class-0 frame rather
 * than special-casing it, because nothing in the framing says otherwise.
 */
static void test_parser_all_zero_frame_is_well_formed(void)
{
	static const uint8_t zeros[] = {0xB5, 0x62, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00};
	ubx_msg_t m;

	parser_fresh(UBX_PAYLOAD_CAP_DEFAULT);
	TEST_ASSERT_EQUAL_UINT(1U, feed_clean(zeros, sizeof(zeros)));
	TEST_ASSERT_EQUAL_INT(0, ubx_parser_msg(&g_parser, &m));
	TEST_ASSERT_EQUAL_HEX8(0x00, m.cls);
	TEST_ASSERT_EQUAL_HEX8(0x00, m.id);
	TEST_ASSERT_EQUAL_UINT16(0U, m.len);
}

/*
 * Split a clean two-frame stream at every byte boundary and feed it as two
 * calls. A byte-at-a-time parser should not care, and this proves it carries no
 * state in the call, only in the context.
 */
static void test_parser_split_at_every_boundary(void)
{
	static const uint8_t good[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00,
				       0x06, 0x8A, 0x98, 0xC1};
	uint8_t stream[2U * sizeof(good)];
	size_t s;

	(void)memcpy(stream, good, sizeof(good));
	(void)memcpy(&stream[sizeof(good)], good, sizeof(good));

	for (s = 0U; s <= sizeof(stream); s++) {
		unsigned int frames = 0U;

		parser_fresh(UBX_PAYLOAD_CAP_DEFAULT);
		frames += feed_clean(stream, s);
		frames += feed_clean(&stream[s], sizeof(stream) - s);
		TEST_ASSERT_EQUAL_UINT(2U, frames);
	}
}

/* Garbage of every byte value in front of a frame must not defeat it. */
static void test_parser_leading_garbage(void)
{
	static const uint8_t good[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00,
				       0x06, 0x8A, 0x98, 0xC1};
	unsigned int v;

	for (v = 0U; v < 256U; v++) {
		uint8_t junk = (uint8_t)v;
		unsigned int frames;

		parser_fresh(UBX_PAYLOAD_CAP_DEFAULT);
		frames = feed_any(&junk, 1U);
		frames += feed_any(good, sizeof(good));
		TEST_ASSERT_EQUAL_UINT(1U, frames);
	}
}

/* --------------------------------------------------------- CFG-VALSET -- */

static void test_cfg_key_decoders(void)
{
	TEST_ASSERT_EQUAL_INT(1, ubx_cfg_key_size_id(UBX_CFG_TP_TP1_ENA));
	TEST_ASSERT_EQUAL_INT(1, ubx_cfg_key_bytes(UBX_CFG_TP_TP1_ENA));
	TEST_ASSERT_EQUAL_INT(2, ubx_cfg_key_size_id(UBX_CFG_MSGOUT_NAV_PVT_UART1));
	TEST_ASSERT_EQUAL_INT(1, ubx_cfg_key_bytes(UBX_CFG_MSGOUT_NAV_PVT_UART1));
	TEST_ASSERT_EQUAL_INT(3, ubx_cfg_key_size_id(UBX_CFG_RATE_MEAS));
	TEST_ASSERT_EQUAL_INT(2, ubx_cfg_key_bytes(UBX_CFG_RATE_MEAS));
	TEST_ASSERT_EQUAL_INT(4, ubx_cfg_key_size_id(UBX_CFG_UART1_BAUDRATE));
	TEST_ASSERT_EQUAL_INT(4, ubx_cfg_key_bytes(UBX_CFG_UART1_BAUDRATE));

	TEST_ASSERT_EQUAL_HEX16(0x052, ubx_cfg_key_group(UBX_CFG_UART1_BAUDRATE));
	TEST_ASSERT_EQUAL_HEX16(0x074, ubx_cfg_key_group(UBX_CFG_UART1OUTPROT_NMEA));

	/* Size fields 0, 6 and 7 are not defined storage sizes. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_cfg_key_size_id(0x00520001UL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_cfg_key_bytes(0x60520001UL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_cfg_key_bytes(0x70520001UL));
	/* Eight-byte keys decode even though the board uses none. */
	TEST_ASSERT_EQUAL_INT(8, ubx_cfg_key_bytes(0x50520001UL));
}

/*
 * Every configuration key belongs to a known group and declares a plausible
 * width. This does not prove an ID is the one u-blox published — only the ICD
 * can — but it does catch the transposition that turns a valid ID into a
 * differently-valid-looking one, which is the realistic transcription failure.
 */
static void test_cfg_key_table_encoding(void)
{
	typedef struct {
		uint32_t key;
		uint16_t group;
		int size_id;
		const char *name;
	} key_row_t;
	static const key_row_t rows[] = {
		{UBX_CFG_UART1_BAUDRATE, 0x052, 4, "UART1-BAUDRATE"},
		{UBX_CFG_UART1_ENABLED, 0x052, 1, "UART1-ENABLED"},
		{UBX_CFG_UART1INPROT_UBX, 0x073, 1, "UART1INPROT-UBX"},
		{UBX_CFG_UART1INPROT_NMEA, 0x073, 1, "UART1INPROT-NMEA"},
		{UBX_CFG_UART1INPROT_RTCM3X, 0x073, 1, "UART1INPROT-RTCM3X"},
		{UBX_CFG_UART1OUTPROT_UBX, 0x074, 1, "UART1OUTPROT-UBX"},
		{UBX_CFG_UART1OUTPROT_NMEA, 0x074, 1, "UART1OUTPROT-NMEA"},
		{UBX_CFG_UART1OUTPROT_RTCM3X, 0x074, 1, "UART1OUTPROT-RTCM3X"},
		{UBX_CFG_MSGOUT_NAV_PVT_UART1, 0x091, 2, "MSGOUT-NAV_PVT"},
		{UBX_CFG_MSGOUT_NAV_SAT_UART1, 0x091, 2, "MSGOUT-NAV_SAT"},
		{UBX_CFG_MSGOUT_NAV_TIMELS_UART1, 0x091, 2, "MSGOUT-NAV_TIMELS"},
		{UBX_CFG_MSGOUT_NAV_SVIN_UART1, 0x091, 2, "MSGOUT-NAV_SVIN"},
		{UBX_CFG_MSGOUT_NAV_SIG_UART1, 0x091, 2, "MSGOUT-NAV_SIG"},
		{UBX_CFG_MSGOUT_TIM_TP_UART1, 0x091, 2, "MSGOUT-TIM_TP"},
		{UBX_CFG_MSGOUT_MON_RF_UART1, 0x091, 2, "MSGOUT-MON_RF"},
		{UBX_CFG_MSGOUT_MON_HW_UART1, 0x091, 2, "MSGOUT-MON_HW"},
		{UBX_CFG_RATE_MEAS, 0x021, 3, "RATE-MEAS"},
		{UBX_CFG_RATE_NAV, 0x021, 3, "RATE-NAV"},
		{UBX_CFG_RATE_TIMEREF, 0x021, 2, "RATE-TIMEREF"},
		{UBX_CFG_NAVSPG_DYNMODEL, 0x011, 2, "NAVSPG-DYNMODEL"},
		{UBX_CFG_NAVSPG_INFIL_MINELEV, 0x011, 2, "NAVSPG-MINELEV"},
		{UBX_CFG_SIGNAL_GPS_ENA, 0x031, 1, "SIGNAL-GPS"},
		{UBX_CFG_SIGNAL_SBAS_ENA, 0x031, 1, "SIGNAL-SBAS"},
		{UBX_CFG_SIGNAL_GAL_ENA, 0x031, 1, "SIGNAL-GAL"},
		{UBX_CFG_SIGNAL_BDS_ENA, 0x031, 1, "SIGNAL-BDS"},
		{UBX_CFG_SIGNAL_QZSS_ENA, 0x031, 1, "SIGNAL-QZSS"},
		{UBX_CFG_SIGNAL_GLO_ENA, 0x031, 1, "SIGNAL-GLO"},
		{UBX_CFG_TP_PULSE_DEF, 0x005, 2, "TP-PULSE_DEF"},
		{UBX_CFG_TP_PULSE_LENGTH_DEF, 0x005, 2, "TP-PULSE_LENGTH_DEF"},
		{UBX_CFG_TP_ANT_CABLEDELAY, 0x005, 3, "TP-ANT_CABLEDELAY"},
		{UBX_CFG_TP_PERIOD_TP1, 0x005, 4, "TP-PERIOD_TP1"},
		{UBX_CFG_TP_PERIOD_LOCK_TP1, 0x005, 4, "TP-PERIOD_LOCK_TP1"},
		{UBX_CFG_TP_LEN_TP1, 0x005, 4, "TP-LEN_TP1"},
		{UBX_CFG_TP_LEN_LOCK_TP1, 0x005, 4, "TP-LEN_LOCK_TP1"},
		{UBX_CFG_TP_USER_DELAY_TP1, 0x005, 4, "TP-USER_DELAY_TP1"},
		{UBX_CFG_TP_TP1_ENA, 0x005, 1, "TP-TP1_ENA"},
		{UBX_CFG_TP_SYNC_GNSS_TP1, 0x005, 1, "TP-SYNC_GNSS_TP1"},
		{UBX_CFG_TP_USE_LOCKED_TP1, 0x005, 1, "TP-USE_LOCKED_TP1"},
		{UBX_CFG_TP_ALIGN_TO_TOW_TP1, 0x005, 1, "TP-ALIGN_TO_TOW_TP1"},
		{UBX_CFG_TP_POL_TP1, 0x005, 1, "TP-POL_TP1"},
		{UBX_CFG_TP_TIMEGRID_TP1, 0x005, 2, "TP-TIMEGRID_TP1"},
		{UBX_CFG_TP_PERIOD_TP2, 0x005, 4, "TP-PERIOD_TP2"},
		{UBX_CFG_TP_PERIOD_LOCK_TP2, 0x005, 4, "TP-PERIOD_LOCK_TP2"},
		{UBX_CFG_TP_LEN_TP2, 0x005, 4, "TP-LEN_TP2"},
		{UBX_CFG_TP_LEN_LOCK_TP2, 0x005, 4, "TP-LEN_LOCK_TP2"},
		{UBX_CFG_TP_USER_DELAY_TP2, 0x005, 4, "TP-USER_DELAY_TP2"},
		{UBX_CFG_TP_TP2_ENA, 0x005, 1, "TP-TP2_ENA"},
		{UBX_CFG_TP_SYNC_GNSS_TP2, 0x005, 1, "TP-SYNC_GNSS_TP2"},
		{UBX_CFG_TP_USE_LOCKED_TP2, 0x005, 1, "TP-USE_LOCKED_TP2"},
		{UBX_CFG_TP_ALIGN_TO_TOW_TP2, 0x005, 1, "TP-ALIGN_TO_TOW_TP2"},
		{UBX_CFG_TP_POL_TP2, 0x005, 1, "TP-POL_TP2"},
		{UBX_CFG_TP_TIMEGRID_TP2, 0x005, 2, "TP-TIMEGRID_TP2"},
		{UBX_CFG_TXREADY_ENABLED, 0x0A2, 1, "TXREADY-ENABLED"},
		{UBX_CFG_TXREADY_POLARITY, 0x0A2, 1, "TXREADY-POLARITY"},
		{UBX_CFG_TXREADY_PIN, 0x0A2, 2, "TXREADY-PIN"},
		{UBX_CFG_TXREADY_THRESHOLD, 0x0A2, 3, "TXREADY-THRESHOLD"},
		{UBX_CFG_TXREADY_INTERFACE, 0x0A2, 2, "TXREADY-INTERFACE"},
		{UBX_CFG_TMODE_MODE, 0x003, 2, "TMODE-MODE"},
		{UBX_CFG_TMODE_POS_TYPE, 0x003, 2, "TMODE-POS_TYPE"},
		{UBX_CFG_TMODE_ECEF_X, 0x003, 4, "TMODE-ECEF_X"},
		{UBX_CFG_TMODE_ECEF_Y, 0x003, 4, "TMODE-ECEF_Y"},
		{UBX_CFG_TMODE_ECEF_Z, 0x003, 4, "TMODE-ECEF_Z"},
		{UBX_CFG_TMODE_ECEF_X_HP, 0x003, 2, "TMODE-ECEF_X_HP"},
		{UBX_CFG_TMODE_ECEF_Y_HP, 0x003, 2, "TMODE-ECEF_Y_HP"},
		{UBX_CFG_TMODE_ECEF_Z_HP, 0x003, 2, "TMODE-ECEF_Z_HP"},
		{UBX_CFG_TMODE_FIXED_POS_ACC, 0x003, 4, "TMODE-FIXED_POS_ACC"},
		{UBX_CFG_TMODE_SVIN_MIN_DUR, 0x003, 4, "TMODE-SVIN_MIN_DUR"},
		{UBX_CFG_TMODE_SVIN_ACC_LIMIT, 0x003, 4, "TMODE-SVIN_ACC_LIMIT"},
	};
	size_t i;

	for (i = 0U; i < ARRAY_LEN(rows); i++) {
		TEST_ASSERT_EQUAL_HEX16_MESSAGE(rows[i].group,
						ubx_cfg_key_group(rows[i].key),
						rows[i].name);
		TEST_ASSERT_EQUAL_INT_MESSAGE(rows[i].size_id,
					      ubx_cfg_key_size_id(rows[i].key),
					      rows[i].name);
		/* Bit 31 is reserved and must be zero in every key. */
		TEST_ASSERT_EQUAL_HEX32_MESSAGE(0U, rows[i].key & 0x80000000UL,
						rows[i].name);
	}

	/* No two keys may collide. */
	for (i = 0U; i < ARRAY_LEN(rows); i++) {
		size_t j;

		for (j = i + 1U; j < ARRAY_LEN(rows); j++) {
			TEST_ASSERT_TRUE_MESSAGE(rows[i].key != rows[j].key,
						 rows[i].name);
		}
	}
}

/*
 * Hand-computed VALSET vector 1: layers RAM|BBR|FLASH, one L key set false.
 *
 *   payload  00 07 00 00 | 02 00 74 10 | 00                (len 9)
 *   body     06 8A 09 00 00 07 00 00 02 00 74 10 00
 *   A        06 90 99 99 99 A0 A0 A0 A2 A2 16 26 26
 *   B        06 96 2F C8 61 01 A1 41 E3 85 9B C1 E7
 *   -> B5 62 06 8A 09 00 00 07 00 00 02 00 74 10 00 26 E7
 */
static void test_valset_wire_bool(void)
{
	static const uint8_t expect[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00,
					 0x00, 0x07, 0x00, 0x00,
					 0x02, 0x00, 0x74, 0x10, 0x00,
					 0x26, 0xE7};
	ubx_valset_t v;
	uint8_t buf[64];

	TEST_ASSERT_EQUAL_INT(0, ubx_valset_begin(&v, buf, sizeof(buf),
						  UBX_CFG_LAYER_ALL));
	TEST_ASSERT_EQUAL_INT(0, ubx_valset_add_bool(&v, UBX_CFG_UART1OUTPROT_NMEA,
						     false));
	TEST_ASSERT_EQUAL_INT((int)sizeof(expect), ubx_valset_end(&v));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, buf, sizeof(expect));
	TEST_ASSERT_EQUAL_UINT16(1U, v.items);
}

/*
 * Hand-computed VALSET vector 2: one U4 key, CFG-TMODE-SVIN_MIN_DUR = 3600.
 *
 *   payload  00 07 00 00 | 10 00 03 40 | 10 0E 00 00       (len 12)
 *   body     06 8A 0C 00 00 07 00 00 10 00 03 40 10 0E 00 00
 *   A        06 90 9C 9C 9C A3 A3 A3 B3 B3 B6 F6 06 14 14 14
 *   B        06 96 32 CE 6A 0D B0 53 06 B9 6F 65 6B 7F 93 A7
 *   -> ... 14 A7
 */
static void test_valset_wire_u4(void)
{
	static const uint8_t expect[] = {0xB5, 0x62, 0x06, 0x8A, 0x0C, 0x00,
					 0x00, 0x07, 0x00, 0x00,
					 0x10, 0x00, 0x03, 0x40,
					 0x10, 0x0E, 0x00, 0x00,
					 0x14, 0xA7};
	ubx_valset_t v;
	uint8_t buf[64];

	TEST_ASSERT_EQUAL_INT(0, ubx_valset_begin(&v, buf, sizeof(buf),
						  UBX_CFG_LAYER_ALL));
	TEST_ASSERT_EQUAL_INT(0, ubx_valset_add_u4(&v, UBX_CFG_TMODE_SVIN_MIN_DUR,
						   3600U));
	TEST_ASSERT_EQUAL_INT((int)sizeof(expect), ubx_valset_end(&v));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, buf, sizeof(expect));
}

/* Every setter width, and the parser's view of the result. */
static void test_valset_all_widths(void)
{
	ubx_valset_t v;
	uint8_t buf[128];
	ubx_msg_t m;
	int len;

	TEST_ASSERT_EQUAL_INT(0, ubx_valset_begin(&v, buf, sizeof(buf),
						  UBX_CFG_LAYER_RAM));
	TEST_ASSERT_EQUAL_INT(0, ubx_valset_add_u1(&v, UBX_CFG_TMODE_MODE, 2U));
	TEST_ASSERT_EQUAL_INT(0, ubx_valset_add_u2(&v, UBX_CFG_RATE_MEAS, 1000U));
	TEST_ASSERT_EQUAL_INT(0, ubx_valset_add_u4(&v, UBX_CFG_UART1_BAUDRATE,
						   460800U));
	TEST_ASSERT_EQUAL_INT(0, ubx_valset_add_u8(&v, 0x50520099UL,
						   0x0123456789ABCDEFULL));
	TEST_ASSERT_EQUAL_INT(0, ubx_valset_add_i1(&v, UBX_CFG_NAVSPG_INFIL_MINELEV,
						   -5));
	TEST_ASSERT_EQUAL_INT(0, ubx_valset_add_i2(&v, UBX_CFG_TP_ANT_CABLEDELAY,
						   -300));
	TEST_ASSERT_EQUAL_INT(0, ubx_valset_add_i4(&v, UBX_CFG_TMODE_ECEF_X,
						   -100000L));
	TEST_ASSERT_EQUAL_INT(0, ubx_valset_add_bool(&v, UBX_CFG_TP_TP1_ENA, true));
	len = ubx_valset_end(&v);
	TEST_ASSERT_TRUE(len > 0);
	TEST_ASSERT_EQUAL_UINT16(8U, v.items);

	parser_fresh(UBX_PAYLOAD_CAP_DEFAULT);
	TEST_ASSERT_EQUAL_UINT(1U, feed_clean(buf, (size_t)len));
	TEST_ASSERT_EQUAL_INT(0, ubx_parser_msg(&g_parser, &m));
	TEST_ASSERT_EQUAL_HEX8(UBX_CLASS_CFG, m.cls);
	TEST_ASSERT_EQUAL_HEX8(UBX_ID_CFG_VALSET, m.id);
	TEST_ASSERT_EQUAL_HEX8(0x00, m.payload[0]);            /* version */
	TEST_ASSERT_EQUAL_HEX8(UBX_CFG_LAYER_RAM, m.payload[1]);

	/* i1 / i2 / i4 must be two's complement little-endian on the wire.
	 * u1(1) u2(2) u4(4) u8(8) precede them: 4 + 5 + 6 + 8 + 12 = 35. */
	TEST_ASSERT_EQUAL_HEX8(0xFB, m.payload[35U + 4U]);      /* -5 */
	TEST_ASSERT_EQUAL_HEX8(0xD4, m.payload[40U + 4U]);      /* -300 lo */
	TEST_ASSERT_EQUAL_HEX8(0xFE, m.payload[40U + 5U]);      /* -300 hi */
	TEST_ASSERT_EQUAL_HEX8(0x60, m.payload[46U + 4U]);      /* -100000 */
	TEST_ASSERT_EQUAL_HEX8(0x79, m.payload[46U + 5U]);
	TEST_ASSERT_EQUAL_HEX8(0xFE, m.payload[46U + 6U]);
	TEST_ASSERT_EQUAL_HEX8(0xFF, m.payload[46U + 7U]);
	/* u8, little-endian. */
	TEST_ASSERT_EQUAL_HEX8(0xEF, m.payload[23U + 4U]);
	TEST_ASSERT_EQUAL_HEX8(0x01, m.payload[23U + 11U]);
}

static void test_valset_rejects(void)
{
	ubx_valset_t v;
	uint8_t buf[64];

	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_valset_begin(NULL, buf, sizeof(buf), 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_valset_begin(&v, NULL, sizeof(buf), 1U));
	/* Layers must name at least one real layer. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_valset_begin(&v, buf, sizeof(buf), 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_valset_begin(&v, buf, sizeof(buf), 0xF8U));
	/* Too small for even an empty frame. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ubx_valset_begin(&v, buf, 11U, 1U));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ubx_valset_end(&v));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_valset_add_u1(NULL, 0x20030001UL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_valset_end(NULL));

	/* end() on a builder that was never begun. */
	(void)memset(&v, 0, sizeof(v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_valset_end(&v));

	/* An empty VALSET is legal, if pointless. */
	TEST_ASSERT_EQUAL_INT(0, ubx_valset_begin(&v, buf, sizeof(buf), 1U));
	TEST_ASSERT_EQUAL_INT(12, ubx_valset_end(&v));
	/* ...but it is closed afterwards. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_valset_add_u1(&v, UBX_CFG_TMODE_MODE, 1U));
}

/* A key/setter width disagreement is a caught bug, not a malformed frame. */
static void test_valset_width_mismatch_latches(void)
{
	ubx_valset_t v;
	uint8_t buf[64];

	TEST_ASSERT_EQUAL_INT(0, ubx_valset_begin(&v, buf, sizeof(buf), 1U));
	/* CFG-RATE-MEAS is two bytes wide. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_valset_add_u4(&v, UBX_CFG_RATE_MEAS, 1U));
	/* The failure is latched: later adds are refused with the same code... */
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_valset_add_u2(&v, UBX_CFG_RATE_MEAS, 1U));
	/* ...and end() reports it instead of a length. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_valset_end(&v));

	/* add_bool insists on a one-bit (L) key, not merely a one-byte one. */
	TEST_ASSERT_EQUAL_INT(0, ubx_valset_begin(&v, buf, sizeof(buf), 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_valset_add_bool(&v, UBX_CFG_TMODE_MODE,
							   true));

	/* An undecodable key is refused too. */
	TEST_ASSERT_EQUAL_INT(0, ubx_valset_begin(&v, buf, sizeof(buf), 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_valset_add_u1(&v, 0x00030001UL, 1U));
}

static void test_valset_out_of_space(void)
{
	ubx_valset_t v;
	uint8_t buf[16]; /* header 10 + checksum 2 leaves room for nothing */

	TEST_ASSERT_EQUAL_INT(0, ubx_valset_begin(&v, buf, sizeof(buf), 1U));
	/* A 4-byte key + 1-byte value needs 5, and only 4 remain. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ubx_valset_add_u1(&v, UBX_CFG_TMODE_MODE, 1U));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ubx_valset_end(&v));

	/* One byte more and it fits exactly. */
	TEST_ASSERT_EQUAL_INT(0, ubx_valset_begin(&v, buf, 17U, 1U));
	TEST_ASSERT_EQUAL_INT(0, ubx_valset_add_u1(&v, UBX_CFG_TMODE_MODE, 1U));
	TEST_ASSERT_EQUAL_INT(17, ubx_valset_end(&v));
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_checksum_cfg_prt_poll);
	RUN_TEST(test_checksum_mon_ver_poll);
	RUN_TEST(test_checksum_null_tolerant);
	RUN_TEST(test_frame_known_polls);
	RUN_TEST(test_frame_with_payload_cfg_rst);
	RUN_TEST(test_frame_argument_errors);

	RUN_TEST(test_parser_init_errors);
	RUN_TEST(test_parser_zero_length_frame);
	RUN_TEST(test_parser_resync_double_sync1);
	RUN_TEST(test_parser_resync_from_checksum_error);
	RUN_TEST(test_parser_resync_from_length_error);
	RUN_TEST(test_parser_sync_pattern_inside_payload);
	RUN_TEST(test_parser_resync_from_every_truncation);
	RUN_TEST(test_parser_all_zero_frame_is_well_formed);
	RUN_TEST(test_parser_split_at_every_boundary);
	RUN_TEST(test_parser_leading_garbage);

	RUN_TEST(test_nav_pvt_full_vector);
	RUN_TEST(test_nav_pvt_through_parser);
	RUN_TEST(test_nav_pvt_zero_flags);
	RUN_TEST(test_nav_pvt_rejects);
	RUN_TEST(test_nav_sat_iteration);
	RUN_TEST(test_nav_sat_rejects);
	RUN_TEST(test_nav_timels_vector);
	RUN_TEST(test_nav_svin_vector);
	RUN_TEST(test_tim_tp_qerr_signs);
	RUN_TEST(test_mon_rf_iteration);
	RUN_TEST(test_mon_rf_rejects);
	RUN_TEST(test_ack_vector);

	RUN_TEST(test_cfg_key_decoders);
	RUN_TEST(test_cfg_key_table_encoding);
	RUN_TEST(test_valset_wire_bool);
	RUN_TEST(test_valset_wire_u4);
	RUN_TEST(test_valset_all_widths);
	RUN_TEST(test_valset_rejects);
	RUN_TEST(test_valset_width_mismatch_latches);
	RUN_TEST(test_valset_out_of_space);

	return UNITY_END();
}
