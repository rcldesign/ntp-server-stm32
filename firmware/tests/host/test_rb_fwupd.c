/*
 * STS1000 "Meridian" — host unit tests for core/fwupd/rb_fwupd.c.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Three things this suite pins down:
 *
 *  1. The 0x2C/0x2D/0x2E framing, byte by byte, against the layout in
 *     docs/rb_rs232_interface.md — including the protocol's own oddity that the
 *     length field is little-endian while the offset payload is big-endian.
 *  2. Capability classification, including that a healthy unit stops at
 *     telemetry-only unless the operator explicitly asks for more, and that
 *     firmware update reports NOT_SUPPORTED.
 *  3. The EFC trim guards: absolute bounds, step bound, readback verification,
 *     the EEPROM permission and its rate limit. Spec §3.4 is hands-off by
 *     default and these are what make "hands-off" enforceable rather than
 *     aspirational.
 */

#include <errno.h>
#include <string.h>

#include "fwupd/rb_fwupd.h"
#include "unity.h"

/* ===================================================================== *
 *  Fake FE-5680A
 * ===================================================================== */

#define FIFO 512U

typedef struct {
	uint8_t tx[FIFO];
	size_t tx_len;
	uint8_t rx[FIFO];
	size_t rx_head;
	size_t rx_len;

	uint64_t now;
	bool rail;

	int32_t offset;        /**< the unit's current volatile offset */
	int32_t eeprom_offset;

	/* behaviour */
	bool answer_read;      /**< reply to 0x2D at all */
	bool answer_set;       /**< reply to 0x2E / 0x2C */
	bool set_takes_effect; /**< a set actually changes the offset */
	bool emit_junk;        /**< prepend a corrupt frame to the next reply */
	bool emit_unknown;     /**< also emit an unsolicited unknown-id frame */
	int rc_tx;
	int rc_rx;

	unsigned int reads;
	unsigned int sets_vol;
	unsigned int sets_save;

	/* audit trail */
	unsigned int audits;
	char last_audit[48];
	int last_audit_rc;
} unit_t;

static void unit_reset(unit_t *u)
{
	(void)memset(u, 0, sizeof(*u));
	u->rail = true;
	u->answer_read = true;
	u->answer_set = true;
	u->set_takes_effect = true;
	u->offset = 1234;
}

static void push(unit_t *u, const uint8_t *d, size_t n)
{
	if ((u->rx_len + n) > sizeof(u->rx)) {
		return;
	}
	(void)memcpy(&u->rx[u->rx_len], d, n);
	u->rx_len += n;
}

static void push_offset_reply(unit_t *u, uint8_t id, int32_t v)
{
	uint8_t data[RB_OFFSET_DATA_LEN];
	uint8_t frame[RB_FRAME_MAX];
	size_t len = 0U;

	(void)rb_offset_encode(v, data);
	if (rb_frame_encode(frame, sizeof(frame), id, data, sizeof(data), &len) == 0) {
		push(u, frame, len);
	}
}

static void react(unit_t *u, const uint8_t *d, size_t len)
{
	rb_frame_t f;

	if (rb_frame_decode(d, len, &f) != 0) {
		return;
	}

	if (u->emit_junk) {
		/* A frame with a valid header checksum but a bad data checksum. */
		uint8_t junk[RB_OFFSET_FRAME_LEN];

		junk[0] = 0x77U;
		junk[1] = (uint8_t)RB_OFFSET_FRAME_LEN;
		junk[2] = 0U;
		junk[3] = (uint8_t)(junk[0] ^ junk[1] ^ junk[2]);
		junk[4] = 1U;
		junk[5] = 2U;
		junk[6] = 3U;
		junk[7] = 4U;
		junk[8] = 0xFFU; /* wrong */
		push(u, junk, sizeof(junk));
	}
	if (u->emit_unknown) {
		push_offset_reply(u, 0x31U, 42);
	}

	switch (f.id) {
	case RB_CMD_REQ_OFFSET:
		u->reads++;
		if (u->answer_read) {
			push_offset_reply(u, (uint8_t)RB_CMD_REQ_OFFSET, u->offset);
		}
		break;
	case RB_CMD_SET_OFFSET_VOL:
	case RB_CMD_SET_OFFSET_SAVE:
		if (f.id == (uint8_t)RB_CMD_SET_OFFSET_VOL) {
			u->sets_vol++;
		} else {
			u->sets_save++;
		}
		if (f.data_len == (uint8_t)RB_OFFSET_DATA_LEN) {
			int32_t v = 0;

			(void)rb_offset_decode(f.data, &v);
			if (u->set_takes_effect) {
				u->offset = v;
				if (f.id == (uint8_t)RB_CMD_SET_OFFSET_SAVE) {
					u->eeprom_offset = v;
				}
			}
		}
		if (u->answer_set) {
			push_offset_reply(u, f.id, u->offset);
		}
		break;
	default:
		break;
	}
}

static int o_tx(void *user, const uint8_t *d, size_t len)
{
	unit_t *u = user;

	if (u->rc_tx != 0) {
		return u->rc_tx;
	}
	if ((u->tx_len + len) <= sizeof(u->tx)) {
		(void)memcpy(&u->tx[u->tx_len], d, len);
		u->tx_len += len;
	}
	react(u, d, len);
	return 0;
}

static int o_rx(void *user, uint8_t *d, size_t cap)
{
	unit_t *u = user;
	size_t avail;

	if (u->rc_rx != 0) {
		return u->rc_rx;
	}
	avail = u->rx_len - u->rx_head;
	if (avail == 0U) {
		return 0;
	}
	if (avail > cap) {
		avail = cap;
	}
	(void)memcpy(d, &u->rx[u->rx_head], avail);
	u->rx_head += avail;
	return (int)avail;
}

static uint64_t o_now(void *user)
{
	unit_t *u = user;

	u->now += 1U;
	return u->now;
}

static void o_delay(void *user, uint32_t ms)
{
	unit_t *u = user;

	u->now += ms;
}

static bool o_rail(void *user)
{
	unit_t *u = user;

	return u->rail;
}

static void o_audit(void *user, const char *what, int32_t value, int rc)
{
	unit_t *u = user;

	(void)value;
	u->audits++;
	{
		size_t n = strlen(what);

		if (n >= sizeof(u->last_audit)) {
			n = sizeof(u->last_audit) - 1U;
		}
		(void)memcpy(u->last_audit, what, n);
		u->last_audit[n] = '\0';
	}
	u->last_audit_rc = rc;
}

static void ops_from(rb_fwupd_ops_t *o, unit_t *u)
{
	(void)memset(o, 0, sizeof(*o));
	o->tx = o_tx;
	o->rx = o_rx;
	o->now_ms = o_now;
	o->delay_ms = o_delay;
	o->rail_up = o_rail;
	o->audit = o_audit;
	o->user = u;
}

/* A context whose unit has already been classified EFC-trimmable. */
static void setup_trimmable(rb_ctx_t *c, unit_t *u, rb_fwupd_cfg_t *cfg)
{
	rb_fwupd_ops_t ops;
	rb_probe_req_t req;

	unit_reset(u);
	ops_from(&ops, u);
	rb_fwupd_cfg_defaults(cfg);
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_init(c, cfg, &ops));

	(void)memset(&req, 0, sizeof(req));
	req.probe_trim = true;
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_probe(c, &req));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)RB_CAP_EFC_TRIMMABLE, rb_capability(c));
}

/* ===================================================================== *
 *  Framing
 * ===================================================================== */

/*
 * A 0x2D request is exactly four octets: id, length low, length high, header
 * checksum. No data means no data checksum.
 */
static void test_frame_encode_no_data(void)
{
	uint8_t buf[RB_FRAME_MAX];
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, rb_frame_encode(buf, sizeof(buf),
			(uint8_t)RB_CMD_REQ_OFFSET, NULL, 0U, &len));
	TEST_ASSERT_EQUAL_UINT(RB_FRAME_HDR_LEN, len);
	TEST_ASSERT_EQUAL_HEX8(0x2DU, buf[0]);
	/* Length is the TOTAL frame length, little-endian. */
	TEST_ASSERT_EQUAL_HEX8(0x04U, buf[1]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, buf[2]);
	/* Header checksum is the XOR of the three octets before it. */
	TEST_ASSERT_EQUAL_HEX8((uint8_t)(0x2DU ^ 0x04U ^ 0x00U), buf[3]);
}

/*
 * A 0x2E write is nine octets: the four-octet header, a four-octet BIG-endian
 * signed offset, and the XOR of those four data octets.
 */
static void test_frame_encode_offset(void)
{
	uint8_t buf[RB_FRAME_MAX];
	uint8_t data[RB_OFFSET_DATA_LEN];
	size_t len = 0U;

	TEST_ASSERT_EQUAL_INT(0, rb_offset_encode(0x11223344L, data));
	TEST_ASSERT_EQUAL_HEX8(0x11U, data[0]);
	TEST_ASSERT_EQUAL_HEX8(0x22U, data[1]);
	TEST_ASSERT_EQUAL_HEX8(0x33U, data[2]);
	TEST_ASSERT_EQUAL_HEX8(0x44U, data[3]);

	TEST_ASSERT_EQUAL_INT(0, rb_frame_encode(buf, sizeof(buf),
			(uint8_t)RB_CMD_SET_OFFSET_VOL, data, sizeof(data), &len));
	TEST_ASSERT_EQUAL_UINT(RB_OFFSET_FRAME_LEN, len);
	TEST_ASSERT_EQUAL_UINT(9U, len);
	TEST_ASSERT_EQUAL_HEX8(0x2EU, buf[0]);
	TEST_ASSERT_EQUAL_HEX8(0x09U, buf[1]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, buf[2]);
	TEST_ASSERT_EQUAL_HEX8((uint8_t)(0x2EU ^ 0x09U ^ 0x00U), buf[3]);
	TEST_ASSERT_EQUAL_HEX8(0x11U, buf[4]);
	TEST_ASSERT_EQUAL_HEX8(0x22U, buf[5]);
	TEST_ASSERT_EQUAL_HEX8(0x33U, buf[6]);
	TEST_ASSERT_EQUAL_HEX8(0x44U, buf[7]);
	TEST_ASSERT_EQUAL_HEX8((uint8_t)(0x11U ^ 0x22U ^ 0x33U ^ 0x44U), buf[8]);

	/* The EEPROM variant differs only in the id. */
	TEST_ASSERT_EQUAL_INT(0, rb_frame_encode(buf, sizeof(buf),
			(uint8_t)RB_CMD_SET_OFFSET_SAVE, data, sizeof(data), &len));
	TEST_ASSERT_EQUAL_HEX8(0x2CU, buf[0]);
	TEST_ASSERT_EQUAL_HEX8((uint8_t)(0x2CU ^ 0x09U ^ 0x00U), buf[3]);
}

static void test_offset_signed_roundtrip(void)
{
	static const int32_t vals[] = {
		0, 1, -1, 1234, -1234, 0x7FFFFFFFL, -0x7FFFFFFFL - 1L,
		(int32_t)RB_OFFSET_MAX, -(int32_t)RB_OFFSET_MAX, 1000000L, -1000000L,
	};
	size_t i;

	for (i = 0U; i < (sizeof(vals) / sizeof(vals[0])); i++) {
		uint8_t d[RB_OFFSET_DATA_LEN];
		int32_t back = 0;

		TEST_ASSERT_EQUAL_INT(0, rb_offset_encode(vals[i], d));
		TEST_ASSERT_EQUAL_INT(0, rb_offset_decode(d, &back));
		TEST_ASSERT_EQUAL_INT32(vals[i], back);
	}

	/* INT32_MIN specifically: the sign reconstruction must not overflow. */
	{
		uint8_t d[4] = { 0x80U, 0x00U, 0x00U, 0x00U };
		int32_t back = 0;

		TEST_ASSERT_EQUAL_INT(0, rb_offset_decode(d, &back));
		TEST_ASSERT_EQUAL_INT32(-2147483647L - 1L, back);
	}

	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_offset_encode(0, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_offset_decode(NULL, (int32_t[1]){ 0 }));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_offset_decode((uint8_t[4]){ 0 }, NULL));
}

static void test_frame_decode_validates(void)
{
	uint8_t buf[RB_FRAME_MAX];
	uint8_t data[RB_OFFSET_DATA_LEN] = { 1, 2, 3, 4 };
	size_t len = 0U;
	rb_frame_t f;

	TEST_ASSERT_EQUAL_INT(0, rb_frame_encode(buf, sizeof(buf), 0x2DU, data,
						 sizeof(data), &len));
	TEST_ASSERT_EQUAL_INT(0, rb_frame_decode(buf, len, &f));
	TEST_ASSERT_EQUAL_HEX8(0x2DU, f.id);
	TEST_ASSERT_EQUAL_UINT8(4U, f.data_len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(data, f.data, 4U);

	/* A corrupted header checksum. */
	buf[3] ^= 0x01U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, rb_frame_decode(buf, len, &f));
	buf[3] ^= 0x01U;

	/* A corrupted data checksum. */
	buf[len - 1U] ^= 0x01U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, rb_frame_decode(buf, len, &f));
	buf[len - 1U] ^= 0x01U;

	/*
	 * A length field that disagrees with what arrived. Accepting this would
	 * mean decoding an offset from octets that are not there.
	 */
	TEST_ASSERT_EQUAL_INT(-EBADMSG, rb_frame_decode(buf, len - 1U, &f));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, rb_frame_decode(buf, len + 1U, &f));

	/* Too short to be a frame. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG, rb_frame_decode(buf, 3U, &f));

	/* A header claiming data but with no room for data + checksum. */
	buf[0] = 0x2DU;
	buf[1] = 0x05U;
	buf[2] = 0x00U;
	buf[3] = (uint8_t)(buf[0] ^ buf[1] ^ buf[2]);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, rb_frame_decode(buf, 5U, &f));

	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_frame_decode(NULL, 4U, &f));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_frame_decode(buf, 4U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_frame_encode(NULL, 8U, 0U, NULL, 0U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_frame_encode(buf, 8U, 0U, NULL, 4U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_frame_encode(buf, sizeof(buf), 0U, data,
			(size_t)RB_FRAME_MAX + 1U, NULL));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, rb_frame_encode(buf, 3U, 0U, NULL, 0U, NULL));
}

/*
 * The protocol has no sync byte, so the parser has to slide one octet at a time
 * out of garbage. Every frame it accepts has passed both checksums.
 */
static void test_parser_resynchronises(void)
{
	rb_parser_t p;
	rb_frame_t f;
	uint8_t good[RB_FRAME_MAX];
	size_t glen = 0U;
	uint8_t data[RB_OFFSET_DATA_LEN] = { 0xDE, 0xAD, 0xBE, 0xEF };
	size_t i;
	int hits = 0;

	TEST_ASSERT_EQUAL_INT(0, rb_frame_encode(good, sizeof(good), 0x2DU, data,
						 sizeof(data), &glen));

	rb_parser_init(&p);
	/* Leading garbage, then a good frame. */
	for (i = 0U; i < 7U; i++) {
		TEST_ASSERT_EQUAL_INT(0, rb_parse_byte(&p, (uint8_t)(0xA0U + i), &f));
	}
	for (i = 0U; i < glen; i++) {
		if (rb_parse_byte(&p, good[i], &f) == 1) {
			hits++;
		}
	}
	TEST_ASSERT_EQUAL_INT(1, hits);
	TEST_ASSERT_EQUAL_HEX8(0x2DU, f.id);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(data, f.data, 4U);
	TEST_ASSERT_TRUE(p.resyncs > 0U);
	TEST_ASSERT_EQUAL_UINT32(1U, p.frames);

	/* Two back-to-back frames both arrive. */
	rb_parser_init(&p);
	hits = 0;
	for (i = 0U; i < glen; i++) {
		if (rb_parse_byte(&p, good[i], &f) == 1) {
			hits++;
		}
	}
	for (i = 0U; i < glen; i++) {
		if (rb_parse_byte(&p, good[i], &f) == 1) {
			hits++;
		}
	}
	TEST_ASSERT_EQUAL_INT(2, hits);

	/* A stream of garbage never yields a frame and never runs off the buffer. */
	rb_parser_init(&p);
	for (i = 0U; i < 4U * RB_FRAME_MAX; i++) {
		TEST_ASSERT_EQUAL_INT(0, rb_parse_byte(&p, 0xFFU, &f));
	}
	TEST_ASSERT_EQUAL_UINT32(0U, p.frames);

	/* A frame whose header checksums but whose data does not: rejected, and
	 * the parser recovers to find the good frame that follows. */
	rb_parser_init(&p);
	{
		uint8_t bad[RB_OFFSET_FRAME_LEN];

		(void)memcpy(bad, good, glen);
		bad[glen - 1U] ^= 0xFFU;
		hits = 0;
		for (i = 0U; i < glen; i++) {
			if (rb_parse_byte(&p, bad[i], &f) == 1) {
				hits++;
			}
		}
		TEST_ASSERT_EQUAL_INT(0, hits);
		for (i = 0U; i < glen; i++) {
			if (rb_parse_byte(&p, good[i], &f) == 1) {
				hits++;
			}
		}
		TEST_ASSERT_EQUAL_INT(1, hits);
	}

	rb_parser_init(NULL);
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_parse_byte(NULL, 0U, &f));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_parse_byte(&p, 0U, NULL));
}

/* ===================================================================== *
 *  Read / probe / capability classification
 * ===================================================================== */

static void test_read_offset(void)
{
	rb_ctx_t c;
	unit_t u;
	rb_fwupd_cfg_t cfg;
	rb_fwupd_ops_t ops;
	int32_t v = 0;

	unit_reset(&u);
	u.offset = -4321;
	ops_from(&ops, &u);
	rb_fwupd_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_init(&c, &cfg, &ops));

	TEST_ASSERT_EQUAL_INT(0, rb_read_offset(&c, &v));
	TEST_ASSERT_EQUAL_INT32(-4321, v);
	TEST_ASSERT_EQUAL_UINT(1U, u.reads);

	/* The request that went out is a bare 0x2D. */
	TEST_ASSERT_EQUAL_UINT(RB_FRAME_HDR_LEN, u.tx_len);
	TEST_ASSERT_EQUAL_HEX8(0x2DU, u.tx[0]);

	/* NULL out is allowed: the status block is updated either way. */
	TEST_ASSERT_EQUAL_INT(0, rb_read_offset(&c, NULL));
	{
		rb_status_t st;

		TEST_ASSERT_EQUAL_INT(0, rb_fwupd_status(&c, &st));
		TEST_ASSERT_TRUE(st.offset_valid);
		TEST_ASSERT_EQUAL_INT32(-4321, st.offset);
		/* Lock and temperature are not in the documented command set. */
		TEST_ASSERT_FALSE(st.lock_valid);
		TEST_ASSERT_FALSE(st.temp_valid);
	}

	/*
	 * The rail is down, so the SN65C3221E has no power. This must be
	 * distinguishable from a dead unit: -ENODEV, and nothing transmitted.
	 */
	u.rail = false;
	u.tx_len = 0U;
	TEST_ASSERT_EQUAL_INT(-ENODEV, rb_read_offset(&c, &v));
	TEST_ASSERT_EQUAL_UINT(0U, u.tx_len);
	u.rail = true;

	/* A silent unit times out. */
	u.answer_read = false;
	TEST_ASSERT_EQUAL_INT(-ETIMEDOUT, rb_read_offset(&c, &v));
	u.answer_read = true;

	/* Transport failures surface. */
	u.rc_tx = -EIO;
	TEST_ASSERT_EQUAL_INT(-EIO, rb_read_offset(&c, &v));
	u.rc_tx = 0;
	u.rc_rx = -EIO;
	TEST_ASSERT_EQUAL_INT(-EIO, rb_read_offset(&c, &v));
	u.rc_rx = 0;

	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_read_offset(NULL, &v));
}

/*
 * A healthy unit classifies as telemetry-only and STOPS there. Probing further
 * means writing to a running frequency reference, so it takes an explicit
 * request.
 */
static void test_probe_stops_at_telemetry(void)
{
	rb_ctx_t c;
	unit_t u;
	rb_fwupd_cfg_t cfg;
	rb_fwupd_ops_t ops;

	unit_reset(&u);
	ops_from(&ops, &u);
	rb_fwupd_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_init(&c, &cfg, &ops));

	TEST_ASSERT_EQUAL_UINT8((uint8_t)RB_CAP_UNKNOWN, rb_capability(&c));

	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_probe(&c, NULL));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)RB_CAP_TELEMETRY_ONLY, rb_capability(&c));
	/* Not one write was attempted. */
	TEST_ASSERT_EQUAL_UINT(0U, u.sets_vol);
	TEST_ASSERT_EQUAL_UINT(0U, u.sets_save);

	/* An explicit request goes one step further. */
	{
		rb_probe_req_t req;

		(void)memset(&req, 0, sizeof(req));
		req.probe_trim = true;
		TEST_ASSERT_EQUAL_INT(0, rb_fwupd_probe(&c, &req));
		TEST_ASSERT_EQUAL_UINT8((uint8_t)RB_CAP_EFC_TRIMMABLE,
					rb_capability(&c));
		/* And the probe wrote back the value the unit already had, so it
		 * moved nothing. */
		TEST_ASSERT_EQUAL_UINT(1U, u.sets_vol);
		TEST_ASSERT_EQUAL_INT32(1234, u.offset);
		/* A probe must never consume an EEPROM write. */
		TEST_ASSERT_EQUAL_UINT(0U, u.sets_save);
	}
}

static void test_probe_classifies_absent(void)
{
	rb_ctx_t c;
	unit_t u;
	rb_fwupd_cfg_t cfg;
	rb_fwupd_ops_t ops;
	char buf[64];

	/* Nothing answers — which is also what a swapped J6.8/J6.9 looks like. */
	unit_reset(&u);
	u.answer_read = false;
	ops_from(&ops, &u);
	rb_fwupd_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_init(&c, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_probe(&c, NULL));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)RB_CAP_NONE, rb_capability(&c));

	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_identify(&c, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_STRING("FE-5680A absent", buf);

	/*
	 * The rail being down is NOT "absent" — it is "not asked yet".
	 *
	 * This assertion used to read the other way, and it was encoding a real
	 * defect as the contract. The SN65C3221E is powered from VCC_RB, so with
	 * the rail down the read cannot do anything but fail; recording
	 * RB_CAP_NONE for it answers a question that was never put. And it
	 * sticks: `cap` returns to UNKNOWN only in rb_fwupd_init(), and
	 * rb_fwupd_identify() re-probes only while it is UNKNOWN. Since
	 * `fw.inventory` is G0, one unauthenticated inventory during ordinary
	 * bring-up — before the power sequencer raises RB_PWR_EN — pinned the
	 * rubidium at "absent, check the cable" for the rest of the uptime.
	 *
	 * rb_fwupd.h already called rail_up "strongly advised" for exactly this
	 * reason; rb_fwupd_probe() simply never consulted it.
	 */
	unit_reset(&u);
	u.rail = false;
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_init(&c, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(-EHOSTDOWN, rb_fwupd_probe(&c, NULL));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)RB_CAP_UNKNOWN, rb_capability(&c));

	/* An inventory taken now says so, rather than claiming a verdict. */
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_identify(&c, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_STRING("FE-5680A unknown", buf);

	/*
	 * And it heals. The rail comes up, the next identify() probes again
	 * because the class is still UNKNOWN, and the unit is classified. This
	 * is the assertion that would have caught the original defect: under the
	 * old behaviour the class was already latched at NONE and no probe ever
	 * ran again.
	 */
	u.rail = true;
	u.answer_read = true;
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_identify(&c, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)RB_CAP_TELEMETRY_ONLY,
				rb_capability(&c));
}

/* A variant that acknowledges a set but does not apply it is not trimmable. */
static void test_probe_rejects_a_lying_unit(void)
{
	rb_ctx_t c;
	unit_t u;
	rb_fwupd_cfg_t cfg;
	rb_fwupd_ops_t ops;
	rb_probe_req_t req;

	unit_reset(&u);
	u.set_takes_effect = false;
	ops_from(&ops, &u);
	rb_fwupd_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_init(&c, &cfg, &ops));

	(void)memset(&req, 0, sizeof(req));
	req.probe_trim = true;
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_probe(&c, &req));
	/*
	 * The probe wrote the unit's own value back, so a unit that ignores sets
	 * still reads back the same number. That is indistinguishable from success
	 * by design — writing a *different* value to find out would move a running
	 * reference. The class is therefore EFC_TRIMMABLE here, and the real
	 * protection is rb_fwupd_trim()'s own readback (see
	 * test_trim_readback_mismatch).
	 */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)RB_CAP_EFC_TRIMMABLE, rb_capability(&c));

	/* A unit that does not answer a set at all stays telemetry-only. */
	unit_reset(&u);
	u.answer_set = false;
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_init(&c, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_probe(&c, &req));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)RB_CAP_TELEMETRY_ONLY, rb_capability(&c));
}

/*
 * The firmware-update path. No FE-5680A loader protocol is documented anywhere
 * this project can reach, so NOT_SUPPORTED is the required answer, not a
 * shortfall — and the probe must not classify anything as updatable.
 */
static void test_firmware_update_is_not_supported(void)
{
	rb_ctx_t c;
	unit_t u;
	rb_fwupd_cfg_t cfg;
	rb_fwupd_ops_t ops;
	rb_probe_req_t req;

	unit_reset(&u);
	ops_from(&ops, &u);
	rb_fwupd_cfg_defaults(&cfg);
	TEST_ASSERT_FALSE(cfg.loader_verified);
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_init(&c, &cfg, &ops));

	(void)memset(&req, 0, sizeof(req));
	req.probe_trim = true;
	req.probe_loader = true;
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_probe(&c, &req));
	/* Never RB_CAP_FW_UPDATABLE while the loader is unverified. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)RB_CAP_EFC_TRIMMABLE, rb_capability(&c));
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, rb_fwupd_begin(&c, 4096U));

	/* Even with the bit set, the class gates it. */
	cfg.loader_verified = true;
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_init(&c, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_probe(&c, &req));
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, rb_fwupd_begin(&c, 4096U));

	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_fwupd_begin(NULL, 1U));
}

static void test_identify(void)
{
	rb_ctx_t c;
	unit_t u;
	rb_fwupd_cfg_t cfg;
	char buf[64];

	setup_trimmable(&c, &u, &cfg);
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_identify(&c, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_STRING("FE-5680A efc-trimmable offset=1234", buf);

	/* Negative offsets render with a sign. */
	u.offset = -7;
	TEST_ASSERT_EQUAL_INT(0, rb_read_offset(&c, NULL));
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_identify(&c, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_STRING("FE-5680A efc-trimmable offset=-7", buf);

	/* Zero renders as "0", not as an empty string. */
	u.offset = 0;
	TEST_ASSERT_EQUAL_INT(0, rb_read_offset(&c, NULL));
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_identify(&c, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_STRING("FE-5680A efc-trimmable offset=0", buf);

	/* INT32_MIN, where a naive negation would overflow. */
	u.offset = -2147483647L - 1L;
	TEST_ASSERT_EQUAL_INT(0, rb_read_offset(&c, NULL));
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_identify(&c, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_STRING("FE-5680A efc-trimmable offset=-2147483648", buf);

	/* A short buffer truncates rather than overflowing. */
	{
		char small[10];

		TEST_ASSERT_EQUAL_INT(0, rb_fwupd_identify(&c, small,
							   sizeof(small)));
		TEST_ASSERT_EQUAL_UINT(9U, strlen(small));
	}

	/* identify() on an unprobed context probes first. */
	{
		rb_ctx_t c2;
		rb_fwupd_ops_t ops;

		unit_reset(&u);
		ops_from(&ops, &u);
		TEST_ASSERT_EQUAL_INT(0, rb_fwupd_init(&c2, &cfg, &ops));
		TEST_ASSERT_EQUAL_UINT8((uint8_t)RB_CAP_UNKNOWN,
					rb_capability(&c2));
		TEST_ASSERT_EQUAL_INT(0, rb_fwupd_identify(&c2, buf, sizeof(buf)));
		TEST_ASSERT_EQUAL_STRING("FE-5680A telemetry-only offset=1234",
					 buf);
	}

	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_fwupd_identify(NULL, buf, 8U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_fwupd_identify(&c, NULL, 8U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_fwupd_identify(&c, buf, 0U));
}

static void test_cap_names(void)
{
	unsigned int i;

	for (i = 0U; i < (unsigned int)RB_CAP__COUNT; i++) {
		TEST_ASSERT_NOT_NULL(rb_cap_name((uint8_t)i));
		TEST_ASSERT_TRUE(rb_cap_name((uint8_t)i)[0] != '?');
	}
	TEST_ASSERT_EQUAL_STRING("?", rb_cap_name(0xFFU));
}

/* ===================================================================== *
 *  EFC trim guards — spec §3.4 is hands-off by default
 * ===================================================================== */

static void test_trim_defaults_are_conservative(void)
{
	rb_fwupd_cfg_t cfg;

	rb_fwupd_cfg_defaults(&cfg);
	/* Two orders of magnitude inside full scale. */
	TEST_ASSERT_EQUAL_INT32(-1000000L, cfg.trim.min_offset);
	TEST_ASSERT_EQUAL_INT32(1000000L, cfg.trim.max_offset);
	TEST_ASSERT_EQUAL_INT32(100000L, cfg.trim.max_step);
	TEST_ASSERT_TRUE(cfg.trim.max_offset < (int32_t)RB_OFFSET_MAX / 100);
	/* EEPROM writes off, and rate-limited when enabled. */
	TEST_ASSERT_FALSE(cfg.trim.allow_eeprom);
	TEST_ASSERT_EQUAL_UINT32(3600U, cfg.trim.eeprom_min_interval_s);
	rb_fwupd_cfg_defaults(NULL);
}

static void test_trim_happy_path(void)
{
	rb_ctx_t c;
	unit_t u;
	rb_fwupd_cfg_t cfg;

	setup_trimmable(&c, &u, &cfg);
	u.sets_vol = 0U;

	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_trim(&c, 1300, false, 100U));
	TEST_ASSERT_EQUAL_INT32(1300, u.offset);
	TEST_ASSERT_EQUAL_UINT(1U, u.sets_vol);
	TEST_ASSERT_EQUAL_UINT(0U, u.sets_save);
	TEST_ASSERT_EQUAL_UINT32(1U, c.trims_ok);
	/* Audited. */
	TEST_ASSERT_EQUAL_STRING("trim-volatile", u.last_audit);
	TEST_ASSERT_EQUAL_INT(0, u.last_audit_rc);
}

static void test_trim_bounds(void)
{
	rb_ctx_t c;
	unit_t u;
	rb_fwupd_cfg_t cfg;

	setup_trimmable(&c, &u, &cfg);

	/* Outside the absolute envelope. */
	TEST_ASSERT_EQUAL_INT(-ERANGE, rb_fwupd_trim(&c, 1000001L, false, 1U));
	TEST_ASSERT_EQUAL_INT(-ERANGE, rb_fwupd_trim(&c, -1000001L, false, 1U));
	TEST_ASSERT_EQUAL_INT(-ERANGE, rb_fwupd_trim(&c,
			(int32_t)RB_OFFSET_MAX, false, 1U));
	TEST_ASSERT_EQUAL_STRING("trim-out-of-bounds", u.last_audit);
	/* Nothing was written. */
	TEST_ASSERT_EQUAL_INT32(1234, u.offset);

	/*
	 * The step bound. Inside the absolute envelope but too far in one go — a
	 * decimal-point slip must not be able to move the reference.
	 */
	TEST_ASSERT_EQUAL_INT(-ERANGE, rb_fwupd_trim(&c, 1234 + 100001L, false, 1U));
	TEST_ASSERT_EQUAL_STRING("trim-step-too-large", u.last_audit);
	TEST_ASSERT_EQUAL_INT32(1234, u.offset);

	/* Exactly at the step limit is allowed. */
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_trim(&c, 1234 + 100000L, false, 1U));
	TEST_ASSERT_EQUAL_INT32(101234, u.offset);

	/* And the same in the negative direction. */
	TEST_ASSERT_EQUAL_INT(-ERANGE, rb_fwupd_trim(&c, 101234 - 100001L, false, 2U));
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_trim(&c, 101234 - 100000L, false, 2U));
	TEST_ASSERT_EQUAL_INT32(1234, u.offset);

	TEST_ASSERT_TRUE(c.trims_refused >= 5U);
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_fwupd_trim(NULL, 0, false, 0U));
}

static void test_trim_requires_trimmable_class(void)
{
	rb_ctx_t c;
	unit_t u;
	rb_fwupd_cfg_t cfg;
	rb_fwupd_ops_t ops;

	unit_reset(&u);
	ops_from(&ops, &u);
	rb_fwupd_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_init(&c, &cfg, &ops));

	/* Unprobed. */
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, rb_fwupd_trim(&c, 100, false, 1U));

	/* Telemetry-only is not enough. */
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_probe(&c, NULL));
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, rb_fwupd_trim(&c, 100, false, 1U));
	TEST_ASSERT_EQUAL_STRING("trim-not-trimmable", u.last_audit);
	TEST_ASSERT_EQUAL_UINT(0U, u.sets_vol);

	/* Rail down is refused before anything else. */
	u.rail = false;
	TEST_ASSERT_EQUAL_INT(-ENODEV, rb_fwupd_trim(&c, 100, false, 1U));
	TEST_ASSERT_EQUAL_STRING("trim-rail-down", u.last_audit);
}

static void test_trim_eeprom_permission_and_rate_limit(void)
{
	rb_ctx_t c;
	unit_t u;
	rb_fwupd_cfg_t cfg;

	setup_trimmable(&c, &u, &cfg);

	/* Denied by default. */
	TEST_ASSERT_EQUAL_INT(-EACCES, rb_fwupd_trim(&c, 1300, true, 100U));
	TEST_ASSERT_EQUAL_STRING("trim-eeprom-not-allowed", u.last_audit);
	TEST_ASSERT_EQUAL_UINT(0U, u.sets_save);

	/* Enabled. */
	cfg.trim.allow_eeprom = true;
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_init(&c, &cfg, &(rb_fwupd_ops_t){
		.tx = o_tx, .rx = o_rx, .now_ms = o_now, .delay_ms = o_delay,
		.rail_up = o_rail, .audit = o_audit, .user = &u }));
	{
		rb_probe_req_t req;

		(void)memset(&req, 0, sizeof(req));
		req.probe_trim = true;
		TEST_ASSERT_EQUAL_INT(0, rb_fwupd_probe(&c, &req));
	}

	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_trim(&c, 1300, true, 1000U));
	TEST_ASSERT_EQUAL_UINT(1U, u.sets_save);
	TEST_ASSERT_EQUAL_INT32(1300, u.eeprom_offset);
	TEST_ASSERT_EQUAL_STRING("trim-saved", u.last_audit);

	/* A second EEPROM write inside the hour is refused. */
	TEST_ASSERT_EQUAL_INT(-EAGAIN, rb_fwupd_trim(&c, 1400, true, 1000U + 3599U));
	TEST_ASSERT_EQUAL_STRING("trim-eeprom-rate-limited", u.last_audit);
	TEST_ASSERT_EQUAL_UINT(1U, u.sets_save);

	/* A volatile write in the meantime is fine — that is the point of having
	 * two commands. */
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_trim(&c, 1400, false, 1000U + 10U));
	TEST_ASSERT_EQUAL_INT32(1400, u.offset);

	/* Once the interval has passed, EEPROM is allowed again. */
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_trim(&c, 1450, true, 1000U + 3600U));
	TEST_ASSERT_EQUAL_UINT(2U, u.sets_save);
}

/*
 * The failure mode these surplus units actually exhibit: the write is
 * acknowledged but not applied. The readback is what catches it.
 */
static void test_trim_readback_mismatch(void)
{
	rb_ctx_t c;
	unit_t u;
	rb_fwupd_cfg_t cfg;

	setup_trimmable(&c, &u, &cfg);

	u.set_takes_effect = false;
	TEST_ASSERT_EQUAL_INT(-EIO, rb_fwupd_trim(&c, 1300, false, 1U));
	TEST_ASSERT_EQUAL_STRING("trim-readback-mismatch", u.last_audit);
	TEST_ASSERT_TRUE(c.trims_refused >= 1U);
	TEST_ASSERT_EQUAL_UINT32(0U, c.trims_ok);

	/* A write that is not acknowledged at all. */
	u.set_takes_effect = true;
	u.answer_set = false;
	TEST_ASSERT_EQUAL_INT(-ETIMEDOUT, rb_fwupd_trim(&c, 1300, false, 2U));
	TEST_ASSERT_EQUAL_STRING("trim-write", u.last_audit);

	/* A unit that stops answering reads between the write and the readback. */
	u.answer_set = true;
	u.answer_read = false;
	TEST_ASSERT_EQUAL_INT(-ETIMEDOUT, rb_fwupd_trim(&c, 1300, false, 3U));
	TEST_ASSERT_EQUAL_STRING("trim-read-current", u.last_audit);
}

/* ===================================================================== *
 *  Unsolicited frames
 * ===================================================================== */

static void test_unknown_frames_are_kept_for_the_tool(void)
{
	rb_ctx_t c;
	unit_t u;
	rb_fwupd_cfg_t cfg;
	rb_fwupd_ops_t ops;
	rb_status_t st;

	unit_reset(&u);
	u.emit_unknown = true;
	ops_from(&ops, &u);
	rb_fwupd_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_init(&c, &cfg, &ops));

	TEST_ASSERT_EQUAL_INT(0, rb_read_offset(&c, NULL));
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_status(&c, &st));
	/*
	 * The unrecognised frame is counted and retained: it is the only evidence
	 * available about what a given surplus variant actually speaks.
	 */
	TEST_ASSERT_TRUE(st.unknown_frames >= 1U);
	TEST_ASSERT_EQUAL_HEX8(0x31U, st.last_unknown.id);

	/* A corrupt frame ahead of the reply does not prevent the reply. */
	unit_reset(&u);
	u.emit_junk = true;
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_init(&c, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(0, rb_read_offset(&c, NULL));

	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_fwupd_status(NULL, &st));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_fwupd_status(&c, NULL));
}

/* The tunnel/monitor feed path used by the maintenance tool. */
static void test_feed(void)
{
	rb_ctx_t c;
	unit_t u;
	rb_fwupd_cfg_t cfg;
	rb_fwupd_ops_t ops;
	uint8_t frame[RB_FRAME_MAX];
	size_t len = 0U;
	uint8_t data[RB_OFFSET_DATA_LEN];
	rb_status_t st;

	unit_reset(&u);
	ops_from(&ops, &u);
	rb_fwupd_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_init(&c, &cfg, &ops));

	/* A 0x2D response fed in updates the offset. */
	(void)rb_offset_encode(-999, data);
	TEST_ASSERT_EQUAL_INT(0, rb_frame_encode(frame, sizeof(frame),
			(uint8_t)RB_CMD_REQ_OFFSET, data, sizeof(data), &len));
	TEST_ASSERT_EQUAL_INT(1, rb_fwupd_feed(&c, frame, len));
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_status(&c, &st));
	TEST_ASSERT_TRUE(st.offset_valid);
	TEST_ASSERT_EQUAL_INT32(-999, st.offset);

	/* An unknown id is counted, not decoded as an offset. */
	TEST_ASSERT_EQUAL_INT(0, rb_frame_encode(frame, sizeof(frame), 0x55U,
						 data, sizeof(data), &len));
	TEST_ASSERT_EQUAL_INT(1, rb_fwupd_feed(&c, frame, len));
	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_status(&c, &st));
	TEST_ASSERT_EQUAL_INT32(-999, st.offset);
	TEST_ASSERT_EQUAL_UINT32(1U, st.unknown_frames);
	TEST_ASSERT_EQUAL_HEX8(0x55U, st.last_unknown.id);

	/* Garbage yields nothing. */
	{
		uint8_t junk[16];

		(void)memset(junk, 0xA5U, sizeof(junk));
		TEST_ASSERT_EQUAL_INT(0, rb_fwupd_feed(&c, junk, sizeof(junk)));
	}

	TEST_ASSERT_EQUAL_INT(0, rb_fwupd_feed(&c, frame, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_fwupd_feed(NULL, frame, 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_fwupd_feed(&c, NULL, 1U));
}

static void test_init_rejects(void)
{
	rb_ctx_t c;
	rb_fwupd_cfg_t cfg;
	rb_fwupd_ops_t ops;
	unit_t u;

	unit_reset(&u);
	ops_from(&ops, &u);
	rb_fwupd_cfg_defaults(&cfg);

	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_fwupd_init(NULL, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_fwupd_init(&c, NULL, &ops));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_fwupd_init(&c, &cfg, NULL));
	{
		rb_fwupd_ops_t bad = ops;

		bad.tx = NULL;
		TEST_ASSERT_EQUAL_INT(-EINVAL, rb_fwupd_init(&c, &cfg, &bad));
		bad = ops;
		bad.rx = NULL;
		TEST_ASSERT_EQUAL_INT(-EINVAL, rb_fwupd_init(&c, &cfg, &bad));
		bad = ops;
		bad.now_ms = NULL;
		TEST_ASSERT_EQUAL_INT(-EINVAL, rb_fwupd_init(&c, &cfg, &bad));
	}

	/* rail_up is optional: without it the rail is assumed up. */
	{
		rb_fwupd_ops_t no_rail = ops;

		no_rail.rail_up = NULL;
		TEST_ASSERT_EQUAL_INT(0, rb_fwupd_init(&c, &cfg, &no_rail));
		TEST_ASSERT_EQUAL_INT(0, rb_read_offset(&c, NULL));
	}
	TEST_ASSERT_EQUAL_UINT8((uint8_t)RB_CAP_UNKNOWN, rb_capability(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rb_fwupd_probe(NULL, NULL));
}

/* ===================================================================== *
 *  Runner
 * ===================================================================== */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_frame_encode_no_data);
	RUN_TEST(test_frame_encode_offset);
	RUN_TEST(test_offset_signed_roundtrip);
	RUN_TEST(test_frame_decode_validates);
	RUN_TEST(test_parser_resynchronises);

	RUN_TEST(test_read_offset);
	RUN_TEST(test_probe_stops_at_telemetry);
	RUN_TEST(test_probe_classifies_absent);
	RUN_TEST(test_probe_rejects_a_lying_unit);
	RUN_TEST(test_firmware_update_is_not_supported);
	RUN_TEST(test_identify);
	RUN_TEST(test_cap_names);

	RUN_TEST(test_trim_defaults_are_conservative);
	RUN_TEST(test_trim_happy_path);
	RUN_TEST(test_trim_bounds);
	RUN_TEST(test_trim_requires_trimmable_class);
	RUN_TEST(test_trim_eeprom_permission_and_rate_limit);
	RUN_TEST(test_trim_readback_mismatch);

	RUN_TEST(test_unknown_frames_are_kept_for_the_tool);
	RUN_TEST(test_feed);
	RUN_TEST(test_init_rejects);

	return UNITY_END();
}
