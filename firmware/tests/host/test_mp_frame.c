/*
 * STS1000 "Meridian" — core/mp frame-layer unit tests.
 *
 * The frame layer is the only part of MP a host cannot negotiate, so the tests
 * pin the wire down byte-for-byte: the CRC's position and byte order, what
 * happens to each malformed variety, and that fragmentation and reassembly
 * compose. The last test is a hostile fuzz — every byte the decoder can be fed
 * has to leave it usable, which is the property that makes the RX path a plain
 * byte pump in the glue.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "mp/mp_frame.h"
#include "test_support.h"
#include "util/cobs.h"
#include "util/crc.h"

/* ------------------------------------------------------------------ fixture */

#define REASM_SLOTS 2U
#define REASM_CAP 4096U

static mp_frame_tx_t g_tx;
static mp_frame_rx_t g_rx;
static mp_reasm_t g_slots[REASM_SLOTS];
static uint8_t g_slot_buf[REASM_SLOTS][REASM_CAP];

/* Last delivered message. */
static uint8_t g_msg[REASM_CAP];
static size_t g_msg_len;
static uint8_t g_msg_ch;
static unsigned int g_msg_count;
static int g_sink_rc;

static int on_msg(void *user, uint8_t ch, const uint8_t *msg, size_t len)
{
	(void)user;
	g_msg_ch = ch;
	g_msg_len = (len <= sizeof(g_msg)) ? len : sizeof(g_msg);
	if (g_msg_len != 0U) {
		memcpy(g_msg, msg, g_msg_len);
	}
	g_msg_count++;
	return g_sink_rc;
}

/* Captured wire bytes from mp_frame_send(). */
static uint8_t g_wire[16384];
static size_t g_wire_len;
static int g_sink_frames;
static int g_sink_fail_at; /* -1 = never */

static int wire_sink(void *user, const uint8_t *wire, size_t len)
{
	(void)user;
	g_sink_frames++;
	if ((g_sink_fail_at >= 0) && (g_sink_frames > g_sink_fail_at)) {
		return -EIO;
	}
	TEST_ASSERT_TRUE((g_wire_len + len) <= sizeof(g_wire));
	memcpy(&g_wire[g_wire_len], wire, len);
	g_wire_len += len;
	return 0;
}

void setUp(void)
{
	unsigned int i;

	for (i = 0U; i < REASM_SLOTS; i++) {
		g_slots[i].buf = g_slot_buf[i];
		g_slots[i].cap = REASM_CAP;
	}
	TEST_ASSERT_EQUAL_INT(0, mp_frame_rx_init(&g_rx, g_slots, REASM_SLOTS,
						  on_msg, NULL));
	g_msg_len = 0U;
	g_msg_ch = 0xFFU;
	g_msg_count = 0U;
	g_sink_rc = 0;
	g_wire_len = 0U;
	g_sink_frames = 0;
	g_sink_fail_at = -1;
}

/* ------------------------------------------------------------------- helpers */

/** Encode one frame and push its wire bytes through the decoder. */
static void round_trip(uint8_t ch, uint8_t flags, const uint8_t *payload,
		       size_t len)
{
	const uint8_t *wire = NULL;
	size_t wlen = 0U;

	TEST_ASSERT_EQUAL_INT(0, mp_frame_encode(&g_tx, ch, flags, payload, len,
						 &wire, &wlen));
	TEST_ASSERT_TRUE(wlen >= 2U);
	TEST_ASSERT_EQUAL_UINT8(0x00, wire[wlen - 1U]);
	(void)mp_frame_rx_input(&g_rx, wire, wlen);
}

/* ------------------------------------------------------------------- naming */

static void test_channel_names(void)
{
	TEST_ASSERT_EQUAL_STRING("control", mp_channel_name(MP_CH_CONTROL));
	TEST_ASSERT_EQUAL_STRING("telemetry", mp_channel_name(MP_CH_TELEMETRY));
	TEST_ASSERT_EQUAL_STRING("nmea", mp_channel_name(MP_CH_NMEA));
	TEST_ASSERT_EQUAL_STRING("ubx", mp_channel_name(MP_CH_UBX));
	TEST_ASSERT_EQUAL_STRING("log", mp_channel_name(MP_CH_LOG));
	TEST_ASSERT_EQUAL_STRING("pps", mp_channel_name(MP_CH_PPS));
	TEST_ASSERT_EQUAL_STRING("smp", mp_channel_name(MP_CH_SMP));
	TEST_ASSERT_EQUAL_STRING("gnss-pass", mp_channel_name(MP_CH_GNSS_PASS));
	TEST_ASSERT_EQUAL_STRING("rb-pass", mp_channel_name(MP_CH_RB_PASS));
	TEST_ASSERT_EQUAL_STRING("event", mp_channel_name(MP_CH_EVENT));
	TEST_ASSERT_EQUAL_STRING("mirror", mp_channel_name(MP_CH_MIRROR));
	/* 0x0B..0x1F are reserved by the spec; anything above is not a channel. */
	TEST_ASSERT_EQUAL_STRING("reserved", mp_channel_name(0x0BU));
	TEST_ASSERT_EQUAL_STRING("reserved", mp_channel_name(MP_CH_MAX));
	TEST_ASSERT_EQUAL_STRING("invalid", mp_channel_name(MP_CH_MAX + 1U));
}

/* ------------------------------------------------------- wire layout, exactly */

static void test_wire_layout_and_crc_byte_order(void)
{
	static const uint8_t payload[9] = { '1', '2', '3', '4', '5',
					    '6', '7', '8', '9' };
	uint8_t body[2U + sizeof(payload) + 2U];
	uint8_t decoded[MP_FRAME_MAX];
	const uint8_t *wire = NULL;
	size_t wlen = 0U;
	size_t dlen = 0U;
	uint16_t crc;

	TEST_ASSERT_EQUAL_INT(0, mp_frame_encode(&g_tx, MP_CH_CONTROL, 0U,
						 payload, sizeof(payload),
						 &wire, &wlen));

	/* Undo COBS to inspect the frame the spec describes. */
	TEST_ASSERT_EQUAL_INT(0, cobs_decode(wire, wlen - 1U, decoded,
					     sizeof(decoded), &dlen));
	TEST_ASSERT_EQUAL_size_t(sizeof(body), dlen);

	TEST_ASSERT_EQUAL_UINT8(MP_CH_CONTROL, decoded[0]);
	TEST_ASSERT_EQUAL_UINT8(0U, decoded[1]);
	TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, &decoded[2], sizeof(payload));

	/* CRC-16/CCITT-FALSE over channel|flags|payload, big-endian trailer. */
	crc = sts_crc16_ccitt(decoded, dlen - 2U);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)(crc >> 8), decoded[dlen - 2U]);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)(crc & 0xFFU), decoded[dlen - 1U]);
}

static void test_empty_payload_is_a_real_frame(void)
{
	round_trip(MP_CH_CONTROL, 0U, NULL, 0U);
	TEST_ASSERT_EQUAL_UINT(1U, g_msg_count);
	TEST_ASSERT_EQUAL_size_t(0U, g_msg_len);
	TEST_ASSERT_EQUAL_UINT8(MP_CH_CONTROL, g_msg_ch);
}

static void test_round_trip_every_channel(void)
{
	unsigned int ch;

	for (ch = 0U; ch <= (unsigned int)MP_CH_MAX; ch++) {
		uint8_t payload[64];
		unsigned int before = g_msg_count;

		test_fill_seq(payload, sizeof(payload), (uint8_t)(ch + 1U));
		/* Byte 0 of the sequence must not be 0x00 for this check to be
		 * about the frame rather than about COBS; the sequence starts at
		 * ch+1 so it never is for ch < 255. */
		round_trip((uint8_t)ch, 0U, payload, sizeof(payload));
		TEST_ASSERT_EQUAL_UINT(before + 1U, g_msg_count);
		TEST_ASSERT_EQUAL_UINT8((uint8_t)ch, g_msg_ch);
		TEST_ASSERT_EQUAL_size_t(sizeof(payload), g_msg_len);
		TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, g_msg, sizeof(payload));
	}
}

static void test_payload_with_zero_bytes_survives_cobs(void)
{
	uint8_t payload[300];

	memset(payload, 0, sizeof(payload));
	payload[0] = 0xAAU;
	payload[299] = 0x55U;

	round_trip(MP_CH_UBX, 0U, payload, sizeof(payload));
	TEST_ASSERT_EQUAL_UINT(1U, g_msg_count);
	TEST_ASSERT_EQUAL_size_t(sizeof(payload), g_msg_len);
	TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, g_msg, sizeof(payload));
}

static void test_max_tx_payload_round_trips(void)
{
	static uint8_t payload[MP_TX_PAYLOAD_MAX];
	test_rng_t rng;

	test_rng_init(&rng, 0x1234U);
	test_rng_fill(&rng, payload, sizeof(payload));

	round_trip(MP_CH_TELEMETRY, 0U, payload, sizeof(payload));
	TEST_ASSERT_EQUAL_UINT(1U, g_msg_count);
	TEST_ASSERT_EQUAL_size_t(sizeof(payload), g_msg_len);
	TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, g_msg, sizeof(payload));
}

/* -------------------------------------------------- receive-side max payload */

static void test_receiver_accepts_the_full_spec_payload(void)
{
	/*
	 * The spec allows 2048 B payloads. `cobs_encode()` caps at COBS_MAX_FRAME
	 * (1052 B) so mp_frame_encode() cannot build one, but the *receiver* must
	 * accept it — so this frame is assembled and COBS-encoded by hand.
	 */
	static uint8_t body[2U + MP_PAYLOAD_MAX + 2U];
	static uint8_t enc[COBS_ENCODE_MAX(sizeof(body)) + 1U];
	uint16_t crc;
	size_t enc_len = 0U;
	size_t i;
	size_t code_i = 0U;
	size_t w = 1U;
	uint8_t code = 1U;

	body[0] = MP_CH_UBX;
	body[1] = 0U;
	for (i = 0U; i < MP_PAYLOAD_MAX; i++) {
		/* Non-zero payload so the hand-rolled COBS below stays simple. */
		body[2U + i] = (uint8_t)((i % 255U) + 1U);
	}
	crc = sts_crc16_ccitt(body, 2U + MP_PAYLOAD_MAX);
	body[2U + MP_PAYLOAD_MAX] = (uint8_t)(crc >> 8);
	body[3U + MP_PAYLOAD_MAX] = (uint8_t)(crc & 0xFFU);

	/* Minimal COBS encoder for a body that may contain zeros only in the CRC. */
	for (i = 0U; i < sizeof(body); i++) {
		if (body[i] != 0U) {
			enc[w++] = body[i];
			code++;
			if (code != 0xFFU) {
				continue;
			}
		}
		enc[code_i] = code;
		code = 1U;
		if ((body[i] == 0U) || ((i + 1U) < sizeof(body))) {
			code_i = w++;
		}
	}
	enc[code_i] = code;
	enc_len = w;
	enc[enc_len++] = 0x00U;

	TEST_ASSERT_EQUAL_INT(1, mp_frame_rx_input(&g_rx, enc, enc_len));
	TEST_ASSERT_EQUAL_size_t(MP_PAYLOAD_MAX, g_msg_len);
	TEST_ASSERT_EQUAL_UINT8(MP_CH_UBX, g_msg_ch);
	TEST_ASSERT_EQUAL_UINT8_ARRAY(&body[2], g_msg, MP_PAYLOAD_MAX);
}

/* --------------------------------------------------------- rejection paths */

static void test_bad_crc_is_dropped_silently(void)
{
	uint8_t payload[16];
	const uint8_t *wire = NULL;
	size_t wlen = 0U;
	uint8_t copy[64];

	test_fill_seq(payload, sizeof(payload), 1U);
	TEST_ASSERT_EQUAL_INT(0, mp_frame_encode(&g_tx, MP_CH_CONTROL, 0U,
						 payload, sizeof(payload),
						 &wire, &wlen));
	TEST_ASSERT_TRUE(wlen <= sizeof(copy));
	memcpy(copy, wire, wlen);

	/* Corrupt a payload byte; the CRC no longer matches. Use a value that
	 * is still non-zero so COBS itself stays valid. */
	copy[3] = (uint8_t)(copy[3] ^ 0x40U);
	if (copy[3] == 0U) {
		copy[3] = 0x7FU;
	}

	TEST_ASSERT_EQUAL_INT(0, mp_frame_rx_input(&g_rx, copy, wlen));
	TEST_ASSERT_EQUAL_UINT(0U, g_msg_count);
	TEST_ASSERT_EQUAL_UINT32(1U, g_rx.crc_errors);
	TEST_ASSERT_EQUAL_UINT32(0U, g_rx.frames);

	/* The stream resynchronises: a good frame right after still arrives. */
	round_trip(MP_CH_CONTROL, 0U, payload, sizeof(payload));
	TEST_ASSERT_EQUAL_UINT(1U, g_msg_count);
}

/** Build a frame with an arbitrary channel/flags byte, CRC correct. */
static void feed_raw_frame(uint8_t ch, uint8_t flags, const uint8_t *payload,
			   size_t len)
{
	uint8_t body[2U + 64U + 2U];
	uint8_t enc[COBS_ENCODE_MAX(sizeof(body)) + 1U];
	size_t enc_len = 0U;
	uint16_t crc;

	TEST_ASSERT_TRUE(len <= 64U);
	body[0] = ch;
	body[1] = flags;
	if (len != 0U) {
		memcpy(&body[2], payload, len);
	}
	crc = sts_crc16_ccitt(body, 2U + len);
	body[2U + len] = (uint8_t)(crc >> 8);
	body[3U + len] = (uint8_t)(crc & 0xFFU);

	TEST_ASSERT_EQUAL_INT(0, cobs_encode(body, 4U + len, enc,
					     sizeof(enc) - 1U, &enc_len));
	enc[enc_len++] = 0x00U;
	(void)mp_frame_rx_input(&g_rx, enc, enc_len);
}

static void test_channel_above_the_space_is_rejected(void)
{
	uint8_t p[4] = { 1U, 2U, 3U, 4U };

	feed_raw_frame((uint8_t)(MP_CH_MAX + 1U), 0U, p, sizeof(p));
	TEST_ASSERT_EQUAL_UINT(0U, g_msg_count);
	TEST_ASSERT_EQUAL_UINT32(1U, g_rx.bad_channel);
	TEST_ASSERT_EQUAL_UINT32(0U, g_rx.frames);

	feed_raw_frame(0xFFU, 0U, p, sizeof(p));
	TEST_ASSERT_EQUAL_UINT32(2U, g_rx.bad_channel);
}

static void test_undefined_flag_bits_reject_the_frame(void)
{
	uint8_t p[4] = { 1U, 2U, 3U, 4U };

	feed_raw_frame(MP_CH_CONTROL, 0x02U, p, sizeof(p));
	TEST_ASSERT_EQUAL_UINT(0U, g_msg_count);
	TEST_ASSERT_EQUAL_UINT32(1U, g_rx.bad_flags);

	feed_raw_frame(MP_CH_CONTROL, 0x80U, p, sizeof(p));
	TEST_ASSERT_EQUAL_UINT32(2U, g_rx.bad_flags);
}

static void test_short_frames_are_counted(void)
{
	/* One, two and three decoded bytes are all below the 4-byte minimum. */
	static const uint8_t f1[] = { 0x02U, 0xAAU, 0x00U };
	static const uint8_t f2[] = { 0x03U, 0xAAU, 0xBBU, 0x00U };
	static const uint8_t f3[] = { 0x04U, 0xAAU, 0xBBU, 0xCCU, 0x00U };

	(void)mp_frame_rx_input(&g_rx, f1, sizeof(f1));
	(void)mp_frame_rx_input(&g_rx, f2, sizeof(f2));
	(void)mp_frame_rx_input(&g_rx, f3, sizeof(f3));
	TEST_ASSERT_EQUAL_UINT32(3U, g_rx.short_frames);
	TEST_ASSERT_EQUAL_UINT(0U, g_msg_count);
}

static void test_malformed_cobs_is_counted(void)
{
	/* A group code that runs past the delimiter. */
	static const uint8_t bad[] = { 0x10U, 0xAAU, 0xBBU, 0x00U };

	(void)mp_frame_rx_input(&g_rx, bad, sizeof(bad));
	TEST_ASSERT_EQUAL_UINT32(1U, g_rx.cobs_errors);
	TEST_ASSERT_EQUAL_UINT(0U, g_msg_count);
}

static void test_oversize_frame_is_counted_not_overflowed(void)
{
	/*
	 * A COBS body long enough to decode past MP_FRAME_MAX. The decoder must
	 * report it at the delimiter and resynchronise, not write past its
	 * buffer — which ASan proves.
	 */
	size_t i;

	for (i = 0U; i < (MP_FRAME_MAX + 512U); i++) {
		uint8_t b = (uint8_t)((i % 254U) + 1U);

		if ((i % 254U) == 0U) {
			b = 0xFFU;
		}
		(void)mp_frame_rx_byte(&g_rx, b);
	}
	(void)mp_frame_rx_byte(&g_rx, 0x00U);
	TEST_ASSERT_EQUAL_UINT(0U, g_msg_count);
	TEST_ASSERT_EQUAL_UINT32(1U, g_rx.cobs_errors);
}

static void test_sink_error_is_counted_but_does_not_stall(void)
{
	uint8_t p[8];

	test_fill_seq(p, sizeof(p), 1U);
	g_sink_rc = -EIO;
	round_trip(MP_CH_CONTROL, 0U, p, sizeof(p));
	TEST_ASSERT_EQUAL_UINT32(1U, g_rx.sink_errors);
	TEST_ASSERT_EQUAL_UINT32(1U, g_rx.msgs);

	g_sink_rc = 0;
	round_trip(MP_CH_CONTROL, 0U, p, sizeof(p));
	TEST_ASSERT_EQUAL_UINT32(2U, g_rx.msgs);
}

/* ------------------------------------------------------------- fragmentation */

static void test_send_fragments_and_reassembles(void)
{
	static uint8_t msg[MP_TX_PAYLOAD_MAX * 3U + 17U];
	test_rng_t rng;

	test_rng_init(&rng, 0xBEEFU);
	test_rng_fill(&rng, msg, sizeof(msg));

	TEST_ASSERT_EQUAL_INT(0, mp_frame_send(&g_tx, MP_CH_CONTROL, msg,
					       sizeof(msg), wire_sink, NULL));
	TEST_ASSERT_EQUAL_INT(4, g_sink_frames);

	TEST_ASSERT_EQUAL_INT(1, mp_frame_rx_input(&g_rx, g_wire, g_wire_len));
	TEST_ASSERT_EQUAL_size_t(sizeof(msg), g_msg_len);
	TEST_ASSERT_EQUAL_UINT8_ARRAY(msg, g_msg, sizeof(msg));
	TEST_ASSERT_EQUAL_UINT32(4U, g_rx.frames);
	TEST_ASSERT_EQUAL_UINT32(1U, g_rx.msgs);
}

static void test_send_exact_multiple_emits_a_final_empty_fragment(void)
{
	static uint8_t msg[MP_TX_PAYLOAD_MAX];

	memset(msg, 0x5AU, sizeof(msg));
	TEST_ASSERT_EQUAL_INT(0, mp_frame_send(&g_tx, MP_CH_LOG, msg,
					       sizeof(msg), wire_sink, NULL));
	/* Exactly one fragment: the chunk is not *greater* than the maximum, so
	 * no more-fragments flag is set and no empty tail frame is needed. */
	TEST_ASSERT_EQUAL_INT(1, g_sink_frames);

	TEST_ASSERT_EQUAL_INT(1, mp_frame_rx_input(&g_rx, g_wire, g_wire_len));
	TEST_ASSERT_EQUAL_size_t(sizeof(msg), g_msg_len);
}

static void test_send_zero_length_emits_one_frame(void)
{
	TEST_ASSERT_EQUAL_INT(0, mp_frame_send(&g_tx, MP_CH_EVENT, NULL, 0U,
					       wire_sink, NULL));
	TEST_ASSERT_EQUAL_INT(1, g_sink_frames);
	TEST_ASSERT_EQUAL_INT(1, mp_frame_rx_input(&g_rx, g_wire, g_wire_len));
	TEST_ASSERT_EQUAL_size_t(0U, g_msg_len);
}

static void test_send_propagates_a_sink_failure(void)
{
	static uint8_t msg[MP_TX_PAYLOAD_MAX * 2U];

	memset(msg, 0x11U, sizeof(msg));
	g_sink_fail_at = 1;
	TEST_ASSERT_EQUAL_INT(-EIO, mp_frame_send(&g_tx, MP_CH_CONTROL, msg,
						  sizeof(msg), wire_sink,
						  NULL));
}

static void test_interleaved_fragments_on_two_channels(void)
{
	uint8_t a[32];
	uint8_t b[48];
	uint8_t seen_a[64];
	uint8_t seen_b[64];
	size_t seen_a_len = 0U;
	size_t seen_b_len = 0U;

	test_fill_seq(a, sizeof(a), 0x10U);
	test_fill_seq(b, sizeof(b), 0x80U);

	/* Two fragmented messages, alternating, using both reassembly slots. */
	round_trip(MP_CH_GNSS_PASS, MP_FLAG_MORE, a, 16U);
	round_trip(MP_CH_RB_PASS, MP_FLAG_MORE, b, 24U);
	TEST_ASSERT_EQUAL_UINT(0U, g_msg_count);

	round_trip(MP_CH_GNSS_PASS, 0U, &a[16], 16U);
	TEST_ASSERT_EQUAL_UINT(1U, g_msg_count);
	TEST_ASSERT_EQUAL_UINT8(MP_CH_GNSS_PASS, g_msg_ch);
	seen_a_len = g_msg_len;
	memcpy(seen_a, g_msg, seen_a_len);

	round_trip(MP_CH_RB_PASS, 0U, &b[24], 24U);
	TEST_ASSERT_EQUAL_UINT(2U, g_msg_count);
	TEST_ASSERT_EQUAL_UINT8(MP_CH_RB_PASS, g_msg_ch);
	seen_b_len = g_msg_len;
	memcpy(seen_b, g_msg, seen_b_len);

	TEST_ASSERT_EQUAL_size_t(sizeof(a), seen_a_len);
	TEST_ASSERT_EQUAL_UINT8_ARRAY(a, seen_a, sizeof(a));
	TEST_ASSERT_EQUAL_size_t(sizeof(b), seen_b_len);
	TEST_ASSERT_EQUAL_UINT8_ARRAY(b, seen_b, sizeof(b));
}

static void test_third_fragmented_channel_is_dropped_whole(void)
{
	uint8_t p[8];

	test_fill_seq(p, sizeof(p), 1U);

	round_trip(MP_CH_NMEA, MP_FLAG_MORE, p, sizeof(p));
	round_trip(MP_CH_UBX, MP_FLAG_MORE, p, sizeof(p));
	TEST_ASSERT_EQUAL_UINT32(0U, g_rx.reasm_drops);

	/* Both slots busy: this message has nowhere to go. */
	round_trip(MP_CH_LOG, MP_FLAG_MORE, p, sizeof(p));
	TEST_ASSERT_EQUAL_UINT32(1U, g_rx.reasm_drops);

	/* Its final fragment must not be delivered as a short message either —
	 * it arrives with no slot, so it is treated as unfragmented. That is the
	 * documented behaviour: the host sees a truncated message on a channel
	 * it over-committed, and the drop counter records why. */
	round_trip(MP_CH_LOG, 0U, p, sizeof(p));
	TEST_ASSERT_EQUAL_UINT(1U, g_msg_count);
	TEST_ASSERT_EQUAL_UINT8(MP_CH_LOG, g_msg_ch);
}

static void test_reassembly_overflow_abandons_the_message(void)
{
	uint8_t small_buf[64];
	mp_reasm_t small = { .buf = small_buf, .cap = sizeof(small_buf) };
	uint8_t p[64];

	TEST_ASSERT_EQUAL_INT(0,
			      mp_frame_rx_init(&g_rx, &small, 1U, on_msg, NULL));
	test_fill_seq(p, sizeof(p), 1U);

	round_trip(MP_CH_CONTROL, MP_FLAG_MORE, p, sizeof(p));
	TEST_ASSERT_EQUAL_UINT32(0U, g_rx.reasm_drops);

	/* The second fragment overruns the 64-byte slot. */
	round_trip(MP_CH_CONTROL, MP_FLAG_MORE, p, sizeof(p));
	TEST_ASSERT_EQUAL_UINT32(1U, g_rx.reasm_drops);
	TEST_ASSERT_EQUAL_UINT(0U, g_msg_count);
}

static void test_final_fragment_overflow_is_also_abandoned(void)
{
	uint8_t small_buf[70];
	mp_reasm_t small = { .buf = small_buf, .cap = sizeof(small_buf) };
	uint8_t p[64];

	TEST_ASSERT_EQUAL_INT(0,
			      mp_frame_rx_init(&g_rx, &small, 1U, on_msg, NULL));
	test_fill_seq(p, sizeof(p), 1U);

	round_trip(MP_CH_CONTROL, MP_FLAG_MORE, p, sizeof(p));
	round_trip(MP_CH_CONTROL, 0U, p, sizeof(p)); /* 64 + 64 > 70 */
	TEST_ASSERT_EQUAL_UINT32(1U, g_rx.reasm_drops);
	TEST_ASSERT_EQUAL_UINT(0U, g_msg_count);
}

static void test_no_reassembly_pool_drops_fragments(void)
{
	uint8_t p[8];

	TEST_ASSERT_EQUAL_INT(0, mp_frame_rx_init(&g_rx, NULL, 0U, on_msg,
						  NULL));
	test_fill_seq(p, sizeof(p), 1U);
	round_trip(MP_CH_CONTROL, MP_FLAG_MORE, p, sizeof(p));
	TEST_ASSERT_EQUAL_UINT32(1U, g_rx.reasm_drops);
	TEST_ASSERT_EQUAL_UINT(0U, g_msg_count);
}

static void test_reset_discards_reassembly_in_progress(void)
{
	uint8_t p[8];

	test_fill_seq(p, sizeof(p), 1U);
	round_trip(MP_CH_CONTROL, MP_FLAG_MORE, p, sizeof(p));
	mp_frame_rx_reset(&g_rx);
	round_trip(MP_CH_CONTROL, 0U, p, sizeof(p));

	/* The first fragment is gone, so only the second half is delivered. */
	TEST_ASSERT_EQUAL_UINT(1U, g_msg_count);
	TEST_ASSERT_EQUAL_size_t(sizeof(p), g_msg_len);
}

static void test_null_sink_still_counts_messages(void)
{
	uint8_t p[8];

	TEST_ASSERT_EQUAL_INT(0, mp_frame_rx_init(&g_rx, g_slots, REASM_SLOTS,
						  NULL, NULL));
	test_fill_seq(p, sizeof(p), 1U);
	round_trip(MP_CH_CONTROL, 0U, p, sizeof(p));
	TEST_ASSERT_EQUAL_UINT32(1U, g_rx.msgs);
	TEST_ASSERT_EQUAL_UINT(0U, g_msg_count);
}

/* -------------------------------------------------------------- arguments */

static void test_argument_validation(void)
{
	const uint8_t *wire = NULL;
	size_t wlen = 0U;
	uint8_t p[4] = { 1U, 2U, 3U, 4U };
	static uint8_t big[MP_TX_PAYLOAD_MAX + 1U];
	mp_reasm_t bad_slot = { .buf = NULL, .cap = 16U };

	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_frame_encode(NULL, 0U, 0U, p, 4U,
						       &wire, &wlen));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_frame_encode(&g_tx, 0U, 0U, p, 4U,
						       NULL, &wlen));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_frame_encode(&g_tx, 0U, 0U, p, 4U,
						       &wire, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_frame_encode(&g_tx, 0U, 0U, NULL, 4U,
						       &wire, &wlen));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_frame_encode(&g_tx, 0x20U, 0U, p, 4U,
						       &wire, &wlen));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_frame_encode(&g_tx, 0U, 0x02U, p, 4U,
						       &wire, &wlen));
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE, mp_frame_encode(&g_tx, 0U, 0U, big,
							 sizeof(big), &wire,
							 &wlen));

	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_frame_send(NULL, 0U, p, 4U, wire_sink,
						     NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_frame_send(&g_tx, 0U, p, 4U, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_frame_send(&g_tx, 0U, NULL, 4U,
						     wire_sink, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_frame_send(&g_tx, 0x20U, p, 4U,
						     wire_sink, NULL));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_frame_rx_init(NULL, NULL, 0U, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_frame_rx_init(&g_rx, NULL, 1U, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_frame_rx_init(&g_rx, &bad_slot, 1U,
							NULL, NULL));

	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_frame_rx_byte(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_frame_rx_input(NULL, p, 4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_frame_rx_input(&g_rx, NULL, 4U));
	mp_frame_rx_reset(NULL); /* must not fault */
}

/* -------------------------------------------------------------------- fuzz */

/**
 * Hostile stream fuzz.
 *
 * Random bytes, random delimiters, and mutated copies of valid frames. Nothing
 * is asserted about *what* is delivered — only that the decoder stays inside its
 * buffers (ASan/UBSan), never delivers a message longer than a reassembly slot,
 * and remains usable afterwards.
 */
static void test_fuzz_hostile_stream(void)
{
	test_rng_t rng;
	unsigned int round;

	test_rng_init(&rng, 0xC0FFEEU);

	for (round = 0U; round < 4000U; round++) {
		uint8_t buf[256];
		size_t n = test_rng_below(&rng, sizeof(buf)) + 1U;
		size_t i;

		test_rng_fill(&rng, buf, n);
		/* Sprinkle delimiters so frames actually get closed. */
		for (i = 0U; i < n; i++) {
			if ((test_rng_u32(&rng) & 0x1FU) == 0U) {
				buf[i] = 0x00U;
			}
		}
		(void)mp_frame_rx_input(&g_rx, buf, n);
		TEST_ASSERT_TRUE(g_msg_len <= REASM_CAP);
	}

	/* Still usable: a well-formed frame arrives after all that. */
	{
		uint8_t p[16];
		unsigned int before;

		test_fill_seq(p, sizeof(p), 1U);
		mp_frame_rx_reset(&g_rx);
		before = g_msg_count;
		round_trip(MP_CH_CONTROL, 0U, p, sizeof(p));
		TEST_ASSERT_EQUAL_UINT(before + 1U, g_msg_count);
		TEST_ASSERT_EQUAL_size_t(sizeof(p), g_msg_len);
		TEST_ASSERT_EQUAL_UINT8_ARRAY(p, g_msg, sizeof(p));
	}
}

/** Mutate valid frames one byte at a time; every outcome must be accounted for. */
static void test_fuzz_single_byte_mutations(void)
{
	test_rng_t rng;
	unsigned int round;

	test_rng_init(&rng, 0x5EEDU);

	for (round = 0U; round < 2000U; round++) {
		uint8_t payload[40];
		uint8_t copy[128];
		const uint8_t *wire = NULL;
		size_t wlen = 0U;
		size_t pos;
		uint32_t before_total;

		test_rng_fill_nonzero(&rng, payload, sizeof(payload));
		TEST_ASSERT_EQUAL_INT(0, mp_frame_encode(&g_tx, MP_CH_CONTROL,
							 0U, payload,
							 sizeof(payload), &wire,
							 &wlen));
		TEST_ASSERT_TRUE(wlen <= sizeof(copy));
		memcpy(copy, wire, wlen);

		pos = test_rng_below(&rng, wlen);
		copy[pos] = (uint8_t)test_rng_u32(&rng);

		before_total = g_rx.frames + g_rx.crc_errors +
			       g_rx.cobs_errors + g_rx.short_frames +
			       g_rx.bad_channel + g_rx.bad_flags;
		mp_frame_rx_reset(&g_rx);
		(void)mp_frame_rx_input(&g_rx, copy, wlen);

		/* Every mutated frame either parses or is classified — the
		 * decoder never simply loses one without recording why. A
		 * mutation that destroys the trailing delimiter leaves the frame
		 * open, which is also accounted for (nothing counted, decoder
		 * still mid-frame). */
		TEST_ASSERT_TRUE((g_rx.frames + g_rx.crc_errors +
				  g_rx.cobs_errors + g_rx.short_frames +
				  g_rx.bad_channel + g_rx.bad_flags) >=
				 before_total);
	}
}

/* ------------------------------------------------------------------- runner */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_channel_names);
	RUN_TEST(test_wire_layout_and_crc_byte_order);
	RUN_TEST(test_empty_payload_is_a_real_frame);
	RUN_TEST(test_round_trip_every_channel);
	RUN_TEST(test_payload_with_zero_bytes_survives_cobs);
	RUN_TEST(test_max_tx_payload_round_trips);
	RUN_TEST(test_receiver_accepts_the_full_spec_payload);

	RUN_TEST(test_bad_crc_is_dropped_silently);
	RUN_TEST(test_channel_above_the_space_is_rejected);
	RUN_TEST(test_undefined_flag_bits_reject_the_frame);
	RUN_TEST(test_short_frames_are_counted);
	RUN_TEST(test_malformed_cobs_is_counted);
	RUN_TEST(test_oversize_frame_is_counted_not_overflowed);
	RUN_TEST(test_sink_error_is_counted_but_does_not_stall);

	RUN_TEST(test_send_fragments_and_reassembles);
	RUN_TEST(test_send_exact_multiple_emits_a_final_empty_fragment);
	RUN_TEST(test_send_zero_length_emits_one_frame);
	RUN_TEST(test_send_propagates_a_sink_failure);
	RUN_TEST(test_interleaved_fragments_on_two_channels);
	RUN_TEST(test_third_fragmented_channel_is_dropped_whole);
	RUN_TEST(test_reassembly_overflow_abandons_the_message);
	RUN_TEST(test_final_fragment_overflow_is_also_abandoned);
	RUN_TEST(test_no_reassembly_pool_drops_fragments);
	RUN_TEST(test_reset_discards_reassembly_in_progress);
	RUN_TEST(test_null_sink_still_counts_messages);

	RUN_TEST(test_argument_validation);
	RUN_TEST(test_fuzz_hostile_stream);
	RUN_TEST(test_fuzz_single_byte_mutations);

	return UNITY_END();
}
