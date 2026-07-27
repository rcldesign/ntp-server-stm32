/*
 * STS1000 "Meridian" — core/ntp unit tests.
 *
 * Provenance of the expectations (ARCHITECTURE.md §9 asks for known-good
 * vectors, not self-round-trips):
 *
 *   - The response header is asserted as a hand-written 48-octet array, field
 *     by field against the layout in RFC 5905 §7.3. Nothing in it is produced
 *     by calling the code under test: the timestamps are literals derived from
 *     the epoch arithmetic below, so a sign error or a swapped field shows up
 *     as a byte difference rather than agreeing with itself.
 *
 *   - Epoch arithmetic: the NTP prime epoch is 1900-01-01 and the Unix epoch
 *     1970-01-01, 2208988800 s apart (RFC 5905 §6). Hence UTC 2024-01-01
 *     00:00:00 = Unix 1704067200 = NTP 3913056000 = 0xE93C7F00. Era 0 ends at
 *     Unix 2085978495 (NTP 0xFFFFFFFF) and era 1 begins one second later at
 *     NTP 0, since 2085978496 + 2208988800 = 2^32 exactly.
 *
 *   - The Kiss-o'-Death is checked against RFC 5905 §7.4: stratum 0 and a
 *     four-octet ASCII kiss code, here "RATE".
 *
 *   - Interleaved mode follows draft-ietf-ntp-interleaved-modes, the variant
 *     ntpd and chrony ship: the client marks a request by echoing the previous
 *     response's transmit field as its origin, and the reply then reports the
 *     previous request's receive timestamp and the previous response's real
 *     transmit timestamp. The test drives the full three-packet handshake
 *     rather than asserting on one message.
 *
 *   - The symmetric MAC is checked structurally (key id, digest length,
 *     placement) and behaviourally (a one-bit change anywhere fails). The
 *     HMAC-SHA-256 underneath is pinned to the RFC 4231 vectors in
 *     test_aes_siv.c, so it is taken as given here.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "host_aes.h"
#include "host_sha256.h"
#include "ntp/ntp.h"
#include "util/bytes.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* UTC 2024-01-01T00:00:00Z. */
#define UNIX_2024 INT64_C(1704067200)
#define NTP_2024 UINT32_C(0xE93C7F00)
/* The last second of NTP era 0: 2036-02-07T06:28:15Z. */
#define UNIX_ERA0_LAST INT64_C(2085978495)
#define TAI_OFFSET 37

#define NS INT64_C(1000000000)

static host_crypto_t g_hc;
static port_crypto_t g_port;
static ntp_ctx_t g_ctx; /* ~20 kB of client table: static, not on the stack */

void setUp(void)
{
	host_crypto_init(&g_hc, 0x1234ABCDU);
	g_port = host_crypto_port(&g_hc);
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, NULL, &g_port, 0));
}

void tearDown(void)
{
}

/* ------------------------------------------------------------------ helpers */

/** A well-formed client request. */
static size_t make_request(uint8_t *buf, uint8_t vn, uint8_t mode, uint8_t poll,
			   uint64_t xmt, uint64_t org)
{
	memset(buf, 0, NTP_HDR_LEN);
	buf[0] = (uint8_t)((0U << 6) | ((vn & 7U) << 3) | (mode & 7U));
	buf[1] = 0U;  /* stratum: unspecified, as a client sends */
	buf[2] = poll;
	buf[3] = 0xE9U; /* precision -23 */
	bytes_put_be64(&buf[24], org);
	bytes_put_be64(&buf[40], xmt);
	return NTP_HDR_LEN;
}

/** A locked stratum-1 quality snapshot. */
static void good_quality(ntp_quality_view_t *q)
{
	ntp_quality_view_default(q);
	q->leap = (uint8_t)NTP_LI_NONE;
	q->stratum = (uint8_t)NTP_STRATUM_PRIM;
	q->precision = -20;
	q->refid = 0U; /* derive: stratum 1 means GPS */
	q->root_delay_q16 = 0x00000100U;
	q->root_disp_q16 = 0x00001000U;
	q->tai_minus_utc = TAI_OFFSET;
	q->ref_tai_ns = (UNIX_2024 + TAI_OFFSET) * NS;
	q->synchronized = true;
	q->holdover = false;
}

static void fill_rx(ntp_rx_t *rx, const uint8_t *pkt, size_t len, uint32_t id,
		    int64_t now_ms)
{
	memset(rx, 0, sizeof(*rx));
	rx->pkt = pkt;
	rx->len = len;
	rx->client_id = id;
	/* Tie the hardware timestamps to the virtual clock so two exchanges a
	 * millisecond apart do not share a receive timestamp. The 20 µs gap is a
	 * plausible receive-to-transmit turnaround. */
	rx->rx_tai_ns = (UNIX_2024 + TAI_OFFSET) * NS + now_ms * 1000000;
	rx->tx_tai_ns = rx->rx_tai_ns + 20000;
	rx->now_ms = now_ms;
}

/* ---------------------------------------------------------- timestamp model */

static void test_tai_to_ntp_conversion(void)
{
	/* The Unix epoch itself, with no leap offset: the 2208988800 constant. */
	TEST_ASSERT_EQUAL_HEX64(UINT64_C(0x83AA7E8000000000),
				ntp_ts_from_tai(0, 0));
	/* 2024-01-01, TAI−UTC = 37: the offset is subtracted, not added. */
	TEST_ASSERT_EQUAL_HEX64(((uint64_t)NTP_2024 << 32),
				ntp_ts_from_tai((UNIX_2024 + TAI_OFFSET) * NS,
						TAI_OFFSET));
	/* Ignoring the offset would shift the answer by exactly 37 s. */
	TEST_ASSERT_EQUAL_HEX64((((uint64_t)NTP_2024 + 37U) << 32),
				ntp_ts_from_tai((UNIX_2024 + TAI_OFFSET) * NS, 0));

	/* Fractions: half a second is the top bit; a nanosecond is 4 units of
	 * the 232 ps LSB; 999999999 ns must not round up into the next second. */
	TEST_ASSERT_EQUAL_HEX32(0x80000000U,
				(uint32_t)ntp_ts_from_tai(500000000, 0));
	TEST_ASSERT_EQUAL_HEX32(0x00000004U, (uint32_t)ntp_ts_from_tai(1, 0));
	TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFBU,
				(uint32_t)ntp_ts_from_tai(999999999, 0));

	/* Before the Unix epoch: C truncates division toward zero, so the
	 * negative case needs explicit normalisation or the fraction comes out
	 * negative and the second is off by one. */
	TEST_ASSERT_EQUAL_HEX64(UINT64_C(0x83AA7E7F00000000),
				ntp_ts_from_tai(-NS, 0));
	TEST_ASSERT_EQUAL_HEX64(UINT64_C(0x83AA7E7F80000000),
				ntp_ts_from_tai(-NS / 2, 0));
}

static void test_era_rollover(void)
{
	/*
	 * Era 0 ends at 2036-02-07T06:28:15Z and the next second is era 1
	 * second 0. RFC 5905 §6 puts only the low 32 bits on the wire, so the
	 * wrap is the required behaviour, not an overflow to guard against —
	 * the client resolves the era from its own approximate time.
	 */
	TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFU,
				(uint32_t)(ntp_ts_from_tai(UNIX_ERA0_LAST * NS, 0) >>
					   32));
	TEST_ASSERT_EQUAL_HEX32(0x00000000U,
				(uint32_t)(ntp_ts_from_tai((UNIX_ERA0_LAST + 1) * NS,
							   0) >>
					   32));
	TEST_ASSERT_EQUAL_HEX32(0x00000001U,
				(uint32_t)(ntp_ts_from_tai((UNIX_ERA0_LAST + 2) * NS,
							   0) >>
					   32));
	/* The fraction is unaffected by the wrap. */
	TEST_ASSERT_EQUAL_HEX64(UINT64_C(0xFFFFFFFF80000000),
				ntp_ts_from_tai(UNIX_ERA0_LAST * NS + NS / 2, 0));
}

static void test_short_format_saturates(void)
{
	TEST_ASSERT_EQUAL_HEX32(0U, ntp_short_from_q16(0U));
	TEST_ASSERT_EQUAL_HEX32(0x00010000U, ntp_short_from_q16(0x00010000U));
	TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFU, ntp_short_from_q16(0xFFFFFFFFU));
	/* Past the format maximum it clamps; wrapping would turn a day of
	 * accumulated holdover dispersion into a claim of microseconds. */
	TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFU,
				ntp_short_from_q16(UINT64_C(0x100000000)));
	TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFU,
				ntp_short_from_q16(UINT64_C(0xFFFFFFFFFFFF)));
}

static void test_defaults(void)
{
	ntp_quality_view_t q;
	ntp_cfg_t cfg;

	ntp_quality_view_default(&q);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)NTP_LI_UNSYNC, q.leap);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)NTP_STRATUM_UNSYNC, q.stratum);
	TEST_ASSERT_EQUAL_INT8(-20, q.precision);
	TEST_ASSERT_FALSE(q.synchronized);

	ntp_cfg_default(&cfg);
	TEST_ASSERT_EQUAL_UINT32(8U, cfg.client_rate);
	TEST_ASSERT_EQUAL_UINT32(16U, cfg.client_burst);
	TEST_ASSERT_TRUE(cfg.kod_on_limit);
	TEST_ASSERT_TRUE(cfg.interleave);

	/* NULL is a no-op rather than a crash: these are called from bring-up
	 * paths that have no way to report a failure. */
	ntp_quality_view_default(NULL);
	ntp_cfg_default(NULL);
}

/* ------------------------------------------------------------------- parse */

static void test_parse_header_fields(void)
{
	/* A complete request, laid out by hand per RFC 5905 §7.3. */
	static const uint8_t pkt[NTP_HDR_LEN] = {
		0xE3,                   /* LI 3, VN 4, mode 3 */
		0x00,                   /* stratum 0 */
		0x06,                   /* poll 6 */
		0xE9,                   /* precision -23 */
		0x00, 0x00, 0x01, 0x00, /* root delay 1/256 s */
		0x00, 0x00, 0x10, 0x00, /* root dispersion 1/16 s */
		0x49, 0x4E, 0x49, 0x54, /* refid "INIT" */
		0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, /* reference */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* origin */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* receive */
		0xE1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, /* transmit */
	};
	ntp_pkt_t p;

	TEST_ASSERT_EQUAL_INT(0, ntp_parse(pkt, sizeof(pkt), &p));
	TEST_ASSERT_EQUAL_UINT8(3U, p.li);
	TEST_ASSERT_EQUAL_UINT8(4U, p.vn);
	TEST_ASSERT_EQUAL_UINT8(3U, p.mode);
	TEST_ASSERT_EQUAL_UINT8(0U, p.stratum);
	TEST_ASSERT_EQUAL_UINT8(6U, p.poll);
	TEST_ASSERT_EQUAL_INT8(-23, p.precision);
	TEST_ASSERT_EQUAL_HEX32(0x00000100U, p.root_delay);
	TEST_ASSERT_EQUAL_HEX32(0x00001000U, p.root_disp);
	TEST_ASSERT_EQUAL_HEX32(NTP_REFID_INIT, p.refid);
	TEST_ASSERT_EQUAL_HEX64(UINT64_C(0x1122334455667788), p.ref_ts);
	TEST_ASSERT_EQUAL_HEX64(UINT64_C(0), p.org_ts);
	TEST_ASSERT_EQUAL_HEX64(UINT64_C(0xE100000000000001), p.xmt_ts);
	TEST_ASSERT_EQUAL_size_t(NTP_HDR_LEN, p.ext_off);
	TEST_ASSERT_EQUAL_size_t(0U, p.ext_len);
	TEST_ASSERT_EQUAL_size_t(0U, p.mac_len);

	/* The refid literals really are the four ASCII octets. */
	TEST_ASSERT_EQUAL_HEX32(0x47505300U, NTP_REFID_GPS);
	TEST_ASSERT_EQUAL_HEX32(0x52415445U, NTP_REFID_RATE);
	TEST_ASSERT_EQUAL_HEX32(0x4E54534EU, NTP_REFID_NTSN);
	TEST_ASSERT_EQUAL_HEX32(0x44454E59U, NTP_REFID_DENY);

	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_parse(NULL, sizeof(pkt), &p));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_parse(pkt, sizeof(pkt), NULL));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ntp_parse(pkt, NTP_HDR_LEN - 1U, &p));
}

static void test_parse_tail_disambiguation(void)
{
	uint8_t buf[NTP_HDR_LEN + 256U];
	ntp_pkt_t p;

	/* RFC 7822 reserves three trailing lengths for the MAC field, which is
	 * exactly why no extension field may be 4, 20 or 24 octets long. */
	memset(buf, 0, sizeof(buf));
	bytes_put_be32(&buf[NTP_HDR_LEN], 0xDEADBEEFU);

	TEST_ASSERT_EQUAL_INT(0, ntp_parse(buf, NTP_HDR_LEN + 4U, &p));
	TEST_ASSERT_EQUAL_size_t(4U, p.mac_len);
	TEST_ASSERT_EQUAL_size_t(NTP_HDR_LEN, p.mac_off);
	TEST_ASSERT_EQUAL_HEX32(0xDEADBEEFU, p.keyid);

	TEST_ASSERT_EQUAL_INT(0, ntp_parse(buf, NTP_HDR_LEN + 20U, &p));
	TEST_ASSERT_EQUAL_size_t(20U, p.mac_len);

	TEST_ASSERT_EQUAL_INT(0, ntp_parse(buf, NTP_HDR_LEN + 24U, &p));
	TEST_ASSERT_EQUAL_size_t(24U, p.mac_len);
	TEST_ASSERT_EQUAL_size_t(0U, p.ext_len);

	/* Anything else is walked as extension fields. */
	bytes_put_be16(&buf[NTP_HDR_LEN], 0x0104U);
	bytes_put_be16(&buf[NTP_HDR_LEN + 2U], 28U);
	TEST_ASSERT_EQUAL_INT(0, ntp_parse(buf, NTP_HDR_LEN + 28U, &p));
	TEST_ASSERT_EQUAL_size_t(28U, p.ext_len);
	TEST_ASSERT_EQUAL_size_t(0U, p.mac_len);

	/* Fields then a MAC. */
	bytes_put_be32(&buf[NTP_HDR_LEN + 28U], 0x01020304U);
	TEST_ASSERT_EQUAL_INT(0, ntp_parse(buf, NTP_HDR_LEN + 28U + 24U, &p));
	TEST_ASSERT_EQUAL_size_t(28U, p.ext_len);
	TEST_ASSERT_EQUAL_size_t(24U, p.mac_len);
	TEST_ASSERT_EQUAL_HEX32(0x01020304U, p.keyid);
}

static void test_parse_tolerates_a_broken_tail(void)
{
	uint8_t buf[NTP_HDR_LEN + 64U];
	ntp_pkt_t p;

	memset(buf, 0, sizeof(buf));
	bytes_put_be16(&buf[NTP_HDR_LEN], 0x0104U);
	bytes_put_be16(&buf[NTP_HDR_LEN + 2U], 28U);

	/* A second field that is unaligned, then one that overruns, then a
	 * two-octet runt. Each stops the walk without rejecting the packet:
	 * nothing past ext_len is ever echoed, so tolerance is free, and a
	 * client with a broken tail still gets the time. */
	bytes_put_be16(&buf[NTP_HDR_LEN + 28U], 0x0204U);
	bytes_put_be16(&buf[NTP_HDR_LEN + 30U], 13U);
	TEST_ASSERT_EQUAL_INT(0, ntp_parse(buf, NTP_HDR_LEN + 60U, &p));
	TEST_ASSERT_EQUAL_size_t(28U, p.ext_len);

	bytes_put_be16(&buf[NTP_HDR_LEN + 30U], 0x0100U);
	TEST_ASSERT_EQUAL_INT(0, ntp_parse(buf, NTP_HDR_LEN + 60U, &p));
	TEST_ASSERT_EQUAL_size_t(28U, p.ext_len);

	bytes_put_be16(&buf[NTP_HDR_LEN + 30U], 2U);
	TEST_ASSERT_EQUAL_INT(0, ntp_parse(buf, NTP_HDR_LEN + 60U, &p));
	TEST_ASSERT_EQUAL_size_t(28U, p.ext_len);

	TEST_ASSERT_EQUAL_INT(0, ntp_parse(buf, NTP_HDR_LEN + 30U, &p));
	TEST_ASSERT_EQUAL_size_t(28U, p.ext_len);
	TEST_ASSERT_EQUAL_size_t(0U, p.mac_len);
}

static void test_extension_field_iterator(void)
{
	uint8_t buf[NTP_HDR_LEN + 64U];
	ntp_pkt_t p;
	ntp_ef_iter_t it;
	ntp_ef_t ef;

	memset(buf, 0, sizeof(buf));
	bytes_put_be16(&buf[NTP_HDR_LEN], 0x0104U);
	bytes_put_be16(&buf[NTP_HDR_LEN + 2U], 28U);
	buf[NTP_HDR_LEN + 4U] = 0xAAU;
	bytes_put_be16(&buf[NTP_HDR_LEN + 28U], 0x0204U);
	bytes_put_be16(&buf[NTP_HDR_LEN + 30U], 32U);
	buf[NTP_HDR_LEN + 32U] = 0xBBU;

	TEST_ASSERT_EQUAL_INT(0, ntp_parse(buf, NTP_HDR_LEN + 60U, &p));
	TEST_ASSERT_EQUAL_size_t(60U, p.ext_len);

	TEST_ASSERT_EQUAL_INT(0, ntp_ef_iter_init(&it, buf, &p));
	TEST_ASSERT_EQUAL_INT(0, ntp_ef_iter_next(&it, &ef));
	TEST_ASSERT_EQUAL_HEX16(0x0104U, ef.type);
	TEST_ASSERT_EQUAL_UINT16(28U, ef.len);
	TEST_ASSERT_EQUAL_size_t(NTP_HDR_LEN, ef.offset);
	TEST_ASSERT_EQUAL_size_t(24U, ef.body_len);
	TEST_ASSERT_EQUAL_HEX8(0xAAU, ef.body[0]);

	TEST_ASSERT_EQUAL_INT(0, ntp_ef_iter_next(&it, &ef));
	TEST_ASSERT_EQUAL_HEX16(0x0204U, ef.type);
	TEST_ASSERT_EQUAL_UINT16(32U, ef.len);
	TEST_ASSERT_EQUAL_HEX8(0xBBU, ef.body[0]);

	TEST_ASSERT_EQUAL_INT(-ENOENT, ntp_ef_iter_next(&it, &ef));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_ef_iter_init(NULL, buf, &p));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_ef_iter_init(&it, NULL, &p));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_ef_iter_init(&it, buf, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_ef_iter_next(NULL, &ef));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_ef_iter_next(&it, NULL));
}

/* --------------------------------------------------------------- responses */

static void test_response_byte_vector(void)
{
	/*
	 * The whole response, written out by hand. Timestamps are the era-0
	 * literals derived at the top of this file, not values fetched from the
	 * code under test.
	 */
	static const uint8_t want[NTP_HDR_LEN] = {
		0x24,                   /* LI 0, VN 4, mode 4 (server) */
		0x01,                   /* stratum 1 */
		0x06,                   /* poll, echoed from the request */
		0xEC,                   /* precision -20 */
		0x00, 0x00, 0x01, 0x00, /* root delay */
		0x00, 0x00, 0x10, 0x00, /* root dispersion */
		0x47, 0x50, 0x53, 0x00, /* refid "GPS\0" */
		0xE9, 0x3C, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, /* reference */
		0xE1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, /* origin */
		0xE9, 0x3C, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, /* receive */
		0xE9, 0x3C, 0x7F, 0x00, 0x00, 0x01, 0x4F, 0x8B, /* transmit */
	};
	uint8_t req[NTP_HDR_LEN];
	uint8_t out[NTP_PKT_MAX];
	ntp_quality_view_t q;
	ntp_rx_t rx;
	ntp_result_t res;
	ntp_stats_t st;
	size_t len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U,
				  UINT64_C(0xE100000000000001), 0U);

	good_quality(&q);
	fill_rx(&rx, req, len, 0x0A000001U, 0);

	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_RESPOND, res.action);
	TEST_ASSERT_EQUAL_size_t(NTP_HDR_LEN, res.len);
	TEST_ASSERT_FALSE(res.interleaved);
	TEST_ASSERT_FALSE(res.authenticated);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want, out, sizeof(want));

	TEST_ASSERT_EQUAL_INT(0, ntp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.rx);
	TEST_ASSERT_EQUAL_UINT64(1U, st.served);
	TEST_ASSERT_EQUAL_UINT64(0U, st.dropped);
}

static void test_response_maps_the_quality_view(void)
{
	uint8_t req[NTP_HDR_LEN];
	uint8_t out[NTP_PKT_MAX];
	ntp_quality_view_t q;
	ntp_rx_t rx;
	ntp_result_t res;
	size_t len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 10U, 1U, 0U);

	/* A pending leap second and a secondary stratum with an upstream refid. */
	good_quality(&q);
	q.leap = (uint8_t)NTP_LI_ADD;
	q.stratum = 2U;
	q.refid = 0xC0A80001U; /* an upstream IPv4 address, as RFC 5905 §7.3 has it */
	q.precision = -18;
	q.root_delay_q16 = 0x00023456U;
	q.root_disp_q16 = 0x000789ABU;
	fill_rx(&rx, req, len, 1U, 1000);

	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_UINT8(0x64U, out[0]); /* LI 1, VN 4, mode 4 */
	TEST_ASSERT_EQUAL_UINT8(2U, out[1]);
	TEST_ASSERT_EQUAL_UINT8(10U, out[2]);
	TEST_ASSERT_EQUAL_HEX8(0xEEU, out[3]); /* -18 */
	TEST_ASSERT_EQUAL_HEX32(0x00023456U, bytes_get_be32(&out[4]));
	TEST_ASSERT_EQUAL_HEX32(0x000789ABU, bytes_get_be32(&out[8]));
	TEST_ASSERT_EQUAL_HEX32(0xC0A80001U, bytes_get_be32(&out[12]));

	/* Stratum 1 with an explicit refid keeps it: a future GNSS/other
	 * distinction lives in the quality block, not here. */
	q.stratum = 1U;
	q.refid = NTP_REFID_MAKE('G', 'N', 'S', 'S');
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_HEX32(NTP_REFID_MAKE('G', 'N', 'S', 'S'),
				bytes_get_be32(&out[12]));

	/* Unsynchronised overrides everything the block claims: an unlocked
	 * server that advertised stratum 1 would be worse than useless. */
	q.synchronized = false;
	q.stratum = 1U;
	q.leap = (uint8_t)NTP_LI_NONE;
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_UINT8(0xE4U, out[0]); /* LI 3, VN 4, mode 4 */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)NTP_STRATUM_UNSYNC, out[1]);
	TEST_ASSERT_EQUAL_HEX32(NTP_REFID_INIT, bytes_get_be32(&out[12]));

	/* A NULL quality view is the same thing: nothing is known. */
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, NULL, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_UINT8(0xE4U, out[0]);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)NTP_STRATUM_UNSYNC, out[1]);

	/* A reference timestamp of zero falls back to the receive time, so the
	 * field is never a bare 1900-01-01. */
	good_quality(&q);
	q.ref_tai_ns = 0;
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_HEX64(bytes_get_be64(&out[32]), bytes_get_be64(&out[16]));

	/* Version 3 clients get version 3 replies. */
	len = make_request(req, 3U, (uint8_t)NTP_MODE_CLIENT, 6U, 1U, 0U);
	fill_rx(&rx, req, len, 1U, 1000);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_UINT8(0x1CU, out[0]); /* LI 0, VN 3, mode 4 */
}

static void test_serve_unsync_can_be_disabled(void)
{
	uint8_t req[NTP_HDR_LEN];
	uint8_t out[NTP_PKT_MAX];
	ntp_cfg_t cfg;
	ntp_rx_t rx;
	ntp_result_t res;
	size_t len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U, 1U, 0U);

	ntp_cfg_default(&cfg);
	cfg.serve_unsync = false;
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, &cfg, &g_port, 0));
	fill_rx(&rx, req, len, 1U, 1000);

	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, NULL, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_IGNORE, res.action);
	TEST_ASSERT_EQUAL_INT(NTP_DROP_UNSYNC, res.drop);
}

static void test_unserved_modes_and_versions(void)
{
	uint8_t req[NTP_HDR_LEN];
	uint8_t out[NTP_PKT_MAX];
	ntp_quality_view_t q;
	ntp_rx_t rx;
	ntp_result_t res;
	ntp_stats_t st;
	/* Modes 6 and 7 are the reflection vectors (ntpq control, ntpdc
	 * monlist); the rest are peer modes a grandmaster has no use for. */
	static const uint8_t modes[] = { 0U, 1U, 2U, 4U, 5U, 6U, 7U };
	static const uint8_t versions[] = { 0U, 1U, 2U, 5U, 6U, 7U };

	good_quality(&q);

	for (size_t i = 0U; i < ARRAY_LEN(modes); i++) {
		size_t len = make_request(req, 4U, modes[i], 6U, 1U, 0U);

		fill_rx(&rx, req, len, 1U, 1000);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
		TEST_ASSERT_EQUAL_INT(NTP_ACT_IGNORE, res.action);
		TEST_ASSERT_EQUAL_INT(NTP_DROP_MODE, res.drop);
		TEST_ASSERT_EQUAL_size_t(0U, res.len);
	}

	for (size_t i = 0U; i < ARRAY_LEN(versions); i++) {
		size_t len = make_request(req, versions[i],
					  (uint8_t)NTP_MODE_CLIENT, 6U, 1U, 0U);

		fill_rx(&rx, req, len, 1U, 1000);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
		TEST_ASSERT_EQUAL_INT(NTP_ACT_IGNORE, res.action);
		TEST_ASSERT_EQUAL_INT(NTP_DROP_VERSION, res.drop);
	}

	TEST_ASSERT_EQUAL_INT(0, ntp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(ARRAY_LEN(modes) + ARRAY_LEN(versions), st.ignored);
	TEST_ASSERT_EQUAL_UINT64(0U, st.dropped);
	TEST_ASSERT_EQUAL_UINT64(0U, st.served);
}

static void test_malformed_and_argument_errors(void)
{
	uint8_t req[NTP_PKT_MAX + 8U];
	uint8_t out[NTP_PKT_MAX];
	ntp_quality_view_t q;
	ntp_rx_t rx;
	ntp_result_t res;

	good_quality(&q);
	memset(req, 0, sizeof(req));
	(void)make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U, 1U, 0U);

	fill_rx(&rx, req, NTP_HDR_LEN - 1U, 1U, 1000);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_DROP_SHORT, res.drop);

	fill_rx(&rx, req, NTP_PKT_MAX + 1U, 1U, 1000);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_DROP_SHORT, res.drop);

	fill_rx(&rx, req, NTP_HDR_LEN, 1U, 1000);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    NTP_HDR_LEN - 1U, &res));
	TEST_ASSERT_EQUAL_INT(NTP_DROP_NOSPACE, res.drop);

	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_handle_request(NULL, &rx, &q, out,
							  sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_handle_request(&g_ctx, NULL, &q, out,
							  sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_handle_request(&g_ctx, &rx, &q, NULL,
							  sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_handle_request(&g_ctx, &rx, &q, out,
							  sizeof(out), NULL));
	rx.pkt = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_handle_request(&g_ctx, &rx, &q, out,
							  sizeof(out), &res));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_init(NULL, NULL, NULL, 0));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_stats_get(NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_stats_reset(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_set_ext_hook(NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_tx_complete(NULL, 0U, 0U));
}

/* ------------------------------------------------------------ rate limiting */

static void test_rate_limit_burst_throttle_recover(void)
{
	uint8_t req[NTP_HDR_LEN];
	uint8_t out[NTP_PKT_MAX];
	ntp_cfg_t cfg;
	ntp_quality_view_t q;
	ntp_rx_t rx;
	ntp_result_t res;
	ntp_stats_t st;
	size_t len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U, 1U, 0U);
	unsigned served = 0U;

	ntp_cfg_default(&cfg);
	cfg.client_rate = 8U;
	cfg.client_burst = 16U;
	cfg.global_rate = 0U; /* isolate the per-client bucket */
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, &cfg, &g_port, 0));
	good_quality(&q);

	/* A new client starts with a full bucket, so the configured burst is
	 * what it actually gets. */
	for (unsigned i = 0U; i < 20U; i++) {
		fill_rx(&rx, req, len, 0x7FU, 0);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
		if (res.action == NTP_ACT_RESPOND) {
			served++;
		}
	}
	TEST_ASSERT_EQUAL_UINT(16U, served);

	/* Over the limit: one Kiss-o'-Death, then silence. Answering every
	 * excess request would make the server a full-rate reflector for a
	 * spoofed source. */
	TEST_ASSERT_EQUAL_INT(0, ntp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.kod);
	TEST_ASSERT_EQUAL_UINT64(4U, st.rate_limited);
	TEST_ASSERT_EQUAL_UINT64(3U, st.dropped);

	/* One second later the bucket holds eight more. */
	served = 0U;
	for (unsigned i = 0U; i < 12U; i++) {
		fill_rx(&rx, req, len, 0x7FU, 1000);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
		if (res.action == NTP_ACT_RESPOND) {
			served++;
		}
	}
	TEST_ASSERT_EQUAL_UINT(8U, served);

	/* Sub-token accumulation: 125 ms is exactly one token at 8 req/s, so
	 * the bucket refills smoothly rather than in whole-second steps. */
	fill_rx(&rx, req, len, 0x7FU, 1124);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_NOT_EQUAL(NTP_ACT_RESPOND, res.action);
	fill_rx(&rx, req, len, 0x7FU, 1125);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_RESPOND, res.action);

	/* A different client is unaffected by the first one's exhaustion. */
	fill_rx(&rx, req, len, 0x80U, 1125);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_RESPOND, res.action);
}

static void test_kod_rate_bytes(void)
{
	uint8_t req[NTP_HDR_LEN];
	uint8_t out[NTP_PKT_MAX];
	ntp_cfg_t cfg;
	ntp_quality_view_t q;
	ntp_rx_t rx;
	ntp_result_t res;
	size_t len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 9U,
				  UINT64_C(0xAABBCCDD00112233), 0U);

	ntp_cfg_default(&cfg);
	cfg.client_rate = 1U;
	cfg.client_burst = 1U;
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, &cfg, &g_port, 0));
	good_quality(&q);

	fill_rx(&rx, req, len, 5U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_RESPOND, res.action);

	fill_rx(&rx, req, len, 5U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_KOD, res.action);
	TEST_ASSERT_EQUAL_size_t(NTP_HDR_LEN, res.len);

	/* RFC 5905 §7.4: LI 3, stratum 0, a four-octet ASCII kiss code. The
	 * origin timestamp still echoes so the client can correlate, and the
	 * poll is echoed so it knows which of its requests was refused. */
	TEST_ASSERT_EQUAL_HEX8(0xE4U, out[0]);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)NTP_STRATUM_KOD, out[1]);
	TEST_ASSERT_EQUAL_UINT8(9U, out[2]);
	TEST_ASSERT_EQUAL_HEX8(0x52U, out[12]); /* 'R' */
	TEST_ASSERT_EQUAL_HEX8(0x41U, out[13]); /* 'A' */
	TEST_ASSERT_EQUAL_HEX8(0x54U, out[14]); /* 'T' */
	TEST_ASSERT_EQUAL_HEX8(0x45U, out[15]); /* 'E' */
	TEST_ASSERT_EQUAL_HEX64(UINT64_C(0xAABBCCDD00112233),
				bytes_get_be64(&out[24]));

	/* A Kiss-o'-Death never arms interleaved mode: it carries no timestamps
	 * worth remembering. */
	TEST_ASSERT_EQUAL_INT(-ENOENT, ntp_tx_complete(&g_ctx, 5U, 1U));
}

static void test_kod_can_be_disabled_and_global_bucket_bites(void)
{
	uint8_t req[NTP_HDR_LEN];
	uint8_t out[NTP_PKT_MAX];
	ntp_cfg_t cfg;
	ntp_quality_view_t q;
	ntp_rx_t rx;
	ntp_result_t res;
	ntp_stats_t st;
	size_t len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U, 1U, 0U);

	ntp_cfg_default(&cfg);
	cfg.client_rate = 1U;
	cfg.client_burst = 1U;
	cfg.kod_on_limit = false;
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, &cfg, &g_port, 0));
	good_quality(&q);

	fill_rx(&rx, req, len, 5U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	fill_rx(&rx, req, len, 5U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_IGNORE, res.action);
	TEST_ASSERT_EQUAL_INT(NTP_DROP_RATE, res.drop);
	TEST_ASSERT_EQUAL_INT(0, ntp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(0U, st.kod);

	/* The aggregate bucket limits the whole server, so distinct clients
	 * cannot walk past it one at a time. */
	ntp_cfg_default(&cfg);
	cfg.client_rate = 0U; /* per-client disabled: only the global one left */
	cfg.global_rate = 10U;
	cfg.global_burst = 3U;
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, &cfg, &g_port, 0));

	for (unsigned i = 0U; i < 3U; i++) {
		fill_rx(&rx, req, len, 100U + i, 0);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
		TEST_ASSERT_EQUAL_INT(NTP_ACT_RESPOND, res.action);
	}
	fill_rx(&rx, req, len, 200U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_KOD, res.action);

	/* With both buckets off, nothing is ever limited. */
	ntp_cfg_default(&cfg);
	cfg.client_rate = 0U;
	cfg.global_rate = 0U;
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, &cfg, &g_port, 0));
	for (unsigned i = 0U; i < 64U; i++) {
		fill_rx(&rx, req, len, 7U, 0);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
		TEST_ASSERT_EQUAL_INT(NTP_ACT_RESPOND, res.action);
	}
}

static void test_kod_is_damped(void)
{
	uint8_t req[NTP_HDR_LEN];
	uint8_t out[NTP_PKT_MAX];
	ntp_cfg_t cfg;
	ntp_quality_view_t q;
	ntp_rx_t rx;
	ntp_result_t res;
	ntp_stats_t st;
	size_t len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U, 1U, 0U);

	ntp_cfg_default(&cfg);
	cfg.client_rate = 1U;
	cfg.client_burst = 1U;
	cfg.kod_min_interval_ms = 500U;
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, &cfg, &g_port, 0));
	good_quality(&q);

	/* Drain the bucket, then hammer for a second of virtual time. */
	for (unsigned i = 0U; i < 50U; i++) {
		fill_rx(&rx, req, len, 9U, (int64_t)(i * 10U));
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
	}
	TEST_ASSERT_EQUAL_INT(0, ntp_stats_get(&g_ctx, &st));
	/* 500 ms of damping across 490 ms of flooding: one kiss, not dozens. */
	TEST_ASSERT_EQUAL_UINT64(1U, st.kod);
	TEST_ASSERT_TRUE(st.rate_limited > 40U);

	fill_rx(&rx, req, len, 9U, 600);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_KOD, res.action);
}

static void test_client_table_evicts_the_stalest(void)
{
	uint8_t req[NTP_HDR_LEN];
	uint8_t out[NTP_PKT_MAX];
	ntp_cfg_t cfg;
	ntp_quality_view_t q;
	ntp_rx_t rx;
	ntp_result_t res;
	size_t len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U, 1U, 0U);
	uint32_t victim = 0x11110000U;

	ntp_cfg_default(&cfg);
	cfg.client_rate = 1U;
	cfg.client_burst = 1U;
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, &cfg, &g_port, 0));
	good_quality(&q);

	/* Spend the victim's single token. */
	fill_rx(&rx, req, len, victim, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_RESPOND, res.action);
	fill_rx(&rx, req, len, victim, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_NOT_EQUAL(NTP_ACT_RESPOND, res.action);

	/*
	 * Flood the table with distinct sources. The table is bounded and
	 * set-associative, so the victim's entry is eventually reused and it
	 * gets a fresh bucket. That is the accepted cost of a fixed-size table:
	 * a flooder can buy an honest client one extra burst, but it cannot
	 * make the server allocate, and the honest client is never locked out.
	 */
	for (uint32_t i = 0U; i < NTP_CLIENT_SLOTS * 8U; i++) {
		fill_rx(&rx, req, len, 0x90000000U + i, 1);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
	}

	fill_rx(&rx, req, len, victim, 1);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_RESPOND, res.action);

	/* Eviction also forgets the interleave state, so an evicted client
	 * falls back to basic mode rather than being told a stale timestamp. */
	TEST_ASSERT_EQUAL_INT(0, ntp_tx_complete(&g_ctx, victim, 0x1234U));
	for (uint32_t i = 0U; i < NTP_CLIENT_SLOTS * 8U; i++) {
		fill_rx(&rx, req, len, 0xA0000000U + i, 2);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
	}
	TEST_ASSERT_EQUAL_INT(-ENOENT, ntp_tx_complete(&g_ctx, victim, 0x1234U));
}

/* --------------------------------------------------------- interleaved mode */

static void test_interleaved_handshake(void)
{
	uint8_t req[NTP_HDR_LEN];
	uint8_t out[NTP_PKT_MAX];
	ntp_quality_view_t q;
	ntp_rx_t rx;
	ntp_result_t res;
	ntp_stats_t st;
	const uint32_t id = 0x2A2A2A2AU;
	uint64_t rx1;
	uint64_t xmt_field1;
	const uint64_t hw_tx1 = UINT64_C(0xE93C7F0000000000) + 0x3333U;
	size_t len;

	good_quality(&q);

	/* Packet 1 — an ordinary basic-mode exchange. */
	len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U,
			   UINT64_C(0xE1000000AAAA0001), 0U);
	fill_rx(&rx, req, len, id, 1000);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_RESPOND, res.action);
	TEST_ASSERT_FALSE(res.interleaved);
	rx1 = bytes_get_be64(&out[32]);
	xmt_field1 = bytes_get_be64(&out[40]);
	TEST_ASSERT_EQUAL_HEX64(res.xmt, xmt_field1);

	/* Until the driver reports the hardware transmit timestamp, the server
	 * has nothing precise to offer and must stay in basic mode. */
	len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U,
			   UINT64_C(0xE1000000AAAA0002), xmt_field1);
	fill_rx(&rx, req, len, id, 1001);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_FALSE(res.interleaved);

	/* Now report it. */
	TEST_ASSERT_EQUAL_INT(0, ntp_tx_complete(&g_ctx, id, hw_tx1));
	TEST_ASSERT_EQUAL_INT(-ENOENT, ntp_tx_complete(&g_ctx, id, hw_tx1));
	TEST_ASSERT_EQUAL_INT(-ENOENT, ntp_tx_complete(&g_ctx, 0xDEADU, hw_tx1));

	/*
	 * Packet 3 — the client echoes the transmit field it was given as its
	 * origin, which is how it asks for interleaved mode. The reply carries
	 * the *previous* exchange's receive timestamp and the *measured*
	 * transmit timestamp of the previous response — the value the server
	 * could not know when it built it. The origin is still the client's own
	 * transmit timestamp, verbatim.
	 */
	{
		/* xmt_field1 was superseded by the packet-2 exchange; use what
		 * the server last committed. */
		uint64_t handle = bytes_get_be64(&out[40]);

		len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U,
				   UINT64_C(0xE1000000AAAA0003), handle);
		fill_rx(&rx, req, len, id, 1002);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
		TEST_ASSERT_TRUE(res.interleaved);
		TEST_ASSERT_EQUAL_HEX64(UINT64_C(0xE1000000AAAA0003),
					bytes_get_be64(&out[24]));
		TEST_ASSERT_EQUAL_HEX64(hw_tx1, bytes_get_be64(&out[40]));
		/* The receive field is the earlier request's, not this one's. */
		TEST_ASSERT_NOT_EQUAL(rx1, bytes_get_be64(&out[32]));
	}

	TEST_ASSERT_EQUAL_INT(0, ntp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.interleaved);
	TEST_ASSERT_EQUAL_UINT64(3U, st.served);

	/* A stale or invented origin gets a basic response, not a guess. */
	len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U,
			   UINT64_C(0xE1000000AAAA0004), UINT64_C(0x1111111111111111));
	fill_rx(&rx, req, len, id, 1003);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_FALSE(res.interleaved);
}

static void test_interleave_can_be_disabled(void)
{
	uint8_t req[NTP_HDR_LEN];
	uint8_t out[NTP_PKT_MAX];
	ntp_cfg_t cfg;
	ntp_quality_view_t q;
	ntp_rx_t rx;
	ntp_result_t res;
	const uint32_t id = 3U;
	uint64_t handle;
	size_t len;

	ntp_cfg_default(&cfg);
	cfg.interleave = false;
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, &cfg, &g_port, 0));
	good_quality(&q);

	len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U, 1U, 0U);
	fill_rx(&rx, req, len, id, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	handle = bytes_get_be64(&out[40]);
	TEST_ASSERT_EQUAL_INT(0, ntp_tx_complete(&g_ctx, id, 0x999U));

	len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U, 2U, handle);
	fill_rx(&rx, req, len, id, 1);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_FALSE(res.interleaved);
	TEST_ASSERT_NOT_EQUAL(0x999U, bytes_get_be64(&out[40]));
}

/* ------------------------------------------------------- symmetric-key auth */

#define TEST_KEYID 42U
static const uint8_t test_key[20] = { 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
				      0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
				      0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b };

/** Append a keyid + truncated HMAC-SHA-256 MAC field, as a client would. */
static size_t append_mac(uint8_t *buf, size_t len, uint32_t keyid,
			 const uint8_t *key, size_t key_len)
{
	uint8_t digest[32];

	bytes_put_be32(&buf[len], keyid);
	host_hmac_sha256(key, key_len, buf, len, digest);
	memcpy(&buf[len + 4U], digest, NTP_MAC_DIGEST_LEN);
	return len + NTP_MAC_FIELD_LEN;
}

static void test_symmetric_mac(void)
{
	uint8_t req[NTP_HDR_LEN + 64U];
	uint8_t out[NTP_PKT_MAX];
	uint8_t expect[32];
	ntp_quality_view_t q;
	ntp_rx_t rx;
	ntp_result_t res;
	ntp_stats_t st;
	ntp_cfg_t cfg;
	uint64_t expect_fail = 0U;
	size_t len;

	/* This test sends hundreds of requests from one source; the rate
	 * limiter has its own test and would otherwise mask the auth results. */
	ntp_cfg_default(&cfg);
	cfg.client_rate = 0U;
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, &cfg, &g_port, 0));

	good_quality(&q);
	TEST_ASSERT_EQUAL_INT(0, ntp_key_set(&g_ctx, TEST_KEYID, test_key,
					     sizeof(test_key)));

	len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U, 1U, 0U);
	len = append_mac(req, len, TEST_KEYID, test_key, sizeof(test_key));
	TEST_ASSERT_EQUAL_size_t(NTP_HDR_LEN + 24U, len);

	fill_rx(&rx, req, len, 1U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_RESPOND, res.action);
	TEST_ASSERT_TRUE(res.authenticated);
	TEST_ASSERT_EQUAL_size_t(NTP_HDR_LEN + NTP_MAC_FIELD_LEN, res.len);

	/* The response MAC is the same construction over the response. */
	TEST_ASSERT_EQUAL_HEX32(TEST_KEYID, bytes_get_be32(&out[NTP_HDR_LEN]));
	host_hmac_sha256(test_key, sizeof(test_key), out, NTP_HDR_LEN, expect);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(expect, &out[NTP_HDR_LEN + 4U],
				     NTP_MAC_DIGEST_LEN);

	/*
	 * Every octet of the request is inside the authenticated region — the
	 * whole header, the key id, and the digest itself — so flipping any bit
	 * anywhere must fail. Fields the response merely echoes, like the poll
	 * and the origin timestamp, are covered too.
	 */
	for (size_t i = 0U; i < len; i++) {
		uint8_t saved = req[i];

		req[i] ^= 0x01U;
		fill_rx(&rx, req, len, 1U, 0);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
		if (i == 0U) {
			/* Bit 0 of octet 0 is a mode bit: mode 3 becomes 2, so
			 * this one is rejected before authentication. */
			TEST_ASSERT_EQUAL_INT(NTP_DROP_MODE, res.drop);
		} else {
			TEST_ASSERT_EQUAL_INT(NTP_DROP_AUTH, res.drop);
			expect_fail++;
		}
		req[i] = saved;
	}

	/* An unknown key id. */
	len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U, 1U, 0U);
	len = append_mac(req, len, TEST_KEYID + 1U, test_key, sizeof(test_key));
	fill_rx(&rx, req, len, 1U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_DROP_AUTH, res.drop);
	expect_fail++;

	/* A 16-octet digest is an MD5 MAC. Recognised only so it is counted as
	 * an authentication failure instead of mis-parsed as an extension
	 * field; MD5 is not implemented and will not be. */
	len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U, 1U, 0U);
	bytes_put_be32(&req[len], TEST_KEYID);
	memset(&req[len + 4U], 0x55U, 16U);
	fill_rx(&rx, req, len + 20U, 1U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_DROP_AUTH, res.drop);
	expect_fail++;

	/* A bare key id is a client-sent crypto-NAK: nothing to verify. */
	fill_rx(&rx, req, NTP_HDR_LEN + 4U, 1U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_DROP_AUTH, res.drop);
	expect_fail++;

	TEST_ASSERT_EQUAL_INT(0, ntp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(expect_fail, st.auth_fail);
	/* Authentication failures are drops; the one mode-bit flip is counted
	 * as ignored instead, because it never reached authentication. */
	TEST_ASSERT_EQUAL_UINT64(expect_fail, st.dropped);
	TEST_ASSERT_EQUAL_UINT64(1U, st.ignored);
	TEST_ASSERT_EQUAL_UINT64(1U, st.served);
}

static void test_mac_key_table(void)
{
	uint8_t key[NTP_MAC_KEY_MAX + 1U];

	memset(key, 0x77U, sizeof(key));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_key_set(NULL, 1U, key, 8U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_key_set(&g_ctx, 1U, NULL, 8U));
	/* RFC 5905 §7.5 reserves key id 0 for the crypto-NAK. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_key_set(&g_ctx, 0U, key, 8U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_key_set(&g_ctx, 1U, key, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_key_set(&g_ctx, 1U, key, sizeof(key)));

	for (uint32_t i = 1U; i <= NTP_MAC_KEYS; i++) {
		TEST_ASSERT_EQUAL_INT(0, ntp_key_set(&g_ctx, i, key, 16U));
	}
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ntp_key_set(&g_ctx, NTP_MAC_KEYS + 1U, key,
						   16U));
	/* Replacing an existing id reuses its slot. */
	TEST_ASSERT_EQUAL_INT(0, ntp_key_set(&g_ctx, 1U, key, 32U));

	TEST_ASSERT_EQUAL_INT(0, ntp_key_clear(&g_ctx, 1U));
	TEST_ASSERT_EQUAL_INT(-ENOENT, ntp_key_clear(&g_ctx, 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntp_key_clear(NULL, 1U));
	TEST_ASSERT_EQUAL_INT(0, ntp_key_set(&g_ctx, 1U, key, 16U));
}

static void test_mac_without_a_crypto_port(void)
{
	uint8_t req[NTP_HDR_LEN + 64U];
	uint8_t out[NTP_PKT_MAX];
	ntp_quality_view_t q;
	ntp_rx_t rx;
	ntp_result_t res;
	size_t len;

	/* A server built without crypto still serves unauthenticated clients,
	 * and refuses authenticated ones rather than answering them unsigned. */
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, NULL, NULL, 0));
	good_quality(&q);

	len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U, 1U, 0U);
	fill_rx(&rx, req, len, 1U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_RESPOND, res.action);

	TEST_ASSERT_EQUAL_INT(0, ntp_key_set(&g_ctx, TEST_KEYID, test_key,
					     sizeof(test_key)));
	len = append_mac(req, len, TEST_KEYID, test_key, sizeof(test_key));
	fill_rx(&rx, req, len, 1U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_DROP_AUTH, res.drop);

	/* A crypto port that fails while signing the response must not emit an
	 * unauthenticated reply to an authenticated request. */
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, NULL, &g_port, 0));
	TEST_ASSERT_EQUAL_INT(0, ntp_key_set(&g_ctx, TEST_KEYID, test_key,
					     sizeof(test_key)));
	g_hc.fail_hmac_in = 2U; /* verify succeeds, append fails */
	fill_rx(&rx, req, len, 1U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_DROP_INTERNAL, res.drop);

	/* And one that fails while verifying is an authentication failure. */
	g_hc.fail_hmac_in = 1U;
	fill_rx(&rx, req, len, 1U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_DROP_AUTH, res.drop);

	/* Too small an output buffer for the MAC the client asked for. */
	fill_rx(&rx, req, len, 1U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    NTP_HDR_LEN + 8U, &res));
	TEST_ASSERT_EQUAL_INT(NTP_DROP_NOSPACE, res.drop);
}

/* -------------------------------------------------------- extension hook */

typedef struct {
	unsigned calls;
	size_t append;   /* octets of filler field to append, 0 for none */
	uint32_t kod;    /* kiss code to request, 0 for none */
	int rc;          /* return value */
	bool overrun;    /* report a length past the capacity given */
	size_t last_cap; /* capacity the hook was handed */
	size_t last_req_len;
} hook_state_t;

static int test_hook(void *ctx, const uint8_t *req, size_t req_len, uint8_t *pkt,
		     size_t *len, size_t cap, uint32_t *kod_refid)
{
	hook_state_t *h = (hook_state_t *)ctx;

	h->calls++;
	h->last_cap = cap;
	h->last_req_len = req_len;
	TEST_ASSERT_NOT_NULL(req);
	TEST_ASSERT_EQUAL_size_t(NTP_HDR_LEN, *len);

	if (h->rc != 0) {
		return h->rc;
	}
	if (h->overrun) {
		*len = cap + 1U;
		return 0;
	}
	if (h->append != 0U) {
		TEST_ASSERT_TRUE(*len + h->append <= cap);
		bytes_put_be16(&pkt[*len], 0x0104U);
		bytes_put_be16(&pkt[*len + 2U], (uint16_t)h->append);
		memset(&pkt[*len + 4U], 0xC5U, h->append - 4U);
		*len += h->append;
	}
	if (h->kod != 0U) {
		*kod_refid = h->kod;
	}
	return 0;
}

static void test_extension_hook(void)
{
	uint8_t req[NTP_PKT_MAX];
	uint8_t out[NTP_PKT_MAX];
	ntp_quality_view_t q;
	ntp_rx_t rx;
	ntp_result_t res;
	ntp_ext_hook_t hook;
	hook_state_t h;
	size_t len;

	memset(&h, 0, sizeof(h));
	hook.build = test_hook;
	hook.ctx = &h;
	TEST_ASSERT_EQUAL_INT(0, ntp_set_ext_hook(&g_ctx, &hook));
	good_quality(&q);

	/* A request padded out so the hook has room to work in. */
	memset(req, 0, sizeof(req));
	len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U, 1U, 0U);
	bytes_put_be16(&req[len], 0x0104U);
	bytes_put_be16(&req[len + 2U], 200U);
	len += 200U;

	h.append = 40U;
	fill_rx(&rx, req, len, 1U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_RESPOND, res.action);
	TEST_ASSERT_EQUAL_size_t(NTP_HDR_LEN + 40U, res.len);
	TEST_ASSERT_EQUAL_UINT(1U, h.calls);
	TEST_ASSERT_EQUAL_size_t(len, h.last_req_len);
	/* Anti-amplification reaches the hook as a capacity, before it builds
	 * anything — not as a truncation afterwards. */
	TEST_ASSERT_EQUAL_size_t(len, h.last_cap);

	/* The hook can turn the response into a Kiss-o'-Death, which is how the
	 * NTS NAK of RFC 8915 §5.7 is produced without an ntp↔nts dependency. */
	h.kod = NTP_REFID_NTSN;
	fill_rx(&rx, req, len, 2U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_KOD, res.action);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)NTP_STRATUM_KOD, out[1]);
	TEST_ASSERT_EQUAL_HEX8(0xE4U, out[0]);
	TEST_ASSERT_EQUAL_HEX32(NTP_REFID_NTSN, bytes_get_be32(&out[12]));
	/* The appended fields survive the rewrite. */
	TEST_ASSERT_EQUAL_size_t(NTP_HDR_LEN + 40U, res.len);
	TEST_ASSERT_EQUAL_HEX16(0x0104U, bytes_get_be16(&out[NTP_HDR_LEN]));

	/* A hook that declines drops the request. */
	h.kod = 0U;
	h.rc = -EBADMSG;
	fill_rx(&rx, req, len, 3U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_IGNORE, res.action);
	TEST_ASSERT_EQUAL_INT(NTP_DROP_EXT, res.drop);

	/* A hook that overruns its capacity has corrupted the buffer: there is
	 * nothing safe to send, so nothing is. */
	h.rc = 0;
	h.overrun = true;
	fill_rx(&rx, req, len, 4U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_DROP_EXT, res.drop);

	/* A Kiss-o'-Death for rate never reaches the hook. */
	h.overrun = false;
	h.append = 0U;
	{
		ntp_cfg_t cfg;
		unsigned before;

		ntp_cfg_default(&cfg);
		cfg.client_rate = 1U;
		cfg.client_burst = 1U;
		TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, &cfg, &g_port, 0));
		TEST_ASSERT_EQUAL_INT(0, ntp_set_ext_hook(&g_ctx, &hook));
		fill_rx(&rx, req, len, 5U, 0);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
		before = h.calls;
		fill_rx(&rx, req, len, 5U, 0);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
		TEST_ASSERT_EQUAL_INT(NTP_ACT_KOD, res.action);
		TEST_ASSERT_EQUAL_UINT(before, h.calls);
	}

	/* Removing the hook restores plain behaviour. */
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, NULL, &g_port, 0));
	TEST_ASSERT_EQUAL_INT(0, ntp_set_ext_hook(&g_ctx, NULL));
	fill_rx(&rx, req, len, 6U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_size_t(NTP_HDR_LEN, res.len);
}

static void test_response_never_exceeds_request(void)
{
	uint8_t req[NTP_PKT_MAX];
	uint8_t out[NTP_PKT_MAX];
	ntp_quality_view_t q;
	ntp_rx_t rx;
	ntp_result_t res;
	ntp_ext_hook_t hook;
	hook_state_t h;

	/* A hook that always wants far more room than any request can justify.
	 * The budget must stop it, whatever the request size. */
	memset(&h, 0, sizeof(h));
	h.append = 0U;
	hook.build = test_hook;
	hook.ctx = &h;
	TEST_ASSERT_EQUAL_INT(0, ntp_set_ext_hook(&g_ctx, &hook));
	good_quality(&q);
	memset(req, 0, sizeof(req));

	for (size_t extra = 0U; extra <= 400U; extra += 4U) {
		size_t len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U, 1U,
					  0U);

		/* 4, 20 and 24 octets of tail are MAC fields by RFC 7822's
		 * disambiguation rule, not extension fields; test_symmetric_mac
		 * covers those. */
		if (extra == 4U || extra == 20U || extra == 24U) {
			continue;
		}
		if (extra >= 4U) {
			bytes_put_be16(&req[len], 0x0104U);
			bytes_put_be16(&req[len + 2U], (uint16_t)extra);
			len += extra;
		}
		fill_rx(&rx, req, len, (uint32_t)extra, 0);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
		TEST_ASSERT_TRUE(res.len <= len);
		TEST_ASSERT_EQUAL_size_t(len, h.last_cap);
	}
}

/* -------------------------------------------------------------------- fuzz */

static void test_random_input_never_crashes(void)
{
	uint8_t buf[NTP_PKT_MAX + 16U];
	uint8_t out[NTP_PKT_MAX];
	uint32_t s = 0xB16B00B5U;
	ntp_quality_view_t q;
	ntp_pkt_t p;
	ntp_rx_t rx;
	ntp_result_t res;
	ntp_stats_t st;

	good_quality(&q);

	for (unsigned iter = 0U; iter < 20000U; iter++) {
		size_t len;

		/* xorshift32, seeded here so a failure replays exactly. */
		s ^= s << 13;
		s ^= s >> 17;
		s ^= s << 5;
		len = (size_t)(s % (NTP_PKT_MAX + 16U));

		for (size_t i = 0U; i < len; i++) {
			s ^= s << 13;
			s ^= s >> 17;
			s ^= s << 5;
			buf[i] = (uint8_t)(s >> 24);
		}

		/* Every second packet is forced into the served shape, so the
		 * fuzz reaches the response builder and not only the filters. */
		if (len >= NTP_HDR_LEN && (iter & 1U) == 0U) {
			buf[0] = (uint8_t)((buf[0] & 0xC0U) | 0x23U);
		}

		(void)ntp_parse(buf, len, &p);
		fill_rx(&rx, buf, len, s, (int64_t)iter);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
		TEST_ASSERT_TRUE(res.len <= len);
		TEST_ASSERT_TRUE(res.len <= sizeof(out));
		if (res.action == NTP_ACT_IGNORE) {
			TEST_ASSERT_EQUAL_size_t(0U, res.len);
		} else {
			TEST_ASSERT_TRUE(res.len >= NTP_HDR_LEN);
			/* Whatever came in, what goes out is a mode-4 server
			 * reply and nothing else. */
			TEST_ASSERT_EQUAL_UINT8((uint8_t)NTP_MODE_SERVER,
						out[0] & 0x07U);
		}
	}

	/* The counters partition the traffic exactly; a request that fell
	 * through every branch without being counted would show up here. */
	TEST_ASSERT_EQUAL_INT(0, ntp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(20000U, st.rx);
	TEST_ASSERT_EQUAL_UINT64(st.rx,
				 st.served + st.kod + st.dropped + st.ignored);
	TEST_ASSERT_EQUAL_INT(0, ntp_stats_reset(&g_ctx));
	TEST_ASSERT_EQUAL_INT(0, ntp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(0U, st.rx);
}

static void test_token_bucket_bounds(void)
{
	uint8_t req[NTP_HDR_LEN];
	uint8_t out[NTP_PKT_MAX];
	ntp_cfg_t cfg;
	ntp_quality_view_t q;
	ntp_rx_t rx;
	ntp_result_t res;
	size_t len = make_request(req, 4U, (uint8_t)NTP_MODE_CLIENT, 6U, 1U, 0U);
	unsigned served;

	good_quality(&q);

	/* A configured burst of zero would wedge the bucket shut. It is clamped
	 * to one request instead of refused: a nonsensical setting should
	 * degrade the service, not take it down. */
	ntp_cfg_default(&cfg);
	cfg.client_rate = 1U;
	cfg.client_burst = 0U;
	cfg.global_rate = 0U;
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, &cfg, &g_port, 0));
	fill_rx(&rx, req, len, 1U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_RESPOND, res.action);
	fill_rx(&rx, req, len, 1U, 0);
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_NOT_EQUAL(NTP_ACT_RESPOND, res.action);

	/* And an absurd one is clamped down, which is what keeps the
	 * milli-token accounting inside 32 bits. */
	ntp_cfg_default(&cfg);
	cfg.client_rate = 1U;
	cfg.client_burst = NTP_BURST_MAX + 1U;
	cfg.global_rate = 0U;
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, &cfg, &g_port, 0));
	for (unsigned i = 0U; i < 64U; i++) {
		fill_rx(&rx, req, len, 2U, 0);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
		TEST_ASSERT_EQUAL_INT(NTP_ACT_RESPOND, res.action);
	}

	/*
	 * A slot idle for thirty years: the refill multiplies elapsed
	 * milliseconds by the rate, so the elapsed time has to be clamped
	 * before the multiply or the product leaves 64 bits. The bucket must
	 * come back exactly full — four requests, then limited again.
	 */
	ntp_cfg_default(&cfg);
	cfg.client_rate = 1U;
	cfg.client_burst = 4U;
	cfg.global_rate = 0U;
	TEST_ASSERT_EQUAL_INT(0, ntp_init(&g_ctx, &cfg, &g_port, 0));
	for (unsigned i = 0U; i < 8U; i++) {
		fill_rx(&rx, req, len, 3U, 0);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
	}
	served = 0U;
	for (unsigned i = 0U; i < 8U; i++) {
		fill_rx(&rx, req, len, 3U, INT64_C(1000000000000));
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
		if (res.action == NTP_ACT_RESPOND) {
			served++;
		}
	}
	TEST_ASSERT_EQUAL_UINT(4U, served);

	/* A clock that steps backwards must not drain the bucket. */
	fill_rx(&rx, req, len, 4U, INT64_C(1000000000000));
	TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
						    sizeof(out), &res));
	TEST_ASSERT_EQUAL_INT(NTP_ACT_RESPOND, res.action);
	served = 0U;
	for (unsigned i = 0U; i < 8U; i++) {
		fill_rx(&rx, req, len, 4U, 0);
		TEST_ASSERT_EQUAL_INT(0, ntp_handle_request(&g_ctx, &rx, &q, out,
							    sizeof(out), &res));
		if (res.action == NTP_ACT_RESPOND) {
			served++;
		}
	}
	TEST_ASSERT_EQUAL_UINT(3U, served);
}

int main(void)
{
	UNITY_BEGIN();
	RUN_TEST(test_tai_to_ntp_conversion);
	RUN_TEST(test_era_rollover);
	RUN_TEST(test_short_format_saturates);
	RUN_TEST(test_defaults);
	RUN_TEST(test_parse_header_fields);
	RUN_TEST(test_parse_tail_disambiguation);
	RUN_TEST(test_parse_tolerates_a_broken_tail);
	RUN_TEST(test_extension_field_iterator);
	RUN_TEST(test_response_byte_vector);
	RUN_TEST(test_response_maps_the_quality_view);
	RUN_TEST(test_serve_unsync_can_be_disabled);
	RUN_TEST(test_unserved_modes_and_versions);
	RUN_TEST(test_malformed_and_argument_errors);
	RUN_TEST(test_rate_limit_burst_throttle_recover);
	RUN_TEST(test_token_bucket_bounds);
	RUN_TEST(test_kod_rate_bytes);
	RUN_TEST(test_kod_can_be_disabled_and_global_bucket_bites);
	RUN_TEST(test_kod_is_damped);
	RUN_TEST(test_client_table_evicts_the_stalest);
	RUN_TEST(test_interleaved_handshake);
	RUN_TEST(test_interleave_can_be_disabled);
	RUN_TEST(test_symmetric_mac);
	RUN_TEST(test_mac_key_table);
	RUN_TEST(test_mac_without_a_crypto_port);
	RUN_TEST(test_extension_hook);
	RUN_TEST(test_response_never_exceeds_request);
	RUN_TEST(test_random_input_never_crashes);
	return UNITY_END();
}
