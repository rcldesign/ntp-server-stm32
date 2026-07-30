/*
 * STS1000 "Meridian" — core/mcp PHASE_EXPORT (0x24) unit tests.
 *
 * PHASE_EXPORT is an offset-chunked bulk snapshot read that reuses CFG_EXPORT's
 * offset contract verbatim (mcp_wire.h 0x24): offset 0 restarts and snapshots,
 * any other offset must be either the byte the previous chunk ended at or the
 * previous chunk's own offset (a retransmit after a lost response), and
 * anything else is MCP_ERR_OFFSET. That contract is what this suite drives, at
 * the wire, through mcp_input() — not through an internal API.
 *
 * The one property worth the most care is coherence. The source ring advances
 * once a second, so the engine must call mcp_phase_port_t::begin() exactly once
 * per transfer, at offset 0, and serve every later chunk from that frozen copy.
 * Re-sampling per chunk would splice two windows into one phase array and hand
 * the join to an offline estimator as data. The fake port below therefore keeps
 * a "live" buffer distinct from the snapshot begin() takes, so a test can move
 * the source mid-transfer and prove the bytes on the wire did not follow.
 *
 * The engine never inspects the record's contents (mcp.h), so the payload here
 * is a long-period byte pattern rather than a real stats/phase_rec.h record.
 * That keeps the suite about the chunking contract and independent of the
 * encoder, which test_phase_export.c covers separately.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "logring/logring.h"
#include "mcp/mcp.h"
#include "util/bytes.h"
#include "util/cobs.h"
#include "util/crc.h"

/* ------------------------------------------------------------------------- */
/* Transmit capture                                                          */
/* ------------------------------------------------------------------------- */

typedef struct {
	uint8_t buf[MCP_MAX_FRAME];
	size_t  len;
} cap_t;

static cap_t  g_last;
static size_t g_txn;

static int tx_cb(void *user, const uint8_t *buf, size_t len)
{
	size_t dn = 0U;

	(void)user;
	TEST_ASSERT_TRUE(len >= 2U);
	TEST_ASSERT_EQUAL_HEX8(0x00U, buf[len - 1U]);
	TEST_ASSERT_EQUAL_INT(0, cobs_decode(buf, len - 1U, g_last.buf,
					     sizeof(g_last.buf), &dn));
	g_last.len = dn;
	g_txn++;
	return 0;
}

static void rsp_check(void)
{
	TEST_ASSERT_TRUE(g_txn > 0U);
	TEST_ASSERT_TRUE(g_last.len >= (MCP_HDR_LEN + MCP_CRC_LEN));
	TEST_ASSERT_EQUAL_HEX32(sts_crc32_ieee(g_last.buf,
					       g_last.len - MCP_CRC_LEN),
				bytes_get_le32(&g_last.buf[g_last.len -
							   MCP_CRC_LEN]));
}

static uint16_t rsp_len(void)
{
	rsp_check();
	return bytes_get_le16(&g_last.buf[6]);
}

static const uint8_t *rsp_pay(void)
{
	rsp_check();
	return &g_last.buf[MCP_HDR_LEN];
}

static uint8_t rsp_status(void)
{
	TEST_ASSERT_TRUE(rsp_len() >= 1U);
	return rsp_pay()[0];
}

/* PHASE_EXPORT response body: u8 status, u32 offset, u8 more, u8 data[...]. */
#define RSP_OVERHEAD 6U

static uint32_t rsp_offset(void)
{
	TEST_ASSERT_TRUE(rsp_len() >= RSP_OVERHEAD);
	return bytes_get_le32(&rsp_pay()[1]);
}

static uint8_t rsp_more(void)
{
	TEST_ASSERT_TRUE(rsp_len() >= RSP_OVERHEAD);
	return rsp_pay()[5];
}

static uint16_t rsp_data_len(void)
{
	TEST_ASSERT_TRUE(rsp_len() >= RSP_OVERHEAD);
	return (uint16_t)(rsp_len() - RSP_OVERHEAD);
}

static const uint8_t *rsp_data(void)
{
	TEST_ASSERT_TRUE(rsp_len() >= RSP_OVERHEAD);
	return &rsp_pay()[RSP_OVERHEAD];
}

/* ------------------------------------------------------------------------- */
/* Fake phase-record port                                                    */
/* ------------------------------------------------------------------------- */

#define REC_MAX  4096U
#define CHUNK    ((uint32_t)MCP_PHASE_EXPORT_CHUNK)

/*
 * g_live is the moving source — the stand-in for the discipline loop's ring.
 * g_snap is what begin() froze. read() serves g_snap and never looks at g_live,
 * which is exactly the split the real port must implement; a test that moves
 * g_live mid-transfer is therefore asking about the engine's call pattern, not
 * about the fake's.
 */
static uint8_t  g_live[REC_MAX];
static uint32_t g_live_len;
static uint8_t  g_snap[REC_MAX];
static uint32_t g_snap_len;

static unsigned int g_begin_calls;
static unsigned int g_read_calls;
static int          g_begin_rc;
static int          g_read_rc;
static bool         g_read_starves; /* return success with zero bytes */

/*
 * The furthest byte any read() was ASKED for (off + cap), regardless of how
 * many the fake chose to return. The engine clamps its request to the snapshot
 * length, so this must never exceed g_snap_len — a port that clamps silently
 * (as a real one would) hides an over-read, so the reach is recorded here and
 * asserted by the tests instead.
 */
static uint32_t g_read_reach;

static int ph_begin(void *user, uint32_t *out_len)
{
	(void)user;
	g_begin_calls++;
	if (g_begin_rc != 0) {
		return g_begin_rc;
	}
	memcpy(g_snap, g_live, g_live_len);
	g_snap_len = g_live_len;
	*out_len = g_snap_len;
	/* The reach is an assertion about THIS snapshot, so a restart — which
	 * may shrink the record — starts a fresh accounting window. */
	g_read_reach = 0U;
	return 0;
}

static int ph_read(void *user, uint32_t off, uint8_t *buf, uint32_t cap,
		   uint32_t *out_n)
{
	uint32_t n;

	(void)user;
	g_read_calls++;
	if (off + cap > g_read_reach) {
		g_read_reach = off + cap;
	}
	if (g_read_rc != 0) {
		return g_read_rc;
	}
	/*
	 * Not a contract check on the caller — a real port would clamp — but a
	 * genuinely out-of-range start would be a memcpy past g_snap here, so
	 * it is caught rather than executed.
	 */
	TEST_ASSERT_TRUE_MESSAGE(off <= g_snap_len,
				 "PHASE_EXPORT read started past the snapshot");
	if (g_read_starves) {
		*out_n = 0U;
		return 0;
	}
	n = g_snap_len - off;
	if (n > cap) {
		n = cap;
	}
	memcpy(buf, &g_snap[off], n);
	*out_n = n;
	return 0;
}

static const mcp_phase_port_t g_phase = { .begin = ph_begin,
					  .read = ph_read,
					  .user = NULL };

/** Byte pattern with a period far longer than one chunk, so no two chunks of a
 *  record are accidentally identical and a retransmit test cannot pass by
 *  comparing a repeat against its own neighbour. */
static void pattern_fill(uint8_t *buf, uint32_t n, uint32_t seed)
{
	uint32_t s = seed | 1U;
	uint32_t i;

	for (i = 0U; i < n; i++) {
		s = (s * 1664525U) + 1013904223U;
		buf[i] = (uint8_t)(s >> 24);
	}
}

static void live_set(uint32_t len, uint32_t seed)
{
	TEST_ASSERT_TRUE(len <= REC_MAX);
	memset(g_live, 0, sizeof(g_live));
	pattern_fill(g_live, len, seed);
	g_live_len = len;
}

/* ------------------------------------------------------------------------- */
/* Engine under test                                                         */
/* ------------------------------------------------------------------------- */

#define LOG_CAP 16U

static mcp_ctx_t  g_mcp;
static logr_t     g_log;
static logr_rec_t g_log_slots[LOG_CAP];
static uint16_t   g_seq;
static uint64_t   g_now;

static uint64_t clk_mono(void *ctx)
{
	(void)ctx;
	return g_now;
}

static const port_clock_t g_clock = { .mono_ms = clk_mono, .tai_ns = NULL,
				      .ctx = NULL };

/*
 * No cfg registry is wired: with no policy and no credential store the engine
 * treats auth as not required (mcp.h), which is also PHASE_EXPORT's documented
 * posture — the record is phase-error telemetry, the same class STATUS_GET
 * already serves without a session (mcp_wire.h 0x24).
 */
static void engine_up(const mcp_phase_port_t *phase)
{
	mcp_wiring_t w;

	memset(&w, 0, sizeof(w));
	w.clock = &g_clock;
	w.log = &g_log;
	w.phase = phase;
	w.tx = tx_cb;
	memcpy(w.ident.model, "STS1000", 7);

	TEST_ASSERT_EQUAL_INT(0, logr_init(&g_log, g_log_slots, LOG_CAP));
	TEST_ASSERT_EQUAL_INT(0, mcp_init(&g_mcp, &w));
	g_seq = 0U;
	g_txn = 0U;
	g_now = 0U;
}

void setUp(void)
{
	memset(g_live, 0, sizeof(g_live));
	memset(g_snap, 0, sizeof(g_snap));
	g_live_len = 0U;
	g_snap_len = 0U;
	g_begin_calls = 0U;
	g_read_calls = 0U;
	g_begin_rc = 0;
	g_read_rc = 0;
	g_read_starves = false;
	g_read_reach = 0U;
	engine_up(&g_phase);
}

/* ------------------------------------------------------------------------- */
/* Request injection                                                         */
/* ------------------------------------------------------------------------- */

static uint8_t g_raw[MCP_MAX_FRAME];
static uint8_t g_enc[COBS_ENCODE_MAX(MCP_MAX_FRAME) + 1U];

static void feed_req(uint8_t cmd, const uint8_t *pl, uint16_t n)
{
	size_t el = 0U;
	size_t fn = MCP_HDR_LEN + (size_t)n + MCP_CRC_LEN;

	g_seq++;
	g_raw[0] = MCP_VER;
	g_raw[1] = (uint8_t)MCP_T_REQ;
	g_raw[2] = cmd;
	g_raw[3] = 0U;
	bytes_put_le16(&g_raw[4], g_seq);
	bytes_put_le16(&g_raw[6], n);
	if (n != 0U) {
		memcpy(&g_raw[MCP_HDR_LEN], pl, n);
	}
	bytes_put_le32(&g_raw[MCP_HDR_LEN + n],
		       sts_crc32_ieee(g_raw, MCP_HDR_LEN + n));

	TEST_ASSERT_EQUAL_INT(0,
		cobs_encode(g_raw, fn, g_enc, sizeof(g_enc) - 1U, &el));
	g_enc[el] = 0x00U;
	TEST_ASSERT_EQUAL_INT(0, mcp_input(&g_mcp, g_enc, el + 1U, NULL));
}

/** Ask for the chunk at @p off. */
static void ask(uint32_t off)
{
	uint8_t req[4];

	bytes_put_le32(req, off);
	feed_req(MCP_CMD_PHASE_EXPORT, req, 4U);
}

static void expect(uint8_t status)
{
	TEST_ASSERT_EQUAL_HEX8(status, rsp_status());
}

/** Ask for @p off, require MCP_OK, and require the echoed offset to match. */
static void ask_ok(uint32_t off)
{
	ask(off);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT32(off, rsp_offset());
}

/*
 * The engine clamps every read to the snapshot's remaining length, so it must
 * never ask the port for a byte the snapshot does not hold. Checked after any
 * test that streams, because a port that clamps would otherwise absorb the bug.
 */
static void assert_no_overread(void)
{
	TEST_ASSERT_TRUE_MESSAGE(g_read_reach <= g_snap_len,
				 "PHASE_EXPORT asked the port past the "
				 "end of the snapshot");
}

/*
 * Stream a whole record from offset 0 into @p out, checking the per-chunk
 * invariants on the way: the echoed offset is the one asked for, `more` is set
 * for every chunk but the last, and no chunk exceeds MCP_PHASE_EXPORT_CHUNK.
 * Returns the total byte count.
 */
static uint32_t stream_all(uint8_t *out, uint32_t cap, unsigned int *chunks)
{
	uint32_t off = 0U;
	unsigned int rounds = 0U;
	uint8_t more;

	do {
		uint16_t dn;

		ask_ok(off);
		dn = rsp_data_len();
		more = rsp_more();
		TEST_ASSERT_TRUE(dn <= CHUNK);
		TEST_ASSERT_TRUE(((uint64_t)off + dn) <= cap);
		memcpy(&out[off], rsp_data(), dn);
		off += dn;
		rounds++;
		TEST_ASSERT_TRUE(rounds < 32U); /* no unbounded stream */
	} while (more != 0U);

	if (chunks != NULL) {
		*chunks = rounds;
	}
	return off;
}

/* ------------------------------------------------------------------------- */
/* Offset advance                                                            */
/* ------------------------------------------------------------------------- */

/* A record shorter than one chunk is one frame, flagged complete. */
static void test_a_short_record_is_a_single_complete_chunk(void)
{
	uint8_t got[REC_MAX];
	unsigned int chunks = 0U;

	live_set(300U, 0xA1A1A1A1U);

	TEST_ASSERT_EQUAL_UINT32(300U, stream_all(got, sizeof(got), &chunks));
	TEST_ASSERT_EQUAL_UINT(1U, chunks);
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_more());
	TEST_ASSERT_EQUAL_UINT(1U, g_begin_calls);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_live, got, 300U);
	assert_no_overread();
}

/*
 * The property the whole command exists to deliver: sequential chunks
 * reassemble to exactly the record the port snapshotted, byte for byte.
 */
static void test_sequential_chunks_reassemble_the_record(void)
{
	uint8_t got[REC_MAX];
	unsigned int chunks = 0U;
	const uint32_t len = (2U * CHUNK) + 300U;

	live_set(len, 0x5EED0001U);

	TEST_ASSERT_EQUAL_UINT32(len, stream_all(got, sizeof(got), &chunks));
	TEST_ASSERT_EQUAL_UINT(3U, chunks); /* genuinely multi-chunk */
	TEST_ASSERT_EQUAL_UINT(1U, g_begin_calls);
	TEST_ASSERT_EQUAL_UINT32(len, g_snap_len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_snap, got, len);
	assert_no_overread();
}

/*
 * The final chunk of a record whose length is not a multiple of the chunk size
 * carries the remainder and nothing more. A full-length final chunk would mean
 * the engine read past the end of the snapshot and shipped whatever followed.
 */
static void test_the_final_chunk_is_short_not_padded(void)
{
	const uint32_t len = (2U * CHUNK) + 300U;

	live_set(len, 0x0BADF00DU);

	ask_ok(0U);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)CHUNK, rsp_data_len());
	TEST_ASSERT_EQUAL_UINT8(1U, rsp_more());

	ask_ok(CHUNK);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)CHUNK, rsp_data_len());
	TEST_ASSERT_EQUAL_UINT8(1U, rsp_more());

	ask_ok(2U * CHUNK);
	TEST_ASSERT_EQUAL_UINT16(300U, rsp_data_len());
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_more());
	TEST_ASSERT_EQUAL_HEX8_ARRAY(&g_snap[2U * CHUNK], rsp_data(), 300U);
	assert_no_overread();
}

/* A record that is an exact multiple of the chunk size still terminates: the
 * last full chunk clears `more` rather than inviting a zero-length one. */
static void test_an_exact_multiple_ends_without_an_empty_chunk(void)
{
	uint8_t got[REC_MAX];
	unsigned int chunks = 0U;

	live_set(2U * CHUNK, 0x11223344U);

	ask_ok(0U);
	TEST_ASSERT_EQUAL_UINT8(1U, rsp_more());
	ask_ok(CHUNK);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)CHUNK, rsp_data_len());
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_more()); /* no third, empty chunk */

	/* And a fresh pass agrees. */
	TEST_ASSERT_EQUAL_UINT32(2U * CHUNK,
				 stream_all(got, sizeof(got), &chunks));
	TEST_ASSERT_EQUAL_UINT(2U, chunks);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_snap, got, 2U * CHUNK);
	assert_no_overread();
}

/* ------------------------------------------------------------------------- */
/* Retransmit of the previous chunk                                          */
/* ------------------------------------------------------------------------- */

/*
 * The documented retry rule: re-asking for the previous chunk's offset returns
 * the identical chunk. It must not advance the cursor and must not corrupt the
 * transfer — so the stream is carried to completion afterwards and the whole
 * record still reassembles.
 */
static void test_a_repeated_offset_re_emits_the_same_chunk(void)
{
	uint8_t got[REC_MAX];
	uint8_t saved[CHUNK];
	uint16_t saved_len;
	uint8_t saved_more;
	const uint32_t len = (2U * CHUNK) + 300U;
	unsigned int reads_after;

	live_set(len, 0xC0FFEE01U);

	ask_ok(0U);
	memcpy(&got[0], rsp_data(), rsp_data_len());

	/* Second chunk, captured. */
	ask_ok(CHUNK);
	saved_len = rsp_data_len();
	saved_more = rsp_more();
	TEST_ASSERT_EQUAL_UINT16((uint16_t)CHUNK, saved_len);
	TEST_ASSERT_EQUAL_UINT8(1U, saved_more);
	memcpy(saved, rsp_data(), saved_len);
	reads_after = g_read_calls;

	/* The response was "lost": ask again for the same offset. */
	ask_ok(CHUNK);
	TEST_ASSERT_EQUAL_UINT16(saved_len, rsp_data_len());
	TEST_ASSERT_EQUAL_UINT8(saved_more, rsp_more());
	TEST_ASSERT_EQUAL_HEX8_ARRAY(saved, rsp_data(), saved_len);

	/* It really re-read rather than replaying a cached frame... */
	TEST_ASSERT_TRUE(g_read_calls > reads_after);
	/* ...and it did not re-snapshot to do so. */
	TEST_ASSERT_EQUAL_UINT(1U, g_begin_calls);

	/* Idempotent: a second repeat answers the same again. */
	ask_ok(CHUNK);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(saved, rsp_data(), saved_len);
	TEST_ASSERT_EQUAL_UINT16(saved_len, rsp_data_len());

	/* State survived: the transfer resumes where it was and completes. */
	memcpy(&got[CHUNK], saved, saved_len);
	ask_ok(2U * CHUNK);
	TEST_ASSERT_EQUAL_UINT16(300U, rsp_data_len());
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_more());
	memcpy(&got[2U * CHUNK], rsp_data(), 300U);

	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_snap, got, len);
	TEST_ASSERT_EQUAL_UINT(1U, g_begin_calls);
	assert_no_overread();
}

/*
 * The final chunk is the one whose response is most likely to be lost — it is
 * the last thing on the wire and nothing follows to reveal the loss. It stays
 * re-emittable after the transfer has ended, which is why the "previous chunk"
 * memory deliberately outlives the active flag (mcp.h).
 */
static void test_the_final_chunk_stays_re_emittable_after_completion(void)
{
	uint8_t saved[CHUNK];
	uint16_t saved_len;
	const uint32_t len = CHUNK + 77U;

	live_set(len, 0x7E577E57U);

	ask_ok(0U);
	TEST_ASSERT_EQUAL_UINT8(1U, rsp_more());

	ask_ok(CHUNK);
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_more()); /* transfer complete */
	saved_len = rsp_data_len();
	TEST_ASSERT_EQUAL_UINT16(77U, saved_len);
	memcpy(saved, rsp_data(), saved_len);

	/* Re-ask the final offset after the export has already ended. */
	ask_ok(CHUNK);
	TEST_ASSERT_EQUAL_UINT16(saved_len, rsp_data_len());
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_more());
	TEST_ASSERT_EQUAL_HEX8_ARRAY(saved, rsp_data(), saved_len);

	/* Twice. */
	ask_ok(CHUNK);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(saved, rsp_data(), saved_len);
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_more());
	TEST_ASSERT_EQUAL_UINT(1U, g_begin_calls);
	assert_no_overread();
}

/* ------------------------------------------------------------------------- */
/* Offsets the contract refuses                                              */
/* ------------------------------------------------------------------------- */

/*
 * An offset that is neither the frontier nor the previous chunk is refused
 * outright. It must not reach the port, and it must not disturb a transfer in
 * flight — the very next legitimate request still succeeds.
 */
static void test_a_non_boundary_offset_is_refused_without_disturbing_state(void)
{
	uint8_t got[REC_MAX];
	uint8_t saved[CHUNK];
	const uint32_t len = (2U * CHUNK) + 300U;
	unsigned int reads_before;
	static const uint32_t bogus[] = { 1U, 7U, 300U, CHUNK - 1U,
					  CHUNK + 1U, (2U * CHUNK) - 1U };
	size_t i;

	live_set(len, 0xDEADBEEFU);

	ask_ok(0U);
	memcpy(&got[0], rsp_data(), rsp_data_len());
	ask_ok(CHUNK);
	memcpy(saved, rsp_data(), rsp_data_len());
	memcpy(&got[CHUNK], saved, rsp_data_len());
	reads_before = g_read_calls;

	for (i = 0U; i < (sizeof(bogus) / sizeof(bogus[0])); i++) {
		ask(bogus[i]);
		expect((uint8_t)MCP_ERR_OFFSET);
	}

	/* None of them reached the port at all. */
	TEST_ASSERT_EQUAL_UINT(reads_before, g_read_calls);
	TEST_ASSERT_EQUAL_UINT(1U, g_begin_calls);

	/* The frontier still works... */
	ask_ok(2U * CHUNK);
	TEST_ASSERT_EQUAL_UINT16(300U, rsp_data_len());
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_more());
	memcpy(&got[2U * CHUNK], rsp_data(), 300U);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_snap, got, len);

	/* ...and so does the retransmit memory the refusals ran past. */
	ask_ok(2U * CHUNK);
	TEST_ASSERT_EQUAL_UINT16(300U, rsp_data_len());
	assert_no_overread();
}

/*
 * Offsets at and beyond the end of the record. The tool stops when `more` is
 * clear and never asks for the end-of-stream offset, so it is refused rather
 * than answered with an empty chunk; an offset well past the end must not
 * become a read out of bounds.
 */
static void test_an_offset_past_the_end_is_refused(void)
{
	const uint32_t len = CHUNK + 300U;
	unsigned int reads_before;

	live_set(len, 0x99AABBCCU);

	ask_ok(0U);
	ask_ok(CHUNK);
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_more());
	reads_before = g_read_calls;

	ask(len); /* exactly the end */
	expect((uint8_t)MCP_ERR_OFFSET);

	ask(len + 1U);
	expect((uint8_t)MCP_ERR_OFFSET);

	ask(REC_MAX * 4U);
	expect((uint8_t)MCP_ERR_OFFSET);

	ask(0xFFFFFFFFU); /* would overflow off + cap if it ever reached read */
	expect((uint8_t)MCP_ERR_OFFSET);

	TEST_ASSERT_EQUAL_UINT(reads_before, g_read_calls);
	TEST_ASSERT_EQUAL_UINT(1U, g_begin_calls);
	assert_no_overread();
}

/* Offset 0 always restarts rather than being read as "the previous chunk". */
static void test_offset_zero_restarts_the_export(void)
{
	uint8_t got[REC_MAX];
	unsigned int chunks = 0U;
	const uint32_t len = CHUNK + 200U;

	live_set(len, 0x13571357U);

	ask_ok(0U);
	ask_ok(CHUNK);
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_more());

	/* Restart mid-life: a second full pass from 0 works. */
	TEST_ASSERT_EQUAL_UINT32(len, stream_all(got, sizeof(got), &chunks));
	TEST_ASSERT_EQUAL_UINT(2U, chunks);
	TEST_ASSERT_EQUAL_UINT(2U, g_begin_calls); /* one per pass, no more */
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_snap, got, len);
	assert_no_overread();
}

/* ------------------------------------------------------------------------- */
/* Snapshot coherence                                                        */
/* ------------------------------------------------------------------------- */

/*
 * The point of splitting the port into begin/read. The source ring moves once a
 * second; the record must not. Here the live source is replaced wholesale
 * between chunks, and the bytes on the wire must still be the ones frozen at
 * offset 0 — otherwise a multi-chunk record would contain samples from two
 * different windows, spliced at a chunk boundary, and report the join as data.
 */
static void test_the_snapshot_is_frozen_at_offset_zero(void)
{
	uint8_t got[REC_MAX];
	uint8_t frozen[REC_MAX];
	const uint32_t len = (2U * CHUNK) + 300U;

	live_set(len, 0x0F0F0F0FU);

	/* Chunk 0 takes the snapshot. */
	ask_ok(0U);
	memcpy(&got[0], rsp_data(), rsp_data_len());
	TEST_ASSERT_EQUAL_UINT(1U, g_begin_calls);

	/* Remember what was frozen, then move the source underneath. */
	memcpy(frozen, g_snap, len);
	live_set(len, 0xF0F0F0F0U);
	TEST_ASSERT_TRUE(memcmp(frozen, g_live, len) != 0); /* really moved */

	ask_ok(CHUNK);
	memcpy(&got[CHUNK], rsp_data(), rsp_data_len());

	/* Move it again, differently, before the last chunk. */
	live_set(len, 0x2468ACE0U);

	ask_ok(2U * CHUNK);
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_more());
	memcpy(&got[2U * CHUNK], rsp_data(), rsp_data_len());

	/*
	 * Every chunk came out of the one snapshot. The byte comparison is
	 * first on purpose: it is the observable property — a spliced record —
	 * whereas the call count is the mechanism that delivers it, and a
	 * failure should name the damage rather than the cause.
	 */
	TEST_ASSERT_EQUAL_HEX8_ARRAY(frozen, got, len);
	TEST_ASSERT_EQUAL_UINT(1U, g_begin_calls);
	assert_no_overread();
}

/*
 * The complement, and the reason the test above is not vacuous: the port really
 * does track the live source, so a fresh export at offset 0 sees the moved
 * record. Without this, a fake that had simply stopped following g_live would
 * make the freeze test pass for the wrong reason.
 */
static void test_a_fresh_export_sees_the_moved_source(void)
{
	uint8_t got[REC_MAX];
	uint8_t frozen[REC_MAX];
	unsigned int chunks = 0U;
	const uint32_t len = (2U * CHUNK) + 300U;

	live_set(len, 0x0F0F0F0FU);
	ask_ok(0U);
	memcpy(frozen, g_snap, len);

	live_set(len, 0xF0F0F0F0U);

	/* A new export from 0 re-snapshots and yields the NEW content. */
	TEST_ASSERT_EQUAL_UINT32(len, stream_all(got, sizeof(got), &chunks));
	TEST_ASSERT_EQUAL_UINT(3U, chunks);
	TEST_ASSERT_EQUAL_UINT(2U, g_begin_calls);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_live, got, len);
	TEST_ASSERT_TRUE(memcmp(frozen, got, len) != 0);
	assert_no_overread();
}

/* A restart also resizes: the snapshot's length is re-taken, not remembered. */
static void test_a_restart_re_takes_the_length(void)
{
	uint8_t got[REC_MAX];
	unsigned int chunks = 0U;

	live_set((2U * CHUNK) + 300U, 0xAAAA5555U);
	ask_ok(0U);
	TEST_ASSERT_EQUAL_UINT8(1U, rsp_more());

	/* The ring shrank between exports (a reset, say). */
	live_set(400U, 0x5555AAAAU);

	TEST_ASSERT_EQUAL_UINT32(400U, stream_all(got, sizeof(got), &chunks));
	TEST_ASSERT_EQUAL_UINT(1U, chunks);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_live, got, 400U);

	/* The old transfer's offsets are gone with it. */
	ask(CHUNK);
	expect((uint8_t)MCP_ERR_OFFSET);
	assert_no_overread();
}

/*
 * A dropped connection ends the transfer. The snapshot is scoped to the session
 * so the next tool to attach gets the ring as it is now, not the stale window
 * the previous one abandoned mid-fetch — which means BOTH the frontier and the
 * retransmit window have to go, not just the active flag.
 */
static void test_a_session_reset_drops_the_transfer(void)
{
	uint8_t got[REC_MAX];
	unsigned int chunks = 0U;
	const uint32_t len = (2U * CHUNK) + 300U;

	live_set(len, 0x5E551011U);

	ask_ok(0U);
	ask_ok(CHUNK); /* frontier now 2*CHUNK, previous chunk CHUNK */

	mcp_reset_session(&g_mcp);

	ask(2U * CHUNK); /* the frontier is gone */
	expect((uint8_t)MCP_ERR_OFFSET);
	ask(CHUNK); /* and so is the retransmit window */
	expect((uint8_t)MCP_ERR_OFFSET);
	TEST_ASSERT_EQUAL_UINT(1U, g_begin_calls);

	/* The ring moved while nobody was attached; the new session sees it. */
	live_set(len, 0x11105E55U);
	TEST_ASSERT_EQUAL_UINT32(len, stream_all(got, sizeof(got), &chunks));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_live, got, len);
	TEST_ASSERT_EQUAL_UINT(2U, g_begin_calls);
	assert_no_overread();
}

/* ------------------------------------------------------------------------- */
/* Degenerate and failing sources                                            */
/* ------------------------------------------------------------------------- */

/* An empty ring still encodes to a valid, empty record (mcp.h): one frame,
 * complete, no data, and the port is never asked to read a byte. */
static void test_an_empty_record_is_one_complete_empty_frame(void)
{
	live_set(0U, 0U);

	ask_ok(0U);
	TEST_ASSERT_EQUAL_UINT16(0U, rsp_data_len());
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_more());
	TEST_ASSERT_EQUAL_UINT(1U, g_begin_calls);
	TEST_ASSERT_EQUAL_UINT(0U, g_read_calls);

	/* Offset 0 is both the first and the last chunk, so it re-emits. */
	ask_ok(0U);
	TEST_ASSERT_EQUAL_UINT16(0U, rsp_data_len());
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_more());

	/* Anything else is still refused. */
	ask(1U);
	expect((uint8_t)MCP_ERR_OFFSET);
	TEST_ASSERT_EQUAL_UINT(0U, g_read_calls);
}

/* A snapshot that cannot be taken is reported, and leaves no transfer behind
 * for a later offset to latch onto. */
static void test_a_begin_failure_is_reported_and_starts_nothing(void)
{
	live_set(CHUNK + 100U, 0x31415926U);

	g_begin_rc = -EIO;
	ask(0U);
	expect((uint8_t)MCP_ERR_INTERNAL);
	TEST_ASSERT_EQUAL_UINT(0U, g_read_calls);

	/* No transfer exists, so the frontier a caller might assume is refused. */
	ask(CHUNK);
	expect((uint8_t)MCP_ERR_OFFSET);
	ask(0U);
	expect((uint8_t)MCP_ERR_INTERNAL);

	/* -ENOTSUP is distinguished from a generic failure. */
	g_begin_rc = -ENOTSUP;
	ask(0U);
	expect((uint8_t)MCP_ERR_NOTSUP);

	/* And once the source recovers, a fresh export works. */
	g_begin_rc = 0;
	ask_ok(0U);
	TEST_ASSERT_EQUAL_UINT8(1U, rsp_more());
	assert_no_overread();
}

/* A read failure mid-record ends the transfer rather than truncating it
 * silently: the next forward offset is refused, so the tool must restart. */
static void test_a_read_failure_ends_the_transfer(void)
{
	const uint32_t len = (2U * CHUNK) + 300U;

	live_set(len, 0x8BADF00DU);

	ask_ok(0U);

	g_read_rc = -EIO;
	ask(CHUNK);
	expect((uint8_t)MCP_ERR_INTERNAL);

	g_read_rc = 0;
	ask(2U * CHUNK); /* the frontier a naive tool would try next */
	expect((uint8_t)MCP_ERR_OFFSET);

	/* The failed chunk's own offset is still the "previous" one, so the
	 * documented retry does work — that is the whole point of the rule. */
	ask_ok(CHUNK);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)CHUNK, rsp_data_len());
	TEST_ASSERT_EQUAL_HEX8_ARRAY(&g_snap[CHUNK], rsp_data(), CHUNK);
	assert_no_overread();
}

/* A port that promises bytes and delivers none cannot make progress; answering
 * "complete" would hand the tool a truncated record it would analyse as whole. */
static void test_a_starving_port_is_an_internal_error(void)
{
	live_set(CHUNK + 300U, 0x1BADB002U);

	g_read_starves = true;
	ask(0U);
	expect((uint8_t)MCP_ERR_INTERNAL);
	TEST_ASSERT_EQUAL_UINT(1U, g_read_calls);
}

/* ------------------------------------------------------------------------- */
/* Framing and wiring                                                        */
/* ------------------------------------------------------------------------- */

/* The request is exactly one u32; nothing else is a PHASE_EXPORT. */
static void test_a_malformed_request_is_refused(void)
{
	uint8_t req[8] = { 0 };

	live_set(CHUNK + 10U, 0x24242424U);

	feed_req(MCP_CMD_PHASE_EXPORT, NULL, 0U);
	expect((uint8_t)MCP_ERR_ARG);

	feed_req(MCP_CMD_PHASE_EXPORT, req, 3U);
	expect((uint8_t)MCP_ERR_ARG);

	feed_req(MCP_CMD_PHASE_EXPORT, req, 5U);
	expect((uint8_t)MCP_ERR_ARG);

	feed_req(MCP_CMD_PHASE_EXPORT, req, 8U);
	expect((uint8_t)MCP_ERR_ARG);

	/* None of them started or disturbed an export. */
	TEST_ASSERT_EQUAL_UINT(0U, g_begin_calls);
	feed_req(MCP_CMD_PHASE_EXPORT, req, 4U);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT(1U, g_begin_calls);
}

/* Without a source — or with half a source — the command is unsupported, and
 * the capability bit HELLO advertises follows the same rule. */
static void test_the_command_and_its_capability_need_a_whole_port(void)
{
	static const mcp_phase_port_t no_read = { .begin = ph_begin,
						  .read = NULL,
						  .user = NULL };
	static const mcp_phase_port_t no_begin = { .begin = NULL,
						   .read = ph_read,
						   .user = NULL };
	uint32_t caps;

	/* Fully wired: capability advertised, command answered. */
	feed_req(MCP_CMD_HELLO, NULL, 0U);
	expect((uint8_t)MCP_OK);
	caps = bytes_get_le32(&rsp_pay()[4]);
	TEST_ASSERT_EQUAL_HEX32(MCP_CAP_PHASE, caps & MCP_CAP_PHASE);

	engine_up(NULL);
	ask(0U);
	expect((uint8_t)MCP_ERR_NOTSUP);
	feed_req(MCP_CMD_HELLO, NULL, 0U);
	caps = bytes_get_le32(&rsp_pay()[4]);
	TEST_ASSERT_EQUAL_HEX32(0U, caps & MCP_CAP_PHASE);

	engine_up(&no_read);
	ask(0U);
	expect((uint8_t)MCP_ERR_NOTSUP);
	feed_req(MCP_CMD_HELLO, NULL, 0U);
	caps = bytes_get_le32(&rsp_pay()[4]);
	TEST_ASSERT_EQUAL_HEX32(0U, caps & MCP_CAP_PHASE);

	engine_up(&no_begin);
	ask(0U);
	expect((uint8_t)MCP_ERR_NOTSUP);
	feed_req(MCP_CMD_HELLO, NULL, 0U);
	caps = bytes_get_le32(&rsp_pay()[4]);
	TEST_ASSERT_EQUAL_HEX32(0U, caps & MCP_CAP_PHASE);

	TEST_ASSERT_EQUAL_UINT(0U, g_begin_calls);
	TEST_ASSERT_EQUAL_UINT(0U, g_read_calls);
}

/* A full chunk plus the 6-byte response header must fit one frame — the
 * static assert in mcp.c says so at compile time; this says so on the wire. */
static void test_a_full_chunk_fits_one_frame(void)
{
	live_set(CHUNK + 1U, 0x600DF00DU);

	ask_ok(0U);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)CHUNK, rsp_data_len());
	TEST_ASSERT_TRUE(rsp_len() <= MCP_MAX_PAYLOAD);
	TEST_ASSERT_TRUE(g_last.len <= MCP_MAX_FRAME);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_snap, rsp_data(), CHUNK);
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_a_short_record_is_a_single_complete_chunk);
	RUN_TEST(test_sequential_chunks_reassemble_the_record);
	RUN_TEST(test_the_final_chunk_is_short_not_padded);
	RUN_TEST(test_an_exact_multiple_ends_without_an_empty_chunk);

	RUN_TEST(test_a_repeated_offset_re_emits_the_same_chunk);
	RUN_TEST(test_the_final_chunk_stays_re_emittable_after_completion);

	RUN_TEST(test_a_non_boundary_offset_is_refused_without_disturbing_state);
	RUN_TEST(test_an_offset_past_the_end_is_refused);
	RUN_TEST(test_offset_zero_restarts_the_export);

	RUN_TEST(test_the_snapshot_is_frozen_at_offset_zero);
	RUN_TEST(test_a_fresh_export_sees_the_moved_source);
	RUN_TEST(test_a_restart_re_takes_the_length);
	RUN_TEST(test_a_session_reset_drops_the_transfer);

	RUN_TEST(test_an_empty_record_is_one_complete_empty_frame);
	RUN_TEST(test_a_begin_failure_is_reported_and_starts_nothing);
	RUN_TEST(test_a_read_failure_ends_the_transfer);
	RUN_TEST(test_a_starving_port_is_an_internal_error);

	RUN_TEST(test_a_malformed_request_is_refused);
	RUN_TEST(test_the_command_and_its_capability_need_a_whole_port);
	RUN_TEST(test_a_full_chunk_fits_one_frame);

	return UNITY_END();
}
