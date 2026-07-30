/*
 * STS1000 "Meridian" — spec §14 phase export, composed end to end.
 *
 * WHY THIS SUITE EXISTS
 *
 * Three suites already cover this path, each in isolation:
 *
 *   test_mcp_phase_export.c  the PHASE_EXPORT chunking contract, driven
 *                            against a FAKE mcp_phase_port_t whose record is
 *                            opaque bytes;
 *   test_phase_export.c      the encoder, driven from real disc_tick_pps()
 *                            samples, decoded in the same process;
 *   test_phase_rec/join/saw/stats
 *                            the format, the join, the verdict, the estimators.
 *
 * Nothing composed them. Every layer was proved against a stand-in for its
 * neighbour, so a disagreement BETWEEN two layers — a length the encoder means
 * one way and the transport means another, a series the transport reassembles
 * in the wrong order, a record whose header survives a truncation the transport
 * caused — was invisible to all of them at once. That disagreement is precisely
 * what a bench operator meets first, because the operator drives the whole
 * thing: ask for a record over the wire, reassemble it, and analyse it.
 *
 * So this suite drives the real chain in one process:
 *
 *   disc_tick_pps() x N  ->  disc_phase_snapshot()  ->  phase_rec_encode_f()
 *     ->  the real h_phase_export() chunking, over real COBS+CRC frames
 *     ->  host-side reassembly  ->  phase_rec_decode_hdr/samples/raw()
 *     ->  stats_adev() / stats_saw_verdict()
 *
 * WHAT MAKES THE ASSERTION SHARP
 *
 * A synthetic input with a closed-form answer. A PURE FREQUENCY OFFSET is a
 * phase ramp that is exactly linear in the sample index, and ADEV's second
 * difference annihilates it exactly — stats_adev() documents "exactly 0.0, and
 * that is a guarantee of this implementation, not an approximation". The ramp
 * is chosen in whole nanoseconds so that the float ring, the ns->ps conversion
 * (phase_rec_ns_f_to_ps) and the int64 estimator are each exact, which is what
 * lets the expected value be 0.0 and not "small".
 *
 * Anything that corrupts the record — a byte flipped in transit, a chunk
 * reassembled at the wrong offset, a series read at the wrong stride — makes
 * the deviation non-zero. But an ADEV that came out non-zero would be a
 * terrible bug report, so the byte-identity of the reassembled record is
 * asserted FIRST and separately: the record the host reassembled must equal,
 * byte for byte, the record the encoder produced. A transport defect then
 * names the offending byte offset; only a genuine numerical defect reaches the
 * ADEV assertion.
 *
 * WHAT THIS CANNOT REACH
 *
 * The production mcp_phase_port_t is app/src/zephyr/platform/disc_thread.c's
 * sts_disc_phase_begin()/sts_disc_phase_read(), which is Zephyr-only: it is the
 * request flag, the k_sem hand-off to the discipline thread and the mutex over
 * phase_blob. That wrapper is NOT under test here. What is under test is
 * everything it wraps — the snapshot, the encode into a PHASE_BLOB_CAP-sized
 * buffer, and the serving of that buffer by offset — implemented here by
 * dev_begin()/dev_read() below, which mirror disc_publish_phase() statement for
 * statement so the bytes are the ones the device would ship.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "disc/disc.h"
#include "logring/logring.h"
#include "mcp/mcp.h"
#include "stats/adev.h"
#include "stats/phase_rec.h"
#include "stats/saw.h"
#include "util/bytes.h"
#include "util/cobs.h"
#include "util/crc.h"

/* ------------------------------------------------------------------------- */
/* Sizes                                                                     */
/* ------------------------------------------------------------------------- */

/*
 * The discipline thread's export buffer.
 *
 * Restated from app/src/zephyr/platform/disc_thread.c (PHASE_BLOB_CAP) because
 * that file is Zephyr-only and exports no header. The restatement is not a
 * hazard as long as something checks it, which is what
 * test_the_blob_cap_headroom_is_eight_bytes() is for: it pins the cap, the real
 * record length and the difference between them, so a change to either side
 * fails here with the arithmetic on screen instead of drifting apart quietly.
 */
#define PHASE_BLOB_CAP 8300u

/** Chunk the engine serves (mcp.h). */
#define CHUNK ((uint32_t)MCP_PHASE_EXPORT_CHUNK)

/*
 * Phase ramp: a pure frequency offset of RAMP_NS_PER_S nanoseconds per second,
 * i.e. 2 ppb — an ordinary free-running OCXO. Whole nanoseconds, so every
 * conversion in the chain is exact (see the header comment).
 *
 * NOT a loop response. disc_tick_pps() takes the phase error as an INPUT from
 * the capture; the DAC the loop writes never closes back through a host test.
 * The ramp is therefore the shape of the measurement, which is exactly what the
 * export path carries and what the bench operator's tooling analyses.
 */
#define RAMP_NS_PER_S 2

/** Ticks driven before the export. > DISC_ADEV_CAP so the ring is full and the
 *  oldest samples have been evicted — the eviction path is the one production
 *  runs in. */
#define TICKS 600u

/** Sawtooth amplitude, whole nanoseconds, for the paired case. Small enough to
 *  sit far inside the §3.2 MAD gate's 100 ns floor on top of the ramp. */
#define SAW_MAX_NS 8

/* ------------------------------------------------------------------------- */
/* Device side: the discipline loop and its exporter                         */
/* ------------------------------------------------------------------------- */

static disc_ctx_t          g_disc;
static disc_phase_snap_t   g_snap;

/** The device's export buffer, exactly the production size. */
static uint8_t  g_blob[PHASE_BLOB_CAP];
static uint32_t g_blob_len;

/** Encoder cap the port hands phase_rec_encode_f(). Normally sizeof(g_blob);
 *  lowered by the cap-boundary case to prove the refusal path. */
static size_t g_blob_cap;

/** Result of the last dev_begin() encode, so a test can assert the refusal. */
static int g_encode_rc;

static void env_defaults(disc_env_t *env, uint64_t ms)
{
	memset(env, 0, sizeof(*env));
	env->mono_ms = ms;
	env->gnss_time_locked = true;
	env->osc_temp_mc = 45000;
	env->osc_temp_valid = true;
	env->ocxo_current_ua = 450000;
	env->ocxo_current_valid = true;
	env->vc_sense_valid = false;
	env->anc.active_ref = (uint8_t)QUALITY_REF_OCXO;
	env->anc.gnss_fix = (uint8_t)QUALITY_GNSS_TIME_ONLY;
	env->anc.gnss_sv_used = 14u;
	env->anc.gnss_sv_visible = 20u;
	env->anc.gnss_tacc_ns = 12u;
	env->anc.utc_valid = true;
	env->anc.leap_current_s = 37;
	env->timebase_traceable = true;
}

/*
 * One accepted PPS second.
 *
 * disc.h: e = (expected - captured) * ns_per_count, so the capture follows from
 * the error we want the loop to see. @p qerr_ns is the sawtooth the receiver
 * reports for this pulse; condition_sample() adds cfg.qerr_sign * qerr to the
 * corrected series and leaves the raw one alone, so passing a measurement of
 * (want - qerr) with a qErr of qerr puts EXACTLY `want` in the corrected ring
 * and (want - qerr) in the raw one.
 *
 * Acceptance is asserted, not assumed: a sample the §3.2 gate rejected never
 * reaches the ADEV ring, and a test that silently lost samples would pass for
 * the wrong reason.
 */
static void tick(uint64_t ms, int meas_ns, int qerr_ns, bool qerr_valid)
{
	static uint32_t expected = 1000000u;
	disc_in_t  in;
	disc_out_t out;

	env_defaults(&in.env, ms);
	memset(&in.pps, 0, sizeof(in.pps));
	in.pps.primary_expected = expected;
	in.pps.primary_count = (uint32_t)(expected - (uint32_t)meas_ns);
	in.pps.primary_ns_per_count = 1.0f;
	in.pps.primary_valid = true;
	in.pps.qerr_ps = (int32_t)(qerr_ns * 1000);
	in.pps.qerr_valid = qerr_valid;
	expected += 1000000u;

	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&g_disc, &in, NULL, &out));
	TEST_ASSERT_TRUE_MESSAGE(out.sample_accepted,
				 "sample was rejected; it never reached the ring");
}

/** Deterministic bipolar sawtooth in whole nanoseconds, |s| <= SAW_MAX_NS and
 *  never 0, so every sample of the paired record carries a real correction. */
static int saw_ns(uint32_t i)
{
	int mag = (int)(i % (uint32_t)SAW_MAX_NS) + 1;

	return ((i & 1u) != 0u) ? -mag : mag;
}

/** Drive the loop through a pure frequency offset, optionally with a sawtooth
 *  on the capture that the qErr path must remove. */
static void drive(bool with_saw)
{
	disc_cfg_t cfg;
	uint32_t i;

	TEST_ASSERT_EQUAL_INT(0, disc_cfg_defaults(&cfg));
	/* A non-zero seed is what makes the export carry PHASE_REC_F_IDENT; the
	 * full-size record §14 ships has the identity block, so the composed
	 * path must be driven with one. */
	cfg.adev_epoch_seed = 0xC0FFEEu;
	TEST_ASSERT_EQUAL_INT(0, disc_init(&g_disc, &cfg));

	for (i = 0u; i < TICKS; i++) {
		int want = (int)i * RAMP_NS_PER_S;
		int q = with_saw ? saw_ns(i) : 0;

		tick(1000u * (uint64_t)(i + 1u), want - q, q, with_saw);
	}
}

/*
 * Snapshot + encode, exactly as disc_publish_phase() does — including
 * phase_rec_encode_f() (the float path production uses, not the int64
 * phase_rec_encode_pair() the encoder suite drives) and including the
 * PHASE_BLOB_CAP-sized destination.
 */
static int device_encode(void)
{
	phase_rec_meta_t meta;
	size_t len = 0u;
	int rc;

	rc = disc_phase_snapshot(&g_disc, &g_snap);
	if (rc != 0) {
		return rc;
	}

	memset(&meta, 0, sizeof(meta));
	meta.ver = PHASE_REC_VER;
	meta.flags = g_snap.full ? (uint8_t)PHASE_REC_F_FULL : 0u;
	meta.n = g_snap.n;
	meta.tau0_ns = g_snap.tau0_ns;
	meta.gaps = g_snap.gaps;
	meta.mono_ms = 1234567u;
	meta.has_ident = g_snap.has_ident;
	meta.epoch = g_snap.epoch;
	meta.seq0 = g_snap.seq0;

	rc = phase_rec_encode_f(&meta, g_snap.x_ns, g_snap.x_raw_ns,
			        g_snap.gapmap, g_blob, g_blob_cap, &len);
	g_blob_len = (rc == 0) ? (uint32_t)len : 0u;
	return rc;
}

/* ------------------------------------------------------------------------- */
/* The mcp_phase_port_t the engine is wired to                               */
/* ------------------------------------------------------------------------- */

static unsigned int g_begin_calls;
static uint32_t     g_read_reach;

/** Mirrors sts_disc_phase_begin(): encode a fresh snapshot, publish its
 *  length, report the encoder's error unchanged. */
static int dev_begin(void *user, uint32_t *out_len)
{
	(void)user;
	g_begin_calls++;
	g_read_reach = 0u;
	g_encode_rc = device_encode();
	if (g_encode_rc != 0) {
		*out_len = 0u;
		return g_encode_rc;
	}
	*out_len = g_blob_len;
	return 0;
}

/** Mirrors sts_disc_phase_read(): serve the frozen blob by offset. */
static int dev_read(void *user, uint32_t off, uint8_t *buf, uint32_t cap,
		    uint32_t *out_n)
{
	uint32_t n;

	(void)user;
	if ((off + cap) > g_read_reach) {
		g_read_reach = off + cap;
	}
	TEST_ASSERT_TRUE_MESSAGE(off <= g_blob_len,
				 "PHASE_EXPORT read started past the record");
	n = g_blob_len - off;
	if (n > cap) {
		n = cap;
	}
	memcpy(buf, &g_blob[off], n);
	*out_n = n;
	return 0;
}

static const mcp_phase_port_t g_port = { .begin = dev_begin,
					 .read = dev_read,
					 .user = NULL };

/* ------------------------------------------------------------------------- */
/* Transmit capture + engine (same wire harness as test_mcp_phase_export.c)   */
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

static void engine_up(void)
{
	mcp_wiring_t w;

	memset(&w, 0, sizeof(w));
	w.clock = &g_clock;
	w.log = &g_log;
	w.phase = &g_port;
	w.tx = tx_cb;
	memcpy(w.ident.model, "STS1000", 7);

	TEST_ASSERT_EQUAL_INT(0, logr_init(&g_log, g_log_slots, LOG_CAP));
	TEST_ASSERT_EQUAL_INT(0, mcp_init(&g_mcp, &w));
	g_seq = 0U;
	g_txn = 0U;
	g_now = 0U;
}

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

static void ask(uint32_t off)
{
	uint8_t req[4];

	bytes_put_le32(req, off);
	feed_req(MCP_CMD_PHASE_EXPORT, req, 4U);
}

static void ask_ok(uint32_t off)
{
	ask(off);
	TEST_ASSERT_EQUAL_HEX8((uint8_t)MCP_OK, rsp_status());
	TEST_ASSERT_EQUAL_UINT32(off, rsp_offset());
}

/* ------------------------------------------------------------------------- */
/* Host side: the bench tool                                                 */
/* ------------------------------------------------------------------------- */

static uint8_t  g_host[PHASE_BLOB_CAP];
static uint32_t g_host_len;
static unsigned int g_host_chunks;

/*
 * Fetch the whole record over PHASE_EXPORT, exactly as a bench tool would:
 * start at 0, advance by what each response actually carried, stop when `more`
 * clears. @p stop_after limits the number of chunks taken (0 = no limit), which
 * is how the truncation case builds a short record without touching the engine.
 */
static void host_fetch(unsigned int stop_after)
{
	uint32_t off = 0u;
	unsigned int rounds = 0u;
	uint8_t more;

	memset(g_host, 0, sizeof(g_host));
	g_host_len = 0u;
	g_host_chunks = 0u;

	do {
		uint16_t dn;

		ask_ok(off);
		dn = rsp_data_len();
		more = rsp_more();
		TEST_ASSERT_TRUE(dn <= CHUNK);
		TEST_ASSERT_TRUE(((uint64_t)off + dn) <= sizeof(g_host));
		memcpy(&g_host[off], rsp_data(), dn);
		off += dn;
		rounds++;
		TEST_ASSERT_TRUE_MESSAGE(rounds < 32u,
					 "PHASE_EXPORT did not terminate");
		if ((stop_after != 0u) && (rounds >= stop_after)) {
			break;
		}
	} while (more != 0U);

	g_host_len = off;
	g_host_chunks = rounds;

	TEST_ASSERT_TRUE_MESSAGE(g_read_reach <= g_blob_len,
				 "PHASE_EXPORT asked the port past the end "
				 "of the record");
}

/*
 * The transport assertion, made before anything numerical looks at the bytes.
 *
 * A defect anywhere in the chunking — a dropped chunk, a chunk written at the
 * wrong offset, a byte mangled by the framing — lands here and names the byte,
 * instead of surfacing three assertions later as an ADEV that is merely
 * "wrong".
 */
static void assert_reassembly_is_byte_identical(void)
{
	TEST_ASSERT_EQUAL_UINT32_MESSAGE(g_blob_len, g_host_len,
					 "reassembled record is the wrong length");
	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(g_blob, g_host, g_blob_len,
					     "reassembled record differs from "
					     "the encoded one");
}

/*
 * Exact-zero assertion with the number in the failure message, written by hand
 * for test_stats.c's reason: Unity's double support is compiled out
 * (UNITY_EXCLUDE_DOUBLE is the default in unity_internals.h), so
 * TEST_ASSERT_EQUAL_DOUBLE fails with "Unity Double Precision Disabled" rather
 * than comparing anything. `== 0.0` is the right comparison here and not a
 * tolerance: stats_adev() guarantees exactly 0.0 on exactly linear phase.
 */
static void assert_exact_zero(double got, const char *what)
{
	static char msg[128];

	if (got == 0.0) {
		return;
	}
	(void)snprintf(msg, sizeof(msg), "%s: expected exactly 0.0, got %.17g",
		       what, got);
	TEST_FAIL_MESSAGE(msg);
}

static int64_t g_x_ps[DISC_ADEV_CAP];
static int64_t g_raw_ps[DISC_ADEV_CAP];

/** Decode the reassembled record's header and both series. Returns n. */
static size_t host_decode(phase_rec_meta_t *m)
{
	size_t n = 0u;

	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(g_host, g_host_len, m));
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_samples(g_host, g_host_len,
							  g_x_ps,
							  DISC_ADEV_CAP, &n));
	TEST_ASSERT_EQUAL_size_t(m->n, n);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_raw(g_host, g_host_len,
						      g_raw_ps, DISC_ADEV_CAP,
						      &n));
	TEST_ASSERT_EQUAL_size_t(m->n, n);
	return n;
}

/* ------------------------------------------------------------------------- */

void setUp(void)
{
	memset(g_blob, 0, sizeof(g_blob));
	g_blob_len = 0u;
	g_blob_cap = sizeof(g_blob);
	g_encode_rc = 0;
	g_begin_calls = 0u;
	g_read_reach = 0u;
	g_host_len = 0u;
	g_host_chunks = 0u;
	engine_up();
}

/* ------------------------------------------------------------------- gap 1 */

/*
 * The composed path, end to end, against a closed-form answer.
 *
 * A pure frequency offset is exactly linear in phase, and ADEV annihilates it
 * exactly. Every layer in the chain gets to corrupt that: the ring's float
 * store, the ns->ps conversion, the encoder's little-endian int64 packing, the
 * chunker's offsets, the COBS/CRC framing, the decoder's stride. If the number
 * that comes out the far end is 0.0 at every tau, none of them did.
 */
static void test_a_pure_frequency_offset_survives_the_path_with_zero_adev(void)
{
	static const uint32_t m_tab[] = { 1u, 2u, 4u, 8u, 16u, 32u, 64u, 128u };
	phase_rec_meta_t m;
	stats_saw_t saw;
	size_t n;
	size_t i;

	drive(false);
	host_fetch(0u);

	/* Transport first, so a transport defect reports as one. */
	assert_reassembly_is_byte_identical();

	/* The real v3 size, and the chunk count that follows from it. */
	TEST_ASSERT_EQUAL_UINT32(8292u, g_blob_len);
	TEST_ASSERT_EQUAL_UINT(9u, g_host_chunks);
	TEST_ASSERT_EQUAL_UINT(1u, g_begin_calls);

	n = host_decode(&m);

	TEST_ASSERT_EQUAL_UINT8(PHASE_REC_VER, m.ver);
	TEST_ASSERT_EQUAL_HEX8((uint8_t)(PHASE_REC_F_FULL | PHASE_REC_F_RAW |
					 PHASE_REC_F_IDENT),
			       m.flags);
	TEST_ASSERT_EQUAL_UINT16(DISC_ADEV_CAP, m.n);
	TEST_ASSERT_EQUAL_UINT32(DISC_ADEV_TAU0_NS, m.tau0_ns);
	TEST_ASSERT_EQUAL_UINT32(0u, m.gaps);
	/* Identity survived the split across the header and the trailing block,
	 * which sit 8 KB apart in the record and in different chunks. */
	TEST_ASSERT_TRUE(m.has_ident);
	TEST_ASSERT_EQUAL_UINT32(g_snap.epoch, m.epoch);
	TEST_ASSERT_EQUAL_UINT64(g_snap.seq0, m.seq0);
	/* TICKS samples pushed, DISC_ADEV_CAP retained: the window starts where
	 * the eviction left it. */
	TEST_ASSERT_EQUAL_UINT64((uint64_t)(TICKS - DISC_ADEV_CAP), m.seq0);

	/*
	 * The series itself: an exact arithmetic progression whose common
	 * difference is the frequency offset. Asserted sample by sample rather
	 * than by its endpoints, because a single transposed or duplicated
	 * sample leaves the endpoints intact and is exactly the kind of defect
	 * a chunk boundary produces.
	 */
	TEST_ASSERT_EQUAL_size_t(DISC_ADEV_CAP, n);
	for (i = 0u; i < n; i++) {
		int64_t want = (int64_t)(m.seq0 + (uint64_t)i) *
			       (int64_t)RAMP_NS_PER_S * INT64_C(1000);

		TEST_ASSERT_EQUAL_INT64(want, g_x_ps[i]);
		/* No qErr was reported, so the two series must be identical —
		 * not merely close. */
		TEST_ASSERT_EQUAL_INT64(want, g_raw_ps[i]);
	}

	/* The closed form, at every tau one 512-sample record supports. */
	for (i = 0u; i < (sizeof(m_tab) / sizeof(m_tab[0])); i++) {
		char what[80];
		double a = -1.0;
		double d = -1.0;

		TEST_ASSERT_EQUAL_INT(0, stats_adev(g_x_ps, n, m.tau0_ns,
						    m_tab[i], &a));
		(void)snprintf(what, sizeof(what),
			       "ADEV(m=%u) of the exported ramp",
			       (unsigned int)m_tab[i]);
		assert_exact_zero(a, what);

		/* MDEV sums the same exactly-annihilated second difference. */
		TEST_ASSERT_EQUAL_INT(0, stats_mdev(g_x_ps, n, m.tau0_ns,
						    m_tab[i], &d));
		(void)snprintf(what, sizeof(what),
			       "MDEV(m=%u) of the exported ramp",
			       (unsigned int)m_tab[i]);
		assert_exact_zero(d, what);
	}

	/* And §14's other consumer agrees there was nothing to correct. */
	TEST_ASSERT_EQUAL_INT(0, stats_saw_verdict(g_x_ps, g_raw_ps, n, &saw));
	TEST_ASSERT_EQUAL_size_t(0u, saw.n_applied);
	TEST_ASSERT_EQUAL_INT(STATS_SAW_NONE, saw.verdict);
}

/*
 * The same path carrying the pair §14's sawtooth proof needs.
 *
 * The capture is ramp - saw and the receiver reports saw, so the corrected
 * series is the exactly-linear ramp and the raw one is not. That makes three
 * things assertable at once: the corrected series still gives ADEV 0 after
 * travelling through the qErr path, the raw series does not, and the two
 * arrived in the right halves of the record — a transport that swapped or
 * overlapped them would invert the verdict.
 */
static void test_the_sawtooth_pair_survives_the_path_and_is_judged_corrected(void)
{
	phase_rec_meta_t m;
	stats_saw_t saw;
	double a_corr = -1.0;
	double a_raw = -1.0;
	size_t n;
	size_t i;

	drive(true);
	host_fetch(0u);

	assert_reassembly_is_byte_identical();
	TEST_ASSERT_EQUAL_UINT32(8292u, g_blob_len);

	n = host_decode(&m);
	TEST_ASSERT_EQUAL_size_t(DISC_ADEV_CAP, n);

	for (i = 0u; i < n; i++) {
		uint32_t seq = (uint32_t)(m.seq0 + (uint64_t)i);
		int64_t want = (int64_t)seq * (int64_t)RAMP_NS_PER_S *
			       INT64_C(1000);
		int64_t s = (int64_t)saw_ns(seq) * INT64_C(1000);

		TEST_ASSERT_EQUAL_INT64(want, g_x_ps[i]);
		TEST_ASSERT_EQUAL_INT64(want - s, g_raw_ps[i]);
	}

	TEST_ASSERT_EQUAL_INT(0, stats_adev(g_x_ps, n, m.tau0_ns, 1u,
					    &a_corr));
	assert_exact_zero(a_corr, "ADEV of the sawtooth-corrected series");
	TEST_ASSERT_EQUAL_INT(0, stats_adev(g_raw_ps, n, m.tau0_ns, 1u,
					    &a_raw));
	TEST_ASSERT_TRUE_MESSAGE(a_raw > 0.0,
				 "the uncorrected series carries no sawtooth; "
				 "the pair proves nothing");

	TEST_ASSERT_EQUAL_INT(0, stats_saw_verdict(g_x_ps, g_raw_ps, n, &saw));
	TEST_ASSERT_EQUAL_size_t(n, saw.n_applied);
	TEST_ASSERT_EQUAL_INT(STATS_SAW_CORRECTED, saw.verdict);
}

/*
 * A record the host stopped fetching one chunk early must not decode.
 *
 * This is the failure mode a lost final response produces, and it is the one
 * that matters: the header is intact, its declared sample count is intact, and
 * only the tail is missing. If the decoder trusted the header the tool would
 * analyse 512 samples of which the last hundred bytes were whatever the host's
 * buffer held.
 */
static void test_a_record_truncated_by_one_chunk_is_refused(void)
{
	phase_rec_meta_t m;
	size_t n = 0u;

	drive(false);
	host_fetch(8u); /* the 8 full chunks; the 100-byte tail never arrives */

	TEST_ASSERT_EQUAL_UINT32(8u * CHUNK, g_host_len);
	TEST_ASSERT_TRUE_MESSAGE(g_host_len < g_blob_len,
				 "the fetch was not actually short");
	/* The transport told the truth: it never said the record was complete. */
	TEST_ASSERT_EQUAL_UINT8(1u, rsp_more());

	/*
	 * -EMSGSIZE, not -EBADMSG: the header is not malformed, it is honest
	 * about a record longer than the buffer it arrived in
	 * (phase_rec_decode_hdr's documented split). Both series refuse for the
	 * same reason, so a tool that skipped the header check still cannot
	 * read past what arrived.
	 */
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE,
			      phase_rec_decode_hdr(g_host, g_host_len, &m));
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE,
			      phase_rec_decode_samples(g_host, g_host_len,
						       g_x_ps, DISC_ADEV_CAP,
						       &n));
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE,
			      phase_rec_decode_raw(g_host, g_host_len,
						   g_raw_ps, DISC_ADEV_CAP,
						   &n));
	TEST_ASSERT_EQUAL_size_t(0u, n);
}

/* ------------------------------------------------------------------- gap 2 */

/*
 * The blob cap and what is left of it.
 *
 * PHASE_BLOB_CAP is 8300 and a full paired-and-identified 512-sample record is
 * 8292, so there are EIGHT bytes of headroom. That is not a comfortable margin
 * and this case exists to make any move on either side loud:
 *
 *   - one more sample costs 17 bytes (two int64 plus a gapmap byte every
 *     eighth sample), so DISC_ADEV_CAP cannot grow at all;
 *   - a fourth appended block larger than 8 bytes overflows it;
 *   - disc_thread.c's BUILD_ASSERT restates the layout arithmetic literally
 *     rather than calling phase_rec_size_flags(), so it would NOT notice a new
 *     appended block that phase_rec.h introduced.
 *
 * What saves the last case is the encoder, not the assert:
 * phase_rec_encode_f() refuses with -EMSGSIZE when the record does not fit and
 * writes nothing — see the two cases below. The consequence of overflowing the
 * cap is therefore an export that fails, not a record that is silently short.
 */
static void test_the_blob_cap_headroom_is_eight_bytes(void)
{
	const uint8_t flags = (uint8_t)(PHASE_REC_F_FULL | PHASE_REC_F_RAW |
					PHASE_REC_F_IDENT);
	size_t full = phase_rec_size_flags(DISC_ADEV_CAP, flags);

	TEST_ASSERT_EQUAL_size_t(8292u, full);
	TEST_ASSERT_EQUAL_size_t((size_t)PHASE_BLOB_CAP, sizeof(g_blob));
	TEST_ASSERT_EQUAL_size_t(8u, (size_t)PHASE_BLOB_CAP - full);

	/* The literal layout disc_thread.c's BUILD_ASSERT checks, agreeing with
	 * the format's own length function — for now. */
	TEST_ASSERT_EQUAL_size_t(full,
				 (size_t)PHASE_REC_HDR_LEN +
					 (DISC_ADEV_CAP / 8u) +
					 (2u * DISC_ADEV_CAP * 8u) +
					 (size_t)PHASE_REC_IDENT_LEN);

	/* One more sample does not fit. */
	TEST_ASSERT_TRUE(phase_rec_size_flags(DISC_ADEV_CAP + 1u, flags) >
			 (size_t)PHASE_BLOB_CAP);
}

/*
 * The encoder refuses a record one byte short of fitting, and leaves the
 * destination alone.
 *
 * A truncating encoder here would be the worst defect on this path: the header
 * would be well formed and self-consistent, so phase_rec_decode_hdr() would
 * pass, and the tool would analyse a tail of stale bytes as phase data.
 */
static void test_an_encode_one_byte_short_of_the_cap_is_refused_not_truncated(void)
{
	uint8_t before[PHASE_BLOB_CAP];
	int rc;

	drive(false);

	/* Establish the exact length first. */
	g_blob_cap = sizeof(g_blob);
	TEST_ASSERT_EQUAL_INT(0, device_encode());
	TEST_ASSERT_EQUAL_UINT32(8292u, g_blob_len);

	/* Exactly enough still works. */
	memset(g_blob, 0, sizeof(g_blob));
	g_blob_cap = 8292u;
	TEST_ASSERT_EQUAL_INT(0, device_encode());
	TEST_ASSERT_EQUAL_UINT32(8292u, g_blob_len);

	/* One byte less does not, and writes nothing. */
	memset(g_blob, 0xA5, sizeof(g_blob));
	memcpy(before, g_blob, sizeof(before));
	g_blob_cap = 8291u;
	rc = device_encode();
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE, rc);
	TEST_ASSERT_EQUAL_UINT32(0u, g_blob_len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(before, g_blob, sizeof(before),
					     "a refused encode still wrote to "
					     "the blob");
}

/*
 * And the refusal reaches the operator as an error, not as a short record.
 *
 * The port returns the encoder's -EMSGSIZE from begin(); mcp__port_err() maps
 * anything it does not recognise to MCP_ERR_INTERNAL. The point of the case is
 * the second half: no chunk is served, so there is nothing for a tool to
 * mistake for a record.
 */
static void test_an_export_that_does_not_fit_the_blob_fails_loudly(void)
{
	drive(false);
	g_blob_cap = 8291u;

	ask(0u);
	TEST_ASSERT_EQUAL_HEX8((uint8_t)MCP_ERR_INTERNAL, rsp_status());
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE, g_encode_rc);
	TEST_ASSERT_EQUAL_UINT16(1u, rsp_len()); /* status only, no payload */

	/* The transfer did not start, so a follow-on chunk is refused rather
	 * than served from a stale blob. */
	ask(CHUNK);
	TEST_ASSERT_EQUAL_HEX8((uint8_t)MCP_ERR_OFFSET, rsp_status());
}

int main(void)
{
	UNITY_BEGIN();
	RUN_TEST(test_a_pure_frequency_offset_survives_the_path_with_zero_adev);
	RUN_TEST(test_the_sawtooth_pair_survives_the_path_and_is_judged_corrected);
	RUN_TEST(test_a_record_truncated_by_one_chunk_is_refused);
	RUN_TEST(test_the_blob_cap_headroom_is_eight_bytes);
	RUN_TEST(test_an_encode_one_byte_short_of_the_cap_is_refused_not_truncated);
	RUN_TEST(test_an_export_that_does_not_fit_the_blob_fails_loudly);
	return UNITY_END();
}
