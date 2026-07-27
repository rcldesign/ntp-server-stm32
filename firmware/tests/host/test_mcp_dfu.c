/*
 * STS1000 "Meridian" — core/mcp DFU (FW_*) unit tests.
 *
 * These drive the real command router over a RAM-backed port_image_t whose
 * write path enforces NOR semantics: a byte may only be programmed while it
 * still reads 0xFF. That turns an erase-window bug from "passes in the test,
 * bricks on hardware" into a test failure, which is the whole reason the fake
 * is not a plain memcpy.
 *
 * The upload is driven frame-by-frame through mcp_input(), so what is under
 * test is the wire behaviour a PC tool would see, not an internal API.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "logring/logring.h"
#include "mcp/mcp.h"
#include "util/bytes.h"
#include "util/cobs.h"
#include "util/crc.h"

#include "host_sha256.h"
#include "test_support.h"

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

/* ------------------------------------------------------------------------- */
/* RAM-backed staging slot with NOR write semantics                          */
/* ------------------------------------------------------------------------- */

/*
 * The physical slot is SLOT_PHYS; the port's staging_size() (= im_size) reports
 * STAGE_MAX, which reserves TRAILER_RESERVE at the top for the MCUboot trailer
 * that mark_pending() owns (port_image.h). Core must accept an image of exactly
 * STAGE_MAX and reject STAGE_MAX+1, and must never erase or write into the
 * reserved region — the erase/write bounds checks below are against the
 * physical size so a stray access into the trailer is caught, not masked.
 */
#define SLOT_PHYS       (256U * 1024U)
#define TRAILER_RESERVE (16U * 1024U)
#define STAGE_MAX       (SLOT_PHYS - TRAILER_RESERVE)
#define SLOT_SIZE       STAGE_MAX  /* max image the tests upload */

static uint8_t  g_slot[SLOT_PHYS];
static uint32_t g_slot_cap;
static uint32_t g_erase_calls;
static uint32_t g_erase_bytes;
static uint32_t g_write_calls;
static bool     g_last_flush;
static int      g_erase_rc;
static int      g_write_rc;
static int      g_read_rc;
static int      g_pending_rc;
static int      g_confirm_rc;
static int      g_revert_rc;
static bool     g_write_partial; /* on write failure, program the first half */
static bool     g_pending;
static bool     g_confirmed;
static bool     g_reverted;

static uint32_t im_size(void *ctx)
{
	(void)ctx;
	return g_slot_cap;
}

static int im_erase(void *ctx, uint32_t off, uint32_t len)
{
	(void)ctx;
	if (g_erase_rc != 0) {
		return g_erase_rc;
	}
	/* Physical bound: erasing into the reserved trailer would be a bug. */
	TEST_ASSERT_TRUE((uint64_t)off + len <= sizeof(g_slot));
	memset(&g_slot[off], 0xFF, len);
	g_erase_calls++;
	g_erase_bytes += len;
	return 0;
}

/* Program bytes, enforcing NOR semantics (a byte must still read 0xFF). */
static void nor_program(uint32_t off, const uint8_t *data, size_t len)
{
	size_t i;

	for (i = 0U; i < len; i++) {
		TEST_ASSERT_EQUAL_HEX8_MESSAGE(
			0xFFU, g_slot[off + i],
			"DFU wrote into a region that was never erased");
	}
	memcpy(&g_slot[off], data, len);
}

static int im_write(void *ctx, uint32_t off, const uint8_t *data, size_t len,
		    bool flush)
{
	(void)ctx;
	TEST_ASSERT_TRUE((uint64_t)off + len <= sizeof(g_slot));

	if (g_write_rc != 0) {
		/*
		 * Model a real flash fault mid-program: when asked to fail
		 * partially, commit the first half to flash BEFORE returning
		 * the error. That is what makes a naive retry-at-same-offset
		 * reprogram already-programmed bytes — the NOR check above then
		 * fires unless the engine re-erased first (HIGH-1).
		 */
		if (g_write_partial && (len > 0U)) {
			nor_program(off, data, len / 2U);
		}
		return g_write_rc;
	}

	nor_program(off, data, len);
	g_write_calls++;
	g_last_flush = flush;
	return 0;
}

static int im_read(void *ctx, uint32_t off, uint8_t *data, size_t len)
{
	(void)ctx;
	if (g_read_rc != 0) {
		return g_read_rc;
	}
	TEST_ASSERT_TRUE((uint64_t)off + len <= sizeof(g_slot));
	memcpy(data, &g_slot[off], len);
	return 0;
}

static int im_info(void *ctx, uint8_t slot, port_image_info_t *out)
{
	(void)ctx;
	memset(out, 0, sizeof(*out));
	out->slot = slot;
	if (slot == 0U) {
		out->valid = true;
		out->active = true;
		out->confirmed = g_confirmed;
		out->size = 0x100U;
		out->version[0] = 9U;
	} else {
		out->valid = g_pending;
		out->pending = g_pending;
		out->size = 0x200U;
		out->version[1] = 8U;
	}
	return 0;
}

static int im_mark_pending(void *ctx)
{
	(void)ctx;
	if (g_pending_rc != 0) {
		return g_pending_rc;
	}
	g_pending = true;
	return 0;
}

static int im_confirm(void *ctx)
{
	(void)ctx;
	if (g_confirm_rc != 0) {
		return g_confirm_rc;
	}
	g_confirmed = true;
	return 0;
}

static int im_revert(void *ctx)
{
	(void)ctx;
	if (g_revert_rc != 0) {
		return g_revert_rc;
	}
	g_reverted = true;
	g_pending = false;
	return 0;
}

static port_image_t g_img;

static void img_reset(void)
{
	memset(&g_img, 0, sizeof(g_img));
	g_img.staging_size = im_size;
	g_img.staging_erase = im_erase;
	g_img.staging_write = im_write;
	g_img.staging_read = im_read;
	g_img.image_info = im_info;
	g_img.mark_pending = im_mark_pending;
	g_img.confirm_active = im_confirm;
	g_img.request_revert = im_revert;
	g_img.reboot = NULL;
	g_img.ctx = NULL;

	memset(g_slot, 0x00, sizeof(g_slot)); /* not erased */
	g_slot_cap = STAGE_MAX;               /* staging_size() excludes trailer */
	g_erase_calls = 0U;
	g_erase_bytes = 0U;
	g_write_calls = 0U;
	g_last_flush = false;
	g_erase_rc = 0;
	g_write_rc = 0;
	g_write_partial = false;
	g_read_rc = 0;
	g_pending_rc = 0;
	g_confirm_rc = 0;
	g_revert_rc = 0;
	g_pending = false;
	g_confirmed = false;
	g_reverted = false;
}

/* ------------------------------------------------------------------------- */
/* Engine under test                                                         */
/* ------------------------------------------------------------------------- */

#define LOG_CAP 32U
#define ERASE_GRAN 4096U
#define WRITE_BLOCK 16U

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
 * treats auth as not required (mcp.h), which keeps this suite about the DFU
 * state machine. The auth gate over FW_* is covered in test_mcp.c.
 */
static void engine_up(bool with_sha)
{
	mcp_wiring_t w;

	memset(&w, 0, sizeof(w));
	w.clock = &g_clock;
	w.sha = with_sha ? host_sha256_stream() : NULL;
	w.img = &g_img;
	w.log = &g_log;
	w.tx = tx_cb;
	w.dfu_erase_gran = ERASE_GRAN;
	w.dfu_write_block = WRITE_BLOCK;
	memcpy(w.ident.model, "STS1000", 7);

	TEST_ASSERT_EQUAL_INT(0, logr_init(&g_log, g_log_slots, LOG_CAP));
	TEST_ASSERT_EQUAL_INT(0, mcp_init(&g_mcp, &w));
	g_seq = 0U;
	g_txn = 0U;
	g_now = 0U;
}

void setUp(void)
{
	img_reset();
	engine_up(true);
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

static void expect(uint8_t status)
{
	TEST_ASSERT_EQUAL_HEX8(status, rsp_status());
}

/* ------------------------------------------------------------------------- */
/* Image fixture                                                             */
/* ------------------------------------------------------------------------- */

#define IMAGE_SIZE (128U * 1024U)

static uint8_t g_image[IMAGE_SIZE];
static uint8_t g_image_sha[32];

/* A plausible MCUboot image: the header magic followed by pseudorandom bytes. */
static void make_image(uint32_t seed, bool valid_magic)
{
	test_rng_t rng;

	test_rng_init(&rng, seed);
	test_rng_fill(&rng, g_image, sizeof(g_image));
	bytes_put_le32(g_image, valid_magic ? MCP_MCUBOOT_MAGIC : 0xDEADBEEFU);
	host_sha256(g_image, sizeof(g_image), g_image_sha);
}

static void fw_begin(uint32_t size, const uint8_t sha[32])
{
	uint8_t req[36];

	bytes_put_le32(req, size);
	memcpy(&req[4], sha, 32);
	feed_req(MCP_CMD_FW_BEGIN, req, sizeof(req));
}

static uint32_t rsp_next_expected(void)
{
	TEST_ASSERT_TRUE(rsp_len() >= 5U);
	return bytes_get_le32(&rsp_pay()[1]);
}

static void fw_data(uint32_t off, const uint8_t *data, uint16_t n)
{
	static uint8_t req[4U + MCP_FW_CHUNK_MAX + 8U];

	bytes_put_le32(req, off);
	memcpy(&req[4], data, n);
	feed_req(MCP_CMD_FW_DATA, req, (uint16_t)(4U + n));
}

/* Upload [from, to) in @p chunk-sized pieces, asserting each is accepted. */
static void upload_range(uint32_t from, uint32_t to, uint32_t chunk)
{
	uint32_t off;

	for (off = from; off < to; off += chunk) {
		uint32_t n = to - off;

		if (n > chunk) {
			n = chunk;
		}
		fw_data(off, &g_image[off], (uint16_t)n);
		expect((uint8_t)MCP_OK);
		TEST_ASSERT_EQUAL_UINT32(off + n, rsp_next_expected());
	}
}

/* ------------------------------------------------------------------------- */
/* Happy path                                                                */
/* ------------------------------------------------------------------------- */

static void test_full_upload_verifies_and_stages(void)
{
	const mcp_dfu_t *d;

	make_image(1U, true);

	fw_begin(IMAGE_SIZE, g_image_sha);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT32(0U, rsp_next_expected());
	TEST_ASSERT_EQUAL_UINT32(MCP_FW_CHUNK_MAX,
				 bytes_get_le32(&rsp_pay()[5]));
	TEST_ASSERT_EQUAL_UINT32(WRITE_BLOCK, bytes_get_le32(&rsp_pay()[9]));
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_pay()[13]); /* not a resume */

	/* Erase is progressive: FW_BEGIN clears one granule, not the slot. */
	TEST_ASSERT_EQUAL_UINT32(1U, g_erase_calls);
	TEST_ASSERT_EQUAL_UINT32(ERASE_GRAN, g_erase_bytes);

	upload_range(0U, IMAGE_SIZE, MCP_FW_CHUNK_MAX);

	d = mcp_dfu_status(&g_mcp);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_DFU_ACTIVE, d->state);
	TEST_ASSERT_EQUAL_UINT32(IMAGE_SIZE, d->written);
	TEST_ASSERT_EQUAL_UINT32(IMAGE_SIZE / MCP_FW_CHUNK_MAX, d->chunks);
	TEST_ASSERT_TRUE(g_last_flush); /* the final chunk asked for a flush */

	/* The erase window covered the image and stopped at its last sector:
	 * an erase-ahead that ran past the image would clear flash the next
	 * upload has to erase again. IMAGE_SIZE is a whole number of granules,
	 * so the window lands exactly on it. */
	TEST_ASSERT_EQUAL_UINT32(IMAGE_SIZE, g_erase_bytes);

	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT32(IMAGE_SIZE, bytes_get_le32(&rsp_pay()[1]));

	TEST_ASSERT_TRUE(g_pending);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_DFU_VERIFIED,
				mcp_dfu_status(&g_mcp)->state);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_image, g_slot, IMAGE_SIZE);
}

static void test_odd_sized_image_with_odd_chunks(void)
{
	/* A size that is neither a chunk nor an erase-granule multiple: the
	 * final short write and the last partially-used sector are the two
	 * places an off-by-one hides. The chunk is a write-block multiple (the
	 * only sizes non-final chunks may take); the odd tail rides in the
	 * final chunk, which is allowed to be short. */
	const uint32_t size = (3U * ERASE_GRAN) + 17U;
	uint8_t sha[32];

	make_image(2U, true);
	host_sha256(g_image, size, sha);

	fw_begin(size, sha);
	expect((uint8_t)MCP_OK);
	upload_range(0U, size, 20U * WRITE_BLOCK); /* 320-byte aligned chunks */

	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_TRUE(g_pending);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_image, g_slot, size);
	/* Erased whole sectors only, and never past the image's last one. */
	TEST_ASSERT_EQUAL_UINT32(4U * ERASE_GRAN, g_erase_bytes);
}

/* ------------------------------------------------------------------------- */
/* Retransmission, gaps and resume                                           */
/* ------------------------------------------------------------------------- */

static void test_retransmitted_chunk_is_absorbed(void)
{
	uint32_t writes;

	make_image(3U, true);
	fw_begin(IMAGE_SIZE, g_image_sha);
	upload_range(0U, 4096U, 1024U);
	writes = g_write_calls;

	/* A duplicate of the chunk whose ACK went missing. */
	fw_data(3072U, &g_image[3072], 1024U);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT32(4096U, rsp_next_expected());
	TEST_ASSERT_EQUAL_UINT32(writes, g_write_calls); /* no reprogramming */
	TEST_ASSERT_EQUAL_UINT32(1U, mcp_dfu_status(&g_mcp)->retransmits);

	/* An older chunk entirely behind the frontier, likewise. */
	fw_data(0U, &g_image[0], 1024U);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT32(4096U, rsp_next_expected());
	TEST_ASSERT_EQUAL_UINT32(writes, g_write_calls);

	/* And the upload continues from where it was. */
	upload_range(4096U, 8192U, 1024U);
}

static void test_gap_is_refused_with_the_expected_offset(void)
{
	make_image(4U, true);
	fw_begin(IMAGE_SIZE, g_image_sha);
	upload_range(0U, 2048U, 1024U);

	/* Ahead of the frontier. */
	fw_data(4096U, &g_image[4096], 1024U);
	expect((uint8_t)MCP_ERR_OFFSET);
	TEST_ASSERT_EQUAL_UINT32(2048U, rsp_next_expected());

	/* Straddling it: the overlap would have to reprogram flash. */
	fw_data(1536U, &g_image[1536], 1024U);
	expect((uint8_t)MCP_ERR_OFFSET);
	TEST_ASSERT_EQUAL_UINT32(2048U, rsp_next_expected());

	/* Rewinding to the reported offset recovers. */
	upload_range(2048U, 4096U, 1024U);
	TEST_ASSERT_EQUAL_UINT32(4096U, mcp_dfu_status(&g_mcp)->written);
}

static void test_resume_after_reconnect_keeps_the_frontier(void)
{
	uint32_t erases;
	uint32_t bytes;

	make_image(5U, true);
	fw_begin(IMAGE_SIZE, g_image_sha);
	upload_range(0U, 16384U, 1024U);
	erases = g_erase_calls;
	bytes = g_erase_bytes;

	/* The link drops. The session survives that on purpose. */
	mcp_reset_session(&g_mcp);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_DFU_ACTIVE,
				mcp_dfu_status(&g_mcp)->state);

	fw_begin(IMAGE_SIZE, g_image_sha);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT32(16384U, rsp_next_expected());
	TEST_ASSERT_EQUAL_UINT8(1U, rsp_pay()[13]); /* resumed */
	TEST_ASSERT_EQUAL_UINT32(erases, g_erase_calls);
	TEST_ASSERT_EQUAL_UINT32(bytes, g_erase_bytes);

	upload_range(16384U, IMAGE_SIZE, MCP_FW_CHUNK_MAX);
	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_TRUE(g_pending);
}

static void test_a_different_image_restarts_the_session(void)
{
	uint8_t other[32];

	make_image(6U, true);
	fw_begin(IMAGE_SIZE, g_image_sha);
	upload_range(0U, 8192U, 1024U);

	/* Same size, different content: nothing about the old transfer is
	 * reusable, so the frontier goes back to zero. */
	memcpy(other, g_image_sha, sizeof(other));
	other[0] ^= 0x01U;
	fw_begin(IMAGE_SIZE, other);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT32(0U, rsp_next_expected());
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_pay()[13]);

	/* Same content, different size: also a restart. */
	fw_begin(IMAGE_SIZE, g_image_sha);
	TEST_ASSERT_EQUAL_UINT32(0U, rsp_next_expected());
	fw_begin(IMAGE_SIZE / 2U, g_image_sha);
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_pay()[13]);
}

static void test_idle_session_suspends_and_resumes(void)
{
	make_image(7U, true);
	fw_begin(IMAGE_SIZE, g_image_sha);
	upload_range(0U, 2048U, 1024U);

	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, MCP_DFU_TIMEOUT_MS - 1U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_DFU_ACTIVE,
				mcp_dfu_status(&g_mcp)->state);

	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, MCP_DFU_TIMEOUT_MS));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_DFU_SUSPENDED,
				mcp_dfu_status(&g_mcp)->state);
	TEST_ASSERT_EQUAL_UINT32(1U, mcp_dfu_status(&g_mcp)->timeouts);

	/* A suspended session accepts no data until it is re-established. */
	g_now = MCP_DFU_TIMEOUT_MS;
	fw_data(2048U, &g_image[2048], 1024U);
	expect((uint8_t)MCP_ERR_STATE);

	fw_begin(IMAGE_SIZE, g_image_sha);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT8(1U, rsp_pay()[13]);
	TEST_ASSERT_EQUAL_UINT32(2048U, rsp_next_expected());

	/* The clock keeps running; the refreshed session must not re-suspend
	 * immediately. */
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, MCP_DFU_TIMEOUT_MS + 10U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_DFU_ACTIVE,
				mcp_dfu_status(&g_mcp)->state);
}

/* ------------------------------------------------------------------------- */
/* Verification failures                                                     */
/* ------------------------------------------------------------------------- */

static void test_sha_mismatch_is_rejected_and_not_staged(void)
{
	uint8_t wrong[32];

	make_image(8U, true);
	memcpy(wrong, g_image_sha, sizeof(wrong));
	wrong[31] ^= 0x80U;

	fw_begin(IMAGE_SIZE, wrong);
	upload_range(0U, IMAGE_SIZE, MCP_FW_CHUNK_MAX);

	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_ERR_VERIFY);
	TEST_ASSERT_FALSE(g_pending);

	/* The staged bytes are known bad, so the session is gone: a resume
	 * into them must be impossible. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_DFU_IDLE,
				mcp_dfu_status(&g_mcp)->state);
	TEST_ASSERT_EQUAL_UINT32(0U, mcp_dfu_status(&g_mcp)->written);

	fw_begin(IMAGE_SIZE, wrong);
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_pay()[13]); /* no resume offered */
}

static void test_missing_mcuboot_magic_is_rejected(void)
{
	make_image(9U, false);

	fw_begin(IMAGE_SIZE, g_image_sha);
	upload_range(0U, IMAGE_SIZE, MCP_FW_CHUNK_MAX);

	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_ERR_VERIFY);
	TEST_ASSERT_FALSE(g_pending);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_DFU_IDLE,
				mcp_dfu_status(&g_mcp)->state);
}

static void test_a_runt_image_cannot_carry_a_header(void)
{
	uint8_t tiny[2] = { 0x3D, 0xB8 };
	uint8_t sha[32];

	host_sha256(tiny, sizeof(tiny), sha);
	fw_begin(sizeof(tiny), sha);
	expect((uint8_t)MCP_OK);
	fw_data(0U, tiny, sizeof(tiny));
	expect((uint8_t)MCP_OK);

	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_ERR_VERIFY);
	TEST_ASSERT_FALSE(g_pending);
}

static void test_end_before_the_image_is_complete(void)
{
	make_image(10U, true);
	fw_begin(IMAGE_SIZE, g_image_sha);
	upload_range(0U, 1024U, 1024U);

	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_ERR_STATE);
	TEST_ASSERT_FALSE(g_pending);

	/* And the transfer is still usable afterwards. */
	upload_range(1024U, IMAGE_SIZE, MCP_FW_CHUNK_MAX);
	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_OK);
}

static void test_end_is_idempotent(void)
{
	make_image(11U, true);
	fw_begin(IMAGE_SIZE, g_image_sha);
	upload_range(0U, IMAGE_SIZE, MCP_FW_CHUNK_MAX);

	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_OK);
	g_pending = false;

	/* Re-verifying an already-verified session re-marks it, so a lost
	 * FW_END response costs one round trip, not a re-upload. */
	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_TRUE(g_pending);
}

/* ------------------------------------------------------------------------- */
/* Argument and state validation                                             */
/* ------------------------------------------------------------------------- */

static void test_begin_argument_validation(void)
{
	uint8_t sha[32];

	memset(sha, 0, sizeof(sha));

	feed_req(MCP_CMD_FW_BEGIN, sha, 32U); /* wrong payload length */
	expect((uint8_t)MCP_ERR_ARG);

	fw_begin(0U, sha);
	expect((uint8_t)MCP_ERR_ARG);

	fw_begin(SLOT_SIZE + 1U, sha);
	expect((uint8_t)MCP_ERR_NOSPC);

	/* A port that reports no staging slot at all. */
	g_slot_cap = 0U;
	fw_begin(16U, sha);
	expect((uint8_t)MCP_ERR_NOSPC);
	g_slot_cap = SLOT_SIZE;

	/* An erase that fails takes the session with it. */
	g_erase_rc = -EIO;
	fw_begin(16U, sha);
	expect((uint8_t)MCP_ERR_INTERNAL);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_DFU_IDLE,
				mcp_dfu_status(&g_mcp)->state);
	g_erase_rc = 0;
}

static void test_data_argument_validation(void)
{
	static uint8_t big[4U + MCP_FW_CHUNK_MAX + 1U];

	make_image(12U, true);

	/* Before any FW_BEGIN there is no session. */
	fw_data(0U, g_image, 16U);
	expect((uint8_t)MCP_ERR_STATE);

	fw_begin(IMAGE_SIZE, g_image_sha);

	feed_req(MCP_CMD_FW_DATA, big, 4U); /* offset but no data */
	expect((uint8_t)MCP_ERR_ARG);

	memset(big, 0, sizeof(big));
	bytes_put_le32(big, 0U);
	feed_req(MCP_CMD_FW_DATA, big, (uint16_t)sizeof(big));
	expect((uint8_t)MCP_ERR_ARG); /* chunk above MCP_FW_CHUNK_MAX */

	/* Past the declared end of the image. */
	fw_data(IMAGE_SIZE, g_image, 16U);
	expect((uint8_t)MCP_ERR_ARG);
	fw_data(IMAGE_SIZE - 8U, g_image, 16U);
	expect((uint8_t)MCP_ERR_ARG);
	fw_data(IMAGE_SIZE + 4096U, g_image, 16U);
	expect((uint8_t)MCP_ERR_ARG);
}

static void test_port_failures_are_reported(void)
{
	make_image(13U, true);
	fw_begin(IMAGE_SIZE, g_image_sha);

	/* An erase failure is retryable in place: nothing was written and erase
	 * is idempotent, so the frontier holds and a retry recovers (this is
	 * why the erase path, unlike the write path, does not reset). Push the
	 * frontier far enough that the next chunk needs a fresh window. */
	upload_range(0U, 8192U, 1024U);
	g_erase_rc = -EIO;
	fw_data(8192U, &g_image[8192], 1024U);
	expect((uint8_t)MCP_ERR_INTERNAL);
	TEST_ASSERT_EQUAL_UINT32(8192U, rsp_next_expected());
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_DFU_ACTIVE,
				mcp_dfu_status(&g_mcp)->state);
	g_erase_rc = 0;

	upload_range(8192U, IMAGE_SIZE, MCP_FW_CHUNK_MAX);

	/* Read-back failure during the SHA stream. */
	g_read_rc = -EIO;
	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_ERR_INTERNAL);
	g_read_rc = 0;

	/* And a bootloader that refuses to mark the slot. */
	g_pending_rc = -EBUSY;
	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_ERR_BUSY);
	TEST_ASSERT_FALSE(g_pending);
	g_pending_rc = 0;

	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_OK);
}

/*
 * HIGH-1: a staging_write fault must abandon the session so the tool cannot
 * retry into half-programmed flash. The fake programs the first half of the
 * chunk before reporting the error, exactly the state that would trip the NOR
 * check on a naive retry. The correct behaviour is a reset: the session drops
 * to IDLE, in-place FW_DATA is refused, and a fresh FW_BEGIN re-erases (not a
 * resume) so the eventual upload succeeds and stages byte-correct content.
 */
static void test_write_failure_resets_so_rebegin_re_erases(void)
{
	uint32_t erases_before;

	make_image(21U, true);
	fw_begin(IMAGE_SIZE, g_image_sha);
	upload_range(0U, 4096U, 1024U);
	erases_before = g_erase_calls;

	g_write_rc = -EIO;
	g_write_partial = true;
	fw_data(4096U, &g_image[4096], 1024U);
	expect((uint8_t)MCP_ERR_INTERNAL);
	g_write_rc = 0;
	g_write_partial = false;

	/* The session is gone, so a bare retry at the same offset is refused —
	 * the tool cannot reprogram the half-written block. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_DFU_IDLE,
				mcp_dfu_status(&g_mcp)->state);
	fw_data(4096U, &g_image[4096], 1024U);
	expect((uint8_t)MCP_ERR_STATE);

	/* A fresh FW_BEGIN is NOT treated as a resume (state was reset), so it
	 * re-erases from zero and the retry succeeds with correct content. */
	fw_begin(IMAGE_SIZE, g_image_sha);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT8(0U, rsp_pay()[13]); /* not a resume */
	TEST_ASSERT_EQUAL_UINT32(0U, rsp_next_expected());
	TEST_ASSERT_TRUE(g_erase_calls > erases_before);

	upload_range(0U, IMAGE_SIZE, MCP_FW_CHUNK_MAX);
	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_TRUE(g_pending);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_image, g_slot, IMAGE_SIZE);
}

/*
 * HIGH-2: staging_size() is the max IMAGE size (trailer excluded). An image of
 * exactly that size is accepted; one byte larger is refused so the trailer
 * that mark_pending writes is never overwritten.
 */
static void test_begin_honours_the_trailer_reserve(void)
{
	uint8_t sha[32];

	memset(sha, 0, sizeof(sha));

	fw_begin(STAGE_MAX + 1U, sha);
	expect((uint8_t)MCP_ERR_NOSPC);

	fw_begin(STAGE_MAX, sha);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT32(STAGE_MAX, mcp_dfu_status(&g_mcp)->total);
}

/*
 * MEDIUM-8: only the final chunk may be shorter than a flash write block; a
 * non-final chunk whose length is not a write-block multiple is refused so the
 * port never has to buffer a mid-stream partial block.
 */
static void test_non_final_chunk_must_be_write_block_aligned(void)
{
	make_image(22U, true);
	fw_begin(IMAGE_SIZE, g_image_sha);

	/* 1000 is not a multiple of 16 and does not reach the end. */
	fw_data(0U, g_image, 1000U);
	expect((uint8_t)MCP_ERR_ARG);
	TEST_ASSERT_EQUAL_UINT32(0U, rsp_next_expected());

	/* The aligned form of the same prefix is accepted. */
	fw_data(0U, g_image, 1008U); /* 63 * 16 */
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT32(1008U, rsp_next_expected());

	/* A short *final* chunk is fine even though it is not block-aligned. */
	upload_range(1008U, IMAGE_SIZE - 5U, MCP_FW_CHUNK_MAX);
	fw_data(IMAGE_SIZE - 5U, &g_image[IMAGE_SIZE - 5U], 5U);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT32(IMAGE_SIZE, rsp_next_expected());
}

static void test_trailing_payloads_are_rejected(void)
{
	uint8_t junk = 0U;

	feed_req(MCP_CMD_FW_INFO, &junk, 1U);
	expect((uint8_t)MCP_ERR_ARG);
	feed_req(MCP_CMD_FW_END, &junk, 1U);
	expect((uint8_t)MCP_ERR_ARG);
	feed_req(MCP_CMD_FW_CONFIRM, &junk, 1U);
	expect((uint8_t)MCP_ERR_ARG);
	feed_req(MCP_CMD_FW_REVERT, &junk, 1U);
	expect((uint8_t)MCP_ERR_ARG);
}

/* ------------------------------------------------------------------------- */
/* Slot management                                                           */
/* ------------------------------------------------------------------------- */

static void test_fw_info_reports_slots_and_session(void)
{
	const uint8_t *p;

	make_image(14U, true);
	fw_begin(IMAGE_SIZE, g_image_sha);
	upload_range(0U, 3072U, 1024U);

	feed_req(MCP_CMD_FW_INFO, NULL, 0U);
	expect((uint8_t)MCP_OK);
	p = rsp_pay();

	TEST_ASSERT_EQUAL_UINT8(2U, p[1]);
	TEST_ASSERT_EQUAL_UINT8(0U, p[2]);
	TEST_ASSERT_EQUAL_HEX8(MCP_SLOT_VALID | MCP_SLOT_ACTIVE, p[3]);
	TEST_ASSERT_EQUAL_UINT32(0x100U, bytes_get_le32(&p[4]));
	TEST_ASSERT_EQUAL_UINT32(9U, bytes_get_le32(&p[8]));

	TEST_ASSERT_EQUAL_UINT8(1U, p[24]);
	TEST_ASSERT_EQUAL_HEX8(0U, p[25]); /* nothing staged yet */
	TEST_ASSERT_EQUAL_UINT32(8U, bytes_get_le32(&p[34]));

	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_DFU_ACTIVE, p[46]);
	TEST_ASSERT_EQUAL_UINT32(IMAGE_SIZE, bytes_get_le32(&p[47]));
	TEST_ASSERT_EQUAL_UINT32(3072U, bytes_get_le32(&p[51]));
	TEST_ASSERT_EQUAL_UINT32(MCP_FW_CHUNK_MAX, bytes_get_le32(&p[55]));
	TEST_ASSERT_EQUAL_UINT32(WRITE_BLOCK, bytes_get_le32(&p[59]));
	TEST_ASSERT_EQUAL_UINT16(63U, rsp_len());

	/* Once staged, the slot-1 flags reflect it. */
	upload_range(3072U, IMAGE_SIZE, MCP_FW_CHUNK_MAX);
	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_OK);
	feed_req(MCP_CMD_FW_INFO, NULL, 0U);
	p = rsp_pay();
	TEST_ASSERT_EQUAL_HEX8(MCP_SLOT_VALID | MCP_SLOT_PENDING, p[25]);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_DFU_VERIFIED, p[46]);
}

static void test_confirm_and_revert(void)
{
	feed_req(MCP_CMD_FW_CONFIRM, NULL, 0U);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_TRUE(g_confirmed);

	g_confirm_rc = -EIO;
	feed_req(MCP_CMD_FW_CONFIRM, NULL, 0U);
	expect((uint8_t)MCP_ERR_INTERNAL);
	g_confirm_rc = 0;

	/* A revert abandons whatever this session staged. */
	make_image(15U, true);
	fw_begin(IMAGE_SIZE, g_image_sha);
	upload_range(0U, 1024U, 1024U);

	feed_req(MCP_CMD_FW_REVERT, NULL, 0U);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_TRUE(g_reverted);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_DFU_IDLE,
				mcp_dfu_status(&g_mcp)->state);

	g_revert_rc = -EBUSY;
	feed_req(MCP_CMD_FW_REVERT, NULL, 0U);
	expect((uint8_t)MCP_ERR_BUSY);
	g_revert_rc = 0;
}

/* ------------------------------------------------------------------------- */
/* Missing wiring                                                            */
/* ------------------------------------------------------------------------- */

static void test_without_an_image_port_everything_is_unsupported(void)
{
	mcp_wiring_t w;
	static const uint8_t cmds[] = {
		MCP_CMD_FW_INFO, MCP_CMD_FW_BEGIN, MCP_CMD_FW_DATA,
		MCP_CMD_FW_END,  MCP_CMD_FW_CONFIRM, MCP_CMD_FW_REVERT,
	};
	size_t i;

	memset(&w, 0, sizeof(w));
	w.clock = &g_clock;
	w.tx = tx_cb;
	TEST_ASSERT_EQUAL_INT(0, mcp_init(&g_mcp, &w));

	for (i = 0U; i < sizeof(cmds); i++) {
		feed_req(cmds[i], NULL, 0U);
		expect((uint8_t)MCP_ERR_NOTSUP);
	}
}

static void test_missing_port_functions_are_unsupported(void)
{
	uint8_t sha[32];

	memset(sha, 0, sizeof(sha));

	/* No hash port: nothing can be verified, so nothing may be started. */
	engine_up(false);
	fw_begin(16U, sha);
	expect((uint8_t)MCP_ERR_NOTSUP);

	/* No write function: same. */
	engine_up(true);
	g_img.staging_write = NULL;
	fw_begin(16U, sha);
	expect((uint8_t)MCP_ERR_NOTSUP);
	g_img.staging_write = im_write;

	/* No erase function surfaces when the first window is needed. */
	g_img.staging_erase = NULL;
	fw_begin(16U, sha);
	expect((uint8_t)MCP_ERR_NOTSUP);
	g_img.staging_erase = im_erase;

	/* No confirm/revert entry points. */
	g_img.confirm_active = NULL;
	feed_req(MCP_CMD_FW_CONFIRM, NULL, 0U);
	expect((uint8_t)MCP_ERR_NOTSUP);
	g_img.confirm_active = im_confirm;

	g_img.request_revert = NULL;
	feed_req(MCP_CMD_FW_REVERT, NULL, 0U);
	expect((uint8_t)MCP_ERR_NOTSUP);
	g_img.request_revert = im_revert;

	/* No image_info: FW_INFO still answers, with zeroed slots. */
	g_img.image_info = NULL;
	feed_req(MCP_CMD_FW_INFO, NULL, 0U);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_HEX8(0U, rsp_pay()[3]);
	g_img.image_info = im_info;
}

static void test_image_info_failure_is_reported_as_an_empty_slot(void)
{
	feed_req(MCP_CMD_FW_INFO, NULL, 0U);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_HEX8(MCP_SLOT_VALID | MCP_SLOT_ACTIVE, rsp_pay()[3]);
}

static void test_end_without_a_read_back_path(void)
{
	uint8_t tiny[8];
	uint8_t sha[32];

	memset(tiny, 0, sizeof(tiny));
	bytes_put_le32(tiny, MCP_MCUBOOT_MAGIC);
	host_sha256(tiny, sizeof(tiny), sha);

	fw_begin(sizeof(tiny), sha);
	fw_data(0U, tiny, sizeof(tiny));
	expect((uint8_t)MCP_OK);

	g_img.staging_read = NULL;
	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_ERR_NOTSUP);
	g_img.staging_read = im_read;

	g_img.mark_pending = NULL;
	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_ERR_NOTSUP);
	g_img.mark_pending = im_mark_pending;

	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_OK);
}

static void test_single_byte_erase_granularity(void)
{
	mcp_wiring_t w;
	uint8_t tiny[64];
	uint8_t sha[32];

	/* A port whose erase granularity is a byte: align_up must degenerate
	 * cleanly instead of dividing by a granule of 1 forever. */
	memset(&w, 0, sizeof(w));
	w.clock = &g_clock;
	w.sha = host_sha256_stream();
	w.img = &g_img;
	w.log = &g_log;
	w.tx = tx_cb;
	w.dfu_erase_gran = 1U;
	TEST_ASSERT_EQUAL_INT(0, mcp_init(&g_mcp, &w));
	g_seq = 0U;

	memset(tiny, 0x5A, sizeof(tiny));
	bytes_put_le32(tiny, MCP_MCUBOOT_MAGIC);
	host_sha256(tiny, sizeof(tiny), sha);

	fw_begin(sizeof(tiny), sha);
	expect((uint8_t)MCP_OK);
	fw_data(0U, tiny, sizeof(tiny));
	expect((uint8_t)MCP_OK);
	feed_req(MCP_CMD_FW_END, NULL, 0U);
	expect((uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT32(sizeof(tiny), g_erase_bytes);
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_full_upload_verifies_and_stages);
	RUN_TEST(test_odd_sized_image_with_odd_chunks);

	RUN_TEST(test_retransmitted_chunk_is_absorbed);
	RUN_TEST(test_gap_is_refused_with_the_expected_offset);
	RUN_TEST(test_resume_after_reconnect_keeps_the_frontier);
	RUN_TEST(test_a_different_image_restarts_the_session);
	RUN_TEST(test_idle_session_suspends_and_resumes);

	RUN_TEST(test_sha_mismatch_is_rejected_and_not_staged);
	RUN_TEST(test_missing_mcuboot_magic_is_rejected);
	RUN_TEST(test_a_runt_image_cannot_carry_a_header);
	RUN_TEST(test_end_before_the_image_is_complete);
	RUN_TEST(test_end_is_idempotent);

	RUN_TEST(test_begin_argument_validation);
	RUN_TEST(test_data_argument_validation);
	RUN_TEST(test_port_failures_are_reported);
	RUN_TEST(test_trailing_payloads_are_rejected);

	RUN_TEST(test_fw_info_reports_slots_and_session);
	RUN_TEST(test_confirm_and_revert);

	RUN_TEST(test_without_an_image_port_everything_is_unsupported);
	RUN_TEST(test_missing_port_functions_are_unsupported);
	RUN_TEST(test_image_info_failure_is_reported_as_an_empty_slot);
	RUN_TEST(test_end_without_a_read_back_path);
	RUN_TEST(test_single_byte_erase_granularity);

	return UNITY_END();
}
