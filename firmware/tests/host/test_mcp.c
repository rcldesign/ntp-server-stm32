/*
 * STS1000 "Meridian" — core/mcp unit tests (framing, session, cfg, telemetry,
 * logs, diagnostics). The FW_* family has its own suite in test_mcp_dfu.c.
 *
 * The framing expectations are byte vectors produced independently of this
 * code — the CRC-32 values come from Python's zlib.crc32() over the same
 * header bytes, and the COBS encoding was hand-derived from the Cheshire &
 * Baker rules. A round-trip against mcp_wire_seal() would only prove the
 * encoder agrees with itself.
 *
 * The engine is driven exactly as the Zephyr glue will drive it: bytes in
 * through mcp_input(), time in through mcp_tick(), frames out through the tx
 * callback. Nothing reaches into the context to shortcut a state transition.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "cfg/cfg.h"
#include "logring/logring.h"
#include "mcp/mcp.h"
#include "util/bytes.h"
#include "util/cobs.h"
#include "util/crc.h"

#include "host_sha256.h"

/* ------------------------------------------------------------------------- */
/* Transmit capture                                                          */
/* ------------------------------------------------------------------------- */

#define TXQ 32U

typedef struct {
	uint8_t buf[MCP_MAX_FRAME];
	size_t  len;
} cap_t;

static cap_t    g_tx[TXQ];
static size_t   g_txn;
static bool     g_tx_block;
static uint32_t g_tx_calls;

static int tx_cb(void *user, const uint8_t *buf, size_t len)
{
	size_t dn = 0U;

	(void)user;
	g_tx_calls++;
	if (g_tx_block) {
		return -EAGAIN;
	}

	TEST_ASSERT_TRUE(len >= 2U);
	TEST_ASSERT_EQUAL_HEX8(0x00U, buf[len - 1U]); /* frame delimiter */
	TEST_ASSERT_TRUE(g_txn < TXQ);

	TEST_ASSERT_EQUAL_INT(0, cobs_decode(buf, len - 1U, g_tx[g_txn].buf,
					     sizeof(g_tx[g_txn].buf), &dn));
	g_tx[g_txn].len = dn;
	g_txn++;
	return 0;
}

static const cap_t *last_tx(void)
{
	TEST_ASSERT_TRUE(g_txn > 0U);
	return &g_tx[g_txn - 1U];
}

/* Field accessors over a captured frame, with the CRC re-checked each time. */
static void tx_check(const cap_t *f)
{
	TEST_ASSERT_TRUE(f->len >= (MCP_HDR_LEN + MCP_CRC_LEN));
	TEST_ASSERT_EQUAL_HEX8(MCP_VER, f->buf[0]);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)(f->len - MCP_HDR_LEN -
					    MCP_CRC_LEN),
				 bytes_get_le16(&f->buf[6]));
	TEST_ASSERT_EQUAL_HEX32(sts_crc32_ieee(f->buf, f->len - MCP_CRC_LEN),
				bytes_get_le32(&f->buf[f->len - MCP_CRC_LEN]));
}

static uint8_t tx_type(const cap_t *f) { tx_check(f); return f->buf[1]; }
static uint8_t tx_cmd(const cap_t *f) { tx_check(f); return f->buf[2]; }
static uint8_t tx_flags(const cap_t *f) { tx_check(f); return f->buf[3]; }
static uint16_t tx_seq(const cap_t *f) { tx_check(f); return bytes_get_le16(&f->buf[4]); }
static uint16_t tx_plen(const cap_t *f) { tx_check(f); return bytes_get_le16(&f->buf[6]); }
static const uint8_t *tx_pay(const cap_t *f) { tx_check(f); return &f->buf[MCP_HDR_LEN]; }
static uint8_t tx_status(const cap_t *f)
{
	TEST_ASSERT_TRUE(tx_plen(f) >= 1U);
	return tx_pay(f)[0];
}

/* ------------------------------------------------------------------------- */
/* Fakes: store, clock, status/diag sources, image port                      */
/* ------------------------------------------------------------------------- */

#define STORE_SLOTS 128U
#define STORE_REC   72U

static struct {
	struct {
		uint16_t id;
		uint8_t  buf[STORE_REC];
		uint8_t  len;
		bool     used;
	} e[STORE_SLOTS];
} g_store;

/* When >= 0, fs_save refuses this key ID (drives the MEDIUM-5 persist path). */
static int g_store_fail_save_id = -1;

static int fs_load(void *ctx, uint16_t id, void *buf, size_t cap)
{
	size_t i;

	(void)ctx;
	for (i = 0U; i < STORE_SLOTS; i++) {
		if (g_store.e[i].used && (g_store.e[i].id == id)) {
			if ((size_t)g_store.e[i].len > cap) {
				return -5;
			}
			memcpy(buf, g_store.e[i].buf, g_store.e[i].len);
			return (int)g_store.e[i].len;
		}
	}
	return -2;
}

static int fs_save(void *ctx, uint16_t id, const void *buf, size_t len)
{
	size_t i;
	size_t free_slot = STORE_SLOTS;

	(void)ctx;
	if ((g_store_fail_save_id >= 0) &&
	    ((uint16_t)g_store_fail_save_id == id)) {
		return -5;
	}
	if (len > STORE_REC) {
		return -28;
	}
	for (i = 0U; i < STORE_SLOTS; i++) {
		if (g_store.e[i].used && (g_store.e[i].id == id)) {
			break;
		}
		if (!g_store.e[i].used && (free_slot == STORE_SLOTS)) {
			free_slot = i;
		}
	}
	if (i == STORE_SLOTS) {
		i = free_slot;
		if (i == STORE_SLOTS) {
			return -28;
		}
		g_store.e[i].used = true;
		g_store.e[i].id = id;
	}
	memcpy(g_store.e[i].buf, buf, len);
	g_store.e[i].len = (uint8_t)len;
	return 0;
}

static int fs_erase(void *ctx, uint16_t id)
{
	size_t i;

	(void)ctx;
	for (i = 0U; i < STORE_SLOTS; i++) {
		if (g_store.e[i].used && (g_store.e[i].id == id)) {
			g_store.e[i].used = false;
			return 0;
		}
	}
	return -2;
}

static const port_store_t g_store_port = {
	.load = fs_load, .save = fs_save, .erase = fs_erase, .ctx = NULL,
};

static uint64_t g_now;

static uint64_t clk_mono(void *ctx)
{
	(void)ctx;
	return g_now;
}

static const port_clock_t g_clock = { .mono_ms = clk_mono, .tai_ns = NULL,
				      .ctx = NULL };

/* Status source: a versioned, group-tagged struct with a call counter so the
 * telemetry cadence is observable. */
#define STATUS_STRUCT_VER 2U

static uint32_t g_status_calls;
static int      g_status_fail;   /* group whose fetch fails, -1 = none */
static int      g_status_rc;     /* what that failure returns */

static int status_cb(void *user, uint8_t group, uint8_t *buf, size_t cap)
{
	(void)user;
	g_status_calls++;
	if ((g_status_fail >= 0) && ((uint8_t)g_status_fail == group)) {
		return g_status_rc;
	}
	TEST_ASSERT_TRUE(cap >= 8U);
	buf[0] = STATUS_STRUCT_VER;
	buf[1] = group;
	bytes_put_le32(&buf[2], g_status_calls);
	return 6;
}

static int diag_cb(void *user, uint8_t sub, uint8_t *buf, size_t cap)
{
	(void)user;
	TEST_ASSERT_TRUE(cap >= 4U);
	if (sub == 3U) {
		return -ENOTSUP;
	}
	buf[0] = 1U; /* struct version */
	buf[1] = sub;
	return 2;
}

/* Image port: only what HELLO and REBOOT need here. */
static int      g_reboot_mode;
static uint32_t g_reboot_calls;

static int img_info(void *ctx, uint8_t slot, port_image_info_t *out)
{
	(void)ctx;
	memset(out, 0, sizeof(*out));
	out->slot = slot;
	if (slot == 0U) {
		out->size = 0x1234U;
		out->version[0] = 1U;
		out->version[1] = 2U;
		out->version[2] = 3U;
		out->version[3] = 4U;
		out->valid = true;
		out->active = true;
		out->confirmed = true;
	} else {
		out->size = 0x40U;
		out->version[0] = 1U;
		out->version[1] = 3U;
		out->valid = true;
		out->pending = true;
	}
	return 0;
}

static void img_reboot(void *ctx, int mode)
{
	(void)ctx;
	g_reboot_mode = mode;
	g_reboot_calls++;
}

static const port_image_t g_img = {
	.staging_size = NULL, .staging_erase = NULL, .staging_write = NULL,
	.staging_read = NULL, .image_info = img_info, .mark_pending = NULL,
	.confirm_active = NULL, .request_revert = NULL, .reboot = img_reboot,
	.ctx = NULL,
};

/* ------------------------------------------------------------------------- */
/* Engine under test                                                         */
/* ------------------------------------------------------------------------- */

#define LOG_CAP 16U

static mcp_ctx_t  g_mcp;
static cfg_ctx_t  g_cfg;
static logr_t     g_log;
static logr_rec_t g_log_slots[LOG_CAP];
static uint16_t   g_seq;

static void wire_up(bool with_cfg, bool with_log, bool with_status)
{
	mcp_wiring_t w;

	memset(&w, 0, sizeof(w));
	memset(&g_store, 0, sizeof(g_store));
	g_store_fail_save_id = -1;
	memset(g_tx, 0, sizeof(g_tx));
	g_txn = 0U;
	g_tx_block = false;
	g_tx_calls = 0U;
	g_status_calls = 0U;
	g_status_fail = -1;
	g_status_rc = -ENOTSUP;
	g_reboot_calls = 0U;
	g_reboot_mode = -1;
	g_seq = 0U;
	g_now = 0U;

	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, &g_store_port));
	TEST_ASSERT_EQUAL_INT(0, logr_init(&g_log, g_log_slots, LOG_CAP));

	w.clock = &g_clock;
	w.crypto = host_crypto();
	w.img = &g_img;
	w.cfg = with_cfg ? &g_cfg : NULL;
	w.log = with_log ? &g_log : NULL;
	w.status_cb = with_status ? status_cb : NULL;
	w.diag_cb = with_status ? diag_cb : NULL;
	w.tx = tx_cb;
	memcpy(w.ident.model, "STS1000", 7);
	memcpy(w.ident.board_id, "\x01\x02\x03\x04\x05\x06\x07\x08", 8);

	TEST_ASSERT_EQUAL_INT(0, mcp_init(&g_mcp, &w));
}

/* Disable the auth policy so a test can exercise a mutating command directly. */
static void policy_no_auth(void)
{
	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_u64(&g_cfg, CFG_ID_SEC_AUTH_REQUIRED, 0U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));
}

/* Provision the admin credential straight into cfg, as the local UI would. */
static void provision_password(const char *pw)
{
	static const uint8_t salt[MCP_PW_SALT_LEN] = {
		0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
		0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
	};
	uint8_t blob[MCP_PW_BLOB_LEN];

	TEST_ASSERT_EQUAL_INT(0,
		mcp_auth_make_blob(host_crypto(), salt, (const uint8_t *)pw,
				   strlen(pw), blob, sizeof(blob)));
	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_bytes(&g_cfg, CFG_ID_SEC_ADMIN_PW, blob, sizeof(blob)));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));
}

/* ------------------------------------------------------------------------- */
/* Request injection                                                         */
/* ------------------------------------------------------------------------- */

/* Build a REQ frame (pre-COBS) exactly as mcp_wire.h documents it. */
static size_t build_raw(uint8_t ver, uint8_t type, uint8_t cmd, uint16_t seq,
			const uint8_t *pl, uint16_t n, uint8_t *out)
{
	out[0] = ver;
	out[1] = type;
	out[2] = cmd;
	out[3] = 0U;
	bytes_put_le16(&out[4], seq);
	bytes_put_le16(&out[6], n);
	if (n != 0U) {
		memcpy(&out[MCP_HDR_LEN], pl, n);
	}
	bytes_put_le32(&out[MCP_HDR_LEN + n],
		       sts_crc32_ieee(out, MCP_HDR_LEN + n));
	return MCP_HDR_LEN + (size_t)n + MCP_CRC_LEN;
}

/* COBS-wrap a raw frame and append the delimiter. */
static size_t wrap(const uint8_t *raw, size_t n, uint8_t *out, size_t cap)
{
	size_t el = 0U;

	TEST_ASSERT_EQUAL_INT(0, cobs_encode(raw, n, out, cap - 1U, &el));
	out[el] = 0x00U;
	return el + 1U;
}

static uint16_t feed_req_seq(uint8_t cmd, const uint8_t *pl, uint16_t n,
			     uint16_t seq)
{
	uint8_t raw[MCP_MAX_FRAME];
	uint8_t enc[COBS_ENCODE_MAX(MCP_MAX_FRAME) + 1U];
	size_t rn = build_raw(MCP_VER, (uint8_t)MCP_T_REQ, cmd, seq, pl, n, raw);
	size_t en = wrap(raw, rn, enc, sizeof(enc));

	(void)mcp_input(&g_mcp, enc, en, NULL);
	return seq;
}

static uint16_t feed_req(uint8_t cmd, const uint8_t *pl, uint16_t n)
{
	g_seq++;
	return feed_req_seq(cmd, pl, n, g_seq);
}

/* Feed the same request one byte at a time — the CDC-ACM worst case. */
static uint16_t feed_req_dribble(uint8_t cmd, const uint8_t *pl, uint16_t n)
{
	uint8_t raw[MCP_MAX_FRAME];
	uint8_t enc[COBS_ENCODE_MAX(MCP_MAX_FRAME) + 1U];
	size_t rn;
	size_t en;
	size_t i;

	g_seq++;
	rn = build_raw(MCP_VER, (uint8_t)MCP_T_REQ, cmd, g_seq, pl, n, raw);
	en = wrap(raw, rn, enc, sizeof(enc));

	for (i = 0U; i < en; i++) {
		size_t used = 0U;

		TEST_ASSERT_EQUAL_INT(0, mcp_input(&g_mcp, &enc[i], 1U, &used));
		TEST_ASSERT_EQUAL_size_t(1U, used);
		if (i + 1U < en) {
			TEST_ASSERT_EQUAL_size_t(0U, g_txn);
		}
	}
	return g_seq;
}

static void expect_status(uint8_t cmd, uint16_t seq, uint8_t status)
{
	const cap_t *f = last_tx();

	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_T_RSP, tx_type(f));
	TEST_ASSERT_EQUAL_HEX8(cmd, tx_cmd(f));
	TEST_ASSERT_EQUAL_UINT16(seq, tx_seq(f));
	TEST_ASSERT_EQUAL_HEX8(status, tx_status(f));
}

void setUp(void)
{
	wire_up(true, true, true);
}

/* ------------------------------------------------------------------------- */
/* Wire codec                                                                */
/* ------------------------------------------------------------------------- */

static void test_wire_known_vector(void)
{
	/*
	 * HELLO REQ, seq 1, empty payload. CRC-32/ISO-HDLC over the eight
	 * header bytes is 0xDA686B37 (Python zlib.crc32), stored
	 * little-endian; the COBS body is the hand-derived encoding of the
	 * resulting twelve bytes and the trailing 0x00 is the delimiter.
	 */
	static const uint8_t want_raw[12] = {
		0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
		0x37, 0x6B, 0x68, 0xDA,
	};
	static const uint8_t want_cobs[14] = {
		0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 0x01, 0x01,
		0x05, 0x37, 0x6B, 0x68, 0xDA, 0x00,
	};
	uint8_t raw[MCP_MAX_FRAME];
	uint8_t enc[COBS_ENCODE_MAX(MCP_MAX_FRAME) + 1U];
	mcp_frame_t f;
	size_t n = 0U;

	TEST_ASSERT_EQUAL_INT(0,
		mcp_wire_build((uint8_t)MCP_T_REQ, MCP_CMD_HELLO, 0U, 1U, NULL,
			       0U, raw, sizeof(raw), &n));
	TEST_ASSERT_EQUAL_size_t(sizeof(want_raw), n);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want_raw, raw, sizeof(want_raw));

	TEST_ASSERT_EQUAL_size_t(sizeof(want_cobs),
				 wrap(raw, n, enc, sizeof(enc)));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want_cobs, enc, sizeof(want_cobs));

	TEST_ASSERT_EQUAL_INT(0, mcp_wire_parse(want_raw, sizeof(want_raw), &f));
	TEST_ASSERT_EQUAL_HEX8(MCP_VER, f.ver);
	TEST_ASSERT_EQUAL_HEX8(MCP_T_REQ, f.type);
	TEST_ASSERT_EQUAL_HEX8(MCP_CMD_HELLO, f.cmd);
	TEST_ASSERT_EQUAL_UINT16(1U, f.seq);
	TEST_ASSERT_EQUAL_UINT16(0U, f.len);

	/* A second vector with a payload: STATUS_GET group 2, seq 7. */
	{
		static const uint8_t want[13] = {
			0x01, 0x00, 0x20, 0x00, 0x07, 0x00, 0x01, 0x00,
			0x02, 0x7A, 0xEE, 0x23, 0x63,
		};
		uint8_t grp = 2U;

		TEST_ASSERT_EQUAL_INT(0,
			mcp_wire_build((uint8_t)MCP_T_REQ, MCP_CMD_STATUS_GET,
				       0U, 7U, &grp, 1U, raw, sizeof(raw), &n));
		TEST_ASSERT_EQUAL_size_t(sizeof(want), n);
		TEST_ASSERT_EQUAL_HEX8_ARRAY(want, raw, sizeof(want));
	}
}

static void test_wire_codec_rejects(void)
{
	uint8_t raw[MCP_MAX_FRAME + 8U];
	mcp_frame_t f;
	size_t n = 0U;

	memset(raw, 0, sizeof(raw));

	TEST_ASSERT_EQUAL_INT(-EINVAL, mcp_wire_parse(NULL, 12U, &f));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mcp_wire_parse(raw, 12U, NULL));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, mcp_wire_parse(raw, 11U, &f));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
		mcp_wire_parse(raw, MCP_MAX_FRAME + 1U, &f));

	/* A length field that disagrees with the frame is caught before the
	 * CRC, because the CRC's own position depends on it. */
	(void)build_raw(MCP_VER, (uint8_t)MCP_T_REQ, MCP_CMD_HELLO, 1U, NULL,
			0U, raw);
	bytes_put_le16(&raw[6], 4U);
	TEST_ASSERT_EQUAL_INT(-EPROTO, mcp_wire_parse(raw, 12U, &f));

	n = build_raw(MCP_VER, (uint8_t)MCP_T_REQ, MCP_CMD_HELLO, 1U, NULL, 0U,
		      raw);
	raw[8] ^= 0x01U;
	TEST_ASSERT_EQUAL_INT(-EILSEQ, mcp_wire_parse(raw, n, &f));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
		mcp_wire_build(0U, 0U, 0U, 0U, NULL, 0U, NULL, 0U, &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		mcp_wire_build(0U, 0U, 0U, 0U, NULL, 0U, raw, sizeof(raw),
			       NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		mcp_wire_build(0U, 0U, 0U, 0U, NULL, 4U, raw, sizeof(raw), &n));
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE,
		mcp_wire_build(0U, 0U, 0U, 0U, raw, MCP_MAX_PAYLOAD + 1U, raw,
			       sizeof(raw), &n));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
		mcp_wire_build(0U, 0U, 0U, 0U, raw, 4U, raw, 8U, &n));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
		mcp_wire_seal(NULL, 0U, 0U, 0U, 0U, 0U, 0U, &n));
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE,
		mcp_wire_seal(raw, sizeof(raw), 0U, 0U, 0U, 0U,
			      MCP_MAX_PAYLOAD + 1U, &n));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
		mcp_wire_seal(raw, 8U, 0U, 0U, 0U, 0U, 4U, &n));
}

/* ------------------------------------------------------------------------- */
/* Framing on the wire                                                       */
/* ------------------------------------------------------------------------- */

static void test_frame_assembles_across_split_boundaries(void)
{
	uint16_t seq = feed_req_dribble(MCP_CMD_HELLO, NULL, 0U);

	TEST_ASSERT_EQUAL_size_t(1U, g_txn);
	expect_status(MCP_CMD_HELLO, seq, (uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT32(1U, mcp_stats(&g_mcp)->rx_frames);
}

static void test_two_frames_in_one_buffer(void)
{
	uint8_t raw[MCP_MAX_FRAME];
	uint8_t enc[2U * (COBS_ENCODE_MAX(MCP_MAX_FRAME) + 1U)];
	size_t rn;
	size_t total = 0U;
	size_t used = 0U;

	rn = build_raw(MCP_VER, (uint8_t)MCP_T_REQ, MCP_CMD_HELLO, 11U, NULL,
		       0U, raw);
	total += wrap(raw, rn, enc, sizeof(enc) / 2U);
	rn = build_raw(MCP_VER, (uint8_t)MCP_T_REQ, MCP_CMD_HELLO, 12U, NULL,
		       0U, raw);
	total += wrap(raw, rn, &enc[total], sizeof(enc) - total);

	TEST_ASSERT_EQUAL_INT(0, mcp_input(&g_mcp, enc, total, &used));
	TEST_ASSERT_EQUAL_size_t(total, used);
	TEST_ASSERT_EQUAL_size_t(2U, g_txn);
	TEST_ASSERT_EQUAL_UINT16(11U, tx_seq(&g_tx[0]));
	TEST_ASSERT_EQUAL_UINT16(12U, tx_seq(&g_tx[1]));
}

static void test_bad_crc_is_dropped_silently(void)
{
	uint8_t raw[MCP_MAX_FRAME];
	uint8_t enc[COBS_ENCODE_MAX(MCP_MAX_FRAME) + 1U];
	size_t rn = build_raw(MCP_VER, (uint8_t)MCP_T_REQ, MCP_CMD_HELLO, 5U,
			      NULL, 0U, raw);
	size_t en;

	raw[8] ^= 0x80U; /* corrupt the stored CRC */
	en = wrap(raw, rn, enc, sizeof(enc));

	TEST_ASSERT_EQUAL_INT(0, mcp_input(&g_mcp, enc, en, NULL));
	TEST_ASSERT_EQUAL_size_t(0U, g_txn);
	TEST_ASSERT_EQUAL_UINT32(1U, mcp_stats(&g_mcp)->rx_bad_crc);
	TEST_ASSERT_EQUAL_UINT32(0U, mcp_stats(&g_mcp)->rsp_tx);

	/* The stream resynchronises: the next good frame is answered. */
	(void)feed_req(MCP_CMD_HELLO, NULL, 0U);
	TEST_ASSERT_EQUAL_size_t(1U, g_txn);
}

static void test_malformed_frames_are_counted_not_answered(void)
{
	uint8_t raw[MCP_MAX_FRAME];
	uint8_t enc[COBS_ENCODE_MAX(MCP_MAX_FRAME) + 1U];
	size_t rn;
	size_t en;

	/* Too short to be a frame at all. */
	{
		static const uint8_t runt[4] = { 0x01, 0x00, 0x01, 0x00 };

		en = wrap(runt, sizeof(runt), enc, sizeof(enc));
		TEST_ASSERT_EQUAL_INT(0, mcp_input(&g_mcp, enc, en, NULL));
	}
	TEST_ASSERT_EQUAL_UINT32(1U, mcp_stats(&g_mcp)->rx_bad_hdr);

	/* A COBS body that cannot decode. */
	{
		static const uint8_t junk[3] = { 0x05, 0x01, 0x00 };

		TEST_ASSERT_EQUAL_INT(0,
			mcp_input(&g_mcp, junk, sizeof(junk), NULL));
	}
	TEST_ASSERT_EQUAL_UINT32(1U, mcp_stats(&g_mcp)->rx_bad_cobs);

	/* A well-formed RSP arriving at the server is not our business. */
	rn = build_raw(MCP_VER, (uint8_t)MCP_T_RSP, MCP_CMD_HELLO, 1U, NULL, 0U,
		       raw);
	en = wrap(raw, rn, enc, sizeof(enc));
	TEST_ASSERT_EQUAL_INT(0, mcp_input(&g_mcp, enc, en, NULL));
	TEST_ASSERT_EQUAL_UINT32(1U, mcp_stats(&g_mcp)->rx_ignored);

	/* A future protocol version gets a clear refusal, not silence: the CRC
	 * held, so the header is intelligible. */
	rn = build_raw(2U, (uint8_t)MCP_T_REQ, MCP_CMD_HELLO, 9U, NULL, 0U, raw);
	en = wrap(raw, rn, enc, sizeof(enc));
	TEST_ASSERT_EQUAL_INT(0, mcp_input(&g_mcp, enc, en, NULL));
	TEST_ASSERT_EQUAL_UINT32(1U, mcp_stats(&g_mcp)->rx_bad_ver);
	expect_status(MCP_CMD_HELLO, 9U, (uint8_t)MCP_ERR_NOTSUP);

	TEST_ASSERT_EQUAL_size_t(1U, g_txn);
}

static void test_unknown_command(void)
{
	uint16_t seq = feed_req(0x7FU, NULL, 0U);

	expect_status(0x7FU, seq, (uint8_t)MCP_ERR_NOTSUP);
}

static void test_input_argument_checks(void)
{
	size_t used = 99U;

	TEST_ASSERT_EQUAL_INT(-EINVAL, mcp_input(NULL, NULL, 0U, &used));
	TEST_ASSERT_EQUAL_size_t(0U, used);
	TEST_ASSERT_EQUAL_INT(-EINVAL, mcp_input(&g_mcp, NULL, 4U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mcp_poll_tx(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mcp_tick(NULL, 0U));
	TEST_ASSERT_NULL(mcp_stats(NULL));
	TEST_ASSERT_NULL(mcp_dfu_status(NULL));
	TEST_ASSERT_FALSE(mcp_authenticated(NULL));
	mcp_reset_session(NULL);
}

static void test_init_argument_checks(void)
{
	mcp_wiring_t w;
	mcp_ctx_t c;

	memset(&w, 0, sizeof(w));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mcp_init(NULL, &w));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mcp_init(&c, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mcp_init(&c, &w)); /* no tx callback */
}

/* ------------------------------------------------------------------------- */
/* Transmit back-pressure                                                    */
/* ------------------------------------------------------------------------- */

static void test_response_is_held_not_dropped(void)
{
	uint8_t raw[MCP_MAX_FRAME];
	uint8_t enc[2U * (COBS_ENCODE_MAX(MCP_MAX_FRAME) + 1U)];
	size_t rn;
	size_t total = 0U;
	size_t used = 0U;

	rn = build_raw(MCP_VER, (uint8_t)MCP_T_REQ, MCP_CMD_HELLO, 21U, NULL,
		       0U, raw);
	total += wrap(raw, rn, enc, sizeof(enc) / 2U);
	rn = build_raw(MCP_VER, (uint8_t)MCP_T_REQ, MCP_CMD_HELLO, 22U, NULL,
		       0U, raw);
	total += wrap(raw, rn, &enc[total], sizeof(enc) - total);

	g_tx_block = true;
	TEST_ASSERT_EQUAL_INT(-EAGAIN, mcp_input(&g_mcp, enc, total, &used));
	TEST_ASSERT_TRUE(used < total);
	TEST_ASSERT_EQUAL_size_t(0U, g_txn);
	TEST_ASSERT_EQUAL_INT(1, mcp_poll_tx(&g_mcp));
	TEST_ASSERT_TRUE(mcp_stats(&g_mcp)->tx_stalls > 0U);

	/* Drain: the first response comes out intact, then the rest of the
	 * buffer is accepted. */
	g_tx_block = false;
	TEST_ASSERT_EQUAL_INT(0, mcp_poll_tx(&g_mcp));
	TEST_ASSERT_EQUAL_size_t(1U, g_txn);
	TEST_ASSERT_EQUAL_UINT16(21U, tx_seq(&g_tx[0]));

	TEST_ASSERT_EQUAL_INT(0,
		mcp_input(&g_mcp, &enc[used], total - used, NULL));
	TEST_ASSERT_EQUAL_size_t(2U, g_txn);
	TEST_ASSERT_EQUAL_UINT16(22U, tx_seq(&g_tx[1]));
}

static void test_events_are_dropped_under_back_pressure(void)
{
	uint8_t sub[2] = { 0x01U, 1U }; /* summary group, 1 Hz */
	uint32_t before;

	policy_no_auth();
	(void)feed_req(MCP_CMD_TELEM_SUB, sub, sizeof(sub));
	expect_status(MCP_CMD_TELEM_SUB, g_seq, (uint8_t)MCP_OK);

	before = mcp_stats(&g_mcp)->evt_dropped;
	g_tx_block = true;
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 1000U));
	TEST_ASSERT_EQUAL_UINT32(before + 1U, mcp_stats(&g_mcp)->evt_dropped);
	TEST_ASSERT_EQUAL_UINT32(0U, mcp_stats(&g_mcp)->evt_tx);

	/* Nothing is queued behind it: the next tick simply re-samples. */
	g_tx_block = false;
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 2000U));
	TEST_ASSERT_EQUAL_UINT32(1U, mcp_stats(&g_mcp)->evt_tx);
}

/* ------------------------------------------------------------------------- */
/* HELLO                                                                     */
/* ------------------------------------------------------------------------- */

static void test_hello_reports_identity_and_capabilities(void)
{
	const cap_t *f;
	const uint8_t *p;
	uint16_t seq = feed_req(MCP_CMD_HELLO, NULL, 0U);

	f = last_tx();
	expect_status(MCP_CMD_HELLO, seq, (uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT16(74U, tx_plen(f));

	p = tx_pay(f);
	TEST_ASSERT_EQUAL_HEX8(MCP_VER, p[1]);
	TEST_ASSERT_EQUAL_UINT16(MCP_MAX_PAYLOAD, bytes_get_le16(&p[2]));
	TEST_ASSERT_EQUAL_HEX32(MCP_CAP_DFU | MCP_CAP_TELEM | MCP_CAP_LOGS |
					MCP_CAP_CFG | MCP_CAP_AUTH |
					MCP_CAP_DIAG,
				bytes_get_le32(&p[4]));
	TEST_ASSERT_EQUAL_HEX8_ARRAY("STS1000", &p[8], 7);
	TEST_ASSERT_EQUAL_HEX8(0U, p[15]); /* NUL-padded */
	TEST_ASSERT_EQUAL_HEX8_ARRAY("\x01\x02\x03\x04\x05\x06\x07\x08", &p[24],
				     8);

	/* Running slot: version 1.2.3.4, size 0x1234, valid+active+confirmed. */
	TEST_ASSERT_EQUAL_UINT32(1U, bytes_get_le32(&p[32]));
	TEST_ASSERT_EQUAL_UINT32(4U, bytes_get_le32(&p[44]));
	TEST_ASSERT_EQUAL_UINT32(0x1234U, bytes_get_le32(&p[48]));
	TEST_ASSERT_EQUAL_HEX8(MCP_SLOT_VALID | MCP_SLOT_ACTIVE |
				       MCP_SLOT_CONFIRMED,
			       p[52]);
	/* Staged slot: pending. */
	TEST_ASSERT_EQUAL_HEX8(MCP_SLOT_VALID | MCP_SLOT_PENDING, p[73]);
}

static void test_hello_capabilities_track_the_wiring(void)
{
	const uint8_t *p;

	wire_up(false, false, false);
	(void)feed_req(MCP_CMD_HELLO, NULL, 0U);

	p = tx_pay(last_tx());
	TEST_ASSERT_EQUAL_HEX32(MCP_CAP_DFU, bytes_get_le32(&p[4]));

	/* And the commands those capabilities gate are refused, not crashed. */
	(void)feed_req(MCP_CMD_CFG_COMMIT, NULL, 0U);
	expect_status(MCP_CMD_CFG_COMMIT, g_seq, (uint8_t)MCP_ERR_NOTSUP);
	(void)feed_req(MCP_CMD_LOG_LEVEL, (const uint8_t *)"\x00\x06", 2U);
	expect_status(MCP_CMD_LOG_LEVEL, g_seq, (uint8_t)MCP_ERR_NOTSUP);
	(void)feed_req(MCP_CMD_STATUS_GET, (const uint8_t *)"\x00", 1U);
	expect_status(MCP_CMD_STATUS_GET, g_seq, (uint8_t)MCP_ERR_NOTSUP);
	(void)feed_req(MCP_CMD_TELEM_SUB, (const uint8_t *)"\x01\x01", 2U);
	expect_status(MCP_CMD_TELEM_SUB, g_seq, (uint8_t)MCP_ERR_NOTSUP);
	(void)feed_req(MCP_CMD_DIAG, (const uint8_t *)"\x00", 1U);
	expect_status(MCP_CMD_DIAG, g_seq, (uint8_t)MCP_ERR_NOTSUP);
	(void)feed_req(MCP_CMD_DIAG, (const uint8_t *)"\x02", 1U);
	expect_status(MCP_CMD_DIAG, g_seq, (uint8_t)MCP_ERR_NOTSUP);
}

/* ------------------------------------------------------------------------- */
/* Authentication and the gating matrix                                      */
/* ------------------------------------------------------------------------- */

static void test_auth_success_and_failure(void)
{
	provision_password("correct horse");

	(void)feed_req(MCP_CMD_AUTH, (const uint8_t *)"wrong", 5U);
	expect_status(MCP_CMD_AUTH, g_seq, (uint8_t)MCP_ERR_AUTH);
	TEST_ASSERT_FALSE(mcp_authenticated(&g_mcp));
	TEST_ASSERT_EQUAL_UINT32(1U, mcp_stats(&g_mcp)->auth_fail);

	(void)feed_req(MCP_CMD_AUTH, (const uint8_t *)"correct horse", 13U);
	expect_status(MCP_CMD_AUTH, g_seq, (uint8_t)MCP_OK);
	TEST_ASSERT_TRUE(mcp_authenticated(&g_mcp));
	TEST_ASSERT_EQUAL_UINT8(1U, tx_pay(last_tx())[1]);
	TEST_ASSERT_EQUAL_UINT16(600U, bytes_get_le16(&tx_pay(last_tx())[2]));
	TEST_ASSERT_EQUAL_UINT32(1U, mcp_stats(&g_mcp)->auth_ok);

	/* Empty and over-long passwords are argument errors, not attempts. */
	(void)feed_req(MCP_CMD_AUTH, NULL, 0U);
	expect_status(MCP_CMD_AUTH, g_seq, (uint8_t)MCP_ERR_ARG);
	{
		uint8_t big[MCP_PW_MAX + 1U];

		memset(big, 'p', sizeof(big));
		(void)feed_req(MCP_CMD_AUTH, big, sizeof(big));
		expect_status(MCP_CMD_AUTH, g_seq, (uint8_t)MCP_ERR_ARG);
	}
	TEST_ASSERT_EQUAL_UINT32(1U, mcp_stats(&g_mcp)->auth_fail);
}

static void test_auth_without_a_provisioned_credential_locks_the_box(void)
{
	/* Auth required by default, no password stored: the box is locked, not
	 * wide open. */
	(void)feed_req(MCP_CMD_AUTH, (const uint8_t *)"anything", 8U);
	expect_status(MCP_CMD_AUTH, g_seq, (uint8_t)MCP_ERR_STATE);
	TEST_ASSERT_FALSE(mcp_authenticated(&g_mcp));

	(void)feed_req(MCP_CMD_CFG_COMMIT, NULL, 0U);
	expect_status(MCP_CMD_CFG_COMMIT, g_seq, (uint8_t)MCP_ERR_AUTH);
	TEST_ASSERT_EQUAL_UINT32(1U, mcp_stats(&g_mcp)->auth_denied);
}

static void test_auth_needs_a_credential_store_and_crypto(void)
{
	mcp_wiring_t w;

	/* No cfg: no policy and no credential, so AUTH is unimplemented and
	 * mutating commands are not gated (see mcp.h). */
	wire_up(false, false, false);
	(void)feed_req(MCP_CMD_AUTH, (const uint8_t *)"x", 1U);
	expect_status(MCP_CMD_AUTH, g_seq, (uint8_t)MCP_ERR_NOTSUP);
	(void)feed_req(MCP_CMD_REBOOT, (const uint8_t *)"\x00", 1U);
	expect_status(MCP_CMD_REBOOT, g_seq, (uint8_t)MCP_OK);

	/* cfg but no crypto: AUTH cannot be evaluated, and because the policy
	 * still demands it, mutating commands stay refused. */
	memset(&w, 0, sizeof(w));
	w.clock = &g_clock;
	w.cfg = &g_cfg;
	w.tx = tx_cb;
	TEST_ASSERT_EQUAL_INT(0, mcp_init(&g_mcp, &w));
	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, &g_store_port));

	(void)feed_req(MCP_CMD_AUTH, (const uint8_t *)"x", 1U);
	expect_status(MCP_CMD_AUTH, g_seq, (uint8_t)MCP_ERR_NOTSUP);
	(void)feed_req(MCP_CMD_CFG_COMMIT, NULL, 0U);
	expect_status(MCP_CMD_CFG_COMMIT, g_seq, (uint8_t)MCP_ERR_AUTH);
}

static void test_auth_gating_matrix(void)
{
	static const uint8_t mutating[] = {
		MCP_CMD_REBOOT,     MCP_CMD_CFG_SET,    MCP_CMD_CFG_COMMIT,
		MCP_CMD_CFG_REVERT, MCP_CMD_CFG_IMPORT, MCP_CMD_FACTORY_RESET,
		MCP_CMD_LOG_LEVEL,  MCP_CMD_FW_BEGIN,   MCP_CMD_FW_DATA,
		MCP_CMD_FW_END,     MCP_CMD_FW_CONFIRM, MCP_CMD_FW_REVERT,
	};
	static const uint8_t readonly[] = {
		MCP_CMD_HELLO,    MCP_CMD_CFG_LIST,    MCP_CMD_CFG_GET,
		MCP_CMD_CFG_EXPORT, MCP_CMD_STATUS_GET, MCP_CMD_TELEM_UNSUB,
		MCP_CMD_LOG_TAIL, MCP_CMD_FW_INFO,     MCP_CMD_DIAG,
	};
	size_t i;

	provision_password("pw");

	/* Unauthenticated: every mutating command is refused up front, before
	 * its payload is even parsed. */
	for (i = 0U; i < sizeof(mutating); i++) {
		(void)feed_req(mutating[i], NULL, 0U);
		expect_status(mutating[i], g_seq, (uint8_t)MCP_ERR_AUTH);
	}
	TEST_ASSERT_EQUAL_UINT32((uint32_t)sizeof(mutating),
				 mcp_stats(&g_mcp)->auth_denied);

	/* Read-only commands are never refused for lack of a session — they
	 * may fail on their own arguments, but not with ERR_AUTH. */
	for (i = 0U; i < sizeof(readonly); i++) {
		(void)feed_req(readonly[i], NULL, 0U);
		TEST_ASSERT_NOT_EQUAL_UINT8((uint8_t)MCP_ERR_AUTH,
					    tx_status(last_tx()));
	}

	/* With a session, the mutating commands get past the gate. */
	(void)feed_req(MCP_CMD_AUTH, (const uint8_t *)"pw", 2U);
	expect_status(MCP_CMD_AUTH, g_seq, (uint8_t)MCP_OK);
	(void)feed_req(MCP_CMD_CFG_COMMIT, NULL, 0U);
	expect_status(MCP_CMD_CFG_COMMIT, g_seq, (uint8_t)MCP_OK);
}

static void test_session_expires_when_idle(void)
{
	provision_password("pw");
	(void)feed_req(MCP_CMD_AUTH, (const uint8_t *)"pw", 2U);
	TEST_ASSERT_TRUE(mcp_authenticated(&g_mcp));

	/* Default sec.session.s is 600. */
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 599999U));
	TEST_ASSERT_TRUE(mcp_authenticated(&g_mcp));
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 600000U));
	TEST_ASSERT_FALSE(mcp_authenticated(&g_mcp));
	TEST_ASSERT_EQUAL_UINT32(1U, mcp_stats(&g_mcp)->session_expired);

	(void)feed_req(MCP_CMD_CFG_COMMIT, NULL, 0U);
	expect_status(MCP_CMD_CFG_COMMIT, g_seq, (uint8_t)MCP_ERR_AUTH);

	/* Activity holds a session open. */
	(void)feed_req(MCP_CMD_AUTH, (const uint8_t *)"pw", 2U);
	TEST_ASSERT_TRUE(mcp_authenticated(&g_mcp));
	g_now = 500000U;
	(void)feed_req(MCP_CMD_HELLO, NULL, 0U);
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 1000000U));
	TEST_ASSERT_TRUE(mcp_authenticated(&g_mcp));
}

static void test_reset_session_drops_everything_but_dfu(void)
{
	provision_password("pw");
	(void)feed_req(MCP_CMD_AUTH, (const uint8_t *)"pw", 2U);
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 500U));
	TEST_ASSERT_EQUAL_UINT16(1U, cfg_staged_count(&g_cfg));

	mcp_reset_session(&g_mcp);

	TEST_ASSERT_FALSE(mcp_authenticated(&g_mcp));
	TEST_ASSERT_EQUAL_UINT16(0U, cfg_staged_count(&g_cfg));
	TEST_ASSERT_EQUAL_INT(0, mcp_poll_tx(&g_mcp));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_DFU_IDLE,
				mcp_dfu_status(&g_mcp)->state);
}

static void test_auth_blob_helper(void)
{
	uint8_t salt[MCP_PW_SALT_LEN];
	uint8_t a[MCP_PW_BLOB_LEN];
	uint8_t b[MCP_PW_BLOB_LEN];
	port_crypto_t no_hmac = *host_crypto();
	port_crypto_t no_rand = *host_crypto();

	memset(salt, 0xA5, sizeof(salt));

	TEST_ASSERT_EQUAL_INT(0,
		mcp_auth_make_blob(host_crypto(), salt,
				   (const uint8_t *)"pw", 2U, a, sizeof(a)));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(salt, a, sizeof(salt));

	/* Same salt and password, same blob; a random salt gives a different
	 * one even for the same password. */
	TEST_ASSERT_EQUAL_INT(0,
		mcp_auth_make_blob(host_crypto(), salt,
				   (const uint8_t *)"pw", 2U, b, sizeof(b)));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(a, b, sizeof(a));

	TEST_ASSERT_EQUAL_INT(0,
		mcp_auth_make_blob(host_crypto(), NULL, (const uint8_t *)"pw",
				   2U, b, sizeof(b)));
	TEST_ASSERT_TRUE(memcmp(a, b, sizeof(a)) != 0);

	TEST_ASSERT_EQUAL_INT(-EINVAL,
		mcp_auth_make_blob(NULL, salt, (const uint8_t *)"pw", 2U, a,
				   sizeof(a)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		mcp_auth_make_blob(host_crypto(), salt, NULL, 2U, a,
				   sizeof(a)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		mcp_auth_make_blob(host_crypto(), salt, (const uint8_t *)"pw",
				   2U, NULL, sizeof(a)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		mcp_auth_make_blob(host_crypto(), salt, (const uint8_t *)"pw",
				   0U, a, sizeof(a)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		mcp_auth_make_blob(host_crypto(), salt, (const uint8_t *)"pw",
				   MCP_PW_MAX + 1U, a, sizeof(a)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		mcp_auth_make_blob(host_crypto(), salt, (const uint8_t *)"pw",
				   2U, a, 4U));

	no_hmac.hmac_sha256 = NULL;
	TEST_ASSERT_EQUAL_INT(-ENOTSUP,
		mcp_auth_make_blob(&no_hmac, salt, (const uint8_t *)"pw", 2U, a,
				   sizeof(a)));
	no_rand.rand = NULL;
	TEST_ASSERT_EQUAL_INT(-ENOTSUP,
		mcp_auth_make_blob(&no_rand, NULL, (const uint8_t *)"pw", 2U, a,
				   sizeof(a)));
}

/* ------------------------------------------------------------------------- */
/* REBOOT                                                                    */
/* ------------------------------------------------------------------------- */

static void test_reboot_is_deferred_until_the_answer_is_out(void)
{
	uint8_t mode = MCP_REBOOT_RECOVERY;

	policy_no_auth();

	(void)feed_req(MCP_CMD_REBOOT, &mode, 1U);
	expect_status(MCP_CMD_REBOOT, g_seq, (uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT32(0U, g_reboot_calls);

	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, MCP_REBOOT_DELAY_MS - 1U));
	TEST_ASSERT_EQUAL_UINT32(0U, g_reboot_calls);
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, MCP_REBOOT_DELAY_MS));
	TEST_ASSERT_EQUAL_UINT32(1U, g_reboot_calls);
	TEST_ASSERT_EQUAL_INT((int)MCP_REBOOT_RECOVERY, g_reboot_mode);

	/* It fires exactly once. */
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 10000U));
	TEST_ASSERT_EQUAL_UINT32(1U, g_reboot_calls);
}

static void test_reboot_validates_its_mode(void)
{
	uint8_t bad = 3U;

	policy_no_auth();
	(void)feed_req(MCP_CMD_REBOOT, &bad, 1U);
	expect_status(MCP_CMD_REBOOT, g_seq, (uint8_t)MCP_ERR_ARG);
	(void)feed_req(MCP_CMD_REBOOT, NULL, 0U);
	expect_status(MCP_CMD_REBOOT, g_seq, (uint8_t)MCP_ERR_ARG);
	TEST_ASSERT_EQUAL_UINT32(0U, g_reboot_calls);
}

/* ------------------------------------------------------------------------- */
/* Config over the wire                                                      */
/* ------------------------------------------------------------------------- */

static void test_cfg_list_pages_in_id_order(void)
{
	uint8_t req[3];
	const uint8_t *p;
	uint16_t next;
	size_t o;
	uint8_t n;
	uint16_t prev = 0U;
	unsigned int pages = 0U;
	uint16_t total = 0U;

	bytes_put_le16(req, 0U);
	req[2] = 5U;

	do {
		(void)feed_req(MCP_CMD_CFG_LIST, req, sizeof(req));
		expect_status(MCP_CMD_CFG_LIST, g_seq, (uint8_t)MCP_OK);
		p = tx_pay(last_tx());
		next = bytes_get_le16(&p[1]);
		n = p[3];
		TEST_ASSERT_TRUE(n <= 5U);

		o = 4U;
		while (n-- > 0U) {
			uint16_t id = bytes_get_le16(&p[o]);
			uint8_t nl = p[o + 4U];
			const cfg_key_t *k = cfg_key_find(id);

			TEST_ASSERT_NOT_NULL(k);
			TEST_ASSERT_TRUE(id > prev);
			prev = id;
			TEST_ASSERT_EQUAL_UINT8(k->type, p[o + 2U]);
			TEST_ASSERT_EQUAL_UINT8(k->flags, p[o + 3U]);
			TEST_ASSERT_EQUAL_size_t(strlen(k->name), nl);
			TEST_ASSERT_EQUAL_HEX8_ARRAY(k->name, &p[o + 5U], nl);
			o += 5U + nl;
			total++;
		}
		TEST_ASSERT_EQUAL_UINT16((uint16_t)o, tx_plen(last_tx()));

		bytes_put_le16(req, next);
		pages++;
		TEST_ASSERT_TRUE(pages < 100U);
	} while (next != 0U);

	TEST_ASSERT_EQUAL_UINT16((uint16_t)cfg_key_count(), total);

	/* Argument checks. */
	req[2] = 0U;
	bytes_put_le16(req, 0U);
	(void)feed_req(MCP_CMD_CFG_LIST, req, sizeof(req));
	expect_status(MCP_CMD_CFG_LIST, g_seq, (uint8_t)MCP_ERR_ARG);
	(void)feed_req(MCP_CMD_CFG_LIST, req, 2U);
	expect_status(MCP_CMD_CFG_LIST, g_seq, (uint8_t)MCP_ERR_ARG);

	/* A start ID past the end lists nothing and terminates. */
	bytes_put_le16(req, 0xFFFEU);
	req[2] = 5U;
	(void)feed_req(MCP_CMD_CFG_LIST, req, sizeof(req));
	p = tx_pay(last_tx());
	TEST_ASSERT_EQUAL_UINT8(0U, p[3]);
	TEST_ASSERT_EQUAL_UINT16(0U, bytes_get_le16(&p[1]));
}

static void test_cfg_get_set_commit_round_trip(void)
{
	uint8_t req[16];
	const uint8_t *p;
	uint64_t live = 0U;

	memset(req, 0, sizeof(req));

	policy_no_auth();

	/* GET the live default. */
	bytes_put_le16(req, CFG_ID_TIM_TAU_S);
	(void)feed_req(MCP_CMD_CFG_GET, req, 2U);
	expect_status(MCP_CMD_CFG_GET, g_seq, (uint8_t)MCP_OK);
	p = tx_pay(last_tx());
	TEST_ASSERT_EQUAL_UINT16(CFG_ID_TIM_TAU_S, bytes_get_le16(&p[1]));
	TEST_ASSERT_EQUAL_UINT8(CFG_T_U16, p[3]);
	TEST_ASSERT_EQUAL_UINT8(CFG_F_RUNTIME_APPLY, p[4]);
	TEST_ASSERT_EQUAL_UINT16(2U, bytes_get_le16(&p[5]));
	TEST_ASSERT_EQUAL_UINT16(200U, bytes_get_le16(&p[7]));

	/* SET stages it. */
	bytes_put_le16(req, CFG_ID_TIM_TAU_S);
	req[2] = CFG_T_U16;
	bytes_put_le16(&req[3], 2U);
	bytes_put_le16(&req[5], 750U);
	(void)feed_req(MCP_CMD_CFG_SET, req, 7U);
	expect_status(MCP_CMD_CFG_SET, g_seq, (uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT16(CFG_ID_TIM_TAU_S,
				 bytes_get_le16(&tx_pay(last_tx())[1]));

	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &live));
	TEST_ASSERT_EQUAL_UINT64(200U, live);

	/* GET with the staged flag sees the pending value. */
	bytes_put_le16(req, CFG_ID_TIM_TAU_S);
	req[2] = 0x01U;
	(void)feed_req(MCP_CMD_CFG_GET, req, 3U);
	TEST_ASSERT_EQUAL_UINT16(750U, bytes_get_le16(&tx_pay(last_tx())[7]));

	/* COMMIT applies it. */
	(void)feed_req(MCP_CMD_CFG_COMMIT, NULL, 0U);
	expect_status(MCP_CMD_CFG_COMMIT, g_seq, (uint8_t)MCP_OK);
	p = tx_pay(last_tx());
	TEST_ASSERT_EQUAL_UINT16(1U, bytes_get_le16(&p[1])); /* applied */
	TEST_ASSERT_EQUAL_UINT16(0U, bytes_get_le16(&p[3])); /* reboot keys */
	TEST_ASSERT_EQUAL_HEX32(0U, bytes_get_le32(&p[5]));

	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &live));
	TEST_ASSERT_EQUAL_UINT64(750U, live);

	/* A reboot-scoped key reports its group. */
	bytes_put_le16(req, CFG_ID_NET_DHCP);
	req[2] = CFG_T_BOOL;
	bytes_put_le16(&req[3], 1U);
	req[5] = 0U;
	(void)feed_req(MCP_CMD_CFG_SET, req, 6U);
	(void)feed_req(MCP_CMD_CFG_COMMIT, NULL, 0U);
	p = tx_pay(last_tx());
	TEST_ASSERT_EQUAL_UINT16(1U, bytes_get_le16(&p[3]));
	TEST_ASSERT_EQUAL_HEX32(CFG_GROUP_BIT(CFG_G_NET), bytes_get_le32(&p[5]));
}

static void test_cfg_revert_drops_the_staged_set(void)
{
	uint8_t req[8];

	policy_no_auth();

	bytes_put_le16(req, CFG_ID_UI_BRIGHTNESS);
	req[2] = CFG_T_U8;
	bytes_put_le16(&req[3], 1U);
	req[5] = 33U;
	(void)feed_req(MCP_CMD_CFG_SET, req, 6U);
	TEST_ASSERT_EQUAL_UINT16(1U, cfg_staged_count(&g_cfg));

	(void)feed_req(MCP_CMD_CFG_REVERT, NULL, 0U);
	expect_status(MCP_CMD_CFG_REVERT, g_seq, (uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT16(1U, bytes_get_le16(&tx_pay(last_tx())[1]));
	TEST_ASSERT_EQUAL_UINT16(0U, cfg_staged_count(&g_cfg));

	(void)feed_req(MCP_CMD_CFG_REVERT, (const uint8_t *)"\x00", 1U);
	expect_status(MCP_CMD_CFG_REVERT, g_seq, (uint8_t)MCP_ERR_ARG);
}

static void test_cfg_argument_errors(void)
{
	uint8_t req[16];

	memset(req, 0, sizeof(req));
	policy_no_auth();

	(void)feed_req(MCP_CMD_CFG_GET, req, 1U);
	expect_status(MCP_CMD_CFG_GET, g_seq, (uint8_t)MCP_ERR_ARG);

	bytes_put_le16(req, 0xFFFFU);
	(void)feed_req(MCP_CMD_CFG_GET, req, 2U);
	expect_status(MCP_CMD_CFG_GET, g_seq, (uint8_t)MCP_ERR_ARG);

	(void)feed_req(MCP_CMD_CFG_SET, req, 4U);
	expect_status(MCP_CMD_CFG_SET, g_seq, (uint8_t)MCP_ERR_ARG);

	/* Declared value length disagreeing with the frame. */
	bytes_put_le16(req, CFG_ID_TIM_TAU_S);
	req[2] = CFG_T_U16;
	bytes_put_le16(&req[3], 4U);
	(void)feed_req(MCP_CMD_CFG_SET, req, 7U);
	expect_status(MCP_CMD_CFG_SET, g_seq, (uint8_t)MCP_ERR_ARG);

	/* Out of range for the schema. */
	bytes_put_le16(&req[3], 2U);
	bytes_put_le16(&req[5], 5U); /* tau minimum is 10 */
	(void)feed_req(MCP_CMD_CFG_SET, req, 7U);
	expect_status(MCP_CMD_CFG_SET, g_seq, (uint8_t)MCP_ERR_ARG);

	/* Unknown key. */
	bytes_put_le16(req, 0xFFFFU);
	(void)feed_req(MCP_CMD_CFG_SET, req, 7U);
	expect_status(MCP_CMD_CFG_SET, g_seq, (uint8_t)MCP_ERR_ARG);

	(void)feed_req(MCP_CMD_CFG_COMMIT, (const uint8_t *)"\x00", 1U);
	expect_status(MCP_CMD_CFG_COMMIT, g_seq, (uint8_t)MCP_ERR_ARG);
}

static void test_cfg_secret_needs_a_session(void)
{
	uint8_t req[8];

	provision_password("pw");

	bytes_put_le16(req, CFG_ID_SNMP_COMMUNITY);
	(void)feed_req(MCP_CMD_CFG_GET, req, 2U);
	expect_status(MCP_CMD_CFG_GET, g_seq, (uint8_t)MCP_ERR_AUTH);

	(void)feed_req(MCP_CMD_AUTH, (const uint8_t *)"pw", 2U);
	(void)feed_req(MCP_CMD_CFG_GET, req, 2U);
	expect_status(MCP_CMD_CFG_GET, g_seq, (uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_HEX8_ARRAY("public", &tx_pay(last_tx())[7], 6);
}

static void test_cfg_export_streams_and_reimports(void)
{
	static uint8_t blob[8192];
	uint8_t req[8];
	size_t total = 0U;
	unsigned int rounds = 0U;
	uint8_t more;
	cfg_commit_res_t res;

	policy_no_auth();
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 815U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));

	do {
		const uint8_t *p;
		uint16_t n;

		bytes_put_le32(req, (uint32_t)total);
		req[4] = 0U;
		(void)feed_req(MCP_CMD_CFG_EXPORT, req, 5U);
		expect_status(MCP_CMD_CFG_EXPORT, g_seq, (uint8_t)MCP_OK);

		p = tx_pay(last_tx());
		TEST_ASSERT_EQUAL_UINT32((uint32_t)total, bytes_get_le32(&p[1]));
		more = p[5];
		n = (uint16_t)(tx_plen(last_tx()) - 6U);
		TEST_ASSERT_TRUE((total + n) <= sizeof(blob));
		memcpy(&blob[total], &p[6], n);
		total += n;
		rounds++;
		TEST_ASSERT_TRUE(rounds < 50U);
	} while (more != 0U);

	TEST_ASSERT_EQUAL_HEX8_ARRAY("MCF1", blob, 4);

	/* Resuming an export at the wrong offset is refused, not guessed. */
	bytes_put_le32(req, 7U);
	req[4] = 0U;
	(void)feed_req(MCP_CMD_CFG_EXPORT, req, 5U);
	expect_status(MCP_CMD_CFG_EXPORT, g_seq, (uint8_t)MCP_ERR_OFFSET);
	(void)feed_req(MCP_CMD_CFG_EXPORT, req, 4U);
	expect_status(MCP_CMD_CFG_EXPORT, g_seq, (uint8_t)MCP_ERR_ARG);

	/* The captured stream really is a valid config image. */
	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, &g_store_port));
	TEST_ASSERT_EQUAL_INT(0,
		cfg_import_all(&g_cfg, blob, total, true, &res));
	{
		uint64_t u = 0U;

		TEST_ASSERT_EQUAL_INT(0,
			cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
		TEST_ASSERT_EQUAL_UINT64(815U, u);
	}
}

static void test_cfg_export_secrets_need_a_session(void)
{
	uint8_t req[8];

	provision_password("pw");

	bytes_put_le32(req, 0U);
	req[4] = 0x01U; /* include secrets */
	(void)feed_req(MCP_CMD_CFG_EXPORT, req, 5U);
	expect_status(MCP_CMD_CFG_EXPORT, g_seq, (uint8_t)MCP_ERR_AUTH);

	(void)feed_req(MCP_CMD_AUTH, (const uint8_t *)"pw", 2U);
	(void)feed_req(MCP_CMD_CFG_EXPORT, req, 5U);
	expect_status(MCP_CMD_CFG_EXPORT, g_seq, (uint8_t)MCP_OK);
}

static void test_cfg_import_streams_in_chunks(void)
{
	static uint8_t blob[8192];
	static uint8_t req[MCP_MAX_PAYLOAD];
	size_t total = 0U;
	size_t off = 0U;
	uint64_t u = 0U;

	policy_no_auth();
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 640U));
	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_u64(&g_cfg, CFG_ID_NTS_KE_PORT, 4470U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));
	TEST_ASSERT_EQUAL_INT(0,
		cfg_export_all(&g_cfg, blob, sizeof(blob), false, &total));

	/* Wipe the live tree, then push the blob back over the wire. */
	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, &g_store_port));
	policy_no_auth();

	while (off < total) {
		size_t n = total - off;
		bool last;

		if (n > 128U) {
			n = 128U;
		}
		last = ((off + n) == total);

		bytes_put_le32(req, (uint32_t)off);
		req[4] = last ? 0x02U : 0x00U;
		memcpy(&req[5], &blob[off], n);
		(void)feed_req(MCP_CMD_CFG_IMPORT, req, (uint16_t)(5U + n));
		expect_status(MCP_CMD_CFG_IMPORT, g_seq, (uint8_t)MCP_OK);

		off += n;
		TEST_ASSERT_EQUAL_UINT32((uint32_t)off,
					 bytes_get_le32(&tx_pay(last_tx())[1]));
		TEST_ASSERT_EQUAL_UINT8(last ? 1U : 0U, tx_pay(last_tx())[5]);
	}

	TEST_ASSERT_EQUAL_UINT16(2U, bytes_get_le16(&tx_pay(last_tx())[6]));
	TEST_ASSERT_EQUAL_HEX32(CFG_GROUP_BIT(CFG_G_NTS),
				bytes_get_le32(&tx_pay(last_tx())[8]));
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(640U, u);
}

static void test_cfg_import_rejects_bad_streams(void)
{
	static uint8_t blob[8192];
	static uint8_t req[MCP_MAX_PAYLOAD];
	size_t total = 0U;

	memset(req, 0, sizeof(req));
	policy_no_auth();
	TEST_ASSERT_EQUAL_INT(0,
		cfg_export_all(&g_cfg, blob, sizeof(blob), false, &total));
	TEST_ASSERT_TRUE(total < (sizeof(req) - 5U));

	/* Short payload. */
	(void)feed_req(MCP_CMD_CFG_IMPORT, req, 4U);
	expect_status(MCP_CMD_CFG_IMPORT, g_seq, (uint8_t)MCP_ERR_ARG);

	/* Resuming at an offset the engine never reached. */
	bytes_put_le32(req, 99U);
	req[4] = 0U;
	(void)feed_req(MCP_CMD_CFG_IMPORT, req, 5U);
	expect_status(MCP_CMD_CFG_IMPORT, g_seq, (uint8_t)MCP_ERR_OFFSET);

	/* A corrupt trailer is reported as a CRC failure, distinctly from the
	 * malformed-record errors — the records themselves all decode. */
	blob[total - 1U] ^= 0xFFU;
	bytes_put_le32(req, 0U);
	req[4] = 0x03U; /* strict + final */
	memcpy(&req[5], blob, total);
	(void)feed_req(MCP_CMD_CFG_IMPORT, req, (uint16_t)(5U + total));
	expect_status(MCP_CMD_CFG_IMPORT, g_seq, (uint8_t)MCP_ERR_CRC);
	blob[total - 1U] ^= 0xFFU;

	/* Declaring the final chunk before the stream is complete. */
	bytes_put_le32(req, 0U);
	req[4] = 0x02U;
	memcpy(&req[5], blob, total - 4U);
	(void)feed_req(MCP_CMD_CFG_IMPORT, req, (uint16_t)(5U + total - 4U));
	expect_status(MCP_CMD_CFG_IMPORT, g_seq, (uint8_t)MCP_ERR_STATE);
	TEST_ASSERT_EQUAL_UINT16(0U, cfg_staged_count(&g_cfg));
}

static void test_factory_reset_needs_the_magic(void)
{
	uint8_t req[4];
	uint64_t u = 0U;

	provision_password("pw");
	(void)feed_req(MCP_CMD_AUTH, (const uint8_t *)"pw", 2U);
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 900U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));

	bytes_put_le32(req, 0xDEADBEEFU);
	(void)feed_req(MCP_CMD_FACTORY_RESET, req, 4U);
	expect_status(MCP_CMD_FACTORY_RESET, g_seq, (uint8_t)MCP_ERR_ARG);
	(void)feed_req(MCP_CMD_FACTORY_RESET, req, 3U);
	expect_status(MCP_CMD_FACTORY_RESET, g_seq, (uint8_t)MCP_ERR_ARG);
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(900U, u);

	bytes_put_le32(req, MCP_FACTORY_MAGIC);
	(void)feed_req(MCP_CMD_FACTORY_RESET, req, 4U);
	expect_status(MCP_CMD_FACTORY_RESET, g_seq, (uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(200U, u);

	/* The credential the session authenticated against is gone, so the
	 * session must be gone too. */
	TEST_ASSERT_FALSE(mcp_authenticated(&g_mcp));
}

/* ------------------------------------------------------------------------- */
/* Status, telemetry, diagnostics                                            */
/* ------------------------------------------------------------------------- */

static void test_status_get_is_versioned_and_group_tagged(void)
{
	uint8_t grp = MCP_GRP_GNSS;
	const uint8_t *p;

	(void)feed_req(MCP_CMD_STATUS_GET, &grp, 1U);
	expect_status(MCP_CMD_STATUS_GET, g_seq, (uint8_t)MCP_OK);
	p = tx_pay(last_tx());
	TEST_ASSERT_EQUAL_UINT8(MCP_GRP_GNSS, p[1]);
	TEST_ASSERT_EQUAL_UINT8(STATUS_STRUCT_VER, p[2]);
	TEST_ASSERT_EQUAL_UINT8(MCP_GRP_GNSS, p[3]);
	TEST_ASSERT_EQUAL_UINT16(8U, tx_plen(last_tx()));

	grp = MCP_GRP_COUNT;
	(void)feed_req(MCP_CMD_STATUS_GET, &grp, 1U);
	expect_status(MCP_CMD_STATUS_GET, g_seq, (uint8_t)MCP_ERR_ARG);
	(void)feed_req(MCP_CMD_STATUS_GET, NULL, 0U);
	expect_status(MCP_CMD_STATUS_GET, g_seq, (uint8_t)MCP_ERR_ARG);

	/* A group this build does not produce. */
	g_status_fail = MCP_GRP_PTP;
	g_status_rc = -ENOTSUP;
	grp = MCP_GRP_PTP;
	(void)feed_req(MCP_CMD_STATUS_GET, &grp, 1U);
	expect_status(MCP_CMD_STATUS_GET, g_seq, (uint8_t)MCP_ERR_NOTSUP);

	/* A source that fails some other way, and one that forgets the
	 * mandatory version byte. */
	g_status_rc = -EIO;
	(void)feed_req(MCP_CMD_STATUS_GET, &grp, 1U);
	expect_status(MCP_CMD_STATUS_GET, g_seq, (uint8_t)MCP_ERR_INTERNAL);
	g_status_rc = 0;
	(void)feed_req(MCP_CMD_STATUS_GET, &grp, 1U);
	expect_status(MCP_CMD_STATUS_GET, g_seq, (uint8_t)MCP_ERR_INTERNAL);
}

static void test_telemetry_cadence_and_unsubscribe(void)
{
	uint8_t sub[2] = { (1U << MCP_GRP_SUMMARY) | (1U << MCP_GRP_TIMING),
			   2U };

	(void)feed_req(MCP_CMD_TELEM_SUB, sub, sizeof(sub));
	expect_status(MCP_CMD_TELEM_SUB, g_seq, (uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_UINT8(sub[0], tx_pay(last_tx())[1]);
	TEST_ASSERT_EQUAL_UINT8(2U, tx_pay(last_tx())[2]);
	g_txn = 0U;

	/* 2 Hz: nothing before 500 ms, then one EVT per subscribed group. */
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 499U));
	TEST_ASSERT_EQUAL_size_t(0U, g_txn);

	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 500U));
	TEST_ASSERT_EQUAL_size_t(2U, g_txn);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_T_EVT, tx_type(&g_tx[0]));
	TEST_ASSERT_EQUAL_HEX8(MCP_CMD_TELEM_SUB, tx_cmd(&g_tx[0]));
	TEST_ASSERT_EQUAL_HEX8(MCP_EVT_SUB_TELEM,
			       tx_flags(&g_tx[0]) & MCP_EVT_SUB_MASK);
	TEST_ASSERT_EQUAL_UINT8(MCP_GRP_SUMMARY, tx_pay(&g_tx[0])[0]);
	TEST_ASSERT_EQUAL_UINT8(STATUS_STRUCT_VER, tx_pay(&g_tx[0])[1]);
	TEST_ASSERT_EQUAL_UINT8(MCP_GRP_TIMING, tx_pay(&g_tx[1])[0]);

	/* EVT sequence numbers are their own space and advance. */
	TEST_ASSERT_EQUAL_UINT16(tx_seq(&g_tx[0]) + 1U, tx_seq(&g_tx[1]));

	g_txn = 0U;
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 999U));
	TEST_ASSERT_EQUAL_size_t(0U, g_txn);
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 1000U));
	TEST_ASSERT_EQUAL_size_t(2U, g_txn);

	/* A long stall resynchronises instead of bursting a backlog. */
	g_txn = 0U;
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 60000U));
	TEST_ASSERT_EQUAL_size_t(2U, g_txn);
	g_txn = 0U;
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 60001U));
	TEST_ASSERT_EQUAL_size_t(0U, g_txn);

	/* Unsubscribe stops the stream. */
	(void)feed_req(MCP_CMD_TELEM_UNSUB, NULL, 0U);
	expect_status(MCP_CMD_TELEM_UNSUB, g_seq, (uint8_t)MCP_OK);
	g_txn = 0U;
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 120000U));
	TEST_ASSERT_EQUAL_size_t(0U, g_txn);
}

static void test_telemetry_argument_checks(void)
{
	uint8_t sub[2];

	sub[0] = 0U;
	sub[1] = 1U;
	(void)feed_req(MCP_CMD_TELEM_SUB, sub, 2U);
	expect_status(MCP_CMD_TELEM_SUB, g_seq, (uint8_t)MCP_ERR_ARG);

	sub[0] = 0x80U; /* a group bit that does not exist */
	(void)feed_req(MCP_CMD_TELEM_SUB, sub, 2U);
	expect_status(MCP_CMD_TELEM_SUB, g_seq, (uint8_t)MCP_ERR_ARG);

	sub[0] = 0x01U;
	sub[1] = 0U;
	(void)feed_req(MCP_CMD_TELEM_SUB, sub, 2U);
	expect_status(MCP_CMD_TELEM_SUB, g_seq, (uint8_t)MCP_ERR_ARG);
	sub[1] = 5U; /* above the 4 Hz ceiling */
	(void)feed_req(MCP_CMD_TELEM_SUB, sub, 2U);
	expect_status(MCP_CMD_TELEM_SUB, g_seq, (uint8_t)MCP_ERR_ARG);

	(void)feed_req(MCP_CMD_TELEM_SUB, sub, 1U);
	expect_status(MCP_CMD_TELEM_SUB, g_seq, (uint8_t)MCP_ERR_ARG);
	(void)feed_req(MCP_CMD_TELEM_UNSUB, sub, 1U);
	expect_status(MCP_CMD_TELEM_UNSUB, g_seq, (uint8_t)MCP_ERR_ARG);
}

static void test_telemetry_skips_a_group_it_cannot_render(void)
{
	uint8_t sub[2] = { (1U << MCP_GRP_SUMMARY) | (1U << MCP_GRP_PTP), 1U };

	g_status_fail = MCP_GRP_PTP;
	g_status_rc = -ENOTSUP;

	(void)feed_req(MCP_CMD_TELEM_SUB, sub, sizeof(sub));
	g_txn = 0U;
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 1000U));
	TEST_ASSERT_EQUAL_size_t(1U, g_txn);
	TEST_ASSERT_EQUAL_UINT32(1U, mcp_stats(&g_mcp)->evt_dropped);
}

static void test_diag_dispatch(void)
{
	uint8_t sub;
	const uint8_t *p;

	/* Sub 0 is the health snapshot and comes from the one status source,
	 * not from a second definition of "healthy". */
	sub = 0U;
	(void)feed_req(MCP_CMD_DIAG, &sub, 1U);
	expect_status(MCP_CMD_DIAG, g_seq, (uint8_t)MCP_OK);
	p = tx_pay(last_tx());
	TEST_ASSERT_EQUAL_UINT8(0U, p[1]);
	TEST_ASSERT_EQUAL_UINT8(STATUS_STRUCT_VER, p[2]);
	TEST_ASSERT_EQUAL_UINT8(MCP_GRP_SUMMARY, p[3]);

	sub = 2U;
	(void)feed_req(MCP_CMD_DIAG, &sub, 1U);
	expect_status(MCP_CMD_DIAG, g_seq, (uint8_t)MCP_OK);
	p = tx_pay(last_tx());
	TEST_ASSERT_EQUAL_UINT8(2U, p[1]);
	TEST_ASSERT_EQUAL_UINT8(2U, p[3]);

	sub = 3U; /* the fake reports this one unimplemented */
	(void)feed_req(MCP_CMD_DIAG, &sub, 1U);
	expect_status(MCP_CMD_DIAG, g_seq, (uint8_t)MCP_ERR_NOTSUP);

	sub = MCP_DIAG_SUB_COUNT;
	(void)feed_req(MCP_CMD_DIAG, &sub, 1U);
	expect_status(MCP_CMD_DIAG, g_seq, (uint8_t)MCP_ERR_ARG);

	(void)feed_req(MCP_CMD_DIAG, NULL, 0U);
	expect_status(MCP_CMD_DIAG, g_seq, (uint8_t)MCP_ERR_ARG);
}

/* ------------------------------------------------------------------------- */
/* Logging over the wire                                                     */
/* ------------------------------------------------------------------------- */

static void log_req(uint32_t cursor, uint8_t follow, uint8_t max,
		    uint8_t max_level, uint16_t sub_mask)
{
	uint8_t req[9];

	bytes_put_le32(req, cursor);
	req[4] = follow;
	req[5] = max;
	req[6] = max_level;
	bytes_put_le16(&req[7], sub_mask);
	(void)feed_req(MCP_CMD_LOG_TAIL, req, sizeof(req));
}

static void test_log_tail_returns_records_and_a_cursor(void)
{
	const uint8_t *p;
	size_t o;
	uint8_t n;

	TEST_ASSERT_EQUAL_INT(0, logr_puts(&g_log, LOGR_ERR, LOGR_SUB_TIMING,
					   11U, "pps lost"));
	TEST_ASSERT_EQUAL_INT(0, logr_puts(&g_log, LOGR_INFO, LOGR_SUB_GNSS,
					   12U, "fix 3d"));

	log_req(0U, 0U, 10U, (uint8_t)LOGR_DEBUG, 0U);
	expect_status(MCP_CMD_LOG_TAIL, g_seq, (uint8_t)MCP_OK);

	p = tx_pay(last_tx());
	TEST_ASSERT_EQUAL_UINT32(2U, bytes_get_le32(&p[1]));  /* next cursor */
	TEST_ASSERT_EQUAL_UINT32(0U, bytes_get_le32(&p[5]));  /* no gap */
	n = p[9];
	TEST_ASSERT_EQUAL_UINT8(2U, n);

	o = 10U;
	TEST_ASSERT_EQUAL_UINT32(0U, bytes_get_le32(&p[o]));
	TEST_ASSERT_EQUAL_UINT64(11U, bytes_get_le64(&p[o + 4U]));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)LOGR_ERR, p[o + 12U]);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)LOGR_SUB_TIMING, p[o + 13U]);
	TEST_ASSERT_EQUAL_UINT8(8U, p[o + 14U]);
	TEST_ASSERT_EQUAL_HEX8_ARRAY("pps lost", &p[o + 15U], 8);

	o += 15U + 8U;
	TEST_ASSERT_EQUAL_UINT32(1U, bytes_get_le32(&p[o]));
	TEST_ASSERT_EQUAL_UINT8(6U, p[o + 14U]);
	TEST_ASSERT_EQUAL_HEX8_ARRAY("fix 3d", &p[o + 15U], 6);

	/* Resuming from the returned cursor yields nothing new. */
	log_req(2U, 0U, 10U, (uint8_t)LOGR_DEBUG, 0U);
	TEST_ASSERT_EQUAL_UINT8(0U, tx_pay(last_tx())[9]);
}

static void test_log_tail_filters_and_reports_gaps(void)
{
	const uint8_t *p;
	unsigned int i;

	TEST_ASSERT_EQUAL_INT(0, logr_level_set(&g_log, 0xFFU, LOGR_DEBUG));
	for (i = 0U; i < 3U; i++) {
		TEST_ASSERT_EQUAL_INT(0,
			logr_puts(&g_log, LOGR_DEBUG, LOGR_SUB_NET, i, "chat"));
	}
	TEST_ASSERT_EQUAL_INT(0,
		logr_puts(&g_log, LOGR_CRIT, LOGR_SUB_PWR, 9U, "rail lost"));

	/* Severity filter. */
	log_req(0U, 0U, 10U, (uint8_t)LOGR_ERR, 0U);
	p = tx_pay(last_tx());
	TEST_ASSERT_EQUAL_UINT8(1U, p[9]);
	TEST_ASSERT_EQUAL_UINT32(4U, bytes_get_le32(&p[1]));

	/* Subsystem filter. */
	log_req(0U, 0U, 10U, (uint8_t)LOGR_DEBUG,
		(uint16_t)(1U << LOGR_SUB_NET));
	TEST_ASSERT_EQUAL_UINT8(3U, tx_pay(last_tx())[9]);

	/* Overrun the reader and check the gap is reported, not hidden. */
	for (i = 0U; i < (2U * LOG_CAP); i++) {
		TEST_ASSERT_EQUAL_INT(0,
			logr_puts(&g_log, LOGR_INFO, LOGR_SUB_SYS, i, "flood"));
	}
	log_req(0U, 0U, 4U, (uint8_t)LOGR_DEBUG, 0U);
	p = tx_pay(last_tx());
	TEST_ASSERT_EQUAL_UINT32(logr_oldest(&g_log), bytes_get_le32(&p[5]));
	TEST_ASSERT_EQUAL_UINT8(4U, p[9]);

	/* Argument checks. */
	log_req(0U, 0U, 4U, 8U, 0U);
	expect_status(MCP_CMD_LOG_TAIL, g_seq, (uint8_t)MCP_ERR_ARG);
	log_req(0U, 2U, 4U, (uint8_t)LOGR_DEBUG, 0U);
	expect_status(MCP_CMD_LOG_TAIL, g_seq, (uint8_t)MCP_ERR_ARG);
	(void)feed_req(MCP_CMD_LOG_TAIL, NULL, 0U);
	expect_status(MCP_CMD_LOG_TAIL, g_seq, (uint8_t)MCP_ERR_ARG);
}

static void test_log_follow_streams_events(void)
{
	const uint8_t *p;
	unsigned int i;

	log_req(0U, 1U, 8U, (uint8_t)LOGR_DEBUG, 0U);
	expect_status(MCP_CMD_LOG_TAIL, g_seq, (uint8_t)MCP_OK);
	g_txn = 0U;

	/* Nothing new: no event, and no wasted frame. */
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 10U));
	TEST_ASSERT_EQUAL_size_t(0U, g_txn);

	TEST_ASSERT_EQUAL_INT(0, logr_puts(&g_log, LOGR_WARN, LOGR_SUB_THERM,
					   20U, "fan slow"));
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 20U));
	TEST_ASSERT_EQUAL_size_t(1U, g_txn);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)MCP_T_EVT, tx_type(last_tx()));
	TEST_ASSERT_EQUAL_HEX8(MCP_CMD_LOG_TAIL, tx_cmd(last_tx()));
	TEST_ASSERT_EQUAL_HEX8(MCP_EVT_SUB_LOG,
			       tx_flags(last_tx()) & MCP_EVT_SUB_MASK);
	p = tx_pay(last_tx());
	TEST_ASSERT_EQUAL_UINT8(1U, p[8]);
	TEST_ASSERT_EQUAL_HEX8_ARRAY("fan slow", &p[9U + 15U], 8);

	/* Flood past the ring and confirm the follower is told what it lost. */
	g_txn = 0U;
	for (i = 0U; i < (3U * LOG_CAP); i++) {
		TEST_ASSERT_EQUAL_INT(0,
			logr_puts(&g_log, LOGR_INFO, LOGR_SUB_SYS, i, "flood"));
	}
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 30U));
	TEST_ASSERT_EQUAL_size_t(1U, g_txn);
	p = tx_pay(last_tx());
	TEST_ASSERT_TRUE(bytes_get_le32(&p[4]) > 0U); /* gap reported */
	TEST_ASSERT_EQUAL_UINT8(MCP_LOG_EVT_RECS, p[8]);

	/* A second LOG_TAIL with follow=0 turns the stream off. */
	log_req(0U, 0U, 1U, (uint8_t)LOGR_DEBUG, 0U);
	g_txn = 0U;
	TEST_ASSERT_EQUAL_INT(0,
		logr_puts(&g_log, LOGR_INFO, LOGR_SUB_SYS, 99U, "quiet"));
	TEST_ASSERT_EQUAL_INT(0, mcp_tick(&g_mcp, 40U));
	TEST_ASSERT_EQUAL_size_t(0U, g_txn);
}

static void test_log_level_over_the_wire(void)
{
	uint8_t req[2];

	policy_no_auth();

	req[0] = (uint8_t)LOGR_SUB_GNSS;
	req[1] = (uint8_t)LOGR_ERR;
	(void)feed_req(MCP_CMD_LOG_LEVEL, req, 2U);
	expect_status(MCP_CMD_LOG_LEVEL, g_seq, (uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_INT((int)LOGR_ERR,
			      logr_level_get(&g_log, LOGR_SUB_GNSS));

	req[0] = 0xFFU; /* all subsystems */
	req[1] = (uint8_t)LOGR_DEBUG;
	(void)feed_req(MCP_CMD_LOG_LEVEL, req, 2U);
	expect_status(MCP_CMD_LOG_LEVEL, g_seq, (uint8_t)MCP_OK);
	TEST_ASSERT_EQUAL_INT((int)LOGR_DEBUG,
			      logr_level_get(&g_log, LOGR_SUB_NTP));

	req[1] = 9U;
	(void)feed_req(MCP_CMD_LOG_LEVEL, req, 2U);
	expect_status(MCP_CMD_LOG_LEVEL, g_seq, (uint8_t)MCP_ERR_ARG);
	(void)feed_req(MCP_CMD_LOG_LEVEL, req, 1U);
	expect_status(MCP_CMD_LOG_LEVEL, g_seq, (uint8_t)MCP_ERR_ARG);
}

static void test_engine_events_reach_the_log(void)
{
	logr_rec_t rec;
	uint16_t n = 0U;
	uint32_t next = 0U;
	uint32_t gap = 0U;
	logr_filter_t f;

	provision_password("pw");
	(void)feed_req(MCP_CMD_AUTH, (const uint8_t *)"bad", 3U);

	logr_filter_all(&f);
	f.sub_mask = (uint16_t)(1U << LOGR_SUB_MCP);
	TEST_ASSERT_EQUAL_INT(0,
		logr_tail(&g_log, 0U, &f, &rec, 1U, &n, &next, &gap));
	TEST_ASSERT_EQUAL_UINT16(1U, n);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)LOGR_SUB_MCP, rec.subsys);
	TEST_ASSERT_EQUAL_HEX8_ARRAY("auth: rejected", rec.msg, 14);
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_wire_known_vector);
	RUN_TEST(test_wire_codec_rejects);

	RUN_TEST(test_frame_assembles_across_split_boundaries);
	RUN_TEST(test_two_frames_in_one_buffer);
	RUN_TEST(test_bad_crc_is_dropped_silently);
	RUN_TEST(test_malformed_frames_are_counted_not_answered);
	RUN_TEST(test_unknown_command);
	RUN_TEST(test_input_argument_checks);
	RUN_TEST(test_init_argument_checks);

	RUN_TEST(test_response_is_held_not_dropped);
	RUN_TEST(test_events_are_dropped_under_back_pressure);

	RUN_TEST(test_hello_reports_identity_and_capabilities);
	RUN_TEST(test_hello_capabilities_track_the_wiring);

	RUN_TEST(test_auth_success_and_failure);
	RUN_TEST(test_auth_without_a_provisioned_credential_locks_the_box);
	RUN_TEST(test_auth_needs_a_credential_store_and_crypto);
	RUN_TEST(test_auth_gating_matrix);
	RUN_TEST(test_session_expires_when_idle);
	RUN_TEST(test_reset_session_drops_everything_but_dfu);
	RUN_TEST(test_auth_blob_helper);

	RUN_TEST(test_reboot_is_deferred_until_the_answer_is_out);
	RUN_TEST(test_reboot_validates_its_mode);

	RUN_TEST(test_cfg_list_pages_in_id_order);
	RUN_TEST(test_cfg_get_set_commit_round_trip);
	RUN_TEST(test_cfg_revert_drops_the_staged_set);
	RUN_TEST(test_cfg_argument_errors);
	RUN_TEST(test_cfg_secret_needs_a_session);
	RUN_TEST(test_cfg_export_streams_and_reimports);
	RUN_TEST(test_cfg_export_secrets_need_a_session);
	RUN_TEST(test_cfg_import_streams_in_chunks);
	RUN_TEST(test_cfg_import_rejects_bad_streams);
	RUN_TEST(test_factory_reset_needs_the_magic);

	RUN_TEST(test_status_get_is_versioned_and_group_tagged);
	RUN_TEST(test_telemetry_cadence_and_unsubscribe);
	RUN_TEST(test_telemetry_argument_checks);
	RUN_TEST(test_telemetry_skips_a_group_it_cannot_render);
	RUN_TEST(test_diag_dispatch);

	RUN_TEST(test_log_tail_returns_records_and_a_cursor);
	RUN_TEST(test_log_tail_filters_and_reports_gaps);
	RUN_TEST(test_log_follow_streams_events);
	RUN_TEST(test_log_level_over_the_wire);
	RUN_TEST(test_engine_events_reach_the_log);

	return UNITY_END();
}
