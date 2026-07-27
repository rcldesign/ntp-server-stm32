/*
 * STS1000 "Meridian" — core/mcp: the FW_* (DFU) command family.
 *
 * ARCHITECTURE.md §7 and §3 upgrade path 1: the PC tool streams a signed
 * MCUboot image into the staging slot over the console channel, this module
 * verifies it, and MCUboot does the rest on the next boot. Everything here
 * runs against port_image_t, so the host tests drive the real state machine
 * against a RAM-backed slot.
 *
 * Why the session is stateful
 * ---------------------------
 * A 900 KB image over a serial link takes long enough that a disconnect
 * mid-transfer is normal, not exceptional. The three properties that make that
 * survivable are all implemented here:
 *
 *   idempotence — a FW_DATA whose range is entirely below the write frontier
 *                 is acknowledged without re-writing flash, so a lost ACK
 *                 costs one round trip instead of the whole upload;
 *   gap refusal — a FW_DATA above the frontier is refused with the expected
 *                 offset rather than leaving a hole in a region the erase
 *                 window has already passed;
 *   resume      — a FW_BEGIN repeating the same size and SHA-256 re-attaches
 *                 to the existing session and re-reports the frontier without
 *                 erasing anything.
 *
 * Erase is progressive: the window is pushed one granule ahead of the write
 * frontier as data arrives, so a FW_BEGIN does not stall for the seconds a
 * whole-slot erase would take, and a resumed session does not re-erase bytes
 * it already holds.
 */

#include "mcp/mcp.h"
#include "mcp/mcp_internal.h"

#include <errno.h>
#include <string.h>

#include "util/bytes.h"

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */

uint8_t mcp_dfu__slot_flags(const port_image_info_t *info)
{
	uint8_t f = 0U;

	if (info->valid) {
		f |= MCP_SLOT_VALID;
	}
	if (info->active) {
		f |= MCP_SLOT_ACTIVE;
	}
	if (info->pending) {
		f |= MCP_SLOT_PENDING;
	}
	if (info->confirmed) {
		f |= MCP_SLOT_CONFIRMED;
	}
	return f;
}

void mcp_dfu__reset(mcp_ctx_t *c)
{
	uint32_t gran = c->dfu.erase_gran;

	memset(&c->dfu, 0, sizeof(c->dfu));
	c->dfu.state = (uint8_t)MCP_DFU_IDLE;
	c->dfu.erase_gran = gran;
}

void mcp_dfu__tick(mcp_ctx_t *c, uint64_t now_ms)
{
	if (c->dfu.state != (uint8_t)MCP_DFU_ACTIVE) {
		return;
	}
	if ((now_ms - c->dfu.last_ms) < MCP_DFU_TIMEOUT_MS) {
		return;
	}

	/* Suspend, do not discard: the erased-and-written prefix is still good
	 * and an identical FW_BEGIN picks it back up. */
	c->dfu.state = (uint8_t)MCP_DFU_SUSPENDED;
	c->dfu.timeouts++;
	mcp__log(c, (uint8_t)LOGR_WARN, "dfu: session idle, suspended");
}

static uint32_t align_up(uint32_t v, uint32_t gran)
{
	uint32_t r;

	if (gran <= 1U) {
		return v;
	}
	r = v % gran;
	if (r == 0U) {
		return v;
	}
	if (v > (UINT32_MAX - (gran - r))) {
		return UINT32_MAX;
	}
	return v + (gran - r);
}

static uint32_t staging_cap(const mcp_ctx_t *c)
{
	if ((c->w.img == NULL) || (c->w.img->staging_size == NULL)) {
		return 0U;
	}
	return c->w.img->staging_size(c->w.img->ctx);
}

/*
 * Guarantee that [0, need_end) is erased, pushing the window one granule past
 * the request so the next chunk does not have to wait for an erase.
 */
static int ensure_erased(mcp_ctx_t *c, uint32_t need_end)
{
	uint32_t gran = c->dfu.erase_gran;
	uint32_t limit;
	uint32_t target;
	uint32_t cap;
	int rc;

	if (c->w.img->staging_erase == NULL) {
		return -ENOTSUP;
	}

	cap = staging_cap(c);
	limit = align_up(c->dfu.total, gran);
	if (limit > cap) {
		limit = cap;
	}

	target = (need_end > (UINT32_MAX - gran)) ? limit : (need_end + gran);
	target = align_up(target, gran);
	if (target > limit) {
		target = limit;
	}
	if (target < need_end) {
		target = need_end; /* a granule bigger than the slot */
	}
	if (target <= c->dfu.erased) {
		return 0;
	}

	rc = c->w.img->staging_erase(c->w.img->ctx, c->dfu.erased,
				     target - c->dfu.erased);
	if (rc != 0) {
		return rc;
	}
	c->dfu.erased = target;
	return 0;
}

/* Stream the staged image back through the incremental SHA-256 port. */
static int verify_hash(mcp_ctx_t *c, uint8_t out[32])
{
	uint8_t state[PORT_SHA256_CTX_SIZE];
	uint8_t buf[MCP_DFU_VERIFY_CHUNK];
	uint32_t off = 0U;
	int rc;

	if ((c->w.sha == NULL) || (c->w.sha->init == NULL) ||
	    (c->w.sha->update == NULL) || (c->w.sha->final == NULL) ||
	    (c->w.img->staging_read == NULL)) {
		return -ENOTSUP;
	}

	rc = c->w.sha->init(c->w.sha->ctx, state);
	if (rc != 0) {
		return rc;
	}

	while (off < c->dfu.total) {
		uint32_t n = c->dfu.total - off;

		if (n > sizeof(buf)) {
			n = (uint32_t)sizeof(buf);
		}
		rc = c->w.img->staging_read(c->w.img->ctx, off, buf, n);
		if (rc != 0) {
			return rc;
		}
		rc = c->w.sha->update(c->w.sha->ctx, state, buf, n);
		if (rc != 0) {
			return rc;
		}
		off += n;
	}

	return c->w.sha->final(c->w.sha->ctx, state, out);
}

/* ------------------------------------------------------------------------- */
/* Handlers                                                                  */
/* ------------------------------------------------------------------------- */

static int h_fw_info(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t *p = mcp__rsp_buf(c);
	size_t o = 0U;
	uint8_t slot;

	if (f->len != 0U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}

	p[o++] = (uint8_t)MCP_OK;
	p[o++] = 2U;

	for (slot = 0U; slot < 2U; slot++) {
		port_image_info_t info;
		size_t i;

		memset(&info, 0, sizeof(info));
		if (c->w.img->image_info != NULL) {
			if (c->w.img->image_info(c->w.img->ctx, slot,
						 &info) != 0) {
				memset(&info, 0, sizeof(info));
			}
		}

		p[o++] = slot;
		p[o++] = mcp_dfu__slot_flags(&info);
		bytes_put_le32(&p[o], info.size);
		o += 4U;
		for (i = 0U; i < 4U; i++) {
			bytes_put_le32(&p[o], info.version[i]);
			o += 4U;
		}
	}

	p[o++] = c->dfu.state;
	bytes_put_le32(&p[o], c->dfu.total);
	o += 4U;
	bytes_put_le32(&p[o], c->dfu.written);
	o += 4U;
	bytes_put_le32(&p[o], (uint32_t)MCP_FW_CHUNK_MAX);
	o += 4U;

	return mcp__reply(c, f->cmd, f->seq, (uint16_t)o);
}

static int h_fw_begin(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t *p = mcp__rsp_buf(c);
	uint32_t size;
	uint32_t cap;
	bool resumed;

	if (f->len != (4U + 32U)) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	if ((c->w.sha == NULL) || (c->w.img->staging_write == NULL)) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_NOTSUP);
	}

	size = bytes_get_le32(f->payload);
	cap = staging_cap(c);
	if (size == 0U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	if ((cap == 0U) || (size > cap)) {
		mcp__log(c, (uint8_t)LOGR_ERR, "dfu: image exceeds staging slot");
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_NOSPC);
	}

	resumed = ((c->dfu.state == (uint8_t)MCP_DFU_ACTIVE) ||
		   (c->dfu.state == (uint8_t)MCP_DFU_SUSPENDED)) &&
		  (c->dfu.total == size) &&
		  (memcmp(c->dfu.sha, &f->payload[4], 32) == 0);

	if (!resumed) {
		mcp_dfu__reset(c);
		c->dfu.total = size;
		memcpy(c->dfu.sha, &f->payload[4], 32);
	}

	c->dfu.state = (uint8_t)MCP_DFU_ACTIVE;
	c->dfu.last_ms = c->now_ms;

	if (!resumed && (ensure_erased(c, 0U) != 0)) {
		mcp_dfu__reset(c);
		mcp__log(c, (uint8_t)LOGR_ERR, "dfu: staging erase failed");
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_INTERNAL);
	}

	mcp__log(c, (uint8_t)LOGR_NOTICE,
		 resumed ? "dfu: session resumed" : "dfu: session started");

	p[0] = (uint8_t)MCP_OK;
	bytes_put_le32(&p[1], c->dfu.written);
	bytes_put_le32(&p[5], (uint32_t)MCP_FW_CHUNK_MAX);
	p[9] = resumed ? 1U : 0U;
	return mcp__reply(c, f->cmd, f->seq, 10U);
}

/* Every FW_DATA answer carries the frontier, success or not. */
static int fw_data_reply(mcp_ctx_t *c, const mcp_frame_t *f, uint8_t status)
{
	uint8_t *p = mcp__rsp_buf(c);

	p[0] = status;
	bytes_put_le32(&p[1], c->dfu.written);
	return mcp__reply(c, f->cmd, f->seq, 5U);
}

static int h_fw_data(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint32_t off;
	size_t dlen;
	bool flush;
	int rc;

	if (c->dfu.state != (uint8_t)MCP_DFU_ACTIVE) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_STATE);
	}
	if (f->len < 5U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}

	off = bytes_get_le32(f->payload);
	dlen = (size_t)f->len - 4U;
	if (dlen > MCP_FW_CHUNK_MAX) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	if ((off > c->dfu.total) || (dlen > (size_t)(c->dfu.total - off))) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}

	if (off != c->dfu.written) {
		if ((off < c->dfu.written) &&
		    ((off + (uint32_t)dlen) <= c->dfu.written)) {
			/* Wholly below the frontier: a retransmit after a lost
			 * ACK. Acknowledge without touching flash. */
			c->dfu.retransmits++;
			c->dfu.last_ms = c->now_ms;
			return fw_data_reply(c, f, (uint8_t)MCP_OK);
		}
		/* Ahead of the frontier, or straddling it — a straddling chunk
		 * would have to rewrite already-programmed flash, which NOR
		 * cannot do without an erase. Send the tool back. */
		return fw_data_reply(c, f, (uint8_t)MCP_ERR_OFFSET);
	}

	rc = ensure_erased(c, off + (uint32_t)dlen);
	if (rc != 0) {
		mcp__log(c, (uint8_t)LOGR_ERR, "dfu: erase failed");
		return fw_data_reply(c, f, mcp__port_err(rc));
	}

	flush = ((off + (uint32_t)dlen) == c->dfu.total);
	rc = c->w.img->staging_write(c->w.img->ctx, off, &f->payload[4], dlen,
				     flush);
	if (rc != 0) {
		mcp__log(c, (uint8_t)LOGR_ERR, "dfu: write failed");
		return fw_data_reply(c, f, mcp__port_err(rc));
	}

	c->dfu.written = off + (uint32_t)dlen;
	c->dfu.chunks++;
	c->dfu.last_ms = c->now_ms;
	return fw_data_reply(c, f, (uint8_t)MCP_OK);
}

static int h_fw_end(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t *p = mcp__rsp_buf(c);
	uint8_t digest[32];
	uint8_t hdr[4];
	uint32_t size;
	int rc;

	if (f->len != 0U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	if ((c->dfu.state != (uint8_t)MCP_DFU_ACTIVE) &&
	    (c->dfu.state != (uint8_t)MCP_DFU_VERIFIED)) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_STATE);
	}
	if (c->dfu.written != c->dfu.total) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_STATE);
	}
	if (c->w.img->staging_read == NULL) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_NOTSUP);
	}

	/* Cheap structural check before the expensive one: anything that is not
	 * an MCUboot image cannot be worth hashing. */
	if (c->dfu.total < sizeof(hdr)) {
		mcp_dfu__reset(c);
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_VERIFY);
	}
	rc = c->w.img->staging_read(c->w.img->ctx, 0U, hdr, sizeof(hdr));
	if (rc != 0) {
		return mcp__reply_status(c, f->cmd, f->seq, mcp__port_err(rc));
	}
	if (bytes_get_le32(hdr) != MCP_MCUBOOT_MAGIC) {
		mcp__log(c, (uint8_t)LOGR_ERR, "dfu: not an MCUboot image");
		mcp_dfu__reset(c);
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_VERIFY);
	}

	rc = verify_hash(c, digest);
	if (rc != 0) {
		return mcp__reply_status(c, f->cmd, f->seq, mcp__port_err(rc));
	}
	if (memcmp(digest, c->dfu.sha, sizeof(digest)) != 0) {
		/* The staged bytes are known bad, so the session must not stay
		 * resumable — a later FW_BEGIN has to erase and start over. */
		mcp__log(c, (uint8_t)LOGR_ERR, "dfu: SHA-256 mismatch");
		mcp_dfu__reset(c);
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_VERIFY);
	}

	if (c->w.img->mark_pending == NULL) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_NOTSUP);
	}
	rc = c->w.img->mark_pending(c->w.img->ctx);
	if (rc != 0) {
		mcp__log(c, (uint8_t)LOGR_ERR, "dfu: mark-pending failed");
		return mcp__reply_status(c, f->cmd, f->seq, mcp__port_err(rc));
	}

	size = c->dfu.total;
	c->dfu.state = (uint8_t)MCP_DFU_VERIFIED;
	mcp__log(c, (uint8_t)LOGR_NOTICE, "dfu: image staged and pending");

	p[0] = (uint8_t)MCP_OK;
	bytes_put_le32(&p[1], size);
	return mcp__reply(c, f->cmd, f->seq, 5U);
}

static int h_fw_confirm(mcp_ctx_t *c, const mcp_frame_t *f)
{
	int rc;

	if (f->len != 0U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	if (c->w.img->confirm_active == NULL) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_NOTSUP);
	}
	rc = c->w.img->confirm_active(c->w.img->ctx);
	if (rc == 0) {
		mcp__log(c, (uint8_t)LOGR_NOTICE, "dfu: running image confirmed");
	}
	return mcp__reply_status(c, f->cmd, f->seq, mcp__port_err(rc));
}

static int h_fw_revert(mcp_ctx_t *c, const mcp_frame_t *f)
{
	int rc;

	if (f->len != 0U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	if (c->w.img->request_revert == NULL) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_NOTSUP);
	}
	rc = c->w.img->request_revert(c->w.img->ctx);
	if (rc == 0) {
		/* Whatever was staged is no longer the plan for the next boot. */
		mcp_dfu__reset(c);
		mcp__log(c, (uint8_t)LOGR_NOTICE, "dfu: revert requested");
	}
	return mcp__reply_status(c, f->cmd, f->seq, mcp__port_err(rc));
}

int mcp_dfu__handle(mcp_ctx_t *c, const mcp_frame_t *f)
{
	if (c->w.img == NULL) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_NOTSUP);
	}

	switch (f->cmd) {
	case MCP_CMD_FW_INFO:
		return h_fw_info(c, f);
	case MCP_CMD_FW_BEGIN:
		return h_fw_begin(c, f);
	case MCP_CMD_FW_DATA:
		return h_fw_data(c, f);
	case MCP_CMD_FW_END:
		return h_fw_end(c, f);
	case MCP_CMD_FW_CONFIRM:
		return h_fw_confirm(c, f);
	default: /* MCP_CMD_FW_REVERT */
		return h_fw_revert(c, f);
	}
}
