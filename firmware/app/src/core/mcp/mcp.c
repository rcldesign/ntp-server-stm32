/*
 * STS1000 "Meridian" — core/mcp: Meridian Console Protocol engine.
 *
 * See mcp.h for the contract and mcp_wire.h for the byte layouts. The DFU
 * command family lives in mcp_dfu.c.
 */

#include "mcp/mcp.h"
#include "mcp/mcp_internal.h"

#include <errno.h>
#include <string.h>

#include "port/port.h"
#include "util/bytes.h"
#include "util/cobs.h"
#include "util/crc.h"

_Static_assert(MCP_CFG_EXPORT_CHUNK >= CFG_EXPORT_MIN_CHUNK,
	       "export chunk must hold at least one record");
_Static_assert(MCP_CFG_EXPORT_CHUNK <= (MCP_MAX_PAYLOAD - 6U),
	       "export chunk must fit the response payload after its 6-byte head");

/* ------------------------------------------------------------------------- */
/* Wire codec                                                                */
/* ------------------------------------------------------------------------- */

int mcp_wire_seal(uint8_t *out, size_t cap, uint8_t type, uint8_t cmd,
		  uint8_t flags, uint16_t seq, uint16_t len, size_t *out_n)
{
	size_t n;
	uint32_t crc;

	if ((out == NULL) || (out_n == NULL)) {
		return -EINVAL;
	}
	if (len > MCP_MAX_PAYLOAD) {
		return -EMSGSIZE;
	}
	n = MCP_HDR_LEN + (size_t)len + MCP_CRC_LEN;
	if (cap < n) {
		return -ENOSPC;
	}

	out[0] = (uint8_t)MCP_VER;
	out[1] = type;
	out[2] = cmd;
	out[3] = flags;
	bytes_put_le16(&out[4], seq);
	bytes_put_le16(&out[6], len);

	crc = sts_crc32_ieee(out, MCP_HDR_LEN + (size_t)len);
	bytes_put_le32(&out[MCP_HDR_LEN + len], crc);

	*out_n = n;
	return 0;
}

int mcp_wire_build(uint8_t type, uint8_t cmd, uint8_t flags, uint16_t seq,
		   const uint8_t *payload, uint16_t len,
		   uint8_t *out, size_t cap, size_t *out_n)
{
	if ((out == NULL) || (out_n == NULL)) {
		return -EINVAL;
	}
	if ((payload == NULL) && (len != 0U)) {
		return -EINVAL;
	}
	if (len > MCP_MAX_PAYLOAD) {
		return -EMSGSIZE;
	}
	if (cap < (MCP_HDR_LEN + (size_t)len + MCP_CRC_LEN)) {
		return -ENOSPC;
	}
	if (len != 0U) {
		memcpy(&out[MCP_HDR_LEN], payload, len);
	}
	return mcp_wire_seal(out, cap, type, cmd, flags, seq, len, out_n);
}

int mcp_wire_parse(const uint8_t *frame, size_t n, mcp_frame_t *out)
{
	uint16_t len;
	uint32_t want;
	uint32_t got;

	if ((frame == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if ((n < MCP_MIN_FRAME) || (n > MCP_MAX_FRAME)) {
		return -EBADMSG;
	}

	len = bytes_get_le16(&frame[6]);
	if ((size_t)len != (n - MCP_MIN_FRAME)) {
		return -EPROTO;
	}

	want = bytes_get_le32(&frame[n - MCP_CRC_LEN]);
	got = sts_crc32_ieee(frame, n - MCP_CRC_LEN);
	if (want != got) {
		return -EILSEQ;
	}

	out->ver = frame[0];
	out->type = frame[1];
	out->cmd = frame[2];
	out->flags = frame[3];
	out->seq = bytes_get_le16(&frame[4]);
	out->len = len;
	out->payload = &frame[MCP_HDR_LEN];
	return 0;
}

/* ------------------------------------------------------------------------- */
/* Transmit path                                                             */
/* ------------------------------------------------------------------------- */

uint8_t *mcp__rsp_buf(mcp_ctx_t *c)
{
	return &c->frame[MCP_HDR_LEN];
}

/*
 * Ports report failure either with the platform libc's errno numbering or with
 * the fixed PORT_* aliases declared in port/port.h. Those two agree on
 * glibc/newlib but not on Zephyr's minimal libc, where ENOTSUP is 134 rather
 * than 95, so both encodings are recognised.
 */
uint8_t mcp__port_err(int rc)
{
	if (rc == 0) {
		return (uint8_t)MCP_OK;
	}
	if ((rc == -ENOTSUP) || (rc == PORT_ENOTSUP)) {
		return (uint8_t)MCP_ERR_NOTSUP;
	}
	if ((rc == -EBUSY) || (rc == PORT_EBUSY)) {
		return (uint8_t)MCP_ERR_BUSY;
	}
	return (uint8_t)MCP_ERR_INTERNAL;
}

void mcp__log(mcp_ctx_t *c, uint8_t level, const char *msg)
{
	if (c->w.log != NULL) {
		(void)logr_puts(c->w.log, level, (uint8_t)LOGR_SUB_MCP,
				c->now_ms, msg);
	}
}

/* Seal the frame buffer and COBS-encode it into c->enc, delimiter included. */
static int frame_encode(mcp_ctx_t *c, uint8_t type, uint8_t cmd, uint8_t flags,
			uint16_t seq, uint16_t len)
{
	size_t fn = 0U;
	size_t en = 0U;
	int rc;

	rc = mcp_wire_seal(c->frame, sizeof(c->frame), type, cmd, flags, seq,
			   len, &fn);
	if (rc != 0) {
		return rc;
	}
	rc = cobs_encode(c->frame, fn, c->enc, sizeof(c->enc) - 1U, &en);
	if (rc != 0) {
		return rc;
	}
	c->enc[en] = 0x00U;
	c->enc_len = en + 1U;
	return 0;
}

int mcp_poll_tx(mcp_ctx_t *c)
{
	int rc;

	if (c == NULL) {
		return -EINVAL;
	}
	if (!c->tx_pending) {
		return 0;
	}

	rc = c->w.tx(c->w.tx_user, c->enc, c->enc_len);
	if (rc != 0) {
		c->stats.tx_stalls++;
		return 1;
	}
	c->tx_pending = false;
	c->stats.rsp_tx++;
	return 0;
}

int mcp__reply(mcp_ctx_t *c, uint8_t cmd, uint16_t seq, uint16_t len)
{
	if (frame_encode(c, (uint8_t)MCP_T_RSP, cmd, 0U, seq, len) != 0) {
		/* A handler produced an over-long payload: that is a firmware
		 * bug, but the peer still gets an answer rather than a hang. */
		uint8_t *p = mcp__rsp_buf(c);

		p[0] = (uint8_t)MCP_ERR_INTERNAL;
		if (frame_encode(c, (uint8_t)MCP_T_RSP, cmd, 0U, seq, 1U) != 0) {
			return -EIO;
		}
	}

	c->tx_pending = true;
	return (mcp_poll_tx(c) == 1) ? -EAGAIN : 0;
}

int mcp__reply_status(mcp_ctx_t *c, uint8_t cmd, uint16_t seq, uint8_t status)
{
	uint8_t *p = mcp__rsp_buf(c);

	p[0] = status;
	return mcp__reply(c, cmd, seq, 1U);
}

/* EVTs are droppable by contract: never hold the link for one. */
static void emit_evt(mcp_ctx_t *c, uint8_t cmd, uint8_t sub, uint16_t len)
{
	if (c->tx_pending) {
		c->stats.evt_dropped++;
		return;
	}
	if (frame_encode(c, (uint8_t)MCP_T_EVT, cmd,
			 (uint8_t)(sub & MCP_EVT_SUB_MASK), c->evt_seq,
			 len) != 0) {
		c->stats.evt_dropped++;
		return;
	}
	c->evt_seq++;
	if (c->w.tx(c->w.tx_user, c->enc, c->enc_len) != 0) {
		c->stats.evt_dropped++;
		return;
	}
	c->stats.evt_tx++;
}

/* ------------------------------------------------------------------------- */
/* Config access: locking and commits                                        */
/* ------------------------------------------------------------------------- */

/*
 * mcp.h "Config locking": the glue's critical section over the shared
 * cfg_ctx_t. Taken per REQUEST rather than per call, so a multi-step operation
 * (feed a CFG_IMPORT chunk, then commit it) is atomic against the shell and the
 * local UI. Never nested by the engine.
 */
static void cfg_enter(mcp_ctx_t *c)
{
	if (c->w.cfg_lock != NULL) {
		c->w.cfg_lock(c->w.cfg_lock_user);
	}
}

static void cfg_leave(mcp_ctx_t *c)
{
	if (c->w.cfg_unlock != NULL) {
		c->w.cfg_unlock(c->w.cfg_lock_user);
	}
}

/*
 * mcp.h "Config commits": the ONE place the engine commits. Routed to the glue
 * so the per-group appliers run — a wire commit that skipped them would answer
 * "applied, no reboot needed" while nothing in the running system had changed.
 *
 * @p locked says whether the caller holds the cfg critical section; the callback
 * is always invoked without it, because it takes the config mutex itself and
 * then dispatches appliers that may block.
 */
static int cfg_commit_through_glue(mcp_ctx_t *c, cfg_commit_res_t *res,
				   bool locked)
{
	int rc;

	if (c->w.cfg_commit_cb == NULL) {
		return cfg_commit(c->w.cfg, res);
	}

	if (locked) {
		cfg_leave(c);
	}
	rc = c->w.cfg_commit_cb(c->w.cfg_commit_user, res);
	if (locked) {
		cfg_enter(c);
	}
	return rc;
}

/*
 * Same shape and the same reason, for FACTORY_RESET (mcp.h "Config commits"). A
 * reset that rewrites every key and tells no subscriber is the commit defect with
 * a wider blast radius: the reply says the unit is back to defaults while the
 * network stack, the log level, the display and the timing loop all keep the
 * configuration they were handed before it.
 */
static int cfg_factory_through_glue(mcp_ctx_t *c, bool locked)
{
	int rc;

	if (c->w.cfg_factory_cb == NULL) {
		return cfg_factory_reset(c->w.cfg);
	}

	if (locked) {
		cfg_leave(c);
	}
	rc = c->w.cfg_factory_cb(c->w.cfg_factory_user);
	if (locked) {
		cfg_enter(c);
	}
	return rc;
}

/* ------------------------------------------------------------------------- */
/* Session / policy                                                          */
/* ------------------------------------------------------------------------- */

/*
 * Commands that change persistent or operational state. Everything here is
 * gated on an authenticated session whenever policy demands one.
 *
 * CFG_REVERT is in the list even though ARCHITECTURE.md §7 only calls out
 * SET/COMMIT/IMPORT: it discards whatever an operator has staged, which is a
 * mutation of session state, and gating it costs nothing because it is only
 * reachable after CFG_SET, which is gated anyway.
 */
static bool cmd_mutating(uint8_t cmd)
{
	switch (cmd) {
	case MCP_CMD_REBOOT:
	case MCP_CMD_CFG_SET:
	case MCP_CMD_CFG_COMMIT:
	case MCP_CMD_CFG_REVERT:
	case MCP_CMD_CFG_IMPORT:
	case MCP_CMD_FACTORY_RESET:
	case MCP_CMD_LOG_LEVEL:
	case MCP_CMD_FW_BEGIN:
	case MCP_CMD_FW_DATA:
	case MCP_CMD_FW_END:
	case MCP_CMD_FW_CONFIRM:
	case MCP_CMD_FW_REVERT:
		return true;
	default:
		return false;
	}
}

static bool auth_required(const mcp_ctx_t *c)
{
	bool req = true;

	if (c->w.cfg == NULL) {
		/* No policy store and no credential store: see mcp.h. */
		return false;
	}
	if (cfg_get_bool(c->w.cfg, (uint16_t)CFG_ID_SEC_AUTH_REQUIRED,
			 &req) != 0) {
		return true; /* fail closed */
	}
	return req;
}

/* True when the peer may see secrets and issue mutating commands. */
static bool session_ok(const mcp_ctx_t *c)
{
	return c->authed || !auth_required(c);
}

static uint32_t session_seconds(const mcp_ctx_t *c)
{
	uint64_t s = MCP_DEFAULT_SESSION_S;

	if (c->w.cfg != NULL) {
		(void)cfg_get_u64(c->w.cfg, (uint16_t)CFG_ID_SEC_SESSION_S, &s);
	}
	return (uint32_t)s;
}

/* Constant-time equality: the comparison itself must not leak how much of a
 * candidate MAC matched. */
static bool ct_eq(const uint8_t *a, const uint8_t *b, size_t n)
{
	uint8_t diff = 0U;
	size_t i;

	for (i = 0U; i < n; i++) {
		diff |= (uint8_t)(a[i] ^ b[i]);
	}
	return diff == 0U;
}

int mcp_auth_make_blob(const port_crypto_t *crypto, const uint8_t *salt,
		       const uint8_t *pw, size_t pw_len, uint8_t *out,
		       size_t cap)
{
	uint8_t s[MCP_PW_SALT_LEN];

	if ((crypto == NULL) || (pw == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if ((pw_len == 0U) || (pw_len > MCP_PW_MAX) ||
	    (cap < MCP_PW_BLOB_LEN)) {
		return -EINVAL;
	}
	if (crypto->hmac_sha256 == NULL) {
		return -ENOTSUP;
	}

	if (salt != NULL) {
		memcpy(s, salt, sizeof(s));
	} else {
		if (crypto->rand == NULL) {
			return -ENOTSUP;
		}
		if (crypto->rand(crypto->ctx, s, sizeof(s)) != 0) {
			return -EIO;
		}
	}

	memcpy(out, s, sizeof(s));
	if (crypto->hmac_sha256(crypto->ctx, s, sizeof(s), pw, pw_len,
				&out[MCP_PW_SALT_LEN]) != 0) {
		return -EIO;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/* Handlers — identity and session                                           */
/* ------------------------------------------------------------------------- */

/* 21 bytes: u32 version[4], u32 size, u8 flags. */
static size_t slot_block(mcp_ctx_t *c, uint8_t slot, uint8_t *p)
{
	port_image_info_t info;
	size_t i;

	memset(&info, 0, sizeof(info));
	if ((c->w.img != NULL) && (c->w.img->image_info != NULL)) {
		if (c->w.img->image_info(c->w.img->ctx, slot, &info) != 0) {
			memset(&info, 0, sizeof(info));
		}
	}

	for (i = 0U; i < 4U; i++) {
		bytes_put_le32(&p[i * 4U], info.version[i]);
	}
	bytes_put_le32(&p[16], info.size);
	p[20] = mcp_dfu__slot_flags(&info);
	return 21U;
}

static int h_hello(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t *p = mcp__rsp_buf(c);
	size_t o = 0U;

	p[o++] = (uint8_t)MCP_OK;
	p[o++] = (uint8_t)MCP_VER;
	bytes_put_le16(&p[o], (uint16_t)MCP_MAX_PAYLOAD);
	o += 2U;
	bytes_put_le32(&p[o], c->caps);
	o += 4U;
	memcpy(&p[o], c->w.ident.model, sizeof(c->w.ident.model));
	o += sizeof(c->w.ident.model);
	memcpy(&p[o], c->w.ident.board_id, sizeof(c->w.ident.board_id));
	o += sizeof(c->w.ident.board_id);
	o += slot_block(c, 0U, &p[o]);
	o += slot_block(c, 1U, &p[o]);

	return mcp__reply(c, f->cmd, f->seq, (uint16_t)o);
}

/* Arm the brute-force backoff after a mismatch. Escalates from a doubling
 * throttle window into a long lockout window (spec §9.4). */
static void auth_penalise(mcp_ctx_t *c)
{
	c->auth_fails++;

	if (c->auth_fails >= MCP_AUTH_LOCK_TRIES) {
		c->auth_lock_until_ms = c->now_ms + MCP_AUTH_LOCKOUT_MS;
	} else if (c->auth_fails > MCP_AUTH_FREE_TRIES) {
		uint32_t step = c->auth_fails - MCP_AUTH_FREE_TRIES; /* 1,2,3… */
		uint64_t delay = (uint64_t)MCP_AUTH_THROTTLE_MS
				 << (step - 1U);

		if (delay > MCP_AUTH_THROTTLE_MAX_MS) {
			delay = MCP_AUTH_THROTTLE_MAX_MS;
		}
		c->auth_lock_until_ms = c->now_ms + delay;
	}
}

static int h_auth(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t blob[MCP_PW_BLOB_LEN];
	uint8_t mac[MCP_PW_MAC_LEN];
	uint8_t *p = mcp__rsp_buf(c);
	size_t blob_len = 0U;

	if ((c->w.cfg == NULL) || (c->w.crypto == NULL) ||
	    (c->w.crypto->hmac_sha256 == NULL)) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_NOTSUP);
	}

	/* Brute-force backoff: while a window is armed, refuse without testing
	 * the password so guesses cannot be spun faster than the schedule. The
	 * window is wall-clock based and survives a reconnect. */
	if ((c->auth_lock_until_ms != 0U) &&
	    (c->now_ms < c->auth_lock_until_ms)) {
		c->stats.auth_throttled++;
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_BUSY);
	}

	if ((f->len == 0U) || (f->len > MCP_PW_MAX)) {
		/* A malformed request is a client bug, not a guess: it neither
		 * counts toward the lockout nor is throttled by it. */
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	if (cfg_get_bytes(c->w.cfg, (uint16_t)CFG_ID_SEC_ADMIN_PW, blob,
			  sizeof(blob), &blob_len) != 0) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_INTERNAL);
	}
	if (blob_len != MCP_PW_BLOB_LEN) {
		/*
		 * No credential provisioned. Refusing rather than falling open
		 * is the whole point: a box with auth required and no password
		 * is locked, not wide open.
		 *
		 * The answer is deliberately indistinguishable from a wrong
		 * password, and carries the same penalty. A distinct status here
		 * told an unauthenticated peer whether the box had ever been
		 * commissioned, and — because it returned before auth_penalise()
		 * — did so as fast as the link allowed. Provisioning state is
		 * reported to the local shell (`sts sec`) and to an
		 * authenticated session, not to the world.
		 */
		c->stats.auth_fail++;
		auth_penalise(c);
		mcp__log(c, (uint8_t)LOGR_WARN,
			 "auth: rejected (no credential provisioned)");
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_AUTH);
	}

	if (c->w.crypto->hmac_sha256(c->w.crypto->ctx, blob, MCP_PW_SALT_LEN,
				     f->payload, f->len, mac) != 0) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_INTERNAL);
	}
	if (!ct_eq(mac, &blob[MCP_PW_SALT_LEN], MCP_PW_MAC_LEN)) {
		c->stats.auth_fail++;
		auth_penalise(c);
		mcp__log(c, (uint8_t)LOGR_WARN, "auth: rejected");
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_AUTH);
	}

	c->authed = true;
	c->last_activity_ms = c->now_ms;
	c->auth_fails = 0U;
	c->auth_lock_until_ms = 0U;
	c->stats.auth_ok++;
	mcp__log(c, (uint8_t)LOGR_NOTICE, "auth: session granted");

	p[0] = (uint8_t)MCP_OK;
	p[1] = 1U;
	bytes_put_le16(&p[2], (uint16_t)session_seconds(c));
	return mcp__reply(c, f->cmd, f->seq, 4U);
}

static int h_reboot(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t mode;

	if (f->len != 1U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	mode = f->payload[0];
	if (mode > MCP_REBOOT_HALT) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	if ((c->w.img == NULL) || (c->w.img->reboot == NULL)) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_NOTSUP);
	}

	c->reboot_pending = true;
	c->reboot_mode = mode;
	c->reboot_at_ms = c->now_ms + MCP_REBOOT_DELAY_MS;
	mcp__log(c, (uint8_t)LOGR_NOTICE, "reboot requested");

	return mcp__reply_status(c, f->cmd, f->seq, (uint8_t)MCP_OK);
}

/* ------------------------------------------------------------------------- */
/* Handlers — config                                                         */
/* ------------------------------------------------------------------------- */

/* cfg errno -> MCP status. Everything an operator can provoke is an argument
 * problem; anything else is ours. */
static uint8_t cfg_err(int rc)
{
	switch (rc) {
	case 0:
		return (uint8_t)MCP_OK;
	case -ENOENT:
	case -EPROTO:
	case -ERANGE:
	case -EINVAL:
		return (uint8_t)MCP_ERR_ARG;
	case -ENOTSUP:
		return (uint8_t)MCP_ERR_NOTSUP;
	case -EILSEQ:
		return (uint8_t)MCP_ERR_CRC;
	case -EBADMSG:
		return (uint8_t)MCP_ERR_ARG;
	case -EIO:
		return (uint8_t)MCP_ERR_INTERNAL;
	default:
		return (uint8_t)MCP_ERR_STATE;
	}
}

static int h_cfg_list(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t *p = mcp__rsp_buf(c);
	size_t o = 4U; /* status, next_id(2), n */
	uint16_t start;
	uint8_t want;
	uint8_t n = 0U;
	int idx;
	uint16_t next = 0U;

	if (f->len != 3U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	start = bytes_get_le16(f->payload);
	want = f->payload[2];
	if (want == 0U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}

	idx = cfg_key_lower_bound(start);
	while (idx >= 0) {
		const cfg_key_t *k = cfg_key_at((size_t)idx);
		size_t nl;

		if (k == NULL) {
			break;
		}
		if (n >= want) {
			next = k->id;
			break;
		}
		nl = strlen(k->name);
		if (nl > CFG_NAME_MAX) {
			nl = CFG_NAME_MAX;
		}
		if ((o + 5U + nl) > MCP_MAX_PAYLOAD) {
			next = k->id;
			break;
		}

		bytes_put_le16(&p[o], k->id);
		o += 2U;
		p[o++] = k->type;
		p[o++] = k->flags;
		p[o++] = (uint8_t)nl;
		memcpy(&p[o], k->name, nl);
		o += nl;
		n++;
		idx++;
		if ((size_t)idx >= cfg_key_count()) {
			break;
		}
	}

	p[0] = (uint8_t)MCP_OK;
	bytes_put_le16(&p[1], next);
	p[3] = n;
	return mcp__reply(c, f->cmd, f->seq, (uint16_t)o);
}

static int h_cfg_get(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t *p = mcp__rsp_buf(c);
	const cfg_key_t *k;
	cfg_val_t v;
	uint16_t id;
	uint8_t flags = 0U;
	int n;
	int rc;

	if ((f->len != 2U) && (f->len != 3U)) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	id = bytes_get_le16(f->payload);
	if (f->len == 3U) {
		flags = f->payload[2];
	}

	k = cfg_key_find(id);
	if (k == NULL) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	if ((k->flags & CFG_F_NOEXPORT) != 0U) {
		/* Write-only over the wire (the admin credential): never read
		 * back, regardless of session — see cfg_schema.h. */
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_AUTH);
	}
	if (((k->flags & CFG_F_SECRET) != 0U) && !c->authed) {
		/* A real authenticated session, not merely session_ok(): a box
		 * with auth disabled must still not leak secrets. */
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_AUTH);
	}

	rc = ((flags & 0x01U) != 0U) ? cfg_get_effective(c->w.cfg, id, &v)
				     : cfg_get(c->w.cfg, id, &v);
	if (rc != 0) {
		return mcp__reply_status(c, f->cmd, f->seq, cfg_err(rc));
	}

	n = cfg_val_encode(&v, &p[7], MCP_MAX_PAYLOAD - 7U);
	if (n < 0) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_INTERNAL);
	}

	p[0] = (uint8_t)MCP_OK;
	bytes_put_le16(&p[1], id);
	p[3] = k->type;
	p[4] = k->flags;
	bytes_put_le16(&p[5], (uint16_t)n);
	return mcp__reply(c, f->cmd, f->seq, (uint16_t)(7 + n));
}

static int h_cfg_set(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t *p = mcp__rsp_buf(c);
	cfg_val_t v;
	uint16_t id;
	uint16_t vlen;
	uint8_t type;
	int rc;

	if (f->len < 5U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	id = bytes_get_le16(f->payload);
	type = f->payload[2];
	vlen = bytes_get_le16(&f->payload[3]);
	if ((size_t)vlen != ((size_t)f->len - 5U)) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}

	rc = cfg_val_decode(&v, type, &f->payload[5], vlen);
	if (rc == 0) {
		rc = cfg_set(c->w.cfg, id, &v);
	}
	if (rc != 0) {
		return mcp__reply_status(c, f->cmd, f->seq, cfg_err(rc));
	}

	p[0] = (uint8_t)MCP_OK;
	bytes_put_le16(&p[1], id);
	return mcp__reply(c, f->cmd, f->seq, 3U);
}

static int h_cfg_commit(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t *p = mcp__rsp_buf(c);
	cfg_commit_res_t res;
	int rc;

	if (f->len != 0U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}

	rc = cfg_commit_through_glue(c, &res, true);
	/*
	 * -EIO means the staged set validated and was applied to the live tree
	 * but some keys did not reach non-volatile storage. Reporting that as a
	 * bare error would tell the operator "nothing changed" and invite a
	 * reboot that silently reverts the live change. Instead answer OK and
	 * surface persist_errors so the tool can say "live now, N not saved".
	 * Only a validation/cross-field failure (nothing applied) is an error.
	 */
	if ((rc != 0) && (rc != -EIO)) {
		mcp__log(c, (uint8_t)LOGR_ERR, "cfg: commit rejected");
		return mcp__reply_status(c, f->cmd, f->seq, cfg_err(rc));
	}

	if (res.persist_errors != 0U) {
		mcp__log(c, (uint8_t)LOGR_ERR,
			 "cfg: applied to RAM, persist failed");
	} else {
		mcp__log(c, (uint8_t)LOGR_NOTICE, "cfg: committed");
	}
	p[0] = (uint8_t)MCP_OK;
	bytes_put_le16(&p[1], res.applied);
	bytes_put_le16(&p[3], res.reboot_keys);
	bytes_put_le32(&p[5], res.reboot_groups);
	bytes_put_le16(&p[9], res.persist_errors);
	return mcp__reply(c, f->cmd, f->seq, 11U);
}

static int h_cfg_revert(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t *p = mcp__rsp_buf(c);
	uint16_t dropped;

	if (f->len != 0U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	dropped = cfg_staged_count(c->w.cfg);
	(void)cfg_revert(c->w.cfg);

	p[0] = (uint8_t)MCP_OK;
	bytes_put_le16(&p[1], dropped);
	return mcp__reply(c, f->cmd, f->seq, 3U);
}

static int h_cfg_export(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t *p = mcp__rsp_buf(c);
	uint32_t off;
	uint32_t start;
	uint8_t flags;
	bool secrets;
	size_t n = 0U;
	int rc;

	if (f->len != 5U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	off = bytes_get_le32(f->payload);
	flags = f->payload[4];
	secrets = (flags & 0x01U) != 0U;

	if (secrets && !c->authed) {
		/* A real session, not session_ok(): a box with auth disabled
		 * must not hand out secrets. */
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_AUTH);
	}

	if (off == 0U) {
		rc = cfg_export_begin(c->w.cfg, &c->exp, secrets);
		if (rc != 0) {
			return mcp__reply_status(c, f->cmd, f->seq,
						 cfg_err(rc));
		}
		c->exp_active = true;
		c->exp_prev_valid = false;
	} else if (c->exp_active && (off == c->exp.offset)) {
		/* Normal forward advance. */
	} else if (c->exp_prev_valid && (off == c->exp_prev.offset)) {
		/*
		 * Retransmit of the last emitted chunk after a lost response:
		 * rewind the cursor to the start of that chunk and re-emit the
		 * identical bytes (the CRC state is part of the snapshot).
		 *
		 * Deliberately NOT gated on exp_active. The final chunk clears
		 * exp_active, and the final chunk is exactly the one whose
		 * response the tool is most likely to lose; refusing its repeat
		 * with ERR_OFFSET aborted an export that had in fact succeeded.
		 * CFG_IMPORT has always handled its own final chunk this way
		 * (imp_done / imp_done_off) — this is the same rule.
		 */
		c->exp = c->exp_prev;
	} else {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_OFFSET);
	}

	/* Snapshot the cursor at the start of the chunk we are about to emit so
	 * a repeat of this offset can rewind here. */
	c->exp_prev = c->exp;
	c->exp_prev_valid = true;

	start = c->exp.offset;
	rc = cfg_export_read(c->w.cfg, &c->exp, &p[6], MCP_CFG_EXPORT_CHUNK, &n);
	if (rc < 0) {
		c->exp_active = false;
		return mcp__reply_status(c, f->cmd, f->seq, cfg_err(rc));
	}
	if (rc == 1) {
		c->exp_active = false;
	}

	p[0] = (uint8_t)MCP_OK;
	bytes_put_le32(&p[1], start);
	p[5] = (rc == 1) ? 0U : 1U;
	return mcp__reply(c, f->cmd, f->seq, (uint16_t)(6U + n));
}

static int h_cfg_import(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t *p = mcp__rsp_buf(c);
	cfg_commit_res_t res;
	uint32_t off;
	uint8_t flags;
	bool final_chunk;
	size_t dlen;
	int rc;

	memset(&res, 0, sizeof(res));

	if (f->len < 5U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	off = bytes_get_le32(f->payload);
	flags = f->payload[4];
	final_chunk = (flags & 0x02U) != 0U;
	dlen = (size_t)f->len - 5U;

	if (off == 0U) {
		rc = cfg_import_begin(c->w.cfg, &c->imp,
				      (flags & 0x01U) != 0U);
		if (rc != 0) {
			return mcp__reply_status(c, f->cmd, f->seq,
						 cfg_err(rc));
		}
		c->imp_active = true;
		c->imp_prev_valid = false;
		c->imp_done = false;
	} else if (c->imp_done && (off == c->imp_done_off)) {
		/* Retransmit of the committed final chunk after a lost
		 * response: re-emit the cached result, do not re-commit. */
		p[0] = (uint8_t)MCP_OK;
		bytes_put_le32(&p[1], c->imp.offset);
		p[5] = 1U;
		bytes_put_le16(&p[6], c->imp_res.applied);
		bytes_put_le32(&p[8], c->imp_res.reboot_groups);
		bytes_put_le16(&p[12], c->imp_res.persist_errors);
		return mcp__reply(c, f->cmd, f->seq, 14U);
	} else if (c->imp_active && (off == c->imp.offset)) {
		/* Normal forward advance. */
	} else if (c->imp_active && c->imp_prev_valid &&
		   (off == c->imp_prev_off)) {
		/* Retransmit of the previous chunk after a lost response: we
		 * already consumed those bytes, so absorb the repeat and just
		 * re-report the current frontier — re-feeding would double the
		 * parse state. */
		p[0] = (uint8_t)MCP_OK;
		bytes_put_le32(&p[1], c->imp.offset);
		p[5] = 0U;
		bytes_put_le16(&p[6], 0U);
		bytes_put_le32(&p[8], 0U);
		bytes_put_le16(&p[12], 0U);
		return mcp__reply(c, f->cmd, f->seq, 14U);
	} else {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_OFFSET);
	}

	c->imp_prev_off = off;
	c->imp_prev_valid = true;

	rc = cfg_import_feed(c->w.cfg, &c->imp, &f->payload[5], dlen, NULL);
	if (rc < 0) {
		c->imp_active = false;
		mcp__log(c, (uint8_t)LOGR_ERR, "cfg: import rejected");
		return mcp__reply_status(c, f->cmd, f->seq, cfg_err(rc));
	}

	if (final_chunk) {
		if (rc != 1) {
			/* The operator declared the last chunk but the stream
			 * is short of its own record count or CRC trailer. */
			c->imp_active = false;
			(void)cfg_revert(c->w.cfg);
			return mcp__reply_status(c, f->cmd, f->seq,
						 (uint8_t)MCP_ERR_STATE);
		}
		/*
		 * cfg_import_finish() is a completeness check plus cfg_commit(),
		 * and the feed above already answered the completeness half with
		 * rc == 1 (the short-stream case returned MCP_ERR_STATE). Commit
		 * through the same glue path CFG_COMMIT uses so an imported tree
		 * runs exactly the same appliers as an individually-set key.
		 */
		rc = cfg_commit_through_glue(c, &res, true);
		c->imp_active = false;
		/* As in CFG_COMMIT, -EIO means live-but-not-persisted; report
		 * it through persist_errors, not as a failure. */
		if ((rc != 0) && (rc != -EIO)) {
			return mcp__reply_status(c, f->cmd, f->seq,
						 cfg_err(rc));
		}
		c->imp_done = true;
		c->imp_done_off = off;
		c->imp_res = res;
		mcp__log(c, (uint8_t)LOGR_NOTICE, "cfg: imported");
		rc = 1;
	}

	p[0] = (uint8_t)MCP_OK;
	bytes_put_le32(&p[1], c->imp.offset);
	p[5] = (rc == 1) ? 1U : 0U;
	bytes_put_le16(&p[6], res.applied);
	bytes_put_le32(&p[8], res.reboot_groups);
	bytes_put_le16(&p[12], res.persist_errors);
	return mcp__reply(c, f->cmd, f->seq, 14U);
}

static int h_factory_reset(mcp_ctx_t *c, const mcp_frame_t *f)
{
	int rc;

	if (f->len != 4U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	if (bytes_get_le32(f->payload) != MCP_FACTORY_MAGIC) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}

	/* Through the glue, so every group's appliers run (mcp.h "Config
	 * commits"); the section is held here, so it is dropped for the call. */
	rc = cfg_factory_through_glue(c, true);
	c->exp_active = false;
	c->imp_active = false;
	/* The credential this session authenticated against no longer
	 * exists — the session must not outlive it. */
	c->authed = false;
	mcp__log(c, (uint8_t)LOGR_ALERT, "cfg: factory reset");

	return mcp__reply_status(c, f->cmd, f->seq, cfg_err(rc));
}

/* ------------------------------------------------------------------------- */
/* Handlers — status, telemetry, diagnostics                                 */
/* ------------------------------------------------------------------------- */

/* Write group @p g's packed struct at @p p. Returns the length or a negative
 * errno straight from the callback. */
static int status_pack(mcp_ctx_t *c, uint8_t g, uint8_t *p, size_t cap)
{
	int rc = c->w.status_cb(c->w.status_user, g, p, cap);

	if (rc == 0) {
		/* The version byte is mandatory: an empty group struct would be
		 * indistinguishable from a truncated one on the wire. */
		return -EPROTO;
	}
	if (rc > (int)cap) {
		/* A callback that claims to have written past its buffer would
		 * make us frame bytes it never produced. Refuse rather than
		 * leak whatever follows the scratch. */
		return -EPROTO;
	}
	return rc;
}

static int h_status_get(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t *p = mcp__rsp_buf(c);
	uint8_t g;
	int n;

	if (f->len != 1U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	g = f->payload[0];
	if (g >= (uint8_t)MCP_GRP_COUNT) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	if (c->w.status_cb == NULL) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_NOTSUP);
	}

	n = status_pack(c, g, &p[2], MCP_MAX_PAYLOAD - 2U);
	if (n < 0) {
		return mcp__reply_status(c, f->cmd, f->seq, mcp__port_err(n));
	}

	p[0] = (uint8_t)MCP_OK;
	p[1] = g;
	return mcp__reply(c, f->cmd, f->seq, (uint16_t)(2 + n));
}

static int h_telem_sub(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t *p = mcp__rsp_buf(c);
	uint8_t mask;
	uint8_t rate;

	if (f->len != 2U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	if (c->w.status_cb == NULL) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_NOTSUP);
	}
	mask = f->payload[0];
	rate = f->payload[1];
	if ((mask == 0U) || (mask > MCP_GRP_MASK_ALL)) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	if ((rate == 0U) || (rate > 4U)) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}

	c->telem_mask = mask;
	c->telem_rate = rate;
	c->telem_next_ms = c->now_ms + (1000U / rate);

	p[0] = (uint8_t)MCP_OK;
	p[1] = mask;
	p[2] = rate;
	return mcp__reply(c, f->cmd, f->seq, 3U);
}

static int h_telem_unsub(mcp_ctx_t *c, const mcp_frame_t *f)
{
	if (f->len != 0U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	c->telem_mask = 0U;
	c->telem_rate = 0U;
	return mcp__reply_status(c, f->cmd, f->seq, (uint8_t)MCP_OK);
}

static int h_diag(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t *p = mcp__rsp_buf(c);
	uint8_t sub;
	int n;

	if (f->len != 1U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	sub = f->payload[0];
	if (sub >= MCP_DIAG_SUB_COUNT) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}

	if (sub == 0U) {
		/* The health snapshot is the summary status group; there is one
		 * definition of device health (spec §10.5) and DIAG does not
		 * get a second one. */
		if (c->w.status_cb == NULL) {
			return mcp__reply_status(c, f->cmd, f->seq,
						 (uint8_t)MCP_ERR_NOTSUP);
		}
		n = status_pack(c, (uint8_t)MCP_GRP_SUMMARY, &p[2],
				MCP_MAX_PAYLOAD - 2U);
	} else {
		if (c->w.diag_cb == NULL) {
			return mcp__reply_status(c, f->cmd, f->seq,
						 (uint8_t)MCP_ERR_NOTSUP);
		}
		n = c->w.diag_cb(c->w.diag_user, sub, &p[2],
				 MCP_MAX_PAYLOAD - 2U);
		if (n > (int)(MCP_MAX_PAYLOAD - 2U)) {
			/* Same guard as status_pack: never frame past the cap. */
			n = -EPROTO;
		}
	}

	if (n < 0) {
		return mcp__reply_status(c, f->cmd, f->seq, mcp__port_err(n));
	}

	p[0] = (uint8_t)MCP_OK;
	p[1] = sub;
	return mcp__reply(c, f->cmd, f->seq, (uint16_t)(2 + n));
}

/* ------------------------------------------------------------------------- */
/* Handlers — logging                                                        */
/* ------------------------------------------------------------------------- */

/* Encoded size of one log record on the wire. */
#define LOG_REC_FIXED 15U

/*
 * Pack {u32 next, u32 gap, u8 n, records...} at @p p.
 *
 * Records are pulled one at a time so the cursor never advances past a record
 * that did not fit — a dropped record here would be indistinguishable from one
 * the ring evicted, and the whole point of the cursor is that loss is
 * accounted for.
 */
static size_t log_pack(mcp_ctx_t *c, uint32_t cursor, const logr_filter_t *filt,
		       uint16_t max, uint8_t *p, size_t cap)
{
	uint32_t next = cursor;
	uint32_t gap = 0U;
	size_t o = 9U;
	uint16_t n = 0U;

	while (n < max) {
		logr_rec_t rec;
		uint32_t prev = next;
		uint32_t g = 0U;
		uint16_t got = 0U;

		if (logr_tail(c->w.log, next, filt, &rec, 1U, &got, &next,
			      &g) != 0) {
			next = prev;
			break;
		}
		if (n == 0U) {
			gap = g;
		}
		if (got == 0U) {
			break;
		}
		if ((o + LOG_REC_FIXED + rec.len) > cap) {
			next = prev;
			break;
		}

		bytes_put_le32(&p[o], rec.seq);
		o += 4U;
		bytes_put_le64(&p[o], rec.mono_ms);
		o += 8U;
		p[o++] = rec.level;
		p[o++] = rec.subsys;
		p[o++] = rec.len;
		if (rec.len != 0U) {
			memcpy(&p[o], rec.msg, rec.len);
			o += rec.len;
		}
		n++;
	}

	bytes_put_le32(&p[0], next);
	bytes_put_le32(&p[4], gap);
	p[8] = (uint8_t)n;
	return o;
}

static int h_log_tail(mcp_ctx_t *c, const mcp_frame_t *f)
{
	uint8_t *p = mcp__rsp_buf(c);
	logr_filter_t filt;
	uint32_t cursor;
	uint8_t follow;
	uint8_t max;
	size_t o;

	if (f->len != 9U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	cursor = bytes_get_le32(f->payload);
	follow = f->payload[4];
	max = f->payload[5];
	filt.max_level = f->payload[6];
	filt.sub_mask = bytes_get_le16(&f->payload[7]);

	if ((filt.max_level >= LOGR_LEVEL_COUNT) || (follow > 1U)) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}

	o = log_pack(c, cursor, &filt, max, &p[1], MCP_MAX_PAYLOAD - 1U);

	c->log_follow = (follow != 0U);
	if (c->log_follow) {
		c->log_filter = filt;
		c->log_cursor = bytes_get_le32(&p[1]);
	}

	p[0] = (uint8_t)MCP_OK;
	return mcp__reply(c, f->cmd, f->seq, (uint16_t)(1U + o));
}

static int h_log_level(mcp_ctx_t *c, const mcp_frame_t *f)
{
	int rc;

	if (f->len != 2U) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_ARG);
	}
	rc = logr_level_set(c->w.log, f->payload[0], f->payload[1]);
	return mcp__reply_status(c, f->cmd, f->seq,
				 (rc == 0) ? (uint8_t)MCP_OK
					   : (uint8_t)MCP_ERR_ARG);
}

/* ------------------------------------------------------------------------- */
/* Dispatch                                                                  */
/* ------------------------------------------------------------------------- */

/* A command whose backing module was not wired answers NOTSUP once, here,
 * rather than each handler re-checking. */
static bool wiring_missing(const mcp_ctx_t *c, uint8_t cmd)
{
	switch (cmd) {
	case MCP_CMD_CFG_LIST:
	case MCP_CMD_CFG_GET:
	case MCP_CMD_CFG_SET:
	case MCP_CMD_CFG_COMMIT:
	case MCP_CMD_CFG_REVERT:
	case MCP_CMD_CFG_EXPORT:
	case MCP_CMD_CFG_IMPORT:
	case MCP_CMD_FACTORY_RESET:
		return c->w.cfg == NULL;
	case MCP_CMD_LOG_TAIL:
	case MCP_CMD_LOG_LEVEL:
		return c->w.log == NULL;
	default:
		return false;
	}
}

/*
 * Commands whose handler reaches into the cfg_ctx_t. handle_frame() runs these
 * inside the glue's cfg critical section (mcp.h "Config locking"); everything
 * else runs outside it, so a FW_DATA flash write or a telemetry encode never
 * blocks the shell or the UI on the config mutex.
 *
 * AUTH is on the list because it reads the stored credential and the session
 * policy. CFG_LIST only walks the static schema table, but it is listed anyway:
 * "every CFG_* command holds the section" is a rule that survives someone later
 * making CFG_LIST report live values.
 */
static bool cmd_touches_cfg(uint8_t cmd)
{
	switch (cmd) {
	case MCP_CMD_AUTH:
	case MCP_CMD_CFG_LIST:
	case MCP_CMD_CFG_GET:
	case MCP_CMD_CFG_SET:
	case MCP_CMD_CFG_COMMIT:
	case MCP_CMD_CFG_REVERT:
	case MCP_CMD_CFG_EXPORT:
	case MCP_CMD_CFG_IMPORT:
	case MCP_CMD_FACTORY_RESET:
		return true;
	default:
		return false;
	}
}

static int dispatch(mcp_ctx_t *c, const mcp_frame_t *f)
{
	if (wiring_missing(c, f->cmd)) {
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_NOTSUP);
	}

	switch (f->cmd) {
	case MCP_CMD_HELLO:
		return h_hello(c, f);
	case MCP_CMD_REBOOT:
		return h_reboot(c, f);
	case MCP_CMD_AUTH:
		return h_auth(c, f);

	case MCP_CMD_CFG_LIST:
		return h_cfg_list(c, f);
	case MCP_CMD_CFG_GET:
		return h_cfg_get(c, f);
	case MCP_CMD_CFG_SET:
		return h_cfg_set(c, f);
	case MCP_CMD_CFG_COMMIT:
		return h_cfg_commit(c, f);
	case MCP_CMD_CFG_REVERT:
		return h_cfg_revert(c, f);
	case MCP_CMD_CFG_EXPORT:
		return h_cfg_export(c, f);
	case MCP_CMD_CFG_IMPORT:
		return h_cfg_import(c, f);
	case MCP_CMD_FACTORY_RESET:
		return h_factory_reset(c, f);

	case MCP_CMD_STATUS_GET:
		return h_status_get(c, f);
	case MCP_CMD_TELEM_SUB:
		return h_telem_sub(c, f);
	case MCP_CMD_TELEM_UNSUB:
		return h_telem_unsub(c, f);

	case MCP_CMD_LOG_TAIL:
		return h_log_tail(c, f);
	case MCP_CMD_LOG_LEVEL:
		return h_log_level(c, f);

	case MCP_CMD_FW_INFO:
	case MCP_CMD_FW_BEGIN:
	case MCP_CMD_FW_DATA:
	case MCP_CMD_FW_END:
	case MCP_CMD_FW_CONFIRM:
	case MCP_CMD_FW_REVERT:
		return mcp_dfu__handle(c, f);

	case MCP_CMD_DIAG:
		return h_diag(c, f);

	default:
		return mcp__reply_status(c, f->cmd, f->seq,
					 (uint8_t)MCP_ERR_NOTSUP);
	}
}

static void handle_frame(mcp_ctx_t *c, const uint8_t *buf, size_t n)
{
	mcp_frame_t f;
	int rc = mcp_wire_parse(buf, n, &f);

	c->stats.rx_frames++;

	if (rc == -EILSEQ) {
		c->stats.rx_bad_crc++;
		return;
	}
	if (rc != 0) {
		c->stats.rx_bad_hdr++;
		return;
	}
	if (f.ver != MCP_VER) {
		/* The CRC held, so the header is intelligible even though the
		 * body may not be: answer rather than leave the tool guessing. */
		c->stats.rx_bad_ver++;
		(void)mcp__reply_status(c, f.cmd, f.seq,
					(uint8_t)MCP_ERR_NOTSUP);
		return;
	}
	if (f.type != (uint8_t)MCP_T_REQ) {
		c->stats.rx_ignored++;
		return;
	}

	if (cmd_mutating(f.cmd)) {
		bool ok;

		/* session_ok() reads the auth policy out of the shared tree, so
		 * it needs the section too — briefly, and never nested with the
		 * per-handler scope below. */
		cfg_enter(c);
		ok = session_ok(c);
		cfg_leave(c);

		if (!ok) {
			/* Refused before the activity timer is touched: an
			 * unauthenticated peer spamming mutating commands must
			 * not keep a lapsed session's idle timer alive. */
			c->stats.auth_denied++;
			(void)mcp__reply_status(c, f.cmd, f.seq,
						(uint8_t)MCP_ERR_AUTH);
			return;
		}
	}

	c->last_activity_ms = c->now_ms;

	if (cmd_touches_cfg(f.cmd)) {
		cfg_enter(c);
		(void)dispatch(c, &f);
		cfg_leave(c);
	} else {
		(void)dispatch(c, &f);
	}
}

/* ------------------------------------------------------------------------- */
/* Event emission                                                            */
/* ------------------------------------------------------------------------- */

static void emit_telemetry(mcp_ctx_t *c)
{
	uint32_t interval;
	uint8_t g;

	if ((c->telem_mask == 0U) || (c->telem_rate == 0U) ||
	    (c->w.status_cb == NULL)) {
		return;
	}
	if (c->now_ms < c->telem_next_ms) {
		return;
	}

	interval = 1000U / c->telem_rate;
	c->telem_next_ms += interval;
	if (c->telem_next_ms <= c->now_ms) {
		/* Fell far enough behind that catching up would burst; resync
		 * to the current instant instead of emitting a backlog. */
		c->telem_next_ms = c->now_ms + interval;
	}

	for (g = 0U; g < (uint8_t)MCP_GRP_COUNT; g++) {
		uint8_t *p = mcp__rsp_buf(c);
		int n;

		if ((c->telem_mask & (uint8_t)(1U << g)) == 0U) {
			continue;
		}
		n = status_pack(c, g, &p[1], MCP_MAX_PAYLOAD - 1U);
		if (n < 0) {
			c->stats.evt_dropped++;
			continue;
		}
		p[0] = g;
		emit_evt(c, (uint8_t)MCP_CMD_TELEM_SUB,
			 (uint8_t)MCP_EVT_SUB_TELEM, (uint16_t)(1 + n));
	}
}

static void emit_log_events(mcp_ctx_t *c)
{
	uint8_t *p = mcp__rsp_buf(c);
	size_t o;

	if (!c->log_follow || (c->w.log == NULL)) {
		return;
	}

	o = log_pack(c, c->log_cursor, &c->log_filter, MCP_LOG_EVT_RECS, p,
		     MCP_MAX_PAYLOAD);
	if ((p[8] == 0U) && (bytes_get_le32(&p[4]) == 0U)) {
		/* Nothing matched and nothing was lost; keep the advanced
		 * cursor so a torrent of filtered records is not rescanned
		 * every tick, and stay quiet. */
		c->log_cursor = bytes_get_le32(&p[0]);
		return;
	}

	c->log_cursor = bytes_get_le32(&p[0]);
	emit_evt(c, (uint8_t)MCP_CMD_LOG_TAIL, (uint8_t)MCP_EVT_SUB_LOG,
		 (uint16_t)o);
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

int mcp_init(mcp_ctx_t *c, const mcp_wiring_t *w)
{
	if ((c == NULL) || (w == NULL) || (w->tx == NULL)) {
		return -EINVAL;
	}
	/* Half a critical section is worse than none: it would look guarded and
	 * silently deadlock or never release. Reject the asymmetric wiring. */
	if ((w->cfg_lock == NULL) != (w->cfg_unlock == NULL)) {
		return -EINVAL;
	}

	memset(c, 0, sizeof(*c));
	c->w = *w;

	(void)cobs_dec_init(&c->dec, c->rx, sizeof(c->rx));

	c->caps = 0U;
	if (w->img != NULL) {
		c->caps |= MCP_CAP_DFU;
	}
	if (w->status_cb != NULL) {
		c->caps |= MCP_CAP_TELEM;
	}
	if (w->log != NULL) {
		c->caps |= MCP_CAP_LOGS;
	}
	if (w->cfg != NULL) {
		c->caps |= MCP_CAP_CFG;
	}
	if ((w->cfg != NULL) && (w->crypto != NULL) &&
	    (w->crypto->hmac_sha256 != NULL)) {
		c->caps |= MCP_CAP_AUTH;
	}
	if (w->diag_cb != NULL) {
		c->caps |= MCP_CAP_DIAG;
	}

	mcp_dfu__reset(c);
	c->dfu.erase_gran = (w->dfu_erase_gran != 0U) ? w->dfu_erase_gran
						      : MCP_DFU_ERASE_GRAN;
	c->dfu.write_block = (w->dfu_write_block != 0U) ? w->dfu_write_block
							: MCP_DFU_WRITE_BLOCK;
	return 0;
}

void mcp_reset_session(mcp_ctx_t *c)
{
	if (c == NULL) {
		return;
	}

	c->authed = false;
	c->telem_mask = 0U;
	c->telem_rate = 0U;
	c->log_follow = false;
	c->log_cursor = 0U;
	c->exp_active = false;
	c->exp_prev_valid = false;
	c->imp_active = false;
	c->imp_prev_valid = false;
	c->imp_done = false;
	c->tx_pending = false;
	c->enc_len = 0U;
	cobs_dec_reset(&c->dec);

	if (c->w.cfg != NULL) {
		cfg_enter(c);
		(void)cfg_revert(c->w.cfg);
		cfg_leave(c);
	}
	/* The DFU session deliberately survives: a link drop in the middle of
	 * an upload is exactly the case FW_BEGIN resume exists for. */
}

int mcp_input(mcp_ctx_t *c, const uint8_t *buf, size_t len, size_t *consumed)
{
	size_t i = 0U;

	if ((c == NULL) || ((buf == NULL) && (len != 0U))) {
		if (consumed != NULL) {
			*consumed = 0U;
		}
		return -EINVAL;
	}

	if ((c->w.clock != NULL) && (c->w.clock->mono_ms != NULL)) {
		c->now_ms = c->w.clock->mono_ms(c->w.clock->ctx);
	}

	while (i < len) {
		int rc;

		if (c->tx_pending && (mcp_poll_tx(c) == 1)) {
			break;
		}

		rc = cobs_dec_byte(&c->dec, buf[i]);
		i++;

		if (rc == 1) {
			handle_frame(c, cobs_dec_buf(&c->dec),
				     cobs_dec_len(&c->dec));
		} else if (rc < 0) {
			c->stats.rx_bad_cobs++;
		} else {
			/* more input needed */
		}
	}

	if (consumed != NULL) {
		*consumed = i;
	}
	return (i < len) ? -EAGAIN : 0;
}

int mcp_tick(mcp_ctx_t *c, uint64_t now_ms)
{
	if (c == NULL) {
		return -EINVAL;
	}

	c->now_ms = now_ms;
	(void)mcp_poll_tx(c);

	if (c->authed) {
		uint64_t limit;

		/* session_seconds() reads the shared tree. */
		cfg_enter(c);
		limit = (uint64_t)session_seconds(c) * 1000ULL;
		cfg_leave(c);

		if ((now_ms - c->last_activity_ms) >= limit) {
			c->authed = false;
			c->stats.session_expired++;
			mcp__log(c, (uint8_t)LOGR_NOTICE,
				 "auth: session expired");
		}
	}

	mcp_dfu__tick(c, now_ms);

	if (c->reboot_pending && (now_ms >= c->reboot_at_ms)) {
		c->reboot_pending = false;
		if ((c->w.img != NULL) && (c->w.img->reboot != NULL)) {
			c->w.img->reboot(c->w.img->ctx, (int)c->reboot_mode);
		}
		return 0;
	}

	if (c->tx_pending) {
		return 0; /* an unsent response owns the buffers */
	}

	emit_telemetry(c);
	emit_log_events(c);
	return 0;
}

bool mcp_authenticated(const mcp_ctx_t *c)
{
	return (c != NULL) && c->authed;
}

const mcp_stats_t *mcp_stats(const mcp_ctx_t *c)
{
	return (c != NULL) ? &c->stats : NULL;
}

const mcp_dfu_t *mcp_dfu_status(const mcp_ctx_t *c)
{
	return (c != NULL) ? &c->dfu : NULL;
}
