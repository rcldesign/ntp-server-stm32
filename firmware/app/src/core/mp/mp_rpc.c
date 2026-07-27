/*
 * STS1000 "Meridian" — core/mp: JSON-RPC 2.0 control plane (FMT §3.2) plus the
 * module's lifecycle, mode machine, tick and stream pump.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Batching. JSON-RPC 2.0 §6 permits an array of requests. This device rejects
 * one with MP_E_INVALID_REQ: the reply buffer is a fixed MP_REPLY_MAX and a
 * batch has no bound on its response size, so accepting batches would mean
 * either an unbounded buffer or a partial reply that is not valid JSON-RPC.
 * A maintenance tool loses nothing — it is already sequencing guarded actions.
 */

#include <errno.h>
#include <string.h>

#include "mp/mp_internal.h"
#include "util/crc.h"

/* ==================================================================== util */

uint32_t mp_now(const mp_ctx_t *c)
{
	if ((c == NULL) || (c->w.mono_ms == NULL)) {
		return 0U;
	}
	return c->w.mono_ms(c->w.clock_user);
}

const char *mp_err_msg(int code)
{
	switch (code) {
	case 0:
		return "ok";
	case MP_E_PARSE:
		return "parse error";
	case MP_E_INVALID_REQ:
		return "invalid request";
	case MP_E_NO_METHOD:
		return "method not found";
	case MP_E_BAD_PARAMS:
		return "invalid params";
	case MP_E_INTERNAL:
		return "internal error";
	case MP_E_NO_SESSION:
		return "no valid session";
	case MP_E_GUARD:
		return "confirmation required";
	case MP_E_INTERLOCK:
		return "interlock refused";
	case MP_E_RANGE:
		return "value out of range";
	case MP_E_BUSY:
		return "busy";
	case MP_E_NOTSUP:
		return "not supported";
	case MP_E_STATE:
		return "not legal in this state";
	case MP_E_HOLD:
		return "hold not satisfied";
	case MP_E_VETO:
		return "vetoed by firmware";
	case MP_E_IO:
		return "device error";
	default:
		return "error";
	}
}

void mp_fail(mp_ctx_t *c, int code, const char *what)
{
	if (c == NULL) {
		return;
	}
	c->err_code = code;
	c->err_msg = mp_err_msg(code);
	c->err_data[0] = '\0';
	if (what != NULL) {
		size_t n = strlen(what);

		if (n > (MP_ERR_DATA_MAX - 1U)) {
			n = MP_ERR_DATA_MAX - 1U;
		}
		(void)memcpy(c->err_data, what, n);
		c->err_data[n] = '\0';
	}
}

int mp_map_errno(int rc)
{
	switch (rc) {
	case 0:
		return 0;
	case -EINVAL:
		return MP_E_BAD_PARAMS;
	case -ENOENT:
		return MP_E_BAD_PARAMS;
	case -ERANGE:
		return MP_E_RANGE;
	case -EPERM:
		return MP_E_INTERLOCK;
	case -EACCES:
		return MP_E_VETO;
	case -EBUSY:
		return MP_E_BUSY;
	case -ENOTSUP:
		return MP_E_NOTSUP;
	case -ENOLINK:
		return MP_E_NO_SESSION;
	case -ENOSPC:
		return MP_E_BUSY;
	case -EPROTO:
		return MP_E_STATE;
	case -EIO:
		return MP_E_IO;
	default:
		return MP_E_INTERNAL;
	}
}

int mp_send(mp_ctx_t *c, uint8_t ch, const uint8_t *msg, size_t len)
{
	int rc;

	if ((c == NULL) || (c->w.tx == NULL)) {
		return -EINVAL;
	}
	rc = mp_frame_send(&c->txf, ch, msg, len, c->w.tx, c->w.tx_user);
	if (rc != 0) {
		c->tx_errors++;
		return rc;
	}
	c->records_sent++;
	return 0;
}

/* ======================================================== param accessors */

static int p_get(const mp_json_t *p, int params, const char *key)
{
	if (params < 0) {
		return -ENOENT;
	}
	return mp_json_obj_get(p, params, key);
}

/** Optional u32 param. Absent leaves @p out at @p dflt. */
static int p_u32(const mp_json_t *p, int params, const char *key, uint32_t dflt,
		 uint32_t *out)
{
	int t = p_get(p, params, key);
	int64_t v = 0;
	int rc;

	*out = dflt;
	if (t < 0) {
		return 0;
	}
	rc = mp_json_i64(p, t, &v);
	if (rc != 0) {
		return -EINVAL;
	}
	if ((v < 0) || (v > (int64_t)0xFFFFFFFFLL)) {
		return -ERANGE;
	}
	*out = (uint32_t)v;
	return 0;
}

/** Optional i32 param. */
static int p_i32(const mp_json_t *p, int params, const char *key, int32_t dflt,
		 int32_t *out)
{
	int t = p_get(p, params, key);
	int64_t v = 0;
	int rc;

	*out = dflt;
	if (t < 0) {
		return 0;
	}
	rc = mp_json_i64(p, t, &v);
	if (rc != 0) {
		return -EINVAL;
	}
	if ((v < -2147483648LL) || (v > 2147483647LL)) {
		return -ERANGE;
	}
	*out = (int32_t)v;
	return 0;
}

/** Optional bool param. */
static int p_bool(const mp_json_t *p, int params, const char *key, bool dflt,
		  bool *out)
{
	int t = p_get(p, params, key);

	*out = dflt;
	if (t < 0) {
		return 0;
	}
	if (mp_json_bool(p, t, out) != 0) {
		return -EINVAL;
	}
	return 0;
}

/** Optional string param; @p out is emptied when absent. */
static int p_str(const mp_json_t *p, int params, const char *key, char *out,
		 size_t cap)
{
	int t = p_get(p, params, key);

	out[0] = '\0';
	if (t < 0) {
		return 0;
	}
	if (mp_json_str(p, t, out, cap) < 0) {
		return -EINVAL;
	}
	return 1;
}

/** True when the params object carries @p key with a JSON null. */
static bool p_is_null(const mp_json_t *p, int params, const char *key)
{
	int t = p_get(p, params, key);
	const mp_json_tok_t *tok = mp_json_at(p, t);

	return (tok != NULL) && (tok->type == (uint8_t)MP_J_NULL);
}

/** Resolve the `id` param to a manifest index. */
static int p_obj(mp_ctx_t *c, const mp_json_t *p, int params)
{
	char id[48];
	int idx;

	if (p_str(p, params, "id", id, sizeof(id)) <= 0) {
		mp_fail(c, MP_E_BAD_PARAMS, "id");
		return -1;
	}
	idx = mp_obj_find(id);
	if (idx < 0) {
		mp_fail(c, MP_E_BAD_PARAMS, "unknown object");
		return -1;
	}
	return idx;
}

/**
 * Run the guard for @p guard, translating the outcome into an error latch.
 *
 * @param w  When the result is MP_GC_ARMED the arming detail is written into the
 *           reply through @p w, and the function returns 1.
 *
 * @retval 1   Armed: the caller must return 0 with the arming reply already
 *             written.
 * @retval 0   Permitted.
 * @retval -1  Refused; the error is latched.
 */
static int guard_or_fail(mp_ctx_t *c, uint8_t guard, const mp_json_t *p,
			 int params, uint16_t obj1, uint8_t tag, mp_jw_t *w)
{
	char confirm[MP_CONFIRM_MAX];
	char phrase[MP_CONFIRM_MAX];
	uint32_t sid = 0U;
	uint32_t nonce = 0U;
	mp_gc_arm_t arm;
	int gc;

	(void)p_u32(p, params, "sid", 0U, &sid);
	(void)p_u32(p, params, "nonce", 0U, &nonce);
	(void)p_str(p, params, "confirm", confirm, sizeof(confirm));
	(void)p_str(p, params, "phrase", phrase, sizeof(phrase));

	(void)memset(&arm, 0, sizeof(arm));
	gc = mp_ovr_guard(&c->ovr, guard, sid, obj1, tag, confirm, phrase, nonce,
			  mp_now(c), &arm);
	if (gc < 0) {
		mp_fail(c, MP_E_INTERNAL, "guard");
		return -1;
	}

	switch ((mp_gc_t)gc) {
	case MP_GC_OK:
		return 0;
	case MP_GC_ARMED:
		(void)mp_jw_obj_open(w);
		(void)mp_jw_kv_bool(w, "armed", true);
		(void)mp_jw_kv_u64(w, "nonce", arm.nonce);
		(void)mp_jw_kv_u64(w, "hold_ms", arm.hold_ms);
		(void)mp_jw_kv_u64(w, "expires_ms", arm.expires_ms);
		(void)mp_jw_kv_str(w, "guard", mp_guard_name(guard));
		(void)mp_jw_obj_close(w);
		return 1;
	case MP_GC_NEED_SESSION:
	case MP_GC_STALE:
		mp_fail(c, MP_E_NO_SESSION, mp_gc_name((uint8_t)gc));
		return -1;
	case MP_GC_NEED_HOLD:
		mp_fail(c, MP_E_HOLD, mp_gc_name((uint8_t)gc));
		return -1;
	default:
		mp_fail(c, MP_E_GUARD, mp_gc_name((uint8_t)gc));
		return -1;
	}
}

/** Write an object's value as the JSON member `value`. */
static void emit_val(mp_jw_t *w, const mp_obj_t *o, const mp_val_t *v)
{
	(void)mp_jw_key(w, "value");

	if (!v->valid && (o->kind != (uint8_t)MP_KIND_RAIL)) {
		(void)mp_jw_null(w);
		return;
	}

	switch (o->kind) {
	case MP_KIND_BOOL:
		(void)mp_jw_bool(w, v->i != 0);
		break;
	case MP_KIND_REAL:
		(void)mp_jw_f32(w, v->f, 6U);
		break;
	case MP_KIND_BITS:
		(void)mp_jw_u64(w, v->u);
		break;
	case MP_KIND_TEXT:
		(void)mp_jw_str(w, v->text);
		break;
	case MP_KIND_RAIL:
		(void)mp_jw_obj_open(w);
		(void)mp_jw_kv_i64(w, "mv", v->rail[0]);
		(void)mp_jw_kv_i64(w, "ua", v->rail[1]);
		(void)mp_jw_kv_i64(w, "uw", v->rail[2]);
		(void)mp_jw_kv_u64(w, "diag", (uint64_t)(uint32_t)v->rail[3]);
		(void)mp_jw_kv_bool(w, "valid", v->valid);
		(void)mp_jw_obj_close(w);
		break;
	default:
		(void)mp_jw_i64(w, v->i);
		break;
	}
}

/* ========================================================= method handlers */

typedef int (*handler_fn)(mp_ctx_t *c, const mp_json_t *p, int params,
			  mp_jw_t *w);

/**
 * Largest raw cfg-transfer chunk one `cfg.import` request may carry.
 *
 * Published in `hello.limits.cfg_chunk` so a host does not have to guess. It
 * bounds two stack buffers in m_cfg_import(), which is why it is a fixed
 * constant rather than the size of the whole import stream.
 */
#define MP_CFG_CHUNK_MAX 384U

/* ------------------------------------------------------------------- hello */

static int m_hello(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	unsigned int ch;

	(void)p;
	(void)params;

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "proto", MP_PROTO_VER);
	(void)mp_jw_kv_str(w, "protocol", "meridian-mp");
	(void)mp_jw_kv_str(w, "model", c->w.model);
	(void)mp_jw_kv_str(w, "serial", c->w.serial);
	(void)mp_jw_kv_str(w, "fw", c->w.fw_version);
	(void)mp_jw_kv_str(w, "boot", c->w.boot_version);
	(void)mp_jw_kv_str(w, "board", c->w.board_id);

	(void)mp_jw_key(w, "manifest");
	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "ver", MP_MANIFEST_VER);
	(void)mp_jw_kv_u64(w, "hash", c->manifest_hash);
	(void)mp_jw_kv_str(w, "hash_alg", "crc32-iso-hdlc");
	(void)mp_jw_kv_u64(w, "objects", mp_obj_count());
	/* No compressor is linked, so the manifest is served uncompressed. The
	 * spec's gzip encoding is optional; say so explicitly rather than let a
	 * host guess from a missing field. */
	(void)mp_jw_kv_str(w, "encoding", "identity");
	(void)mp_jw_kv_bool(w, "gzip", false);
	(void)mp_jw_obj_close(w);

	(void)mp_jw_key(w, "channels");
	(void)mp_jw_arr_open(w);
	for (ch = 0U; ch <= (unsigned int)MP_CH_MIRROR; ch++) {
		(void)mp_jw_obj_open(w);
		(void)mp_jw_kv_u64(w, "id", ch);
		(void)mp_jw_kv_str(w, "name", mp_channel_name((uint8_t)ch));
		(void)mp_jw_kv_bool(w, "sub",
				    mp_stream_subscribable((uint8_t)ch));
		(void)mp_jw_kv_bool(w, "paced", mp_stream_paced((uint8_t)ch));
		(void)mp_jw_obj_close(w);
	}
	(void)mp_jw_arr_close(w);

	(void)mp_jw_key(w, "limits");
	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "rx_payload", MP_PAYLOAD_MAX);
	(void)mp_jw_kv_u64(w, "tx_payload", MP_TX_PAYLOAD_MAX);
	(void)mp_jw_kv_u64(w, "rate_max_hz", MP_RATE_MAX_HZ);
	(void)mp_jw_kv_u64(w, "leases", MP_LEASE_MAX);
	(void)mp_jw_kv_u64(w, "keepalive_ms", MP_KEEPALIVE_TTL_MS);
	(void)mp_jw_kv_u64(w, "revert_ms", MP_DEADMAN_REVERT_MS);
	(void)mp_jw_kv_u64(w, "lease_ttl_max_ms", MP_LEASE_TTL_MAX_MS);
	(void)mp_jw_kv_u64(w, "g3_hold_ms", MP_G3_HOLD_MS);
	(void)mp_jw_kv_u64(w, "cfg_chunk", MP_CFG_CHUNK_MAX);
	(void)mp_jw_kv_bool(w, "batch", false);
	(void)mp_jw_obj_close(w);

	(void)mp_jw_key(w, "features");
	(void)mp_jw_arr_open(w);
	(void)mp_jw_str(w, "manifest");
	(void)mp_jw_str(w, "override");
	(void)mp_jw_str(w, "streams");
	(void)mp_jw_str(w, "diag");
	(void)mp_jw_str(w, "mirror");
	(void)mp_jw_str(w, "cfg");
	(void)mp_jw_str(w, "log");
	if (c->w.img != NULL) {
		(void)mp_jw_str(w, "sys");
	}
	(void)mp_jw_arr_close(w);

	(void)mp_jw_obj_close(w);
	return 0;
}

/* ------------------------------------------------------------ manifest.get */

static int m_manifest_get(mp_ctx_t *c, const mp_json_t *p, int params,
			  mp_jw_t *w)
{
	uint32_t from = 0U;
	size_t len = 0U;
	size_t next = 0U;
	int n;

	if (p_u32(p, params, "from", 0U, &from) != 0) {
		mp_fail(c, MP_E_BAD_PARAMS, "from");
		return MP_E_BAD_PARAMS;
	}
	if ((size_t)from > mp_obj_count()) {
		mp_fail(c, MP_E_RANGE, "from");
		return MP_E_RANGE;
	}

	/*
	 * The page is serialised into the reply buffer's own tail and then spliced
	 * down, so the manifest never needs a second buffer the size of the
	 * document. `HEAD_RESERVE` is the space kept for the members written
	 * before the array; it is checked, not assumed.
	 */
	{
#define HEAD_RESERVE 256U
		size_t page_off = w->len + HEAD_RESERVE;
		char *page;
		size_t page_cap;

		if (w->cap <= (page_off + 8U)) {
			mp_fail(c, MP_E_INTERNAL, "reply too small");
			return MP_E_INTERNAL;
		}
		page = &w->buf[page_off];
		/* Leave two bytes for the closing "]}" and one for the NUL. */
		page_cap = w->cap - page_off - 3U;

		n = mp_manifest_page((size_t)from, page, page_cap, &len, &next);
		if (n < 0) {
			mp_fail(c, MP_E_INTERNAL, "manifest");
			return MP_E_INTERNAL;
		}

		(void)mp_jw_obj_open(w);
		(void)mp_jw_kv_u64(w, "ver", MP_MANIFEST_VER);
		(void)mp_jw_kv_u64(w, "hash", c->manifest_hash);
		(void)mp_jw_kv_u64(w, "total", mp_obj_count());
		(void)mp_jw_kv_u64(w, "from", from);
		(void)mp_jw_kv_u64(w, "count", (uint64_t)n);
		(void)mp_jw_kv_u64(w, "next", next);
		(void)mp_jw_kv_bool(w, "done", next >= mp_obj_count());
		(void)mp_jw_kv_str(w, "encoding", "identity");
		(void)mp_jw_key(w, "objects");
		(void)mp_jw_arr_open(w);

		if (w->err != 0) {
			mp_fail(c, MP_E_INTERNAL, "reply overflow");
			return MP_E_INTERNAL;
		}
		/* The header must not have reached the staged fragment. */
		if (w->len > page_off) {
			mp_fail(c, MP_E_INTERNAL, "header reserve");
			return MP_E_INTERNAL;
		}

		if (len != 0U) {
			if ((w->len + len) >= w->cap) {
				mp_fail(c, MP_E_INTERNAL, "reply overflow");
				return MP_E_INTERNAL;
			}
			/* memmove: the regions overlap once the header is short. */
			(void)memmove(&w->buf[w->len], page, len);
			w->len += len;
			/* The fragment is a complete comma-separated element
			 * list, so the array now owes a separator. */
			w->sep[w->depth - 1U] = true;
		}
		(void)mp_jw_arr_close(w);
		(void)mp_jw_obj_close(w);
#undef HEAD_RESERVE
	}
	return 0;
}

/* ----------------------------------------------------------------- session */

static int m_session_open(mp_ctx_t *c, const mp_json_t *p, int params,
			  mp_jw_t *w)
{
	char client[32];
	uint32_t ttl = 0U;
	uint32_t sid = 0U;
	int rc;

	(void)p_u32(p, params, "ttl_ms", 0U, &ttl);
	(void)p_str(p, params, "client", client, sizeof(client));

	rc = mp_ovr_session_open(&c->ovr, ttl, mp_now(c), &sid);
	if (rc != 0) {
		mp_fail(c, mp_map_errno(rc), "session");
		return mp_map_errno(rc);
	}

	/* A new session starts from a clean stream and cfg-transfer state. */
	mp_stream_unsub_all(&c->st);
	c->cfg_ex_active = false;
	c->cfg_im_active = false;

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "sid", sid);
	(void)mp_jw_kv_u64(w, "keepalive_ms", c->ovr.sess.ttl_ms);
	(void)mp_jw_kv_u64(w, "expires_ms",
			   mp_ovr_session_remaining(&c->ovr, mp_now(c)));
	(void)mp_jw_kv_bool(w, "serial_required", true);
	(void)mp_jw_kv_str(w, "client", (client[0] != '\0') ? client : NULL);
	(void)mp_jw_obj_close(w);
	return 0;
}

static int m_session_keepalive(mp_ctx_t *c, const mp_json_t *p, int params,
			       mp_jw_t *w)
{
	uint32_t sid = 0U;
	int rc;

	(void)p_u32(p, params, "sid", 0U, &sid);
	rc = mp_ovr_keepalive(&c->ovr, sid, mp_now(c));
	if (rc != 0) {
		mp_fail(c, MP_E_NO_SESSION, "sid");
		return MP_E_NO_SESSION;
	}

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "sid", sid);
	(void)mp_jw_kv_u64(w, "expires_ms",
			   mp_ovr_session_remaining(&c->ovr, mp_now(c)));
	(void)mp_jw_kv_u64(w, "overrides", mp_ovr_active(&c->ovr));
	(void)mp_jw_obj_close(w);
	return 0;
}

static int m_session_close(mp_ctx_t *c, const mp_json_t *p, int params,
			   mp_jw_t *w)
{
	uint32_t sid = 0U;
	int rc;

	(void)p_u32(p, params, "sid", 0U, &sid);
	rc = mp_ovr_session_close(&c->ovr, sid, mp_now(c));
	if (rc != 0) {
		mp_fail(c, MP_E_NO_SESSION, "sid");
		return MP_E_NO_SESSION;
	}
	mp_stream_unsub_all(&c->st);
	c->cfg_ex_active = false;
	c->cfg_im_active = false;

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_bool(w, "closed", true);
	(void)mp_jw_obj_close(w);
	return 0;
}

/* ------------------------------------------------------------------ objects */

static int m_obj_get(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	int idx = p_obj(c, p, params);
	const mp_obj_t *o;
	const mp_lease_t *l;
	mp_val_t v;
	int rc;

	if (idx < 0) {
		return c->err_code;
	}
	o = mp_obj_at((size_t)idx);
	if (c->w.obj_read == NULL) {
		mp_fail(c, MP_E_NOTSUP, "obj_read");
		return MP_E_NOTSUP;
	}

	(void)memset(&v, 0, sizeof(v));
	v.kind = o->kind;
	rc = c->w.obj_read(c->w.obj_read_user, (size_t)idx, &v);
	if (rc < 0) {
		mp_fail(c, mp_map_errno(rc), "read");
		return mp_map_errno(rc);
	}

	l = mp_ovr_lease(&c->ovr, (size_t)idx);

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_str(w, "id", o->id);
	(void)mp_jw_kv_str(w, "kind", mp_kind_name(o->kind));
	emit_val(w, o, &v);
	(void)mp_jw_kv_str(w, "source", (l != NULL) ? "override" : "auto");
	if (l != NULL) {
		(void)mp_jw_key(w, "lease");
		(void)mp_jw_obj_open(w);
		(void)mp_jw_kv_u64(w, "sid", l->sid);
		(void)mp_jw_kv_i64(w, "value", l->value);
		(void)mp_jw_kv_u64(w, "remaining_ms",
				   (uint64_t)(uint32_t)(l->deadline_ms -
							mp_now(c)));
		(void)mp_jw_obj_close(w);
	}
	if (o->unit != NULL) {
		(void)mp_jw_kv_str(w, "unit", o->unit);
	}
	(void)mp_jw_obj_close(w);
	return 0;
}

/** Stage and commit a cfg-backed object's value. */
static int cfg_write(mp_ctx_t *c, const mp_obj_t *o, int32_t value)
{
	const cfg_key_t *k;
	cfg_commit_res_t res;
	int rc;

	if ((c->w.cfg == NULL) || (c->w.cfg_commit == NULL)) {
		return -ENOTSUP;
	}
	k = cfg_key_find(o->cfg_key);
	if (k == NULL) {
		return -ENOENT;
	}

	switch (k->type) {
	case CFG_T_BOOL:
	case CFG_T_U8:
	case CFG_T_U16:
	case CFG_T_U32:
	case CFG_T_U64:
		if (value < 0) {
			return -ERANGE;
		}
		rc = cfg_set_u64(c->w.cfg, o->cfg_key, (uint64_t)value);
		break;
	case CFG_T_I32:
		rc = cfg_set_i32(c->w.cfg, o->cfg_key, value);
		break;
	case CFG_T_F32:
		rc = cfg_set_f32(c->w.cfg, o->cfg_key, (float)value);
		break;
	default:
		return -ENOTSUP;
	}
	if (rc != 0) {
		return rc;
	}

	(void)memset(&res, 0, sizeof(res));
	return c->w.cfg_commit(c->w.cfg_user, &res);
}

static int m_obj_set(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	int idx = p_obj(c, p, params);
	const mp_obj_t *o;
	mp_ilk_state_t ilk;
	mp_ilk_res_t res;
	int32_t value = 0;
	int g;
	int rc;

	if (idx < 0) {
		return c->err_code;
	}
	o = mp_obj_at((size_t)idx);
	if ((o->flags & MP_OF_WRITE) == 0U) {
		mp_fail(c, MP_E_NOTSUP, "not writable");
		return MP_E_NOTSUP;
	}
	if (p_get(p, params, "value") < 0) {
		mp_fail(c, MP_E_BAD_PARAMS, "value");
		return MP_E_BAD_PARAMS;
	}
	if (o->kind == (uint8_t)MP_KIND_BOOL) {
		bool b = false;

		if (p_bool(p, params, "value", false, &b) != 0) {
			int32_t n = 0;

			if (p_i32(p, params, "value", 0, &n) != 0) {
				mp_fail(c, MP_E_BAD_PARAMS, "value");
				return MP_E_BAD_PARAMS;
			}
			b = (n != 0);
		}
		value = b ? 1 : 0;
	} else if (p_i32(p, params, "value", 0, &value) != 0) {
		mp_fail(c, MP_E_BAD_PARAMS, "value");
		return MP_E_BAD_PARAMS;
	}

	/*
	 * The manifest's envelope binds `set` exactly as it binds `override`. A
	 * cfg-backed object would otherwise be limited only by its schema row,
	 * which may be wider than what the manifest promised the tool.
	 */
	rc = mp_obj_check_value((size_t)idx, value);
	if (rc != 0) {
		mp_fail(c, mp_map_errno(rc), "value");
		return mp_map_errno(rc);
	}

	g = guard_or_fail(c, o->guard, p, params, (uint16_t)(idx + 1), 0U, w);
	if (g != 0) {
		return (g > 0) ? 0 : c->err_code;
	}

	(void)memset(&ilk, 0, sizeof(ilk));
	if (c->w.ilk != NULL) {
		if (c->w.ilk(c->w.state_user, &ilk) != 0) {
			mp_fail(c, MP_E_IO, "interlock state");
			return MP_E_IO;
		}
	}

	rc = mp_ilk_eval((size_t)idx, value, (c->w.ilk != NULL) ? &ilk : NULL,
			 mp_now(c), &res);
	if (rc != 0) {
		mp_fail(c, MP_E_INTERLOCK, mp_ilk_name(res.failed));
		return MP_E_INTERLOCK;
	}

	if ((o->flags & MP_OF_CFG) != 0U) {
		rc = cfg_write(c, o, res.value);
	} else if (c->w.apply != NULL) {
		/*
		 * A `set` is not a lease: firmware keeps ownership and may
		 * re-assert the object on its next pass. That is the documented
		 * difference from `obj.override` (FMT §5.1) and the reason a set
		 * needs no keepalive to survive.
		 */
		rc = c->w.apply(c->w.apply_user, (size_t)idx, &res.value);
	} else {
		rc = -ENOTSUP;
	}
	if (rc != 0) {
		mp_fail(c, mp_map_errno(rc), "apply");
		return mp_map_errno(rc);
	}

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_str(w, "id", o->id);
	(void)mp_jw_kv_i64(w, "value", res.value);
	(void)mp_jw_kv_bool(w, "clamped", res.clamped);
	(void)mp_jw_kv_bool(w, "persistent", (o->flags & MP_OF_CFG) != 0U);
	(void)mp_jw_obj_close(w);
	return 0;
}

static int m_obj_override(mp_ctx_t *c, const mp_json_t *p, int params,
			  mp_jw_t *w)
{
	int idx = p_obj(c, p, params);
	const mp_obj_t *o;
	mp_ilk_state_t ilk;
	mp_ilk_res_t res;
	uint32_t sid = 0U;
	uint32_t ttl = 0U;
	int32_t value = 0;
	int g;
	int rc;

	if (idx < 0) {
		return c->err_code;
	}
	o = mp_obj_at((size_t)idx);
	(void)p_u32(p, params, "sid", 0U, &sid);

	/* `value: null` releases the lease and returns the object to firmware. */
	if (p_is_null(p, params, "value")) {
		rc = mp_ovr_release(&c->ovr, (size_t)idx, sid, mp_now(c));
		if (rc != 0) {
			mp_fail(c, mp_map_errno(rc), "release");
			return mp_map_errno(rc);
		}
		(void)mp_jw_obj_open(w);
		(void)mp_jw_kv_str(w, "id", o->id);
		(void)mp_jw_kv_bool(w, "released", true);
		(void)mp_jw_obj_close(w);
		return 0;
	}

	if ((o->flags & MP_OF_OVERRIDE) == 0U) {
		mp_fail(c, MP_E_NOTSUP, "not overridable");
		return MP_E_NOTSUP;
	}
	if (o->kind == (uint8_t)MP_KIND_BOOL) {
		bool b = false;

		if (p_bool(p, params, "value", false, &b) != 0) {
			int32_t n = 0;

			if (p_i32(p, params, "value", 0, &n) != 0) {
				mp_fail(c, MP_E_BAD_PARAMS, "value");
				return MP_E_BAD_PARAMS;
			}
			b = (n != 0);
		}
		value = b ? 1 : 0;
	} else if (p_i32(p, params, "value", 0, &value) != 0) {
		mp_fail(c, MP_E_BAD_PARAMS, "value");
		return MP_E_BAD_PARAMS;
	}
	(void)p_u32(p, params, "ttl_ms", 0U, &ttl);

	g = guard_or_fail(c, o->guard, p, params, (uint16_t)(idx + 1), 0U, w);
	if (g != 0) {
		return (g > 0) ? 0 : c->err_code;
	}

	(void)memset(&ilk, 0, sizeof(ilk));
	if (c->w.ilk != NULL) {
		if (c->w.ilk(c->w.state_user, &ilk) != 0) {
			mp_fail(c, MP_E_IO, "interlock state");
			return MP_E_IO;
		}
	}

	rc = mp_ovr_grant(&c->ovr, (size_t)idx, value, ttl, sid,
			  (c->w.ilk != NULL) ? &ilk : NULL, mp_now(c), &res);
	if (rc != 0) {
		int code = mp_map_errno(rc);

		mp_fail(c, code,
			(rc == -EPERM) ? mp_ilk_name(res.failed) : "override");
		return code;
	}

	{
		const mp_lease_t *l = mp_ovr_lease(&c->ovr, (size_t)idx);

		(void)mp_jw_obj_open(w);
		(void)mp_jw_kv_str(w, "id", o->id);
		(void)mp_jw_kv_i64(w, "value", res.value);
		(void)mp_jw_kv_bool(w, "clamped", res.clamped);
		(void)mp_jw_kv_bool(w, "verify_pending", res.verify);
		if (res.tunnel) {
			/* §5.5: a tunnel suspends firmware's use of the port, so
			 * the reference it feeds must be treated as suspect. */
			(void)mp_jw_kv_bool(w, "reference_suspect", true);
		}
		(void)mp_jw_kv_u64(
			w, "expires_ms",
			(l != NULL)
				? (uint64_t)(uint32_t)(l->deadline_ms -
						       mp_now(c))
				: 0U);
		(void)mp_jw_kv_u64(w, "keepalive_ms", MP_KEEPALIVE_TTL_MS);
		(void)mp_jw_obj_close(w);
	}
	return 0;
}

static int m_obj_pulse(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	int idx = p_obj(c, p, params);
	const mp_obj_t *o;
	mp_ilk_state_t ilk;
	mp_ilk_res_t res;
	uint32_t ms = 0U;
	int g;
	int rc;

	if (idx < 0) {
		return c->err_code;
	}
	o = mp_obj_at((size_t)idx);
	if ((o->flags & MP_OF_PULSE) == 0U) {
		mp_fail(c, MP_E_NOTSUP, "not pulsable");
		return MP_E_NOTSUP;
	}
	if (p_u32(p, params, "ms", (uint32_t)o->min, &ms) != 0) {
		mp_fail(c, MP_E_BAD_PARAMS, "ms");
		return MP_E_BAD_PARAMS;
	}
	if ((ms < (uint32_t)o->min) || (ms > (uint32_t)o->max)) {
		mp_fail(c, MP_E_RANGE, "ms");
		return MP_E_RANGE;
	}
	if (c->w.pulse == NULL) {
		mp_fail(c, MP_E_NOTSUP, "pulse");
		return MP_E_NOTSUP;
	}

	g = guard_or_fail(c, o->guard, p, params, (uint16_t)(idx + 1), 0U, w);
	if (g != 0) {
		return (g > 0) ? 0 : c->err_code;
	}

	(void)memset(&ilk, 0, sizeof(ilk));
	if (c->w.ilk != NULL) {
		if (c->w.ilk(c->w.state_user, &ilk) != 0) {
			mp_fail(c, MP_E_IO, "interlock state");
			return MP_E_IO;
		}
	}
	/* A pulse asserts the object, so the interlocks are evaluated against a
	 * non-zero request even though the value itself is a duration. */
	rc = mp_ilk_eval((size_t)idx, 1, (c->w.ilk != NULL) ? &ilk : NULL,
			 mp_now(c), &res);
	if (rc != 0) {
		mp_fail(c, MP_E_INTERLOCK, mp_ilk_name(res.failed));
		return MP_E_INTERLOCK;
	}

	rc = c->w.pulse(c->w.pulse_user, (size_t)idx, ms);
	if (rc != 0) {
		mp_fail(c, mp_map_errno(rc), "pulse");
		return mp_map_errno(rc);
	}

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_str(w, "id", o->id);
	(void)mp_jw_kv_u64(w, "ms", ms);
	(void)mp_jw_obj_close(w);
	return 0;
}

/* ------------------------------------------------------------------ streams */

/** Resolve the `ch` param, which may be a number or a channel name. */
static int p_channel(mp_ctx_t *c, const mp_json_t *p, int params)
{
	int t = p_get(p, params, "ch");
	const mp_json_tok_t *tok = mp_json_at(p, t);
	unsigned int i;

	if (tok == NULL) {
		mp_fail(c, MP_E_BAD_PARAMS, "ch");
		return -1;
	}
	if (tok->type == (uint8_t)MP_J_NUM) {
		int64_t v = 0;

		if ((mp_json_i64(p, t, &v) != 0) || (v < 0) ||
		    (v > (int64_t)MP_CH_MAX)) {
			mp_fail(c, MP_E_BAD_PARAMS, "ch");
			return -1;
		}
		return (int)v;
	}
	if (tok->type == (uint8_t)MP_J_STR) {
		for (i = 0U; i <= (unsigned int)MP_CH_MIRROR; i++) {
			if (mp_json_streq(p, t,
					  mp_channel_name((uint8_t)i))) {
				return (int)i;
			}
		}
	}
	mp_fail(c, MP_E_BAD_PARAMS, "ch");
	return -1;
}

static int m_stream_sub(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	int ch = p_channel(c, p, params);
	uint32_t rate = 0U;
	uint32_t cursor = 0U;
	int g;
	int rc;

	if (ch < 0) {
		return c->err_code;
	}
	(void)p_u32(p, params, "rate_hz", 1U, &rate);
	(void)p_u32(p, params, "cursor", 0U, &cursor);

	/* A subscription is observation only, but it consumes device time and
	 * bandwidth, so it needs a session (G1). */
	g = guard_or_fail(c, (uint8_t)MP_GUARD_G1, p, params, 0U, 0U, w);
	if (g != 0) {
		return (g > 0) ? 0 : c->err_code;
	}

	if (rate > 255U) {
		mp_fail(c, MP_E_RANGE, "rate_hz");
		return MP_E_RANGE;
	}
	rc = mp_stream_sub(&c->st, (uint8_t)ch, (uint8_t)rate, mp_now(c));
	if (rc != 0) {
		mp_fail(c, mp_map_errno(rc), "sub");
		return mp_map_errno(rc);
	}
	if ((uint8_t)ch == MP_CH_LOG) {
		mp_stream_set_cursor(&c->st, (uint8_t)ch, cursor);
	}
	if ((uint8_t)ch == MP_CH_MIRROR) {
		/* A new mirror subscriber must start from a keyframe. */
		mp_mirror_reset(&c->mirror);
	}

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "ch", (uint64_t)ch);
	(void)mp_jw_kv_str(w, "name", mp_channel_name((uint8_t)ch));
	(void)mp_jw_kv_u64(w, "rate_hz", c->st.sub[ch].rate_hz);
	(void)mp_jw_kv_bool(w, "paced", mp_stream_paced((uint8_t)ch));
	(void)mp_jw_obj_close(w);
	return 0;
}

static int m_stream_unsub(mp_ctx_t *c, const mp_json_t *p, int params,
			  mp_jw_t *w)
{
	int ch = p_channel(c, p, params);
	int rc;

	if (ch < 0) {
		return c->err_code;
	}
	rc = mp_stream_unsub(&c->st, (uint8_t)ch);
	if (rc != 0) {
		mp_fail(c, mp_map_errno(rc), "unsub");
		return mp_map_errno(rc);
	}

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "ch", (uint64_t)ch);
	(void)mp_jw_kv_bool(w, "subscribed", false);
	(void)mp_jw_obj_close(w);
	return 0;
}

/* -------------------------------------------------------------- diagnostics */

/**
 * diag.list — a documented extension.
 *
 * FMT §3.2 lists only `diag.run`, but the manifest describes objects, not tests,
 * so without this a tool would have to hard-code the registry and would silently
 * disagree with a firmware that added a test.
 */
static int m_diag_list(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	size_t i;

	(void)c;
	(void)p;
	(void)params;

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "count", MP_DIAG_COUNT);
	(void)mp_jw_key(w, "tests");
	(void)mp_jw_arr_open(w);
	for (i = 0U; i < MP_DIAG_COUNT; i++) {
		const mp_diag_test_t *t = &mp_diag_tests[i];

		(void)mp_jw_obj_open(w);
		(void)mp_jw_kv_str(w, "name", t->name);
		(void)mp_jw_kv_str(w, "guard", mp_guard_name(t->guard));
		(void)mp_jw_kv_u64(w, "steps", t->steps);
		(void)mp_jw_kv_u64(w, "timeout_ms", t->step_timeout_ms);
		(void)mp_jw_kv_bool(w, "deferred", t->deferred);
		(void)mp_jw_kv_bool(w, "disruptive", t->disruptive);
		if (t->ilk != 0U) {
			uint32_t bit;

			(void)mp_jw_key(w, "ilk");
			(void)mp_jw_arr_open(w);
			for (bit = 1U; bit <= MP_ILK_ALL; bit <<= 1) {
				if ((t->ilk & bit) != 0U) {
					(void)mp_jw_str(w, mp_ilk_name(bit));
				}
			}
			(void)mp_jw_arr_close(w);
		}
		(void)mp_jw_kv_str(w, "desc", t->desc);
		(void)mp_jw_obj_close(w);
	}
	(void)mp_jw_arr_close(w);
	(void)mp_jw_obj_close(w);
	return 0;
}

static int m_diag_run(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	char name[32];
	const mp_diag_test_t *t;
	mp_ilk_state_t ilk;
	mp_ilk_res_t res;
	uint32_t sid = 0U;
	uint32_t run_id = 0U;
	int id;
	int g;
	int rc;

	if (p_str(p, params, "test", name, sizeof(name)) <= 0) {
		mp_fail(c, MP_E_BAD_PARAMS, "test");
		return MP_E_BAD_PARAMS;
	}
	id = mp_diag_find(name);
	if (id < 0) {
		mp_fail(c, MP_E_BAD_PARAMS, "unknown test");
		return MP_E_BAD_PARAMS;
	}
	t = &mp_diag_tests[id];
	(void)p_u32(p, params, "sid", 0U, &sid);

	/* A diagnostic is not an object, so the G3 arm is bound to the test id
	 * through the action tag rather than an object index. */
	g = guard_or_fail(c, t->guard, p, params, 0U, (uint8_t)(id + 1), w);
	if (g != 0) {
		return (g > 0) ? 0 : c->err_code;
	}

	if (t->ilk != 0U) {
		(void)memset(&ilk, 0, sizeof(ilk));
		if (c->w.ilk != NULL) {
			if (c->w.ilk(c->w.state_user, &ilk) != 0) {
				mp_fail(c, MP_E_IO, "interlock state");
				return MP_E_IO;
			}
		}
		/*
		 * The registry's interlock mask is evaluated through a synthetic
		 * object-free path: only the refusal half of mp_ilk_eval() is
		 * meaningful for a test, so an object that declares the same
		 * mask is used as the carrier. Tests declare only refusal-type
		 * interlocks (fan floor, relay, WDT liveness).
		 */
		(void)memset(&res, 0, sizeof(res));
		if ((t->ilk & MP_ILK_WDT_LIVE) != 0U) {
			if ((c->w.ilk == NULL) || !ilk.liveness_ok) {
				mp_fail(c, MP_E_INTERLOCK,
					mp_ilk_name(MP_ILK_WDT_LIVE));
				return MP_E_INTERLOCK;
			}
		}
		if ((t->ilk & MP_ILK_RELAY_OK) != 0U) {
			if ((c->w.ilk == NULL) || ilk.relay_blocked) {
				mp_fail(c, MP_E_INTERLOCK,
					mp_ilk_name(MP_ILK_RELAY_OK));
				return MP_E_INTERLOCK;
			}
		}
	}

	rc = mp_diag_start(&c->diag, (uint8_t)id, sid, mp_now(c), &run_id);
	if (rc != 0) {
		mp_fail(c, mp_map_errno(rc), "start");
		return mp_map_errno(rc);
	}

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_str(w, "test", t->name);
	(void)mp_jw_kv_u64(w, "run_id", run_id);
	(void)mp_jw_kv_u64(w, "steps", t->steps);
	(void)mp_jw_kv_str(w, "guard", mp_guard_name(t->guard));
	(void)mp_jw_kv_u64(w, "channel", MP_CH_EVENT);
	(void)mp_jw_obj_close(w);
	return 0;
}

static int m_diag_abort(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	int g = guard_or_fail(c, (uint8_t)MP_GUARD_G1, p, params, 0U, 0U, w);
	int rc;

	if (g != 0) {
		return (g > 0) ? 0 : c->err_code;
	}
	rc = mp_diag_abort(&c->diag, mp_now(c));
	if (rc != 0) {
		mp_fail(c, mp_map_errno(rc), "abort");
		return mp_map_errno(rc);
	}
	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_bool(w, "aborted", true);
	(void)mp_jw_obj_close(w);
	return 0;
}

/**
 * diag.snapshot — the §8.1 support bundle.
 *
 * The archive is kilobytes of CBOR, so it goes out fragmented on the telemetry
 * channel and the reply carries only its length and a CRC. Base64 in a JSON
 * reply would need a buffer four thirds the size of the archive for no gain: the
 * host already has a decoder for that channel.
 */
static int m_diag_snapshot(mp_ctx_t *c, const mp_json_t *p, int params,
			   mp_jw_t *w)
{
	mp_bundle_t b;
	mp_telem_t telem;
	int g;
	int n;
	int rc;

	g = guard_or_fail(c, (uint8_t)MP_GUARD_G1, p, params, 0U, 0U, w);
	if (g != 0) {
		return (g > 0) ? 0 : c->err_code;
	}
	if (c->w.bundle == NULL) {
		mp_fail(c, MP_E_NOTSUP, "bundle");
		return MP_E_NOTSUP;
	}

	(void)memset(&b, 0, sizeof(b));
	(void)memset(&telem, 0, sizeof(telem));

	rc = c->w.bundle(c->w.state_user, &b);
	if (rc != 0) {
		mp_fail(c, mp_map_errno(rc), "bundle");
		return mp_map_errno(rc);
	}
	/* Fill in the pieces core already owns, so the glue cannot forget them. */
	if (b.telem == NULL) {
		if ((c->w.telem != NULL) &&
		    (c->w.telem(c->w.state_user, &telem) == 0)) {
			b.telem = &telem;
		}
	}
	if (b.serial == NULL) {
		b.serial = c->w.serial;
	}
	if (b.fw_version == NULL) {
		b.fw_version = c->w.fw_version;
	}
	if (b.boot_version == NULL) {
		b.boot_version = c->w.boot_version;
	}
	if (b.board_id == NULL) {
		b.board_id = c->w.board_id;
	}
	b.manifest_hash = c->manifest_hash;

	n = mp_enc_bundle(&b, mp_stream_next_seq(&c->st, MP_CH_TELEMETRY),
			  mp_now(c), c->w.scratch, c->w.scratch_len);
	if (n < 0) {
		mp_fail(c, (n == -ENOSPC) ? MP_E_BUSY : MP_E_INTERNAL,
			"encode");
		return (n == -ENOSPC) ? MP_E_BUSY : MP_E_INTERNAL;
	}

	rc = mp_send(c, MP_CH_TELEMETRY, c->w.scratch, (size_t)n);
	if (rc != 0) {
		mp_fail(c, MP_E_IO, "tx");
		return MP_E_IO;
	}

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "bytes", (uint64_t)n);
	(void)mp_jw_kv_u64(w, "channel", MP_CH_TELEMETRY);
	(void)mp_jw_kv_u64(w, "record", MP_REC_BUNDLE);
	(void)mp_jw_kv_u64(w, "crc32",
			   sts_crc32_ieee(c->w.scratch, (size_t)n));
	(void)mp_jw_obj_close(w);
	return 0;
}

/* ------------------------------------------------------------ calibration */

/** Emit one cfg key as a JSON object. */
static void emit_cfg_key(mp_jw_t *w, const cfg_ctx_t *cfg, const cfg_key_t *k)
{
	cfg_val_t v;

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "key", k->id);
	(void)mp_jw_kv_str(w, "name", k->name);
	(void)mp_jw_kv_u64(w, "type", k->type);
	if (cfg_get_effective(cfg, k->id, &v) == 0) {
		switch (v.type) {
		case CFG_T_I32:
			(void)mp_jw_kv_i64(w, "value", v.v.i);
			break;
		case CFG_T_F32:
			(void)mp_jw_kv_f32(w, "value", v.v.f, 6U);
			break;
		case CFG_T_STR:
		case CFG_T_BLOB:
			(void)mp_jw_kv_u64(w, "len", v.len);
			break;
		default:
			(void)mp_jw_kv_u64(w, "value", v.v.u);
			break;
		}
	} else {
		(void)mp_jw_kv_null(w, "value");
	}
	(void)mp_jw_kv_bool(w, "staged", cfg_is_staged(cfg, k->id));
	(void)mp_jw_obj_close(w);
}

static int m_cal_list(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	size_t i;
	size_t n = cfg_key_count();

	(void)p;
	(void)params;

	if (c->w.cfg == NULL) {
		mp_fail(c, MP_E_NOTSUP, "cfg");
		return MP_E_NOTSUP;
	}

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "group", CFG_G_CAL);
	(void)mp_jw_key(w, "keys");
	(void)mp_jw_arr_open(w);
	for (i = 0U; i < n; i++) {
		const cfg_key_t *k = cfg_key_at(i);

		if ((k != NULL) && (CFG_GROUP(k->id) == CFG_G_CAL)) {
			emit_cfg_key(w, c->w.cfg, k);
		}
	}
	(void)mp_jw_arr_close(w);
	(void)mp_jw_obj_close(w);
	return 0;
}

/** Resolve the `key` param (numeric id or schema name) to a cal key row. */
static const cfg_key_t *p_cal_key(mp_ctx_t *c, const mp_json_t *p, int params)
{
	int t = p_get(p, params, "key");
	const mp_json_tok_t *tok = mp_json_at(p, t);
	const cfg_key_t *k = NULL;

	if (tok == NULL) {
		mp_fail(c, MP_E_BAD_PARAMS, "key");
		return NULL;
	}
	if (tok->type == (uint8_t)MP_J_NUM) {
		int64_t v = 0;

		if ((mp_json_i64(p, t, &v) != 0) || (v < 0) || (v > 0xFFFF)) {
			mp_fail(c, MP_E_BAD_PARAMS, "key");
			return NULL;
		}
		k = cfg_key_find((uint16_t)v);
	} else if (tok->type == (uint8_t)MP_J_STR) {
		size_t i;
		size_t n = cfg_key_count();

		for (i = 0U; i < n; i++) {
			const cfg_key_t *row = cfg_key_at(i);

			if ((row != NULL) && mp_json_streq(p, t, row->name)) {
				k = row;
				break;
			}
		}
	}
	if (k == NULL) {
		mp_fail(c, MP_E_BAD_PARAMS, "unknown key");
		return NULL;
	}
	if (CFG_GROUP(k->id) != CFG_G_CAL) {
		/* cal.* is scoped to group 0x0C on purpose: it must not become a
		 * second, unguarded path into the whole config tree. */
		mp_fail(c, MP_E_BAD_PARAMS, "not a cal key");
		return NULL;
	}
	return k;
}

static int m_cal_get(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	const cfg_key_t *k;

	if (c->w.cfg == NULL) {
		mp_fail(c, MP_E_NOTSUP, "cfg");
		return MP_E_NOTSUP;
	}
	k = p_cal_key(c, p, params);
	if (k == NULL) {
		return c->err_code;
	}

	(void)mp_jw_obj_open(w);
	(void)mp_jw_key(w, "key");
	emit_cfg_key(w, c->w.cfg, k);
	(void)mp_jw_obj_close(w);
	return 0;
}

static int m_cal_set(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	const cfg_key_t *k;
	int g;
	int rc;

	if (c->w.cfg == NULL) {
		mp_fail(c, MP_E_NOTSUP, "cfg");
		return MP_E_NOTSUP;
	}
	k = p_cal_key(c, p, params);
	if (k == NULL) {
		return c->err_code;
	}

	/* Calibration changes what every downstream reading means, so G2. */
	g = guard_or_fail(c, (uint8_t)MP_GUARD_G2, p, params, 0U, 0U, w);
	if (g != 0) {
		return (g > 0) ? 0 : c->err_code;
	}

	if (k->type == CFG_T_F32) {
		int t = p_get(p, params, "value");
		float f = 0.0f;

		if ((t < 0) || (mp_json_f32(p, t, &f) != 0)) {
			mp_fail(c, MP_E_BAD_PARAMS, "value");
			return MP_E_BAD_PARAMS;
		}
		rc = cfg_set_f32(c->w.cfg, k->id, f);
	} else if (k->type == CFG_T_I32) {
		int32_t v = 0;

		if (p_i32(p, params, "value", 0, &v) != 0) {
			mp_fail(c, MP_E_BAD_PARAMS, "value");
			return MP_E_BAD_PARAMS;
		}
		rc = cfg_set_i32(c->w.cfg, k->id, v);
	} else {
		uint32_t v = 0U;

		if (p_u32(p, params, "value", 0U, &v) != 0) {
			mp_fail(c, MP_E_BAD_PARAMS, "value");
			return MP_E_BAD_PARAMS;
		}
		rc = cfg_set_u64(c->w.cfg, k->id, v);
	}
	if (rc != 0) {
		mp_fail(c, mp_map_errno(rc), "stage");
		return mp_map_errno(rc);
	}

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "key", k->id);
	(void)mp_jw_kv_bool(w, "staged", true);
	(void)mp_jw_kv_u64(w, "pending", cfg_staged_count(c->w.cfg));
	(void)mp_jw_obj_close(w);
	return 0;
}

static int m_cal_commit(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	cfg_commit_res_t res;
	int g;
	int rc;

	if ((c->w.cfg == NULL) || (c->w.cfg_commit == NULL)) {
		mp_fail(c, MP_E_NOTSUP, "cfg");
		return MP_E_NOTSUP;
	}
	g = guard_or_fail(c, (uint8_t)MP_GUARD_G2, p, params, 0U, 0U, w);
	if (g != 0) {
		return (g > 0) ? 0 : c->err_code;
	}

	(void)memset(&res, 0, sizeof(res));
	rc = c->w.cfg_commit(c->w.cfg_user, &res);

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "staged", res.staged);
	(void)mp_jw_kv_u64(w, "applied", res.applied);
	(void)mp_jw_kv_u64(w, "persist_errors", res.persist_errors);
	(void)mp_jw_kv_u64(w, "reboot_keys", res.reboot_keys);
	(void)mp_jw_kv_bool(w, "ok", rc == 0);
	(void)mp_jw_obj_close(w);

	if ((rc != 0) && (rc != -EIO)) {
		/* -EIO means applied-but-not-persisted, which the reply above
		 * already reports; anything else means nothing was applied. */
		mp_fail(c, mp_map_errno(rc), "commit");
		return mp_map_errno(rc);
	}
	return 0;
}

static int m_cal_revert(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	int g;

	if (c->w.cfg == NULL) {
		mp_fail(c, MP_E_NOTSUP, "cfg");
		return MP_E_NOTSUP;
	}
	g = guard_or_fail(c, (uint8_t)MP_GUARD_G1, p, params, 0U, 0U, w);
	if (g != 0) {
		return (g > 0) ? 0 : c->err_code;
	}
	(void)cfg_revert(c->w.cfg);

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_bool(w, "reverted", true);
	(void)mp_jw_obj_close(w);
	return 0;
}

/* --------------------------------------------------------------- cfg export */

static int m_cfg_export(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	uint8_t chunk[CFG_EXPORT_MIN_CHUNK];
	size_t len = 0U;
	bool restart = false;
	bool secrets = false;
	int g;
	int rc;

	if (c->w.cfg == NULL) {
		mp_fail(c, MP_E_NOTSUP, "cfg");
		return MP_E_NOTSUP;
	}
	g = guard_or_fail(c, (uint8_t)MP_GUARD_G1, p, params, 0U, 0U, w);
	if (g != 0) {
		return (g > 0) ? 0 : c->err_code;
	}
	(void)p_bool(p, params, "restart", false, &restart);
	(void)p_bool(p, params, "secrets", false, &secrets);

	if (restart || !c->cfg_ex_active) {
		rc = cfg_export_begin(c->w.cfg, &c->cfg_ex, secrets);
		if (rc != 0) {
			mp_fail(c, mp_map_errno(rc), "export");
			return mp_map_errno(rc);
		}
		c->cfg_ex_active = true;
	}

	rc = cfg_export_read(c->w.cfg, &c->cfg_ex, chunk, sizeof(chunk), &len);
	if (rc < 0) {
		c->cfg_ex_active = false;
		mp_fail(c, mp_map_errno(rc), "export");
		return mp_map_errno(rc);
	}

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "offset", c->cfg_ex.offset - len);
	(void)mp_jw_kv_u64(w, "len", len);
	(void)mp_jw_kv_bool(w, "done", rc == 1);
	(void)mp_jw_kv_str(w, "encoding", "base64");
	(void)mp_jw_key(w, "data");
	{
		char b64[MP_B64_LEN(CFG_EXPORT_MIN_CHUNK) + 1U];

		if (mp_b64_encode(chunk, len, b64, sizeof(b64)) < 0) {
			mp_fail(c, MP_E_INTERNAL, "base64");
			return MP_E_INTERNAL;
		}
		(void)mp_jw_str(w, b64);
	}
	(void)mp_jw_obj_close(w);

	if (rc == 1) {
		c->cfg_ex_active = false;
	}
	return 0;
}

static int m_cfg_import(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	uint8_t raw[MP_CFG_CHUNK_MAX];
	char b64[MP_B64_LEN(MP_CFG_CHUNK_MAX) + 4U];
	cfg_commit_res_t res;
	size_t consumed = 0U;
	bool restart = false;
	bool strict = false;
	int n;
	int g;
	int rc;

	if (c->w.cfg == NULL) {
		mp_fail(c, MP_E_NOTSUP, "cfg");
		return MP_E_NOTSUP;
	}
	/* Importing a config image rewrites the whole tree; G2. */
	g = guard_or_fail(c, (uint8_t)MP_GUARD_G2, p, params, 0U, 0U, w);
	if (g != 0) {
		return (g > 0) ? 0 : c->err_code;
	}
	(void)p_bool(p, params, "restart", false, &restart);
	(void)p_bool(p, params, "strict", false, &strict);

	if (restart || !c->cfg_im_active) {
		rc = cfg_import_begin(c->w.cfg, &c->cfg_im, strict);
		if (rc != 0) {
			mp_fail(c, mp_map_errno(rc), "import");
			return mp_map_errno(rc);
		}
		c->cfg_im_active = true;
	}

	if (p_str(p, params, "data", b64, sizeof(b64)) <= 0) {
		mp_fail(c, MP_E_BAD_PARAMS, "data");
		return MP_E_BAD_PARAMS;
	}
	n = mp_b64_decode(b64, strlen(b64), raw, sizeof(raw));
	if (n < 0) {
		mp_fail(c, MP_E_BAD_PARAMS, "base64");
		return MP_E_BAD_PARAMS;
	}

	rc = cfg_import_feed(c->w.cfg, &c->cfg_im, raw, (size_t)n, &consumed);
	if (rc < 0) {
		c->cfg_im_active = false;
		mp_fail(c, mp_map_errno(rc), "import");
		return mp_map_errno(rc);
	}

	(void)memset(&res, 0, sizeof(res));
	if (rc == 1) {
		int frc;

		if (c->w.cfg_commit != NULL) {
			/* Reuse the glue's commit so the appliers run; the
			 * import's own finish() would call cfg_commit()
			 * directly and no subsystem would be told. */
			frc = c->w.cfg_commit(c->w.cfg_user, &res);
		} else {
			frc = cfg_import_finish(c->w.cfg, &c->cfg_im, &res);
		}
		c->cfg_im_active = false;

		(void)mp_jw_obj_open(w);
		(void)mp_jw_kv_u64(w, "consumed", consumed);
		(void)mp_jw_kv_bool(w, "done", true);
		(void)mp_jw_kv_u64(w, "applied", res.applied);
		(void)mp_jw_kv_u64(w, "skipped", c->cfg_im.skipped);
		(void)mp_jw_kv_u64(w, "persist_errors", res.persist_errors);
		(void)mp_jw_kv_bool(w, "ok", (frc == 0) || (frc == -EIO));
		(void)mp_jw_obj_close(w);

		if ((frc != 0) && (frc != -EIO)) {
			mp_fail(c, mp_map_errno(frc), "commit");
			return mp_map_errno(frc);
		}
		return 0;
	}

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "consumed", consumed);
	(void)mp_jw_kv_u64(w, "offset", c->cfg_im.offset);
	(void)mp_jw_kv_bool(w, "done", false);
	(void)mp_jw_obj_close(w);
	return 0;
}

/* ---------------------------------------------------------------- log.fetch */

static int m_log_fetch(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	logr_rec_t recs[8];
	logr_filter_t f;
	uint32_t cursor = 0U;
	uint32_t max = 8U;
	uint32_t level = LOGR_DEBUG;
	uint16_t n = 0U;
	uint32_t next = 0U;
	uint32_t gap = 0U;
	uint16_t i;
	int rc;

	if (c->w.log == NULL) {
		mp_fail(c, MP_E_NOTSUP, "log");
		return MP_E_NOTSUP;
	}
	(void)p_u32(p, params, "cursor", 0U, &cursor);
	(void)p_u32(p, params, "max", 8U, &max);
	(void)p_u32(p, params, "level", (uint32_t)LOGR_DEBUG, &level);

	if (max > (sizeof(recs) / sizeof(recs[0]))) {
		max = (uint32_t)(sizeof(recs) / sizeof(recs[0]));
	}
	if (level >= LOGR_LEVEL_COUNT) {
		mp_fail(c, MP_E_RANGE, "level");
		return MP_E_RANGE;
	}

	logr_filter_all(&f);
	f.max_level = (uint8_t)level;
	{
		/* `subs` is a bit per logr_sub_t; 0 means every subsystem. */
		uint32_t subs = 0U;

		if (p_u32(p, params, "subs", 0U, &subs) != 0) {
			mp_fail(c, MP_E_BAD_PARAMS, "subs");
			return MP_E_BAD_PARAMS;
		}
		f.sub_mask = (uint16_t)subs;
	}

	rc = logr_tail(c->w.log, cursor, &f, recs, (uint16_t)max, &n, &next,
		       &gap);
	if (rc != 0) {
		mp_fail(c, mp_map_errno(rc), "tail");
		return mp_map_errno(rc);
	}

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "cursor", next);
	(void)mp_jw_kv_u64(w, "gap", gap);
	(void)mp_jw_kv_u64(w, "count", n);
	(void)mp_jw_kv_u64(w, "head", logr_head(c->w.log));
	(void)mp_jw_kv_u64(w, "dropped", logr_dropped(c->w.log));
	(void)mp_jw_key(w, "records");
	(void)mp_jw_arr_open(w);
	for (i = 0U; i < n; i++) {
		size_t tl = recs[i].len;

		if (tl > LOGR_MSG_MAX) {
			tl = LOGR_MSG_MAX;
		}
		(void)mp_jw_arr_open(w);
		(void)mp_jw_u64(w, recs[i].seq);
		(void)mp_jw_u64(w, recs[i].mono_ms);
		(void)mp_jw_u64(w, recs[i].level);
		(void)mp_jw_u64(w, recs[i].subsys);
		(void)mp_jw_strn(w, recs[i].msg, tl);
		(void)mp_jw_arr_close(w);
	}
	(void)mp_jw_arr_close(w);
	(void)mp_jw_obj_close(w);
	return 0;
}

/* ------------------------------------------------------------------- system */

static int m_sys_reboot(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	uint32_t mode = 0U;
	int g;

	if (c->w.img == NULL) {
		mp_fail(c, MP_E_NOTSUP, "image port");
		return MP_E_NOTSUP;
	}
	if (p_u32(p, params, "mode", 0U, &mode) != 0) {
		mp_fail(c, MP_E_BAD_PARAMS, "mode");
		return MP_E_BAD_PARAMS;
	}
	if (mode > 2U) {
		mp_fail(c, MP_E_RANGE, "mode");
		return MP_E_RANGE;
	}

	/* Tag 200 + mode keeps the G3 arm bound to the specific reboot mode, so
	 * an arm for "normal" cannot be completed as "stay in bootloader". */
	g = guard_or_fail(c, (uint8_t)MP_GUARD_G3, p, params, 0U,
			  (uint8_t)(200U + mode), w);
	if (g != 0) {
		return (g > 0) ? 0 : c->err_code;
	}

	c->pending_reboot = (uint8_t)(mode + 1U);

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_bool(w, "rebooting", true);
	(void)mp_jw_kv_u64(w, "mode", mode);
	(void)mp_jw_obj_close(w);
	return 0;
}

static int m_sys_bootloader(mp_ctx_t *c, const mp_json_t *p, int params,
			    mp_jw_t *w)
{
	int g;

	if (c->w.img == NULL) {
		mp_fail(c, MP_E_NOTSUP, "image port");
		return MP_E_NOTSUP;
	}
	g = guard_or_fail(c, (uint8_t)MP_GUARD_G3, p, params, 0U, 210U, w);
	if (g != 0) {
		return (g > 0) ? 0 : c->err_code;
	}

	c->pending_reboot = 2U; /* mode 1 = stay in serial recovery */

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_bool(w, "rebooting", true);
	(void)mp_jw_kv_str(w, "target", "bootloader");
	(void)mp_jw_obj_close(w);
	return 0;
}

/**
 * sys.mode — documented extension.
 *
 * `mp exit` in FMT §2.3 is a shell command, and the shell is not reading the
 * port while MP owns it. This is the in-band equivalent for a host that cannot
 * assert BREAK and would rather not send the raw exit magic.
 */
static int m_sys_mode(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	char mode[16];

	if (p_str(p, params, "mode", mode, sizeof(mode)) <= 0) {
		mp_fail(c, MP_E_BAD_PARAMS, "mode");
		return MP_E_BAD_PARAMS;
	}
	if (strcmp(mode, "shell") != 0) {
		mp_fail(c, MP_E_BAD_PARAMS, "mode");
		return MP_E_BAD_PARAMS;
	}

	c->pending_exit = true;

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_str(w, "mode", "shell");
	(void)mp_jw_kv_bool(w, "reverted", mp_ovr_active(&c->ovr) > 0U);
	(void)mp_jw_obj_close(w);
	return 0;
}

static int m_sys_status(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	(void)p;
	(void)params;

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "requests", c->requests);
	(void)mp_jw_kv_u64(w, "errors", c->rpc_errors);
	(void)mp_jw_kv_u64(w, "records", c->records_sent);
	(void)mp_jw_kv_u64(w, "tx_errors", c->tx_errors);
	(void)mp_jw_kv_u64(w, "overrides", mp_ovr_active(&c->ovr));
	(void)mp_jw_kv_u64(w, "grants", c->ovr.grants);
	(void)mp_jw_kv_u64(w, "vetoes", c->ovr.vetoes);
	(void)mp_jw_kv_u64(w, "deadman_reverts", c->ovr.deadman_reverts);
	(void)mp_jw_kv_u64(w, "verify_failures", c->ovr.verify_failures);
	(void)mp_jw_kv_u64(w, "refusals", c->ovr.refusals);
	(void)mp_jw_kv_u64(w, "late_ticks", c->ovr.late_ticks);

	(void)mp_jw_key(w, "frames");
	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "rx", c->rx.frames);
	(void)mp_jw_kv_u64(w, "msgs", c->rx.msgs);
	(void)mp_jw_kv_u64(w, "crc_errors", c->rx.crc_errors);
	(void)mp_jw_kv_u64(w, "cobs_errors", c->rx.cobs_errors);
	(void)mp_jw_kv_u64(w, "short", c->rx.short_frames);
	(void)mp_jw_kv_u64(w, "bad_channel", c->rx.bad_channel);
	(void)mp_jw_kv_u64(w, "bad_flags", c->rx.bad_flags);
	(void)mp_jw_kv_u64(w, "reasm_drops", c->rx.reasm_drops);
	(void)mp_jw_obj_close(w);

	(void)mp_jw_key(w, "events");
	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "queued", mp_stream_event_count(&c->st));
	(void)mp_jw_kv_u64(w, "dropped", mp_stream_event_dropped(&c->st));
	(void)mp_jw_obj_close(w);

	(void)mp_jw_key(w, "mirror");
	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "frames", c->mirror.frames);
	(void)mp_jw_kv_u64(w, "keyframes", c->mirror.keyframes);
	(void)mp_jw_kv_u64(w, "rows", c->mirror.rows_sent);
	(void)mp_jw_kv_u64(w, "partials", c->mirror.partials);
	(void)mp_jw_obj_close(w);

	(void)mp_jw_obj_close(w);
	return 0;
}

/* --------------------------------------------------------------- time.get */

static int m_time_get(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	mp_telem_t t;
	uint64_t tai_ns = 0U;
	bool fallback = true;

	(void)p;
	(void)params;

	if (c->w.time_get != NULL) {
		if (c->w.time_get(c->w.state_user, &tai_ns, &fallback) != 0) {
			mp_fail(c, MP_E_IO, "time");
			return MP_E_IO;
		}
	}

	(void)memset(&t, 0, sizeof(t));
	if (c->w.telem != NULL) {
		(void)c->w.telem(c->w.state_user, &t);
	}

	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "tai_ns", tai_ns);
	(void)mp_jw_kv_bool(w, "traceable", !fallback);
	(void)mp_jw_kv_u64(w, "mono_ms", mp_now(c));
	(void)mp_jw_kv_u64(w, "stratum", t.q.stratum);
	(void)mp_jw_kv_str(w, "lock", quality_lock_state_name(t.q.lock_state));
	(void)mp_jw_kv_str(w, "ref", quality_ref_name(t.q.active_ref));
	(void)mp_jw_kv_bool(w, "utc_valid", t.q.utc_valid);
	(void)mp_jw_key(w, "leap");
	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_i64(w, "pending", t.q.leap_pending);
	(void)mp_jw_kv_i64(w, "current_s", t.q.leap_current_s);
	(void)mp_jw_kv_u64(w, "at_tai_s", t.q.leap_at_tai_s);
	(void)mp_jw_obj_close(w);
	(void)mp_jw_obj_close(w);
	return 0;
}

/* -------------------------------------------------------------- mirror.get */

static int m_mirror_get(mp_ctx_t *c, const mp_json_t *p, int params, mp_jw_t *w)
{
	mp_mirror_in_t in;
	uint32_t seq;
	int n;
	int rc;

	(void)p;
	(void)params;

	if ((c->w.mirror == NULL) || (c->w.mirror_prev_ch == NULL)) {
		mp_fail(c, MP_E_NOTSUP, "mirror");
		return MP_E_NOTSUP;
	}

	(void)memset(&in, 0, sizeof(in));
	rc = c->w.mirror(c->w.state_user, &in);
	if (rc != 0) {
		mp_fail(c, mp_map_errno(rc), "mirror");
		return mp_map_errno(rc);
	}

	seq = mp_stream_next_seq(&c->st, MP_CH_MIRROR);
	n = mp_mirror_encode(&c->mirror, &in, seq, mp_now(c), true,
			     c->w.scratch, c->w.scratch_len);
	if (n < 0) {
		mp_fail(c, mp_map_errno(n), "encode");
		return mp_map_errno(n);
	}
	rc = mp_send(c, MP_CH_MIRROR, c->w.scratch, (size_t)n);
	if (rc != 0) {
		mp_fail(c, MP_E_IO, "tx");
		return MP_E_IO;
	}

	/*
	 * The keyframe itself goes out on channel 0x0A rather than inline: a full
	 * 60x20 keyframe base64s to over 4 KB, which no bounded JSON reply can
	 * carry. The reply is the receipt.
	 */
	(void)mp_jw_obj_open(w);
	(void)mp_jw_kv_u64(w, "channel", MP_CH_MIRROR);
	(void)mp_jw_kv_u64(w, "record", MP_REC_MIRROR);
	(void)mp_jw_kv_u64(w, "ver", MP_MIRROR_VER);
	(void)mp_jw_kv_u64(w, "seq", seq);
	(void)mp_jw_kv_u64(w, "bytes", (uint64_t)n);
	(void)mp_jw_kv_bool(w, "keyframe", true);
	(void)mp_jw_obj_close(w);
	return 0;
}

/* ------------------------------------------------------------ method table */

static const struct {
	const char *name;
	handler_fn fn;
} methods[] = {
	{ "hello", m_hello },
	{ "manifest.get", m_manifest_get },
	{ "session.open", m_session_open },
	{ "session.keepalive", m_session_keepalive },
	{ "session.close", m_session_close },
	{ "obj.get", m_obj_get },
	{ "obj.set", m_obj_set },
	{ "obj.override", m_obj_override },
	{ "obj.pulse", m_obj_pulse },
	{ "stream.sub", m_stream_sub },
	{ "stream.unsub", m_stream_unsub },
	{ "diag.list", m_diag_list },
	{ "diag.run", m_diag_run },
	{ "diag.abort", m_diag_abort },
	{ "diag.snapshot", m_diag_snapshot },
	{ "cal.list", m_cal_list },
	{ "cal.get", m_cal_get },
	{ "cal.set", m_cal_set },
	{ "cal.commit", m_cal_commit },
	{ "cal.revert", m_cal_revert },
	{ "cfg.export", m_cfg_export },
	{ "cfg.import", m_cfg_import },
	{ "log.fetch", m_log_fetch },
	{ "sys.reboot", m_sys_reboot },
	{ "sys.bootloader", m_sys_bootloader },
	{ "sys.mode", m_sys_mode },
	{ "sys.status", m_sys_status },
	{ "time.get", m_time_get },
	{ "mirror.get", m_mirror_get },
};

#define METHOD_COUNT (sizeof(methods) / sizeof(methods[0]))

/* ===================================================== request dispatch === */

/** Write `"id":<token>` verbatim, or `"id":null`. */
static void emit_id(mp_jw_t *w, const mp_json_t *p, int id_tok)
{
	const mp_json_tok_t *t = mp_json_at(p, id_tok);

	(void)mp_jw_key(w, "id");
	if (t == NULL) {
		(void)mp_jw_null(w);
		return;
	}
	if (t->type == (uint8_t)MP_J_STR) {
		/* Re-quote the raw span: it is already escaped correctly and
		 * re-escaping an unescaped copy could change it. */
		(void)mp_jw_str_escaped(w, &p->src[t->start],
					(size_t)(t->end - t->start));
		return;
	}
	(void)mp_jw_raw(w, &p->src[t->start], (size_t)(t->end - t->start));
}

/** Build a standalone error response with a null id (parse-level failures). */
static int reply_bare_error(mp_ctx_t *c, int code, const char *data,
			    const char **out, size_t *out_len)
{
	mp_jw_t w;
	size_t len = 0U;

	(void)mp_jw_init(&w, c->reply, sizeof(c->reply));
	(void)mp_jw_obj_open(&w);
	(void)mp_jw_kv_str(&w, "jsonrpc", "2.0");
	(void)mp_jw_kv_null(&w, "id");
	(void)mp_jw_key(&w, "error");
	(void)mp_jw_obj_open(&w);
	(void)mp_jw_kv_i64(&w, "code", code);
	(void)mp_jw_kv_str(&w, "message", mp_err_msg(code));
	if (data != NULL) {
		(void)mp_jw_key(&w, "data");
		(void)mp_jw_obj_open(&w);
		(void)mp_jw_kv_str(&w, "reason", data);
		(void)mp_jw_obj_close(&w);
	}
	(void)mp_jw_obj_close(&w);
	(void)mp_jw_obj_close(&w);

	if (mp_jw_finish(&w, &len) != 0) {
		return -ENOSPC;
	}
	c->rpc_errors++;
	if (out != NULL) {
		*out = c->reply;
	}
	if (out_len != NULL) {
		*out_len = len;
	}
	return 0;
}

int mp_rpc_handle(mp_ctx_t *c, const uint8_t *msg, size_t len, const char **out,
		  size_t *out_len)
{
	mp_json_t p;
	mp_jw_t w;
	int root;
	int id_tok;
	int method_tok;
	int params;
	size_t i;
	size_t rlen = 0U;
	int rc;
	int hrc;
	bool notification;

	if ((c == NULL) || ((msg == NULL) && (len != 0U))) {
		return -EINVAL;
	}
	if (out != NULL) {
		*out = NULL;
	}
	if (out_len != NULL) {
		*out_len = 0U;
	}

	c->requests++;
	c->err_code = 0;
	c->err_msg = NULL;
	c->err_data[0] = '\0';

	rc = mp_json_parse(&p, (const char *)msg, len, c->tok, MP_RPC_TOKENS,
			   (uint8_t)MP_JSON_DEPTH_MAX);
	if (rc < 0) {
		return reply_bare_error(c, MP_E_PARSE, NULL, out, out_len);
	}

	root = mp_json_root(&p);
	if ((root < 0) || (mp_json_at(&p, root)->type != (uint8_t)MP_J_OBJ)) {
		/* An array here is a batch, which this device does not accept
		 * (see the file header). */
		return reply_bare_error(c, MP_E_INVALID_REQ, "object required",
					out, out_len);
	}

	/* jsonrpc must be exactly "2.0" (JSON-RPC 2.0 §4). */
	{
		int v = mp_json_obj_get(&p, root, "jsonrpc");

		if ((v < 0) || !mp_json_streq(&p, v, "2.0")) {
			return reply_bare_error(c, MP_E_INVALID_REQ, "jsonrpc",
						out, out_len);
		}
	}

	method_tok = mp_json_obj_get(&p, root, "method");
	if ((method_tok < 0) ||
	    (mp_json_at(&p, method_tok)->type != (uint8_t)MP_J_STR)) {
		return reply_bare_error(c, MP_E_INVALID_REQ, "method", out,
					out_len);
	}

	id_tok = mp_json_obj_get(&p, root, "id");
	notification = (id_tok < 0);
	if (!notification) {
		uint8_t t = mp_json_at(&p, id_tok)->type;

		if ((t != (uint8_t)MP_J_NUM) && (t != (uint8_t)MP_J_STR) &&
		    (t != (uint8_t)MP_J_NULL)) {
			return reply_bare_error(c, MP_E_INVALID_REQ, "id", out,
						out_len);
		}
	}

	params = mp_json_obj_get(&p, root, "params");
	if (params >= 0) {
		uint8_t t = mp_json_at(&p, params)->type;

		if (t != (uint8_t)MP_J_OBJ) {
			/* Positional params are legal JSON-RPC but every method
			 * here is named-only; refusing is clearer than silently
			 * ignoring an array. */
			if (!notification) {
				(void)mp_jw_init(&w, c->reply,
						 sizeof(c->reply));
				(void)mp_jw_obj_open(&w);
				(void)mp_jw_kv_str(&w, "jsonrpc", "2.0");
				emit_id(&w, &p, id_tok);
				(void)mp_jw_key(&w, "error");
				(void)mp_jw_obj_open(&w);
				(void)mp_jw_kv_i64(&w, "code", MP_E_BAD_PARAMS);
				(void)mp_jw_kv_str(&w, "message",
						   mp_err_msg(MP_E_BAD_PARAMS));
				(void)mp_jw_kv_str(&w, "data",
						   "params must be an object");
				(void)mp_jw_obj_close(&w);
				(void)mp_jw_obj_close(&w);
				if (mp_jw_finish(&w, &rlen) == 0) {
					c->rpc_errors++;
					if (out != NULL) {
						*out = c->reply;
					}
					if (out_len != NULL) {
						*out_len = rlen;
					}
				}
			}
			return 0;
		}
	}

	/* Build the response envelope, then let the handler write `result`. */
	(void)mp_jw_init(&w, c->reply, sizeof(c->reply));
	(void)mp_jw_obj_open(&w);
	(void)mp_jw_kv_str(&w, "jsonrpc", "2.0");
	if (!notification) {
		emit_id(&w, &p, id_tok);
	}

	hrc = MP_E_NO_METHOD;
	for (i = 0U; i < METHOD_COUNT; i++) {
		if (mp_json_streq(&p, method_tok, methods[i].name)) {
			(void)mp_jw_key(&w, "result");
			hrc = methods[i].fn(c, &p, params, &w);
			break;
		}
	}

	if (i >= METHOD_COUNT) {
		mp_fail(c, MP_E_NO_METHOD, NULL);
	}

	if (hrc != 0) {
		/*
		 * A handler failed after `"result":` was opened, so the envelope
		 * has to be rebuilt as an error object. Restart the writer
		 * rather than trying to unwind it.
		 */
		int code = (c->err_code != 0) ? c->err_code : hrc;

		(void)mp_jw_init(&w, c->reply, sizeof(c->reply));
		(void)mp_jw_obj_open(&w);
		(void)mp_jw_kv_str(&w, "jsonrpc", "2.0");
		if (!notification) {
			emit_id(&w, &p, id_tok);
		}
		(void)mp_jw_key(&w, "error");
		(void)mp_jw_obj_open(&w);
		(void)mp_jw_kv_i64(&w, "code", code);
		(void)mp_jw_kv_str(&w, "message", mp_err_msg(code));
		(void)mp_jw_key(&w, "data");
		(void)mp_jw_obj_open(&w);
		(void)mp_jw_kv_str(&w, "reason",
				   (c->err_data[0] != '\0') ? c->err_data
							    : NULL);
		(void)mp_jw_kv_str(&w, "method", NULL);
		(void)mp_jw_obj_close(&w);
		(void)mp_jw_obj_close(&w);
		c->rpc_errors++;
	}

	(void)mp_jw_obj_close(&w);

	if (notification) {
		c->notifications++;
		return 0;
	}

	if (mp_jw_finish(&w, &rlen) != 0) {
		/* The reply did not fit. Answering with an internal error is the
		 * only honest outcome: a truncated JSON document is worse than
		 * an error the tool can retry against a narrower request. */
		return reply_bare_error(c, MP_E_INTERNAL, "reply overflow", out,
					out_len);
	}

	c->replies++;
	if (out != NULL) {
		*out = c->reply;
	}
	if (out_len != NULL) {
		*out_len = rlen;
	}
	return 0;
}

/* ================================================================ lifecycle */

/** Frame-layer callback: a complete inbound message on some channel. */
static int on_msg(void *user, uint8_t ch, const uint8_t *msg, size_t len)
{
	mp_ctx_t *c = (mp_ctx_t *)user;
	const char *reply = NULL;
	size_t rlen = 0U;

	if (ch != MP_CH_CONTROL) {
		/*
		 * Inbound bytes on a tunnel channel are the host writing to a
		 * peripheral. Core does not own those UARTs, so the glue
		 * registers its own handler by wrapping this one; unhandled
		 * channels are counted and dropped rather than guessed at.
		 */
		return -ENOTSUP;
	}

	if (mp_rpc_handle(c, msg, len, &reply, &rlen) != 0) {
		return -EIO;
	}
	if ((reply != NULL) && (rlen != 0U)) {
		return mp_send(c, MP_CH_CONTROL, (const uint8_t *)reply, rlen);
	}
	return 0;
}

/** Override-engine event sink: mirror it onto the event channel. */
static void ovr_evt(void *user, uint8_t ev, size_t obj, uint32_t sid,
		    const char *reason)
{
	mp_ctx_t *c = (mp_ctx_t *)user;
	const mp_obj_t *o = mp_obj_at(obj);

	(void)sid;
	(void)mp_stream_eventf(&c->st, (uint8_t)MP_EV_OVERRIDE, ev,
			       (uint16_t)obj, 1U, 0, mp_now(c),
			       (reason != NULL) ? reason
						: ((o != NULL) ? o->id : NULL));

	if (c->w.log != NULL) {
		/* §5.3 requires the reversion to be logged, not just streamed. */
		(void)logr_puts(c->w.log,
				(ev == (uint8_t)MP_OVR_EV_GRANT)
					? (uint8_t)LOGR_NOTICE
					: (uint8_t)LOGR_WARN,
				(uint8_t)LOGR_SUB_MCP, mp_now(c),
				(o != NULL) ? o->id : "override");
	}
}

/** Diagnostic event sink: onto the event channel. */
static void diag_evt(void *user, uint8_t ev, uint8_t test, uint8_t step,
		     uint8_t verdict, int32_t value, const char *text)
{
	mp_ctx_t *c = (mp_ctx_t *)user;

	(void)mp_stream_eventf(&c->st, (uint8_t)MP_EV_DIAG, ev,
			       (uint16_t)(((uint16_t)test << 8) | step),
			       verdict, value, mp_now(c), text);
}

int mp_init(mp_ctx_t *c, const mp_wiring_t *w)
{
	int rc;

	if ((c == NULL) || (w == NULL)) {
		return -EINVAL;
	}
	if ((w->tx == NULL) || (w->mono_ms == NULL) || (w->apply == NULL)) {
		return -EINVAL;
	}
	if ((w->scratch == NULL) || (w->scratch_len < MP_SCRATCH_MIN)) {
		return -ENOMEM;
	}

	(void)memset(c, 0, sizeof(*c));
	c->w = *w;
	c->mode = (uint8_t)MP_MODE_SHELL;

	rc = mp_frame_rx_init(&c->rx, w->reasm, w->reasm_n, on_msg, c);
	if (rc != 0) {
		return rc;
	}
	rc = mp_ovr_init(&c->ovr, w->apply, w->apply_user, ovr_evt, c,
			 w->serial);
	if (rc != 0) {
		return rc;
	}
	rc = mp_stream_init(&c->st);
	if (rc != 0) {
		return rc;
	}
	if (w->diag != NULL) {
		rc = mp_diag_init(&c->diag, w->diag, w->diag_user, diag_evt, c);
		if (rc != 0) {
			return rc;
		}
	}
	rc = mp_mirror_init(&c->mirror, w->mirror_prev_ch, w->mirror_prev_attr,
			    w->mirror_cells);
	if (rc != 0) {
		return rc;
	}

	/* Lend the reply buffer as the serialisation scratch, so the hash
	 * costs no stack of its own. */
	c->manifest_hash = mp_manifest_hash(c->reply, sizeof(c->reply));
	return 0;
}

uint8_t mp_mode(const mp_ctx_t *c)
{
	return (c != NULL) ? c->mode : (uint8_t)MP_MODE_SHELL;
}

uint32_t mp_manifest_hash_cached(const mp_ctx_t *c)
{
	return (c != NULL) ? c->manifest_hash : 0U;
}

int mp_mode_enter(mp_ctx_t *c)
{
	if (c == NULL) {
		return -EINVAL;
	}
	if (c->mode == (uint8_t)MP_MODE_MP) {
		return 0;
	}
	c->mode = (uint8_t)MP_MODE_MP;
	c->magic_n = 0U;
	mp_frame_rx_reset(&c->rx);
	mp_mirror_reset(&c->mirror);
	c->mode_enters++;
	(void)mp_stream_eventf(&c->st, (uint8_t)MP_EV_MODE, 0U, 0U, 1U, 0,
			       mp_now(c), "mp");
	return 0;
}

int mp_mode_exit(mp_ctx_t *c)
{
	if (c == NULL) {
		return -EINVAL;
	}
	if (c->mode == (uint8_t)MP_MODE_SHELL) {
		return 0;
	}

	/* Leaving MP mode must not leave the board commanded. */
	(void)mp_ovr_revert_all(&c->ovr, (uint8_t)MP_OVR_EV_RELEASE,
				"left MP mode", mp_now(c));
	if (c->ovr.sess.id != 0U) {
		(void)mp_ovr_session_close(&c->ovr, c->ovr.sess.id, mp_now(c));
	}
	mp_stream_unsub_all(&c->st);
	if (mp_diag_busy(&c->diag)) {
		(void)mp_diag_abort(&c->diag, mp_now(c));
	}
	c->cfg_ex_active = false;
	c->cfg_im_active = false;
	c->pending_exit = false;

	c->mode = (uint8_t)MP_MODE_SHELL;
	c->magic_n = 0U;
	mp_frame_rx_reset(&c->rx);
	c->mode_exits++;
	return 0;
}

int mp_set_link(mp_ctx_t *c, bool up)
{
	if (c == NULL) {
		return -EINVAL;
	}
	(void)mp_ovr_set_link(&c->ovr, up, mp_now(c));
	if (!up && (c->mode == (uint8_t)MP_MODE_MP)) {
		(void)mp_mode_exit(c);
	}
	return 0;
}

/* ------------------------------------------------------------------- input */

/** Run one byte through a magic matcher. Returns true on a complete match. */
static bool magic_byte(uint8_t *n, const char *magic, uint8_t b)
{
	if (b == (uint8_t)magic[*n]) {
		(*n)++;
		if (*n >= MP_MAGIC_LEN) {
			*n = 0U;
			return true;
		}
		return false;
	}
	/*
	 * Restart, but allow this byte to begin a fresh match — otherwise
	 * "\x01\x01MP1\x02" would not be recognised.
	 */
	*n = (b == (uint8_t)magic[0]) ? 1U : 0U;
	return false;
}

int mp_shell_byte(mp_ctx_t *c, uint8_t b)
{
	if (c == NULL) {
		return -EINVAL;
	}
	if (c->mode == (uint8_t)MP_MODE_MP) {
		return 0;
	}
	if (magic_byte(&c->magic_n, MP_MAGIC_ENTER, b)) {
		(void)mp_mode_enter(c);
		return 1;
	}
	return 0;
}

int mp_input(mp_ctx_t *c, const uint8_t *data, size_t len)
{
	size_t i;
	int msgs = 0;

	if ((c == NULL) || ((data == NULL) && (len != 0U))) {
		return -EINVAL;
	}
	if (c->mode != (uint8_t)MP_MODE_MP) {
		return -EPERM;
	}

	for (i = 0U; i < len; i++) {
		/*
		 * The exit magic is matched on the raw stream, ahead of the
		 * framer: it has to work even when framing has desynchronised,
		 * which is exactly when an operator reaches for it.
		 */
		if (magic_byte(&c->magic_n, MP_MAGIC_EXIT, data[i])) {
			(void)mp_mode_exit(c);
			return msgs;
		}
		if (mp_frame_rx_byte(&c->rx, data[i]) == 1) {
			msgs++;
		}

		/* A handler may have asked to reboot or leave; honour it only
		 * after its reply has been transmitted by on_msg(). */
		if (c->pending_exit) {
			(void)mp_mode_exit(c);
			return msgs;
		}
		if (c->pending_reboot != 0U) {
			uint8_t mode = (uint8_t)(c->pending_reboot - 1U);

			c->pending_reboot = 0U;
			if ((c->w.img != NULL) &&
			    (c->w.img->reboot != NULL)) {
				c->w.img->reboot(c->w.img->ctx, (int)mode);
			}
			return msgs;
		}
	}
	return msgs;
}

/* -------------------------------------------------------------------- tick */

/** Emit the telemetry record if the channel is due. */
static int pump_telem(mp_ctx_t *c)
{
	mp_telem_t t;
	uint8_t buf[MP_TELEM_MAX];
	int n;

	if (c->w.telem == NULL) {
		return 0;
	}
	(void)memset(&t, 0, sizeof(t));
	if (c->w.telem(c->w.state_user, &t) != 0) {
		return 0;
	}
	t.seq = mp_stream_next_seq(&c->st, MP_CH_TELEMETRY);
	t.mono_ms = mp_now(c);

	n = mp_enc_telem(&t, buf, sizeof(buf));
	if (n < 0) {
		return 0;
	}
	return (mp_send(c, MP_CH_TELEMETRY, buf, (size_t)n) == 0) ? 1 : 0;
}

static int pump_pps(mp_ctx_t *c)
{
	mp_pps_t p;
	uint8_t buf[MP_PPS_MAX];
	int n;

	if (c->w.pps == NULL) {
		return 0;
	}
	(void)memset(&p, 0, sizeof(p));
	if (c->w.pps(c->w.state_user, &p) != 0) {
		return 0;
	}
	p.seq = mp_stream_next_seq(&c->st, MP_CH_PPS);
	p.mono_ms = mp_now(c);

	n = mp_enc_pps(&p, buf, sizeof(buf));
	if (n < 0) {
		return 0;
	}
	return (mp_send(c, MP_CH_PPS, buf, (size_t)n) == 0) ? 1 : 0;
}

static int pump_log(mp_ctx_t *c)
{
	logr_rec_t recs[8];
	logr_filter_t f;
	uint16_t n = 0U;
	uint32_t next = 0U;
	uint32_t gap = 0U;
	int enc;

	if (c->w.log == NULL) {
		return 0;
	}
	logr_filter_all(&f);
	if (logr_tail(c->w.log, mp_stream_cursor(&c->st, MP_CH_LOG), &f, recs,
		      (uint16_t)(sizeof(recs) / sizeof(recs[0])), &n, &next,
		      &gap) != 0) {
		return 0;
	}
	if ((n == 0U) && (gap == 0U)) {
		mp_stream_set_cursor(&c->st, MP_CH_LOG, next);
		return 0;
	}

	enc = mp_enc_log(mp_stream_next_seq(&c->st, MP_CH_LOG), mp_now(c), recs,
			 n, next, gap, c->w.scratch, c->w.scratch_len);
	if (enc < 0) {
		return 0;
	}
	if (mp_send(c, MP_CH_LOG, c->w.scratch, (size_t)enc) != 0) {
		return 0;
	}
	mp_stream_set_cursor(&c->st, MP_CH_LOG, next);
	return 1;
}

static int pump_mirror(mp_ctx_t *c)
{
	mp_mirror_in_t in;
	int n;

	if ((c->w.mirror == NULL) || (c->w.mirror_prev_ch == NULL)) {
		return 0;
	}
	(void)memset(&in, 0, sizeof(in));
	if (c->w.mirror(c->w.state_user, &in) != 0) {
		return 0;
	}
	n = mp_mirror_encode(&c->mirror, &in,
			     mp_stream_next_seq(&c->st, MP_CH_MIRROR),
			     mp_now(c), false, c->w.scratch, c->w.scratch_len);
	if (n < 0) {
		return 0;
	}
	return (mp_send(c, MP_CH_MIRROR, c->w.scratch, (size_t)n) == 0) ? 1 : 0;
}

static int pump_events(mp_ctx_t *c)
{
	uint8_t buf[512];
	int n;

	if (!mp_stream_is_sub(&c->st, MP_CH_EVENT)) {
		return 0;
	}
	if (mp_stream_event_count(&c->st) == 0U) {
		return 0;
	}
	n = mp_enc_events(&c->st, mp_stream_next_seq(&c->st, MP_CH_EVENT),
			  mp_now(c), 0U, buf, sizeof(buf));
	if (n <= 0) {
		return 0;
	}
	return (mp_send(c, MP_CH_EVENT, buf, (size_t)n) == 0) ? 1 : 0;
}

int mp_tick(mp_ctx_t *c)
{
	mp_ilk_state_t ilk;
	uint32_t now;
	int sent = 0;

	if (c == NULL) {
		return -EINVAL;
	}
	now = mp_now(c);

	(void)memset(&ilk, 0, sizeof(ilk));
	if (c->w.ilk != NULL) {
		if (c->w.ilk(c->w.state_user, &ilk) != 0) {
			/* No state means a pending VCC_RB verification fails
			 * safe; that is mp_ovr_tick()'s documented behaviour for
			 * a NULL state. */
			(void)mp_ovr_tick(&c->ovr, NULL, now);
		} else {
			(void)mp_ovr_tick(&c->ovr, &ilk, now);
		}
	} else {
		(void)mp_ovr_tick(&c->ovr, NULL, now);
	}

	if (c->mode != (uint8_t)MP_MODE_MP) {
		return 0;
	}

	/* Diagnostics advance one step per tick so a long sequence cannot hold
	 * the console thread. */
	if (mp_diag_busy(&c->diag)) {
		(void)mp_diag_step(&c->diag, now);
	}

	if (mp_stream_due(&c->st, MP_CH_TELEMETRY, now)) {
		sent += pump_telem(c);
	}
	if (mp_stream_due(&c->st, MP_CH_PPS, now)) {
		sent += pump_pps(c);
	}
	if (mp_stream_due(&c->st, MP_CH_LOG, now)) {
		sent += pump_log(c);
	}
	if (mp_stream_due(&c->st, MP_CH_MIRROR, now)) {
		sent += pump_mirror(c);
	}
	sent += pump_events(c);

	if (c->pending_reboot != 0U) {
		uint8_t mode = (uint8_t)(c->pending_reboot - 1U);

		c->pending_reboot = 0U;
		if ((c->w.img != NULL) && (c->w.img->reboot != NULL)) {
			c->w.img->reboot(c->w.img->ctx, (int)mode);
		}
	}
	return sent;
}

/* ------------------------------------------------------------ stream feeds */

int mp_stream_raw(mp_ctx_t *c, uint8_t ch, const uint8_t *data, size_t len)
{
	if ((c == NULL) || ((data == NULL) && (len != 0U))) {
		return -EINVAL;
	}
	switch (ch) {
	case MP_CH_NMEA:
	case MP_CH_UBX:
	case MP_CH_GNSS_PASS:
	case MP_CH_RB_PASS:
	case MP_CH_SMP:
		break;
	default:
		return -EINVAL;
	}
	if (c->mode != (uint8_t)MP_MODE_MP) {
		return -ENOENT;
	}
	/* The SMP tunnel is not a subscription: it is opened by an override on
	 * sys.smp.tunnel and then carries bytes both ways. */
	if ((ch != MP_CH_SMP) && !mp_stream_is_sub(&c->st, ch)) {
		return -ENOENT;
	}
	if (mp_send(c, ch, data, len) != 0) {
		return -EIO;
	}
	return (int)len;
}

int mp_post_event(mp_ctx_t *c, uint8_t kind, uint8_t sub, uint16_t id,
		  uint8_t edge, int32_t value, const char *text)
{
	if (c == NULL) {
		return -EINVAL;
	}
	return mp_stream_eventf(&c->st, kind, sub, id, edge, value, mp_now(c),
				text);
}

int mp_veto(mp_ctx_t *c, size_t obj, const char *reason)
{
	if (c == NULL) {
		return -EINVAL;
	}
	return mp_ovr_veto(&c->ovr, obj, reason, mp_now(c));
}
