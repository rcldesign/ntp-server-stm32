/*
 * STS1000 "Meridian" — core/mp: CBOR writer + stream records (FMT §7, §8.1).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See mp_stream.h for the wire shape and the per-record key tables.
 */

#include "mp/mp_stream.h"

#include <errno.h>
#include <string.h>

/* ========================================================= CBOR writer === */

#define CBOR_MAJOR_UINT 0U
#define CBOR_MAJOR_NINT 1U
#define CBOR_MAJOR_BSTR 2U
#define CBOR_MAJOR_TSTR 3U
#define CBOR_MAJOR_ARR 4U
#define CBOR_MAJOR_MAP 5U
#define CBOR_MAJOR_SIMPLE 7U

int mp_cbor_init(mp_cbor_t *w, uint8_t *buf, size_t cap)
{
	if ((w == NULL) || ((buf == NULL) && (cap != 0U))) {
		return -EINVAL;
	}
	w->buf = buf;
	w->cap = cap;
	w->len = 0U;
	w->err = 0;
	return 0;
}

static void raw(mp_cbor_t *w, const uint8_t *b, size_t n)
{
	if (w->err != 0) {
		return;
	}
	if ((w->len + n) > w->cap) {
		w->err = -ENOSPC;
		return;
	}
	(void)memcpy(&w->buf[w->len], b, n);
	w->len += n;
}

static void byte(mp_cbor_t *w, uint8_t b)
{
	raw(w, &b, 1U);
}

/** Emit a major type with its argument in the shortest legal form. */
static void head(mp_cbor_t *w, uint8_t major, uint64_t arg)
{
	uint8_t m = (uint8_t)(major << 5);

	if (arg < 24U) {
		byte(w, (uint8_t)(m | (uint8_t)arg));
	} else if (arg <= 0xFFU) {
		byte(w, (uint8_t)(m | 24U));
		byte(w, (uint8_t)arg);
	} else if (arg <= 0xFFFFU) {
		byte(w, (uint8_t)(m | 25U));
		byte(w, (uint8_t)(arg >> 8));
		byte(w, (uint8_t)arg);
	} else if (arg <= 0xFFFFFFFFU) {
		byte(w, (uint8_t)(m | 26U));
		byte(w, (uint8_t)(arg >> 24));
		byte(w, (uint8_t)(arg >> 16));
		byte(w, (uint8_t)(arg >> 8));
		byte(w, (uint8_t)arg);
	} else {
		byte(w, (uint8_t)(m | 27U));
		byte(w, (uint8_t)(arg >> 56));
		byte(w, (uint8_t)(arg >> 48));
		byte(w, (uint8_t)(arg >> 40));
		byte(w, (uint8_t)(arg >> 32));
		byte(w, (uint8_t)(arg >> 24));
		byte(w, (uint8_t)(arg >> 16));
		byte(w, (uint8_t)(arg >> 8));
		byte(w, (uint8_t)arg);
	}
}

int mp_cbor_uint(mp_cbor_t *w, uint64_t v)
{
	if (w == NULL) {
		return -EINVAL;
	}
	head(w, CBOR_MAJOR_UINT, v);
	return w->err;
}

int mp_cbor_int(mp_cbor_t *w, int64_t v)
{
	if (w == NULL) {
		return -EINVAL;
	}
	if (v < 0) {
		/* RFC 8949 §3.1: a negative integer encodes -1 - n. */
		uint64_t n = (v == (int64_t)(-0x7FFFFFFFFFFFFFFFLL) - 1LL)
				     ? (uint64_t)0x7FFFFFFFFFFFFFFFULL
				     : (uint64_t)(-(v + 1));

		head(w, CBOR_MAJOR_NINT, n);
	} else {
		head(w, CBOR_MAJOR_UINT, (uint64_t)v);
	}
	return w->err;
}

int mp_cbor_bytes(mp_cbor_t *w, const uint8_t *b, size_t n)
{
	if (w == NULL) {
		return -EINVAL;
	}
	if ((b == NULL) && (n != 0U)) {
		return -EINVAL;
	}
	head(w, CBOR_MAJOR_BSTR, (uint64_t)n);
	raw(w, b, n);
	return w->err;
}

int mp_cbor_textn(mp_cbor_t *w, const char *s, size_t n)
{
	if (w == NULL) {
		return -EINVAL;
	}
	if ((s == NULL) && (n != 0U)) {
		return -EINVAL;
	}
	head(w, CBOR_MAJOR_TSTR, (uint64_t)n);
	raw(w, (const uint8_t *)s, n);
	return w->err;
}

int mp_cbor_text(mp_cbor_t *w, const char *s)
{
	if (s == NULL) {
		return mp_cbor_null(w);
	}
	return mp_cbor_textn(w, s, strlen(s));
}

int mp_cbor_arr(mp_cbor_t *w, size_t n)
{
	if (w == NULL) {
		return -EINVAL;
	}
	head(w, CBOR_MAJOR_ARR, (uint64_t)n);
	return w->err;
}

int mp_cbor_map(mp_cbor_t *w, size_t n)
{
	if (w == NULL) {
		return -EINVAL;
	}
	head(w, CBOR_MAJOR_MAP, (uint64_t)n);
	return w->err;
}

int mp_cbor_bool(mp_cbor_t *w, bool v)
{
	if (w == NULL) {
		return -EINVAL;
	}
	byte(w, v ? 0xF5U : 0xF4U);
	return w->err;
}

int mp_cbor_null(mp_cbor_t *w)
{
	if (w == NULL) {
		return -EINVAL;
	}
	byte(w, 0xF6U);
	return w->err;
}

int mp_cbor_f32(mp_cbor_t *w, float v)
{
	uint32_t bits;

	if (w == NULL) {
		return -EINVAL;
	}
	(void)memcpy(&bits, &v, sizeof(bits));
	byte(w, (uint8_t)((CBOR_MAJOR_SIMPLE << 5) | 26U));
	byte(w, (uint8_t)(bits >> 24));
	byte(w, (uint8_t)(bits >> 16));
	byte(w, (uint8_t)(bits >> 8));
	byte(w, (uint8_t)bits);
	return w->err;
}

int mp_cbor_kv_uint(mp_cbor_t *w, uint64_t key, uint64_t v)
{
	(void)mp_cbor_uint(w, key);
	return mp_cbor_uint(w, v);
}

int mp_cbor_kv_int(mp_cbor_t *w, uint64_t key, int64_t v)
{
	(void)mp_cbor_uint(w, key);
	return mp_cbor_int(w, v);
}

int mp_cbor_kv_bool(mp_cbor_t *w, uint64_t key, bool v)
{
	(void)mp_cbor_uint(w, key);
	return mp_cbor_bool(w, v);
}

int mp_cbor_kv_f32(mp_cbor_t *w, uint64_t key, float v)
{
	(void)mp_cbor_uint(w, key);
	return mp_cbor_f32(w, v);
}

int mp_cbor_kv_text(mp_cbor_t *w, uint64_t key, const char *s)
{
	(void)mp_cbor_uint(w, key);
	return mp_cbor_text(w, s);
}

int mp_cbor_kv_bytes(mp_cbor_t *w, uint64_t key, const uint8_t *b, size_t n)
{
	(void)mp_cbor_uint(w, key);
	if (b == NULL) {
		return mp_cbor_null(w);
	}
	return mp_cbor_bytes(w, b, n);
}

int mp_cbor_finish(const mp_cbor_t *w, size_t *out_len)
{
	if (w == NULL) {
		return -EINVAL;
	}
	if (w->err != 0) {
		return w->err;
	}
	if (out_len != NULL) {
		*out_len = w->len;
	}
	return 0;
}

/* ------------------------------------------------------------ CBOR reader */

int mp_cbor_rd_init(mp_cbor_rd_t *r, const uint8_t *buf, size_t len)
{
	if ((r == NULL) || ((buf == NULL) && (len != 0U))) {
		return -EINVAL;
	}
	r->buf = buf;
	r->len = len;
	r->pos = 0U;
	return 0;
}

int mp_cbor_peek_major(const mp_cbor_rd_t *r)
{
	if (r == NULL) {
		return -EINVAL;
	}
	if (r->pos >= r->len) {
		return -ENODATA;
	}
	return (int)(r->buf[r->pos] >> 5);
}

/** Decode one head; leaves the reader after the argument bytes. */
static int rd_head(mp_cbor_rd_t *r, uint8_t *out_major, uint64_t *out_arg)
{
	uint8_t ib;
	uint8_t ai;
	size_t n;
	uint64_t v = 0U;
	size_t k;

	if (r->pos >= r->len) {
		return -ENODATA;
	}
	ib = r->buf[r->pos];
	r->pos++;
	*out_major = (uint8_t)(ib >> 5);
	ai = (uint8_t)(ib & 0x1FU);

	if (ai < 24U) {
		*out_arg = ai;
		return 0;
	}
	switch (ai) {
	case 24U:
		n = 1U;
		break;
	case 25U:
		n = 2U;
		break;
	case 26U:
		n = 4U;
		break;
	case 27U:
		n = 8U;
		break;
	default:
		/* 28..30 are reserved; 31 is indefinite length, which this
		 * writer never emits and this reader will not accept. */
		return -EBADMSG;
	}
	if ((r->pos + n) > r->len) {
		return -EBADMSG;
	}
	for (k = 0U; k < n; k++) {
		v = (v << 8) | (uint64_t)r->buf[r->pos + k];
	}
	r->pos += n;
	*out_arg = v;
	return 0;
}

int mp_cbor_rd_uint(mp_cbor_rd_t *r, uint64_t *out)
{
	uint8_t major;
	uint64_t arg;
	int rc;

	if ((r == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	rc = rd_head(r, &major, &arg);
	if (rc != 0) {
		return rc;
	}
	if (major != CBOR_MAJOR_UINT) {
		return -EBADMSG;
	}
	*out = arg;
	return 0;
}

int mp_cbor_rd_int(mp_cbor_rd_t *r, int64_t *out)
{
	uint8_t major;
	uint64_t arg;
	int rc;

	if ((r == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	rc = rd_head(r, &major, &arg);
	if (rc != 0) {
		return rc;
	}
	if (major == CBOR_MAJOR_UINT) {
		if (arg > (uint64_t)0x7FFFFFFFFFFFFFFFULL) {
			return -ERANGE;
		}
		*out = (int64_t)arg;
		return 0;
	}
	if (major == CBOR_MAJOR_NINT) {
		if (arg > (uint64_t)0x7FFFFFFFFFFFFFFFULL) {
			return -ERANGE;
		}
		*out = -1LL - (int64_t)arg;
		return 0;
	}
	return -EBADMSG;
}

static int rd_container(mp_cbor_rd_t *r, uint8_t want, size_t *out)
{
	uint8_t major;
	uint64_t arg;
	int rc;

	if ((r == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	rc = rd_head(r, &major, &arg);
	if (rc != 0) {
		return rc;
	}
	if (major != want) {
		return -EBADMSG;
	}
	*out = (size_t)arg;
	return 0;
}

int mp_cbor_rd_map(mp_cbor_rd_t *r, size_t *out)
{
	return rd_container(r, CBOR_MAJOR_MAP, out);
}

int mp_cbor_rd_arr(mp_cbor_rd_t *r, size_t *out)
{
	return rd_container(r, CBOR_MAJOR_ARR, out);
}

static int rd_string(mp_cbor_rd_t *r, uint8_t want, const uint8_t **out,
		     size_t *out_n)
{
	uint8_t major;
	uint64_t arg;
	int rc;

	if ((r == NULL) || (out == NULL) || (out_n == NULL)) {
		return -EINVAL;
	}
	rc = rd_head(r, &major, &arg);
	if (rc != 0) {
		return rc;
	}
	if (major != want) {
		return -EBADMSG;
	}
	if (arg > (uint64_t)(r->len - r->pos)) {
		return -EBADMSG;
	}
	*out = &r->buf[r->pos];
	*out_n = (size_t)arg;
	r->pos += (size_t)arg;
	return 0;
}

int mp_cbor_rd_bytes(mp_cbor_rd_t *r, const uint8_t **out, size_t *out_n)
{
	return rd_string(r, CBOR_MAJOR_BSTR, out, out_n);
}

int mp_cbor_rd_text(mp_cbor_rd_t *r, const char **out, size_t *out_n)
{
	const uint8_t *p = NULL;
	int rc = rd_string(r, CBOR_MAJOR_TSTR, &p, out_n);

	if (rc != 0) {
		return rc;
	}
	if (out == NULL) {
		return -EINVAL;
	}
	*out = (const char *)p;
	return 0;
}

int mp_cbor_rd_bool(mp_cbor_rd_t *r, bool *out)
{
	if ((r == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (r->pos >= r->len) {
		return -ENODATA;
	}
	if (r->buf[r->pos] == 0xF5U) {
		*out = true;
	} else if (r->buf[r->pos] == 0xF4U) {
		*out = false;
	} else {
		return -EBADMSG;
	}
	r->pos++;
	return 0;
}

int mp_cbor_rd_f32(mp_cbor_rd_t *r, float *out)
{
	uint32_t bits;

	if ((r == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if ((r->pos + 5U) > r->len) {
		return -ENODATA;
	}
	if (r->buf[r->pos] != (uint8_t)((CBOR_MAJOR_SIMPLE << 5) | 26U)) {
		return -EBADMSG;
	}
	bits = ((uint32_t)r->buf[r->pos + 1U] << 24) |
	       ((uint32_t)r->buf[r->pos + 2U] << 16) |
	       ((uint32_t)r->buf[r->pos + 3U] << 8) |
	       (uint32_t)r->buf[r->pos + 4U];
	(void)memcpy(out, &bits, sizeof(*out));
	r->pos += 5U;
	return 0;
}

/**
 * Deepest nesting mp_cbor_skip() will walk.
 *
 * The skip is recursive, and it is fed attacker-controlled bytes when the host
 * tunnels a record back at us, so the depth is capped rather than trusted. The
 * writer never nests more than three deep (bundle -> array -> array).
 */
#define CBOR_SKIP_DEPTH_MAX 16U

static int cbor_skip_d(mp_cbor_rd_t *r, unsigned int depth)
{
	uint8_t major;
	uint64_t arg;
	int rc;
	size_t k;
	size_t n;

	if (r == NULL) {
		return -EINVAL;
	}
	if (depth > CBOR_SKIP_DEPTH_MAX) {
		return -E2BIG;
	}
	rc = rd_head(r, &major, &arg);
	if (rc != 0) {
		return rc;
	}

	switch (major) {
	case CBOR_MAJOR_UINT:
	case CBOR_MAJOR_NINT:
		return 0;
	case CBOR_MAJOR_BSTR:
	case CBOR_MAJOR_TSTR:
		if (arg > (uint64_t)(r->len - r->pos)) {
			return -EBADMSG;
		}
		r->pos += (size_t)arg;
		return 0;
	case CBOR_MAJOR_ARR:
	case CBOR_MAJOR_MAP:
		/* A declared element count larger than the bytes that remain
		 * cannot be honest; reject before iterating it. */
		if (arg > (uint64_t)(r->len - r->pos)) {
			return -EBADMSG;
		}
		n = (size_t)arg * ((major == CBOR_MAJOR_MAP) ? 2U : 1U);
		for (k = 0U; k < n; k++) {
			rc = cbor_skip_d(r, depth + 1U);
			if (rc != 0) {
				return rc;
			}
		}
		return 0;
	case CBOR_MAJOR_SIMPLE:
		/* The writer emits only false/true/null and binary32. */
		if (arg == 26U) {
			if ((r->pos + 4U) > r->len) {
				return -EBADMSG;
			}
			r->pos += 4U;
		}
		return 0;
	default:
		return -EBADMSG;
	}
}

int mp_cbor_skip(mp_cbor_rd_t *r)
{
	return cbor_skip_d(r, 0U);
}

int mp_cbor_map_find(mp_cbor_rd_t *r, uint64_t key)
{
	size_t pairs = 0U;
	size_t i;
	int rc;

	if (r == NULL) {
		return -EINVAL;
	}
	rc = mp_cbor_rd_map(r, &pairs);
	if (rc != 0) {
		return rc;
	}
	for (i = 0U; i < pairs; i++) {
		uint64_t k = 0U;

		rc = mp_cbor_rd_uint(r, &k);
		if (rc != 0) {
			return rc;
		}
		if (k == key) {
			return 0;
		}
		rc = mp_cbor_skip(r);
		if (rc != 0) {
			return rc;
		}
	}
	return -ENOENT;
}

/* ------------------------------------------------------------ record head */

static void rec_head(mp_cbor_t *w, size_t pairs, uint64_t type, uint64_t ver,
		     uint32_t seq, uint64_t mono_ms)
{
	(void)mp_cbor_map(w, pairs);
	(void)mp_cbor_kv_uint(w, MP_K_TYPE, type);
	(void)mp_cbor_kv_uint(w, MP_K_VER, ver);
	(void)mp_cbor_kv_uint(w, MP_K_SEQ, seq);
	(void)mp_cbor_kv_uint(w, MP_K_MONO, mono_ms);
}

/* ========================================================== telemetry ==== */

/** Pair count of a telemetry record: 4 common + 44 payload keys (8..51). */
#define TELEM_PAIRS (4U + 44U)

int mp_enc_telem(const mp_telem_t *t, uint8_t *buf, size_t cap)
{
	mp_cbor_t w;
	size_t len = 0U;
	size_t i;
	uint8_t valid_flags = 0U;
	uint32_t refs = 0U;
	int rc;

	if ((t == NULL) || (buf == NULL)) {
		return -EINVAL;
	}
	rc = mp_cbor_init(&w, buf, cap);
	if (rc != 0) {
		return rc;
	}

	if (t->h.tmp_osc_valid) {
		valid_flags |= 1U << 0;
	}
	if (t->h.tmp_amb_valid) {
		valid_flags |= 1U << 1;
	}
	if (t->h.die_valid) {
		valid_flags |= 1U << 2;
	}
	if (t->h.humidity_valid) {
		valid_flags |= 1U << 3;
	}

	if (t->rb_lock) {
		refs |= 1U << 0;
	}
	if (t->rb_powered) {
		refs |= 1U << 1;
	}
	if (t->extref_ok) {
		refs |= 1U << 2;
	}
	if (t->pfi) {
		refs |= 1U << 3;
	}

	rec_head(&w, TELEM_PAIRS, MP_REC_TELEM, 1U, t->seq, t->mono_ms);

	(void)mp_cbor_kv_uint(&w, 8U, t->tai_ns);
	(void)mp_cbor_kv_bool(&w, 9U, t->time_fallback);
	(void)mp_cbor_kv_uint(&w, 10U, t->uptime_s);

	(void)mp_cbor_kv_uint(&w, 11U, t->q.stratum);
	(void)mp_cbor_kv_uint(&w, 12U, t->q.lock_state);
	(void)mp_cbor_kv_uint(&w, 13U, t->q.active_ref);
	(void)mp_cbor_kv_uint(&w, 14U, t->q.gnss_fix);
	(void)mp_cbor_kv_uint(&w, 15U, t->q.gnss_sv_used);
	(void)mp_cbor_kv_uint(&w, 16U, t->q.gnss_sv_visible);
	(void)mp_cbor_kv_uint(&w, 17U, t->q.gnss_tacc_ns);
	(void)mp_cbor_kv_int(&w, 18U, t->q.last_pps_off_ns);
	(void)mp_cbor_kv_f32(&w, 19U, t->q.pps_off_mean_ns);
	(void)mp_cbor_kv_f32(&w, 20U, t->q.pps_off_sigma_ns);
	(void)mp_cbor_kv_f32(&w, 21U, t->q.freq_err_ppb);
	(void)mp_cbor_kv_int(&w, 22U, t->q.vc_cmd_mv);
	(void)mp_cbor_kv_int(&w, 23U, t->q.vc_sense_mv);
	(void)mp_cbor_kv_uint(&w, 24U, t->q.dac_code);

	(void)mp_cbor_uint(&w, 25U);
	(void)mp_cbor_arr(&w, 3U);
	(void)mp_cbor_f32(&w, t->q.adev_1s);
	(void)mp_cbor_f32(&w, t->q.adev_10s);
	(void)mp_cbor_f32(&w, t->q.adev_100s);

	(void)mp_cbor_kv_uint(&w, 26U, t->q.root_delay_q16);
	(void)mp_cbor_kv_uint(&w, 27U, t->q.root_disp_q16);
	(void)mp_cbor_kv_uint(&w, 28U, t->q.holdover_elapsed_s);
	(void)mp_cbor_kv_int(&w, 29U, t->q.holdover_est_err_ns);
	(void)mp_cbor_kv_uint(&w, 30U, t->q.holdover_t_demote_s);
	(void)mp_cbor_kv_uint(&w, 31U, t->q.flags);
	(void)mp_cbor_kv_int(&w, 32U, t->q.osc_temp_mc);

	(void)mp_cbor_uint(&w, 33U);
	(void)mp_cbor_arr(&w, 3U);
	(void)mp_cbor_int(&w, t->q.leap_pending);
	(void)mp_cbor_int(&w, t->q.leap_current_s);
	(void)mp_cbor_uint(&w, t->q.leap_at_tai_s);

	(void)mp_cbor_uint(&w, 34U);
	(void)mp_cbor_arr(&w, MP_RAIL_COUNT);
	for (i = 0U; i < MP_RAIL_COUNT; i++) {
		const mp_rail_t *r = &t->h.rail[i];

		(void)mp_cbor_arr(&w, 5U);
		(void)mp_cbor_int(&w, r->bus_mv);
		(void)mp_cbor_int(&w, r->current_ua);
		(void)mp_cbor_uint(&w, r->power_uw);
		(void)mp_cbor_uint(&w, r->diag);
		(void)mp_cbor_bool(&w, r->valid);
	}

	(void)mp_cbor_kv_int(&w, 35U, t->h.tmp_osc_mc);
	(void)mp_cbor_kv_int(&w, 36U, t->h.tmp_amb_mc);
	(void)mp_cbor_kv_int(&w, 37U, t->h.die_mc);
	(void)mp_cbor_kv_int(&w, 38U, t->h.humidity_mpct);
	(void)mp_cbor_kv_uint(&w, 39U, valid_flags);

	(void)mp_cbor_uint(&w, 40U);
	(void)mp_cbor_arr(&w, 2U);
	(void)mp_cbor_uint(&w, t->h.fan_rpm);
	(void)mp_cbor_uint(&w, t->h.fan_duty_pct);

	(void)mp_cbor_uint(&w, 41U);
	(void)mp_cbor_arr(&w, 3U);
	(void)mp_cbor_uint(&w, t->h.poe_class);
	(void)mp_cbor_uint(&w, t->h.poe_draw_mw);
	(void)mp_cbor_uint(&w, t->h.poe_budget_mw);

	(void)mp_cbor_uint(&w, 42U);
	(void)mp_cbor_arr(&w, 2U);
	(void)mp_cbor_bool(&w, t->h.bkp_stm_pg);
	(void)mp_cbor_bool(&w, t->h.bkp_gps_pg);

	(void)mp_cbor_kv_uint(&w, 43U, t->alarms);
	(void)mp_cbor_kv_uint(&w, 44U, t->alarms_latched);
	(void)mp_cbor_kv_uint(&w, 45U, t->scan_state);

	(void)mp_cbor_uint(&w, 46U);
	(void)mp_cbor_arr(&w, 3U);
	(void)mp_cbor_uint(&w, t->pwrseq_stage);
	(void)mp_cbor_uint(&w, t->pwrseq_shed);
	(void)mp_cbor_uint(&w, t->pwrseq_alarms);

	(void)mp_cbor_uint(&w, 47U);
	(void)mp_cbor_arr(&w, 2U);
	(void)mp_cbor_uint(&w, t->gnss_state);
	(void)mp_cbor_uint(&w, t->gnss_ant);

	(void)mp_cbor_uint(&w, 48U);
	(void)mp_cbor_arr(&w, 2U);
	(void)mp_cbor_uint(&w, t->survey_dur_s);
	(void)mp_cbor_uint(&w, t->survey_acc_mm);

	(void)mp_cbor_kv_uint(&w, 49U, refs);
	(void)mp_cbor_kv_uint(&w, 50U, t->extref_hz);
	(void)mp_cbor_kv_uint(&w, 51U, t->refsel_state);

	rc = mp_cbor_finish(&w, &len);
	if (rc != 0) {
		return rc;
	}
	return (int)len;
}

/* ================================================================ PPS ==== */

#define PPS_PAIRS (4U + 16U)

int mp_enc_pps(const mp_pps_t *p, uint8_t *buf, size_t cap)
{
	mp_cbor_t w;
	size_t len = 0U;
	int rc;

	if ((p == NULL) || (buf == NULL)) {
		return -EINVAL;
	}
	rc = mp_cbor_init(&w, buf, cap);
	if (rc != 0) {
		return rc;
	}

	rec_head(&w, PPS_PAIRS, MP_REC_PPS, 1U, p->seq, p->mono_ms);

	(void)mp_cbor_kv_int(&w, 8U, p->pa0_ns);
	(void)mp_cbor_kv_int(&w, 9U, p->pc6_ns);
	(void)mp_cbor_kv_bool(&w, 10U, p->pc6_valid);
	(void)mp_cbor_kv_int(&w, 11U, p->qerr_ps);
	(void)mp_cbor_kv_int(&w, 12U, p->residual_ns);
	(void)mp_cbor_kv_int(&w, 13U, p->residual_corr_ns);
	(void)mp_cbor_kv_int(&w, 14U, p->cable_delay_ns);
	(void)mp_cbor_kv_int(&w, 15U, p->vc_cmd_mv);
	(void)mp_cbor_kv_int(&w, 16U, p->vc_sense_mv);
	(void)mp_cbor_kv_uint(&w, 17U, p->dac_code);
	(void)mp_cbor_kv_uint(&w, 18U, p->loop_state);
	(void)mp_cbor_kv_uint(&w, 19U, p->active_ref);
	(void)mp_cbor_kv_uint(&w, 20U, p->ref_flags);
	(void)mp_cbor_kv_bool(&w, 21U, p->accepted);
	(void)mp_cbor_kv_uint(&w, 22U, p->reject_reason);
	(void)mp_cbor_kv_int(&w, 23U, p->interval_ns);

	rc = mp_cbor_finish(&w, &len);
	if (rc != 0) {
		return rc;
	}
	return (int)len;
}

/* ================================================================ log ==== */

int mp_enc_log(uint32_t seq, uint64_t mono_ms, const logr_rec_t *recs,
	       uint16_t n, uint32_t next_cursor, uint32_t gap, uint8_t *buf,
	       size_t cap)
{
	mp_cbor_t w;
	size_t len = 0U;
	uint16_t i;
	int rc;

	if (buf == NULL) {
		return -EINVAL;
	}
	if ((recs == NULL) && (n != 0U)) {
		return -EINVAL;
	}
	rc = mp_cbor_init(&w, buf, cap);
	if (rc != 0) {
		return rc;
	}

	rec_head(&w, 4U + 3U, MP_REC_LOG, 1U, seq, mono_ms);
	(void)mp_cbor_kv_uint(&w, 8U, next_cursor);
	(void)mp_cbor_kv_uint(&w, 9U, gap);

	(void)mp_cbor_uint(&w, 10U);
	(void)mp_cbor_arr(&w, n);
	for (i = 0U; i < n; i++) {
		const logr_rec_t *r = &recs[i];
		size_t tl = r->len;

		if (tl > LOGR_MSG_MAX) {
			tl = LOGR_MSG_MAX;
		}
		(void)mp_cbor_arr(&w, 5U);
		(void)mp_cbor_uint(&w, r->seq);
		(void)mp_cbor_uint(&w, r->mono_ms);
		(void)mp_cbor_uint(&w, r->level);
		(void)mp_cbor_uint(&w, r->subsys);
		(void)mp_cbor_textn(&w, r->msg, tl);
	}

	rc = mp_cbor_finish(&w, &len);
	if (rc != 0) {
		return rc;
	}
	return (int)len;
}

/* ============================================================== events === */

static const char *const ev_kind_names[MP_EV_KIND_COUNT] = {
	"fault", "button", "prox", "touch", "alarm", "override", "diag", "mode",
};

const char *mp_ev_kind_name(uint8_t kind)
{
	return (kind < (uint8_t)MP_EV_KIND_COUNT) ? ev_kind_names[kind]
						  : "unknown";
}

/* ======================================================= subscriptions === */

int mp_stream_init(mp_stream_ctx_t *c)
{
	if (c == NULL) {
		return -EINVAL;
	}
	(void)memset(c, 0, sizeof(*c));
	return 0;
}

bool mp_stream_subscribable(uint8_t ch)
{
	switch (ch) {
	case MP_CH_TELEMETRY:
	case MP_CH_NMEA:
	case MP_CH_UBX:
	case MP_CH_LOG:
	case MP_CH_PPS:
	case MP_CH_GNSS_PASS:
	case MP_CH_RB_PASS:
	case MP_CH_EVENT:
	case MP_CH_MIRROR:
		return true;
	default:
		/* Control and the SMP tunnel are not subscriptions: they are
		 * request/response and a framed byte tunnel. */
		return false;
	}
}

bool mp_stream_paced(uint8_t ch)
{
	switch (ch) {
	case MP_CH_TELEMETRY:
	case MP_CH_PPS:
	case MP_CH_LOG:
	case MP_CH_MIRROR:
		return true;
	default:
		/* NMEA/UBX/passthrough are tees and the event channel is
		 * edge-triggered: none of them has a rate to pace. */
		return false;
	}
}

int mp_stream_sub(mp_stream_ctx_t *c, uint8_t ch, uint8_t rate_hz,
		  uint32_t now_ms)
{
	mp_sub_t *s;

	if (c == NULL) {
		return -EINVAL;
	}
	if (!MP_CH_VALID(ch) || !mp_stream_subscribable(ch)) {
		return -ENOTSUP;
	}
	s = &c->sub[ch];

	if (mp_stream_paced(ch)) {
		if (rate_hz == 0U) {
			rate_hz = 1U;
		}
		if ((rate_hz < MP_RATE_MIN_HZ) || (rate_hz > MP_RATE_MAX_HZ)) {
			return -ERANGE;
		}
		s->rate_hz = rate_hz;
		s->next_ms = now_ms;
	} else {
		s->rate_hz = 0U;
		s->next_ms = now_ms;
	}
	s->on = true;
	return 0;
}

int mp_stream_unsub(mp_stream_ctx_t *c, uint8_t ch)
{
	if (c == NULL) {
		return -EINVAL;
	}
	if (!MP_CH_VALID(ch)) {
		return -ENOTSUP;
	}
	if (!c->sub[ch].on) {
		return -ENOENT;
	}
	c->sub[ch].on = false;
	c->sub[ch].rate_hz = 0U;
	return 0;
}

void mp_stream_unsub_all(mp_stream_ctx_t *c)
{
	size_t i;

	if (c == NULL) {
		return;
	}
	for (i = 0U; i < MP_CH_COUNT; i++) {
		c->sub[i].on = false;
		c->sub[i].rate_hz = 0U;
	}
}

bool mp_stream_is_sub(const mp_stream_ctx_t *c, uint8_t ch)
{
	if ((c == NULL) || !MP_CH_VALID(ch)) {
		return false;
	}
	return c->sub[ch].on;
}

bool mp_stream_due(mp_stream_ctx_t *c, uint8_t ch, uint32_t now_ms)
{
	mp_sub_t *s;
	uint32_t period;

	if ((c == NULL) || !MP_CH_VALID(ch)) {
		return false;
	}
	s = &c->sub[ch];
	if (!s->on || (s->rate_hz == 0U)) {
		return false;
	}
	if ((int32_t)(now_ms - s->next_ms) < 0) {
		return false;
	}

	period = 1000U / (uint32_t)s->rate_hz;
	if (period == 0U) {
		period = 1U;
	}
	s->next_ms += period;
	/* More than a whole period late: resynchronise instead of bursting. */
	if ((int32_t)(now_ms - s->next_ms) >= 0) {
		s->next_ms = now_ms + period;
	}
	s->sent++;
	return true;
}

uint32_t mp_stream_next_seq(mp_stream_ctx_t *c, uint8_t ch)
{
	if ((c == NULL) || !MP_CH_VALID(ch)) {
		return 0U;
	}
	c->sub[ch].seq++;
	return c->sub[ch].seq;
}

uint32_t mp_stream_cursor(const mp_stream_ctx_t *c, uint8_t ch)
{
	if ((c == NULL) || !MP_CH_VALID(ch)) {
		return 0U;
	}
	return c->sub[ch].cursor;
}

void mp_stream_set_cursor(mp_stream_ctx_t *c, uint8_t ch, uint32_t cursor)
{
	if ((c == NULL) || !MP_CH_VALID(ch)) {
		return;
	}
	c->sub[ch].cursor = cursor;
}

/* ---------------------------------------------------------- event queue */

int mp_stream_event(mp_stream_ctx_t *c, const mp_ev_t *ev)
{
	uint16_t slot;

	if ((c == NULL) || (ev == NULL)) {
		return -EINVAL;
	}
	if (ev->kind >= (uint8_t)MP_EV_KIND_COUNT) {
		return -EINVAL;
	}
	if (c->evq_len >= MP_EVQ_LEN) {
		c->evq_dropped++;
		return -ENOSPC;
	}
	slot = (uint16_t)((c->evq_head + c->evq_len) % MP_EVQ_LEN);
	c->evq[slot] = *ev;
	c->evq[slot].text[MP_EV_TEXT_MAX - 1U] = '\0';
	c->evq_len++;
	return 0;
}

int mp_stream_eventf(mp_stream_ctx_t *c, uint8_t kind, uint8_t sub, uint16_t id,
		     uint8_t edge, int32_t value, uint32_t mono_ms,
		     const char *text)
{
	mp_ev_t ev;

	(void)memset(&ev, 0, sizeof(ev));
	ev.kind = kind;
	ev.sub = sub;
	ev.id = id;
	ev.edge = edge;
	ev.value = value;
	ev.mono_ms = mono_ms;
	if (text != NULL) {
		size_t n = strlen(text);

		if (n > (MP_EV_TEXT_MAX - 1U)) {
			n = MP_EV_TEXT_MAX - 1U;
		}
		(void)memcpy(ev.text, text, n);
	}
	return mp_stream_event(c, &ev);
}

size_t mp_stream_event_count(const mp_stream_ctx_t *c)
{
	return (c != NULL) ? (size_t)c->evq_len : 0U;
}

uint32_t mp_stream_event_dropped(const mp_stream_ctx_t *c)
{
	return (c != NULL) ? c->evq_dropped : 0U;
}

int mp_enc_events(mp_stream_ctx_t *c, uint32_t seq, uint64_t mono_ms,
		  uint16_t max, uint8_t *buf, size_t cap)
{
	mp_cbor_t w;
	size_t len = 0U;
	uint16_t n;
	uint16_t i;
	int rc;

	if ((c == NULL) || (buf == NULL)) {
		return -EINVAL;
	}
	if (c->evq_len == 0U) {
		return 0;
	}

	n = c->evq_len;
	if ((max != 0U) && (n > max)) {
		n = max;
	}

	/*
	 * The batch size is part of the CBOR array header, so it has to be
	 * decided before encoding. Try the whole batch and halve on overflow:
	 * bounded (log2 of the queue depth) and it never leaves the queue in a
	 * state where an event has been consumed but not encoded.
	 */
	for (;;) {
		rc = mp_cbor_init(&w, buf, cap);
		if (rc != 0) {
			return rc;
		}
		rec_head(&w, 4U + 2U, MP_REC_EVENT, 1U, seq, mono_ms);
		(void)mp_cbor_kv_uint(&w, 8U, c->evq_dropped);
		(void)mp_cbor_uint(&w, 9U);
		(void)mp_cbor_arr(&w, n);
		for (i = 0U; i < n; i++) {
			const mp_ev_t *e =
				&c->evq[(c->evq_head + i) % MP_EVQ_LEN];

			(void)mp_cbor_arr(&w, 7U);
			(void)mp_cbor_uint(&w, e->kind);
			(void)mp_cbor_uint(&w, e->sub);
			(void)mp_cbor_uint(&w, e->id);
			(void)mp_cbor_uint(&w, e->edge);
			(void)mp_cbor_int(&w, e->value);
			(void)mp_cbor_uint(&w, e->mono_ms);
			(void)mp_cbor_text(&w, e->text);
		}
		if (mp_cbor_finish(&w, &len) == 0) {
			break;
		}
		if (n <= 1U) {
			return -ENOSPC;
		}
		n = (uint16_t)(n / 2U);
	}

	c->evq_head = (uint16_t)((c->evq_head + n) % MP_EVQ_LEN);
	c->evq_len = (uint16_t)(c->evq_len - n);
	if (c->evq_len == 0U) {
		c->evq_dropped = 0U;
	}
	return (int)len;
}

/* ============================================== §8.1 support bundle ====== */

int mp_enc_bundle(const mp_bundle_t *b, uint32_t seq, uint64_t mono_ms,
		  uint8_t *buf, size_t cap)
{
	mp_cbor_t w;
	size_t len = 0U;
	uint16_t i;
	int rc;

	if ((b == NULL) || (buf == NULL)) {
		return -EINVAL;
	}
	rc = mp_cbor_init(&w, buf, cap);
	if (rc != 0) {
		return rc;
	}

	rec_head(&w, 4U + 13U, MP_REC_BUNDLE, 1U, seq, mono_ms);

	/* 8: the telemetry record, nested whole so the host can reuse its
	 * telemetry decoder rather than a bundle-specific one. */
	(void)mp_cbor_uint(&w, 8U);
	if (b->telem != NULL) {
		uint8_t tmp[MP_TELEM_MAX];
		int n = mp_enc_telem(b->telem, tmp, sizeof(tmp));

		if (n < 0) {
			return n;
		}
		/* Emitted as a byte string: an embedded CBOR document is
		 * self-delimiting and this keeps the bundle's own structure
		 * flat and skippable. */
		(void)mp_cbor_bytes(&w, tmp, (size_t)n);
	} else {
		(void)mp_cbor_null(&w);
	}

	(void)mp_cbor_uint(&w, 9U);
	(void)mp_cbor_arr(&w, 3U);
	(void)mp_cbor_text(&w, b->fw_version);
	(void)mp_cbor_text(&w, b->boot_version);
	(void)mp_cbor_text(&w, b->board_id);

	(void)mp_cbor_kv_text(&w, 10U, b->serial);
	(void)mp_cbor_kv_uint(&w, 11U, b->fault_latched);
	(void)mp_cbor_kv_uint(&w, 12U, b->fault_active);
	(void)mp_cbor_kv_bytes(&w, 13U, b->i2c_scan,
			       (b->i2c_scan != NULL) ? 16U : 0U);

	(void)mp_cbor_uint(&w, 14U);
	if (b->pps_hist != NULL) {
		(void)mp_cbor_arr(&w, b->pps_hist_n);
		for (i = 0U; i < b->pps_hist_n; i++) {
			(void)mp_cbor_uint(&w, b->pps_hist[i]);
		}
	} else {
		(void)mp_cbor_null(&w);
	}
	(void)mp_cbor_kv_uint(&w, 15U, b->pps_hist_bin_ns);

	(void)mp_cbor_uint(&w, 16U);
	if (b->log != NULL) {
		(void)mp_cbor_arr(&w, b->log_n);
		for (i = 0U; i < b->log_n; i++) {
			const logr_rec_t *r = &b->log[i];
			size_t tl = r->len;

			if (tl > LOGR_MSG_MAX) {
				tl = LOGR_MSG_MAX;
			}
			(void)mp_cbor_arr(&w, 5U);
			(void)mp_cbor_uint(&w, r->seq);
			(void)mp_cbor_uint(&w, r->mono_ms);
			(void)mp_cbor_uint(&w, r->level);
			(void)mp_cbor_uint(&w, r->subsys);
			(void)mp_cbor_textn(&w, r->msg, tl);
		}
	} else {
		(void)mp_cbor_null(&w);
	}

	(void)mp_cbor_kv_bytes(&w, 17U, b->navsat, b->navsat_n);
	(void)mp_cbor_kv_bytes(&w, 18U, b->cfg_tlv, b->cfg_tlv_n);
	(void)mp_cbor_kv_uint(&w, 19U, b->manifest_hash);
	(void)mp_cbor_kv_text(&w, 20U, b->notes);

	rc = mp_cbor_finish(&w, &len);
	if (rc != 0) {
		return rc;
	}
	return (int)len;
}
