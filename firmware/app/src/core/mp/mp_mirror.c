/*
 * STS1000 "Meridian" — core/mp: live front-panel mirror.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See mp_mirror.h for the wire shape, the channel choice and the delta rule.
 */

#include "mp/mp_mirror.h"

#include <errno.h>
#include <string.h>

#include "mp/mp_stream.h"

/** One changed row, narrowed to its column span. */
typedef struct {
	uint8_t row;
	uint8_t c0;
	uint8_t c1;
} span_t;

int mp_mirror_init(mp_mirror_ctx_t *c, char *prev_ch, uint8_t *prev_attr,
		   size_t cells)
{
	if (c == NULL) {
		return -EINVAL;
	}
	if ((cells != 0U) && ((prev_ch == NULL) || (prev_attr == NULL))) {
		return -EINVAL;
	}

	(void)memset(c, 0, sizeof(*c));
	c->prev_ch = prev_ch;
	c->prev_attr = prev_attr;
	c->cells = cells;
	c->keyframe_every = (uint16_t)MP_MIRROR_KEYFRAME_EVERY;
	return 0;
}

void mp_mirror_reset(mp_mirror_ctx_t *c)
{
	if (c == NULL) {
		return;
	}
	c->have_prev = false;
	c->since_key = 0U;
}

/** Geometry sanity, shared by the dirty scan and the encoder. */
static bool geom_ok(const mp_mirror_in_t *in)
{
	if ((in->rows == 0U) || (in->cols == 0U)) {
		return false;
	}
	if ((in->rows > UI_SURF_MAX_ROWS) || (in->cols > UI_SURF_MAX_COLS)) {
		return false;
	}
	return (in->ch != NULL) && (in->attr != NULL);
}

/** Narrow row @p r to its changed span. Returns false when nothing changed. */
static bool row_span(const mp_mirror_ctx_t *c, const mp_mirror_in_t *in,
		     uint8_t r, uint8_t *out_c0, uint8_t *out_c1)
{
	size_t base = (size_t)r * (size_t)in->cols;
	int first = -1;
	int last = -1;
	uint8_t col;

	for (col = 0U; col < in->cols; col++) {
		size_t k = base + (size_t)col;
		bool diff = (c->prev_ch[k] != in->ch[k]) ||
			    (c->prev_attr[k] != in->attr[k]);

		if (diff) {
			if (first < 0) {
				first = (int)col;
			}
			last = (int)col;
		}
	}
	if (first < 0) {
		return false;
	}
	*out_c0 = (uint8_t)first;
	*out_c1 = (uint8_t)last;
	return true;
}

uint32_t mp_mirror_dirty(const mp_mirror_ctx_t *c, const mp_mirror_in_t *in)
{
	uint32_t mask = 0U;
	uint8_t r;

	if ((c == NULL) || (in == NULL) || !geom_ok(in)) {
		return 0U;
	}
	if (!c->have_prev || (c->rows != in->rows) || (c->cols != in->cols)) {
		/* No usable history: every row is dirty. */
		if (in->rows >= 32U) {
			return 0xFFFFFFFFU;
		}
		return (uint32_t)((1ULL << in->rows) - 1ULL);
	}

	for (r = 0U; r < in->rows; r++) {
		uint8_t c0;
		uint8_t c1;

		if (row_span(c, in, r, &c0, &c1)) {
			mask |= (uint32_t)1U << r;
		}
	}
	return mask;
}

/** Emit the fixed part of the record, then the row array of @p n spans. */
static int encode_try(mp_mirror_ctx_t *c, const mp_mirror_in_t *in,
		      uint32_t seq, uint64_t mono_ms, uint8_t flags,
		      const span_t *spans, size_t n, uint8_t *buf, size_t cap,
		      size_t *out_len)
{
	mp_cbor_t w;
	size_t i;
	int rc;

	rc = mp_cbor_init(&w, buf, cap);
	if (rc != 0) {
		return rc;
	}

	(void)mp_cbor_map(&w, 4U + 10U);
	(void)mp_cbor_kv_uint(&w, MP_K_TYPE, MP_REC_MIRROR);
	(void)mp_cbor_kv_uint(&w, MP_K_VER, MP_MIRROR_VER);
	(void)mp_cbor_kv_uint(&w, MP_K_SEQ, seq);
	(void)mp_cbor_kv_uint(&w, MP_K_MONO, mono_ms);

	(void)mp_cbor_kv_uint(&w, 8U, flags);
	(void)mp_cbor_kv_uint(&w, 9U, in->page);
	(void)mp_cbor_kv_uint(&w, 10U, in->depth);
	(void)mp_cbor_kv_uint(&w, 11U, in->sel);

	(void)mp_cbor_uint(&w, 12U);
	if (in->dialog) {
		(void)mp_cbor_arr(&w, 2U);
		(void)mp_cbor_uint(&w, in->dialog_item);
		(void)mp_cbor_uint(&w, in->dialog_stage);
	} else {
		(void)mp_cbor_null(&w);
	}

	(void)mp_cbor_uint(&w, 13U);
	(void)mp_cbor_arr(&w, 4U);
	(void)mp_cbor_uint(&w, in->rows);
	(void)mp_cbor_uint(&w, in->cols);
	(void)mp_cbor_uint(&w, in->cell_w);
	(void)mp_cbor_uint(&w, in->cell_h);

	(void)mp_cbor_uint(&w, 14U);
	(void)mp_cbor_arr(&w, n);
	for (i = 0U; i < n; i++) {
		size_t base = (size_t)spans[i].row * (size_t)in->cols;
		size_t len = (size_t)(spans[i].c1 - spans[i].c0) + 1U;

		(void)mp_cbor_arr(&w, 4U);
		(void)mp_cbor_uint(&w, spans[i].row);
		(void)mp_cbor_uint(&w, spans[i].c0);
		/*
		 * The text goes out as a byte string, not a text string: a cell
		 * may hold any byte the page code wrote, and a CBOR text string
		 * promises well-formed UTF-8.
		 */
		(void)mp_cbor_bytes(
			&w, (const uint8_t *)&in->ch[base + spans[i].c0], len);
		(void)mp_cbor_bytes(&w, &in->attr[base + spans[i].c0], len);
	}

	(void)mp_cbor_uint(&w, 15U);
	{
		size_t hn = (in->hint != NULL) ? (size_t)in->hint_count : 0U;

		if (hn > UI_SURF_MAX_HINTS) {
			hn = UI_SURF_MAX_HINTS;
		}
		(void)mp_cbor_arr(&w, hn);
		for (i = 0U; i < hn; i++) {
			(void)mp_cbor_arr(&w, 6U);
			(void)mp_cbor_uint(&w, in->hint[i].kind);
			(void)mp_cbor_uint(&w, in->hint[i].row);
			(void)mp_cbor_uint(&w, in->hint[i].col);
			(void)mp_cbor_uint(&w, in->hint[i].len);
			(void)mp_cbor_uint(&w, in->hint[i].attr);
			(void)mp_cbor_uint(&w, in->hint[i].value);
		}
	}

	(void)mp_cbor_uint(&w, 16U);
	(void)mp_cbor_arr(&w, 9U);
	(void)mp_cbor_bool(&w, in->panel_rail_on);
	(void)mp_cbor_uint(&w, in->panel_duty_pct);
	(void)mp_cbor_bool(&w, in->panel_fault);
	(void)mp_cbor_uint(&w, in->led_logical);
	(void)mp_cbor_uint(&w, in->bl_permille);
	(void)mp_cbor_uint(&w, in->rgb_state);
	(void)mp_cbor_uint(&w, in->rgb_r);
	(void)mp_cbor_uint(&w, in->rgb_g);
	(void)mp_cbor_uint(&w, in->rgb_b);

	(void)mp_cbor_uint(&w, 17U);
	(void)mp_cbor_arr(&w, 5U);
	(void)mp_cbor_uint(&w, in->buttons_down);
	(void)mp_cbor_int(&w, in->enc_pos);
	(void)mp_cbor_uint(&w, in->touch_x);
	(void)mp_cbor_uint(&w, in->touch_y);
	(void)mp_cbor_uint(&w, in->touch_ms);

	(void)c;
	return mp_cbor_finish(&w, out_len);
}

int mp_mirror_encode(mp_mirror_ctx_t *c, const mp_mirror_in_t *in, uint32_t seq,
		     uint64_t mono_ms, bool force_key, uint8_t *buf, size_t cap)
{
	span_t spans[UI_SURF_MAX_ROWS];
	size_t n = 0U;
	size_t sent;
	size_t cells;
	size_t len = 0U;
	uint8_t flags = 0U;
	bool key;
	bool geom_changed;
	uint8_t r;
	int rc;

	if ((c == NULL) || (in == NULL) || (buf == NULL)) {
		return -EINVAL;
	}
	if (!geom_ok(in)) {
		return -EINVAL;
	}
	cells = (size_t)in->rows * (size_t)in->cols;
	if (cells > c->cells) {
		return -ENOMEM;
	}

	geom_changed = c->have_prev && ((c->rows != in->rows) ||
					(c->cols != in->cols) ||
					(c->cell_w != in->cell_w) ||
					(c->cell_h != in->cell_h));

	key = force_key || !c->have_prev || geom_changed ||
	      ((c->keyframe_every != 0U) &&
	       (c->since_key >= c->keyframe_every));

	/* Build the candidate span list. */
	if (key) {
		for (r = 0U; r < in->rows; r++) {
			spans[n].row = r;
			spans[n].c0 = 0U;
			spans[n].c1 = (uint8_t)(in->cols - 1U);
			n++;
		}
	} else {
		for (r = 0U; r < in->rows; r++) {
			uint8_t c0;
			uint8_t c1;

			if (row_span(c, in, r, &c0, &c1)) {
				spans[n].row = r;
				spans[n].c0 = c0;
				spans[n].c1 = c1;
				n++;
			}
		}
	}

	if (key) {
		flags |= MP_MIRROR_F_KEYFRAME;
	}
	if (geom_changed || !c->have_prev) {
		flags |= MP_MIRROR_F_GEOM;
	}
	if (in->awake) {
		flags |= MP_MIRROR_F_AWAKE;
	}
	if (in->identify) {
		flags |= MP_MIRROR_F_IDENTIFY;
	}
	if (in->lamp_test) {
		flags |= MP_MIRROR_F_LAMP;
	}
	if (in->dialog) {
		flags |= MP_MIRROR_F_DIALOG;
	}

	/*
	 * The row-array length is part of the CBOR header, so how many rows fit
	 * must be decided before encoding. Shrink one row at a time — the rows
	 * are already ordered top-to-bottom and a caller-sized buffer normally
	 * takes all of them, so this loop is not on the hot path.
	 */
	sent = n;
	for (;;) {
		uint8_t f = flags;

		if (sent < n) {
			f |= MP_MIRROR_F_PARTIAL;
		}
		rc = encode_try(c, in, seq, mono_ms, f, spans, sent, buf, cap,
				&len);
		if (rc == 0) {
			flags = f;
			break;
		}
		if (rc != -ENOSPC) {
			return rc;
		}
		if (sent == 0U) {
			return -ENOSPC;
		}
		sent--;
	}

	/*
	 * Commit the previous-frame store for the rows that actually went out.
	 * A row left behind stays dirty, which is what makes a truncated frame
	 * safe rather than a silent desynchronisation.
	 */
	if (key && (sent == n)) {
		(void)memcpy(c->prev_ch, in->ch, cells);
		(void)memcpy(c->prev_attr, in->attr, cells);
	} else {
		size_t i;

		if (!c->have_prev || geom_changed) {
			/*
			 * Geometry just changed and the keyframe was truncated.
			 * The store's old contents describe a different grid, so
			 * blank it: a stale cell from the previous geometry must
			 * never be mistaken for "unchanged".
			 */
			(void)memset(c->prev_ch, 0, c->cells);
			(void)memset(c->prev_attr, 0xFF, c->cells);
		}
		for (i = 0U; i < sent; i++) {
			size_t base = (size_t)spans[i].row * (size_t)in->cols;
			size_t off = base + (size_t)spans[i].c0;
			size_t rlen =
				(size_t)(spans[i].c1 - spans[i].c0) + 1U;

			(void)memcpy(&c->prev_ch[off], &in->ch[off], rlen);
			(void)memcpy(&c->prev_attr[off], &in->attr[off], rlen);
		}
	}

	c->rows = in->rows;
	c->cols = in->cols;
	c->cell_w = in->cell_w;
	c->cell_h = in->cell_h;
	c->have_prev = true;

	c->frames++;
	c->rows_sent += (uint32_t)sent;
	if ((flags & MP_MIRROR_F_KEYFRAME) != 0U) {
		c->keyframes++;
	}
	if ((flags & MP_MIRROR_F_PARTIAL) != 0U) {
		c->partials++;
	}

	/*
	 * The periodic counter resets on any *attempted* keyframe, complete or
	 * truncated.
	 *
	 * Leaving it set after a truncated keyframe would re-force a keyframe on
	 * the next frame, and a keyframe's span list is ordered row 0 upwards —
	 * so a buffer that can never hold a whole keyframe would send the same
	 * leading rows forever and starve the tail. Resetting hands the
	 * remaining rows to the delta path, which lists only what is still dirty
	 * and therefore advances. MP_MIRROR_F_PARTIAL already told the host this
	 * keyframe was incomplete.
	 */
	if ((flags & MP_MIRROR_F_KEYFRAME) != 0U) {
		c->since_key = 0U;
	} else if (c->since_key < 0xFFFFU) {
		c->since_key++;
	}

	return (int)len;
}
