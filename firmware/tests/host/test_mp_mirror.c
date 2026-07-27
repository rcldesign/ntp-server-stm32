/*
 * STS1000 "Meridian" — core/mp front-panel mirror unit tests.
 *
 * The property that matters is delta correctness: a host that applies every
 * frame in order must end up with exactly the grid the device rendered. So
 * rather than asserting on row counts alone, most of these tests keep a model
 * grid, apply each frame's row spans to it the way a host would, and compare it
 * against the input. That catches an off-by-one in the span, a row committed to
 * the previous-frame store without being sent, and a truncated frame that
 * silently drops a dirty row — the three ways a delta stream desynchronises.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "mp/mp_mirror.h"
#include "mp/mp_stream.h"
#include "test_support.h"

#define ROWS 20U
#define COLS 60U
#define CELLS (ROWS * COLS)

static mp_mirror_ctx_t g_m;
static char g_prev_ch[CELLS];
static uint8_t g_prev_attr[CELLS];

/* The "device" grid handed to the encoder. */
static char g_ch[CELLS];
static uint8_t g_attr[CELLS];

/* The "host" model reconstructed from the frames. */
static char g_model_ch[CELLS];
static uint8_t g_model_attr[CELLS];

static uint8_t g_buf[MP_MIRROR_MAX];
static mp_mirror_in_t g_in;

static void grid_fill(char c, uint8_t a)
{
	memset(g_ch, c, sizeof(g_ch));
	memset(g_attr, a, sizeof(g_attr));
}

void setUp(void)
{
	TEST_ASSERT_EQUAL_INT(0, mp_mirror_init(&g_m, g_prev_ch, g_prev_attr,
						CELLS));
	grid_fill(' ', UI_ATTR_NORMAL);
	/* Deliberately un-blank, so a keyframe that failed to send a row shows
	 * up as a mismatch instead of coincidentally matching. */
	memset(g_model_ch, '?', sizeof(g_model_ch));
	memset(g_model_attr, 0xFF, sizeof(g_model_attr));

	memset(&g_in, 0, sizeof(g_in));
	g_in.rows = ROWS;
	g_in.cols = COLS;
	g_in.cell_w = 8U;
	g_in.cell_h = 16U;
	g_in.ch = g_ch;
	g_in.attr = g_attr;
	g_in.page = (uint8_t)UI_PAGE_HOME;
	g_in.depth = 1U;
	g_in.awake = true;
	g_in.bl_permille = 600U;
	g_in.panel_rail_on = true;
	g_in.panel_duty_pct = 60U;
}

/* ------------------------------------------------------------------ decoding */

typedef struct {
	uint8_t flags;
	uint8_t page;
	uint8_t depth;
	uint8_t sel;
	bool dialog;
	uint8_t dialog_item;
	uint8_t dialog_stage;
	uint8_t rows;
	uint8_t cols;
	uint8_t cell_w;
	uint8_t cell_h;
	unsigned int row_count;
	uint32_t rows_seen; /* bitmap of row indices present */
	unsigned int hint_count;
	/* leds */
	bool panel_rail_on;
	uint8_t panel_duty_pct;
	bool panel_fault;
	uint16_t led_logical;
	uint16_t bl_permille;
	uint8_t rgb_state;
	uint8_t rgb_r;
	uint8_t rgb_g;
	uint8_t rgb_b;
	/* input */
	uint32_t buttons_down;
	int32_t enc_pos;
	uint16_t touch_x;
	uint16_t touch_y;
	uint32_t touch_ms;
} frame_t;

/** Decode one mirror record, applying its rows to the model grid. */
static void decode(const uint8_t *buf, size_t len, frame_t *f)
{
	mp_cbor_rd_t r;
	uint64_t u = 0U;
	int64_t i = 0;
	size_t cnt = 0U;
	size_t k;

	memset(f, 0, sizeof(*f));

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, MP_K_TYPE));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	TEST_ASSERT_EQUAL_UINT64(MP_REC_MIRROR, u);

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, MP_K_VER));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	TEST_ASSERT_EQUAL_UINT64(MP_MIRROR_VER, u);

#define GETU(key, dst)                                                         \
	do {                                                                   \
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, buf, len));        \
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, key));            \
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));              \
		(dst) = (uint8_t)u;                                            \
	} while (0)

	GETU(8U, f->flags);
	GETU(9U, f->page);
	GETU(10U, f->depth);
	GETU(11U, f->sel);
#undef GETU

	/* Dialog. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, 12U));
	if (mp_cbor_peek_major(&r) == 4) {
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
		TEST_ASSERT_EQUAL_size_t(2U, cnt);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		f->dialog_item = (uint8_t)u;
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		f->dialog_stage = (uint8_t)u;
		f->dialog = true;
	}

	/* Geometry. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, 13U));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
	TEST_ASSERT_EQUAL_size_t(4U, cnt);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	f->rows = (uint8_t)u;
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	f->cols = (uint8_t)u;
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	f->cell_w = (uint8_t)u;
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	f->cell_h = (uint8_t)u;

	/* Rows: apply each span to the model, exactly as a host would. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, 14U));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
	f->row_count = (unsigned int)cnt;
	for (k = 0U; k < cnt; k++) {
		size_t fields = 0U;
		uint8_t row;
		uint8_t c0;
		const uint8_t *text = NULL;
		size_t tn = 0U;
		const uint8_t *attrs = NULL;
		size_t an = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &fields));
		TEST_ASSERT_EQUAL_size_t(4U, fields);
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		row = (uint8_t)u;
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		c0 = (uint8_t)u;
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_bytes(&r, &text, &tn));
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_bytes(&r, &attrs, &an));

		TEST_ASSERT_EQUAL_size_t(tn, an);
		TEST_ASSERT_TRUE(row < f->rows);
		TEST_ASSERT_TRUE(((size_t)c0 + tn) <= (size_t)f->cols);
		/* Each row appears at most once per frame. */
		TEST_ASSERT_EQUAL_UINT32(0U, f->rows_seen & (1U << row));
		f->rows_seen |= (uint32_t)1U << row;

		memcpy(&g_model_ch[((size_t)row * f->cols) + c0], text, tn);
		memcpy(&g_model_attr[((size_t)row * f->cols) + c0], attrs, an);
	}

	/* Hints. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, 15U));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
	f->hint_count = (unsigned int)cnt;
	for (k = 0U; k < cnt; k++) {
		size_t fields = 0U;
		size_t j;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &fields));
		TEST_ASSERT_EQUAL_size_t(6U, fields);
		for (j = 0U; j < 6U; j++) {
			TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
		}
	}

	/* Indicators. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, 16U));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
	TEST_ASSERT_EQUAL_size_t(9U, cnt);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_bool(&r, &f->panel_rail_on));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	f->panel_duty_pct = (uint8_t)u;
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_bool(&r, &f->panel_fault));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	f->led_logical = (uint16_t)u;
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	f->bl_permille = (uint16_t)u;
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	f->rgb_state = (uint8_t)u;
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	f->rgb_r = (uint8_t)u;
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	f->rgb_g = (uint8_t)u;
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	f->rgb_b = (uint8_t)u;

	/* Live inputs. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, 17U));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
	TEST_ASSERT_EQUAL_size_t(5U, cnt);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	f->buttons_down = (uint32_t)u;
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_int(&r, &i));
	f->enc_pos = (int32_t)i;
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	f->touch_x = (uint16_t)u;
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	f->touch_y = (uint16_t)u;
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	f->touch_ms = (uint32_t)u;

	/* The record is one well-formed CBOR item. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_skip(&r));
	TEST_ASSERT_EQUAL_INT(-ENODATA, mp_cbor_peek_major(&r));
}

/** Encode one frame into @p f, applying it to the model. */
static void frame(frame_t *f, bool force_key, uint32_t seq)
{
	int len = mp_mirror_encode(&g_m, &g_in, seq, 1000U + seq, force_key,
				  g_buf, sizeof(g_buf));

	TEST_ASSERT_TRUE(len > 0);
	decode(g_buf, (size_t)len, f);
}

static void assert_model_matches_device(void)
{
	TEST_ASSERT_EQUAL_UINT8_ARRAY(g_ch, g_model_ch, CELLS);
	TEST_ASSERT_EQUAL_UINT8_ARRAY(g_attr, g_model_attr, CELLS);
}

/* ------------------------------------------------------------------- basics */

static void test_init_validation(void)
{
	mp_mirror_ctx_t c;

	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_mirror_init(NULL, g_prev_ch,
						      g_prev_attr, CELLS));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_mirror_init(&c, NULL, g_prev_attr, CELLS));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_mirror_init(&c, g_prev_ch, NULL, CELLS));
	/* A zero-cell store is legal: the mirror is then simply disabled. */
	TEST_ASSERT_EQUAL_INT(0, mp_mirror_init(&c, NULL, NULL, 0U));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)MP_MIRROR_KEYFRAME_EVERY,
				 c.keyframe_every);
	mp_mirror_reset(NULL); /* must not fault */
}

static void test_encode_validation(void)
{
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_mirror_encode(NULL, &g_in, 1U, 1U,
							true, g_buf,
							sizeof(g_buf)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_mirror_encode(&g_m, NULL, 1U, 1U,
							true, g_buf,
							sizeof(g_buf)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_mirror_encode(&g_m, &g_in, 1U, 1U,
							true, NULL, 64U));

	/* Geometry limits. */
	g_in.rows = 0U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_mirror_encode(&g_m, &g_in, 1U, 1U,
							true, g_buf,
							sizeof(g_buf)));
	g_in.rows = ROWS;
	g_in.cols = 0U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_mirror_encode(&g_m, &g_in, 1U, 1U,
							true, g_buf,
							sizeof(g_buf)));
	g_in.cols = COLS;
	g_in.rows = (uint8_t)(UI_SURF_MAX_ROWS + 1U);
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_mirror_encode(&g_m, &g_in, 1U, 1U,
							true, g_buf,
							sizeof(g_buf)));
	g_in.rows = ROWS;
	g_in.ch = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_mirror_encode(&g_m, &g_in, 1U, 1U,
							true, g_buf,
							sizeof(g_buf)));
	g_in.ch = g_ch;
	g_in.attr = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_mirror_encode(&g_m, &g_in, 1U, 1U,
							true, g_buf,
							sizeof(g_buf)));
	g_in.attr = g_attr;

	/* A store too small for the grid. */
	{
		mp_mirror_ctx_t c;
		char small_ch[16];
		uint8_t small_attr[16];

		TEST_ASSERT_EQUAL_INT(0, mp_mirror_init(&c, small_ch,
							small_attr, 16U));
		TEST_ASSERT_EQUAL_INT(-ENOMEM,
				      mp_mirror_encode(&c, &g_in, 1U, 1U, true,
						       g_buf, sizeof(g_buf)));
	}
}

static void test_first_frame_is_a_full_keyframe(void)
{
	frame_t f;
	uint8_t r;

	memcpy(g_ch, "HOME", 4U);
	frame(&f, false, 1U);

	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_KEYFRAME) != 0U);
	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_GEOM) != 0U);
	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_AWAKE) != 0U);
	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_PARTIAL) == 0U);
	TEST_ASSERT_EQUAL_UINT(ROWS, f.row_count);
	for (r = 0U; r < ROWS; r++) {
		TEST_ASSERT_TRUE((f.rows_seen & (1U << r)) != 0U);
	}
	TEST_ASSERT_EQUAL_UINT8(ROWS, f.rows);
	TEST_ASSERT_EQUAL_UINT8(COLS, f.cols);
	TEST_ASSERT_EQUAL_UINT8(8U, f.cell_w);
	TEST_ASSERT_EQUAL_UINT8(16U, f.cell_h);

	assert_model_matches_device();
	TEST_ASSERT_EQUAL_UINT32(1U, g_m.keyframes);
	TEST_ASSERT_EQUAL_UINT32(ROWS, g_m.rows_sent);
}

static void test_unchanged_frame_carries_no_rows(void)
{
	frame_t f;

	frame(&f, false, 1U);
	assert_model_matches_device();

	frame(&f, false, 2U);
	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_KEYFRAME) == 0U);
	TEST_ASSERT_EQUAL_UINT(0U, f.row_count);
	assert_model_matches_device();
	TEST_ASSERT_EQUAL_UINT32(0U, mp_mirror_dirty(&g_m, &g_in));
}

static void test_single_cell_change_sends_one_narrow_span(void)
{
	frame_t f;
	mp_cbor_rd_t r;
	size_t cnt = 0U;
	size_t fields = 0U;
	uint64_t u = 0U;
	int len;

	frame(&f, false, 1U);

	/* One cell on row 5, column 30. */
	g_ch[(5U * COLS) + 30U] = 'X';

	TEST_ASSERT_EQUAL_UINT32((uint32_t)1U << 5,
				 mp_mirror_dirty(&g_m, &g_in));

	len = mp_mirror_encode(&g_m, &g_in, 2U, 2000U, false, g_buf,
			       sizeof(g_buf));
	TEST_ASSERT_TRUE(len > 0);

	/* The span is exactly that one cell, not the whole row. */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, (size_t)len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, 14U));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
	TEST_ASSERT_EQUAL_size_t(1U, cnt);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &fields));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	TEST_ASSERT_EQUAL_UINT64(5U, u);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u));
	TEST_ASSERT_EQUAL_UINT64(30U, u);
	{
		const uint8_t *text = NULL;
		size_t tn = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_bytes(&r, &text, &tn));
		TEST_ASSERT_EQUAL_size_t(1U, tn);
		TEST_ASSERT_EQUAL_UINT8('X', text[0]);
	}

	decode(g_buf, (size_t)len, &f);
	assert_model_matches_device();

	/* A delta frame is far smaller than the keyframe — the whole point. */
	TEST_ASSERT_TRUE((size_t)len < 120U);
}

static void test_span_spans_the_changed_range_only(void)
{
	frame_t f;
	mp_cbor_rd_t r;
	size_t cnt = 0U;
	size_t fields = 0U;
	uint64_t u = 0U;
	int len;

	frame(&f, false, 1U);

	/* Two changes on row 3, at columns 10 and 20: one span covering both. */
	g_ch[(3U * COLS) + 10U] = 'A';
	g_ch[(3U * COLS) + 20U] = 'B';

	len = mp_mirror_encode(&g_m, &g_in, 2U, 2000U, false, g_buf,
			       sizeof(g_buf));
	TEST_ASSERT_TRUE(len > 0);

	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(&r, g_buf, (size_t)len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(&r, 14U));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
	TEST_ASSERT_EQUAL_size_t(1U, cnt);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &fields));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u)); /* row */
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &u)); /* col0 */
	TEST_ASSERT_EQUAL_UINT64(10U, u);
	{
		const uint8_t *text = NULL;
		size_t tn = 0U;

		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_bytes(&r, &text, &tn));
		TEST_ASSERT_EQUAL_size_t(11U, tn); /* columns 10..20 */
		TEST_ASSERT_EQUAL_UINT8('A', text[0]);
		TEST_ASSERT_EQUAL_UINT8('B', text[10]);
	}

	decode(g_buf, (size_t)len, &f);
	assert_model_matches_device();
}

static void test_attribute_only_change_is_detected(void)
{
	frame_t f;

	frame(&f, false, 1U);

	/* The character is identical; only the attribute moved. A diff that
	 * looked at text alone would miss this and the host would keep showing
	 * the old colour. */
	g_attr[(7U * COLS) + 4U] = UI_ATTR_ALARM;

	TEST_ASSERT_EQUAL_UINT32((uint32_t)1U << 7,
				 mp_mirror_dirty(&g_m, &g_in));
	frame(&f, false, 2U);
	TEST_ASSERT_EQUAL_UINT(1U, f.row_count);
	assert_model_matches_device();
}

static void test_forced_keyframe(void)
{
	frame_t f;

	frame(&f, false, 1U);
	frame(&f, false, 2U);
	TEST_ASSERT_EQUAL_UINT(0U, f.row_count);

	/* `mirror.get` forces one, so a host joining mid-stream converges. */
	frame(&f, true, 3U);
	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_KEYFRAME) != 0U);
	TEST_ASSERT_EQUAL_UINT(ROWS, f.row_count);
	assert_model_matches_device();
}

static void test_reset_forces_a_keyframe(void)
{
	frame_t f;

	frame(&f, false, 1U);
	mp_mirror_reset(&g_m);
	frame(&f, false, 2U);
	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_KEYFRAME) != 0U);
	TEST_ASSERT_EQUAL_UINT(ROWS, f.row_count);
}

static void test_periodic_keyframe(void)
{
	frame_t f;
	unsigned int i;
	unsigned int keyframes = 0U;

	g_m.keyframe_every = 5U;

	for (i = 0U; i < 21U; i++) {
		/* Change one cell each frame so deltas are non-empty. */
		g_ch[(1U * COLS) + (i % COLS)] = (char)('a' + (char)(i % 26U));
		frame(&f, false, i + 1U);
		if ((f.flags & MP_MIRROR_F_KEYFRAME) != 0U) {
			keyframes++;
		}
		assert_model_matches_device();
	}
	/* `keyframe_every` counts *delta* frames, so a keyframe lands every
	 * sixth frame: i = 0, 6, 12, 18 out of 21. */
	TEST_ASSERT_EQUAL_UINT(4U, keyframes);

	/* keyframe_every == 0 disables the periodic refresh. */
	mp_mirror_reset(&g_m);
	g_m.keyframe_every = 0U;
	frame(&f, false, 100U); /* the post-reset keyframe */
	keyframes = 0U;
	for (i = 0U; i < 60U; i++) {
		g_ch[(2U * COLS) + (i % COLS)] = (char)('A' + (char)(i % 26U));
		frame(&f, false, 200U + i);
		if ((f.flags & MP_MIRROR_F_KEYFRAME) != 0U) {
			keyframes++;
		}
	}
	TEST_ASSERT_EQUAL_UINT(0U, keyframes);
}

static void test_geometry_change_forces_a_keyframe(void)
{
	frame_t f;

	frame(&f, false, 1U);

	/* A narrower grid: every previously stored cell describes a different
	 * position now, so the frame must be a keyframe and flag the change. */
	g_in.cols = 40U;
	memset(g_model_ch, '?', sizeof(g_model_ch));
	memset(g_model_attr, 0xFF, sizeof(g_model_attr));
	frame(&f, false, 2U);
	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_KEYFRAME) != 0U);
	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_GEOM) != 0U);
	TEST_ASSERT_EQUAL_UINT8(40U, f.cols);
	TEST_ASSERT_EQUAL_UINT(ROWS, f.row_count);

	/* A cell-metric change alone also counts as a geometry change. */
	frame(&f, false, 3U);
	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_KEYFRAME) == 0U);
	g_in.cell_h = 20U;
	frame(&f, false, 4U);
	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_GEOM) != 0U);
	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_KEYFRAME) != 0U);
}

/* --------------------------------------------------- state, lamps, inputs */

static void test_flags_and_dialog(void)
{
	frame_t f;

	g_in.awake = false;
	g_in.identify = true;
	g_in.lamp_test = true;
	g_in.dialog = true;
	g_in.dialog_item = (uint8_t)UI_MENU_FACTORY_RESET;
	g_in.dialog_stage = 1U;
	g_in.page = (uint8_t)UI_PAGE_CONFIRM;
	g_in.depth = 3U;
	g_in.sel = 2U;

	frame(&f, true, 1U);

	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_AWAKE) == 0U);
	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_IDENTIFY) != 0U);
	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_LAMP) != 0U);
	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_DIALOG) != 0U);
	TEST_ASSERT_TRUE(f.dialog);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_MENU_FACTORY_RESET, f.dialog_item);
	TEST_ASSERT_EQUAL_UINT8(1U, f.dialog_stage);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_PAGE_CONFIRM, f.page);
	TEST_ASSERT_EQUAL_UINT8(3U, f.depth);
	TEST_ASSERT_EQUAL_UINT8(2U, f.sel);

	/* With no dialog the member is CBOR null, not a zeroed pair. */
	g_in.dialog = false;
	frame(&f, true, 2U);
	TEST_ASSERT_FALSE(f.dialog);
	TEST_ASSERT_TRUE((f.flags & MP_MIRROR_F_DIALOG) == 0U);
}

static void test_indicator_state(void)
{
	frame_t f;

	g_in.panel_rail_on = true;
	g_in.panel_duty_pct = 45U;
	g_in.panel_fault = true;
	g_in.led_logical = 0x0055U;
	g_in.bl_permille = 1000U;
	g_in.rgb_state = 2U; /* fault_rgb_state_t FAULT_RGB_AMBER */
	g_in.rgb_r = 100U;
	g_in.rgb_g = 60U;
	g_in.rgb_b = 0U;

	frame(&f, true, 1U);

	TEST_ASSERT_TRUE(f.panel_rail_on);
	TEST_ASSERT_EQUAL_UINT8(45U, f.panel_duty_pct);
	TEST_ASSERT_TRUE(f.panel_fault);
	TEST_ASSERT_EQUAL_UINT16(0x0055U, f.led_logical);
	TEST_ASSERT_EQUAL_UINT16(1000U, f.bl_permille);
	TEST_ASSERT_EQUAL_UINT8(2U, f.rgb_state);
	TEST_ASSERT_EQUAL_UINT8(100U, f.rgb_r);
	TEST_ASSERT_EQUAL_UINT8(60U, f.rgb_g);
	TEST_ASSERT_EQUAL_UINT8(0U, f.rgb_b);
}

static void test_input_echo(void)
{
	frame_t f;

	/* Buttons 1 and 3 plus the encoder switch, as the 1 kHz scan reports
	 * them (fault_sig_t bits 0, 2 and 11). */
	g_in.buttons_down = (1U << 0) | (1U << 2) | (1U << 11);
	g_in.enc_pos = -12345;
	g_in.touch_x = 320U;
	g_in.touch_y = 200U;
	g_in.touch_ms = 987654U;

	frame(&f, true, 1U);

	TEST_ASSERT_EQUAL_UINT32(g_in.buttons_down, f.buttons_down);
	TEST_ASSERT_EQUAL_INT32(-12345, f.enc_pos);
	TEST_ASSERT_EQUAL_UINT16(320U, f.touch_x);
	TEST_ASSERT_EQUAL_UINT16(200U, f.touch_y);
	TEST_ASSERT_EQUAL_UINT32(987654U, f.touch_ms);
}

static void test_hints_are_carried(void)
{
	static const ui_hint_t hints[3] = {
		{ .kind = (uint8_t)UI_HINT_BIGNUM, .row = 2U, .col = 4U,
		  .len = 16U, .attr = UI_ATTR_ACCENT, .value = 0U },
		{ .kind = (uint8_t)UI_HINT_PROGRESS, .row = 8U, .col = 0U,
		  .len = 40U, .attr = UI_ATTR_OK, .value = 750U },
		{ .kind = (uint8_t)UI_HINT_RULE, .row = 1U, .col = 0U,
		  .len = 60U, .attr = UI_ATTR_DIM, .value = 0U },
	};
	frame_t f;

	g_in.hint = hints;
	g_in.hint_count = 3U;
	frame(&f, true, 1U);
	TEST_ASSERT_EQUAL_UINT(3U, f.hint_count);

	/* A count beyond the surface's own limit is clamped, not trusted. */
	g_in.hint_count = 200U;
	frame(&f, true, 2U);
	TEST_ASSERT_EQUAL_UINT(UI_SURF_MAX_HINTS, f.hint_count);

	/* A NULL list with a non-zero count emits no hints. */
	g_in.hint = NULL;
	g_in.hint_count = 3U;
	frame(&f, true, 3U);
	TEST_ASSERT_EQUAL_UINT(0U, f.hint_count);
}

/* --------------------------------------------- truncation and convergence */

/**
 * A frame that does not fit must leave the rows it could not send dirty.
 *
 * This is the delta-correctness property that matters most: the previous-frame
 * store may only record a row that actually went out, or the host and the device
 * disagree permanently with nothing to detect it.
 */
static void test_truncated_keyframe_stays_correct(void)
{
	uint8_t small[600];
	frame_t f;
	unsigned int frames = 0U;
	unsigned int i;

	/* Distinctive content so a missing row is a visible mismatch. */
	for (i = 0U; i < CELLS; i++) {
		g_ch[i] = (char)('a' + (char)(i % 26U));
		g_attr[i] = (uint8_t)(i % 7U);
	}

	/* Repeatedly encode into a buffer far too small for a keyframe; the host
	 * model must converge on the device grid. */
	for (frames = 0U; frames < 40U; frames++) {
		int len = mp_mirror_encode(&g_m, &g_in, frames + 1U, 1000U,
					   false, small, sizeof(small));

		TEST_ASSERT_TRUE(len > 0);
		TEST_ASSERT_TRUE((size_t)len <= sizeof(small));
		decode(small, (size_t)len, &f);
		if (memcmp(g_model_ch, g_ch, CELLS) == 0) {
			break;
		}
	}

	TEST_ASSERT_TRUE_MESSAGE(frames < 40U,
				 "truncated mirror never converged");
	assert_model_matches_device();
	TEST_ASSERT_TRUE(g_m.partials > 0U);
}

/** The same, but the grid keeps changing while the frames are truncated. */
static void test_truncated_deltas_under_churn_stay_correct(void)
{
	uint8_t small[400];
	frame_t f;
	test_rng_t rng;
	unsigned int step;

	test_rng_init(&rng, 0x9EED1U);

	/* Start converged with a full keyframe. */
	frame(&f, true, 1U);
	assert_model_matches_device();

	for (step = 0U; step < 400U; step++) {
		unsigned int k;
		int len;

		/* Scribble on a handful of random cells. */
		for (k = 0U; k < 12U; k++) {
			size_t at = test_rng_below(&rng, CELLS);

			g_ch[at] = (char)(0x20U + (test_rng_u32(&rng) % 0x5FU));
			g_attr[at] = (uint8_t)(test_rng_u32(&rng) & 0x3FU);
		}

		len = mp_mirror_encode(&g_m, &g_in, step + 2U, 2000U + step,
				       false, small, sizeof(small));
		TEST_ASSERT_TRUE(len > 0);
		TEST_ASSERT_TRUE((size_t)len <= sizeof(small));
		decode(small, (size_t)len, &f);
	}

	/* Stop changing the grid and let it drain. */
	for (step = 0U; step < 60U; step++) {
		int len = mp_mirror_encode(&g_m, &g_in, 1000U + step, 9000U,
					   false, small, sizeof(small));

		TEST_ASSERT_TRUE(len > 0);
		decode(small, (size_t)len, &f);
		if (memcmp(g_model_ch, g_ch, CELLS) == 0) {
			break;
		}
	}
	assert_model_matches_device();
}

static void test_buffer_too_small_for_any_row(void)
{
	uint8_t tiny[8];

	TEST_ASSERT_EQUAL_INT(-ENOSPC, mp_mirror_encode(&g_m, &g_in, 1U, 1U,
							true, tiny,
							sizeof(tiny)));
}

static void test_a_keyframe_fits_the_documented_bound(void)
{
	unsigned int i;
	int len;

	/* Worst case: the as-built 60x20 grid, every cell distinct. */
	for (i = 0U; i < CELLS; i++) {
		g_ch[i] = (char)('!' + (char)(i % 90U));
		g_attr[i] = (uint8_t)(i % 63U);
	}
	len = mp_mirror_encode(&g_m, &g_in, 1U, 1U, true, g_buf,
			       sizeof(g_buf));
	TEST_ASSERT_TRUE(len > 0);
	TEST_ASSERT_TRUE_MESSAGE((size_t)len <= MP_MIRROR_MAX,
				 "MP_MIRROR_MAX is too small for a keyframe");
	TEST_ASSERT_TRUE((g_m.frames == 1U) && (g_m.partials == 0U));
}

static void test_dirty_mask_edges(void)
{
	TEST_ASSERT_EQUAL_UINT32(0U, mp_mirror_dirty(NULL, &g_in));
	TEST_ASSERT_EQUAL_UINT32(0U, mp_mirror_dirty(&g_m, NULL));

	/* No history: every row is dirty. */
	TEST_ASSERT_EQUAL_UINT32((uint32_t)((1ULL << ROWS) - 1ULL),
				 mp_mirror_dirty(&g_m, &g_in));

	g_in.rows = 0U;
	TEST_ASSERT_EQUAL_UINT32(0U, mp_mirror_dirty(&g_m, &g_in));
	g_in.rows = ROWS;

	/* A 32-row surface reports all 32 bits. */
	{
		static char ch32[UI_SURF_MAX_ROWS * COLS];
		static uint8_t attr32[UI_SURF_MAX_ROWS * COLS];
		static char prev32[UI_SURF_MAX_ROWS * COLS];
		static uint8_t prevattr32[UI_SURF_MAX_ROWS * COLS];
		mp_mirror_ctx_t c;
		mp_mirror_in_t in = g_in;

		memset(ch32, ' ', sizeof(ch32));
		memset(attr32, 0, sizeof(attr32));
		TEST_ASSERT_EQUAL_INT(0, mp_mirror_init(&c, prev32, prevattr32,
							sizeof(prev32)));
		in.rows = (uint8_t)UI_SURF_MAX_ROWS;
		in.ch = ch32;
		in.attr = attr32;
		TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFU,
					 mp_mirror_dirty(&c, &in));
	}
}

static void test_counters(void)
{
	frame_t f;

	frame(&f, false, 1U);
	TEST_ASSERT_EQUAL_UINT32(1U, g_m.frames);
	TEST_ASSERT_EQUAL_UINT32(1U, g_m.keyframes);
	TEST_ASSERT_EQUAL_UINT32(ROWS, g_m.rows_sent);

	g_ch[0] = 'Z';
	frame(&f, false, 2U);
	TEST_ASSERT_EQUAL_UINT32(2U, g_m.frames);
	TEST_ASSERT_EQUAL_UINT32(1U, g_m.keyframes);
	TEST_ASSERT_EQUAL_UINT32(ROWS + 1U, g_m.rows_sent);
	TEST_ASSERT_EQUAL_UINT32(0U, g_m.partials);
}

/* ------------------------------------------------------------------- runner */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_init_validation);
	RUN_TEST(test_encode_validation);

	RUN_TEST(test_first_frame_is_a_full_keyframe);
	RUN_TEST(test_unchanged_frame_carries_no_rows);
	RUN_TEST(test_single_cell_change_sends_one_narrow_span);
	RUN_TEST(test_span_spans_the_changed_range_only);
	RUN_TEST(test_attribute_only_change_is_detected);
	RUN_TEST(test_forced_keyframe);
	RUN_TEST(test_reset_forces_a_keyframe);
	RUN_TEST(test_periodic_keyframe);
	RUN_TEST(test_geometry_change_forces_a_keyframe);

	RUN_TEST(test_flags_and_dialog);
	RUN_TEST(test_indicator_state);
	RUN_TEST(test_input_echo);
	RUN_TEST(test_hints_are_carried);

	RUN_TEST(test_truncated_keyframe_stays_correct);
	RUN_TEST(test_truncated_deltas_under_churn_stay_correct);
	RUN_TEST(test_buffer_too_small_for_any_row);
	RUN_TEST(test_a_keyframe_fits_the_documented_bound);
	RUN_TEST(test_dirty_mask_edges);
	RUN_TEST(test_counters);

	return UNITY_END();
}
