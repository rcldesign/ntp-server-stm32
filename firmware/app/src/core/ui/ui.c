/*
 * STS1000 "Meridian" — core/ui implementation.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See ui.h for the contract. Layout convention used by every page:
 *
 *   row 0            header  — page title, live UTC time
 *   row 1            status  — stratum, lock, reference, SV count, alarm count
 *   rows 2..         body    — page specific
 *   row rows-2       pager   — Home only: the §6.2 destination selector
 *   row rows-1       footer  — the keys that do something on this page
 *
 * Every write goes through the clipping surface primitives, so a surface too
 * small to hold the layout degrades to a truncated screen instead of a fault.
 * The host tests render a 4x16 grid for exactly that reason.
 *
 * No stdio and no libm: all formatting is integer string building (the sb_*
 * helpers below) and the four float fields that reach the panel are scaled to
 * integers behind an explicit non-finite guard.
 */

#include "ui/ui.h"

#include <errno.h>
#include <string.h>

/* ===================================================================== *
 *  Bounded string building
 * ===================================================================== */

/**
 * A cursor over a caller-owned char buffer that always leaves the result
 * NUL-terminated and never writes past the end. Overflow truncates silently: a
 * panel field that does not fit is a cosmetic problem, and an error return from
 * the middle of a layout would only produce a half-drawn screen.
 */
typedef struct {
	char *b;
	size_t cap; /* capacity including the NUL */
	size_t len;
} sb_t;

static void sb_init(sb_t *sb, char *buf, size_t cap)
{
	sb->b = buf;
	sb->cap = cap;
	sb->len = 0u;
	if (cap > 0u) {
		buf[0] = '\0';
	}
}

static void sb_ch(sb_t *sb, char c)
{
	if (sb->cap == 0u || sb->len + 1u >= sb->cap) {
		return;
	}
	sb->b[sb->len++] = c;
	sb->b[sb->len] = '\0';
}

static void sb_str(sb_t *sb, const char *s)
{
	if (s == NULL) {
		return;
	}
	while (*s != '\0') {
		sb_ch(sb, *s++);
	}
}

/** Append @p s with a-z folded to A-Z (panel headings, ASCII only). */
static void sb_upper(sb_t *sb, const char *s)
{
	if (s == NULL) {
		return;
	}
	while (*s != '\0') {
		char c = *s++;

		if (c >= 'a' && c <= 'z') {
			c = (char)(c - 'a' + 'A');
		}
		sb_ch(sb, c);
	}
}

/** Pad with spaces until the cursor sits at column @p col. */
static void sb_col(sb_t *sb, size_t col)
{
	while (sb->len < col) {
		sb_ch(sb, ' ');
	}
}

/** Unsigned decimal, zero-padded to @p digits (0 = no padding). */
static void sb_upad(sb_t *sb, uint64_t v, unsigned int digits)
{
	char tmp[21];
	unsigned int n = 0u;

	do {
		tmp[n++] = (char)('0' + (unsigned int)(v % 10u));
		v /= 10u;
	} while (v != 0u && n < sizeof(tmp));

	while (n < digits && n < sizeof(tmp)) {
		tmp[n++] = '0';
	}
	while (n > 0u) {
		sb_ch(sb, tmp[--n]);
	}
}

static void sb_u32(sb_t *sb, uint32_t v)
{
	sb_upad(sb, v, 0u);
}

static void sb_i64(sb_t *sb, int64_t v)
{
	uint64_t mag;

	if (v < 0) {
		sb_ch(sb, '-');
		/* Negate in unsigned space so INT64_MIN stays representable. */
		mag = (uint64_t)(-(v + 1)) + 1u;
	} else {
		mag = (uint64_t)v;
	}
	sb_upad(sb, mag, 0u);
}

static void sb_i32(sb_t *sb, int32_t v)
{
	sb_i64(sb, (int64_t)v);
}

/**
 * Fixed-point decimal: @p scaled is the value multiplied by 10^@p dec.
 * sb_fix(-1234, 3) yields "-1.234"; sb_fix(5, 3) yields "0.005".
 */
static void sb_fix(sb_t *sb, int64_t scaled, unsigned int dec)
{
	uint64_t mag;
	uint64_t div = 1u;
	unsigned int i;

	if (scaled < 0) {
		sb_ch(sb, '-');
		mag = (uint64_t)(-(scaled + 1)) + 1u;
	} else {
		mag = (uint64_t)scaled;
	}

	for (i = 0u; i < dec; i++) {
		div *= 10u;
	}

	sb_upad(sb, mag / div, 0u);
	if (dec > 0u) {
		sb_ch(sb, '.');
		sb_upad(sb, mag % div, dec);
	}
}

/** Append @p txt so that it ends at column @p end (right-justified field). */
static void sb_rjust(sb_t *sb, const char *txt, size_t end)
{
	size_t n = (txt != NULL) ? strlen(txt) : 0u;

	while (sb->len + n < end) {
		sb_ch(sb, ' ');
	}
	sb_str(sb, txt);
}

/** Right-justified signed integer. */
static void sb_rjust_i(sb_t *sb, int64_t v, size_t end)
{
	char tmp[24];
	sb_t t;

	sb_init(&t, tmp, sizeof(tmp));
	sb_i64(&t, v);
	sb_rjust(sb, tmp, end);
}

/** Right-justified fixed-point value with a trailing unit, e.g. "3.300 V". */
static void sb_rjust_fix(sb_t *sb, int64_t scaled, unsigned int dec,
			 const char *unit, size_t end)
{
	char tmp[32];
	sb_t t;

	sb_init(&t, tmp, sizeof(tmp));
	sb_fix(&t, scaled, dec);
	sb_str(&t, unit);
	sb_rjust(sb, tmp, end);
}

/**
 * Engineering-formatted interval in nanoseconds: "-12 ns", "1.234 us",
 * "12.345 ms", "1.234 s". A single unit will not do — the panel puts PPS
 * residuals (tens of ns) next to holdover estimates (milliseconds).
 */
static void sb_ns(sb_t *sb, int64_t ns)
{
	uint64_t mag = (ns < 0) ? ((uint64_t)(-(ns + 1)) + 1u) : (uint64_t)ns;

	if (mag < 10000u) {
		sb_i64(sb, ns);
		sb_str(sb, " ns");
	} else if (mag < 10000000u) {
		/* ns printed with 3 decimals *is* microseconds. */
		sb_fix(sb, ns, 3u);
		sb_str(sb, " us");
	} else if (mag < UINT64_C(10000000000)) {
		sb_fix(sb, ns / 1000, 3u);
		sb_str(sb, " ms");
	} else {
		sb_fix(sb, ns / 1000000, 3u);
		sb_str(sb, " s");
	}
}

/** "HH:MM:SS" from a second count; hours are unbounded. */
static void sb_hms(sb_t *sb, uint32_t secs)
{
	sb_upad(sb, secs / 3600u, 2u);
	sb_ch(sb, ':');
	sb_upad(sb, (secs / 60u) % 60u, 2u);
	sb_ch(sb, ':');
	sb_upad(sb, secs % 60u, 2u);
}

/** "3d 04:12:33", or "04:12:33" when under a day. */
static void sb_uptime(sb_t *sb, uint32_t secs)
{
	uint32_t days = secs / 86400u;

	if (days != 0u) {
		sb_u32(sb, days);
		sb_str(sb, "d ");
	}
	sb_hms(sb, secs % 86400u);
}

/**
 * Scientific notation for the ADEV fields, e.g. "1.20e-11". There is no libm
 * here, so the exponent is found by repeated scaling; the loop bounds cover
 * 1e-30..1e+30, far outside anything an OCXO or rubidium can produce.
 */
static void sb_sci(sb_t *sb, float v)
{
	int exp10 = 0;
	int mant;

	/* An Allan deviation is a positive real. NaN, infinity, zero and
	 * negatives all render as "--" rather than as a made-up number. */
	if (!(v > 0.0f) || !(v < 1.0e30f)) {
		sb_str(sb, "--");
		return;
	}

	while (v >= 10.0f && exp10 < 30) {
		v /= 10.0f;
		exp10++;
	}
	while (v < 1.0f && exp10 > -30) {
		v *= 10.0f;
		exp10--;
	}

	mant = (int)((v * 100.0f) + 0.5f);
	if (mant >= 1000) { /* rounding carried past 9.995 */
		mant = 100;
		exp10++;
	}

	sb_fix(sb, mant, 2u);
	sb_ch(sb, 'e');
	if (exp10 < 0) {
		sb_ch(sb, '-');
		exp10 = -exp10;
	} else {
		sb_ch(sb, '+');
	}
	sb_upad(sb, (uint64_t)exp10, 2u);
}

/**
 * Scale a float by 1000 into an int32, mapping every non-finite or absurd input
 * to 0. The quality block carries four floats; a NaN in any of them must
 * produce a boring cell, not a wild one.
 */
static int32_t f_milli(float v)
{
	if (!(v > -2.0e6f) || !(v < 2.0e6f)) {
		return 0;
	}
	return (int32_t)((v * 1000.0f) + ((v >= 0.0f) ? 0.5f : -0.5f));
}

/* ===================================================================== *
 *  Surface primitives
 * ===================================================================== */

static bool surf_bound(const ui_surface_t *s)
{
	return s != NULL && s->ch != NULL && s->attr != NULL && s->rows > 0u &&
	       s->cols > 0u;
}

int ui_surface_init(ui_surface_t *s, uint8_t rows, uint8_t cols, char *ch,
		    uint8_t *attr, size_t cells)
{
	if (s == NULL || ch == NULL || attr == NULL) {
		return -EINVAL;
	}
	if (rows == 0u || cols == 0u || rows > UI_SURF_MAX_ROWS ||
	    cols > UI_SURF_MAX_COLS) {
		return -EINVAL;
	}
	if (cells < (size_t)rows * (size_t)cols) {
		return -EINVAL;
	}

	memset(s, 0, sizeof(*s));
	s->rows = rows;
	s->cols = cols;
	s->cell_w = 8u;
	s->cell_h = 16u;
	s->ch = ch;
	s->attr = attr;
	ui_surface_clear(s);
	return 0;
}

void ui_surface_clear(ui_surface_t *s)
{
	size_t n;

	if (!surf_bound(s)) {
		return;
	}
	n = (size_t)s->rows * (size_t)s->cols;
	memset(s->ch, ' ', n);
	memset(s->attr, UI_ATTR_NORMAL, n);
	s->hint_count = 0u;
}

size_t ui_surface_putn(ui_surface_t *s, uint8_t row, uint8_t col,
		       const char *str, size_t max, uint8_t attr)
{
	size_t base;
	size_t i;

	if (!surf_bound(s) || str == NULL || row >= s->rows || col >= s->cols) {
		return 0u;
	}

	base = (size_t)row * (size_t)s->cols;
	for (i = 0u; i < max; i++) {
		char c = str[i];
		size_t x = (size_t)col + i;

		if (c == '\0' || x >= s->cols) {
			break;
		}
		s->ch[base + x] = c;
		s->attr[base + x] = attr;
	}
	return i;
}

size_t ui_surface_put(ui_surface_t *s, uint8_t row, uint8_t col,
		      const char *str, uint8_t attr)
{
	return ui_surface_putn(s, row, col, str, (size_t)UI_SURF_MAX_COLS, attr);
}

size_t ui_surface_fill(ui_surface_t *s, uint8_t row, uint8_t col, size_t len,
		       char c, uint8_t attr)
{
	size_t base;
	size_t i;

	if (!surf_bound(s) || row >= s->rows || col >= s->cols) {
		return 0u;
	}

	base = (size_t)row * (size_t)s->cols;
	for (i = 0u; i < len; i++) {
		size_t x = (size_t)col + i;

		if (x >= s->cols) {
			break;
		}
		s->ch[base + x] = c;
		s->attr[base + x] = attr;
	}
	return i;
}

int ui_surface_hint(ui_surface_t *s, const ui_hint_t *h)
{
	if (s == NULL || h == NULL || h->kind == (uint8_t)UI_HINT_NONE ||
	    h->kind >= (uint8_t)UI_HINT__COUNT) {
		return -EINVAL;
	}
	if (s->hint_count >= (uint8_t)UI_SURF_MAX_HINTS) {
		s->hint_dropped++;
		return -ENOSPC;
	}
	s->hint[s->hint_count++] = *h;
	return 0;
}

bool ui_surface_is_bignum_tail(const ui_surface_t *s, uint8_t row)
{
	uint8_t i;

	if (s == NULL || row == 0u) {
		return false;
	}
	for (i = 0u; i < s->hint_count; i++) {
		if (s->hint[i].kind == (uint8_t)UI_HINT_BIGNUM &&
		    (uint8_t)(s->hint[i].row + 1u) == row) {
			return true;
		}
	}
	return false;
}

size_t ui_surface_row_text(const ui_surface_t *s, uint8_t row, char *buf,
			   size_t cap)
{
	size_t base;
	size_t n;
	size_t i;

	if (buf == NULL || cap == 0u) {
		return 0u;
	}
	buf[0] = '\0';
	if (!surf_bound(s) || row >= s->rows) {
		return 0u;
	}

	base = (size_t)row * (size_t)s->cols;
	n = s->cols;
	while (n > 0u && s->ch[base + n - 1u] == ' ') {
		n--;
	}
	if (n > cap - 1u) {
		n = cap - 1u;
	}
	for (i = 0u; i < n; i++) {
		buf[i] = s->ch[base + i];
	}
	buf[n] = '\0';
	return n;
}

static bool surf_same_geom(const ui_surface_t *a, const ui_surface_t *b)
{
	return a->rows == b->rows && a->cols == b->cols;
}

int ui_surface_diff_rows(const ui_surface_t *cur, const ui_surface_t *prev,
			 uint32_t *out)
{
	uint8_t r;
	uint32_t mask = 0u;

	if (!surf_bound(cur) || out == NULL) {
		return -EINVAL;
	}
	if (prev == NULL || !surf_bound(prev)) {
		*out = (cur->rows >= 32u) ? 0xFFFFFFFFu
					  : (((uint32_t)1u << cur->rows) - 1u);
		return 0;
	}
	if (!surf_same_geom(cur, prev)) {
		return -EDOM;
	}

	for (r = 0u; r < cur->rows; r++) {
		size_t base = (size_t)r * (size_t)cur->cols;

		if (memcmp(&cur->ch[base], &prev->ch[base], cur->cols) != 0 ||
		    memcmp(&cur->attr[base], &prev->attr[base], cur->cols) != 0) {
			mask |= (uint32_t)1u << r;
		}
	}
	*out = mask;
	return 0;
}

int ui_surface_row_span(const ui_surface_t *cur, const ui_surface_t *prev,
			uint8_t row, uint8_t *c0, uint8_t *c1)
{
	size_t base;
	size_t i;
	size_t first = (size_t)-1;
	size_t last = 0u;

	if (!surf_bound(cur) || c0 == NULL || c1 == NULL || row >= cur->rows) {
		return -EINVAL;
	}
	if (prev == NULL || !surf_bound(prev)) {
		*c0 = 0u;
		*c1 = (uint8_t)(cur->cols - 1u);
		return 0;
	}
	if (!surf_same_geom(cur, prev)) {
		return -EDOM;
	}

	base = (size_t)row * (size_t)cur->cols;
	for (i = 0u; i < cur->cols; i++) {
		if (cur->ch[base + i] != prev->ch[base + i] ||
		    cur->attr[base + i] != prev->attr[base + i]) {
			if (first == (size_t)-1) {
				first = i;
			}
			last = i;
		}
	}
	if (first == (size_t)-1) {
		return -ENOENT;
	}
	*c0 = (uint8_t)first;
	*c1 = (uint8_t)last;
	return 0;
}

/* ===================================================================== *
 *  Name tables
 * ===================================================================== */

const char *ui_page_name(uint8_t page)
{
	static const char *const names[UI_PAGE__COUNT] = {
		"HOME",   "SKY VIEW", "CLOCKS", "NETWORK", "POWER/HEALTH",
		"ALARMS", "MENU",     "EDIT",   "CONFIRM",
	};

	return (page < (uint8_t)UI_PAGE__COUNT) ? names[page] : "?";
}

const char *ui_menu_name(uint8_t item)
{
	static const char *const names[UI_MENU__COUNT] = {
		"Brightness", "Blank timeout", "Identify", "Reboot",
		"Factory reset",
	};

	return (item < (uint8_t)UI_MENU__COUNT) ? names[item] : "?";
}

const char *ui_gnss_sys_name(uint8_t sys)
{
	static const char *const names[UI_GNSS__COUNT] = {
		"GPS", "GAL", "GLO", "BDS", "SBS", "QZS", "OTH",
	};

	return (sys < (uint8_t)UI_GNSS__COUNT) ? names[sys] : "???";
}

const char *ui_ant_state_name(uint8_t st)
{
	static const char *const names[UI_ANT__COUNT] = {
		"UNKNOWN", "OK", "OPEN", "SHORT", "OFF",
	};

	return (st < (uint8_t)UI_ANT__COUNT) ? names[st] : "?";
}

const char *ui_ptp_state_name(uint8_t st)
{
	static const char *const names[UI_PTP__COUNT] = {
		"DISABLED", "LISTENING", "MASTER", "PASSIVE",
	};

	return (st < (uint8_t)UI_PTP__COUNT) ? names[st] : "?";
}

static const uint8_t ui_home_dests[UI_HOME_DEST_COUNT] = {
	(uint8_t)UI_PAGE_SKYPLOT, (uint8_t)UI_PAGE_CLOCKS,
	(uint8_t)UI_PAGE_NETWORK, (uint8_t)UI_PAGE_POWER,
	(uint8_t)UI_PAGE_ALARMS,  (uint8_t)UI_PAGE_MENU,
};

/** Short pager label for a Home destination. */
static const char *home_dest_tag(uint8_t i)
{
	static const char *const tags[UI_HOME_DEST_COUNT] = {
		"SKY", "CLOCKS", "NET", "POWER", "ALARMS", "MENU",
	};

	return (i < (uint8_t)UI_HOME_DEST_COUNT) ? tags[i] : "?";
}

ui_page_t ui_home_dest(uint8_t i)
{
	return (i < (uint8_t)UI_HOME_DEST_COUNT) ? (ui_page_t)ui_home_dests[i]
						 : UI_PAGE_HOME;
}

const uint16_t ui_timeout_choices[UI_TIMEOUT_CHOICES] = {
	0u, 15u, 30u, 60u, 120u, 300u, 600u,
};

/* ===================================================================== *
 *  Configuration and lifecycle
 * ===================================================================== */

void ui_cfg_default(ui_cfg_t *cfg)
{
	if (cfg == NULL) {
		return;
	}
	cfg->brightness_pct = 60u;
	cfg->timeout_s = 120u;
	cfg->identify_timeout_s = 120u;
}

static bool cfg_valid(const ui_cfg_t *c)
{
	return c->brightness_pct >= (uint8_t)UI_BRIGHTNESS_MIN_PCT &&
	       c->brightness_pct <= (uint8_t)UI_BRIGHTNESS_MAX_PCT;
}

int ui_init(ui_ctx_t *ctx, const ui_cfg_t *cfg)
{
	if (ctx == NULL) {
		return -EINVAL;
	}

	memset(ctx, 0, sizeof(*ctx));
	if (cfg != NULL) {
		if (!cfg_valid(cfg)) {
			return -EINVAL;
		}
		ctx->cfg = *cfg;
	} else {
		ui_cfg_default(&ctx->cfg);
	}

	ctx->depth = 1u;
	ctx->stack[0].page = (uint8_t)UI_PAGE_HOME;
	ctx->awake = true;
	ctx->bl_step = (uint8_t)UI_BL_FULL;
	ctx->started = true;
	return 0;
}

/* ===================================================================== *
 *  Action queue
 * ===================================================================== */

static void action_emit(ui_ctx_t *ctx, ui_action_kind_t kind, int32_t arg)
{
	ui_action_t *a;

	if (ctx->q_len >= (uint16_t)UI_ACTION_QUEUE_LEN) {
		/* Drop the newest: an operator leaning on a key must not be
		 * able to evict an earlier REBOOT that was already accepted. */
		ctx->q_dropped++;
		return;
	}

	a = &ctx->q[(ctx->q_head + ctx->q_len) % (uint16_t)UI_ACTION_QUEUE_LEN];
	a->kind = (uint8_t)kind;
	a->_pad = 0u;
	a->_pad2 = 0;
	a->arg = arg;
	a->mono_ms = ctx->now_ms;
	ctx->q_len++;
}

int ui_action_get(ui_ctx_t *ctx, ui_action_t *out)
{
	if (ctx == NULL || out == NULL) {
		return -EINVAL;
	}
	if (ctx->q_len == 0u) {
		return -EAGAIN;
	}

	*out = ctx->q[ctx->q_head];
	ctx->q_head =
		(uint16_t)((ctx->q_head + 1u) % (uint16_t)UI_ACTION_QUEUE_LEN);
	ctx->q_len--;
	return 0;
}

size_t ui_action_count(const ui_ctx_t *ctx)
{
	return (ctx == NULL) ? 0u : (size_t)ctx->q_len;
}

uint32_t ui_action_dropped(const ui_ctx_t *ctx)
{
	return (ctx == NULL) ? 0u : ctx->q_dropped;
}

void ui_action_clear_dropped(ui_ctx_t *ctx)
{
	if (ctx != NULL) {
		ctx->q_dropped = 0u;
	}
}

/* ===================================================================== *
 *  Navigation primitives
 * ===================================================================== */

static ui_frame_t *top(ui_ctx_t *ctx)
{
	return &ctx->stack[ctx->depth - 1u];
}

static const ui_frame_t *top_c(const ui_ctx_t *ctx)
{
	return &ctx->stack[ctx->depth - 1u];
}

/** Push a fresh frame. Silently ignored at maximum depth. */
static void nav_push(ui_ctx_t *ctx, ui_page_t page, uint8_t item, uint8_t stage)
{
	ui_frame_t *f;

	if (ctx->depth >= (uint8_t)UI_STACK_MAX) {
		return;
	}
	f = &ctx->stack[ctx->depth++];
	memset(f, 0, sizeof(*f));
	f->page = (uint8_t)page;
	f->item = item;
	f->stage = stage;
}

/** Pop one frame. Home (depth 1) is the floor and never pops. */
static void nav_pop(ui_ctx_t *ctx)
{
	if (ctx->depth > 1u) {
		ctx->depth--;
	}
}

/** Collapse to Home. */
static void nav_home(ui_ctx_t *ctx)
{
	ctx->depth = 1u;
	ctx->stack[0].page = (uint8_t)UI_PAGE_HOME;
}

/** Move a wrapping selection index by @p d over @p count entries. */
static uint8_t sel_move(uint8_t sel, int32_t d, uint8_t count)
{
	int32_t v;

	if (count == 0u) {
		return 0u;
	}
	v = (int32_t)sel + (d % (int32_t)count);
	if (v < 0) {
		v += (int32_t)count;
	} else if (v >= (int32_t)count) {
		v -= (int32_t)count;
	}
	return (uint8_t)v;
}

/* ===================================================================== *
 *  Backlight / wake policy (spec §6.4)
 * ===================================================================== */

/** Duty multiplier per DISPLAY-key step, permille of the brightness setting. */
static uint16_t bl_step_permille(uint8_t step)
{
	static const uint16_t mult[UI_BL__COUNT] = {1000u, 400u, 100u, 0u};

	return (step < (uint8_t)UI_BL__COUNT) ? mult[step] : 0u;
}

uint16_t ui_backlight_permille(const ui_ctx_t *ctx)
{
	uint32_t base;

	if (ctx == NULL || !ctx->started || !ctx->awake) {
		return 0u;
	}
	base = (uint32_t)ctx->cfg.brightness_pct * 10u; /* percent -> permille */
	return (uint16_t)((base * bl_step_permille(ctx->bl_step)) / 1000u);
}

/* ===================================================================== *
 *  Menu item helpers
 * ===================================================================== */

/** Current value of an editable menu item, in its own units. */
static int32_t menu_value(const ui_ctx_t *ctx, uint8_t item)
{
	switch (item) {
	case UI_MENU_BRIGHTNESS:
		return (int32_t)ctx->cfg.brightness_pct;
	case UI_MENU_TIMEOUT:
		return (int32_t)ctx->cfg.timeout_s;
	case UI_MENU_IDENTIFY:
		return ctx->identify ? 1 : 0;
	default:
		return 0;
	}
}

/** Step an edit value by @p d detents: brightness saturates, timeout wraps. */
static int32_t menu_step(uint8_t item, int32_t cur, int32_t d)
{
	int32_t v;
	uint8_t i;
	uint8_t idx = 0u;

	switch (item) {
	case UI_MENU_BRIGHTNESS:
		v = cur + (d * (int32_t)UI_BRIGHTNESS_STEP_PCT);
		if (v < (int32_t)UI_BRIGHTNESS_MIN_PCT) {
			v = (int32_t)UI_BRIGHTNESS_MIN_PCT;
		}
		if (v > (int32_t)UI_BRIGHTNESS_MAX_PCT) {
			v = (int32_t)UI_BRIGHTNESS_MAX_PCT;
		}
		return v;
	case UI_MENU_TIMEOUT:
		for (i = 0u; i < (uint8_t)UI_TIMEOUT_CHOICES; i++) {
			if ((int32_t)ui_timeout_choices[i] == cur) {
				idx = i;
				break;
			}
		}
		idx = sel_move(idx, d, (uint8_t)UI_TIMEOUT_CHOICES);
		return (int32_t)ui_timeout_choices[idx];
	default:
		return cur;
	}
}

/** How many confirmations a guarded menu item demands. 0 = not guarded. */
static uint8_t menu_confirm_stages(uint8_t item)
{
	switch (item) {
	case UI_MENU_REBOOT:
		return 1u;
	case UI_MENU_FACTORY_RESET:
		return 2u;
	default:
		return 0u;
	}
}

static void identify_set(ui_ctx_t *ctx, bool on)
{
	if (ctx->identify == on) {
		return;
	}
	ctx->identify = on;
	ctx->identify_ms = 0u;
	action_emit(ctx, UI_ACTION_IDENTIFY, on ? 1 : 0);
}

/** Commit an edit: update the local copy of cfg and ask the glue to persist. */
static void menu_commit(ui_ctx_t *ctx, uint8_t item, int32_t val)
{
	switch (item) {
	case UI_MENU_BRIGHTNESS:
		if (val < (int32_t)UI_BRIGHTNESS_MIN_PCT) {
			val = (int32_t)UI_BRIGHTNESS_MIN_PCT;
		}
		if (val > (int32_t)UI_BRIGHTNESS_MAX_PCT) {
			val = (int32_t)UI_BRIGHTNESS_MAX_PCT;
		}
		ctx->cfg.brightness_pct = (uint8_t)val;
		/* Committing a brightness while the panel sits at the Dim or
		 * Night step would otherwise show no effect at all. */
		ctx->bl_step = (uint8_t)UI_BL_FULL;
		action_emit(ctx, UI_ACTION_SET_BRIGHTNESS, val);
		break;
	case UI_MENU_TIMEOUT:
		if (val < 0) {
			val = 0;
		}
		ctx->cfg.timeout_s = (uint16_t)val;
		action_emit(ctx, UI_ACTION_SET_TIMEOUT, val);
		break;
	default:
		break;
	}
}

/** Activate the selected menu row. */
static void menu_activate(ui_ctx_t *ctx, uint8_t item)
{
	switch (item) {
	case UI_MENU_BRIGHTNESS:
	case UI_MENU_TIMEOUT:
		ctx->edit_val = menu_value(ctx, item);
		nav_push(ctx, UI_PAGE_EDIT, item, 0u);
		break;
	case UI_MENU_IDENTIFY:
		identify_set(ctx, !ctx->identify);
		break;
	case UI_MENU_REBOOT:
	case UI_MENU_FACTORY_RESET:
		nav_push(ctx, UI_PAGE_CONFIRM, item, 0u);
		break;
	default:
		break;
	}
}

/** The affirmative answer was given on the last required stage. */
static void confirm_fire(ui_ctx_t *ctx, uint8_t item)
{
	switch (item) {
	case UI_MENU_REBOOT:
		action_emit(ctx, UI_ACTION_REBOOT, 0);
		break;
	case UI_MENU_FACTORY_RESET:
		action_emit(ctx, UI_ACTION_FACTORY_RESET, 0);
		break;
	default:
		break;
	}
	nav_home(ctx);
}

/* ===================================================================== *
 *  Input handling
 * ===================================================================== */

/** Number of selectable entries on @p page, for wrap-around arithmetic. */
static uint8_t page_sel_count(const ui_ctx_t *ctx, uint8_t page)
{
	switch (page) {
	case UI_PAGE_HOME:
		return (uint8_t)UI_HOME_DEST_COUNT;
	case UI_PAGE_MENU:
		return (uint8_t)UI_MENU__COUNT;
	case UI_PAGE_ALARMS:
		return (ctx->alarm_id_count > 0u) ? ctx->alarm_id_count : 1u;
	case UI_PAGE_CONFIRM:
		return 2u; /* No / Yes */
	default:
		return 0u; /* scrolling pages have no discrete selection */
	}
}

/** Apply a relative move (encoder or UP/DOWN) to the top frame. */
static void nav_move(ui_ctx_t *ctx, int32_t d)
{
	ui_frame_t *f = top(ctx);
	uint8_t count;

	if (d == 0) {
		return;
	}

	if (f->page == (uint8_t)UI_PAGE_EDIT) {
		ctx->edit_val = menu_step(f->item, ctx->edit_val, d);
		return;
	}

	count = page_sel_count(ctx, f->page);
	if (count != 0u) {
		f->sel = sel_move(f->sel, d, count);
		return;
	}

	/*
	 * Scrolling page. The content length is only known at render time, so
	 * the offset advances optimistically here and the renderer clamps it;
	 * that keeps ui_input() independent of the health snapshot.
	 */
	if (d < 0) {
		uint32_t back = (uint32_t)(-d);

		f->scroll = (f->scroll > back) ? (uint8_t)(f->scroll - back)
					       : 0u;
	} else {
		uint32_t fwd = (uint32_t)f->scroll + (uint32_t)d;

		f->scroll = (fwd > 250u) ? 250u : (uint8_t)fwd;
	}
}

/** ENTER / RIGHT-descend on the top frame. */
static void nav_enter(ui_ctx_t *ctx)
{
	ui_frame_t *f = top(ctx);
	uint8_t item;

	switch (f->page) {
	case UI_PAGE_HOME:
		nav_push(ctx, ui_home_dest(f->sel), 0u, 0u);
		break;
	case UI_PAGE_MENU:
		menu_activate(ctx, f->sel);
		break;
	case UI_PAGE_EDIT:
		item = f->item;
		menu_commit(ctx, item, ctx->edit_val);
		nav_pop(ctx);
		break;
	case UI_PAGE_CONFIRM:
		item = f->item;
		if (f->sel == 0u) {
			nav_pop(ctx);
		} else if ((uint8_t)(f->stage + 1u) < menu_confirm_stages(item)) {
			/* The second ask defaults to "No" on purpose: a double
			 * confirmation that starts on Yes is one dialog with an
			 * extra keypress, not two decisions. */
			nav_push(ctx, UI_PAGE_CONFIRM, item,
				 (uint8_t)(f->stage + 1u));
		} else {
			confirm_fire(ctx, item);
		}
		break;
	case UI_PAGE_ALARMS:
		if (ctx->alarm_id_count > 0u && f->sel < ctx->alarm_id_count) {
			action_emit(ctx, UI_ACTION_ACK_ALARMS,
				    (int32_t)ctx->alarm_id[f->sel]);
		}
		break;
	default:
		break;
	}
}

/** Open the menu, or leave it if it is already on top (the MENU key). */
static void nav_toggle_menu(ui_ctx_t *ctx)
{
	bool on_menu = (top(ctx)->page == (uint8_t)UI_PAGE_MENU);

	nav_home(ctx);
	if (!on_menu) {
		nav_push(ctx, UI_PAGE_MENU, 0u, 0u);
	}
}

/**
 * Map a touch coordinate onto a navigation event.
 *
 * The geometry comes from the last ui_render(); a touch that arrives before the
 * first frame has been drawn still wakes the panel but cannot be located, so it
 * is dropped rather than guessed at.
 */
static void nav_touch(ui_ctx_t *ctx, uint16_t y)
{
	ui_frame_t *f = top(ctx);
	uint8_t rows = ctx->geom_rows;
	uint8_t cell_h = ctx->geom_cell_h;
	uint8_t row;
	uint8_t count;
	uint8_t body_last;

	if (rows < 4u || cell_h == 0u) {
		return;
	}

	row = (uint16_t)(y / cell_h) >= rows ? (uint8_t)(rows - 1u)
					     : (uint8_t)(y / cell_h);

	if (row == 0u) {
		nav_pop(ctx); /* the title bar is the back affordance */
		return;
	}
	if (row == (uint8_t)(rows - 1u)) {
		nav_toggle_menu(ctx); /* the footer mirrors the MENU key */
		return;
	}

	count = page_sel_count(ctx, f->page);
	if (count == 0u) {
		return;
	}

	body_last = (uint8_t)(rows - 2u);
	if (f->page == (uint8_t)UI_PAGE_HOME) {
		/* Home's selector lives on the pager row, not in the body. */
		if (row == body_last) {
			nav_enter(ctx);
		}
		return;
	}
	if (row < 2u || row > body_last) {
		return;
	}

	{
		uint8_t idx = (uint8_t)(row - 2u + f->scroll);

		if (idx >= count) {
			return;
		}
		if (idx == f->sel) {
			nav_enter(ctx); /* second tap on a row opens it */
		} else {
			f->sel = idx;
		}
	}
}

int ui_input(ui_ctx_t *ctx, const ui_input_t *in)
{
	bool woke = false;

	if (ctx == NULL || in == NULL || !ctx->started ||
	    in->kind == (uint8_t)UI_IN_NONE ||
	    in->kind >= (uint8_t)UI_IN__COUNT) {
		return -EINVAL;
	}

	ctx->inputs++;

	if (in->kind == (uint8_t)UI_IN_TICK) {
		ctx->now_ms += in->dt_ms;

		if (ctx->identify && ctx->cfg.identify_timeout_s != 0u) {
			ctx->identify_ms += in->dt_ms;
			if (ctx->identify_ms >=
			    (uint32_t)ctx->cfg.identify_timeout_s * 1000u) {
				identify_set(ctx, false);
			}
		}

		if (ctx->awake) {
			ctx->idle_ms += in->dt_ms;
			if (ctx->cfg.timeout_s != 0u &&
			    ctx->idle_ms >=
				    (uint32_t)ctx->cfg.timeout_s * 1000u) {
				ctx->awake = false;
			}
		}
		return 0;
	}

	/* Any real input restores the panel first (spec §6.4). */
	if (!ctx->awake) {
		ctx->awake = true;
		ctx->bl_step = (uint8_t)UI_BL_FULL;
		woke = true;
	}
	ctx->idle_ms = 0u;

	if (woke) {
		/*
		 * The waking event is consumed: the touch that lights the
		 * screen must not also press whatever was underneath it. Touch
		 * and proximity are the two documented "wake to Home"
		 * surfaces; a button wakes in place so a field engineer does
		 * not lose the page they were reading.
		 */
		if (in->kind == (uint8_t)UI_IN_TOUCH ||
		    in->kind == (uint8_t)UI_IN_PROX) {
			nav_home(ctx);
		}
		return 0;
	}

	switch (in->kind) {
	case UI_IN_UP:
		nav_move(ctx, -1);
		break;
	case UI_IN_DOWN:
		nav_move(ctx, 1);
		break;
	case UI_IN_ENCODER:
		nav_move(ctx, in->delta);
		break;
	case UI_IN_LEFT:
		nav_pop(ctx);
		break;
	case UI_IN_RIGHT:
		if (top(ctx)->page == (uint8_t)UI_PAGE_CONFIRM) {
			top(ctx)->sel = 1u; /* move to the affirmative choice */
		} else {
			nav_enter(ctx);
		}
		break;
	case UI_IN_ENTER:
		nav_enter(ctx);
		break;
	case UI_IN_FN:
		nav_toggle_menu(ctx);
		break;
	case UI_IN_TOUCH:
		nav_touch(ctx, in->y);
		break;
	case UI_IN_PROX:
		/* Already awake: presence only defers the blank timer. */
		break;
	case UI_IN_DISPLAY:
		ctx->bl_step =
			(uint8_t)((ctx->bl_step + 1u) % (uint8_t)UI_BL__COUNT);
		break;
	case UI_IN_ACK:
		action_emit(ctx, UI_ACTION_ACK_ALARMS, UI_ALARM_ALL);
		nav_home(ctx);
		nav_push(ctx, UI_PAGE_ALARMS, 0u, 0u);
		break;
	case UI_IN_INFO:
		nav_home(ctx);
		nav_push(ctx, UI_PAGE_NETWORK, 0u, 0u);
		break;
	case UI_IN_LAMP: {
		bool on = (in->delta != 0);

		if (on != ctx->lamp_test) {
			ctx->lamp_test = on;
			action_emit(ctx, UI_ACTION_LAMP_TEST, on ? 1 : 0);
		}
		break;
	}
	case UI_IN_RESET_HOLD:
		nav_home(ctx);
		nav_push(ctx, UI_PAGE_CONFIRM, (uint8_t)UI_MENU_REBOOT, 0u);
		break;
	default:
		break;
	}

	return 0;
}

int ui_tick(ui_ctx_t *ctx, uint32_t dt_ms)
{
	ui_input_t in;

	memset(&in, 0, sizeof(in));
	in.kind = (uint8_t)UI_IN_TICK;
	in.dt_ms = dt_ms;
	return ui_input(ctx, &in);
}

/* ===================================================================== *
 *  Chrome
 * ===================================================================== */

/** Right-align @p str so that its last character sits at @p right_col. */
static void put_right(ui_surface_t *s, uint8_t row, uint8_t right_col,
		      const char *str, uint8_t attr)
{
	size_t len = strlen(str);
	uint8_t col;

	if (len > (size_t)right_col + 1u) {
		col = 0u;
	} else {
		col = (uint8_t)((size_t)right_col + 1u - len);
	}
	(void)ui_surface_put(s, row, col, str, attr);
}

/** Label at @p col, value right-aligned inside a field @p width wide. */
static void kv(ui_surface_t *s, uint8_t row, uint8_t col, uint8_t width,
	       const char *label, const char *value, uint8_t vattr)
{
	if (width == 0u) {
		return;
	}
	(void)ui_surface_put(s, row, col, label, UI_ATTR_DIM);
	put_right(s, row, (uint8_t)(col + width - 1u), value, vattr);
}

static void draw_rule(ui_surface_t *s, uint8_t row)
{
	ui_hint_t h;

	if (row >= s->rows) {
		return;
	}
	memset(&h, 0, sizeof(h));
	h.kind = (uint8_t)UI_HINT_RULE;
	h.row = row;
	h.col = 0u;
	h.len = s->cols;
	h.attr = UI_ATTR_DIM;
	(void)ui_surface_hint(s, &h);
}

static void fmt_clock(sb_t *sb, const ui_time_t *t)
{
	if (t->valid) {
		sb_upad(sb, t->hour, 2u);
		sb_ch(sb, ':');
		sb_upad(sb, t->min, 2u);
		sb_ch(sb, ':');
		sb_upad(sb, t->sec, 2u);
	} else {
		sb_str(sb, "--:--:--");
	}
}

static void fmt_date(sb_t *sb, const ui_time_t *t)
{
	if (t->valid) {
		sb_upad(sb, t->year, 4u);
		sb_ch(sb, '-');
		sb_upad(sb, t->mon, 2u);
		sb_ch(sb, '-');
		sb_upad(sb, t->day, 2u);
	} else {
		sb_str(sb, "----------");
	}
}

static void draw_header(const ui_ctx_t *ctx, const ui_health_t *h,
			ui_surface_t *s)
{
	char buf[32];
	sb_t sb;

	(void)ui_surface_put(s, 0u, 0u, ui_page_name(top_c(ctx)->page),
			     UI_ATTR_ACCENT);

	sb_init(&sb, buf, sizeof(buf));
	fmt_clock(&sb, &h->utc);
	sb_str(&sb, h->utc.valid ? " UTC" : " ---");
	put_right(s, 0u, (uint8_t)(s->cols - 1u), buf,
		  h->utc.valid ? UI_ATTR_ACCENT : UI_ATTR_WARN);
}

/** Count of currently active alarms in the health snapshot. */
static uint8_t alarms_active(const ui_health_t *h)
{
	uint8_t n = 0u;
	uint8_t i;
	uint8_t count = (h->alarm_count > (uint8_t)UI_MAX_ALARMS)
				? (uint8_t)UI_MAX_ALARMS
				: h->alarm_count;

	for (i = 0u; i < count; i++) {
		if (h->alarm[i].active) {
			n++;
		}
	}
	return n;
}

static void draw_status(const quality_block_t *q, const ui_health_t *h,
			ui_surface_t *s)
{
	char buf[64];
	sb_t sb;
	uint8_t nact;
	uint8_t attr;

	if (s->rows < 2u) {
		return;
	}

	sb_init(&sb, buf, sizeof(buf));
	sb_ch(&sb, 'S');
	sb_u32(&sb, q->stratum);
	sb_str(&sb, "  ");
	sb_upper(&sb, quality_lock_state_name(q->lock_state));
	sb_str(&sb, "  ");
	sb_upper(&sb, quality_ref_name(q->active_ref));
	sb_str(&sb, "  SV ");
	sb_u32(&sb, q->gnss_sv_used);
	sb_ch(&sb, '/');
	sb_u32(&sb, q->gnss_sv_visible);

	if (q->stratum == QUALITY_STRATUM_PRIMARY) {
		attr = UI_ATTR_OK;
	} else if (q->holdover) {
		attr = UI_ATTR_WARN;
	} else {
		attr = UI_ATTR_ALARM;
	}
	(void)ui_surface_put(s, 1u, 0u, buf, attr);

	nact = alarms_active(h);
	sb_init(&sb, buf, sizeof(buf));
	sb_str(&sb, "ALM ");
	sb_u32(&sb, nact);
	put_right(s, 1u, (uint8_t)(s->cols - 1u), buf,
		  (nact != 0u) ? UI_ATTR_ALARM : UI_ATTR_DIM);
}

static void draw_footer(ui_surface_t *s, const char *hints)
{
	if (s->rows < 3u) {
		return;
	}
	(void)ui_surface_fill(s, (uint8_t)(s->rows - 1u), 0u, s->cols, ' ',
			      UI_ATTR_DIM);
	(void)ui_surface_put(s, (uint8_t)(s->rows - 1u), 0u, hints, UI_ATTR_DIM);
}

/** Home's destination pager, one row above the footer. */
static void draw_pager(const ui_ctx_t *ctx, ui_surface_t *s)
{
	uint8_t row;
	uint8_t col = 0u;
	uint8_t i;

	if (s->rows < 4u) {
		return;
	}
	row = (uint8_t)(s->rows - 2u);

	for (i = 0u; i < (uint8_t)UI_HOME_DEST_COUNT; i++) {
		const char *tag = home_dest_tag(i);
		size_t n = strlen(tag);
		bool selected = (top_c(ctx)->sel == i);

		if ((size_t)col + n + 2u > (size_t)s->cols) {
			break;
		}
		if (selected) {
			(void)ui_surface_fill(s, row, col, n + 2u, ' ',
					      UI_ATTR_INVERSE);
		}
		(void)ui_surface_put(s, row, (uint8_t)(col + 1u), tag,
				     selected ? UI_ATTR_INVERSE : UI_ATTR_DIM);
		col = (uint8_t)((size_t)col + n + 3u);
	}
}

/* ===================================================================== *
 *  Pages
 * ===================================================================== */

/** First body row; no page draws above it. */
#define BODY_TOP 2u

/**
 * Last body row available to a page, given the footer (and the Home pager).
 * Never returns less than BODY_TOP, so a caller can always write one row.
 */
static uint8_t body_bottom(const ui_surface_t *s, bool with_pager)
{
	unsigned int reserved = with_pager ? 2u : 1u;

	if ((unsigned int)s->rows <= BODY_TOP + reserved) {
		return (uint8_t)BODY_TOP;
	}
	return (uint8_t)((unsigned int)s->rows - reserved - 1u);
}

/** Half-width of the two-column key/value grid used by several pages. */
static uint8_t grid_half(const ui_surface_t *s)
{
	return (uint8_t)((s->cols > 4u) ? ((unsigned int)(s->cols - 2u) / 2u)
					: 1u);
}

static void page_home(const quality_block_t *q, const ui_health_t *h,
		      ui_surface_t *s)
{
	char buf[40];
	sb_t sb;
	ui_hint_t hint;
	uint8_t half = grid_half(s);
	uint8_t colb = (uint8_t)(1u + half);
	uint8_t r;

	/* Big clock, occupying the pixel band of rows 2 and 3. */
	sb_init(&sb, buf, sizeof(buf));
	fmt_clock(&sb, &h->utc);
	(void)ui_surface_put(s, (uint8_t)BODY_TOP, 2u, buf, UI_ATTR_ACCENT);

	memset(&hint, 0, sizeof(hint));
	hint.kind = (uint8_t)UI_HINT_BIGNUM;
	hint.row = (uint8_t)BODY_TOP;
	hint.col = 2u;
	hint.len = (uint8_t)strlen(buf);
	hint.attr = UI_ATTR_ACCENT;
	(void)ui_surface_hint(s, &hint);

	r = (uint8_t)(BODY_TOP + 2u);
	sb_init(&sb, buf, sizeof(buf));
	fmt_date(&sb, &h->utc);
	(void)ui_surface_put(s, r, 2u, buf, UI_ATTR_DIM);

	sb_init(&sb, buf, sizeof(buf));
	sb_str(&sb, "up ");
	sb_uptime(&sb, h->uptime_s);
	put_right(s, r, (uint8_t)(s->cols - 1u), buf, UI_ATTR_DIM);

	r++;
	draw_rule(s, r);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_u32(&sb, q->stratum);
	kv(s, r, 1u, half, "Stratum", buf, UI_ATTR_NORMAL);
	sb_init(&sb, buf, sizeof(buf));
	sb_upper(&sb, quality_ref_name(q->active_ref));
	kv(s, r, colb, half, "Reference", buf, UI_ATTR_NORMAL);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_upper(&sb, quality_lock_state_name(q->lock_state));
	kv(s, r, 1u, half, "Lock", buf,
	   (q->lock_state == (uint8_t)QUALITY_LOCK_LOCKED) ? UI_ATTR_OK
							   : UI_ATTR_WARN);
	kv(s, r, colb, half, "Antenna", ui_ant_state_name(h->ant_state),
	   (h->ant_state == (uint8_t)UI_ANT_OK) ? UI_ATTR_OK : UI_ATTR_WARN);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_u32(&sb, q->gnss_sv_used);
	sb_ch(&sb, '/');
	sb_u32(&sb, q->gnss_sv_visible);
	kv(s, r, 1u, half, "SV used", buf, UI_ATTR_NORMAL);
	sb_init(&sb, buf, sizeof(buf));
	sb_ns(&sb, (int64_t)q->last_pps_off_ns);
	kv(s, r, colb, half, "PPS offset", buf, UI_ATTR_NORMAL);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_ns(&sb, quality_ns_from_ntp_short(q->root_disp_q16));
	kv(s, r, 1u, half, "Root disp", buf, UI_ATTR_NORMAL);
	sb_init(&sb, buf, sizeof(buf));
	sb_fix(&sb, f_milli(q->freq_err_ppb), 3u);
	sb_str(&sb, " ppb");
	kv(s, r, colb, half, "Freq err", buf, UI_ATTR_NORMAL);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	if (q->holdover) {
		sb_hms(&sb, q->holdover_elapsed_s);
	} else {
		sb_str(&sb, "--");
	}
	kv(s, r, 1u, half, "Holdover", buf,
	   q->holdover ? UI_ATTR_WARN : UI_ATTR_DIM);
	sb_init(&sb, buf, sizeof(buf));
	if (q->holdover && q->holdover_t_demote_s != UINT32_MAX) {
		sb_hms(&sb, q->holdover_t_demote_s);
	} else {
		sb_str(&sb, "--");
	}
	kv(s, r, colb, half, "Demote in", buf,
	   q->holdover ? UI_ATTR_WARN : UI_ATTR_DIM);
}

/**
 * Fill @p out with the indices of the @p want strongest satellites by C/N0.
 * Partial selection sort over a bounded array — no allocation, no qsort.
 */
static uint8_t sky_top(const ui_health_t *h, uint8_t *out, uint8_t want)
{
	uint8_t taken[UI_MAX_SV];
	uint8_t count = (h->sv_count > (uint8_t)UI_MAX_SV) ? (uint8_t)UI_MAX_SV
							   : h->sv_count;
	uint8_t n = 0u;
	uint8_t k;

	memset(taken, 0, sizeof(taken));
	for (k = 0u; k < want && n < count; k++) {
		uint8_t best = (uint8_t)UI_MAX_SV;
		uint8_t i;

		for (i = 0u; i < count; i++) {
			if (taken[i] != 0u) {
				continue;
			}
			if (best == (uint8_t)UI_MAX_SV ||
			    h->sv[i].cno_dbhz > h->sv[best].cno_dbhz) {
				best = i;
			}
		}
		if (best == (uint8_t)UI_MAX_SV) {
			break;
		}
		taken[best] = 1u;
		out[n++] = best;
	}
	return n;
}

static void page_sky(const ui_health_t *h, ui_surface_t *s)
{
	char buf[64];
	sb_t sb;
	uint8_t used[UI_GNSS__COUNT];
	uint8_t vis[UI_GNSS__COUNT];
	uint8_t top_idx[UI_SKY_TOP_N];
	uint8_t count = (h->sv_count > (uint8_t)UI_MAX_SV) ? (uint8_t)UI_MAX_SV
							   : h->sv_count;
	uint8_t ntop;
	uint8_t i;
	uint8_t r = (uint8_t)BODY_TOP;
	uint8_t last = body_bottom(s, false);

	memset(used, 0, sizeof(used));
	memset(vis, 0, sizeof(vis));

	for (i = 0u; i < count; i++) {
		uint8_t sys = (h->sv[i].sys < (uint8_t)UI_GNSS__COUNT)
				      ? h->sv[i].sys
				      : (uint8_t)UI_GNSS_OTHER;

		if (vis[sys] < 255u) {
			vis[sys]++;
		}
		if (h->sv[i].used && used[sys] < 255u) {
			used[sys]++;
		}
	}

	sb_init(&sb, buf, sizeof(buf));
	for (i = 0u; i < (uint8_t)UI_GNSS__COUNT; i++) {
		if (vis[i] == 0u) {
			continue;
		}
		sb_str(&sb, ui_gnss_sys_name(i));
		sb_ch(&sb, ' ');
		sb_u32(&sb, used[i]);
		sb_ch(&sb, '/');
		sb_u32(&sb, vis[i]);
		sb_str(&sb, "  ");
	}
	if (sb.len == 0u) {
		sb_str(&sb, "no satellites tracked");
	}
	(void)ui_surface_put(s, r, 1u, buf, UI_ATTR_NORMAL);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_str(&sb, "Antenna ");
	sb_str(&sb, ui_ant_state_name(h->ant_state));
	if (h->survey_active) {
		sb_str(&sb, "   Survey ");
		sb_u32(&sb, h->survey_dur_s);
		sb_str(&sb, " s  acc ");
		sb_u32(&sb, h->survey_acc_mm);
		sb_str(&sb, " mm");
	}
	(void)ui_surface_put(s, r, 1u, buf,
			     (h->ant_state == (uint8_t)UI_ANT_OK)
				     ? UI_ATTR_NORMAL
				     : UI_ATTR_WARN);

	r++;
	draw_rule(s, r);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_str(&sb, "SYS");
	sb_rjust(&sb, "SVID", 10u);
	sb_rjust(&sb, "AZ", 16u);
	sb_rjust(&sb, "EL", 22u);
	sb_rjust(&sb, "C/N0", 29u);
	sb_col(&sb, 32u);
	sb_str(&sb, "USE");
	(void)ui_surface_put(s, r, 1u, buf, UI_ATTR_DIM);

	ntop = sky_top(h, top_idx, (uint8_t)UI_SKY_TOP_N);
	for (i = 0u; i < ntop; i++) {
		const ui_sv_t *sv = &h->sv[top_idx[i]];

		r++;
		if (r > last) {
			break;
		}
		sb_init(&sb, buf, sizeof(buf));
		sb_str(&sb, ui_gnss_sys_name(sv->sys));
		sb_rjust_i(&sb, sv->svid, 10u);
		sb_rjust_i(&sb, sv->azim_deg, 16u);
		sb_rjust_i(&sb, sv->elev_deg, 22u);
		sb_rjust_i(&sb, sv->cno_dbhz, 29u);
		sb_col(&sb, 32u);
		sb_str(&sb, sv->used ? "YES" : "-");
		(void)ui_surface_put(s, r, 1u, buf,
				     sv->used ? UI_ATTR_OK : UI_ATTR_DIM);
	}

	if (ntop == 0u && (uint8_t)(r + 1u) <= last) {
		(void)ui_surface_put(s, (uint8_t)(r + 1u), 1u,
				     "(no satellite data)", UI_ATTR_DIM);
	}
}

static void page_clocks(const quality_block_t *q, const ui_health_t *h,
			ui_surface_t *s)
{
	char buf[48];
	sb_t sb;
	uint8_t half = grid_half(s);
	uint8_t colb = (uint8_t)(1u + half);
	uint8_t r = (uint8_t)BODY_TOP;

	(void)ui_surface_put(s, r, 0u, "OCXO", UI_ATTR_ACCENT);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_i32(&sb, q->vc_cmd_mv);
	sb_str(&sb, " mV");
	kv(s, r, 1u, half, "Vc commanded", buf, UI_ATTR_NORMAL);
	sb_init(&sb, buf, sizeof(buf));
	sb_i32(&sb, q->vc_sense_mv);
	sb_str(&sb, " mV");
	kv(s, r, colb, half, "Vc sensed", buf,
	   ((q->flags & QUALITY_FLAG_DAC_FAULT) != 0u) ? UI_ATTR_ALARM
						       : UI_ATTR_NORMAL);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_u32(&sb, q->dac_code);
	kv(s, r, 1u, half, "DAC code", buf,
	   ((q->flags & QUALITY_FLAG_DAC_SAT) != 0u) ? UI_ATTR_WARN
						     : UI_ATTR_NORMAL);
	sb_init(&sb, buf, sizeof(buf));
	sb_fix(&sb, q->osc_temp_mc, 3u);
	sb_str(&sb, " C");
	kv(s, r, colb, half, "Oven temp", buf,
	   ((q->flags & QUALITY_FLAG_OCXO_WARM) != 0u) ? UI_ATTR_OK
						       : UI_ATTR_WARN);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_sci(&sb, q->adev_1s);
	kv(s, r, 1u, half, "ADEV 1 s", buf, UI_ATTR_NORMAL);
	sb_init(&sb, buf, sizeof(buf));
	sb_sci(&sb, q->adev_100s);
	kv(s, r, colb, half, "ADEV 100 s", buf, UI_ATTR_NORMAL);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_fix(&sb, f_milli(q->pps_off_sigma_ns), 3u);
	sb_str(&sb, " ns");
	kv(s, r, 1u, half, "PPS sigma", buf, UI_ATTR_NORMAL);
	sb_init(&sb, buf, sizeof(buf));
	sb_u32(&sb, q->gnss_tacc_ns);
	sb_str(&sb, " ns");
	kv(s, r, colb, half, "GNSS tAcc", buf, UI_ATTR_NORMAL);

	r++;
	draw_rule(s, r);
	r++;
	(void)ui_surface_put(s, r, 0u, "RUBIDIUM / EXTERNAL", UI_ATTR_ACCENT);

	r++;
	kv(s, r, 1u, half, "Rb power",
	   h->rb_present ? (h->rb_powered ? "ON" : "OFF") : "ABSENT",
	   h->rb_powered ? UI_ATTR_OK : UI_ATTR_DIM);
	kv(s, r, colb, half, "Rb lock", h->rb_locked ? "LOCKED" : "UNLOCKED",
	   h->rb_locked ? UI_ATTR_OK : UI_ATTR_DIM);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_fix(&sb, h->rb_temp_mc, 3u);
	sb_str(&sb, " C");
	kv(s, r, 1u, half, "Rb temp", buf, UI_ATTR_NORMAL);
	sb_init(&sb, buf, sizeof(buf));
	if (h->extref_ok) {
		sb_u32(&sb, h->extref_hz);
		sb_str(&sb, " Hz");
	} else {
		sb_str(&sb, "not in band");
	}
	kv(s, r, colb, half, "Ext ref", buf,
	   h->extref_ok ? UI_ATTR_OK : UI_ATTR_DIM);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_upper(&sb, quality_ref_name(q->active_ref));
	kv(s, r, 1u, half, "Active ref", buf, UI_ATTR_ACCENT);
	sb_init(&sb, buf, sizeof(buf));
	sb_ns(&sb, q->holdover_est_err_ns);
	kv(s, r, colb, half, "Holdover err", buf,
	   q->holdover ? UI_ATTR_WARN : UI_ATTR_DIM);
}

static void page_network(const ui_health_t *h, ui_surface_t *s)
{
	char buf[48];
	sb_t sb;
	uint8_t half = grid_half(s);
	uint8_t colb = (uint8_t)(1u + half);
	uint8_t r = (uint8_t)BODY_TOP;

	sb_init(&sb, buf, sizeof(buf));
	if (h->link_up) {
		sb_str(&sb, "UP ");
		sb_u32(&sb, h->link_mbps);
		sb_str(&sb, " Mb/s");
	} else {
		sb_str(&sb, "DOWN");
	}
	kv(s, r, 1u, half, "Link", buf, h->link_up ? UI_ATTR_OK : UI_ATTR_ALARM);
	kv(s, r, colb, half, "Address", h->dhcp ? "DHCP" : "STATIC",
	   UI_ATTR_NORMAL);

	r++;
	(void)ui_surface_put(s, r, 1u, "IPv4", UI_ATTR_DIM);
	(void)ui_surface_putn(s, r, 8u, h->ipv4, UI_STR_LEN, UI_ATTR_NORMAL);
	r++;
	(void)ui_surface_put(s, r, 1u, "IPv6", UI_ATTR_DIM);
	(void)ui_surface_putn(s, r, 8u, h->ipv6, UI_STR_LEN, UI_ATTR_NORMAL);
	r++;
	(void)ui_surface_put(s, r, 1u, "Host", UI_ATTR_DIM);
	(void)ui_surface_putn(s, r, 8u, h->hostname, UI_STR_LEN, UI_ATTR_NORMAL);

	r++;
	draw_rule(s, r);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_u32(&sb, h->ntp_req_per_s);
	sb_str(&sb, " req/s");
	kv(s, r, 1u, half, "NTP rate", buf, UI_ATTR_NORMAL);
	sb_init(&sb, buf, sizeof(buf));
	sb_u32(&sb, h->ntp_served);
	kv(s, r, colb, half, "NTP served", buf, UI_ATTR_NORMAL);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_u32(&sb, h->ntp_dropped);
	kv(s, r, 1u, half, "NTP dropped", buf,
	   (h->ntp_dropped != 0u) ? UI_ATTR_WARN : UI_ATTR_NORMAL);
	sb_init(&sb, buf, sizeof(buf));
	if (h->nts_enabled) {
		sb_u32(&sb, h->nts_sessions);
		sb_str(&sb, " sess");
	} else {
		sb_str(&sb, "off");
	}
	kv(s, r, colb, half, "NTS", buf,
	   h->nts_enabled ? UI_ATTR_OK : UI_ATTR_DIM);

	r++;
	kv(s, r, 1u, half, "PTP", ui_ptp_state_name(h->ptp_state),
	   (h->ptp_state == (uint8_t)UI_PTP_MASTER) ? UI_ATTR_OK : UI_ATTR_DIM);
	sb_init(&sb, buf, sizeof(buf));
	sb_str(&sb, "class ");
	sb_u32(&sb, h->ptp_clock_class);
	sb_str(&sb, ", ");
	sb_u32(&sb, h->ptp_clients);
	sb_str(&sb, " cli");
	kv(s, r, colb, half, "", buf, UI_ATTR_NORMAL);

	r++;
	draw_rule(s, r);
	r++;
	(void)ui_surface_put(s, r, 1u, "F/W", UI_ATTR_DIM);
	(void)ui_surface_putn(s, r, 8u, h->fw_version, UI_STR_LEN,
			      UI_ATTR_NORMAL);
	r++;
	(void)ui_surface_put(s, r, 1u, "S/N", UI_ATTR_DIM);
	(void)ui_surface_putn(s, r, 8u, h->serial, UI_STR_LEN, UI_ATTR_NORMAL);
}

/** Rows the Power page reserves at the bottom for its summary block. */
#define POWER_SUMMARY_ROWS 4u

static void page_power(ui_ctx_t *ctx, const ui_health_t *h, ui_surface_t *s)
{
	char buf[64];
	sb_t sb;
	ui_frame_t *f = top(ctx);
	uint8_t count = (h->rail_count > (uint8_t)UI_MAX_RAILS)
				? (uint8_t)UI_MAX_RAILS
				: h->rail_count;
	unsigned int last = body_bottom(s, false);
	unsigned int rule_row;
	unsigned int visible;
	uint8_t half = grid_half(s);
	uint8_t colb = (uint8_t)(1u + half);
	uint8_t r;
	uint8_t i;

	sb_init(&sb, buf, sizeof(buf));
	sb_str(&sb, "RAIL");
	sb_rjust(&sb, "VOLTAGE", 21u);
	sb_rjust(&sb, "CURRENT", 31u);
	sb_col(&sb, 34u);
	sb_str(&sb, "PG");
	sb_col(&sb, 38u);
	sb_str(&sb, "AL");
	(void)ui_surface_put(s, (uint8_t)BODY_TOP, 1u, buf, UI_ATTR_DIM);

	/* The summary block occupies the last POWER_SUMMARY_ROWS rows of the
	 * body: a rule and three key/value rows. Everything between the header
	 * row and it belongs to the scrolling rail list. */
	rule_row = (last >= BODY_TOP + POWER_SUMMARY_ROWS)
			   ? (last - (POWER_SUMMARY_ROWS - 1u))
			   : (BODY_TOP + 1u);
	visible = (rule_row > BODY_TOP + 1u) ? (rule_row - BODY_TOP - 1u) : 0u;

	{
		unsigned int max_scroll =
			(count > visible) ? ((unsigned int)count - visible) : 0u;

		if ((unsigned int)f->scroll > max_scroll) {
			f->scroll = (uint8_t)max_scroll;
		}
	}

	for (i = 0u; i < (uint8_t)visible; i++) {
		unsigned int idx = (unsigned int)f->scroll + i;
		const ui_rail_t *rail;

		if (idx >= count) {
			break;
		}
		rail = &h->rail[idx];

		sb_init(&sb, buf, sizeof(buf));
		sb_str(&sb, rail->name);
		sb_col(&sb, 12u);
		sb_rjust_fix(&sb, (int64_t)rail->mv, 3u, " V", 21u);
		sb_rjust_fix(&sb, (int64_t)rail->ma, 3u, " A", 31u);
		sb_col(&sb, 34u);
		sb_str(&sb, rail->pg ? "OK" : "NO");
		sb_col(&sb, 38u);
		sb_str(&sb, rail->alert ? "AL" : "--");
		(void)ui_surface_put(s, (uint8_t)(BODY_TOP + 1u + i), 1u, buf,
				     (!rail->pg || rail->alert)
					     ? UI_ATTR_ALARM
					     : UI_ATTR_NORMAL);
	}

	r = (uint8_t)rule_row;
	draw_rule(s, r);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_fix(&sb, h->temp_enclosure_mc, 3u);
	sb_str(&sb, " C");
	kv(s, r, 1u, half, "Enclosure", buf, UI_ATTR_NORMAL);
	sb_init(&sb, buf, sizeof(buf));
	sb_fix(&sb, h->temp_die_mc, 3u);
	sb_str(&sb, " C");
	kv(s, r, colb, half, "MCU die", buf, UI_ATTR_NORMAL);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_u32(&sb, h->fan_duty_pct);
	sb_str(&sb, " %, ");
	sb_u32(&sb, h->fan_rpm);
	sb_str(&sb, " rpm");
	kv(s, r, 1u, half, "Fan", buf, UI_ATTR_NORMAL);
	sb_init(&sb, buf, sizeof(buf));
	sb_fix(&sb, h->poe_mw, 3u);
	sb_str(&sb, " W cls ");
	sb_u32(&sb, h->poe_class);
	kv(s, r, colb, half, "PoE", buf, UI_ATTR_NORMAL);

	r++;
	sb_init(&sb, buf, sizeof(buf));
	sb_fix(&sb, h->supercap_stm_mv, 3u);
	sb_str(&sb, " V");
	kv(s, r, 1u, half, "Backup STM", buf,
	   h->bkp_stm_pg ? UI_ATTR_OK : UI_ATTR_WARN);
	sb_init(&sb, buf, sizeof(buf));
	sb_fix(&sb, h->supercap_gps_mv, 3u);
	sb_str(&sb, " V");
	kv(s, r, colb, half, "Backup GPS", buf,
	   h->bkp_gps_pg ? UI_ATTR_OK : UI_ATTR_WARN);
}

static void page_alarms(ui_ctx_t *ctx, const ui_health_t *h, ui_surface_t *s)
{
	char buf[64];
	sb_t sb;
	ui_frame_t *f = top(ctx);
	uint8_t count = (h->alarm_count > (uint8_t)UI_MAX_ALARMS)
				? (uint8_t)UI_MAX_ALARMS
				: h->alarm_count;
	unsigned int last = body_bottom(s, false);
	unsigned int visible = (last >= BODY_TOP) ? (last - BODY_TOP + 1u) : 0u;
	uint8_t i;

	/* Capture the row->id mapping so ENTER can acknowledge without needing
	 * the health snapshot (see ui_ctx_t::alarm_id). */
	ctx->alarm_id_count = count;
	for (i = 0u; i < count; i++) {
		ctx->alarm_id[i] = h->alarm[i].id;
	}

	if (count == 0u) {
		f->sel = 0u;
		f->scroll = 0u;
		(void)ui_surface_put(s, (uint8_t)BODY_TOP, 1u,
				     "No alarms latched.", UI_ATTR_OK);
		return;
	}

	if (f->sel >= count) {
		f->sel = (uint8_t)(count - 1u);
	}
	if (f->sel < f->scroll) {
		f->scroll = f->sel;
	} else if (visible != 0u &&
		   (unsigned int)f->sel >= (unsigned int)f->scroll + visible) {
		f->scroll = (uint8_t)((unsigned int)f->sel - visible + 1u);
	}
	{
		unsigned int max_scroll =
			(count > visible) ? ((unsigned int)count - visible) : 0u;

		if ((unsigned int)f->scroll > max_scroll) {
			f->scroll = (uint8_t)max_scroll;
		}
	}

	for (i = 0u; i < (uint8_t)visible; i++) {
		unsigned int idx = (unsigned int)f->scroll + i;
		const ui_alarm_row_t *a;
		uint8_t attr;

		if (idx >= count) {
			break;
		}
		a = &h->alarm[idx];

		sb_init(&sb, buf, sizeof(buf));
		sb_str(&sb, (idx == f->sel) ? "> " : "  ");
		if (h->alarm_name != NULL) {
			sb_str(&sb, h->alarm_name(a->id));
		} else {
			sb_str(&sb, "ALARM ");
			sb_u32(&sb, a->id);
		}
		sb_col(&sb, 24u);
		sb_str(&sb, a->active ? "ACTIVE"
				      : (a->latched ? "LATCHED" : "CLEAR"));
		sb_col(&sb, 33u);
		sb_ch(&sb, 'x');
		sb_u32(&sb, a->count);
		sb_col(&sb, 39u);
		sb_str(&sb, "at ");
		sb_hms(&sb, a->first_s);

		if (a->active) {
			attr = UI_ATTR_ALARM;
		} else if (a->latched) {
			attr = UI_ATTR_WARN;
		} else {
			attr = UI_ATTR_DIM;
		}
		if (idx == f->sel) {
			attr = (uint8_t)(attr | UI_ATTR_INVERSE);
			(void)ui_surface_fill(s, (uint8_t)(BODY_TOP + i), 0u,
					      s->cols, ' ', attr);
		}
		(void)ui_surface_put(s, (uint8_t)(BODY_TOP + i), 0u, buf, attr);
	}
}

static void page_menu(ui_ctx_t *ctx, ui_surface_t *s)
{
	char buf[64];
	sb_t sb;
	ui_frame_t *f = top(ctx);
	uint8_t last = body_bottom(s, false);
	uint8_t i;

	if (f->sel >= (uint8_t)UI_MENU__COUNT) {
		f->sel = (uint8_t)(UI_MENU__COUNT - 1u);
	}

	for (i = 0u; i < (uint8_t)UI_MENU__COUNT; i++) {
		uint8_t row = (uint8_t)(BODY_TOP + i);
		uint8_t attr = (i == f->sel) ? UI_ATTR_INVERSE : UI_ATTR_NORMAL;

		if (row > last) {
			break;
		}

		sb_init(&sb, buf, sizeof(buf));
		sb_str(&sb, (i == f->sel) ? "> " : "  ");
		sb_str(&sb, ui_menu_name(i));
		sb_col(&sb, 24u);
		switch (i) {
		case UI_MENU_BRIGHTNESS:
			sb_u32(&sb, ctx->cfg.brightness_pct);
			sb_str(&sb, " %");
			break;
		case UI_MENU_TIMEOUT:
			if (ctx->cfg.timeout_s == 0u) {
				sb_str(&sb, "never");
			} else {
				sb_u32(&sb, ctx->cfg.timeout_s);
				sb_str(&sb, " s");
			}
			break;
		case UI_MENU_IDENTIFY:
			sb_str(&sb, ctx->identify ? "ON" : "off");
			break;
		default:
			sb_str(&sb, ">");
			break;
		}
		if (i == f->sel) {
			(void)ui_surface_fill(s, row, 0u, s->cols, ' ', attr);
		}
		(void)ui_surface_put(s, row, 0u, buf, attr);
	}
}

static void page_edit(const ui_ctx_t *ctx, ui_surface_t *s)
{
	char buf[32];
	sb_t sb;
	const ui_frame_t *f = top_c(ctx);
	ui_hint_t hint;
	uint8_t r = (uint8_t)BODY_TOP;
	uint32_t permille = 0u;

	(void)ui_surface_put(s, r, 1u, ui_menu_name(f->item), UI_ATTR_ACCENT);

	r = (uint8_t)(BODY_TOP + 2u);
	sb_init(&sb, buf, sizeof(buf));
	switch (f->item) {
	case UI_MENU_BRIGHTNESS:
		sb_i32(&sb, ctx->edit_val);
		sb_str(&sb, " %");
		permille = (ctx->edit_val <= 0)
				   ? 0u
				   : ((uint32_t)ctx->edit_val * 10u);
		break;
	case UI_MENU_TIMEOUT:
		if (ctx->edit_val == 0) {
			sb_str(&sb, "never");
		} else {
			sb_i32(&sb, ctx->edit_val);
			sb_str(&sb, " s");
		}
		permille = (ctx->edit_val <= 0)
				   ? 0u
				   : ((uint32_t)ctx->edit_val * 1000u / 600u);
		break;
	default:
		sb_i32(&sb, ctx->edit_val);
		break;
	}
	(void)ui_surface_put(s, r, 2u, buf, UI_ATTR_ACCENT);

	memset(&hint, 0, sizeof(hint));
	hint.kind = (uint8_t)UI_HINT_BIGNUM;
	hint.row = r;
	hint.col = 2u;
	hint.len = (uint8_t)strlen(buf);
	hint.attr = UI_ATTR_ACCENT;
	(void)ui_surface_hint(s, &hint);

	r = (uint8_t)(r + 3u);
	if (r < s->rows && s->cols > 4u) {
		(void)ui_surface_fill(s, r, 2u, (size_t)(s->cols - 4u), ' ',
				      UI_ATTR_NORMAL);
		memset(&hint, 0, sizeof(hint));
		hint.kind = (uint8_t)UI_HINT_PROGRESS;
		hint.row = r;
		hint.col = 2u;
		hint.len = (uint8_t)(s->cols - 4u);
		hint.attr = UI_ATTR_ACCENT;
		hint.value = (permille > 1000u) ? 1000u : (uint16_t)permille;
		(void)ui_surface_hint(s, &hint);
	}
}

static void page_confirm(const ui_ctx_t *ctx, ui_surface_t *s)
{
	const ui_frame_t *f = top_c(ctx);
	char buf[64];
	sb_t sb;
	uint8_t r = (uint8_t)BODY_TOP;
	uint8_t row;
	uint8_t col;

	sb_init(&sb, buf, sizeof(buf));
	sb_str(&sb, ui_menu_name(f->item));
	sb_ch(&sb, '?');
	(void)ui_surface_put(s, r, 1u, buf, UI_ATTR_ACCENT);

	if (menu_confirm_stages(f->item) > 1u) {
		sb_init(&sb, buf, sizeof(buf));
		sb_str(&sb, "confirmation ");
		sb_u32(&sb, (uint32_t)f->stage + 1u);
		sb_str(&sb, " of ");
		sb_u32(&sb, menu_confirm_stages(f->item));
		put_right(s, r, (uint8_t)(s->cols - 1u), buf, UI_ATTR_DIM);
	}

	r = (uint8_t)(BODY_TOP + 2u);
	switch (f->item) {
	case UI_MENU_REBOOT:
		(void)ui_surface_put(s, r, 1u,
				     "The unit restarts and the OCXO must",
				     UI_ATTR_NORMAL);
		(void)ui_surface_put(s, (uint8_t)(r + 1u), 1u,
				     "re-discipline: served time degrades",
				     UI_ATTR_NORMAL);
		(void)ui_surface_put(s, (uint8_t)(r + 2u), 1u,
				     "for several minutes.", UI_ATTR_NORMAL);
		break;
	case UI_MENU_FACTORY_RESET:
		if (f->stage == 0u) {
			(void)ui_surface_put(s, r, 1u,
					     "Erase ALL configuration and",
					     UI_ATTR_ALARM);
			(void)ui_surface_put(s, (uint8_t)(r + 1u), 1u,
					     "calibration, then reboot?",
					     UI_ATTR_ALARM);
		} else {
			(void)ui_surface_put(s, r, 1u,
					     "FINAL WARNING: survey position,",
					     UI_ATTR_ALARM);
			(void)ui_surface_put(s, (uint8_t)(r + 1u), 1u,
					     "INA228 trims and NTS keys are",
					     UI_ATTR_ALARM);
			(void)ui_surface_put(s, (uint8_t)(r + 2u), 1u,
					     "destroyed. Confirm again.",
					     UI_ATTR_ALARM);
		}
		break;
	default:
		(void)ui_surface_put(s, r, 1u, "Confirm this action?",
				     UI_ATTR_NORMAL);
		break;
	}

	row = body_bottom(s, false);
	col = (s->cols > 24u) ? (uint8_t)((unsigned int)(s->cols - 20u) / 2u)
			      : 0u;
	(void)ui_surface_put(s, row, col, "[ No ]",
			     (f->sel == 0u) ? UI_ATTR_INVERSE : UI_ATTR_DIM);
	(void)ui_surface_put(s, row, (uint8_t)(col + 12u), "[ Yes ]",
			     (f->sel == 1u)
				     ? (uint8_t)(UI_ATTR_INVERSE | UI_ATTR_ALARM)
				     : UI_ATTR_DIM);
}

/* ===================================================================== *
 *  Render entry point
 * ===================================================================== */

/** Footer hint text per page. */
static const char *footer_for(uint8_t page)
{
	switch (page) {
	case UI_PAGE_HOME:
		return "ENC select   ENTER open   MENU menu";
	case UI_PAGE_MENU:
		return "ENC move   ENTER select   BACK home";
	case UI_PAGE_EDIT:
		return "ENC adjust   ENTER save   BACK cancel";
	case UI_PAGE_CONFIRM:
		return "ENC choose   ENTER confirm   BACK cancel";
	case UI_PAGE_ALARMS:
		return "ENC move   ENTER ack   BACK home";
	default:
		return "ENC scroll   BACK home";
	}
}

int ui_render(ui_ctx_t *ctx, const quality_block_t *q, const ui_health_t *h,
	      ui_surface_t *s)
{
	uint8_t page;

	if (ctx == NULL || q == NULL || h == NULL || !surf_bound(s) ||
	    !ctx->started) {
		return -EINVAL;
	}

	ui_surface_clear(s);
	ctx->renders++;

	/* Remember the geometry so a later touch can be mapped onto a cell. */
	ctx->geom_rows = s->rows;
	ctx->geom_cols = s->cols;
	ctx->geom_cell_h = (s->cell_h != 0u) ? s->cell_h : 16u;

	page = top(ctx)->page;

	draw_header(ctx, h, s);
	draw_status(q, h, s);

	switch (page) {
	case UI_PAGE_HOME:
		page_home(q, h, s);
		draw_pager(ctx, s);
		break;
	case UI_PAGE_SKYPLOT:
		page_sky(h, s);
		break;
	case UI_PAGE_CLOCKS:
		page_clocks(q, h, s);
		break;
	case UI_PAGE_NETWORK:
		page_network(h, s);
		break;
	case UI_PAGE_POWER:
		page_power(ctx, h, s);
		break;
	case UI_PAGE_ALARMS:
		page_alarms(ctx, h, s);
		break;
	case UI_PAGE_MENU:
		page_menu(ctx, s);
		break;
	case UI_PAGE_EDIT:
		page_edit(ctx, s);
		break;
	case UI_PAGE_CONFIRM:
		page_confirm(ctx, s);
		break;
	default:
		(void)ui_surface_put(s, (uint8_t)BODY_TOP, 1u,
				     "(no such page)", UI_ATTR_ALARM);
		break;
	}

	draw_footer(s, footer_for(page));
	return 0;
}

/* ===================================================================== *
 *  Small accessors
 * ===================================================================== */

ui_page_t ui_page(const ui_ctx_t *ctx)
{
	if (ctx == NULL || !ctx->started) {
		return UI_PAGE_HOME;
	}
	return (ui_page_t)top_c(ctx)->page;
}

uint8_t ui_depth(const ui_ctx_t *ctx)
{
	return (ctx == NULL) ? 0u : ctx->depth;
}

uint8_t ui_selection(const ui_ctx_t *ctx)
{
	if (ctx == NULL || !ctx->started) {
		return 0u;
	}
	return top_c(ctx)->sel;
}

bool ui_awake(const ui_ctx_t *ctx)
{
	return (ctx != NULL) && ctx->awake;
}

bool ui_identify(const ui_ctx_t *ctx)
{
	return (ctx != NULL) && ctx->identify;
}
