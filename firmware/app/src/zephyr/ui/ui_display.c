/*
 * STS1000 "Meridian" — ST7796 display glue: power sequencing, tile rasteriser,
 * dirty-region blitter and DISP_BL backlight.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * DRIVER DECISION (in-tree, not custom)
 * -------------------------------------
 * The panel is driven by Zephyr's in-tree `sitronix,st7796s` on top of the
 * MIPI-DBI mode-C SPI shim (`zephyr,mipi-dbi-spi`), declared in
 * app/dts/ui.overlay. That shim was chosen over a hand-rolled ST7796 init for
 * one concrete reason: it shares SPI4 correctly. MIPI_DBI_SPI_CONFIG_DT() takes
 * the chip select from the parent controller's cs-gpios at index `reg`, so the
 * display's `reg = <2>` selects &spi4 cs-gpios[2] = PE9 = DISP_CS and leaves
 * the NOR (index 0) and the digipot (index 1) untouched. Zephyr's SPI core
 * serialises transfers per controller and re-applies each device's spi_config,
 * so no application-level SPI4 mutex is needed. This file therefore never
 * touches the controller directly — it only calls the display API.
 *
 * POWER / RESET SEQUENCING (spec §6.5)
 * ------------------------------------
 * DISP_EN (PC11) gates the 5 V display rail (RT9742 U33) and the PCA9306 touch
 * translator; it is deferrable under PoE pressure, so it may be OFF when the UI
 * area starts. The panel is therefore brought up lazily, as a small state
 * machine driven by ui_display_blit():
 *
 *   OFF       -> assert DISP_EN, stamp the time, wait for the soft-start
 *   ENABLING  -> after the soft-start window, device_init() the DBI shim then
 *                the panel (the panel driver pulses DISP_RST/PA10 and runs its
 *                init table), blanking-off, init the touch controller
 *   READY     -> blit dirty regions
 *
 * A failed init backs off and retries; the render thread is never blocked
 * waiting on the rail. The one bounded block is the panel driver's own ~200 ms
 * reset+init, which runs once on the OFF->READY transition of the lowest
 * priority thread and never touches the timing path.
 *
 * RASTERISER
 * ----------
 * core/ui renders into an abstract text-tile surface (60x20 of 8x16 cells). We
 * diff it against a shadow of the previous frame (ui_surface_diff_rows), narrow
 * each changed row to the column span that actually moved (ui_surface_row_span)
 * and push only that rectangle as RGB565 — the §6.5 partial-region strategy
 * that bounds SPI traffic. The BIGNUM/PROGRESS/RULE overlay hints are drawn on
 * top of the affected rows, which are force-repainted each frame.
 *
 * THE SKYPLOT (spec §6.3)
 * -----------------------
 * UI_HINT_SKYPLOT is the one hint that is not a decoration on a text row: it
 * claims a rectangle of *rows* and asks for a polar plot in it. core/ui cannot
 * draw one — a text-tile surface has no way to express it — so the page leaves
 * the cells blank and this file fills them from core/ui/skyplot.h, with every
 * decision between the hint and the pixels coming from sts_sky_policy.h.
 *
 * Three properties are worth stating because getting any of them wrong is
 * silent:
 *
 *   1. The plot is repainted only when its CONTENT changes, not every frame.
 *      A 224x224 canvas is 100 KiB of RGB565; at the 10 Hz render tick that
 *      would be a megabyte a second of SPI4 traffic to redraw a picture whose
 *      inputs (UBX-NAV-SAT, the 1 Hz e-compass sweep) move at 1 Hz. The gate is
 *      a content signature plus "did a text row inside the band get repainted",
 *      because a repainted row paints black over whatever was under it.
 *
 *   2. When the page stops asking for a plot, the rectangle is blanked. The
 *      sky page's TABLE view writes text over the top of the band and leaves
 *      the bottom of it blank, so without an explicit clear the lower half of
 *      the previous plot would sit under the table indefinitely.
 *
 *   3. The "north unverified" label is stamped into the INDEXED canvas rather
 *      than composited over the RGB565 strips. sky_render()'s badge is two
 *      pixel rows — a graphical marker, because core owns no font — and the
 *      words have to go somewhere the operator will read them. Stamping into
 *      the palette buffer keeps the caption transparent (glyph pixels only)
 *      when north IS verified, which is the difference between a caption and a
 *      black box over the southern horizon.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ui/skyplot.h"
#include "ui/ui.h"
#include "ui/ui_font8x16.h"
#include "zephyr/ui/sts_sky_policy.h"
#include "zephyr/ui/sts_ui.h"

LOG_MODULE_REGISTER(sts_ui_display, CONFIG_STS1000_LOG_LEVEL);

/* ------------------------------------------------------------------ config */

#define ZEPHYR_USER DT_PATH(zephyr_user)
#define DISP_NODE DT_NODELABEL(display)
#define DBI_NODE DT_NODELABEL(disp_dbi)
#define BL_PWM_NODE DT_NODELABEL(panel_pwm)

/* DISP_BL is TIM15_CH2 (PE6). TIM15_CH1 (PE5) is the fan at 25 kHz, and the ARR
 * (period) is shared across the timer's channels — see the sharing note in the
 * file trailer. We therefore drive the backlight at exactly the fan's period so
 * a pwm_set() here only rewrites CCR2 and never disturbs the fan's ARR. */
#define BL_CHANNEL 2u
#define BL_PERIOD_NS (1000000000U / 25000U) /* 25 kHz, == FAN_PWM_PERIOD_NS */

/* Soft-start window after DISP_EN before the controller is touched (spec §6.5:
 * RT9742 soft-starts 5V_DISP and V_DISP_EN_FAULT is masked during it). */
#define DISP_SOFTSTART_MS 60u
/* Backoff between failed init attempts, so a wedged/absent panel does not spin
 * the render thread on device_init(). */
#define DISP_RETRY_MS 2000u

/* ------------------------------------------------------------------ state */

static const struct device *const disp = DEVICE_DT_GET(DISP_NODE);
static const struct device *const dbi = DEVICE_DT_GET(DBI_NODE);
static const struct device *const bl_pwm = DEVICE_DT_GET(BL_PWM_NODE);
static const struct gpio_dt_spec disp_en =
	GPIO_DT_SPEC_GET(ZEPHYR_USER, disp_en_gpios);

enum disp_state {
	DISP_OFF = 0,   /* rail not yet asserted */
	DISP_ENABLING,  /* rail asserted, waiting on soft-start / retrying */
	DISP_READY,     /* controller initialised */
};

static enum disp_state disp_state;
static uint32_t disp_enable_ms;  /* mono_ms at which DISP_EN was asserted */
static uint32_t disp_next_try_ms;/* earliest mono_ms for the next init attempt */
static bool disp_ever_ready;
static uint16_t bl_last_permille = 0xFFFFu; /* force the first duty write */

/* Shadow of the last surface pushed, for the per-row diff. */
static char shadow_ch[STS_UI_ROWS * STS_UI_COLS];
static uint8_t shadow_attr[STS_UI_ROWS * STS_UI_COLS];
static ui_surface_t shadow;
static bool shadow_valid;

/*
 * RGB565 scratch for one blit rectangle. Sized for a full 60-column text row
 * at 8x16 (60*8 * 16 = 7680 px). A BIGNUM band is at most a dozen columns at
 * double height (12*8 * 32 = 3072 px), so it fits the same buffer.
 */
#define BLIT_MAX_PX (STS_UI_COLS * UI_FONT_W * UI_FONT_H)
static uint16_t blit[BLIT_MAX_PX];

/* ---------------------------------------------------------------- skyplot */

/*
 * Indexed-colour canvas for the polar plot, one octet per pixel.
 *
 * 224 px is the as-built band, not a round number: core/ui gives the plot rows
 * BODY_TOP+3 .. rows-2, which on the 60x20 grid is 14 rows of 16 px. The band
 * is 480 px wide, so the square is height-limited and sts_sky_layout() will ask
 * for exactly 224. Sizing the buffer to the band rather than to SKY_MAX_DIM
 * (320) saves 52 KiB of SRAM that no geometry on this panel can use.
 *
 * A larger grid — a future panel, or a debug surface — is not a fault: the
 * layout call is given this as its `max_side` ceiling and simply centres a
 * smaller square in the taller band.
 */
static uint8_t sky_px[STS_UI_SKY_MAX_SIDE * STS_UI_SKY_MAX_SIDE];
static sky_canvas_t sky_canvas;

/*
 * The console dump renders into its own small canvas rather than sharing the
 * panel's. Two reasons, and the first is the one that matters: the shell runs
 * on the console thread and the panel canvas is written by ui_local, so sharing
 * the buffer would be a data race on 50 KiB. The second is that the panel
 * canvas is 224 rows of 224 characters — 50 KiB of console output for a picture
 * an operator wants to glance at.
 */
#define SKY_ASCII_SIDE 31u
static uint8_t sky_ascii_px[SKY_ASCII_SIDE * SKY_ASCII_SIDE];

/*
 * Published by sts_ui.c once per render tick, consumed here and by the console
 * dump. The mutex is not for the render path — ui_display_sky_set() and
 * ui_display_blit() are both called from ui_local, in that order — it is for
 * the shell, which reads this from another thread and would otherwise dump a
 * frame torn across the memcpy.
 */
static sts_ui_sky_t sky_in;
static K_MUTEX_DEFINE(sky_mutex);

/* What is currently ON the panel, so a repaint can be skipped or a stale plot
 * blanked. Touched only by the render thread. */
static bool sky_drawn;
static sts_sky_layout_t sky_at;
static uint32_t sky_sig;

/* Palette index -> RGB565, byte-swapped for the wire. Built once at init so the
 * per-pixel inner loop is a table lookup and not a switch. */
static uint16_t sky_lut[SKY_C__COUNT];

/* ------------------------------------------------------------------ colour */

/* RGB565, big-endian on the wire (Zephyr PIXEL_FORMAT_RGB_565 convention). */
#define RGB565(r, g, b)                                                        \
	((uint16_t)((((r) & 0xF8u) << 8) | (((g) & 0xFCu) << 3) | ((b) >> 3)))

#define C_BLACK RGB565(0x00, 0x00, 0x00)
#define C_WHITE RGB565(0xE0, 0xE0, 0xE0)
#define C_GRAY RGB565(0x70, 0x78, 0x80)
#define C_CYAN RGB565(0x30, 0xC0, 0xE0)
#define C_GREEN RGB565(0x30, 0xC0, 0x40)
#define C_AMBER RGB565(0xF0, 0xB0, 0x20)
#define C_RED RGB565(0xE0, 0x30, 0x30)

/** Map a tile attribute to a foreground/background pair. */
static void attr_colors(uint8_t attr, uint16_t *fg, uint16_t *bg)
{
	uint16_t f = C_WHITE;

	if ((attr & UI_ATTR_ALARM) != 0u) {
		f = C_RED;
	} else if ((attr & UI_ATTR_WARN) != 0u) {
		f = C_AMBER;
	} else if ((attr & UI_ATTR_OK) != 0u) {
		f = C_GREEN;
	} else if ((attr & UI_ATTR_ACCENT) != 0u) {
		f = C_CYAN;
	} else if ((attr & UI_ATTR_DIM) != 0u) {
		f = C_GRAY;
	}

	if ((attr & UI_ATTR_INVERSE) != 0u) {
		/* Selected row: paint the colour as the background, text black. */
		*bg = f;
		*fg = C_BLACK;
	} else {
		*fg = f;
		*bg = C_BLACK;
	}
}

/* Convert big-endian RGB565 for the ST7796 8-bit MIPI-DBI stream: the shim
 * ships bytes in array order, and the controller wants the high byte first. */
static inline uint16_t be16(uint16_t v)
{
	return (uint16_t)((v >> 8) | (v << 8));
}

/* ------------------------------------------------------------- rasterising */

/**
 * Rasterise one text cell into @p px (a @p pitch-wide RGB565 sub-image) at
 * pixel origin (dx, dy), optionally scaled @p scale x.
 */
static void raster_cell(uint16_t *px, unsigned int pitch, unsigned int dx,
			unsigned int dy, char c, uint8_t attr, unsigned int scale)
{
	const uint8_t *rows = ui_font8x16_glyph(c);
	uint16_t fg;
	uint16_t bg;
	unsigned int gy;

	attr_colors(attr, &fg, &bg);
	fg = be16(fg);
	bg = be16(bg);

	for (gy = 0u; gy < UI_FONT_H; gy++) {
		uint8_t bits = rows[gy];
		unsigned int gx;

		for (gx = 0u; gx < UI_FONT_W; gx++) {
			uint16_t v = ((bits & (0x80u >> gx)) != 0u) ? fg : bg;
			unsigned int sx;
			unsigned int sy;

			for (sy = 0u; sy < scale; sy++) {
				unsigned int y = dy + (gy * scale) + sy;

				for (sx = 0u; sx < scale; sx++) {
					unsigned int x = dx + (gx * scale) + sx;

					px[(y * pitch) + x] = v;
				}
			}
		}
	}
}

/** Fill a @p pitch-wide sub-image rectangle with one colour. */
static void raster_fill(uint16_t *px, unsigned int pitch, unsigned int x0,
			unsigned int y0, unsigned int w, unsigned int h,
			uint16_t colour)
{
	uint16_t v = be16(colour);
	unsigned int y;

	for (y = 0u; y < h; y++) {
		unsigned int x;

		for (x = 0u; x < w; x++) {
			px[((y0 + y) * pitch) + x0 + x] = v;
		}
	}
}

/** Push one rectangle of the blit scratch to the panel. */
static int push_region(unsigned int col_px, unsigned int row_px,
		       unsigned int w_px, unsigned int h_px)
{
	struct display_buffer_descriptor desc = {
		.buf_size = (uint32_t)w_px * h_px * sizeof(uint16_t),
		.width = (uint16_t)w_px,
		.height = (uint16_t)h_px,
		.pitch = (uint16_t)w_px,
		.frame_incomplete = false,
	};

	return display_write(disp, (uint16_t)col_px, (uint16_t)row_px, &desc,
			     blit);
}

/** Find the BIGNUM hint whose top row is @p row, if any. */
static const ui_hint_t *bignum_at(const ui_surface_t *s, uint8_t row)
{
	uint8_t i;

	for (i = 0u; i < s->hint_count; i++) {
		if (s->hint[i].kind == (uint8_t)UI_HINT_BIGNUM &&
		    s->hint[i].row == row) {
			return &s->hint[i];
		}
	}
	return NULL;
}

/** Draw the PROGRESS and RULE overlays that fall on @p row, into the scratch. */
static void overlay_row(const ui_surface_t *s, uint8_t row, unsigned int span_c0,
			unsigned int span_w_cols)
{
	uint8_t i;
	unsigned int pitch = span_w_cols * UI_FONT_W;

	for (i = 0u; i < s->hint_count; i++) {
		const ui_hint_t *h = &s->hint[i];
		unsigned int hx;
		unsigned int hw;
		uint16_t fg;
		uint16_t bg;

		if (h->row != row) {
			continue;
		}
		if (h->kind != (uint8_t)UI_HINT_PROGRESS &&
		    h->kind != (uint8_t)UI_HINT_RULE) {
			continue;
		}
		if (h->col < span_c0) {
			continue;
		}

		attr_colors(h->attr, &fg, &bg);
		hx = ((unsigned int)h->col - span_c0) * UI_FONT_W;
		hw = (unsigned int)h->len * UI_FONT_W;
		if (hx + hw > pitch) {
			hw = (pitch > hx) ? (pitch - hx) : 0u;
		}
		if (hw == 0u) {
			continue;
		}

		if (h->kind == (uint8_t)UI_HINT_RULE) {
			/* A one-pixel line across the vertical middle. */
			raster_fill(blit, pitch, hx, UI_FONT_H / 2u, hw, 1u,
				    fg);
		} else {
			/* Track (dim) + fill (attr colour) proportional to value. */
			unsigned int fill =
				(hw * (h->value > 1000u ? 1000u : h->value)) /
				1000u;
			unsigned int inset = 2u;
			unsigned int bar_y = inset;
			unsigned int bar_h = (UI_FONT_H > 2u * inset)
						     ? (UI_FONT_H - 2u * inset)
						     : UI_FONT_H;

			raster_fill(blit, pitch, hx, bar_y, hw, bar_h, C_GRAY);
			if (fill > 0u) {
				raster_fill(blit, pitch, hx, bar_y, fill, bar_h,
					    fg);
			}
		}
	}
}

/**
 * Rasterise and push one changed row. Handles the three row kinds:
 *   - a BIGNUM band (this row is a hint's top row): 2x glyphs, 32 px tall;
 *   - a BIGNUM tail (the row below a bignum): drawn as part of the band above,
 *     so nothing to do here;
 *   - a normal text row, optionally carrying PROGRESS/RULE overlays.
 */
static int blit_row(const ui_surface_t *s, uint8_t row)
{
	const ui_hint_t *big = bignum_at(s, row);
	size_t base = (size_t)row * s->cols;

	if (ui_surface_is_bignum_tail(s, row)) {
		return 0; /* painted by its band's top row */
	}

	if (big != NULL) {
		unsigned int c0 = big->col;
		unsigned int ncol = big->len;
		/*
		 * The stride must be the width actually WRITTEN, not the width
		 * of the source cells.
		 *
		 * raster_cell() below is called with scale 2 at x = i*UI_FONT_W*2,
		 * so `ncol` glyphs occupy ncol*16 px. This was ncol*8 — the
		 * source-cell width — so the writer laid out ncol*16 columns at a
		 * stride of ncol*8. Two consequences, both real:
		 *
		 *   * every glyph past the halfway point wrote into the NEXT
		 *     buffer row. On the Home page's big clock (12 columns)
		 *     glyph 6 landed at offset y*96 + 96, so the second half of
		 *     the time overwrote the first half shifted down one pixel
		 *     row;
		 *   * the BLIT_MAX_PX guard admitted ncol up to 30, at which the
		 *     highest index written is 31*(30*8) + 30*16 - 1 = 7919
		 *     against a 7680-element blit[] — a 480-byte out-of-bounds
		 *     write into whatever the linker placed next, which now
		 *     includes the 50 KiB skyplot canvas.
		 *
		 * The pages in tree emit ~12 columns, so the overflow was latent
		 * while the mis-render was not.
		 *
		 * NOTE on the contract: core/ui/ui.h describes UI_HINT_BIGNUM as
		 * "the cells at (row, col..col+len-1) with double-size glyphs",
		 * and ui_surface_hint() validates col+len against the grid — but
		 * `len` double-width glyphs need 2*len cells of room, so the
		 * validation is for the source extent and not the drawn one.
		 * Rather than reinterpret the hint here, this refuses to draw
		 * anything that would leave the panel, which turns a
		 * would-be-corrupting hint into a missing one.
		 */
		unsigned int pitch = ncol * UI_FONT_W * 2u;
		unsigned int h = 2u * UI_FONT_H;
		unsigned int i;

		if (ncol == 0u || (size_t)c0 + ncol > s->cols ||
		    (size_t)c0 * UI_FONT_W + pitch >
			    (size_t)s->cols * UI_FONT_W ||
		    (size_t)pitch * h > BLIT_MAX_PX) {
			return 0;
		}
		/* Background first, then the scaled glyphs. */
		raster_fill(blit, pitch, 0u, 0u, pitch, h, C_BLACK);
		for (i = 0u; i < ncol; i++) {
			raster_cell(blit, pitch, i * UI_FONT_W * 2u, 0u,
				    s->ch[base + c0 + i], big->attr, 2u);
		}
		return push_region(c0 * UI_FONT_W, (unsigned int)row * UI_FONT_H,
				   pitch, h);
	}

	{
		uint8_t c0 = 0u;
		uint8_t c1 = (uint8_t)(s->cols - 1u);
		unsigned int ncol;
		unsigned int pitch;
		unsigned int i;
		int rc;

		rc = ui_surface_row_span(s, shadow_valid ? &shadow : NULL, row,
					 &c0, &c1);
		if (rc == -ENOENT) {
			c0 = 0u; /* a hint may still need repainting */
			c1 = (uint8_t)(s->cols - 1u);
		} else if (rc != 0) {
			c0 = 0u;
			c1 = (uint8_t)(s->cols - 1u);
		}

		ncol = (unsigned int)(c1 - c0 + 1u);
		pitch = ncol * UI_FONT_W;
		for (i = 0u; i < ncol; i++) {
			raster_cell(blit, pitch, i * UI_FONT_W, 0u,
				    s->ch[base + c0 + i], s->attr[base + c0 + i],
				    1u);
		}
		overlay_row(s, row, c0, ncol);
		return push_region((unsigned int)c0 * UI_FONT_W,
				   (unsigned int)row * UI_FONT_H, pitch,
				   UI_FONT_H);
	}
}

/* --------------------------------------------------------------- skyplot */

/** Fill a pixel rectangle with black, in strips the blit scratch can hold. */
static int push_black(unsigned int x0, unsigned int y0, unsigned int w,
		      unsigned int h)
{
	unsigned int per = (w != 0u) ? (BLIT_MAX_PX / w) : 0u;
	unsigned int y = 0u;

	if (w == 0u || h == 0u || per == 0u) {
		return 0;
	}
	/* RGB565 black is 0x0000, and be16(0) is 0, so one memset serves every
	 * strip. */
	memset(blit, 0, (size_t)w * ((per < h) ? per : h) * sizeof(blit[0]));

	while (y < h) {
		unsigned int n = ((h - y) < per) ? (h - y) : per;
		int rc = push_region(x0, y0 + y, w, n);

		if (rc != 0) {
			return rc;
		}
		y += n;
	}
	return 0;
}

/** The frame's UI_HINT_SKYPLOT, or NULL. Only the first is honoured. */
static const ui_hint_t *skyplot_hint(const ui_surface_t *s)
{
	uint8_t i;

	for (i = 0u; i < s->hint_count; i++) {
		if (s->hint[i].kind == (uint8_t)UI_HINT_SKYPLOT) {
			return &s->hint[i];
		}
	}
	return NULL;
}

/**
 * Stamp one line of 8x16 text into the INDEXED canvas.
 *
 * @param opaque  true paints @p bg across the whole cell rectangle, so the text
 *                reads as a solid badge band; false leaves every non-glyph pixel
 *                as the plot drew it, so a caption does not punch a black hole
 *                in the southern horizon.
 *
 * Writes sky_canvas.px directly. That buffer is this file's — skyplot.h is
 * explicit that canvas storage is caller-owned — but core's own put_px() is
 * static, so the bounds check is repeated here rather than borrowed.
 */
static void sky_stamp(int32_t x0, int32_t y0, const char *str, size_t n,
		      uint8_t fg, uint8_t bg, bool opaque)
{
	size_t i;

	for (i = 0u; i < n; i++) {
		const uint8_t *rows = ui_font8x16_glyph(str[i]);
		unsigned int gy;

		for (gy = 0u; gy < UI_FONT_H; gy++) {
			int32_t py = y0 + (int32_t)gy;
			unsigned int gx;

			if (py < 0 || py >= (int32_t)sky_canvas.h) {
				continue;
			}
			for (gx = 0u; gx < UI_FONT_W; gx++) {
				int32_t px = x0 + (int32_t)(i * UI_FONT_W) +
					     (int32_t)gx;
				bool on = (rows[gy] & (0x80u >> gx)) != 0u;

				if (px < 0 || px >= (int32_t)sky_canvas.w) {
					continue;
				}
				if (!on && !opaque) {
					continue;
				}
				sky_canvas.px[((size_t)py * sky_canvas.w) +
					      (size_t)px] = on ? fg : bg;
			}
		}
	}
}

/**
 * Write the §6.3 north verdict across the bottom of the canvas.
 *
 * Unverified: black on SKY_C_BADGE, which thickens sky_render()'s two-pixel
 * badge into something an operator reads from across the room. That costs the
 * southernmost strip of the plot, which is the right trade in exactly the state
 * where the azimuths are not trustworthy anyway.
 *
 * Verified: the words alone, in the grid's bright colour, transparent over the
 * plot — "TRUE NORTH" and "NORTH APPROX (MODEL)" are different enough to matter
 * (the modelled declination is good only to 10-15 degrees) and neither is worth
 * a black band.
 *
 * A canvas too narrow for the whole string gets no text at all. Half a label is
 * worse than none: sky_render()'s graphical badge still says the orientation is
 * suspect, and "NORT" says nothing.
 */
static void sky_label(void)
{
	bool unverified = sts_sky_north_unverified(sky_in.north_reason);
	const char *label = sts_sky_north_label(sky_in.north_reason);
	size_t n = strlen(label);
	unsigned int w = (unsigned int)n * UI_FONT_W;

	if (n == 0u || w > sky_canvas.w || UI_FONT_H > sky_canvas.h) {
		return;
	}

	sky_stamp((int32_t)((sky_canvas.w - w) / 2u),
		  (int32_t)sky_canvas.h - (int32_t)UI_FONT_H, label, n,
		  unverified ? (uint8_t)SKY_C_BG : (uint8_t)SKY_C_GRID_MAJOR,
		  (uint8_t)SKY_C_BADGE, unverified);
}

/** Render the published sky into @p c and stamp the north label over it. */
static int sky_compose(sky_canvas_t *c, uint8_t *px, uint16_t side, size_t cap)
{
	int rc = sky_canvas_init(c, px, side, side, cap);

	if (rc != 0) {
		return rc;
	}
	rc = sky_render(c, sky_in.sv, sky_in.sv_count,
			sky_in.orient.north_valid ? &sky_in.orient : NULL);
	if (rc != 0) {
		return rc;
	}
	return 0;
}

/** Convert the composed canvas to RGB565 and push it, in horizontal strips. */
static int sky_push(void)
{
	unsigned int side = sky_canvas.w;
	unsigned int per = BLIT_MAX_PX / side;
	unsigned int y = 0u;

	if (per == 0u) {
		return -ENOSPC;
	}

	while (y < side) {
		unsigned int n = ((side - y) < per) ? (side - y) : per;
		unsigned int row;
		int rc;

		for (row = 0u; row < n; row++) {
			unsigned int x;

			for (x = 0u; x < side; x++) {
				/* sky_canvas_get() rather than indexing px[]:
				 * it is the module's read accessor and it is
				 * what defines the out-of-bounds answer. The
				 * clamp below is still needed — it guards the
				 * LUT against a palette that grows without this
				 * file's table growing with it. */
				uint8_t idx = sky_canvas_get(
					&sky_canvas, (int32_t)x,
					(int32_t)(y + row));

				blit[(row * side) + x] =
					sky_lut[(idx < (uint8_t)SKY_C__COUNT)
							? idx
							: (uint8_t)SKY_C_BG];
			}
		}

		rc = push_region(sky_at.x0, sky_at.y0 + y, side, n);
		if (rc != 0) {
			return rc;
		}
		y += n;
	}
	return 0;
}

/**
 * Draw, skip or erase the polar plot for this frame.
 *
 * @param dirty  Rows this frame repainted. A repaint inside the plot's band
 *               paints black over it, so it forces a redraw even when nothing
 *               about the sky itself moved.
 */
static void skyplot_frame(const ui_surface_t *surf, uint32_t dirty)
{
	const ui_hint_t *h = skyplot_hint(surf);
	sts_sky_layout_t lay;
	uint32_t sig;
	bool clobbered;
	int rc;

	if (h == NULL) {
		/*
		 * No plot this frame. If one is on the panel it has to go: the
		 * sky page's TABLE view writes text over the top of the band and
		 * leaves the bottom blank, so the text diff alone would erase
		 * part of the plot and leave the rest.
		 */
		if (sky_drawn) {
			(void)push_black(sky_at.x0, sky_at.y0, sky_at.side,
					 sky_at.side);
			sky_drawn = false;
		}
		return;
	}

	rc = sts_sky_layout(h, surf->rows, surf->cols, surf->cell_w,
			    surf->cell_h, (uint16_t)STS_UI_SKY_MAX_SIDE, &lay);
	if (rc != 0) {
		/* The band cannot hold a legible plot (or the hint does not fit
		 * the grid). Leave the cells blank rather than paint a smudge —
		 * and erase a plot placed by an earlier, larger geometry. */
		if (sky_drawn) {
			(void)push_black(sky_at.x0, sky_at.y0, sky_at.side,
					 sky_at.side);
			sky_drawn = false;
		}
		return;
	}

	clobbered = (dirty & sts_sky_band_mask(h, surf->rows)) != 0u;

	(void)k_mutex_lock(&sky_mutex, K_FOREVER);
	sig = sts_sky_repaint_sig(&lay, sky_in.sv, sky_in.sv_count,
				  sky_in.orient.north_valid ? &sky_in.orient
							    : NULL,
				  sky_in.north_reason);

	if (sky_drawn && !clobbered && sig == sky_sig &&
	    lay.x0 == sky_at.x0 && lay.y0 == sky_at.y0 &&
	    lay.side == sky_at.side) {
		k_mutex_unlock(&sky_mutex);
		return; /* the panel already shows exactly this picture */
	}

	rc = sky_compose(&sky_canvas, sky_px, lay.side, sizeof(sky_px));
	if (rc == 0) {
		sky_label();
	}
	k_mutex_unlock(&sky_mutex);

	if (rc != 0) {
		LOG_WRN("skyplot render failed (%d)", rc);
		return;
	}

	/*
	 * A plot smaller than the one on the panel leaves a border of the old
	 * one behind, so clear the previous rectangle before drawing the new
	 * one. Only on a geometry change: the common case is the same rectangle
	 * being overwritten in full.
	 */
	if (sky_drawn && (lay.x0 != sky_at.x0 || lay.y0 != sky_at.y0 ||
			  lay.side != sky_at.side)) {
		(void)push_black(sky_at.x0, sky_at.y0, sky_at.side,
				 sky_at.side);
	}

	sky_at = lay;
	rc = sky_push();
	if (rc != 0) {
		LOG_WRN("skyplot blit failed (%d)", rc);
		sky_drawn = false; /* retry the whole plot next frame */
		return;
	}
	sky_drawn = true;
	sky_sig = sig;
}

void ui_display_sky_set(const sts_ui_sky_t *s)
{
	(void)k_mutex_lock(&sky_mutex, K_FOREVER);
	if (s == NULL) {
		memset(&sky_in, 0, sizeof(sky_in));
		sky_in.north_reason = (uint8_t)STS_SKY_NORTH_NO_SAMPLE;
	} else {
		sky_in = *s;
		if (sky_in.sv_count > (uint8_t)UI_MAX_SV) {
			sky_in.sv_count = (uint8_t)UI_MAX_SV;
		}
	}
	k_mutex_unlock(&sky_mutex);
}

size_t ui_display_sky_ascii(char *out, size_t cap)
{
	sky_canvas_t c;
	size_t n;

	if (out == NULL || cap == 0u) {
		return 0u;
	}

	/*
	 * Rendered here rather than copied out of the panel canvas: the caller
	 * is the shell, on another thread and at another size. Under the same
	 * mutex sts_ui.c publishes through, so the picture is one consistent
	 * frame and not a memcpy caught mid-update.
	 */
	(void)k_mutex_lock(&sky_mutex, K_FOREVER);
	if (sky_compose(&c, sky_ascii_px, (uint16_t)SKY_ASCII_SIDE,
			sizeof(sky_ascii_px)) != 0) {
		k_mutex_unlock(&sky_mutex);
		return 0u;
	}
	n = sky_canvas_to_ascii(&c, out, cap);
	k_mutex_unlock(&sky_mutex);

	return n;
}

/* ------------------------------------------------------------- lifecycle */

static uint32_t now_ms(void)
{
	return (uint32_t)k_uptime_get();
}

/** Advance the power/init state machine at most one step. 0 = READY. */
static int ensure_panel(void)
{
	uint32_t t = now_ms();
	int rc;

	switch (disp_state) {
	case DISP_READY:
		return 0;

	case DISP_OFF:
		if (!gpio_is_ready_dt(&disp_en)) {
			return -ENODEV;
		}
		rc = gpio_pin_configure_dt(&disp_en, GPIO_OUTPUT_ACTIVE);
		if (rc != 0) {
			LOG_ERR("DISP_EN assert failed (%d)", rc);
			return rc;
		}
		disp_enable_ms = t;
		disp_next_try_ms = t + DISP_SOFTSTART_MS;
		disp_state = DISP_ENABLING;
		LOG_INF("DISP_EN asserted; waiting %u ms for 5V_DISP soft-start",
			DISP_SOFTSTART_MS);
		return -EAGAIN;

	case DISP_ENABLING:
		if ((int32_t)(t - disp_next_try_ms) < 0) {
			return -EAGAIN;
		}
		if (!device_is_ready(bl_pwm)) {
			LOG_WRN("DISP_BL PWM not ready");
		}
		/* Bring up the DBI shim, then the panel (which pulses DISP_RST
		 * and runs its init table). Both are zephyr,deferred-init. */
		rc = device_init(dbi);
		if (rc != 0 && rc != -EALREADY) {
			LOG_WRN("MIPI-DBI shim init failed (%d); retrying", rc);
			disp_next_try_ms = t + DISP_RETRY_MS;
			return -EAGAIN;
		}
		rc = device_init(disp);
		if (rc != 0 && rc != -EALREADY) {
			LOG_WRN("ST7796 init failed (%d); retrying", rc);
			disp_next_try_ms = t + DISP_RETRY_MS;
			return -EAGAIN;
		}
		(void)display_blanking_off(disp);

		/* DISP_RST also reset the FT6336U; it is now out of reset. */
		(void)ui_input_touch_init();

		shadow_valid = false; /* force a full first repaint */
		disp_state = DISP_READY;
		disp_ever_ready = true;
		LOG_INF("ST7796 + FT6336U up (%ux%u, 8x16 tiles %ux%u)",
			(unsigned int)STS_UI_COLS * UI_FONT_W,
			(unsigned int)STS_UI_ROWS * UI_FONT_H,
			STS_UI_COLS, STS_UI_ROWS);
		return 0;

	default:
		return -EINVAL;
	}
}

int ui_display_init(void)
{
	unsigned int i;
	int rc;

	if (!device_is_ready(bl_pwm)) {
		LOG_WRN("DISP_BL PWM device not ready at init");
	} else {
		/* Backlight off until the UI sets a duty. */
		(void)pwm_set(bl_pwm, BL_CHANNEL, BL_PERIOD_NS, 0u, 0);
	}

	rc = ui_surface_init(&shadow, (uint8_t)STS_UI_ROWS, (uint8_t)STS_UI_COLS,
			     shadow_ch, shadow_attr, sizeof(shadow_ch));
	if (rc != 0) {
		return rc;
	}

	for (i = 0u; i < (unsigned int)SKY_C__COUNT; i++) {
		sky_lut[i] = be16(sts_sky_palette_rgb565((uint8_t)i));
	}
	sky_drawn = false;
	memset(&sky_at, 0, sizeof(sky_at));
	ui_display_sky_set(NULL);

	shadow_valid = false;
	disp_state = DISP_OFF;
	return 0;
}

int ui_display_blit(const ui_surface_t *surf)
{
	uint32_t dirty = 0u;
	uint32_t hint_rows = 0u;
	uint8_t i;
	uint8_t r;
	int rc;

	if (surf == NULL) {
		return -EINVAL;
	}
	if (surf->rows != shadow.rows || surf->cols != shadow.cols) {
		return -EDOM;
	}

	rc = ensure_panel();
	if (rc != 0) {
		return rc; /* -EAGAIN while powering/retrying; never blocks */
	}

	/* Rows changed by text/attr since the last frame. */
	rc = ui_surface_diff_rows(surf, shadow_valid ? &shadow : NULL, &dirty);
	if (rc != 0) {
		return rc;
	}

	/*
	 * Overlay hints (BIGNUM/PROGRESS/RULE) are data-driven and not captured
	 * by the text diff, so force-repaint every row a hint touches — plus the
	 * tail row of each bignum band.
	 */
	for (i = 0u; i < surf->hint_count; i++) {
		uint8_t hr = surf->hint[i].row;

		if (hr < surf->rows) {
			hint_rows |= (uint32_t)1u << hr;
		}
		if (surf->hint[i].kind == (uint8_t)UI_HINT_BIGNUM &&
		    (uint8_t)(hr + 1u) < surf->rows) {
			hint_rows |= (uint32_t)1u << (hr + 1u);
		}
	}
	dirty |= hint_rows;

	for (r = 0u; r < surf->rows; r++) {
		if ((dirty & ((uint32_t)1u << r)) == 0u) {
			continue;
		}
		rc = blit_row(surf, r);
		if (rc != 0) {
			LOG_WRN("display_write row %u failed (%d)", r, rc);
			/* A transient bus error should not wedge the panel; the
			 * shadow is not updated for this row, so the next frame
			 * re-attempts it. */
			return rc;
		}
	}

	/*
	 * The polar plot goes on AFTER the text rows: its band is blank cells,
	 * and blit_row() paints those black. Drawing first would hand the
	 * blitter a picture and then erase it.
	 */
	skyplot_frame(surf, dirty);

	/* Adopt the frame as the new shadow. */
	memcpy(shadow.ch, surf->ch, (size_t)surf->rows * surf->cols);
	memcpy(shadow.attr, surf->attr, (size_t)surf->rows * surf->cols);
	shadow_valid = true;
	return 0;
}

void ui_display_backlight_permille(uint16_t permille)
{
	uint32_t pulse;

	if (permille > 1000u) {
		permille = 1000u;
	}
	if (permille == bl_last_permille) {
		return;
	}
	if (!device_is_ready(bl_pwm)) {
		return;
	}

	pulse = ((uint32_t)BL_PERIOD_NS * permille) / 1000u;
	if (pwm_set(bl_pwm, BL_CHANNEL, BL_PERIOD_NS, pulse, 0) == 0) {
		bl_last_permille = permille;
	}
}

bool ui_display_ready(void)
{
	return disp_ever_ready;
}

/*
 * TIM15 SHARING NOTE
 * ------------------
 * TIM15 carries FAN_PWM on CH1 (PE5, platform/hk.c) and DISP_BL on CH2 (PE6,
 * here). On STM32 the auto-reload (ARR = period) is a per-timer register while
 * the compare (CCRx = pulse) is per-channel, so two channels of one timer are
 * locked to a single period but keep independent duties. The fan requires
 * 25 kHz for a quiet 4-wire PWM; a LED backlight is indifferent to carrier
 * frequency well above the flicker threshold, so 25 kHz is perfectly acceptable
 * for it. Both this file and hk.c therefore call pwm_set() with the SAME
 * BL_PERIOD_NS/FAN_PWM_PERIOD_NS (40000 ns); a matching period means the STM32
 * PWM driver's LL_TIM_SetAutoReload() rewrites ARR to its current value (a
 * harmless glitchless reload) and only CCR2 changes. Driving the backlight at a
 * different period WOULD reprogram ARR and disturb the fan, which is why the
 * constant is duplicated rather than chosen independently.
 */
