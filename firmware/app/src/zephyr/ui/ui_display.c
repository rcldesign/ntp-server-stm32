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

#include "ui/ui.h"
#include "ui/ui_font8x16.h"
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
		unsigned int pitch = ncol * UI_FONT_W;
		unsigned int h = 2u * UI_FONT_H;
		unsigned int i;

		if (ncol == 0u || (size_t)c0 + ncol > s->cols ||
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
