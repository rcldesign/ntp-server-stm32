/*
 * STS1000 "Meridian" — 8x16 bitmap font for the panel rasteriser.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * FONT DATA LICENSE / PROVENANCE
 * ------------------------------
 * The glyph bitmaps in ui_font8x16.c are ORIGINAL WORK, hand-authored for this
 * project (a 5x7 body with two descender rows, drawn at x=1,y=4 inside the 8x16
 * cell). No glyph was traced, copied, or derived from any existing typeface or
 * font file. To keep it freely reusable the bitmap table is dedicated to the
 * public domain under Creative Commons CC0 1.0
 * (https://creativecommons.org/publicdomain/zero/1.0/); the surrounding C
 * source follows the repository's Apache-2.0 like every other file.
 *
 * Format: printable ASCII 0x20..0x7E, one uint8_t per pixel row, 16 rows per
 * glyph, bit 7 = leftmost pixel of the cell. Non-printable and >0x7E code
 * points fall back to the 0x20 (space) cell via ui_font8x16_glyph().
 */

#ifndef STS1000_ZEPHYR_UI_UI_FONT8X16_H_
#define STS1000_ZEPHYR_UI_UI_FONT8X16_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UI_FONT_W 8u
#define UI_FONT_H 16u
#define UI_FONT_FIRST 0x20u
#define UI_FONT_LAST 0x7Eu

/** Row bitmaps for one glyph, index [0x20..0x7E] - 0x20. */
extern const uint8_t ui_font8x16[UI_FONT_LAST - UI_FONT_FIRST + 1u][UI_FONT_H];

/**
 * Row bitmaps for @p c, clamped to the printable range: anything outside
 * 0x20..0x7E renders as a blank cell. Never NULL.
 */
const uint8_t *ui_font8x16_glyph(char c);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_UI_UI_FONT8X16_H_ */
