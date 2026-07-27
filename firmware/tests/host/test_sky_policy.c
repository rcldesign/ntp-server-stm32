/*
 * STS1000 "Meridian" — host unit tests for the skyplot glue's decisions
 * (app/src/zephyr/ui/sts_sky_policy.h).
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * test_skyplot.c pins the renderer against golden images. This suite pins the
 * layer between the renderer and the panel: where the square canvas goes, which
 * UBX records become markers, whether the plot may be rotated to true north,
 * what the operator is told when it may not, and when the picture has to be
 * pushed to the display again.
 *
 * Every one of those is silent when it is wrong. An oversized canvas is an
 * out-of-bounds write into the framebuffer; an unfiltered NAV-SAT record paints
 * a phantom satellite at due north on the horizon, which is exactly where an
 * operator looks for an obstruction; a plausible-but-wrong rotation is worse
 * than an honest un-rotated one (spec §6.3); and a repaint gate that never
 * fires freezes the plot on a picture that looks perfectly current.
 *
 * The header is Zephyr-free by construction so all of it is reachable here.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "ui/skyplot.h"
#include "ui/ui.h"
#include "unity.h"
#include "zephyr/ui/sts_sky_policy.h"

/* ------------------------------------------------------------------ helpers */

/*
 * The as-built panel: a 480x320 ST7796 with an 8x16 font, so a 60x20 tile grid
 * (sts_ui.h STS_UI_COLS/ROWS). core/ui gives UI_PAGE_SKYPLOT the rows from
 * BODY_TOP+3 to rows-2 inclusive — 5..18, i.e. 14 rows — and the glue's canvas
 * ceiling is STS_UI_SKY_MAX_SIDE. Duplicated rather than included because
 * sts_ui.h is a Zephyr header; if these drift the layout assertions below stop
 * describing the product, which is the point of asserting them.
 */
#define AB_ROWS    20u
#define AB_COLS    60u
#define AB_CELL_W  8u
#define AB_CELL_H  16u
#define AB_PLOT_ROW 5u
#define AB_PLOT_LEN 14u
#define AB_MAX_SIDE 224u

static ui_hint_t hint(uint8_t kind, uint8_t row, uint8_t col, uint8_t len)
{
	ui_hint_t h;

	(void)memset(&h, 0, sizeof(h));
	h.kind = kind;
	h.row = row;
	h.col = col;
	h.len = len;
	return h;
}

static ui_hint_t as_built_hint(void)
{
	return hint((uint8_t)UI_HINT_SKYPLOT, (uint8_t)AB_PLOT_ROW, 0u,
		    (uint8_t)AB_PLOT_LEN);
}

static int layout_as_built(sts_sky_layout_t *out)
{
	ui_hint_t h = as_built_hint();

	return sts_sky_layout(&h, (uint8_t)AB_ROWS, (uint8_t)AB_COLS,
			      (uint8_t)AB_CELL_W, (uint8_t)AB_CELL_H,
			      (uint16_t)AB_MAX_SIDE, out);
}

/* ===================================================================== *
 *  Layout
 * ===================================================================== */

static void test_layout_as_built_geometry(void)
{
	sts_sky_layout_t l;

	(void)memset(&l, 0xAA, sizeof(l));
	TEST_ASSERT_EQUAL_INT(0, layout_as_built(&l));

	/*
	 * The band is 14 rows of 16 px = 224 px tall and 60 cols of 8 px =
	 * 480 px wide, so the square is height-limited at exactly 224 — and the
	 * canvas ceiling was sized to it, so neither clips the other.
	 */
	TEST_ASSERT_EQUAL_UINT16(224u, l.side);

	/* Centred: 128 px of margin each side, and no vertical slack at all. */
	TEST_ASSERT_EQUAL_UINT16(128u, l.x0);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)(AB_PLOT_ROW * AB_CELL_H), l.y0);

	/* It must actually fit the panel: 480x320. */
	TEST_ASSERT_TRUE((uint32_t)l.x0 + l.side <= 480u);
	TEST_ASSERT_TRUE((uint32_t)l.y0 + l.side <= 320u);
}

static void test_layout_is_centred_in_a_wider_band(void)
{
	/* Same band, but starting at column 4: 56 cols = 448 px wide. The
	 * square is still height-limited, and the extra margin is split. */
	ui_hint_t h = hint((uint8_t)UI_HINT_SKYPLOT, 5u, 4u, 14u);
	sts_sky_layout_t l;

	TEST_ASSERT_EQUAL_INT(0, sts_sky_layout(&h, (uint8_t)AB_ROWS,
						(uint8_t)AB_COLS,
						(uint8_t)AB_CELL_W,
						(uint8_t)AB_CELL_H,
						(uint16_t)AB_MAX_SIDE, &l));
	TEST_ASSERT_EQUAL_UINT16(224u, l.side);
	/* col 4 -> 32 px, plus (448-224)/2 = 112. */
	TEST_ASSERT_EQUAL_UINT16(32u + 112u, l.x0);
	TEST_ASSERT_EQUAL_UINT16(80u, l.y0);
}

static void test_layout_is_width_limited_in_a_narrow_band(void)
{
	/* 8 columns = 64 px wide, 14 rows = 224 px tall: width wins. */
	ui_hint_t h = hint((uint8_t)UI_HINT_SKYPLOT, 5u, 52u, 14u);
	sts_sky_layout_t l;

	TEST_ASSERT_EQUAL_INT(0, sts_sky_layout(&h, (uint8_t)AB_ROWS,
						(uint8_t)AB_COLS,
						(uint8_t)AB_CELL_W,
						(uint8_t)AB_CELL_H,
						(uint16_t)AB_MAX_SIDE, &l));
	TEST_ASSERT_EQUAL_UINT16(64u, l.side);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)(52u * 8u), l.x0);
	/* Vertically centred in the 224 px band: (224-64)/2 = 80. */
	TEST_ASSERT_EQUAL_UINT16((uint16_t)((5u * 16u) + 80u), l.y0);
}

static void test_layout_honours_the_canvas_ceiling(void)
{
	ui_hint_t h = as_built_hint();
	sts_sky_layout_t l;

	/* A caller with only a 64 px buffer gets a 64 px square, still centred
	 * — never a canvas larger than the storage it was handed. */
	TEST_ASSERT_EQUAL_INT(0, sts_sky_layout(&h, (uint8_t)AB_ROWS,
						(uint8_t)AB_COLS,
						(uint8_t)AB_CELL_W,
						(uint8_t)AB_CELL_H, 64u, &l));
	TEST_ASSERT_EQUAL_UINT16(64u, l.side);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)((480u - 64u) / 2u), l.x0);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)((5u * 16u) + ((224u - 64u) / 2u)),
				 l.y0);
}

static void test_layout_never_exceeds_the_renderers_maximum(void)
{
	/* A grid whose band is larger than SKY_MAX_DIM in both axes, and a
	 * caller willing to allocate for it. The renderer's ceiling still
	 * applies — sky_canvas_init() would refuse anything past it. */
	ui_hint_t h = hint((uint8_t)UI_HINT_SKYPLOT, 0u, 0u, 30u);
	sts_sky_layout_t l;

	TEST_ASSERT_EQUAL_INT(0, sts_sky_layout(&h, 32u, 80u, 16u, 32u,
						0xFFFFu, &l));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)SKY_MAX_DIM, l.side);
}

static void test_layout_rejects_bad_arguments(void)
{
	ui_hint_t h = as_built_hint();
	sts_sky_layout_t l;

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      sts_sky_layout(NULL, 20u, 60u, 8u, 16u, 224u, &l));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      sts_sky_layout(&h, 20u, 60u, 8u, 16u, 224u, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      sts_sky_layout(&h, 20u, 60u, 0u, 16u, 224u, &l));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      sts_sky_layout(&h, 20u, 60u, 8u, 0u, 224u, &l));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      sts_sky_layout(&h, 0u, 60u, 8u, 16u, 224u, &l));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      sts_sky_layout(&h, 20u, 0u, 8u, 16u, 224u, &l));

	/* Another hint kind is not a skyplot, however well it fits. */
	{
		ui_hint_t p = hint((uint8_t)UI_HINT_PROGRESS, 5u, 0u, 14u);

		TEST_ASSERT_EQUAL_INT(-EINVAL,
				      sts_sky_layout(&p, 20u, 60u, 8u, 16u,
						     224u, &l));
	}
}

static void test_layout_refuses_a_band_it_cannot_fill(void)
{
	sts_sky_layout_t l;

	/* Zero rows claimed. */
	{
		ui_hint_t h = hint((uint8_t)UI_HINT_SKYPLOT, 5u, 0u, 0u);

		TEST_ASSERT_EQUAL_INT(-EDOM,
				      sts_sky_layout(&h, 20u, 60u, 8u, 16u,
						     224u, &l));
	}
	/* Row outside the grid. */
	{
		ui_hint_t h = hint((uint8_t)UI_HINT_SKYPLOT, 20u, 0u, 1u);

		TEST_ASSERT_EQUAL_INT(-EDOM,
				      sts_sky_layout(&h, 20u, 60u, 8u, 16u,
						     224u, &l));
	}
	/* Column outside the grid. */
	{
		ui_hint_t h = hint((uint8_t)UI_HINT_SKYPLOT, 5u, 60u, 4u);

		TEST_ASSERT_EQUAL_INT(-EDOM,
				      sts_sky_layout(&h, 20u, 60u, 8u, 16u,
						     224u, &l));
	}
	/* Band overhangs the bottom of the grid: rows 18..20 on a 20-row grid.
	 * This is the one that would be an out-of-bounds framebuffer write. */
	{
		ui_hint_t h = hint((uint8_t)UI_HINT_SKYPLOT, 18u, 0u, 3u);

		TEST_ASSERT_EQUAL_INT(-EDOM,
				      sts_sky_layout(&h, 20u, 60u, 8u, 16u,
						     224u, &l));
	}
	/* Fits, but is below the legibility floor: one 16 px row. */
	{
		ui_hint_t h = hint((uint8_t)UI_HINT_SKYPLOT, 5u, 0u, 1u);

		TEST_ASSERT_EQUAL_INT(-EDOM,
				      sts_sky_layout(&h, 20u, 60u, 8u, 16u,
						     224u, &l));
	}
	/* Fits, but the caller's buffer is below the floor. */
	{
		ui_hint_t h = as_built_hint();

		TEST_ASSERT_EQUAL_INT(-EDOM,
				      sts_sky_layout(&h, 20u, 60u, 8u, 16u,
						     STS_SKY_MIN_SIDE - 1u,
						     &l));
	}
}

static void test_layout_accepts_exactly_the_minimum(void)
{
	ui_hint_t h = as_built_hint();
	sts_sky_layout_t l;

	TEST_ASSERT_EQUAL_INT(0, sts_sky_layout(&h, 20u, 60u, 8u, 16u,
						(uint16_t)STS_SKY_MIN_SIDE,
						&l));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)STS_SKY_MIN_SIDE, l.side);
}

static void test_layout_output_is_a_valid_canvas(void)
{
	/*
	 * The end-to-end property the arithmetic exists for: whatever geometry
	 * comes out, sky_canvas_init() must accept it against a buffer sized to
	 * max_side. A layout the renderer rejects is a blank plot; a layout it
	 * accepts but that overhangs the panel is a corrupted framebuffer.
	 */
	static uint8_t px[AB_MAX_SIDE * AB_MAX_SIDE];
	uint8_t rows;
	uint8_t len;

	for (rows = 4u; rows <= 20u; rows++) {
		for (len = 1u; len < rows; len++) {
			ui_hint_t h = hint((uint8_t)UI_HINT_SKYPLOT, 1u, 0u,
					   len);
			sts_sky_layout_t l;
			sky_canvas_t c;

			if (sts_sky_layout(&h, rows, (uint8_t)AB_COLS,
					   (uint8_t)AB_CELL_W,
					   (uint8_t)AB_CELL_H,
					   (uint16_t)AB_MAX_SIDE, &l) != 0) {
				continue;
			}
			TEST_ASSERT_EQUAL_INT(0,
					      sky_canvas_init(&c, px, l.side,
							      l.side,
							      sizeof(px)));
			TEST_ASSERT_TRUE((uint32_t)l.x0 + l.side <=
					 (uint32_t)AB_COLS * AB_CELL_W);
			TEST_ASSERT_TRUE((uint32_t)l.y0 + l.side <=
					 (uint32_t)rows * AB_CELL_H);
		}
	}
}

/* ===================================================================== *
 *  gnssId -> constellation
 * ===================================================================== */

static void test_gnss_id_mapping(void)
{
	/* UBX-NAV-SAT numbering. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_GNSS_GPS,
				sts_sky_sys_from_gnss_id(0u));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_GNSS_SBAS,
				sts_sky_sys_from_gnss_id(1u));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_GNSS_GALILEO,
				sts_sky_sys_from_gnss_id(2u));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_GNSS_BEIDOU,
				sts_sky_sys_from_gnss_id(3u));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_GNSS_QZSS,
				sts_sky_sys_from_gnss_id(5u));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_GNSS_GLONASS,
				sts_sky_sys_from_gnss_id(6u));

	/* 4 is IMES, which this receiver does not track: a distinct colour, not
	 * a silent promotion to GPS. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_GNSS_OTHER,
				sts_sky_sys_from_gnss_id(4u));
}

static void test_gnss_id_unknown_ids_are_never_a_guess(void)
{
	unsigned int id;

	for (id = 7u; id <= 255u; id++) {
		TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_GNSS_OTHER,
					sts_sky_sys_from_gnss_id((uint8_t)id));
	}

	/* And every mapping lands inside the enum, so sky_colour_for_sys()
	 * never has to fall back. */
	for (id = 0u; id <= 255u; id++) {
		TEST_ASSERT_TRUE(sts_sky_sys_from_gnss_id((uint8_t)id) <
				 (uint8_t)UI_GNSS__COUNT);
	}
}

/* ===================================================================== *
 *  UBX record -> marker
 * ===================================================================== */

static void test_sv_admits_a_measured_satellite(void)
{
	ui_sv_t sv;

	(void)memset(&sv, 0xAA, sizeof(sv));
	TEST_ASSERT_TRUE(sts_sky_sv_from_ubx(2u, 27u, 41, 133, 44u, true, &sv));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_GNSS_GALILEO, sv.sys);
	TEST_ASSERT_EQUAL_UINT8(27u, sv.svid);
	TEST_ASSERT_EQUAL_INT8(41, sv.elev_deg);
	TEST_ASSERT_EQUAL_INT16(133, sv.azim_deg);
	TEST_ASSERT_EQUAL_UINT8(44u, sv.cno_dbhz);
	TEST_ASSERT_TRUE(sv.used);
}

static void test_sv_rejects_the_almanac_phantom(void)
{
	ui_sv_t sv;

	/*
	 * The record that made the whole filter necessary: a satellite the
	 * receiver knows only from the almanac reports C/N0 0, azimuth 0 and
	 * elevation 0, and a fan of those lands on due north at the horizon —
	 * where the operator is looking for an obstruction.
	 */
	TEST_ASSERT_FALSE(sts_sky_sv_from_ubx(0u, 9u, 0, 0, 0u, false, &sv));

	/* Not just the all-zero case: any unmeasured, unused record. */
	TEST_ASSERT_FALSE(sts_sky_sv_from_ubx(0u, 9u, 30, 120, 0u, false, &sv));
}

static void test_sv_used_beats_a_zero_cno(void)
{
	ui_sv_t sv;

	/* Being in the timing solution is stronger evidence than a momentary
	 * zero C/N0, so the marker is drawn. */
	TEST_ASSERT_TRUE(sts_sky_sv_from_ubx(0u, 9u, 30, 120, 0u, true, &sv));
	TEST_ASSERT_TRUE(sv.used);
	TEST_ASSERT_EQUAL_UINT8(0u, sv.cno_dbhz);
}

static void test_sv_normalises_azimuth(void)
{
	ui_sv_t sv;

	/* 360 is out of sky_place()'s 0..359 range and would silently drop the
	 * marker; it folds to 0. */
	TEST_ASSERT_TRUE(sts_sky_sv_from_ubx(0u, 1u, 10, 360, 30u, false, &sv));
	TEST_ASSERT_EQUAL_INT16(0, sv.azim_deg);

	TEST_ASSERT_TRUE(sts_sky_sv_from_ubx(0u, 1u, 10, 361, 30u, false, &sv));
	TEST_ASSERT_EQUAL_INT16(1, sv.azim_deg);

	TEST_ASSERT_TRUE(sts_sky_sv_from_ubx(0u, 1u, 10, -1, 30u, false, &sv));
	TEST_ASSERT_EQUAL_INT16(359, sv.azim_deg);

	TEST_ASSERT_TRUE(sts_sky_sv_from_ubx(0u, 1u, 10, -360, 30u, false,
					     &sv));
	TEST_ASSERT_EQUAL_INT16(0, sv.azim_deg);
}

static void test_sv_admitted_azimuth_is_always_plottable(void)
{
	/*
	 * The contract that matters downstream: every azimuth this admits must
	 * be one sky_place() accepts, or the marker is filtered in and then
	 * dropped without trace.
	 */
	static uint8_t px[64 * 64];
	sky_canvas_t c;
	int32_t a;

	TEST_ASSERT_EQUAL_INT(0, sky_canvas_init(&c, px, 64u, 64u, sizeof(px)));

	for (a = -720; a <= 720; a++) {
		ui_sv_t sv;
		int32_t x;
		int32_t y;

		TEST_ASSERT_TRUE(sts_sky_sv_from_ubx(0u, 1u, 45, (int16_t)a,
						     35u, false, &sv));
		TEST_ASSERT_TRUE(sv.azim_deg >= 0);
		TEST_ASSERT_TRUE(sv.azim_deg <= 359);
		TEST_ASSERT_EQUAL_INT(0, sky_place(&c, sv.elev_deg,
						   sv.azim_deg, 0, &x, &y));
	}
}

static void test_sv_leaves_a_below_horizon_elevation_alone(void)
{
	ui_sv_t sv;

	/* sky_render() drops it; dropping it twice would hide a receiver
	 * reporting nonsense. */
	TEST_ASSERT_TRUE(sts_sky_sv_from_ubx(0u, 3u, -20, 90, 25u, false, &sv));
	TEST_ASSERT_EQUAL_INT8(-20, sv.elev_deg);
}

static void test_sv_rejects_a_null_output(void)
{
	TEST_ASSERT_FALSE(sts_sky_sv_from_ubx(0u, 1u, 45, 90, 40u, true, NULL));
}

/* ===================================================================== *
 *  Projection: policy output through the renderer
 * ===================================================================== */

static void test_projection_horizon_and_zenith(void)
{
	/*
	 * The two edges of the polar projection, driven through the policy's
	 * own output so the whole path is exercised: zenith collapses to the
	 * centre whatever the azimuth, and the horizon lands on the rim.
	 */
	static uint8_t px[65 * 65];
	sky_canvas_t c;
	ui_sv_t sv;
	int32_t x;
	int32_t y;
	int32_t az;

	TEST_ASSERT_EQUAL_INT(0, sky_canvas_init(&c, px, 65u, 65u, sizeof(px)));
	/* geometry(): cx = cy = 32, r = 31. */

	for (az = 0; az < 360; az += 30) {
		TEST_ASSERT_TRUE(sts_sky_sv_from_ubx(0u, 1u, 90, (int16_t)az,
						     40u, true, &sv));
		TEST_ASSERT_EQUAL_INT(0, sky_place(&c, sv.elev_deg,
						   sv.azim_deg, 0, &x, &y));
		TEST_ASSERT_EQUAL_INT32(32, x);
		TEST_ASSERT_EQUAL_INT32(32, y);
	}

	/* Horizon, due north: straight up the screen to the rim. */
	TEST_ASSERT_TRUE(sts_sky_sv_from_ubx(0u, 1u, 0, 0, 40u, true, &sv));
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, sv.elev_deg, sv.azim_deg, 0, &x,
					   &y));
	TEST_ASSERT_EQUAL_INT32(32, x);
	TEST_ASSERT_EQUAL_INT32(1, y);

	/* Horizon, due east: to the right. */
	TEST_ASSERT_TRUE(sts_sky_sv_from_ubx(0u, 1u, 0, 90, 40u, true, &sv));
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, sv.elev_deg, sv.azim_deg, 0, &x,
					   &y));
	TEST_ASSERT_EQUAL_INT32(63, x);
	TEST_ASSERT_EQUAL_INT32(32, y);

	/* Horizon, due south and west: the other two rim points. */
	TEST_ASSERT_TRUE(sts_sky_sv_from_ubx(0u, 1u, 0, 180, 40u, true, &sv));
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, sv.elev_deg, sv.azim_deg, 0, &x,
					   &y));
	TEST_ASSERT_EQUAL_INT32(32, x);
	TEST_ASSERT_EQUAL_INT32(63, y);

	TEST_ASSERT_TRUE(sts_sky_sv_from_ubx(0u, 1u, 0, 270, 40u, true, &sv));
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, sv.elev_deg, sv.azim_deg, 0, &x,
					   &y));
	TEST_ASSERT_EQUAL_INT32(1, x);
	TEST_ASSERT_EQUAL_INT32(32, y);
}

static void test_projection_radius_is_linear_in_elevation(void)
{
	static uint8_t px[65 * 65];
	sky_canvas_t c;
	ui_sv_t sv;
	int32_t x;
	int32_t y;
	int8_t el;
	int32_t prev = -1;

	TEST_ASSERT_EQUAL_INT(0, sky_canvas_init(&c, px, 65u, 65u, sizeof(px)));

	/* Due north: y falls monotonically from the centre to the rim as the
	 * elevation drops, and 45 degrees is halfway out. */
	for (el = 90; el >= 0; el--) {
		TEST_ASSERT_TRUE(sts_sky_sv_from_ubx(0u, 1u, el, 0, 40u, true,
						     &sv));
		TEST_ASSERT_EQUAL_INT(0, sky_place(&c, sv.elev_deg,
						   sv.azim_deg, 0, &x, &y));
		if (prev >= 0) {
			TEST_ASSERT_TRUE(y <= prev);
		}
		prev = y;
	}

	TEST_ASSERT_TRUE(sts_sky_sv_from_ubx(0u, 1u, 45, 0, 40u, true, &sv));
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, sv.elev_deg, sv.azim_deg, 0, &x,
					   &y));
	/* r = 31, rr = 31*45/90 = 15, so y = 32 - 15. */
	TEST_ASSERT_EQUAL_INT32(17, y);
}

static void test_projection_rejects_below_the_horizon(void)
{
	static uint8_t px[65 * 65];
	sky_canvas_t c;
	ui_sv_t sv;
	int32_t x;
	int32_t y;

	TEST_ASSERT_EQUAL_INT(0, sky_canvas_init(&c, px, 65u, 65u, sizeof(px)));

	TEST_ASSERT_TRUE(sts_sky_sv_from_ubx(0u, 1u, -1, 45, 40u, true, &sv));
	TEST_ASSERT_EQUAL_INT(-ERANGE, sky_place(&c, sv.elev_deg, sv.azim_deg,
						 0, &x, &y));
}

/* ===================================================================== *
 *  North: the verdict and its words
 * ===================================================================== */

static void test_north_unverified_per_reason(void)
{
	/* Rotated: the plot is honest, no badge. */
	TEST_ASSERT_FALSE(sts_sky_north_unverified(
		(uint8_t)STS_SKY_NORTH_TRUE));
	TEST_ASSERT_FALSE(sts_sky_north_unverified(
		(uint8_t)STS_SKY_NORTH_MODELLED));

	/* Not rotated: GNSS-north behind the badge. */
	TEST_ASSERT_TRUE(sts_sky_north_unverified(
		(uint8_t)STS_SKY_NORTH_NO_SAMPLE));
	TEST_ASSERT_TRUE(sts_sky_north_unverified(
		(uint8_t)STS_SKY_NORTH_UNCALIBRATED));
	TEST_ASSERT_TRUE(sts_sky_north_unverified(
		(uint8_t)STS_SKY_NORTH_DISTURBED));
	TEST_ASSERT_TRUE(sts_sky_north_unverified(
		(uint8_t)STS_SKY_NORTH_NO_DECLINATION));

	/* An out-of-range reason must fail safe to "unverified". */
	TEST_ASSERT_TRUE(sts_sky_north_unverified(
		(uint8_t)STS_SKY_NORTH_COUNT));
	TEST_ASSERT_TRUE(sts_sky_north_unverified(200u));
}

static void test_north_labels_are_usable(void)
{
	unsigned int r;
	unsigned int i;
	unsigned int j;
	const char *seen[STS_SKY_NORTH_COUNT];

	for (r = 0u; r < (unsigned int)STS_SKY_NORTH_COUNT; r++) {
		const char *s = sts_sky_north_label((uint8_t)r);

		/* Never NULL, never longer than the 24 the header promises —
		 * that bound is what the glue's "does the label fit the canvas"
		 * check is written against. */
		TEST_ASSERT_NOT_NULL(s);
		TEST_ASSERT_TRUE(strlen(s) > 0u);
		TEST_ASSERT_TRUE(strlen(s) <= 24u);
		seen[r] = s;
	}

	/* Every reason says something different: the whole point of returning a
	 * reason rather than a boolean is that "run the compass calibration" and
	 * "there is something ferrous next to the box" are different actions. */
	for (i = 0u; i < (unsigned int)STS_SKY_NORTH_COUNT; i++) {
		for (j = i + 1u; j < (unsigned int)STS_SKY_NORTH_COUNT; j++) {
			TEST_ASSERT_TRUE(strcmp(seen[i], seen[j]) != 0);
		}
	}

	/* Out of range still answers, and answers conservatively. */
	TEST_ASSERT_NOT_NULL(sts_sky_north_label(200u));
	TEST_ASSERT_TRUE(strlen(sts_sky_north_label(200u)) <= 24u);
}

/* ===================================================================== *
 *  Orientation
 * ===================================================================== */

/*
 * A believable e-compass sample. The board is mounted vertically (+z out of
 * the board face, horizontal in service), so gravity is along -y and the field
 * has a dip component along the board's z axis.
 */
static void sample_facing_north(sts_sky_orient_in_t *in)
{
	(void)memset(in, 0, sizeof(*in));
	in->sample_valid = true;
	/* Vertical board: gravity down the panel. */
	in->acc[0] = 0;
	in->acc[1] = -1000;
	in->acc[2] = 0;
	/* Field forward (+z, the direction the panel faces) with dip into -y. */
	in->mag[0] = 0;
	in->mag[1] = -400;
	in->mag[2] = 300;
	in->mag_scale[0] = 4096;
	in->mag_scale[1] = 4096;
	in->mag_scale[2] = 4096;
	in->cal_valid = true;
	in->field_ref = 500u;
}

static void test_orient_true_north_from_a_site_declination(void)
{
	sts_sky_orient_in_t in;
	sky_orient_t o;
	sky_heading_t hd;
	uint8_t r;

	sample_facing_north(&in);
	in.decl_site_valid = true;
	in.decl_site_ddeg = -105; /* -10.5 degrees, a real US east-coast value */

	(void)memset(&o, 0xAA, sizeof(o));
	r = sts_sky_orient(&in, &o, &hd);

	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_SKY_NORTH_TRUE, r);
	TEST_ASSERT_FALSE(sts_sky_north_unverified(r));
	TEST_ASSERT_TRUE(o.north_valid);
	TEST_ASSERT_FALSE(o.declination_modelled);
	TEST_ASSERT_EQUAL_INT16(-105, o.declination_ddeg);
	TEST_ASSERT_TRUE(o.tilt_valid);
	TEST_ASSERT_TRUE(hd.valid);
	TEST_ASSERT_EQUAL_INT16(hd.heading_ddeg, o.heading_ddeg);

	/* A verified orientation is what makes the renderer actually rotate. */
	TEST_ASSERT_EQUAL_INT32(sky_rotation_ddeg(&o),
				sky_rotation_ddeg(&o)); /* stable */
	TEST_ASSERT_TRUE(sky_rotation_ddeg(&o) >= 0);
	TEST_ASSERT_TRUE(sky_rotation_ddeg(&o) < 3600);
}

static void test_orient_falls_back_to_the_dipole_model(void)
{
	sts_sky_orient_in_t in;
	sky_orient_t o;
	int16_t expect = 0;
	uint8_t r;

	sample_facing_north(&in);
	in.decl_site_valid = false;
	in.pos_valid = true;
	in.lat_1e7 = 475000000;   /* 47.5 N */
	in.lon_1e7 = -1222000000; /* 122.2 W — Seattle */

	r = sts_sky_orient(&in, &o, NULL);

	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_SKY_NORTH_MODELLED, r);
	/* Rotated, but marked approximate: the model is good to 10-15 deg. */
	TEST_ASSERT_FALSE(sts_sky_north_unverified(r));
	TEST_ASSERT_TRUE(o.north_valid);
	TEST_ASSERT_TRUE(o.declination_modelled);

	TEST_ASSERT_EQUAL_INT(0, sky_declination_dipole_ddeg(in.lat_1e7,
							     in.lon_1e7,
							     &expect));
	TEST_ASSERT_EQUAL_INT16(expect, o.declination_ddeg);
}

static void test_orient_site_declination_beats_the_model(void)
{
	sts_sky_orient_in_t in;
	sky_orient_t o;

	sample_facing_north(&in);
	in.decl_site_valid = true;
	in.decl_site_ddeg = 155;
	in.pos_valid = true;
	in.lat_1e7 = 475000000;
	in.lon_1e7 = -1222000000;

	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_SKY_NORTH_TRUE,
				sts_sky_orient(&in, &o, NULL));
	TEST_ASSERT_EQUAL_INT16(155, o.declination_ddeg);
	TEST_ASSERT_FALSE(o.declination_modelled);
}

static void test_orient_no_sample(void)
{
	sts_sky_orient_in_t in;
	sky_orient_t o;
	sky_heading_t hd;

	sample_facing_north(&in);
	in.sample_valid = false;
	in.decl_site_valid = true;
	in.decl_site_ddeg = 0;

	(void)memset(&o, 0xAA, sizeof(o));
	(void)memset(&hd, 0xAA, sizeof(hd));

	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_SKY_NORTH_NO_SAMPLE,
				sts_sky_orient(&in, &o, &hd));
	TEST_ASSERT_TRUE(sts_sky_north_unverified(
		(uint8_t)STS_SKY_NORTH_NO_SAMPLE));
	/* out is ALWAYS written, so a caller cannot render from a stale one. */
	TEST_ASSERT_FALSE(o.north_valid);
	TEST_ASSERT_EQUAL_INT32(0, sky_rotation_ddeg(&o));
	TEST_ASSERT_FALSE(hd.valid);

	/* NULL input is the same state, not a crash. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_SKY_NORTH_NO_SAMPLE,
				sts_sky_orient(NULL, &o, NULL));
	TEST_ASSERT_FALSE(o.north_valid);

	/* NULL output has nowhere to write, so it reports the same and does
	 * nothing. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_SKY_NORTH_NO_SAMPLE,
				sts_sky_orient(&in, NULL, NULL));
}

static void test_orient_uncalibrated_and_disturbed_are_distinguished(void)
{
	sts_sky_orient_in_t in;
	sky_orient_t o;

	/*
	 * §10.4 fit never performed. The heading is unusable, and the operator
	 * action is "run the calibration".
	 */
	sample_facing_north(&in);
	in.cal_valid = false;
	in.field_ref = 0u;
	in.decl_site_valid = true;
	in.decl_site_ddeg = 0;
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_SKY_NORTH_UNCALIBRATED,
				sts_sky_orient(&in, &o, NULL));
	TEST_ASSERT_FALSE(o.north_valid);

	/*
	 * Calibrated, but the field is nowhere near the calibrated magnitude —
	 * something ferrous or energised is next to the box. Different words,
	 * different action.
	 */
	sample_facing_north(&in);
	in.decl_site_valid = true;
	in.decl_site_ddeg = 0;
	in.mag[0] = 40000;
	in.mag[1] = 0;
	in.mag[2] = 0;
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_SKY_NORTH_DISTURBED,
				sts_sky_orient(&in, &o, NULL));
	TEST_ASSERT_FALSE(o.north_valid);

	TEST_ASSERT_TRUE(strcmp(sts_sky_north_label(
					(uint8_t)STS_SKY_NORTH_UNCALIBRATED),
				sts_sky_north_label(
					(uint8_t)STS_SKY_NORTH_DISTURBED)) != 0);
}

static void test_orient_heading_without_a_declination_stays_unrotated(void)
{
	sts_sky_orient_in_t in;
	sky_orient_t o;

	/*
	 * A believed heading with no declination source is MAGNETIC north, up
	 * to ~20 degrees off at this product's latitudes. sky_orient_t has no
	 * way to say "rotated, but by an unknown amount", so the plot stays in
	 * GNSS-north and says so.
	 */
	sample_facing_north(&in);
	in.decl_site_valid = false;
	in.pos_valid = false;

	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_SKY_NORTH_NO_DECLINATION,
				sts_sky_orient(&in, &o, NULL));
	TEST_ASSERT_TRUE(sts_sky_north_unverified(
		(uint8_t)STS_SKY_NORTH_NO_DECLINATION));
	TEST_ASSERT_FALSE(o.north_valid);
	TEST_ASSERT_EQUAL_INT32(0, sky_rotation_ddeg(&o));
}

static void test_orient_tilt_is_reported_and_used(void)
{
	sts_sky_orient_in_t in;
	sky_orient_t flat;
	sky_orient_t tilted;
	sky_heading_t hd;

	/* Vertical mount, no roll: the reference heading. */
	sample_facing_north(&in);
	in.decl_site_valid = true;
	in.decl_site_ddeg = 0;
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_SKY_NORTH_TRUE,
				sts_sky_orient(&in, &flat, &hd));
	TEST_ASSERT_TRUE(flat.tilt_valid);
	TEST_ASSERT_TRUE(hd.tilt_valid);

	/*
	 * Same magnetic field, but the enclosure rolled about the panel normal.
	 * Tilt compensation is what keeps the heading the same; without it the
	 * plot would spin when the box was tipped.
	 */
	sample_facing_north(&in);
	in.decl_site_valid = true;
	in.decl_site_ddeg = 0;
	in.acc[0] = -707;
	in.acc[1] = -707;
	in.acc[2] = 0;
	in.mag[0] = -283; /* the same field vector, rotated with the board */
	in.mag[1] = -283;
	in.mag[2] = 300;
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_SKY_NORTH_TRUE,
				sts_sky_orient(&in, &tilted, NULL));
	TEST_ASSERT_TRUE(tilted.tilt_valid);
	{
		int32_t d = (int32_t)tilted.heading_ddeg -
			    (int32_t)flat.heading_ddeg;

		if (d < 0) {
			d = -d;
		}
		if (d > 1800) {
			d = 3600 - d;
		}
		/* Within 5 degrees of the untilted answer. */
		TEST_ASSERT_TRUE(d <= 50);
	}

	/*
	 * No usable gravity vector at all: the tilt term cannot be formed, so
	 * the heading is refused rather than computed from a bare magnetometer.
	 */
	sample_facing_north(&in);
	in.decl_site_valid = true;
	in.decl_site_ddeg = 0;
	in.acc[0] = 0;
	in.acc[1] = 0;
	in.acc[2] = 0;
	TEST_ASSERT_TRUE(sts_sky_north_unverified(
		sts_sky_orient(&in, &tilted, NULL)));
	TEST_ASSERT_FALSE(tilted.tilt_valid);
	TEST_ASSERT_FALSE(tilted.north_valid);
}

static void test_orient_uncalibrated_beats_a_stale_field_ref(void)
{
	sts_sky_orient_in_t in;
	sky_orient_t o;
	sky_heading_t hd;

	/*
	 * An uncommissioned unit carrying a stale `cal.mag.ref` in cfg must
	 * still report UNCALIBRATED — "run the compass calibration" — and not
	 * DISTURBED, which would send the operator hunting a phantom magnet.
	 *
	 * Note on where that guarantee actually comes from. sts_sky_orient()
	 * passes `cal_valid ? field_ref : 0`, which reads like the mechanism,
	 * but it is not: sky_heading_from_ecompass() refuses on !calibrated
	 * BEFORE it looks at field_ref at all (skyplot.c), so the ternary is
	 * belt-and-braces and cannot be observed from out here. Mutating it away
	 * leaves this suite green, and that is correct rather than a gap — the
	 * property below is real and is what the operator depends on; only the
	 * redundant guard is untestable. What DOES pin the two apart is
	 * test_orient_uncalibrated_and_disturbed_are_distinguished().
	 */
	sample_facing_north(&in);
	in.cal_valid = false;
	in.field_ref = 60000u; /* nothing could match this */
	in.decl_site_valid = true;
	in.decl_site_ddeg = 0;

	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_SKY_NORTH_UNCALIBRATED,
				sts_sky_orient(&in, &o, &hd));
	TEST_ASSERT_FALSE(hd.valid);
	TEST_ASSERT_FALSE(o.north_valid);
}

/* ===================================================================== *
 *  Palette
 * ===================================================================== */

static void test_palette_covers_every_index(void)
{
	unsigned int i;

	/* Background is black, and nothing else is. Two constellations sharing
	 * a colour, or a colour that is the background, is a plot the operator
	 * cannot read. */
	TEST_ASSERT_EQUAL_HEX16(0u,
				sts_sky_palette_rgb565((uint8_t)SKY_C_BG));

	for (i = 1u; i < (unsigned int)SKY_C__COUNT; i++) {
		TEST_ASSERT_NOT_EQUAL_UINT16(
			0u, sts_sky_palette_rgb565((uint8_t)i));
	}
}

static void test_palette_constellations_are_distinct(void)
{
	static const uint8_t sys[] = {
		(uint8_t)SKY_C_GPS,    (uint8_t)SKY_C_GALILEO,
		(uint8_t)SKY_C_GLONASS, (uint8_t)SKY_C_BEIDOU,
		(uint8_t)SKY_C_SBAS,   (uint8_t)SKY_C_QZSS,
		(uint8_t)SKY_C_OTHER,
	};
	unsigned int i;
	unsigned int j;

	for (i = 0u; i < sizeof(sys) / sizeof(sys[0]); i++) {
		for (j = i + 1u; j < sizeof(sys) / sizeof(sys[0]); j++) {
			TEST_ASSERT_NOT_EQUAL_UINT16(
				sts_sky_palette_rgb565(sys[i]),
				sts_sky_palette_rgb565(sys[j]));
		}
	}

	/* The grid and the badge must also be distinguishable from the markers
	 * and from each other. */
	TEST_ASSERT_NOT_EQUAL_UINT16(
		sts_sky_palette_rgb565((uint8_t)SKY_C_GRID),
		sts_sky_palette_rgb565((uint8_t)SKY_C_GRID_MAJOR));
	TEST_ASSERT_NOT_EQUAL_UINT16(
		sts_sky_palette_rgb565((uint8_t)SKY_C_BADGE),
		sts_sky_palette_rgb565((uint8_t)SKY_C_BEIDOU));
}

static void test_palette_out_of_range_is_background(void)
{
	unsigned int i;

	/*
	 * A palette that grows without this table growing shows as missing
	 * pixels, not as wrong ones.
	 */
	for (i = (unsigned int)SKY_C__COUNT; i <= 255u; i++) {
		TEST_ASSERT_EQUAL_HEX16(
			sts_sky_palette_rgb565((uint8_t)SKY_C_BG),
			sts_sky_palette_rgb565((uint8_t)i));
	}
}

static void test_palette_encodes_rgb565_correctly(void)
{
	/* SKY_C_GPS is 0x30/0xC0/0x40. RGB565: r>>3 = 6 (0b00110),
	 * g>>2 = 48 (0b110000), b>>3 = 8 (0b01000). */
	TEST_ASSERT_EQUAL_HEX16((uint16_t)((6u << 11) | (48u << 5) | 8u),
				sts_sky_palette_rgb565((uint8_t)SKY_C_GPS));

	/* Pure white and pure black through the macro itself. */
	TEST_ASSERT_EQUAL_HEX16(0xFFFFu, STS_SKY_RGB565(0xFF, 0xFF, 0xFF));
	TEST_ASSERT_EQUAL_HEX16(0x0000u, STS_SKY_RGB565(0x00, 0x00, 0x00));
}

/* ===================================================================== *
 *  Repaint gate
 * ===================================================================== */

static void test_band_mask_covers_the_claimed_rows(void)
{
	ui_hint_t h = as_built_hint();
	uint32_t m = sts_sky_band_mask(&h, (uint8_t)AB_ROWS);
	unsigned int r;

	/* Rows 5..18 inclusive, and nothing else. */
	for (r = 0u; r < 32u; r++) {
		bool in_band = (r >= 5u) && (r <= 18u);

		TEST_ASSERT_EQUAL_INT(in_band ? 1 : 0,
				      ((m >> r) & 1u) ? 1 : 0);
	}
}

static void test_band_mask_refuses_what_is_not_a_skyplot(void)
{
	ui_hint_t p = hint((uint8_t)UI_HINT_PROGRESS, 5u, 0u, 14u);

	TEST_ASSERT_EQUAL_UINT32(0u, sts_sky_band_mask(&p, (uint8_t)AB_ROWS));
	TEST_ASSERT_EQUAL_UINT32(0u, sts_sky_band_mask(NULL, (uint8_t)AB_ROWS));
}

static void test_band_mask_clamps_an_overhanging_hint(void)
{
	/* A hint claiming past the last row must not turn into a mask of
	 * everything, and must never shift by 32 (undefined). */
	ui_hint_t h = hint((uint8_t)UI_HINT_SKYPLOT, 30u, 0u, 40u);

	TEST_ASSERT_EQUAL_UINT32(((uint32_t)1u << 30) | ((uint32_t)1u << 31),
				 sts_sky_band_mask(&h, 32u));

	h = hint((uint8_t)UI_HINT_SKYPLOT, 18u, 0u, 10u);
	TEST_ASSERT_EQUAL_UINT32(((uint32_t)1u << 18) | ((uint32_t)1u << 19),
				 sts_sky_band_mask(&h, 20u));
}

static void sig_sv(ui_sv_t *sv, uint8_t sys, uint8_t id, int8_t el,
		   int16_t az, uint8_t cno, bool used)
{
	(void)memset(sv, 0, sizeof(*sv));
	sv->sys = sys;
	sv->svid = id;
	sv->elev_deg = el;
	sv->azim_deg = az;
	sv->cno_dbhz = cno;
	sv->used = used;
}

static void test_repaint_sig_is_stable_for_an_unchanged_picture(void)
{
	sts_sky_layout_t l;
	ui_sv_t sv[3];
	sky_orient_t o;
	uint32_t a;
	uint32_t b;

	TEST_ASSERT_EQUAL_INT(0, layout_as_built(&l));
	sig_sv(&sv[0], (uint8_t)UI_GNSS_GPS, 3u, 45, 90, 44u, true);
	sig_sv(&sv[1], (uint8_t)UI_GNSS_GALILEO, 12u, 20, 210, 33u, false);
	sig_sv(&sv[2], (uint8_t)UI_GNSS_GLONASS, 7u, 70, 5, 48u, true);
	(void)memset(&o, 0, sizeof(o));
	o.north_valid = true;
	o.heading_ddeg = 1234;
	o.declination_ddeg = -105;

	a = sts_sky_repaint_sig(&l, sv, 3u, &o,
				(uint8_t)STS_SKY_NORTH_TRUE);
	b = sts_sky_repaint_sig(&l, sv, 3u, &o,
				(uint8_t)STS_SKY_NORTH_TRUE);
	TEST_ASSERT_EQUAL_HEX32(a, b);

	/* Nothing about the picture depends on uninitialised padding: a copy
	 * built from differently-poisoned memory must hash the same. */
	{
		ui_sv_t sv2[3];
		sky_orient_t o2;

		(void)memset(sv2, 0xA5, sizeof(sv2));
		(void)memset(&o2, 0x5A, sizeof(o2));
		sig_sv(&sv2[0], (uint8_t)UI_GNSS_GPS, 3u, 45, 90, 44u, true);
		sig_sv(&sv2[1], (uint8_t)UI_GNSS_GALILEO, 12u, 20, 210, 33u,
		       false);
		sig_sv(&sv2[2], (uint8_t)UI_GNSS_GLONASS, 7u, 70, 5, 48u, true);
		(void)memset(&o2, 0, sizeof(o2));
		o2.north_valid = true;
		o2.heading_ddeg = 1234;
		o2.declination_ddeg = -105;

		TEST_ASSERT_EQUAL_HEX32(a, sts_sky_repaint_sig(
						  &l, sv2, 3u, &o2,
						  (uint8_t)
							  STS_SKY_NORTH_TRUE));
	}
}

static void test_repaint_sig_notices_every_input(void)
{
	sts_sky_layout_t l;
	sts_sky_layout_t l2;
	ui_sv_t sv[2];
	ui_sv_t mod[2];
	sky_orient_t o;
	sky_orient_t o2;
	uint32_t base;

	TEST_ASSERT_EQUAL_INT(0, layout_as_built(&l));
	sig_sv(&sv[0], (uint8_t)UI_GNSS_GPS, 3u, 45, 90, 44u, true);
	sig_sv(&sv[1], (uint8_t)UI_GNSS_GALILEO, 12u, 20, 210, 33u, false);
	(void)memset(&o, 0, sizeof(o));
	o.north_valid = true;
	o.heading_ddeg = 900;
	o.declination_ddeg = 120;

	base = sts_sky_repaint_sig(&l, sv, 2u, &o,
				   (uint8_t)STS_SKY_NORTH_TRUE);

	/* Every field that moves a pixel must move the signature. */
#define ASSERT_DIFFERENT(expr)                                                 \
	TEST_ASSERT_NOT_EQUAL_UINT32(base, (expr))

	/* Placement. */
	l2 = l;
	l2.x0 = (uint16_t)(l.x0 + 1u);
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l2, sv, 2u, &o,
					     (uint8_t)STS_SKY_NORTH_TRUE));
	l2 = l;
	l2.y0 = (uint16_t)(l.y0 + 1u);
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l2, sv, 2u, &o,
					     (uint8_t)STS_SKY_NORTH_TRUE));
	l2 = l;
	l2.side = (uint16_t)(l.side - 16u);
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l2, sv, 2u, &o,
					     (uint8_t)STS_SKY_NORTH_TRUE));

	/* The label is part of the picture. */
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l, sv, 2u, &o,
					     (uint8_t)STS_SKY_NORTH_MODELLED));

	/* Rotation. */
	o2 = o;
	o2.heading_ddeg = 901;
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l, sv, 2u, &o2,
					     (uint8_t)STS_SKY_NORTH_TRUE));
	o2 = o;
	o2.declination_ddeg = 121;
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l, sv, 2u, &o2,
					     (uint8_t)STS_SKY_NORTH_TRUE));
	o2 = o;
	o2.north_valid = false;
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l, sv, 2u, &o2,
					     (uint8_t)STS_SKY_NORTH_TRUE));
	o2 = o;
	o2.declination_modelled = true;
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l, sv, 2u, &o2,
					     (uint8_t)STS_SKY_NORTH_TRUE));
	o2 = o;
	o2.tilt_valid = true;
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l, sv, 2u, &o2,
					     (uint8_t)STS_SKY_NORTH_TRUE));
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l, sv, 2u, NULL,
					     (uint8_t)STS_SKY_NORTH_TRUE));

	/* Every per-satellite field: colour, identity, position, marker size
	 * and — the one a lazy hash would miss — filled versus hollow. */
	memcpy(mod, sv, sizeof(mod));
	mod[1].sys = (uint8_t)UI_GNSS_BEIDOU;
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l, mod, 2u, &o,
					     (uint8_t)STS_SKY_NORTH_TRUE));
	memcpy(mod, sv, sizeof(mod));
	mod[1].svid = 13u;
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l, mod, 2u, &o,
					     (uint8_t)STS_SKY_NORTH_TRUE));
	memcpy(mod, sv, sizeof(mod));
	mod[1].elev_deg = 21;
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l, mod, 2u, &o,
					     (uint8_t)STS_SKY_NORTH_TRUE));
	memcpy(mod, sv, sizeof(mod));
	mod[1].azim_deg = 211;
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l, mod, 2u, &o,
					     (uint8_t)STS_SKY_NORTH_TRUE));
	memcpy(mod, sv, sizeof(mod));
	mod[1].cno_dbhz = 34u;
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l, mod, 2u, &o,
					     (uint8_t)STS_SKY_NORTH_TRUE));
	memcpy(mod, sv, sizeof(mod));
	mod[1].used = true;
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l, mod, 2u, &o,
					     (uint8_t)STS_SKY_NORTH_TRUE));

	/* A satellite appearing or disappearing. */
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l, sv, 1u, &o,
					     (uint8_t)STS_SKY_NORTH_TRUE));
	ASSERT_DIFFERENT(sts_sky_repaint_sig(&l, sv, 0u, &o,
					     (uint8_t)STS_SKY_NORTH_TRUE));

	/* Reordering the same two satellites IS a different picture only in the
	 * order they are drawn, but the gate must still be well-defined; assert
	 * it terminates and produces something rather than asserting a value. */
	memcpy(mod, sv, sizeof(mod));
	mod[0] = sv[1];
	mod[1] = sv[0];
	(void)sts_sky_repaint_sig(&l, mod, 2u, &o,
				  (uint8_t)STS_SKY_NORTH_TRUE);

#undef ASSERT_DIFFERENT
}

static void test_repaint_sig_tolerates_degenerate_input(void)
{
	sts_sky_layout_t l;
	uint32_t a;
	uint32_t b;

	TEST_ASSERT_EQUAL_INT(0, layout_as_built(&l));

	/* No layout, no satellites, no orientation: still a defined value, and
	 * still stable. */
	a = sts_sky_repaint_sig(NULL, NULL, 0u, NULL,
				(uint8_t)STS_SKY_NORTH_NO_SAMPLE);
	b = sts_sky_repaint_sig(NULL, NULL, 0u, NULL,
				(uint8_t)STS_SKY_NORTH_NO_SAMPLE);
	TEST_ASSERT_EQUAL_HEX32(a, b);

	/* A NULL array with a non-zero count must not be dereferenced. */
	TEST_ASSERT_EQUAL_HEX32(sts_sky_repaint_sig(&l, NULL, 8u, NULL,
						    (uint8_t)
							    STS_SKY_NORTH_TRUE),
				sts_sky_repaint_sig(&l, NULL, 8u, NULL,
						    (uint8_t)
							    STS_SKY_NORTH_TRUE));

	/* A count past UI_MAX_SV is clamped, not read past. */
	{
		ui_sv_t sv[UI_MAX_SV];
		unsigned int i;

		for (i = 0u; i < (unsigned int)UI_MAX_SV; i++) {
			sig_sv(&sv[i], (uint8_t)UI_GNSS_GPS, (uint8_t)i, 30,
			       (int16_t)(i * 11u), 40u, false);
		}
		TEST_ASSERT_EQUAL_HEX32(
			sts_sky_repaint_sig(&l, sv, (uint8_t)UI_MAX_SV, NULL,
					    (uint8_t)STS_SKY_NORTH_TRUE),
			sts_sky_repaint_sig(&l, sv, 255u, NULL,
					    (uint8_t)STS_SKY_NORTH_TRUE));
	}
}

/* ===================================================================== *
 *  End to end: a unit with no compass still draws its sky
 * ===================================================================== */

static void test_no_compass_still_draws_the_plot(void)
{
	/*
	 * §6.3's headline requirement, exercised through the whole glue: a unit
	 * whose e-compass is absent must draw the plot in GNSS-north behind the
	 * badge, NOT refuse to draw.
	 */
	static uint8_t px[AB_MAX_SIDE * AB_MAX_SIDE];
	sts_sky_layout_t l;
	sts_sky_orient_in_t in;
	sky_orient_t o;
	sky_canvas_t c;
	ui_sv_t sv[4];
	uint8_t n = 0u;
	uint8_t reason;
	uint32_t markers = 0u;
	uint32_t badge = 0u;
	int32_t x;
	int32_t y;

	/* No sample at all — the parts never answered their WHO_AM_I. */
	(void)memset(&in, 0, sizeof(in));
	in.sample_valid = false;
	reason = sts_sky_orient(&in, &o, NULL);

	TEST_ASSERT_TRUE(sts_sky_north_unverified(reason));
	TEST_ASSERT_FALSE(o.north_valid);
	TEST_ASSERT_EQUAL_INT32(0, sky_rotation_ddeg(&o));

	if (sts_sky_sv_from_ubx(0u, 3u, 45, 90, 44u, true, &sv[n])) {
		n++;
	}
	if (sts_sky_sv_from_ubx(2u, 12u, 20, 210, 33u, false, &sv[n])) {
		n++;
	}
	if (sts_sky_sv_from_ubx(6u, 7u, 70, 5, 48u, true, &sv[n])) {
		n++;
	}
	/* An almanac-only record, which must not become a phantom at due
	 * north on the horizon. */
	if (sts_sky_sv_from_ubx(3u, 21u, 0, 0, 0u, false, &sv[n])) {
		n++;
	}
	TEST_ASSERT_EQUAL_UINT8(3u, n);

	TEST_ASSERT_EQUAL_INT(0, layout_as_built(&l));
	TEST_ASSERT_EQUAL_INT(0, sky_canvas_init(&c, px, l.side, l.side,
						 sizeof(px)));
	TEST_ASSERT_EQUAL_INT(0, sky_render(&c, sv, n,
					    o.north_valid ? &o : NULL));

	/* The plot is there: the three real satellites are on the canvas. */
	{
		uint16_t iy;
		uint16_t ix;

		for (iy = 0u; iy < c.h; iy++) {
			for (ix = 0u; ix < c.w; ix++) {
				uint8_t v = sky_canvas_get(&c, (int32_t)ix,
							   (int32_t)iy);

				if (v == (uint8_t)SKY_C_BADGE) {
					badge++;
				} else if (v == (uint8_t)SKY_C_GPS ||
					   v == (uint8_t)SKY_C_GALILEO ||
					   v == (uint8_t)SKY_C_GLONASS) {
					markers++;
				}
			}
		}
	}
	TEST_ASSERT_TRUE(markers > 0u);

	/* And the badge is up. */
	TEST_ASSERT_EQUAL_UINT32(2u * (uint32_t)c.w, badge);

	/* Un-rotated: azimuth 90 at elevation 45 is due east of centre, exactly
	 * where the receiver reported it. */
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, 45, 90, sky_rotation_ddeg(&o),
					   &x, &y));
	TEST_ASSERT_TRUE(x > (int32_t)(c.w / 2u));
	TEST_ASSERT_EQUAL_INT32((int32_t)(c.h / 2u), y);

	/* Nothing was drawn at the phantom's would-be position (due north on
	 * the horizon), because the record never became a marker. */
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, 0, 0, 0, &x, &y));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_GRID_MAJOR,
				sky_canvas_get(&c, x, y));
}

/* ===================================================================== *
 *  Runner
 * ===================================================================== */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_layout_as_built_geometry);
	RUN_TEST(test_layout_is_centred_in_a_wider_band);
	RUN_TEST(test_layout_is_width_limited_in_a_narrow_band);
	RUN_TEST(test_layout_honours_the_canvas_ceiling);
	RUN_TEST(test_layout_never_exceeds_the_renderers_maximum);
	RUN_TEST(test_layout_rejects_bad_arguments);
	RUN_TEST(test_layout_refuses_a_band_it_cannot_fill);
	RUN_TEST(test_layout_accepts_exactly_the_minimum);
	RUN_TEST(test_layout_output_is_a_valid_canvas);

	RUN_TEST(test_gnss_id_mapping);
	RUN_TEST(test_gnss_id_unknown_ids_are_never_a_guess);

	RUN_TEST(test_sv_admits_a_measured_satellite);
	RUN_TEST(test_sv_rejects_the_almanac_phantom);
	RUN_TEST(test_sv_used_beats_a_zero_cno);
	RUN_TEST(test_sv_normalises_azimuth);
	RUN_TEST(test_sv_admitted_azimuth_is_always_plottable);
	RUN_TEST(test_sv_leaves_a_below_horizon_elevation_alone);
	RUN_TEST(test_sv_rejects_a_null_output);

	RUN_TEST(test_projection_horizon_and_zenith);
	RUN_TEST(test_projection_radius_is_linear_in_elevation);
	RUN_TEST(test_projection_rejects_below_the_horizon);

	RUN_TEST(test_north_unverified_per_reason);
	RUN_TEST(test_north_labels_are_usable);

	RUN_TEST(test_orient_true_north_from_a_site_declination);
	RUN_TEST(test_orient_falls_back_to_the_dipole_model);
	RUN_TEST(test_orient_site_declination_beats_the_model);
	RUN_TEST(test_orient_no_sample);
	RUN_TEST(test_orient_uncalibrated_and_disturbed_are_distinguished);
	RUN_TEST(test_orient_heading_without_a_declination_stays_unrotated);
	RUN_TEST(test_orient_tilt_is_reported_and_used);
	RUN_TEST(test_orient_uncalibrated_beats_a_stale_field_ref);

	RUN_TEST(test_palette_covers_every_index);
	RUN_TEST(test_palette_constellations_are_distinct);
	RUN_TEST(test_palette_out_of_range_is_background);
	RUN_TEST(test_palette_encodes_rgb565_correctly);

	RUN_TEST(test_band_mask_covers_the_claimed_rows);
	RUN_TEST(test_band_mask_refuses_what_is_not_a_skyplot);
	RUN_TEST(test_band_mask_clamps_an_overhanging_hint);
	RUN_TEST(test_repaint_sig_is_stable_for_an_unchanged_picture);
	RUN_TEST(test_repaint_sig_notices_every_input);
	RUN_TEST(test_repaint_sig_tolerates_degenerate_input);

	RUN_TEST(test_no_compass_still_draws_the_plot);

	return UNITY_END();
}
