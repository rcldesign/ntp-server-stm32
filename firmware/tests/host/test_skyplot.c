/*
 * STS1000 "Meridian" — host unit tests for core/ui/skyplot.c.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The renderer is pure — canvas plus satellite array in, palette indices out —
 * so it can be pinned with golden images. The canvas is rendered to ASCII (one
 * character per palette index) and compared against a literal picture, which
 * means a regression prints a readable before/after rather than a byte offset.
 *
 * Golden images are kept at 31x31 so they stay legible in the source and so a
 * one-pixel geometry change is visible. Tests on the as-built 240x240 plot assert
 * structural properties instead — marker centres, colour coverage, the badge
 * band, determinism — because a 240x240 literal would be unreviewable.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "ui/skyplot.h"
#include "unity.h"

/* ------------------------------------------------------------------ helpers */

#define W 31U
#define H 31U

static uint8_t px[SKY_MAX_DIM * SKY_MAX_DIM];
static char ascii[(SKY_MAX_DIM + 1U) * SKY_MAX_DIM + 1U];

static sky_canvas_t canvas(uint16_t w, uint16_t h)
{
	sky_canvas_t c;

	TEST_ASSERT_EQUAL_INT(0, sky_canvas_init(&c, px, w, h, sizeof(px)));
	return c;
}

/* Compare against a golden picture given as an array of row strings. */
static void assert_golden(sky_canvas_t *c, const char *const *rows, uint16_t n)
{
	uint16_t y;

	for (y = 0U; y < n; y++) {
		char line[SKY_MAX_DIM + 1U];
		uint16_t x;

		for (x = 0U; x < c->w; x++) {
			line[x] = sky_ascii_char(sky_canvas_get(c, (int32_t)x,
								(int32_t)y));
		}
		line[c->w] = '\0';
		if (strcmp(line, rows[y]) != 0) {
			char msg[128];

			(void)snprintf(msg, sizeof(msg), "row %u", (unsigned)y);
			UNITY_TEST_ASSERT_EQUAL_STRING(rows[y], line, __LINE__, msg);
		}
	}
}

static ui_sv_t sv_make(uint8_t sys, uint8_t svid, int8_t el, int16_t az,
		       uint8_t cno, bool used)
{
	ui_sv_t s;

	(void)memset(&s, 0, sizeof(s));
	s.sys = sys;
	s.svid = svid;
	s.elev_deg = el;
	s.azim_deg = az;
	s.cno_dbhz = cno;
	s.used = used;
	return s;
}

/* Count pixels of a given palette index. */
static unsigned int count_colour(const sky_canvas_t *c, uint8_t colour)
{
	unsigned int n = 0U;
	uint16_t y;
	uint16_t x;

	for (y = 0U; y < c->h; y++) {
		for (x = 0U; x < c->w; x++) {
			if (sky_canvas_get(c, (int32_t)x, (int32_t)y) == colour) {
				n++;
			}
		}
	}
	return n;
}

/* ===================================================================== *
 *  Canvas
 * ===================================================================== */

static void test_canvas_init(void)
{
	sky_canvas_t c;

	TEST_ASSERT_EQUAL_INT(-EINVAL, sky_canvas_init(NULL, px, 8U, 8U,
						       sizeof(px)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sky_canvas_init(&c, NULL, 8U, 8U,
						       sizeof(px)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sky_canvas_init(&c, px, 0U, 8U,
						       sizeof(px)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sky_canvas_init(&c, px, 8U, 0U,
						       sizeof(px)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sky_canvas_init(&c, px,
			(uint16_t)SKY_MAX_DIM + 1U, 8U, sizeof(px)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sky_canvas_init(&c, px, 8U,
			(uint16_t)SKY_MAX_DIM + 1U, sizeof(px)));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, sky_canvas_init(&c, px, 8U, 8U, 63U));
	TEST_ASSERT_EQUAL_INT(0, sky_canvas_init(&c, px, 8U, 8U, 64U));

	/* Initialised canvases are blank. */
	TEST_ASSERT_EQUAL_UINT(64U, count_colour(&c, (uint8_t)SKY_C_BG));

	/* Out-of-bounds reads are background, not a crash. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_BG, sky_canvas_get(&c, -1, 0));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_BG, sky_canvas_get(&c, 0, -1));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_BG, sky_canvas_get(&c, 8, 0));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_BG, sky_canvas_get(&c, 0, 8));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_BG, sky_canvas_get(NULL, 0, 0));

	sky_canvas_clear(NULL);
	TEST_ASSERT_EQUAL_UINT(0U, sky_canvas_to_ascii(NULL, ascii,
						       sizeof(ascii)));
	TEST_ASSERT_EQUAL_UINT(0U, sky_canvas_to_ascii(&c, NULL, 8U));
	TEST_ASSERT_EQUAL_UINT(0U, sky_canvas_to_ascii(&c, ascii, 0U));

	/* A short ASCII buffer truncates and still terminates. */
	{
		char small[10];

		TEST_ASSERT_TRUE(sky_canvas_to_ascii(&c, small, sizeof(small)) <
				 sizeof(small));
		TEST_ASSERT_TRUE(strlen(small) < sizeof(small));
	}
}

static void test_palette_and_ascii_map(void)
{
	unsigned int i;

	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_GPS,
				sky_colour_for_sys((uint8_t)UI_GNSS_GPS));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_GALILEO,
				sky_colour_for_sys((uint8_t)UI_GNSS_GALILEO));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_GLONASS,
				sky_colour_for_sys((uint8_t)UI_GNSS_GLONASS));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_BEIDOU,
				sky_colour_for_sys((uint8_t)UI_GNSS_BEIDOU));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_SBAS,
				sky_colour_for_sys((uint8_t)UI_GNSS_SBAS));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_QZSS,
				sky_colour_for_sys((uint8_t)UI_GNSS_QZSS));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_OTHER,
				sky_colour_for_sys((uint8_t)UI_GNSS_OTHER));
	/* Anything unexpected lands on OTHER rather than off the end. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_OTHER, sky_colour_for_sys(200U));

	/* Every palette index has a distinct ASCII character. */
	for (i = 0U; i < (unsigned int)SKY_C__COUNT; i++) {
		unsigned int j;

		for (j = i + 1U; j < (unsigned int)SKY_C__COUNT; j++) {
			TEST_ASSERT_TRUE(sky_ascii_char((uint8_t)i) !=
					 sky_ascii_char((uint8_t)j));
		}
	}
	TEST_ASSERT_EQUAL_INT('?', sky_ascii_char(0xFFU));
}

/* ===================================================================== *
 *  Trigonometry
 * ===================================================================== */

static void test_trig(void)
{
	/* Cardinal values. */
	TEST_ASSERT_EQUAL_INT32(0, sky_sin_ddeg(0));
	TEST_ASSERT_EQUAL_INT32(SKY_TRIG_ONE, sky_sin_ddeg(900));
	TEST_ASSERT_EQUAL_INT32(0, sky_sin_ddeg(1800));
	TEST_ASSERT_EQUAL_INT32(-SKY_TRIG_ONE, sky_sin_ddeg(2700));
	TEST_ASSERT_EQUAL_INT32(SKY_TRIG_ONE, sky_cos_ddeg(0));
	TEST_ASSERT_EQUAL_INT32(0, sky_cos_ddeg(900));
	TEST_ASSERT_EQUAL_INT32(-SKY_TRIG_ONE, sky_cos_ddeg(1800));

	/* Wrapping, including negatives and multiple turns. */
	TEST_ASSERT_EQUAL_INT32(sky_sin_ddeg(450), sky_sin_ddeg(450 + 3600));
	TEST_ASSERT_EQUAL_INT32(sky_sin_ddeg(450), sky_sin_ddeg(450 - 3600));
	TEST_ASSERT_EQUAL_INT32(sky_sin_ddeg(2700), sky_sin_ddeg(-900));

	/* sin^2 + cos^2 == 1 to within the table's interpolation error. */
	{
		int32_t d;

		for (d = 0; d < 3600; d += 7) {
			int32_t s = sky_sin_ddeg(d);
			int32_t k = sky_cos_ddeg(d);
			int64_t sum = ((int64_t)s * s) + ((int64_t)k * k);
			int64_t one = (int64_t)SKY_TRIG_ONE * SKY_TRIG_ONE;
			int64_t err = sum - one;

			if (err < 0) {
				err = -err;
			}
			/* 1 % of unity is far tighter than one pixel in 320. */
			TEST_ASSERT_TRUE(err < (one / 100));
		}
	}

	/* 45 degrees: sin == cos, and both near 0.7071. */
	TEST_ASSERT_INT_WITHIN(40, 2896, sky_sin_ddeg(450));
	TEST_ASSERT_INT_WITHIN(40, 2896, sky_cos_ddeg(450));
}

static void test_atan2(void)
{
	/* Compass convention: 0 at +y, 90 at +x. */
	TEST_ASSERT_EQUAL_INT32(0, sky_atan2_ddeg(1000, 0));
	TEST_ASSERT_INT_WITHIN(5, 900, sky_atan2_ddeg(0, 1000));
	TEST_ASSERT_INT_WITHIN(5, 1800, sky_atan2_ddeg(-1000, 0));
	TEST_ASSERT_INT_WITHIN(5, 2700, sky_atan2_ddeg(0, -1000));

	/* The four diagonals. */
	TEST_ASSERT_INT_WITHIN(10, 450, sky_atan2_ddeg(1000, 1000));
	TEST_ASSERT_INT_WITHIN(10, 1350, sky_atan2_ddeg(-1000, 1000));
	TEST_ASSERT_INT_WITHIN(10, 2250, sky_atan2_ddeg(-1000, -1000));
	TEST_ASSERT_INT_WITHIN(10, 3150, sky_atan2_ddeg(1000, -1000));

	/* Degenerate. */
	TEST_ASSERT_EQUAL_INT32(0, sky_atan2_ddeg(0, 0));

	/*
	 * Round-trip against the sine table over the whole circle: build a vector
	 * at a known bearing and recover it. This is the check that would catch an
	 * octant or quadrant mix-up, which is the classic atan2 bug.
	 */
	{
		int32_t d;

		for (d = 0; d < 3600; d += 13) {
			int32_t x = sky_sin_ddeg(d);
			int32_t y = sky_cos_ddeg(d);
			int32_t got = sky_atan2_ddeg(y, x);
			int32_t err = got - d;

			if (err > 1800) {
				err -= 3600;
			}
			if (err < -1800) {
				err += 3600;
			}
			if (err < 0) {
				err = -err;
			}
			/* The approximation is specified to about 0.2 degrees. */
			TEST_ASSERT_TRUE(err <= 5);
		}
	}
}

static void test_isqrt(void)
{
	TEST_ASSERT_EQUAL_UINT32(0U, sky_isqrt(0U));
	TEST_ASSERT_EQUAL_UINT32(1U, sky_isqrt(1U));
	TEST_ASSERT_EQUAL_UINT32(1U, sky_isqrt(3U));
	TEST_ASSERT_EQUAL_UINT32(2U, sky_isqrt(4U));
	TEST_ASSERT_EQUAL_UINT32(3U, sky_isqrt(15U));
	TEST_ASSERT_EQUAL_UINT32(4U, sky_isqrt(16U));
	TEST_ASSERT_EQUAL_UINT32(1000U, sky_isqrt(1000000U));
	TEST_ASSERT_EQUAL_UINT32(65535U, sky_isqrt(65535ULL * 65535ULL));
	TEST_ASSERT_EQUAL_UINT32(4294967295U,
		sky_isqrt(4294967295ULL * 4294967295ULL));
}

/* ===================================================================== *
 *  Placement
 * ===================================================================== */

static void test_place(void)
{
	sky_canvas_t c = canvas(W, H);
	int32_t x = 0;
	int32_t y = 0;
	int32_t cx = (int32_t)W / 2;
	int32_t cy = (int32_t)H / 2;
	int32_t r = ((int32_t)W / 2) - 1;

	/* Zenith lands exactly at the centre, whatever the azimuth. */
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, 90, 0, 0, &x, &y));
	TEST_ASSERT_EQUAL_INT32(cx, x);
	TEST_ASSERT_EQUAL_INT32(cy, y);
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, 90, 217, 0, &x, &y));
	TEST_ASSERT_EQUAL_INT32(cx, x);
	TEST_ASSERT_EQUAL_INT32(cy, y);

	/* Horizon, north: straight up (screen y decreases). */
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, 0, 0, 0, &x, &y));
	TEST_ASSERT_EQUAL_INT32(cx, x);
	TEST_ASSERT_EQUAL_INT32(cy - r, y);

	/* Horizon, east / south / west. */
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, 0, 90, 0, &x, &y));
	TEST_ASSERT_EQUAL_INT32(cx + r, x);
	TEST_ASSERT_EQUAL_INT32(cy, y);
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, 0, 180, 0, &x, &y));
	TEST_ASSERT_EQUAL_INT32(cx, x);
	TEST_ASSERT_EQUAL_INT32(cy + r, y);
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, 0, 270, 0, &x, &y));
	TEST_ASSERT_EQUAL_INT32(cx - r, x);
	TEST_ASSERT_EQUAL_INT32(cy, y);

	/* Radius is linear in (90 - elevation): 45 degrees is half way out. */
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, 45, 0, 0, &x, &y));
	TEST_ASSERT_EQUAL_INT32(cy - (r / 2), y);

	/* A 90-degree rotation moves north to the east position. */
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, 0, 0, 900, &x, &y));
	TEST_ASSERT_EQUAL_INT32(cx + r, x);
	TEST_ASSERT_EQUAL_INT32(cy, y);

	/* Below the horizon and bad azimuths are refused, not clamped. */
	TEST_ASSERT_EQUAL_INT(-ERANGE, sky_place(&c, -1, 0, 0, &x, &y));
	TEST_ASSERT_EQUAL_INT(-ERANGE, sky_place(&c, 91, 0, 0, &x, &y));
	TEST_ASSERT_EQUAL_INT(-ERANGE, sky_place(&c, 10, -1, 0, &x, &y));
	TEST_ASSERT_EQUAL_INT(-ERANGE, sky_place(&c, 10, 360, 0, &x, &y));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sky_place(NULL, 10, 0, 0, &x, &y));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sky_place(&c, 10, 0, 0, NULL, &y));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sky_place(&c, 10, 0, 0, &x, NULL));
}

static void test_marker_radius(void)
{
	/* Clamped at both ends, monotonic in between. */
	TEST_ASSERT_EQUAL_INT32(SKY_MARKER_MIN_R, sky_marker_radius(0U));
	TEST_ASSERT_EQUAL_INT32(SKY_MARKER_MIN_R,
				sky_marker_radius((uint8_t)SKY_CNO_MIN));
	TEST_ASSERT_EQUAL_INT32(SKY_MARKER_MAX_R,
				sky_marker_radius((uint8_t)SKY_CNO_MAX));
	TEST_ASSERT_EQUAL_INT32(SKY_MARKER_MAX_R, sky_marker_radius(255U));
	{
		unsigned int cno;
		int32_t prev = SKY_MARKER_MIN_R;

		for (cno = 0U; cno <= 255U; cno++) {
			int32_t r = sky_marker_radius((uint8_t)cno);

			TEST_ASSERT_TRUE(r >= prev);
			TEST_ASSERT_TRUE(r >= SKY_MARKER_MIN_R);
			TEST_ASSERT_TRUE(r <= SKY_MARKER_MAX_R);
			prev = r;
		}
	}
	/* A stronger signal really is bigger. */
	TEST_ASSERT_TRUE(sky_marker_radius(45U) > sky_marker_radius(25U));
}

/* ===================================================================== *
 *  Golden images
 * ===================================================================== */

/*
 * The empty plot: horizon ring (=), two elevation rings (-), the north tick (=)
 * and the three minor cardinal ticks (-), and the two-row badge (#) because no
 * orientation was supplied.
 */
static void test_golden_empty_gnss_north(void)
{
	/*
	 * The empty plot in GNSS-north: horizon ring (=), the 30- and 60-degree
	 * elevation rings (-), a long north tick (=) with three shorter cardinal
	 * ticks (-), and the two-row "north unverified" badge (#) because no
	 * orientation was supplied.
	 *
	 * The horizon ring is continuous all the way round — the cardinal ticks are
	 * drawn before it precisely so they cannot punch holes in the one line that
	 * tells an operator where the sky ends.
	 */
	static const char *const golden[H] = {
		"...............................",
		"............=======............",
		".........===...=...===.........",
		"........=.............=........",
		"......==...............==......",
		".....=...................=.....",
		"....=........-----........=....",
		"....=......--.....--......=....",
		"...=.....--.........--.....=...",
		"..=.....-.............-.....=..",
		"..=.....-.............-.....=..",
		"..=....-......---......-....=..",
		".=.....-.....-...-.....-.....=.",
		".=....-.....-.....-.....-....=.",
		".=....-....-.......-....-....=.",
		".=-...-....-.......-....-...-=.",
		".=....-....-.......-....-....=.",
		".=....-.....-.....-.....-....=.",
		".=.....-.....-...-.....-.....=.",
		"..=....-......---......-....=..",
		"..=.....-.............-.....=..",
		"..=.....-.............-.....=..",
		"...=.....--.........--.....=...",
		"....=......--.....--......=....",
		"....=........-----........=....",
		".....=...................=.....",
		"......==...............==......",
		"........=.............=........",
		".........===...-...===.........",
		"###############################",
		"###############################",
	};

	sky_canvas_t c = canvas(W, H);

	TEST_ASSERT_EQUAL_INT(0, sky_render(&c, NULL, 0U, NULL));
	assert_golden(&c, golden, H);

	/* The badge is exactly the bottom two rows and nothing else. */
	TEST_ASSERT_EQUAL_UINT(2U * W, count_colour(&c, (uint8_t)SKY_C_BADGE));

	/* Nothing is drawn at the centre with no satellites. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_BG,
		sky_canvas_get(&c, (int32_t)W / 2, (int32_t)H / 2));

	TEST_ASSERT_EQUAL_INT(-EINVAL, sky_render(NULL, NULL, 0U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sky_render(&c, NULL, 3U, NULL));
}

/*
 * A small canvas with one satellite of each kind, asserted verbatim. 21x21 with
 * markers at the zenith keeps the picture legible.
 */
static void test_golden_used_vs_visible(void)
{
	/*
	 * Two satellites, identical in every respect except @ref ui_sv_t::used:
	 * GPS to the east (filled disc, '0') and Galileo to the west (hollow ring,
	 * '1'), both at 45 degrees elevation and 50 dB-Hz. The only difference in
	 * the picture is therefore exactly the distinction §6.3 asks for, which is
	 * what makes this golden worth asserting: if filled and hollow were ever
	 * swapped, or if `used` stopped being read, the image changes.
	 */
	static const char *const golden[H] = {
		"...............................",
		"............=======............",
		".........===...=...===.........",
		"........=.............=........",
		"......==...............==......",
		".....=...................=.....",
		"....=........-----........=....",
		"....=......--.....--......=....",
		"...=.....--.........--.....=...",
		"..=.....-.............-.....=..",
		"..=...11111...........0.....=..",
		"..=..1.-...1..---..0000000..=..",
		".=..1..-....1-...-000000000..=.",
		".=.1..-.....-1....000000000..=.",
		".=.1..-....-.1....000000000..=.",
		".=-1..-....-.1...00000000000-=.",
		".=.1..-....-.1....000000000..=.",
		".=.1..-.....-1....000000000..=.",
		".=..1..-....1-...-000000000..=.",
		"..=..1.-...1..---..0000000..=..",
		"..=...11111...........0.....=..",
		"..=.....-.............-.....=..",
		"...=.....--.........--.....=...",
		"....=......--.....--......=....",
		"....=........-----........=....",
		".....=...................=.....",
		"......==...............==......",
		"........=.............=........",
		".........===...-...===.........",
		"###############################",
		"###############################",
	};

	sky_canvas_t c = canvas(W, H);
	ui_sv_t sv[2];
	int32_t x = 0;
	int32_t y = 0;
	int32_t r = sky_marker_radius(50U);

	sv[0] = sv_make((uint8_t)UI_GNSS_GPS, 1U, 45, 90, 50U, true);
	sv[1] = sv_make((uint8_t)UI_GNSS_GALILEO, 2U, 45, 270, 50U, false);

	TEST_ASSERT_EQUAL_INT(0, sky_render(&c, sv, 2U, NULL));
	assert_golden(&c, golden, H);

	/* Filled: the marker centre carries the constellation colour. */
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, 45, 90, 0, &x, &y));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_GPS, sky_canvas_get(&c, x, y));

	/* Hollow: the centre is background, the rim is the constellation colour. */
	TEST_ASSERT_EQUAL_INT(0, sky_place(&c, 45, 270, 0, &x, &y));
	TEST_ASSERT_TRUE(sky_canvas_get(&c, x, y) != (uint8_t)SKY_C_GALILEO);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_GALILEO,
				sky_canvas_get(&c, x + r, y));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_GALILEO,
				sky_canvas_get(&c, x - r, y));

	/*
	 * The used marker paints strictly more of its own colour than the unused
	 * one at the same C/N0 — a disc versus a ring of the same radius.
	 */
	{
		unsigned int filled = count_colour(&c, (uint8_t)SKY_C_GPS);
		unsigned int hollow = count_colour(&c, (uint8_t)SKY_C_GALILEO);

		TEST_ASSERT_TRUE(filled > hollow);
		TEST_ASSERT_TRUE(hollow > 0U);
	}
}

/* Every constellation gets its own colour on the canvas. */
static void test_all_constellations_are_distinguishable(void)
{
	sky_canvas_t c = canvas(120U, 120U);
	ui_sv_t sv[UI_GNSS__COUNT];
	unsigned int i;

	for (i = 0U; i < (unsigned int)UI_GNSS__COUNT; i++) {
		sv[i] = sv_make((uint8_t)i, (uint8_t)(i + 1U), 40,
				(int16_t)(i * 50U), 45U, true);
	}
	TEST_ASSERT_EQUAL_INT(0, sky_render(&c, sv, (uint8_t)UI_GNSS__COUNT, NULL));

	for (i = 0U; i < (unsigned int)UI_GNSS__COUNT; i++) {
		TEST_ASSERT_TRUE(count_colour(&c,
			sky_colour_for_sys((uint8_t)i)) > 0U);
	}
}

/* Satellites below the horizon and with bad azimuths are skipped silently. */
static void test_render_skips_unplottable(void)
{
	sky_canvas_t c = canvas(60U, 60U);
	ui_sv_t sv[4];

	sv[0] = sv_make((uint8_t)UI_GNSS_GPS, 1U, -5, 0, 45U, true);
	sv[1] = sv_make((uint8_t)UI_GNSS_GPS, 2U, 100, 0, 45U, true);
	sv[2] = sv_make((uint8_t)UI_GNSS_GPS, 3U, 40, -10, 45U, true);
	sv[3] = sv_make((uint8_t)UI_GNSS_GPS, 4U, 40, 400, 45U, true);

	TEST_ASSERT_EQUAL_INT(0, sky_render(&c, sv, 4U, NULL));
	TEST_ASSERT_EQUAL_UINT(0U, count_colour(&c, (uint8_t)SKY_C_GPS));

	/* A count above UI_MAX_SV is clamped rather than read past. */
	{
		ui_sv_t many[UI_MAX_SV];
		unsigned int i;

		for (i = 0U; i < (unsigned int)UI_MAX_SV; i++) {
			many[i] = sv_make((uint8_t)UI_GNSS_GPS, (uint8_t)i, 45,
					  (int16_t)(i * 11U), 40U, true);
		}
		TEST_ASSERT_EQUAL_INT(0, sky_render(&c, many, 255U, NULL));
		TEST_ASSERT_TRUE(count_colour(&c, (uint8_t)SKY_C_GPS) > 0U);
	}
}

/* ===================================================================== *
 *  North-unverified badge and rotation
 * ===================================================================== */

static void test_badge_tracks_north_validity(void)
{
	sky_canvas_t c = canvas(60U, 60U);
	sky_orient_t o;

	/* NULL orientation: GNSS-north, badge up. */
	TEST_ASSERT_EQUAL_INT(0, sky_render(&c, NULL, 0U, NULL));
	TEST_ASSERT_EQUAL_UINT(2U * 60U, count_colour(&c, (uint8_t)SKY_C_BADGE));
	TEST_ASSERT_EQUAL_INT32(0, sky_rotation_ddeg(NULL));

	/* Uncalibrated compass: still GNSS-north, still badged. */
	(void)memset(&o, 0, sizeof(o));
	o.north_valid = false;
	o.heading_ddeg = 900;
	TEST_ASSERT_EQUAL_INT(0, sky_render(&c, NULL, 0U, &o));
	TEST_ASSERT_EQUAL_UINT(2U * 60U, count_colour(&c, (uint8_t)SKY_C_BADGE));
	/* The heading must be IGNORED, not applied, when north is unverified. */
	TEST_ASSERT_EQUAL_INT32(0, sky_rotation_ddeg(&o));

	/* Verified: no badge. */
	o.north_valid = true;
	TEST_ASSERT_EQUAL_INT(0, sky_render(&c, NULL, 0U, &o));
	TEST_ASSERT_EQUAL_UINT(0U, count_colour(&c, (uint8_t)SKY_C_BADGE));
}

static void test_rotation_arithmetic(void)
{
	sky_orient_t o;

	(void)memset(&o, 0, sizeof(o));
	o.north_valid = true;

	/* Rotation is the negative of (heading + declination), normalised. */
	o.heading_ddeg = 0;
	o.declination_ddeg = 0;
	TEST_ASSERT_EQUAL_INT32(0, sky_rotation_ddeg(&o));

	o.heading_ddeg = 900;
	TEST_ASSERT_EQUAL_INT32(2700, sky_rotation_ddeg(&o));

	o.heading_ddeg = 0;
	o.declination_ddeg = 150;
	TEST_ASSERT_EQUAL_INT32(3450, sky_rotation_ddeg(&o));

	/* A westerly (negative) declination rotates the other way. */
	o.declination_ddeg = -150;
	TEST_ASSERT_EQUAL_INT32(150, sky_rotation_ddeg(&o));

	/* Both terms together, wrapping past a full turn. */
	o.heading_ddeg = 3500;
	o.declination_ddeg = 200;
	TEST_ASSERT_EQUAL_INT32(3600 - ((3500 + 200) % 3600),
				sky_rotation_ddeg(&o));
}

/*
 * With north verified, a satellite's marker moves. Facing east (heading 90) the
 * plot rotates 270, so a satellite at azimuth 90 draws where north used to be.
 */
static void test_rotation_moves_markers(void)
{
	sky_canvas_t c = canvas(60U, 60U);
	sky_orient_t o;
	ui_sv_t sv = sv_make((uint8_t)UI_GNSS_GPS, 1U, 0, 90, 45U, true);
	int32_t cx = 30;
	int32_t cy = 30;
	int32_t r = 29;

	(void)memset(&o, 0, sizeof(o));
	o.north_valid = true;
	o.heading_ddeg = 900;
	o.declination_ddeg = 0;

	TEST_ASSERT_EQUAL_INT(0, sky_render(&c, &sv, 1U, &o));

	/* The marker is at the top of the plot, not the right. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_GPS,
				sky_canvas_get(&c, cx, cy - r));

	/* Without the rotation it would be on the right. */
	o.north_valid = false;
	TEST_ASSERT_EQUAL_INT(0, sky_render(&c, &sv, 1U, &o));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_GPS,
				sky_canvas_get(&c, cx + r, cy));
}

/* ===================================================================== *
 *  Tilt-compensated heading
 * ===================================================================== */

static void test_heading_level_board(void)
{
	sky_ecompass_t e;
	sky_heading_t h;

	/*
	 * Board flat, +z up (gravity along -z as an accelerometer reports it as
	 * +z when the axis points up; only the ratios matter here). Field pointing
	 * along +y: heading 0.
	 */
	(void)memset(&e, 0, sizeof(e));
	e.calibrated = true;
	e.acc[2] = 1000;
	e.mag[1] = 300;

	TEST_ASSERT_EQUAL_INT(0, sky_heading_from_ecompass(&e, 300U, &h));
	TEST_ASSERT_TRUE(h.valid);
	TEST_ASSERT_TRUE(h.tilt_valid);
	TEST_ASSERT_INT_WITHIN(30, 0, h.heading_ddeg);
	TEST_ASSERT_EQUAL_UINT32(300U, h.field_mag);

	/* Field along +x: heading 90. */
	(void)memset(e.mag, 0, sizeof(e.mag));
	e.mag[0] = 300;
	TEST_ASSERT_EQUAL_INT(0, sky_heading_from_ecompass(&e, 300U, &h));
	TEST_ASSERT_TRUE(h.valid);
	TEST_ASSERT_INT_WITHIN(30, 900, h.heading_ddeg);

	/* Field along -y: heading 180. */
	(void)memset(e.mag, 0, sizeof(e.mag));
	e.mag[1] = -300;
	TEST_ASSERT_EQUAL_INT(0, sky_heading_from_ecompass(&e, 300U, &h));
	TEST_ASSERT_INT_WITHIN(30, 1800, h.heading_ddeg);

	/* Field along -x: heading 270. */
	(void)memset(e.mag, 0, sizeof(e.mag));
	e.mag[0] = -300;
	TEST_ASSERT_EQUAL_INT(0, sky_heading_from_ecompass(&e, 300U, &h));
	TEST_ASSERT_INT_WITHIN(30, 2700, h.heading_ddeg);
}

/*
 * The board is mounted vertically (§6.3), so tilt compensation is not a
 * refinement — it is what makes the heading mean anything. This is the test that
 * would fail if the de-rotation were dropped.
 */
static void test_heading_compensates_tilt(void)
{
	sky_ecompass_t e;
	sky_heading_t h_level;
	sky_heading_t h_tilted;

	/* Level reference: field 45 degrees between +y and +x. */
	(void)memset(&e, 0, sizeof(e));
	e.calibrated = true;
	e.acc[2] = 1000;
	e.mag[0] = 200;
	e.mag[1] = 200;
	e.mag[2] = 0;
	TEST_ASSERT_EQUAL_INT(0, sky_heading_from_ecompass(&e, 0U, &h_level));
	TEST_ASSERT_TRUE(h_level.valid);
	TEST_ASSERT_INT_WITHIN(40, 450, h_level.heading_ddeg);

	/*
	 * Now pitch the board 90 degrees forward — the as-built vertical mount.
	 * Gravity moves to -y, and the field's +y component rotates into -z. A
	 * naive atan2(mx, my) would read 90 degrees; the compensated answer stays
	 * at 45.
	 */
	(void)memset(&e, 0, sizeof(e));
	e.calibrated = true;
	e.acc[1] = -1000;
	e.mag[0] = 200;
	e.mag[1] = 0;
	e.mag[2] = -200;
	TEST_ASSERT_EQUAL_INT(0, sky_heading_from_ecompass(&e, 0U, &h_tilted));
	TEST_ASSERT_TRUE(h_tilted.tilt_valid);
	TEST_ASSERT_INT_WITHIN(60, 450, h_tilted.heading_ddeg);
	/* Pitch is reported, and it is the 90 degrees we applied. */
	TEST_ASSERT_INT_WITHIN(60, 900, h_tilted.pitch_ddeg);
}

static void test_heading_rejects_untrustworthy_input(void)
{
	sky_ecompass_t e;
	sky_heading_t h;

	/* Uncalibrated: a heading is computed but must not be believed. */
	(void)memset(&e, 0, sizeof(e));
	e.calibrated = false;
	e.acc[2] = 1000;
	e.mag[1] = 300;
	TEST_ASSERT_EQUAL_INT(0, sky_heading_from_ecompass(&e, 300U, &h));
	TEST_ASSERT_FALSE(h.valid);
	TEST_ASSERT_TRUE(h.tilt_valid);

	/* Calibrated but the field is implausibly strong: something is nearby. */
	e.calibrated = true;
	e.mag[1] = 3000;
	TEST_ASSERT_EQUAL_INT(0, sky_heading_from_ecompass(&e, 300U, &h));
	TEST_ASSERT_FALSE(h.valid);

	/* ...or implausibly weak. */
	e.mag[1] = 10;
	TEST_ASSERT_EQUAL_INT(0, sky_heading_from_ecompass(&e, 300U, &h));
	TEST_ASSERT_FALSE(h.valid);

	/* Inside the tolerance band: believed. */
	e.mag[1] = 320;
	TEST_ASSERT_EQUAL_INT(0, sky_heading_from_ecompass(&e, 300U, &h));
	TEST_ASSERT_TRUE(h.valid);

	/* No gravity vector at all: no tilt compensation is possible. */
	(void)memset(&e, 0, sizeof(e));
	e.calibrated = true;
	e.mag[1] = 300;
	TEST_ASSERT_EQUAL_INT(0, sky_heading_from_ecompass(&e, 0U, &h));
	TEST_ASSERT_FALSE(h.tilt_valid);
	TEST_ASSERT_FALSE(h.valid);

	/* A zero field is never a heading. */
	(void)memset(&e, 0, sizeof(e));
	e.calibrated = true;
	e.acc[2] = 1000;
	TEST_ASSERT_EQUAL_INT(0, sky_heading_from_ecompass(&e, 0U, &h));
	TEST_ASSERT_FALSE(h.valid);

	TEST_ASSERT_EQUAL_INT(-EINVAL, sky_heading_from_ecompass(NULL, 0U, &h));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sky_heading_from_ecompass(&e, 0U, NULL));
}

/* Hard- and soft-iron correction is applied before anything else. */
static void test_heading_applies_calibration(void)
{
	sky_ecompass_t e;
	sky_heading_t h;

	/*
	 * A large hard-iron offset on +x. Uncorrected the heading would be pulled
	 * towards east; with the offset subtracted it reads north.
	 */
	(void)memset(&e, 0, sizeof(e));
	e.calibrated = true;
	e.acc[2] = 1000;
	e.mag[0] = 1000;  /* all of it is the offset */
	e.mag[1] = 300;
	e.mag_offset[0] = 1000;
	TEST_ASSERT_EQUAL_INT(0, sky_heading_from_ecompass(&e, 300U, &h));
	TEST_ASSERT_TRUE(h.valid);
	TEST_ASSERT_INT_WITHIN(30, 0, h.heading_ddeg);

	/* Soft-iron scale: zero is treated as unity, so a zeroed struct works. */
	(void)memset(&e, 0, sizeof(e));
	e.calibrated = true;
	e.acc[2] = 1000;
	e.mag[1] = 300;
	e.mag_scale[0] = 0;
	e.mag_scale[1] = 0;
	e.mag_scale[2] = 0;
	TEST_ASSERT_EQUAL_INT(0, sky_heading_from_ecompass(&e, 300U, &h));
	TEST_ASSERT_TRUE(h.valid);
	TEST_ASSERT_EQUAL_UINT32(300U, h.field_mag);

	/* An explicit half-scale on y halves the measured magnitude. */
	e.mag_scale[1] = 2048;
	TEST_ASSERT_EQUAL_INT(0, sky_heading_from_ecompass(&e, 150U, &h));
	TEST_ASSERT_EQUAL_UINT32(150U, h.field_mag);
}

/* ===================================================================== *
 *  Declination
 * ===================================================================== */

static void test_declination_dipole(void)
{
	int16_t d = 0;

	/*
	 * The centred-dipole model, with the accuracy caveat in skyplot.h. These
	 * assertions pin sign and rough magnitude, which is all the model
	 * supports — a tighter bound would be asserting a precision it does not
	 * have.
	 */

	/* Williams Manor, roughly 39.0 N 77.5 W: declination is westerly. */
	TEST_ASSERT_EQUAL_INT(0, sky_declination_dipole_ddeg(390000000L,
							     -775000000L, &d));
	TEST_ASSERT_TRUE(d < 0);
	TEST_ASSERT_TRUE(d > -300);

	/* Western North America, roughly 47 N 122 W: easterly. */
	TEST_ASSERT_EQUAL_INT(0, sky_declination_dipole_ddeg(470000000L,
							     -1220000000L, &d));
	TEST_ASSERT_TRUE(d > 0);

	/* On the pole's own meridian the declination is essentially zero. */
	TEST_ASSERT_EQUAL_INT(0, sky_declination_dipole_ddeg(450000000L,
							     -727000000L, &d));
	TEST_ASSERT_INT_WITHIN(60, 0, d);

	/* Always folded to +-180 degrees, never 0..360. */
	{
		int32_t lat;
		int32_t lon;

		for (lat = -600000000L; lat <= 600000000L; lat += 150000000L) {
			for (lon = -1800000000L; lon < 1800000000L;
			     lon += 300000000L) {
				TEST_ASSERT_EQUAL_INT(0,
					sky_declination_dipole_ddeg(lat, lon, &d));
				TEST_ASSERT_TRUE(d >= -1800);
				TEST_ASSERT_TRUE(d <= 1800);
			}
		}
	}

	/* Rejections. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, sky_declination_dipole_ddeg(0L, 0L, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		sky_declination_dipole_ddeg(900000001L, 0L, &d));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		sky_declination_dipole_ddeg(-900000001L, 0L, &d));
}

/* ===================================================================== *
 *  Degenerate canvases
 * ===================================================================== */

static void test_tiny_canvas_does_not_misbehave(void)
{
	sky_canvas_t c;
	ui_sv_t sv = sv_make((uint8_t)UI_GNSS_GPS, 1U, 45, 45, 45U, true);

	/* 1x1: the radius clamps to 1 and nothing runs off the buffer. */
	TEST_ASSERT_EQUAL_INT(0, sky_canvas_init(&c, px, 1U, 1U, sizeof(px)));
	TEST_ASSERT_EQUAL_INT(0, sky_render(&c, &sv, 1U, NULL));

	/* 3x3. */
	TEST_ASSERT_EQUAL_INT(0, sky_canvas_init(&c, px, 3U, 3U, sizeof(px)));
	TEST_ASSERT_EQUAL_INT(0, sky_render(&c, &sv, 1U, NULL));

	/* Strongly non-square: the disc is sized by the shorter axis. */
	TEST_ASSERT_EQUAL_INT(0, sky_canvas_init(&c, px, 80U, 20U, sizeof(px)));
	TEST_ASSERT_EQUAL_INT(0, sky_render(&c, &sv, 1U, NULL));
	TEST_ASSERT_EQUAL_UINT(2U * 80U, count_colour(&c, (uint8_t)SKY_C_BADGE));

	/* And the maximum. */
	TEST_ASSERT_EQUAL_INT(0, sky_canvas_init(&c, px, (uint16_t)SKY_MAX_DIM,
			(uint16_t)SKY_MAX_DIM, sizeof(px)));
	TEST_ASSERT_EQUAL_INT(0, sky_render(&c, &sv, 1U, NULL));
}

/* A realistic constellation on the as-built plot size, as a smoke test. */
static void test_realistic_sky(void)
{
	sky_canvas_t c = canvas(240U, 240U);
	sky_orient_t o;
	ui_sv_t sv[12];
	unsigned int i;

	sv[0] = sv_make((uint8_t)UI_GNSS_GPS, 1U, 78, 45, 48U, true);
	sv[1] = sv_make((uint8_t)UI_GNSS_GPS, 11U, 55, 130, 45U, true);
	sv[2] = sv_make((uint8_t)UI_GNSS_GPS, 14U, 33, 210, 41U, true);
	sv[3] = sv_make((uint8_t)UI_GNSS_GPS, 22U, 12, 300, 32U, false);
	sv[4] = sv_make((uint8_t)UI_GNSS_GALILEO, 5U, 66, 78, 46U, true);
	sv[5] = sv_make((uint8_t)UI_GNSS_GALILEO, 9U, 24, 190, 36U, true);
	sv[6] = sv_make((uint8_t)UI_GNSS_GLONASS, 3U, 44, 15, 43U, true);
	sv[7] = sv_make((uint8_t)UI_GNSS_GLONASS, 18U, 8, 255, 28U, false);
	sv[8] = sv_make((uint8_t)UI_GNSS_BEIDOU, 27U, 51, 340, 44U, true);
	sv[9] = sv_make((uint8_t)UI_GNSS_BEIDOU, 30U, 19, 100, 31U, false);
	sv[10] = sv_make((uint8_t)UI_GNSS_SBAS, 33U, 29, 200, 38U, false);
	sv[11] = sv_make((uint8_t)UI_GNSS_QZSS, 44U, 62, 160, 42U, true);

	(void)memset(&o, 0, sizeof(o));
	o.north_valid = true;
	o.heading_ddeg = 1234;
	o.declination_ddeg = -112;
	o.tilt_valid = true;

	TEST_ASSERT_EQUAL_INT(0, sky_render(&c, sv, 12U, &o));

	/* Every constellation present is drawn, and the plot is not badged. */
	for (i = 0U; i < 12U; i++) {
		TEST_ASSERT_TRUE(count_colour(&c,
			sky_colour_for_sys(sv[i].sys)) > 0U);
	}
	TEST_ASSERT_EQUAL_UINT(0U, count_colour(&c, (uint8_t)SKY_C_BADGE));

	/* Nothing was drawn outside the disc. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_BG, sky_canvas_get(&c, 0, 0));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_BG, sky_canvas_get(&c, 239, 0));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_BG, sky_canvas_get(&c, 0, 239));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)SKY_C_BG, sky_canvas_get(&c, 239, 239));

	/* Rendering is deterministic: the same inputs give the same pixels. */
	{
		static uint8_t again[SKY_MAX_DIM * SKY_MAX_DIM];
		sky_canvas_t c2;

		TEST_ASSERT_EQUAL_INT(0, sky_canvas_init(&c2, again, 240U, 240U,
							sizeof(again)));
		TEST_ASSERT_EQUAL_INT(0, sky_render(&c2, sv, 12U, &o));
		TEST_ASSERT_EQUAL_HEX8_ARRAY(px, again, 240U * 240U);
	}
}

/* ===================================================================== *
 *  Runner
 * ===================================================================== */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_canvas_init);
	RUN_TEST(test_palette_and_ascii_map);

	RUN_TEST(test_trig);
	RUN_TEST(test_atan2);
	RUN_TEST(test_isqrt);

	RUN_TEST(test_place);
	RUN_TEST(test_marker_radius);

	RUN_TEST(test_golden_empty_gnss_north);
	RUN_TEST(test_golden_used_vs_visible);
	RUN_TEST(test_all_constellations_are_distinguishable);
	RUN_TEST(test_render_skips_unplottable);

	RUN_TEST(test_badge_tracks_north_validity);
	RUN_TEST(test_rotation_arithmetic);
	RUN_TEST(test_rotation_moves_markers);

	RUN_TEST(test_heading_level_board);
	RUN_TEST(test_heading_compensates_tilt);
	RUN_TEST(test_heading_rejects_untrustworthy_input);
	RUN_TEST(test_heading_applies_calibration);

	RUN_TEST(test_declination_dipole);

	RUN_TEST(test_tiny_canvas_does_not_misbehave);
	RUN_TEST(test_realistic_sky);

	return UNITY_END();
}
