/*
 * STS1000 "Meridian" — core/ui: polar skyplot renderer.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See skyplot.h for the geometry, the palette and the true-north honesty rules.
 * Everything here is integer arithmetic: core carries no libm (ARCHITECTURE.md
 * §4) and a skyplot is the one place in this firmware where that constraint
 * actually bites.
 */

#include "ui/skyplot.h"

#include <errno.h>
#include <string.h>

/* ===================================================================== *
 *  Trigonometry
 * ===================================================================== */

/*
 * sin(theta) * 4096 for theta = 0, 1, ... 90 degrees.
 *
 * Whole-degree granularity with linear interpolation between entries. The
 * interpolation error peaks near zero crossings at about 0.6 LSB of 4096, i.e.
 * 0.015 % — far below one pixel on a 320-pixel canvas, which is the only
 * resolution that matters here.
 */
static const uint16_t sin_tbl[91] = {
	   0,   71,  143,  214,  286,  357,  428,  499,  570,  641,
	 711,  781,  851,  921,  990, 1059, 1128, 1196, 1264, 1331,
	1398, 1465, 1531, 1596, 1661, 1725, 1789, 1852, 1915, 1977,
	2038, 2098, 2158, 2217, 2276, 2333, 2390, 2446, 2501, 2556,
	2609, 2662, 2714, 2765, 2815, 2865, 2913, 2960, 3007, 3053,
	3097, 3141, 3183, 3225, 3266, 3305, 3344, 3381, 3417, 3453,
	3487, 3520, 3552, 3583, 3612, 3641, 3668, 3695, 3720, 3744,
	3767, 3788, 3809, 3828, 3846, 3863, 3879, 3893, 3907, 3919,
	3930, 3939, 3948, 3955, 3961, 3966, 3970, 3972, 3974, 3974,
	4096,
};

/* Normalise tenths of a degree into 0..3599, for any input including negatives. */
static int32_t norm_ddeg(int32_t ddeg)
{
	int32_t v = ddeg % 3600;

	if (v < 0) {
		v += 3600;
	}
	return v;
}

/* sin over the first quadrant, argument in tenths of a degree, 0..900. */
static int32_t sin_q1(int32_t ddeg)
{
	int32_t deg = ddeg / 10;
	int32_t frac = ddeg % 10;
	int32_t a;
	int32_t b;

	if (deg >= 90) {
		return SKY_TRIG_ONE;
	}
	a = (int32_t)sin_tbl[deg];
	b = (int32_t)sin_tbl[deg + 1];
	return a + (((b - a) * frac) / 10);
}

int32_t sky_sin_ddeg(int32_t ddeg)
{
	int32_t v = norm_ddeg(ddeg);

	if (v <= 900) {
		return sin_q1(v);
	}
	if (v <= 1800) {
		return sin_q1(1800 - v);
	}
	if (v <= 2700) {
		return -sin_q1(v - 1800);
	}
	return -sin_q1(3600 - v);
}

int32_t sky_cos_ddeg(int32_t ddeg)
{
	return sky_sin_ddeg(ddeg + 900);
}

uint32_t sky_isqrt(uint64_t v)
{
	uint64_t rem = 0U;
	uint64_t root = 0U;
	int i;

	for (i = 0; i < 32; i++) {
		root <<= 1;
		rem = (rem << 2) | (v >> 62);
		v <<= 2;
		if (root < rem) {
			root++;
			rem -= root;
			root++;
		}
	}
	return (uint32_t)(root >> 1);
}

/*
 * atan(z) for 0 <= z <= 1, z in Q12, result in tenths of a degree.
 *
 * The classic single-rational approximation: atan(z) ~ z * (c1 - c2*z^2), with
 * the constants scaled so the output lands directly in tenths of a degree. Peak
 * error is about 0.2 degrees, which is a fifth of a pixel at the rim of a
 * 320-pixel plot.
 */
static int32_t atan_unit_q12(int32_t z)
{
	int32_t z2 = (int32_t)(((int64_t)z * (int64_t)z) >> 12);
	int64_t t;

	/* 450 tenths of a degree at z = 1, with the cubic term shaping the curve. */
	t = (int64_t)z * (int64_t)(5730 - (((int64_t)1266 * z2) >> 12));
	return (int32_t)(t >> 12);
}

int32_t sky_atan2_ddeg(int32_t y, int32_t x)
{
	int32_t ax;
	int32_t ay;
	int32_t base;
	int32_t a;
	bool swap;

	if ((x == 0) && (y == 0)) {
		return 0;
	}

	ax = (x < 0) ? -x : x;
	ay = (y < 0) ? -y : y;

	/*
	 * Reduce to the octant where the ratio is <= 1, then map back. Working on
	 * magnitudes and adding the quadrant afterwards keeps the approximation
	 * inside its valid domain.
	 */
	swap = (ax > ay);
	if (swap) {
		a = atan_unit_q12((int32_t)(((int64_t)ay << 12) / ax));
		base = 900 - a; /* measured from +y, so a large |x| is near 90 deg */
	} else {
		a = atan_unit_q12((int32_t)(((int64_t)ax << 12) / ay));
		base = a;
	}

	/* Compass convention: 0 at +y, increasing clockwise through +x. */
	if ((x >= 0) && (y >= 0)) {
		return norm_ddeg(base);
	}
	if ((x >= 0) && (y < 0)) {
		return norm_ddeg(1800 - base);
	}
	if ((x < 0) && (y < 0)) {
		return norm_ddeg(1800 + base);
	}
	return norm_ddeg(3600 - base);
}

/* ===================================================================== *
 *  Canvas
 * ===================================================================== */

uint8_t sky_colour_for_sys(uint8_t sys)
{
	switch (sys) {
	case (uint8_t)UI_GNSS_GPS:
		return (uint8_t)SKY_C_GPS;
	case (uint8_t)UI_GNSS_GALILEO:
		return (uint8_t)SKY_C_GALILEO;
	case (uint8_t)UI_GNSS_GLONASS:
		return (uint8_t)SKY_C_GLONASS;
	case (uint8_t)UI_GNSS_BEIDOU:
		return (uint8_t)SKY_C_BEIDOU;
	case (uint8_t)UI_GNSS_SBAS:
		return (uint8_t)SKY_C_SBAS;
	case (uint8_t)UI_GNSS_QZSS:
		return (uint8_t)SKY_C_QZSS;
	default:
		return (uint8_t)SKY_C_OTHER;
	}
}

int sky_canvas_init(sky_canvas_t *c, uint8_t *px, uint16_t w, uint16_t h,
		    size_t cap)
{
	if ((c == NULL) || (px == NULL)) {
		return -EINVAL;
	}
	if ((w == 0U) || (h == 0U) || (w > (uint16_t)SKY_MAX_DIM) ||
	    (h > (uint16_t)SKY_MAX_DIM)) {
		return -EINVAL;
	}
	if (cap < ((size_t)w * (size_t)h)) {
		return -ENOSPC;
	}

	c->px = px;
	c->w = w;
	c->h = h;
	c->cap = cap;
	sky_canvas_clear(c);
	return 0;
}

void sky_canvas_clear(sky_canvas_t *c)
{
	if ((c == NULL) || (c->px == NULL)) {
		return;
	}
	(void)memset(c->px, (int)SKY_C_BG, (size_t)c->w * (size_t)c->h);
}

static void put_px(sky_canvas_t *c, int32_t x, int32_t y, uint8_t colour)
{
	if ((x < 0) || (y < 0) || (x >= (int32_t)c->w) || (y >= (int32_t)c->h)) {
		return;
	}
	c->px[((size_t)y * (size_t)c->w) + (size_t)x] = colour;
}

uint8_t sky_canvas_get(const sky_canvas_t *c, int32_t x, int32_t y)
{
	if ((c == NULL) || (c->px == NULL)) {
		return (uint8_t)SKY_C_BG;
	}
	if ((x < 0) || (y < 0) || (x >= (int32_t)c->w) || (y >= (int32_t)c->h)) {
		return (uint8_t)SKY_C_BG;
	}
	return c->px[((size_t)y * (size_t)c->w) + (size_t)x];
}

char sky_ascii_char(uint8_t colour)
{
	switch (colour) {
	case (uint8_t)SKY_C_BG:
		return '.';
	case (uint8_t)SKY_C_GRID:
		return '-';
	case (uint8_t)SKY_C_GRID_MAJOR:
		return '=';
	case (uint8_t)SKY_C_GPS:
		return '0';
	case (uint8_t)SKY_C_GALILEO:
		return '1';
	case (uint8_t)SKY_C_GLONASS:
		return '2';
	case (uint8_t)SKY_C_BEIDOU:
		return '3';
	case (uint8_t)SKY_C_SBAS:
		return '4';
	case (uint8_t)SKY_C_QZSS:
		return '5';
	case (uint8_t)SKY_C_OTHER:
		return '6';
	case (uint8_t)SKY_C_BADGE:
		return '#';
	default:
		return '?';
	}
}

size_t sky_canvas_to_ascii(const sky_canvas_t *c, char *out, size_t cap)
{
	size_t n = 0U;
	uint16_t y;
	uint16_t x;

	if ((c == NULL) || (c->px == NULL) || (out == NULL) || (cap == 0U)) {
		return 0U;
	}
	out[0] = '\0';

	for (y = 0U; y < c->h; y++) {
		for (x = 0U; x < c->w; x++) {
			if ((n + 1U) >= cap) {
				out[n] = '\0';
				return n;
			}
			out[n] = sky_ascii_char(sky_canvas_get(c, (int32_t)x,
							       (int32_t)y));
			n++;
		}
		if ((n + 1U) >= cap) {
			out[n] = '\0';
			return n;
		}
		out[n] = '\n';
		n++;
	}
	out[n] = '\0';
	return n;
}

/* ===================================================================== *
 *  Orientation
 * ===================================================================== */

static int32_t scale_or_unity(int32_t s)
{
	return (s == 0) ? 4096 : s;
}

int sky_heading_from_ecompass(const sky_ecompass_t *e, uint32_t field_ref,
			      sky_heading_t *out)
{
	int32_t mx;
	int32_t my;
	int32_t mz;
	int32_t ax;
	int32_t ay;
	int32_t az;
	int32_t pitch;
	int32_t roll;
	int32_t sp;
	int32_t cp;
	int32_t sr;
	int32_t cr;
	int64_t xh;
	int64_t yh;
	uint32_t mag;
	uint32_t acc_mag;

	if ((e == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	(void)memset(out, 0, sizeof(*out));

	/* Hard-iron offset, then soft-iron scale (§10.4 calibration). */
	mx = ((e->mag[0] - e->mag_offset[0]) * scale_or_unity(e->mag_scale[0])) / 4096;
	my = ((e->mag[1] - e->mag_offset[1]) * scale_or_unity(e->mag_scale[1])) / 4096;
	mz = ((e->mag[2] - e->mag_offset[2]) * scale_or_unity(e->mag_scale[2])) / 4096;

	ax = e->acc[0];
	ay = e->acc[1];
	az = e->acc[2];

	mag = sky_isqrt(((uint64_t)((int64_t)mx * mx)) +
			((uint64_t)((int64_t)my * my)) +
			((uint64_t)((int64_t)mz * mz)));
	out->field_mag = mag;

	acc_mag = sky_isqrt(((uint64_t)((int64_t)ax * ax)) +
			    ((uint64_t)((int64_t)ay * ay)) +
			    ((uint64_t)((int64_t)az * az)));

	if (acc_mag == 0U) {
		/* No gravity vector: nothing to tilt-compensate against. */
		return 0;
	}
	out->tilt_valid = true;

	/*
	 * Pitch and roll from gravity. The board stands vertically (§6.3), so
	 * these are large in normal service — which is exactly why skipping the
	 * compensation would produce a heading that is wrong by tens of degrees
	 * rather than by a rounding error.
	 */
	roll = sky_atan2_ddeg(ax, az);
	pitch = sky_atan2_ddeg(-ay, (int32_t)sky_isqrt(
			((uint64_t)((int64_t)ax * ax)) +
			((uint64_t)((int64_t)az * az))));

	out->roll_ddeg = (int16_t)((roll > 1800) ? (roll - 3600) : roll);
	out->pitch_ddeg = (int16_t)((pitch > 1800) ? (pitch - 3600) : pitch);

	sp = sky_sin_ddeg(pitch);
	cp = sky_cos_ddeg(pitch);
	sr = sky_sin_ddeg(roll);
	cr = sky_cos_ddeg(roll);

	/*
	 * De-rotate the magnetic vector into the horizontal plane. Standard
	 * tilt-compensated compass, in Q12 throughout:
	 *
	 *   xh = mx*cr + mz*sr
	 *   yh = mx*sr*sp + my*cp - mz*cr*sp
	 */
	xh = (((int64_t)mx * cr) + ((int64_t)mz * sr)) / SKY_TRIG_ONE;
	yh = ((((int64_t)mx * sr) / SKY_TRIG_ONE) * sp) / SKY_TRIG_ONE;
	yh += ((int64_t)my * cp) / SKY_TRIG_ONE;
	yh -= ((((int64_t)mz * cr) / SKY_TRIG_ONE) * sp) / SKY_TRIG_ONE;

	if ((xh == 0) && (yh == 0)) {
		return 0;
	}

	out->heading_ddeg = (int16_t)sky_atan2_ddeg((int32_t)yh, (int32_t)xh);

	/*
	 * Only now decide whether to believe it. Two independent reasons not to:
	 * the §10.4 calibration was never performed, or the field magnitude is
	 * outside the plausible band because something ferrous or energised is
	 * next to the sensor. Either way §6.3 says fall back to GNSS-north and
	 * say so rather than draw a confident wrong picture.
	 */
	if (!e->calibrated) {
		return 0;
	}
	if (field_ref != 0U) {
		uint32_t tol = (field_ref * (uint32_t)SKY_FIELD_TOLERANCE_PCT) / 100U;

		if ((mag + tol) < field_ref) {
			return 0;
		}
		if (mag > (field_ref + tol)) {
			return 0;
		}
	}
	if (mag == 0U) {
		return 0;
	}

	out->valid = true;
	return 0;
}

int sky_declination_dipole_ddeg(int32_t lat_1e7, int32_t lon_1e7, int16_t *out)
{
	/*
	 * IGRF-14 epoch-2025 geomagnetic north pole: 80.7 N, 72.7 W. The centred
	 * dipole declination is the bearing difference between geographic north
	 * and the great-circle bearing to that pole:
	 *
	 *   D = atan2( sin(lon_p - lon) * cos(lat_p),
	 *              sin(lat_p)*cos(lat) - cos(lat_p)*sin(lat)*cos(lon_p - lon) )
	 *
	 * Accuracy: a few degrees at mid latitudes, much worse above ~60 deg or
	 * near an agonic line. That is why skyplot.h prefers a commissioned site
	 * constant and flags this result as modelled.
	 */
	const int32_t pole_lat_ddeg = 807;
	const int32_t pole_lon_ddeg = -727;
	int32_t lat_ddeg;
	int32_t lon_ddeg;
	int32_t dlon;
	int64_t y;
	int64_t x;

	if (out == NULL) {
		return -EINVAL;
	}
	if ((lat_1e7 > 900000000L) || (lat_1e7 < -900000000L)) {
		return -EINVAL;
	}

	/* 1e-7 degrees -> tenths of a degree. */
	lat_ddeg = lat_1e7 / 1000000L;
	lon_ddeg = lon_1e7 / 1000000L;
	dlon = pole_lon_ddeg - lon_ddeg;

	y = ((int64_t)sky_sin_ddeg(dlon) * sky_cos_ddeg(pole_lat_ddeg)) /
	    SKY_TRIG_ONE;
	x = ((int64_t)sky_sin_ddeg(pole_lat_ddeg) * sky_cos_ddeg(lat_ddeg)) /
	    SKY_TRIG_ONE;
	x -= ((((int64_t)sky_cos_ddeg(pole_lat_ddeg) * sky_sin_ddeg(lat_ddeg)) /
	       SKY_TRIG_ONE) * sky_cos_ddeg(dlon)) / SKY_TRIG_ONE;

	if ((x == 0) && (y == 0)) {
		*out = 0;
		return 0;
	}

	{
		/*
		 * sky_atan2_ddeg() is compass-convention (0 at +y). Here the
		 * numerator y is the "east" term and the denominator x the
		 * "north" term, which is the same convention, so the bearing
		 * comes out directly. Fold to +-180 degrees: a declination of
		 * 350 degrees is -10.
		 */
		int32_t b = sky_atan2_ddeg((int32_t)y, (int32_t)x);

		if (b > 1800) {
			b -= 3600;
		}
		*out = (int16_t)b;
	}
	return 0;
}

int32_t sky_rotation_ddeg(const sky_orient_t *o)
{
	if ((o == NULL) || !o->north_valid) {
		/* GNSS-north: azimuths exactly as the receiver reported them. */
		return 0;
	}
	/*
	 * Rotate by the negative of the true heading so that true north ends up
	 * pointing at the top of the plot.
	 */
	return norm_ddeg(-((int32_t)o->heading_ddeg + (int32_t)o->declination_ddeg));
}

/* ===================================================================== *
 *  Render
 * ===================================================================== */

int32_t sky_marker_radius(uint8_t cno_dbhz)
{
	int32_t span = (int32_t)SKY_CNO_MAX - (int32_t)SKY_CNO_MIN;
	int32_t c;

	if (cno_dbhz <= (uint8_t)SKY_CNO_MIN) {
		return SKY_MARKER_MIN_R;
	}
	if (cno_dbhz >= (uint8_t)SKY_CNO_MAX) {
		return SKY_MARKER_MAX_R;
	}
	c = (int32_t)cno_dbhz - (int32_t)SKY_CNO_MIN;
	return SKY_MARKER_MIN_R +
	       ((c * (SKY_MARKER_MAX_R - SKY_MARKER_MIN_R)) / span);
}

/* Plot centre and radius: the largest disc that fits, leaving a badge margin. */
static void geometry(const sky_canvas_t *c, int32_t *cx, int32_t *cy, int32_t *r)
{
	int32_t w = (int32_t)c->w;
	int32_t h = (int32_t)c->h;
	int32_t d = (w < h) ? w : h;

	*cx = w / 2;
	*cy = h / 2;
	/* One pixel of margin so the horizon ring is never clipped. */
	*r = (d / 2) - 1;
	if (*r < 1) {
		*r = 1;
	}
}

int sky_place(const sky_canvas_t *c, int8_t elev_deg, int16_t azim_deg,
	      int32_t rotation_ddeg, int32_t *x, int32_t *y)
{
	int32_t cx;
	int32_t cy;
	int32_t r;
	int32_t rr;
	int32_t az;

	if ((c == NULL) || (c->px == NULL) || (x == NULL) || (y == NULL)) {
		return -EINVAL;
	}
	if ((elev_deg < 0) || (elev_deg > 90)) {
		return -ERANGE;
	}
	if ((azim_deg < 0) || (azim_deg > 359)) {
		return -ERANGE;
	}

	geometry(c, &cx, &cy, &r);

	/* Radius linear in (90 - elevation): zenith at the centre, horizon at the rim. */
	rr = (r * (90 - (int32_t)elev_deg)) / 90;
	az = norm_ddeg(((int32_t)azim_deg * 10) + rotation_ddeg);

	*x = cx + ((rr * sky_sin_ddeg(az)) / SKY_TRIG_ONE);
	/* Screen y grows downward, so north (azimuth 0) must go up. */
	*y = cy - ((rr * sky_cos_ddeg(az)) / SKY_TRIG_ONE);
	return 0;
}

/* Midpoint circle, outline only. */
static void ring(sky_canvas_t *c, int32_t cx, int32_t cy, int32_t rad,
		 uint8_t colour)
{
	int32_t x = rad;
	int32_t y = 0;
	int32_t err = 1 - rad;

	if (rad <= 0) {
		put_px(c, cx, cy, colour);
		return;
	}

	while (x >= y) {
		put_px(c, cx + x, cy + y, colour);
		put_px(c, cx + y, cy + x, colour);
		put_px(c, cx - y, cy + x, colour);
		put_px(c, cx - x, cy + y, colour);
		put_px(c, cx - x, cy - y, colour);
		put_px(c, cx - y, cy - x, colour);
		put_px(c, cx + y, cy - x, colour);
		put_px(c, cx + x, cy - y, colour);
		y++;
		if (err < 0) {
			err += (2 * y) + 1;
		} else {
			x--;
			err += 2 * (y - x + 1);
		}
	}
}

/* Filled disc. */
static void disc(sky_canvas_t *c, int32_t cx, int32_t cy, int32_t rad,
		 uint8_t colour)
{
	int32_t dy;

	for (dy = -rad; dy <= rad; dy++) {
		int32_t dx;

		for (dx = -rad; dx <= rad; dx++) {
			if (((dx * dx) + (dy * dy)) <= (rad * rad)) {
				put_px(c, cx + dx, cy + dy, colour);
			}
		}
	}
}

/* A short radial tick at @p az_ddeg, from the rim inwards. */
static void tick(sky_canvas_t *c, int32_t cx, int32_t cy, int32_t r,
		 int32_t az_ddeg, int32_t len, uint8_t colour)
{
	int32_t i;

	for (i = 0; i <= len; i++) {
		int32_t rr = r - i;
		int32_t x = cx + ((rr * sky_sin_ddeg(az_ddeg)) / SKY_TRIG_ONE);
		int32_t y = cy - ((rr * sky_cos_ddeg(az_ddeg)) / SKY_TRIG_ONE);

		put_px(c, x, y, colour);
	}
}

int sky_render(sky_canvas_t *c, const ui_sv_t *sv, uint8_t n,
	       const sky_orient_t *o)
{
	int32_t cx;
	int32_t cy;
	int32_t r;
	int32_t rotation;
	uint8_t count;
	uint8_t i;
	bool north_ok = (o != NULL) && o->north_valid;

	if ((c == NULL) || (c->px == NULL)) {
		return -EINVAL;
	}
	if ((sv == NULL) && (n != 0U)) {
		return -EINVAL;
	}

	sky_canvas_clear(c);
	geometry(c, &cx, &cy, &r);
	rotation = sky_rotation_ddeg(o);

	/* Horizon (elevation 0) and the 30/60-degree elevation rings. */
	ring(c, cx, cy, r, (uint8_t)SKY_C_GRID_MAJOR);
	ring(c, cx, cy, (r * 2) / 3, (uint8_t)SKY_C_GRID);
	ring(c, cx, cy, r / 3, (uint8_t)SKY_C_GRID);

	/*
	 * Cardinal ticks. They rotate with the plot, so when north is verified the
	 * north tick genuinely points at true north, and when it is not the ticks
	 * are GNSS-north and the badge says so.
	 */
	tick(c, cx, cy, r, rotation, r / 8, (uint8_t)SKY_C_GRID_MAJOR);
	tick(c, cx, cy, r, rotation + 900, r / 12, (uint8_t)SKY_C_GRID);
	tick(c, cx, cy, r, rotation + 1800, r / 12, (uint8_t)SKY_C_GRID);
	tick(c, cx, cy, r, rotation + 2700, r / 12, (uint8_t)SKY_C_GRID);

	count = (n > (uint8_t)UI_MAX_SV) ? (uint8_t)UI_MAX_SV : n;
	for (i = 0U; i < count; i++) {
		const ui_sv_t *s = &sv[i];
		int32_t x = 0;
		int32_t y = 0;
		int32_t rad;
		uint8_t colour;

		if (sky_place(c, s->elev_deg, s->azim_deg, rotation, &x, &y) != 0) {
			continue;
		}
		colour = sky_colour_for_sys(s->sys);
		rad = sky_marker_radius(s->cno_dbhz);

		if (s->used) {
			/* Filled = contributing to the solution (§6.3). */
			disc(c, x, y, rad, colour);
		} else {
			/* Hollow = visible but not used. */
			ring(c, x, y, rad, colour);
		}
	}

	if (!north_ok) {
		/*
		 * The "north unverified" badge: a solid band along the bottom two
		 * rows. Graphical rather than textual because this module owns no
		 * font — the glue writes the words over it. It is drawn last so it
		 * is never obscured by a low-elevation marker.
		 */
		int32_t y;

		for (y = (int32_t)c->h - 2; y < (int32_t)c->h; y++) {
			int32_t x;

			for (x = 0; x < (int32_t)c->w; x++) {
				put_px(c, x, y, (uint8_t)SKY_C_BADGE);
			}
		}
	}

	return 0;
}
