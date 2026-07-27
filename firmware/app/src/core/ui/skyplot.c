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
	 711,  782,  852,  921,  991, 1060, 1129, 1198, 1266, 1334,
	1401, 1468, 1534, 1600, 1666, 1731, 1796, 1860, 1923, 1986,
	2048, 2110, 2171, 2231, 2290, 2349, 2408, 2465, 2522, 2578,
	2633, 2687, 2741, 2793, 2845, 2896, 2946, 2996, 3044, 3091,
	3138, 3183, 3228, 3271, 3314, 3355, 3396, 3435, 3474, 3511,
	3547, 3582, 3617, 3650, 3681, 3712, 3742, 3770, 3798, 3824,
	3849, 3873, 3896, 3917, 3937, 3956, 3974, 3991, 4006, 4021,
	4034, 4046, 4056, 4065, 4074, 4080, 4086, 4090, 4094, 4095,
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
 * The standard rational approximation
 *
 *   atan(z) = (pi/4)*z - z*(z - 1)*(0.2447 + 0.0663*z)      radians
 *
 * scaled into tenths of a degree by 1800/pi = 572.958:
 *
 *   atan(z) = 450*z - z*(z - 1)*(140.2 + 38.0*z)            tenths of a degree
 *
 * Exact at z = 0 and z = 1; peak error about 0.1 degrees in between, which is a
 * tenth of a pixel at the rim of a 320-pixel plot.
 */
static int32_t atan_unit_q12(int32_t z)
{
	int32_t t1 = (int32_t)(((int64_t)450 * (int64_t)z) >> 12);
	/* zf*(zf - 1) in Q12; negative over the whole open interval. */
	int64_t a = ((int64_t)z * (int64_t)(z - SKY_TRIG_ONE)) >> 12;
	/* 140.2 + 38.0*zf in Q12. */
	int64_t k = ((int64_t)140 << 12) + ((int64_t)38 * (int64_t)z);

	return t1 - (int32_t)((a * k) >> 24);
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
	int32_t m[3];
	int32_t g[3];
	int64_t mh[3];
	int64_t fh[3];
	int64_t mdotg;
	int64_t fdotg;
	int64_t dot;
	int64_t cross[3];
	int64_t sinterm;
	uint32_t mag;
	uint32_t acc_mag;
	uint32_t mh_len;
	uint32_t fh_len;
	unsigned int i;

	if ((e == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	(void)memset(out, 0, sizeof(*out));

	/* Hard-iron offset, then soft-iron scale (§10.4 calibration). */
	for (i = 0U; i < 3U; i++) {
		m[i] = ((e->mag[i] - e->mag_offset[i]) *
			scale_or_unity(e->mag_scale[i])) / SKY_TRIG_ONE;
		g[i] = e->acc[i];
	}

	mag = sky_isqrt(((uint64_t)((int64_t)m[0] * m[0])) +
			((uint64_t)((int64_t)m[1] * m[1])) +
			((uint64_t)((int64_t)m[2] * m[2])));
	out->field_mag = mag;

	acc_mag = sky_isqrt(((uint64_t)((int64_t)g[0] * g[0])) +
			    ((uint64_t)((int64_t)g[1] * g[1])) +
			    ((uint64_t)((int64_t)g[2] * g[2])));
	if (acc_mag == 0U) {
		/* No gravity vector: nothing defines the horizontal plane. */
		return 0;
	}
	out->tilt_valid = true;

	/*
	 * Pitch and roll, for telemetry. Reported in the usual aerospace sense
	 * about the board's own axes; the heading below does not use them.
	 *
	 * sky_atan2_ddeg() is compass-convention (0 at +y, clockwise), so a
	 * mathematical atan2(a, b) is written sky_atan2_ddeg(b, a).
	 */
	{
		int32_t roll = sky_atan2_ddeg(g[2], g[0]);
		int32_t pitch = sky_atan2_ddeg((int32_t)sky_isqrt(
					((uint64_t)((int64_t)g[0] * g[0])) +
					((uint64_t)((int64_t)g[2] * g[2]))),
					-g[1]);

		out->roll_ddeg = (int16_t)((roll > 1800) ? (roll - 3600) : roll);
		out->pitch_ddeg = (int16_t)((pitch > 1800) ? (pitch - 3600) : pitch);
	}

	/*
	 * The heading, derived geometrically rather than from an axis convention.
	 *
	 * Reported heading is the compass bearing of the board's **+z axis** — the
	 * front-panel normal, i.e. the direction the panel faces. That is the only
	 * choice that means anything for this product: the board is mounted
	 * vertically (§6.3), so +y is nearly vertical and has no useful horizontal
	 * projection, while +z is horizontal and is what an operator standing in
	 * front of the unit is looking along.
	 *
	 * Construction, using only dot and cross products so that no axis-order or
	 * sign convention has to be argued about:
	 *
	 *   g       gravity, from the accelerometer (points up, by convention)
	 *   mh      the magnetic vector with its vertical component removed
	 *           -> horizontal magnetic north
	 *   fh      the +z axis with its vertical component removed
	 *           -> where the panel faces, in the horizontal plane
	 *   heading = signed angle from mh to fh, measured clockwise about g
	 *
	 * Both projections are scaled by |g|^2 rather than normalised, which keeps
	 * everything in integers; the common factor cancels in the angle.
	 */
	mdotg = ((int64_t)m[0] * g[0]) + ((int64_t)m[1] * g[1]) +
		((int64_t)m[2] * g[2]);
	/* fh = z - (z.g)g, and z = (0, 0, 1), so z.g is simply g[2]. */
	fdotg = (int64_t)g[2];

	for (i = 0U; i < 3U; i++) {
		int64_t gg = (int64_t)g[i];

		mh[i] = ((int64_t)m[i] * (int64_t)acc_mag * (int64_t)acc_mag) -
			(mdotg * gg);
		fh[i] = -(fdotg * gg);
	}
	fh[2] += (int64_t)acc_mag * (int64_t)acc_mag;

	/*
	 * Scale both projections down into a range where the cross product cannot
	 * overflow int64. |g|^2 can be ~1e6 and m ~1e3, so mh can reach ~1e9;
	 * squaring that in the cross product would be ~1e18, which is within
	 * int64 but leaves no headroom for the subsequent multiply by g.
	 */
	for (i = 0U; i < 3U; i++) {
		mh[i] /= (int64_t)acc_mag;
		fh[i] /= (int64_t)acc_mag;
	}

	mh_len = sky_isqrt((uint64_t)((mh[0] * mh[0]) + (mh[1] * mh[1]) +
				      (mh[2] * mh[2])));
	fh_len = sky_isqrt((uint64_t)((fh[0] * fh[0]) + (fh[1] * fh[1]) +
				      (fh[2] * fh[2])));

	if ((mh_len == 0U) || (fh_len == 0U)) {
		/*
		 * Either the field is exactly vertical (at the magnetic pole) or the
		 * panel normal is exactly vertical (the board is lying flat). In both
		 * cases there is no horizontal reference and §6.3's GNSS-north
		 * fallback is the honest answer.
		 */
		return 0;
	}

	/*
	 * The panel normal must have a real horizontal projection, not a residue
	 * of rounding: below a quarter of its length the bearing is dominated by
	 * noise. This is what makes a near-flat board report "north unverified"
	 * instead of a confident random number.
	 */
	if ((uint64_t)fh_len * 4U < (uint64_t)acc_mag) {
		return 0;
	}

	dot = (mh[0] * fh[0]) + (mh[1] * fh[1]) + (mh[2] * fh[2]);

	cross[0] = (mh[1] * fh[2]) - (mh[2] * fh[1]);
	cross[1] = (mh[2] * fh[0]) - (mh[0] * fh[2]);
	cross[2] = (mh[0] * fh[1]) - (mh[1] * fh[0]);
	sinterm = ((cross[0] * g[0]) + (cross[1] * g[1]) + (cross[2] * g[2])) /
		  (int64_t)acc_mag;

	/*
	 * Scale both terms into int32 before the bearing. Only their ratio
	 * matters, so a common right shift is free of consequence.
	 */
	{
		int64_t a = (dot < 0) ? -dot : dot;
		int64_t b = (sinterm < 0) ? -sinterm : sinterm;
		int64_t big = (a > b) ? a : b;
		int shift = 0;

		while ((big >> shift) > 0x3FFFFFFF) {
			shift++;
		}
		/*
		 * Negated: the cross product gives the angle from magnetic north to
		 * the panel normal measured one way about gravity, and a compass
		 * bearing runs the other way.
		 */
		out->heading_ddeg = (int16_t)sky_atan2_ddeg(
			(int32_t)(dot >> shift), (int32_t)(-sinterm >> shift));
	}

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
	if (mag == 0U) {
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
		/*
		 * sky_atan2_ddeg() is compass-convention: the first argument is the
		 * north component, the second the east component. Here `x` holds the
		 * north term of the great-circle bearing and `y` the east term.
		 */
		int32_t b = sky_atan2_ddeg((int32_t)x, (int32_t)y);

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

	/* The 30/60-degree elevation rings. */
	ring(c, cx, cy, (r * 2) / 3, (uint8_t)SKY_C_GRID);
	ring(c, cx, cy, r / 3, (uint8_t)SKY_C_GRID);

	/*
	 * Cardinal ticks. They rotate with the plot, so when north is verified the
	 * north tick genuinely points at true north, and when it is not the ticks
	 * are GNSS-north and the badge says so. North gets a longer, brighter tick
	 * than the other three — on a rotated plot the operator needs to find north
	 * at a glance, and "the long one" is faster to read than a letter.
	 */
	tick(c, cx, cy, r, rotation, r / 8, (uint8_t)SKY_C_GRID_MAJOR);
	tick(c, cx, cy, r, rotation + 900, r / 12, (uint8_t)SKY_C_GRID);
	tick(c, cx, cy, r, rotation + 1800, r / 12, (uint8_t)SKY_C_GRID);
	tick(c, cx, cy, r, rotation + 2700, r / 12, (uint8_t)SKY_C_GRID);

	/*
	 * The horizon (elevation 0) goes on LAST of the grid elements so the ticks
	 * cannot punch holes in it. A broken horizon ring reads as a rendering
	 * fault to anyone looking at the panel, and the ring is the one line that
	 * tells them where the sky ends.
	 */
	ring(c, cx, cy, r, (uint8_t)SKY_C_GRID_MAJOR);

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
