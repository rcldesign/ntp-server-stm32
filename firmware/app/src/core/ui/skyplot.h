/*
 * STS1000 "Meridian" — core/ui: polar skyplot renderer.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No allocation, no libm, no platform headers. Dependency
 * edge: ui (for ui_sv_t) and util only.
 *
 * Spec reference: ntp_server_software_spec.md §6.3 —
 *
 *   "Polar plot, zenith center, horizon edge; per-SV marker placed by az/el from
 *    UBX-NAV-SAT, colored by constellation, sized/labelled by CNR, hollow =
 *    visible-not-used, filled = used in solution.
 *    True-north orientation: rotate the plot by the magnetometer heading
 *    (IIS2MDC), corrected by hard/soft-iron calibration (§10.4) and by magnetic
 *    declination computed from the GPS fix (WMM/IGRF); tilt-compensate with the
 *    accelerometer (LIS2DH12) — mandatory given the vertical board mount. If the
 *    compass is uncalibrated/disturbed, fall back to 'GNSS-north' (az as
 *    reported) with a clear 'north unverified' badge."
 *
 * ---------------------------------------------------------------------------
 * Why an indexed-colour canvas
 * ---------------------------------------------------------------------------
 *
 * core/ui renders into a text-tile surface, which cannot express a polar plot.
 * Rather than push pixels into the glue and lose testability, this module writes
 * **palette indices** into a caller-owned byte buffer: one octet per pixel, each
 * a sky_colour_t. The glue maps indices to RGB565 when it blits.
 *
 * That choice is what makes golden-image testing practical. A test renders into
 * a small canvas and compares it against an ASCII picture where each character
 * is a palette index — human-readable, diffable in a failure message, and
 * independent of the display's pixel format. sky_canvas_to_ascii() produces
 * exactly that.
 *
 * ---------------------------------------------------------------------------
 * Geometry
 * ---------------------------------------------------------------------------
 *
 * Zenith at the centre, horizon at the rim, radius linear in (90 - elevation):
 *
 *   r = R * (90 - el) / 90        el in degrees, 0 at the horizon
 *   screen_az = az + rotation     rotation applied to bring true north up
 *   x = cx + r * sin(screen_az)
 *   y = cy - r * cos(screen_az)   screen y grows downward, so north (az 0) is up
 *
 * Linear-in-elevation (rather than the stereographic projection a receiver
 * datasheet might use) is deliberate: on a 240-pixel disc an operator is reading
 * "is that satellite low" and linear radius makes the horizon band the widest
 * part of the plot, which is where multipath and obstruction live.
 *
 * ---------------------------------------------------------------------------
 * True north, and being honest about it
 * ---------------------------------------------------------------------------
 *
 * The plot is rotated by  -(heading + declination)  so that true north points
 * up. Both terms can be wrong in ways the firmware can detect, and §6.3 requires
 * saying so rather than quietly drawing a plausible picture:
 *
 *   heading      tilt-compensated from the IIS2MDC magnetometer and the LIS2DH12
 *                accelerometer by sky_heading_from_ecompass(). Requires a valid
 *                hard/soft-iron calibration (§10.4). Without one, or with the
 *                field magnitude outside the plausible band (a nearby motor, a
 *                steel rack), north_valid is false.
 *   declination   preferred source is the *site* constant in
 *                sky_orient_t::declination_ddeg, entered at commissioning from a
 *                WMM/IGRF calculator. A fixed-site grandmaster never moves, so
 *                this is both the most accurate and the most honest option.
 *                sky_declination_dipole_ddeg() is the automatic fallback and is
 *                a centred-dipole model: typically within a few degrees at mid
 *                latitudes, far worse near the poles and the agonic lines. When
 *                the fallback is used, declination_modelled is reported so the
 *                UI can mark north as approximate.
 *
 * With north_valid false the plot is drawn in GNSS-north (azimuths exactly as
 * the receiver reports, rotation 0) and a SKY_C_BADGE band is drawn along the
 * bottom of the canvas. The badge is a *graphical* element rather than text
 * because this module owns no font; the glue writes the words over it.
 */

#ifndef STS1000_CORE_UI_SKYPLOT_H_
#define STS1000_CORE_UI_SKYPLOT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ui/ui.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================== *
 *  Canvas
 * ===================================================================== */

/** Palette indices. The glue maps these to RGB565. */
typedef enum {
	SKY_C_BG = 0,       /**< background */
	SKY_C_GRID,         /**< elevation rings, cardinal ticks */
	SKY_C_GRID_MAJOR,   /**< horizon ring and the north tick */
	SKY_C_GPS,
	SKY_C_GALILEO,
	SKY_C_GLONASS,
	SKY_C_BEIDOU,
	SKY_C_SBAS,
	SKY_C_QZSS,
	SKY_C_OTHER,
	SKY_C_BADGE,        /**< the "north unverified" band */
	SKY_C__COUNT,
} sky_colour_t;

/** Palette index for a ui_gnss_sys_t; SKY_C_OTHER for anything unexpected. */
uint8_t sky_colour_for_sys(uint8_t sys);

/** Widest and tallest canvas this module will render into. */
#define SKY_MAX_DIM 320U

/** An 8-bit indexed-colour canvas. Storage is caller-owned. */
typedef struct {
	uint8_t *px;  /**< w*h octets, row-major */
	uint16_t w;
	uint16_t h;
	size_t cap;
} sky_canvas_t;

/**
 * Bind caller-owned storage and clear to SKY_C_BG.
 *
 * @retval 0        Bound.
 * @retval -EINVAL  NULL argument, a zero or over-large dimension.
 * @retval -ENOSPC  @p cap is below w*h.
 */
int sky_canvas_init(sky_canvas_t *c, uint8_t *px, uint16_t w, uint16_t h,
		    size_t cap);

/** Fill the whole canvas with SKY_C_BG. */
void sky_canvas_clear(sky_canvas_t *c);

/** Palette index at (x, y), or SKY_C_BG when out of bounds. */
uint8_t sky_canvas_get(const sky_canvas_t *c, int32_t x, int32_t y);

/**
 * Render the canvas as one ASCII character per pixel, rows separated by '\n'.
 *
 * The golden-image accessor the host tests assert against, and a debug dump for
 * the console. Characters are '.', '-', '=', then '0'..'6' for the constellation
 * colours and '#' for the badge — see sky_ascii_char().
 *
 * @return Characters written excluding the NUL, or 0 on a bad argument.
 */
size_t sky_canvas_to_ascii(const sky_canvas_t *c, char *out, size_t cap);

/** The ASCII character sky_canvas_to_ascii() uses for a palette index. */
char sky_ascii_char(uint8_t colour);

/* ===================================================================== *
 *  Integer trigonometry (no libm anywhere in core)
 * ===================================================================== */

/** Scale of the fixed-point sine/cosine results: sin(90 deg) == 4096. */
#define SKY_TRIG_ONE 4096

/** sin(@p ddeg tenths of a degree), scaled by SKY_TRIG_ONE. Any input wraps. */
int32_t sky_sin_ddeg(int32_t ddeg);

/** cos(@p ddeg tenths of a degree), scaled by SKY_TRIG_ONE. */
int32_t sky_cos_ddeg(int32_t ddeg);

/**
 * atan2 in tenths of a degree, 0..3599, measured clockwise from +y.
 *
 * The convention is compass-like on purpose: bearing 0 is +y (north) and 900 is
 * +x (east), which is what both the heading computation and the marker placement
 * want, and avoids a sign-flip at every call site.
 *
 * Accurate to about 0.2 degrees, from the standard octant reduction plus the
 * rational approximation atan(z) ~ z*(3910 - 772*z^2/4096)/4096 in Q12.
 */
int32_t sky_atan2_ddeg(int32_t y, int32_t x);

/** Integer square root, floor. */
uint32_t sky_isqrt(uint64_t v);

/* ===================================================================== *
 *  Orientation
 * ===================================================================== */

/**
 * Raw e-compass sample, in whatever consistent units the sensors report.
 *
 * Only ratios matter, so milligauss and milli-g are as good as LSBs provided
 * each triple is self-consistent. The axis convention is the board's: +x right,
 * +y up the front panel, +z out of the board face. Because the board is mounted
 * vertically (§6.3), +z is horizontal in service, which is exactly why tilt
 * compensation is mandatory rather than optional here.
 */
typedef struct {
	int32_t mag[3];
	int32_t acc[3];
	/** Hard-iron offsets to subtract from @ref mag (§10.4 calibration). */
	int32_t mag_offset[3];
	/**
	 * Soft-iron scale per axis, Q12 (4096 = unity). Zero is treated as unity
	 * so a zeroed struct behaves as "uncalibrated but usable".
	 */
	int32_t mag_scale[3];
	/** The §10.4 calibration has been performed and stored. */
	bool calibrated;
} sky_ecompass_t;

/** Result of the heading computation. */
typedef struct {
	/** Tilt-compensated magnetic heading, tenths of a degree, 0..3599. */
	int16_t heading_ddeg;
	int16_t pitch_ddeg;
	int16_t roll_ddeg;
	bool valid;      /**< the heading may be used */
	bool tilt_valid; /**< the accelerometer gave a usable gravity vector */
	/** Field magnitude, same units as the input, for a plausibility check. */
	uint32_t field_mag;
} sky_heading_t;

/**
 * Field-magnitude band, as a fraction of the calibrated reference, within which a
 * magnetometer reading is believed.
 *
 * Earth's field is 25-65 uT depending on location. A reading well outside the
 * calibrated magnitude means something ferrous or energised is nearby, and the
 * heading it produces is wrong in a way no amount of arithmetic fixes — so it is
 * rejected and the badge goes up.
 */
#define SKY_FIELD_TOLERANCE_PCT 40U

/**
 * Compute a tilt-compensated magnetic heading.
 *
 * Applies the hard/soft-iron correction, derives pitch and roll from the
 * accelerometer, de-rotates the magnetic vector into the horizontal plane and
 * takes the bearing. All integer arithmetic.
 *
 * @param field_ref  Expected field magnitude from the calibration, or 0 to skip
 *                   the plausibility check.
 *
 * @retval 0        @p out written; check @ref sky_heading_t::valid.
 * @retval -EINVAL  NULL argument.
 */
int sky_heading_from_ecompass(const sky_ecompass_t *e, uint32_t field_ref,
			      sky_heading_t *out);

/**
 * Magnetic declination from a geodetic position, centred-dipole model.
 *
 * The automatic fallback when no site constant has been configured. Uses the
 * IGRF-14 epoch-2025 geomagnetic north pole at 80.7 N, 72.7 W. Typical error is
 * a few degrees at mid latitudes and much worse above about 60 degrees of
 * latitude or near an agonic line, which is why the result is always flagged as
 * modelled and why a commissioned site should carry a measured constant instead.
 *
 * @param lat_1e7  Latitude in 1e-7 degrees (the UBX-NAV-PVT scaling).
 * @param lon_1e7  Longitude in 1e-7 degrees.
 * @param out      Declination in tenths of a degree, east positive.
 *
 * @retval 0        @p out written.
 * @retval -EINVAL  NULL @p out, or a latitude outside +-90 degrees.
 */
int sky_declination_dipole_ddeg(int32_t lat_1e7, int32_t lon_1e7, int16_t *out);

/** How the plot is oriented for one frame. */
typedef struct {
	/**
	 * The compass heading may be used. When false the plot is drawn in
	 * GNSS-north with rotation 0 and the badge is drawn.
	 */
	bool north_valid;
	int16_t heading_ddeg;     /**< magnetic heading, tenths of a degree */
	int16_t declination_ddeg; /**< east positive, tenths of a degree */
	/**
	 * The declination came from sky_declination_dipole_ddeg() rather than
	 * from a commissioned site constant. Reported so the UI can mark north
	 * approximate; it does not by itself suppress the rotation.
	 */
	bool declination_modelled;
	bool tilt_valid;
} sky_orient_t;

/**
 * Rotation applied to azimuths, tenths of a degree, 0..3599.
 *
 * 0 when @p o is NULL or north is not valid — i.e. GNSS-north.
 */
int32_t sky_rotation_ddeg(const sky_orient_t *o);

/* ===================================================================== *
 *  Render
 * ===================================================================== */

/** Marker radii, pixels. CNR scales linearly between these. */
#define SKY_MARKER_MIN_R 2
#define SKY_MARKER_MAX_R 5
/** C/N0 range mapped onto the marker radius, dB-Hz. */
#define SKY_CNO_MIN 20U
#define SKY_CNO_MAX 50U

/**
 * Render a polar skyplot.
 *
 * Pure: the only mutated state is @p c->px. Satellites at or below the horizon
 * (elevation < 0) are skipped, as are entries with an out-of-range azimuth.
 *
 * @param c   Canvas, already bound.
 * @param sv  Satellite array; may be NULL when @p n is 0.
 * @param n   Entries in @p sv, clamped to UI_MAX_SV.
 * @param o   Orientation; NULL means GNSS-north with the badge drawn.
 *
 * @retval 0        Rendered.
 * @retval -EINVAL  @p c is NULL or unbound, or @p sv is NULL with @p n > 0.
 */
int sky_render(sky_canvas_t *c, const ui_sv_t *sv, uint8_t n,
	       const sky_orient_t *o);

/** Marker radius for a C/N0, pixels. */
int32_t sky_marker_radius(uint8_t cno_dbhz);

/**
 * Place one satellite on the canvas.
 *
 * Exposed so a test can assert placement arithmetic independently of the drawing.
 *
 * @param rotation_ddeg  As sky_rotation_ddeg().
 * @param x, y           Receive the pixel coordinates.
 *
 * @retval 0        Placed.
 * @retval -EINVAL  NULL argument.
 * @retval -ERANGE  Below the horizon or an out-of-range azimuth.
 */
int sky_place(const sky_canvas_t *c, int8_t elev_deg, int16_t azim_deg,
	      int32_t rotation_ddeg, int32_t *x, int32_t *y);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_UI_SKYPLOT_H_ */
