/*
 * STS1000 "Meridian" — the skyplot glue's decisions, as pure logic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/ui/, and deliberately free of every Zephyr, cfg and
 * logging dependency so tests/host can compile it — the same arrangement as
 * console/sts_confirm_gate.h and net/sts_ntp_keys.h.
 *
 * ---------------------------------------------------------------------------
 * What was wrong
 * ---------------------------------------------------------------------------
 *
 * core/ui/skyplot.c is a complete, golden-image-tested polar renderer, and
 * core/ui emits UI_HINT_SKYPLOT with the cell rectangle the plot belongs in.
 * Nothing consumed the hint. ui_display.c rasterised BIGNUM, PROGRESS and RULE
 * and silently ignored SKYPLOT, so the sky page drew its summary lines over an
 * empty rectangle, and `--gc-sections` discarded all twenty-one of skyplot.c's
 * symbols from the image — the linker agreeing that the feature did not exist.
 *
 * This header owns the three decisions between the hint and the pixels, all of
 * which are silent when they are wrong:
 *
 *   LAYOUT       where on the panel the square canvas goes, and how big it can
 *                be. Get this wrong by a row and the plot either overhangs the
 *                summary text or is clipped by the blitter — and an oversized
 *                canvas is an out-of-bounds write into the framebuffer, which
 *                is why ui.h documents hints as validated rather than clipped.
 *
 *   SV SELECTION which UBX-NAV-SAT records become markers, and how gnssId maps
 *                onto the constellation colours. UBX reports azimuth 0 and
 *                elevation 0 for a satellite it has not measured, so passing
 *                every record through paints a fan of phantom markers at due
 *                north on the horizon — exactly where an operator looks for
 *                obstruction.
 *
 *   NORTH        whether the plot may be rotated to true north at all. §6.3 is
 *                explicit that a plausible-but-wrong rotation is worse than an
 *                honest un-rotated one, so this returns a REASON and not just a
 *                boolean, and the glue prints it over the badge band.
 */

#ifndef STS1000_ZEPHYR_UI_STS_SKY_POLICY_H_
#define STS1000_ZEPHYR_UI_STS_SKY_POLICY_H_

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "ui/skyplot.h"
#include "ui/ui.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================== *
 *  Layout
 * ===================================================================== */

/**
 * Smallest canvas worth drawing, pixels.
 *
 * Below this the 30/60-degree rings collide with the horizon ring and the
 * 2..5-pixel markers merge into the grid, so the plot stops carrying
 * information. A band that cannot hold this much is refused and the glue
 * leaves the cells blank rather than painting a smudge.
 */
#define STS_SKY_MIN_SIDE 32u

/** Where the indexed canvas lands on the panel. */
typedef struct {
	uint16_t x0;   /**< left edge, pixels from the panel origin */
	uint16_t y0;   /**< top edge, pixels */
	uint16_t side; /**< the plot is square; this is both width and height */
} sts_sky_layout_t;

/**
 * Place the square canvas inside a UI_HINT_SKYPLOT band.
 *
 * The hint's `len` is a ROW count, not a column count (ui.h): the band is rows
 * `row .. row+len-1`, columns `col .. cols-1`. The canvas is the largest square
 * that fits in that band and in @p max_side, centred in it.
 *
 * @param h         The hint, as core/ui emitted it.
 * @param rows      Surface rows.
 * @param cols      Surface columns.
 * @param cell_w    Glyph width, pixels (8 for the as-built font).
 * @param cell_h    Glyph height, pixels (16).
 * @param max_side  Canvas storage the glue has, as a side length.
 * @param out       Receives the placement.
 *
 * @retval 0        Placed.
 * @retval -EINVAL  NULL argument, a zero cell metric, or a hint of another kind.
 * @retval -EDOM    The hint does not fit the grid, or the band cannot hold
 *                  STS_SKY_MIN_SIDE pixels.
 */
static inline int sts_sky_layout(const ui_hint_t *h, uint8_t rows, uint8_t cols,
				 uint8_t cell_w, uint8_t cell_h,
				 uint16_t max_side, sts_sky_layout_t *out)
{
	uint32_t band_w;
	uint32_t band_h;
	uint32_t side;

	if ((h == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if ((cell_w == 0u) || (cell_h == 0u) || (rows == 0u) || (cols == 0u)) {
		return -EINVAL;
	}
	if (h->kind != (uint8_t)UI_HINT_SKYPLOT) {
		return -EINVAL;
	}
	if ((h->len == 0u) || (h->row >= rows) || (h->col >= cols)) {
		return -EDOM;
	}
	if (((uint32_t)h->row + (uint32_t)h->len) > (uint32_t)rows) {
		return -EDOM;
	}

	band_w = (uint32_t)(cols - h->col) * (uint32_t)cell_w;
	band_h = (uint32_t)h->len * (uint32_t)cell_h;

	side = (band_w < band_h) ? band_w : band_h;
	if ((uint32_t)max_side < side) {
		side = (uint32_t)max_side;
	}
	if (side > (uint32_t)SKY_MAX_DIM) {
		side = (uint32_t)SKY_MAX_DIM;
	}
	if (side < (uint32_t)STS_SKY_MIN_SIDE) {
		return -EDOM;
	}

	out->side = (uint16_t)side;
	out->x0 = (uint16_t)(((uint32_t)h->col * (uint32_t)cell_w) +
			     ((band_w - side) / 2u));
	out->y0 = (uint16_t)(((uint32_t)h->row * (uint32_t)cell_h) +
			     ((band_h - side) / 2u));
	return 0;
}

/* ===================================================================== *
 *  Satellite selection
 * ===================================================================== */

/**
 * UBX gnssId -> ui_gnss_sys_t.
 *
 * u-blox numbering (interface description, UBX-NAV-SAT): 0 GPS, 1 SBAS,
 * 2 Galileo, 3 BeiDou, 4 IMES, 5 QZSS, 6 GLONASS. Anything else — including
 * IMES, which this receiver does not track — is UI_GNSS_OTHER rather than a
 * guess, so an unexpected id shows as a distinct colour instead of silently
 * joining GPS.
 */
static inline uint8_t sts_sky_sys_from_gnss_id(uint8_t gnss_id)
{
	switch (gnss_id) {
	case 0u:
		return (uint8_t)UI_GNSS_GPS;
	case 1u:
		return (uint8_t)UI_GNSS_SBAS;
	case 2u:
		return (uint8_t)UI_GNSS_GALILEO;
	case 3u:
		return (uint8_t)UI_GNSS_BEIDOU;
	case 5u:
		return (uint8_t)UI_GNSS_QZSS;
	case 6u:
		return (uint8_t)UI_GNSS_GLONASS;
	default:
		return (uint8_t)UI_GNSS_OTHER;
	}
}

/**
 * Convert one UBX-NAV-SAT record into a plot marker.
 *
 * @return true when the record should be drawn.
 *
 * Rejected: a record the receiver is not actually measuring (C/N0 zero and not
 * in the solution). UBX reports azimuth and elevation as zero for a satellite
 * it only knows from the almanac, so admitting those paints a stack of phantom
 * markers at due north on the horizon. A satellite that IS in the timing
 * solution is always admitted regardless of its reported C/N0, because being
 * used is stronger evidence than a momentary zero.
 *
 * Also normalised here: azimuth 360, which UBX may report and which
 * sky_place() rejects as out of range (0..359), folds to 0. Elevation below
 * the horizon is left alone — sky_render() drops it, and dropping it twice
 * would hide a receiver reporting nonsense.
 */
static inline bool sts_sky_sv_from_ubx(uint8_t gnss_id, uint8_t sv_id,
				       int8_t elev_deg, int16_t azim_deg,
				       uint8_t cno_dbhz, bool used, ui_sv_t *out)
{
	int32_t az;

	if (out == NULL) {
		return false;
	}
	if ((cno_dbhz == 0u) && !used) {
		return false;
	}

	az = (int32_t)azim_deg % 360;
	if (az < 0) {
		az += 360;
	}

	(void)memset(out, 0, sizeof(*out));
	out->sys = sts_sky_sys_from_gnss_id(gnss_id);
	out->svid = sv_id;
	out->elev_deg = elev_deg;
	out->azim_deg = (int16_t)az;
	out->cno_dbhz = cno_dbhz;
	out->used = used;
	return true;
}

/* ===================================================================== *
 *  True north
 * ===================================================================== */

/**
 * The sentinel `cal.decl.ddeg` carries when no site declination has been
 * commissioned. Out of the +-1800 range a real declination occupies, so it
 * cannot collide with one.
 */
#define STS_SKY_DECL_UNSET 32767

/** Why the plot is, or is not, rotated to true north. */
typedef enum {
	/** Rotated to true north from a commissioned site declination. */
	STS_SKY_NORTH_TRUE = 0,
	/** Rotated, but with a modelled declination good only to 10-15 deg. */
	STS_SKY_NORTH_MODELLED,
	/** No e-compass reading this frame. */
	STS_SKY_NORTH_NO_SAMPLE,
	/** The §10.4 hard/soft-iron fit has never been performed. */
	STS_SKY_NORTH_UNCALIBRATED,
	/** Calibrated, but the reading is not believable (field or tilt). */
	STS_SKY_NORTH_DISTURBED,
	/** Heading good, but nothing says what the declination is here. */
	STS_SKY_NORTH_NO_DECLINATION,
	STS_SKY_NORTH_COUNT,
} sts_sky_north_t;

/** True when @p reason means the plot is drawn in GNSS-north with the badge. */
static inline bool sts_sky_north_unverified(uint8_t reason)
{
	return (reason != (uint8_t)STS_SKY_NORTH_TRUE) &&
	       (reason != (uint8_t)STS_SKY_NORTH_MODELLED);
}

/**
 * The words the glue writes over the badge band, or over the plot's caption
 * when north IS verified. Never NULL, never longer than 24 characters.
 */
static inline const char *sts_sky_north_label(uint8_t reason)
{
	switch (reason) {
	case (uint8_t)STS_SKY_NORTH_TRUE:
		return "TRUE NORTH";
	case (uint8_t)STS_SKY_NORTH_MODELLED:
		return "NORTH APPROX (MODEL)";
	case (uint8_t)STS_SKY_NORTH_NO_SAMPLE:
		return "NORTH UNVERIFIED: NO MAG";
	case (uint8_t)STS_SKY_NORTH_UNCALIBRATED:
		return "NORTH UNVERIFIED: UNCAL";
	case (uint8_t)STS_SKY_NORTH_DISTURBED:
		return "NORTH UNVERIFIED: FIELD";
	case (uint8_t)STS_SKY_NORTH_NO_DECLINATION:
		return "NORTH UNVERIFIED: NO DEC";
	default:
		return "NORTH UNVERIFIED";
	}
}

/** Everything the orientation decision reads, in one place. */
typedef struct {
	/** Both sensors were read successfully this sweep. */
	bool sample_valid;
	int32_t mag[3]; /**< milligauss, board axes */
	int32_t acc[3]; /**< milli-g, board axes */

	/** `cal.mag.ref` is non-zero, i.e. the §10.4 fit has been stored. */
	bool cal_valid;
	int32_t mag_offset[3]; /**< `cal.mag.off.*`, milligauss */
	int32_t mag_scale[3];  /**< `cal.mag.scl.*`, Q12 */
	uint32_t field_ref;    /**< `cal.mag.ref`, milligauss */

	/** `cal.decl.ddeg` is not STS_SKY_DECL_UNSET. */
	bool decl_site_valid;
	int16_t decl_site_ddeg;

	/** A NAV-PVT fix is available for the dipole fallback. */
	bool pos_valid;
	int32_t lat_1e7;
	int32_t lon_1e7;
} sts_sky_orient_in_t;

/**
 * Decide how to orient one frame.
 *
 * Always writes @p out. When the reason is anything sts_sky_north_unverified()
 * accepts, @p out is the GNSS-north orientation — north_valid false, which is
 * what makes sky_render() draw the badge.
 *
 * The declination rule is the part worth stating plainly: a heading with no
 * declination is MAGNETIC north, which at this product's latitudes is up to
 * ~20 degrees from true north. §6.3 asks for true north or an honest fallback,
 * and there is no third option in sky_orient_t — declination_modelled marks a
 * rotation approximate, not unknown. So a believed heading with no declination
 * source at all is reported NO_DECLINATION and the plot stays un-rotated. In
 * service that state does not arise: a fixed-site grandmaster has a fix, so the
 * dipole fallback is always available even before the site constant is entered.
 *
 * @param heading_out  Optional; receives the raw heading computation for
 *                     telemetry (pitch, roll, field magnitude). May be NULL.
 * @return An sts_sky_north_t.
 */
static inline uint8_t sts_sky_orient(const sts_sky_orient_in_t *in,
				     sky_orient_t *out,
				     sky_heading_t *heading_out)
{
	sky_ecompass_t e;
	sky_heading_t hd;
	int16_t decl = 0;
	bool modelled = false;

	if (out == NULL) {
		return (uint8_t)STS_SKY_NORTH_NO_SAMPLE;
	}
	(void)memset(out, 0, sizeof(*out));
	(void)memset(&hd, 0, sizeof(hd));

	if (in == NULL || !in->sample_valid) {
		if (heading_out != NULL) {
			*heading_out = hd;
		}
		return (uint8_t)STS_SKY_NORTH_NO_SAMPLE;
	}

	(void)memset(&e, 0, sizeof(e));
	e.mag[0] = in->mag[0];
	e.mag[1] = in->mag[1];
	e.mag[2] = in->mag[2];
	e.acc[0] = in->acc[0];
	e.acc[1] = in->acc[1];
	e.acc[2] = in->acc[2];
	e.mag_offset[0] = in->mag_offset[0];
	e.mag_offset[1] = in->mag_offset[1];
	e.mag_offset[2] = in->mag_offset[2];
	e.mag_scale[0] = in->mag_scale[0];
	e.mag_scale[1] = in->mag_scale[1];
	e.mag_scale[2] = in->mag_scale[2];
	e.calibrated = in->cal_valid;

	(void)sky_heading_from_ecompass(&e, in->cal_valid ? in->field_ref : 0u,
					&hd);
	if (heading_out != NULL) {
		*heading_out = hd;
	}
	out->tilt_valid = hd.tilt_valid;

	if (!hd.valid) {
		/*
		 * Two distinguishable causes, and they call for different
		 * operator actions: "run the compass calibration" versus "there
		 * is something ferrous or energised next to the box".
		 */
		return in->cal_valid ? (uint8_t)STS_SKY_NORTH_DISTURBED
				     : (uint8_t)STS_SKY_NORTH_UNCALIBRATED;
	}

	if (in->decl_site_valid) {
		decl = in->decl_site_ddeg;
	} else if (in->pos_valid &&
		   (sky_declination_dipole_ddeg(in->lat_1e7, in->lon_1e7,
						&decl) == 0)) {
		modelled = true;
	} else {
		return (uint8_t)STS_SKY_NORTH_NO_DECLINATION;
	}

	out->north_valid = true;
	out->heading_ddeg = hd.heading_ddeg;
	out->declination_ddeg = decl;
	out->declination_modelled = modelled;
	return modelled ? (uint8_t)STS_SKY_NORTH_MODELLED
			: (uint8_t)STS_SKY_NORTH_TRUE;
}

/* ===================================================================== *
 *  Repaint gate
 * ===================================================================== */

/**
 * Rows a UI_HINT_SKYPLOT claims, as a bitmask over the surface's rows.
 *
 * The glue force-repaints a text row whose cells changed, and a repainted row
 * is painted black before its glyphs go down — so a repaint anywhere inside the
 * band destroys part of the plot. Intersecting this with the frame's dirty-row
 * mask is what tells the glue the picture on the panel is no longer the picture
 * it last drew, independently of whether the sky itself moved.
 *
 * Rows at or past @p rows are dropped rather than shifted out of a 32-bit word:
 * `1u << 32` is undefined, and a hint that overhangs the grid must not be able
 * to turn into a mask of everything.
 */
static inline uint32_t sts_sky_band_mask(const ui_hint_t *h, uint8_t rows)
{
	uint32_t mask = 0u;
	unsigned int r;
	unsigned int end;

	if ((h == NULL) || (h->kind != (uint8_t)UI_HINT_SKYPLOT)) {
		return 0u;
	}
	end = (unsigned int)h->row + (unsigned int)h->len;
	for (r = h->row; (r < end) && (r < rows) && (r < 32u); r++) {
		mask |= (uint32_t)1u << r;
	}
	return mask;
}

/**
 * Signature of everything a rendered skyplot depends on.
 *
 * The plot is ~50 KiB of RGB565 at the as-built geometry. Pushing it at the
 * 10 Hz render tick would be a megabyte a second of SPI to redraw a picture
 * whose inputs — UBX-NAV-SAT and the 1 Hz e-compass sweep — move at 1 Hz, so
 * the glue repaints only when this value changes.
 *
 * Field by field rather than a hash over the structs: ui_sv_t, sky_orient_t and
 * sts_sky_layout_t all carry compiler padding, and hashing indeterminate bytes
 * would make the gate fire at random — which looks exactly like the bug it is
 * meant to prevent, only intermittently.
 *
 * It is a change detector, not a checksum: FNV-1a over ~10 words, chosen for
 * being cheap and having no collisions that a satellite moving one degree could
 * hit. A missed change costs one stale frame, not a wrong one.
 *
 * @param lay     Placement; NULL contributes nothing, so a caller that has not
 *                placed the canvas yet still gets a well-defined value.
 * @param sv      Markers; may be NULL when @p n is 0.
 * @param n       Entries in @p sv, clamped to UI_MAX_SV.
 * @param o       Orientation; NULL is GNSS-north.
 * @param reason  An sts_sky_north_t, because the label is part of the picture.
 */
static inline uint32_t sts_sky_repaint_sig(const sts_sky_layout_t *lay,
					   const ui_sv_t *sv, uint8_t n,
					   const sky_orient_t *o,
					   uint8_t reason)
{
	uint32_t h = 2166136261u;
	uint8_t count = (n > (uint8_t)UI_MAX_SV) ? (uint8_t)UI_MAX_SV : n;
	uint8_t i;

	/* FNV-1a over one 32-bit word, least-significant octet first. */
#define STS_SKY_SIG_MIX(acc, v)                                                \
	do {                                                                   \
		uint32_t v_ = (uint32_t)(v);                                   \
		unsigned int b_;                                               \
		for (b_ = 0u; b_ < 4u; b_++) {                                 \
			(acc) ^= (v_ >> (8u * b_)) & 0xFFu;                    \
			(acc) *= 16777619u;                                    \
		}                                                              \
	} while (0)

	STS_SKY_SIG_MIX(h, (lay != NULL) ? lay->x0 : 0u);
	STS_SKY_SIG_MIX(h, (lay != NULL) ? lay->y0 : 0u);
	STS_SKY_SIG_MIX(h, (lay != NULL) ? lay->side : 0u);
	STS_SKY_SIG_MIX(h, reason);
	STS_SKY_SIG_MIX(h, ((o != NULL) && o->north_valid) ? 1u : 0u);
	STS_SKY_SIG_MIX(h, (o != NULL) ? (uint32_t)(uint16_t)o->heading_ddeg
				       : 0u);
	STS_SKY_SIG_MIX(h, (o != NULL) ? (uint32_t)(uint16_t)o->declination_ddeg
				       : 0u);
	STS_SKY_SIG_MIX(h, ((o != NULL) && o->declination_modelled) ? 1u : 0u);
	STS_SKY_SIG_MIX(h, ((o != NULL) && o->tilt_valid) ? 1u : 0u);
	STS_SKY_SIG_MIX(h, count);

	for (i = 0u; (sv != NULL) && (i < count); i++) {
		STS_SKY_SIG_MIX(h, ((uint32_t)sv[i].sys << 24) |
					   ((uint32_t)sv[i].svid << 16) |
					   ((uint32_t)(uint8_t)sv[i].elev_deg
					    << 8) |
					   (uint32_t)sv[i].cno_dbhz);
		STS_SKY_SIG_MIX(h, ((uint32_t)(uint16_t)sv[i].azim_deg << 1) |
					   (sv[i].used ? 1u : 0u));
	}

#undef STS_SKY_SIG_MIX
	return h;
}

/* ===================================================================== *
 *  Palette
 * ===================================================================== */

/** RGB565, host order. ui_display.c byte-swaps for the ST7796's 8-bit DBI. */
#define STS_SKY_RGB565(r, g, b)                                                \
	((uint16_t)((((r) & 0xF8u) << 8) | (((g) & 0xFCu) << 3) | ((b) >> 3)))

/**
 * Map a sky_colour_t palette index to RGB565.
 *
 * The constellation colours are chosen to stay distinguishable at the 2..5
 * pixel marker sizes and on a panel viewed off-axis, which rules out pairs
 * separated only by brightness. Out-of-range indices come back as the
 * background rather than as an arbitrary colour, so a palette that grows
 * without this table growing shows as missing pixels, not as wrong ones.
 */
static inline uint16_t sts_sky_palette_rgb565(uint8_t colour)
{
	switch (colour) {
	case (uint8_t)SKY_C_BG:
		return STS_SKY_RGB565(0x00, 0x00, 0x00);
	case (uint8_t)SKY_C_GRID:
		return STS_SKY_RGB565(0x38, 0x40, 0x48);
	case (uint8_t)SKY_C_GRID_MAJOR:
		return STS_SKY_RGB565(0x90, 0x98, 0xA0);
	case (uint8_t)SKY_C_GPS:
		return STS_SKY_RGB565(0x30, 0xC0, 0x40); /* green */
	case (uint8_t)SKY_C_GALILEO:
		return STS_SKY_RGB565(0x40, 0x80, 0xF0); /* blue */
	case (uint8_t)SKY_C_GLONASS:
		return STS_SKY_RGB565(0xE0, 0x40, 0x40); /* red */
	case (uint8_t)SKY_C_BEIDOU:
		return STS_SKY_RGB565(0xF0, 0xC0, 0x20); /* amber */
	case (uint8_t)SKY_C_SBAS:
		return STS_SKY_RGB565(0xD0, 0x50, 0xD0); /* magenta */
	case (uint8_t)SKY_C_QZSS:
		return STS_SKY_RGB565(0x30, 0xC8, 0xD0); /* cyan */
	case (uint8_t)SKY_C_OTHER:
		return STS_SKY_RGB565(0xC0, 0xC0, 0xC0); /* grey */
	case (uint8_t)SKY_C_BADGE:
		return STS_SKY_RGB565(0xF0, 0x90, 0x10); /* warning amber */
	default:
		return STS_SKY_RGB565(0x00, 0x00, 0x00);
	}
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_UI_STS_SKY_POLICY_H_ */
