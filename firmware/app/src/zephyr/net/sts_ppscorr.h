/*
 * STS1000 "Meridian" — PPS <-> ETH-PTP correlation, as pure logic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/net/, and deliberately free of every Zephyr, cfg and
 * logging dependency so tests/host can compile it — the same arrangement as
 * console/sts_confirm_gate.h and net/sts_ntp_keys.h, for the same reason: what
 * it decides is invisible at runtime when it is wrong. An arithmetic slip here
 * does not crash anything; it serves a confidently wrong timestamp.
 *
 * ---------------------------------------------------------------------------
 * What is being correlated, and why it is exact
 * ---------------------------------------------------------------------------
 *
 * Two counters, both clocked from the same PLL fed by PH0 (the disciplined
 * OCXO or the rubidium, spec §2.1):
 *
 *   TIM2   32-bit, PCLK1, latches the GPS PPS edge on PA0 in hardware.
 *   MACSTSR/MACSTNR  the 1588 timestamp unit, which stamps every NTP and PTP
 *          packet this appliance serves.
 *
 * TIM2 knows *where in the second* a reference edge fell, to one count (4 ns at
 * 250 MHz), with no software in the path. It does not know *which* second — a
 * free-running counter carries no date. The receiver supplies that: UBX-TIM-TP
 * names the pulse it is about to emit by GPS week and time-of-week, gnssmgr
 * normalises that ToW onto the GPS timescale, and the discipline glue proves by
 * ToW match that the record describes the edge that was actually captured.
 *
 * Given one PPS capture and one {TIM2, PTP} pair read back-to-back with
 * interrupts locked:
 *
 *     t_true   the instant of the TAI second boundary the pulse marks
 *     t_obs    the instant the edge was observed at PA0
 *              t_obs = t_true + correction, where `correction` is the sawtooth
 *              qErr plus the calibrated antenna-cable delay — both refer the
 *              measured edge backwards in time, exactly as core/disc's §3.2
 *              conditioning does (disc.c: "Both refer the measured edge
 *              backwards in time, so both add to e")
 *     t_s      the instant of the paired read
 *     age      t_s - t_obs, MEASURED ON TIM2 (32-bit unsigned difference)
 *
 *     TAI(t_s) = pulse_tai + correction + age
 *     offset   = TAI(t_s) - PTP(t_s)
 *
 * Note what is absent: the PTP counter is never propagated across an interval.
 * The reference and the counter are compared at ONE instant, so the servo's own
 * rate correction and the driver's addend calibration cancel out entirely
 * instead of contributing `age * rate_error`. That is why `age` may be most of
 * a second without costing accuracy: the interval is measured on TIM2, whose
 * rate *is* the disciplined reference.
 *
 * ---------------------------------------------------------------------------
 * What it refuses, and why each refusal exists
 * ---------------------------------------------------------------------------
 *
 * The whole value of this reference is that it is three to five orders of
 * magnitude tighter than the receiver's civil time over USART3. A silently
 * wrong one is therefore far worse than none, so every input that could be
 * misread is checked rather than assumed:
 *
 *   NO_CAPTURE    no PPS edge has ever been captured.
 *   OVERCAPTURE   an edge was lost before the latched one was read, so the
 *                 TIM-TP record paired with it may name a different pulse.
 *   NO_EPOCH      no UBX-TIM-TP was positively paired with this edge, so the
 *                 second is unknown and the sawtooth cannot be applied. Spec
 *                 §3.2 makes the sawtooth correction mandatory; without it this
 *                 reference has no claim to be better than the coarse one.
 *   STALE         the capture is too old, or the TIM2-measured age and the
 *                 millisecond clock disagree — which is what a full 32-bit TIM2
 *                 wrap (17.18 s at 250 MHz) looks like from inside the
 *                 arithmetic, and it is indistinguishable from a fresh capture
 *                 without this cross-check.
 *   SKEW          the two TIM2 reads bracketing the PTP read are further apart
 *                 than a locked-interrupt register access can explain, so the
 *                 midpoint no longer names the instant the PTP counter was read.
 *   NO_COARSE /   the receiver's civil time is unavailable, or disagrees with
 *   DISAGREE      the week/ToW-derived answer. A GPS week number is the one
 *                 field here that can be silently wrong by a whole rollover
 *                 epoch (1024 weeks = 19.6 years) and still look plausible, and
 *                 the coarse UTC reference is the only independent witness on
 *                 the board. Requiring agreement costs nothing — the coarse
 *                 reference is available whenever the receiver is — and it is
 *                 what makes this a REFINEMENT of the civil-time reference
 *                 rather than a second, unwitnessed opinion. It also inherits
 *                 the coarse path's own >= 2020 sanity floor.
 *
 * Every refusal leaves the caller's existing reference table intact; the coarse
 * civil-time and realtime references remain the fallback.
 */

#ifndef STS1000_ZEPHYR_NET_STS_PPSCORR_H_
#define STS1000_ZEPHYR_NET_STS_PPSCORR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * TAI - GPS, fixed at 19 s since the GPS epoch (1980-01-06) and not a leap
 * second. Duplicated from sts_app.h's STS_TAI_MINUS_GPS_S rather than included,
 * because this header must compile with no Zephyr and no core dependency;
 * sts_ptpclk.c carries a BUILD_ASSERT that the two agree, so the copy cannot
 * drift. A 19-second error of exactly this kind has been found in this project
 * once already.
 */
#define STS_PPSCORR_TAI_MINUS_GPS_S 19

/** 1980-01-06T00:00:00Z as POSIX seconds. */
#define STS_PPSCORR_GPS_EPOCH_UNIX_S INT64_C(315964800)

/**
 * The GPS epoch expressed on the PTP timescale (TAI seconds since
 * 1970-01-01T00:00:00 TAI). TAI - UTC was 19 s on 1980-01-06, which is the same
 * 19 s as TAI - GPS: the two definitions coincide at the epoch by construction.
 */
#define STS_PPSCORR_GPS_EPOCH_TAI_S                                            \
	(STS_PPSCORR_GPS_EPOCH_UNIX_S + STS_PPSCORR_TAI_MINUS_GPS_S)

/** Seconds and milliseconds in a GPS week. */
#define STS_PPSCORR_WEEK_S  INT64_C(604800)
#define STS_PPSCORR_WEEK_MS UINT32_C(604800000)

/**
 * Largest GPS week this can place.
 *
 * Not arbitrary: ptp_clock_set() lands the second count in the MAC's 32-bit
 * MACSTSR, so a TAI second beyond UINT32_MAX cannot be represented by the
 * hardware at all. Week 6578 is the last one whose every second still fits
 * (~2106). Refusing above it keeps the int64 nanosecond arithmetic below from
 * overflowing on a garbage week, which is the failure this bound really guards:
 * 65535 * 604800 s in nanoseconds is 3.96e19, four times int64's range.
 */
#define STS_PPSCORR_MAX_WEEK 6578U

/** Minimum plausible TIM2 rate; mirrors pps.c's own floor. */
#define STS_PPSCORR_MIN_TIMER_HZ UINT32_C(1000000)

/**
 * Ceiling on the span between the two TIM2 reads that bracket the PTP read.
 *
 * Interrupts are locked across all three, so the span is three ETH register
 * reads plus two timer reads — a few hundred nanoseconds at 250 MHz. 4 us is an
 * order of magnitude of headroom for bus contention while still bounding the
 * midpoint estimator's residual at +-2 us in the worst accepted case.
 */
#define STS_PPSCORR_MAX_SKEW_NS UINT64_C(4000)

/**
 * Oldest capture still worth correlating. Two and a half seconds lets a
 * 1 Hz servo miss a tick without losing the reference, and is far short of
 * TIM2's 17.18 s wrap.
 */
#define STS_PPSCORR_MAX_AGE_NS UINT64_C(2500000000)

/**
 * How far the TIM2-measured age and the millisecond-clock age may disagree.
 *
 * Both are quantised to 1 ms and derive from the same PLL, so the real
 * disagreement is a couple of milliseconds. 100 ms is generous against that and
 * still catches the case this check exists for — a TIM2 wrap, which shows up as
 * a 17.18 s discrepancy.
 */
#define STS_PPSCORR_AGE_TOL_MS INT64_C(100)

/**
 * How far the receiver's civil time may sit from the week/ToW answer.
 *
 * The civil-time reference's own stated envelope is 50 ms (sts_ptpclk.c
 * GNSS_UNCERTAINTY_NS). 250 ms accepts that comfortably while still being half
 * a second short of the point where the two could be describing different
 * seconds.
 */
#define STS_PPSCORR_COARSE_TOL_NS INT64_C(250000000)

#define STS_PPSCORR_NS_PER_S  INT64_C(1000000000)
#define STS_PPSCORR_NS_PER_MS INT64_C(1000000)

/** Why a correlation was refused. */
typedef enum {
	STS_PPSCORR_OK = 0,
	STS_PPSCORR_NO_CAPTURE,  /**< no PPS edge has ever been captured */
	STS_PPSCORR_OVERCAPTURE, /**< an edge was lost before this one was read */
	STS_PPSCORR_NO_EPOCH,    /**< no TIM-TP paired with this edge */
	STS_PPSCORR_BAD_TIMER,   /**< implausible TIM2 rate */
	STS_PPSCORR_BAD_QSIGN,   /**< qerr_sign is not +-1 */
	STS_PPSCORR_BAD_EPOCH,   /**< week/ToW outside the representable range */
	STS_PPSCORR_SKEW,        /**< the paired read took too long */
	STS_PPSCORR_STALE,       /**< capture too old, or TIM2 vs ms clock disagree */
	STS_PPSCORR_NO_COARSE,   /**< no civil-time reference to cross-check against */
	STS_PPSCORR_DISAGREE,    /**< civil time and week/ToW name different instants */
	STS_PPSCORR__COUNT,
} sts_ppscorr_rc_t;

/** One capture, one paired read, and the conditioning that turns them into TAI. */
typedef struct {
	/* -- the PPS capture (platform area, sts_pps_epoch_t) -- */
	uint32_t seq;          /**< capture sequence; 0 = nothing captured yet */
	uint32_t tim2_cnt;     /**< TIM2_CCR1 latch of the PA0 rising edge */
	uint32_t timer_hz;     /**< TIM2 count rate, from the live RCC tree */
	uint64_t cap_mono_ms;  /**< monotonic ms latched in the capture ISR */
	uint16_t gps_week;     /**< GPS week of the pulse (UBX-TIM-TP, normalised) */
	uint32_t gps_tow_ms;   /**< GPS ToW of the pulse, ms */
	int32_t  qerr_ps;      /**< sawtooth quantisation error of this pulse */
	bool     epoch_valid;  /**< week/ToW/qErr positively paired with this edge */
	bool     overcapture;  /**< an earlier edge was lost before this one */

	/* -- the paired read, taken with interrupts locked -- */
	uint32_t tim2_a;       /**< TIM2->CNT immediately before the PTP read */
	uint32_t tim2_b;       /**< TIM2->CNT immediately after it */
	int64_t  ptp_ns;       /**< the PTP counter, TAI ns, at that instant */
	uint64_t now_mono_ms;  /**< monotonic ms around the paired read */

	/* -- conditioning and the independent witness -- */
	int8_t   qerr_sign;      /**< +1 or -1; disc_cfg_t::qerr_sign */
	int32_t  cable_delay_ns; /**< calibrated antenna-cable + board PPS delay */
	int64_t  coarse_tai_ns;  /**< the receiver's civil time as TAI ns at now_mono_ms */
	bool     coarse_valid;   /**< coarse_tai_ns is usable */
} sts_ppscorr_in_t;

/** What the correlation measured. Filled as far as evaluation got. */
typedef struct {
	int64_t  off_ns;          /**< reference - PTP counter, at the paired read */
	int64_t  pulse_tai_ns;    /**< TAI ns of the second boundary the pulse marks */
	int64_t  ref_tai_ns;      /**< TAI ns at the paired-read instant */
	int64_t  edge_age_ns;     /**< paired read - observed edge, on TIM2 */
	int64_t  coarse_resid_ns; /**< civil time - correlated reference */
	uint32_t skew_ns;         /**< measured span of the paired read */
} sts_ppscorr_out_t;

/** Counter difference to nanoseconds, rounded to nearest. Exact at 250 MHz. */
static inline uint64_t sts_ppscorr_counts_to_ns(uint32_t counts, uint32_t hz)
{
	if (hz == 0U) {
		return 0U;
	}
	/* counts * 1e9 peaks at 4.295e18, inside uint64's 1.845e19. */
	return (((uint64_t)counts * (uint64_t)1000000000U) + (uint64_t)(hz / 2U)) /
	       (uint64_t)hz;
}

/** Picoseconds to nanoseconds, rounded to nearest, symmetric about zero. */
static inline int64_t sts_ppscorr_ps_to_ns(int64_t ps)
{
	return (ps >= 0) ? ((ps + 500) / 1000) : -((-ps + 500) / 1000);
}

/** Human-readable refusal, for the log line that says why a second was skipped. */
static inline const char *sts_ppscorr_rc_name(sts_ppscorr_rc_t rc)
{
	switch (rc) {
	case STS_PPSCORR_OK:
		return "ok";
	case STS_PPSCORR_NO_CAPTURE:
		return "no-capture";
	case STS_PPSCORR_OVERCAPTURE:
		return "overcapture";
	case STS_PPSCORR_NO_EPOCH:
		return "no-epoch";
	case STS_PPSCORR_BAD_TIMER:
		return "bad-timer";
	case STS_PPSCORR_BAD_QSIGN:
		return "bad-qerr-sign";
	case STS_PPSCORR_BAD_EPOCH:
		return "bad-epoch";
	case STS_PPSCORR_SKEW:
		return "skew";
	case STS_PPSCORR_STALE:
		return "stale";
	case STS_PPSCORR_NO_COARSE:
		return "no-coarse-ref";
	case STS_PPSCORR_DISAGREE:
		return "coarse-disagrees";
	default:
		return "?";
	}
}

/**
 * Turn one PPS capture plus one paired {TIM2, PTP} read into a PTP-clock offset.
 *
 * @param in   Never NULL.
 * @param out  Never NULL. Always fully written; the fields the evaluation
 *             reached carry values, the rest are zero. On a refusal the
 *             partially-filled diagnostics (skew, age, coarse residual) are what
 *             the caller annunciates.
 * @return STS_PPSCORR_OK when @p out->off_ns is a usable measurement, else the
 *         reason the correlation was refused.
 */
static inline sts_ppscorr_rc_t sts_ppscorr_eval(const sts_ppscorr_in_t *in,
						sts_ppscorr_out_t *out)
{
	uint32_t skew_counts;
	uint32_t age_counts;
	uint32_t mid;
	uint64_t skew_ns;
	uint64_t age_ns;
	int64_t age_ms;
	int64_t mono_age_ms;
	int64_t pulse_tai_ns;
	int64_t ref_ns;
	int64_t resid;

	memset(out, 0, sizeof(*out));

	if (in->timer_hz < STS_PPSCORR_MIN_TIMER_HZ) {
		return STS_PPSCORR_BAD_TIMER;
	}
	if (in->qerr_sign != 1 && in->qerr_sign != -1) {
		return STS_PPSCORR_BAD_QSIGN;
	}
	if (in->seq == 0U) {
		return STS_PPSCORR_NO_CAPTURE;
	}
	/*
	 * Overcapture before epoch_valid on purpose: a lost edge means the
	 * capture register may hold a pulse the paired TIM-TP does not describe,
	 * and "the ToW matched" cannot detect that — the pairing is done against
	 * the capture's own timestamp, which is the one that survived.
	 */
	if (in->overcapture) {
		return STS_PPSCORR_OVERCAPTURE;
	}
	if (!in->epoch_valid) {
		return STS_PPSCORR_NO_EPOCH;
	}
	if (in->gps_tow_ms >= STS_PPSCORR_WEEK_MS ||
	    (uint32_t)in->gps_week > STS_PPSCORR_MAX_WEEK) {
		return STS_PPSCORR_BAD_EPOCH;
	}

	/*
	 * The paired read. tim2_b - tim2_a is an unsigned 32-bit difference, so
	 * a counter wrap between the two reads costs nothing; a nonsensical span
	 * (the reads swapped, or a stall) shows up as a huge value and is
	 * rejected by the same bound.
	 */
	skew_counts = in->tim2_b - in->tim2_a;
	skew_ns = sts_ppscorr_counts_to_ns(skew_counts, in->timer_hz);
	if (skew_ns > STS_PPSCORR_MAX_SKEW_NS) {
		return STS_PPSCORR_SKEW;
	}
	out->skew_ns = (uint32_t)skew_ns;

	/*
	 * The PTP read sits between the two TIM2 reads, so the midpoint is the
	 * best available estimate of the instant it names. The residual is
	 * bounded by half the span above, and it is a bias rather than jitter —
	 * it moves the served time, it does not destabilise it.
	 */
	mid = in->tim2_a + (skew_counts / 2U);
	age_counts = mid - in->tim2_cnt;
	age_ns = sts_ppscorr_counts_to_ns(age_counts, in->timer_hz);
	if (age_ns > STS_PPSCORR_MAX_AGE_NS) {
		return STS_PPSCORR_STALE;
	}

	if (in->now_mono_ms < in->cap_mono_ms) {
		return STS_PPSCORR_STALE;
	}
	mono_age_ms = (int64_t)(in->now_mono_ms - in->cap_mono_ms);
	age_ms = (int64_t)(age_ns / (uint64_t)STS_PPSCORR_NS_PER_MS);
	if ((age_ms - mono_age_ms) > STS_PPSCORR_AGE_TOL_MS ||
	    (mono_age_ms - age_ms) > STS_PPSCORR_AGE_TOL_MS) {
		return STS_PPSCORR_STALE;
	}
	out->edge_age_ns = (int64_t)age_ns;

	/* TAI = GPS + 19 s exactly; GPS carries no leap seconds, so the second
	 * this names does not depend on the receiver's leap bookkeeping. */
	pulse_tai_ns = (STS_PPSCORR_GPS_EPOCH_TAI_S +
			(int64_t)in->gps_week * STS_PPSCORR_WEEK_S +
			(int64_t)(in->gps_tow_ms / 1000U)) *
			       STS_PPSCORR_NS_PER_S +
		       (int64_t)(in->gps_tow_ms % 1000U) * STS_PPSCORR_NS_PER_MS;
	out->pulse_tai_ns = pulse_tai_ns;

	/* §3.2 conditioning, same signs as core/disc: both terms say the observed
	 * edge lagged the true instant, so both advance the reference. */
	ref_ns = pulse_tai_ns +
		 sts_ppscorr_ps_to_ns((int64_t)in->qerr_sign * (int64_t)in->qerr_ps) +
		 (int64_t)in->cable_delay_ns + (int64_t)age_ns;
	out->ref_tai_ns = ref_ns;
	out->off_ns = ref_ns - in->ptp_ns;

	if (!in->coarse_valid) {
		return STS_PPSCORR_NO_COARSE;
	}
	resid = in->coarse_tai_ns - ref_ns;
	out->coarse_resid_ns = resid;
	if (resid > STS_PPSCORR_COARSE_TOL_NS ||
	    resid < -STS_PPSCORR_COARSE_TOL_NS) {
		return STS_PPSCORR_DISAGREE;
	}

	return STS_PPSCORR_OK;
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_NET_STS_PPSCORR_H_ */
