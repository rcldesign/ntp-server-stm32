/*
 * STS1000 "Meridian" — PPS <-> ETH-PTP correlation, tested away from Zephyr.
 *
 * sts_ppscorr.h is what makes the served time absolutely accurate rather than
 * merely stable: it turns a hardware-latched PPS capture plus one paired
 * {TIM2, PTP-counter} read into the offset the servo drives to zero. Every
 * defect it can carry is silent at runtime. A sign error, a 19-second timescale
 * slip, a mishandled 32-bit wrap — none of them crash anything, none of them
 * destabilise the loop, and all of them ship a grandmaster that is confidently
 * wrong. That is the whole reason the arithmetic lives in a Zephyr-free header.
 *
 * The vectors below are hand-derived, not round-trips of the implementation:
 *
 *   - the epoch anchor is 2026-07-27T00:00:00Z, worked out from the calendar
 *     (see test_gps_week_tow_to_tai_anchor) rather than from this code;
 *   - TAI - GPS = 19 s is asserted as a number, because a 19-second error of
 *     exactly this shape has already been found and fixed once in this project;
 *   - the wrap cases are built by construction (capture at 0xFFFF_FF00, sample
 *     past the wrap) rather than by letting the code choose the arithmetic.
 */

#include <string.h>

#include "unity.h"

#include "zephyr/net/sts_ppscorr.h"

/* ------------------------------------------------------------------ fixture */

/* The board's as-built timebase: PCLK1 = 250 MHz, prescaler 0 (pps.c banner). */
#define HZ 250000000U
#define NS_PER_COUNT 4U

/*
 * 2026-07-27T00:00:00Z, derived by hand in
 * test_gps_week_tow_to_tai_anchor() and reused as the fixture's pulse.
 */
#define ANCHOR_WEEK 2429U
#define ANCHOR_TOW_MS 86418000U
#define ANCHOR_TAI_S INT64_C(1785110437)
#define ANCHOR_TAI_NS (ANCHOR_TAI_S * INT64_C(1000000000))

static sts_ppscorr_in_t in;
static sts_ppscorr_out_t out;

/**
 * A nominal, everything-agrees input.
 *
 * The edge was captured at TIM2 = 0x1000_0000; the paired read happens
 * `age_counts` later and the PTP counter reads exactly the right TAI at that
 * instant, so the nominal offset is zero and every test below can state its
 * expectation as a departure from zero.
 */
static void nominal(uint32_t age_counts)
{
	memset(&in, 0, sizeof(in));
	memset(&out, 0, sizeof(out));

	in.seq = 7U;
	in.tim2_cnt = 0x10000000U;
	in.timer_hz = HZ;
	in.cap_mono_ms = 100000U;
	in.gps_week = ANCHOR_WEEK;
	in.gps_tow_ms = ANCHOR_TOW_MS;
	in.qerr_ps = 0;
	in.epoch_valid = true;
	in.overcapture = false;

	in.tim2_a = in.tim2_cnt + age_counts;
	in.tim2_b = in.tim2_a; /* a zero-span paired read: no midpoint shift */
	in.now_mono_ms = in.cap_mono_ms + (age_counts / (HZ / 1000U));

	in.qerr_sign = 1;
	in.cable_delay_ns = 0;

	in.ptp_ns = ANCHOR_TAI_NS + (int64_t)age_counts * (int64_t)NS_PER_COUNT;
	in.coarse_tai_ns = in.ptp_ns;
	in.coarse_valid = true;
}

void setUp(void)
{
	nominal(HZ / 4U); /* 250 ms after the edge */
}

void tearDown(void) {}

static sts_ppscorr_rc_t eval(void)
{
	return sts_ppscorr_eval(&in, &out);
}

/* ------------------------------------------------- the timescale conversion */

/**
 * GPS week/ToW -> TAI, against a date worked out from the calendar.
 *
 * 2026-07-27T00:00:00Z:
 *   1970-01-01 -> 2026-01-01 is 20455 days                   = 1767225600 s
 *   2026-01-01 -> 2026-07-27 is 207 days (2026 is not a leap
 *   year: 31+28+31+30+31+30 = 181 to Jul 1, plus 26)         =   17884800 s
 *   POSIX UTC                                                 = 1785110400 s
 *
 *   GPS - UTC = 18 s today, GPS epoch = POSIX 315964800, so
 *   GPS seconds = 1785110400 - 315964800 + 18                 = 1469145618
 *   week        = 1469145618 / 604800 = 2429 rem 86418        -> week 2429,
 *                                                                ToW 86418 s
 *
 *   The remainder is a cross-check in itself: GPS weeks start on Sunday and
 *   2026-07-27 is a Monday (20661 days after Thursday 1970-01-01; 20661 mod 7
 *   = 4), so the ToW must be one whole day plus the 18 s leap offset —
 *   86400 + 18. A ToW under a day here would mean the week boundary was
 *   misplaced.
 *
 *   TAI - UTC = 37 s today, so TAI since the PTP epoch        = 1785110437 s
 *
 * The last line is the one that matters: 1785110437 - 1785110400 = 37, and this
 * header reaches it as GPS + 19 without ever consulting a leap-second table.
 * GPS time carries no leap seconds, which is exactly why the correlated
 * reference can name its second without one.
 */
static void test_gps_week_tow_to_tai_anchor(void)
{
	nominal(0U);
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());

	TEST_ASSERT_EQUAL_INT64(ANCHOR_TAI_NS, out.pulse_tai_ns);

	/* The constant itself, spelled out. Off by 19 is the classic failure. */
	TEST_ASSERT_EQUAL_INT(19, STS_PPSCORR_TAI_MINUS_GPS_S);
	TEST_ASSERT_EQUAL_INT64(INT64_C(315964819), STS_PPSCORR_GPS_EPOCH_TAI_S);

	/* TAI - POSIX-UTC must come out as today's 37 s. */
	TEST_ASSERT_EQUAL_INT64(INT64_C(37),
				(out.pulse_tai_ns / INT64_C(1000000000)) -
					INT64_C(1785110400));
}

/** A sub-second ToW is carried, not truncated to whole seconds. */
static void test_sub_second_tow_is_carried(void)
{
	nominal(0U);
	in.gps_tow_ms = ANCHOR_TOW_MS + 250U;
	in.ptp_ns += INT64_C(250000000);
	in.coarse_tai_ns = in.ptp_ns;

	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());
	TEST_ASSERT_EQUAL_INT64(ANCHOR_TAI_NS + INT64_C(250000000),
				out.pulse_tai_ns);
	TEST_ASSERT_EQUAL_INT64(0, out.off_ns);
}

/** A week beyond what MACSTSR can hold is refused, not silently overflowed. */
static void test_week_beyond_the_counters_range_is_refused(void)
{
	in.gps_week = (uint16_t)(STS_PPSCORR_MAX_WEEK + 1U);
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_BAD_EPOCH, eval());

	in.gps_week = (uint16_t)STS_PPSCORR_MAX_WEEK;
	in.coarse_valid = false; /* the coarse witness cannot follow us there */
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_NO_COARSE, eval());
	/* It got far enough to do the arithmetic without overflowing int64. */
	TEST_ASSERT_TRUE(out.pulse_tai_ns > 0);
}

/** A ToW past the end of a week is not a time of week. */
static void test_tow_past_the_week_is_refused(void)
{
	in.gps_tow_ms = STS_PPSCORR_WEEK_MS;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_BAD_EPOCH, eval());
}

/* --------------------------------------------------- the offset arithmetic */

/** The nominal case: reference and counter agree, so the offset is zero. */
static void test_nominal_offset_is_zero(void)
{
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());
	TEST_ASSERT_EQUAL_INT64(0, out.off_ns);
	TEST_ASSERT_EQUAL_INT64(250000000, out.edge_age_ns);
	TEST_ASSERT_EQUAL_UINT32(0U, out.skew_ns);
}

/** A counter running slow by 1 us shows up as a +1 us offset, and vice versa. */
static void test_offset_tracks_the_counter(void)
{
	in.ptp_ns -= 1000;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());
	TEST_ASSERT_EQUAL_INT64(1000, out.off_ns);

	nominal(HZ / 4U);
	in.ptp_ns += 1000;
	in.coarse_tai_ns = in.ptp_ns; /* keep the witness on the counter */
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());
	TEST_ASSERT_EQUAL_INT64(-1000, out.off_ns);
}

/**
 * The edge age is what carries the reference forward from the pulse to the
 * paired read, so it must scale with the counter difference and nothing else.
 */
static void test_edge_age_scales_with_the_counter(void)
{
	nominal(HZ); /* a full second of counts */
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());
	TEST_ASSERT_EQUAL_INT64(1000000000, out.edge_age_ns);
	TEST_ASSERT_EQUAL_INT64(ANCHOR_TAI_NS + INT64_C(1000000000),
				out.ref_tai_ns);
	TEST_ASSERT_EQUAL_INT64(0, out.off_ns);
}

/* ------------------------------------------------------------- 32-bit wrap */

/**
 * TIM2 is 32 bits at 250 MHz — it wraps every 17.18 s, which is well inside the
 * span between a capture and a paired read that arrives a second later. The
 * difference must be taken modulo 2^32, and getting that wrong turns a 1 ms-old
 * capture into a 17-second-old one, i.e. a 17-second error in the served time.
 */
static void test_capture_before_a_counter_wrap(void)
{
	nominal(0U);

	in.tim2_cnt = 0xFFFFFF00U;          /* 256 counts before the wrap */
	in.tim2_a = 0x00000100U;            /* 256 counts after it */
	in.tim2_b = in.tim2_a;
	/* 512 counts = 2048 ns of real time across the wrap. */
	in.ptp_ns = ANCHOR_TAI_NS + 2048;
	in.coarse_tai_ns = in.ptp_ns;

	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());
	TEST_ASSERT_EQUAL_INT64(2048, out.edge_age_ns);
	TEST_ASSERT_EQUAL_INT64(0, out.off_ns);
}

/** The same wrap, with the paired read's own span straddling the wrap too. */
static void test_paired_read_spanning_a_counter_wrap(void)
{
	nominal(0U);

	in.tim2_cnt = 0xFFFF0000U;
	in.tim2_a = 0xFFFFFFF0U; /* 16 counts before the wrap */
	in.tim2_b = 0x00000010U; /* 16 counts after it: a 32-count span */
	in.ptp_ns = ANCHOR_TAI_NS + (int64_t)(0x10000U * NS_PER_COUNT);
	in.coarse_tai_ns = in.ptp_ns;

	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());
	TEST_ASSERT_EQUAL_UINT32(32U * NS_PER_COUNT, out.skew_ns);
	/* Midpoint of [0xFFFFFFF0, 0x00000010] is 0x00000000, i.e. 0x10000
	 * counts after the capture. */
	TEST_ASSERT_EQUAL_INT64((int64_t)0x10000U * NS_PER_COUNT,
				out.edge_age_ns);
	TEST_ASSERT_EQUAL_INT64(0, out.off_ns);
}

/**
 * A capture exactly 2^32 counts old aliases to age 0 on TIM2 alone.
 *
 * This is the case the millisecond cross-check exists for, and it is the one an
 * implementation that only looks at the counters cannot see: the arithmetic is
 * self-consistent and the answer is 17.18 seconds wrong.
 */
static void test_a_full_wrap_is_caught_by_the_millisecond_clock(void)
{
	nominal(0U);

	in.tim2_a = in.tim2_cnt; /* 2^32 counts later == the same value */
	in.tim2_b = in.tim2_a;
	in.now_mono_ms = in.cap_mono_ms + 17179U; /* 2^32 / 250e6 s */

	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_STALE, eval());
}

/* ------------------------------------------------- the paired read's midpoint */

/**
 * The PTP counter is read between the two TIM2 reads, so the midpoint of the
 * pair is the best estimate of the instant it names. Half the span, not zero and
 * not all of it.
 */
static void test_midpoint_of_the_paired_read(void)
{
	nominal(0U);

	in.tim2_a = in.tim2_cnt + 1000U;
	in.tim2_b = in.tim2_a + 100U; /* a 400 ns span */
	/* The counter is right for the MIDPOINT, 1050 counts after the edge. */
	in.ptp_ns = ANCHOR_TAI_NS + (int64_t)(1050U * NS_PER_COUNT);
	in.coarse_tai_ns = in.ptp_ns;

	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());
	TEST_ASSERT_EQUAL_UINT32(400U, out.skew_ns);
	TEST_ASSERT_EQUAL_INT64((int64_t)(1050U * NS_PER_COUNT), out.edge_age_ns);
	TEST_ASSERT_EQUAL_INT64(0, out.off_ns);
}

/** A paired read that took too long no longer names one instant. */
static void test_an_overlong_paired_read_is_refused(void)
{
	uint32_t max_counts = (uint32_t)(STS_PPSCORR_MAX_SKEW_NS / NS_PER_COUNT);

	nominal(0U);
	in.tim2_a = in.tim2_cnt;
	in.tim2_b = in.tim2_a + max_counts;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());
	TEST_ASSERT_EQUAL_UINT32((uint32_t)STS_PPSCORR_MAX_SKEW_NS, out.skew_ns);

	in.tim2_b = in.tim2_a + max_counts + 1U;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_SKEW, eval());
}

/** Reads that came back out of order look like an enormous span, and are refused. */
static void test_reversed_paired_reads_are_refused(void)
{
	nominal(0U);
	in.tim2_a = in.tim2_cnt + 1000U;
	in.tim2_b = in.tim2_cnt + 900U; /* "before" a: wraps to ~4.29e9 counts */
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_SKEW, eval());
}

/* --------------------------------------------------------------- staleness */

static void test_a_capture_older_than_the_limit_is_refused(void)
{
	uint32_t counts = (uint32_t)((STS_PPSCORR_MAX_AGE_NS / NS_PER_COUNT) + 1U);

	nominal(0U);
	in.tim2_a = in.tim2_cnt + counts;
	in.tim2_b = in.tim2_a;
	in.now_mono_ms = in.cap_mono_ms + 2500U;
	in.ptp_ns = ANCHOR_TAI_NS + (int64_t)STS_PPSCORR_MAX_AGE_NS + 4;
	in.coarse_tai_ns = in.ptp_ns;

	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_STALE, eval());
}

/** A capture stamped in the future is not a capture. */
static void test_a_capture_from_the_future_is_refused(void)
{
	in.now_mono_ms = in.cap_mono_ms - 1U;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_STALE, eval());
}

/** The two ages may differ by the tolerance, but not past it. */
static void test_the_two_clocks_must_agree_on_the_age(void)
{
	nominal(HZ / 4U); /* 250 ms of TIM2 counts */

	in.now_mono_ms = in.cap_mono_ms + 250U + (uint64_t)STS_PPSCORR_AGE_TOL_MS;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());

	in.now_mono_ms = in.cap_mono_ms + 250U +
			 (uint64_t)STS_PPSCORR_AGE_TOL_MS + 1U;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_STALE, eval());

	/* And the other way: the millisecond clock behind the counter. */
	in.now_mono_ms = in.cap_mono_ms + 250U -
			 (uint64_t)STS_PPSCORR_AGE_TOL_MS - 1U;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_STALE, eval());
}

/* ------------------------------------------------- §3.2 conditioning terms */

/**
 * The sawtooth. u-blox reports a quantisation error in picoseconds; a positive
 * qErr means the pulse came out LATE, so the true second boundary was earlier
 * and the reference at the paired read is correspondingly further on. Same sign
 * convention as core/disc, which adds it to the phase error for the same reason.
 */
static void test_the_sawtooth_advances_the_reference(void)
{
	nominal(0U);
	in.qerr_ps = 12000; /* +12 ns */
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());
	TEST_ASSERT_EQUAL_INT64(ANCHOR_TAI_NS + 12, out.ref_tai_ns);
	TEST_ASSERT_EQUAL_INT64(12, out.off_ns);

	nominal(0U);
	in.qerr_ps = -12000;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());
	TEST_ASSERT_EQUAL_INT64(-12, out.off_ns);
}

/** disc_cfg_t::qerr_sign inverts it, for a receiver with the other convention. */
static void test_qerr_sign_inverts_the_sawtooth(void)
{
	nominal(0U);
	in.qerr_sign = -1;
	in.qerr_ps = 12000;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());
	TEST_ASSERT_EQUAL_INT64(-12, out.off_ns);
}

/** Picoseconds round to the nearest nanosecond, symmetrically about zero. */
static void test_sawtooth_rounding(void)
{
	TEST_ASSERT_EQUAL_INT64(0, sts_ppscorr_ps_to_ns(499));
	TEST_ASSERT_EQUAL_INT64(1, sts_ppscorr_ps_to_ns(500));
	TEST_ASSERT_EQUAL_INT64(0, sts_ppscorr_ps_to_ns(-499));
	TEST_ASSERT_EQUAL_INT64(-1, sts_ppscorr_ps_to_ns(-500));
	TEST_ASSERT_EQUAL_INT64(-3, sts_ppscorr_ps_to_ns(-2500));
}

/** Anything but +-1 is not a sign, and must not be multiplied by. */
static void test_a_nonsense_qerr_sign_is_refused(void)
{
	in.qerr_sign = 0;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_BAD_QSIGN, eval());
	in.qerr_sign = 2;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_BAD_QSIGN, eval());
}

/**
 * The cable delay. The edge arrives LATER than the instant it represents, so
 * compensating for it moves the reference forward exactly as the sawtooth does
 * — core/disc: "Both refer the measured edge backwards in time".
 */
static void test_the_cable_delay_advances_the_reference(void)
{
	nominal(0U);
	in.cable_delay_ns = 137;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());
	TEST_ASSERT_EQUAL_INT64(ANCHOR_TAI_NS + 137, out.ref_tai_ns);
	TEST_ASSERT_EQUAL_INT64(137, out.off_ns);
}

/* ------------------------------------------------------ the refusal set */

static void test_nothing_captured_yet(void)
{
	in.seq = 0U;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_NO_CAPTURE, eval());
}

/**
 * A lost edge invalidates the pairing even though the ToW still matches: the
 * TIM-TP record was paired against the capture's own timestamp, and the capture
 * that survived is not necessarily the one it names.
 */
static void test_a_lost_edge_refuses_the_correlation(void)
{
	in.overcapture = true;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OVERCAPTURE, eval());
}

/** No positively-paired TIM-TP: no second, no sawtooth, no correlation. */
static void test_an_unnamed_pulse_refuses_the_correlation(void)
{
	in.epoch_valid = false;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_NO_EPOCH, eval());
}

static void test_an_implausible_timer_rate_is_refused(void)
{
	in.timer_hz = STS_PPSCORR_MIN_TIMER_HZ - 1U;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_BAD_TIMER, eval());
	in.timer_hz = 0U;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_BAD_TIMER, eval());
}

/* -------------------------------------------- the independent civil-time witness */

/** Without the witness the offset is still computed, but it is not accepted. */
static void test_no_witness_means_no_correlation(void)
{
	in.coarse_valid = false;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_NO_COARSE, eval());
	TEST_ASSERT_EQUAL_INT64(0, out.off_ns); /* computed, and it agrees */
	TEST_ASSERT_EQUAL_INT64(ANCHOR_TAI_NS + 250000000, out.ref_tai_ns);
}

/** Inside the tolerance, either way, the witness confirms. */
static void test_the_witness_may_disagree_up_to_the_tolerance(void)
{
	in.coarse_tai_ns = in.ptp_ns + STS_PPSCORR_COARSE_TOL_NS;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());
	TEST_ASSERT_EQUAL_INT64(STS_PPSCORR_COARSE_TOL_NS, out.coarse_resid_ns);

	in.coarse_tai_ns = in.ptp_ns - STS_PPSCORR_COARSE_TOL_NS;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OK, eval());
}

static void test_the_witness_vetoes_beyond_the_tolerance(void)
{
	in.coarse_tai_ns = in.ptp_ns + STS_PPSCORR_COARSE_TOL_NS + 1;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_DISAGREE, eval());

	in.coarse_tai_ns = in.ptp_ns - STS_PPSCORR_COARSE_TOL_NS - 1;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_DISAGREE, eval());
}

/**
 * The case the witness is really there for: a GPS week number short by one
 * 1024-week rollover. The arithmetic is perfectly self-consistent and the answer
 * is 19.6 years wrong, so only an independent source can catch it.
 */
static void test_a_week_rollover_is_caught_by_the_witness(void)
{
	nominal(0U);
	in.gps_week = ANCHOR_WEEK - 1024U;

	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_DISAGREE, eval());
	TEST_ASSERT_EQUAL_INT64(INT64_C(1024) * STS_PPSCORR_WEEK_S *
					INT64_C(1000000000),
				out.coarse_resid_ns);
}

/* ------------------------------------------------------------- conversions */

static void test_counts_to_ns(void)
{
	/* Exact at 250 MHz. */
	TEST_ASSERT_EQUAL_UINT64(4U, sts_ppscorr_counts_to_ns(1U, HZ));
	TEST_ASSERT_EQUAL_UINT64(1000000000U, sts_ppscorr_counts_to_ns(HZ, HZ));
	/* The full 32-bit range must not overflow: 2^32-1 counts = 17.179 s. */
	TEST_ASSERT_EQUAL_UINT64(17179869180U,
				 sts_ppscorr_counts_to_ns(0xFFFFFFFFU, HZ));
	/* Rounds to nearest on a rate that is not a divisor of 1e9. */
	TEST_ASSERT_EQUAL_UINT64(3U, sts_ppscorr_counts_to_ns(1U, 300000000U));
	TEST_ASSERT_EQUAL_UINT64(0U, sts_ppscorr_counts_to_ns(1U, 0U));
}

/* ---------------------------------------------------------------- ordering */

/**
 * The refusals are ordered so the most fundamental wins. A capture that never
 * happened must not be reported as a skew problem, and a bad timer rate must not
 * be reported as staleness — the diagnostics are what a bench engineer reads.
 */
static void test_refusal_precedence(void)
{
	in.timer_hz = 0U;
	in.seq = 0U;
	in.overcapture = true;
	in.epoch_valid = false;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_BAD_TIMER, eval());

	in.timer_hz = HZ;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_NO_CAPTURE, eval());

	in.seq = 1U;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_OVERCAPTURE, eval());

	in.overcapture = false;
	TEST_ASSERT_EQUAL_INT(STS_PPSCORR_NO_EPOCH, eval());
}

/** Every refusal has a name; an out-of-range one still yields a printable string. */
static void test_every_refusal_has_a_name(void)
{
	int i;

	for (i = 0; i < (int)STS_PPSCORR__COUNT; i++) {
		const char *n = sts_ppscorr_rc_name((sts_ppscorr_rc_t)i);

		TEST_ASSERT_NOT_NULL(n);
		TEST_ASSERT_TRUE(n[0] != '\0');
	}
	TEST_ASSERT_EQUAL_STRING("?", sts_ppscorr_rc_name(
					      (sts_ppscorr_rc_t)STS_PPSCORR__COUNT));
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_gps_week_tow_to_tai_anchor);
	RUN_TEST(test_sub_second_tow_is_carried);
	RUN_TEST(test_week_beyond_the_counters_range_is_refused);
	RUN_TEST(test_tow_past_the_week_is_refused);

	RUN_TEST(test_nominal_offset_is_zero);
	RUN_TEST(test_offset_tracks_the_counter);
	RUN_TEST(test_edge_age_scales_with_the_counter);

	RUN_TEST(test_capture_before_a_counter_wrap);
	RUN_TEST(test_paired_read_spanning_a_counter_wrap);
	RUN_TEST(test_a_full_wrap_is_caught_by_the_millisecond_clock);

	RUN_TEST(test_midpoint_of_the_paired_read);
	RUN_TEST(test_an_overlong_paired_read_is_refused);
	RUN_TEST(test_reversed_paired_reads_are_refused);

	RUN_TEST(test_a_capture_older_than_the_limit_is_refused);
	RUN_TEST(test_a_capture_from_the_future_is_refused);
	RUN_TEST(test_the_two_clocks_must_agree_on_the_age);

	RUN_TEST(test_the_sawtooth_advances_the_reference);
	RUN_TEST(test_qerr_sign_inverts_the_sawtooth);
	RUN_TEST(test_sawtooth_rounding);
	RUN_TEST(test_a_nonsense_qerr_sign_is_refused);
	RUN_TEST(test_the_cable_delay_advances_the_reference);

	RUN_TEST(test_nothing_captured_yet);
	RUN_TEST(test_a_lost_edge_refuses_the_correlation);
	RUN_TEST(test_an_unnamed_pulse_refuses_the_correlation);
	RUN_TEST(test_an_implausible_timer_rate_is_refused);

	RUN_TEST(test_no_witness_means_no_correlation);
	RUN_TEST(test_the_witness_may_disagree_up_to_the_tolerance);
	RUN_TEST(test_the_witness_vetoes_beyond_the_tolerance);
	RUN_TEST(test_a_week_rollover_is_caught_by_the_witness);

	RUN_TEST(test_counts_to_ns);
	RUN_TEST(test_refusal_precedence);
	RUN_TEST(test_every_refusal_has_a_name);

	return UNITY_END();
}
