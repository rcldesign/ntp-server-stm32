/*
 * STS1000 "Meridian" — the web plane's GNSS-detail decisions, tested away from
 * Zephyr.
 *
 * net/sts_web_sky.h exists because the web skyplot had no per-satellite data at
 * all: pv_gnss() memset the block and hardcoded `detail_available = false`,
 * behind a comment claiming the GNSS thread was not in the tree. It was, and
 * the front panel had been rendering from it for some time. Spec §336 ("the
 * skyplot mirrors the local-UI renderer") and §371 ("identical data feeds the
 * web skyplot so both views agree") were unimplemented rather than merely
 * divergent, and the stale comment is why that survived — it read as a known
 * deferral rather than a live defect.
 *
 * Everything this file pins is invisible at runtime when it is wrong:
 *
 *   1. STALENESS. sts_app.h is explicit that a receiver which has stopped
 *      talking LEAVES THE LAST GOOD FRAME IN THE CACHE. An unaged read
 *      therefore serves an hour-old sky as current — a full satellite list,
 *      plausible C/N0s, and nothing anywhere saying the antenna cable was cut.
 *      That is the failure mode a skyplot exists to make visible, so a defect
 *      here defeats the feature silently and completely.
 *
 *   2. EMPTY IS NOT ABSENT. `detail_available` must be true for a fresh but
 *      empty list. An indoor unit with no sky legitimately reports zero
 *      satellites, and a client that reads that as "no data" cannot tell it
 *      from a dead receiver. Conflating the two is a one-character change and
 *      produces output that looks entirely reasonable.
 *
 *   3. THE ARRAY BOUND. The cache holds STS_GNSS_SKY_MAX_SV records, the
 *      document REST_SAT_MAX. Both are 32 today. An off-by-one writes past
 *      rest_gnss_t::sat[] on a worker stack, from receiver-supplied data.
 *
 *   4. AGREEMENT WITH THE PANEL. §371 is a claim about two independently
 *      written renderers, and the only way to hold it is to compare them. The
 *      sweep below drives ui/sts_sky_policy.h's sts_sky_sv_from_ubx() and
 *      net/sts_web_sky.h's sts_web_sky_admit() over the whole gnssId x
 *      elevation x azimuth x C/N0 x used space and requires the same verdict
 *      and the same normalised numbers from both. A future edit to either that
 *      breaks the other fails here instead of in the field, where the symptom
 *      is one marker present on the panel and absent in a browser.
 *
 *      This file reaches across the area seam that sts_app.h forbids the
 *      PRODUCTION code to cross (net must not include a ui-private header). A
 *      test is not an area, and pinning the two implementations together is
 *      exactly what the seam makes impossible to do in C — the same argument
 *      test_smear_isolation.c makes for reading the sources.
 *
 *   5. THE DOCUMENT FITS THE BUFFER. Filling a previously always-empty array
 *      with up to 32 objects is a ~2.6 KB growth in a response the net area
 *      serves from a fixed STS_WEB_RESP_SIZE buffer, and web_jw_finish()'s
 *      -ENOSPC turns into a telemetry frame that is simply never sent. The
 *      worst-case document is built here and measured against the real
 *      constant, not a test-local one.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "web/rest.h"
#include "web/wss.h"
#include "zephyr/net/sts_web.h"     /* STS_WEB_RESP_SIZE */
#include "zephyr/net/sts_web_sky.h" /* the unit under test */
#include "zephyr/ui/sts_sky_policy.h" /* the panel's rule, for the §371 pin */

#ifndef STS_APP_SRC_DIR
#error "STS_APP_SRC_DIR must be defined by tests/host/CMakeLists.txt"
#endif

/* ===================================================================== */
/* fixtures                                                              */
/* ===================================================================== */

static sts_gnss_sky_t g_sky;
static rest_gnss_t    g_out;

/* An arbitrary "now" well clear of zero, so an age can be taken either way. */
#define NOW 1000000ULL

static void sky_reset(void)
{
	memset(&g_sky, 0, sizeof(g_sky));
	memset(&g_out, 0, sizeof(g_out));
}

/*
 * File-scope and mutable, so setUp() has to clear them rather than leaving it to
 * every test to remember. sky_add() uses g_sky.count as its append index, so a
 * frame left over from the previous test does not merely add noise — it shifts
 * every index the assertions name, and the suite reads as if it were testing
 * something else. This suite happened to be correct by convention; a static that
 * was reset by convention is exactly what once left every later test in another
 * suite wired to the previous one's fixture.
 */
void setUp(void)
{
	sky_reset();
}

void tearDown(void)
{
}

/* Append one measurable satellite. Returns its index. */
static uint8_t sky_add(uint8_t gnss_id, uint8_t sv_id, int8_t elev,
		       int16_t azim, uint8_t cno, bool used)
{
	uint8_t i = g_sky.count;

	TEST_ASSERT_TRUE(i < (uint8_t)STS_GNSS_SKY_MAX_SV);
	g_sky.sv[i].gnss_id = gnss_id;
	g_sky.sv[i].sv_id = sv_id;
	g_sky.sv[i].elev_deg = elev;
	g_sky.sv[i].azim_deg = azim;
	g_sky.sv[i].cno_dbhz = cno;
	g_sky.sv[i].used = used;
	g_sky.count = (uint8_t)(i + 1U);
	return i;
}

/* ===================================================================== */
/* freshness                                                             */
/* ===================================================================== */

static void test_a_frame_that_never_arrived_is_not_fresh(void)
{
	/* mono_ms 0 is sts_app.h's "never received", not "at boot". */
	TEST_ASSERT_FALSE(sts_web_sky_fresh(NOW, 0ULL));
	TEST_ASSERT_FALSE(sts_web_sky_fresh(0ULL, 0ULL));
}

static void test_the_staleness_window_is_inclusive_at_its_edge(void)
{
	uint64_t frame = NOW - (uint64_t)STS_WEB_SKY_STALE_MS;

	/* Exactly at the window: still fresh. One millisecond past: not. */
	TEST_ASSERT_TRUE(sts_web_sky_fresh(NOW, frame));
	TEST_ASSERT_FALSE(sts_web_sky_fresh(NOW, frame - 1ULL));

	/* And the trivially fresh cases either side of the edge. */
	TEST_ASSERT_TRUE(sts_web_sky_fresh(NOW, NOW));
	TEST_ASSERT_TRUE(sts_web_sky_fresh(NOW, NOW - 1ULL));
	TEST_ASSERT_TRUE(sts_web_sky_fresh(NOW, frame + 1ULL));
}

static void test_a_frame_from_the_future_is_refused_not_wrapped(void)
{
	/*
	 * Cannot happen from one monotonic clock. If it is seen, the snapshot
	 * is torn or the caller passed the wrong clock — and an unsigned
	 * subtraction would turn it into either an enormous age or, one
	 * millisecond out, a tiny one that passes every test forever.
	 */
	TEST_ASSERT_FALSE(sts_web_sky_fresh(NOW, NOW + 1ULL));
	TEST_ASSERT_FALSE(sts_web_sky_fresh(0ULL, 1ULL));
	TEST_ASSERT_FALSE(sts_web_sky_fresh(NOW, UINT64_MAX));
}

static void test_a_long_uptime_does_not_break_the_window(void)
{
	/* 64-bit milliseconds: no 49.7-day wrap, unlike a uint32 clock. */
	uint64_t late = 5ULL * 365ULL * 24ULL * 3600ULL * 1000ULL;

	TEST_ASSERT_TRUE(sts_web_sky_fresh(late, late - 1000ULL));
	TEST_ASSERT_FALSE(sts_web_sky_fresh(late, late - 60000ULL));
}

static void test_the_age_is_only_meaningful_when_it_is_measurable(void)
{
	uint32_t age = 0xDEADBEEFu;

	/* Never received: not an age of zero. */
	TEST_ASSERT_FALSE(sts_web_sky_age_ms(NOW, 0ULL, &age));
	TEST_ASSERT_EQUAL_UINT32(0u, age);

	/* Future-stamped: refused, NOT reported as 2^64-minus-a-bit saturated
	 * to "49.7 days old", which is what an unguarded subtraction gives. */
	age = 0xDEADBEEFu;
	TEST_ASSERT_FALSE(sts_web_sky_age_ms(NOW, NOW + 1ULL, &age));
	TEST_ASSERT_EQUAL_UINT32(0u, age);
	TEST_ASSERT_FALSE(sts_web_sky_age_ms(NOW, UINT64_MAX, &age));
	TEST_ASSERT_EQUAL_UINT32(0u, age);

	/* A real age, exactly. */
	TEST_ASSERT_TRUE(sts_web_sky_age_ms(NOW, NOW - 1234ULL, &age));
	TEST_ASSERT_EQUAL_UINT32(1234u, age);
	TEST_ASSERT_TRUE(sts_web_sky_age_ms(NOW, NOW, &age));
	TEST_ASSERT_EQUAL_UINT32(0u, age);

	/* Saturating, not truncating: 2^32 ms is 49.7 days and this box is
	 * specified to run for years. */
	TEST_ASSERT_TRUE(sts_web_sky_age_ms((1ULL << 32) + 6ULL, 1ULL, &age));
	TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, age);

	/* And the out pointer is optional. */
	TEST_ASSERT_TRUE(sts_web_sky_age_ms(NOW, NOW - 1ULL, NULL));
	TEST_ASSERT_FALSE(sts_web_sky_age_ms(NOW, 0ULL, NULL));
}

static void test_the_readable_record_count_is_clamped_to_the_array(void)
{
	/*
	 * `count` is a uint8_t the platform area copies out of a UBX frame. A
	 * value past the array it indexes must become a bound, not a loop trip
	 * count — the failure mode otherwise is an out-of-bounds read of
	 * receiver-supplied data, which no host test can observe.
	 */
	TEST_ASSERT_EQUAL_UINT(0u, sts_web_sky_usable(0u));
	TEST_ASSERT_EQUAL_UINT(1u, sts_web_sky_usable(1u));
	TEST_ASSERT_EQUAL_UINT((unsigned int)STS_GNSS_SKY_MAX_SV - 1U,
			       sts_web_sky_usable(
				       (uint8_t)(STS_GNSS_SKY_MAX_SV - 1U)));
	TEST_ASSERT_EQUAL_UINT((unsigned int)STS_GNSS_SKY_MAX_SV,
			       sts_web_sky_usable(
				       (uint8_t)STS_GNSS_SKY_MAX_SV));
	TEST_ASSERT_EQUAL_UINT((unsigned int)STS_GNSS_SKY_MAX_SV,
			       sts_web_sky_usable(
				       (uint8_t)(STS_GNSS_SKY_MAX_SV + 1U)));
	TEST_ASSERT_EQUAL_UINT((unsigned int)STS_GNSS_SKY_MAX_SV,
			       sts_web_sky_usable(255u));
}

/* ===================================================================== */
/* admission                                                             */
/* ===================================================================== */

static void test_an_unmeasured_almanac_record_is_refused(void)
{
	rest_sat_t s;

	/*
	 * C/N0 zero and not in the solution: UBX's shape for "I know this one
	 * from the almanac". Its azimuth and elevation are both zero, so
	 * admitting it paints a phantom marker at due north on the horizon.
	 */
	TEST_ASSERT_FALSE(sts_web_sky_admit(0u, 9u, 0, 0, 0u, false, &s));
	TEST_ASSERT_FALSE(sts_web_sky_admit(0u, 9u, 30, 120, 0u, false, &s));
}

static void test_a_satellite_in_the_solution_is_always_admitted(void)
{
	rest_sat_t s;

	/* Being used is stronger evidence than a momentary C/N0 of zero. */
	memset(&s, 0xAA, sizeof(s));
	TEST_ASSERT_TRUE(sts_web_sky_admit(0u, 9u, 30, 120, 0u, true, &s));
	TEST_ASSERT_TRUE(s.used);
	TEST_ASSERT_EQUAL_UINT(0u, s.cno);
	TEST_ASSERT_EQUAL_UINT(9u, s.sv_id);
}

static void test_every_field_survives_admission(void)
{
	rest_sat_t s;

	memset(&s, 0xAA, sizeof(s));
	TEST_ASSERT_TRUE(sts_web_sky_admit(2u, 27u, 41, 133, 44u, true, &s));
	TEST_ASSERT_EQUAL_UINT(2u, s.gnss_id);
	TEST_ASSERT_EQUAL_UINT(27u, s.sv_id);
	TEST_ASSERT_EQUAL_INT(41, s.elev_deg);
	TEST_ASSERT_EQUAL_INT(133, s.azim_deg);
	TEST_ASSERT_EQUAL_UINT(44u, s.cno);
	TEST_ASSERT_TRUE(s.used);

	/* The receiver's own gnssId is carried, not a display colour: a client
	 * can name a constellation this firmware has never heard of. */
	TEST_ASSERT_TRUE(sts_web_sky_admit(31u, 1u, 5, 5, 20u, false, &s));
	TEST_ASSERT_EQUAL_UINT(31u, s.gnss_id);
	TEST_ASSERT_FALSE(s.used);
}

static void test_azimuth_is_folded_into_a_full_circle(void)
{
	rest_sat_t s;
	int a;

	/* 360 is a value UBX may report; it must land at 0, not be dropped. */
	TEST_ASSERT_TRUE(sts_web_sky_admit(0u, 1u, 10, 360, 30u, false, &s));
	TEST_ASSERT_EQUAL_INT(0, s.azim_deg);
	TEST_ASSERT_TRUE(sts_web_sky_admit(0u, 1u, 10, 361, 30u, false, &s));
	TEST_ASSERT_EQUAL_INT(1, s.azim_deg);
	TEST_ASSERT_TRUE(sts_web_sky_admit(0u, 1u, 10, -1, 30u, false, &s));
	TEST_ASSERT_EQUAL_INT(359, s.azim_deg);
	TEST_ASSERT_TRUE(sts_web_sky_admit(0u, 1u, 10, -360, 30u, false, &s));
	TEST_ASSERT_EQUAL_INT(0, s.azim_deg);

	/* Nothing in int16 escapes 0..359. */
	for (a = -32768; a <= 32767; a += 7) {
		TEST_ASSERT_TRUE(sts_web_sky_admit(0u, 1u, 45, (int16_t)a, 40u,
						   true, &s));
		TEST_ASSERT_TRUE(s.azim_deg >= 0);
		TEST_ASSERT_TRUE(s.azim_deg <= 359);
	}
}

static void test_a_negative_elevation_is_kept(void)
{
	rest_sat_t s;

	/* The renderers clamp it. Dropping it here would hide a receiver
	 * reporting nonsense behind a plot that merely looks sparse. */
	TEST_ASSERT_TRUE(sts_web_sky_admit(0u, 3u, -20, 90, 25u, false, &s));
	TEST_ASSERT_EQUAL_INT(-20, s.elev_deg);
}

static void test_admission_refuses_a_null_destination(void)
{
	TEST_ASSERT_FALSE(sts_web_sky_admit(0u, 1u, 45, 90, 40u, true, NULL));
}

/* ===================================================================== */
/* fill: the three states a caller must be able to tell apart            */
/* ===================================================================== */

static void test_no_frame_ever_is_reported_as_no_data(void)
{
	sky_reset();
	/* count deliberately non-zero: a cache with records but no timestamp
	 * has never been published, and must not be served. */
	(void)sky_add(0u, 1u, 45, 90, 44u, true);
	g_sky.mono_ms = 0ULL;

	sts_web_sky_fill(&g_sky, NOW, &g_out);
	TEST_ASSERT_FALSE(g_out.detail_available);
	TEST_ASSERT_FALSE(g_out.sat_age_valid);
	TEST_ASSERT_EQUAL_UINT(0u, g_out.n_sats);
}

static void test_a_fresh_but_empty_list_is_data_not_absence(void)
{
	sky_reset();
	g_sky.count = 0U;
	g_sky.mono_ms = NOW - 250ULL;

	sts_web_sky_fill(&g_sky, NOW, &g_out);

	/*
	 * The distinction the whole field exists for. An indoor unit reports
	 * zero satellites and a working receiver; a dead one reports nothing.
	 */
	TEST_ASSERT_TRUE(g_out.detail_available);
	TEST_ASSERT_EQUAL_UINT(0u, g_out.n_sats);
	TEST_ASSERT_TRUE(g_out.sat_age_valid);
	TEST_ASSERT_EQUAL_UINT32(250u, g_out.sat_age_ms);
}

static void test_a_fresh_list_serialises_every_satellite(void)
{
	sky_reset();
	(void)sky_add(0u, 1u, 65, 180, 44u, true);
	(void)sky_add(2u, 11u, 20, 300, 33u, false);
	(void)sky_add(6u, 7u, 70, 5, 48u, true);
	g_sky.mono_ms = NOW - 900ULL;

	sts_web_sky_fill(&g_sky, NOW, &g_out);

	TEST_ASSERT_TRUE(g_out.detail_available);
	TEST_ASSERT_EQUAL_UINT(3u, g_out.n_sats);
	TEST_ASSERT_TRUE(g_out.sat_age_valid);
	TEST_ASSERT_EQUAL_UINT32(900u, g_out.sat_age_ms);

	TEST_ASSERT_EQUAL_UINT(0u, g_out.sat[0].gnss_id);
	TEST_ASSERT_EQUAL_UINT(1u, g_out.sat[0].sv_id);
	TEST_ASSERT_EQUAL_UINT(44u, g_out.sat[0].cno);
	TEST_ASSERT_EQUAL_INT(65, g_out.sat[0].elev_deg);
	TEST_ASSERT_EQUAL_INT(180, g_out.sat[0].azim_deg);
	TEST_ASSERT_TRUE(g_out.sat[0].used);

	TEST_ASSERT_EQUAL_UINT(2u, g_out.sat[1].gnss_id);
	TEST_ASSERT_EQUAL_UINT(11u, g_out.sat[1].sv_id);
	TEST_ASSERT_EQUAL_UINT(33u, g_out.sat[1].cno);
	TEST_ASSERT_EQUAL_INT(20, g_out.sat[1].elev_deg);
	TEST_ASSERT_EQUAL_INT(300, g_out.sat[1].azim_deg);
	TEST_ASSERT_FALSE(g_out.sat[1].used);

	TEST_ASSERT_EQUAL_UINT(6u, g_out.sat[2].gnss_id);
	TEST_ASSERT_EQUAL_UINT(7u, g_out.sat[2].sv_id);
	TEST_ASSERT_EQUAL_UINT(48u, g_out.sat[2].cno);
	TEST_ASSERT_EQUAL_INT(70, g_out.sat[2].elev_deg);
	TEST_ASSERT_EQUAL_INT(5, g_out.sat[2].azim_deg);
	TEST_ASSERT_TRUE(g_out.sat[2].used);
}

static void test_a_stale_list_reports_no_detail_but_keeps_its_age(void)
{
	sky_reset();
	(void)sky_add(0u, 1u, 65, 180, 44u, true);
	(void)sky_add(2u, 11u, 20, 300, 33u, false);
	g_sky.mono_ms = NOW - (uint64_t)STS_WEB_SKY_STALE_MS - 1ULL;

	sts_web_sky_fill(&g_sky, NOW, &g_out);

	/* The satellites are NOT served. Serving them with a flag saying not to
	 * trust them invites exactly the client that ignores the flag. */
	TEST_ASSERT_FALSE(g_out.detail_available);
	TEST_ASSERT_EQUAL_UINT(0u, g_out.n_sats);

	/* But the age is, because it is the useful half of the answer. */
	TEST_ASSERT_TRUE(g_out.sat_age_valid);
	TEST_ASSERT_EQUAL_UINT32((uint32_t)STS_WEB_SKY_STALE_MS + 1u,
				 g_out.sat_age_ms);
}

static void test_the_verdict_flips_exactly_at_the_window(void)
{
	uint64_t frame;

	sky_reset();
	(void)sky_add(0u, 1u, 65, 180, 44u, true);
	frame = NOW - (uint64_t)STS_WEB_SKY_STALE_MS;
	g_sky.mono_ms = frame;

	sts_web_sky_fill(&g_sky, NOW, &g_out);
	TEST_ASSERT_TRUE(g_out.detail_available);
	TEST_ASSERT_EQUAL_UINT(1u, g_out.n_sats);

	sts_web_sky_fill(&g_sky, NOW + 1ULL, &g_out);
	TEST_ASSERT_FALSE(g_out.detail_available);
	TEST_ASSERT_EQUAL_UINT(0u, g_out.n_sats);
}

static void test_phantom_records_are_filtered_out_of_a_fresh_frame(void)
{
	sky_reset();
	(void)sky_add(0u, 3u, 45, 90, 44u, true);   /* kept */
	(void)sky_add(3u, 21u, 0, 0, 0u, false);    /* almanac only: dropped */
	(void)sky_add(2u, 12u, 20, 210, 33u, false);/* kept */
	(void)sky_add(5u, 194u, 0, 0, 0u, false);   /* almanac only: dropped */
	g_sky.mono_ms = NOW - 100ULL;

	sts_web_sky_fill(&g_sky, NOW, &g_out);

	/* Fresh with a filtered-down list is still detail: the count is the
	 * count of what is worth drawing, not of what the frame carried. */
	TEST_ASSERT_TRUE(g_out.detail_available);
	TEST_ASSERT_EQUAL_UINT(2u, g_out.n_sats);
	TEST_ASSERT_EQUAL_UINT(3u, g_out.sat[0].sv_id);
	TEST_ASSERT_EQUAL_UINT(12u, g_out.sat[1].sv_id);
	/* The survivors are packed, not left with holes at the drop sites. */
	TEST_ASSERT_EQUAL_UINT(0u, g_out.sat[2].sv_id);
}

static void test_a_frame_of_only_phantoms_is_fresh_and_empty(void)
{
	sky_reset();
	(void)sky_add(0u, 21u, 0, 0, 0u, false);
	(void)sky_add(0u, 22u, 0, 0, 0u, false);
	g_sky.mono_ms = NOW - 100ULL;

	sts_web_sky_fill(&g_sky, NOW, &g_out);

	/* Records arrived, none were measurable. That is data, and it is what
	 * a receiver whose antenna has just been unplugged reports. */
	TEST_ASSERT_TRUE(g_out.detail_available);
	TEST_ASSERT_EQUAL_UINT(0u, g_out.n_sats);
}

/* ===================================================================== */
/* the array bound                                                       */
/* ===================================================================== */

static void test_a_full_frame_is_carried_whole(void)
{
	unsigned int i;

	sky_reset();
	for (i = 0U; i < (unsigned int)STS_GNSS_SKY_MAX_SV; i++) {
		(void)sky_add((uint8_t)(i % 7U), (uint8_t)(i + 1U), 30,
			      (int16_t)(i * 11U), (uint8_t)(20U + i), true);
	}
	g_sky.mono_ms = NOW - 100ULL;

	sts_web_sky_fill(&g_sky, NOW, &g_out);
	TEST_ASSERT_TRUE(g_out.detail_available);
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_SAT_MAX, g_out.n_sats);
	TEST_ASSERT_EQUAL_UINT((unsigned int)STS_GNSS_SKY_MAX_SV, g_out.n_sats);
	for (i = 0U; i < (unsigned int)REST_SAT_MAX; i++) {
		TEST_ASSERT_EQUAL_UINT(i + 1U, g_out.sat[i].sv_id);
	}
}

static void test_a_count_past_the_cache_bound_is_clamped(void)
{
	unsigned int i;
	uint8_t canary;

	sky_reset();
	for (i = 0U; i < (unsigned int)STS_GNSS_SKY_MAX_SV; i++) {
		(void)sky_add(0u, (uint8_t)(i + 1U), 30, 90, 40u, true);
	}
	/*
	 * A count one past what the cache can hold. The platform area truncates
	 * on the way in so this cannot arise today, but the loop must be bound
	 * by the ARRAY and not only by the count — otherwise a single wrong
	 * store in platform/gnss.c becomes an out-of-bounds READ here and an
	 * out-of-bounds WRITE into rest_gnss_t::sat[] on a worker stack, from
	 * receiver-supplied data.
	 */
	g_sky.count = (uint8_t)(STS_GNSS_SKY_MAX_SV + 1U);
	g_sky.mono_ms = NOW - 100ULL;

	memset(&g_out, 0, sizeof(g_out));
	sts_web_sky_fill(&g_sky, NOW, &g_out);
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_SAT_MAX, g_out.n_sats);

	/* Nothing was written past the last legal slot. */
	canary = 0U;
	for (i = 0U; i < sizeof(g_out.sw_version); i++) {
		canary |= (uint8_t)g_out.sw_version[i];
	}
	TEST_ASSERT_EQUAL_UINT(0u, canary);

	/* And an absurd count is clamped just the same. */
	g_sky.count = 255U;
	memset(&g_out, 0, sizeof(g_out));
	sts_web_sky_fill(&g_sky, NOW, &g_out);
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_SAT_MAX, g_out.n_sats);
}

static void test_a_full_frame_of_phantoms_below_the_bound_still_counts(void)
{
	unsigned int i;

	/* 32 records, only the last two measurable: the count must be 2, not
	 * 32 and not 0, and the two must be at index 0 and 1. */
	sky_reset();
	for (i = 0U; i < (unsigned int)STS_GNSS_SKY_MAX_SV; i++) {
		bool real = (i >= ((unsigned int)STS_GNSS_SKY_MAX_SV - 2U));

		(void)sky_add(0u, (uint8_t)(i + 1U), real ? 30 : 0,
			      real ? 90 : 0, real ? 40u : 0u, false);
	}
	g_sky.mono_ms = NOW - 100ULL;

	sts_web_sky_fill(&g_sky, NOW, &g_out);
	TEST_ASSERT_EQUAL_UINT(2u, g_out.n_sats);
	TEST_ASSERT_EQUAL_UINT((unsigned int)STS_GNSS_SKY_MAX_SV - 1U,
			       g_out.sat[0].sv_id);
	TEST_ASSERT_EQUAL_UINT((unsigned int)STS_GNSS_SKY_MAX_SV,
			       g_out.sat[1].sv_id);
}

static void test_fill_tolerates_a_missing_snapshot(void)
{
	memset(&g_out, 0xAA, sizeof(g_out));
	sts_web_sky_fill(NULL, NOW, &g_out);
	TEST_ASSERT_FALSE(g_out.detail_available);
	TEST_ASSERT_FALSE(g_out.sat_age_valid);
	TEST_ASSERT_EQUAL_UINT(0u, g_out.n_sats);

	/* And a NULL destination is a no-op rather than a fault. */
	sky_reset();
	g_sky.mono_ms = NOW;
	sts_web_sky_fill(&g_sky, NOW, NULL);
}

static void test_fill_leaves_the_quality_derived_members_alone(void)
{
	/* pv_gnss() fills the quality half and the sky half in whichever order
	 * reads best; the sky half must not clear the other. */
	sky_reset();
	g_sky.mono_ms = NOW;
	g_out.fix_type = 3U;
	g_out.sv_used = 12U;
	g_out.sv_visible = 18U;
	g_out.ant_state = (uint8_t)REST_ANT_OK;
	g_out.leap_current_s = 37;

	sts_web_sky_fill(&g_sky, NOW, &g_out);

	TEST_ASSERT_EQUAL_UINT(3u, g_out.fix_type);
	TEST_ASSERT_EQUAL_UINT(12u, g_out.sv_used);
	TEST_ASSERT_EQUAL_UINT(18u, g_out.sv_visible);
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_ANT_OK, g_out.ant_state);
	TEST_ASSERT_EQUAL_INT(37, g_out.leap_current_s);
}

static void test_an_enormous_age_saturates_rather_than_truncating(void)
{
	sky_reset();
	g_sky.mono_ms = 1ULL;

	/* 2^32 + 5 ms of uptime. A truncating cast would report 5 ms — a
	 * receiver silent for seven weeks described as current. */
	sts_web_sky_fill(&g_sky, (1ULL << 32) + 6ULL, &g_out);
	TEST_ASSERT_TRUE(g_out.sat_age_valid);
	TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, g_out.sat_age_ms);
	TEST_ASSERT_FALSE(g_out.detail_available);
}

static void test_a_future_stamped_frame_is_not_served_and_has_no_age(void)
{
	sky_reset();
	(void)sky_add(0u, 1u, 45, 90, 44u, true);
	g_sky.mono_ms = NOW + 1ULL;

	sts_web_sky_fill(&g_sky, NOW, &g_out);

	/*
	 * A torn snapshot or the wrong clock. Neither the satellites nor an age
	 * may be served: an unguarded subtraction gives 2^64-1 ms, which
	 * saturates to UINT32_MAX and would be published as "the receiver last
	 * spoke 49.7 days ago" — a measurement, from nothing.
	 */
	TEST_ASSERT_FALSE(g_out.detail_available);
	TEST_ASSERT_FALSE(g_out.sat_age_valid);
	TEST_ASSERT_EQUAL_UINT32(0u, g_out.sat_age_ms);
	TEST_ASSERT_EQUAL_UINT(0u, g_out.n_sats);
}

/* ===================================================================== */
/* §371: the two views agree                                             */
/* ===================================================================== */

static void test_the_admission_rule_matches_the_panels(void)
{
	static const uint8_t gnss_ids[] = { 0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u,
					    31u, 255u };
	static const int16_t azims[] = { -32768, -721, -360, -359, -1, 0, 1,
					 179, 180, 359, 360, 361, 719, 32767 };
	static const int8_t elevs[] = { -128, -90, -1, 0, 1, 45, 89, 90, 127 };
	static const uint8_t cnos[] = { 0u, 1u, 20u, 44u, 255u };
	unsigned int gi, ai, ei, ci, ui;
	unsigned long compared = 0UL;

	for (gi = 0U; gi < (sizeof(gnss_ids) / sizeof(gnss_ids[0])); gi++) {
		for (ai = 0U; ai < (sizeof(azims) / sizeof(azims[0])); ai++) {
			for (ei = 0U; ei < (sizeof(elevs) / sizeof(elevs[0]));
			     ei++) {
				for (ci = 0U;
				     ci < (sizeof(cnos) / sizeof(cnos[0]));
				     ci++) {
					for (ui = 0U; ui < 2U; ui++) {
						rest_sat_t web;
						ui_sv_t panel;
						bool wa, pa;
						bool used = (ui != 0U);

						memset(&web, 0, sizeof(web));
						memset(&panel, 0,
						       sizeof(panel));
						wa = sts_web_sky_admit(
							gnss_ids[gi], 7u,
							elevs[ei], azims[ai],
							cnos[ci], used, &web);
						pa = sts_sky_sv_from_ubx(
							gnss_ids[gi], 7u,
							elevs[ei], azims[ai],
							cnos[ci], used, &panel);

						/* Same verdict... */
						TEST_ASSERT_EQUAL_INT((int)pa,
								      (int)wa);
						compared++;
						if (!wa) {
							continue;
						}
						/* ...and the same numbers. */
						TEST_ASSERT_EQUAL_UINT(
							panel.svid, web.sv_id);
						TEST_ASSERT_EQUAL_INT(
							panel.elev_deg,
							web.elev_deg);
						TEST_ASSERT_EQUAL_INT(
							panel.azim_deg,
							web.azim_deg);
						TEST_ASSERT_EQUAL_UINT(
							panel.cno_dbhz,
							web.cno);
						TEST_ASSERT_EQUAL_INT(
							(int)panel.used,
							(int)web.used);
					}
				}
			}
		}
	}
	/* The sweep actually swept. */
	TEST_ASSERT_TRUE(compared > 1000UL);
}

static void test_both_views_hold_the_same_number_of_satellites(void)
{
	/*
	 * §371 is a claim about two renderers, and it fails just as completely
	 * if one of them silently carries fewer markers than the other.
	 */
	TEST_ASSERT_EQUAL_UINT((unsigned int)UI_MAX_SV,
			       (unsigned int)REST_SAT_MAX);
	TEST_ASSERT_EQUAL_UINT((unsigned int)STS_GNSS_SKY_MAX_SV,
			       (unsigned int)REST_SAT_MAX);
}

#define UI_SRC "zephyr/ui/sts_ui.c"
#define PANEL_STALE_DEFINE "#define SKY_STALE_MS"

/**
 * The panel's staleness window, READ OUT OF ui/sts_ui.c.
 *
 * SKY_STALE_MS is a #define private to a Zephyr translation unit, so it cannot
 * be included — and a local `const unsigned int panel_sky_stale_ms = 5000U;`
 * with the provenance in a comment is not a substitute. That compares
 * STS_WEB_SKY_STALE_MS against a copy of itself: change the panel's define to
 * 3000 and the assertion stays green while the two views blank their satellites
 * two seconds apart, which is the one drift §371 needs this to catch.
 *
 * So the define is parsed out of the source, the same way
 * test_factory_policy.c and test_smear_isolation.c read the tree they cannot
 * link. Absent, duplicated or unparseable is a FAILURE and never a default: a
 * scan that cannot find its subject must not report agreement with it.
 */
static unsigned long panel_sky_stale_ms(void)
{
	static char src[256U * 1024U];
	char path[512];
	FILE *f;
	size_t n;
	size_t i;
	size_t at = 0U;
	size_t tag = sizeof(PANEL_STALE_DEFINE) - 1U;
	unsigned int hits = 0U;
	unsigned long v = 0UL;
	bool digits = false;

	(void)snprintf(path, sizeof(path), "%s/%s", STS_APP_SRC_DIR, UI_SRC);
	f = fopen(path, "rb");
	TEST_ASSERT_NOT_NULL_MESSAGE(f, path);
	n = fread(src, 1U, sizeof(src) - 1U, f);
	(void)fclose(f);
	/* A file that exactly filled the buffer was probably truncated. */
	TEST_ASSERT_TRUE_MESSAGE(n < (sizeof(src) - 1U),
				 UI_SRC " did not fit the scan buffer");
	TEST_ASSERT_TRUE_MESSAGE(n > 4096U, UI_SRC " is implausibly small");
	src[n] = '\0';

	for (i = 0U; (i + tag + 1U) <= n; i++) {
		if (memcmp(&src[i], PANEL_STALE_DEFINE, tag) != 0) {
			continue;
		}
		/* Whole macro name: `#define SKY_STALE_MS_X` is a different one. */
		if ((src[i + tag] != ' ') && (src[i + tag] != '\t')) {
			continue;
		}
		hits++;
		at = i + tag;
	}
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, hits,
		"expected exactly one `" PANEL_STALE_DEFINE "` in " UI_SRC
		"; the panel's staleness window cannot be read, so the "
		"agreement spec §371 requires cannot be checked — fix this "
		"scan, do not delete the assertion it feeds");

	while ((at < n) && ((src[at] == ' ') || (src[at] == '\t'))) {
		at++;
	}
	while ((at < n) && (src[at] >= '0') && (src[at] <= '9')) {
		v = (v * 10UL) + (unsigned long)(src[at] - '0');
		digits = true;
		at++;
	}
	TEST_ASSERT_TRUE_MESSAGE(digits,
				 PANEL_STALE_DEFINE " in " UI_SRC
				 " is not a decimal literal");
	/* Only an integer suffix may follow: an expression would mean the number
	 * read here is not the number the panel compiles. */
	while ((at < n) && ((src[at] == 'u') || (src[at] == 'U') ||
			    (src[at] == 'l') || (src[at] == 'L'))) {
		at++;
	}
	TEST_ASSERT_TRUE_MESSAGE((at < n) && ((src[at] == '\n') ||
					      (src[at] == '\r') ||
					      (src[at] == ' ') ||
					      (src[at] == '\t') ||
					      (src[at] == '/')),
				 PANEL_STALE_DEFINE " in " UI_SRC
				 " is not a plain integer literal");
	return v;
}

static void test_the_two_staleness_windows_are_independent_but_agree(void)
{
	/*
	 * The web owns its own constant on purpose (see sts_web_sky.h), so this
	 * is not a redundant copy of a shared value — it is the assertion that
	 * the two independently derived numbers still coincide. If a future
	 * change moves either one, §371 has to be re-argued, and this is where
	 * that argument gets forced.
	 */
	unsigned long panel = panel_sky_stale_ms();

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		(unsigned int)panel, (unsigned int)STS_WEB_SKY_STALE_MS,
		"ui/sts_ui.c SKY_STALE_MS and net/sts_web_sky.h "
		"STS_WEB_SKY_STALE_MS have drifted apart: the panel and the web "
		"skyplot would blank their satellites at different ages, which "
		"is exactly what spec §371 forbids. Re-argue §371 — do not "
		"re-baseline this number");

	/* And both must clear the floor the web's derivation sets: one NAV-SAT
	 * period (1 s) plus one SPA poll (1 s) plus transport slack. */
	TEST_ASSERT_TRUE((unsigned int)STS_WEB_SKY_STALE_MS >= 3000U);
	TEST_ASSERT_TRUE(panel >= 3000UL);
}

/* ===================================================================== */
/* the antenna verdict                                                   */
/* ===================================================================== */

#define ALRM_OPEN  FAULT_ALARM_BIT(FAULT_ALARM_ANTENNA_OPEN)
#define ALRM_SHORT FAULT_ALARM_BIT(FAULT_ALARM_ANTENNA_SHORT)
#define LOCKED     ((uint32_t)QUALITY_FLAG_GNSS_TIME_LOCKED)

static void test_a_short_outranks_everything_including_a_cut_bias(void)
{
	/*
	 * The precedence that matters most: gnssmgr CUTS THE BIAS when a short
	 * latches, so testing bias_on first would erase the reason the bias is
	 * off the instant the supervisor acted.
	 */
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_ANT_SHORT,
			       sts_web_ant_state(ALRM_SHORT, 0U, false, false));
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_ANT_SHORT,
			       sts_web_ant_state(ALRM_SHORT, LOCKED, false,
						 true));
	/* Short beats open when both are somehow set. */
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_ANT_SHORT,
			       sts_web_ant_state(ALRM_SHORT | ALRM_OPEN, LOCKED,
						 false, true));
}

static void test_a_latched_short_survives_the_alarm_clearing(void)
{
	/*
	 * The supervisor cuts the bias, the short goes away because no current
	 * flows, and the ALARM clears — but the bias stays cut until an
	 * operator calls sts_gnss_ant_reenable(). Without the latch the page
	 * would read "unknown" for a unit that needs exactly one action.
	 */
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_ANT_SHORT,
			       sts_web_ant_state(0U, LOCKED, true, true));
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_ANT_SHORT,
			       sts_web_ant_state(0U, 0U, true, false));
	/* And the latch outranks an open reported at the same time. */
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_ANT_SHORT,
			       sts_web_ant_state(ALRM_OPEN, LOCKED, true,
						 true));
}

static void test_an_open_outranks_an_inference_from_the_time_lock(void)
{
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_ANT_OPEN,
			       sts_web_ant_state(ALRM_OPEN, LOCKED, false,
						 true));
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_ANT_OPEN,
			       sts_web_ant_state(ALRM_OPEN, 0U, false, false));
}

static void test_bias_commanded_off_is_unknown_not_ok(void)
{
	/* No bias, no current to measure: "ok" would be a claim about an
	 * unpowered LNA. */
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_ANT_UNKNOWN,
			       sts_web_ant_state(0U, LOCKED, false, false));
}

static void test_a_time_locked_receiver_on_bias_reads_ok(void)
{
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_ANT_OK,
			       sts_web_ant_state(0U, LOCKED, false, true));
}

static void test_a_cold_start_is_unknown_not_open(void)
{
	/* A cold start under a roof is not a cabling fault, and REST_ANT_OPEN
	 * is what the SPA paints red. */
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_ANT_UNKNOWN,
			       sts_web_ant_state(0U, 0U, false, true));
}

static void test_unrelated_alarms_do_not_move_the_antenna_verdict(void)
{
	uint64_t noise = FAULT_ALARM_BIT(FAULT_ALARM_GNSS_LOST) |
			 FAULT_ALARM_BIT(FAULT_ALARM_THERMAL_WARN) |
			 FAULT_ALARM_BIT(FAULT_ALARM_RB_OV) |
			 FAULT_ALARM_BIT(FAULT_ALARM_PFI);

	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_ANT_OK,
			       sts_web_ant_state(noise, LOCKED, false, true));
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_ANT_SHORT,
			       sts_web_ant_state(noise | ALRM_SHORT, LOCKED,
						 false, true));
}

static void test_the_antenna_verdict_is_total(void)
{
	/* Every (short, open, locked, latch, bias) combination lands on a real
	 * enumerator — no input falls through to a value the SPA cannot name. */
	unsigned int m;

	for (m = 0U; m < 32U; m++) {
		uint64_t alarms = 0U;
		uint32_t flags = 0U;
		uint8_t v;

		if ((m & 1U) != 0U) {
			alarms |= ALRM_SHORT;
		}
		if ((m & 2U) != 0U) {
			alarms |= ALRM_OPEN;
		}
		if ((m & 4U) != 0U) {
			flags |= LOCKED;
		}
		v = sts_web_ant_state(alarms, flags, (m & 16U) != 0U,
				      (m & 8U) != 0U);
		TEST_ASSERT_TRUE(v <= (uint8_t)REST_ANT_SHORT);
	}
}

/* ===================================================================== */
/* survey-in and the stored position                                     */
/* ===================================================================== */

static void test_no_receiver_detail_reads_idle_and_invalid(void)
{
	memset(&g_out, 0xAA, sizeof(g_out));
	sts_web_sky_survey(NULL, &g_out);
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_SURVEY_IDLE,
			       g_out.survey_state);
	TEST_ASSERT_FALSE(g_out.position_valid);
	TEST_ASSERT_EQUAL_UINT32(0u, g_out.survey_dur_s);
	TEST_ASSERT_EQUAL_UINT32(0u, g_out.survey_obs);
	TEST_ASSERT_EQUAL_UINT32(0u, g_out.survey_acc_mm);
	TEST_ASSERT_EQUAL_INT64(0, g_out.ecef_x_cm);

	/* And a NULL destination is a no-op rather than a fault. */
	sts_web_sky_survey(NULL, NULL);
}

static void test_a_running_survey_reports_its_own_progress(void)
{
	sts_gnss_detail_t d;

	memset(&d, 0, sizeof(d));
	d.svin_seen = true;
	d.svin_active = true;
	d.svin_dur_s = 1234U;
	d.svin_obs = 4321U;
	d.svin_acc_0p1mm = 34567U; /* 3456.7 mm */

	memset(&g_out, 0, sizeof(g_out));
	sts_web_sky_survey(&d, &g_out);
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_SURVEY_ACTIVE,
			       g_out.survey_state);
	TEST_ASSERT_EQUAL_UINT32(1234u, g_out.survey_dur_s);
	TEST_ASSERT_EQUAL_UINT32(4321u, g_out.survey_obs);
	TEST_ASSERT_EQUAL_UINT32(3457u, g_out.survey_acc_mm); /* rounded */
}

static void test_a_survey_outranks_a_position_still_on_file(void)
{
	sts_gnss_detail_t d;

	/*
	 * The receiver is no longer using the stored position, so reporting
	 * "fixed" would tell an operator the site is commissioned while it is
	 * in the middle of re-surveying it.
	 */
	memset(&d, 0, sizeof(d));
	d.svin_active = true;
	d.svin_acc_0p1mm = 100U;
	d.pos_valid = true;
	d.pos_x_cm = 1;

	memset(&g_out, 0, sizeof(g_out));
	sts_web_sky_survey(&d, &g_out);
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_SURVEY_ACTIVE,
			       g_out.survey_state);
	/* The position is still reported — it is still on file. */
	TEST_ASSERT_TRUE(g_out.position_valid);
}

static void test_a_stored_position_is_fixed_mode(void)
{
	sts_gnss_detail_t d;

	memset(&d, 0, sizeof(d));
	d.pos_valid = true;
	d.pos_x_cm = 111111111;
	d.pos_y_cm = -222222222;
	d.pos_z_cm = 333333333;
	d.pos_acc_0p1mm = 155U; /* 15.5 mm */

	memset(&g_out, 0, sizeof(g_out));
	sts_web_sky_survey(&d, &g_out);
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_SURVEY_FIXED,
			       g_out.survey_state);
	TEST_ASSERT_TRUE(g_out.position_valid);
	TEST_ASSERT_EQUAL_INT64(111111111, g_out.ecef_x_cm);
	TEST_ASSERT_EQUAL_INT64(-222222222, g_out.ecef_y_cm);
	TEST_ASSERT_EQUAL_INT64(333333333, g_out.ecef_z_cm);
	/* The figure in force is the stored position's, not the last survey's. */
	TEST_ASSERT_EQUAL_UINT32(16u, g_out.survey_acc_mm);
}

/**
 * And the mirror case: a survey that has not yet produced a meanAcc must not
 * borrow the stored position's figure.
 *
 * It is the same one-line ternary that serves the test below, read in the other
 * direction, and it is the case sts_web_sky_survey() used to restate as a
 * follow-up `if` — a branch that could only ever assign 0 over a 0 the line
 * above had already written. Pinned here so deleting that no-op is a change
 * with a test behind it rather than a change nothing describes.
 */
static void test_a_survey_with_no_accuracy_yet_does_not_borrow_the_stored_one(void)
{
	sts_gnss_detail_t d;

	memset(&d, 0, sizeof(d));
	d.svin_seen = true;
	d.svin_active = true;
	d.svin_acc_0p1mm = 0U;   /* NAV-SVIN has not reported meanAcc yet */
	d.pos_valid = true;
	d.pos_acc_0p1mm = 12345U; /* the stored figure, which is NOT in force */

	memset(&g_out, 0, sizeof(g_out));
	sts_web_sky_survey(&d, &g_out);
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_SURVEY_ACTIVE,
			       g_out.survey_state);
	TEST_ASSERT_EQUAL_UINT32(0u, g_out.survey_acc_mm);
}

static void test_a_finished_survey_does_not_lend_its_accuracy_to_a_seed(void)
{
	sts_gnss_detail_t d;

	/* A position seeded from NVS with no accuracy of its own, while the
	 * survey block still holds a completed survey's number. */
	memset(&d, 0, sizeof(d));
	d.svin_seen = true;
	d.svin_ok = true;
	d.svin_acc_0p1mm = 99999U;
	d.pos_valid = true;
	d.pos_acc_0p1mm = 0U;

	memset(&g_out, 0, sizeof(g_out));
	sts_web_sky_survey(&d, &g_out);
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_SURVEY_FIXED,
			       g_out.survey_state);
	TEST_ASSERT_EQUAL_UINT32(0u, g_out.survey_acc_mm);
}

static void test_nothing_stored_and_nothing_running_is_idle(void)
{
	sts_gnss_detail_t d;

	memset(&d, 0, sizeof(d));
	d.svin_seen = true; /* NAV-SVIN decoded, but no survey and no fix */

	memset(&g_out, 0xAA, sizeof(g_out));
	sts_web_sky_survey(&d, &g_out);
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_SURVEY_IDLE,
			       g_out.survey_state);
	TEST_ASSERT_FALSE(g_out.position_valid);
}

static void test_the_accuracy_unit_conversion_is_tenths_of_a_millimetre(void)
{
	/*
	 * UBX reports 0.1 mm; the REST contract carries mm. A missed conversion
	 * is a factor of ten on the one number an operator compares against
	 * `gnss.survey.acc`, and it is plausible either way.
	 */
	TEST_ASSERT_EQUAL_UINT32(0u, sts_web_0p1mm_to_mm(0U));
	TEST_ASSERT_EQUAL_UINT32(0u, sts_web_0p1mm_to_mm(4U));
	TEST_ASSERT_EQUAL_UINT32(1u, sts_web_0p1mm_to_mm(5U));
	TEST_ASSERT_EQUAL_UINT32(1u, sts_web_0p1mm_to_mm(10U));
	TEST_ASSERT_EQUAL_UINT32(100u, sts_web_0p1mm_to_mm(1000U));
	TEST_ASSERT_EQUAL_UINT32(1234u, sts_web_0p1mm_to_mm(12340U));
	TEST_ASSERT_EQUAL_UINT32(1235u, sts_web_0p1mm_to_mm(12345U));

	/*
	 * The widest value UBX can send must not wrap to something small. A
	 * naive (v + 5) / 10 reports 0 mm here — "perfectly surveyed" — for the
	 * least accurate reading the protocol can express.
	 */
	TEST_ASSERT_EQUAL_UINT32(429496730u, sts_web_0p1mm_to_mm(UINT32_MAX));
}

static void test_survey_fill_touches_only_its_own_members(void)
{
	sts_gnss_detail_t d;

	memset(&d, 0, sizeof(d));
	d.pos_valid = true;

	memset(&g_out, 0, sizeof(g_out));
	g_out.fix_type = 3U;
	g_out.n_sats = 9U;
	g_out.detail_available = true;
	g_out.ant_state = (uint8_t)REST_ANT_OK;

	sts_web_sky_survey(&d, &g_out);

	TEST_ASSERT_EQUAL_UINT(3u, g_out.fix_type);
	TEST_ASSERT_EQUAL_UINT(9u, g_out.n_sats);
	TEST_ASSERT_TRUE(g_out.detail_available);
	TEST_ASSERT_EQUAL_UINT((unsigned int)REST_ANT_OK, g_out.ant_state);
}

/* ===================================================================== */
/* the worst-case document fits the buffer the net area serves it from   */
/* ===================================================================== */

/*
 * A provider that returns the largest GNSS block the firmware can produce: a
 * full, fresh satellite list with the longest constellation name and the widest
 * numbers in every field.
 */
static int pv_worst_gnss(void *u, rest_gnss_t *out)
{
	unsigned int i;

	(void)u;
	memset(out, 0, sizeof(*out));
	out->fix_type = (uint8_t)QUALITY_GNSS_TIME_ONLY;
	out->sv_used = 32U;
	out->sv_visible = 32U;
	out->tacc_ns = UINT32_MAX;
	out->survey_state = (uint8_t)REST_SURVEY_ACTIVE;
	out->survey_dur_s = UINT32_MAX;
	out->survey_obs = UINT32_MAX;
	out->survey_acc_mm = UINT32_MAX;
	out->ecef_x_cm = INT64_MIN;
	out->ecef_y_cm = INT64_MIN;
	out->ecef_z_cm = INT64_MIN;
	out->position_valid = true;
	out->ant_state = (uint8_t)REST_ANT_UNKNOWN;
	out->ant_bias_on = true;
	out->leap_current_s = -32768;
	out->leap_pending = -128;
	out->utc_valid = true;
	out->detail_available = true;
	out->sat_age_valid = true;
	out->sat_age_ms = UINT32_MAX;
	out->n_sats = (uint8_t)REST_SAT_MAX;
	for (i = 0U; i < (unsigned int)REST_SAT_MAX; i++) {
		out->sat[i].gnss_id = 6U; /* "glonass": the longest name */
		out->sat[i].sv_id = 255U;
		out->sat[i].cno = 255U;
		out->sat[i].elev_deg = -128;
		out->sat[i].azim_deg = 359;
		out->sat[i].used = true;
	}
	memset(out->sw_version, 'X', sizeof(out->sw_version) - 1U);
	memset(out->hw_version, 'X', sizeof(out->hw_version) - 1U);
	return 0;
}

static void test_the_worst_case_gnss_block_fits_the_response_buffer(void)
{
	static char buf[STS_WEB_RESP_SIZE];
	rest_providers_t pv;
	rest_ctx_t ctx;
	web_jw_t w;
	size_t len = 0U;

	memset(&pv, 0, sizeof(pv));
	pv.gnss = pv_worst_gnss;
	memset(&ctx, 0, sizeof(ctx));
	TEST_ASSERT_EQUAL_INT(0, rest_init(&ctx, NULL, NULL, NULL, &pv));

	web_jw_init(&w, buf, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(0, rest_encode_group(&ctx, WSS_GRP_GNSS, &w));
	TEST_ASSERT_EQUAL_INT(0, web_jw_finish(&w, &len));

	/* It really is the full list, and every field is at its widest. */
	TEST_ASSERT_TRUE(len > 2000U);
	TEST_ASSERT_NOT_NULL(strstr(buf, "\"constellation\":\"glonass\""));
	TEST_ASSERT_NOT_NULL(strstr(buf, "\"sat_age_ms\":4294967295"));
	TEST_ASSERT_NOT_NULL(strstr(buf, "\"detail_available\":true"));

	/*
	 * This group alone must leave room for the six others that share the
	 * WebSocket frame. test_rest.c's test_telemetry_encoder() measures the
	 * assembled frame with every provider bound — it is the suite that owns
	 * the fakes — but the growth is entirely this group's, so the bound
	 * belongs beside the code that caused it.
	 */
	printf("worst-case gnss group: %u B of the %u B response buffer\n",
	       (unsigned int)len, (unsigned int)sizeof(buf));
	TEST_ASSERT_TRUE(len < (sizeof(buf) / 2U));
}

/* ===================================================================== */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_a_frame_that_never_arrived_is_not_fresh);
	RUN_TEST(test_the_staleness_window_is_inclusive_at_its_edge);
	RUN_TEST(test_a_frame_from_the_future_is_refused_not_wrapped);
	RUN_TEST(test_a_long_uptime_does_not_break_the_window);
	RUN_TEST(test_the_age_is_only_meaningful_when_it_is_measurable);
	RUN_TEST(test_the_readable_record_count_is_clamped_to_the_array);

	RUN_TEST(test_an_unmeasured_almanac_record_is_refused);
	RUN_TEST(test_a_satellite_in_the_solution_is_always_admitted);
	RUN_TEST(test_every_field_survives_admission);
	RUN_TEST(test_azimuth_is_folded_into_a_full_circle);
	RUN_TEST(test_a_negative_elevation_is_kept);
	RUN_TEST(test_admission_refuses_a_null_destination);

	RUN_TEST(test_no_frame_ever_is_reported_as_no_data);
	RUN_TEST(test_a_fresh_but_empty_list_is_data_not_absence);
	RUN_TEST(test_a_fresh_list_serialises_every_satellite);
	RUN_TEST(test_a_stale_list_reports_no_detail_but_keeps_its_age);
	RUN_TEST(test_the_verdict_flips_exactly_at_the_window);
	RUN_TEST(test_phantom_records_are_filtered_out_of_a_fresh_frame);
	RUN_TEST(test_a_frame_of_only_phantoms_is_fresh_and_empty);

	RUN_TEST(test_a_full_frame_is_carried_whole);
	RUN_TEST(test_a_count_past_the_cache_bound_is_clamped);
	RUN_TEST(test_a_full_frame_of_phantoms_below_the_bound_still_counts);
	RUN_TEST(test_fill_tolerates_a_missing_snapshot);
	RUN_TEST(test_fill_leaves_the_quality_derived_members_alone);
	RUN_TEST(test_an_enormous_age_saturates_rather_than_truncating);
	RUN_TEST(test_a_future_stamped_frame_is_not_served_and_has_no_age);

	RUN_TEST(test_the_admission_rule_matches_the_panels);
	RUN_TEST(test_both_views_hold_the_same_number_of_satellites);
	RUN_TEST(test_the_two_staleness_windows_are_independent_but_agree);

	RUN_TEST(test_a_short_outranks_everything_including_a_cut_bias);
	RUN_TEST(test_a_latched_short_survives_the_alarm_clearing);
	RUN_TEST(test_an_open_outranks_an_inference_from_the_time_lock);
	RUN_TEST(test_bias_commanded_off_is_unknown_not_ok);
	RUN_TEST(test_a_time_locked_receiver_on_bias_reads_ok);
	RUN_TEST(test_a_cold_start_is_unknown_not_open);
	RUN_TEST(test_unrelated_alarms_do_not_move_the_antenna_verdict);
	RUN_TEST(test_the_antenna_verdict_is_total);

	RUN_TEST(test_no_receiver_detail_reads_idle_and_invalid);
	RUN_TEST(test_a_running_survey_reports_its_own_progress);
	RUN_TEST(test_a_survey_outranks_a_position_still_on_file);
	RUN_TEST(test_a_stored_position_is_fixed_mode);
	RUN_TEST(test_a_survey_with_no_accuracy_yet_does_not_borrow_the_stored_one);
	RUN_TEST(test_a_finished_survey_does_not_lend_its_accuracy_to_a_seed);
	RUN_TEST(test_nothing_stored_and_nothing_running_is_idle);
	RUN_TEST(test_the_accuracy_unit_conversion_is_tenths_of_a_millimetre);
	RUN_TEST(test_survey_fill_touches_only_its_own_members);

	RUN_TEST(test_the_worst_case_gnss_block_fits_the_response_buffer);

	return UNITY_END();
}
