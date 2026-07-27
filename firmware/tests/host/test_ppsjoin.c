/*
 * STS1000 "Meridian" — the PPS <-> receiver JOIN, swept across the whole second.
 *
 * tests/host/test_ppscorr.c takes `epoch_valid` as an INPUT and asks what the
 * arithmetic does with a named pulse. tests/host/test_disc.c drives
 * disc_pulse_tow_ms() and disc_qerr_matches_pulse() from a caller sitting
 * exactly on the PPS edge. Neither covers the step between them — deciding
 * WHICH second a latched capture belongs to from receiver evidence that arrives
 * at its own times — and a defect lived in precisely that gap:
 *
 *   NAV-PVT for epoch N is decoded AFTER pulse N has been captured, and
 *   UBX-TIM-TP names the NEXT pulse. So for most of every second, both of the
 *   receiver's newest records are on the wrong side of the newest capture. A
 *   consumer sitting on the PPS edge never sees this. The PTP servo is a
 *   free-running k_timer with no phase relationship to the PPS, so it sees it
 *   for whatever fraction of the second its phase happens to fall in — and
 *   because both clocks come off the PLL that this same PPS disciplines, that
 *   phase is frozen at boot to about a part in 1e9. Roughly 28 years to walk
 *   through the dead region. Whether a given unit ever gets sub-microsecond
 *   time was decided by boot luck and then stayed that way.
 *
 * So this suite does not test a function, it tests a JOIN, and it does it the
 * way the failure was found: build one second of real receiver behaviour, sweep
 * the consumer's polling phase across all 1000 milliseconds of it, and count.
 * `test_the_pairing_succeeds_at_every_phase_of_the_second` is the acceptance
 * criterion; `test_a_single_record_of_history_fails_most_of_the_second`
 * reproduces the original defect against the same model, so the harness is
 * proven able to see it.
 */

#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "disc/disc.h"

/* ------------------------------------------------------------------- model */

/*
 * One 1 Hz ZED-F9T and one board, as timestamps.
 *
 * Pulse N        fires at  base + N*1000                       (TIM2 latch)
 * NAV-PVT  (N)   decoded   base + N*1000 + pvt_lat             (names epoch N)
 * UBX-TIM-TP(N)  decoded   base + (N-1)*1000 + tp_lat          (names pulse N)
 *
 * The TIM-TP row is the one that is easy to get wrong: the receiver emits it in
 * the second BEFORE the pulse it describes, which is why the record naming
 * pulse N carries a reception timestamp one epoch earlier.
 */
typedef struct {
	uint64_t base_ms;    /* monotonic ms of pulse 0 */
	uint32_t itow0_ms;   /* GPS ToW of pulse 0 */
	uint16_t week0;      /* GPS week of pulse 0 */
	uint32_t pvt_lat_ms; /* pulse -> NAV-PVT decoded */
	uint32_t tp_lat_ms;  /* pulse -> TIM-TP (for the NEXT pulse) decoded */
	uint32_t pub_lag_ms; /* how stale the published evidence may be */
	bool     truncate32; /* stamp TIM-TP arrivals through a uint32_t */
} model_t;

#define EPOCH_MS 1000u

/* Two navigation epochs of slack, the value platform/pps.c passes. */
#define MAX_PVT_AGE_MS 2000u

static void model_defaults(model_t *m)
{
	memset(m, 0, sizeof(*m));
	/*
	 * An arbitrary but realistic uptime and epoch. 3 600 000 ms = one hour of
	 * uptime; the ToW is a Sunday-morning value well clear of the week wrap,
	 * which gets its own test.
	 */
	m->base_ms = 3600000u;
	m->itow0_ms = 259200000u;
	m->week0 = 2429u;
	/*
	 * NAV-PVT and TIM-TP land together in the receiver's once-per-second
	 * output burst. 120 ms is a plausible figure for a 1 Hz burst at this
	 * board's 460800 baud plus the GNSS thread's 50 ms drain cadence, and it
	 * is what the original defect measured out at: 120 successes in 1000.
	 */
	m->pvt_lat_ms = 120u;
	m->tp_lat_ms = 120u;
	m->pub_lag_ms = 0u;
	m->truncate32 = false;
}

/** Monotonic instant of pulse @p n. */
static uint64_t pulse_mono(const model_t *m, uint32_t n)
{
	return m->base_ms + (uint64_t)n * EPOCH_MS;
}

/** GPS ToW of pulse @p n, wrapping the week. */
static uint32_t pulse_tow(const model_t *m, uint32_t n)
{
	uint64_t tow = (uint64_t)m->itow0_ms + (uint64_t)n * EPOCH_MS;

	return (uint32_t)(tow % DISC_GPS_WEEK_MS);
}

/** GPS week of pulse @p n, incremented across each wrap. */
static uint16_t pulse_week(const model_t *m, uint32_t n)
{
	uint64_t tow = (uint64_t)m->itow0_ms + (uint64_t)n * EPOCH_MS;

	return (uint16_t)(m->week0 + (uint16_t)(tow / DISC_GPS_WEEK_MS));
}

/** A deterministic, sign-alternating sawtooth of the size an F9T really emits. */
static int32_t pulse_qerr_ps(uint32_t n)
{
	return (((n % 2u) == 0u) ? 1 : -1) * (int32_t)(3000u + (n % 11u) * 1700u);
}

/** When the record naming pulse @p n was decoded. */
static uint64_t tp_rx_mono(const model_t *m, uint32_t n)
{
	uint64_t rx = pulse_mono(m, n) - EPOCH_MS + m->tp_lat_ms;

	/*
	 * The 32-bit trap, modelled rather than described: gnssmgr_qerr_t used to
	 * carry this stamp in a uint32_t while the capture it is compared against
	 * was always 64-bit. Past 49.71 days of uptime the capture permanently
	 * exceeds UINT32_MAX, the computed lead is always ~4.29e9 ms, and the
	 * pairing — with disc's sawtooth correction behind it — is refused for the
	 * rest of that uptime.
	 */
	return m->truncate32 ? (uint64_t)(uint32_t)rx : rx;
}

/** When the NAV-PVT naming epoch @p n was decoded. */
static uint64_t pvt_rx_mono(const model_t *m, uint32_t n)
{
	return pulse_mono(m, n) + m->pvt_lat_ms;
}

/*
 * Rebuild the evidence a consumer polling at @p now_ms would be handed, exactly
 * as platform/gnss.c assembles it: the newest record of each kind that has been
 * decoded and published, plus the one it replaced, newest first.
 *
 * @p depth is how many slots the consumer is offered. 1 is the original,
 * newest-record-only pairing; 2 is what gnss.c publishes now.
 */
static void evidence_at(const model_t *m, uint64_t now_ms, size_t depth,
			disc_pvt_obs_t *pvt, disc_qerr_obs_t *qerr)
{
	/* Publication is not instantaneous: the GNSS thread republishes on its
	 * own 50 ms cadence, so a consumer can see evidence this much stale. */
	uint64_t seen_ms = (now_ms >= m->pub_lag_ms) ? (now_ms - m->pub_lag_ms) : 0u;
	uint32_t newest_pvt;
	uint32_t newest_tp;
	size_t i;

	memset(pvt, 0, depth * sizeof(*pvt));
	memset(qerr, 0, depth * sizeof(*qerr));

	/* Largest n with pvt_rx_mono(n) <= seen_ms. */
	newest_pvt = (uint32_t)((seen_ms - m->base_ms - m->pvt_lat_ms) / EPOCH_MS);
	/* Largest n with tp_rx_mono(n) <= seen_ms; the record names pulse n, and
	 * was decoded one epoch earlier, hence the +1. */
	newest_tp = (uint32_t)((seen_ms - m->base_ms - m->tp_lat_ms) / EPOCH_MS) + 1u;

	for (i = 0u; i < depth; i++) {
		uint32_t n = newest_pvt - (uint32_t)i;

		pvt[i].itow_ms = pulse_tow(m, n);
		pvt[i].rx_mono_ms = pvt_rx_mono(m, n);
		pvt[i].valid = true;
	}

	for (i = 0u; i < depth; i++) {
		uint32_t n = newest_tp - (uint32_t)i;

		qerr[i].target_tow_ms = pulse_tow(m, n);
		qerr[i].rx_mono_ms = tp_rx_mono(m, n);
		qerr[i].week = pulse_week(m, n);
		qerr[i].qerr_ps = pulse_qerr_ps(n);
		qerr[i].qerr_valid = true;
		qerr[i].valid = true;
	}
}

/** Which pulse a consumer polling at @p now_ms finds latched in TIM2. */
static uint32_t latched_pulse(const model_t *m, uint64_t now_ms)
{
	return (uint32_t)((now_ms - m->base_ms) / EPOCH_MS);
}

/* ---------------------------------------------------------------- sweeping */

typedef struct {
	unsigned int polls;
	unsigned int named;    /* the join produced a name */
	unsigned int correct;  /* ...and it was the right pulse */
	unsigned int mislabel; /* ...and it was NOT: the unforgivable outcome */
} sweep_t;

/*
 * Poll once a second for @p seconds seconds at a fixed offset into the second,
 * then repeat for every offset 0..999. That is the sweep the servo's frozen
 * phase makes: each column of it is one unit's entire service life.
 */
static void sweep(const model_t *m, size_t depth, uint32_t first_second,
		  uint32_t seconds, sweep_t *r)
{
	uint32_t phase;
	uint32_t s;

	memset(r, 0, sizeof(*r));

	for (phase = 0u; phase < EPOCH_MS; phase++) {
		for (s = 0u; s < seconds; s++) {
			disc_pvt_obs_t pvt[4];
			disc_qerr_obs_t qerr[4];
			disc_pulse_name_t name;
			uint64_t now = pulse_mono(m, first_second + s) + phase;
			uint32_t n = latched_pulse(m, now);

			TEST_ASSERT_TRUE(depth <= 4u);
			evidence_at(m, now, depth, pvt, qerr);

			r->polls++;
			if (!disc_name_pulse(pvt, depth, qerr, depth,
					     pulse_mono(m, n), MAX_PVT_AGE_MS,
					     &name)) {
				continue;
			}
			r->named++;
			if ((name.tow_ms == pulse_tow(m, n)) &&
			    (name.week == pulse_week(m, n)) &&
			    (name.qerr_ps == pulse_qerr_ps(n))) {
				r->correct++;
			} else {
				r->mislabel++;
			}
		}
	}
}

/* ============================================================ the criterion */

/*
 * THE acceptance test. One thousand polling phases, each held for ten seconds
 * as a real servo holds its own: every one of them must name the pulse, and
 * name it correctly.
 */
static void test_the_pairing_succeeds_at_every_phase_of_the_second(void)
{
	model_t m;
	sweep_t r;

	model_defaults(&m);
	sweep(&m, 2u, 8u, 10u, &r);

	printf("\n  phase sweep, depth 2: %u/%u polls named, %u correct, %u mislabelled\n",
	       r.named, r.polls, r.correct, r.mislabel);

	TEST_ASSERT_EQUAL_UINT(10000u, r.polls);
	TEST_ASSERT_EQUAL_UINT_MESSAGE(10000u, r.correct,
				       "the join must hold at every servo phase");
	TEST_ASSERT_EQUAL_UINT(0u, r.mislabel);
}

/*
 * The same sweep against the evidence depth the code used to publish. This is
 * the defect, reproduced: the pairing works only between the pulse and that
 * second's messages, and the servo's phase decides once, at boot, whether it is
 * ever in that window.
 *
 * It is asserted as a range rather than an exact count because the window is
 * `pvt_lat_ms` wide by construction and the point is the shape, not the digit.
 * If this test ever passes at 100 % the harness has stopped being able to see
 * the failure it exists to detect, and the criterion above means nothing.
 */
static void test_a_single_record_of_history_fails_most_of_the_second(void)
{
	model_t m;
	sweep_t r;

	model_defaults(&m);
	sweep(&m, 1u, 8u, 10u, &r);

	printf("  phase sweep, depth 1: %u/%u polls named, %u correct, %u mislabelled\n",
	       r.named, r.polls, r.correct, r.mislabel);

	TEST_ASSERT_EQUAL_UINT(10000u, r.polls);
	/* ~120 of every 1000 phases, i.e. the pvt_lat_ms window. */
	TEST_ASSERT_EQUAL_UINT(10u * m.pvt_lat_ms, r.correct);
	/* Never a wrong answer, even at depth 1: it degrades, it does not lie. */
	TEST_ASSERT_EQUAL_UINT(0u, r.mislabel);
}

/* ------------------------------------------------- the 49.71-day boundary */

/*
 * Past 2^32 milliseconds of uptime the capture timestamp no longer fits in 32
 * bits. With every timestamp on the join 64-bit, nothing changes.
 */
static void test_the_join_holds_past_the_32_bit_millisecond_wrap(void)
{
	model_t m;
	sweep_t r;

	model_defaults(&m);
	/* 50 days = 4 320 000 000 ms, comfortably past UINT32_MAX (4 294 967 295). */
	m.base_ms = 4320000000ull;
	TEST_ASSERT_TRUE(m.base_ms > (uint64_t)UINT32_MAX);

	sweep(&m, 2u, 8u, 10u, &r);

	TEST_ASSERT_EQUAL_UINT_MESSAGE(10000u, r.correct,
				       "49.71 days of uptime must not end the "
				       "correlation");
	TEST_ASSERT_EQUAL_UINT(0u, r.mislabel);
}

/*
 * The same run with the TIM-TP arrival stamp squeezed back through a uint32_t,
 * which is what gnssmgr_qerr_t::rx_mono_ms used to be. The lead computed
 * against a 64-bit capture is then always ~4.29e9 ms and every pairing is
 * refused — silently, permanently, and taking disc's sawtooth correction with
 * it. This test exists so the field can never be narrowed again unnoticed.
 */
static void test_a_32_bit_record_stamp_breaks_the_join_after_49_days(void)
{
	model_t m;
	sweep_t r;

	model_defaults(&m);
	m.base_ms = 4320000000ull;
	m.truncate32 = true;

	sweep(&m, 2u, 8u, 10u, &r);

	TEST_ASSERT_EQUAL_UINT_MESSAGE(0u, r.named,
				       "a truncated record stamp cannot pair "
				       "with a 64-bit capture");

	/* And below the wrap the same truncation is the identity, so it pairs. */
	model_defaults(&m);
	m.truncate32 = true;
	sweep(&m, 2u, 8u, 10u, &r);
	TEST_ASSERT_EQUAL_UINT(10000u, r.correct);
}

/* ------------------------------------------------------ the other variables */

/*
 * The receiver's output latency is not a constant of nature: it moves with
 * baud, message set and the GNSS thread's drain cadence. The join must not care
 * where in the second the burst lands, only that it lands.
 */
static void test_the_pairing_holds_across_receiver_output_latencies(void)
{
	static const uint32_t lat_ms[] = {10u, 40u, 120u, 250u, 400u, 499u};
	size_t i;

	for (i = 0u; i < (sizeof(lat_ms) / sizeof(lat_ms[0])); i++) {
		char msg[64];
		model_t m;
		sweep_t r;

		model_defaults(&m);
		m.pvt_lat_ms = lat_ms[i];
		m.tp_lat_ms = lat_ms[i];

		sweep(&m, 2u, 8u, 4u, &r);

		(void)snprintf(msg, sizeof(msg), "latency %u ms", lat_ms[i]);
		TEST_ASSERT_EQUAL_UINT_MESSAGE(4000u, r.correct, msg);
		TEST_ASSERT_EQUAL_UINT_MESSAGE(0u, r.mislabel, msg);
	}
}

/*
 * Beyond half a second the NAV-PVT epoch count rounds to the neighbouring
 * second — disc_pulse_tow_ms()'s second stated caller obligation. That is the
 * one place this join can be handed a wrong belief, so what matters is what it
 * does with one: the TIM-TP ToW comparison must refuse it. Degraded, never
 * mislabelled.
 */
static void test_an_overdue_navpvt_degrades_but_never_mislabels(void)
{
	static const uint32_t lat_ms[] = {500u, 600u, 800u, 950u};
	size_t i;

	for (i = 0u; i < (sizeof(lat_ms) / sizeof(lat_ms[0])); i++) {
		char msg[64];
		model_t m;
		sweep_t r;

		model_defaults(&m);
		m.pvt_lat_ms = lat_ms[i];
		sweep(&m, 2u, 8u, 4u, &r);

		(void)snprintf(msg, sizeof(msg), "overdue NAV-PVT %u ms",
			       lat_ms[i]);
		TEST_ASSERT_EQUAL_UINT_MESSAGE(0u, r.mislabel, msg);
		TEST_ASSERT_EQUAL_UINT_MESSAGE(r.named, r.correct, msg);
	}
}

/*
 * NAV-PVT and TIM-TP do not have to arrive together, and the join must not
 * assume an ordering between them.
 */
static void test_the_pairing_holds_when_the_two_messages_are_split(void)
{
	model_t m;
	sweep_t r;

	model_defaults(&m);
	m.pvt_lat_ms = 60u;
	m.tp_lat_ms = 430u;
	sweep(&m, 2u, 8u, 4u, &r);
	TEST_ASSERT_EQUAL_UINT(4000u, r.correct);

	model_defaults(&m);
	m.pvt_lat_ms = 430u;
	m.tp_lat_ms = 60u;
	sweep(&m, 2u, 8u, 4u, &r);
	TEST_ASSERT_EQUAL_UINT(4000u, r.correct);
	TEST_ASSERT_EQUAL_UINT(0u, r.mislabel);
}

/*
 * The evidence is republished on the GNSS thread's own 50 ms cadence, so a
 * consumer can be looking at a snapshot that predates the newest record. That
 * only ever moves it back into the regime the previous design already handled,
 * but "only ever" is a claim, so it is measured.
 */
static void test_the_pairing_tolerates_stale_published_evidence(void)
{
	static const uint32_t lag_ms[] = {0u, 1u, 25u, 50u, 99u};
	size_t i;

	for (i = 0u; i < (sizeof(lag_ms) / sizeof(lag_ms[0])); i++) {
		char msg[64];
		model_t m;
		sweep_t r;

		model_defaults(&m);
		m.pub_lag_ms = lag_ms[i];
		sweep(&m, 2u, 8u, 4u, &r);

		(void)snprintf(msg, sizeof(msg), "publish lag %u ms", lag_ms[i]);
		TEST_ASSERT_EQUAL_UINT_MESSAGE(4000u, r.correct, msg);
		TEST_ASSERT_EQUAL_UINT_MESSAGE(0u, r.mislabel, msg);
	}
}

/* A pulse either side of the GPS week rollover: ToW wraps to 0, week steps. */
static void test_the_pairing_holds_across_a_week_rollover(void)
{
	model_t m;
	sweep_t r;
	disc_pvt_obs_t pvt[2];
	disc_qerr_obs_t qerr[2];
	disc_pulse_name_t name;
	uint64_t now;

	model_defaults(&m);
	/* Pulse 20 is the first of the new week. */
	m.itow0_ms = DISC_GPS_WEEK_MS - 20000u;

	sweep(&m, 2u, 8u, 30u, &r);
	TEST_ASSERT_EQUAL_UINT(30000u, r.correct);
	TEST_ASSERT_EQUAL_UINT(0u, r.mislabel);

	/* And the wrap itself, named explicitly rather than only counted. */
	now = pulse_mono(&m, 20u) + 700u;
	evidence_at(&m, now, 2u, pvt, qerr);
	TEST_ASSERT_TRUE(disc_name_pulse(pvt, 2u, qerr, 2u, pulse_mono(&m, 20u),
					 MAX_PVT_AGE_MS, &name));
	TEST_ASSERT_EQUAL_UINT32(0u, name.tow_ms);
	TEST_ASSERT_EQUAL_UINT16(m.week0 + 1u, name.week);
}

/* ------------------------------------------------------------- refusals */

/** Build the nominal late-in-the-second evidence the refusal tests start from. */
static void nominal_late(model_t *m, disc_pvt_obs_t *pvt, disc_qerr_obs_t *qerr,
			 uint64_t *cap_mono)
{
	model_defaults(m);
	*cap_mono = pulse_mono(m, 8u);
	/* 700 ms in: both of this second's messages have arrived, so the newest
	 * record of each kind is on the wrong side of the capture. */
	evidence_at(m, *cap_mono + 700u, 2u, pvt, qerr);
}

static void test_the_fixture_itself_pairs(void)
{
	disc_pvt_obs_t pvt[2];
	disc_qerr_obs_t qerr[2];
	disc_pulse_name_t name;
	uint64_t cap;
	model_t m;

	nominal_late(&m, pvt, qerr, &cap);

	/* The premise of every refusal below: unmodified, this pairs — and it
	 * pairs off the RETAINED record, not the newest one. */
	TEST_ASSERT_TRUE(qerr[0].rx_mono_ms > cap);
	TEST_ASSERT_TRUE(qerr[1].rx_mono_ms <= cap);
	TEST_ASSERT_TRUE(pvt[0].rx_mono_ms > cap);
	TEST_ASSERT_TRUE(pvt[1].rx_mono_ms <= cap);

	TEST_ASSERT_TRUE(disc_name_pulse(pvt, 2u, qerr, 2u, cap, MAX_PVT_AGE_MS,
					 &name));
	TEST_ASSERT_EQUAL_UINT32(pulse_tow(&m, 8u), name.tow_ms);
	TEST_ASSERT_EQUAL_INT32(pulse_qerr_ps(8u), name.qerr_ps);
}

static void test_a_record_the_receiver_flagged_bad_is_not_used(void)
{
	disc_pvt_obs_t pvt[2];
	disc_qerr_obs_t qerr[2];
	disc_pulse_name_t name;
	uint64_t cap;
	model_t m;

	nominal_late(&m, pvt, qerr, &cap);
	qerr[1].qerr_valid = false;

	/*
	 * qerr_valid false means the receiver flagged qErrInvalid, or the
	 * UTC->GPS conversion was impossible. Either way the record cannot place
	 * the pulse, and there is no second-best answer to fall back to.
	 */
	TEST_ASSERT_FALSE(disc_name_pulse(pvt, 2u, qerr, 2u, cap, MAX_PVT_AGE_MS,
					  &name));
	TEST_ASSERT_FALSE(name.valid);
	TEST_ASSERT_EQUAL_UINT32(0u, name.tow_ms);
}

static void test_a_stale_record_with_an_aliasing_tow_is_refused(void)
{
	disc_pvt_obs_t pvt[2];
	disc_qerr_obs_t qerr[2];
	disc_pulse_name_t name;
	uint64_t cap;
	model_t m;

	nominal_late(&m, pvt, qerr, &cap);

	/*
	 * A week wrap, or a receiver that restarted onto the same second, can
	 * produce an old record whose ToW matches by coincidence. Age it by a
	 * week: the ToW still matches, and it must still be refused.
	 */
	qerr[1].rx_mono_ms -= (uint64_t)7u * 24u * 3600u * 1000u;
	TEST_ASSERT_FALSE(disc_name_pulse(pvt, 2u, qerr, 2u, cap, MAX_PVT_AGE_MS,
					  &name));

	/* One epoch is the whole allowance, and it is inclusive. */
	nominal_late(&m, pvt, qerr, &cap);
	qerr[1].rx_mono_ms = cap - 1000u;
	TEST_ASSERT_TRUE(disc_name_pulse(pvt, 2u, qerr, 2u, cap, MAX_PVT_AGE_MS,
					 &name));
	qerr[1].rx_mono_ms = cap - 1001u;
	TEST_ASSERT_FALSE(disc_name_pulse(pvt, 2u, qerr, 2u, cap, MAX_PVT_AGE_MS,
					  &name));
}

static void test_a_dropped_tim_tp_leaves_the_pulse_unnamed(void)
{
	disc_pvt_obs_t pvt[2];
	disc_qerr_obs_t qerr[2];
	disc_pulse_name_t name;
	uint64_t cap;
	model_t m;

	nominal_late(&m, pvt, qerr, &cap);

	/*
	 * The record naming pulse 8 never arrived, so the retained slot holds the
	 * one before it. Its ToW is a second early and its lead is two epochs:
	 * both checks refuse it, and neither the sawtooth nor the epoch may be
	 * invented from the record that IS present.
	 */
	qerr[1].target_tow_ms = pulse_tow(&m, 7u);
	qerr[1].rx_mono_ms = tp_rx_mono(&m, 7u);
	qerr[1].qerr_ps = pulse_qerr_ps(7u);

	TEST_ASSERT_FALSE(disc_name_pulse(pvt, 2u, qerr, 2u, cap, MAX_PVT_AGE_MS,
					  &name));
}

static void test_no_navpvt_means_no_independent_belief(void)
{
	disc_pvt_obs_t pvt[2];
	disc_qerr_obs_t qerr[2];
	disc_pulse_name_t name;
	uint64_t cap;
	model_t m;

	nominal_late(&m, pvt, qerr, &cap);
	pvt[0].valid = false;
	pvt[1].valid = false;

	/*
	 * The TIM-TP record alone asserts a ToW; believing it unchecked is what
	 * the ToW comparison exists to prevent. With no NAV-PVT there is nothing
	 * to compare against, so there is no name.
	 */
	TEST_ASSERT_FALSE(disc_name_pulse(pvt, 2u, qerr, 2u, cap, MAX_PVT_AGE_MS,
					  &name));
}

static void test_a_navpvt_too_old_to_attribute_is_refused(void)
{
	disc_pvt_obs_t pvt[2];
	disc_qerr_obs_t qerr[2];
	disc_pulse_name_t name;
	uint64_t cap;
	model_t m;

	nominal_late(&m, pvt, qerr, &cap);
	/* Both observations pushed beyond max_pvt_age_ms before the capture. */
	pvt[0].rx_mono_ms = cap - 5000u;
	pvt[1].rx_mono_ms = cap - 6000u;

	TEST_ASSERT_FALSE(disc_name_pulse(pvt, 2u, qerr, 2u, cap, MAX_PVT_AGE_MS,
					  &name));
}

static void test_a_capture_older_than_all_the_evidence_is_refused(void)
{
	disc_pvt_obs_t pvt[2];
	disc_qerr_obs_t qerr[2];
	disc_pulse_name_t name;
	model_t m;

	model_defaults(&m);
	/*
	 * Cold start: the receiver's first messages arrive after a pulse that was
	 * captured before any of them. Extrapolating a NAV-PVT backwards is the
	 * one thing disc_pulse_tow_ms() must never do.
	 */
	evidence_at(&m, pulse_mono(&m, 8u) + 700u, 2u, pvt, qerr);
	TEST_ASSERT_FALSE(disc_name_pulse(pvt, 2u, qerr, 2u, pulse_mono(&m, 2u),
					  MAX_PVT_AGE_MS, &name));
}

static void test_empty_and_null_evidence(void)
{
	disc_pvt_obs_t pvt[2];
	disc_qerr_obs_t qerr[2];
	disc_pulse_name_t name;
	uint64_t cap;
	model_t m;

	nominal_late(&m, pvt, qerr, &cap);

	TEST_ASSERT_FALSE(disc_name_pulse(NULL, 2u, qerr, 2u, cap, MAX_PVT_AGE_MS,
					  &name));
	TEST_ASSERT_FALSE(disc_name_pulse(pvt, 2u, NULL, 2u, cap, MAX_PVT_AGE_MS,
					  &name));
	TEST_ASSERT_FALSE(disc_name_pulse(pvt, 0u, qerr, 0u, cap, MAX_PVT_AGE_MS,
					  &name));
	TEST_ASSERT_FALSE(name.valid);
	/* NULL output is a refusal, not a crash. */
	TEST_ASSERT_FALSE(disc_name_pulse(pvt, 2u, qerr, 2u, cap, MAX_PVT_AGE_MS,
					  NULL));
}

static void test_invalid_slots_are_skipped_not_trusted(void)
{
	disc_pvt_obs_t pvt[2];
	disc_qerr_obs_t qerr[2];
	disc_pulse_name_t name;
	uint64_t cap;
	model_t m;

	/*
	 * A boot with only one TIM-TP so far: slot 1 is empty. Its zeroed
	 * contents must never be read as a record whose ToW happens to be 0.
	 */
	nominal_late(&m, pvt, qerr, &cap);
	memset(&qerr[1], 0, sizeof(qerr[1]));
	TEST_ASSERT_FALSE(disc_name_pulse(pvt, 2u, qerr, 2u, cap, MAX_PVT_AGE_MS,
					  &name));

	/* Same for the NAV-PVT history. */
	nominal_late(&m, pvt, qerr, &cap);
	memset(&pvt[1], 0, sizeof(pvt[1]));
	TEST_ASSERT_FALSE(disc_name_pulse(pvt, 2u, qerr, 2u, cap, MAX_PVT_AGE_MS,
					  &name));
}

/* ------------------------------------------------------------------- main */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_the_pairing_succeeds_at_every_phase_of_the_second);
	RUN_TEST(test_a_single_record_of_history_fails_most_of_the_second);

	RUN_TEST(test_the_join_holds_past_the_32_bit_millisecond_wrap);
	RUN_TEST(test_a_32_bit_record_stamp_breaks_the_join_after_49_days);

	RUN_TEST(test_the_pairing_holds_across_receiver_output_latencies);
	RUN_TEST(test_an_overdue_navpvt_degrades_but_never_mislabels);
	RUN_TEST(test_the_pairing_holds_when_the_two_messages_are_split);
	RUN_TEST(test_the_pairing_tolerates_stale_published_evidence);
	RUN_TEST(test_the_pairing_holds_across_a_week_rollover);

	RUN_TEST(test_the_fixture_itself_pairs);
	RUN_TEST(test_a_record_the_receiver_flagged_bad_is_not_used);
	RUN_TEST(test_a_stale_record_with_an_aliasing_tow_is_refused);
	RUN_TEST(test_a_dropped_tim_tp_leaves_the_pulse_unnamed);
	RUN_TEST(test_no_navpvt_means_no_independent_belief);
	RUN_TEST(test_a_navpvt_too_old_to_attribute_is_refused);
	RUN_TEST(test_a_capture_older_than_all_the_evidence_is_refused);
	RUN_TEST(test_empty_and_null_evidence);
	RUN_TEST(test_invalid_slots_are_skipped_not_trusted);

	return UNITY_END();
}
