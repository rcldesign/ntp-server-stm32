/*
 * STS1000 "Meridian" — ETH PTP clock: TAI time source and discipline servo.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * ---------------------------------------------------------------------------
 * What this clock is, and what disciplines it
 * ---------------------------------------------------------------------------
 *
 * The STM32H5 MAC's 1588 timestamp unit counts from HCLK, and HCLK comes from
 * the PLL fed by PH0 — the clock mux output, i.e. the OCXO or the rubidium
 * (spec §2.1, interface ref §3). So the PTP clock is *already syntonized* to
 * the disciplined reference in hardware: whatever the `discipline` thread does
 * to the OCXO's Vc, the PTP clock's rate follows. That is the whole reason the
 * board steers a physical oscillator instead of a software timescale, and it is
 * why this file does not run a frequency loop of its own by default.
 *
 * What is left is *synchronization*: the counter's absolute offset. The counter
 * powers up at zero, and something has to place it on the TAI timescale and
 * keep it there. That is this servo.
 *
 * ---------------------------------------------------------------------------
 * The references, and their honest uncertainties
 * ---------------------------------------------------------------------------
 *
 * Three, tried in order, each carrying its own stated uncertainty (ref_src_t
 * below). The first is what makes this a stratum-1 grandmaster rather than a
 * very stable clock that is wrong:
 *
 *   1. The GPS PPS edge, correlated with this counter in software
 *      (net/sts_ppscorr.h holds the arithmetic; sts_app.h's PPS capture seam
 *      supplies the capture).
 *
 *      TIM2 latches the PA0 edge in hardware, to one count — 4 ns at 250 MHz —
 *      with no software in the path. Reading TIM2 and this counter back to back
 *      with interrupts locked re-expresses that hardware-latched instant on the
 *      PTP counter's own scale, and UBX-TIM-TP names the second the pulse marks
 *      (by GPS week and time-of-week) and supplies the sawtooth qErr belonging
 *      to it. TAI = GPS + 19 s exactly, so the second this reference names does
 *      not depend on the receiver's leap bookkeeping at all.
 *
 *      Both counters hang off the same PLL, and — this is the part that makes it
 *      exact rather than merely close — the reference and the counter are
 *      compared at ONE instant. The PTP counter is never propagated across an
 *      interval, so neither the servo's own rate correction nor the driver's
 *      addend calibration enters the arithmetic. The interval that IS measured,
 *      from the edge to the paired read, is measured on TIM2, whose rate is the
 *      disciplined reference.
 *
 *      Uncertainty: PPS_UNCERTAINTY_NS. It is dominated by the paired read's own
 *      span — two timer reads bracketing three ETH register reads, a few hundred
 *      nanoseconds at 250 MHz — which is measured every tick and published as
 *      sts_ptpclk_stats_t::pps_skew_ns rather than asserted. The receiver's
 *      time-pulse accuracy after sawtooth correction and the 4 ns capture
 *      quantisation sit an order of magnitude below that. The antenna-cable and
 *      board PPS routing delay is a pure BIAS on top: it is subtracted here from
 *      `gnss.cable.ns`, which is zero — i.e. uncompensated — until §10.4 has
 *      measured it on the bench.
 *
 *      Refused unless the capture is fresh, no edge was lost before it, a
 *      UBX-TIM-TP was positively paired with that exact edge (no pairing means
 *      neither a second nor a sawtooth, and spec §3.2 makes the sawtooth
 *      mandatory), and the answer agrees with reference 2 to inside 250 ms. That
 *      last one is not belt-and-braces: a GPS week number can be wrong by a whole
 *      1024-week rollover and still look entirely plausible, and the civil-time
 *      reference is the only independent witness on the board.
 *
 *   2. The GNSS receiver's civil time, sts_gnss_wallclock():
 *
 *          reference_tai_ns = (UTC_unix_s + (TAI − UTC)) * 1e9 + nano
 *                             + (now − decode_instant)
 *
 *      The fallback that can still *place* the counter when the correlation is
 *      refused. TAI = UTC + leap is computed explicitly and the reference is
 *      refused outright unless the receiver reports the date fully resolved, the
 *      fix usable for timing, AND the leap offset known — guessing 37 would bake
 *      in a silent error the day it changes, and a half-resolved fix is worse
 *      than no fix. Reference 1 also uses it as its witness, so this check gates
 *      both.
 *
 *      Uncertainty: tens of milliseconds. The receiver emits NAV-PVT over
 *      USART3 after the second it describes, the message takes ~10 ms on the
 *      wire, and the decode-to-publish latency is neither constant nor
 *      measured. GNSS_UNCERTAINTY_NS states the bound; the servo deadbands
 *      against it rather than chasing UART jitter into a hardware-timestamped
 *      path.
 *
 *   3. Zephyr's realtime clock plus the §3.8 leap offset. Kept because it costs
 *      nothing and is an order of magnitude tighter *if* anything ever sets it.
 *      **Nothing in this tree does** (grep sys_clock_set / clock_settime), which
 *      is exactly why it cannot be the only reference: relying on it alone left
 *      read_reference() returning -EAGAIN forever, ptp_clock_set() never called,
 *      and the counter reading seconds-since-power-on while the box advertised
 *      stratum 1 (F1).
 *
 *      Uncertainty: the kernel tick, 1 ms at CONFIG_SYS_CLOCK_TICKS_PER_SEC =
 *      1000, plus whatever error the setter left in it. The two reads are taken
 *      back to back with interrupts locked, so sampling skew is a few hundred
 *      nanoseconds and negligible beside it.
 *
 * Consequence, stated plainly so nobody reads more into the numbers than is
 * there: while reference 1 is answering, NTP and PTP timestamps taken off this
 * clock are absolutely accurate to the sub-microsecond class, plus whatever
 * antenna-cable delay §10.4 has not yet calibrated out. When it is refused the
 * clock falls back to 2 and 3 and the absolute accuracy falls back with it, to
 * the tens-of-milliseconds class those references can honestly support —
 * sts_ptpclk_stats_t::ref_src says which one is live. The counter's *stability*
 * is the OCXO's either way, because it is syntonized in hardware.
 *
 * ---------------------------------------------------------------------------
 * What is still open
 * ---------------------------------------------------------------------------
 *
 * Spec §2.3/§15 name two ways to place this counter from the PPS edge and bless
 * the second as the fallback:
 *
 *   (a) the MAC's *auxiliary snapshot* input, which stamps the PPS edge in the
 *       PTP counter's own domain with no software in the path at all;
 *   (b) "correlate TIM2 capture <-> PTP-time in software".
 *
 * (b) is what reference 1 above is, and it is not a placeholder: the edge itself
 * is latched by TIM2 hardware, so software latency is out of the *measurement*
 * entirely and only the paired read's own span remains.
 *
 * (a) is still worth having, and is still open for the reason spec §15 gives.
 * The H5 silicon has the registers — stm32h563xx.h declares ETH MACATSNR /
 * MACATSSR, ETH_MACACR_ATSEN0..3 and ETH_MACTSSR_AUXTSTRIG — but neither
 * Zephyr's eth_stm32_hal driver nor the ST HAL exposes any of them, and whether
 * PA0's PPS can reach the auxiliary trigger internally is an RM0481 routing
 * question that needs hardware to answer. It would collapse the paired-read
 * span, which is the dominant term in PPS_UNCERTAINTY_NS, to zero.
 *
 * TODO(spec §15): resolve the RM0481 auxiliary-trigger routing on hardware. If
 * it routes, add an ETH PTP auxiliary-snapshot driver hook and make it a fourth,
 * preferred entry in the reference table below; reference 1 stays as its
 * fallback, because a driver hook is one more thing that can be absent.
 *
 * ---------------------------------------------------------------------------
 * Traceability, and why it is a separate question from lock
 * ---------------------------------------------------------------------------
 *
 * sts_ptpclk_traceable() is the gate every service uses before claiming a
 * primary timescale, and it is deliberately NOT derived from the discipline
 * loop. `disc` locks the oscillator's rate to the PPS and publishes stratum
 * from that; it never observes the counter's absolute offset. So a perfectly
 * locked loop on a counter that was never placed produces exquisitely stable
 * timestamps that are decades wrong, and every guard in the tree missed it:
 * sts_time_is_fallback() only reports whether a source was registered, and
 * `disc`'s served_stratum() derives stratum from PPS lock alone.
 *
 * Hence: nothing is traceable until this file has actually placed the counter
 * (epoch_set), and it stops being traceable if the reference goes away without
 * `disc` declaring holdover. Once `disc` *is* in holdover, its §3.6 policy owns
 * the demotion timetable and this file stops second-guessing it — that is the
 * whole purpose of holdover.
 *
 * ---------------------------------------------------------------------------
 * The loop
 * ---------------------------------------------------------------------------
 *
 * A 1 Hz PI controller on the offset, with a step/slew split, both thresholds
 * scaled to the active reference's own uncertainty:
 *
 *   |offset| >= src->step_ns  ->  ptp_clock_set(): a jump. Taken at start-up to
 *       place the counter, and afterwards only if the reference itself steps;
 *       every step is counted and logged, because a silent step is
 *       indistinguishable from a bug.
 *   |offset| <  src->dead_ns ->  nothing. Inside the reference's own noise
 *       there is nothing to learn and acting would pump software jitter into a
 *       hardware timescale.
 *   otherwise                ->  ptp_clock_adjust() with the proportional term,
 *       slew-limited to SLEW_MAX_NS_PER_S, plus an integral term folded into
 *       ptp_clock_rate_adjust(). The integrator absorbs a standing rate error
 *       the hardware syntonization does not cover (for instance while the OCXO
 *       loop is still pulling in), and it is clamped so a stuck reference
 *       cannot wind it into the driver's ±10 % rate bound.
 */

#include <errno.h>
#include <string.h>
#include <time.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/ptp_time.h>
#include <zephyr/sys/clock.h>

#include "cfg/cfg.h"
#include "net/sts_net.h"
#include "net/sts_ppscorr.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_ptpclk, CONFIG_STS1000_LOG_LEVEL);

/*
 * sts_ppscorr.h carries its own copy of TAI − GPS so that tests/host can
 * compile it with no Zephyr and no core dependency. sts_app.h owns the
 * definition; this is where the two are in scope together, so this is where the
 * copy is pinned to it. A 19-second error of exactly this shape has been found
 * in this project once already.
 */
BUILD_ASSERT(STS_PPSCORR_TAI_MINUS_GPS_S == STS_TAI_MINUS_GPS_S,
	     "sts_ppscorr.h's TAI-GPS copy has drifted from sts_app.h");

#define NSEC_PER_SEC_U64 UINT64_C(1000000000)

/** Bound on the realtime clock's own error; see the header comment. */
#define SERVO_UNCERTAINTY_NS INT64_C(1500000) /* 1.5 ms */

/** Above this the servo steps rather than slews, on the realtime reference. */
#define STEP_THRESHOLD_NS INT64_C(20000000) /* 20 ms */

/**
 * Bound on the GNSS civil-time reference's own error.
 *
 * NAV-PVT describes a second that has already passed, is emitted after it, and
 * takes ~10 ms to cross USART3 at 115200 8N1; neither the receiver's internal
 * latency nor the decode-to-publish delay is constant or measured here. 50 ms
 * is a deliberately pessimistic envelope: overstating it costs only servo
 * responsiveness, understating it would make the loop chase UART jitter.
 */
#define GNSS_UNCERTAINTY_NS INT64_C(50000000) /* 50 ms */

/**
 * Step threshold on the GNSS reference. Comfortably above its own noise, so a
 * noisy measurement can never be mistaken for a genuine phase step; the initial
 * placement of the counter is hundreds of millions of seconds out, so a coarse
 * threshold costs nothing there.
 */
#define GNSS_STEP_THRESHOLD_NS INT64_C(250000000) /* 250 ms */

/**
 * A GNSS reading older than this is stale: the receiver has gone quiet and the
 * age extrapolation would be doing all the work.
 */
#define GNSS_MAX_AGE_MS UINT64_C(3000)

/**
 * Bound on the correlated PPS reference's own error, and therefore the servo's
 * deadband on it.
 *
 * The measurement's jitter is the run-to-run variation of the paired read's
 * span (sts_ppscorr_out_t::skew_ns), which is a handful of register accesses
 * with interrupts locked: a few hundred nanoseconds, and the midpoint estimator
 * halves what survives of it. 250 ns is comfortably above that, so the loop does
 * not pump its own read jitter into a hardware timescale, and comfortably below
 * anything a client could notice.
 *
 * What this number does NOT cover, because it is a bias rather than an
 * uncertainty: the antenna cable and board PPS routing delay. That is
 * subtracted explicitly from `gnss.cable.ns` and is zero until §10.4 has
 * measured it.
 */
#define PPS_UNCERTAINTY_NS INT64_C(250)

/**
 * Step threshold on the correlated PPS reference.
 *
 * SLEW_MAX_NS_PER_S is 1 ms/s, so anything at or beyond a millisecond would take
 * more than a second to walk out anyway, and an offset that large against a
 * reference this tight is a discontinuity (the counter was just placed from the
 * coarse reference, a leap was applied, the mux handed over) rather than a
 * measurement.
 */
#define PPS_STEP_THRESHOLD_NS INT64_C(1000000) /* 1 ms */

/**
 * How long after the last successful reference read the clock still counts as
 * traceable while `disc` has NOT declared holdover.
 *
 * Generous relative to the 1 Hz servo so a couple of missed ticks do not flap
 * the stratum, and short enough that a silently dead reference demotes us well
 * before a client could accumulate a meaningful error against an undisciplined
 * counter.
 */
#define REF_STALE_MS UINT64_C(10000)

/** Largest phase correction applied in one 1 Hz tick. */
#define SLEW_MAX_NS_PER_S INT64_C(1000000) /* 1 ms/s */

/** Proportional gain numerator/denominator (offset -> phase correction). */
#define KP_NUM 1
#define KP_DEN 4

/** Integral gain: ns of offset per tick -> parts-per-billion of rate. */
#define KI_NUM 1
#define KI_DEN 64

/** Integrator clamp, ppb. Well inside the driver's ±10 % addend window. */
#define RATE_CLAMP_PPB 50000

/** Earliest wall-clock value treated as "somebody actually knows the date":
 *  2020-01-01T00:00:00Z. Zephyr's realtime clock starts at the Unix epoch, and
 *  a GNSS receiver reporting a pre-2020 date has not resolved anything. */
#define REALTIME_SANE_S INT64_C(1577836800)

static const struct device *dev;
static struct k_timer servo_timer;
static struct k_work servo_work;

static struct {
	bool clock_ok;
	bool synced;
	bool epoch_set;
	int64_t last_off_ns;
	int32_t rate_ppb;
	uint32_t steps;
	uint32_t slews;
	uint32_t updates;
	uint32_t no_ref;
	uint64_t last_ref_ok_ms;

	/* Which reference last answered, and how the correlated one is doing. */
	uint8_t ref_src; /* sts_ptpclk_ref_t */
	uint32_t pps_ok;
	uint32_t pps_reject;
	uint8_t pps_last_rc; /* sts_ppscorr_rc_t */
	uint32_t pps_skew_ns;
	int64_t pps_age_ns;
	int64_t pps_coarse_resid_ns;
} st;

static struct k_spinlock st_lock;

/*
 * Conservative default for the platform-owned GNSS time interface. See the
 * contract at sts_gnss_wallclock_t in sts_net.h: a strong definition in the
 * platform area overrides this with no change here, and until one exists the
 * appliance simply has no absolute reference — which the traceability gate
 * turns into an honest stratum 16 / clockClass 248 rather than a confident
 * wrong answer.
 */
__weak int sts_gnss_wallclock(sts_gnss_wallclock_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));
	return -ENODATA;
}

const struct device *sts_ptpclk_dev(void)
{
	return dev;
}

int sts_ptpclk_tai_ns(uint64_t *out_ns)
{
	struct net_ptp_time tm;
	int rc;

	if (out_ns == NULL) {
		return -EINVAL;
	}
	if (dev == NULL) {
		return -ENODEV;
	}

	rc = ptp_clock_get(dev, &tm);
	if (rc != 0) {
		return rc;
	}

	*out_ns = ((uint64_t)tm.second * NSEC_PER_SEC_U64) +
		  (uint64_t)tm.nanosecond;
	return 0;
}

uint64_t sts_ptpclk_ts_to_tai_ns(uint64_t seconds, uint32_t nanoseconds)
{
	/* eth_stm32_hal marks an absent RX stamp with UINT64_MAX/UINT32_MAX. */
	if (seconds == UINT64_MAX || nanoseconds == UINT32_MAX ||
	    nanoseconds >= (uint32_t)NSEC_PER_SEC_U64) {
		return 0U;
	}
	return (seconds * NSEC_PER_SEC_U64) + (uint64_t)nanoseconds;
}

/** sts_app.h TAI source shim. */
static int tai_source(void *ctx, uint64_t *out_ns)
{
	ARG_UNUSED(ctx);
	return sts_ptpclk_tai_ns(out_ns);
}

/* ------------------------------------------------------------------------- */
/* servo                                                                     */
/* ------------------------------------------------------------------------- */

/** Which reference produced a measurement, and how much to trust it. */
typedef struct {
	const char *name;
	uint8_t id;      /* sts_ptpclk_ref_t, for the telemetry snapshot */
	int64_t dead_ns; /* below this, do nothing: it is the reference's own noise */
	int64_t step_ns; /* at or above this, step rather than slew */
} ref_src_t;

/*
 * The table, in preference order. read_reference() walks it top-down; each entry
 * is refused rather than degraded, so falling through to the next one is the
 * only way a weaker reference is ever used.
 */
static const ref_src_t ref_pps = {
	.name = "pps",
	.id = STS_PTPCLK_REF_PPS,
	.dead_ns = PPS_UNCERTAINTY_NS,
	.step_ns = PPS_STEP_THRESHOLD_NS,
};

static const ref_src_t ref_gnss = {
	.name = "gnss",
	.id = STS_PTPCLK_REF_GNSS,
	.dead_ns = GNSS_UNCERTAINTY_NS,
	.step_ns = GNSS_STEP_THRESHOLD_NS,
};

static const ref_src_t ref_realtime = {
	.name = "realtime",
	.id = STS_PTPCLK_REF_REALTIME,
	.dead_ns = SERVO_UNCERTAINTY_NS,
	.step_ns = STEP_THRESHOLD_NS,
};

/** Read the PTP counter as TAI nanoseconds, or return the driver's error. */
static int ptp_now_ns(int64_t *out_ns)
{
	struct net_ptp_time tm;
	int rc = ptp_clock_get(dev, &tm);

	if (rc != 0) {
		return rc;
	}
	*out_ns = (int64_t)((uint64_t)tm.second * NSEC_PER_SEC_U64 +
			    (uint64_t)tm.nanosecond);
	return 0;
}

/**
 * The GNSS receiver's civil time, projected onto TAI nanoseconds.
 *
 * @retval 0        @p out_ref_ns is a usable absolute reference.
 * @retval -EAGAIN  No receiver, or the fix cannot place a timescale.
 */
static int gnss_reference(int64_t *out_ref_ns)
{
	sts_gnss_wallclock_t w;
	uint64_t now_ms;
	int64_t ref_ns;

	if (sts_gnss_wallclock(&w) != 0) {
		return -EAGAIN;
	}
	/*
	 * Every one of these is a refusal to guess. Without utc_valid the date
	 * is not fully resolved; without leap_valid TAI − UTC is unknown and
	 * assuming today's 37 would bake in a silent error the day it changes;
	 * without time_locked the receiver itself does not consider the fix good
	 * enough for timing.
	 */
	if (!w.utc_valid || !w.leap_valid || !w.time_locked) {
		return -EAGAIN;
	}
	if (w.utc_unix_s < REALTIME_SANE_S) {
		return -EAGAIN;
	}

	now_ms = sts_mono_ms();
	if (w.mono_ms > now_ms || (now_ms - w.mono_ms) > GNSS_MAX_AGE_MS) {
		/* Stale, or stamped in the future: either way not a reference. */
		return -EAGAIN;
	}

	/* TAI = UTC + (TAI − UTC), explicitly, then advanced by the age of the
	 * reading so the comparison is against *now* and not the decode instant. */
	ref_ns = (w.utc_unix_s + (int64_t)w.tai_minus_utc) *
			 (int64_t)NSEC_PER_SEC_U64 +
		 (int64_t)w.utc_nano_ns;
	ref_ns += (int64_t)(now_ms - w.mono_ms) * INT64_C(1000000);

	*out_ref_ns = ref_ns;
	return 0;
}

/**
 * Zephyr's realtime clock plus the §3.8 leap offset, projected onto TAI ns.
 *
 * Sampled with interrupts locked against the PTP counter so the two reads are
 * separated by the register accesses alone; without the lock a preemption
 * between them would inject a whole kernel tick.
 */
static int realtime_reference(int64_t *out_ref_ns, int64_t *out_now_ns)
{
	quality_block_t q;
	struct timespec ts;
	unsigned int key;
	int rc_rt;
	int rc_ptp;
	int64_t now_ns = 0;

	if (sts_quality_snapshot(&q) != 0) {
		return -EAGAIN;
	}
	if (!q.utc_valid) {
		return -EAGAIN;
	}

	key = irq_lock();
	rc_rt = sys_clock_gettime(SYS_CLOCK_REALTIME, &ts);
	rc_ptp = ptp_now_ns(&now_ns);
	irq_unlock(key);

	if (rc_rt != 0) {
		return -EAGAIN;
	}
	if (rc_ptp != 0) {
		return rc_ptp;
	}
	if (ts.tv_sec < REALTIME_SANE_S) {
		return -EAGAIN;
	}

	*out_ref_ns = ((int64_t)ts.tv_sec + (int64_t)q.leap_current_s) *
			      (int64_t)NSEC_PER_SEC_U64 +
		      (int64_t)ts.tv_nsec;
	*out_now_ns = now_ns;
	return 0;
}

/** Record what the last correlation attempt did, for telemetry and the log. */
static void pps_note(sts_ppscorr_rc_t rc, const sts_ppscorr_out_t *res)
{
	K_SPINLOCK(&st_lock) {
		st.pps_last_rc = (uint8_t)rc;
		st.pps_skew_ns = res->skew_ns;
		st.pps_age_ns = res->edge_age_ns;
		st.pps_coarse_resid_ns = res->coarse_resid_ns;
		if (rc == STS_PPSCORR_OK) {
			st.pps_ok++;
		} else {
			st.pps_reject++;
		}
	}
}

/**
 * The GPS PPS edge, correlated with this counter.
 *
 * The measurement the whole file exists for: TIM2 holds the PA0 edge latched in
 * hardware, this reads TIM2 and the PTP counter back to back with interrupts
 * locked, and net/sts_ppscorr.h turns the pair into an offset. See that header
 * for the arithmetic and for every condition under which it refuses.
 *
 * @param coarse_ok  The civil-time reference is usable (its independent witness).
 * @param coarse_ns  That reference, TAI ns.
 * @param out_off_ns reference - ptp_clock, nanoseconds.
 *
 * @retval 0        @p out_off_ns is a usable measurement.
 * @retval -EAGAIN  Refused; sts_ptpclk_stats_t::pps_last_rc says why.
 * @retval <0       A driver error reading the PTP counter.
 */
static int pps_reference(bool coarse_ok, int64_t coarse_ns, int64_t *out_off_ns)
{
	sts_pps_epoch_t cap;
	sts_ppscorr_in_t in;
	sts_ppscorr_out_t res;
	sts_ppscorr_rc_t rc;
	int64_t ptp_ns = 0;
	unsigned int key;
	int prc;

	if (sts_pps_epoch_get(&cap) != 0) {
		/*
		 * No PPS capture path in this build, or it has not started. Still
		 * recorded as a refusal: the reason field is what note_ref_src()
		 * prints when the servo falls back, and leaving it holding a
		 * previous tick's verdict — or the zero-initialised STS_PPSCORR_OK
		 * — would annotate the fallback with a lie.
		 */
		memset(&res, 0, sizeof(res));
		pps_note(STS_PPSCORR_NO_CAPTURE, &res);
		return -EAGAIN;
	}

	memset(&in, 0, sizeof(in));
	in.seq = cap.seq;
	in.tim2_cnt = cap.tim2_cnt;
	in.timer_hz = cap.timer_hz;
	in.cap_mono_ms = cap.mono_ms;
	in.gps_week = cap.gps_week;
	in.gps_tow_ms = cap.gps_tow_ms;
	in.qerr_ps = cap.qerr_ps;
	in.epoch_valid = cap.epoch_valid;
	in.overcapture = cap.overcapture;

	/*
	 * The same §3.2 conditioning core/disc applies to a phase sample.
	 * qerr_sign has no cfg key — disc_cfg_defaults() fixes it at +1 — and the
	 * cable delay is `gnss.cable.ns`, which is a §10.4 bench calibration and
	 * reads 0 until somebody measures it. Zero is the honest default: a wrong
	 * non-zero delay would be worse than none.
	 */
	in.qerr_sign = 1;
	in.cable_delay_ns = sts_net_cfg_i32(CFG_ID_GNSS_CABLE_DELAY_NS, 0);

	in.coarse_valid = coarse_ok;
	in.coarse_tai_ns = coarse_ns;

	/*
	 * The pairing. Nothing between the three reads but the reads themselves:
	 * a preemption here would put a whole scheduling quantum between the two
	 * timebases, and the span that remains is measured (tim2_a..tim2_b) rather
	 * than assumed, so a stall shows up as a refusal instead of as error.
	 *
	 * sts_mono_ms() is taken inside the lock, after the measurement, so the
	 * millisecond age and the TIM2 age describe the same instant — that
	 * cross-check is what detects a 32-bit TIM2 wrap, and a preemption between
	 * the two reads would make it fire spuriously.
	 */
	key = irq_lock();
	in.tim2_a = sts_pps_counter_now();
	prc = ptp_now_ns(&ptp_ns);
	in.tim2_b = sts_pps_counter_now();
	in.now_mono_ms = sts_mono_ms();
	irq_unlock(key);

	if (prc != 0) {
		return prc;
	}
	in.ptp_ns = ptp_ns;

	rc = sts_ppscorr_eval(&in, &res);
	pps_note(rc, &res);
	if (rc != STS_PPSCORR_OK) {
		return -EAGAIN;
	}

	*out_off_ns = res.off_ns;
	return 0;
}

/**
 * Sample the best available reference and the PTP clock as close together as
 * the CPU allows.
 *
 * @param out_off_ns  reference - ptp_clock, nanoseconds.
 * @param out_src     Receives which reference answered.
 * @retval 0        A usable measurement.
 * @retval -EAGAIN  No absolute reference: no usable GNSS fix, and Zephyr's
 *                  realtime clock is unset or the leap offset is unknown.
 * @retval <0       A driver error.
 */
static int read_reference(int64_t *out_off_ns, const ref_src_t **out_src)
{
	int64_t coarse_ns = 0;
	int64_t ref_ns = 0;
	int64_t now_ns = 0;
	bool coarse_ok;
	int rc;

	/*
	 * Read the civil-time reference once, up front. The correlated reference
	 * needs it as its independent witness and the fallback needs it as its
	 * measurement, and taking two snapshots a few microseconds apart would
	 * mean the witness and the fallback could disagree with each other.
	 */
	coarse_ok = (gnss_reference(&coarse_ns) == 0);

	/* 1. the PPS edge, correlated against this counter. */
	rc = pps_reference(coarse_ok, coarse_ns, out_off_ns);
	if (rc == 0) {
		*out_src = &ref_pps;
		return 0;
	}

	/* 2. the receiver's civil time, on its own. */
	if (coarse_ok) {
		rc = ptp_now_ns(&now_ns);
		if (rc != 0) {
			return rc;
		}
		*out_off_ns = coarse_ns - now_ns;
		*out_src = &ref_gnss;
		return 0;
	}

	/* 3. Zephyr's realtime clock. */
	rc = realtime_reference(&ref_ns, &now_ns);
	if (rc != 0) {
		return rc;
	}
	*out_off_ns = ref_ns - now_ns;
	*out_src = &ref_realtime;
	return 0;
}

static void servo_apply_rate(int32_t ppb)
{
	double ratio;

	if (ppb > RATE_CLAMP_PPB) {
		ppb = RATE_CLAMP_PPB;
	} else if (ppb < -RATE_CLAMP_PPB) {
		ppb = -RATE_CLAMP_PPB;
	}

	ratio = 1.0 + ((double)ppb * 1e-9);
	if (ptp_clock_rate_adjust(dev, ratio) != 0) {
		LOG_WRN("rate_adjust(%d ppb) refused", ppb);
		return;
	}

	K_SPINLOCK(&st_lock) {
		st.rate_ppb = ppb;
	}
}

/**
 * Place the counter at reference time.
 *
 * @return true when the counter is now on the reference's timescale.
 */
static bool servo_step(int64_t off_ns, const char *src)
{
	struct net_ptp_time tm;
	int64_t now_ns = 0;
	int64_t target;

	if (ptp_now_ns(&now_ns) != 0) {
		return false;
	}
	target = now_ns + off_ns;
	if (target < 0) {
		target = 0;
	}

	memset(&tm, 0, sizeof(tm));
	tm.second = (uint64_t)target / NSEC_PER_SEC_U64;
	tm.nanosecond = (uint32_t)((uint64_t)target % NSEC_PER_SEC_U64);

	if (ptp_clock_set(dev, &tm) != 0) {
		LOG_ERR("ptp_clock_set failed");
		return false;
	}

	K_SPINLOCK(&st_lock) {
		st.steps++;
	}
	sts_log(LOGR_SUB_PTP, LOGR_NOTICE,
		"ptp clock stepped %lld ns from the %s reference",
		(long long)off_ns, src);
	return true;
}

/** Record that the counter is on TAI, announcing the first time only. */
static void mark_epoch_set(const char *src)
{
	bool first = false;

	K_SPINLOCK(&st_lock) {
		first = !st.epoch_set;
		st.epoch_set = true;
	}
	if (first) {
		sts_log(LOGR_SUB_PTP, LOGR_NOTICE,
			"ptp clock placed on TAI from the %s reference; served "
			"time is now traceable", src);
	}
}

/**
 * Announce a change of reference.
 *
 * Worth a log line every time: which reference is answering is the difference
 * between served time that is accurate to a few hundred nanoseconds and served
 * time that is accurate to tens of milliseconds, and nothing else on the box
 * makes that visible. The refusal reason is carried too — "fell back to gnss"
 * with no cause is the kind of entry that costs an afternoon on the bench.
 */
static void note_ref_src(const ref_src_t *src)
{
	static uint8_t last_id = STS_PTPCLK_REF_NONE;
	uint8_t id = (src != NULL) ? src->id : (uint8_t)STS_PTPCLK_REF_NONE;
	uint8_t rc;

	if (id == last_id) {
		return;
	}
	last_id = id;

	if (src == NULL) {
		/* Recorded, not logged: servo_run() already announces the loss of
		 * traceability, and the reference is reported every tick that has
		 * one, so the next acquisition is what needs the line. */
		return;
	}

	K_SPINLOCK(&st_lock) {
		rc = st.pps_last_rc;
	}

	if (id == (uint8_t)STS_PTPCLK_REF_PPS) {
		sts_log(LOGR_SUB_PTP, LOGR_NOTICE,
			"ptp clock reference is now the correlated GPS PPS edge "
			"(sub-microsecond)");
	} else {
		sts_log(LOGR_SUB_PTP, LOGR_NOTICE,
			"ptp clock reference is now %s; the correlated PPS edge "
			"was refused (%s)", src->name,
			sts_ppscorr_rc_name((sts_ppscorr_rc_t)rc));
	}
}

static void servo_iterate(void)
{
	static int32_t integ_ppb;
	const ref_src_t *src = NULL;
	int64_t off_ns = 0;
	int64_t corr;
	int rc;

	if (dev == NULL) {
		return;
	}

	rc = read_reference(&off_ns, &src);
	if (rc != 0) {
		note_ref_src(NULL);
		K_SPINLOCK(&st_lock) {
			st.synced = false;
			st.no_ref++;
			st.ref_src = (uint8_t)STS_PTPCLK_REF_NONE;
		}
		return;
	}

	note_ref_src(src);

	K_SPINLOCK(&st_lock) {
		st.last_off_ns = off_ns;
		st.updates++;
		st.synced = true;
		st.ref_src = src->id;
		st.last_ref_ok_ms = sts_mono_ms();
	}

	if (off_ns >= src->step_ns || off_ns <= -src->step_ns) {
		bool placed = servo_step(off_ns, src->name);

		/*
		 * A successful step is what puts the counter on TAI, and it is
		 * the *only* thing that may set this flag: everything the
		 * appliance advertises about its timescale hangs off it (F1).
		 */
		if (placed) {
			mark_epoch_set(src->name);
		}
		integ_ppb = 0;
		servo_apply_rate(0);
		return;
	}

	/*
	 * The measurement agrees with the counter to inside a step, so the
	 * counter is already on the reference's timescale — either we placed it
	 * earlier, or it happened to be right. Either way it is placed now.
	 */
	mark_epoch_set(src->name);

	/* Inside the reference's own uncertainty there is nothing to learn, and
	 * acting anyway would pump software jitter into a hardware timescale. */
	if (off_ns < src->dead_ns && off_ns > -src->dead_ns) {
		return;
	}

	corr = (off_ns * KP_NUM) / KP_DEN;
	if (corr > SLEW_MAX_NS_PER_S) {
		corr = SLEW_MAX_NS_PER_S;
	} else if (corr < -SLEW_MAX_NS_PER_S) {
		corr = -SLEW_MAX_NS_PER_S;
	}

	if (corr != 0) {
		if (ptp_clock_adjust(dev, (int)corr) == 0) {
			K_SPINLOCK(&st_lock) {
				st.slews++;
			}
		}
	}

	integ_ppb += (int32_t)((off_ns * KI_NUM) / (KI_DEN * 1000));
	if (integ_ppb > RATE_CLAMP_PPB) {
		integ_ppb = RATE_CLAMP_PPB;
	} else if (integ_ppb < -RATE_CLAMP_PPB) {
		integ_ppb = -RATE_CLAMP_PPB;
	}
	servo_apply_rate(integ_ppb);
}

/**
 * One servo tick, then publish the traceability verdict.
 *
 * The publish is the other half of the F1 fix. This file gates what *this area*
 * advertises, but core/disc derives the served stratum independently and used to
 * do it from PPS lock alone — a locked loop on a counter that was never placed
 * on TAI still yielded stratum 1. So the platform grew disc_env_t::
 * timebase_traceable, fed from sts_time_is_traceable(), and this servo is the
 * one thing on the board that knows the answer. Both transitions are published,
 * every tick, so `disc` demotes on the way down as promptly as it promotes on
 * the way up; publishing only the acquisition would leave a stale primary claim
 * standing after the reference was lost.
 */
static void servo_run(struct k_work *w)
{
	static bool published;
	static bool published_valid;
	bool now_traceable;

	ARG_UNUSED(w);

	servo_iterate();

	now_traceable = sts_ptpclk_traceable();
	if (!published_valid || now_traceable != published) {
		published = now_traceable;
		published_valid = true;
		sts_time_set_traceable(now_traceable);
		sts_log(LOGR_SUB_PTP, LOGR_NOTICE,
			"served timescale is %s",
			now_traceable ? "traceable" : "NOT traceable");
	}
}

static void servo_tick(struct k_timer *t)
{
	ARG_UNUSED(t);
	k_work_submit(&servo_work);
}

bool sts_ptpclk_traceable(void)
{
	quality_block_t q;
	uint64_t now_ms = sts_mono_ms();
	uint64_t last_ok;
	bool ok;

	K_SPINLOCK(&st_lock) {
		ok = st.clock_ok && st.epoch_set;
		last_ok = st.last_ref_ok_ms;
	}
	if (!ok) {
		/* The counter has never been placed on TAI. Whatever it reads is
		 * time since power-on, and no amount of oscillator lock changes
		 * that — this is the hard gate. */
		return false;
	}

	if (last_ok != 0U && now_ms >= last_ok &&
	    (now_ms - last_ok) <= REF_STALE_MS) {
		return true;
	}

	/*
	 * The reference has gone quiet. That is exactly the condition holdover
	 * exists for, and §3.6 makes `disc` the owner of how long traceability
	 * survives it — including the demotion of stratum, which this file must
	 * not duplicate or contradict. So defer to it while it says holdover, and
	 * only report untraceable when the reference vanished without `disc`
	 * noticing, which is a fault rather than a policy.
	 */
	if (sts_quality_snapshot(&q) == 0 && q.holdover) {
		return true;
	}
	return false;
}

void sts_ptpclk_stats(sts_ptpclk_stats_t *out)
{
	if (out == NULL) {
		return;
	}
	memset(out, 0, sizeof(*out));
	out->traceable = sts_ptpclk_traceable();
	K_SPINLOCK(&st_lock) {
		out->clock_ok = st.clock_ok;
		out->synced = st.synced;
		out->epoch_set = st.epoch_set;
		out->last_off_ns = st.last_off_ns;
		out->rate_ppb = st.rate_ppb;
		out->steps = st.steps;
		out->slews = st.slews;
		out->updates = st.updates;
		out->no_ref = st.no_ref;
		out->ref_src = st.ref_src;
		out->pps_ok = st.pps_ok;
		out->pps_reject = st.pps_reject;
		out->pps_last_rc = st.pps_last_rc;
		out->pps_skew_ns = st.pps_skew_ns;
		out->pps_age_ns = st.pps_age_ns;
		out->pps_coarse_resid_ns = st.pps_coarse_resid_ns;
	}
}

int sts_ptpclk_init(void)
{
	struct net_if *iface = sts_net_iface();

	if (iface == NULL) {
		return -ENODEV;
	}

	dev = net_eth_get_ptp_clock(iface);
	if (dev == NULL) {
		LOG_ERR("MAC exposes no PTP clock; NTP/PTP hardware "
			"timestamping is unavailable");
		return -ENODEV;
	}
	if (!device_is_ready(dev)) {
		LOG_ERR("PTP clock %s not ready", dev->name);
		dev = NULL;
		return -ENODEV;
	}

	st.clock_ok = true;
	/* Before the first servo tick nothing has been correlated, and 0 would
	 * read as STS_PPSCORR_OK. Say so explicitly. */
	st.pps_last_rc = (uint8_t)STS_PPSCORR_NO_CAPTURE;
	st.ref_src = (uint8_t)STS_PTPCLK_REF_NONE;
	sts_time_register_source(tai_source, NULL);
	LOG_INF("PTP clock %s bound as the TAI source", dev->name);
	return 0;
}

int sts_ptpclk_start(void)
{
	if (dev == NULL) {
		return -ENODEV;
	}

	k_work_init(&servo_work, servo_run);
	k_timer_init(&servo_timer, servo_tick, NULL);
	k_timer_start(&servo_timer, K_MSEC(500), K_MSEC(1000));
	LOG_INF("PTP-clock servo started; the counter is NOT on TAI until an "
		"absolute reference places it, and nothing is served as "
		"traceable before then");
	return 0;
}
