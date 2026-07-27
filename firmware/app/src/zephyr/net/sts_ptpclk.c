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
 * The reference, and its honest uncertainty
 * ---------------------------------------------------------------------------
 *
 * Spec §2.3 and §15 name the right answer: capture the GPS PPS with the MAC's
 * *auxiliary snapshot* input, which stamps the PPS edge in the PTP counter's
 * own domain with no software in the path. Two things stand between this file
 * and that:
 *
 *   1. RM0481's auxiliary-trigger routing for the H5 is the open item in
 *      spec §15 — whether PPS can reach ETH_PTP_AUX_TS internally at all.
 *   2. TIM2 (PA0) already captures the PPS, but TIM2 belongs to the platform
 *      area and sts_app.h exposes no capture hook, so the software correlation
 *      the spec names as the fallback ("correlate TIM2 capture <-> PTP-time in
 *      software") cannot be built from inside this area today.
 *
 * TODO(spec §15 / cross-area): land one of
 *   (a) an ETH PTP auxiliary-snapshot driver hook, or
 *   (b) an sts_app.h capture callback carrying {TIM2 capture, PPS TAI second},
 * and add it as a third, preferred entry in the reference table below. Both
 * reduce the offset uncertainty from the tens of milliseconds quantified next
 * to sub-microsecond.
 *
 * Until then there are two software references, tried in order, each carrying
 * its own stated uncertainty (ref_src_t below):
 *
 *   1. The GNSS receiver's civil time, sts_gnss_wallclock():
 *
 *          reference_tai_ns = (UTC_unix_s + (TAI − UTC)) * 1e9 + nano
 *                             + (now − decode_instant)
 *
 *      This is the only reference that can *place* the counter, because it is
 *      the only one that knows the date. TAI = UTC + leap is computed
 *      explicitly and the reference is refused outright unless the receiver
 *      reports the date fully resolved, the fix usable for timing, AND the leap
 *      offset known — guessing 37 would bake in a silent error the day it
 *      changes, and a half-resolved fix is worse than no fix.
 *
 *      Uncertainty: tens of milliseconds. The receiver emits NAV-PVT over
 *      USART3 after the second it describes, the message takes ~10 ms on the
 *      wire, and the decode-to-publish latency is neither constant nor
 *      measured. GNSS_UNCERTAINTY_NS states the bound; the servo deadbands
 *      against it rather than chasing UART jitter into a hardware-timestamped
 *      path.
 *
 *   2. Zephyr's realtime clock plus the §3.8 leap offset. Kept because it costs
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
 * there: NTP and PTP timestamps taken off this clock have OCXO-grade
 * *stability* from the moment the servo settles, but their *absolute* accuracy
 * is limited to the tens-of-milliseconds class until the PPS correlation above
 * lands.
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

#include "net/sts_net.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_ptpclk, CONFIG_STS1000_LOG_LEVEL);

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
	int64_t dead_ns; /* below this, do nothing: it is the reference's own noise */
	int64_t step_ns; /* at or above this, step rather than slew */
} ref_src_t;

static const ref_src_t ref_gnss = {
	.name = "gnss",
	.dead_ns = GNSS_UNCERTAINTY_NS,
	.step_ns = GNSS_STEP_THRESHOLD_NS,
};

static const ref_src_t ref_realtime = {
	.name = "realtime",
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
	int64_t ref_ns = 0;
	int64_t now_ns = 0;
	int rc;

	rc = gnss_reference(&ref_ns);
	if (rc == 0) {
		rc = ptp_now_ns(&now_ns);
		if (rc != 0) {
			return rc;
		}
		*out_off_ns = ref_ns - now_ns;
		*out_src = &ref_gnss;
		return 0;
	}

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
		K_SPINLOCK(&st_lock) {
			st.synced = false;
			st.no_ref++;
		}
		return;
	}

	K_SPINLOCK(&st_lock) {
		st.last_off_ns = off_ns;
		st.updates++;
		st.synced = true;
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
