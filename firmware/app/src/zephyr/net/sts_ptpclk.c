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
 * and replace read_reference() below. Both reduce the offset uncertainty from
 * the hundreds of microseconds quantified next to sub-microsecond.
 *
 * Until then the reference is Zephyr's realtime clock plus the TAI-UTC offset
 * the GNSS engine publishes in the §3.8 quality block:
 *
 *     reference_tai_ns = CLOCK_REALTIME(UTC) + leap_current_s
 *
 * and the *uncertainty of that reference is bounded and stated*, not assumed
 * away. Two terms, both software:
 *
 *   - Sampling skew. read_reference() reads the realtime clock and the PTP
 *     clock back to back with interrupts locked, so the two reads are separated
 *     by the two register accesses only — a few hundred nanoseconds. Interrupt
 *     locking is what makes this term small; without it a preemption between
 *     the reads would inject a whole tick.
 *   - Realtime-clock granularity. Zephyr's realtime clock advances with the
 *     kernel tick and carries whatever error the setter left in it. At
 *     CONFIG_SYS_CLOCK_TICKS_PER_SEC = 1000 that is up to 1 ms, and it is the
 *     dominant term by three orders of magnitude.
 *
 * SERVO_UNCERTAINTY_NS below records that bound. It is used, not merely
 * documented: the servo refuses to slew for a measurement smaller than it,
 * because chasing noise of that size would inject it straight into the
 * hardware-timestamped path that the whole appliance exists to keep clean.
 *
 * Consequence, stated plainly so nobody reads more into the numbers than is
 * there: NTP and PTP timestamps taken off this clock have OCXO-grade
 * *stability* from the moment the servo settles, but their *absolute* accuracy
 * is limited to the millisecond class until the PPS correlation above lands.
 * The discipline engine's own lock criteria (spec §3.3) gate stratum-1
 * advertisement, so the box does not claim more than it has.
 *
 * ---------------------------------------------------------------------------
 * The loop
 * ---------------------------------------------------------------------------
 *
 * A 1 Hz PI controller on the offset, with a step/slew split:
 *
 *   |offset| >= STEP_THRESHOLD_NS  ->  ptp_clock_set(): a jump. Only ever taken
 *       at start-up or after the reference itself steps; every step is counted
 *       and logged, because a silent step is indistinguishable from a bug.
 *   otherwise                      ->  ptp_clock_adjust() with the proportional
 *       term, slew-limited to SLEW_MAX_NS_PER_S, plus an integral term folded
 *       into ptp_clock_rate_adjust(). The integrator is what absorbs a standing
 *       rate error the hardware syntonization does not cover (for instance while
 *       the OCXO loop is still pulling in), and it is clamped so a stuck
 *       reference cannot wind it into the driver's ±10 % rate bound.
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

/** Bound on the software reference's own error; see the header comment. */
#define SERVO_UNCERTAINTY_NS INT64_C(1500000) /* 1.5 ms */

/** Above this the servo steps rather than slews. */
#define STEP_THRESHOLD_NS INT64_C(20000000) /* 20 ms */

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

/** Earliest realtime-clock value treated as "somebody actually set this":
 *  2020-01-01T00:00:00Z. Zephyr's realtime clock starts at the Unix epoch. */
#define REALTIME_SANE_S INT64_C(1577836800)

static const struct device *dev;
static struct k_timer servo_timer;
static struct k_work servo_work;

static struct {
	bool clock_ok;
	bool synced;
	int64_t last_off_ns;
	int32_t rate_ppb;
	uint32_t steps;
	uint32_t slews;
	uint32_t updates;
} st;

static struct k_spinlock st_lock;

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

/**
 * Sample the reference and the PTP clock as close together as the CPU allows.
 *
 * @param out_off_ns  reference - ptp_clock, nanoseconds.
 * @retval 0        A usable measurement.
 * @retval -EAGAIN  No absolute reference yet (realtime clock unset, or the
 *                  leap offset is not known).
 * @retval <0       A driver error.
 */
static int read_reference(int64_t *out_off_ns)
{
	quality_block_t q;
	struct timespec ts;
	struct net_ptp_time tm;
	unsigned int key;
	int rc_rt;
	int rc_ptp;
	int64_t ref_ns;
	int64_t now_ns;

	if (sts_quality_snapshot(&q) != 0) {
		return -EAGAIN;
	}
	/* Without a known TAI-UTC offset the realtime clock cannot be placed on
	 * the TAI timescale at all, and guessing 37 would bake in a silent
	 * error the day it changes. */
	if (!q.utc_valid) {
		return -EAGAIN;
	}

	key = irq_lock();
	rc_rt = sys_clock_gettime(SYS_CLOCK_REALTIME, &ts);
	rc_ptp = ptp_clock_get(dev, &tm);
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

	ref_ns = ((int64_t)ts.tv_sec + (int64_t)q.leap_current_s) *
			 (int64_t)NSEC_PER_SEC_U64 +
		 (int64_t)ts.tv_nsec;
	now_ns = (int64_t)((uint64_t)tm.second * NSEC_PER_SEC_U64 +
			   (uint64_t)tm.nanosecond);

	*out_off_ns = ref_ns - now_ns;
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

static void servo_step(int64_t off_ns)
{
	struct net_ptp_time tm;
	int64_t target;

	if (ptp_clock_get(dev, &tm) != 0) {
		return;
	}
	target = (int64_t)((uint64_t)tm.second * NSEC_PER_SEC_U64 +
			   (uint64_t)tm.nanosecond) +
		 off_ns;
	if (target < 0) {
		target = 0;
	}

	tm.second = (uint64_t)target / NSEC_PER_SEC_U64;
	tm.nanosecond = (uint32_t)((uint64_t)target % NSEC_PER_SEC_U64);

	if (ptp_clock_set(dev, &tm) != 0) {
		LOG_ERR("ptp_clock_set failed");
		return;
	}

	K_SPINLOCK(&st_lock) {
		st.steps++;
	}
	sts_log(LOGR_SUB_PTP, LOGR_NOTICE, "ptp clock stepped %lld ns",
		(long long)off_ns);
}

static void servo_run(struct k_work *w)
{
	static int32_t integ_ppb;
	int64_t off_ns = 0;
	int64_t corr;
	int rc;

	ARG_UNUSED(w);

	if (dev == NULL) {
		return;
	}

	rc = read_reference(&off_ns);
	if (rc != 0) {
		K_SPINLOCK(&st_lock) {
			st.synced = false;
		}
		return;
	}

	K_SPINLOCK(&st_lock) {
		st.last_off_ns = off_ns;
		st.updates++;
		st.synced = true;
	}

	if (off_ns >= STEP_THRESHOLD_NS || off_ns <= -STEP_THRESHOLD_NS) {
		servo_step(off_ns);
		integ_ppb = 0;
		servo_apply_rate(0);
		return;
	}

	/* Inside the reference's own uncertainty there is nothing to learn, and
	 * acting anyway would pump software jitter into a hardware timescale. */
	if (off_ns < SERVO_UNCERTAINTY_NS && off_ns > -SERVO_UNCERTAINTY_NS) {
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

static void servo_tick(struct k_timer *t)
{
	ARG_UNUSED(t);
	k_work_submit(&servo_work);
}

void sts_ptpclk_stats(sts_ptpclk_stats_t *out)
{
	if (out == NULL) {
		return;
	}
	memset(out, 0, sizeof(*out));
	K_SPINLOCK(&st_lock) {
		out->clock_ok = st.clock_ok;
		out->synced = st.synced;
		out->last_off_ns = st.last_off_ns;
		out->rate_ppb = st.rate_ppb;
		out->steps = st.steps;
		out->slews = st.slews;
		out->updates = st.updates;
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
	return 0;
}
