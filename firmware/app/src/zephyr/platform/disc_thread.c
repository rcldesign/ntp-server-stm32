/*
 * STS1000 "Meridian" — the discipline thread (ARCHITECTURE.md §6, priority 4).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * The one thread allowed to write DAC1_OUT1 / PA4 (ARCHITECTURE.md §10.2) and
 * the sole publisher of the quality block (§10.10). Paced by the PPS capture
 * semaphore at 1 Hz with a 1.2 s timeout; the timeout is the missed-PPS path,
 * not an error path, and core/disc decides what a missed second means.
 *
 * Per second, in order:
 *   1. wait for a TIM2_CH1 capture (or time out)
 *   2. pair it with the UBX-TIM-TP sawtooth qErr for the same second
 *   3. read Vc back on PA3 and take the OCXO temperature and rail current
 *      from the housekeeping cache
 *   4. disc_tick_pps() / disc_tick_no_pps() — which publishes quality itself
 *   5. write the DAC
 *   6. step refsel and execute any handoff it decided on
 *
 * ---------------------------------------------------------------------------
 * Turning a free-running capture into a phase error
 * ---------------------------------------------------------------------------
 * core/disc wants (count, expected, ns_per_count) per channel and computes
 * wrap_diff(expected, count) — so `expected` has to be the count the *local*
 * timebase predicted for the top of second. It is maintained here as a running
 * accumulator seeded from the first capture and advanced by exactly one
 * second's worth of counts every second, whether or not that second produced a
 * usable sample. That is what makes the result a phase error rather than a
 * per-second frequency difference: re-anchoring `expected` to the previous
 * capture each second would differentiate the very quantity the PI loop's
 * proportional term needs.
 *
 * Re-anchoring is done only when the error exceeds half a second, which means
 * the timebase has lost coherence entirely (a mux handoff, a CSS event, or a
 * gap long enough that the accumulator no longer describes reality). The
 * offending sample is dropped, not fed to the loop.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "zephyr/platform/platform.h"
#include "zephyr/sts_app.h"

#include "disc/disc.h"
#include "gnssmgr/gnssmgr.h"
#include "ubx/ubx.h"
#include "quality/quality.h"
#include "refsel/refsel.h"
#include "storage/sts_store.h"

LOG_MODULE_REGISTER(sts_disc, CONFIG_STS1000_LOG_LEVEL);

#define DISC_STACK_SIZE 4096
#define DISC_PRIO       4    /* ARCHITECTURE.md §6 */
#define DISC_PPS_TIMEOUT_MS 1200

K_THREAD_STACK_DEFINE(disc_stack, DISC_STACK_SIZE);
static struct k_thread disc_tcb;

/* ---- actuator and sense ------------------------------------------------- */

static const struct device *const dac_dev = DEVICE_DT_GET(DT_NODELABEL(dac1));

#define DISC_DAC_CHANNEL   DT_PROP(STS_ZEPHYR_USER, dac_channel_id)
#define DISC_DAC_RESOLUTION DT_PROP(STS_ZEPHYR_USER, dac_resolution)

static const struct adc_dt_spec vc_sense = ADC_DT_SPEC_GET(STS_ZEPHYR_USER);

static const struct dac_channel_cfg disc_dac_cfg = {
	.channel_id = DISC_DAC_CHANNEL,
	.resolution = DISC_DAC_RESOLUTION,
	.buffered = true,
};

/* ---- module state ------------------------------------------------------- */

static disc_ctx_t disc;
static refsel_ctx_t refsel;

static struct {
	uint32_t expected_primary;
	uint32_t expected_secondary;
	bool have_expected;
	/*
	 * Monotonic time the prediction is currently anchored at, pinned to the
	 * reference's one-second grid. Kept so the accumulator can advance by the
	 * number of seconds that actually elapsed rather than by one per loop
	 * iteration: a second with no PPS costs DISC_PPS_TIMEOUT_MS (1200 ms) of
	 * real time, so advancing by exactly 1 s drifted 200 ms per missed second
	 * and after three misses forced a re-anchor that discarded the first
	 * returning sample — the one the loop most needed.
	 */
	uint64_t expected_ms;
	uint32_t counts_per_second;
	float ns_per_count;

	uint16_t last_dac_code;
	bool dac_written;

	int liveness_id;
	bool started;

	uint32_t reanchors;
	uint32_t css_recoveries;
} dt_state;

static atomic_t disc_park_req = ATOMIC_INIT(0);
static atomic_t disc_unpark_req = ATOMIC_INIT(0);

/* qErr pairing bookkeeping, owned by this thread. */
static struct {
	uint32_t applied;      /* sawtooth corrections applied */
	uint32_t unpaired;     /* captures with no qErr that belonged to them */
	uint32_t no_record;    /* captures with no TIM-TP at all */
} qerr_stats;

/* ---- Vc read-back -------------------------------------------------------- */

/*
 * VREF+ is the external MCP1502-33, which Zephyr's STM32 ADC driver models as
 * ADC_REF_INTERNAL; adc_raw_to_millivolts_dt() therefore scales against
 * vc_sense.vref_mv, populated from the devicetree.
 */
static bool disc_read_vc_mv(int32_t *mv)
{
	uint16_t sample = 0;
	struct adc_sequence seq = {
		.buffer = &sample,
		.buffer_size = sizeof(sample),
	};
	int32_t val;
	int rc;

	rc = adc_sequence_init_dt(&vc_sense, &seq);
	if (rc != 0) {
		return false;
	}

	rc = adc_read_dt(&vc_sense, &seq);
	if (rc != 0) {
		return false;
	}

	val = (int32_t)sample;
	rc = adc_raw_to_millivolts_dt(&vc_sense, &val);
	if (rc != 0) {
		return false;
	}

	*mv = val;
	return true;
}

/* ---- DAC ---------------------------------------------------------------- */

static void disc_write_dac(uint16_t code)
{
	int rc;

	if (dt_state.dac_written && code == dt_state.last_dac_code) {
		return;
	}

	rc = dac_write_value(dac_dev, DISC_DAC_CHANNEL, code);
	if (rc != 0) {
		LOG_ERR("DAC write %u failed (%d)", code, rc);
		(void)sts_alarm_set(FAULT_ALARM_DAC_FAULT, true);
		return;
	}

	dt_state.last_dac_code = code;
	dt_state.dac_written = true;

	/*
	 * Record the Vc for the PFI fast-save (storage/sts_store.h). Only the
	 * latest code matters, so noting it on every change keeps the
	 * volatile-critical set current for a power-fail that could land at any
	 * instant. Cheap no-op when the console/storage backend is absent.
	 */
	sts_store_note_dac_code(code);
}

void sts_discipline_park(void)
{
	/* ISR context (PFI). Only a flag; the thread does the work, and the
	 * DAC keeps holding its last code in the meantime, which is exactly
	 * the parked behaviour. */
	atomic_set(&disc_park_req, 1);
}

/*
 * refsel handoff bracket (ARCHITECTURE.md §3.5, Wave-2b contract). The refsel
 * action list now opens with REFSEL_ACT_PARK_DISCIPLINE and closes with
 * REFSEL_ACT_UNPARK_DISCIPLINE around the mux flip; the clock-mux executor
 * calls these on those actions. Both run in the discipline thread (the
 * executor is invoked from disc_step_refsel), so there is no concurrency
 * against the loop, and this is a *transient* park distinct from the latched
 * PFI park above — disc_unpark() resumes in RECOVERING, which absorbs the
 * HSI-bridge phase realignment with no step.
 */
void sts_disc_handoff_park(void)
{
	uint16_t code = 0;

	if (disc_park(&disc, &code) == 0) {
		disc_write_dac(code);
	}
}

void sts_disc_handoff_unpark(void)
{
	(void)disc_unpark(&disc);
}

void sts_discipline_unpark_request(void)
{
	/* Any thread. The discipline loop owns the context (ARCHITECTURE.md
	 * §10.2/§10.10), so the PFI recovery path in housekeeping asks rather than
	 * calls disc_unpark() itself. */
	atomic_set(&disc_unpark_req, 1);
}

/* ---- environment -------------------------------------------------------- */

/* UBX fix type -> the quality block's coarser enumeration (§3.8). */
static uint8_t gnss_fix_to_quality(const sts_gnss_snap_t *g)
{
	if (!g->have_status) {
		return (uint8_t)QUALITY_GNSS_NO_FIX;
	}
	if (g->time_locked) {
		return (uint8_t)QUALITY_GNSS_TIME_ONLY;
	}
	switch (g->fix_type) {
	case UBX_FIX_3D:
	case UBX_FIX_GNSS_DR:
		return (uint8_t)QUALITY_GNSS_3D;
	case UBX_FIX_2D:
		return (uint8_t)QUALITY_GNSS_2D;
	case UBX_FIX_TIME_ONLY:
		return (uint8_t)QUALITY_GNSS_TIME_ONLY;
	default:
		return (uint8_t)QUALITY_GNSS_NO_FIX;
	}
}

static void disc_fill_env(disc_env_t *env, sts_gnss_snap_t *g, uint64_t mono_ms)
{
	sts_hk_snapshot_t hk;
	int32_t vc_mv = 0;

	memset(env, 0, sizeof(*env));
	memset(g, 0, sizeof(*g));
	env->mono_ms = mono_ms;

	if (sts_hk_read(&hk) == 0) {
		env->osc_temp_mc = hk.temp_osc_mc;
		env->osc_temp_valid = hk.temp_osc_valid;

		env->ocxo_current_ua = hk.ina[INA228_RAIL_OCXO].current_ua;
		env->ocxo_current_valid = hk.ina[INA228_RAIL_OCXO].valid &&
					  hk.ina[INA228_RAIL_OCXO].cal_ok;
	}

	env->vc_sense_valid = disc_read_vc_mv(&vc_mv);
	env->vc_sense_mv = vc_mv;

	switch (refsel_state(&refsel)) {
	case REFSEL_RB_ACTIVE:
		env->anc.active_ref = QUALITY_REF_RB;
		break;
	case REFSEL_EXTREF_ACTIVE:
		env->anc.active_ref = QUALITY_REF_EXTREF;
		break;
	case REFSEL_OCXO_ACTIVE:
	default:
		env->anc.active_ref = QUALITY_REF_OCXO;
		break;
	}

	/*
	 * The GNSS half of the environment. This is the input the loop cannot run
	 * without: disc_tick_pps() takes the !gnss_time_locked branch and processes
	 * *no* PPS sample at all while it is false, so leaving it zeroed left the
	 * whole timing engine inert at ACQUIRING/stratum 16 with the DAC parked at
	 * centre, however good the pulses were.
	 */
	if (sts_gnss_snapshot(g) == 0) {
		env->gnss_time_locked = g->time_locked;
		env->anc.gnss_fix = gnss_fix_to_quality(g);
		env->anc.gnss_sv_used = g->sv_used;
		env->anc.gnss_sv_visible = g->sv_visible;
		env->anc.gnss_tacc_ns = g->tacc_ns;
		env->anc.utc_valid = g->utc_valid;
		if (g->leap_valid) {
			env->anc.leap_current_s = g->leap_current_s;
			env->anc.leap_pending = g->leap_pending;
			env->anc.leap_at_tai_s = g->leap_at_tai_s;
		}
	}

	/*
	 * Traceability is a separate question from lock. A disciplined oscillator
	 * tracking PPS is a superb frequency source and says nothing about which
	 * second it is; the served timescale only has an absolute epoch once the
	 * net area has set the PTP hardware clock from GNSS and its servo reports
	 * synchronised. Until then core/disc serves stratum UNSYNC rather than a
	 * confidently wrong timestamp.
	 */
	env->timebase_traceable = sts_time_is_traceable();
}

/* ---- reference selection ------------------------------------------------ */

static void disc_step_refsel(uint64_t mono_ms, bool switch_failed)
{
	const refsel_step_t *steps = NULL;
	refsel_in_t in;
	refsel_out_t out;
	size_t n = 0;
	uint32_t hz = 0;
	bool valid = false;
	bool edges = false;

	sts_extref_mon_read(&hz, &valid, &edges);

	memset(&in, 0, sizeof(in));
	in.mono_ms = mono_ms;
	in.extref_freq_hz = hz;
	in.extref_valid = valid;
	in.extref_edges_advancing = edges;
	in.request = REFSEL_REQ_AUTO;
	in.switch_failed = switch_failed;

	/*
	 * RB_LOCK comes from the pwrseq area rather than being read here: PB13 is a
	 * polled pin (interface ref §9) and the polarity inversion the opto
	 * introduces is a firmware bit, both of which belong with the rest of the Rb
	 * state and with the single owner of that pin. Hard-coded false, refsel
	 * could never leave the OCXO — which also made the refsel unpark
	 * unreachable and so left a PFI park permanent.
	 */
	in.rb_lock = sts_pwrseq_rb_lock();

	if (refsel_step(&refsel, &in, &out) != 0) {
		return;
	}

	if (!out.transition) {
		return;
	}

	if (refsel_actions(&refsel, &steps, &n) != 0 || n == 0U) {
		return;
	}

	LOG_INF("refsel: %s -> %s", refsel_state_name(out.from),
		refsel_state_name(out.state));

	if (sts_clkmux_execute(steps, n) != 0) {
		/*
		 * refsel_in_t::switch_failed is edge-style: assert it on the
		 * next step so the machine falls back and starts its lockout.
		 * Doing it here rather than recursing keeps one handoff per
		 * second, which is what the lockout arithmetic assumes.
		 */
		sts_log(LOGR_SUB_TIMING, LOGR_ERR,
			"reference handoff failed; reverting to OCXO");
		(void)sts_alarm_set(FAULT_ALARM_REFERENCE_LOST, true);

		memset(&in, 0, sizeof(in));
		in.mono_ms = mono_ms;
		in.request = REFSEL_REQ_AUTO;
		in.switch_failed = true;
		(void)refsel_step(&refsel, &in, &out);
	}
}

/* ---- the tick ----------------------------------------------------------- */

/* Adopt this capture as the prediction's anchor, on the current second grid. */
static void disc_anchor(const sts_pps_capture_t *cap, uint64_t mono_ms)
{
	dt_state.expected_primary = cap->tim2_cnt;
	dt_state.expected_secondary = cap->tim3_cnt;
	dt_state.expected_ms = mono_ms;
	dt_state.have_expected = true;
}

static bool disc_build_pps(const sts_pps_capture_t *cap, disc_pps_t *pps,
			   uint64_t mono_ms)
{
	uint32_t d_primary;
	uint32_t half_second = dt_state.counts_per_second / 2U;

	memset(pps, 0, sizeof(*pps));

	if (!dt_state.have_expected) {
		/* First capture of this run: adopt it as the anchor. There is
		 * no phase error to report against a prediction that does not
		 * exist yet. */
		disc_anchor(cap, mono_ms);
		return false;
	}

	/*
	 * An overcapture means a PPS edge was missed entirely, so the
	 * accumulator is one or more seconds behind the pulse just captured
	 * and the difference is not a phase error.
	 */
	if (cap->tim2_overcapture) {
		disc_anchor(cap, mono_ms);
		dt_state.reanchors++;
		return false;
	}

	d_primary = dt_state.expected_primary - cap->tim2_cnt;
	if (d_primary > half_second && d_primary < (0U - half_second)) {
		/* More than half a second either way: the timebase and the
		 * reference are no longer describing the same second. */
		disc_anchor(cap, mono_ms);
		dt_state.reanchors++;
		sts_log(LOGR_SUB_TIMING, LOGR_WARN,
			"PPS re-anchored (offset beyond half a second)");
		return false;
	}

	pps->primary_count = cap->tim2_cnt;
	pps->primary_expected = dt_state.expected_primary;
	pps->primary_ns_per_count = dt_state.ns_per_count;
	pps->primary_valid = true;

	if (cap->tim3_valid) {
		pps->secondary_count = cap->tim3_cnt;
		pps->secondary_expected = dt_state.expected_secondary;
		pps->secondary_ns_per_count = dt_state.ns_per_count;
		pps->secondary_valid = true;
	}

	return true;
}

/**
 * Advance the prediction by the reference seconds that actually elapsed.
 *
 * Not "by one second per iteration": a second with no PPS costs the loop its
 * whole DISC_PPS_TIMEOUT_MS, so one second per iteration under-counts by 200 ms
 * every time, and after three missed seconds the accumulator is more than half a
 * second out and forces a re-anchor — throwing away the first sample the
 * returning reference delivers. disc_expected_advance_s() rounds the measured
 * elapsed time to whole seconds, which keeps the prediction on the reference's
 * grid instead of feeding the loop's own scheduling jitter into the phase error.
 */
static void disc_advance_expected(uint64_t mono_ms)
{
	uint32_t secs;

	if (!dt_state.have_expected) {
		return;
	}

	secs = disc_expected_advance_s(dt_state.expected_ms, mono_ms);
	dt_state.expected_primary += secs * dt_state.counts_per_second;
	dt_state.expected_secondary += secs * dt_state.counts_per_second;
	dt_state.expected_ms += (uint64_t)secs * 1000u;
}

static void disc_handle_park(void)
{
	uint16_t code = 0;

	if (atomic_set(&disc_park_req, 0) != 0) {
		if (disc_park(&disc, &code) == 0) {
			/* The DAC already holds this value — disc_park() freezes
			 * rather than moves — so the write is a confirmation, not a
			 * change, and it is skipped by disc_write_dac()'s dedup. */
			disc_write_dac(code);
		}

		sts_log(LOGR_SUB_TIMING, LOGR_CRIT,
			"PFI: discipline parked, Vc frozen");
		(void)sts_alarm_set(FAULT_ALARM_PFI, true);
	}

	/*
	 * The recovery half. sts_pfi_service() sets this once PE8 has read
	 * de-asserted for its dwell; without it a single spurious PFI edge froze
	 * the DAC and pinned the stratum at UNSYNC until the next reboot. The
	 * prediction is dropped because the actuator was frozen across the gap, so
	 * the accumulator no longer describes the timebase.
	 */
	if (atomic_set(&disc_unpark_req, 0) != 0) {
		if (disc_unpark(&disc) == 0) {
			dt_state.have_expected = false;
			sts_log(LOGR_SUB_TIMING, LOGR_NOTICE,
				"PFI cleared: discipline resumed (recovering, "
				"retained error %.0f ns)",
				(double)disc_retained_error_ns(&disc));
		}
	}
}

/*
 * Pair the UBX-TIM-TP sawtooth with the edge just captured, by GPS time of week.
 *
 * Spec §3.2 makes this correction mandatory — uncorrected qErr dominates the
 * short-term error budget — but a correction applied to the wrong second *adds*
 * sawtooth. gnssmgr normalises target_tow_ms to GPS ToW and clears qerr_valid
 * when the leap offset needed for that conversion is unknown, so the only thing
 * left for the glue is to work out which ToW it just captured and insist on an
 * exact match.
 */
static void disc_apply_qerr(disc_pps_t *pps, const sts_gnss_snap_t *g,
			    uint64_t cap_mono_ms)
{
	disc_qerr_match_t m;
	uint32_t pulse_tow = 0;

	pps->qerr_ps = 0;
	pps->qerr_valid = false;

	if (!g->qerr.valid) {
		qerr_stats.no_record++;
		return;
	}

	memset(&m, 0, sizeof(m));
	m.record_valid = g->qerr.valid;
	m.qerr_valid = g->qerr.qerr_valid;
	m.qerr_ps = g->qerr.qerr_ps;
	m.target_tow_ms = g->qerr.target_tow_ms;
	m.record_rx_mono_ms = g->qerr.rx_mono_ms;
	m.capture_mono_ms = cap_mono_ms;

	/* Two navigation epochs of slack: the capture may land either side of the
	 * NAV-PVT that names its own second. */
	if (g->have_status &&
	    disc_pulse_tow_ms(g->pvt_itow_ms, g->pvt_rx_mono_ms, cap_mono_ms, 2000U,
			      &pulse_tow) == 0) {
		m.pulse_tow_ms = pulse_tow;
		m.pulse_tow_valid = true;
	}

	if (!disc_qerr_matches_pulse(&m)) {
		qerr_stats.unpaired++;
		return;
	}

	pps->qerr_ps = m.qerr_ps;
	pps->qerr_valid = true;
	qerr_stats.applied++;
}

static void disc_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		sts_pps_capture_t cap;
		sts_gnss_snap_t gnss;
		disc_env_t env;
		disc_out_t out;
		uint64_t mono_ms;
		bool have_pps;
		bool css;

		have_pps = (sts_pps_wait(&cap, DISC_PPS_TIMEOUT_MS) == 0);
		mono_ms = sts_mono_ms();

		disc_handle_park();

		/*
		 * A CSS event means the hardware has already dropped SYSCLK to
		 * HSI because PH0 stopped. Rebuild the tree on the OCXO before
		 * anything else this second: every measurement below is scaled
		 * by a timer clock that is currently wrong.
		 */
		css = sts_clkmux_css_fired();
		if (css) {
			dt_state.css_recoveries++;
			sts_log(LOGR_SUB_TIMING, LOGR_CRIT,
				"CSS: reference stopped, recovering on OCXO");
			(void)sts_alarm_set(FAULT_ALARM_REFERENCE_LOST, true);
			(void)sts_clkmux_recover();
			dt_state.have_expected = false;
		}

		disc_fill_env(&env, &gnss, mono_ms);

		if (have_pps) {
			disc_in_t in;
			disc_pps_t pps;

			if (disc_build_pps(&cap, &pps, cap.mono_ms)) {
				disc_apply_qerr(&pps, &gnss, cap.mono_ms);

				in.env = env;
				in.pps = pps;
				(void)disc_tick_pps(&disc, &in,
						    sts_app_quality_state(), &out);
			} else {
				(void)disc_tick_no_pps(&disc, &env,
						       sts_app_quality_state(), &out);
			}
		} else {
			(void)disc_tick_no_pps(&disc, &env, sts_app_quality_state(),
					       &out);
		}

		disc_write_dac(out.dac_code);
		/* Anchored on the capture instant when there was one, so the grid
		 * follows the reference rather than this thread's wake-up. */
		disc_advance_expected(have_pps ? cap.mono_ms : mono_ms);

		/*
		 * Record the leap state for the PFI fast-save. It rides in the
		 * ancillary block the discipline loop copies into the published
		 * quality (env.anc), sourced from gnssmgr. Cheap no-op with no
		 * storage backend.
		 */
		sts_store_note_leap(env.anc.leap_current_s,
				    (int16_t)env.anc.leap_pending,
				    gnss.leap_valid);

		disc_step_refsel(mono_ms, css);

		sts_liveness_feed(dt_state.liveness_id);
	}
}

int sts_discipline_start(void)
{
	disc_cfg_t cfg;
	k_tid_t tid;
	int rc;

	if (dt_state.started) {
		return -EALREADY;
	}

	if (!device_is_ready(dac_dev)) {
		LOG_ERR("DAC1 not ready");
		return -ENODEV;
	}
	if (!adc_is_ready_dt(&vc_sense)) {
		LOG_ERR("ADC1 (OCXO_V / PA3) not ready");
		return -ENODEV;
	}

	rc = dac_channel_setup(dac_dev, &disc_dac_cfg);
	if (rc != 0) {
		LOG_ERR("DAC1_OUT1 channel setup failed (%d)", rc);
		return rc;
	}

	rc = adc_channel_setup_dt(&vc_sense);
	if (rc != 0) {
		LOG_ERR("ADC1_INP15 channel setup failed (%d)", rc);
		return rc;
	}

	rc = disc_cfg_defaults(&cfg);
	if (rc != 0) {
		return rc;
	}
	cfg.dac_vref_mv = vc_sense.vref_mv;

	rc = disc_init(&disc, &cfg);
	if (rc != 0) {
		LOG_ERR("disc_init failed (%d)", rc);
		return rc;
	}

	rc = refsel_init(&refsel, NULL);
	if (rc != 0) {
		LOG_ERR("refsel_init failed (%d)", rc);
		return rc;
	}

	dt_state.counts_per_second = sts_pps_timer_hz();
	if (dt_state.counts_per_second == 0U) {
		LOG_ERR("discipline: PPS capture not initialised");
		return -ENODEV;
	}
	dt_state.ns_per_count = 1000000000.0f / (float)dt_state.counts_per_second;

	/*
	 * Park the actuator at the centre code before the loop runs. R121
	 * (10 M) would hold Vc at 1.65 V if PA4 were Hi-Z, but the DAC is
	 * about to drive it and must not start from an undefined output
	 * register (interface ref §3).
	 */
	disc_write_dac(cfg.dac_center_code);

	dt_state.liveness_id = sts_liveness_register("discipline");

	tid = k_thread_create(&disc_tcb, disc_stack, DISC_STACK_SIZE, disc_entry,
			      NULL, NULL, NULL, DISC_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(tid, "discipline");

	dt_state.started = true;

	LOG_INF("discipline up: %u counts/s, %.3f ns/count, DAC centre %u",
		dt_state.counts_per_second, (double)dt_state.ns_per_count,
		cfg.dac_center_code);

	return 0;
}
