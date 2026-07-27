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
#include "quality/quality.h"
#include "refsel/refsel.h"

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
}

void sts_discipline_park(void)
{
	/* ISR context (PFI). Only a flag; the thread does the work, and the
	 * DAC keeps holding its last code in the meantime, which is exactly
	 * the parked behaviour. */
	atomic_set(&disc_park_req, 1);
}

/* ---- environment -------------------------------------------------------- */

static void disc_fill_env(disc_env_t *env, uint64_t mono_ms)
{
	sts_hk_snapshot_t hk;
	int32_t vc_mv = 0;

	memset(env, 0, sizeof(*env));
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

	/*
	 * Ancillary GNSS/reference fields. gnssmgr lives in the net-adjacent
	 * gnss thread and is not wired yet; until it is, the block stays at
	 * its zeroed "nothing known" state, which core/disc reads as no fix
	 * and no UTC — the correct pre-service answer.
	 * TODO(wave-3b): populate from gnssmgr_status() once the gnss thread
	 * publishes it, and set active_ref from refsel_state().
	 */
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
	 * RB_LOCK arrives through the housekeeping snapshot rather than being
	 * read here: PB13 is a polled pin (interface ref §9) and the polarity
	 * inversion the opto introduces is a config bit, both of which belong
	 * with the rest of the Rb state.
	 * TODO(wave-3b): in.rb_lock = hk.rb_lock once pwrseq publishes it.
	 */

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

static bool disc_build_pps(const sts_pps_capture_t *cap, disc_pps_t *pps)
{
	uint32_t d_primary;
	uint32_t half_second = dt_state.counts_per_second / 2U;

	memset(pps, 0, sizeof(*pps));

	if (!dt_state.have_expected) {
		/* First capture of this run: adopt it as the anchor. There is
		 * no phase error to report against a prediction that does not
		 * exist yet. */
		dt_state.expected_primary = cap->tim2_cnt;
		dt_state.expected_secondary = cap->tim3_cnt;
		dt_state.have_expected = true;
		return false;
	}

	/*
	 * An overcapture means a PPS edge was missed entirely, so the
	 * accumulator is one or more seconds behind the pulse just captured
	 * and the difference is not a phase error.
	 */
	if (cap->tim2_overcapture) {
		dt_state.expected_primary = cap->tim2_cnt;
		dt_state.expected_secondary = cap->tim3_cnt;
		dt_state.reanchors++;
		return false;
	}

	d_primary = dt_state.expected_primary - cap->tim2_cnt;
	if (d_primary > half_second && d_primary < (0U - half_second)) {
		/* More than half a second either way: the timebase and the
		 * reference are no longer describing the same second. */
		dt_state.expected_primary = cap->tim2_cnt;
		dt_state.expected_secondary = cap->tim3_cnt;
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

/** Advance the prediction by one second, whatever happened this tick. */
static void disc_advance_expected(void)
{
	if (!dt_state.have_expected) {
		return;
	}

	dt_state.expected_primary += dt_state.counts_per_second;
	dt_state.expected_secondary += dt_state.counts_per_second;
}

static void disc_handle_park(void)
{
	uint16_t code = 0;

	if (atomic_set(&disc_park_req, 0) == 0) {
		return;
	}

	if (disc_park(&disc, &code) == 0) {
		/* The DAC already holds this value — disc_park() freezes
		 * rather than moves — so the write is a confirmation, not a
		 * change, and it is skipped by disc_write_dac()'s dedup. */
		disc_write_dac(code);
	}

	sts_log(LOGR_SUB_TIMING, LOGR_CRIT, "PFI: discipline parked, Vc frozen");
	(void)sts_alarm_set(FAULT_ALARM_PFI, true);
}

static void disc_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		sts_pps_capture_t cap;
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

		disc_fill_env(&env, mono_ms);

		if (have_pps) {
			disc_in_t in;
			disc_pps_t pps;
			gnssmgr_qerr_t qerr;

			if (disc_build_pps(&cap, &pps)) {
				/*
				 * Sawtooth correction is mandatory (spec §3.2):
				 * uncorrected qErr dominates the short-term
				 * error budget. gnssmgr pairs TIM-TP with the
				 * pulse by time-of-week, so the value fetched
				 * here belongs to the edge just captured.
				 * TODO(wave-3b): source `g` from the gnss
				 * thread; until then qErr is absent and disc
				 * runs uncorrected, which is correct-but-noisy
				 * rather than wrong.
				 */
				memset(&qerr, 0, sizeof(qerr));
				pps.qerr_ps = qerr.qerr_ps;
				pps.qerr_valid = qerr.qerr_valid;

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
		disc_advance_expected();

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
