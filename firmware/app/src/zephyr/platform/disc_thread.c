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
#include "stats/phase_rec.h"
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

	/*
	 * The code FIRMWARE last wanted on PA4, kept apart from the code the pin
	 * actually holds. While a maintenance override is in force the two differ,
	 * and this is what the release re-drives — "firmware's automatic level"
	 * for a parked loop being the frozen code, not zero and not the centre.
	 */
	uint16_t auto_code;
	bool auto_valid;

	/* disc_cfg_t's actuator transfer, cached at init so the mailbox drain
	 * and the published read-back convert without reaching into `disc`
	 * while a tick is running. Write-once, before the thread starts. */
	uint16_t dac_vref_mv;
	uint16_t dac_max_code;

	int liveness_id;
	bool started;

	uint32_t reanchors;
	uint32_t css_recoveries;
} dt_state;

static atomic_t disc_park_req = ATOMIC_INIT(0);
static atomic_t disc_unpark_req = ATOMIC_INIT(0);

/*
 * ---------------------------------------------------------------------------
 * The two park latches, and why there are two
 * ---------------------------------------------------------------------------
 * Both are owned by this thread; nothing else writes them.
 *
 * `disc_pfi_parked` tracks the LATCHED power-fail park above: set when the PFI
 * request lands, cleared when sts_pfi_service() reports the rail back.
 *
 * `disc_maint_parked` tracks the operator's `ref.disc.park` lease. It is a latch
 * too — deliberately, and not the transient bracket sts_disc_handoff_park()
 * opens below. The bracket exists to span ONE mux flip and closes itself within
 * the same refsel action list; a maintenance park has to outlive the RPC that
 * asked for it and stay closed across every DAC write the technician then makes,
 * because MP_ILK_DAC_PARK is re-evaluated on each of them.
 *
 * They are separate flags rather than one counter because the two directions
 * must not defeat each other. A maintenance release while PFI still holds the
 * rail down would resume steering into a browning-out board; a PFI recovery
 * while a technician holds a DAC override would resume the loop underneath that
 * override and fight it for PA4. disc_resume_if_idle() therefore resumes only
 * when BOTH are clear, which is the one rule both directions need.
 */
static bool disc_pfi_parked;
static bool disc_maint_parked;

/*
 * The maintenance override of DAC1_OUT1, owned by this thread.
 *
 * `dac_ovr_row` is the mailbox row holding it (STS_PWRSEQ_REQ_OCXO_VC_MV or
 * _OCXO_DAC_CODE). Recording WHICH row owns the pin is what lets the two views
 * of one actuator have a row each without racing: a release arriving on the row
 * that does not own the override re-drives nothing, so the order the two rows
 * drain in cannot decide the value left on the pin.
 */
static bool dac_ovr_active;
static uint8_t dac_ovr_row;

/*
 * What PA4 is actually holding, published for the console's read-back and for
 * MP_ILK_DAC_SOLE / MP_ILK_DAC_IDLE.
 *
 * One aligned word, written only by this thread: code in the low 16 bits,
 * "an override is in force" in bit 16. The same argument ref_override_req below
 * makes — a single word IS the whole synchronisation requirement, and a mutex
 * shared with a priority-12 reader would put a management plane on the timing
 * path (ARCHITECTURE.md §10 invariant 10).
 */
#define DISC_DACPUB_OVR BIT(16)
static atomic_t disc_dac_pub = ATOMIC_INIT(0);

/*
 * The operator's request to clear core/refsel's sticky flap latch.
 *
 * A flag rather than a call, for exactly the reason the park requests above are
 * flags: this thread owns `refsel` and refsel_clear_flap() writes five of its
 * fields, so a console command calling it directly would mutate the state
 * machine between refsel_step() and refsel_actions() — i.e. while an action list
 * built from the old state is still being executed.
 */
static atomic_t ref_flap_clear_req = ATOMIC_INIT(0);

/*
 * The operator's standing reference request (sts_app.h sts_ref_req_t), read by
 * this thread once a second as refsel_in_t::request.
 *
 * One atomic word, no lock on either side. This thread is priority 4 and owns
 * the DAC and the reference state machine; a mutex shared with a priority-12
 * web worker would put a management plane on the timing path, which
 * ARCHITECTURE.md §10 invariant 10 forbids outright. A single aligned word is
 * all the state there is, so an atomic is not a shortcut — it is the whole
 * synchronisation requirement.
 */
static atomic_t ref_request = ATOMIC_INIT((atomic_val_t)STS_REF_REQ_AUTO);

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
	atomic_set(&disc_dac_pub,
		   (atomic_val_t)code |
			   (dac_ovr_active ? (atomic_val_t)DISC_DACPUB_OVR : 0));

	/*
	 * Record the Vc for the PFI fast-save (storage/sts_store.h). Only the
	 * latest code matters, so noting it on every change keeps the
	 * volatile-critical set current for a power-fail that could land at any
	 * instant. Cheap no-op when the console/storage backend is absent.
	 */
	sts_store_note_dac_code(code);
}

/**
 * Drive the code FIRMWARE wants, and remember it.
 *
 * Every automatic writer — the loop, both park paths, the initial centre —
 * goes through here, and the maintenance override in disc_service_mailbox()
 * is the only caller of disc_write_dac() directly. That split is what makes
 * the override stick: without it the loop re-writes disc_out_t::dac_code every
 * second and undoes a technician's Vc within one PPS period, because a parked
 * loop keeps emitting the code it froze at.
 *
 * The remembered value is also the release target, so "return to
 * firmware-automatic control" has a definite answer at the instant the lease
 * lapses rather than one that arrives on the next tick.
 */
static void disc_apply_auto_dac(uint16_t code)
{
	dt_state.auto_code = code;
	dt_state.auto_valid = true;

	if (dac_ovr_active) {
		return;
	}
	disc_write_dac(code);
}

void sts_discipline_park(void)
{
	/* ISR context (PFI). Only a flag; the thread does the work, and the
	 * DAC keeps holding its last code in the meantime, which is exactly
	 * the parked behaviour. */
	atomic_set(&disc_park_req, 1);
}

bool sts_disc_dac_state(uint16_t *out_code, int32_t *out_mv, bool *out_ovr)
{
	atomic_val_t v = atomic_get(&disc_dac_pub);
	uint16_t code = (uint16_t)(v & 0xFFFF);

	if (!dt_state.started) {
		return false;
	}
	if (out_code != NULL) {
		*out_code = code;
	}
	if (out_mv != NULL) {
		*out_mv = disc_code_to_mv(dt_state.dac_vref_mv,
					  dt_state.dac_max_code, code);
	}
	if (out_ovr != NULL) {
		*out_ovr = ((v & DISC_DACPUB_OVR) != 0);
	}
	return true;
}

/**
 * Resume steering, but only if NOTHING is still holding the loop down.
 *
 * The single gate every unpark path goes through. Each holder clears its own
 * latch and then asks; whoever clears the last one is the one that actually
 * resumes. Without it the three parks are three independent claims on one
 * state variable and the last release wins regardless of who still needs it —
 * which is how a PFI recovery would resume the loop under a technician's DAC
 * override, and how releasing that override's park would resume it into a
 * browning-out rail.
 *
 * @retval true   Nothing holds it and disc_unpark() accepted (which is also
 *                its answer when the loop was not parked at all).
 */
static bool disc_resume_if_idle(void)
{
	if (disc_pfi_parked || disc_maint_parked) {
		return false;
	}
	return disc_unpark(&disc) == 0;
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
		disc_apply_auto_dac(code);
	}
}

void sts_disc_handoff_unpark(void)
{
	/*
	 * The bracket's own close. It goes through the same gate as every other
	 * resume: a mux flip that completed while the PFI latch or an operator's
	 * maintenance park was standing must not resume steering just because
	 * refsel is finished with the pin.
	 */
	disc_resume_if_idle();
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

uint8_t sts_ref_override_get(void)
{
	return (uint8_t)atomic_get(&ref_request);
}

int sts_ref_override_set(uint8_t mode)
{
	if (mode >= (uint8_t)STS_REF_REQ__COUNT) {
		/* An out-of-range request must not select a reference. refsel's
		 * own default branch maps anything unrecognised to "want the
		 * rubidium", so letting a bad REST value through would engage
		 * input B on a typo. */
		return -EINVAL;
	}

	if (sts_ref_override_get() == mode) {
		/* Idempotent on purpose. refsel.h: changing the request restarts
		 * the guard-debounce window, so a management plane that re-POSTs
		 * or polls its own control must not be able to hold the machine
		 * off its reference for as long as it keeps asking. */
		return 0;
	}

	(void)atomic_set(&ref_request, (atomic_val_t)mode);

	sts_log(LOGR_SUB_TIMING, LOGR_NOTICE,
		"reference request set to %s by operator",
		(mode == (uint8_t)STS_REF_REQ_OCXO)     ? "force-ocxo"
		: (mode == (uint8_t)STS_REF_REQ_EXTREF) ? "extref"
							: "auto");

	return 0;
}

/**
 * The operator request as core/refsel spells it.
 *
 * An explicit mapping rather than a cast: the two enumerations agree
 * numerically today and nothing keeps them that way, and the failure mode of a
 * silent divergence is the wrong 10 MHz feeding PH0. Anything unrecognised
 * becomes AUTO — the safe answer, and the one that cannot reach
 * REFSEL_EXTREF_ACTIVE, which refsel.h says is never entered automatically.
 */
static refsel_request_t disc_refsel_request(void)
{
	switch (sts_ref_override_get()) {
	case (uint8_t)STS_REF_REQ_OCXO:
		return REFSEL_REQ_FORCE_OCXO;
	case (uint8_t)STS_REF_REQ_EXTREF:
		return REFSEL_REQ_EXTREF;
	case (uint8_t)STS_REF_REQ_AUTO:
	default:
		return REFSEL_REQ_AUTO;
	}
}

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
	in.request = disc_refsel_request();
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
		/* Same standing request: this re-step exists to deliver the
		 * switch_failed edge, not to change what the operator asked for.
		 * Passing AUTO here would have restarted the guard window under a
		 * different target the moment a handoff failed. */
		in.request = disc_refsel_request();
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
		disc_pfi_parked = true;
		if (disc_park(&disc, &code) == 0) {
			/* The DAC already holds this value — disc_park() freezes
			 * rather than moves — so the write is a confirmation, not a
			 * change, and it is skipped by disc_write_dac()'s dedup. */
			disc_apply_auto_dac(code);
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
		disc_pfi_parked = false;
		if (disc_maint_parked) {
			/* The operator's park outlives the power-fail one. Said
			 * out loud, because "PFI cleared" with the loop still
			 * parked is otherwise an unexplained stratum. */
			sts_log(LOGR_SUB_TIMING, LOGR_NOTICE,
				"PFI cleared: maintenance park still held, Vc "
				"frozen");
		} else if (disc_unpark(&disc) == 0) {
			dt_state.have_expected = false;
			sts_log(LOGR_SUB_TIMING, LOGR_NOTICE,
				"PFI cleared: discipline resumed (recovering, "
				"retained error %.0f ns)",
				(double)disc_retained_error_ns(&disc));
		}
	}
}

/* ---- maintenance mailbox (sts_pwrseq_req.h, foreign rows) --------------- */

/**
 * Drop any maintenance override and put PA4 back where firmware wants it.
 *
 * @return the row that had been holding it, or STS_PWRSEQ_REQ_NONE.
 */
static uint8_t disc_drop_dac_override(void)
{
	uint8_t row = dac_ovr_row;

	if (!dac_ovr_active) {
		return (uint8_t)STS_PWRSEQ_REQ_NONE;
	}

	dac_ovr_active = false;
	dac_ovr_row = (uint8_t)STS_PWRSEQ_REQ_NONE;
	if (dt_state.auto_valid) {
		disc_write_dac(dt_state.auto_code);
	} else {
		/* Nothing automatic has ever been driven, so there is no level
		 * to go back to; re-publish what the pin holds so the override
		 * flag stops claiming a lease that is gone. */
		atomic_set(&disc_dac_pub,
			   (atomic_val_t)dt_state.last_dac_code);
	}
	return row;
}

/** Settle @p row and stamp it with the code PA4 now holds. */
static void disc_settle_dac(uint8_t row, bool applied, uint8_t err, bool as_mv)
{
	int32_t v = as_mv ? disc_code_to_mv(dt_state.dac_vref_mv,
					    dt_state.dac_max_code,
					    dt_state.last_dac_code)
			  : (int32_t)dt_state.last_dac_code;

	(void)sts_pwrseq_req_settle(row, applied, err, v,
				    (uint32_t)k_uptime_get_32());
}

/** Execute one claimed Vc row. @p as_mv selects which of the two views it is. */
static void disc_service_dac_row(uint8_t row, bool release, int32_t value,
				 bool as_mv)
{
	uint16_t code;

	if (release) {
		/*
		 * Only the OWNING row puts the pin back. The two views share one
		 * actuator, and MP_ILK_DAC_SOLE means only one of them can hold
		 * a lease — but a lapsed lease on the other view still arrives
		 * here, and letting it re-drive the automatic level would undo
		 * an override it never took. Never refused: a release that could
		 * fail is a dead-man that cannot fire.
		 */
		if (dac_ovr_active && (dac_ovr_row == row)) {
			(void)disc_drop_dac_override();
		}
		disc_settle_dac(row, true, (uint8_t)STS_PWRSEQ_REQ_ERR_NONE,
				as_mv);
		return;
	}

	/*
	 * The park is re-checked HERE, not only at the interlock. MP_ILK_DAC_PARK
	 * is evaluated on the console thread at grant time; between that and this
	 * drain the loop can have resumed — a refsel handoff closing its bracket,
	 * or a PFI recovery — and a Vc write into a running loop is a control that
	 * reports success and is overwritten within the second.
	 */
	if (disc_state(&disc) != DISC_STATE_PARKED) {
		disc_settle_dac(row, false, (uint8_t)STS_PWRSEQ_REQ_ERR_STAGE,
				as_mv);
		return;
	}

	if (as_mv) {
		if (disc_mv_to_code(dt_state.dac_vref_mv, dt_state.dac_max_code,
				    value, &code) != 0) {
			disc_settle_dac(row, false,
					(uint8_t)STS_PWRSEQ_REQ_ERR_HW, as_mv);
			return;
		}
	} else {
		if (value < 0) {
			value = 0;
		}
		if (value > (int32_t)dt_state.dac_max_code) {
			value = (int32_t)dt_state.dac_max_code;
		}
		code = (uint16_t)value;
	}

	dac_ovr_active = true;
	dac_ovr_row = row;
	disc_write_dac(code);
	disc_settle_dac(row, true, (uint8_t)STS_PWRSEQ_REQ_ERR_NONE, as_mv);
}

/** Execute one claimed `ref.disc.park` request. */
static void disc_service_park_row(bool release, int32_t value)
{
	bool want = !release && (value != 0);
	uint16_t code = 0;
	uint8_t dropped;

	if (want) {
		disc_maint_parked = true;
		if (disc_park(&disc, &code) == 0) {
			disc_apply_auto_dac(code);
		}
		sts_log(LOGR_SUB_TIMING, LOGR_NOTICE,
			"maintenance: discipline parked, Vc frozen at %u",
			(unsigned int)dt_state.last_dac_code);
		(void)sts_pwrseq_req_settle(
			(uint8_t)STS_PWRSEQ_REQ_DISC_PARK, true,
			(uint8_t)STS_PWRSEQ_REQ_ERR_NONE, 1,
			(uint32_t)k_uptime_get_32());
		return;
	}

	/*
	 * The release direction, and the ordering hazard it closes.
	 *
	 * MP_ILK_DAC_IDLE refuses a DELIBERATE `ref.disc.park = 0` while either
	 * Vc object is leased, so a technician who asks for this in the wrong
	 * order is told so. It cannot cover the INVOLUNTARY release: a lease that
	 * lapses reaches obj_apply() with a NULL value and no interlock is
	 * evaluated on that path. Refusing here instead would be worse than the
	 * hazard — the park lease is already gone on the console side, so a
	 * refusal would strand the loop parked with nothing left able to release
	 * it.
	 *
	 * So the override is dropped WITH the park, and the row that held it is
	 * settled REFUSED so mp_veto() withdraws that lease with a stated reason.
	 * The technician loses the override they were told they could not keep,
	 * and hears why; what they never get is an unparked loop and a live
	 * override both driving PA4.
	 */
	dropped = disc_drop_dac_override();
	if (dropped != (uint8_t)STS_PWRSEQ_REQ_NONE) {
		disc_settle_dac(dropped, false,
				(uint8_t)STS_PWRSEQ_REQ_ERR_STAGE,
				dropped == (uint8_t)STS_PWRSEQ_REQ_OCXO_VC_MV);
		sts_log(LOGR_SUB_TIMING, LOGR_WARN,
			"maintenance: Vc override withdrawn with the park");
	}

	disc_maint_parked = false;
	if (disc_resume_if_idle()) {
		dt_state.have_expected = false;
	}
	sts_log(LOGR_SUB_TIMING, LOGR_NOTICE,
		"maintenance: park released (%s)",
		disc_pfi_parked ? "PFI still holds it" : "discipline resumed");
	(void)sts_pwrseq_req_settle((uint8_t)STS_PWRSEQ_REQ_DISC_PARK, true,
				    (uint8_t)STS_PWRSEQ_REQ_ERR_NONE,
				    disc_pfi_parked ? 1 : 0,
				    (uint32_t)k_uptime_get_32());
}

/**
 * Drain this thread's foreign mailbox rows. **discipline-thread context only.**
 *
 * Claim, execute, settle — pwrseq_service_mailbox()'s discipline, reaching the
 * queue through sts_pwrseq_req_claim()/_settle(), which refuse every row this
 * area does not own.
 *
 * THE ORDER IS DELIBERATE: both Vc rows before the park row.
 *
 * A lease lapse can drop a Vc lease and the park lease in the same window. Taken
 * park-first, the park release would find the override still standing, withdraw
 * it as above, and report a veto for a lease that was expiring on its own — the
 * technician would be told their override was revoked when in fact it simply ran
 * out. Taken Vc-first the override is already gone by the time the park release
 * runs, there is nothing to withdraw, and the veto path stays for the case it
 * describes. The grant direction needs no such care: MP_ILK_DAC_PARK will not
 * pass a Vc grant until the park has landed AND been published, so the two can
 * never be granted inside one window.
 */
static void disc_service_mailbox(void)
{
	static const struct {
		uint8_t row;
		bool as_mv;
	} vc_rows[] = {
		{ (uint8_t)STS_PWRSEQ_REQ_OCXO_VC_MV, true },
		{ (uint8_t)STS_PWRSEQ_REQ_OCXO_DAC_CODE, false },
	};
	int32_t value = 0;
	bool release = false;
	size_t i;

	for (i = 0U; i < ARRAY_SIZE(vc_rows); i++) {
		if (sts_pwrseq_req_claim(vc_rows[i].row, &release, &value)) {
			disc_service_dac_row(vc_rows[i].row, release, value,
					     vc_rows[i].as_mv);
		}
	}

	if (sts_pwrseq_req_claim((uint8_t)STS_PWRSEQ_REQ_DISC_PARK, &release,
				 &value)) {
		disc_service_park_row(release, value);
	}
}

/**
 * Apply a pending operator request to clear the reference-flap latch.
 *
 * Runs at the top of this thread's loop, before disc_step_refsel() builds the
 * next input — so a cleared latch is visible to the same pass rather than one
 * second later, and the clear can never land between refsel_step() and the
 * execution of the action list it produced.
 */
static void disc_handle_ref_requests(void)
{
	if (atomic_set(&ref_flap_clear_req, 0) == 0) {
		return;
	}

	if (refsel_clear_flap(&refsel) == 0) {
		sts_log(LOGR_SUB_TIMING, LOGR_NOTICE,
			"operator cleared the reference-flap latch");
	} else {
		sts_log(LOGR_SUB_TIMING, LOGR_WARN,
			"reference-flap clear refused: refsel is not initialised");
	}
}

int sts_ref_clear_flap(void)
{
	if (!dt_state.started) {
		return -ENODEV;
	}
	atomic_set(&ref_flap_clear_req, 1);
	return 0;
}

uint32_t sts_clock_css_events(void)
{
	/*
	 * The counter itself lives in clkmux.c, which owns the CSS NMI handler.
	 * This is the seam rather than a second counter: platform.h is private to
	 * this area (ARCHITECTURE.md §2), so the console cannot ask clkmux.c
	 * directly, and a duplicate count kept here would drift the moment the
	 * NMI fired between the two increments.
	 */
	return sts_clkmux_css_events();
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

/* ---- phase-record export (MCP PHASE_EXPORT) ----------------------------- */

/*
 * The bench operator's window into the ADEV phase ring, encoded as a
 * core/stats phase record (stats/phase_rec.h).
 *
 * WHY IT IS PRODUCED BY THIS THREAD AND NOT THE CONSOLE'S
 *
 * disc_ctx_t is owned by the discipline thread. Reading its 2 KB phase ring
 * from the console thread would race disc_tick_pps() — the ring shifts down by
 * one every second — and the obvious fix, a mutex over the loop state, is
 * exactly what ARCHITECTURE.md 10 and the firmware CLAUDE.md forbid: no
 * console/web/SNMP/UI thread may hold a lock on timing state.
 *
 * So the console asks and this thread answers. A request sets `phase_req`; the
 * bottom of the per-second pass notices it, encodes the record into
 * `phase_blob` and posts `phase_done`. Nothing is encoded when nobody is
 * asking, so the steady-state cost on the timing path is one atomic read per
 * second.
 *
 * The consequence the caller must live with is latency: an export waits up to
 * one PPS period (plus the missed-PPS timeout) for the next pass. That is the
 * right trade for a bench capture, and it is bounded below by the fact that
 * this thread keeps ticking through holdover and missed pulses.
 */
#define PHASE_BLOB_CAP 4200u
#define PHASE_WAIT_MS  4000u

static K_MUTEX_DEFINE(phase_mutex);
static K_SEM_DEFINE(phase_done, 0, 1);
static atomic_t phase_req;

static uint8_t phase_blob[PHASE_BLOB_CAP];
static uint32_t phase_blob_len;
static int phase_blob_rc = -ENODATA;

BUILD_ASSERT(PHASE_BLOB_CAP >= (24u + (DISC_ADEV_CAP / 8u) +
				(DISC_ADEV_CAP * 8u)),
	     "phase_blob too small for a full DISC_ADEV_CAP record");

/* Runs on the discipline thread. Encodes the ring into phase_blob. */
static void disc_publish_phase(void)
{
	static disc_phase_snap_t snap;
	static int64_t x_ps[DISC_ADEV_CAP];
	phase_rec_meta_t meta;
	size_t len = 0u;
	size_t i;
	int rc;

	rc = disc_phase_snapshot(&disc, &snap);
	if (rc == 0) {
		for (i = 0u; i < (size_t)snap.n; i++) {
			x_ps[i] = phase_rec_ns_f_to_ps(snap.x_ns[i]);
		}

		meta.ver = PHASE_REC_VER;
		meta.flags = snap.full ? PHASE_REC_F_FULL : 0u;
		meta.n = snap.n;
		meta.tau0_ns = snap.tau0_ns;
		meta.gaps = snap.gaps;
		meta.mono_ms = sts_mono_ms();

		rc = phase_rec_encode(&meta, x_ps, snap.gapmap, phase_blob,
				      sizeof(phase_blob), &len);
	}

	(void)k_mutex_lock(&phase_mutex, K_FOREVER);
	phase_blob_rc = rc;
	phase_blob_len = (rc == 0) ? (uint32_t)len : 0u;
	(void)k_mutex_unlock(&phase_mutex);
}

/* Runs on the discipline thread, once per pass. */
static void disc_service_phase_req(void)
{
	if (atomic_cas(&phase_req, 1, 0)) {
		disc_publish_phase();
		k_sem_give(&phase_done);
	}
}

int sts_disc_phase_begin(uint32_t *out_len)
{
	int rc;

	if (out_len == NULL) {
		return -EINVAL;
	}
	if (!dt_state.started) {
		return -ENODEV;
	}

	/* Drop any completion left by an abandoned request so the wait below
	 * cannot be satisfied by a stale post and hand out the previous
	 * snapshot as if it were fresh. */
	k_sem_reset(&phase_done);
	atomic_set(&phase_req, 1);

	if (k_sem_take(&phase_done, K_MSEC(PHASE_WAIT_MS)) != 0) {
		atomic_set(&phase_req, 0);
		return -ETIMEDOUT;
	}

	(void)k_mutex_lock(&phase_mutex, K_FOREVER);
	rc = phase_blob_rc;
	*out_len = (rc == 0) ? phase_blob_len : 0u;
	(void)k_mutex_unlock(&phase_mutex);

	return rc;
}

int sts_disc_phase_read(uint32_t off, uint8_t *buf, uint32_t cap,
			uint32_t *out_n)
{
	uint32_t n;

	if ((buf == NULL) || (out_n == NULL)) {
		return -EINVAL;
	}

	(void)k_mutex_lock(&phase_mutex, K_FOREVER);
	if (phase_blob_rc != 0) {
		(void)k_mutex_unlock(&phase_mutex);
		return -ENODATA;
	}
	if (off > phase_blob_len) {
		(void)k_mutex_unlock(&phase_mutex);
		return -EINVAL;
	}

	n = phase_blob_len - off;
	if (n > cap) {
		n = cap;
	}
	if (n > 0u) {
		memcpy(buf, &phase_blob[off], n);
	}
	(void)k_mutex_unlock(&phase_mutex);

	*out_n = n;
	return 0;
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
		 * Before the tick, so a park requested from the console is in
		 * force for the pass that follows it rather than one later —
		 * and so a Vc override lands on a loop this pass has not yet
		 * had the chance to resume.
		 */
		disc_service_mailbox();
		disc_handle_ref_requests();

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

		/*
		 * The automatic writer. While a maintenance override is held
		 * this records the level to go back to and leaves PA4 alone —
		 * a parked loop re-emits its frozen code every second, which
		 * would otherwise undo a technician's Vc within one PPS period.
		 */
		disc_apply_auto_dac(out.dac_code);
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

		/*
		 * Last, so an exported record reflects the ring as it stands
		 * after this second's push rather than before it.
		 */
		disc_service_phase_req();

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

	/* The actuator transfer, cached before the thread exists so the mailbox
	 * drain and sts_disc_dac_state() never read `disc` mid-tick. */
	dt_state.dac_vref_mv = cfg.dac_vref_mv;
	dt_state.dac_max_code = cfg.dac_max_code;

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
	disc_apply_auto_dac(cfg.dac_center_code);

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
