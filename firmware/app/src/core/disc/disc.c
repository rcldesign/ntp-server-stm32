/*
 * STS1000 "Meridian" — core/disc implementation.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * The sign conventions and the controller derivation are in disc.h. Read them
 * before changing anything in here.
 */

#include "disc/disc.h"

#include <errno.h>
#include <math.h>
#include <string.h>

/* Scale factor that makes the median absolute deviation a consistent estimator
 * of the standard deviation for normally distributed data (1/Phi^-1(0.75)). */
#define MAD_TO_SIGMA 1.4826f

/* Minimum window occupancy before the median/MAD gate is allowed to reject.
 * Below this the median is not robust enough to be worth trusting. */
#define MAD_MIN_SAMPLES 4u

/* Bounds on the measured tick interval. Outside these the sample is still used
 * for phase, but the frequency estimate and the ADEV ring treat it as a gap. */
#define DT_MIN_S 0.2f
#define DT_MAX_S 5.0f

/*
 * Largest sample gap the ADEV phase series will absorb before it is discarded.
 *
 * The estimator assumes uniform sampling, so the pedantic rule would be to
 * reset the ring whenever a second is missed. In practice the median/MAD gate
 * rejects a sample every few minutes on a noisy antenna, and a pedantic reset
 * means the ring never reaches the 201 points that tau = 100 s needs — the
 * estimate would simply never exist. Tolerating a gap of a second or two
 * perturbs a handful of second differences out of several hundred, which is
 * far less wrong than reporting nothing. Anything longer is a real
 * discontinuity and does reset the series.
 */
#define ADEV_MAX_GAP_S 3.0f

/* Averaging factors reported in the quality block (tau0 = 1 s). */
#define ADEV_M_1 1u
#define ADEV_M_10 10u
#define ADEV_M_100 100u

/* ------------------------------------------------------------------ config */

int disc_cfg_defaults(disc_cfg_t *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}

	memset(cfg, 0, sizeof(*cfg));

	/* §3.2 conditioning. Both delays are zero until calibrated (§10.4);
	 * a wrong non-zero default would be worse than none. */
	cfg->cable_delay_ns = 0;
	cfg->board_delay_ns = 0;
	cfg->qerr_sign = 1;
	cfg->xcheck_tol_ns = 250u;
	cfg->mad_window = 8u;
	cfg->mad_k = 5.0f;
	cfg->mad_floor_ns = 100.0f;
	cfg->max_consec_reject = 5u;

	/* §3.3 controller. tau 150 s sits in the spec's "~100-300 s" band;
	 * a = 2 gives zeta = 0.707. */
	cfg->tau_s = 150.0f;
	cfg->pi_damping_a = 2.0f;
	cfg->fll_gain = 0.25f;
	cfg->acq_exit_ns = 10000.0f;
	cfg->acq_reentry_ns = 50000.0f;
	cfg->acq_min_ticks = 8u;
	cfg->acq_ramp_max_ppb = 400.0f;
	cfg->recover_ramp_max_ppb = 50.0f;

	/* Actuator, interface ref §3: 12-bit DAC1_OUT1, centre 1.65 V,
	 * full scale = the OCXO's +-0.4 ppm pull. */
	cfg->dac_full_scale_ppb = 400.0f;
	cfg->dac_center_code = 2048u;
	cfg->dac_max_code = 4095u;
	cfg->dac_vref_mv = 3300u;
	/* 5 LSB/s = 0.98 ppb/s while serving; far above OCXO ageing and far
	 * below anything a client can see. A converging loop is not serving
	 * yet, so it gets a limit that can traverse the range in ~8 s. */
	cfg->slew_lsb_per_s = 5.0f;
	cfg->slew_acq_lsb_per_s = 256.0f;

	cfg->tempco_ppb_per_c = 0.0f;
	cfg->tempco_enable = false;

	cfg->lock_phase_ns = 500.0f;
	cfg->lock_var_ns2 = 40000.0f; /* sigma = 200 ns */
	cfg->lock_dwell_s = 30u;
	cfg->unlock_dwell_s = 10u;

	cfg->warm_temp_mc = 40000;
	cfg->warm_current_ua = 700000;
	cfg->warm_dwell_s = 60u;
	cfg->warm_timeout_s = 600u;

	cfg->pps_loss_ticks = 3u;
	cfg->holdover.base_ns = 100.0f;
	cfg->holdover.drift_ns_per_s = 1.0f;   /* 1 ppb: placeholder, BENCH */
	cfg->holdover.aging_ns_per_s2 = 0.0f;
	cfg->holdover.temp_ns_per_s_per_c = 0.0f;
	cfg->demote_threshold_ns = 1000000.0f; /* 1 ms */
	cfg->holdover_stratum = (uint8_t)QUALITY_STRATUM_UNSYNC;

	cfg->vc_tol_mv = 100;
	cfg->vc_fault_dwell_s = 3u;

	/*
	 * Dispersion floor while locked. RFC 5905 §11.1 makes every server add
	 * PHI = 15 ppm times the age of its last update, which for a 1 Hz PPS
	 * is already 15 us; the measurement residual, the receiver's own time
	 * accuracy and the cable-delay calibration error are all far below
	 * that. 16 us is therefore both honest and just over one unit of the
	 * NTP short format (1/65536 s = 15.26 us), so the served value never
	 * quantises to a zero dispersion.
	 */
	cfg->base_disp_ns = 16000.0f;

	return 0;
}

static bool cfg_valid(const disc_cfg_t *c)
{
	if (!(c->tau_s >= 10.0f) || !(c->tau_s <= 1000.0f)) {
		return false;
	}
	if (!(c->pi_damping_a > 0.0f) || !isfinite(c->pi_damping_a)) {
		return false;
	}
	if (!(c->fll_gain > 0.0f) || !(c->fll_gain <= 1.0f)) {
		return false;
	}
	if (c->qerr_sign != 1 && c->qerr_sign != -1) {
		return false;
	}
	if (c->mad_window < 3u || c->mad_window > DISC_MAD_WIN_MAX) {
		return false;
	}
	if (!(c->mad_k > 0.0f) || !(c->mad_floor_ns >= 0.0f)) {
		return false;
	}
	if (c->max_consec_reject == 0u) {
		return false;
	}
	if (c->lock_dwell_s < 2u || c->lock_dwell_s > DISC_LOCK_WIN_MAX) {
		return false;
	}
	if (c->unlock_dwell_s == 0u) {
		return false;
	}
	if (!(c->lock_phase_ns > 0.0f) || !(c->lock_var_ns2 > 0.0f)) {
		return false;
	}
	if (!(c->acq_exit_ns > 0.0f) || !(c->acq_reentry_ns > c->acq_exit_ns)) {
		return false;
	}
	if (!(c->acq_ramp_max_ppb > 0.0f) || !(c->recover_ramp_max_ppb > 0.0f)) {
		return false;
	}
	if (c->dac_max_code < 16u || c->dac_center_code == 0u ||
	    c->dac_center_code >= c->dac_max_code) {
		return false;
	}
	if (c->dac_vref_mv == 0u) {
		return false;
	}
	if (!isfinite(c->dac_full_scale_ppb) || c->dac_full_scale_ppb == 0.0f) {
		return false;
	}
	if (!(c->slew_lsb_per_s > 0.0f) || !(c->slew_acq_lsb_per_s > 0.0f)) {
		return false;
	}
	if (c->pps_loss_ticks == 0u || c->vc_fault_dwell_s == 0u) {
		return false;
	}
	if (!(c->demote_threshold_ns > 0.0f)) {
		return false;
	}
	if (!(c->base_disp_ns >= 0.0f)) {
		return false;
	}
	/*
	 * Every remaining float field must be finite. A `> 0` comparison
	 * already rejects NaN, but it lets +Inf through, and an infinite
	 * dispersion or holdover coefficient would later be cast to int64 for
	 * the quality block — undefined behaviour. tau_s, pi_damping_a and
	 * dac_full_scale_ppb are finiteness-checked above.
	 */
	if (!isfinite(c->mad_k) || !isfinite(c->mad_floor_ns) ||
	    !isfinite(c->acq_exit_ns) || !isfinite(c->acq_reentry_ns) ||
	    !isfinite(c->acq_ramp_max_ppb) || !isfinite(c->recover_ramp_max_ppb) ||
	    !isfinite(c->slew_lsb_per_s) || !isfinite(c->slew_acq_lsb_per_s) ||
	    !isfinite(c->tempco_ppb_per_c) || !isfinite(c->lock_phase_ns) ||
	    !isfinite(c->lock_var_ns2) || !isfinite(c->demote_threshold_ns) ||
	    !isfinite(c->base_disp_ns) || !isfinite(c->holdover.base_ns) ||
	    !isfinite(c->holdover.drift_ns_per_s) ||
	    !isfinite(c->holdover.aging_ns_per_s2) ||
	    !isfinite(c->holdover.temp_ns_per_s_per_c)) {
		return false;
	}
	return true;
}

/* ------------------------------------------------------------- small utils */

static float f_abs(float v)
{
	return (v < 0.0f) ? -v : v;
}

/*
 * Saturating float -> int32. A direct cast of an out-of-range or NaN float to
 * int32 is undefined behaviour in C, and last_e_ns can legitimately be a couple
 * of billion ns on a garbage capture before the loop rejects it, so telemetry
 * must clamp rather than cast blind.
 */
static int32_t clamp_i32(float v)
{
	if (!(v >= (float)INT32_MIN)) { /* also catches NaN */
		return INT32_MIN;
	}
	if (v >= (float)INT32_MAX) {
		return INT32_MAX;
	}
	return (int32_t)v;
}

/*
 * Signed difference of two free-running 32-bit counters, correct across the
 * wrap (17.18 s at 250 MHz). Written without relying on the implementation-
 * defined conversion of an out-of-range unsigned to signed, and handling
 * INT32_MIN explicitly because it is the one value that cannot be produced by
 * negating a positive int32_t.
 */
static int32_t wrap_diff(uint32_t a, uint32_t b)
{
	uint32_t d = a - b;

	if (d < 0x80000000u) {
		return (int32_t)d;
	}
	if (d == 0x80000000u) {
		return INT32_MIN;
	}
	return -(int32_t)(0xFFFFFFFFu - d + 1u);
}

static void sort_f(float *a, uint8_t n)
{
	uint8_t i;

	for (i = 1u; i < n; i++) {
		float v = a[i];
		uint8_t j = i;

		while (j > 0u && a[j - 1u] > v) {
			a[j] = a[j - 1u];
			j--;
		}
		a[j] = v;
	}
}

static float median_sorted(const float *a, uint8_t n)
{
	if (n == 0u) {
		return 0.0f;
	}
	if ((n & 1u) != 0u) {
		return a[n / 2u];
	}
	return 0.5f * (a[(n / 2u) - 1u] + a[n / 2u]);
}

static uint8_t state_to_quality(disc_state_t s)
{
	switch (s) {
	case DISC_STATE_ACQUIRING:
		return (uint8_t)QUALITY_LOCK_ACQUIRING;
	case DISC_STATE_LOCKING:
		return (uint8_t)QUALITY_LOCK_LOCKING;
	case DISC_STATE_LOCKED:
		return (uint8_t)QUALITY_LOCK_LOCKED;
	case DISC_STATE_HOLDOVER:
		return (uint8_t)QUALITY_LOCK_HOLDOVER;
	case DISC_STATE_RECOVERING:
		return (uint8_t)QUALITY_LOCK_RECOVERING;
	case DISC_STATE_PARKED:
		return (uint8_t)QUALITY_LOCK_PARKED;
	default:
		return (uint8_t)QUALITY_LOCK_UNKNOWN;
	}
}

/* --------------------------------------------------------------- lifecycle */

static void windows_reset(disc_ctx_t *ctx)
{
	ctx->mad_n = 0u;
	ctx->mad_head = 0u;
	ctx->consec_reject = 0u;
	ctx->lock_n = 0u;
	ctx->lock_head = 0u;
	ctx->lock_run = 0u;
	ctx->unlock_run = 0u;
	ctx->adev_n = 0u;
	ctx->adev_head = 0u;
}

int disc_init(disc_ctx_t *ctx, const disc_cfg_t *cfg)
{
	disc_cfg_t local;

	if (ctx == NULL) {
		return -EINVAL;
	}

	if (cfg == NULL) {
		(void)disc_cfg_defaults(&local);
		cfg = &local;
	}
	if (!cfg_valid(cfg)) {
		return -EINVAL;
	}

	memset(ctx, 0, sizeof(*ctx));
	ctx->cfg = *cfg;

	/* Kp = 1/tau, Ki = 1/(a*tau^2) — see the derivation in disc.h. */
	ctx->kp = 1.0f / cfg->tau_s;
	ctx->ki = 1.0f / (cfg->pi_damping_a * cfg->tau_s * cfg->tau_s);

	/* Signed, so an inverting OCXO tuning slope needs only a negative
	 * dac_full_scale_ppb and nothing else changes. */
	ctx->ppb_per_code = cfg->dac_full_scale_ppb / (float)cfg->dac_center_code;

	{
		float y_at_zero = (0.0f - (float)cfg->dac_center_code) *
				  ctx->ppb_per_code;
		float y_at_max = ((float)cfg->dac_max_code -
				  (float)cfg->dac_center_code) *
				 ctx->ppb_per_code;

		ctx->y_range_lo = (y_at_zero < y_at_max) ? y_at_zero : y_at_max;
		ctx->y_range_hi = (y_at_zero < y_at_max) ? y_at_max : y_at_zero;
	}

	ctx->state = DISC_STATE_ACQUIRING;
	ctx->dac_code = cfg->dac_center_code;
	ctx->vc_cmd_mv = (int32_t)(((uint32_t)ctx->dac_code *
				    (uint32_t)cfg->dac_vref_mv) /
				   (uint32_t)cfg->dac_max_code);
	ctx->t_demote_s = UINT32_MAX;
	windows_reset(ctx);
	ctx->initialised = true;

	return 0;
}

disc_state_t disc_state(const disc_ctx_t *ctx)
{
	if (ctx == NULL || !ctx->initialised) {
		return DISC_STATE__COUNT;
	}
	return ctx->state;
}

/* ------------------------------------------------------------ ADEV support */

int disc_adev_overlapping(const float *phase_ns, size_t n, uint32_t m,
			  float tau0_s, float *out)
{
	size_t terms;
	size_t i;
	float acc = 0.0f;
	float denom;

	if (phase_ns == NULL || out == NULL || m == 0u) {
		return -EINVAL;
	}
	if (!(tau0_s > 0.0f) || !isfinite(tau0_s)) {
		return -EINVAL;
	}
	if (n < (size_t)2u * (size_t)m + 1u) {
		return -ENODATA;
	}

	terms = n - (size_t)2u * (size_t)m;
	for (i = 0u; i < terms; i++) {
		float d = phase_ns[i + 2u * (size_t)m] -
			  2.0f * phase_ns[i + (size_t)m] + phase_ns[i];

		acc += d * d;
	}

	/* sigma_y^2 = acc / (2 m^2 tau0^2 terms); phase is in ns, so the
	 * dimensionless deviation carries the 1e-9. */
	denom = 2.0f * (float)m * (float)m * tau0_s * tau0_s * (float)terms;
	*out = quality_sqrtf(acc / denom) * 1.0e-9f;
	return 0;
}

static void adev_reset(disc_ctx_t *ctx)
{
	ctx->adev_n = 0u;
	ctx->adev_head = 0u;
	ctx->adev_1s = 0.0f;
	ctx->adev_10s = 0.0f;
	ctx->adev_100s = 0.0f;
}

/*
 * Append a phase sample. The buffer is kept linear and oldest-first — when it
 * is full the whole thing shifts down by one. That is 2 KB of memmove once a
 * second, which is cheaper than the alternative of copying a wrapped ring into
 * a scratch array every time the estimator runs (and would need a second 2 KB).
 */
static void adev_push(disc_ctx_t *ctx, float e_ns)
{
	if (ctx->adev_n < DISC_ADEV_CAP) {
		ctx->adev_buf[ctx->adev_n] = e_ns;
		ctx->adev_n++;
	} else {
		memmove(&ctx->adev_buf[0], &ctx->adev_buf[1],
			(DISC_ADEV_CAP - 1u) * sizeof(ctx->adev_buf[0]));
		ctx->adev_buf[DISC_ADEV_CAP - 1u] = e_ns;
	}
}

static void adev_update(disc_ctx_t *ctx)
{
	float v;

	if (disc_adev_overlapping(ctx->adev_buf, ctx->adev_n, ADEV_M_1, 1.0f,
				  &v) == 0) {
		ctx->adev_1s = v;
	}
	if (disc_adev_overlapping(ctx->adev_buf, ctx->adev_n, ADEV_M_10, 1.0f,
				  &v) == 0) {
		ctx->adev_10s = v;
	}
	if (disc_adev_overlapping(ctx->adev_buf, ctx->adev_n, ADEV_M_100, 1.0f,
				  &v) == 0) {
		ctx->adev_100s = v;
	}
}

/* --------------------------------------------------------- sample windows */

static void mad_push(disc_ctx_t *ctx, float e)
{
	uint8_t win = ctx->cfg.mad_window;

	ctx->mad_buf[ctx->mad_head] = e;
	ctx->mad_head = (uint8_t)((ctx->mad_head + 1u) % win);
	if (ctx->mad_n < win) {
		ctx->mad_n++;
	}
}

/*
 * §3.2 median/MAD outlier gate.
 *
 * The window holds accepted samples only, so one glitch cannot poison the
 * median. The scaled MAD is floored (cfg.mad_floor_ns) because a very quiet
 * window drives the MAD to zero, which would otherwise reject every subsequent
 * sample; and cfg.max_consec_reject force-accepts after a run of rejections, so
 * a genuine step cannot lock the loop out of its own input for ever.
 */
static bool mad_accept(disc_ctx_t *ctx, float e)
{
	float tmp[DISC_MAD_WIN_MAX];
	float dev[DISC_MAD_WIN_MAX];
	uint8_t n = ctx->mad_n;
	uint8_t i;
	float med;
	float mad;
	float thresh;

	if (n < MAD_MIN_SAMPLES) {
		return true;
	}
	if (ctx->consec_reject >= ctx->cfg.max_consec_reject) {
		/* The window no longer describes reality. Start again around
		 * this sample rather than keep rejecting. */
		ctx->mad_n = 0u;
		ctx->mad_head = 0u;
		ctx->consec_reject = 0u;
		return true;
	}

	for (i = 0u; i < n; i++) {
		tmp[i] = ctx->mad_buf[i];
	}
	sort_f(tmp, n);
	med = median_sorted(tmp, n);

	for (i = 0u; i < n; i++) {
		dev[i] = f_abs(ctx->mad_buf[i] - med);
	}
	sort_f(dev, n);
	mad = median_sorted(dev, n);

	thresh = ctx->cfg.mad_k * MAD_TO_SIGMA * mad;
	if (thresh < ctx->cfg.mad_floor_ns) {
		thresh = ctx->cfg.mad_floor_ns;
	}

	return f_abs(e - med) <= thresh;
}

static void lockwin_push(disc_ctx_t *ctx, float e)
{
	uint8_t win = ctx->cfg.lock_dwell_s;

	ctx->lock_buf[ctx->lock_head] = e;
	ctx->lock_head = (uint8_t)((ctx->lock_head + 1u) % win);
	if (ctx->lock_n < win) {
		ctx->lock_n++;
	}
}

/* Rolling mean and sample variance over the lock window. */
static void lockwin_stats(const disc_ctx_t *ctx, float *mean, float *var)
{
	uint8_t n = ctx->lock_n;
	uint8_t i;
	float sum = 0.0f;
	float acc = 0.0f;

	if (n < 2u) {
		/* A single sample has a mean and no variance; an empty window
		 * has neither. Both are only seen on the first tick. */
		*mean = (n == 1u) ? ctx->lock_buf[0] : 0.0f;
		*var = 0.0f;
		return;
	}

	for (i = 0u; i < n; i++) {
		sum += ctx->lock_buf[i];
	}
	*mean = sum / (float)n;

	for (i = 0u; i < n; i++) {
		float d = ctx->lock_buf[i] - *mean;

		acc += d * d;
	}
	*var = acc / (float)(n - 1u);
}

/* ----------------------------------------------------------- environmental */

static void update_warm(disc_ctx_t *ctx, const disc_env_t *env)
{
	bool have_sensor = env->osc_temp_valid || env->ocxo_current_valid;
	bool cond = false;
	uint64_t elapsed_s = 0u;

	if (have_sensor) {
		bool t_ok = !env->osc_temp_valid ||
			    (env->osc_temp_mc >= ctx->cfg.warm_temp_mc);
		bool i_ok = !env->ocxo_current_valid ||
			    (env->ocxo_current_ua <= ctx->cfg.warm_current_ua);

		cond = t_ok && i_ok;
	}

	if (cond) {
		if (ctx->warm_run < UINT16_MAX) {
			ctx->warm_run++;
		}
	} else {
		ctx->warm_run = 0u;
	}

	if (ctx->have_first && env->mono_ms >= ctx->first_mono_ms) {
		elapsed_s = (env->mono_ms - ctx->first_mono_ms) / 1000u;
	}

	/*
	 * DELIBERATE DEVIATION from the spec §3.3 MUST ("advertise stratum-1
	 * only when ... OCXO warm (INA228 #5 + TMP117 #1)"). A stuck-cold
	 * TMP117 or a mis-set threshold would otherwise block stratum-1 for
	 * ever, turning one failed sensor into a total loss of service. After
	 * warm_timeout_s the loop's own lock criteria (phase error and its
	 * variance under threshold for the dwell) are taken as sufficient
	 * evidence the oven is up — a disciplined OCXO is warm by definition.
	 * Set warm_timeout_s = 0 to disable the backstop and honour the sensor
	 * MUST absolutely. Recorded in the completion report as a known
	 * deviation for the architect's ruling.
	 */
	ctx->ocxo_warm = (ctx->warm_run >= ctx->cfg.warm_dwell_s) ||
			 ((ctx->cfg.warm_timeout_s != 0u) &&
			  (elapsed_s >= (uint64_t)ctx->cfg.warm_timeout_s));
}

/* Compare the commanded Vc against the PA3 read-back (interface ref §3). */
static void update_vc_check(disc_ctx_t *ctx, const disc_env_t *env)
{
	int32_t diff;

	ctx->vc_sense_valid = env->vc_sense_valid;
	if (!env->vc_sense_valid) {
		/* No evidence either way: hold the current verdict rather than
		 * clear a latched fault on missing data. */
		return;
	}

	ctx->vc_sense_mv = env->vc_sense_mv;
	diff = ctx->vc_cmd_mv - env->vc_sense_mv;
	if (diff < 0) {
		diff = -diff;
	}

	if (diff > ctx->cfg.vc_tol_mv) {
		if (ctx->vc_bad_run < UINT8_MAX) {
			ctx->vc_bad_run++;
		}
		if (ctx->vc_bad_run >= ctx->cfg.vc_fault_dwell_s) {
			ctx->vc_fault = true;
		}
	} else {
		ctx->vc_bad_run = 0u;
		ctx->vc_fault = false;
	}
}

/* ------------------------------------------------------------- holdover */

/*
 * Current temperature excursion from the holdover entry temperature. When the
 * live reading is missing the LAST KNOWN excursion is held (mirroring the
 * hold-on-missing-data policy in update_vc_check): a dropped sensor must not
 * zero the growth rate and so make the estimate stop rising — that would let
 * the served dispersion, and with it the advertised stratum, recover with no
 * actual reference behind it.
 */
static float holdover_dt_c(disc_ctx_t *ctx, const disc_env_t *env)
{
	if (ctx->holdover_start_temp_valid && env->osc_temp_valid) {
		ctx->holdover_last_dt_c =
			(float)(env->osc_temp_mc - ctx->holdover_start_temp_mc) *
			0.001f;
	}
	return ctx->holdover_last_dt_c;
}

/* Instantaneous growth rate d(est)/dt, ns/s, for the current excursion and
 * elapsed time. All three terms are non-negative (aging is clamped). */
static float holdover_slope(const disc_ctx_t *ctx, float dt_c, float t_s)
{
	const quality_holdover_model_t *m = &ctx->cfg.holdover;
	float rate = m->drift_ns_per_s +
		     m->temp_ns_per_s_per_c * f_abs(dt_c);
	float aging = (m->aging_ns_per_s2 > 0.0f) ? m->aging_ns_per_s2 : 0.0f;

	if (rate < 0.0f) {
		rate = 0.0f; /* a negative characterised drift is nonsensical */
	}
	return rate + aging * t_s;
}

/* Seconds from now until the accumulated estimate reaches the demote
 * threshold, anchored at the CURRENT estimate rather than at the model's base
 * — so it is consistent with the monotonic accumulation and never implies more
 * headroom than the estimate itself has already spent. */
static uint32_t holdover_time_to_demote(const disc_ctx_t *ctx, float dt_c,
					float t_s)
{
	float budget = ctx->cfg.demote_threshold_ns - ctx->holdover_est_ns;
	float aging = (ctx->cfg.holdover.aging_ns_per_s2 > 0.0f)
			      ? ctx->cfg.holdover.aging_ns_per_s2
			      : 0.0f;
	float slope = holdover_slope(ctx, dt_c, t_s);
	float tau;

	if (!(budget > 0.0f)) {
		return 0u;
	}
	if (aging > 0.0f) {
		float disc = slope * slope + 2.0f * aging * budget;

		/* Stable positive root, conjugate form (see quality.c L7). */
		tau = (2.0f * budget) / (slope + quality_sqrtf(disc));
	} else if (slope > 0.0f) {
		tau = budget / slope;
	} else {
		return UINT32_MAX;
	}

	if (!isfinite(tau)) {
		return UINT32_MAX;
	}
	return (tau >= (float)UINT32_MAX) ? UINT32_MAX : (uint32_t)tau;
}

static void enter_holdover(disc_ctx_t *ctx, const disc_env_t *env)
{
	ctx->state = DISC_STATE_HOLDOVER;
	ctx->holdover_start_ms = env->mono_ms;
	ctx->holdover_prev_ms = env->mono_ms;
	ctx->holdover_start_temp_mc = env->osc_temp_mc;
	ctx->holdover_start_temp_valid = env->osc_temp_valid;
	ctx->holdover_last_dt_c = 0.0f;
	ctx->holdover_elapsed_s = 0u;
	/* Seed at the model's base error (clamped non-negative). */
	ctx->holdover_est_ns = quality_holdover_err_ns(&ctx->cfg.holdover, 0.0f,
						       0.0f);
	ctx->t_demote_s = holdover_time_to_demote(ctx, 0.0f, 0.0f);

	/* The actuator is frozen from here; nothing that assumes continuity of
	 * the phase record survives the gap. */
	ctx->have_prev = false;
	windows_reset(ctx);
	adev_reset(ctx);
}

static void update_holdover(disc_ctx_t *ctx, const disc_env_t *env)
{
	float dt_c = holdover_dt_c(ctx, env);
	float t_s = 0.0f;
	float dt_tick = 0.0f;

	if (env->mono_ms >= ctx->holdover_start_ms) {
		uint64_t ms = env->mono_ms - ctx->holdover_start_ms;

		ctx->holdover_elapsed_s = (uint32_t)(ms / 1000u);
		t_s = (float)ms * 0.001f;
	}
	if (env->mono_ms > ctx->holdover_prev_ms) {
		dt_tick = (float)(env->mono_ms - ctx->holdover_prev_ms) * 0.001f;
	}
	ctx->holdover_prev_ms = env->mono_ms;

	/*
	 * Accumulate: est += (d/dt)(est) * dt_tick, using the growth rate for
	 * the excursion and elapsed time observed *now*. Integrating the rate
	 * instead of re-evaluating the closed form means a temperature that
	 * rises and falls leaves its accumulated contribution in place, and a
	 * held-over sensor keeps the last rate — the estimate can only ever
	 * grow. The increment is clamped non-negative as a final guard against
	 * a pathological (negative) characterised drift.
	 */
	{
		float delta = holdover_slope(ctx, dt_c, t_s) * dt_tick;

		if (delta > 0.0f) {
			ctx->holdover_est_ns += delta;
		}
	}

	ctx->t_demote_s = holdover_time_to_demote(ctx, dt_c, t_s);
}

/* --------------------------------------------------------------- actuator */

static float ramp_limit(const disc_ctx_t *ctx)
{
	switch (ctx->state) {
	case DISC_STATE_ACQUIRING:
		return ctx->cfg.acq_ramp_max_ppb;
	case DISC_STATE_RECOVERING:
		return ctx->cfg.recover_ramp_max_ppb;
	default:
		return INFINITY;
	}
}

/*
 * The tight slew limit applies exactly when the clock is serving a primary
 * stratum — that is what it is for: bounding the rate of frequency change a
 * client can see. ACQUIRING and LOCKING are not serving (served_stratum()
 * returns 16 for both), so holding them to 5 LSB/s would only stop the loop
 * converging; RECOVERING is serving, and its phase correction is separately
 * rate-limited by cfg.recover_ramp_max_ppb.
 */
static float slew_limit(const disc_ctx_t *ctx)
{
	switch (ctx->state) {
	case DISC_STATE_LOCKED:
	case DISC_STATE_RECOVERING:
		return ctx->cfg.slew_lsb_per_s;
	default:
		return ctx->cfg.slew_acq_lsb_per_s;
	}
}

static int32_t code_to_mv(const disc_ctx_t *ctx, uint16_t code)
{
	return (int32_t)(((uint32_t)code * (uint32_t)ctx->cfg.dac_vref_mv) /
			 (uint32_t)ctx->cfg.dac_max_code);
}

/* ------------------------------------------------------------- publication */

static void fill_out(const disc_ctx_t *ctx, disc_out_t *o, uint32_t flags,
		     bool accepted, uint8_t stratum)
{
	memset(o, 0, sizeof(*o));
	o->state = ctx->state;
	o->dac_code = ctx->dac_code;
	o->vc_cmd_mv = ctx->vc_cmd_mv;
	o->sample_accepted = accepted;
	o->phase_valid = ctx->last_e_valid && accepted;
	o->phase_err_ns = ctx->last_e_ns;
	o->applied_ppb = ctx->y_eff_prev;
	o->freq_err_ppb = ctx->freq_err_ppb;
	o->adev_1s = ctx->adev_1s;
	o->adev_10s = ctx->adev_10s;
	o->adev_100s = ctx->adev_100s;
	o->holdover = (ctx->state == DISC_STATE_HOLDOVER);
	o->holdover_est_err_ns = o->holdover ? (int64_t)ctx->holdover_est_ns : 0;
	o->holdover_t_demote_s = o->holdover ? ctx->t_demote_s : UINT32_MAX;
	o->stratum = stratum;
	o->flags = flags;
}

/*
 * RFC 5905 root dispersion, in nanoseconds before the NTP-short conversion.
 *
 * cfg.base_disp_ns is the floor a disciplined server always carries (the PHI
 * term, see disc_cfg_defaults). Everything else is added on top of it, never
 * substituted for it — in particular, entering holdover must only ever make
 * the served dispersion grow. The §3.6 estimate is the *additional* error
 * accumulated since the reference was lost.
 */
static float served_dispersion_ns(const disc_ctx_t *ctx)
{
	switch (ctx->state) {
	case DISC_STATE_HOLDOVER:
		return ctx->cfg.base_disp_ns + ctx->holdover_est_ns;
	case DISC_STATE_LOCKED:
		return ctx->cfg.base_disp_ns + ctx->pps_sigma_ns;
	default:
		return ctx->cfg.base_disp_ns + f_abs(ctx->last_e_ns);
	}
}

static uint8_t served_stratum(const disc_ctx_t *ctx)
{
	if (!ctx->ever_locked) {
		/* A clock that has never been disciplined must not advertise a
		 * primary stratum, however it got into HOLDOVER or RECOVERING
		 * (an unpark from ACQUIRING is the reachable case). */
		return (uint8_t)QUALITY_STRATUM_UNSYNC;
	}

	switch (ctx->state) {
	case DISC_STATE_LOCKED:
		return (uint8_t)QUALITY_STRATUM_PRIMARY;
	case DISC_STATE_HOLDOVER:
		return (ctx->holdover_est_ns <= ctx->cfg.demote_threshold_ns)
			       ? (uint8_t)QUALITY_STRATUM_PRIMARY
			       : ctx->cfg.holdover_stratum;
	case DISC_STATE_RECOVERING:
		return (f_abs(ctx->last_e_ns) <= ctx->cfg.demote_threshold_ns)
			       ? (uint8_t)QUALITY_STRATUM_PRIMARY
			       : ctx->cfg.holdover_stratum;
	default:
		return (uint8_t)QUALITY_STRATUM_UNSYNC;
	}
}

static void publish(const disc_ctx_t *ctx, const disc_env_t *env,
		    const disc_out_t *o, quality_state_t *qs)
{
	quality_block_t b;

	if (qs == NULL) {
		return;
	}

	quality_block_init(&b);
	b.updated_mono_ms = env->mono_ms;
	b.stratum = o->stratum;
	b.lock_state = state_to_quality(ctx->state);
	b.active_ref = env->anc.active_ref;
	b.gnss_fix = env->anc.gnss_fix;
	b.holdover = o->holdover;
	b.utc_valid = env->anc.utc_valid;
	b.gnss_sv_used = env->anc.gnss_sv_used;
	b.gnss_sv_visible = env->anc.gnss_sv_visible;
	b.refid = (env->anc.refid != 0u) ? env->anc.refid
					 : quality_refid('G', 'P', 'S', '\0');
	b.root_delay_q16 = quality_ntp_short_from_ns(
		(int64_t)env->anc.root_delay_ns);
	b.root_disp_q16 = quality_ntp_short_from_ns(
		(int64_t)served_dispersion_ns(ctx));
	b.holdover_est_err_ns = o->holdover_est_err_ns;
	b.holdover_elapsed_s = o->holdover ? ctx->holdover_elapsed_s : 0u;
	b.holdover_t_demote_s = o->holdover_t_demote_s;
	b.last_pps_off_ns = clamp_i32(ctx->last_e_ns);
	b.pps_off_mean_ns = ctx->pps_mean_ns;
	b.pps_off_sigma_ns = ctx->pps_sigma_ns;
	b.freq_err_ppb = ctx->freq_err_ppb;
	b.vc_cmd_mv = ctx->vc_cmd_mv;
	b.vc_sense_mv = ctx->vc_sense_valid ? ctx->vc_sense_mv : 0;
	b.dac_code = ctx->dac_code;
	b.adev_1s = ctx->adev_1s;
	b.adev_10s = ctx->adev_10s;
	b.adev_100s = ctx->adev_100s;
	b.gnss_tacc_ns = env->anc.gnss_tacc_ns;
	b.leap_pending = env->anc.leap_pending;
	b.leap_current_s = env->anc.leap_current_s;
	b.leap_at_tai_s = env->anc.leap_at_tai_s;
	b.osc_temp_mc = env->osc_temp_valid ? env->osc_temp_mc : 0;
	/* Flag the two fields whose zero would otherwise be ambiguous. */
	b.flags = o->flags;
	if (ctx->vc_sense_valid) {
		b.flags |= QUALITY_FLAG_VC_SENSE_VALID;
	}
	if (env->osc_temp_valid) {
		b.flags |= QUALITY_FLAG_OSC_TEMP_VALID;
	}

	(void)quality_publish(qs, &b);
}

/* §3.6: holdover or recovery that has spent its error budget is demoted. */
static uint32_t demoted_flag(const disc_ctx_t *ctx, uint8_t stratum)
{
	if (stratum == (uint8_t)QUALITY_STRATUM_PRIMARY) {
		return 0u;
	}
	if (ctx->state != DISC_STATE_HOLDOVER &&
	    ctx->state != DISC_STATE_RECOVERING) {
		return 0u;
	}
	return QUALITY_FLAG_DEMOTED;
}

/* Flags common to every tick. */
static uint32_t base_flags(const disc_ctx_t *ctx, const disc_env_t *env)
{
	uint32_t f = 0u;

	if (ctx->ocxo_warm) {
		f |= QUALITY_FLAG_OCXO_WARM;
	}
	if (env->gnss_time_locked) {
		f |= QUALITY_FLAG_GNSS_TIME_LOCKED;
	}
	if (ctx->vc_fault) {
		f |= QUALITY_FLAG_DAC_FAULT;
	}
	if (ctx->cfg.tempco_enable && ctx->tref_valid) {
		f |= QUALITY_FLAG_TEMPCO;
	}
	if (ctx->state == DISC_STATE_HOLDOVER) {
		f |= QUALITY_FLAG_HOLDOVER;
	}
	if (ctx->state == DISC_STATE_PARKED) {
		f |= QUALITY_FLAG_PARKED;
	}
	return f;
}

/* ------------------------------------------------------- the no-sample tick */

/*
 * Shared by disc_tick_no_pps(), a rejected sample, an all-invalid capture pair
 * and a GNSS time-unlock. Spec §3.6: freeze the actuator and let the holdover
 * transitions run.
 */
static void tick_no_sample(disc_ctx_t *ctx, const disc_env_t *env,
			   uint32_t extra_flags, quality_state_t *qs,
			   disc_out_t *out)
{
	disc_out_t o;
	uint32_t flags;
	uint8_t stratum;

	if (ctx->miss_run < UINT32_MAX) {
		ctx->miss_run++;
	}
	ctx->lock_run = 0u;
	if (ctx->unlock_run < UINT16_MAX) {
		ctx->unlock_run++;
	}

	if (ctx->state == DISC_STATE_LOCKED &&
	    ctx->unlock_run >= (uint16_t)ctx->cfg.unlock_dwell_s) {
		ctx->state = DISC_STATE_LOCKING;
	}

	switch (ctx->state) {
	case DISC_STATE_LOCKING:
	case DISC_STATE_LOCKED:
	case DISC_STATE_RECOVERING:
		if (!env->gnss_time_locked ||
		    ctx->miss_run >= (uint32_t)ctx->cfg.pps_loss_ticks) {
			enter_holdover(ctx, env);
		}
		break;
	default:
		break;
	}

	if (ctx->state == DISC_STATE_HOLDOVER) {
		update_holdover(ctx, env);
	}

	ctx->last_e_valid = false;
	stratum = served_stratum(ctx);
	flags = base_flags(ctx, env) | extra_flags |
		demoted_flag(ctx, stratum);
	fill_out(ctx, &o, flags, false, stratum);
	ctx->flags = flags;

	publish(ctx, env, &o, qs);
	if (out != NULL) {
		*out = o;
	}
}

/* ------------------------------------------------------- the sample pipeline */

/*
 * §3.2 steps 1, 3 and 4. Returns false when no capture is usable.
 *
 * `diverge` is set when both channels captured but disagree by more than
 * cfg.xcheck_tol_ns. The sample is not dropped on divergence — PA0 is the
 * primary and a disagreeing PC6 is more likely a secondary-channel problem —
 * but the flag is published and the median/MAD gate still gets its say.
 */
static bool condition_sample(const disc_ctx_t *ctx, const disc_pps_t *p,
			     float *out_e, bool *diverge)
{
	float e_pri = 0.0f;
	float e_sec = 0.0f;
	bool have_pri = false;
	bool have_sec = false;
	float e;

	*diverge = false;

	if (p->primary_valid && isfinite(p->primary_ns_per_count)) {
		int32_t d = wrap_diff(p->primary_expected, p->primary_count);

		e_pri = (float)d * p->primary_ns_per_count;
		have_pri = true;
	}
	if (p->secondary_valid && isfinite(p->secondary_ns_per_count)) {
		int32_t d = wrap_diff(p->secondary_expected, p->secondary_count);

		e_sec = (float)d * p->secondary_ns_per_count;
		have_sec = true;
	}

	if (have_pri && have_sec) {
		if (f_abs(e_pri - e_sec) > (float)ctx->cfg.xcheck_tol_ns) {
			*diverge = true;
			e = e_pri;
		} else {
			/* Averaging two independent captures of the same edge
			 * removes ~3 dB of capture jitter. */
			e = 0.5f * (e_pri + e_sec);
		}
	} else if (have_pri) {
		e = e_pri;
	} else if (have_sec) {
		e = e_sec;
	} else {
		return false;
	}

	/* §3.2 step 2: sawtooth. §3.2 step 3: cable and board delay. Both
	 * refer the measured edge backwards in time, so both add to e. */
	if (p->qerr_valid) {
		e += (float)ctx->cfg.qerr_sign * ((float)p->qerr_ps * 0.001f);
	}
	e += (float)ctx->cfg.cable_delay_ns;
	e += (float)ctx->cfg.board_delay_ns;

	if (!isfinite(e)) {
		return false;
	}

	*out_e = e;
	return true;
}

/*
 * §3.3 lock criteria. The variance must merely be defined (>= 2 samples), not
 * computed over a full lock_dwell_s window: the dwell itself is enforced
 * separately by lock_run counting consecutive seconds these criteria hold, so
 * requiring a full window here as well made the effective lock time ~2*dwell.
 * By the time lock_run reaches the dwell the window is full anyway, so the
 * variance that actually gates the promotion is over the whole window.
 */
static bool lock_criteria_met(const disc_ctx_t *ctx, const disc_env_t *env,
			      float e, float var)
{
	return (f_abs(e) < ctx->cfg.lock_phase_ns) &&
	       (var < ctx->cfg.lock_var_ns2) && (ctx->lock_n >= 2u) &&
	       ctx->ocxo_warm && env->gnss_time_locked;
}

/* Promote to LOCKED from LOCKING or RECOVERING. */
static void promote_to_locked(disc_ctx_t *ctx, const disc_env_t *env)
{
	ctx->state = DISC_STATE_LOCKED;
	ctx->ever_locked = true;
	/* Reference temperature for the §3.3 feed-forward, captured once so the
	 * term is zero the moment it starts being applied and the actuator does
	 * not bump. Only meaningful with a live sensor. */
	if (!ctx->tref_valid && env->osc_temp_valid) {
		ctx->tref_mc = env->osc_temp_mc;
		ctx->tref_valid = true;
	}
}

int disc_tick_pps(disc_ctx_t *ctx, const disc_in_t *in, quality_state_t *qs,
		  disc_out_t *out)
{
	disc_out_t o;
	const disc_env_t *env;
	float e = 0.0f;
	bool diverge = false;
	float dt = 1.0f;
	bool freq_ok = false;
	float ff = 0.0f;
	float ramp_max;
	float p_raw;
	float p_term;
	bool ramp_clamped = false;
	float slew_step;
	float code_lo_f;
	float code_hi_f;
	float band_lo;
	float band_hi;
	float y_des;
	float code_des_f;
	bool sat = false;
	bool slewed = false;
	uint16_t code;
	float mean = 0.0f;
	float var = 0.0f;
	uint32_t flags;
	uint8_t stratum;

	if (ctx == NULL || in == NULL || !ctx->initialised) {
		return -EINVAL;
	}
	env = &in->env;

	if (ctx->state == DISC_STATE_PARKED) {
		fill_out(ctx, &o, base_flags(ctx, env), false,
			 (uint8_t)QUALITY_STRATUM_UNSYNC);
		ctx->flags = o.flags;
		publish(ctx, env, &o, qs);
		if (out != NULL) {
			*out = o;
		}
		return 0;
	}

	if (!ctx->have_first) {
		ctx->have_first = true;
		ctx->first_mono_ms = env->mono_ms;
	}
	update_warm(ctx, env);
	update_vc_check(ctx, env);

	/* GNSS time unusable: the capture may be present but it is not a UTC
	 * reference any more (§3.6). */
	if (!env->gnss_time_locked) {
		tick_no_sample(ctx, env, 0u, qs, out);
		return 0;
	}

	if (!condition_sample(ctx, &in->pps, &e, &diverge)) {
		ctx->n_miss++;
		tick_no_sample(ctx, env, QUALITY_FLAG_NO_PPS, qs, out);
		return 0;
	}
	if (diverge) {
		ctx->n_diverge++;
	}

	if (!mad_accept(ctx, e)) {
		ctx->n_reject++;
		ctx->consec_reject++;
		tick_no_sample(ctx,
			       env,
			       QUALITY_FLAG_PPS_REJECT |
				       (diverge ? QUALITY_FLAG_PPS_DIVERGE : 0u),
			       qs, out);
		return 0;
	}

	/* ---- accepted from here on ---- */
	ctx->consec_reject = 0u;
	ctx->miss_run = 0u;
	ctx->n_accept++;

	if (ctx->have_prev) {
		if (env->mono_ms > ctx->prev_mono_ms) {
			dt = (float)(env->mono_ms - ctx->prev_mono_ms) * 0.001f;
		} else {
			dt = 0.0f;
		}
		freq_ok = (dt >= DT_MIN_S) && (dt <= DT_MAX_S);
	}
	if (!freq_ok) {
		dt = 1.0f;
	}

	/* Holdover exit: §3.6 requires a rate-limited pull-in, never a step. */
	if (ctx->state == DISC_STATE_HOLDOVER) {
		ctx->state = DISC_STATE_RECOVERING;
		ctx->holdover_elapsed_s = 0u;
		ctx->t_demote_s = UINT32_MAX;
	}

	/* §3.3 tempco feed-forward. The reference temperature is captured on
	 * first lock, so the term is zero at the moment it is switched on and
	 * the actuator does not bump. */
	if (ctx->cfg.tempco_enable && ctx->tref_valid && env->osc_temp_valid) {
		float d_c = (float)(env->osc_temp_mc - ctx->tref_mc) * 0.001f;

		ff = -ctx->cfg.tempco_ppb_per_c * d_c;
	}
	ctx->ff_ppb = ff;

	/* Achievable command band this tick: the DAC range intersected with
	 * what the slew limiter allows from where the actuator already is. */
	slew_step = slew_limit(ctx) * dt;
	code_lo_f = (float)ctx->dac_code - slew_step;
	code_hi_f = (float)ctx->dac_code + slew_step;
	if (code_lo_f < 0.0f) {
		code_lo_f = 0.0f;
	}
	if (code_hi_f > (float)ctx->cfg.dac_max_code) {
		code_hi_f = (float)ctx->cfg.dac_max_code;
	}
	{
		float ya = (code_lo_f - (float)ctx->cfg.dac_center_code) *
			   ctx->ppb_per_code;
		float yb = (code_hi_f - (float)ctx->cfg.dac_center_code) *
			   ctx->ppb_per_code;

		band_lo = (ya < yb) ? ya : yb;
		band_hi = (ya < yb) ? yb : ya;
	}

	ramp_max = ramp_limit(ctx);
	p_raw = ctx->kp * e;
	p_term = p_raw;
	if (p_term > ramp_max) {
		p_term = ramp_max;
		ramp_clamped = true;
	} else if (p_term < -ramp_max) {
		p_term = -ramp_max;
		ramp_clamped = true;
	}

	/*
	 * Frequency state update. ACQUIRING always uses the FLL; RECOVERING
	 * uses it whenever the phase ramp is clamped, because then the PI
	 * integral is held and the FLL is the only thing that can null a
	 * frequency error accumulated during holdover.
	 */
	if (freq_ok) {
		float f_obs = -(e - ctx->e_prev) / dt;
		/* Telemetry only: a one-pole filter of the measured residual
		 * frequency error, with the loop's own time constant. dt is
		 * clamped to DT_MAX_S and tau is validated at 10 s or more, so
		 * alpha cannot reach 1 and the filter cannot overshoot. */
		float alpha = dt / ctx->cfg.tau_s;

		ctx->freq_err_ppb += alpha * (f_obs - ctx->freq_err_ppb);

		if (ctx->state == DISC_STATE_ACQUIRING ||
		    (ctx->state == DISC_STATE_RECOVERING && ramp_clamped)) {
			float f_osc = ctx->y_eff_prev - f_obs;

			ctx->y_int += ctx->cfg.fll_gain *
				      ((f_osc - ff) - ctx->y_int);
		}
	}

	/*
	 * PI integral with conditional-integration anti-windup: the update is
	 * dropped only when it would push the command further outside what the
	 * actuator can reach this tick. Holding it while the ramp clamp is
	 * active is what stops the integrator racing ahead of a deliberately
	 * rate-limited phase correction.
	 */
	if (ctx->state != DISC_STATE_ACQUIRING && !ramp_clamped) {
		float i_delta = ctx->ki * e * dt;
		float y_post = ff + ctx->y_int + i_delta + p_term;
		bool push_out = (y_post > band_hi && i_delta > 0.0f) ||
				(y_post < band_lo && i_delta < 0.0f);

		if (!push_out) {
			ctx->y_int += i_delta;
		}
	}

	y_des = ff + ctx->y_int + p_term;
	if (!isfinite(y_des)) {
		/* Numerically impossible with finite inputs, but a NaN reaching
		 * the DAC would be unrecoverable. Hold and re-centre the
		 * frequency state. */
		y_des = ctx->y_eff_prev;
		ctx->y_int = 0.0f;
	}

	code_des_f = (float)ctx->cfg.dac_center_code + y_des / ctx->ppb_per_code;
	if (code_des_f < 0.0f) {
		code_des_f = 0.0f;
		sat = true;
	} else if (code_des_f > (float)ctx->cfg.dac_max_code) {
		code_des_f = (float)ctx->cfg.dac_max_code;
		sat = true;
	}
	if (code_des_f < code_lo_f) {
		code_des_f = code_lo_f;
		slewed = true;
	} else if (code_des_f > code_hi_f) {
		code_des_f = code_hi_f;
		slewed = true;
	}

	code = (uint16_t)(code_des_f + 0.5f);
	/* Belt and braces on a value that goes straight into a DAC register:
	 * the two clamps above already bound code_des_f to [0, dac_max_code]
	 * and rounding cannot carry past the top, but an out-of-range write to
	 * PA4 is not a failure mode worth leaving to an argument. */
	if (code > ctx->cfg.dac_max_code) {
		code = ctx->cfg.dac_max_code;
	}
	ctx->dac_code = code;
	ctx->vc_cmd_mv = code_to_mv(ctx, code);
	ctx->y_eff_prev = ((float)code - (float)ctx->cfg.dac_center_code) *
			  ctx->ppb_per_code;

	/* ---- history and statistics ---- */
	ctx->e_prev = e;
	ctx->prev_mono_ms = env->mono_ms;
	ctx->have_prev = true;
	ctx->last_e_ns = e;
	ctx->last_e_valid = true;

	mad_push(ctx, e);
	lockwin_push(ctx, e);
	lockwin_stats(ctx, &mean, &var);
	ctx->pps_mean_ns = mean;
	ctx->pps_sigma_ns = quality_sqrtf(var);

	if (!freq_ok || dt > ADEV_MAX_GAP_S) {
		adev_reset(ctx);
	}
	adev_push(ctx, e);
	adev_update(ctx);

	/* ---- §3.3 / §3.6 state machine ---- */
	if (lock_criteria_met(ctx, env, e, var)) {
		if (ctx->lock_run < UINT16_MAX) {
			ctx->lock_run++;
		}
		ctx->unlock_run = 0u;
	} else {
		ctx->lock_run = 0u;
		if (ctx->unlock_run < UINT16_MAX) {
			ctx->unlock_run++;
		}
	}

	switch (ctx->state) {
	case DISC_STATE_ACQUIRING:
		if (ctx->acq_ticks < UINT32_MAX) {
			ctx->acq_ticks++;
		}
		if (f_abs(e) < ctx->cfg.acq_exit_ns &&
		    ctx->acq_ticks >= (uint32_t)ctx->cfg.acq_min_ticks) {
			ctx->state = DISC_STATE_LOCKING;
		}
		break;
	case DISC_STATE_LOCKING:
		/*
		 * Cold-start path only. LOCKING has never served a primary
		 * stratum (served_stratum() returns 16 for it), so a large
		 * phase excursion can safely drop back to the fast FLL pull-in
		 * — no client sees a step.
		 */
		if (f_abs(e) > ctx->cfg.acq_reentry_ns) {
			ctx->state = DISC_STATE_ACQUIRING;
			ctx->acq_ticks = 0u;
		} else if (ctx->lock_run >= (uint16_t)ctx->cfg.lock_dwell_s) {
			promote_to_locked(ctx, env);
		}
		break;
	case DISC_STATE_RECOVERING:
		/*
		 * §3.6 / the disc.h banner: RECOVERING exists precisely because
		 * the frequency estimate SURVIVED holdover, so re-convergence is
		 * a rate-limited phase pull-in with NO step — regardless of how
		 * large the post-holdover excursion is. It must NOT fall back to
		 * ACQUIRING (400 ppb ramp, 256 LSB/s slew ≈ 50 ppb/s DAC slam)
		 * or a healthy GNSS-return would jump a stratum-1 server's time
		 * and demote it. The FLL assist (run while the phase ramp is
		 * clamped) re-learns any frequency lost during the outage, still
		 * bounded by slew_lsb_per_s; served stratum follows the
		 * dispersion/demote policy in served_stratum().
		 */
		if (ctx->lock_run >= (uint16_t)ctx->cfg.lock_dwell_s) {
			promote_to_locked(ctx, env);
		}
		break;
	case DISC_STATE_LOCKED:
		if (ctx->unlock_run >= (uint16_t)ctx->cfg.unlock_dwell_s) {
			ctx->state = DISC_STATE_LOCKING;
		}
		break;
	default:
		break;
	}

	/* ---- publish ---- */
	flags = base_flags(ctx, env);
	if (diverge) {
		flags |= QUALITY_FLAG_PPS_DIVERGE;
	}
	if (sat) {
		flags |= QUALITY_FLAG_DAC_SAT;
	}
	if (slewed) {
		flags |= QUALITY_FLAG_DAC_SLEW;
	}
	stratum = served_stratum(ctx);
	flags |= demoted_flag(ctx, stratum);
	ctx->flags = flags;

	fill_out(ctx, &o, flags, true, stratum);
	publish(ctx, env, &o, qs);
	if (out != NULL) {
		*out = o;
	}

	return 0;
}

int disc_tick_no_pps(disc_ctx_t *ctx, const disc_env_t *env, quality_state_t *qs,
		     disc_out_t *out)
{
	disc_out_t o;
	uint32_t flags;

	if (ctx == NULL || env == NULL || !ctx->initialised) {
		return -EINVAL;
	}

	if (ctx->state == DISC_STATE_PARKED) {
		fill_out(ctx, &o, base_flags(ctx, env), false,
			 (uint8_t)QUALITY_STRATUM_UNSYNC);
		ctx->flags = o.flags;
		publish(ctx, env, &o, qs);
		if (out != NULL) {
			*out = o;
		}
		return 0;
	}

	if (!ctx->have_first) {
		ctx->have_first = true;
		ctx->first_mono_ms = env->mono_ms;
	}
	update_warm(ctx, env);
	update_vc_check(ctx, env);

	ctx->n_miss++;
	flags = QUALITY_FLAG_NO_PPS;
	tick_no_sample(ctx, env, flags, qs, out);
	return 0;
}

/* -------------------------------------------------------------- PFI park */

int disc_park(disc_ctx_t *ctx, uint16_t *out_code)
{
	if (ctx == NULL || !ctx->initialised) {
		return -EINVAL;
	}

	ctx->state = DISC_STATE_PARKED;
	ctx->have_prev = false;
	if (out_code != NULL) {
		*out_code = ctx->dac_code;
	}
	return 0;
}

int disc_unpark(disc_ctx_t *ctx)
{
	if (ctx == NULL || !ctx->initialised) {
		return -EINVAL;
	}
	if (ctx->state != DISC_STATE_PARKED) {
		return 0;
	}

	/* The frequency estimate survived the park, so this is a §3.6 recovery,
	 * not a cold acquisition: rate-limited pull-in, never a step. */
	ctx->state = DISC_STATE_RECOVERING;
	ctx->have_prev = false;
	ctx->miss_run = 0u;
	windows_reset(ctx);
	adev_reset(ctx);
	return 0;
}

/* ------------------------------------------------------------ tempco fit */

int disc_tempco_fit(const float *temp_c, const float *osc_ppb, size_t n,
		    float *out_ppb_per_c, float *out_offset_ppb, float *out_r2)
{
	size_t i;
	double sx = 0.0;
	double sy = 0.0;
	double sxx = 0.0;
	double syy = 0.0;
	double sxy = 0.0;
	double nd;
	double xbar;
	double ybar;
	double slope;

	if (temp_c == NULL || osc_ppb == NULL || out_ppb_per_c == NULL || n < 2u) {
		return -EINVAL;
	}

	/* Offline helper, not a hot path: double accumulators keep the normal
	 * equations well conditioned over long logs. */
	for (i = 0u; i < n; i++) {
		sx += (double)temp_c[i];
		sy += (double)osc_ppb[i];
	}
	nd = (double)n;
	xbar = sx / nd;
	ybar = sy / nd;

	for (i = 0u; i < n; i++) {
		double dx = (double)temp_c[i] - xbar;
		double dy = (double)osc_ppb[i] - ybar;

		sxx += dx * dx;
		syy += dy * dy;
		sxy += dx * dy;
	}

	if (!(sxx > 0.0)) {
		return -EDOM;
	}

	slope = sxy / sxx;
	*out_ppb_per_c = (float)slope;
	if (out_offset_ppb != NULL) {
		*out_offset_ppb = (float)(ybar - slope * xbar);
	}
	if (out_r2 != NULL) {
		*out_r2 = (syy > 0.0) ? (float)((sxy * sxy) / (sxx * syy)) : 1.0f;
	}
	return 0;
}
