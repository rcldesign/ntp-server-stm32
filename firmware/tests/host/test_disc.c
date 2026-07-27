/*
 * STS1000 "Meridian" — core/disc unit tests.
 *
 * The discipline loop cannot be tested by poking one function at a time: what
 * matters is what a closed loop does over hundreds of seconds. Most cases here
 * therefore drive the module second by second against an oscillator model and
 * assert the property spec §3.2/§3.3/§3.6 asks for — convergence, bounded
 * actuator movement, outlier immunity, a frozen actuator in holdover, a
 * rate-limited recovery.
 *
 * Provenance of the expectations:
 *
 *  - The sign-convention and correction cases are hand-computed from the
 *    formulae in the disc.h banner, with the arithmetic written out in the
 *    comment. They are the tests that would catch an inverted loop.
 *
 *  - The ADEV cases are checked against the analytic law for the noise type
 *    injected: white FM has sigma_y(tau) = sigma_y(1)/sqrt(m), which is a
 *    property of the noise process, not of this estimator.
 *
 *  - The least-squares fit is checked against data generated from a known line
 *    (exact recovery) and against a hand-computable two-point case.
 *
 *  - Several cases are differential: they run the same scenario twice, with a
 *    mechanism enabled and disabled, and assert the mechanism helps. A test
 *    that only asserts "the number is small" cannot tell whether the feature
 *    under test did anything.
 *
 * The oscillator model is documented at its definition. Its one job is to be
 * an honest integrator: phase error accumulates as the negative of frequency
 * offset, which is the plant the controller was derived against.
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include "unity.h"

#include "disc/disc.h"
#include "quality/quality.h"

/* ------------------------------------------------------------ plant model */

#define CENTER_CODE 2048.0
#define MAX_CODE 4095.0
#define FULL_SCALE_PPB 400.0
#define PPB_PER_CODE (FULL_SCALE_PPB / CENTER_CODE) /* 0.1953125 */

/*
 * A 10 MHz oscillator seen through the phase error the loop measures.
 *
 *   y_total = intrinsic + tempco*(T - T_ref) + (code - centre)*ppb_per_code
 *   de/dt   = -y_total            (disc.h: e = t_true - t_local, 1 ppb = 1 ns/s)
 *
 * `intrinsic` is the oscillator's own offset at the centre code — the thing
 * the loop has to discover and cancel.
 */
struct osc {
	double e_ns;
	double intrinsic_ppb;
	double drift_ppb_per_s;
	double tempco_ppb_per_c;
	double temp_c;
	double temp_ref_c;
};

static void osc_init(struct osc *p, double e0_ns, double intrinsic_ppb)
{
	memset(p, 0, sizeof(*p));
	p->e_ns = e0_ns;
	p->intrinsic_ppb = intrinsic_ppb;
	p->temp_c = 45.0;
	p->temp_ref_c = 45.0;
}

static double osc_commanded_ppb(uint16_t code)
{
	return ((double)code - CENTER_CODE) * PPB_PER_CODE;
}

static void osc_step(struct osc *p, uint16_t code, double dt)
{
	double y = p->intrinsic_ppb + osc_commanded_ppb(code) +
		   p->tempco_ppb_per_c * (p->temp_c - p->temp_ref_c);

	p->e_ns += -y * dt;
	p->intrinsic_ppb += p->drift_ppb_per_s * dt;
}

/* ---------------------------------------------------------------- helpers */

static double d_abs(double v)
{
	return (v < 0.0) ? -v : v;
}

static int64_t round_i64(double x)
{
	return (int64_t)((x >= 0.0) ? (x + 0.5) : (x - 0.5));
}

/*
 * Deterministic pseudo-noise. Twelve uniforms summed and centred (Irwin-Hall)
 * has variance exactly 1, so the requested sigma is exact — which matters for
 * the ADEV comparisons. No libm, no wall clock, replayable from the seed.
 */
static uint32_t noise_state = 1u;

static void noise_seed(uint32_t s)
{
	noise_state = s;
}

static double noise(double sigma)
{
	double acc = 0.0;
	int k;

	for (k = 0; k < 12; k++) {
		noise_state = noise_state * 1664525u + 1013904223u;
		acc += (double)(noise_state >> 8) / 16777216.0;
	}
	return (acc - 6.0) * sigma;
}

/*
 * Encode a measured phase error into a capture pair.
 *
 * Inverting disc.h's e = (expected - captured) * ns_per_count gives
 * captured = expected - e/ns_per_count, computed in uint32 so the counter wrap
 * is exercised for free.
 */
static void pps_from_error(disc_pps_t *p, double e_meas_ns, double ns_per_count,
			   uint32_t expected)
{
	int64_t d = round_i64(e_meas_ns / ns_per_count);

	memset(p, 0, sizeof(*p));
	p->primary_expected = expected;
	p->primary_count = (uint32_t)(expected - (uint32_t)(int64_t)d);
	p->primary_ns_per_count = (float)ns_per_count;
	p->primary_valid = true;
}

static void env_defaults(disc_env_t *env, uint64_t ms)
{
	memset(env, 0, sizeof(*env));
	env->mono_ms = ms;
	env->gnss_time_locked = true;
	env->osc_temp_mc = 45000;
	env->osc_temp_valid = true;
	/* Below the 700 mA warm threshold, so the oven reads warm after the
	 * 60 s dwell. */
	env->ocxo_current_ua = 450000;
	env->ocxo_current_valid = true;
	env->vc_sense_valid = false;
	env->anc.active_ref = (uint8_t)QUALITY_REF_OCXO;
	env->anc.gnss_fix = (uint8_t)QUALITY_GNSS_TIME_ONLY;
	env->anc.gnss_sv_used = 14u;
	env->anc.gnss_sv_visible = 20u;
	env->anc.gnss_tacc_ns = 12u;
	env->anc.utc_valid = true;
	env->anc.leap_current_s = 37;
}

/* One closed-loop second: measure the plant, tick, advance the plant. */
static disc_out_t loop_second(disc_ctx_t *ctx, struct osc *p, uint64_t *ms,
			      double sigma_ns, uint32_t *expected)
{
	disc_in_t in;
	disc_out_t out;

	*ms += 1000u;
	env_defaults(&in.env, *ms);
	in.env.osc_temp_mc = (int32_t)round_i64(p->temp_c * 1000.0);
	pps_from_error(&in.pps, p->e_ns + noise(sigma_ns), 1.0, *expected);
	*expected += 1000000u;

	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(ctx, &in, NULL, &out));
	osc_step(p, out.dac_code, 1.0);
	return out;
}

/* Run the loop until LOCKED, or fail. Returns the number of seconds taken. */
static unsigned int run_to_lock(disc_ctx_t *ctx, struct osc *p, uint64_t *ms,
				double sigma_ns, uint32_t *expected,
				unsigned int max_s)
{
	unsigned int i;

	for (i = 0u; i < max_s; i++) {
		disc_out_t out = loop_second(ctx, p, ms, sigma_ns, expected);

		if (out.state == DISC_STATE_LOCKED) {
			return i + 1u;
		}
	}
	TEST_FAIL_MESSAGE("loop never reached LOCKED");
	return max_s;
}

/* ------------------------------------------------------------ config/init */

static void test_defaults(void)
{
	disc_cfg_t cfg;

	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_cfg_defaults(NULL));
	TEST_ASSERT_EQUAL_INT(0, disc_cfg_defaults(&cfg));

	/* Spec §3.2/§3.3 and interface ref §3 values. */
	TEST_ASSERT_EQUAL_INT8(1, cfg.qerr_sign);
	TEST_ASSERT_EQUAL_UINT32(250u, cfg.xcheck_tol_ns);
	TEST_ASSERT_EQUAL_UINT8(8u, cfg.mad_window);
	TEST_ASSERT_EQUAL_FLOAT(5.0f, cfg.mad_k);
	TEST_ASSERT_EQUAL_FLOAT(150.0f, cfg.tau_s);
	TEST_ASSERT_EQUAL_FLOAT(400.0f, cfg.dac_full_scale_ppb);
	TEST_ASSERT_EQUAL_UINT16(2048u, cfg.dac_center_code);
	TEST_ASSERT_EQUAL_UINT16(4095u, cfg.dac_max_code);
	TEST_ASSERT_EQUAL_FLOAT(5.0f, cfg.slew_lsb_per_s);
	TEST_ASSERT_EQUAL_FLOAT(500.0f, cfg.lock_phase_ns);
	TEST_ASSERT_EQUAL_UINT8(30u, cfg.lock_dwell_s);
	TEST_ASSERT_EQUAL_FLOAT(50.0f, cfg.recover_ramp_max_ppb);
	TEST_ASSERT_EQUAL_UINT8(3u, cfg.pps_loss_ticks);
}

static void test_init_centres_the_actuator(void)
{
	disc_ctx_t ctx;

	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_init(NULL, NULL));
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));

	/* Interface ref §3: Vc centre is 1.65 V and PA4 must never float. */
	TEST_ASSERT_EQUAL_UINT16(2048u, ctx.dac_code);
	TEST_ASSERT_EQUAL_INT32(1650, ctx.vc_cmd_mv);
	TEST_ASSERT_EQUAL_INT(DISC_STATE_ACQUIRING, disc_state(&ctx));
	TEST_ASSERT_EQUAL_INT(DISC_STATE__COUNT, disc_state(NULL));

	/* Kp = 1/tau, Ki = 1/(a*tau^2) with the defaults tau=150, a=2. */
	TEST_ASSERT_FLOAT_WITHIN(1e-9f, 1.0f / 150.0f, ctx.kp);
	TEST_ASSERT_FLOAT_WITHIN(1e-12f, 1.0f / (2.0f * 150.0f * 150.0f), ctx.ki);
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, (float)PPB_PER_CODE, ctx.ppb_per_code);
}

static void test_init_rejects_bad_config(void)
{
	disc_ctx_t ctx;
	disc_cfg_t cfg;
	unsigned int i;

	/* Each entry breaks exactly one invariant the loop depends on. */
	for (i = 0u; i < 22u; i++) {
		(void)disc_cfg_defaults(&cfg);
		switch (i) {
		case 0: cfg.tau_s = 9.0f; break;             /* §3.3 range */
		case 1: cfg.tau_s = 1001.0f; break;
		case 2: cfg.pi_damping_a = 0.0f; break;
		case 3: cfg.pi_damping_a = INFINITY; break;
		case 4: cfg.fll_gain = 0.0f; break;
		case 5: cfg.fll_gain = 1.5f; break;
		case 6: cfg.qerr_sign = 0; break;
		case 7: cfg.mad_window = 2u; break;
		case 8: cfg.mad_window = DISC_MAD_WIN_MAX + 1u; break;
		case 9: cfg.mad_k = 0.0f; break;
		case 10: cfg.mad_floor_ns = -1.0f; break;
		case 11: cfg.max_consec_reject = 0u; break;
		case 12: cfg.lock_dwell_s = 1u; break;
		case 13: cfg.lock_dwell_s = DISC_LOCK_WIN_MAX + 1u; break;
		case 14: cfg.unlock_dwell_s = 0u; break;
		case 15: cfg.lock_phase_ns = 0.0f; break;
		/* re-entry must be above exit or the state machine oscillates */
		case 16: cfg.acq_reentry_ns = cfg.acq_exit_ns; break;
		case 17: cfg.dac_center_code = 0u; break;
		case 18: cfg.dac_center_code = 5000u; break;
		case 19: cfg.dac_full_scale_ppb = 0.0f; break;
		case 20: cfg.slew_lsb_per_s = 0.0f; break;
		case 21: cfg.pps_loss_ticks = 0u; break;
		default: break;
		}
		TEST_ASSERT_EQUAL_INT(-EINVAL, disc_init(&ctx, &cfg));
	}

	/* And a few more that do not fit the switch neatly. */
	(void)disc_cfg_defaults(&cfg);
	cfg.dac_max_code = 8u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_init(&ctx, &cfg));
	(void)disc_cfg_defaults(&cfg);
	cfg.dac_vref_mv = 0u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_init(&ctx, &cfg));
	(void)disc_cfg_defaults(&cfg);
	cfg.slew_acq_lsb_per_s = 0.0f;
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_init(&ctx, &cfg));
	(void)disc_cfg_defaults(&cfg);
	cfg.vc_fault_dwell_s = 0u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_init(&ctx, &cfg));
	(void)disc_cfg_defaults(&cfg);
	cfg.demote_threshold_ns = 0.0f;
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_init(&ctx, &cfg));
	(void)disc_cfg_defaults(&cfg);
	cfg.base_disp_ns = -1.0f;
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_init(&ctx, &cfg));
	(void)disc_cfg_defaults(&cfg);
	cfg.lock_var_ns2 = 0.0f;
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_init(&ctx, &cfg));
	(void)disc_cfg_defaults(&cfg);
	cfg.acq_exit_ns = 0.0f;
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_init(&ctx, &cfg));
	(void)disc_cfg_defaults(&cfg);
	cfg.acq_ramp_max_ppb = 0.0f;
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_init(&ctx, &cfg));
	(void)disc_cfg_defaults(&cfg);
	cfg.recover_ramp_max_ppb = 0.0f;
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_init(&ctx, &cfg));
}

static void test_api_rejects_null_and_uninitialised(void)
{
	disc_ctx_t ctx;
	disc_in_t in;
	disc_out_t out;
	uint16_t code;

	memset(&ctx, 0, sizeof(ctx)); /* initialised == false */
	env_defaults(&in.env, 1000u);
	pps_from_error(&in.pps, 0.0, 1.0, 1000u);

	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_tick_no_pps(&ctx, &in.env, NULL, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_park(&ctx, &code));
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_unpark(&ctx));

	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_tick_pps(NULL, &in, NULL, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_tick_pps(&ctx, NULL, NULL, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_tick_no_pps(NULL, &in.env, NULL, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_tick_no_pps(&ctx, NULL, NULL, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_park(NULL, &code));
	TEST_ASSERT_EQUAL_INT(-EINVAL, disc_unpark(NULL));
	/* NULL out and NULL quality slot are both legal. */
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(0, disc_tick_no_pps(&ctx, &in.env, NULL, NULL));
}

/* -------------------------------------------------- §3.2 sign conventions */

static void test_sign_convention_late_clock_is_positive(void)
{
	disc_ctx_t ctx;
	disc_in_t in;
	disc_out_t out;

	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	env_defaults(&in.env, 1000u);

	/*
	 * The true second arrives at count 900 but the local clock predicted
	 * 1000: it has not reached its own mark yet, so it is LATE and e is
	 * +100 ns. Getting this backwards inverts the whole loop.
	 */
	memset(&in.pps, 0, sizeof(in.pps));
	in.pps.primary_expected = 1000u;
	in.pps.primary_count = 900u;
	in.pps.primary_ns_per_count = 1.0f;
	in.pps.primary_valid = true;

	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_TRUE(out.sample_accepted);
	TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, out.phase_err_ns);

	/* A late clock must be told to speed up: the command goes positive,
	 * i.e. the DAC code rises above centre. */
	TEST_ASSERT_TRUE(out.dac_code > 2048u);
	TEST_ASSERT_TRUE(out.applied_ppb > 0.0f);
}

static void test_sign_convention_early_clock_is_negative(void)
{
	disc_ctx_t ctx;
	disc_in_t in;
	disc_out_t out;

	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	env_defaults(&in.env, 1000u);

	memset(&in.pps, 0, sizeof(in.pps));
	in.pps.primary_expected = 1000u;
	in.pps.primary_count = 1100u;
	in.pps.primary_ns_per_count = 1.0f;
	in.pps.primary_valid = true;

	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_FLOAT_WITHIN(0.01f, -100.0f, out.phase_err_ns);
	TEST_ASSERT_TRUE(out.dac_code < 2048u);
	TEST_ASSERT_TRUE(out.applied_ppb < 0.0f);
}

static void test_capture_difference_survives_the_counter_wrap(void)
{
	disc_ctx_t ctx;
	disc_in_t in;
	disc_out_t out;

	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	env_defaults(&in.env, 1000u);

	/* TIM2 wraps every 17.18 s at 250 MHz, so a capture straddling the
	 * wrap is routine, not exotic. expected=5, captured=0xFFFFFFF0 is a
	 * difference of +21 counts, not -4294967275. */
	memset(&in.pps, 0, sizeof(in.pps));
	in.pps.primary_expected = 5u;
	in.pps.primary_count = 0xFFFFFFF0u;
	in.pps.primary_ns_per_count = 4.0f; /* TIM2 at 250 MHz */
	in.pps.primary_valid = true;

	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_FLOAT_WITHIN(0.01f, 84.0f, out.phase_err_ns); /* 21 * 4 */

	/* The other side of the wrap. */
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	in.env.mono_ms = 2000u;
	in.pps.primary_expected = 0xFFFFFFF0u;
	in.pps.primary_count = 5u;
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_FLOAT_WITHIN(0.01f, -84.0f, out.phase_err_ns);

	/* The exact half-way point, where a naive negation would overflow. */
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	in.env.mono_ms = 3000u;
	in.pps.primary_expected = 0x80000000u;
	in.pps.primary_count = 0u;
	in.pps.primary_ns_per_count = 1.0f;
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_TRUE(out.phase_err_ns < -2.0e9f);
}

static void test_sawtooth_and_delays_are_applied_with_the_right_sign(void)
{
	disc_ctx_t ctx;
	disc_cfg_t cfg;
	disc_in_t in;
	disc_out_t out;

	(void)disc_cfg_defaults(&cfg);
	cfg.cable_delay_ns = 60;  /* 12 m of RG-58 at 0.66c */
	cfg.board_delay_ns = 4;
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));
	env_defaults(&in.env, 1000u);

	/*
	 * Raw error 100 ns, plus a +7500 ps sawtooth and 64 ns of propagation:
	 * every correction refers the measured edge backwards in time, so
	 * e = 100 + 7.5 + 60 + 4 = 171.5 ns.
	 */
	memset(&in.pps, 0, sizeof(in.pps));
	in.pps.primary_expected = 1000u;
	in.pps.primary_count = 900u;
	in.pps.primary_ns_per_count = 1.0f;
	in.pps.primary_valid = true;
	in.pps.qerr_ps = 7500;
	in.pps.qerr_valid = true;

	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_FLOAT_WITHIN(0.01f, 171.5f, out.phase_err_ns);

	/* An unusable TIM-TP message leaves the sawtooth uncorrected. */
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));
	in.env.mono_ms = 2000u;
	in.pps.qerr_valid = false;
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_FLOAT_WITHIN(0.01f, 164.0f, out.phase_err_ns);

	/* The polarity is a configuration bit pending bench confirmation. */
	cfg.qerr_sign = -1;
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));
	in.env.mono_ms = 3000u;
	in.pps.qerr_valid = true;
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_FLOAT_WITHIN(0.01f, 156.5f, out.phase_err_ns);
}

static void test_sawtooth_correction_shrinks_the_residual(void)
{
	/*
	 * Differential test: the same deterministic sawtooth, once corrected
	 * and once not. "Mandatory" in §3.2 is a claim about the residual, not
	 * about whether the field is read.
	 *
	 * The sawtooth is a 600 s triangle of +-60 ns. The period matters: the
	 * receiver's quantisation error walks slowly as its internal clock
	 * beats against the second, and it is precisely the components *inside*
	 * the loop bandwidth that the controller cannot filter out and will
	 * instead faithfully steer the oscillator to follow. A fast dither
	 * would be rejected by the loop whether it was corrected or not, and
	 * would prove nothing.
	 */
	unsigned int pass;
	double var[2];

	for (pass = 0u; pass < 2u; pass++) {
		disc_ctx_t ctx;
		disc_cfg_t cfg;
		struct osc p;
		uint64_t ms = 0u;
		uint32_t expected = 0x20000000u;
		unsigned int i;
		double acc = 0.0;
		unsigned int n = 0u;

		(void)disc_cfg_defaults(&cfg);
		cfg.tau_s = 50.0f;
		TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));
		osc_init(&p, 0.0, -20.0);

		for (i = 0u; i < 1800u; i++) {
			disc_in_t in;
			disc_out_t out;
			unsigned int ph = i % 600u;
			double saw_ns = (ph < 300u)
						? (-60.0 + (120.0 * (double)ph) /
								   300.0)
						: (60.0 -
						   (120.0 * (double)(ph - 300u)) /
							   300.0);

			ms += 1000u;
			env_defaults(&in.env, ms);
			/* The receiver's pulse is late by qErr, so the measured
			 * raw error is the true error minus qErr. */
			pps_from_error(&in.pps, p.e_ns - saw_ns, 1.0, expected);
			expected += 1000000u;
			in.pps.qerr_ps = (int32_t)round_i64(saw_ns * 1000.0);
			in.pps.qerr_valid = (pass == 0u);

			TEST_ASSERT_EQUAL_INT(0,
					      disc_tick_pps(&ctx, &in, NULL, &out));
			osc_step(&p, out.dac_code, 1.0);

			if (i >= 600u) {
				acc += p.e_ns * p.e_ns;
				n++;
			}
		}
		var[pass] = acc / (double)n;
	}

	/* Corrected must beat uncorrected by a wide margin. These are
	 * variances, so a factor of 100 here is a factor of 10 in RMS. */
	TEST_ASSERT_TRUE(var[0] * 100.0 < var[1]);
}

/* ------------------------------------------------------- §3.2 cross-check */

static void test_dual_capture_is_averaged_when_the_channels_agree(void)
{
	disc_ctx_t ctx;
	disc_in_t in;
	disc_out_t out;

	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	env_defaults(&in.env, 1000u);

	memset(&in.pps, 0, sizeof(in.pps));
	in.pps.primary_expected = 1000u;
	in.pps.primary_count = 900u; /* +100 ns */
	in.pps.primary_ns_per_count = 1.0f;
	in.pps.primary_valid = true;
	in.pps.secondary_expected = 2000u;
	in.pps.secondary_count = 1800u; /* +200 ns */
	in.pps.secondary_ns_per_count = 1.0f;
	in.pps.secondary_valid = true;

	/* 100 ns apart is inside the 250 ns tolerance, so the two independent
	 * captures of one edge are averaged. */
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_FLOAT_WITHIN(0.01f, 150.0f, out.phase_err_ns);
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_PPS_DIVERGE) == 0u);
}

static void test_channel_divergence_is_flagged_and_the_primary_wins(void)
{
	disc_ctx_t ctx;
	disc_in_t in;
	disc_out_t out;

	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	env_defaults(&in.env, 1000u);

	memset(&in.pps, 0, sizeof(in.pps));
	in.pps.primary_expected = 1000u;
	in.pps.primary_count = 900u; /* +100 ns */
	in.pps.primary_ns_per_count = 1.0f;
	in.pps.primary_valid = true;
	in.pps.secondary_expected = 2000u;
	in.pps.secondary_count = 1000u; /* +1000 ns: 900 ns apart */
	in.pps.secondary_ns_per_count = 1.0f;
	in.pps.secondary_valid = true;

	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_PPS_DIVERGE) != 0u);
	/* PA0 is the primary; a disagreeing PC6 does not get a vote. */
	TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, out.phase_err_ns);
	TEST_ASSERT_TRUE(out.sample_accepted);
}

static void test_secondary_alone_is_used(void)
{
	disc_ctx_t ctx;
	disc_in_t in;
	disc_out_t out;

	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	env_defaults(&in.env, 1000u);

	memset(&in.pps, 0, sizeof(in.pps));
	in.pps.secondary_expected = 2000u;
	in.pps.secondary_count = 1800u;
	in.pps.secondary_ns_per_count = 1.0f;
	in.pps.secondary_valid = true;

	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_TRUE(out.sample_accepted);
	TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, out.phase_err_ns);
}

static void test_no_usable_capture_is_a_missed_second(void)
{
	disc_ctx_t ctx;
	disc_in_t in;
	disc_out_t out;

	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	env_defaults(&in.env, 1000u);

	memset(&in.pps, 0, sizeof(in.pps));
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_FALSE(out.sample_accepted);
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_NO_PPS) != 0u);
	TEST_ASSERT_EQUAL_UINT16(2048u, out.dac_code);

	/* A garbage scale is as unusable as a missing capture. */
	in.env.mono_ms = 2000u;
	in.pps.primary_valid = true;
	in.pps.primary_ns_per_count = INFINITY;
	in.pps.secondary_valid = true;
	in.pps.secondary_ns_per_count = NAN;
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_FALSE(out.sample_accepted);
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_NO_PPS) != 0u);

	/* A finite but absurd scale that overflows the product is caught after
	 * the corrections rather than propagating an infinity into the loop. */
	in.env.mono_ms = 3000u;
	in.pps.primary_ns_per_count = 1.0e30f;
	in.pps.primary_expected = 0u;
	in.pps.primary_count = 0x40000000u;
	in.pps.secondary_valid = false;
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_FALSE(out.sample_accepted);
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_NO_PPS) != 0u);
	TEST_ASSERT_EQUAL_UINT16(2048u, out.dac_code);
}

/* --------------------------------------------------------- §3.2 MAD gate */

static void test_single_glitch_is_rejected_and_the_loop_is_undisturbed(void)
{
	disc_ctx_t ctx;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0x30000000u;
	unsigned int i;
	disc_out_t out;
	uint16_t code_before;
	disc_in_t in;
	double e_before;

	noise_seed(7u);
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	osc_init(&p, 0.0, -25.0);
	(void)run_to_lock(&ctx, &p, &ms, 20.0, &expected, 900u);

	for (i = 0u; i < 30u; i++) {
		out = loop_second(&ctx, &p, &ms, 20.0, &expected);
	}
	code_before = out.dac_code;
	e_before = p.e_ns;

	/* One 10 us multipath glitch. It must not reach the loop at all. */
	ms += 1000u;
	env_defaults(&in.env, ms);
	pps_from_error(&in.pps, p.e_ns + 10000.0, 1.0, expected);
	expected += 1000000u;
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));

	TEST_ASSERT_FALSE(out.sample_accepted);
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_PPS_REJECT) != 0u);
	TEST_ASSERT_EQUAL_UINT16(code_before, out.dac_code);
	TEST_ASSERT_EQUAL_INT(DISC_STATE_LOCKED, out.state);
	osc_step(&p, out.dac_code, 1.0);

	/* And the loop carries on as if nothing happened. */
	for (i = 0u; i < 60u; i++) {
		out = loop_second(&ctx, &p, &ms, 20.0, &expected);
	}
	TEST_ASSERT_EQUAL_INT(DISC_STATE_LOCKED, out.state);
	TEST_ASSERT_TRUE(d_abs(p.e_ns - e_before) < 500.0);
}

static void test_a_real_step_is_eventually_accepted(void)
{
	disc_ctx_t ctx;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0x31000000u;
	unsigned int i;
	unsigned int rejects = 0u;
	disc_out_t out;

	/*
	 * The gate must not be able to lock the loop out of its own input. A
	 * sustained offset (a re-cabled antenna, a corrected cable-delay
	 * constant) is rejected a few times and then the window is rebuilt
	 * around the new reality.
	 */
	noise_seed(11u);
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	osc_init(&p, 0.0, -25.0);
	(void)run_to_lock(&ctx, &p, &ms, 10.0, &expected, 900u);

	p.e_ns += 5000.0; /* a genuine 5 us step in the reference */

	for (i = 0u; i < 12u; i++) {
		out = loop_second(&ctx, &p, &ms, 10.0, &expected);
		if (!out.sample_accepted) {
			rejects++;
		}
	}
	/* Some rejection is correct; permanent rejection is not. */
	TEST_ASSERT_TRUE(rejects >= 1u);
	TEST_ASSERT_TRUE(rejects <= 6u);
	TEST_ASSERT_TRUE(out.sample_accepted);
}

/* ------------------------------------------------------ §3.3 acquisition */

static void test_acquisition_from_100us_converges_and_locks(void)
{
	disc_ctx_t ctx;
	disc_cfg_t cfg;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0x40000000u;
	unsigned int locked_at;
	unsigned int i;
	disc_out_t out;

	(void)disc_cfg_defaults(&cfg);
	cfg.tau_s = 50.0f;
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));

	/* 100 us of initial phase error and 50 ppb of oscillator offset. The
	 * pull range is +-400 ppb, so the first 250 s or so are actuator-
	 * limited however clever the controller is. */
	noise_seed(3u);
	osc_init(&p, 100000.0, -50.0);

	locked_at = run_to_lock(&ctx, &p, &ms, 5.0, &expected, 1500u);
	TEST_ASSERT_TRUE(locked_at > 250u);  /* cannot beat the pull rate */
	TEST_ASSERT_TRUE(locked_at < 1200u);
	TEST_ASSERT_TRUE(d_abs(p.e_ns) < 500.0);

	/* It stays locked and keeps closing. */
	for (i = 0u; i < 400u; i++) {
		out = loop_second(&ctx, &p, &ms, 5.0, &expected);
	}
	TEST_ASSERT_EQUAL_INT(DISC_STATE_LOCKED, out.state);
	TEST_ASSERT_TRUE(d_abs(p.e_ns) < 100.0);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_STRATUM_PRIMARY, out.stratum);
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_OCXO_WARM) != 0u);

	/* The FLL found the oscillator's own offset, so the steady-state
	 * command is +50 ppb: code 2048 + 50/0.1953 = 2304. */
	TEST_ASSERT_TRUE(out.dac_code > 2294u);
	TEST_ASSERT_TRUE(out.dac_code < 2314u);
}

static void test_lock_requires_warm_and_gnss(void)
{
	disc_ctx_t ctx;
	disc_in_t in;
	disc_out_t out;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0x41000000u;
	unsigned int i;

	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	osc_init(&p, 0.0, -10.0);
	noise_seed(5u);

	/* A perfect phase error is not enough while the oven is still drawing
	 * warm-up current (§3.3 "OCXO warm"). warm_timeout_s is disabled here
	 * so the sensors are the only evidence. */
	ctx.cfg.warm_timeout_s = 0u;
	for (i = 0u; i < 300u; i++) {
		ms += 1000u;
		env_defaults(&in.env, ms);
		in.env.ocxo_current_ua = 1200000; /* oven heating */
		pps_from_error(&in.pps, p.e_ns + noise(5.0), 1.0, expected);
		expected += 1000000u;
		TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
		osc_step(&p, out.dac_code, 1.0);
	}
	TEST_ASSERT_NOT_EQUAL_INT(DISC_STATE_LOCKED, out.state);
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_OCXO_WARM) == 0u);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_STRATUM_UNSYNC, out.stratum);

	/* Oven up: the dwell starts, then lock follows. */
	for (i = 0u; i < 200u; i++) {
		out = loop_second(&ctx, &p, &ms, 5.0, &expected);
	}
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_OCXO_WARM) != 0u);
	TEST_ASSERT_EQUAL_INT(DISC_STATE_LOCKED, out.state);
}

static void test_warm_timeout_backstops_dead_sensors(void)
{
	disc_ctx_t ctx;
	disc_cfg_t cfg;
	disc_in_t in;
	disc_out_t out;
	unsigned int i;
	uint64_t ms = 0u;

	/* Neither sensor reports. The loop must still be able to declare the
	 * oven warm eventually, or a failed TMP117 would cost stratum-1
	 * forever. */
	(void)disc_cfg_defaults(&cfg);
	cfg.warm_timeout_s = 30u;
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));

	for (i = 0u; i < 20u; i++) {
		ms += 1000u;
		env_defaults(&in.env, ms);
		in.env.osc_temp_valid = false;
		in.env.ocxo_current_valid = false;
		TEST_ASSERT_EQUAL_INT(0,
				      disc_tick_no_pps(&ctx, &in.env, NULL, &out));
	}
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_OCXO_WARM) == 0u);

	for (i = 0u; i < 20u; i++) {
		ms += 1000u;
		env_defaults(&in.env, ms);
		in.env.osc_temp_valid = false;
		in.env.ocxo_current_valid = false;
		TEST_ASSERT_EQUAL_INT(0,
				      disc_tick_no_pps(&ctx, &in.env, NULL, &out));
	}
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_OCXO_WARM) != 0u);
}

static void test_steady_state_noise_rejection_bounds_the_actuator(void)
{
	disc_ctx_t ctx;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0x50000000u;
	unsigned int i;
	uint16_t prev;
	int max_step = 0;
	double worst_e = 0.0;
	disc_out_t out;

	noise_seed(17u);
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	osc_init(&p, 0.0, -30.0);
	(void)run_to_lock(&ctx, &p, &ms, 30.0, &expected, 1500u);

	prev = ctx.dac_code;
	for (i = 0u; i < 600u; i++) {
		int step;

		out = loop_second(&ctx, &p, &ms, 30.0, &expected);
		step = (int)out.dac_code - (int)prev;
		if (step < 0) {
			step = -step;
		}
		if (step > max_step) {
			max_step = step;
		}
		prev = out.dac_code;
		/* Measured over the second half only: LOCKED is declared as
		 * soon as the criteria hold, so the first few hundred seconds
		 * still carry the tail of the acquisition transient and say
		 * nothing about noise rejection. */
		if (i >= 300u && d_abs(p.e_ns) > worst_e) {
			worst_e = d_abs(p.e_ns);
		}
	}

	/* 30 ns rms of PPS noise must not become a frequency step: the §3.3
	 * slew limit is 5 LSB/s and the loop must respect it. */
	TEST_ASSERT_TRUE(max_step <= 5);
	/* And the loop stays locked, with a true phase error far below the
	 * per-sample measurement noise — that is what the loop filter is for. */
	TEST_ASSERT_EQUAL_INT(DISC_STATE_LOCKED, out.state);
	TEST_ASSERT_TRUE(worst_e < 200.0);
	/* The published statistics track the injected noise. */
	TEST_ASSERT_TRUE(ctx.pps_sigma_ns > 15.0f);
	TEST_ASSERT_TRUE(ctx.pps_sigma_ns < 50.0f);
}

/* ----------------------------------------------- §3.3 clamp / anti-windup */

static void test_beyond_pull_range_saturates_cleanly(void)
{
	disc_ctx_t ctx;
	disc_cfg_t cfg;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0x60000000u;
	unsigned int i;
	disc_out_t out;
	unsigned int locked_at;

	/*
	 * An oscillator 900 ppb off cannot be corrected by a +-400 ppb pull.
	 * The actuator must sit on the rail, say so, and — crucially — the
	 * frequency state must not run away, so that the loop converges
	 * promptly once the oscillator comes back into range.
	 */
	(void)disc_cfg_defaults(&cfg);
	cfg.tau_s = 30.0f;
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));
	noise_seed(23u);
	osc_init(&p, 0.0, -900.0);

	for (i = 0u; i < 400u; i++) {
		out = loop_second(&ctx, &p, &ms, 2.0, &expected);
	}
	TEST_ASSERT_EQUAL_UINT16(4095u, out.dac_code);
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_DAC_SAT) != 0u);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_STRATUM_UNSYNC, out.stratum);
	/* The estimate is allowed to say "I need 900 ppb" — that is true and
	 * useful — but it must stay a bounded, physical number. */
	TEST_ASSERT_TRUE(ctx.y_int < 1200.0f);

	/* Oscillator trimmed back into range: convergence, with no residue of
	 * the saturated interval driving an overshoot. */
	p.intrinsic_ppb = -100.0;
	locked_at = run_to_lock(&ctx, &p, &ms, 2.0, &expected, 1500u);
	TEST_ASSERT_TRUE(locked_at < 1200u);

	for (i = 0u; i < 300u; i++) {
		out = loop_second(&ctx, &p, &ms, 2.0, &expected);
		TEST_ASSERT_TRUE(d_abs(p.e_ns) < 2000.0);
	}
	TEST_ASSERT_EQUAL_INT(DISC_STATE_LOCKED, out.state);
	/* Steady state is +100 ppb: 2048 + 100/0.1953 = 2560. */
	TEST_ASSERT_TRUE(out.dac_code > 2545u);
	TEST_ASSERT_TRUE(out.dac_code < 2575u);
}

static void test_pi_integral_does_not_wind_up_against_the_rail(void)
{
	disc_ctx_t ctx;
	disc_cfg_t cfg;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0x61000000u;
	unsigned int i;
	disc_out_t out;

	float y_int_when_railed;

	/*
	 * Keep the loop in the PI state throughout (re-entry to ACQUIRING
	 * disabled) so this exercises the Ki integrator specifically rather
	 * than the FLL, which only runs in ACQUIRING and RECOVERING.
	 *
	 * The oscillator is walked out of range rather than stepped: an abrupt
	 * 875 ppb step trips the median/MAD gate for several seconds running,
	 * which is a holdover trigger (§3.6 "PPS outliers") and would put the
	 * loop into the FLL-driven recovery path instead. A ramp is also the
	 * physically realistic failure — an oscillator does not teleport.
	 */
	(void)disc_cfg_defaults(&cfg);
	cfg.tau_s = 30.0f;
	cfg.acq_reentry_ns = 1.0e9f;
	cfg.acq_exit_ns = 1.0e8f;
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));
	noise_seed(29u);
	osc_init(&p, 0.0, -25.0);
	(void)run_to_lock(&ctx, &p, &ms, 2.0, &expected, 900u);

	/* Walk to 900 ppb — more than twice the pull range — over 175 s, then
	 * hold there. Without conditional integration the integral would keep
	 * accumulating Ki*e*dt against a phase error growing at ~875 ns/s for
	 * the next ten minutes. */
	for (i = 0u; i < 200u; i++) {
		if (p.intrinsic_ppb > -900.0) {
			p.intrinsic_ppb -= 5.0;
		}
		out = loop_second(&ctx, &p, &ms, 2.0, &expected);
	}
	TEST_ASSERT_EQUAL_UINT16(4095u, out.dac_code);
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_DAC_SAT) != 0u);
	TEST_ASSERT_NOT_EQUAL_INT(DISC_STATE_ACQUIRING, out.state);
	TEST_ASSERT_NOT_EQUAL_INT(DISC_STATE_RECOVERING, out.state);
	y_int_when_railed = ctx.y_int;

	for (i = 0u; i < 600u; i++) {
		out = loop_second(&ctx, &p, &ms, 2.0, &expected);
	}
	/* Ten more minutes on the rail with a phase error in the hundreds of
	 * microseconds: the integral must not have moved at all. */
	TEST_ASSERT_EQUAL_UINT16(4095u, out.dac_code);
	TEST_ASSERT_TRUE(d_abs((double)p.e_ns) > 100000.0);
	TEST_ASSERT_FLOAT_WITHIN(0.01f, y_int_when_railed, ctx.y_int);
	/* Bounded by the achievable command, not by how long we sat there. */
	TEST_ASSERT_TRUE(ctx.y_int < 500.0f);

	/* Oscillator repaired: the loop must come back without a wound-up
	 * integral to unwind first. */
	p.intrinsic_ppb = -25.0;
	for (i = 0u; i < 2500u; i++) {
		out = loop_second(&ctx, &p, &ms, 2.0, &expected);
	}
	TEST_ASSERT_TRUE(d_abs(p.e_ns) < 1000.0);
}

static void test_a_nonsense_tempco_constant_cannot_reach_the_dac(void)
{
	disc_ctx_t ctx;
	disc_cfg_t cfg;
	disc_in_t in;
	disc_out_t out;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0x62000000u;
	uint16_t code_before;

	/* A corrupt calibration constant read back from NVS must not be able
	 * to put a NaN or an infinity on PA4. */
	(void)disc_cfg_defaults(&cfg);
	cfg.tempco_enable = true;
	cfg.tempco_ppb_per_c = 3.0e38f;
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));
	noise_seed(31u);
	osc_init(&p, 0.0, -20.0);
	(void)run_to_lock(&ctx, &p, &ms, 2.0, &expected, 900u);
	code_before = ctx.dac_code;

	ms += 1000u;
	env_defaults(&in.env, ms);
	in.env.osc_temp_mc = 55000; /* 10 C from the reference */
	pps_from_error(&in.pps, p.e_ns, 1.0, expected);
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));

	TEST_ASSERT_TRUE(out.dac_code <= 4095u);
	TEST_ASSERT_TRUE(isfinite(out.applied_ppb));
	/* Held at the last good command rather than slammed to a rail. */
	TEST_ASSERT_TRUE((int)out.dac_code - (int)code_before <= 5);
	TEST_ASSERT_TRUE((int)code_before - (int)out.dac_code <= 5);
}

/* ------------------------------------------------------------- §3.6 holdover */

static void test_missed_pulses_enter_holdover_and_freeze_the_actuator(void)
{
	disc_ctx_t ctx;
	disc_cfg_t cfg;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0x70000000u;
	unsigned int i;
	disc_out_t out;
	disc_in_t in;
	uint16_t frozen;

	(void)disc_cfg_defaults(&cfg);
	cfg.holdover.base_ns = 100.0f;
	cfg.holdover.drift_ns_per_s = 2.0f;
	cfg.demote_threshold_ns = 5000.0f;
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));
	noise_seed(37u);
	osc_init(&p, 0.0, -30.0);
	(void)run_to_lock(&ctx, &p, &ms, 10.0, &expected, 1500u);
	frozen = ctx.dac_code;

	/* Two missed seconds are tolerated; the third crosses pps_loss_ticks. */
	for (i = 0u; i < 2u; i++) {
		ms += 1000u;
		env_defaults(&in.env, ms);
		TEST_ASSERT_EQUAL_INT(0,
				      disc_tick_no_pps(&ctx, &in.env, NULL, &out));
		TEST_ASSERT_EQUAL_INT(DISC_STATE_LOCKED, out.state);
		TEST_ASSERT_FALSE(out.holdover);
	}

	ms += 1000u;
	env_defaults(&in.env, ms);
	TEST_ASSERT_EQUAL_INT(0, disc_tick_no_pps(&ctx, &in.env, NULL, &out));
	TEST_ASSERT_EQUAL_INT(DISC_STATE_HOLDOVER, out.state);
	TEST_ASSERT_TRUE(out.holdover);
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_HOLDOVER) != 0u);
	TEST_ASSERT_EQUAL_UINT16(frozen, out.dac_code);

	/*
	 * Estimator growth: base 100 ns plus 2 ns/s. At 200 s that is 500 ns,
	 * and the 5 us budget is reached at (5000-100)/2 = 2450 s.
	 */
	for (i = 0u; i < 200u; i++) {
		ms += 1000u;
		env_defaults(&in.env, ms);
		TEST_ASSERT_EQUAL_INT(0,
				      disc_tick_no_pps(&ctx, &in.env, NULL, &out));
		/* §3.6: the actuator is frozen for the whole of holdover. */
		TEST_ASSERT_EQUAL_UINT16(frozen, out.dac_code);
	}
	TEST_ASSERT_EQUAL_UINT32(200u, ctx.holdover_elapsed_s);
	TEST_ASSERT_INT64_WITHIN(5, 500, out.holdover_est_err_ns);
	TEST_ASSERT_UINT32_WITHIN(5u, 2250u, out.holdover_t_demote_s);
	/* Still inside policy, so still serving. */
	TEST_ASSERT_EQUAL_UINT8(QUALITY_STRATUM_PRIMARY, out.stratum);
}

static void test_holdover_demotes_when_the_budget_is_spent(void)
{
	disc_ctx_t ctx;
	disc_cfg_t cfg;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0x71000000u;
	unsigned int i;
	disc_out_t out;
	disc_in_t in;

	(void)disc_cfg_defaults(&cfg);
	cfg.holdover.base_ns = 100.0f;
	cfg.holdover.drift_ns_per_s = 10.0f;
	cfg.demote_threshold_ns = 1000.0f; /* reached at 90 s */
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));
	noise_seed(41u);
	osc_init(&p, 0.0, -30.0);
	(void)run_to_lock(&ctx, &p, &ms, 10.0, &expected, 1500u);

	for (i = 0u; i < 80u; i++) {
		ms += 1000u;
		env_defaults(&in.env, ms);
		TEST_ASSERT_EQUAL_INT(0,
				      disc_tick_no_pps(&ctx, &in.env, NULL, &out));
	}
	TEST_ASSERT_EQUAL_INT(DISC_STATE_HOLDOVER, out.state);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_STRATUM_PRIMARY, out.stratum);
	TEST_ASSERT_TRUE(out.holdover_t_demote_s > 0u);

	for (i = 0u; i < 40u; i++) {
		ms += 1000u;
		env_defaults(&in.env, ms);
		TEST_ASSERT_EQUAL_INT(0,
				      disc_tick_no_pps(&ctx, &in.env, NULL, &out));
	}
	/* §3.6: advance stratum once dispersion exceeds policy. */
	TEST_ASSERT_EQUAL_UINT8(QUALITY_STRATUM_UNSYNC, out.stratum);
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_DEMOTED) != 0u);
	TEST_ASSERT_EQUAL_UINT32(0u, out.holdover_t_demote_s);
}

static void test_holdover_temperature_term_and_unreachable_budget(void)
{
	disc_ctx_t ctx;
	disc_cfg_t cfg;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0x72000000u;
	unsigned int i;
	disc_out_t out;
	disc_in_t in;

	/* A model with no drift at all can never spend the budget: the
	 * time-to-demotion must read "not on the horizon", not zero. */
	(void)disc_cfg_defaults(&cfg);
	cfg.holdover.base_ns = 50.0f;
	cfg.holdover.drift_ns_per_s = 0.0f;
	cfg.holdover.temp_ns_per_s_per_c = 1.0f;
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));
	noise_seed(43u);
	osc_init(&p, 0.0, -30.0);
	(void)run_to_lock(&ctx, &p, &ms, 10.0, &expected, 1500u);

	for (i = 0u; i < 10u; i++) {
		ms += 1000u;
		env_defaults(&in.env, ms);
		TEST_ASSERT_EQUAL_INT(0,
				      disc_tick_no_pps(&ctx, &in.env, NULL, &out));
	}
	TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, out.holdover_t_demote_s);
	TEST_ASSERT_INT64_WITHIN(1, 50, out.holdover_est_err_ns);

	/* Now let the enclosure move 4 C from where holdover started: the
	 * temperature term adds 4 ns/s. After another 100 s the estimate is
	 * 50 + 4*100 = 450 ns. */
	for (i = 0u; i < 100u; i++) {
		ms += 1000u;
		env_defaults(&in.env, ms);
		in.env.osc_temp_mc = 49000;
		TEST_ASSERT_EQUAL_INT(0,
				      disc_tick_no_pps(&ctx, &in.env, NULL, &out));
	}
	TEST_ASSERT_INT64_WITHIN(20, 490, out.holdover_est_err_ns);
	/* Finite now, and enormous rather than clamped to zero. */
	TEST_ASSERT_TRUE(out.holdover_t_demote_s > 100000u);

	/* A missing temperature reading falls back to no excursion. */
	ms += 1000u;
	env_defaults(&in.env, ms);
	in.env.osc_temp_valid = false;
	TEST_ASSERT_EQUAL_INT(0, disc_tick_no_pps(&ctx, &in.env, NULL, &out));
	TEST_ASSERT_INT64_WITHIN(1, 50, out.holdover_est_err_ns);
}

static void test_gnss_time_unlock_forces_holdover_even_with_pulses(void)
{
	disc_ctx_t ctx;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0x73000000u;
	disc_out_t out;
	disc_in_t in;

	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	noise_seed(47u);
	osc_init(&p, 0.0, -30.0);
	(void)run_to_lock(&ctx, &p, &ms, 10.0, &expected, 1500u);

	/* Pulses keep arriving but the receiver says its time is not usable.
	 * A pulse without a valid timescale behind it is not a reference. */
	ms += 1000u;
	env_defaults(&in.env, ms);
	in.env.gnss_time_locked = false;
	pps_from_error(&in.pps, p.e_ns, 1.0, expected);
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));

	TEST_ASSERT_EQUAL_INT(DISC_STATE_HOLDOVER, out.state);
	TEST_ASSERT_FALSE(out.sample_accepted);
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_GNSS_TIME_LOCKED) == 0u);
}

static void test_recovery_is_rate_limited_and_never_steps(void)
{
	unsigned int pass;
	double max_rate[2];
	double final_e[2];

	/*
	 * Differential: the same holdover excursion recovered with the §3.6
	 * rate limit at its default 50 ppb, and with it opened right up. Both
	 * must converge; only the limited one may bound the phase-correction
	 * rate a client would see.
	 */
	for (pass = 0u; pass < 2u; pass++) {
		disc_ctx_t ctx;
		disc_cfg_t cfg;
		struct osc p;
		uint64_t ms = 0u;
		uint32_t expected = 0x80000000u;
		unsigned int i;
		disc_out_t out;
		disc_in_t in;
		double prev_e;
		double worst = 0.0;

		(void)disc_cfg_defaults(&cfg);
		cfg.recover_ramp_max_ppb = (pass == 0u) ? 50.0f : 400.0f;
		TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));
		noise_seed(53u);
		osc_init(&p, 0.0, -30.0);
		(void)run_to_lock(&ctx, &p, &ms, 5.0, &expected, 1500u);

		/* 500 s of holdover during which the oscillator wanders by
		 * 40 ppb, accumulating about 20 us of phase error. */
		p.intrinsic_ppb -= 40.0;
		for (i = 0u; i < 500u; i++) {
			ms += 1000u;
			env_defaults(&in.env, ms);
			TEST_ASSERT_EQUAL_INT(
				0, disc_tick_no_pps(&ctx, &in.env, NULL, &out));
			osc_step(&p, out.dac_code, 1.0);
		}
		TEST_ASSERT_EQUAL_INT(DISC_STATE_HOLDOVER, out.state);
		TEST_ASSERT_TRUE(d_abs(p.e_ns) > 15000.0);

		/* Pulses return. */
		prev_e = p.e_ns;
		for (i = 0u; i < 2000u; i++) {
			double rate;

			out = loop_second(&ctx, &p, &ms, 5.0, &expected);
			if (i == 0u) {
				TEST_ASSERT_EQUAL_INT(DISC_STATE_RECOVERING,
						      out.state);
			}
			rate = d_abs(p.e_ns - prev_e);
			prev_e = p.e_ns;
			if (i > 20u && rate > worst) {
				worst = rate;
			}
		}
		max_rate[pass] = worst;
		final_e[pass] = d_abs(p.e_ns);
	}

	/* Both recover... */
	TEST_ASSERT_TRUE(final_e[0] < 500.0);
	TEST_ASSERT_TRUE(final_e[1] < 500.0);
	/* ...but only the limited run keeps the correction inside the budget,
	 * and the unlimited one is visibly faster at the client's expense. */
	TEST_ASSERT_TRUE(max_rate[0] <= 55.0);
	TEST_ASSERT_TRUE(max_rate[1] > 70.0);
}

/* --------------------------------------------------------------- Vc sense */

static void test_vc_sense_divergence_raises_a_dac_fault(void)
{
	disc_ctx_t ctx;
	disc_in_t in;
	disc_out_t out;
	uint64_t ms = 0u;
	uint32_t expected = 0x90000000u;
	unsigned int i;

	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));

	/* The op-amp buffer output on PA3 should track the commanded Vc. A
	 * persistent disagreement means the loop filter or the DAC has failed
	 * and the oscillator is no longer being steered. */
	for (i = 0u; i < 2u; i++) {
		ms += 1000u;
		env_defaults(&in.env, ms);
		in.env.vc_sense_valid = true;
		in.env.vc_sense_mv = 100; /* commanded is ~1650 */
		pps_from_error(&in.pps, 0.0, 1.0, expected);
		expected += 1000000u;
		TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
		TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_DAC_FAULT) == 0u);
	}

	ms += 1000u;
	env_defaults(&in.env, ms);
	in.env.vc_sense_valid = true;
	in.env.vc_sense_mv = 100;
	pps_from_error(&in.pps, 0.0, 1.0, expected);
	expected += 1000000u;
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_DAC_FAULT) != 0u);

	/* Losing the ADC reading must not clear a latched fault: absence of
	 * evidence is not evidence of health. */
	ms += 1000u;
	env_defaults(&in.env, ms);
	pps_from_error(&in.pps, 0.0, 1.0, expected);
	expected += 1000000u;
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_DAC_FAULT) != 0u);

	/* A healthy reading does clear it. */
	ms += 1000u;
	env_defaults(&in.env, ms);
	in.env.vc_sense_valid = true;
	in.env.vc_sense_mv = ctx.vc_cmd_mv;
	pps_from_error(&in.pps, 0.0, 1.0, expected);
	expected += 1000000u;
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_DAC_FAULT) == 0u);

	/* The comparison is symmetric: a sense stuck high is as much of a
	 * fault as one stuck low. */
	for (i = 0u; i < 3u; i++) {
		ms += 1000u;
		env_defaults(&in.env, ms);
		in.env.vc_sense_valid = true;
		in.env.vc_sense_mv = ctx.vc_cmd_mv + 900;
		pps_from_error(&in.pps, 0.0, 1.0, expected);
		expected += 1000000u;
		TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	}
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_DAC_FAULT) != 0u);
}

/* ---------------------------------------------------------------- PFI park */

static void test_park_holds_the_actuator_and_unpark_recovers(void)
{
	disc_ctx_t ctx;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0xA0000000u;
	uint16_t code = 0u;
	disc_out_t out;
	disc_in_t in;
	unsigned int i;

	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	noise_seed(59u);
	osc_init(&p, 0.0, -30.0);
	(void)run_to_lock(&ctx, &p, &ms, 5.0, &expected, 1500u);

	/* §10.3: the PFI handler persists this code and the DAC keeps driving
	 * it — PA4 must never be tri-stated (interface ref §10 caution 12). */
	TEST_ASSERT_EQUAL_INT(0, disc_park(&ctx, &code));
	TEST_ASSERT_EQUAL_UINT16(ctx.dac_code, code);
	TEST_ASSERT_EQUAL_INT(DISC_STATE_PARKED, disc_state(&ctx));
	/* Idempotent, and a caller that does not want the code may pass NULL. */
	TEST_ASSERT_EQUAL_INT(0, disc_park(&ctx, NULL));

	for (i = 0u; i < 10u; i++) {
		ms += 1000u;
		env_defaults(&in.env, ms);
		pps_from_error(&in.pps, 12345.0, 1.0, expected);
		expected += 1000000u;
		TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
		TEST_ASSERT_EQUAL_INT(DISC_STATE_PARKED, out.state);
		TEST_ASSERT_EQUAL_UINT16(code, out.dac_code);
		TEST_ASSERT_FALSE(out.sample_accepted);
		TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_PARKED) != 0u);
		TEST_ASSERT_EQUAL_UINT8(QUALITY_STRATUM_UNSYNC, out.stratum);

		ms += 1000u;
		env_defaults(&in.env, ms);
		TEST_ASSERT_EQUAL_INT(0,
				      disc_tick_no_pps(&ctx, &in.env, NULL, &out));
		TEST_ASSERT_EQUAL_INT(DISC_STATE_PARKED, out.state);
		TEST_ASSERT_EQUAL_UINT16(code, out.dac_code);
	}

	/* The rail came back without a reset. The frequency estimate survived,
	 * so this is a §3.6 recovery, not a cold start. */
	TEST_ASSERT_EQUAL_INT(0, disc_unpark(&ctx));
	TEST_ASSERT_EQUAL_INT(DISC_STATE_RECOVERING, disc_state(&ctx));
	TEST_ASSERT_EQUAL_INT(0, disc_unpark(&ctx)); /* no-op when not parked */
	TEST_ASSERT_EQUAL_INT(DISC_STATE_RECOVERING, disc_state(&ctx));

	for (i = 0u; i < 400u; i++) {
		out = loop_second(&ctx, &p, &ms, 5.0, &expected);
	}
	TEST_ASSERT_EQUAL_INT(DISC_STATE_LOCKED, out.state);
}

static void test_unpark_from_a_never_locked_loop_does_not_serve(void)
{
	disc_ctx_t ctx;
	disc_in_t in;
	disc_out_t out;

	/* Parking during acquisition and unparking lands in RECOVERING with no
	 * disciplining history behind it. That must not advertise stratum-1. */
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	TEST_ASSERT_EQUAL_INT(0, disc_park(&ctx, NULL));
	TEST_ASSERT_EQUAL_INT(0, disc_unpark(&ctx));

	env_defaults(&in.env, 1000u);
	pps_from_error(&in.pps, 10.0, 1.0, 1000u);
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_EQUAL_INT(DISC_STATE_RECOVERING, out.state);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_STRATUM_UNSYNC, out.stratum);
}

/* ------------------------------------------------------------ tempco */

static void test_tempco_feedforward_reduces_error_under_a_ramp(void)
{
	unsigned int pass;
	double worst[2];

	/*
	 * Differential: identical temperature ramp, once with the learned
	 * coefficient applied and once without. §3.3 claims the feed-forward
	 * improves post-transient settling; this is that claim, measured.
	 */
	for (pass = 0u; pass < 2u; pass++) {
		disc_ctx_t ctx;
		disc_cfg_t cfg;
		struct osc p;
		uint64_t ms = 0u;
		uint32_t expected = 0xB0000000u;
		unsigned int i;
		disc_out_t out;
		double w = 0.0;

		(void)disc_cfg_defaults(&cfg);
		cfg.tempco_enable = (pass == 0u);
		cfg.tempco_ppb_per_c = 3.0f;
		TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));
		noise_seed(61u);
		osc_init(&p, 0.0, -20.0);
		p.tempco_ppb_per_c = 3.0; /* the plant the coefficient describes */
		p.temp_ref_c = 45.0;

		(void)run_to_lock(&ctx, &p, &ms, 3.0, &expected, 1500u);
		TEST_ASSERT_TRUE(ctx.tref_valid);
		TEST_ASSERT_EQUAL_INT32(45000, ctx.tref_mc);

		/* Ramp 45 -> 60 C over 300 s, then hold. */
		for (i = 0u; i < 600u; i++) {
			if (p.temp_c < 60.0) {
				p.temp_c += 0.05;
			}
			out = loop_second(&ctx, &p, &ms, 3.0, &expected);
			if (i > 30u && d_abs(p.e_ns) > w) {
				w = d_abs(p.e_ns);
			}
		}
		worst[pass] = w;
		if (pass == 0u) {
			TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_TEMPCO) != 0u);
		} else {
			TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_TEMPCO) == 0u);
		}
	}

	/* The feed-forward removes the disturbance before the integrator has
	 * to chase it, so the excursion is far smaller. */
	TEST_ASSERT_TRUE(worst[0] * 3.0 < worst[1]);
}

static void test_tempco_fit_recovers_a_known_line(void)
{
	float temp[41];
	float ppb[41];
	unsigned int i;
	float slope = 0.0f;
	float offset = 0.0f;
	float r2 = 0.0f;

	/* osc_ppb = -12.5 + 2.75 * T, sampled over 20..60 C. */
	for (i = 0u; i < 41u; i++) {
		temp[i] = 20.0f + (float)i;
		ppb[i] = -12.5f + 2.75f * temp[i];
	}

	TEST_ASSERT_EQUAL_INT(0, disc_tempco_fit(temp, ppb, 41u, &slope, &offset,
						 &r2));
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.75f, slope);
	TEST_ASSERT_FLOAT_WITHIN(1e-2f, -12.5f, offset);
	TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, r2);
}

static void test_tempco_fit_two_points_and_noise(void)
{
	float temp2[2] = { 10.0f, 20.0f };
	float ppb2[2] = { 5.0f, 25.0f };
	float temp[9];
	float ppb[9];
	unsigned int i;
	float slope = 0.0f;
	float offset = 0.0f;
	float r2 = 0.0f;

	/* Two points define the line exactly: slope (25-5)/(20-10) = 2,
	 * intercept 5 - 2*10 = -15. */
	TEST_ASSERT_EQUAL_INT(0,
			      disc_tempco_fit(temp2, ppb2, 2u, &slope, &offset,
					      &r2));
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f, slope);
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, -15.0f, offset);

	/* Scattered data still recovers the trend, with r2 below one. */
	for (i = 0u; i < 9u; i++) {
		temp[i] = 30.0f + (float)i;
		ppb[i] = 1.5f * temp[i] + (((i % 2u) == 0u) ? 4.0f : -4.0f);
	}
	TEST_ASSERT_EQUAL_INT(0, disc_tempco_fit(temp, ppb, 9u, &slope, &offset,
						 &r2));
	TEST_ASSERT_FLOAT_WITHIN(0.5f, 1.5f, slope);
	TEST_ASSERT_TRUE(r2 > 0.0f);
	TEST_ASSERT_TRUE(r2 < 1.0f);

	/* Optional outputs really are optional. */
	TEST_ASSERT_EQUAL_INT(0,
			      disc_tempco_fit(temp, ppb, 9u, &slope, NULL, NULL));
}

static void test_tempco_fit_rejects_undefined_problems(void)
{
	float temp[3] = { 40.0f, 40.0f, 40.0f };
	float ppb[3] = { 1.0f, 2.0f, 3.0f };
	float flat_ppb[3] = { 7.0f, 7.0f, 7.0f };
	float t2[3] = { 1.0f, 2.0f, 3.0f };
	float slope = 0.0f;
	float r2 = 0.0f;

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      disc_tempco_fit(NULL, ppb, 3u, &slope, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      disc_tempco_fit(temp, NULL, 3u, &slope, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      disc_tempco_fit(temp, ppb, 3u, NULL, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      disc_tempco_fit(temp, ppb, 1u, &slope, NULL, NULL));
	/* No temperature variation: the slope is not identifiable, and
	 * returning a number would be worse than refusing. */
	TEST_ASSERT_EQUAL_INT(-EDOM,
			      disc_tempco_fit(temp, ppb, 3u, &slope, NULL, NULL));
	/* A perfectly flat response is identifiable (slope 0) and r2 is
	 * defined as 1 rather than 0/0. */
	TEST_ASSERT_EQUAL_INT(0, disc_tempco_fit(t2, flat_ppb, 3u, &slope, NULL,
						 &r2));
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, slope);
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, r2);
}

/* ------------------------------------------------------------------- ADEV */

static void test_adev_matches_white_fm_theory(void)
{
	/*
	 * White FM: independent frequency samples, so the phase is their
	 * running sum. The analytic law is sigma_y(tau) = sigma_y(1)/sqrt(m),
	 * which is a property of the noise, not of this estimator. With
	 * sigma_y(1) = 1 ppb the three answers are 1e-9, 3.16e-10 and 1e-10.
	 */
	static float phase[4000];
	unsigned int i;
	double x = 0.0;
	float a1 = 0.0f;
	float a10 = 0.0f;
	float a100 = 0.0f;

	noise_seed(101u);
	for (i = 0u; i < 4000u; i++) {
		phase[i] = (float)x;
		x += noise(1.0); /* 1 ppb over 1 s == 1 ns of phase */
	}

	TEST_ASSERT_EQUAL_INT(0,
			      disc_adev_overlapping(phase, 4000u, 1u, 1.0f, &a1));
	TEST_ASSERT_EQUAL_INT(0, disc_adev_overlapping(phase, 4000u, 10u, 1.0f,
						       &a10));
	TEST_ASSERT_EQUAL_INT(0, disc_adev_overlapping(phase, 4000u, 100u, 1.0f,
						       &a100));

	/* Within a factor of two of theory at every tau. */
	TEST_ASSERT_TRUE(a1 > 0.5e-9f && a1 < 2.0e-9f);
	TEST_ASSERT_TRUE(a10 > 0.158e-9f && a10 < 0.632e-9f);
	TEST_ASSERT_TRUE(a100 > 0.05e-9f && a100 < 0.2e-9f);

	/* And the tau^-1/2 slope itself, which is the signature of white FM
	 * and the thing a mis-normalised estimator gets wrong. */
	TEST_ASSERT_TRUE((double)a1 / (double)a10 > 2.2);
	TEST_ASSERT_TRUE((double)a1 / (double)a10 < 4.5);
}

static void test_adev_of_a_pure_frequency_offset_is_zero(void)
{
	static float phase[500];
	unsigned int i;
	float a = 1.0f;

	/* The second difference removes any constant rate, so a perfectly
	 * linear phase ramp has no Allan deviation at all. This is what makes
	 * the estimator measure the residual of a steered clock rather than
	 * its steering. */
	for (i = 0u; i < 500u; i++) {
		phase[i] = 12345.0f + 7.5f * (float)i;
	}
	TEST_ASSERT_EQUAL_INT(0, disc_adev_overlapping(phase, 500u, 1u, 1.0f, &a));
	TEST_ASSERT_TRUE(a < 1.0e-12f);
}

static void test_adev_scales_with_tau0(void)
{
	static float phase[400];
	unsigned int i;
	float a1 = 0.0f;
	float a2 = 0.0f;

	/* sigma_y is dimensionless: doubling the declared sample interval with
	 * the same phase record halves the deviation at the same m. */
	noise_seed(103u);
	for (i = 0u; i < 400u; i++) {
		phase[i] = (float)noise(10.0);
	}
	TEST_ASSERT_EQUAL_INT(0, disc_adev_overlapping(phase, 400u, 2u, 1.0f, &a1));
	TEST_ASSERT_EQUAL_INT(0, disc_adev_overlapping(phase, 400u, 2u, 2.0f, &a2));
	TEST_ASSERT_FLOAT_WITHIN(a1 * 0.01f, a1 * 0.5f, a2);
}

static void test_adev_argument_errors(void)
{
	float phase[10] = { 0 };
	float out = 0.0f;

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      disc_adev_overlapping(NULL, 10u, 1u, 1.0f, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      disc_adev_overlapping(phase, 10u, 1u, 1.0f, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      disc_adev_overlapping(phase, 10u, 0u, 1.0f, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      disc_adev_overlapping(phase, 10u, 1u, 0.0f, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      disc_adev_overlapping(phase, 10u, 1u, NAN, &out));
	/* 2m+1 samples are the minimum for even one second difference. */
	TEST_ASSERT_EQUAL_INT(-ENODATA,
			      disc_adev_overlapping(phase, 2u, 1u, 1.0f, &out));
	TEST_ASSERT_EQUAL_INT(0,
			      disc_adev_overlapping(phase, 3u, 1u, 1.0f, &out));
	TEST_ASSERT_EQUAL_INT(-ENODATA,
			      disc_adev_overlapping(phase, 10u, 5u, 1.0f, &out));
}

static void test_loop_publishes_adev_once_the_ring_has_filled(void)
{
	disc_ctx_t ctx;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0xC0000000u;
	unsigned int i;
	disc_out_t out;

	noise_seed(107u);
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	osc_init(&p, 0.0, -30.0);
	(void)run_to_lock(&ctx, &p, &ms, 25.0, &expected, 1500u);
	for (i = 0u; i < 700u; i++) {
		out = loop_second(&ctx, &p, &ms, 25.0, &expected);
	}

	/* tau = 100 s needs 201 samples; the ring is 512 deep, so all three
	 * are available and none is the "not yet" sentinel. */
	TEST_ASSERT_TRUE(out.adev_1s > 0.0f);
	TEST_ASSERT_TRUE(out.adev_10s > 0.0f);
	TEST_ASSERT_TRUE(out.adev_100s > 0.0f);
	/* 25 ns of white phase noise gives sigma_y(1) = sqrt(3)*25e-9/1. */
	TEST_ASSERT_TRUE(out.adev_1s > 1.0e-8f);
	TEST_ASSERT_TRUE(out.adev_1s < 1.0e-7f);
	/* White PM falls as tau^-1, so tau=100 is two decades down. */
	TEST_ASSERT_TRUE(out.adev_100s < out.adev_10s);
	TEST_ASSERT_TRUE(out.adev_10s < out.adev_1s);
}

/* ------------------------------------------------------------- publication */

static void test_publishes_the_quality_block(void)
{
	disc_ctx_t ctx;
	quality_state_t qs;
	quality_block_t b;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0xD0000000u;
	disc_in_t in;
	disc_out_t out;
	unsigned int i;

	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	TEST_ASSERT_EQUAL_INT(0, quality_state_init(&qs));
	noise_seed(109u);
	osc_init(&p, 0.0, -30.0);

	for (i = 0u; i < 1500u; i++) {
		ms += 1000u;
		env_defaults(&in.env, ms);
		in.env.anc.refid = 0u; /* ask for the default */
		pps_from_error(&in.pps, p.e_ns + noise(5.0), 1.0, expected);
		expected += 1000000u;
		TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, &qs, &out));
		osc_step(&p, out.dac_code, 1.0);
		if (out.state == DISC_STATE_LOCKED) {
			break;
		}
	}
	TEST_ASSERT_EQUAL_INT(DISC_STATE_LOCKED, out.state);

	TEST_ASSERT_EQUAL_INT(0, quality_snapshot(&qs, &b));
	TEST_ASSERT_EQUAL_UINT16(QUALITY_BLOCK_VER, b.ver);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_STRATUM_PRIMARY, b.stratum);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_LOCK_LOCKED, b.lock_state);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_REF_OCXO, b.active_ref);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_GNSS_TIME_ONLY, b.gnss_fix);
	TEST_ASSERT_EQUAL_UINT8(14u, b.gnss_sv_used);
	TEST_ASSERT_EQUAL_UINT8(20u, b.gnss_sv_visible);
	TEST_ASSERT_EQUAL_UINT32(12u, b.gnss_tacc_ns);
	TEST_ASSERT_TRUE(b.utc_valid);
	TEST_ASSERT_EQUAL_INT16(37, b.leap_current_s);
	TEST_ASSERT_FALSE(b.holdover);
	TEST_ASSERT_EQUAL_UINT64(ms, b.updated_mono_ms);
	TEST_ASSERT_EQUAL_UINT16(ctx.dac_code, b.dac_code);
	TEST_ASSERT_EQUAL_INT32(ctx.vc_cmd_mv, b.vc_cmd_mv);
	TEST_ASSERT_EQUAL_INT32(45000, b.osc_temp_mc);
	/* RFC 5905 §7.3: a stratum-1 GNSS server advertises "GPS". */
	TEST_ASSERT_EQUAL_HEX32(quality_refid('G', 'P', 'S', '\0'), b.refid);
	/* Root delay is zero for a primary; dispersion is the floor plus the
	 * measured scatter, and must be a sane sub-millisecond number. */
	TEST_ASSERT_EQUAL_UINT32(0u, b.root_delay_q16);
	TEST_ASSERT_TRUE(b.root_disp_q16 > 0u);
	TEST_ASSERT_TRUE(quality_ns_from_ntp_short(b.root_disp_q16) < 1000000);
	TEST_ASSERT_TRUE((b.flags & QUALITY_FLAG_GNSS_TIME_LOCKED) != 0u);

	/* An explicit reference identifier overrides the default. */
	ms += 1000u;
	env_defaults(&in.env, ms);
	in.env.anc.refid = quality_refid('G', 'N', 'S', 'S');
	in.env.anc.root_delay_ns = 1000000; /* 1 ms */
	pps_from_error(&in.pps, p.e_ns, 1.0, expected);
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, &qs, &out));
	TEST_ASSERT_EQUAL_INT(0, quality_snapshot(&qs, &b));
	TEST_ASSERT_EQUAL_HEX32(quality_refid('G', 'N', 'S', 'S'), b.refid);
	TEST_ASSERT_EQUAL_UINT32(quality_ntp_short_from_ns(1000000),
				 b.root_delay_q16);

	/* Holdover publishes its own state through the same block. */
	for (i = 0u; i < 100u; i++) {
		ms += 1000u;
		env_defaults(&in.env, ms);
		TEST_ASSERT_EQUAL_INT(0,
				      disc_tick_no_pps(&ctx, &in.env, &qs, &out));
	}
	TEST_ASSERT_EQUAL_INT(0, quality_snapshot(&qs, &b));
	TEST_ASSERT_EQUAL_UINT8(QUALITY_LOCK_HOLDOVER, b.lock_state);
	TEST_ASSERT_TRUE(b.holdover);
	TEST_ASSERT_TRUE(b.holdover_elapsed_s > 90u);
	TEST_ASSERT_TRUE(b.holdover_est_err_ns > 0);
	TEST_ASSERT_TRUE(b.root_disp_q16 > 0u);

	/* And so does the park. */
	TEST_ASSERT_EQUAL_INT(0, disc_park(&ctx, NULL));
	ms += 1000u;
	env_defaults(&in.env, ms);
	TEST_ASSERT_EQUAL_INT(0, disc_tick_no_pps(&ctx, &in.env, &qs, &out));
	TEST_ASSERT_EQUAL_INT(0, quality_snapshot(&qs, &b));
	TEST_ASSERT_EQUAL_UINT8(QUALITY_LOCK_PARKED, b.lock_state);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_STRATUM_UNSYNC, b.stratum);
}

/* --------------------------------------------------------- odd cadences */

static void test_irregular_and_backwards_timestamps_are_survivable(void)
{
	disc_ctx_t ctx;
	disc_in_t in;
	disc_out_t out;
	uint32_t expected = 0xE0000000u;
	uint16_t code;

	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));

	env_defaults(&in.env, 10000u);
	pps_from_error(&in.pps, 100.0, 1.0, expected);
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	code = out.dac_code;

	/* A duplicated timestamp: no time has passed, so no frequency can be
	 * inferred. The loop must not divide by it. */
	env_defaults(&in.env, 10000u);
	pps_from_error(&in.pps, 100.0, 1.0, expected + 1000000u);
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_TRUE(isfinite(out.applied_ppb));

	/* A very long gap: the interval is out of range, so the frequency
	 * update is skipped and the nominal second is used for the integral. */
	env_defaults(&in.env, 400000u);
	pps_from_error(&in.pps, 100.0, 1.0, expected + 2000000u);
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_TRUE(isfinite(out.applied_ppb));

	/* And a timestamp older than the last one. */
	env_defaults(&in.env, 5000u);
	pps_from_error(&in.pps, 100.0, 1.0, expected + 3000000u);
	TEST_ASSERT_EQUAL_INT(0, disc_tick_pps(&ctx, &in, NULL, &out));
	TEST_ASSERT_TRUE(isfinite(out.applied_ppb));
	TEST_ASSERT_TRUE(out.dac_code <= 4095u);
	TEST_ASSERT_TRUE(out.dac_code >= code - 300u);
}

static void test_slew_limiter_is_reported(void)
{
	disc_ctx_t ctx;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0xE1000000u;
	unsigned int i;
	disc_out_t out;
	bool saw_slew = false;

	/* Once locked, a sustained pull demands more movement per second than
	 * the 5 LSB/s limit allows, and the limiter must say so. */
	noise_seed(113u);
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, NULL));
	osc_init(&p, 0.0, -30.0);
	(void)run_to_lock(&ctx, &p, &ms, 3.0, &expected, 1500u);

	p.intrinsic_ppb = -200.0; /* a 170 ppb step while serving */
	for (i = 0u; i < 60u; i++) {
		out = loop_second(&ctx, &p, &ms, 3.0, &expected);
		if ((out.flags & QUALITY_FLAG_DAC_SLEW) != 0u) {
			saw_slew = true;
		}
	}
	TEST_ASSERT_TRUE(saw_slew);
}

static void test_lock_is_lost_before_holdover_when_pulses_are_merely_late(void)
{
	disc_ctx_t ctx;
	disc_cfg_t cfg;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0xE3000000u;
	unsigned int i;
	disc_out_t out;
	disc_in_t in;

	/*
	 * With a holdover threshold deliberately longer than the unlock dwell,
	 * a run of missing pulses must first cost the lock and only later
	 * become holdover. Serving stratum-1 off a lock that has not been
	 * confirmed for a dozen seconds would be a lie.
	 */
	(void)disc_cfg_defaults(&cfg);
	cfg.pps_loss_ticks = 40u;
	cfg.unlock_dwell_s = 5u;
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));
	noise_seed(131u);
	osc_init(&p, 0.0, -30.0);
	(void)run_to_lock(&ctx, &p, &ms, 3.0, &expected, 1500u);

	for (i = 0u; i < 10u; i++) {
		ms += 1000u;
		env_defaults(&in.env, ms);
		TEST_ASSERT_EQUAL_INT(0,
				      disc_tick_no_pps(&ctx, &in.env, NULL, &out));
	}
	TEST_ASSERT_EQUAL_INT(DISC_STATE_LOCKING, out.state);
	TEST_ASSERT_FALSE(out.holdover);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_STRATUM_UNSYNC, out.stratum);

	for (i = 0u; i < 40u; i++) {
		ms += 1000u;
		env_defaults(&in.env, ms);
		TEST_ASSERT_EQUAL_INT(0,
				      disc_tick_no_pps(&ctx, &in.env, NULL, &out));
	}
	TEST_ASSERT_EQUAL_INT(DISC_STATE_HOLDOVER, out.state);
}

static void test_negative_rail_saturates_and_is_reported(void)
{
	disc_ctx_t ctx;
	disc_cfg_t cfg;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0xE4000000u;
	unsigned int i;
	disc_out_t out;

	/* The mirror image of the positive-rail case: an oscillator running
	 * 900 ppb fast drives the actuator to code 0, and the slew band has to
	 * clamp at the bottom of the range as well as the top. */
	(void)disc_cfg_defaults(&cfg);
	cfg.tau_s = 30.0f;
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));
	noise_seed(137u);
	osc_init(&p, 0.0, 900.0);

	for (i = 0u; i < 300u; i++) {
		out = loop_second(&ctx, &p, &ms, 2.0, &expected);
	}
	TEST_ASSERT_EQUAL_UINT16(0u, out.dac_code);
	TEST_ASSERT_TRUE((out.flags & QUALITY_FLAG_DAC_SAT) != 0u);
	TEST_ASSERT_EQUAL_INT32(0, out.vc_cmd_mv);
	TEST_ASSERT_FLOAT_WITHIN(1.0f, -400.0f, out.applied_ppb);
}

static void test_lock_is_lost_after_sustained_bad_seconds(void)
{
	disc_ctx_t ctx;
	disc_cfg_t cfg;
	struct osc p;
	uint64_t ms = 0u;
	uint32_t expected = 0xE2000000u;
	unsigned int i;
	disc_out_t out;

	/* pps_loss_ticks raised out of the way so this exercises the
	 * "criteria no longer met" demotion rather than the holdover path. */
	(void)disc_cfg_defaults(&cfg);
	cfg.pps_loss_ticks = 200u;
	TEST_ASSERT_EQUAL_INT(0, disc_init(&ctx, &cfg));
	noise_seed(127u);
	osc_init(&p, 0.0, -30.0);
	(void)run_to_lock(&ctx, &p, &ms, 3.0, &expected, 1500u);

	/* Enough noise that |e| and the rolling variance both fail. */
	for (i = 0u; i < 40u; i++) {
		out = loop_second(&ctx, &p, &ms, 900.0, &expected);
	}
	TEST_ASSERT_EQUAL_INT(DISC_STATE_LOCKING, out.state);
	TEST_ASSERT_EQUAL_UINT8(QUALITY_STRATUM_UNSYNC, out.stratum);
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_defaults);
	RUN_TEST(test_init_centres_the_actuator);
	RUN_TEST(test_init_rejects_bad_config);
	RUN_TEST(test_api_rejects_null_and_uninitialised);

	RUN_TEST(test_sign_convention_late_clock_is_positive);
	RUN_TEST(test_sign_convention_early_clock_is_negative);
	RUN_TEST(test_capture_difference_survives_the_counter_wrap);
	RUN_TEST(test_sawtooth_and_delays_are_applied_with_the_right_sign);
	RUN_TEST(test_sawtooth_correction_shrinks_the_residual);

	RUN_TEST(test_dual_capture_is_averaged_when_the_channels_agree);
	RUN_TEST(test_channel_divergence_is_flagged_and_the_primary_wins);
	RUN_TEST(test_secondary_alone_is_used);
	RUN_TEST(test_no_usable_capture_is_a_missed_second);

	RUN_TEST(test_single_glitch_is_rejected_and_the_loop_is_undisturbed);
	RUN_TEST(test_a_real_step_is_eventually_accepted);

	RUN_TEST(test_acquisition_from_100us_converges_and_locks);
	RUN_TEST(test_lock_requires_warm_and_gnss);
	RUN_TEST(test_warm_timeout_backstops_dead_sensors);
	RUN_TEST(test_steady_state_noise_rejection_bounds_the_actuator);

	RUN_TEST(test_beyond_pull_range_saturates_cleanly);
	RUN_TEST(test_pi_integral_does_not_wind_up_against_the_rail);
	RUN_TEST(test_a_nonsense_tempco_constant_cannot_reach_the_dac);

	RUN_TEST(test_missed_pulses_enter_holdover_and_freeze_the_actuator);
	RUN_TEST(test_holdover_demotes_when_the_budget_is_spent);
	RUN_TEST(test_holdover_temperature_term_and_unreachable_budget);
	RUN_TEST(test_gnss_time_unlock_forces_holdover_even_with_pulses);
	RUN_TEST(test_recovery_is_rate_limited_and_never_steps);

	RUN_TEST(test_vc_sense_divergence_raises_a_dac_fault);

	RUN_TEST(test_park_holds_the_actuator_and_unpark_recovers);
	RUN_TEST(test_unpark_from_a_never_locked_loop_does_not_serve);

	RUN_TEST(test_tempco_feedforward_reduces_error_under_a_ramp);
	RUN_TEST(test_tempco_fit_recovers_a_known_line);
	RUN_TEST(test_tempco_fit_two_points_and_noise);
	RUN_TEST(test_tempco_fit_rejects_undefined_problems);

	RUN_TEST(test_adev_matches_white_fm_theory);
	RUN_TEST(test_adev_of_a_pure_frequency_offset_is_zero);
	RUN_TEST(test_adev_scales_with_tau0);
	RUN_TEST(test_adev_argument_errors);
	RUN_TEST(test_loop_publishes_adev_once_the_ring_has_filled);

	RUN_TEST(test_publishes_the_quality_block);

	RUN_TEST(test_irregular_and_backwards_timestamps_are_survivable);
	RUN_TEST(test_slew_limiter_is_reported);
	RUN_TEST(test_lock_is_lost_after_sustained_bad_seconds);

	return UNITY_END();
}
