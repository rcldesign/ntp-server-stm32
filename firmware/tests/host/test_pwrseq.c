/*
 * STS1000 "Meridian" — core/pwrseq unit tests.
 *
 * The sequencer's whole job is *ordering*, so these tests record every action
 * it emits into a log and assert against the sequence, not just the end state.
 * A bring-up that reaches the right final configuration by the wrong route is a
 * failure here — most sharply for the rubidium, where the interlocks only work
 * if they run in the documented order:
 *
 *     safe digipot code -> readback verify -> RB_PWR_EN -> soft-start ->
 *     INA228 0x47 window -> RB_VCC_GATE -> lock
 *
 * `test_rb_never_powers_before_the_digipot_is_verified` and
 * `test_rb_never_gates_before_the_rail_window_is_proven` are the two that
 * matter most: they assert the *absence* of an action, which is the only way to
 * catch an interlock that has quietly stopped interlocking.
 *
 * Numeric expectations for the VCC_RB transfer function come from the table in
 * `docs/sts1000_vcc_rb_supply.md` §5, and the watchdog window from
 * `docs/sts1000_external_wdt.md` §4.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "ina228/ina228.h"
#include "pwrseq/pwrseq.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))
#define LOG_MAX 256

/* ---------------------------------------------------------------- harness */

typedef struct {
	pwrseq_ctx_t ctx;
	pwrseq_in_t in;
	pwrseq_act_t log[LOG_MAX];
	size_t log_len;
} model_t;

/* A board where every reading is nominal and every flag is the happy one. */
static void in_healthy(pwrseq_in_t *in)
{
	int i;

	memset(in, 0, sizeof(*in));

	in->mono_ms = 0U;
	in->nor_ready = true;
	in->i2c_probe_ok = true;
	in->cal_loaded = true;
	in->shunt_trims_applied = true;

	for (i = 0; i < (int)INA228_RAIL_COUNT; i++) {
		in->ina_valid[i] = true;
		in->ina_vbus_mv[i] = ina228_rail_tbl[i].nominal_mv;
		in->ina_current_ma[i] = 10;
	}
	/*
	 * VCC_RB is the exception: its nominal is a documentation figure for a
	 * 15 V-class FE, but the rail comes up at whatever the commanded
	 * digipot code implies. With the safe-low code that is 4.515 V.
	 */
	in->ina_vbus_mv[INA228_RAIL_VCC_RB] = 4515;
	/* A warm oven settles well under the 350 mA warm threshold. */
	in->ina_current_ma[INA228_RAIL_OCXO] = 300;

	in->pg_mask = 0xFFU;
	in->phy_refclk_stable = true;
	in->phy_id_ok = true;
	in->gnss_cfg_ack = true;
	in->ocxo_warm = false; /* proven from current + temperature instead */
	in->ocxo_temp_stable = true;
	in->rb_wanted = true;
	in->supercaps_charged = true;
	in->digipot_readback = 0U;
	in->digipot_readback_valid = true;
	in->rb_lock = true;
	in->extref_in_band = true;
	in->rb_ov_det = false;
	in->poe_granted_mw = 51000U; /* Class 6 */
	in->poe_measured_mw = 14000U;
	in->ui_wanted = true;
	in->liveness_ok = true;
	in->debugger_attached = false;
}

static void model_init(model_t *m, const pwrseq_cfg_t *cfg)
{
	memset(m, 0, sizeof(*m));
	TEST_ASSERT_EQUAL_INT(0, pwrseq_init(&m->ctx, cfg));
	in_healthy(&m->in);
	TEST_ASSERT_EQUAL_INT(0, pwrseq_start(&m->ctx, 0U));
}

/* One step() call, draining every action into the log. */
static void pump(model_t *m)
{
	pwrseq_act_t a;

	TEST_ASSERT_EQUAL_INT(0, pwrseq_step(&m->ctx, &m->in));
	while (pwrseq_action_get(&m->ctx, &a) == 0) {
		TEST_ASSERT_TRUE_MESSAGE(m->log_len < LOG_MAX,
					 "action log overflowed");
		m->log[m->log_len++] = a;
	}
}

static void advance(model_t *m, uint32_t ms)
{
	m->in.mono_ms += ms;
	pump(m);
}

/* Pump until the sequence finishes or halts, or the budget runs out. */
static void run_out(model_t *m, uint32_t step_ms, unsigned int max_iters)
{
	unsigned int i;

	pump(m);
	for (i = 0U; i < max_iters; i++) {
		if ((pwrseq_stage(&m->ctx) == PWRSEQ_STAGE_DONE) ||
		    m->ctx.halted) {
			return;
		}
		advance(m, step_ms);
	}
}

static int idx_of_from(const model_t *m, pwrseq_action_t a, size_t start)
{
	size_t i;

	for (i = start; i < m->log_len; i++) {
		if (m->log[i].action == (uint16_t)a) {
			return (int)i;
		}
	}
	return -1;
}

static int idx_of(const model_t *m, pwrseq_action_t a)
{
	return idx_of_from(m, a, 0U);
}

static unsigned int count_of(const model_t *m, pwrseq_action_t a)
{
	unsigned int n = 0U;
	size_t i;

	for (i = 0U; i < m->log_len; i++) {
		if (m->log[i].action == (uint16_t)a) {
			n++;
		}
	}
	return n;
}

static void expect_absent(const model_t *m, pwrseq_action_t a)
{
	TEST_ASSERT_EQUAL_INT_MESSAGE(-1, idx_of(m, a), pwrseq_action_name(a));
}

static void expect_present(const model_t *m, pwrseq_action_t a)
{
	TEST_ASSERT_TRUE_MESSAGE(idx_of(m, a) >= 0, pwrseq_action_name(a));
}

/* Assert the log is exactly this sequence. */
static void expect_seq(const model_t *m, const uint8_t *want, size_t n)
{
	size_t i;

	for (i = 0U; (i < n) && (i < m->log_len); i++) {
		if (m->log[i].action != want[i]) {
			char msg[128];

			(void)snprintf(msg, sizeof(msg),
				       "action %u: want %s, got %s", (unsigned)i,
				       pwrseq_action_name(
					       (pwrseq_action_t)want[i]),
				       pwrseq_action_name(
					       (pwrseq_action_t)m->log[i].action));
			TEST_FAIL_MESSAGE(msg);
		}
	}
	TEST_ASSERT_EQUAL_size_t(n, m->log_len);
}

/* ------------------------------------------------------------------- init */

static void test_init_defaults_and_validation(void)
{
	pwrseq_cfg_t cfg;
	pwrseq_ctx_t ctx;

	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_init(NULL, NULL));

	TEST_ASSERT_EQUAL_INT(0, pwrseq_init(&ctx, NULL));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_IDLE, pwrseq_stage(&ctx));
	TEST_ASSERT_EQUAL_UINT16(0U, ctx.cfg.digipot_safe_code);
	TEST_ASSERT_EQUAL_UINT32(5U, ctx.cfg.rb_vbus_tol_pct);
	TEST_ASSERT_EQUAL_UINT32(16000U, ctx.cfg.rb_cold_start_mw);
	TEST_ASSERT_EQUAL_UINT32(500U, ctx.cfg.rb_softstart_ms);
	TEST_ASSERT_EQUAL_UINT32(600000U, ctx.cfg.rb_lock_timeout_ms);
	TEST_ASSERT_EQUAL_UINT32(350U, ctx.cfg.ocxo_warm_current_ma);
	TEST_ASSERT_EQUAL_UINT32(800U, ctx.cfg.ocxo_warmup_current_ma);
	TEST_ASSERT_EQUAL_INT32(42500, ctx.cfg.poe_min_mv);
	TEST_ASSERT_EQUAL_UINT32(PWRSEQ_WDT_KICK_PERIOD_MS,
				 ctx.cfg.wdt_kick_period_ms);

	/* The as-built VCC_RB transfer function. */
	TEST_ASSERT_EQUAL_UINT16(3000U, ctx.cfg.rb_xfer.vref_mv);
	TEST_ASSERT_EQUAL_UINT16(1024U, ctx.cfg.rb_xfer.steps);
	TEST_ASSERT_EQUAL_UINT32(24450U, ctx.cfg.rb_xfer.pedestal_mv);
	TEST_ASSERT_EQUAL_UINT32(6645U, ctx.cfg.rb_xfer.gain_m1000);

	/* A degenerate transfer function would divide by zero downstream. */
	pwrseq_cfg_default(&cfg);
	cfg.rb_xfer.steps = 0U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_init(&ctx, &cfg));
	pwrseq_cfg_default(&cfg);
	cfg.rb_xfer.vref_mv = 0U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_init(&ctx, &cfg));

	pwrseq_cfg_default(NULL); /* must not fault */
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_IDLE, pwrseq_stage(NULL));
}

static void test_wdt_cadence_outside_the_window_is_rejected(void)
{
	pwrseq_cfg_t cfg;
	pwrseq_ctx_t ctx;

	/*
	 * external_wdt §4: the valid window is (tWDL max, tWDU min) =
	 * 920-1360 ms. A windowed watchdog cold-cycles the board for a kick
	 * that is too *fast* just as readily as one that is too slow, so a
	 * misconfigured cadence has to be caught here and not in the field.
	 */
	TEST_ASSERT_EQUAL_UINT32(920U, PWRSEQ_WDT_WINDOW_MIN_MS);
	TEST_ASSERT_EQUAL_UINT32(1360U, PWRSEQ_WDT_WINDOW_MAX_MS);
	TEST_ASSERT_EQUAL_UINT32(920U, PWRSEQ_WDT_TWDL_MAX_MS);
	TEST_ASSERT_EQUAL_UINT32(1360U, PWRSEQ_WDT_TWDU_MIN_MS);
	TEST_ASSERT_EQUAL_UINT32(1840U, PWRSEQ_WDT_TWDU_MAX_MS);
	TEST_ASSERT_EQUAL_UINT32(680U, PWRSEQ_WDT_TWDL_MIN_MS);
	TEST_ASSERT_EQUAL_UINT32(200U, PWRSEQ_WDT_TRST_TYP_MS);

	/* The documented 1.10 s cadence sits inside the window, biased early. */
	TEST_ASSERT_EQUAL_UINT32(1100U, PWRSEQ_WDT_KICK_PERIOD_MS);
	TEST_ASSERT_TRUE(PWRSEQ_WDT_KICK_PERIOD_MS > PWRSEQ_WDT_WINDOW_MIN_MS);
	TEST_ASSERT_TRUE(PWRSEQ_WDT_KICK_PERIOD_MS < PWRSEQ_WDT_WINDOW_MAX_MS);
	TEST_ASSERT_EQUAL_UINT32(180U, PWRSEQ_WDT_KICK_PERIOD_MS -
					       PWRSEQ_WDT_WINDOW_MIN_MS);
	TEST_ASSERT_EQUAL_UINT32(260U, PWRSEQ_WDT_WINDOW_MAX_MS -
					       PWRSEQ_WDT_KICK_PERIOD_MS);

	pwrseq_cfg_default(&cfg);
	cfg.wdt_kick_period_ms = PWRSEQ_WDT_WINDOW_MIN_MS - 1U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_init(&ctx, &cfg));

	cfg.wdt_kick_period_ms = PWRSEQ_WDT_WINDOW_MAX_MS + 1U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_init(&ctx, &cfg));

	cfg.wdt_kick_period_ms = PWRSEQ_WDT_WINDOW_MIN_MS;
	TEST_ASSERT_EQUAL_INT(0, pwrseq_init(&ctx, &cfg));
	cfg.wdt_kick_period_ms = PWRSEQ_WDT_WINDOW_MAX_MS;
	TEST_ASSERT_EQUAL_INT(0, pwrseq_init(&ctx, &cfg));
}

/* ------------------------------------------------- VCC_RB transfer function */

static void test_digipot_transfer_matches_the_documented_table(void)
{
	pwrseq_rb_xfer_t x = {
		.vref_mv = 3000U,
		.steps = 1024U,
		.pedestal_mv = 24450U,
		.gain_m1000 = 6645U,
	};

	/*
	 * vcc_rb_supply §5. Note the sense: code 0 parks the wiper at terminal
	 * B = VREF, which is *maximum* VCTRL and therefore *minimum* VOUT. Get
	 * this backwards and the safe-low power-up code becomes 24 V into a
	 * 15 V-class FE-5680A.
	 */
	TEST_ASSERT_EQUAL_UINT32(3000U, pwrseq_digipot_vctrl_mv(&x, 0U));
	TEST_ASSERT_EQUAL_INT32(4515, pwrseq_rb_expected_mv(&x, 0U)); /* 4.51 V */

	TEST_ASSERT_EQUAL_UINT32(2927U, pwrseq_digipot_vctrl_mv(&x, 25U));
	TEST_ASSERT_EQUAL_INT32(5000, pwrseq_rb_expected_mv(&x, 25U)); /* 5 V */

	TEST_ASSERT_EQUAL_UINT32(1421U, pwrseq_digipot_vctrl_mv(&x, 539U));
	TEST_ASSERT_EQUAL_INT32(15007, pwrseq_rb_expected_mv(&x, 539U)); /* 15 V */

	TEST_ASSERT_EQUAL_UINT32(67U, pwrseq_digipot_vctrl_mv(&x, 1001U));
	TEST_ASSERT_EQUAL_INT32(24005, pwrseq_rb_expected_mv(&x, 1001U)); /* 24 V */

	TEST_ASSERT_EQUAL_UINT32(3U, pwrseq_digipot_vctrl_mv(&x, 1023U));
	TEST_ASSERT_EQUAL_INT32(24430, pwrseq_rb_expected_mv(&x, 1023U));

	/* Monotonic and bounded by the fixed-resistor envelope: no wiper code
	 * may exit 4.51-24.45 V (vcc_rb_supply §6b). */
	{
		uint16_t c;
		int32_t prev = pwrseq_rb_expected_mv(&x, 0U);

		for (c = 1U; c < 1024U; c++) {
			int32_t v = pwrseq_rb_expected_mv(&x, c);

			TEST_ASSERT_TRUE(v >= prev);
			TEST_ASSERT_TRUE(v >= 4515);
			TEST_ASSERT_TRUE(v <= 24450);
			prev = v;
		}
	}

	/* Out-of-range codes saturate at the top code rather than producing a
	 * negative VCTRL, which would read back as an impossible rail. */
	TEST_ASSERT_EQUAL_UINT32(pwrseq_digipot_vctrl_mv(&x, 1023U),
				 pwrseq_digipot_vctrl_mv(&x, 5000U));

	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_digipot_vctrl_mv(NULL, 0U));
	TEST_ASSERT_EQUAL_INT32(0, pwrseq_rb_expected_mv(NULL, 0U));
	x.steps = 0U;
	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_digipot_vctrl_mv(&x, 0U));
}

static void test_rb_acceptance_window(void)
{
	pwrseq_rb_xfer_t x = {
		.vref_mv = 3000U,
		.steps = 1024U,
		.pedestal_mv = 24450U,
		.gain_m1000 = 6645U,
	};
	int32_t lo = 0;
	int32_t hi = 0;

	TEST_ASSERT_EQUAL_INT(0, pwrseq_rb_window(&x, 0U, 5U, &lo, &hi));
	TEST_ASSERT_EQUAL_INT32(4515 - 226, lo);
	TEST_ASSERT_EQUAL_INT32(4515 + 226, hi);

	TEST_ASSERT_EQUAL_INT(0, pwrseq_rb_window(&x, 539U, 5U, &lo, &hi));
	TEST_ASSERT_EQUAL_INT32(15007 - 750, lo);
	TEST_ASSERT_EQUAL_INT32(15007 + 750, hi);

	/* Zero tolerance is a legal, if unforgiving, configuration. */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_rb_window(&x, 0U, 0U, &lo, &hi));
	TEST_ASSERT_EQUAL_INT32(4515, lo);
	TEST_ASSERT_EQUAL_INT32(4515, hi);

	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_rb_window(NULL, 0U, 5U, &lo, &hi));
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_rb_window(&x, 0U, 5U, NULL, &hi));
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_rb_window(&x, 0U, 5U, &lo, NULL));
}

/* ------------------------------------------------------ derived predicates */

static void test_poe_headroom_never_wraps(void)
{
	pwrseq_in_t in;

	memset(&in, 0, sizeof(in));
	in.poe_granted_mw = 51000U;
	in.poe_measured_mw = 14000U;
	TEST_ASSERT_EQUAL_UINT32(37000U, pwrseq_poe_headroom_mw(&in));

	/* Drawing more than granted must read as no headroom, not 4 GW. */
	in.poe_measured_mw = 60000U;
	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_poe_headroom_mw(&in));

	in.poe_measured_mw = 51000U;
	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_poe_headroom_mw(&in));

	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_poe_headroom_mw(NULL));
}

static void test_ocxo_warm_needs_both_current_and_temperature(void)
{
	pwrseq_ctx_t ctx;
	pwrseq_in_t in;

	TEST_ASSERT_EQUAL_INT(0, pwrseq_init(&ctx, NULL));
	in_healthy(&in);

	/* Settled draw plus a stable oscillator temperature. */
	in.ina_current_ma[INA228_RAIL_OCXO] = 300;
	TEST_ASSERT_TRUE(pwrseq_ocxo_is_warm(&ctx, &in));
	TEST_ASSERT_FALSE(pwrseq_ocxo_is_warming(&ctx, &in));

	/* A low reading alone is not enough: an oven that has only just been
	 * switched on also draws little for a moment. */
	in.ocxo_temp_stable = false;
	TEST_ASSERT_FALSE(pwrseq_ocxo_is_warm(&ctx, &in));

	/* Nor is a stale reading. */
	in.ocxo_temp_stable = true;
	in.ina_valid[INA228_RAIL_OCXO] = false;
	TEST_ASSERT_FALSE(pwrseq_ocxo_is_warm(&ctx, &in));
	TEST_ASSERT_FALSE(pwrseq_ocxo_is_warming(&ctx, &in));

	/* The discipline loop can declare it outright. */
	in.ocxo_warm = true;
	TEST_ASSERT_TRUE(pwrseq_ocxo_is_warm(&ctx, &in));

	/* Cold oven: over the warm-up threshold, so warming, not warm. */
	in.ocxo_warm = false;
	in.ina_valid[INA228_RAIL_OCXO] = true;
	in.ina_current_ma[INA228_RAIL_OCXO] = 900;
	TEST_ASSERT_FALSE(pwrseq_ocxo_is_warm(&ctx, &in));
	TEST_ASSERT_TRUE(pwrseq_ocxo_is_warming(&ctx, &in));

	/* Between the two thresholds: neither warm nor still warming. */
	in.ina_current_ma[INA228_RAIL_OCXO] = 500;
	TEST_ASSERT_FALSE(pwrseq_ocxo_is_warm(&ctx, &in));
	TEST_ASSERT_FALSE(pwrseq_ocxo_is_warming(&ctx, &in));

	/* A negative reading (reverse current) is nonsense for this rail. */
	in.ina_current_ma[INA228_RAIL_OCXO] = -10;
	TEST_ASSERT_FALSE(pwrseq_ocxo_is_warm(&ctx, &in));
	TEST_ASSERT_FALSE(pwrseq_ocxo_is_warming(&ctx, &in));

	TEST_ASSERT_FALSE(pwrseq_ocxo_is_warm(NULL, &in));
	TEST_ASSERT_FALSE(pwrseq_ocxo_is_warm(&ctx, NULL));
	TEST_ASSERT_FALSE(pwrseq_ocxo_is_warming(NULL, &in));
	TEST_ASSERT_FALSE(pwrseq_ocxo_is_warming(&ctx, NULL));
}

/* -------------------------------------------------------------- happy path */

static void test_full_bring_up_emits_the_documented_sequence(void)
{
	static const uint8_t want[] = {
		/* Stage 2 — release held-in-reset digital devices */
		PWRSEQ_ACT_NOR_RST_RELEASE,
		PWRSEQ_ACT_I2C_PROBE,
		PWRSEQ_ACT_DISP_RST_RELEASE,
		/* Stage 3 — trims first, then the baseline and the rail checks */
		PWRSEQ_ACT_APPLY_SHUNT_TRIMS,
		PWRSEQ_ACT_READ_RAIL_BASELINE,
		/* Stage 4 — PHY */
		PWRSEQ_ACT_LAN_RST_RELEASE,
		PWRSEQ_ACT_PHY_MDIO_POLL,
		/* Stage 5 — GNSS and antenna */
		PWRSEQ_ACT_GPS_PWR_EN,
		PWRSEQ_ACT_GPS_RST_RELEASE,
		PWRSEQ_ACT_ANT_BIAS_EN,
		PWRSEQ_ACT_ANT_SUPERVISOR_START,
		PWRSEQ_ACT_GNSS_CONFIG_REQUEST,
		/* Stage 6 — display and panel */
		PWRSEQ_ACT_DISP_EN,
		PWRSEQ_ACT_PANEL_LED_EN,
		PWRSEQ_ACT_PANEL_LED_PWM,
		/* Stage 7 — discipline */
		PWRSEQ_ACT_DISC_START,
		/* Stage 8 — the guarded rubidium sequence */
		PWRSEQ_ACT_DIGIPOT_WRITE,
		PWRSEQ_ACT_DIGIPOT_VERIFY,
		PWRSEQ_ACT_RB_PWR_EN,
		PWRSEQ_ACT_RB_VCC_GATE_EN,
		/* Stage 9 — watchdog, then the relay */
		PWRSEQ_ACT_WDT_EN,
		PWRSEQ_ACT_WDT_KICK_START,
		PWRSEQ_ACT_RELAY_ELIGIBLE,
	};
	pwrseq_status_t st;
	model_t m;

	model_init(&m, NULL);
	run_out(&m, 100U, 100U);

	expect_seq(&m, want, ARRAY_LEN(want));

	TEST_ASSERT_EQUAL_INT(0, pwrseq_status(&m.ctx, &st));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, st.stage);
	TEST_ASSERT_FALSE(st.halted);
	TEST_ASSERT_FALSE(st.rb_deferred);
	TEST_ASSERT_EQUAL_UINT32(0U, st.alarms);
	TEST_ASSERT_TRUE(st.rb_enabled);
	TEST_ASSERT_TRUE(st.rb_gated);
	TEST_ASSERT_TRUE(st.rb_locked);
	TEST_ASSERT_TRUE(st.wdt_armed);
	TEST_ASSERT_TRUE(st.relay_eligible);
	TEST_ASSERT_TRUE(st.display_on);
	TEST_ASSERT_TRUE(st.panel_led_on);
	TEST_ASSERT_TRUE(st.gps_on);
	TEST_ASSERT_TRUE(st.ant_bias_on);
	TEST_ASSERT_TRUE(st.disc_started);
	TEST_ASSERT_TRUE(st.phy_released);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_SHED_NONE, st.shed);

	/* The sequencer reserves queue room before every step, so it can never
	 * be the source of a dropped action. */
	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_actions_dropped(&m.ctx));

	/* Idempotent once finished. */
	advance(&m, 1000U);
	TEST_ASSERT_EQUAL_size_t(ARRAY_LEN(want), m.log_len);

	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_status(NULL, &st));
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_status(&m.ctx, NULL));
}

static void test_action_arguments_come_from_config(void)
{
	pwrseq_cfg_t cfg;
	model_t m;
	int i;

	/*
	 * Drive the whole chain from a non-zero safe code — 25 is the 5 V point
	 * in the vcc_rb_supply §5 table — so the digipot argument, the expected
	 * rail voltage and the window all have to agree end to end. A hard-coded
	 * zero anywhere in that path fails here and nowhere else.
	 */
	pwrseq_cfg_default(&cfg);
	cfg.digipot_safe_code = 25U;
	cfg.panel_led_duty_pct = 40U;

	model_init(&m, &cfg);
	m.in.digipot_readback = 25U;
	m.in.ina_vbus_mv[INA228_RAIL_VCC_RB] = 5000;
	run_out(&m, 100U, 100U);

	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_TRUE(m.ctx.rb_gated);

	for (i = 0; i < (int)m.log_len; i++) {
		if (m.log[i].action == PWRSEQ_ACT_DIGIPOT_WRITE) {
			TEST_ASSERT_EQUAL_UINT16(25U, m.log[i].arg);
		} else if (m.log[i].action == PWRSEQ_ACT_PANEL_LED_PWM) {
			TEST_ASSERT_EQUAL_UINT16(40U, m.log[i].arg);
		} else {
			TEST_ASSERT_EQUAL_UINT16(0U, m.log[i].arg);
		}
	}
}

static void test_guards_skip_the_optional_stages(void)
{
	static const uint8_t want[] = {
		PWRSEQ_ACT_NOR_RST_RELEASE,
		PWRSEQ_ACT_I2C_PROBE,
		/* no DISP_RST_RELEASE: no UI */
		PWRSEQ_ACT_APPLY_SHUNT_TRIMS,
		PWRSEQ_ACT_READ_RAIL_BASELINE,
		PWRSEQ_ACT_LAN_RST_RELEASE,
		PWRSEQ_ACT_PHY_MDIO_POLL,
		PWRSEQ_ACT_GPS_PWR_EN,
		PWRSEQ_ACT_GPS_RST_RELEASE,
		PWRSEQ_ACT_ANT_BIAS_EN,
		PWRSEQ_ACT_ANT_SUPERVISOR_START,
		PWRSEQ_ACT_GNSS_CONFIG_REQUEST,
		/* no stage 6 at all */
		PWRSEQ_ACT_DISC_START,
		/* no stage 8 at all */
		/* no WDT: a debugger is attached */
		PWRSEQ_ACT_RELAY_ELIGIBLE,
	};
	model_t m;

	model_init(&m, NULL);
	m.in.ui_wanted = false;
	m.in.rb_wanted = false;
	m.in.debugger_attached = true;
	run_out(&m, 100U, 100U);

	expect_seq(&m, want, ARRAY_LEN(want));

	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_alarms(&m.ctx));
	TEST_ASSERT_FALSE(m.ctx.rb_enabled);
	TEST_ASSERT_FALSE(m.ctx.display_on);

	/*
	 * external_wdt §7.4: the TPS3430 has no debug-freeze input, so a
	 * debug session leaves it disarmed. Bring-up still completes and the
	 * relay still becomes eligible — only the watchdog is skipped.
	 */
	TEST_ASSERT_FALSE(pwrseq_wdt_armed(&m.ctx));
	TEST_ASSERT_TRUE(m.ctx.relay_eligible);
	TEST_ASSERT_FALSE(pwrseq_wdt_kick_ok(&m.ctx, PWRSEQ_LIVE_ALL, 100000U));
}

/* -------------------------------------------------- the rubidium interlocks */

static void test_rb_never_powers_before_the_digipot_is_verified(void)
{
	model_t m;

	model_init(&m, NULL);
	run_out(&m, 100U, 100U);

	/* Both must be present, and in this order — ARCHITECTURE.md
	 * invariant 3. */
	expect_present(&m, PWRSEQ_ACT_DIGIPOT_WRITE);
	expect_present(&m, PWRSEQ_ACT_DIGIPOT_VERIFY);
	expect_present(&m, PWRSEQ_ACT_RB_PWR_EN);
	TEST_ASSERT_TRUE(idx_of(&m, PWRSEQ_ACT_DIGIPOT_WRITE) <
			 idx_of(&m, PWRSEQ_ACT_DIGIPOT_VERIFY));
	TEST_ASSERT_TRUE(idx_of(&m, PWRSEQ_ACT_DIGIPOT_VERIFY) <
			 idx_of(&m, PWRSEQ_ACT_RB_PWR_EN));
	TEST_ASSERT_TRUE(idx_of(&m, PWRSEQ_ACT_RB_PWR_EN) <
			 idx_of(&m, PWRSEQ_ACT_RB_VCC_GATE_EN));
}

static void test_a_digipot_that_will_not_read_back_stops_the_sequence(void)
{
	model_t m;

	model_init(&m, NULL);
	m.in.digipot_readback_valid = false;
	run_out(&m, 100U, 200U);

	/* The write is attempted, the verify retried, and then the whole stage
	 * is abandoned — the rail is never energised on an unproven wiper. */
	TEST_ASSERT_EQUAL_UINT(1U, count_of(&m, PWRSEQ_ACT_DIGIPOT_WRITE));
	TEST_ASSERT_EQUAL_UINT(3U, count_of(&m, PWRSEQ_ACT_DIGIPOT_VERIFY));
	expect_absent(&m, PWRSEQ_ACT_RB_PWR_EN);
	expect_absent(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);

	/* Belt and braces: the failure path drives both lines low anyway. */
	expect_present(&m, PWRSEQ_ACT_RB_VCC_GATE_DIS);
	expect_present(&m, PWRSEQ_ACT_RB_PWR_DIS);
	TEST_ASSERT_TRUE(idx_of(&m, PWRSEQ_ACT_RB_VCC_GATE_DIS) <
			 idx_of(&m, PWRSEQ_ACT_RB_PWR_DIS));

	TEST_ASSERT_TRUE(pwrseq_rb_fault(&m.ctx));
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_DIGIPOT)) != 0U);

	/* The rest of bring-up still completes. */
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_TRUE(m.ctx.relay_eligible);
}

static void test_a_digipot_that_reads_back_the_wrong_code_is_rejected(void)
{
	model_t m;

	model_init(&m, NULL);
	/* POR mid-scale is about 14.5 V — inside the OV envelope, and lethal to
	 * a 15 V-class FE if it were trusted as the safe-low code. */
	m.in.digipot_readback = 512U;
	run_out(&m, 100U, 200U);

	expect_absent(&m, PWRSEQ_ACT_RB_PWR_EN);
	TEST_ASSERT_TRUE(pwrseq_rb_fault(&m.ctx));
}

static void test_rb_never_gates_before_the_rail_window_is_proven(void)
{
	model_t m;

	model_init(&m, NULL);
	/* Rail comes up at 12 V when 4.515 V +-5 % was commanded. */
	m.in.ina_vbus_mv[INA228_RAIL_VCC_RB] = 12000;
	run_out(&m, 100U, 200U);

	/* Powered, because that is how the rail is measured at all... */
	expect_present(&m, PWRSEQ_ACT_RB_PWR_EN);
	/* ...but never connected to the FE. */
	expect_absent(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);
	/* And dropped, load side first. */
	expect_present(&m, PWRSEQ_ACT_RB_PWR_DIS);
	TEST_ASSERT_TRUE(idx_of(&m, PWRSEQ_ACT_RB_VCC_GATE_DIS) <
			 idx_of(&m, PWRSEQ_ACT_RB_PWR_DIS));
	TEST_ASSERT_TRUE(idx_of(&m, PWRSEQ_ACT_RB_PWR_EN) <
			 idx_of(&m, PWRSEQ_ACT_RB_PWR_DIS));

	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_WINDOW)) != 0U);
	TEST_ASSERT_TRUE(pwrseq_rb_fault(&m.ctx));
	TEST_ASSERT_FALSE(m.ctx.rb_enabled);
	TEST_ASSERT_FALSE(m.ctx.rb_gated);

	/* Bring-up still finishes on the OCXO. */
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_TRUE(m.ctx.relay_eligible);
}

static void test_a_rail_just_outside_the_window_is_still_rejected(void)
{
	model_t m;

	/* 4515 + 5 % is 4741; one millivolt past it must fail. */
	model_init(&m, NULL);
	m.in.ina_vbus_mv[INA228_RAIL_VCC_RB] = 4742;
	run_out(&m, 100U, 200U);
	expect_absent(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);

	/* And exactly on the boundary must pass. */
	model_init(&m, NULL);
	m.in.ina_vbus_mv[INA228_RAIL_VCC_RB] = 4741;
	run_out(&m, 100U, 200U);
	expect_present(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);
	TEST_ASSERT_FALSE(pwrseq_rb_fault(&m.ctx));

	model_init(&m, NULL);
	m.in.ina_vbus_mv[INA228_RAIL_VCC_RB] = 4289;
	run_out(&m, 100U, 200U);
	expect_present(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);

	model_init(&m, NULL);
	m.in.ina_vbus_mv[INA228_RAIL_VCC_RB] = 4288;
	run_out(&m, 100U, 200U);
	expect_absent(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);
}

static void test_a_stale_vcc_rb_reading_is_not_a_pass(void)
{
	model_t m;

	model_init(&m, NULL);
	m.in.ina_valid[INA228_RAIL_VCC_RB] = false;
	run_out(&m, 100U, 200U);

	/* Stage 3's baseline needs all nine, so the sequence never reaches the
	 * rubidium at all — but if it did, an invalid reading must not satisfy
	 * the window either. */
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RAIL_BASELINE)) != 0U);
	expect_absent(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);
}

static void test_lock_timeout_drops_the_rubidium(void)
{
	model_t m;

	model_init(&m, NULL);
	m.in.rb_lock = false;
	run_out(&m, 60000U, 40U); /* 10 min lock timeout, in minute steps */

	/* It was powered and gated — the rail was fine — but never locked. */
	expect_present(&m, PWRSEQ_ACT_RB_PWR_EN);
	expect_present(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);
	expect_present(&m, PWRSEQ_ACT_RB_PWR_DIS);
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_LOCK)) != 0U);
	TEST_ASSERT_TRUE(pwrseq_rb_fault(&m.ctx));
	TEST_ASSERT_FALSE(m.ctx.rb_locked);
}

static void test_an_out_of_band_external_reference_is_not_a_lock(void)
{
	model_t m;

	/* Interface ref §3: the dual guard is EXTREF_MON in band *and*
	 * RB_LOCK. One without the other is not a usable reference. */
	model_init(&m, NULL);
	m.in.extref_in_band = false;
	run_out(&m, 60000U, 40U);

	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_LOCK)) != 0U);
	TEST_ASSERT_FALSE(m.ctx.rb_locked);
}

static void test_a_rail_excursion_after_gating_drops_the_rubidium(void)
{
	model_t m;
	size_t settled;

	model_init(&m, NULL);
	run_out(&m, 100U, 100U);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_TRUE(m.ctx.rb_gated);
	settled = m.log_len;

	/*
	 * Supervision does not stop when bring-up does. A rail that wanders out
	 * of its window hours later is dropped just as promptly.
	 */
	m.in.ina_vbus_mv[INA228_RAIL_VCC_RB] = 9000;
	advance(&m, 1000U);

	TEST_ASSERT_EQUAL_size_t(settled + 2U, m.log_len);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_RB_VCC_GATE_DIS,
			      (int)m.log[settled].action);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_RB_PWR_DIS,
			      (int)m.log[settled + 1U].action);
	TEST_ASSERT_TRUE(pwrseq_rb_fault(&m.ctx));
	TEST_ASSERT_FALSE(m.ctx.rb_enabled);

	/* Once dropped, it stays dropped — no oscillation. */
	advance(&m, 1000U);
	TEST_ASSERT_EQUAL_size_t(settled + 2U, m.log_len);
}

static void test_an_over_voltage_latch_drops_the_rubidium_at_any_stage(void)
{
	model_t m;
	size_t settled;

	model_init(&m, NULL);
	run_out(&m, 100U, 100U);
	settled = m.log_len;
	TEST_ASSERT_TRUE(m.ctx.rb_enabled);

	/* The 26 V latch has already killed the buck in hardware; firmware only
	 * observes, drops its own enables and records the cause. */
	m.in.rb_ov_det = true;
	advance(&m, 100U);

	TEST_ASSERT_TRUE(pwrseq_ov_latched(&m.ctx));
	TEST_ASSERT_EQUAL_size_t(settled + 2U, m.log_len);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_RB_VCC_GATE_DIS,
			      (int)m.log[settled].action);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_RB_PWR_DIS,
			      (int)m.log[settled + 1U].action);
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_OV)) != 0U);
	TEST_ASSERT_TRUE(pwrseq_rb_fault(&m.ctx));
}

/* ------------------------------------------------------------- PoE budget */

static void test_an_inadequate_poe_budget_defers_the_rubidium(void)
{
	model_t m;

	model_init(&m, NULL);
	/* Class 3 at 12.95 W granted, 14 W already drawn: no headroom at all,
	 * let alone the 16 W the rubidium cold start needs. */
	m.in.poe_granted_mw = 12950U;
	run_out(&m, 5000U, 40U);

	/* Nothing rubidium-shaped happened. */
	expect_absent(&m, PWRSEQ_ACT_DIGIPOT_WRITE);
	expect_absent(&m, PWRSEQ_ACT_RB_PWR_EN);
	expect_absent(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);

	/* Deferred and flagged, not faulted — the board runs on the OCXO. */
	TEST_ASSERT_TRUE(m.ctx.rb_deferred);
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_POE_BUDGET)) != 0U);
	TEST_ASSERT_FALSE(pwrseq_rb_fault(&m.ctx));

	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_TRUE(m.ctx.relay_eligible);
	TEST_ASSERT_TRUE(m.ctx.wdt_armed);
}

static void test_a_budget_exactly_at_the_cold_start_figure_is_enough(void)
{
	model_t m;

	model_init(&m, NULL);
	m.in.poe_granted_mw = 30000U;
	m.in.poe_measured_mw = 14000U; /* headroom exactly 16000 */
	run_out(&m, 100U, 100U);
	expect_present(&m, PWRSEQ_ACT_RB_PWR_EN);

	model_init(&m, NULL);
	m.in.poe_granted_mw = 29999U; /* one milliwatt short */
	m.in.poe_measured_mw = 14000U;
	run_out(&m, 5000U, 40U);
	expect_absent(&m, PWRSEQ_ACT_RB_PWR_EN);
	TEST_ASSERT_TRUE(m.ctx.rb_deferred);
}

static void test_shedding_the_display_frees_budget_for_a_retry(void)
{
	model_t m;

	model_init(&m, NULL);
	m.in.poe_granted_mw = 25500U; /* Class 4 */
	run_out(&m, 5000U, 40U);
	TEST_ASSERT_TRUE(m.ctx.rb_deferred);
	expect_absent(&m, PWRSEQ_ACT_RB_PWR_EN);

	/* Shed the display, which is the documented first item to go, and the
	 * measured draw falls. */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_shed_step(&m.ctx, m.in.mono_ms));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_SHED_DISPLAY, pwrseq_shed_level(&m.ctx));
	m.in.poe_measured_mw = 8000U; /* 17.5 W of headroom now */

	TEST_ASSERT_EQUAL_INT(0, pwrseq_rb_retry(&m.ctx, m.in.mono_ms));
	TEST_ASSERT_FALSE(m.ctx.rb_deferred);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_8_RB, pwrseq_stage(&m.ctx));
	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_alarms(&m.ctx));

	run_out(&m, 100U, 100U);
	expect_present(&m, PWRSEQ_ACT_RB_PWR_EN);
	expect_present(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));

	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_rb_retry(NULL, 0U));
}

static void test_uncharged_supercaps_defer_the_rubidium(void)
{
	model_t m;

	/* Interface ref §2 stage 8: budget AND warm OCXO AND charged
	 * supercaps. The budget is fine here; the supercaps are not. */
	model_init(&m, NULL);
	m.in.supercaps_charged = false;
	run_out(&m, 5000U, 40U);

	expect_absent(&m, PWRSEQ_ACT_RB_PWR_EN);
	TEST_ASSERT_TRUE(m.ctx.rb_deferred);
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_PRECONDITION)) != 0U);
	TEST_ASSERT_FALSE(pwrseq_rb_fault(&m.ctx));
}

/* --------------------------------------------------------- failure policies */

static void test_retry_reissues_the_action_then_escalates(void)
{
	model_t m;

	model_init(&m, NULL);
	m.in.nor_ready = false;

	pump(&m);
	TEST_ASSERT_EQUAL_UINT(1U, count_of(&m, PWRSEQ_ACT_NOR_RST_RELEASE));

	advance(&m, 100U); /* first timeout -> retry 1 */
	TEST_ASSERT_EQUAL_UINT(2U, count_of(&m, PWRSEQ_ACT_NOR_RST_RELEASE));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_2_RELEASE, pwrseq_stage(&m.ctx));

	advance(&m, 100U); /* second timeout -> retry 2 */
	TEST_ASSERT_EQUAL_UINT(3U, count_of(&m, PWRSEQ_ACT_NOR_RST_RELEASE));
	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_alarms(&m.ctx));

	advance(&m, 100U); /* retries exhausted -> alarm, abandon the stage */
	TEST_ASSERT_EQUAL_UINT(3U, count_of(&m, PWRSEQ_ACT_NOR_RST_RELEASE));
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_NOR)) != 0U);

	/*
	 * Abandoning the stage skips step 2.2 and 2.3 — I2C_PROBE never runs,
	 * because a device map read over a bus whose config store never came
	 * out of reset is not worth having. Stage 3 starts regardless.
	 */
	expect_absent(&m, PWRSEQ_ACT_I2C_PROBE);
	expect_present(&m, PWRSEQ_ACT_APPLY_SHUNT_TRIMS);
}

static void test_alarm_policy_abandons_the_stage_and_continues(void)
{
	model_t m;

	model_init(&m, NULL);
	m.in.gnss_cfg_ack = false;
	run_out(&m, 1000U, 60U);

	/* Three attempts at the config request, then on with bring-up. */
	TEST_ASSERT_EQUAL_UINT(3U,
			       count_of(&m, PWRSEQ_ACT_GNSS_CONFIG_REQUEST));
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_GNSS_CFG)) != 0U);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_FALSE(m.ctx.halted);
}

static void test_a_failed_gps_rail_never_reaches_the_antenna_bias(void)
{
	model_t m;

	model_init(&m, NULL);
	m.in.pg_mask &= (uint8_t)~PWRSEQ_PG_3V3_GPS_LDO;
	run_out(&m, 250U, 60U);

	/* Two attempts (one retry), then the rail is dropped and the rest of
	 * stage 5 is abandoned — no bias onto a receiver that has no supply. */
	TEST_ASSERT_EQUAL_UINT(2U, count_of(&m, PWRSEQ_ACT_GPS_PWR_EN));
	expect_present(&m, PWRSEQ_ACT_GPS_PWR_DIS);
	expect_absent(&m, PWRSEQ_ACT_GPS_RST_RELEASE);
	expect_absent(&m, PWRSEQ_ACT_ANT_BIAS_EN);
	expect_absent(&m, PWRSEQ_ACT_GNSS_CONFIG_REQUEST);
	TEST_ASSERT_FALSE(m.ctx.gps_on);
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_GPS_RAIL)) != 0U);

	/* The rest of the board still comes up. */
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
}

static void test_halt_policy_stops_the_sequencer(void)
{
	model_t m;
	size_t at_halt;

	model_init(&m, NULL);
	m.in.ina_vbus_mv[INA228_RAIL_3V3_MAIN] = 2000; /* nowhere near 3.3 V */
	run_out(&m, 250U, 40U);

	TEST_ASSERT_TRUE(m.ctx.halted);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_3_RAILS, pwrseq_stage(&m.ctx));
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RAIL_3V3)) != 0U);

	/*
	 * Nothing downstream ran: no gated load was enabled, the watchdog was
	 * never armed and the relay stays de-energized, which is the correct
	 * resting state for a hardware fault a cold cycle cannot fix.
	 */
	expect_absent(&m, PWRSEQ_ACT_GPS_PWR_EN);
	expect_absent(&m, PWRSEQ_ACT_RB_PWR_EN);
	expect_absent(&m, PWRSEQ_ACT_WDT_EN);
	expect_absent(&m, PWRSEQ_ACT_RELAY_ELIGIBLE);
	TEST_ASSERT_FALSE(m.ctx.relay_eligible);

	at_halt = m.log_len;
	advance(&m, 10000U);
	TEST_ASSERT_EQUAL_size_t(at_halt, m.log_len);
}

static void test_pg4_must_corroborate_the_3v3_telemetry(void)
{
	model_t m;

	/* Interface ref §3: confirm 3V3 via INA228 0x43 *corroborated by PG4*.
	 * A good reading with a dead power-good is not a good rail. */
	model_init(&m, NULL);
	m.in.pg_mask &= (uint8_t)~PWRSEQ_PG_3V3_PSU;
	run_out(&m, 250U, 40U);

	TEST_ASSERT_TRUE(m.ctx.halted);
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RAIL_3V3)) != 0U);
}

static void test_a_sagging_poe_bus_alarms_but_does_not_halt(void)
{
	model_t m;

	model_init(&m, NULL);
	m.in.ina_vbus_mv[INA228_RAIL_POE] = 40000; /* under the 42.5 V floor */
	run_out(&m, 250U, 60U);

	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RAIL_POE)) != 0U);
	TEST_ASSERT_FALSE(m.ctx.halted);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
}

static void test_stage_re_entry_after_a_failure(void)
{
	model_t m;

	model_init(&m, NULL);
	m.in.ina_vbus_mv[INA228_RAIL_3V3_MAIN] = 2000;
	run_out(&m, 250U, 40U);
	TEST_ASSERT_TRUE(m.ctx.halted);

	/* Fix the rail and re-enter the stage: the halt clears and the trim
	 * runs again from the top of stage 3. */
	m.in.ina_vbus_mv[INA228_RAIL_3V3_MAIN] = 3300;
	TEST_ASSERT_EQUAL_INT(
		0, pwrseq_restart_stage(&m.ctx, PWRSEQ_STAGE_3_RAILS,
					m.in.mono_ms));
	TEST_ASSERT_FALSE(m.ctx.halted);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_3_RAILS, pwrseq_stage(&m.ctx));

	run_out(&m, 100U, 100U);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_EQUAL_UINT(2U, count_of(&m, PWRSEQ_ACT_APPLY_SHUNT_TRIMS));
	TEST_ASSERT_TRUE(m.ctx.relay_eligible);

	/* The alarm from the first attempt is a record, not a live state — the
	 * caller decides whether to clear it. */
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RAIL_3V3)) != 0U);

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      pwrseq_restart_stage(&m.ctx,
						   PWRSEQ_STAGE_IDLE, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      pwrseq_restart_stage(&m.ctx,
						   PWRSEQ_STAGE_DONE, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      pwrseq_restart_stage(NULL, PWRSEQ_STAGE_3_RAILS,
						   0U));
}

/* --------------------------------------------------------- settle delays */

static void test_settle_delays_are_observed(void)
{
	model_t m;

	model_init(&m, NULL);

	/*
	 * A settle delay is measured from the scan that emitted the step's
	 * action, not from the start of the stage, so each wait below is
	 * anchored to the emission it follows.
	 */
	advance(&m, 10U); /* LAN_RST_RELEASE emitted here, at t = 10 */
	expect_present(&m, PWRSEQ_ACT_LAN_RST_RELEASE);
	expect_absent(&m, PWRSEQ_ACT_PHY_MDIO_POLL);

	advance(&m, 49U); /* 49 ms since the release: one short */
	expect_absent(&m, PWRSEQ_ACT_PHY_MDIO_POLL);

	advance(&m, 1U); /* 50 ms — the LAN8742 may be addressed now */
	expect_present(&m, PWRSEQ_ACT_PHY_MDIO_POLL);
	expect_present(&m, PWRSEQ_ACT_GPS_PWR_EN);
	expect_present(&m, PWRSEQ_ACT_GPS_RST_RELEASE);
	expect_absent(&m, PWRSEQ_ACT_ANT_BIAS_EN);

	advance(&m, 999U); /* F9T boot: 999 ms is not 1000 */
	expect_absent(&m, PWRSEQ_ACT_ANT_BIAS_EN);
	advance(&m, 1U);
	expect_present(&m, PWRSEQ_ACT_ANT_BIAS_EN);

	/* Rubidium soft-start: the rail is not sampled until the buck has had
	 * its 500 ms, or the window check would judge a ramp. */
	expect_present(&m, PWRSEQ_ACT_RB_PWR_EN);
	expect_absent(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);
	advance(&m, 499U);
	expect_absent(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);
	advance(&m, 1U);
	expect_present(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);
}

/* ------------------------------------------------------------ shed ladder */

static void test_shed_ladder_order_and_exhaustion(void)
{
	model_t m;
	size_t base;

	model_init(&m, NULL);
	run_out(&m, 100U, 100U);
	base = m.log_len;

	TEST_ASSERT_EQUAL_INT(PWRSEQ_SHED_NONE, pwrseq_shed_level(&m.ctx));

	/* Display first (interface ref §2 stage 6, spec §10.3). */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_shed_step(&m.ctx, 1000U));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_SHED_DISPLAY, pwrseq_shed_level(&m.ctx));

	/* Then the panel LEDs. */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_shed_step(&m.ctx, 1100U));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_SHED_PANEL_LED, pwrseq_shed_level(&m.ctx));

	/* Then the rubidium, load side first (spec §13's over-temp ladder). */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_shed_step(&m.ctx, 1200U));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_SHED_RB, pwrseq_shed_level(&m.ctx));

	/* Nothing further is sheddable short of POE_KILL. */
	TEST_ASSERT_EQUAL_INT(-ENOENT, pwrseq_shed_step(&m.ctx, 1300U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_shed_step(NULL, 0U));

	pump(&m);
	TEST_ASSERT_EQUAL_size_t(base + 4U, m.log_len);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_DISP_DIS, (int)m.log[base].action);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_PANEL_LED_DIS,
			      (int)m.log[base + 1U].action);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_RB_VCC_GATE_DIS,
			      (int)m.log[base + 2U].action);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_RB_PWR_DIS,
			      (int)m.log[base + 3U].action);

	TEST_ASSERT_EQUAL_INT(PWRSEQ_SHED_NONE, pwrseq_shed_level(NULL));
}

static void test_restoring_the_rubidium_re_runs_the_guarded_sequence(void)
{
	model_t m;
	size_t base;
	int i;

	model_init(&m, NULL);
	run_out(&m, 100U, 100U);

	TEST_ASSERT_EQUAL_INT(0, pwrseq_shed_step(&m.ctx, 1000U)); /* display */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_shed_step(&m.ctx, 1000U)); /* panel */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_shed_step(&m.ctx, 1000U)); /* rubidium */
	pump(&m);
	base = m.log_len;

	/*
	 * Restoring must NOT simply re-assert RB_PWR_EN. That would skip the
	 * digipot verify and the rail-window check, turning shed-and-restore
	 * into a back door around ARCHITECTURE.md invariant 3.
	 */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_shed_restore(&m.ctx, 2000U));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_SHED_PANEL_LED, pwrseq_shed_level(&m.ctx));
	TEST_ASSERT_EQUAL_size_t(base, m.log_len); /* no immediate actions */
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_8_RB, pwrseq_stage(&m.ctx));

	m.in.mono_ms = 2000U;
	run_out(&m, 100U, 100U);

	/* The whole interlock ran again, in order. */
	for (i = (int)base; i < (int)m.log_len; i++) {
		/* nothing before the digipot write may touch the rail */
		if (m.log[i].action == PWRSEQ_ACT_RB_PWR_EN) {
			break;
		}
	}
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_DIGIPOT_WRITE, (int)m.log[base].action);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_DIGIPOT_VERIFY,
			      (int)m.log[base + 1U].action);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_RB_PWR_EN, (int)m.log[base + 2U].action);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_RB_VCC_GATE_EN,
			      (int)m.log[base + 3U].action);
	TEST_ASSERT_TRUE(m.ctx.rb_gated);

	/* And back up the ladder for the UI loads. */
	base = m.log_len;
	TEST_ASSERT_EQUAL_INT(0, pwrseq_shed_restore(&m.ctx, 3000U));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_SHED_DISPLAY, pwrseq_shed_level(&m.ctx));
	TEST_ASSERT_EQUAL_INT(0, pwrseq_shed_restore(&m.ctx, 3100U));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_SHED_NONE, pwrseq_shed_level(&m.ctx));
	TEST_ASSERT_EQUAL_INT(-ENOENT, pwrseq_shed_restore(&m.ctx, 3200U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_shed_restore(NULL, 0U));

	pump(&m);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_PANEL_LED_EN, (int)m.log[base].action);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_PANEL_LED_PWM,
			      (int)m.log[base + 1U].action);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_DISP_EN, (int)m.log[base + 2U].action);
}

static void test_a_shed_load_is_not_re_enabled_by_a_stage_replay(void)
{
	model_t m;

	model_init(&m, NULL);
	run_out(&m, 100U, 100U);
	TEST_ASSERT_EQUAL_INT(0, pwrseq_shed_step(&m.ctx, 1000U)); /* display */
	pump(&m);

	/* Re-running stage 6 while the display is shed must respect the shed:
	 * a sequencer replay is not an excuse to spend the watts again. */
	{
		size_t base = m.log_len;

		TEST_ASSERT_EQUAL_INT(
			0, pwrseq_restart_stage(&m.ctx, PWRSEQ_STAGE_6_PANEL,
						2000U));
		m.in.mono_ms = 2000U;
		run_out(&m, 100U, 100U);

		/* Nothing after the shed re-enables the display... */
		TEST_ASSERT_EQUAL_INT(-1, idx_of_from(&m, PWRSEQ_ACT_DISP_EN,
						      base));
		/* ...while the panel LEDs, which are not shed, do come back. */
		TEST_ASSERT_TRUE(idx_of_from(&m, PWRSEQ_ACT_PANEL_LED_EN,
					     base) >= 0);
	}
	TEST_ASSERT_EQUAL_UINT(1U, count_of(&m, PWRSEQ_ACT_DISP_DIS));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
}

/* ---------------------------------------------------- over-voltage latch */

static void test_ov_clear_is_guarded(void)
{
	pwrseq_ctx_t ctx;
	pwrseq_act_t a;

	TEST_ASSERT_EQUAL_INT(0, pwrseq_init(&ctx, NULL));

	/* Nothing latched. */
	TEST_ASSERT_FALSE(pwrseq_ov_latched(&ctx));
	TEST_ASSERT_EQUAL_INT(-ENOENT, pwrseq_ov_clear(&ctx, 0U));

	TEST_ASSERT_EQUAL_INT(0, pwrseq_ov_observe(&ctx, true, 500U));
	TEST_ASSERT_TRUE(pwrseq_ov_latched(&ctx));
	TEST_ASSERT_EQUAL_UINT32(500U, ctx.ov_first_ms);
	TEST_ASSERT_EQUAL_UINT32(1U, ctx.ov_count);

	/* Re-observing while still asserted does not re-latch or re-count. */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_ov_observe(&ctx, true, 600U));
	TEST_ASSERT_EQUAL_UINT32(500U, ctx.ov_first_ms);
	TEST_ASSERT_EQUAL_UINT32(1U, ctx.ov_count);

	/*
	 * Pulsing the reset while the rail is still over-voltage would re-arm a
	 * supply already known to be running away.
	 */
	TEST_ASSERT_EQUAL_INT(-EBUSY, pwrseq_ov_clear(&ctx, 700U));
	TEST_ASSERT_TRUE(pwrseq_ov_latched(&ctx));
	TEST_ASSERT_EQUAL_size_t(0U, pwrseq_action_count(&ctx));

	/* Cause gone: now the pulse is allowed. */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_ov_observe(&ctx, false, 800U));
	TEST_ASSERT_TRUE(pwrseq_ov_latched(&ctx)); /* still latched until told */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_ov_clear(&ctx, 900U));
	TEST_ASSERT_FALSE(pwrseq_ov_latched(&ctx));

	TEST_ASSERT_EQUAL_INT(0, pwrseq_action_get(&ctx, &a));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_RB_OV_RESET_PULSE, (int)a.action);
	TEST_ASSERT_EQUAL_INT(-EAGAIN, pwrseq_action_get(&ctx, &a));

	/* A second trip latches again and counts separately. */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_ov_observe(&ctx, true, 1000U));
	TEST_ASSERT_EQUAL_UINT32(2U, ctx.ov_count);
	TEST_ASSERT_EQUAL_UINT32(1000U, ctx.ov_first_ms);

	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_ov_observe(NULL, true, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_ov_clear(NULL, 0U));
	TEST_ASSERT_FALSE(pwrseq_ov_latched(NULL));
}

/* ------------------------------------------------------------- power fail */

static void test_pfi_emits_the_park_list_in_priority_order(void)
{
	static const uint8_t want[] = {
		PWRSEQ_ACT_PARK_DAC,
		PWRSEQ_ACT_PERSIST_STATE,
		PWRSEQ_ACT_RB_VCC_GATE_DIS,
		PWRSEQ_ACT_RB_PWR_DIS,
		PWRSEQ_ACT_SET_SHUTDOWN_FLAG,
	};
	pwrseq_status_t st;
	model_t m;
	size_t i;

	model_init(&m, NULL);
	run_out(&m, 100U, 100U);
	TEST_ASSERT_TRUE(m.ctx.rb_enabled);

	/* Queue something the sequencer wanted, to prove PFI discards it. */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_shed_step(&m.ctx, 5000U));
	TEST_ASSERT_EQUAL_size_t(1U, pwrseq_action_count(&m.ctx));

	TEST_ASSERT_EQUAL_INT(0, pwrseq_pfi(&m.ctx, 5001U));
	TEST_ASSERT_EQUAL_size_t(ARRAY_LEN(want), pwrseq_action_count(&m.ctx));

	for (i = 0U; i < ARRAY_LEN(want); i++) {
		pwrseq_act_t a;

		TEST_ASSERT_EQUAL_INT(0, pwrseq_action_get(&m.ctx, &a));
		TEST_ASSERT_EQUAL_INT_MESSAGE(
			(int)want[i], (int)a.action,
			pwrseq_action_name((pwrseq_action_t)want[i]));
	}

	TEST_ASSERT_EQUAL_INT(0, pwrseq_status(&m.ctx, &st));
	TEST_ASSERT_TRUE(st.pfi_seen);
	/* No POE_KILL was commanded, so this is an unexpected line drop. */
	TEST_ASSERT_FALSE(st.pfi_expected);

	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_pfi(NULL, 0U));
}

static void test_pfi_without_a_running_rubidium_omits_the_shutdown(void)
{
	static const uint8_t want[] = {
		PWRSEQ_ACT_PARK_DAC,
		PWRSEQ_ACT_PERSIST_STATE,
		PWRSEQ_ACT_SET_SHUTDOWN_FLAG,
	};
	model_t m;
	size_t i;

	model_init(&m, NULL);
	m.in.rb_wanted = false;
	run_out(&m, 100U, 100U);

	TEST_ASSERT_EQUAL_INT(0, pwrseq_pfi(&m.ctx, 9000U));
	TEST_ASSERT_EQUAL_size_t(ARRAY_LEN(want), pwrseq_action_count(&m.ctx));
	for (i = 0U; i < ARRAY_LEN(want); i++) {
		pwrseq_act_t a;

		TEST_ASSERT_EQUAL_INT(0, pwrseq_action_get(&m.ctx, &a));
		TEST_ASSERT_EQUAL_INT((int)want[i], (int)a.action);
	}
}

static void test_a_commanded_kill_makes_the_following_pfi_expected(void)
{
	pwrseq_ctx_t ctx;
	pwrseq_status_t st;
	pwrseq_act_t a;

	TEST_ASSERT_EQUAL_INT(0, pwrseq_init(&ctx, NULL));

	/* Guarded: a stray call must not cold-cycle the box. */
	TEST_ASSERT_EQUAL_INT(-EPERM, pwrseq_poe_kill(&ctx, 0U, 100U));
	TEST_ASSERT_EQUAL_INT(-EPERM, pwrseq_poe_kill(&ctx, 0x4B494C4DU, 100U));
	TEST_ASSERT_EQUAL_size_t(0U, pwrseq_action_count(&ctx));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      pwrseq_poe_kill(NULL, PWRSEQ_POE_KILL_MAGIC, 0U));

	TEST_ASSERT_EQUAL_INT(
		0, pwrseq_poe_kill(&ctx, PWRSEQ_POE_KILL_MAGIC, 200U));
	TEST_ASSERT_EQUAL_INT(0, pwrseq_action_get(&ctx, &a));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_POE_KILL, (int)a.action);

	/*
	 * power_fail_input §5: the PFI handler must distinguish a deliberate
	 * cold cycle from an unexpected line drop so the two log differently.
	 */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_pfi(&ctx, 400U));
	TEST_ASSERT_EQUAL_INT(0, pwrseq_status(&ctx, &st));
	TEST_ASSERT_TRUE(st.pfi_seen);
	TEST_ASSERT_TRUE(st.pfi_expected);
}

/* ---------------------------------------------------------------- watchdog */

static void test_wdt_kick_requires_arming_liveness_and_cadence(void)
{
	model_t m;
	uint32_t t;

	model_init(&m, NULL);
	run_out(&m, 100U, 100U);
	TEST_ASSERT_TRUE(pwrseq_wdt_armed(&m.ctx));

	t = m.in.mono_ms;

	/* The cadence has not elapsed since WDT_KICK_START. */
	TEST_ASSERT_FALSE(pwrseq_wdt_kick_ok(&m.ctx, PWRSEQ_LIVE_ALL, t));

	t = m.ctx.last_kick_ms + PWRSEQ_WDT_KICK_PERIOD_MS - 1U;
	TEST_ASSERT_FALSE(pwrseq_wdt_kick_ok(&m.ctx, PWRSEQ_LIVE_ALL, t));
	t++;
	TEST_ASSERT_TRUE(pwrseq_wdt_kick_ok(&m.ctx, PWRSEQ_LIVE_ALL, t));

	/*
	 * Liveness is an AND gate. Kicking a windowed watchdog on a bare timer
	 * protects nothing — a stalled thread has to be able to stop the kick.
	 */
	TEST_ASSERT_FALSE(pwrseq_wdt_kick_ok(&m.ctx, 0U, t));
	TEST_ASSERT_FALSE(
		pwrseq_wdt_kick_ok(&m.ctx, PWRSEQ_LIVE_TIMING, t));
	TEST_ASSERT_FALSE(pwrseq_wdt_kick_ok(
		&m.ctx, PWRSEQ_LIVE_TIMING | PWRSEQ_LIVE_NETWORK, t));
	TEST_ASSERT_FALSE(pwrseq_wdt_kick_ok(
		&m.ctx, PWRSEQ_LIVE_NETWORK | PWRSEQ_LIVE_HOUSEKEEPING, t));
	TEST_ASSERT_TRUE(pwrseq_wdt_kick_ok(&m.ctx, PWRSEQ_LIVE_ALL, t));

	/* Recording a kick restarts the cadence. */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_wdt_kicked(&m.ctx, t));
	TEST_ASSERT_FALSE(pwrseq_wdt_kick_ok(&m.ctx, PWRSEQ_LIVE_ALL, t));
	TEST_ASSERT_TRUE(pwrseq_wdt_kick_ok(
		&m.ctx, PWRSEQ_LIVE_ALL, t + PWRSEQ_WDT_KICK_PERIOD_MS));

	/* Wrap-safe. */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_wdt_kicked(&m.ctx, 0xFFFFFF00U));
	TEST_ASSERT_FALSE(pwrseq_wdt_kick_ok(&m.ctx, PWRSEQ_LIVE_ALL, 0x00000100U));
	TEST_ASSERT_TRUE(pwrseq_wdt_kick_ok(&m.ctx, PWRSEQ_LIVE_ALL, 0x00000400U));

	TEST_ASSERT_FALSE(pwrseq_wdt_kick_ok(NULL, PWRSEQ_LIVE_ALL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_wdt_kicked(NULL, 0U));
	TEST_ASSERT_FALSE(pwrseq_wdt_armed(NULL));
}

static void test_wdt_is_not_armed_until_liveness_passes(void)
{
	model_t m;

	model_init(&m, NULL);
	m.in.liveness_ok = false;
	run_out(&m, 5000U, 40U);

	/* Stage 9's liveness gate never opened: no arming, and — because the
	 * stage is abandoned — no relay eligibility either. */
	expect_absent(&m, PWRSEQ_ACT_WDT_EN);
	expect_absent(&m, PWRSEQ_ACT_RELAY_ELIGIBLE);
	TEST_ASSERT_FALSE(pwrseq_wdt_armed(&m.ctx));
	TEST_ASSERT_FALSE(m.ctx.relay_eligible);
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_LIVENESS)) != 0U);
}

/* ------------------------------------------------------------ action queue */

static void test_the_step_machine_never_drops_an_action(void)
{
	model_t m;
	pwrseq_act_t a;
	unsigned int i;

	model_init(&m, NULL);

	/*
	 * A whole bring-up is 23 actions against a 32-slot queue, so a glue
	 * layer that only drains once at the end still loses nothing. That
	 * headroom is the reason the queue is sized as it is.
	 */
	for (i = 0U; i < 100U; i++) {
		m.in.mono_ms += 100U;
		TEST_ASSERT_EQUAL_INT(0, pwrseq_step(&m.ctx, &m.in));
	}

	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_actions_dropped(&m.ctx));
	TEST_ASSERT_EQUAL_size_t(23U, pwrseq_action_count(&m.ctx));
	TEST_ASSERT_TRUE(pwrseq_action_count(&m.ctx) <
			 (size_t)PWRSEQ_ACT_QUEUE_LEN);

	while (pwrseq_action_get(&m.ctx, &a) == 0) {
	}

	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_action_get(NULL, &a));
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_action_get(&m.ctx, NULL));
	TEST_ASSERT_EQUAL_size_t(0U, pwrseq_action_count(NULL));
	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_actions_dropped(NULL));
}

static void test_the_step_machine_stalls_rather_than_overflowing(void)
{
	model_t m;
	pwrseq_act_t a;
	unsigned int i;

	model_init(&m, NULL);

	/*
	 * Leave fewer than one step's worth of slots free. The sequencer must
	 * stop rather than emit into a queue it cannot fit into — a dropped
	 * enable would leave this module's model of the board wrong.
	 */
	for (i = 0U; i < (unsigned int)(PWRSEQ_ACT_QUEUE_LEN -
					PWRSEQ_ACT_PER_STEP_MAX + 1); i++) {
		TEST_ASSERT_EQUAL_INT(
			0, pwrseq_poe_kill(&m.ctx, PWRSEQ_POE_KILL_MAGIC, i));
	}

	m.in.mono_ms = 100U;
	TEST_ASSERT_EQUAL_INT(0, pwrseq_step(&m.ctx, &m.in));

	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_2_RELEASE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_actions_dropped(&m.ctx));
	TEST_ASSERT_EQUAL_size_t(
		(size_t)(PWRSEQ_ACT_QUEUE_LEN - PWRSEQ_ACT_PER_STEP_MAX + 1),
		pwrseq_action_count(&m.ctx));

	/* Draining lets it pick up exactly where it stopped. */
	while (pwrseq_action_get(&m.ctx, &a) == 0) {
	}
	run_out(&m, 100U, 100U);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_actions_dropped(&m.ctx));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_NOR_RST_RELEASE, (int)m.log[0].action);
}

static void test_an_out_of_band_caller_that_never_drains_is_counted(void)
{
	pwrseq_ctx_t ctx;
	unsigned int i;

	TEST_ASSERT_EQUAL_INT(0, pwrseq_init(&ctx, NULL));

	/* Only the out-of-band entry points can overflow, and when they do the
	 * loss is counted rather than hidden. */
	for (i = 0U; i < (unsigned int)PWRSEQ_ACT_QUEUE_LEN; i++) {
		TEST_ASSERT_EQUAL_INT(
			0, pwrseq_poe_kill(&ctx, PWRSEQ_POE_KILL_MAGIC, i));
	}
	TEST_ASSERT_EQUAL_size_t((size_t)PWRSEQ_ACT_QUEUE_LEN,
				 pwrseq_action_count(&ctx));
	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_actions_dropped(&ctx));

	TEST_ASSERT_EQUAL_INT(
		0, pwrseq_poe_kill(&ctx, PWRSEQ_POE_KILL_MAGIC, 999U));
	TEST_ASSERT_EQUAL_UINT32(1U, pwrseq_actions_dropped(&ctx));
}

static void test_rb_shutdown_waits_for_queue_room_rather_than_half_applying(void)
{
	model_t m;
	unsigned int i;
	pwrseq_act_t a;

	model_init(&m, NULL);
	run_out(&m, 100U, 100U);
	TEST_ASSERT_TRUE(m.ctx.rb_enabled);

	/* Wedge the queue with undrained out-of-band actions. */
	for (i = 0U; i < (unsigned int)PWRSEQ_ACT_QUEUE_LEN; i++) {
		TEST_ASSERT_EQUAL_INT(
			0, pwrseq_poe_kill(&m.ctx, PWRSEQ_POE_KILL_MAGIC, i));
	}

	/*
	 * An over-voltage now. Both halves of the shutdown must land together
	 * or not at all — dropping RB_PWR_DIS while RB_VCC_GATE_DIS went
	 * through would leave the buck running into a disconnected load and
	 * this module believing otherwise.
	 */
	m.in.rb_ov_det = true;
	m.in.mono_ms += 100U;
	TEST_ASSERT_EQUAL_INT(0, pwrseq_step(&m.ctx, &m.in));
	TEST_ASSERT_TRUE(pwrseq_ov_latched(&m.ctx));
	TEST_ASSERT_TRUE(m.ctx.rb_enabled); /* deferred, not half-applied */
	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_actions_dropped(&m.ctx));

	while (pwrseq_action_get(&m.ctx, &a) == 0) {
	}

	m.in.mono_ms += 100U;
	TEST_ASSERT_EQUAL_INT(0, pwrseq_step(&m.ctx, &m.in));
	TEST_ASSERT_FALSE(m.ctx.rb_enabled);
	TEST_ASSERT_EQUAL_INT(0, pwrseq_action_get(&m.ctx, &a));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_RB_VCC_GATE_DIS, (int)a.action);
	TEST_ASSERT_EQUAL_INT(0, pwrseq_action_get(&m.ctx, &a));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_RB_PWR_DIS, (int)a.action);
}

/* --------------------------------------------------------------- misc API */

static void test_null_and_bounds_handling(void)
{
	pwrseq_ctx_t ctx;
	pwrseq_in_t in;

	TEST_ASSERT_EQUAL_INT(0, pwrseq_init(&ctx, NULL));
	in_healthy(&in);

	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_step(NULL, &in));
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_step(&ctx, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_start(NULL, 0U));

	/* Stepping before start does nothing. */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_step(&ctx, &in));
	TEST_ASSERT_EQUAL_size_t(0U, pwrseq_action_count(&ctx));
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_IDLE, pwrseq_stage(&ctx));

	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_alarms(NULL));
	TEST_ASSERT_FALSE(pwrseq_rb_fault(NULL));
}

static void test_names_are_present_for_every_action_and_alarm(void)
{
	int i;

	for (i = 0; i < (int)PWRSEQ_ACT_COUNT; i++) {
		const char *s = pwrseq_action_name((pwrseq_action_t)i);

		TEST_ASSERT_NOT_NULL(s);
		TEST_ASSERT_TRUE(s[0] != '\0');
		TEST_ASSERT_EQUAL_STRING_MESSAGE(s, s, "name lookup");
	}
	for (i = 0; i < (int)PWRSEQ_ALARM_COUNT; i++) {
		const char *s = pwrseq_alarm_name((pwrseq_alarm_t)i);

		TEST_ASSERT_NOT_NULL(s);
		TEST_ASSERT_TRUE(s[0] != '\0');
	}

	TEST_ASSERT_EQUAL_STRING("rb-pwr-en",
				 pwrseq_action_name(PWRSEQ_ACT_RB_PWR_EN));
	TEST_ASSERT_EQUAL_STRING("rb-window",
				 pwrseq_alarm_name(PWRSEQ_ALARM_RB_WINDOW));
	TEST_ASSERT_EQUAL_STRING("invalid",
				 pwrseq_action_name(PWRSEQ_ACT_COUNT));
	TEST_ASSERT_EQUAL_STRING("invalid",
				 pwrseq_alarm_name(PWRSEQ_ALARM_COUNT));
	TEST_ASSERT_EQUAL_STRING("invalid",
				 pwrseq_action_name((pwrseq_action_t)-1));
	TEST_ASSERT_EQUAL_STRING("invalid",
				 pwrseq_alarm_name((pwrseq_alarm_t)-1));
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_init_defaults_and_validation);
	RUN_TEST(test_wdt_cadence_outside_the_window_is_rejected);

	RUN_TEST(test_digipot_transfer_matches_the_documented_table);
	RUN_TEST(test_rb_acceptance_window);

	RUN_TEST(test_poe_headroom_never_wraps);
	RUN_TEST(test_ocxo_warm_needs_both_current_and_temperature);

	RUN_TEST(test_full_bring_up_emits_the_documented_sequence);
	RUN_TEST(test_action_arguments_come_from_config);
	RUN_TEST(test_guards_skip_the_optional_stages);

	RUN_TEST(test_rb_never_powers_before_the_digipot_is_verified);
	RUN_TEST(test_a_digipot_that_will_not_read_back_stops_the_sequence);
	RUN_TEST(test_a_digipot_that_reads_back_the_wrong_code_is_rejected);
	RUN_TEST(test_rb_never_gates_before_the_rail_window_is_proven);
	RUN_TEST(test_a_rail_just_outside_the_window_is_still_rejected);
	RUN_TEST(test_a_stale_vcc_rb_reading_is_not_a_pass);
	RUN_TEST(test_lock_timeout_drops_the_rubidium);
	RUN_TEST(test_an_out_of_band_external_reference_is_not_a_lock);
	RUN_TEST(test_a_rail_excursion_after_gating_drops_the_rubidium);
	RUN_TEST(test_an_over_voltage_latch_drops_the_rubidium_at_any_stage);

	RUN_TEST(test_an_inadequate_poe_budget_defers_the_rubidium);
	RUN_TEST(test_a_budget_exactly_at_the_cold_start_figure_is_enough);
	RUN_TEST(test_shedding_the_display_frees_budget_for_a_retry);
	RUN_TEST(test_uncharged_supercaps_defer_the_rubidium);

	RUN_TEST(test_retry_reissues_the_action_then_escalates);
	RUN_TEST(test_alarm_policy_abandons_the_stage_and_continues);
	RUN_TEST(test_a_failed_gps_rail_never_reaches_the_antenna_bias);
	RUN_TEST(test_halt_policy_stops_the_sequencer);
	RUN_TEST(test_pg4_must_corroborate_the_3v3_telemetry);
	RUN_TEST(test_a_sagging_poe_bus_alarms_but_does_not_halt);
	RUN_TEST(test_stage_re_entry_after_a_failure);
	RUN_TEST(test_settle_delays_are_observed);

	RUN_TEST(test_shed_ladder_order_and_exhaustion);
	RUN_TEST(test_restoring_the_rubidium_re_runs_the_guarded_sequence);
	RUN_TEST(test_a_shed_load_is_not_re_enabled_by_a_stage_replay);

	RUN_TEST(test_ov_clear_is_guarded);

	RUN_TEST(test_pfi_emits_the_park_list_in_priority_order);
	RUN_TEST(test_pfi_without_a_running_rubidium_omits_the_shutdown);
	RUN_TEST(test_a_commanded_kill_makes_the_following_pfi_expected);

	RUN_TEST(test_wdt_kick_requires_arming_liveness_and_cadence);
	RUN_TEST(test_wdt_is_not_armed_until_liveness_passes);

	RUN_TEST(test_the_step_machine_never_drops_an_action);
	RUN_TEST(test_the_step_machine_stalls_rather_than_overflowing);
	RUN_TEST(test_an_out_of_band_caller_that_never_drains_is_counted);
	RUN_TEST(test_rb_shutdown_waits_for_queue_room_rather_than_half_applying);

	RUN_TEST(test_null_and_bounds_handling);
	RUN_TEST(test_names_are_present_for_every_action_and_alarm);

	return UNITY_END();
}
