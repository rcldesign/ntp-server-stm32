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

	/*
	 * Responsive rubidium "glue". The old harness fed a single static input,
	 * which could not model the two-code sequence (the rail is 4.5 V after
	 * the safe write and ~14 V after the operating write) and — as MEDIUM-4
	 * noted — let a physically impossible happy path pass (an FE "locking" at
	 * 4.5 V). When rb_glue is on, pump() reflects the digipot writes back into
	 * the readback register and drives VCC_RB to the voltage the *commanded*
	 * code implies, exactly as the real buck + digipot would. Fault injectors
	 * override specific responses; everything non-rubidium the test still sets
	 * directly.
	 */
	bool rb_glue;
	uint16_t rb_cmd;     /* last digipot code the glue saw written */
	bool rb_powered;     /* RB_PWR_EN asserted per the action stream */
	bool bad_readback;   /* inject: readback never valid (SPI dead) */
	bool wrong_readback; /* inject: readback matches no commanded code */
	bool hold_no_lock;   /* inject: RB_LOCK never asserts */
	bool hold_no_extref; /* inject: EXTREF_MON never in band */
	int32_t force_op_rail_mv; /* inject: rail while operating code active (0=auto) */
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
	m->rb_glue = true;
	m->rb_cmd = m->ctx.cfg.digipot_safe_code;
	TEST_ASSERT_EQUAL_INT(0, pwrseq_start(&m->ctx, 0U));
}

/*
 * Model the buck + digipot response to the actions just emitted: the readback
 * register follows the last code written, and VCC_RB regulates to the voltage
 * that code implies once RB_PWR_EN is up. This is what makes the two-code
 * guarded sequence testable end to end.
 */
static void glue_apply(model_t *m, size_t from)
{
	size_t i;

	for (i = from; i < m->log_len; i++) {
		switch (m->log[i].action) {
		case PWRSEQ_ACT_DIGIPOT_WRITE:
		case PWRSEQ_ACT_DIGIPOT_WRITE_OP:
			m->rb_cmd = m->log[i].arg;
			break;
		case PWRSEQ_ACT_RB_PWR_EN:
			m->rb_powered = true;
			break;
		case PWRSEQ_ACT_RB_PWR_DIS:
			m->rb_powered = false;
			break;
		default:
			break;
		}
	}

	if (m->bad_readback) {
		m->in.digipot_readback_valid = false;
	} else if (m->wrong_readback) {
		/* A value that matches neither the safe nor the operating code. */
		m->in.digipot_readback = (uint16_t)(m->rb_cmd + 1U);
		m->in.digipot_readback_valid = true;
	} else {
		m->in.digipot_readback = m->rb_cmd;
		m->in.digipot_readback_valid = true;
	}

	if (!m->rb_powered) {
		m->in.ina_vbus_mv[INA228_RAIL_VCC_RB] = 0;
	} else if ((m->force_op_rail_mv != 0) &&
		   (m->rb_cmd == m->ctx.cfg.digipot_operating_code)) {
		m->in.ina_vbus_mv[INA228_RAIL_VCC_RB] = m->force_op_rail_mv;
	} else {
		m->in.ina_vbus_mv[INA228_RAIL_VCC_RB] =
			pwrseq_rb_expected_mv(&m->ctx.cfg.rb_xfer, m->rb_cmd);
	}

	m->in.rb_lock = !m->hold_no_lock;
	m->in.extref_in_band = !m->hold_no_extref;
}

/* One step() call, draining every action into the log. */
static void pump(model_t *m)
{
	pwrseq_act_t a;
	size_t before = m->log_len;

	TEST_ASSERT_EQUAL_INT(0, pwrseq_step(&m->ctx, &m->in));
	while (pwrseq_action_get(&m->ctx, &a) == 0) {
		TEST_ASSERT_TRUE_MESSAGE(m->log_len < LOG_MAX,
					 "action log overflowed");
		m->log[m->log_len++] = a;
	}
	if (m->rb_glue) {
		glue_apply(m, before);
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

/*
 * Pump until @p stage is reached. Used by the tests that have to spoil a
 * reading *after* stage 3's baseline has accepted it — stage 3 demands all nine
 * monitors report, so a rail cannot simply start out unreadable.
 */
static void run_to_stage(model_t *m, pwrseq_stage_t stage, uint32_t step_ms,
			 unsigned int max_iters)
{
	unsigned int i;

	pump(m);
	for (i = 0U; i < max_iters; i++) {
		if ((pwrseq_stage(&m->ctx) >= stage) || m->ctx.halted) {
			return;
		}
		advance(m, step_ms);
	}
	TEST_FAIL_MESSAGE("never reached the requested stage");
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

/* Advance 1 ms per step until @p a has been emitted; fail if it does not appear
 * within @p budget_ms. Robust to the glue's per-step response roundtrips. */
static void advance_until(model_t *m, pwrseq_action_t a, uint32_t budget_ms)
{
	uint32_t start = m->in.mono_ms;

	while (idx_of(m, a) < 0) {
		TEST_ASSERT_TRUE_MESSAGE(m->in.mono_ms - start <= budget_ms,
					 pwrseq_action_name(a));
		advance(m, 1U);
	}
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

	/*
	 * BLOCKER-1: an out-of-range code must clamp to the SAFE-LOW end, never
	 * toward steps-1. Since VOUT rises with the code, clamping to steps-1
	 * would turn any garbage code into ~24.4 V — maximum output onto a
	 * 15 V-class FE. A code at or above `steps` therefore returns the
	 * code-0 VCTRL (= vref, minimum VOUT), not the code-1023 VCTRL.
	 */
	TEST_ASSERT_EQUAL_UINT32(pwrseq_digipot_vctrl_mv(&x, 0U),
				 pwrseq_digipot_vctrl_mv(&x, 5000U));
	TEST_ASSERT_EQUAL_UINT32(3000U, pwrseq_digipot_vctrl_mv(&x, 5000U));
	TEST_ASSERT_EQUAL_INT32(4515, pwrseq_rb_expected_mv(&x, 5000U));
	TEST_ASSERT_EQUAL_INT32(4515, pwrseq_rb_expected_mv(&x, 1024U));
	/* Emphatically NOT the maximum-output end. */
	TEST_ASSERT_TRUE(pwrseq_rb_expected_mv(&x, 5000U) <
			 pwrseq_rb_expected_mv(&x, 1023U));

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

static void test_rb_window_survives_a_degenerate_transfer_function(void)
{
	/*
	 * The transfer function comes from configuration, so a wrong pedestal
	 * or gain can drive the expected voltage negative. The window must stay
	 * an ordered interval around it — a tolerance that inherited the sign
	 * would produce hi < lo, which reads as "always out of window" for the
	 * over-voltage case and, worse, could read as "always in" if the
	 * comparison were written the other way round.
	 */
	pwrseq_rb_xfer_t bad = {
		.vref_mv = 3000U,
		.steps = 1024U,
		.pedestal_mv = 0U,
		.gain_m1000 = 335U,
	};
	int32_t lo = 0;
	int32_t hi = 0;

	TEST_ASSERT_EQUAL_INT32(-1005, pwrseq_rb_expected_mv(&bad, 0U));

	TEST_ASSERT_EQUAL_INT(0, pwrseq_rb_window(&bad, 0U, 5U, &lo, &hi));
	TEST_ASSERT_EQUAL_INT32(-1055, lo);
	TEST_ASSERT_EQUAL_INT32(-955, hi);
	TEST_ASSERT_TRUE(lo < hi);
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
		/* Stage 8 — the guarded rubidium sequence. Safe-low precharge
		 * is written and verified and the rail brought up at it, THEN
		 * the bounded operating code is written, verified and the rail
		 * re-checked against the FE ceiling, and only then is the FE
		 * connected. */
		PWRSEQ_ACT_DIGIPOT_WRITE,    /* safe-low precharge */
		PWRSEQ_ACT_DIGIPOT_VERIFY,   /* readback == safe */
		PWRSEQ_ACT_RB_PWR_EN,        /* + soft-start, verify safe rail */
		PWRSEQ_ACT_DIGIPOT_WRITE_OP, /* bounded operating setpoint */
		PWRSEQ_ACT_DIGIPOT_VERIFY,   /* readback == operating */
		PWRSEQ_ACT_RB_VCC_GATE_EN,   /* only after the op-window + vmax gate */
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
	 * The two digipot writes must carry their own codes: the safe/precharge
	 * write its safe code, the operating write the operating code. Drive
	 * both from non-default values — safe 0 (4.5 V), operating 25 (5 V per
	 * the vcc_rb_supply §5 table) — plus a non-default panel duty, so a
	 * hard-coded or swapped argument anywhere in the path fails here.
	 */
	pwrseq_cfg_default(&cfg);
	cfg.digipot_safe_code = 0U;
	cfg.digipot_operating_code = 25U; /* 5 V, well under the 15 V ceiling */
	cfg.panel_led_duty_pct = 40U;

	model_init(&m, &cfg);
	run_out(&m, 100U, 100U);

	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_TRUE(m.ctx.rb_gated);

	/* The first digipot write is the safe code, the second the operating
	 * code — order matters, so track which write we are looking at. */
	{
		int writes = 0;

		for (i = 0; i < (int)m.log_len; i++) {
			switch (m.log[i].action) {
			case PWRSEQ_ACT_DIGIPOT_WRITE:
				TEST_ASSERT_EQUAL_UINT16(0U, m.log[i].arg);
				writes++;
				break;
			case PWRSEQ_ACT_DIGIPOT_WRITE_OP:
				TEST_ASSERT_EQUAL_UINT16(25U, m.log[i].arg);
				writes++;
				break;
			case PWRSEQ_ACT_PANEL_LED_PWM:
				TEST_ASSERT_EQUAL_UINT16(40U, m.log[i].arg);
				break;
			default:
				TEST_ASSERT_EQUAL_UINT16(0U, m.log[i].arg);
				break;
			}
		}
		TEST_ASSERT_EQUAL_INT(2, writes);
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
	m.bad_readback = true; /* SPI dead: readback never valid */
	run_out(&m, 100U, 200U);

	/*
	 * The safe-code write is attempted, its verify retried three times, and
	 * then the whole stage abandoned — RB_PWR_EN is never even reached,
	 * because the readback verify sits before it. Nothing energises on an
	 * unproven wiper.
	 */
	TEST_ASSERT_EQUAL_UINT(1U, count_of(&m, PWRSEQ_ACT_DIGIPOT_WRITE));
	TEST_ASSERT_EQUAL_UINT(3U, count_of(&m, PWRSEQ_ACT_DIGIPOT_VERIFY));
	expect_absent(&m, PWRSEQ_ACT_DIGIPOT_WRITE_OP);
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

	/*
	 * An SPI glitch leaves the wiper register reading a value that matches
	 * neither commanded code. The safe-code readback verify catches it
	 * before RB_PWR_EN.
	 */
	model_init(&m, NULL);
	m.wrong_readback = true;
	run_out(&m, 100U, 200U);

	expect_absent(&m, PWRSEQ_ACT_RB_PWR_EN);
	TEST_ASSERT_TRUE(pwrseq_rb_fault(&m.ctx));
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_DIGIPOT)) != 0U);
}

static void test_rb_never_gates_before_the_rail_window_is_proven(void)
{
	model_t m;

	model_init(&m, NULL);
	/* Once the operating code is commanded (~14.25 V), the buck instead
	 * regulates to 12 V — out of window. The precharge safe-low rail is
	 * fine, so the sequence powers up and reaches the operating check. */
	m.force_op_rail_mv = 12000;
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

	/*
	 * Operating expected is 14 250 mV (code 500, VCTRL 1535 mV); the ±5 %
	 * window (tol 713) is [13537, 14963]. The measured<=vmax gate (15 000)
	 * does not bite inside this band, so these boundaries isolate the window
	 * check itself.
	 */
	model_init(&m, NULL);
	m.force_op_rail_mv = 14964; /* one past the top edge */
	run_out(&m, 100U, 200U);
	expect_absent(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);

	model_init(&m, NULL);
	m.force_op_rail_mv = 14963; /* exactly the top edge */
	run_out(&m, 100U, 200U);
	expect_present(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);
	TEST_ASSERT_FALSE(pwrseq_rb_fault(&m.ctx));

	model_init(&m, NULL);
	m.force_op_rail_mv = 13537; /* exactly the bottom edge */
	run_out(&m, 100U, 200U);
	expect_present(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);

	model_init(&m, NULL);
	m.force_op_rail_mv = 13536; /* one below */
	run_out(&m, 100U, 200U);
	expect_absent(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);
}

static void test_rb_measured_over_vmax_is_refused_even_inside_the_window(void)
{
	pwrseq_cfg_t cfg;
	model_t m;

	/*
	 * BLOCKER-1's defence in depth: a rail that sits INSIDE the code's own
	 * tolerance window but ABOVE the FE ceiling must still be refused. This
	 * is the case the self-referential check missed — if the window were the
	 * only gate, a rail matching its (bounded but high) setpoint would pass
	 * and connect an over-voltage to the FE.
	 *
	 * Operating code 520 -> ~14 638 mV expected; ±5 % window is roughly
	 * [13906, 15370]. A measured 15 100 mV is inside that window yet above
	 * the 15 000 mV ceiling, so only the independent measured<=vmax gate can
	 * catch it.
	 */
	pwrseq_cfg_default(&cfg);
	cfg.digipot_operating_code = 520U;
	TEST_ASSERT_EQUAL_INT(0, pwrseq_init(&m.ctx, &cfg)); /* expected(520) < vmax */

	/* Sanity: 15 100 really is inside the code's own tolerance window, so
	 * the window check alone would have let it through. */
	{
		int32_t lo = 0;
		int32_t hi = 0;

		TEST_ASSERT_EQUAL_INT(0,
				      pwrseq_rb_window(&cfg.rb_xfer, 520U,
						       cfg.rb_vbus_tol_pct, &lo,
						       &hi));
		TEST_ASSERT_TRUE((15100 >= lo) && (15100 <= hi));
		TEST_ASSERT_TRUE(15100 > (int32_t)cfg.rb_vmax_mv);
	}

	model_init(&m, &cfg);
	m.force_op_rail_mv = 15100; /* in window, over vmax */
	run_out(&m, 100U, 200U);

	expect_present(&m, PWRSEQ_ACT_RB_PWR_EN);
	expect_absent(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);
	expect_present(&m, PWRSEQ_ACT_RB_PWR_DIS);
	TEST_ASSERT_TRUE(pwrseq_rb_fault(&m.ctx));
	TEST_ASSERT_FALSE(m.ctx.rb_gated);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_TRUE(m.ctx.relay_eligible);
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
	m.hold_no_lock = true;
	run_out(&m, 60000U, 40U); /* 10 min lock timeout, in minute steps */

	/* It was powered and gated — the rail was fine, precharge and operating
	 * both verified — but never locked. */
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
	m.hold_no_extref = true;
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

static void test_a_gated_rail_going_unreadable_drops_the_rubidium(void)
{
	model_t m;
	size_t settled;

	/*
	 * A gated FE with a VCC_RB reading that has gone stale cannot be
	 * confirmed safe, so it must be dropped — "unknown" is not "in range".
	 * This also exercises the independent measured<=vmax gate's invalid-read
	 * path, which the window check would otherwise mask.
	 */
	model_init(&m, NULL);
	run_out(&m, 100U, 100U);
	TEST_ASSERT_TRUE(m.ctx.rb_gated);
	settled = m.log_len;

	m.rb_glue = false; /* stop the glue refreshing the reading */
	m.in.ina_valid[INA228_RAIL_VCC_RB] = false;
	advance(&m, 100U);

	TEST_ASSERT_FALSE(m.ctx.rb_enabled);
	TEST_ASSERT_FALSE(m.ctx.rb_gated);
	TEST_ASSERT_EQUAL_size_t(settled + 2U, m.log_len);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_RB_VCC_GATE_DIS,
			      (int)m.log[settled].action);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_RB_PWR_DIS,
			      (int)m.log[settled + 1U].action);
	TEST_ASSERT_TRUE(pwrseq_rb_fault(&m.ctx));
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

static void test_an_i2c_probe_that_never_answers_alarms(void)
{
	model_t m;

	model_init(&m, NULL);
	m.in.i2c_probe_ok = false;
	run_out(&m, 250U, 60U);

	TEST_ASSERT_EQUAL_UINT(3U, count_of(&m, PWRSEQ_ACT_I2C_PROBE));
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_I2C)) != 0U);
	/* Stage 2 is abandoned, so the display controller is left in reset. */
	expect_absent(&m, PWRSEQ_ACT_DISP_RST_RELEASE);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
}

static void test_shunt_trims_that_never_apply_stop_the_rail_checks(void)
{
	model_t m;

	/*
	 * ARCHITECTURE.md invariant 7 and interface ref §10 caution 7: an
	 * INA228 that has reverted to POR calibration reads about 1 % off and
	 * raises no flag of its own. If the trim cannot be written, no reading
	 * downstream of it may be believed — so the stage is abandoned rather
	 * than continuing on to judge rails with uncalibrated monitors.
	 */
	model_init(&m, NULL);
	m.in.shunt_trims_applied = false;
	run_out(&m, 100U, 60U);

	TEST_ASSERT_EQUAL_UINT(3U, count_of(&m, PWRSEQ_ACT_APPLY_SHUNT_TRIMS));
	expect_absent(&m, PWRSEQ_ACT_READ_RAIL_BASELINE);
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_SHUNT_TRIM)) != 0U);
	/* The rail checks were part of the abandoned stage, so no halt. */
	TEST_ASSERT_FALSE(m.ctx.halted);
}

static void test_a_missing_phy_reference_clock_holds_the_reset(void)
{
	model_t m;

	/* Interface ref §2 stage 4: release LAN_RST_N only "after rails stable
	 * and the 25 MHz PHY clock is up". A PHY reset released into a dead
	 * clock reads back an all-ones ID and never links. */
	model_init(&m, NULL);
	m.in.phy_refclk_stable = false;
	run_out(&m, 250U, 60U);

	expect_absent(&m, PWRSEQ_ACT_LAN_RST_RELEASE);
	expect_absent(&m, PWRSEQ_ACT_PHY_MDIO_POLL);
	TEST_ASSERT_FALSE(m.ctx.phy_released);
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_PHY)) != 0U);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
}

static void test_a_phy_that_will_not_identify_alarms(void)
{
	model_t m;

	model_init(&m, NULL);
	m.in.phy_id_ok = false;
	run_out(&m, 250U, 80U);

	expect_present(&m, PWRSEQ_ACT_LAN_RST_RELEASE);
	TEST_ASSERT_EQUAL_UINT(3U, count_of(&m, PWRSEQ_ACT_PHY_MDIO_POLL));
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_PHY)) != 0U);
	/* Networking is not a precondition for timing: GNSS still comes up. */
	expect_present(&m, PWRSEQ_ACT_GPS_PWR_EN);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
}

static void test_an_unreadable_antenna_current_drops_the_bias(void)
{
	model_t m;

	model_init(&m, NULL);
	run_to_stage(&m, PWRSEQ_STAGE_5_GNSS, 10U, 200U);
	/* The monitor answered at the stage-3 baseline and has since gone
	 * quiet — the antenna supervisor would be blind. */
	m.in.ina_valid[INA228_RAIL_V_ANT] = false;
	run_out(&m, 100U, 200U);

	expect_present(&m, PWRSEQ_ACT_ANT_BIAS_EN);
	expect_present(&m, PWRSEQ_ACT_ANT_BIAS_DIS);
	TEST_ASSERT_FALSE(m.ctx.ant_bias_on);
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_ANTENNA)) != 0U);
	/* Stage 5 is abandoned, so the receiver is never reconfigured. */
	expect_absent(&m, PWRSEQ_ACT_ANT_SUPERVISOR_START);
	expect_absent(&m, PWRSEQ_ACT_GNSS_CONFIG_REQUEST);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
}

static void test_a_display_rail_that_never_comes_up_is_dropped(void)
{
	model_t m;

	model_init(&m, NULL);
	/* Stop inside stage 5, where the 1 s F9T boot delay forces a break, so
	 * the reading can be spoiled before stage 6 ever looks at it — a single
	 * step() call would otherwise walk stage 6 to completion. */
	run_to_stage(&m, PWRSEQ_STAGE_5_GNSS, 10U, 200U);
	m.in.ina_valid[INA228_RAIL_5V_DISP] = false;
	run_out(&m, 100U, 200U);

	expect_present(&m, PWRSEQ_ACT_DISP_EN);
	expect_present(&m, PWRSEQ_ACT_DISP_DIS);
	TEST_ASSERT_FALSE(m.ctx.display_on);
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_DISPLAY)) != 0U);
	/* Spec §13: a display fault is UI-only and never blocks timing. */
	expect_present(&m, PWRSEQ_ACT_RELAY_ELIGIBLE);
	TEST_ASSERT_TRUE(m.ctx.relay_eligible);
	TEST_ASSERT_TRUE(m.ctx.rb_gated);
}

static void test_a_panel_rail_that_never_comes_up_is_dropped(void)
{
	model_t m;

	model_init(&m, NULL);
	run_to_stage(&m, PWRSEQ_STAGE_5_GNSS, 10U, 200U);
	m.in.ina_vbus_mv[INA228_RAIL_PANEL_5V] = 1000; /* nowhere near 5 V */
	run_out(&m, 100U, 200U);

	/* The display still comes up; only the LED string is dropped. */
	expect_present(&m, PWRSEQ_ACT_DISP_EN);
	TEST_ASSERT_TRUE(m.ctx.display_on);
	expect_present(&m, PWRSEQ_ACT_PANEL_LED_EN);
	expect_present(&m, PWRSEQ_ACT_PANEL_LED_DIS);
	expect_absent(&m, PWRSEQ_ACT_PANEL_LED_PWM);
	TEST_ASSERT_FALSE(m.ctx.panel_led_on);
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_PANEL_LED)) != 0U);
	TEST_ASSERT_TRUE(m.ctx.relay_eligible);
}

static void test_an_oven_that_never_warms_alarms_but_lets_the_board_run(void)
{
	model_t m;

	model_init(&m, NULL);
	m.in.ocxo_temp_stable = false;
	m.in.ina_current_ma[INA228_RAIL_OCXO] = 900; /* still heating */
	run_out(&m, 60000U, 30U);                    /* 10 min warm timeout */

	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_OCXO_WARM)) != 0U);
	/* Stage 7 is abandoned, so the discipline loop is not handed a cold
	 * oscillator... */
	expect_absent(&m, PWRSEQ_ACT_DISC_START);
	/* ...and stage 8's warm precondition defers the rubidium too. */
	expect_absent(&m, PWRSEQ_ACT_RB_PWR_EN);
	TEST_ASSERT_TRUE(m.ctx.rb_deferred);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
}

static void test_a_rail_excursion_between_gating_and_lock_aborts_stage_8(void)
{
	model_t m;

	/*
	 * The window is not a one-shot gate at the operating step. Holding at
	 * step 8.15 waiting for lock, a rail that wanders must still drop the
	 * FE — the supervisor runs every call, whatever stage the machine is in.
	 */
	model_init(&m, NULL);
	m.hold_no_lock = true;
	run_out(&m, 100U, 40U);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_8_RB, pwrseq_stage(&m.ctx));
	TEST_ASSERT_TRUE(m.ctx.rb_gated);

	/*
	 * The buck wanders to 20 V while gated to the FE. Freeze the glue and
	 * inject the reading directly so the very next step() sees the excursion
	 * (the glue applies its response only after each step).
	 */
	m.rb_glue = false;
	m.in.ina_vbus_mv[INA228_RAIL_VCC_RB] = 20000;
	advance(&m, 100U);

	TEST_ASSERT_FALSE(m.ctx.rb_enabled);
	TEST_ASSERT_FALSE(m.ctx.rb_gated);
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_WINDOW)) != 0U);
	TEST_ASSERT_TRUE(pwrseq_rb_fault(&m.ctx));

	/* The abort abandons stage 8 rather than sitting out the ten-minute
	 * lock timeout, so bring-up finishes promptly on the OCXO. */
	TEST_ASSERT_TRUE(pwrseq_stage(&m.ctx) >= PWRSEQ_STAGE_9_ARM);
	run_out(&m, 100U, 40U);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_TRUE(m.ctx.relay_eligible);
}

static void test_an_over_voltage_during_the_lock_wait_aborts_stage_8(void)
{
	model_t m;

	model_init(&m, NULL);
	m.hold_no_lock = true;
	run_out(&m, 100U, 40U);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_8_RB, pwrseq_stage(&m.ctx));

	m.in.rb_ov_det = true;
	advance(&m, 100U);

	TEST_ASSERT_TRUE(pwrseq_ov_latched(&m.ctx));
	TEST_ASSERT_FALSE(m.ctx.rb_enabled);
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_OV)) != 0U);
	TEST_ASSERT_TRUE(pwrseq_stage(&m.ctx) >= PWRSEQ_STAGE_9_ARM);

	/* The latch is still set, so a clear is refused until PE3 goes low. */
	TEST_ASSERT_EQUAL_INT(-EBUSY, pwrseq_ov_clear(&m.ctx, m.in.mono_ms));

	run_out(&m, 100U, 40U);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_TRUE(m.ctx.relay_eligible);
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

	/*
	 * That same step burst runs the stage-8 preconditions and the safe-code
	 * write/verify, so RB_PWR_EN is now asserted and blocked on its 500 ms
	 * soft-start. The *operating* code must not be commanded until the buck
	 * has settled at safe-low, so DIGIPOT_WRITE_OP is gated by the soft-start.
	 */
	expect_present(&m, PWRSEQ_ACT_RB_PWR_EN);
	expect_absent(&m, PWRSEQ_ACT_DIGIPOT_WRITE_OP);
	advance(&m, 480U); /* still inside the 500 ms soft-start */
	expect_absent(&m, PWRSEQ_ACT_DIGIPOT_WRITE_OP);
	advance_until(&m, PWRSEQ_ACT_DIGIPOT_WRITE_OP, 200U);

	/*
	 * And the FE connect is gated by the ramp: after the operating code is
	 * written and read back, the rail is given rb_ramp_ms (100 ms) to settle
	 * before the operating-window + vmax gate is judged and RB_VCC_GATE
	 * asserted.
	 */
	expect_absent(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);
	advance_until(&m, PWRSEQ_ACT_RB_VCC_GATE_EN, 400U);
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

	/*
	 * The whole two-code interlock ran again, in order: safe write + verify,
	 * power, operating write + verify, then the FE connect. RB_VCC_GATE is
	 * strictly last, and no RB_PWR_EN appears before the first digipot write.
	 */
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_DIGIPOT_WRITE, (int)m.log[base].action);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_DIGIPOT_VERIFY,
			      (int)m.log[base + 1U].action);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_RB_PWR_EN, (int)m.log[base + 2U].action);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_DIGIPOT_WRITE_OP,
			      (int)m.log[base + 3U].action);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_DIGIPOT_VERIFY,
			      (int)m.log[base + 4U].action);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_ACT_RB_VCC_GATE_EN,
			      (int)m.log[base + 5U].action);
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

static void test_pfi_shuts_down_the_rubidium_unconditionally(void)
{
	static const uint8_t want[] = {
		PWRSEQ_ACT_PARK_DAC,
		PWRSEQ_ACT_PERSIST_STATE,
		PWRSEQ_ACT_RB_VCC_GATE_DIS,
		PWRSEQ_ACT_RB_PWR_DIS,
		PWRSEQ_ACT_SET_SHUTDOWN_FLAG,
	};
	model_t m;
	size_t i;

	/*
	 * MEDIUM-3: the park list drops the rubidium unconditionally, not gated
	 * on the sequencer's belief that it is running. Even with the rubidium
	 * never wanted — so RB_PWR_EN was never asserted — the RB_VCC_GATE_DIS /
	 * RB_PWR_DIS pin writes are still emitted, because the supervisor may
	 * have queued (and the purge may have deleted) an earlier shutdown, and
	 * the writes are idempotent at the pin. Better a redundant safe-off than
	 * a live FE through a brown-out.
	 */
	model_init(&m, NULL);
	m.in.rb_wanted = false;
	run_out(&m, 100U, 100U);
	TEST_ASSERT_FALSE(m.ctx.rb_enabled);

	TEST_ASSERT_EQUAL_INT(0, pwrseq_pfi(&m.ctx, 9000U));
	TEST_ASSERT_EQUAL_size_t(ARRAY_LEN(want), pwrseq_action_count(&m.ctx));
	for (i = 0U; i < ARRAY_LEN(want); i++) {
		pwrseq_act_t a;

		TEST_ASSERT_EQUAL_INT(0, pwrseq_action_get(&m.ctx, &a));
		TEST_ASSERT_EQUAL_INT_MESSAGE(
			(int)want[i], (int)a.action,
			pwrseq_action_name((pwrseq_action_t)want[i]));
	}
}

static void test_pfi_shutdown_survives_an_undrained_supervisor_shutdown(void)
{
	model_t m;
	pwrseq_act_t a;
	int gate_dis = 0;
	int pwr_dis = 0;

	/*
	 * The exact MEDIUM-3 race: the supervisor drops the rubidium (queuing
	 * RB_VCC_GATE_DIS / RB_PWR_DIS and clearing rb_enabled), the glue has
	 * NOT drained them, and then PFI fires and purges the queue. A guard on
	 * rb_enabled would now emit nothing. Verify both pin writes still reach
	 * the queue after the purge.
	 */
	model_init(&m, NULL);
	run_out(&m, 100U, 100U);
	TEST_ASSERT_TRUE(m.ctx.rb_enabled);

	/* Provoke a supervisor shutdown but do NOT drain it. */
	m.rb_glue = false;      /* stop the harness draining/responding */
	m.in.rb_ov_det = true;  /* OV latch → supervisor rb_shutdown */
	m.in.mono_ms += 100U;
	TEST_ASSERT_EQUAL_INT(0, pwrseq_step(&m.ctx, &m.in));
	TEST_ASSERT_FALSE(m.ctx.rb_enabled); /* cleared at emit time */
	TEST_ASSERT_TRUE(pwrseq_action_count(&m.ctx) >= 2U);

	/* PFI now — the purge deletes the un-drained shutdown. */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_pfi(&m.ctx, m.in.mono_ms + 1U));

	while (pwrseq_action_get(&m.ctx, &a) == 0) {
		if (a.action == PWRSEQ_ACT_RB_VCC_GATE_DIS) {
			gate_dis++;
		} else if (a.action == PWRSEQ_ACT_RB_PWR_DIS) {
			pwr_dis++;
		}
	}
	TEST_ASSERT_EQUAL_INT(1, gate_dis);
	TEST_ASSERT_EQUAL_INT(1, pwr_dis);
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
	 * Run the whole sequence without ever draining and without the glue
	 * responding, so the input never advances the two-code readback and the
	 * operating verify fails into RB_OFF — a deliberately messy run with
	 * retries and a shutdown. The property under test is not a tidy action
	 * count but that the step machine reserves queue room before every step:
	 * it never drops an action, and it fits inside the queue.
	 */
	for (i = 0U; i < 100U; i++) {
		m.in.mono_ms += 100U;
		TEST_ASSERT_EQUAL_INT(0, pwrseq_step(&m.ctx, &m.in));
	}

	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_EQUAL_UINT32(0U, pwrseq_actions_dropped(&m.ctx));
	TEST_ASSERT_TRUE(pwrseq_action_count(&m.ctx) <=
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

/* ---------------------------------------- BLOCKER-1 / HIGH-1 / HIGH-2 / L1 */

static void test_init_bounds_the_digipot_codes_by_voltage(void)
{
	pwrseq_cfg_t cfg;
	pwrseq_ctx_t ctx;

	/*
	 * BLOCKER-1: a digipot code whose commanded rail would exceed the FE
	 * ceiling is rejected at boot, so the config never runs — the fix's
	 * front line. Code 1000 is ~24 V against the 15 V default ceiling.
	 */
	pwrseq_cfg_default(&cfg);
	cfg.digipot_operating_code = 1000U;
	TEST_ASSERT_TRUE(pwrseq_rb_expected_mv(&cfg.rb_xfer, 1000U) >
			 (int32_t)cfg.rb_vmax_mv);
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_init(&ctx, &cfg));

	/* A "safe" code that is not actually low is a config error too. */
	pwrseq_cfg_default(&cfg);
	cfg.digipot_safe_code = 900U; /* ~22 V */
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_init(&ctx, &cfg));

	/* Out-of-range codes (>= steps) are rejected outright. */
	pwrseq_cfg_default(&cfg);
	cfg.digipot_operating_code = 2000U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_init(&ctx, &cfg));
	pwrseq_cfg_default(&cfg);
	cfg.digipot_safe_code = 1024U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_init(&ctx, &cfg));

	/* A zero ceiling can never be satisfied. */
	pwrseq_cfg_default(&cfg);
	cfg.rb_vmax_mv = 0U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_init(&ctx, &cfg));

	/* Raising the ceiling admits the documented 15 V nominal (code 539). */
	pwrseq_cfg_default(&cfg);
	cfg.digipot_operating_code = 539U;
	cfg.rb_vmax_mv = 15500U;
	TEST_ASSERT_EQUAL_INT(0, pwrseq_init(&ctx, &cfg));

	/* L4: nonsense tolerances are rejected. */
	pwrseq_cfg_default(&cfg);
	cfg.rail_tol_pct = 101U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_init(&ctx, &cfg));
	pwrseq_cfg_default(&cfg);
	cfg.rb_vbus_tol_pct = 101U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, pwrseq_init(&ctx, &cfg));
}

static void test_withdrawing_rb_wanted_mid_stage_8_shuts_down(void)
{
	model_t m;
	size_t base;

	/*
	 * HIGH-1: rb_wanted going false after RB_PWR_EN used to guard-skip the
	 * remaining stage-8 rows and leave the buck enabled forever, because the
	 * old window branch required rb_gated. Hold gated at the lock wait, then
	 * withdraw intent: the supervisor drops both enables even though nothing
	 * is faulted.
	 */
	model_init(&m, NULL);
	m.hold_no_lock = true;
	run_out(&m, 100U, 40U);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_8_RB, pwrseq_stage(&m.ctx));
	TEST_ASSERT_TRUE(m.ctx.rb_enabled);
	TEST_ASSERT_TRUE(m.ctx.rb_gated);
	base = m.log_len;

	m.in.rb_wanted = false;
	advance(&m, 100U);

	TEST_ASSERT_FALSE(m.ctx.rb_enabled);
	TEST_ASSERT_FALSE(m.ctx.rb_gated);
	/* Deliberate withdrawal, so NOT a fault. */
	TEST_ASSERT_FALSE(pwrseq_rb_fault(&m.ctx));
	TEST_ASSERT_EQUAL_UINT32(0U,
				 pwrseq_alarms(&m.ctx) & PWRSEQ_ALARM_RB_MASK);
	TEST_ASSERT_TRUE(idx_of_from(&m, PWRSEQ_ACT_RB_VCC_GATE_DIS, base) >= 0);
	TEST_ASSERT_TRUE(idx_of_from(&m, PWRSEQ_ACT_RB_PWR_DIS, base) >= 0);

	/* Bring-up still completes on the OCXO. */
	run_out(&m, 100U, 40U);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_DONE, pwrseq_stage(&m.ctx));
	TEST_ASSERT_TRUE(m.ctx.relay_eligible);
}

static void test_withdrawing_rb_wanted_before_the_gate_shuts_down(void)
{
	model_t m;

	/*
	 * The narrow HIGH-1 window that was the actual hole: intent withdrawn
	 * while powered but NOT yet gated. Catch it during the 500 ms
	 * soft-start, where the step machine is merely waiting — the old code
	 * would skip the rest of the stage and strand RB_PWR_EN high.
	 */
	model_init(&m, NULL);
	advance_until(&m, PWRSEQ_ACT_RB_PWR_EN, 5000U);
	TEST_ASSERT_TRUE(m.ctx.rb_enabled);
	TEST_ASSERT_FALSE(m.ctx.rb_gated);

	m.in.rb_wanted = false;
	advance(&m, 10U); /* still inside soft-start */

	TEST_ASSERT_FALSE(m.ctx.rb_enabled);
	expect_absent(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);
	TEST_ASSERT_FALSE(pwrseq_rb_fault(&m.ctx));
}

static void test_halted_board_refuses_rb_retry_and_shed_restore(void)
{
	model_t m;

	/*
	 * HIGH-2: a board halted on a hard fault (a 3V3 rail-verify failure)
	 * must not be resurrected by a routine rubidium retry or a
	 * thermal/PoE shed-restore — both funnel through restart_stage, which
	 * clears the halt and jumps into stage 8 with RB_PWR_EN and
	 * RB_VCC_GATE. Only an explicit pwrseq_restart_stage() may clear a halt.
	 */
	model_init(&m, NULL);
	m.in.ina_vbus_mv[INA228_RAIL_3V3_MAIN] = 2000; /* rail verify fails */
	run_out(&m, 250U, 40U);
	TEST_ASSERT_TRUE(m.ctx.halted);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_3_RAILS, pwrseq_stage(&m.ctx));

	TEST_ASSERT_EQUAL_INT(-EPERM, pwrseq_rb_retry(&m.ctx, 5000U));
	TEST_ASSERT_EQUAL_INT(-EPERM, pwrseq_shed_restore(&m.ctx, 5000U));

	/* Still halted, still at stage 3, nothing energised, relay down. */
	TEST_ASSERT_TRUE(m.ctx.halted);
	TEST_ASSERT_EQUAL_INT(PWRSEQ_STAGE_3_RAILS, pwrseq_stage(&m.ctx));
	expect_absent(&m, PWRSEQ_ACT_RB_PWR_EN);
	expect_absent(&m, PWRSEQ_ACT_RB_VCC_GATE_EN);
	TEST_ASSERT_FALSE(m.ctx.relay_eligible);

	/* The explicit recovery path is the only thing that clears the halt. */
	m.in.ina_vbus_mv[INA228_RAIL_3V3_MAIN] = 3300;
	TEST_ASSERT_EQUAL_INT(
		0, pwrseq_restart_stage(&m.ctx, PWRSEQ_STAGE_3_RAILS, 6000U));
	TEST_ASSERT_FALSE(m.ctx.halted);
}

static void test_ov_clear_also_clears_the_rb_fault_umbrella(void)
{
	model_t m;

	/*
	 * L1: an over-voltage sets both RB_OV and the RB_FAULT umbrella. Once
	 * the operator clears the latch with the cause gone, pwrseq_rb_fault()
	 * must return false — otherwise a cleared, acknowledged fault leaves the
	 * board reading permanently Rb-faulted.
	 */
	model_init(&m, NULL);
	run_out(&m, 100U, 100U);
	TEST_ASSERT_TRUE(m.ctx.rb_enabled);

	m.in.rb_ov_det = true;
	advance(&m, 100U);
	TEST_ASSERT_TRUE(pwrseq_rb_fault(&m.ctx));
	TEST_ASSERT_TRUE((pwrseq_alarms(&m.ctx) &
			  PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_OV)) != 0U);

	m.in.rb_ov_det = false;
	advance(&m, 10U); /* the latch input reads low again */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_ov_clear(&m.ctx, m.in.mono_ms));

	TEST_ASSERT_FALSE(pwrseq_ov_latched(&m.ctx));
	TEST_ASSERT_EQUAL_UINT32(
		0U, pwrseq_alarms(&m.ctx) & PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_OV));
	TEST_ASSERT_FALSE(pwrseq_rb_fault(&m.ctx));
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_init_defaults_and_validation);
	RUN_TEST(test_init_bounds_the_digipot_codes_by_voltage);
	RUN_TEST(test_wdt_cadence_outside_the_window_is_rejected);

	RUN_TEST(test_digipot_transfer_matches_the_documented_table);
	RUN_TEST(test_rb_acceptance_window);
	RUN_TEST(test_rb_window_survives_a_degenerate_transfer_function);

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
	RUN_TEST(test_rb_measured_over_vmax_is_refused_even_inside_the_window);
	RUN_TEST(test_a_stale_vcc_rb_reading_is_not_a_pass);
	RUN_TEST(test_lock_timeout_drops_the_rubidium);
	RUN_TEST(test_an_out_of_band_external_reference_is_not_a_lock);
	RUN_TEST(test_a_rail_excursion_after_gating_drops_the_rubidium);
	RUN_TEST(test_a_gated_rail_going_unreadable_drops_the_rubidium);
	RUN_TEST(test_an_over_voltage_latch_drops_the_rubidium_at_any_stage);
	RUN_TEST(test_withdrawing_rb_wanted_mid_stage_8_shuts_down);
	RUN_TEST(test_withdrawing_rb_wanted_before_the_gate_shuts_down);

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
	RUN_TEST(test_an_i2c_probe_that_never_answers_alarms);
	RUN_TEST(test_shunt_trims_that_never_apply_stop_the_rail_checks);
	RUN_TEST(test_a_missing_phy_reference_clock_holds_the_reset);
	RUN_TEST(test_a_phy_that_will_not_identify_alarms);
	RUN_TEST(test_an_unreadable_antenna_current_drops_the_bias);
	RUN_TEST(test_a_display_rail_that_never_comes_up_is_dropped);
	RUN_TEST(test_a_panel_rail_that_never_comes_up_is_dropped);
	RUN_TEST(test_an_oven_that_never_warms_alarms_but_lets_the_board_run);
	RUN_TEST(test_a_rail_excursion_between_gating_and_lock_aborts_stage_8);
	RUN_TEST(test_an_over_voltage_during_the_lock_wait_aborts_stage_8);
	RUN_TEST(test_settle_delays_are_observed);

	RUN_TEST(test_shed_ladder_order_and_exhaustion);
	RUN_TEST(test_restoring_the_rubidium_re_runs_the_guarded_sequence);
	RUN_TEST(test_a_shed_load_is_not_re_enabled_by_a_stage_replay);
	RUN_TEST(test_halted_board_refuses_rb_retry_and_shed_restore);

	RUN_TEST(test_ov_clear_is_guarded);
	RUN_TEST(test_ov_clear_also_clears_the_rb_fault_umbrella);

	RUN_TEST(test_pfi_emits_the_park_list_in_priority_order);
	RUN_TEST(test_pfi_shuts_down_the_rubidium_unconditionally);
	RUN_TEST(test_pfi_shutdown_survives_an_undrained_supervisor_shutdown);
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
