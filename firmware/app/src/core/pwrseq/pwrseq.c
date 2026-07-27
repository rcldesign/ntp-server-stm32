/*
 * STS1000 "Meridian" — core/pwrseq: bring-up stage machine and power policy.
 *
 * See pwrseq.h for the contract. This file holds the step table, which is the
 * whole point of the module: the ordering in interface ref §2 exists as data,
 * one row per numbered step, rather than as control flow that could drift from
 * the document.
 *
 * A row is {stage, entry action, guard, settle delay, exit predicate, timeout,
 * failure policy}. `pwrseq_step()` walks it: emit the entry action once, wait
 * out any settle delay, poll the exit predicate against the caller's snapshot,
 * and on timeout apply the row's policy. Guards skip a row entirely — that is
 * how the display stages disappear on a headless unit and how the whole
 * rubidium stage disappears when the PoE budget cannot cover it.
 *
 * Two things sit outside the walk because they must not wait for it:
 *
 *   - Rubidium supervision. Once RB_PWR_EN is asserted, an over-voltage latch
 *     or a rail that leaves its window drops the load *before* the step machine
 *     gets a turn, at any stage, including long after bring-up finished.
 *   - Power fail. pwrseq_pfi() discards whatever the sequencer had queued and
 *     replaces it with the park list; with ~4.8 ms of hold-up left, nothing the
 *     sequencer wanted matters any more.
 */

#include "pwrseq/pwrseq.h"

#include <errno.h>
#include <string.h>

_Static_assert(PWRSEQ_ACT_QUEUE_LEN >= (2 * PWRSEQ_ACT_PER_STEP_MAX),
	       "action queue must hold more than one step's worth");

/* -------------------------------------------------------------------- names */

static const char *const action_names[PWRSEQ_ACT_COUNT] = {
	[PWRSEQ_ACT_NONE] = "none",
	[PWRSEQ_ACT_NOR_RST_RELEASE] = "nor-rst-release",
	[PWRSEQ_ACT_I2C_PROBE] = "i2c-probe",
	[PWRSEQ_ACT_DISP_RST_RELEASE] = "disp-rst-release",
	[PWRSEQ_ACT_APPLY_SHUNT_TRIMS] = "apply-shunt-trims",
	[PWRSEQ_ACT_READ_RAIL_BASELINE] = "read-rail-baseline",
	[PWRSEQ_ACT_LAN_RST_RELEASE] = "lan-rst-release",
	[PWRSEQ_ACT_PHY_MDIO_POLL] = "phy-mdio-poll",
	[PWRSEQ_ACT_GPS_PWR_EN] = "gps-pwr-en",
	[PWRSEQ_ACT_GPS_PWR_DIS] = "gps-pwr-dis",
	[PWRSEQ_ACT_GPS_RST_RELEASE] = "gps-rst-release",
	[PWRSEQ_ACT_ANT_BIAS_EN] = "ant-bias-en",
	[PWRSEQ_ACT_ANT_BIAS_DIS] = "ant-bias-dis",
	[PWRSEQ_ACT_ANT_SUPERVISOR_START] = "ant-supervisor-start",
	[PWRSEQ_ACT_GNSS_CONFIG_REQUEST] = "gnss-config-request",
	[PWRSEQ_ACT_DISP_EN] = "disp-en",
	[PWRSEQ_ACT_DISP_DIS] = "disp-dis",
	[PWRSEQ_ACT_PANEL_LED_EN] = "panel-led-en",
	[PWRSEQ_ACT_PANEL_LED_DIS] = "panel-led-dis",
	[PWRSEQ_ACT_PANEL_LED_PWM] = "panel-led-pwm",
	[PWRSEQ_ACT_DISC_START] = "disc-start",
	[PWRSEQ_ACT_DIGIPOT_WRITE] = "digipot-write",
	[PWRSEQ_ACT_DIGIPOT_WRITE_OP] = "digipot-write-op",
	[PWRSEQ_ACT_DIGIPOT_VERIFY] = "digipot-verify",
	[PWRSEQ_ACT_RB_PWR_EN] = "rb-pwr-en",
	[PWRSEQ_ACT_RB_PWR_DIS] = "rb-pwr-dis",
	[PWRSEQ_ACT_RB_VCC_GATE_EN] = "rb-vcc-gate-en",
	[PWRSEQ_ACT_RB_VCC_GATE_DIS] = "rb-vcc-gate-dis",
	[PWRSEQ_ACT_RB_OV_RESET_PULSE] = "rb-ov-reset-pulse",
	[PWRSEQ_ACT_WDT_EN] = "wdt-en",
	[PWRSEQ_ACT_WDT_KICK_START] = "wdt-kick-start",
	[PWRSEQ_ACT_RELAY_ELIGIBLE] = "relay-eligible",
	[PWRSEQ_ACT_PARK_DAC] = "park-dac",
	[PWRSEQ_ACT_PERSIST_STATE] = "persist-state",
	[PWRSEQ_ACT_SET_SHUTDOWN_FLAG] = "set-shutdown-flag",
	[PWRSEQ_ACT_POE_KILL] = "poe-kill",
};

const char *pwrseq_action_name(pwrseq_action_t a)
{
	if (((unsigned int)a >= (unsigned int)PWRSEQ_ACT_COUNT) ||
	    (action_names[(unsigned int)a] == NULL)) {
		return "invalid";
	}
	return action_names[(unsigned int)a];
}

static const char *const alarm_names[PWRSEQ_ALARM_COUNT] = {
	[PWRSEQ_ALARM_NONE] = "none",
	[PWRSEQ_ALARM_NOR] = "nor",
	[PWRSEQ_ALARM_I2C] = "i2c",
	[PWRSEQ_ALARM_SHUNT_TRIM] = "shunt-trim",
	[PWRSEQ_ALARM_RAIL_BASELINE] = "rail-baseline",
	[PWRSEQ_ALARM_RAIL_3V3] = "rail-3v3",
	[PWRSEQ_ALARM_RAIL_3V3_STM] = "rail-3v3-stm",
	[PWRSEQ_ALARM_RAIL_POE] = "rail-poe",
	[PWRSEQ_ALARM_PHY] = "phy",
	[PWRSEQ_ALARM_GPS_RAIL] = "gps-rail",
	[PWRSEQ_ALARM_ANTENNA] = "antenna",
	[PWRSEQ_ALARM_GNSS_CFG] = "gnss-cfg",
	[PWRSEQ_ALARM_DISPLAY] = "display",
	[PWRSEQ_ALARM_PANEL_LED] = "panel-led",
	[PWRSEQ_ALARM_OCXO_WARM] = "ocxo-warm",
	[PWRSEQ_ALARM_POE_BUDGET] = "poe-budget",
	[PWRSEQ_ALARM_RB_PRECONDITION] = "rb-precondition",
	[PWRSEQ_ALARM_RB_DIGIPOT] = "rb-digipot",
	[PWRSEQ_ALARM_RB_WINDOW] = "rb-window",
	[PWRSEQ_ALARM_RB_LOCK] = "rb-lock",
	[PWRSEQ_ALARM_RB_OV] = "rb-ov",
	[PWRSEQ_ALARM_RB_FAULT] = "rb-fault",
	[PWRSEQ_ALARM_LIVENESS] = "liveness",
};

const char *pwrseq_alarm_name(pwrseq_alarm_t a)
{
	if (((unsigned int)a >= (unsigned int)PWRSEQ_ALARM_COUNT) ||
	    (alarm_names[(unsigned int)a] == NULL)) {
		return "invalid";
	}
	return alarm_names[(unsigned int)a];
}

/* The rubidium failures that constitute a real fault, as opposed to a
 * deferral. Raising any of them also raises the umbrella. */
#define RB_HARD_FAULT_MASK                                                     \
	(PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_DIGIPOT) |                           \
	 PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_WINDOW) |                            \
	 PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_LOCK) |                              \
	 PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_OV))

/* ------------------------------------------------------------------ config */

void pwrseq_cfg_default(pwrseq_cfg_t *cfg)
{
	if (cfg == NULL) {
		return;
	}

	cfg->nor_timeout_ms = 100U;
	cfg->i2c_timeout_ms = 500U;
	cfg->trim_timeout_ms = 200U;
	cfg->rail_timeout_ms = 500U;
	cfg->phy_clk_timeout_ms = 1000U;
	cfg->phy_settle_ms = 50U;
	cfg->phy_id_timeout_ms = 1000U;
	cfg->gps_rail_timeout_ms = 500U;
	cfg->gps_boot_ms = 1000U;
	cfg->ant_timeout_ms = 500U;
	cfg->gnss_cfg_timeout_ms = 5000U;
	cfg->disp_timeout_ms = 500U;
	cfg->ocxo_warm_timeout_ms = 600000U;      /* 10 min */
	cfg->rb_precondition_timeout_ms = 30000U; /* 30 s */
	cfg->rb_softstart_ms = 500U;
	cfg->rb_window_timeout_ms = 200U;
	cfg->rb_lock_timeout_ms = 600000U; /* 10 min */
	cfg->liveness_timeout_ms = 30000U;

	cfg->rail_tol_pct = 10U;
	cfg->poe_min_mv = 42500; /* IEEE 802.3bt Type-3 PD operating floor */

	cfg->ocxo_warm_current_ma = 350U;
	cfg->ocxo_warmup_current_ma = 800U;

	cfg->digipot_safe_code = 0U; /* terminal B = VREF = minimum VOUT */
	/*
	 * ~14.25 V at the default transfer function (code 500). Deliberately
	 * below the 15 000 mV ceiling by more than rb_vbus_tol_pct so the rail
	 * reliably passes the operating-window + measured<=vmax gate; the
	 * documented 15 V nominal (code 539) sits at 15 007 mV, one hair over
	 * the default ceiling, so it must not be the default. Set per FE unit.
	 */
	cfg->digipot_operating_code = 500U;
	cfg->rb_vmax_mv = 15000U; /* cfg key PWR_RB_VMAX_MV default */
	cfg->rb_vbus_tol_pct = 5U;
	cfg->rb_ramp_ms = 100U;
	cfg->rb_cold_start_mw = 16000U;

	cfg->rb_xfer.vref_mv = 3000U;
	cfg->rb_xfer.steps = 1024U;
	cfg->rb_xfer.pedestal_mv = 24450U;
	cfg->rb_xfer.gain_m1000 = 6645U;

	cfg->panel_led_duty_pct = 60U;

	cfg->wdt_kick_period_ms = PWRSEQ_WDT_KICK_PERIOD_MS;
}

/* -------------------------------------------------------------- arithmetic */

/* round(v * mul / div), half away from zero. div must be positive. */
static int64_t mul_div_round(int64_t v, int64_t mul, int64_t div)
{
	int64_t q = v / div;
	int64_t r = v % div;
	int64_t frac = r * mul;
	int64_t half = div / 2;

	if (frac >= 0) {
		frac = (frac + half) / div;
	} else {
		frac = (frac - half) / div;
	}
	return (q * mul) + frac;
}

uint32_t pwrseq_digipot_vctrl_mv(const pwrseq_rb_xfer_t *x, uint16_t code)
{
	if ((x == NULL) || (x->steps == 0U)) {
		return 0U;
	}

	/*
	 * VOUT rises with the code (code 0 = safe-low ~4.5 V, code steps-1 =
	 * pedestal ~24.4 V), so an out-of-range code must clamp to the
	 * *safe-low* end, never to steps-1. Clamping to steps-1 — the previous
	 * behaviour — turned any garbage code into MAXIMUM output, exactly the
	 * failure that destroys a 15 V-class FE (BLOCKER-1). A code at or above
	 * `steps` therefore returns the code-0 VCTRL (= vref, maximum VCTRL,
	 * minimum VOUT).
	 */
	if ((uint32_t)code >= (uint32_t)x->steps) {
		return x->vref_mv;
	}

	return (uint32_t)mul_div_round((int64_t)x->steps - (int64_t)code,
				       (int64_t)x->vref_mv, (int64_t)x->steps);
}

int32_t pwrseq_rb_expected_mv(const pwrseq_rb_xfer_t *x, uint16_t code)
{
	int64_t vctrl;
	int64_t drop;

	if (x == NULL) {
		return 0;
	}

	vctrl = (int64_t)pwrseq_digipot_vctrl_mv(x, code);
	drop = mul_div_round(vctrl, (int64_t)x->gain_m1000, 1000);

	return (int32_t)((int64_t)x->pedestal_mv - drop);
}

int pwrseq_rb_window(const pwrseq_rb_xfer_t *x, uint16_t code, uint32_t tol_pct,
		     int32_t *lo, int32_t *hi)
{
	int32_t expected;
	int32_t tol;

	if ((x == NULL) || (lo == NULL) || (hi == NULL)) {
		return -EINVAL;
	}

	expected = pwrseq_rb_expected_mv(x, code);
	tol = (int32_t)mul_div_round((int64_t)expected, (int64_t)tol_pct, 100);
	if (tol < 0) {
		tol = -tol;
	}

	*lo = expected - tol;
	*hi = expected + tol;
	return 0;
}

uint32_t pwrseq_poe_headroom_mw(const pwrseq_in_t *in)
{
	if ((in == NULL) || (in->poe_measured_mw >= in->poe_granted_mw)) {
		return 0U;
	}
	return in->poe_granted_mw - in->poe_measured_mw;
}

/* --------------------------------------------------------- shared predicates */

/* Is @p rail's bus voltage inside the configured band around its nominal? */
static bool rail_in_band(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in,
			 ina228_rail_t rail)
{
	const ina228_rail_info_t *info = ina228_rail(rail);
	int32_t tol;
	int32_t delta;

	if ((info == NULL) || !in->ina_valid[rail]) {
		return false;
	}

	tol = (int32_t)mul_div_round((int64_t)info->nominal_mv,
				     (int64_t)ctx->cfg.rail_tol_pct, 100);
	delta = in->ina_vbus_mv[rail] - info->nominal_mv;
	if (delta < 0) {
		delta = -delta;
	}
	return delta <= tol;
}

bool pwrseq_ocxo_is_warm(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	if ((ctx == NULL) || (in == NULL)) {
		return false;
	}

	/* The discipline loop can declare it directly once it has characterised
	 * the oven; otherwise infer it from the settled draw plus a stable
	 * oscillator temperature (interface ref §7). */
	if (in->ocxo_warm) {
		return true;
	}
	if (!in->ocxo_temp_stable || !in->ina_valid[INA228_RAIL_OCXO]) {
		return false;
	}
	if (in->ina_current_ma[INA228_RAIL_OCXO] < 0) {
		return false;
	}
	return (uint32_t)in->ina_current_ma[INA228_RAIL_OCXO] <=
	       ctx->cfg.ocxo_warm_current_ma;
}

bool pwrseq_ocxo_is_warming(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	if ((ctx == NULL) || (in == NULL) ||
	    !in->ina_valid[INA228_RAIL_OCXO] ||
	    (in->ina_current_ma[INA228_RAIL_OCXO] < 0)) {
		return false;
	}
	return (uint32_t)in->ina_current_ma[INA228_RAIL_OCXO] >=
	       ctx->cfg.ocxo_warmup_current_ma;
}

/* True when VCC_RB reads inside the window implied by the *currently commanded*
 * code — safe-low during precharge, the operating setpoint after it is written.
 * Deriving from rb_current_code rather than always from the safe code is what
 * lets the same check serve both the precharge-verify step and the post-gate
 * supervisor. It proves the buck is regulating to whatever was commanded; it is
 * NOT on its own a guarantee the rail is FE-safe, which is why the operating
 * step and the supervisor also apply the code-independent measured<=vmax gate. */
static bool rb_rail_in_window(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	/* Seeded as an empty window (lo > hi), so if pwrseq_rb_window() ever
	 * declined to write them the answer is "out of window" rather than a
	 * comparison against uninitialised memory. */
	int32_t lo = 1;
	int32_t hi = 0;
	int32_t mv;

	if (!in->ina_valid[INA228_RAIL_VCC_RB]) {
		return false;
	}

	(void)pwrseq_rb_window(&ctx->cfg.rb_xfer, ctx->rb_current_code,
			       ctx->cfg.rb_vbus_tol_pct, &lo, &hi);

	mv = in->ina_vbus_mv[INA228_RAIL_VCC_RB];
	return (mv >= lo) && (mv <= hi);
}

/*
 * The independent FE-protection gate: the *measured* rail, straight off INA228
 * 0x47, at or below the FE ceiling. This does not consult the digipot code at
 * all, so a buck that has run away cannot pass it by happening to match its own
 * (wrong) setpoint — the self-referential hole BLOCKER-1 identified. An invalid
 * reading is not provably safe, so it fails.
 */
static bool rb_measured_le_vmax(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	if (!in->ina_valid[INA228_RAIL_VCC_RB]) {
		return false;
	}
	return in->ina_vbus_mv[INA228_RAIL_VCC_RB] <=
	       (int32_t)ctx->cfg.rb_vmax_mv;
}

/* The commanded operating setpoint's expected voltage is within the FE ceiling.
 * Redundant with the init bound (which rejects a config that violates it), but
 * the reviewer asked for a runtime gate that is independent of the measurement,
 * and defence in depth here is free. */
static bool rb_operating_expected_le_vmax(const pwrseq_ctx_t *ctx)
{
	return pwrseq_rb_expected_mv(&ctx->cfg.rb_xfer,
				     ctx->cfg.digipot_operating_code) <=
	       (int32_t)ctx->cfg.rb_vmax_mv;
}

/* --------------------------------------------------------------- step table */

typedef bool (*pwrseq_pred_fn)(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in);
typedef uint32_t (*pwrseq_tmo_fn)(const pwrseq_cfg_t *cfg);

typedef struct {
	uint8_t stage;
	uint8_t action;
	uint8_t onfail;
	uint8_t retries;
	uint8_t failact;
	uint8_t alarm;
	pwrseq_pred_fn guard;   /* NULL = always run this step */
	pwrseq_tmo_fn delay;    /* NULL = no settle delay */
	pwrseq_pred_fn exit;    /* NULL = advance once the delay is done */
	pwrseq_tmo_fn timeout;  /* NULL = wait forever */
	const char *name;
} pwrseq_step_def_t;

/* --- guards --- */

static bool g_ui(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	(void)ctx;
	return in->ui_wanted;
}

static bool g_display(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	return in->ui_wanted && (ctx->shed < (uint8_t)PWRSEQ_SHED_DISPLAY);
}

static bool g_panel(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	return in->ui_wanted && (ctx->shed < (uint8_t)PWRSEQ_SHED_PANEL_LED);
}

static bool g_rb(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	return in->rb_wanted && (ctx->shed < (uint8_t)PWRSEQ_SHED_RB);
}

static bool g_no_debugger(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	(void)ctx;
	/* external_wdt §7.4: the TPS3430 cannot freeze on a core halt, so a
	 * debug session leaves it disarmed rather than cold-cycling the board
	 * at the first breakpoint. */
	return !in->debugger_attached;
}

/* --- exit predicates --- */

static bool p_nor(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	(void)ctx;
	return in->nor_ready;
}

static bool p_i2c(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	(void)ctx;
	return in->i2c_probe_ok;
}

static bool p_trims(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	(void)ctx;
	return in->shunt_trims_applied;
}

static bool p_baseline(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	int i;

	(void)ctx;
	for (i = 0; i < (int)INA228_RAIL_COUNT; i++) {
		if (!in->ina_valid[i]) {
			return false;
		}
	}
	return true;
}

static bool p_rail_3v3(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	/* Interface ref §3: confirm 3V3 via INA228 0x43, corroborated by PG4. */
	return ((in->pg_mask & PWRSEQ_PG_3V3_PSU) != 0U) &&
	       rail_in_band(ctx, in, INA228_RAIL_3V3_MAIN);
}

static bool p_rail_3v3_stm(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	return rail_in_band(ctx, in, INA228_RAIL_3V3_STM);
}

static bool p_rail_poe(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	/* PG7 plus a real bus voltage on 0x40 — VBUS reads the 54 V rail
	 * directly, no divider (interface ref §4.2). */
	return ((in->pg_mask & PWRSEQ_PG_POE) != 0U) &&
	       in->ina_valid[INA228_RAIL_POE] &&
	       (in->ina_vbus_mv[INA228_RAIL_POE] >= ctx->cfg.poe_min_mv);
}

static bool p_phy_clk(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	(void)ctx;
	return in->phy_refclk_stable;
}

static bool p_phy_id(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	(void)ctx;
	/* The ID read, not link: the box must finish bring-up with the cable
	 * unplugged. */
	return in->phy_id_ok;
}

static bool p_gps_rail(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	return ((in->pg_mask & PWRSEQ_PG_3V3_GPS_LDO) != 0U) &&
	       rail_in_band(ctx, in, INA228_RAIL_3V3_GPS);
}

static bool p_ant(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	(void)ctx;
	/* A current reading is all this step needs; open-versus-short is the
	 * antenna supervisor's job, started by the next step. */
	return in->ina_valid[INA228_RAIL_V_ANT];
}

static bool p_gnss_cfg(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	(void)ctx;
	return in->gnss_cfg_ack;
}

static bool p_rail_disp(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	return rail_in_band(ctx, in, INA228_RAIL_5V_DISP);
}

static bool p_rail_panel(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	return rail_in_band(ctx, in, INA228_RAIL_PANEL_5V);
}

static bool p_ocxo_warm(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	return pwrseq_ocxo_is_warm(ctx, in);
}

static bool p_poe_budget(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	return pwrseq_poe_headroom_mw(in) >= ctx->cfg.rb_cold_start_mw;
}

static bool p_rb_precond(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	/* Interface ref §2 stage 8: "Only when PoE budget covers it AND OCXO is
	 * warm AND supercaps charged." The budget is the previous step. */
	return in->supercaps_charged && pwrseq_ocxo_is_warm(ctx, in);
}

static bool p_digipot(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	return in->digipot_readback_valid &&
	       (in->digipot_readback == ctx->cfg.digipot_safe_code);
}

static bool p_digipot_op(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	return in->digipot_readback_valid &&
	       (in->digipot_readback == ctx->cfg.digipot_operating_code);
}

/* Precharge (safe-low) rail verify: the buck is regulating to the safe code. */
static bool p_rb_window(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	return rb_rail_in_window(ctx, in);
}

/*
 * The trust gate before RB_VCC_GATE. All three must hold: the rail matches the
 * commanded operating code (regulation / SPI integrity), the commanded setpoint
 * cannot exceed the FE ceiling (code check), and the *measured* rail is at or
 * below the FE ceiling (the code-independent check that closes BLOCKER-1's
 * self-referential hole). RB_VCC_GATE is the row after this, so the FE is only
 * ever connected once every one of these is true.
 */
static bool p_rb_operating_ok(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	return rb_rail_in_window(ctx, in) &&
	       rb_operating_expected_le_vmax(ctx) &&
	       rb_measured_le_vmax(ctx, in);
}

static bool p_rb_lock(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	(void)ctx;
	return in->rb_lock && in->extref_in_band;
}

static bool p_liveness(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	(void)ctx;
	return in->liveness_ok;
}

/* --- timeout accessors --- */

static uint32_t t_nor(const pwrseq_cfg_t *c) { return c->nor_timeout_ms; }
static uint32_t t_i2c(const pwrseq_cfg_t *c) { return c->i2c_timeout_ms; }
static uint32_t t_trim(const pwrseq_cfg_t *c) { return c->trim_timeout_ms; }
static uint32_t t_rail(const pwrseq_cfg_t *c) { return c->rail_timeout_ms; }
static uint32_t t_phy_clk(const pwrseq_cfg_t *c) { return c->phy_clk_timeout_ms; }
static uint32_t t_phy_settle(const pwrseq_cfg_t *c) { return c->phy_settle_ms; }
static uint32_t t_phy_id(const pwrseq_cfg_t *c) { return c->phy_id_timeout_ms; }
static uint32_t t_gps_rail(const pwrseq_cfg_t *c) { return c->gps_rail_timeout_ms; }
static uint32_t t_gps_boot(const pwrseq_cfg_t *c) { return c->gps_boot_ms; }
static uint32_t t_ant(const pwrseq_cfg_t *c) { return c->ant_timeout_ms; }
static uint32_t t_gnss(const pwrseq_cfg_t *c) { return c->gnss_cfg_timeout_ms; }
static uint32_t t_disp(const pwrseq_cfg_t *c) { return c->disp_timeout_ms; }
static uint32_t t_ocxo(const pwrseq_cfg_t *c) { return c->ocxo_warm_timeout_ms; }
static uint32_t t_precond(const pwrseq_cfg_t *c)
{
	return c->rb_precondition_timeout_ms;
}
static uint32_t t_softstart(const pwrseq_cfg_t *c) { return c->rb_softstart_ms; }
static uint32_t t_ramp(const pwrseq_cfg_t *c) { return c->rb_ramp_ms; }
static uint32_t t_rb_win(const pwrseq_cfg_t *c) { return c->rb_window_timeout_ms; }
static uint32_t t_rb_lock(const pwrseq_cfg_t *c) { return c->rb_lock_timeout_ms; }
static uint32_t t_liveness(const pwrseq_cfg_t *c) { return c->liveness_timeout_ms; }

/*
 * The sequence, one row per numbered step of interface ref §2. The `name`
 * strings carry the document's own step numbers so a log line or a failing
 * assertion points straight at the paragraph it came from.
 */
static const pwrseq_step_def_t step_tbl[] = {
	/* ---- Stage 2: release held-in-reset digital devices (early) ---- */
	{
		.stage = PWRSEQ_STAGE_2_RELEASE,
		.action = PWRSEQ_ACT_NOR_RST_RELEASE,
		.exit = p_nor, .timeout = t_nor,
		.onfail = PWRSEQ_ONFAIL_RETRY, .retries = 2,
		.alarm = PWRSEQ_ALARM_NOR,
		.name = "2.1 nor-rst",
	},
	{
		.stage = PWRSEQ_STAGE_2_RELEASE,
		.action = PWRSEQ_ACT_I2C_PROBE,
		.exit = p_i2c, .timeout = t_i2c,
		.onfail = PWRSEQ_ONFAIL_RETRY, .retries = 2,
		.alarm = PWRSEQ_ALARM_I2C,
		.name = "2.2 i2c-probe",
	},
	{
		.stage = PWRSEQ_STAGE_2_RELEASE,
		.action = PWRSEQ_ACT_DISP_RST_RELEASE,
		.guard = g_ui,
		.name = "2.3 disp-rst",
	},

	/* ---- Stage 3: verify main rails via telemetry + PG lines ---- */
	{
		/*
		 * The trim comes first, ahead of the document's own prose
		 * order, because §3's last bullet requires it "before any
		 * current reading is used for a budget or alarm decision" and
		 * ARCHITECTURE.md invariant 7 makes that binding. A
		 * freshly-reset INA228 silently reverts to POR calibration.
		 */
		.stage = PWRSEQ_STAGE_3_RAILS,
		.action = PWRSEQ_ACT_APPLY_SHUNT_TRIMS,
		.exit = p_trims, .timeout = t_trim,
		.onfail = PWRSEQ_ONFAIL_RETRY, .retries = 2,
		.alarm = PWRSEQ_ALARM_SHUNT_TRIM,
		.name = "3.1 shunt-trims",
	},
	{
		.stage = PWRSEQ_STAGE_3_RAILS,
		.action = PWRSEQ_ACT_READ_RAIL_BASELINE,
		.exit = p_baseline, .timeout = t_rail,
		.onfail = PWRSEQ_ONFAIL_RETRY, .retries = 2,
		.alarm = PWRSEQ_ALARM_RAIL_BASELINE,
		.name = "3.2 baseline",
	},
	{
		/* No 3V3 means no peripherals; halting leaves every gated load
		 * off, the relay de-energized and the watchdog unarmed, which
		 * is the correct resting state for a hardware fault a cold
		 * cycle cannot fix. */
		.stage = PWRSEQ_STAGE_3_RAILS,
		.exit = p_rail_3v3, .timeout = t_rail,
		.onfail = PWRSEQ_ONFAIL_HALT,
		.alarm = PWRSEQ_ALARM_RAIL_3V3,
		.name = "3.3 rail-3v3",
	},
	{
		.stage = PWRSEQ_STAGE_3_RAILS,
		.exit = p_rail_3v3_stm, .timeout = t_rail,
		.onfail = PWRSEQ_ONFAIL_HALT,
		.alarm = PWRSEQ_ALARM_RAIL_3V3_STM,
		.name = "3.4 rail-3v3-stm",
	},
	{
		.stage = PWRSEQ_STAGE_3_RAILS,
		.exit = p_rail_poe, .timeout = t_rail,
		.onfail = PWRSEQ_ONFAIL_ALARM,
		.alarm = PWRSEQ_ALARM_RAIL_POE,
		.name = "3.5 rail-poe",
	},

	/* ---- Stage 4: bring up the PHY ---- */
	{
		.stage = PWRSEQ_STAGE_4_PHY,
		.exit = p_phy_clk, .timeout = t_phy_clk,
		.onfail = PWRSEQ_ONFAIL_ALARM,
		.alarm = PWRSEQ_ALARM_PHY,
		.name = "4.0 refclk",
	},
	{
		.stage = PWRSEQ_STAGE_4_PHY,
		.action = PWRSEQ_ACT_LAN_RST_RELEASE,
		.delay = t_phy_settle,
		.name = "4.1 lan-rst",
	},
	{
		.stage = PWRSEQ_STAGE_4_PHY,
		.action = PWRSEQ_ACT_PHY_MDIO_POLL,
		.exit = p_phy_id, .timeout = t_phy_id,
		.onfail = PWRSEQ_ONFAIL_RETRY, .retries = 2,
		.alarm = PWRSEQ_ALARM_PHY,
		.name = "4.2 mdio",
	},

	/* ---- Stage 5: GNSS + antenna ---- */
	{
		.stage = PWRSEQ_STAGE_5_GNSS,
		.action = PWRSEQ_ACT_GPS_PWR_EN,
		.exit = p_gps_rail, .timeout = t_gps_rail,
		.onfail = PWRSEQ_ONFAIL_RETRY, .retries = 1,
		.failact = PWRSEQ_FAILACT_GPS_OFF,
		.alarm = PWRSEQ_ALARM_GPS_RAIL,
		.name = "5.1 gps-pwr",
	},
	{
		.stage = PWRSEQ_STAGE_5_GNSS,
		.action = PWRSEQ_ACT_GPS_RST_RELEASE,
		.delay = t_gps_boot,
		.name = "5.2 gps-rst",
	},
	{
		.stage = PWRSEQ_STAGE_5_GNSS,
		.action = PWRSEQ_ACT_ANT_BIAS_EN,
		.exit = p_ant, .timeout = t_ant,
		.onfail = PWRSEQ_ONFAIL_ALARM,
		.failact = PWRSEQ_FAILACT_ANT_OFF,
		.alarm = PWRSEQ_ALARM_ANTENNA,
		.name = "5.3 ant-bias",
	},
	{
		.stage = PWRSEQ_STAGE_5_GNSS,
		.action = PWRSEQ_ACT_ANT_SUPERVISOR_START,
		.name = "5.4 ant-supervisor",
	},
	{
		/* Interface ref §10 caution 2: PD5 is GEOFENCE_STAT until
		 * CFG-TXREADY re-maps it, so this ACK gates trusting GPS_TXRDY
		 * (ARCHITECTURE.md invariant 8). */
		.stage = PWRSEQ_STAGE_5_GNSS,
		.action = PWRSEQ_ACT_GNSS_CONFIG_REQUEST,
		.exit = p_gnss_cfg, .timeout = t_gnss,
		.onfail = PWRSEQ_ONFAIL_RETRY, .retries = 2,
		.alarm = PWRSEQ_ALARM_GNSS_CFG,
		.name = "5.5 gnss-cfg",
	},

	/* ---- Stage 6: display / panel (deferrable) ---- */
	{
		.stage = PWRSEQ_STAGE_6_PANEL,
		.action = PWRSEQ_ACT_DISP_EN,
		.guard = g_display,
		.exit = p_rail_disp, .timeout = t_disp,
		.onfail = PWRSEQ_ONFAIL_ALARM,
		.failact = PWRSEQ_FAILACT_DISP_OFF,
		.alarm = PWRSEQ_ALARM_DISPLAY,
		.name = "6.1 disp-en",
	},
	{
		.stage = PWRSEQ_STAGE_6_PANEL,
		.action = PWRSEQ_ACT_PANEL_LED_EN,
		.guard = g_panel,
		.exit = p_rail_panel, .timeout = t_disp,
		.onfail = PWRSEQ_ONFAIL_ALARM,
		.failact = PWRSEQ_FAILACT_PANEL_OFF,
		.alarm = PWRSEQ_ALARM_PANEL_LED,
		.name = "6.2 panel-led-en",
	},
	{
		.stage = PWRSEQ_STAGE_6_PANEL,
		.action = PWRSEQ_ACT_PANEL_LED_PWM,
		.guard = g_panel,
		.name = "6.3 panel-led-pwm",
	},

	/* ---- Stage 7: OCXO discipline ---- */
	{
		.stage = PWRSEQ_STAGE_7_OCXO,
		.exit = p_ocxo_warm, .timeout = t_ocxo,
		.onfail = PWRSEQ_ONFAIL_ALARM,
		.alarm = PWRSEQ_ALARM_OCXO_WARM,
		.name = "7.1 ocxo-warm",
	},
	{
		.stage = PWRSEQ_STAGE_7_OCXO,
		.action = PWRSEQ_ACT_DISC_START,
		.name = "7.2 disc-start",
	},

	/* ---- Stage 8: rubidium (last, guarded) ---- */
	{
		/* An inadequate budget defers rather than fails: the board runs
		 * on the OCXO and pwrseq_rb_retry() can try again after a shed
		 * frees the watts (spec §10.3). */
		.stage = PWRSEQ_STAGE_8_RB,
		.guard = g_rb,
		.exit = p_poe_budget, .timeout = t_precond,
		.onfail = PWRSEQ_ONFAIL_DEFER,
		.alarm = PWRSEQ_ALARM_POE_BUDGET,
		.name = "8.0 poe-budget",
	},
	{
		.stage = PWRSEQ_STAGE_8_RB,
		.guard = g_rb,
		.exit = p_rb_precond, .timeout = t_precond,
		.onfail = PWRSEQ_ONFAIL_DEFER,
		.alarm = PWRSEQ_ALARM_RB_PRECONDITION,
		.name = "8.0b warm+supercaps",
	},
	{
		/* Step 11: write a known safe-low digipot code. */
		.stage = PWRSEQ_STAGE_8_RB,
		.guard = g_rb,
		.action = PWRSEQ_ACT_DIGIPOT_WRITE,
		.name = "8.11 digipot-write",
	},
	{
		/*
		 * ...and prove it took. Nothing downstream may run on an
		 * unverified wiper: a POR mid-scale code is ~14.5 V and an SPI
		 * fault could leave anything at all on the ladder.
		 */
		.stage = PWRSEQ_STAGE_8_RB,
		.guard = g_rb,
		.action = PWRSEQ_ACT_DIGIPOT_VERIFY,
		.exit = p_digipot, .timeout = t_rb_win,
		.onfail = PWRSEQ_ONFAIL_RETRY, .retries = 2,
		.failact = PWRSEQ_FAILACT_RB_OFF,
		.alarm = PWRSEQ_ALARM_RB_DIGIPOT,
		.name = "8.11b digipot-verify",
	},
	{
		/* Step 12: RB_PWR_EN, then let the buck soft-start. */
		.stage = PWRSEQ_STAGE_8_RB,
		.guard = g_rb,
		.action = PWRSEQ_ACT_RB_PWR_EN,
		.delay = t_softstart,
		.name = "8.12 rb-pwr-en",
	},
	{
		/* Step 13: verify VCC_RB on INA228 0x47 before trusting it. */
		.stage = PWRSEQ_STAGE_8_RB,
		.guard = g_rb,
		.exit = p_rb_window, .timeout = t_rb_win,
		.onfail = PWRSEQ_ONFAIL_ALARM,
		.failact = PWRSEQ_FAILACT_RB_OFF,
		.alarm = PWRSEQ_ALARM_RB_WINDOW,
		.name = "8.13 rail-window",
	},
	{
		/* Step 14: only now connect VCC_RB_G to the FE. */
		.stage = PWRSEQ_STAGE_8_RB,
		.guard = g_rb,
		.action = PWRSEQ_ACT_RB_VCC_GATE_EN,
		.name = "8.14 rb-vcc-gate",
	},
	{
		/* Step 15: wait for lock and an in-band external reference. */
		.stage = PWRSEQ_STAGE_8_RB,
		.guard = g_rb,
		.exit = p_rb_lock, .timeout = t_rb_lock,
		.onfail = PWRSEQ_ONFAIL_ALARM,
		.failact = PWRSEQ_FAILACT_RB_OFF,
		.alarm = PWRSEQ_ALARM_RB_LOCK,
		.name = "8.15 rb-lock",
	},

	/* ---- Stage 9: arm watchdog and alarms ---- */
	{
		.stage = PWRSEQ_STAGE_9_ARM,
		.exit = p_liveness, .timeout = t_liveness,
		.onfail = PWRSEQ_ONFAIL_ALARM,
		.alarm = PWRSEQ_ALARM_LIVENESS,
		.name = "9.16a liveness",
	},
	{
		.stage = PWRSEQ_STAGE_9_ARM,
		.action = PWRSEQ_ACT_WDT_EN,
		.guard = g_no_debugger,
		.name = "9.16b wdt-en",
	},
	{
		.stage = PWRSEQ_STAGE_9_ARM,
		.action = PWRSEQ_ACT_WDT_KICK_START,
		.guard = g_no_debugger,
		.name = "9.16c wdt-kick",
	},
	{
		.stage = PWRSEQ_STAGE_9_ARM,
		.action = PWRSEQ_ACT_RELAY_ELIGIBLE,
		.name = "9.17 relay",
	},
};

#define STEP_COUNT ((int16_t)(sizeof(step_tbl) / sizeof(step_tbl[0])))

/* First row belonging to @p stage, or -1 if the stage has no rows. */
static int16_t first_step_of(uint8_t stage)
{
	int16_t i;

	for (i = 0; i < STEP_COUNT; i++) {
		if (step_tbl[i].stage == stage) {
			return i;
		}
	}
	return -1;
}

/* ---------------------------------------------------------- action queue */

static size_t act_free(const pwrseq_ctx_t *ctx)
{
	return (size_t)PWRSEQ_ACT_QUEUE_LEN - (size_t)ctx->q_len;
}

static void act_push_raw(pwrseq_ctx_t *ctx, pwrseq_action_t a, uint16_t arg)
{
	pwrseq_act_t *e;

	if (ctx->q_len >= (uint16_t)PWRSEQ_ACT_QUEUE_LEN) {
		/*
		 * The step machine reserves room before every step, so it can
		 * never land here. Only an out-of-band caller that has stopped
		 * draining can, and losing an action means this module's belief
		 * about the board has diverged from reality — hence a counter
		 * the caller is expected to check, not a silent drop.
		 */
		ctx->q_dropped++;
		return;
	}

	e = &ctx->q[(ctx->q_head + ctx->q_len) % (uint16_t)PWRSEQ_ACT_QUEUE_LEN];
	e->action = (uint16_t)a;
	e->arg = arg;
	ctx->q_len++;
}

static uint16_t action_arg(const pwrseq_ctx_t *ctx, pwrseq_action_t a)
{
	if (a == PWRSEQ_ACT_DIGIPOT_WRITE) {
		return ctx->cfg.digipot_safe_code;
	}
	if (a == PWRSEQ_ACT_PANEL_LED_PWM) {
		return ctx->cfg.panel_led_duty_pct;
	}
	return 0U;
}

/* Queue @p a and fold its effect into the sequencer's model of the board. */
static void emit(pwrseq_ctx_t *ctx, pwrseq_action_t a)
{
	switch (a) {
	case PWRSEQ_ACT_NOR_RST_RELEASE:
		ctx->nor_released = true;
		break;
	case PWRSEQ_ACT_DISP_RST_RELEASE:
		ctx->disp_rst_released = true;
		break;
	case PWRSEQ_ACT_LAN_RST_RELEASE:
		ctx->phy_released = true;
		break;
	case PWRSEQ_ACT_GPS_PWR_EN:
		ctx->gps_on = true;
		break;
	case PWRSEQ_ACT_GPS_PWR_DIS:
		ctx->gps_on = false;
		break;
	case PWRSEQ_ACT_ANT_BIAS_EN:
		ctx->ant_bias_on = true;
		break;
	case PWRSEQ_ACT_ANT_BIAS_DIS:
		ctx->ant_bias_on = false;
		break;
	case PWRSEQ_ACT_DISP_EN:
		ctx->display_on = true;
		break;
	case PWRSEQ_ACT_DISP_DIS:
		ctx->display_on = false;
		break;
	case PWRSEQ_ACT_PANEL_LED_EN:
		ctx->panel_led_on = true;
		break;
	case PWRSEQ_ACT_PANEL_LED_DIS:
		ctx->panel_led_on = false;
		break;
	case PWRSEQ_ACT_DISC_START:
		ctx->disc_started = true;
		break;
	case PWRSEQ_ACT_RB_PWR_EN:
		ctx->rb_enabled = true;
		break;
	case PWRSEQ_ACT_RB_PWR_DIS:
		ctx->rb_enabled = false;
		ctx->rb_locked = false;
		break;
	case PWRSEQ_ACT_RB_VCC_GATE_EN:
		ctx->rb_gated = true;
		break;
	case PWRSEQ_ACT_RB_VCC_GATE_DIS:
		ctx->rb_gated = false;
		break;
	case PWRSEQ_ACT_WDT_EN:
		ctx->wdt_armed = true;
		break;
	case PWRSEQ_ACT_WDT_KICK_START:
		ctx->last_kick_ms = ctx->now_ms;
		break;
	case PWRSEQ_ACT_RELAY_ELIGIBLE:
		ctx->relay_eligible = true;
		break;
	case PWRSEQ_ACT_POE_KILL:
		ctx->kill_armed = true;
		break;
	default:
		break;
	}

	act_push_raw(ctx, a, action_arg(ctx, a));
}

int pwrseq_action_get(pwrseq_ctx_t *ctx, pwrseq_act_t *out)
{
	if ((ctx == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (ctx->q_len == 0U) {
		return -EAGAIN;
	}

	*out = ctx->q[ctx->q_head];
	ctx->q_head =
		(uint16_t)((ctx->q_head + 1U) % (uint16_t)PWRSEQ_ACT_QUEUE_LEN);
	ctx->q_len--;
	return 0;
}

size_t pwrseq_action_count(const pwrseq_ctx_t *ctx)
{
	return (ctx != NULL) ? (size_t)ctx->q_len : 0U;
}

uint32_t pwrseq_actions_dropped(const pwrseq_ctx_t *ctx)
{
	return (ctx != NULL) ? ctx->q_dropped : 0U;
}

/* ------------------------------------------------------------------ alarms */

static void raise_alarm(pwrseq_ctx_t *ctx, uint8_t a)
{
	/* Bit 0 is PWRSEQ_ALARM_NONE, the step table's "no alarm" sentinel, and
	 * is masked off rather than branched on so it can never appear in the
	 * reported set. */
	uint32_t bit = PWRSEQ_ALARM_BIT(a) &
		       ~PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_NONE);

	ctx->alarms |= bit;
	if ((bit & RB_HARD_FAULT_MASK) != 0U) {
		ctx->alarms |= PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_FAULT);
	}
}

uint32_t pwrseq_alarms(const pwrseq_ctx_t *ctx)
{
	return (ctx != NULL) ? ctx->alarms : 0U;
}

bool pwrseq_rb_fault(const pwrseq_ctx_t *ctx)
{
	if (ctx == NULL) {
		return false;
	}
	return (ctx->alarms & PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_FAULT)) != 0U;
}

/* ------------------------------------------------------- rubidium shutdown */

/*
 * Drop the rubidium, load side first.
 *
 * Both actions are emitted unconditionally rather than only for the parts this
 * module believes are on. This path runs because something already went wrong,
 * and it must not depend on the sequencer's own bookkeeping being correct — the
 * writes are idempotent at the pin.
 */
static void rb_shutdown(pwrseq_ctx_t *ctx)
{
	emit(ctx, PWRSEQ_ACT_RB_VCC_GATE_DIS);
	emit(ctx, PWRSEQ_ACT_RB_PWR_DIS);
	ctx->rb_locked = false;
}

static void do_failact(pwrseq_ctx_t *ctx, uint8_t fa)
{
	switch ((pwrseq_failact_t)fa) {
	case PWRSEQ_FAILACT_GPS_OFF:
		emit(ctx, PWRSEQ_ACT_GPS_PWR_DIS);
		break;
	case PWRSEQ_FAILACT_ANT_OFF:
		emit(ctx, PWRSEQ_ACT_ANT_BIAS_DIS);
		break;
	case PWRSEQ_FAILACT_DISP_OFF:
		emit(ctx, PWRSEQ_ACT_DISP_DIS);
		break;
	case PWRSEQ_FAILACT_PANEL_OFF:
		emit(ctx, PWRSEQ_ACT_PANEL_LED_DIS);
		break;
	case PWRSEQ_FAILACT_RB_OFF:
		rb_shutdown(ctx);
		break;
	case PWRSEQ_FAILACT_NONE:
	default:
		break;
	}
}

/* --------------------------------------------------------- stage traversal */

/*
 * Open @p stage at its first row.
 *
 * Every caller passes a stage that has rows: pwrseq_start() uses stage 2,
 * pwrseq_restart_stage() validates before calling, and abandon_stage() only
 * ever moves to 3..9. A stageless value would leave `step` at -1, which the
 * step loop already reads as "nothing further to do".
 */
static void enter_stage(pwrseq_ctx_t *ctx, uint8_t stage, uint32_t ms)
{
	ctx->stage = stage;
	ctx->stage_entered_ms = ms;
	ctx->step_entered_ms = ms;
	ctx->retries = 0U;
	ctx->step_armed = false;
	ctx->step = first_step_of(stage);
}

/* Abandon whatever remains of the current stage and open the next one. */
static void abandon_stage(pwrseq_ctx_t *ctx, uint32_t ms)
{
	if (ctx->stage >= (uint8_t)PWRSEQ_STAGE_9_ARM) {
		ctx->stage = (uint8_t)PWRSEQ_STAGE_DONE;
		ctx->stage_entered_ms = ms;
		ctx->step = -1;
		ctx->step_armed = false;
		return;
	}
	enter_stage(ctx, (uint8_t)(ctx->stage + 1U), ms);
}

static void advance_step(pwrseq_ctx_t *ctx, uint32_t ms)
{
	ctx->step++;
	ctx->step_armed = false;
	ctx->retries = 0U;
	ctx->step_entered_ms = ms;

	if (ctx->step >= STEP_COUNT) {
		ctx->stage = (uint8_t)PWRSEQ_STAGE_DONE;
		ctx->stage_entered_ms = ms;
		ctx->step = -1;
	} else if (step_tbl[ctx->step].stage != ctx->stage) {
		ctx->stage = step_tbl[ctx->step].stage;
		ctx->stage_entered_ms = ms;
	}
}

static void handle_failure(pwrseq_ctx_t *ctx, const pwrseq_step_def_t *def,
			   uint32_t ms)
{
	if ((pwrseq_onfail_t)def->onfail == PWRSEQ_ONFAIL_RETRY) {
		if (ctx->retries < def->retries) {
			ctx->retries++;
			ctx->step_armed = false; /* re-emit on the next pass */
			return;
		}
		/* Retries exhausted: escalate exactly as ALARM would. */
	}

	raise_alarm(ctx, def->alarm);
	do_failact(ctx, def->failact);

	switch ((pwrseq_onfail_t)def->onfail) {
	case PWRSEQ_ONFAIL_HALT:
		ctx->halted = true;
		break;
	case PWRSEQ_ONFAIL_DEFER:
		if (def->stage == (uint8_t)PWRSEQ_STAGE_8_RB) {
			ctx->rb_deferred = true;
		}
		abandon_stage(ctx, ms);
		break;
	case PWRSEQ_ONFAIL_RETRY:
	case PWRSEQ_ONFAIL_ALARM:
	default:
		abandon_stage(ctx, ms);
		break;
	}
}

/* ------------------------------------------------------------------- init */

int pwrseq_init(pwrseq_ctx_t *ctx, const pwrseq_cfg_t *cfg)
{
	if (ctx == NULL) {
		return -EINVAL;
	}

	memset(ctx, 0, sizeof(*ctx));

	if (cfg != NULL) {
		ctx->cfg = *cfg;
	} else {
		pwrseq_cfg_default(&ctx->cfg);
	}

	if ((ctx->cfg.rb_xfer.steps == 0U) || (ctx->cfg.rb_xfer.vref_mv == 0U)) {
		return -EINVAL;
	}

	/*
	 * A windowed watchdog punishes an early kick exactly as hard as a late
	 * one, so a cadence outside 920-1360 ms is rejected here rather than
	 * discovered as an unexplained cold cycle in the field.
	 */
	if ((ctx->cfg.wdt_kick_period_ms < PWRSEQ_WDT_WINDOW_MIN_MS) ||
	    (ctx->cfg.wdt_kick_period_ms > PWRSEQ_WDT_WINDOW_MAX_MS)) {
		return -EINVAL;
	}

	ctx->stage = (uint8_t)PWRSEQ_STAGE_IDLE;
	ctx->step = -1;
	return 0;
}

int pwrseq_start(pwrseq_ctx_t *ctx, uint32_t mono_ms)
{
	if (ctx == NULL) {
		return -EINVAL;
	}

	ctx->now_ms = mono_ms;
	ctx->halted = false;
	enter_stage(ctx, (uint8_t)PWRSEQ_STAGE_2_RELEASE, mono_ms);
	return 0;
}

/* ------------------------------------------------------------------- step */

int pwrseq_step(pwrseq_ctx_t *ctx, const pwrseq_in_t *in)
{
	unsigned int iter;
	uint32_t ms;

	if ((ctx == NULL) || (in == NULL)) {
		return -EINVAL;
	}

	ms = in->mono_ms;
	ctx->now_ms = ms;
	(void)pwrseq_ov_observe(ctx, in->rb_ov_det, ms);

	/*
	 * Rubidium supervision, ahead of the step machine and independent of
	 * the stage: an over-voltage latch or a rail that has left its window
	 * drops the load now, whether that happens during stage 8 or hours into
	 * service. Deferred if the caller has not drained enough room to queue
	 * both halves — a half-applied shutdown is worse than a late one.
	 */
	if (ctx->rb_enabled && (act_free(ctx) >= 2U)) {
		if (ctx->ov_latched) {
			rb_shutdown(ctx);
			raise_alarm(ctx, (uint8_t)PWRSEQ_ALARM_RB_OV);
			if (ctx->stage == (uint8_t)PWRSEQ_STAGE_8_RB) {
				abandon_stage(ctx, ms);
			}
		} else if (ctx->rb_gated && !rb_rail_in_window(ctx, in)) {
			rb_shutdown(ctx);
			raise_alarm(ctx, (uint8_t)PWRSEQ_ALARM_RB_WINDOW);
			if (ctx->stage == (uint8_t)PWRSEQ_STAGE_8_RB) {
				abandon_stage(ctx, ms);
			}
		}
	}

	for (iter = 0U; iter < (unsigned int)PWRSEQ_MAX_STEPS_PER_CALL; iter++) {
		const pwrseq_step_def_t *def;
		uint32_t elapsed;

		if (ctx->halted || (ctx->step < 0) ||
		    (ctx->stage == (uint8_t)PWRSEQ_STAGE_IDLE)) {
			break;
		}
		if (act_free(ctx) < (size_t)PWRSEQ_ACT_PER_STEP_MAX) {
			break; /* drain first; nothing is lost by waiting */
		}

		def = &step_tbl[ctx->step];

		if (!ctx->step_armed) {
			if ((def->guard != NULL) && !def->guard(ctx, in)) {
				advance_step(ctx, ms);
				continue;
			}
			if (def->action != (uint8_t)PWRSEQ_ACT_NONE) {
				emit(ctx, (pwrseq_action_t)def->action);
			}
			ctx->step_entered_ms = ms;
			ctx->step_armed = true;
		}

		elapsed = ms - ctx->step_entered_ms;

		if ((def->delay != NULL) && (elapsed < def->delay(&ctx->cfg))) {
			break; /* still settling */
		}
		if (def->exit == NULL) {
			advance_step(ctx, ms);
			continue;
		}
		if (def->exit(ctx, in)) {
			advance_step(ctx, ms);
			continue;
		}
		if ((def->timeout != NULL) &&
		    (elapsed >= def->timeout(&ctx->cfg))) {
			handle_failure(ctx, def, ms);
			continue;
		}
		break; /* waiting on the exit condition */
	}

	/* Derived, so it is never stale: the rubidium counts as locked only
	 * while it is actually gated on and both guards hold. */
	ctx->rb_locked = ctx->rb_gated && in->rb_lock && in->extref_in_band;

	return 0;
}

/* ----------------------------------------------------------------- status */

int pwrseq_status(const pwrseq_ctx_t *ctx, pwrseq_status_t *out)
{
	if ((ctx == NULL) || (out == NULL)) {
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));
	out->stage = (pwrseq_stage_t)ctx->stage;
	out->stage_entered_ms = ctx->stage_entered_ms;
	out->retries = ctx->retries;
	out->halted = ctx->halted;
	out->rb_deferred = ctx->rb_deferred;
	out->rb_enabled = ctx->rb_enabled;
	out->rb_gated = ctx->rb_gated;
	out->rb_locked = ctx->rb_locked;
	out->ov_latched = ctx->ov_latched;
	out->wdt_armed = ctx->wdt_armed;
	out->relay_eligible = ctx->relay_eligible;
	out->display_on = ctx->display_on;
	out->panel_led_on = ctx->panel_led_on;
	out->gps_on = ctx->gps_on;
	out->ant_bias_on = ctx->ant_bias_on;
	out->disc_started = ctx->disc_started;
	out->phy_released = ctx->phy_released;
	out->pfi_seen = ctx->pfi_seen;
	out->pfi_expected = ctx->pfi_expected;
	out->shed = (pwrseq_shed_level_t)ctx->shed;
	out->alarms = ctx->alarms;
	return 0;
}

pwrseq_stage_t pwrseq_stage(const pwrseq_ctx_t *ctx)
{
	return (ctx != NULL) ? (pwrseq_stage_t)ctx->stage : PWRSEQ_STAGE_IDLE;
}

int pwrseq_restart_stage(pwrseq_ctx_t *ctx, pwrseq_stage_t stage,
			 uint32_t mono_ms)
{
	if ((ctx == NULL) || (first_step_of((uint8_t)stage) < 0)) {
		return -EINVAL;
	}

	ctx->now_ms = mono_ms;
	ctx->halted = false;
	enter_stage(ctx, (uint8_t)stage, mono_ms);
	return 0;
}

int pwrseq_rb_retry(pwrseq_ctx_t *ctx, uint32_t mono_ms)
{
	if (ctx == NULL) {
		return -EINVAL;
	}

	ctx->rb_deferred = false;
	ctx->alarms &= ~(uint32_t)PWRSEQ_ALARM_RB_MASK;
	ctx->alarms &= ~PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_POE_BUDGET);

	return pwrseq_restart_stage(ctx, PWRSEQ_STAGE_8_RB, mono_ms);
}

/* ------------------------------------------------------------ shed ladder */

pwrseq_shed_level_t pwrseq_shed_level(const pwrseq_ctx_t *ctx)
{
	return (ctx != NULL) ? (pwrseq_shed_level_t)ctx->shed : PWRSEQ_SHED_NONE;
}

int pwrseq_shed_step(pwrseq_ctx_t *ctx, uint32_t mono_ms)
{
	if (ctx == NULL) {
		return -EINVAL;
	}

	ctx->now_ms = mono_ms;

	switch ((pwrseq_shed_level_t)ctx->shed) {
	case PWRSEQ_SHED_NONE:
		emit(ctx, PWRSEQ_ACT_DISP_DIS);
		ctx->shed = (uint8_t)PWRSEQ_SHED_DISPLAY;
		return 0;
	case PWRSEQ_SHED_DISPLAY:
		emit(ctx, PWRSEQ_ACT_PANEL_LED_DIS);
		ctx->shed = (uint8_t)PWRSEQ_SHED_PANEL_LED;
		return 0;
	case PWRSEQ_SHED_PANEL_LED:
		rb_shutdown(ctx);
		ctx->shed = (uint8_t)PWRSEQ_SHED_RB;
		return 0;
	case PWRSEQ_SHED_RB:
	default:
		return -ENOENT;
	}
}

int pwrseq_shed_restore(pwrseq_ctx_t *ctx, uint32_t mono_ms)
{
	if (ctx == NULL) {
		return -EINVAL;
	}

	ctx->now_ms = mono_ms;

	switch ((pwrseq_shed_level_t)ctx->shed) {
	case PWRSEQ_SHED_RB:
		/*
		 * Never a bare RB_PWR_EN. Re-entering stage 8 runs the digipot
		 * write, the readback verify and the rail-window check again,
		 * which is the interlock ARCHITECTURE.md invariant 3 exists to
		 * keep — a shed-and-restore cycle must not become a back door
		 * around it.
		 */
		ctx->shed = (uint8_t)PWRSEQ_SHED_PANEL_LED;
		return pwrseq_rb_retry(ctx, mono_ms);
	case PWRSEQ_SHED_PANEL_LED:
		ctx->shed = (uint8_t)PWRSEQ_SHED_DISPLAY;
		emit(ctx, PWRSEQ_ACT_PANEL_LED_EN);
		emit(ctx, PWRSEQ_ACT_PANEL_LED_PWM);
		return 0;
	case PWRSEQ_SHED_DISPLAY:
		ctx->shed = (uint8_t)PWRSEQ_SHED_NONE;
		emit(ctx, PWRSEQ_ACT_DISP_EN);
		return 0;
	case PWRSEQ_SHED_NONE:
	default:
		return -ENOENT;
	}
}

/* ---------------------------------------------------- over-voltage latch */

int pwrseq_ov_observe(pwrseq_ctx_t *ctx, bool rb_ov_det, uint32_t mono_ms)
{
	if (ctx == NULL) {
		return -EINVAL;
	}

	ctx->ov_present = rb_ov_det;
	if (rb_ov_det && !ctx->ov_latched) {
		ctx->ov_latched = true;
		ctx->ov_first_ms = mono_ms;
		ctx->ov_count++;
	}
	return 0;
}

bool pwrseq_ov_latched(const pwrseq_ctx_t *ctx)
{
	return (ctx != NULL) && ctx->ov_latched;
}

int pwrseq_ov_clear(pwrseq_ctx_t *ctx, uint32_t mono_ms)
{
	if (ctx == NULL) {
		return -EINVAL;
	}
	if (!ctx->ov_latched) {
		return -ENOENT;
	}
	if (ctx->ov_present) {
		return -EBUSY;
	}

	ctx->now_ms = mono_ms;
	emit(ctx, PWRSEQ_ACT_RB_OV_RESET_PULSE);
	ctx->ov_latched = false;
	ctx->alarms &= ~PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_OV);
	return 0;
}

/* ------------------------------------------------------------- power fail */

int pwrseq_pfi(pwrseq_ctx_t *ctx, uint32_t mono_ms)
{
	if (ctx == NULL) {
		return -EINVAL;
	}

	ctx->now_ms = mono_ms;
	ctx->pfi_seen = true;
	ctx->pfi_expected = ctx->kill_armed;

	/* Whatever the sequencer wanted next is irrelevant with ~4.8 ms of
	 * hold-up left; make room for the park list instead. */
	ctx->q_head = 0U;
	ctx->q_len = 0U;

	emit(ctx, PWRSEQ_ACT_PARK_DAC);
	emit(ctx, PWRSEQ_ACT_PERSIST_STATE);
	if (ctx->rb_enabled) {
		rb_shutdown(ctx);
	}
	emit(ctx, PWRSEQ_ACT_SET_SHUTDOWN_FLAG);
	return 0;
}

int pwrseq_poe_kill(pwrseq_ctx_t *ctx, uint32_t magic, uint32_t mono_ms)
{
	if (ctx == NULL) {
		return -EINVAL;
	}
	if (magic != PWRSEQ_POE_KILL_MAGIC) {
		return -EPERM;
	}

	ctx->now_ms = mono_ms;
	emit(ctx, PWRSEQ_ACT_POE_KILL);
	return 0;
}

/* --------------------------------------------------------------- watchdog */

bool pwrseq_wdt_kick_ok(const pwrseq_ctx_t *ctx, uint32_t liveness,
			uint32_t mono_ms)
{
	if ((ctx == NULL) || !ctx->wdt_armed) {
		return false;
	}
	if ((liveness & PWRSEQ_LIVE_ALL) != PWRSEQ_LIVE_ALL) {
		return false;
	}
	return (uint32_t)(mono_ms - ctx->last_kick_ms) >=
	       ctx->cfg.wdt_kick_period_ms;
}

int pwrseq_wdt_kicked(pwrseq_ctx_t *ctx, uint32_t mono_ms)
{
	if (ctx == NULL) {
		return -EINVAL;
	}
	ctx->last_kick_ms = mono_ms;
	return 0;
}

bool pwrseq_wdt_armed(const pwrseq_ctx_t *ctx)
{
	return (ctx != NULL) && ctx->wdt_armed;
}
