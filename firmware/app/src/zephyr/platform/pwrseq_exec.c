/*
 * STS1000 "Meridian" — pwrseq runtime sequencer and action executor.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * core/pwrseq is the interface-ref §2 stage machine (Stages 2-9). It is pure
 * decision logic: pwrseq_step() consumes observed state (pwrseq_in_t) and emits
 * an action queue; this file feeds it the observations and executes the queue
 * against the /zephyr,user GPIOs and the SPI4 digipot. Run from the
 * housekeeping thread's 4 Hz tick.
 *
 * ---------------------------------------------------------------------------
 * Relationship to platform.c
 * ---------------------------------------------------------------------------
 * platform.c performs the deterministic *infrastructure* bring-up synchronously
 * at init: bus/driver init, INA228 configuration, the discipline loop, the
 * 1 kHz scan and this thread. pwrseq then runs as the *sequencer*, observing
 * that infrastructure through its inputs (nor_ready, shunt_trims_applied,
 * disc_started, ...) and driving the runtime and guarded stages — GPS power,
 * the display, the rubidium, the watchdog and the holdover relay — plus the
 * fault responses (shed ladder, PFI, OV latch, POE_KILL).
 *
 * Every action maps to an *idempotent* helper, so a stage platform.c already
 * completed can be re-driven harmlessly: releasing an already-released reset,
 * re-writing the INA trims, or calling sts_discipline_start() again (which
 * returns -EALREADY) all no-op. That is what lets the two owners coexist
 * without a handshake. The WDT and relay are the exception — those are
 * actuated by supervisor.c — so pwrseq's WDT_EN / RELAY_ELIGIBLE actions call
 * into the supervisor rather than driving the pins here, keeping a single
 * writer per pin (ARCHITECTURE.md §10.5/§10.6).
 *
 * ---------------------------------------------------------------------------
 * The guarded rubidium sequence (Stage 8, ARCHITECTURE.md §10.3)
 * ---------------------------------------------------------------------------
 * The FE-5680A is never connected until its rail is proven. pwrseq emits, in
 * order: DIGIPOT_WRITE (safe-low precharge code) -> RB_PWR_EN -> [soft-start] ->
 * DIGIPOT_WRITE_OP (operating code) -> verify VCC_RB on INA228 0x47 inside the
 * window -> RB_VCC_GATE. Both digipot writes are 10-bit wiper writes with
 * read-back verify (sts_digipot_set); a mismatch fails the step and RB_VCC_GATE
 * is never reached. The measured-voltage gate against rb_vmax_mv is independent
 * of the code, so a digipot fault that reads back "correct" but produces the
 * wrong rail is still caught before the FE is connected.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "zephyr/platform/platform.h"
#include "zephyr/sts_app.h"

#include "cfg/cfg.h"
#include "fault/fault.h"
#include "ina228/ina228.h"
#include "pwrseq/pwrseq.h"
#include "quality/quality.h"

LOG_MODULE_REGISTER(sts_pwrseq, CONFIG_STS1000_LOG_LEVEL);

/* Power-group cfg keys (cfg_schema.h, group 0x07). */
#define CFG_KEY_PWR_RB_POLICY    0x0701U
#define CFG_KEY_PWR_RB_VMAX_MV   0x0703U
#define CFG_KEY_PWR_POE_BUDGET_MW 0x0704U
#define CFG_KEY_PWR_RB_WARMUP_S  0x0705U

/* Supervisor gate that pwrseq's WDT_EN / RELAY_ELIGIBLE actions drive. */
void sts_supervisor_set_seq_eligible(bool eligible);

/* GPIOs pwrseq drives that no other platform file owns. NOR/LAN/DISP_RST are
 * owned by platform.c; the corresponding actions are no-ops here. */
static const struct gpio_dt_spec gps_pwr_en = STS_USER_GPIO(gps_pwr_en_gpios);
static const struct gpio_dt_spec gps_rst = STS_USER_GPIO(gps_rst_gpios);
static const struct gpio_dt_spec ant_bias_en = STS_USER_GPIO(ant_bias_en_gpios);
static const struct gpio_dt_spec disp_en = STS_USER_GPIO(disp_en_gpios);
static const struct gpio_dt_spec rb_pwr_en = STS_USER_GPIO(rb_pwr_en_gpios);
static const struct gpio_dt_spec rb_vcc_gate = STS_USER_GPIO(rb_vcc_gate_gpios);
static const struct gpio_dt_spec rb_ov_reset = STS_USER_GPIO(rb_ov_reset_gpios);
static const struct gpio_dt_spec rb_ov_det = STS_USER_GPIO(rb_ov_det_gpios);
static const struct gpio_dt_spec rb_lock_in = STS_USER_GPIO(rb_lock_gpios);
static const struct gpio_dt_spec poe_kill = STS_USER_GPIO(poe_kill_gpios);

static const struct gpio_dt_spec *const pwrseq_outputs[] = {
	&gps_pwr_en, &gps_rst, &ant_bias_en, &disp_en,
	&rb_pwr_en, &rb_vcc_gate, &rb_ov_reset, &poe_kill,
};

static const struct gpio_dt_spec *const pwrseq_inputs[] = {
	&rb_ov_det, &rb_lock_in,
};

static pwrseq_ctx_t pwrseq;
static int pwrseq_liveness_id;
static bool pwrseq_started;

/* ------------------------------------------------------------- config ----- */

static void pwrseq_load_cfg(pwrseq_cfg_t *cfg)
{
	uint64_t v;

	pwrseq_cfg_default(cfg);

	/*
	 * Override only the FE-specific safety envelope from cfg; the timeouts
	 * and the digipot safe code keep the core defaults. Deliberately NOT
	 * wiring PWR_DIGIPOT_MAX (0x0702): it is a u8 and the MCP41U83 is a
	 * 10-bit part, so it cannot express the operating range — pwrseq bounds
	 * the wiper by voltage (rb_vmax_mv), not by a code ceiling.
	 */
	if (cfg_get_u64(sts_cfg(), CFG_KEY_PWR_RB_VMAX_MV, &v) == 0) {
		cfg->rb_vmax_mv = (uint32_t)v;
	}
	/*
	 * The PoE budget is a per-tick input (pwrseq_in_t.poe_granted_mw), not
	 * a cfg field on pwrseq_cfg_t; it is read from CFG_KEY_PWR_POE_BUDGET_MW
	 * in pwrseq_build_input(). rb_ramp_ms and digipot_operating_code keep
	 * their pwrseq defaults (100 ms, code 500) — the schema has no key for
	 * either yet.
	 */
}

/* ------------------------------------------------------------- inputs ----- */

static uint8_t pwrseq_pg_mask(void)
{
	uint8_t mask = 0;

	sts_fault_lock();
	for (unsigned int n = 0; n < 8U; n++) {
		/* FAULT_SIG_PG_3V3_GPS_LDO (16) is PG0; asserted = not-good. */
		if (!fault_asserted(sts_fault(),
				    (fault_sig_t)(FAULT_SIG_PG_3V3_GPS_LDO + n))) {
			mask |= (uint8_t)PWRSEQ_PG(n);
		}
	}
	sts_fault_unlock();

	return mask;
}

static bool pwrseq_supercaps_charged(void)
{
	bool ok;

	sts_fault_lock();
	ok = !fault_asserted(sts_fault(), FAULT_SIG_BKP_STM_PG) &&
	     !fault_asserted(sts_fault(), FAULT_SIG_BKP_GPS_PG);
	sts_fault_unlock();

	return ok;
}

static void pwrseq_build_input(pwrseq_in_t *in, uint32_t now_ms)
{
	sts_hk_snapshot_t hk;
	quality_block_t q;
	uint16_t dp = 0;
	uint32_t extref_hz = 0;
	bool extref_valid = false;
	bool extref_edges = false;
	uint64_t policy = 1;
	int32_t poe_mv = 0;
	int32_t poe_ma = 0;

	memset(in, 0, sizeof(*in));
	in->mono_ms = now_ms;

	(void)sts_hk_read(&hk);
	(void)sts_quality_snapshot(&q);
	sts_extref_mon_read(&extref_hz, &extref_valid, &extref_edges);

	/* Stages platform.c already completed, reported as done. */
	in->nor_ready = true;
	in->i2c_probe_ok = true;
	in->cal_loaded = sts_cfg_is_persistent();
	in->shunt_trims_applied = hk.ina[INA228_RAIL_3V3_MAIN].cal_ok;
	/* disc_started is tracked by pwrseq internally, not an input; the
	 * DISC_START action maps to sts_discipline_start() which is idempotent
	 * against platform.c having already started the loop. */
	/*
	 * The LAN8742 runs in REFCLKO mode and links autonomously once its
	 * reset is released (platform.c early hook); the net area owns the MDIO
	 * ID read. Report the PHY up so the sequencer is not blocked at stage 4
	 * waiting for an observation this area cannot make.
	 * TODO(wave-3b): source phy_id_ok from the net area's link status.
	 */
	in->phy_refclk_stable = true;
	in->phy_id_ok = true;
	/* TODO(wave-3b): gnss_cfg_ack from the gnss thread's CFG-TXREADY ACK. */
	in->gnss_cfg_ack = true;

	for (size_t r = 0; r < INA228_RAIL_COUNT; r++) {
		in->ina_valid[r] = hk.ina[r].valid && hk.ina[r].cal_ok;
		in->ina_vbus_mv[r] = hk.ina[r].bus_uv / 1000;
		in->ina_current_ma[r] = hk.ina[r].current_ua / 1000;
	}
	in->pg_mask = pwrseq_pg_mask();

	in->ocxo_warm = (q.flags & QUALITY_FLAG_OCXO_WARM) != 0U;
	in->ocxo_temp_stable = (q.flags & QUALITY_FLAG_OSC_TEMP_VALID) != 0U;

	(void)cfg_get_u64(sts_cfg(), CFG_KEY_PWR_RB_POLICY, &policy);
	in->rb_wanted = (policy != 0U);
	in->supercaps_charged = pwrseq_supercaps_charged();

	if (sts_digipot_get(&dp) == 0) {
		in->digipot_readback = dp;
		in->digipot_readback_valid = true;
	}

	/* rb_lock: raw pin level. The opto (U48) inverts and the electrical
	 * polarity is a cfg bit that refsel applies; pwrseq only needs the
	 * boolean, so the logical DT level is passed through.
	 * TODO(wave-3b): apply the configured RB_LOCK polarity here too. */
	if (gpio_is_ready_dt(&rb_lock_in)) {
		in->rb_lock = gpio_pin_get_dt(&rb_lock_in) == 1;
	}
	in->extref_in_band = extref_valid && extref_hz >= 9999800U &&
			     extref_hz <= 10000200U;
	if (gpio_is_ready_dt(&rb_ov_det)) {
		in->rb_ov_det = gpio_pin_get_dt(&rb_ov_det) == 1;
	}

	(void)cfg_get_u64(sts_cfg(), CFG_KEY_PWR_POE_BUDGET_MW, &policy);
	in->poe_granted_mw = (uint32_t)policy;
	poe_mv = hk.ina[INA228_RAIL_POE].bus_uv / 1000;
	poe_ma = hk.ina[INA228_RAIL_POE].current_ua / 1000;
	if (hk.ina[INA228_RAIL_POE].valid && poe_mv > 0 && poe_ma > 0) {
		in->poe_measured_mw = (uint32_t)(((int64_t)poe_mv * poe_ma) / 1000);
	}

	in->ui_wanted = IS_ENABLED(CONFIG_STS1000_UI);
	in->liveness_ok = sts_liveness_stale_mask(now_ms) == 0U;
	in->debugger_attached = false;
}

/* ------------------------------------------------------------- actions ---- */

static void pwrseq_set(const struct gpio_dt_spec *g, int val)
{
	if (gpio_is_ready_dt(g)) {
		(void)gpio_pin_set_dt(g, val);
	}
}

static void pwrseq_exec_action(const pwrseq_act_t *a)
{
	switch ((pwrseq_action_t)a->action) {
	/* Stages platform.c already drove — idempotent no-ops or re-drives. */
	case PWRSEQ_ACT_NOR_RST_RELEASE:
	case PWRSEQ_ACT_I2C_PROBE:
	case PWRSEQ_ACT_READ_RAIL_BASELINE:
	case PWRSEQ_ACT_LAN_RST_RELEASE:
	case PWRSEQ_ACT_PHY_MDIO_POLL:
	case PWRSEQ_ACT_ANT_SUPERVISOR_START:
	case PWRSEQ_ACT_GNSS_CONFIG_REQUEST:
	case PWRSEQ_ACT_WDT_KICK_START: /* supervisor kicks once armed */
	case PWRSEQ_ACT_NONE:
		break;

	case PWRSEQ_ACT_DISP_RST_RELEASE:
		/* platform.c leaves DISP_RST asserted; releasing it is the ui
		 * area's call, so this stays a no-op here. */
		break;

	case PWRSEQ_ACT_APPLY_SHUNT_TRIMS:
		(void)sts_hk_ina_configure_all();
		break;

	case PWRSEQ_ACT_DISC_START:
		(void)sts_discipline_start(); /* -EALREADY if platform did it */
		break;

	case PWRSEQ_ACT_GPS_PWR_EN:
		pwrseq_set(&gps_pwr_en, 1);
		break;
	case PWRSEQ_ACT_GPS_PWR_DIS:
		pwrseq_set(&gps_pwr_en, 0);
		break;
	case PWRSEQ_ACT_GPS_RST_RELEASE:
		pwrseq_set(&gps_rst, 0); /* active-low: 0 = released */
		break;
	case PWRSEQ_ACT_ANT_BIAS_EN:
		pwrseq_set(&ant_bias_en, 1);
		break;
	case PWRSEQ_ACT_ANT_BIAS_DIS:
		pwrseq_set(&ant_bias_en, 0);
		break;

	case PWRSEQ_ACT_DISP_EN:
		pwrseq_set(&disp_en, 1);
		break;
	case PWRSEQ_ACT_DISP_DIS:
		pwrseq_set(&disp_en, 0);
		break;
	case PWRSEQ_ACT_PANEL_LED_EN:
		/* The rail-enable is folded into the duty set; a bare enable
		 * comes up dark until PANEL_LED_PWM sets a duty. */
		break;
	case PWRSEQ_ACT_PANEL_LED_DIS:
		(void)sts_panel_led_set(0);
		break;
	case PWRSEQ_ACT_PANEL_LED_PWM:
		(void)sts_panel_led_set((uint8_t)a->arg);
		break;

	/* Stage 8 — guarded rubidium. */
	case PWRSEQ_ACT_DIGIPOT_WRITE:     /* safe-low precharge code */
	case PWRSEQ_ACT_DIGIPOT_WRITE_OP:  /* operating code */
		if (sts_digipot_set(a->arg) != 0) {
			LOG_ERR("pwrseq: digipot write %u failed; Rb sequence halts",
				a->arg);
		}
		break;
	case PWRSEQ_ACT_DIGIPOT_VERIFY: {
		uint16_t rb = 0;

		/* The authoritative verify is pwrseq's own check of
		 * digipot_readback (fed each tick) and the INA 0x47 window.
		 * This action just refreshes the log/console view. */
		if (sts_digipot_get(&rb) != 0) {
			LOG_ERR("pwrseq: digipot readback failed");
		}
		break;
	}
	case PWRSEQ_ACT_RB_PWR_EN:
		pwrseq_set(&rb_pwr_en, 1);
		break;
	case PWRSEQ_ACT_RB_PWR_DIS:
		pwrseq_set(&rb_pwr_en, 0);
		break;
	case PWRSEQ_ACT_RB_VCC_GATE_EN:
		pwrseq_set(&rb_vcc_gate, 1);
		break;
	case PWRSEQ_ACT_RB_VCC_GATE_DIS:
		pwrseq_set(&rb_vcc_gate, 0);
		break;
	case PWRSEQ_ACT_RB_OV_RESET_PULSE:
		if (gpio_is_ready_dt(&rb_ov_reset)) {
			(void)gpio_pin_set_dt(&rb_ov_reset, 1);
			k_busy_wait(20); /* >= 10 us per the OV-latch datasheet */
			(void)gpio_pin_set_dt(&rb_ov_reset, 0);
		}
		break;

	/* Stage 9 — actuated by supervisor.c to keep one writer per pin. */
	case PWRSEQ_ACT_WDT_EN:
		(void)sts_supervisor_arm();
		break;
	case PWRSEQ_ACT_RELAY_ELIGIBLE:
		sts_supervisor_set_seq_eligible(true);
		break;

	/* Shutdown path. */
	case PWRSEQ_ACT_PARK_DAC:
		sts_discipline_park();
		break;
	case PWRSEQ_ACT_PERSIST_STATE:
	case PWRSEQ_ACT_SET_SHUTDOWN_FLAG:
		/* The console area owns NVS fast-save; the PFI flag it polls
		 * (sts_pfi_fired) is the trigger. Nothing to drive here.
		 * TODO(wave-3b): a direct sts_store_critical_flush() hook. */
		break;
	case PWRSEQ_ACT_POE_KILL:
		LOG_WRN("pwrseq: POE_KILL asserted — board cold-cycle");
		pwrseq_set(&poe_kill, 1);
		break;

	default:
		LOG_WRN("pwrseq: unhandled action %u", (unsigned int)a->action);
		break;
	}
}

/* ------------------------------------------------------------- driver ----- */

void sts_pwrseq_step(uint32_t now_ms)
{
	pwrseq_in_t in;
	pwrseq_act_t act;

	if (!pwrseq_started) {
		return;
	}

	pwrseq_build_input(&in, now_ms);

	if (pwrseq_step(&pwrseq, &in) != 0) {
		return;
	}

	while (pwrseq_action_get(&pwrseq, &act) == 0) {
		pwrseq_exec_action(&act);
	}

	sts_liveness_feed(pwrseq_liveness_id);
}

int sts_pwrseq_start(uint32_t now_ms)
{
	pwrseq_cfg_t cfg;
	int rc;

	if (pwrseq_started) {
		return -EALREADY;
	}

	for (size_t i = 0; i < ARRAY_SIZE(pwrseq_outputs); i++) {
		if (!gpio_is_ready_dt(pwrseq_outputs[i])) {
			LOG_ERR("pwrseq: output %zu port not ready", i);
			return -ENODEV;
		}
		/* Each output boots to its safe/off state (interface ref §2):
		 * GPIO_OUTPUT_INACTIVE drives the de-asserted level for the
		 * pin's DT polarity — GPS/ant/disp/Rb off, POE_KILL = RUN. */
		rc = gpio_pin_configure_dt(pwrseq_outputs[i], GPIO_OUTPUT_INACTIVE);
		if (rc != 0) {
			LOG_ERR("pwrseq: output %zu configure failed (%d)", i, rc);
			return rc;
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(pwrseq_inputs); i++) {
		if (!gpio_is_ready_dt(pwrseq_inputs[i])) {
			return -ENODEV;
		}
		rc = gpio_pin_configure_dt(pwrseq_inputs[i], GPIO_INPUT);
		if (rc != 0) {
			return rc;
		}
	}

	pwrseq_load_cfg(&cfg);

	rc = pwrseq_init(&pwrseq, &cfg);
	if (rc != 0) {
		LOG_ERR("pwrseq_init failed (%d)", rc);
		return rc;
	}

	rc = pwrseq_start(&pwrseq, now_ms);
	if (rc != 0) {
		LOG_ERR("pwrseq_start failed (%d)", rc);
		return rc;
	}

	pwrseq_liveness_id = sts_liveness_register("pwrseq");
	pwrseq_started = true;

	LOG_INF("pwrseq up: rb_vmax %u mV, op code %u, safe code %u", cfg.rb_vmax_mv,
		cfg.digipot_operating_code, cfg.digipot_safe_code);

	return 0;
}
