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
#include <zephyr/sys/atomic.h>

#include "zephyr/platform/platform.h"
#include "zephyr/platform/sts_rbguard.h"
#include "zephyr/sts_app.h"

#include "cfg/cfg.h"
#include "fault/fault.h"
#include "ina228/ina228.h"
#include "pwrseq/pwrseq.h"
#include "quality/quality.h"
#include "storage/sts_store.h"

LOG_MODULE_REGISTER(sts_pwrseq, CONFIG_STS1000_LOG_LEVEL);

/* Power-group cfg keys (cfg_schema.h, group 0x07). */
#define CFG_KEY_PWR_RB_POLICY    0x0701U
#define CFG_KEY_PWR_RB_VMAX_MV   0x0703U
#define CFG_KEY_PWR_POE_BUDGET_MW 0x0704U
#define CFG_KEY_PWR_RB_WARMUP_S  0x0705U

/*
 * The VCC_RB ceiling, the PG decode, the INA freshness conversion, the EXTREF
 * band and the RB_LOCK polarity all live in sts_rbguard.h — Zephyr-free, so the
 * envelope that stands between a config typo and a destroyed rubidium is
 * asserted by tests/host/test_rbguard.c rather than by inspection.
 */

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
		int32_t need = pwrseq_rb_expected_mv(&cfg->rb_xfer,
						     cfg->digipot_operating_code);
		sts_rb_vmax_t d;

		sts_rb_vmax_decide(v, cfg->rb_vmax_mv, need, &d);

		if (d.clamped) {
			LOG_WRN("pwr.rb.vmax.mv %llu mV exceeds the %u mV hardware "
				"ceiling; clamped",
				(unsigned long long)v, STS_RB_VMAX_MV_CEILING);
		}
		/*
		 * A ceiling below the fixed operating setpoint would make
		 * pwrseq_init() reject the whole configuration — and a sequencer
		 * that never starts means no GPS, no display, no watchdog and no
		 * relay, i.e. a config typo would brick the box far beyond the
		 * rubidium. sts_rb_vmax_decide() refuses the value instead and
		 * hands back the default.
		 */
		if (d.refused) {
			LOG_ERR("pwr.rb.vmax.mv %llu mV is below the %d mV operating "
				"setpoint; keeping the %u mV default",
				(unsigned long long)v, need, d.vmax_mv);
		}

		cfg->rb_vmax_mv = d.vmax_mv;
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

/*
 * One locked read of the debounced bitmap, decoded outside the lock. Both
 * consumers used to take the fault lock separately and call fault_asserted()
 * ten times between them; one snapshot is both cheaper and self-consistent —
 * the rails and the supercaps now describe the same instant.
 */
static uint32_t pwrseq_fault_snapshot(void)
{
	uint32_t asserted;

	sts_fault_lock();
	asserted = fault_state(sts_fault());
	sts_fault_unlock();

	return asserted;
}

/*
 * Defined below, next to the operator-request handlers it dispatches, but
 * called from the tick above them. Without this declaration C assumes a
 * returns-int function and the definition then conflicts with it — which is a
 * hard error here rather than a warning, and the right place to fix it is the
 * declaration, not the definition's type.
 */
static bool pwrseq_service_requests(uint32_t now_ms);

static void pwrseq_build_input(pwrseq_in_t *in, uint32_t now_ms)
{
	sts_hk_snapshot_t hk;
	quality_block_t q;
	uint32_t faults;
	uint16_t dp = 0;
	uint32_t extref_hz = 0;
	bool extref_valid = false;
	bool extref_edges = false;
	uint64_t policy = 1;

	memset(in, 0, sizeof(*in));
	in->mono_ms = now_ms;

	faults = pwrseq_fault_snapshot();

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
	in->gnss_cfg_ack = sts_gnss_cfg_ack();

	for (size_t r = 0; r < INA228_RAIL_COUNT; r++) {
		in->ina_valid[r] = hk.ina[r].valid && hk.ina[r].cal_ok;
		in->ina_vbus_mv[r] = hk.ina[r].bus_uv / 1000;
		in->ina_current_ma[r] = hk.ina[r].current_ua / 1000;
		/*
		 * sts_ina_reading_t::age_ms is the *timestamp* of the reading, so
		 * the age is the difference. Without this the rubidium rail windows
		 * judged whatever happened to be in the cache, which on a 1 Hz sweep
		 * against a 250 ms tick meant a reading up to a second older than
		 * the rail change it was supposed to observe.
		 */
		in->ina_age_ms[r] = sts_rb_ina_age_ms(hk.ina[r].valid, now_ms,
						      hk.ina[r].age_ms);
	}
	in->pg_mask = sts_rb_pg_mask(faults);

	in->ocxo_warm = (q.flags & QUALITY_FLAG_OCXO_WARM) != 0U;
	in->ocxo_temp_stable = (q.flags & QUALITY_FLAG_OSC_TEMP_VALID) != 0U;

	(void)cfg_get_u64(sts_cfg(), CFG_KEY_PWR_RB_POLICY, &policy);
	in->rb_wanted = (policy != 0U);
	in->supercaps_charged = sts_rb_supercaps_charged(faults);

	if (sts_digipot_get(&dp) == 0) {
		in->digipot_readback = dp;
		in->digipot_readback_valid = true;
	}

	in->rb_lock = sts_pwrseq_rb_lock();
	in->extref_in_band = sts_rb_extref_in_band(extref_hz, extref_valid);
	if (gpio_is_ready_dt(&rb_ov_det)) {
		in->rb_ov_det = gpio_pin_get_dt(&rb_ov_det) == 1;
	}

	(void)cfg_get_u64(sts_cfg(), CFG_KEY_PWR_POE_BUDGET_MW, &policy);
	in->poe_granted_mw = (uint32_t)policy;
	in->poe_measured_mw = sts_rb_poe_mw(hk.ina[INA228_RAIL_POE].valid,
					    hk.ina[INA228_RAIL_POE].bus_uv,
					    hk.ina[INA228_RAIL_POE].current_ua);

	in->ui_wanted = IS_ENABLED(CONFIG_STS1000_UI);
	in->liveness_ok = sts_liveness_stale_mask(now_ms) == 0U;
	/*
	 * external_wdt §7.4: the TPS3430 cannot freeze on a core halt, so the
	 * watchdog must stay disarmed under a debugger or the first breakpoint
	 * cold-cycles the board. CoreDebug DHCSR C_DEBUGEN is the only reliable
	 * "a probe has claimed this core" bit on a Cortex-M33 and it is readable
	 * from software.
	 */
	in->debugger_attached = (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U;

	/* Rungs 2 and 3 of the thermal ladder — the actuator side of spec §13. */
	sts_hk_thermal_requests(&in->thermal_shed_rb, &in->thermal_poe_kill);
}

/* ------------------------------------------------------------- actions ---- */

static void pwrseq_set(const struct gpio_dt_spec *g, int val)
{
	if (gpio_is_ready_dt(g)) {
		(void)gpio_pin_set_dt(g, val);
	}
}

/*
 * Signals whose asserted level is *expected* while a load is deliberately off.
 *
 * fault_set_expected_off() had no caller outside its own tests, so every
 * commanded load-off raised a real fault. That is not cosmetic: the GNSS LDO's
 * power-good (FAULT_SIG_PG_3V3_GPS_LDO) is in FAULT_RELAY_DISQUALIFY_DEFAULT, so
 * the stage-5 GPS-off failure path released K2 and reddened the status RGB for a
 * condition firmware had just commanded. Each entry is (load-off action, the
 * signals that then legitimately read asserted).
 */
static void pwrseq_expect_off(bool off, const fault_sig_t *sigs, size_t n)
{
	sts_fault_lock();
	for (size_t i = 0; i < n; i++) {
		(void)fault_set_expected_off(sts_fault(), sigs[i], off);
	}
	sts_fault_unlock();
}

/* GPS rail: the LDO power-good plus its own INA228 alert. */
static const fault_sig_t sig_gps[] = {
	FAULT_SIG_PG_3V3_GPS_LDO,
	FAULT_SIG_INA_ALERT_3V3_GPS,
};
/* Antenna bias: the RT9742 nFLG and the antenna-rail alert. An unbiased antenna
 * rail reads as an open, which is the definition of "expected" here. */
static const fault_sig_t sig_ant[] = {
	FAULT_SIG_V_ANT_EN_FAULT,
	FAULT_SIG_INA_ALERT_V_ANT,
};
/* Display 5 V: load-switch nFLG and the 5V_DISP alert. */
static const fault_sig_t sig_disp[] = {
	FAULT_SIG_V_DISP_EN_FAULT,
	FAULT_SIG_INA_ALERT_5V_DISP,
};
/* Panel LEDs: load-switch nFLG and the panel-rail alert. */
static const fault_sig_t sig_panel[] = {
	FAULT_SIG_PANEL_LED_FAULT,
	FAULT_SIG_INA_ALERT_PANEL,
};
/* Rubidium: the MIC28516 power-good and the VCC_RB alert. Both are excluded from
 * the relay-disqualify set already, but they still hold fault_any_active() true
 * and so keep the status RGB red on a healthy OCXO-only unit. */
static const fault_sig_t sig_rb[] = {
	FAULT_SIG_PG_RB_PSU,
	FAULT_SIG_INA_ALERT_VCC_RB,
};

#define EXPECT_OFF(off, arr) pwrseq_expect_off((off), (arr), ARRAY_SIZE(arr))

static void pwrseq_exec_action(const pwrseq_act_t *a)
{
	switch ((pwrseq_action_t)a->action) {
	/* Stages platform.c already drove — idempotent no-ops or re-drives. */
	case PWRSEQ_ACT_NOR_RST_RELEASE:
	case PWRSEQ_ACT_I2C_PROBE:
	case PWRSEQ_ACT_READ_RAIL_BASELINE:
	case PWRSEQ_ACT_LAN_RST_RELEASE:
	case PWRSEQ_ACT_PHY_MDIO_POLL:
	case PWRSEQ_ACT_NONE:
		break;

	case PWRSEQ_ACT_ANT_SUPERVISOR_START:
		sts_gnss_ant_supervisor_start();
		break;
	case PWRSEQ_ACT_GNSS_CONFIG_REQUEST:
		/* Interface ref §2 step 5.5. Idempotent: gnssmgr restarts the walk
		 * from its first step, which is what a retry of this row wants. */
		(void)sts_gnss_configure(k_uptime_get_32());
		break;
	case PWRSEQ_ACT_WDT_KICK_START:
		sts_supervisor_kick_start(k_uptime_get_32());
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
		EXPECT_OFF(false, sig_gps);
		pwrseq_set(&gps_pwr_en, 1);
		break;
	case PWRSEQ_ACT_GPS_PWR_DIS:
		pwrseq_set(&gps_pwr_en, 0);
		EXPECT_OFF(true, sig_gps);
		break;
	case PWRSEQ_ACT_GPS_RST_RELEASE:
		pwrseq_set(&gps_rst, 0); /* active-low: 0 = released */
		/* Its configuration is gone with the reset; gnssmgr has to know
		 * before it starts believing NAV messages from the new session. */
		(void)sts_gnss_notify_reset(k_uptime_get_32());
		break;
	case PWRSEQ_ACT_ANT_BIAS_EN:
		EXPECT_OFF(false, sig_ant);
		pwrseq_set(&ant_bias_en, 1);
		break;
	case PWRSEQ_ACT_ANT_BIAS_DIS:
		pwrseq_set(&ant_bias_en, 0);
		EXPECT_OFF(true, sig_ant);
		break;

	case PWRSEQ_ACT_DISP_EN:
		EXPECT_OFF(false, sig_disp);
		pwrseq_set(&disp_en, 1);
		break;
	case PWRSEQ_ACT_DISP_DIS:
		pwrseq_set(&disp_en, 0);
		EXPECT_OFF(true, sig_disp);
		break;
	case PWRSEQ_ACT_PANEL_LED_EN:
		/* The rail-enable is folded into the duty set; a bare enable
		 * comes up dark until PANEL_LED_PWM sets a duty. */
		EXPECT_OFF(false, sig_panel);
		break;
	case PWRSEQ_ACT_PANEL_LED_DIS:
		(void)sts_panel_led_set(0);
		EXPECT_OFF(true, sig_panel);
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
		/*
		 * Ask housekeeping to re-read VCC_RB now. The window steps that
		 * follow are judged against this cache, and its unconditional
		 * sweep is only 1 Hz — a step that has to decide inside its own
		 * timeout cannot wait for that and must not judge the reading that
		 * predates this write.
		 */
		sts_hk_request_ina((uint8_t)INA228_RAIL_VCC_RB);
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
		EXPECT_OFF(false, sig_rb);
		pwrseq_set(&rb_pwr_en, 1);
		/* Same reason as the digipot writes: step 8.13 verifies this rail
		 * inside its own window and needs a reading taken after the buck
		 * came up, not before. */
		sts_hk_request_ina((uint8_t)INA228_RAIL_VCC_RB);
		break;
	case PWRSEQ_ACT_RB_PWR_DIS:
		pwrseq_set(&rb_pwr_en, 0);
		EXPECT_OFF(true, sig_rb);
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
		/* power_fail_input §5 item 2. The PFI ISR already scheduled this on
		 * the system work queue (which is cooperative, so it runs ahead of
		 * every application thread); re-requesting it is idempotent and
		 * covers a park list run for any other reason. */
		sts_store_critical_flush_from_isr();
		break;
	case PWRSEQ_ACT_SET_SHUTDOWN_FLAG:
		/*
		 * power_fail_input §5 item 4. Records *why* the record was written,
		 * which is what lets the next boot tell an orderly line drop from a
		 * crash — previously nothing emitted this at all, so every boot
		 * looked like a crash.
		 */
		(void)sts_store_critical_flush(STS_CRITICAL_REASON_SHUTDOWN);
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

static void pwrseq_drain(void)
{
	pwrseq_act_t act;

	while (pwrseq_action_get(&pwrseq, &act) == 0) {
		pwrseq_exec_action(&act);
	}
}

void sts_pwrseq_step(uint32_t now_ms)
{
	pwrseq_in_t in;
	bool operator_acted;

	if (!pwrseq_started) {
		return;
	}

	/* Before the step, so the stage machine's own input already reflects a
	 * cleared OV latch or a re-entered stage 8 rather than seeing it a tick
	 * late. */
	operator_acted = pwrseq_service_requests(now_ms);

	pwrseq_build_input(&in, now_ms);

	if (pwrseq_step(&pwrseq, &in) != 0) {
		/* The sequencer refused its input, but an operator action has
		 * already queued pin work — a POE_KILL that sat undrained here
		 * would be a stop button that did nothing. */
		if (operator_acted) {
			pwrseq_drain();
		}
		return;
	}

	pwrseq_drain();

	sts_liveness_feed(pwrseq_liveness_id);
}

void sts_pwrseq_pfi(uint32_t now_ms)
{
	if (!pwrseq_started) {
		return;
	}

	/*
	 * Housekeeping-thread context, so nothing can be mid-drain: this is the
	 * mutual exclusion pwrseq.h §pwrseq_pfi requires and that an ISR-side call
	 * would break. Drained synchronously right here, because the whole point of
	 * the park list is that it completes inside the hold-up window rather than
	 * waiting for the next 250 ms tick.
	 */
	(void)pwrseq_pfi(&pwrseq, now_ms);
	pwrseq_drain();

	if (pwrseq_actions_dropped(&pwrseq) != 0U) {
		LOG_ERR("pwrseq: %u actions were dropped before the power fail; "
			"the board state may not match the sequencer's model",
			pwrseq_actions_dropped(&pwrseq));
	}
}

void sts_pwrseq_rb_quiesce_from_isr(void)
{
	/*
	 * Load side first, then the supply — the same order as pwrseq's own
	 * rb_shutdown(). Two GPIO register writes, no locks and no logging, so it
	 * is safe from the PFI EXTI8 handler; idempotent, so the park list
	 * re-emitting them later costs nothing.
	 */
	pwrseq_set(&rb_vcc_gate, 0);
	pwrseq_set(&rb_pwr_en, 0);
}

/* ------------------------------------------------- operator requests ------ */

/*
 * Three recovery actions reach core/pwrseq from outside the housekeeping
 * thread, and none of them may call into it directly.
 *
 * pwrseq_rb_retry(), pwrseq_ov_clear() and pwrseq_poe_kill() all end in emit(),
 * which writes the action ring's head and length. pwrseq_drain() reads the same
 * ring and then EXECUTES each action, and executing is not instantaneous — a
 * digipot write is an SPI transfer and the OV reset pulse busy-waits. A console
 * or MP thread that called any of the three would therefore be appending to a
 * queue whose consumer is mid-drain, with no lock anywhere: exactly the mutual
 * exclusion this file already states for pwrseq_pfi(), which is documented as
 * safe only *because* it runs in housekeeping context.
 *
 * So the entry points post a bit and this function, called from the sequencer
 * pass, performs the action on the owning thread. The cost is up to one 250 ms
 * tick of latency, which is invisible for a latch acknowledgement and actively
 * wanted for POE_KILL — the caller's reply has to reach the wire before the
 * board loses power.
 *
 * A single atomic word rather than a queue: each bit is idempotent (retrying a
 * retry, clearing a cleared latch, killing an already-killed rail are all
 * no-ops) so coalescing two requests into one is the correct behaviour, not a
 * lost message.
 */
#define PWRSEQ_REQ_RB_RETRY BIT(0)
#define PWRSEQ_REQ_OV_CLEAR BIT(1)
#define PWRSEQ_REQ_POE_KILL BIT(2)

static atomic_t pwrseq_req;

/** Claim one request bit. -EBUSY when the same one is already pending. */
static int pwrseq_req_post(atomic_val_t bit)
{
	if (!pwrseq_started) {
		return -ENODEV;
	}
	if ((atomic_or(&pwrseq_req, bit) & bit) != 0) {
		return -EBUSY;
	}
	return 0;
}

/**
 * Apply every pending operator request. Housekeeping-thread context only.
 *
 * @return true when at least one request produced actions, so the caller knows
 *         the queue must be drained even if pwrseq_step() bailed.
 */
static bool pwrseq_service_requests(uint32_t now_ms)
{
	atomic_val_t req = atomic_clear(&pwrseq_req);
	bool acted = false;
	int rc;

	if (req == 0) {
		return false;
	}

	if ((req & PWRSEQ_REQ_OV_CLEAR) != 0) {
		rc = pwrseq_ov_clear(&pwrseq, now_ms);
		sts_log(LOGR_SUB_PWR, (rc == 0) ? LOGR_NOTICE : LOGR_WARN,
			"operator cleared the Rb over-voltage latch (rc %d)", rc);
		acted = acted || (rc == 0);
	}

	if ((req & PWRSEQ_REQ_RB_RETRY) != 0) {
		rc = pwrseq_rb_retry(&pwrseq, now_ms);
		sts_log(LOGR_SUB_PWR, (rc == 0) ? LOGR_NOTICE : LOGR_WARN,
			"operator retried the guarded rubidium sequence (rc %d)",
			rc);
		acted = acted || (rc == 0);
	}

	if ((req & PWRSEQ_REQ_POE_KILL) != 0) {
		rc = pwrseq_poe_kill(&pwrseq, PWRSEQ_POE_KILL_MAGIC, now_ms);
		sts_log(LOGR_SUB_PWR, LOGR_CRIT,
			"operator asserted POE_KILL: board cold-cycle (rc %d)",
			rc);
		acted = acted || (rc == 0);
	}

	return acted;
}

int sts_pwrseq_ov_clear(void)
{
	return pwrseq_req_post(PWRSEQ_REQ_OV_CLEAR);
}

int sts_pwrseq_rb_retry(void)
{
	return pwrseq_req_post(PWRSEQ_REQ_RB_RETRY);
}

int sts_pwrseq_poe_kill(void)
{
	return pwrseq_req_post(PWRSEQ_REQ_POE_KILL);
}

bool sts_pwrseq_rb_lock(void)
{
	int level;

	if (!gpio_is_ready_dt(&rb_lock_in)) {
		return false;
	}

	level = gpio_pin_get_dt(&rb_lock_in);

	/*
	 * The opto (U48) inverts: FE lock line high -> LED on -> transistor on ->
	 * RB_LOCK pulled low (rb_rs232_interface §6A.3). Which level means "locked"
	 * cannot be assumed for a surplus FE variant, so it is a firmware bit,
	 * commissioned per unit — not a hard-coded polarity.
	 */
	return sts_rb_lock_from_level(
		level, IS_ENABLED(CONFIG_STS1000_RB_LOCK_ACTIVE_LOW));
}

void sts_pwrseq_ant_bias_request(bool on)
{
	if (!pwrseq_started) {
		return;
	}

	/* Single writer per pin: the antenna supervisor decides, this file drives
	 * (ARCHITECTURE.md §10). Keep the expected-off mask in step with it. */
	if (on) {
		EXPECT_OFF(false, sig_ant);
		pwrseq_set(&ant_bias_en, 1);
	} else {
		pwrseq_set(&ant_bias_en, 0);
		EXPECT_OFF(true, sig_ant);
	}
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

	/*
	 * Every gated load boots off, so every one of their fault signals reads
	 * asserted before firmware has enabled anything. Marking them expected-off
	 * here is what keeps a cold board from coming up with a red status RGB and
	 * a released relay for rails nobody has switched on yet.
	 */
	EXPECT_OFF(true, sig_gps);
	EXPECT_OFF(true, sig_ant);
	EXPECT_OFF(true, sig_disp);
	EXPECT_OFF(true, sig_panel);
	EXPECT_OFF(true, sig_rb);

	pwrseq_load_cfg(&cfg);

	rc = pwrseq_init(&pwrseq, &cfg);
	if (rc != 0) {
		/*
		 * A rejected safety envelope must not leave the board with no
		 * sequencer: stages 5-9 (GPS, display, rubidium, watchdog, relay)
		 * all live here, so refusing to start would cost far more than the
		 * bad setting. Fall back to the documented defaults and say so.
		 */
		LOG_ERR("pwrseq_init rejected the configured envelope (%d); "
			"falling back to the built-in defaults",
			rc);
		pwrseq_cfg_default(&cfg);
		rc = pwrseq_init(&pwrseq, &cfg);
		if (rc != 0) {
			LOG_ERR("pwrseq_init failed on the defaults too (%d)", rc);
			return rc;
		}
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
