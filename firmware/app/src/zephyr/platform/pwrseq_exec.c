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

#include "console/mp_glue.h"
#include "zephyr/platform/platform.h"
#include "zephyr/platform/sts_pwrseq_pub.h"
#include "zephyr/platform/sts_pwrseq_req.h"
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
/*
 * REF_TERM_EN (PC10) — the external-reference SMA's 50 ohm termination.
 *
 * Deliberately NOT in pwrseq_outputs[] below: that loop drives every member
 * GPIO_OUTPUT_INACTIVE, and inactive here means UN-terminated. The board boots
 * terminated through R176's 100k pull-up and the software spec records that as
 * a settled decision, so this pin is configured ACTIVE at start and firmware's
 * automatic level is on.
 */
static const struct gpio_dt_spec ref_term_en = STS_USER_GPIO(ref_term_en_gpios);

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

/*
 * The configuration pwrseq_init() actually accepted.
 *
 * Kept because two things outside the stage machine need it and neither may
 * guess: the VCC_RB transfer function, which turns a maintenance setpoint in
 * millivolts into a wiper code, and `rb_vmax_mv`, which is the platform-side
 * bound on that code. Taken AFTER the init that succeeded, so it is the
 * envelope in force and not the one that was rejected — pwrseq_load_cfg() can
 * fall back to the built-in defaults and a stale copy would then let a setpoint
 * be computed against a ceiling the sequencer is not using.
 */
static pwrseq_cfg_t pwrseq_cfg;

/*
 * The U43 wiper, as two different facts that must not be confused.
 *
 * `dp_code` is what the PART holds: written through on every successful write
 * here and re-read from the device on every sequencer pass
 * (pwrseq_build_input), so it is an observation and it is what
 * sts_pwrseq_rb_expected_mv() reports. `dp_known` is false until one of those
 * has happened — before that the honest answer is "unknown", not "safe-low".
 *
 * `dp_auto_code` is what FIRMWARE last commanded, folded from the two digipot
 * actions, and it is what a released `pwr.rb.vset_mv` lease restores. They
 * differ exactly while an override holds the setpoint, which is the whole
 * reason both exist.
 */
static uint16_t dp_code;
static uint16_t dp_auto_code;
static bool dp_known;

/*
 * The cross-area view (sts_app.h sts_pwrseq_snap_t). Written only from the
 * housekeeping thread's sequencer pass, read lock-free by the console, MP, web,
 * SNMP and panel planes — see sts_pwrseq_pub.h for why it is a seqlock and not
 * a mutex.
 */
static sts_pwrseq_pub_t pwrseq_pub;

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

/* Same arrangement, same reason: the parameterised mailbox is defined with the
 * other operator-request plumbing but is drained from the tick above it. */
static void pwrseq_service_mailbox(uint32_t now_ms);

/* And the same again for the digipot observation, which is written from the
 * input build and the action executor above but is guarded by the mailbox
 * spinlock declared with the mailbox below. */
static void dp_observe(uint16_t code);

/*
 * @p out_extref_hz / @p out_extref_valid hand the raw EXTREF_MON measurement
 * back to the caller for publication. pwrseq_in_t carries only the in-band
 * verdict, which is all the stage machine decides with, but a technician needs
 * the number: "the sequencer refused the external reference" and "the external
 * reference reads 9.9994 MHz" are different sentences and only one of them is
 * actionable. Reading TIM12 a second time in the publish path would be a second
 * gate interval and a different instant, so it is threaded through instead.
 *
 * @p out_faults hands back the debounced fault bitmap for the same reason and
 * with a sharper edge on it. pwrseq_in_t keeps only what the stage machine
 * decides with — pg_mask and supercaps_charged, both derived from the bitmap by
 * sts_rb_pg_mask() / sts_rb_supercaps_charged() — and neither derivation is
 * invertible, so the 32-signal picture a technician needs is unrecoverable from
 * the input struct. Calling pwrseq_fault_snapshot() again in the publish path
 * would take the fault lock a second time for a word already in hand AND answer
 * from a later 1 kHz scan than the pg_mask published beside it, so the two
 * halves of one snapshot could disagree within a single tick.
 */
static void pwrseq_build_input(pwrseq_in_t *in, uint32_t now_ms,
			       uint32_t *out_extref_hz, bool *out_extref_valid,
			       uint32_t *out_faults)
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
	*out_faults = faults;

	(void)sts_hk_read(&hk);
	(void)sts_quality_snapshot(&q);
	sts_extref_mon_read(&extref_hz, &extref_valid, &extref_edges);
	*out_extref_hz = extref_hz;
	*out_extref_valid = extref_valid;

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
		/*
		 * The same read serves the `pwr.rb.vset_mv` read-back. Folding it
		 * in here rather than adding a second SPI transfer on the console
		 * path is not only cheaper: it makes the reported setpoint an
		 * observation of the PART refreshed at 4 Hz, so a write that did
		 * not take stops being reported as the value that was asked for.
		 */
		dp_observe(dp);
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

/* ------------------------------------------ the mailbox-controlled rails --- */

/*
 * One GPIO rail a maintenance lease may hold, indexed by sts_pwrseq_req_id_t.
 *
 * `auto_on` is THE SEQUENCER'S OWN COMMANDED LEVEL for the pin, and it is the
 * pivot the whole override policy turns on. It is folded from the same places
 * firmware drives the pin — pwrseq_exec_action() below and, for PC9,
 * sts_pwrseq_ant_bias_request() — and it is deliberately NOT updated when a
 * mailbox request moves the pin. That asymmetry is the design:
 *
 *   - an override may only raise a load firmware also wants raised, so a lease
 *     can never energise something the sequencer has shed, has not reached, or
 *     has taken down as a fail-action;
 *   - a RELEASE re-drives the pin to `auto_on`, i.e. to firmware-automatic
 *     control, rather than to off. A dead-man that left the GPS rail dead would
 *     have broken the box it exists to protect.
 *
 * `shed_at` is only for the refusal SENTENCE: shed and not-yet-reached both
 * leave `auto_on` false, and they send a technician to different places.
 *
 * THREADING. Every field is written and read on the housekeeping thread except
 * `auto_on` for the antenna row, which the priority-6 GNSS thread writes through
 * sts_pwrseq_ant_bias_request(). That is a single aligned bool on a single-core
 * M33 — it cannot tear — and the store is ordered before the atomic_or inside
 * sts_mp_veto(), so the console cannot learn of the supervisor's cut before the
 * level it must restore is visible. What remains is a genuine but bounded race:
 * a release already past its `auto_on` read when the supervisor latches a short
 * re-drives PC9 high. The RT9742's own current limit and nFLG are what bound
 * that, and PC9 already had these two writers before the mailbox existed.
 */
typedef struct {
	const struct gpio_dt_spec *gpio;
	const fault_sig_t *sigs;
	size_t n_sigs;
	uint8_t shed_at; /* pwrseq_shed_level_t; PWRSEQ_SHED_NONE = never shed */
	bool auto_on;
} pwrseq_rail_t;

static pwrseq_rail_t pwrseq_rails[STS_PWRSEQ_REQ_COUNT] = {
	[STS_PWRSEQ_REQ_GPS_EN] = { .gpio = &gps_pwr_en,
				    .sigs = sig_gps,
				    .n_sigs = ARRAY_SIZE(sig_gps),
				    .shed_at = (uint8_t)PWRSEQ_SHED_NONE },
	[STS_PWRSEQ_REQ_ANT_BIAS_EN] = { .gpio = &ant_bias_en,
					 .sigs = sig_ant,
					 .n_sigs = ARRAY_SIZE(sig_ant),
					 .shed_at = (uint8_t)PWRSEQ_SHED_NONE },
	[STS_PWRSEQ_REQ_DISP_EN] = { .gpio = &disp_en,
				     .sigs = sig_disp,
				     .n_sigs = ARRAY_SIZE(sig_disp),
				     .shed_at = (uint8_t)PWRSEQ_SHED_DISPLAY },
	/*
	 * RB_VCC_GATE only, never RB_PWR_EN. `pwr.rb.en` is deliberately absent
	 * from this table — see the note above sts_pwrseq_req_post().
	 *
	 * No expected-off signals: the gate connects a load to VCC_RB, it does
	 * not gate the rail, so FAULT_SIG_PG_RB_PSU and the 0x47 alert stay
	 * live and must keep reading. Suppressing them here would mask a buck
	 * fault for as long as a technician held the gate open.
	 */
	[STS_PWRSEQ_REQ_RB_GATE] = { .gpio = &rb_vcc_gate,
				     .sigs = NULL,
				     .n_sigs = 0U,
				     .shed_at = (uint8_t)PWRSEQ_SHED_RB },
	/*
	 * The one row whose `auto_on` starts TRUE — set in sts_pwrseq_start(),
	 * beside the configure that drives the pin, so the two cannot drift.
	 *
	 * No expected-off signals and PWRSEQ_SHED_NONE, and neither is an
	 * omission: a 50 ohm shunt across an unpowered SMA is not a load, so no
	 * rung of the ladder and no fail-action ever wants it gone, and there is
	 * no fault signal that reads asserted while it is off. Un-terminating is
	 * the direction a technician has to ask for; re-terminating is what a
	 * release does, and — being the automatic level — is never refused.
	 */
	[STS_PWRSEQ_REQ_REF_TERM_EN] = { .gpio = &ref_term_en,
					 .sigs = NULL,
					 .n_sigs = 0U,
					 .shed_at = (uint8_t)PWRSEQ_SHED_NONE },
};

/** Drive one rail and keep its expected-off mask in step. */
static void pwrseq_rail_drive(pwrseq_rail_t *r, bool on)
{
	/* Clearing the mask before energising and setting it after de-energising
	 * is the order pwrseq_exec_action() uses, and it is the one that leaves
	 * no window in which a real failure reads as expected. */
	if (on) {
		pwrseq_expect_off(false, r->sigs, r->n_sigs);
		pwrseq_set(r->gpio, 1);
		return;
	}
	pwrseq_set(r->gpio, 0);
	pwrseq_expect_off(true, r->sigs, r->n_sigs);
}

/** Fold firmware's own command into the rail's automatic level. */
static void pwrseq_rail_auto(uint8_t req, bool on)
{
	pwrseq_rails[req].auto_on = on;
}

static void pwrseq_exec_action(const pwrseq_act_t *a)
{
	sts_mp_veto_t subj;

	/*
	 * The firmware veto (FMT §5.1.2, §5.4), raised BEFORE the pin moves.
	 *
	 * This switch is the single choke point for every rail an override can
	 * hold, so it is the one place that knows firmware is about to act on an
	 * object a maintenance lease may be holding the other way. An override
	 * cannot be honoured through a shed, a latched over-voltage or a
	 * stage fail-action: the hardware has already acted or is about to, and
	 * a lease that stands until its keepalive lapses is a lie about the
	 * board on the one channel a technician trusts.
	 *
	 * pwrseq_ov_latched() is read here rather than folded into the action
	 * because the same two actions carry both stories — the autonomous 26 V
	 * latch killed the buck, or the sequencer took the rail down — and only
	 * this side knows which. The mapping itself is
	 * console/sts_mp_veto_policy.h, which also states what is deliberately
	 * NOT mapped (the ON direction, the VCC_RB setpoint objects) and why.
	 *
	 * Deferred, never synchronous: sts_mp_veto() is one atomic OR and the
	 * console supervisor does the reverting inside the engine lock it
	 * already holds. This is the housekeeping thread and the action it is
	 * about to run may be a busy-wait or an SPI transfer; queueing it behind
	 * a maintenance request would be the priority inversion the console area
	 * is built to prevent, and on the PFI park list it would miss the
	 * hold-up window outright.
	 */
	subj = sts_mp_veto_of_action((uint16_t)a->action,
				     pwrseq_ov_latched(&pwrseq));
	if (subj != STS_MP_VETO_NONE) {
		sts_mp_veto(subj);
	}

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

	/*
	 * Each rail an override can hold is driven through pwrseq_rail_drive()
	 * and records firmware's commanded level as it goes, so the mailbox
	 * drain and this executor cannot drift apart. The pin work is identical
	 * to what these cases did before; what is new is the one-line record.
	 */
	case PWRSEQ_ACT_GPS_PWR_EN:
		pwrseq_rail_auto((uint8_t)STS_PWRSEQ_REQ_GPS_EN, true);
		pwrseq_rail_drive(&pwrseq_rails[STS_PWRSEQ_REQ_GPS_EN], true);
		break;
	case PWRSEQ_ACT_GPS_PWR_DIS:
		pwrseq_rail_auto((uint8_t)STS_PWRSEQ_REQ_GPS_EN, false);
		pwrseq_rail_drive(&pwrseq_rails[STS_PWRSEQ_REQ_GPS_EN], false);
		break;
	case PWRSEQ_ACT_GPS_RST_RELEASE:
		pwrseq_set(&gps_rst, 0); /* active-low: 0 = released */
		/* Its configuration is gone with the reset; gnssmgr has to know
		 * before it starts believing NAV messages from the new session. */
		(void)sts_gnss_notify_reset(k_uptime_get_32());
		break;
	case PWRSEQ_ACT_ANT_BIAS_EN:
		pwrseq_rail_auto((uint8_t)STS_PWRSEQ_REQ_ANT_BIAS_EN, true);
		pwrseq_rail_drive(&pwrseq_rails[STS_PWRSEQ_REQ_ANT_BIAS_EN],
				  true);
		break;
	case PWRSEQ_ACT_ANT_BIAS_DIS:
		pwrseq_rail_auto((uint8_t)STS_PWRSEQ_REQ_ANT_BIAS_EN, false);
		pwrseq_rail_drive(&pwrseq_rails[STS_PWRSEQ_REQ_ANT_BIAS_EN],
				  false);
		break;

	case PWRSEQ_ACT_DISP_EN:
		pwrseq_rail_auto((uint8_t)STS_PWRSEQ_REQ_DISP_EN, true);
		pwrseq_rail_drive(&pwrseq_rails[STS_PWRSEQ_REQ_DISP_EN], true);
		break;
	case PWRSEQ_ACT_DISP_DIS:
		pwrseq_rail_auto((uint8_t)STS_PWRSEQ_REQ_DISP_EN, false);
		pwrseq_rail_drive(&pwrseq_rails[STS_PWRSEQ_REQ_DISP_EN], false);
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
		/*
		 * Firmware's intent is recorded whether or not the part took it:
		 * this is what a released `pwr.rb.vset_mv` lease restores, and
		 * "the code the sequencer wants" is still that code after a
		 * failed SPI transfer. `dp_code`, which reports what the part
		 * HOLDS, is only advanced on success.
		 */
		dp_auto_code = a->arg;
		if (sts_digipot_set(a->arg) != 0) {
			LOG_ERR("pwrseq: digipot write %u failed; Rb sequence halts",
				a->arg);
		} else {
			dp_observe(a->arg);
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
		pwrseq_rail_auto((uint8_t)STS_PWRSEQ_REQ_RB_GATE, true);
		pwrseq_rail_drive(&pwrseq_rails[STS_PWRSEQ_REQ_RB_GATE], true);
		break;
	case PWRSEQ_ACT_RB_VCC_GATE_DIS:
		pwrseq_rail_auto((uint8_t)STS_PWRSEQ_REQ_RB_GATE, false);
		pwrseq_rail_drive(&pwrseq_rails[STS_PWRSEQ_REQ_RB_GATE], false);
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

/*
 * Refresh the cross-area view. Housekeeping-thread context only — it is the
 * seqlock's single writer.
 *
 * Called AFTER pwrseq_drain(), so the published pin states are the ones the
 * board is actually in rather than the ones the sequencer has just decided to
 * command. Called on the refused-input path too: a sequencer that is rejecting
 * its own observations is exactly when an operator needs to see its stage, and
 * publishing only on the happy path would freeze the view at the last good tick
 * with nothing saying so.
 *
 * Publishing after the drain does not stale @p scan_state: the actions this
 * file executes reach GPIO outputs, the digipot and the expected-off mask, and
 * none of them writes fault_ctx_t::stable — only the io_scan thread does. So the
 * bitmap taken at build-input time is still the observation this tick decided
 * from, which is the one worth publishing. Re-reading it here would be a
 * *different*, later scan sitting beside a pg_mask derived from the earlier one.
 *
 * @p in may be NULL at start, before any observation exists; the observed half
 * then stays zero and `tick_mono_ms` stays 0, which reads as "no tick yet".
 */
static void pwrseq_publish(const pwrseq_in_t *in, uint32_t extref_hz,
			   bool extref_valid, uint32_t scan_state)
{
	sts_pwrseq_snap_t s;
	pwrseq_status_t st;

	if (pwrseq_status(&pwrseq, &st) != 0) {
		return;
	}

	sts_pwrseq_snap_from_status(&s, &st);
	sts_pwrseq_snap_observe(&s, in, extref_hz, extref_valid, scan_state);
	sts_pwrseq_pub_publish(&pwrseq_pub, &s);
}

int sts_pwrseq_snapshot(sts_pwrseq_snap_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}
	if (!pwrseq_started) {
		memset(out, 0, sizeof(*out));
		return -ENODEV;
	}

	return sts_pwrseq_pub_read(&pwrseq_pub, out);
}

void sts_pwrseq_step(uint32_t now_ms)
{
	pwrseq_in_t in;
	uint32_t extref_hz = 0;
	bool extref_valid = false;
	uint32_t faults = 0;
	bool operator_acted;

	if (!pwrseq_started) {
		return;
	}

	/* Before the step, so the stage machine's own input already reflects a
	 * cleared OV latch or a re-entered stage 8 rather than seeing it a tick
	 * late. */
	operator_acted = pwrseq_service_requests(now_ms);

	pwrseq_build_input(&in, now_ms, &extref_hz, &extref_valid, &faults);

	if (pwrseq_step(&pwrseq, &in) != 0) {
		/* The sequencer refused its input, but an operator action has
		 * already queued pin work — a POE_KILL that sat undrained here
		 * would be a stop button that did nothing. */
		if (operator_acted) {
			pwrseq_drain();
		}
		pwrseq_publish(&in, extref_hz, extref_valid, faults);
		pwrseq_service_mailbox(now_ms);
		return;
	}

	pwrseq_drain();
	pwrseq_publish(&in, extref_hz, extref_valid, faults);

	/*
	 * Last, and on both exits. After pwrseq_drain() so a request cannot
	 * outrank this tick's shed or fail-action, and after the publish so the
	 * snapshot describes exactly what pwrseq_step() decided. On the bail
	 * path too: a mailbox that only drained when the sequencer accepted its
	 * input would strand an override's release — including the dead-man's —
	 * for as long as the input stayed bad.
	 */
	pwrseq_service_mailbox(now_ms);

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
 *
 * The PARAMETERISED mailbox below is the other half of the same idea, for the
 * requests that carry a value and therefore cannot be a bit. Both drain on this
 * one pass, so every rail still has exactly one writer.
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

/* ------------------------------------ the parameterised request mailbox --- */

/*
 * Storage and the lock. The queue mechanics, the collision rule and the whole
 * argument for a spinlock rather than a mutex or a seqlock are in
 * sts_pwrseq_req.h; this is the part that needs Zephyr.
 *
 * The lock is held only across the header's bounded struct copies — never
 * across sts_panel_led_set(), which is claim-execute-settle for the same reason
 * sts_mp_evq.h peeks, sends and only then discards.
 */
static sts_pwrseq_reqq_t pwrseq_mbox;
static struct k_spinlock pwrseq_mbox_lock;

/**
 * Record the wiper the part now holds. Any thread; two scalar stores.
 *
 * Under the mailbox spinlock purely so sts_pwrseq_rb_expected_mv(), which the
 * console thread calls, cannot observe `dp_known` true against a half-written
 * `dp_code`. It is not actuation and holds the lock across nothing else — the
 * discipline sts_pwrseq_req.h states is about the ACTUATION being outside the
 * lock, not about the lock being reserved for the queue.
 */
static void dp_observe(uint16_t code)
{
	k_spinlock_key_t key = k_spin_lock(&pwrseq_mbox_lock);

	dp_code = code;
	dp_known = true;
	k_spin_unlock(&pwrseq_mbox_lock, key);
}

/*
 * WHAT IS DELIBERATELY NOT A ROW HERE: `pwr.rb.en` (RB_PWR_EN, PB7).
 *
 * ARCHITECTURE.md §10 invariant 3 fixes the order the rubidium may be brought
 * up in — safe digipot code (verify readback) -> RB_PWR_EN -> INA 0x47 window
 * -> RB_VCC_GATE — and an override on this pin cannot honour it in either
 * direction:
 *
 *   ON   is only ever reachable when firmware also commands the rail on, which
 *        means RB_VCC_GATE is already asserted. Re-raising PB7 there energises
 *        the FE-5680A from a rail nothing has re-verified, with no precharge:
 *        precisely the "FE connected before the rail is proven" ordering the
 *        invariant exists to forbid. core/pwrseq refuses the same shortcut for
 *        itself — pwrseq_shed_restore() will not emit a bare RB_PWR_EN and says
 *        why: "a shed-and-restore cycle must not become a back door around it".
 *   OFF  drops the supply with the load still connected, inverting the
 *        gate-then-supply order rb_shutdown() and sts_pwrseq_rb_quiesce_from_isr()
 *        both document.
 *
 * And a lease has no honest way back: a release must return the pin to
 * firmware-automatic control, which for this pin means re-running the guarded
 * stage-8 sequence — pwrseq_rb_retry(), an OPERATOR action that clears the
 * rubidium alarms and the deferral. A dead-man revert may not do that on a
 * board whose technician has walked away.
 *
 * `pwr.rb.gate` is the half of the pair that respects the order, so it is the
 * one that is wired; the supported way to bring the rubidium back is the
 * existing retry request. The manifest keeps `pwr.rb.en` MP_OF_DEFERRED so a
 * host is told before it draws the control.
 */

int sts_pwrseq_req_post(uint8_t req, const int32_t *value)
{
	k_spinlock_key_t key;
	int rc;

	if (!sts_pwrseq_req_id_ok(req)) {
		return -EINVAL;
	}
	/*
	 * Refused rather than queued when the sequencer has not started: the
	 * drain lives on its pass, so a request posted now would sit in the
	 * mailbox with nothing to execute it and no outcome would ever come
	 * back. The caller can act on -ENODEV; it cannot act on silence.
	 */
	if (!pwrseq_started) {
		return -ENODEV;
	}

	key = k_spin_lock(&pwrseq_mbox_lock);
	rc = sts_pwrseq_reqq_post(&pwrseq_mbox, req, (value == NULL),
				  (value != NULL) ? *value : 0,
				  k_uptime_get_32());
	k_spin_unlock(&pwrseq_mbox_lock, key);

	return rc;
}

bool sts_pwrseq_req_take(uint8_t req, sts_pwrseq_req_result_t *out)
{
	k_spinlock_key_t key;
	bool got;

	key = k_spin_lock(&pwrseq_mbox_lock);
	got = sts_pwrseq_reqq_take(&pwrseq_mbox, req, out);
	k_spin_unlock(&pwrseq_mbox_lock, key);

	return got;
}

const char *sts_pwrseq_req_reason(uint8_t err)
{
	return sts_pwrseq_req_reason_of(err);
}

/*
 * The two halves of the SAME claim-execute-settle discipline the housekeeping
 * drain uses, exported for the rows whose executor is somewhere else.
 *
 * Both refuse every row sts_pwrseq_req_is_foreign() does not name, which is what
 * makes the single-writer rule structural rather than documentary: the
 * housekeeping drain skips exactly the foreign rows and these refuse exactly the
 * rest, so neither side can execute a row the other owns even by mistake. The
 * spinlock is held across the header's bounded copy and nothing else — the
 * actuation happens on the caller's own thread, between the two calls.
 */
bool sts_pwrseq_req_claim(uint8_t req, bool *release, int32_t *value)
{
	k_spinlock_key_t key;
	bool got;

	if (!sts_pwrseq_req_id_ok(req) || !sts_pwrseq_req_is_foreign(req) ||
	    !pwrseq_started) {
		return false;
	}

	key = k_spin_lock(&pwrseq_mbox_lock);
	got = sts_pwrseq_reqq_claim(&pwrseq_mbox, req, release, value);
	k_spin_unlock(&pwrseq_mbox_lock, key);

	return got;
}

int sts_pwrseq_req_settle(uint8_t req, bool applied, uint8_t err, int32_t value,
			  uint32_t now_ms)
{
	k_spinlock_key_t key;
	int rc;

	if (!sts_pwrseq_req_id_ok(req) || !sts_pwrseq_req_is_foreign(req) ||
	    !pwrseq_started) {
		return -EINVAL;
	}

	key = k_spin_lock(&pwrseq_mbox_lock);
	rc = sts_pwrseq_reqq_settle(&pwrseq_mbox, req,
				    applied ? (uint8_t)STS_PWRSEQ_REQ_OUT_APPLIED
					    : (uint8_t)STS_PWRSEQ_REQ_OUT_REFUSED,
				    err, value, now_ms);
	k_spin_unlock(&pwrseq_mbox_lock, key);

	if (!applied) {
		LOG_WRN("pwrseq mailbox: foreign request %u refused (%s)",
			(unsigned int)req, sts_pwrseq_req_reason_of(err));
	}

	return rc;
}

/**
 * Execute one claimed PANEL_LED_EN request. Housekeeping-thread context only.
 *
 * PANEL_LED_EN (PC0) is not in pwrseq_outputs[] — panel_pwm.c owns the pin, and
 * pwrseq reaches it by calling sts_panel_led_set() from PWRSEQ_ACT_PANEL_LED_*
 * exactly as this does. That is precisely why the request has to arrive here
 * rather than being written from the console thread: the two would otherwise
 * interleave, and worse, pwrseq raises STS_MP_VETO_PANEL_LED *before* it
 * executes PANEL_LED_DIS, so a console-thread write landing after the veto
 * drain would re-light a panel whose lease had already been withdrawn.
 *
 * @param out_err  Receives the refusal reason when this returns false.
 * @param out_val  Receives the value that actually reached the actuator.
 */
static bool pwrseq_mbox_apply_panel(bool release, int32_t value,
				    uint8_t *out_err, int32_t *out_val)
{
	pwrseq_status_t st;
	bool want_on = !release && (value != 0);
	uint8_t duty;

	*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_NONE;
	*out_val = 0;

	/*
	 * OFF and release are never refused. Taking a load DOWN cannot be
	 * contrary to firmware — it is the direction the shed ladder, the
	 * fail-actions and the dead-man all move in — so the fail-safe answer is
	 * always available, which is what makes a lapsed lease reliable.
	 */
	if (!want_on) {
		if (sts_panel_led_set(0) != 0) {
			*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_HW;
			return false;
		}
		return true;
	}

	if (pwrseq_status(&pwrseq, &st) != 0) {
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_STAGE;
		return false;
	}

	/*
	 * Firmware wins, and it says which of the two ways it is winning. Shed
	 * and not-yet-reached both leave panel_led_on false, but they are
	 * different sentences for a technician: one means the board is over
	 * budget or hot, the other means the sequencer has not got there.
	 */
	if (st.shed >= (uint8_t)PWRSEQ_SHED_PANEL_LED) {
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_SHED;
		return false;
	}
	if (!st.panel_led_on) {
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_STAGE;
		return false;
	}

	/*
	 * The rail enable and the dimmer share one setter, so "on" needs a duty.
	 * Whatever is already programmed, or full brightness when that is 0 —
	 * a rail commanded on that stayed dark would be a control reporting
	 * success while the panel showed nothing.
	 */
	duty = sts_panel_led_get();
	if (duty == 0U) {
		duty = 100U;
	}
	if (sts_panel_led_set(duty) != 0) {
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_HW;
		return false;
	}

	*out_val = 1;
	return true;
}

/**
 * Execute one claimed PANEL_LED_PWM request — the `ui.panel.duty` dimmer.
 *
 * Migrated onto the mailbox from a direct sts_panel_led_set() on the console
 * thread. Two things change for a host and both are improvements:
 *
 *   - the write is now up to one 250 ms sequencer pass late, and the reply says
 *     so through `verify_pending` instead of asserting an effect it had not
 *     had. That cost the object MP_OF_WRITE: `obj.set` has no field that could
 *     admit the delay, so it is refused rather than answered.
 *   - a duty that would re-light a SHED panel is now refused. Written from the
 *     console thread it simply landed, re-energising a rail the ladder had
 *     dropped for a power or thermal reason — the same defect
 *     `pwr.panel.led.en` was routed through this mailbox to close, on the very
 *     same setter.
 *
 * A release resolves to 0, matching PANEL_LED_EN's row and what this object
 * released to before — see STS_PWRSEQ_REQ_PANEL_DUTY in sts_app.h for why it is
 * the one row that does not restore firmware's configured level.
 */
static bool pwrseq_mbox_apply_duty(bool release, int32_t value, uint8_t *out_err,
				   int32_t *out_val)
{
	pwrseq_status_t st;
	uint8_t duty;

	*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_NONE;
	*out_val = 0;

	duty = release ? 0U : (uint8_t)CLAMP(value, 0, 100);

	/* Dark is the fail-safe direction and is never refused, exactly as it is
	 * for the rail enable this setter shares a pin pair with. */
	if (duty != 0U) {
		if (pwrseq_status(&pwrseq, &st) != 0) {
			*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_STAGE;
			return false;
		}
		if (st.shed >= (uint8_t)PWRSEQ_SHED_PANEL_LED) {
			*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_SHED;
			return false;
		}
		if (!st.panel_led_on) {
			*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_STAGE;
			return false;
		}
	}

	if (sts_panel_led_set(duty) != 0) {
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_HW;
		return false;
	}

	*out_val = (int32_t)duty;
	return true;
}

/**
 * Execute one claimed GPIO-rail request. Housekeeping-thread context only.
 *
 * The whole policy is three lines, and every one of them is stated in the
 * pwrseq_rail_t comment: ON only where firmware also wants it on, OFF never
 * refused, release back to firmware's own level.
 */
static bool pwrseq_mbox_apply_rail(uint8_t req, bool release, int32_t value,
				   uint8_t *out_err, int32_t *out_val)
{
	pwrseq_rail_t *r = &pwrseq_rails[req];
	pwrseq_status_t st;
	bool want;
	bool was;

	*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_NONE;
	*out_val = 0;

	if ((r->gpio == NULL) || !gpio_is_ready_dt(r->gpio)) {
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_HW;
		return false;
	}

	want = release ? r->auto_on : (value != 0);

	if (want && !r->auto_on) {
		/*
		 * Firmware wins, and it says which of the two ways it is winning.
		 * A shed load and a stage the sequencer has not reached both
		 * leave `auto_on` false and are different sentences at a bench.
		 */
		if ((r->shed_at != (uint8_t)PWRSEQ_SHED_NONE) &&
		    (pwrseq_status(&pwrseq, &st) == 0) &&
		    (st.shed >= r->shed_at)) {
			*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_SHED;
		} else {
			*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_STAGE;
		}
		return false;
	}

	was = (gpio_pin_get_dt(r->gpio) == 1);
	pwrseq_rail_drive(r, want);

	/*
	 * Re-powering the receiver is a reset it must be told about.
	 *
	 * PWRSEQ_ACT_GPS_RST_RELEASE says it for the sequencer's own path — "its
	 * configuration is gone with the reset; gnssmgr has to know before it
	 * starts believing NAV messages from the new session" — and a rail an
	 * override dropped and restored is the same event. Without it gnssmgr
	 * keeps its ACK bookkeeping and its survey state across a receiver that
	 * cold-booted, and the sequencer's `gnss_cfg_ack` input stays true for a
	 * configuration that no longer exists. Same call, same thread as the
	 * action executor makes it from.
	 */
	if (want && !was && (req == (uint8_t)STS_PWRSEQ_REQ_GPS_EN)) {
		(void)sts_gnss_notify_reset(k_uptime_get_32());
	}

	*out_val = want ? 1 : 0;
	return true;
}

/**
 * Execute one claimed VCC_RB setpoint request. Housekeeping-thread context only.
 *
 * THE TWO REFUSALS ARE THE REASON THIS OBJECT CAN BE OFFERED AT ALL. The root
 * CLAUDE.md is explicit that a digipot in this buck's feedback node can destroy
 * the FE-5680A, so a maintenance setpoint is permitted only where it cannot
 * reach the FE and cannot fight the sequencer:
 *
 *   stage 8    the guarded sequence owns the wiper. It writes the safe code,
 *              verifies the read-back, writes the operating code and judges the
 *              rail against ITS model of the commanded code; a setpoint landing
 *              in there fails that verify and abandons the stage.
 *   gated      the FE is connected. core/pwrseq's post-gate supervisor drops the
 *              whole rubidium chain the moment VCC_RB leaves the window implied
 *              by its own `rb_current_code` (pwrseq.c, rb_rail_in_window), so a
 *              setpoint change against a gated FE would not merely be a step on
 *              its supply — it would shed the rubidium and raise
 *              PWRSEQ_ALARM_RB_WINDOW as a consequence of a commanded action.
 *
 * What is left is exactly the bench window a technician wants: the buck up, the
 * FE disconnected, trim the rail and read it back on INA228 0x47.
 *
 * The value is bounded HERE as well as by MP_ILK_RB_VMAX in core. The interlock
 * clamps against a ceiling the console read from cfg; this clamps against the
 * ceiling the SEQUENCER is running, which sts_rb_vmax_decide() has already
 * bounded by STS_RB_VMAX_MV_CEILING. The platform does not delegate the last
 * bound on this rail to a number handed across an area seam.
 */
static bool pwrseq_mbox_apply_vset(bool release, int32_t value, uint8_t *out_err,
				   int32_t *out_val)
{
	pwrseq_status_t st;
	uint16_t code;

	*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_NONE;
	*out_val = 0;

	if (pwrseq_status(&pwrseq, &st) != 0) {
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_STAGE;
		return false;
	}

	if (release) {
		/* Firmware's own code, whatever it is — the release path is
		 * never refused, or a lapsed lease would strand the rail at a
		 * maintenance setpoint. */
		code = dp_auto_code;
	} else {
		if (st.stage == PWRSEQ_STAGE_8_RB) {
			*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_STAGE;
			return false;
		}
		if (st.rb_gated) {
			*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_GATED;
			return false;
		}
		code = sts_rb_code_for_mv(&pwrseq_cfg.rb_xfer, value,
					  pwrseq_cfg.rb_vmax_mv);
	}

	if (sts_digipot_set(code) != 0) {
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_HW;
		return false;
	}
	dp_observe(code);

	/*
	 * Ask housekeeping to re-read VCC_RB now, for the reason the sequencer's
	 * own digipot writes do: MP_ILK_RB_VERIFY judges this setpoint 250 ms
	 * from now against the sensor cache, whose unconditional sweep is 1 Hz.
	 * Without this the read-back could judge a reading taken before the
	 * write and revert a lease that worked.
	 */
	sts_hk_request_ina((uint8_t)INA228_RAIL_VCC_RB);

	*out_val = pwrseq_rb_expected_mv(&pwrseq_cfg.rb_xfer, code);
	return true;
}

/**
 * Execute one claimed FAN_DUTY request — `sys.fan.duty`. Housekeeping only.
 *
 * TWO OF THIS FILE'S CONVENTIONS ARE INVERTED HERE, and sts_app.h says why at
 * STS_PWRSEQ_REQ_FAN_DUTY: the fail-safe direction for a fan is UP, so the
 * override is a FLOOR that housekeeping re-applies as max(loop, override) on
 * every thermal step, and a RELEASE resolves to full airflow rather than to
 * firmware's last answer.
 *
 * Nothing is refused. There is no stage that owns the fan (it runs from
 * hk start, before the sequencer), no ladder rung that sheds it — rung 1 of the
 * thermal ladder IS "fan max" — and no direction of this control that can leave
 * the box hotter than the loop wants it. What can fail is the actuator, and
 * that is reported as ERR_HW like every other refused write.
 */
static bool pwrseq_mbox_apply_fan(bool release, int32_t value, uint8_t *out_err,
				  int32_t *out_val)
{
	sts_hk_fan_t f;
	uint8_t pct = release ? 0U : (uint8_t)CLAMP(value, 0, 100);
	int rc;

	*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_NONE;
	*out_val = 0;

	rc = sts_hk_fan_override(!release, pct);
	if (rc != 0) {
		*out_err = (uint8_t)((rc == -ENODEV)
					     ? STS_PWRSEQ_REQ_ERR_STAGE
					     : STS_PWRSEQ_REQ_ERR_HW);
		return false;
	}

	/* The DUTY THE PIN CARRIES, which after the floor is applied is not
	 * necessarily the one that was asked for — a request below the loop's
	 * current answer is granted and immediately outranked, and the reply has
	 * to say so or a host would read back its request. */
	(void)sts_hk_fan_state(&f);
	*out_val = (int32_t)f.out_pct;
	return true;
}

/**
 * Execute one claimed RGB_MODE request — `ui.rgb.mode`. Housekeeping only.
 *
 * A release IS STS_RGB_MODE_AUTO: the object's own enum names "hand D5 back to
 * core/fault's priority encode" as value 0, so the release direction and the
 * automatic direction are the same value rather than two ideas that could drift.
 * Never refused — an indicator pattern cannot be contrary to a shed or a stage,
 * and the supervisor keeps annunciating underneath a forced colour regardless.
 */
static bool pwrseq_mbox_apply_rgb_mode(bool release, int32_t value,
				       uint8_t *out_err, int32_t *out_val)
{
	uint8_t mode = release ? (uint8_t)STS_RGB_MODE_AUTO
			       : (uint8_t)CLAMP(value, 0, (int32_t)STS_RGB_MODE_COUNT - 1);

	*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_NONE;
	*out_val = 0;

	if (sts_supervisor_rgb_mode(mode) != 0) {
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_HW;
		return false;
	}

	/* The pattern D5 is now SHOWING, which for a release is whatever the
	 * fault policy resolved to and never STS_RGB_MODE_AUTO. */
	*out_val = (int32_t)sts_supervisor_rgb_mode_get();
	return true;
}

/**
 * Execute one claimed RGB leg request — `ui.rgb.r/g/b`. Housekeeping only.
 *
 * The legs compose ON TOP of whatever pattern is in force, so each is released
 * independently and a release returns that one leg to the pattern's own duty
 * without disturbing the other two or the pattern itself.
 */
static bool pwrseq_mbox_apply_rgb_leg(uint8_t req, bool release, int32_t value,
				      uint8_t *out_err, int32_t *out_val)
{
	uint8_t pct = release ? 0U : (uint8_t)CLAMP(value, 0, 100);
	uint8_t leg;

	*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_NONE;
	*out_val = 0;

	/* Spelled out rather than computed from the row index: two enums in two
	 * headers happening to run in the same order is not a contract, and
	 * getting it wrong turns red into blue on a fault indicator. */
	switch ((sts_pwrseq_req_id_t)req) {
	case STS_PWRSEQ_REQ_RGB_R:
		leg = (uint8_t)STS_RGB_LEG_R;
		break;
	case STS_PWRSEQ_REQ_RGB_G:
		leg = (uint8_t)STS_RGB_LEG_G;
		break;
	case STS_PWRSEQ_REQ_RGB_B:
		leg = (uint8_t)STS_RGB_LEG_B;
		break;
	default:
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_STAGE;
		return false;
	}

	if (sts_supervisor_rgb_leg(leg, !release, pct) != 0) {
		*out_err = (uint8_t)STS_PWRSEQ_REQ_ERR_HW;
		return false;
	}

	/* The duty actually in the compare register, so a released leg reports
	 * the pattern's own colour rather than the 0 the request carried. */
	*out_val = (int32_t)sts_supervisor_rgb_leg_get(leg);
	return true;
}

bool sts_pwrseq_rail_on(uint8_t req)
{
	const pwrseq_rail_t *r;

	if (!pwrseq_started || (req >= (uint8_t)STS_PWRSEQ_REQ_COUNT)) {
		return false;
	}

	r = &pwrseq_rails[req];
	if ((r->gpio == NULL) || !gpio_is_ready_dt(r->gpio)) {
		return false;
	}

	return gpio_pin_get_dt(r->gpio) == 1;
}

int32_t sts_pwrseq_rb_expected_mv(void)
{
	k_spinlock_key_t key;
	uint16_t code;
	bool known;

	if (!pwrseq_started) {
		return 0;
	}

	/* Under the mailbox lock so the console cannot read `known` true against
	 * a code the drain is mid-update on. Two scalars, no I/O. */
	key = k_spin_lock(&pwrseq_mbox_lock);
	code = dp_code;
	known = dp_known;
	k_spin_unlock(&pwrseq_mbox_lock, key);

	return known ? pwrseq_rb_expected_mv(&pwrseq_cfg.rb_xfer, code) : 0;
}

/**
 * Drain the mailbox. Housekeeping-thread context only, after pwrseq_drain().
 *
 * After, not before, and it is load-bearing: this pass's shed or fail-action has
 * already run, so an override asking for a load firmware has just taken down is
 * refused on this tick rather than granted and withdrawn on the next.
 */
static void pwrseq_service_mailbox(uint32_t now_ms)
{
	uint8_t req;

	for (req = (uint8_t)STS_PWRSEQ_REQ_NONE + 1U;
	     req < (uint8_t)STS_PWRSEQ_REQ_COUNT; req++) {
		k_spinlock_key_t key;
		uint8_t err = (uint8_t)STS_PWRSEQ_REQ_ERR_NONE;
		int32_t applied = 0;
		int32_t value = 0;
		bool release = false;
		bool ok = false;

		/*
		 * Rows another area's thread owns are not this drain's, and the
		 * skip is BEFORE the claim: claiming clears `posted`, so a claim
		 * here would consume a request the owning drain must still see
		 * and then settle it from a thread that never touched the pin.
		 * sts_pwrseq_req_is_foreign() is the single fact both sides read.
		 */
		if (sts_pwrseq_req_is_foreign(req)) {
			continue;
		}

		key = k_spin_lock(&pwrseq_mbox_lock);
		ok = sts_pwrseq_reqq_claim(&pwrseq_mbox, req, &release, &value);
		k_spin_unlock(&pwrseq_mbox_lock, key);

		if (!ok) {
			continue;
		}

		/* Outside the lock: this is where the pin actually moves. */
		switch ((sts_pwrseq_req_id_t)req) {
		case STS_PWRSEQ_REQ_PANEL_LED_EN:
			ok = pwrseq_mbox_apply_panel(release, value, &err,
						     &applied);
			break;
		case STS_PWRSEQ_REQ_PANEL_DUTY:
			ok = pwrseq_mbox_apply_duty(release, value, &err,
						    &applied);
			break;
		case STS_PWRSEQ_REQ_LAMP_TEST:
			/*
			 * The dimmer's executor with a boolean in front of it:
			 * a lamp test IS "the panel string at full brightness",
			 * so it inherits the shed and stage refusals rather than
			 * restating them — which is the whole point of routing
			 * sts_ui.c's UI_ACTION_LAMP_TEST through here.
			 */
			ok = pwrseq_mbox_apply_duty(release,
						    (value != 0) ? 100 : 0, &err,
						    &applied);
			break;
		case STS_PWRSEQ_REQ_GPS_EN:
		case STS_PWRSEQ_REQ_ANT_BIAS_EN:
		case STS_PWRSEQ_REQ_DISP_EN:
		case STS_PWRSEQ_REQ_RB_GATE:
		case STS_PWRSEQ_REQ_REF_TERM_EN:
			ok = pwrseq_mbox_apply_rail(req, release, value, &err,
						    &applied);
			break;
		case STS_PWRSEQ_REQ_RB_VSET_MV:
			ok = pwrseq_mbox_apply_vset(release, value, &err,
						    &applied);
			break;
		case STS_PWRSEQ_REQ_FAN_DUTY:
			ok = pwrseq_mbox_apply_fan(release, value, &err,
						   &applied);
			break;
		case STS_PWRSEQ_REQ_RGB_MODE:
			ok = pwrseq_mbox_apply_rgb_mode(release, value, &err,
							&applied);
			break;
		case STS_PWRSEQ_REQ_RGB_R:
		case STS_PWRSEQ_REQ_RGB_G:
		case STS_PWRSEQ_REQ_RGB_B:
			ok = pwrseq_mbox_apply_rgb_leg(req, release, value, &err,
						       &applied);
			break;
		default:
			/*
			 * A row added to the enum with no executor. Refused
			 * rather than ignored, so the gap surfaces as a failed
			 * override instead of a request that vanishes.
			 */
			ok = false;
			err = (uint8_t)STS_PWRSEQ_REQ_ERR_STAGE;
			break;
		}

		key = k_spin_lock(&pwrseq_mbox_lock);
		(void)sts_pwrseq_reqq_settle(
			&pwrseq_mbox, req,
			ok ? (uint8_t)STS_PWRSEQ_REQ_OUT_APPLIED
			   : (uint8_t)STS_PWRSEQ_REQ_OUT_REFUSED,
			err, applied, now_ms);
		k_spin_unlock(&pwrseq_mbox_lock, key);

		if (!ok) {
			LOG_WRN("pwrseq mailbox: request %u refused (%s)",
				(unsigned int)req,
				sts_pwrseq_req_reason_of(err));
		}
	}
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

	/*
	 * Single writer per pin: the antenna supervisor decides, this file
	 * drives (ARCHITECTURE.md §10). The supervisor's decision IS firmware's
	 * commanded level for PC9 — pwrseq's own `ant_bias_on` does not move
	 * when gnssmgr cuts the bias — so it is recorded here, and it is what a
	 * released `pwr.ant.bias.en` lease restores. Without this record a
	 * lapsed lease would re-energise the bias into a latched short by
	 * restoring the level pwrseq still believed in.
	 */
	pwrseq_rail_auto((uint8_t)STS_PWRSEQ_REQ_ANT_BIAS_EN, on);
	pwrseq_rail_drive(&pwrseq_rails[STS_PWRSEQ_REQ_ANT_BIAS_EN], on);

	if (on) {
		return;
	}

	/*
	 * The one writer of a rail an override can hold that does NOT pass
	 * through the action queue, so it carries its own veto rather than
	 * inheriting pwrseq_exec_action()'s.
	 *
	 * THE VETO BELONGS TO THIS DECISION, NOT TO THE PIN WRITE, and that is
	 * why the mailbox drain drives PC9 through pwrseq_rail_drive() rather
	 * than by calling this function. Routing an override through here would
	 * raise STS_MP_VETO_ANT_BIAS on the operator's own request and withdraw
	 * the lease that had just been granted — one event, two vetoes, and the
	 * second one aimed at the technician who caused the first.
	 *
	 * gnssmgr reaches here only from ant_enter(GNSSMGR_ANT_SHORT), once, on
	 * the latch — recovery is gnssmgr_ant_reenable() and nothing else — so
	 * this is a genuine edge and not a level re-assertion. A lease holding
	 * `pwr.ant.bias.en` on against a latched short is exactly the case FMT
	 * §5.1.2 reserves to firmware. The early return above is what keeps the
	 * re-enable path from raising one: restoring the bias is not contrary to
	 * an override that wanted it on.
	 */
	sts_mp_veto(STS_MP_VETO_ANT_BIAS);
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

	/*
	 * REF_TERM_EN is the one output that boots ASSERTED. GPIO_OUTPUT_ACTIVE
	 * drives PC10 to the terminated level R176's pull-up already holds it at,
	 * so the pin never passes through un-terminated on the way to being
	 * driven; `auto_on` records that as firmware's commanded level, which is
	 * what a released `ref.term.en` lease restores.
	 */
	if (!gpio_is_ready_dt(&ref_term_en)) {
		LOG_ERR("pwrseq: REF_TERM_EN port not ready");
		return -ENODEV;
	}
	rc = gpio_pin_configure_dt(&ref_term_en, GPIO_OUTPUT_ACTIVE);
	if (rc != 0) {
		LOG_ERR("pwrseq: REF_TERM_EN configure failed (%d)", rc);
		return rc;
	}
	pwrseq_rail_auto((uint8_t)STS_PWRSEQ_REQ_REF_TERM_EN, true);

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

	/*
	 * The envelope that is actually in force — after the fallback above, not
	 * before it. sts_rb_code_for_mv() bounds every maintenance setpoint
	 * against `rb_vmax_mv` from this copy, so a stale one would let a request
	 * be computed against a ceiling the sequencer rejected.
	 */
	pwrseq_cfg = cfg;
	dp_auto_code = cfg.digipot_safe_code;

	rc = pwrseq_start(&pwrseq, now_ms);
	if (rc != 0) {
		LOG_ERR("pwrseq_start failed (%d)", rc);
		return rc;
	}

	pwrseq_liveness_id = sts_liveness_register("pwrseq");
	pwrseq_started = true;

	/*
	 * Publish before returning. sts_pwrseq_snapshot() answers -ENODEV until
	 * pwrseq_started, and 0 from the moment it is set; without this the
	 * window between here and the first 4 Hz tick would report stage 0 with
	 * everything false, which is the exact class of untruth this snapshot
	 * exists to end. No observation has been taken yet, hence NULL.
	 *
	 * `scan_state` goes out as an explicit 0, and it is a *chosen* zero, not
	 * a forgotten one. io_scan has in fact been running since platform.c's
	 * bring-up step several hundred milliseconds ago, so a real bitmap could
	 * be fetched here — but the observed half is timestamped as a set by
	 * `tick_mono_ms`, which is 0 on this path. Publishing a live bitmap
	 * against a zero timestamp would hand a reader a word it cannot date,
	 * sitting beside a pg_mask and an EXTREF reading that are genuinely
	 * absent. One tick (<= 250 ms) of honest zero beats a number nobody can
	 * place in time. sts_pwrseq_snap_observe() enforces this by ignoring the
	 * argument whenever @p in is NULL; passing 0 states the intent at the
	 * call site as well.
	 */
	pwrseq_publish(NULL, 0U, false, 0U);

	LOG_INF("pwrseq up: rb_vmax %u mV, op code %u, safe code %u", cfg.rb_vmax_mv,
		cfg.digipot_operating_code, cfg.digipot_safe_code);

	return 0;
}
