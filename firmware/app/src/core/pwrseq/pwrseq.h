/*
 * STS1000 "Meridian" — core/pwrseq: bring-up stage machine and power policy.
 *
 * Platform-neutral C11. This module owns the ordering of the power-up sequence
 * in `docs/sts1000_firmware_hardware_interface.md` §2, the guarded rubidium
 * turn-on, the PoE load-shed ladder, the power-fail park list, the rubidium
 * over-voltage latch bookkeeping, and the external watchdog's arm/kick policy.
 *
 * It touches no hardware. `pwrseq_step()` reads a caller-filled snapshot of the
 * board (`pwrseq_in_t`) and appends typed **actions** to a queue that the glue
 * drains and executes — set this GPIO, write that digipot code, start the
 * discipline loop. That inversion is what makes an eight-stage sequence with
 * three interlocks on the rubidium testable at all: a host test scripts the
 * inputs and asserts the exact action order, which is the property that
 * matters.
 *
 * Fail-safe principle (interface ref §2): every gated load and every resettable
 * device sits in its safe state at reset, held there by an external pull, and
 * firmware releases them deliberately and in order. Nothing here ever enables a
 * load before its rail has been read back.
 *
 * ARCHITECTURE.md §10 invariants this module is responsible for:
 *   3.  Rb order is safe digipot code (verify readback) -> RB_PWR_EN -> INA228
 *       0x47 window -> RB_VCC_GATE; any out-of-window drops RB_PWR_EN and
 *       latches a fault.
 *   5.  WDT kick only after the liveness AND-gate; WDT_EN last in bring-up.
 *   6.  The relay is eligible only once the sequence has reached that point;
 *       any fault path drops it first.
 *   7.  SHUNT_CAL is re-applied before any INA228 reading is trusted.
 */

#ifndef STS1000_CORE_PWRSEQ_PWRSEQ_H_
#define STS1000_CORE_PWRSEQ_PWRSEQ_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ina228/ina228.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------- stages */

/**
 * Bring-up stages, numbered as in interface ref §2.
 *
 * Stages 0 and 1 are not represented: stage 0 is autonomous hardware (PoE
 * negotiation, the housekeeping bucks, the supercap managers, the OCXO
 * starting to warm) and stage 1 is the MCU leaving reset. Firmware's first
 * deliberate act is stage 2.
 */
typedef enum {
	PWRSEQ_STAGE_IDLE = 0,     /* before pwrseq_start() */
	PWRSEQ_STAGE_2_RELEASE = 2, /* release held-in-reset digital devices */
	PWRSEQ_STAGE_3_RAILS = 3,   /* verify main rails via telemetry + PG */
	PWRSEQ_STAGE_4_PHY = 4,     /* bring up the Ethernet PHY */
	PWRSEQ_STAGE_5_GNSS = 5,    /* GNSS receiver and antenna bias */
	PWRSEQ_STAGE_6_PANEL = 6,   /* display and panel LEDs (deferrable) */
	PWRSEQ_STAGE_7_OCXO = 7,    /* start disciplining once the OCXO is warm */
	PWRSEQ_STAGE_8_RB = 8,      /* the guarded rubidium sequence */
	PWRSEQ_STAGE_9_ARM = 9,     /* watchdog and alarm arming */
	PWRSEQ_STAGE_DONE = 10,
} pwrseq_stage_t;

/* ------------------------------------------------------------------- actions */

/**
 * Actions the sequencer emits for the glue to perform.
 *
 * Names say what the board does, not which pin moves, but the pin is given in
 * the comment because the glue has to bind them one-for-one.
 */
typedef enum {
	PWRSEQ_ACT_NONE = 0,

	/* Stage 2 */
	PWRSEQ_ACT_NOR_RST_RELEASE,  /* NOR_RST_N  PE10 -> HIGH */
	PWRSEQ_ACT_I2C_PROBE,        /* bring up I2C1, probe the 15-device map,
				      * read config/calibration from NOR */
	PWRSEQ_ACT_DISP_RST_RELEASE, /* DISP_RST   PA10 -> HIGH */

	/* Stage 3 */
	PWRSEQ_ACT_APPLY_SHUNT_TRIMS, /* write SHUNT_CAL to all nine INA228s */
	PWRSEQ_ACT_READ_RAIL_BASELINE,

	/* Stage 4 */
	PWRSEQ_ACT_LAN_RST_RELEASE, /* LAN_RST_N  PD10 -> HIGH */
	PWRSEQ_ACT_PHY_MDIO_POLL,   /* read the PHY ID at address 0 */

	/* Stage 5 */
	PWRSEQ_ACT_GPS_PWR_EN,       /* GPS_PWR_EN  PC8 -> HIGH */
	PWRSEQ_ACT_GPS_PWR_DIS,      /* GPS_PWR_EN  PC8 -> LOW */
	PWRSEQ_ACT_GPS_RST_RELEASE,  /* GPS_RST_N   PD11 deasserted */
	PWRSEQ_ACT_ANT_BIAS_EN,      /* ANT_BIAS_EN PC9 -> HIGH */
	PWRSEQ_ACT_ANT_BIAS_DIS,     /* ANT_BIAS_EN PC9 -> LOW */
	PWRSEQ_ACT_ANT_SUPERVISOR_START,
	PWRSEQ_ACT_GNSS_CONFIG_REQUEST, /* CFG-TXREADY remap + TIMEPULSE setup */

	/* Stage 6 */
	PWRSEQ_ACT_DISP_EN,       /* DISP_EN      PC11 -> HIGH */
	PWRSEQ_ACT_DISP_DIS,      /* DISP_EN      PC11 -> LOW */
	PWRSEQ_ACT_PANEL_LED_EN,  /* PANEL_LED_EN PC0 -> HIGH */
	PWRSEQ_ACT_PANEL_LED_DIS, /* PANEL_LED_EN PC0 -> LOW */
	PWRSEQ_ACT_PANEL_LED_PWM, /* PANEL_LED_PWM PE0, arg = duty percent */

	/* Stage 7 */
	PWRSEQ_ACT_DISC_START, /* hand the OCXO to the discipline loop */

	/* Stage 8 */
	PWRSEQ_ACT_DIGIPOT_WRITE,    /* U43 over SPI4, arg = safe/precharge code */
	PWRSEQ_ACT_DIGIPOT_WRITE_OP, /* U43, arg = operating code (bounded) */
	PWRSEQ_ACT_DIGIPOT_VERIFY,   /* read the wiper back */
	PWRSEQ_ACT_RB_PWR_EN,      /* RB_PWR_EN    PB7 -> HIGH */
	PWRSEQ_ACT_RB_PWR_DIS,     /* RB_PWR_EN    PB7 -> LOW */
	PWRSEQ_ACT_RB_VCC_GATE_EN, /* RB_VCC_GATE  PB1 -> HIGH */
	PWRSEQ_ACT_RB_VCC_GATE_DIS,/* RB_VCC_GATE  PB1 -> LOW */
	PWRSEQ_ACT_RB_OV_RESET_PULSE, /* RB_OV_RESET PD3 pulsed HIGH >= 10 us */

	/* Stage 9 */
	PWRSEQ_ACT_WDT_EN,         /* WDT_EN  PC12 -> HIGH (arms U64 SET1) */
	PWRSEQ_ACT_WDT_KICK_START, /* begin the WDT_KICK PB2 cadence */
	PWRSEQ_ACT_RELAY_ELIGIBLE, /* the sequence no longer blocks K2 (PA6) */

	/* Out-of-band */
	PWRSEQ_ACT_PARK_DAC,          /* freeze the OCXO loop, hold last Vc */
	PWRSEQ_ACT_PERSIST_STATE,     /* flush volatile critical state to NVS */
	PWRSEQ_ACT_SET_SHUTDOWN_FLAG, /* clean-shutdown marker for the next boot */
	PWRSEQ_ACT_POE_KILL,          /* POE_KILL PE15 -> HIGH, cold cycle */

	PWRSEQ_ACT_COUNT,
} pwrseq_action_t;

typedef struct {
	uint16_t action; /* pwrseq_action_t */
	uint16_t arg;    /* digipot code, PWM duty percent, else 0 */
} pwrseq_act_t;

/** Short, stable name for @p a, for logs and test failure messages. */
const char *pwrseq_action_name(pwrseq_action_t a);

/**
 * Most actions any single sequencer step can emit: its own, plus the two-action
 * rubidium shutdown group on the failure path.
 */
#define PWRSEQ_ACT_PER_STEP_MAX 3

/** Action queue depth. Must leave room for a step at every loop iteration. */
#ifndef PWRSEQ_ACT_QUEUE_LEN
#define PWRSEQ_ACT_QUEUE_LEN 32
#endif

/** Steps a single pwrseq_step() call may advance through. */
#ifndef PWRSEQ_MAX_STEPS_PER_CALL
#define PWRSEQ_MAX_STEPS_PER_CALL 16
#endif

/* ------------------------------------------------------------ failure policy */

/** What to do when a step's exit condition does not arrive in time. */
typedef enum {
	/**
	 * Re-enter the step, re-emitting its action, up to `retries` times.
	 * When the retries are exhausted this escalates to PWRSEQ_ONFAIL_ALARM.
	 */
	PWRSEQ_ONFAIL_RETRY = 0,
	/**
	 * Raise the step's alarm, run its failure action, abandon the rest of
	 * the stage and move to the next one.
	 *
	 * Abandoning the remainder of the stage is the safe universal
	 * behaviour: a step whose precondition never arrived must not be
	 * followed by the step that depends on it. It is why a failed GPS rail
	 * never reaches ANT_BIAS_EN and a failed digipot verify never reaches
	 * RB_PWR_EN.
	 */
	PWRSEQ_ONFAIL_ALARM,
	/** Raise the alarm, run the failure action, and stop the sequencer. */
	PWRSEQ_ONFAIL_HALT,
	/**
	 * Mark the stage deferred and move on, without treating it as a fault.
	 * Used for the rubidium preconditions: an inadequate PoE budget is a
	 * reason to run on the OCXO, not a reason to fail bring-up.
	 */
	PWRSEQ_ONFAIL_DEFER,
} pwrseq_onfail_t;

/** Grouped failure actions, expanded into individual queue entries. */
typedef enum {
	PWRSEQ_FAILACT_NONE = 0,
	PWRSEQ_FAILACT_GPS_OFF,   /* GPS_PWR_EN low */
	PWRSEQ_FAILACT_ANT_OFF,   /* ANT_BIAS_EN low */
	PWRSEQ_FAILACT_DISP_OFF,  /* DISP_EN low */
	PWRSEQ_FAILACT_PANEL_OFF, /* PANEL_LED_EN low */
	/** RB_VCC_GATE low then RB_PWR_EN low — load first, then supply. */
	PWRSEQ_FAILACT_RB_OFF,
} pwrseq_failact_t;

/* -------------------------------------------------------------------- alarms */

/**
 * Alarms the sequencer raises. Bit 0 (PWRSEQ_ALARM_NONE) is the step table's
 * "no alarm" sentinel and is never set in the mask.
 */
typedef enum {
	PWRSEQ_ALARM_NONE = 0,
	PWRSEQ_ALARM_NOR,
	PWRSEQ_ALARM_I2C,
	PWRSEQ_ALARM_SHUNT_TRIM,
	PWRSEQ_ALARM_RAIL_BASELINE,
	PWRSEQ_ALARM_RAIL_3V3,
	PWRSEQ_ALARM_RAIL_3V3_STM,
	PWRSEQ_ALARM_RAIL_POE,
	PWRSEQ_ALARM_PHY,
	PWRSEQ_ALARM_GPS_RAIL,
	PWRSEQ_ALARM_ANTENNA,
	PWRSEQ_ALARM_GNSS_CFG,
	PWRSEQ_ALARM_DISPLAY,
	PWRSEQ_ALARM_PANEL_LED,
	PWRSEQ_ALARM_OCXO_WARM,
	PWRSEQ_ALARM_POE_BUDGET,
	PWRSEQ_ALARM_RB_PRECONDITION,
	PWRSEQ_ALARM_RB_DIGIPOT,
	PWRSEQ_ALARM_RB_WINDOW,
	PWRSEQ_ALARM_RB_LOCK,
	PWRSEQ_ALARM_RB_OV,
	PWRSEQ_ALARM_RB_FAULT, /* umbrella, set alongside any specific Rb cause */
	PWRSEQ_ALARM_LIVENESS,
	/**
	 * VCC_RB telemetry was too stale for too long to judge a rail window.
	 *
	 * Deliberately NOT a rubidium *hard* fault: "I could not see the rail"
	 * is not "the rail was wrong". A stale reading must never latch
	 * RB_FAULT, because that latch abandons stage 8 with no way back, so a
	 * telemetry hiccup would permanently cost the board its rubidium.
	 */
	PWRSEQ_ALARM_RB_TELEMETRY,
	PWRSEQ_ALARM_COUNT,
} pwrseq_alarm_t;

#define PWRSEQ_ALARM_BIT(a) ((uint32_t)1U << (unsigned int)(a))

/** Every rubidium-related alarm, including the umbrella. */
#define PWRSEQ_ALARM_RB_MASK                                                   \
	(PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_PRECONDITION) |                      \
	 PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_DIGIPOT) |                           \
	 PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_WINDOW) |                            \
	 PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_LOCK) |                              \
	 PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_OV) |                                \
	 PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_TELEMETRY) |                         \
	 PWRSEQ_ALARM_BIT(PWRSEQ_ALARM_RB_FAULT))

/** Short, stable name for @p a. Never NULL. */
const char *pwrseq_alarm_name(pwrseq_alarm_t a);

/* ---------------------------------------------------------------- shed ladder */

/**
 * PoE / thermal load-shed ladder, in the order the documentation gives.
 *
 * Interface ref §2 stage 6 calls the display "the first item to shed under PoE
 * pressure", and spec §10.3 repeats it; spec §13's over-temperature response is
 * "fan max; if exceeded, shed Rb, then thermal POE_KILL". The display rail also
 * happens to be the larger of the two UI loads, so dropping it first sheds the
 * most watts for the least loss of function.
 */
typedef enum {
	PWRSEQ_SHED_NONE = 0,
	PWRSEQ_SHED_DISPLAY,   /* DISP_EN dropped */
	PWRSEQ_SHED_PANEL_LED, /* + PANEL_LED_EN dropped */
	PWRSEQ_SHED_RB,        /* + the rubidium dropped */
} pwrseq_shed_level_t;

/* ---------------------------------------------------------------- watchdog */

/*
 * TPS3430 (U64) window timing with SET0 = 1, SET1 = 1, CWD and CRST floating —
 * the factory preset, no timing capacitors. From `docs/sts1000_external_wdt.md`
 * §4.
 */
#define PWRSEQ_WDT_TWDL_MIN_MS 680U  /* too-early boundary, min */
#define PWRSEQ_WDT_TWDL_TYP_MS 800U
#define PWRSEQ_WDT_TWDL_MAX_MS 920U
#define PWRSEQ_WDT_TWDU_MIN_MS 1360U /* too-late boundary, min */
#define PWRSEQ_WDT_TWDU_TYP_MS 1600U
#define PWRSEQ_WDT_TWDU_MAX_MS 1840U
#define PWRSEQ_WDT_TRST_TYP_MS 200U  /* WDO assert duration */

/** Earliest a kick may be issued without tripping the runaway boundary. */
#define PWRSEQ_WDT_WINDOW_MIN_MS PWRSEQ_WDT_TWDL_MAX_MS /* 920 */
/** Latest a kick may be issued without tripping the stall boundary. */
#define PWRSEQ_WDT_WINDOW_MAX_MS PWRSEQ_WDT_TWDU_MIN_MS /* 1360 */

/**
 * Nominal kick cadence, straight from external_wdt §4.
 *
 * The valid window is 920-1360 ms, whose arithmetic centre is 1140 ms. The
 * documented cadence is 1100 ms, biased 40 ms early on purpose: a software
 * timer under load runs late far more readily than it runs early, so the
 * margin is spent where it will be needed (+180 ms above the early bound,
 * -260 ms below the late bound).
 */
#define PWRSEQ_WDT_KICK_PERIOD_MS 1100U

/**
 * Verdict on an observed interval between two WDI falling edges.
 *
 * external_wdt §4: a kick faster than tWDL(min) = 680 ms is a RUNAWAY fault and
 * a kick slower than tWDU(min) = 1360 ms is a STALL fault; either drives WDO_N,
 * which drives POE_KILL, which drops the board's PoE port. Both directions are
 * therefore equally fatal, and both have to be *observable* — a cadence bug that
 * only shows up as an unexplained field reboot is a bug nobody can diagnose.
 */
typedef enum {
	PWRSEQ_WDT_INTERVAL_OK = 0,
	PWRSEQ_WDT_INTERVAL_EARLY, /* below PWRSEQ_WDT_WINDOW_MIN_MS: runaway */
	PWRSEQ_WDT_INTERVAL_LATE,  /* above PWRSEQ_WDT_WINDOW_MAX_MS: stall */
} pwrseq_wdt_interval_t;

/** Classify @p interval_ms against the TPS3430 valid window. */
pwrseq_wdt_interval_t pwrseq_wdt_classify_interval(uint32_t interval_ms);

/** What one supervisor tick decided about the watchdog. */
typedef struct {
	/** Drive a WDI edge now. The caller does the pin, nothing else may. */
	bool kick;
	/** The configured cadence has elapsed since the last kick. */
	bool due;
	/** Every liveness bit was set. */
	bool liveness_ok;
	/** Milliseconds since the previous kick. Only meaningful with @p kick. */
	uint32_t interval_ms;
	/** pwrseq_wdt_interval_t for @p interval_ms; only with @p verdict_valid. */
	uint8_t verdict;
	/** False for the very first kick, which has no previous edge. */
	bool verdict_valid;
} pwrseq_wdt_tick_t;

/** Supervisor liveness bits; all must be set before a kick is permitted. */
typedef enum {
	PWRSEQ_LIVE_TIMING = 1U << 0,
	PWRSEQ_LIVE_NETWORK = 1U << 1,
	PWRSEQ_LIVE_HOUSEKEEPING = 1U << 2,
} pwrseq_live_t;

#define PWRSEQ_LIVE_ALL                                                        \
	((uint32_t)(PWRSEQ_LIVE_TIMING | PWRSEQ_LIVE_NETWORK |                 \
		    PWRSEQ_LIVE_HOUSEKEEPING))

/* --------------------------------------------------- rubidium rail transfer */

/**
 * VCC_RB setpoint transfer function (`docs/sts1000_vcc_rb_supply.md` §5).
 *
 *     VCTRL = ((steps - code) / steps) * vref_mv
 *     VOUT  = pedestal_mv - (gain_m1000 / 1000) * VCTRL
 *
 * With the as-built parts — VREF_3V0 = 3.0 V, a 10-bit MCP41U83, R147/R148 =
 * 33.11 and R147/R134 = 6.645 — that is `VOUT = 24.45 - 6.645*VCTRL`, i.e.
 * 4.515 V at code 0 and 24.43 V at code 1023. Note the sense: **code 0 puts the
 * wiper at terminal B = VREF, which is maximum VCTRL and therefore minimum
 * VOUT.** That is the safe-low power-up direction, and getting it backwards
 * would put 24 V onto a 15 V-class FE-5680A.
 *
 * Because VOUT *rises* toward the pedestal as the code rises, an out-of-range
 * code must never be clamped toward `steps-1` — that is the *maximum*-output
 * end. pwrseq_digipot_vctrl_mv() clamps any code at or above `steps` to the
 * safe-low VCTRL (= the code-0 value), so a garbage code produces the minimum
 * rail, not the maximum. See the CLAUDE.md gotcha: "a digipot in a buck FB
 * node can destroy the Rb; no wiper code (POR, mid-scale, SPI fault) may exit
 * the safe envelope."
 */
typedef struct {
	uint16_t vref_mv;     /* 3000 — VREF_3V0, MCP1502-30 */
	uint16_t steps;       /* 1024 — MCP41U83 is 10-bit */
	uint32_t pedestal_mv; /* 24450 — 0.6*(1 + R147/R148 + R147/R134) */
	uint32_t gain_m1000;  /* 6645 — 1000 * R147/R134 */
} pwrseq_rb_xfer_t;

/** VCTRL in millivolts for @p code. Codes at or above `steps` clamp to the
 *  top code. 0 when @p x is NULL or misconfigured. */
uint32_t pwrseq_digipot_vctrl_mv(const pwrseq_rb_xfer_t *x, uint16_t code);

/** Expected VCC_RB in millivolts for @p code. 0 when @p x is NULL. */
int32_t pwrseq_rb_expected_mv(const pwrseq_rb_xfer_t *x, uint16_t code);

/**
 * Acceptance window around the expected rail voltage.
 *
 * @retval 0        @p lo and @p hi written.
 * @retval -EINVAL  @p x, @p lo or @p hi is NULL.
 */
int pwrseq_rb_window(const pwrseq_rb_xfer_t *x, uint16_t code, uint32_t tol_pct,
		     int32_t *lo, int32_t *hi);

/* -------------------------------------------------------------------- config */

typedef struct {
	/* Stage timeouts, milliseconds. */
	uint32_t nor_timeout_ms;
	uint32_t i2c_timeout_ms;
	uint32_t trim_timeout_ms;
	uint32_t rail_timeout_ms;
	uint32_t phy_clk_timeout_ms;
	uint32_t phy_settle_ms;
	uint32_t phy_id_timeout_ms;
	uint32_t gps_rail_timeout_ms;
	uint32_t gps_boot_ms;
	uint32_t ant_timeout_ms;
	uint32_t gnss_cfg_timeout_ms;
	uint32_t disp_timeout_ms;
	uint32_t ocxo_warm_timeout_ms;
	uint32_t rb_precondition_timeout_ms;
	uint32_t rb_softstart_ms;
	/**
	 * How long a rubidium rail window may go unsatisfied before the step
	 * fails, milliseconds.
	 *
	 * Sized against the **telemetry cadence**, not the buck's settling time.
	 * The rail is judged from INA228 0x47 through the housekeeping cache,
	 * which the glue refreshes on request at its 4 Hz tick and unconditionally
	 * at 1 Hz. A timeout shorter than a couple of cache refreshes gives the
	 * step one evaluation against a reading that may predate the event it is
	 * meant to observe — which is exactly how a window that the hardware
	 * satisfies still fails on every board.
	 */
	uint32_t rb_window_timeout_ms;
	uint32_t rb_lock_timeout_ms;
	uint32_t liveness_timeout_ms;

	/**
	 * Oldest VCC_RB reading a rubidium rail decision may be made on,
	 * milliseconds. Older than this and the step waits rather than judging
	 * (see PWRSEQ_ALARM_RB_TELEMETRY). Must comfortably exceed the caller's
	 * unconditional telemetry sweep period.
	 */
	uint32_t ina_max_age_ms;
	/**
	 * How long a rubidium rail step may sit with unusably stale telemetry
	 * before it gives up, milliseconds. Bounded so a dead I²C bus cannot
	 * park the sequencer in stage 8 and leave the watchdog unarmed.
	 */
	uint32_t rb_stale_stall_ms;

	/** Automatic whole-stage-8 retries after a rubidium failure or deferral. */
	uint8_t rb_auto_retry_max;
	/** Quiet time between automatic rubidium retries, milliseconds. */
	uint32_t rb_auto_retry_delay_ms;

	/** Continuous zero PoE headroom before the shed ladder advances, ms. */
	uint32_t poe_shed_dwell_ms;
	/** Continuous headroom above poe_relief_mw before a rung is released, ms. */
	uint32_t poe_restore_dwell_ms;
	/** Headroom that counts as relief for the display/panel rungs, milliwatts. */
	uint32_t poe_relief_mw;
	/**
	 * How long core/thermal's rung-3 request must persist before POE_KILL is
	 * commanded, milliseconds. The consequence is a full cold cycle with
	 * PSE-driven recovery, so a single glitched sample must not cause one.
	 */
	uint32_t poe_kill_confirm_ms;

	/** Rail acceptance band as a percentage of the nominal voltage (0..100). */
	uint32_t rail_tol_pct;
	/** PoE bus floor. Below this the PD is out of its operating range. */
	int32_t poe_min_mv;

	/**
	 * OCXO warm detection. A cold oven pulls well over
	 * @p ocxo_warmup_current_ma; once the oven reaches setpoint the draw
	 * settles below @p ocxo_warm_current_ma. Both are bench-tunable.
	 */
	uint32_t ocxo_warm_current_ma;
	uint32_t ocxo_warmup_current_ma;

	/**
	 * Digipot **precharge** code, written and readback-verified before
	 * RB_PWR_EN (interface ref §2 step 11; ARCHITECTURE.md invariant 3).
	 * Must be the safe-low end: 0 = terminal B = VREF = minimum VOUT
	 * (~4.5 V). The buck is brought up here and the rail verified at this
	 * known-low point before the operating setpoint is commanded.
	 */
	uint16_t digipot_safe_code;
	/**
	 * Digipot **operating** setpoint, written only after the precharged
	 * rail has been verified, then re-verified against rb_vmax_mv before
	 * RB_VCC_GATE. Bounded at init so its expected voltage cannot exceed
	 * rb_vmax_mv — this, plus the independent measured-voltage gate, is
	 * what stops a wrong setpoint from validating itself (BLOCKER-1). Set
	 * per FE unit; the default targets ~14.25 V, comfortably below a
	 * 15 000 mV ceiling with tolerance headroom.
	 */
	uint16_t digipot_operating_code;
	/**
	 * Absolute VCC_RB ceiling for the connected FE, millivolts (from cfg
	 * key PWR_RB_VMAX_MV, default 15000). The rubidium is never gated to
	 * the FE unless BOTH the commanded setpoint's expected voltage AND the
	 * INA228-measured rail sit at or below this — the measured check is
	 * independent of the digipot code, so a runaway buck cannot pass by
	 * matching its own wrong setpoint.
	 */
	uint32_t rb_vmax_mv;
	/** VCC_RB acceptance tolerance, percent (0..100). */
	uint32_t rb_vbus_tol_pct;
	/** Settle time after writing the operating code before the rail is
	 *  judged, milliseconds — the buck ramps from precharge to setpoint. */
	uint32_t rb_ramp_ms;
	/**
	 * PoE headroom (granted − measured) the budget must show before the
	 * rubidium may start, milliwatts. Default 16 000.
	 *
	 * This is a *gate*, not the peak draw. The FE-5680A warm-up transient is
	 * ~2.1 A at ~15 V ≈ 31 W (root CLAUDE.md: "~25–30 W cold-start peak"
	 * for both references together), which the port's bulk capacitance and
	 * the staged bring-up (OCXO already warm, display sheddable) absorb; the
	 * steady both-on draw is ~12–14 W. 16 W is chosen as the minimum
	 * *sustained* headroom below which the rail should not even be attempted
	 * — enough for the ~0.65 A steady FE draw plus margin, while a
	 * Class-4/Type-2 budget that cannot clear it defers to OCXO-only rather
	 * than browning out mid warm-up. Tune against the measured C_port.
	 */
	uint32_t rb_cold_start_mw;
	pwrseq_rb_xfer_t rb_xfer;

	/** Panel-LED PWM duty, percent, applied at stage 6. */
	uint16_t panel_led_duty_pct;

	/** WDT kick cadence; must lie inside the TPS3430 window. */
	uint32_t wdt_kick_period_ms;
} pwrseq_cfg_t;

/** Fill @p cfg with the documented defaults. */
void pwrseq_cfg_default(pwrseq_cfg_t *cfg);

/* -------------------------------------------------------------------- inputs */

/** Bit for power-good line PGn in pwrseq_in_t::pg_mask (1 = reads high/good). */
#define PWRSEQ_PG(n) ((uint8_t)(1U << (n)))

#define PWRSEQ_PG_3V3_GPS_LDO PWRSEQ_PG(0)
#define PWRSEQ_PG_OCXO_LDO    PWRSEQ_PG(1)
#define PWRSEQ_PG_3V0_RF_LDO  PWRSEQ_PG(2)
#define PWRSEQ_PG_5V_PSU      PWRSEQ_PG(3)
#define PWRSEQ_PG_3V3_PSU     PWRSEQ_PG(4)
#define PWRSEQ_PG_OCXO_PSU    PWRSEQ_PG(5)
#define PWRSEQ_PG_RB_PSU      PWRSEQ_PG(6)
#define PWRSEQ_PG_POE         PWRSEQ_PG(7)

/**
 * Board snapshot. The caller refreshes this before each pwrseq_step(); the
 * sequencer never reaches out for anything itself.
 *
 * The INA228 arrays are indexed by `ina228_rail_t`. `ina_valid[r]` says the
 * reading is fresh *and* taken after the per-board SHUNT_CAL trim was applied —
 * an uncalibrated monitor reads about 1 % off and raises no flag of its own
 * (interface ref §10 caution 7), which is why stage 3 applies the trim before
 * anything else and why this flag exists rather than being assumed.
 */
typedef struct {
	uint32_t mono_ms;

	/* stage 2 */
	bool nor_ready;
	bool i2c_probe_ok;
	bool cal_loaded; /* advisory: false means the trims are POR defaults */

	/* stage 3 onward */
	bool shunt_trims_applied;
	bool ina_valid[INA228_RAIL_COUNT];
	int32_t ina_vbus_mv[INA228_RAIL_COUNT];
	int32_t ina_current_ma[INA228_RAIL_COUNT];
	/**
	 * Age of each reading at `mono_ms`, milliseconds; UINT32_MAX when the
	 * rail has never been read.
	 *
	 * `ina_valid` says the reading was taken after the SHUNT_CAL trim; it says
	 * nothing about *when*. A rail commanded to move 100 ms ago and judged
	 * against a reading taken 900 ms before that is not a measurement of
	 * anything. Populate this or the rubidium windows will judge history.
	 */
	uint32_t ina_age_ms[INA228_RAIL_COUNT];
	uint8_t pg_mask;

	/* stage 4 */
	bool phy_refclk_stable; /* the 25 MHz PHY clock is running */
	bool phy_id_ok;         /* MDIO read a plausible ID at address 0 */

	/* stage 5 */
	bool gnss_cfg_ack; /* the F9T ACKed CFG-TXREADY and the TP config */

	/* stage 7 */
	bool ocxo_warm;        /* explicit "warm" from the discipline loop */
	bool ocxo_temp_stable; /* TMP117 0x49 has settled */

	/* stage 8 */
	bool rb_wanted;
	bool supercaps_charged;
	uint16_t digipot_readback;
	bool digipot_readback_valid;
	bool rb_lock;        /* PB13, already through the polarity bit */
	bool extref_in_band; /* PB14/TIM12 measured a real ~10 MHz */
	bool rb_ov_det;      /* PE3, the autonomous 26 V latch tripped */

	/* PoE budget */
	uint32_t poe_granted_mw;
	uint32_t poe_measured_mw;

	/* policy */
	bool ui_wanted;

	/**
	 * core/thermal rung 2: shed the rubidium (thermal_out_t::request_rb_shed).
	 * Already hysteresis-latched by that module, so pwrseq acts on the edge
	 * without adding a second filter.
	 */
	bool thermal_shed_rb;
	/**
	 * core/thermal rung 3: cold-cycle the board
	 * (thermal_out_t::request_poe_kill). Confirmed over
	 * `cfg.poe_kill_confirm_ms` before POE_KILL is commanded.
	 */
	bool thermal_poe_kill;

	/* stage 9 */
	bool liveness_ok;
	bool debugger_attached;
} pwrseq_in_t;

/* ------------------------------------------------------------------- context */

typedef struct {
	pwrseq_cfg_t cfg;

	uint8_t stage;   /* pwrseq_stage_t */
	int16_t step;    /* index into the internal step table, -1 = none */
	bool step_armed; /* the current step's entry action has been emitted */
	uint8_t retries;
	uint32_t stage_entered_ms;
	uint32_t step_entered_ms;
	uint32_t now_ms;
	/*
	 * Time the current step spent unable to judge itself, because the
	 * evidence its exit predicate needs was too stale. That time is held out
	 * of `step_entered_ms` so the step's timeout measures "the condition did
	 * not arrive", never "I could not see whether it arrived".
	 */
	uint32_t step_stall_ms;

	bool halted;
	bool rb_deferred;

	/*
	 * The digipot code currently commanded (safe precharge, then operating).
	 * The rail-window checks derive the expected voltage from this rather
	 * than always from the safe code, so the supervisor judges the rail
	 * against whatever is actually set — and the operating-window step also
	 * gates on the code-independent measured-vs-rb_vmax_mv check.
	 */
	uint16_t rb_current_code;

	/* what the sequencer believes it has turned on */
	bool nor_released;
	bool disp_rst_released;
	bool phy_released;
	bool gps_on;
	bool ant_bias_on;
	bool display_on;
	bool panel_led_on;
	bool disc_started;
	bool rb_enabled; /* RB_PWR_EN asserted */
	bool rb_gated;   /* RB_VCC_GATE asserted */
	bool rb_locked;
	bool wdt_armed;
	bool relay_eligible;

	/* rubidium over-voltage latch */
	bool ov_present; /* RB_OV_DET reads high right now */
	bool ov_latched; /* it has since the last successful clear */
	uint32_t ov_first_ms;
	uint32_t ov_count;

	/* power fail */
	bool pfi_seen;
	bool pfi_expected; /* a commanded POE_KILL was already armed */
	bool kill_armed;

	uint8_t shed; /* pwrseq_shed_level_t */

	uint32_t alarms;

	uint32_t last_kick_ms;

	pwrseq_act_t q[PWRSEQ_ACT_QUEUE_LEN];
	uint16_t q_head;
	uint16_t q_len;
	uint32_t q_dropped;
} pwrseq_ctx_t;

/** Flattened status, for telemetry and the console. */
typedef struct {
	pwrseq_stage_t stage;
	uint32_t stage_entered_ms;
	uint8_t retries;
	bool halted;
	bool rb_deferred;
	bool rb_enabled;
	bool rb_gated;
	bool rb_locked;
	bool ov_latched;
	bool wdt_armed;
	bool relay_eligible;
	bool display_on;
	bool panel_led_on;
	bool gps_on;
	bool ant_bias_on;
	bool disc_started;
	bool phy_released;
	bool pfi_seen;
	bool pfi_expected;
	pwrseq_shed_level_t shed;
	uint32_t alarms;
} pwrseq_status_t;

/* ----------------------------------------------------------------------- API */

/**
 * Initialise @p ctx. @p cfg may be NULL for pwrseq_cfg_default().
 *
 * @retval 0        Initialised; the sequencer sits at PWRSEQ_STAGE_IDLE.
 * @retval -EINVAL  @p ctx is NULL; the rubidium transfer function is degenerate
 *                  (zero steps or zero reference); a digipot code is out of
 *                  range or commands a rail above rb_vmax_mv (BLOCKER-1 — a
 *                  misconfigured setpoint is rejected at boot, not discovered
 *                  as a destroyed FE); a tolerance percentage exceeds 100; or
 *                  the WDT kick cadence falls outside the TPS3430 window — a
 *                  kick that is too fast trips the runaway boundary just as
 *                  surely as one too slow trips the stall boundary.
 */
int pwrseq_init(pwrseq_ctx_t *ctx, const pwrseq_cfg_t *cfg);

/**
 * Enter stage 2 and begin.
 *
 * @retval 0        Started.
 * @retval -EINVAL  @p ctx is NULL.
 */
int pwrseq_start(pwrseq_ctx_t *ctx, uint32_t mono_ms);

/**
 * Advance the sequence as far as @p in allows.
 *
 * Emits actions into the queue; drain it with pwrseq_action_get() and execute
 * them before the next call. The call advances at most
 * PWRSEQ_MAX_STEPS_PER_CALL steps and stops early if the queue lacks room for
 * another step, so the queue can never overflow from the sequencer itself.
 *
 * @retval 0        Processed (including "no progress" and "halted").
 * @retval -EINVAL  @p ctx or @p in is NULL.
 */
int pwrseq_step(pwrseq_ctx_t *ctx, const pwrseq_in_t *in);

/**
 * Pop the oldest queued action.
 *
 * @retval 0        Action written to @p out.
 * @retval -EINVAL  @p ctx or @p out is NULL.
 * @retval -EAGAIN  Queue empty.
 */
int pwrseq_action_get(pwrseq_ctx_t *ctx, pwrseq_act_t *out);

/** Actions waiting to be executed. */
size_t pwrseq_action_count(const pwrseq_ctx_t *ctx);

/**
 * Actions lost because the queue was full.
 *
 * The step machine reserves room before every step, so it never contributes
 * here. Only the out-of-band entry points (shed, kill, OV clear) can, and only
 * if the caller stops draining. Non-zero means the glue is not keeping up and
 * the board's actual state may not match this module's belief about it.
 */
uint32_t pwrseq_actions_dropped(const pwrseq_ctx_t *ctx);

/** Snapshot the sequencer state.
 *
 * @retval 0        Written to @p out.
 * @retval -EINVAL  @p ctx or @p out is NULL. */
int pwrseq_status(const pwrseq_ctx_t *ctx, pwrseq_status_t *out);

/* -------------------------------------------------------- derived readings */

/**
 * PoE headroom in milliwatts: granted minus measured, floored at zero so an
 * over-budget reading cannot wrap into an apparently huge allowance. 0 for a
 * NULL @p in.
 */
uint32_t pwrseq_poe_headroom_mw(const pwrseq_in_t *in);

/**
 * True when the OCXO oven has reached setpoint.
 *
 * Either the discipline loop says so outright, or the INA228 0x46 draw has
 * settled below `ocxo_warm_current_ma` *and* the oscillator TMP117 has
 * stabilised. Current alone is not enough — an oven that has only just been
 * switched on also draws little for a moment, and a stratum-1 advertisement
 * that jumped the gun would be worse than a slow one (spec §3.4).
 */
bool pwrseq_ocxo_is_warm(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in);

/**
 * True while the oven is still pulling warm-up current (at or above
 * `ocxo_warmup_current_ma`). Feeds the amber RGB state and the PoE budget
 * estimate during the cold-start peak.
 */
bool pwrseq_ocxo_is_warming(const pwrseq_ctx_t *ctx, const pwrseq_in_t *in);

/** Current stage. PWRSEQ_STAGE_IDLE when @p ctx is NULL. */
pwrseq_stage_t pwrseq_stage(const pwrseq_ctx_t *ctx);

/** Raised alarms, as a PWRSEQ_ALARM_BIT mask. */
uint32_t pwrseq_alarms(const pwrseq_ctx_t *ctx);

/** True when any rubidium alarm is raised. */
bool pwrseq_rb_fault(const pwrseq_ctx_t *ctx);

/**
 * Re-enter @p stage from its first step, clearing any halt.
 *
 * @retval 0        Re-entered.
 * @retval -EINVAL  @p ctx is NULL or @p stage is not a real stage.
 */
int pwrseq_restart_stage(pwrseq_ctx_t *ctx, pwrseq_stage_t stage,
			 uint32_t mono_ms);

/**
 * Re-attempt the rubidium after a deferral — for instance once the PoE budget
 * frees up because the display was shed. Clears the deferral and the rubidium
 * alarms, then re-enters stage 8 at its first step, so the full guarded
 * sequence runs again rather than resuming mid-way.
 *
 * Refused while halted: a halt is a hard fault (e.g. a 3V3 rail-verify
 * failure) that must not be cleared by a routine rubidium retry. Only
 * pwrseq_restart_stage() clears a halt, and only under explicit operator
 * intent (HIGH-2).
 *
 * @retval 0        Re-entered stage 8.
 * @retval -EINVAL  @p ctx is NULL.
 * @retval -EPERM   The sequencer is halted; use pwrseq_restart_stage().
 */
int pwrseq_rb_retry(pwrseq_ctx_t *ctx, uint32_t mono_ms);

/* -------------------------------------------------------------- shed ladder */

/** Current shed level. */
pwrseq_shed_level_t pwrseq_shed_level(const pwrseq_ctx_t *ctx);

/**
 * Shed the next load down the ladder.
 *
 * @retval 0        Shed; actions queued.
 * @retval -EINVAL  @p ctx is NULL.
 * @retval -ENOENT  Everything sheddable is already shed.
 */
int pwrseq_shed_step(pwrseq_ctx_t *ctx, uint32_t mono_ms);

/**
 * Restore one level.
 *
 * Restoring the display or the panel LEDs simply re-enables them. Restoring the
 * **rubidium does not**: re-asserting RB_PWR_EN directly would bypass the
 * digipot verify and the rail-window check, which is exactly the sequence
 * ARCHITECTURE.md invariant 3 exists to enforce. Instead this re-enters stage 8
 * so the guarded sequence runs from the top.
 *
 * Refused while halted, for the same reason as pwrseq_rb_retry(): restoring a
 * load — especially re-entering stage 8 to restore the rubidium — must not
 * resurrect a board that halted on a hard fault (HIGH-2).
 *
 * @retval 0        Restored one level.
 * @retval -EINVAL  @p ctx is NULL.
 * @retval -ENOENT  Nothing is shed.
 * @retval -EPERM   The sequencer is halted.
 */
int pwrseq_shed_restore(pwrseq_ctx_t *ctx, uint32_t mono_ms);

/* ------------------------------------------------------ over-voltage latch */

/**
 * Record the current RB_OV_DET level (PE3, polled).
 *
 * The 26 V latch is autonomous: by the time firmware sees this the buck has
 * already been disabled in hardware. Firmware only observes and, later, clears.
 *
 * @retval 0        Recorded.
 * @retval -EINVAL  @p ctx is NULL.
 */
int pwrseq_ov_observe(pwrseq_ctx_t *ctx, bool rb_ov_det, uint32_t mono_ms);

/** True when an over-voltage has been latched and not yet cleared. */
bool pwrseq_ov_latched(const pwrseq_ctx_t *ctx);

/**
 * Command a latch reset: queues an RB_OV_RESET pulse.
 *
 * Only ever on an explicit command, and only when the cause has gone. Pulsing
 * the latch while the rail is still over-voltage re-arms a supply that is
 * already known to be running away, so it is refused. On success this also
 * clears the RB_OV alarm and, if no other rubidium hard-fault remains, the
 * RB_FAULT umbrella (L1) — otherwise a cleared over-voltage would leave the
 * board looking permanently Rb-faulted.
 *
 * @retval 0        Pulse queued and the latch cleared.
 * @retval -EINVAL  @p ctx is NULL.
 * @retval -ENOENT  Nothing is latched.
 * @retval -EBUSY   RB_OV_DET still reads high; the cause has not gone.
 */
int pwrseq_ov_clear(pwrseq_ctx_t *ctx, uint32_t mono_ms);

/* --------------------------------------------------------------- power fail */

/**
 * Handle the PFI early warning (PE8/EXTI8).
 *
 * At the 180 µF port-capacitance ceiling there are roughly 4.8 ms of run time
 * after PFI asserts (`docs/sts1000_power_fail_input.md` §3.5), so the queue is
 * cleared first — whatever the sequencer wanted next is irrelevant now — and
 * replaced with the park list, in the priority order that document gives:
 *
 *   1. park the DAC (freeze the loop, latch the last good Vc);
 *   2. persist volatile timing state and the log;
 *   3. quiesce the rubidium — RB_VCC_GATE then RB_PWR_EN low — **unconditionally**,
 *      not gated on the sequencer's belief that it is running. The supervisor
 *      may have already queued that shutdown and the glue may not have drained
 *      it before the purge; the pin writes are idempotent, so re-emitting them
 *      guarantees they reach the pins during the hold-up window (MEDIUM-3);
 *   4. set the clean-shutdown flag for the next boot.
 *
 * Sets `pfi_expected` when a POE_KILL was already commanded, so the two paths
 * log differently — a deliberate cold cycle is not a line drop.
 *
 * **Concurrency contract.** PFI is delivered on EXTI8, so this runs in ISR
 * context and mutates the same action queue as pwrseq_step() and
 * pwrseq_action_get(). It is NOT internally locked (core is platform-neutral):
 * the glue MUST ensure it does not run concurrently with those — on this board
 * the PFI handler is the highest-priority path and the firmware spins/halts
 * after parking (power_fail_input §5), so nothing races it afterward. A caller
 * that runs pwrseq_step() from an interruptible context must mask this handler
 * across the step/drain, or route the PFI event through the same queue drain.
 *
 * @retval 0        Park list queued.
 * @retval -EINVAL  @p ctx is NULL.
 */
int pwrseq_pfi(pwrseq_ctx_t *ctx, uint32_t mono_ms);

/** Magic for pwrseq_poe_kill(): ASCII "KILL". */
#define PWRSEQ_POE_KILL_MAGIC 0x4B494C4CU

/**
 * Command a board cold cycle.
 *
 * Guarded by a magic value because the consequence is a full power cycle with
 * PSE-driven recovery — there is no in-band restart (`docs/sts1000_poe_kill.md`).
 *
 * @retval 0        POE_KILL queued.
 * @retval -EINVAL  @p ctx is NULL.
 * @retval -EPERM   Wrong magic; nothing queued.
 */
int pwrseq_poe_kill(pwrseq_ctx_t *ctx, uint32_t magic, uint32_t mono_ms);

/* ----------------------------------------------------------------- watchdog */

/**
 * True when a WDT_KICK edge should be issued now.
 *
 * Requires the watchdog to be armed, **all** liveness bits set, and the
 * configured cadence to have elapsed since the last kick. A windowed watchdog
 * kicked unconditionally on a timer protects nothing, which is why the liveness
 * gate is inside this predicate rather than left to the caller.
 *
 * Pure: call pwrseq_wdt_kicked() after actually driving the pin.
 */
bool pwrseq_wdt_kick_ok(const pwrseq_ctx_t *ctx, uint32_t liveness,
			uint32_t mono_ms);

/**
 * Record that a kick edge was issued.
 *
 * @retval 0        Recorded.
 * @retval -EINVAL  @p ctx is NULL.
 */
int pwrseq_wdt_kicked(pwrseq_ctx_t *ctx, uint32_t mono_ms);

/** True once WDT_EN has been asserted. */
bool pwrseq_wdt_armed(const pwrseq_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_PWRSEQ_PWRSEQ_H_ */
