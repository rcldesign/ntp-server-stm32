/*
 * STS1000 "Meridian" — platform-area internal header.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/platform/. The net, console and ui areas talk to the
 * platform through src/zephyr/sts_app.h and nothing else (ARCHITECTURE.md §2).
 */

#ifndef STS1000_ZEPHYR_PLATFORM_PLATFORM_H_
#define STS1000_ZEPHYR_PLATFORM_PLATFORM_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include "disc/disc.h"
#include "fault/fault.h"
#include "fwupd/rb_fwupd.h"
#include "gnssmgr/gnssmgr.h"
#include "ina228/ina228.h"
#include "quality/quality.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STS_ZEPHYR_USER DT_PATH(zephyr_user)

/** Fetch a gpio_dt_spec from /zephyr,user by property name. */
#define STS_USER_GPIO(prop) GPIO_DT_SPEC_GET(STS_ZEPHYR_USER, prop)

/* ------------------------------------------------------------------------- */
/* sts_app.c — cross-area state owned by the platform                        */
/* ------------------------------------------------------------------------- */

/** Bring up the cfg context (RAM store), the log ring and the quality state. */
int sts_app_early_init(void);

/**
 * The one seqlock-published quality state.
 *
 * Handed to disc_tick_pps()/disc_tick_no_pps() so core/disc publishes straight
 * into it: the discipline thread is the single writer (ARCHITECTURE.md §10.10)
 * and routing the block through an intermediate copy would only add a window
 * in which the published block and the loop's own state disagree.
 */
quality_state_t *sts_app_quality_state(void);

/** Supervisor view: mask of registered liveness ids that are currently late. */
uint32_t sts_liveness_stale_mask(uint32_t now_ms);

/** Number of registered liveness participants. */
uint32_t sts_liveness_count(void);

/** Name of a registered liveness participant, or NULL. */
const char *sts_liveness_name(uint32_t id);

/* ------------------------------------------------------------------------- */
/* faults.c — the shared fault context                                       */
/* ------------------------------------------------------------------------- */

/**
 * The single fault_ctx_t.
 *
 * Mutating entry points (fault_scan_input, fault_evt_get, fault_alarm_set)
 * are serialised by sts_fault_lock()/sts_fault_unlock(). The io_scan thread
 * holds the lock for the duration of one scan; everything else is a short
 * alarm set or a query.
 */
fault_ctx_t *sts_fault(void);
void sts_fault_lock(void);
void sts_fault_unlock(void);
int sts_fault_init(void);

/* ------------------------------------------------------------------------- */
/* pps.c — TIM2_CH1 / TIM3_CH1 input capture                                 */
/* ------------------------------------------------------------------------- */

/** One captured PPS edge pair. */
typedef struct {
	uint32_t tim2_cnt;   /* TIM2_CCR1, 32-bit, PA0 GPS_PPS */
	uint32_t tim3_cnt;   /* TIM3_CCR1 zero-extended, 16-bit, PC6 GPS_TP2 */
	uint32_t tim3_wraps; /* TIM3 update events since the previous capture */
	uint64_t mono_ms;    /* k_uptime_get() latched in the ISR */
	uint32_t seq;        /* incremented per PA0 capture; detects overrun */
	bool tim3_valid;     /* a TIM3 capture accompanied this TIM2 capture */
	bool tim2_overcapture; /* CC1OF was set: a previous capture was lost */
} sts_pps_capture_t;

/** Configure and start both capture channels. */
int sts_pps_init(void);

/**
 * Block until the next PA0 capture, or @p timeout_ms elapses.
 *
 * @retval 0        @p out holds a fresh capture.
 * @retval -EAGAIN  Timed out; no PPS arrived.
 */
int sts_pps_wait(sts_pps_capture_t *out, uint32_t timeout_ms);

/** Timer input-clock frequency in hertz, as derived from the live RCC tree. */
uint32_t sts_pps_timer_hz(void);

/** Total PA0 captures and lost (overcapture) events since boot. */
void sts_pps_counters(uint32_t *captures, uint32_t *lost);

/* ------------------------------------------------------------------------- */
/* clkmux.c — MUX_SEL executor (HSI bridge) + TIM12 EXTREF_MON               */
/* ------------------------------------------------------------------------- */

/** Take ownership of MUX_SEL, install the CSS NMI handler and arm CSS. */
int sts_clkmux_init(void);

/** Execute one refsel action list. Returns 0, or negative on a failed handoff. */
int sts_clkmux_execute(const void *steps, size_t n_steps);

/** Current MUX_SEL level: 0 = OCXO (input A), 1 = external (input B). */
int sts_clkmux_get(void);

/** Consume the CSS-fired latch. True when the clock security system tripped. */
bool sts_clkmux_css_fired(void);

/** Total CSS events since boot (not consumed by the read). */
uint32_t sts_clkmux_css_events(void);

/** Rebuild the clock tree on the OCXO after a CSS event. */
int sts_clkmux_recover(void);

/** Start the TIM12_CH1 (PB14) frequency measurement. */
int sts_extref_mon_init(void);

/** Close the current EXTREF_MON gate and open the next. Call at ~1 Hz. */
void sts_extref_mon_sample(void);

/**
 * Latest EXTREF_MON measurement.
 *
 * @param hz     Receives the measured frequency in hertz.
 * @param valid  Receives whether the measurement is fresh and trustworthy.
 * @param edges  Receives whether any edges arrived in the last gate interval.
 */
void sts_extref_mon_read(uint32_t *hz, bool *valid, bool *edges);

/* ------------------------------------------------------------------------- */
/* spi4.c — bus arbitration + MCP41U83 digipot                               */
/* ------------------------------------------------------------------------- */

/** Bind the SPI4 controller and build the digipot's transfer configuration. */
int sts_spi4_init(void);

/** Write the 10-bit digipot wiper (0..1023) and read it back. */
int sts_digipot_set(uint16_t code);

/** Read the 10-bit digipot volatile wiper register. */
int sts_digipot_get(uint16_t *code);

/** Program the non-volatile wiper so a POR comes up at @p code (0..1023). */
int sts_digipot_set_nv(uint16_t code);

/* ------------------------------------------------------------------------- */
/* hk.c — housekeeping: I2C sweep cache, pwrseq, thermal, supervisor         */
/* ------------------------------------------------------------------------- */

/** One INA228 rail reading. */
typedef struct {
	int32_t  bus_uv;      /* bus voltage, microvolts */
	int32_t  current_ua;  /* shunt current, microamps */
	uint32_t power_uw;    /* microwatts */
	uint16_t diag_alrt;   /* last DIAG_ALRT read (only on alert) */
	uint32_t age_ms;      /* mono_ms of the reading */
	bool     valid;
	bool     cal_ok;      /* SHUNT_CAL confirmed present since last reset */
} sts_ina_reading_t;

/** Snapshot of the housekeeping sensor cache. */
typedef struct {
	sts_ina_reading_t ina[INA228_RAIL_COUNT];
	int32_t  temp_enclosure_mc; /* TMP117 0x48, millicelsius */
	int32_t  temp_osc_mc;       /* TMP117 0x49, millicelsius */
	int32_t  die_mc;            /* STM32 internal sensor, millicelsius */
	int32_t  humidity_mpct;     /* SHT45 0x44, milli-percent RH */
	int32_t  temp_sht_mc;
	bool     temp_enclosure_valid;
	bool     temp_osc_valid;
	bool     die_valid;
	bool     sht_valid;
	uint32_t fan_rpm;
	uint16_t fan_duty_pct;
	uint32_t mono_ms;
} sts_hk_snapshot_t;

/** Copy the housekeeping cache. Mutex-protected; never call from an ISR. */
int sts_hk_read(sts_hk_snapshot_t *out);

/** Ask the housekeeping thread to re-read one INA228's DIAG_ALRT + values. */
void sts_hk_request_ina(uint8_t rail_idx);

/**
 * Read one INA228 synchronously, on the calling thread.
 *
 * For bring-up only, before the housekeeping thread exists: stage 3 has to check
 * rails it is about to allow other stages to depend on, and a request queued for
 * a thread that has not started yet is answered by an empty cache.
 *
 * @retval 0        The cache now holds a fresh reading for @p rail_idx.
 * @retval -EINVAL  @p rail_idx out of range.
 * @retval -EIO     The transfer failed; the reading is marked invalid.
 */
int sts_hk_read_ina_now(uint8_t rail_idx);

/**
 * Latest core/thermal escalation requests (rungs 2 and 3), for pwrseq.
 *
 * Cached from the 1 Hz thermal step so the 4 Hz sequencer sees a stable value.
 * Either pointer may be NULL.
 */
void sts_hk_thermal_requests(bool *shed_rb, bool *poe_kill);

/** Run the PFI latch/recovery state machine. Called at 4 Hz from housekeeping. */
void sts_pfi_service(uint32_t now_ms);

/**
 * Stage 3: probe all nine INA228s, apply CONFIG/ADC_CONFIG/SHUNT_CAL/SOVL/SUVL.
 *
 * @retval 0     All nine configured.
 * @retval -EIO  At least one is absent or rejected its configuration; the ones
 *               that did respond are usable and flagged cal_ok.
 */
int sts_hk_ina_configure_all(void);

/** Start the housekeeping thread. */
int sts_hk_start(void);

/* ------------------------------------------------------------------------- */
/* supervisor.c — WDT kick, holdover relay, status RGB                       */
/* ------------------------------------------------------------------------- */

int sts_supervisor_init(void);

/**
 * Start the dedicated watchdog-kick thread.
 *
 * Separate from the housekeeping thread on purpose. The kick used to be issued
 * from housekeeping (priority 14, the lowest-priority liveness participant on the
 * board), which made a starved or slow low-priority thread able to *cause* the
 * very hardware power cycle the watchdog exists to trigger only on a real hang.
 * The kicker therefore runs at a cooperative priority — unpreemptable by any
 * application thread — and consumes the liveness mask rather than being in it.
 */
int sts_supervisor_wdt_start(void);

/** Stage 9: begin the WDT_KICK cadence (PWRSEQ_ACT_WDT_KICK_START). */
void sts_supervisor_kick_start(uint32_t now_ms);

/** Called at 4 Hz from the housekeeping thread. */
void sts_supervisor_step(uint32_t now_ms);

/* ------------------------------------------------------------------------- */
/* pwrseq_exec.c — core/pwrseq runtime sequencer + action executor           */
/* ------------------------------------------------------------------------- */

/** Initialise pwrseq (cfg from the power group) and start the stage machine. */
int sts_pwrseq_start(uint32_t now_ms);

/** Step pwrseq and drain its action queue. Called at 4 Hz from housekeeping. */
void sts_pwrseq_step(uint32_t now_ms);

/**
 * Run pwrseq's power-fail park list and drain it synchronously.
 *
 * Housekeeping-thread context, so it cannot race pwrseq_step()/action_get() —
 * which is the mutual exclusion pwrseq.h §pwrseq_pfi demands and which calling it
 * from the EXTI8 ISR would violate (the ISR's queue purge could delete a
 * mid-flight drain's remaining park list). The two pin writes that genuinely must
 * happen inside the ~4.8 ms hold-up window are done in the ISR instead, by
 * sts_pwrseq_rb_quiesce_from_isr().
 */
void sts_pwrseq_pfi(uint32_t now_ms);

/**
 * Drop the rubidium at the pins: RB_VCC_GATE low, then RB_PWR_EN low.
 *
 * ISR-safe (two GPIO register writes, no locks, no logging) and idempotent.
 * Called from the PFI handler: the FE's warm-up surge is the largest concurrent
 * load on the board, so shedding it is what extends the hold-up window that the
 * NVS fast-save has to complete inside (power_fail_input §5 item 3).
 */
void sts_pwrseq_rb_quiesce_from_isr(void);

/**
 * Operator-initiated retry of the guarded rubidium sequence.
 *
 * Exposed for a console/MCP command; the bounded automatic retry lives in
 * core/pwrseq. Returns pwrseq_rb_retry()'s result, or -ENODEV before start.
 */
int sts_pwrseq_rb_retry(uint32_t now_ms);

/** RB_LOCK (PB13) as a logical "the FE reports lock", polarity applied. */
bool sts_pwrseq_rb_lock(void);

/**
 * Drive ANT_BIAS_EN (PC9) on gnssmgr's behalf.
 *
 * The antenna supervisor decides to cut the bias on a persistent short, but
 * pwrseq_exec.c is the single writer of that pin (ARCHITECTURE.md §10), so the
 * request is routed here rather than driven from the gnss thread.
 */
void sts_pwrseq_ant_bias_request(bool on);

/** Stage 9: assert WDT_EN and begin the kick cadence. */
int sts_supervisor_arm(void);

/** Blink the status RGB blue for @p duration_ms (operator locate). */
void sts_supervisor_identify(uint32_t duration_ms);

void sts_supervisor_counters(uint32_t *kicks, uint32_t *withheld, bool *armed);

/* ------------------------------------------------------------------------- */
/* io_scan.c                                                                  */
/* ------------------------------------------------------------------------- */

int sts_io_scan_start(void);

/** Scans completed and scans that overran their 1 ms slot. */
void sts_io_scan_counters(uint32_t *scans, uint32_t *overruns);

/* ------------------------------------------------------------------------- */
/* disc_thread.c                                                              */
/* ------------------------------------------------------------------------- */

int sts_discipline_start(void);

/** PFI handler: park the loop and freeze the DAC. ISR-safe. */
void sts_discipline_park(void);

/**
 * Ask the discipline thread to leave a latched park.
 *
 * A flag, not a call into core/disc: the discipline thread is the only writer of
 * the loop context (ARCHITECTURE.md §10.2/§10.10), so the PFI recovery path in
 * the housekeeping thread must not touch it directly. Safe from any thread.
 */
void sts_discipline_unpark_request(void);

/* refsel handoff bracket: transient park/unpark around a mux flip, called by
 * the clock-mux executor on REFSEL_ACT_PARK/UNPARK_DISCIPLINE. Discipline
 * thread context only. */
void sts_disc_handoff_park(void);
void sts_disc_handoff_unpark(void);

/* ------------------------------------------------------------------------- */
/* pfi.c — PE8 power-fail early warning (EXTI8)                              */
/* ------------------------------------------------------------------------- */

int sts_pfi_init(void);

/** True once PFI has fired; the console area polls this to fast-save. */
bool sts_pfi_fired(void);

/* ------------------------------------------------------------------------- */
/* gnss.c — USART3 UBX link + core/gnssmgr (ARCHITECTURE.md §6, priority 6)   */
/* ------------------------------------------------------------------------- */

/** Everything the discipline loop and the sequencer need from the receiver. */
typedef struct {
	bool     have_status;      /* at least one NAV-PVT decoded */
	bool     time_locked;      /* fix usable for timing, tAcc inside window */
	bool     utc_valid;
	uint8_t  fix_type;         /* UBX_FIX_* */
	uint8_t  sv_used;
	uint8_t  sv_visible;
	uint32_t tacc_ns;
	int16_t  leap_current_s;   /* TAI-UTC offset, 0 when unknown */
	int8_t   leap_pending;     /* +1 insert, -1 delete, 0 none */
	uint64_t leap_at_tai_s;
	bool     leap_valid;
	bool     cfg_ack;          /* the config walk has passed its ACK gate */
	bool     cfg_failed;
	uint8_t  ant_state;        /* gnssmgr_ant_state_t */

	/* Pulse-pairing evidence: the NAV-PVT iTOW and when it was decoded. */
	uint32_t pvt_itow_ms;
	uint64_t pvt_rx_mono_ms;

	/* The latest UBX-TIM-TP, already normalised to GPS ToW by gnssmgr. */
	gnssmgr_qerr_t qerr;
} sts_gnss_snap_t;

/** Start the gnss thread and open USART3. Idempotent (-EALREADY). */
int sts_gnss_start(void);

/** Stage 5.5: begin (or restart) the UBX configuration walk. */
int sts_gnss_configure(uint32_t now_ms);

/** Stage 5.2: the receiver was just reset; its configuration is gone. */
int sts_gnss_notify_reset(uint32_t now_ms);

/** Stage 5.4: begin fusing the antenna supervisor's three evidence sources. */
void sts_gnss_ant_supervisor_start(void);

/** Copy the published receiver view. Returns 0, or -EINVAL/-ENODEV. */
int sts_gnss_snapshot(sts_gnss_snap_t *out);

/** True once the F9T has ACKed the essential configuration (stage-5 gate). */
bool sts_gnss_cfg_ack(void);

/* ------------------------------------------------------------------------- */
/* panel_pwm.c — LPTIM2_CH2 on PE0                                            */
/* ------------------------------------------------------------------------- */

int sts_panel_led_init(void);

/* ------------------------------------------------------------------------- */
/* rb_serial.c — UART7 (PB4/PE7), the K1 RS-232/CMOS relay, RB_LOCK           */
/* ------------------------------------------------------------------------- */

/** K1 DPDT relay position. LOW / RS-232 is the reset and default state. */
typedef enum {
	RB_SERIAL_MODE_RS232 = 0, /**< through the SN65C3221E level shifter (U46) */
	RB_SERIAL_MODE_CMOS,      /**< direct CMOS, for a variant that needs it */
	RB_SERIAL_MODE__COUNT,
} rb_serial_mode_t;

/** Settling time allowed for the K1 contacts after a mode change. */
#define RB_SERIAL_RELAY_SETTLE_MS 20

/**
 * Bring up UART7 and the two control pins.
 *
 * @param lock_active_low  RB_LOCK polarity for the fitted FE-5680A variant. The
 *                         board does not fix this; the variant does
 *                         (docs/rb_rs232_interface.md), so it is configuration.
 *
 * @retval 0        Ready, relay in the RS-232 position.
 * @retval -ENODEV  UART7 is not available.
 * @retval other    A GPIO could not be configured.
 */
int rb_serial_init(bool lock_active_low);

/**
 * Move the K1 relay. Blocks for RB_SERIAL_RELAY_SETTLE_MS.
 *
 * @retval 0        Moved.
 * @retval -EINVAL  Bad mode.
 * @retval -EBUSY   A raw tunnel holds the port.
 * @retval -ENODEV  Not initialised.
 */
int rb_serial_set_mode(uint8_t mode);

/** Current relay position; rb_serial_mode_t. */
uint8_t rb_serial_mode(void);

/**
 * True when RB_PWR_EN is asserted, i.e. the SN65C3221E has a supply.
 *
 * With this false there is nothing on the far side of the relay: the level
 * shifter is powered from the Rb domain (docs/rb_rs232_interface.md).
 */
bool rb_serial_rail_up(void);

/** RB_LOCK (PB13), with the configured variant polarity applied. */
bool rb_serial_locked(void);

/** The core/fwupd rb_fwupd port bound to UART7. Never NULL. */
const rb_fwupd_ops_t *rb_serial_ops(void);

/**
 * Raw byte tunnel, for a maintenance tool that needs to speak to a variant this
 * firmware does not recognise.
 *
 * The tunnel *channel* — its framing, its authorisation, the command that
 * carries it — belongs to core/mp and is owned by another area. This is the
 * whole seam that side needs.
 *
 * @{
 */

/** Called from the UART ISR with every octet that arrives. */
typedef void (*rb_serial_tunnel_cb_t)(void *user, const uint8_t *data, size_t len);

/**
 * Take the port. Suspends this file's own use of it; rb_serial_ops()'s transmit
 * path then returns -EBUSY.
 *
 * @retval 0        Tunnel open.
 * @retval -EINVAL  @p cb is NULL.
 * @retval -EBUSY   A tunnel is already open.
 * @retval -ENODEV  Not initialised.
 */
int rb_serial_tunnel_open(rb_serial_tunnel_cb_t cb, void *user);

/**
 * Put octets on the wire.
 *
 * @retval 0        Queued.
 * @retval -EPERM   No tunnel is open.
 * @retval -ENODEV  Not initialised, or the Rb rail is down.
 */
int rb_serial_tunnel_write(const uint8_t *data, size_t len);

/** Give the port back. Idempotent. */
int rb_serial_tunnel_close(void);

/** True while a tunnel holds the port. */
bool rb_serial_tunnel_active(void);

/** @} */

/** Byte counters and receive-overrun count. Any pointer may be NULL. */
void rb_serial_stats(uint32_t *tx, uint32_t *rx, uint32_t *overruns);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_PLATFORM_PLATFORM_H_ */
