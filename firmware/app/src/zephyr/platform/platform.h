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

#include "fault/fault.h"
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

/** Write the digipot wiper (0..255) and read it back. */
int sts_digipot_set(uint8_t code);

/** Read the digipot volatile wiper register. */
int sts_digipot_get(uint8_t *code);

/** Program the non-volatile wiper so a POR comes up at @p code. */
int sts_digipot_set_nv(uint8_t code);

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
	int32_t  humidity_mpct;     /* SHT45 0x44, milli-percent RH */
	int32_t  temp_sht_mc;
	bool     temp_enclosure_valid;
	bool     temp_osc_valid;
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

/** Called at 4 Hz from the housekeeping thread. */
void sts_supervisor_step(uint32_t now_ms);

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

/* ------------------------------------------------------------------------- */
/* pfi.c — PE8 power-fail early warning (EXTI8)                              */
/* ------------------------------------------------------------------------- */

int sts_pfi_init(void);

/** True once PFI has fired; the console area polls this to fast-save. */
bool sts_pfi_fired(void);

/* ------------------------------------------------------------------------- */
/* panel_pwm.c — LPTIM2_CH2 on PE0                                            */
/* ------------------------------------------------------------------------- */

int sts_panel_led_init(void);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_PLATFORM_PLATFORM_H_ */
