/*
 * STS1000 "Meridian" — housekeeping thread (ARCHITECTURE.md §6, priority 14).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Everything that is periodic, I2C-bound and not timing-critical:
 *
 *   4 Hz  drain INA228 re-read requests raised by the io_scan thread
 *   1 Hz  sweep all nine INA228 monitors, both TMP117s and the SHT45
 *   1 Hz  close the EXTREF_MON gate
 *   1 Hz  thermal_step_1hz() -> TIM15_CH1 fan duty, FAN_TACH -> RPM
 *   4 Hz  pwrseq_step() and its action list
 *   4 Hz  supervisor: liveness AND-gate -> WDT_KICK, holdover relay, RGB
 *
 * The readings cache is the only shared state and is mutex-protected. The
 * discipline thread reads it for the OCXO temperature and rail current; the
 * status encoders read it for telemetry. Neither is timing-critical in the
 * §1.2 sense — the values are 1 Hz environmental data, not phase — so a mutex
 * here does not violate the "no timing locks in management threads" rule.
 *
 * ---------------------------------------------------------------------------
 * INA228 access
 * ---------------------------------------------------------------------------
 * The nine monitors are driven as raw I2C: core/ina228 owns the register codec
 * and the per-rail constants, this file owns the transfers. Two invariants
 * from ARCHITECTURE.md §10.7 are enforced here rather than assumed:
 *
 *   - SHUNT_CAL (4096, scaled by the per-board trim) is written at stage 3 and
 *     re-written whenever a device reports it has been reset. A freshly reset
 *     INA228 reverts to POR calibration silently, so every reading after an
 *     unnoticed reset would be wrong by the trim factor. MEMSTAT in DIAG_ALRT
 *     is the detector; `cal_ok` in the cache is the consumer-visible answer,
 *     and disc refuses the OCXO current unless it is set.
 *   - The GPS monitor is at 0x4A. SHT45 owns 0x44. Both come from
 *     ina228_rail_tbl[], never from a literal here.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include "zephyr/platform/platform.h"
#include "zephyr/sts_app.h"

#include "cal/cal.h"
#include "cfg/cfg.h"
#include "thermal/thermal.h"

LOG_MODULE_REGISTER(sts_hk, CONFIG_STS1000_LOG_LEVEL);

/*
 * sts_atecc_reapply() applies `sec.atecc.en` / `sec.atecc.slot` to the secure
 * element. It is implemented in the console-gated storage area
 * (src/zephyr/storage/sts_atecc.c), so the reference is WEAK — the same idiom
 * sts_web.c uses for sts_dfu_port() — and with CONFIG_STS1000_CONSOLE=n it
 * resolves to NULL: that image has no cfg persistence and no secure-element
 * services, so there is nothing to configure.
 *
 * Declared here rather than by including the area's header, so the always-built
 * platform area gains no hard link dependency on an optional area.
 */
extern void sts_atecc_reapply(void) __attribute__((weak));

#define HK_STACK_SIZE 4096
#define HK_PRIO       14   /* ARCHITECTURE.md §6 */
#define HK_PERIOD_MS  250  /* 4 Hz base tick; the 1 Hz work runs every 4th */

K_THREAD_STACK_DEFINE(hk_stack, HK_STACK_SIZE);
static struct k_thread hk_tcb;

static const struct device *const i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));

/* INA228 register numbers come from core/ina228 (INA228_REG_*); this file owns
 * only the transfers. */

/* ---- other I2C devices --------------------------------------------------- */

#define TMP117_ENCLOSURE_ADDR 0x48U
#define TMP117_OSC_ADDR       0x49U
#define TMP117_REG_TEMP       0x00U

#define SHT45_ADDR            0x44U
#define SHT45_CMD_MEAS_HIGH   0xFDU
#define SHT45_MEAS_MS         10

/* ---- fan ----------------------------------------------------------------- */

#define FAN_PWM_NODE DT_NODELABEL(panel_pwm)
#define FAN_PWM_CHANNEL 1  /* TIM15_CH1 on PE5 */
#define FAN_PWM_PERIOD_NS (1000000000U / 25000U) /* 25 kHz */

static const struct device *const fan_pwm = DEVICE_DT_GET(FAN_PWM_NODE);
static const struct gpio_dt_spec fan_tach = STS_USER_GPIO(fan_tach_gpios);

/* STM32 internal die-temperature sensor — thermal ladder tertiary fallback. */
#define DIE_TEMP_NODE DT_NODELABEL(die_temp)
#if DT_NODE_HAS_STATUS(DIE_TEMP_NODE, okay)
static const struct device *const die_temp = DEVICE_DT_GET(DIE_TEMP_NODE);
#else
static const struct device *const die_temp;
#endif

/* ---- module state -------------------------------------------------------- */

static sts_hk_snapshot_t hk_cache;
static K_MUTEX_DEFINE(hk_mutex);

static thermal_ctx_t thermal;

static struct {
	uint16_t shunt_cal[INA228_RAIL_COUNT];
	atomic_t ina_request_mask;
	int liveness_id;
	bool started;
	bool i2c_ok;

	struct gpio_callback tach_cb;
	atomic_t tach_edges;
	uint32_t tach_last_ms;

	/*
	 * core/thermal's escalation requests, cached from the 1 Hz step for the
	 * 4 Hz sequencer. Rungs 2 and 3 of ARCHITECTURE.md §5 / spec §13 —
	 * "fan max -> shed Rb -> POE_KILL" — which used to become alarm *bits* and
	 * nothing else: nothing consumed request_rb_shed at all, so a stalled fan
	 * past the kill threshold lit an LED and sent a trap while the ~12 W
	 * rubidium kept running and the board never cold-cycled.
	 */
	atomic_t thermal_shed_rb;
	atomic_t thermal_poe_kill;

	/*
	 * Set by the group-0x0C config applier, drained by this thread. A commit
	 * of cal.ina.* used to reach the parts only at the next reboot or the
	 * next MEMSTAT recovery, so a freshly trimmed board kept reporting on the
	 * old calibration until someone power-cycled it. Deferred rather than
	 * applied in the applier because appliers run on the COMMITTING thread —
	 * which is a web worker or the shell — and nine I2C writes do not belong
	 * there (sts_app.h: "must be quick or defer to the area's own thread").
	 */
	atomic_t ina_cal_reload;
} hk;

/* ========================================================================= */
/* raw I2C helpers                                                           */
/* ========================================================================= */

static int ina_read(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
	return i2c_write_read(i2c1, addr, &reg, 1U, buf, len);
}

static int ina_write16(uint8_t addr, uint8_t reg, uint16_t val)
{
	uint8_t frame[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFFU) };

	return i2c_write(i2c1, frame, sizeof(frame), addr);
}

static int ina_read16(uint8_t addr, uint8_t reg, uint16_t *val)
{
	uint8_t buf[2];
	int rc = ina_read(addr, reg, buf, sizeof(buf));

	if (rc == 0) {
		*val = ina228_raw16(buf);
	}

	return rc;
}

/* ========================================================================= */
/* INA228 configuration                                                      */
/* ========================================================================= */

/*
 * SHUNT_CAL = 4096 on every rail before trimming. Setting Max_Expected_Current
 * to each device's own full scale makes the shunt resistance cancel out of the
 * TI calibration formula, so one constant serves all nine and the per-rail
 * resolution comes from CURRENT_LSB = 78.125 nV / R_SHUNT instead.
 *
 * config group 0x0C holds the per-board trimmed value itself — keys
 * CAL_INA_TRIM_0..8 at 0x0C01..0x0C09, in ina228_rail_t order, defaulting to
 * 4096 and bounded to +-20 % by the schema. A board that has never been
 * calibrated therefore comes up on the untrimmed constant rather than on
 * nothing, and the bench trim (interface ref §4.2) only has to write the keys.
 */
#define INA228_SHUNT_CAL_BASE 4096U
#define CFG_KEY_CAL_INA_TRIM_0 0x0C01U

static uint16_t hk_shunt_cal_for(ina228_rail_t rail)
{
	uint16_t id = (uint16_t)(CFG_KEY_CAL_INA_TRIM_0 + (uint16_t)rail);
	uint64_t cal = 0;

	if (cfg_get_u64(sts_cfg(), id, &cal) != 0 || cal == 0U || cal > 0x7FFFU) {
		return INA228_SHUNT_CAL_BASE;
	}

	return (uint16_t)cal;
}

/** Write CONFIG/ADC_CONFIG/SHUNT_CAL/SOVL/SUVL for one monitor. */
static int hk_ina_configure(ina228_rail_t rail)
{
	const ina228_rail_info_t *info = &ina228_rail_tbl[rail];
	ina228_config_t cfg;
	ina228_adc_config_t adc;
	int rc;

	/* Board defaults from core/ina228: ADCRANGE=1 (+-40.96 mV) on all nine,
	 * because every shunt was sized against that full scale, and continuous
	 * bus+shunt+temperature averaging slower than both the 1 kHz scan and
	 * the panel-LED PWM period (interface ref §4.1, §4.2). */
	ina228_config_default(&cfg);
	ina228_adc_config_default(&adc);

	rc = ina_write16(info->addr, INA228_REG_CONFIG, ina228_config_encode(&cfg));
	if (rc != 0) {
		return rc;
	}

	rc = ina_write16(info->addr, INA228_REG_ADC_CONFIG,
			 ina228_adc_config_encode(&adc));
	if (rc != 0) {
		return rc;
	}

	rc = ina_write16(info->addr, INA228_REG_SHUNT_CAL, hk.shunt_cal[rail]);
	if (rc != 0) {
		return rc;
	}

	rc = ina_write16(info->addr, INA228_REG_SOVL, (uint16_t)info->sovl_code_default);
	if (rc != 0) {
		return rc;
	}

	return ina_write16(info->addr, INA228_REG_SUVL,
			   (uint16_t)info->suvl_code_default);
}

int sts_hk_ina_configure_all(void)
{
	int configured = 0;

	if (!device_is_ready(i2c1)) {
		LOG_ERR("I2C1 not ready");
		return -ENODEV;
	}

	for (size_t i = 0; i < INA228_RAIL_COUNT; i++) {
		const ina228_rail_info_t *info = &ina228_rail_tbl[i];
		uint16_t manuf = 0;
		uint16_t devid = 0;
		int rc;

		hk.shunt_cal[i] = hk_shunt_cal_for((ina228_rail_t)i);

		rc = ina_read16(info->addr, INA228_REG_MANUFACTURER_ID, &manuf);
		if (rc == 0) {
			rc = ina_read16(info->addr, INA228_REG_DEVICE_ID, &devid);
		}

		if (rc != 0 || !ina228_id_matches(manuf, devid)) {
			LOG_ERR("INA228 %s (%s @ 0x%02x): absent or wrong ID "
				"(rc %d, manuf 0x%04x, dev 0x%04x)",
				info->name, info->designator, info->addr, rc, manuf,
				devid);
			continue;
		}

		rc = hk_ina_configure((ina228_rail_t)i);
		if (rc != 0) {
			LOG_ERR("INA228 %s: configure failed (%d)", info->name, rc);
			continue;
		}

		k_mutex_lock(&hk_mutex, K_FOREVER);
		hk_cache.ina[i].cal_ok = true;
		k_mutex_unlock(&hk_mutex);

		configured++;
	}

	LOG_INF("INA228: %d/%d monitors configured (SHUNT_CAL base %u)", configured,
		(int)INA228_RAIL_COUNT, INA228_SHUNT_CAL_BASE);

	hk.i2c_ok = (configured > 0);

	return (configured == (int)INA228_RAIL_COUNT) ? 0 : -EIO;
}

/* ---- per-board trim reload (group 0x0C) --------------------------------- */

/** Config applier for the cal group. Runs on the committing thread: flag only. */
static void hk_cal_applier(void *ctx, uint8_t group)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(group);

	atomic_set(&hk.ina_cal_reload, 1);
}

/**
 * Push any changed cal.ina.* trim into its part. Housekeeping-thread context.
 *
 * Only rails already flagged `cal_ok` are touched. A rail that is absent or
 * that never accepted its configuration needs the full CONFIG/ADC_CONFIG/SOVL
 * walk, not a lone SHUNT_CAL write, and that is what the MEMSTAT recovery path
 * in hk_read_ina() does; writing one register here would flip `cal_ok` true on
 * a part that is not otherwise set up.
 */
static void hk_reload_shunt_cal(void)
{
	if (atomic_set(&hk.ina_cal_reload, 0) == 0) {
		return;
	}

	for (size_t i = 0; i < INA228_RAIL_COUNT; i++) {
		const ina228_rail_info_t *info = &ina228_rail_tbl[i];
		uint16_t want = hk_shunt_cal_for((ina228_rail_t)i);
		bool ok;

		if (want == hk.shunt_cal[i]) {
			continue;
		}

		k_mutex_lock(&hk_mutex, K_FOREVER);
		ok = hk_cache.ina[i].cal_ok;
		k_mutex_unlock(&hk_mutex);
		if (!ok) {
			continue;
		}

		if (ina_write16(info->addr, INA228_REG_SHUNT_CAL, want) != 0) {
			LOG_ERR("INA228 %s: SHUNT_CAL %u write failed", info->name,
				want);
			k_mutex_lock(&hk_mutex, K_FOREVER);
			hk_cache.ina[i].cal_ok = false;
			k_mutex_unlock(&hk_mutex);
			continue;
		}

		hk.shunt_cal[i] = want;
		sts_log(LOGR_SUB_PWR, LOGR_NOTICE, "INA228 %s: SHUNT_CAL -> %u",
			info->name, want);
	}
}

/* ---- bench calibration (sts_app.h sts_cal_run) --------------------------- */

/*
 * Oldest INA228 reading a trim will accept. The 1 Hz sweep refreshes every
 * rail, so 2 s tolerates one missed sweep and nothing more: a trim is a ratio
 * against a load the operator is holding steady *now*, and a stale reading
 * would silently calibrate against whatever the board was doing before.
 */
#define HK_CAL_INA_MAX_AGE_MS 2000U

static int hk_cal_ina_trim(uint32_t arg)
{
	const ina228_rail_info_t *info;
	sts_hk_snapshot_t snap;
	cal_ina_in_t in;
	cal_ina_out_t out;
	uint32_t rail = (arg >> 24) & 0xFFU;
	uint32_t i_ref_ua = arg & 0x00FFFFFFU;
	uint16_t key;
	int rc;

	if ((rail >= (uint32_t)INA228_RAIL_COUNT) || (i_ref_ua == 0U)) {
		return -EINVAL;
	}
	info = &ina228_rail_tbl[rail];

	if (hk.shunt_cal[rail] == 0U) {
		/* Stage 3 never configured this part, so there is no calibration
		 * in force to trim from and the reading means nothing. */
		return -ENODATA;
	}

	(void)sts_hk_read(&snap);

	memset(&in, 0, sizeof(in));
	in.base_shunt_cal = hk.shunt_cal[rail];
	in.i_ref_ua = i_ref_ua;
	in.i_meas_ua = snap.ina[rail].current_ua;
	in.meas_valid = snap.ina[rail].valid && snap.ina[rail].cal_ok;
	in.meas_age_ms = k_uptime_get_32() - snap.ina[rail].age_ms;
	in.max_age_ms = HK_CAL_INA_MAX_AGE_MS;
	in.fs_current_ua = info->fs_current_ua;
	in.trim_min = (uint16_t)CAL_INA_TRIM_MIN;
	in.trim_max = (uint16_t)CAL_INA_TRIM_MAX;

	rc = cal_ina_trim(&in, &out);
	if (rc != 0) {
		sts_log(LOGR_SUB_PWR, LOGR_WARN,
			"INA trim %s refused: %s (ref %u uA, read %d uA, %d ppm)",
			info->name, cal_ina_reason_name(out.reason),
			(unsigned int)i_ref_ua, (int)in.i_meas_ua,
			(int)out.error_ppm);
		return rc;
	}

	/*
	 * Persist. A calibration constant that evaporates on the next reboot is
	 * worse than none, because the operator has been told it took — so this
	 * commits rather than leaving the key staged, and the REST response
	 * carries no "commit required" field to say otherwise.
	 *
	 * Committing is a whole-tree operation, so refuse when anything else is
	 * staged rather than sweeping an operator's half-finished config edit
	 * into a calibration run. (The window between this check and the commit
	 * is the same one every commit path on the box has; it is not widened
	 * here.)
	 */
	key = (uint16_t)(CFG_KEY_CAL_INA_TRIM_0 + rail);

	sts_cfg_lock();
	if (cfg_staged_count(sts_cfg()) != 0U) {
		sts_cfg_unlock();
		return -EBUSY;
	}
	rc = cfg_set_u64(sts_cfg(), key, (uint64_t)out.shunt_cal);
	sts_cfg_unlock();

	if (rc != 0) {
		/* The band checks above are the schema's own, so a bounds
		 * rejection here would mean the two have drifted apart. */
		LOG_ERR("cal.ina.%u: staging %u failed (%d)", (unsigned int)rail,
			out.shunt_cal, rc);
		return -EIO;
	}

	/* Not under sts_cfg_lock(): sts_cfg_commit() takes it itself and then
	 * dispatches appliers with it released (sts_app.h). */
	rc = sts_cfg_commit(NULL);
	if ((rc != 0) && (rc != -EIO)) {
		sts_cfg_lock();
		(void)cfg_revert(sts_cfg());
		sts_cfg_unlock();
		LOG_ERR("cal.ina.%u: commit failed (%d)", (unsigned int)rail, rc);
		return -EIO;
	}

	/* hk_cal_applier() has already flagged the reload; the part picks the new
	 * SHUNT_CAL up on this thread's next tick. */
	sts_log(LOGR_SUB_PWR, LOGR_NOTICE,
		"INA trim %s: SHUNT_CAL %u -> %u (%d ppm against %u uA)",
		info->name, in.base_shunt_cal, out.shunt_cal, (int)out.error_ppm,
		(unsigned int)i_ref_ua);

	return (rc == -EIO) ? -EIO : 0;
}

int sts_cal_run(uint8_t proc, uint32_t arg)
{
	switch (proc) {
	case (uint8_t)STS_CAL_INA_TRIM:
		return hk_cal_ina_trim(arg);

	case (uint8_t)STS_CAL_HOLDOVER:
	case (uint8_t)STS_CAL_OCXO_TUNE:
	case (uint8_t)STS_CAL_COMPASS:
		/*
		 * Bench operations, not device procedures. Holdover
		 * characterisation is hours of GNSS-denied running against a
		 * reference clock; the OCXO pull/tempco sweep is a thermal
		 * chamber; the e-compass fit needs the enclosure rotated through
		 * a sphere — and this build has no magnetometer sampling path at
		 * all (the IIS2MDC and LIS2DH12 are in the devicetree and no glue
		 * reads them) nor any cfg key to store a hard/soft-iron fit in.
		 *
		 * Their results are entered through the group 0x0C keys, which
		 * the config API already exposes. A firmware entry point that
		 * wrote one of those keys would be a config set with a
		 * procedure's name on it, and would report success for work
		 * nobody did.
		 */
		return -ENOTSUP;

	default:
		return -EINVAL;
	}
}

/* ========================================================================= */
/* sweeps                                                                    */
/* ========================================================================= */

static void hk_read_ina(ina228_rail_t rail, bool with_diag)
{
	const ina228_rail_info_t *info = &ina228_rail_tbl[rail];
	sts_ina_reading_t r = { 0 };
	uint8_t buf[3];
	uint32_t current_lsb_pa;
	uint32_t power_lsb_nw;
	int32_t current_raw;
	int rc;

	rc = ina_read(info->addr, INA228_REG_VBUS, buf, 3U);
	if (rc != 0) {
		goto out;
	}
	/* VBUS is a 20-bit unsigned value in the top of a 24-bit field, LSB
	 * 195.3125 uV. The 54 V PoE bus and the 24 V VCC_RB therefore read
	 * directly with no external divider (interface ref §4.2). */
	r.bus_uv = (int32_t)(((uint64_t)ina228_raw20_unsigned(buf) * 1953125ULL) /
			     10000ULL);

	rc = ina_read(info->addr, INA228_REG_CURRENT, buf, 3U);
	if (rc != 0) {
		goto out;
	}
	current_raw = (int32_t)ina228_raw20_unsigned(buf);
	if (current_raw >= 0x80000) {
		current_raw -= 0x100000; /* 20-bit two's complement */
	}
	current_lsb_pa = ina228_current_lsb_pa(info->r_shunt_uohm, INA228_ADCRANGE_40_96MV);
	r.current_ua = (int32_t)(((int64_t)current_raw * (int64_t)current_lsb_pa) /
				 1000000LL);

	rc = ina_read(info->addr, INA228_REG_POWER, buf, 3U);
	if (rc != 0) {
		goto out;
	}
	power_lsb_nw = ina228_power_lsb_nw(info->r_shunt_uohm, INA228_ADCRANGE_40_96MV);
	r.power_uw = (uint32_t)(((uint64_t)ina228_raw24(buf) * (uint64_t)power_lsb_nw) /
				1000ULL);

	r.valid = true;

out:
	k_mutex_lock(&hk_mutex, K_FOREVER);
	r.cal_ok = hk_cache.ina[rail].cal_ok;
	r.diag_alrt = hk_cache.ina[rail].diag_alrt;
	r.age_ms = k_uptime_get_32();
	hk_cache.ina[rail] = r;
	k_mutex_unlock(&hk_mutex);

	if (!with_diag) {
		return;
	}

	{
		uint16_t diag = 0;
		ina228_diag_t d;

		if (ina_read16(info->addr, INA228_REG_DIAG_ALRT, &diag) != 0) {
			return;
		}

		k_mutex_lock(&hk_mutex, K_FOREVER);
		hk_cache.ina[rail].diag_alrt = diag;
		k_mutex_unlock(&hk_mutex);

		if (ina228_diag_decode(diag, &d) != 0) {
			return;
		}

		/*
		 * MEMSTAT clear means the device's own checksum over its
		 * configuration failed — the part has effectively reset. Its
		 * SHUNT_CAL is back at POR and every subsequent current
		 * reading would be silently mis-scaled, so re-configure before
		 * anything is allowed to trust it again (ARCHITECTURE.md
		 * §10.7).
		 */
		if (!d.memstat) {
			LOG_WRN("INA228 %s: MEMSTAT lost, re-applying configuration",
				info->name);

			k_mutex_lock(&hk_mutex, K_FOREVER);
			hk_cache.ina[rail].cal_ok = false;
			k_mutex_unlock(&hk_mutex);

			if (hk_ina_configure(rail) == 0) {
				k_mutex_lock(&hk_mutex, K_FOREVER);
				hk_cache.ina[rail].cal_ok = true;
				k_mutex_unlock(&hk_mutex);
			}
		}

		if (ina228_diag_fault_mask(diag) != 0U) {
			sts_log(LOGR_SUB_PWR, LOGR_WARN,
				"INA228 %s DIAG_ALRT 0x%04x", info->name, diag);
		}
	}
}

static bool hk_read_tmp117(uint8_t addr, int32_t *mc)
{
	uint8_t buf[2];
	int16_t raw;

	if (i2c_burst_read(i2c1, addr, TMP117_REG_TEMP, buf, sizeof(buf)) != 0) {
		return false;
	}

	raw = (int16_t)sys_get_be16(buf);
	/* TMP117 LSB = 7.8125 m°C = 1/128 °C. */
	*mc = ((int32_t)raw * 78125) / 10000;

	return true;
}

static bool hk_read_sht45(int32_t *rh_mpct, int32_t *t_mc)
{
	uint8_t cmd = SHT45_CMD_MEAS_HIGH;
	uint8_t buf[6];

	if (i2c_write(i2c1, &cmd, 1U, SHT45_ADDR) != 0) {
		return false;
	}

	k_msleep(SHT45_MEAS_MS);

	if (i2c_read(i2c1, buf, sizeof(buf), SHT45_ADDR) != 0) {
		return false;
	}

	/* Datasheet §4.6: T = -45 + 175 * raw/65535, RH = -6 + 125 * raw/65535,
	 * with the CRC bytes at [2] and [5] left to the bus's own integrity
	 * (a corrupted reading is environmental data, not a control input). */
	*t_mc = -45000 + (int32_t)(((int64_t)sys_get_be16(&buf[0]) * 175000LL) / 65535LL);
	*rh_mpct = -6000 + (int32_t)(((int64_t)sys_get_be16(&buf[3]) * 125000LL) / 65535LL);

	if (*rh_mpct < 0) {
		*rh_mpct = 0;
	} else if (*rh_mpct > 100000) {
		*rh_mpct = 100000;
	}

	return true;
}

/* ========================================================================= */
/* fan                                                                       */
/* ========================================================================= */

static void hk_tach_isr(const struct device *port, struct gpio_callback *cb,
			gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	atomic_inc(&hk.tach_edges);
}

/** Edges since the last call, converted to RPM. Two pulses per revolution. */
static uint32_t hk_tach_rpm(uint32_t now_ms)
{
	uint32_t edges = (uint32_t)atomic_set(&hk.tach_edges, 0);
	uint32_t dt_ms = now_ms - hk.tach_last_ms;

	hk.tach_last_ms = now_ms;

	if (dt_ms == 0U) {
		return 0U;
	}

	/* Standard 4-wire fan: 2 tach pulses per revolution, and the callback
	 * fires on one edge per pulse. */
	return (uint32_t)(((uint64_t)edges * 60000ULL) / ((uint64_t)dt_ms * 2ULL));
}

static void hk_fan_set_duty(uint8_t duty_pct)
{
	uint32_t pulse_ns;
	int rc;

	if (duty_pct > 100U) {
		duty_pct = 100U;
	}

	pulse_ns = ((uint32_t)duty_pct * FAN_PWM_PERIOD_NS) / 100U;

	rc = pwm_set(fan_pwm, FAN_PWM_CHANNEL, FAN_PWM_PERIOD_NS, pulse_ns, 0);
	if (rc != 0) {
		LOG_ERR("FAN_PWM set %u%% failed (%d)", duty_pct, rc);
	}
}

/* ========================================================================= */
/* the thread                                                                */
/* ========================================================================= */

void sts_hk_request_ina(uint8_t rail_idx)
{
	if (rail_idx >= INA228_RAIL_COUNT) {
		return;
	}

	atomic_or(&hk.ina_request_mask, (atomic_val_t)BIT(rail_idx));
}

int sts_hk_read_ina_now(uint8_t rail_idx)
{
	if (rail_idx >= INA228_RAIL_COUNT) {
		return -EINVAL;
	}
	if (!device_is_ready(i2c1)) {
		return -EIO;
	}

	hk_read_ina((ina228_rail_t)rail_idx, true);

	return hk_cache.ina[rail_idx].valid ? 0 : -EIO;
}

void sts_hk_thermal_requests(bool *shed_rb, bool *poe_kill)
{
	if (shed_rb != NULL) {
		*shed_rb = atomic_get(&hk.thermal_shed_rb) != 0;
	}
	if (poe_kill != NULL) {
		*poe_kill = atomic_get(&hk.thermal_poe_kill) != 0;
	}
}

int sts_hk_read(sts_hk_snapshot_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}

	(void)k_mutex_lock(&hk_mutex, K_FOREVER);
	*out = hk_cache;
	(void)k_mutex_unlock(&hk_mutex);

	return 0;
}

int sts_health_snapshot(sts_health_t *out)
{
	sts_hk_snapshot_t hk;
	uint64_t budget_mw = 0;

	if (out == NULL) {
		return -EINVAL;
	}

	(void)sts_hk_read(&hk);

	memset(out, 0, sizeof(*out));
	out->ver = STS_HEALTH_VER;
	out->ina_count = STS_HEALTH_INA_COUNT;

	for (size_t r = 0; r < INA228_RAIL_COUNT && r < STS_HEALTH_INA_COUNT; r++) {
		out->ina[r].bus_mv = hk.ina[r].bus_uv / 1000;
		out->ina[r].current_ua = hk.ina[r].current_ua;
		out->ina[r].power_uw = hk.ina[r].power_uw;
		out->ina[r].diag_alrt = hk.ina[r].diag_alrt;
		out->ina[r].valid = hk.ina[r].valid && hk.ina[r].cal_ok;
	}

	out->tmp_osc_mc = hk.temp_osc_mc;
	out->tmp_osc_valid = hk.temp_osc_valid;
	out->tmp_amb_mc = hk.temp_enclosure_mc;
	out->tmp_amb_valid = hk.temp_enclosure_valid;
	out->die_mc = hk.die_mc;
	out->die_valid = hk.die_valid;
	out->humidity_mpct = hk.humidity_mpct;
	out->humidity_valid = hk.sht_valid;

	out->fan_rpm = hk.fan_rpm;
	out->fan_duty_pct = hk.fan_duty_pct;

	/*
	 * poe_draw_mw from the PoE-input monitor (INA228 0x40, which reads the
	 * 54 V bus directly, no divider); poe_budget_mw from the configured
	 * grant. poe_class stays 0/unknown: the NCP1095 negotiates the class in
	 * hardware and the NCM/NCL/LCF pins encode it, but decoding them is not
	 * wired yet (documented TODO).
	 */
	if (hk.ina[INA228_RAIL_POE].valid) {
		int32_t mv = hk.ina[INA228_RAIL_POE].bus_uv / 1000;
		int32_t ma = hk.ina[INA228_RAIL_POE].current_ua / 1000;

		if (mv > 0 && ma > 0) {
			out->poe_draw_mw = (uint32_t)(((int64_t)mv * ma) / 1000);
		}
	}
	(void)cfg_get_u64(sts_cfg(), (uint16_t)(0x0700U | 0x04U), &budget_mw);
	out->poe_budget_mw = (uint32_t)budget_mw;
	out->poe_class = 0U;

	sts_fault_lock();
	out->bkp_stm_pg = !fault_asserted(sts_fault(), FAULT_SIG_BKP_STM_PG);
	out->bkp_gps_pg = !fault_asserted(sts_fault(), FAULT_SIG_BKP_GPS_PG);
	sts_fault_unlock();

	out->mono_ms = hk.mono_ms;

	return 0;
}

static void hk_service_requests(void)
{
	atomic_val_t mask = atomic_set(&hk.ina_request_mask, 0);

	for (size_t i = 0; i < INA228_RAIL_COUNT; i++) {
		if ((mask & BIT(i)) != 0) {
			hk_read_ina((ina228_rail_t)i, true);
		}
	}
}

static void hk_sweep_1hz(uint32_t now_ms)
{
	int32_t enc_mc = 0;
	int32_t osc_mc = 0;
	bool enc_ok;
	bool osc_ok;
	int32_t rh;
	int32_t t;

	for (size_t i = 0; i < INA228_RAIL_COUNT; i++) {
		hk_read_ina((ina228_rail_t)i, false);
	}

	/*
	 * Both TMP117 transfers happen OUTSIDE hk_mutex, as the SHT45 read below
	 * already did. Holding the cache lock across blocking I2C made every
	 * sts_hk_read() caller wait on the bus — including the priority-4
	 * discipline thread, which takes it once a second in disc_fill_env(), and
	 * now the web thread's calibration path. The lock exists to keep the
	 * cache self-consistent, not to serialise transfers.
	 */
	enc_ok = hk_read_tmp117(TMP117_ENCLOSURE_ADDR, &enc_mc);
	osc_ok = hk_read_tmp117(TMP117_OSC_ADDR, &osc_mc);

	k_mutex_lock(&hk_mutex, K_FOREVER);

	if (enc_ok) {
		hk_cache.temp_enclosure_mc = enc_mc;
	}
	hk_cache.temp_enclosure_valid = enc_ok;

	if (osc_ok) {
		hk_cache.temp_osc_mc = osc_mc;
	}
	hk_cache.temp_osc_valid = osc_ok;

	hk_cache.mono_ms = now_ms;

	k_mutex_unlock(&hk_mutex);

	/* SHT45 measurement blocks for 10 ms; do it outside the lock. */
	if (hk_read_sht45(&rh, &t)) {
		k_mutex_lock(&hk_mutex, K_FOREVER);
		hk_cache.humidity_mpct = rh;
		hk_cache.temp_sht_mc = t;
		hk_cache.sht_valid = true;
		k_mutex_unlock(&hk_mutex);
	} else {
		k_mutex_lock(&hk_mutex, K_FOREVER);
		hk_cache.sht_valid = false;
		k_mutex_unlock(&hk_mutex);
	}
}

static void hk_thermal_1hz(uint32_t now_ms)
{
	sts_hk_snapshot_t snap;
	thermal_in_t in;
	thermal_out_t out;

	(void)sts_hk_read(&snap);

	memset(&in, 0, sizeof(in));
	in.mono_ms = now_ms;
	in.enclosure_mc = snap.temp_enclosure_mc;
	in.enclosure_valid = snap.temp_enclosure_valid;
	in.osc_mc = snap.temp_osc_mc;
	in.osc_valid = snap.temp_osc_valid;

	/*
	 * die_mc/die_valid are load-bearing, not telemetry: they are the
	 * thermal ladder's tertiary sensor when the enclosure TMP117 fails
	 * (Wave-2b contract). Read the STM32 die sensor every tick; if it is
	 * unavailable the ladder still has the oscillator TMP117 above it.
	 */
	in.die_valid = false;
	if (die_temp != NULL && device_is_ready(die_temp)) {
		struct sensor_value v;

		if (sensor_sample_fetch(die_temp) == 0 &&
		    sensor_channel_get(die_temp, SENSOR_CHAN_DIE_TEMP, &v) == 0) {
			in.die_mc = v.val1 * 1000 + v.val2 / 1000;
			in.die_valid = true;
		}
	}

	/* Cache the die reading too, so the health snapshot (sts_health_t)
	 * publishes the same value the ladder acted on. */
	k_mutex_lock(&hk_mutex, K_FOREVER);
	hk_cache.die_mc = in.die_mc;
	hk_cache.die_valid = in.die_valid;
	k_mutex_unlock(&hk_mutex);

	in.fan_rpm = (uint16_t)MIN(hk_tach_rpm(now_ms), (uint32_t)UINT16_MAX);
	in.rpm_valid = true;

	if (thermal_step_1hz(&thermal, &in, &out) != 0) {
		/* A loop that will not step is a loop that is not cooling;
		 * ARCHITECTURE.md §10.9 says the resting state is full speed. */
		hk_fan_set_duty(100U);
		return;
	}

	hk_fan_set_duty(out.duty_pct);

	k_mutex_lock(&hk_mutex, K_FOREVER);
	hk_cache.fan_rpm = in.fan_rpm;
	hk_cache.fan_duty_pct = out.duty_pct;
	k_mutex_unlock(&hk_mutex);

	(void)sts_alarm_set(FAULT_ALARM_THERMAL_WARN, out.alarm_overtemp);
	(void)sts_alarm_set(FAULT_ALARM_THERMAL_CRITICAL, out.request_poe_kill);
	(void)sts_alarm_set(FAULT_ALARM_FAN_FAULT, out.fan_stall);

	/*
	 * Rungs 2 and 3 reach an actuator: pwrseq_build_input() picks these up and
	 * pwrseq's shed policy drops the rubidium and, if the temperature keeps
	 * climbing, commands POE_KILL. Annunciating alone was the whole of the
	 * previous response, which meant the ladder had no bottom.
	 */
	atomic_set(&hk.thermal_shed_rb, out.request_rb_shed ? 1 : 0);
	atomic_set(&hk.thermal_poe_kill, out.request_poe_kill ? 1 : 0);
}

static void hk_entry(void *p1, void *p2, void *p3)
{
	uint32_t tick = 0;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	hk.tach_last_ms = k_uptime_get_32();

	for (;;) {
		uint32_t now_ms;

		k_msleep(HK_PERIOD_MS);
		now_ms = k_uptime_get_32();
		tick++;

		hk_service_requests();
		hk_reload_shunt_cal();

		if ((tick % (1000U / HK_PERIOD_MS)) == 0U) {
			hk_sweep_1hz(now_ms);
			sts_extref_mon_sample();
			hk_thermal_1hz(now_ms);
		}

		/* Latch bookkeeping, the ordered park list, and the recovery
		 * dwell that releases a spurious or ridden-out PFI edge. */
		sts_pfi_service(now_ms);

		/*
		 * pwrseq first (it may arm the watchdog and mark the relay
		 * eligible this tick), then the supervisor, which does the
		 * windowed WDT kick and drives the relay/RGB from the state
		 * pwrseq just set.
		 */
		sts_pwrseq_step(now_ms);
		sts_supervisor_step(now_ms);

		sts_liveness_feed(hk.liveness_id);
	}
}

int sts_hk_start(void)
{
	thermal_cfg_t cfg;
	k_tid_t tid;
	int rc;

	if (hk.started) {
		return -EALREADY;
	}

	if (!device_is_ready(fan_pwm)) {
		LOG_ERR("FAN_PWM (TIM15_CH1) not ready");
		return -ENODEV;
	}
	if (!gpio_is_ready_dt(&fan_tach)) {
		LOG_ERR("FAN_TACH not ready");
		return -ENODEV;
	}

	/*
	 * Full speed before the loop has an opinion. A fan that is not being
	 * commanded must be running, not stopped (ARCHITECTURE.md §10.9).
	 */
	hk_fan_set_duty(100U);

	rc = gpio_pin_configure_dt(&fan_tach, GPIO_INPUT);
	if (rc == 0) {
		rc = gpio_pin_interrupt_configure_dt(&fan_tach, GPIO_INT_EDGE_FALLING);
	}
	if (rc != 0) {
		LOG_ERR("FAN_TACH configure failed (%d)", rc);
		return rc;
	}

	gpio_init_callback(&hk.tach_cb, hk_tach_isr, BIT(fan_tach.pin));
	rc = gpio_add_callback_dt(&fan_tach, &hk.tach_cb);
	if (rc != 0) {
		return rc;
	}

	rc = thermal_cfg_defaults(&cfg);
	if (rc == 0) {
		rc = thermal_init(&thermal, &cfg);
	}
	if (rc != 0) {
		LOG_ERR("thermal_init failed (%d)", rc);
		return rc;
	}

	/*
	 * Push the ATECC608B's configuration into the façade. It had no caller, so
	 * `sec.atecc.en` and `sec.atecc.slot` were read by nothing and the part ran
	 * on the Kconfig default slot however cfg was set.
	 *
	 * Nothing to check and nothing to propagate: the call returns void, and
	 * every entry point behind it answers -ENODEV when the part is absent,
	 * unprovisioned or disabled — which means "use the software key"
	 * (port_crypto over PSA/mbedTLS), not "housekeeping failed to start"
	 * (sts_atecc.h). The absence is logged once, by that area.
	 */
	if (sts_atecc_reapply != NULL) {
		sts_atecc_reapply();
	}

	/*
	 * Group 0x0C carries the nine per-board SHUNT_CAL trims. Nothing was
	 * subscribed to it, so committing a trim — from the calibration
	 * procedure, the config page, MCP or the shell — changed the stored value
	 * and left the parts running the old one until the next reboot.
	 */
	rc = sts_cfg_register_applier(CFG_G_CAL, hk_cal_applier, NULL);
	if (rc != 0) {
		LOG_ERR("cal-group applier registration failed (%d)", rc);
		return rc;
	}

	hk.liveness_id = sts_liveness_register("housekeeping");

	tid = k_thread_create(&hk_tcb, hk_stack, HK_STACK_SIZE, hk_entry, NULL, NULL,
			      NULL, HK_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(tid, "housekeeping");

	hk.started = true;

	LOG_INF("housekeeping up: %u ms tick, 9x INA228 + 2x TMP117 + SHT45",
		HK_PERIOD_MS);

	return 0;
}
