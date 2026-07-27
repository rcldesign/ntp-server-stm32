/*
 * STS1000 "Meridian" — core/ina228: TI INA228 register codec + board rail table.
 *
 * Platform-neutral C11. This module is **pure arithmetic and data**: it never
 * touches I²C. The Zephyr glue owns the bus and hands raw register frames here
 * for decoding, or takes encoded words from here and writes them out. That split
 * is what makes every conversion on this page host-testable to the LSB.
 *
 * Board convention (interface ref §4.2, enforced by ARCHITECTURE.md invariant 7):
 * all nine monitors run **ADCRANGE = 1** (±40.96 mV shunt full scale) with
 * `Max_Expected_Current` set to each device's own full scale, which makes the
 * shunt resistance cancel out of the calibration formula:
 *
 *     CURRENT_LSB = 78.125 nV / R_SHUNT
 *     SHUNT_CAL   = 4 · 13107.2e6 · CURRENT_LSB · R_SHUNT
 *                 = 4 · 13107.2e6 · 78.125e-9      (R cancels)
 *                 = 4096                            on every one of the nine
 *
 * Per-rail resolution therefore comes from R_SHUNT alone, and every scaling
 * helper below derives its constant from `r_shunt_uohm` rather than storing a
 * rounded LSB. The rounded LSBs in `ina228_rail_info_t` are for display and for
 * cross-checking against the documentation table — never for conversion.
 *
 * Units. Raw registers decode to fixed-point integers, no floating point:
 *
 *     VSHUNT   -> nanovolts        (int32,  |v| <= 163_840_000)
 *     VBUS     -> microvolts       (int32,  0 .. 204_799_805, always positive)
 *     DIETEMP  -> milli-degrees C  (int32,  |t| <= 256_000)
 *     CURRENT  -> nanoamps         (int64)
 *     POWER    -> nanowatts        (int64)
 *     ENERGY   -> microjoules      (int64)
 *     CHARGE   -> nanocoulombs     (int64)
 *
 * All conversions round half away from zero.
 *
 * Register layout note (the easy thing to get wrong): VSHUNT, VBUS and CURRENT
 * are read as **three** bytes but carry a **20-bit** value in bits [23:4]; the
 * low nibble is reserved. POWER is a full 24-bit unsigned quantity with no
 * shift. ENERGY and CHARGE are five bytes / 40 bits.
 *
 * Reference: TI INA228 datasheet (SBOS939) register map; STS1000 constants from
 * `docs/sts1000_firmware_hardware_interface.md` §4 and §4.2.
 */

#ifndef STS1000_CORE_INA228_INA228_H_
#define STS1000_CORE_INA228_INA228_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------- register map */

#define INA228_REG_CONFIG          0x00u /* 16-bit */
#define INA228_REG_ADC_CONFIG      0x01u /* 16-bit */
#define INA228_REG_SHUNT_CAL       0x02u /* 16-bit, 15-bit field */
#define INA228_REG_SHUNT_TEMPCO    0x03u /* 16-bit, 14-bit field */
#define INA228_REG_VSHUNT          0x04u /* 24-bit frame, 20-bit signed value */
#define INA228_REG_VBUS            0x05u /* 24-bit frame, 20-bit unsigned value */
#define INA228_REG_DIETEMP         0x06u /* 16-bit signed */
#define INA228_REG_CURRENT         0x07u /* 24-bit frame, 20-bit signed value */
#define INA228_REG_POWER           0x08u /* 24-bit unsigned */
#define INA228_REG_ENERGY          0x09u /* 40-bit unsigned */
#define INA228_REG_CHARGE          0x0Au /* 40-bit signed */
#define INA228_REG_DIAG_ALRT       0x0Bu /* 16-bit */
#define INA228_REG_SOVL            0x0Cu /* 16-bit signed, shunt-voltage code */
#define INA228_REG_SUVL            0x0Du /* 16-bit signed, shunt-voltage code */
#define INA228_REG_BOVL            0x0Eu /* 16-bit unsigned, bus-voltage code */
#define INA228_REG_BUVL            0x0Fu /* 16-bit unsigned, bus-voltage code */
#define INA228_REG_TEMP_LIMIT      0x10u /* 16-bit signed, DIETEMP code */
#define INA228_REG_PWR_LIMIT       0x11u /* 16-bit unsigned, POWER[23:8] code */
#define INA228_REG_MANUFACTURER_ID 0x3Eu /* 16-bit, reads 0x5449 ("TI") */
#define INA228_REG_DEVICE_ID       0x3Fu /* 16-bit, [15:4] die, [3:0] revision */

/** MANUFACTURER_ID contents on a genuine part: the ASCII pair "TI". */
#define INA228_MANUFACTURER_ID_TI 0x5449u

/** DEVICE_ID[15:4] on the INA228. */
#define INA228_DEVICE_ID_DIE 0x228u

/**
 * Byte width of @p reg as it appears on the wire, or 0 for an address the
 * INA228 does not implement. Glue uses this to size its I²C read.
 */
size_t ina228_reg_width(uint8_t reg);

/* ------------------------------------------------------------- scale limits */

/** Shunt-voltage full-scale code (SOVL/SUVL): +40.96 mV at ADCRANGE = 1. */
#define INA228_SHUNT_CODE_MAX ((int16_t)32767)
/** Shunt-voltage minimum code. Also the "never trips" SUVL setting. */
#define INA228_SHUNT_CODE_MIN ((int16_t)(-32768))

/**
 * SUVL value that can never assert.
 *
 * The INA228 has no per-comparator enable bit — a limit register is always
 * compared — so a threshold is disabled by parking it where the measurement
 * cannot reach. The most negative shunt code is below the ADC's own full scale,
 * so SUVL at this value never fires.
 */
#define INA228_SUVL_DISABLED INA228_SHUNT_CODE_MIN

/** SOVL value that can never assert, by the same argument. */
#define INA228_SOVL_DISABLED INA228_SHUNT_CODE_MAX

/** SHUNT_CAL for this board's convention (interface ref §4.2), all nine parts. */
#define INA228_SHUNT_CAL_DEFAULT 4096u

/** SHUNT_CAL is a 15-bit field. */
#define INA228_SHUNT_CAL_MAX 0x7FFFu

/* ----------------------------------------------------------------- ADCRANGE */

/**
 * Shunt full-scale range. The STS1000 uses INA228_ADCRANGE_40_96MV on every
 * monitor; the wide range exists so the codec is honest about the register.
 */
typedef enum {
	INA228_ADCRANGE_163_84MV = 0, /* CONFIG.ADCRANGE = 0, 312.5 nV/LSB */
	INA228_ADCRANGE_40_96MV = 1,  /* CONFIG.ADCRANGE = 1,  78.125 nV/LSB */
} ina228_adcrange_t;

/* ------------------------------------------------------------ CONFIG (0x00) */

/**
 * CONFIG register fields.
 *
 * @p convdly_steps is CONVDLY: initial conversion delay in 2 ms steps (0..255,
 * so 0..510 ms). @p tempcomp enables the shunt temperature compensation that
 * uses SHUNT_TEMPCO — left off on this board, whose shunts are metal-strip
 * parts with a small enough tempco that the per-board SHUNT_CAL trim dominates.
 */
typedef struct {
	bool rst;              /* CONFIG.RST    — full device reset */
	bool rstacc;           /* CONFIG.RSTACC — clear ENERGY/CHARGE accumulators */
	uint8_t convdly_steps; /* CONFIG.CONVDLY, 2 ms per step */
	bool tempcomp;         /* CONFIG.TEMPCOMP */
	ina228_adcrange_t adcrange;
} ina228_config_t;

/** Fill @p cfg with the board default: ADCRANGE = 1, no delay, no tempco. */
void ina228_config_default(ina228_config_t *cfg);

/** Encode @p cfg into a CONFIG word. Reserved bits are written 0. Returns 0
 *  when @p cfg is NULL, which is the register's own reset value. */
uint16_t ina228_config_encode(const ina228_config_t *cfg);

/**
 * Decode a CONFIG word.
 *
 * @retval 0        Decoded into @p out.
 * @retval -EINVAL  @p out is NULL.
 */
int ina228_config_decode(uint16_t raw, ina228_config_t *out);

/* -------------------------------------------------------- ADC_CONFIG (0x01) */

/** ADC_CONFIG.MODE. */
typedef enum {
	INA228_MODE_SHUTDOWN = 0x0,
	INA228_MODE_TRIG_BUS = 0x1,
	INA228_MODE_TRIG_SHUNT = 0x2,
	INA228_MODE_TRIG_SHUNT_BUS = 0x3,
	INA228_MODE_TRIG_TEMP = 0x4,
	INA228_MODE_TRIG_TEMP_BUS = 0x5,
	INA228_MODE_TRIG_TEMP_SHUNT = 0x6,
	INA228_MODE_TRIG_ALL = 0x7,
	INA228_MODE_SHUTDOWN_ALT = 0x8,
	INA228_MODE_CONT_BUS = 0x9,
	INA228_MODE_CONT_SHUNT = 0xA,
	INA228_MODE_CONT_SHUNT_BUS = 0xB,
	INA228_MODE_CONT_TEMP = 0xC,
	INA228_MODE_CONT_TEMP_BUS = 0xD,
	INA228_MODE_CONT_TEMP_SHUNT = 0xE,
	INA228_MODE_CONT_ALL = 0xF,
} ina228_mode_t;

/** ADC_CONFIG.VBUSCT / VSHCT / VTCT — per-channel conversion time. */
typedef enum {
	INA228_CT_50US = 0,
	INA228_CT_84US = 1,
	INA228_CT_150US = 2,
	INA228_CT_280US = 3,
	INA228_CT_540US = 4,
	INA228_CT_1052US = 5,
	INA228_CT_2074US = 6,
	INA228_CT_4120US = 7,
} ina228_ct_t;

/** ADC_CONFIG.AVG — hardware averaging depth. */
typedef enum {
	INA228_AVG_1 = 0,
	INA228_AVG_4 = 1,
	INA228_AVG_16 = 2,
	INA228_AVG_64 = 3,
	INA228_AVG_128 = 4,
	INA228_AVG_256 = 5,
	INA228_AVG_512 = 6,
	INA228_AVG_1024 = 7,
} ina228_avg_t;

typedef struct {
	ina228_mode_t mode;
	ina228_ct_t vbusct;
	ina228_ct_t vshct;
	ina228_ct_t vtct;
	ina228_avg_t avg;
} ina228_adc_config_t;

/**
 * Fill @p cfg with the board default for a housekeeping rail monitor:
 * continuous bus + shunt + temperature, 1052 µs per channel, 16-sample average.
 *
 * That gives a ~50 ms averaged result — comfortably slower than the 1 kHz GPIO
 * scan that services the ALERT, and (per interface ref §4.1) far longer than the
 * 1 ms panel-LED PWM period, so the panel monitor alerts on average current
 * rather than on a PWM chop.
 */
void ina228_adc_config_default(ina228_adc_config_t *cfg);

/** Encode @p cfg into an ADC_CONFIG word. Returns 0 when @p cfg is NULL. */
uint16_t ina228_adc_config_encode(const ina228_adc_config_t *cfg);

/**
 * Decode an ADC_CONFIG word.
 *
 * @retval 0        Decoded into @p out.
 * @retval -EINVAL  @p out is NULL.
 */
int ina228_adc_config_decode(uint16_t raw, ina228_adc_config_t *out);

/** Averaging depth as a sample count: 1, 4, 16, 64, 128, 256, 512 or 1024. */
uint32_t ina228_avg_samples(ina228_avg_t avg);

/** Single-channel conversion time in microseconds. */
uint32_t ina228_ct_us(ina228_ct_t ct);

/**
 * Time in microseconds between successive averaged results for @p cfg: the sum
 * of the conversion times of the channels the mode actually converts,
 * multiplied by the averaging depth. 0 for a shutdown or triggered mode, and
 * for a NULL @p cfg.
 */
uint32_t ina228_conv_interval_us(const ina228_adc_config_t *cfg);

/* --------------------------------------------------------- SHUNT_CAL (0x02) */

/**
 * Per-board SHUNT_CAL trim: `round(base · i_ref / i_reported)`, clamped to the
 * 15-bit field (interface ref §4.2, spec §10.4, bench item BM-1).
 *
 * The 1 % shunt tolerance dominates the uncalibrated error budget, so each board
 * is trimmed once against a reference load and the nine results are stored in
 * NOR. @p i_ref and @p i_reported are in the same arbitrary current unit.
 *
 * @param base         Nominal calibration, normally INA228_SHUNT_CAL_DEFAULT.
 * @param i_ref        Current the reference instrument reports.
 * @param i_reported   Current the INA228 reports for the same load.
 * @param out          Receives the trimmed value.
 *
 * @retval 0        Trim computed.
 * @retval -EINVAL  @p out is NULL, or @p i_ref or @p i_reported is 0. A zero
 *                  @p i_reported yields no ratio; a zero @p i_ref would compute
 *                  SHUNT_CAL = 0, which the part accepts silently and then
 *                  reads 0 A forever with no error flag.
 * @retval -ERANGE  Result clamped to INA228_SHUNT_CAL_MAX; @p out is the clamp.
 */
int ina228_shunt_cal_trim(uint16_t base, uint32_t i_ref, uint32_t i_reported,
			  uint16_t *out);

/* ------------------------------------------------- threshold code encodings */

/**
 * SOVL / SUVL code for a current trip point.
 *
 * SOVL and SUVL are **shunt-voltage** registers, so the code is
 * `round(I · R_SHUNT / V_LSB)` with V_LSB = 1.25 µV at ADCRANGE = 1 (5 µV at
 * ADCRANGE = 0). They are not scaled by SHUNT_CAL.
 *
 * @param r_shunt_uohm  Shunt resistance in microhms.
 * @param i_ua          Trip current in microamps; may be negative.
 * @param range         ADCRANGE in force on the device.
 * @param out           Receives the 16-bit signed code.
 *
 * @retval 0        Code computed.
 * @retval -EINVAL  @p out is NULL or @p r_shunt_uohm is 0.
 * @retval -ERANGE  Trip point is outside the ADC's full scale; @p out holds the
 *                  clamped code, which the hardware can still never assert on.
 */
int ina228_shunt_code(uint32_t r_shunt_uohm, int32_t i_ua,
		      ina228_adcrange_t range, int16_t *out);

/** As ina228_shunt_code(), with the trip point in milliamps. */
int ina228_shunt_code_ma(uint32_t r_shunt_uohm, int32_t i_ma,
			 ina228_adcrange_t range, int16_t *out);

/** Inverse of ina228_shunt_code(): trip current in microamps for @p code. */
int64_t ina228_shunt_code_to_ua(uint32_t r_shunt_uohm, int16_t code,
				ina228_adcrange_t range);

/**
 * BOVL / BUVL code for a bus-voltage threshold. Conversion factor 3.125 mV/LSB,
 * unsigned; the 85 V bus full scale is code 27200.
 *
 * @retval 0        Code computed.
 * @retval -EINVAL  @p out is NULL or @p mv is negative.
 * @retval -ERANGE  Clamped to 0xFFFF.
 */
int ina228_bus_code(int32_t mv, uint16_t *out);

/**
 * TEMP_LIMIT code for a die-temperature threshold in milli-degrees C. Same
 * 7.8125 m°C/LSB scale as DIETEMP.
 *
 * @retval 0        Code computed.
 * @retval -EINVAL  @p out is NULL.
 * @retval -ERANGE  Clamped to the 16-bit signed range.
 */
int ina228_temp_code(int32_t mc, int16_t *out);

/**
 * PWR_LIMIT code for a power threshold in nanowatts. PWR_LIMIT is compared
 * against the upper 16 bits of POWER, so one code is 256 · POWER_LSB.
 *
 * @retval 0        Code computed.
 * @retval -EINVAL  @p out is NULL, @p r_shunt_uohm is 0, or @p nw is negative.
 * @retval -ERANGE  Clamped to 0xFFFF.
 */
int ina228_power_code(uint32_t r_shunt_uohm, int64_t nw, ina228_adcrange_t range,
		      uint16_t *out);

/* ----------------------------------------------------- raw frame extraction */

/** 16-bit register frame (MSB first) as an unsigned word. */
uint16_t ina228_raw16(const uint8_t frame[2]);

/** 24-bit frame -> the sign-extended 20-bit value in bits [23:4]. */
int32_t ina228_raw20_signed(const uint8_t frame[3]);

/** 24-bit frame -> the unsigned 20-bit value in bits [23:4]. */
uint32_t ina228_raw20_unsigned(const uint8_t frame[3]);

/** 24-bit frame -> the full unsigned 24-bit value (POWER). */
uint32_t ina228_raw24(const uint8_t frame[3]);

/** 40-bit frame -> unsigned 40-bit value (ENERGY). */
uint64_t ina228_raw40(const uint8_t frame[5]);

/** 40-bit frame -> sign-extended 40-bit value (CHARGE). */
int64_t ina228_raw40_signed(const uint8_t frame[5]);

/* --------------------------------------------------------- value conversion */

/** Nominal CURRENT_LSB in picoamps for @p r_shunt_uohm under the board's
 *  SHUNT_CAL convention. 0 when @p r_shunt_uohm is 0. */
uint32_t ina228_current_lsb_pa(uint32_t r_shunt_uohm, ina228_adcrange_t range);

/** Nominal POWER_LSB in nanowatts (3.2 · CURRENT_LSB). 0 when @p r_shunt_uohm
 *  is 0. */
uint32_t ina228_power_lsb_nw(uint32_t r_shunt_uohm, ina228_adcrange_t range);

/** Full-scale current in microamps for @p r_shunt_uohm. 0 when it is 0. */
uint32_t ina228_fs_current_ua(uint32_t r_shunt_uohm, ina228_adcrange_t range);

/** VSHUNT frame -> nanovolts. */
int32_t ina228_vshunt_nv(const uint8_t frame[3], ina228_adcrange_t range);

/** VBUS frame -> microvolts. Always non-negative (datasheet: two's complement
 *  but positive only), so a decode result below zero is impossible. */
int32_t ina228_vbus_uv(const uint8_t frame[3]);

/** VBUS frame -> millivolts, rounded. Convenience for rail-window checks. */
int32_t ina228_vbus_mv(const uint8_t frame[3]);

/** DIETEMP frame -> milli-degrees Celsius. */
int32_t ina228_dietemp_mc(const uint8_t frame[2]);

/** CURRENT frame -> nanoamps for a device with @p r_shunt_uohm. 0 when
 *  @p r_shunt_uohm is 0. */
int64_t ina228_current_na(const uint8_t frame[3], uint32_t r_shunt_uohm,
			  ina228_adcrange_t range);

/** CURRENT frame -> microamps, rounded. */
int32_t ina228_current_ua(const uint8_t frame[3], uint32_t r_shunt_uohm,
			  ina228_adcrange_t range);

/** POWER frame -> nanowatts. 0 when @p r_shunt_uohm is 0. */
int64_t ina228_power_nw(const uint8_t frame[3], uint32_t r_shunt_uohm,
			ina228_adcrange_t range);

/** ENERGY frame -> microjoules (ENERGY_LSB = 16 · POWER_LSB). */
int64_t ina228_energy_uj(const uint8_t frame[5], uint32_t r_shunt_uohm,
			 ina228_adcrange_t range);

/** CHARGE frame -> nanocoulombs (CHARGE_LSB = CURRENT_LSB). */
int64_t ina228_charge_nc(const uint8_t frame[5], uint32_t r_shunt_uohm,
			 ina228_adcrange_t range);

/* --------------------------------------------------------- DIAG_ALRT (0x0B) */

#define INA228_DIAG_ALATCH    (1u << 15) /* latch ALERT until DIAG_ALRT is read */
#define INA228_DIAG_CNVR      (1u << 14) /* assert ALERT on conversion ready */
#define INA228_DIAG_SLOWALERT (1u << 13) /* compare against averaged value */
#define INA228_DIAG_APOL      (1u << 12) /* ALERT polarity (0 = active low) */
#define INA228_DIAG_ENERGYOF  (1u << 11)
#define INA228_DIAG_CHARGEOF  (1u << 10)
#define INA228_DIAG_MATHOF    (1u << 9)
/* bit 8 reserved */
#define INA228_DIAG_TMPOL     (1u << 7)
#define INA228_DIAG_SHNTOL    (1u << 6)
#define INA228_DIAG_SHNTUL    (1u << 5)
#define INA228_DIAG_BUSOL     (1u << 4)
#define INA228_DIAG_BUSUL     (1u << 3)
#define INA228_DIAG_POL       (1u << 2)
#define INA228_DIAG_CNVRF     (1u << 1)
#define INA228_DIAG_MEMSTAT   (1u << 0)

/** Every bit the codec defines. Bit 8 is reserved and always excluded. */
#define INA228_DIAG_KNOWN_MASK 0xFEFFu

/** Bits that are configuration, written by firmware and read back unchanged. */
#define INA228_DIAG_CONFIG_MASK                                                \
	(INA228_DIAG_ALATCH | INA228_DIAG_CNVR | INA228_DIAG_SLOWALERT |       \
	 INA228_DIAG_APOL)

/** Bits that indicate a real fault. CNVRF is a liveness flag, not a fault, and
 *  MEMSTAT reads 1 when memory is *healthy* — see ina228_diag_fault_mask(). */
#define INA228_DIAG_FAULT_MASK                                                 \
	(INA228_DIAG_ENERGYOF | INA228_DIAG_CHARGEOF | INA228_DIAG_MATHOF |    \
	 INA228_DIAG_TMPOL | INA228_DIAG_SHNTOL | INA228_DIAG_SHNTUL |         \
	 INA228_DIAG_BUSOL | INA228_DIAG_BUSUL | INA228_DIAG_POL)

typedef struct {
	/* configuration read-back */
	bool alatch;
	bool cnvr;
	bool slowalert;
	bool apol;
	/* accumulator / arithmetic status */
	bool energyof;
	bool chargeof;
	bool mathof;
	/* limit comparators */
	bool tmpol;
	bool shntol;
	bool shntul;
	bool busol;
	bool busul;
	bool pol;
	/* liveness / self-test */
	bool cnvrf;
	bool memstat; /* 1 = memory checksum OK, 0 = corrupt */
} ina228_diag_t;

/**
 * Decode a DIAG_ALRT word.
 *
 * @retval 0        Decoded into @p out.
 * @retval -EINVAL  @p out is NULL.
 */
int ina228_diag_decode(uint16_t raw, ina228_diag_t *out);

/** Encode a DIAG_ALRT word. Reserved bit 8 is written 0. 0 for a NULL @p d. */
uint16_t ina228_diag_encode(const ina228_diag_t *d);

/**
 * Fault bits asserted in @p raw, including a synthesised MEMSTAT.
 *
 * MEMSTAT is inverted relative to every other flag — the datasheet defines it as
 * 1 when the device memory checksum passes — so a naive `raw & FAULT_MASK` would
 * report a healthy part as faulted and a corrupt one as fine. This helper folds
 * the inversion in so callers never have to remember it.
 */
uint16_t ina228_diag_fault_mask(uint16_t raw);

/** True when @p raw carries any fault (see ina228_diag_fault_mask()). */
bool ina228_diag_is_fault(uint16_t raw);

/**
 * Single cause to report for an alert, highest severity first.
 *
 * Priority is a firmware policy choice, not a datasheet fact; it is ordered so
 * that the annunciated cause is the one an operator must act on:
 *
 *   MEM_ERROR   the device's own memory is corrupt, so every other bit and
 *               every reading from this part is untrustworthy
 *   SHUNT_OVER  over-current — the hard hazard the shunts exist to catch
 *   BUS_OVER    over-voltage — the other hard hazard
 *   TEMP_OVER   the part is cooking
 *   POWER_OVER  derived limit; implies one of the two above is marginal
 *   BUS_UNDER   rail has collapsed (service outage, not a hazard)
 *   SHUNT_UNDER load has disappeared (this is the V_ANT open-antenna detector)
 *   MATH_OVF / CHARGE_OVF / ENERGY_OVF   accumulator housekeeping
 *   CONV_READY  not a fault at all
 */
typedef enum {
	INA228_CAUSE_NONE = 0,
	INA228_CAUSE_MEM_ERROR,
	INA228_CAUSE_SHUNT_OVER,
	INA228_CAUSE_BUS_OVER,
	INA228_CAUSE_TEMP_OVER,
	INA228_CAUSE_POWER_OVER,
	INA228_CAUSE_BUS_UNDER,
	INA228_CAUSE_SHUNT_UNDER,
	INA228_CAUSE_MATH_OVF,
	INA228_CAUSE_CHARGE_OVF,
	INA228_CAUSE_ENERGY_OVF,
	INA228_CAUSE_CONV_READY,
	INA228_CAUSE_COUNT,
} ina228_cause_t;

/** Highest-severity cause asserted in @p raw, or INA228_CAUSE_NONE. */
ina228_cause_t ina228_diag_cause(uint16_t raw);

/** Short, stable name for @p cause. Never NULL. */
const char *ina228_cause_str(ina228_cause_t cause);

/* ------------------------------------------------------------- ID registers */

/** DEVICE_ID[15:4] — the die identifier. */
uint16_t ina228_device_id_die(uint16_t device_id);

/** DEVICE_ID[3:0] — the silicon revision. */
uint8_t ina228_device_id_rev(uint16_t device_id);

/** True when the ID pair is that of an INA228. Revision is not checked. */
bool ina228_id_matches(uint16_t manufacturer_id, uint16_t device_id);

/* --------------------------------------------------------------- rail table */

/**
 * The nine monitors, in ascending I²C address order.
 *
 * GPS is at **0x4A, not 0x44** — SHT45 U72 owns 0x44 on the same bus, so the
 * historically documented address would collide (interface ref §4).
 */
typedef enum {
	INA228_RAIL_POE = 0,   /* U10 0x40 — PoE input V_POE */
	INA228_RAIL_3V3_STM,   /* U31 0x41 — always-on housekeeping rail */
	INA228_RAIL_5V_DISP,   /* U32 0x42 — display rail */
	INA228_RAIL_3V3_MAIN,  /* U30 0x43 — main/general 3V3 */
	INA228_RAIL_V_ANT,     /* U26 0x45 — GNSS antenna bias */
	INA228_RAIL_OCXO,      /* U37 0x46 — OCXO rail */
	INA228_RAIL_VCC_RB,    /* U44 0x47 — rubidium rail, programmable */
	INA228_RAIL_3V3_GPS,   /* U23 0x4A — GNSS receiver rail */
	INA228_RAIL_PANEL_5V,  /* U54 0x4C — panel-LED 5 V */
	INA228_RAIL_COUNT,
} ina228_rail_t;

/**
 * Static facts about one monitor. Every numeric field is from interface ref
 * §4.2 except the two noted below.
 *
 * @p current_lsb_pa and @p power_lsb_nw are **rounded** (520.8333… nA/LSB does
 * not land on an integer picoamp) and exist for display and for cross-checking
 * against the documentation table. Conversions use @p r_shunt_uohm.
 *
 * @p nominal_mv is the rail's design voltage, used by pwrseq for its rail
 * windows. VCC_RB carries the doc's 15 V nominal, but that rail is programmable
 * over 4.51–24.45 V and pwrseq computes its expected value from the digipot
 * code instead.
 *
 * @p suvl_code_default is INA228_SUVL_DISABLED on every rail but V_ANT, whose
 * foldback limiter makes SOVL a backstop rather than a detector: an antenna
 * fault shows up as *under*-current (interface ref §4.2, §7).
 *
 * @p alert_port is 'F' or 'G' and @p alert_bit the bit within that port's IDR —
 * the panel monitor is the one alert on GPIOF.
 */
typedef struct {
	uint8_t addr;
	const char *name;       /* canonical rail/net name */
	const char *designator; /* schematic reference, e.g. "U10" */
	const char *shunt_ref;  /* shunt designator, e.g. "R30" */
	uint32_t r_shunt_uohm;
	uint32_t current_lsb_pa;
	uint32_t power_lsb_nw;
	uint32_t fs_current_ua;
	uint32_t design_max_ma;
	int32_t nominal_mv;
	int16_t sovl_code_default;
	int16_t suvl_code_default;
	uint8_t alert_port;
	uint8_t alert_bit;
} ina228_rail_info_t;

/** The nine monitors. Indexed by ina228_rail_t. */
extern const ina228_rail_info_t ina228_rail_tbl[INA228_RAIL_COUNT];

/** Entry for @p rail, or NULL when @p rail is out of range. */
const ina228_rail_info_t *ina228_rail(ina228_rail_t rail);

/**
 * Reverse lookup by I²C address.
 *
 * @retval 0        Found; @p out holds the rail id.
 * @retval -EINVAL  @p out is NULL.
 * @retval -ENOENT  No monitor at @p addr.
 */
int ina228_rail_by_addr(uint8_t addr, ina228_rail_t *out);

/**
 * Recomputed SOVL default for @p rail: 125 % of its design maximum, per
 * interface ref §4.2. Used to check the stored table against the shunt values
 * rather than trusting a transcription.
 *
 * @retval 0        Computed into @p out.
 * @retval -EINVAL  @p rail out of range or @p out is NULL.
 */
int ina228_rail_sovl_default(ina228_rail_t rail, int16_t *out);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_INA228_INA228_H_ */
