/*
 * STS1000 "Meridian" — core/ina228: TI INA228 register codec + board rail table.
 *
 * See ina228.h for the contract and the unit conventions.
 *
 * Everything scales through mul_div_round(), which splits the multiply around
 * the divide so a 40-bit ENERGY accumulator times a nanoscale LSB cannot
 * overflow int64 on the way to a result that comfortably fits. Every LSB on this
 * part is an exact rational — 78.125 nV is 625/8, 195.3125 µV is 3125/16,
 * 7.8125 m°C is 125/16 — so nothing here needs floating point, and the results
 * are bit-exact rather than "close enough".
 */

#include "ina228/ina228.h"

#include <errno.h>

/* --------------------------------------------------------------- arithmetic */

/*
 * round(v * mul / div), half away from zero, without overflowing int64.
 *
 * The naive form overflows on ENERGY: 2^40 counts times a multiplier of 4e6 is
 * 4.4e21. Splitting v into quotient and remainder around div keeps both partial
 * products small — q*mul is bounded by the true result and r*mul by div*mul,
 * which for every (mul, div) pair used here is under 4e16.
 *
 * div must be positive and non-zero; every call site guarantees that.
 */
static int64_t mul_div_round(int64_t v, int64_t mul, int64_t div)
{
	int64_t q = v / div;
	int64_t r = v % div; /* same sign as v */
	int64_t frac = r * mul;
	int64_t half = div / 2;

	if (frac >= 0) {
		frac = (frac + half) / div;
	} else {
		frac = (frac - half) / div;
	}

	return (q * mul) + frac;
}

/* Numerators of the exact per-ADCRANGE LSBs, all over a shunt in microhms. */
#define CURRENT_LSB_NUM_HI INT64_C(78125000)   /* nA*uohm, ADCRANGE=1 */
#define CURRENT_LSB_NUM_LO INT64_C(312500000)  /* nA*uohm, ADCRANGE=0 */
#define POWER_LSB_NUM_HI   INT64_C(250000000)  /* nW*uohm, ADCRANGE=1 */
#define POWER_LSB_NUM_LO   INT64_C(1000000000) /* nW*uohm, ADCRANGE=0 */
#define ENERGY_LSB_NUM_HI  INT64_C(4000000)    /* uJ*uohm, = 16*POWER/1000 */
#define ENERGY_LSB_NUM_LO  INT64_C(16000000)   /* uJ*uohm */
#define FS_CURRENT_NUM_HI  INT64_C(40960000000)  /* uA*uohm */
#define FS_CURRENT_NUM_LO  INT64_C(163840000000) /* uA*uohm */
#define PWR_LIMIT_DIV_HI   INT64_C(64000000000)  /* 256 * POWER_LSB_NUM_HI */
#define PWR_LIMIT_DIV_LO   INT64_C(256000000000) /* 256 * POWER_LSB_NUM_LO */

/* SOVL/SUVL LSB in picovolts: 1.25 uV at ADCRANGE=1, 5 uV at ADCRANGE=0. */
#define SHUNT_LIMIT_LSB_PV_HI INT64_C(1250000)
#define SHUNT_LIMIT_LSB_PV_LO INT64_C(5000000)

static bool range_is_hi(ina228_adcrange_t range)
{
	return range == INA228_ADCRANGE_40_96MV;
}

/* --------------------------------------------------------------- reg widths */

size_t ina228_reg_width(uint8_t reg)
{
	switch (reg) {
	case INA228_REG_VSHUNT:
	case INA228_REG_VBUS:
	case INA228_REG_CURRENT:
	case INA228_REG_POWER:
		return 3U;
	case INA228_REG_ENERGY:
	case INA228_REG_CHARGE:
		return 5U;
	case INA228_REG_CONFIG:
	case INA228_REG_ADC_CONFIG:
	case INA228_REG_SHUNT_CAL:
	case INA228_REG_SHUNT_TEMPCO:
	case INA228_REG_DIETEMP:
	case INA228_REG_DIAG_ALRT:
	case INA228_REG_SOVL:
	case INA228_REG_SUVL:
	case INA228_REG_BOVL:
	case INA228_REG_BUVL:
	case INA228_REG_TEMP_LIMIT:
	case INA228_REG_PWR_LIMIT:
	case INA228_REG_MANUFACTURER_ID:
	case INA228_REG_DEVICE_ID:
		return 2U;
	default:
		return 0U;
	}
}

/* ------------------------------------------------------------------- CONFIG */

void ina228_config_default(ina228_config_t *cfg)
{
	if (cfg == NULL) {
		return;
	}
	cfg->rst = false;
	cfg->rstacc = false;
	cfg->convdly_steps = 0U;
	cfg->tempcomp = false;
	cfg->adcrange = INA228_ADCRANGE_40_96MV;
}

uint16_t ina228_config_encode(const ina228_config_t *cfg)
{
	uint16_t raw = 0U;

	if (cfg == NULL) {
		return 0U;
	}

	if (cfg->rst) {
		raw |= (uint16_t)(1U << 15);
	}
	if (cfg->rstacc) {
		raw |= (uint16_t)(1U << 14);
	}
	raw |= (uint16_t)((uint16_t)cfg->convdly_steps << 6);
	if (cfg->tempcomp) {
		raw |= (uint16_t)(1U << 5);
	}
	if (range_is_hi(cfg->adcrange)) {
		raw |= (uint16_t)(1U << 4);
	}

	return raw;
}

int ina228_config_decode(uint16_t raw, ina228_config_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}

	out->rst = (raw & (1U << 15)) != 0U;
	out->rstacc = (raw & (1U << 14)) != 0U;
	out->convdly_steps = (uint8_t)((raw >> 6) & 0xFFU);
	out->tempcomp = (raw & (1U << 5)) != 0U;
	out->adcrange = ((raw & (1U << 4)) != 0U) ? INA228_ADCRANGE_40_96MV
						  : INA228_ADCRANGE_163_84MV;
	return 0;
}

/* --------------------------------------------------------------- ADC_CONFIG */

void ina228_adc_config_default(ina228_adc_config_t *cfg)
{
	if (cfg == NULL) {
		return;
	}
	cfg->mode = INA228_MODE_CONT_ALL;
	cfg->vbusct = INA228_CT_1052US;
	cfg->vshct = INA228_CT_1052US;
	cfg->vtct = INA228_CT_1052US;
	cfg->avg = INA228_AVG_16;
}

uint16_t ina228_adc_config_encode(const ina228_adc_config_t *cfg)
{
	uint16_t raw;

	if (cfg == NULL) {
		return 0U;
	}

	raw = (uint16_t)(((uint16_t)cfg->mode & 0x0FU) << 12);
	raw |= (uint16_t)(((uint16_t)cfg->vbusct & 0x07U) << 9);
	raw |= (uint16_t)(((uint16_t)cfg->vshct & 0x07U) << 6);
	raw |= (uint16_t)(((uint16_t)cfg->vtct & 0x07U) << 3);
	raw |= (uint16_t)((uint16_t)cfg->avg & 0x07U);

	return raw;
}

int ina228_adc_config_decode(uint16_t raw, ina228_adc_config_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}

	out->mode = (ina228_mode_t)((raw >> 12) & 0x0FU);
	out->vbusct = (ina228_ct_t)((raw >> 9) & 0x07U);
	out->vshct = (ina228_ct_t)((raw >> 6) & 0x07U);
	out->vtct = (ina228_ct_t)((raw >> 3) & 0x07U);
	out->avg = (ina228_avg_t)(raw & 0x07U);
	return 0;
}

uint32_t ina228_avg_samples(ina228_avg_t avg)
{
	static const uint16_t tbl[8] = {1U, 4U, 16U, 64U, 128U, 256U, 512U, 1024U};

	return tbl[(unsigned int)avg & 0x07U];
}

uint32_t ina228_ct_us(ina228_ct_t ct)
{
	static const uint16_t tbl[8] = {50U,  84U,   150U,  280U,
					540U, 1052U, 2074U, 4120U};

	return tbl[(unsigned int)ct & 0x07U];
}

uint32_t ina228_conv_interval_us(const ina228_adc_config_t *cfg)
{
	unsigned int mode;
	uint32_t sum = 0U;

	if (cfg == NULL) {
		return 0U;
	}

	mode = (unsigned int)cfg->mode & 0x0FU;

	/*
	 * MODE bit 3 selects continuous; bits 2/1/0 select temperature, shunt
	 * and bus respectively. A triggered mode has no repeating interval, so
	 * it reports 0 rather than a number the caller would mistake for a
	 * refresh rate.
	 */
	if ((mode & 0x8U) == 0U) {
		return 0U;
	}
	if ((mode & 0x1U) != 0U) {
		sum += ina228_ct_us(cfg->vbusct);
	}
	if ((mode & 0x2U) != 0U) {
		sum += ina228_ct_us(cfg->vshct);
	}
	if ((mode & 0x4U) != 0U) {
		sum += ina228_ct_us(cfg->vtct);
	}

	return sum * ina228_avg_samples(cfg->avg);
}

/* ---------------------------------------------------------------- SHUNT_CAL */

int ina228_shunt_cal_trim(uint16_t base, uint32_t i_ref, uint32_t i_reported,
			  uint16_t *out)
{
	int64_t v;

	if ((out == NULL) || (i_reported == 0U)) {
		return -EINVAL;
	}

	v = mul_div_round((int64_t)base, (int64_t)i_ref, (int64_t)i_reported);
	if (v > (int64_t)INA228_SHUNT_CAL_MAX) {
		*out = (uint16_t)INA228_SHUNT_CAL_MAX;
		return -ERANGE;
	}

	*out = (uint16_t)v;
	return 0;
}

/* -------------------------------------------------------- threshold codings */

static int clamp_i16(int64_t v, int16_t *out)
{
	if (v > (int64_t)INA228_SHUNT_CODE_MAX) {
		*out = INA228_SHUNT_CODE_MAX;
		return -ERANGE;
	}
	if (v < (int64_t)INA228_SHUNT_CODE_MIN) {
		*out = INA228_SHUNT_CODE_MIN;
		return -ERANGE;
	}
	*out = (int16_t)v;
	return 0;
}

/* Callers guarantee v >= 0 by rejecting negative inputs up front. */
static int clamp_u16(int64_t v, uint16_t *out)
{
	if (v > (int64_t)0xFFFF) {
		*out = 0xFFFFU;
		return -ERANGE;
	}
	*out = (uint16_t)v;
	return 0;
}

int ina228_shunt_code(uint32_t r_shunt_uohm, int32_t i_ua,
		      ina228_adcrange_t range, int16_t *out)
{
	int64_t lsb_pv;
	int64_t code;

	if ((out == NULL) || (r_shunt_uohm == 0U)) {
		return -EINVAL;
	}

	lsb_pv = range_is_hi(range) ? SHUNT_LIMIT_LSB_PV_HI
				    : SHUNT_LIMIT_LSB_PV_LO;

	/* V_shunt in picovolts is exactly microamps times microhms. */
	code = mul_div_round((int64_t)i_ua, (int64_t)r_shunt_uohm, lsb_pv);
	return clamp_i16(code, out);
}

int ina228_shunt_code_ma(uint32_t r_shunt_uohm, int32_t i_ma,
			 ina228_adcrange_t range, int16_t *out)
{
	if ((out == NULL) || (r_shunt_uohm == 0U)) {
		return -EINVAL;
	}
	if ((i_ma > 2000000) || (i_ma < -2000000)) {
		return -EINVAL; /* would overflow the microamp conversion */
	}

	return ina228_shunt_code(r_shunt_uohm, i_ma * 1000, range, out);
}

int64_t ina228_shunt_code_to_ua(uint32_t r_shunt_uohm, int16_t code,
				ina228_adcrange_t range)
{
	int64_t lsb_pv;

	if (r_shunt_uohm == 0U) {
		return 0;
	}

	lsb_pv = range_is_hi(range) ? SHUNT_LIMIT_LSB_PV_HI
				    : SHUNT_LIMIT_LSB_PV_LO;

	return mul_div_round((int64_t)code, lsb_pv, (int64_t)r_shunt_uohm);
}

int ina228_bus_code(int32_t mv, uint16_t *out)
{
	if ((out == NULL) || (mv < 0)) {
		return -EINVAL;
	}

	/* 3.125 mV/LSB == 25/8 mV, so the code is mv * 8 / 25. */
	return clamp_u16(mul_div_round((int64_t)mv, 8, 25), out);
}

int ina228_temp_code(int32_t mc, int16_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}

	/* 7.8125 m degC/LSB == 125/16, so the code is mc * 16 / 125. */
	return clamp_i16(mul_div_round((int64_t)mc, 16, 125), out);
}

int ina228_power_code(uint32_t r_shunt_uohm, int64_t nw, ina228_adcrange_t range,
		      uint16_t *out)
{
	int64_t div;

	if ((out == NULL) || (r_shunt_uohm == 0U) || (nw < 0)) {
		return -EINVAL;
	}

	div = range_is_hi(range) ? PWR_LIMIT_DIV_HI : PWR_LIMIT_DIV_LO;

	return clamp_u16(mul_div_round(nw, (int64_t)r_shunt_uohm, div), out);
}

/* -------------------------------------------------------- raw frame decodes */

uint16_t ina228_raw16(const uint8_t frame[2])
{
	if (frame == NULL) {
		return 0U;
	}
	return (uint16_t)(((uint16_t)frame[0] << 8) | (uint16_t)frame[1]);
}

uint32_t ina228_raw24(const uint8_t frame[3])
{
	if (frame == NULL) {
		return 0U;
	}
	return ((uint32_t)frame[0] << 16) | ((uint32_t)frame[1] << 8) |
	       (uint32_t)frame[2];
}

uint32_t ina228_raw20_unsigned(const uint8_t frame[3])
{
	return ina228_raw24(frame) >> 4;
}

int32_t ina228_raw20_signed(const uint8_t frame[3])
{
	uint32_t v = ina228_raw20_unsigned(frame);

	/* Sign-extend from bit 19 without relying on implementation-defined
	 * right-shift behaviour for negative values. */
	if ((v & 0x00080000U) != 0U) {
		return (int32_t)(v | 0xFFF00000U);
	}
	return (int32_t)v;
}

uint64_t ina228_raw40(const uint8_t frame[5])
{
	if (frame == NULL) {
		return 0U;
	}
	return ((uint64_t)frame[0] << 32) | ((uint64_t)frame[1] << 24) |
	       ((uint64_t)frame[2] << 16) | ((uint64_t)frame[3] << 8) |
	       (uint64_t)frame[4];
}

int64_t ina228_raw40_signed(const uint8_t frame[5])
{
	uint64_t v = ina228_raw40(frame);

	if ((v & UINT64_C(0x8000000000)) != 0U) {
		return (int64_t)(v | UINT64_C(0xFFFFFF0000000000));
	}
	return (int64_t)v;
}

/* --------------------------------------------------------- value conversion */

uint32_t ina228_current_lsb_pa(uint32_t r_shunt_uohm, ina228_adcrange_t range)
{
	int64_t num;

	if (r_shunt_uohm == 0U) {
		return 0U;
	}
	/* nA*uohm numerator scaled to picoamps: nA -> pA is a factor of 1000. */
	num = range_is_hi(range) ? CURRENT_LSB_NUM_HI : CURRENT_LSB_NUM_LO;
	return (uint32_t)mul_div_round(num * 1000, 1, (int64_t)r_shunt_uohm);
}

uint32_t ina228_power_lsb_nw(uint32_t r_shunt_uohm, ina228_adcrange_t range)
{
	int64_t num;

	if (r_shunt_uohm == 0U) {
		return 0U;
	}
	num = range_is_hi(range) ? POWER_LSB_NUM_HI : POWER_LSB_NUM_LO;
	return (uint32_t)mul_div_round(num, 1, (int64_t)r_shunt_uohm);
}

uint32_t ina228_fs_current_ua(uint32_t r_shunt_uohm, ina228_adcrange_t range)
{
	int64_t num;

	if (r_shunt_uohm == 0U) {
		return 0U;
	}
	num = range_is_hi(range) ? FS_CURRENT_NUM_HI : FS_CURRENT_NUM_LO;
	return (uint32_t)mul_div_round(num, 1, (int64_t)r_shunt_uohm);
}

int32_t ina228_vshunt_nv(const uint8_t frame[3], ina228_adcrange_t range)
{
	int32_t raw = ina228_raw20_signed(frame);

	/* 78.125 nV == 625/8; 312.5 nV == 2500/8. */
	return (int32_t)mul_div_round((int64_t)raw, range_is_hi(range) ? 625 : 2500,
				      8);
}

int32_t ina228_vbus_uv(const uint8_t frame[3])
{
	uint32_t raw = ina228_raw20_unsigned(frame);

	/* 195.3125 uV == 3125/16. */
	return (int32_t)mul_div_round((int64_t)raw, 3125, 16);
}

int32_t ina228_vbus_mv(const uint8_t frame[3])
{
	return (int32_t)mul_div_round((int64_t)ina228_vbus_uv(frame), 1, 1000);
}

int32_t ina228_dietemp_mc(const uint8_t frame[2])
{
	int16_t raw = (int16_t)ina228_raw16(frame);

	/* 7.8125 m degC == 125/16. */
	return (int32_t)mul_div_round((int64_t)raw, 125, 16);
}

int64_t ina228_current_na(const uint8_t frame[3], uint32_t r_shunt_uohm,
			  ina228_adcrange_t range)
{
	int64_t num;

	if (r_shunt_uohm == 0U) {
		return 0;
	}
	num = range_is_hi(range) ? CURRENT_LSB_NUM_HI : CURRENT_LSB_NUM_LO;
	return mul_div_round((int64_t)ina228_raw20_signed(frame), num,
			     (int64_t)r_shunt_uohm);
}

int32_t ina228_current_ua(const uint8_t frame[3], uint32_t r_shunt_uohm,
			  ina228_adcrange_t range)
{
	return (int32_t)mul_div_round(
		ina228_current_na(frame, r_shunt_uohm, range), 1, 1000);
}

int64_t ina228_power_nw(const uint8_t frame[3], uint32_t r_shunt_uohm,
			ina228_adcrange_t range)
{
	int64_t num;

	if (r_shunt_uohm == 0U) {
		return 0;
	}
	num = range_is_hi(range) ? POWER_LSB_NUM_HI : POWER_LSB_NUM_LO;
	return mul_div_round((int64_t)ina228_raw24(frame), num,
			     (int64_t)r_shunt_uohm);
}

int64_t ina228_energy_uj(const uint8_t frame[5], uint32_t r_shunt_uohm,
			 ina228_adcrange_t range)
{
	int64_t num;

	if (r_shunt_uohm == 0U) {
		return 0;
	}
	num = range_is_hi(range) ? ENERGY_LSB_NUM_HI : ENERGY_LSB_NUM_LO;
	return mul_div_round((int64_t)ina228_raw40(frame), num,
			     (int64_t)r_shunt_uohm);
}

int64_t ina228_charge_nc(const uint8_t frame[5], uint32_t r_shunt_uohm,
			 ina228_adcrange_t range)
{
	int64_t num;

	if (r_shunt_uohm == 0U) {
		return 0;
	}
	num = range_is_hi(range) ? CURRENT_LSB_NUM_HI : CURRENT_LSB_NUM_LO;
	return mul_div_round(ina228_raw40_signed(frame), num,
			     (int64_t)r_shunt_uohm);
}

/* ---------------------------------------------------------------- DIAG_ALRT */

int ina228_diag_decode(uint16_t raw, ina228_diag_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}

	out->alatch = (raw & INA228_DIAG_ALATCH) != 0U;
	out->cnvr = (raw & INA228_DIAG_CNVR) != 0U;
	out->slowalert = (raw & INA228_DIAG_SLOWALERT) != 0U;
	out->apol = (raw & INA228_DIAG_APOL) != 0U;
	out->energyof = (raw & INA228_DIAG_ENERGYOF) != 0U;
	out->chargeof = (raw & INA228_DIAG_CHARGEOF) != 0U;
	out->mathof = (raw & INA228_DIAG_MATHOF) != 0U;
	out->tmpol = (raw & INA228_DIAG_TMPOL) != 0U;
	out->shntol = (raw & INA228_DIAG_SHNTOL) != 0U;
	out->shntul = (raw & INA228_DIAG_SHNTUL) != 0U;
	out->busol = (raw & INA228_DIAG_BUSOL) != 0U;
	out->busul = (raw & INA228_DIAG_BUSUL) != 0U;
	out->pol = (raw & INA228_DIAG_POL) != 0U;
	out->cnvrf = (raw & INA228_DIAG_CNVRF) != 0U;
	out->memstat = (raw & INA228_DIAG_MEMSTAT) != 0U;
	return 0;
}

uint16_t ina228_diag_encode(const ina228_diag_t *d)
{
	uint16_t raw = 0U;

	if (d == NULL) {
		return 0U;
	}

	if (d->alatch) {
		raw |= INA228_DIAG_ALATCH;
	}
	if (d->cnvr) {
		raw |= INA228_DIAG_CNVR;
	}
	if (d->slowalert) {
		raw |= INA228_DIAG_SLOWALERT;
	}
	if (d->apol) {
		raw |= INA228_DIAG_APOL;
	}
	if (d->energyof) {
		raw |= INA228_DIAG_ENERGYOF;
	}
	if (d->chargeof) {
		raw |= INA228_DIAG_CHARGEOF;
	}
	if (d->mathof) {
		raw |= INA228_DIAG_MATHOF;
	}
	if (d->tmpol) {
		raw |= INA228_DIAG_TMPOL;
	}
	if (d->shntol) {
		raw |= INA228_DIAG_SHNTOL;
	}
	if (d->shntul) {
		raw |= INA228_DIAG_SHNTUL;
	}
	if (d->busol) {
		raw |= INA228_DIAG_BUSOL;
	}
	if (d->busul) {
		raw |= INA228_DIAG_BUSUL;
	}
	if (d->pol) {
		raw |= INA228_DIAG_POL;
	}
	if (d->cnvrf) {
		raw |= INA228_DIAG_CNVRF;
	}
	if (d->memstat) {
		raw |= INA228_DIAG_MEMSTAT;
	}

	return raw;
}

uint16_t ina228_diag_fault_mask(uint16_t raw)
{
	uint16_t m = (uint16_t)(raw & INA228_DIAG_FAULT_MASK);

	/*
	 * MEMSTAT is the one inverted flag on the part: it reads 1 when the
	 * trim-memory checksum passes. Fold the inversion in here so callers
	 * cannot get it backwards and treat a healthy device as faulted.
	 */
	if ((raw & INA228_DIAG_MEMSTAT) == 0U) {
		m |= (uint16_t)INA228_DIAG_MEMSTAT;
	}

	return m;
}

bool ina228_diag_is_fault(uint16_t raw)
{
	return ina228_diag_fault_mask(raw) != 0U;
}

ina228_cause_t ina228_diag_cause(uint16_t raw)
{
	uint16_t f = ina228_diag_fault_mask(raw);

	if ((f & INA228_DIAG_MEMSTAT) != 0U) {
		return INA228_CAUSE_MEM_ERROR;
	}
	if ((f & INA228_DIAG_SHNTOL) != 0U) {
		return INA228_CAUSE_SHUNT_OVER;
	}
	if ((f & INA228_DIAG_BUSOL) != 0U) {
		return INA228_CAUSE_BUS_OVER;
	}
	if ((f & INA228_DIAG_TMPOL) != 0U) {
		return INA228_CAUSE_TEMP_OVER;
	}
	if ((f & INA228_DIAG_POL) != 0U) {
		return INA228_CAUSE_POWER_OVER;
	}
	if ((f & INA228_DIAG_BUSUL) != 0U) {
		return INA228_CAUSE_BUS_UNDER;
	}
	if ((f & INA228_DIAG_SHNTUL) != 0U) {
		return INA228_CAUSE_SHUNT_UNDER;
	}
	if ((f & INA228_DIAG_MATHOF) != 0U) {
		return INA228_CAUSE_MATH_OVF;
	}
	if ((f & INA228_DIAG_CHARGEOF) != 0U) {
		return INA228_CAUSE_CHARGE_OVF;
	}
	if ((f & INA228_DIAG_ENERGYOF) != 0U) {
		return INA228_CAUSE_ENERGY_OVF;
	}
	if ((raw & INA228_DIAG_CNVRF) != 0U) {
		return INA228_CAUSE_CONV_READY;
	}

	return INA228_CAUSE_NONE;
}

const char *ina228_cause_str(ina228_cause_t cause)
{
	static const char *const names[INA228_CAUSE_COUNT] = {
		"none",	    "mem-error",  "shunt-over", "bus-over",
		"temp-over", "power-over", "bus-under",  "shunt-under",
		"math-ovf",  "charge-ovf", "energy-ovf", "conv-ready",
	};

	/* Unsigned compare catches both a negative cast and an out-of-range
	 * value without tripping -Wtype-limits on an unsigned enum. */
	if ((unsigned int)cause >= (unsigned int)INA228_CAUSE_COUNT) {
		return "invalid";
	}
	return names[(unsigned int)cause];
}

/* ---------------------------------------------------------------------- IDs */

uint16_t ina228_device_id_die(uint16_t device_id)
{
	return (uint16_t)(device_id >> 4);
}

uint8_t ina228_device_id_rev(uint16_t device_id)
{
	return (uint8_t)(device_id & 0x0FU);
}

bool ina228_id_matches(uint16_t manufacturer_id, uint16_t device_id)
{
	return (manufacturer_id == INA228_MANUFACTURER_ID_TI) &&
	       (ina228_device_id_die(device_id) == INA228_DEVICE_ID_DIE);
}

/* --------------------------------------------------------------- rail table */

/*
 * All nine monitors, interface ref §4.2. SOVL defaults are 125 % of each rail's
 * design maximum; ina228_rail_sovl_default() recomputes them from r_shunt_uohm
 * and design_max_ma so the table cannot drift from the shunt values unnoticed.
 *
 * SUVL is parked at "never asserts" everywhere except V_ANT. The antenna bias
 * has an autonomous R77 foldback limiter at ~182 mA, so its 227 mA SOVL is only
 * a backstop for a foldback failure — the useful antenna detector is
 * under-current. 10 mA sits below the 15–30 mA a healthy active antenna draws
 * (interface ref §7) and far above a disconnected one; final value is bench
 * item BM-1.
 */
const ina228_rail_info_t ina228_rail_tbl[INA228_RAIL_COUNT] = {
	[INA228_RAIL_POE] = {
		.addr = 0x40U,
		.name = "V_POE",
		.designator = "U10",
		.shunt_ref = "R30",
		.r_shunt_uohm = 25000U,
		.current_lsb_pa = 3125000U,
		.power_lsb_nw = 10000U,
		.fs_current_ua = 1638400U,
		.design_max_ma = 1200U,
		.nominal_mv = 54000,
		.sovl_code_default = 30000,
		.suvl_code_default = INA228_SUVL_DISABLED,
		.alert_port = 'G',
		.alert_bit = 8U,
	},
	[INA228_RAIL_3V3_STM] = {
		.addr = 0x41U,
		.name = "3V3_STM",
		.designator = "U31",
		.shunt_ref = "R106",
		.r_shunt_uohm = 100000U,
		.current_lsb_pa = 781250U,
		.power_lsb_nw = 2500U,
		.fs_current_ua = 409600U,
		.design_max_ma = 300U,
		.nominal_mv = 3300,
		.sovl_code_default = 30000,
		.suvl_code_default = INA228_SUVL_DISABLED,
		.alert_port = 'G',
		.alert_bit = 9U,
	},
	[INA228_RAIL_5V_DISP] = {
		.addr = 0x42U,
		.name = "5V_DISP",
		.designator = "U32",
		.shunt_ref = "R107",
		.r_shunt_uohm = 100000U,
		.current_lsb_pa = 781250U,
		.power_lsb_nw = 2500U,
		.fs_current_ua = 409600U,
		.design_max_ma = 250U,
		.nominal_mv = 5000,
		.sovl_code_default = 25000,
		.suvl_code_default = INA228_SUVL_DISABLED,
		.alert_port = 'G',
		.alert_bit = 10U,
	},
	[INA228_RAIL_3V3_MAIN] = {
		.addr = 0x43U,
		.name = "3V3",
		.designator = "U30",
		.shunt_ref = "R102",
		.r_shunt_uohm = 50000U,
		.current_lsb_pa = 1562500U,
		.power_lsb_nw = 5000U,
		.fs_current_ua = 819200U,
		.design_max_ma = 450U,
		.nominal_mv = 3300,
		.sovl_code_default = 22500,
		.suvl_code_default = INA228_SUVL_DISABLED,
		.alert_port = 'G',
		.alert_bit = 11U,
	},
	[INA228_RAIL_V_ANT] = {
		.addr = 0x45U,
		.name = "V_ANT",
		.designator = "U26",
		.shunt_ref = "R89",
		.r_shunt_uohm = 150000U,
		.current_lsb_pa = 520833U,
		.power_lsb_nw = 1667U,
		.fs_current_ua = 273067U,
		.design_max_ma = 182U,
		.nominal_mv = 5000,
		.sovl_code_default = 27300,
		.suvl_code_default = 1200, /* 10 mA open-antenna detect */
		.alert_port = 'G',
		.alert_bit = 13U,
	},
	[INA228_RAIL_OCXO] = {
		.addr = 0x46U,
		.name = "OCXO",
		.designator = "U37",
		.shunt_ref = "R126",
		.r_shunt_uohm = 25000U,
		.current_lsb_pa = 3125000U,
		.power_lsb_nw = 10000U,
		.fs_current_ua = 1638400U,
		.design_max_ma = 1200U,
		.nominal_mv = 3327,
		.sovl_code_default = 30000,
		.suvl_code_default = INA228_SUVL_DISABLED,
		.alert_port = 'G',
		.alert_bit = 14U,
	},
	[INA228_RAIL_VCC_RB] = {
		.addr = 0x47U,
		.name = "VCC_RB",
		.designator = "U44",
		.shunt_ref = "R159",
		.r_shunt_uohm = 7000U,
		.current_lsb_pa = 11160714U,
		.power_lsb_nw = 35714U,
		.fs_current_ua = 5851429U,
		.design_max_ma = 2100U,
		.nominal_mv = 15000, /* programmable 4510..24450; see pwrseq */
		.sovl_code_default = 14700,
		.suvl_code_default = INA228_SUVL_DISABLED,
		.alert_port = 'G',
		.alert_bit = 15U,
	},
	[INA228_RAIL_3V3_GPS] = {
		.addr = 0x4AU, /* NOT 0x44 — SHT45 U72 owns that address */
		.name = "3V3_GPS",
		.designator = "U23",
		.shunt_ref = "R72",
		.r_shunt_uohm = 75000U,
		.current_lsb_pa = 1041667U,
		.power_lsb_nw = 3333U,
		.fs_current_ua = 546133U,
		.design_max_ma = 130U,
		.nominal_mv = 3300,
		.sovl_code_default = 9750,
		.suvl_code_default = INA228_SUVL_DISABLED,
		.alert_port = 'G',
		.alert_bit = 12U,
	},
	[INA228_RAIL_PANEL_5V] = {
		.addr = 0x4CU,
		.name = "V_PANEL_LED",
		.designator = "U54",
		.shunt_ref = "R199",
		.r_shunt_uohm = 150000U,
		.current_lsb_pa = 520833U,
		.power_lsb_nw = 1667U,
		.fs_current_ua = 273067U,
		.design_max_ma = 132U,
		.nominal_mv = 5000,
		.sovl_code_default = 19800,
		.suvl_code_default = INA228_SUVL_DISABLED,
		.alert_port = 'F', /* the one INA alert not on GPIOG */
		.alert_bit = 13U,
	},
};

const ina228_rail_info_t *ina228_rail(ina228_rail_t rail)
{
	if ((unsigned int)rail >= (unsigned int)INA228_RAIL_COUNT) {
		return NULL;
	}
	return &ina228_rail_tbl[(unsigned int)rail];
}

int ina228_rail_by_addr(uint8_t addr, ina228_rail_t *out)
{
	int i;

	if (out == NULL) {
		return -EINVAL;
	}

	for (i = 0; i < (int)INA228_RAIL_COUNT; i++) {
		if (ina228_rail_tbl[i].addr == addr) {
			*out = (ina228_rail_t)i;
			return 0;
		}
	}

	return -ENOENT;
}

int ina228_rail_sovl_default(ina228_rail_t rail, int16_t *out)
{
	const ina228_rail_info_t *info = ina228_rail(rail);
	int32_t trip_ua;

	if ((info == NULL) || (out == NULL)) {
		return -EINVAL;
	}

	/* 125 % of the design maximum, in microamps, without losing the half
	 * milliamp that several rails land on. */
	trip_ua = (int32_t)((info->design_max_ma * 1250U));

	return ina228_shunt_code(info->r_shunt_uohm, trip_ua,
				 INA228_ADCRANGE_40_96MV, out);
}
