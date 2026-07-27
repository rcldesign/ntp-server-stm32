/*
 * STS1000 "Meridian" — core/ina228 unit tests.
 *
 * The INA228 codec is pure arithmetic against a published register map, so
 * these tests are vector-driven rather than structural. Three sources of truth
 * are checked independently:
 *
 *   1. The TI INA228 datasheet (SBOS939) register layout — field positions,
 *      the 20-bit-in-24 packing, sign extension, and the exact LSB rationals.
 *   2. The nine-row constant table in `docs/sts1000_firmware_hardware_interface.md`
 *      §4.2 — every CURRENT_LSB, POWER_LSB, full-scale current and SOVL code is
 *      asserted against the number printed in the document, and separately
 *      recomputed from the shunt value so a transcription slip in either the
 *      table or the doc shows up as a failure.
 *   3. The board rules the doc states in prose — GPS at 0x4A (0x44 belongs to
 *      the SHT45), the panel monitor's alert on GPIOF, SHUNT_CAL = 4096 on all
 *      nine, and averaging long enough to swamp the 1 ms panel PWM period.
 *
 * Frames are hand-built byte triples/quintuples, MSB first, exactly as the part
 * clocks them out — the packing is the thing most likely to be wrong, so no
 * helper is allowed to hide it from the assertions that matter.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "ina228/ina228.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* ---------------------------------------------------------- frame builders */

static void fr16(uint8_t out[2], uint16_t v)
{
	out[0] = (uint8_t)(v >> 8);
	out[1] = (uint8_t)(v & 0xFFU);
}

/* A raw 24-bit frame, used directly for POWER. */
static void fr24(uint8_t out[3], uint32_t v)
{
	out[0] = (uint8_t)((v >> 16) & 0xFFU);
	out[1] = (uint8_t)((v >> 8) & 0xFFU);
	out[2] = (uint8_t)(v & 0xFFU);
}

/*
 * A 24-bit frame carrying a 20-bit measurement in bits [23:4]. `junk` fills the
 * reserved low nibble; every decode must ignore it.
 */
static void fr20(uint8_t out[3], uint32_t v20, uint8_t junk)
{
	fr24(out, ((v20 & 0xFFFFFU) << 4) | (junk & 0x0FU));
}

static void fr40(uint8_t out[5], uint64_t v)
{
	out[0] = (uint8_t)((v >> 32) & 0xFFU);
	out[1] = (uint8_t)((v >> 24) & 0xFFU);
	out[2] = (uint8_t)((v >> 16) & 0xFFU);
	out[3] = (uint8_t)((v >> 8) & 0xFFU);
	out[4] = (uint8_t)(v & 0xFFU);
}

/* ------------------------------------------------------------ register map */

static void test_register_widths_match_the_datasheet_map(void)
{
	static const uint8_t two[] = {
		INA228_REG_CONFIG,     INA228_REG_ADC_CONFIG,
		INA228_REG_SHUNT_CAL,  INA228_REG_SHUNT_TEMPCO,
		INA228_REG_DIETEMP,    INA228_REG_DIAG_ALRT,
		INA228_REG_SOVL,       INA228_REG_SUVL,
		INA228_REG_BOVL,       INA228_REG_BUVL,
		INA228_REG_TEMP_LIMIT, INA228_REG_PWR_LIMIT,
		INA228_REG_MANUFACTURER_ID, INA228_REG_DEVICE_ID,
	};
	static const uint8_t three[] = {INA228_REG_VSHUNT, INA228_REG_VBUS,
					INA228_REG_CURRENT, INA228_REG_POWER};
	static const uint8_t five[] = {INA228_REG_ENERGY, INA228_REG_CHARGE};
	size_t i;

	for (i = 0U; i < ARRAY_LEN(two); i++) {
		TEST_ASSERT_EQUAL_size_t(2U, ina228_reg_width(two[i]));
	}
	for (i = 0U; i < ARRAY_LEN(three); i++) {
		TEST_ASSERT_EQUAL_size_t(3U, ina228_reg_width(three[i]));
	}
	for (i = 0U; i < ARRAY_LEN(five); i++) {
		TEST_ASSERT_EQUAL_size_t(5U, ina228_reg_width(five[i]));
	}

	/* Unimplemented addresses report 0 so glue cannot silently read them. */
	TEST_ASSERT_EQUAL_size_t(0U, ina228_reg_width(0x12U));
	TEST_ASSERT_EQUAL_size_t(0U, ina228_reg_width(0x3DU));
	TEST_ASSERT_EQUAL_size_t(0U, ina228_reg_width(0xFFU));
}

static void test_register_addresses_are_the_documented_ones(void)
{
	TEST_ASSERT_EQUAL_HEX8(0x00, INA228_REG_CONFIG);
	TEST_ASSERT_EQUAL_HEX8(0x01, INA228_REG_ADC_CONFIG);
	TEST_ASSERT_EQUAL_HEX8(0x02, INA228_REG_SHUNT_CAL);
	TEST_ASSERT_EQUAL_HEX8(0x04, INA228_REG_VSHUNT);
	TEST_ASSERT_EQUAL_HEX8(0x05, INA228_REG_VBUS);
	TEST_ASSERT_EQUAL_HEX8(0x06, INA228_REG_DIETEMP);
	TEST_ASSERT_EQUAL_HEX8(0x07, INA228_REG_CURRENT);
	TEST_ASSERT_EQUAL_HEX8(0x08, INA228_REG_POWER);
	TEST_ASSERT_EQUAL_HEX8(0x09, INA228_REG_ENERGY);
	TEST_ASSERT_EQUAL_HEX8(0x0A, INA228_REG_CHARGE);
	TEST_ASSERT_EQUAL_HEX8(0x0B, INA228_REG_DIAG_ALRT);
	TEST_ASSERT_EQUAL_HEX8(0x0C, INA228_REG_SOVL);
	TEST_ASSERT_EQUAL_HEX8(0x0D, INA228_REG_SUVL);
	TEST_ASSERT_EQUAL_HEX8(0x0E, INA228_REG_BOVL);
	TEST_ASSERT_EQUAL_HEX8(0x0F, INA228_REG_BUVL);
	TEST_ASSERT_EQUAL_HEX8(0x10, INA228_REG_TEMP_LIMIT);
	TEST_ASSERT_EQUAL_HEX8(0x11, INA228_REG_PWR_LIMIT);
	TEST_ASSERT_EQUAL_HEX8(0x3E, INA228_REG_MANUFACTURER_ID);
	TEST_ASSERT_EQUAL_HEX8(0x3F, INA228_REG_DEVICE_ID);
}

/* ----------------------------------------------------------------- CONFIG */

static void test_config_default_selects_the_board_adcrange(void)
{
	ina228_config_t cfg;

	ina228_config_default(&cfg);

	/* Interface ref §4.2: ADCRANGE = 1 on every one of the nine. */
	TEST_ASSERT_EQUAL_INT(INA228_ADCRANGE_40_96MV, cfg.adcrange);
	TEST_ASSERT_FALSE(cfg.rst);
	TEST_ASSERT_FALSE(cfg.rstacc);
	TEST_ASSERT_FALSE(cfg.tempcomp);
	TEST_ASSERT_EQUAL_UINT8(0U, cfg.convdly_steps);

	/* ADCRANGE is bit 4 and nothing else is set. */
	TEST_ASSERT_EQUAL_HEX16(0x0010, ina228_config_encode(&cfg));

	ina228_config_default(NULL); /* must not fault */
}

static void test_config_field_positions(void)
{
	ina228_config_t cfg;

	ina228_config_default(&cfg);
	cfg.rst = true;
	TEST_ASSERT_EQUAL_HEX16(0x8010, ina228_config_encode(&cfg));

	ina228_config_default(&cfg);
	cfg.rstacc = true;
	TEST_ASSERT_EQUAL_HEX16(0x4010, ina228_config_encode(&cfg));

	ina228_config_default(&cfg);
	cfg.tempcomp = true;
	TEST_ASSERT_EQUAL_HEX16(0x0030, ina228_config_encode(&cfg));

	ina228_config_default(&cfg);
	cfg.adcrange = INA228_ADCRANGE_163_84MV;
	TEST_ASSERT_EQUAL_HEX16(0x0000, ina228_config_encode(&cfg));

	/* CONVDLY occupies bits [13:6] at 2 ms per step. */
	ina228_config_default(&cfg);
	cfg.convdly_steps = 1U;
	TEST_ASSERT_EQUAL_HEX16(0x0050, ina228_config_encode(&cfg));
	cfg.convdly_steps = 0xFFU;
	TEST_ASSERT_EQUAL_HEX16(0x3FD0, ina228_config_encode(&cfg));

	TEST_ASSERT_EQUAL_HEX16(0x0000, ina228_config_encode(NULL));
}

static void test_config_round_trips_every_field(void)
{
	ina228_config_t in;
	ina228_config_t out;
	uint16_t raw;

	in.rst = true;
	in.rstacc = false;
	in.convdly_steps = 0x5AU;
	in.tempcomp = true;
	in.adcrange = INA228_ADCRANGE_40_96MV;

	raw = ina228_config_encode(&in);
	TEST_ASSERT_EQUAL_INT(0, ina228_config_decode(raw, &out));

	TEST_ASSERT_EQUAL_INT(in.rst, out.rst);
	TEST_ASSERT_EQUAL_INT(in.rstacc, out.rstacc);
	TEST_ASSERT_EQUAL_UINT8(in.convdly_steps, out.convdly_steps);
	TEST_ASSERT_EQUAL_INT(in.tempcomp, out.tempcomp);
	TEST_ASSERT_EQUAL_INT(in.adcrange, out.adcrange);

	TEST_ASSERT_EQUAL_INT(-EINVAL, ina228_config_decode(raw, NULL));
}

/* ------------------------------------------------------------- ADC_CONFIG */

static void test_adc_config_field_positions(void)
{
	ina228_adc_config_t cfg;

	cfg.mode = INA228_MODE_CONT_ALL;
	cfg.vbusct = INA228_CT_50US;
	cfg.vshct = INA228_CT_50US;
	cfg.vtct = INA228_CT_50US;
	cfg.avg = INA228_AVG_1;
	TEST_ASSERT_EQUAL_HEX16(0xF000, ina228_adc_config_encode(&cfg));

	cfg.mode = INA228_MODE_SHUTDOWN;
	cfg.vbusct = INA228_CT_4120US; /* bits [11:9] */
	TEST_ASSERT_EQUAL_HEX16(0x0E00, ina228_adc_config_encode(&cfg));

	cfg.vbusct = INA228_CT_50US;
	cfg.vshct = INA228_CT_4120US; /* bits [8:6] */
	TEST_ASSERT_EQUAL_HEX16(0x01C0, ina228_adc_config_encode(&cfg));

	cfg.vshct = INA228_CT_50US;
	cfg.vtct = INA228_CT_4120US; /* bits [5:3] */
	TEST_ASSERT_EQUAL_HEX16(0x0038, ina228_adc_config_encode(&cfg));

	cfg.vtct = INA228_CT_50US;
	cfg.avg = INA228_AVG_1024; /* bits [2:0] */
	TEST_ASSERT_EQUAL_HEX16(0x0007, ina228_adc_config_encode(&cfg));

	TEST_ASSERT_EQUAL_HEX16(0x0000, ina228_adc_config_encode(NULL));
}

static void test_adc_config_round_trips(void)
{
	ina228_adc_config_t in;
	ina228_adc_config_t out;

	in.mode = INA228_MODE_CONT_TEMP_SHUNT;
	in.vbusct = INA228_CT_280US;
	in.vshct = INA228_CT_2074US;
	in.vtct = INA228_CT_150US;
	in.avg = INA228_AVG_64;

	TEST_ASSERT_EQUAL_INT(
		0, ina228_adc_config_decode(ina228_adc_config_encode(&in), &out));

	TEST_ASSERT_EQUAL_INT(in.mode, out.mode);
	TEST_ASSERT_EQUAL_INT(in.vbusct, out.vbusct);
	TEST_ASSERT_EQUAL_INT(in.vshct, out.vshct);
	TEST_ASSERT_EQUAL_INT(in.vtct, out.vtct);
	TEST_ASSERT_EQUAL_INT(in.avg, out.avg);

	TEST_ASSERT_EQUAL_INT(-EINVAL, ina228_adc_config_decode(0U, NULL));
}

static void test_avg_and_conversion_time_tables(void)
{
	static const uint32_t avg[] = {1U, 4U, 16U, 64U, 128U, 256U, 512U, 1024U};
	static const uint32_t ct[] = {50U,  84U,   150U,  280U,
				      540U, 1052U, 2074U, 4120U};
	unsigned int i;

	for (i = 0U; i < 8U; i++) {
		TEST_ASSERT_EQUAL_UINT32(avg[i],
					 ina228_avg_samples((ina228_avg_t)i));
		TEST_ASSERT_EQUAL_UINT32(ct[i], ina228_ct_us((ina228_ct_t)i));
	}
}

static void test_conversion_interval_counts_only_converted_channels(void)
{
	ina228_adc_config_t cfg;

	cfg.vbusct = INA228_CT_1052US;
	cfg.vshct = INA228_CT_540US;
	cfg.vtct = INA228_CT_280US;
	cfg.avg = INA228_AVG_1;

	/* MODE bits [2:0] gate temperature, shunt and bus respectively. */
	cfg.mode = INA228_MODE_CONT_BUS;
	TEST_ASSERT_EQUAL_UINT32(1052U, ina228_conv_interval_us(&cfg));
	cfg.mode = INA228_MODE_CONT_SHUNT;
	TEST_ASSERT_EQUAL_UINT32(540U, ina228_conv_interval_us(&cfg));
	cfg.mode = INA228_MODE_CONT_TEMP;
	TEST_ASSERT_EQUAL_UINT32(280U, ina228_conv_interval_us(&cfg));
	cfg.mode = INA228_MODE_CONT_SHUNT_BUS;
	TEST_ASSERT_EQUAL_UINT32(1592U, ina228_conv_interval_us(&cfg));
	cfg.mode = INA228_MODE_CONT_ALL;
	TEST_ASSERT_EQUAL_UINT32(1872U, ina228_conv_interval_us(&cfg));

	/* Averaging multiplies the whole loop. */
	cfg.avg = INA228_AVG_16;
	TEST_ASSERT_EQUAL_UINT32(1872U * 16U, ina228_conv_interval_us(&cfg));

	/* Triggered and shutdown modes have no repeating interval. */
	cfg.mode = INA228_MODE_TRIG_ALL;
	TEST_ASSERT_EQUAL_UINT32(0U, ina228_conv_interval_us(&cfg));
	cfg.mode = INA228_MODE_SHUTDOWN;
	TEST_ASSERT_EQUAL_UINT32(0U, ina228_conv_interval_us(&cfg));
	cfg.mode = INA228_MODE_SHUTDOWN_ALT;
	TEST_ASSERT_EQUAL_UINT32(0U, ina228_conv_interval_us(&cfg));

	TEST_ASSERT_EQUAL_UINT32(0U, ina228_conv_interval_us(NULL));
}

static void test_default_averaging_swamps_the_panel_pwm_period(void)
{
	ina228_adc_config_t cfg;
	uint32_t interval;

	ina228_adc_config_default(&cfg);
	interval = ina228_conv_interval_us(&cfg);

	/*
	 * Interface ref §4.1: the panel-LED monitor must average over a window
	 * far longer than the 1 ms PANEL_LED_PWM period, or its ALERT fires on
	 * the PWM chop rather than on average current. 3 x 1052 us x 16.
	 */
	TEST_ASSERT_EQUAL_UINT32(50496U, interval);
	TEST_ASSERT_GREATER_THAN_UINT32(1000U * 10U, interval);

	ina228_adc_config_default(NULL); /* must not fault */
}

/* -------------------------------------------------------------- SHUNT_CAL */

static void test_shunt_cal_default_is_4096_for_every_rail(void)
{
	/*
	 * Interface ref §4.2: with Max_Expected_Current set to each device's own
	 * full scale, R_SHUNT cancels out of the calibration formula and the
	 * same constant serves all nine. Re-derive it here rather than restating
	 * it: SHUNT_CAL = 4 * 13107.2e6 * 78.125e-9, written with the decimal
	 * points cleared as 13107.2e6 = 131072e5 and 78.125e-9 = 78125e-12.
	 */
	TEST_ASSERT_EQUAL_UINT16(4096U, INA228_SHUNT_CAL_DEFAULT);
	TEST_ASSERT_EQUAL_UINT32(
		(uint32_t)((UINT64_C(4) * UINT64_C(131072) * UINT64_C(78125)) /
			   UINT64_C(10000000)),
		INA228_SHUNT_CAL_DEFAULT);
}

static void test_shunt_cal_trim_scales_by_the_measurement_ratio(void)
{
	uint16_t cal = 0U;

	/* Reporting 1 % low means the calibration must go up 1 %. */
	TEST_ASSERT_EQUAL_INT(
		0, ina228_shunt_cal_trim(4096U, 100000U, 99010U, &cal));
	TEST_ASSERT_EQUAL_UINT16(4137U, cal);

	/* Exact agreement leaves the nominal untouched. */
	TEST_ASSERT_EQUAL_INT(
		0, ina228_shunt_cal_trim(4096U, 250000U, 250000U, &cal));
	TEST_ASSERT_EQUAL_UINT16(4096U, cal);

	/* Rounds rather than truncates: 4096 * 3 / 2 is exact, 4096/3 is not. */
	TEST_ASSERT_EQUAL_INT(0, ina228_shunt_cal_trim(4096U, 1U, 3U, &cal));
	TEST_ASSERT_EQUAL_UINT16(1365U, cal); /* 1365.33 -> 1365 */
	TEST_ASSERT_EQUAL_INT(0, ina228_shunt_cal_trim(4096U, 2U, 3U, &cal));
	TEST_ASSERT_EQUAL_UINT16(2731U, cal); /* 2730.67 -> 2731 */
}

static void test_shunt_cal_trim_clamps_and_validates(void)
{
	uint16_t cal = 0U;

	/* 15-bit field: a wild ratio saturates instead of wrapping. */
	TEST_ASSERT_EQUAL_INT(
		-ERANGE, ina228_shunt_cal_trim(4096U, 1000U, 1U, &cal));
	TEST_ASSERT_EQUAL_UINT16(INA228_SHUNT_CAL_MAX, cal);
	TEST_ASSERT_EQUAL_UINT16(0x7FFFU, INA228_SHUNT_CAL_MAX);

	/* A zero reading yields no derivable ratio. */
	TEST_ASSERT_EQUAL_INT(
		-EINVAL, ina228_shunt_cal_trim(4096U, 1000U, 0U, &cal));
	TEST_ASSERT_EQUAL_INT(
		-EINVAL, ina228_shunt_cal_trim(4096U, 1000U, 1000U, NULL));

	/*
	 * A zero reference current must be rejected, not accepted as a ratio of
	 * 0. SHUNT_CAL = 0 is a legal register value the INA228 takes without
	 * complaint, and then reports 0 A on every reading — a monitor that
	 * looks calibrated and measures nothing. Refuse it rather than blind the
	 * rail (finding L3).
	 */
	cal = 0xABCDU;
	TEST_ASSERT_EQUAL_INT(
		-EINVAL, ina228_shunt_cal_trim(4096U, 0U, 1000U, &cal));
	TEST_ASSERT_EQUAL_UINT16(0xABCDU, cal); /* left untouched */
}

/* ------------------------------------------------- threshold code encoding */

/*
 * The nine SOVL defaults printed in interface ref §4.2. These are the numbers a
 * bring-up engineer will read off the document and expect to see in a register
 * dump, so they are asserted literally.
 */
static void test_sovl_defaults_match_the_documented_table(void)
{
	struct {
		uint32_t r_uohm;
		int32_t trip_ua;
		int16_t code;
		const char *rail;
	} const v[] = {
		{25000U, 1500000, 30000, "U10 0x40 V_POE"},
		{100000U, 375000, 30000, "U31 0x41 3V3_STM"},
		{100000U, 312500, 25000, "U32 0x42 5V_DISP"},
		{50000U, 562500, 22500, "U30 0x43 3V3"},
		{150000U, 227500, 27300, "U26 0x45 V_ANT"},
		{25000U, 1500000, 30000, "U37 0x46 OCXO"},
		{7000U, 2625000, 14700, "U44 0x47 VCC_RB"},
		{75000U, 162500, 9750, "U23 0x4A 3V3_GPS"},
		{150000U, 165000, 19800, "U54 0x4C panel"},
	};
	size_t i;
	int16_t code;

	for (i = 0U; i < ARRAY_LEN(v); i++) {
		TEST_ASSERT_EQUAL_INT(0,
				      ina228_shunt_code(v[i].r_uohm, v[i].trip_ua,
							INA228_ADCRANGE_40_96MV,
							&code));
		TEST_ASSERT_EQUAL_INT16_MESSAGE(v[i].code, code, v[i].rail);
	}
}

static void test_rail_table_sovl_defaults_are_recomputable(void)
{
	int i;

	/*
	 * The stored defaults must equal 125 % of each rail's design maximum
	 * computed from that rail's shunt. This is what catches a shunt value
	 * changing without its threshold following: seven of the nine changed in
	 * the last hardware revision (interface ref §10 caution 7).
	 */
	for (i = 0; i < (int)INA228_RAIL_COUNT; i++) {
		const ina228_rail_info_t *r = ina228_rail((ina228_rail_t)i);
		int16_t code = 0;

		TEST_ASSERT_EQUAL_INT(
			0, ina228_rail_sovl_default((ina228_rail_t)i, &code));
		TEST_ASSERT_EQUAL_INT16_MESSAGE(r->sovl_code_default, code,
						r->name);
	}

	TEST_ASSERT_EQUAL_INT(
		-EINVAL, ina228_rail_sovl_default(INA228_RAIL_COUNT, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ina228_rail_sovl_default(INA228_RAIL_POE, NULL));
}

static void test_shunt_code_full_scale_and_sign(void)
{
	int16_t code = 0;

	/*
	 * The threshold registers are 16-bit *signed*, so they reach 32767
	 * codes = 40.95875 mV — one LSB short of the ADC's own +40.96 mV full
	 * scale. A trip set at exactly full scale therefore has to clamp, and
	 * must say so rather than wrapping to a negative threshold.
	 */
	TEST_ASSERT_EQUAL_INT(0, ina228_shunt_code(25000U, 1638350,
						   INA228_ADCRANGE_40_96MV, &code));
	TEST_ASSERT_EQUAL_INT16(32767, code);
	TEST_ASSERT_EQUAL_INT(-ERANGE,
			      ina228_shunt_code(25000U, 1638400,
						INA228_ADCRANGE_40_96MV, &code));
	TEST_ASSERT_EQUAL_INT16(INA228_SHUNT_CODE_MAX, code);

	/* Negative trips are legal (reverse current). */
	TEST_ASSERT_EQUAL_INT(0, ina228_shunt_code(25000U, -1500000,
						   INA228_ADCRANGE_40_96MV, &code));
	TEST_ASSERT_EQUAL_INT16(-30000, code);

	/* Beyond full scale the code saturates and says so. */
	TEST_ASSERT_EQUAL_INT(-ERANGE,
			      ina228_shunt_code(25000U, 5000000,
						INA228_ADCRANGE_40_96MV, &code));
	TEST_ASSERT_EQUAL_INT16(INA228_SHUNT_CODE_MAX, code);
	TEST_ASSERT_EQUAL_INT(-ERANGE,
			      ina228_shunt_code(25000U, -5000000,
						INA228_ADCRANGE_40_96MV, &code));
	TEST_ASSERT_EQUAL_INT16(INA228_SHUNT_CODE_MIN, code);

	/* The wide range is 4x coarser, so the same current is a quarter code. */
	TEST_ASSERT_EQUAL_INT(0, ina228_shunt_code(25000U, 1500000,
						   INA228_ADCRANGE_163_84MV, &code));
	TEST_ASSERT_EQUAL_INT16(7500, code);

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ina228_shunt_code(0U, 1000,
						INA228_ADCRANGE_40_96MV, &code));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ina228_shunt_code(25000U, 1000,
						INA228_ADCRANGE_40_96MV, NULL));
}

static void test_shunt_code_milliamp_wrapper(void)
{
	int16_t a = 0;
	int16_t b = 0;

	TEST_ASSERT_EQUAL_INT(
		0, ina228_shunt_code_ma(25000U, 1500, INA228_ADCRANGE_40_96MV, &a));
	TEST_ASSERT_EQUAL_INT(0, ina228_shunt_code(25000U, 1500000,
						   INA228_ADCRANGE_40_96MV, &b));
	TEST_ASSERT_EQUAL_INT16(b, a);
	TEST_ASSERT_EQUAL_INT16(30000, a);

	/* Guarded against overflowing the microamp conversion. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ina228_shunt_code_ma(25000U, 3000000,
						   INA228_ADCRANGE_40_96MV, &a));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ina228_shunt_code_ma(25000U, -3000000,
						   INA228_ADCRANGE_40_96MV, &a));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ina228_shunt_code_ma(0U, 100,
						   INA228_ADCRANGE_40_96MV, &a));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ina228_shunt_code_ma(25000U, 100,
						   INA228_ADCRANGE_40_96MV, NULL));
}

static void test_shunt_code_inverts_back_to_current(void)
{
	/* Round-trips exactly where the code lands on a whole microamp. */
	TEST_ASSERT_EQUAL_INT64(1500000, ina228_shunt_code_to_ua(
					       25000U, 30000,
					       INA228_ADCRANGE_40_96MV));
	TEST_ASSERT_EQUAL_INT64(2625000, ina228_shunt_code_to_ua(
					       7000U, 14700,
					       INA228_ADCRANGE_40_96MV));
	TEST_ASSERT_EQUAL_INT64(-1500000, ina228_shunt_code_to_ua(
						25000U, -30000,
						INA228_ADCRANGE_40_96MV));
	TEST_ASSERT_EQUAL_INT64(6000000, ina228_shunt_code_to_ua(
					       25000U, 30000,
					       INA228_ADCRANGE_163_84MV));
	TEST_ASSERT_EQUAL_INT64(0, ina228_shunt_code_to_ua(
					 0U, 30000, INA228_ADCRANGE_40_96MV));
}

static void test_v_ant_suvl_detects_an_open_antenna(void)
{
	const ina228_rail_info_t *ant = ina228_rail(INA228_RAIL_V_ANT);
	int16_t code = 0;
	int64_t ua;

	/*
	 * Interface ref §4.2/§7: the R77 foldback clamps at ~182 mA on its own,
	 * so V_ANT's SOVL is only a backstop; the useful detector is
	 * under-current. The stored SUVL must sit below the 15-30 mA a healthy
	 * active antenna draws and well above a disconnected one.
	 */
	TEST_ASSERT_EQUAL_INT(0, ina228_shunt_code(ant->r_shunt_uohm, 10000,
						   INA228_ADCRANGE_40_96MV, &code));
	TEST_ASSERT_EQUAL_INT16(code, ant->suvl_code_default);

	ua = ina228_shunt_code_to_ua(ant->r_shunt_uohm, ant->suvl_code_default,
				     INA228_ADCRANGE_40_96MV);
	TEST_ASSERT_EQUAL_INT64(10000, ua);
	TEST_ASSERT_TRUE(ua < 15000); /* below the healthy floor */
	TEST_ASSERT_TRUE(ua > 0);
}

static void test_every_other_rail_parks_suvl_where_it_cannot_assert(void)
{
	int i;

	/*
	 * The INA228 has no per-comparator enable, so an unwanted threshold is
	 * disabled by parking it beyond the ADC's reach. -32768 codes is
	 * -40.96 mV, below full scale, so it can never trip.
	 */
	TEST_ASSERT_EQUAL_INT16(-32768, INA228_SUVL_DISABLED);
	TEST_ASSERT_EQUAL_INT16(32767, INA228_SOVL_DISABLED);

	for (i = 0; i < (int)INA228_RAIL_COUNT; i++) {
		const ina228_rail_info_t *r = ina228_rail((ina228_rail_t)i);

		if (i == (int)INA228_RAIL_V_ANT) {
			continue;
		}
		TEST_ASSERT_EQUAL_INT16_MESSAGE(INA228_SUVL_DISABLED,
						r->suvl_code_default, r->name);
	}
}

static void test_bus_temp_and_power_threshold_codes(void)
{
	uint16_t u = 0U;
	int16_t s = 0;

	/* BOVL/BUVL: 3.125 mV/LSB, so 85 V full scale is code 27200. */
	TEST_ASSERT_EQUAL_INT(0, ina228_bus_code(85000, &u));
	TEST_ASSERT_EQUAL_UINT16(27200U, u);
	TEST_ASSERT_EQUAL_INT(0, ina228_bus_code(54000, &u));
	TEST_ASSERT_EQUAL_UINT16(17280U, u);
	TEST_ASSERT_EQUAL_INT(0, ina228_bus_code(3300, &u));
	TEST_ASSERT_EQUAL_UINT16(1056U, u);
	TEST_ASSERT_EQUAL_INT(0, ina228_bus_code(0, &u));
	TEST_ASSERT_EQUAL_UINT16(0U, u);
	TEST_ASSERT_EQUAL_INT(-ERANGE, ina228_bus_code(1000000, &u));
	TEST_ASSERT_EQUAL_UINT16(0xFFFFU, u);
	TEST_ASSERT_EQUAL_INT(-EINVAL, ina228_bus_code(-1, &u));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ina228_bus_code(1000, NULL));

	/* TEMP_LIMIT shares DIETEMP's 7.8125 m degC scale. */
	TEST_ASSERT_EQUAL_INT(0, ina228_temp_code(100000, &s));
	TEST_ASSERT_EQUAL_INT16(12800, s);
	TEST_ASSERT_EQUAL_INT(0, ina228_temp_code(-40000, &s));
	TEST_ASSERT_EQUAL_INT16(-5120, s);
	TEST_ASSERT_EQUAL_INT(-ERANGE, ina228_temp_code(1000000, &s));
	TEST_ASSERT_EQUAL_INT16(INA228_SHUNT_CODE_MAX, s);
	TEST_ASSERT_EQUAL_INT(-ERANGE, ina228_temp_code(-1000000, &s));
	TEST_ASSERT_EQUAL_INT16(INA228_SHUNT_CODE_MIN, s);
	TEST_ASSERT_EQUAL_INT(-EINVAL, ina228_temp_code(0, NULL));

	/* PWR_LIMIT compares against POWER[23:8], so one code is 256 LSBs.
	 * On the PoE rail POWER_LSB is 10 uW, hence 2.56 mW per code. */
	TEST_ASSERT_EQUAL_INT(0, ina228_power_code(25000U, 2560000,
						   INA228_ADCRANGE_40_96MV, &u));
	TEST_ASSERT_EQUAL_UINT16(1U, u);
	TEST_ASSERT_EQUAL_INT(0, ina228_power_code(25000U, 25600000000LL,
						   INA228_ADCRANGE_40_96MV, &u));
	TEST_ASSERT_EQUAL_UINT16(10000U, u);
	TEST_ASSERT_EQUAL_INT(-ERANGE,
			      ina228_power_code(25000U, 1000000000000LL,
						INA228_ADCRANGE_40_96MV, &u));
	TEST_ASSERT_EQUAL_UINT16(0xFFFFU, u);
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ina228_power_code(0U, 1000,
						INA228_ADCRANGE_40_96MV, &u));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ina228_power_code(25000U, -1,
						INA228_ADCRANGE_40_96MV, &u));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ina228_power_code(25000U, 1000,
						INA228_ADCRANGE_40_96MV, NULL));
}

/* ------------------------------------------------------- raw frame decodes */

static void test_raw20_packs_into_bits_23_to_4(void)
{
	uint8_t f[3];

	/* The reserved low nibble must not reach the value. */
	fr20(f, 1U, 0x0U);
	TEST_ASSERT_EQUAL_INT32(1, ina228_raw20_signed(f));
	fr20(f, 1U, 0xFU);
	TEST_ASSERT_EQUAL_INT32(1, ina228_raw20_signed(f));
	TEST_ASSERT_EQUAL_UINT32(1U, ina228_raw20_unsigned(f));

	/* Hand-built, not builder-built: 0x00 0x00 0x1F is one LSB. */
	f[0] = 0x00U;
	f[1] = 0x00U;
	f[2] = 0x1FU;
	TEST_ASSERT_EQUAL_INT32(1, ina228_raw20_signed(f));
	TEST_ASSERT_EQUAL_UINT32(0x00001FU, ina228_raw24(f));
}

static void test_raw20_sign_extends_both_polarities(void)
{
	uint8_t f[3];

	/* Largest positive: 0x7FFFF. */
	f[0] = 0x7FU;
	f[1] = 0xFFU;
	f[2] = 0xF0U;
	TEST_ASSERT_EQUAL_INT32(524287, ina228_raw20_signed(f));
	TEST_ASSERT_EQUAL_UINT32(524287U, ina228_raw20_unsigned(f));

	/* Most negative: 0x80000. */
	f[0] = 0x80U;
	f[1] = 0x00U;
	f[2] = 0x00U;
	TEST_ASSERT_EQUAL_INT32(-524288, ina228_raw20_signed(f));
	TEST_ASSERT_EQUAL_UINT32(524288U, ina228_raw20_unsigned(f));

	/* Minus one: 0xFFFFF. */
	f[0] = 0xFFU;
	f[1] = 0xFFU;
	f[2] = 0xF0U;
	TEST_ASSERT_EQUAL_INT32(-1, ina228_raw20_signed(f));
	TEST_ASSERT_EQUAL_UINT32(1048575U, ina228_raw20_unsigned(f));

	/* Zero. */
	memset(f, 0, sizeof(f));
	TEST_ASSERT_EQUAL_INT32(0, ina228_raw20_signed(f));
}

static void test_raw24_and_raw40_frames(void)
{
	uint8_t f3[3];
	uint8_t f5[5];

	fr24(f3, 0x123456U);
	TEST_ASSERT_EQUAL_UINT32(0x123456U, ina228_raw24(f3));
	fr24(f3, 0xFFFFFFU);
	TEST_ASSERT_EQUAL_UINT32(0xFFFFFFU, ina228_raw24(f3));

	fr40(f5, UINT64_C(0x123456789A));
	TEST_ASSERT_EQUAL_UINT64(UINT64_C(0x123456789A), ina228_raw40(f5));
	TEST_ASSERT_EQUAL_INT64(INT64_C(0x123456789A), ina228_raw40_signed(f5));

	/* 40-bit sign extension. */
	fr40(f5, UINT64_C(0xFFFFFFFFFF));
	TEST_ASSERT_EQUAL_UINT64(UINT64_C(0xFFFFFFFFFF), ina228_raw40(f5));
	TEST_ASSERT_EQUAL_INT64(-1, ina228_raw40_signed(f5));

	fr40(f5, UINT64_C(0x8000000000));
	TEST_ASSERT_EQUAL_INT64(INT64_C(-549755813888), ina228_raw40_signed(f5));

	fr40(f5, UINT64_C(0x7FFFFFFFFF));
	TEST_ASSERT_EQUAL_INT64(INT64_C(549755813887), ina228_raw40_signed(f5));
}

static void test_raw_decoders_tolerate_a_null_frame(void)
{
	/* Core code must never trap the host runner, so a NULL frame reads 0. */
	TEST_ASSERT_EQUAL_UINT16(0U, ina228_raw16(NULL));
	TEST_ASSERT_EQUAL_UINT32(0U, ina228_raw24(NULL));
	TEST_ASSERT_EQUAL_UINT32(0U, ina228_raw20_unsigned(NULL));
	TEST_ASSERT_EQUAL_INT32(0, ina228_raw20_signed(NULL));
	TEST_ASSERT_EQUAL_UINT64(0U, ina228_raw40(NULL));
	TEST_ASSERT_EQUAL_INT64(0, ina228_raw40_signed(NULL));
}

/* ------------------------------------------------------- value conversions */

static void test_vshunt_scales_at_78_125_nv(void)
{
	uint8_t f[3];

	fr20(f, 1U, 0U);
	TEST_ASSERT_EQUAL_INT32(78, ina228_vshunt_nv(f, INA228_ADCRANGE_40_96MV));
	TEST_ASSERT_EQUAL_INT32(313,
				ina228_vshunt_nv(f, INA228_ADCRANGE_163_84MV));

	/* Eight LSBs is exactly 625 nV — the rational made whole. */
	fr20(f, 8U, 0U);
	TEST_ASSERT_EQUAL_INT32(625, ina228_vshunt_nv(f, INA228_ADCRANGE_40_96MV));

	/* Most negative code is exactly -40.96 mV. */
	f[0] = 0x80U;
	f[1] = 0x00U;
	f[2] = 0x00U;
	TEST_ASSERT_EQUAL_INT32(-40960000,
				ina228_vshunt_nv(f, INA228_ADCRANGE_40_96MV));
	TEST_ASSERT_EQUAL_INT32(-163840000,
				ina228_vshunt_nv(f, INA228_ADCRANGE_163_84MV));

	/* Largest positive code, rounded from 40 959 921.875 nV. */
	f[0] = 0x7FU;
	f[1] = 0xFFU;
	f[2] = 0xF0U;
	TEST_ASSERT_EQUAL_INT32(40959922,
				ina228_vshunt_nv(f, INA228_ADCRANGE_40_96MV));

	/* Rounding is symmetric about zero. */
	fr20(f, 0xFFFFFU, 0U); /* -1 */
	TEST_ASSERT_EQUAL_INT32(-78, ina228_vshunt_nv(f, INA228_ADCRANGE_40_96MV));
}

static void test_vbus_reads_the_54v_and_24v_rails_with_no_divider(void)
{
	uint8_t f[3];

	/*
	 * Interface ref §4.2: VBUS LSB is 195.3125 uV over 0-85 V, so the 54 V
	 * PoE bus and the 24 V VCC_RB read directly with no divider and no
	 * scaling in firmware.
	 */
	fr20(f, 276480U, 0xFU);
	TEST_ASSERT_EQUAL_INT32(54000000, ina228_vbus_uv(f));
	TEST_ASSERT_EQUAL_INT32(54000, ina228_vbus_mv(f));

	fr20(f, 16896U, 0U);
	TEST_ASSERT_EQUAL_INT32(3300000, ina228_vbus_uv(f));
	TEST_ASSERT_EQUAL_INT32(3300, ina228_vbus_mv(f));

	fr20(f, 125184U, 0U); /* 24.45 V pedestal */
	TEST_ASSERT_EQUAL_INT32(24450000, ina228_vbus_uv(f));
	TEST_ASSERT_EQUAL_INT32(24450, ina228_vbus_mv(f));

	fr20(f, 1U, 0U);
	TEST_ASSERT_EQUAL_INT32(195, ina228_vbus_uv(f)); /* 195.3125 -> 195 */

	fr20(f, 16U, 0U);
	TEST_ASSERT_EQUAL_INT32(3125, ina228_vbus_uv(f)); /* exact */

	/* Full 20-bit scale; the part itself is specified only to 85 V. */
	fr20(f, 0xFFFFFU, 0U);
	TEST_ASSERT_EQUAL_INT32(204799805, ina228_vbus_uv(f));

	memset(f, 0, sizeof(f));
	TEST_ASSERT_EQUAL_INT32(0, ina228_vbus_uv(f));
}

static void test_dietemp_scales_at_7_8125_millidegrees(void)
{
	uint8_t f[2];

	fr16(f, 3200U);
	TEST_ASSERT_EQUAL_INT32(25000, ina228_dietemp_mc(f)); /* +25.000 C */

	fr16(f, (uint16_t)(int16_t)-5120);
	TEST_ASSERT_EQUAL_INT32(-40000, ina228_dietemp_mc(f)); /* -40.000 C */

	fr16(f, 16U);
	TEST_ASSERT_EQUAL_INT32(125, ina228_dietemp_mc(f)); /* exact rational */

	fr16(f, 1U);
	TEST_ASSERT_EQUAL_INT32(8, ina228_dietemp_mc(f)); /* 7.8125 -> 8 */

	fr16(f, 0xFFFFU);
	TEST_ASSERT_EQUAL_INT32(-8, ina228_dietemp_mc(f));

	fr16(f, 0x7FFFU);
	TEST_ASSERT_EQUAL_INT32(255992, ina228_dietemp_mc(f));

	fr16(f, 0x8000U);
	TEST_ASSERT_EQUAL_INT32(-256000, ina228_dietemp_mc(f));
}

static void test_one_current_lsb_matches_the_documented_table(void)
{
	uint8_t f[3];
	int i;

	/*
	 * Feed each monitor a raw CURRENT of exactly one count and check the
	 * decoded nanoamps against the CURRENT_LSB the documentation prints.
	 * This is the single most consequential constant on the board: an
	 * uncalibrated or mis-scaled monitor reads wrong with no error flag
	 * (interface ref §10 caution 7).
	 */
	static const int64_t doc_lsb_na[INA228_RAIL_COUNT] = {
		3125,  /* U10 0x40  3.125 uA */
		781,   /* U31 0x41  781.25 nA -> 781 */
		781,   /* U32 0x42 */
		1563,  /* U30 0x43  1.5625 uA -> 1563 */
		521,   /* U26 0x45  520.833 nA -> 521 */
		3125,  /* U37 0x46 */
		11161, /* U44 0x47  11.1607 uA -> 11161 */
		1042,  /* U23 0x4A  1.04167 uA -> 1042 */
		521,   /* U54 0x4C */
	};

	fr20(f, 1U, 0xFU);
	for (i = 0; i < (int)INA228_RAIL_COUNT; i++) {
		const ina228_rail_info_t *r = ina228_rail((ina228_rail_t)i);

		TEST_ASSERT_EQUAL_INT64_MESSAGE(
			doc_lsb_na[i],
			ina228_current_na(f, r->r_shunt_uohm,
					  INA228_ADCRANGE_40_96MV),
			r->name);
	}
}

static void test_current_decodes_realistic_rail_readings(void)
{
	uint8_t f[3];

	/* VCC_RB at the FE-5680A's ~0.65 A steady draw across R159 7 mohm.
	 * 58240 * 78 125 000 / 7000 is exactly 650 mA — no rounding to hide
	 * behind, so a scale error of even one part in 10^8 fails here. */
	fr20(f, 58240U, 0U);
	TEST_ASSERT_EQUAL_INT64(650000000,
				ina228_current_na(f, 7000U,
						  INA228_ADCRANGE_40_96MV));
	TEST_ASSERT_EQUAL_INT32(650000,
				ina228_current_ua(f, 7000U,
						  INA228_ADCRANGE_40_96MV));

	/* Reverse current decodes negative, not as a huge positive. */
	fr20(f, (uint32_t)(-58240) & 0xFFFFFU, 0U);
	TEST_ASSERT_EQUAL_INT32(-650000,
				ina228_current_ua(f, 7000U,
						  INA228_ADCRANGE_40_96MV));

	/* Wide range is 4x coarser for the same count. */
	fr20(f, 1000U, 0U);
	TEST_ASSERT_EQUAL_INT64(4U * ina228_current_na(f, 25000U,
						       INA228_ADCRANGE_40_96MV),
				ina228_current_na(f, 25000U,
						  INA228_ADCRANGE_163_84MV));

	TEST_ASSERT_EQUAL_INT64(0, ina228_current_na(f, 0U,
						     INA228_ADCRANGE_40_96MV));
}

static void test_power_energy_and_charge_lsbs(void)
{
	uint8_t f3[3];
	uint8_t f5[5];

	/* POWER is a full 24-bit unsigned value with no reserved nibble. */
	fr24(f3, 1U);
	TEST_ASSERT_EQUAL_INT64(10000,
				ina228_power_nw(f3, 25000U,
						INA228_ADCRANGE_40_96MV));
	TEST_ASSERT_EQUAL_INT64(35714,
				ina228_power_nw(f3, 7000U,
						INA228_ADCRANGE_40_96MV));
	fr24(f3, 0xFFFFFFU);
	TEST_ASSERT_EQUAL_INT64(INT64_C(167772150000),
				ina228_power_nw(f3, 25000U,
						INA228_ADCRANGE_40_96MV));
	TEST_ASSERT_EQUAL_INT64(0, ina228_power_nw(f3, 0U,
						   INA228_ADCRANGE_40_96MV));

	/* ENERGY_LSB is 16 x POWER_LSB: 160 uJ on the 25 mohm rails. */
	fr40(f5, 1U);
	TEST_ASSERT_EQUAL_INT64(160, ina228_energy_uj(f5, 25000U,
						      INA228_ADCRANGE_40_96MV));
	fr40(f5, 1000U);
	TEST_ASSERT_EQUAL_INT64(160000,
				ina228_energy_uj(f5, 25000U,
						 INA228_ADCRANGE_40_96MV));
	/*
	 * Full 40-bit accumulator on the smallest shunt — the worst case for the
	 * int64 arithmetic. 2^40-1 counts x 4e6/7000 uJ is 6.28e14 uJ; computed
	 * naively the intermediate would be 4.4e21 and overflow.
	 */
	fr40(f5, UINT64_C(0xFFFFFFFFFF));
	TEST_ASSERT_EQUAL_INT64(INT64_C(628292358728571),
				ina228_energy_uj(f5, 7000U,
						 INA228_ADCRANGE_40_96MV));
	TEST_ASSERT_EQUAL_INT64(0, ina228_energy_uj(f5, 0U,
						    INA228_ADCRANGE_40_96MV));

	/* CHARGE_LSB equals CURRENT_LSB and the register is signed. */
	fr40(f5, 1U);
	TEST_ASSERT_EQUAL_INT64(3125, ina228_charge_nc(f5, 25000U,
						       INA228_ADCRANGE_40_96MV));
	fr40(f5, UINT64_C(0xFFFFFFFFFF)); /* -1 */
	TEST_ASSERT_EQUAL_INT64(-3125, ina228_charge_nc(f5, 25000U,
							INA228_ADCRANGE_40_96MV));
	/* Most negative 40-bit charge: -2^39 counts x 3125 nC. */
	fr40(f5, UINT64_C(0x8000000000));
	TEST_ASSERT_EQUAL_INT64(INT64_C(-1717986918400000),
				ina228_charge_nc(f5, 25000U,
						 INA228_ADCRANGE_40_96MV));
	TEST_ASSERT_EQUAL_INT64(0, ina228_charge_nc(f5, 0U,
						    INA228_ADCRANGE_40_96MV));
}

static void test_derived_lsb_helpers_match_the_rail_table(void)
{
	int i;

	/*
	 * The stored current_lsb_pa / power_lsb_nw / fs_current_ua fields are
	 * for display only; conversions derive from r_shunt_uohm. They must
	 * still agree, or a reader of the telemetry page sees one number and the
	 * alarm logic uses another.
	 */
	for (i = 0; i < (int)INA228_RAIL_COUNT; i++) {
		const ina228_rail_info_t *r = ina228_rail((ina228_rail_t)i);

		TEST_ASSERT_EQUAL_UINT32_MESSAGE(
			r->current_lsb_pa,
			ina228_current_lsb_pa(r->r_shunt_uohm,
					      INA228_ADCRANGE_40_96MV),
			r->name);
		TEST_ASSERT_EQUAL_UINT32_MESSAGE(
			r->power_lsb_nw,
			ina228_power_lsb_nw(r->r_shunt_uohm,
					    INA228_ADCRANGE_40_96MV),
			r->name);
		TEST_ASSERT_EQUAL_UINT32_MESSAGE(
			r->fs_current_ua,
			ina228_fs_current_ua(r->r_shunt_uohm,
					     INA228_ADCRANGE_40_96MV),
			r->name);

		/* POWER_LSB is 3.2 x CURRENT_LSB. Scale before dividing down
		 * from picoamps, or the 520.833 nA rails lose the fraction. */
		TEST_ASSERT_INT32_WITHIN_MESSAGE(
			2,
			(int32_t)(((uint64_t)r->current_lsb_pa * 32U) / 10000U),
			(int32_t)r->power_lsb_nw, r->name);
	}

	TEST_ASSERT_EQUAL_UINT32(0U, ina228_current_lsb_pa(
					     0U, INA228_ADCRANGE_40_96MV));
	TEST_ASSERT_EQUAL_UINT32(0U, ina228_power_lsb_nw(
					     0U, INA228_ADCRANGE_40_96MV));
	TEST_ASSERT_EQUAL_UINT32(0U, ina228_fs_current_ua(
					     0U, INA228_ADCRANGE_40_96MV));

	/* Wide range quadruples every derived scale. */
	TEST_ASSERT_EQUAL_UINT32(
		4U * 3125000U,
		ina228_current_lsb_pa(25000U, INA228_ADCRANGE_163_84MV));
	TEST_ASSERT_EQUAL_UINT32(
		4U * 10000U,
		ina228_power_lsb_nw(25000U, INA228_ADCRANGE_163_84MV));
	TEST_ASSERT_EQUAL_UINT32(
		4U * 1638400U,
		ina228_fs_current_ua(25000U, INA228_ADCRANGE_163_84MV));
}

static void test_full_scale_currents_match_the_documented_table(void)
{
	static const uint32_t doc_fs_ua[INA228_RAIL_COUNT] = {
		1638400U, /* +-1.6384 A */
		409600U,  /* +-0.4096 A */
		409600U,
		819200U,  /* +-0.8192 A */
		273067U,  /* +-0.2731 A */
		1638400U,
		5851429U, /* +-5.8514 A */
		546133U,  /* +-0.5461 A */
		273067U,
	};
	int i;

	for (i = 0; i < (int)INA228_RAIL_COUNT; i++) {
		TEST_ASSERT_EQUAL_UINT32_MESSAGE(
			doc_fs_ua[i], ina228_rail_tbl[i].fs_current_ua,
			ina228_rail_tbl[i].name);
	}
}

/* ---------------------------------------------------------------- DIAG_ALRT */

static void test_diag_bit_positions(void)
{
	TEST_ASSERT_EQUAL_HEX16(0x8000, INA228_DIAG_ALATCH);
	TEST_ASSERT_EQUAL_HEX16(0x4000, INA228_DIAG_CNVR);
	TEST_ASSERT_EQUAL_HEX16(0x2000, INA228_DIAG_SLOWALERT);
	TEST_ASSERT_EQUAL_HEX16(0x1000, INA228_DIAG_APOL);
	TEST_ASSERT_EQUAL_HEX16(0x0800, INA228_DIAG_ENERGYOF);
	TEST_ASSERT_EQUAL_HEX16(0x0400, INA228_DIAG_CHARGEOF);
	TEST_ASSERT_EQUAL_HEX16(0x0200, INA228_DIAG_MATHOF);
	TEST_ASSERT_EQUAL_HEX16(0x0080, INA228_DIAG_TMPOL);
	TEST_ASSERT_EQUAL_HEX16(0x0040, INA228_DIAG_SHNTOL);
	TEST_ASSERT_EQUAL_HEX16(0x0020, INA228_DIAG_SHNTUL);
	TEST_ASSERT_EQUAL_HEX16(0x0010, INA228_DIAG_BUSOL);
	TEST_ASSERT_EQUAL_HEX16(0x0008, INA228_DIAG_BUSUL);
	TEST_ASSERT_EQUAL_HEX16(0x0004, INA228_DIAG_POL);
	TEST_ASSERT_EQUAL_HEX16(0x0002, INA228_DIAG_CNVRF);
	TEST_ASSERT_EQUAL_HEX16(0x0001, INA228_DIAG_MEMSTAT);

	/* Bit 8 is reserved and must never appear in a known mask. */
	TEST_ASSERT_EQUAL_HEX16(0x0000, INA228_DIAG_KNOWN_MASK & 0x0100U);
	TEST_ASSERT_EQUAL_HEX16(0xFEFF, INA228_DIAG_KNOWN_MASK);
}

static void test_diag_round_trips_and_drops_the_reserved_bit(void)
{
	ina228_diag_t d;
	uint16_t raw;

	/* Every defined bit set at once. */
	TEST_ASSERT_EQUAL_INT(0, ina228_diag_decode(0xFFFFU, &d));
	TEST_ASSERT_TRUE(d.alatch);
	TEST_ASSERT_TRUE(d.cnvr);
	TEST_ASSERT_TRUE(d.slowalert);
	TEST_ASSERT_TRUE(d.apol);
	TEST_ASSERT_TRUE(d.energyof);
	TEST_ASSERT_TRUE(d.chargeof);
	TEST_ASSERT_TRUE(d.mathof);
	TEST_ASSERT_TRUE(d.tmpol);
	TEST_ASSERT_TRUE(d.shntol);
	TEST_ASSERT_TRUE(d.shntul);
	TEST_ASSERT_TRUE(d.busol);
	TEST_ASSERT_TRUE(d.busul);
	TEST_ASSERT_TRUE(d.pol);
	TEST_ASSERT_TRUE(d.cnvrf);
	TEST_ASSERT_TRUE(d.memstat);

	/* Re-encoding drops only reserved bit 8. */
	raw = ina228_diag_encode(&d);
	TEST_ASSERT_EQUAL_HEX16(INA228_DIAG_KNOWN_MASK, raw);

	TEST_ASSERT_EQUAL_INT(0, ina228_diag_decode(0x0000U, &d));
	TEST_ASSERT_FALSE(d.alatch);
	TEST_ASSERT_FALSE(d.memstat);
	TEST_ASSERT_EQUAL_HEX16(0x0000, ina228_diag_encode(&d));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ina228_diag_decode(0U, NULL));
	TEST_ASSERT_EQUAL_HEX16(0x0000, ina228_diag_encode(NULL));
}

static void test_diag_encode_isolates_each_bit(void)
{
	static const uint16_t bits[] = {
		INA228_DIAG_ALATCH,   INA228_DIAG_CNVR,     INA228_DIAG_SLOWALERT,
		INA228_DIAG_APOL,     INA228_DIAG_ENERGYOF, INA228_DIAG_CHARGEOF,
		INA228_DIAG_MATHOF,   INA228_DIAG_TMPOL,    INA228_DIAG_SHNTOL,
		INA228_DIAG_SHNTUL,   INA228_DIAG_BUSOL,    INA228_DIAG_BUSUL,
		INA228_DIAG_POL,      INA228_DIAG_CNVRF,    INA228_DIAG_MEMSTAT,
	};
	ina228_diag_t d;
	size_t i;

	/* Decoding one bit and re-encoding must reproduce exactly that bit —
	 * catches a field wired to the wrong struct member. */
	for (i = 0U; i < ARRAY_LEN(bits); i++) {
		TEST_ASSERT_EQUAL_INT(0, ina228_diag_decode(bits[i], &d));
		TEST_ASSERT_EQUAL_HEX16(bits[i], ina228_diag_encode(&d));
	}
}

static void test_memstat_polarity_is_inverted(void)
{
	/*
	 * MEMSTAT reads 1 when the trim-memory checksum passes. A healthy part
	 * with the reset-default DIAG_ALRT must not look faulted, and a part
	 * reporting 0 must.
	 */
	TEST_ASSERT_FALSE(ina228_diag_is_fault(INA228_DIAG_MEMSTAT));
	TEST_ASSERT_EQUAL_HEX16(0x0000,
				ina228_diag_fault_mask(INA228_DIAG_MEMSTAT));

	TEST_ASSERT_TRUE(ina228_diag_is_fault(0x0000U));
	TEST_ASSERT_EQUAL_HEX16(INA228_DIAG_MEMSTAT,
				ina228_diag_fault_mask(0x0000U));
	TEST_ASSERT_EQUAL_INT(INA228_CAUSE_MEM_ERROR,
			      ina228_diag_cause(0x0000U));

	/* Configuration bits alone are not a fault. */
	TEST_ASSERT_FALSE(ina228_diag_is_fault(
		(uint16_t)(INA228_DIAG_CONFIG_MASK | INA228_DIAG_MEMSTAT)));

	/* Nor is a conversion-ready flag. */
	TEST_ASSERT_FALSE(ina228_diag_is_fault(
		(uint16_t)(INA228_DIAG_CNVRF | INA228_DIAG_MEMSTAT)));
	TEST_ASSERT_EQUAL_INT(INA228_CAUSE_CONV_READY,
			      ina228_diag_cause((uint16_t)(INA228_DIAG_CNVRF |
							   INA228_DIAG_MEMSTAT)));
}

static void test_diag_cause_is_a_strict_priority_encoder(void)
{
	/* Highest severity first; each entry also asserts the one below it. */
	static const struct {
		uint16_t raw;
		ina228_cause_t cause;
	} v[] = {
		{0x0000U, INA228_CAUSE_MEM_ERROR}, /* MEMSTAT low beats all */
		{INA228_DIAG_MEMSTAT, INA228_CAUSE_NONE},
		{INA228_DIAG_MEMSTAT | INA228_DIAG_SHNTOL,
		 INA228_CAUSE_SHUNT_OVER},
		{INA228_DIAG_MEMSTAT | INA228_DIAG_BUSOL, INA228_CAUSE_BUS_OVER},
		{INA228_DIAG_MEMSTAT | INA228_DIAG_TMPOL, INA228_CAUSE_TEMP_OVER},
		{INA228_DIAG_MEMSTAT | INA228_DIAG_POL, INA228_CAUSE_POWER_OVER},
		{INA228_DIAG_MEMSTAT | INA228_DIAG_BUSUL, INA228_CAUSE_BUS_UNDER},
		{INA228_DIAG_MEMSTAT | INA228_DIAG_SHNTUL,
		 INA228_CAUSE_SHUNT_UNDER},
		{INA228_DIAG_MEMSTAT | INA228_DIAG_MATHOF, INA228_CAUSE_MATH_OVF},
		{INA228_DIAG_MEMSTAT | INA228_DIAG_CHARGEOF,
		 INA228_CAUSE_CHARGE_OVF},
		{INA228_DIAG_MEMSTAT | INA228_DIAG_ENERGYOF,
		 INA228_CAUSE_ENERGY_OVF},
		{INA228_DIAG_MEMSTAT | INA228_DIAG_CNVRF,
		 INA228_CAUSE_CONV_READY},
	};
	size_t i;

	for (i = 0U; i < ARRAY_LEN(v); i++) {
		TEST_ASSERT_EQUAL_INT_MESSAGE(v[i].cause,
					      ina228_diag_cause(v[i].raw),
					      ina228_cause_str(v[i].cause));
	}

	/* With everything asserted the top of the ladder wins. */
	TEST_ASSERT_EQUAL_INT(INA228_CAUSE_MEM_ERROR,
			      ina228_diag_cause(0xFFFEU));

	/* Over-current outranks the under-current and accumulator flags. */
	TEST_ASSERT_EQUAL_INT(
		INA228_CAUSE_SHUNT_OVER,
		ina228_diag_cause((uint16_t)(INA228_DIAG_MEMSTAT |
					     INA228_DIAG_SHNTOL |
					     INA228_DIAG_SHNTUL |
					     INA228_DIAG_ENERGYOF |
					     INA228_DIAG_CNVRF)));
}

static void test_cause_names_are_present_and_bounded(void)
{
	int i;

	for (i = 0; i < (int)INA228_CAUSE_COUNT; i++) {
		const char *s = ina228_cause_str((ina228_cause_t)i);

		TEST_ASSERT_NOT_NULL(s);
		TEST_ASSERT_TRUE(s[0] != '\0');
	}

	TEST_ASSERT_EQUAL_STRING("none", ina228_cause_str(INA228_CAUSE_NONE));
	TEST_ASSERT_EQUAL_STRING("shunt-over",
				 ina228_cause_str(INA228_CAUSE_SHUNT_OVER));
	TEST_ASSERT_EQUAL_STRING("invalid",
				 ina228_cause_str(INA228_CAUSE_COUNT));
	TEST_ASSERT_EQUAL_STRING("invalid", ina228_cause_str((ina228_cause_t)-1));
}

/* ---------------------------------------------------------------------- ID */

static void test_device_identification(void)
{
	TEST_ASSERT_EQUAL_HEX16(0x5449, INA228_MANUFACTURER_ID_TI); /* "TI" */
	TEST_ASSERT_EQUAL_HEX16(0x228, ina228_device_id_die(0x2281U));
	TEST_ASSERT_EQUAL_UINT8(0x1U, ina228_device_id_rev(0x2281U));

	TEST_ASSERT_TRUE(ina228_id_matches(0x5449U, 0x2281U));
	TEST_ASSERT_TRUE(ina228_id_matches(0x5449U, 0x2280U)); /* rev ignored */
	TEST_ASSERT_FALSE(ina228_id_matches(0x5448U, 0x2281U));
	TEST_ASSERT_FALSE(ina228_id_matches(0x5449U, 0x2370U)); /* INA237 die */
	TEST_ASSERT_FALSE(ina228_id_matches(0x0000U, 0x0000U));
}

/* -------------------------------------------------------------- rail table */

static void test_rail_table_addresses_are_the_as_built_map(void)
{
	static const uint8_t addr[INA228_RAIL_COUNT] = {
		0x40U, 0x41U, 0x42U, 0x43U, 0x45U, 0x46U, 0x47U, 0x4AU, 0x4CU,
	};
	int i;
	int j;

	for (i = 0; i < (int)INA228_RAIL_COUNT; i++) {
		TEST_ASSERT_EQUAL_HEX8_MESSAGE(addr[i],
					       ina228_rail_tbl[i].addr,
					       ina228_rail_tbl[i].name);
		TEST_ASSERT_NOT_NULL(ina228_rail_tbl[i].name);
		TEST_ASSERT_NOT_NULL(ina228_rail_tbl[i].designator);
		TEST_ASSERT_NOT_NULL(ina228_rail_tbl[i].shunt_ref);
		TEST_ASSERT_TRUE(ina228_rail_tbl[i].r_shunt_uohm > 0U);
	}

	/* No collisions among the nine. */
	for (i = 0; i < (int)INA228_RAIL_COUNT; i++) {
		for (j = i + 1; j < (int)INA228_RAIL_COUNT; j++) {
			TEST_ASSERT_NOT_EQUAL_HEX8(ina228_rail_tbl[i].addr,
						   ina228_rail_tbl[j].addr);
		}
	}
}

static void test_gps_monitor_is_at_0x4a_because_sht45_owns_0x44(void)
{
	ina228_rail_t rail;

	/*
	 * Interface ref §4: "Do not attempt to correct U23 to 0x44 (SHT45
	 * collision)." A regression here silently addresses the humidity sensor
	 * as a current monitor.
	 */
	TEST_ASSERT_EQUAL_HEX8(0x4A,
			       ina228_rail_tbl[INA228_RAIL_3V3_GPS].addr);
	TEST_ASSERT_EQUAL_INT(-ENOENT, ina228_rail_by_addr(0x44U, &rail));

	TEST_ASSERT_EQUAL_INT(0, ina228_rail_by_addr(0x4AU, &rail));
	TEST_ASSERT_EQUAL_INT(INA228_RAIL_3V3_GPS, rail);
	TEST_ASSERT_EQUAL_STRING("U23",
				 ina228_rail_tbl[INA228_RAIL_3V3_GPS].designator);
}

static void test_rail_lookup_by_address(void)
{
	ina228_rail_t rail;
	int i;

	for (i = 0; i < (int)INA228_RAIL_COUNT; i++) {
		TEST_ASSERT_EQUAL_INT(
			0, ina228_rail_by_addr(ina228_rail_tbl[i].addr, &rail));
		TEST_ASSERT_EQUAL_INT(i, (int)rail);
	}

	/* Neighbouring I2C devices are not INA228s. */
	TEST_ASSERT_EQUAL_INT(-ENOENT, ina228_rail_by_addr(0x19U, &rail)); /* accel */
	TEST_ASSERT_EQUAL_INT(-ENOENT, ina228_rail_by_addr(0x1EU, &rail)); /* mag */
	TEST_ASSERT_EQUAL_INT(-ENOENT, ina228_rail_by_addr(0x48U, &rail)); /* TMP117 */
	TEST_ASSERT_EQUAL_INT(-ENOENT, ina228_rail_by_addr(0x49U, &rail)); /* TMP117 */
	TEST_ASSERT_EQUAL_INT(-ENOENT, ina228_rail_by_addr(0x60U, &rail)); /* ATECC */
	TEST_ASSERT_EQUAL_INT(-EINVAL, ina228_rail_by_addr(0x40U, NULL));
}

static void test_rail_accessor_bounds(void)
{
	TEST_ASSERT_NOT_NULL(ina228_rail(INA228_RAIL_POE));
	TEST_ASSERT_NOT_NULL(ina228_rail((ina228_rail_t)(INA228_RAIL_COUNT - 1)));
	TEST_ASSERT_NULL(ina228_rail(INA228_RAIL_COUNT));
	TEST_ASSERT_NULL(ina228_rail((ina228_rail_t)-1));
	TEST_ASSERT_NULL(ina228_rail((ina228_rail_t)99));
}

static void test_alert_lines_land_where_the_scan_expects_them(void)
{
	int i;
	int on_f = 0;

	/*
	 * Fault-aggregation doc §3: eight alerts on GPIOG[8:15], the panel
	 * monitor alone on PF13. The 1 kHz scan masks depend on this split.
	 */
	for (i = 0; i < (int)INA228_RAIL_COUNT; i++) {
		const ina228_rail_info_t *r = &ina228_rail_tbl[i];

		if (r->alert_port == 'F') {
			on_f++;
			TEST_ASSERT_EQUAL_INT(INA228_RAIL_PANEL_5V, i);
			TEST_ASSERT_EQUAL_UINT8(13U, r->alert_bit);
		} else {
			TEST_ASSERT_EQUAL_UINT8('G', r->alert_port);
			TEST_ASSERT_TRUE(r->alert_bit >= 8U);
			TEST_ASSERT_TRUE(r->alert_bit <= 15U);
		}
	}
	TEST_ASSERT_EQUAL_INT(1, on_f);

	/* The GPIOG alert bits are a permutation of 8..15 with no duplicates. */
	{
		uint16_t seen = 0U;

		for (i = 0; i < (int)INA228_RAIL_COUNT; i++) {
			if (ina228_rail_tbl[i].alert_port != 'G') {
				continue;
			}
			TEST_ASSERT_EQUAL_HEX16(
				0, seen & (uint16_t)(1U << ina228_rail_tbl[i]
							     .alert_bit));
			seen |= (uint16_t)(1U << ina228_rail_tbl[i].alert_bit);
		}
		TEST_ASSERT_EQUAL_HEX16(0xFF00, seen);
	}
}

static void test_shunt_sizing_stays_inside_full_scale(void)
{
	/*
	 * Root CLAUDE.md: "target 50-75 % of the INA228 ADCRANGE=1 full scale at
	 * the rail's design maximum" — a resolution-versus-insertion-drop trade,
	 * not one rule. Pinning the actual utilisation per rail turns a prose
	 * guideline into a regression guard: change a shunt or a design maximum
	 * and this table has to be revisited deliberately.
	 *
	 * Three rails sit outside the band, all for reasons the documentation
	 * states:
	 *   VCC_RB   36 % — R159 is sized for the *low-voltage* end of the
	 *                   4.51-24.45 V programmable range, where the same FE
	 *                   power draws several times the 15 V current.
	 *   3V3_GPS  23 % — R72 is sized by insertion drop; it is in series with
	 *                   an LDO feeding a receiver with a 2.7 V floor.
	 *   panel    48 % — R199 shares the 150 mohm value with R89 (V_ANT) for
	 *                   BOM commonality; a hair under the band, and one
	 *                   CURRENT LSB is still 0.4 % of a single LED.
	 */
	static const uint32_t expect_pct[INA228_RAIL_COUNT] = {
		73U, /* V_POE       1.200 A / 1.6384 A */
		73U, /* 3V3_STM     0.300 A / 0.4096 A */
		61U, /* 5V_DISP     0.250 A / 0.4096 A */
		54U, /* 3V3         0.450 A / 0.8192 A */
		66U, /* V_ANT       0.182 A / 0.2731 A */
		73U, /* OCXO        1.200 A / 1.6384 A */
		35U, /* VCC_RB      2.100 A / 5.8514 A  (low-end sizing) */
		23U, /* 3V3_GPS     0.130 A / 0.5461 A  (insertion drop) */
		48U, /* panel 5 V   0.132 A / 0.2731 A  (shared 150 mohm) */
	};
	int i;

	for (i = 0; i < (int)INA228_RAIL_COUNT; i++) {
		const ina228_rail_info_t *r = &ina228_rail_tbl[i];
		uint32_t design_ua = r->design_max_ma * 1000U;
		uint32_t pct = (design_ua * 100U) / r->fs_current_ua;

		/* Load-bearing invariant: normal operation must never clip. */
		TEST_ASSERT_TRUE_MESSAGE(design_ua < r->fs_current_ua, r->name);
		TEST_ASSERT_EQUAL_UINT32_MESSAGE(expect_pct[i], pct, r->name);
	}
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_register_widths_match_the_datasheet_map);
	RUN_TEST(test_register_addresses_are_the_documented_ones);

	RUN_TEST(test_config_default_selects_the_board_adcrange);
	RUN_TEST(test_config_field_positions);
	RUN_TEST(test_config_round_trips_every_field);

	RUN_TEST(test_adc_config_field_positions);
	RUN_TEST(test_adc_config_round_trips);
	RUN_TEST(test_avg_and_conversion_time_tables);
	RUN_TEST(test_conversion_interval_counts_only_converted_channels);
	RUN_TEST(test_default_averaging_swamps_the_panel_pwm_period);

	RUN_TEST(test_shunt_cal_default_is_4096_for_every_rail);
	RUN_TEST(test_shunt_cal_trim_scales_by_the_measurement_ratio);
	RUN_TEST(test_shunt_cal_trim_clamps_and_validates);

	RUN_TEST(test_sovl_defaults_match_the_documented_table);
	RUN_TEST(test_rail_table_sovl_defaults_are_recomputable);
	RUN_TEST(test_shunt_code_full_scale_and_sign);
	RUN_TEST(test_shunt_code_milliamp_wrapper);
	RUN_TEST(test_shunt_code_inverts_back_to_current);
	RUN_TEST(test_v_ant_suvl_detects_an_open_antenna);
	RUN_TEST(test_every_other_rail_parks_suvl_where_it_cannot_assert);
	RUN_TEST(test_bus_temp_and_power_threshold_codes);

	RUN_TEST(test_raw20_packs_into_bits_23_to_4);
	RUN_TEST(test_raw20_sign_extends_both_polarities);
	RUN_TEST(test_raw24_and_raw40_frames);
	RUN_TEST(test_raw_decoders_tolerate_a_null_frame);

	RUN_TEST(test_vshunt_scales_at_78_125_nv);
	RUN_TEST(test_vbus_reads_the_54v_and_24v_rails_with_no_divider);
	RUN_TEST(test_dietemp_scales_at_7_8125_millidegrees);
	RUN_TEST(test_one_current_lsb_matches_the_documented_table);
	RUN_TEST(test_current_decodes_realistic_rail_readings);
	RUN_TEST(test_power_energy_and_charge_lsbs);
	RUN_TEST(test_derived_lsb_helpers_match_the_rail_table);
	RUN_TEST(test_full_scale_currents_match_the_documented_table);

	RUN_TEST(test_diag_bit_positions);
	RUN_TEST(test_diag_round_trips_and_drops_the_reserved_bit);
	RUN_TEST(test_diag_encode_isolates_each_bit);
	RUN_TEST(test_memstat_polarity_is_inverted);
	RUN_TEST(test_diag_cause_is_a_strict_priority_encoder);
	RUN_TEST(test_cause_names_are_present_and_bounded);

	RUN_TEST(test_device_identification);

	RUN_TEST(test_rail_table_addresses_are_the_as_built_map);
	RUN_TEST(test_gps_monitor_is_at_0x4a_because_sht45_owns_0x44);
	RUN_TEST(test_rail_lookup_by_address);
	RUN_TEST(test_rail_accessor_bounds);
	RUN_TEST(test_alert_lines_land_where_the_scan_expects_them);
	RUN_TEST(test_shunt_sizing_stays_inside_full_scale);

	return UNITY_END();
}
