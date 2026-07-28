/*
 * STS1000 "Meridian" — the stage-8 rubidium envelope, tested away from Zephyr.
 *
 * The root CLAUDE.md is explicit about what is at stake:
 *
 *     "Rb digipot in a buck FB node can destroy the Rb: hard-bound the range
 *      with fixed series resistors so no wiper code exits the safe envelope
 *      [...] verify the rail on INA228 0x47 (U44) before trusting the Rb."
 *
 * The resistors bound the *hardware*. platform/sts_rbguard.h bounds the
 * *configuration*, and until this file existed nothing asserted that it does.
 * `pwr.rb.vmax.mv` is schema-bounded only by the MIC28516's own 4.51-24.43 V
 * range, so a config import can raise the FE ceiling to 24 V — and pwrseq's
 * measured <= vmax gate then approves a rail that destroys a 15 V-class
 * FE-5680A. Nothing about that is visible until the part is dead.
 *
 * The opposite error is nearly as bad and much easier to make: a ceiling below
 * the fixed operating setpoint makes pwrseq_init() reject the whole
 * configuration, and a sequencer that never starts costs GPS, display, watchdog
 * and relay. A typo aimed at the rubidium would brick the grandmaster.
 *
 * The rest of the file pins the input conditioning that feeds pwrseq_step():
 * the power-good decode (an inversion AND a re-index, both silent), the INA
 * timestamp-to-age conversion (which was once passed through unconverted), the
 * external-reference band, and the RB_LOCK opto polarity.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "zephyr/platform/sts_rbguard.h"

/* ---------------------------------------------------- the VCC_RB ceiling -- */

/* The number itself, and what it is: the FE's survivable maximum, not a
 * preference. A silent bump of this constant is the single most destructive
 * one-line change available in this tree. */
static void test_the_hardware_ceiling_is_the_fe_rating(void)
{
	TEST_ASSERT_EQUAL_UINT32(15000U, STS_RB_VMAX_MV_CEILING);
}

/*
 * The case the ceiling exists for. 24 V is inside the buck's range and inside
 * the schema's bounds, so nothing upstream rejects it.
 */
static void test_a_configured_ceiling_above_the_fe_rating_is_clamped(void)
{
	sts_rb_vmax_t d;

	sts_rb_vmax_decide(24000U, 12000U, 11000, &d);

	TEST_ASSERT_EQUAL_UINT32_MESSAGE(
		STS_RB_VMAX_MV_CEILING, d.vmax_mv,
		"a 24 V FE ceiling was accepted; the rubidium would be destroyed");
	TEST_ASSERT_TRUE(d.clamped);
	TEST_ASSERT_FALSE(d.refused);

	/* The top of the buck's own range, and beyond a u16 entirely. */
	sts_rb_vmax_decide(24450U, 12000U, 11000, &d);
	TEST_ASSERT_EQUAL_UINT32(STS_RB_VMAX_MV_CEILING, d.vmax_mv);
	sts_rb_vmax_decide(UINT64_C(0xFFFFFFFFFF), 12000U, 11000, &d);
	TEST_ASSERT_EQUAL_UINT32(STS_RB_VMAX_MV_CEILING, d.vmax_mv);
	TEST_ASSERT_TRUE(d.clamped);
}

/* Exactly at the ceiling is allowed and is not a clamp — the boundary belongs
 * to the safe side, and reporting a clamp here would cry wolf on every board
 * commissioned at the FE's rated maximum. */
static void test_exactly_the_ceiling_is_accepted_unclamped(void)
{
	sts_rb_vmax_t d;

	sts_rb_vmax_decide(STS_RB_VMAX_MV_CEILING, 12000U, 11000, &d);

	TEST_ASSERT_EQUAL_UINT32(STS_RB_VMAX_MV_CEILING, d.vmax_mv);
	TEST_ASSERT_FALSE(d.clamped);
	TEST_ASSERT_FALSE(d.refused);
}

/* Lowering the ceiling for a lower-voltage FE variant is the whole point of the
 * key being configurable, and must pass through untouched. */
static void test_a_lower_ceiling_is_honoured(void)
{
	sts_rb_vmax_t d;

	sts_rb_vmax_decide(9000U, 12000U, 8000, &d);

	TEST_ASSERT_EQUAL_UINT32(9000U, d.vmax_mv);
	TEST_ASSERT_FALSE(d.clamped);
	TEST_ASSERT_FALSE(d.refused);
}

/*
 * A ceiling under the fixed operating setpoint must be REFUSED, not accepted
 * and not clamped up to meet it. Accepting it would make pwrseq_init() reject
 * the configuration and the sequencer would never start; clamping up would
 * silently defeat the operator's intent to run the FE lower.
 */
static void test_a_ceiling_below_the_operating_setpoint_is_refused(void)
{
	sts_rb_vmax_t d;

	sts_rb_vmax_decide(5000U, 12000U, 11000, &d);

	TEST_ASSERT_TRUE_MESSAGE(d.refused, "a below-setpoint ceiling was accepted");
	TEST_ASSERT_EQUAL_UINT32_MESSAGE(
		12000U, d.vmax_mv,
		"a refused ceiling did not fall back to the built-in default");
	TEST_ASSERT_NOT_EQUAL_UINT32(5000U, d.vmax_mv);

	/* One millivolt under is still under. */
	sts_rb_vmax_decide(10999U, 12000U, 11000, &d);
	TEST_ASSERT_TRUE(d.refused);
	/* Exactly at the setpoint is fine — the gate is measured <= vmax. */
	sts_rb_vmax_decide(11000U, 12000U, 11000, &d);
	TEST_ASSERT_FALSE(d.refused);
	TEST_ASSERT_EQUAL_UINT32(11000U, d.vmax_mv);
}

/*
 * Clamping happens first, so a configured 24 V on a board whose operating
 * setpoint is above the hardware ceiling is refused rather than approved at
 * 15 V. Order matters: check-then-clamp would have accepted the 24 V.
 */
static void test_clamping_precedes_the_setpoint_check(void)
{
	sts_rb_vmax_t d;

	sts_rb_vmax_decide(24000U, 12000U, 20000, &d);

	TEST_ASSERT_TRUE(d.clamped);
	TEST_ASSERT_TRUE(d.refused);
	TEST_ASSERT_EQUAL_UINT32(12000U, d.vmax_mv);
}

/* An unknown setpoint (pwrseq_rb_expected_mv() returning 0 for a misconfigured
 * transfer function) must skip the comparison rather than guess. */
static void test_an_unknown_setpoint_skips_the_below_check(void)
{
	sts_rb_vmax_t d;

	sts_rb_vmax_decide(6000U, 12000U, 0, &d);
	TEST_ASSERT_FALSE(d.refused);
	TEST_ASSERT_EQUAL_UINT32(6000U, d.vmax_mv);

	sts_rb_vmax_decide(6000U, 12000U, -1, &d);
	TEST_ASSERT_FALSE(d.refused);
	TEST_ASSERT_EQUAL_UINT32(6000U, d.vmax_mv);
}

/* Whatever the inputs, the answer never exceeds the hardware ceiling. Swept
 * across the buck's whole range because the destructive direction is the one
 * that must have no gap in it. */
static void test_no_input_can_produce_a_value_above_the_ceiling(void)
{
	for (uint64_t mv = 0; mv <= 30000U; mv += 137U) {
		for (int32_t setpoint = -1000; setpoint <= 25000; setpoint += 4999) {
			sts_rb_vmax_t d;

			sts_rb_vmax_decide(mv, 12000U, setpoint, &d);
			TEST_ASSERT_LESS_OR_EQUAL_UINT32(STS_RB_VMAX_MV_CEILING,
							 d.vmax_mv);
		}
	}
}

/* ------------------------------------------------------------- PG decode --- */

/*
 * The decode is an inversion and a re-index at once. core/fault says asserted =
 * not-good; pwrseq says a set bit = good. Each of the eight rails is checked on
 * its own so a shift by one produces a failure naming the rail it moved to.
 */
static void test_pg_decode_inverts_and_reindexes_each_rail(void)
{
	static const struct {
		fault_sig_t sig;
		uint8_t bit;
		const char *name;
	} rails[] = {
		{ FAULT_SIG_PG_3V3_GPS_LDO, PWRSEQ_PG_3V3_GPS_LDO, "3V3_GPS_LDO" },
		{ FAULT_SIG_PG_OCXO_LDO, PWRSEQ_PG_OCXO_LDO, "OCXO_LDO" },
		{ FAULT_SIG_PG_3V0_RF_LDO, PWRSEQ_PG_3V0_RF_LDO, "3V0_RF_LDO" },
		{ FAULT_SIG_PG_5V_PSU, PWRSEQ_PG_5V_PSU, "5V_PSU" },
		{ FAULT_SIG_PG_3V3_PSU, PWRSEQ_PG_3V3_PSU, "3V3_PSU" },
		{ FAULT_SIG_PG_OCXO_PSU, PWRSEQ_PG_OCXO_PSU, "OCXO_PSU" },
		{ FAULT_SIG_PG_RB_PSU, PWRSEQ_PG_RB_PSU, "RB_PSU" },
		{ FAULT_SIG_PG_POE, PWRSEQ_PG_POE, "POE" },
	};

	/* Nothing asserted: every rail good. */
	TEST_ASSERT_EQUAL_HEX8(0xFFU, sts_rb_pg_mask(0U));

	for (size_t i = 0; i < sizeof(rails) / sizeof(rails[0]); i++) {
		uint8_t mask = sts_rb_pg_mask(FAULT_SIG_BIT(rails[i].sig));
		char msg[80];

		snprintf(msg, sizeof(msg), "%s down decoded to the wrong PG bit",
			 rails[i].name);
		TEST_ASSERT_EQUAL_HEX8_MESSAGE((uint8_t)(0xFFU & ~rails[i].bit),
					       mask, msg);
	}
}

/* Signals outside the eight power-good lines must not leak into the mask — a
 * button press must not read as a rail collapse. */
static void test_pg_decode_ignores_every_other_signal(void)
{
	uint32_t others = 0xFFFFFFFFU;

	for (unsigned int n = 0; n < 8U; n++) {
		others &= ~FAULT_SIG_BIT((unsigned int)FAULT_SIG_PG_3V3_GPS_LDO + n);
	}

	TEST_ASSERT_EQUAL_HEX8_MESSAGE(
		0xFFU, sts_rb_pg_mask(others),
		"a non-PG signal changed the power-good mask");
}

/* Both supercap rails required, and only those two. */
static void test_supercap_gate_needs_both_backup_rails(void)
{
	TEST_ASSERT_TRUE(sts_rb_supercaps_charged(0U));
	TEST_ASSERT_FALSE(
		sts_rb_supercaps_charged(FAULT_SIG_BIT(FAULT_SIG_BKP_STM_PG)));
	TEST_ASSERT_FALSE(
		sts_rb_supercaps_charged(FAULT_SIG_BIT(FAULT_SIG_BKP_GPS_PG)));
	TEST_ASSERT_FALSE(sts_rb_supercaps_charged(
		FAULT_SIG_BIT(FAULT_SIG_BKP_STM_PG) |
		FAULT_SIG_BIT(FAULT_SIG_BKP_GPS_PG)));
	/* An unrelated fault does not block the rubidium sequence here. */
	TEST_ASSERT_TRUE(sts_rb_supercaps_charged(FAULT_SIG_BIT(FAULT_SIG_BUTTON_1)));
}

/* --------------------------------------------------------- INA freshness -- */

/*
 * sts_ina_reading_t::age_ms is a TIMESTAMP. Returning it unconverted is the
 * defect this exists to prevent: the rubidium rail windows would judge a
 * reading up to a second older than the rail change they were watching, and a
 * fresh timestamp is numerically large, so it would look ancient and the step
 * would time out instead.
 */
static void test_ina_age_is_a_difference_not_a_timestamp(void)
{
	TEST_ASSERT_EQUAL_UINT32(250U, sts_rb_ina_age_ms(true, 10250U, 10000U));
	TEST_ASSERT_EQUAL_UINT32(0U, sts_rb_ina_age_ms(true, 10000U, 10000U));
}

/* An invalid reading is infinitely old, which every pwrseq freshness test
 * already treats as a refusal. Reporting 0 would make a rail that was never
 * read look like the freshest evidence on the board. */
static void test_an_invalid_reading_is_infinitely_old(void)
{
	TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, sts_rb_ina_age_ms(false, 10250U, 10000U));
	TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, sts_rb_ina_age_ms(false, 0U, 0U));
}

/* The uptime clock wraps every 49.7 days; a reading taken just before it must
 * not read as 49 days old and stall the Rb sequence forever. */
static void test_ina_age_survives_the_32_bit_wrap(void)
{
	TEST_ASSERT_EQUAL_UINT32(300U, sts_rb_ina_age_ms(true, 299U, UINT32_MAX));
}

/* ------------------------------------------------------------ PoE power --- */

static void test_poe_power_is_the_product_of_the_two_readings(void)
{
	/* 53.5 V at 250 mA = 13.375 W. */
	TEST_ASSERT_EQUAL_UINT32(13375U, sts_rb_poe_mw(true, 53500000, 250000));
	/* 48 V at 100 mA = 4.8 W. */
	TEST_ASSERT_EQUAL_UINT32(4800U, sts_rb_poe_mw(true, 48000000, 100000));
}

/*
 * The INA228 current register is signed and reads slightly negative at no load.
 * A power computed from it must not wrap into a huge positive draw — that would
 * trip the PoE budget gate and shed loads on a board that is drawing nothing.
 */
static void test_poe_power_refuses_non_positive_readings(void)
{
	TEST_ASSERT_EQUAL_UINT32(0U, sts_rb_poe_mw(true, 53500000, -2000));
	TEST_ASSERT_EQUAL_UINT32(0U, sts_rb_poe_mw(true, -1000, 250000));
	TEST_ASSERT_EQUAL_UINT32(0U, sts_rb_poe_mw(true, 0, 250000));
	TEST_ASSERT_EQUAL_UINT32(0U, sts_rb_poe_mw(true, 53500000, 0));
	TEST_ASSERT_EQUAL_UINT32(0U, sts_rb_poe_mw(false, 53500000, 250000));
}

/* --------------------------------------------------- external reference --- */

/*
 * +-20 Hz on 10 MHz. Both edges are inside, both one hertz outside are not.
 * A band that has drifted wide hands the system clock to a rubidium that is not
 * on frequency, which is precisely the failure the OCXO fallback exists for and
 * precisely the one that produces confidently wrong time rather than an outage.
 */
static void test_extref_band_edges(void)
{
	TEST_ASSERT_TRUE(sts_rb_extref_in_band(10000000U, true));
	TEST_ASSERT_TRUE(sts_rb_extref_in_band(9999800U, true));
	TEST_ASSERT_TRUE(sts_rb_extref_in_band(10000200U, true));

	TEST_ASSERT_FALSE(sts_rb_extref_in_band(9999799U, true));
	TEST_ASSERT_FALSE(sts_rb_extref_in_band(10000201U, true));
	TEST_ASSERT_FALSE(sts_rb_extref_in_band(9000000U, true));
	TEST_ASSERT_FALSE(sts_rb_extref_in_band(0U, true));
}

/* An untrustworthy measurement is not in band whatever the number says — a
 * gate that has not closed reports a stale count, and a stale 10 MHz is
 * indistinguishable from a live one by value alone. */
static void test_extref_requires_a_valid_measurement(void)
{
	TEST_ASSERT_FALSE_MESSAGE(sts_rb_extref_in_band(10000000U, false),
				  "an invalid EXTREF measurement was accepted");
}

/* -------------------------------------------------------------- RB_LOCK --- */

/* The opto inverts, and the polarity is a per-unit commissioning bit. Both
 * settings are pinned, because "it worked on the bench unit" is exactly how the
 * wrong one ships. */
static void test_rb_lock_polarity_is_configurable_both_ways(void)
{
	TEST_ASSERT_TRUE(sts_rb_lock_from_level(0, true));
	TEST_ASSERT_FALSE(sts_rb_lock_from_level(1, true));

	TEST_ASSERT_TRUE(sts_rb_lock_from_level(1, false));
	TEST_ASSERT_FALSE(sts_rb_lock_from_level(0, false));
}

/*
 * A pin that could not be read is NOT locked, under either polarity. Under
 * active-low the naive `level == 0` test is false for a negative errno by luck;
 * `level != 1` would have reported a read failure as a locked rubidium and let
 * the reference state machine hand it the system clock.
 */
static void test_a_read_error_is_never_locked(void)
{
	TEST_ASSERT_FALSE_MESSAGE(sts_rb_lock_from_level(-EIO, true),
				  "a GPIO read error read as RB_LOCK asserted");
	TEST_ASSERT_FALSE(sts_rb_lock_from_level(-EIO, false));
	TEST_ASSERT_FALSE(sts_rb_lock_from_level(-1, true));
	TEST_ASSERT_FALSE(sts_rb_lock_from_level(-1, false));
}

/* -------------------------------------------------------------------- main - */

/* ===================================================================== *
 *  "Can the FE answer" is BOTH pins
 *
 *  CLAUDE.md states the order: "assert RB_PWR_EN (PB7) -> soft-start ->
 *  verify VCC_RB on INA228 U44 (0x47) -> then assert RB_VCC_GATE (PB1) to
 *  connect the FE". So there is a window in which the buck is live and the
 *  rubidium is not attached, and rb_serial_rail_up() used to report that
 *  window as "up" because it read RB_PWR_EN alone.
 *
 *  Why that mattered: rb_fwupd_probe() treats rail-up silence as a verdict
 *  and latches RB_CAP_NONE — "absent, check the cable" — for the rest of
 *  the uptime, since cap returns to UNKNOWN only in rb_fwupd_init(). An
 *  unauthenticated G0 `fw.inventory` lands in that window during ordinary
 *  bring-up.
 *
 *  A previous commit fixed the RB_PWR_EN-low half and claimed the whole of
 *  it. No test could have caught the rest: the predicate lived in Zephyr
 *  glue and test_rb_fwupd.c's fixture models the ANSWER (one `rail` bool)
 *  rather than the pins. That is why the decision moved here.
 * ===================================================================== */

static void test_the_rail_is_up_only_when_both_pins_are(void)
{
	/* The whole defect in one row: buck on, FE not yet connected. */
	TEST_ASSERT_FALSE_MESSAGE(
		sts_rb_serial_rail_up(1, 0),
		"RB_PWR_EN high with RB_VCC_GATE low is the pwrseq window "
		"between steps 8.12 and 8.14 — the FE cannot answer");

	TEST_ASSERT_TRUE(sts_rb_serial_rail_up(1, 1));
	TEST_ASSERT_FALSE(sts_rb_serial_rail_up(0, 0));
	TEST_ASSERT_FALSE(sts_rb_serial_rail_up(0, 1));
}

static void test_an_unreadable_pin_is_never_up(void)
{
	/*
	 * Same reasoning as test_a_read_error_is_never_locked: concluding "the
	 * rubidium is absent" from a pin nobody could read is a verdict this
	 * code has no business reaching. gpio_pin_get_dt() returns negative on
	 * error.
	 */
	TEST_ASSERT_FALSE(sts_rb_serial_rail_up(-EIO, 1));
	TEST_ASSERT_FALSE(sts_rb_serial_rail_up(1, -EIO));
	TEST_ASSERT_FALSE(sts_rb_serial_rail_up(-EIO, -EIO));
	/* Not "non-zero means asserted" — a negative errno is not a level. */
	TEST_ASSERT_FALSE(sts_rb_serial_rail_up(-1, -1));
}

static void test_no_input_but_both_asserted_reports_up(void)
{
	/* Exhaustive over the plausible domain, so a future "clever" encoding
	 * cannot sneak a third truthy value through. */
	int probes[] = { -2, -1, 0, 1, 2, 7 };
	size_t i, j;

	for (i = 0U; i < (sizeof(probes) / sizeof(probes[0])); i++) {
		for (j = 0U; j < (sizeof(probes) / sizeof(probes[0])); j++) {
			bool want = (probes[i] == 1) && (probes[j] == 1);

			TEST_ASSERT_EQUAL_MESSAGE(
				want,
				sts_rb_serial_rail_up(probes[i], probes[j]),
				"only level 1 on both pins is 'up'");
		}
	}
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_the_hardware_ceiling_is_the_fe_rating);
	RUN_TEST(test_a_configured_ceiling_above_the_fe_rating_is_clamped);
	RUN_TEST(test_exactly_the_ceiling_is_accepted_unclamped);
	RUN_TEST(test_a_lower_ceiling_is_honoured);
	RUN_TEST(test_a_ceiling_below_the_operating_setpoint_is_refused);
	RUN_TEST(test_clamping_precedes_the_setpoint_check);
	RUN_TEST(test_an_unknown_setpoint_skips_the_below_check);
	RUN_TEST(test_no_input_can_produce_a_value_above_the_ceiling);

	RUN_TEST(test_pg_decode_inverts_and_reindexes_each_rail);
	RUN_TEST(test_pg_decode_ignores_every_other_signal);
	RUN_TEST(test_supercap_gate_needs_both_backup_rails);

	RUN_TEST(test_ina_age_is_a_difference_not_a_timestamp);
	RUN_TEST(test_an_invalid_reading_is_infinitely_old);
	RUN_TEST(test_ina_age_survives_the_32_bit_wrap);

	RUN_TEST(test_poe_power_is_the_product_of_the_two_readings);
	RUN_TEST(test_poe_power_refuses_non_positive_readings);

	RUN_TEST(test_extref_band_edges);
	RUN_TEST(test_extref_requires_a_valid_measurement);

	RUN_TEST(test_rb_lock_polarity_is_configurable_both_ways);
	RUN_TEST(test_a_read_error_is_never_locked);

	RUN_TEST(test_the_rail_is_up_only_when_both_pins_are);
	RUN_TEST(test_an_unreadable_pin_is_never_up);
	RUN_TEST(test_no_input_but_both_asserted_reports_up);

	return UNITY_END();
}
