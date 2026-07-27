/*
 * STS1000 "Meridian" — the 1 kHz GPIOF/GPIOG scan's decisions, tested away
 * from Zephyr.
 *
 * platform/sts_io_policy.h holds everything io_scan.c decides once core/fault
 * has debounced a signal: which physical pin a signal number names, which of
 * the nine INA228 monitors owns an ALERT line, what each debounced event turns
 * into, and when a dropped-event burst is worth annunciating.
 *
 * Every defect here is silent. An off-by-one in the port split re-reads the
 * wrong monitor's DIAG_ALRT while the real fault goes unread; an inverted
 * severity or verb turns a recovery into what reads as a second failure; a
 * mis-latched drop counter fills the log ring with warnings about nothing. None
 * of it fails, crashes or shows up in a build.
 *
 * The vectors are written against docs/sts1000_firmware_hardware_interface.md
 * §5 and the INA228 address map in the root CLAUDE.md — the *documents*, not a
 * round-trip of the code. Where the two could disagree (GPS at 0x4A, the panel
 * monitor's alert being the one line on GPIOF) the document wins and the test
 * says so by address.
 */

#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "zephyr/platform/sts_io_policy.h"

/* ------------------------------------------------------- port / bit split -- */

/*
 * fault.h's contract: GPIOF occupies signals 0..15, GPIOG 16..31. Spot-checked
 * at both ends of both ports plus the two boundary signals, because an
 * off-by-one only ever shows at a boundary.
 */
static void test_sig_port_bit_boundaries(void)
{
	uint8_t port = 0;
	uint8_t bit = 0xFF;

	TEST_ASSERT_EQUAL_INT(0, sts_io_sig_port_bit(FAULT_SIG_BUTTON_1, &port, &bit));
	TEST_ASSERT_EQUAL_UINT8('F', port);
	TEST_ASSERT_EQUAL_UINT8(0U, bit);

	TEST_ASSERT_EQUAL_INT(0, sts_io_sig_port_bit(FAULT_SIG_BKP_GPS_PG, &port, &bit));
	TEST_ASSERT_EQUAL_UINT8('F', port);
	TEST_ASSERT_EQUAL_UINT8(15U, bit);

	/* The very next signal must cross to GPIOG bit 0, not stay on F. */
	TEST_ASSERT_EQUAL_INT(
		0, sts_io_sig_port_bit(FAULT_SIG_PG_3V3_GPS_LDO, &port, &bit));
	TEST_ASSERT_EQUAL_UINT8('G', port);
	TEST_ASSERT_EQUAL_UINT8(0U, bit);

	TEST_ASSERT_EQUAL_INT(
		0, sts_io_sig_port_bit(FAULT_SIG_INA_ALERT_VCC_RB, &port, &bit));
	TEST_ASSERT_EQUAL_UINT8('G', port);
	TEST_ASSERT_EQUAL_UINT8(15U, bit);

	/* Documented pins, straight from interface ref §5: PF13 is the one INA
	 * alert on GPIOF, PG8 is the first one on GPIOG. */
	TEST_ASSERT_EQUAL_INT(
		0, sts_io_sig_port_bit(FAULT_SIG_INA_ALERT_PANEL, &port, &bit));
	TEST_ASSERT_EQUAL_UINT8('F', port);
	TEST_ASSERT_EQUAL_UINT8(13U, bit);

	TEST_ASSERT_EQUAL_INT(
		0, sts_io_sig_port_bit(FAULT_SIG_INA_ALERT_V_POE, &port, &bit));
	TEST_ASSERT_EQUAL_UINT8('G', port);
	TEST_ASSERT_EQUAL_UINT8(8U, bit);
}

/* The split must be a bijection onto (F,0..15) u (G,0..15): every signal lands
 * somewhere, and no two land on the same pin. A duplicate is exactly how a
 * mask slip misattributes a rail. */
static void test_sig_port_bit_is_a_bijection(void)
{
	bool seen_f[16] = { false };
	bool seen_g[16] = { false };

	for (unsigned int s = 0; s < (unsigned int)FAULT_SIG_COUNT; s++) {
		uint8_t port = 0;
		uint8_t bit = 0;

		TEST_ASSERT_EQUAL_INT(0,
				      sts_io_sig_port_bit((fault_sig_t)s, &port, &bit));
		TEST_ASSERT_LESS_THAN_UINT8(16U, bit);

		if (port == 'F') {
			TEST_ASSERT_FALSE_MESSAGE(seen_f[bit], "duplicate GPIOF bit");
			seen_f[bit] = true;
		} else {
			TEST_ASSERT_EQUAL_UINT8('G', port);
			TEST_ASSERT_FALSE_MESSAGE(seen_g[bit], "duplicate GPIOG bit");
			seen_g[bit] = true;
		}
	}

	for (unsigned int i = 0; i < 16U; i++) {
		TEST_ASSERT_TRUE_MESSAGE(seen_f[i], "GPIOF bit unclaimed");
		TEST_ASSERT_TRUE_MESSAGE(seen_g[i], "GPIOG bit unclaimed");
	}
}

static void test_sig_port_bit_rejects_bad_input(void)
{
	uint8_t port = 0;
	uint8_t bit = 0;

	TEST_ASSERT_EQUAL_INT(
		-EINVAL, sts_io_sig_port_bit((fault_sig_t)FAULT_SIG_COUNT, &port, &bit));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      sts_io_sig_port_bit(FAULT_SIG_BUTTON_1, NULL, &bit));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      sts_io_sig_port_bit(FAULT_SIG_BUTTON_1, &port, NULL));
}

/* ------------------------------------------------------- ALERT -> monitor -- */

/*
 * Named by I2C address, not by rail index. The addresses are the fact the root
 * CLAUDE.md fixes ("GPS INA228 is 0x4A, not 0x44"); the enum order is an
 * implementation detail that a refactor could legitimately change, and a test
 * written against it would keep passing through the very mistake it exists to
 * catch.
 */
static const struct {
	fault_sig_t sig;
	uint8_t addr;
	const char *designator;
} alert_map[] = {
	{ FAULT_SIG_INA_ALERT_PANEL, 0x4CU, "U54" },   /* PF13 — panel-LED 5 V */
	{ FAULT_SIG_INA_ALERT_V_POE, 0x40U, "U10" },   /* PG8  — PoE input */
	{ FAULT_SIG_INA_ALERT_3V3_STM, 0x41U, "U31" }, /* PG9 */
	{ FAULT_SIG_INA_ALERT_5V_DISP, 0x42U, "U32" }, /* PG10 */
	{ FAULT_SIG_INA_ALERT_3V3, 0x43U, "U30" },     /* PG11 */
	{ FAULT_SIG_INA_ALERT_3V3_GPS, 0x4AU, "U23" }, /* PG12 — NOT 0x44 */
	{ FAULT_SIG_INA_ALERT_V_ANT, 0x45U, "U26" },   /* PG13 */
	{ FAULT_SIG_INA_ALERT_OCXO, 0x46U, "U37" },    /* PG14 */
	{ FAULT_SIG_INA_ALERT_VCC_RB, 0x47U, "U44" },  /* PG15 */
};

static void test_alert_signal_resolves_to_the_documented_monitor(void)
{
	for (size_t i = 0; i < sizeof(alert_map) / sizeof(alert_map[0]); i++) {
		ina228_rail_t rail = INA228_RAIL_COUNT;
		char msg[64];

		snprintf(msg, sizeof(msg), "%s alert -> wrong monitor",
			 alert_map[i].designator);

		TEST_ASSERT_EQUAL_INT(
			0, sts_io_ina_rail_for_sig(alert_map[i].sig, &rail));
		TEST_ASSERT_LESS_THAN_UINT32((uint32_t)INA228_RAIL_COUNT,
					     (uint32_t)rail);
		TEST_ASSERT_EQUAL_UINT8_MESSAGE(alert_map[i].addr,
						ina228_rail_tbl[rail].addr, msg);
	}
}

/* All nine monitors must be reachable, and each from exactly one signal. A
 * monitor no signal maps to is a rail whose alert is never diagnosed. */
static void test_every_monitor_has_exactly_one_alert_signal(void)
{
	unsigned int hits[INA228_RAIL_COUNT] = { 0 };

	for (unsigned int s = 0; s < (unsigned int)FAULT_SIG_COUNT; s++) {
		ina228_rail_t rail;

		if (sts_io_ina_rail_for_sig((fault_sig_t)s, &rail) == 0) {
			hits[rail]++;
		}
	}

	for (size_t r = 0; r < (size_t)INA228_RAIL_COUNT; r++) {
		char msg[80];

		snprintf(msg, sizeof(msg), "%s (0x%02x): %u alert signals",
			 ina228_rail_tbl[r].designator, ina228_rail_tbl[r].addr,
			 hits[r]);
		TEST_ASSERT_EQUAL_UINT_MESSAGE(1U, hits[r], msg);
	}
}

/* A button is not an alert. Returning a rail for one would make io_scan ask
 * housekeeping to read a monitor because somebody pressed a key. */
static void test_non_alert_signals_resolve_to_nothing(void)
{
	static const fault_sig_t not_alerts[] = {
		FAULT_SIG_BUTTON_1,      FAULT_SIG_BUTTON_7,
		FAULT_SIG_TOUCH_INT,     FAULT_SIG_V_ANT_EN_FAULT,
		FAULT_SIG_PROX_WAKE,     FAULT_SIG_ENC_BUTTON,
		FAULT_SIG_PANEL_LED_FAULT, FAULT_SIG_BKP_STM_PG,
		FAULT_SIG_BKP_GPS_PG,    FAULT_SIG_PG_3V3_GPS_LDO,
		FAULT_SIG_PG_POE,
	};

	for (size_t i = 0; i < sizeof(not_alerts) / sizeof(not_alerts[0]); i++) {
		ina228_rail_t rail = INA228_RAIL_COUNT;

		TEST_ASSERT_EQUAL_INT(-ENOENT,
				      sts_io_ina_rail_for_sig(not_alerts[i], &rail));
	}

	TEST_ASSERT_EQUAL_INT(
		-EINVAL,
		sts_io_ina_rail_for_sig(FAULT_SIG_INA_ALERT_OCXO, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      sts_io_ina_rail_for_sig((fault_sig_t)99, NULL));
}

/* ------------------------------------------------------------- dispatch ---- */

static sts_io_dispatch_t plan_for(fault_evt_type_t type, fault_sig_t id,
				  fault_edge_t edge)
{
	sts_io_dispatch_t p;

	memset(&p, 0xA5, sizeof(p));
	sts_io_dispatch_plan((uint8_t)type, (uint8_t)id, (uint8_t)edge, &p);

	return p;
}

static void test_button_press_and_release_carry_the_edge(void)
{
	sts_io_dispatch_t p =
		plan_for(FAULT_EVT_BUTTON, FAULT_SIG_BUTTON_3, FAULT_EDGE_ASSERT);

	TEST_ASSERT_TRUE(p.post_ui);
	TEST_ASSERT_EQUAL_INT8((int8_t)STS_IO_UI_BUTTON, p.ui_type);
	TEST_ASSERT_EQUAL_INT16(1, p.ui_value);
	TEST_ASSERT_FALSE(p.log);
	TEST_ASSERT_FALSE(p.ina_reread);

	p = plan_for(FAULT_EVT_BUTTON, FAULT_SIG_BUTTON_3, FAULT_EDGE_DEASSERT);
	TEST_ASSERT_TRUE(p.post_ui);
	TEST_ASSERT_EQUAL_INT8((int8_t)STS_IO_UI_BUTTON, p.ui_type);
	TEST_ASSERT_EQUAL_INT16(0, p.ui_value);
}

/* Long-press and repeat exist only in the held direction; both must report
 * value 1 regardless of the edge core/fault happens to stamp them with. */
static void test_long_and_repeat_are_always_asserting(void)
{
	sts_io_dispatch_t p = plan_for(FAULT_EVT_BUTTON_LONG, FAULT_SIG_BUTTON_1,
				       FAULT_EDGE_ASSERT);

	TEST_ASSERT_TRUE(p.post_ui);
	TEST_ASSERT_EQUAL_INT8((int8_t)STS_IO_UI_BUTTON_LONG, p.ui_type);
	TEST_ASSERT_EQUAL_INT16(1, p.ui_value);

	p = plan_for(FAULT_EVT_BUTTON_REPEAT, FAULT_SIG_ENC_BUTTON,
		     FAULT_EDGE_ASSERT);
	TEST_ASSERT_TRUE(p.post_ui);
	TEST_ASSERT_EQUAL_INT8((int8_t)STS_IO_UI_BUTTON_REPEAT, p.ui_type);
	TEST_ASSERT_EQUAL_INT16(1, p.ui_value);
}

/* The FT6336 INT deasserting means the UI drained the FIFO; posting that would
 * re-enter the touch path with nothing to read. */
static void test_touch_release_posts_nothing(void)
{
	sts_io_dispatch_t p =
		plan_for(FAULT_EVT_TOUCH, FAULT_SIG_TOUCH_INT, FAULT_EDGE_ASSERT);

	TEST_ASSERT_TRUE(p.post_ui);
	TEST_ASSERT_EQUAL_INT8((int8_t)STS_IO_UI_TOUCH, p.ui_type);
	TEST_ASSERT_EQUAL_INT16(1, p.ui_value);

	p = plan_for(FAULT_EVT_TOUCH, FAULT_SIG_TOUCH_INT, FAULT_EDGE_DEASSERT);
	TEST_ASSERT_FALSE(p.post_ui);
	TEST_ASSERT_FALSE(p.log);
	TEST_ASSERT_FALSE(p.ina_reread);
}

/* The reed switch reports both directions — the UI wakes on magnet-present and
 * re-arms the screen blank on magnet-gone. */
static void test_prox_reports_both_directions(void)
{
	sts_io_dispatch_t p =
		plan_for(FAULT_EVT_PROX, FAULT_SIG_PROX_WAKE, FAULT_EDGE_ASSERT);

	TEST_ASSERT_TRUE(p.post_ui);
	TEST_ASSERT_EQUAL_INT8((int8_t)STS_IO_UI_PROX, p.ui_type);
	TEST_ASSERT_EQUAL_INT16(1, p.ui_value);

	p = plan_for(FAULT_EVT_PROX, FAULT_SIG_PROX_WAKE, FAULT_EDGE_DEASSERT);
	TEST_ASSERT_TRUE(p.post_ui);
	TEST_ASSERT_EQUAL_INT16(0, p.ui_value);
}

/*
 * The alert path is the one that reaches I2C. It must name the right monitor,
 * and it must do nothing at all on the release edge — an ALERT going away is
 * the fault clearing, and re-reading DIAG_ALRT then would clear the latched
 * cause before anyone had seen it.
 */
static void test_ina_alert_requests_the_right_monitor_on_assert_only(void)
{
	sts_io_dispatch_t p = plan_for(FAULT_EVT_INA_ALERT,
				       FAULT_SIG_INA_ALERT_VCC_RB, FAULT_EDGE_ASSERT);

	TEST_ASSERT_TRUE(p.ina_reread);
	TEST_ASSERT_EQUAL_UINT8(0x47U, ina228_rail_tbl[p.ina_rail].addr);
	TEST_ASSERT_TRUE(p.log);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)LOGR_SUB_PWR, p.log_sub);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)LOGR_WARN, p.log_level);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_IO_MSG_INA_ALERT, p.msg);
	TEST_ASSERT_FALSE(p.post_ui);

	/* The GPS monitor is the one whose address is easy to get wrong. */
	p = plan_for(FAULT_EVT_INA_ALERT, FAULT_SIG_INA_ALERT_3V3_GPS,
		     FAULT_EDGE_ASSERT);
	TEST_ASSERT_TRUE(p.ina_reread);
	TEST_ASSERT_EQUAL_UINT8(0x4AU, ina228_rail_tbl[p.ina_rail].addr);

	/* And the one alert that lives on GPIOF. */
	p = plan_for(FAULT_EVT_INA_ALERT, FAULT_SIG_INA_ALERT_PANEL,
		     FAULT_EDGE_ASSERT);
	TEST_ASSERT_TRUE(p.ina_reread);
	TEST_ASSERT_EQUAL_UINT8(0x4CU, ina228_rail_tbl[p.ina_rail].addr);

	p = plan_for(FAULT_EVT_INA_ALERT, FAULT_SIG_INA_ALERT_VCC_RB,
		     FAULT_EDGE_DEASSERT);
	TEST_ASSERT_FALSE(p.ina_reread);
	TEST_ASSERT_FALSE(p.log);
}

/*
 * Severity and wording are pinned together, in both directions, for all three
 * paired events. This is the assertion the file exists for: each pair is one
 * boolean away from reading as its own opposite, and nothing at runtime would
 * ever say so.
 */
static void test_paired_events_keep_severity_and_wording_together(void)
{
	sts_io_dispatch_t p;

	p = plan_for(FAULT_EVT_PG_FAULT, FAULT_SIG_PG_OCXO_LDO, FAULT_EDGE_ASSERT);
	TEST_ASSERT_TRUE(p.log);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)LOGR_ERR, p.log_level);
	TEST_ASSERT_EQUAL_STRING("power-good lost: %s", sts_io_msg_fmt(p.msg));

	p = plan_for(FAULT_EVT_PG_RECOVER, FAULT_SIG_PG_OCXO_LDO,
		     FAULT_EDGE_DEASSERT);
	TEST_ASSERT_TRUE(p.log);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)LOGR_NOTICE, p.log_level);
	TEST_ASSERT_EQUAL_STRING("power-good restored: %s", sts_io_msg_fmt(p.msg));

	p = plan_for(FAULT_EVT_EN_FAULT, FAULT_SIG_V_DISP_EN_FAULT,
		     FAULT_EDGE_ASSERT);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)LOGR_ERR, p.log_level);
	TEST_ASSERT_EQUAL_STRING("load switch %s faulted", sts_io_msg_fmt(p.msg));

	p = plan_for(FAULT_EVT_EN_FAULT, FAULT_SIG_V_DISP_EN_FAULT,
		     FAULT_EDGE_DEASSERT);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)LOGR_NOTICE, p.log_level);
	TEST_ASSERT_EQUAL_STRING("load switch %s recovered", sts_io_msg_fmt(p.msg));

	/* The backup rails read high when good, and core/fault normalises low to
	 * "asserted" — so the ASSERT edge is the *bad* news. Getting this
	 * backwards points a field diagnosis at the wrong end of the board. */
	p = plan_for(FAULT_EVT_BKP_PG, FAULT_SIG_BKP_GPS_PG, FAULT_EDGE_ASSERT);
	TEST_ASSERT_EQUAL_STRING("backup supply %s not good", sts_io_msg_fmt(p.msg));

	p = plan_for(FAULT_EVT_BKP_PG, FAULT_SIG_BKP_GPS_PG, FAULT_EDGE_DEASSERT);
	TEST_ASSERT_EQUAL_STRING("backup supply %s good", sts_io_msg_fmt(p.msg));
}

/* Every event this dispatcher logs about power belongs to the PWR subsystem —
 * that is what makes `log tail --sub pwr` a usable diagnostic. */
static void test_power_events_all_land_in_the_pwr_subsystem(void)
{
	static const fault_evt_type_t pwr_evts[] = {
		FAULT_EVT_INA_ALERT, FAULT_EVT_PG_FAULT, FAULT_EVT_PG_RECOVER,
		FAULT_EVT_EN_FAULT,  FAULT_EVT_BKP_PG,
	};

	for (size_t i = 0; i < sizeof(pwr_evts) / sizeof(pwr_evts[0]); i++) {
		sts_io_dispatch_t p = plan_for(pwr_evts[i],
					       FAULT_SIG_INA_ALERT_OCXO,
					       FAULT_EDGE_ASSERT);

		TEST_ASSERT_TRUE(p.log);
		TEST_ASSERT_EQUAL_UINT8((uint8_t)LOGR_SUB_PWR, p.log_sub);
	}
}

/* An unknown event type must produce a plan that does nothing, not a stale one
 * left over from whatever was in the caller's stack slot. */
static void test_unknown_event_is_fully_zeroed(void)
{
	sts_io_dispatch_t p = plan_for((fault_evt_type_t)FAULT_EVT_TYPE_COUNT,
				       FAULT_SIG_BUTTON_1, FAULT_EDGE_ASSERT);

	TEST_ASSERT_FALSE(p.post_ui);
	TEST_ASSERT_FALSE(p.log);
	TEST_ASSERT_FALSE(p.ina_reread);
	TEST_ASSERT_EQUAL_INT8((int8_t)STS_IO_UI_NONE, p.ui_type);
	TEST_ASSERT_EQUAL_STRING("", sts_io_msg_fmt(p.msg));
}

/* Every message template takes exactly one %s — io_scan.c passes exactly one
 * argument to all of them from a single call site. */
static void test_every_message_takes_one_string_argument(void)
{
	for (unsigned int m = 0; m < (unsigned int)STS_IO_MSG__COUNT; m++) {
		const char *fmt = sts_io_msg_fmt((uint8_t)m);
		unsigned int pct = 0;
		unsigned int s = 0;

		TEST_ASSERT_NOT_NULL(fmt);

		for (const char *c = fmt; *c != '\0'; c++) {
			if (*c == '%') {
				pct++;
				if (*(c + 1) == 's') {
					s++;
				}
			}
		}

		if (m == (unsigned int)STS_IO_MSG_NONE) {
			TEST_ASSERT_EQUAL_UINT(0U, pct);
		} else {
			TEST_ASSERT_EQUAL_UINT(1U, pct);
			TEST_ASSERT_EQUAL_UINT(1U, s);
		}
	}
}

/* -------------------------------------------------------------- overrun ---- */

static void test_overrun_allows_one_millisecond_of_slack(void)
{
	/* Nominal and one tick of quantisation are not overruns. */
	TEST_ASSERT_FALSE(sts_io_scan_overrun(1000U, 1000U, 1U));
	TEST_ASSERT_FALSE(sts_io_scan_overrun(1001U, 1000U, 1U));
	TEST_ASSERT_FALSE(sts_io_scan_overrun(1002U, 1000U, 1U));
	/* 3 ms for a 1 ms slot is a missed slot. */
	TEST_ASSERT_TRUE(sts_io_scan_overrun(1003U, 1000U, 1U));
}

/* k_uptime_get_32() wraps at 49.7 days. The scan straddling that instant must
 * not log an overrun for every slot until it is rebooted. */
static void test_overrun_survives_the_32_bit_wrap(void)
{
	TEST_ASSERT_FALSE(sts_io_scan_overrun(0U, UINT32_MAX, 1U));
	TEST_ASSERT_FALSE(sts_io_scan_overrun(1U, UINT32_MAX, 1U));
	TEST_ASSERT_TRUE(sts_io_scan_overrun(2U, UINT32_MAX, 1U));
}

/* ------------------------------------------------------- dropped events ---- */

/*
 * The counter is cleared as part of reporting it, so the next scan reads zero.
 * A policy that remembered the last reported value would see 0 != N and emit a
 * second warning saying zero events were dropped — which is what the code did
 * before this seam existed.
 */
static void test_drop_reporting_does_not_double_report(void)
{
	sts_io_drop_action_t a;

	sts_io_drop_action(0U, &a);
	TEST_ASSERT_FALSE(a.clear);
	TEST_ASSERT_FALSE(a.log);

	sts_io_drop_action(5U, &a);
	TEST_ASSERT_TRUE(a.clear);
	TEST_ASSERT_TRUE(a.log);

	/* The scan right after the clear. */
	sts_io_drop_action(0U, &a);
	TEST_ASSERT_FALSE_MESSAGE(a.log, "reported a drop burst of zero events");
	TEST_ASSERT_FALSE(a.clear);

	/* A second, genuine burst still reports. */
	sts_io_drop_action(1U, &a);
	TEST_ASSERT_TRUE(a.log);
	TEST_ASSERT_TRUE(a.clear);
}

/* -------------------------------------------------------------------- main - */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_sig_port_bit_boundaries);
	RUN_TEST(test_sig_port_bit_is_a_bijection);
	RUN_TEST(test_sig_port_bit_rejects_bad_input);

	RUN_TEST(test_alert_signal_resolves_to_the_documented_monitor);
	RUN_TEST(test_every_monitor_has_exactly_one_alert_signal);
	RUN_TEST(test_non_alert_signals_resolve_to_nothing);

	RUN_TEST(test_button_press_and_release_carry_the_edge);
	RUN_TEST(test_long_and_repeat_are_always_asserting);
	RUN_TEST(test_touch_release_posts_nothing);
	RUN_TEST(test_prox_reports_both_directions);
	RUN_TEST(test_ina_alert_requests_the_right_monitor_on_assert_only);
	RUN_TEST(test_paired_events_keep_severity_and_wording_together);
	RUN_TEST(test_power_events_all_land_in_the_pwr_subsystem);
	RUN_TEST(test_unknown_event_is_fully_zeroed);
	RUN_TEST(test_every_message_takes_one_string_argument);

	RUN_TEST(test_overrun_allows_one_millisecond_of_slack);
	RUN_TEST(test_overrun_survives_the_32_bit_wrap);

	RUN_TEST(test_drop_reporting_does_not_double_report);

	return UNITY_END();
}
