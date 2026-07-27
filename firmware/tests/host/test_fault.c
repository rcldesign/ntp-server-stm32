/*
 * STS1000 "Meridian" — core/fault unit tests.
 *
 * The fault module is a pure function of (GPIOF IDR, GPIOG IDR, milliseconds),
 * so every test here drives a synthetic IDR stream one scan at a time and
 * inspects the events and alarms that fall out. `scan_for()` advances the model
 * at exactly 1 kHz, which is the rate the `io_scan` thread runs at, so the
 * millisecond figures in the assertions are also sample counts.
 *
 * The IDR words are built from *idle* levels and the bits under test are
 * cleared, never set: every one of the 32 signals is active low (buttons pull
 * to ground, the nFLG flags and INA ALERTs are open drain, the power-goods read
 * high when good), so idle is all-ones and a fault is a zero. A test that built
 * a word the other way round would pass against an inverted implementation, so
 * `idle_f`/`idle_g` and `clr()` are the only way words are constructed here.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "fault/fault.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* ---------------------------------------------------------------- harness */

typedef struct {
	fault_ctx_t ctx;
	uint16_t f;
	uint16_t g;
	uint32_t ms;
} model_t;

static void model_init(model_t *m, const fault_cfg_t *cfg)
{
	memset(m, 0, sizeof(*m));
	TEST_ASSERT_EQUAL_INT(0, fault_init(&m->ctx, cfg));
	m->f = 0xFFFFU; /* every signal idle = electrically high */
	m->g = 0xFFFFU;
	m->ms = 0U;
}

/* Clear (assert) bit @p b of a port word. */
static uint16_t clr(uint16_t word, unsigned int b)
{
	return (uint16_t)(word & ~(uint16_t)(1U << b));
}

/* Set (de-assert) bit @p b of a port word. */
static uint16_t set(uint16_t word, unsigned int b)
{
	return (uint16_t)(word | (uint16_t)(1U << b));
}

/* Run @p n scans at 1 kHz with the current port words. */
static void scan_for(model_t *m, unsigned int n)
{
	unsigned int i;

	for (i = 0U; i < n; i++) {
		TEST_ASSERT_EQUAL_INT(
			0, fault_scan_input(&m->ctx, m->f, m->g, m->ms));
		m->ms++;
	}
}

/* Drain and discard everything queued. */
static void drain(model_t *m)
{
	fault_evt_t e;

	while (fault_evt_get(&m->ctx, &e) == 0) {
	}
	fault_evt_clear_dropped(&m->ctx);
}

/* Pop one event and assert its shape. */
static void expect_evt(model_t *m, fault_evt_type_t type, fault_sig_t sig,
		       fault_edge_t edge)
{
	fault_evt_t e;

	TEST_ASSERT_EQUAL_INT_MESSAGE(0, fault_evt_get(&m->ctx, &e),
				      "expected an event, queue was empty");
	TEST_ASSERT_EQUAL_INT_MESSAGE((int)type, (int)e.type, "event type");
	TEST_ASSERT_EQUAL_INT_MESSAGE((int)sig, (int)e.id, fault_sig_name(sig));
	TEST_ASSERT_EQUAL_INT_MESSAGE((int)edge, (int)e.edge, "event edge");
}

static void expect_empty(model_t *m)
{
	fault_evt_t e;

	TEST_ASSERT_EQUAL_INT_MESSAGE(-EAGAIN, fault_evt_get(&m->ctx, &e),
				      "expected no further events");
}

/* ---------------------------------------------------------------- basics */

static void test_init_validates_and_defaults(void)
{
	fault_cfg_t cfg;
	fault_ctx_t ctx;

	TEST_ASSERT_EQUAL_INT(-EINVAL, fault_init(NULL, NULL));

	TEST_ASSERT_EQUAL_INT(0, fault_init(&ctx, NULL));
	TEST_ASSERT_EQUAL_UINT16(25U, ctx.cfg.button_debounce_ms);
	TEST_ASSERT_EQUAL_UINT16(5U, ctx.cfg.en_fault_debounce_ms);
	TEST_ASSERT_EQUAL_UINT16(5U, ctx.cfg.pg_debounce_ms);
	TEST_ASSERT_EQUAL_UINT16(50U, ctx.cfg.prox_debounce_ms);
	TEST_ASSERT_EQUAL_UINT16(2U, ctx.cfg.ina_debounce_samples);
	TEST_ASSERT_EQUAL_UINT16(800U, ctx.cfg.long_press_ms);
	TEST_ASSERT_EQUAL_UINT16(1000U, ctx.cfg.repeat_delay_ms);
	TEST_ASSERT_EQUAL_UINT16(250U, ctx.cfg.repeat_period_ms);
	TEST_ASSERT_EQUAL_UINT64(FAULT_RELAY_DISQUALIFY_DEFAULT,
				 fault_relay_disqualify_mask(&ctx));

	/* Interface ref §5.3 allows 20-30 ms for the button class. */
	TEST_ASSERT_TRUE(ctx.cfg.button_debounce_ms >= 20U);
	TEST_ASSERT_TRUE(ctx.cfg.button_debounce_ms <= 30U);

	/* A zero repeat period would fire on every scan. */
	fault_cfg_default(&cfg);
	cfg.repeat_period_ms = 0U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, fault_init(&ctx, &cfg));

	/* A zero sample gate would defeat the INA glitch filter. */
	fault_cfg_default(&cfg);
	cfg.ina_debounce_samples = 0U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, fault_init(&ctx, &cfg));

	fault_cfg_default(NULL); /* must not fault */
}

static void test_signal_table_covers_every_pin_of_both_ports(void)
{
	unsigned int seen[FAULT_CLASS_COUNT];
	unsigned int i;

	memset(seen, 0, sizeof(seen));

	for (i = 0U; i < (unsigned int)FAULT_SIG_COUNT; i++) {
		fault_class_t c = fault_sig_class((fault_sig_t)i);

		TEST_ASSERT_TRUE(c < FAULT_CLASS_COUNT);
		seen[c]++;
		TEST_ASSERT_NOT_NULL(fault_sig_name((fault_sig_t)i));
		TEST_ASSERT_TRUE(fault_sig_name((fault_sig_t)i)[0] != '\0');
	}

	/* Fault-aggregation doc §4 and interface ref §5: 7 keypad buttons plus
	 * the encoder switch, one touch INT, one reed switch, three RT9742
	 * nFLG flags, eight rail power-goods, two backup power-goods, and nine
	 * INA228 alerts. 8+1+1+3+8+2+9 = 32, all sixteen pins of both ports. */
	TEST_ASSERT_EQUAL_UINT(8U, seen[FAULT_CLASS_BUTTON]);
	TEST_ASSERT_EQUAL_UINT(1U, seen[FAULT_CLASS_TOUCH]);
	TEST_ASSERT_EQUAL_UINT(1U, seen[FAULT_CLASS_PROX]);
	TEST_ASSERT_EQUAL_UINT(3U, seen[FAULT_CLASS_EN_FAULT]);
	TEST_ASSERT_EQUAL_UINT(8U, seen[FAULT_CLASS_PG]);
	TEST_ASSERT_EQUAL_UINT(2U, seen[FAULT_CLASS_BKP_PG]);
	TEST_ASSERT_EQUAL_UINT(9U, seen[FAULT_CLASS_INA_ALERT]);

	TEST_ASSERT_EQUAL_INT(FAULT_CLASS_COUNT,
			      fault_sig_class((fault_sig_t)FAULT_SIG_COUNT));
	TEST_ASSERT_EQUAL_STRING("invalid",
				 fault_sig_name((fault_sig_t)FAULT_SIG_COUNT));
}

static void test_documented_masks_match_the_signal_assignments(void)
{
	unsigned int i;
	uint16_t buttons = 0U;
	uint16_t en_faults = 0U;
	uint16_t bkp = 0U;
	uint16_t pg_g = 0U;
	uint16_t ina_g = 0U;

	for (i = 0U; i < (unsigned int)FAULT_SIG_COUNT; i++) {
		fault_class_t c = fault_sig_class((fault_sig_t)i);
		uint16_t bit = (uint16_t)(1U << (i & 15U));

		if (i < 16U) {
			if ((c == FAULT_CLASS_BUTTON) &&
			    (i != FAULT_SIG_ENC_BUTTON)) {
				buttons |= bit;
			} else if (c == FAULT_CLASS_EN_FAULT) {
				en_faults |= bit;
			} else if (c == FAULT_CLASS_BKP_PG) {
				bkp |= bit;
			}
		} else {
			if (c == FAULT_CLASS_PG) {
				pg_g |= bit;
			} else if (c == FAULT_CLASS_INA_ALERT) {
				ina_g |= bit;
			}
		}
	}

	TEST_ASSERT_EQUAL_HEX16(FAULT_MASK_F_BUTTONS, buttons);
	TEST_ASSERT_EQUAL_HEX16(FAULT_MASK_F_EN_FAULTS, en_faults);
	TEST_ASSERT_EQUAL_HEX16(FAULT_MASK_F_BKP_PG, bkp);
	TEST_ASSERT_EQUAL_HEX16(FAULT_MASK_G_PG_RAILS, pg_g);
	TEST_ASSERT_EQUAL_HEX16(FAULT_MASK_G_INA_ALERTS, ina_g);

	/* The literal masks from interface ref §5.1/§5.2. */
	TEST_ASSERT_EQUAL_HEX16(0x007F, FAULT_MASK_F_BUTTONS);
	TEST_ASSERT_EQUAL_HEX16(0x0080, FAULT_MASK_F_TOUCH);
	TEST_ASSERT_EQUAL_HEX16(0x0400, FAULT_MASK_F_PROX);
	TEST_ASSERT_EQUAL_HEX16(0x0800, FAULT_MASK_F_ENC_BUTTON);
	TEST_ASSERT_EQUAL_HEX16(0x2000, FAULT_MASK_F_INA_ALERT);
	TEST_ASSERT_EQUAL_HEX16(0xC000, FAULT_MASK_F_BKP_PG);
	TEST_ASSERT_EQUAL_HEX16(0x00FF, FAULT_MASK_G_PG_RAILS);
	TEST_ASSERT_EQUAL_HEX16(0xFF00, FAULT_MASK_G_INA_ALERTS);
}

static void test_idle_ports_produce_nothing(void)
{
	model_t m;

	model_init(&m, NULL);
	scan_for(&m, 200U);

	TEST_ASSERT_EQUAL_UINT32(0U, fault_state(&m.ctx));
	TEST_ASSERT_EQUAL_size_t(0U, fault_evt_count(&m.ctx));
	TEST_ASSERT_EQUAL_UINT64(0U, fault_alarms(&m.ctx));
	TEST_ASSERT_EQUAL_UINT64(0U, fault_alarms_latched(&m.ctx));
	TEST_ASSERT_TRUE(fault_relay_eligible(&m.ctx));
}

static void test_null_context_is_handled_everywhere(void)
{
	fault_evt_t e;

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      fault_scan_input(NULL, 0xFFFFU, 0xFFFFU, 0U));
	TEST_ASSERT_EQUAL_UINT32(0U, fault_state(NULL));
	TEST_ASSERT_FALSE(fault_asserted(NULL, FAULT_SIG_BUTTON_1));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fault_evt_get(NULL, &e));
	TEST_ASSERT_EQUAL_size_t(0U, fault_evt_count(NULL));
	TEST_ASSERT_EQUAL_UINT32(0U, fault_evt_dropped(NULL));
	fault_evt_clear_dropped(NULL);
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      fault_alarm_set(NULL, FAULT_ALARM_PFI, true, 0U));
	TEST_ASSERT_NULL(fault_alarm_get(NULL, FAULT_ALARM_PFI));
	TEST_ASSERT_EQUAL_UINT64(0U, fault_alarms(NULL));
	TEST_ASSERT_EQUAL_UINT64(0U, fault_alarms_latched(NULL));
	TEST_ASSERT_FALSE(fault_any_active(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fault_alarm_clear(NULL, FAULT_ALARM_PFI));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fault_alarm_clear_all(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, fault_relay_set_disqualify_mask(NULL, 0U));
	TEST_ASSERT_EQUAL_UINT64(0U, fault_relay_disqualify_mask(NULL));
	TEST_ASSERT_EQUAL_UINT64(0U, fault_relay_blocking(NULL));

	/* Fail-safe: an absent context must not authorise energizing K2. */
	TEST_ASSERT_FALSE(fault_relay_eligible(NULL));
}

/* -------------------------------------------------------------- debounce */

static void test_button_press_commits_after_the_debounce_interval(void)
{
	model_t m;

	model_init(&m, NULL);
	scan_for(&m, 10U);

	m.f = clr(m.f, FAULT_SIG_BUTTON_1);

	/* Nothing until the interval elapses. */
	scan_for(&m, 25U); /* scans at t=10..34, candidate opened at t=10 */
	TEST_ASSERT_EQUAL_size_t(0U, fault_evt_count(&m.ctx));
	TEST_ASSERT_FALSE(fault_asserted(&m.ctx, FAULT_SIG_BUTTON_1));

	scan_for(&m, 1U); /* t=35, elapsed 25 ms */
	TEST_ASSERT_TRUE(fault_asserted(&m.ctx, FAULT_SIG_BUTTON_1));
	expect_evt(&m, FAULT_EVT_BUTTON, FAULT_SIG_BUTTON_1,
		   FAULT_EDGE_ASSERT);
	expect_empty(&m);

	/* Release debounces the same way and reports the other edge. */
	m.f = set(m.f, FAULT_SIG_BUTTON_1);
	scan_for(&m, 25U);
	TEST_ASSERT_EQUAL_size_t(0U, fault_evt_count(&m.ctx));
	scan_for(&m, 1U);
	expect_evt(&m, FAULT_EVT_BUTTON, FAULT_SIG_BUTTON_1,
		   FAULT_EDGE_DEASSERT);
	TEST_ASSERT_FALSE(fault_asserted(&m.ctx, FAULT_SIG_BUTTON_1));
}

static void test_a_bouncing_contact_yields_exactly_one_event(void)
{
	model_t m;
	unsigned int i;

	model_init(&m, NULL);
	scan_for(&m, 5U);

	/* 3 ms of chatter — six transitions — then a settled low. */
	for (i = 0U; i < 3U; i++) {
		m.f = clr(m.f, FAULT_SIG_BUTTON_3);
		scan_for(&m, 1U);
		m.f = set(m.f, FAULT_SIG_BUTTON_3);
		scan_for(&m, 1U);
	}
	m.f = clr(m.f, FAULT_SIG_BUTTON_3);
	scan_for(&m, 40U);

	TEST_ASSERT_TRUE(fault_asserted(&m.ctx, FAULT_SIG_BUTTON_3));
	expect_evt(&m, FAULT_EVT_BUTTON, FAULT_SIG_BUTTON_3,
		   FAULT_EDGE_ASSERT);
	expect_empty(&m);
}

static void test_a_sub_debounce_glitch_yields_nothing(void)
{
	model_t m;

	model_init(&m, NULL);
	scan_for(&m, 5U);

	/* 10 ms low, well under the 25 ms button interval. */
	m.f = clr(m.f, FAULT_SIG_BUTTON_5);
	scan_for(&m, 10U);
	m.f = set(m.f, FAULT_SIG_BUTTON_5);
	scan_for(&m, 100U);

	TEST_ASSERT_FALSE(fault_asserted(&m.ctx, FAULT_SIG_BUTTON_5));
	expect_empty(&m);
}

static void test_debounce_restarts_on_every_bounce(void)
{
	model_t m;

	model_init(&m, NULL);
	scan_for(&m, 5U);

	/*
	 * 20 ms low, 1 ms high, 20 ms low. Neither low run reaches 25 ms and
	 * the high sample cancels the candidate, so a debouncer that merely
	 * accumulated time would fire here and a correct one does not.
	 */
	m.f = clr(m.f, FAULT_SIG_BUTTON_2);
	scan_for(&m, 20U);
	m.f = set(m.f, FAULT_SIG_BUTTON_2);
	scan_for(&m, 1U);
	m.f = clr(m.f, FAULT_SIG_BUTTON_2);
	scan_for(&m, 20U);

	TEST_ASSERT_FALSE(fault_asserted(&m.ctx, FAULT_SIG_BUTTON_2));
	expect_empty(&m);

	/* Five more scans completes the second run's 25 ms. */
	scan_for(&m, 6U);
	TEST_ASSERT_TRUE(fault_asserted(&m.ctx, FAULT_SIG_BUTTON_2));
	expect_evt(&m, FAULT_EVT_BUTTON, FAULT_SIG_BUTTON_2,
		   FAULT_EDGE_ASSERT);
}

static void test_each_class_uses_its_own_interval(void)
{
	struct {
		unsigned int port_f; /* 1 = GPIOF, 0 = GPIOG */
		unsigned int bit;
		fault_sig_t sig;
		unsigned int settle_ms;
	} const v[] = {
		{1U, 8U, FAULT_SIG_V_ANT_EN_FAULT, 5U},  /* EN fault: 5 ms */
		{1U, 9U, FAULT_SIG_V_DISP_EN_FAULT, 5U},
		{1U, 12U, FAULT_SIG_PANEL_LED_FAULT, 5U},
		{1U, 10U, FAULT_SIG_PROX_WAKE, 50U},     /* reed: 50 ms */
		{0U, 4U, FAULT_SIG_PG_3V3_PSU, 5U},      /* rail PG: 5 ms */
		{1U, 14U, FAULT_SIG_BKP_STM_PG, 5U},     /* backup PG: 5 ms */
	};
	size_t i;

	for (i = 0U; i < ARRAY_LEN(v); i++) {
		model_t m;

		model_init(&m, NULL);
		scan_for(&m, 2U);

		if (v[i].port_f != 0U) {
			m.f = clr(m.f, v[i].bit);
		} else {
			m.g = clr(m.g, v[i].bit);
		}

		/* One scan short of the interval: still nothing. */
		scan_for(&m, v[i].settle_ms);
		TEST_ASSERT_FALSE_MESSAGE(fault_asserted(&m.ctx, v[i].sig),
					  fault_sig_name(v[i].sig));

		scan_for(&m, 1U);
		TEST_ASSERT_TRUE_MESSAGE(fault_asserted(&m.ctx, v[i].sig),
					 fault_sig_name(v[i].sig));
	}
}

static void test_touch_int_has_no_debounce_and_reports_only_the_assert(void)
{
	model_t m;

	model_init(&m, NULL);
	scan_for(&m, 3U);

	/*
	 * PF7 carries no debounce capacitor (fault-aggregation doc §4) and the
	 * INT is a request to read the controller over I2C, so the very first
	 * scan that sees it low must dispatch.
	 */
	m.f = clr(m.f, FAULT_SIG_TOUCH_INT);
	scan_for(&m, 1U);
	TEST_ASSERT_TRUE(fault_asserted(&m.ctx, FAULT_SIG_TOUCH_INT));
	expect_evt(&m, FAULT_EVT_TOUCH, FAULT_SIG_TOUCH_INT,
		   FAULT_EDGE_ASSERT);
	expect_empty(&m);

	/* The release carries no information, so it produces no event — but
	 * the state must still follow it or the next touch is lost. */
	m.f = set(m.f, FAULT_SIG_TOUCH_INT);
	scan_for(&m, 1U);
	TEST_ASSERT_FALSE(fault_asserted(&m.ctx, FAULT_SIG_TOUCH_INT));
	expect_empty(&m);

	m.f = clr(m.f, FAULT_SIG_TOUCH_INT);
	scan_for(&m, 1U);
	expect_evt(&m, FAULT_EVT_TOUCH, FAULT_SIG_TOUCH_INT,
		   FAULT_EDGE_ASSERT);
}

static void test_ina_alert_needs_two_consecutive_samples(void)
{
	model_t m;

	model_init(&m, NULL);
	scan_for(&m, 3U);

	/* One scan is not enough. */
	m.g = clr(m.g, 8U); /* PG8 INA_ALERT_V_POE */
	scan_for(&m, 1U);
	TEST_ASSERT_FALSE(fault_asserted(&m.ctx, FAULT_SIG_INA_ALERT_V_POE));
	expect_empty(&m);

	/* The second agreeing scan commits. */
	scan_for(&m, 1U);
	TEST_ASSERT_TRUE(fault_asserted(&m.ctx, FAULT_SIG_INA_ALERT_V_POE));
	expect_evt(&m, FAULT_EVT_INA_ALERT, FAULT_SIG_INA_ALERT_V_POE,
		   FAULT_EDGE_ASSERT);

	/* A single-scan glitch on the open-drain line is rejected. */
	drain(&m);
	m.g = set(m.g, 8U);
	scan_for(&m, 1U);
	m.g = clr(m.g, 8U);
	scan_for(&m, 5U);
	TEST_ASSERT_TRUE(fault_asserted(&m.ctx, FAULT_SIG_INA_ALERT_V_POE));
	expect_empty(&m);
}

static void test_ina_sample_gate_is_configurable(void)
{
	model_t m;
	fault_cfg_t cfg;

	fault_cfg_default(&cfg);
	cfg.ina_debounce_samples = 4U;
	model_init(&m, &cfg);
	scan_for(&m, 2U);

	m.g = clr(m.g, 15U); /* PG15 INA_ALERT_VCC_RB */
	scan_for(&m, 3U);
	TEST_ASSERT_FALSE(fault_asserted(&m.ctx, FAULT_SIG_INA_ALERT_VCC_RB));
	scan_for(&m, 1U);
	TEST_ASSERT_TRUE(fault_asserted(&m.ctx, FAULT_SIG_INA_ALERT_VCC_RB));
}

/* -------------------------------------------------- long press and repeat */

static void test_long_press_fires_once_at_the_threshold(void)
{
	model_t m;

	model_init(&m, NULL);
	scan_for(&m, 1U);

	m.f = clr(m.f, FAULT_SIG_BUTTON_4);
	scan_for(&m, 26U); /* press commits at t=26, press_ms = 26 */
	expect_evt(&m, FAULT_EVT_BUTTON, FAULT_SIG_BUTTON_4,
		   FAULT_EDGE_ASSERT);
	expect_empty(&m);

	/* 799 ms after the press: still nothing. */
	scan_for(&m, 799U);
	expect_empty(&m);

	scan_for(&m, 1U); /* held 800 ms */
	expect_evt(&m, FAULT_EVT_BUTTON_LONG, FAULT_SIG_BUTTON_4,
		   FAULT_EDGE_ASSERT);

	/* Exactly once, however long the hold continues. */
	scan_for(&m, 150U);
	expect_empty(&m);
}

static void test_auto_repeat_starts_after_one_second_at_four_hertz(void)
{
	model_t m;
	unsigned int i;

	model_init(&m, NULL);
	scan_for(&m, 1U);

	m.f = clr(m.f, FAULT_SIG_BUTTON_6);
	scan_for(&m, 26U);
	expect_evt(&m, FAULT_EVT_BUTTON, FAULT_SIG_BUTTON_6,
		   FAULT_EDGE_ASSERT);

	scan_for(&m, 800U);
	expect_evt(&m, FAULT_EVT_BUTTON_LONG, FAULT_SIG_BUTTON_6,
		   FAULT_EDGE_ASSERT);
	expect_empty(&m);

	/* First repeat at 1000 ms after the press, i.e. 200 ms after LONG. */
	scan_for(&m, 199U);
	expect_empty(&m);
	scan_for(&m, 1U);
	expect_evt(&m, FAULT_EVT_BUTTON_REPEAT, FAULT_SIG_BUTTON_6,
		   FAULT_EDGE_ASSERT);

	/* Then one every 250 ms. */
	for (i = 0U; i < 4U; i++) {
		scan_for(&m, 249U);
		expect_empty(&m);
		scan_for(&m, 1U);
		expect_evt(&m, FAULT_EVT_BUTTON_REPEAT, FAULT_SIG_BUTTON_6,
			   FAULT_EDGE_ASSERT);
	}

	/* Release stops the repeat immediately after its own debounce. */
	m.f = set(m.f, FAULT_SIG_BUTTON_6);
	scan_for(&m, 26U);
	expect_evt(&m, FAULT_EVT_BUTTON, FAULT_SIG_BUTTON_6,
		   FAULT_EDGE_DEASSERT);
	scan_for(&m, 1000U);
	expect_empty(&m);
}

static void test_repeat_never_bursts_when_scans_are_skipped(void)
{
	model_t m;
	unsigned int i;
	unsigned int repeats = 0U;
	fault_evt_t e;

	model_init(&m, NULL);
	scan_for(&m, 1U);
	m.f = clr(m.f, FAULT_SIG_ENC_BUTTON);
	scan_for(&m, 26U);
	drain(&m);

	/*
	 * A stalled io_scan that resumes 5 s later must not dump twenty
	 * queued repeats into the nav state machine. At most one per scan.
	 */
	for (i = 0U; i < 3U; i++) {
		m.ms += 5000U;
		TEST_ASSERT_EQUAL_INT(
			0, fault_scan_input(&m.ctx, m.f, m.g, m.ms));
		m.ms++;

		while (fault_evt_get(&m.ctx, &e) == 0) {
			if (e.type == FAULT_EVT_BUTTON_REPEAT) {
				repeats++;
			}
		}
		TEST_ASSERT_TRUE(repeats <= (i + 1U));
	}
	TEST_ASSERT_EQUAL_UINT(3U, repeats);
}

static void test_the_encoder_switch_behaves_as_a_button(void)
{
	model_t m;

	model_init(&m, NULL);
	scan_for(&m, 1U);

	/* PF11 is scanned; the quadrature A/B on PA8/PA9 is not (TIM1). */
	TEST_ASSERT_EQUAL_INT(FAULT_CLASS_BUTTON,
			      fault_sig_class(FAULT_SIG_ENC_BUTTON));

	m.f = clr(m.f, FAULT_SIG_ENC_BUTTON);
	scan_for(&m, 26U);
	expect_evt(&m, FAULT_EVT_BUTTON, FAULT_SIG_ENC_BUTTON,
		   FAULT_EDGE_ASSERT);
	scan_for(&m, 800U);
	expect_evt(&m, FAULT_EVT_BUTTON_LONG, FAULT_SIG_ENC_BUTTON,
		   FAULT_EDGE_ASSERT);
}

static void test_buttons_are_independent(void)
{
	model_t m;

	model_init(&m, NULL);
	scan_for(&m, 1U);

	m.f = clr(m.f, FAULT_SIG_BUTTON_1);
	scan_for(&m, 26U);
	expect_evt(&m, FAULT_EVT_BUTTON, FAULT_SIG_BUTTON_1,
		   FAULT_EDGE_ASSERT);

	/* A second button pressed 100 ms later keeps its own hold clock. */
	scan_for(&m, 100U);
	m.f = clr(m.f, FAULT_SIG_BUTTON_7);
	scan_for(&m, 26U);
	expect_evt(&m, FAULT_EVT_BUTTON, FAULT_SIG_BUTTON_7,
		   FAULT_EDGE_ASSERT);

	scan_for(&m, 674U); /* button 1 reaches 800 ms held */
	expect_evt(&m, FAULT_EVT_BUTTON_LONG, FAULT_SIG_BUTTON_1,
		   FAULT_EDGE_ASSERT);
	expect_empty(&m);

	scan_for(&m, 126U); /* button 7 reaches 800 ms */
	expect_evt(&m, FAULT_EVT_BUTTON_LONG, FAULT_SIG_BUTTON_7,
		   FAULT_EDGE_ASSERT);
}

/* -------------------------------------------------------- rails and alerts */

static void test_pg_drop_and_recover_emit_distinct_events(void)
{
	model_t m;
	const fault_alarm_t *a;

	model_init(&m, NULL);
	scan_for(&m, 10U);

	/* PG1 OCXO_LDO_PG goes high->low: rail fault. */
	m.g = clr(m.g, 1U);
	scan_for(&m, 6U);
	expect_evt(&m, FAULT_EVT_PG_FAULT, FAULT_SIG_PG_OCXO_LDO,
		   FAULT_EDGE_ASSERT);

	a = fault_alarm_get(&m.ctx, (fault_alarm_id_t)FAULT_SIG_PG_OCXO_LDO);
	TEST_ASSERT_NOT_NULL(a);
	TEST_ASSERT_TRUE(a->active);
	TEST_ASSERT_TRUE(a->latched);
	TEST_ASSERT_EQUAL_UINT32(1U, a->count);
	TEST_ASSERT_EQUAL_UINT32(15U, a->first_ms);

	/* And back: recovery is its own event type. */
	m.g = set(m.g, 1U);
	scan_for(&m, 6U);
	expect_evt(&m, FAULT_EVT_PG_RECOVER, FAULT_SIG_PG_OCXO_LDO,
		   FAULT_EDGE_DEASSERT);

	a = fault_alarm_get(&m.ctx, (fault_alarm_id_t)FAULT_SIG_PG_OCXO_LDO);
	TEST_ASSERT_FALSE(a->active);
	TEST_ASSERT_TRUE(a->latched); /* the visit is remembered */
	TEST_ASSERT_EQUAL_UINT32(1U, a->count);
}

static void test_all_eight_pg_rails_are_usable_with_no_masking(void)
{
	model_t m;
	unsigned int b;

	/*
	 * Interface ref §5.2: "All eight PG bits are usable — no masking."
	 * PG2 (3V0_RF_LDO_PG) used to need masking; R266 is now fitted, so a
	 * regression that special-cased it would show up here.
	 */
	for (b = 0U; b < 8U; b++) {
		fault_sig_t sig = (fault_sig_t)(FAULT_SIG_PG_3V3_GPS_LDO + b);

		model_init(&m, NULL);
		scan_for(&m, 2U);
		m.g = clr(m.g, b);
		scan_for(&m, 6U);

		TEST_ASSERT_TRUE_MESSAGE(fault_asserted(&m.ctx, sig),
					 fault_sig_name(sig));
		expect_evt(&m, FAULT_EVT_PG_FAULT, sig, FAULT_EDGE_ASSERT);
		expect_empty(&m);
	}
}

static void test_all_nine_ina_alerts_dispatch_including_the_one_on_port_f(void)
{
	model_t m;
	unsigned int b;

	model_init(&m, NULL);
	scan_for(&m, 2U);

	/* Eight on GPIOG[8:15] plus the panel monitor on PF13. */
	for (b = 8U; b < 16U; b++) {
		m.g = clr(m.g, b);
	}
	m.f = clr(m.f, 13U);
	scan_for(&m, 2U);

	TEST_ASSERT_EQUAL_size_t(9U, fault_evt_count(&m.ctx));
	TEST_ASSERT_TRUE(fault_asserted(&m.ctx, FAULT_SIG_INA_ALERT_PANEL));
	TEST_ASSERT_TRUE(fault_asserted(&m.ctx, FAULT_SIG_INA_ALERT_VCC_RB));

	/* The panel alert dispatches first because PF13 precedes GPIOG in the
	 * combined bitmap — identity is preserved either way. */
	expect_evt(&m, FAULT_EVT_INA_ALERT, FAULT_SIG_INA_ALERT_PANEL,
		   FAULT_EDGE_ASSERT);
	for (b = 0U; b < 8U; b++) {
		expect_evt(&m, FAULT_EVT_INA_ALERT,
			   (fault_sig_t)(FAULT_SIG_INA_ALERT_V_POE + b),
			   FAULT_EDGE_ASSERT);
	}
	expect_empty(&m);
}

static void test_en_fault_flags_raise_their_alarms(void)
{
	model_t m;

	model_init(&m, NULL);
	scan_for(&m, 2U);

	m.f = clr(m.f, 9U); /* V_DISP_EN_FAULT, U33 RT9742 nFLG */
	scan_for(&m, 6U);
	expect_evt(&m, FAULT_EVT_EN_FAULT, FAULT_SIG_V_DISP_EN_FAULT,
		   FAULT_EDGE_ASSERT);
	TEST_ASSERT_TRUE(
		(fault_alarms(&m.ctx) &
		 FAULT_ALARM_BIT(FAULT_SIG_V_DISP_EN_FAULT)) != 0U);

	m.f = set(m.f, 9U);
	scan_for(&m, 6U);
	expect_evt(&m, FAULT_EVT_EN_FAULT, FAULT_SIG_V_DISP_EN_FAULT,
		   FAULT_EDGE_DEASSERT);
	TEST_ASSERT_EQUAL_UINT64(0U, fault_alarms(&m.ctx));
}

static void test_proximity_reports_both_directions(void)
{
	model_t m;

	model_init(&m, NULL);
	scan_for(&m, 2U);

	/* Dry reed contact: closing to ground means a magnet is present. */
	m.f = clr(m.f, 10U);
	scan_for(&m, 51U);
	expect_evt(&m, FAULT_EVT_PROX, FAULT_SIG_PROX_WAKE, FAULT_EDGE_ASSERT);

	m.f = set(m.f, 10U);
	scan_for(&m, 51U);
	expect_evt(&m, FAULT_EVT_PROX, FAULT_SIG_PROX_WAKE,
		   FAULT_EDGE_DEASSERT);

	/* Proximity is a UI wake, not an alarm. */
	TEST_ASSERT_EQUAL_UINT64(0U, fault_alarms_latched(&m.ctx));
}

static void test_backup_rail_power_good_is_reported_and_alarmed(void)
{
	model_t m;

	model_init(&m, NULL);
	scan_for(&m, 2U);

	m.f = clr(m.f, 15U); /* BKP_GPS_PG — the ephemeris-hold rail */
	scan_for(&m, 6U);
	expect_evt(&m, FAULT_EVT_BKP_PG, FAULT_SIG_BKP_GPS_PG,
		   FAULT_EDGE_ASSERT);
	TEST_ASSERT_TRUE((fault_alarms(&m.ctx) &
			  FAULT_ALARM_BIT(FAULT_SIG_BKP_GPS_PG)) != 0U);

	/* It must not disqualify the relay: the supercaps back the RTC and the
	 * GNSS ephemeris, not the time being served. */
	TEST_ASSERT_TRUE(fault_relay_eligible(&m.ctx));
}

static void test_a_rail_already_down_at_boot_raises_its_fault(void)
{
	model_t m;

	/*
	 * The committed state starts de-asserted rather than adopting the first
	 * scan, so a board that comes up with a dead rail annunciates it
	 * instead of treating it as normal.
	 */
	model_init(&m, NULL);
	m.g = clr(m.g, 3U); /* 5V_PSU_PG low from the very first scan */
	scan_for(&m, 6U);

	expect_evt(&m, FAULT_EVT_PG_FAULT, FAULT_SIG_PG_5V_PSU,
		   FAULT_EDGE_ASSERT);
	TEST_ASSERT_FALSE(fault_relay_eligible(&m.ctx));
}

/* ----------------------------------------------------------- event queue */

static void test_queue_drops_the_newest_and_counts_it(void)
{
	model_t m;
	fault_evt_t e;
	unsigned int i;

	model_init(&m, NULL);
	scan_for(&m, 2U);

	/*
	 * Assert all 32 signals at once. That is exactly one event per signal —
	 * which fills the queue to the brim without overflowing it — so hold
	 * long enough for the eight buttons to reach their long-press threshold
	 * and produce eight more that have nowhere to go.
	 */
	m.g = 0x0000U; /* every GPIOG signal asserted at once */
	m.f = 0x0000U; /* every GPIOF signal asserted at once */
	scan_for(&m, 900U);

	TEST_ASSERT_EQUAL_size_t((size_t)FAULT_EVT_QUEUE_LEN,
				 fault_evt_count(&m.ctx));
	TEST_ASSERT_EQUAL_UINT32(8U, fault_evt_dropped(&m.ctx));

	/*
	 * Drop-newest: the head must still be the earliest event, which for
	 * this stimulus is the touch INT (no debounce, so it commits on the
	 * first scan) — the root of the cascade, not a survivor of it.
	 */
	TEST_ASSERT_EQUAL_INT(0, fault_evt_get(&m.ctx, &e));
	TEST_ASSERT_EQUAL_INT(FAULT_EVT_TOUCH, (int)e.type);
	TEST_ASSERT_EQUAL_INT(FAULT_SIG_TOUCH_INT, (int)e.id);

	/* Timestamps are non-decreasing through the retained window. */
	{
		uint32_t prev = e.mono_ms;

		while (fault_evt_get(&m.ctx, &e) == 0) {
			TEST_ASSERT_TRUE(e.mono_ms >= prev);
			prev = e.mono_ms;
		}
	}

	fault_evt_clear_dropped(&m.ctx);
	TEST_ASSERT_EQUAL_UINT32(0U, fault_evt_dropped(&m.ctx));

	/* Once drained the queue accepts new events again. */
	m.f = 0xFFFFU;
	m.g = 0xFFFFU;
	scan_for(&m, 60U);
	TEST_ASSERT_TRUE(fault_evt_count(&m.ctx) > 0U);

	for (i = 0U; i < 5U; i++) {
		TEST_ASSERT_EQUAL_INT(0, fault_evt_get(&m.ctx, &e));
	}
}

static void test_queue_wraps_correctly_under_sustained_traffic(void)
{
	model_t m;
	fault_evt_t e;
	unsigned int cycle;
	unsigned int got = 0U;

	model_init(&m, NULL);
	scan_for(&m, 2U);

	/* Many more presses than the queue is deep, drained as we go, so the
	 * ring index laps repeatedly. */
	for (cycle = 0U; cycle < 40U; cycle++) {
		m.f = clr(m.f, FAULT_SIG_BUTTON_1);
		scan_for(&m, 26U);
		m.f = set(m.f, FAULT_SIG_BUTTON_1);
		scan_for(&m, 26U);

		while (fault_evt_get(&m.ctx, &e) == 0) {
			TEST_ASSERT_EQUAL_INT(FAULT_EVT_BUTTON, (int)e.type);
			got++;
		}
	}

	TEST_ASSERT_EQUAL_UINT(80U, got); /* one press + one release each */
	TEST_ASSERT_EQUAL_UINT32(0U, fault_evt_dropped(&m.ctx));
}

/* ---------------------------------------------------------------- alarms */

static void test_alarm_latch_records_first_time_and_count(void)
{
	fault_ctx_t ctx;
	const fault_alarm_t *a;

	TEST_ASSERT_EQUAL_INT(0, fault_init(&ctx, NULL));

	TEST_ASSERT_EQUAL_INT(
		0, fault_alarm_set(&ctx, FAULT_ALARM_GNSS_LOST, true, 1000U));
	a = fault_alarm_get(&ctx, FAULT_ALARM_GNSS_LOST);
	TEST_ASSERT_TRUE(a->active);
	TEST_ASSERT_TRUE(a->latched);
	TEST_ASSERT_EQUAL_UINT32(1U, a->count);
	TEST_ASSERT_EQUAL_UINT32(1000U, a->first_ms);
	TEST_ASSERT_EQUAL_UINT32(1000U, a->last_ms);

	/* Repeating the same level is idempotent — no count inflation. */
	TEST_ASSERT_EQUAL_INT(
		0, fault_alarm_set(&ctx, FAULT_ALARM_GNSS_LOST, true, 1500U));
	TEST_ASSERT_EQUAL_UINT32(1U, a->count);
	TEST_ASSERT_EQUAL_UINT32(1000U, a->last_ms);

	/* Off then on again counts a second occurrence but keeps first_ms. */
	TEST_ASSERT_EQUAL_INT(
		0, fault_alarm_set(&ctx, FAULT_ALARM_GNSS_LOST, false, 2000U));
	TEST_ASSERT_FALSE(a->active);
	TEST_ASSERT_TRUE(a->latched);
	TEST_ASSERT_EQUAL_INT(
		0, fault_alarm_set(&ctx, FAULT_ALARM_GNSS_LOST, true, 3000U));
	TEST_ASSERT_EQUAL_UINT32(2U, a->count);
	TEST_ASSERT_EQUAL_UINT32(1000U, a->first_ms);
	TEST_ASSERT_EQUAL_UINT32(3000U, a->last_ms);

	TEST_ASSERT_EQUAL_INT(
		-EINVAL, fault_alarm_set(&ctx, FAULT_ALARM_COUNT, true, 0U));
	TEST_ASSERT_NULL(fault_alarm_get(&ctx, FAULT_ALARM_COUNT));
}

static void test_clearing_a_live_alarm_is_refused(void)
{
	fault_ctx_t ctx;

	TEST_ASSERT_EQUAL_INT(0, fault_init(&ctx, NULL));

	/* Nothing latched yet. */
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      fault_alarm_clear(&ctx, FAULT_ALARM_RB_FAULT));

	TEST_ASSERT_EQUAL_INT(
		0, fault_alarm_set(&ctx, FAULT_ALARM_RB_FAULT, true, 10U));

	/* Still true: acknowledging must not make the alarm page read clean. */
	TEST_ASSERT_EQUAL_INT(-EBUSY,
			      fault_alarm_clear(&ctx, FAULT_ALARM_RB_FAULT));
	TEST_ASSERT_TRUE(fault_alarm_get(&ctx, FAULT_ALARM_RB_FAULT)->latched);

	/* Gone away: now it clears. */
	TEST_ASSERT_EQUAL_INT(
		0, fault_alarm_set(&ctx, FAULT_ALARM_RB_FAULT, false, 20U));
	TEST_ASSERT_EQUAL_INT(0, fault_alarm_clear(&ctx, FAULT_ALARM_RB_FAULT));
	TEST_ASSERT_FALSE(fault_alarm_get(&ctx, FAULT_ALARM_RB_FAULT)->latched);
	TEST_ASSERT_EQUAL_UINT32(
		0U, fault_alarm_get(&ctx, FAULT_ALARM_RB_FAULT)->count);

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      fault_alarm_clear(&ctx, FAULT_ALARM_COUNT));
}

static void test_clear_all_spares_the_still_active(void)
{
	fault_ctx_t ctx;

	TEST_ASSERT_EQUAL_INT(0, fault_init(&ctx, NULL));

	TEST_ASSERT_EQUAL_INT(
		0, fault_alarm_set(&ctx, FAULT_ALARM_THERMAL_WARN, true, 1U));
	TEST_ASSERT_EQUAL_INT(
		0, fault_alarm_set(&ctx, FAULT_ALARM_THERMAL_WARN, false, 2U));
	TEST_ASSERT_EQUAL_INT(
		0, fault_alarm_set(&ctx, FAULT_ALARM_FAN_FAULT, true, 3U));

	TEST_ASSERT_EQUAL_UINT64(
		FAULT_ALARM_BIT(FAULT_ALARM_THERMAL_WARN) |
			FAULT_ALARM_BIT(FAULT_ALARM_FAN_FAULT),
		fault_alarms_latched(&ctx));

	TEST_ASSERT_EQUAL_INT(0, fault_alarm_clear_all(&ctx));

	/* The fan is still stalled, so its latch survives. */
	TEST_ASSERT_EQUAL_UINT64(FAULT_ALARM_BIT(FAULT_ALARM_FAN_FAULT),
				 fault_alarms_latched(&ctx));
	TEST_ASSERT_EQUAL_UINT64(FAULT_ALARM_BIT(FAULT_ALARM_FAN_FAULT),
				 fault_alarms(&ctx));
	TEST_ASSERT_TRUE(fault_any_active(&ctx));
}

static void test_scan_signal_alarms_share_the_signal_numbering(void)
{
	model_t m;

	model_init(&m, NULL);
	scan_for(&m, 2U);

	/* A scanned fault and its alarm are the same id, so a consumer can go
	 * straight from an event to the latch entry. */
	m.g = clr(m.g, 7U); /* PG7 POE_PG */
	scan_for(&m, 6U);

	TEST_ASSERT_TRUE((fault_alarms(&m.ctx) &
			  FAULT_ALARM_BIT(FAULT_SIG_PG_POE)) != 0U);
	TEST_ASSERT_EQUAL_UINT32(
		1U,
		fault_alarm_get(&m.ctx, (fault_alarm_id_t)FAULT_SIG_PG_POE)
			->count);

	/* Ids 0..31 are the scan space; 32.. are software-raised. */
	TEST_ASSERT_EQUAL_INT(32, (int)FAULT_ALARM_REFERENCE_LOST);
	TEST_ASSERT_TRUE((int)FAULT_ALARM_COUNT <= 64);
}

/* ---------------------------------------------------------- relay policy */

static void test_relay_default_mask_covers_the_timing_chain(void)
{
	uint64_t m = FAULT_RELAY_DISQUALIFY_DEFAULT;

	/* In: rails the timing chain depends on. */
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_SIG_PG_OCXO_LDO)) != 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_SIG_PG_OCXO_PSU)) != 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_SIG_PG_3V3_PSU)) != 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_SIG_PG_5V_PSU)) != 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_SIG_PG_3V3_GPS_LDO)) != 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_SIG_PG_POE)) != 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_SIG_INA_ALERT_OCXO)) != 0U);

	/* In: the software faults that mean the served time is suspect. */
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_ALARM_REFERENCE_LOST)) != 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_ALARM_GNSS_LOST)) != 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_ALARM_OCXO_UNHEALTHY)) != 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_ALARM_DAC_FAULT)) != 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_ALARM_THERMAL_CRITICAL)) !=
			 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_ALARM_PFI)) != 0U);

	/* Out: spec §13 says the UI never blocks timing. */
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_SIG_PANEL_LED_FAULT)) == 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_SIG_V_DISP_EN_FAULT)) == 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_SIG_INA_ALERT_PANEL)) == 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_SIG_INA_ALERT_5V_DISP)) ==
			 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_ALARM_DISPLAY_FAULT)) == 0U);

	/* Out: losing the rubidium or the external reference reverts to the
	 * OCXO and service continues. */
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_SIG_PG_RB_PSU)) == 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_SIG_PG_3V0_RF_LDO)) == 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_SIG_INA_ALERT_VCC_RB)) == 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_ALARM_RB_FAULT)) == 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_ALARM_RB_OV)) == 0U);

	/* Out: housekeeping. */
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_ALARM_I2C_WEDGE)) == 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_ALARM_NOR_FAULT)) == 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_ALARM_POE_BUDGET)) == 0U);
	TEST_ASSERT_TRUE((m & FAULT_ALARM_BIT(FAULT_ALARM_TAMPER)) == 0U);
}

static void test_relay_drops_on_a_disqualifying_alarm_and_returns(void)
{
	fault_ctx_t ctx;

	TEST_ASSERT_EQUAL_INT(0, fault_init(&ctx, NULL));
	TEST_ASSERT_TRUE(fault_relay_eligible(&ctx));

	/* A non-disqualifying fault does not drop K2. */
	TEST_ASSERT_EQUAL_INT(
		0, fault_alarm_set(&ctx, FAULT_ALARM_RB_FAULT, true, 1U));
	TEST_ASSERT_TRUE(fault_relay_eligible(&ctx));
	TEST_ASSERT_EQUAL_UINT64(0U, fault_relay_blocking(&ctx));

	/* A disqualifying one does, and names itself. */
	TEST_ASSERT_EQUAL_INT(
		0, fault_alarm_set(&ctx, FAULT_ALARM_OCXO_UNHEALTHY, true, 2U));
	TEST_ASSERT_FALSE(fault_relay_eligible(&ctx));
	TEST_ASSERT_EQUAL_UINT64(FAULT_ALARM_BIT(FAULT_ALARM_OCXO_UNHEALTHY),
				 fault_relay_blocking(&ctx));

	/*
	 * Clearing the condition re-qualifies even though the latch survives:
	 * a fault that has genuinely gone must not hold K2 de-energized
	 * forever. Hysteresis on re-energizing lives in the quality gate.
	 */
	TEST_ASSERT_EQUAL_INT(
		0, fault_alarm_set(&ctx, FAULT_ALARM_OCXO_UNHEALTHY, false, 3U));
	TEST_ASSERT_TRUE(fault_relay_eligible(&ctx));
	TEST_ASSERT_TRUE(
		(fault_alarms_latched(&ctx) &
		 FAULT_ALARM_BIT(FAULT_ALARM_OCXO_UNHEALTHY)) != 0U);
}

static void test_relay_mask_is_replaceable(void)
{
	fault_ctx_t ctx;

	TEST_ASSERT_EQUAL_INT(0, fault_init(&ctx, NULL));

	/* An installation that treats a rubidium failure as service-affecting
	 * can say so without touching this module. */
	TEST_ASSERT_EQUAL_INT(
		0, fault_relay_set_disqualify_mask(
			   &ctx, FAULT_ALARM_BIT(FAULT_ALARM_RB_FAULT)));
	TEST_ASSERT_EQUAL_UINT64(FAULT_ALARM_BIT(FAULT_ALARM_RB_FAULT),
				 fault_relay_disqualify_mask(&ctx));

	TEST_ASSERT_EQUAL_INT(
		0, fault_alarm_set(&ctx, FAULT_ALARM_OCXO_UNHEALTHY, true, 1U));
	TEST_ASSERT_TRUE(fault_relay_eligible(&ctx)); /* no longer in the set */

	TEST_ASSERT_EQUAL_INT(
		0, fault_alarm_set(&ctx, FAULT_ALARM_RB_FAULT, true, 2U));
	TEST_ASSERT_FALSE(fault_relay_eligible(&ctx));

	/* An empty mask means nothing ever blocks. */
	TEST_ASSERT_EQUAL_INT(0, fault_relay_set_disqualify_mask(&ctx, 0U));
	TEST_ASSERT_TRUE(fault_relay_eligible(&ctx));
}

/* ------------------------------------------------------------ RGB policy */

static void test_rgb_priority_truth_table(void)
{
	unsigned int bits;

	/*
	 * All 32 input combinations against the documented priority:
	 * identify > fault > holdover|warming > locked > off.
	 */
	for (bits = 0U; bits < 32U; bits++) {
		fault_rgb_in_t in;
		fault_rgb_state_t want;

		in.locked = (bits & 1U) != 0U;
		in.holdover = (bits & 2U) != 0U;
		in.warming = (bits & 4U) != 0U;
		in.any_fault = (bits & 8U) != 0U;
		in.identify = (bits & 16U) != 0U;

		if (in.identify) {
			want = FAULT_RGB_BLUE_PULSE;
		} else if (in.any_fault) {
			want = FAULT_RGB_RED;
		} else if (in.holdover || in.warming) {
			want = FAULT_RGB_AMBER;
		} else if (in.locked) {
			want = FAULT_RGB_GREEN;
		} else {
			want = FAULT_RGB_OFF;
		}

		TEST_ASSERT_EQUAL_INT((int)want, (int)fault_rgb_state(&in));
	}
}

static void test_rgb_named_cases(void)
{
	fault_rgb_in_t in;

	memset(&in, 0, sizeof(in));
	TEST_ASSERT_EQUAL_INT(FAULT_RGB_OFF, fault_rgb_state(&in));

	in.locked = true;
	TEST_ASSERT_EQUAL_INT(FAULT_RGB_GREEN, fault_rgb_state(&in));

	/* Holdover while still nominally locked is amber, not green. */
	in.holdover = true;
	TEST_ASSERT_EQUAL_INT(FAULT_RGB_AMBER, fault_rgb_state(&in));

	in.any_fault = true;
	TEST_ASSERT_EQUAL_INT(FAULT_RGB_RED, fault_rgb_state(&in));

	/* Identify wins even over a fault: someone is looking for this box. */
	in.identify = true;
	TEST_ASSERT_EQUAL_INT(FAULT_RGB_BLUE_PULSE, fault_rgb_state(&in));

	TEST_ASSERT_EQUAL_INT(FAULT_RGB_OFF, fault_rgb_state(NULL));
}

/* ------------------------------------------------------------------ misc */

static void test_millisecond_wrap_is_transparent(void)
{
	model_t m;

	model_init(&m, NULL);

	/* Start just below the 32-bit rollover. */
	m.ms = 0xFFFFFFF0U;
	scan_for(&m, 5U);

	m.f = clr(m.f, FAULT_SIG_BUTTON_1);
	scan_for(&m, 26U); /* crosses zero mid-debounce */
	TEST_ASSERT_TRUE(fault_asserted(&m.ctx, FAULT_SIG_BUTTON_1));
	expect_evt(&m, FAULT_EVT_BUTTON, FAULT_SIG_BUTTON_1,
		   FAULT_EDGE_ASSERT);

	/* Long press and repeat survive the wrap too. */
	scan_for(&m, 800U);
	expect_evt(&m, FAULT_EVT_BUTTON_LONG, FAULT_SIG_BUTTON_1,
		   FAULT_EDGE_ASSERT);
	scan_for(&m, 200U);
	expect_evt(&m, FAULT_EVT_BUTTON_REPEAT, FAULT_SIG_BUTTON_1,
		   FAULT_EDGE_ASSERT);
}

static void test_simultaneous_transitions_keep_their_identity(void)
{
	model_t m;
	uint32_t expect;

	model_init(&m, NULL);
	scan_for(&m, 2U);

	/*
	 * The point of one pin per signal rather than a wire-OR: a rail
	 * collapse that takes four signals with it must land as four
	 * distinguishable events, not one aggregate interrupt.
	 */
	m.g = clr(m.g, 3U);  /* 5V_PSU_PG */
	m.g = clr(m.g, 10U); /* INA_ALERT_5V_DISP */
	m.f = clr(m.f, 9U);  /* V_DISP_EN_FAULT */
	m.f = clr(m.f, 12U); /* PANEL_LED_FAULT */
	scan_for(&m, 6U);

	expect = FAULT_SIG_BIT(FAULT_SIG_V_DISP_EN_FAULT) |
		 FAULT_SIG_BIT(FAULT_SIG_PANEL_LED_FAULT) |
		 FAULT_SIG_BIT(FAULT_SIG_PG_5V_PSU) |
		 FAULT_SIG_BIT(FAULT_SIG_INA_ALERT_5V_DISP);
	TEST_ASSERT_EQUAL_HEX32(expect, fault_state(&m.ctx));
	TEST_ASSERT_EQUAL_size_t(4U, fault_evt_count(&m.ctx));

	/*
	 * Ordered by commit time first, then by bit index. The INA alert leads
	 * because its gate is two samples while the EN-fault and power-good
	 * classes wait 5 ms — the alert is the earliest hard evidence of the
	 * collapse, which is the useful order for a diagnosis.
	 */
	expect_evt(&m, FAULT_EVT_INA_ALERT, FAULT_SIG_INA_ALERT_5V_DISP,
		   FAULT_EDGE_ASSERT);
	expect_evt(&m, FAULT_EVT_EN_FAULT, FAULT_SIG_V_DISP_EN_FAULT,
		   FAULT_EDGE_ASSERT);
	expect_evt(&m, FAULT_EVT_EN_FAULT, FAULT_SIG_PANEL_LED_FAULT,
		   FAULT_EDGE_ASSERT);
	expect_evt(&m, FAULT_EVT_PG_FAULT, FAULT_SIG_PG_5V_PSU,
		   FAULT_EDGE_ASSERT);
}

static void test_polarity_is_uniformly_active_low(void)
{
	model_t m;
	unsigned int i;

	/*
	 * Every signal on both ports asserts low. Driving both words to zero
	 * must assert all 32; driving them to ones must assert none. An
	 * inverted bit anywhere in the table shows up as a hole here.
	 */
	TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFF, FAULT_ACTIVE_LOW_MASK);

	model_init(&m, NULL);
	m.f = 0x0000U;
	m.g = 0x0000U;
	scan_for(&m, 60U);
	TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFF, fault_state(&m.ctx));

	for (i = 0U; i < (unsigned int)FAULT_SIG_COUNT; i++) {
		TEST_ASSERT_TRUE_MESSAGE(
			fault_asserted(&m.ctx, (fault_sig_t)i),
			fault_sig_name((fault_sig_t)i));
	}

	m.f = 0xFFFFU;
	m.g = 0xFFFFU;
	scan_for(&m, 60U);
	TEST_ASSERT_EQUAL_HEX32(0x00000000, fault_state(&m.ctx));

	TEST_ASSERT_FALSE(fault_asserted(&m.ctx, (fault_sig_t)FAULT_SIG_COUNT));
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_init_validates_and_defaults);
	RUN_TEST(test_signal_table_covers_every_pin_of_both_ports);
	RUN_TEST(test_documented_masks_match_the_signal_assignments);
	RUN_TEST(test_idle_ports_produce_nothing);
	RUN_TEST(test_null_context_is_handled_everywhere);

	RUN_TEST(test_button_press_commits_after_the_debounce_interval);
	RUN_TEST(test_a_bouncing_contact_yields_exactly_one_event);
	RUN_TEST(test_a_sub_debounce_glitch_yields_nothing);
	RUN_TEST(test_debounce_restarts_on_every_bounce);
	RUN_TEST(test_each_class_uses_its_own_interval);
	RUN_TEST(test_touch_int_has_no_debounce_and_reports_only_the_assert);
	RUN_TEST(test_ina_alert_needs_two_consecutive_samples);
	RUN_TEST(test_ina_sample_gate_is_configurable);

	RUN_TEST(test_long_press_fires_once_at_the_threshold);
	RUN_TEST(test_auto_repeat_starts_after_one_second_at_four_hertz);
	RUN_TEST(test_repeat_never_bursts_when_scans_are_skipped);
	RUN_TEST(test_the_encoder_switch_behaves_as_a_button);
	RUN_TEST(test_buttons_are_independent);

	RUN_TEST(test_pg_drop_and_recover_emit_distinct_events);
	RUN_TEST(test_all_eight_pg_rails_are_usable_with_no_masking);
	RUN_TEST(test_all_nine_ina_alerts_dispatch_including_the_one_on_port_f);
	RUN_TEST(test_en_fault_flags_raise_their_alarms);
	RUN_TEST(test_proximity_reports_both_directions);
	RUN_TEST(test_backup_rail_power_good_is_reported_and_alarmed);
	RUN_TEST(test_a_rail_already_down_at_boot_raises_its_fault);

	RUN_TEST(test_queue_drops_the_newest_and_counts_it);
	RUN_TEST(test_queue_wraps_correctly_under_sustained_traffic);

	RUN_TEST(test_alarm_latch_records_first_time_and_count);
	RUN_TEST(test_clearing_a_live_alarm_is_refused);
	RUN_TEST(test_clear_all_spares_the_still_active);
	RUN_TEST(test_scan_signal_alarms_share_the_signal_numbering);

	RUN_TEST(test_relay_default_mask_covers_the_timing_chain);
	RUN_TEST(test_relay_drops_on_a_disqualifying_alarm_and_returns);
	RUN_TEST(test_relay_mask_is_replaceable);

	RUN_TEST(test_rgb_priority_truth_table);
	RUN_TEST(test_rgb_named_cases);

	RUN_TEST(test_millisecond_wrap_is_transparent);
	RUN_TEST(test_simultaneous_transitions_keep_their_identity);
	RUN_TEST(test_polarity_is_uniformly_active_low);

	return UNITY_END();
}
