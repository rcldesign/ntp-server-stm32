/*
 * STS1000 "Meridian" — core/ui unit tests.
 *
 * Three kinds of test live here, and they are deliberately different in style:
 *
 *   1. Surface primitives. Clipping, diffing and the golden-row accessor are
 *      pure functions with sharp edges (off-grid rows, geometry mismatch,
 *      attribute-only changes), so they get exhaustive argument coverage.
 *
 *   2. Navigation. These are scenario tests: a walk of the whole §6.2 tree
 *      asserting that every page is reachable and that Left/Back pops back out,
 *      the two guarded flows (single confirm for reboot, double for factory
 *      reset) driven key by key, and the §6.4 wake/sleep policy driven by the
 *      tick.
 *
 *   3. Rendering. Golden text rows for Home built from the documented layout
 *      rule (margin 1, two columns of (cols-2)/2 with a one-column gutter),
 *      plus a sweep that renders every page against both a populated and an
 *      empty health snapshot, on both a realistic 60x20 grid and a 4x16 grid
 *      too small for any of the layouts.
 *
 * The synthetic quality block in mk_quality() is the same one used to derive
 * the golden rows; every number in the expected strings traces to a field
 * there. Where a value is transformed (root dispersion is stored as an NTP
 * 16.16 short and therefore quantised) the test asserts the transformed value,
 * not the input.
 */

#include <errno.h>
#include <math.h>
#include <string.h>

#include "unity.h"

#include "test_support.h"
#include "ui/ui.h"

/* ------------------------------------------------------------- fixtures */

#define ROWS 20u
#define COLS 60u

static char s_ch[UI_SURF_MAX_ROWS * UI_SURF_MAX_COLS];
static uint8_t s_attr[UI_SURF_MAX_ROWS * UI_SURF_MAX_COLS];
static char s_ch2[UI_SURF_MAX_ROWS * UI_SURF_MAX_COLS];
static uint8_t s_attr2[UI_SURF_MAX_ROWS * UI_SURF_MAX_COLS];

static ui_surface_t surf;
static ui_surface_t surf2;
static ui_ctx_t ctx;
static ui_health_t hp; /* populated */
static ui_health_t he; /* empty */
static quality_block_t qb;

static const char *alarm_name(uint16_t id)
{
	switch (id) {
	case 23:
		return "PG_POE";
	case 33:
		return "GNSS_LOST";
	case 41:
		return "THERMAL_WARN";
	default:
		return "OTHER";
	}
}

static void mk_quality(quality_block_t *q)
{
	quality_block_init(q);
	q->stratum = QUALITY_STRATUM_PRIMARY;
	q->lock_state = (uint8_t)QUALITY_LOCK_LOCKED;
	q->active_ref = (uint8_t)QUALITY_REF_OCXO;
	q->gnss_fix = (uint8_t)QUALITY_GNSS_TIME_ONLY;
	q->gnss_sv_used = 12u;
	q->gnss_sv_visible = 18u;
	q->refid = quality_refid('G', 'P', 'S', 0);
	q->root_disp_q16 = quality_ntp_short_from_ns(1234000);
	q->last_pps_off_ns = -12;
	q->pps_off_sigma_ns = 3.4f;
	q->freq_err_ppb = -0.031f;
	q->vc_cmd_mv = 1650;
	q->vc_sense_mv = 1648;
	q->dac_code = 2048u;
	q->adev_1s = 1.2e-11f;
	q->adev_10s = 4.0e-12f;
	q->adev_100s = 9.5e-13f;
	q->gnss_tacc_ns = 25u;
	q->osc_temp_mc = 55000;
	q->flags = QUALITY_FLAG_OCXO_WARM | QUALITY_FLAG_GNSS_TIME_LOCKED;
	q->holdover_t_demote_s = UINT32_MAX;
}

static void mk_health(ui_health_t *h)
{
	static const char *const rn[6] = {"V_POE", "3V3",  "3V3_STM",
					  "5V_DISP", "OCXO", "VCC_RB"};
	static const uint32_t mv[6] = {53200u, 3301u, 3299u, 5010u, 5020u, 15020u};
	static const int32_t ma[6] = {210, 405, 92, 180, 640, 1250};
	unsigned int i;

	memset(h, 0, sizeof(*h));
	h->utc.valid = true;
	h->utc.year = 2026u;
	h->utc.mon = 7u;
	h->utc.day = 27u;
	h->utc.hour = 12u;
	h->utc.min = 34u;
	h->utc.sec = 56u;
	h->uptime_s = (3u * 86400u) + (4u * 3600u) + (12u * 60u) + 33u;
	strcpy(h->hostname, "meridian-01");
	strcpy(h->fw_version, "1.0.0+42");
	strcpy(h->serial, "STS1000-000123");

	h->link_up = true;
	h->dhcp = true;
	h->link_mbps = 100u;
	strcpy(h->ipv4, "192.168.10.50");
	strcpy(h->ipv6, "fd00::1");
	h->ntp_req_per_s = 42u;
	h->ntp_served = 1234567u;
	h->ntp_dropped = 3u;
	h->nts_enabled = true;
	h->nts_sessions = 4u;
	h->ptp_state = (uint8_t)UI_PTP_MASTER;
	h->ptp_clock_class = 6u;
	h->ptp_clients = 7u;

	h->ant_state = (uint8_t)UI_ANT_OK;
	h->sv_count = 10u;
	for (i = 0u; i < 10u; i++) {
		h->sv[i].sys = (uint8_t)(i % 4u);
		h->sv[i].svid = (uint8_t)(1u + (i * 3u));
		h->sv[i].elev_deg = (int8_t)(70 - ((int)i * 8));
		h->sv[i].azim_deg = (int16_t)(15 + ((int)i * 33));
		h->sv[i].cno_dbhz = (uint8_t)(48u - (i * 2u));
		h->sv[i].used = (i < 7u);
	}

	h->rb_present = true;
	h->rb_powered = true;
	h->rb_locked = true;
	h->rb_temp_mc = 56250;
	h->extref_ok = true;
	h->extref_hz = 10000000u;

	h->rail_count = 6u;
	for (i = 0u; i < 6u; i++) {
		strcpy(h->rail[i].name, rn[i]);
		h->rail[i].mv = mv[i];
		h->rail[i].ma = ma[i];
		h->rail[i].pg = true;
	}
	h->rail[3].pg = false;
	h->rail[5].alert = true;

	h->temp_enclosure_mc = 41500;
	h->temp_osc_mc = 55000;
	h->temp_die_mc = 47250;
	h->fan_duty_pct = 35u;
	h->fan_rpm = 2450u;
	h->poe_mw = 13400u;
	h->poe_class = 4u;
	h->supercap_stm_mv = 2700u;
	h->supercap_gps_mv = 2650u;
	h->bkp_stm_pg = true;
	h->bkp_gps_pg = true;

	h->alarm_count = 3u;
	h->alarm[0].id = 23u;
	h->alarm[0].active = true;
	h->alarm[0].latched = true;
	h->alarm[0].count = 3u;
	h->alarm[0].first_s = 252u;
	h->alarm[1].id = 33u;
	h->alarm[1].latched = true;
	h->alarm[1].count = 1u;
	h->alarm[1].first_s = 3661u;
	h->alarm[2].id = 41u;
	h->alarm_name = alarm_name;
}

void setUp(void)
{
	TEST_ASSERT_EQUAL_INT(0, ui_init(&ctx, NULL));
	TEST_ASSERT_EQUAL_INT(0, ui_surface_init(&surf, (uint8_t)ROWS,
						 (uint8_t)COLS, s_ch, s_attr,
						 sizeof(s_ch)));
	TEST_ASSERT_EQUAL_INT(0, ui_surface_init(&surf2, (uint8_t)ROWS,
						 (uint8_t)COLS, s_ch2, s_attr2,
						 sizeof(s_ch2)));
	mk_quality(&qb);
	mk_health(&hp);
	memset(&he, 0, sizeof(he));
}

void tearDown(void)
{
}

/* --------------------------------------------------------------- helpers */

static char rowbuf[UI_SURF_MAX_COLS + 1u];

static const char *row(uint8_t r)
{
	(void)ui_surface_row_text(&surf, r, rowbuf, sizeof(rowbuf));
	return rowbuf;
}

/** Post one input of @p kind with no payload. */
static void key(uint8_t kind)
{
	ui_input_t in;

	memset(&in, 0, sizeof(in));
	in.kind = kind;
	TEST_ASSERT_EQUAL_INT(0, ui_input(&ctx, &in));
}

static void encoder(int16_t d)
{
	ui_input_t in;

	memset(&in, 0, sizeof(in));
	in.kind = (uint8_t)UI_IN_ENCODER;
	in.delta = d;
	TEST_ASSERT_EQUAL_INT(0, ui_input(&ctx, &in));
}

static void touch(uint16_t x, uint16_t y)
{
	ui_input_t in;

	memset(&in, 0, sizeof(in));
	in.kind = (uint8_t)UI_IN_TOUCH;
	in.x = x;
	in.y = y;
	TEST_ASSERT_EQUAL_INT(0, ui_input(&ctx, &in));
}

static void lamp(bool held)
{
	ui_input_t in;

	memset(&in, 0, sizeof(in));
	in.kind = (uint8_t)UI_IN_LAMP;
	in.delta = held ? 1 : 0;
	TEST_ASSERT_EQUAL_INT(0, ui_input(&ctx, &in));
}

static void render(const ui_health_t *h)
{
	TEST_ASSERT_EQUAL_INT(0, ui_render(&ctx, &qb, h, &surf));
}

/** Pop the next action, asserting it exists. */
static ui_action_t next_action(void)
{
	ui_action_t a;

	TEST_ASSERT_EQUAL_INT(0, ui_action_get(&ctx, &a));
	return a;
}

static void assert_no_actions(void)
{
	ui_action_t a;

	TEST_ASSERT_EQUAL_INT(-EAGAIN, ui_action_get(&ctx, &a));
}

/**
 * Expected text of a row that holds one left-aligned and one right-aligned
 * field across the full width, e.g. the header. Encodes the layout rule, not
 * the implementation: a mistake in ui.c's arithmetic changes the padding and
 * the comparison fails.
 */
static char lrbuf[UI_SURF_MAX_COLS + 1u];

static const char *row_lr(const char *left, const char *right)
{
	size_t l = strlen(left);
	size_t r = strlen(right);
	size_t i;

	TEST_ASSERT_TRUE(l + r <= COLS);
	memset(lrbuf, ' ', COLS);
	memcpy(lrbuf, left, l);
	memcpy(&lrbuf[COLS - r], right, r);
	i = COLS;
	while (i > 0u && lrbuf[i - 1u] == ' ') {
		i--;
	}
	lrbuf[i] = '\0';
	return lrbuf;
}

/**
 * Expected text of a two-column key/value row: margin(1) | A(29) | gutter(1) |
 * B(29) at 60 columns, values right-aligned in their column.
 */
static const char *row_kv(const char *la, const char *va, const char *lb,
			  const char *vb)
{
	const size_t half = (COLS - 2u) / 2u;
	size_t i;

	memset(lrbuf, ' ', COLS);
	memcpy(&lrbuf[1], la, strlen(la));
	memcpy(&lrbuf[1u + half - strlen(va)], va, strlen(va));
	memcpy(&lrbuf[2u + half], lb, strlen(lb));
	memcpy(&lrbuf[2u + half + half - strlen(vb)], vb, strlen(vb));
	i = COLS;
	while (i > 0u && lrbuf[i - 1u] == ' ') {
		i--;
	}
	lrbuf[i] = '\0';
	return lrbuf;
}

/* ===================================================================== *
 *  Surface primitives
 * ===================================================================== */

static void test_surface_init_validates(void)
{
	ui_surface_t s;

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ui_surface_init(NULL, 4, 4, s_ch, s_attr, 16));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ui_surface_init(&s, 4, 4, NULL, s_attr, 16));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ui_surface_init(&s, 4, 4, s_ch, NULL, 16));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ui_surface_init(&s, 0, 4, s_ch, s_attr, 16));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ui_surface_init(&s, 4, 0, s_ch, s_attr, 16));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ui_surface_init(&s, (uint8_t)(UI_SURF_MAX_ROWS + 1u),
					      4, s_ch, s_attr, 4096));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ui_surface_init(&s, 4,
					      (uint8_t)(UI_SURF_MAX_COLS + 1u),
					      s_ch, s_attr, 4096));
	/* cells short by one */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ui_surface_init(&s, 4, 4, s_ch, s_attr, 15));

	TEST_ASSERT_EQUAL_INT(0, ui_surface_init(&s, 4, 4, s_ch, s_attr, 16));
	TEST_ASSERT_EQUAL_UINT8(4, s.rows);
	TEST_ASSERT_EQUAL_UINT8(4, s.cols);
	TEST_ASSERT_EQUAL_UINT8(8, s.cell_w);
	TEST_ASSERT_EQUAL_UINT8(16, s.cell_h);
	TEST_ASSERT_EQUAL_UINT8(0, s.hint_count);
}

static void test_surface_put_clips(void)
{
	/* Fits. */
	TEST_ASSERT_EQUAL_size_t(5u, ui_surface_put(&surf, 3u, 0u, "hello",
						    UI_ATTR_NORMAL));
	TEST_ASSERT_EQUAL_STRING("hello", row(3u));

	/* Truncated by the right edge. */
	TEST_ASSERT_EQUAL_size_t(2u, ui_surface_put(&surf, 3u,
						    (uint8_t)(COLS - 2u), "abc",
						    UI_ATTR_NORMAL));

	/* Off-grid row and column are rejected, not clamped. */
	TEST_ASSERT_EQUAL_size_t(0u, ui_surface_put(&surf, (uint8_t)ROWS, 0u,
						    "x", UI_ATTR_NORMAL));
	TEST_ASSERT_EQUAL_size_t(0u, ui_surface_put(&surf, 0u, (uint8_t)COLS,
						    "x", UI_ATTR_NORMAL));
	TEST_ASSERT_EQUAL_size_t(0u, ui_surface_put(&surf, 0u, 0u, NULL,
						    UI_ATTR_NORMAL));
	TEST_ASSERT_EQUAL_size_t(0u, ui_surface_put(NULL, 0u, 0u, "x",
						    UI_ATTR_NORMAL));

	/* putn honours the byte cap. */
	TEST_ASSERT_EQUAL_size_t(3u, ui_surface_putn(&surf, 5u, 1u, "abcdef", 3u,
						     UI_ATTR_ACCENT));
	TEST_ASSERT_EQUAL_STRING(" abc", row(5u));
	TEST_ASSERT_EQUAL_UINT8(UI_ATTR_ACCENT, surf.attr[(5u * COLS) + 1u]);
}

static void test_surface_fill_and_clear(void)
{
	TEST_ASSERT_EQUAL_size_t(4u,
				 ui_surface_fill(&surf, 2u, 0u, 4u, '=',
						 UI_ATTR_DIM));
	TEST_ASSERT_EQUAL_STRING("====", row(2u));

	/* Clipped at the right edge. */
	TEST_ASSERT_EQUAL_size_t(2u, ui_surface_fill(&surf, 2u,
						     (uint8_t)(COLS - 2u), 9u,
						     '#', UI_ATTR_DIM));
	TEST_ASSERT_EQUAL_size_t(0u, ui_surface_fill(&surf, (uint8_t)ROWS, 0u,
						     1u, '#', UI_ATTR_DIM));
	TEST_ASSERT_EQUAL_size_t(0u, ui_surface_fill(&surf, 0u, (uint8_t)COLS,
						     1u, '#', UI_ATTR_DIM));
	TEST_ASSERT_EQUAL_size_t(0u,
				 ui_surface_fill(NULL, 0u, 0u, 1u, '#', 0u));

	ui_surface_clear(&surf);
	TEST_ASSERT_EQUAL_STRING("", row(2u));
	ui_surface_clear(NULL); /* must not fault */
}

static void test_surface_hints(void)
{
	ui_hint_t h;
	unsigned int i;

	memset(&h, 0, sizeof(h));
	h.kind = (uint8_t)UI_HINT_PROGRESS;
	h.row = 1u;
	h.len = 10u;
	h.value = 500u;

	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_surface_hint(NULL, &h));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_surface_hint(&surf, NULL));

	h.kind = (uint8_t)UI_HINT_NONE;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_surface_hint(&surf, &h));
	h.kind = (uint8_t)UI_HINT__COUNT;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_surface_hint(&surf, &h));

	h.kind = (uint8_t)UI_HINT_PROGRESS;
	for (i = 0u; i < UI_SURF_MAX_HINTS; i++) {
		TEST_ASSERT_EQUAL_INT(0, ui_surface_hint(&surf, &h));
	}
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ui_surface_hint(&surf, &h));
	TEST_ASSERT_EQUAL_UINT32(1u, surf.hint_dropped);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_SURF_MAX_HINTS, surf.hint_count);
}

static void test_surface_bignum_tail(void)
{
	ui_hint_t h;

	memset(&h, 0, sizeof(h));
	h.kind = (uint8_t)UI_HINT_BIGNUM;
	h.row = 4u;
	h.len = 6u;
	TEST_ASSERT_EQUAL_INT(0, ui_surface_hint(&surf, &h));

	TEST_ASSERT_FALSE(ui_surface_is_bignum_tail(&surf, 0u));
	TEST_ASSERT_FALSE(ui_surface_is_bignum_tail(&surf, 4u));
	TEST_ASSERT_TRUE(ui_surface_is_bignum_tail(&surf, 5u));
	TEST_ASSERT_FALSE(ui_surface_is_bignum_tail(&surf, 6u));
	TEST_ASSERT_FALSE(ui_surface_is_bignum_tail(NULL, 5u));
}

static void test_surface_row_text(void)
{
	char small[4];

	(void)ui_surface_put(&surf, 1u, 0u, "abcdef", UI_ATTR_NORMAL);

	/* Trailing blanks trimmed. */
	TEST_ASSERT_EQUAL_STRING("abcdef", row(1u));

	/* Capacity-limited copy stays NUL-terminated. */
	TEST_ASSERT_EQUAL_size_t(3u,
				 ui_surface_row_text(&surf, 1u, small,
						     sizeof(small)));
	TEST_ASSERT_EQUAL_STRING("abc", small);

	/* Bad arguments blank the buffer instead of leaving it undefined. */
	small[0] = 'z';
	TEST_ASSERT_EQUAL_size_t(0u, ui_surface_row_text(&surf, (uint8_t)ROWS,
							 small, sizeof(small)));
	TEST_ASSERT_EQUAL_STRING("", small);
	TEST_ASSERT_EQUAL_size_t(0u, ui_surface_row_text(&surf, 0u, NULL, 4u));
	TEST_ASSERT_EQUAL_size_t(0u, ui_surface_row_text(&surf, 0u, small, 0u));
	TEST_ASSERT_EQUAL_size_t(0u,
				 ui_surface_row_text(NULL, 0u, small,
						     sizeof(small)));
}

static void test_surface_diff_rows(void)
{
	uint32_t mask = 0u;
	ui_surface_t small;
	char sc[16];
	uint8_t sa[16];

	/* No previous frame: every row is dirty. */
	TEST_ASSERT_EQUAL_INT(0, ui_surface_diff_rows(&surf, NULL, &mask));
	TEST_ASSERT_EQUAL_HEX32(((uint32_t)1u << ROWS) - 1u, mask);

	/* Identical frames: nothing dirty. */
	TEST_ASSERT_EQUAL_INT(0, ui_surface_diff_rows(&surf, &surf2, &mask));
	TEST_ASSERT_EQUAL_HEX32(0u, mask);

	/* One character. */
	(void)ui_surface_put(&surf, 7u, 3u, "x", UI_ATTR_NORMAL);
	TEST_ASSERT_EQUAL_INT(0, ui_surface_diff_rows(&surf, &surf2, &mask));
	TEST_ASSERT_EQUAL_HEX32((uint32_t)1u << 7, mask);

	/* An attribute-only change still dirties the row: the rasteriser has
	 * to repaint it in a different colour. */
	ui_surface_clear(&surf);
	surf.attr[(9u * COLS) + 2u] = UI_ATTR_ALARM;
	TEST_ASSERT_EQUAL_INT(0, ui_surface_diff_rows(&surf, &surf2, &mask));
	TEST_ASSERT_EQUAL_HEX32((uint32_t)1u << 9, mask);

	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_surface_diff_rows(NULL, NULL, &mask));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_surface_diff_rows(&surf, NULL, NULL));

	TEST_ASSERT_EQUAL_INT(0, ui_surface_init(&small, 4, 4, sc, sa, 16));
	TEST_ASSERT_EQUAL_INT(-EDOM,
			      ui_surface_diff_rows(&surf, &small, &mask));
}

static void test_surface_row_span(void)
{
	uint8_t c0 = 0u;
	uint8_t c1 = 0u;
	ui_surface_t small;
	char sc[16];
	uint8_t sa[16];

	/* No previous frame: the whole row. */
	TEST_ASSERT_EQUAL_INT(0,
			      ui_surface_row_span(&surf, NULL, 2u, &c0, &c1));
	TEST_ASSERT_EQUAL_UINT8(0u, c0);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)(COLS - 1u), c1);

	/* Identical rows. */
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      ui_surface_row_span(&surf, &surf2, 2u, &c0, &c1));

	/* A narrow change is reported as a narrow span. */
	(void)ui_surface_put(&surf, 2u, 10u, "abc", UI_ATTR_NORMAL);
	TEST_ASSERT_EQUAL_INT(0,
			      ui_surface_row_span(&surf, &surf2, 2u, &c0, &c1));
	TEST_ASSERT_EQUAL_UINT8(10u, c0);
	TEST_ASSERT_EQUAL_UINT8(12u, c1);

	/* Attribute-only. */
	ui_surface_clear(&surf);
	surf.attr[(2u * COLS) + 40u] = UI_ATTR_OK;
	TEST_ASSERT_EQUAL_INT(0,
			      ui_surface_row_span(&surf, &surf2, 2u, &c0, &c1));
	TEST_ASSERT_EQUAL_UINT8(40u, c0);
	TEST_ASSERT_EQUAL_UINT8(40u, c1);

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ui_surface_row_span(NULL, NULL, 0u, &c0, &c1));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ui_surface_row_span(&surf, NULL, 0u, NULL, &c1));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ui_surface_row_span(&surf, NULL, 0u, &c0, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      ui_surface_row_span(&surf, NULL, (uint8_t)ROWS,
						  &c0, &c1));

	TEST_ASSERT_EQUAL_INT(0, ui_surface_init(&small, 4, 4, sc, sa, 16));
	TEST_ASSERT_EQUAL_INT(-EDOM,
			      ui_surface_row_span(&surf, &small, 0u, &c0, &c1));
}

static void test_surface_diff_full_height(void)
{
	ui_surface_t big;
	uint32_t mask = 0u;

	TEST_ASSERT_EQUAL_INT(0,
			      ui_surface_init(&big, (uint8_t)UI_SURF_MAX_ROWS,
					      2u, s_ch, s_attr,
					      sizeof(s_ch)));
	TEST_ASSERT_EQUAL_INT(0, ui_surface_diff_rows(&big, NULL, &mask));
	TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFu, mask);
}

/* ===================================================================== *
 *  Name tables and constants
 * ===================================================================== */

static void test_name_tables(void)
{
	uint8_t i;

	for (i = 0u; i < (uint8_t)UI_PAGE__COUNT; i++) {
		TEST_ASSERT_NOT_NULL(ui_page_name(i));
		TEST_ASSERT_TRUE(strlen(ui_page_name(i)) > 0u);
	}
	TEST_ASSERT_EQUAL_STRING("?", ui_page_name((uint8_t)UI_PAGE__COUNT));

	for (i = 0u; i < (uint8_t)UI_MENU__COUNT; i++) {
		TEST_ASSERT_NOT_NULL(ui_menu_name(i));
	}
	TEST_ASSERT_EQUAL_STRING("?", ui_menu_name((uint8_t)UI_MENU__COUNT));

	for (i = 0u; i < (uint8_t)UI_GNSS__COUNT; i++) {
		TEST_ASSERT_EQUAL_size_t(3u, strlen(ui_gnss_sys_name(i)));
	}
	TEST_ASSERT_EQUAL_STRING("???",
				 ui_gnss_sys_name((uint8_t)UI_GNSS__COUNT));

	TEST_ASSERT_EQUAL_STRING("OK", ui_ant_state_name((uint8_t)UI_ANT_OK));
	TEST_ASSERT_EQUAL_STRING("?", ui_ant_state_name((uint8_t)UI_ANT__COUNT));
	TEST_ASSERT_EQUAL_STRING("MASTER",
				 ui_ptp_state_name((uint8_t)UI_PTP_MASTER));
	TEST_ASSERT_EQUAL_STRING("?", ui_ptp_state_name((uint8_t)UI_PTP__COUNT));
}

static void test_home_destinations(void)
{
	/* Spec §6.2 order, and Home is not one of its own destinations. */
	TEST_ASSERT_EQUAL_INT(UI_PAGE_SKYPLOT, ui_home_dest(0u));
	TEST_ASSERT_EQUAL_INT(UI_PAGE_CLOCKS, ui_home_dest(1u));
	TEST_ASSERT_EQUAL_INT(UI_PAGE_NETWORK, ui_home_dest(2u));
	TEST_ASSERT_EQUAL_INT(UI_PAGE_POWER, ui_home_dest(3u));
	TEST_ASSERT_EQUAL_INT(UI_PAGE_ALARMS, ui_home_dest(4u));
	TEST_ASSERT_EQUAL_INT(UI_PAGE_MENU, ui_home_dest(5u));
	TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME,
			      ui_home_dest((uint8_t)UI_HOME_DEST_COUNT));

	TEST_ASSERT_EQUAL_UINT16(0u, ui_timeout_choices[0]);
	TEST_ASSERT_EQUAL_UINT16(600u,
				 ui_timeout_choices[UI_TIMEOUT_CHOICES - 1u]);
}

/* ===================================================================== *
 *  Lifecycle
 * ===================================================================== */

static void test_init_defaults_and_validation(void)
{
	ui_cfg_t cfg;
	ui_ctx_t c;

	ui_cfg_default(NULL); /* must not fault */
	ui_cfg_default(&cfg);
	TEST_ASSERT_EQUAL_UINT8(60u, cfg.brightness_pct);
	TEST_ASSERT_EQUAL_UINT16(120u, cfg.timeout_s);
	TEST_ASSERT_EQUAL_UINT16(120u, cfg.identify_timeout_s);

	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_init(NULL, &cfg));

	cfg.brightness_pct = 5u; /* below UI_BRIGHTNESS_MIN_PCT */
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_init(&c, &cfg));
	cfg.brightness_pct = 101u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_init(&c, &cfg));

	cfg.brightness_pct = 100u;
	cfg.timeout_s = 30u;
	TEST_ASSERT_EQUAL_INT(0, ui_init(&c, &cfg));
	TEST_ASSERT_EQUAL_UINT8(100u, c.cfg.brightness_pct);
	TEST_ASSERT_EQUAL_UINT16(1000u, ui_backlight_permille(&c));
	TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME, ui_page(&c));
	TEST_ASSERT_EQUAL_UINT8(1u, ui_depth(&c));
	TEST_ASSERT_TRUE(ui_awake(&c));
	TEST_ASSERT_FALSE(ui_identify(&c));
}

static void test_null_accessors(void)
{
	ui_ctx_t zero;

	memset(&zero, 0, sizeof(zero)); /* never ui_init()'d */

	TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME, ui_page(NULL));
	TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME, ui_page(&zero));
	TEST_ASSERT_EQUAL_UINT8(0u, ui_depth(NULL));
	TEST_ASSERT_EQUAL_UINT8(0u, ui_selection(NULL));
	TEST_ASSERT_EQUAL_UINT8(0u, ui_selection(&zero));
	TEST_ASSERT_FALSE(ui_awake(NULL));
	TEST_ASSERT_FALSE(ui_identify(NULL));
	TEST_ASSERT_EQUAL_UINT16(0u, ui_backlight_permille(NULL));
	TEST_ASSERT_EQUAL_UINT16(0u, ui_backlight_permille(&zero));
	TEST_ASSERT_EQUAL_size_t(0u, ui_action_count(NULL));
	TEST_ASSERT_EQUAL_UINT32(0u, ui_action_dropped(NULL));
	ui_action_clear_dropped(NULL);
}

static void test_input_rejects_bad_events(void)
{
	ui_input_t in;
	ui_ctx_t zero;

	memset(&in, 0, sizeof(in));
	in.kind = (uint8_t)UI_IN_NONE;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_input(&ctx, &in));

	in.kind = (uint8_t)UI_IN__COUNT;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_input(&ctx, &in));

	in.kind = (uint8_t)UI_IN_ENTER;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_input(NULL, &in));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_input(&ctx, NULL));

	memset(&zero, 0, sizeof(zero));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_input(&zero, &in));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_tick(&zero, 100u));
}

/* ===================================================================== *
 *  Navigation
 * ===================================================================== */

static void test_every_page_is_reachable_and_back_pops(void)
{
	uint8_t i;

	for (i = 0u; i < (uint8_t)UI_HOME_DEST_COUNT; i++) {
		uint8_t j;

		/* Walk the Home selector round to entry i. */
		TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME, ui_page(&ctx));
		for (j = 0u; j < i; j++) {
			key((uint8_t)UI_IN_DOWN);
		}
		TEST_ASSERT_EQUAL_UINT8(i, ui_selection(&ctx));

		key((uint8_t)UI_IN_ENTER);
		TEST_ASSERT_EQUAL_INT(ui_home_dest(i), ui_page(&ctx));
		TEST_ASSERT_EQUAL_UINT8(2u, ui_depth(&ctx));

		/* Every destination renders. */
		render(&hp);

		key((uint8_t)UI_IN_LEFT);
		TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME, ui_page(&ctx));
		TEST_ASSERT_EQUAL_UINT8(1u, ui_depth(&ctx));

		/* Restore the selector to 0 for the next iteration. */
		for (j = 0u; j < i; j++) {
			key((uint8_t)UI_IN_UP);
		}
	}

	/* Back at the root is a no-op, not an underflow. */
	key((uint8_t)UI_IN_LEFT);
	TEST_ASSERT_EQUAL_UINT8(1u, ui_depth(&ctx));
}

static void test_home_selector_wraps_both_ways(void)
{
	key((uint8_t)UI_IN_UP);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)(UI_HOME_DEST_COUNT - 1u),
				ui_selection(&ctx));
	key((uint8_t)UI_IN_DOWN);
	TEST_ASSERT_EQUAL_UINT8(0u, ui_selection(&ctx));

	/* A large encoder throw wraps rather than saturating. */
	encoder(13); /* 13 mod 6 == 1 */
	TEST_ASSERT_EQUAL_UINT8(1u, ui_selection(&ctx));
	encoder(-13);
	TEST_ASSERT_EQUAL_UINT8(0u, ui_selection(&ctx));
	encoder(0); /* no movement, no crash */
	TEST_ASSERT_EQUAL_UINT8(0u, ui_selection(&ctx));
}

static void test_right_descends_like_enter(void)
{
	key((uint8_t)UI_IN_RIGHT);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_SKYPLOT, ui_page(&ctx));
}

static void test_fn_toggles_the_menu_from_anywhere(void)
{
	key((uint8_t)UI_IN_FN);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_MENU, ui_page(&ctx));
	TEST_ASSERT_EQUAL_UINT8(2u, ui_depth(&ctx));

	key((uint8_t)UI_IN_FN);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME, ui_page(&ctx));
	TEST_ASSERT_EQUAL_UINT8(1u, ui_depth(&ctx));

	/* From a deep page, FN collapses the stack and opens the menu. */
	key((uint8_t)UI_IN_ENTER); /* Home -> Skyplot */
	key((uint8_t)UI_IN_FN);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_MENU, ui_page(&ctx));
	TEST_ASSERT_EQUAL_UINT8(2u, ui_depth(&ctx));
}

static void test_info_ack_and_reset_shortcuts(void)
{
	ui_action_t a;

	key((uint8_t)UI_IN_INFO);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_NETWORK, ui_page(&ctx));

	key((uint8_t)UI_IN_ACK);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_ALARMS, ui_page(&ctx));
	a = next_action();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ACTION_ACK_ALARMS, a.kind);
	TEST_ASSERT_EQUAL_INT32(UI_ALARM_ALL, a.arg);

	key((uint8_t)UI_IN_RESET_HOLD);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_CONFIRM, ui_page(&ctx));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_MENU_REBOOT,
				ctx.stack[ctx.depth - 1u].item);
	assert_no_actions();
}

static void test_lamp_test_is_edge_triggered(void)
{
	ui_action_t a;

	lamp(true);
	a = next_action();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ACTION_LAMP_TEST, a.kind);
	TEST_ASSERT_EQUAL_INT32(1, a.arg);

	lamp(true); /* repeat while held emits nothing */
	assert_no_actions();

	lamp(false);
	a = next_action();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ACTION_LAMP_TEST, a.kind);
	TEST_ASSERT_EQUAL_INT32(0, a.arg);
}

static void test_stack_depth_is_bounded(void)
{
	unsigned int i;

	/* Repeatedly descend into the menu's confirm dialog; the stack must
	 * never exceed UI_STACK_MAX regardless of how it is driven. */
	for (i = 0u; i < 50u; i++) {
		key((uint8_t)UI_IN_FN);
		key((uint8_t)UI_IN_DOWN);
		key((uint8_t)UI_IN_DOWN);
		key((uint8_t)UI_IN_DOWN);
		key((uint8_t)UI_IN_DOWN); /* factory reset */
		key((uint8_t)UI_IN_ENTER);
		key((uint8_t)UI_IN_RIGHT);
		key((uint8_t)UI_IN_ENTER); /* -> second confirmation */
		TEST_ASSERT_TRUE(ui_depth(&ctx) <= (uint8_t)UI_STACK_MAX);
	}
}

static void test_push_at_max_depth_is_ignored(void)
{
	uint8_t before;

	ctx.depth = (uint8_t)UI_STACK_MAX;
	ctx.stack[UI_STACK_MAX - 1u].page = (uint8_t)UI_PAGE_HOME;
	ctx.stack[UI_STACK_MAX - 1u].sel = 0u;
	before = ui_depth(&ctx);

	key((uint8_t)UI_IN_ENTER); /* would push Skyplot */
	TEST_ASSERT_EQUAL_UINT8(before, ui_depth(&ctx));
	TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME, ui_page(&ctx));
}

/* ===================================================================== *
 *  Menu edit flows
 * ===================================================================== */

static void test_brightness_edit_commits(void)
{
	ui_action_t a;

	key((uint8_t)UI_IN_FN);    /* menu */
	key((uint8_t)UI_IN_ENTER); /* brightness -> edit */
	TEST_ASSERT_EQUAL_INT(UI_PAGE_EDIT, ui_page(&ctx));
	TEST_ASSERT_EQUAL_INT32(60, ctx.edit_val);

	encoder(2);
	TEST_ASSERT_EQUAL_INT32(80, ctx.edit_val);
	render(&hp);

	key((uint8_t)UI_IN_ENTER);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_MENU, ui_page(&ctx));
	TEST_ASSERT_EQUAL_UINT8(80u, ctx.cfg.brightness_pct);
	TEST_ASSERT_EQUAL_UINT16(800u, ui_backlight_permille(&ctx));

	a = next_action();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ACTION_SET_BRIGHTNESS, a.kind);
	TEST_ASSERT_EQUAL_INT32(80, a.arg);
}

static void test_brightness_edit_saturates(void)
{
	key((uint8_t)UI_IN_FN);
	key((uint8_t)UI_IN_ENTER);

	encoder(100);
	TEST_ASSERT_EQUAL_INT32((int32_t)UI_BRIGHTNESS_MAX_PCT, ctx.edit_val);
	encoder(-100);
	TEST_ASSERT_EQUAL_INT32((int32_t)UI_BRIGHTNESS_MIN_PCT, ctx.edit_val);

	/* UP/DOWN work as single detents. */
	key((uint8_t)UI_IN_DOWN);
	TEST_ASSERT_EQUAL_INT32((int32_t)(UI_BRIGHTNESS_MIN_PCT +
					  UI_BRIGHTNESS_STEP_PCT),
				ctx.edit_val);
	key((uint8_t)UI_IN_UP);
	TEST_ASSERT_EQUAL_INT32((int32_t)UI_BRIGHTNESS_MIN_PCT, ctx.edit_val);
}

static void test_edit_cancel_discards(void)
{
	key((uint8_t)UI_IN_FN);
	key((uint8_t)UI_IN_ENTER);
	encoder(3);
	TEST_ASSERT_EQUAL_INT32(90, ctx.edit_val);

	key((uint8_t)UI_IN_LEFT); /* back = cancel */
	TEST_ASSERT_EQUAL_INT(UI_PAGE_MENU, ui_page(&ctx));
	TEST_ASSERT_EQUAL_UINT8(60u, ctx.cfg.brightness_pct);
	assert_no_actions();
}

static void test_timeout_edit_walks_the_choice_list(void)
{
	ui_action_t a;

	key((uint8_t)UI_IN_FN);
	key((uint8_t)UI_IN_DOWN);  /* -> blank timeout */
	key((uint8_t)UI_IN_ENTER); /* -> edit */
	TEST_ASSERT_EQUAL_INT32(120, ctx.edit_val); /* index 4 */

	key((uint8_t)UI_IN_DOWN);
	TEST_ASSERT_EQUAL_INT32(300, ctx.edit_val);
	key((uint8_t)UI_IN_DOWN);
	TEST_ASSERT_EQUAL_INT32(600, ctx.edit_val);
	key((uint8_t)UI_IN_DOWN); /* wraps to "never" */
	TEST_ASSERT_EQUAL_INT32(0, ctx.edit_val);
	render(&hp); /* the "never" rendering path */

	key((uint8_t)UI_IN_UP); /* wraps backwards to the longest choice */
	TEST_ASSERT_EQUAL_INT32(600, ctx.edit_val);
	key((uint8_t)UI_IN_DOWN);
	key((uint8_t)UI_IN_DOWN);
	TEST_ASSERT_EQUAL_INT32(15, ctx.edit_val);

	/* RIGHT commits, exactly like ENTER (only a confirm dialog treats the
	 * two differently). */
	key((uint8_t)UI_IN_RIGHT);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_MENU, ui_page(&ctx));
	TEST_ASSERT_EQUAL_UINT16(15u, ctx.cfg.timeout_s);
	a = next_action();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ACTION_SET_TIMEOUT, a.kind);
	TEST_ASSERT_EQUAL_INT32(15, a.arg);
}

static void test_timeout_edit_from_an_unlisted_value(void)
{
	/* A value that is not in the choice list (arrives from cfg or MCP)
	 * must still step somewhere sane rather than looping forever. */
	ctx.cfg.timeout_s = 47u;
	key((uint8_t)UI_IN_FN);
	key((uint8_t)UI_IN_DOWN);
	key((uint8_t)UI_IN_ENTER);
	TEST_ASSERT_EQUAL_INT32(47, ctx.edit_val);

	key((uint8_t)UI_IN_DOWN);
	TEST_ASSERT_EQUAL_INT32((int32_t)ui_timeout_choices[1], ctx.edit_val);
}

static void test_identify_toggles_and_expires(void)
{
	ui_action_t a;
	unsigned int i;

	key((uint8_t)UI_IN_FN);
	key((uint8_t)UI_IN_DOWN);
	key((uint8_t)UI_IN_DOWN);  /* -> identify */
	key((uint8_t)UI_IN_ENTER); /* toggles immediately, no edit page */
	TEST_ASSERT_EQUAL_INT(UI_PAGE_MENU, ui_page(&ctx));
	TEST_ASSERT_TRUE(ui_identify(&ctx));
	a = next_action();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ACTION_IDENTIFY, a.kind);
	TEST_ASSERT_EQUAL_INT32(1, a.arg);

	render(&hp); /* the "Identify ON" menu rendering */

	/* Auto-cancel after identify_timeout_s. */
	for (i = 0u; i < 119u; i++) {
		TEST_ASSERT_EQUAL_INT(0, ui_tick(&ctx, 1000u));
	}
	TEST_ASSERT_TRUE(ui_identify(&ctx));
	TEST_ASSERT_EQUAL_INT(0, ui_tick(&ctx, 1000u));
	TEST_ASSERT_FALSE(ui_identify(&ctx));
	a = next_action();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ACTION_IDENTIFY, a.kind);
	TEST_ASSERT_EQUAL_INT32(0, a.arg);
}

static void test_identify_latches_when_the_timeout_is_zero(void)
{
	unsigned int i;

	ctx.cfg.identify_timeout_s = 0u;
	ctx.cfg.timeout_s = 0u; /* keep the panel awake so ENTER is not eaten */
	key((uint8_t)UI_IN_FN);
	key((uint8_t)UI_IN_DOWN);
	key((uint8_t)UI_IN_DOWN);
	key((uint8_t)UI_IN_ENTER);
	TEST_ASSERT_TRUE(ui_identify(&ctx));

	for (i = 0u; i < 1000u; i++) {
		TEST_ASSERT_EQUAL_INT(0, ui_tick(&ctx, 1000u));
	}
	TEST_ASSERT_TRUE(ui_identify(&ctx));

	/* Selecting it again turns it off. */
	key((uint8_t)UI_IN_ENTER);
	TEST_ASSERT_FALSE(ui_identify(&ctx));
}

static void test_reboot_needs_one_confirmation(void)
{
	ui_action_t a;

	key((uint8_t)UI_IN_FN);
	key((uint8_t)UI_IN_DOWN);
	key((uint8_t)UI_IN_DOWN);
	key((uint8_t)UI_IN_DOWN); /* -> reboot */
	key((uint8_t)UI_IN_ENTER);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_CONFIRM, ui_page(&ctx));
	TEST_ASSERT_EQUAL_UINT8(0u, ui_selection(&ctx)); /* defaults to No */
	render(&hp);

	/* Answering No pops without acting. */
	key((uint8_t)UI_IN_ENTER);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_MENU, ui_page(&ctx));
	assert_no_actions();

	/* Answering Yes acts once and returns Home. */
	key((uint8_t)UI_IN_ENTER);
	key((uint8_t)UI_IN_RIGHT);
	TEST_ASSERT_EQUAL_UINT8(1u, ui_selection(&ctx));
	key((uint8_t)UI_IN_ENTER);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME, ui_page(&ctx));
	TEST_ASSERT_EQUAL_UINT8(1u, ui_depth(&ctx));
	a = next_action();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ACTION_REBOOT, a.kind);
	assert_no_actions();
}

static void test_factory_reset_needs_two_confirmations(void)
{
	ui_action_t a;

	key((uint8_t)UI_IN_FN);
	key((uint8_t)UI_IN_UP); /* wraps to the last item: factory reset */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_MENU_FACTORY_RESET,
				ui_selection(&ctx));
	key((uint8_t)UI_IN_ENTER);

	TEST_ASSERT_EQUAL_INT(UI_PAGE_CONFIRM, ui_page(&ctx));
	TEST_ASSERT_EQUAL_UINT8(0u, ctx.stack[ctx.depth - 1u].stage);
	render(&hp);

	key((uint8_t)UI_IN_RIGHT); /* Yes */
	key((uint8_t)UI_IN_ENTER);

	/* Still nothing has happened: a second dialog is now on top, and it
	 * deliberately starts on "No". */
	assert_no_actions();
	TEST_ASSERT_EQUAL_INT(UI_PAGE_CONFIRM, ui_page(&ctx));
	TEST_ASSERT_EQUAL_UINT8(1u, ctx.stack[ctx.depth - 1u].stage);
	TEST_ASSERT_EQUAL_UINT8(0u, ui_selection(&ctx));
	TEST_ASSERT_EQUAL_UINT8(4u, ui_depth(&ctx));
	render(&hp);

	/* Backing out of the second ask returns to the first, not to Home. */
	key((uint8_t)UI_IN_LEFT);
	TEST_ASSERT_EQUAL_UINT8(0u, ctx.stack[ctx.depth - 1u].stage);
	assert_no_actions();

	/* Full affirmative path. */
	key((uint8_t)UI_IN_RIGHT);
	key((uint8_t)UI_IN_ENTER);
	key((uint8_t)UI_IN_RIGHT);
	key((uint8_t)UI_IN_ENTER);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME, ui_page(&ctx));
	a = next_action();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ACTION_FACTORY_RESET, a.kind);
	assert_no_actions();
}

static void test_confirm_choice_moves_with_the_encoder(void)
{
	key((uint8_t)UI_IN_RESET_HOLD);
	TEST_ASSERT_EQUAL_UINT8(0u, ui_selection(&ctx));
	encoder(1);
	TEST_ASSERT_EQUAL_UINT8(1u, ui_selection(&ctx));
	encoder(1); /* wraps back to No */
	TEST_ASSERT_EQUAL_UINT8(0u, ui_selection(&ctx));
}

static void test_unguarded_confirm_frame_is_inert(void)
{
	/*
	 * A confirm frame can only be created for a guarded item through the
	 * public API. Forcing an unguarded one exercises the defensive default
	 * arms of confirm_fire()/page_confirm(): it must render and dismiss
	 * without emitting anything.
	 */
	ctx.depth = 2u;
	memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
	ctx.stack[1].page = (uint8_t)UI_PAGE_CONFIRM;
	ctx.stack[1].item = (uint8_t)UI_MENU_IDENTIFY;
	ctx.stack[1].sel = 1u;

	render(&hp);
	TEST_ASSERT_EQUAL_STRING(" Identify?", row(2u));

	key((uint8_t)UI_IN_ENTER);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME, ui_page(&ctx));
	assert_no_actions();
}

static void test_unguarded_edit_frame_is_inert(void)
{
	/* Likewise for the edit page: an item with no editable value must not
	 * change anything when stepped or committed. */
	ctx.depth = 2u;
	memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
	ctx.stack[1].page = (uint8_t)UI_PAGE_EDIT;
	ctx.stack[1].item = (uint8_t)UI_MENU_REBOOT;
	ctx.edit_val = 7;

	render(&hp);
	encoder(5);
	TEST_ASSERT_EQUAL_INT32(7, ctx.edit_val);

	key((uint8_t)UI_IN_ENTER);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME, ui_page(&ctx));
	assert_no_actions();
}

/* ===================================================================== *
 *  Wake / sleep policy (spec §6.4)
 * ===================================================================== */

static void test_idle_timeout_blanks_the_backlight_only(void)
{
	unsigned int i;

	TEST_ASSERT_EQUAL_UINT16(600u, ui_backlight_permille(&ctx));

	key((uint8_t)UI_IN_ENTER); /* descend so we can prove the page holds */
	TEST_ASSERT_EQUAL_INT(UI_PAGE_SKYPLOT, ui_page(&ctx));

	for (i = 0u; i < 119u; i++) {
		TEST_ASSERT_EQUAL_INT(0, ui_tick(&ctx, 1000u));
	}
	TEST_ASSERT_TRUE(ui_awake(&ctx));

	TEST_ASSERT_EQUAL_INT(0, ui_tick(&ctx, 1000u));
	TEST_ASSERT_FALSE(ui_awake(&ctx));
	TEST_ASSERT_EQUAL_UINT16(0u, ui_backlight_permille(&ctx));

	/* Sleep is a duty change only: the rendered page is untouched. */
	TEST_ASSERT_EQUAL_INT(UI_PAGE_SKYPLOT, ui_page(&ctx));
	render(&hp);
	TEST_ASSERT_EQUAL_STRING(row_lr("SKY VIEW", "12:34:56 UTC"), row(0u));
}

static void test_a_key_wakes_in_place_and_is_consumed(void)
{
	unsigned int i;

	key((uint8_t)UI_IN_ENTER); /* Home -> Skyplot */
	for (i = 0u; i < 120u; i++) {
		TEST_ASSERT_EQUAL_INT(0, ui_tick(&ctx, 1000u));
	}
	TEST_ASSERT_FALSE(ui_awake(&ctx));

	/* ENTER would normally do something; while asleep it only wakes. */
	key((uint8_t)UI_IN_ENTER);
	TEST_ASSERT_TRUE(ui_awake(&ctx));
	TEST_ASSERT_EQUAL_INT(UI_PAGE_SKYPLOT, ui_page(&ctx));
	TEST_ASSERT_EQUAL_UINT16(600u, ui_backlight_permille(&ctx));
}

static void test_touch_and_prox_wake_to_home(void)
{
	unsigned int i;

	key((uint8_t)UI_IN_FN); /* -> menu */
	for (i = 0u; i < 120u; i++) {
		TEST_ASSERT_EQUAL_INT(0, ui_tick(&ctx, 1000u));
	}
	TEST_ASSERT_FALSE(ui_awake(&ctx));

	touch(100u, 100u);
	TEST_ASSERT_TRUE(ui_awake(&ctx));
	TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME, ui_page(&ctx));
	TEST_ASSERT_EQUAL_UINT8(1u, ui_depth(&ctx));

	/* And the same for the reed switch. */
	key((uint8_t)UI_IN_FN);
	for (i = 0u; i < 120u; i++) {
		TEST_ASSERT_EQUAL_INT(0, ui_tick(&ctx, 1000u));
	}
	key((uint8_t)UI_IN_PROX);
	TEST_ASSERT_TRUE(ui_awake(&ctx));
	TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME, ui_page(&ctx));

	/* Proximity while already awake only defers the blank timer. */
	key((uint8_t)UI_IN_FN);
	key((uint8_t)UI_IN_PROX);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_MENU, ui_page(&ctx));
}

static void test_zero_timeout_never_blanks(void)
{
	unsigned int i;

	ctx.cfg.timeout_s = 0u;
	for (i = 0u; i < 5000u; i++) {
		TEST_ASSERT_EQUAL_INT(0, ui_tick(&ctx, 1000u));
	}
	TEST_ASSERT_TRUE(ui_awake(&ctx));
	TEST_ASSERT_EQUAL_UINT16(600u, ui_backlight_permille(&ctx));
}

static void test_display_key_cycles_the_backlight(void)
{
	/* Brightness 60 % => 600 permille at Full, 40 %/10 %/0 % of that. */
	TEST_ASSERT_EQUAL_UINT16(600u, ui_backlight_permille(&ctx));
	key((uint8_t)UI_IN_DISPLAY);
	TEST_ASSERT_EQUAL_UINT16(240u, ui_backlight_permille(&ctx));
	key((uint8_t)UI_IN_DISPLAY);
	TEST_ASSERT_EQUAL_UINT16(60u, ui_backlight_permille(&ctx));
	key((uint8_t)UI_IN_DISPLAY);
	TEST_ASSERT_EQUAL_UINT16(0u, ui_backlight_permille(&ctx));
	key((uint8_t)UI_IN_DISPLAY);
	TEST_ASSERT_EQUAL_UINT16(600u, ui_backlight_permille(&ctx));
}

static void test_wake_restores_a_manually_blanked_panel(void)
{
	unsigned int i;

	key((uint8_t)UI_IN_DISPLAY);
	key((uint8_t)UI_IN_DISPLAY);
	key((uint8_t)UI_IN_DISPLAY); /* manual off */
	TEST_ASSERT_EQUAL_UINT16(0u, ui_backlight_permille(&ctx));
	TEST_ASSERT_TRUE(ui_awake(&ctx)); /* still awake: the timer runs */

	for (i = 0u; i < 120u; i++) {
		TEST_ASSERT_EQUAL_INT(0, ui_tick(&ctx, 1000u));
	}
	TEST_ASSERT_FALSE(ui_awake(&ctx));

	key((uint8_t)UI_IN_PROX);
	TEST_ASSERT_EQUAL_UINT16(600u, ui_backlight_permille(&ctx));
}

/* ===================================================================== *
 *  Touch mapping
 * ===================================================================== */

static void test_touch_before_the_first_render_is_dropped(void)
{
	/* No geometry is known yet, so the touch can wake but not navigate. */
	key((uint8_t)UI_IN_FN);
	touch(10u, 10u);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_MENU, ui_page(&ctx));
}

static void test_touch_zones(void)
{
	const uint16_t cell = 16u;

	key((uint8_t)UI_IN_FN); /* -> menu */
	render(&hp);

	/* Body row 3 is menu index 1 (rows 2..) -> select it. */
	touch(0u, 3u * cell);
	TEST_ASSERT_EQUAL_UINT8(1u, ui_selection(&ctx));

	/* Tapping the selected row again activates it. */
	render(&hp);
	touch(0u, 3u * cell);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_EDIT, ui_page(&ctx));

	/* The title bar is Back. */
	render(&hp);
	touch(0u, 0u);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_MENU, ui_page(&ctx));

	/* The footer mirrors MENU: from the menu it returns Home. */
	render(&hp);
	touch(0u, (uint16_t)((ROWS - 1u) * cell));
	TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME, ui_page(&ctx));

	/* ...and from elsewhere it opens the menu. */
	render(&hp);
	touch(0u, (uint16_t)((ROWS - 1u) * cell));
	TEST_ASSERT_EQUAL_INT(UI_PAGE_MENU, ui_page(&ctx));
}

static void test_touch_on_home_pager_opens_the_selection(void)
{
	const uint16_t cell = 16u;

	render(&hp);
	key((uint8_t)UI_IN_DOWN); /* select CLOCKS */
	render(&hp);

	/* A tap in the body does nothing on Home. */
	touch(0u, 5u * cell);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME, ui_page(&ctx));

	/* The pager row opens it. */
	touch(0u, (uint16_t)((ROWS - 2u) * cell));
	TEST_ASSERT_EQUAL_INT(UI_PAGE_CLOCKS, ui_page(&ctx));
}

static void test_touch_beyond_the_panel_is_clamped(void)
{
	key((uint8_t)UI_IN_FN);
	render(&hp);

	/* A y beyond the last row clamps onto the footer row, i.e. MENU. */
	touch(0u, 60000u);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_HOME, ui_page(&ctx));
}

static void test_touch_on_a_scrolling_page_does_not_select(void)
{
	const uint16_t cell = 16u;

	key((uint8_t)UI_IN_DOWN); /* CLOCKS */
	key((uint8_t)UI_IN_ENTER);
	render(&hp);

	touch(0u, 5u * cell);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_CLOCKS, ui_page(&ctx));
	TEST_ASSERT_EQUAL_UINT8(0u, ui_selection(&ctx));
}

/* ===================================================================== *
 *  Action queue
 * ===================================================================== */

static void test_action_queue_drops_the_newest(void)
{
	ui_action_t a;
	unsigned int i;

	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_action_get(NULL, &a));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_action_get(&ctx, NULL));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, ui_action_get(&ctx, &a));

	/* Fill the queue with lamp-test toggles, then one more. */
	for (i = 0u; i < UI_ACTION_QUEUE_LEN; i++) {
		lamp((i % 2u) == 0u);
	}
	TEST_ASSERT_EQUAL_size_t(UI_ACTION_QUEUE_LEN, ui_action_count(&ctx));
	TEST_ASSERT_EQUAL_UINT32(0u, ui_action_dropped(&ctx));

	lamp((UI_ACTION_QUEUE_LEN % 2u) == 0u);
	TEST_ASSERT_EQUAL_size_t(UI_ACTION_QUEUE_LEN, ui_action_count(&ctx));
	TEST_ASSERT_EQUAL_UINT32(1u, ui_action_dropped(&ctx));

	/* The head survived: the oldest request is the one that is kept. */
	a = next_action();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ACTION_LAMP_TEST, a.kind);
	TEST_ASSERT_EQUAL_INT32(1, a.arg);

	ui_action_clear_dropped(&ctx);
	TEST_ASSERT_EQUAL_UINT32(0u, ui_action_dropped(&ctx));
}

static void test_action_timestamps_follow_the_tick(void)
{
	ui_action_t a;

	TEST_ASSERT_EQUAL_INT(0, ui_tick(&ctx, 2500u));
	lamp(true);
	a = next_action();
	TEST_ASSERT_EQUAL_UINT32(2500u, a.mono_ms);
}

/* ===================================================================== *
 *  Rendering
 * ===================================================================== */

static void test_render_rejects_bad_arguments(void)
{
	ui_surface_t unbound;
	ui_ctx_t zero;

	memset(&unbound, 0, sizeof(unbound));
	memset(&zero, 0, sizeof(zero));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_render(NULL, &qb, &hp, &surf));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_render(&ctx, NULL, &hp, &surf));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_render(&ctx, &qb, NULL, &surf));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_render(&ctx, &qb, &hp, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_render(&ctx, &qb, &hp, &unbound));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_render(&zero, &qb, &hp, &surf));
}

static void test_home_golden_rows(void)
{
	render(&hp);

	/* Chrome. */
	TEST_ASSERT_EQUAL_STRING(row_lr("HOME", "12:34:56 UTC"), row(0u));
	TEST_ASSERT_EQUAL_STRING(row_lr("S1  LOCKED  OCXO  SV 12/18", "ALM 1"),
				 row(1u));

	/* Big clock at (2,2); row 3 is its lower half and stays blank. */
	TEST_ASSERT_EQUAL_STRING("  12:34:56", row(2u));
	TEST_ASSERT_EQUAL_STRING("", row(3u));
	TEST_ASSERT_TRUE(ui_surface_is_bignum_tail(&surf, 3u));

	TEST_ASSERT_EQUAL_STRING(row_lr("  2026-07-27", "up 3d 04:12:33"),
				 row(4u));

	/* The two-column grid. Root dispersion is quantised by the NTP 16.16
	 * short format, so 1.234 ms is served back as 1235.962 us. */
	TEST_ASSERT_EQUAL_STRING(row_kv("Stratum", "1", "Reference", "OCXO"),
				 row(6u));
	TEST_ASSERT_EQUAL_STRING(row_kv("Lock", "LOCKED", "Antenna", "OK"),
				 row(7u));
	TEST_ASSERT_EQUAL_STRING(row_kv("SV used", "12/18", "PPS offset",
					"-12 ns"),
				 row(8u));
	TEST_ASSERT_EQUAL_STRING(row_kv("Root disp", "1235.962 us", "Freq err",
					"-0.031 ppb"),
				 row(9u));
	TEST_ASSERT_EQUAL_STRING(row_kv("Holdover", "--", "Demote in", "--"),
				 row(10u));
	TEST_ASSERT_EQUAL_STRING(row_kv("Host", "meridian-01", "IPv4",
					"192.168.10.50"),
				 row(12u));
	TEST_ASSERT_EQUAL_STRING(row_kv("NTP", "42 req/s", "PTP", "MASTER"),
				 row(13u));
	TEST_ASSERT_EQUAL_STRING(" SV in solution 12/18", row(14u));

	/* Pager and footer. */
	TEST_ASSERT_EQUAL_STRING(" SKY   CLOCKS   NET   POWER   ALARMS   MENU",
				 row((uint8_t)(ROWS - 2u)));
	TEST_ASSERT_EQUAL_STRING("ENC select   ENTER open   MENU menu",
				 row((uint8_t)(ROWS - 1u)));
}

static void test_home_hints(void)
{
	uint8_t i;
	bool saw_bignum = false;
	bool saw_bar = false;

	render(&hp);

	for (i = 0u; i < surf.hint_count; i++) {
		if (surf.hint[i].kind == (uint8_t)UI_HINT_BIGNUM) {
			saw_bignum = true;
			TEST_ASSERT_EQUAL_UINT8(2u, surf.hint[i].row);
			TEST_ASSERT_EQUAL_UINT8(2u, surf.hint[i].col);
			TEST_ASSERT_EQUAL_UINT8(8u, surf.hint[i].len);
		} else if (surf.hint[i].kind == (uint8_t)UI_HINT_PROGRESS) {
			saw_bar = true;
			/* 12 of 18 satellites == 666 permille. */
			TEST_ASSERT_EQUAL_UINT16(666u, surf.hint[i].value);
			TEST_ASSERT_EQUAL_UINT8((uint8_t)(COLS - 2u),
						surf.hint[i].len);
		}
	}
	TEST_ASSERT_TRUE(saw_bignum);
	TEST_ASSERT_TRUE(saw_bar);
}

static void test_home_selection_is_marked_on_the_pager(void)
{
	const uint8_t pager = (uint8_t)(ROWS - 2u);

	render(&hp);
	/* " SKY " is inverse when index 0 is selected. */
	TEST_ASSERT_EQUAL_UINT8(UI_ATTR_INVERSE,
				surf.attr[(pager * COLS) + 1u]);
	TEST_ASSERT_EQUAL_UINT8(UI_ATTR_DIM, surf.attr[(pager * COLS) + 7u]);

	key((uint8_t)UI_IN_DOWN);
	render(&hp);
	TEST_ASSERT_EQUAL_UINT8(UI_ATTR_DIM, surf.attr[(pager * COLS) + 1u]);
	TEST_ASSERT_EQUAL_UINT8(UI_ATTR_INVERSE,
				surf.attr[(pager * COLS) + 7u]);
}

static void test_home_without_a_time_or_a_fix(void)
{
	qb.stratum = QUALITY_STRATUM_UNSYNC;
	qb.lock_state = (uint8_t)QUALITY_LOCK_ACQUIRING;
	qb.active_ref = (uint8_t)QUALITY_REF_NONE;
	qb.gnss_sv_used = 0u;
	qb.gnss_sv_visible = 0u;
	qb.last_pps_off_ns = 0;

	render(&he);
	TEST_ASSERT_EQUAL_STRING(row_lr("HOME", "--:--:-- ---"), row(0u));
	TEST_ASSERT_EQUAL_STRING("  --:--:--", row(2u));
	TEST_ASSERT_EQUAL_STRING(row_lr("  ----------", "up 00:00:00"),
				 row(4u));
	TEST_ASSERT_EQUAL_STRING(row_kv("SV used", "0/0", "PPS offset", "0 ns"),
				 row(8u));
	/* No satellites visible: the bar reads zero rather than dividing. */
	TEST_ASSERT_EQUAL_STRING(" SV in solution 0/0", row(14u));
}

static void test_home_in_holdover(void)
{
	qb.holdover = true;
	qb.flags |= QUALITY_FLAG_HOLDOVER;
	qb.lock_state = (uint8_t)QUALITY_LOCK_HOLDOVER;
	qb.stratum = QUALITY_STRATUM_UNSYNC;
	qb.holdover_elapsed_s = 754u;      /* 00:12:34 */
	qb.holdover_t_demote_s = 14400u;   /* 04:00:00 */

	render(&hp);
	TEST_ASSERT_EQUAL_STRING(row_kv("Holdover", "00:12:34", "Demote in",
					"04:00:00"),
				 row(10u));
	TEST_ASSERT_EQUAL_STRING(row_lr("S16  HOLDOVER  OCXO  SV 12/18",
					"ALM 1"),
				 row(1u));
}

static void test_status_line_flags_an_unsynchronised_clock(void)
{
	qb.stratum = QUALITY_STRATUM_UNSYNC;
	qb.holdover = false;
	render(&hp);
	TEST_ASSERT_EQUAL_UINT8(UI_ATTR_ALARM, surf.attr[COLS + 0u]);

	qb.stratum = QUALITY_STRATUM_PRIMARY;
	render(&hp);
	TEST_ASSERT_EQUAL_UINT8(UI_ATTR_OK, surf.attr[COLS + 0u]);
}

/*
 * Put UI_PAGE_SKYPLOT on top and select @p view.
 *
 * The page's default view is the polar plot (spec §6.3); the numeric table is
 * the second view, reached with Right/Enter. Tests that assert table rows select
 * it explicitly.
 */
static void goto_sky(uint8_t view)
{
	ctx.depth = 2u;
	memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
	ctx.stack[1].page = (uint8_t)UI_PAGE_SKYPLOT;
	ctx.sky_view = view;
}

/* The default view is the plot, and it claims the body with a UI_HINT_SKYPLOT. */
/*
 * A hint is a rectangle the glue rasterises straight into the framebuffer, so
 * unlike the text accessors — which clip — it must be validated here. An
 * out-of-range hint accepted at this seam becomes an out-of-bounds write a long
 * way away in src/zephyr/ui/ui_display.c.
 */
static void test_surface_hint_validates_geometry(void)
{
	ui_surface_t s2;
	char ch[4u * 8u];
	uint8_t at[4u * 8u];
	ui_hint_t h;

	TEST_ASSERT_EQUAL_INT(0, ui_surface_init(&s2, 4u, 8u, ch, at, sizeof(ch)));

	/* In-bounds: the whole row, and a single cell at the far corner. */
	memset(&h, 0, sizeof(h));
	h.kind = (uint8_t)UI_HINT_RULE;
	h.row = 3u;
	h.col = 0u;
	h.len = 8u;
	TEST_ASSERT_EQUAL_INT(0, ui_surface_hint(&s2, &h));
	h.col = 7u;
	h.len = 1u;
	TEST_ASSERT_EQUAL_INT(0, ui_surface_hint(&s2, &h));

	/* One past the right edge. */
	h.col = 7u;
	h.len = 2u;
	TEST_ASSERT_EQUAL_INT(-EDOM, ui_surface_hint(&s2, &h));
	h.col = 0u;
	h.len = 9u;
	TEST_ASSERT_EQUAL_INT(-EDOM, ui_surface_hint(&s2, &h));
	h.col = 8u;
	h.len = 0u;
	TEST_ASSERT_EQUAL_INT(-EDOM, ui_surface_hint(&s2, &h));

	/* Off-grid row, including the 8-bit wrap a naive check would miss. */
	h.col = 0u;
	h.len = 1u;
	h.row = 4u;
	TEST_ASSERT_EQUAL_INT(-EDOM, ui_surface_hint(&s2, &h));
	h.row = 255u;
	TEST_ASSERT_EQUAL_INT(-EDOM, ui_surface_hint(&s2, &h));

	/* BIGNUM needs its second row on-grid too. */
	h.kind = (uint8_t)UI_HINT_BIGNUM;
	h.row = 3u;
	TEST_ASSERT_EQUAL_INT(-EDOM, ui_surface_hint(&s2, &h));
	h.row = 2u;
	TEST_ASSERT_EQUAL_INT(0, ui_surface_hint(&s2, &h));

	/* PROGRESS permille is bounded, so the glue cannot be asked to fill 6553%. */
	h.kind = (uint8_t)UI_HINT_PROGRESS;
	h.row = 0u;
	h.value = 1000u;
	TEST_ASSERT_EQUAL_INT(0, ui_surface_hint(&s2, &h));
	h.value = 1001u;
	TEST_ASSERT_EQUAL_INT(-EDOM, ui_surface_hint(&s2, &h));
	h.value = 65535u;
	TEST_ASSERT_EQUAL_INT(-EDOM, ui_surface_hint(&s2, &h));

	/* A refused hint must not consume a slot or bump the dropped counter. */
	TEST_ASSERT_EQUAL_UINT8(4u, s2.hint_count);
	TEST_ASSERT_EQUAL_UINT32(0u, s2.hint_dropped);

	/* The pre-existing argument checks still apply. */
	h.value = 0u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_surface_hint(NULL, &h));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_surface_hint(&s2, NULL));
	h.kind = (uint8_t)UI_HINT_NONE;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_surface_hint(&s2, &h));
	h.kind = (uint8_t)UI_HINT__COUNT;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ui_surface_hint(&s2, &h));

	/* And a full list still reports -ENOSPC, counted. */
	h.kind = (uint8_t)UI_HINT_RULE;
	h.row = 0u;
	h.col = 0u;
	h.len = 1u;
	while (s2.hint_count < (uint8_t)UI_SURF_MAX_HINTS) {
		TEST_ASSERT_EQUAL_INT(0, ui_surface_hint(&s2, &h));
	}
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ui_surface_hint(&s2, &h));
	TEST_ASSERT_EQUAL_UINT32(1u, s2.hint_dropped);
}

static void test_sky_page_plot_view(void)
{
	uint8_t i;
	const ui_hint_t *sky = NULL;

	goto_sky((uint8_t)UI_SKY_VIEW_PLOT);
	render(&hp);

	/* The summary lines are shared by both views. */
	TEST_ASSERT_EQUAL_STRING(" GPS 2/3  GAL 2/3  GLO 2/2  BDS 1/2",
				 row(2u));
	TEST_ASSERT_EQUAL_STRING(" Antenna OK", row(3u));

	for (i = 0u; i < surf.hint_count; i++) {
		if (surf.hint[i].kind == (uint8_t)UI_HINT_SKYPLOT) {
			sky = &surf.hint[i];
		}
	}
	TEST_ASSERT_NOT_NULL(sky);
	/* `len` is a row count for this hint kind, and the plot starts below the
	 * rule at row 4. */
	TEST_ASSERT_EQUAL_UINT8(5u, sky->row);
	TEST_ASSERT_EQUAL_UINT8(0u, sky->col);
	TEST_ASSERT_TRUE(sky->len > 0u);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)UI_SKY_VIEW_PLOT, sky->value);
	/* The body is left blank for the glue to draw over. */
	TEST_ASSERT_EQUAL_STRING("", row(6u));

	/* Right/Enter cycles to the table and back. */
	TEST_ASSERT_EQUAL_INT(0, ui_input(&ctx, &(ui_input_t){
		.kind = (uint8_t)UI_IN_ENTER }));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_SKY_VIEW_TABLE, ctx.sky_view);
	TEST_ASSERT_EQUAL_INT(0, ui_input(&ctx, &(ui_input_t){
		.kind = (uint8_t)UI_IN_ENTER }));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_SKY_VIEW_PLOT, ctx.sky_view);

	/* FN still opens the menu from the sky page, unchanged. */
	TEST_ASSERT_EQUAL_INT(0, ui_input(&ctx, &(ui_input_t){
		.kind = (uint8_t)UI_IN_FN }));
	TEST_ASSERT_EQUAL_INT(UI_PAGE_MENU, ui_page(&ctx));

	TEST_ASSERT_EQUAL_STRING("PLOT", ui_sky_view_name(
					(uint8_t)UI_SKY_VIEW_PLOT));
	TEST_ASSERT_EQUAL_STRING("TABLE", ui_sky_view_name(
					(uint8_t)UI_SKY_VIEW_TABLE));
	TEST_ASSERT_EQUAL_STRING("?", ui_sky_view_name(99u));
}

static void test_sky_page(void)
{
	goto_sky((uint8_t)UI_SKY_VIEW_TABLE);

	render(&hp);
	TEST_ASSERT_EQUAL_STRING(" GPS 2/3  GAL 2/3  GLO 2/2  BDS 1/2",
				 row(2u));
	TEST_ASSERT_EQUAL_STRING(" Antenna OK", row(3u));
	TEST_ASSERT_EQUAL_STRING(" SYS   SVID    AZ    EL   C/N0   USE",
				 row(5u));
	/* Strongest first: index 0 has C/N0 48. */
	TEST_ASSERT_EQUAL_STRING(" GPS      1    15    70     48   YES",
				 row(6u));
	/* Only eight rows, and the eighth is the unused SV at C/N0 34. */
	TEST_ASSERT_EQUAL_STRING(" BDS     22   246    14     34     -",
				 row(13u));
	TEST_ASSERT_EQUAL_STRING("", row(14u));
}

static void test_sky_page_with_no_satellites(void)
{
	goto_sky((uint8_t)UI_SKY_VIEW_TABLE);

	/* Genuinely nothing: no SV table and the quality block agrees. */
	qb.gnss_sv_used = 0u;
	qb.gnss_sv_visible = 0u;
	he.ant_state = (uint8_t)UI_ANT_OPEN;
	render(&he);
	TEST_ASSERT_EQUAL_STRING(" no satellites tracked", row(2u));
	TEST_ASSERT_EQUAL_STRING(" Antenna OPEN", row(3u));
	TEST_ASSERT_EQUAL_STRING(" (no satellite data)", row(6u));
}

static void test_sky_page_falls_back_to_the_quality_counts(void)
{
	/*
	 * The glue can have a locked receiver (used/visible in the quality
	 * block) without a NAV-SAT snapshot to build the table from. The page
	 * must report the counts it does have rather than an empty sky.
	 */
	ctx.depth = 2u;
	memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
	ctx.stack[1].page = (uint8_t)UI_PAGE_SKYPLOT;

	he.ant_state = (uint8_t)UI_ANT_OK;
	qb.gnss_sv_used = 11u;
	qb.gnss_sv_visible = 16u;
	render(&he); /* he has sv_count == 0 */
	TEST_ASSERT_EQUAL_STRING(" SV detail unavailable -- 11 of 16 in "
				 "solution",
				 row(2u));
}

static void test_sky_page_survey_and_odd_constellations(void)
{
	goto_sky((uint8_t)UI_SKY_VIEW_TABLE);

	/* Oversized count and an out-of-range constellation id both have to
	 * be absorbed, not indexed with. */
	hp.sv_count = (uint8_t)(UI_MAX_SV + 8u);
	hp.sv[0].sys = 200u;
	hp.sv[1].elev_deg = -7;
	hp.survey_active = true;
	hp.survey_dur_s = 3600u;
	hp.survey_acc_mm = 1250u;

	render(&hp);
	TEST_ASSERT_EQUAL_STRING(" Antenna OK   Survey 3600 s  acc 1250 mm",
				 row(3u));
	TEST_ASSERT_EQUAL_STRING(" OTH      1    15    70     48   YES",
				 row(6u));
}

static void test_clocks_page(void)
{
	ctx.depth = 2u;
	memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
	ctx.stack[1].page = (uint8_t)UI_PAGE_CLOCKS;

	render(&hp);
	TEST_ASSERT_EQUAL_STRING("OCXO", row(2u));
	TEST_ASSERT_EQUAL_STRING(row_kv("Vc commanded", "1650 mV", "Vc sensed",
					"1648 mV"),
				 row(3u));
	TEST_ASSERT_EQUAL_STRING(row_kv("DAC code", "2048", "Oven temp",
					"55.000 C"),
				 row(4u));
	TEST_ASSERT_EQUAL_STRING(row_kv("ADEV 1 s", "1.20e-11", "ADEV 100 s",
					"9.50e-13"),
				 row(5u));
	TEST_ASSERT_EQUAL_STRING(row_kv("PPS sigma", "3.400 ns", "GNSS tAcc",
					"25 ns"),
				 row(6u));
	TEST_ASSERT_EQUAL_STRING("RUBIDIUM / EXTERNAL", row(8u));
	TEST_ASSERT_EQUAL_STRING(row_kv("Rb power", "ON", "Rb lock", "LOCKED"),
				 row(9u));
	TEST_ASSERT_EQUAL_STRING(row_kv("Rb temp", "56.250 C", "Ext ref",
					"10.000000 MHz"),
				 row(10u));
	TEST_ASSERT_EQUAL_STRING(row_kv("Active ref", "OCXO", "Holdover err",
					"0 ns"),
				 row(11u));
}

static void test_clocks_page_degraded(void)
{
	ctx.depth = 2u;
	memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
	ctx.stack[1].page = (uint8_t)UI_PAGE_CLOCKS;

	qb.flags = QUALITY_FLAG_DAC_FAULT | QUALITY_FLAG_DAC_SAT;
	qb.adev_1s = 0.0f;         /* not yet computed */
	qb.adev_100s = NAN;
	qb.holdover = true;
	qb.holdover_est_err_ns = 12345678901LL; /* exercises the seconds arm */

	hp.rb_present = false;
	hp.rb_powered = false;
	hp.rb_locked = false;
	hp.extref_ok = false;

	render(&hp);
	TEST_ASSERT_EQUAL_STRING(row_kv("ADEV 1 s", "--", "ADEV 100 s", "--"),
				 row(5u));
	TEST_ASSERT_EQUAL_STRING(row_kv("Rb power", "ABSENT", "Rb lock",
					"UNLOCKED"),
				 row(9u));
	TEST_ASSERT_EQUAL_STRING(row_kv("Rb temp", "56.250 C", "Ext ref",
					"not in band"),
				 row(10u));
	TEST_ASSERT_EQUAL_STRING(row_kv("Active ref", "OCXO", "Holdover err",
					"12.345 s"),
				 row(11u));
	/* Vc sense is coloured by the DAC-fault flag. */
	TEST_ASSERT_EQUAL_UINT8(UI_ATTR_ALARM,
				surf.attr[(3u * COLS) + (COLS - 1u)]);
}

static void test_clocks_page_scientific_edges(void)
{
	ctx.depth = 2u;
	memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
	ctx.stack[1].page = (uint8_t)UI_PAGE_CLOCKS;

	/* A value above 1 exercises the down-scaling loop and the positive
	 * exponent; 9.999e-12 exercises the rounding carry. */
	qb.adev_1s = 12.5f;
	qb.adev_100s = 9.999e-12f;
	render(&hp);
	TEST_ASSERT_EQUAL_STRING(row_kv("ADEV 1 s", "1.25e+01", "ADEV 100 s",
					"1.00e-11"),
				 row(5u));

	qb.adev_1s = INFINITY;
	qb.adev_100s = -1.0f;
	render(&hp);
	TEST_ASSERT_EQUAL_STRING(row_kv("ADEV 1 s", "--", "ADEV 100 s", "--"),
				 row(5u));
}

static void test_non_finite_floats_render_as_zero(void)
{
	qb.freq_err_ppb = NAN;
	qb.pps_off_sigma_ns = INFINITY;
	render(&hp);
	TEST_ASSERT_EQUAL_STRING(row_kv("Root disp", "1235.962 us", "Freq err",
					"0.000 ppb"),
				 row(9u));
}

static void test_network_page(void)
{
	ctx.depth = 2u;
	memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
	ctx.stack[1].page = (uint8_t)UI_PAGE_NETWORK;

	render(&hp);
	TEST_ASSERT_EQUAL_STRING(row_kv("Link", "UP 100 Mb/s", "Address",
					"DHCP"),
				 row(2u));
	TEST_ASSERT_EQUAL_STRING(" IPv4   192.168.10.50", row(3u));
	TEST_ASSERT_EQUAL_STRING(" IPv6   fd00::1", row(4u));
	TEST_ASSERT_EQUAL_STRING(" Host   meridian-01", row(5u));
	TEST_ASSERT_EQUAL_STRING(row_kv("NTP rate", "42 req/s", "NTP served",
					"1234567"),
				 row(7u));
	TEST_ASSERT_EQUAL_STRING(row_kv("NTP dropped", "3", "NTS", "4 sess"),
				 row(8u));
	TEST_ASSERT_EQUAL_STRING(row_kv("PTP", "MASTER", "", "class 6, 7 cli"),
				 row(9u));
	TEST_ASSERT_EQUAL_STRING(" F/W    1.0.0+42", row(11u));
	TEST_ASSERT_EQUAL_STRING(" S/N    STS1000-000123", row(12u));
}

static void test_network_page_down(void)
{
	ctx.depth = 2u;
	memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
	ctx.stack[1].page = (uint8_t)UI_PAGE_NETWORK;

	hp.link_up = false;
	hp.dhcp = false;
	hp.nts_enabled = false;
	hp.ntp_dropped = 0u;
	hp.ptp_state = (uint8_t)UI_PTP_DISABLED;

	render(&hp);
	TEST_ASSERT_EQUAL_STRING(row_kv("Link", "DOWN", "Address", "STATIC"),
				 row(2u));
	TEST_ASSERT_EQUAL_STRING(row_kv("NTP dropped", "0", "NTS", "off"),
				 row(8u));
	TEST_ASSERT_EQUAL_STRING(row_kv("PTP", "DISABLED", "",
					"class 6, 7 cli"),
				 row(9u));
}

static void test_power_page(void)
{
	ctx.depth = 2u;
	memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
	ctx.stack[1].page = (uint8_t)UI_PAGE_POWER;

	render(&hp);
	TEST_ASSERT_EQUAL_STRING(" RAIL          VOLTAGE   CURRENT   PG  AL",
				 row(2u));
	TEST_ASSERT_EQUAL_STRING(" V_POE        53.200 V   0.210 A   OK  --",
				 row(3u));
	TEST_ASSERT_EQUAL_STRING(" 5V_DISP       5.010 V   0.180 A   NO  --",
				 row(6u));
	TEST_ASSERT_EQUAL_STRING(" VCC_RB       15.020 V   1.250 A   OK  AL",
				 row(8u));

	/* A bad rail is coloured as an alarm. */
	TEST_ASSERT_EQUAL_UINT8(UI_ATTR_ALARM, surf.attr[(6u * COLS) + 1u]);
	TEST_ASSERT_EQUAL_UINT8(UI_ATTR_NORMAL, surf.attr[(3u * COLS) + 1u]);

	/* The summary block is anchored to the bottom of the body. */
	TEST_ASSERT_EQUAL_STRING(row_kv("Enclosure", "41.500 C", "MCU die",
					"47.250 C"),
				 row(16u));
	TEST_ASSERT_EQUAL_STRING(row_kv("Fan", "35 %, 2450 rpm", "PoE",
					"13.400 W cls 4"),
				 row(17u));
	TEST_ASSERT_EQUAL_STRING(row_kv("Backup STM", "2.700 V", "Backup GPS",
					"2.650 V"),
				 row(18u));
}

static void test_power_page_scroll_is_clamped(void)
{
	uint8_t i;

	key((uint8_t)UI_IN_DOWN);
	key((uint8_t)UI_IN_DOWN);
	key((uint8_t)UI_IN_DOWN); /* -> POWER */
	key((uint8_t)UI_IN_ENTER);
	TEST_ASSERT_EQUAL_INT(UI_PAGE_POWER, ui_page(&ctx));

	/* Fill every rail slot so the list is longer than the window. */
	hp.rail_count = (uint8_t)(UI_MAX_RAILS + 4u); /* deliberately over */
	for (i = 0u; i < (uint8_t)UI_MAX_RAILS; i++) {
		strcpy(hp.rail[i].name, "RAIL");
		hp.rail[i].mv = 1000u + i;
		hp.rail[i].ma = i;
		hp.rail[i].pg = true;
		hp.rail[i].alert = false;
	}

	for (i = 0u; i < 200u; i++) {
		key((uint8_t)UI_IN_DOWN);
	}
	render(&hp);
	/* Clamped to (count - visible); the window is 13 rows and there are
	 * 12 rails, so the clamp is 0 and the first rail is still on screen. */
	TEST_ASSERT_EQUAL_UINT8(0u, ctx.stack[ctx.depth - 1u].scroll);

	for (i = 0u; i < 200u; i++) {
		key((uint8_t)UI_IN_UP);
	}
	render(&hp);
	TEST_ASSERT_EQUAL_UINT8(0u, ctx.stack[ctx.depth - 1u].scroll);
}

static void test_power_page_scrolls_a_short_window(void)
{
	ui_surface_t small;
	static char sc[10u * COLS];
	static uint8_t sa[10u * COLS];
	char line[UI_SURF_MAX_COLS + 1u];
	uint8_t i;

	TEST_ASSERT_EQUAL_INT(0, ui_surface_init(&small, 10u, (uint8_t)COLS, sc,
						 sa, sizeof(sc)));
	ctx.depth = 2u;
	memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
	ctx.stack[1].page = (uint8_t)UI_PAGE_POWER;

	/* Body rows 2..8, summary takes the last four, so the rail window is
	 * two rows and six rails do not fit. */
	TEST_ASSERT_EQUAL_INT(0, ui_render(&ctx, &qb, &hp, &small));
	(void)ui_surface_row_text(&small, 3u, line, sizeof(line));
	TEST_ASSERT_EQUAL_STRING(" V_POE        53.200 V   0.210 A   OK  --",
				 line);

	for (i = 0u; i < 3u; i++) {
		key((uint8_t)UI_IN_DOWN);
	}
	TEST_ASSERT_EQUAL_INT(0, ui_render(&ctx, &qb, &hp, &small));
	(void)ui_surface_row_text(&small, 3u, line, sizeof(line));
	TEST_ASSERT_EQUAL_STRING(" 5V_DISP       5.010 V   0.180 A   NO  --",
				 line);

	/* Scrolling past the end clamps rather than showing blank rows. */
	for (i = 0u; i < 50u; i++) {
		key((uint8_t)UI_IN_DOWN);
	}
	TEST_ASSERT_EQUAL_INT(0, ui_render(&ctx, &qb, &hp, &small));
	TEST_ASSERT_EQUAL_UINT8(4u, ctx.stack[1].scroll);
	(void)ui_surface_row_text(&small, 3u, line, sizeof(line));
	TEST_ASSERT_EQUAL_STRING(" OCXO          5.020 V   0.640 A   OK  --",
				 line);
}

static void test_alarms_page(void)
{
	ui_action_t a;

	key((uint8_t)UI_IN_ACK);
	(void)next_action(); /* the ACK-all raised by the shortcut */
	render(&hp);

	TEST_ASSERT_EQUAL_STRING("> PG_POE                ACTIVE   x3    "
				 "at 00:04:12",
				 row(2u));
	TEST_ASSERT_EQUAL_STRING("  GNSS_LOST             LATCHED  x1    "
				 "at 01:01:01",
				 row(3u));
	TEST_ASSERT_EQUAL_STRING("  THERMAL_WARN          CLEAR    x0    "
				 "at 00:00:00",
				 row(4u));

	/* ENTER acknowledges the selected alarm by id. */
	key((uint8_t)UI_IN_DOWN);
	render(&hp);
	key((uint8_t)UI_IN_ENTER);
	a = next_action();
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ACTION_ACK_ALARMS, a.kind);
	TEST_ASSERT_EQUAL_INT32(33, a.arg);
}

static void test_alarms_page_without_a_name_resolver(void)
{
	ctx.depth = 2u;
	memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
	ctx.stack[1].page = (uint8_t)UI_PAGE_ALARMS;

	hp.alarm_name = NULL;
	render(&hp);
	TEST_ASSERT_EQUAL_STRING("> ALARM 23              ACTIVE   x3    "
				 "at 00:04:12",
				 row(2u));
}

static void test_alarms_page_when_empty(void)
{
	ctx.depth = 2u;
	memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
	ctx.stack[1].page = (uint8_t)UI_PAGE_ALARMS;
	ctx.stack[1].sel = 5u;
	ctx.stack[1].scroll = 3u;

	render(&he);
	TEST_ASSERT_EQUAL_STRING(" No alarms latched.", row(2u));
	TEST_ASSERT_EQUAL_UINT8(0u, ctx.stack[1].sel);
	TEST_ASSERT_EQUAL_UINT8(0u, ctx.stack[1].scroll);

	/* ENTER with nothing selected must not emit an acknowledgement. */
	key((uint8_t)UI_IN_ENTER);
	assert_no_actions();
}

static void test_alarms_page_scrolls_the_selection_into_view(void)
{
	ui_surface_t small;
	static char sc[8u * COLS];
	static uint8_t sa[8u * COLS];
	char line[UI_SURF_MAX_COLS + 1u];
	uint8_t i;

	TEST_ASSERT_EQUAL_INT(0, ui_surface_init(&small, 8u, (uint8_t)COLS, sc,
						 sa, sizeof(sc)));

	hp.alarm_name = NULL;
	hp.alarm_count = (uint8_t)UI_MAX_ALARMS;
	for (i = 0u; i < (uint8_t)UI_MAX_ALARMS; i++) {
		hp.alarm[i].id = (uint16_t)(100u + i);
		hp.alarm[i].active = true;
		hp.alarm[i].latched = true;
		hp.alarm[i].count = 1u;
		hp.alarm[i].first_s = 0u;
	}

	ctx.depth = 2u;
	memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
	ctx.stack[1].page = (uint8_t)UI_PAGE_ALARMS;
	TEST_ASSERT_EQUAL_INT(0, ui_render(&ctx, &qb, &hp, &small));

	/* Five body rows; walking down eight steps must pull the window. */
	for (i = 0u; i < 8u; i++) {
		key((uint8_t)UI_IN_DOWN);
	}
	TEST_ASSERT_EQUAL_INT(0, ui_render(&ctx, &qb, &hp, &small));
	TEST_ASSERT_EQUAL_UINT8(8u, ctx.stack[1].sel);
	TEST_ASSERT_EQUAL_UINT8(4u, ctx.stack[1].scroll);
	(void)ui_surface_row_text(&small, 6u, line, sizeof(line));
	TEST_ASSERT_EQUAL_STRING("> ALARM 108             ACTIVE   x1    "
				 "at 00:00:00",
				 line);

	/* And back up again. */
	for (i = 0u; i < 8u; i++) {
		key((uint8_t)UI_IN_UP);
	}
	TEST_ASSERT_EQUAL_INT(0, ui_render(&ctx, &qb, &hp, &small));
	TEST_ASSERT_EQUAL_UINT8(0u, ctx.stack[1].sel);
	TEST_ASSERT_EQUAL_UINT8(0u, ctx.stack[1].scroll);
}

static void test_alarms_selection_clamps_when_the_list_shrinks(void)
{
	ctx.depth = 2u;
	memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
	ctx.stack[1].page = (uint8_t)UI_PAGE_ALARMS;
	ctx.stack[1].sel = 2u;
	render(&hp);
	TEST_ASSERT_EQUAL_UINT8(2u, ui_selection(&ctx));

	hp.alarm_count = 1u;
	render(&hp);
	TEST_ASSERT_EQUAL_UINT8(0u, ui_selection(&ctx));

	/* An oversized count is truncated to the array bound. */
	hp.alarm_count = (uint8_t)(UI_MAX_ALARMS + 9u);
	render(&hp);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_MAX_ALARMS, ctx.alarm_id_count);
}

static void test_menu_page(void)
{
	key((uint8_t)UI_IN_FN);
	render(&hp);

	TEST_ASSERT_EQUAL_STRING("> Brightness            60 %", row(2u));
	TEST_ASSERT_EQUAL_STRING("  Blank timeout         120 s", row(3u));
	TEST_ASSERT_EQUAL_STRING("  Identify              off", row(4u));
	TEST_ASSERT_EQUAL_STRING("  Reboot                >", row(5u));
	TEST_ASSERT_EQUAL_STRING("  Factory reset         >", row(6u));
	TEST_ASSERT_EQUAL_UINT8(UI_ATTR_INVERSE, surf.attr[(2u * COLS) + 0u]);
	TEST_ASSERT_EQUAL_UINT8(UI_ATTR_NORMAL, surf.attr[(3u * COLS) + 0u]);

	ctx.cfg.timeout_s = 0u;
	render(&hp);
	TEST_ASSERT_EQUAL_STRING("  Blank timeout         never", row(3u));
}

static void test_menu_selection_clamped(void)
{
	key((uint8_t)UI_IN_FN);
	ctx.stack[1].sel = 99u;
	render(&hp);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)(UI_MENU__COUNT - 1u),
				ui_selection(&ctx));
}

static void test_edit_page(void)
{
	key((uint8_t)UI_IN_FN);
	key((uint8_t)UI_IN_ENTER);
	render(&hp);

	TEST_ASSERT_EQUAL_STRING(" Brightness", row(2u));
	TEST_ASSERT_EQUAL_STRING("  60 %", row(4u));
	TEST_ASSERT_TRUE(ui_surface_is_bignum_tail(&surf, 5u));
	TEST_ASSERT_EQUAL_UINT8(2u, surf.hint_count);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_HINT_PROGRESS, surf.hint[1].kind);
	TEST_ASSERT_EQUAL_UINT16(600u, surf.hint[1].value);
}

static void test_confirm_page_text(void)
{
	key((uint8_t)UI_IN_RESET_HOLD);
	render(&hp);
	TEST_ASSERT_EQUAL_STRING(" Reboot?", row(2u));
	TEST_ASSERT_EQUAL_STRING(" The unit restarts and the OCXO must",
				 row(4u));
	TEST_ASSERT_EQUAL_STRING("                    [ No ]      [ Yes ]",
				 row(18u));
	TEST_ASSERT_EQUAL_UINT8(UI_ATTR_INVERSE, surf.attr[(18u * COLS) + 20u]);

	key((uint8_t)UI_IN_RIGHT);
	render(&hp);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)(UI_ATTR_INVERSE | UI_ATTR_ALARM),
				surf.attr[(18u * COLS) + 32u]);
}

static void test_unknown_page_renders_a_placeholder(void)
{
	ctx.depth = 2u;
	memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
	ctx.stack[1].page = (uint8_t)UI_PAGE__COUNT;

	render(&hp);
	TEST_ASSERT_EQUAL_STRING(row_lr("?", "12:34:56 UTC"), row(0u));
	TEST_ASSERT_EQUAL_STRING(" (no such page)", row(2u));
}

static void test_every_page_renders_on_a_tiny_surface(void)
{
	ui_surface_t tiny;
	static char tc[4u * 16u];
	static uint8_t ta[4u * 16u];
	uint8_t page;

	TEST_ASSERT_EQUAL_INT(0,
			      ui_surface_init(&tiny, 4u, 16u, tc, ta, sizeof(tc)));

	for (page = 0u; page < (uint8_t)UI_PAGE__COUNT; page++) {
		ctx.depth = 2u;
		memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
		ctx.stack[1].page = page;
		ctx.stack[1].item = (uint8_t)UI_MENU_FACTORY_RESET;

		TEST_ASSERT_EQUAL_INT(0, ui_render(&ctx, &qb, &hp, &tiny));
		TEST_ASSERT_EQUAL_INT(0, ui_render(&ctx, &qb, &he, &tiny));
	}
}

static void test_every_page_renders_on_a_single_row_surface(void)
{
	ui_surface_t sliver;
	static char lc[1u * 8u];
	static uint8_t la[1u * 8u];
	uint8_t page;

	/* One row: no status line, no footer, no body. Every guard has to
	 * hold, because nothing can be written except the header. */
	TEST_ASSERT_EQUAL_INT(0, ui_surface_init(&sliver, 1u, 8u, lc, la,
						 sizeof(lc)));

	for (page = 0u; page < (uint8_t)UI_PAGE__COUNT; page++) {
		ctx.depth = 2u;
		memset(&ctx.stack[1], 0, sizeof(ctx.stack[1]));
		ctx.stack[1].page = page;
		TEST_ASSERT_EQUAL_INT(0, ui_render(&ctx, &qb, &hp, &sliver));
	}
}

static void test_render_counts_frames(void)
{
	TEST_ASSERT_EQUAL_UINT32(0u, ctx.renders);
	render(&hp);
	render(&hp);
	TEST_ASSERT_EQUAL_UINT32(2u, ctx.renders);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)ROWS, ctx.geom_rows);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)COLS, ctx.geom_cols);
	TEST_ASSERT_EQUAL_UINT8(16u, ctx.geom_cell_h);
}

static void test_render_defaults_a_zero_cell_height(void)
{
	surf.cell_h = 0u;
	render(&hp);
	TEST_ASSERT_EQUAL_UINT8(16u, ctx.geom_cell_h);
}

/* ===================================================================== *
 *  Robustness
 * ===================================================================== */

static void test_input_flood_keeps_the_machine_consistent(void)
{
	test_rng_t rng;
	unsigned int i;

	test_rng_init(&rng, 0xC0FFEEu);

	for (i = 0u; i < 40000u; i++) {
		ui_input_t in;
		uint32_t r = test_rng_u32(&rng);

		memset(&in, 0, sizeof(in));
		in.kind = (uint8_t)(1u + (r % (uint32_t)(UI_IN__COUNT - 1u)));
		in.delta = (int16_t)((int32_t)((r >> 8) % 41u) - 20);
		in.x = (uint16_t)((r >> 13) % 700u);
		in.y = (uint16_t)((r >> 19) % 500u);
		in.dt_ms = (r >> 3) % 400u;

		TEST_ASSERT_EQUAL_INT(0, ui_input(&ctx, &in));

		/* Invariants that must hold after *every* event. */
		TEST_ASSERT_TRUE(ctx.depth >= 1u);
		TEST_ASSERT_TRUE(ctx.depth <= (uint8_t)UI_STACK_MAX);
		TEST_ASSERT_TRUE(ctx.stack[ctx.depth - 1u].page <
				 (uint8_t)UI_PAGE__COUNT);
		TEST_ASSERT_TRUE(ctx.bl_step < (uint8_t)UI_BL__COUNT);
		TEST_ASSERT_TRUE(ui_backlight_permille(&ctx) <= 1000u);
		TEST_ASSERT_TRUE(ctx.cfg.brightness_pct >=
				 (uint8_t)UI_BRIGHTNESS_MIN_PCT);
		TEST_ASSERT_TRUE(ctx.cfg.brightness_pct <=
				 (uint8_t)UI_BRIGHTNESS_MAX_PCT);
		TEST_ASSERT_EQUAL_UINT8(UI_PAGE_HOME, ctx.stack[0].page);

		if ((i % 16u) == 0u) {
			TEST_ASSERT_EQUAL_INT(0,
					      ui_render(&ctx, &qb,
							((i % 32u) == 0u) ? &hp
									  : &he,
							&surf));
		}

		/* Keep the action queue drained so the flood exercises the
		 * emit path rather than sitting at the drop counter. */
		if ((i % 4u) == 0u) {
			ui_action_t a;

			while (ui_action_get(&ctx, &a) == 0) {
			}
		}
	}

	TEST_ASSERT_TRUE(ctx.inputs == 40000u);
}

static void test_flood_of_actions_is_accounted_for(void)
{
	unsigned int i;

	/* Never drained: the queue saturates and every excess is counted. */
	for (i = 0u; i < 100u; i++) {
		lamp((i % 2u) == 0u);
	}
	TEST_ASSERT_EQUAL_size_t(UI_ACTION_QUEUE_LEN, ui_action_count(&ctx));
	TEST_ASSERT_EQUAL_UINT32(100u - UI_ACTION_QUEUE_LEN,
				 ui_action_dropped(&ctx));
}

/* ------------------------------------------------------------------ main */

int main(void)
{
	UNITY_BEGIN();

	/* surface */
	RUN_TEST(test_surface_init_validates);
	RUN_TEST(test_surface_put_clips);
	RUN_TEST(test_surface_fill_and_clear);
	RUN_TEST(test_surface_hints);
	RUN_TEST(test_surface_bignum_tail);
	RUN_TEST(test_surface_row_text);
	RUN_TEST(test_surface_diff_rows);
	RUN_TEST(test_surface_row_span);
	RUN_TEST(test_surface_diff_full_height);

	/* tables and lifecycle */
	RUN_TEST(test_name_tables);
	RUN_TEST(test_home_destinations);
	RUN_TEST(test_init_defaults_and_validation);
	RUN_TEST(test_null_accessors);
	RUN_TEST(test_input_rejects_bad_events);

	/* navigation */
	RUN_TEST(test_every_page_is_reachable_and_back_pops);
	RUN_TEST(test_home_selector_wraps_both_ways);
	RUN_TEST(test_right_descends_like_enter);
	RUN_TEST(test_fn_toggles_the_menu_from_anywhere);
	RUN_TEST(test_info_ack_and_reset_shortcuts);
	RUN_TEST(test_lamp_test_is_edge_triggered);
	RUN_TEST(test_stack_depth_is_bounded);
	RUN_TEST(test_push_at_max_depth_is_ignored);

	/* menu flows */
	RUN_TEST(test_brightness_edit_commits);
	RUN_TEST(test_brightness_edit_saturates);
	RUN_TEST(test_edit_cancel_discards);
	RUN_TEST(test_timeout_edit_walks_the_choice_list);
	RUN_TEST(test_timeout_edit_from_an_unlisted_value);
	RUN_TEST(test_identify_toggles_and_expires);
	RUN_TEST(test_identify_latches_when_the_timeout_is_zero);
	RUN_TEST(test_reboot_needs_one_confirmation);
	RUN_TEST(test_factory_reset_needs_two_confirmations);
	RUN_TEST(test_confirm_choice_moves_with_the_encoder);
	RUN_TEST(test_unguarded_confirm_frame_is_inert);
	RUN_TEST(test_unguarded_edit_frame_is_inert);

	/* wake / sleep */
	RUN_TEST(test_idle_timeout_blanks_the_backlight_only);
	RUN_TEST(test_a_key_wakes_in_place_and_is_consumed);
	RUN_TEST(test_touch_and_prox_wake_to_home);
	RUN_TEST(test_zero_timeout_never_blanks);
	RUN_TEST(test_display_key_cycles_the_backlight);
	RUN_TEST(test_wake_restores_a_manually_blanked_panel);

	/* touch */
	RUN_TEST(test_touch_before_the_first_render_is_dropped);
	RUN_TEST(test_touch_zones);
	RUN_TEST(test_touch_on_home_pager_opens_the_selection);
	RUN_TEST(test_touch_beyond_the_panel_is_clamped);
	RUN_TEST(test_touch_on_a_scrolling_page_does_not_select);

	/* actions */
	RUN_TEST(test_action_queue_drops_the_newest);
	RUN_TEST(test_action_timestamps_follow_the_tick);

	/* rendering */
	RUN_TEST(test_render_rejects_bad_arguments);
	RUN_TEST(test_home_golden_rows);
	RUN_TEST(test_home_hints);
	RUN_TEST(test_home_selection_is_marked_on_the_pager);
	RUN_TEST(test_home_without_a_time_or_a_fix);
	RUN_TEST(test_home_in_holdover);
	RUN_TEST(test_status_line_flags_an_unsynchronised_clock);
	RUN_TEST(test_surface_hint_validates_geometry);
	RUN_TEST(test_sky_page_plot_view);
	RUN_TEST(test_sky_page);
	RUN_TEST(test_sky_page_with_no_satellites);
	RUN_TEST(test_sky_page_falls_back_to_the_quality_counts);
	RUN_TEST(test_sky_page_survey_and_odd_constellations);
	RUN_TEST(test_clocks_page);
	RUN_TEST(test_clocks_page_degraded);
	RUN_TEST(test_clocks_page_scientific_edges);
	RUN_TEST(test_non_finite_floats_render_as_zero);
	RUN_TEST(test_network_page);
	RUN_TEST(test_network_page_down);
	RUN_TEST(test_power_page);
	RUN_TEST(test_power_page_scroll_is_clamped);
	RUN_TEST(test_power_page_scrolls_a_short_window);
	RUN_TEST(test_alarms_page);
	RUN_TEST(test_alarms_page_without_a_name_resolver);
	RUN_TEST(test_alarms_page_when_empty);
	RUN_TEST(test_alarms_page_scrolls_the_selection_into_view);
	RUN_TEST(test_alarms_selection_clamps_when_the_list_shrinks);
	RUN_TEST(test_menu_page);
	RUN_TEST(test_menu_selection_clamped);
	RUN_TEST(test_edit_page);
	RUN_TEST(test_confirm_page_text);
	RUN_TEST(test_unknown_page_renders_a_placeholder);
	RUN_TEST(test_every_page_renders_on_a_tiny_surface);
	RUN_TEST(test_every_page_renders_on_a_single_row_surface);
	RUN_TEST(test_render_counts_frames);
	RUN_TEST(test_render_defaults_a_zero_cell_height);

	/* robustness */
	RUN_TEST(test_input_flood_keeps_the_machine_consistent);
	RUN_TEST(test_flood_of_actions_is_accounted_for);

	return UNITY_END();
}
