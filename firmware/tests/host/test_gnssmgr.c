/*
 * STS1000 "Meridian" — core/gnssmgr unit tests.
 *
 * gnssmgr performs no I/O, so every test here is a scripted conversation: the
 * fake callbacks record the actions the manager requests, and the test feeds
 * back the UBX messages a ZED-F9T would have sent in reply. Time is an
 * argument, so the ACK-timeout and staleness paths run in zero wall-clock time
 * and are exactly reproducible.
 *
 * The emitted frames are not trusted blindly either: valset_get() re-parses
 * each one as a UBX-CFG-VALSET and looks the key up by ID, so a test asserts
 * "the survey-in duration the receiver was told" rather than "some bytes were
 * sent". test_ubx.c owns the wire format itself.
 *
 * Behaviour references: spec §3.7, gnss_antenna_bias_supervisor.md §9.2/§9.3,
 * sts1000_firmware_hardware_interface.md §2 Stage 5 and §10 caution 2.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"
#include "gnssmgr/gnssmgr.h"
#include "ubx/ubx.h"

#include "test_support.h"

/* ------------------------------------------------------------ fake world -- */

typedef struct {
	unsigned int   sends;
	uint8_t        frame[GNSSMGR_TXBUF_SZ];
	size_t         frame_len;
	int            send_rc;

	unsigned int   bias_calls;
	bool           bias_last;

	unsigned int   store_calls;
	gnssmgr_ecef_t store_last;

	unsigned int   alarm_calls;
	bool           alarm_state[GNSSMGR_ALARM_COUNT];
	unsigned int   alarm_edges[GNSSMGR_ALARM_COUNT];
} fake_t;

static fake_t g_fake;
static gnssmgr_t g_mgr;

static int fake_send(void *user, const uint8_t *frame, size_t len)
{
	fake_t *f = (fake_t *)user;

	TEST_ASSERT_NOT_NULL(frame);
	TEST_ASSERT_TRUE(len <= sizeof(f->frame));
	(void)memcpy(f->frame, frame, len);
	f->frame_len = len;
	f->sends++;
	return f->send_rc;
}

static void fake_bias(void *user, bool on)
{
	fake_t *f = (fake_t *)user;

	f->bias_calls++;
	f->bias_last = on;
}

static void fake_store(void *user, const gnssmgr_ecef_t *pos)
{
	fake_t *f = (fake_t *)user;

	f->store_calls++;
	f->store_last = *pos;
}

static void fake_alarm(void *user, gnssmgr_alarm_t id, bool active)
{
	fake_t *f = (fake_t *)user;

	TEST_ASSERT_TRUE((unsigned int)id < (unsigned int)GNSSMGR_ALARM_COUNT);
	/* The manager must only ever report edges. */
	TEST_ASSERT_NOT_EQUAL(f->alarm_state[id], active);
	f->alarm_state[id] = active;
	f->alarm_edges[id]++;
	f->alarm_calls++;
}

static const gnssmgr_cb_t k_cb = {
	.send_ubx = fake_send,
	.set_ant_bias = fake_bias,
	.store_ecef = fake_store,
	.alarm = fake_alarm,
	.user = &g_fake,
};

static void fake_reset(void)
{
	(void)memset(&g_fake, 0, sizeof(g_fake));
}

/* ----------------------------------------------------------- VALSET peek -- */

/**
 * Look up @p key in the VALSET frame the manager last emitted.
 *
 * Walks the key/value list using the width declared by each key, so it also
 * proves the frame is self-consistent: a wrong width would desynchronise the
 * walk and the lookup would fail.
 */
static bool valset_get(const fake_t *f, uint32_t key, uint64_t *val)
{
	size_t plen;
	size_t end;
	size_t i;

	if (f->frame_len < (6U + UBX_VALSET_HDR_LEN + 2U)) {
		return false;
	}
	if ((f->frame[2] != UBX_CLASS_CFG) || (f->frame[3] != UBX_ID_CFG_VALSET)) {
		return false;
	}
	plen = (size_t)f->frame[4] | ((size_t)f->frame[5] << 8);
	end = 6U + plen;
	if ((end + 2U) != f->frame_len) {
		return false;
	}

	for (i = 6U + UBX_VALSET_HDR_LEN; (i + 4U) <= end;) {
		uint32_t k = (uint32_t)f->frame[i] |
			     ((uint32_t)f->frame[i + 1U] << 8) |
			     ((uint32_t)f->frame[i + 2U] << 16) |
			     ((uint32_t)f->frame[i + 3U] << 24);
		int w = ubx_cfg_key_bytes(k);
		int b;

		if (w < 0) {
			return false;
		}
		i += 4U;
		if ((i + (size_t)w) > end) {
			return false;
		}
		if (k == key) {
			uint64_t v = 0U;

			for (b = 0; b < w; b++) {
				v |= (uint64_t)f->frame[i + (size_t)b] << (8 * b);
			}
			*val = v;
			return true;
		}
		i += (size_t)w;
	}
	return false;
}

static uint64_t valset_must_get(const fake_t *f, uint32_t key)
{
	uint64_t v = 0U;
	char msg[48];

	(void)snprintf(msg, sizeof(msg), "key 0x%08lX missing",
		       (unsigned long)key);
	TEST_ASSERT_TRUE_MESSAGE(valset_get(f, key, &v), msg);
	return v;
}

/* Every emitted frame must be a well-formed UBX frame. */
static void assert_frame_well_formed(const fake_t *f)
{
	static uint8_t pbuf[GNSSMGR_TXBUF_SZ];
	ubx_parser_t p;
	unsigned int frames = 0U;
	size_t i;

	TEST_ASSERT_EQUAL_INT(0, ubx_parser_init(&p, pbuf, sizeof(pbuf)));
	for (i = 0U; i < f->frame_len; i++) {
		int rc = ubx_parse_byte(&p, f->frame[i]);

		TEST_ASSERT_TRUE(rc >= 0);
		frames += (rc == 1) ? 1U : 0U;
	}
	TEST_ASSERT_EQUAL_UINT(1U, frames);
}

/* ------------------------------------------------------- message builders -- */

static void deliver(uint8_t cls, uint8_t id, const uint8_t *p, uint16_t len,
		    uint32_t t)
{
	ubx_msg_t m;

	m.cls = cls;
	m.id = id;
	m.len = len;
	m.payload = p;
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_on_msg(&g_mgr, &m, t));
}

static void send_ack(bool ok, uint32_t t)
{
	static const uint8_t payload[2] = {UBX_CLASS_CFG, UBX_ID_CFG_VALSET};

	deliver(UBX_CLASS_ACK, ok ? UBX_ID_ACK_ACK : UBX_ID_ACK_NAK, payload,
		2U, t);
}

static void put_le16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFU);
	p[1] = (uint8_t)(v >> 8);
}

static void put_le32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFU);
	p[1] = (uint8_t)((v >> 8) & 0xFFU);
	p[2] = (uint8_t)((v >> 16) & 0xFFU);
	p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

/** NAV-PVT with the fields the manager reads; everything else zero. */
static void send_pvt(uint32_t itow, uint8_t fix_type, bool fix_ok,
		     uint32_t tacc_ns, bool utc_ok, uint8_t num_sv, uint32_t t)
{
	uint8_t p[UBX_LEN_NAV_PVT];

	(void)memset(p, 0, sizeof(p));
	put_le32(&p[0], itow);
	put_le16(&p[4], 2026U);
	p[6] = 7U;
	p[7] = 27U;
	p[8] = 12U;
	p[9] = 0U;
	p[10] = 0U;
	p[11] = utc_ok ? 0x07U : 0x00U;
	put_le32(&p[12], tacc_ns);
	p[20] = fix_type;
	p[21] = fix_ok ? 0x01U : 0x00U;
	p[23] = num_sv;
	put_le16(&p[76], 120U);

	deliver(UBX_CLASS_NAV, UBX_ID_NAV_PVT, p, sizeof(p), t);
}

/** NAV-SVIN. */
static void send_svin(uint32_t dur, int32_t x, int32_t y, int32_t z,
		      uint32_t acc, bool valid, bool active, uint32_t t)
{
	uint8_t p[UBX_LEN_NAV_SVIN];

	(void)memset(p, 0, sizeof(p));
	put_le32(&p[4], 1000U);
	put_le32(&p[8], dur);
	put_le32(&p[12], (uint32_t)x);
	put_le32(&p[16], (uint32_t)y);
	put_le32(&p[20], (uint32_t)z);
	p[24] = (uint8_t)(int8_t)-7;
	p[25] = (uint8_t)(int8_t)3;
	p[26] = 0U;
	put_le32(&p[28], acc);
	put_le32(&p[32], 4242U);
	p[36] = valid ? 1U : 0U;
	p[37] = active ? 1U : 0U;

	deliver(UBX_CLASS_NAV, UBX_ID_NAV_SVIN, p, sizeof(p), t);
}

/** NAV-TIMELS. */
static void send_timels(int8_t curr_ls, int8_t change, int32_t to_event,
			uint8_t valid, uint32_t t)
{
	uint8_t p[UBX_LEN_NAV_TIMELS];

	(void)memset(p, 0, sizeof(p));
	p[8] = 2U;
	p[9] = (uint8_t)curr_ls;
	p[10] = 2U;
	p[11] = (uint8_t)change;
	put_le32(&p[12], (uint32_t)to_event);
	put_le16(&p[16], 2500U);
	put_le16(&p[18], 7U);
	p[23] = valid;

	deliver(UBX_CLASS_NAV, UBX_ID_NAV_TIMELS, p, sizeof(p), t);
}

/** TIM-TP. */
static void send_tim_tp(uint32_t tow_ms, int32_t qerr_ps, bool qerr_invalid,
			uint32_t t)
{
	uint8_t p[UBX_LEN_TIM_TP];

	(void)memset(p, 0, sizeof(p));
	put_le32(&p[0], tow_ms);
	put_le32(&p[4], 0x40000000UL);
	put_le32(&p[8], (uint32_t)qerr_ps);
	put_le16(&p[12], 2500U);
	p[14] = (uint8_t)(0x03U | (qerr_invalid ? 0x10U : 0x00U));
	p[15] = 0x50U;

	deliver(UBX_CLASS_TIM, UBX_ID_TIM_TP, p, sizeof(p), t);
}

/** MON-RF with one block. */
static void send_mon_rf(uint8_t ant_status, uint8_t ant_power, uint32_t t)
{
	uint8_t p[UBX_MON_RF_HDR_LEN + UBX_MON_RF_BLK_LEN];

	(void)memset(p, 0, sizeof(p));
	p[0] = 0U;
	p[1] = 1U;
	p[4] = 0U;
	p[5] = (uint8_t)UBX_JAMMING_OK;
	p[6] = ant_status;
	p[7] = ant_power;

	deliver(UBX_CLASS_MON, UBX_ID_MON_RF, p, sizeof(p), t);
}

/** NAV-SAT with three SVs: two used, one merely tracked. */
static void send_nav_sat(uint32_t t)
{
	uint8_t p[UBX_NAV_SAT_HDR_LEN + (3U * UBX_NAV_SAT_SV_LEN)];
	uint8_t *sv;

	(void)memset(p, 0, sizeof(p));
	put_le32(&p[0], 1000U);
	p[4] = 1U;
	p[5] = 3U;

	sv = &p[UBX_NAV_SAT_HDR_LEN];
	sv[0] = 0U;   /* GPS */
	sv[1] = 8U;
	sv[2] = 44U;  /* cno */
	put_le32(&sv[8], 0x08U); /* used */

	sv = &p[UBX_NAV_SAT_HDR_LEN + UBX_NAV_SAT_SV_LEN];
	sv[0] = 2U;
	sv[1] = 27U;
	sv[2] = 50U;
	put_le32(&sv[8], 0x08U); /* used */

	sv = &p[UBX_NAV_SAT_HDR_LEN + (2U * UBX_NAV_SAT_SV_LEN)];
	sv[0] = 6U;
	sv[1] = 17U;
	sv[2] = 20U;
	put_le32(&sv[8], 0x00U); /* tracked, not used */

	deliver(UBX_CLASS_NAV, UBX_ID_NAV_SAT, p, sizeof(p), t);
}

/* ---------------------------------------------------------------- set-up -- */

/* Number of VALSETs in a full cold configuration walk. */
#define WALK_STEPS 8U
/* Index (1-based, in send order) of the CFG-TXREADY step. */
#define TXREADY_SEND 7U

static void setup_mgr(const gnssmgr_cfg_t *cfg)
{
	gnssmgr_cfg_t local;

	fake_reset();
	if (cfg == NULL) {
		TEST_ASSERT_EQUAL_INT(0, gnssmgr_cfg_default(&local));
		cfg = &local;
	}
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_init(&g_mgr, cfg, &k_cb));
}

/** Run the whole config walk, ACKing each step. Returns the time after it. */
static uint32_t walk_config(uint32_t t)
{
	unsigned int i;

	TEST_ASSERT_EQUAL_INT(0, gnssmgr_start(&g_mgr, t));
	for (i = 1U; i < WALK_STEPS; i++) {
		TEST_ASSERT_EQUAL_UINT(i, g_fake.sends);
		assert_frame_well_formed(&g_fake);
		t += 10U;
		send_ack(true, t);
	}
	TEST_ASSERT_EQUAL_UINT(WALK_STEPS, g_fake.sends);
	assert_frame_well_formed(&g_fake);
	t += 10U;
	send_ack(true, t);
	return t;
}

/* ------------------------------------------------------------ init / cfg -- */

static void test_cfg_defaults(void)
{
	gnssmgr_cfg_t c;

	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_cfg_default(NULL));
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_cfg_default(&c));

	TEST_ASSERT_EQUAL_UINT32(1500U, c.ack_timeout_ms);
	TEST_ASSERT_EQUAL_UINT8(3U, c.ack_retries);
	TEST_ASSERT_EQUAL_HEX8(UBX_CFG_LAYER_ALL, c.cfg_layers);
	TEST_ASSERT_EQUAL_UINT32(1000000U, c.tp1_period_us);
	TEST_ASSERT_EQUAL_UINT32(1000000U, c.tp2_period_us);
	TEST_ASSERT_EQUAL_UINT16(1000U, c.meas_rate_ms);
	TEST_ASSERT_TRUE(c.txready_enable);
	TEST_ASSERT_EQUAL_UINT32(3600U, c.survey_min_dur_s);
	TEST_ASSERT_EQUAL_UINT32(10000U, c.survey_acc_limit_0p1mm);
	TEST_ASSERT_EQUAL_UINT32(100U, c.tacc_lock_ns);
	TEST_ASSERT_EQUAL_UINT32(250U, c.tacc_unlock_ns);
	TEST_ASSERT_EQUAL_UINT32(5000U, c.ant_open_ua);
	TEST_ASSERT_EQUAL_UINT32(170000U, c.ant_short_ua);
	TEST_ASSERT_EQUAL_UINT8(3U, c.ant_debounce);
	TEST_ASSERT_EQUAL_UINT32(86400U, c.leap_announce_s);
}

static void test_init_rejects(void)
{
	gnssmgr_cfg_t c;
	gnssmgr_cb_t cb = k_cb;

	TEST_ASSERT_EQUAL_INT(0, gnssmgr_cfg_default(&c));

	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_init(NULL, &c, &k_cb));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_init(&g_mgr, NULL, &k_cb));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_init(&g_mgr, &c, NULL));

	cb.send_ubx = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_init(&g_mgr, &c, &cb));
}

/* Start from the defaults, break exactly one field, and expect a refusal. */
#define ASSERT_CFG_REJECTED(mutation, why)                                 \
	do {                                                               \
		gnssmgr_cfg_t c_;                                          \
		TEST_ASSERT_EQUAL_INT(0, gnssmgr_cfg_default(&c_));        \
		mutation;                                                  \
		TEST_ASSERT_EQUAL_INT_MESSAGE(                             \
			-EINVAL, gnssmgr_init(&g_mgr, &c_, &k_cb), why);   \
	} while (0)

/* Every guarded configuration field, one at a time. */
static void test_init_config_validation(void)
{
	ASSERT_CFG_REJECTED(c_.ack_timeout_ms = 0U, "zero ACK timeout");
	ASSERT_CFG_REJECTED(c_.pvt_stale_ms = 0U, "zero staleness window");
	ASSERT_CFG_REJECTED(c_.cfg_layers = 0U, "no configuration layer");
	ASSERT_CFG_REJECTED(c_.cfg_layers = 0xF8U, "no known layer bit");
	ASSERT_CFG_REJECTED(c_.meas_rate_ms = 0U, "zero measurement rate");
	ASSERT_CFG_REJECTED(c_.nav_rate_cycles = 0U, "zero navigation rate");
	ASSERT_CFG_REJECTED(c_.tp1_period_us = 0U, "zero TP1 period");
	ASSERT_CFG_REJECTED(c_.tp2_period_us = 0U, "zero TP2 period");
	ASSERT_CFG_REJECTED(c_.survey_min_dur_s = 0U, "zero survey duration");
	ASSERT_CFG_REJECTED(c_.survey_acc_limit_0p1mm = 0U, "zero survey limit");
	ASSERT_CFG_REJECTED(c_.ant_debounce = 0U, "zero antenna debounce");
	ASSERT_CFG_REJECTED(c_.itow_backstep_ms = 0U, "zero iTOW backstep");
	/* Past half a week the guard cannot tell a restart from a rollover. */
	ASSERT_CFG_REJECTED(c_.itow_backstep_ms = 302400001UL,
			    "iTOW backstep over half a week");

	/* A pulse cannot be as long as, or longer than, its own period. */
	ASSERT_CFG_REJECTED(c_.tp1_len_us = c_.tp1_period_us, "TP1 pulse fills period");
	ASSERT_CFG_REJECTED(c_.tp2_len_us = c_.tp2_period_us + 1U,
			    "TP2 pulse exceeds period");

	/* Unlock tighter than lock inverts the hysteresis and makes the lock
	 * flag chatter once per second on a marginal receiver. */
	ASSERT_CFG_REJECTED(c_.tacc_unlock_ns = c_.tacc_lock_ns - 1U,
			    "inverted lock hysteresis");

	/* The short threshold must sit above the open threshold. */
	ASSERT_CFG_REJECTED(c_.ant_short_ua = c_.ant_open_ua, "short == open");
	ASSERT_CFG_REJECTED(c_.ant_short_ua = 1U, "short below open");

	/* The elevation mask is a signed degree value with real bounds. */
	ASSERT_CFG_REJECTED(c_.min_elev_deg = 91, "elevation above 90");
	ASSERT_CFG_REJECTED(c_.min_elev_deg = -91, "elevation below -90");

	{
		gnssmgr_cfg_t c;

		TEST_ASSERT_EQUAL_INT(0, gnssmgr_cfg_default(&c));
		c.min_elev_deg = -90;
		c.tacc_unlock_ns = c.tacc_lock_ns; /* equal is legal */
		TEST_ASSERT_EQUAL_INT(0, gnssmgr_init(&g_mgr, &c, &k_cb));
	}
}

static void test_init_leaves_manager_idle(void)
{
	gnssmgr_status_t st;
	gnssmgr_leap_t ls;
	gnssmgr_qerr_t q;
	gnssmgr_svin_t sv;
	gnssmgr_sats_t sat;
	gnssmgr_ecef_t pos;

	setup_mgr(NULL);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_IDLE, gnssmgr_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(0U, g_fake.sends);
	TEST_ASSERT_EQUAL_UINT32(0U, gnssmgr_alarms(&g_mgr));
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_UNKNOWN, gnssmgr_ant_get_state(&g_mgr));
	TEST_ASSERT_FALSE(gnssmgr_ant_short_latched(&g_mgr));
	TEST_ASSERT_FALSE(gnssmgr_txready_trusted(&g_mgr));

	TEST_ASSERT_EQUAL_INT(-EAGAIN, gnssmgr_status(&g_mgr, &st));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, gnssmgr_leap(&g_mgr, &ls));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, gnssmgr_qerr_for_pps(&g_mgr, &q));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, gnssmgr_svin(&g_mgr, &sv));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, gnssmgr_sats(&g_mgr, &sat));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, gnssmgr_position(&g_mgr, &pos));
	TEST_ASSERT_EQUAL_INT(GNSSMGR_LI_UNSYNC, gnssmgr_leap_indicator(&g_mgr));
}

static void test_null_arguments(void)
{
	gnssmgr_status_t st;
	ubx_msg_t m;

	setup_mgr(NULL);
	m.cls = UBX_CLASS_NAV;
	m.id = UBX_ID_NAV_PVT;
	m.len = 0U;
	m.payload = NULL;

	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_start(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_notify_reset(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_step(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_on_msg(NULL, &m, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_on_msg(&g_mgr, NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_tick_1hz(NULL, NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_set_stored_ecef(NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_position(NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_request_survey(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_ant_reenable(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_status(NULL, &st));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_status(&g_mgr, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_leap(&g_mgr, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_qerr_for_pps(&g_mgr, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_svin(&g_mgr, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_sats(&g_mgr, NULL));

	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_IDLE, gnssmgr_get_state(NULL));
	TEST_ASSERT_EQUAL_UINT32(0U, gnssmgr_alarms(NULL));
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_UNKNOWN, gnssmgr_ant_get_state(NULL));
	TEST_ASSERT_FALSE(gnssmgr_ant_short_latched(NULL));
	TEST_ASSERT_FALSE(gnssmgr_txready_trusted(NULL));
	TEST_ASSERT_EQUAL_INT(GNSSMGR_LI_UNSYNC, gnssmgr_leap_indicator(NULL));

	/* A message of a known type that fails its typed decode. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG, gnssmgr_on_msg(&g_mgr, &m, 0U));
}

/* -------------------------------------------------------- config walk ---- */

static void test_cold_config_walk_to_survey(void)
{
	uint32_t t = 1000U;

	setup_mgr(NULL);

	/* Step 1 is emitted by start(), before any reply. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_start(&g_mgr, t));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_CONFIG, gnssmgr_get_state(&g_mgr));
	assert_frame_well_formed(&g_fake);
	/* Protocols first: UBX on, NMEA off both ways (spec §3.7). */
	TEST_ASSERT_EQUAL_UINT64(1U, valset_must_get(&g_fake, UBX_CFG_UART1INPROT_UBX));
	TEST_ASSERT_EQUAL_UINT64(0U, valset_must_get(&g_fake, UBX_CFG_UART1INPROT_NMEA));
	TEST_ASSERT_EQUAL_UINT64(1U, valset_must_get(&g_fake, UBX_CFG_UART1OUTPROT_UBX));
	TEST_ASSERT_EQUAL_UINT64(0U, valset_must_get(&g_fake, UBX_CFG_UART1OUTPROT_NMEA));
	/* Persisted to RAM, BBR and flash. */
	TEST_ASSERT_EQUAL_HEX8(UBX_CFG_LAYER_ALL, g_fake.frame[7]);

	/* Step 2: message rates. */
	t += 10U;
	send_ack(true, t);
	TEST_ASSERT_EQUAL_UINT(2U, g_fake.sends);
	TEST_ASSERT_EQUAL_UINT64(1U, valset_must_get(&g_fake,
						     UBX_CFG_MSGOUT_NAV_PVT_UART1));
	TEST_ASSERT_EQUAL_UINT64(1U, valset_must_get(&g_fake,
						     UBX_CFG_MSGOUT_TIM_TP_UART1));
	TEST_ASSERT_EQUAL_UINT64(1U, valset_must_get(&g_fake,
						     UBX_CFG_MSGOUT_MON_RF_UART1));
	TEST_ASSERT_EQUAL_UINT64(1U, valset_must_get(&g_fake,
						     UBX_CFG_MSGOUT_NAV_SVIN_UART1));

	/* Step 3: rate + stationary dynamic model. */
	t += 10U;
	send_ack(true, t);
	TEST_ASSERT_EQUAL_UINT(3U, g_fake.sends);
	TEST_ASSERT_EQUAL_UINT64(1000U, valset_must_get(&g_fake, UBX_CFG_RATE_MEAS));
	TEST_ASSERT_EQUAL_UINT64(UBX_DYNMODEL_STATIONARY,
				 valset_must_get(&g_fake, UBX_CFG_NAVSPG_DYNMODEL));
	TEST_ASSERT_EQUAL_UINT64(10U, valset_must_get(&g_fake,
						      UBX_CFG_NAVSPG_INFIL_MINELEV));

	/* Step 4: constellations. */
	t += 10U;
	send_ack(true, t);
	TEST_ASSERT_EQUAL_UINT(4U, g_fake.sends);
	TEST_ASSERT_EQUAL_UINT64(1U, valset_must_get(&g_fake, UBX_CFG_SIGNAL_GPS_ENA));
	TEST_ASSERT_EQUAL_UINT64(1U, valset_must_get(&g_fake, UBX_CFG_SIGNAL_GAL_ENA));
	TEST_ASSERT_EQUAL_UINT64(1U, valset_must_get(&g_fake, UBX_CFG_SIGNAL_GLO_ENA));
	TEST_ASSERT_EQUAL_UINT64(1U, valset_must_get(&g_fake, UBX_CFG_SIGNAL_BDS_ENA));

	/* Step 5: TIMEPULSE 1 — 1 Hz, UTC grid, locked-only, ToW aligned. */
	t += 10U;
	send_ack(true, t);
	TEST_ASSERT_EQUAL_UINT(5U, g_fake.sends);
	TEST_ASSERT_EQUAL_UINT64(1U, valset_must_get(&g_fake, UBX_CFG_TP_TP1_ENA));
	TEST_ASSERT_EQUAL_UINT64(1000000U,
				 valset_must_get(&g_fake, UBX_CFG_TP_PERIOD_TP1));
	TEST_ASSERT_EQUAL_UINT64(1000000U,
				 valset_must_get(&g_fake, UBX_CFG_TP_PERIOD_LOCK_TP1));
	TEST_ASSERT_EQUAL_UINT64(100000U, valset_must_get(&g_fake, UBX_CFG_TP_LEN_TP1));
	TEST_ASSERT_EQUAL_UINT64(1U, valset_must_get(&g_fake,
						     UBX_CFG_TP_ALIGN_TO_TOW_TP1));
	TEST_ASSERT_EQUAL_UINT64(1U, valset_must_get(&g_fake,
						     UBX_CFG_TP_USE_LOCKED_TP1));
	TEST_ASSERT_EQUAL_UINT64(UBX_TP_TIMEGRID_UTC,
				 valset_must_get(&g_fake, UBX_CFG_TP_TIMEGRID_TP1));
	/* Cable delay is removed in software, not by the receiver (spec §3.2). */
	TEST_ASSERT_EQUAL_UINT64(0U, valset_must_get(&g_fake, UBX_CFG_TP_USER_DELAY_TP1));

	/* Step 6: TIMEPULSE 2 matches TP1 so PC6 can cross-check PA0. */
	t += 10U;
	send_ack(true, t);
	TEST_ASSERT_EQUAL_UINT(6U, g_fake.sends);
	TEST_ASSERT_EQUAL_UINT64(1U, valset_must_get(&g_fake, UBX_CFG_TP_TP2_ENA));
	TEST_ASSERT_EQUAL_UINT64(1000000U,
				 valset_must_get(&g_fake, UBX_CFG_TP_PERIOD_TP2));

	/* Step 7: the TX_READY remap. PD5 is untrusted until this is ACKed. */
	t += 10U;
	send_ack(true, t);
	TEST_ASSERT_EQUAL_UINT(TXREADY_SEND, g_fake.sends);
	TEST_ASSERT_FALSE(gnssmgr_txready_trusted(&g_mgr));
	TEST_ASSERT_EQUAL_UINT64(1U, valset_must_get(&g_fake, UBX_CFG_TXREADY_ENABLED));
	TEST_ASSERT_EQUAL_UINT64(6U, valset_must_get(&g_fake, UBX_CFG_TXREADY_PIN));
	TEST_ASSERT_EQUAL_UINT64(UBX_TXREADY_IF_UART1,
				 valset_must_get(&g_fake, UBX_CFG_TXREADY_INTERFACE));

	/* Step 8: TMODE. No stored position, so survey-in. */
	t += 10U;
	send_ack(true, t);
	TEST_ASSERT_TRUE(gnssmgr_txready_trusted(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(WALK_STEPS, g_fake.sends);
	TEST_ASSERT_EQUAL_UINT64(UBX_TMODE_SURVEY_IN,
				 valset_must_get(&g_fake, UBX_CFG_TMODE_MODE));
	TEST_ASSERT_EQUAL_UINT64(3600U,
				 valset_must_get(&g_fake, UBX_CFG_TMODE_SVIN_MIN_DUR));
	TEST_ASSERT_EQUAL_UINT64(10000U,
				 valset_must_get(&g_fake, UBX_CFG_TMODE_SVIN_ACC_LIMIT));
	/* The fixed-position keys must be absent from a survey-in request. */
	{
		uint64_t v;

		TEST_ASSERT_FALSE(valset_get(&g_fake, UBX_CFG_TMODE_ECEF_X, &v));
	}

	t += 10U;
	send_ack(true, t);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_SURVEY_IN, gnssmgr_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(WALK_STEPS, g_fake.sends); /* nothing extra */
	TEST_ASSERT_EQUAL_UINT32(0U, gnssmgr_alarms(&g_mgr));
}

static void test_stored_position_skips_survey(void)
{
	gnssmgr_ecef_t pos = {
		.x_cm = -2694045L, .y_cm = -4293642L, .z_cm = 3857878L,
		.x_hp = -5, .y_hp = 7, .z_hp = -1,
		.acc_0p1mm = 2500U, .valid = true,
	};
	gnssmgr_ecef_t got;
	uint32_t t = 500U;

	setup_mgr(NULL);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_set_stored_ecef(&g_mgr, &pos));
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_position(&g_mgr, &got));
	TEST_ASSERT_EQUAL_INT32(pos.x_cm, got.x_cm);

	t = walk_config(t);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_FIXED, gnssmgr_get_state(&g_mgr));
	/* Nothing was surveyed, so nothing was stored. */
	TEST_ASSERT_EQUAL_UINT(0U, g_fake.store_calls);

	/* The TMODE frame carries the fixed position verbatim. */
	TEST_ASSERT_EQUAL_UINT64(UBX_TMODE_FIXED,
				 valset_must_get(&g_fake, UBX_CFG_TMODE_MODE));
	TEST_ASSERT_EQUAL_UINT64(UBX_TMODE_POS_ECEF,
				 valset_must_get(&g_fake, UBX_CFG_TMODE_POS_TYPE));
	TEST_ASSERT_EQUAL_HEX32((uint32_t)pos.x_cm,
				(uint32_t)valset_must_get(&g_fake, UBX_CFG_TMODE_ECEF_X));
	TEST_ASSERT_EQUAL_HEX32((uint32_t)pos.y_cm,
				(uint32_t)valset_must_get(&g_fake, UBX_CFG_TMODE_ECEF_Y));
	TEST_ASSERT_EQUAL_HEX32((uint32_t)pos.z_cm,
				(uint32_t)valset_must_get(&g_fake, UBX_CFG_TMODE_ECEF_Z));
	TEST_ASSERT_EQUAL_HEX8((uint8_t)pos.x_hp,
			       (uint8_t)valset_must_get(&g_fake, UBX_CFG_TMODE_ECEF_X_HP));
	TEST_ASSERT_EQUAL_HEX8((uint8_t)pos.z_hp,
			       (uint8_t)valset_must_get(&g_fake, UBX_CFG_TMODE_ECEF_Z_HP));
	/* fixed_pos_acc_0p1mm defaults to 0 = "use the survey's own accuracy". */
	TEST_ASSERT_EQUAL_UINT64(2500U,
				 valset_must_get(&g_fake, UBX_CFG_TMODE_FIXED_POS_ACC));

	/* An invalid position is refused rather than stored. */
	pos.valid = false;
	TEST_ASSERT_EQUAL_INT(-EINVAL, gnssmgr_set_stored_ecef(&g_mgr, &pos));
}

static void test_fixed_pos_acc_override(void)
{
	gnssmgr_cfg_t c;
	gnssmgr_ecef_t pos = {
		.x_cm = 1L, .y_cm = 2L, .z_cm = 3L,
		.x_hp = 0, .y_hp = 0, .z_hp = 0,
		.acc_0p1mm = 2500U, .valid = true,
	};

	TEST_ASSERT_EQUAL_INT(0, gnssmgr_cfg_default(&c));
	c.fixed_pos_acc_0p1mm = 999U;
	setup_mgr(&c);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_set_stored_ecef(&g_mgr, &pos));
	(void)walk_config(100U);
	TEST_ASSERT_EQUAL_UINT64(999U,
				 valset_must_get(&g_fake, UBX_CFG_TMODE_FIXED_POS_ACC));
}

static void test_txready_disabled_is_never_trusted(void)
{
	gnssmgr_cfg_t c;

	TEST_ASSERT_EQUAL_INT(0, gnssmgr_cfg_default(&c));
	c.txready_enable = false;
	setup_mgr(&c);
	(void)walk_config(100U);
	/* The step still runs (it must disable the pin), but PD5 is never
	 * trustworthy because no TX_READY function was requested. */
	TEST_ASSERT_FALSE(gnssmgr_txready_trusted(&g_mgr));
}

static void test_gps_timegrid_option(void)
{
	gnssmgr_cfg_t c;
	uint32_t t = 100U;
	unsigned int i;

	TEST_ASSERT_EQUAL_INT(0, gnssmgr_cfg_default(&c));
	c.tp_utc_timegrid = false;
	c.tp_ant_cable_delay_ns = -250;
	setup_mgr(&c);

	TEST_ASSERT_EQUAL_INT(0, gnssmgr_start(&g_mgr, t));
	for (i = 1U; i < 5U; i++) { /* advance to the TP1 step */
		t += 10U;
		send_ack(true, t);
	}
	TEST_ASSERT_EQUAL_UINT(5U, g_fake.sends);
	TEST_ASSERT_EQUAL_UINT64(UBX_TP_TIMEGRID_GPS,
				 valset_must_get(&g_fake, UBX_CFG_TP_TIMEGRID_TP1));
	TEST_ASSERT_EQUAL_HEX16(0xFF06,
				(uint16_t)valset_must_get(&g_fake,
							  UBX_CFG_TP_ANT_CABLEDELAY));
}

/* ---------------------------------------------------- ACK failure paths -- */

static void test_ack_timeout_retries_then_fails(void)
{
	uint32_t t = 1000U;

	setup_mgr(NULL);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_start(&g_mgr, t));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.sends);

	/* Nothing happens before the deadline. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_step(&g_mgr, t + 1499U));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.sends);

	/* Three retries: attempts 2, 3 and 4. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_step(&g_mgr, t + 1500U));
	TEST_ASSERT_EQUAL_UINT(2U, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_step(&g_mgr, t + 3000U));
	TEST_ASSERT_EQUAL_UINT(3U, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_step(&g_mgr, t + 4500U));
	TEST_ASSERT_EQUAL_UINT(4U, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_CONFIG, gnssmgr_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT32(0U, gnssmgr_alarms(&g_mgr));

	/* The fifth deadline gives up. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_step(&g_mgr, t + 6000U));
	TEST_ASSERT_EQUAL_UINT(4U, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_CONFIG_FAILED, gnssmgr_get_state(&g_mgr));
	TEST_ASSERT_TRUE(g_fake.alarm_state[GNSSMGR_ALARM_CONFIG_FAILED]);
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.alarm_edges[GNSSMGR_ALARM_CONFIG_FAILED]);

	/* Once failed it stays failed and stops transmitting. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_step(&g_mgr, t + 100000U));
	TEST_ASSERT_EQUAL_UINT(4U, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(-EPERM, gnssmgr_request_survey(&g_mgr, t));

	/* An explicit restart clears the alarm and resumes at step 1. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_start(&g_mgr, t + 100000U));
	TEST_ASSERT_EQUAL_UINT(5U, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_CONFIG, gnssmgr_get_state(&g_mgr));
	TEST_ASSERT_FALSE(g_fake.alarm_state[GNSSMGR_ALARM_CONFIG_FAILED]);
}

static void test_nak_retries_immediately(void)
{
	uint32_t t = 1000U;

	setup_mgr(NULL);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_start(&g_mgr, t));

	/* A NAK does not wait for the timeout. */
	send_ack(false, t + 5U);
	TEST_ASSERT_EQUAL_UINT(2U, g_fake.sends);
	send_ack(false, t + 10U);
	TEST_ASSERT_EQUAL_UINT(3U, g_fake.sends);
	send_ack(false, t + 15U);
	TEST_ASSERT_EQUAL_UINT(4U, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_CONFIG, gnssmgr_get_state(&g_mgr));

	send_ack(false, t + 20U);
	TEST_ASSERT_EQUAL_UINT(4U, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_CONFIG_FAILED, gnssmgr_get_state(&g_mgr));
	TEST_ASSERT_TRUE(g_fake.alarm_state[GNSSMGR_ALARM_CONFIG_FAILED]);
}

/* An ACK for something else must not advance the walk. */
static void test_foreign_ack_ignored(void)
{
	static const uint8_t other[2] = {UBX_CLASS_CFG, 0x01U};
	uint32_t t = 1000U;

	setup_mgr(NULL);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_start(&g_mgr, t));
	deliver(UBX_CLASS_ACK, UBX_ID_ACK_ACK, other, 2U, t + 1U);
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.sends);

	/* Nor may one arriving when nothing is outstanding. */
	fake_reset();
	t = walk_config(t + 10U);
	TEST_ASSERT_EQUAL_UINT(WALK_STEPS, g_fake.sends);
	send_ack(true, t + 10U);
	TEST_ASSERT_EQUAL_UINT(WALK_STEPS, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_SURVEY_IN, gnssmgr_get_state(&g_mgr));
}

static void test_send_failure_is_reported_and_retried(void)
{
	uint32_t t = 1000U;

	setup_mgr(NULL);
	g_fake.send_rc = -EIO;

	/* The caller is told, and the retry timer is armed anyway. */
	TEST_ASSERT_EQUAL_INT(-EIO, gnssmgr_start(&g_mgr, t));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_CONFIG, gnssmgr_get_state(&g_mgr));

	/* It does not spin: exactly one attempt per timeout. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_step(&g_mgr, t + 100U));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_step(&g_mgr, t + 1500U));
	TEST_ASSERT_EQUAL_UINT(2U, g_fake.sends);

	/* Once the transport recovers, the walk continues normally. */
	g_fake.send_rc = 0;
	send_ack(true, t + 1600U);
	TEST_ASSERT_EQUAL_UINT(3U, g_fake.sends);
}

/* Deadlines are computed wrap-safely over the uint32 millisecond clock. */
static void test_ack_deadline_survives_clock_wrap(void)
{
	uint32_t t = 0xFFFFFF00UL; /* 1500 ms later wraps past 2^32 */

	setup_mgr(NULL);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_start(&g_mgr, t));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.sends);

	/* 0xFFFFFF00 + 1500 = 0x1000004DC, i.e. 0x000004DC after the wrap. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_step(&g_mgr, 0xFFFFFFFFUL));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_step(&g_mgr, 0x000004DBUL));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.sends);
	/* Exactly at the deadline. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_step(&g_mgr, 0x000004DCUL));
	TEST_ASSERT_EQUAL_UINT(2U, g_fake.sends);
}

/* --------------------------------------------------------- survey-in ----- */

static void test_survey_in_progress_then_fixed(void)
{
	gnssmgr_svin_t sv;
	gnssmgr_ecef_t pos;
	uint32_t t;

	setup_mgr(NULL);
	t = walk_config(1000U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_SURVEY_IN, gnssmgr_get_state(&g_mgr));

	/* Still accumulating: progress is published, nothing else happens. */
	t += 1000U;
	send_svin(120U, 100L, 200L, 300L, 50000U, false, true, t);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_SURVEY_IN, gnssmgr_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(WALK_STEPS, g_fake.sends);
	TEST_ASSERT_EQUAL_UINT(0U, g_fake.store_calls);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_svin(&g_mgr, &sv));
	TEST_ASSERT_TRUE(sv.active);
	TEST_ASSERT_FALSE(sv.valid);
	TEST_ASSERT_EQUAL_UINT32(120U, sv.dur_s);
	TEST_ASSERT_EQUAL_UINT32(50000U, sv.mean_acc_0p1mm);
	TEST_ASSERT_EQUAL_UINT32(4242U, sv.obs);

	/* "valid but still active" is not a finished survey. */
	t += 1000U;
	send_svin(3000U, 100L, 200L, 300L, 12000U, true, true, t);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_SURVEY_IN, gnssmgr_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(0U, g_fake.store_calls);

	/* Valid and stopped: store the position and go fixed. */
	t += 1000U;
	send_svin(3600U, -2694045L, -4293642L, 3857878L, 900U, true, false, t);
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.store_calls);
	TEST_ASSERT_EQUAL_INT32(-2694045L, g_fake.store_last.x_cm);
	TEST_ASSERT_EQUAL_INT32(-4293642L, g_fake.store_last.y_cm);
	TEST_ASSERT_EQUAL_INT32(3857878L, g_fake.store_last.z_cm);
	TEST_ASSERT_EQUAL_INT8(-7, g_fake.store_last.x_hp);
	TEST_ASSERT_EQUAL_INT8(3, g_fake.store_last.y_hp);
	TEST_ASSERT_EQUAL_UINT32(900U, g_fake.store_last.acc_0p1mm);
	TEST_ASSERT_TRUE(g_fake.store_last.valid);

	/* The TMODE step is re-issued, now in fixed form. */
	TEST_ASSERT_EQUAL_UINT(WALK_STEPS + 1U, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_CONFIG, gnssmgr_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT64(UBX_TMODE_FIXED,
				 valset_must_get(&g_fake, UBX_CFG_TMODE_MODE));
	TEST_ASSERT_EQUAL_HEX32((uint32_t)-2694045L,
				(uint32_t)valset_must_get(&g_fake, UBX_CFG_TMODE_ECEF_X));

	t += 10U;
	send_ack(true, t);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_FIXED, gnssmgr_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_position(&g_mgr, &pos));
	TEST_ASSERT_EQUAL_INT32(3857878L, pos.z_cm);

	/* A later NAV-SVIN in fixed mode updates telemetry but changes nothing. */
	t += 1000U;
	send_svin(4000U, 1L, 2L, 3L, 100U, true, false, t);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_FIXED, gnssmgr_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.store_calls);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_position(&g_mgr, &pos));
	TEST_ASSERT_EQUAL_INT32(3857878L, pos.z_cm);
}

static void test_request_survey_is_explicit_only(void)
{
	uint32_t t;

	setup_mgr(NULL);
	/* Not started: refused. */
	TEST_ASSERT_EQUAL_INT(-EPERM, gnssmgr_request_survey(&g_mgr, 0U));

	t = walk_config(1000U);
	t += 100U;
	send_svin(3600U, 10L, 20L, 30L, 900U, true, false, t);
	t += 10U;
	send_ack(true, t);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_FIXED, gnssmgr_get_state(&g_mgr));

	/* An operator asks for a re-survey: only the TMODE step is re-sent. */
	t += 1000U;
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_request_survey(&g_mgr, t));
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_CONFIG, gnssmgr_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT64(UBX_TMODE_SURVEY_IN,
				 valset_must_get(&g_fake, UBX_CFG_TMODE_MODE));

	/* The stored position is dropped, so nothing claims a fixed position. */
	{
		gnssmgr_ecef_t pos;

		TEST_ASSERT_EQUAL_INT(-EAGAIN, gnssmgr_position(&g_mgr, &pos));
	}

	t += 10U;
	send_ack(true, t);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_SURVEY_IN, gnssmgr_get_state(&g_mgr));

	/* The re-survey keeps the TX_READY trust it already earned. */
	TEST_ASSERT_TRUE(gnssmgr_txready_trusted(&g_mgr));
}

/*
 * Asking for a survey while the configuration walk is still running must not
 * fast-forward to the TMODE step: everything between — message rates, dynamic
 * model, constellations, both time pulses, TX_READY — would never be sent, and
 * the receiver would sit in survey-in emitting nothing this firmware reads.
 */
static void test_request_survey_mid_walk_does_not_skip_steps(void)
{
	gnssmgr_ecef_t pos = {
		.x_cm = 1L, .y_cm = 2L, .z_cm = 3L,
		.x_hp = 0, .y_hp = 0, .z_hp = 0,
		.acc_0p1mm = 500U, .valid = true,
	};
	uint32_t t = 1000U;
	unsigned int i;

	setup_mgr(NULL);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_set_stored_ecef(&g_mgr, &pos));
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_start(&g_mgr, t));

	/* Advance to step 3 of 8, then ask for a re-survey. */
	t += 10U;
	send_ack(true, t);
	t += 10U;
	send_ack(true, t);
	TEST_ASSERT_EQUAL_UINT(3U, g_fake.sends);

	TEST_ASSERT_EQUAL_INT(0, gnssmgr_request_survey(&g_mgr, t));
	/* Nothing was re-sent and the walk was not rewound. */
	TEST_ASSERT_EQUAL_UINT(3U, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_CONFIG, gnssmgr_get_state(&g_mgr));

	/* The remaining steps still run, in order. */
	for (i = 3U; i < WALK_STEPS; i++) {
		t += 10U;
		send_ack(true, t);
	}
	TEST_ASSERT_EQUAL_UINT(WALK_STEPS, g_fake.sends);
	TEST_ASSERT_TRUE(gnssmgr_txready_trusted(&g_mgr));

	/* ...and the TMODE step, when it arrives, asks for a survey. */
	TEST_ASSERT_EQUAL_UINT64(UBX_TMODE_SURVEY_IN,
				 valset_must_get(&g_fake, UBX_CFG_TMODE_MODE));
	t += 10U;
	send_ack(true, t);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_SURVEY_IN, gnssmgr_get_state(&g_mgr));
}

/* Asking while the TMODE step itself is outstanding must re-issue it. */
static void test_request_survey_reissues_outstanding_tmode(void)
{
	gnssmgr_ecef_t pos = {
		.x_cm = 1L, .y_cm = 2L, .z_cm = 3L,
		.x_hp = 0, .y_hp = 0, .z_hp = 0,
		.acc_0p1mm = 500U, .valid = true,
	};
	uint32_t t = 1000U;
	unsigned int i;

	setup_mgr(NULL);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_set_stored_ecef(&g_mgr, &pos));
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_start(&g_mgr, t));
	for (i = 1U; i < WALK_STEPS; i++) {
		t += 10U;
		send_ack(true, t);
	}
	/* The fixed-position TMODE frame is on the wire, unacknowledged. */
	TEST_ASSERT_EQUAL_UINT(WALK_STEPS, g_fake.sends);
	TEST_ASSERT_EQUAL_UINT64(UBX_TMODE_FIXED,
				 valset_must_get(&g_fake, UBX_CFG_TMODE_MODE));

	t += 5U;
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_request_survey(&g_mgr, t));
	TEST_ASSERT_EQUAL_UINT(WALK_STEPS + 1U, g_fake.sends);
	TEST_ASSERT_EQUAL_UINT64(UBX_TMODE_SURVEY_IN,
				 valset_must_get(&g_fake, UBX_CFG_TMODE_MODE));

	t += 10U;
	send_ack(true, t);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_SURVEY_IN, gnssmgr_get_state(&g_mgr));
}

/* A receiver reset does not throw away the leap schedule. */
static void test_reset_keeps_leap_schedule(void)
{
	gnssmgr_leap_t ls;
	uint32_t t;

	setup_mgr(NULL);
	t = walk_config(1000U);

	t += 1000U;
	send_timels(18, 1, 3600, 0x03U, t);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_LI_INSERT, gnssmgr_leap_indicator(&g_mgr));

	t += 100U;
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_notify_reset(&g_mgr, t));
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_leap(&g_mgr, &ls));
	TEST_ASSERT_EQUAL_INT8(18, ls.current_ls);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_LI_INSERT, gnssmgr_leap_indicator(&g_mgr));
}

/* ------------------------------------------------------------- fix/lock -- */

static void test_time_lock_thresholds_and_hysteresis(void)
{
	gnssmgr_status_t st;
	uint32_t t;

	setup_mgr(NULL);
	t = walk_config(1000U);

	/* A 3D fix whose tAcc is above the lock threshold does not lock. */
	t += 1000U;
	send_pvt(1000U, UBX_FIX_3D, true, 150U, true, 12U, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_status(&g_mgr, &st));
	TEST_ASSERT_TRUE(st.valid);
	TEST_ASSERT_FALSE(st.time_locked);
	TEST_ASSERT_TRUE(st.gnss_fix_ok);
	TEST_ASSERT_TRUE(st.utc_valid);
	TEST_ASSERT_EQUAL_UINT8(12U, st.num_sv);
	TEST_ASSERT_EQUAL_UINT32(150U, st.tacc_ns);
	TEST_ASSERT_EQUAL_UINT16(2026U, st.year);
	TEST_ASSERT_EQUAL_UINT16(120U, st.pdop);
	TEST_ASSERT_EQUAL_UINT32(0U, gnssmgr_alarms(&g_mgr));

	/* Inside the lock threshold: locked, and no alarm was ever raised. */
	t += 1000U;
	send_pvt(2000U, UBX_FIX_3D, true, 90U, true, 14U, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_status(&g_mgr, &st));
	TEST_ASSERT_TRUE(st.time_locked);
	TEST_ASSERT_EQUAL_UINT(0U, g_fake.alarm_edges[GNSSMGR_ALARM_TIME_UNLOCKED]);

	/* Between the two thresholds: the hysteresis holds the lock. */
	t += 1000U;
	send_pvt(3000U, UBX_FIX_3D, true, 200U, true, 14U, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_status(&g_mgr, &st));
	TEST_ASSERT_TRUE(st.time_locked);

	/* Above the unlock threshold: lock drops and the alarm is raised once. */
	t += 1000U;
	send_pvt(4000U, UBX_FIX_3D, true, 251U, true, 14U, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_status(&g_mgr, &st));
	TEST_ASSERT_FALSE(st.time_locked);
	TEST_ASSERT_TRUE(g_fake.alarm_state[GNSSMGR_ALARM_TIME_UNLOCKED]);
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.alarm_edges[GNSSMGR_ALARM_TIME_UNLOCKED]);

	/* 200 ns no longer suffices: re-locking needs the tighter threshold. */
	t += 1000U;
	send_pvt(5000U, UBX_FIX_3D, true, 200U, true, 14U, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_status(&g_mgr, &st));
	TEST_ASSERT_FALSE(st.time_locked);

	t += 1000U;
	send_pvt(6000U, UBX_FIX_3D, true, 100U, true, 14U, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_status(&g_mgr, &st));
	TEST_ASSERT_TRUE(st.time_locked);
	TEST_ASSERT_FALSE(g_fake.alarm_state[GNSSMGR_ALARM_TIME_UNLOCKED]);
	TEST_ASSERT_EQUAL_UINT(2U, g_fake.alarm_edges[GNSSMGR_ALARM_TIME_UNLOCKED]);
}

static void test_time_lock_requires_fix_and_utc(void)
{
	gnssmgr_status_t st;
	uint32_t t;

	setup_mgr(NULL);
	t = walk_config(1000U);

	/* TIME_ONLY is a valid timing fix. */
	t += 1000U;
	send_pvt(1000U, UBX_FIX_TIME_ONLY, true, 20U, true, 6U, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_status(&g_mgr, &st));
	TEST_ASSERT_TRUE(st.time_locked);

	/* gnssFixOK clear disqualifies it however good tAcc looks. */
	t += 1000U;
	send_pvt(2000U, UBX_FIX_3D, false, 5U, true, 14U, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_status(&g_mgr, &st));
	TEST_ASSERT_FALSE(st.time_locked);

	/* So does a 2D fix. */
	t += 1000U;
	send_pvt(3000U, UBX_FIX_2D, true, 5U, true, 4U, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_status(&g_mgr, &st));
	TEST_ASSERT_FALSE(st.time_locked);

	/* And so does an unresolved UTC. */
	t += 1000U;
	send_pvt(4000U, UBX_FIX_3D, true, 5U, false, 14U, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_status(&g_mgr, &st));
	TEST_ASSERT_FALSE(st.time_locked);
	TEST_ASSERT_FALSE(st.utc_valid);
}

static void test_stale_pvt_drops_lock(void)
{
	gnssmgr_status_t st;
	uint32_t t;

	setup_mgr(NULL);
	t = walk_config(1000U);

	t += 1000U;
	send_pvt(1000U, UBX_FIX_3D, true, 20U, true, 14U, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_status(&g_mgr, &st));
	TEST_ASSERT_TRUE(st.time_locked);

	/* Inside the staleness window nothing changes. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_tick_1hz(&g_mgr, NULL, t + 3000U));
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_status(&g_mgr, &st));
	TEST_ASSERT_TRUE(st.time_locked);
	TEST_ASSERT_EQUAL_UINT32(3000U, st.age_ms);

	/* Past it, the lock is withdrawn and the alarm raised. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_tick_1hz(&g_mgr, NULL, t + 3001U));
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_status(&g_mgr, &st));
	TEST_ASSERT_FALSE(st.time_locked);
	TEST_ASSERT_TRUE(g_fake.alarm_state[GNSSMGR_ALARM_TIME_UNLOCKED]);
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.alarm_edges[GNSSMGR_ALARM_TIME_UNLOCKED]);

	/* Repeated ticks do not re-raise it. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_tick_1hz(&g_mgr, NULL, t + 9000U));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.alarm_edges[GNSSMGR_ALARM_TIME_UNLOCKED]);

	/* A tick before any NAV-PVT is harmless. */
	setup_mgr(NULL);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_tick_1hz(&g_mgr, NULL, 100000U));
	TEST_ASSERT_EQUAL_UINT32(0U, gnssmgr_alarms(&g_mgr));
}

/* ---------------------------------------------------------------- leap --- */

static void test_leap_tracking_and_indicator(void)
{
	gnssmgr_leap_t ls;
	uint32_t t;

	setup_mgr(NULL);
	t = walk_config(1000U);

	/* No change scheduled. */
	t += 1000U;
	send_timels(18, 0, 0, 0x03U, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_leap(&g_mgr, &ls));
	TEST_ASSERT_TRUE(ls.valid);
	TEST_ASSERT_TRUE(ls.curr_ls_valid);
	TEST_ASSERT_EQUAL_INT8(18, ls.current_ls);
	TEST_ASSERT_FALSE(ls.event_pending);
	TEST_ASSERT_EQUAL_UINT16(2500U, ls.gps_wn);
	TEST_ASSERT_EQUAL_UINT16(7U, ls.gps_dn);
	TEST_ASSERT_EQUAL_UINT8(2U, ls.src_curr);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_LI_NONE, gnssmgr_leap_indicator(&g_mgr));

	/* An insertion further out than the announce window stays quiet. */
	t += 1000U;
	send_timels(18, 1, 90000, 0x03U, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_leap(&g_mgr, &ls));
	TEST_ASSERT_TRUE(ls.event_pending);
	TEST_ASSERT_EQUAL_INT8(1, ls.ls_change);
	TEST_ASSERT_EQUAL_INT32(90000, ls.time_to_event_s);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_LI_NONE, gnssmgr_leap_indicator(&g_mgr));

	/* Exactly at the window edge it is announced. */
	t += 1000U;
	send_timels(18, 1, 86400, 0x03U, t);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_LI_INSERT, gnssmgr_leap_indicator(&g_mgr));

	/* Deletion inside the window. */
	t += 1000U;
	send_timels(18, -1, 3600, 0x03U, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_leap(&g_mgr, &ls));
	TEST_ASSERT_EQUAL_INT8(-1, ls.ls_change);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_LI_DELETE, gnssmgr_leap_indicator(&g_mgr));

	/* Once the event is behind us the announcement stops. */
	t += 1000U;
	send_timels(17, -1, -5, 0x03U, t);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_LI_NONE, gnssmgr_leap_indicator(&g_mgr));

	/* timeToLsEvent not valid: pending is not asserted. */
	t += 1000U;
	send_timels(17, 1, 100, 0x01U, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_leap(&g_mgr, &ls));
	TEST_ASSERT_FALSE(ls.event_valid);
	TEST_ASSERT_FALSE(ls.event_pending);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_LI_NONE, gnssmgr_leap_indicator(&g_mgr));

	/* currLs not valid: the whole leap state is unusable. */
	t += 1000U;
	send_timels(17, 1, 100, 0x02U, t);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_LI_UNSYNC, gnssmgr_leap_indicator(&g_mgr));
}

/* --------------------------------------------------------------- TIM-TP -- */

static void test_qerr_pairing(void)
{
	gnssmgr_qerr_t q;
	uint32_t t;

	setup_mgr(NULL);
	t = walk_config(1000U);

	/*
	 * TIM-TP received during second N describes the pulse at the start of
	 * second N+1, and its towMS already names that pulse. The record must
	 * therefore be tagged with towMS verbatim — a manager that "corrected"
	 * it by a second would hand the discipline loop the wrong pairing.
	 */
	t += 1000U;
	send_tim_tp(259201000UL, -1750, false, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_qerr_for_pps(&g_mgr, &q));
	TEST_ASSERT_TRUE(q.valid);
	TEST_ASSERT_TRUE(q.qerr_valid);
	TEST_ASSERT_EQUAL_INT32(-1750, q.qerr_ps);
	TEST_ASSERT_EQUAL_UINT32(259201000UL, q.target_tow_ms);
	TEST_ASSERT_EQUAL_HEX32(0x40000000UL, q.target_tow_sub_ms);
	TEST_ASSERT_EQUAL_UINT16(2500U, q.week);
	TEST_ASSERT_TRUE(q.time_base_utc);
	TEST_ASSERT_TRUE(q.utc_available);
	TEST_ASSERT_EQUAL_UINT32(t, q.rx_mono_ms);

	/* Positive qErr, and the next second's pulse. */
	t += 1000U;
	send_tim_tp(259202000UL, 2500, false, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_qerr_for_pps(&g_mgr, &q));
	TEST_ASSERT_EQUAL_INT32(2500, q.qerr_ps);
	TEST_ASSERT_EQUAL_UINT32(259202000UL, q.target_tow_ms);

	/* qErrInvalid: the record still arrives, flagged not to be applied. */
	t += 1000U;
	send_tim_tp(259203000UL, 12345, true, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_qerr_for_pps(&g_mgr, &q));
	TEST_ASSERT_TRUE(q.valid);
	TEST_ASSERT_FALSE(q.qerr_valid);
	TEST_ASSERT_EQUAL_UINT32(259203000UL, q.target_tow_ms);
}

/* -------------------------------------------------------------- NAV-SAT -- */

static void test_nav_sat_summary(void)
{
	gnssmgr_sats_t s;
	uint32_t t;

	setup_mgr(NULL);
	t = walk_config(1000U);

	t += 1000U;
	send_nav_sat(t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_sats(&g_mgr, &s));
	TEST_ASSERT_TRUE(s.valid);
	TEST_ASSERT_EQUAL_UINT32(1000U, s.itow_ms);
	TEST_ASSERT_EQUAL_UINT8(3U, s.tracked);
	TEST_ASSERT_EQUAL_UINT8(2U, s.used);
	TEST_ASSERT_EQUAL_UINT8(50U, s.cno_max);
	TEST_ASSERT_EQUAL_UINT8(47U, s.cno_avg_used); /* (44 + 50) / 2 */
}

/* A malformed message of a known type is reported, not silently absorbed. */
static void test_malformed_known_messages(void)
{
	static const uint8_t junk[4] = {0, 0, 0, 0};
	ubx_msg_t m;
	unsigned int i;
	static const struct {
		uint8_t cls;
		uint8_t id;
	} kinds[] = {
		{UBX_CLASS_NAV, UBX_ID_NAV_PVT},
		{UBX_CLASS_NAV, UBX_ID_NAV_SAT},
		{UBX_CLASS_NAV, UBX_ID_NAV_TIMELS},
		{UBX_CLASS_NAV, UBX_ID_NAV_SVIN},
		{UBX_CLASS_TIM, UBX_ID_TIM_TP},
		{UBX_CLASS_MON, UBX_ID_MON_RF},
		{UBX_CLASS_ACK, UBX_ID_ACK_ACK},
	};

	setup_mgr(NULL);
	for (i = 0U; i < (sizeof(kinds) / sizeof(kinds[0])); i++) {
		m.cls = kinds[i].cls;
		m.id = kinds[i].id;
		m.len = 3U; /* wrong for every one of them */
		m.payload = junk;
		TEST_ASSERT_EQUAL_INT(-EBADMSG, gnssmgr_on_msg(&g_mgr, &m, 0U));
	}

	/* An unknown message is not an error — the receiver emits plenty. */
	m.cls = UBX_CLASS_MON;
	m.id = 0x04U; /* MON-VER */
	m.len = 4U;
	m.payload = junk;
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_on_msg(&g_mgr, &m, 0U));

	/* A block-less MON-RF is well formed but says nothing. */
	{
		static const uint8_t empty_rf[4] = {0, 0, 0, 0};

		m.cls = UBX_CLASS_MON;
		m.id = UBX_ID_MON_RF;
		m.len = 4U;
		m.payload = empty_rf;
		TEST_ASSERT_EQUAL_INT(0, gnssmgr_on_msg(&g_mgr, &m, 0U));
	}
}

/* ---------------------------------------------------- antenna supervisor -- */

static gnssmgr_ant_input_t ant_ok_input(void)
{
	gnssmgr_ant_input_t in;

	in.ant_off_mon = false;
	in.bias_en = true;
	in.current_valid = true;
	in.current_ua = 25000U; /* 25 mA — a normal active antenna */
	return in;
}

/** Feed @p n identical antenna samples, one per second. */
static void ant_feed(const gnssmgr_ant_input_t *in, uint32_t *t, unsigned int n)
{
	unsigned int i;

	for (i = 0U; i < n; i++) {
		*t += 1000U;
		TEST_ASSERT_EQUAL_INT(0, gnssmgr_tick_1hz(&g_mgr, in, *t));
	}
}

/*
 * The debounce guards the first classification too: UNKNOWN reaches OK only
 * after the full run of samples. That is deliberate — the alternative, adopting
 * whatever the first sample says, would let one spurious reading at power-up
 * cut the antenna bias before the rail has even settled.
 */
static void test_antenna_first_verdict_is_debounced(void)
{
	gnssmgr_ant_input_t in = ant_ok_input();
	uint32_t t;

	setup_mgr(NULL);
	t = walk_config(1000U);
	send_mon_rf(UBX_ANT_STATUS_OK, UBX_ANT_POWER_ON, t);

	ant_feed(&in, &t, 1U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_UNKNOWN, gnssmgr_ant_get_state(&g_mgr));
	ant_feed(&in, &t, 1U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_UNKNOWN, gnssmgr_ant_get_state(&g_mgr));
	ant_feed(&in, &t, 1U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OK, gnssmgr_ant_get_state(&g_mgr));

	/* An already-held verdict re-asserted costs nothing and raises nothing. */
	ant_feed(&in, &t, 5U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OK, gnssmgr_ant_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT32(0U, gnssmgr_alarms(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(0U, g_fake.bias_calls);
}

static void test_antenna_short_debounce_drops_bias(void)
{
	gnssmgr_ant_input_t in = ant_ok_input();
	uint32_t t;

	setup_mgr(NULL);
	t = walk_config(1000U);

	/* Healthy for a while. */
	send_mon_rf(UBX_ANT_STATUS_OK, UBX_ANT_POWER_ON, t);
	ant_feed(&in, &t, 3U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OK, gnssmgr_ant_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(0U, g_fake.bias_calls);

	/* Into the foldback band. Two samples are not enough. */
	in.current_ua = 181000U;
	ant_feed(&in, &t, 1U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OK, gnssmgr_ant_get_state(&g_mgr));
	ant_feed(&in, &t, 1U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OK, gnssmgr_ant_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(0U, g_fake.bias_calls);

	/* The third consecutive sample latches it. */
	ant_feed(&in, &t, 1U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_SHORT, gnssmgr_ant_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.bias_calls);
	TEST_ASSERT_FALSE(g_fake.bias_last);
	TEST_ASSERT_TRUE(g_fake.alarm_state[GNSSMGR_ALARM_ANT_SHORT]);
	TEST_ASSERT_TRUE(gnssmgr_ant_short_latched(&g_mgr));

	/*
	 * The bias is now off, so the caller reports it off and the supervisor
	 * masks the (phantom) fault flags — but the latch and the alarm stay:
	 * there is no automatic recovery from a short.
	 */
	in.bias_en = false;
	ant_feed(&in, &t, 1U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OFF, gnssmgr_ant_get_state(&g_mgr));
	TEST_ASSERT_TRUE(g_fake.alarm_state[GNSSMGR_ALARM_ANT_SHORT]);
	TEST_ASSERT_TRUE(gnssmgr_ant_short_latched(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.bias_calls);

	/* Only an explicit re-enable clears it. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_ant_reenable(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(2U, g_fake.bias_calls);
	TEST_ASSERT_TRUE(g_fake.bias_last);
	TEST_ASSERT_FALSE(g_fake.alarm_state[GNSSMGR_ALARM_ANT_SHORT]);
	TEST_ASSERT_FALSE(gnssmgr_ant_short_latched(&g_mgr));
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_UNKNOWN, gnssmgr_ant_get_state(&g_mgr));

	/* It is idempotent. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_ant_reenable(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(3U, g_fake.bias_calls);
	TEST_ASSERT_EQUAL_UINT(2U, g_fake.alarm_edges[GNSSMGR_ALARM_ANT_SHORT]);
}

static void test_antenna_open_alarms_without_cutting_bias(void)
{
	gnssmgr_ant_input_t in = ant_ok_input();
	uint32_t t;

	setup_mgr(NULL);
	t = walk_config(1000U);

	send_mon_rf(UBX_ANT_STATUS_OK, UBX_ANT_POWER_ON, t);
	ant_feed(&in, &t, 3U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OK, gnssmgr_ant_get_state(&g_mgr));

	/* Below the ~5 mA DETECT threshold: nothing is drawing current. */
	in.current_ua = 400U;
	ant_feed(&in, &t, 3U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OPEN, gnssmgr_ant_get_state(&g_mgr));
	TEST_ASSERT_TRUE(g_fake.alarm_state[GNSSMGR_ALARM_ANT_OPEN]);
	/* An open antenna is a fault to report, not a reason to cut power. */
	TEST_ASSERT_EQUAL_UINT(0U, g_fake.bias_calls);

	/* Plugging it back in clears the alarm without any operator action. */
	in.current_ua = 25000U;
	ant_feed(&in, &t, 3U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OK, gnssmgr_ant_get_state(&g_mgr));
	TEST_ASSERT_FALSE(g_fake.alarm_state[GNSSMGR_ALARM_ANT_OPEN]);

	/* Exactly at the threshold is "present", not "open". */
	in.current_ua = 5000U;
	ant_feed(&in, &t, 3U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OK, gnssmgr_ant_get_state(&g_mgr));
	in.current_ua = 4999U;
	ant_feed(&in, &t, 3U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OPEN, gnssmgr_ant_get_state(&g_mgr));
}

/*
 * With the antenna commanded off the analog supervisor reports SHORT and
 * "absent" even though nothing is wrong (bias doc §9.3). Those flags must be
 * masked outright — a debounce would still fire on every commanded cycle.
 */
static void test_antenna_commanded_off_masks_faults(void)
{
	gnssmgr_ant_input_t in = ant_ok_input();
	uint32_t t;
	unsigned int i;

	setup_mgr(NULL);
	t = walk_config(1000U);

	/* The F9T pulls ANT_OFF and everything collapses. */
	in.ant_off_mon = true;
	in.current_ua = 0U;
	send_mon_rf(UBX_ANT_STATUS_SHORT, UBX_ANT_POWER_OFF, t);
	for (i = 0U; i < 5U; i++) {
		t += 1000U;
		TEST_ASSERT_EQUAL_INT(0, gnssmgr_tick_1hz(&g_mgr, &in, t));
	}
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OFF, gnssmgr_ant_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT32(0U, gnssmgr_alarms(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(0U, g_fake.bias_calls);

	/* Firmware holding ANT_BIAS_EN low masks it just the same. */
	in.ant_off_mon = false;
	in.bias_en = false;
	for (i = 0U; i < 5U; i++) {
		t += 1000U;
		TEST_ASSERT_EQUAL_INT(0, gnssmgr_tick_1hz(&g_mgr, &in, t));
	}
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OFF, gnssmgr_ant_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT32(0U, gnssmgr_alarms(&g_mgr));
}

/* MON-RF alone is enough to condemn the antenna when the INA read fails. */
static void test_antenna_mon_rf_only(void)
{
	gnssmgr_ant_input_t in = ant_ok_input();
	uint32_t t;
	unsigned int i;

	setup_mgr(NULL);
	t = walk_config(1000U);

	in.current_valid = false;
	in.current_ua = 0U; /* stale value the manager must ignore */

	/* No evidence at all yet. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_tick_1hz(&g_mgr, &in, t));
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_UNKNOWN, gnssmgr_ant_get_state(&g_mgr));

	send_mon_rf(UBX_ANT_STATUS_SHORT, UBX_ANT_POWER_ON, t);
	for (i = 0U; i < 3U; i++) {
		t += 1000U;
		TEST_ASSERT_EQUAL_INT(0, gnssmgr_tick_1hz(&g_mgr, &in, t));
	}
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_SHORT, gnssmgr_ant_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.bias_calls);
	TEST_ASSERT_FALSE(g_fake.bias_last);
}

/* A short claimed by either source outranks an "ok" from the other. */
static void test_antenna_short_outranks_open_and_ok(void)
{
	gnssmgr_ant_input_t in = ant_ok_input();
	uint32_t t;
	unsigned int i;

	setup_mgr(NULL);
	t = walk_config(1000U);

	/* MON-RF says OK, the shunt says foldback. */
	send_mon_rf(UBX_ANT_STATUS_OK, UBX_ANT_POWER_ON, t);
	in.current_ua = 200000U;
	for (i = 0U; i < 3U; i++) {
		t += 1000U;
		TEST_ASSERT_EQUAL_INT(0, gnssmgr_tick_1hz(&g_mgr, &in, t));
	}
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_SHORT, gnssmgr_ant_get_state(&g_mgr));

	/* Recover, then have MON-RF claim OPEN while the current is normal. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_ant_reenable(&g_mgr));
	send_mon_rf(UBX_ANT_STATUS_OPEN, UBX_ANT_POWER_ON, t);
	in.current_ua = 25000U;
	for (i = 0U; i < 3U; i++) {
		t += 1000U;
		TEST_ASSERT_EQUAL_INT(0, gnssmgr_tick_1hz(&g_mgr, &in, t));
	}
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OPEN, gnssmgr_ant_get_state(&g_mgr));
	TEST_ASSERT_TRUE(g_fake.alarm_state[GNSSMGR_ALARM_ANT_OPEN]);

	/* Now both claim a fault; SHORT wins and the alarms swap over. */
	send_mon_rf(UBX_ANT_STATUS_SHORT, UBX_ANT_POWER_ON, t);
	for (i = 0U; i < 3U; i++) {
		t += 1000U;
		TEST_ASSERT_EQUAL_INT(0, gnssmgr_tick_1hz(&g_mgr, &in, t));
	}
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_SHORT, gnssmgr_ant_get_state(&g_mgr));
	TEST_ASSERT_FALSE(g_fake.alarm_state[GNSSMGR_ALARM_ANT_OPEN]);
	TEST_ASSERT_TRUE(g_fake.alarm_state[GNSSMGR_ALARM_ANT_SHORT]);
}

/* An interrupted run of bad samples must not accumulate towards the latch. */
static void test_antenna_debounce_needs_consecutive_samples(void)
{
	gnssmgr_ant_input_t in = ant_ok_input();
	uint32_t t;
	unsigned int i;

	setup_mgr(NULL);
	t = walk_config(1000U);

	send_mon_rf(UBX_ANT_STATUS_OK, UBX_ANT_POWER_ON, t);
	ant_feed(&in, &t, 3U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OK, gnssmgr_ant_get_state(&g_mgr));

	for (i = 0U; i < 6U; i++) {
		in.current_ua = 200000U; /* short */
		ant_feed(&in, &t, 1U);
		in.current_ua = 25000U;  /* ...and back */
		ant_feed(&in, &t, 1U);
	}
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OK, gnssmgr_ant_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(0U, g_fake.bias_calls);
	TEST_ASSERT_EQUAL_UINT32(0U, gnssmgr_alarms(&g_mgr));

	/*
	 * Two bad samples, one good, two bad again: the good sample restarted
	 * the run, so nothing has latched — the counter stands at two, not four.
	 */
	in.current_ua = 200000U;
	ant_feed(&in, &t, 2U);
	in.current_ua = 25000U;
	ant_feed(&in, &t, 1U);
	in.current_ua = 200000U;
	ant_feed(&in, &t, 2U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OK, gnssmgr_ant_get_state(&g_mgr));

	/* ...and one more completes a real run of three, which does latch. */
	ant_feed(&in, &t, 1U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_SHORT, gnssmgr_ant_get_state(&g_mgr));

	/* Recover, then alternate between two *different* faults: neither ever
	 * accumulates a run, so neither latches. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_ant_reenable(&g_mgr));
	in.current_ua = 25000U;
	ant_feed(&in, &t, 3U);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OK, gnssmgr_ant_get_state(&g_mgr));

	for (i = 0U; i < 6U; i++) {
		in.current_ua = 200000U;
		ant_feed(&in, &t, 1U);
		in.current_ua = 100U;
		ant_feed(&in, &t, 1U);
	}
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_OK, gnssmgr_ant_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT32(0U, gnssmgr_alarms(&g_mgr));
}

static void test_antenna_debounce_of_one_is_immediate(void)
{
	gnssmgr_cfg_t c;
	gnssmgr_ant_input_t in = ant_ok_input();
	uint32_t t;

	TEST_ASSERT_EQUAL_INT(0, gnssmgr_cfg_default(&c));
	c.ant_debounce = 1U;
	setup_mgr(&c);
	t = walk_config(1000U);

	in.current_ua = 200000U;
	t += 1000U;
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_tick_1hz(&g_mgr, &in, t));
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_SHORT, gnssmgr_ant_get_state(&g_mgr));
}

/* --------------------------------------------------- receiver restarts --- */

static void test_notify_reset_reruns_config(void)
{
	gnssmgr_status_t st;
	gnssmgr_qerr_t q;
	gnssmgr_ecef_t pos;
	uint32_t t;

	setup_mgr(NULL);
	t = walk_config(1000U);
	t += 100U;
	send_svin(3600U, 10L, 20L, 30L, 900U, true, false, t);
	t += 10U;
	send_ack(true, t);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_FIXED, gnssmgr_get_state(&g_mgr));

	t += 1000U;
	send_pvt(5000U, UBX_FIX_3D, true, 20U, true, 14U, t);
	send_tim_tp(6000U, -100, false, t);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_status(&g_mgr, &st));
	TEST_ASSERT_TRUE(st.time_locked);

	/* The glue pulses GPS_RST_N. */
	t += 100U;
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_notify_reset(&g_mgr, t));
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_CONFIG, gnssmgr_get_state(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(WALK_STEPS + 2U, g_fake.sends);
	TEST_ASSERT_FALSE(gnssmgr_txready_trusted(&g_mgr));

	/* Everything the old receiver told us is discarded... */
	TEST_ASSERT_EQUAL_INT(-EAGAIN, gnssmgr_status(&g_mgr, &st));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, gnssmgr_qerr_for_pps(&g_mgr, &q));
	TEST_ASSERT_FALSE(g_fake.alarm_state[GNSSMGR_ALARM_TIME_UNLOCKED]);
	/* ...except the surveyed position, which describes the site. */
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_position(&g_mgr, &pos));
	TEST_ASSERT_EQUAL_INT32(30L, pos.z_cm);

	/* So the walk ends in fixed mode, not another survey. */
	{
		unsigned int i;

		for (i = 1U; i <= WALK_STEPS; i++) {
			t += 10U;
			send_ack(true, t);
		}
	}
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_FIXED, gnssmgr_get_state(&g_mgr));
}

/*
 * Secondary detector: an iTOW that jumps backwards by more than the configured
 * threshold means the receiver restarted without telling anyone. The weekly
 * rollover is a backwards jump too and must not trigger it.
 */
static void test_itow_backstep_detection(void)
{
	uint32_t t;
	unsigned int base;

	setup_mgr(NULL);
	t = walk_config(1000U);

	t += 1000U;
	send_pvt(500000U, UBX_FIX_3D, true, 20U, true, 14U, t);
	base = g_fake.sends;

	/* A small backwards step is jitter, not a restart. */
	t += 1000U;
	send_pvt(490000U, UBX_FIX_3D, true, 20U, true, 14U, t);
	TEST_ASSERT_EQUAL_UINT(base, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_SURVEY_IN, gnssmgr_get_state(&g_mgr));

	/* A large one re-runs the configuration. */
	t += 1000U;
	send_pvt(1000U, UBX_FIX_3D, true, 20U, true, 14U, t);
	TEST_ASSERT_EQUAL_UINT(base + 1U, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_CONFIG, gnssmgr_get_state(&g_mgr));

	/* The week rollover: 604799000 -> 500 must be left alone. */
	setup_mgr(NULL);
	t = walk_config(1000U);
	t += 1000U;
	send_pvt(604799000UL, UBX_FIX_3D, true, 20U, true, 14U, t);
	base = g_fake.sends;
	t += 1000U;
	send_pvt(500U, UBX_FIX_3D, true, 20U, true, 14U, t);
	TEST_ASSERT_EQUAL_UINT(base, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_SURVEY_IN, gnssmgr_get_state(&g_mgr));

	/* An idle manager never emits, however the iTOW behaves. */
	setup_mgr(NULL);
	send_pvt(500000U, UBX_FIX_3D, true, 20U, true, 14U, 10U);
	send_pvt(1000U, UBX_FIX_3D, true, 20U, true, 14U, 20U);
	TEST_ASSERT_EQUAL_UINT(0U, g_fake.sends);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_IDLE, gnssmgr_get_state(&g_mgr));
}

/* Optional callbacks may be absent; the manager must not assume otherwise. */
static void test_optional_callbacks_may_be_null(void)
{
	gnssmgr_cfg_t c;
	gnssmgr_cb_t cb;
	gnssmgr_ant_input_t in = ant_ok_input();
	uint32_t t = 1000U;
	unsigned int i;

	fake_reset();
	(void)memset(&cb, 0, sizeof(cb));
	cb.send_ubx = fake_send;
	cb.user = &g_fake;

	TEST_ASSERT_EQUAL_INT(0, gnssmgr_cfg_default(&c));
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_init(&g_mgr, &c, &cb));

	TEST_ASSERT_EQUAL_INT(0, gnssmgr_start(&g_mgr, t));
	for (i = 1U; i <= WALK_STEPS; i++) {
		t += 10U;
		send_ack(true, t);
	}
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_SURVEY_IN, gnssmgr_get_state(&g_mgr));

	/* store_ecef is absent: the survey still completes. */
	t += 100U;
	send_svin(3600U, 1L, 2L, 3L, 900U, true, false, t);
	t += 10U;
	send_ack(true, t);
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ST_FIXED, gnssmgr_get_state(&g_mgr));

	/* alarm and set_ant_bias are absent: the short still latches. */
	in.current_ua = 200000U;
	for (i = 0U; i < 3U; i++) {
		t += 1000U;
		TEST_ASSERT_EQUAL_INT(0, gnssmgr_tick_1hz(&g_mgr, &in, t));
	}
	TEST_ASSERT_EQUAL_INT(GNSSMGR_ANT_SHORT, gnssmgr_ant_get_state(&g_mgr));
	TEST_ASSERT_TRUE(gnssmgr_ant_short_latched(&g_mgr));
	TEST_ASSERT_NOT_EQUAL(0U, gnssmgr_alarms(&g_mgr) &
				      (1U << GNSSMGR_ALARM_ANT_SHORT));
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_ant_reenable(&g_mgr));
	TEST_ASSERT_EQUAL_UINT(0U, g_fake.bias_calls);
}

/*
 * The scratch buffer must hold the largest step with room to spare. The TP1
 * group is the biggest at fourteen keys; if a future step outgrows
 * GNSSMGR_TXBUF_SZ the builder latches -ENOSPC and the manager reports
 * CONFIG_FAILED, which is safe but useless, so measure the headroom here
 * instead of discovering it on hardware.
 */
static void test_every_step_fits_the_scratch_buffer(void)
{
	size_t largest = 0U;
	uint32_t t = 1000U;
	unsigned int i;

	setup_mgr(NULL);
	TEST_ASSERT_EQUAL_INT(0, gnssmgr_start(&g_mgr, t));
	for (i = 1U; i <= WALK_STEPS; i++) {
		TEST_ASSERT_TRUE(g_fake.frame_len > UBX_FRAME_OVERHEAD);
		TEST_ASSERT_TRUE(g_fake.frame_len < GNSSMGR_TXBUF_SZ);
		assert_frame_well_formed(&g_fake);
		if (g_fake.frame_len > largest) {
			largest = g_fake.frame_len;
		}
		t += 10U;
		send_ack(true, t);
	}
	/* The TP1 group: 4 VALSET prefix + 14 keys + 30 value bytes + 8. */
	TEST_ASSERT_EQUAL_UINT(98U, largest);
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_cfg_defaults);
	RUN_TEST(test_init_rejects);
	RUN_TEST(test_init_config_validation);
	RUN_TEST(test_init_leaves_manager_idle);
	RUN_TEST(test_null_arguments);

	RUN_TEST(test_cold_config_walk_to_survey);
	RUN_TEST(test_stored_position_skips_survey);
	RUN_TEST(test_fixed_pos_acc_override);
	RUN_TEST(test_txready_disabled_is_never_trusted);
	RUN_TEST(test_gps_timegrid_option);

	RUN_TEST(test_ack_timeout_retries_then_fails);
	RUN_TEST(test_nak_retries_immediately);
	RUN_TEST(test_foreign_ack_ignored);
	RUN_TEST(test_send_failure_is_reported_and_retried);
	RUN_TEST(test_ack_deadline_survives_clock_wrap);

	RUN_TEST(test_survey_in_progress_then_fixed);
	RUN_TEST(test_request_survey_is_explicit_only);
	RUN_TEST(test_request_survey_mid_walk_does_not_skip_steps);
	RUN_TEST(test_request_survey_reissues_outstanding_tmode);
	RUN_TEST(test_reset_keeps_leap_schedule);

	RUN_TEST(test_time_lock_thresholds_and_hysteresis);
	RUN_TEST(test_time_lock_requires_fix_and_utc);
	RUN_TEST(test_stale_pvt_drops_lock);

	RUN_TEST(test_leap_tracking_and_indicator);
	RUN_TEST(test_qerr_pairing);
	RUN_TEST(test_nav_sat_summary);
	RUN_TEST(test_malformed_known_messages);

	RUN_TEST(test_antenna_first_verdict_is_debounced);
	RUN_TEST(test_antenna_short_debounce_drops_bias);
	RUN_TEST(test_antenna_open_alarms_without_cutting_bias);
	RUN_TEST(test_antenna_commanded_off_masks_faults);
	RUN_TEST(test_antenna_mon_rf_only);
	RUN_TEST(test_antenna_short_outranks_open_and_ok);
	RUN_TEST(test_antenna_debounce_needs_consecutive_samples);
	RUN_TEST(test_antenna_debounce_of_one_is_immediate);

	RUN_TEST(test_notify_reset_reruns_config);
	RUN_TEST(test_itow_backstep_detection);
	RUN_TEST(test_optional_callbacks_may_be_null);
	RUN_TEST(test_every_step_fits_the_scratch_buffer);

	return UNITY_END();
}
