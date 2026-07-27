/*
 * STS1000 "Meridian" — host unit tests for core/fwupd/ubx_fwupd.c.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The invariant this suite exists to prove: **after any failure, abort or
 * timeout, GPS_SAFEBOOT_N is released and the receiver has been reset.** A
 * receiver left in safeboot does not run its application, which means no PPS and
 * a unit that looks dead — so every injected failure below ends with
 * ubx_fwupd_in_safeboot() asserted false and the fake's pin trace checked.
 *
 * The second property: with the opcode table unverified (the default),
 * ubx_fwupd_begin() must refuse *without touching a pin at all*.
 */

#include <errno.h>
#include <string.h>

#include "fwupd/ubx_fwupd.h"
#include "unity.h"

/* ===================================================================== *
 *  Fake receiver
 * ===================================================================== */

#define FIFO 4096U

typedef struct {
	/* host -> receiver */
	uint8_t tx[FIFO];
	size_t tx_len;
	/* receiver -> host */
	uint8_t rx[FIFO];
	size_t rx_head;
	size_t rx_len;

	uint64_t now;
	uint32_t baud;
	bool safeboot;   /**< pin currently asserted low */
	bool in_reset;

	/* pin trace */
	unsigned int reset_pulses;
	unsigned int safeboot_asserts;
	unsigned int safeboot_releases;
	/* True if a reset was ever released while safeboot was still asserted
	 * AFTER the session ended — i.e. the receiver was left in the loader. */
	bool left_in_loader;
	bool session_over;

	/* behaviour */
	uint32_t loader_baud;   /**< baud at which IDENT is answered; 0 = never */
	bool nak_erase;
	bool silent_erase;
	unsigned int nak_writes;   /**< NAK this many WRITEs, then ACK */
	unsigned int silent_writes;/**< ignore this many WRITEs, then ACK */
	bool nak_finalise;
	bool no_mon_ver;
	bool bad_mon_ver;
	int rc_tx;
	int rc_rx;
	int rc_set_baud;
	int rc_safeboot;
	int rc_reset;

	unsigned int writes_seen;
	uint32_t last_write_off;
	unsigned int idents_seen;
} rcv_t;

static void rcv_reset(rcv_t *r)
{
	(void)memset(r, 0, sizeof(*r));
	r->baud = 38400U;
	r->loader_baud = 9600U;
}

static void push_rx(rcv_t *r, const uint8_t *d, size_t n)
{
	if ((r->rx_len + n) > sizeof(r->rx)) {
		return;
	}
	(void)memcpy(&r->rx[r->rx_len], d, n);
	r->rx_len += n;
}

/* Emit UBX-ACK-ACK or ACK-NAK for (cls, id). */
static void push_ack(rcv_t *r, uint8_t cls, uint8_t id, bool ack)
{
	uint8_t pl[2] = { cls, id };
	uint8_t f[32];
	int n = ubx_frame(f, sizeof(f), UBX_CLASS_ACK,
			  ack ? UBX_ID_ACK_ACK : UBX_ID_ACK_NAK, pl, 2U);

	if (n > 0) {
		push_rx(r, f, (size_t)n);
	}
}

static void push_frame(rcv_t *r, uint8_t cls, uint8_t id, const uint8_t *pl,
		       uint16_t len)
{
	uint8_t f[256];
	int n = ubx_frame(f, sizeof(f), cls, id, pl, len);

	if (n > 0) {
		push_rx(r, f, (size_t)n);
	}
}

static void push_mon_ver(rcv_t *r, const char *sw, const char *hw,
			 const char *ext)
{
	uint8_t pl[UBX_MON_VER_MIN_LEN + UBX_MON_VER_EXT_LEN];
	size_t n;

	(void)memset(pl, 0, sizeof(pl));
	n = strlen(sw);
	if (n > UBX_MON_VER_SW_LEN) {
		n = UBX_MON_VER_SW_LEN;
	}
	(void)memcpy(pl, sw, n);
	n = strlen(hw);
	if (n > UBX_MON_VER_HW_LEN) {
		n = UBX_MON_VER_HW_LEN;
	}
	(void)memcpy(&pl[UBX_MON_VER_SW_LEN], hw, n);

	if (ext == NULL) {
		push_frame(r, UBX_CLASS_MON, UBX_ID_MON_VER, pl,
			   (uint16_t)UBX_MON_VER_MIN_LEN);
		return;
	}
	n = strlen(ext);
	if (n > UBX_MON_VER_EXT_LEN) {
		n = UBX_MON_VER_EXT_LEN;
	}
	(void)memcpy(&pl[UBX_MON_VER_MIN_LEN], ext, n);
	push_frame(r, UBX_CLASS_MON, UBX_ID_MON_VER, pl, (uint16_t)sizeof(pl));
}

/*
 * Consume whatever the host sent and react. Runs inside the tx callback, which
 * is how the fake stays synchronous: by the time the module starts polling for a
 * reply, the reply is already queued.
 */
static void react(rcv_t *r, const uint8_t *d, size_t len)
{
	size_t i = 0U;

	/* The loader only answers at its own baud. */
	while ((i + UBX_FRAME_OVERHEAD) <= len) {
		uint8_t cls;
		uint8_t id;
		uint16_t plen;

		if ((d[i] != UBX_SYNC1) || (d[i + 1U] != UBX_SYNC2)) {
			i++;
			continue;
		}
		cls = d[i + 2U];
		id = d[i + 3U];
		plen = (uint16_t)((uint16_t)d[i + 4U] | ((uint16_t)d[i + 5U] << 8));

		if (cls == UBX_CLASS_MON && id == UBX_ID_MON_VER) {
			if (!r->no_mon_ver) {
				if (r->bad_mon_ver) {
					/* A truncated MON-VER: not a whole
					 * extension string, so it must be
					 * rejected rather than half-parsed. */
					uint8_t pl[UBX_MON_VER_MIN_LEN + 5U];

					(void)memset(pl, 'x', sizeof(pl));
					push_frame(r, UBX_CLASS_MON,
						   UBX_ID_MON_VER, pl,
						   (uint16_t)sizeof(pl));
				} else {
					push_mon_ver(r, "EXT CORE 1.00 (a1b2c3)",
						     "00190000",
						     "FWVER=TIM 2.30");
				}
			}
		} else if (cls == UBX_FWUPD_CLASS) {
			switch (id) {
			case UBX_FWUPD_ID_IDENT:
				r->idents_seen++;
				if ((r->loader_baud != 0U) &&
				    (r->baud == r->loader_baud) && r->safeboot) {
					push_ack(r, cls, id, true);
				}
				break;
			case UBX_FWUPD_ID_ERASE:
				if (!r->silent_erase) {
					push_ack(r, cls, id, !r->nak_erase);
				}
				break;
			case UBX_FWUPD_ID_WRITE:
				r->writes_seen++;
				if ((i + 6U + 4U) <= len) {
					r->last_write_off =
						(uint32_t)d[i + 6U] |
						((uint32_t)d[i + 7U] << 8) |
						((uint32_t)d[i + 8U] << 16) |
						((uint32_t)d[i + 9U] << 24);
				}
				if (r->silent_writes > 0U) {
					r->silent_writes--;
				} else if (r->nak_writes > 0U) {
					r->nak_writes--;
					push_ack(r, cls, id, false);
				} else {
					push_ack(r, cls, id, true);
				}
				break;
			case UBX_FWUPD_ID_FINALISE:
				push_ack(r, cls, id, !r->nak_finalise);
				break;
			default:
				break;
			}
		} else {
			/* not for the fake */
		}
		i += (size_t)UBX_FRAME_OVERHEAD + plen;
	}
}

static int o_tx(void *u, const uint8_t *d, size_t len)
{
	rcv_t *r = u;

	if (r->rc_tx != 0) {
		return r->rc_tx;
	}
	if ((r->tx_len + len) <= sizeof(r->tx)) {
		(void)memcpy(&r->tx[r->tx_len], d, len);
		r->tx_len += len;
	}
	react(r, d, len);
	return 0;
}

static int o_rx(void *u, uint8_t *d, size_t cap)
{
	rcv_t *r = u;
	size_t avail;

	if (r->rc_rx != 0) {
		return r->rc_rx;
	}
	avail = r->rx_len - r->rx_head;
	if (avail == 0U) {
		return 0;
	}
	if (avail > cap) {
		avail = cap;
	}
	(void)memcpy(d, &r->rx[r->rx_head], avail);
	r->rx_head += avail;
	return (int)avail;
}

static int o_set_baud(void *u, uint32_t baud)
{
	rcv_t *r = u;

	if (r->rc_set_baud != 0) {
		return r->rc_set_baud;
	}
	r->baud = baud;
	/* A baud change drops anything mid-flight, like real hardware. */
	r->rx_head = 0U;
	r->rx_len = 0U;
	return 0;
}

static int o_safeboot(void *u, bool assert_low)
{
	rcv_t *r = u;

	if (r->rc_safeboot != 0) {
		return r->rc_safeboot;
	}
	if (assert_low && !r->safeboot) {
		r->safeboot_asserts++;
	}
	if (!assert_low && r->safeboot) {
		r->safeboot_releases++;
	}
	r->safeboot = assert_low;
	return 0;
}

static int o_reset(void *u, bool assert_low)
{
	rcv_t *r = u;

	if (r->rc_reset != 0) {
		return r->rc_reset;
	}
	if (r->in_reset && !assert_low) {
		r->reset_pulses++;
		/* Coming out of reset with safeboot still low = into the loader. */
		if (r->safeboot && r->session_over) {
			r->left_in_loader = true;
		}
	}
	r->in_reset = assert_low;
	return 0;
}

static uint64_t o_now(void *u)
{
	rcv_t *r = u;

	/* Time advances on every read, so the timeout loops terminate. */
	r->now += 1U;
	return r->now;
}

static void o_delay(void *u, uint32_t ms)
{
	rcv_t *r = u;

	r->now += ms;
}

static void ops_from(ubx_fwupd_ops_t *o, rcv_t *r)
{
	(void)memset(o, 0, sizeof(*o));
	o->tx = o_tx;
	o->rx = o_rx;
	o->set_baud = o_set_baud;
	o->set_safeboot = o_safeboot;
	o->set_reset = o_reset;
	o->now_ms = o_now;
	o->delay_ms = o_delay;
	o->user = r;
}

static void armed_cfg(ubx_fwupd_cfg_t *cfg)
{
	ubx_fwupd_cfg_defaults(cfg);
	cfg->opcodes_verified = true;
	cfg->chunk = 256U;
}

/* The invariant. */
static void assert_recovered(const ubx_fwupd_t *u, const rcv_t *r)
{
	TEST_ASSERT_FALSE(ubx_fwupd_in_safeboot(u));
	TEST_ASSERT_FALSE(r->safeboot);
	TEST_ASSERT_FALSE(r->in_reset);
	/* At least one reset pulse was issued to get it out of the loader. */
	TEST_ASSERT_TRUE(r->reset_pulses >= 1U);
	/* The UART is back at the operating baud. */
	TEST_ASSERT_EQUAL_UINT32(38400U, r->baud);
}

static uint8_t img[1024];

static void make_img(void)
{
	size_t i;

	for (i = 0U; i < sizeof(img); i++) {
		img[i] = (uint8_t)(i ^ 0x5AU);
	}
}

/* ===================================================================== *
 *  The gate
 * ===================================================================== */

/*
 * With the opcode table unverified — the default — begin() must refuse WITHOUT
 * touching a pin. That is what makes it safe to carry an unverified opcode table
 * in the tree at all: a disarmed module cannot leave the receiver in safeboot
 * even in principle.
 */
static void test_unverified_opcodes_touch_nothing(void)
{
	ubx_fwupd_t u;
	ubx_fwupd_cfg_t cfg;
	ubx_fwupd_ops_t ops;
	rcv_t r;

	rcv_reset(&r);
	ops_from(&ops, &r);
	ubx_fwupd_cfg_defaults(&cfg);
	TEST_ASSERT_FALSE(cfg.opcodes_verified);
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));

	TEST_ASSERT_EQUAL_INT(-ENOTSUP, ubx_fwupd_begin(&u, 1024U));

	/* Not one pin moved, no reset, no baud change. */
	TEST_ASSERT_EQUAL_UINT(0U, r.safeboot_asserts);
	TEST_ASSERT_EQUAL_UINT(0U, r.reset_pulses);
	TEST_ASSERT_FALSE(r.safeboot);
	TEST_ASSERT_FALSE(r.in_reset);
	TEST_ASSERT_EQUAL_UINT32(38400U, r.baud);
	TEST_ASSERT_EQUAL_UINT(0U, r.tx_len);
	TEST_ASSERT_FALSE(ubx_fwupd_in_safeboot(&u));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UBX_FWUPD_ST_IDLE, ubx_fwupd_state(&u));
}

static void test_init_rejects(void)
{
	ubx_fwupd_t u;
	ubx_fwupd_cfg_t cfg;
	ubx_fwupd_ops_t ops;
	rcv_t r;

	rcv_reset(&r);
	ubx_fwupd_cfg_defaults(&cfg);
	ops_from(&ops, &r);

	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_init(NULL, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_init(&u, NULL, &ops));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_init(&u, &cfg, NULL));

	/* Every mandatory callback in turn. */
	{
		ubx_fwupd_ops_t bad = ops;

		bad.tx = NULL;
		TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_init(&u, &cfg, &bad));
		bad = ops;
		bad.rx = NULL;
		TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_init(&u, &cfg, &bad));
		bad = ops;
		bad.set_baud = NULL;
		TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_init(&u, &cfg, &bad));
		bad = ops;
		bad.set_safeboot = NULL;
		TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_init(&u, &cfg, &bad));
		bad = ops;
		bad.set_reset = NULL;
		TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_init(&u, &cfg, &bad));
		bad = ops;
		bad.now_ms = NULL;
		TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_init(&u, &cfg, &bad));
	}

	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_begin(&u, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_begin(NULL, 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_recover(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_abort(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_step(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_write(NULL, 0U, img, 4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_finish(NULL));
	TEST_ASSERT_FALSE(ubx_fwupd_in_safeboot(NULL));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UBX_FWUPD_ST_IDLE, ubx_fwupd_state(NULL));
	TEST_ASSERT_NULL(ubx_fwupd_version(NULL));
	TEST_ASSERT_NULL(ubx_fwupd_version(&u));
}

/* ===================================================================== *
 *  Version query
 * ===================================================================== */

static void test_query_version(void)
{
	ubx_fwupd_t u;
	ubx_fwupd_cfg_t cfg;
	ubx_fwupd_ops_t ops;
	rcv_t r;
	char buf[64];

	rcv_reset(&r);
	ops_from(&ops, &r);
	armed_cfg(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));

	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_query_version(&u, buf, sizeof(buf)));
	/* swVersion plus the FWVER extension, which is what an update moves. */
	TEST_ASSERT_EQUAL_STRING("EXT CORE 1.00 (a1b2c3) TIM 2.30", buf);
	TEST_ASSERT_NOT_NULL(ubx_fwupd_version(&u));
	TEST_ASSERT_EQUAL_STRING("00190000", ubx_fwupd_version(&u)->hw_version);

	/* A silent receiver times out rather than hanging. */
	rcv_reset(&r);
	r.no_mon_ver = true;
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(-ETIMEDOUT,
		ubx_fwupd_query_version(&u, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_STRING("", buf);

	/* A malformed MON-VER is refused, not half-parsed. */
	rcv_reset(&r);
	r.bad_mon_ver = true;
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
		ubx_fwupd_query_version(&u, buf, sizeof(buf)));

	/* A transport error surfaces. */
	rcv_reset(&r);
	r.rc_tx = -EIO;
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(-EIO, ubx_fwupd_query_version(&u, buf, sizeof(buf)));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_query_version(NULL, buf, 8U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_query_version(&u, NULL, 8U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_query_version(&u, buf, 0U));
}

/* ===================================================================== *
 *  Happy path
 * ===================================================================== */

static void test_full_update(void)
{
	ubx_fwupd_t u;
	ubx_fwupd_cfg_t cfg;
	ubx_fwupd_ops_t ops;
	rcv_t r;
	uint32_t off;

	make_img();
	rcv_reset(&r);
	ops_from(&ops, &r);
	armed_cfg(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));

	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_begin(&u, sizeof(img)));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UBX_FWUPD_ST_WRITE, ubx_fwupd_state(&u));

	/* Safeboot was entered: pin asserted, then a reset pulse. */
	TEST_ASSERT_EQUAL_UINT(1U, r.safeboot_asserts);
	TEST_ASSERT_TRUE(r.reset_pulses >= 1U);
	TEST_ASSERT_TRUE(ubx_fwupd_in_safeboot(&u));
	/* And the loader answered, so the bulk baud was selected. */
	TEST_ASSERT_TRUE(r.idents_seen >= 1U);
	TEST_ASSERT_EQUAL_UINT32((uint32_t)UBX_FWUPD_XFER_BAUD, r.baud);

	for (off = 0U; off < (uint32_t)sizeof(img); off += 256U) {
		TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_write(&u, off, &img[off], 256U));
	}
	TEST_ASSERT_EQUAL_UINT(sizeof(img) / 256U, r.writes_seen);

	r.session_over = true;
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_finish(&u));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UBX_FWUPD_ST_DONE, ubx_fwupd_state(&u));

	/* Left safeboot, rebooted, and confirmed the version. */
	assert_recovered(&u, &r);
	TEST_ASSERT_FALSE(r.left_in_loader);
	TEST_ASSERT_NOT_NULL(ubx_fwupd_version(&u));
	TEST_ASSERT_EQUAL_STRING("TIM 2.30",
		ubx_mon_ver_ext(ubx_fwupd_version(&u), "FWVER"));
}

/* Offsets must be sequential, and a chunk must fit. */
static void test_write_argument_rules(void)
{
	ubx_fwupd_t u;
	ubx_fwupd_cfg_t cfg;
	ubx_fwupd_ops_t ops;
	rcv_t r;

	make_img();
	rcv_reset(&r);
	ops_from(&ops, &r);
	armed_cfg(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));

	/* Not in a session yet. */
	TEST_ASSERT_EQUAL_INT(-EPERM, ubx_fwupd_write(&u, 0U, img, 16U));
	TEST_ASSERT_EQUAL_INT(-EPERM, ubx_fwupd_finish(&u));

	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_begin(&u, sizeof(img)));
	TEST_ASSERT_EQUAL_INT(-EBUSY, ubx_fwupd_begin(&u, sizeof(img)));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_write(&u, 0U, NULL, 16U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_write(&u, 0U, img, 0U));
	/* Over the configured chunk size. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_write(&u, 0U, img, 257U));
	/* The loader has no seek. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_write(&u, 256U, img, 16U));
	/* Past the declared image. */
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_write(&u, 0U, img, 256U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_fwupd_write(&u, 256U,
			&img[256], 256U + 1U));

	/* finish() before the whole image is written is an error, and recovers. */
	r.session_over = true;
	TEST_ASSERT_EQUAL_INT(-EIO, ubx_fwupd_finish(&u));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UBX_FWUPD_ST_FAILED, ubx_fwupd_state(&u));
	assert_recovered(&u, &r);
	TEST_ASSERT_FALSE(r.left_in_loader);
}

/* ===================================================================== *
 *  Chunk retry
 * ===================================================================== */

static void test_chunk_retry_on_nak_then_succeeds(void)
{
	ubx_fwupd_t u;
	ubx_fwupd_cfg_t cfg;
	ubx_fwupd_ops_t ops;
	rcv_t r;

	make_img();
	rcv_reset(&r);
	/* NAK the first two attempts; the third is acknowledged. */
	r.nak_writes = 2U;
	ops_from(&ops, &r);
	armed_cfg(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_begin(&u, 256U));

	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_write(&u, 0U, img, 256U));
	/* Three attempts reached the receiver for one chunk. */
	TEST_ASSERT_EQUAL_UINT(3U, r.writes_seen);
	TEST_ASSERT_TRUE(u.retries >= 2U);
	/* Still in the write phase, still in safeboot: a retry is not a failure. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UBX_FWUPD_ST_WRITE, ubx_fwupd_state(&u));
	TEST_ASSERT_TRUE(ubx_fwupd_in_safeboot(&u));

	r.session_over = true;
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_finish(&u));
	assert_recovered(&u, &r);
}

static void test_chunk_retry_on_silence_then_succeeds(void)
{
	ubx_fwupd_t u;
	ubx_fwupd_cfg_t cfg;
	ubx_fwupd_ops_t ops;
	rcv_t r;

	make_img();
	rcv_reset(&r);
	r.silent_writes = 1U; /* one timeout, then an ACK */
	ops_from(&ops, &r);
	armed_cfg(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_begin(&u, 256U));

	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_write(&u, 0U, img, 256U));
	TEST_ASSERT_EQUAL_UINT(2U, r.writes_seen);
	TEST_ASSERT_TRUE(u.timeouts >= 1U);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UBX_FWUPD_ST_WRITE, ubx_fwupd_state(&u));
}

/*
 * A chunk that fails every retry leaves a partially-written image — which is
 * precisely the state that must not be left sitting behind safeboot.
 */
static void test_chunk_exhausts_retries_and_recovers(void)
{
	ubx_fwupd_t u;
	ubx_fwupd_cfg_t cfg;
	ubx_fwupd_ops_t ops;
	rcv_t r;

	make_img();
	rcv_reset(&r);
	r.nak_writes = 0xFFFFU; /* never acknowledge */
	ops_from(&ops, &r);
	armed_cfg(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_begin(&u, 512U));
	r.session_over = true;

	TEST_ASSERT_EQUAL_INT(-EIO, ubx_fwupd_write(&u, 0U, img, 256U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UBX_FWUPD_ST_FAILED, ubx_fwupd_state(&u));
	/* One initial attempt plus UBX_FWUPD_CHUNK_RETRIES. */
	TEST_ASSERT_EQUAL_UINT((unsigned int)UBX_FWUPD_CHUNK_RETRIES + 1U,
			       r.writes_seen);
	assert_recovered(&u, &r);
	TEST_ASSERT_FALSE(r.left_in_loader);
}

/* ===================================================================== *
 *  Fail-safe: every failure path releases safeboot
 * ===================================================================== */

static void test_no_loader_recovers(void)
{
	ubx_fwupd_t u;
	ubx_fwupd_cfg_t cfg;
	ubx_fwupd_ops_t ops;
	rcv_t r;

	rcv_reset(&r);
	r.loader_baud = 0U; /* nothing answers IDENT at any baud */
	ops_from(&ops, &r);
	armed_cfg(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));
	r.session_over = true;

	TEST_ASSERT_EQUAL_INT(-ENODEV, ubx_fwupd_begin(&u, 1024U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UBX_FWUPD_ST_FAILED, ubx_fwupd_state(&u));
	/* Every baud in the list was tried, twice each. */
	TEST_ASSERT_EQUAL_UINT((unsigned int)UBX_FWUPD_BAUD_COUNT *
			       (unsigned int)UBX_FWUPD_PROBE_RETRIES,
			       r.idents_seen);
	assert_recovered(&u, &r);
	TEST_ASSERT_FALSE(r.left_in_loader);
}

/* Baud negotiation finds a loader that is not at the first rate tried. */
static void test_baud_negotiation_scans(void)
{
	unsigned int i;

	for (i = 0U; i < (unsigned int)UBX_FWUPD_BAUD_COUNT; i++) {
		ubx_fwupd_t u;
		ubx_fwupd_cfg_t cfg;
		ubx_fwupd_ops_t ops;
		rcv_t r;

		rcv_reset(&r);
		r.loader_baud = ubx_fwupd_baud_list[i];
		ops_from(&ops, &r);
		armed_cfg(&cfg);
		TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));

		TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_begin(&u, 256U));
		TEST_ASSERT_EQUAL_UINT8((uint8_t)UBX_FWUPD_ST_WRITE,
					ubx_fwupd_state(&u));
		TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_abort(&u));
		assert_recovered(&u, &r);
	}
}

static void test_erase_failures_recover(void)
{
	ubx_fwupd_t u;
	ubx_fwupd_cfg_t cfg;
	ubx_fwupd_ops_t ops;
	rcv_t r;

	/* NAK. */
	rcv_reset(&r);
	r.nak_erase = true;
	ops_from(&ops, &r);
	armed_cfg(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));
	r.session_over = true;
	TEST_ASSERT_EQUAL_INT(-EIO, ubx_fwupd_begin(&u, 1024U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UBX_FWUPD_ST_FAILED, ubx_fwupd_state(&u));
	assert_recovered(&u, &r);
	TEST_ASSERT_FALSE(r.left_in_loader);

	/* Silence. */
	rcv_reset(&r);
	r.silent_erase = true;
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));
	r.session_over = true;
	TEST_ASSERT_EQUAL_INT(-ETIMEDOUT, ubx_fwupd_begin(&u, 1024U));
	assert_recovered(&u, &r);
	TEST_ASSERT_FALSE(r.left_in_loader);
}

static void test_finalise_and_verify_failures_recover(void)
{
	ubx_fwupd_t u;
	ubx_fwupd_cfg_t cfg;
	ubx_fwupd_ops_t ops;
	rcv_t r;

	make_img();

	/* FINALISE is NAKed. */
	rcv_reset(&r);
	r.nak_finalise = true;
	ops_from(&ops, &r);
	armed_cfg(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_begin(&u, 256U));
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_write(&u, 0U, img, 256U));
	r.session_over = true;
	TEST_ASSERT_EQUAL_INT(-EIO, ubx_fwupd_finish(&u));
	assert_recovered(&u, &r);
	TEST_ASSERT_FALSE(r.left_in_loader);

	/*
	 * The image committed but the receiver never came back. This must NOT be
	 * reported as success — an operator has to know the difference between
	 * "updated" and "wrote an image and then lost the receiver" — and the
	 * receiver must still be out of safeboot.
	 */
	rcv_reset(&r);
	r.no_mon_ver = true;
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_begin(&u, 256U));
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_write(&u, 0U, img, 256U));
	r.session_over = true;
	TEST_ASSERT_EQUAL_INT(-ETIMEDOUT, ubx_fwupd_finish(&u));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UBX_FWUPD_ST_FAILED, ubx_fwupd_state(&u));
	assert_recovered(&u, &r);
	TEST_ASSERT_FALSE(r.left_in_loader);
}

static void test_abort_at_every_phase(void)
{
	unsigned int phase;

	make_img();
	for (phase = 0U; phase < 3U; phase++) {
		ubx_fwupd_t u;
		ubx_fwupd_cfg_t cfg;
		ubx_fwupd_ops_t ops;
		rcv_t r;

		rcv_reset(&r);
		ops_from(&ops, &r);
		armed_cfg(&cfg);
		TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));

		if (phase >= 1U) {
			TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_begin(&u, 512U));
		}
		if (phase >= 2U) {
			TEST_ASSERT_EQUAL_INT(0,
				ubx_fwupd_write(&u, 0U, img, 256U));
		}

		r.session_over = true;
		TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_abort(&u));
		TEST_ASSERT_FALSE(ubx_fwupd_in_safeboot(&u));
		TEST_ASSERT_FALSE(r.safeboot);
		TEST_ASSERT_FALSE(r.left_in_loader);

		/* Abort is idempotent. */
		TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_abort(&u));
		TEST_ASSERT_FALSE(ubx_fwupd_in_safeboot(&u));
	}
}

/*
 * A tool that opens a session and disappears must not leave the receiver behind
 * safeboot forever: step() enforces a stall timeout, and the timeout recovers.
 */
static void test_stalled_session_times_out_and_recovers(void)
{
	ubx_fwupd_t u;
	ubx_fwupd_cfg_t cfg;
	ubx_fwupd_ops_t ops;
	rcv_t r;
	int rc = 0;
	unsigned int i;

	make_img();
	rcv_reset(&r);
	ops_from(&ops, &r);
	armed_cfg(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_begin(&u, 4096U));
	TEST_ASSERT_TRUE(ubx_fwupd_in_safeboot(&u));
	r.session_over = true;

	/* step() reports -EAGAIN while inside the budget, then fails. */
	for (i = 0U; i < 10000U; i++) {
		rc = ubx_fwupd_step(&u);
		if (rc != -EAGAIN) {
			break;
		}
		r.now += 10U;
	}
	TEST_ASSERT_EQUAL_INT(-ETIMEDOUT, rc);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UBX_FWUPD_ST_FAILED, ubx_fwupd_state(&u));
	assert_recovered(&u, &r);
	TEST_ASSERT_FALSE(r.left_in_loader);

	/* step() on a finished session is a no-op. */
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_step(&u));
}

/* A GPIO failure during recovery is the one case the receiver may stay held —
 * and it must be reported rather than swallowed. */
static void test_recover_reports_gpio_failure(void)
{
	ubx_fwupd_t u;
	ubx_fwupd_cfg_t cfg;
	ubx_fwupd_ops_t ops;
	rcv_t r;

	rcv_reset(&r);
	ops_from(&ops, &r);
	armed_cfg(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));

	r.rc_safeboot = -EIO;
	TEST_ASSERT_EQUAL_INT(-EIO, ubx_fwupd_recover(&u));

	r.rc_safeboot = 0;
	r.rc_reset = -EIO;
	TEST_ASSERT_EQUAL_INT(-EIO, ubx_fwupd_recover(&u));

	r.rc_reset = 0;
	r.rc_set_baud = -EIO;
	TEST_ASSERT_EQUAL_INT(-EIO, ubx_fwupd_recover(&u));

	r.rc_set_baud = 0;
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_recover(&u));
	TEST_ASSERT_FALSE(ubx_fwupd_in_safeboot(&u));
}

/* Entering safeboot can fail too, and that path must also recover. */
static void test_enter_safeboot_failure_recovers(void)
{
	ubx_fwupd_t u;
	ubx_fwupd_cfg_t cfg;
	ubx_fwupd_ops_t ops;
	rcv_t r;

	rcv_reset(&r);
	ops_from(&ops, &r);
	armed_cfg(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));

	r.rc_reset = -EIO;
	TEST_ASSERT_EQUAL_INT(-EIO, ubx_fwupd_begin(&u, 1024U));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UBX_FWUPD_ST_FAILED, ubx_fwupd_state(&u));
	/* The module no longer believes it holds the pin. */
	TEST_ASSERT_FALSE(ubx_fwupd_in_safeboot(&u));

	/* A transmit failure during negotiation. */
	rcv_reset(&r);
	r.rc_tx = -EIO;
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));
	r.session_over = true;
	TEST_ASSERT_EQUAL_INT(-EIO, ubx_fwupd_begin(&u, 1024U));
	assert_recovered(&u, &r);

	/* A receive failure during negotiation. */
	rcv_reset(&r);
	r.rc_rx = -EIO;
	TEST_ASSERT_EQUAL_INT(0, ubx_fwupd_init(&u, &cfg, &ops));
	r.session_over = true;
	TEST_ASSERT_EQUAL_INT(-EIO, ubx_fwupd_begin(&u, 1024U));
	assert_recovered(&u, &r);
}

static void test_state_names(void)
{
	unsigned int i;

	for (i = 0U; i < (unsigned int)UBX_FWUPD_ST__COUNT; i++) {
		TEST_ASSERT_NOT_NULL(ubx_fwupd_state_name((uint8_t)i));
		TEST_ASSERT_TRUE(ubx_fwupd_state_name((uint8_t)i)[0] != '?');
	}
	TEST_ASSERT_EQUAL_STRING("?", ubx_fwupd_state_name(0xFFU));
}

/* ===================================================================== *
 *  UBX-MON-VER decoding
 * ===================================================================== */

static void test_mon_ver_parser(void)
{
	uint8_t pl[UBX_MON_VER_MIN_LEN + (2U * UBX_MON_VER_EXT_LEN)];
	ubx_msg_t m;
	ubx_mon_ver_t v;

	(void)memset(pl, 0, sizeof(pl));
	(void)memcpy(pl, "SW 1.23", 7U);
	(void)memcpy(&pl[UBX_MON_VER_SW_LEN], "HW9", 3U);
	(void)memcpy(&pl[UBX_MON_VER_MIN_LEN], "FWVER=TIM 2.20", 14U);
	(void)memcpy(&pl[UBX_MON_VER_MIN_LEN + UBX_MON_VER_EXT_LEN],
		     "PROTVER=32.00", 13U);

	(void)memset(&m, 0, sizeof(m));
	m.cls = UBX_CLASS_MON;
	m.id = UBX_ID_MON_VER;
	m.payload = pl;
	m.len = (uint16_t)sizeof(pl);

	TEST_ASSERT_EQUAL_INT(0, ubx_parse_mon_ver(&m, &v));
	TEST_ASSERT_EQUAL_STRING("SW 1.23", v.sw_version);
	TEST_ASSERT_EQUAL_STRING("HW9", v.hw_version);
	TEST_ASSERT_EQUAL_UINT8(2U, v.n_ext);
	TEST_ASSERT_EQUAL_STRING("TIM 2.20", ubx_mon_ver_ext(&v, "FWVER"));
	TEST_ASSERT_EQUAL_STRING("32.00", ubx_mon_ver_ext(&v, "PROTVER"));
	TEST_ASSERT_NULL(ubx_mon_ver_ext(&v, "NOPE"));
	TEST_ASSERT_NULL(ubx_mon_ver_ext(&v, ""));
	TEST_ASSERT_NULL(ubx_mon_ver_ext(NULL, "FWVER"));
	TEST_ASSERT_NULL(ubx_mon_ver_ext(&v, NULL));

	/* A full-width field with no room for a terminator is still terminated. */
	(void)memset(pl, 'A', UBX_MON_VER_SW_LEN);
	TEST_ASSERT_EQUAL_INT(0, ubx_parse_mon_ver(&m, &v));
	TEST_ASSERT_EQUAL_UINT(UBX_MON_VER_SW_LEN, strlen(v.sw_version));

	/* Rejections. */
	m.len = (uint16_t)(UBX_MON_VER_MIN_LEN - 1U);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ubx_parse_mon_ver(&m, &v));
	m.len = (uint16_t)(UBX_MON_VER_MIN_LEN + 7U);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ubx_parse_mon_ver(&m, &v));
	m.len = (uint16_t)UBX_MON_VER_MIN_LEN;
	TEST_ASSERT_EQUAL_INT(0, ubx_parse_mon_ver(&m, &v));
	TEST_ASSERT_EQUAL_UINT8(0U, v.n_ext);
	m.id = 0x99U;
	TEST_ASSERT_EQUAL_INT(-ENOMSG, ubx_parse_mon_ver(&m, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_parse_mon_ver(NULL, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ubx_parse_mon_ver(&m, NULL));
}

/* ===================================================================== *
 *  Runner
 * ===================================================================== */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_unverified_opcodes_touch_nothing);
	RUN_TEST(test_init_rejects);
	RUN_TEST(test_query_version);

	RUN_TEST(test_full_update);
	RUN_TEST(test_write_argument_rules);

	RUN_TEST(test_chunk_retry_on_nak_then_succeeds);
	RUN_TEST(test_chunk_retry_on_silence_then_succeeds);
	RUN_TEST(test_chunk_exhausts_retries_and_recovers);

	RUN_TEST(test_no_loader_recovers);
	RUN_TEST(test_baud_negotiation_scans);
	RUN_TEST(test_erase_failures_recover);
	RUN_TEST(test_finalise_and_verify_failures_recover);
	RUN_TEST(test_abort_at_every_phase);
	RUN_TEST(test_stalled_session_times_out_and_recovers);
	RUN_TEST(test_recover_reports_gpio_failure);
	RUN_TEST(test_enter_safeboot_failure_recovers);
	RUN_TEST(test_state_names);

	RUN_TEST(test_mon_ver_parser);

	return UNITY_END();
}
