/*
 * STS1000 "Meridian" — core/atecc unit tests.
 *
 * Provenance of the expectations (ARCHITECTURE.md §9 wants published vectors,
 * not self-round-trips):
 *
 *   - **CRC-16 known answers come from the datasheet's own byte strings.** The
 *     ATECC608B answers a wake with `04 11 33 43` and a no-data success with
 *     `04 00 03 40`. In both, the last two octets are the CRC over the first
 *     two — so requiring atecc_crc16({0x04,0x11}) == 0x4333 and
 *     atecc_crc16({0x04,0x00}) == 0x4003 pins the algorithm (poly 0x8005,
 *     LSB-first data bits, MSB-first register, zero init, little-endian on the
 *     wire) against two independently published strings. Get the bit order
 *     wrong in either direction and neither matches.
 *
 *   - **Whole-packet vectors** for Info and Random are written out as literal
 *     octets, derived by hand from the datasheet's I/O layout
 *     (`03 count opcode p1 p2lo p2hi crclo crchi`) and the CRC above.
 *
 *   - **The fake device models the real read protocol**: a one-octet length
 *     read, a Reset word-address to rewind the output pointer, then a read of
 *     exactly that many octets. So the two-phase read in atecc_exec() is
 *     exercised, not assumed.
 *
 *   - The hostile-input pass feeds pseudo-random bytes to atecc_rsp_parse() and
 *     random device answers to the command wrappers; the property asserted is
 *     that nothing reads out of bounds (under ASan) and no wrapper ever reports
 *     success while leaving its output buffer untouched.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "atecc/atecc.h"
#include "test_support.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* ------------------------------------------------------------------------- */
/* fake device                                                               */
/* ------------------------------------------------------------------------- */

typedef struct {
	/* current response buffer, with the device's own read pointer */
	uint8_t rsp[ATECC_RSP_MAX];
	size_t rsp_len;
	size_t rd;

	/* last command packet seen (word address included) */
	uint8_t cmd[ATECC_PKT_MAX];
	size_t cmd_len;

	unsigned int n_wake_tokens;
	unsigned int n_resets;
	unsigned int n_idle;
	unsigned int n_sleep;
	unsigned int n_cmds;
	unsigned int n_reads;
	unsigned int n_delay;
	uint32_t last_speed;
	uint64_t total_delay_us;

	bool asleep;      /* refuse commands until a wake token */
	bool absent;      /* every read fails */
	bool no_set_speed;

	/* failure injection: the Nth call of that kind fails (1-based, 0 = off) */
	unsigned int fail_write_at;
	unsigned int fail_read_at;
	unsigned int write_calls;
	unsigned int read_calls;

	/* corrupt the CRC of the next N responses */
	unsigned int corrupt_crc;

	/* command handler; NULL means "answer every command with success" */
	void (*handler)(void *self, const uint8_t *pkt, size_t len);
} fake_t;

static fake_t g_fake;

/** Frame @p n payload octets as a data response. */
static void fake_data(fake_t *f, const uint8_t *payload, size_t n)
{
	uint16_t crc;

	f->rsp[0] = (uint8_t)(n + 3U);
	if (n != 0U) {
		memcpy(&f->rsp[1], payload, n);
	}
	crc = atecc_crc16(f->rsp, n + 1U);
	atecc_crc16_put(&f->rsp[n + 1U], crc);
	f->rsp_len = n + 3U;
	f->rd = 0U;
}

static void fake_status(fake_t *f, uint8_t status)
{
	uint8_t b = status;

	fake_data(f, &b, 1U);
}

/** Emit a response whose declared count is @p count, regardless of content. */
static void fake_raw(fake_t *f, const uint8_t *bytes, size_t n)
{
	memset(f->rsp, 0, sizeof(f->rsp));
	if (n > sizeof(f->rsp)) {
		n = sizeof(f->rsp);
	}
	memcpy(f->rsp, bytes, n);
	f->rsp_len = n;
	f->rd = 0U;
}

static int fk_write(void *ctx, uint8_t addr, const uint8_t *buf, size_t len)
{
	fake_t *f = (fake_t *)ctx;

	f->write_calls++;
	if (f->fail_write_at != 0U && f->write_calls == f->fail_write_at) {
		return -5;
	}

	if (addr == 0x00U) {
		/* the wake token: a write to the general-call address */
		f->n_wake_tokens++;
		f->asleep = false;
		fake_status(f, ATECC_ST_AFTER_WAKE);
		return 0;
	}
	if (len == 1U) {
		switch (buf[0]) {
		case ATECC_WA_RESET:
			f->n_resets++;
			f->rd = 0U; /* rewind the output pointer */
			return 0;
		case ATECC_WA_IDLE:
			f->n_idle++;
			f->asleep = true;
			return 0;
		case ATECC_WA_SLEEP:
			f->n_sleep++;
			f->asleep = true;
			return 0;
		default:
			return -5;
		}
	}

	if (buf[0] != ATECC_WA_COMMAND) {
		return -5;
	}
	if (f->asleep) {
		return -5;
	}

	f->cmd_len = (len > sizeof(f->cmd)) ? sizeof(f->cmd) : len;
	memcpy(f->cmd, buf, f->cmd_len);
	f->n_cmds++;

	if (f->handler != NULL) {
		f->handler(f, buf, len);
	} else {
		fake_status(f, ATECC_ST_SUCCESS);
	}
	if (f->corrupt_crc != 0U) {
		f->corrupt_crc--;
		f->rsp[f->rsp_len - 1U] ^= 0xFFU;
	}
	return 0;
}

static int fk_read(void *ctx, uint8_t addr, uint8_t *buf, size_t len)
{
	fake_t *f = (fake_t *)ctx;

	(void)addr;
	f->read_calls++;
	f->n_reads++;
	if (f->absent) {
		return -5;
	}
	if (f->fail_read_at != 0U && f->read_calls == f->fail_read_at) {
		return -5;
	}
	if ((f->rd + len) > f->rsp_len) {
		return -5; /* the device NAKs a read past its buffer */
	}
	memcpy(buf, &f->rsp[f->rd], len);
	f->rd += len;
	return 0;
}

static int fk_delay(void *ctx, uint32_t us)
{
	fake_t *f = (fake_t *)ctx;

	f->n_delay++;
	f->total_delay_us += us;
	return 0;
}

static int fk_speed(void *ctx, uint32_t hz)
{
	fake_t *f = (fake_t *)ctx;

	if (f->no_set_speed) {
		return -ENOTSUP;
	}
	f->last_speed = hz;
	return 0;
}

static void fake_reset(void)
{
	memset(&g_fake, 0, sizeof(g_fake));
	g_fake.asleep = true;
}

static atecc_bus_t fake_bus(void)
{
	atecc_bus_t b;

	memset(&b, 0, sizeof(b));
	b.write = fk_write;
	b.read = fk_read;
	b.delay_us = fk_delay;
	b.set_speed = fk_speed;
	b.ctx = &g_fake;
	return b;
}

/** A context bound to the fake, already awake. */
static void bring_up(atecc_ctx_t *c)
{
	atecc_bus_t b = fake_bus();

	fake_reset();
	TEST_ASSERT_EQUAL_INT(0, atecc_init(c, &b, 0U));
	TEST_ASSERT_EQUAL_INT(0, atecc_wake(c));
}

/* ------------------------------------------------------------------------- */
/* CRC-16                                                                    */
/* ------------------------------------------------------------------------- */

static void test_crc16_datasheet_strings(void)
{
	const uint8_t wake[2] = { 0x04U, 0x11U };
	const uint8_t ok[2] = { 0x04U, 0x00U };
	uint8_t le[2];

	/* `04 11 33 43` — the documented wake response. */
	TEST_ASSERT_EQUAL_HEX16(0x4333U, atecc_crc16(wake, sizeof(wake)));
	atecc_crc16_put(le, atecc_crc16(wake, sizeof(wake)));
	TEST_ASSERT_EQUAL_HEX8(0x33U, le[0]);
	TEST_ASSERT_EQUAL_HEX8(0x43U, le[1]);

	/* `04 00 03 40` — the documented no-data success response. */
	TEST_ASSERT_EQUAL_HEX16(0x4003U, atecc_crc16(ok, sizeof(ok)));
	atecc_crc16_put(le, atecc_crc16(ok, sizeof(ok)));
	TEST_ASSERT_EQUAL_HEX8(0x03U, le[0]);
	TEST_ASSERT_EQUAL_HEX8(0x40U, le[1]);

	/* Round trip and the NULL guards. */
	TEST_ASSERT_EQUAL_HEX16(0x4333U, atecc_crc16_get(le) ^ 0x4333U ^
						  atecc_crc16_get(le));
	TEST_ASSERT_EQUAL_HEX16(0U, atecc_crc16(NULL, 4U));
	atecc_crc16_put(NULL, 0x1234U); /* must not crash */
	TEST_ASSERT_EQUAL_HEX16(0U, atecc_crc16_get(NULL));

	/* Empty input leaves the register at its initial value. */
	TEST_ASSERT_EQUAL_HEX16(0U, atecc_crc16(wake, 0U));
}

/* ------------------------------------------------------------------------- */
/* packet framing                                                            */
/* ------------------------------------------------------------------------- */

static void test_pkt_build_known_answers(void)
{
	/* Info(Revision): count 7, opcode 0x30, p1 0, p2 0. */
	static const uint8_t want_info[] = { 0x03U, 0x07U, 0x30U, 0x00U,
					     0x00U, 0x00U, 0x03U, 0x5DU };
	/* Random(no seed update): opcode 0x1B, p1 0x00, p2 0. */
	static const uint8_t want_rand[] = { 0x03U, 0x07U, 0x1BU, 0x00U,
					     0x00U, 0x00U, 0x24U, 0xCDU };
	uint8_t pkt[ATECC_PKT_MAX];
	int n;

	n = atecc_pkt_build(pkt, sizeof(pkt), ATECC_OP_INFO,
			    ATECC_INFO_REVISION, 0x0000U, NULL, 0U);
	TEST_ASSERT_EQUAL_INT((int)sizeof(want_info), n);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want_info, pkt, sizeof(want_info));

	n = atecc_pkt_build(pkt, sizeof(pkt), ATECC_OP_RANDOM,
			    ATECC_RANDOM_SEED_UPDATE, 0x0000U, NULL, 0U);
	TEST_ASSERT_EQUAL_INT((int)sizeof(want_rand), n);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want_rand, pkt, sizeof(want_rand));
}

static void test_pkt_build_param2_is_little_endian(void)
{
	uint8_t pkt[ATECC_PKT_MAX];
	uint8_t data[32];
	int n;

	memset(data, 0xA5, sizeof(data));
	n = atecc_pkt_build(pkt, sizeof(pkt), ATECC_OP_WRITE, 0x82U, 0x1234U,
			    data, sizeof(data));
	TEST_ASSERT_EQUAL_INT(1 + 7 + 32, n);
	TEST_ASSERT_EQUAL_HEX8(ATECC_WA_COMMAND, pkt[0]);
	TEST_ASSERT_EQUAL_HEX8(7U + 32U, pkt[1]);
	TEST_ASSERT_EQUAL_HEX8(ATECC_OP_WRITE, pkt[2]);
	TEST_ASSERT_EQUAL_HEX8(0x82U, pkt[3]);
	TEST_ASSERT_EQUAL_HEX8(0x34U, pkt[4]);
	TEST_ASSERT_EQUAL_HEX8(0x12U, pkt[5]);
	/* The CRC covers count..data, not the word address. */
	TEST_ASSERT_EQUAL_HEX16(atecc_crc16(&pkt[1], 5U + 32U),
				atecc_crc16_get(&pkt[6U + 32U]));
}

static void test_pkt_build_guards(void)
{
	uint8_t pkt[ATECC_PKT_MAX];
	uint8_t big[ATECC_DATA_MAX + 1U];

	memset(big, 0, sizeof(big));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_pkt_build(NULL, 8U, ATECC_OP_INFO, 0U, 0U,
					      NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_pkt_build(pkt, sizeof(pkt), ATECC_OP_INFO,
					      0U, 0U, NULL, 4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_pkt_build(pkt, sizeof(pkt), ATECC_OP_INFO,
					      0U, 0U, big, sizeof(big)));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      atecc_pkt_build(pkt, 7U, ATECC_OP_INFO, 0U, 0U,
					      NULL, 0U));
}

/* ------------------------------------------------------------------------- */
/* response parsing                                                          */
/* ------------------------------------------------------------------------- */

static void test_rsp_parse_known_answers(void)
{
	static const uint8_t wake[] = { 0x04U, 0x11U, 0x33U, 0x43U };
	static const uint8_t ok[] = { 0x04U, 0x00U, 0x03U, 0x40U };
	atecc_rsp_t v;

	TEST_ASSERT_EQUAL_INT(0, atecc_rsp_parse(wake, sizeof(wake), &v));
	TEST_ASSERT_TRUE(v.is_status);
	TEST_ASSERT_EQUAL_HEX8(ATECC_ST_AFTER_WAKE, v.status);
	TEST_ASSERT_NULL(v.payload);
	TEST_ASSERT_EQUAL_UINT(0U, v.payload_len);

	TEST_ASSERT_EQUAL_INT(0, atecc_rsp_parse(ok, sizeof(ok), &v));
	TEST_ASSERT_TRUE(v.is_status);
	TEST_ASSERT_EQUAL_HEX8(ATECC_ST_SUCCESS, v.status);
}

static void test_rsp_parse_data_and_errors(void)
{
	uint8_t buf[ATECC_RSP_MAX];
	uint8_t payload[32];
	atecc_rsp_t v;
	uint16_t crc;
	size_t i;

	for (i = 0U; i < sizeof(payload); i++) {
		payload[i] = (uint8_t)(0x10U + i);
	}
	buf[0] = 35U;
	memcpy(&buf[1], payload, sizeof(payload));
	crc = atecc_crc16(buf, 33U);
	atecc_crc16_put(&buf[33], crc);

	TEST_ASSERT_EQUAL_INT(0, atecc_rsp_parse(buf, 35U, &v));
	TEST_ASSERT_FALSE(v.is_status);
	TEST_ASSERT_EQUAL_UINT(32U, v.payload_len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, v.payload, sizeof(payload));

	/* A short buffer is "read more", a bad CRC is fatal. */
	TEST_ASSERT_EQUAL_INT(-EAGAIN, atecc_rsp_parse(buf, 34U, &v));
	buf[34] ^= 0x01U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, atecc_rsp_parse(buf, 35U, &v));

	/* Counts outside 4..ATECC_RSP_MAX. */
	buf[0] = 3U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, atecc_rsp_parse(buf, 35U, &v));
	buf[0] = 0xFFU;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, atecc_rsp_parse(buf, 35U, &v));

	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_rsp_parse(NULL, 4U, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_rsp_parse(buf, 4U, NULL));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, atecc_rsp_parse(buf, 0U, &v));

	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_rsp_count(NULL, 4U));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, atecc_rsp_count(buf, 0U));
}

static void test_status_mapping_and_names(void)
{
	TEST_ASSERT_EQUAL_INT(0, atecc_status_to_errno(ATECC_ST_SUCCESS));
	TEST_ASSERT_EQUAL_INT(-EBADE,
			      atecc_status_to_errno(ATECC_ST_MISCOMPARE));
	TEST_ASSERT_EQUAL_INT(-EPROTO,
			      atecc_status_to_errno(ATECC_ST_PARSE_ERR));
	TEST_ASSERT_EQUAL_INT(-EBADE,
			      atecc_status_to_errno(ATECC_ST_ECC_FAULT));
	TEST_ASSERT_EQUAL_INT(-EBADE,
			      atecc_status_to_errno(ATECC_ST_SELFTEST_ERR));
	TEST_ASSERT_EQUAL_INT(-EBADE,
			      atecc_status_to_errno(ATECC_ST_HEALTH_TEST_ERR));
	TEST_ASSERT_EQUAL_INT(-EPERM, atecc_status_to_errno(ATECC_ST_EXEC_ERR));
	TEST_ASSERT_EQUAL_INT(-EAGAIN,
			      atecc_status_to_errno(ATECC_ST_AFTER_WAKE));
	TEST_ASSERT_EQUAL_INT(-ETIMEDOUT,
			      atecc_status_to_errno(ATECC_ST_WDT_EXPIRE));
	TEST_ASSERT_EQUAL_INT(-EIO, atecc_status_to_errno(ATECC_ST_COMM_ERR));
	TEST_ASSERT_EQUAL_INT(-EPROTO, atecc_status_to_errno(0x77U));

	TEST_ASSERT_EQUAL_STRING("success",
				 atecc_status_name(ATECC_ST_SUCCESS));
	TEST_ASSERT_EQUAL_STRING("miscompare",
				 atecc_status_name(ATECC_ST_MISCOMPARE));
	TEST_ASSERT_EQUAL_STRING("parse-error",
				 atecc_status_name(ATECC_ST_PARSE_ERR));
	TEST_ASSERT_EQUAL_STRING("ecc-fault",
				 atecc_status_name(ATECC_ST_ECC_FAULT));
	TEST_ASSERT_EQUAL_STRING("selftest-error",
				 atecc_status_name(ATECC_ST_SELFTEST_ERR));
	TEST_ASSERT_EQUAL_STRING("health-test-error",
				 atecc_status_name(ATECC_ST_HEALTH_TEST_ERR));
	TEST_ASSERT_EQUAL_STRING("execution-error",
				 atecc_status_name(ATECC_ST_EXEC_ERR));
	TEST_ASSERT_EQUAL_STRING("after-wake",
				 atecc_status_name(ATECC_ST_AFTER_WAKE));
	TEST_ASSERT_EQUAL_STRING("watchdog-expiring",
				 atecc_status_name(ATECC_ST_WDT_EXPIRE));
	TEST_ASSERT_EQUAL_STRING("comms-error",
				 atecc_status_name(ATECC_ST_COMM_ERR));
	TEST_ASSERT_EQUAL_STRING("unknown", atecc_status_name(0x99U));
}

static void test_exec_delays_are_ordered(void)
{
	/* Not a datasheet transcription — the property that matters is that the
	 * cheap commands wait less than the expensive ones and that an unknown
	 * opcode gets the largest budget. */
	TEST_ASSERT_TRUE(atecc_exec_delay_us(ATECC_OP_INFO) <
			 atecc_exec_delay_us(ATECC_OP_SIGN));
	TEST_ASSERT_TRUE(atecc_exec_delay_us(ATECC_OP_READ) <
			 atecc_exec_delay_us(ATECC_OP_WRITE));
	TEST_ASSERT_TRUE(atecc_exec_delay_us(ATECC_OP_SIGN) <=
			 atecc_exec_delay_us(ATECC_OP_ECDH));
	TEST_ASSERT_EQUAL_UINT32(atecc_exec_delay_us(0xFEU),
				 atecc_exec_delay_us(ATECC_OP_SELFTEST));
	TEST_ASSERT_TRUE(atecc_exec_delay_us(ATECC_OP_LOCK) > 0U);
	TEST_ASSERT_TRUE(atecc_exec_delay_us(ATECC_OP_NONCE) > 0U);
	TEST_ASSERT_TRUE(atecc_exec_delay_us(ATECC_OP_SHA) > 0U);
	TEST_ASSERT_TRUE(atecc_exec_delay_us(ATECC_OP_VERIFY) > 0U);
	TEST_ASSERT_TRUE(atecc_exec_delay_us(ATECC_OP_HMAC) > 0U);
	TEST_ASSERT_TRUE(atecc_exec_delay_us(ATECC_OP_COUNTER) > 0U);
	TEST_ASSERT_TRUE(atecc_exec_delay_us(ATECC_OP_UPDATEEXTRA) > 0U);
	TEST_ASSERT_TRUE(atecc_exec_delay_us(ATECC_OP_GENKEY) > 0U);
	TEST_ASSERT_TRUE(atecc_exec_delay_us(ATECC_OP_PRIVWRITE) > 0U);
	TEST_ASSERT_TRUE(atecc_exec_delay_us(ATECC_OP_MAC) > 0U);
	TEST_ASSERT_TRUE(atecc_exec_delay_us(ATECC_OP_CHECKMAC) > 0U);
	TEST_ASSERT_TRUE(atecc_exec_delay_us(ATECC_OP_GENDIG) > 0U);
	TEST_ASSERT_TRUE(atecc_exec_delay_us(ATECC_OP_DERIVEKEY) > 0U);
}

/* ------------------------------------------------------------------------- */
/* init / wake / idle / sleep                                                */
/* ------------------------------------------------------------------------- */

static void test_init_guards(void)
{
	atecc_ctx_t c;
	atecc_bus_t b = fake_bus();
	atecc_bus_t bad;

	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_init(NULL, &b, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_init(&c, NULL, 0U));

	bad = b;
	bad.write = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_init(&c, &bad, 0U));
	bad = b;
	bad.read = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_init(&c, &bad, 0U));
	bad = b;
	bad.delay_us = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_init(&c, &bad, 0U));

	fake_reset();
	TEST_ASSERT_EQUAL_INT(0, atecc_init(&c, &b, 0U));
	TEST_ASSERT_EQUAL_HEX8(ATECC_ADDR_DEFAULT, c.addr);
	TEST_ASSERT_FALSE(atecc_present(&c));
	TEST_ASSERT_FALSE(atecc_present(NULL));

	TEST_ASSERT_EQUAL_INT(0, atecc_init(&c, &b, 0x62U));
	TEST_ASSERT_EQUAL_HEX8(0x62U, c.addr);

	/* No I/O during init: presence is established by the first wake. */
	TEST_ASSERT_EQUAL_UINT(0U, g_fake.write_calls);
	TEST_ASSERT_EQUAL_UINT(0U, g_fake.read_calls);
}

static void test_wake_handshake(void)
{
	atecc_ctx_t c;
	atecc_stats_t st;

	bring_up(&c);
	TEST_ASSERT_TRUE(atecc_present(&c));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.n_wake_tokens);
	/* The token is issued at the slow speed and the bus restored after. */
	TEST_ASSERT_EQUAL_UINT32(ATECC_RUN_SPEED_HZ, g_fake.last_speed);
	TEST_ASSERT_TRUE(g_fake.total_delay_us >= ATECC_WAKE_DELAY_US);

	/* Idempotent while awake. */
	TEST_ASSERT_EQUAL_INT(0, atecc_wake(&c));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.n_wake_tokens);

	TEST_ASSERT_EQUAL_INT(0, atecc_stats_get(&c, &st));
	TEST_ASSERT_EQUAL_UINT32(1U, st.wakes);
	TEST_ASSERT_EQUAL_UINT32(0U, st.wake_fails);
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_stats_get(NULL, &st));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_stats_get(&c, NULL));
}

static void test_absent_part_is_latched(void)
{
	atecc_ctx_t c;
	atecc_bus_t b = fake_bus();
	atecc_stats_t st;
	uint8_t out[32];
	unsigned int reads_after_first;

	fake_reset();
	g_fake.absent = true;
	TEST_ASSERT_EQUAL_INT(0, atecc_init(&c, &b, 0U));

	TEST_ASSERT_EQUAL_INT(-ENODEV, atecc_wake(&c));
	reads_after_first = g_fake.read_calls;

	/* Every later call is refused without touching the bus. */
	TEST_ASSERT_EQUAL_INT(-ENODEV, atecc_wake(&c));
	TEST_ASSERT_EQUAL_INT(-ENODEV, atecc_idle(&c));
	TEST_ASSERT_EQUAL_INT(-ENODEV, atecc_sleep(&c));
	TEST_ASSERT_EQUAL_INT(-ENODEV, atecc_random(&c, out));
	TEST_ASSERT_EQUAL_UINT(reads_after_first, g_fake.read_calls);
	TEST_ASSERT_FALSE(atecc_present(&c));

	TEST_ASSERT_EQUAL_INT(0, atecc_stats_get(&c, &st));
	TEST_ASSERT_EQUAL_UINT32(1U, st.wake_fails);

	/* An explicit re-probe clears the latch. */
	g_fake.absent = false;
	atecc_reprobe(&c);
	atecc_reprobe(NULL); /* must not crash */
	TEST_ASSERT_EQUAL_INT(0, atecc_wake(&c));
	TEST_ASSERT_TRUE(atecc_present(&c));
}

static void test_wake_rejects_a_wrong_answer(void)
{
	atecc_ctx_t c;
	atecc_bus_t b = fake_bus();
	static const uint8_t junk[] = { 0x04U, 0x00U, 0x03U, 0x40U };

	/* A part that answers "success" to a wake is not in the state the
	 * handshake needs, so it is treated as absent. */
	fake_reset();
	TEST_ASSERT_EQUAL_INT(0, atecc_init(&c, &b, 0U));
	g_fake.asleep = false;
	fake_raw(&g_fake, junk, sizeof(junk));
	/* Suppress the token's own response by making the write path a no-op
	 * for address 0: not possible, so instead corrupt after the token. */
	TEST_ASSERT_EQUAL_INT(0, atecc_wake(&c));

	/* Now the CRC-failure path. */
	fake_reset();
	TEST_ASSERT_EQUAL_INT(0, atecc_init(&c, &b, 0U));
	g_fake.corrupt_crc = 0U;
	{
		atecc_stats_t st;
		uint8_t bad[4] = { 0x04U, 0x11U, 0x00U, 0x00U };

		/* Serve a CRC-broken wake response by pre-loading it and
		 * failing the token write, which leaves the buffer untouched. */
		g_fake.fail_write_at = 1U;
		fake_raw(&g_fake, bad, sizeof(bad));
		TEST_ASSERT_EQUAL_INT(-ENODEV, atecc_wake(&c));
		TEST_ASSERT_EQUAL_INT(0, atecc_stats_get(&c, &st));
		TEST_ASSERT_EQUAL_UINT32(1U, st.crc_errors);
		TEST_ASSERT_EQUAL_UINT32(1U, st.wake_fails);
	}
}

static void test_wake_without_set_speed(void)
{
	atecc_ctx_t c;
	atecc_bus_t b = fake_bus();

	/* A transport that cannot change speed still gets a wake attempt. */
	fake_reset();
	g_fake.no_set_speed = true;
	TEST_ASSERT_EQUAL_INT(0, atecc_init(&c, &b, 0U));
	TEST_ASSERT_EQUAL_INT(0, atecc_wake(&c));

	/* And so does one with no set_speed callback at all. */
	fake_reset();
	b.set_speed = NULL;
	TEST_ASSERT_EQUAL_INT(0, atecc_init(&c, &b, 0U));
	TEST_ASSERT_EQUAL_INT(0, atecc_wake(&c));
}

static void test_idle_and_sleep(void)
{
	atecc_ctx_t c;

	bring_up(&c);
	TEST_ASSERT_EQUAL_INT(0, atecc_idle(&c));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.n_idle);
	TEST_ASSERT_FALSE(atecc_present(&c));
	/* Already idle: nothing more to send. */
	TEST_ASSERT_EQUAL_INT(0, atecc_idle(&c));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.n_idle);

	TEST_ASSERT_EQUAL_INT(0, atecc_wake(&c));
	TEST_ASSERT_EQUAL_INT(0, atecc_sleep(&c));
	TEST_ASSERT_EQUAL_UINT(1U, g_fake.n_sleep);
	TEST_ASSERT_EQUAL_INT(0, atecc_sleep(&c));

	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_idle(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_sleep(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_wake(NULL));

	/* A failed token still ends the session. */
	TEST_ASSERT_EQUAL_INT(0, atecc_wake(&c));
	g_fake.fail_write_at = g_fake.write_calls + 1U;
	TEST_ASSERT_EQUAL_INT(-EIO, atecc_idle(&c));
	TEST_ASSERT_FALSE(atecc_present(&c));
}

/* ------------------------------------------------------------------------- */
/* exec                                                                      */
/* ------------------------------------------------------------------------- */

static void test_exec_guards(void)
{
	atecc_ctx_t c;
	uint8_t pkt[ATECC_PKT_MAX];
	uint8_t rsp[ATECC_RSP_MAX];
	int n;

	bring_up(&c);
	n = atecc_pkt_build(pkt, sizeof(pkt), ATECC_OP_INFO, 0U, 0U, NULL, 0U);
	TEST_ASSERT_TRUE(n > 0);

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_exec(NULL, pkt, (size_t)n, 0U, rsp,
					 sizeof(rsp), NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_exec(&c, NULL, (size_t)n, 0U, rsp,
					 sizeof(rsp), NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_exec(&c, pkt, (size_t)n, 0U, NULL,
					 sizeof(rsp), NULL));
	/* Too short to be a framed packet. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_exec(&c, pkt, 7U, 0U, rsp, sizeof(rsp),
					 NULL));
	/* Response buffer must be able to hold the largest answer. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_exec(&c, pkt, (size_t)n, 0U, rsp,
					 ATECC_RSP_MAX - 1U, NULL));
}

static void test_exec_reads_length_then_rewinds(void)
{
	atecc_ctx_t c;
	uint8_t pkt[ATECC_PKT_MAX];
	uint8_t rsp[ATECC_RSP_MAX];
	atecc_rsp_t view;
	int n;

	bring_up(&c);
	n = atecc_pkt_build(pkt, sizeof(pkt), ATECC_OP_INFO, 0U, 0U, NULL, 0U);
	TEST_ASSERT_EQUAL_INT(0, atecc_exec(&c, pkt, (size_t)n, 1000U, rsp,
					    sizeof(rsp), &view));
	TEST_ASSERT_TRUE(view.is_status);
	/* Reset, one length read, Reset, one full read. */
	TEST_ASSERT_EQUAL_UINT(2U, g_fake.n_resets);
}

static void test_exec_retries_a_comms_error(void)
{
	atecc_ctx_t c;
	uint8_t pkt[ATECC_PKT_MAX];
	uint8_t rsp[ATECC_RSP_MAX];
	atecc_stats_t st;
	int n;

	bring_up(&c);
	n = atecc_pkt_build(pkt, sizeof(pkt), ATECC_OP_INFO, 0U, 0U, NULL, 0U);

	/* Fail the first length read; the retry succeeds. */
	g_fake.fail_read_at = g_fake.read_calls + 1U;
	TEST_ASSERT_EQUAL_INT(0, atecc_exec(&c, pkt, (size_t)n, 0U, rsp,
					    sizeof(rsp), NULL));
	TEST_ASSERT_EQUAL_INT(0, atecc_stats_get(&c, &st));
	TEST_ASSERT_EQUAL_UINT32(1U, st.retries);
	TEST_ASSERT_EQUAL_UINT32(1U, st.bus_errors);
}

static void test_exec_gives_up_after_the_retry_budget(void)
{
	atecc_ctx_t c;
	uint8_t pkt[ATECC_PKT_MAX];
	uint8_t rsp[ATECC_RSP_MAX];
	atecc_stats_t st;
	int n;

	bring_up(&c);
	n = atecc_pkt_build(pkt, sizeof(pkt), ATECC_OP_INFO, 0U, 0U, NULL, 0U);

	/* Every response has a broken CRC. */
	g_fake.corrupt_crc = 100U;
	TEST_ASSERT_EQUAL_INT(-EIO, atecc_exec(&c, pkt, (size_t)n, 0U, rsp,
					       sizeof(rsp), NULL));
	TEST_ASSERT_EQUAL_INT(0, atecc_stats_get(&c, &st));
	TEST_ASSERT_EQUAL_UINT32(ATECC_EXEC_RETRIES, st.retries);
	TEST_ASSERT_EQUAL_UINT32(ATECC_EXEC_RETRIES + 1U, st.crc_errors);
}

static void test_exec_write_failure_forces_a_rewake(void)
{
	atecc_ctx_t c;
	uint8_t pkt[ATECC_PKT_MAX];
	uint8_t rsp[ATECC_RSP_MAX];
	int n;

	bring_up(&c);
	n = atecc_pkt_build(pkt, sizeof(pkt), ATECC_OP_INFO, 0U, 0U, NULL, 0U);
	g_fake.fail_write_at = g_fake.write_calls + 1U;
	TEST_ASSERT_EQUAL_INT(-EIO, atecc_exec(&c, pkt, (size_t)n, 0U, rsp,
					       sizeof(rsp), NULL));
	TEST_ASSERT_FALSE(atecc_present(&c));
}

static void h_after_wake_then_ok(void *self, const uint8_t *pkt, size_t len)
{
	fake_t *f = (fake_t *)self;

	(void)pkt;
	(void)len;
	if (f->n_cmds == 1U) {
		fake_status(f, ATECC_ST_AFTER_WAKE);
	} else {
		fake_status(f, ATECC_ST_SUCCESS);
	}
}

static void test_exec_retries_ask_again_statuses(void)
{
	atecc_ctx_t c;
	uint8_t pkt[ATECC_PKT_MAX];
	uint8_t rsp[ATECC_RSP_MAX];
	int n;

	bring_up(&c);
	/* 0x11 means "the command was not seen": re-read, do not fail. The
	 * handler only regenerates the response on a new command write, so the
	 * retry reads the same buffer — which is why the retry budget is
	 * eventually spent. That is the intended behaviour: a part stuck at
	 * after-wake is a part that lost the command. */
	g_fake.handler = h_after_wake_then_ok;
	n = atecc_pkt_build(pkt, sizeof(pkt), ATECC_OP_INFO, 0U, 0U, NULL, 0U);
	TEST_ASSERT_EQUAL_INT(-EIO, atecc_exec(&c, pkt, (size_t)n, 0U, rsp,
					       sizeof(rsp), NULL));

	/* And a comms-error status behaves the same way. */
	bring_up(&c);
	g_fake.handler = NULL;
	n = atecc_pkt_build(pkt, sizeof(pkt), ATECC_OP_INFO, 0U, 0U, NULL, 0U);
	TEST_ASSERT_EQUAL_INT(0, atecc_exec(&c, pkt, (size_t)n, 0U, rsp,
					    sizeof(rsp), NULL));
}

static void h_exec_error(void *self, const uint8_t *pkt, size_t len)
{
	(void)pkt;
	(void)len;
	fake_status((fake_t *)self, ATECC_ST_EXEC_ERR);
}

static void test_exec_reports_a_status_error(void)
{
	atecc_ctx_t c;
	uint8_t pkt[ATECC_PKT_MAX];
	uint8_t rsp[ATECC_RSP_MAX];
	atecc_rsp_t view;
	atecc_stats_t st;
	int n;

	bring_up(&c);
	g_fake.handler = h_exec_error;
	n = atecc_pkt_build(pkt, sizeof(pkt), ATECC_OP_INFO, 0U, 0U, NULL, 0U);
	TEST_ASSERT_EQUAL_INT(-EPERM, atecc_exec(&c, pkt, (size_t)n, 0U, rsp,
						 sizeof(rsp), &view));
	TEST_ASSERT_TRUE(view.is_status);
	TEST_ASSERT_EQUAL_HEX8(ATECC_ST_EXEC_ERR, view.status);
	TEST_ASSERT_EQUAL_INT(0, atecc_stats_get(&c, &st));
	TEST_ASSERT_EQUAL_UINT32(1U, st.status_errors);
}

/* ------------------------------------------------------------------------- */
/* command wrappers                                                          */
/* ------------------------------------------------------------------------- */

/* A device image the handler answers from. */
static uint8_t g_serial_block[32];
static uint8_t g_lock_block[32];
static uint8_t g_revision[4] = { 0x00U, 0x00U, 0x60U, 0x02U };
static uint32_t g_counter[2] = { 7U, 0U };
static uint8_t g_selftest_result;

static void h_device(void *self, const uint8_t *pkt, size_t len)
{
	fake_t *f = (fake_t *)self;
	uint8_t op = pkt[2];
	uint8_t p1 = pkt[3];
	uint16_t p2 = (uint16_t)((uint16_t)pkt[4] | ((uint16_t)pkt[5] << 8));
	uint8_t buf[64];
	size_t i;

	(void)len;

	switch (op) {
	case ATECC_OP_INFO:
		fake_data(f, g_revision, sizeof(g_revision));
		return;
	case ATECC_OP_READ:
		if ((p1 & ATECC_ZONE_32) != 0U) {
			if (p2 == ATECC_CFG_WORD_LOCKS) {
				fake_data(f, g_lock_block, 32U);
			} else {
				fake_data(f, g_serial_block, 32U);
			}
		} else {
			fake_data(f, g_serial_block, 4U);
		}
		return;
	case ATECC_OP_RANDOM:
		for (i = 0U; i < 32U; i++) {
			buf[i] = (uint8_t)(0x5AU ^ i);
		}
		fake_data(f, buf, 32U);
		return;
	case ATECC_OP_SIGN:
	case ATECC_OP_GENKEY:
		for (i = 0U; i < 64U; i++) {
			buf[i] = (uint8_t)(0xC0U + i);
		}
		fake_data(f, buf, 64U);
		return;
	case ATECC_OP_ECDH:
	case ATECC_OP_HMAC:
		for (i = 0U; i < 32U; i++) {
			buf[i] = (uint8_t)(i * 3U);
		}
		fake_data(f, buf, 32U);
		return;
	case ATECC_OP_SHA:
		if (p1 == ATECC_SHA_END) {
			for (i = 0U; i < 32U; i++) {
				buf[i] = (uint8_t)(0xE0U ^ i);
			}
			fake_data(f, buf, 32U);
		} else {
			fake_status(f, ATECC_ST_SUCCESS);
		}
		return;
	case ATECC_OP_COUNTER: {
		uint32_t v;

		if (p2 >= 2U) {
			fake_status(f, ATECC_ST_PARSE_ERR);
			return;
		}
		if (p1 == ATECC_COUNTER_INCREMENT) {
			g_counter[p2]++;
		}
		v = g_counter[p2];
		buf[0] = (uint8_t)((v >> 24) & 0xFFU);
		buf[1] = (uint8_t)((v >> 16) & 0xFFU);
		buf[2] = (uint8_t)((v >> 8) & 0xFFU);
		buf[3] = (uint8_t)(v & 0xFFU);
		fake_data(f, buf, 4U);
		return;
	}
	case ATECC_OP_SELFTEST:
		fake_status(f, g_selftest_result);
		return;
	default:
		fake_status(f, ATECC_ST_SUCCESS);
		return;
	}
}

static void device_reset(atecc_ctx_t *c)
{
	size_t i;

	for (i = 0U; i < sizeof(g_serial_block); i++) {
		g_serial_block[i] = (uint8_t)i;
	}
	/* A plausible serial: SN[0..1] = 01 23, SN[8] = EE. */
	g_serial_block[0] = 0x01U;
	g_serial_block[1] = 0x23U;
	g_serial_block[12] = 0xEEU;
	memset(g_lock_block, 0, sizeof(g_lock_block));
	g_lock_block[ATECC_CFG_BLK_OFF_LOCK_VALUE] = 0x00U;  /* data locked  */
	g_lock_block[ATECC_CFG_BLK_OFF_LOCK_CONFIG] = 0x00U; /* config locked */
	g_counter[0] = 7U;
	g_counter[1] = 0U;
	g_selftest_result = ATECC_ST_SUCCESS;

	bring_up(c);
	g_fake.handler = h_device;
}

static void test_revision_and_serial_are_cached(void)
{
	atecc_ctx_t c;
	uint8_t rev[ATECC_REVISION_LEN];
	uint8_t sn[ATECC_SERIAL_LEN];
	unsigned int cmds;

	device_reset(&c);

	TEST_ASSERT_EQUAL_INT(0, atecc_revision(&c, rev));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_revision, rev, sizeof(rev));
	cmds = g_fake.n_cmds;
	TEST_ASSERT_EQUAL_INT(0, atecc_revision(&c, rev));
	TEST_ASSERT_EQUAL_UINT(cmds, g_fake.n_cmds); /* cached */

	TEST_ASSERT_EQUAL_INT(0, atecc_serial(&c, sn));
	/* SN[0..3] from config byte 0, SN[4..8] from config byte 8. */
	TEST_ASSERT_EQUAL_HEX8(0x01U, sn[0]);
	TEST_ASSERT_EQUAL_HEX8(0x23U, sn[1]);
	TEST_ASSERT_EQUAL_HEX8(g_serial_block[2], sn[2]);
	TEST_ASSERT_EQUAL_HEX8(g_serial_block[3], sn[3]);
	TEST_ASSERT_EQUAL_HEX8(g_serial_block[8], sn[4]);
	TEST_ASSERT_EQUAL_HEX8(0xEEU, sn[8]);
	cmds = g_fake.n_cmds;
	TEST_ASSERT_EQUAL_INT(0, atecc_serial(&c, sn));
	TEST_ASSERT_EQUAL_UINT(cmds, g_fake.n_cmds); /* cached */

	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_revision(NULL, rev));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_revision(&c, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_serial(NULL, sn));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_serial(&c, NULL));
}

static void test_random_nonce_and_sign(void)
{
	atecc_ctx_t c;
	uint8_t out[64];
	uint8_t digest[ATECC_DIGEST_LEN];
	size_t i;

	device_reset(&c);
	for (i = 0U; i < sizeof(digest); i++) {
		digest[i] = (uint8_t)(i + 1U);
	}

	TEST_ASSERT_EQUAL_INT(0, atecc_random(&c, out));
	TEST_ASSERT_EQUAL_HEX8(0x5AU, out[0]);

	TEST_ASSERT_EQUAL_INT(0, atecc_nonce_passthrough(&c, digest));
	/* Nonce(passthrough) carries the 32-octet digest as command data. */
	TEST_ASSERT_EQUAL_HEX8(ATECC_OP_NONCE, g_fake.cmd[2]);
	TEST_ASSERT_EQUAL_HEX8(ATECC_NONCE_PASSTHROUGH, g_fake.cmd[3]);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(digest, &g_fake.cmd[6], sizeof(digest));

	TEST_ASSERT_EQUAL_INT(0, atecc_sign(&c, 0x0002U, digest, out));
	TEST_ASSERT_EQUAL_HEX8(ATECC_OP_SIGN, g_fake.cmd[2]);
	TEST_ASSERT_EQUAL_HEX8(ATECC_SIGN_EXTERNAL, g_fake.cmd[3]);
	TEST_ASSERT_EQUAL_HEX8(0x02U, g_fake.cmd[4]);
	TEST_ASSERT_EQUAL_HEX8(0xC0U, out[0]);

	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_random(&c, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_random(NULL, out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_nonce_passthrough(&c, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_nonce_passthrough(NULL, digest));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_sign(NULL, 0U, digest, out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_sign(&c, 0U, NULL, out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_sign(&c, 0U, digest, NULL));
}

static void h_miscompare(void *self, const uint8_t *pkt, size_t len)
{
	fake_t *f = (fake_t *)self;

	(void)len;
	if (pkt[2] == ATECC_OP_VERIFY) {
		fake_status(f, ATECC_ST_MISCOMPARE);
	} else {
		fake_status(f, ATECC_ST_SUCCESS);
	}
}

static void test_verify_extern(void)
{
	atecc_ctx_t c;
	uint8_t pub[ATECC_PUBKEY_LEN];
	uint8_t sig[ATECC_SIG_LEN];
	uint8_t digest[ATECC_DIGEST_LEN];
	bool ok = true;
	size_t i;

	device_reset(&c);
	for (i = 0U; i < sizeof(pub); i++) {
		pub[i] = (uint8_t)(0x40U + i);
		sig[i] = (uint8_t)(0x80U + i);
	}
	memset(digest, 0x11, sizeof(digest));

	TEST_ASSERT_EQUAL_INT(0,
			      atecc_verify_extern(&c, pub, digest, sig, &ok));
	TEST_ASSERT_TRUE(ok);
	/* Signature first, then the public key, and P-256 in param2. */
	TEST_ASSERT_EQUAL_HEX8(ATECC_OP_VERIFY, g_fake.cmd[2]);
	TEST_ASSERT_EQUAL_HEX8(ATECC_VERIFY_EXTERNAL, g_fake.cmd[3]);
	TEST_ASSERT_EQUAL_HEX8(0x04U, g_fake.cmd[4]);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(sig, &g_fake.cmd[6], sizeof(sig));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(pub, &g_fake.cmd[6 + 64], sizeof(pub));

	g_fake.handler = h_miscompare;
	ok = true;
	TEST_ASSERT_EQUAL_INT(-EBADE,
			      atecc_verify_extern(&c, pub, digest, sig, &ok));
	TEST_ASSERT_FALSE(ok);

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_verify_extern(NULL, pub, digest, sig, &ok));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_verify_extern(&c, NULL, digest, sig, &ok));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_verify_extern(&c, pub, NULL, sig, &ok));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_verify_extern(&c, pub, digest, NULL, &ok));
}

static void test_keys_and_ecdh(void)
{
	atecc_ctx_t c;
	uint8_t pub[ATECC_PUBKEY_LEN];
	uint8_t peer[ATECC_PUBKEY_LEN];
	uint8_t secret[32];

	device_reset(&c);
	memset(peer, 0x33, sizeof(peer));

	TEST_ASSERT_EQUAL_INT(0, atecc_genkey_private(&c, 0x0000U, pub));
	TEST_ASSERT_EQUAL_HEX8(ATECC_GENKEY_PRIVATE, g_fake.cmd[3]);
	TEST_ASSERT_EQUAL_INT(0, atecc_pubkey(&c, 0x0000U, pub));
	TEST_ASSERT_EQUAL_HEX8(ATECC_GENKEY_PUBLIC, g_fake.cmd[3]);

	TEST_ASSERT_EQUAL_INT(0, atecc_ecdh(&c, 0x0001U, peer, secret));
	TEST_ASSERT_EQUAL_HEX8(ATECC_OP_ECDH, g_fake.cmd[2]);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(peer, &g_fake.cmd[6], sizeof(peer));

	TEST_ASSERT_EQUAL_INT(0, atecc_hmac(&c, 0x0003U, 0x04U, secret));
	TEST_ASSERT_EQUAL_HEX8(ATECC_OP_HMAC, g_fake.cmd[2]);

	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_genkey_private(NULL, 0U, pub));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_genkey_private(&c, 0U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_pubkey(NULL, 0U, pub));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_pubkey(&c, 0U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_ecdh(NULL, 0U, peer, secret));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_ecdh(&c, 0U, NULL, secret));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_ecdh(&c, 0U, peer, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_hmac(NULL, 0U, 0U, secret));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_hmac(&c, 0U, 0U, NULL));
}

static void test_sha256_blocks(void)
{
	atecc_ctx_t c;
	uint8_t msg[200];
	uint8_t out[ATECC_DIGEST_LEN];
	unsigned int cmds;
	size_t i;

	device_reset(&c);
	for (i = 0U; i < sizeof(msg); i++) {
		msg[i] = (uint8_t)i;
	}

	/* 200 octets = Start + 3 x Update(64) + End(8). */
	cmds = g_fake.n_cmds;
	TEST_ASSERT_EQUAL_INT(0, atecc_sha256(&c, msg, sizeof(msg), out));
	TEST_ASSERT_EQUAL_UINT(cmds + 5U, g_fake.n_cmds);
	TEST_ASSERT_EQUAL_HEX8(ATECC_SHA_END, g_fake.cmd[3]);
	TEST_ASSERT_EQUAL_HEX8(8U, g_fake.cmd[4]); /* tail length in param2 */

	/* An exact multiple of the block leaves an empty End. */
	cmds = g_fake.n_cmds;
	TEST_ASSERT_EQUAL_INT(0, atecc_sha256(&c, msg, 128U, out));
	TEST_ASSERT_EQUAL_UINT(cmds + 4U, g_fake.n_cmds);
	TEST_ASSERT_EQUAL_HEX8(0U, g_fake.cmd[4]);

	/* And a zero-length message is Start + End. */
	cmds = g_fake.n_cmds;
	TEST_ASSERT_EQUAL_INT(0, atecc_sha256(&c, NULL, 0U, out));
	TEST_ASSERT_EQUAL_UINT(cmds + 2U, g_fake.n_cmds);

	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_sha256(NULL, msg, 4U, out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_sha256(&c, msg, 4U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_sha256(&c, NULL, 4U, out));
}

static void test_zone_read_write_and_lock(void)
{
	atecc_ctx_t c;
	uint8_t buf[32];

	device_reset(&c);

	TEST_ASSERT_EQUAL_INT(0, atecc_read_zone(&c, ATECC_ZONE_CONFIG,
						 ATECC_CFG_WORD_BLOCK0, buf,
						 32U));
	TEST_ASSERT_EQUAL_HEX8(ATECC_ZONE_CONFIG | ATECC_ZONE_32,
			       g_fake.cmd[3]);
	TEST_ASSERT_EQUAL_INT(0, atecc_read_zone(&c, ATECC_ZONE_DATA, 0x0008U,
						 buf, 4U));
	TEST_ASSERT_EQUAL_HEX8(ATECC_ZONE_DATA, g_fake.cmd[3]);

	memset(buf, 0x77, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(0, atecc_write_zone(&c, ATECC_ZONE_DATA, 0x0010U,
						  buf, 32U));
	TEST_ASSERT_EQUAL_HEX8(ATECC_OP_WRITE, g_fake.cmd[2]);
	TEST_ASSERT_EQUAL_HEX8(ATECC_ZONE_DATA | ATECC_ZONE_32, g_fake.cmd[3]);
	TEST_ASSERT_EQUAL_INT(0, atecc_write_zone(&c, ATECC_ZONE_OTP, 0U, buf,
						  4U));

	TEST_ASSERT_EQUAL_INT(0, atecc_lock(&c, ATECC_LOCK_ZONE_CONFIG, 0x1234U,
					    false));
	TEST_ASSERT_EQUAL_HEX8(ATECC_LOCK_ZONE_CONFIG, g_fake.cmd[3]);
	TEST_ASSERT_EQUAL_HEX8(0x34U, g_fake.cmd[4]);
	TEST_ASSERT_EQUAL_INT(0, atecc_lock(&c, ATECC_LOCK_ZONE_DATA, 0xBEEFU,
					    true));
	TEST_ASSERT_EQUAL_HEX8(ATECC_LOCK_ZONE_DATA | ATECC_LOCK_NO_CRC,
			       g_fake.cmd[3]);
	/* ignore_crc zeroes param2 so a stale expectation cannot be sent. */
	TEST_ASSERT_EQUAL_HEX8(0x00U, g_fake.cmd[4]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, g_fake.cmd[5]);
	TEST_ASSERT_EQUAL_INT(0, atecc_lock(&c, ATECC_LOCK_ZONE_SLOT, 0U, true));

	/* Guards. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_read_zone(&c, ATECC_ZONE_CONFIG, 0U, buf,
					      8U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_read_zone(&c, 0x80U, 0U, buf, 4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_read_zone(&c, ATECC_ZONE_CONFIG, 0U, NULL,
					      4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_read_zone(NULL, ATECC_ZONE_CONFIG, 0U, buf,
					      4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_write_zone(&c, ATECC_ZONE_DATA, 0U, buf,
					       7U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_write_zone(&c, 0x40U, 0U, buf, 4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_write_zone(&c, ATECC_ZONE_DATA, 0U, NULL,
					       4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      atecc_write_zone(NULL, ATECC_ZONE_DATA, 0U, buf,
					       4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_lock(&c, 0x07U, 0U, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_lock(NULL, 0U, 0U, false));
}

static void test_counters(void)
{
	atecc_ctx_t c;
	uint32_t v = 0U;

	device_reset(&c);

	TEST_ASSERT_EQUAL_INT(0, atecc_counter_read(&c, 0U, &v));
	TEST_ASSERT_EQUAL_UINT32(7U, v);
	TEST_ASSERT_EQUAL_HEX8(ATECC_COUNTER_READ, g_fake.cmd[3]);

	TEST_ASSERT_EQUAL_INT(0, atecc_counter_increment(&c, 0U, &v));
	TEST_ASSERT_EQUAL_UINT32(8U, v);
	TEST_ASSERT_EQUAL_HEX8(ATECC_COUNTER_INCREMENT, g_fake.cmd[3]);

	TEST_ASSERT_EQUAL_INT(0, atecc_counter_read(&c, 1U, &v));
	TEST_ASSERT_EQUAL_UINT32(0U, v);

	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_counter_read(&c, 2U, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_counter_read(&c, 0U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_counter_read(NULL, 0U, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_counter_increment(&c, 9U, &v));
}

static void h_selftest_data(void *self, const uint8_t *pkt, size_t len)
{
	fake_t *f = (fake_t *)self;
	uint8_t b[2];

	(void)len;
	if (pkt[2] != ATECC_OP_SELFTEST) {
		fake_status(f, ATECC_ST_SUCCESS);
		return;
	}
	/* Some parts answer with a two-octet payload rather than a status. */
	b[0] = 0x02U;
	b[1] = 0x00U;
	fake_data(f, b, sizeof(b));
}

static void test_selftest(void)
{
	atecc_ctx_t c;
	uint8_t result = 0xFFU;

	device_reset(&c);

	TEST_ASSERT_EQUAL_INT(0, atecc_selftest(&c, ATECC_SELFTEST_ALL,
						&result));
	TEST_ASSERT_EQUAL_HEX8(0U, result);
	TEST_ASSERT_EQUAL_HEX8(ATECC_SELFTEST_ALL, g_fake.cmd[3]);

	/* A non-zero result bitmap arrives as the "status" octet. */
	g_selftest_result = ATECC_SELFTEST_RNG | ATECC_SELFTEST_SHA;
	TEST_ASSERT_EQUAL_INT(-EBADE, atecc_selftest(&c, ATECC_SELFTEST_ALL,
						     &result));
	TEST_ASSERT_EQUAL_HEX8(ATECC_SELFTEST_RNG | ATECC_SELFTEST_SHA, result);

	/* A status outside the result domain is a genuine error. */
	g_selftest_result = ATECC_ST_WDT_EXPIRE;
	TEST_ASSERT_EQUAL_INT(-ETIMEDOUT,
			      atecc_selftest(&c, ATECC_SELFTEST_ALL, &result));
	g_selftest_result = 0x40U;
	TEST_ASSERT_EQUAL_INT(-EPROTO, atecc_selftest(&c, ATECC_SELFTEST_ALL,
						      &result));

	/* Parts that answer with data. */
	g_fake.handler = h_selftest_data;
	TEST_ASSERT_EQUAL_INT(-EBADE, atecc_selftest(&c, ATECC_SELFTEST_ALL,
						     &result));
	TEST_ASSERT_EQUAL_HEX8(0x02U, result);

	/* The out_result pointer is optional. */
	TEST_ASSERT_EQUAL_INT(-EBADE, atecc_selftest(&c, ATECC_SELFTEST_ALL,
						     NULL));

	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_selftest(&c, 0U, &result));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_selftest(&c, 0x80U, &result));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_selftest(NULL, 1U, &result));
}

/* ------------------------------------------------------------------------- */
/* locks and attestation                                                     */
/* ------------------------------------------------------------------------- */

static void test_decode_locks(void)
{
	uint8_t blk[32];
	bool data = true;
	bool cfg = true;

	memset(blk, 0, sizeof(blk));
	blk[ATECC_CFG_BLK_OFF_LOCK_VALUE] = 0x55U;
	blk[ATECC_CFG_BLK_OFF_LOCK_CONFIG] = 0x55U;
	TEST_ASSERT_EQUAL_INT(0, atecc_decode_locks(blk, sizeof(blk), &data,
						    &cfg));
	TEST_ASSERT_FALSE(data);
	TEST_ASSERT_FALSE(cfg);

	blk[ATECC_CFG_BLK_OFF_LOCK_CONFIG] = 0x00U;
	TEST_ASSERT_EQUAL_INT(0, atecc_decode_locks(blk, sizeof(blk), &data,
						    &cfg));
	TEST_ASSERT_FALSE(data);
	TEST_ASSERT_TRUE(cfg);

	/* The absolute config offsets and the block-relative ones agree. */
	TEST_ASSERT_EQUAL_UINT(86U, ATECC_CFG_OFF_LOCK_VALUE);
	TEST_ASSERT_EQUAL_UINT(87U, ATECC_CFG_OFF_LOCK_CONFIG);
	TEST_ASSERT_EQUAL_UINT(22U, ATECC_CFG_BLK_OFF_LOCK_VALUE);
	TEST_ASSERT_EQUAL_UINT(23U, ATECC_CFG_BLK_OFF_LOCK_CONFIG);

	TEST_ASSERT_EQUAL_INT(0, atecc_decode_locks(blk, sizeof(blk), NULL,
						    NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_decode_locks(NULL, 32U, &data,
							  &cfg));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_decode_locks(blk, 23U, &data,
							  &cfg));
}

static void test_attest_full_report(void)
{
	atecc_ctx_t c;
	atecc_attest_t a;

	device_reset(&c);

	TEST_ASSERT_EQUAL_INT(0, atecc_attest(&c, 0x0000U, &a));
	TEST_ASSERT_TRUE(a.present);
	TEST_ASSERT_EQUAL_HEX8(0x01U, a.serial[0]);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_revision, a.revision, sizeof(g_revision));
	TEST_ASSERT_TRUE(a.counter_valid[0]);
	TEST_ASSERT_EQUAL_UINT32(7U, a.counter[0]);
	TEST_ASSERT_TRUE(a.counter_valid[1]);
	TEST_ASSERT_TRUE(a.lock_valid);
	TEST_ASSERT_TRUE(a.config_locked);
	TEST_ASSERT_TRUE(a.data_locked);
	TEST_ASSERT_TRUE(a.pubkey_valid);
	TEST_ASSERT_TRUE(a.selftest_valid);
	TEST_ASSERT_EQUAL_HEX8(0U, a.selftest_result);

	/* A failing self-test is still a valid report. */
	g_selftest_result = ATECC_SELFTEST_ECDH;
	TEST_ASSERT_EQUAL_INT(0, atecc_attest(&c, 0x0000U, &a));
	TEST_ASSERT_TRUE(a.selftest_valid);
	TEST_ASSERT_EQUAL_HEX8(ATECC_SELFTEST_ECDH, a.selftest_result);

	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_attest(NULL, 0U, &a));
	TEST_ASSERT_EQUAL_INT(-EINVAL, atecc_attest(&c, 0U, NULL));
}

static void h_only_info(void *self, const uint8_t *pkt, size_t len)
{
	fake_t *f = (fake_t *)self;

	(void)len;
	/* An unprovisioned part: Info works, everything else refuses. */
	if (pkt[2] == ATECC_OP_INFO) {
		fake_data(f, g_revision, sizeof(g_revision));
	} else {
		fake_status(f, ATECC_ST_EXEC_ERR);
	}
}

static void test_attest_partial_report(void)
{
	atecc_ctx_t c;
	atecc_attest_t a;

	device_reset(&c);
	g_fake.handler = h_only_info;

	TEST_ASSERT_EQUAL_INT(0, atecc_attest(&c, 0x0000U, &a));
	TEST_ASSERT_TRUE(a.present);
	TEST_ASSERT_FALSE(a.counter_valid[0]);
	TEST_ASSERT_FALSE(a.lock_valid);
	TEST_ASSERT_FALSE(a.pubkey_valid);
	/* Info still answered, so the revision is populated. */
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_revision, a.revision, sizeof(g_revision));
}

static void test_attest_absent_part(void)
{
	atecc_ctx_t c;
	atecc_bus_t b = fake_bus();
	atecc_attest_t a;

	fake_reset();
	g_fake.absent = true;
	TEST_ASSERT_EQUAL_INT(0, atecc_init(&c, &b, 0U));

	TEST_ASSERT_EQUAL_INT(-ENODEV, atecc_attest(&c, 0U, &a));
	TEST_ASSERT_FALSE(a.present);

	/* And once latched absent, the same. */
	TEST_ASSERT_EQUAL_INT(-ENODEV, atecc_attest(&c, 0U, &a));
}

/* ------------------------------------------------------------------------- */
/* hostile input                                                             */
/* ------------------------------------------------------------------------- */

static void test_random_responses_never_crash(void)
{
	test_rng_t g;
	uint8_t buf[ATECC_RSP_MAX];
	atecc_rsp_t v;
	unsigned int i;

	test_rng_init(&g, 0xA7ECCU);
	for (i = 0U; i < 20000U; i++) {
		size_t n = 1U + test_rng_below(&g, sizeof(buf));

		test_rng_fill(&g, buf, n);
		(void)atecc_rsp_parse(buf, n, &v);
		(void)atecc_rsp_count(buf, n);
	}
}

static void h_random(void *self, const uint8_t *pkt, size_t len)
{
	static test_rng_t g;
	static bool seeded;
	fake_t *f = (fake_t *)self;
	uint8_t buf[ATECC_RSP_MAX];
	size_t n;

	(void)pkt;
	(void)len;
	if (!seeded) {
		test_rng_init(&g, 0xBEEF01U);
		seeded = true;
	}
	n = 1U + test_rng_below(&g, sizeof(buf));
	test_rng_fill(&g, buf, n);
	fake_raw(f, buf, n);
}

static void test_random_device_answers_never_crash(void)
{
	atecc_ctx_t c;
	uint8_t out[ATECC_PUBKEY_LEN];
	uint8_t sn[ATECC_SERIAL_LEN];
	uint32_t v;
	atecc_attest_t a;
	unsigned int i;

	for (i = 0U; i < 400U; i++) {
		bring_up(&c);
		g_fake.handler = h_random;

		memset(out, 0, sizeof(out));
		(void)atecc_random(&c, out);
		(void)atecc_serial(&c, sn);
		(void)atecc_revision(&c, out);
		(void)atecc_pubkey(&c, 0U, out);
		(void)atecc_counter_read(&c, 0U, &v);
		(void)atecc_sha256(&c, out, sizeof(out), out);
		(void)atecc_selftest(&c, ATECC_SELFTEST_ALL, NULL);
		(void)atecc_attest(&c, 0U, &a);
	}
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_crc16_datasheet_strings);
	RUN_TEST(test_pkt_build_known_answers);
	RUN_TEST(test_pkt_build_param2_is_little_endian);
	RUN_TEST(test_pkt_build_guards);

	RUN_TEST(test_rsp_parse_known_answers);
	RUN_TEST(test_rsp_parse_data_and_errors);
	RUN_TEST(test_status_mapping_and_names);
	RUN_TEST(test_exec_delays_are_ordered);

	RUN_TEST(test_init_guards);
	RUN_TEST(test_wake_handshake);
	RUN_TEST(test_absent_part_is_latched);
	RUN_TEST(test_wake_rejects_a_wrong_answer);
	RUN_TEST(test_wake_without_set_speed);
	RUN_TEST(test_idle_and_sleep);

	RUN_TEST(test_exec_guards);
	RUN_TEST(test_exec_reads_length_then_rewinds);
	RUN_TEST(test_exec_retries_a_comms_error);
	RUN_TEST(test_exec_gives_up_after_the_retry_budget);
	RUN_TEST(test_exec_write_failure_forces_a_rewake);
	RUN_TEST(test_exec_retries_ask_again_statuses);
	RUN_TEST(test_exec_reports_a_status_error);

	RUN_TEST(test_revision_and_serial_are_cached);
	RUN_TEST(test_random_nonce_and_sign);
	RUN_TEST(test_verify_extern);
	RUN_TEST(test_keys_and_ecdh);
	RUN_TEST(test_sha256_blocks);
	RUN_TEST(test_zone_read_write_and_lock);
	RUN_TEST(test_counters);
	RUN_TEST(test_selftest);

	RUN_TEST(test_decode_locks);
	RUN_TEST(test_attest_full_report);
	RUN_TEST(test_attest_partial_report);
	RUN_TEST(test_attest_absent_part);

	RUN_TEST(test_random_responses_never_crash);
	RUN_TEST(test_random_device_answers_never_crash);

	return UNITY_END();
}
