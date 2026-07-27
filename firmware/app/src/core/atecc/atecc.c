/*
 * STS1000 "Meridian" — core/atecc: ATECC608B command protocol.
 *
 * See atecc.h for the wire format, the transport contract and the
 * absence-tolerance rule.
 *
 * Two design points worth stating once:
 *
 * 1. **Every wire buffer is a local array.** The largest packet is
 *    Verify(External) at 136 octets and the largest response is 72, so the
 *    deepest stack frame here is under 256 octets. That keeps the module
 *    allocation-free (ARCHITECTURE.md §4) without a caller-supplied scratch
 *    struct, and it means two contexts can never share a buffer.
 *
 * 2. **A malformed response is retried, not failed.** The part answers 0xFF or
 *    NAKs while it is still executing, and the datasheet's maximum execution
 *    times are worst-case over temperature and supply. Rather than trust the
 *    delay table, atecc_exec() polls: delay, read, and on a transport error or
 *    a bad CRC wait a further slice and read again, up to ATECC_EXEC_RETRIES.
 *    Only then is it -EIO.
 */

#include "atecc/atecc.h"

#include <errno.h>
#include <string.h>

/* ========================================================================= */
/* CRC-16                                                                    */
/* ========================================================================= */

uint16_t atecc_crc16(const uint8_t *data, size_t len)
{
	uint16_t reg = 0U;
	size_t i;

	if (data == NULL) {
		return 0U;
	}

	for (i = 0U; i < len; i++) {
		unsigned int shift;

		/* Data bits enter least-significant-first while the register
		 * shifts most-significant-first. That mixed order is what makes
		 * this CRC unlike the named CRC-16 variants; do not "simplify"
		 * it into a table lookup without re-deriving the table. */
		for (shift = 0x01U; shift < 0x100U; shift <<= 1) {
			uint16_t data_bit = ((data[i] & (uint8_t)shift) != 0U)
						    ? 1U
						    : 0U;
			uint16_t crc_bit = (uint16_t)((reg >> 15) & 1U);

			reg = (uint16_t)(reg << 1);
			if (data_bit != crc_bit) {
				reg ^= 0x8005U;
			}
		}
	}
	return reg;
}

void atecc_crc16_put(uint8_t *out, uint16_t crc)
{
	if (out == NULL) {
		return;
	}
	out[0] = (uint8_t)(crc & 0xFFU);
	out[1] = (uint8_t)((crc >> 8) & 0xFFU);
}

uint16_t atecc_crc16_get(const uint8_t *in)
{
	if (in == NULL) {
		return 0U;
	}
	return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

/* ========================================================================= */
/* packet build / response parse                                             */
/* ========================================================================= */

int atecc_pkt_build(uint8_t *out, size_t cap, uint8_t opcode, uint8_t p1,
		    uint16_t p2, const uint8_t *data, size_t data_len)
{
	size_t count;
	size_t total;
	uint16_t crc;

	if (out == NULL) {
		return -EINVAL;
	}
	if (data == NULL && data_len != 0U) {
		return -EINVAL;
	}
	if (data_len > ATECC_DATA_MAX) {
		return -EINVAL;
	}

	/* count covers itself, the 5 header octets and the 2 CRC octets. */
	count = 7U + data_len;
	total = 1U + count;
	if (cap < total) {
		return -ENOSPC;
	}

	out[0] = ATECC_WA_COMMAND;
	out[1] = (uint8_t)count;
	out[2] = opcode;
	out[3] = p1;
	out[4] = (uint8_t)(p2 & 0xFFU);
	out[5] = (uint8_t)((p2 >> 8) & 0xFFU);
	if (data_len != 0U) {
		memcpy(&out[6], data, data_len);
	}
	crc = atecc_crc16(&out[1], count - 2U);
	atecc_crc16_put(&out[1U + count - 2U], crc);
	return (int)total;
}

int atecc_rsp_count(const uint8_t *buf, size_t len)
{
	uint8_t count;

	if (buf == NULL) {
		return -EINVAL;
	}
	if (len == 0U) {
		return -EAGAIN;
	}
	count = buf[0];
	if (count < 4U || (size_t)count > ATECC_RSP_MAX) {
		return -EBADMSG;
	}
	return (int)count;
}

int atecc_rsp_parse(const uint8_t *buf, size_t len, atecc_rsp_t *out)
{
	int count;
	uint16_t want;
	uint16_t got;

	if (buf == NULL || out == NULL) {
		return -EINVAL;
	}
	count = atecc_rsp_count(buf, len);
	if (count < 0) {
		return count;
	}
	if (len < (size_t)count) {
		return -EAGAIN;
	}

	want = atecc_crc16(buf, (size_t)count - 2U);
	got = atecc_crc16_get(&buf[(size_t)count - 2U]);
	if (want != got) {
		return -EBADMSG;
	}

	memset(out, 0, sizeof(*out));
	if (count == 4) {
		out->is_status = true;
		out->status = buf[1];
	} else {
		out->payload = &buf[1];
		out->payload_len = (size_t)count - 3U;
	}
	return 0;
}

int atecc_status_to_errno(uint8_t status)
{
	switch (status) {
	case ATECC_ST_SUCCESS:
		return 0;
	case ATECC_ST_MISCOMPARE:
		return -EBADE;
	case ATECC_ST_PARSE_ERR:
		return -EPROTO;
	case ATECC_ST_ECC_FAULT:
	case ATECC_ST_SELFTEST_ERR:
	case ATECC_ST_HEALTH_TEST_ERR:
		return -EBADE;
	case ATECC_ST_EXEC_ERR:
		return -EPERM;
	case ATECC_ST_AFTER_WAKE:
		/* The part is awake but has been handed no command: from the
		 * point of view of a caller that just sent one, that is a lost
		 * command, not a result. */
		return -EAGAIN;
	case ATECC_ST_WDT_EXPIRE:
		return -ETIMEDOUT;
	case ATECC_ST_COMM_ERR:
		return -EIO;
	default:
		return -EPROTO;
	}
}

const char *atecc_status_name(uint8_t status)
{
	switch (status) {
	case ATECC_ST_SUCCESS:
		return "success";
	case ATECC_ST_MISCOMPARE:
		return "miscompare";
	case ATECC_ST_PARSE_ERR:
		return "parse-error";
	case ATECC_ST_ECC_FAULT:
		return "ecc-fault";
	case ATECC_ST_SELFTEST_ERR:
		return "selftest-error";
	case ATECC_ST_HEALTH_TEST_ERR:
		return "health-test-error";
	case ATECC_ST_EXEC_ERR:
		return "execution-error";
	case ATECC_ST_AFTER_WAKE:
		return "after-wake";
	case ATECC_ST_WDT_EXPIRE:
		return "watchdog-expiring";
	case ATECC_ST_COMM_ERR:
		return "comms-error";
	default:
		return "unknown";
	}
}

uint32_t atecc_exec_delay_us(uint8_t opcode)
{
	/* Ceilings above the datasheet maximum execution times. atecc_exec()
	 * polls after the delay, so being generous costs latency and being
	 * short costs a retry — neither costs correctness. */
	switch (opcode) {
	case ATECC_OP_INFO:
	case ATECC_OP_READ:
		return 5000U;
	case ATECC_OP_LOCK:
	case ATECC_OP_UPDATEEXTRA:
		return 20000U;
	case ATECC_OP_RANDOM:
	case ATECC_OP_HMAC:
	case ATECC_OP_MAC:
	case ATECC_OP_CHECKMAC:
	case ATECC_OP_GENDIG:
		return 30000U;
	case ATECC_OP_NONCE:
	case ATECC_OP_COUNTER:
	case ATECC_OP_DERIVEKEY:
		return 35000U;
	case ATECC_OP_SHA:
		return 45000U;
	case ATECC_OP_WRITE:
	case ATECC_OP_PRIVWRITE:
		return 55000U;
	case ATECC_OP_SIGN:
		return 100000U;
	case ATECC_OP_GENKEY:
	case ATECC_OP_VERIFY:
		return 130000U;
	case ATECC_OP_ECDH:
		return 180000U;
	case ATECC_OP_SELFTEST:
		return 700000U;
	default:
		return 700000U;
	}
}

/* ========================================================================= */
/* context / transport                                                       */
/* ========================================================================= */

int atecc_init(atecc_ctx_t *c, const atecc_bus_t *bus, uint8_t addr)
{
	if (c == NULL || bus == NULL) {
		return -EINVAL;
	}
	if (bus->write == NULL || bus->read == NULL || bus->delay_us == NULL) {
		return -EINVAL;
	}

	memset(c, 0, sizeof(*c));
	c->bus = *bus;
	c->addr = (addr != 0U) ? addr : (uint8_t)ATECC_ADDR_DEFAULT;
	c->ready = true;
	return 0;
}

bool atecc_present(const atecc_ctx_t *c)
{
	return (c != NULL) && c->ready && c->awake && !c->absent;
}

void atecc_reprobe(atecc_ctx_t *c)
{
	if (c == NULL) {
		return;
	}
	c->absent = false;
	c->awake = false;
}

int atecc_stats_get(const atecc_ctx_t *c, atecc_stats_t *out)
{
	if (c == NULL || out == NULL) {
		return -EINVAL;
	}
	*out = c->stats;
	return 0;
}

static void set_speed(atecc_ctx_t *c, uint32_t hz)
{
	if (c->bus.set_speed != NULL) {
		(void)c->bus.set_speed(c->bus.ctx, hz);
	}
}

static void bus_delay(atecc_ctx_t *c, uint32_t us)
{
	(void)c->bus.delay_us(c->bus.ctx, us);
}

/** Send a bare word-address token (Reset / Sleep / Idle). */
static int token(atecc_ctx_t *c, uint8_t wa)
{
	uint8_t b = wa;
	int rc;

	rc = c->bus.write(c->bus.ctx, c->addr, &b, 1U);
	if (rc != 0) {
		c->stats.bus_errors++;
		return -EIO;
	}
	return 0;
}

int atecc_wake(atecc_ctx_t *c)
{
	uint8_t rsp[4];
	atecc_rsp_t view;
	uint8_t zero = 0x00U;
	int rc;

	if (c == NULL || !c->ready) {
		return -EINVAL;
	}
	if (c->absent) {
		return -ENODEV;
	}
	if (c->awake) {
		return 0;
	}

	/* The wake token is a write of 0x00 to address 0x00 at a bus speed slow
	 * enough that the transfer holds SDA low for the datasheet's minimum
	 * wake-low time. At 100 kHz one byte plus addressing is ~180 µs of
	 * mostly-low line, comfortably over the 60 µs floor; at the board's
	 * running 400 kHz it is not. The NAK the general-call address returns
	 * is expected and ignored — the electrical event is the point. */
	set_speed(c, ATECC_WAKE_SPEED_HZ);
	(void)c->bus.write(c->bus.ctx, 0x00U, &zero, 1U);
	bus_delay(c, ATECC_WAKE_LOW_US);
	set_speed(c, ATECC_RUN_SPEED_HZ);
	bus_delay(c, ATECC_WAKE_DELAY_US);

	rc = c->bus.read(c->bus.ctx, c->addr, rsp, sizeof(rsp));
	if (rc != 0) {
		c->stats.bus_errors++;
		c->stats.wake_fails++;
		c->absent = true;
		return -ENODEV;
	}

	rc = atecc_rsp_parse(rsp, sizeof(rsp), &view);
	if (rc != 0 || !view.is_status || view.status != ATECC_ST_AFTER_WAKE) {
		if (rc == -EBADMSG) {
			c->stats.crc_errors++;
		}
		c->stats.wake_fails++;
		c->absent = true;
		return -ENODEV;
	}

	c->awake = true;
	c->stats.wakes++;
	return 0;
}

int atecc_idle(atecc_ctx_t *c)
{
	int rc;

	if (c == NULL || !c->ready) {
		return -EINVAL;
	}
	if (c->absent) {
		return -ENODEV;
	}
	if (!c->awake) {
		return 0;
	}
	rc = token(c, ATECC_WA_IDLE);
	/* Idle and Sleep both end the session as far as this module is
	 * concerned; a failed token still leaves the part in an unknown state,
	 * so re-waking before the next command is the safe assumption. */
	c->awake = false;
	return rc;
}

int atecc_sleep(atecc_ctx_t *c)
{
	int rc;

	if (c == NULL || !c->ready) {
		return -EINVAL;
	}
	if (c->absent) {
		return -ENODEV;
	}
	if (!c->awake) {
		return 0;
	}
	rc = token(c, ATECC_WA_SLEEP);
	c->awake = false;
	return rc;
}

/* ========================================================================= */
/* execution                                                                 */
/* ========================================================================= */

/**
 * Read one response, length-first.
 *
 * The part's output buffer has a read pointer that a read advances and only the
 * Reset word address rewinds. So: read the single count octet, rewind, then
 * read exactly that many. The alternative — one over-long read padded with
 * 0xFF — asks the transport to tolerate an early NAK, which a Zephyr I²C
 * controller reports as a failed transfer rather than as padding.
 *
 * @retval >0        Octets placed in @p rsp (its declared count).
 * @retval -EIO      A transport call failed.
 * @retval -EBADMSG  The count octet was not a plausible length (0xFF while
 *                   busy, or a truncated buffer).
 */
static int read_rsp(atecc_ctx_t *c, uint8_t *rsp)
{
	int cnt;

	if (c->bus.read(c->bus.ctx, c->addr, rsp, 1U) != 0) {
		c->stats.bus_errors++;
		return -EIO;
	}
	cnt = atecc_rsp_count(rsp, 1U);
	if (cnt < 0) {
		return cnt;
	}
	if (token(c, ATECC_WA_RESET) != 0) {
		return -EIO;
	}
	if (c->bus.read(c->bus.ctx, c->addr, rsp, (size_t)cnt) != 0) {
		c->stats.bus_errors++;
		return -EIO;
	}
	return cnt;
}

int atecc_exec(atecc_ctx_t *c, const uint8_t *pkt, size_t pkt_len,
	       uint32_t exec_delay_us, uint8_t *rsp, size_t rsp_cap,
	       atecc_rsp_t *out_rsp)
{
	atecc_rsp_t view;
	unsigned int attempt;
	uint32_t slice;
	int rc;

	if (c == NULL || !c->ready || pkt == NULL || rsp == NULL) {
		return -EINVAL;
	}
	/* A framed packet is at least the word address plus the seven-octet
	 * minimum command; anything shorter cannot have come from
	 * atecc_pkt_build(). */
	if (pkt_len < 8U || rsp_cap < ATECC_RSP_MAX) {
		return -EINVAL;
	}
	if (c->absent) {
		return -ENODEV;
	}

	rc = atecc_wake(c);
	if (rc != 0) {
		return rc;
	}

	if (exec_delay_us == 0U) {
		exec_delay_us = atecc_exec_delay_us(pkt[2]);
	}

	if (c->bus.write(c->bus.ctx, c->addr, pkt, pkt_len) != 0) {
		c->stats.bus_errors++;
		/* The part may or may not have latched the command; force a
		 * re-wake so the next caller starts from a known state. */
		c->awake = false;
		return -EIO;
	}
	c->stats.commands++;

	/* Poll for the response. The first wait is the full budget; each retry
	 * adds an eighth of it, so a part that is merely slow is tolerated
	 * without stretching the common case. */
	slice = exec_delay_us / 8U;
	if (slice == 0U) {
		slice = 1000U;
	}
	bus_delay(c, exec_delay_us);

	for (attempt = 0U; attempt <= ATECC_EXEC_RETRIES; attempt++) {
		int got;

		if (attempt != 0U) {
			c->stats.retries++;
			bus_delay(c, slice);
		}

		memset(rsp, 0, ATECC_RSP_MAX);
		got = read_rsp(c, rsp);
		if (got < 0) {
			continue;
		}

		rc = atecc_rsp_parse(rsp, (size_t)got, &view);
		if (rc == -EBADMSG) {
			c->stats.crc_errors++;
			continue;
		}
		if (rc != 0) {
			continue;
		}

		if (out_rsp != NULL) {
			*out_rsp = view;
		}
		if (!view.is_status) {
			return 0;
		}
		if (view.status == ATECC_ST_SUCCESS) {
			return 0;
		}
		c->stats.status_errors++;
		if (view.status == ATECC_ST_AFTER_WAKE ||
		    view.status == ATECC_ST_COMM_ERR) {
			/* Both mean "ask again": the command was not seen, or
			 * the part could not frame its answer. */
			continue;
		}
		return atecc_status_to_errno(view.status);
	}

	return -EIO;
}

/**
 * Build and run one command, requiring a data response of exactly @p want.
 *
 * Folds the four lines every wrapper would otherwise repeat, and turns "the
 * part answered success with no data where 64 octets were expected" into
 * -EPROTO rather than a silently short copy.
 */
static int cmd_data(atecc_ctx_t *c, uint8_t opcode, uint8_t p1, uint16_t p2,
		    const uint8_t *data, size_t data_len, uint8_t *out,
		    size_t want)
{
	uint8_t pkt[ATECC_PKT_MAX];
	uint8_t rsp[ATECC_RSP_MAX];
	atecc_rsp_t view;
	int n;
	int rc;

	n = atecc_pkt_build(pkt, sizeof(pkt), opcode, p1, p2, data, data_len);
	if (n < 0) {
		return n;
	}
	rc = atecc_exec(c, pkt, (size_t)n, 0U, rsp, sizeof(rsp), &view);
	if (rc != 0) {
		return rc;
	}
	if (want == 0U) {
		return 0;
	}
	if (view.is_status || view.payload_len != want) {
		return -EPROTO;
	}
	if (out != NULL) {
		memcpy(out, view.payload, want);
	}
	return 0;
}

/** Build and run one command that is expected to answer with a bare status. */
static int cmd_status(atecc_ctx_t *c, uint8_t opcode, uint8_t p1, uint16_t p2,
		      const uint8_t *data, size_t data_len)
{
	return cmd_data(c, opcode, p1, p2, data, data_len, NULL, 0U);
}

/* ========================================================================= */
/* command wrappers                                                          */
/* ========================================================================= */

int atecc_revision(atecc_ctx_t *c, uint8_t out[ATECC_REVISION_LEN])
{
	int rc;

	if (c == NULL || out == NULL) {
		return -EINVAL;
	}
	if (c->have_revision) {
		memcpy(out, c->revision, ATECC_REVISION_LEN);
		return 0;
	}
	rc = cmd_data(c, ATECC_OP_INFO, ATECC_INFO_REVISION, 0x0000U, NULL, 0U,
		      out, ATECC_REVISION_LEN);
	if (rc == 0) {
		memcpy(c->revision, out, ATECC_REVISION_LEN);
		c->have_revision = true;
	}
	return rc;
}

int atecc_serial(atecc_ctx_t *c, uint8_t out[ATECC_SERIAL_LEN])
{
	uint8_t block[32];
	int rc;

	if (c == NULL || out == NULL) {
		return -EINVAL;
	}
	if (c->have_serial) {
		memcpy(out, c->serial, ATECC_SERIAL_LEN);
		return 0;
	}

	rc = atecc_read_zone(c, ATECC_ZONE_CONFIG, ATECC_CFG_WORD_BLOCK0, block,
			     sizeof(block));
	if (rc != 0) {
		return rc;
	}

	/* SN[0..3] at config byte 0, SN[4..8] at config byte 8. Bytes 4..7 are
	 * the revision, which is why the serial is not one contiguous run. */
	memcpy(&c->serial[0], &block[ATECC_CFG_OFF_SN03], 4U);
	memcpy(&c->serial[4], &block[ATECC_CFG_OFF_SN48], 5U);
	c->have_serial = true;
	memcpy(out, c->serial, ATECC_SERIAL_LEN);
	return 0;
}

int atecc_random(atecc_ctx_t *c, uint8_t out[32])
{
	if (c == NULL || out == NULL) {
		return -EINVAL;
	}
	return cmd_data(c, ATECC_OP_RANDOM, ATECC_RANDOM_NO_SEED, 0x0000U, NULL,
			0U, out, 32U);
}

int atecc_nonce_passthrough(atecc_ctx_t *c,
			    const uint8_t digest[ATECC_DIGEST_LEN])
{
	if (c == NULL || digest == NULL) {
		return -EINVAL;
	}
	/* Passthrough answers with a bare success status: the value went into
	 * TempKey, nothing comes back. */
	return cmd_status(c, ATECC_OP_NONCE, ATECC_NONCE_PASSTHROUGH, 0x0000U,
			  digest, ATECC_DIGEST_LEN);
}

int atecc_sign(atecc_ctx_t *c, uint16_t key_id,
	       const uint8_t digest[ATECC_DIGEST_LEN],
	       uint8_t sig[ATECC_SIG_LEN])
{
	int rc;

	if (c == NULL || digest == NULL || sig == NULL) {
		return -EINVAL;
	}
	rc = atecc_nonce_passthrough(c, digest);
	if (rc != 0) {
		return rc;
	}
	return cmd_data(c, ATECC_OP_SIGN, ATECC_SIGN_EXTERNAL, key_id, NULL, 0U,
			sig, ATECC_SIG_LEN);
}

int atecc_verify_extern(atecc_ctx_t *c, const uint8_t pub[ATECC_PUBKEY_LEN],
			const uint8_t digest[ATECC_DIGEST_LEN],
			const uint8_t sig[ATECC_SIG_LEN], bool *out_ok)
{
	uint8_t data[ATECC_SIG_LEN + ATECC_PUBKEY_LEN];
	int rc;

	if (c == NULL || pub == NULL || digest == NULL || sig == NULL) {
		return -EINVAL;
	}
	if (out_ok != NULL) {
		*out_ok = false;
	}

	rc = atecc_nonce_passthrough(c, digest);
	if (rc != 0) {
		return rc;
	}

	memcpy(&data[0], sig, ATECC_SIG_LEN);
	memcpy(&data[ATECC_SIG_LEN], pub, ATECC_PUBKEY_LEN);
	rc = cmd_status(c, ATECC_OP_VERIFY, ATECC_VERIFY_EXTERNAL,
			ATECC_KEY_TYPE_P256, data, sizeof(data));
	if (rc == 0 && out_ok != NULL) {
		*out_ok = true;
	}
	return rc;
}

int atecc_genkey_private(atecc_ctx_t *c, uint16_t key_id,
			 uint8_t pub[ATECC_PUBKEY_LEN])
{
	if (c == NULL || pub == NULL) {
		return -EINVAL;
	}
	return cmd_data(c, ATECC_OP_GENKEY, ATECC_GENKEY_PRIVATE, key_id, NULL,
			0U, pub, ATECC_PUBKEY_LEN);
}

int atecc_pubkey(atecc_ctx_t *c, uint16_t key_id, uint8_t pub[ATECC_PUBKEY_LEN])
{
	if (c == NULL || pub == NULL) {
		return -EINVAL;
	}
	return cmd_data(c, ATECC_OP_GENKEY, ATECC_GENKEY_PUBLIC, key_id, NULL,
			0U, pub, ATECC_PUBKEY_LEN);
}

int atecc_ecdh(atecc_ctx_t *c, uint16_t key_id,
	       const uint8_t peer[ATECC_PUBKEY_LEN], uint8_t secret[32])
{
	if (c == NULL || peer == NULL || secret == NULL) {
		return -EINVAL;
	}
	return cmd_data(c, ATECC_OP_ECDH, ATECC_ECDH_OUTPUT_CLEAR, key_id, peer,
			ATECC_PUBKEY_LEN, secret, 32U);
}

int atecc_sha256(atecc_ctx_t *c, const uint8_t *msg, size_t len,
		 uint8_t out[ATECC_DIGEST_LEN])
{
	size_t off = 0U;
	int rc;

	if (c == NULL || out == NULL || (msg == NULL && len != 0U)) {
		return -EINVAL;
	}

	rc = cmd_status(c, ATECC_OP_SHA, ATECC_SHA_START, 0x0000U, NULL, 0U);
	if (rc != 0) {
		return rc;
	}

	/* Update takes exactly one block; the tail (0..63 octets) goes to End,
	 * which is also where the padding is applied by the part. */
	while ((len - off) >= ATECC_SHA_BLOCK) {
		rc = cmd_status(c, ATECC_OP_SHA, ATECC_SHA_UPDATE,
				(uint16_t)ATECC_SHA_BLOCK, &msg[off],
				ATECC_SHA_BLOCK);
		if (rc != 0) {
			return rc;
		}
		off += ATECC_SHA_BLOCK;
	}

	return cmd_data(c, ATECC_OP_SHA, ATECC_SHA_END, (uint16_t)(len - off),
			(len - off) != 0U ? &msg[off] : NULL, len - off, out,
			ATECC_DIGEST_LEN);
}

int atecc_hmac(atecc_ctx_t *c, uint16_t key_id, uint8_t mode,
	       uint8_t out[ATECC_DIGEST_LEN])
{
	if (c == NULL || out == NULL) {
		return -EINVAL;
	}
	return cmd_data(c, ATECC_OP_HMAC, mode, key_id, NULL, 0U, out,
			ATECC_DIGEST_LEN);
}

int atecc_read_zone(atecc_ctx_t *c, uint8_t zone, uint16_t word, uint8_t *out,
		    size_t len)
{
	uint8_t p1;

	if (c == NULL || out == NULL) {
		return -EINVAL;
	}
	if (len != 4U && len != 32U) {
		return -EINVAL;
	}
	if ((zone & ~(uint8_t)ATECC_ZONE_MASK) != 0U) {
		return -EINVAL;
	}

	p1 = (uint8_t)(zone & ATECC_ZONE_MASK);
	if (len == 32U) {
		p1 |= (uint8_t)ATECC_ZONE_32;
	}
	return cmd_data(c, ATECC_OP_READ, p1, word, NULL, 0U, out, len);
}

int atecc_write_zone(atecc_ctx_t *c, uint8_t zone, uint16_t word,
		     const uint8_t *in, size_t len)
{
	uint8_t p1;

	if (c == NULL || in == NULL) {
		return -EINVAL;
	}
	if (len != 4U && len != 32U) {
		return -EINVAL;
	}
	if ((zone & ~(uint8_t)ATECC_ZONE_MASK) != 0U) {
		return -EINVAL;
	}

	p1 = (uint8_t)(zone & ATECC_ZONE_MASK);
	if (len == 32U) {
		p1 |= (uint8_t)ATECC_ZONE_32;
	}
	return cmd_status(c, ATECC_OP_WRITE, p1, word, in, len);
}

int atecc_lock(atecc_ctx_t *c, uint8_t zone, uint16_t crc, bool ignore_crc)
{
	uint8_t p1;

	if (c == NULL) {
		return -EINVAL;
	}
	if (zone != ATECC_LOCK_ZONE_CONFIG && zone != ATECC_LOCK_ZONE_DATA &&
	    zone != ATECC_LOCK_ZONE_SLOT) {
		return -EINVAL;
	}

	p1 = zone;
	if (ignore_crc) {
		p1 |= (uint8_t)ATECC_LOCK_NO_CRC;
		crc = 0U;
	}
	return cmd_status(c, ATECC_OP_LOCK, p1, crc, NULL, 0U);
}

/** Both counter modes answer with the four-octet counter value, big-endian. */
static int counter_op(atecc_ctx_t *c, uint8_t mode, uint8_t idx, uint32_t *out)
{
	uint8_t val[4];
	int rc;

	if (c == NULL || out == NULL) {
		return -EINVAL;
	}
	if (idx >= ATECC_COUNTER_COUNT) {
		return -EINVAL;
	}

	rc = cmd_data(c, ATECC_OP_COUNTER, mode, (uint16_t)idx, NULL, 0U, val,
		      sizeof(val));
	if (rc != 0) {
		return rc;
	}
	*out = ((uint32_t)val[0] << 24) | ((uint32_t)val[1] << 16) |
	       ((uint32_t)val[2] << 8) | (uint32_t)val[3];
	return 0;
}

int atecc_counter_read(atecc_ctx_t *c, uint8_t idx, uint32_t *out)
{
	return counter_op(c, ATECC_COUNTER_READ, idx, out);
}

int atecc_counter_increment(atecc_ctx_t *c, uint8_t idx, uint32_t *out)
{
	return counter_op(c, ATECC_COUNTER_INCREMENT, idx, out);
}

int atecc_selftest(atecc_ctx_t *c, uint8_t mode, uint8_t *out_result)
{
	uint8_t pkt[ATECC_PKT_MAX];
	uint8_t rsp[ATECC_RSP_MAX];
	atecc_rsp_t view;
	uint8_t result;
	int n;
	int rc;

	if (c == NULL) {
		return -EINVAL;
	}
	if (mode == 0U || (mode & ~(uint8_t)ATECC_SELFTEST_ALL) != 0U) {
		return -EINVAL;
	}
	memset(&view, 0, sizeof(view));

	n = atecc_pkt_build(pkt, sizeof(pkt), ATECC_OP_SELFTEST, mode, 0x0000U,
			    NULL, 0U);
	if (n < 0) {
		return n;
	}

	/*
	 * SelfTest is the one command whose *failure* is its data: the answer is
	 * a single octet whose set bits name the subsystems that failed. A
	 * one-octet answer is a four-octet response, which is the same shape as
	 * a status code — so the "status" byte of a SelfTest response *is* the
	 * result bitmap, and 0x00 means every selected test passed.
	 *
	 * That makes result 0x0F indistinguishable from a genuine execution
	 * error, and 0x03 from a parse error. The ambiguity is in the protocol,
	 * not in this decode; the resolution taken here is that a value the
	 * requested mode could have produced (within ATECC_SELFTEST_ALL) is
	 * read as a result, and anything outside that as an error. Values that
	 * mean "ask again" (0x11, 0xFF) never reach here — atecc_exec() retries
	 * them and reports -EIO if they persist.
	 */
	rc = atecc_exec(c, pkt, (size_t)n, 0U, rsp, sizeof(rsp), &view);
	if (rc == -ENODEV || rc == -EIO || rc == -EINVAL) {
		return rc;
	}

	if (view.is_status) {
		if ((view.status & ~(uint8_t)ATECC_SELFTEST_ALL) != 0U) {
			return atecc_status_to_errno(view.status);
		}
		result = view.status;
	} else {
		if (view.payload_len < 1U) {
			return -EPROTO;
		}
		result = view.payload[0];
	}

	if (out_result != NULL) {
		*out_result = result;
	}
	return (result == 0U) ? 0 : -EBADE;
}

/* ========================================================================= */
/* attestation                                                               */
/* ========================================================================= */

int atecc_decode_locks(const uint8_t *cfg_block, size_t len, bool *out_data,
		       bool *out_config)
{
	if (cfg_block == NULL) {
		return -EINVAL;
	}
	if (len <= ATECC_CFG_BLK_OFF_LOCK_CONFIG) {
		return -EINVAL;
	}
	if (out_data != NULL) {
		*out_data = (cfg_block[ATECC_CFG_BLK_OFF_LOCK_VALUE] != 0x55U);
	}
	if (out_config != NULL) {
		*out_config = (cfg_block[ATECC_CFG_BLK_OFF_LOCK_CONFIG] != 0x55U);
	}
	return 0;
}

int atecc_attest(atecc_ctx_t *c, uint16_t key_id, atecc_attest_t *out)
{
	uint8_t block[32];
	uint8_t i;

	if (c == NULL || out == NULL) {
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));
	if (!c->ready || c->absent) {
		return -ENODEV;
	}
	if (atecc_wake(c) != 0) {
		return -ENODEV;
	}
	out->present = true;

	(void)atecc_serial(c, out->serial);
	(void)atecc_revision(c, out->revision);

	for (i = 0U; i < (uint8_t)ATECC_COUNTER_COUNT; i++) {
		out->counter_valid[i] =
			(atecc_counter_read(c, i, &out->counter[i]) == 0);
	}

	if (atecc_read_zone(c, ATECC_ZONE_CONFIG, ATECC_CFG_WORD_LOCKS, block,
			    sizeof(block)) == 0) {
		out->lock_valid =
			(atecc_decode_locks(block, sizeof(block),
					    &out->data_locked,
					    &out->config_locked) == 0);
	}

	/* A public key can only be derived once the data zone is locked, so an
	 * unprovisioned part legitimately has none. */
	out->pubkey_valid = (atecc_pubkey(c, key_id, out->pubkey) == 0);

	{
		uint8_t result = 0U;
		int rc = atecc_selftest(c, (uint8_t)ATECC_SELFTEST_ALL, &result);

		if (rc == 0 || rc == -EBADE) {
			out->selftest_result = result;
			out->selftest_valid = true;
		}
	}

	return 0;
}
