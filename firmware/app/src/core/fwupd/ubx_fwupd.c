/*
 * STS1000 "Meridian" — core/fwupd: u-blox safeboot loader client.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See ubx_fwupd.h for the VERIFY status of the opcode table and for the
 * fail-safe contract. The structure of this file exists to make that contract
 * checkable: every function that can fail returns through fail(), and fail() is
 * the only path into UBX_FWUPD_ST_FAILED — and it calls ubx_fwupd_recover()
 * first.
 */

#include "fwupd/ubx_fwupd.h"

#include <errno.h>
#include <string.h>

#include "util/bytes.h"

const uint32_t ubx_fwupd_baud_list[UBX_FWUPD_BAUD_COUNT] = {
	9600U, 115200U, 38400U, 57600U,
};

void ubx_fwupd_cfg_defaults(ubx_fwupd_cfg_t *cfg)
{
	if (cfg == NULL) {
		return;
	}
	(void)memset(cfg, 0, sizeof(*cfg));
	/*
	 * Disarmed. The opcode table is an informed reconstruction, not a
	 * specification, and arming it is a deliberate act by whoever verified it.
	 */
	cfg->opcodes_verified = false;
	cfg->operating_baud = 38400U;
	cfg->chunk = (uint16_t)UBX_FWUPD_CHUNK;
}

const char *ubx_fwupd_state_name(uint8_t st)
{
	switch (st) {
	case (uint8_t)UBX_FWUPD_ST_IDLE:
		return "IDLE";
	case (uint8_t)UBX_FWUPD_ST_ENTER_SAFEBOOT:
		return "ENTER_SAFEBOOT";
	case (uint8_t)UBX_FWUPD_ST_NEGOTIATE:
		return "NEGOTIATE";
	case (uint8_t)UBX_FWUPD_ST_ERASE:
		return "ERASE";
	case (uint8_t)UBX_FWUPD_ST_WRITE:
		return "WRITE";
	case (uint8_t)UBX_FWUPD_ST_FINALISE:
		return "FINALISE";
	case (uint8_t)UBX_FWUPD_ST_LEAVE_SAFEBOOT:
		return "LEAVE_SAFEBOOT";
	case (uint8_t)UBX_FWUPD_ST_VERIFY:
		return "VERIFY";
	case (uint8_t)UBX_FWUPD_ST_DONE:
		return "DONE";
	case (uint8_t)UBX_FWUPD_ST_FAILED:
		return "FAILED";
	default:
		return "?";
	}
}

/* ------------------------------------------------------------- internals -- */

static uint64_t now(const ubx_fwupd_t *u)
{
	return u->ops.now_ms(u->ops.user);
}

static void hold(const ubx_fwupd_t *u, uint32_t ms)
{
	if (u->ops.delay_ms != NULL) {
		u->ops.delay_ms(u->ops.user, ms);
	}
}

static uint16_t chunk_size(const ubx_fwupd_t *u)
{
	uint16_t n = u->cfg.chunk;

	if ((n == 0U) || (n > (uint16_t)UBX_FWUPD_CHUNK)) {
		n = (uint16_t)UBX_FWUPD_CHUNK;
	}
	return n;
}

static bool running(const ubx_fwupd_t *u)
{
	return (u->state != (uint8_t)UBX_FWUPD_ST_IDLE) &&
	       (u->state != (uint8_t)UBX_FWUPD_ST_DONE) &&
	       (u->state != (uint8_t)UBX_FWUPD_ST_FAILED);
}

/*
 * The only way into FAILED. Recovery runs first, unconditionally, so the
 * receiver cannot be left in safeboot by a path somebody forgot to unwind.
 */
static int fail(ubx_fwupd_t *u, int rc)
{
	(void)ubx_fwupd_recover(u);
	u->state = (uint8_t)UBX_FWUPD_ST_FAILED;
	u->last_rc = rc;
	return rc;
}

/* Drain the UART into the UBX parser, returning the first complete frame. */
static int pump(ubx_fwupd_t *u, ubx_msg_t *out, bool *have)
{
	uint8_t buf[64];
	int n;
	int i;

	*have = false;

	n = u->ops.rx(u->ops.user, buf, sizeof(buf));
	if (n < 0) {
		return n;
	}
	for (i = 0; i < n; i++) {
		if (ubx_parse_byte(&u->parser, buf[i]) == 1) {
			if (ubx_parser_msg(&u->parser, out) == 0) {
				*have = true;
				/*
				 * Return on the first frame rather than draining
				 * the rest: the remaining octets stay in the
				 * driver's buffer and are picked up next call, so
				 * no frame is lost and none is skipped.
				 */
				return 0;
			}
		}
	}
	return 0;
}

/*
 * Wait for an ACK-ACK/ACK-NAK for (cls, id), or for a specific class/id reply.
 *
 * @param want_cls/@p want_id  a non-ACK reply to accept instead, or 0/0 for none.
 *
 * @retval 1         ACK-ACK, or the requested reply, arrived.
 * @retval 0         ACK-NAK arrived.
 * @retval -ETIMEDOUT nothing arrived inside @p budget_ms.
 * @retval negative  the transport failed.
 */
static int await_ack(ubx_fwupd_t *u, uint8_t cls, uint8_t id, uint8_t want_cls,
		     uint8_t want_id, uint32_t budget_ms, ubx_msg_t *reply)
{
	uint64_t deadline = now(u) + (uint64_t)budget_ms;

	for (;;) {
		ubx_msg_t m;
		bool have = false;
		int rc = pump(u, &m, &have);

		if (rc != 0) {
			return rc;
		}
		if (have) {
			ubx_ack_t a;

			if ((want_cls != 0U) && ubx_msg_is(&m, want_cls, want_id)) {
				if (reply != NULL) {
					*reply = m;
				}
				return 1;
			}
			if (ubx_parse_ack(&m, &a) == 0) {
				if ((a.cls_id == cls) && (a.msg_id == id)) {
					if (a.ack) {
						u->acks++;
						return 1;
					}
					u->naks++;
					return 0;
				}
			}
			/* Anything else on the wire is not for us; keep waiting. */
		}
		if (now(u) >= deadline) {
			u->timeouts++;
			return -ETIMEDOUT;
		}
		hold(u, 1U);
	}
}

static int send_frame(ubx_fwupd_t *u, uint8_t id, const uint8_t *payload,
		      size_t len)
{
	int n;
	int rc;

	n = ubx_frame(u->txbuf, sizeof(u->txbuf), (uint8_t)UBX_FWUPD_CLASS, id,
		      payload, (uint16_t)len);
	if (n < 0) {
		return n;
	}
	rc = u->ops.tx(u->ops.user, u->txbuf, (size_t)n);
	if (rc != 0) {
		return rc;
	}
	u->frames_sent++;
	return 0;
}

static int set_baud(ubx_fwupd_t *u, uint32_t baud)
{
	int rc = u->ops.set_baud(u->ops.user, baud);

	if (rc == 0) {
		u->baud = baud;
		ubx_parser_reset(&u->parser);
	}
	return rc;
}

/* ------------------------------------------------------------ lifecycle --- */

int ubx_fwupd_init(ubx_fwupd_t *u, const ubx_fwupd_cfg_t *cfg,
		   const ubx_fwupd_ops_t *ops)
{
	if ((u == NULL) || (cfg == NULL) || (ops == NULL) || (ops->tx == NULL) ||
	    (ops->rx == NULL) || (ops->set_baud == NULL) ||
	    (ops->set_safeboot == NULL) || (ops->set_reset == NULL) ||
	    (ops->now_ms == NULL)) {
		return -EINVAL;
	}

	(void)memset(u, 0, sizeof(*u));
	u->cfg = *cfg;
	u->ops = *ops;
	u->state = (uint8_t)UBX_FWUPD_ST_IDLE;
	u->baud = cfg->operating_baud;
	(void)ubx_parser_init(&u->parser, u->parse_buf, (uint16_t)sizeof(u->parse_buf));
	return 0;
}

int ubx_fwupd_recover(ubx_fwupd_t *u)
{
	int rc;
	int first = 0;

	if (u == NULL) {
		return -EINVAL;
	}

	u->recoveries++;

	/*
	 * Order matters. Release safeboot BEFORE the reset pulse: the receiver
	 * samples GPS_SAFEBOOT_N as it comes out of reset, so releasing it after
	 * the pulse would boot it straight back into the loader.
	 */
	rc = u->ops.set_safeboot(u->ops.user, false);
	if (rc != 0) {
		first = rc;
	}
	u->safeboot_asserted = false;

	hold(u, UBX_FWUPD_SAFEBOOT_SETUP_MS);

	rc = u->ops.set_reset(u->ops.user, true);
	if ((rc != 0) && (first == 0)) {
		first = rc;
	}
	hold(u, UBX_FWUPD_RESET_HOLD_MS);
	rc = u->ops.set_reset(u->ops.user, false);
	if ((rc != 0) && (first == 0)) {
		first = rc;
	}

	/* Back to the rate the rest of the firmware expects. */
	rc = u->ops.set_baud(u->ops.user, u->cfg.operating_baud);
	if ((rc != 0) && (first == 0)) {
		first = rc;
	}
	u->baud = u->cfg.operating_baud;
	ubx_parser_reset(&u->parser);

	return first;
}

bool ubx_fwupd_in_safeboot(const ubx_fwupd_t *u)
{
	return (u != NULL) ? u->safeboot_asserted : false;
}

uint8_t ubx_fwupd_state(const ubx_fwupd_t *u)
{
	return (u != NULL) ? u->state : (uint8_t)UBX_FWUPD_ST_IDLE;
}

const ubx_mon_ver_t *ubx_fwupd_version(const ubx_fwupd_t *u)
{
	if ((u == NULL) || !u->ver_valid) {
		return NULL;
	}
	return &u->ver;
}

/* -------------------------------------------------------- version query --- */

/* Render a MON-VER into the one-line summary the inventory shows. */
static void ver_summary(const ubx_mon_ver_t *v, char *out, size_t cap)
{
	const char *fw;
	size_t n = 0U;
	size_t k;

	if (cap == 0U) {
		return;
	}
	out[0] = '\0';

	k = strlen(v->sw_version);
	if (k >= cap) {
		k = cap - 1U;
	}
	(void)memcpy(out, v->sw_version, k);
	n = k;
	out[n] = '\0';

	fw = ubx_mon_ver_ext(v, "FWVER");
	if ((fw != NULL) && ((n + 2U) < cap)) {
		out[n++] = ' ';
		k = strlen(fw);
		if (k > ((cap - 1U) - n)) {
			k = (cap - 1U) - n;
		}
		(void)memcpy(&out[n], fw, k);
		n += k;
		out[n] = '\0';
	}
}

static int poll_mon_ver(ubx_fwupd_t *u, uint32_t budget_ms)
{
	uint8_t req[UBX_FRAME_OVERHEAD];
	ubx_msg_t reply;
	int n;
	int rc;

	n = ubx_poll(req, sizeof(req), UBX_CLASS_MON, UBX_ID_MON_VER);
	if (n < 0) {
		return n;
	}
	rc = u->ops.tx(u->ops.user, req, (size_t)n);
	if (rc != 0) {
		return rc;
	}

	rc = await_ack(u, UBX_CLASS_MON, UBX_ID_MON_VER, UBX_CLASS_MON,
		       UBX_ID_MON_VER, budget_ms, &reply);
	if (rc < 0) {
		return rc;
	}
	if (rc == 0) {
		return -EIO; /* NAK to a MON-VER poll: not a working receiver */
	}

	rc = ubx_parse_mon_ver(&reply, &u->ver);
	if (rc != 0) {
		return rc;
	}
	u->ver_valid = true;
	return 0;
}

int ubx_fwupd_query_version(ubx_fwupd_t *u, char *out, size_t cap)
{
	int rc;

	if ((u == NULL) || (out == NULL) || (cap == 0U)) {
		return -EINVAL;
	}
	out[0] = '\0';
	if (running(u)) {
		return -EBUSY;
	}

	rc = poll_mon_ver(u, (uint32_t)UBX_FWUPD_ACK_MS);
	if (rc != 0) {
		return rc;
	}
	ver_summary(&u->ver, out, cap);
	return 0;
}

/* --------------------------------------------------------------- safeboot -- */

/*
 * The pin sequence, per docs/sts1000_firmware_hardware_interface.md §9's
 * polarity and the ZED-F9T reset timing:
 *
 *   SAFEBOOT_N low  ->  setup  ->  RST_N low  ->  hold  ->  RST_N high
 *                   ->  boot   ->  loader listening, SAFEBOOT_N still low
 *
 * SAFEBOOT_N stays asserted for the whole session: releasing it early on some
 * parts drops the loader.
 */
static int enter_safeboot(ubx_fwupd_t *u)
{
	int rc;

	rc = u->ops.set_safeboot(u->ops.user, true);
	if (rc != 0) {
		return rc;
	}
	u->safeboot_asserted = true;
	hold(u, UBX_FWUPD_SAFEBOOT_SETUP_MS);

	rc = u->ops.set_reset(u->ops.user, true);
	if (rc != 0) {
		return rc;
	}
	hold(u, UBX_FWUPD_RESET_HOLD_MS);

	rc = u->ops.set_reset(u->ops.user, false);
	if (rc != 0) {
		return rc;
	}
	hold(u, UBX_FWUPD_SAFEBOOT_HOLD_MS);
	hold(u, UBX_FWUPD_BOOT_MS);
	return 0;
}

/* Try each baud in turn until the loader answers an IDENT. */
static int negotiate(ubx_fwupd_t *u)
{
	unsigned int i;

	for (i = 0U; i < (unsigned int)UBX_FWUPD_BAUD_COUNT; i++) {
		unsigned int t;

		if (set_baud(u, ubx_fwupd_baud_list[i]) != 0) {
			continue;
		}
		for (t = 0U; t < (unsigned int)UBX_FWUPD_PROBE_RETRIES; t++) {
			int rc = send_frame(u, (uint8_t)UBX_FWUPD_ID_IDENT, NULL,
					    0U);

			if (rc != 0) {
				return rc;
			}
			rc = await_ack(u, (uint8_t)UBX_FWUPD_CLASS,
				       (uint8_t)UBX_FWUPD_ID_IDENT,
				       (uint8_t)UBX_FWUPD_CLASS,
				       (uint8_t)UBX_FWUPD_ID_IDENT,
				       (uint32_t)UBX_FWUPD_ACK_MS, NULL);
			if (rc == 1) {
				u->baud_idx = (uint8_t)i;
				/* Move to the bulk rate for the image. */
				if (set_baud(u, (uint32_t)UBX_FWUPD_XFER_BAUD) != 0) {
					/* Stay where we are; slower but correct. */
					(void)set_baud(u, ubx_fwupd_baud_list[i]);
				}
				return 0;
			}
			if (rc < 0 && rc != -ETIMEDOUT) {
				return rc;
			}
			u->retries++;
		}
	}
	return -ENODEV;
}

/* ------------------------------------------------------------------ API --- */

int ubx_fwupd_begin(ubx_fwupd_t *u, uint32_t image_size)
{
	int rc;

	if ((u == NULL) || (image_size == 0U)) {
		return -EINVAL;
	}
	if (running(u)) {
		return -EBUSY;
	}

	/*
	 * The gate. Refused BEFORE any pin moves, so a disarmed module cannot
	 * leave the receiver in safeboot even in principle — which is the whole
	 * reason the unverified opcodes are safe to have in the tree.
	 */
	if (!u->cfg.opcodes_verified) {
		u->last_rc = -ENOTSUP;
		return -ENOTSUP;
	}

	u->total = image_size;
	u->done = 0U;
	u->chunk_tries = 0U;
	u->ver_valid = false;
	u->last_rc = 0;

	u->state = (uint8_t)UBX_FWUPD_ST_ENTER_SAFEBOOT;
	rc = enter_safeboot(u);
	if (rc != 0) {
		return fail(u, rc);
	}

	u->state = (uint8_t)UBX_FWUPD_ST_NEGOTIATE;
	rc = negotiate(u);
	if (rc != 0) {
		return fail(u, rc);
	}

	u->state = (uint8_t)UBX_FWUPD_ST_ERASE;
	{
		uint8_t pl[4];

		bytes_put_le32(pl, image_size);
		rc = send_frame(u, (uint8_t)UBX_FWUPD_ID_ERASE, pl, sizeof(pl));
		if (rc != 0) {
			return fail(u, rc);
		}
	}
	rc = await_ack(u, (uint8_t)UBX_FWUPD_CLASS, (uint8_t)UBX_FWUPD_ID_ERASE,
		       0U, 0U, (uint32_t)UBX_FWUPD_ERASE_MS, NULL);
	if (rc < 0) {
		return fail(u, rc);
	}
	if (rc == 0) {
		return fail(u, -EIO);
	}

	u->state = (uint8_t)UBX_FWUPD_ST_WRITE;
	u->deadline_ms = now(u) + (uint64_t)UBX_FWUPD_ACK_MS;
	return 0;
}

int ubx_fwupd_write(ubx_fwupd_t *u, uint32_t off, const uint8_t *data, size_t len)
{
	uint8_t pl[4U + UBX_FWUPD_CHUNK];
	unsigned int attempt;

	if ((u == NULL) || (data == NULL) || (len == 0U)) {
		return -EINVAL;
	}
	if (u->state != (uint8_t)UBX_FWUPD_ST_WRITE) {
		return -EPERM;
	}
	if (len > (size_t)chunk_size(u)) {
		return -EINVAL;
	}
	if (off != u->done) {
		/* The loader has no seek: offsets must arrive in order. */
		return -EINVAL;
	}
	if (((uint64_t)off + (uint64_t)len) > (uint64_t)u->total) {
		return -EINVAL;
	}

	bytes_put_le32(pl, off);
	(void)memcpy(&pl[4], data, len);

	for (attempt = 0U; attempt <= (unsigned int)UBX_FWUPD_CHUNK_RETRIES;
	     attempt++) {
		int rc;

		if (attempt > 0U) {
			u->retries++;
		}
		rc = send_frame(u, (uint8_t)UBX_FWUPD_ID_WRITE, pl, 4U + len);
		if (rc != 0) {
			return fail(u, rc);
		}
		rc = await_ack(u, (uint8_t)UBX_FWUPD_CLASS,
			       (uint8_t)UBX_FWUPD_ID_WRITE, 0U, 0U,
			       (uint32_t)UBX_FWUPD_ACK_MS, NULL);
		if (rc == 1) {
			u->done += (uint32_t)len;
			u->chunk_tries = 0U;
			/*
			 * Refresh the stall deadline on progress, not on entry:
			 * the budget is "time since the last acknowledged chunk",
			 * so a large image does not trip a timer sized for one
			 * command exchange.
			 */
			u->deadline_ms = now(u) + (uint64_t)UBX_FWUPD_ACK_MS;
			return 0;
		}
		if ((rc < 0) && (rc != -ETIMEDOUT)) {
			return fail(u, rc);
		}
		/* NAK or timeout: resend the same chunk. */
	}

	/*
	 * Out of retries. The image is now partially written, which is exactly the
	 * state that must not be left behind safeboot: recover so the receiver
	 * reboots and the operator can retry from a known state.
	 */
	return fail(u, -EIO);
}

int ubx_fwupd_finish(ubx_fwupd_t *u)
{
	int rc;

	if (u == NULL) {
		return -EINVAL;
	}
	if (u->state != (uint8_t)UBX_FWUPD_ST_WRITE) {
		return -EPERM;
	}
	if (u->done != u->total) {
		return fail(u, -EIO);
	}

	u->state = (uint8_t)UBX_FWUPD_ST_FINALISE;
	rc = send_frame(u, (uint8_t)UBX_FWUPD_ID_FINALISE, NULL, 0U);
	if (rc != 0) {
		return fail(u, rc);
	}
	rc = await_ack(u, (uint8_t)UBX_FWUPD_CLASS,
		       (uint8_t)UBX_FWUPD_ID_FINALISE, 0U, 0U,
		       (uint32_t)UBX_FWUPD_ERASE_MS, NULL);
	if (rc < 0) {
		return fail(u, rc);
	}
	if (rc == 0) {
		return fail(u, -EIO);
	}

	/*
	 * Leave safeboot. This is the same primitive the failure paths use — a
	 * successful update and a failed one exit through identical pin handling,
	 * so there is only one sequence to get right.
	 */
	u->state = (uint8_t)UBX_FWUPD_ST_LEAVE_SAFEBOOT;
	rc = ubx_fwupd_recover(u);
	if (rc != 0) {
		u->state = (uint8_t)UBX_FWUPD_ST_FAILED;
		u->last_rc = rc;
		return rc;
	}
	hold(u, UBX_FWUPD_APP_BOOT_MS);

	u->state = (uint8_t)UBX_FWUPD_ST_VERIFY;
	rc = poll_mon_ver(u, (uint32_t)UBX_FWUPD_ACK_MS);
	if (rc != 0) {
		/*
		 * The bytes are committed but the receiver did not answer. Do not
		 * report success: an operator has to know the difference between
		 * "updated" and "wrote an image and then lost the receiver".
		 * recover() has already run, so it is not held in safeboot.
		 */
		u->state = (uint8_t)UBX_FWUPD_ST_FAILED;
		u->last_rc = rc;
		return rc;
	}

	u->state = (uint8_t)UBX_FWUPD_ST_DONE;
	return 0;
}

int ubx_fwupd_abort(ubx_fwupd_t *u)
{
	if (u == NULL) {
		return -EINVAL;
	}
	if (!running(u)) {
		/* Still release the pins: a previous session may have died hard. */
		(void)ubx_fwupd_recover(u);
		u->state = (uint8_t)UBX_FWUPD_ST_IDLE;
		return 0;
	}
	(void)fail(u, -ECANCELED);
	return 0;
}

int ubx_fwupd_step(ubx_fwupd_t *u)
{
	if (u == NULL) {
		return -EINVAL;
	}
	if (!running(u)) {
		return 0;
	}

	/*
	 * Only the WRITE phase is genuinely asynchronous from the caller's point
	 * of view; the others complete inside their own entry point. What step()
	 * adds is the stall timeout, so a tool that opens a session and then goes
	 * away cannot leave the receiver behind safeboot forever.
	 */
	if (u->state == (uint8_t)UBX_FWUPD_ST_WRITE) {
		if (now(u) >= u->deadline_ms) {
			u->timeouts++;
			return fail(u, -ETIMEDOUT);
		}
		return -EAGAIN;
	}
	return -EAGAIN;
}
