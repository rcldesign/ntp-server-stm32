/*
 * STS1000 "Meridian" — core/fwupd: FE-5680A serial client.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See rb_fwupd.h for the framing, the capability classes and why the firmware
 * update path reports NOT_SUPPORTED.
 */

#include "fwupd/rb_fwupd.h"

#include <errno.h>
#include <string.h>

/* ------------------------------------------------------------- framing --- */

static uint8_t xor_sum(const uint8_t *p, size_t n)
{
	uint8_t x = 0U;
	size_t i;

	for (i = 0U; i < n; i++) {
		x ^= p[i];
	}
	return x;
}

int rb_frame_encode(uint8_t *buf, size_t cap, uint8_t id, const uint8_t *data,
		    size_t data_len, size_t *out_len)
{
	size_t total;

	if (buf == NULL) {
		return -EINVAL;
	}
	if ((data == NULL) && (data_len != 0U)) {
		return -EINVAL;
	}
	if (data_len > (size_t)RB_FRAME_MAX) {
		return -EINVAL;
	}

	/* Header, plus (data + its own checksum) when there is any data. */
	total = (size_t)RB_FRAME_HDR_LEN + ((data_len != 0U) ? (data_len + 1U) : 0U);
	if (total > cap) {
		return -ENOSPC;
	}

	buf[0] = id;
	/* The length field is little-endian; the offset payload is not. */
	buf[1] = (uint8_t)(total & 0xFFU);
	buf[2] = (uint8_t)((total >> 8) & 0xFFU);
	buf[3] = xor_sum(buf, 3U);

	if (data_len != 0U) {
		(void)memcpy(&buf[RB_FRAME_HDR_LEN], data, data_len);
		buf[RB_FRAME_HDR_LEN + data_len] = xor_sum(data, data_len);
	}

	if (out_len != NULL) {
		*out_len = total;
	}
	return 0;
}

int rb_frame_decode(const uint8_t *buf, size_t len, rb_frame_t *out)
{
	uint16_t adv;
	size_t data_len;

	if ((buf == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (len < (size_t)RB_FRAME_HDR_LEN) {
		return -EBADMSG;
	}
	if (buf[3] != xor_sum(buf, 3U)) {
		return -EBADMSG;
	}

	adv = (uint16_t)((uint16_t)buf[1] | ((uint16_t)buf[2] << 8));
	/*
	 * The advertised length must match exactly. Accepting a frame that
	 * claims to be longer than what arrived would mean decoding an offset
	 * from octets that are not there.
	 */
	if ((size_t)adv != len) {
		return -EBADMSG;
	}
	if (adv == (uint16_t)RB_FRAME_HDR_LEN) {
		(void)memset(out, 0, sizeof(*out));
		out->id = buf[0];
		out->len = adv;
		out->data_len = 0U;
		return 0;
	}
	if (adv < (uint16_t)(RB_FRAME_HDR_LEN + 2U)) {
		/* Data present but no room for a data octet plus its checksum. */
		return -EBADMSG;
	}

	data_len = (size_t)adv - (size_t)RB_FRAME_HDR_LEN - 1U;
	if (data_len > (size_t)RB_FRAME_MAX) {
		return -EBADMSG;
	}
	if (buf[RB_FRAME_HDR_LEN + data_len] !=
	    xor_sum(&buf[RB_FRAME_HDR_LEN], data_len)) {
		return -EBADMSG;
	}

	(void)memset(out, 0, sizeof(*out));
	out->id = buf[0];
	out->len = adv;
	out->data_len = (uint8_t)data_len;
	(void)memcpy(out->data, &buf[RB_FRAME_HDR_LEN], data_len);
	return 0;
}

int rb_offset_encode(int32_t offset, uint8_t out[RB_OFFSET_DATA_LEN])
{
	uint32_t u;

	if (out == NULL) {
		return -EINVAL;
	}
	/* Two's-complement reinterpretation, big-endian on the wire. */
	u = (uint32_t)offset;
	out[0] = (uint8_t)((u >> 24) & 0xFFU);
	out[1] = (uint8_t)((u >> 16) & 0xFFU);
	out[2] = (uint8_t)((u >> 8) & 0xFFU);
	out[3] = (uint8_t)(u & 0xFFU);
	return 0;
}

int rb_offset_decode(const uint8_t in[RB_OFFSET_DATA_LEN], int32_t *out)
{
	uint32_t u;

	if ((in == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	u = ((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16) |
	    ((uint32_t)in[2] << 8) | (uint32_t)in[3];
	/*
	 * Reconstruct the signed value without relying on implementation-defined
	 * conversion of an out-of-range unsigned to int32_t.
	 */
	if ((u & 0x80000000UL) != 0U) {
		*out = -(int32_t)(~u) - 1;
	} else {
		*out = (int32_t)u;
	}
	return 0;
}

/* -------------------------------------------------------------- parser --- */

void rb_parser_init(rb_parser_t *p)
{
	if (p == NULL) {
		return;
	}
	(void)memset(p, 0, sizeof(*p));
}

int rb_parse_byte(rb_parser_t *p, uint8_t b, rb_frame_t *out)
{
	if ((p == NULL) || (out == NULL)) {
		return -EINVAL;
	}

	if (p->n < (uint8_t)RB_FRAME_MAX) {
		p->buf[p->n] = b;
		p->n++;
	} else {
		/* Overlong: drop the oldest octet and keep scanning. */
		(void)memmove(p->buf, &p->buf[1], (size_t)RB_FRAME_MAX - 1U);
		p->buf[RB_FRAME_MAX - 1U] = b;
		p->resyncs++;
	}

	if (p->n < (uint8_t)RB_FRAME_HDR_LEN) {
		return 0;
	}

	if (p->want == 0U) {
		uint16_t adv;

		/*
		 * The protocol has no sync byte, so a corrupt frame can only be
		 * escaped by sliding one octet at a time until a header
		 * checksums. Every frame accepted here has passed both this
		 * check and the data checksum in rb_frame_decode().
		 */
		if (p->buf[3] != xor_sum(p->buf, 3U)) {
			(void)memmove(p->buf, &p->buf[1], (size_t)p->n - 1U);
			p->n--;
			p->resyncs++;
			return 0;
		}
		adv = (uint16_t)((uint16_t)p->buf[1] | ((uint16_t)p->buf[2] << 8));
		if ((adv < (uint16_t)RB_FRAME_HDR_LEN) ||
		    (adv > (uint16_t)RB_FRAME_MAX)) {
			(void)memmove(p->buf, &p->buf[1], (size_t)p->n - 1U);
			p->n--;
			p->resyncs++;
			return 0;
		}
		p->want = adv;
	}

	if ((uint16_t)p->n < p->want) {
		return 0;
	}

	if (rb_frame_decode(p->buf, (size_t)p->want, out) == 0) {
		p->n = 0U;
		p->want = 0U;
		p->frames++;
		return 1;
	}

	/* Data checksum failed: resynchronise past the first octet. */
	(void)memmove(p->buf, &p->buf[1], (size_t)p->n - 1U);
	p->n--;
	p->want = 0U;
	p->resyncs++;
	return 0;
}

/* -------------------------------------------------------- capabilities --- */

const char *rb_cap_name(uint8_t cap)
{
	switch (cap) {
	case (uint8_t)RB_CAP_UNKNOWN:
		return "unknown";
	case (uint8_t)RB_CAP_NONE:
		return "absent";
	case (uint8_t)RB_CAP_TELEMETRY_ONLY:
		return "telemetry-only";
	case (uint8_t)RB_CAP_EFC_TRIMMABLE:
		return "efc-trimmable";
	case (uint8_t)RB_CAP_FW_UPDATABLE:
		return "firmware-updatable";
	default:
		return "?";
	}
}

/* ------------------------------------------------------------- config --- */

void rb_fwupd_cfg_defaults(rb_fwupd_cfg_t *cfg)
{
	if (cfg == NULL) {
		return;
	}
	(void)memset(cfg, 0, sizeof(*cfg));
	cfg->reply_timeout_ms = 1000U;
	/* No documented FE-5680A loader exists; see the header comment. */
	cfg->loader_verified = false;

	/*
	 * +-1e6 counts is about +-0.18 Hz of a 60 MHz reference, roughly 3e-9
	 * fractional — two orders of magnitude inside full scale. A fine trim
	 * that removes a residual the Rb's own loop is not removing needs far
	 * less than this, and a bound that cannot express a gross error cannot
	 * cause one.
	 */
	cfg->trim.min_offset = -1000000L;
	cfg->trim.max_offset = 1000000L;
	cfg->trim.max_step = 100000L;
	cfg->trim.allow_eeprom = false;
	cfg->trim.eeprom_min_interval_s = 3600U;
}

/* ---------------------------------------------------------- internals --- */

static void audit(const rb_ctx_t *c, const char *what, int32_t value, int rc)
{
	if (c->ops.audit != NULL) {
		c->ops.audit(c->ops.user, what, value, rc);
	}
}

static bool rail_ok(const rb_ctx_t *c)
{
	if (c->ops.rail_up == NULL) {
		return true;
	}
	return c->ops.rail_up(c->ops.user);
}

static void hold(const rb_ctx_t *c, uint32_t ms)
{
	if (c->ops.delay_ms != NULL) {
		c->ops.delay_ms(c->ops.user, ms);
	}
}

/* Send a frame and wait for a reply with the same id. */
static int exchange(rb_ctx_t *c, uint8_t id, const uint8_t *data, size_t data_len,
		    rb_frame_t *reply)
{
	uint8_t frame[RB_FRAME_MAX];
	size_t len = 0U;
	uint64_t deadline;
	int rc;

	rc = rb_frame_encode(frame, sizeof(frame), id, data, data_len, &len);
	if (rc != 0) {
		return rc;
	}

	rb_parser_init(&c->parser);

	rc = c->ops.tx(c->ops.user, frame, len);
	if (rc != 0) {
		return rc;
	}
	c->tx_frames++;

	if (reply == NULL) {
		return 0;
	}

	deadline = c->ops.now_ms(c->ops.user) + (uint64_t)c->cfg.reply_timeout_ms;
	for (;;) {
		uint8_t buf[32];
		int n = c->ops.rx(c->ops.user, buf, sizeof(buf));
		int i;

		if (n < 0) {
			return n;
		}
		for (i = 0; i < n; i++) {
			rb_frame_t f;

			if (rb_parse_byte(&c->parser, buf[i], &f) != 1) {
				continue;
			}
			c->rx_frames++;
			if (f.id == id) {
				*reply = f;
				return 0;
			}
			/*
			 * Some variants emit unsolicited status frames. Keep the
			 * most recent unrecognised one so the maintenance tool can
			 * show it — it is the only evidence available about what a
			 * given surplus variant actually speaks.
			 */
			c->status.unknown_frames++;
			c->status.last_unknown = f;
		}
		if (c->ops.now_ms(c->ops.user) >= deadline) {
			c->timeouts++;
			return -ETIMEDOUT;
		}
		hold(c, 1U);
	}
}

/* ------------------------------------------------------------- API --- */

int rb_fwupd_init(rb_ctx_t *c, const rb_fwupd_cfg_t *cfg,
		  const rb_fwupd_ops_t *ops)
{
	if ((c == NULL) || (cfg == NULL) || (ops == NULL) || (ops->tx == NULL) ||
	    (ops->rx == NULL) || (ops->now_ms == NULL)) {
		return -EINVAL;
	}
	(void)memset(c, 0, sizeof(*c));
	c->cfg = *cfg;
	c->ops = *ops;
	c->cap = (uint8_t)RB_CAP_UNKNOWN;
	rb_parser_init(&c->parser);
	return 0;
}

int rb_read_offset(rb_ctx_t *c, int32_t *out)
{
	rb_frame_t reply;
	int32_t v = 0;
	int rc;

	if (c == NULL) {
		return -EINVAL;
	}
	if (!rail_ok(c)) {
		/*
		 * The SN65C3221E is powered from the Rb rail
		 * (docs/rb_rs232_interface.md), so with the rail down there is
		 * nothing to talk to and a timeout would be misread as a dead
		 * unit.
		 */
		return -ENODEV;
	}

	rc = exchange(c, (uint8_t)RB_CMD_REQ_OFFSET, NULL, 0U, &reply);
	if (rc != 0) {
		return rc;
	}
	if (reply.data_len != (uint8_t)RB_OFFSET_DATA_LEN) {
		c->bad_frames++;
		return -EBADMSG;
	}
	(void)rb_offset_decode(reply.data, &v);

	c->status.offset = v;
	c->status.offset_valid = true;
	if (out != NULL) {
		*out = v;
	}
	return 0;
}

int rb_fwupd_probe(rb_ctx_t *c, const rb_probe_req_t *req)
{
	int32_t cur = 0;
	int rc;

	if (c == NULL) {
		return -EINVAL;
	}

	c->cap = (uint8_t)RB_CAP_NONE;

	rc = rb_read_offset(c, &cur);
	audit(c, "probe-read", cur, rc);
	if (rc != 0) {
		/*
		 * Nothing answered. Per docs/rb_rs232_interface.md this is also
		 * what a variant with J6.8/J6.9 swapped looks like, so the class
		 * is "absent" rather than "broken" and the operator is told to
		 * check the cable before the unit.
		 */
		return 0;
	}
	c->cap = (uint8_t)RB_CAP_TELEMETRY_ONLY;

	if ((req == NULL) || !req->probe_trim) {
		return 0;
	}

	/*
	 * Establish trimmability by writing the value the unit already has: a
	 * successful probe is a no-op on a running reference, and a failed one
	 * has not moved anything. Volatile (0x2E) only — a probe must never
	 * consume an EEPROM write.
	 */
	{
		uint8_t data[RB_OFFSET_DATA_LEN];
		rb_frame_t reply;
		int32_t back = 0;

		(void)rb_offset_encode(cur, data);
		rc = exchange(c, (uint8_t)RB_CMD_SET_OFFSET_VOL, data,
			      sizeof(data), &reply);
		if (rc != 0) {
			/*
			 * Many variants do not acknowledge a set at all. Treat a
			 * timeout as "not established" rather than as an error:
			 * the unit is still perfectly usable for telemetry.
			 */
			audit(c, "probe-trim", cur, rc);
			return 0;
		}
		rc = rb_read_offset(c, &back);
		if ((rc == 0) && (back == cur)) {
			c->cap = (uint8_t)RB_CAP_EFC_TRIMMABLE;
			audit(c, "probe-trim", back, 0);
		} else {
			audit(c, "probe-trim", back, (rc != 0) ? rc : -EIO);
			return 0;
		}
	}

	if (!req->probe_loader) {
		return 0;
	}
	if (!c->cfg.loader_verified) {
		/*
		 * No FE-5680A loader protocol is documented anywhere this project
		 * can reach, so there is nothing to probe *with*. Refusing to
		 * guess is the whole point: a wrong opcode aimed at a running
		 * rubidium is not a recoverable mistake.
		 */
		audit(c, "probe-loader-unverified", 0, -ENOTSUP);
		return 0;
	}

	/*
	 * Reachable only once someone sets loader_verified, which means they have
	 * a real protocol for a real variant. Until then this deliberately does
	 * not classify anything as updatable.
	 */
	audit(c, "probe-loader", 0, -ENOTSUP);
	return 0;
}

uint8_t rb_capability(const rb_ctx_t *c)
{
	return (c != NULL) ? c->cap : (uint8_t)RB_CAP_UNKNOWN;
}

int rb_fwupd_status(const rb_ctx_t *c, rb_status_t *out)
{
	if ((c == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	*out = c->status;
	return 0;
}

/* Append a NUL-terminated string, tracking the write position. */
static void app_str(char *out, size_t cap, size_t *n, const char *s)
{
	size_t k = strlen(s);

	if (*n >= cap) {
		return;
	}
	if (k > ((cap - 1U) - *n)) {
		k = (cap - 1U) - *n;
	}
	(void)memcpy(&out[*n], s, k);
	*n += k;
	out[*n] = '\0';
}

static void app_i32(char *out, size_t cap, size_t *n, int32_t v)
{
	char tmp[12];
	size_t i = sizeof(tmp);
	uint32_t mag;

	if (v < 0) {
		/* -INT32_MIN is not representable; take the magnitude unsigned. */
		mag = (uint32_t)(-(v + 1)) + 1U;
	} else {
		mag = (uint32_t)v;
	}
	if (mag == 0U) {
		tmp[--i] = '0';
	}
	while (mag != 0U) {
		tmp[--i] = (char)('0' + (mag % 10U));
		mag /= 10U;
	}
	if (v < 0) {
		tmp[--i] = '-';
	}
	while (i < sizeof(tmp)) {
		if (*n >= (cap - 1U)) {
			return;
		}
		out[*n] = tmp[i];
		(*n)++;
		i++;
	}
	out[*n] = '\0';
}

int rb_fwupd_identify(rb_ctx_t *c, char *out, size_t cap)
{
	size_t n = 0U;

	if ((c == NULL) || (out == NULL) || (cap == 0U)) {
		return -EINVAL;
	}
	out[0] = '\0';

	if (c->cap == (uint8_t)RB_CAP_UNKNOWN) {
		rb_probe_req_t req;

		(void)memset(&req, 0, sizeof(req));
		(void)rb_fwupd_probe(c, &req);
	}

	app_str(out, cap, &n, "FE-5680A ");
	app_str(out, cap, &n, rb_cap_name(c->cap));
	if (c->status.offset_valid) {
		app_str(out, cap, &n, " offset=");
		app_i32(out, cap, &n, c->status.offset);
	}
	return 0;
}

int rb_fwupd_trim(rb_ctx_t *c, int32_t new_offset, bool persist, uint64_t now_s)
{
	uint8_t data[RB_OFFSET_DATA_LEN];
	rb_frame_t reply;
	int32_t cur = 0;
	int32_t back = 0;
	int64_t step;
	int rc;

	if (c == NULL) {
		return -EINVAL;
	}
	if (!rail_ok(c)) {
		c->trims_refused++;
		audit(c, "trim-rail-down", new_offset, -ENODEV);
		return -ENODEV;
	}
	if ((c->cap != (uint8_t)RB_CAP_EFC_TRIMMABLE) &&
	    (c->cap != (uint8_t)RB_CAP_FW_UPDATABLE)) {
		c->trims_refused++;
		audit(c, "trim-not-trimmable", new_offset, -ENOTSUP);
		return -ENOTSUP;
	}
	if ((new_offset < c->cfg.trim.min_offset) ||
	    (new_offset > c->cfg.trim.max_offset)) {
		c->trims_refused++;
		audit(c, "trim-out-of-bounds", new_offset, -ERANGE);
		return -ERANGE;
	}
	if (persist && !c->cfg.trim.allow_eeprom) {
		c->trims_refused++;
		audit(c, "trim-eeprom-not-allowed", new_offset, -EACCES);
		return -EACCES;
	}
	if (persist && c->eeprom_written) {
		uint64_t since = (now_s > c->last_eeprom_s)
					 ? (now_s - c->last_eeprom_s) : 0U;

		if (since < (uint64_t)c->cfg.trim.eeprom_min_interval_s) {
			c->trims_refused++;
			audit(c, "trim-eeprom-rate-limited", new_offset, -EAGAIN);
			return -EAGAIN;
		}
	}

	/* Bound the *change*, so a decimal-point slip cannot move the reference. */
	rc = rb_read_offset(c, &cur);
	if (rc != 0) {
		c->trims_refused++;
		audit(c, "trim-read-current", new_offset, rc);
		return rc;
	}
	step = (int64_t)new_offset - (int64_t)cur;
	if (step < 0) {
		step = -step;
	}
	if (step > (int64_t)c->cfg.trim.max_step) {
		c->trims_refused++;
		audit(c, "trim-step-too-large", new_offset, -ERANGE);
		return -ERANGE;
	}

	(void)rb_offset_encode(new_offset, data);
	rc = exchange(c, persist ? (uint8_t)RB_CMD_SET_OFFSET_SAVE
				 : (uint8_t)RB_CMD_SET_OFFSET_VOL,
		      data, sizeof(data), &reply);
	if (rc != 0) {
		c->trims_refused++;
		audit(c, "trim-write", new_offset, rc);
		return rc;
	}

	/* Readback verification: a write that was acknowledged but not applied
	 * is the failure mode these surplus units actually exhibit. */
	rc = rb_read_offset(c, &back);
	if (rc != 0) {
		c->trims_refused++;
		audit(c, "trim-readback", new_offset, rc);
		return rc;
	}
	if (back != new_offset) {
		c->trims_refused++;
		audit(c, "trim-readback-mismatch", back, -EIO);
		return -EIO;
	}

	c->last_offset_written = new_offset;
	if (persist) {
		c->last_eeprom_s = now_s;
		c->eeprom_written = true;
	}
	c->trims_ok++;
	audit(c, persist ? "trim-saved" : "trim-volatile", new_offset, 0);
	return 0;
}

int rb_fwupd_begin(rb_ctx_t *c, uint32_t image_size)
{
	(void)image_size;

	if (c == NULL) {
		return -EINVAL;
	}
	/*
	 * spec §8.6: "Rb (FE-5680A) firmware is vendor-flashed over UART7 only if
	 * the module supports it (guarded, rare)." For every variant this project
	 * has documentation for, it does not — and no loader protocol is
	 * documented, so there is nothing to attempt. Reporting NOT_SUPPORTED is
	 * the required behaviour, not a shortfall.
	 */
	if (!c->cfg.loader_verified ||
	    (c->cap != (uint8_t)RB_CAP_FW_UPDATABLE)) {
		audit(c, "fwupd-not-supported", 0, -ENOTSUP);
		return -ENOTSUP;
	}
	audit(c, "fwupd-begin", 0, -ENOTSUP);
	return -ENOTSUP;
}

int rb_fwupd_feed(rb_ctx_t *c, const uint8_t *data, size_t len)
{
	int done = 0;
	size_t i;

	if ((c == NULL) || (data == NULL)) {
		return -EINVAL;
	}
	for (i = 0U; i < len; i++) {
		rb_frame_t f;

		if (rb_parse_byte(&c->parser, data[i], &f) != 1) {
			continue;
		}
		c->rx_frames++;
		done++;
		if (f.id == (uint8_t)RB_CMD_REQ_OFFSET) {
			if (f.data_len == (uint8_t)RB_OFFSET_DATA_LEN) {
				int32_t v = 0;

				(void)rb_offset_decode(f.data, &v);
				c->status.offset = v;
				c->status.offset_valid = true;
			}
		} else {
			c->status.unknown_frames++;
			c->status.last_unknown = f;
		}
	}
	return done;
}
