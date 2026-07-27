/*
 * STS1000 "Meridian" — core/web: RFC 6455 WebSocket server (see wss.h).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 */

#include "web/wss.h"

#include <errno.h>
#include <string.h>

/* ========================================================================= */
/* SHA-1                                                                     */
/* ========================================================================= */
/*
 * Written for this repository from the FIPS 180-4 pseudocode (§6.1.2) and
 * placed under the same Apache-2.0 licence as the rest of the tree. No third
 * party code is incorporated.
 *
 * WHY SHA-1 IS HERE AT ALL. port_crypto.h deliberately offers no SHA-1: it is
 * broken for signatures and nothing in the timing or security path may use it.
 * RFC 6455 §4.2.2 nevertheless *defines* Sec-WebSocket-Accept as
 * base64(SHA-1(key || GUID)), and that value is not a security mechanism — it
 * proves only that the peer read the WebSocket spec, which is why the RFC
 * itself notes it is "not used to provide security". Adding SHA-1 to
 * port_crypto would put a broken primitive within reach of every core module;
 * keeping it ~60 lines local to the one caller that the RFC forces to use it
 * keeps that door shut.
 *
 * It is a plain reference implementation: correct, small, not constant-time
 * (it does not need to be — the input is a public nonce and a public constant).
 */

#define SHA1_BLOCK 64U

typedef struct {
	uint32_t h[5];
	uint64_t bits;
	uint8_t  buf[SHA1_BLOCK];
	size_t   n;
} sha1_ctx_t;

static uint32_t rotl32(uint32_t v, unsigned int s)
{
	return (uint32_t)((v << s) | (v >> (32U - s)));
}

static void sha1_block(sha1_ctx_t *c, const uint8_t *p)
{
	uint32_t w[80];
	uint32_t a;
	uint32_t b;
	uint32_t d;
	uint32_t e;
	uint32_t f;
	unsigned int i;

	for (i = 0U; i < 16U; i++) {
		w[i] = ((uint32_t)p[i * 4U] << 24) |
		       ((uint32_t)p[i * 4U + 1U] << 16) |
		       ((uint32_t)p[i * 4U + 2U] << 8) |
		       (uint32_t)p[i * 4U + 3U];
	}
	for (i = 16U; i < 80U; i++) {
		w[i] = rotl32(w[i - 3U] ^ w[i - 8U] ^ w[i - 14U] ^ w[i - 16U], 1U);
	}

	a = c->h[0];
	b = c->h[1];
	f = c->h[2]; /* 'c' in the spec; renamed, the context owns that name */
	d = c->h[3];
	e = c->h[4];

	for (i = 0U; i < 80U; i++) {
		uint32_t k;
		uint32_t t;

		if (i < 20U) {
			t = (b & f) | ((~b) & d);
			k = 0x5A827999U;
		} else if (i < 40U) {
			t = b ^ f ^ d;
			k = 0x6ED9EBA1U;
		} else if (i < 60U) {
			t = (b & f) | (b & d) | (f & d);
			k = 0x8F1BBCDCU;
		} else {
			t = b ^ f ^ d;
			k = 0xCA62C1D6U;
		}
		t = rotl32(a, 5U) + t + e + k + w[i];
		e = d;
		d = f;
		f = rotl32(b, 30U);
		b = a;
		a = t;
	}

	c->h[0] += a;
	c->h[1] += b;
	c->h[2] += f;
	c->h[3] += d;
	c->h[4] += e;
}

static void sha1_init(sha1_ctx_t *c)
{
	c->h[0] = 0x67452301U;
	c->h[1] = 0xEFCDAB89U;
	c->h[2] = 0x98BADCFEU;
	c->h[3] = 0x10325476U;
	c->h[4] = 0xC3D2E1F0U;
	c->bits = 0U;
	c->n = 0U;
}

static void sha1_update(sha1_ctx_t *c, const uint8_t *p, size_t n)
{
	c->bits += (uint64_t)n * 8U;
	while (n > 0U) {
		size_t take = SHA1_BLOCK - c->n;

		if (take > n) {
			take = n;
		}
		memcpy(&c->buf[c->n], p, take);
		c->n += take;
		p += take;
		n -= take;
		if (c->n == SHA1_BLOCK) {
			sha1_block(c, c->buf);
			c->n = 0U;
		}
	}
}

static void sha1_final(sha1_ctx_t *c, uint8_t out[20])
{
	uint8_t pad = 0x80U;
	uint8_t lenb[8];
	unsigned int i;

	for (i = 0U; i < 8U; i++) {
		lenb[i] = (uint8_t)((c->bits >> (56U - (8U * i))) & 0xFFU);
	}
	sha1_update(c, &pad, 1U);
	/* sha1_update() has already counted the pad byte's bits; the length we
	 * captured above is the pre-pad value, which is what the spec wants. */
	while (c->n != 56U) {
		uint8_t z = 0U;

		sha1_update(c, &z, 1U);
	}
	memcpy(&c->buf[56], lenb, 8U);
	sha1_block(c, c->buf);
	c->n = 0U;

	for (i = 0U; i < 5U; i++) {
		out[i * 4U] = (uint8_t)((c->h[i] >> 24) & 0xFFU);
		out[i * 4U + 1U] = (uint8_t)((c->h[i] >> 16) & 0xFFU);
		out[i * 4U + 2U] = (uint8_t)((c->h[i] >> 8) & 0xFFU);
		out[i * 4U + 3U] = (uint8_t)(c->h[i] & 0xFFU);
	}
}

void wss_sha1(const uint8_t *in, size_t n, uint8_t out[20])
{
	sha1_ctx_t c;

	if (out == NULL) {
		return;
	}
	sha1_init(&c);
	if (in != NULL && n != 0U) {
		sha1_update(&c, in, n);
	}
	sha1_final(&c, out);
}

/* ========================================================================= */
/* handshake                                                                 */
/* ========================================================================= */

int wss_accept_key(const char *key, size_t key_len, char *out, size_t cap)
{
	uint8_t buf[WSS_KEY_MAX + sizeof(WSS_GUID)];
	uint8_t digest[20];
	size_t glen = sizeof(WSS_GUID) - 1U;

	if (key == NULL || out == NULL) {
		return -EINVAL;
	}
	if (key_len == 0U || key_len > WSS_KEY_MAX) {
		return -EINVAL;
	}
	if (cap < (WSS_ACCEPT_LEN + 1U)) {
		return -ENOSPC;
	}
	memcpy(buf, key, key_len);
	memcpy(&buf[key_len], WSS_GUID, glen);
	wss_sha1(buf, key_len + glen, digest);
	if (web_b64_encode(digest, sizeof(digest), out, cap) != WSS_ACCEPT_LEN) {
		return -ENOSPC;
	}
	return 0;
}

/* ========================================================================= */
/* UTF-8 validation (RFC 3629)                                               */
/* ========================================================================= */

bool wss_utf8_valid(const uint8_t *p, size_t n)
{
	size_t i = 0U;

	if (p == NULL) {
		return n == 0U;
	}
	while (i < n) {
		uint8_t c = p[i];
		size_t need;
		uint32_t cp;
		uint32_t min;
		size_t k;

		if (c < 0x80U) {
			i++;
			continue;
		}
		if ((c & 0xE0U) == 0xC0U) {
			need = 1U;
			cp = (uint32_t)(c & 0x1FU);
			min = 0x80U;
		} else if ((c & 0xF0U) == 0xE0U) {
			need = 2U;
			cp = (uint32_t)(c & 0x0FU);
			min = 0x800U;
		} else if ((c & 0xF8U) == 0xF0U) {
			need = 3U;
			cp = (uint32_t)(c & 0x07U);
			min = 0x10000U;
		} else {
			return false; /* continuation byte or 5/6-byte form */
		}
		if ((n - i - 1U) < need) {
			return false;
		}
		for (k = 1U; k <= need; k++) {
			uint8_t cc = p[i + k];

			if ((cc & 0xC0U) != 0x80U) {
				return false;
			}
			cp = (cp << 6) | (uint32_t)(cc & 0x3FU);
		}
		if (cp < min) {
			return false; /* overlong encoding */
		}
		if (cp > 0x10FFFFU) {
			return false;
		}
		if (cp >= 0xD800U && cp <= 0xDFFFU) {
			return false; /* UTF-16 surrogate half */
		}
		i += need + 1U;
	}
	return true;
}

/* ========================================================================= */
/* frame codec                                                               */
/* ========================================================================= */

static bool op_is_control(uint8_t op)
{
	return (op & 0x08U) != 0U;
}

static bool op_known(uint8_t op)
{
	switch (op) {
	case WSS_OP_CONT:
	case WSS_OP_TEXT:
	case WSS_OP_BIN:
	case WSS_OP_CLOSE:
	case WSS_OP_PING:
	case WSS_OP_PONG:
		return true;
	default:
		return false;
	}
}

int wss_hdr_parse(const uint8_t *buf, size_t n, wss_hdr_t *out)
{
	uint8_t b0;
	uint8_t b1;
	uint8_t len7;
	size_t need = 2U;

	if (buf == NULL || out == NULL) {
		return -EINVAL;
	}
	if (n < 2U) {
		return -EAGAIN;
	}
	b0 = buf[0];
	b1 = buf[1];

	memset(out, 0, sizeof(*out));
	out->fin = (b0 & 0x80U) != 0U;
	if ((b0 & 0x70U) != 0U) {
		/* RSV1..3 with no negotiated extension. */
		return -EPROTO;
	}
	out->op = (uint8_t)(b0 & 0x0FU);
	if (!op_known(out->op)) {
		return -EPROTO;
	}
	out->masked = (b1 & 0x80U) != 0U;
	len7 = (uint8_t)(b1 & 0x7FU);

	if (len7 == 126U) {
		need += 2U;
		if (n < need) {
			return -EAGAIN;
		}
		out->len = ((uint64_t)buf[2] << 8) | (uint64_t)buf[3];
		if (out->len < 126U) {
			/* Not the shortest form: two encodings of one length is
			 * exactly the ambiguity a framing parser must refuse. */
			return -EPROTO;
		}
	} else if (len7 == 127U) {
		unsigned int i;

		need += 8U;
		if (n < need) {
			return -EAGAIN;
		}
		out->len = 0U;
		for (i = 0U; i < 8U; i++) {
			out->len = (out->len << 8) | (uint64_t)buf[2U + i];
		}
		if ((buf[2] & 0x80U) != 0U) {
			/* RFC 6455 §5.2: the MSB of a 64-bit length must be 0. */
			return -EMSGSIZE;
		}
		if (out->len <= 0xFFFFU) {
			return -EPROTO; /* not the shortest form */
		}
	} else {
		out->len = (uint64_t)len7;
	}

	if (out->masked) {
		need += 4U;
		if (n < need) {
			return -EAGAIN;
		}
		memcpy(out->mask, &buf[need - 4U], 4U);
	}

	if (op_is_control(out->op)) {
		if (!out->fin) {
			return -EPROTO; /* control frames are never fragmented */
		}
		if (out->len > WSS_CONTROL_MAX) {
			return -EPROTO;
		}
	}

	out->hdr_len = (uint8_t)need;
	return 0;
}

void wss_unmask(uint8_t *p, size_t n, const uint8_t mask[4], uint64_t offset)
{
	size_t i;

	if (p == NULL || mask == NULL) {
		return;
	}
	for (i = 0U; i < n; i++) {
		p[i] ^= mask[(size_t)((offset + (uint64_t)i) & 3U)];
	}
}

int wss_encode_header(uint8_t op, bool fin, size_t len, uint8_t *out, size_t cap)
{
	size_t hdr;
	size_t o = 0U;

	if (out == NULL) {
		return -EINVAL;
	}
	if (!op_known(op)) {
		return -EINVAL;
	}
	if (op_is_control(op) && (len > WSS_CONTROL_MAX || !fin)) {
		return -EINVAL;
	}

	if (len < 126U) {
		hdr = 2U;
	} else if (len <= 0xFFFFU) {
		hdr = 4U;
	} else {
		hdr = 10U;
	}
	if (hdr > cap) {
		return -ENOSPC;
	}

	out[o++] = (uint8_t)((fin ? 0x80U : 0x00U) | (op & 0x0FU));
	if (hdr == 2U) {
		out[o++] = (uint8_t)len;
	} else if (hdr == 4U) {
		out[o++] = 126U;
		out[o++] = (uint8_t)((len >> 8) & 0xFFU);
		out[o++] = (uint8_t)(len & 0xFFU);
	} else {
		unsigned int i;

		out[o++] = 127U;
		for (i = 0U; i < 8U; i++) {
			out[o++] = (uint8_t)(((uint64_t)len >>
					      (56U - (8U * i))) & 0xFFU);
		}
	}
	return (int)o;
}

int wss_encode(uint8_t op, bool fin, const uint8_t *payload, size_t len,
	       uint8_t *out, size_t cap)
{
	int hdr;

	if (payload == NULL && len != 0U) {
		return -EINVAL;
	}
	hdr = wss_encode_header(op, fin, len, out, cap);
	if (hdr < 0) {
		return hdr;
	}
	if (((size_t)hdr + len) > cap) {
		return -ENOSPC;
	}
	if (len != 0U) {
		memcpy(&out[hdr], payload, len);
	}
	return hdr + (int)len;
}

/* True when @p code may appear in a Close frame on the wire. */
static bool close_code_sendable(uint16_t code)
{
	if (code >= 3000U && code <= 4999U) {
		return true; /* registered / private use */
	}
	switch (code) {
	case WSS_CLOSE_NORMAL:
	case WSS_CLOSE_GOING_AWAY:
	case WSS_CLOSE_PROTOCOL_ERROR:
	case WSS_CLOSE_UNSUPPORTED:
	case WSS_CLOSE_INVALID_DATA:
	case WSS_CLOSE_POLICY:
	case WSS_CLOSE_TOO_BIG:
	case 1010U:
	case WSS_CLOSE_INTERNAL:
		return true;
	default:
		/* 1004, 1005, 1006, 1015 and everything below 1000 are
		 * reserved and must never be sent (RFC 6455 §7.4.1). */
		return false;
	}
}

int wss_encode_close(uint16_t code, const char *reason, uint8_t *out, size_t cap)
{
	uint8_t body[WSS_CONTROL_MAX];
	size_t n = 0U;

	if (out == NULL) {
		return -EINVAL;
	}
	if (code != 0U) {
		size_t rlen = (reason == NULL) ? 0U : strlen(reason);

		if (!close_code_sendable(code)) {
			return -EINVAL;
		}
		if (rlen > (WSS_CONTROL_MAX - 2U)) {
			rlen = WSS_CONTROL_MAX - 2U;
		}
		body[0] = (uint8_t)((code >> 8) & 0xFFU);
		body[1] = (uint8_t)(code & 0xFFU);
		if (rlen != 0U) {
			memcpy(&body[2], reason, rlen);
		}
		n = 2U + rlen;
	}
	return wss_encode((uint8_t)WSS_OP_CLOSE, true, body, n, out, cap);
}

/* ========================================================================= */
/* reassembly                                                                */
/* ========================================================================= */

int wss_rx_init(wss_rx_t *r, uint8_t *buf, size_t cap)
{
	if (r == NULL || buf == NULL || cap == 0U) {
		return -EINVAL;
	}
	memset(r, 0, sizeof(*r));
	r->buf = buf;
	r->cap = cap;
	return 0;
}

/* Validate a received Close payload (RFC 6455 §7.4.1 / §8.1). */
static bool close_payload_ok(const uint8_t *p, size_t n)
{
	uint16_t code;

	if (n == 0U) {
		return true;
	}
	if (n == 1U) {
		return false; /* a truncated status code */
	}
	code = (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
	if (!close_code_sendable(code)) {
		return false;
	}
	return wss_utf8_valid(&p[2], n - 2U);
}

int wss_rx_feed(wss_rx_t *r, uint8_t *in, size_t n, size_t *consumed,
		uint8_t *out_op, const uint8_t **out_p, size_t *out_len,
		uint16_t *close_code)
{
	size_t i = 0U;
	uint16_t code = WSS_CLOSE_PROTOCOL_ERROR;
	int rc = 0;

	if (r == NULL || r->buf == NULL || out_op == NULL || out_p == NULL ||
	    out_len == NULL || (in == NULL && n != 0U)) {
		return -EINVAL;
	}
	if (consumed != NULL) {
		*consumed = 0U;
	}

	while (i < n) {
		wss_hdr_t h;
		uint8_t *payload;
		int hrc = wss_hdr_parse(&in[i], n - i, &h);

		if (hrc == -EAGAIN) {
			break;
		}
		if (hrc == -EMSGSIZE) {
			r->protocol_errors++;
			code = WSS_CLOSE_TOO_BIG;
			rc = -EPROTO;
			goto fail;
		}
		if (hrc != 0) {
			r->protocol_errors++;
			rc = -EPROTO;
			goto fail;
		}
		if (!h.masked) {
			/* RFC 6455 §5.1: every client frame must be masked. */
			r->protocol_errors++;
			rc = -EPROTO;
			goto fail;
		}
		if (h.len > (uint64_t)(n - i - h.hdr_len)) {
			break; /* payload not fully arrived */
		}

		/* Unmask in place; see the wss_rx_feed() contract. */
		payload = &in[i + h.hdr_len];
		wss_unmask(payload, (size_t)h.len, h.mask, 0U);
		r->frames++;

		if (op_is_control(h.op)) {
			i += (size_t)h.hdr_len + (size_t)h.len;
			if (h.op == (uint8_t)WSS_OP_PING) {
				r->pings++;
			} else if (h.op == (uint8_t)WSS_OP_PONG) {
				r->pongs++;
			} else if (!close_payload_ok(payload, (size_t)h.len)) {
				r->protocol_errors++;
				code = WSS_CLOSE_PROTOCOL_ERROR;
				rc = -EPROTO;
				goto fail;
			}
			*out_op = h.op;
			*out_p = payload;
			*out_len = (size_t)h.len;
			if (consumed != NULL) {
				*consumed = i;
			}
			return 1;
		}

		/* Data frame: check the fragmentation state machine first. */
		if (h.op == (uint8_t)WSS_OP_CONT) {
			if (!r->in_frag) {
				r->protocol_errors++;
				rc = -EPROTO;
				goto fail;
			}
		} else {
			if (r->in_frag) {
				/* A new data frame while a message is open. */
				r->protocol_errors++;
				rc = -EPROTO;
				goto fail;
			}
			r->msg_op = h.op;
			r->len = 0U;
		}

		if (((uint64_t)r->len + h.len) > (uint64_t)r->cap) {
			r->oversize++;
			r->len = 0U;
			r->in_frag = false;
			r->msg_op = 0U;
			code = WSS_CLOSE_TOO_BIG;
			rc = -EMSGSIZE;
			i += (size_t)h.hdr_len + (size_t)h.len;
			goto fail;
		}
		if (h.len != 0U) {
			memcpy(&r->buf[r->len], payload, (size_t)h.len);
			r->len += (size_t)h.len;
		}
		i += (size_t)h.hdr_len + (size_t)h.len;

		if (!h.fin) {
			r->in_frag = true;
			continue;
		}

		r->in_frag = false;
		if (r->msg_op == (uint8_t)WSS_OP_TEXT &&
		    !wss_utf8_valid(r->buf, r->len)) {
			/* §8.1: a TEXT message must be valid UTF-8, checked on
			 * the reassembled whole because a code point may span
			 * two fragments. */
			r->protocol_errors++;
			r->len = 0U;
			r->msg_op = 0U;
			code = WSS_CLOSE_INVALID_DATA;
			rc = -EPROTO;
			goto fail;
		}
		r->messages++;
		*out_op = r->msg_op;
		*out_p = r->buf;
		*out_len = r->len;
		r->msg_op = 0U;
		if (consumed != NULL) {
			*consumed = i;
		}
		return 1;
	}

	if (consumed != NULL) {
		*consumed = i;
	}
	return 0;

fail:
	if (consumed != NULL) {
		*consumed = i;
	}
	if (close_code != NULL) {
		*close_code = code;
	}
	return rc;
}

/* ========================================================================= */
/* send queue                                                                */
/* ========================================================================= */

void wss_txq_init(wss_txq_t *q)
{
	if (q == NULL) {
		return;
	}
	memset(q, 0, sizeof(*q));
}

uint8_t wss_txq_count(const wss_txq_t *q)
{
	return (q == NULL) ? 0U : q->count;
}

int wss_txq_push(wss_txq_t *q, const uint8_t *frame, size_t n, bool urgent)
{
	uint8_t slot;
	int dropped = 0;

	if (q == NULL || frame == NULL || n == 0U) {
		return -EINVAL;
	}
	if (n > WSS_TXQ_SLOT_BYTES) {
		return -ENOSPC;
	}
	if (q->count == WSS_TXQ_SLOTS) {
		/* Drop the oldest: telemetry is re-sampled, so the newest record
		 * is strictly more useful than the one the peer never read. */
		q->head = (uint8_t)((q->head + 1U) % WSS_TXQ_SLOTS);
		q->count--;
		q->dropped++;
		dropped = 1;
	}
	if (urgent) {
		/* Push in front: a pong or close must not sit behind telemetry. */
		q->head = (uint8_t)((q->head + WSS_TXQ_SLOTS - 1U) % WSS_TXQ_SLOTS);
		slot = q->head;
	} else {
		slot = (uint8_t)((q->head + q->count) % WSS_TXQ_SLOTS);
	}
	memcpy(q->slot[slot], frame, n);
	q->slot_len[slot] = (uint16_t)n;
	q->count++;
	q->pushed++;
	return dropped;
}

int wss_txq_peek(const wss_txq_t *q, const uint8_t **p, size_t *n)
{
	if (q == NULL || p == NULL || n == NULL) {
		return -EINVAL;
	}
	if (q->count == 0U) {
		return -ENOENT;
	}
	*p = q->slot[q->head];
	*n = q->slot_len[q->head];
	return 0;
}

int wss_txq_pop(wss_txq_t *q)
{
	if (q == NULL) {
		return -EINVAL;
	}
	if (q->count == 0U) {
		return -ENOENT;
	}
	q->head = (uint8_t)((q->head + 1U) % WSS_TXQ_SLOTS);
	q->count--;
	q->sent++;
	return 0;
}

/* ========================================================================= */
/* subscriptions                                                             */
/* ========================================================================= */

static const struct {
	const char *name;
	uint8_t     bit;
} group_tbl[] = {
	{ "summary", WSS_GRP_SUMMARY }, { "timing", WSS_GRP_TIMING },
	{ "gnss", WSS_GRP_GNSS },       { "power", WSS_GRP_POWER },
	{ "net", WSS_GRP_NET },         { "ptp", WSS_GRP_PTP },
	{ "alarms", WSS_GRP_ALARMS },   { "logs", WSS_GRP_LOGS },
};

uint8_t wss_group_bit(const char *name, size_t n)
{
	size_t i;

	if (name == NULL || n == 0U) {
		return 0U;
	}
	for (i = 0U; i < (sizeof(group_tbl) / sizeof(group_tbl[0])); i++) {
		if (web_span_eq(name, n, group_tbl[i].name)) {
			return group_tbl[i].bit;
		}
	}
	return 0U;
}

const char *wss_group_name(uint8_t bit)
{
	size_t i;

	for (i = 0U; i < (sizeof(group_tbl) / sizeof(group_tbl[0])); i++) {
		if (group_tbl[i].bit == bit) {
			return group_tbl[i].name;
		}
	}
	return NULL;
}

void wss_sub_init(wss_sub_t *s)
{
	if (s == NULL) {
		return;
	}
	memset(s, 0, sizeof(*s));
	s->groups = (uint8_t)(WSS_GRP_SUMMARY | WSS_GRP_ALARMS);
	s->rate_hz = WSS_RATE_HZ_DEFAULT;
}

int wss_sub_apply(wss_sub_t *s, const char *json, size_t len)
{
	web_json_val_t op;
	web_json_val_t v;
	uint8_t mask = 0U;
	bool replace;

	if (s == NULL || json == NULL) {
		return -EINVAL;
	}
	if (web_json_obj_get(json, len, "op", &op) != 0 ||
	    op.type != (uint8_t)WEB_JSON_STR) {
		return -EBADMSG;
	}
	if (web_json_str_eq(&op, "subscribe")) {
		replace = true;
	} else if (web_json_str_eq(&op, "unsubscribe")) {
		replace = false;
	} else if (web_json_str_eq(&op, "ping")) {
		return 0;
	} else {
		return -EBADMSG;
	}

	if (web_json_obj_get(json, len, "groups", &v) == 0 &&
	    v.type == (uint8_t)WEB_JSON_ARR) {
		size_t cur = 0U;
		web_json_val_t el;

		while (web_json_arr_next(&v, &cur, &el) == 1) {
			if (el.type != (uint8_t)WEB_JSON_STR) {
				continue;
			}
			if (web_span_eq(el.p, el.n, "all")) {
				mask = WSS_GRP_ALL;
				continue;
			}
			mask |= wss_group_bit(el.p, el.n);
		}
	}

	if (replace) {
		if (mask != 0U) {
			s->groups = mask;
		}
	} else {
		s->groups &= (uint8_t)~mask;
	}

	if (web_json_obj_get(json, len, "rate", &v) == 0) {
		uint64_t r = 0U;

		if (web_json_u64(&v, &r) == 0) {
			if (r == 0U) {
				r = 1U;
			}
			if (r > WSS_RATE_HZ_MAX) {
				r = WSS_RATE_HZ_MAX;
			}
			s->rate_hz = (uint8_t)r;
		}
	}
	if (web_json_obj_get(json, len, "log_cursor", &v) == 0) {
		uint64_t cur = 0U;

		if (web_json_u64(&v, &cur) == 0 && cur <= 0xFFFFFFFFULL) {
			s->log_cursor = (uint32_t)cur;
		}
	}
	/* Re-arm immediately so a subscribe change is visible on the next tick
	 * rather than after the previous rate's interval. */
	s->next_ms = 0U;
	return 0;
}
