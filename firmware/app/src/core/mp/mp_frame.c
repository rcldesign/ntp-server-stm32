/*
 * STS1000 "Meridian" — core/mp: Maintenance Protocol frame layer.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See mp_frame.h for the wire format and the two encoding decisions.
 */

#include "mp/mp_frame.h"

#include <errno.h>
#include <string.h>

#include "util/crc.h"

/* ------------------------------------------------------------------ names */

static const char *const ch_names[] = {
	"control", "telemetry", "nmea",  "ubx",   "log",   "pps",
	"smp",     "gnss-pass", "rb-pass", "event", "mirror",
};

const char *mp_channel_name(uint8_t ch)
{
	if ((size_t)ch < (sizeof(ch_names) / sizeof(ch_names[0]))) {
		return ch_names[ch];
	}
	if (MP_CH_VALID(ch)) {
		return "reserved";
	}
	return "invalid";
}

/* --------------------------------------------------------------- transmit */

int mp_frame_encode(mp_frame_tx_t *tx, uint8_t ch, uint8_t flags,
		    const uint8_t *payload, size_t len, const uint8_t **out,
		    size_t *out_len)
{
	size_t raw_len;
	size_t enc_len = 0U;
	uint16_t crc;
	int rc;

	if ((tx == NULL) || (out == NULL) || (out_len == NULL)) {
		return -EINVAL;
	}
	if ((payload == NULL) && (len != 0U)) {
		return -EINVAL;
	}
	if (!MP_CH_VALID(ch)) {
		return -EINVAL;
	}
	if ((flags & (uint8_t)~MP_FLAG_KNOWN) != 0U) {
		return -EINVAL;
	}
	if (len > MP_TX_PAYLOAD_MAX) {
		return -EMSGSIZE;
	}

	tx->raw[0] = ch;
	tx->raw[1] = flags;
	if (len != 0U) {
		(void)memcpy(&tx->raw[2], payload, len);
	}
	raw_len = 2U + len;

	crc = sts_crc16_ccitt(tx->raw, raw_len);
#if MP_FRAME_CRC_BE
	tx->raw[raw_len] = (uint8_t)(crc >> 8);
	tx->raw[raw_len + 1U] = (uint8_t)(crc & 0xFFU);
#else
	tx->raw[raw_len] = (uint8_t)(crc & 0xFFU);
	tx->raw[raw_len + 1U] = (uint8_t)(crc >> 8);
#endif
	raw_len += 2U;

	rc = cobs_encode(tx->raw, raw_len, tx->enc, sizeof(tx->enc) - 1U,
			 &enc_len);
	if (rc != 0) {
		return rc;
	}

	tx->enc[enc_len] = 0x00U; /* frame delimiter */
	*out = tx->enc;
	*out_len = enc_len + 1U;
	return 0;
}

int mp_frame_send(mp_frame_tx_t *tx, uint8_t ch, const uint8_t *msg, size_t len,
		  mp_frame_sink_fn sink, void *user)
{
	size_t off = 0U;

	if ((tx == NULL) || (sink == NULL)) {
		return -EINVAL;
	}
	if ((msg == NULL) && (len != 0U)) {
		return -EINVAL;
	}
	if (!MP_CH_VALID(ch)) {
		return -EINVAL;
	}

	do {
		size_t chunk = len - off;
		uint8_t flags = 0U;
		const uint8_t *wire = NULL;
		size_t wire_len = 0U;
		int rc;

		if (chunk > MP_TX_PAYLOAD_MAX) {
			chunk = MP_TX_PAYLOAD_MAX;
			flags = MP_FLAG_MORE;
		}

		rc = mp_frame_encode(tx, ch, flags,
				     (chunk != 0U) ? &msg[off] : NULL, chunk,
				     &wire, &wire_len);
		if (rc != 0) {
			return rc;
		}

		rc = sink(user, wire, wire_len);
		if (rc < 0) {
			return rc;
		}

		off += chunk;
	} while (off < len);

	return 0;
}

/* ---------------------------------------------------------------- receive */

int mp_frame_rx_init(mp_frame_rx_t *rx, mp_reasm_t *reasm, uint8_t reasm_n,
		     mp_frame_msg_fn on_msg, void *user)
{
	uint8_t i;

	if (rx == NULL) {
		return -EINVAL;
	}
	if ((reasm == NULL) && (reasm_n != 0U)) {
		return -EINVAL;
	}
	for (i = 0U; i < reasm_n; i++) {
		if ((reasm[i].buf == NULL) && (reasm[i].cap != 0U)) {
			return -EINVAL;
		}
	}

	(void)memset(rx, 0, sizeof(*rx));
	rx->reasm = reasm;
	rx->reasm_n = reasm_n;
	rx->on_msg = on_msg;
	rx->user = user;

	for (i = 0U; i < reasm_n; i++) {
		reasm[i].len = 0U;
		reasm[i].ch = 0U;
		reasm[i].busy = false;
	}

	return cobs_dec_init(&rx->dec, rx->frame, sizeof(rx->frame));
}

void mp_frame_rx_reset(mp_frame_rx_t *rx)
{
	uint8_t i;

	if (rx == NULL) {
		return;
	}
	cobs_dec_reset(&rx->dec);
	for (i = 0U; i < rx->reasm_n; i++) {
		rx->reasm[i].len = 0U;
		rx->reasm[i].busy = false;
	}
}

/** Slot currently reassembling @p ch, or NULL. */
static mp_reasm_t *reasm_find(mp_frame_rx_t *rx, uint8_t ch)
{
	uint8_t i;

	for (i = 0U; i < rx->reasm_n; i++) {
		if (rx->reasm[i].busy && (rx->reasm[i].ch == ch)) {
			return &rx->reasm[i];
		}
	}
	return NULL;
}

/** A free slot, claimed for @p ch, or NULL when the pool is exhausted. */
static mp_reasm_t *reasm_claim(mp_frame_rx_t *rx, uint8_t ch)
{
	uint8_t i;

	for (i = 0U; i < rx->reasm_n; i++) {
		if (!rx->reasm[i].busy) {
			rx->reasm[i].busy = true;
			rx->reasm[i].ch = ch;
			rx->reasm[i].len = 0U;
			return &rx->reasm[i];
		}
	}
	return NULL;
}

static void reasm_release(mp_reasm_t *slot)
{
	slot->busy = false;
	slot->len = 0U;
}

/** Deliver one complete logical message; returns 1 when the sink took it. */
static int deliver(mp_frame_rx_t *rx, uint8_t ch, const uint8_t *msg,
		   size_t len)
{
	rx->msgs++;
	if (rx->on_msg == NULL) {
		return 1;
	}
	if (rx->on_msg(rx->user, ch, msg, len) < 0) {
		rx->sink_errors++;
	}
	return 1;
}

/**
 * Handle one CRC-validated frame body.
 *
 * @p body is `channel | flags | payload`, i.e. the decoded frame with the CRC
 * trailer already removed and verified.
 */
static int handle_frame(mp_frame_rx_t *rx, const uint8_t *body, size_t body_len)
{
	uint8_t ch = body[0];
	uint8_t flags = body[1];
	const uint8_t *payload = &body[2];
	size_t plen = body_len - 2U;
	mp_reasm_t *slot;

	if (!MP_CH_VALID(ch)) {
		rx->bad_channel++;
		return 0;
	}
	if ((flags & (uint8_t)~MP_FLAG_KNOWN) != 0U) {
		/*
		 * An unknown flag changes the meaning of the frame, so the frame
		 * cannot be interpreted. Dropping it (rather than masking the
		 * bit off) is the only safe reading of a versioned flag field.
		 */
		rx->bad_flags++;
		return 0;
	}

	rx->frames++;

	slot = reasm_find(rx, ch);

	if ((flags & MP_FLAG_MORE) != 0U) {
		if (slot == NULL) {
			slot = reasm_claim(rx, ch);
			if (slot == NULL) {
				rx->reasm_drops++;
				return 0;
			}
		}
		if ((slot->len + plen) > slot->cap) {
			/* Over-long message: abandon the whole thing, not just
			 * the overflowing fragment, or the consumer would be
			 * handed a truncated message it cannot detect. */
			rx->reasm_drops++;
			reasm_release(slot);
			return 0;
		}
		if (plen != 0U) {
			(void)memcpy(&slot->buf[slot->len], payload, plen);
		}
		slot->len += plen;
		return 0;
	}

	if (slot == NULL) {
		/* Unfragmented: the common case, delivered without a copy. */
		return deliver(rx, ch, payload, plen);
	}

	if ((slot->len + plen) > slot->cap) {
		rx->reasm_drops++;
		reasm_release(slot);
		return 0;
	}
	if (plen != 0U) {
		(void)memcpy(&slot->buf[slot->len], payload, plen);
	}
	slot->len += plen;

	{
		size_t total = slot->len;
		const uint8_t *msg = slot->buf;

		reasm_release(slot);
		return deliver(rx, ch, msg, total);
	}
}

int mp_frame_rx_byte(mp_frame_rx_t *rx, uint8_t b)
{
	const uint8_t *frame;
	size_t len;
	uint16_t got;
	uint16_t want;
	int rc;

	if (rx == NULL) {
		return -EINVAL;
	}

	rc = cobs_dec_byte(&rx->dec, b);
	if (rc == 0) {
		return 0;
	}
	if (rc < 0) {
		/* Malformed or oversized COBS body; the decoder has already
		 * resynchronised on the delimiter. */
		rx->cobs_errors++;
		return 0;
	}

	frame = cobs_dec_buf(&rx->dec);
	len = cobs_dec_len(&rx->dec);

	if (len < MP_FRAME_OVERHEAD) {
		rx->short_frames++;
		return 0;
	}

#if MP_FRAME_CRC_BE
	got = (uint16_t)(((uint16_t)frame[len - 2U] << 8) |
			 (uint16_t)frame[len - 1U]);
#else
	got = (uint16_t)(((uint16_t)frame[len - 1U] << 8) |
			 (uint16_t)frame[len - 2U]);
#endif
	want = sts_crc16_ccitt(frame, len - 2U);
	if (got != want) {
		rx->crc_errors++; /* §3.1: dropped silently */
		return 0;
	}

	return handle_frame(rx, frame, len - 2U);
}

int mp_frame_rx_input(mp_frame_rx_t *rx, const uint8_t *data, size_t len)
{
	size_t i;
	int msgs = 0;

	if (rx == NULL) {
		return -EINVAL;
	}
	if ((data == NULL) && (len != 0U)) {
		return -EINVAL;
	}

	for (i = 0U; i < len; i++) {
		if (mp_frame_rx_byte(rx, data[i]) == 1) {
			msgs++;
		}
	}
	return msgs;
}
