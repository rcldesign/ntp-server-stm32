/*
 * STS1000 "Meridian" — core/mp: Maintenance Protocol frame layer (FMT §3.1).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation; every buffer is caller-owned.
 *
 * Wire format, exactly as specified by the FMT spec §3.1:
 *
 *     COBS( channel:u8 | flags:u8 | payload[<=2048] | crc16-ccitt:u16 ) 0x00
 *
 * The COBS body carries no 0x00, so the trailing 0x00 delimits frames. CRC-16/
 * CCITT-FALSE (`sts_crc16_ccitt`) runs over `channel | flags | payload` and is
 * appended **big-endian** — see the note below. `flags` bit 0 is
 * more-fragments; a run of fragments on one channel is reassembled into a
 * single logical message before it reaches the consumer.
 *
 * Two decisions this file had to make, recorded because the frame layer is the
 * one part of MP that a host implementation cannot negotiate:
 *
 * 1. CRC byte order. The spec writes the trailer as `crc16-ccitt:u16` without
 *    stating an order. This implementation uses **big-endian** (MSB first),
 *    matching the network order of the JSON/CBOR payloads above it and the
 *    conventional presentation of CRC-16/CCITT-FALSE. `MP_FRAME_CRC_BE` is the
 *    single switch if that ever has to change.
 *
 * 2. Transmit payload ceiling. `COBS_MAX_FRAME` in `core/util/cobs.h` is 1052 B
 *    (sized for MCP) and `cobs_encode()` refuses anything longer with
 *    -EMSGSIZE. The *streaming* decoder has no such limit — it is bounded by
 *    the caller's buffer — so MP **receives** the full 2048 B spec payload.
 *    Transmit is capped at MP_TX_PAYLOAD_MAX (1024 B) so header+payload+CRC
 *    stays inside COBS_MAX_FRAME, and anything longer is fragmented with the
 *    more-fragments flag, which is a first-class part of the spec's own frame
 *    layer. Raising COBS_MAX_FRAME to 2052 would let TX use full-size frames;
 *    that is a build-system change outside this module.
 */

#ifndef STS1000_CORE_MP_MP_FRAME_H_
#define STS1000_CORE_MP_MP_FRAME_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "util/cobs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------- constants */

/** Largest payload a received frame may carry (FMT §3.1). */
#define MP_PAYLOAD_MAX 2048U

/** Bytes of frame overhead: channel, flags and the CRC-16 trailer. */
#define MP_FRAME_OVERHEAD 4U

/** Largest decoded receive frame, i.e. the streaming decoder's buffer size. */
#define MP_FRAME_MAX (MP_PAYLOAD_MAX + MP_FRAME_OVERHEAD)

/**
 * Largest payload this implementation *transmits* in one frame.
 *
 * Bounded by COBS_MAX_FRAME (see the file header): 1024 + 4 = 1028 <= 1052.
 * Longer logical messages are fragmented.
 */
#ifndef MP_TX_PAYLOAD_MAX
#define MP_TX_PAYLOAD_MAX 1024U
#endif

/** Largest decoded transmit frame. */
#define MP_TX_FRAME_MAX (MP_TX_PAYLOAD_MAX + MP_FRAME_OVERHEAD)

/** CRC trailer byte order: 1 = big-endian (as built), 0 = little-endian. */
#ifndef MP_FRAME_CRC_BE
#define MP_FRAME_CRC_BE 1
#endif

/** `flags` bit 0: more fragments follow on this channel. */
#define MP_FLAG_MORE 0x01U

/** Flag bits this implementation defines; the rest must be zero. */
#define MP_FLAG_KNOWN MP_FLAG_MORE

/* ---------------------------------------------------------------- channels */

/**
 * Channel allocation (FMT §3.1 table).
 *
 * 0x0A is taken from the spec's reserved range for the device mirror, which the
 * spec does not cover — see mp_mirror.h for why it is a channel of its own
 * rather than a telemetry record type.
 */
typedef enum {
	MP_CH_CONTROL = 0x00,   /**< JSON-RPC 2.0 control plane (§3.2) */
	MP_CH_TELEMETRY = 0x01, /**< CBOR telemetry records (§7.1) */
	MP_CH_NMEA = 0x02,      /**< raw NMEA tee */
	MP_CH_UBX = 0x03,       /**< raw UBX tee */
	MP_CH_LOG = 0x04,       /**< structured log records */
	MP_CH_PPS = 0x05,       /**< PPS / discipline records (§7.2) */
	MP_CH_SMP = 0x06,       /**< MCUmgr/SMP tunnel */
	MP_CH_GNSS_PASS = 0x07, /**< GNSS UART passthrough */
	MP_CH_RB_PASS = 0x08,   /**< rubidium UART passthrough */
	MP_CH_EVENT = 0x09,     /**< edge-triggered event / alarm stream */
	MP_CH_MIRROR = 0x0A,    /**< device front-panel mirror (this project) */
	MP_CH_MAX = 0x1F,       /**< highest channel the frame layer accepts */
} mp_channel_t;

/** Number of distinct channel ids (0x00..0x1F). */
#define MP_CH_COUNT 0x20U

/** True when @p ch is inside the protocol's channel space. */
#define MP_CH_VALID(ch) ((unsigned int)(ch) <= (unsigned int)MP_CH_MAX)

/** Short, stable channel name, e.g. "control", "pps". Never NULL. */
const char *mp_channel_name(uint8_t ch);

/* --------------------------------------------------------------- transmit */

/**
 * Transmit scratch. Caller-owned; one per transmit context.
 *
 * `raw` assembles header+payload+CRC, `enc` holds the COBS body plus the
 * delimiter. Both are sized from MP_TX_PAYLOAD_MAX so a maximum frame always
 * fits and mp_frame_encode() can never fail for lack of room.
 */
typedef struct {
	uint8_t raw[MP_TX_FRAME_MAX];
	uint8_t enc[COBS_ENCODE_MAX(MP_TX_FRAME_MAX) + 1U];
} mp_frame_tx_t;

/**
 * Encode one frame.
 *
 * @param tx       Scratch; @p out points into it and stays valid until the next
 *                 mp_frame_encode() on the same scratch.
 * @param ch       Channel id; must satisfy MP_CH_VALID().
 * @param flags    MP_FLAG_* bits.
 * @param payload  Payload bytes; may be NULL only when @p len is 0.
 * @param len      Payload length; at most MP_TX_PAYLOAD_MAX.
 * @param out      Receives a pointer to the wire bytes (COBS body + 0x00).
 * @param out_len  Receives the wire length.
 *
 * @retval 0          Encoded.
 * @retval -EINVAL    NULL argument, bad channel, or unknown flag bits.
 * @retval -EMSGSIZE  @p len exceeds MP_TX_PAYLOAD_MAX.
 * @retval -ENOSPC    COBS refused the scratch (cannot happen with this sizing;
 *                    reported rather than asserted).
 */
int mp_frame_encode(mp_frame_tx_t *tx, uint8_t ch, uint8_t flags,
		    const uint8_t *payload, size_t len, const uint8_t **out,
		    size_t *out_len);

/**
 * Fragment and emit a logical message of any length.
 *
 * Splits @p len into MP_TX_PAYLOAD_MAX chunks, sets MP_FLAG_MORE on every chunk
 * but the last, and hands each encoded frame to @p sink. A zero-length message
 * emits exactly one empty frame.
 *
 * @param sink  Called once per frame; a negative return aborts and is returned.
 *
 * @retval 0        Every fragment was accepted.
 * @retval -EINVAL  NULL argument or bad channel.
 * @retval <0       Whatever @p sink returned.
 */
typedef int (*mp_frame_sink_fn)(void *user, const uint8_t *wire, size_t len);

int mp_frame_send(mp_frame_tx_t *tx, uint8_t ch, const uint8_t *msg, size_t len,
		  mp_frame_sink_fn sink, void *user);

/* ---------------------------------------------------------------- receive */

/**
 * One reassembly slot.
 *
 * Slots are assigned to channels on demand, so a bounded pool serves the whole
 * channel space; a fragmented message arriving while every slot is busy is
 * dropped whole and counted in mp_frame_rx_t::reasm_drops. `buf` is
 * caller-owned, which is what lets the glue decide how large a reassembled
 * message it will accept per channel.
 */
typedef struct {
	uint8_t *buf;
	size_t cap;
	size_t len;
	uint8_t ch;
	bool busy;
} mp_reasm_t;

/**
 * Complete-message callback.
 *
 * @param ch    Channel the message arrived on.
 * @param msg   Message bytes (reassembled if it was fragmented).
 * @param len   Message length.
 * @retval 0    Consumed. A negative return is counted in
 *              mp_frame_rx_t::sink_errors and otherwise ignored — the decoder
 *              must not stall the wire because a consumer is busy.
 */
typedef int (*mp_frame_msg_fn)(void *user, uint8_t ch, const uint8_t *msg,
			       size_t len);

/** Receive state. Caller-owned; populated by mp_frame_rx_init(). */
typedef struct {
	cobs_dec_t dec;
	uint8_t frame[MP_FRAME_MAX];

	mp_reasm_t *reasm;
	uint8_t reasm_n;

	mp_frame_msg_fn on_msg;
	void *user;

	/* counters — all monotonic, for `diag` and the event stream */
	uint32_t frames;       /**< frames that passed COBS, length and CRC */
	uint32_t msgs;         /**< logical messages delivered */
	uint32_t cobs_errors;  /**< malformed or oversized COBS bodies */
	uint32_t crc_errors;   /**< frames dropped silently on CRC (§3.1) */
	uint32_t short_frames; /**< fewer than MP_FRAME_OVERHEAD bytes */
	uint32_t bad_channel;  /**< channel id above MP_CH_MAX */
	uint32_t bad_flags;    /**< undefined flag bits set */
	uint32_t reasm_drops;  /**< fragments dropped: no slot, or slot overflow */
	uint32_t sink_errors;  /**< on_msg() returned negative */
} mp_frame_rx_t;

/**
 * Bind a receive context.
 *
 * @param rx       Context.
 * @param reasm    Reassembly slot array; may be NULL with @p reasm_n 0, in
 *                 which case fragmented messages are dropped and counted.
 * @param reasm_n  Number of slots. Every slot's `buf`/`cap` must already be
 *                 set by the caller.
 * @param on_msg   Message callback; may be NULL (messages are then counted and
 *                 discarded, which is what the fuzz harness wants).
 * @param user     Opaque callback argument.
 *
 * @retval 0        Bound.
 * @retval -EINVAL  @p rx is NULL, or a slot has a NULL buf with a non-zero cap.
 */
int mp_frame_rx_init(mp_frame_rx_t *rx, mp_reasm_t *reasm, uint8_t reasm_n,
		     mp_frame_msg_fn on_msg, void *user);

/** Discard any frame and any reassembly in progress; counters are kept. */
void mp_frame_rx_reset(mp_frame_rx_t *rx);

/**
 * Feed one received byte.
 *
 * Every failure mode is absorbed: a bad CRC, a bad channel, an over-long frame
 * and a malformed COBS body all resynchronise at the next delimiter and bump a
 * counter. Nothing a hostile host can send makes this function return an error
 * that the caller has to handle, which is what lets the RX path be a plain byte
 * pump.
 *
 * @retval 1        A complete message was delivered to the callback.
 * @retval 0        More input needed, or the frame was dropped.
 * @retval -EINVAL  @p rx is NULL.
 */
int mp_frame_rx_byte(mp_frame_rx_t *rx, uint8_t b);

/** Feed a buffer; returns the number of messages delivered, or -EINVAL. */
int mp_frame_rx_input(mp_frame_rx_t *rx, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_MP_MP_FRAME_H_ */
