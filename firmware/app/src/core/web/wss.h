/*
 * STS1000 "Meridian" — core/web: RFC 6455 WebSocket server (spec §5.1).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11, allocation-free. The glue owns the TLS socket; this
 * module owns the handshake computation, the frame codec, the message
 * reassembler and the telemetry record encoder.
 *
 * Strictness (the frame decoder is attacker-facing and is fuzzed)
 * --------------------------------------------------------------
 *   - a client frame that is not masked is a protocol error (RFC 6455 §5.1);
 *   - RSV1..3 must be zero (no extensions negotiated);
 *   - a control frame must have FIN set and a payload <= 125 bytes (§5.5);
 *   - lengths must use the shortest form: 126 for 126..65535, 127 for
 *     65536.. only, and the 64-bit form's high bit must be clear (§5.2);
 *   - a continuation frame with no message in progress, or a new data frame
 *     while one is in progress, is a protocol error;
 *   - a close frame carries either no payload or a 2-byte code plus valid
 *     UTF-8 reason; reserved and out-of-range codes are rejected (§7.4.1);
 *   - a TEXT message must be valid UTF-8 (§8.1) — validated on the whole
 *     reassembled message, since a code point may straddle a fragment.
 *
 * Every rejection maps to a close code so the peer learns what it did wrong,
 * which is what makes an autobahn-style suite pass rather than time out.
 */

#ifndef STS1000_CORE_WEB_WSS_H_
#define STS1000_CORE_WEB_WSS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "web/web.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------- handshake */

/** RFC 6455 §1.3 magic GUID. */
#define WSS_GUID "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

/** Length of a Sec-WebSocket-Accept value, excluding the NUL. */
#define WSS_ACCEPT_LEN 28U

/** Longest Sec-WebSocket-Key this accepts (a correct one is exactly 24). */
#define WSS_KEY_MAX 32U

/**
 * Compute Sec-WebSocket-Accept for the client's Sec-WebSocket-Key.
 *
 * base64(SHA-1(key || WSS_GUID)). The canonical RFC 6455 §1.3 example
 * ("dGhlIHNhbXBsZSBub25jZQ==" -> "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") is pinned by
 * the host suite.
 *
 * @param out  Receives a NUL-terminated 28-character value.
 * @retval 0        Written.
 * @retval -EINVAL  Bad argument, or a key longer than WSS_KEY_MAX.
 * @retval -ENOSPC  @p cap < WSS_ACCEPT_LEN + 1.
 */
int wss_accept_key(const char *key, size_t key_len, char *out, size_t cap);

/**
 * SHA-1 of @p n bytes. Exposed only so the host suite can pin it against the
 * FIPS 180-4 vectors; nothing else should use SHA-1.
 */
void wss_sha1(const uint8_t *in, size_t n, uint8_t out[20]);

/* ------------------------------------------------------------------ frames */

/** Opcodes (RFC 6455 §5.2). */
typedef enum {
	WSS_OP_CONT  = 0x0,
	WSS_OP_TEXT  = 0x1,
	WSS_OP_BIN   = 0x2,
	WSS_OP_CLOSE = 0x8,
	WSS_OP_PING  = 0x9,
	WSS_OP_PONG  = 0xA,
} wss_op_t;

/** Close codes this server emits (RFC 6455 §7.4.1). */
#define WSS_CLOSE_NORMAL         1000U
#define WSS_CLOSE_GOING_AWAY     1001U
#define WSS_CLOSE_PROTOCOL_ERROR 1002U
#define WSS_CLOSE_UNSUPPORTED    1003U
#define WSS_CLOSE_NO_STATUS      1005U /* never sent on the wire */
#define WSS_CLOSE_INVALID_DATA   1007U
#define WSS_CLOSE_POLICY         1008U
#define WSS_CLOSE_TOO_BIG        1009U
#define WSS_CLOSE_INTERNAL       1011U

/** Largest control-frame payload (RFC 6455 §5.5). */
#define WSS_CONTROL_MAX 125U

/** Largest frame header: 2 + 8 length + 4 mask. */
#define WSS_HDR_MAX 14U

/** A decoded frame header. */
typedef struct {
	uint8_t  op;
	bool     fin;
	bool     masked;
	uint64_t len;     /**< payload length */
	uint8_t  hdr_len; /**< bytes the header occupies */
	uint8_t  mask[4];
} wss_hdr_t;

/**
 * Parse a frame header.
 *
 * @retval 0        Parsed; @p out is filled.
 * @retval -EAGAIN  Fewer than @ref wss_hdr_t::hdr_len bytes available.
 * @retval -EINVAL  Bad argument.
 * @retval -EPROTO  A rule from the header comment was broken.
 * @retval -EMSGSIZE A 64-bit length with the high bit set.
 */
int wss_hdr_parse(const uint8_t *buf, size_t n, wss_hdr_t *out);

/**
 * XOR-unmask @p n bytes in place.
 *
 * @param offset  Byte offset of @p p within the frame payload, so a payload
 *                delivered in pieces stays in phase with the 4-byte key.
 */
void wss_unmask(uint8_t *p, size_t n, const uint8_t mask[4], uint64_t offset);

/**
 * Encode a server->client frame (never masked, per RFC 6455 §5.1).
 *
 * @retval >=0      Total bytes written.
 * @retval -EINVAL  Bad argument or an illegal opcode/length combination.
 * @retval -ENOSPC  @p cap too small.
 */
int wss_encode(uint8_t op, bool fin, const uint8_t *payload, size_t len,
	       uint8_t *out, size_t cap);

/**
 * Write just the frame header for a payload of @p len bytes.
 *
 * For payloads too large to copy into a staging buffer: emit the header, then
 * write the payload straight from wherever it already is.
 *
 * @retval >=0      Header bytes written (2, 4 or 10).
 * @retval -EINVAL  Bad argument or an illegal opcode/length combination.
 * @retval -ENOSPC  @p cap too small.
 */
int wss_encode_header(uint8_t op, bool fin, size_t len, uint8_t *out,
		      size_t cap);

/**
 * Encode a Close frame. @p code of 0 emits an empty close (no status).
 *
 * @retval >=0      Bytes written.
 * @retval -EINVAL  A code the RFC forbids on the wire.
 * @retval -ENOSPC  @p cap too small.
 */
int wss_encode_close(uint16_t code, const char *reason, uint8_t *out,
		     size_t cap);

/** True when @p n bytes at @p p are well-formed UTF-8 (RFC 3629, no surrogates,
 *  no overlong forms, no code point above U+10FFFF). */
bool wss_utf8_valid(const uint8_t *p, size_t n);

/* ------------------------------------------------------- message reassembly */

/** Reassembler. Caller supplies the message buffer; nothing is allocated. */
typedef struct {
	uint8_t *buf;
	size_t   cap;
	size_t   len;      /**< bytes of the message assembled so far */
	uint8_t  msg_op;   /**< opcode of the message in progress, 0 if none */
	bool     in_frag;

	uint32_t frames;
	uint32_t messages;
	uint32_t pings;
	uint32_t pongs;
	uint32_t protocol_errors;
	uint32_t oversize;
} wss_rx_t;

/** Bind a reassembler to its message buffer. */
int wss_rx_init(wss_rx_t *r, uint8_t *buf, size_t cap);

/**
 * Consume as many whole frames as (@p in, @p n) holds, stopping at the first
 * complete application message or control frame.
 *
 * @p in is NOT const: client frames are always masked, and the payload is
 * unmasked *in place* rather than into a second full-size buffer — a real
 * consideration on a 640 KB part. The caller must therefore treat the consumed
 * region as clobbered, and must not re-feed bytes it has already offered.
 *
 * @param consumed    Bytes taken from @p in.
 * @param out_op      Receives the opcode of the delivered message/frame.
 * @param out_p       Receives a pointer to the payload (inside @p r->buf for a
 *                    reassembled message, inside @p in for a control frame).
 * @param out_len     Receives the payload length.
 * @param close_code  On a protocol error, the code to close with.
 *
 * @retval 1        A message (TEXT/BIN) or control frame is ready.
 * @retval 0        More bytes needed; @p consumed may still be non-zero.
 * @retval -EINVAL  Bad argument.
 * @retval -EPROTO  Protocol error; the caller must send Close(@p close_code).
 * @retval -EMSGSIZE The message exceeds the reassembly buffer.
 */
int wss_rx_feed(wss_rx_t *r, uint8_t *in, size_t n, size_t *consumed,
		uint8_t *out_op, const uint8_t **out_p, size_t *out_len,
		uint16_t *close_code);

/* ------------------------------------------------------------- send queue */

/** Queued frames. Four covers a pong, a close and two in flight. */
#ifndef WSS_TXQ_SLOTS
#define WSS_TXQ_SLOTS 4U
#endif

/**
 * Bytes per queue slot.
 *
 * The queue carries CONTROL frames only, so a slot needs the 2-byte header plus
 * WSS_CONTROL_MAX. Telemetry records are far larger than any sane queue slot (a
 * nine-rail power snapshot alone runs to a couple of kilobytes), so they are
 * framed with wss_encode_header() and written straight to the socket instead of
 * being buffered — see the wss_txq_t comment.
 */
#ifndef WSS_TXQ_SLOT_BYTES
#define WSS_TXQ_SLOT_BYTES (WSS_CONTROL_MAX + 4U)
#endif

/**
 * Bounded, lossy-by-design send queue for CONTROL frames.
 *
 * A push into a full queue drops the OLDEST entry and counts it, so a slow reader
 * can never back-pressure the server thread — the discipline core/mcp applies to
 * its EVT frames. @p urgent pushes to the front so a pong or a close cannot be
 * starved.
 *
 * Application messages deliberately do NOT go through here: queueing a
 * multi-kilobyte telemetry record per connection would cost more SRAM than the
 * whole web plane is allowed, and a record that cannot be written now is better
 * dropped than buffered, because the next one supersedes it.
 */
typedef struct {
	uint8_t  slot[WSS_TXQ_SLOTS][WSS_TXQ_SLOT_BYTES];
	uint16_t slot_len[WSS_TXQ_SLOTS];
	uint8_t  head; /**< next to send */
	uint8_t  count;
	uint32_t pushed;
	uint32_t dropped;
	uint32_t sent;
} wss_txq_t;

void wss_txq_init(wss_txq_t *q);

/**
 * Queue an already-framed buffer.
 *
 * @retval 0        Queued.
 * @retval 1        Queued after dropping the oldest entry.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOSPC  Larger than WSS_TXQ_SLOT_BYTES.
 */
int wss_txq_push(wss_txq_t *q, const uint8_t *frame, size_t n, bool urgent);

/** Peek the next frame. Returns 0 with @p p/@p n set, or -ENOENT when empty. */
int wss_txq_peek(const wss_txq_t *q, const uint8_t **p, size_t *n);

/** Drop the frame at the head after a successful send. */
int wss_txq_pop(wss_txq_t *q);

/** Frames currently queued. */
uint8_t wss_txq_count(const wss_txq_t *q);

/* -------------------------------------------------------- telemetry stream */

/** Telemetry group bits, matching the REST /status/<name> route set. */
#define WSS_GRP_SUMMARY (1U << 0)
#define WSS_GRP_TIMING  (1U << 1)
#define WSS_GRP_GNSS    (1U << 2)
#define WSS_GRP_POWER   (1U << 3)
#define WSS_GRP_NET     (1U << 4)
#define WSS_GRP_PTP     (1U << 5)
#define WSS_GRP_ALARMS  (1U << 6)
#define WSS_GRP_LOGS    (1U << 7)
#define WSS_GRP_ALL     0xFFU

/** Default and maximum push rates (spec §5.1: 1-4 Hz). */
#define WSS_RATE_HZ_DEFAULT 1U
#define WSS_RATE_HZ_MAX     4U

/** Per-connection subscription state. */
typedef struct {
	uint8_t  groups;      /**< WSS_GRP_* mask */
	uint8_t  rate_hz;     /**< 1..WSS_RATE_HZ_MAX */
	uint32_t log_cursor;  /**< logring cursor for WSS_GRP_LOGS */
	uint32_t seq;         /**< records emitted */
	uint64_t next_ms;
	uint64_t last_ping_ms;
	bool     awaiting_pong;
} wss_sub_t;

/** Initialise a subscription to the defaults (summary + alarms at 1 Hz). */
void wss_sub_init(wss_sub_t *s);

/**
 * Apply a client `{"op":"subscribe","groups":[...],"rate":N}` message.
 *
 * Unknown group names are ignored rather than fatal, so a newer SPA talking to
 * an older firmware degrades instead of being disconnected.
 *
 * @retval 0        Applied.
 * @retval -EINVAL  Bad argument.
 * @retval -EBADMSG Not a JSON object, or `op` missing/unknown.
 */
int wss_sub_apply(wss_sub_t *s, const char *json, size_t len);

/** Map a group name to its WSS_GRP_* bit, or 0 when unknown. */
uint8_t wss_group_bit(const char *name, size_t n);

/** Name of a single group bit, or NULL. */
const char *wss_group_name(uint8_t bit);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_WEB_WSS_H_ */
