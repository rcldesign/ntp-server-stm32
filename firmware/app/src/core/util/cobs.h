/*
 * STS1000 "Meridian" — core/util: Consistent Overhead Byte Stuffing.
 *
 * Platform-neutral C11. No dynamic allocation, no platform headers.
 *
 * Implements COBS exactly as specified by S. Cheshire and M. Baker,
 * "Consistent Overhead Byte Stuffing", IEEE/ACM Transactions on Networking
 * 7(2), April 1999. Encoded output contains no 0x00 byte, so 0x00 is free to
 * serve as the frame delimiter. This is the framing layer of the Meridian
 * Console Protocol (ARCHITECTURE.md §7).
 *
 * Delimiter ownership: cobs_encode() emits the encoded *body only*. The caller
 * appends the trailing 0x00. cobs_decode() likewise consumes a body with no
 * delimiter. The streaming decoder (cobs_dec_*) is the exception — it is fed
 * the raw wire stream including delimiters and reports frame boundaries.
 *
 * Encoded-body properties (n = decoded length):
 *   - n == 0 encodes to the single byte 0x01.
 *   - Encoded length is at most cobs_encode_max(n) == n + n/254 + 1.
 *   - A run of exactly 254 non-zero bytes encodes to 0xFF followed by those
 *     254 bytes, with no trailing 0x01 group: a group code of 0xFF means
 *     "254 data bytes and no implied zero", which is what removes the
 *     ambiguity at the 254-byte boundary.
 */

#ifndef STS1000_CORE_UTIL_COBS_H_
#define STS1000_CORE_UTIL_COBS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Largest decoded frame body the codec will accept, in bytes.
 *
 * Default is sized for the Meridian Console Protocol frame (ARCHITECTURE.md
 * §7): 8 B header + 1040 B payload + 4 B CRC-32 = 1052 B. Override at build
 * time if a larger framing user appears; nothing here assumes the value.
 */
#ifndef COBS_MAX_FRAME
#define COBS_MAX_FRAME 1052U
#endif

/**
 * Worst-case encoded length for @p n decoded bytes: one overhead byte plus one
 * extra per 254-byte run. Usable in a constant expression for buffer sizing.
 */
#define COBS_ENCODE_MAX(n) ((n) + (n) / 254U + 1U)

/** Encoded-body length bound for COBS_MAX_FRAME. */
#define COBS_MAX_FRAME_ENCODED COBS_ENCODE_MAX(COBS_MAX_FRAME)

/** Worst-case encoded length for @p len decoded bytes (function form). */
size_t cobs_encode_max(size_t len);

/** Worst-case decoded length for @p enc_len encoded bytes. */
size_t cobs_decode_max(size_t enc_len);

/**
 * COBS-encode a frame body.
 *
 * @param src      Input bytes; may be NULL only when @p src_len is 0.
 * @param src_len  Number of input bytes; must not exceed COBS_MAX_FRAME.
 * @param dst      Output buffer, at least cobs_encode_max(@p src_len) bytes for
 *                 a guaranteed fit. Never written past the returned length.
 * @param dst_cap  Capacity of @p dst.
 * @param out_len  Receives the encoded length on success. Untouched on failure.
 *
 * @retval 0          Success.
 * @retval -EINVAL    @p dst or @p out_len is NULL, or @p src is NULL with a
 *                    non-zero @p src_len.
 * @retval -EMSGSIZE  @p src_len exceeds COBS_MAX_FRAME.
 * @retval -ENOSPC    @p dst_cap is too small. @p dst holds partial output.
 */
int cobs_encode(const uint8_t *src, size_t src_len,
		uint8_t *dst, size_t dst_cap, size_t *out_len);

/**
 * COBS-decode a frame body (no delimiter).
 *
 * @param src      Encoded bytes, delimiter excluded. Must be non-empty: the
 *                 shortest legal encoded body is the single byte 0x01.
 * @param src_len  Number of encoded bytes; must not exceed
 *                 COBS_MAX_FRAME_ENCODED.
 * @param dst      Output buffer; may be NULL only when @p dst_cap is 0.
 * @param dst_cap  Capacity of @p dst.
 * @param out_len  Receives the decoded length on success. Untouched on failure.
 *
 * @retval 0          Success.
 * @retval -EINVAL    @p out_len is NULL, or @p dst is NULL with a non-zero
 *                    @p dst_cap, or @p src is NULL.
 * @retval -EMSGSIZE  @p src_len exceeds COBS_MAX_FRAME_ENCODED.
 * @retval -EBADMSG   Malformed: empty input, a 0x00 code byte, a 0x00 inside
 *                    the frame body, or a group that runs past the input.
 * @retval -ENOSPC    @p dst_cap is too small. @p dst holds partial output.
 */
int cobs_decode(const uint8_t *src, size_t src_len,
		uint8_t *dst, size_t dst_cap, size_t *out_len);

/**
 * Incremental COBS decoder for a byte-at-a-time wire stream (UART RX).
 *
 * All fields are private; use the cobs_dec_* functions. The caller owns the
 * output buffer, so the decoder allocates nothing.
 */
typedef struct {
	uint8_t *buf;       /* caller-owned output buffer */
	size_t cap;         /* capacity of buf */
	size_t len;         /* decoded bytes written so far */
	size_t left;        /* data bytes still expected in the current group */
	bool zero_pending;  /* current group implies a zero if another follows */
	bool started;       /* at least one code byte seen since the last frame */
	bool complete;      /* a frame was reported and buf holds it */
	int err;            /* sticky error, reported at the next delimiter */
} cobs_dec_t;

/**
 * Bind a streaming decoder to an output buffer and reset it.
 *
 * @param d    Decoder.
 * @param buf  Output buffer; may be NULL only when @p cap is 0.
 * @param cap  Capacity of @p buf. Size it to the largest decoded frame the
 *             caller accepts (COBS_MAX_FRAME for MCP) — the decoder reports
 *             -ENOSPC for anything longer rather than overrunning.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p d is NULL, or @p buf is NULL with a non-zero @p cap.
 */
int cobs_dec_init(cobs_dec_t *d, uint8_t *buf, size_t cap);

/** Discard any frame in progress and clear the sticky error. */
void cobs_dec_reset(cobs_dec_t *d);

/**
 * Feed one received byte.
 *
 * A 0x00 byte is the frame delimiter. Repeated delimiters and a leading
 * delimiter are frame separators, not errors: they are skipped silently. Note
 * that a *zero-length payload* is still a real frame — it arrives as the two
 * bytes 0x01 0x00 and is reported with cobs_dec_len() == 0.
 *
 * Once an error is detected the decoder swallows bytes until the next
 * delimiter, reports the error there, and resynchronises — one corrupt frame
 * cannot desynchronise the stream.
 *
 * @param d  Decoder.
 * @param b  Received byte.
 *
 * @retval 1          A complete frame is available; read it with cobs_dec_buf()
 *                    and cobs_dec_len() before feeding the next byte.
 * @retval 0          More input needed.
 * @retval -EINVAL    @p d is NULL.
 * @retval -EBADMSG   The frame just delimited was malformed.
 * @retval -ENOSPC    The frame just delimited exceeded the output buffer.
 */
int cobs_dec_byte(cobs_dec_t *d, uint8_t b);

/** Decoded length of the frame reported by the last cobs_dec_byte() == 1. */
size_t cobs_dec_len(const cobs_dec_t *d);

/** Decoded bytes of the frame reported by the last cobs_dec_byte() == 1. */
const uint8_t *cobs_dec_buf(const cobs_dec_t *d);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_UTIL_COBS_H_ */
