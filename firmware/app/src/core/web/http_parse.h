/*
 * STS1000 "Meridian" — core/web: HTTP/1.1 request parser.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. Bounded, allocation-free, zero-copy: the parser records
 * spans into the caller's receive buffer instead of copying, so the whole
 * request head costs one http_req_t (~160 B) of state.
 *
 * Threat model
 * ------------
 * This is the first code an unauthenticated attacker reaches, so it is strict by
 * construction rather than lenient by default. Specifically it REFUSES, with a
 * distinct error, every ambiguity that has produced a real request-smuggling or
 * response-splitting CVE:
 *
 *   - `Content-Length` together with `Transfer-Encoding`     (-EPROTO)
 *   - two `Content-Length` headers that disagree             (-EPROTO)
 *   - a `Transfer-Encoding` whose final coding is not chunked(-ENOTSUP)
 *   - obs-fold (a header line starting with SP/HTAB)         (-EBADMSG)
 *   - whitespace between a header name and its colon         (-EBADMSG)
 *   - a bare LF line terminator (CRLF is required)           (-EBADMSG)
 *   - NUL, CR or LF anywhere inside a header value           (-EBADMSG)
 *   - a non-token character in the method or a header name   (-EBADMSG)
 *   - a request-target that is not origin-form ('/'-leading) (-EBADMSG)
 *   - a head larger than HTTP_HEAD_MAX or more than
 *     HTTP_HEADERS_MAX headers                               (-E2BIG)
 *
 * It also never allocates, never recurses, and never reads past @p len — the
 * host suite fuzzes it under ASan+UBSan against random and mutated input to
 * hold that line.
 *
 * What it deliberately does NOT do: absolute-form targets (proxy requests),
 * multipart bodies, and content codings other than chunked. This is a device
 * management interface, not a general-purpose HTTP server.
 */

#ifndef STS1000_CORE_WEB_HTTP_PARSE_H_
#define STS1000_CORE_WEB_HTTP_PARSE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "web/web.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ limits */

/** Largest request head (request line + headers + terminator) accepted. */
#ifndef HTTP_HEAD_MAX
#define HTTP_HEAD_MAX 3072U
#endif

/** Largest request-target accepted, in bytes. */
#ifndef HTTP_TARGET_MAX
#define HTTP_TARGET_MAX 512U
#endif

/** Most header fields accepted in one request. */
#ifndef HTTP_HEADERS_MAX
#define HTTP_HEADERS_MAX 32U
#endif

/** Largest single header field line accepted, in bytes. */
#ifndef HTTP_HEADER_LINE_MAX
#define HTTP_HEADER_LINE_MAX 1024U
#endif

/** Largest body the server will accept for a non-streaming route. */
#ifndef HTTP_BODY_MAX
#define HTTP_BODY_MAX 4096U
#endif

/* ------------------------------------------------------------------- types */

/** A span into the caller's buffer. Never NUL-terminated. */
typedef struct {
	const char *p;
	uint16_t    n;
} http_span_t;

/** Flags on a parsed request. */
#define HTTP_F_KEEPALIVE   (1U << 0) /**< connection may be reused */
#define HTTP_F_CHUNKED     (1U << 1) /**< body is chunked, length unknown */
#define HTTP_F_UPGRADE_WS  (1U << 2) /**< a well-formed WebSocket upgrade */
#define HTTP_F_GZIP_OK     (1U << 3) /**< client accepts Content-Encoding: gzip */
#define HTTP_F_HAS_BODY    (1U << 4) /**< chunked, or Content-Length > 0 */
#define HTTP_F_EXPECT_100  (1U << 5) /**< Expect: 100-continue was sent */
#define HTTP_F_HAS_CL      (1U << 6) /**< a Content-Length header was present */

/** A parsed request head. Spans point into the buffer passed to the parser. */
typedef struct {
	uint8_t  method;        /**< web_method_t (WEB_METHOD_UNKNOWN allowed) */
	uint8_t  version_minor; /**< 0 for HTTP/1.0, 1 for HTTP/1.1 */
	uint32_t flags;         /**< HTTP_F_* */
	uint32_t content_length;

	http_span_t target;  /**< full request-target, still percent-encoded */
	http_span_t path;    /**< target up to '?', still percent-encoded */
	http_span_t query;   /**< after '?', excluding it; n == 0 when absent */

	http_span_t host;
	http_span_t authorization;
	http_span_t cookie;
	http_span_t content_type;
	http_span_t if_none_match;
	http_span_t accept_encoding;
	http_span_t origin;
	http_span_t referer;
	http_span_t csrf;        /**< X-CSRF-Token */
	http_span_t ws_key;      /**< Sec-WebSocket-Key */
	http_span_t ws_version;  /**< Sec-WebSocket-Version */
	http_span_t ws_protocol; /**< Sec-WebSocket-Protocol */

	uint16_t head_len;  /**< bytes consumed, including the final CRLFCRLF */
	uint16_t n_headers;
} http_req_t;

/* ------------------------------------------------------------------- parse */

/**
 * Parse a request head out of (@p buf, @p len).
 *
 * @param out  Zeroed and filled on success. Its spans alias @p buf, which must
 *             therefore outlive every use of @p out.
 *
 * @retval 0        A complete head was parsed; @ref http_req_t::head_len says
 *                  where the body starts.
 * @retval -EAGAIN  No CRLFCRLF yet and the input is still within the limits.
 * @retval -EINVAL  Bad argument.
 * @retval -E2BIG   Head, header count or a single line over its limit.
 * @retval -EBADMSG Malformed syntax (see the threat model above).
 * @retval -ENOTSUP Unsupported HTTP version or transfer coding.
 * @retval -EPROTO  A framing ambiguity (CL/TE conflict, duplicate CL).
 */
int http_parse_request(const char *buf, size_t len, http_req_t *out);

/* ------------------------------------------------------------- accessors */

/** True when @p s equals @p lit, ASCII case-insensitively. */
bool http_span_eq_ci(http_span_t s, const char *lit);

/** True when @p s equals @p lit exactly. */
bool http_span_eq(http_span_t s, const char *lit);

/** Copy @p s into @p out as a NUL-terminated string, truncating to fit. */
size_t http_span_str(http_span_t s, char *out, size_t cap);

/**
 * Percent-decode (@p in, @p n) into @p out, NUL-terminated.
 *
 * '+' is NOT translated to a space: this decoder is used for both path segments
 * and query values, and RFC 3986 gives '+' no special meaning in a path. Query
 * values that need a literal '+' must percent-encode it, which every correct
 * client does.
 *
 * @retval >=0      Bytes written, excluding the NUL.
 * @retval -EINVAL  Bad argument.
 * @retval -EILSEQ  A truncated or non-hex escape, or an encoded NUL.
 * @retval -ENOSPC  @p cap too small.
 */
int http_pct_decode(const char *in, size_t n, char *out, size_t cap);

/** http_pct_decode() of @ref http_req_t::path. */
int http_path_decode(const http_req_t *r, char *out, size_t cap);

/**
 * Look up query parameter @p name and percent-decode its value.
 *
 * @retval >=0      Bytes written, excluding the NUL (0 for `?flag` / `?flag=`).
 * @retval -EINVAL  Bad argument.
 * @retval -ENOENT  No such parameter.
 * @retval -EILSEQ  The value is not valid percent-encoding.
 * @retval -ENOSPC  @p cap too small.
 */
int http_query_get(const http_req_t *r, const char *name, char *out, size_t cap);

/**
 * Look up query parameter @p name as an unsigned decimal.
 *
 * @retval 0        Parsed into @p out.
 * @retval -ENOENT  Absent.
 * @retval -EBADMSG Present but not a decimal number.
 * @retval -ERANGE  Larger than UINT32_MAX.
 */
int http_query_get_u32(const http_req_t *r, const char *name, uint32_t *out);

/** True when query parameter @p name is present (with or without a value). */
bool http_query_has(const http_req_t *r, const char *name);

/**
 * Look up cookie @p name in the request's Cookie header.
 *
 * The value is NOT percent-decoded: cookie values in this API are opaque
 * session tokens drawn from [0-9a-f], and decoding them would let `%00` smuggle
 * a terminator into a token comparison.
 *
 * @retval >=0      Bytes written, excluding the NUL.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOENT  No such cookie.
 * @retval -ENOSPC  @p cap too small.
 */
int http_cookie_get(const http_req_t *r, const char *name, char *out,
		    size_t cap);

/** True when @p r carries `Authorization: Bearer <tok>`; @p out gets the token. */
bool http_bearer_token(const http_req_t *r, char *out, size_t cap);

/* --------------------------------------------------------- chunked bodies */

/** Chunked-decoder phase. Exposed only so a caller can log it. */
typedef enum {
	HTTP_CHUNK_SIZE = 0, /**< reading the hex size line */
	HTTP_CHUNK_EXT,      /**< skipping a chunk extension to CRLF */
	HTTP_CHUNK_DATA,     /**< copying chunk-data */
	HTTP_CHUNK_DATA_CR,  /**< expecting the CR after chunk-data */
	HTTP_CHUNK_DATA_LF,  /**< expecting the LF after chunk-data */
	HTTP_CHUNK_TRAILER,  /**< after the 0-chunk, skipping trailers */
	HTTP_CHUNK_DONE,
	HTTP_CHUNK_ERROR,
} http_chunk_phase_t;

/** Chunked-body decoder state. Caller-owned; init with http_chunked_init(). */
typedef struct {
	uint8_t  phase;      /**< http_chunk_phase_t */
	uint8_t  size_digits;/**< hex digits seen in the current size line */
	uint32_t remain;     /**< bytes left in the current chunk */
	uint32_t total;      /**< bytes decoded so far */
	uint32_t max_total;  /**< refuse past this many decoded bytes */
	uint32_t trailer_len;/**< bytes of trailer section seen */
	bool     saw_cr;     /**< a CR is pending inside a line */
	int      err;        /**< latched error */
} http_chunked_t;

/** Bind a decoder and set its decoded-size ceiling (0 uses HTTP_BODY_MAX). */
void http_chunked_init(http_chunked_t *st, uint32_t max_total);

/**
 * Feed encoded bytes, appending decoded body bytes to @p out.
 *
 * Idempotent on error: once an error latches, every later call returns it and
 * consumes nothing.
 *
 * @param consumed  Optional; encoded bytes taken from @p in.
 * @param out_len   In/out: current decoded length, advanced by this call.
 *
 * @retval 1        The terminating 0-chunk and its trailer section arrived.
 * @retval 0        More encoded bytes needed.
 * @retval -EINVAL  Bad argument.
 * @retval -EBADMSG Malformed chunk framing.
 * @retval -E2BIG   Decoded size past @ref http_chunked_t::max_total, or a
 *                  trailer section larger than HTTP_HEADER_LINE_MAX.
 * @retval -ENOSPC  @p out_cap too small for the decoded body.
 */
int http_chunked_feed(http_chunked_t *st, const char *in, size_t in_len,
		      size_t *consumed, uint8_t *out, size_t out_cap,
		      size_t *out_len);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_WEB_HTTP_PARSE_H_ */
