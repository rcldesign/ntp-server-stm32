/*
 * STS1000 "Meridian" — core/web: shared types, JSON codec and small codecs.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation, no platform headers, no globals.
 * This is the management-plane counterpart of core/mcp: core/web owns the whole
 * HTTP/REST/WebSocket state machine and the JSON it speaks, and the Zephyr glue
 * (src/zephyr/net/sts_web.c) owns only the TLS sockets, the threads and the
 * data providers. That split is what makes the attacker-facing parsers
 * (http_parse.c, wss.c) fuzzable on the host.
 *
 * Why a private JSON codec
 * ------------------------
 * The obvious move is to share a JSON module with the rest of the tree. There
 * isn't one yet, and inventing a cross-module dependency for a consumer that
 * needs exactly "write a flat object of scalars" plus "read a named member out
 * of a small request body" would buy nothing. Both halves here are:
 *
 *   - allocation-free: the writer appends into a caller-owned char buffer and
 *     latches an overflow flag rather than truncating silently;
 *   - float-free on the output path: numbers are emitted from integers with an
 *     explicit decimal shift, so no printf("%f") and no libm (see web_jw_kf32).
 *   - non-recursive on the input path: the reader is a bounded scanner with an
 *     explicit depth counter, so a body of 10 000 nested arrays costs O(1)
 *     stack.
 *
 * If a `core/mp` JSON module ever lands, this can be retired; nothing outside
 * core/web depends on it.
 */

#ifndef STS1000_CORE_WEB_WEB_H_
#define STS1000_CORE_WEB_WEB_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ limits */

/** REST API prefix every routed path starts with (spec §5.1). */
#define WEB_API_PREFIX "/api/v1/"
/** Length of WEB_API_PREFIX, excluding the NUL. */
#define WEB_API_PREFIX_LEN 8U

/** Nesting depth the JSON writer tracks. Deeper is a caller bug, not input. */
#define WEB_JSON_DEPTH_MAX 8U

/** Nesting depth the JSON reader will descend before refusing a document. */
#define WEB_JSON_READ_DEPTH_MAX 8U

/* ----------------------------------------------------------------- methods */

/** HTTP method. Anything else parses as WEB_METHOD_UNKNOWN (405, not 400). */
typedef enum {
	WEB_METHOD_GET = 0,
	WEB_METHOD_HEAD,
	WEB_METHOD_POST,
	WEB_METHOD_PUT,
	WEB_METHOD_DELETE,
	WEB_METHOD_OPTIONS,
	WEB_METHOD_COUNT,
	WEB_METHOD_UNKNOWN = 0xFF,
} web_method_t;

/** Method name, e.g. "GET". "?" when out of range. */
const char *web_method_name(uint8_t m);

/** Parse a method token. Returns WEB_METHOD_UNKNOWN for anything unknown. */
uint8_t web_method_parse(const char *p, size_t n);

/** True when @p m may change state (and therefore needs CSRF + a role). */
bool web_method_is_mutating(uint8_t m);

/* ------------------------------------------------------------------- roles */

/**
 * Management role (spec §9.4). Ordered: a route requiring OPERATOR is also
 * open to ADMIN, so the check is a `>=` comparison.
 */
typedef enum {
	WEB_ROLE_NONE = 0,     /**< unauthenticated */
	WEB_ROLE_VIEWER = 1,   /**< read-only */
	WEB_ROLE_OPERATOR = 2, /**< operational control, no security/firmware */
	WEB_ROLE_ADMIN = 3,    /**< everything */
} web_role_t;

/** Role name, e.g. "operator". "none" when out of range. */
const char *web_role_name(uint8_t r);

/**
 * Parse a role name.
 *
 * @retval 0        Parsed into @p out.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOENT  Not a known role name.
 */
int web_role_parse(const char *p, size_t n, uint8_t *out);

/* -------------------------------------------------------------- JSON write */

/**
 * Streaming JSON writer over a caller-owned buffer.
 *
 * Every append is bounds-checked; the first append that would overflow sets
 * @ref overflow and every later append is a no-op, so a caller may emit a whole
 * document and check once at the end (web_jw_finish()). The buffer is left
 * NUL-terminated whenever it holds at least one byte, so a partially-written
 * document is still safe to log.
 */
typedef struct {
	char  *buf;
	size_t cap;
	size_t len;
	bool   overflow;
	bool   err;                        /**< structural misuse detected */
	uint8_t depth;
	bool   nonempty[WEB_JSON_DEPTH_MAX]; /**< a comma is due at this depth */
} web_jw_t;

/** Bind a writer to @p buf. @p cap must be >= 1 (room for the NUL). */
void web_jw_init(web_jw_t *w, char *buf, size_t cap);

void web_jw_obj_begin(web_jw_t *w);
void web_jw_obj_end(web_jw_t *w);
void web_jw_arr_begin(web_jw_t *w);
void web_jw_arr_end(web_jw_t *w);

/** Emit a member name. Must be followed by exactly one value. */
void web_jw_key(web_jw_t *w, const char *key);

/** Emit a string value, escaping per RFC 8259 (control chars as \uXXXX). */
void web_jw_str(web_jw_t *w, const char *s);
/** web_jw_str() for a non-NUL-terminated span. */
void web_jw_strn(web_jw_t *w, const char *s, size_t n);
void web_jw_i64(web_jw_t *w, int64_t v);
void web_jw_u64(web_jw_t *w, uint64_t v);
void web_jw_bool(web_jw_t *w, bool v);
void web_jw_null(web_jw_t *w);

/**
 * Emit @p scaled / 10^@p decimals as a JSON number, without floating point.
 *
 * e.g. web_jw_fixed(w, -12345, 3) writes `-12.345`. @p decimals is clamped to
 * 9; a value of 0 is identical to web_jw_i64().
 */
void web_jw_fixed(web_jw_t *w, int64_t scaled, uint8_t decimals);

/**
 * Emit a float with @p decimals digits after the point.
 *
 * The float is scaled to an integer and handed to web_jw_fixed(), so the output
 * path stays integer-only. Non-finite input (NaN, +-inf) is emitted as `null`,
 * which is what a JSON consumer can actually represent.
 */
void web_jw_f32(web_jw_t *w, float v, uint8_t decimals);

/* key+value convenience forms */
void web_jw_kstr(web_jw_t *w, const char *key, const char *s);
void web_jw_kstrn(web_jw_t *w, const char *key, const char *s, size_t n);
void web_jw_ki64(web_jw_t *w, const char *key, int64_t v);
void web_jw_ku64(web_jw_t *w, const char *key, uint64_t v);
void web_jw_kbool(web_jw_t *w, const char *key, bool v);
void web_jw_knull(web_jw_t *w, const char *key);
void web_jw_kfixed(web_jw_t *w, const char *key, int64_t scaled,
		   uint8_t decimals);
void web_jw_kf32(web_jw_t *w, const char *key, float v, uint8_t decimals);
void web_jw_kobj(web_jw_t *w, const char *key);
void web_jw_karr(web_jw_t *w, const char *key);

/** Emit a lowercase hex string *value* for @p n bytes at @p p. */
void web_jw_hexn(web_jw_t *w, const uint8_t *p, size_t n);

/** Emit a lowercase hex string value for @p n bytes at @p p, under @p key. */
void web_jw_khex(web_jw_t *w, const char *key, const uint8_t *p, size_t n);

/**
 * Close the document.
 *
 * @param out_len  Optional; receives the byte count (excluding the NUL).
 * @retval 0        Complete and well-formed.
 * @retval -ENOSPC  The buffer overflowed; @p out_len is the truncated length.
 * @retval -EPROTO  Unbalanced begin/end, or a key with no value.
 * @retval -EINVAL  @p w is NULL.
 */
int web_jw_finish(web_jw_t *w, size_t *out_len);

/* --------------------------------------------------------------- JSON read */

/** Value kind of a scanned JSON value. */
typedef enum {
	WEB_JSON_NONE = 0,
	WEB_JSON_NULL,
	WEB_JSON_BOOL,
	WEB_JSON_NUM,
	WEB_JSON_STR, /**< span excludes the quotes and is still escaped */
	WEB_JSON_OBJ, /**< span includes the braces */
	WEB_JSON_ARR, /**< span includes the brackets */
} web_json_type_t;

/** A scanned value: a kind plus a span into the caller's document. */
typedef struct {
	uint8_t     type; /**< web_json_type_t */
	const char *p;
	size_t      n;
} web_json_val_t;

/**
 * Find member @p key in the object at (@p json, @p len).
 *
 * The document is scanned, not parsed into a tree: cost is O(len) and stack is
 * O(1). Only the top level of the object is searched; use the returned OBJ/ARR
 * span to descend.
 *
 * @retval 0        Found; @p out describes the value.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOENT  No such member (the object itself was well-formed).
 * @retval -EBADMSG Not an object, or malformed JSON.
 * @retval -E2BIG   Nesting deeper than WEB_JSON_READ_DEPTH_MAX.
 */
int web_json_obj_get(const char *json, size_t len, const char *key,
		     web_json_val_t *out);

/**
 * Iterate the members of the object span @p obj.
 *
 * @param cursor  Set to 0 before the first call; opaque afterwards.
 * @retval 1        @p key / @p val hold the next member.
 * @retval 0        No more members.
 * @retval <0       Same errors as web_json_obj_get().
 */
int web_json_obj_next(const web_json_val_t *obj, size_t *cursor,
		      web_json_val_t *key, web_json_val_t *val);

/**
 * Iterate the elements of the array span @p arr. Same contract as
 * web_json_obj_next().
 */
int web_json_arr_next(const web_json_val_t *arr, size_t *cursor,
		      web_json_val_t *out);

/**
 * Decode a NUM value as a signed integer. Rejects anything with a fraction or
 * an exponent — every numeric field in this API is an integer or a fixed-point
 * integer, and silently truncating 1.9 to 1 is worse than a 400.
 *
 * @retval 0        Decoded.
 * @retval -EINVAL  Bad argument or not a NUM.
 * @retval -EBADMSG Not an integer literal.
 * @retval -ERANGE  Outside int64.
 */
int web_json_i64(const web_json_val_t *v, int64_t *out);

/** web_json_i64() constrained to [0, UINT64_MAX]. */
int web_json_u64(const web_json_val_t *v, uint64_t *out);

/** Decode a BOOL value. -EINVAL when @p v is not a BOOL. */
int web_json_bool(const web_json_val_t *v, bool *out);

/**
 * Copy a STR value out, resolving escapes, NUL-terminated.
 *
 * \uXXXX is decoded to UTF-8; a lone surrogate is rejected (-EILSEQ) rather
 * than emitted, and an embedded NUL ( ) is rejected for the same reason a
 * NUL in a header is: it makes the string mean two different things to two
 * different consumers.
 *
 * @retval >=0      Bytes written, excluding the NUL.
 * @retval -EINVAL  Bad argument or not a STR.
 * @retval -ENOSPC  @p cap too small.
 * @retval -EILSEQ  Bad escape sequence.
 */
int web_json_str_copy(const web_json_val_t *v, char *out, size_t cap);

/** True when the STR value @p v equals @p lit exactly (no escapes resolved). */
bool web_json_str_eq(const web_json_val_t *v, const char *lit);

/* ---------------------------------------------------------- small codecs */

/**
 * Lowercase-hex encode @p n bytes. Always NUL-terminates when @p cap >= 1.
 *
 * @return Characters written, excluding the NUL, or 0 when @p cap is too small.
 */
size_t web_hex_encode(const uint8_t *in, size_t n, char *out, size_t cap);

/**
 * Hex decode, accepting either case. Rejects an odd digit count.
 *
 * @retval >=0      Bytes written.
 * @retval -EINVAL  Bad argument.
 * @retval -EILSEQ  A non-hex character or an odd length.
 * @retval -ENOSPC  @p cap too small.
 */
int web_hex_decode(const char *in, size_t n, uint8_t *out, size_t cap);

/**
 * Standard base64 encode (RFC 4648 §4, padded). Always NUL-terminates when
 * @p cap >= 1.
 *
 * @return Characters written, excluding the NUL, or 0 when @p cap is too small.
 */
size_t web_b64_encode(const uint8_t *in, size_t n, char *out, size_t cap);

/**
 * Standard base64 decode. Whitespace is skipped; padding is optional but must
 * be consistent when present.
 *
 * @retval >=0      Bytes written.
 * @retval -EINVAL  Bad argument.
 * @retval -EILSEQ  Not valid base64.
 * @retval -ENOSPC  @p cap too small.
 */
int web_b64_decode(const char *in, size_t n, uint8_t *out, size_t cap);

/**
 * Constant-time comparison of @p n bytes.
 *
 * @return 0 when equal, non-zero otherwise. The running time depends only on
 *         @p n, so it is safe for tokens, CSRF values and password tags.
 */
int web_ct_memcmp(const void *a, const void *b, size_t n);

/** Constant-time comparison of two NUL-terminated strings of known capacity. */
int web_ct_streq(const char *a, const char *b, size_t max);

/**
 * ASCII case-insensitive comparison of a span against a NUL-terminated literal.
 * Locale-independent by construction (no tolower()).
 */
bool web_span_eq_ci(const char *p, size_t n, const char *lit);

/** ASCII case-sensitive comparison of a span against a literal. */
bool web_span_eq(const char *p, size_t n, const char *lit);

/**
 * Copy a span into @p out as a NUL-terminated string, truncating to fit.
 *
 * @return Bytes written excluding the NUL (0 when @p cap is 0).
 */
size_t web_span_copy(char *out, size_t cap, const char *p, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_WEB_WEB_H_ */
