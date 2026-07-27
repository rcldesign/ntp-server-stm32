/*
 * STS1000 "Meridian" — core/mp: minimal allocation-free JSON codec + base64.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation, no stdio, no libm, no external
 * dependency: the FMT control plane is JSON-RPC 2.0 (spec §3.2) and pulling a
 * general-purpose JSON library into a 2 MB part for it is not a trade worth
 * making.
 *
 * The parser is a flat token array over the caller's buffer (the jsmn shape):
 * nothing is copied, every token is an offset pair into the source, and the
 * caller sizes the token table. Strings are unescaped on demand into a caller
 * buffer, so a hostile 2 KB string cannot cost anything until it is asked for.
 *
 * The writer is a builder over a caller buffer with a sticky error: a caller
 * emits a whole document without checking each call and inspects the result
 * once at mp_jw_finish(). Truncation is an error, never a silently short
 * document.
 *
 * Both halves are bounded by construction — no recursion in the parser (an
 * explicit parent index), no recursion in the writer (an explicit container
 * stack) — because the input is attacker-controlled.
 */

#ifndef STS1000_CORE_MP_MP_JSON_H_
#define STS1000_CORE_MP_MP_JSON_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================== parser ===== */

/** Token kinds. */
typedef enum {
	MP_J_UNDEF = 0,
	MP_J_OBJ,
	MP_J_ARR,
	MP_J_STR,  /**< quotes excluded from [start,end) */
	MP_J_NUM,
	MP_J_BOOL,
	MP_J_NULL,
} mp_json_type_t;

/** Deepest nesting the parser will accept unless the caller lowers it. */
#define MP_JSON_DEPTH_MAX 12U

/** One token: a span of the source plus its place in the tree. */
typedef struct {
	uint8_t type;   /**< mp_json_type_t */
	uint8_t depth;  /**< 0 for the root value */
	int16_t parent; /**< index of the containing token, -1 for the root */
	uint16_t start; /**< first byte of the value in the source */
	uint16_t end;   /**< one past the last byte */
	uint16_t size;  /**< OBJ: member count; ARR: element count; else 0 */
} mp_json_tok_t;

/** Parse result. All fields are read-only after mp_json_parse(). */
typedef struct {
	const char *src;
	size_t len;
	mp_json_tok_t *tok;
	uint16_t tok_cap;
	uint16_t tok_n;
} mp_json_t;

/**
 * Parse a complete JSON document.
 *
 * Exactly one top-level value is accepted; trailing non-whitespace is an error.
 * The document must be shorter than 64 KiB (token offsets are 16-bit).
 *
 * @param p          Receives the parse result.
 * @param src        Source bytes; not copied and must outlive @p p.
 * @param len        Source length.
 * @param tok        Token array, caller-owned.
 * @param tok_cap    Capacity of @p tok in tokens.
 * @param depth_max  Deepest nesting accepted; 0 selects MP_JSON_DEPTH_MAX.
 *
 * @retval >=0        Number of tokens produced.
 * @retval -EINVAL    Bad argument.
 * @retval -EMSGSIZE  @p len is 64 KiB or more.
 * @retval -EBADMSG   Malformed JSON (including a truncated document).
 * @retval -ENOSPC    @p tok_cap exhausted.
 * @retval -E2BIG     Nesting deeper than @p depth_max.
 */
int mp_json_parse(mp_json_t *p, const char *src, size_t len,
		  mp_json_tok_t *tok, uint16_t tok_cap, uint8_t depth_max);

/** The root token index (0), or -ENOENT for an empty parse. */
int mp_json_root(const mp_json_t *p);

/** Token @p i, or NULL when out of range. */
const mp_json_tok_t *mp_json_at(const mp_json_t *p, int i);

/**
 * Look up @p key in object token @p obj.
 *
 * @retval >=0      Token index of the member's *value*.
 * @retval -EINVAL  @p p is NULL, @p obj out of range, or @p key NULL.
 * @retval -ENOTDIR @p obj is not an object.
 * @retval -ENOENT  No such member.
 */
int mp_json_obj_get(const mp_json_t *p, int obj, const char *key);

/** True when object @p obj has member @p key. */
bool mp_json_has(const mp_json_t *p, int obj, const char *key);

/**
 * Element @p idx of array token @p arr.
 *
 * @retval >=0      Token index.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOTDIR @p arr is not an array.
 * @retval -ENOENT  @p idx out of range.
 */
int mp_json_arr_at(const mp_json_t *p, int arr, uint16_t idx);

/** Element count of an array or member count of an object; 0 otherwise. */
uint16_t mp_json_count(const mp_json_t *p, int i);

/**
 * Copy string token @p i out, resolving JSON escapes.
 *
 * `\uXXXX` is encoded as UTF-8, including surrogate pairs; an unpaired
 * surrogate is emitted as U+FFFD rather than rejected, because rejecting it
 * would let a malformed name field fail a whole request that has nothing else
 * wrong with it.
 *
 * @param out  Destination; always NUL-terminated on success.
 * @param cap  Capacity of @p out including the NUL.
 *
 * @retval >=0      Bytes written, excluding the NUL.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOTDIR @p i is not a string.
 * @retval -ENOSPC  @p cap too small.
 * @retval -EBADMSG Invalid escape sequence.
 */
int mp_json_str(const mp_json_t *p, int i, char *out, size_t cap);

/** True when string token @p i equals the NUL-terminated @p s (after escapes). */
bool mp_json_streq(const mp_json_t *p, int i, const char *s);

/**
 * Decode number token @p i as a signed 64-bit integer.
 *
 * A fractional or exponent form is rejected: a control-plane field declared
 * integral must not silently accept `1.5`.
 *
 * @retval 0        Decoded.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOTDIR @p i is not a number.
 * @retval -EBADMSG Not an integer literal.
 * @retval -ERANGE  Outside int64.
 */
int mp_json_i64(const mp_json_t *p, int i, int64_t *out);

/** mp_json_i64() clamped into [lo, hi]; -ERANGE when outside. */
int mp_json_i32_range(const mp_json_t *p, int i, int32_t lo, int32_t hi,
		      int32_t *out);

/** Decode number token @p i as a float (accepts integer, fraction, exponent). */
int mp_json_f32(const mp_json_t *p, int i, float *out);

/** Decode boolean token @p i. -ENOTDIR when it is not a boolean. */
int mp_json_bool(const mp_json_t *p, int i, bool *out);

/* =========================================================== writer ===== */

/** Container-stack depth of the writer. */
#define MP_JW_DEPTH_MAX 12U

/** JSON builder over a caller buffer. */
typedef struct {
	char *buf;
	size_t cap;
	size_t len;
	int err; /**< sticky: first failure wins */
	uint8_t depth;
	/** Per level: 1 = object, 0 = array. */
	uint8_t kind[MP_JW_DEPTH_MAX];
	/** Per level: whether a separator is owed before the next item. */
	bool sep[MP_JW_DEPTH_MAX];
	/** True while a key has been written and its value is owed. */
	bool want_value;
} mp_jw_t;

/**
 * Bind a builder. @p cap must leave room for the NUL mp_jw_finish() writes.
 *
 * @retval 0        Bound.
 * @retval -EINVAL  @p w or @p buf is NULL, or @p cap is 0.
 */
int mp_jw_init(mp_jw_t *w, char *buf, size_t cap);

int mp_jw_obj_open(mp_jw_t *w);
int mp_jw_obj_close(mp_jw_t *w);
int mp_jw_arr_open(mp_jw_t *w);
int mp_jw_arr_close(mp_jw_t *w);

/** Write a member name. Only legal directly inside an object. */
int mp_jw_key(mp_jw_t *w, const char *key);

int mp_jw_str(mp_jw_t *w, const char *s);
int mp_jw_strn(mp_jw_t *w, const char *s, size_t n);
int mp_jw_i64(mp_jw_t *w, int64_t v);
int mp_jw_u64(mp_jw_t *w, uint64_t v);
int mp_jw_bool(mp_jw_t *w, bool v);
int mp_jw_null(mp_jw_t *w);

/**
 * Write a float with @p decimals fixed decimal places (0..6).
 *
 * NaN and infinity are written as `null`: JSON has no representation for them,
 * and a telemetry field that is genuinely unknown is better expressed as null
 * than as a token no host can parse.
 */
int mp_jw_f32(mp_jw_t *w, float v, uint8_t decimals);

/** Write pre-formatted JSON verbatim (used for cached manifest fragments). */
int mp_jw_raw(mp_jw_t *w, const char *json, size_t n);

/* Convenience: key + value in one call. */
int mp_jw_kv_str(mp_jw_t *w, const char *key, const char *v);
int mp_jw_kv_i64(mp_jw_t *w, const char *key, int64_t v);
int mp_jw_kv_u64(mp_jw_t *w, const char *key, uint64_t v);
int mp_jw_kv_bool(mp_jw_t *w, const char *key, bool v);
int mp_jw_kv_f32(mp_jw_t *w, const char *key, float v, uint8_t decimals);
int mp_jw_kv_null(mp_jw_t *w, const char *key);

/**
 * Terminate the document.
 *
 * @param out_len  Optional; receives the length excluding the NUL.
 *
 * @retval 0        Complete and NUL-terminated.
 * @retval -EINVAL  @p w is NULL.
 * @retval -ENOSPC  The buffer overflowed at some point.
 * @retval -EPROTO  A container or a key was left open.
 */
int mp_jw_finish(mp_jw_t *w, size_t *out_len);

/* =========================================================== base64 ===== */

/** Encoded length of @p n bytes, excluding the NUL. */
#define MP_B64_LEN(n) ((((n) + 2U) / 3U) * 4U)

/**
 * Standard base64 with padding.
 *
 * @retval >=0      Characters written, excluding the NUL.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOSPC  @p cap too small (needs MP_B64_LEN(len) + 1).
 */
int mp_b64_encode(const uint8_t *in, size_t len, char *out, size_t cap);

/**
 * Decode standard base64. Whitespace is skipped; padding is optional but must
 * be consistent when present.
 *
 * @retval >=0      Bytes written.
 * @retval -EINVAL  Bad argument.
 * @retval -EBADMSG Invalid character or truncated group.
 * @retval -ENOSPC  @p cap too small.
 */
int mp_b64_decode(const char *in, size_t len, uint8_t *out, size_t cap);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_MP_MP_JSON_H_ */
