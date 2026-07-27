/*
 * STS1000 "Meridian" — core/mp: minimal allocation-free JSON codec + base64.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See mp_json.h for the contract. Both halves are iterative by design: the
 * input is attacker-controlled and a recursive descent would put the nesting
 * depth on the C stack.
 */

#include "mp/mp_json.h"

#include <errno.h>
#include <string.h>

/* ------------------------------------------------------------------ common */

static bool is_ws(char c)
{
	return (c == ' ') || (c == '\t') || (c == '\n') || (c == '\r');
}

static bool is_digit(char c)
{
	return (c >= '0') && (c <= '9');
}

/* =========================================================== parser ===== */

typedef struct {
	mp_json_t *p;
	size_t i;    /* cursor into src */
	int16_t cur; /* token index of the container being filled, -1 at root */
	uint8_t depth;
	bool root_done;
} pstate_t;

/** Allocate a token, wired to the current parent. */
static int tok_alloc(pstate_t *s, uint8_t type, size_t start)
{
	mp_json_tok_t *t;
	int idx;

	if (s->p->tok_n >= s->p->tok_cap) {
		return -ENOSPC;
	}
	idx = (int)s->p->tok_n;
	t = &s->p->tok[idx];
	s->p->tok_n++;

	t->type = type;
	t->depth = s->depth;
	t->parent = s->cur;
	t->start = (uint16_t)start;
	t->end = (uint16_t)start;
	t->size = 0U;

	if (s->cur >= 0) {
		s->p->tok[s->cur].size++;
	}
	return idx;
}

/** Scan a string literal starting at the opening quote. */
static int scan_string(pstate_t *s, size_t *out_end_quote)
{
	size_t i = s->i + 1U; /* past the opening quote */

	while (i < s->p->len) {
		char c = s->p->src[i];

		if (c == '"') {
			*out_end_quote = i;
			return 0;
		}
		if (c == '\\') {
			if ((i + 1U) >= s->p->len) {
				return -EBADMSG;
			}
			switch (s->p->src[i + 1U]) {
			case '"':
			case '\\':
			case '/':
			case 'b':
			case 'f':
			case 'n':
			case 'r':
			case 't':
				i += 2U;
				break;
			case 'u': {
				size_t k;

				if ((i + 5U) >= s->p->len) {
					return -EBADMSG;
				}
				for (k = 0U; k < 4U; k++) {
					char h = s->p->src[i + 2U + k];
					bool ok = is_digit(h) ||
						  ((h >= 'a') && (h <= 'f')) ||
						  ((h >= 'A') && (h <= 'F'));

					if (!ok) {
						return -EBADMSG;
					}
				}
				i += 6U;
				break;
			}
			default:
				return -EBADMSG;
			}
			continue;
		}
		/* RFC 8259 §7: raw control characters are not allowed. */
		if ((unsigned char)c < 0x20U) {
			return -EBADMSG;
		}
		i++;
	}
	return -EBADMSG;
}

/** Validate a JSON number literal and return its length. */
static int scan_number(const char *src, size_t len, size_t i, size_t *out_len)
{
	size_t start = i;
	bool any;

	if ((i < len) && (src[i] == '-')) {
		i++;
	}
	/* int part: 0 | [1-9][0-9]* */
	if ((i < len) && (src[i] == '0')) {
		i++;
	} else {
		any = false;
		while ((i < len) && is_digit(src[i])) {
			i++;
			any = true;
		}
		if (!any) {
			return -EBADMSG;
		}
	}
	if ((i < len) && (src[i] == '.')) {
		i++;
		any = false;
		while ((i < len) && is_digit(src[i])) {
			i++;
			any = true;
		}
		if (!any) {
			return -EBADMSG;
		}
	}
	if ((i < len) && ((src[i] == 'e') || (src[i] == 'E'))) {
		i++;
		if ((i < len) && ((src[i] == '+') || (src[i] == '-'))) {
			i++;
		}
		any = false;
		while ((i < len) && is_digit(src[i])) {
			i++;
			any = true;
		}
		if (!any) {
			return -EBADMSG;
		}
	}

	*out_len = i - start;
	return 0;
}

static int match_lit(const char *src, size_t len, size_t i, const char *lit)
{
	size_t n = strlen(lit);

	if ((len - i) < n) {
		return -EBADMSG;
	}
	if (memcmp(&src[i], lit, n) != 0) {
		return -EBADMSG;
	}
	return (int)n;
}

/**
 * Close the innermost container and step back out.
 *
 * The closing brace/bracket must match the container kind, or `{"a":1]` would
 * parse.
 */
static int close_container(pstate_t *s, uint8_t want)
{
	mp_json_tok_t *t;

	if (s->cur < 0) {
		return -EBADMSG;
	}
	t = &s->p->tok[s->cur];
	if (t->type != want) {
		return -EBADMSG;
	}
	t->end = (uint16_t)(s->i + 1U);
	s->cur = t->parent;
	s->depth--;
	if (s->cur < 0) {
		s->root_done = true;
	}
	return 0;
}

/**
 * Structural bookkeeping between values.
 *
 * Rather than a full grammar automaton this validates the two properties that
 * matter for safety and for rejecting nonsense: commas and colons appear only
 * where a container is open, and an object member is always `string : value`.
 */
typedef enum {
	EXP_VALUE = 0, /* a value (or a closing bracket) is next */
	EXP_KEY,       /* an object member name is next */
	EXP_COLON,
	EXP_COMMA_OR_END,
} expect_t;

int mp_json_parse(mp_json_t *p, const char *src, size_t len,
		  mp_json_tok_t *tok, uint16_t tok_cap, uint8_t depth_max)
{
	pstate_t s;
	expect_t exp = EXP_VALUE;

	if ((p == NULL) || (src == NULL) || (tok == NULL) || (tok_cap == 0U)) {
		return -EINVAL;
	}
	if (len >= 0xFFFFU) {
		return -EMSGSIZE;
	}
	if (depth_max == 0U) {
		depth_max = (uint8_t)MP_JSON_DEPTH_MAX;
	}

	p->src = src;
	p->len = len;
	p->tok = tok;
	p->tok_cap = tok_cap;
	p->tok_n = 0U;

	s.p = p;
	s.i = 0U;
	s.cur = -1;
	s.depth = 0U;
	s.root_done = false;

	while (s.i < len) {
		char c = src[s.i];
		int rc;

		if (is_ws(c)) {
			s.i++;
			continue;
		}

		if (s.root_done) {
			return -EBADMSG; /* trailing garbage */
		}

		switch (c) {
		case '{':
		case '[': {
			uint8_t kind = (c == '{') ? (uint8_t)MP_J_OBJ
						  : (uint8_t)MP_J_ARR;
			int idx;

			if (exp != EXP_VALUE) {
				return -EBADMSG;
			}
			if (s.depth >= depth_max) {
				return -E2BIG;
			}
			idx = tok_alloc(&s, kind, s.i);
			if (idx < 0) {
				return idx;
			}
			s.cur = (int16_t)idx;
			s.depth++;
			exp = (kind == (uint8_t)MP_J_OBJ) ? EXP_KEY : EXP_VALUE;
			s.i++;
			break;
		}

		case '}':
		case ']': {
			uint8_t want = (c == '}') ? (uint8_t)MP_J_OBJ
						  : (uint8_t)MP_J_ARR;

			/* An empty container closes from EXP_KEY/EXP_VALUE; a
			 * populated one from EXP_COMMA_OR_END. Anything else
			 * (after a comma, after a colon) is malformed. */
			if (exp == EXP_COLON) {
				return -EBADMSG;
			}
			if ((s.cur < 0) || (p->tok[s.cur].type != want)) {
				return -EBADMSG;
			}
			if ((exp != EXP_COMMA_OR_END) &&
			    (p->tok[s.cur].size != 0U)) {
				return -EBADMSG;
			}
			rc = close_container(&s, want);
			if (rc != 0) {
				return rc;
			}
			exp = EXP_COMMA_OR_END;
			s.i++;
			break;
		}

		case ',':
			if (exp != EXP_COMMA_OR_END) {
				return -EBADMSG;
			}
			if (s.cur < 0) {
				return -EBADMSG;
			}
			exp = (p->tok[s.cur].type == (uint8_t)MP_J_OBJ)
				      ? EXP_KEY
				      : EXP_VALUE;
			s.i++;
			break;

		case ':':
			if (exp != EXP_COLON) {
				return -EBADMSG;
			}
			exp = EXP_VALUE;
			s.i++;
			break;

		case '"': {
			size_t endq = 0U;
			int idx;

			if ((exp != EXP_VALUE) && (exp != EXP_KEY)) {
				return -EBADMSG;
			}
			rc = scan_string(&s, &endq);
			if (rc != 0) {
				return rc;
			}
			idx = tok_alloc(&s, (uint8_t)MP_J_STR, s.i + 1U);
			if (idx < 0) {
				return idx;
			}
			p->tok[idx].end = (uint16_t)endq;
			if (exp == EXP_KEY) {
				exp = EXP_COLON;
			} else {
				exp = EXP_COMMA_OR_END;
				if (s.cur < 0) {
					s.root_done = true;
				}
			}
			s.i = endq + 1U;
			break;
		}

		default: {
			int idx;
			size_t n = 0U;
			uint8_t type;

			if (exp != EXP_VALUE) {
				return -EBADMSG;
			}

			if ((c == '-') || is_digit(c)) {
				rc = scan_number(src, len, s.i, &n);
				if (rc != 0) {
					return rc;
				}
				type = (uint8_t)MP_J_NUM;
			} else if (c == 't') {
				rc = match_lit(src, len, s.i, "true");
				if (rc < 0) {
					return rc;
				}
				n = (size_t)rc;
				type = (uint8_t)MP_J_BOOL;
			} else if (c == 'f') {
				rc = match_lit(src, len, s.i, "false");
				if (rc < 0) {
					return rc;
				}
				n = (size_t)rc;
				type = (uint8_t)MP_J_BOOL;
			} else if (c == 'n') {
				rc = match_lit(src, len, s.i, "null");
				if (rc < 0) {
					return rc;
				}
				n = (size_t)rc;
				type = (uint8_t)MP_J_NULL;
			} else {
				return -EBADMSG;
			}

			idx = tok_alloc(&s, type, s.i);
			if (idx < 0) {
				return idx;
			}
			p->tok[idx].end = (uint16_t)(s.i + n);
			s.i += n;
			exp = EXP_COMMA_OR_END;
			if (s.cur < 0) {
				s.root_done = true;
			}
			break;
		}
		}
	}

	if (!s.root_done || (s.cur >= 0)) {
		return -EBADMSG; /* truncated */
	}
	return (int)p->tok_n;
}

/* ------------------------------------------------------------- accessors */

int mp_json_root(const mp_json_t *p)
{
	if ((p == NULL) || (p->tok_n == 0U)) {
		return -ENOENT;
	}
	return 0;
}

const mp_json_tok_t *mp_json_at(const mp_json_t *p, int i)
{
	if ((p == NULL) || (i < 0) || ((uint16_t)i >= p->tok_n)) {
		return NULL;
	}
	return &p->tok[i];
}

int mp_json_obj_get(const mp_json_t *p, int obj, const char *key)
{
	const mp_json_tok_t *o = mp_json_at(p, obj);
	uint16_t seen = 0U;
	int i;

	if ((o == NULL) || (key == NULL)) {
		return -EINVAL;
	}
	if (o->type != (uint8_t)MP_J_OBJ) {
		return -ENOTDIR;
	}

	for (i = obj + 1; ((uint16_t)i < p->tok_n) && (seen < o->size); i++) {
		if (p->tok[i].parent != (int16_t)obj) {
			continue;
		}
		seen++;
		if (p->tok[i].type != (uint8_t)MP_J_STR) {
			continue; /* cannot happen for a valid parse */
		}
		if (mp_json_streq(p, i, key)) {
			if ((uint16_t)(i + 1) >= p->tok_n) {
				return -EBADMSG;
			}
			return i + 1;
		}
	}
	return -ENOENT;
}

bool mp_json_has(const mp_json_t *p, int obj, const char *key)
{
	return mp_json_obj_get(p, obj, key) >= 0;
}

int mp_json_arr_at(const mp_json_t *p, int arr, uint16_t idx)
{
	const mp_json_tok_t *a = mp_json_at(p, arr);
	uint16_t seen = 0U;
	int i;

	if (a == NULL) {
		return -EINVAL;
	}
	if (a->type != (uint8_t)MP_J_ARR) {
		return -ENOTDIR;
	}
	if (idx >= a->size) {
		return -ENOENT;
	}

	for (i = arr + 1; (uint16_t)i < p->tok_n; i++) {
		if (p->tok[i].parent != (int16_t)arr) {
			continue;
		}
		if (seen == idx) {
			return i;
		}
		seen++;
	}
	return -ENOENT;
}

uint16_t mp_json_count(const mp_json_t *p, int i)
{
	const mp_json_tok_t *t = mp_json_at(p, i);

	if (t == NULL) {
		return 0U;
	}
	if ((t->type != (uint8_t)MP_J_OBJ) && (t->type != (uint8_t)MP_J_ARR)) {
		return 0U;
	}
	return t->size;
}

/* -------------------------------------------------------------- strings */

static int hex4(const char *s, uint32_t *out)
{
	uint32_t v = 0U;
	size_t k;

	for (k = 0U; k < 4U; k++) {
		char c = s[k];
		uint32_t d;

		if (is_digit(c)) {
			d = (uint32_t)(c - '0');
		} else if ((c >= 'a') && (c <= 'f')) {
			d = (uint32_t)(c - 'a') + 10U;
		} else if ((c >= 'A') && (c <= 'F')) {
			d = (uint32_t)(c - 'A') + 10U;
		} else {
			return -EBADMSG;
		}
		v = (v << 4) | d;
	}
	*out = v;
	return 0;
}

/** Append @p cp as UTF-8. Returns bytes written or -ENOSPC. */
static int utf8_put(char *out, size_t cap, size_t at, uint32_t cp)
{
	if (cp < 0x80U) {
		if ((at + 1U) > cap) {
			return -ENOSPC;
		}
		out[at] = (char)cp;
		return 1;
	}
	if (cp < 0x800U) {
		if ((at + 2U) > cap) {
			return -ENOSPC;
		}
		out[at] = (char)(0xC0U | (cp >> 6));
		out[at + 1U] = (char)(0x80U | (cp & 0x3FU));
		return 2;
	}
	if (cp < 0x10000U) {
		if ((at + 3U) > cap) {
			return -ENOSPC;
		}
		out[at] = (char)(0xE0U | (cp >> 12));
		out[at + 1U] = (char)(0x80U | ((cp >> 6) & 0x3FU));
		out[at + 2U] = (char)(0x80U | (cp & 0x3FU));
		return 3;
	}
	if ((at + 4U) > cap) {
		return -ENOSPC;
	}
	out[at] = (char)(0xF0U | (cp >> 18));
	out[at + 1U] = (char)(0x80U | ((cp >> 12) & 0x3FU));
	out[at + 2U] = (char)(0x80U | ((cp >> 6) & 0x3FU));
	out[at + 3U] = (char)(0x80U | (cp & 0x3FU));
	return 4;
}

int mp_json_str(const mp_json_t *p, int i, char *out, size_t cap)
{
	const mp_json_tok_t *t = mp_json_at(p, i);
	size_t r;
	size_t w = 0U;

	if ((t == NULL) || (out == NULL) || (cap == 0U)) {
		return -EINVAL;
	}
	if (t->type != (uint8_t)MP_J_STR) {
		return -ENOTDIR;
	}

	for (r = t->start; r < t->end; r++) {
		char c = p->src[r];
		char lit;

		if (c != '\\') {
			if ((w + 1U) >= cap) {
				return -ENOSPC;
			}
			out[w++] = c;
			continue;
		}

		r++;
		if (r >= t->end) {
			return -EBADMSG;
		}

		switch (p->src[r]) {
		case '"':
			lit = '"';
			break;
		case '\\':
			lit = '\\';
			break;
		case '/':
			lit = '/';
			break;
		case 'b':
			lit = '\b';
			break;
		case 'f':
			lit = '\f';
			break;
		case 'n':
			lit = '\n';
			break;
		case 'r':
			lit = '\r';
			break;
		case 't':
			lit = '\t';
			break;
		case 'u': {
			uint32_t cp = 0U;
			int n;

			/* r indexes the 'u'; the four hex digits are r+1..r+4.
			 * The scanner already guarantees they are inside the
			 * literal — checked again so this function is safe on a
			 * hand-built token, which the fuzz tests do use. */
			if ((r + 4U) >= (size_t)t->end) {
				return -EBADMSG;
			}
			if (hex4(&p->src[r + 1U], &cp) != 0) {
				return -EBADMSG;
			}
			r += 4U;

			if ((cp >= 0xD800U) && (cp <= 0xDBFFU)) {
				/* High surrogate: pair it if the low half is
				 * present, else emit the replacement char. */
				/* r now indexes the last hex digit; a paired low
				 * surrogate would occupy r+1..r+6 ("\uXXXX"). */
				if (((r + 6U) < (size_t)t->end) &&
				    (p->src[r + 1U] == '\\') &&
				    (p->src[r + 2U] == 'u')) {
					uint32_t lo = 0U;

					if (hex4(&p->src[r + 3U], &lo) == 0) {
						if ((lo >= 0xDC00U) &&
						    (lo <= 0xDFFFU)) {
							cp = 0x10000U +
							     ((cp - 0xD800U)
							      << 10) +
							     (lo - 0xDC00U);
							r += 6U;
						} else {
							cp = 0xFFFDU;
						}
					} else {
						cp = 0xFFFDU;
					}
				} else {
					cp = 0xFFFDU;
				}
			} else if ((cp >= 0xDC00U) && (cp <= 0xDFFFU)) {
				cp = 0xFFFDU; /* lone low surrogate */
			}

			n = utf8_put(out, cap - 1U, w, cp);
			if (n < 0) {
				return n;
			}
			w += (size_t)n;
			continue;
		}
		default:
			return -EBADMSG;
		}

		if ((w + 1U) >= cap) {
			return -ENOSPC;
		}
		out[w++] = lit;
	}

	out[w] = '\0';
	return (int)w;
}

bool mp_json_streq(const mp_json_t *p, int i, const char *s)
{
	const mp_json_tok_t *t = mp_json_at(p, i);
	size_t r;
	size_t k = 0U;

	if ((t == NULL) || (s == NULL)) {
		return false;
	}
	if (t->type != (uint8_t)MP_J_STR) {
		return false;
	}

	/*
	 * Compared escape-aware but without a scratch buffer: control-plane
	 * keys and method names are ASCII, so only the simple escapes can
	 * appear, and a \u escape in a key never matches a plain-ASCII name.
	 */
	for (r = t->start; r < t->end; r++) {
		char c = p->src[r];

		if (c == '\\') {
			r++;
			if (r >= t->end) {
				return false;
			}
			switch (p->src[r]) {
			case '"':
				c = '"';
				break;
			case '\\':
				c = '\\';
				break;
			case '/':
				c = '/';
				break;
			case 'b':
				c = '\b';
				break;
			case 'f':
				c = '\f';
				break;
			case 'n':
				c = '\n';
				break;
			case 'r':
				c = '\r';
				break;
			case 't':
				c = '\t';
				break;
			default:
				return false; /* \u: never an ASCII key match */
			}
		}
		if (s[k] != c) {
			return false;
		}
		k++;
	}
	return s[k] == '\0';
}

/* -------------------------------------------------------------- numbers */

int mp_json_i64(const mp_json_t *p, int i, int64_t *out)
{
	const mp_json_tok_t *t = mp_json_at(p, i);
	size_t r;
	bool neg = false;
	uint64_t mag = 0U;
	bool any = false;

	if ((t == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (t->type != (uint8_t)MP_J_NUM) {
		return -ENOTDIR;
	}

	r = t->start;
	if ((r < t->end) && (p->src[r] == '-')) {
		neg = true;
		r++;
	}
	for (; r < t->end; r++) {
		char c = p->src[r];

		if (!is_digit(c)) {
			return -EBADMSG; /* '.', 'e' — not an integer literal */
		}
		if (mag > ((uint64_t)0x7FFFFFFFFFFFFFFFULL / 10U)) {
			return -ERANGE;
		}
		mag = (mag * 10U) + (uint64_t)(c - '0');
		any = true;
		if (!neg && (mag > (uint64_t)0x7FFFFFFFFFFFFFFFULL)) {
			return -ERANGE;
		}
		if (neg && (mag > (uint64_t)0x8000000000000000ULL)) {
			return -ERANGE;
		}
	}
	if (!any) {
		return -EBADMSG;
	}

	if (neg) {
		if (mag == (uint64_t)0x8000000000000000ULL) {
			*out = (int64_t)(-0x7FFFFFFFFFFFFFFFLL) - 1LL;
		} else {
			*out = -(int64_t)mag;
		}
	} else {
		*out = (int64_t)mag;
	}
	return 0;
}

int mp_json_i32_range(const mp_json_t *p, int i, int32_t lo, int32_t hi,
		      int32_t *out)
{
	int64_t v = 0;
	int rc;

	if (out == NULL) {
		return -EINVAL;
	}
	rc = mp_json_i64(p, i, &v);
	if (rc != 0) {
		return rc;
	}
	if ((v < (int64_t)lo) || (v > (int64_t)hi)) {
		return -ERANGE;
	}
	*out = (int32_t)v;
	return 0;
}

/** 10^n for n in 0..38, computed without libm. */
static float pow10f_(int n)
{
	float r = 1.0f;
	int k;
	bool inv = false;

	if (n < 0) {
		inv = true;
		n = -n;
	}
	if (n > 38) {
		n = 38;
	}
	for (k = 0; k < n; k++) {
		r *= 10.0f;
	}
	return inv ? (1.0f / r) : r;
}

int mp_json_f32(const mp_json_t *p, int i, float *out)
{
	const mp_json_tok_t *t = mp_json_at(p, i);
	size_t r;
	bool neg = false;
	uint64_t mant = 0U;
	int exp10 = 0;
	bool any = false;

	if ((t == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (t->type != (uint8_t)MP_J_NUM) {
		return -ENOTDIR;
	}

	r = t->start;
	if ((r < t->end) && (p->src[r] == '-')) {
		neg = true;
		r++;
	}
	while ((r < t->end) && is_digit(p->src[r])) {
		if (mant < 1000000000000000000ULL) {
			mant = (mant * 10U) + (uint64_t)(p->src[r] - '0');
		} else {
			exp10++; /* saturate the mantissa, keep the scale */
		}
		any = true;
		r++;
	}
	if ((r < t->end) && (p->src[r] == '.')) {
		r++;
		while ((r < t->end) && is_digit(p->src[r])) {
			if (mant < 1000000000000000000ULL) {
				mant = (mant * 10U) +
				       (uint64_t)(p->src[r] - '0');
				exp10--;
			}
			any = true;
			r++;
		}
	}
	if (!any) {
		return -EBADMSG;
	}
	if ((r < t->end) && ((p->src[r] == 'e') || (p->src[r] == 'E'))) {
		bool eneg = false;
		int e = 0;

		r++;
		if (r < t->end) {
			if (p->src[r] == '-') {
				eneg = true;
				r++;
			} else if (p->src[r] == '+') {
				r++;
			}
		}
		while ((r < t->end) && is_digit(p->src[r])) {
			if (e < 1000) {
				e = (e * 10) + (p->src[r] - '0');
			}
			r++;
		}
		exp10 += eneg ? -e : e;
	}

	{
		float v = (float)mant * pow10f_(exp10);

		*out = neg ? -v : v;
	}
	return 0;
}

int mp_json_bool(const mp_json_t *p, int i, bool *out)
{
	const mp_json_tok_t *t = mp_json_at(p, i);

	if ((t == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (t->type != (uint8_t)MP_J_BOOL) {
		return -ENOTDIR;
	}
	*out = (p->src[t->start] == 't');
	return 0;
}

/* =========================================================== writer ===== */

int mp_jw_init(mp_jw_t *w, char *buf, size_t cap)
{
	if ((w == NULL) || (buf == NULL) || (cap == 0U)) {
		return -EINVAL;
	}
	(void)memset(w, 0, sizeof(*w));
	w->buf = buf;
	w->cap = cap;
	buf[0] = '\0';
	return 0;
}

/** Append raw bytes, latching -ENOSPC. One byte is always reserved for the NUL. */
static void put(mp_jw_t *w, const char *s, size_t n)
{
	if (w->err != 0) {
		return;
	}
	if ((w->len + n) >= w->cap) {
		w->err = -ENOSPC;
		return;
	}
	(void)memcpy(&w->buf[w->len], s, n);
	w->len += n;
}

static void putc_(mp_jw_t *w, char c)
{
	put(w, &c, 1U);
}

/** Emit the separator owed before the next value at the current level. */
static void sep_before_value(mp_jw_t *w)
{
	if (w->err != 0) {
		return;
	}
	if (w->want_value) {
		w->want_value = false;
		return; /* the colon was already written by mp_jw_key() */
	}
	if (w->depth == 0U) {
		return;
	}
	if (w->kind[w->depth - 1U] != 0U) {
		/* Inside an object, a bare value with no key is a programming
		 * error, not a wire error. Latch it so mp_jw_finish() reports. */
		w->err = -EPROTO;
		return;
	}
	if (w->sep[w->depth - 1U]) {
		putc_(w, ',');
	}
	w->sep[w->depth - 1U] = true;
}

static void after_value(mp_jw_t *w)
{
	if ((w->depth > 0U) && (w->kind[w->depth - 1U] != 0U)) {
		w->sep[w->depth - 1U] = true;
	}
}

static int container_open(mp_jw_t *w, char c, uint8_t kind)
{
	if (w == NULL) {
		return -EINVAL;
	}
	sep_before_value(w);
	if (w->depth >= (uint8_t)MP_JW_DEPTH_MAX) {
		if (w->err == 0) {
			w->err = -E2BIG;
		}
		return w->err;
	}
	putc_(w, c);
	w->kind[w->depth] = kind;
	w->sep[w->depth] = false;
	w->depth++;
	return w->err;
}

static int container_close(mp_jw_t *w, char c, uint8_t kind)
{
	if (w == NULL) {
		return -EINVAL;
	}
	if (w->err != 0) {
		return w->err;
	}
	if ((w->depth == 0U) || (w->kind[w->depth - 1U] != kind) ||
	    w->want_value) {
		w->err = -EPROTO;
		return w->err;
	}
	w->depth--;
	putc_(w, c);
	after_value(w);
	return w->err;
}

int mp_jw_obj_open(mp_jw_t *w)
{
	return container_open(w, '{', 1U);
}

int mp_jw_obj_close(mp_jw_t *w)
{
	return container_close(w, '}', 1U);
}

int mp_jw_arr_open(mp_jw_t *w)
{
	return container_open(w, '[', 0U);
}

int mp_jw_arr_close(mp_jw_t *w)
{
	return container_close(w, ']', 0U);
}

/** Write a JSON string literal, escaping per RFC 8259 §7. */
static void put_quoted(mp_jw_t *w, const char *s, size_t n)
{
	size_t i;

	putc_(w, '"');
	for (i = 0U; i < n; i++) {
		unsigned char c = (unsigned char)s[i];

		switch (c) {
		case '"':
			put(w, "\\\"", 2U);
			break;
		case '\\':
			put(w, "\\\\", 2U);
			break;
		case '\b':
			put(w, "\\b", 2U);
			break;
		case '\f':
			put(w, "\\f", 2U);
			break;
		case '\n':
			put(w, "\\n", 2U);
			break;
		case '\r':
			put(w, "\\r", 2U);
			break;
		case '\t':
			put(w, "\\t", 2U);
			break;
		default:
			if (c < 0x20U) {
				static const char hexd[] = "0123456789abcdef";
				char esc[6];

				esc[0] = '\\';
				esc[1] = 'u';
				esc[2] = '0';
				esc[3] = '0';
				esc[4] = hexd[(c >> 4) & 0x0FU];
				esc[5] = hexd[c & 0x0FU];
				put(w, esc, sizeof(esc));
			} else {
				put(w, (const char *)&s[i], 1U);
			}
			break;
		}
	}
	putc_(w, '"');
}

int mp_jw_key(mp_jw_t *w, const char *key)
{
	if ((w == NULL) || (key == NULL)) {
		return -EINVAL;
	}
	if (w->err != 0) {
		return w->err;
	}
	if ((w->depth == 0U) || (w->kind[w->depth - 1U] == 0U) ||
	    w->want_value) {
		w->err = -EPROTO;
		return w->err;
	}
	if (w->sep[w->depth - 1U]) {
		putc_(w, ',');
	}
	put_quoted(w, key, strlen(key));
	putc_(w, ':');
	w->want_value = true;
	return w->err;
}

int mp_jw_strn(mp_jw_t *w, const char *s, size_t n)
{
	if (w == NULL) {
		return -EINVAL;
	}
	sep_before_value(w);
	if (s == NULL) {
		put(w, "null", 4U);
	} else {
		put_quoted(w, s, n);
	}
	after_value(w);
	return w->err;
}

int mp_jw_str(mp_jw_t *w, const char *s)
{
	return mp_jw_strn(w, s, (s != NULL) ? strlen(s) : 0U);
}

/** Render an unsigned value into @p out (max 20 digits); returns the length. */
static size_t u64_dec(uint64_t v, char *out)
{
	char tmp[20];
	size_t n = 0U;
	size_t i;

	do {
		tmp[n++] = (char)('0' + (char)(v % 10U));
		v /= 10U;
	} while (v != 0U);

	for (i = 0U; i < n; i++) {
		out[i] = tmp[n - 1U - i];
	}
	return n;
}

int mp_jw_u64(mp_jw_t *w, uint64_t v)
{
	char d[20];
	size_t n;

	if (w == NULL) {
		return -EINVAL;
	}
	sep_before_value(w);
	n = u64_dec(v, d);
	put(w, d, n);
	after_value(w);
	return w->err;
}

int mp_jw_i64(mp_jw_t *w, int64_t v)
{
	char d[21];
	size_t n = 0U;
	uint64_t mag;

	if (w == NULL) {
		return -EINVAL;
	}
	sep_before_value(w);
	if (v < 0) {
		d[0] = '-';
		n = 1U;
		mag = (v == (int64_t)(-0x7FFFFFFFFFFFFFFFLL) - 1LL)
			      ? (uint64_t)0x8000000000000000ULL
			      : (uint64_t)(-v);
	} else {
		mag = (uint64_t)v;
	}
	n += u64_dec(mag, &d[n]);
	put(w, d, n);
	after_value(w);
	return w->err;
}

int mp_jw_bool(mp_jw_t *w, bool v)
{
	if (w == NULL) {
		return -EINVAL;
	}
	sep_before_value(w);
	put(w, v ? "true" : "false", v ? 4U : 5U);
	after_value(w);
	return w->err;
}

int mp_jw_null(mp_jw_t *w)
{
	if (w == NULL) {
		return -EINVAL;
	}
	sep_before_value(w);
	put(w, "null", 4U);
	after_value(w);
	return w->err;
}

int mp_jw_f32(mp_jw_t *w, float v, uint8_t decimals)
{
	char d[40];
	size_t n = 0U;
	float scale;
	float a;
	uint64_t scaled;
	uint64_t ip;
	uint64_t fp;

	if (w == NULL) {
		return -EINVAL;
	}
	if (decimals > 6U) {
		decimals = 6U;
	}

	/* NaN / infinity have no JSON form (see the header). The NaN test is
	 * self-comparison so no libm and no -ffast-math surprise. */
	if (v != v) {
		return mp_jw_null(w);
	}
	a = (v < 0.0f) ? -v : v;
	if (a > 1.0e18f) {
		return mp_jw_null(w);
	}

	sep_before_value(w);

	if (v < 0.0f) {
		d[n++] = '-';
	}

	scale = pow10f_((int)decimals);
	scaled = (uint64_t)((a * scale) + 0.5f);
	ip = scaled / (uint64_t)scale;
	fp = scaled - (ip * (uint64_t)scale);

	n += u64_dec(ip, &d[n]);
	if (decimals > 0U) {
		char fbuf[8];
		size_t fn;
		size_t pad;

		d[n++] = '.';
		fn = u64_dec(fp, fbuf);
		for (pad = fn; pad < (size_t)decimals; pad++) {
			d[n++] = '0';
		}
		(void)memcpy(&d[n], fbuf, fn);
		n += fn;
	}

	put(w, d, n);
	after_value(w);
	return w->err;
}

int mp_jw_raw(mp_jw_t *w, const char *json, size_t n)
{
	if (w == NULL) {
		return -EINVAL;
	}
	if ((json == NULL) && (n != 0U)) {
		return -EINVAL;
	}
	sep_before_value(w);
	put(w, json, n);
	after_value(w);
	return w->err;
}

int mp_jw_kv_str(mp_jw_t *w, const char *key, const char *v)
{
	(void)mp_jw_key(w, key);
	return mp_jw_str(w, v);
}

int mp_jw_kv_i64(mp_jw_t *w, const char *key, int64_t v)
{
	(void)mp_jw_key(w, key);
	return mp_jw_i64(w, v);
}

int mp_jw_kv_u64(mp_jw_t *w, const char *key, uint64_t v)
{
	(void)mp_jw_key(w, key);
	return mp_jw_u64(w, v);
}

int mp_jw_kv_bool(mp_jw_t *w, const char *key, bool v)
{
	(void)mp_jw_key(w, key);
	return mp_jw_bool(w, v);
}

int mp_jw_kv_f32(mp_jw_t *w, const char *key, float v, uint8_t decimals)
{
	(void)mp_jw_key(w, key);
	return mp_jw_f32(w, v, decimals);
}

int mp_jw_kv_null(mp_jw_t *w, const char *key)
{
	(void)mp_jw_key(w, key);
	return mp_jw_null(w);
}

int mp_jw_finish(mp_jw_t *w, size_t *out_len)
{
	if (w == NULL) {
		return -EINVAL;
	}
	if (w->err == 0) {
		if ((w->depth != 0U) || w->want_value) {
			w->err = -EPROTO;
		}
	}
	w->buf[w->len] = '\0';
	if (out_len != NULL) {
		*out_len = w->len;
	}
	return w->err;
}

/* =========================================================== base64 ===== */

static const char b64_alpha[] =
	"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

int mp_b64_encode(const uint8_t *in, size_t len, char *out, size_t cap)
{
	size_t i = 0U;
	size_t w = 0U;
	size_t need = MP_B64_LEN(len);

	if ((out == NULL) || ((in == NULL) && (len != 0U))) {
		return -EINVAL;
	}
	if (cap < (need + 1U)) {
		return -ENOSPC;
	}

	while ((i + 3U) <= len) {
		uint32_t v = ((uint32_t)in[i] << 16) |
			     ((uint32_t)in[i + 1U] << 8) | (uint32_t)in[i + 2U];

		out[w++] = b64_alpha[(v >> 18) & 0x3FU];
		out[w++] = b64_alpha[(v >> 12) & 0x3FU];
		out[w++] = b64_alpha[(v >> 6) & 0x3FU];
		out[w++] = b64_alpha[v & 0x3FU];
		i += 3U;
	}

	if ((len - i) == 1U) {
		uint32_t v = (uint32_t)in[i] << 16;

		out[w++] = b64_alpha[(v >> 18) & 0x3FU];
		out[w++] = b64_alpha[(v >> 12) & 0x3FU];
		out[w++] = '=';
		out[w++] = '=';
	} else if ((len - i) == 2U) {
		uint32_t v = ((uint32_t)in[i] << 16) |
			     ((uint32_t)in[i + 1U] << 8);

		out[w++] = b64_alpha[(v >> 18) & 0x3FU];
		out[w++] = b64_alpha[(v >> 12) & 0x3FU];
		out[w++] = b64_alpha[(v >> 6) & 0x3FU];
		out[w++] = '=';
	}

	out[w] = '\0';
	return (int)w;
}

static int b64_val(char c)
{
	if ((c >= 'A') && (c <= 'Z')) {
		return c - 'A';
	}
	if ((c >= 'a') && (c <= 'z')) {
		return (c - 'a') + 26;
	}
	if ((c >= '0') && (c <= '9')) {
		return (c - '0') + 52;
	}
	if (c == '+') {
		return 62;
	}
	if (c == '/') {
		return 63;
	}
	return -1;
}

int mp_b64_decode(const char *in, size_t len, uint8_t *out, size_t cap)
{
	uint32_t acc = 0U;
	unsigned int nbits = 0U;
	size_t w = 0U;
	size_t i;
	unsigned int pad = 0U;

	if ((in == NULL) || ((out == NULL) && (cap != 0U))) {
		return -EINVAL;
	}

	for (i = 0U; i < len; i++) {
		char c = in[i];
		int v;

		if (is_ws(c)) {
			continue;
		}
		if (c == '=') {
			pad++;
			if (pad > 2U) {
				return -EBADMSG;
			}
			continue;
		}
		if (pad != 0U) {
			return -EBADMSG; /* data after padding */
		}
		v = b64_val(c);
		if (v < 0) {
			return -EBADMSG;
		}
		acc = (acc << 6) | (uint32_t)v;
		nbits += 6U;
		if (nbits >= 8U) {
			nbits -= 8U;
			if (w >= cap) {
				return -ENOSPC;
			}
			out[w++] = (uint8_t)((acc >> nbits) & 0xFFU);
		}
	}

	/* A group must be complete: 6 leftover bits is a truncated quantum. */
	if (nbits >= 6U) {
		return -EBADMSG;
	}
	if ((acc & ((1U << nbits) - 1U)) != 0U) {
		return -EBADMSG; /* non-zero pad bits */
	}
	return (int)w;
}
