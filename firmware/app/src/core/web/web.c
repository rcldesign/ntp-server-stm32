/*
 * STS1000 "Meridian" — core/web: shared helpers, JSON writer/reader, codecs.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See web.h for the contract. Everything here is pure: no state outside the
 * caller's structs, no allocation, no platform headers.
 */

#include "web/web.h"

#include <errno.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/* methods and roles                                                         */
/* ------------------------------------------------------------------------- */

static const char *const method_names[WEB_METHOD_COUNT] = {
	"GET", "HEAD", "POST", "PUT", "DELETE", "OPTIONS",
};

const char *web_method_name(uint8_t m)
{
	if (m >= WEB_METHOD_COUNT) {
		return "?";
	}
	return method_names[m];
}

uint8_t web_method_parse(const char *p, size_t n)
{
	size_t i;

	if (p == NULL || n == 0U) {
		return (uint8_t)WEB_METHOD_UNKNOWN;
	}
	for (i = 0U; i < WEB_METHOD_COUNT; i++) {
		if (web_span_eq(p, n, method_names[i])) {
			return (uint8_t)i;
		}
	}
	return (uint8_t)WEB_METHOD_UNKNOWN;
}

bool web_method_is_mutating(uint8_t m)
{
	return (m == (uint8_t)WEB_METHOD_POST) || (m == (uint8_t)WEB_METHOD_PUT) ||
	       (m == (uint8_t)WEB_METHOD_DELETE);
}

const char *web_role_name(uint8_t r)
{
	switch (r) {
	case WEB_ROLE_VIEWER:
		return "viewer";
	case WEB_ROLE_OPERATOR:
		return "operator";
	case WEB_ROLE_ADMIN:
		return "admin";
	default:
		return "none";
	}
}

int web_role_parse(const char *p, size_t n, uint8_t *out)
{
	if (p == NULL || out == NULL) {
		return -EINVAL;
	}
	if (web_span_eq(p, n, "viewer")) {
		*out = (uint8_t)WEB_ROLE_VIEWER;
	} else if (web_span_eq(p, n, "operator")) {
		*out = (uint8_t)WEB_ROLE_OPERATOR;
	} else if (web_span_eq(p, n, "admin")) {
		*out = (uint8_t)WEB_ROLE_ADMIN;
	} else {
		return -ENOENT;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/* span helpers                                                              */
/* ------------------------------------------------------------------------- */

static char ascii_lower(char c)
{
	if (c >= 'A' && c <= 'Z') {
		return (char)(c + ('a' - 'A'));
	}
	return c;
}

bool web_span_eq_ci(const char *p, size_t n, const char *lit)
{
	size_t i;

	if (p == NULL || lit == NULL) {
		return false;
	}
	for (i = 0U; i < n; i++) {
		if (lit[i] == '\0') {
			return false;
		}
		if (ascii_lower(p[i]) != ascii_lower(lit[i])) {
			return false;
		}
	}
	return lit[n] == '\0';
}

bool web_span_eq(const char *p, size_t n, const char *lit)
{
	if (p == NULL || lit == NULL) {
		return false;
	}
	if (strlen(lit) != n) {
		return false;
	}
	return memcmp(p, lit, n) == 0;
}

size_t web_span_copy(char *out, size_t cap, const char *p, size_t n)
{
	size_t k;

	if (out == NULL || cap == 0U) {
		return 0U;
	}
	k = n;
	if (k > (cap - 1U)) {
		k = cap - 1U;
	}
	if (k > 0U && p != NULL) {
		memcpy(out, p, k);
	} else {
		k = 0U;
	}
	out[k] = '\0';
	return k;
}

/* ------------------------------------------------------------------------- */
/* constant-time compare                                                     */
/* ------------------------------------------------------------------------- */

int web_ct_memcmp(const void *a, const void *b, size_t n)
{
	const uint8_t *x = a;
	const uint8_t *y = b;
	uint8_t acc = 0U;
	size_t i;

	if (a == NULL || b == NULL) {
		return 1;
	}
	for (i = 0U; i < n; i++) {
		acc |= (uint8_t)(x[i] ^ y[i]);
	}
	return (acc == 0U) ? 0 : 1;
}

int web_ct_streq(const char *a, const char *b, size_t max)
{
	uint8_t acc = 0U;
	size_t i;
	bool ended = false;

	if (a == NULL || b == NULL) {
		return 1;
	}
	/*
	 * Walk the full window either way, so the timing reveals neither the
	 * common prefix length nor the strings' lengths. Once both have ended,
	 * subsequent bytes are compared as 0 == 0 and contribute nothing.
	 */
	for (i = 0U; i < max; i++) {
		char ca = ended ? '\0' : a[i];
		char cb = ended ? '\0' : b[i];

		acc |= (uint8_t)(ca ^ cb);
		if (ca == '\0' && cb == '\0') {
			ended = true;
		}
	}
	return (acc == 0U) ? 0 : 1;
}

/* ------------------------------------------------------------------------- */
/* JSON writer                                                               */
/* ------------------------------------------------------------------------- */

void web_jw_init(web_jw_t *w, char *buf, size_t cap)
{
	if (w == NULL) {
		return;
	}
	memset(w, 0, sizeof(*w));
	w->buf = buf;
	w->cap = cap;
	if (buf == NULL || cap == 0U) {
		w->err = true;
		w->cap = 0U;
		return;
	}
	buf[0] = '\0';
}

/* Append raw bytes. Reserves one byte for the terminating NUL at all times. */
static void jw_raw(web_jw_t *w, const char *s, size_t n)
{
	if (w == NULL || w->overflow || w->err) {
		return;
	}
	if (w->cap == 0U) {
		w->overflow = true;
		return;
	}
	if (n > ((w->cap - 1U) - w->len)) {
		w->overflow = true;
		return;
	}
	memcpy(&w->buf[w->len], s, n);
	w->len += n;
	w->buf[w->len] = '\0';
}

static void jw_ch(web_jw_t *w, char c)
{
	jw_raw(w, &c, 1U);
}

/* Emit the separator a new value needs at the current depth. */
static void jw_sep(web_jw_t *w)
{
	if (w == NULL || w->err) {
		return;
	}
	if (w->depth > 0U) {
		if (w->nonempty[w->depth - 1U]) {
			jw_ch(w, ',');
		}
		w->nonempty[w->depth - 1U] = true;
	}
}

/*
 * A key emits its own separator and then suppresses the value's, which is how
 * `{"a":1,"b":2}` gets exactly one comma per member.
 */
static bool jw_key_pending(web_jw_t *w)
{
	return (w->len > 0U) && (w->buf[w->len - 1U] == ':');
}

static void jw_value_sep(web_jw_t *w)
{
	if (w == NULL || w->err) {
		return;
	}
	if (jw_key_pending(w)) {
		return;
	}
	jw_sep(w);
}

static void jw_push(web_jw_t *w, char open)
{
	if (w == NULL || w->err) {
		return;
	}
	if (w->depth >= WEB_JSON_DEPTH_MAX) {
		w->err = true;
		return;
	}
	jw_value_sep(w);
	jw_ch(w, open);
	w->nonempty[w->depth] = false;
	w->depth++;
}

static void jw_pop(web_jw_t *w, char close)
{
	if (w == NULL || w->err) {
		return;
	}
	if (w->depth == 0U) {
		w->err = true;
		return;
	}
	w->depth--;
	jw_ch(w, close);
}

void web_jw_obj_begin(web_jw_t *w)
{
	jw_push(w, '{');
}

void web_jw_obj_end(web_jw_t *w)
{
	jw_pop(w, '}');
}

void web_jw_arr_begin(web_jw_t *w)
{
	jw_push(w, '[');
}

void web_jw_arr_end(web_jw_t *w)
{
	jw_pop(w, ']');
}

/* Escape and emit a JSON string body (no surrounding quotes). */
static void jw_escaped(web_jw_t *w, const char *s, size_t n)
{
	static const char hexd[] = "0123456789abcdef";
	size_t i;

	for (i = 0U; i < n; i++) {
		unsigned char c = (unsigned char)s[i];

		switch (c) {
		case '"':
			jw_raw(w, "\\\"", 2U);
			continue;
		case '\\':
			jw_raw(w, "\\\\", 2U);
			continue;
		case '\n':
			jw_raw(w, "\\n", 2U);
			continue;
		case '\r':
			jw_raw(w, "\\r", 2U);
			continue;
		case '\t':
			jw_raw(w, "\\t", 2U);
			continue;
		case '\b':
			jw_raw(w, "\\b", 2U);
			continue;
		case '\f':
			jw_raw(w, "\\f", 2U);
			continue;
		default:
			break;
		}
		if (c < 0x20U || c == 0x7FU) {
			char esc[6];

			esc[0] = '\\';
			esc[1] = 'u';
			esc[2] = '0';
			esc[3] = '0';
			esc[4] = hexd[(c >> 4) & 0x0FU];
			esc[5] = hexd[c & 0x0FU];
			jw_raw(w, esc, sizeof(esc));
			continue;
		}
		jw_ch(w, (char)c);
	}
}

void web_jw_key(web_jw_t *w, const char *key)
{
	if (w == NULL || w->err) {
		return;
	}
	if (key == NULL) {
		w->err = true;
		return;
	}
	if (jw_key_pending(w)) {
		/* Two keys in a row: a caller bug, not bad input. */
		w->err = true;
		return;
	}
	jw_sep(w);
	jw_ch(w, '"');
	jw_escaped(w, key, strlen(key));
	jw_raw(w, "\":", 2U);
}

void web_jw_strn(web_jw_t *w, const char *s, size_t n)
{
	jw_value_sep(w);
	jw_ch(w, '"');
	if (s != NULL) {
		jw_escaped(w, s, n);
	}
	jw_ch(w, '"');
}

void web_jw_str(web_jw_t *w, const char *s)
{
	web_jw_strn(w, s, (s == NULL) ? 0U : strlen(s));
}

/* Render an unsigned magnitude; returns the digit count written to buf. */
static size_t u64_digits(uint64_t v, char *buf)
{
	char tmp[20];
	size_t n = 0U;
	size_t i;

	do {
		tmp[n++] = (char)('0' + (unsigned int)(v % 10U));
		v /= 10U;
	} while (v != 0U);

	for (i = 0U; i < n; i++) {
		buf[i] = tmp[n - 1U - i];
	}
	return n;
}

void web_jw_u64(web_jw_t *w, uint64_t v)
{
	char d[20];
	size_t n = u64_digits(v, d);

	jw_value_sep(w);
	jw_raw(w, d, n);
}

void web_jw_i64(web_jw_t *w, int64_t v)
{
	char d[21];
	size_t n = 0U;
	uint64_t mag;

	if (v < 0) {
		d[0] = '-';
		n = 1U;
		/* Negate in unsigned space so INT64_MIN is representable. */
		mag = ~(uint64_t)v + 1U;
	} else {
		mag = (uint64_t)v;
	}
	n += u64_digits(mag, &d[n]);
	jw_value_sep(w);
	jw_raw(w, d, n);
}

void web_jw_bool(web_jw_t *w, bool v)
{
	jw_value_sep(w);
	if (v) {
		jw_raw(w, "true", 4U);
	} else {
		jw_raw(w, "false", 5U);
	}
}

void web_jw_null(web_jw_t *w)
{
	jw_value_sep(w);
	jw_raw(w, "null", 4U);
}

/** 10^n for n in 0..9. */
static uint64_t pow10_u64(uint8_t n)
{
	static const uint64_t t[10] = {
		1ULL,      10ULL,      100ULL,      1000ULL,      10000ULL,
		100000ULL, 1000000ULL, 10000000ULL, 100000000ULL, 1000000000ULL,
	};

	return t[(n > 9U) ? 9U : n];
}

void web_jw_fixed(web_jw_t *w, int64_t scaled, uint8_t decimals)
{
	char out[42];
	size_t n = 0U;
	uint64_t mag;
	uint64_t div;
	uint64_t whole;
	uint64_t frac;
	size_t i;

	if (decimals == 0U) {
		web_jw_i64(w, scaled);
		return;
	}
	if (decimals > 9U) {
		decimals = 9U;
	}
	if (scaled < 0) {
		out[n++] = '-';
		mag = ~(uint64_t)scaled + 1U;
	} else {
		mag = (uint64_t)scaled;
	}
	div = pow10_u64(decimals);
	whole = mag / div;
	frac = mag % div;

	n += u64_digits(whole, &out[n]);
	out[n++] = '.';

	/* Zero-pad the fraction to exactly `decimals` digits. */
	{
		char fb[20];
		size_t fn = u64_digits(frac, fb);

		for (i = fn; i < (size_t)decimals; i++) {
			out[n++] = '0';
		}
		for (i = 0U; i < fn; i++) {
			out[n++] = fb[i];
		}
	}

	jw_value_sep(w);
	jw_raw(w, out, n);
}

void web_jw_f32(web_jw_t *w, float v, uint8_t decimals)
{
	float scale;
	float scaled;

	/* NaN is the only value that compares unequal to itself. */
	if (v != v) {
		web_jw_null(w);
		return;
	}
	if (decimals > 9U) {
		decimals = 9U;
	}
	scale = (float)pow10_u64(decimals);
	scaled = v * scale;
	/*
	 * Guard the int64 conversion: anything beyond ~9.2e18 (and every
	 * infinity) is undefined behaviour to cast, so report it as null. The
	 * bound is deliberately conservative (1e18) because float has only 24
	 * bits of mantissa and the exact limit is not representable.
	 */
	if (scaled >= 1e18f || scaled <= -1e18f) {
		web_jw_null(w);
		return;
	}
	/* Round half away from zero. */
	scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
	web_jw_fixed(w, (int64_t)scaled, decimals);
}

void web_jw_kstr(web_jw_t *w, const char *key, const char *s)
{
	web_jw_key(w, key);
	web_jw_str(w, s);
}

void web_jw_kstrn(web_jw_t *w, const char *key, const char *s, size_t n)
{
	web_jw_key(w, key);
	web_jw_strn(w, s, n);
}

void web_jw_ki64(web_jw_t *w, const char *key, int64_t v)
{
	web_jw_key(w, key);
	web_jw_i64(w, v);
}

void web_jw_ku64(web_jw_t *w, const char *key, uint64_t v)
{
	web_jw_key(w, key);
	web_jw_u64(w, v);
}

void web_jw_kbool(web_jw_t *w, const char *key, bool v)
{
	web_jw_key(w, key);
	web_jw_bool(w, v);
}

void web_jw_knull(web_jw_t *w, const char *key)
{
	web_jw_key(w, key);
	web_jw_null(w);
}

void web_jw_kfixed(web_jw_t *w, const char *key, int64_t scaled,
		   uint8_t decimals)
{
	web_jw_key(w, key);
	web_jw_fixed(w, scaled, decimals);
}

void web_jw_kf32(web_jw_t *w, const char *key, float v, uint8_t decimals)
{
	web_jw_key(w, key);
	web_jw_f32(w, v, decimals);
}

void web_jw_kobj(web_jw_t *w, const char *key)
{
	web_jw_key(w, key);
	web_jw_obj_begin(w);
}

void web_jw_karr(web_jw_t *w, const char *key)
{
	web_jw_key(w, key);
	web_jw_arr_begin(w);
}

void web_jw_khex(web_jw_t *w, const char *key, const uint8_t *p, size_t n)
{
	static const char hexd[] = "0123456789abcdef";
	size_t i;

	web_jw_key(w, key);
	jw_value_sep(w);
	jw_ch(w, '"');
	for (i = 0U; i < n && p != NULL; i++) {
		char pair[2];

		pair[0] = hexd[(p[i] >> 4) & 0x0FU];
		pair[1] = hexd[p[i] & 0x0FU];
		jw_raw(w, pair, 2U);
	}
	jw_ch(w, '"');
}

int web_jw_finish(web_jw_t *w, size_t *out_len)
{
	if (w == NULL) {
		return -EINVAL;
	}
	if (out_len != NULL) {
		*out_len = w->len;
	}
	if (w->err || w->depth != 0U || jw_key_pending(w)) {
		return -EPROTO;
	}
	if (w->overflow) {
		return -ENOSPC;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/* JSON reader                                                               */
/* ------------------------------------------------------------------------- */

static bool json_ws(char c)
{
	return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

static size_t json_skip_ws(const char *p, size_t n, size_t i)
{
	while (i < n && json_ws(p[i])) {
		i++;
	}
	return i;
}

/*
 * Skip the string starting at p[i] (which must be '"'). Returns the index one
 * past the closing quote, or 0 on malformed input (0 is never a valid result
 * because i >= 0 and the string is at least 2 bytes long).
 */
static size_t json_skip_str(const char *p, size_t n, size_t i)
{
	if (i >= n || p[i] != '"') {
		return 0U;
	}
	i++;
	while (i < n) {
		unsigned char c = (unsigned char)p[i];

		if (c == '"') {
			return i + 1U;
		}
		if (c == '\\') {
			if ((i + 1U) >= n) {
				return 0U;
			}
			i += 2U;
			continue;
		}
		if (c < 0x20U) {
			/* Raw control characters are invalid inside a string. */
			return 0U;
		}
		i++;
	}
	return 0U;
}

static bool json_num_char(char c)
{
	return (c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' ||
	       c == 'e' || c == 'E';
}

/*
 * Scan one value starting at p[i] (leading whitespace already skipped).
 * On success returns the index one past the value and fills @p out; on failure
 * returns 0.
 */
static size_t json_scan_val(const char *p, size_t n, size_t i,
			    web_json_val_t *out, uint8_t depth)
{
	size_t start = i;

	if (i >= n) {
		return 0U;
	}
	if (depth > WEB_JSON_READ_DEPTH_MAX) {
		return 0U;
	}

	switch (p[i]) {
	case '"': {
		size_t end = json_skip_str(p, n, i);

		if (end == 0U) {
			return 0U;
		}
		out->type = (uint8_t)WEB_JSON_STR;
		out->p = &p[start + 1U];
		out->n = (end - 1U) - (start + 1U);
		return end;
	}
	case '{':
	case '[': {
		char open = p[i];
		char close = (open == '{') ? '}' : ']';
		unsigned int level = 0U;

		while (i < n) {
			char c = p[i];

			if (c == '"') {
				size_t end = json_skip_str(p, n, i);

				if (end == 0U) {
					return 0U;
				}
				i = end;
				continue;
			}
			if (c == '{' || c == '[') {
				level++;
				if (level > (unsigned int)WEB_JSON_READ_DEPTH_MAX +
						    (unsigned int)depth) {
					return 0U;
				}
			} else if (c == '}' || c == ']') {
				if (level == 0U) {
					return 0U;
				}
				level--;
				if (level == 0U) {
					if (c != close) {
						return 0U;
					}
					out->type = (open == '{')
							    ? (uint8_t)WEB_JSON_OBJ
							    : (uint8_t)WEB_JSON_ARR;
					out->p = &p[start];
					out->n = (i + 1U) - start;
					return i + 1U;
				}
			}
			i++;
		}
		return 0U;
	}
	case 't':
		if ((n - i) < 4U || memcmp(&p[i], "true", 4U) != 0) {
			return 0U;
		}
		out->type = (uint8_t)WEB_JSON_BOOL;
		out->p = &p[start];
		out->n = 4U;
		return i + 4U;
	case 'f':
		if ((n - i) < 5U || memcmp(&p[i], "false", 5U) != 0) {
			return 0U;
		}
		out->type = (uint8_t)WEB_JSON_BOOL;
		out->p = &p[start];
		out->n = 5U;
		return i + 5U;
	case 'n':
		if ((n - i) < 4U || memcmp(&p[i], "null", 4U) != 0) {
			return 0U;
		}
		out->type = (uint8_t)WEB_JSON_NULL;
		out->p = &p[start];
		out->n = 4U;
		return i + 4U;
	default:
		break;
	}

	if (!json_num_char(p[i])) {
		return 0U;
	}
	while (i < n && json_num_char(p[i])) {
		i++;
	}
	out->type = (uint8_t)WEB_JSON_NUM;
	out->p = &p[start];
	out->n = i - start;
	return i;
}

/*
 * One step of object iteration. @p i is the index of the next member (or of the
 * closing brace). Returns 1 with key/val filled, 0 at the end, negative on
 * malformed input; *next receives the index to resume from.
 */
static int json_obj_step(const char *p, size_t n, size_t i,
			 web_json_val_t *key, web_json_val_t *val, size_t *next)
{
	size_t end;

	i = json_skip_ws(p, n, i);
	if (i >= n) {
		return -EBADMSG;
	}
	if (p[i] == '}') {
		*next = i + 1U;
		return 0;
	}
	end = json_skip_str(p, n, i);
	if (end == 0U) {
		return -EBADMSG;
	}
	key->type = (uint8_t)WEB_JSON_STR;
	key->p = &p[i + 1U];
	key->n = (end - 1U) - (i + 1U);

	i = json_skip_ws(p, n, end);
	if (i >= n || p[i] != ':') {
		return -EBADMSG;
	}
	i = json_skip_ws(p, n, i + 1U);
	end = json_scan_val(p, n, i, val, 1U);
	if (end == 0U) {
		return -EBADMSG;
	}

	i = json_skip_ws(p, n, end);
	if (i >= n) {
		return -EBADMSG;
	}
	if (p[i] == ',') {
		/* A comma promises another member: reject `{"a":1,}` here rather
		 * than reporting a clean end of object one step later. */
		size_t peek = json_skip_ws(p, n, i + 1U);

		if (peek >= n || p[peek] != '"') {
			return -EBADMSG;
		}
		*next = i + 1U;
		return 1;
	}
	if (p[i] == '}') {
		/* Last member; the next step reports the end. */
		*next = i;
		return 1;
	}
	return -EBADMSG;
}

int web_json_obj_get(const char *json, size_t len, const char *key,
		     web_json_val_t *out)
{
	size_t i;

	if (json == NULL || key == NULL || out == NULL) {
		return -EINVAL;
	}
	i = json_skip_ws(json, len, 0U);
	if (i >= len || json[i] != '{') {
		return -EBADMSG;
	}
	i++;
	for (;;) {
		web_json_val_t k;
		web_json_val_t v;
		size_t next = 0U;
		int rc = json_obj_step(json, len, i, &k, &v, &next);

		if (rc < 0) {
			return rc;
		}
		if (rc == 0) {
			return -ENOENT;
		}
		if (web_span_eq(k.p, k.n, key)) {
			*out = v;
			return 0;
		}
		i = next;
	}
}

int web_json_obj_next(const web_json_val_t *obj, size_t *cursor,
		      web_json_val_t *key, web_json_val_t *val)
{
	size_t i;
	size_t next = 0U;
	int rc;

	if (obj == NULL || cursor == NULL || key == NULL || val == NULL) {
		return -EINVAL;
	}
	if (obj->type != (uint8_t)WEB_JSON_OBJ || obj->p == NULL || obj->n < 2U) {
		return -EBADMSG;
	}
	i = (*cursor == 0U) ? 1U : *cursor;
	if (i >= obj->n) {
		return 0;
	}
	rc = json_obj_step(obj->p, obj->n, i, key, val, &next);
	if (rc <= 0) {
		return rc;
	}
	*cursor = next;
	return 1;
}

int web_json_arr_next(const web_json_val_t *arr, size_t *cursor,
		      web_json_val_t *out)
{
	size_t i;
	size_t end;

	if (arr == NULL || cursor == NULL || out == NULL) {
		return -EINVAL;
	}
	if (arr->type != (uint8_t)WEB_JSON_ARR || arr->p == NULL || arr->n < 2U) {
		return -EBADMSG;
	}
	i = (*cursor == 0U) ? 1U : *cursor;
	if (i >= arr->n) {
		return 0;
	}
	i = json_skip_ws(arr->p, arr->n, i);
	if (i >= arr->n) {
		return -EBADMSG;
	}
	if (arr->p[i] == ']') {
		*cursor = arr->n;
		return 0;
	}
	end = json_scan_val(arr->p, arr->n, i, out, 1U);
	if (end == 0U) {
		return -EBADMSG;
	}
	i = json_skip_ws(arr->p, arr->n, end);
	if (i >= arr->n) {
		return -EBADMSG;
	}
	if (arr->p[i] == ',') {
		size_t peek = json_skip_ws(arr->p, arr->n, i + 1U);

		/* Reject `[1,]` at the comma (see json_obj_step). */
		if (peek >= arr->n || arr->p[peek] == ']') {
			return -EBADMSG;
		}
		*cursor = i + 1U;
		return 1;
	}
	if (arr->p[i] == ']') {
		*cursor = arr->n;
		return 1;
	}
	return -EBADMSG;
}

int web_json_i64(const web_json_val_t *v, int64_t *out)
{
	uint64_t mag = 0U;
	size_t i = 0U;
	bool neg = false;

	if (v == NULL || out == NULL) {
		return -EINVAL;
	}
	if (v->type != (uint8_t)WEB_JSON_NUM || v->n == 0U) {
		return -EINVAL;
	}
	if (v->p[0] == '-') {
		neg = true;
		i = 1U;
	} else if (v->p[0] == '+') {
		/* RFC 8259 forbids a leading '+'. */
		return -EBADMSG;
	}
	if (i >= v->n) {
		return -EBADMSG;
	}
	for (; i < v->n; i++) {
		char c = v->p[i];

		if (c < '0' || c > '9') {
			/* A fraction or exponent is not an integer field. */
			return -EBADMSG;
		}
		if (mag > (UINT64_MAX / 10U)) {
			return -ERANGE;
		}
		mag *= 10U;
		if ((uint64_t)(c - '0') > (UINT64_MAX - mag)) {
			return -ERANGE;
		}
		mag += (uint64_t)(c - '0');
	}
	if (neg) {
		if (mag > ((uint64_t)INT64_MAX + 1U)) {
			return -ERANGE;
		}
		*out = (mag == ((uint64_t)INT64_MAX + 1U))
			       ? INT64_MIN
			       : -(int64_t)mag;
		return 0;
	}
	if (mag > (uint64_t)INT64_MAX) {
		return -ERANGE;
	}
	*out = (int64_t)mag;
	return 0;
}

int web_json_u64(const web_json_val_t *v, uint64_t *out)
{
	uint64_t mag = 0U;
	size_t i;

	if (v == NULL || out == NULL) {
		return -EINVAL;
	}
	if (v->type != (uint8_t)WEB_JSON_NUM || v->n == 0U) {
		return -EINVAL;
	}
	if (v->p[0] == '-' || v->p[0] == '+') {
		return -ERANGE;
	}
	for (i = 0U; i < v->n; i++) {
		char c = v->p[i];

		if (c < '0' || c > '9') {
			return -EBADMSG;
		}
		if (mag > (UINT64_MAX / 10U)) {
			return -ERANGE;
		}
		mag *= 10U;
		if ((uint64_t)(c - '0') > (UINT64_MAX - mag)) {
			return -ERANGE;
		}
		mag += (uint64_t)(c - '0');
	}
	*out = mag;
	return 0;
}

int web_json_bool(const web_json_val_t *v, bool *out)
{
	if (v == NULL || out == NULL) {
		return -EINVAL;
	}
	if (v->type != (uint8_t)WEB_JSON_BOOL) {
		return -EINVAL;
	}
	*out = (v->n == 4U);
	return 0;
}

static int hexval(char c)
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	if (c >= 'a' && c <= 'f') {
		return (c - 'a') + 10;
	}
	if (c >= 'A' && c <= 'F') {
		return (c - 'A') + 10;
	}
	return -1;
}

int web_json_str_copy(const web_json_val_t *v, char *out, size_t cap)
{
	size_t i;
	size_t o = 0U;

	if (v == NULL || out == NULL || cap == 0U) {
		return -EINVAL;
	}
	if (v->type != (uint8_t)WEB_JSON_STR) {
		return -EINVAL;
	}
	for (i = 0U; i < v->n; i++) {
		char c = v->p[i];
		char emit;

		if (c != '\\') {
			emit = c;
			goto put;
		}
		i++;
		if (i >= v->n) {
			return -EILSEQ;
		}
		switch (v->p[i]) {
		case '"':
			emit = '"';
			break;
		case '\\':
			emit = '\\';
			break;
		case '/':
			emit = '/';
			break;
		case 'b':
			emit = '\b';
			break;
		case 'f':
			emit = '\f';
			break;
		case 'n':
			emit = '\n';
			break;
		case 'r':
			emit = '\r';
			break;
		case 't':
			emit = '\t';
			break;
		case 'u': {
			uint32_t cp = 0U;
			size_t k;

			if ((v->n - i) < 5U) {
				return -EILSEQ;
			}
			for (k = 1U; k <= 4U; k++) {
				int hv = hexval(v->p[i + k]);

				if (hv < 0) {
					return -EILSEQ;
				}
				cp = (cp << 4) | (uint32_t)hv;
			}
			i += 4U;
			if (cp >= 0xD800U && cp <= 0xDFFFU) {
				/* Surrogates only mean anything in pairs, and a
				 * pair encodes a codepoint this API has no use
				 * for. Refuse rather than emit CESU-8. */
				return -EILSEQ;
			}
			if (cp == 0U) {
				/* An embedded NUL would truncate the string for
				 * every C consumer downstream. */
				return -EILSEQ;
			}
			if (cp < 0x80U) {
				emit = (char)cp;
				break;
			}
			if (cp < 0x800U) {
				if ((o + 2U) > (cap - 1U)) {
					return -ENOSPC;
				}
				out[o++] = (char)(0xC0U | (cp >> 6));
				out[o++] = (char)(0x80U | (cp & 0x3FU));
				continue;
			}
			if ((o + 3U) > (cap - 1U)) {
				return -ENOSPC;
			}
			out[o++] = (char)(0xE0U | (cp >> 12));
			out[o++] = (char)(0x80U | ((cp >> 6) & 0x3FU));
			out[o++] = (char)(0x80U | (cp & 0x3FU));
			continue;
		}
		default:
			return -EILSEQ;
		}
put:
		if (emit == '\0') {
			return -EILSEQ;
		}
		if (o >= (cap - 1U)) {
			return -ENOSPC;
		}
		out[o++] = emit;
	}
	out[o] = '\0';
	return (int)o;
}

bool web_json_str_eq(const web_json_val_t *v, const char *lit)
{
	if (v == NULL || v->type != (uint8_t)WEB_JSON_STR) {
		return false;
	}
	return web_span_eq(v->p, v->n, lit);
}

/* ------------------------------------------------------------------------- */
/* hex / base64                                                              */
/* ------------------------------------------------------------------------- */

size_t web_hex_encode(const uint8_t *in, size_t n, char *out, size_t cap)
{
	static const char hexd[] = "0123456789abcdef";
	size_t i;

	if (out == NULL || cap == 0U) {
		return 0U;
	}
	out[0] = '\0';
	if (in == NULL) {
		return 0U;
	}
	if ((n * 2U + 1U) > cap) {
		return 0U;
	}
	for (i = 0U; i < n; i++) {
		out[i * 2U] = hexd[(in[i] >> 4) & 0x0FU];
		out[i * 2U + 1U] = hexd[in[i] & 0x0FU];
	}
	out[n * 2U] = '\0';
	return n * 2U;
}

int web_hex_decode(const char *in, size_t n, uint8_t *out, size_t cap)
{
	size_t i;

	if (in == NULL || out == NULL) {
		return -EINVAL;
	}
	if ((n % 2U) != 0U) {
		return -EILSEQ;
	}
	if ((n / 2U) > cap) {
		return -ENOSPC;
	}
	for (i = 0U; i < n; i += 2U) {
		int hi = hexval(in[i]);
		int lo = hexval(in[i + 1U]);

		if (hi < 0 || lo < 0) {
			return -EILSEQ;
		}
		out[i / 2U] = (uint8_t)((hi << 4) | lo);
	}
	return (int)(n / 2U);
}

static const char b64_alpha[] =
	"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t web_b64_encode(const uint8_t *in, size_t n, char *out, size_t cap)
{
	size_t need = ((n + 2U) / 3U) * 4U;
	size_t i = 0U;
	size_t o = 0U;

	if (out == NULL || cap == 0U) {
		return 0U;
	}
	out[0] = '\0';
	if (in == NULL && n != 0U) {
		return 0U;
	}
	if ((need + 1U) > cap) {
		return 0U;
	}
	while ((n - i) >= 3U) {
		uint32_t v = ((uint32_t)in[i] << 16) | ((uint32_t)in[i + 1U] << 8) |
			     (uint32_t)in[i + 2U];

		out[o++] = b64_alpha[(v >> 18) & 0x3FU];
		out[o++] = b64_alpha[(v >> 12) & 0x3FU];
		out[o++] = b64_alpha[(v >> 6) & 0x3FU];
		out[o++] = b64_alpha[v & 0x3FU];
		i += 3U;
	}
	if ((n - i) == 1U) {
		uint32_t v = (uint32_t)in[i] << 16;

		out[o++] = b64_alpha[(v >> 18) & 0x3FU];
		out[o++] = b64_alpha[(v >> 12) & 0x3FU];
		out[o++] = '=';
		out[o++] = '=';
	} else if ((n - i) == 2U) {
		uint32_t v = ((uint32_t)in[i] << 16) | ((uint32_t)in[i + 1U] << 8);

		out[o++] = b64_alpha[(v >> 18) & 0x3FU];
		out[o++] = b64_alpha[(v >> 12) & 0x3FU];
		out[o++] = b64_alpha[(v >> 6) & 0x3FU];
		out[o++] = '=';
	}
	out[o] = '\0';
	return o;
}

static int b64val(char c)
{
	if (c >= 'A' && c <= 'Z') {
		return c - 'A';
	}
	if (c >= 'a' && c <= 'z') {
		return (c - 'a') + 26;
	}
	if (c >= '0' && c <= '9') {
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

int web_b64_decode(const char *in, size_t n, uint8_t *out, size_t cap)
{
	uint32_t acc = 0U;
	unsigned int bits = 0U;
	size_t i;
	size_t o = 0U;
	unsigned int pad = 0U;

	if (in == NULL || out == NULL) {
		return -EINVAL;
	}
	for (i = 0U; i < n; i++) {
		char c = in[i];
		int v;

		if (json_ws(c)) {
			continue;
		}
		if (c == '=') {
			pad++;
			if (pad > 2U) {
				return -EILSEQ;
			}
			continue;
		}
		if (pad != 0U) {
			/* Data after padding. */
			return -EILSEQ;
		}
		v = b64val(c);
		if (v < 0) {
			return -EILSEQ;
		}
		acc = (acc << 6) | (uint32_t)v;
		bits += 6U;
		if (bits >= 8U) {
			bits -= 8U;
			if (o >= cap) {
				return -ENOSPC;
			}
			out[o++] = (uint8_t)((acc >> bits) & 0xFFU);
		}
	}
	/* Leftover bits must be zero padding, never a partial byte of data. */
	if (bits >= 6U) {
		return -EILSEQ;
	}
	if (bits != 0U && ((acc & ((1U << bits) - 1U)) != 0U)) {
		return -EILSEQ;
	}
	return (int)o;
}
