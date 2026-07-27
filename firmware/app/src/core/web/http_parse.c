/*
 * STS1000 "Meridian" — core/web: HTTP/1.1 request parser (see http_parse.h).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 */

#include "web/http_parse.h"

#include <errno.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/* character classes (RFC 9110 §5.6.2 token, §4.1 request-target)             */
/* ------------------------------------------------------------------------- */

static bool is_tchar(char c)
{
	switch (c) {
	case '!':
	case '#':
	case '$':
	case '%':
	case '&':
	case '\'':
	case '*':
	case '+':
	case '-':
	case '.':
	case '^':
	case '_':
	case '`':
	case '|':
	case '~':
		return true;
	default:
		break;
	}
	return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
	       (c >= 'A' && c <= 'Z');
}

/*
 * Legal in an origin-form request-target: RFC 3986 pchar plus '/', '?' and the
 * sub-delims. Anything outside this — including SP, DEL, control characters and
 * every byte >= 0x80 — must arrive percent-encoded, so accepting it raw would
 * mean two clients can spell the same target two ways.
 */
static bool is_target_char(char c)
{
	unsigned char u = (unsigned char)c;

	if (u < 0x21U || u > 0x7EU) {
		return false;
	}
	switch (c) {
	case '"':
	case '<':
	case '>':
	case '\\':
	case '^':
	case '`':
	case '{':
	case '|':
	case '}':
		return false;
	default:
		return true;
	}
}

/* Legal in a field-value: VCHAR, SP, HTAB and obs-text. Excludes CR/LF/NUL. */
static bool is_field_char(char c)
{
	unsigned char u = (unsigned char)c;

	if (c == '\t') {
		return true;
	}
	return u >= 0x20U && u != 0x7FU;
}

static bool is_ows(char c)
{
	return c == ' ' || c == '\t';
}

static http_span_t span_trim(const char *p, size_t n)
{
	http_span_t s;

	while (n > 0U && is_ows(p[0])) {
		p++;
		n--;
	}
	while (n > 0U && is_ows(p[n - 1U])) {
		n--;
	}
	s.p = p;
	s.n = (uint16_t)n;
	return s;
}

/* ------------------------------------------------------------------------- */
/* span accessors                                                            */
/* ------------------------------------------------------------------------- */

bool http_span_eq_ci(http_span_t s, const char *lit)
{
	return web_span_eq_ci(s.p, s.n, lit);
}

bool http_span_eq(http_span_t s, const char *lit)
{
	return web_span_eq(s.p, s.n, lit);
}

size_t http_span_str(http_span_t s, char *out, size_t cap)
{
	return web_span_copy(out, cap, s.p, s.n);
}

/*
 * Case-insensitive "does this comma-separated list contain `tok`" test, used for
 * Connection, Upgrade and Accept-Encoding. Elements may carry parameters
 * (`gzip;q=0`), which are ignored here — a client that sends `q=0` and then
 * complains about a gzip body is broken, and honouring q-values in a 200-byte
 * parser is not worth the code.
 */
static bool list_has_token(http_span_t s, const char *tok)
{
	size_t i = 0U;

	while (i < s.n) {
		size_t start;
		size_t end;

		while (i < s.n && (is_ows(s.p[i]) || s.p[i] == ',')) {
			i++;
		}
		start = i;
		while (i < s.n && s.p[i] != ',') {
			i++;
		}
		end = i;
		/* Trim OWS and any ;parameters off the element. */
		{
			size_t semi = start;

			while (semi < end && s.p[semi] != ';') {
				semi++;
			}
			end = semi;
		}
		while (end > start && is_ows(s.p[end - 1U])) {
			end--;
		}
		if (end > start && web_span_eq_ci(&s.p[start], end - start, tok)) {
			return true;
		}
	}
	return false;
}

/* ------------------------------------------------------------------------- */
/* percent decoding                                                          */
/* ------------------------------------------------------------------------- */

static int hexv(char c)
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

int http_pct_decode(const char *in, size_t n, char *out, size_t cap)
{
	size_t i;
	size_t o = 0U;

	if (out == NULL || cap == 0U) {
		return -EINVAL;
	}
	if (in == NULL && n != 0U) {
		return -EINVAL;
	}
	for (i = 0U; i < n; i++) {
		char c = in[i];

		if (c == '%') {
			int hi;
			int lo;

			if ((n - i) < 3U) {
				return -EILSEQ;
			}
			hi = hexv(in[i + 1U]);
			lo = hexv(in[i + 2U]);
			if (hi < 0 || lo < 0) {
				return -EILSEQ;
			}
			c = (char)((hi << 4) | lo);
			if (c == '\0') {
				/* %00 exists only to truncate a C string. */
				return -EILSEQ;
			}
			i += 2U;
		}
		if (o >= (cap - 1U)) {
			return -ENOSPC;
		}
		out[o++] = c;
	}
	out[o] = '\0';
	return (int)o;
}

int http_path_decode(const http_req_t *r, char *out, size_t cap)
{
	if (r == NULL) {
		return -EINVAL;
	}
	return http_pct_decode(r->path.p, r->path.n, out, cap);
}

/* ------------------------------------------------------------------------- */
/* query and cookies                                                         */
/* ------------------------------------------------------------------------- */

/*
 * Locate `name` in the query string. On success *vp/*vn describe the raw
 * (still-encoded) value, which may be empty.
 */
static bool query_find(const http_req_t *r, const char *name, const char **vp,
		       size_t *vn)
{
	size_t i = 0U;
	size_t nlen;

	if (r == NULL || name == NULL) {
		return false;
	}
	nlen = strlen(name);
	if (nlen == 0U) {
		return false;
	}
	while (i < r->query.n) {
		size_t start = i;
		size_t eq;
		size_t end;

		while (i < r->query.n && r->query.p[i] != '&') {
			i++;
		}
		end = i;
		if (i < r->query.n) {
			i++; /* skip '&' */
		}
		eq = start;
		while (eq < end && r->query.p[eq] != '=') {
			eq++;
		}
		if ((eq - start) == nlen &&
		    memcmp(&r->query.p[start], name, nlen) == 0) {
			if (eq < end) {
				*vp = &r->query.p[eq + 1U];
				*vn = end - (eq + 1U);
			} else {
				*vp = &r->query.p[end];
				*vn = 0U;
			}
			return true;
		}
	}
	return false;
}

int http_query_get(const http_req_t *r, const char *name, char *out, size_t cap)
{
	const char *vp = NULL;
	size_t vn = 0U;

	if (r == NULL || name == NULL || out == NULL || cap == 0U) {
		return -EINVAL;
	}
	if (!query_find(r, name, &vp, &vn)) {
		return -ENOENT;
	}
	return http_pct_decode(vp, vn, out, cap);
}

bool http_query_has(const http_req_t *r, const char *name)
{
	const char *vp = NULL;
	size_t vn = 0U;

	return query_find(r, name, &vp, &vn);
}

int http_query_get_u32(const http_req_t *r, const char *name, uint32_t *out)
{
	char buf[16];
	int n;
	uint64_t v = 0U;
	int i;

	if (r == NULL || name == NULL || out == NULL) {
		return -EINVAL;
	}
	n = http_query_get(r, name, buf, sizeof(buf));
	if (n == -ENOSPC) {
		/* Too long to be a u32 in the first place. */
		return -ERANGE;
	}
	if (n < 0) {
		return (n == -EILSEQ) ? -EBADMSG : n;
	}
	if (n == 0) {
		return -EBADMSG;
	}
	for (i = 0; i < n; i++) {
		if (buf[i] < '0' || buf[i] > '9') {
			return -EBADMSG;
		}
		v = (v * 10U) + (uint64_t)(buf[i] - '0');
		if (v > 0xFFFFFFFFULL) {
			return -ERANGE;
		}
	}
	*out = (uint32_t)v;
	return 0;
}

int http_cookie_get(const http_req_t *r, const char *name, char *out, size_t cap)
{
	size_t i = 0U;
	size_t nlen;

	if (r == NULL || name == NULL || out == NULL || cap == 0U) {
		return -EINVAL;
	}
	nlen = strlen(name);
	if (nlen == 0U) {
		return -EINVAL;
	}
	while (i < r->cookie.n) {
		size_t start;
		size_t eq;
		size_t end;

		while (i < r->cookie.n &&
		       (is_ows(r->cookie.p[i]) || r->cookie.p[i] == ';')) {
			i++;
		}
		start = i;
		while (i < r->cookie.n && r->cookie.p[i] != ';') {
			i++;
		}
		end = i;
		while (end > start && is_ows(r->cookie.p[end - 1U])) {
			end--;
		}
		eq = start;
		while (eq < end && r->cookie.p[eq] != '=') {
			eq++;
		}
		if (eq >= end) {
			continue; /* a cookie-pair with no '=' is not one */
		}
		if ((eq - start) == nlen &&
		    memcmp(&r->cookie.p[start], name, nlen) == 0) {
			size_t vlen = end - (eq + 1U);

			if ((vlen + 1U) > cap) {
				return -ENOSPC;
			}
			if (vlen > 0U) {
				memcpy(out, &r->cookie.p[eq + 1U], vlen);
			}
			out[vlen] = '\0';
			return (int)vlen;
		}
	}
	return -ENOENT;
}

bool http_bearer_token(const http_req_t *r, char *out, size_t cap)
{
	const char *p;
	size_t n;
	size_t i = 0U;

	if (r == NULL || out == NULL || cap == 0U) {
		return false;
	}
	p = r->authorization.p;
	n = r->authorization.n;
	if (p == NULL || n < 7U) {
		return false;
	}
	if (!web_span_eq_ci(p, 6U, "Bearer")) {
		return false;
	}
	i = 6U;
	while (i < n && is_ows(p[i])) {
		i++;
	}
	if (i >= n) {
		return false;
	}
	if ((n - i + 1U) > cap) {
		return false;
	}
	(void)web_span_copy(out, cap, &p[i], n - i);
	return true;
}

/* ------------------------------------------------------------------------- */
/* the request head                                                          */
/* ------------------------------------------------------------------------- */

/*
 * Find the CRLF ending the line that starts at buf[i]. Returns the index of the
 * CR, or SIZE_MAX when the line is not yet complete. A bare LF sets *bad.
 */
static size_t find_eol(const char *buf, size_t len, size_t i, bool *bad)
{
	while (i < len) {
		if (buf[i] == '\r') {
			if ((i + 1U) >= len) {
				return SIZE_MAX; /* need the LF */
			}
			if (buf[i + 1U] != '\n') {
				*bad = true;
				return SIZE_MAX;
			}
			return i;
		}
		if (buf[i] == '\n') {
			/* Bare LF: RFC 9112 §2.2 lets a recipient accept it,
			 * but a proxy in front of us might not, and that
			 * disagreement is exactly how requests get smuggled. */
			*bad = true;
			return SIZE_MAX;
		}
		i++;
	}
	return SIZE_MAX;
}

static int parse_request_line(const char *buf, size_t len, size_t *pos,
			      http_req_t *out)
{
	bool bad = false;
	size_t eol = find_eol(buf, len, 0U, &bad);
	size_t sp1;
	size_t sp2;
	size_t i;

	if (bad) {
		return -EBADMSG;
	}
	if (eol == SIZE_MAX) {
		return -EAGAIN;
	}
	if (eol > HTTP_HEADER_LINE_MAX) {
		return -E2BIG;
	}

	/* method SP request-target SP HTTP-version */
	sp1 = 0U;
	while (sp1 < eol && buf[sp1] != ' ') {
		if (!is_tchar(buf[sp1])) {
			return -EBADMSG;
		}
		sp1++;
	}
	if (sp1 == 0U || sp1 >= eol) {
		return -EBADMSG;
	}
	out->method = web_method_parse(buf, sp1);

	sp2 = sp1 + 1U;
	while (sp2 < eol && buf[sp2] != ' ') {
		sp2++;
	}
	if (sp2 >= eol) {
		return -EBADMSG;
	}
	if ((sp2 - (sp1 + 1U)) == 0U) {
		return -EBADMSG;
	}
	if ((sp2 - (sp1 + 1U)) > HTTP_TARGET_MAX) {
		return -E2BIG;
	}
	/* Origin-form only: no absolute-form (proxy) or authority-form. */
	if (buf[sp1 + 1U] != '/') {
		return -EBADMSG;
	}
	for (i = sp1 + 1U; i < sp2; i++) {
		if (!is_target_char(buf[i])) {
			return -EBADMSG;
		}
	}
	out->target.p = &buf[sp1 + 1U];
	out->target.n = (uint16_t)(sp2 - (sp1 + 1U));

	/* Split at the first '?'. */
	{
		uint16_t q = 0U;

		while (q < out->target.n && out->target.p[q] != '?') {
			q++;
		}
		out->path.p = out->target.p;
		out->path.n = q;
		if (q < out->target.n) {
			out->query.p = &out->target.p[q + 1U];
			out->query.n = (uint16_t)(out->target.n - (q + 1U));
		} else {
			out->query.p = &out->target.p[out->target.n];
			out->query.n = 0U;
		}
	}

	/* HTTP-version: exactly "HTTP/1.0" or "HTTP/1.1". */
	{
		size_t vn = eol - (sp2 + 1U);
		const char *v = &buf[sp2 + 1U];

		if (vn != 8U || memcmp(v, "HTTP/1.", 7U) != 0) {
			return -ENOTSUP;
		}
		if (v[7] == '1') {
			out->version_minor = 1U;
		} else if (v[7] == '0') {
			out->version_minor = 0U;
		} else {
			return -ENOTSUP;
		}
	}

	*pos = eol + 2U;
	return 0;
}

/* Assign a recognised header into the request struct, or ignore it. */
static int store_header(http_req_t *out, http_span_t name, http_span_t val,
			bool *saw_te, http_span_t *connection,
			http_span_t *upgrade)
{
	if (http_span_eq_ci(name, "host")) {
		out->host = val;
		return 0;
	}
	if (http_span_eq_ci(name, "content-length")) {
		uint64_t v = 0U;
		uint16_t i;

		if (val.n == 0U || val.n > 10U) {
			return -EBADMSG;
		}
		for (i = 0U; i < val.n; i++) {
			if (val.p[i] < '0' || val.p[i] > '9') {
				return -EBADMSG;
			}
			v = (v * 10U) + (uint64_t)(val.p[i] - '0');
		}
		if (v > 0xFFFFFFFFULL) {
			return -E2BIG;
		}
		if ((out->flags & HTTP_F_HAS_CL) != 0U) {
			/* A second Content-Length is only benign when it agrees;
			 * RFC 9112 §6.3 makes disagreement unrecoverable. */
			if (out->content_length != (uint32_t)v) {
				return -EPROTO;
			}
			return 0;
		}
		out->flags |= HTTP_F_HAS_CL;
		out->content_length = (uint32_t)v;
		return 0;
	}
	if (http_span_eq_ci(name, "transfer-encoding")) {
		*saw_te = true;
		/* Only `chunked`, and only as the sole coding: `gzip, chunked`
		 * would need a decompressor we do not have, and `chunked, gzip`
		 * is illegal. */
		if (!http_span_eq_ci(val, "chunked")) {
			return -ENOTSUP;
		}
		out->flags |= HTTP_F_CHUNKED;
		return 0;
	}
	if (http_span_eq_ci(name, "connection")) {
		*connection = val;
		return 0;
	}
	if (http_span_eq_ci(name, "upgrade")) {
		*upgrade = val;
		return 0;
	}
	if (http_span_eq_ci(name, "authorization")) {
		out->authorization = val;
		return 0;
	}
	if (http_span_eq_ci(name, "cookie")) {
		out->cookie = val;
		return 0;
	}
	if (http_span_eq_ci(name, "content-type")) {
		out->content_type = val;
		return 0;
	}
	if (http_span_eq_ci(name, "if-none-match")) {
		out->if_none_match = val;
		return 0;
	}
	if (http_span_eq_ci(name, "accept-encoding")) {
		out->accept_encoding = val;
		if (list_has_token(val, "gzip")) {
			out->flags |= HTTP_F_GZIP_OK;
		}
		return 0;
	}
	if (http_span_eq_ci(name, "origin")) {
		out->origin = val;
		return 0;
	}
	if (http_span_eq_ci(name, "referer")) {
		out->referer = val;
		return 0;
	}
	if (http_span_eq_ci(name, "x-csrf-token")) {
		out->csrf = val;
		return 0;
	}
	if (http_span_eq_ci(name, "sec-websocket-key")) {
		out->ws_key = val;
		return 0;
	}
	if (http_span_eq_ci(name, "sec-websocket-version")) {
		out->ws_version = val;
		return 0;
	}
	if (http_span_eq_ci(name, "sec-websocket-protocol")) {
		out->ws_protocol = val;
		return 0;
	}
	if (http_span_eq_ci(name, "expect")) {
		if (http_span_eq_ci(val, "100-continue")) {
			out->flags |= HTTP_F_EXPECT_100;
		}
		return 0;
	}
	return 0;
}

int http_parse_request(const char *buf, size_t len, http_req_t *out)
{
	size_t pos = 0U;
	bool saw_te = false;
	http_span_t connection = { NULL, 0U };
	http_span_t upgrade = { NULL, 0U };
	int rc;

	if (buf == NULL || out == NULL) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));
	out->method = (uint8_t)WEB_METHOD_UNKNOWN;

	if (len > HTTP_HEAD_MAX) {
		/*
		 * The caller hands us its whole receive buffer, which may
		 * already contain body bytes. Only refuse when no complete head
		 * fits in the first HTTP_HEAD_MAX bytes.
		 */
		size_t probe;
		bool found = false;

		for (probe = 0U; (probe + 3U) < HTTP_HEAD_MAX; probe++) {
			if (buf[probe] == '\r' && buf[probe + 1U] == '\n' &&
			    buf[probe + 2U] == '\r' && buf[probe + 3U] == '\n') {
				found = true;
				break;
			}
		}
		if (!found) {
			return -E2BIG;
		}
		len = HTTP_HEAD_MAX;
	}

	rc = parse_request_line(buf, len, &pos, out);
	if (rc != 0) {
		return rc;
	}

	for (;;) {
		bool bad = false;
		size_t eol;
		size_t colon;
		http_span_t name;
		http_span_t val;

		if (pos >= len) {
			return -EAGAIN;
		}
		if (pos > HTTP_HEAD_MAX) {
			return -E2BIG;
		}
		eol = find_eol(buf, len, pos, &bad);
		if (bad) {
			return -EBADMSG;
		}
		if (eol == SIZE_MAX) {
			return -EAGAIN;
		}
		if (eol == pos) {
			/* The empty line: end of head. */
			pos = eol + 2U;
			out->head_len = (uint16_t)pos;
			break;
		}
		if ((eol - pos) > HTTP_HEADER_LINE_MAX) {
			return -E2BIG;
		}
		if (is_ows(buf[pos])) {
			/* obs-fold. Deprecated by RFC 9112 §5.2 and a classic
			 * smuggling primitive; reject rather than unfold. */
			return -EBADMSG;
		}
		if (out->n_headers >= HTTP_HEADERS_MAX) {
			return -E2BIG;
		}

		colon = pos;
		while (colon < eol && buf[colon] != ':') {
			if (!is_tchar(buf[colon])) {
				return -EBADMSG;
			}
			colon++;
		}
		if (colon >= eol || colon == pos) {
			return -EBADMSG;
		}
		/* is_tchar() already rejected SP/HTAB before the colon, so a
		 * "Name : value" line cannot reach here. */
		name.p = &buf[pos];
		name.n = (uint16_t)(colon - pos);

		{
			size_t vs = colon + 1U;
			size_t i;

			for (i = vs; i < eol; i++) {
				if (!is_field_char(buf[i])) {
					return -EBADMSG;
				}
			}
			val = span_trim(&buf[vs], eol - vs);
		}

		out->n_headers++;
		rc = store_header(out, name, val, &saw_te, &connection,
				  &upgrade);
		if (rc != 0) {
			return rc;
		}
		pos = eol + 2U;
	}

	/*
	 * Framing. RFC 9112 §6.3: Transfer-Encoding wins over Content-Length,
	 * but a request carrying both is a smuggling attempt against whatever
	 * is in front of us, so refuse it outright.
	 */
	if (saw_te && ((out->flags & HTTP_F_HAS_CL) != 0U)) {
		return -EPROTO;
	}
	if ((out->flags & HTTP_F_CHUNKED) != 0U) {
		out->flags |= HTTP_F_HAS_BODY;
		out->content_length = 0U;
	} else if (out->content_length > 0U) {
		out->flags |= HTTP_F_HAS_BODY;
	}

	/* HTTP/1.1 defaults to keep-alive, HTTP/1.0 to close. */
	if (out->version_minor >= 1U) {
		out->flags |= HTTP_F_KEEPALIVE;
	}
	if (list_has_token(connection, "close")) {
		out->flags &= ~(uint32_t)HTTP_F_KEEPALIVE;
	} else if (list_has_token(connection, "keep-alive")) {
		out->flags |= HTTP_F_KEEPALIVE;
	}

	/*
	 * A WebSocket upgrade is only recognised when every RFC 6455 §4.1
	 * precondition holds: GET, HTTP/1.1, `Connection: Upgrade`,
	 * `Upgrade: websocket`, a key, and version 13. A request missing any of
	 * them is left as a plain GET so the router answers 400/426 with a
	 * useful message instead of half-upgrading.
	 */
	if ((out->method == (uint8_t)WEB_METHOD_GET) && (out->version_minor >= 1U) &&
	    list_has_token(connection, "upgrade") &&
	    list_has_token(upgrade, "websocket") && (out->ws_key.n > 0U) &&
	    http_span_eq(out->ws_version, "13")) {
		out->flags |= HTTP_F_UPGRADE_WS;
	}

	/* Host is mandatory in HTTP/1.1 (RFC 9112 §3.2). */
	if ((out->version_minor >= 1U) && (out->host.n == 0U)) {
		return -EBADMSG;
	}

	return 0;
}

/* ------------------------------------------------------------------------- */
/* chunked decoding                                                          */
/* ------------------------------------------------------------------------- */

void http_chunked_init(http_chunked_t *st, uint32_t max_total)
{
	if (st == NULL) {
		return;
	}
	memset(st, 0, sizeof(*st));
	st->phase = (uint8_t)HTTP_CHUNK_SIZE;
	st->max_total = (max_total == 0U) ? HTTP_BODY_MAX : max_total;
}

static int chunk_fail(http_chunked_t *st, int err)
{
	st->phase = (uint8_t)HTTP_CHUNK_ERROR;
	st->err = err;
	return err;
}

int http_chunked_feed(http_chunked_t *st, const char *in, size_t in_len,
		      size_t *consumed, uint8_t *out, size_t out_cap,
		      size_t *out_len)
{
	size_t i = 0U;
	size_t o;

	if (st == NULL || out_len == NULL || (in == NULL && in_len != 0U) ||
	    (out == NULL && out_cap != 0U)) {
		return -EINVAL;
	}
	o = *out_len;
	if (consumed != NULL) {
		*consumed = 0U;
	}
	if (st->phase == (uint8_t)HTTP_CHUNK_ERROR) {
		return st->err;
	}
	if (st->phase == (uint8_t)HTTP_CHUNK_DONE) {
		return 1;
	}

	while (i < in_len) {
		char c = in[i];

		switch (st->phase) {
		case HTTP_CHUNK_SIZE: {
			int hv;

			if (c == '\r') {
				st->saw_cr = true;
				i++;
				continue;
			}
			if (c == '\n') {
				if (!st->saw_cr || st->size_digits == 0U) {
					goto bad;
				}
				st->saw_cr = false;
				st->size_digits = 0U;
				i++;
				if (st->remain == 0U) {
					st->phase = (uint8_t)HTTP_CHUNK_TRAILER;
				} else {
					st->phase = (uint8_t)HTTP_CHUNK_DATA;
				}
				continue;
			}
			if (st->saw_cr) {
				goto bad; /* CR not followed by LF */
			}
			if (c == ';') {
				if (st->size_digits == 0U) {
					goto bad;
				}
				st->phase = (uint8_t)HTTP_CHUNK_EXT;
				i++;
				continue;
			}
			hv = hexv(c);
			if (hv < 0) {
				goto bad;
			}
			if (st->size_digits >= 8U) {
				/* > 2^32-1 bytes in one chunk. */
				return chunk_fail(st, -E2BIG);
			}
			st->remain = (st->remain << 4) | (uint32_t)hv;
			st->size_digits++;
			if (st->remain > st->max_total) {
				return chunk_fail(st, -E2BIG);
			}
			i++;
			continue;
		}
		case HTTP_CHUNK_EXT:
			if (c == '\r') {
				st->saw_cr = true;
				i++;
				continue;
			}
			if (c == '\n') {
				if (!st->saw_cr) {
					goto bad;
				}
				st->saw_cr = false;
				st->size_digits = 0U;
				i++;
				st->phase = (st->remain == 0U)
						    ? (uint8_t)HTTP_CHUNK_TRAILER
						    : (uint8_t)HTTP_CHUNK_DATA;
				continue;
			}
			if (st->saw_cr || !is_field_char(c)) {
				goto bad;
			}
			i++;
			continue;
		case HTTP_CHUNK_DATA: {
			size_t take = in_len - i;

			if (take > st->remain) {
				take = st->remain;
			}
			if ((st->total + take) > st->max_total) {
				return chunk_fail(st, -E2BIG);
			}
			if ((o + take) > out_cap) {
				return chunk_fail(st, -ENOSPC);
			}
			memcpy(&out[o], &in[i], take);
			o += take;
			st->total += (uint32_t)take;
			st->remain -= (uint32_t)take;
			i += take;
			if (st->remain == 0U) {
				st->phase = (uint8_t)HTTP_CHUNK_DATA_CR;
			}
			continue;
		}
		case HTTP_CHUNK_DATA_CR:
			if (c != '\r') {
				goto bad;
			}
			st->phase = (uint8_t)HTTP_CHUNK_DATA_LF;
			i++;
			continue;
		case HTTP_CHUNK_DATA_LF:
			if (c != '\n') {
				goto bad;
			}
			st->phase = (uint8_t)HTTP_CHUNK_SIZE;
			st->remain = 0U;
			st->size_digits = 0U;
			i++;
			continue;
		case HTTP_CHUNK_TRAILER:
			/*
			 * After the 0-chunk: zero or more trailer field lines,
			 * then an empty line. Trailers are skipped, not parsed —
			 * honouring a trailer would let a header arrive after
			 * the routing decision was already made.
			 */
			st->trailer_len++;
			if (st->trailer_len > HTTP_HEADER_LINE_MAX) {
				return chunk_fail(st, -E2BIG);
			}
			if (c == '\r') {
				st->saw_cr = true;
				i++;
				continue;
			}
			if (c == '\n') {
				if (!st->saw_cr) {
					goto bad;
				}
				st->saw_cr = false;
				i++;
				if (st->size_digits == 0U) {
					/* An empty line ends the trailers. */
					st->phase = (uint8_t)HTTP_CHUNK_DONE;
					*out_len = o;
					if (consumed != NULL) {
						*consumed = i;
					}
					return 1;
				}
				st->size_digits = 0U;
				continue;
			}
			if (st->saw_cr || !is_field_char(c)) {
				goto bad;
			}
			/* size_digits doubles as "this trailer line is not
			 * empty" so the terminator is unambiguous. */
			st->size_digits = 1U;
			i++;
			continue;
		default:
			goto bad;
		}
	}

	*out_len = o;
	if (consumed != NULL) {
		*consumed = i;
	}
	return 0;

bad:
	*out_len = o;
	if (consumed != NULL) {
		*consumed = i;
	}
	return chunk_fail(st, -EBADMSG);
}
