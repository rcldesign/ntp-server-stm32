/*
 * STS1000 "Meridian" — core/auth: LDAP v3 client codec (RFC 4511).
 *
 * See ldap.h for the contract and for why this carries its own BER codec.
 *
 * Envelope built and parsed:
 *
 *   LDAPMessage ::= SEQUENCE {
 *     messageID  INTEGER (1 .. 2147483647),
 *     protocolOp CHOICE { bindRequest [APPLICATION 0] ... },
 *     controls   [0] Controls OPTIONAL      -- never emitted, always skipped
 *   }
 */

#include "auth/ldap.h"

#include <errno.h>
#include <string.h>

/* ========================================================================= */
/* BER writer                                                                */
/* ========================================================================= */

/** Octets reserved for a constructed length, shrunk on close. */
#define WR_RESERVE 3U

typedef struct {
	uint8_t *buf;
	size_t cap;
	size_t len;
	bool err;
} wr_t;

static void wr_init(wr_t *w, uint8_t *buf, size_t cap)
{
	w->buf = buf;
	w->cap = cap;
	w->len = 0U;
	w->err = false;
}

static bool wr_room(wr_t *w, size_t n)
{
	if (w->err) {
		return false;
	}
	if (n > (w->cap - w->len)) {
		w->err = true;
		return false;
	}
	return true;
}

static size_t len_size(size_t l)
{
	if (l < 0x80U) {
		return 1U;
	}
	if (l <= 0xFFU) {
		return 2U;
	}
	return 3U;
}

static void len_put(uint8_t *p, size_t l)
{
	if (l < 0x80U) {
		p[0] = (uint8_t)l;
	} else if (l <= 0xFFU) {
		p[0] = 0x81U;
		p[1] = (uint8_t)l;
	} else {
		p[0] = 0x82U;
		p[1] = (uint8_t)((l >> 8) & 0xFFU);
		p[2] = (uint8_t)(l & 0xFFU);
	}
}

static void wr_tlv(wr_t *w, uint8_t tag, const void *val, size_t len)
{
	size_t ls = len_size(len);

	if (len > 0xFFFFU) {
		w->err = true;
		return;
	}
	if (!wr_room(w, 1U + ls + len)) {
		return;
	}
	w->buf[w->len++] = tag;
	len_put(&w->buf[w->len], len);
	w->len += ls;
	if (len != 0U) {
		memcpy(&w->buf[w->len], val, len);
		w->len += len;
	}
}

static void wr_str(wr_t *w, uint8_t tag, const char *s)
{
	size_t n = (s == NULL) ? 0U : strlen(s);

	wr_tlv(w, tag, s, n);
}

/** Minimal two's-complement INTEGER/ENUMERATED. Values here are never negative. */
static void wr_uint(wr_t *w, uint8_t tag, uint32_t v)
{
	uint8_t tmp[5];
	size_t n = 0U;
	int i;

	for (i = 3; i > 0; i--) {
		if (((v >> (8 * (unsigned int)i)) & 0xFFU) != 0U) {
			break;
		}
	}
	/* A leading 0x00 keeps a value whose top bit is set from decoding as
	 * negative — LDAP messageIDs run to 2^31-1, so this matters. */
	if ((((v >> (8 * (unsigned int)i)) & 0xFFU) & 0x80U) != 0U) {
		tmp[n++] = 0x00U;
	}
	for (; i >= 0; i--) {
		tmp[n++] = (uint8_t)((v >> (8 * (unsigned int)i)) & 0xFFU);
	}
	wr_tlv(w, tag, tmp, n);
}

static void wr_bool(wr_t *w, bool v)
{
	uint8_t b = v ? 0xFFU : 0x00U;

	wr_tlv(w, LDAP_T_BOOLEAN, &b, 1U);
}

static size_t wr_begin(wr_t *w, uint8_t tag)
{
	size_t mark;

	if (!wr_room(w, 1U + WR_RESERVE)) {
		return 0U;
	}
	w->buf[w->len++] = tag;
	mark = w->len;
	w->len += WR_RESERVE;
	return mark;
}

static void wr_end(wr_t *w, size_t mark)
{
	size_t content;
	size_t ls;

	if (w->err) {
		return;
	}
	if ((mark + WR_RESERVE) > w->len) {
		w->err = true;
		return;
	}
	content = w->len - (mark + WR_RESERVE);
	if (content > 0xFFFFU) {
		w->err = true;
		return;
	}
	ls = len_size(content);
	if (ls < WR_RESERVE) {
		memmove(&w->buf[mark + ls], &w->buf[mark + WR_RESERVE], content);
		w->len -= (WR_RESERVE - ls);
	}
	len_put(&w->buf[mark], content);
}

/* ========================================================================= */
/* BER reader                                                                */
/* ========================================================================= */

typedef struct {
	const uint8_t *buf;
	size_t len;
	size_t off;
} rd_t;

static void rd_init(rd_t *r, const uint8_t *buf, size_t len)
{
	r->buf = buf;
	r->len = len;
	r->off = 0U;
}

static bool rd_done(const rd_t *r)
{
	return r->off >= r->len;
}

/**
 * Read one TLV header.
 *
 * @retval 0         Read; the reader sits on the content.
 * @retval -EAGAIN   Truncated inside the header or the content.
 * @retval -EBADMSG  Multi-octet identifier, indefinite length, or a length over
 *                   four octets.
 */
static int rd_hdr(rd_t *r, uint8_t *tag, size_t *len)
{
	uint8_t t;
	uint8_t l0;
	size_t l;

	if ((r->len - r->off) < 2U) {
		return -EAGAIN;
	}
	t = r->buf[r->off++];
	if ((t & 0x1FU) == 0x1FU) {
		return -EBADMSG;
	}
	l0 = r->buf[r->off++];
	if ((l0 & 0x80U) == 0U) {
		l = l0;
	} else {
		uint8_t nb = (uint8_t)(l0 & 0x7FU);
		uint8_t i;

		if (nb == 0U || nb > 4U) {
			return -EBADMSG;
		}
		if (nb > (r->len - r->off)) {
			return -EAGAIN;
		}
		l = 0U;
		for (i = 0U; i < nb; i++) {
			l = (l << 8) | (size_t)r->buf[r->off++];
		}
	}
	if (l > (r->len - r->off)) {
		return -EAGAIN;
	}
	*tag = t;
	*len = l;
	return 0;
}

static int rd_int(rd_t *r, uint8_t want_tag, int32_t *out)
{
	uint8_t tag;
	size_t len;
	int64_t v;
	size_t i;
	int rc = rd_hdr(r, &tag, &len);

	if (rc != 0) {
		return rc;
	}
	if (tag != want_tag || len == 0U || len > 4U) {
		return -EBADMSG;
	}
	v = ((r->buf[r->off] & 0x80U) != 0U) ? -1 : 0;
	for (i = 0U; i < len; i++) {
		v = (int64_t)(((uint64_t)v << 8) | (uint64_t)r->buf[r->off + i]);
	}
	r->off += len;
	*out = (int32_t)v;
	return 0;
}

/** Read an OCTET STRING into a fixed field, flagging truncation. */
static int rd_str(rd_t *r, char *out, size_t cap, bool *trunc)
{
	uint8_t tag;
	size_t len;
	size_t n;
	size_t i;
	int rc = rd_hdr(r, &tag, &len);

	if (rc != 0) {
		return rc;
	}
	if (tag != LDAP_T_OCTET_STRING) {
		return -EBADMSG;
	}
	n = (len > (cap - 1U)) ? (cap - 1U) : len;
	for (i = 0U; i < n; i++) {
		uint8_t ch = r->buf[r->off + i];

		/* Directory data reaches logs and the UI; keep it printable. */
		out[i] = (ch >= 0x20U && ch < 0x7FU) ? (char)ch : '.';
	}
	out[n] = '\0';
	if (n != len && trunc != NULL) {
		*trunc = true;
	}
	r->off += len;
	return 0;
}

/* ========================================================================= */
/* result codes                                                              */
/* ========================================================================= */

const char *ldap_result_name(int32_t code)
{
	switch (code) {
	case LDAP_RES_SUCCESS:
		return "success";
	case LDAP_RES_OPERATIONS_ERROR:
		return "operationsError";
	case LDAP_RES_PROTOCOL_ERROR:
		return "protocolError";
	case LDAP_RES_TIME_LIMIT_EXCEEDED:
		return "timeLimitExceeded";
	case LDAP_RES_SIZE_LIMIT_EXCEEDED:
		return "sizeLimitExceeded";
	case LDAP_RES_AUTH_METHOD_NOT_SUPPORTED:
		return "authMethodNotSupported";
	case LDAP_RES_STRONGER_AUTH_REQUIRED:
		return "strongerAuthRequired";
	case LDAP_RES_REFERRAL:
		return "referral";
	case LDAP_RES_NO_SUCH_OBJECT:
		return "noSuchObject";
	case LDAP_RES_INVALID_DN_SYNTAX:
		return "invalidDNSyntax";
	case LDAP_RES_INAPPROPRIATE_AUTH:
		return "inappropriateAuthentication";
	case LDAP_RES_INVALID_CREDENTIALS:
		return "invalidCredentials";
	case LDAP_RES_INSUFFICIENT_ACCESS:
		return "insufficientAccessRights";
	case LDAP_RES_BUSY:
		return "busy";
	case LDAP_RES_UNAVAILABLE:
		return "unavailable";
	case LDAP_RES_UNWILLING_TO_PERFORM:
		return "unwillingToPerform";
	default:
		return "other";
	}
}

int ldap_result_to_errno(int32_t code)
{
	switch (code) {
	case LDAP_RES_SUCCESS:
		return 0;
	case LDAP_RES_INVALID_CREDENTIALS:
	case LDAP_RES_INAPPROPRIATE_AUTH:
	case LDAP_RES_INSUFFICIENT_ACCESS:
		return -EACCES;
	case LDAP_RES_BUSY:
	case LDAP_RES_UNAVAILABLE:
	case LDAP_RES_UNWILLING_TO_PERFORM:
	case LDAP_RES_REFERRAL:
		return -EHOSTUNREACH;
	default:
		return -EPROTO;
	}
}

/* ========================================================================= */
/* framing                                                                   */
/* ========================================================================= */

int ldap_msg_len(const uint8_t *buf, size_t len)
{
	rd_t r;
	uint8_t tag;
	size_t clen;
	int rc;

	if (buf == NULL) {
		return -EINVAL;
	}
	rd_init(&r, buf, len);
	rc = rd_hdr(&r, &tag, &clen);
	if (rc == -EAGAIN) {
		/* rd_hdr also reports -EAGAIN when the content runs past what
		 * arrived, which for a stream reader means "read more" —
		 * except when the message could never fit our buffer. */
		if (len >= 4U && (buf[1] & 0x80U) != 0U) {
			uint8_t nb = (uint8_t)(buf[1] & 0x7FU);

			if (nb >= 3U) {
				return -EBADMSG;
			}
		}
		return -EAGAIN;
	}
	if (rc != 0) {
		return rc;
	}
	if (tag != LDAP_T_SEQUENCE) {
		return -EBADMSG;
	}
	if ((r.off + clen) > LDAP_BUF_MAX) {
		return -EBADMSG;
	}
	return (int)(r.off + clen);
}

/** Open the envelope: returns a reader positioned on the protocolOp. */
static int open_msg(const uint8_t *buf, size_t len, rd_t *body,
		    uint32_t *out_msgid)
{
	rd_t r;
	uint8_t tag;
	size_t clen;
	int32_t id = 0;
	int rc;

	rd_init(&r, buf, len);
	rc = rd_hdr(&r, &tag, &clen);
	if (rc != 0) {
		return rc;
	}
	if (tag != LDAP_T_SEQUENCE) {
		return -EBADMSG;
	}
	rd_init(body, &buf[r.off], clen);

	rc = rd_int(body, LDAP_T_INTEGER, &id);
	if (rc != 0) {
		return (rc == -EAGAIN) ? -EBADMSG : rc;
	}
	if (id < 0) {
		return -EBADMSG;
	}
	*out_msgid = (uint32_t)id;
	return 0;
}

int ldap_msg_peek(const uint8_t *buf, size_t len, uint32_t *out_msgid,
		  uint8_t *out_op)
{
	rd_t body;
	uint32_t id = 0U;
	uint8_t tag;
	size_t clen;
	int rc;

	if (buf == NULL || out_msgid == NULL || out_op == NULL) {
		return -EINVAL;
	}
	rc = open_msg(buf, len, &body, &id);
	if (rc != 0) {
		return rc;
	}
	rc = rd_hdr(&body, &tag, &clen);
	if (rc == -EAGAIN) {
		/* An UnbindRequest is primitive and zero-length, so it is the
		 * one operation whose header is only two octets with nothing
		 * after it — that is complete, not truncated. */
		if ((body.len - body.off) >= 1U &&
		    body.buf[body.off] == LDAP_OP_UNBIND_REQUEST) {
			*out_msgid = id;
			*out_op = (uint8_t)LDAP_OP_UNBIND_REQUEST;
			return 0;
		}
		return -EBADMSG;
	}
	if (rc != 0) {
		return rc;
	}
	*out_msgid = id;
	*out_op = tag;
	return 0;
}

/* ========================================================================= */
/* builders                                                                  */
/* ========================================================================= */

/** Close the envelope and report. */
static int wr_finish(wr_t *w, size_t mark, size_t *out_len)
{
	wr_end(w, mark);
	if (w->err) {
		return -ENOSPC;
	}
	*out_len = w->len;
	return 0;
}

int ldap_build_bind_simple(uint32_t msgid, const char *dn, const char *pw,
			   uint8_t *out, size_t cap, size_t *out_len)
{
	wr_t w;
	size_t m_msg;
	size_t m_op;

	if (dn == NULL || pw == NULL || out == NULL || out_len == NULL) {
		return -EINVAL;
	}
	if (strlen(dn) > LDAP_DN_MAX || strlen(pw) > LDAP_PW_MAX) {
		return -EINVAL;
	}
	/*
	 * RFC 4513 §5.1.2: a non-empty name with an empty password is an
	 * "unauthenticated bind", which a great many servers answer with
	 * success. Accepting it would make an empty password a valid login, so
	 * it is refused here rather than in the caller.
	 */
	if (dn[0] != '\0' && pw[0] == '\0') {
		return -EINVAL;
	}
	if (msgid == 0U) {
		return -EINVAL;
	}

	wr_init(&w, out, cap);
	m_msg = wr_begin(&w, LDAP_T_SEQUENCE);
	wr_uint(&w, LDAP_T_INTEGER, msgid);
	m_op = wr_begin(&w, LDAP_OP_BIND_REQUEST);
	wr_uint(&w, LDAP_T_INTEGER, 3U); /* LDAP v3 */
	wr_str(&w, LDAP_T_OCTET_STRING, dn);
	wr_str(&w, LDAP_AUTH_SIMPLE, pw);
	wr_end(&w, m_op);
	return wr_finish(&w, m_msg, out_len);
}

int ldap_build_unbind(uint32_t msgid, uint8_t *out, size_t cap, size_t *out_len)
{
	wr_t w;
	size_t m_msg;

	if (out == NULL || out_len == NULL || msgid == 0U) {
		return -EINVAL;
	}
	wr_init(&w, out, cap);
	m_msg = wr_begin(&w, LDAP_T_SEQUENCE);
	wr_uint(&w, LDAP_T_INTEGER, msgid);
	wr_tlv(&w, LDAP_OP_UNBIND_REQUEST, NULL, 0U);
	return wr_finish(&w, m_msg, out_len);
}

int ldap_build_starttls(uint32_t msgid, uint8_t *out, size_t cap,
			size_t *out_len)
{
	wr_t w;
	size_t m_msg;
	size_t m_op;

	if (out == NULL || out_len == NULL || msgid == 0U) {
		return -EINVAL;
	}
	wr_init(&w, out, cap);
	m_msg = wr_begin(&w, LDAP_T_SEQUENCE);
	wr_uint(&w, LDAP_T_INTEGER, msgid);
	m_op = wr_begin(&w, LDAP_OP_EXTENDED_REQUEST);
	wr_str(&w, LDAP_EXT_REQUEST_NAME, LDAP_OID_STARTTLS);
	wr_end(&w, m_op);
	return wr_finish(&w, m_msg, out_len);
}

/** Write `(attr=value)` as an equalityMatch filter. */
static void wr_equality(wr_t *w, const char *attr, const char *value)
{
	size_t m = wr_begin(w, LDAP_FILTER_EQUALITY);

	wr_str(w, LDAP_T_OCTET_STRING, attr);
	wr_str(w, LDAP_T_OCTET_STRING, value);
	wr_end(w, m);
}

int ldap_build_search(uint32_t msgid, const ldap_search_req_t *s, uint8_t *out,
		      size_t cap, size_t *out_len)
{
	wr_t w;
	size_t m_msg;
	size_t m_op;
	size_t m_attrs;

	if (s == NULL || out == NULL || out_len == NULL || msgid == 0U) {
		return -EINVAL;
	}
	if (s->base == NULL || s->attr == NULL || s->value == NULL) {
		return -EINVAL;
	}
	if (s->attr[0] == '\0' || s->value[0] == '\0') {
		return -EINVAL;
	}
	if (s->scope > LDAP_SCOPE_SUBTREE) {
		return -EINVAL;
	}
	if (strlen(s->base) > LDAP_DN_MAX || strlen(s->attr) > LDAP_ATTR_MAX ||
	    strlen(s->value) > LDAP_DN_MAX) {
		return -EINVAL;
	}
	if (s->attr2 != NULL) {
		if (s->value2 == NULL || s->attr2[0] == '\0' ||
		    s->value2[0] == '\0') {
			return -EINVAL;
		}
		if (strlen(s->attr2) > LDAP_ATTR_MAX ||
		    strlen(s->value2) > LDAP_DN_MAX) {
			return -EINVAL;
		}
	}
	if (s->want_attr != NULL && strlen(s->want_attr) > LDAP_ATTR_MAX) {
		return -EINVAL;
	}

	wr_init(&w, out, cap);
	m_msg = wr_begin(&w, LDAP_T_SEQUENCE);
	wr_uint(&w, LDAP_T_INTEGER, msgid);
	m_op = wr_begin(&w, LDAP_OP_SEARCH_REQUEST);
	wr_str(&w, LDAP_T_OCTET_STRING, s->base);
	wr_uint(&w, LDAP_T_ENUMERATED, s->scope);
	wr_uint(&w, LDAP_T_ENUMERATED, LDAP_DEREF_NEVER);
	wr_uint(&w, LDAP_T_INTEGER, s->size_limit);
	wr_uint(&w, LDAP_T_INTEGER, s->time_limit);
	/* typesOnly false: a membership check needs no values, but a role
	 * lookup does, and one shape for both is one shape to get right. */
	wr_bool(&w, false);

	if (s->attr2 != NULL) {
		size_t m_and = wr_begin(&w, LDAP_FILTER_AND);

		wr_equality(&w, s->attr, s->value);
		wr_equality(&w, s->attr2, s->value2);
		wr_end(&w, m_and);
	} else {
		wr_equality(&w, s->attr, s->value);
	}

	m_attrs = wr_begin(&w, LDAP_T_SEQUENCE);
	if (s->want_attr != NULL && s->want_attr[0] != '\0') {
		wr_str(&w, LDAP_T_OCTET_STRING, s->want_attr);
	}
	wr_end(&w, m_attrs);

	wr_end(&w, m_op);
	return wr_finish(&w, m_msg, out_len);
}

/* ========================================================================= */
/* parsers                                                                   */
/* ========================================================================= */

int ldap_parse_result(const uint8_t *buf, size_t len, uint8_t want_op,
		      ldap_result_t *out)
{
	rd_t body;
	rd_t op;
	uint32_t id = 0U;
	uint8_t tag;
	size_t clen;
	int32_t code = 0;
	int rc;

	if (buf == NULL || out == NULL) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));

	rc = open_msg(buf, len, &body, &id);
	if (rc != 0) {
		return rc;
	}
	rc = rd_hdr(&body, &tag, &clen);
	if (rc != 0) {
		return rc;
	}
	if (want_op != 0U) {
		if (tag != want_op) {
			return -ENOMSG;
		}
	} else if (tag != LDAP_OP_BIND_RESPONSE && tag != LDAP_OP_SEARCH_DONE &&
		   tag != LDAP_OP_EXTENDED_RESPONSE) {
		return -ENOMSG;
	}

	rd_init(&op, &body.buf[body.off], clen);
	rc = rd_int(&op, LDAP_T_ENUMERATED, &code);
	if (rc != 0) {
		return (rc == -EAGAIN) ? -EBADMSG : rc;
	}
	rc = rd_str(&op, out->matched_dn, sizeof(out->matched_dn),
		    &out->truncated);
	if (rc != 0) {
		return (rc == -EAGAIN) ? -EBADMSG : rc;
	}
	rc = rd_str(&op, out->diagnostic, sizeof(out->diagnostic),
		    &out->truncated);
	if (rc != 0) {
		return (rc == -EAGAIN) ? -EBADMSG : rc;
	}
	/* referral [3] and, for an ExtendedResponse, responseName [10] /
	 * responseValue [11] may follow. Nothing here needs them. */

	out->msgid = id;
	out->op = tag;
	out->code = code;
	return 0;
}

int ldap_parse_search_entry(const uint8_t *buf, size_t len,
			    const char *want_attr, ldap_entry_t *out)
{
	rd_t body;
	rd_t op;
	rd_t attrs;
	uint32_t id = 0U;
	uint8_t tag;
	size_t clen;
	bool got = false;
	int rc;

	if (buf == NULL || out == NULL) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));

	rc = open_msg(buf, len, &body, &id);
	if (rc != 0) {
		return rc;
	}
	rc = rd_hdr(&body, &tag, &clen);
	if (rc != 0) {
		return rc;
	}
	if (tag != LDAP_OP_SEARCH_ENTRY) {
		return -ENOMSG;
	}
	rd_init(&op, &body.buf[body.off], clen);

	rc = rd_str(&op, out->dn, sizeof(out->dn), &out->truncated);
	if (rc != 0) {
		return (rc == -EAGAIN) ? -EBADMSG : rc;
	}

	rc = rd_hdr(&op, &tag, &clen);
	if (rc != 0) {
		return (rc == -EAGAIN) ? -EBADMSG : rc;
	}
	if (tag != LDAP_T_SEQUENCE) {
		return -EBADMSG;
	}
	rd_init(&attrs, &op.buf[op.off], clen);

	while (!rd_done(&attrs)) {
		rd_t one;
		rd_t vals;
		char type[LDAP_ATTR_MAX + 1U];
		bool want;

		rc = rd_hdr(&attrs, &tag, &clen);
		if (rc != 0) {
			return -EBADMSG;
		}
		if (tag != LDAP_T_SEQUENCE) {
			return -EBADMSG;
		}
		rd_init(&one, &attrs.buf[attrs.off], clen);
		attrs.off += clen;

		rc = rd_str(&one, type, sizeof(type), &out->truncated);
		if (rc != 0) {
			return -EBADMSG;
		}
		rc = rd_hdr(&one, &tag, &clen);
		if (rc != 0) {
			return -EBADMSG;
		}
		if (tag != LDAP_T_SET) {
			return -EBADMSG;
		}
		rd_init(&vals, &one.buf[one.off], clen);

		want = (want_attr == NULL) ||
		       (strcmp(type, want_attr) == 0);
		if (got || !want) {
			continue;
		}

		(void)strncpy(out->attr, type, LDAP_ATTR_MAX);
		out->attr[LDAP_ATTR_MAX] = '\0';
		while (!rd_done(&vals) && out->n_values < LDAP_VALUES_MAX) {
			rc = rd_str(&vals, out->values[out->n_values],
				    sizeof(out->values[0]), &out->truncated);
			if (rc != 0) {
				return -EBADMSG;
			}
			out->n_values++;
		}
		if (!rd_done(&vals)) {
			out->truncated = true;
		}
		got = true;
	}

	out->msgid = id;
	return 0;
}

/* ========================================================================= */
/* role mapping                                                              */
/* ========================================================================= */

static char lc(char ch)
{
	return (ch >= 'A' && ch <= 'Z') ? (char)(ch - 'A' + 'a') : ch;
}

static bool dn_eq(const char *a, const char *b)
{
	size_t i = 0U;

	if (a == NULL || b == NULL || a[0] == '\0' || b[0] == '\0') {
		return false;
	}
	while (a[i] != '\0' && b[i] != '\0') {
		if (lc(a[i]) != lc(b[i])) {
			return false;
		}
		i++;
	}
	return a[i] == b[i];
}

uint8_t ldap_role_from_group(const ldap_role_map_t *map, const char *dn,
			     bool *out_explicit)
{
	if (out_explicit != NULL) {
		*out_explicit = false;
	}
	if (map == NULL || dn == NULL) {
		return (uint8_t)AUTH_ROLE_NONE;
	}

	/* Highest privilege first: a user in both the admin and viewer groups
	 * is an admin, not a viewer. */
	if (dn_eq(map->admin, dn)) {
		if (out_explicit != NULL) {
			*out_explicit = true;
		}
		return (uint8_t)AUTH_ROLE_ADMIN;
	}
	if (dn_eq(map->oper, dn)) {
		if (out_explicit != NULL) {
			*out_explicit = true;
		}
		return (uint8_t)AUTH_ROLE_OPERATOR;
	}
	if (dn_eq(map->viewer, dn)) {
		if (out_explicit != NULL) {
			*out_explicit = true;
		}
		return (uint8_t)AUTH_ROLE_VIEWER;
	}
	return (uint8_t)AUTH_ROLE_NONE;
}

bool ldap_user_is_dn_safe(const char *user)
{
	size_t n;
	size_t i;

	if (user == NULL || user[0] == '\0') {
		return false;
	}
	n = strlen(user);
	if (n > AUTH_USER_MAX) {
		return false;
	}
	/* RFC 4514 §2.4 escapables, plus a leading/trailing space and '#' at
	 * the start. Refusing beats escaping: an escaping bug is a DN-injection
	 * bug, and there is no legitimate management account whose name needs a
	 * comma in it. */
	if (user[0] == ' ' || user[0] == '#' || user[n - 1U] == ' ') {
		return false;
	}
	for (i = 0U; i < n; i++) {
		char ch = user[i];

		if (ch == ',' || ch == '+' || ch == '"' || ch == '\\' ||
		    ch == '<' || ch == '>' || ch == ';' || ch == '=') {
			return false;
		}
		if ((unsigned char)ch < 0x20U || (unsigned char)ch >= 0x7FU) {
			return false;
		}
		if (ch == '*' || ch == '(' || ch == ')') {
			/* Also an LDAP *filter* metacharacter (RFC 4515 §3);
			 * the same name is substituted into filters. */
			return false;
		}
	}
	return true;
}

int ldap_dn_from_template(const char *tmpl, const char *user, char *out,
			  size_t cap)
{
	const char *pct;
	size_t head;
	size_t ulen;
	size_t tail;

	if (tmpl == NULL || user == NULL || out == NULL || cap == 0U) {
		return -EINVAL;
	}
	if (!ldap_user_is_dn_safe(user)) {
		return -EINVAL;
	}
	pct = strstr(tmpl, "%s");
	if (pct == NULL) {
		return -EINVAL;
	}

	head = (size_t)(pct - tmpl);
	ulen = strlen(user);
	tail = strlen(pct + 2U);
	if ((head + ulen + tail + 1U) > cap) {
		return -ENOSPC;
	}

	memcpy(out, tmpl, head);
	memcpy(&out[head], user, ulen);
	memcpy(&out[head + ulen], pct + 2U, tail);
	out[head + ulen + tail] = '\0';
	return (int)(head + ulen + tail);
}
