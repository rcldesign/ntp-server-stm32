/*
 * STS1000 "Meridian" — core/auth: TACACS+ client codec (RFC 8907).
 *
 * See tacacs.h for the contract and for why the body transform is called
 * obfuscation rather than encryption.
 */

#include "auth/tacacs.h"

#include <errno.h>
#include <string.h>

#define MD5_LEN AUTH_MD5_LEN

/* ========================================================================= */
/* header                                                                    */
/* ========================================================================= */

static void put_be32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)((v >> 24) & 0xFFU);
	p[1] = (uint8_t)((v >> 16) & 0xFFU);
	p[2] = (uint8_t)((v >> 8) & 0xFFU);
	p[3] = (uint8_t)(v & 0xFFU);
}

static uint32_t get_be32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
	       ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static uint16_t get_be16(const uint8_t *p)
{
	return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static void put_be16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)((v >> 8) & 0xFFU);
	p[1] = (uint8_t)(v & 0xFFU);
}

int tacacs_hdr_build(uint8_t *out, size_t cap, const tacacs_hdr_t *h)
{
	if (out == NULL || h == NULL) {
		return -EINVAL;
	}
	if (h->length > TACACS_BODY_MAX) {
		return -EINVAL;
	}
	if (cap < TACACS_HDR_LEN) {
		return -ENOSPC;
	}

	out[0] = h->version;
	out[1] = h->type;
	out[2] = h->seq_no;
	out[3] = h->flags;
	put_be32(&out[4], h->session_id);
	put_be32(&out[8], h->length);
	return 0;
}

int tacacs_hdr_parse(const uint8_t *in, size_t len, tacacs_hdr_t *out)
{
	if (in == NULL || out == NULL) {
		return -EINVAL;
	}
	if (len < TACACS_HDR_LEN) {
		return -EAGAIN;
	}

	memset(out, 0, sizeof(*out));
	out->version = in[0];
	out->type = in[1];
	out->seq_no = in[2];
	out->flags = in[3];
	out->session_id = get_be32(&in[4]);
	out->length = get_be32(&in[8]);

	if ((out->version >> 4) != TACACS_VER_MAJOR) {
		return -EBADMSG;
	}
	if (out->length > TACACS_BODY_MAX) {
		return -EBADMSG;
	}
	return 0;
}

/* ========================================================================= */
/* obfuscation                                                               */
/* ========================================================================= */

int tacacs_obfuscate(const tacacs_hdr_t *h, const uint8_t *key, size_t key_len,
		     uint8_t *body, size_t body_len)
{
	uint8_t sid[4];
	uint8_t pad[MD5_LEN];
	auth_md5_t md;
	size_t off = 0U;
	bool first = true;

	if (h == NULL) {
		return -EINVAL;
	}
	if (body == NULL && body_len != 0U) {
		return -EINVAL;
	}
	if (body_len > TACACS_BODY_MAX) {
		return -EINVAL;
	}
	if (key == NULL || key_len == 0U || body_len == 0U) {
		/* Unencrypted session, or nothing to transform. */
		return 0;
	}

	put_be32(sid, h->session_id);

	while (off < body_len) {
		size_t n = body_len - off;
		size_t i;

		auth_md5_init(&md);
		auth_md5_update(&md, sid, sizeof(sid));
		auth_md5_update(&md, key, key_len);
		auth_md5_update(&md, &h->version, 1U);
		auth_md5_update(&md, &h->seq_no, 1U);
		if (!first) {
			auth_md5_update(&md, pad, sizeof(pad));
		}
		auth_md5_final(&md, pad);
		first = false;

		if (n > MD5_LEN) {
			n = MD5_LEN;
		}
		for (i = 0U; i < n; i++) {
			body[off + i] ^= pad[i];
		}
		off += n;
	}
	return 0;
}

/* ========================================================================= */
/* helpers                                                                   */
/* ========================================================================= */

/** strlen of a possibly-NULL string, capped at @p max; -EINVAL if longer. */
static int slen_max(const char *s, size_t max, size_t *out)
{
	size_t n = (s == NULL) ? 0U : strlen(s);

	if (n > max) {
		return -EINVAL;
	}
	*out = n;
	return 0;
}

static void append(uint8_t *buf, size_t *off, const char *s, size_t n)
{
	if (n != 0U) {
		memcpy(&buf[*off], s, n);
		*off += n;
	}
}

/** Copy up to @p cap-1 printable octets into a NUL-terminated field. */
static void copy_msg(char *dst, size_t cap, const uint8_t *src, size_t len)
{
	size_t n = (len > (cap - 1U)) ? (cap - 1U) : len;
	size_t i;

	for (i = 0U; i < n; i++) {
		dst[i] = (src[i] >= 0x20U && src[i] < 0x7FU) ? (char)src[i]
							     : '.';
	}
	dst[n] = '\0';
}

/**
 * Finish a packet: set the length, obfuscate, and set the unencrypted flag when
 * there is no key. Doing all three in one place is what keeps the flag and the
 * transform from ever disagreeing.
 */
static int finish(uint8_t *out, size_t cap, tacacs_hdr_t *h, size_t body_len,
		  const uint8_t *key, size_t key_len, size_t *out_len)
{
	int rc;

	if (body_len > TACACS_BODY_MAX) {
		return -ENOSPC;
	}
	if (cap < (TACACS_HDR_LEN + body_len)) {
		return -ENOSPC;
	}

	h->length = (uint32_t)body_len;
	if (key == NULL || key_len == 0U) {
		h->flags |= (uint8_t)TACACS_FLAG_UNENCRYPTED;
	}

	rc = tacacs_hdr_build(out, cap, h);
	if (rc != 0) {
		return rc;
	}
	rc = tacacs_obfuscate(h, key, key_len, &out[TACACS_HDR_LEN], body_len);
	if (rc != 0) {
		return rc;
	}
	*out_len = TACACS_HDR_LEN + body_len;
	return 0;
}

/**
 * Common receive path: validate the header, check the session, de-obfuscate
 * into @p body.
 *
 * @param want_type   TACACS_TYPE_*.
 * @param body        Scratch of TACACS_BODY_MAX octets.
 * @param out_body_len Receives the body length.
 * @param out_seq     Receives the sequence number.
 */
static int recv_body(const uint8_t *pkt, size_t len, uint32_t session_id,
		     uint8_t expect_seq, uint8_t want_type, const uint8_t *key,
		     size_t key_len, uint8_t *body, size_t *out_body_len,
		     uint8_t *out_seq)
{
	tacacs_hdr_t h;
	int rc;

	rc = tacacs_hdr_parse(pkt, len, &h);
	if (rc != 0) {
		return rc;
	}
	if (len < (TACACS_HDR_LEN + (size_t)h.length)) {
		return -EAGAIN;
	}
	if (h.type != want_type) {
		return -EBADMSG;
	}
	if (h.session_id != session_id) {
		return -ENOMSG;
	}
	if (expect_seq != 0U) {
		if (h.seq_no != expect_seq) {
			return -ENOMSG;
		}
	} else if ((h.seq_no & 1U) != 0U || h.seq_no == 0U) {
		/* Server packets carry even, non-zero sequence numbers. */
		return -ENOMSG;
	}

	if (h.length != 0U) {
		memcpy(body, &pkt[TACACS_HDR_LEN], h.length);
	}
	if ((h.flags & TACACS_FLAG_UNENCRYPTED) == 0U) {
		rc = tacacs_obfuscate(&h, key, key_len, body, h.length);
		if (rc != 0) {
			return rc;
		}
	}

	*out_body_len = h.length;
	*out_seq = h.seq_no;
	return 0;
}

/* ========================================================================= */
/* authentication                                                            */
/* ========================================================================= */

int tacacs_build_authen_start(const tacacs_authen_start_t *in,
			      const uint8_t *key, size_t key_len, uint8_t *out,
			      size_t cap, size_t *out_len)
{
	tacacs_hdr_t h;
	size_t ulen;
	size_t plen;
	size_t rlen;
	size_t dlen = 0U;
	size_t off;
	bool pap;

	if (in == NULL || out == NULL || out_len == NULL) {
		return -EINVAL;
	}
	if (in->user == NULL || in->user[0] == '\0') {
		return -EINVAL;
	}
	if (slen_max(in->user, AUTH_USER_MAX, &ulen) != 0) {
		return -EINVAL;
	}
	if (slen_max(in->port, 64U, &plen) != 0) {
		return -EINVAL;
	}
	if (slen_max(in->rem_addr, 64U, &rlen) != 0) {
		return -EINVAL;
	}

	pap = (in->authen_type == TACACS_AUTHEN_TYPE_PAP);
	if (pap) {
		if (slen_max(in->password, AUTH_SECRET_MAX, &dlen) != 0) {
			return -EINVAL;
		}
		if (dlen == 0U) {
			return -EINVAL;
		}
	}

	if (cap < (TACACS_HDR_LEN + 8U + ulen + plen + rlen + dlen)) {
		return -ENOSPC;
	}

	memset(&h, 0, sizeof(h));
	/* RFC 8907 §5.1: minor version 1 selects the PAP/CHAP variant of the
	 * authentication flow; ASCII login uses minor 0. */
	h.version = TACACS_VER(pap ? TACACS_VER_MINOR_ONE
				   : TACACS_VER_MINOR_DEFAULT);
	h.type = (uint8_t)TACACS_TYPE_AUTHEN;
	h.seq_no = 1U;
	h.flags = in->single_connect ? (uint8_t)TACACS_FLAG_SINGLE_CONNECT : 0U;
	h.session_id = in->session_id;

	off = TACACS_HDR_LEN;
	out[off++] = in->action;
	out[off++] = in->priv_lvl;
	out[off++] = in->authen_type;
	out[off++] = in->authen_service;
	out[off++] = (uint8_t)ulen;
	out[off++] = (uint8_t)plen;
	out[off++] = (uint8_t)rlen;
	out[off++] = (uint8_t)dlen;
	append(out, &off, in->user, ulen);
	append(out, &off, in->port, plen);
	append(out, &off, in->rem_addr, rlen);
	if (dlen != 0U) {
		append(out, &off, in->password, dlen);
	}

	return finish(out, cap, &h, off - TACACS_HDR_LEN, key, key_len, out_len);
}

int tacacs_build_authen_continue(uint32_t session_id, uint8_t seq_no,
				 const char *user_msg, bool abort,
				 const uint8_t *key, size_t key_len,
				 uint8_t *out, size_t cap, size_t *out_len)
{
	tacacs_hdr_t h;
	size_t mlen = 0U;
	size_t dlen = 0U;
	size_t off;

	if (out == NULL || out_len == NULL) {
		return -EINVAL;
	}
	if (seq_no == 0U || (seq_no & 1U) == 0U) {
		/* Client packets are odd-numbered; an even one here would be
		 * an internal sequencing bug, not a wire condition. */
		return -EINVAL;
	}
	if (abort) {
		if (slen_max(user_msg, TACACS_MSG_MAX, &dlen) != 0) {
			return -EINVAL;
		}
	} else if (slen_max(user_msg, AUTH_SECRET_MAX, &mlen) != 0) {
		return -EINVAL;
	}

	if (cap < (TACACS_HDR_LEN + 5U + mlen + dlen)) {
		return -ENOSPC;
	}

	memset(&h, 0, sizeof(h));
	h.version = TACACS_VER(TACACS_VER_MINOR_DEFAULT);
	h.type = (uint8_t)TACACS_TYPE_AUTHEN;
	h.seq_no = seq_no;
	h.session_id = session_id;

	off = TACACS_HDR_LEN;
	put_be16(&out[off], (uint16_t)mlen);
	off += 2U;
	put_be16(&out[off], (uint16_t)dlen);
	off += 2U;
	out[off++] = abort ? (uint8_t)TACACS_CONTINUE_FLAG_ABORT : 0U;
	if (mlen != 0U) {
		append(out, &off, user_msg, mlen);
	}
	if (dlen != 0U) {
		append(out, &off, user_msg, dlen);
	}

	return finish(out, cap, &h, off - TACACS_HDR_LEN, key, key_len, out_len);
}

int tacacs_parse_authen_reply(const uint8_t *pkt, size_t len,
			      uint32_t session_id, uint8_t expect_seq,
			      const uint8_t *key, size_t key_len,
			      tacacs_authen_reply_t *out)
{
	uint8_t body[TACACS_BODY_MAX];
	size_t body_len = 0U;
	size_t msg_len;
	size_t data_len;
	uint8_t seq = 0U;
	int rc;

	if (pkt == NULL || out == NULL) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));

	rc = recv_body(pkt, len, session_id, expect_seq,
		       (uint8_t)TACACS_TYPE_AUTHEN, key, key_len, body,
		       &body_len, &seq);
	if (rc != 0) {
		return rc;
	}
	if (body_len < 6U) {
		return -EBADMSG;
	}

	msg_len = get_be16(&body[2]);
	data_len = get_be16(&body[4]);
	if ((6U + msg_len + data_len) != body_len) {
		/* The three length fields must exactly account for the body;
		 * a mismatch is the classic place a hand-rolled parser walks
		 * off the end. */
		return -EBADMSG;
	}

	out->status = body[0];
	out->flags = body[1];
	out->seq_no = seq;
	out->data_len = data_len;
	copy_msg(out->server_msg, sizeof(out->server_msg), &body[6], msg_len);
	return 0;
}

/* ========================================================================= */
/* authorization                                                             */
/* ========================================================================= */

int tacacs_build_author_request(const tacacs_author_req_t *in,
				const uint8_t *key, size_t key_len,
				uint8_t *out, size_t cap, size_t *out_len)
{
	tacacs_hdr_t h;
	size_t alen[TACACS_ARGS_MAX];
	size_t ulen;
	size_t plen;
	size_t rlen;
	size_t total;
	size_t off;
	size_t i;

	if (in == NULL || out == NULL || out_len == NULL) {
		return -EINVAL;
	}
	if (in->user == NULL || in->user[0] == '\0') {
		return -EINVAL;
	}
	if (in->n_args > TACACS_ARGS_MAX) {
		return -EINVAL;
	}
	if (in->n_args != 0U && in->args == NULL) {
		return -EINVAL;
	}
	if (in->seq_no == 0U || (in->seq_no & 1U) == 0U) {
		return -EINVAL;
	}
	if (slen_max(in->user, AUTH_USER_MAX, &ulen) != 0 ||
	    slen_max(in->port, 64U, &plen) != 0 ||
	    slen_max(in->rem_addr, 64U, &rlen) != 0) {
		return -EINVAL;
	}

	total = 8U + in->n_args + ulen + plen + rlen;
	for (i = 0U; i < in->n_args; i++) {
		if (in->args[i] == NULL) {
			return -EINVAL;
		}
		alen[i] = strlen(in->args[i]);
		if (alen[i] == 0U || alen[i] > 255U) {
			return -EINVAL;
		}
		total += alen[i];
	}
	if (cap < (TACACS_HDR_LEN + total)) {
		return -ENOSPC;
	}

	memset(&h, 0, sizeof(h));
	h.version = TACACS_VER(TACACS_VER_MINOR_DEFAULT);
	h.type = (uint8_t)TACACS_TYPE_AUTHOR;
	h.seq_no = in->seq_no;
	h.flags = in->single_connect ? (uint8_t)TACACS_FLAG_SINGLE_CONNECT : 0U;
	h.session_id = in->session_id;

	off = TACACS_HDR_LEN;
	out[off++] = in->authen_method;
	out[off++] = in->priv_lvl;
	out[off++] = in->authen_type;
	out[off++] = in->authen_service;
	out[off++] = (uint8_t)ulen;
	out[off++] = (uint8_t)plen;
	out[off++] = (uint8_t)rlen;
	out[off++] = (uint8_t)in->n_args;
	for (i = 0U; i < in->n_args; i++) {
		out[off++] = (uint8_t)alen[i];
	}
	append(out, &off, in->user, ulen);
	append(out, &off, in->port, plen);
	append(out, &off, in->rem_addr, rlen);
	for (i = 0U; i < in->n_args; i++) {
		append(out, &off, in->args[i], alen[i]);
	}

	return finish(out, cap, &h, off - TACACS_HDR_LEN, key, key_len, out_len);
}

int tacacs_parse_author_response(const uint8_t *pkt, size_t len,
				 uint32_t session_id, uint8_t expect_seq,
				 const uint8_t *key, size_t key_len,
				 tacacs_author_rsp_t *out)
{
	uint8_t body[TACACS_BODY_MAX];
	size_t body_len = 0U;
	size_t msg_len;
	size_t data_len;
	size_t need;
	size_t off;
	size_t n_args;
	uint8_t seq = 0U;
	size_t i;
	int rc;

	if (pkt == NULL || out == NULL) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));

	rc = recv_body(pkt, len, session_id, expect_seq,
		       (uint8_t)TACACS_TYPE_AUTHOR, key, key_len, body,
		       &body_len, &seq);
	if (rc != 0) {
		return rc;
	}
	if (body_len < 6U) {
		return -EBADMSG;
	}

	n_args = body[1];
	msg_len = get_be16(&body[2]);
	data_len = get_be16(&body[4]);

	need = 6U + n_args;
	if (body_len < need) {
		return -EBADMSG;
	}
	need += msg_len + data_len;
	for (i = 0U; i < n_args; i++) {
		need += body[6U + i];
	}
	if (need != body_len) {
		return -EBADMSG;
	}

	out->status = body[0];
	out->seq_no = seq;
	copy_msg(out->server_msg, sizeof(out->server_msg), &body[6U + n_args],
		 msg_len);

	off = 6U + n_args + msg_len + data_len;
	for (i = 0U; i < n_args; i++) {
		size_t al = body[6U + i];

		if (out->n_args >= TACACS_ARGS_MAX || al > TACACS_ARG_LEN_MAX) {
			if (out->args_dropped < UINT8_MAX) {
				out->args_dropped++;
			}
		} else {
			copy_msg(out->args[out->n_args],
				 sizeof(out->args[0]), &body[off], al);
			out->n_args++;
		}
		off += al;
	}
	return 0;
}

/** Value side of "name=value" / "name*value", or NULL when @p arg is not that. */
static const char *arg_value(const char *arg, const char *name)
{
	size_t n = strlen(name);

	if (strncmp(arg, name, n) != 0) {
		return NULL;
	}
	/* RFC 8907 §6.1: '=' is mandatory, '*' is optional. Both are values. */
	if (arg[n] != '=' && arg[n] != '*') {
		return NULL;
	}
	return &arg[n + 1U];
}

uint8_t tacacs_role_from_args(const tacacs_author_rsp_t *rsp,
			      bool *out_explicit)
{
	size_t i;

	if (out_explicit != NULL) {
		*out_explicit = false;
	}
	if (rsp == NULL) {
		return (uint8_t)AUTH_ROLE_VIEWER;
	}

	for (i = 0U; i < rsp->n_args; i++) {
		const char *v = arg_value(rsp->args[i], "role");
		int r;

		if (v == NULL) {
			continue;
		}
		r = auth_role_parse(v);
		if (r > 0) {
			if (out_explicit != NULL) {
				*out_explicit = true;
			}
			return (uint8_t)r;
		}
	}

	for (i = 0U; i < rsp->n_args; i++) {
		const char *v = arg_value(rsp->args[i], "priv-lvl");
		unsigned int lvl = 0U;
		size_t d = 0U;

		if (v == NULL) {
			v = arg_value(rsp->args[i], "priv_lvl");
		}
		if (v == NULL) {
			continue;
		}
		while (v[d] >= '0' && v[d] <= '9' && d < 3U) {
			lvl = (lvl * 10U) + (unsigned int)(v[d] - '0');
			d++;
		}
		if (d == 0U) {
			continue;
		}
		if (out_explicit != NULL) {
			*out_explicit = true;
		}
		if (lvl >= TACACS_PRIV_ROOT) {
			return (uint8_t)AUTH_ROLE_ADMIN;
		}
		if (lvl >= 2U) {
			return (uint8_t)AUTH_ROLE_OPERATOR;
		}
		return (uint8_t)AUTH_ROLE_VIEWER;
	}

	return (uint8_t)AUTH_ROLE_VIEWER;
}
