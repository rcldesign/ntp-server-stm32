/*
 * STS1000 "Meridian" — core/auth: RADIUS client codec.
 *
 * See radius.h for the contract. Header layout (RFC 2865 §3):
 *
 *   0        1        2      3      4 .. 19    20 ..
 *   code     id       length(BE)    authenticator   attributes (type,len,value)
 */

#include "auth/radius.h"

#include <errno.h>
#include <string.h>

#define MD5_LEN AUTH_MD5_LEN

/* ========================================================================= */
/* primitives                                                                */
/* ========================================================================= */

int radius_hide_password(const uint8_t *shared, size_t shared_len,
			 const uint8_t authenticator[RADIUS_AUTH_LEN],
			 const char *password, uint8_t *out, size_t cap,
			 size_t *out_len)
{
	uint8_t plain[AUTH_SECRET_MAX + MD5_LEN];
	uint8_t b[MD5_LEN];
	auth_md5_t h;
	size_t plen;
	size_t chunks;
	size_t i;
	size_t j;

	if (shared == NULL || authenticator == NULL || password == NULL ||
	    out == NULL || out_len == NULL) {
		return -EINVAL;
	}
	if (shared_len == 0U) {
		return -EINVAL;
	}
	plen = strlen(password);
	if (plen > AUTH_SECRET_MAX) {
		return -EINVAL;
	}

	/* Zero-pad to a whole number of 16-octet chunks; an empty password
	 * still occupies one chunk. */
	chunks = (plen + MD5_LEN - 1U) / MD5_LEN;
	if (chunks == 0U) {
		chunks = 1U;
	}
	if (cap < chunks * MD5_LEN) {
		return -ENOSPC;
	}

	memset(plain, 0, sizeof(plain));
	memcpy(plain, password, plen);

	for (i = 0U; i < chunks; i++) {
		auth_md5_init(&h);
		auth_md5_update(&h, shared, shared_len);
		if (i == 0U) {
			auth_md5_update(&h, authenticator, RADIUS_AUTH_LEN);
		} else {
			/* Chained on the previous *ciphertext* chunk. */
			auth_md5_update(&h, &out[(i - 1U) * MD5_LEN], MD5_LEN);
		}
		auth_md5_final(&h, b);

		for (j = 0U; j < MD5_LEN; j++) {
			out[(i * MD5_LEN) + j] =
				(uint8_t)(plain[(i * MD5_LEN) + j] ^ b[j]);
		}
	}

	*out_len = chunks * MD5_LEN;
	return 0;
}

int radius_response_auth(const uint8_t *pkt, size_t len,
			 const uint8_t req_auth[RADIUS_AUTH_LEN],
			 const uint8_t *shared, size_t shared_len,
			 uint8_t out[RADIUS_AUTH_LEN])
{
	auth_md5_t h;

	if (pkt == NULL || req_auth == NULL || shared == NULL || out == NULL) {
		return -EINVAL;
	}
	if (len < RADIUS_HDR_LEN) {
		return -EINVAL;
	}

	auth_md5_init(&h);
	auth_md5_update(&h, pkt, 4U); /* code, id, length */
	auth_md5_update(&h, req_auth, RADIUS_AUTH_LEN);
	if (len > RADIUS_HDR_LEN) {
		auth_md5_update(&h, &pkt[RADIUS_HDR_LEN], len - RADIUS_HDR_LEN);
	}
	auth_md5_update(&h, shared, shared_len);
	auth_md5_final(&h, out);
	return 0;
}

int radius_attr_find(const uint8_t *pkt, size_t len, uint8_t type, size_t skip,
		     const uint8_t **val, size_t *val_len)
{
	size_t off;

	if (pkt == NULL || val == NULL || val_len == NULL) {
		return -EINVAL;
	}
	if (len < RADIUS_HDR_LEN) {
		return -EINVAL;
	}

	off = RADIUS_HDR_LEN;
	while (off < len) {
		size_t alen;

		if ((len - off) < 2U) {
			return -EBADMSG;
		}
		alen = pkt[off + 1U];
		/* RFC 2865 §5: length counts the type and length octets, so it
		 * is at least 2. A zero or one length would loop forever. */
		if (alen < 2U || (off + alen) > len) {
			return -EBADMSG;
		}
		if (pkt[off] == type) {
			if (skip == 0U) {
				*val = &pkt[off + 2U];
				*val_len = alen - 2U;
				return 0;
			}
			skip--;
		}
		off += alen;
	}
	return -ENOENT;
}

int radius_attrs_check(const uint8_t *pkt, size_t len)
{
	const uint8_t *v = NULL;
	size_t vl = 0U;
	int rc;

	/* Type 0 is not assignable, so this walks the whole list and can only
	 * end in -ENOENT (well formed) or -EBADMSG. */
	rc = radius_attr_find(pkt, len, 0U, 0U, &v, &vl);
	if (rc == -ENOENT) {
		return 0;
	}
	return (rc == 0) ? -EBADMSG : rc;
}

uint8_t radius_role_from_attrs(const uint8_t *pkt, size_t len,
			       bool *out_explicit)
{
	const uint8_t *v = NULL;
	size_t vl = 0U;
	size_t idx;

	if (out_explicit != NULL) {
		*out_explicit = false;
	}

	/* Filter-Id first, and every instance of it: a server commonly returns
	 * several and only one of them names a role. */
	for (idx = 0U; idx < 8U; idx++) {
		char buf[32];
		int r;

		if (radius_attr_find(pkt, len, RADIUS_AT_FILTER_ID, idx, &v,
				     &vl) != 0) {
			break;
		}
		if (vl == 0U || vl >= sizeof(buf)) {
			continue;
		}
		memcpy(buf, v, vl);
		buf[vl] = '\0';
		r = auth_role_parse(buf);
		if (r > 0) {
			if (out_explicit != NULL) {
				*out_explicit = true;
			}
			return (uint8_t)r;
		}
	}

	if (radius_attr_find(pkt, len, RADIUS_AT_SERVICE_TYPE, 0U, &v, &vl) ==
		    0 &&
	    vl == 4U) {
		uint32_t st = ((uint32_t)v[0] << 24) | ((uint32_t)v[1] << 16) |
			      ((uint32_t)v[2] << 8) | (uint32_t)v[3];

		switch (st) {
		case RADIUS_ST_ADMINISTRATIVE:
			if (out_explicit != NULL) {
				*out_explicit = true;
			}
			return (uint8_t)AUTH_ROLE_ADMIN;
		case RADIUS_ST_NAS_PROMPT:
			if (out_explicit != NULL) {
				*out_explicit = true;
			}
			return (uint8_t)AUTH_ROLE_OPERATOR;
		case RADIUS_ST_LOGIN:
		case RADIUS_ST_FRAMED:
			if (out_explicit != NULL) {
				*out_explicit = true;
			}
			return (uint8_t)AUTH_ROLE_VIEWER;
		default:
			break;
		}
	}

	return (uint8_t)AUTH_ROLE_VIEWER;
}

/* ========================================================================= */
/* request                                                                   */
/* ========================================================================= */

/** Append one attribute; -ENOSPC when it will not fit. */
static int put_attr(uint8_t *buf, size_t cap, size_t *off, uint8_t type,
		    const uint8_t *val, size_t len)
{
	if (len > 253U) {
		return -EINVAL;
	}
	if ((cap - *off) < (len + 2U)) {
		return -ENOSPC;
	}
	buf[*off] = type;
	buf[*off + 1U] = (uint8_t)(len + 2U);
	if (len != 0U) {
		memcpy(&buf[*off + 2U], val, len);
	}
	*off += len + 2U;
	return 0;
}

static int put_attr_u32(uint8_t *buf, size_t cap, size_t *off, uint8_t type,
			uint32_t v)
{
	uint8_t b[4];

	b[0] = (uint8_t)((v >> 24) & 0xFFU);
	b[1] = (uint8_t)((v >> 16) & 0xFFU);
	b[2] = (uint8_t)((v >> 8) & 0xFFU);
	b[3] = (uint8_t)(v & 0xFFU);
	return put_attr(buf, cap, off, type, b, sizeof(b));
}

int radius_build_access_request(const radius_req_t *req, uint8_t *out,
				size_t cap, size_t *out_len)
{
	uint8_t hidden[AUTH_SECRET_MAX + MD5_LEN];
	size_t hidden_len = 0U;
	size_t off;
	size_t ulen;
	size_t ma_off = 0U;
	int rc;

	if (req == NULL || out == NULL || out_len == NULL) {
		return -EINVAL;
	}
	if (req->user == NULL || req->secret == NULL || req->shared == NULL ||
	    req->authenticator == NULL || req->shared_len == 0U) {
		return -EINVAL;
	}
	ulen = strlen(req->user);
	if (ulen == 0U || ulen > AUTH_USER_MAX) {
		return -EINVAL;
	}
	if (req->state != NULL && req->state_len > RADIUS_STATE_MAX) {
		return -EINVAL;
	}
	if (cap < RADIUS_HDR_LEN || cap > RADIUS_BUF_MAX) {
		return -ENOSPC;
	}

	rc = radius_hide_password(req->shared, req->shared_len,
				  req->authenticator, req->secret, hidden,
				  sizeof(hidden), &hidden_len);
	if (rc != 0) {
		return rc;
	}

	memset(out, 0, RADIUS_HDR_LEN);
	out[0] = (uint8_t)RADIUS_CODE_ACCESS_REQUEST;
	out[1] = req->id;
	memcpy(&out[4], req->authenticator, RADIUS_AUTH_LEN);
	off = RADIUS_HDR_LEN;

	rc = put_attr(out, cap, &off, RADIUS_AT_USER_NAME,
		      (const uint8_t *)req->user, ulen);
	if (rc != 0) {
		return rc;
	}
	rc = put_attr(out, cap, &off, RADIUS_AT_USER_PASSWORD, hidden,
		      hidden_len);
	if (rc != 0) {
		return rc;
	}
	if (req->nas_ip != 0U) {
		rc = put_attr_u32(out, cap, &off, RADIUS_AT_NAS_IP_ADDRESS,
				  req->nas_ip);
		if (rc != 0) {
			return rc;
		}
	}
	if (req->nas_port_set) {
		rc = put_attr_u32(out, cap, &off, RADIUS_AT_NAS_PORT,
				  req->nas_port);
		if (rc != 0) {
			return rc;
		}
	}
	if (req->service_type != 0U) {
		rc = put_attr_u32(out, cap, &off, RADIUS_AT_SERVICE_TYPE,
				  req->service_type);
		if (rc != 0) {
			return rc;
		}
	}
	if (req->nas_port_type != 0U) {
		rc = put_attr_u32(out, cap, &off, RADIUS_AT_NAS_PORT_TYPE,
				  req->nas_port_type);
		if (rc != 0) {
			return rc;
		}
	}
	if (req->nas_id != NULL && req->nas_id[0] != '\0') {
		size_t n = strlen(req->nas_id);

		if (n > 253U) {
			n = 253U;
		}
		rc = put_attr(out, cap, &off, RADIUS_AT_NAS_IDENTIFIER,
			      (const uint8_t *)req->nas_id, n);
		if (rc != 0) {
			return rc;
		}
	}
	if (req->state != NULL && req->state_len != 0U) {
		rc = put_attr(out, cap, &off, RADIUS_AT_STATE, req->state,
			      req->state_len);
		if (rc != 0) {
			return rc;
		}
	}

	if (req->message_authenticator) {
		uint8_t zero[MD5_LEN];

		/* RFC 3579 §3.2: the attribute is appended with a zero value,
		 * the HMAC is computed over the complete datagram *including*
		 * the zeroed field and the final Length, and then written back
		 * in place. So the length has to be finalised first. */
		memset(zero, 0, sizeof(zero));
		ma_off = off;
		rc = put_attr(out, cap, &off,
			      RADIUS_AT_MESSAGE_AUTHENTICATOR, zero,
			      sizeof(zero));
		if (rc != 0) {
			return rc;
		}
	}

	if (off > RADIUS_PKT_MAX) {
		return -ENOSPC;
	}
	out[2] = (uint8_t)((off >> 8) & 0xFFU);
	out[3] = (uint8_t)(off & 0xFFU);

	if (req->message_authenticator) {
		uint8_t mac[MD5_LEN];

		auth_hmac_md5(req->shared, req->shared_len, out, off, mac);
		memcpy(&out[ma_off + 2U], mac, sizeof(mac));
	}

	*out_len = off;
	return 0;
}

/* ========================================================================= */
/* response                                                                  */
/* ========================================================================= */

/**
 * Verify a Message-Authenticator if present.
 *
 * The HMAC is over the datagram with the attribute's 16-octet value zeroed and
 * — for a reply — with the authenticator field replaced by the *request's*
 * authenticator (RFC 3579 §3.2). Both substitutions happen in a scratch copy;
 * the caller's buffer is never modified.
 *
 * @retval 0         Absent, or present and correct.
 * @retval -EBADE    Present and wrong.
 * @retval -EBADMSG  Present with a length other than 16.
 */
static int check_msg_auth(const uint8_t *pkt, size_t len,
			  const uint8_t req_auth[RADIUS_AUTH_LEN],
			  const uint8_t *shared, size_t shared_len,
			  bool *out_present)
{
	uint8_t scratch[RADIUS_BUF_MAX];
	uint8_t mac[MD5_LEN];
	const uint8_t *v = NULL;
	size_t vl = 0U;
	size_t voff;
	int rc;

	*out_present = false;
	rc = radius_attr_find(pkt, len, RADIUS_AT_MESSAGE_AUTHENTICATOR, 0U, &v,
			      &vl);
	if (rc == -ENOENT) {
		return 0;
	}
	if (rc != 0) {
		return rc;
	}
	if (vl != MD5_LEN) {
		return -EBADMSG;
	}
	if (len > sizeof(scratch)) {
		return -EBADMSG;
	}

	voff = (size_t)(v - pkt);
	memcpy(scratch, pkt, len);
	memcpy(&scratch[4], req_auth, RADIUS_AUTH_LEN);
	memset(&scratch[voff], 0, MD5_LEN);

	auth_hmac_md5(shared, shared_len, scratch, len, mac);
	if (!auth_ct_eq(mac, v, MD5_LEN)) {
		return -EBADE;
	}
	*out_present = true;
	return 0;
}

int radius_parse_response(const uint8_t *pkt, size_t len, const uint8_t *shared,
			  size_t shared_len, uint8_t req_id,
			  const uint8_t req_auth[RADIUS_AUTH_LEN],
			  radius_rsp_t *out)
{
	uint8_t want[RADIUS_AUTH_LEN];
	const uint8_t *v = NULL;
	size_t vl = 0U;
	size_t declared;
	bool ma_present = false;
	int rc;

	if (pkt == NULL || shared == NULL || req_auth == NULL || out == NULL) {
		return -EINVAL;
	}
	if (shared_len == 0U) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));

	if (len < RADIUS_PKT_MIN || len > RADIUS_BUF_MAX) {
		return -EBADMSG;
	}
	declared = ((size_t)pkt[2] << 8) | (size_t)pkt[3];
	/* RFC 2865 §3: octets past Length are padding and MUST be ignored;
	 * a Length longer than what arrived is a malformed datagram. */
	if (declared < RADIUS_PKT_MIN || declared > len) {
		return -EBADMSG;
	}
	len = declared;

	rc = radius_attrs_check(pkt, len);
	if (rc != 0) {
		return rc;
	}

	if (pkt[1] != req_id) {
		return -ENOMSG;
	}

	switch (pkt[0]) {
	case RADIUS_CODE_ACCESS_ACCEPT:
	case RADIUS_CODE_ACCESS_REJECT:
	case RADIUS_CODE_ACCESS_CHALLENGE:
		break;
	default:
		return -EPROTO;
	}

	rc = radius_response_auth(pkt, len, req_auth, shared, shared_len, want);
	if (rc != 0) {
		return rc;
	}
	if (!auth_ct_eq(want, &pkt[4], RADIUS_AUTH_LEN)) {
		return -EBADE;
	}

	rc = check_msg_auth(pkt, len, req_auth, shared, shared_len, &ma_present);
	if (rc != 0) {
		return rc;
	}

	out->code = pkt[0];
	out->id = pkt[1];
	out->have_msg_auth = ma_present;

	if (radius_attr_find(pkt, len, RADIUS_AT_REPLY_MESSAGE, 0U, &v, &vl) ==
	    0) {
		size_t n = (vl > RADIUS_REPLY_MAX) ? RADIUS_REPLY_MAX : vl;
		size_t i;

		for (i = 0U; i < n; i++) {
			/* A server-supplied string reaches the log and the UI;
			 * keep it to printable ASCII. */
			out->reply[i] = (v[i] >= 0x20U && v[i] < 0x7FU)
						? (char)v[i]
						: '.';
		}
		out->reply[n] = '\0';
	}

	if (out->code == RADIUS_CODE_ACCESS_CHALLENGE) {
		if (radius_attr_find(pkt, len, RADIUS_AT_STATE, 0U, &v, &vl) ==
			    0 &&
		    vl <= RADIUS_STATE_MAX) {
			memcpy(out->state, v, vl);
			out->state_len = vl;
		}
	}

	if (out->code == RADIUS_CODE_ACCESS_ACCEPT) {
		out->role = radius_role_from_attrs(pkt, len,
						   &out->role_explicit);
	}

	return 0;
}
