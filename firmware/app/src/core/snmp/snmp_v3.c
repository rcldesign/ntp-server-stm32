/*
 * STS1000 "Meridian" — core/snmp: SNMPv3 message processing (RFC 3412, 3414).
 *
 * See snmp_v3.h for the contract; snmp_usm.c holds the cryptography.
 *
 * Message served:
 *
 *   SEQUENCE {
 *     INTEGER  msgVersion (3)
 *     SEQUENCE {                                  -- msgGlobalData
 *       INTEGER       msgID
 *       INTEGER       msgMaxSize
 *       OCTET STRING  msgFlags (1 octet: auth|priv|reportable)
 *       INTEGER       msgSecurityModel (3 = USM)
 *     }
 *     OCTET STRING {                              -- msgSecurityParameters
 *       SEQUENCE {
 *         OCTET STRING  msgAuthoritativeEngineID
 *         INTEGER       msgAuthoritativeEngineBoots
 *         INTEGER       msgAuthoritativeEngineTime
 *         OCTET STRING  msgUserName
 *         OCTET STRING  msgAuthenticationParameters
 *         OCTET STRING  msgPrivacyParameters
 *       }
 *     }
 *     -- msgData: the scopedPDU, plain or encrypted
 *     SEQUENCE { OCTET STRING contextEngineID, OCTET STRING contextName, PDU }
 *       | OCTET STRING (AES-128-CFB of the above)
 *   }
 *
 * ---------------------------------------------------------------------------
 * Locating msgAuthenticationParameters by re-parsing our own output
 * ---------------------------------------------------------------------------
 *
 * RFC 3414 §6.3.1 computes the digest over the whole serialised message with the
 * authentication field zeroed at its final length, then writes the digest into
 * that field. So the builder needs the field's byte offset in the finished
 * message — and the BER writer shrinks reserved length octets on close, moving
 * everything after them by up to two octets per nesting level.
 *
 * Rather than carry a fix-up list through three levels of memmove, the builder
 * finishes the message with the field zero-filled and then *parses its own
 * output* with the same parser the receive path uses, which reports the offset.
 * It costs one extra parse per transmitted message and it cannot be wrong by
 * construction: if the offset were wrong the message would not have parsed.
 */

#include "snmp/snmp_v3.h"

#include <errno.h>
#include <string.h>

#include "snmp/snmp_internal.h"

/** Largest serialised USM security-parameters blob. */
#define V3_USM_PARAMS_MAX 160U

/** Largest contextEngineID/contextName pair plus the scopedPDU SEQUENCE header. */
#define V3_SCOPED_OVERHEAD (3U + 2U + SNMP_V3_ENGINEID_MAX + 2U)

/* ========================================================================= */
/* usmStats OIDs                                                             */
/* ========================================================================= */

/* 1.3.6.1.6.3.15.1.1.<n>.0 — snmpUsmMIB usmStats group (RFC 3414 §5). */
static const uint32_t o_usm_stats[] = { 1U, 3U, 6U, 1U, 6U, 3U, 15U, 1U, 1U };

#define USM_STAT_UNSUPPORTED_SEC_LEVELS 1U
#define USM_STAT_NOT_IN_TIME_WINDOWS 2U
#define USM_STAT_UNKNOWN_USER_NAMES 3U
#define USM_STAT_UNKNOWN_ENGINE_IDS 4U
#define USM_STAT_WRONG_DIGESTS 5U
#define USM_STAT_DECRYPTION_ERRORS 6U

int snmp_usm_stats_oid(unsigned int which, uint32_t *arcs)
{
	size_t base = sizeof(o_usm_stats) / sizeof(o_usm_stats[0]);

	if (arcs == NULL || which < 1U || which > 6U) {
		return -EINVAL;
	}
	memcpy(arcs, o_usm_stats, base * sizeof(arcs[0]));
	arcs[base] = which;
	arcs[base + 1U] = 0U; /* the scalar instance */
	return (int)(base + 2U);
}

/* ========================================================================= */
/* engine id                                                                 */
/* ========================================================================= */

int snmp_v3_engine_id_build(uint32_t pen, const uint8_t *uniq, size_t uniq_len,
			    uint8_t *out, size_t cap)
{
	size_t n;

	if (out == NULL || uniq == NULL || uniq_len == 0U) {
		return -EINVAL;
	}
	if (pen > 0x7FFFFFFFU) {
		return -EINVAL;
	}

	n = uniq_len;
	if (n > (SNMP_V3_ENGINEID_MAX - 5U)) {
		n = SNMP_V3_ENGINEID_MAX - 5U;
	}
	if (cap < (5U + n)) {
		return -ENOSPC;
	}

	/* RFC 3411 §5: four octets of enterprise number with the most
	 * significant bit set, then a format octet. Format 5 is
	 * "administratively assigned octets", which is what a serial number is. */
	out[0] = (uint8_t)(0x80U | ((pen >> 24) & 0x7FU));
	out[1] = (uint8_t)((pen >> 16) & 0xFFU);
	out[2] = (uint8_t)((pen >> 8) & 0xFFU);
	out[3] = (uint8_t)(pen & 0xFFU);
	out[4] = 5U;
	memcpy(&out[5], uniq, n);
	return (int)(5U + n);
}

/* ========================================================================= */
/* context                                                                   */
/* ========================================================================= */

int snmp_v3_init(snmp_v3_ctx_t *c, const snmp_v3_cfg_t *cfg)
{
	if (c == NULL || cfg == NULL || cfg->ports.crypto == NULL) {
		return -EINVAL;
	}
	if (cfg->ports.crypto->hmac_sha256 == NULL ||
	    cfg->ports.crypto->sha256 == NULL) {
		return -ENOTSUP;
	}
	if (cfg->engine_id == NULL ||
	    cfg->engine_id_len < SNMP_V3_ENGINEID_MIN ||
	    cfg->engine_id_len > SNMP_V3_ENGINEID_MAX) {
		return -EINVAL;
	}

	memset(c, 0, sizeof(*c));
	c->cfg = *cfg;
	if (c->cfg.max_msg_size < SNMP_V3_MSG_MAX_SIZE_MIN) {
		c->cfg.max_msg_size = SNMP_PKT_MAX;
	}
	memcpy(c->engine_id, cfg->engine_id, cfg->engine_id_len);
	c->engine_id_len = cfg->engine_id_len;
	/* The stored pointer would dangle if the caller's buffer went away; the
	 * copy above is authoritative from here on. */
	c->cfg.engine_id = c->engine_id;
	c->ready = true;
	return 0;
}

int snmp_v3_set_engine_id(snmp_v3_ctx_t *c, const uint8_t *id, size_t len)
{
	if (c == NULL || !c->ready || id == NULL) {
		return -EINVAL;
	}
	if (len < SNMP_V3_ENGINEID_MIN || len > SNMP_V3_ENGINEID_MAX) {
		return -EINVAL;
	}
	memcpy(c->engine_id, id, len);
	c->engine_id_len = len;
	c->cfg.engine_id = c->engine_id;
	c->cfg.engine_id_len = len;
	/* Localised keys are bound to the engine id (RFC 3414 §2.6). Keeping
	 * them would leave an agent that looks configured and authenticates
	 * nobody. */
	snmp_v3_users_clear(c);
	return 0;
}

int snmp_v3_get_engine_id(const snmp_v3_ctx_t *c, uint8_t *out, size_t *out_len)
{
	if (c == NULL || !c->ready || out == NULL || out_len == NULL) {
		return -EINVAL;
	}
	memcpy(out, c->engine_id, c->engine_id_len);
	*out_len = c->engine_id_len;
	return 0;
}

int snmp_v3_set_boots(snmp_v3_ctx_t *c, uint32_t boots)
{
	if (c == NULL || !c->ready) {
		return -EINVAL;
	}
	/* RFC 3414 §2.2.2: 2147483647 is the terminal value; an engine that
	 * reaches it must be re-keyed rather than roll over. Clamping there
	 * makes every later timeliness check fail, which is the specified
	 * behaviour and is safe. */
	if (boots > 0x7FFFFFFFU) {
		boots = 0x7FFFFFFFU;
	}
	c->cfg.engine_boots = boots;
	return 0;
}

uint32_t snmp_v3_get_boots(const snmp_v3_ctx_t *c)
{
	return (c != NULL) ? c->cfg.engine_boots : 0U;
}

void snmp_v3_users_clear(snmp_v3_ctx_t *c)
{
	if (c == NULL) {
		return;
	}
	memset(c->users, 0, sizeof(c->users));
}

size_t snmp_v3_user_count(const snmp_v3_ctx_t *c)
{
	size_t n = 0U;
	size_t i;

	if (c == NULL) {
		return 0U;
	}
	for (i = 0U; i < SNMP_V3_USERS; i++) {
		if (c->users[i].used) {
			n++;
		}
	}
	return n;
}

static snmp_v3_user_t *user_find(snmp_v3_ctx_t *c, const char *name,
				 size_t name_len)
{
	size_t i;

	for (i = 0U; i < SNMP_V3_USERS; i++) {
		if (!c->users[i].used) {
			continue;
		}
		if (strlen(c->users[i].name) != name_len) {
			continue;
		}
		if (name_len == 0U ||
		    memcmp(c->users[i].name, name, name_len) == 0) {
			return &c->users[i];
		}
	}
	return NULL;
}

/** Derive one localised key from either a passphrase or a ready-made key. */
static int load_key(snmp_v3_ctx_t *c, uint8_t proto, const void *secret,
		    size_t len, bool localized, uint8_t *out, uint8_t *out_len)
{
	size_t klen = snmp_usm_key_len(proto);
	size_t got = 0U;
	int rc;

	if (klen == 0U) {
		*out_len = 0U;
		return 0;
	}
	if (secret == NULL) {
		return -EINVAL;
	}

	if (localized) {
		if (len != klen) {
			return -EINVAL;
		}
		memcpy(out, secret, klen);
		*out_len = (uint8_t)klen;
		return 0;
	}

	/* A passphrase must be NUL-terminated within `len`: it is stored in a
	 * cfg blob, so its length is what tells us where it ends. */
	{
		const char *s = (const char *)secret;
		size_t i;

		for (i = 0U; i < len; i++) {
			if (s[i] == '\0') {
				break;
			}
		}
		if (i == 0U || i > SNMP_V3_PASSPHRASE_MAX) {
			return -EINVAL;
		}
		if (i < SNMP_V3_PASSPHRASE_MIN) {
			return -EINVAL;
		}
		if (i == len) {
			/* Not terminated inside the blob — copy and terminate. */
			char tmp[SNMP_V3_PASSPHRASE_MAX + 1U];

			memcpy(tmp, s, i);
			tmp[i] = '\0';
			rc = snmp_usm_password_to_localized(&c->cfg.ports, proto,
							    tmp, c->engine_id,
							    c->engine_id_len,
							    out, &got);
		} else {
			rc = snmp_usm_password_to_localized(&c->cfg.ports, proto,
							    s, c->engine_id,
							    c->engine_id_len,
							    out, &got);
		}
	}
	if (rc != 0) {
		return rc;
	}
	*out_len = (uint8_t)got;
	return 0;
}

int snmp_v3_user_set(snmp_v3_ctx_t *c, const char *name, uint8_t auth_proto,
		     const void *auth_secret, size_t auth_len,
		     bool auth_localized, uint8_t priv_proto,
		     const void *priv_secret, size_t priv_len,
		     bool priv_localized)
{
	snmp_v3_user_t tmp;
	snmp_v3_user_t *slot;
	size_t nlen;
	size_t i;
	int rc;

	if (c == NULL || !c->ready || name == NULL) {
		return -EINVAL;
	}
	nlen = strlen(name);
	if (nlen == 0U || nlen > SNMP_V3_USER_MAX) {
		return -EINVAL;
	}
	if (auth_proto > (uint8_t)SNMP_AUTH_HMAC_SHA256_192 ||
	    priv_proto > (uint8_t)SNMP_PRIV_AES128_CFB) {
		return -EINVAL;
	}
	/* RFC 3414 defines no privacy-without-authentication level; allowing it
	 * would hand an unauthenticated peer a decryption oracle. */
	if (priv_proto != (uint8_t)SNMP_PRIV_NONE &&
	    auth_proto == (uint8_t)SNMP_AUTH_NONE) {
		return -EINVAL;
	}
	if (c->engine_id_len == 0U) {
		return -EPERM;
	}

	memset(&tmp, 0, sizeof(tmp));
	memcpy(tmp.name, name, nlen);
	tmp.auth_proto = auth_proto;
	tmp.priv_proto = priv_proto;

	if (auth_proto != (uint8_t)SNMP_AUTH_NONE) {
		rc = load_key(c, auth_proto, auth_secret, auth_len,
			      auth_localized, tmp.auth_key, &tmp.auth_key_len);
		if (rc != 0) {
			return rc;
		}
	}
	if (priv_proto != (uint8_t)SNMP_PRIV_NONE) {
		/*
		 * RFC 3826 §3.1.2.1: the privacy key is localised with the
		 * *auth* protocol's hash and its first 16 octets are the
		 * AES-128 key. So a SHA-1 user's privacy key is 20 octets long
		 * and a SHA-256 user's is 32; either way only 16 are used.
		 */
		rc = load_key(c, auth_proto, priv_secret, priv_len,
			      priv_localized, tmp.priv_key, &tmp.priv_key_len);
		if (rc != 0) {
			return rc;
		}
		if (tmp.priv_key_len < SNMP_V3_PRIV_KEY_LEN) {
			return -EINVAL;
		}
	}
	tmp.used = true;

	slot = user_find(c, name, nlen);
	if (slot == NULL) {
		for (i = 0U; i < SNMP_V3_USERS; i++) {
			if (!c->users[i].used) {
				slot = &c->users[i];
				break;
			}
		}
	}
	if (slot == NULL) {
		return -ENOSPC;
	}
	*slot = tmp;
	return 0;
}

int snmp_v3_user_clear(snmp_v3_ctx_t *c, const char *name)
{
	snmp_v3_user_t *u;

	if (c == NULL || !c->ready || name == NULL) {
		return -EINVAL;
	}
	u = user_find(c, name, strlen(name));
	if (u == NULL) {
		return -ENOENT;
	}
	memset(u, 0, sizeof(*u));
	return 0;
}

/** The security level a user's key material demands. */
static uint8_t user_level(const snmp_v3_user_t *u)
{
	if (u->priv_proto != (uint8_t)SNMP_PRIV_NONE) {
		return (uint8_t)SNMP_SEC_AUTH_PRIV;
	}
	if (u->auth_proto != (uint8_t)SNMP_AUTH_NONE) {
		return (uint8_t)SNMP_SEC_AUTH_NOPRIV;
	}
	return (uint8_t)SNMP_SEC_NOAUTH_NOPRIV;
}

int snmp_v3_user_level(const snmp_v3_ctx_t *c, const char *name,
		       uint8_t *out_level)
{
	const snmp_v3_user_t *u;

	if (c == NULL || !c->ready || name == NULL || out_level == NULL) {
		return -EINVAL;
	}
	u = user_find((snmp_v3_ctx_t *)c, name, strlen(name));
	if (u == NULL) {
		return -ENOENT;
	}
	*out_level = user_level(u);
	return 0;
}

int snmp_v3_stats_get(const snmp_v3_ctx_t *c, snmp_v3_stats_t *out)
{
	if (c == NULL || out == NULL) {
		return -EINVAL;
	}
	*out = c->stats;
	return 0;
}

int snmp_v3_stats_reset(snmp_v3_ctx_t *c)
{
	if (c == NULL) {
		return -EINVAL;
	}
	memset(&c->stats, 0, sizeof(c->stats));
	return 0;
}

/* ========================================================================= */
/* parse                                                                     */
/* ========================================================================= */

/** A parsed v3 message. Pointers reference the buffer that was parsed. */
typedef struct {
	int32_t msg_id;
	uint32_t max_size;
	uint8_t flags;
	int32_t sec_model;

	uint8_t eid[SNMP_V3_ENGINEID_MAX];
	size_t eid_len;
	uint32_t boots;
	uint32_t time_s;
	char user[SNMP_V3_USER_MAX + 1U];
	size_t user_len;

	size_t authparam_off; /**< absolute offset in the parsed buffer */
	size_t authparam_len;
	const uint8_t *privparam;
	size_t privparam_len;

	size_t scoped_off; /**< absolute offset of the scopedPDU content */
	size_t scoped_len;
	bool encrypted;
} v3_msg_t;

/** Security level the msgFlags describe. */
static uint8_t flags_level(uint8_t flags)
{
	if ((flags & SNMP_V3_FLAG_PRIV) != 0U) {
		return (uint8_t)SNMP_SEC_AUTH_PRIV;
	}
	if ((flags & SNMP_V3_FLAG_AUTH) != 0U) {
		return (uint8_t)SNMP_SEC_AUTH_NOPRIV;
	}
	return (uint8_t)SNMP_SEC_NOAUTH_NOPRIV;
}

/**
 * Parse a whole v3 message.
 *
 * @retval 0         Parsed.
 * @retval -EPROTO   msgVersion is not 3.
 * @retval -EBADMSG  Anything else wrong.
 */
static int parse_v3(const uint8_t *msg, size_t len, v3_msg_t *m)
{
	snmp_rd_t r;
	snmp_rd_t body;
	snmp_rd_t gd;
	snmp_rd_t sp;
	snmp_rd_t usm;
	uint8_t tag;
	size_t clen;
	int64_t v64;
	const uint8_t *oct;
	size_t oct_len;
	int rc;

	memset(m, 0, sizeof(*m));

	rc = snmp_rd_init(&r, msg, len);
	if (rc != 0) {
		return -EBADMSG;
	}
	rc = snmp_rd_hdr(&r, &tag, &clen);
	if (rc != 0 || tag != SNMP_TAG_SEQUENCE) {
		return -EBADMSG;
	}
	rc = snmp_rd_init(&body, &msg[r.off], clen);
	if (rc != 0) {
		return -EBADMSG;
	}

	rc = snmp_rd_int(&body, &v64);
	if (rc != 0) {
		return -EBADMSG;
	}
	if (v64 != SNMP_VERSION_3) {
		return -EPROTO;
	}

	/* ---- msgGlobalData ---- */
	rc = snmp_rd_hdr(&body, &tag, &clen);
	if (rc != 0 || tag != SNMP_TAG_SEQUENCE) {
		return -EBADMSG;
	}
	rc = snmp_rd_init(&gd, &body.buf[body.off], clen);
	if (rc != 0) {
		return -EBADMSG;
	}
	if (snmp_rd_skip(&body, clen) != 0) {
		return -EBADMSG;
	}

	if (snmp_rd_int(&gd, &v64) != 0) {
		return -EBADMSG;
	}
	if (v64 < 0 || v64 > INT32_MAX) {
		return -EBADMSG;
	}
	m->msg_id = (int32_t)v64;

	if (snmp_rd_int(&gd, &v64) != 0) {
		return -EBADMSG;
	}
	if (v64 < 0) {
		return -EBADMSG;
	}
	m->max_size = (v64 > (int64_t)0xFFFFFFFF) ? 0xFFFFFFFFU
						  : (uint32_t)v64;

	if (snmp_rd_octets(&gd, &oct, &oct_len) != 0 || oct_len != 1U) {
		return -EBADMSG;
	}
	m->flags = oct[0];

	if (snmp_rd_int(&gd, &v64) != 0) {
		return -EBADMSG;
	}
	if (v64 < 0 || v64 > INT32_MAX) {
		return -EBADMSG;
	}
	m->sec_model = (int32_t)v64;

	/* RFC 3412 §6.4: privacy without authentication is not a level that
	 * exists. Reject the combination outright rather than reasoning about
	 * an unauthenticated ciphertext. */
	if ((m->flags & SNMP_V3_FLAG_PRIV) != 0U &&
	    (m->flags & SNMP_V3_FLAG_AUTH) == 0U) {
		return -EBADMSG;
	}

	/* ---- msgSecurityParameters ---- */
	rc = snmp_rd_hdr(&body, &tag, &clen);
	if (rc != 0 || tag != SNMP_TAG_OCTET_STRING) {
		return -EBADMSG;
	}
	rc = snmp_rd_init(&sp, &body.buf[body.off], clen);
	if (rc != 0) {
		return -EBADMSG;
	}
	if (snmp_rd_skip(&body, clen) != 0) {
		return -EBADMSG;
	}

	rc = snmp_rd_hdr(&sp, &tag, &clen);
	if (rc != 0 || tag != SNMP_TAG_SEQUENCE) {
		return -EBADMSG;
	}
	rc = snmp_rd_init(&usm, &sp.buf[sp.off], clen);
	if (rc != 0) {
		return -EBADMSG;
	}

	if (snmp_rd_octets(&usm, &oct, &oct_len) != 0) {
		return -EBADMSG;
	}
	if (oct_len > SNMP_V3_ENGINEID_MAX) {
		return -EBADMSG;
	}
	if (oct_len != 0U) {
		memcpy(m->eid, oct, oct_len);
	}
	m->eid_len = oct_len;

	if (snmp_rd_int(&usm, &v64) != 0 || v64 < 0 || v64 > INT32_MAX) {
		return -EBADMSG;
	}
	m->boots = (uint32_t)v64;
	if (snmp_rd_int(&usm, &v64) != 0 || v64 < 0 || v64 > INT32_MAX) {
		return -EBADMSG;
	}
	m->time_s = (uint32_t)v64;

	if (snmp_rd_octets(&usm, &oct, &oct_len) != 0) {
		return -EBADMSG;
	}
	if (oct_len > SNMP_V3_USER_MAX) {
		return -EBADMSG;
	}
	if (oct_len != 0U) {
		memcpy(m->user, oct, oct_len);
	}
	m->user[oct_len] = '\0';
	m->user_len = oct_len;

	if (snmp_rd_octets(&usm, &oct, &oct_len) != 0) {
		return -EBADMSG;
	}
	if (oct_len > SNMP_V3_AUTHPARAM_MAX) {
		return -EBADMSG;
	}
	m->authparam_off = (size_t)(oct - msg);
	m->authparam_len = oct_len;

	if (snmp_rd_octets(&usm, &oct, &oct_len) != 0) {
		return -EBADMSG;
	}
	if (oct_len > SNMP_V3_SALT_LEN) {
		return -EBADMSG;
	}
	m->privparam = oct;
	m->privparam_len = oct_len;

	/* ---- msgData ---- */
	rc = snmp_rd_hdr(&body, &tag, &clen);
	if (rc != 0) {
		return -EBADMSG;
	}
	if (tag == SNMP_TAG_OCTET_STRING) {
		m->encrypted = true;
	} else if (tag == SNMP_TAG_SEQUENCE) {
		m->encrypted = false;
		/* The scopedPDU *is* the SEQUENCE, header included, when it is
		 * not encrypted — but the parser below wants its content, so
		 * record the content and re-open it there. */
	} else {
		return -EBADMSG;
	}
	m->scoped_off = (size_t)(&body.buf[body.off] - msg);
	m->scoped_len = clen;
	return 0;
}

/**
 * Parse a scopedPDU's content into @p rq.
 *
 * @param content  contextEngineID onwards.
 * @retval 0         Parsed.
 * @retval -EBADMSG  Malformed.
 * @retval -ENOTSUP  A PDU tag this agent does not serve.
 * @retval -E2BIG    Too many varbinds (reqid still valid).
 * @retval -ENOENT   Wrong contextEngineID or a non-empty contextName.
 */
static int parse_scoped(snmp_v3_ctx_t *c, const uint8_t *content, size_t len,
			snmp_req_t *rq)
{
	snmp_rd_t r;
	const uint8_t *oct;
	size_t oct_len;
	uint8_t tag;
	size_t clen;

	if (snmp_rd_init(&r, content, len) != 0) {
		return -EBADMSG;
	}
	if (snmp_rd_octets(&r, &oct, &oct_len) != 0) {
		return -EBADMSG;
	}
	/* One context, so an empty contextEngineID (what most managers send)
	 * and our own are the only acceptable values. */
	if (oct_len != 0U) {
		if (oct_len != c->engine_id_len ||
		    memcmp(oct, c->engine_id, oct_len) != 0) {
			return -ENOENT;
		}
	}
	if (snmp_rd_octets(&r, &oct, &oct_len) != 0) {
		return -EBADMSG;
	}
	if (oct_len != 0U) {
		return -ENOENT; /* the default context is the only one */
	}
	if (snmp_rd_hdr(&r, &tag, &clen) != 0) {
		return -EBADMSG;
	}
	return snmp__parse_pdu(tag, &r.buf[r.off], clen, rq);
}

/* ========================================================================= */
/* build                                                                     */
/* ========================================================================= */

/** What emit_message() needs to know about the security wrapper. */
typedef struct {
	int32_t msg_id;
	uint8_t flags;
	const char *user;
	const snmp_v3_user_t *keys; /**< NULL for an unauthenticated message */
	uint32_t boots;
	uint32_t time_s;
} v3_sec_t;

/** Writes the PDU (not the scopedPDU wrapper) into @p w. */
typedef int (*v3_pdu_fn)(snmp_wr_t *w, void *ctx);

/** Serialise the USM security parameters with a zero-filled digest field. */
static int emit_usm_params(const snmp_v3_ctx_t *c, const v3_sec_t *sec,
			   const uint8_t *salt, size_t salt_len, uint8_t *buf,
			   size_t cap, size_t *out_len)
{
	snmp_wr_t w;
	uint8_t zero[SNMP_V3_AUTHPARAM_MAX];
	size_t alen = 0U;
	size_t mark;
	int rc;

	if (sec->keys != NULL && (sec->flags & SNMP_V3_FLAG_AUTH) != 0U) {
		alen = snmp_usm_authparam_len(sec->keys->auth_proto);
	}
	memset(zero, 0, sizeof(zero));

	rc = snmp_wr_init(&w, buf, cap);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_begin(&w, SNMP_TAG_SEQUENCE, &mark);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_tlv(&w, SNMP_TAG_OCTET_STRING, c->engine_id,
			 c->engine_id_len);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_int(&w, SNMP_TAG_INTEGER, (int64_t)sec->boots);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_int(&w, SNMP_TAG_INTEGER, (int64_t)sec->time_s);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_tlv(&w, SNMP_TAG_OCTET_STRING,
			 (const uint8_t *)sec->user,
			 (sec->user != NULL) ? strlen(sec->user) : 0U);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_tlv(&w, SNMP_TAG_OCTET_STRING, zero, alen);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_tlv(&w, SNMP_TAG_OCTET_STRING, salt, salt_len);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_end(&w, mark);
	if (rc != 0) {
		return rc;
	}
	*out_len = snmp_wr_len(&w);
	return 0;
}

/** Write the scopedPDU SEQUENCE (context ids plus the PDU). */
static int emit_scoped(const snmp_v3_ctx_t *c, snmp_wr_t *w, v3_pdu_fn pdu,
		       void *pdu_ctx)
{
	size_t mark;
	int rc;

	rc = snmp_wr_begin(w, SNMP_TAG_SEQUENCE, &mark);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_tlv(w, SNMP_TAG_OCTET_STRING, c->engine_id,
			 c->engine_id_len);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_tlv(w, SNMP_TAG_OCTET_STRING, NULL, 0U); /* contextName */
	if (rc != 0) {
		return rc;
	}
	rc = pdu(w, pdu_ctx);
	if (rc != 0) {
		return rc;
	}
	return snmp_wr_end(w, mark);
}

/** Draw a privacy salt that cannot repeat within a boot. */
static int make_salt(snmp_v3_ctx_t *c, uint8_t salt[SNMP_V3_SALT_LEN])
{
	const port_crypto_t *cr = c->cfg.ports.crypto;
	uint32_t n;

	if (cr == NULL || cr->rand == NULL) {
		return -ENOTSUP;
	}
	if (cr->rand(cr->ctx, salt, SNMP_V3_SALT_LEN) != 0) {
		return -EIO;
	}
	/*
	 * RFC 3826 §3.1.2.1 requires the salt never to repeat for a given key.
	 * A CSPRNG should guarantee that, but a silent RNG failure that returned
	 * a constant would produce a repeating IV and destroy the confidentiality
	 * of every message — so the low four octets are overwritten with a
	 * monotonic counter. The IV is then unique by construction, and still
	 * unpredictable in its high four octets.
	 */
	c->salt_counter++;
	n = c->salt_counter;
	salt[4] = (uint8_t)((n >> 24) & 0xFFU);
	salt[5] = (uint8_t)((n >> 16) & 0xFFU);
	salt[6] = (uint8_t)((n >> 8) & 0xFFU);
	salt[7] = (uint8_t)(n & 0xFFU);
	return 0;
}

/**
 * Build a complete v3 message: envelope, security parameters, scopedPDU
 * (encrypted when asked), then the authentication digest.
 */
static int emit_message(snmp_v3_ctx_t *c, const v3_sec_t *sec, v3_pdu_fn pdu,
			void *pdu_ctx, uint8_t *out, size_t cap,
			size_t *out_len)
{
	uint8_t params[V3_USM_PARAMS_MAX];
	uint8_t salt[SNMP_V3_SALT_LEN];
	uint8_t iv[SNMP_V3_AES_BLOCK];
	uint8_t digest[SNMP_V3_AUTHPARAM_MAX];
	snmp_wr_t w;
	v3_msg_t back;
	size_t params_len = 0U;
	size_t salt_len = 0U;
	size_t m_msg;
	size_t m_gd;
	size_t alen = 0U;
	bool want_auth = (sec->flags & SNMP_V3_FLAG_AUTH) != 0U;
	bool want_priv = (sec->flags & SNMP_V3_FLAG_PRIV) != 0U;
	int rc;

	if (want_priv) {
		rc = make_salt(c, salt);
		if (rc != 0) {
			return rc;
		}
		salt_len = SNMP_V3_SALT_LEN;
		rc = snmp_usm_priv_iv(sec->boots, sec->time_s, salt, iv);
		if (rc != 0) {
			return rc;
		}
	}

	rc = emit_usm_params(c, sec, want_priv ? salt : NULL, salt_len, params,
			     sizeof(params), &params_len);
	if (rc != 0) {
		return rc;
	}

	rc = snmp_wr_init(&w, out, cap);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_begin(&w, SNMP_TAG_SEQUENCE, &m_msg);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_int(&w, SNMP_TAG_INTEGER, SNMP_VERSION_3);
	if (rc != 0) {
		return rc;
	}

	rc = snmp_wr_begin(&w, SNMP_TAG_SEQUENCE, &m_gd);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_int(&w, SNMP_TAG_INTEGER, sec->msg_id);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_int(&w, SNMP_TAG_INTEGER, (int64_t)c->cfg.max_msg_size);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_tlv(&w, SNMP_TAG_OCTET_STRING, &sec->flags, 1U);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_int(&w, SNMP_TAG_INTEGER, SNMP_V3_SEC_MODEL_USM);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_end(&w, m_gd);
	if (rc != 0) {
		return rc;
	}

	rc = snmp_wr_tlv(&w, SNMP_TAG_OCTET_STRING, params, params_len);
	if (rc != 0) {
		return rc;
	}

	if (want_priv) {
		size_t m_enc;
		size_t start;

		rc = snmp_wr_begin(&w, SNMP_TAG_OCTET_STRING, &m_enc);
		if (rc != 0) {
			return rc;
		}
		start = snmp_wr_len(&w);
		rc = emit_scoped(c, &w, pdu, pdu_ctx);
		if (rc != 0) {
			return rc;
		}
		/* Encrypt before the length is shrunk: snmp_wr_end() only
		 * memmoves the ciphertext, which leaves it intact and in
		 * order. AES-CFB needs no padding, so the length is unchanged
		 * (RFC 3826 §3.1.4). */
		rc = snmp_usm_aes_cfb(&c->cfg.ports, sec->keys->priv_key, iv,
				      &out[start], &out[start],
				      snmp_wr_len(&w) - start, true);
		if (rc != 0) {
			return rc;
		}
		rc = snmp_wr_end(&w, m_enc);
	} else {
		rc = emit_scoped(c, &w, pdu, pdu_ctx);
	}
	if (rc != 0) {
		return rc;
	}

	rc = snmp_wr_end(&w, m_msg);
	if (rc != 0) {
		return rc;
	}
	*out_len = snmp_wr_len(&w);

	if (!want_auth) {
		return 0;
	}

	/* Locate the (currently zero-filled) digest field by parsing what we
	 * just built — see the file comment. */
	rc = parse_v3(out, *out_len, &back);
	if (rc != 0) {
		return -EPROTO;
	}
	alen = snmp_usm_authparam_len(sec->keys->auth_proto);
	if (back.authparam_len != alen || alen == 0U) {
		return -EPROTO;
	}
	rc = snmp_usm_auth(&c->cfg.ports, sec->keys->auth_proto,
			   sec->keys->auth_key, sec->keys->auth_key_len, out,
			   *out_len, digest, NULL);
	if (rc != 0) {
		return rc;
	}
	memcpy(&out[back.authparam_off], digest, alen);
	return 0;
}

/* ========================================================================= */
/* reports                                                                   */
/* ========================================================================= */

typedef struct {
	unsigned int which;
	uint64_t value;
} report_ctx_t;

static int fill_report_vb(snmp_wr_t *w, void *ctx)
{
	report_ctx_t *rc_ctx = (report_ctx_t *)ctx;
	uint32_t arcs[SNMP_OID_MAX_LEN];
	snmp_value_t v;
	int n;

	n = snmp_usm_stats_oid(rc_ctx->which, arcs);
	if (n < 0) {
		return n;
	}
	/* RFC 3414 §5: the usmStats objects are Counter32. */
	snmp_val_uint(&v, SNMP_TAG_COUNTER32,
		      rc_ctx->value & 0xFFFFFFFFU);
	return snmp__put_varbind(w, arcs, (size_t)n, &v);
}

typedef struct {
	report_ctx_t vb;
	int32_t reqid;
} report_pdu_ctx_t;

static int emit_report_pdu(snmp_wr_t *w, void *ctx)
{
	report_pdu_ctx_t *rp = (report_pdu_ctx_t *)ctx;

	return snmp__emit_pdu(w, SNMP_PDU_REPORT, rp->reqid, 0, 0,
			      fill_report_vb, &rp->vb);
}

/** The counter a usmStats index names, so a Report carries the live value. */
static uint64_t stat_value(const snmp_v3_ctx_t *c, unsigned int which)
{
	switch (which) {
	case USM_STAT_UNSUPPORTED_SEC_LEVELS:
		return c->stats.unsupported_sec_levels;
	case USM_STAT_NOT_IN_TIME_WINDOWS:
		return c->stats.not_in_time_windows;
	case USM_STAT_UNKNOWN_USER_NAMES:
		return c->stats.unknown_user_names;
	case USM_STAT_UNKNOWN_ENGINE_IDS:
		return c->stats.unknown_engine_ids;
	case USM_STAT_WRONG_DIGESTS:
		return c->stats.wrong_digests;
	case USM_STAT_DECRYPTION_ERRORS:
		return c->stats.decryption_errors;
	default:
		return 0U;
	}
}

/**
 * Build a Report message.
 *
 * @param keys  Non-NULL and @p authed true builds an authenticated Report,
 *              which RFC 3414 §3.2(7) needs for the notInTimeWindows case so a
 *              manager can trust the boots/time it learns. Every other report
 *              goes out unauthenticated: they are answers to messages this
 *              engine could not authenticate in the first place.
 */
static int build_report(snmp_v3_ctx_t *c, const v3_msg_t *in,
			unsigned int which, const snmp_v3_user_t *keys,
			bool authed, uint32_t engine_time_s, int32_t reqid,
			uint8_t *out, size_t cap, size_t *out_len)
{
	v3_sec_t sec;
	report_pdu_ctx_t rp;
	int rc;

	memset(&sec, 0, sizeof(sec));
	sec.msg_id = in->msg_id;
	/* A Report is never itself reportable (RFC 3412 §6.4). */
	sec.flags = authed ? (uint8_t)SNMP_V3_FLAG_AUTH : 0U;
	sec.user = (authed && keys != NULL) ? keys->name : "";
	sec.keys = authed ? keys : NULL;
	sec.boots = c->cfg.engine_boots;
	sec.time_s = engine_time_s;

	rp.reqid = reqid;
	rp.vb.which = which;
	rp.vb.value = stat_value(c, which);

	rc = emit_message(c, &sec, emit_report_pdu, &rp, out, cap, out_len);
	if (rc == 0) {
		c->stats.reports++;
	}
	return rc;
}

/* ========================================================================= */
/* handling                                                                  */
/* ========================================================================= */

typedef struct {
	snmp_ctx_t *agent;
	snmp_req_t *rq;
	int32_t err;
	int32_t err_index;
	uint32_t uptime_cs;
} response_ctx_t;

static int emit_response_pdu(snmp_wr_t *w, void *ctx)
{
	response_ctx_t *r = (response_ctx_t *)ctx;

	return snmp__emit_response(r->agent, r->rq, r->err, r->err_index,
				   r->uptime_cs, w);
}

int snmp_v3_handle(snmp_ctx_t *agent, snmp_v3_ctx_t *c, const uint8_t *req,
		   size_t req_len, uint32_t engine_time_s, uint32_t uptime_cs,
		   uint8_t *rsp, size_t rsp_cap, size_t *rsp_len)
{
	v3_msg_t m;
	snmp_req_t rq;
	snmp_v3_user_t *u = NULL;
	response_ctx_t rctx;
	v3_sec_t sec;
	const uint8_t *scoped;
	uint8_t level;
	uint8_t digest[SNMP_V3_AUTHPARAM_MAX];
	bool reportable;
	size_t cap;
	int32_t reqid = 0;
	int rc;

	if (agent == NULL || c == NULL || !c->ready || req == NULL ||
	    rsp == NULL || rsp_len == NULL) {
		return -EINVAL;
	}
	if (req_len == 0U || req_len > SNMP_PKT_MAX) {
		c->stats.rx++;
		c->stats.malformed++;
		return -EBADMSG;
	}
	if (rsp_cap < SNMP_V3_MSG_MAX_SIZE_MIN) {
		return -ENOSPC;
	}

	c->stats.rx++;
	memset(&rq, 0, sizeof(rq));

	rc = parse_v3(req, req_len, &m);
	if (rc == -EPROTO) {
		return -EPROTO;
	}
	if (rc != 0) {
		c->stats.malformed++;
		return -EBADMSG;
	}

	if (m.sec_model != SNMP_V3_SEC_MODEL_USM) {
		/* No usmStats counter covers this — USM was not even selected. */
		c->stats.bad_sec_model++;
		return -EPROTO;
	}

	reportable = (m.flags & SNMP_V3_FLAG_REPORTABLE) != 0U;
	level = flags_level(m.flags);

	/* Never answer with more than the peer said it can accept (RFC 3412
	 * §6.6), and never with less than the 484-octet floor. */
	cap = rsp_cap;
	if (m.max_size >= SNMP_V3_MSG_MAX_SIZE_MIN && m.max_size < cap) {
		cap = m.max_size;
	}

	/* ---- RFC 3414 §3.2(3): is this engine the authoritative one? ---- */
	if (m.eid_len != c->engine_id_len ||
	    (m.eid_len != 0U &&
	     memcmp(m.eid, c->engine_id, m.eid_len) != 0)) {
		c->stats.unknown_engine_ids++;
		if (!reportable) {
			return -EACCES;
		}
		rc = build_report(c, &m, USM_STAT_UNKNOWN_ENGINE_IDS, NULL,
				  false, engine_time_s, 0, rsp, cap, rsp_len);
		return (rc == 0) ? 0 : -ENOSPC;
	}

	/* ---- §3.2(4): known user? ---- */
	u = user_find(c, m.user, m.user_len);
	if (u == NULL) {
		c->stats.unknown_user_names++;
		if (!reportable) {
			return -EACCES;
		}
		rc = build_report(c, &m, USM_STAT_UNKNOWN_USER_NAMES, NULL,
				  false, engine_time_s, 0, rsp, cap, rsp_len);
		return (rc == 0) ? 0 : -ENOSPC;
	}

	/* ---- §3.2(5): does the message meet the user's level? ----
	 *
	 * Both directions are refused. A request weaker than the user's
	 * configuration would let an unauthenticated peer read the MIB under an
	 * authenticated user's name; a request stronger than it cannot be
	 * satisfied because the keys do not exist. */
	if (level != user_level(u)) {
		c->stats.unsupported_sec_levels++;
		if (!reportable) {
			return -EACCES;
		}
		rc = build_report(c, &m, USM_STAT_UNSUPPORTED_SEC_LEVELS, NULL,
				  false, engine_time_s, 0, rsp, cap, rsp_len);
		return (rc == 0) ? 0 : -ENOSPC;
	}

	/* ---- §3.2(6): verify the digest ---- */
	if (level >= (uint8_t)SNMP_SEC_AUTH_NOPRIV) {
		size_t alen = snmp_usm_authparam_len(u->auth_proto);

		if (m.authparam_len != alen) {
			c->stats.wrong_digests++;
			if (!reportable) {
				return -EACCES;
			}
			rc = build_report(c, &m, USM_STAT_WRONG_DIGESTS, NULL,
					  false, engine_time_s, 0, rsp, cap,
					  rsp_len);
			return (rc == 0) ? 0 : -ENOSPC;
		}

		/* The digest is computed over the message with the digest field
		 * zeroed, so verification needs a mutable copy. */
		memcpy(c->scratch, req, req_len);
		memset(&c->scratch[m.authparam_off], 0, alen);
		rc = snmp_usm_auth(&c->cfg.ports, u->auth_proto, u->auth_key,
				   u->auth_key_len, c->scratch, req_len, digest,
				   NULL);
		if (rc != 0) {
			return rc;
		}
		{
			uint8_t diff = 0U;
			size_t i;

			/* Constant-time: a byte-at-a-time comparison here is a
			 * forgery oracle. */
			for (i = 0U; i < alen; i++) {
				diff |= (uint8_t)(digest[i] ^
						  req[m.authparam_off + i]);
			}
			if (diff != 0U) {
				c->stats.wrong_digests++;
				if (!reportable) {
					return -EACCES;
				}
				rc = build_report(c, &m, USM_STAT_WRONG_DIGESTS,
						  NULL, false, engine_time_s, 0,
						  rsp, cap, rsp_len);
				return (rc == 0) ? 0 : -ENOSPC;
			}
		}
		c->stats.authenticated++;

		/* ---- §3.2(7): timeliness ---- */
		{
			bool stale;
			uint32_t ours = engine_time_s;
			uint32_t theirs = m.time_s;
			uint32_t delta = (ours > theirs) ? (ours - theirs)
							 : (theirs - ours);

			stale = (c->cfg.engine_boots == 0x7FFFFFFFU) ||
				(m.boots != c->cfg.engine_boots) ||
				(delta > (uint32_t)SNMP_V3_TIME_WINDOW_S);
			if (stale) {
				c->stats.not_in_time_windows++;
				if (!reportable) {
					return -EACCES;
				}
				/* Authenticated, so the manager can trust the
				 * boots/time it is about to synchronise to. */
				rc = build_report(c, &m,
						  USM_STAT_NOT_IN_TIME_WINDOWS,
						  u, true, engine_time_s, 0,
						  rsp, cap, rsp_len);
				return (rc == 0) ? 0 : -ENOSPC;
			}
		}
	}

	/* ---- §3.2(8): decrypt ---- */
	if (level == (uint8_t)SNMP_SEC_AUTH_PRIV) {
		uint8_t iv[SNMP_V3_AES_BLOCK];

		if (!m.encrypted || m.privparam_len != SNMP_V3_SALT_LEN ||
		    m.scoped_len == 0U) {
			c->stats.decryption_errors++;
			if (!reportable) {
				return -EACCES;
			}
			rc = build_report(c, &m, USM_STAT_DECRYPTION_ERRORS,
					  NULL, false, engine_time_s, 0, rsp,
					  cap, rsp_len);
			return (rc == 0) ? 0 : -ENOSPC;
		}
		/* The IV uses the *message's* boots and time, not ours: that is
		 * what the sender used to build it (RFC 3826 §3.1.2.1). */
		(void)snmp_usm_priv_iv(m.boots, m.time_s, m.privparam, iv);

		/* c->scratch already holds the message (the digest check put it
		 * there and zeroed only the digest field, which is not inside
		 * the ciphertext). */
		memcpy(c->scratch, req, req_len);
		rc = snmp_usm_aes_cfb(&c->cfg.ports, u->priv_key, iv,
				      &c->scratch[m.scoped_off],
				      &c->scratch[m.scoped_off], m.scoped_len,
				      false);
		if (rc != 0) {
			return rc;
		}
		c->stats.decrypted++;
		scoped = &c->scratch[m.scoped_off];

		/* A decrypted scopedPDU must be a SEQUENCE; anything else means
		 * the wrong key, which RFC 3414 counts as a decryption error. */
		{
			snmp_rd_t r;
			uint8_t tag;
			size_t clen;

			if (snmp_rd_init(&r, scoped, m.scoped_len) != 0 ||
			    snmp_rd_hdr(&r, &tag, &clen) != 0 ||
			    tag != SNMP_TAG_SEQUENCE) {
				c->stats.decryption_errors++;
				if (!reportable) {
					return -EACCES;
				}
				rc = build_report(c, &m,
						  USM_STAT_DECRYPTION_ERRORS,
						  NULL, false, engine_time_s, 0,
						  rsp, cap, rsp_len);
				return (rc == 0) ? 0 : -ENOSPC;
			}
			scoped = &r.buf[r.off];
			rc = parse_scoped(c, scoped, clen, &rq);
		}
	} else {
		if (m.encrypted) {
			/* Ciphertext at a level with no privacy: nothing here
			 * can read it. */
			c->stats.decryption_errors++;
			if (!reportable) {
				return -EACCES;
			}
			rc = build_report(c, &m, USM_STAT_DECRYPTION_ERRORS,
					  NULL, false, engine_time_s, 0, rsp,
					  cap, rsp_len);
			return (rc == 0) ? 0 : -ENOSPC;
		}
		rc = parse_scoped(c, &req[m.scoped_off], m.scoped_len, &rq);
	}

	switch (rc) {
	case 0:
		reqid = rq.reqid;
		break;
	case -E2BIG:
		reqid = rq.reqid;
		break;
	case -ENOENT:
		c->stats.bad_context++;
		return -EBADMSG;
	case -ENOTSUP:
		agent->stats.unsupported_pdu++;
		return -ENOTSUP;
	default:
		c->stats.malformed++;
		return -EBADMSG;
	}

	/* ---- answer it ---- */
	memset(&sec, 0, sizeof(sec));
	sec.msg_id = m.msg_id;
	/* A response mirrors the request's security level and is never
	 * reportable. */
	sec.flags = (uint8_t)(m.flags & (SNMP_V3_FLAG_AUTH | SNMP_V3_FLAG_PRIV));
	sec.user = u->name;
	sec.keys = u;
	sec.boots = c->cfg.engine_boots;
	sec.time_s = engine_time_s;

	rctx.agent = agent;
	rctx.rq = &rq;
	rctx.uptime_cs = uptime_cs;
	rctx.err = SNMP_ERR_NO_ERROR;
	rctx.err_index = 0;

	if (rc == -E2BIG) {
		agent->stats.too_big++;
		rctx.err = SNMP_ERR_TOO_BIG;
	} else if (rq.pdu == SNMP_PDU_SET) {
		agent->stats.set_refused++;
		rctx.err = SNMP_ERR_NOT_WRITABLE;
		rctx.err_index = (rq.n_vb > 0U) ? 1 : 0;
	} else if (rq.pdu == SNMP_PDU_GET) {
		agent->stats.get++;
	} else if (rq.pdu == SNMP_PDU_GETNEXT) {
		agent->stats.getnext++;
	} else {
		agent->stats.getbulk++;
	}

	rc = emit_message(c, &sec, emit_response_pdu, &rctx, rsp, cap, rsp_len);
	if (rc == -ENOSPC && rctx.err == SNMP_ERR_NO_ERROR) {
		/* RFC 3416 §4.2.1: too large to send is answered with tooBig
		 * and an empty varbind list. */
		agent->stats.too_big++;
		rctx.err = SNMP_ERR_TOO_BIG;
		rctx.err_index = 0;
		rc = emit_message(c, &sec, emit_response_pdu, &rctx, rsp, cap,
				  rsp_len);
	}
	if (rc != 0) {
		return (rc == -ENOSPC) ? -ENOSPC : rc;
	}

	(void)reqid;
	c->stats.responses++;
	agent->stats.responses++;
	return 0;
}

int snmp_dispatch(snmp_ctx_t *agent, snmp_v3_ctx_t *c, const uint8_t *req,
		  size_t req_len, uint32_t engine_time_s, uint32_t uptime_cs,
		  uint8_t *rsp, size_t rsp_cap, size_t *rsp_len)
{
	int32_t ver = 0;
	int rc;

	if (agent == NULL || req == NULL || rsp == NULL || rsp_len == NULL) {
		return -EINVAL;
	}

	rc = snmp_peek_version(req, req_len, &ver);
	if (rc != 0) {
		/* Let the v2c handler count it: it owns the rx and malformed
		 * counters and will reach the same conclusion. */
		return snmp_handle(agent, req, req_len, uptime_cs, rsp, rsp_cap,
				   rsp_len);
	}

	if (ver == SNMP_VERSION_3) {
		if (c == NULL) {
			agent->stats.rx++;
			agent->stats.bad_version++;
			return -EPROTO;
		}
		return snmp_v3_handle(agent, c, req, req_len, engine_time_s,
				      uptime_cs, rsp, rsp_cap, rsp_len);
	}

	return snmp_handle(agent, req, req_len, uptime_cs, rsp, rsp_cap,
			   rsp_len);
}

/* ========================================================================= */
/* notifications                                                             */
/* ========================================================================= */

typedef struct {
	snmp_ctx_t *agent;
	snmp_trap_t trap;
	uint32_t uptime_cs;
	const snmp_bind_t *binds;
	size_t n_binds;
} notify_vb_ctx_t;

static int fill_notify_vbs(snmp_wr_t *w, void *ctx)
{
	notify_vb_ctx_t *nv = (notify_vb_ctx_t *)ctx;
	uint32_t arcs[SNMP_OID_MAX_LEN];
	snmp_value_t v;
	size_t i;
	int n;
	int rc;

	/* Varbind 1: sysUpTime.0 (RFC 3416 §4.2.6). */
	snmp_val_uint(&v, SNMP_TAG_TIMETICKS, nv->uptime_cs);
	n = snmp_mib_oid_of((uint16_t)SNMP_OBJ_SYS_UPTIME, 0U, arcs);
	if (n < 0) {
		return n;
	}
	rc = snmp__put_varbind(w, arcs, (size_t)n, &v);
	if (rc != 0) {
		return rc;
	}

	/* Varbind 2: snmpTrapOID.0. */
	n = snmp_trap_oid(nv->trap, arcs);
	if (n < 0) {
		return n;
	}
	snmp_val_oid(&v, arcs, (size_t)n);
	rc = snmp__put_varbind(w, snmp__oid_trap_oid, 11U, &v);
	if (rc != 0) {
		return rc;
	}

	for (i = 0U; i < nv->n_binds; i++) {
		n = snmp_mib_oid_of(nv->binds[i].obj, nv->binds[i].inst, arcs);
		if (n < 0) {
			/* An unknown binding is a firmware bug, not a manager
			 * input; dropping the whole notification would hide the
			 * event it exists to report. */
			continue;
		}
		snmp__resolve(nv->agent, nv->binds[i].obj, nv->binds[i].inst,
			      nv->uptime_cs, &v);
		rc = snmp__put_varbind(w, arcs, (size_t)n, &v);
		if (rc != 0) {
			return rc;
		}
	}
	return 0;
}

typedef struct {
	notify_vb_ctx_t vb;
	uint8_t tag;
	int32_t reqid;
} notify_pdu_ctx_t;

static int emit_notify_pdu(snmp_wr_t *w, void *ctx)
{
	notify_pdu_ctx_t *np = (notify_pdu_ctx_t *)ctx;

	return snmp__emit_pdu(w, np->tag, np->reqid, 0, 0, fill_notify_vbs,
			      &np->vb);
}

int snmp_v3_make_notification(snmp_ctx_t *agent, snmp_v3_ctx_t *c,
			      const char *user, uint8_t level, snmp_trap_t t,
			      uint32_t engine_time_s, uint32_t uptime_cs,
			      const snmp_bind_t *binds, size_t n_binds,
			      bool inform, uint8_t *out, size_t cap,
			      size_t *out_len, int32_t *out_msgid)
{
	snmp_v3_user_t *u;
	v3_sec_t sec;
	notify_pdu_ctx_t np;
	int rc;

	if (agent == NULL || c == NULL || !c->ready || user == NULL ||
	    out == NULL || out_len == NULL) {
		return -EINVAL;
	}
	if (binds == NULL && n_binds != 0U) {
		return -EINVAL;
	}
	if ((unsigned int)t >= (unsigned int)SNMP_TRAP__COUNT) {
		return -EINVAL;
	}
	if (level > (uint8_t)SNMP_SEC_AUTH_PRIV) {
		return -EINVAL;
	}

	u = user_find(c, user, strlen(user));
	if (u == NULL) {
		return -ENOENT;
	}
	if (level > user_level(u)) {
		return -EPERM;
	}

	/* msgID and request-id share one counter: a receiver matches an inform
	 * acknowledgement on the request-id, a Report on the msgID, and one
	 * sequence keeps both unambiguous. Kept inside Integer32. */
	c->notify_msgid = (c->notify_msgid >= 0x7FFFFFFE) ? 1
							  : (c->notify_msgid + 1);

	memset(&sec, 0, sizeof(sec));
	sec.msg_id = c->notify_msgid;
	sec.flags = 0U;
	if (level >= (uint8_t)SNMP_SEC_AUTH_NOPRIV) {
		sec.flags |= (uint8_t)SNMP_V3_FLAG_AUTH;
	}
	if (level == (uint8_t)SNMP_SEC_AUTH_PRIV) {
		sec.flags |= (uint8_t)SNMP_V3_FLAG_PRIV;
	}
	/* An InformRequest expects a Response, so it must be reportable; a Trap
	 * expects nothing and must not be. */
	if (inform) {
		sec.flags |= (uint8_t)SNMP_V3_FLAG_REPORTABLE;
	}
	sec.user = u->name;
	sec.keys = u;
	sec.boots = c->cfg.engine_boots;
	sec.time_s = engine_time_s;

	np.tag = inform ? (uint8_t)SNMP_PDU_INFORM : (uint8_t)SNMP_PDU_TRAP_V2;
	np.reqid = c->notify_msgid;
	np.vb.agent = agent;
	np.vb.trap = t;
	np.vb.uptime_cs = uptime_cs;
	np.vb.binds = binds;
	np.vb.n_binds = n_binds;

	rc = emit_message(c, &sec, emit_notify_pdu, &np, out, cap, out_len);
	if (rc != 0) {
		return rc;
	}

	c->stats.notifications++;
	agent->stats.traps++;
	if (out_msgid != NULL) {
		*out_msgid = c->notify_msgid;
	}
	return 0;
}
