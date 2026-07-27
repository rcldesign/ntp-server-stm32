/*
 * STS1000 "Meridian" — core/snmp: USM cryptography (RFC 3414, 3826, 7860).
 *
 * See snmp_v3.h for why SHA-1 is implemented here and how CFB is built from an
 * ECB-only port.
 *
 * Layout of this file:
 *   1. SHA-1 and HMAC-SHA-1, private to USM
 *   2. a hash abstraction over {SHA-1 here, SHA-256 from the port}
 *   3. RFC 3414 §2.6 password expansion and key localisation
 *   4. RFC 3414 §6.3.1 / RFC 7860 §4 authentication
 *   5. RFC 3826 privacy: the IV and AES-128-CFB
 */

#include "snmp/snmp_v3.h"

#include <errno.h>
#include <string.h>

/* ========================================================================= */
/* 1. SHA-1 (FIPS 180-4) and HMAC-SHA-1 (RFC 2104)                           */
/* ========================================================================= */

#define SHA1_LEN 20U
#define SHA1_BLOCK 64U
#define SHA256_LEN 32U
#define SHA256_BLOCK 64U

typedef struct {
	uint32_t h[5];
	uint64_t bits;
	uint8_t buf[SHA1_BLOCK];
	size_t n;
} sha1_t;

static uint32_t rotl(uint32_t v, unsigned int k)
{
	return (uint32_t)((v << k) | (v >> (32U - k)));
}

static void sha1_block(uint32_t h[5], const uint8_t p[SHA1_BLOCK])
{
	uint32_t w[80];
	uint32_t a = h[0];
	uint32_t b = h[1];
	uint32_t c = h[2];
	uint32_t d = h[3];
	uint32_t e = h[4];
	unsigned int i;

	for (i = 0U; i < 16U; i++) {
		w[i] = ((uint32_t)p[i * 4U] << 24) |
		       ((uint32_t)p[(i * 4U) + 1U] << 16) |
		       ((uint32_t)p[(i * 4U) + 2U] << 8) |
		       (uint32_t)p[(i * 4U) + 3U];
	}
	for (i = 16U; i < 80U; i++) {
		w[i] = rotl(w[i - 3U] ^ w[i - 8U] ^ w[i - 14U] ^ w[i - 16U], 1U);
	}

	for (i = 0U; i < 80U; i++) {
		uint32_t f;
		uint32_t k;
		uint32_t t;

		if (i < 20U) {
			f = (b & c) | (~b & d);
			k = 0x5A827999U;
		} else if (i < 40U) {
			f = b ^ c ^ d;
			k = 0x6ED9EBA1U;
		} else if (i < 60U) {
			f = (b & c) | (b & d) | (c & d);
			k = 0x8F1BBCDCU;
		} else {
			f = b ^ c ^ d;
			k = 0xCA62C1D6U;
		}
		t = rotl(a, 5U) + f + e + k + w[i];
		e = d;
		d = c;
		c = rotl(b, 30U);
		b = a;
		a = t;
	}

	h[0] += a;
	h[1] += b;
	h[2] += c;
	h[3] += d;
	h[4] += e;
}

static void sha1_init(sha1_t *s)
{
	s->h[0] = 0x67452301U;
	s->h[1] = 0xEFCDAB89U;
	s->h[2] = 0x98BADCFEU;
	s->h[3] = 0x10325476U;
	s->h[4] = 0xC3D2E1F0U;
	s->bits = 0U;
	s->n = 0U;
}

static void sha1_update(sha1_t *s, const uint8_t *p, size_t len)
{
	size_t off = 0U;

	s->bits += (uint64_t)len * 8U;

	if (s->n != 0U) {
		size_t take = SHA1_BLOCK - s->n;

		if (take > len) {
			take = len;
		}
		memcpy(&s->buf[s->n], p, take);
		s->n += take;
		off = take;
		if (s->n == SHA1_BLOCK) {
			sha1_block(s->h, s->buf);
			s->n = 0U;
		}
	}
	while ((len - off) >= SHA1_BLOCK) {
		sha1_block(s->h, &p[off]);
		off += SHA1_BLOCK;
	}
	if (off < len) {
		memcpy(s->buf, &p[off], len - off);
		s->n = len - off;
	}
}

static void sha1_final(sha1_t *s, uint8_t out[SHA1_LEN])
{
	uint8_t tail[SHA1_BLOCK + 8U];
	uint64_t bits = s->bits;
	size_t pad;
	size_t i;

	pad = (s->n < 56U) ? (56U - s->n) : (120U - s->n);
	memset(tail, 0, sizeof(tail));
	tail[0] = 0x80U;
	for (i = 0U; i < 8U; i++) {
		tail[pad + i] = (uint8_t)((bits >> (8U * (7U - i))) & 0xFFU);
	}
	sha1_update(s, tail, pad + 8U);

	for (i = 0U; i < 5U; i++) {
		out[(i * 4U) + 0U] = (uint8_t)((s->h[i] >> 24) & 0xFFU);
		out[(i * 4U) + 1U] = (uint8_t)((s->h[i] >> 16) & 0xFFU);
		out[(i * 4U) + 2U] = (uint8_t)((s->h[i] >> 8) & 0xFFU);
		out[(i * 4U) + 3U] = (uint8_t)(s->h[i] & 0xFFU);
	}
}

static void hmac_sha1(const uint8_t *key, size_t key_len, const uint8_t *msg,
		      size_t msg_len, uint8_t out[SHA1_LEN])
{
	uint8_t k[SHA1_BLOCK];
	uint8_t pad[SHA1_BLOCK];
	uint8_t inner[SHA1_LEN];
	sha1_t s;
	size_t i;

	memset(k, 0, sizeof(k));
	if (key_len > sizeof(k)) {
		sha1_init(&s);
		sha1_update(&s, key, key_len);
		sha1_final(&s, k);
	} else if (key_len != 0U && key != NULL) {
		memcpy(k, key, key_len);
	}

	for (i = 0U; i < sizeof(pad); i++) {
		pad[i] = (uint8_t)(k[i] ^ 0x36U);
	}
	sha1_init(&s);
	sha1_update(&s, pad, sizeof(pad));
	sha1_update(&s, msg, msg_len);
	sha1_final(&s, inner);

	for (i = 0U; i < sizeof(pad); i++) {
		pad[i] = (uint8_t)(k[i] ^ 0x5CU);
	}
	sha1_init(&s);
	sha1_update(&s, pad, sizeof(pad));
	sha1_update(&s, inner, sizeof(inner));
	sha1_final(&s, out);
}

/* ========================================================================= */
/* 2. protocol tables                                                        */
/* ========================================================================= */

size_t snmp_usm_authparam_len(uint8_t proto)
{
	switch (proto) {
	case SNMP_AUTH_HMAC_SHA1_96:
		return 12U; /* RFC 3414 §6.3.1 */
	case SNMP_AUTH_HMAC_SHA256_192:
		return 24U; /* RFC 7860 §4.2.2 */
	default:
		return 0U;
	}
}

size_t snmp_usm_key_len(uint8_t proto)
{
	switch (proto) {
	case SNMP_AUTH_HMAC_SHA1_96:
		return SHA1_LEN;
	case SNMP_AUTH_HMAC_SHA256_192:
		return SHA256_LEN;
	default:
		return 0U;
	}
}

const char *snmp_auth_proto_name(uint8_t proto)
{
	switch (proto) {
	case SNMP_AUTH_HMAC_SHA1_96:
		return "HMAC-SHA-1-96";
	case SNMP_AUTH_HMAC_SHA256_192:
		return "HMAC-SHA-256-192";
	default:
		return "none";
	}
}

const char *snmp_priv_proto_name(uint8_t proto)
{
	return (proto == SNMP_PRIV_AES128_CFB) ? "AES-128-CFB" : "none";
}

const char *snmp_sec_level_name(uint8_t level)
{
	switch (level) {
	case SNMP_SEC_AUTH_NOPRIV:
		return "authNoPriv";
	case SNMP_SEC_AUTH_PRIV:
		return "authPriv";
	default:
		return "noAuthNoPriv";
	}
}

/* ------------------------------------------------------ hash abstraction */

/**
 * One-shot hash under @p proto's digest.
 *
 * SHA-1 is local; SHA-256 goes through the port so the STM32 HASH block does
 * the work on target.
 */
static int hash_once(const snmp_usm_ports_t *p, uint8_t proto,
		     const uint8_t *msg, size_t len, uint8_t *out)
{
	if (proto == SNMP_AUTH_HMAC_SHA1_96) {
		sha1_t s;

		sha1_init(&s);
		sha1_update(&s, msg, len);
		sha1_final(&s, out);
		return 0;
	}
	if (proto == SNMP_AUTH_HMAC_SHA256_192) {
		if (p->crypto == NULL || p->crypto->sha256 == NULL) {
			return -ENOTSUP;
		}
		return (p->crypto->sha256(p->crypto->ctx, msg, len, out) == 0)
			       ? 0
			       : -EIO;
	}
	return -EINVAL;
}

static int hmac_once(const snmp_usm_ports_t *p, uint8_t proto,
		     const uint8_t *key, size_t key_len, const uint8_t *msg,
		     size_t len, uint8_t *out)
{
	if (proto == SNMP_AUTH_HMAC_SHA1_96) {
		hmac_sha1(key, key_len, msg, len, out);
		return 0;
	}
	if (proto == SNMP_AUTH_HMAC_SHA256_192) {
		if (p->crypto == NULL || p->crypto->hmac_sha256 == NULL) {
			return -ENOTSUP;
		}
		return (p->crypto->hmac_sha256(p->crypto->ctx, key, key_len, msg,
					       len, out) == 0)
			       ? 0
			       : -EIO;
	}
	return -EINVAL;
}

/* ========================================================================= */
/* 3. RFC 3414 §2.6 — password to key, and localisation                      */
/* ========================================================================= */

/** Octets of repeated passphrase the expansion hashes. Not adjustable. */
#define USM_EXPAND_OCTETS 1048576U

int snmp_usm_password_to_key(const snmp_usm_ports_t *p, uint8_t proto,
			     const char *password, uint8_t *out,
			     size_t *out_len)
{
	uint8_t chunk[64];
	size_t klen = snmp_usm_key_len(proto);
	size_t plen;
	size_t idx = 0U;
	size_t done;

	if (p == NULL || password == NULL || out == NULL || klen == 0U) {
		return -EINVAL;
	}
	plen = strlen(password);
	if (plen < SNMP_V3_PASSPHRASE_MIN || plen > SNMP_V3_PASSPHRASE_MAX) {
		return -EINVAL;
	}

	/*
	 * RFC 3414 §2.6 step 1: form 1 048 576 octets by repeating the
	 * passphrase and hash the lot. It is done in 64-octet chunks because
	 * that is what the reference algorithm specifies and because a
	 * streaming hash is the only way to do it without a megabyte of RAM.
	 */
	if (proto == SNMP_AUTH_HMAC_SHA1_96) {
		sha1_t s;

		sha1_init(&s);
		for (done = 0U; done < USM_EXPAND_OCTETS; done += sizeof(chunk)) {
			size_t i;

			for (i = 0U; i < sizeof(chunk); i++) {
				chunk[i] = (uint8_t)password[idx % plen];
				idx++;
			}
			sha1_update(&s, chunk, sizeof(chunk));
		}
		sha1_final(&s, out);
	} else {
		uint8_t state[PORT_SHA256_CTX_SIZE];

		if (p->sha256_stream == NULL ||
		    p->sha256_stream->init == NULL ||
		    p->sha256_stream->update == NULL ||
		    p->sha256_stream->final == NULL) {
			return -ENOTSUP;
		}
		if (p->sha256_stream->init(p->sha256_stream->ctx, state) != 0) {
			return -EIO;
		}
		for (done = 0U; done < USM_EXPAND_OCTETS; done += sizeof(chunk)) {
			size_t i;

			for (i = 0U; i < sizeof(chunk); i++) {
				chunk[i] = (uint8_t)password[idx % plen];
				idx++;
			}
			if (p->sha256_stream->update(p->sha256_stream->ctx,
						     state, chunk,
						     sizeof(chunk)) != 0) {
				return -EIO;
			}
		}
		if (p->sha256_stream->final(p->sha256_stream->ctx, state, out) !=
		    0) {
			return -EIO;
		}
	}

	if (out_len != NULL) {
		*out_len = klen;
	}
	return 0;
}

int snmp_usm_localize(const snmp_usm_ports_t *p, uint8_t proto,
		      const uint8_t *ku, size_t ku_len,
		      const uint8_t *engine_id, size_t engine_id_len,
		      uint8_t *out, size_t *out_len)
{
	uint8_t msg[(2U * SNMP_V3_KEY_MAX) + SNMP_V3_ENGINEID_MAX];
	size_t klen = snmp_usm_key_len(proto);
	size_t n = 0U;
	int rc;

	if (p == NULL || ku == NULL || engine_id == NULL || out == NULL) {
		return -EINVAL;
	}
	if (klen == 0U || ku_len != klen) {
		return -EINVAL;
	}
	if (engine_id_len < SNMP_V3_ENGINEID_MIN ||
	    engine_id_len > SNMP_V3_ENGINEID_MAX) {
		return -EINVAL;
	}

	/* RFC 3414 §2.6 step 2: Kul = H(Ku ‖ snmpEngineID ‖ Ku). */
	memcpy(&msg[n], ku, ku_len);
	n += ku_len;
	memcpy(&msg[n], engine_id, engine_id_len);
	n += engine_id_len;
	memcpy(&msg[n], ku, ku_len);
	n += ku_len;

	rc = hash_once(p, proto, msg, n, out);
	if (rc != 0) {
		return rc;
	}
	if (out_len != NULL) {
		*out_len = klen;
	}
	return 0;
}

int snmp_usm_password_to_localized(const snmp_usm_ports_t *p, uint8_t proto,
				   const char *password,
				   const uint8_t *engine_id,
				   size_t engine_id_len, uint8_t *out,
				   size_t *out_len)
{
	uint8_t ku[SNMP_V3_KEY_MAX];
	size_t ku_len = 0U;
	int rc;

	rc = snmp_usm_password_to_key(p, proto, password, ku, &ku_len);
	if (rc != 0) {
		return rc;
	}
	return snmp_usm_localize(p, proto, ku, ku_len, engine_id, engine_id_len,
				 out, out_len);
}

/* ========================================================================= */
/* 4. authentication                                                         */
/* ========================================================================= */

int snmp_usm_auth(const snmp_usm_ports_t *p, uint8_t proto, const uint8_t *key,
		  size_t key_len, const uint8_t *msg, size_t msg_len,
		  uint8_t *out, size_t *out_len)
{
	uint8_t mac[SNMP_V3_KEY_MAX];
	size_t alen = snmp_usm_authparam_len(proto);
	int rc;

	if (p == NULL || key == NULL || msg == NULL || out == NULL) {
		return -EINVAL;
	}
	if (alen == 0U || key_len != snmp_usm_key_len(proto)) {
		return -EINVAL;
	}

	rc = hmac_once(p, proto, key, key_len, msg, msg_len, mac);
	if (rc != 0) {
		return rc;
	}
	/* RFC 3414 §6.3.1 / RFC 7860 §4.2.2: the first `alen` octets. */
	memcpy(out, mac, alen);
	if (out_len != NULL) {
		*out_len = alen;
	}
	return 0;
}

/* ========================================================================= */
/* 5. privacy                                                                */
/* ========================================================================= */

int snmp_usm_priv_iv(uint32_t boots, uint32_t time_s,
		     const uint8_t salt[SNMP_V3_SALT_LEN],
		     uint8_t iv[SNMP_V3_AES_BLOCK])
{
	if (salt == NULL || iv == NULL) {
		return -EINVAL;
	}
	iv[0] = (uint8_t)((boots >> 24) & 0xFFU);
	iv[1] = (uint8_t)((boots >> 16) & 0xFFU);
	iv[2] = (uint8_t)((boots >> 8) & 0xFFU);
	iv[3] = (uint8_t)(boots & 0xFFU);
	iv[4] = (uint8_t)((time_s >> 24) & 0xFFU);
	iv[5] = (uint8_t)((time_s >> 16) & 0xFFU);
	iv[6] = (uint8_t)((time_s >> 8) & 0xFFU);
	iv[7] = (uint8_t)(time_s & 0xFFU);
	memcpy(&iv[8], salt, SNMP_V3_SALT_LEN);
	return 0;
}

int snmp_usm_aes_cfb(const snmp_usm_ports_t *p,
		     const uint8_t key[SNMP_V3_PRIV_KEY_LEN],
		     const uint8_t iv[SNMP_V3_AES_BLOCK], const uint8_t *in,
		     uint8_t *out, size_t len, bool encrypt)
{
	uint8_t feedback[SNMP_V3_AES_BLOCK];
	uint8_t stream[SNMP_V3_AES_BLOCK];
	uint8_t carry[SNMP_V3_AES_BLOCK];
	size_t off = 0U;

	if (p == NULL || p->crypto == NULL || key == NULL || iv == NULL ||
	    out == NULL) {
		return -EINVAL;
	}
	if (in == NULL && len != 0U) {
		return -EINVAL;
	}
	if (p->crypto->aes_ecb_encrypt == NULL) {
		return -ENOTSUP;
	}

	memcpy(feedback, iv, sizeof(feedback));

	while (off < len) {
		size_t n = len - off;
		size_t i;

		if (n > SNMP_V3_AES_BLOCK) {
			n = SNMP_V3_AES_BLOCK;
		}

		/* NIST SP 800-38A §6.3: the keystream block is the forward
		 * cipher applied to the feedback, in both directions. */
		if (p->crypto->aes_ecb_encrypt(p->crypto->ctx, key,
					       SNMP_V3_PRIV_KEY_LEN, feedback,
					       stream,
					       SNMP_V3_AES_BLOCK) != 0) {
			return -EIO;
		}

		/* When decrypting, the next feedback is this block's
		 * *ciphertext* — which `in` holds now and `out` may overwrite
		 * if the buffers alias, so stash it first. */
		if (!encrypt) {
			memcpy(carry, &in[off], n);
		}

		for (i = 0U; i < n; i++) {
			out[off + i] = (uint8_t)(in[off + i] ^ stream[i]);
		}

		if (n == SNMP_V3_AES_BLOCK) {
			if (encrypt) {
				memcpy(feedback, &out[off], SNMP_V3_AES_BLOCK);
			} else {
				memcpy(feedback, carry, SNMP_V3_AES_BLOCK);
			}
		}
		off += n;
	}
	return 0;
}
