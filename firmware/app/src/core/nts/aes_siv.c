/*
 * STS1000 "Meridian" — core/nts: AES-SIV-CMAC-256 (RFC 5297).
 *
 * See aes_siv.h for the contract. Three layers, bottom up:
 *
 *   AES-CMAC (RFC 4493)  incremental, so S2V can feed it a long plaintext with
 *                        the last block XORed against D without copying the
 *                        message. The trick that makes CMAC streamable is
 *                        holding one block back: a full buffer is only absorbed
 *                        once more input proves it is not the last block.
 *
 *   S2V (§2.4)           the PRF that turns the associated-data vector plus the
 *                        plaintext into the synthetic IV. dbl() is the doubling
 *                        in GF(2^128) with the 0x87 reduction polynomial.
 *
 *   CTR (§2.5)           keystream from AES-ECB, counter incremented over the
 *                        whole 128-bit block, big-endian. The IV is the SIV
 *                        with bit 31 and bit 63 of each half cleared, which is
 *                        what keeps a 2^31-block message from carrying the
 *                        counter out of its lane.
 *
 * Error handling: the AES port returns int, and a failure part-way through a
 * CMAC must not silently produce a plausible tag. The incremental CMAC keeps a
 * sticky error and reports it at cmac_final(), so every path that could fail is
 * checked exactly once, at the end.
 */

#include "nts/aes_siv.h"

#include <errno.h>
#include <string.h>

/* --------------------------------------------------------------- utilities */

bool aes_siv_ct_memeq(const uint8_t *a, const uint8_t *b, size_t n)
{
	uint8_t diff = 0U;

	for (size_t i = 0U; i < n; i++) {
		diff |= (uint8_t)(a[i] ^ b[i]);
	}
	return diff == 0U;
}

static void xor_into(uint8_t *dst, const uint8_t *src, size_t n)
{
	for (size_t i = 0U; i < n; i++) {
		dst[i] ^= src[i];
	}
}

/**
 * Doubling in GF(2^128): a left shift by one, reduced by the low-weight
 * irreducible polynomial x^128 + x^7 + x^2 + x + 1 (constant 0x87) when the
 * shifted-out bit was set. Branch-free so the reduction is not a timing signal.
 */
static void dbl128(uint8_t b[AES_SIV_BLOCK])
{
	uint8_t carry = (uint8_t)(b[0] >> 7);

	for (size_t i = 0U; i < AES_SIV_BLOCK - 1U; i++) {
		b[i] = (uint8_t)((uint8_t)(b[i] << 1) | (uint8_t)(b[i + 1U] >> 7));
	}
	b[AES_SIV_BLOCK - 1U] = (uint8_t)(b[AES_SIV_BLOCK - 1U] << 1);
	b[AES_SIV_BLOCK - 1U] ^= (uint8_t)(0x87U * carry);
}

/* ------------------------------------------------------------- AES-CMAC */

typedef struct {
	const port_crypto_t *crypto;
	const uint8_t *key;
	uint8_t k1[AES_SIV_BLOCK];
	uint8_t k2[AES_SIV_BLOCK];
	uint8_t x[AES_SIV_BLOCK];   /* CBC chaining state */
	uint8_t blk[AES_SIV_BLOCK]; /* held-back block */
	size_t blk_len;
	int err;
} cmac_t;

static int aes_block(const port_crypto_t *cr, const uint8_t *key,
		     const uint8_t *in, uint8_t *out)
{
	return cr->aes_ecb_encrypt(cr->ctx, key, AES_SIV_BLOCK, in, out,
				   AES_SIV_BLOCK);
}

static int cmac_init(cmac_t *c, const port_crypto_t *crypto, const uint8_t *key)
{
	static const uint8_t zero[AES_SIV_BLOCK] = { 0 };

	c->crypto = crypto;
	c->key = key;
	c->blk_len = 0U;
	c->err = 0;
	memset(c->x, 0, sizeof(c->x));

	/* RFC 4493 §2.3 subkey generation: L = AES(K, 0), K1 = dbl(L),
	 * K2 = dbl(K1). */
	if (aes_block(crypto, key, zero, c->k1) != 0) {
		c->err = -EIO;
		return -EIO;
	}
	dbl128(c->k1);
	memcpy(c->k2, c->k1, sizeof(c->k2));
	dbl128(c->k2);
	return 0;
}

static void cmac_absorb(cmac_t *c, const uint8_t blk[AES_SIV_BLOCK])
{
	xor_into(c->x, blk, AES_SIV_BLOCK);
	if (aes_block(c->crypto, c->key, c->x, c->x) != 0) {
		c->err = -EIO;
	}
}

static void cmac_update(cmac_t *c, const uint8_t *p, size_t n)
{
	while (n > 0U) {
		size_t take;

		/* Only absorb a full buffer once more input proves it is not the
		 * last block — the last block gets K1/K2 treatment instead. */
		if (c->blk_len == AES_SIV_BLOCK) {
			cmac_absorb(c, c->blk);
			c->blk_len = 0U;
		}
		take = AES_SIV_BLOCK - c->blk_len;
		if (take > n) {
			take = n;
		}
		memcpy(&c->blk[c->blk_len], p, take);
		c->blk_len += take;
		p += take;
		n -= take;
	}
}

static int cmac_final(cmac_t *c, uint8_t out[AES_SIV_BLOCK])
{
	uint8_t last[AES_SIV_BLOCK];

	if (c->blk_len == AES_SIV_BLOCK) {
		memcpy(last, c->blk, AES_SIV_BLOCK);
		xor_into(last, c->k1, AES_SIV_BLOCK);
	} else {
		/* pad(M) = M || 0x80 || 0...0, then XOR K2. An empty message takes
		 * this path too, which is RFC 4493's definition, not a special
		 * case. */
		memset(last, 0, sizeof(last));
		memcpy(last, c->blk, c->blk_len);
		last[c->blk_len] = 0x80U;
		xor_into(last, c->k2, AES_SIV_BLOCK);
	}
	cmac_absorb(c, last);

	if (c->err != 0) {
		return c->err;
	}
	memcpy(out, c->x, AES_SIV_BLOCK);
	return 0;
}

int aes_cmac(const port_crypto_t *crypto, const uint8_t *key,
	     const uint8_t *msg, size_t msg_len, uint8_t out[AES_SIV_BLOCK])
{
	cmac_t c;
	int rc;

	if (crypto == NULL || crypto->aes_ecb_encrypt == NULL || key == NULL ||
	    out == NULL || (msg == NULL && msg_len != 0U)) {
		return -EINVAL;
	}
	rc = cmac_init(&c, crypto, key);
	if (rc != 0) {
		return rc;
	}
	cmac_update(&c, msg, msg_len);
	return cmac_final(&c, out);
}

/* ------------------------------------------------------------------- S2V */

static int s2v(const aes_siv_ctx_t *c, const aes_siv_ad_t *ad, size_t n_ad,
	       const uint8_t *pt, size_t pt_len, uint8_t v[AES_SIV_BLOCK])
{
	static const uint8_t zero[AES_SIV_BLOCK] = { 0 };
	uint8_t d[AES_SIV_BLOCK];
	uint8_t t[AES_SIV_BLOCK];
	cmac_t m;
	int rc;

	/* D = CMAC(K, <zero>) */
	rc = cmac_init(&m, &c->crypto, c->k_s2v);
	if (rc != 0) {
		return rc;
	}
	cmac_update(&m, zero, sizeof(zero));
	rc = cmac_final(&m, d);
	if (rc != 0) {
		return rc;
	}

	/* D = dbl(D) xor CMAC(K, Si) for every associated-data component. The
	 * plaintext is S(n) and is handled below, not here. */
	for (size_t i = 0U; i < n_ad; i++) {
		rc = cmac_init(&m, &c->crypto, c->k_s2v);
		if (rc != 0) {
			return rc;
		}
		cmac_update(&m, ad[i].p, ad[i].len);
		rc = cmac_final(&m, t);
		if (rc != 0) {
			return rc;
		}
		dbl128(d);
		xor_into(d, t, AES_SIV_BLOCK);
	}

	rc = cmac_init(&m, &c->crypto, c->k_s2v);
	if (rc != 0) {
		return rc;
	}

	if (pt_len >= AES_SIV_BLOCK) {
		/* T = Sn xorend D: CMAC the plaintext with D folded into its last
		 * block. Streaming it this way is why the whole message never has
		 * to be copied. */
		cmac_update(&m, pt, pt_len - AES_SIV_BLOCK);
		memcpy(t, &pt[pt_len - AES_SIV_BLOCK], AES_SIV_BLOCK);
		xor_into(t, d, AES_SIV_BLOCK);
		cmac_update(&m, t, AES_SIV_BLOCK);
	} else {
		/* T = dbl(D) xor pad(Sn) */
		dbl128(d);
		memset(t, 0, sizeof(t));
		if (pt_len > 0U) {
			memcpy(t, pt, pt_len);
		}
		t[pt_len] = 0x80U;
		xor_into(t, d, AES_SIV_BLOCK);
		cmac_update(&m, t, AES_SIV_BLOCK);
	}

	return cmac_final(&m, v);
}

/* ------------------------------------------------------------------- CTR */

static int ctr_xor(const aes_siv_ctx_t *c, const uint8_t iv[AES_SIV_BLOCK],
		   const uint8_t *in, uint8_t *out, size_t len)
{
	uint8_t ctr[AES_SIV_BLOCK];
	uint8_t ks[AES_SIV_BLOCK];

	memcpy(ctr, iv, AES_SIV_BLOCK);

	while (len > 0U) {
		size_t n = (len < AES_SIV_BLOCK) ? len : AES_SIV_BLOCK;

		if (aes_block(&c->crypto, c->k_ctr, ctr, ks) != 0) {
			return -EIO;
		}
		for (size_t i = 0U; i < n; i++) {
			out[i] = (uint8_t)(in[i] ^ ks[i]);
		}
		in += n;
		out += n;
		len -= n;

		for (size_t j = AES_SIV_BLOCK; j-- > 0U;) {
			if (++ctr[j] != 0U) {
				break;
			}
		}
	}
	return 0;
}

/** Q = V with bit 31 and bit 63 of each 64-bit half cleared (RFC 5297 §2.6). */
static void siv_to_ctr_iv(const uint8_t v[AES_SIV_BLOCK],
			  uint8_t q[AES_SIV_BLOCK])
{
	memcpy(q, v, AES_SIV_BLOCK);
	q[8] &= 0x7FU;
	q[12] &= 0x7FU;
}

/* -------------------------------------------------------------- public API */

int aes_siv_init(aes_siv_ctx_t *c, const port_crypto_t *crypto,
		 const uint8_t *key, size_t key_len)
{
	if (c == NULL || crypto == NULL || key == NULL ||
	    crypto->aes_ecb_encrypt == NULL) {
		return -EINVAL;
	}
	if (key_len != AES_SIV_KEY_LEN) {
		return -EINVAL;
	}

	memset(c, 0, sizeof(*c));
	c->crypto = *crypto;
	memcpy(c->k_s2v, key, AES_SIV_BLOCK);
	memcpy(c->k_ctr, &key[AES_SIV_BLOCK], AES_SIV_BLOCK);
	c->ready = true;
	return 0;
}

static bool ad_ok(const aes_siv_ad_t *ad, size_t n_ad)
{
	if (n_ad > AES_SIV_MAX_AD) {
		return false;
	}
	if (n_ad != 0U && ad == NULL) {
		return false;
	}
	for (size_t i = 0U; i < n_ad; i++) {
		if (ad[i].p == NULL && ad[i].len != 0U) {
			return false;
		}
	}
	return true;
}

int aes_siv_encrypt(const aes_siv_ctx_t *c, const aes_siv_ad_t *ad, size_t n_ad,
		    const uint8_t *pt, size_t pt_len, uint8_t *out,
		    size_t out_cap, size_t *out_len)
{
	uint8_t v[AES_SIV_BLOCK];
	uint8_t q[AES_SIV_BLOCK];
	int rc;

	if (c == NULL || !c->ready || out == NULL || out_len == NULL ||
	    (pt == NULL && pt_len != 0U) || !ad_ok(ad, n_ad)) {
		return -EINVAL;
	}
	if (out_cap < pt_len + AES_SIV_TAG_LEN) {
		return -ENOSPC;
	}

	/* S2V reads the plaintext, so it must run before CTR overwrites it in
	 * the aliasing case (out + 16 == pt). */
	rc = s2v(c, ad, n_ad, pt, pt_len, v);
	if (rc != 0) {
		return rc;
	}

	siv_to_ctr_iv(v, q);
	rc = ctr_xor(c, q, pt, &out[AES_SIV_TAG_LEN], pt_len);
	if (rc != 0) {
		return rc;
	}

	memcpy(out, v, AES_SIV_TAG_LEN);
	*out_len = pt_len + AES_SIV_TAG_LEN;
	return 0;
}

int aes_siv_decrypt(const aes_siv_ctx_t *c, const aes_siv_ad_t *ad, size_t n_ad,
		    const uint8_t *in, size_t in_len, uint8_t *pt,
		    size_t pt_cap, size_t *pt_len)
{
	uint8_t v[AES_SIV_BLOCK];
	uint8_t q[AES_SIV_BLOCK];
	uint8_t check[AES_SIV_BLOCK];
	size_t clen;
	int rc;

	if (c == NULL || !c->ready || in == NULL || pt_len == NULL ||
	    !ad_ok(ad, n_ad)) {
		return -EINVAL;
	}
	if (in_len < AES_SIV_TAG_LEN) {
		return -EBADMSG;
	}
	clen = in_len - AES_SIV_TAG_LEN;
	if (pt == NULL && clen != 0U) {
		return -EINVAL;
	}
	if (pt_cap < clen) {
		return -ENOSPC;
	}

	memcpy(v, in, AES_SIV_TAG_LEN);
	siv_to_ctr_iv(v, q);
	rc = ctr_xor(c, q, &in[AES_SIV_TAG_LEN], pt, clen);
	if (rc != 0) {
		return rc;
	}

	rc = s2v(c, ad, n_ad, pt, clen, check);
	if (rc != 0) {
		return rc;
	}
	if (!aes_siv_ct_memeq(check, v, AES_SIV_TAG_LEN)) {
		/* Leave the unauthenticated plaintext where it is but report
		 * nothing about it; the contract forbids the caller reading it. */
		return -EBADMSG;
	}

	*pt_len = clen;
	return 0;
}
