/*
 * STS1000 "Meridian" — host test support: SHA-256 and the crypto port fixtures.
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * FIPS 180-4 §6.2 reference implementation, dedicated to the public domain
 * under CC0 1.0. See host_sha256.h.
 */

#include "host_sha256.h"

#include <string.h>

/* FIPS 180-4 §4.2.2: the first 32 bits of the fractional parts of the cube
 * roots of the first 64 primes. */
static const uint32_t K[64] = {
	0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
	0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
	0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
	0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
	0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
	0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
	0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
	0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
	0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
	0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
	0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
	0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
	0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
	0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
	0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
	0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U,
};

static uint32_t ror32(uint32_t v, unsigned int n)
{
	return (v >> n) | (v << (32U - n));
}

static void sha256_block(host_sha256_t *s, const uint8_t *p)
{
	uint32_t w[64];
	uint32_t a, b, c, d, e, f, g, h;
	unsigned int i;

	for (i = 0U; i < 16U; i++) {
		w[i] = ((uint32_t)p[i * 4U] << 24) |
		       ((uint32_t)p[(i * 4U) + 1U] << 16) |
		       ((uint32_t)p[(i * 4U) + 2U] << 8) |
		       (uint32_t)p[(i * 4U) + 3U];
	}
	for (i = 16U; i < 64U; i++) {
		uint32_t s0 = ror32(w[i - 15U], 7) ^ ror32(w[i - 15U], 18) ^
			      (w[i - 15U] >> 3);
		uint32_t s1 = ror32(w[i - 2U], 17) ^ ror32(w[i - 2U], 19) ^
			      (w[i - 2U] >> 10);

		w[i] = w[i - 16U] + s0 + w[i - 7U] + s1;
	}

	a = s->h[0];
	b = s->h[1];
	c = s->h[2];
	d = s->h[3];
	e = s->h[4];
	f = s->h[5];
	g = s->h[6];
	h = s->h[7];

	for (i = 0U; i < 64U; i++) {
		uint32_t S1 = ror32(e, 6) ^ ror32(e, 11) ^ ror32(e, 25);
		uint32_t ch = (e & f) ^ ((~e) & g);
		uint32_t t1 = h + S1 + ch + K[i] + w[i];
		uint32_t S0 = ror32(a, 2) ^ ror32(a, 13) ^ ror32(a, 22);
		uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
		uint32_t t2 = S0 + maj;

		h = g;
		g = f;
		f = e;
		e = d + t1;
		d = c;
		c = b;
		b = a;
		a = t1 + t2;
	}

	s->h[0] += a;
	s->h[1] += b;
	s->h[2] += c;
	s->h[3] += d;
	s->h[4] += e;
	s->h[5] += f;
	s->h[6] += g;
	s->h[7] += h;
}

void host_sha256_init(host_sha256_t *s)
{
	/* FIPS 180-4 §5.3.3 */
	s->h[0] = 0x6a09e667U;
	s->h[1] = 0xbb67ae85U;
	s->h[2] = 0x3c6ef372U;
	s->h[3] = 0xa54ff53aU;
	s->h[4] = 0x510e527fU;
	s->h[5] = 0x9b05688cU;
	s->h[6] = 0x1f83d9abU;
	s->h[7] = 0x5be0cd19U;
	s->bits = 0U;
	s->n = 0U;
}

void host_sha256_update(host_sha256_t *s, const void *data, size_t len)
{
	const uint8_t *p = (const uint8_t *)data;

	s->bits += (uint64_t)len * 8U;

	while (len > 0U) {
		size_t take = sizeof(s->buf) - s->n;

		if (take > len) {
			take = len;
		}
		memcpy(&s->buf[s->n], p, take);
		s->n += take;
		p += take;
		len -= take;

		if (s->n == sizeof(s->buf)) {
			sha256_block(s, s->buf);
			s->n = 0U;
		}
	}
}

void host_sha256_final(host_sha256_t *s, uint8_t out[32])
{
	uint64_t bits = s->bits;
	uint8_t pad = 0x80U;
	uint8_t len_be[8];
	unsigned int i;

	host_sha256_update(s, &pad, 1U);
	s->bits = bits; /* padding is not message data */

	while (s->n != 56U) {
		uint8_t z = 0U;

		host_sha256_update(s, &z, 1U);
		s->bits = bits;
	}

	for (i = 0U; i < 8U; i++) {
		len_be[i] = (uint8_t)((bits >> (56U - (8U * i))) & 0xFFU);
	}
	host_sha256_update(s, len_be, sizeof(len_be));

	for (i = 0U; i < 8U; i++) {
		out[i * 4U] = (uint8_t)(s->h[i] >> 24);
		out[(i * 4U) + 1U] = (uint8_t)(s->h[i] >> 16);
		out[(i * 4U) + 2U] = (uint8_t)(s->h[i] >> 8);
		out[(i * 4U) + 3U] = (uint8_t)(s->h[i]);
	}
}

void host_sha256(const void *data, size_t len, uint8_t out[32])
{
	host_sha256_t s;

	host_sha256_init(&s);
	host_sha256_update(&s, data, len);
	host_sha256_final(&s, out);
}

void host_hmac_sha256(const uint8_t *key, size_t key_len, const uint8_t *msg,
		      size_t msg_len, uint8_t out[32])
{
	uint8_t k[64];
	uint8_t pad[64];
	uint8_t inner[32];
	host_sha256_t s;
	size_t i;

	memset(k, 0, sizeof(k));
	if (key_len > sizeof(k)) {
		host_sha256(key, key_len, k);
	} else if (key_len != 0U) {
		memcpy(k, key, key_len);
	}

	for (i = 0U; i < sizeof(k); i++) {
		pad[i] = (uint8_t)(k[i] ^ 0x36U);
	}
	host_sha256_init(&s);
	host_sha256_update(&s, pad, sizeof(pad));
	host_sha256_update(&s, msg, msg_len);
	host_sha256_final(&s, inner);

	for (i = 0U; i < sizeof(k); i++) {
		pad[i] = (uint8_t)(k[i] ^ 0x5CU);
	}
	host_sha256_init(&s);
	host_sha256_update(&s, pad, sizeof(pad));
	host_sha256_update(&s, inner, sizeof(inner));
	host_sha256_final(&s, out);
}

/* ------------------------------------------------------------------------- */
/* Port fixtures                                                             */
/* ------------------------------------------------------------------------- */

_Static_assert(sizeof(host_sha256_t) <= PORT_SHA256_CTX_SIZE,
	       "host_sha256_t must fit in the port's opaque state buffer");

static int fx_stream_init(void *ctx, void *state)
{
	(void)ctx;
	host_sha256_init((host_sha256_t *)state);
	return 0;
}

static int fx_stream_update(void *ctx, void *state, const uint8_t *in,
			    size_t len)
{
	(void)ctx;
	host_sha256_update((host_sha256_t *)state, in, len);
	return 0;
}

static int fx_stream_final(void *ctx, void *state, uint8_t out[32])
{
	(void)ctx;
	host_sha256_final((host_sha256_t *)state, out);
	return 0;
}

static const port_sha256_stream_t stream_fixture = {
	.init = fx_stream_init,
	.update = fx_stream_update,
	.final = fx_stream_final,
	.ctx = NULL,
};

const port_sha256_stream_t *host_sha256_stream(void)
{
	return &stream_fixture;
}

static int fx_sha256(void *ctx, const uint8_t *in, size_t len, uint8_t *out)
{
	(void)ctx;
	host_sha256(in, len, out);
	return 0;
}

static int fx_hmac_sha256(void *ctx, const uint8_t *key, size_t key_len,
			  const uint8_t *in, size_t len, uint8_t *out)
{
	(void)ctx;
	host_hmac_sha256(key, key_len, in, len, out);
	return 0;
}

/* Deterministic xorshift32 — reproducible, and obviously not entropy. */
static uint32_t rng_state = 0x1234ABCDU;

void host_crypto_seed(uint32_t seed)
{
	rng_state = (seed != 0U) ? seed : 0x1234ABCDU;
}

static int fx_rand(void *ctx, uint8_t *out, size_t len)
{
	size_t i;

	(void)ctx;
	for (i = 0U; i < len; i++) {
		rng_state ^= rng_state << 13;
		rng_state ^= rng_state >> 17;
		rng_state ^= rng_state << 5;
		out[i] = (uint8_t)(rng_state & 0xFFU);
	}
	return 0;
}

static const port_crypto_t crypto_fixture = {
	.aes_ecb_encrypt = NULL, /* nothing under test needs AES yet */
	.sha256 = fx_sha256,
	.hmac_sha256 = fx_hmac_sha256,
	.rand = fx_rand,
	.ctx = NULL,
};

const port_crypto_t *host_crypto(void)
{
	return &crypto_fixture;
}
