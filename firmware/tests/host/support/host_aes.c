/*
 * STS1000 "Meridian" — host-side AES and crypto-port fixture. See host_aes.h.
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * AES per FIPS-197: the state is held as the 16 input octets in wire order,
 * which is FIPS-197's column-major s[r][c] = in[r + 4c]. Rows are therefore the
 * strided sets {0,4,8,12}, {1,5,9,13}, ... and columns are the contiguous
 * groups of four, which is exactly what ShiftRows and MixColumns want. Clarity
 * over speed: no tables beyond the S-box and the round constants.
 *
 * SHA-256 and HMAC-SHA-256 are not here — support/host_sha256.c owns them, and
 * everything in support/ shares one link namespace.
 */

#include "host_aes.h"

#include <string.h>

#include "host_sha256.h"

/* --------------------------------------------------------------------- AES */

static const uint8_t sbox[256] = {
	0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b,
	0xfe, 0xd7, 0xab, 0x76, 0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0,
	0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26,
	0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
	0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2,
	0xeb, 0x27, 0xb2, 0x75, 0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0,
	0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed,
	0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
	0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f,
	0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5,
	0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, 0xcd, 0x0c, 0x13, 0xec,
	0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
	0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14,
	0xde, 0x5e, 0x0b, 0xdb, 0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c,
	0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d,
	0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
	0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f,
	0x4b, 0xbd, 0x8b, 0x8a, 0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e,
	0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 0xe1, 0xf8, 0x98, 0x11,
	0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
	0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f,
	0xb0, 0x54, 0xbb, 0x16,
};

/* Rcon[i] = x^(i-1) in GF(2^8); ten entries cover AES-128 (the deepest need). */
static const uint8_t rcon[11] = { 0x00, 0x01, 0x02, 0x04, 0x08, 0x10,
				  0x20, 0x40, 0x80, 0x1b, 0x36 };

#define AES_MAX_ROUNDS 14
#define AES_MAX_RK ((AES_MAX_ROUNDS + 1) * 16)

static uint8_t xtime(uint8_t b)
{
	return (uint8_t)((uint8_t)(b << 1) ^ (uint8_t)(0x1BU * (b >> 7)));
}

static int key_expansion(const uint8_t *key, size_t key_len, uint8_t *rk)
{
	size_t nk = key_len / 4U;
	size_t nr = nk + 6U;
	size_t words = 4U * (nr + 1U);

	if (key_len != 16U && key_len != 32U) {
		return -1;
	}
	memcpy(rk, key, key_len);

	for (size_t i = nk; i < words; i++) {
		uint8_t t[4];

		memcpy(t, &rk[(i - 1U) * 4U], 4);
		if (i % nk == 0U) {
			uint8_t tmp = t[0];

			t[0] = (uint8_t)(sbox[t[1]] ^ rcon[i / nk]);
			t[1] = sbox[t[2]];
			t[2] = sbox[t[3]];
			t[3] = sbox[tmp];
		} else if (nk > 6U && i % nk == 4U) {
			t[0] = sbox[t[0]];
			t[1] = sbox[t[1]];
			t[2] = sbox[t[2]];
			t[3] = sbox[t[3]];
		}
		for (size_t j = 0U; j < 4U; j++) {
			rk[i * 4U + j] = (uint8_t)(rk[(i - nk) * 4U + j] ^ t[j]);
		}
	}
	return (int)nr;
}

static void add_round_key(uint8_t s[16], const uint8_t *rk)
{
	for (size_t i = 0U; i < 16U; i++) {
		s[i] ^= rk[i];
	}
}

static void sub_bytes(uint8_t s[16])
{
	for (size_t i = 0U; i < 16U; i++) {
		s[i] = sbox[s[i]];
	}
}

static void shift_rows(uint8_t s[16])
{
	uint8_t t;

	/* row 1 <<< 1 */
	t = s[1];
	s[1] = s[5];
	s[5] = s[9];
	s[9] = s[13];
	s[13] = t;
	/* row 2 <<< 2 */
	t = s[2];
	s[2] = s[10];
	s[10] = t;
	t = s[6];
	s[6] = s[14];
	s[14] = t;
	/* row 3 <<< 3, i.e. >>> 1 */
	t = s[15];
	s[15] = s[11];
	s[11] = s[7];
	s[7] = s[3];
	s[3] = t;
}

static void mix_columns(uint8_t s[16])
{
	for (size_t c = 0U; c < 16U; c += 4U) {
		uint8_t a0 = s[c];
		uint8_t a1 = s[c + 1U];
		uint8_t a2 = s[c + 2U];
		uint8_t a3 = s[c + 3U];
		uint8_t all = (uint8_t)(a0 ^ a1 ^ a2 ^ a3);

		s[c] = (uint8_t)(a0 ^ all ^ xtime((uint8_t)(a0 ^ a1)));
		s[c + 1U] = (uint8_t)(a1 ^ all ^ xtime((uint8_t)(a1 ^ a2)));
		s[c + 2U] = (uint8_t)(a2 ^ all ^ xtime((uint8_t)(a2 ^ a3)));
		s[c + 3U] = (uint8_t)(a3 ^ all ^ xtime((uint8_t)(a3 ^ a0)));
	}
}

int host_aes_ecb_encrypt(const uint8_t *key, size_t key_len, const uint8_t *in,
			 uint8_t *out, size_t len)
{
	uint8_t rk[AES_MAX_RK];
	int nr;

	if (key == NULL || (len != 0U && (in == NULL || out == NULL))) {
		return -1;
	}
	if ((len % 16U) != 0U) {
		return -1;
	}
	nr = key_expansion(key, key_len, rk);
	if (nr < 0) {
		return -1;
	}

	for (size_t off = 0U; off < len; off += 16U) {
		uint8_t s[16];

		memcpy(s, &in[off], 16);
		add_round_key(s, rk);
		for (int r = 1; r < nr; r++) {
			sub_bytes(s);
			shift_rows(s);
			mix_columns(s);
			add_round_key(s, &rk[(size_t)r * 16U]);
		}
		sub_bytes(s);
		shift_rows(s);
		add_round_key(s, &rk[(size_t)nr * 16U]);
		memcpy(&out[off], s, 16);
	}
	return 0;
}

/* -------------------------------------------------------------- the fixture */

static uint32_t xorshift32(uint32_t *s)
{
	uint32_t x = *s;

	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	*s = x;
	return x;
}

/** True when an armed injection counter fires on this call. */
static int fire(unsigned *counter)
{
	if (*counter == 0U) {
		return 0;
	}
	*counter -= 1U;
	return (*counter == 0U) ? 1 : 0;
}

static int hc_aes(void *ctx, const uint8_t *key, size_t key_len,
		  const uint8_t *in, uint8_t *out, size_t len)
{
	host_crypto_t *hc = (host_crypto_t *)ctx;

	hc->n_aes++;
	if (fire(&hc->fail_aes_in)) {
		return -1;
	}
	return host_aes_ecb_encrypt(key, key_len, in, out, len);
}

static int hc_sha(void *ctx, const uint8_t *in, size_t len, uint8_t *out)
{
	host_crypto_t *hc = (host_crypto_t *)ctx;

	hc->n_sha++;
	if (fire(&hc->fail_sha_in)) {
		return -1;
	}
	host_sha256(in, len, out);
	return 0;
}

static int hc_hmac(void *ctx, const uint8_t *key, size_t key_len,
		   const uint8_t *in, size_t len, uint8_t *out)
{
	host_crypto_t *hc = (host_crypto_t *)ctx;

	hc->n_hmac++;
	if (fire(&hc->fail_hmac_in)) {
		return -1;
	}
	host_hmac_sha256(key, key_len, in, len, out);
	return 0;
}

static int hc_rand(void *ctx, uint8_t *out, size_t len)
{
	host_crypto_t *hc = (host_crypto_t *)ctx;

	hc->n_rand++;
	if (fire(&hc->fail_rand_in)) {
		return -1;
	}
	for (size_t i = 0U; i < len; i++) {
		out[i] = (uint8_t)(xorshift32(&hc->rng) >> 24);
	}
	return 0;
}

void host_crypto_init(host_crypto_t *hc, uint32_t seed)
{
	memset(hc, 0, sizeof(*hc));
	hc->rng = (seed != 0U) ? seed : 0xA5A5A5A5U;
}

port_crypto_t host_crypto_port(host_crypto_t *hc)
{
	port_crypto_t p;

	memset(&p, 0, sizeof(p));
	p.aes_ecb_encrypt = hc_aes;
	p.sha256 = hc_sha;
	p.hmac_sha256 = hc_hmac;
	p.rand = hc_rand;
	p.ctx = hc;
	return p;
}
