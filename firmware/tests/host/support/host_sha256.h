/*
 * STS1000 "Meridian" — host test support: SHA-256 and the crypto port fixtures.
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Written for this repository from the FIPS 180-4 pseudocode and dedicated to
 * the public domain under CC0 1.0. It exists so the host tests can exercise
 * core code that hashes (core/mcp's DFU verify, the admin credential check)
 * without linking mbedTLS. It is a straightforward reference implementation:
 * correct and readable, not constant-time or fast. Never ship it.
 *
 * The two port fixtures below satisfy port_crypto.h:
 *
 *   host_sha256_stream()  port_sha256_stream_t — incremental, used by FW_END
 *   host_crypto()         port_crypto_t        — sha256, hmac_sha256, rand
 *
 * host_crypto()'s aes_ecb_encrypt is NULL: nothing under test needs AES yet,
 * and a stub that silently returns garbage would be worse than an obvious
 * NULL. Its rand is a fixed-seed xorshift so a failing case replays exactly —
 * it is emphatically not an entropy source.
 */

#ifndef STS1000_TEST_HOST_SHA256_H_
#define STS1000_TEST_HOST_SHA256_H_

#include <stddef.h>
#include <stdint.h>

#include "port/port_crypto.h"

/** Incremental SHA-256 state. */
typedef struct {
	uint32_t h[8];
	uint64_t bits;   /* total message length in bits */
	uint8_t  buf[64];
	size_t   n;      /* bytes buffered in buf */
} host_sha256_t;

void host_sha256_init(host_sha256_t *s);
void host_sha256_update(host_sha256_t *s, const void *data, size_t len);
void host_sha256_final(host_sha256_t *s, uint8_t out[32]);

/** One-shot SHA-256 of @p len bytes at @p data. */
void host_sha256(const void *data, size_t len, uint8_t out[32]);

/** HMAC-SHA-256 (RFC 2104) with an arbitrary-length key. */
void host_hmac_sha256(const uint8_t *key, size_t key_len, const uint8_t *msg,
		      size_t msg_len, uint8_t out[32]);

/** port_sha256_stream_t bound to the implementation above. */
const port_sha256_stream_t *host_sha256_stream(void);

/** port_crypto_t bound to the implementation above (no AES). */
const port_crypto_t *host_crypto(void);

/** Reseed host_crypto()'s deterministic PRNG. */
void host_crypto_seed(uint32_t seed);

#endif /* STS1000_TEST_HOST_SHA256_H_ */
