/*
 * Crypto primitives crossing the core boundary.
 * Target: mbedTLS/PSA (+ ATECC608B in a later phase). Host tests: vendored
 * small implementations under tests/host/support/.
 * Core builds protocol logic (AES-SIV, NTS cookies, NTP MACs) on top of
 * exactly these primitives — nothing else.
 */
#ifndef STS1000_PORT_CRYPTO_H_
#define STS1000_PORT_CRYPTO_H_

#include <stdint.h>
#include <stddef.h>

typedef struct {
	/* AES-ECB single-block / multi-block encrypt (no padding).
	 * key_len ∈ {16, 32}. in/out may alias. len % 16 == 0. */
	int (*aes_ecb_encrypt)(void *ctx, const uint8_t *key, size_t key_len,
			       const uint8_t *in, uint8_t *out, size_t len);
	/* SHA-256 one-shot. out[32]. */
	int (*sha256)(void *ctx, const uint8_t *in, size_t len, uint8_t *out);
	/* HMAC-SHA-256 one-shot. out[32]. */
	int (*hmac_sha256)(void *ctx, const uint8_t *key, size_t key_len,
			   const uint8_t *in, size_t len, uint8_t *out);
	/* Cryptographically strong random bytes. */
	int (*rand)(void *ctx, uint8_t *out, size_t len);
	void *ctx;
} port_crypto_t;

/* Incremental SHA-256 (used by the DFU image verify; buffer supplied by
 * caller must hold PORT_SHA256_CTX_SIZE bytes, opaque to core). */
#define PORT_SHA256_CTX_SIZE 128

typedef struct {
	int (*init)(void *ctx, void *state);
	int (*update)(void *ctx, void *state, const uint8_t *in, size_t len);
	int (*final)(void *ctx, void *state, uint8_t out[32]);
	void *ctx;
} port_sha256_stream_t;

#endif /* STS1000_PORT_CRYPTO_H_ */
