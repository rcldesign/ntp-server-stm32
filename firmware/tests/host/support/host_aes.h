/*
 * STS1000 "Meridian" — host-side AES and the full crypto-port fixture.
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * core/ takes every primitive through port_crypto_t (ARCHITECTURE.md §4). On
 * target that port is mbedTLS/PSA over the STM32H5 AES and HASH blocks; here it
 * is this file plus its sibling host_sha256.c. What the protocol modules are
 * being tested for is the protocol — AES-SIV, NTS cookies, NTP MACs — so the
 * primitives underneath have to be beyond suspicion. They are anchored to
 * published known-answer tests in test_aes_siv.c: FIPS-197 §C.1/C.3 and NIST
 * SP 800-38A §F.1.1 for AES-ECB, RFC 4493 for CMAC, FIPS-180-4 for SHA-256 and
 * RFC 4231 for HMAC-SHA-256.
 *
 * Provenance: the AES here was written for this repository directly from
 * FIPS-197 and is dedicated to the public domain under CC0 1.0. No third-party
 * source is vendored. It is a reference implementation for tests — readable,
 * not constant-time and not fast. Never ship it.
 *
 * Division of labour with host_sha256.h, which owns SHA-256 and HMAC-SHA-256:
 * everything under support/ links into every test binary, so a second copy of
 * those two would be a duplicate-symbol error rather than a private detail.
 * This file adds only what that one deliberately leaves NULL — AES-ECB — and
 * the fixture below, which differs from host_crypto() in two ways that matter
 * to the crypto suites: it carries AES, and it can inject port failures.
 *
 * All symbols carry the host_ prefix for the same shared-namespace reason.
 */

#ifndef STS1000_TEST_HOST_AES_H_
#define STS1000_TEST_HOST_AES_H_

#include <stddef.h>
#include <stdint.h>

#include "port/port_crypto.h"

/**
 * AES-ECB encrypt, no padding. @p key_len is 16 or 32; @p len must be a whole
 * number of 16-octet blocks. @p in and @p out may be the same pointer.
 *
 * @retval 0   Done.
 * @retval -1  Bad key length, a partial block, or a NULL buffer with a
 *             non-zero length.
 */
int host_aes_ecb_encrypt(const uint8_t *key, size_t key_len, const uint8_t *in,
			 uint8_t *out, size_t len);

/**
 * Fixture state. Deterministic: the "random" stream is a seeded xorshift32, so
 * a cookie or a nonce produced during a test is reproducible and a failure can
 * be replayed from the seed.
 *
 * The fail_*_in fields inject port failures, which is the only way to reach the
 * -EIO paths in the modules under test. Set one to N and the Nth subsequent
 * call to that primitive fails; 0 disables injection. The counter is consumed,
 * so exactly one call fails per arming.
 */
typedef struct {
	uint32_t rng;
	unsigned fail_aes_in;
	unsigned fail_sha_in;
	unsigned fail_hmac_in;
	unsigned fail_rand_in;
	unsigned n_aes;
	unsigned n_sha;
	unsigned n_hmac;
	unsigned n_rand;
} host_crypto_t;

/** Reset @p hc and seed its generator. A @p seed of 0 is remapped. */
void host_crypto_init(host_crypto_t *hc, uint32_t seed);

/**
 * Build a port_crypto_t bound to @p hc, with all four primitives present.
 * @p hc must outlive every use of the returned port.
 */
port_crypto_t host_crypto_port(host_crypto_t *hc);

#endif /* STS1000_TEST_HOST_AES_H_ */
