/*
 * STS1000 "Meridian" — port_crypto_t / port_sha256_stream_t over mbedTLS.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * port_crypto.h fixes the four primitives every core protocol module is
 * allowed to build on: AES-ECB, SHA-256, HMAC-SHA-256 and a CSPRNG. core/nts
 * composes AES-SIV-CMAC-256 out of the first, core/ntp signs symmetric-key
 * MACs with the second, core/mcp authenticates the console session with the
 * third, and core/nts draws cookie keys from the fourth.
 *
 * Entropy is Zephyr's `sys_csrand_get()`, which on this board is backed by the
 * STM32H5 TRNG (`&rng` in the board .dts, CONFIG_ENTROPY_GENERATOR=y). Spec
 * §9.1 eventually moves the device identity key into the ATECC608B via
 * CryptoAuthLib; that is a deferred phase (ARCHITECTURE.md §5) and changes only
 * the key handling, not this primitive set.
 *
 * Streaming SHA-256 and the caller's buffer
 * -----------------------------------------
 * port_crypto.h hands the incremental hash a plain `uint8_t state[128]`, and
 * core/mcp declares that array on the stack — so it carries no alignment
 * guarantee stronger than 1, while mbedtls_sha256_context wants natural word
 * alignment. Rather than type-pun the caller's buffer, the context is copied
 * in and out around each call. mbedtls_sha256_context is a plain-old-data
 * struct (two counters, eight state words, a 64-byte block buffer and an int),
 * so a bitwise copy is exact, and the cost is two ~112-byte memcpy per update —
 * negligible next to hashing the chunk itself.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/util.h>

#include <mbedtls/aes.h>
#include <mbedtls/md.h>
#include <mbedtls/sha256.h>

#include "storage/sts_store.h"

LOG_MODULE_REGISTER(sts_crypto, CONFIG_STS1000_LOG_LEVEL);

#define AES_BLOCK 16U
#define SHA256_LEN 32U

BUILD_ASSERT(sizeof(mbedtls_sha256_context) <= PORT_SHA256_CTX_SIZE,
	     "mbedtls_sha256_context no longer fits PORT_SHA256_CTX_SIZE; "
	     "raise it in port_crypto.h and rebuild the host tests");

/* ------------------------------------------------------------------------- */
/* One-shot primitives                                                       */
/* ------------------------------------------------------------------------- */

static int z_aes_ecb_encrypt(void *ctx, const uint8_t *key, size_t key_len,
			     const uint8_t *in, uint8_t *out, size_t len)
{
	mbedtls_aes_context aes;
	size_t off;
	int rc;

	ARG_UNUSED(ctx);

	if ((key == NULL) || (in == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if ((key_len != 16U) && (key_len != 32U)) {
		return -EINVAL;
	}
	if ((len % AES_BLOCK) != 0U) {
		return -EINVAL;
	}

	mbedtls_aes_init(&aes);
	rc = mbedtls_aes_setkey_enc(&aes, key, (unsigned int)(key_len * 8U));
	if (rc != 0) {
		mbedtls_aes_free(&aes);
		return -EIO;
	}

	for (off = 0U; off < len; off += AES_BLOCK) {
		/* in and out may alias exactly; mbedtls_aes_crypt_ecb reads the
		 * whole block before writing, so that is safe. */
		rc = mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, &in[off],
					   &out[off]);
		if (rc != 0) {
			break;
		}
	}

	mbedtls_aes_free(&aes);
	return (rc == 0) ? 0 : -EIO;
}

static int z_sha256(void *ctx, const uint8_t *in, size_t len, uint8_t *out)
{
	ARG_UNUSED(ctx);

	if ((out == NULL) || ((in == NULL) && (len != 0U))) {
		return -EINVAL;
	}
	return (mbedtls_sha256(in, len, out, 0) == 0) ? 0 : -EIO;
}

static int z_hmac_sha256(void *ctx, const uint8_t *key, size_t key_len,
			 const uint8_t *in, size_t len, uint8_t *out)
{
	const mbedtls_md_info_t *info;

	ARG_UNUSED(ctx);

	if ((key == NULL) || (out == NULL) || ((in == NULL) && (len != 0U))) {
		return -EINVAL;
	}

	info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
	if (info == NULL) {
		return -ENOTSUP;
	}
	return (mbedtls_md_hmac(info, key, key_len, in, len, out) == 0) ? 0
									: -EIO;
}

static int z_rand(void *ctx, uint8_t *out, size_t len)
{
	ARG_UNUSED(ctx);

	if ((out == NULL) && (len != 0U)) {
		return -EINVAL;
	}
	if (len == 0U) {
		return 0;
	}
	return (sys_csrand_get(out, len) == 0) ? 0 : -EIO;
}

static const port_crypto_t crypto_ops = {
	.aes_ecb_encrypt = z_aes_ecb_encrypt,
	.sha256 = z_sha256,
	.hmac_sha256 = z_hmac_sha256,
	.rand = z_rand,
	.ctx = NULL,
};

const port_crypto_t *sts_port_crypto(void)
{
	return &crypto_ops;
}

/* ------------------------------------------------------------------------- */
/* Incremental SHA-256                                                       */
/* ------------------------------------------------------------------------- */

static int z_sha_init(void *ctx, void *state)
{
	mbedtls_sha256_context c;
	int rc;

	ARG_UNUSED(ctx);

	if (state == NULL) {
		return -EINVAL;
	}

	mbedtls_sha256_init(&c);
	rc = mbedtls_sha256_starts(&c, 0);
	if (rc != 0) {
		mbedtls_sha256_free(&c);
		return -EIO;
	}
	memcpy(state, &c, sizeof(c));
	return 0;
}

static int z_sha_update(void *ctx, void *state, const uint8_t *in, size_t len)
{
	mbedtls_sha256_context c;
	int rc;

	ARG_UNUSED(ctx);

	if ((state == NULL) || ((in == NULL) && (len != 0U))) {
		return -EINVAL;
	}
	if (len == 0U) {
		return 0;
	}

	memcpy(&c, state, sizeof(c));
	rc = mbedtls_sha256_update(&c, in, len);
	memcpy(state, &c, sizeof(c));
	return (rc == 0) ? 0 : -EIO;
}

static int z_sha_final(void *ctx, void *state, uint8_t out[32])
{
	mbedtls_sha256_context c;
	int rc;

	ARG_UNUSED(ctx);

	if ((state == NULL) || (out == NULL)) {
		return -EINVAL;
	}

	memcpy(&c, state, sizeof(c));
	rc = mbedtls_sha256_finish(&c, out);
	mbedtls_sha256_free(&c);
	/* Leave no copy of the block buffer in the caller's scratch. */
	memset(state, 0, sizeof(c));
	return (rc == 0) ? 0 : -EIO;
}

static const port_sha256_stream_t sha_ops = {
	.init = z_sha_init,
	.update = z_sha_update,
	.final = z_sha_final,
	.ctx = NULL,
};

const port_sha256_stream_t *sts_port_sha256_stream(void)
{
	return &sha_ops;
}
