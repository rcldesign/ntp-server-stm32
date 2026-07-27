/*
 * STS1000 "Meridian" — core/nts: AES-SIV-CMAC-256 (RFC 5297).
 *
 * Platform-neutral C11, no dynamic allocation. Built on exactly one primitive
 * from port_crypto_t — AES-ECB encrypt — because that is what the STM32H5 AES
 * accelerator, mbedTLS/PSA and a host test fixture all agree on. AES-CMAC
 * (RFC 4493), S2V and CTR are implemented here on top of it.
 *
 * This is the AEAD that NTS mandates: RFC 8915 §5.1 requires every NTS
 * implementation to support AEAD_AES_SIV_CMAC_256 (IANA numeric id 15), and it
 * is the algorithm this server negotiates. It also seals the cookies
 * (nts.h), where SIV's misuse resistance is the point: a repeated nonce costs
 * only the leak that two cookies are identical, not the catastrophic key
 * recovery a repeated GCM nonce would cause.
 *
 * Key sizing, which the name makes needlessly confusing: "AES-SIV-CMAC-256"
 * takes a **256-bit** SIV key and splits it into two **128-bit** AES keys —
 * the left half keys S2V/CMAC, the right half keys CTR. So the underlying
 * block cipher is AES-128 throughout (RFC 5297 §2.2).
 *
 * Output layout is RFC 5297 §2.6: SIV || ciphertext, with the 16-octet
 * synthetic IV first, so a ciphertext is always exactly AES_SIV_TAG_LEN longer
 * than its plaintext.
 */

#ifndef STS1000_CORE_NTS_AES_SIV_H_
#define STS1000_CORE_NTS_AES_SIV_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "port/port_crypto.h"

#ifdef __cplusplus
extern "C" {
#endif

/** SIV key length: 256 bits, split into two AES-128 keys. */
#define AES_SIV_KEY_LEN 32U
/** Synthetic IV / tag length, and the ciphertext expansion. */
#define AES_SIV_TAG_LEN 16U
/** AES block size. */
#define AES_SIV_BLOCK 16U

/**
 * Associated-data components accepted per call.
 *
 * RFC 5297 allows up to 126. Four is enough for everything here — NTS uses one
 * (the packet prefix), the cookie sealing uses two (the cleartext cookie header
 * and the nonce) — and the cap keeps the S2V loop bounded.
 */
#define AES_SIV_MAX_AD 4U

/** One associated-data component. A zero-length component is legal. */
typedef struct {
	const uint8_t *p;
	size_t len;
} aes_siv_ad_t;

/** Key schedule. Caller-owned; all fields private. */
typedef struct {
	port_crypto_t crypto;
	uint8_t k_s2v[AES_SIV_BLOCK]; /* leftmost half: S2V / CMAC */
	uint8_t k_ctr[AES_SIV_BLOCK]; /* rightmost half: CTR */
	bool ready;
} aes_siv_ctx_t;

/**
 * Split @p key into its two halves and bind the crypto port.
 *
 * @param key_len  Must be AES_SIV_KEY_LEN. Other SIV key sizes (384/512-bit,
 *                 keying AES-192/AES-256) are not implemented: NTS does not
 *                 negotiate them and the port only offers 128- and 256-bit AES.
 *
 * @retval 0        Ready.
 * @retval -EINVAL  NULL argument, wrong key length, or a port with no
 *                  aes_ecb_encrypt.
 */
int aes_siv_init(aes_siv_ctx_t *c, const port_crypto_t *crypto,
		 const uint8_t *key, size_t key_len);

/**
 * SIV-Encrypt (RFC 5297 §2.6).
 *
 * There is no separate nonce argument by design: RFC 5297 §3 specifies a nonce
 * as simply the last associated-data component. Pass it as the final element of
 * @p ad to get nonce-based operation, or omit it for the deterministic mode.
 *
 * @param ad       Associated-data components, in order. May be NULL when
 *                 @p n_ad is 0.
 * @param n_ad     0..AES_SIV_MAX_AD.
 * @param pt       Plaintext; may be NULL when @p pt_len is 0.
 * @param out      Receives SIV || ciphertext, @p pt_len + AES_SIV_TAG_LEN
 *                 octets. May alias the plaintext exactly — that is,
 *                 `pt == out + AES_SIV_TAG_LEN`, which lets a caller build a
 *                 packet body in place and seal it where it lies. Any other
 *                 overlap is undefined.
 * @param out_len  Receives the written length. Untouched on failure.
 *
 * @retval 0         Encrypted.
 * @retval -EINVAL   NULL argument, uninitialised context, or n_ad too large.
 * @retval -ENOSPC   @p out_cap is under pt_len + AES_SIV_TAG_LEN.
 * @retval -EIO      The AES port failed.
 */
int aes_siv_encrypt(const aes_siv_ctx_t *c, const aes_siv_ad_t *ad, size_t n_ad,
		    const uint8_t *pt, size_t pt_len, uint8_t *out,
		    size_t out_cap, size_t *out_len);

/**
 * SIV-Decrypt (RFC 5297 §2.7). The tag comparison is constant time.
 *
 * @param in       SIV || ciphertext, at least AES_SIV_TAG_LEN octets.
 * @param pt       Receives in_len − AES_SIV_TAG_LEN octets. May alias exactly
 *                 as `pt == in + AES_SIV_TAG_LEN`; any other overlap is
 *                 undefined.
 *
 * @retval 0         Authentic; @p pt and @p pt_len are valid.
 * @retval -EINVAL   NULL argument, uninitialised context, or n_ad too large.
 * @retval -EBADMSG  @p in_len is under AES_SIV_TAG_LEN, or the tag is wrong.
 *                   @p pt may hold unauthenticated plaintext — the caller MUST
 *                   NOT use it.
 * @retval -ENOSPC   @p pt_cap is too small.
 * @retval -EIO      The AES port failed.
 */
int aes_siv_decrypt(const aes_siv_ctx_t *c, const aes_siv_ad_t *ad, size_t n_ad,
		    const uint8_t *in, size_t in_len, uint8_t *pt,
		    size_t pt_cap, size_t *pt_len);

/**
 * AES-CMAC (RFC 4493) over one contiguous message.
 *
 * Exposed because it is the primitive the SIV construction is built from and it
 * is worth testing directly against the RFC 4493 vectors; nothing outside this
 * module needs it otherwise.
 *
 * @param key  16 octets.
 * @param out  16 octets.
 *
 * @retval 0        Computed.
 * @retval -EINVAL  NULL argument, or a port with no aes_ecb_encrypt.
 * @retval -EIO     The AES port failed.
 */
int aes_cmac(const port_crypto_t *crypto, const uint8_t *key,
	     const uint8_t *msg, size_t msg_len, uint8_t out[AES_SIV_BLOCK]);

/** Constant-time equality over @p n octets. */
bool aes_siv_ct_memeq(const uint8_t *a, const uint8_t *b, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_NTS_AES_SIV_H_ */
