/*
 * STS1000 "Meridian" — core/nts unit tests.
 *
 * Scope split from test_aes_siv.c: that suite pins the AEAD against the
 * RFC 5297 vectors, so this one takes AES-SIV as given and tests what NTS does
 * with it — the master key ring and its rotation window, the cookie format,
 * the RFC 8915 §5 extension-field datapath, and the §4 NTS-KE record protocol.
 *
 * Provenance of the expectations. There is no published byte vector for an NTS
 * exchange: cookies are explicitly server-defined (RFC 8915 §6) and every real
 * one is nonce-randomised. So the anchors here are the *structural* rules the
 * RFC states, checked against bytes this code did not choose:
 *
 *   - Field and record layouts are asserted octet by octet against the figures
 *     in RFC 8915 §4.1 and §5.3–§5.6 — type numbers, the critical bit's
 *     position, the nonce/ciphertext length prefix, the four-octet alignment.
 *   - The response's authenticator is verified by decrypting it with an
 *     independently held S2C key and an independently reconstructed associated
 *     data, not by asking the module whether it agrees with itself.
 *   - Requests are assembled by the test as a client would, using aes_siv
 *     directly, so a change in what the server expects breaks the test rather
 *     than silently changing both sides.
 *   - Every rejection path is reached by mutating a known-good packet, one
 *     field at a time.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "host_aes.h"
#include "host_sha256.h"
#include "nts/nts.h"
#include "util/bytes.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* Comfortably past the 1280-octet working maximum, so a test may deliberately
 * build an over-large request (more placeholders than the server will honour)
 * without the harness being the thing that refuses it. */
#define PKT_CAP 2048U

static host_crypto_t g_hc;
static port_crypto_t g_port;
static nts_keyring_t g_ring;
static nts_ctx_t g_nts;

void setUp(void)
{
	host_crypto_init(&g_hc, 0x5EED1234U);
	g_port = host_crypto_port(&g_hc);
	TEST_ASSERT_EQUAL_INT(0, nts_keyring_init(&g_ring, &g_port, 1000, 0));
	TEST_ASSERT_EQUAL_INT(0, nts_init(&g_nts, &g_ring, 0));
}

void tearDown(void)
{
}

/* ------------------------------------------------------------- packet help */

typedef struct {
	uint8_t buf[PKT_CAP];
	size_t len;
} pkt_t;

/** A plausible mode-3 NTPv4 client header, so the AAD is not all zeros. */
static void pkt_hdr(pkt_t *p)
{
	memset(p, 0, sizeof(*p));
	p->buf[0] = 0x23U; /* LI 0, VN 4, mode 3 */
	p->buf[1] = 0x00U;
	p->buf[2] = 0x06U; /* poll 64 s */
	p->buf[3] = 0xE9U; /* precision -23 */
	bytes_put_be64(&p->buf[40], UINT64_C(0xE701020304050607));
	p->len = NTS_NTP_HDR_LEN;
}

static void pkt_ef(pkt_t *p, uint16_t type, const uint8_t *body, size_t body_len)
{
	size_t field = 4U + body_len;

	TEST_ASSERT_TRUE(p->len + field <= sizeof(p->buf));
	TEST_ASSERT_EQUAL_size_t(0U, field & 3U);
	bytes_put_be16(&p->buf[p->len], type);
	bytes_put_be16(&p->buf[p->len + 2U], (uint16_t)field);
	if (body_len != 0U) {
		memcpy(&p->buf[p->len + 4U], body, body_len);
	}
	p->len += field;
}

/**
 * Append the client's authenticator over everything written so far, exactly as
 * RFC 8915 §5.6 lays it out. Written as a client would: the test owns the C2S
 * key and computes the AEAD itself.
 */
static void pkt_auth(pkt_t *p, const uint8_t c2s[NTS_KEY_LEN], uint8_t nonce_seed)
{
	aes_siv_ctx_t siv;
	aes_siv_ad_t ad[2];
	uint8_t nonce[NTS_NONCE_LEN];
	size_t base = p->len;
	size_t n = 0U;

	for (size_t i = 0U; i < sizeof(nonce); i++) {
		nonce[i] = (uint8_t)(nonce_seed + i);
	}

	bytes_put_be16(&p->buf[base], (uint16_t)NTS_EF_AUTH);
	bytes_put_be16(&p->buf[base + 2U], 40U);
	bytes_put_be16(&p->buf[base + 4U], (uint16_t)NTS_NONCE_LEN);
	bytes_put_be16(&p->buf[base + 6U], (uint16_t)AES_SIV_TAG_LEN);
	memcpy(&p->buf[base + 8U], nonce, sizeof(nonce));

	TEST_ASSERT_EQUAL_INT(0, aes_siv_init(&siv, &g_port, c2s, NTS_KEY_LEN));
	ad[0].p = p->buf;
	ad[0].len = base;
	ad[1].p = nonce;
	ad[1].len = sizeof(nonce);
	TEST_ASSERT_EQUAL_INT(0, aes_siv_encrypt(&siv, ad, 2U, NULL, 0U,
						 &p->buf[base + 24U],
						 AES_SIV_TAG_LEN, &n));
	p->len = base + 40U;
}

/** Keys a test client "negotiated"; distinct and non-trivial. */
static void make_keys(nts_cookie_keys_t *k)
{
	memset(k, 0, sizeof(*k));
	k->aead_id = (uint16_t)NTS_AEAD_AES_SIV_CMAC_256;
	for (size_t i = 0U; i < NTS_KEY_LEN; i++) {
		k->c2s[i] = (uint8_t)(0xC0U + i);
		k->s2c[i] = (uint8_t)(0x50U + i);
	}
}

static void make_uniq(uint8_t *u, size_t n)
{
	for (size_t i = 0U; i < n; i++) {
		u[i] = (uint8_t)(0x11U * (i + 1U));
	}
}

/** A complete, valid NTS request with @p placeholders cookie placeholders. */
static void build_request(pkt_t *p, const nts_cookie_keys_t *keys,
			  unsigned placeholders)
{
	uint8_t uniq[NTS_UNIQ_MIN];
	uint8_t cookie[NTS_COOKIE_LEN];
	uint8_t filler[NTS_COOKIE_LEN];

	make_uniq(uniq, sizeof(uniq));
	memset(filler, 0, sizeof(filler));
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_seal(&g_ring, keys, cookie));

	pkt_hdr(p);
	pkt_ef(p, (uint16_t)NTS_EF_UNIQUE_ID, uniq, sizeof(uniq));
	pkt_ef(p, (uint16_t)NTS_EF_COOKIE, cookie, sizeof(cookie));
	for (unsigned i = 0U; i < placeholders; i++) {
		pkt_ef(p, (uint16_t)NTS_EF_COOKIE_PLACEHOLDER, filler,
		       sizeof(filler));
	}
	pkt_auth(p, keys->c2s, 0x40U);
}

/* --------------------------------------------------------------- key ring */

static void test_keyring_init_and_export(void)
{
	uint16_t id = 0U;
	uint8_t key[NTS_MASTER_KEY_LEN];
	int64_t created = 0;
	bool current = false;

	TEST_ASSERT_EQUAL_INT(0, nts_keyring_export(&g_ring, 0U, &id, key, &created,
						    &current));
	TEST_ASSERT_EQUAL_UINT16(1U, id);
	TEST_ASSERT_EQUAL_INT64(1000, created);
	TEST_ASSERT_TRUE(current);
	/* Untouched slots must report empty, not zero-filled keys. */
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      nts_keyring_export(&g_ring, 1U, NULL, NULL, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      nts_keyring_export(&g_ring, NTS_MASTER_KEY_SLOTS, NULL,
						 NULL, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      nts_keyring_export(NULL, 0U, NULL, NULL, NULL, NULL));
}

static void test_keyring_init_rejects_bad_ports(void)
{
	nts_keyring_t r;
	port_crypto_t p;

	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_keyring_init(NULL, &g_port, 0, 0));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_keyring_init(&r, NULL, 0, 0));

	p = g_port;
	p.rand = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_keyring_init(&r, &p, 0, 0));
	p = g_port;
	p.aes_ecb_encrypt = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_keyring_init(&r, &p, 0, 0));

	/* An entropy failure at init must leave nothing usable behind. */
	host_crypto_init(&g_hc, 1U);
	g_hc.fail_rand_in = 1U;
	p = host_crypto_port(&g_hc);
	TEST_ASSERT_EQUAL_INT(-EIO, nts_keyring_init(&r, &p, 0, 0));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_keyring_rotate(&r, 0));
}

static void test_keyring_rotation_window(void)
{
	nts_cookie_keys_t keys;
	nts_cookie_keys_t back;
	uint8_t cookie[NTS_COOKIE_LEN];

	make_keys(&keys);
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_seal(&g_ring, &keys, cookie));
	TEST_ASSERT_EQUAL_UINT16(1U, bytes_get_be16(cookie));

	/*
	 * A cookie stays usable for as many rotations as the ring has slots
	 * minus one, then falls off. That window is the whole reason the ring
	 * exists, so walk it rather than assert the endpoint.
	 */
	for (unsigned i = 1U; i < NTS_MASTER_KEY_SLOTS; i++) {
		TEST_ASSERT_EQUAL_INT(0, nts_keyring_rotate(&g_ring,
							    1000 + 1000 * (int)i));
		TEST_ASSERT_EQUAL_INT(0, nts_cookie_unseal(&g_ring, cookie,
							   sizeof(cookie), &back));
		TEST_ASSERT_EQUAL_HEX8_ARRAY(keys.c2s, back.c2s, NTS_KEY_LEN);
	}

	TEST_ASSERT_EQUAL_INT(0, nts_keyring_rotate(&g_ring, 9000));
	TEST_ASSERT_EQUAL_INT(-ENOENT, nts_cookie_unseal(&g_ring, cookie,
							 sizeof(cookie), &back));

	/* Fresh cookies use the new key and are unaffected. */
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_seal(&g_ring, &keys, cookie));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)(NTS_MASTER_KEY_SLOTS + 1U),
				 bytes_get_be16(cookie));
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_unseal(&g_ring, cookie, sizeof(cookie),
						   &back));
}

static void test_keyring_tick_and_failures(void)
{
	nts_keyring_t r;

	TEST_ASSERT_EQUAL_INT(0, nts_keyring_init(&r, &g_port, 0, 5000));
	TEST_ASSERT_EQUAL_INT(0, nts_keyring_tick(&r, 4999));
	TEST_ASSERT_EQUAL_INT(1, nts_keyring_tick(&r, 5000));
	TEST_ASSERT_EQUAL_INT(0, nts_keyring_tick(&r, 9999));
	TEST_ASSERT_EQUAL_INT(1, nts_keyring_tick(&r, 10000));

	/* A default interval is a day, so nothing is due a second later. */
	TEST_ASSERT_EQUAL_INT(0, nts_keyring_tick(&g_ring, 2000));

	g_hc.fail_rand_in = 1U;
	TEST_ASSERT_EQUAL_INT(-EIO, nts_keyring_tick(&r, 100000));
	g_hc.fail_rand_in = 1U;
	TEST_ASSERT_EQUAL_INT(-EIO, nts_keyring_rotate(&r, 100000));

	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_keyring_tick(NULL, 0));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_keyring_rotate(NULL, 0));
}

static void test_keyring_install_restores_across_reboot(void)
{
	nts_cookie_keys_t keys;
	nts_cookie_keys_t back;
	nts_keyring_t fresh;
	uint8_t cookie[NTS_COOKIE_LEN];
	uint8_t saved[NTS_MASTER_KEY_LEN];
	uint16_t id = 0U;

	make_keys(&keys);
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_seal(&g_ring, &keys, cookie));
	TEST_ASSERT_EQUAL_INT(0, nts_keyring_export(&g_ring, 0U, &id, saved, NULL,
						    NULL));

	/*
	 * Reboot: a brand-new ring cannot read the old cookie until the sealed
	 * master key is restored (spec §4.2). The rejection is -EBADMSG rather
	 * than -ENOENT because a cold ring mints ids from 1 again, so the id
	 * matches a key that is not the one that sealed this cookie. Both codes
	 * mean the same thing to the datapath — NTS NAK, go and re-key — which
	 * is why the id space is not worth persisting on its own.
	 */
	TEST_ASSERT_EQUAL_INT(0, nts_keyring_init(&fresh, &g_port, 0, 0));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, nts_cookie_unseal(&fresh, cookie,
							  sizeof(cookie), &back));

	TEST_ASSERT_EQUAL_INT(0, nts_keyring_install(&fresh, id, saved, 0, true));
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_unseal(&fresh, cookie, sizeof(cookie),
						   &back));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(keys.s2c, back.s2c, NTS_KEY_LEN);

	/* Installing the same id again replaces that slot rather than consuming
	 * a second one. */
	TEST_ASSERT_EQUAL_INT(0, nts_keyring_install(&fresh, id, saved, 0, false));

	/* Ids minted after a restore must not collide with the restored one. */
	TEST_ASSERT_EQUAL_INT(0, nts_keyring_install(&fresh, 0x00FFU, saved, 0, false));
	TEST_ASSERT_EQUAL_INT(0, nts_keyring_rotate(&fresh, 1));
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_seal(&fresh, &keys, cookie));
	TEST_ASSERT_EQUAL_UINT16(0x0100U, bytes_get_be16(cookie));

	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_keyring_install(&fresh, 0U, saved, 0, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_keyring_install(&fresh, 1U, NULL, 0, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_keyring_install(NULL, 1U, saved, 0, false));
}

/* ---------------------------------------------------------------- cookies */

static void test_cookie_layout_and_roundtrip(void)
{
	nts_cookie_keys_t keys;
	nts_cookie_keys_t back;
	uint8_t a[NTS_COOKIE_LEN];
	uint8_t b[NTS_COOKIE_LEN];

	make_keys(&keys);
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_seal(&g_ring, &keys, a));

	/* Documented layout: key id, AEAD id, nonce, SIV, ciphertext. */
	TEST_ASSERT_EQUAL_UINT16(1U, bytes_get_be16(&a[0]));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTS_AEAD_AES_SIV_CMAC_256,
				 bytes_get_be16(&a[2]));
	TEST_ASSERT_EQUAL_size_t(100U, NTS_COOKIE_LEN);

	memset(&back, 0, sizeof(back));
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_unseal(&g_ring, a, sizeof(a), &back));
	TEST_ASSERT_EQUAL_UINT16(keys.aead_id, back.aead_id);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(keys.c2s, back.c2s, NTS_KEY_LEN);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(keys.s2c, back.s2c, NTS_KEY_LEN);

	/* Two cookies for the same keys must differ: the nonce is fresh. A
	 * repeat would mean the entropy port is not reaching the seal. */
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_seal(&g_ring, &keys, b));
	TEST_ASSERT_FALSE(aes_siv_ct_memeq(a, b, sizeof(a)));
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_unseal(&g_ring, b, sizeof(b), &back));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(keys.c2s, back.c2s, NTS_KEY_LEN);
}

static void test_cookie_rejects_mutation(void)
{
	nts_cookie_keys_t keys;
	nts_cookie_keys_t back;
	uint8_t good[NTS_COOKIE_LEN];
	uint8_t bad[NTS_COOKIE_LEN];

	make_keys(&keys);
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_seal(&g_ring, &keys, good));

	/* Wrong length is not a cookie of ours at all. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG, nts_cookie_unseal(&g_ring, good,
							  sizeof(good) - 1U, &back));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, nts_cookie_unseal(&g_ring, good, 0U, &back));

	/* An unknown key id is the benign expiry case and is distinguishable
	 * from a forgery, which matters: one earns a NAK, the other silence. */
	memcpy(bad, good, sizeof(bad));
	bad[1] ^= 0x10U;
	TEST_ASSERT_EQUAL_INT(-ENOENT, nts_cookie_unseal(&g_ring, bad, sizeof(bad),
							 &back));

	/* The AEAD id is checked before any crypto runs. */
	memcpy(bad, good, sizeof(bad));
	bytes_put_be16(&bad[2], 30U);
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, nts_cookie_unseal(&g_ring, bad, sizeof(bad),
							  &back));

	/* Everything else — nonce, SIV, ciphertext — is covered by the tag. The
	 * AEAD id is too, which a valid id over a mutated body proves. */
	for (size_t i = 2U; i < sizeof(good); i++) {
		memcpy(bad, good, sizeof(bad));
		bad[i] ^= (uint8_t)(1U << (i % 8U));
		if (bytes_get_be16(&bad[2]) != NTS_AEAD_AES_SIV_CMAC_256) {
			continue; /* covered by the -ENOTSUP case above */
		}
		TEST_ASSERT_EQUAL_INT(-EBADMSG,
				      nts_cookie_unseal(&g_ring, bad, sizeof(bad),
							&back));
	}
}

static void test_cookie_bad_arguments_and_port_failures(void)
{
	nts_cookie_keys_t keys;
	nts_cookie_keys_t back;
	uint8_t cookie[NTS_COOKIE_LEN];

	make_keys(&keys);

	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_cookie_seal(NULL, &keys, cookie));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_cookie_seal(&g_ring, NULL, cookie));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_cookie_seal(&g_ring, &keys, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_cookie_unseal(NULL, cookie, 100U, &back));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_cookie_unseal(&g_ring, NULL, 100U, &back));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_cookie_unseal(&g_ring, cookie, 100U, NULL));

	/* Sealing under an algorithm we cannot honour would mint a cookie no
	 * one can use. */
	keys.aead_id = 30U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_cookie_seal(&g_ring, &keys, cookie));
	keys.aead_id = (uint16_t)NTS_AEAD_AES_SIV_CMAC_256;

	g_hc.fail_rand_in = 1U;
	TEST_ASSERT_EQUAL_INT(-EIO, nts_cookie_seal(&g_ring, &keys, cookie));

	g_hc.fail_aes_in = 1U;
	TEST_ASSERT_EQUAL_INT(-EIO, nts_cookie_seal(&g_ring, &keys, cookie));

	TEST_ASSERT_EQUAL_INT(0, nts_cookie_seal(&g_ring, &keys, cookie));
	g_hc.fail_aes_in = 1U;
	TEST_ASSERT_EQUAL_INT(-EIO, nts_cookie_unseal(&g_ring, cookie,
						      sizeof(cookie), &back));
}

/* ------------------------------------------------- request-side processing */

static void test_process_plain_ntp_is_not_ours(void)
{
	nts_req_t req;
	pkt_t p;

	pkt_hdr(&p);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_NONE,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* A non-NTS extension field is somebody else's business, not a fault. */
	{
		uint8_t body[28] = { 0 };

		pkt_ef(&p, 0x0007U, body, sizeof(body));
	}
	TEST_ASSERT_EQUAL_INT(NTS_ACT_NONE,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_process_request(&g_nts, p.buf, 47U, &req));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_process_request(NULL, p.buf, p.len, &req));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_process_request(&g_nts, NULL, p.len, &req));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_process_request(&g_nts, p.buf, p.len, NULL));
}

static void test_process_accepts_a_valid_request(void)
{
	nts_cookie_keys_t keys;
	nts_req_t req;
	nts_stats_t st;
	pkt_t p;
	uint8_t uniq[NTS_UNIQ_MIN];

	make_keys(&keys);
	make_uniq(uniq, sizeof(uniq));
	build_request(&p, &keys, 3U);

	TEST_ASSERT_EQUAL_INT(NTS_ACT_OK,
			      nts_process_request(&g_nts, p.buf, p.len, &req));
	TEST_ASSERT_TRUE(req.has_uniq);
	TEST_ASSERT_TRUE(req.has_keys);
	TEST_ASSERT_EQUAL_size_t(sizeof(uniq), req.uniq_len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(uniq, req.uniq, sizeof(uniq));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(keys.c2s, req.keys.c2s, NTS_KEY_LEN);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(keys.s2c, req.keys.s2c, NTS_KEY_LEN);
	/* One per placeholder, plus one to replace the cookie just spent. */
	TEST_ASSERT_EQUAL_UINT8(4U, req.cookies_wanted);

	TEST_ASSERT_EQUAL_INT(0, nts_stats_get(&g_nts, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.rx);
	TEST_ASSERT_EQUAL_UINT64(1U, st.ok);
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_stats_get(NULL, &st));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_stats_get(&g_nts, NULL));
}

static void test_process_clamps_cookie_demand(void)
{
	nts_cookie_keys_t keys;
	nts_req_t req;
	pkt_t p;

	/* A client asking for more cookies than the server issues per response
	 * gets the server's limit, not its own. */
	make_keys(&keys);
	build_request(&p, &keys, NTS_COOKIES_MAX + 4U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_OK,
			      nts_process_request(&g_nts, p.buf, p.len, &req));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)NTS_COOKIES_MAX, req.cookies_wanted);
}

static void test_process_rejects_structural_faults(void)
{
	nts_cookie_keys_t keys;
	nts_req_t req;
	nts_stats_t st;
	pkt_t p;
	uint8_t uniq[NTS_UNIQ_MIN];
	uint8_t cookie[NTS_COOKIE_LEN];
	uint8_t shortid[NTS_UNIQ_MIN - 4U];
	const size_t cookie_ef = NTS_NTP_HDR_LEN + 4U + NTS_UNIQ_MIN;

	make_keys(&keys);
	make_uniq(uniq, sizeof(uniq));
	make_uniq(shortid, sizeof(shortid));
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_seal(&g_ring, &keys, cookie));

	/* No Unique Identifier. */
	pkt_hdr(&p);
	pkt_ef(&p, (uint16_t)NTS_EF_COOKIE, cookie, sizeof(cookie));
	pkt_auth(&p, keys.c2s, 0x40U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* No cookie: nothing to recover keys from, and nothing to NAK about. */
	pkt_hdr(&p);
	pkt_ef(&p, (uint16_t)NTS_EF_UNIQUE_ID, uniq, sizeof(uniq));
	pkt_auth(&p, keys.c2s, 0x40U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* No authenticator: an unprotected packet wearing NTS fields. */
	pkt_hdr(&p);
	pkt_ef(&p, (uint16_t)NTS_EF_UNIQUE_ID, uniq, sizeof(uniq));
	pkt_ef(&p, (uint16_t)NTS_EF_COOKIE, cookie, sizeof(cookie));
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* Two Unique Identifiers — which one would the response echo? */
	pkt_hdr(&p);
	pkt_ef(&p, (uint16_t)NTS_EF_UNIQUE_ID, uniq, sizeof(uniq));
	pkt_ef(&p, (uint16_t)NTS_EF_UNIQUE_ID, uniq, sizeof(uniq));
	pkt_ef(&p, (uint16_t)NTS_EF_COOKIE, cookie, sizeof(cookie));
	pkt_auth(&p, keys.c2s, 0x40U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* Two cookies. */
	pkt_hdr(&p);
	pkt_ef(&p, (uint16_t)NTS_EF_UNIQUE_ID, uniq, sizeof(uniq));
	pkt_ef(&p, (uint16_t)NTS_EF_COOKIE, cookie, sizeof(cookie));
	pkt_ef(&p, (uint16_t)NTS_EF_COOKIE, cookie, sizeof(cookie));
	pkt_auth(&p, keys.c2s, 0x40U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* Unique Identifier below the RFC 8915 §5.3 floor of 32 octets. */
	pkt_hdr(&p);
	pkt_ef(&p, (uint16_t)NTS_EF_UNIQUE_ID, shortid, sizeof(shortid));
	pkt_ef(&p, (uint16_t)NTS_EF_COOKIE, cookie, sizeof(cookie));
	pkt_auth(&p, keys.c2s, 0x40U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* Unique Identifier longer than the echo buffer. */
	{
		uint8_t big[NTS_UNIQ_MAX + 4U];

		make_uniq(big, sizeof(big));
		pkt_hdr(&p);
		pkt_ef(&p, (uint16_t)NTS_EF_UNIQUE_ID, big, sizeof(big));
		pkt_ef(&p, (uint16_t)NTS_EF_COOKIE, cookie, sizeof(cookie));
		pkt_auth(&p, keys.c2s, 0x40U);
		TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
				      nts_process_request(&g_nts, p.buf, p.len,
							  &req));
	}

	/*
	 * Malformed field headers, corrupted at the *cookie* field so the walk
	 * has already recognised NTS fields and the packet is unambiguously a
	 * broken NTS request rather than an unrelated one.
	 */
	build_request(&p, &keys, 0U);
	bytes_put_be16(&p.buf[cookie_ef + 2U], 35U); /* not a multiple of 4 */
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	build_request(&p, &keys, 0U);
	bytes_put_be16(&p.buf[cookie_ef + 2U], 0U); /* shorter than its header */
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	build_request(&p, &keys, 0U);
	bytes_put_be16(&p.buf[cookie_ef + 2U], 0x0400U); /* past the datagram */
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* A truncated datagram: the authenticator no longer fits. */
	build_request(&p, &keys, 0U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len - 2U, &req));

	TEST_ASSERT_EQUAL_INT(0, nts_stats_get(&g_nts, &st));
	TEST_ASSERT_EQUAL_UINT64(11U, st.dropped);
	TEST_ASSERT_EQUAL_UINT64(0U, st.ok);

	/*
	 * The one case that is deliberately *not* a drop: garbage in the very
	 * first extension field, before anything identifies the packet as NTS.
	 * That is indistinguishable from an ordinary NTP client with a broken
	 * tail, and core/ntp tolerates those. Claiming it as an NTS fault would
	 * let a malformed non-NTS packet be counted against this module.
	 */
	build_request(&p, &keys, 0U);
	bytes_put_be16(&p.buf[NTS_NTP_HDR_LEN + 2U], 35U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_NONE,
			      nts_process_request(&g_nts, p.buf, p.len, &req));
	TEST_ASSERT_EQUAL_INT(0, nts_stats_get(&g_nts, &st));
	TEST_ASSERT_EQUAL_UINT64(11U, st.dropped);
	TEST_ASSERT_EQUAL_UINT64(11U, st.rx);
}

static void test_process_naks_an_unusable_cookie(void)
{
	nts_cookie_keys_t keys;
	nts_req_t req;
	nts_stats_t st;
	pkt_t p;
	uint8_t uniq[NTS_UNIQ_MIN];
	uint8_t cookie[NTS_COOKIE_LEN];
	uint8_t stub[NTS_COOKIE_LEN - 4U];

	make_keys(&keys);
	make_uniq(uniq, sizeof(uniq));

	/* Cookie sealed under a key that has since aged out of the ring. */
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_seal(&g_ring, &keys, cookie));
	for (unsigned i = 0U; i <= NTS_MASTER_KEY_SLOTS; i++) {
		TEST_ASSERT_EQUAL_INT(0, nts_keyring_rotate(&g_ring,
							    2000 + 1000 * (int)i));
	}
	pkt_hdr(&p);
	pkt_ef(&p, (uint16_t)NTS_EF_UNIQUE_ID, uniq, sizeof(uniq));
	pkt_ef(&p, (uint16_t)NTS_EF_COOKIE, cookie, sizeof(cookie));
	pkt_auth(&p, keys.c2s, 0x40U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_NAK,
			      nts_process_request(&g_nts, p.buf, p.len, &req));
	/* The echo survives, because the NAK has to carry it. */
	TEST_ASSERT_TRUE(req.has_uniq);
	TEST_ASSERT_FALSE(req.has_keys);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(uniq, req.uniq, sizeof(uniq));

	/* A cookie of the wrong size — another server's format. */
	memset(stub, 0x5AU, sizeof(stub));
	pkt_hdr(&p);
	pkt_ef(&p, (uint16_t)NTS_EF_UNIQUE_ID, uniq, sizeof(uniq));
	pkt_ef(&p, (uint16_t)NTS_EF_COOKIE, stub, sizeof(stub));
	pkt_auth(&p, keys.c2s, 0x40U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_NAK,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	TEST_ASSERT_EQUAL_INT(0, nts_stats_get(&g_nts, &st));
	TEST_ASSERT_EQUAL_UINT64(2U, st.nak);
}

static void test_process_drops_forgeries(void)
{
	nts_cookie_keys_t keys;
	nts_req_t req;
	pkt_t p;
	size_t auth_off;

	make_keys(&keys);

	/* A modified NTP header: it is associated data, so it is authenticated
	 * even though it is not encrypted. This is the attack NTS exists to
	 * stop — a relay rewriting the timestamps. */
	build_request(&p, &keys, 1U);
	p.buf[2] ^= 0x01U;
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* A modified Unique Identifier, likewise. */
	build_request(&p, &keys, 1U);
	p.buf[NTS_NTP_HDR_LEN + 4U] ^= 0x01U;
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* A modified SIV in the authenticator. */
	build_request(&p, &keys, 0U);
	auth_off = p.len - 40U;
	p.buf[auth_off + 24U] ^= 0x80U;
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* A modified nonce. */
	build_request(&p, &keys, 0U);
	p.buf[p.len - 40U + 8U] ^= 0x01U;
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* Length prefixes that do not fit the field they describe. */
	build_request(&p, &keys, 0U);
	bytes_put_be16(&p.buf[p.len - 40U + 4U], 4096U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* A ciphertext too short to hold a SIV. */
	build_request(&p, &keys, 0U);
	bytes_put_be16(&p.buf[p.len - 40U + 6U], 8U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* An authenticator body too short even for its own length prefix. */
	build_request(&p, &keys, 0U);
	auth_off = p.len - 40U;
	bytes_put_be16(&p.buf[auth_off + 2U], 8U);
	p.len = auth_off + 8U;
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));
}

static void test_process_rejects_oversized_encrypted_payload(void)
{
	nts_cookie_keys_t keys;
	nts_req_t req;
	pkt_t p;
	uint8_t uniq[NTS_UNIQ_MIN];
	uint8_t cookie[NTS_COOKIE_LEN];
	uint8_t big[NTS_REQ_PLAINTEXT_MAX + 16U];
	aes_siv_ctx_t siv;
	aes_siv_ad_t ad[2];
	uint8_t nonce[NTS_NONCE_LEN];
	size_t base;
	size_t n = 0U;
	size_t field;

	make_keys(&keys);
	make_uniq(uniq, sizeof(uniq));
	memset(big, 0x33U, sizeof(big));
	memset(nonce, 0x77U, sizeof(nonce));
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_seal(&g_ring, &keys, cookie));

	pkt_hdr(&p);
	pkt_ef(&p, (uint16_t)NTS_EF_UNIQUE_ID, uniq, sizeof(uniq));
	pkt_ef(&p, (uint16_t)NTS_EF_COOKIE, cookie, sizeof(cookie));

	base = p.len;
	field = 8U + NTS_NONCE_LEN + AES_SIV_TAG_LEN + sizeof(big);
	bytes_put_be16(&p.buf[base], (uint16_t)NTS_EF_AUTH);
	bytes_put_be16(&p.buf[base + 2U], (uint16_t)field);
	bytes_put_be16(&p.buf[base + 4U], (uint16_t)NTS_NONCE_LEN);
	bytes_put_be16(&p.buf[base + 6U],
		       (uint16_t)(AES_SIV_TAG_LEN + sizeof(big)));
	memcpy(&p.buf[base + 8U], nonce, sizeof(nonce));

	TEST_ASSERT_EQUAL_INT(0, aes_siv_init(&siv, &g_port, keys.c2s, NTS_KEY_LEN));
	ad[0].p = p.buf;
	ad[0].len = base;
	ad[1].p = nonce;
	ad[1].len = sizeof(nonce);
	TEST_ASSERT_EQUAL_INT(0, aes_siv_encrypt(&siv, ad, 2U, big, sizeof(big),
						 &p.buf[base + 24U],
						 AES_SIV_TAG_LEN + sizeof(big), &n));
	p.len = base + field;

	/* Authentic, but larger than the module will buffer: refused rather
	 * than given a 272-octet stack allowance it does not need. */
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));
}

static void test_process_ignores_fields_after_the_authenticator(void)
{
	nts_cookie_keys_t keys;
	nts_req_t req;
	pkt_t p;
	uint8_t junk[28];

	make_keys(&keys);
	memset(junk, 0xEEU, sizeof(junk));

	build_request(&p, &keys, 1U);
	pkt_ef(&p, 0x0104U, junk, sizeof(junk)); /* a second "Unique Identifier" */

	/* Appended outside the authenticated region, so it can only be an
	 * attacker's. Honouring it would make the duplicate-field check
	 * defeatable by anyone on the path; rejecting the packet would make
	 * denial of service just as easy. Ignore. */
	TEST_ASSERT_EQUAL_INT(NTS_ACT_OK,
			      nts_process_request(&g_nts, p.buf, p.len, &req));
	TEST_ASSERT_EQUAL_size_t(NTS_UNIQ_MIN, req.uniq_len);
	TEST_ASSERT_EQUAL_UINT8(2U, req.cookies_wanted);
}

/* ------------------------------------------------ response-side generation */

/**
 * Verify a protected response the way a client would: walk the fields, decrypt
 * the authenticator with the S2C key the test holds, and read the cookies out
 * of the plaintext. Returns the cookie count.
 */
static unsigned check_response(const uint8_t *pkt, size_t len,
			       const nts_cookie_keys_t *keys, const uint8_t *uniq,
			       size_t uniq_len)
{
	aes_siv_ctx_t siv;
	aes_siv_ad_t ad[2];
	uint8_t plain[NTS_COOKIES_MAX * NTS_COOKIE_EF_LEN];
	size_t off = NTS_NTP_HDR_LEN;
	size_t auth_off;
	size_t nonce_len;
	size_t ct_len;
	size_t pt_len = 0U;
	unsigned cookies = 0U;

	/* Field 1: the echoed Unique Identifier, authenticated but in clear. */
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTS_EF_UNIQUE_ID, bytes_get_be16(&pkt[off]));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)(4U + uniq_len),
				 bytes_get_be16(&pkt[off + 2U]));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(uniq, &pkt[off + 4U], uniq_len);
	off += 4U + uniq_len;

	/* Field 2: the authenticator, and nothing after it. */
	auth_off = off;
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTS_EF_AUTH, bytes_get_be16(&pkt[off]));
	TEST_ASSERT_EQUAL_size_t(len - auth_off, bytes_get_be16(&pkt[off + 2U]));
	nonce_len = bytes_get_be16(&pkt[off + 4U]);
	ct_len = bytes_get_be16(&pkt[off + 6U]);
	TEST_ASSERT_EQUAL_size_t(NTS_NONCE_LEN, nonce_len);
	TEST_ASSERT_EQUAL_size_t(len - auth_off - 8U - NTS_NONCE_LEN, ct_len);

	TEST_ASSERT_EQUAL_INT(0, aes_siv_init(&siv, &g_port, keys->s2c, NTS_KEY_LEN));
	ad[0].p = pkt;
	ad[0].len = auth_off;
	ad[1].p = &pkt[auth_off + 8U];
	ad[1].len = nonce_len;
	TEST_ASSERT_EQUAL_INT(0, aes_siv_decrypt(&siv, ad, 2U,
						 &pkt[auth_off + 8U + nonce_len],
						 ct_len, plain, sizeof(plain),
						 &pt_len));

	/* The plaintext is a run of NTS Cookie fields and nothing else. */
	TEST_ASSERT_EQUAL_size_t(0U, pt_len % NTS_COOKIE_EF_LEN);
	for (size_t i = 0U; i < pt_len; i += NTS_COOKIE_EF_LEN) {
		nts_cookie_keys_t back;

		TEST_ASSERT_EQUAL_UINT16((uint16_t)NTS_EF_COOKIE,
					 bytes_get_be16(&plain[i]));
		TEST_ASSERT_EQUAL_UINT16((uint16_t)NTS_COOKIE_EF_LEN,
					 bytes_get_be16(&plain[i + 2U]));
		TEST_ASSERT_EQUAL_INT(0, nts_cookie_unseal(&g_ring, &plain[i + 4U],
							   NTS_COOKIE_LEN, &back));
		TEST_ASSERT_EQUAL_HEX8_ARRAY(keys->c2s, back.c2s, NTS_KEY_LEN);
		TEST_ASSERT_EQUAL_HEX8_ARRAY(keys->s2c, back.s2c, NTS_KEY_LEN);
		cookies++;
	}
	return cookies;
}

static void test_response_round_trip(void)
{
	nts_cookie_keys_t keys;
	nts_req_t req;
	nts_stats_t st;
	pkt_t p;
	uint8_t rsp[PKT_CAP];
	uint8_t uniq[NTS_UNIQ_MIN];
	size_t len = NTS_NTP_HDR_LEN;

	make_keys(&keys);
	make_uniq(uniq, sizeof(uniq));
	build_request(&p, &keys, 2U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_OK,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* Stand in for the NTP header the ntp module would have written. */
	memset(rsp, 0, sizeof(rsp));
	rsp[0] = 0x24U; /* LI 0, VN 4, mode 4 */
	rsp[1] = 0x01U; /* stratum 1 */

	TEST_ASSERT_EQUAL_INT(0, nts_append_response(&g_nts, &req, rsp, &len,
						     sizeof(rsp)));
	TEST_ASSERT_EQUAL_UINT(3U, check_response(rsp, len, &keys, uniq,
						  sizeof(uniq)));

	TEST_ASSERT_EQUAL_INT(0, nts_stats_get(&g_nts, &st));
	TEST_ASSERT_EQUAL_UINT64(3U, st.cookies_issued);

	/* Flipping one octet of the response must break the client's check —
	 * this is the assertion that the authenticator really covers the
	 * header, not just the fields. */
	rsp[1] ^= 0x01U;
	{
		aes_siv_ctx_t siv;
		aes_siv_ad_t ad[2];
		uint8_t plain[512];
		size_t auth_off = NTS_NTP_HDR_LEN + 4U + sizeof(uniq);
		size_t pt_len = 0U;

		TEST_ASSERT_EQUAL_INT(0, aes_siv_init(&siv, &g_port, keys.s2c,
						      NTS_KEY_LEN));
		ad[0].p = rsp;
		ad[0].len = auth_off;
		ad[1].p = &rsp[auth_off + 8U];
		ad[1].len = NTS_NONCE_LEN;
		TEST_ASSERT_EQUAL_INT(-EBADMSG,
				      aes_siv_decrypt(&siv, ad, 2U,
						      &rsp[auth_off + 8U +
							   NTS_NONCE_LEN],
						      len - auth_off - 8U -
							      NTS_NONCE_LEN,
						      plain, sizeof(plain),
						      &pt_len));
	}
}

static void test_response_never_exceeds_request(void)
{
	nts_cookie_keys_t keys;
	nts_req_t req;
	pkt_t p;
	uint8_t rsp[PKT_CAP];
	uint8_t uniq[NTS_UNIQ_MIN];
	size_t len;

	make_keys(&keys);
	make_uniq(uniq, sizeof(uniq));

	/*
	 * The anti-amplification contract, exercised across the whole
	 * placeholder range. The cookie count is trimmed to the budget, and
	 * with the maximum number of placeholders the two sizes meet exactly —
	 * which is the arithmetic that makes cookie placeholders work: 36
	 * octets of echo plus a 40-octet authenticator envelope on both sides,
	 * and one 104-octet field per cookie either way.
	 */
	for (unsigned ph = 0U; ph <= NTS_COOKIES_MAX + 2U; ph++) {
		unsigned got;

		build_request(&p, &keys, ph);
		TEST_ASSERT_EQUAL_INT(NTS_ACT_OK,
				      nts_process_request(&g_nts, p.buf, p.len,
							  &req));
		len = NTS_NTP_HDR_LEN;
		memset(rsp, 0, sizeof(rsp));
		TEST_ASSERT_EQUAL_INT(0, nts_append_response(&g_nts, &req, rsp,
							     &len, p.len));
		TEST_ASSERT_TRUE(len <= p.len);
		got = check_response(rsp, len, &keys, uniq, sizeof(uniq));
		if (ph + 1U <= NTS_COOKIES_MAX) {
			TEST_ASSERT_EQUAL_UINT(ph + 1U, got);
		} else {
			TEST_ASSERT_EQUAL_UINT((unsigned)NTS_COOKIES_MAX, got);
		}
	}
}

static void test_response_trims_to_capacity(void)
{
	nts_cookie_keys_t keys;
	nts_req_t req;
	pkt_t p;
	uint8_t rsp[PKT_CAP];
	uint8_t uniq[NTS_UNIQ_MIN];
	size_t base = NTS_NTP_HDR_LEN + 4U + NTS_UNIQ_MIN + 40U;
	size_t len;

	make_keys(&keys);
	make_uniq(uniq, sizeof(uniq));
	build_request(&p, &keys, 5U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_OK,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* Exactly enough for two cookies, one octet short of three. */
	len = NTS_NTP_HDR_LEN;
	TEST_ASSERT_EQUAL_INT(0, nts_append_response(&g_nts, &req, rsp, &len,
						     base + 3U * NTS_COOKIE_EF_LEN -
							     1U));
	TEST_ASSERT_EQUAL_UINT(2U, check_response(rsp, len, &keys, uniq,
						  sizeof(uniq)));

	/* Exactly one cookie fits. */
	len = NTS_NTP_HDR_LEN;
	TEST_ASSERT_EQUAL_INT(0, nts_append_response(&g_nts, &req, rsp, &len,
						     base + NTS_COOKIE_EF_LEN));
	TEST_ASSERT_EQUAL_UINT(1U, check_response(rsp, len, &keys, uniq,
						  sizeof(uniq)));

	/*
	 * Room for the authenticator but not a single cookie is -ENOSPC (L12): a
	 * cookieless response would starve the client, so it is refused rather
	 * than sent. One octet short of a cookie is the boundary.
	 */
	len = NTS_NTP_HDR_LEN;
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      nts_append_response(&g_nts, &req, rsp, &len,
						  base + NTS_COOKIE_EF_LEN - 1U));
	len = NTS_NTP_HDR_LEN;
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      nts_append_response(&g_nts, &req, rsp, &len, base));
	len = NTS_NTP_HDR_LEN;
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      nts_append_response(&g_nts, &req, rsp, &len,
						  NTS_NTP_HDR_LEN + 4U));
}

static void test_response_bad_arguments_and_port_failures(void)
{
	nts_cookie_keys_t keys;
	nts_req_t req;
	nts_req_t empty;
	pkt_t p;
	uint8_t rsp[PKT_CAP];
	size_t len = NTS_NTP_HDR_LEN;

	memset(&empty, 0, sizeof(empty));
	make_keys(&keys);
	build_request(&p, &keys, 1U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_OK,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_append_response(NULL, &req, rsp, &len,
							   sizeof(rsp)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_append_response(&g_nts, NULL, rsp, &len,
							   sizeof(rsp)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_append_response(&g_nts, &req, NULL, &len,
							   sizeof(rsp)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_append_response(&g_nts, &req, rsp, NULL,
							   sizeof(rsp)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_append_response(&g_nts, &empty, rsp, &len,
							   sizeof(rsp)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_append_nak(&g_nts, &empty, rsp, &len,
						      sizeof(rsp)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_append_nak(NULL, &req, rsp, &len,
						      sizeof(rsp)));

	/* A request whose echo survived but whose keys did not — the NAK shape.
	 * It must never produce a protected response. */
	{
		nts_req_t nak = req;

		nak.has_keys = false;
		len = NTS_NTP_HDR_LEN;
		TEST_ASSERT_EQUAL_INT(-EINVAL,
				      nts_append_response(&g_nts, &nak, rsp, &len,
							  sizeof(rsp)));
		TEST_ASSERT_EQUAL_INT(0, nts_append_nak(&g_nts, &nak, rsp, &len,
							sizeof(rsp)));
		TEST_ASSERT_EQUAL_size_t(NTS_NTP_HDR_LEN + 4U + NTS_UNIQ_MIN, len);
	}

	/* Entropy failure while minting a cookie, then while making the
	 * response nonce (the second cookie's rand call is the fourth). */
	len = NTS_NTP_HDR_LEN;
	g_hc.fail_rand_in = 1U;
	TEST_ASSERT_EQUAL_INT(-EIO, nts_append_response(&g_nts, &req, rsp, &len,
							sizeof(rsp)));
	len = NTS_NTP_HDR_LEN;
	g_hc.fail_rand_in = 3U;
	TEST_ASSERT_EQUAL_INT(-EIO, nts_append_response(&g_nts, &req, rsp, &len,
							sizeof(rsp)));

	/* AES failure while sealing a cookie. */
	len = NTS_NTP_HDR_LEN;
	g_hc.fail_aes_in = 2U;
	TEST_ASSERT_EQUAL_INT(-EIO, nts_append_response(&g_nts, &req, rsp, &len,
							sizeof(rsp)));

	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_init(NULL, &g_ring, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_init(&g_nts, NULL, 0U));
	{
		nts_keyring_t blank;

		memset(&blank, 0, sizeof(blank));
		TEST_ASSERT_EQUAL_INT(-EINVAL, nts_init(&g_nts, &blank, 0U));
	}
	TEST_ASSERT_EQUAL_INT(0, nts_init(&g_nts, &g_ring, 200U)); /* clamped */
}

/* ------------------------------------------------------ the ntp hook shape */

static void test_ext_hook(void)
{
	nts_cookie_keys_t keys;
	pkt_t p;
	uint8_t rsp[PKT_CAP];
	uint8_t uniq[NTS_UNIQ_MIN];
	uint32_t kod;
	size_t len;

	/* The kiss code the ntp module must pair this with. Written out on both
	 * sides because the two modules share no header (ARCHITECTURE.md §4). */
	TEST_ASSERT_EQUAL_HEX32(0x4E54534EU, NTS_KISS_NTSN);

	make_keys(&keys);
	make_uniq(uniq, sizeof(uniq));

	/* Plain NTP passes straight through: fields unchanged, no kiss code. */
	pkt_hdr(&p);
	len = NTS_NTP_HDR_LEN;
	kod = 0U;
	TEST_ASSERT_EQUAL_INT(0, nts_ntp_ext_build(&g_nts, p.buf, p.len, rsp, &len,
						   sizeof(rsp), &kod));
	TEST_ASSERT_EQUAL_size_t(NTS_NTP_HDR_LEN, len);
	TEST_ASSERT_EQUAL_HEX32(0U, kod);

	/* A protected request gets a protected response. */
	build_request(&p, &keys, 1U);
	len = NTS_NTP_HDR_LEN;
	kod = 0U;
	memset(rsp, 0, sizeof(rsp));
	TEST_ASSERT_EQUAL_INT(0, nts_ntp_ext_build(&g_nts, p.buf, p.len, rsp, &len,
						   p.len, &kod));
	TEST_ASSERT_EQUAL_HEX32(0U, kod);
	TEST_ASSERT_EQUAL_UINT(2U, check_response(rsp, len, &keys, uniq,
						  sizeof(uniq)));

	/* An expired cookie asks for a KoD 'NTSN' carrying only the echo. */
	for (unsigned i = 0U; i <= NTS_MASTER_KEY_SLOTS; i++) {
		TEST_ASSERT_EQUAL_INT(0, nts_keyring_rotate(&g_ring,
							    2000 + 1000 * (int)i));
	}
	len = NTS_NTP_HDR_LEN;
	kod = 0U;
	TEST_ASSERT_EQUAL_INT(0, nts_ntp_ext_build(&g_nts, p.buf, p.len, rsp, &len,
						   p.len, &kod));
	TEST_ASSERT_EQUAL_HEX32((uint32_t)NTS_KISS_NTSN, kod);
	TEST_ASSERT_EQUAL_size_t(NTS_NTP_HDR_LEN + 4U + sizeof(uniq), len);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTS_EF_UNIQUE_ID,
				 bytes_get_be16(&rsp[NTS_NTP_HDR_LEN]));

	/* No room even for the echo. */
	len = NTS_NTP_HDR_LEN;
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      nts_ntp_ext_build(&g_nts, p.buf, p.len, rsp, &len,
						NTS_NTP_HDR_LEN + 8U, &kod));

	/* A forgery is dropped, not answered. */
	build_request(&p, &keys, 1U);
	p.buf[2] ^= 0x01U;
	len = NTS_NTP_HDR_LEN;
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      nts_ntp_ext_build(&g_nts, p.buf, p.len, rsp, &len,
						p.len, &kod));

	len = NTS_NTP_HDR_LEN;
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_ntp_ext_build(NULL, p.buf, p.len, rsp,
							 &len, p.len, &kod));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_ntp_ext_build(&g_nts, NULL, p.len, rsp,
							 &len, p.len, &kod));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_ntp_ext_build(&g_nts, p.buf, p.len, NULL,
							 &len, p.len, &kod));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_ntp_ext_build(&g_nts, p.buf, p.len, rsp,
							 NULL, p.len, &kod));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_ntp_ext_build(&g_nts, p.buf, p.len, rsp,
							 &len, p.len, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, nts_ntp_ext_build(&g_nts, p.buf, 8U, rsp,
							 &len, p.len, &kod));
}

/* ------------------------------------------------------------------ NTS-KE */

typedef struct {
	unsigned calls;
	unsigned fail_in;
	uint8_t last_ctx[NTSKE_EXPORTER_CONTEXT_LEN];
	char last_label[64];
} exporter_t;

static exporter_t g_exp;

/**
 * Stand-in for the TLS exporter. Deterministic and label/context sensitive, so
 * the C2S and S2C keys differ exactly when RFC 8915 §4.3 says they must.
 */
static int test_export(void *ctx, const char *label, const uint8_t *context,
		       size_t context_len, uint8_t *out, size_t out_len)
{
	exporter_t *e = (exporter_t *)ctx;
	uint8_t buf[128];
	uint8_t digest[32];
	size_t n = strlen(label);

	e->calls++;
	if (e->fail_in != 0U && e->calls >= e->fail_in) {
		return -1;
	}

	TEST_ASSERT_TRUE(n + context_len <= sizeof(buf));
	TEST_ASSERT_TRUE(context_len <= sizeof(e->last_ctx));
	TEST_ASSERT_TRUE(n < sizeof(e->last_label));
	memcpy(buf, label, n);
	memcpy(&buf[n], context, context_len);
	memcpy(e->last_label, label, n + 1U);
	memcpy(e->last_ctx, context, context_len);

	host_sha256(buf, n + context_len, digest);
	TEST_ASSERT_EQUAL_size_t(NTS_KEY_LEN, out_len);
	memcpy(out, digest, out_len);
	return 0;
}

static void ke_ctx(ntske_ctx_t *ke, uint8_t cookies, const char *name,
		   uint16_t port)
{
	ntske_cfg_t cfg;

	memset(&g_exp, 0, sizeof(g_exp));
	memset(&cfg, 0, sizeof(cfg));
	cfg.ring = &g_ring;
	cfg.export_fn = test_export;
	cfg.export_ctx = &g_exp;
	cfg.cookies = cookies;
	cfg.server_name = name;
	cfg.port = port;
	TEST_ASSERT_EQUAL_INT(0, ntske_init(ke, &cfg));
}

/** Minimal well-formed client request: next protocol NTPv4, AEAD, EOM. */
static size_t ke_request(uint8_t *buf, size_t cap, bool with_np, bool with_aead,
			 uint16_t proto, uint16_t aead, bool with_eom)
{
	size_t len = 0U;

	if (with_np) {
		TEST_ASSERT_EQUAL_INT(0, ntske_rec_put_u16(buf, cap, &len, true,
							   (uint16_t)NTSKE_REC_NEXT_PROTO,
							   proto));
	}
	if (with_aead) {
		TEST_ASSERT_EQUAL_INT(0, ntske_rec_put_u16(buf, cap, &len, false,
							   (uint16_t)NTSKE_REC_AEAD,
							   aead));
	}
	if (with_eom) {
		TEST_ASSERT_EQUAL_INT(0, ntske_rec_put(buf, cap, &len, true,
						       (uint16_t)NTSKE_REC_EOM, NULL,
						       0U));
	}
	return len;
}

static void test_ke_record_codec(void)
{
	uint8_t buf[64];
	uint8_t body[6] = { 1, 2, 3, 4, 5, 6 };
	ntske_rec_t rec;
	size_t len = 0U;
	size_t next = 0U;

	TEST_ASSERT_EQUAL_INT(0, ntske_rec_put(buf, sizeof(buf), &len, true, 0x1234U,
					       body, sizeof(body)));
	/* The critical bit is the top bit of the type word (RFC 8915 §4.1). */
	TEST_ASSERT_EQUAL_HEX16(0x9234U, bytes_get_be16(&buf[0]));
	TEST_ASSERT_EQUAL_HEX16(6U, bytes_get_be16(&buf[2]));
	TEST_ASSERT_EQUAL_size_t(10U, len);

	TEST_ASSERT_EQUAL_INT(0, ntske_rec_parse(buf, len, 0U, &rec, &next));
	TEST_ASSERT_TRUE(rec.critical);
	TEST_ASSERT_EQUAL_HEX16(0x1234U, rec.type);
	TEST_ASSERT_EQUAL_UINT16(6U, rec.body_len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(body, rec.body, sizeof(body));
	TEST_ASSERT_EQUAL_size_t(10U, next);

	/* Zero-length body, non-critical: the End of Message shape. */
	len = 0U;
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_put(buf, sizeof(buf), &len, false, 0U,
					       NULL, 0U));
	TEST_ASSERT_EQUAL_size_t(4U, len);
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_parse(buf, len, 0U, &rec, &next));
	TEST_ASSERT_FALSE(rec.critical);
	TEST_ASSERT_EQUAL_UINT16(0U, rec.body_len);

	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_rec_put(NULL, 4U, &len, false, 0U, NULL,
						     0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_rec_put(buf, 4U, NULL, false, 0U, NULL,
						     0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_rec_put(buf, 64U, &len, false, 0U, NULL,
						     4U));
	len = 0U;
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ntske_rec_put(buf, 3U, &len, false, 0U, NULL,
						     0U));
	len = 0U;
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ntske_rec_put(buf, 8U, &len, false, 0U, body,
						     sizeof(body)));
	len = 0U;
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ntske_rec_put_u16(buf, 5U, &len, false, 0U,
							 1U));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_rec_parse(NULL, 4U, 0U, &rec, &next));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_rec_parse(buf, 4U, 0U, NULL, &next));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_rec_parse(buf, 4U, 0U, &rec, NULL));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ntske_rec_parse(buf, 4U, 2U, &rec, &next));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ntske_rec_parse(buf, 4U, 8U, &rec, &next));

	/* A body length that runs past the buffer. */
	bytes_put_be16(&buf[2], 100U);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ntske_rec_parse(buf, 4U, 0U, &rec, &next));
}

static void test_ke_exporter_context(void)
{
	uint8_t c2s[NTSKE_EXPORTER_CONTEXT_LEN];
	uint8_t s2c[NTSKE_EXPORTER_CONTEXT_LEN];
	static const uint8_t want_c2s[5] = { 0x00, 0x00, 0x00, 0x0F, 0x00 };
	static const uint8_t want_s2c[5] = { 0x00, 0x00, 0x00, 0x0F, 0x01 };

	/* RFC 8915 §4.3: protocol id, AEAD id, then 0 for C2S and 1 for S2C. */
	TEST_ASSERT_EQUAL_INT(0, ntske_exporter_context((uint16_t)NTSKE_PROTO_NTPV4,
							(uint16_t)NTS_AEAD_AES_SIV_CMAC_256,
							true, c2s));
	TEST_ASSERT_EQUAL_INT(0, ntske_exporter_context((uint16_t)NTSKE_PROTO_NTPV4,
							(uint16_t)NTS_AEAD_AES_SIV_CMAC_256,
							false, s2c));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want_c2s, c2s, sizeof(want_c2s));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want_s2c, s2c, sizeof(want_s2c));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_exporter_context(0U, 0U, true, NULL));
}

/** Walk a KE response, checking order and collecting the cookies. */
static void ke_check(const uint8_t *rsp, size_t len, unsigned *cookies,
		     const nts_cookie_keys_t *expect, bool expect_server,
		     bool expect_port)
{
	size_t off = 0U;
	unsigned n = 0U;
	bool saw_np = false;
	bool saw_aead = false;
	bool saw_server = false;
	bool saw_port = false;
	bool saw_eom = false;

	while (off < len) {
		ntske_rec_t rec;
		size_t next;

		TEST_ASSERT_EQUAL_INT(0, ntske_rec_parse(rsp, len, off, &rec, &next));
		off = next;

		switch (rec.type) {
		case NTSKE_REC_NEXT_PROTO:
			saw_np = true;
			TEST_ASSERT_TRUE(rec.critical);
			TEST_ASSERT_EQUAL_UINT16(2U, rec.body_len);
			TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_PROTO_NTPV4,
						 bytes_get_be16(rec.body));
			break;
		case NTSKE_REC_AEAD:
			saw_aead = true;
			TEST_ASSERT_EQUAL_UINT16(2U, rec.body_len);
			TEST_ASSERT_EQUAL_UINT16((uint16_t)NTS_AEAD_AES_SIV_CMAC_256,
						 bytes_get_be16(rec.body));
			break;
		case NTSKE_REC_SERVER:
			saw_server = true;
			break;
		case NTSKE_REC_PORT:
			saw_port = true;
			TEST_ASSERT_EQUAL_UINT16(4460U, bytes_get_be16(rec.body));
			break;
		case NTSKE_REC_COOKIE: {
			nts_cookie_keys_t back;

			TEST_ASSERT_EQUAL_UINT16((uint16_t)NTS_COOKIE_LEN,
						 rec.body_len);
			TEST_ASSERT_EQUAL_INT(0, nts_cookie_unseal(&g_ring, rec.body,
								   rec.body_len,
								   &back));
			if (expect != NULL) {
				TEST_ASSERT_EQUAL_HEX8_ARRAY(expect->c2s, back.c2s,
							     NTS_KEY_LEN);
				TEST_ASSERT_EQUAL_HEX8_ARRAY(expect->s2c, back.s2c,
							     NTS_KEY_LEN);
			}
			n++;
			break;
		}
		case NTSKE_REC_EOM:
			saw_eom = true;
			TEST_ASSERT_TRUE(rec.critical);
			TEST_ASSERT_EQUAL_size_t(len, off);
			break;
		default:
			TEST_FAIL_MESSAGE("unexpected record in a KE response");
			break;
		}
	}

	TEST_ASSERT_TRUE(saw_np);
	TEST_ASSERT_TRUE(saw_aead);
	TEST_ASSERT_TRUE(saw_eom);
	TEST_ASSERT_EQUAL(expect_server, saw_server);
	TEST_ASSERT_EQUAL(expect_port, saw_port);
	*cookies = n;
}

static void test_ke_successful_negotiation(void)
{
	ntske_ctx_t ke;
	ntske_result_t res;
	nts_cookie_keys_t expect;
	uint8_t req[64];
	uint8_t rsp[NTSKE_RSP_RECOMMENDED];
	uint8_t ctx5[NTSKE_EXPORTER_CONTEXT_LEN];
	size_t req_len;
	size_t rsp_len = 0U;
	unsigned cookies = 0U;

	ke_ctx(&ke, 0U, "time.example.org", 4460U);
	req_len = ke_request(req, sizeof(req), true, true,
			     (uint16_t)NTSKE_PROTO_NTPV4,
			     (uint16_t)NTS_AEAD_AES_SIV_CMAC_256, true);

	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, sizeof(rsp),
					      &rsp_len, &res));
	TEST_ASSERT_TRUE(res.ok);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_NO_ERROR, res.error);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTS_AEAD_AES_SIV_CMAC_256, res.aead_id);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)NTSKE_COOKIES_DEFAULT, res.cookies);

	/* Reproduce the keys the exporter would have handed the client. */
	memset(&expect, 0, sizeof(expect));
	expect.aead_id = (uint16_t)NTS_AEAD_AES_SIV_CMAC_256;
	TEST_ASSERT_EQUAL_INT(0, ntske_exporter_context((uint16_t)NTSKE_PROTO_NTPV4,
							(uint16_t)NTS_AEAD_AES_SIV_CMAC_256,
							true, ctx5));
	TEST_ASSERT_EQUAL_INT(0, test_export(&g_exp, NTSKE_EXPORTER_LABEL, ctx5,
					     sizeof(ctx5), expect.c2s, NTS_KEY_LEN));
	TEST_ASSERT_EQUAL_INT(0, ntske_exporter_context((uint16_t)NTSKE_PROTO_NTPV4,
							(uint16_t)NTS_AEAD_AES_SIV_CMAC_256,
							false, ctx5));
	TEST_ASSERT_EQUAL_INT(0, test_export(&g_exp, NTSKE_EXPORTER_LABEL, ctx5,
					     sizeof(ctx5), expect.s2c, NTS_KEY_LEN));
	/* Two directions, two different keys — the point of the context octet. */
	TEST_ASSERT_FALSE(aes_siv_ct_memeq(expect.c2s, expect.s2c, NTS_KEY_LEN));
	TEST_ASSERT_EQUAL_STRING(NTSKE_EXPORTER_LABEL, g_exp.last_label);

	ke_check(rsp, rsp_len, &cookies, &expect, true, true);
	TEST_ASSERT_EQUAL_UINT((unsigned)NTSKE_COOKIES_DEFAULT, cookies);

	/* Without the optional records, neither appears. */
	ke_ctx(&ke, 3U, NULL, 0U);
	rsp_len = 0U;
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, sizeof(rsp),
					      &rsp_len, &res));
	TEST_ASSERT_TRUE(res.ok);
	ke_check(rsp, rsp_len, &cookies, &expect, false, false);
	TEST_ASSERT_EQUAL_UINT(3U, cookies);
}

/** Extract the error code from a two-record error response. */
static uint16_t ke_error_code(const uint8_t *rsp, size_t len)
{
	ntske_rec_t rec;
	size_t next = 0U;

	TEST_ASSERT_EQUAL_INT(0, ntske_rec_parse(rsp, len, 0U, &rec, &next));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_REC_ERROR, rec.type);
	TEST_ASSERT_TRUE(rec.critical);
	TEST_ASSERT_EQUAL_UINT16(2U, rec.body_len);
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_parse(rsp, len, next, &rec, &next));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_REC_EOM, rec.type);
	TEST_ASSERT_EQUAL_size_t(len, next);
	return bytes_get_be16(&rsp[4]);
}

static void test_ke_negotiation_errors(void)
{
	ntske_ctx_t ke;
	ntske_result_t res;
	uint8_t req[64];
	uint8_t rsp[NTSKE_RSP_RECOMMENDED];
	size_t req_len;
	size_t rsp_len = 0U;

	ke_ctx(&ke, 0U, NULL, 0U);

	/* No End of Message: the record stream is not terminated. */
	req_len = ke_request(req, sizeof(req), true, true,
			     (uint16_t)NTSKE_PROTO_NTPV4,
			     (uint16_t)NTS_AEAD_AES_SIV_CMAC_256, false);
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, sizeof(rsp),
					      &rsp_len, &res));
	TEST_ASSERT_FALSE(res.ok);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_ERR_BAD_REQUEST,
				 ke_error_code(rsp, rsp_len));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_ERR_BAD_REQUEST, res.error);

	/* No Next Protocol Negotiation record: mandatory in a request. */
	req_len = ke_request(req, sizeof(req), false, true, 0U,
			     (uint16_t)NTS_AEAD_AES_SIV_CMAC_256, true);
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, sizeof(rsp),
					      &rsp_len, &res));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_ERR_BAD_REQUEST,
				 ke_error_code(rsp, rsp_len));

	/* NTPv4 offered but no AEAD record. */
	req_len = ke_request(req, sizeof(req), true, false,
			     (uint16_t)NTSKE_PROTO_NTPV4, 0U, true);
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, sizeof(rsp),
					      &rsp_len, &res));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_ERR_BAD_REQUEST,
				 ke_error_code(rsp, rsp_len));

	/* Duplicated records. */
	req_len = 0U;
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_put_u16(req, sizeof(req), &req_len, true,
						   (uint16_t)NTSKE_REC_NEXT_PROTO, 0U));
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_put_u16(req, sizeof(req), &req_len, true,
						   (uint16_t)NTSKE_REC_NEXT_PROTO, 0U));
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, sizeof(rsp),
					      &rsp_len, &res));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_ERR_BAD_REQUEST,
				 ke_error_code(rsp, rsp_len));

	req_len = 0U;
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_put_u16(req, sizeof(req), &req_len, true,
						   (uint16_t)NTSKE_REC_NEXT_PROTO, 0U));
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_put_u16(req, sizeof(req), &req_len, false,
						   (uint16_t)NTSKE_REC_AEAD, 15U));
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_put_u16(req, sizeof(req), &req_len, false,
						   (uint16_t)NTSKE_REC_AEAD, 15U));
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, sizeof(rsp),
					      &rsp_len, &res));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_ERR_BAD_REQUEST,
				 ke_error_code(rsp, rsp_len));

	/* Odd-length protocol list: not a sequence of uint16. */
	req_len = 0U;
	{
		uint8_t odd[3] = { 0, 0, 0 };

		TEST_ASSERT_EQUAL_INT(0, ntske_rec_put(req, sizeof(req), &req_len,
						       true,
						       (uint16_t)NTSKE_REC_NEXT_PROTO,
						       odd, sizeof(odd)));
	}
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, sizeof(rsp),
					      &rsp_len, &res));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_ERR_BAD_REQUEST,
				 ke_error_code(rsp, rsp_len));

	/* A truncated record. */
	req_len = ke_request(req, sizeof(req), true, true,
			     (uint16_t)NTSKE_PROTO_NTPV4,
			     (uint16_t)NTS_AEAD_AES_SIV_CMAC_256, true);
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len - 1U, rsp,
					      sizeof(rsp), &rsp_len, &res));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_ERR_BAD_REQUEST,
				 ke_error_code(rsp, rsp_len));

	/* An unrecognised record with the critical bit set. */
	req_len = 0U;
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_put_u16(req, sizeof(req), &req_len, true,
						   (uint16_t)NTSKE_REC_NEXT_PROTO, 0U));
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_put(req, sizeof(req), &req_len, true,
					       0x4321U, NULL, 0U));
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, sizeof(rsp),
					      &rsp_len, &res));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_ERR_UNRECOGNIZED_CRITICAL,
				 ke_error_code(rsp, rsp_len));

	/* The same record without the critical bit is simply skipped, and
	 * records only a server sends are ignored on the way in. */
	req_len = 0U;
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_put(req, sizeof(req), &req_len, false,
					       0x4321U, NULL, 0U));
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_put_u16(req, sizeof(req), &req_len, false,
						   (uint16_t)NTSKE_REC_WARNING, 0U));
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_put_u16(req, sizeof(req), &req_len, true,
						   (uint16_t)NTSKE_REC_NEXT_PROTO, 0U));
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_put_u16(req, sizeof(req), &req_len, false,
						   (uint16_t)NTSKE_REC_AEAD, 15U));
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_put(req, sizeof(req), &req_len, true,
					       (uint16_t)NTSKE_REC_EOM, NULL, 0U));
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, sizeof(rsp),
					      &rsp_len, &res));
	TEST_ASSERT_TRUE(res.ok);
}

static void test_ke_declines_gracefully(void)
{
	ntske_ctx_t ke;
	ntske_result_t res;
	ntske_rec_t rec;
	uint8_t req[64];
	uint8_t rsp[NTSKE_RSP_RECOMMENDED];
	size_t req_len;
	size_t rsp_len = 0U;
	size_t next = 0U;

	ke_ctx(&ke, 0U, NULL, 0U);

	/* A protocol we do not speak: RFC 8915 §4.1.2 wants an empty Next
	 * Protocol Negotiation record, not an error. */
	req_len = ke_request(req, sizeof(req), true, true, 0x1234U,
			     (uint16_t)NTS_AEAD_AES_SIV_CMAC_256, true);
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, sizeof(rsp),
					      &rsp_len, &res));
	TEST_ASSERT_FALSE(res.ok);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_NO_ERROR, res.error);
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_parse(rsp, rsp_len, 0U, &rec, &next));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_REC_NEXT_PROTO, rec.type);
	TEST_ASSERT_EQUAL_UINT16(0U, rec.body_len);
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_parse(rsp, rsp_len, next, &rec, &next));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_REC_EOM, rec.type);
	TEST_ASSERT_EQUAL_size_t(rsp_len, next);

	/* An AEAD we do not implement: §4.1.5 wants an empty AEAD record. */
	req_len = ke_request(req, sizeof(req), true, true,
			     (uint16_t)NTSKE_PROTO_NTPV4, 30U, true);
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, sizeof(rsp),
					      &rsp_len, &res));
	TEST_ASSERT_FALSE(res.ok);
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_NO_ERROR, res.error);
	TEST_ASSERT_EQUAL_UINT8(0U, res.cookies);
	next = 0U;
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_parse(rsp, rsp_len, 0U, &rec, &next));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_REC_NEXT_PROTO, rec.type);
	TEST_ASSERT_EQUAL_UINT16(2U, rec.body_len);
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_parse(rsp, rsp_len, next, &rec, &next));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_REC_AEAD, rec.type);
	TEST_ASSERT_EQUAL_UINT16(0U, rec.body_len);
	TEST_ASSERT_EQUAL_INT(0, ntske_rec_parse(rsp, rsp_len, next, &rec, &next));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_REC_EOM, rec.type);
}

static void test_ke_internal_failures(void)
{
	ntske_ctx_t ke;
	ntske_result_t res;
	uint8_t req[64];
	uint8_t rsp[NTSKE_RSP_RECOMMENDED];
	size_t req_len;
	size_t rsp_len = 0U;
	unsigned cookies = 0U;

	ke_ctx(&ke, 0U, NULL, 0U);
	req_len = ke_request(req, sizeof(req), true, true,
			     (uint16_t)NTSKE_PROTO_NTPV4,
			     (uint16_t)NTS_AEAD_AES_SIV_CMAC_256, true);

	/* The TLS exporter failing on either direction is an internal error,
	 * and must not leave a half-built success response on the wire. */
	g_exp.fail_in = 1U;
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, sizeof(rsp),
					      &rsp_len, &res));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_ERR_INTERNAL,
				 ke_error_code(rsp, rsp_len));
	TEST_ASSERT_EQUAL_UINT8(0U, res.cookies);
	TEST_ASSERT_EQUAL_UINT16(0U, res.aead_id);

	ke_ctx(&ke, 0U, NULL, 0U);
	g_exp.fail_in = 2U;
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, sizeof(rsp),
					      &rsp_len, &res));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_ERR_INTERNAL,
				 ke_error_code(rsp, rsp_len));

	/* Entropy failure while minting a cookie. */
	ke_ctx(&ke, 0U, NULL, 0U);
	g_hc.fail_rand_in = 1U;
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, sizeof(rsp),
					      &rsp_len, &res));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_ERR_INTERNAL,
				 ke_error_code(rsp, rsp_len));

	/* Room for some cookies but not all: fewer is a valid answer. */
	ke_ctx(&ke, 0U, NULL, 0U);
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp,
					      4U + 2U + 4U + 2U +
						      3U * (4U + NTS_COOKIE_LEN) + 4U,
					      &rsp_len, &res));
	TEST_ASSERT_TRUE(res.ok);
	TEST_ASSERT_EQUAL_UINT8(3U, res.cookies);
	ke_check(rsp, rsp_len, &cookies, NULL, false, false);
	TEST_ASSERT_EQUAL_UINT(3U, cookies);

	/* Room for none is not: a client with no cookies cannot proceed. */
	ke_ctx(&ke, 0U, NULL, 0U);
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, 32U, &rsp_len,
					      &res));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_ERR_INTERNAL,
				 ke_error_code(rsp, rsp_len));

	/* Not even room for an error response. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ntske_handle(&ke, req, req_len, rsp, 9U,
						    &rsp_len, &res));

	/* A long server name that displaces the cookies. */
	{
		static const char long_name[] =
			"a-very-long-ntp-server-name-that-eats-the-response-buffer"
			".example.invalid";

		ke_ctx(&ke, 0U, long_name, 4460U);
		TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, req, req_len, rsp, 96U,
						      &rsp_len, &res));
		TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_ERR_INTERNAL,
					 ke_error_code(rsp, rsp_len));
	}
}

static void test_ke_bad_arguments(void)
{
	ntske_ctx_t ke;
	ntske_cfg_t cfg;
	ntske_result_t res;
	nts_keyring_t blank;
	uint8_t rsp[64];
	size_t rsp_len = 0U;

	memset(&blank, 0, sizeof(blank));
	memset(&cfg, 0, sizeof(cfg));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_init(NULL, &cfg));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_init(&ke, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_init(&ke, &cfg)); /* no ring */
	cfg.ring = &blank;
	cfg.export_fn = test_export;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_init(&ke, &cfg)); /* ring not ready */
	cfg.ring = &g_ring;
	cfg.export_fn = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_init(&ke, &cfg)); /* no exporter */

	/* An over-large cookie request is clamped rather than refused. */
	cfg.export_fn = test_export;
	cfg.export_ctx = &g_exp;
	cfg.cookies = 200U;
	memset(&g_exp, 0, sizeof(g_exp));
	TEST_ASSERT_EQUAL_INT(0, ntske_init(&ke, &cfg));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_handle(NULL, NULL, 0U, rsp, sizeof(rsp),
						    &rsp_len, &res));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_handle(&ke, NULL, 4U, rsp, sizeof(rsp),
						    &rsp_len, &res));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_handle(&ke, NULL, 0U, NULL,
						    sizeof(rsp), &rsp_len, &res));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_handle(&ke, NULL, 0U, rsp, sizeof(rsp),
						    NULL, &res));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_handle(&ke, NULL, 0U, rsp, sizeof(rsp),
						    &rsp_len, NULL));

	/* An empty request is a request with no End of Message. */
	TEST_ASSERT_EQUAL_INT(0, ntske_handle(&ke, NULL, 0U, rsp, sizeof(rsp),
					      &rsp_len, &res));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)NTSKE_ERR_BAD_REQUEST, res.error);

	{
		ntske_ctx_t blank_ke;

		memset(&blank_ke, 0, sizeof(blank_ke));
		TEST_ASSERT_EQUAL_INT(-EINVAL, ntske_handle(&blank_ke, NULL, 0U, rsp,
							    sizeof(rsp), &rsp_len,
							    &res));
	}
}

static void test_keyring_id_space_wraps_past_zero(void)
{
	nts_keyring_t r;
	nts_cookie_keys_t keys;
	uint8_t key[NTS_MASTER_KEY_LEN];
	uint8_t cookie[NTS_COOKIE_LEN];

	make_keys(&keys);
	memset(key, 0x11U, sizeof(key));
	TEST_ASSERT_EQUAL_INT(0, nts_keyring_init(&r, &g_port, 0, 0));

	/* Restore a key near the top of the id space, as a long-lived unit
	 * would after many rotations. */
	TEST_ASSERT_EQUAL_INT(0, nts_keyring_install(&r, 0xFFFEU, key, 0, false));
	TEST_ASSERT_EQUAL_INT(0, nts_keyring_rotate(&r, 1));
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_seal(&r, &keys, cookie));
	TEST_ASSERT_EQUAL_UINT16(0xFFFFU, bytes_get_be16(cookie));

	/* The next id must skip 0: a cookie naming key 0 would match every
	 * empty slot in the ring. */
	TEST_ASSERT_EQUAL_INT(0, nts_keyring_rotate(&r, 2));
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_seal(&r, &keys, cookie));
	TEST_ASSERT_EQUAL_UINT16(1U, bytes_get_be16(cookie));

	/* Installing the very last id must not roll the counter to 0 either. */
	TEST_ASSERT_EQUAL_INT(0, nts_keyring_install(&r, 0xFFFFU, key, 0, false));
	TEST_ASSERT_EQUAL_INT(0, nts_keyring_rotate(&r, 3));
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_seal(&r, &keys, cookie));
	TEST_ASSERT_NOT_EQUAL(0U, bytes_get_be16(cookie));
}

static void test_remaining_rejection_and_clamp_paths(void)
{
	nts_cookie_keys_t keys;
	nts_req_t req;
	pkt_t p;
	uint8_t rsp[PKT_CAP];
	uint8_t uniq[NTS_UNIQ_MIN];
	uint8_t cookie[NTS_COOKIE_LEN];
	const size_t empty_rsp = NTS_NTP_HDR_LEN + 4U + NTS_UNIQ_MIN + 40U;
	size_t auth_off;
	size_t len;

	make_keys(&keys);
	make_uniq(uniq, sizeof(uniq));

	/* The crypto port failing mid-unseal is the server's problem, not the
	 * client's: a NAK would send a healthy client off to re-key for nothing
	 * and would hide the fault behind a protocol message. */
	build_request(&p, &keys, 0U);
	g_hc.fail_aes_in = 1U;
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* An authenticator with no body at all — too short even for the two
	 * length fields it must begin with. */
	build_request(&p, &keys, 0U);
	auth_off = p.len - 40U;
	bytes_put_be16(&p.buf[auth_off + 2U], 4U);
	p.len = auth_off + 4U;
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* Two stray octets where the next field header should begin. An NTS
	 * packet's extension fields tile the datagram exactly, so a tail too
	 * short to be a field is malformed rather than padding. */
	TEST_ASSERT_EQUAL_INT(0, nts_cookie_seal(&g_ring, &keys, cookie));
	pkt_hdr(&p);
	pkt_ef(&p, (uint16_t)NTS_EF_UNIQUE_ID, uniq, sizeof(uniq));
	pkt_ef(&p, (uint16_t)NTS_EF_COOKIE, cookie, sizeof(cookie));
	p.buf[p.len] = 0U;
	p.buf[p.len + 1U] = 0U;
	p.len += 2U;
	TEST_ASSERT_EQUAL_INT(NTS_ACT_DROP,
			      nts_process_request(&g_nts, p.buf, p.len, &req));

	/* A server configured to issue fewer cookies than the client asks for,
	 * with room to spare: the server's limit is what binds. */
	TEST_ASSERT_EQUAL_INT(0, nts_init(&g_nts, &g_ring, 2U));
	build_request(&p, &keys, 5U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_OK,
			      nts_process_request(&g_nts, p.buf, p.len, &req));
	len = NTS_NTP_HDR_LEN;
	memset(rsp, 0, sizeof(rsp));
	TEST_ASSERT_EQUAL_INT(0, nts_append_response(&g_nts, &req, rsp, &len,
						     sizeof(rsp)));
	TEST_ASSERT_EQUAL_UINT(2U, check_response(rsp, len, &keys, uniq,
						  sizeof(uniq)));

	/* The AEAD failing while sealing the response itself. */
	TEST_ASSERT_EQUAL_INT(0, nts_init(&g_nts, &g_ring, 0U));
	build_request(&p, &keys, 0U);
	TEST_ASSERT_EQUAL_INT(NTS_ACT_OK,
			      nts_process_request(&g_nts, p.buf, p.len, &req));
	len = NTS_NTP_HDR_LEN;
	g_hc.fail_aes_in = 1U;
	TEST_ASSERT_EQUAL_INT(-EIO, nts_append_response(&g_nts, &req, rsp, &len,
							empty_rsp));
}

int main(void)
{
	UNITY_BEGIN();
	RUN_TEST(test_keyring_init_and_export);
	RUN_TEST(test_keyring_init_rejects_bad_ports);
	RUN_TEST(test_keyring_rotation_window);
	RUN_TEST(test_keyring_tick_and_failures);
	RUN_TEST(test_keyring_install_restores_across_reboot);
	RUN_TEST(test_keyring_id_space_wraps_past_zero);
	RUN_TEST(test_cookie_layout_and_roundtrip);
	RUN_TEST(test_cookie_rejects_mutation);
	RUN_TEST(test_cookie_bad_arguments_and_port_failures);
	RUN_TEST(test_process_plain_ntp_is_not_ours);
	RUN_TEST(test_process_accepts_a_valid_request);
	RUN_TEST(test_process_clamps_cookie_demand);
	RUN_TEST(test_process_rejects_structural_faults);
	RUN_TEST(test_process_naks_an_unusable_cookie);
	RUN_TEST(test_process_drops_forgeries);
	RUN_TEST(test_process_rejects_oversized_encrypted_payload);
	RUN_TEST(test_process_ignores_fields_after_the_authenticator);
	RUN_TEST(test_response_round_trip);
	RUN_TEST(test_response_never_exceeds_request);
	RUN_TEST(test_response_trims_to_capacity);
	RUN_TEST(test_response_bad_arguments_and_port_failures);
	RUN_TEST(test_remaining_rejection_and_clamp_paths);
	RUN_TEST(test_ext_hook);
	RUN_TEST(test_ke_record_codec);
	RUN_TEST(test_ke_exporter_context);
	RUN_TEST(test_ke_successful_negotiation);
	RUN_TEST(test_ke_negotiation_errors);
	RUN_TEST(test_ke_declines_gracefully);
	RUN_TEST(test_ke_internal_failures);
	RUN_TEST(test_ke_bad_arguments);
	return UNITY_END();
}
