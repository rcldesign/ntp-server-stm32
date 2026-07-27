/*
 * STS1000 "Meridian" — core/nts: Network Time Security (RFC 8915).
 *
 * See nts.h for the contract, the cookie layout and the reason this module
 * does not include ntp.h.
 *
 * Two AEAD invocations matter and they are easy to confuse, so, explicitly:
 *
 *   cookie      key    = a master key from the ring
 *               AD     = the 4-octet cleartext cookie header, then the nonce
 *               plain  = C2S ‖ S2C
 *
 *   response    key    = the S2C key that cookie carried
 *               AD     = the response packet up to the authenticator field,
 *                        then the nonce
 *               plain  = the fresh NTS Cookie extension fields
 *
 * In both cases the nonce is the *last* associated-data component rather than a
 * separate argument: that is how RFC 5297 §3 defines nonce-based SIV, and it is
 * what RFC 8915 §5.6 means by "the nonce used".
 *
 * The response is assembled in place. Cookie fields are written straight into
 * the packet where their ciphertext will live and then encrypted where they
 * lie, which keeps the largest temporary at one 100-octet cookie instead of the
 * 832 octets eight of them would need on the stack.
 */

#include "nts/nts.h"

#include <errno.h>
#include <string.h>

#include "util/bytes.h"

/* Length prefix inside the authenticator body: nonce length + ciphertext
 * length, two big-endian uint16 (RFC 8915 §5.6). */
#define AUTH_LEN_FIELDS 4U

/* Authenticator field with an empty plaintext: header + length fields + nonce
 * + SIV. Every response has at least this much. */
#define AUTH_FIXED_LEN (4U + AUTH_LEN_FIELDS + NTS_NONCE_LEN + AES_SIV_TAG_LEN)

/* Cookie plaintext: the two AEAD keys, concatenated. */
#define COOKIE_PT_LEN (2U * NTS_KEY_LEN)

/* Offsets inside a sealed cookie (see the map in nts.h). */
#define COOKIE_OFF_KEYID 0U
#define COOKIE_OFF_AEAD 2U
#define COOKIE_OFF_NONCE 4U
#define COOKIE_OFF_SEALED 20U

_Static_assert(NTS_COOKIE_LEN ==
		       COOKIE_OFF_SEALED + AES_SIV_TAG_LEN + COOKIE_PT_LEN,
	       "cookie layout and NTS_COOKIE_LEN disagree");
_Static_assert((NTS_COOKIE_EF_LEN & 3U) == 0U,
	       "a cookie extension field must be a multiple of 4 octets");
_Static_assert(NTS_MASTER_KEY_SLOTS >= 2U,
	       "graceful rotation needs at least the current and previous key");
_Static_assert(NTS_COOKIES_MAX >= 1U, "a response must be able to carry a cookie");

static size_t pad4(size_t n)
{
	return (n + 3U) & ~(size_t)3U;
}

static void put_ef_header(uint8_t *p, uint16_t type, size_t field_len)
{
	bytes_put_be16(p, type);
	bytes_put_be16(&p[2], (uint16_t)field_len);
}

/* ------------------------------------------------------------- key ring */

static nts_master_key_t *ring_find(nts_keyring_t *r, uint16_t id)
{
	for (size_t i = 0U; i < NTS_MASTER_KEY_SLOTS; i++) {
		if (r->keys[i].valid && r->keys[i].id == id) {
			return &r->keys[i];
		}
	}
	return NULL;
}

/** Next key identifier, skipping 0 — 0 marks an empty slot. */
static uint16_t ring_next_id(nts_keyring_t *r)
{
	uint16_t id = r->next_id;

	r->next_id = (uint16_t)(r->next_id + 1U);
	if (r->next_id == 0U) {
		r->next_id = 1U;
	}
	return id;
}

/**
 * Empty slot if there is one, otherwise the least recently created — but never
 * the current key (L7). Evicting the current key to make room for an installed
 * one would leave newly minted cookies sealed under a key no longer in the
 * ring the instant the next install lands. With ≥ 2 slots (asserted below) and
 * every slot valid, skipping the current one still leaves a candidate.
 */
static size_t ring_victim(const nts_keyring_t *r)
{
	size_t victim = NTS_MASTER_KEY_SLOTS; /* sentinel: none chosen yet */

	for (size_t i = 0U; i < NTS_MASTER_KEY_SLOTS; i++) {
		if (!r->keys[i].valid) {
			return i;
		}
	}
	for (size_t i = 0U; i < NTS_MASTER_KEY_SLOTS; i++) {
		if (i == r->current) {
			continue;
		}
		if (victim == NTS_MASTER_KEY_SLOTS ||
		    r->keys[i].created_ms < r->keys[victim].created_ms) {
			victim = i;
		}
	}
	return (victim == NTS_MASTER_KEY_SLOTS) ? r->current : victim;
}

static int ring_mint(nts_keyring_t *r, int64_t now_ms)
{
	size_t slot = ring_victim(r);
	uint8_t key[NTS_MASTER_KEY_LEN];

	if (r->crypto.rand(r->crypto.ctx, key, sizeof(key)) != 0) {
		return -EIO;
	}

	memcpy(r->keys[slot].key, key, sizeof(key));
	r->keys[slot].created_ms = now_ms;
	r->keys[slot].id = ring_next_id(r);
	r->keys[slot].valid = true;
	r->current = (uint8_t)slot;
	r->last_rotate_ms = now_ms;
	return 0;
}

int nts_keyring_init(nts_keyring_t *r, const port_crypto_t *crypto,
		     int64_t now_ms, int64_t rotate_ms)
{
	if (r == NULL || crypto == NULL || crypto->rand == NULL ||
	    crypto->aes_ecb_encrypt == NULL) {
		return -EINVAL;
	}

	memset(r, 0, sizeof(*r));
	r->crypto = *crypto;
	r->rotate_ms = (rotate_ms > 0) ? rotate_ms : NTS_ROTATE_DEFAULT_MS;
	r->next_id = 1U;

	if (ring_mint(r, now_ms) != 0) {
		memset(r, 0, sizeof(*r));
		return -EIO;
	}
	r->ready = true;
	return 0;
}

int nts_keyring_rotate(nts_keyring_t *r, int64_t now_ms)
{
	if (r == NULL || !r->ready) {
		return -EINVAL;
	}
	return ring_mint(r, now_ms);
}

int nts_keyring_tick(nts_keyring_t *r, int64_t now_ms)
{
	int rc;

	if (r == NULL || !r->ready) {
		return -EINVAL;
	}
	if ((now_ms - r->last_rotate_ms) < r->rotate_ms) {
		return 0;
	}
	rc = ring_mint(r, now_ms);
	return (rc != 0) ? rc : 1;
}

int nts_keyring_install(nts_keyring_t *r, uint16_t id,
			const uint8_t key[NTS_MASTER_KEY_LEN], int64_t created_ms,
			bool make_current)
{
	nts_master_key_t *slot;

	if (r == NULL || !r->ready || key == NULL || id == 0U) {
		return -EINVAL;
	}

	slot = ring_find(r, id);
	if (slot == NULL) {
		slot = &r->keys[ring_victim(r)];
	}

	memcpy(slot->key, key, NTS_MASTER_KEY_LEN);
	slot->created_ms = created_ms;
	slot->id = id;
	slot->valid = true;

	if (make_current) {
		r->current = (uint8_t)(slot - &r->keys[0]);
		r->last_rotate_ms = created_ms;
	}
	/* Keep minting ids past the highest restored one so a reboot cannot
	 * re-issue an id that is still live in a client's cookie. */
	if ((uint16_t)(id + 1U) != 0U && id >= r->next_id) {
		r->next_id = (uint16_t)(id + 1U);
	}
	return 0;
}

int nts_keyring_export(const nts_keyring_t *r, size_t slot, uint16_t *id,
		       uint8_t key[NTS_MASTER_KEY_LEN], int64_t *created_ms,
		       bool *is_current)
{
	if (r == NULL || !r->ready || slot >= NTS_MASTER_KEY_SLOTS) {
		return -EINVAL;
	}
	if (!r->keys[slot].valid) {
		return -ENOENT;
	}
	if (id != NULL) {
		*id = r->keys[slot].id;
	}
	if (key != NULL) {
		memcpy(key, r->keys[slot].key, NTS_MASTER_KEY_LEN);
	}
	if (created_ms != NULL) {
		*created_ms = r->keys[slot].created_ms;
	}
	if (is_current != NULL) {
		*is_current = (slot == r->current);
	}
	return 0;
}

/* -------------------------------------------------------------- cookies */

int nts_cookie_seal(nts_keyring_t *r, const nts_cookie_keys_t *k,
		    uint8_t out[NTS_COOKIE_LEN])
{
	uint16_t mk_id;
	uint8_t mk_key[NTS_MASTER_KEY_LEN];
	aes_siv_ctx_t siv;
	aes_siv_ad_t ad[2];
	uint8_t pt[COOKIE_PT_LEN];
	size_t n = 0U;
	int rc;

	if (r == NULL || !r->ready || k == NULL || out == NULL) {
		return -EINVAL;
	}
	if (k->aead_id != NTS_AEAD_AES_SIV_CMAC_256) {
		return -EINVAL;
	}

	/*
	 * Snapshot the current master key's id and material together, up front
	 * (M3). Rotation runs on the housekeeping thread and the datapath here on
	 * the ntp_server thread; the threading contract (nts.h) is that the glue
	 * serialises them, but taking a local copy of {id, key} makes a torn
	 * rotation impossible even if that contract is ever violated — the worst
	 * case degrades from "seal with id A but key B, an unsealable cookie" to
	 * at most sealing under the key that was current at this instant.
	 */
	{
		uint8_t cur = r->current;

		mk_id = r->keys[cur].id;
		memcpy(mk_key, r->keys[cur].key, sizeof(mk_key));
	}

	bytes_put_be16(&out[COOKIE_OFF_KEYID], mk_id);
	bytes_put_be16(&out[COOKIE_OFF_AEAD], k->aead_id);
	if (r->crypto.rand(r->crypto.ctx, &out[COOKIE_OFF_NONCE],
			   NTS_NONCE_LEN) != 0) {
		memset(mk_key, 0, sizeof(mk_key));
		return -EIO;
	}

	memcpy(pt, k->c2s, NTS_KEY_LEN);
	memcpy(&pt[NTS_KEY_LEN], k->s2c, NTS_KEY_LEN);

	rc = aes_siv_init(&siv, &r->crypto, mk_key, NTS_MASTER_KEY_LEN);
	memset(mk_key, 0, sizeof(mk_key));
	if (rc != 0) {
		memset(pt, 0, sizeof(pt));
		return rc;
	}
	ad[0].p = &out[COOKIE_OFF_KEYID];
	ad[0].len = COOKIE_OFF_NONCE;
	ad[1].p = &out[COOKIE_OFF_NONCE];
	ad[1].len = NTS_NONCE_LEN;

	rc = aes_siv_encrypt(&siv, ad, 2U, pt, sizeof(pt), &out[COOKIE_OFF_SEALED],
			     NTS_COOKIE_LEN - COOKIE_OFF_SEALED, &n);
	memset(pt, 0, sizeof(pt));
	return rc;
}

int nts_cookie_unseal(const nts_keyring_t *r, const uint8_t *cookie, size_t len,
		      nts_cookie_keys_t *k)
{
	const nts_master_key_t *mk = NULL;
	aes_siv_ctx_t siv;
	aes_siv_ad_t ad[2];
	uint8_t pt[COOKIE_PT_LEN];
	uint16_t id;
	uint16_t aead;
	size_t n = 0U;
	int rc;

	if (r == NULL || !r->ready || cookie == NULL || k == NULL) {
		return -EINVAL;
	}
	if (len != NTS_COOKIE_LEN) {
		return -EBADMSG;
	}

	id = bytes_get_be16(&cookie[COOKIE_OFF_KEYID]);
	aead = bytes_get_be16(&cookie[COOKIE_OFF_AEAD]);

	for (size_t i = 0U; i < NTS_MASTER_KEY_SLOTS; i++) {
		if (r->keys[i].valid && r->keys[i].id == id) {
			mk = &r->keys[i];
			break;
		}
	}
	if (mk == NULL) {
		/* The ordinary end of a cookie's life, not an attack. */
		return -ENOENT;
	}
	if (aead != NTS_AEAD_AES_SIV_CMAC_256) {
		return -ENOTSUP;
	}

	rc = aes_siv_init(&siv, &r->crypto, mk->key, NTS_MASTER_KEY_LEN);
	if (rc != 0) {
		return rc;
	}
	ad[0].p = &cookie[COOKIE_OFF_KEYID];
	ad[0].len = COOKIE_OFF_NONCE;
	ad[1].p = &cookie[COOKIE_OFF_NONCE];
	ad[1].len = NTS_NONCE_LEN;

	rc = aes_siv_decrypt(&siv, ad, 2U, &cookie[COOKIE_OFF_SEALED],
			     NTS_COOKIE_LEN - COOKIE_OFF_SEALED, pt, sizeof(pt), &n);
	if (rc != 0) {
		memset(pt, 0, sizeof(pt));
		return rc;
	}

	k->aead_id = aead;
	memcpy(k->c2s, pt, NTS_KEY_LEN);
	memcpy(k->s2c, &pt[NTS_KEY_LEN], NTS_KEY_LEN);
	memset(pt, 0, sizeof(pt));
	return 0;
}

/* ------------------------------------------------- extension field walker */

typedef struct {
	const uint8_t *pkt;
	size_t len;
	size_t off;
} ef_iter_t;

typedef struct {
	uint16_t type;
	size_t field_off;
	const uint8_t *body;
	size_t body_len;
} ef_t;

/**
 * @retval 0         @p ef holds the next field.
 * @retval -ENOENT   Clean end of the packet, or the remainder is a legacy MAC
 *                   field (RFC 7822 §7.5.1: a 4/20/24-octet tail is a MAC, not
 *                   an extension field). Applying the same disambiguation ntp
 *                   uses (L13) stops a plain NTP+MAC packet — or an NTS packet
 *                   carrying a trailing MAC — from being mis-parsed as a broken
 *                   extension field and dropped before the datapath can class
 *                   it; the caller sees only the fields before the MAC.
 * @retval -EBADMSG  A short, unaligned or overrunning field. An NTS packet's
 *                   extension fields tile the datagram exactly up to any MAC,
 *                   so anything else is malformed.
 */
static int ef_next(ef_iter_t *it, ef_t *ef)
{
	size_t rem;
	uint16_t flen;

	if (it->off >= it->len) {
		return -ENOENT;
	}
	rem = it->len - it->off;
	if (rem == 4U || rem == 20U || rem == 24U) {
		return -ENOENT; /* RFC 7822 §7.5.1 MAC field: end of extensions */
	}
	if (rem < 4U) {
		return -EBADMSG;
	}
	flen = bytes_get_be16(&it->pkt[it->off + 2U]);
	if (flen < 4U || (flen & 3U) != 0U || (size_t)flen > rem) {
		return -EBADMSG;
	}

	ef->type = bytes_get_be16(&it->pkt[it->off]);
	ef->field_off = it->off;
	ef->body = &it->pkt[it->off + 4U];
	ef->body_len = (size_t)flen - 4U;
	it->off += flen;
	return 0;
}

/* ---------------------------------------------------------- the datapath */

int nts_init(nts_ctx_t *ctx, nts_keyring_t *ring, uint8_t max_cookies)
{
	if (ctx == NULL || ring == NULL || !ring->ready) {
		return -EINVAL;
	}
	memset(ctx, 0, sizeof(*ctx));
	ctx->ring = ring;
	if (max_cookies == 0U || max_cookies > NTS_COOKIES_MAX) {
		max_cookies = (uint8_t)NTS_COOKIES_MAX;
	}
	ctx->max_cookies = max_cookies;
	ctx->ready = true;
	return 0;
}

int nts_stats_get(const nts_ctx_t *ctx, nts_stats_t *out)
{
	if (ctx == NULL || out == NULL) {
		return -EINVAL;
	}
	*out = ctx->stats;
	return 0;
}

int nts_process_request(nts_ctx_t *ctx, const uint8_t *pkt, size_t len,
			nts_req_t *out)
{
	ef_iter_t it;
	ef_t ef;
	const uint8_t *uniq = NULL;
	const uint8_t *cookie = NULL;
	const uint8_t *auth = NULL;
	const uint8_t *nonce;
	const uint8_t *ct;
	size_t uniq_len = 0U;
	size_t cookie_len = 0U;
	size_t auth_len = 0U;
	size_t auth_off = 0U;
	size_t nonce_len;
	size_t ct_len;
	unsigned n_uniq = 0U;
	unsigned n_cookie = 0U;
	unsigned n_ph = 0U;
	unsigned n_auth = 0U;
	bool malformed = false;
	aes_siv_ctx_t siv;
	aes_siv_ad_t ad[2];
	uint8_t scratch[NTS_REQ_PLAINTEXT_MAX];
	size_t pt_len = 0U;
	int rc;

	if (ctx == NULL || !ctx->ready || pkt == NULL || out == NULL) {
		return -EINVAL;
	}
	if (len < NTS_NTP_HDR_LEN) {
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));

	it.pkt = pkt;
	it.len = len;
	it.off = NTS_NTP_HDR_LEN;

	for (;;) {
		rc = ef_next(&it, &ef);
		if (rc == -ENOENT) {
			break;
		}
		if (rc != 0) {
			malformed = true;
			break;
		}

		switch (ef.type) {
		case NTS_EF_UNIQUE_ID:
			n_uniq++;
			uniq = ef.body;
			uniq_len = ef.body_len;
			break;
		case NTS_EF_COOKIE:
			n_cookie++;
			cookie = ef.body;
			cookie_len = ef.body_len;
			break;
		case NTS_EF_COOKIE_PLACEHOLDER:
			/* RFC 8915 §5.5 MUST: a placeholder's body length equals
			 * the cookie length it reserves room for. A mismatch is a
			 * malformed request (L9). */
			if (ef.body_len != NTS_COOKIE_LEN) {
				malformed = true;
			}
			n_ph++;
			break;
		case NTS_EF_AUTH:
			n_auth++;
			auth = ef.body;
			auth_len = ef.body_len;
			auth_off = ef.field_off;
			break;
		default:
			break; /* not ours; some other extension */
		}

		if (ef.type == NTS_EF_AUTH) {
			/* Everything past the authenticator is outside the
			 * authenticated region — ignore it rather than trust it. */
			break;
		}
	}

	if (n_uniq == 0U && n_cookie == 0U && n_ph == 0U && n_auth == 0U) {
		return NTS_ACT_NONE; /* plain NTP; not our packet */
	}

	ctx->stats.rx++;

	if (malformed || n_uniq != 1U || n_cookie != 1U || n_auth != 1U) {
		ctx->stats.dropped++;
		return NTS_ACT_DROP;
	}
	/* RFC 8915 §5.3 makes 32 octets a MUST on the sender. Enforcing it costs
	 * no interoperability with a conforming client and keeps the echo
	 * bounded. */
	if (uniq_len < NTS_UNIQ_MIN || uniq_len > NTS_UNIQ_MAX) {
		ctx->stats.dropped++;
		return NTS_ACT_DROP;
	}

	memcpy(out->uniq, uniq, uniq_len);
	out->uniq_len = uniq_len;
	out->has_uniq = true;

	/* A cookie we cannot use — wrong size, retired key, wrong AEAD, bad tag
	 * — is what NTS NAK exists to report. */
	rc = nts_cookie_unseal(ctx->ring, cookie, cookie_len, &out->keys);
	if (rc == -EBADMSG || rc == -ENOENT || rc == -ENOTSUP) {
		ctx->stats.nak++;
		return NTS_ACT_NAK;
	}
	if (rc != 0) {
		ctx->stats.dropped++;
		return NTS_ACT_DROP;
	}

	/* Authenticator body: nonce length, ciphertext length, then both padded
	 * to a multiple of 4 (RFC 8915 §5.6). */
	if (auth_len < AUTH_LEN_FIELDS) {
		ctx->stats.dropped++;
		return NTS_ACT_DROP;
	}
	nonce_len = bytes_get_be16(auth);
	ct_len = bytes_get_be16(&auth[2]);
	/* RFC 8915 §5.6 requires a client-generated nonce; bound it so a
	 * degenerate or absurd length is refused up front (L10). The AES-SIV-
	 * CMAC-256 nonce is 16 octets in practice, but a small range keeps
	 * interop room without letting the length run wild. */
	if (nonce_len == 0U || nonce_len > NTS_REQ_NONCE_MAX) {
		ctx->stats.dropped++;
		return NTS_ACT_DROP;
	}
	if (AUTH_LEN_FIELDS + pad4(nonce_len) + pad4(ct_len) > auth_len) {
		ctx->stats.dropped++;
		return NTS_ACT_DROP;
	}
	if (ct_len < AES_SIV_TAG_LEN ||
	    (ct_len - AES_SIV_TAG_LEN) > sizeof(scratch)) {
		ctx->stats.dropped++;
		return NTS_ACT_DROP;
	}
	nonce = &auth[AUTH_LEN_FIELDS];
	ct = &auth[AUTH_LEN_FIELDS + pad4(nonce_len)];

	rc = aes_siv_init(&siv, &ctx->ring->crypto, out->keys.c2s, NTS_KEY_LEN);
	if (rc != 0) {
		ctx->stats.dropped++;
		return NTS_ACT_DROP;
	}
	ad[0].p = pkt;
	ad[0].len = auth_off; /* header + every field before the authenticator */
	ad[1].p = nonce;
	ad[1].len = nonce_len;

	rc = aes_siv_decrypt(&siv, ad, 2U, ct, ct_len, scratch, sizeof(scratch),
			     &pt_len);
	memset(scratch, 0, sizeof(scratch));
	if (rc != 0) {
		/*
		 * The cookie was genuine but the packet is not: a forgery, or a
		 * modified relay. Answering — even with a NAK — would tell the
		 * forger the cookie is live, so it gets nothing.
		 */
		ctx->stats.dropped++;
		return NTS_ACT_DROP;
	}

	out->has_keys = true;
	out->cookies_wanted = (uint8_t)((n_ph + 1U > NTS_COOKIES_MAX)
						? NTS_COOKIES_MAX
						: n_ph + 1U);
	ctx->stats.ok++;
	return NTS_ACT_OK;
}

int nts_append_nak(nts_ctx_t *ctx, const nts_req_t *req, uint8_t *pkt,
		   size_t *len, size_t cap)
{
	size_t base;
	size_t field;

	if (ctx == NULL || !ctx->ready || req == NULL || pkt == NULL ||
	    len == NULL || !req->has_uniq) {
		return -EINVAL;
	}
	base = *len;
	field = 4U + pad4(req->uniq_len);
	if (base > cap || (cap - base) < field) {
		return -ENOSPC;
	}

	put_ef_header(&pkt[base], (uint16_t)NTS_EF_UNIQUE_ID, field);
	memcpy(&pkt[base + 4U], req->uniq, req->uniq_len);
	memset(&pkt[base + 4U + req->uniq_len], 0, field - 4U - req->uniq_len);
	*len = base + field;
	return 0;
}

int nts_append_response(nts_ctx_t *ctx, const nts_req_t *req, uint8_t *pkt,
			size_t *len, size_t cap)
{
	aes_siv_ctx_t siv;
	aes_siv_ad_t ad[2];
	size_t after;
	size_t room;
	size_t n;
	size_t ct_off;
	size_t plain_len;
	size_t field_len;
	size_t out_len = 0U;
	int rc;

	if (ctx == NULL || !ctx->ready || req == NULL || pkt == NULL ||
	    len == NULL || !req->has_uniq || !req->has_keys) {
		return -EINVAL;
	}

	/* The echo goes first: RFC 8915 §5.7 has it authenticated but not
	 * encrypted, so it belongs in the associated data the authenticator
	 * covers. It is the same field a NAK carries alone, so the NAK builder
	 * is the one place that lays it out. */
	rc = nts_append_nak(ctx, req, pkt, len, cap);
	if (rc != 0) {
		return rc;
	}
	after = *len;

	/*
	 * There must be room for the authenticator envelope AND at least one
	 * cookie (L12). A response carrying zero fresh cookies would leave the
	 * client unable to make its next request — it spends a cookie per
	 * exchange — so that is -ENOSPC, not a valid response. The anti-
	 * amplification budget always allows it: a request is at least header +
	 * unique-id + one cookie + authenticator, and the matching response is
	 * header + unique-id + authenticator + one cookie, the same size.
	 */
	if ((cap - after) < AUTH_FIXED_LEN + NTS_COOKIE_EF_LEN) {
		return -ENOSPC;
	}
	room = cap - after - AUTH_FIXED_LEN;

	n = room / NTS_COOKIE_EF_LEN;
	if (n > req->cookies_wanted) {
		n = req->cookies_wanted;
	}
	if (n > ctx->max_cookies) {
		n = ctx->max_cookies;
	}

	/* Stage the cookie fields where their ciphertext will end up, so the
	 * encryption is in place and needs no second buffer. */
	ct_off = after + AUTH_FIXED_LEN;
	for (size_t i = 0U; i < n; i++) {
		size_t off = ct_off + i * NTS_COOKIE_EF_LEN;

		put_ef_header(&pkt[off], (uint16_t)NTS_EF_COOKIE,
			      NTS_COOKIE_EF_LEN);
		rc = nts_cookie_seal(ctx->ring, &req->keys, &pkt[off + 4U]);
		if (rc != 0) {
			return rc;
		}
	}
	plain_len = n * NTS_COOKIE_EF_LEN;

	if (ctx->ring->crypto.rand(ctx->ring->crypto.ctx,
				   &pkt[after + 4U + AUTH_LEN_FIELDS],
				   NTS_NONCE_LEN) != 0) {
		return -EIO;
	}

	field_len = AUTH_FIXED_LEN + plain_len;
	put_ef_header(&pkt[after], (uint16_t)NTS_EF_AUTH, field_len);
	bytes_put_be16(&pkt[after + 4U], (uint16_t)NTS_NONCE_LEN);
	bytes_put_be16(&pkt[after + 6U], (uint16_t)(AES_SIV_TAG_LEN + plain_len));

	rc = aes_siv_init(&siv, &ctx->ring->crypto, req->keys.s2c, NTS_KEY_LEN);
	if (rc != 0) {
		return rc;
	}
	ad[0].p = pkt;
	ad[0].len = after; /* NTP header + the echoed Unique Identifier field */
	ad[1].p = &pkt[after + 4U + AUTH_LEN_FIELDS];
	ad[1].len = NTS_NONCE_LEN;

	rc = aes_siv_encrypt(&siv, ad, 2U, &pkt[ct_off], plain_len,
			     &pkt[ct_off - AES_SIV_TAG_LEN],
			     AES_SIV_TAG_LEN + plain_len, &out_len);
	if (rc != 0) {
		return rc;
	}

	*len = after + field_len;
	ctx->stats.cookies_issued += n;
	return 0;
}

int nts_ntp_ext_build(void *ctx, const uint8_t *req, size_t req_len, uint8_t *pkt,
		      size_t *len, size_t cap, uint32_t *kod_refid)
{
	nts_ctx_t *c = (nts_ctx_t *)ctx;
	nts_req_t r;
	int act;
	int rc;

	if (c == NULL || req == NULL || pkt == NULL || len == NULL ||
	    kod_refid == NULL) {
		return -EINVAL;
	}

	act = nts_process_request(c, req, req_len, &r);
	if (act < 0) {
		return act;
	}

	switch (act) {
	case NTS_ACT_NONE:
		return 0;
	case NTS_ACT_OK:
		return nts_append_response(c, &r, pkt, len, cap);
	case NTS_ACT_NAK:
		rc = nts_append_nak(c, &r, pkt, len, cap);
		if (rc != 0) {
			return rc;
		}
		*kod_refid = NTS_KISS_NTSN;
		return 0;
	default:
		return -EBADMSG;
	}
}

/* ------------------------------------------------------------- NTS-KE */

int ntske_rec_parse(const uint8_t *buf, size_t len, size_t off, ntske_rec_t *rec,
		    size_t *next)
{
	uint16_t w;

	if (buf == NULL || rec == NULL || next == NULL) {
		return -EINVAL;
	}
	if (off > len || (len - off) < 4U) {
		return -EBADMSG;
	}

	w = bytes_get_be16(&buf[off]);
	rec->critical = (w & 0x8000U) != 0U;
	rec->type = (uint16_t)(w & 0x7FFFU);
	rec->body_len = bytes_get_be16(&buf[off + 2U]);
	if ((len - off - 4U) < rec->body_len) {
		return -EBADMSG;
	}
	rec->body = &buf[off + 4U];
	*next = off + 4U + rec->body_len;
	return 0;
}

int ntske_rec_put(uint8_t *buf, size_t cap, size_t *len, bool critical,
		  uint16_t type, const uint8_t *body, uint16_t body_len)
{
	size_t at;

	if (buf == NULL || len == NULL || (body == NULL && body_len != 0U)) {
		return -EINVAL;
	}
	at = *len;
	if (at > cap || (cap - at) < ((size_t)body_len + 4U)) {
		return -ENOSPC;
	}

	bytes_put_be16(&buf[at],
		       (uint16_t)((critical ? 0x8000U : 0U) | (type & 0x7FFFU)));
	bytes_put_be16(&buf[at + 2U], body_len);
	if (body_len != 0U) {
		memcpy(&buf[at + 4U], body, body_len);
	}
	*len = at + 4U + body_len;
	return 0;
}

int ntske_rec_put_u16(uint8_t *buf, size_t cap, size_t *len, bool critical,
		      uint16_t type, uint16_t value)
{
	uint8_t body[2];

	bytes_put_be16(body, value);
	return ntske_rec_put(buf, cap, len, critical, type, body, sizeof(body));
}

int ntske_exporter_context(uint16_t proto_id, uint16_t aead_id, bool c2s,
			   uint8_t out[NTSKE_EXPORTER_CONTEXT_LEN])
{
	if (out == NULL) {
		return -EINVAL;
	}
	bytes_put_be16(&out[0], proto_id);
	bytes_put_be16(&out[2], aead_id);
	out[4] = c2s ? 0x00U : 0x01U;
	return 0;
}

int ntske_init(ntske_ctx_t *ctx, const ntske_cfg_t *cfg)
{
	if (ctx == NULL || cfg == NULL || cfg->ring == NULL || !cfg->ring->ready ||
	    cfg->export_fn == NULL) {
		return -EINVAL;
	}
	memset(ctx, 0, sizeof(*ctx));
	ctx->cfg = *cfg;
	if (ctx->cfg.cookies == 0U) {
		ctx->cfg.cookies = (uint8_t)NTSKE_COOKIES_DEFAULT;
	}
	if (ctx->cfg.cookies > NTSKE_COOKIES_MAX) {
		ctx->cfg.cookies = (uint8_t)NTSKE_COOKIES_MAX;
	}
	ctx->ready = true;
	return 0;
}

/** True when @p body, a list of big-endian uint16, contains @p want. */
static bool u16_list_has(const uint8_t *body, size_t body_len, uint16_t want)
{
	for (size_t i = 0U; i + 2U <= body_len; i += 2U) {
		if (bytes_get_be16(&body[i]) == want) {
			return true;
		}
	}
	return false;
}

/** Error response: one Error record and End of Message. */
static int ntske_error_response(uint8_t *rsp, size_t cap, size_t *len,
				uint16_t code)
{
	int rc;

	*len = 0U;
	rc = ntske_rec_put_u16(rsp, cap, len, true, (uint16_t)NTSKE_REC_ERROR, code);
	if (rc != 0) {
		return rc;
	}
	return ntske_rec_put(rsp, cap, len, true, (uint16_t)NTSKE_REC_EOM, NULL, 0U);
}

/** Derive the two directional keys through the caller's TLS exporter. */
static int ntske_derive(const ntske_ctx_t *ctx, uint16_t aead_id,
			nts_cookie_keys_t *keys)
{
	uint8_t context[NTSKE_EXPORTER_CONTEXT_LEN];

	keys->aead_id = aead_id;

	(void)ntske_exporter_context((uint16_t)NTSKE_PROTO_NTPV4, aead_id, true,
				     context);
	if (ctx->cfg.export_fn(ctx->cfg.export_ctx, NTSKE_EXPORTER_LABEL, context,
			       sizeof(context), keys->c2s, NTS_KEY_LEN) != 0) {
		return -EIO;
	}

	(void)ntske_exporter_context((uint16_t)NTSKE_PROTO_NTPV4, aead_id, false,
				     context);
	if (ctx->cfg.export_fn(ctx->cfg.export_ctx, NTSKE_EXPORTER_LABEL, context,
			       sizeof(context), keys->s2c, NTS_KEY_LEN) != 0) {
		return -EIO;
	}
	return 0;
}

int ntske_handle(ntske_ctx_t *ctx, const uint8_t *req, size_t req_len,
		 uint8_t *rsp, size_t rsp_cap, size_t *rsp_len,
		 ntske_result_t *res)
{
	size_t off = 0U;
	size_t body_cap;
	uint16_t error = (uint16_t)NTSKE_NO_ERROR;
	bool saw_eom = false;
	bool saw_np = false;
	bool saw_aead = false;
	bool np_ntpv4 = false;
	bool aead_siv = false;
	nts_cookie_keys_t keys;
	int rc;

	if (ctx == NULL || !ctx->ready || (req == NULL && req_len != 0U) ||
	    rsp == NULL || rsp_len == NULL || res == NULL) {
		return -EINVAL;
	}

	memset(res, 0, sizeof(*res));
	res->error = (uint16_t)NTSKE_NO_ERROR;
	*rsp_len = 0U;

	/* Every response ends with End of Message, so hold four octets back and
	 * build everything else inside the remainder. The smallest legal
	 * response is an Error record plus that terminator. */
	if (rsp_cap < 10U) {
		return -ENOSPC;
	}
	body_cap = rsp_cap - 4U;

	while (off < req_len) {
		ntske_rec_t rec;
		size_t next;

		if (ntske_rec_parse(req, req_len, off, &rec, &next) != 0) {
			error = (uint16_t)NTSKE_ERR_BAD_REQUEST;
			break;
		}
		off = next;

		switch (rec.type) {
		case NTSKE_REC_EOM:
			saw_eom = true;
			break;
		case NTSKE_REC_NEXT_PROTO:
			if (saw_np || (rec.body_len & 1U) != 0U) {
				error = (uint16_t)NTSKE_ERR_BAD_REQUEST;
			}
			saw_np = true;
			np_ntpv4 = np_ntpv4 || u16_list_has(rec.body, rec.body_len,
							    (uint16_t)NTSKE_PROTO_NTPV4);
			break;
		case NTSKE_REC_AEAD:
			if (saw_aead || (rec.body_len & 1U) != 0U) {
				error = (uint16_t)NTSKE_ERR_BAD_REQUEST;
			}
			saw_aead = true;
			aead_siv = aead_siv ||
				   u16_list_has(rec.body, rec.body_len,
						(uint16_t)NTS_AEAD_AES_SIV_CMAC_256);
			break;
		case NTSKE_REC_ERROR:
		case NTSKE_REC_WARNING:
		case NTSKE_REC_COOKIE:
		case NTSKE_REC_SERVER:
		case NTSKE_REC_PORT:
			/* Recognised, but a client has no business sending them
			 * to a server. Ignored, per "recognised" — not an
			 * Unrecognized Critical Record. */
			break;
		default:
			if (rec.critical) {
				error = (uint16_t)NTSKE_ERR_UNRECOGNIZED_CRITICAL;
			}
			break;
		}

		if (error != (uint16_t)NTSKE_NO_ERROR || saw_eom) {
			break;
		}
	}

	if (error == (uint16_t)NTSKE_NO_ERROR && !saw_eom) {
		error = (uint16_t)NTSKE_ERR_BAD_REQUEST;
	}
	if (error == (uint16_t)NTSKE_NO_ERROR && !saw_np) {
		/* RFC 8915 §4.1.2: the record is mandatory in a request. */
		error = (uint16_t)NTSKE_ERR_BAD_REQUEST;
	}
	if (error == (uint16_t)NTSKE_NO_ERROR && np_ntpv4 && !saw_aead) {
		/* RFC 8915 §4.1.5: mandatory once NTPv4 is offered. */
		error = (uint16_t)NTSKE_ERR_BAD_REQUEST;
	}
	if (error != (uint16_t)NTSKE_NO_ERROR) {
		res->error = error;
		return ntske_error_response(rsp, rsp_cap, rsp_len, error);
	}

	if (!np_ntpv4) {
		/* RFC 8915 §4.1.2: an empty Next Protocol Negotiation record is
		 * how a server says it speaks none of the offered protocols. */
		rc = ntske_rec_put(rsp, body_cap, rsp_len, true,
				   (uint16_t)NTSKE_REC_NEXT_PROTO, NULL, 0U);
		if (rc != 0) {
			return ntske_error_response(rsp, rsp_cap, rsp_len,
						    (uint16_t)NTSKE_ERR_INTERNAL);
		}
		return ntske_rec_put(rsp, rsp_cap, rsp_len, true,
				     (uint16_t)NTSKE_REC_EOM, NULL, 0U);
	}

	rc = ntske_rec_put_u16(rsp, body_cap, rsp_len, true,
			       (uint16_t)NTSKE_REC_NEXT_PROTO,
			       (uint16_t)NTSKE_PROTO_NTPV4);
	if (rc != 0) {
		res->error = (uint16_t)NTSKE_ERR_INTERNAL;
		return ntske_error_response(rsp, rsp_cap, rsp_len,
					    (uint16_t)NTSKE_ERR_INTERNAL);
	}

	if (!aead_siv) {
		/* RFC 8915 §4.1.5: empty means "none of yours". */
		rc = ntske_rec_put(rsp, body_cap, rsp_len, false,
				   (uint16_t)NTSKE_REC_AEAD, NULL, 0U);
		if (rc != 0) {
			res->error = (uint16_t)NTSKE_ERR_INTERNAL;
			return ntske_error_response(rsp, rsp_cap, rsp_len,
						    (uint16_t)NTSKE_ERR_INTERNAL);
		}
		return ntske_rec_put(rsp, rsp_cap, rsp_len, true,
				     (uint16_t)NTSKE_REC_EOM, NULL, 0U);
	}

	res->aead_id = (uint16_t)NTS_AEAD_AES_SIV_CMAC_256;
	rc = ntske_rec_put_u16(rsp, body_cap, rsp_len, false,
			       (uint16_t)NTSKE_REC_AEAD,
			       (uint16_t)NTS_AEAD_AES_SIV_CMAC_256);

	if (rc == 0 && ctx->cfg.server_name != NULL) {
		size_t n = strlen(ctx->cfg.server_name);

		if (n > UINT16_MAX) {
			n = UINT16_MAX;
		}
		rc = ntske_rec_put(rsp, body_cap, rsp_len, true,
				   (uint16_t)NTSKE_REC_SERVER,
				   (const uint8_t *)ctx->cfg.server_name,
				   (uint16_t)n);
	}
	if (rc == 0 && ctx->cfg.port != 0U) {
		rc = ntske_rec_put_u16(rsp, body_cap, rsp_len, true,
				       (uint16_t)NTSKE_REC_PORT, ctx->cfg.port);
	}
	if (rc == 0) {
		rc = ntske_derive(ctx, (uint16_t)NTS_AEAD_AES_SIV_CMAC_256, &keys);
	}

	if (rc == 0) {
		for (uint8_t i = 0U; i < ctx->cfg.cookies; i++) {
			uint8_t cookie[NTS_COOKIE_LEN];

			rc = nts_cookie_seal(ctx->cfg.ring, &keys, cookie);
			if (rc != 0) {
				break;
			}
			rc = ntske_rec_put(rsp, body_cap, rsp_len, false,
					   (uint16_t)NTSKE_REC_COOKIE, cookie,
					   (uint16_t)sizeof(cookie));
			if (rc == -ENOSPC) {
				/* Out of room: fewer cookies is a valid answer,
				 * none is not. */
				rc = (res->cookies != 0U) ? 0 : -ENOSPC;
				break;
			}
			if (rc != 0) {
				break;
			}
			res->cookies++;
		}
	}

	memset(&keys, 0, sizeof(keys));

	if (rc != 0) {
		res->aead_id = 0U;
		res->cookies = 0U;
		res->error = (uint16_t)NTSKE_ERR_INTERNAL;
		return ntske_error_response(rsp, rsp_cap, rsp_len,
					    (uint16_t)NTSKE_ERR_INTERNAL);
	}

	rc = ntske_rec_put(rsp, rsp_cap, rsp_len, true, (uint16_t)NTSKE_REC_EOM,
			   NULL, 0U);
	if (rc != 0) {
		return rc;
	}
	res->ok = true;
	return 0;
}
