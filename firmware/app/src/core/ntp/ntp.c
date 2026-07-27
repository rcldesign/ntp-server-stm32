/*
 * STS1000 "Meridian" — core/ntp: RFC 5905 NTPv4 server datapath.
 *
 * See ntp.h for the contract and for the list of things this server
 * deliberately does not implement.
 *
 * Header layout (RFC 5905 §7.3), offsets into the datagram:
 *
 *    0  LI(2) VN(3) Mode(3)      16  Reference Timestamp (8)
 *    1  Stratum                  24  Origin Timestamp    (8)
 *    2  Poll (log2 s)            32  Receive Timestamp   (8)
 *    3  Precision (log2 s, i8)   40  Transmit Timestamp  (8)
 *    4  Root Delay      (short)
 *    8  Root Dispersion (short)
 *   12  Reference Identifier
 *
 * Order of operations in ntp_handle_request(), and why:
 *
 *   1. shape (length, version, mode)   — costs nothing, rejects the most
 *   2. parse                             traffic, and modes 6/7 never get
 *   3. rate limit                        further than step 1
 *   4. authenticate                    — one HMAC; behind the rate limiter so
 *                                        a flood cannot buy CPU time
 *   5. build header, extensions, MAC   — response capacity clipped to the
 *                                        request length before anything
 *                                        authenticated is computed
 */

#include "ntp/ntp.h"

#include <errno.h>
#include <string.h>

#include "util/bytes.h"

_Static_assert((NTP_CLIENT_SLOTS & (NTP_CLIENT_SLOTS - 1U)) == 0U,
	       "NTP_CLIENT_SLOTS must be a power of two");
_Static_assert(NTP_CLIENT_WAYS >= 1U && NTP_CLIENT_WAYS <= NTP_CLIENT_SLOTS,
	       "NTP_CLIENT_WAYS must fit inside the table");
_Static_assert(NTP_PKT_MAX >= NTP_HDR_LEN + NTP_MAC_FIELD_MAX,
	       "NTP_PKT_MAX must hold a header plus a MAC");

/* Header field offsets. */
#define OFF_LIVNMODE 0U
#define OFF_STRATUM  1U
#define OFF_POLL     2U
#define OFF_PRECISION 3U
#define OFF_ROOT_DELAY 4U
#define OFF_ROOT_DISP  8U
#define OFF_REFID     12U
#define OFF_REF_TS    16U
#define OFF_ORG_TS    24U
#define OFF_REC_TS    32U
#define OFF_XMT_TS    40U

/*
 * Trailing lengths that mean "MAC field", not "extension field"
 * (RFC 7822 §7.5.1). Each is 4 (the key id) plus a digest length. The short
 * ones are unambiguous — an extension field beside a MAC is ≥ 28 octets — so
 * 4/20/24 are always MACs. The longer ones (36/52/68 = key id + a 32/48/64
 * octet SHA-2 digest) are ≥ 28 and so could instead be a legitimate extension
 * field; the parser tries an extension-field parse first and only calls them a
 * MAC when that fails.
 */
#define MAC_LEN_NAK 4U   /* key id only: a crypto-NAK, no digest to verify */
#define MAC_LEN_128 20U  /* key id + 16-octet digest: AES-CMAC-128 / HMAC-128 */
#define MAC_LEN_160 24U  /* key id + 20-octet digest: HMAC-SHA-256/160 */
#define MAC_LEN_256 36U  /* key id + 32-octet digest: SHA-256 — unimplemented */
#define MAC_LEN_384 52U  /* key id + 48-octet digest: SHA-384 — unimplemented */
#define MAC_LEN_512 68U  /* key id + 64-octet digest: SHA-512 — unimplemented */

/* The digest length carried by a MAC field of a given total length. */
#define MAC_DIGEST_OF(field_len) ((field_len) - 4U)

/* Milli-tokens per request. Sub-token accumulation is what lets a 8 req/s
 * bucket refill smoothly at millisecond granularity instead of in 125 ms steps. */
#define TOKEN_SCALE 1000U

/* ------------------------------------------------------------------ helpers */

/**
 * Constant-time equality over @p n octets.
 *
 * memcmp() is allowed to return as soon as it finds a difference, which leaks
 * the length of the matching prefix of a MAC and lets an attacker forge one
 * octet at a time. This accumulates every octet before deciding.
 */
static bool ct_memeq(const uint8_t *a, const uint8_t *b, size_t n)
{
	uint8_t diff = 0U;

	for (size_t i = 0U; i < n; i++) {
		diff |= (uint8_t)(a[i] ^ b[i]);
	}
	return diff == 0U;
}

/* murmur3 fmix32: cheap, and avalanches well enough that adjacent IPv4
 * addresses land in different sets of the client table. */
static uint32_t mix32(uint32_t x)
{
	x ^= x >> 16;
	x *= 0x85EBCA6BU;
	x ^= x >> 13;
	x *= 0xC2B2AE35U;
	x ^= x >> 16;
	return x;
}

static uint32_t clamp_burst(uint32_t burst)
{
	if (burst == 0U) {
		return 1U;
	}
	if (burst > NTP_BURST_MAX) {
		return NTP_BURST_MAX;
	}
	return burst;
}

/**
 * Token-bucket refill. @p rate is requests per second, @p burst the depth in
 * requests; both are held in milli-tokens internally.
 *
 * The elapsed time is clamped before it is multiplied so a caller whose
 * monotonic clock starts at an arbitrary large value — or a slot that has sat
 * idle for a week — cannot overflow the product. A backwards step contributes
 * nothing rather than draining the bucket.
 */
static void bucket_refill(uint32_t *tokens, int64_t *last_ms, int64_t now_ms,
			  uint32_t rate, uint32_t burst)
{
	uint64_t cap = (uint64_t)burst * TOKEN_SCALE;
	int64_t d = now_ms - *last_ms;
	uint64_t full;
	uint64_t t;

	*last_ms = now_ms;

	if (rate == 0U) {
		*tokens = (uint32_t)cap; /* bucket disabled: hold it full */
		return;
	}
	if (d <= 0) {
		return;
	}

	/* Milliseconds that alone would fill the bucket from empty. */
	full = cap / rate + 1U;
	if ((uint64_t)d > full) {
		d = (int64_t)full;
	}

	t = (uint64_t)*tokens + (uint64_t)d * rate;
	*tokens = (uint32_t)((t > cap) ? cap : t);
}

/** True when the bucket holds at least one whole token. */
static bool bucket_ready(uint32_t tokens, uint32_t rate)
{
	return rate == 0U || tokens >= TOKEN_SCALE;
}

static void bucket_take(uint32_t *tokens, uint32_t rate)
{
	if (rate != 0U && *tokens >= TOKEN_SCALE) {
		*tokens -= TOKEN_SCALE;
	}
}

/* ------------------------------------------------------------- timestamps */

uint64_t ntp_ts_from_tai(int64_t tai_ns, int32_t tai_minus_utc)
{
	int64_t sec = tai_ns / INT64_C(1000000000);
	int64_t nsec = tai_ns % INT64_C(1000000000);
	uint32_t ntp_sec;
	uint32_t frac;

	/* C truncates toward zero; normalise so nsec is always in [0, 1e9). */
	if (nsec < 0) {
		sec -= 1;
		nsec += INT64_C(1000000000);
	}

	sec -= (int64_t)tai_minus_utc;

	/* Wraps by design: the 32-bit truncation *is* the NTP era (see ntp.h). */
	ntp_sec = (uint32_t)((uint64_t)sec + NTP_UNIX_EPOCH_OFFSET);

	/* nsec < 2^30, so the shift stays inside 64 bits. */
	frac = (uint32_t)(((uint64_t)nsec << 32) / UINT64_C(1000000000));

	return ((uint64_t)ntp_sec << 32) | frac;
}

uint32_t ntp_short_from_q16(uint64_t q16)
{
	return (q16 > UINT64_C(0xFFFFFFFF)) ? UINT32_MAX : (uint32_t)q16;
}

/* ------------------------------------------------------------ quality view */

void ntp_quality_view_default(ntp_quality_view_t *q)
{
	if (q == NULL) {
		return;
	}
	memset(q, 0, sizeof(*q));
	q->leap = (uint8_t)NTP_LI_UNSYNC;
	q->stratum = (uint8_t)NTP_STRATUM_UNSYNC;
	q->precision = -20; /* ~1 µs, the resolution of the MAC PTP timestamps */
	q->refid = NTP_REFID_INIT;
	q->synchronized = false;
}

/* ------------------------------------------------------------------ config */

void ntp_cfg_default(ntp_cfg_t *cfg)
{
	if (cfg == NULL) {
		return;
	}
	memset(cfg, 0, sizeof(*cfg));
	cfg->client_rate = 8U;
	cfg->client_burst = 16U;
	cfg->global_rate = 20000U;
	cfg->global_burst = 40000U;
	cfg->kod_min_interval_ms = 1000U;
	cfg->kod_on_limit = true;
	cfg->interleave = false; /* opt-in, RFC 9769 — see ntp_cfg_t.interleave */
	cfg->serve_unsync = true;
}

/* ------------------------------------------------------------------- parse */

int ntp_parse(const uint8_t *pkt, size_t len, ntp_pkt_t *out)
{
	size_t off;

	if (pkt == NULL || out == NULL) {
		return -EINVAL;
	}
	if (len < NTP_HDR_LEN || len > NTP_PKT_MAX) {
		return -EBADMSG;
	}

	memset(out, 0, sizeof(*out));

	out->li = (uint8_t)((pkt[OFF_LIVNMODE] >> 6) & 0x03U);
	out->vn = (uint8_t)((pkt[OFF_LIVNMODE] >> 3) & 0x07U);
	out->mode = (uint8_t)(pkt[OFF_LIVNMODE] & 0x07U);
	out->stratum = pkt[OFF_STRATUM];
	out->poll = pkt[OFF_POLL];
	out->precision = (int8_t)pkt[OFF_PRECISION];
	out->root_delay = bytes_get_be32(&pkt[OFF_ROOT_DELAY]);
	out->root_disp = bytes_get_be32(&pkt[OFF_ROOT_DISP]);
	out->refid = bytes_get_be32(&pkt[OFF_REFID]);
	out->ref_ts = bytes_get_be64(&pkt[OFF_REF_TS]);
	out->org_ts = bytes_get_be64(&pkt[OFF_ORG_TS]);
	out->rec_ts = bytes_get_be64(&pkt[OFF_REC_TS]);
	out->xmt_ts = bytes_get_be64(&pkt[OFF_XMT_TS]);
	out->ext_off = NTP_HDR_LEN;

	/*
	 * Walk the tail (RFC 7822 §7.5.1). At each step the remainder is either a
	 * MAC field or an extension field.
	 *
	 *  - A remainder of 4, 20 or 24 octets is a MAC: these are shorter than
	 *    the 28-octet minimum for an extension field beside a MAC, so there is
	 *    no ambiguity. 4 is a crypto-NAK (no digest); 20 and 24 carry digests
	 *    we can verify.
	 *  - A remainder of 36/52/68 octets is a keyid + a 32/48/64-octet digest —
	 *    a MAC algorithm this server does not implement. It is ≥ 28, so it
	 *    could instead be an extension field (an NTS Unique Identifier is
	 *    exactly 36 octets); try the extension-field parse first, and only if
	 *    that fails treat it as an unsupported MAC to be rejected. That is the
	 *    fix for the defect where a 36-octet SHA-256 MAC parsed as garbage and
	 *    the request was served unauthenticated.
	 *  - Otherwise it is an extension field; the walk is tolerant of a broken
	 *    tail and simply stops, since nothing past ext_len is echoed.
	 */
	off = NTP_HDR_LEN;
	while (off < len) {
		size_t rem = len - off;
		uint16_t flen;

		if (rem == MAC_LEN_NAK || rem == MAC_LEN_128 || rem == MAC_LEN_160) {
			out->mac_off = off;
			out->mac_len = rem;
			out->keyid = bytes_get_be32(&pkt[off]);
			out->mac_unsupported = (rem == MAC_LEN_NAK);
			break;
		}
		if (rem < 4U) {
			break; /* runt tail: ignored, never echoed */
		}

		flen = bytes_get_be16(&pkt[off + 2U]);
		if (flen >= 4U && (flen & 3U) == 0U && (size_t)flen <= rem) {
			off += flen;
			out->ext_len = off - NTP_HDR_LEN;
			continue;
		}

		/* Not a valid extension field. If the whole remainder is a
		 * MAC-shaped length for a digest we do not implement, say so — the
		 * handler rejects it rather than answering unauthenticated. */
		if (rem == MAC_LEN_256 || rem == MAC_LEN_384 || rem == MAC_LEN_512) {
			out->mac_off = off;
			out->mac_len = rem;
			out->keyid = bytes_get_be32(&pkt[off]);
			out->mac_unsupported = true;
		}
		break; /* malformed tail otherwise: tolerated, not echoed */
	}

	return 0;
}

int ntp_ef_iter_init(ntp_ef_iter_t *it, const uint8_t *pkt, const ntp_pkt_t *p)
{
	if (it == NULL || pkt == NULL || p == NULL) {
		return -EINVAL;
	}
	it->pkt = pkt;
	it->off = p->ext_off;
	it->end = p->ext_off + p->ext_len;
	return 0;
}

int ntp_ef_iter_next(ntp_ef_iter_t *it, ntp_ef_t *ef)
{
	uint16_t flen;

	if (it == NULL || ef == NULL) {
		return -EINVAL;
	}
	if (it->off + 4U > it->end) {
		return -ENOENT;
	}

	flen = bytes_get_be16(&it->pkt[it->off + 2U]);
	/* ntp_parse() already validated every field inside [ext_off, end). */
	ef->type = bytes_get_be16(&it->pkt[it->off]);
	ef->len = flen;
	ef->offset = it->off;
	ef->body = &it->pkt[it->off + 4U];
	ef->body_len = (size_t)flen - 4U;
	it->off += flen;
	return 0;
}

/* ------------------------------------------------------------- client table */

/* Salted so an attacker cannot precompute a set of source addresses that all
 * hash into one NTP_CLIENT_WAYS set and evict a target's entry at will (L14). */
static uint32_t client_slot(const ntp_ctx_t *ctx, uint32_t id)
{
	return mix32(id ^ ctx->hash_seed) & (NTP_CLIENT_SLOTS - 1U);
}

static ntp_client_t *client_find(ntp_ctx_t *ctx, uint32_t id)
{
	uint32_t base = client_slot(ctx, id);

	for (uint32_t w = 0U; w < NTP_CLIENT_WAYS; w++) {
		ntp_client_t *e = &ctx->clients[(base + w) & (NTP_CLIENT_SLOTS - 1U)];

		if (e->used && e->id == id) {
			return e;
		}
	}
	return NULL;
}

/**
 * Find or admit @p id.
 *
 * Set-associative with a NTP_CLIENT_WAYS probe window: bounded work per packet
 * (the whole point at 10 k req/s), and eviction confined to the set. Within the
 * set the victim is the least recently seen entry, which under flood conditions
 * keeps the clients that are actually talking and discards the one-shot
 * spoofed sources. A newly admitted client starts with a full bucket, so the
 * configured burst is what a genuinely new client gets.
 *
 * Eviction also discards that client's interleave state; the next exchange
 * falls back to basic mode, which is correct rather than merely tolerable —
 * the server no longer holds the timestamps an interleaved reply would report.
 */
static ntp_client_t *client_get(ntp_ctx_t *ctx, uint32_t id, int64_t now_ms)
{
	uint32_t base = client_slot(ctx, id);
	ntp_client_t *victim = NULL;

	for (uint32_t w = 0U; w < NTP_CLIENT_WAYS; w++) {
		ntp_client_t *e = &ctx->clients[(base + w) & (NTP_CLIENT_SLOTS - 1U)];

		if (e->used && e->id == id) {
			e->last_seen_ms = now_ms;
			return e;
		}
	}

	for (uint32_t w = 0U; w < NTP_CLIENT_WAYS; w++) {
		ntp_client_t *e = &ctx->clients[(base + w) & (NTP_CLIENT_SLOTS - 1U)];

		if (!e->used) {
			victim = e;
			break;
		}
		if (victim == NULL || e->last_seen_ms < victim->last_seen_ms) {
			victim = e;
		}
	}

	memset(victim, 0, sizeof(*victim));
	victim->id = id;
	victim->used = true;
	victim->last_seen_ms = now_ms;
	victim->tokens_ms = now_ms;
	victim->tokens_milli = clamp_burst(ctx->cfg.client_burst) * TOKEN_SCALE;
	return victim;
}

/* ------------------------------------------------------------------- setup */

int ntp_init(ntp_ctx_t *ctx, const ntp_cfg_t *cfg, const port_crypto_t *crypto,
	     int64_t now_ms)
{
	if (ctx == NULL) {
		return -EINVAL;
	}

	memset(ctx, 0, sizeof(*ctx));

	if (cfg != NULL) {
		ctx->cfg = *cfg;
	} else {
		ntp_cfg_default(&ctx->cfg);
	}
	ctx->cfg.client_burst = clamp_burst(ctx->cfg.client_burst);
	ctx->cfg.global_burst = clamp_burst(ctx->cfg.global_burst);

	if (crypto != NULL) {
		ctx->crypto = *crypto;
	}

	ctx->g_tokens_ms = now_ms;
	ctx->g_tokens_milli = ctx->cfg.global_burst * TOKEN_SCALE;
	return 0;
}

int ntp_set_ext_hook(ntp_ctx_t *ctx, const ntp_ext_hook_t *hook)
{
	if (ctx == NULL) {
		return -EINVAL;
	}
	ctx->ext = hook;
	return 0;
}

int ntp_key_set(ntp_ctx_t *ctx, uint32_t keyid, const uint8_t *key, size_t key_len)
{
	ntp_key_t *slot = NULL;

	if (ctx == NULL || key == NULL) {
		return -EINVAL;
	}
	/* RFC 5905 §7.5: key id 0 marks a crypto-NAK, so it can never name a key. */
	if (keyid == 0U || key_len == 0U || key_len > NTP_MAC_KEY_MAX) {
		return -EINVAL;
	}

	for (size_t i = 0U; i < NTP_MAC_KEYS; i++) {
		if (ctx->keys[i].used && ctx->keys[i].keyid == keyid) {
			slot = &ctx->keys[i];
			break;
		}
		if (!ctx->keys[i].used && slot == NULL) {
			slot = &ctx->keys[i];
		}
	}
	if (slot == NULL) {
		return -ENOSPC;
	}

	memset(slot, 0, sizeof(*slot));
	slot->keyid = keyid;
	slot->key_len = (uint8_t)key_len;
	memcpy(slot->key, key, key_len);
	slot->used = true;
	return 0;
}

int ntp_key_clear(ntp_ctx_t *ctx, uint32_t keyid)
{
	if (ctx == NULL) {
		return -EINVAL;
	}
	for (size_t i = 0U; i < NTP_MAC_KEYS; i++) {
		if (ctx->keys[i].used && ctx->keys[i].keyid == keyid) {
			memset(&ctx->keys[i], 0, sizeof(ctx->keys[i]));
			return 0;
		}
	}
	return -ENOENT;
}

static const ntp_key_t *key_find(const ntp_ctx_t *ctx, uint32_t keyid)
{
	for (size_t i = 0U; i < NTP_MAC_KEYS; i++) {
		if (ctx->keys[i].used && ctx->keys[i].keyid == keyid) {
			return &ctx->keys[i];
		}
	}
	return NULL;
}

int ntp_stats_get(const ntp_ctx_t *ctx, ntp_stats_t *out)
{
	if (ctx == NULL || out == NULL) {
		return -EINVAL;
	}
	*out = ctx->stats;
	return 0;
}

int ntp_stats_reset(ntp_ctx_t *ctx)
{
	if (ctx == NULL) {
		return -EINVAL;
	}
	memset(&ctx->stats, 0, sizeof(ctx->stats));
	return 0;
}

/* --------------------------------------------------------------------- MAC */

/**
 * Verify the MAC field of a request.
 *
 * MAC = HMAC-SHA-256(key, everything before the MAC field), left-truncated to
 * NTP_MAC_DIGEST_LEN octets. See the interop note in ntp.h; the truncation
 * length is the one that keeps the field the same 24 octets as the SHA-1 MACs
 * deployed clients already emit.
 */
static bool mac_verify(const ntp_ctx_t *ctx, const uint8_t *pkt,
		       const ntp_pkt_t *p)
{
	const ntp_key_t *k;
	uint8_t digest[32];

	if (p->mac_len != MAC_LEN_20) {
		/* A 4-octet field is a client-sent crypto-NAK and a 20-octet field
		 * is an MD5 digest; neither is something to authenticate against. */
		return false;
	}
	if (ctx->crypto.hmac_sha256 == NULL) {
		return false;
	}
	k = key_find(ctx, p->keyid);
	if (k == NULL) {
		return false;
	}
	if (ctx->crypto.hmac_sha256(ctx->crypto.ctx, k->key, k->key_len, pkt,
				    p->mac_off, digest) != 0) {
		return false;
	}
	return ct_memeq(digest, &pkt[p->mac_off + 4U], NTP_MAC_DIGEST_LEN);
}

static int mac_append(const ntp_ctx_t *ctx, uint32_t keyid, uint8_t *pkt,
		      size_t *len, size_t cap)
{
	const ntp_key_t *k = key_find(ctx, keyid);
	uint8_t digest[32];

	if (k == NULL || ctx->crypto.hmac_sha256 == NULL) {
		return -ENOENT;
	}
	if (*len + NTP_MAC_FIELD_LEN > cap) {
		return -ENOSPC;
	}
	if (ctx->crypto.hmac_sha256(ctx->crypto.ctx, k->key, k->key_len, pkt, *len,
				    digest) != 0) {
		return -EIO;
	}
	bytes_put_be32(&pkt[*len], keyid);
	memcpy(&pkt[*len + 4U], digest, NTP_MAC_DIGEST_LEN);
	*len += NTP_MAC_FIELD_LEN;
	return 0;
}

/* ---------------------------------------------------------------- response */

static void put_header(uint8_t *b, uint8_t li, uint8_t vn, uint8_t stratum,
		       uint8_t poll, int8_t precision, uint32_t root_delay,
		       uint32_t root_disp, uint32_t refid, uint64_t ref,
		       uint64_t org, uint64_t rec, uint64_t xmt)
{
	b[OFF_LIVNMODE] = (uint8_t)(((li & 0x03U) << 6) | ((vn & 0x07U) << 3) |
				    (uint8_t)NTP_MODE_SERVER);
	b[OFF_STRATUM] = stratum;
	b[OFF_POLL] = poll;
	b[OFF_PRECISION] = (uint8_t)precision;
	bytes_put_be32(&b[OFF_ROOT_DELAY], root_delay);
	bytes_put_be32(&b[OFF_ROOT_DISP], root_disp);
	bytes_put_be32(&b[OFF_REFID], refid);
	bytes_put_be64(&b[OFF_REF_TS], ref);
	bytes_put_be64(&b[OFF_ORG_TS], org);
	bytes_put_be64(&b[OFF_REC_TS], rec);
	bytes_put_be64(&b[OFF_XMT_TS], xmt);
}

/** Rewrite an already-built header as a Kiss-o'-Death carrying @p refid. */
static void make_kod(uint8_t *b, uint32_t refid)
{
	b[OFF_LIVNMODE] = (uint8_t)((b[OFF_LIVNMODE] & 0x3FU) |
				    ((uint8_t)NTP_LI_UNSYNC << 6));
	b[OFF_STRATUM] = (uint8_t)NTP_STRATUM_KOD;
	bytes_put_be32(&b[OFF_REFID], refid);
	/* Root delay and dispersion stay as built: RFC 5905 §7.4 assigns them no
	 * meaning in a KoD, and zeroing them would only hide the server's state
	 * from an operator taking a capture. */
}

/* ------------------------------------------------------------------- entry */

static void finish_drop(ntp_ctx_t *ctx, ntp_result_t *res, ntp_drop_t why)
{
	res->action = NTP_ACT_IGNORE;
	res->drop = why;
	res->len = 0U;
	ctx->stats.dropped++;
}

int ntp_handle_request(ntp_ctx_t *ctx, const ntp_rx_t *rx,
		       const ntp_quality_view_t *q,
		       uint8_t *out, size_t out_cap, ntp_result_t *res)
{
	ntp_quality_view_t qdef;
	ntp_pkt_t p;
	ntp_client_t *cl;
	uint64_t rx_ntp;
	uint64_t ref_ntp;
	uint64_t xmt;
	uint64_t rec;
	uint32_t refid;
	uint32_t kod_refid = 0U;
	uint8_t li;
	uint8_t stratum;
	size_t budget;
	size_t hook_cap;
	size_t mac_reserve = 0U;
	size_t len;
	bool authed = false;
	bool xleave = false;
	bool over_limit;

	if (ctx == NULL || rx == NULL || out == NULL || res == NULL) {
		return -EINVAL;
	}
	if (rx->pkt == NULL && rx->len != 0U) {
		return -EINVAL;
	}

	memset(res, 0, sizeof(*res));
	ctx->stats.rx++;

	if (out_cap < NTP_HDR_LEN) {
		finish_drop(ctx, res, NTP_DROP_NOSPACE);
		return 0;
	}

	/* --- 1. shape ---------------------------------------------------- */
	if (rx->len < NTP_HDR_LEN || rx->len > NTP_PKT_MAX) {
		finish_drop(ctx, res, NTP_DROP_SHORT);
		return 0;
	}
	{
		uint8_t vn = (uint8_t)((rx->pkt[OFF_LIVNMODE] >> 3) & 0x07U);
		uint8_t mode = (uint8_t)(rx->pkt[OFF_LIVNMODE] & 0x07U);

		if (mode != NTP_MODE_CLIENT) {
			/* Modes 6 and 7 land here with everything else: no control
			 * plane, no monlist, no reply of any kind. */
			res->action = NTP_ACT_IGNORE;
			res->drop = NTP_DROP_MODE;
			ctx->stats.ignored++;
			return 0;
		}
		if (vn < NTP_VN_MIN || vn > NTP_VN_MAX) {
			res->action = NTP_ACT_IGNORE;
			res->drop = NTP_DROP_VERSION;
			ctx->stats.ignored++;
			return 0;
		}
	}

	/* --- 2. parse ---------------------------------------------------- */
	if (ntp_parse(rx->pkt, rx->len, &p) != 0) {
		finish_drop(ctx, res, NTP_DROP_SHORT);
		return 0;
	}

	if (q == NULL) {
		ntp_quality_view_default(&qdef);
		q = &qdef;
	}
	if (!q->synchronized && !ctx->cfg.serve_unsync) {
		finish_drop(ctx, res, NTP_DROP_UNSYNC);
		return 0;
	}

	/* --- 3. rate limit ------------------------------------------------ */
	cl = client_get(ctx, rx->client_id, rx->now_ms);

	bucket_refill(&cl->tokens_milli, &cl->tokens_ms, rx->now_ms,
		      ctx->cfg.client_rate, ctx->cfg.client_burst);
	bucket_refill(&ctx->g_tokens_milli, &ctx->g_tokens_ms, rx->now_ms,
		      ctx->cfg.global_rate, ctx->cfg.global_burst);

	over_limit = !bucket_ready(cl->tokens_milli, ctx->cfg.client_rate) ||
		     !bucket_ready(ctx->g_tokens_milli, ctx->cfg.global_rate);

	if (over_limit) {
		bool damped = cl->kod_seen &&
			      (rx->now_ms - cl->kod_last_ms) <
				      (int64_t)ctx->cfg.kod_min_interval_ms;

		ctx->stats.rate_limited++;
		if (!ctx->cfg.kod_on_limit || damped) {
			finish_drop(ctx, res, NTP_DROP_RATE);
			return 0;
		}
		cl->kod_seen = true;
		cl->kod_last_ms = rx->now_ms;
		kod_refid = NTP_REFID_RATE;
	} else {
		bucket_take(&cl->tokens_milli, ctx->cfg.client_rate);
		bucket_take(&ctx->g_tokens_milli, ctx->cfg.global_rate);
	}

	/* --- 4. authenticate ---------------------------------------------- */
	if (p.mac_len != 0U) {
		if (!mac_verify(ctx, rx->pkt, &p)) {
			ctx->stats.auth_fail++;
			finish_drop(ctx, res, NTP_DROP_AUTH);
			return 0;
		}
		authed = true;
		mac_reserve = NTP_MAC_FIELD_LEN;
	}

	/* --- 5. build ------------------------------------------------------ */
	rx_ntp = ntp_ts_from_tai(rx->rx_tai_ns, q->tai_minus_utc);
	ref_ntp = (q->ref_tai_ns != 0)
			  ? ntp_ts_from_tai(q->ref_tai_ns, q->tai_minus_utc)
			  : rx_ntp;
	rec = rx_ntp;
	xmt = ntp_ts_from_tai(rx->tx_tai_ns, q->tai_minus_utc);

	/*
	 * Interleaved client/server mode. The client marks a request interleaved
	 * by echoing, as its origin timestamp, the transmit field of the previous
	 * response. When that matches, the reply reports the *previous* exchange's
	 * receive timestamp and the hardware transmit timestamp of the previous
	 * response — the one the server could not know at the time it built it.
	 * A KoD is never interleaved: it carries no useful timestamps.
	 */
	if (kod_refid == 0U && ctx->cfg.interleave && cl->xl_valid &&
	    cl->xl_tx_field != 0U && p.org_ts == cl->xl_tx_field) {
		rec = cl->xl_rx;
		xmt = cl->xl_tx_actual;
		xleave = true;
	}

	if (!q->synchronized) {
		li = (uint8_t)NTP_LI_UNSYNC;
		stratum = (uint8_t)NTP_STRATUM_UNSYNC;
		refid = NTP_REFID_INIT;
	} else {
		li = (uint8_t)(q->leap & 0x03U);
		stratum = q->stratum;
		refid = q->refid;
		if (refid == 0U && stratum == (uint8_t)NTP_STRATUM_PRIM) {
			refid = NTP_REFID_GPS;
		}
	}

	put_header(out, li, p.vn, stratum, p.poll, q->precision,
		   ntp_short_from_q16(q->root_delay_q16),
		   ntp_short_from_q16(q->root_disp_q16), refid, ref_ntp,
		   p.xmt_ts, rec, xmt);
	len = NTP_HDR_LEN;

	/*
	 * Anti-amplification: clip to the request length before the extension
	 * hook runs, so what it authenticates is what goes on the wire. A
	 * response is never larger than the request that provoked it.
	 */
	budget = (out_cap < rx->len) ? out_cap : rx->len;
	if (budget < NTP_HDR_LEN + mac_reserve) {
		/* Only reachable through an undersized output buffer: a request
		 * carrying a verifiable MAC is itself at least 72 octets. */
		finish_drop(ctx, res, NTP_DROP_NOSPACE);
		return 0;
	}
	hook_cap = budget - mac_reserve;

	if (ctx->ext != NULL && ctx->ext->build != NULL && kod_refid == 0U) {
		uint32_t hook_kod = 0U;
		int rc = ctx->ext->build(ctx->ext->ctx, rx->pkt, rx->len, out, &len,
					 hook_cap, &hook_kod);

		if (rc != 0 || len > hook_cap) {
			/* Either the hook declined, or it reported a length past
			 * the capacity it was given — nothing safe to send. */
			finish_drop(ctx, res, NTP_DROP_EXT);
			return 0;
		}
		if (hook_kod != 0U) {
			kod_refid = hook_kod;
			xleave = false;
		}
	}

	if (kod_refid != 0U) {
		make_kod(out, kod_refid);
	}

	if (authed) {
		if (mac_append(ctx, p.keyid, out, &len, budget) != 0) {
			finish_drop(ctx, res, NTP_DROP_INTERNAL);
			return 0;
		}
	}

	/* Arm interleaved mode for the next request from this client. Only a real
	 * response carries timestamps worth remembering. */
	if (kod_refid == 0U) {
		cl->xl_pend_rx = rx_ntp;
		cl->xl_pend_tx_field = xmt;
		cl->xl_pending = true;
	} else {
		/*
		 * A Kiss-o'-Death is still a datagram, so the caller will report
		 * its transmit timestamp. Disarming here makes that report a
		 * no-op instead of letting it pair an *older* response's
		 * transmit field with the KoD's measured instant — which would
		 * hand the client's next interleaved request a timestamp from a
		 * packet it never saw.
		 */
		cl->xl_pending = false;
	}

	res->action = (kod_refid != 0U) ? NTP_ACT_KOD : NTP_ACT_RESPOND;
	res->len = len;
	res->xmt = xmt;
	res->interleaved = xleave;
	res->authenticated = authed;

	if (kod_refid != 0U) {
		ctx->stats.kod++;
	} else {
		ctx->stats.served++;
		if (xleave) {
			ctx->stats.interleaved++;
		}
	}
	return 0;
}

int ntp_tx_complete(ntp_ctx_t *ctx, uint32_t client_id, uint64_t xmt_ntp)
{
	ntp_client_t *cl;

	if (ctx == NULL) {
		return -EINVAL;
	}
	cl = client_find(ctx, client_id);
	if (cl == NULL || !cl->xl_pending) {
		return -ENOENT;
	}

	cl->xl_rx = cl->xl_pend_rx;
	cl->xl_tx_field = cl->xl_pend_tx_field;
	cl->xl_tx_actual = xmt_ntp;
	cl->xl_valid = true;
	cl->xl_pending = false;
	return 0;
}
