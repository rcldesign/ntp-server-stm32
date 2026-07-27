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

/* ------------------------------------------------------ keyed client identity */

/*
 * SipHash-2-4 (Aumasson & Bernstein, 2012), the standard answer to exactly the
 * problem in ntp_client_id()'s contract: a short-input hash whose *collisions*
 * an adversary must not be able to solve for. ~30 lines, no tables, one pass.
 *
 * Pinned to the reference vectors from the paper in test_ntp.c, so this is not
 * "a hash that looks right": it is SipHash-2-4 or the suite fails.
 */
static uint64_t sip_rotl(uint64_t x, unsigned int b)
{
	return (x << b) | (x >> (64U - b));
}

static uint64_t sip_load_le(const uint8_t *p, size_t n)
{
	uint64_t v = 0U;

	for (size_t i = 0U; i < n; i++) {
		v |= ((uint64_t)p[i]) << (8U * i);
	}
	return v;
}

static uint64_t siphash24(const uint8_t key[NTP_CLIENT_ID_KEY_LEN],
			  const uint8_t *msg, size_t len)
{
	uint64_t k0 = sip_load_le(&key[0], 8U);
	uint64_t k1 = sip_load_le(&key[8], 8U);
	uint64_t v0 = UINT64_C(0x736f6d6570736575) ^ k0;
	uint64_t v1 = UINT64_C(0x646f72616e646f6d) ^ k1;
	uint64_t v2 = UINT64_C(0x6c7967656e657261) ^ k0;
	uint64_t v3 = UINT64_C(0x7465646279746573) ^ k1;
	size_t whole = len & ~(size_t)7U;
	uint64_t b;

#define SIP_ROUND()                                                            \
	do {                                                                   \
		v0 += v1;                                                      \
		v1 = sip_rotl(v1, 13U);                                        \
		v1 ^= v0;                                                      \
		v0 = sip_rotl(v0, 32U);                                        \
		v2 += v3;                                                      \
		v3 = sip_rotl(v3, 16U);                                        \
		v3 ^= v2;                                                      \
		v0 += v3;                                                      \
		v3 = sip_rotl(v3, 21U);                                        \
		v3 ^= v0;                                                      \
		v2 += v1;                                                      \
		v1 = sip_rotl(v1, 17U);                                        \
		v1 ^= v2;                                                      \
		v2 = sip_rotl(v2, 32U);                                        \
	} while (0)

	for (size_t i = 0U; i < whole; i += 8U) {
		uint64_t m = sip_load_le(&msg[i], 8U);

		v3 ^= m;
		SIP_ROUND();
		SIP_ROUND();
		v0 ^= m;
	}

	b = ((uint64_t)(len & 0xFFU)) << 56U;
	b |= sip_load_le(&msg[whole], len - whole);
	v3 ^= b;
	SIP_ROUND();
	SIP_ROUND();
	v0 ^= b;

	v2 ^= 0xFFU;
	SIP_ROUND();
	SIP_ROUND();
	SIP_ROUND();
	SIP_ROUND();

#undef SIP_ROUND

	return v0 ^ v1 ^ v2 ^ v3;
}

uint32_t ntp_client_id(const ntp_ctx_t *ctx, const void *addr, size_t addr_len)
{
	uint64_t tag;

	if (ctx == NULL || addr == NULL || addr_len == 0U) {
		return 0U;
	}
	tag = siphash24(ctx->id_key, (const uint8_t *)addr, addr_len);
	/* Fold rather than truncate, so both halves of the tag contribute. */
	return (uint32_t)(tag ^ (tag >> 32U));
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

int ntp_quality_view_from_block(const quality_block_t *b, uint64_t now_tai_ns,
				uint64_t now_mono_ms, bool time_traceable,
				int8_t precision, ntp_quality_view_t *out)
{
	if (b == NULL || out == NULL) {
		return -EINVAL;
	}

	ntp_quality_view_default(out);
	out->precision = precision;

	out->stratum = b->stratum;
	out->refid = b->refid;
	out->root_delay_q16 = b->root_delay_q16;
	out->root_disp_q16 = b->root_disp_q16;
	out->tai_minus_utc = b->leap_current_s;
	out->holdover = b->holdover;

	/*
	 * Two independent claims have to hold before this server may present
	 * itself as a primary source, and they come from different subsystems:
	 *
	 *   - `disc` says the oscillator is disciplined inside policy, which is
	 *     what b->stratum carries;
	 *   - the platform says the counter the timestamps are *read from* has
	 *     actually been placed on the TAI timescale, which is @p
	 *     time_traceable.
	 *
	 * The second is not implied by the first. The counter powers up at zero
	 * and is placed by a separate mechanism from the one that locks the
	 * oscillator, so a fully locked loop on an unplaced counter yields
	 * perfectly stable timestamps that are decades wrong — served, on this
	 * hardware, under LI=0 / stratum 1 / refid 'GPS' (F1). Requiring both is
	 * the whole point of taking the flag as an argument.
	 */
	out->synchronized = (b->stratum == (uint8_t)QUALITY_STRATUM_PRIMARY) &&
			    time_traceable;

	out->leap = (uint8_t)NTP_LI_NONE;
	if (b->leap_pending != 0 && now_tai_ns != 0U) {
		uint64_t now_s = now_tai_ns / UINT64_C(1000000000);

		if (b->leap_at_tai_s > now_s &&
		    (b->leap_at_tai_s - now_s) <= NTP_LEAP_ANNOUNCE_WINDOW_S) {
			out->leap = (b->leap_pending > 0)
					    ? (uint8_t)NTP_LI_ADD
					    : (uint8_t)NTP_LI_DEL;
		}
	}

	/*
	 * Reference timestamp = the instant `disc` last published. The block
	 * records it in monotonic milliseconds, so it is projected back onto TAI
	 * here. In holdover it correctly stops advancing, which is how a client
	 * sees the staleness (RFC 5905 §7.3).
	 */
	if (now_tai_ns != 0U && b->updated_mono_ms != 0U &&
	    now_mono_ms >= b->updated_mono_ms) {
		uint64_t age_ns = (now_mono_ms - b->updated_mono_ms) *
				  UINT64_C(1000000);

		if (age_ns < now_tai_ns) {
			out->ref_tai_ns = (int64_t)(now_tai_ns - age_ns);
		}
	}

	return 0;
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
	/* Earliest remainder shaped like a MAC we cannot verify; 0 = none.
	 * NTP_HDR_LEN is the smallest legal value, so 0 is a safe sentinel. */
	size_t mac_cand_off = 0U;
	size_t mac_cand_len = 0U;

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
	 *  - A remainder of 4, 20 or 24 octets is a MAC, decided before anything
	 *    else: 4 is below the NTP_EF_LEN_MIN floor entirely, and 20/24 are the
	 *    two digest lengths this server can verify, so a client presenting one
	 *    means it. 4 is a crypto-NAK, with no digest to check.
	 *
	 *  - A remainder of 36/52/68 octets is a key id + a 32/48/64-octet digest,
	 *    a MAC algorithm this server does not implement — but it is also a
	 *    legal extension-field length, so the two readings genuinely collide.
	 *    The rule: walk extension fields, and if the walk consumes the *entire*
	 *    tail (leaving no MAC at all) while some step's remainder was one of
	 *    those three lengths, take the earliest such remainder as the MAC after
	 *    all. `mac_unsupported` then makes the handler reject the request.
	 *
	 *    That rule is the M4/F6 fix, and the ordering is what matters. Trying
	 *    the extension-field parse first and only falling back to the MAC
	 *    reading *if the parse failed* was exploitable: a candidate field's
	 *    Length is read from the same two octets that carry the low half of the
	 *    key id, so `keyid = 36` produced `flen = 36` — 4-aligned, inside the
	 *    remainder, and swallowed as one field. The parse did not fail, so the
	 *    fallback never ran: mac_len came back 0 and a request carrying a MAC
	 *    was answered unauthenticated, against this module's own invariant.
	 *    Splitting the MAC across two aliased fields does not evade the rule
	 *    either, because the *remainder* at the MAC's first octet is what is
	 *    recorded, not the field length. Enforcing NTP_EF_LEN_MIN closes the
	 *    rest of the aliasing window (tails of 8 and 12 were being served as
	 *    extension fields).
	 *
	 *    Cost: a tail that is entirely extension fields but has a 36/52/68
	 *    octet remainder somewhere is rejected as an unverifiable MAC. Nothing
	 *    this firmware speaks is shaped that way — see ntp.h — and the safe
	 *    reading of an ambiguous authenticator is "authentication failed".
	 *
	 *  - Otherwise it is an extension field; the walk is tolerant of a broken
	 *    tail and simply stops, since nothing past ext_len is echoed.
	 */
	off = NTP_HDR_LEN;
	while (off < len) {
		size_t rem = len - off;
		uint16_t flen;

		if (rem == MAC_LEN_NAK || rem == MAC_LEN_128 ||
		    rem == MAC_LEN_160) {
			out->mac_off = off;
			out->mac_len = rem;
			out->keyid = bytes_get_be32(&pkt[off]);
			out->mac_unsupported = (rem == MAC_LEN_NAK);
			break;
		}
		if (rem == MAC_LEN_256 || rem == MAC_LEN_384 ||
		    rem == MAC_LEN_512) {
			if (mac_cand_off == 0U) {
				mac_cand_off = off;
				mac_cand_len = rem;
			}
		}
		if (rem < NTP_EF_LEN_MIN) {
			break; /* runt tail: ignored, never echoed */
		}

		flen = bytes_get_be16(&pkt[off + 2U]);
		if (flen >= NTP_EF_LEN_MIN && (flen & 3U) == 0U &&
		    (size_t)flen <= rem) {
			off += flen;
			out->ext_len = off - NTP_HDR_LEN;
			continue;
		}

		break; /* malformed tail: tolerated, not echoed */
	}

	/* The walk ended without finding a MAC field, yet a MAC-shaped remainder
	 * was passed on the way — whether the tail was consumed entirely as
	 * extension fields or the walk stopped on a malformed one. That remainder
	 * is the MAC. */
	if (out->mac_len == 0U && mac_cand_off != 0U) {
		out->mac_off = mac_cand_off;
		out->mac_len = mac_cand_len;
		out->keyid = bytes_get_be32(&pkt[mac_cand_off]);
		out->mac_unsupported = true;
		out->ext_len = mac_cand_off - NTP_HDR_LEN;
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

	/*
	 * Fixed fallback for the client-identity key. Only reached when there is
	 * no crypto port, or its rand() fails — bring-up must not be blocked by
	 * entropy. It is a *fixed* value rather than zero so the derivation is
	 * still a well-mixed function of the address, but it is public, so an
	 * appliance running on it has an attackable identity (F8): the caller is
	 * expected to supply a crypto port, and every production path does.
	 */
	memcpy(ctx->id_key,
	       "\x53\x54\x53\x31\x30\x30\x30\x2d\x6e\x74\x70\x2d\x69\x64\x00\x01",
	       NTP_CLIENT_ID_KEY_LEN);

	if (crypto != NULL) {
		ctx->crypto = *crypto;
		/*
		 * Key the client identity and salt the table slot from entropy
		 * (F8 / L14). Best-effort and independent: a rand() failure leaves
		 * the documented fallbacks in place and never blocks bring-up.
		 */
		if (crypto->rand != NULL) {
			uint8_t seed[4];
			uint8_t k[NTP_CLIENT_ID_KEY_LEN];

			if (crypto->rand(crypto->ctx, seed, sizeof(seed)) == 0) {
				ctx->hash_seed = ((uint32_t)seed[0]) |
						 ((uint32_t)seed[1] << 8) |
						 ((uint32_t)seed[2] << 16) |
						 ((uint32_t)seed[3] << 24);
			}
			/* Staged, so a partial failure cannot leave half a key. */
			if (crypto->rand(crypto->ctx, k, sizeof(k)) == 0) {
				memcpy(ctx->id_key, k, sizeof(k));
			}
		}
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

/** Digest length a key's algorithm produces, in octets. */
static size_t alg_digest_len(ntp_mac_alg_t alg)
{
	switch (alg) {
	case NTP_MAC_AES_CMAC_128:
	case NTP_MAC_HMAC_SHA256_128:
		return NTP_MAC_DIGEST_128;
	case NTP_MAC_HMAC_SHA256_160:
		return NTP_MAC_DIGEST_160;
	default:
		return 0U;
	}
}

int ntp_key_set(ntp_ctx_t *ctx, uint32_t keyid, ntp_mac_alg_t alg,
		const uint8_t *key, size_t key_len)
{
	ntp_key_t *slot = NULL;

	if (ctx == NULL || key == NULL) {
		return -EINVAL;
	}
	/* RFC 5905 §7.5: key id 0 marks a crypto-NAK, so it can never name a key. */
	if (keyid == 0U || key_len == 0U || key_len > NTP_MAC_KEY_MAX) {
		return -EINVAL;
	}
	if (alg_digest_len(alg) == 0U) {
		return -EINVAL; /* unknown algorithm */
	}
	/* AES-CMAC-128 is keyed by exactly one AES-128 key; anything else would
	 * be truncated or rejected by the block cipher, so refuse it up front. */
	if (alg == NTP_MAC_AES_CMAC_128 && key_len != NTP_MAC_AES_KEY_LEN) {
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
	slot->alg = (uint8_t)alg;
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

/*
 * AES-CMAC-128 (RFC 4493), one-shot over a contiguous message, on top of the
 * port's AES-ECB. core/nts already has a CMAC, but ARCHITECTURE.md §4 forbids
 * an ntp→nts dependency edge, so this is a self-contained copy of the same
 * algorithm; test_ntp pins it to the RFC 4493 example vector independently of
 * test_aes_siv. RFC 8573 defines this as the modern NTP symmetric MAC.
 */
static void cmac_dbl(uint8_t b[16])
{
	uint8_t carry = (uint8_t)(b[0] >> 7);

	for (size_t i = 0U; i < 15U; i++) {
		b[i] = (uint8_t)((uint8_t)(b[i] << 1) | (uint8_t)(b[i + 1U] >> 7));
	}
	b[15] = (uint8_t)((uint8_t)(b[15] << 1) ^ (uint8_t)(0x87U * carry));
}

static int cmac_aes(const port_crypto_t *cr, const uint8_t key[16],
		    const uint8_t in[16], uint8_t out[16])
{
	return cr->aes_ecb_encrypt(cr->ctx, key, 16U, in, out, 16U);
}

static int ntp_cmac128(const port_crypto_t *cr, const uint8_t key[16],
		       const uint8_t *msg, size_t msg_len, uint8_t out[16])
{
	uint8_t k1[16] = { 0 };
	uint8_t k2[16];
	uint8_t x[16] = { 0 };
	uint8_t last[16];
	size_t off = 0U;
	size_t rem;

	/* Subkeys: L = AES(K, 0), K1 = dbl(L), K2 = dbl(K1). */
	if (cmac_aes(cr, key, k1, k1) != 0) {
		return -EIO;
	}
	cmac_dbl(k1);
	memcpy(k2, k1, sizeof(k2));
	cmac_dbl(k2);

	/* CBC-MAC over every block but the last. */
	while ((msg_len - off) > 16U) {
		for (size_t i = 0U; i < 16U; i++) {
			x[i] ^= msg[off + i];
		}
		if (cmac_aes(cr, key, x, x) != 0) {
			return -EIO;
		}
		off += 16U;
	}

	/* Final block: a complete block is XORed with K1; a short (or empty)
	 * block is padded with 0x80 and XORed with K2. */
	rem = msg_len - off;
	memset(last, 0, sizeof(last));
	if (rem == 16U && msg_len != 0U) {
		memcpy(last, &msg[off], 16U);
		for (size_t i = 0U; i < 16U; i++) {
			last[i] ^= k1[i];
		}
	} else {
		if (rem != 0U) {
			memcpy(last, &msg[off], rem);
		}
		last[rem] = 0x80U;
		for (size_t i = 0U; i < 16U; i++) {
			last[i] ^= k2[i];
		}
	}
	for (size_t i = 0U; i < 16U; i++) {
		x[i] ^= last[i];
	}
	if (cmac_aes(cr, key, x, x) != 0) {
		return -EIO;
	}
	memcpy(out, x, 16U);
	return 0;
}

/**
 * Compute a key's MAC digest over @p pkt[0..len). @p out must hold at least
 * NTP_MAC_DIGEST_MAX octets. Returns the digest length, or 0 on failure (the
 * primitive the key's algorithm needs is absent, or the port failed).
 */
static size_t mac_digest(const ntp_ctx_t *ctx, const ntp_key_t *k,
			 const uint8_t *pkt, size_t len, uint8_t *out)
{
	uint8_t full[32];
	size_t dlen = alg_digest_len((ntp_mac_alg_t)k->alg);

	switch ((ntp_mac_alg_t)k->alg) {
	case NTP_MAC_AES_CMAC_128:
		if (ctx->crypto.aes_ecb_encrypt == NULL ||
		    k->key_len != NTP_MAC_AES_KEY_LEN) {
			return 0U;
		}
		if (ntp_cmac128(&ctx->crypto, k->key, pkt, len, out) != 0) {
			return 0U;
		}
		return dlen; /* 16 */
	case NTP_MAC_HMAC_SHA256_128:
	case NTP_MAC_HMAC_SHA256_160:
		if (ctx->crypto.hmac_sha256 == NULL) {
			return 0U;
		}
		if (ctx->crypto.hmac_sha256(ctx->crypto.ctx, k->key, k->key_len,
					    pkt, len, full) != 0) {
			return 0U;
		}
		memcpy(out, full, dlen); /* left-truncate to 16 or 20 */
		return dlen;
	default:
		return 0U;
	}
}

/**
 * Verify the MAC field of a request.
 *
 * The trailing field length fixes the digest length (16 → 20-octet field,
 * 20 → 24-octet field); the key named by the key id must be configured for an
 * algorithm producing exactly that length, or the packet is rejected. See the
 * algorithm map in ntp.h. Comparison is constant time.
 */
static bool mac_verify(const ntp_ctx_t *ctx, const uint8_t *pkt,
		       const ntp_pkt_t *p)
{
	const ntp_key_t *k;
	uint8_t digest[NTP_MAC_DIGEST_MAX];
	size_t dlen;

	/* Reached only for the two verifiable field sizes: the crypto-NAK and the
	 * oversize digests are flagged mac_unsupported and rejected before here,
	 * and the dlen check below rejects any other length regardless. */
	k = key_find(ctx, p->keyid);
	if (k == NULL) {
		return false;
	}
	dlen = alg_digest_len((ntp_mac_alg_t)k->alg);
	if (dlen != MAC_DIGEST_OF(p->mac_len)) {
		/* The key's algorithm does not match the digest length on the wire.
		 * A 16-octet field under a 160-bit key, or vice versa, is a wrong
		 * key type, not a valid MAC. */
		return false;
	}
	if (mac_digest(ctx, k, pkt, p->mac_off, digest) != dlen) {
		return false;
	}
	return ct_memeq(digest, &pkt[p->mac_off + 4U], dlen);
}

static int mac_append(const ntp_ctx_t *ctx, uint32_t keyid, uint8_t *pkt,
		      size_t *len, size_t cap)
{
	const ntp_key_t *k = key_find(ctx, keyid);
	uint8_t digest[NTP_MAC_DIGEST_MAX];
	size_t dlen;

	if (k == NULL) {
		return -ENOENT;
	}
	dlen = alg_digest_len((ntp_mac_alg_t)k->alg);
	if (*len + 4U + dlen > cap) {
		return -ENOSPC;
	}
	if (mac_digest(ctx, k, pkt, *len, digest) != dlen) {
		return -EIO;
	}
	bytes_put_be32(&pkt[*len], keyid);
	memcpy(&pkt[*len + 4U], digest, dlen);
	*len += 4U + dlen;
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
	uint64_t org_out;
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

	/*
	 * --- 4. authenticate ---------------------------------------------
	 *
	 * Behind the rate limiter (L15): when the request is already destined
	 * for a Kiss-o'-Death, skip the MAC entirely, so an over-limit flood of
	 * MAC-bearing packets cannot buy CMAC/HMAC CPU. A KoD is not
	 * authenticated in any case.
	 */
	if (kod_refid == 0U && p.mac_len != 0U) {
		if (p.mac_unsupported || !mac_verify(ctx, rx->pkt, &p)) {
			/* A MAC-shaped tail we cannot verify — unknown key, wrong
			 * key type, a crypto-NAK, or an unimplemented digest length
			 * — is an authentication failure. It is never ignored and
			 * answered unauthenticated: a client that attached a MAC
			 * gets authentication or silence. */
			ctx->stats.auth_fail++;
			finish_drop(ctx, res, NTP_DROP_AUTH);
			return 0;
		}
		authed = true;
		mac_reserve = p.mac_len; /* response uses the same key and length */
	}

	/* --- 5. build ------------------------------------------------------ */
	rx_ntp = ntp_ts_from_tai(rx->rx_tai_ns, q->tai_minus_utc); /* t6 */
	ref_ntp = (q->ref_tai_ns != 0)
			  ? ntp_ts_from_tai(q->ref_tai_ns, q->tai_minus_utc)
			  : rx_ntp;
	rec = rx_ntp;
	xmt = ntp_ts_from_tai(rx->tx_tai_ns, q->tai_minus_utc);
	org_out = p.xmt_ts; /* basic mode (RFC 5905): echo the client transmit */

	/*
	 * Interleaved client/server mode (RFC 9769 §2). The client requests it by
	 * echoing, as its origin timestamp, the *receive* timestamp the server
	 * put in the previous response — NOT the transmit timestamp, which is
	 * what an ordinary RFC 5905 client echoes. Getting that distinction
	 * backwards misreads every basic client as interleaved and answers it
	 * garbage, so the match is against xl_rx_sent and never xl_tx_actual.
	 *
	 * The interleaved reply then reports the current receive timestamp (t6)
	 * and the *measured* transmit instant of the previous response — the value
	 * the server could not know when it built that response. The origin echoes
	 * the request's own receive field.
	 *
	 * Guards: a zero origin (a first-contact client) never matches; the
	 * request's own receive and transmit fields must differ (RFC 9769 forbids
	 * treating an ambiguous request as interleaved); and the committed pair is
	 * consumed after one use so a replayed origin cannot mint a second reply.
	 * A KoD is never interleaved.
	 */
	if (kod_refid == 0U && ctx->cfg.interleave && cl->xl_valid &&
	    p.org_ts != 0U && p.org_ts == cl->xl_rx_sent &&
	    p.rec_ts != p.xmt_ts) {
		org_out = p.rec_ts;
		rec = rx_ntp;
		xmt = cl->xl_tx_actual;
		xleave = true;
		cl->xl_valid = false; /* one-shot */
	}

	/*
	 * Two invariants on the transmit field, both enforced here.
	 *
	 * 1. It MUST differ from this response's receive field: interleaved
	 *    detection tells the two modes apart solely by which of them the
	 *    client later echoes, so they have to be distinct for the next
	 *    exchange to be unambiguous. They differ naturally (transmit is later
	 *    than receive, or is a wholly different exchange's measured instant);
	 *    this is the backstop for the degenerate case where a caller supplies
	 *    identical timestamps.
	 *
	 * 2. It MUST differ from the previous response's transmit field, because
	 *    the platform demultiplexes hardware egress timestamps by it — see
	 *    NTP_XMT_DISTINCT_WINDOW. Note that rule 1 alone made this *worse*
	 *    rather than better: under a coarse clock every client's receive field
	 *    is the same value, so mapping the degenerate case to `rec + 1` handed
	 *    every one of them an identical transmit field (F5).
	 *
	 * One LSB is ~233 ps, so satisfying both costs nothing measurable.
	 */
	if (xmt == rec) {
		xmt = rec + 1U;
	}
	if (xmt == ctx->xmt_last ||
	    (xmt < ctx->xmt_last &&
	     (ctx->xmt_last - xmt) < NTP_XMT_DISTINCT_WINDOW)) {
		xmt = ctx->xmt_last + 1U;
		if (xmt == rec) {
			/* Still one LSB above xmt_last, so rule 2 holds too. */
			xmt = rec + 1U;
		}
	}
	ctx->xmt_last = xmt;

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
		   org_out, rec, xmt);
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

	/*
	 * Arm the interleave pair for this client's next request: remember the
	 * receive field we just sent, and issue a generation token the caller
	 * returns with the measured transmit timestamp (ntp_tx_complete). Only a
	 * real response with interleave enabled arms anything — a KoD carries no
	 * timestamps worth committing, and disarming any prior pending response on
	 * a KoD stops a stale transmit timestamp from being paired with it.
	 */
	if (ctx->cfg.interleave && kod_refid == 0U) {
		cl->xl_gen++;
		if (cl->xl_gen == 0U) {
			cl->xl_gen = 1U; /* token 0 means "no pending" */
		}
		cl->xl_pend_rx_sent = rec;
		cl->xl_pend_token = cl->xl_gen;
		cl->xl_pend_active = true;
		res->xl_token = cl->xl_gen;
	} else if (kod_refid != 0U) {
		cl->xl_pend_active = false;
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

int ntp_tx_complete(ntp_ctx_t *ctx, uint32_t client_id, uint32_t token,
		    uint64_t xmt_ntp)
{
	ntp_client_t *cl;

	if (ctx == NULL || token == 0U) {
		return -EINVAL;
	}
	cl = client_find(ctx, client_id);
	if (cl == NULL || !cl->xl_pend_active || cl->xl_pend_token != token) {
		/* No pending response, or this timestamp is for one a later
		 * request already superseded (M5): drop it rather than pair it
		 * with the wrong exchange. */
		return -ENOENT;
	}

	cl->xl_rx_sent = cl->xl_pend_rx_sent;
	cl->xl_tx_actual = xmt_ntp;
	cl->xl_valid = true;
	cl->xl_pend_active = false;
	return 0;
}
