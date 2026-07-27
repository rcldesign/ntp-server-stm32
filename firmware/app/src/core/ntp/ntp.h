/*
 * STS1000 "Meridian" — core/ntp: RFC 5905 NTPv4 server datapath.
 *
 * Platform-neutral C11. No dynamic allocation, no platform headers; the caller
 * owns every buffer and the ntp_ctx_t. Crypto arrives through port_crypto_t
 * (ARCHITECTURE.md §4); time and entropy are injected, so the whole datapath is
 * deterministic under host test.
 *
 * Scope (spec §4.1/§4.3):
 *
 *   - Request parse/validate, response build from the §3.8 quality block.
 *   - TAI → NTP 64-bit timestamp conversion with explicit era semantics.
 *   - Per-client and aggregate token-bucket rate limiting with Kiss-o'-Death.
 *   - Interleaved client/server mode (RFC 9769), off by default and detected
 *     strictly per §2: the origin echoes the previous response's *receive*
 *     timestamp, never its transmit timestamp — the distinction from a basic
 *     RFC 5905 client, which echoes the transmit timestamp.
 *   - Symmetric-key authentication: RFC 8573 AES-CMAC-128, plus HMAC-SHA-256
 *     truncated to 128 or 160 bits, selectable per key.
 *   - An extension-field hook so core/nts can protect the same datapath
 *     without ntp and nts depending on one another (ARCHITECTURE.md §4 fixes
 *     `ntp→util,quality` and `nts→util`; there is deliberately no edge between
 *     them, so NTS integration is by caller-supplied callback).
 *
 * Deliberately NOT implemented, and why:
 *
 *   - Mode 6 (control) and mode 7 (private/ntpdc). Both are the classic NTP
 *     reflection/amplification vectors (`monlist`); a grandmaster has no need
 *     of either. Such packets are dropped without a response, counted in
 *     ntp_stats_t.ignored.
 *   - Modes 1/2/5 (symmetric active/passive, broadcast). This is a stratum-1
 *     server, not a peer; only mode 3 is answered, with mode 4.
 *   - NTPv1/v2 (VN 1, 2). Their mode-field overloading has no useful client
 *     population left. Only VN 3 and VN 4 are served.
 *   - MD5 MACs (RFC 5905 Appendix A). MD5 is broken for authentication. A
 *     MAC-shaped tail of an unimplemented digest length (including MD5's
 *     16-octet digest presented under a key configured for it, or a 32-octet
 *     SHA-256 digest) is recognised as a MAC and rejected — never mis-parsed
 *     as an extension field and answered unauthenticated.
 *   - Autokey (RFC 5906). Deprecated; NTS (core/nts) is the authenticated path.
 *   - Crypto-NAK responses. Answering an unauthenticated packet with a
 *     distinguishable "your key is wrong" reply is an oracle and an
 *     amplification hook for a spoofed source; authentication failures are
 *     silent (spec §9.7 hardening).
 */

#ifndef STS1000_CORE_NTP_NTP_H_
#define STS1000_CORE_NTP_NTP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "port/port_crypto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ wire */

/** Fixed NTP header length (RFC 5905 §7.3), octets. */
#define NTP_HDR_LEN 48U

/**
 * Largest datagram accepted or produced, octets.
 *
 * Sized so an NTS-protected exchange (48 B header + Unique Identifier +
 * authenticator carrying eight 104-octet cookie fields ≈ 956 B) fits with room
 * to spare while staying under the 1280 B IPv6 minimum MTU, which keeps the
 * time-serving path free of fragmentation.
 */
#ifndef NTP_PKT_MAX
#define NTP_PKT_MAX 1280U
#endif

/** Leap Indicator values (RFC 5905 §7.3). */
#define NTP_LI_NONE   0U /* no warning */
#define NTP_LI_ADD    1U /* last minute of the day has 61 s */
#define NTP_LI_DEL    2U /* last minute of the day has 59 s */
#define NTP_LI_UNSYNC 3U /* clock unsynchronised */

/** Association modes (RFC 5905 §7.3). */
#define NTP_MODE_SYM_ACTIVE  1U
#define NTP_MODE_SYM_PASSIVE 2U
#define NTP_MODE_CLIENT      3U
#define NTP_MODE_SERVER      4U
#define NTP_MODE_BROADCAST   5U
#define NTP_MODE_CONTROL     6U
#define NTP_MODE_PRIVATE     7U

/** Versions this server answers. */
#define NTP_VN_MIN 3U
#define NTP_VN_MAX 4U

/** Stratum values of interest (RFC 5905 §7.3). */
#define NTP_STRATUM_KOD    0U  /* unspecified; refid carries a kiss code */
#define NTP_STRATUM_PRIM   1U  /* primary reference (this server, locked) */
#define NTP_STRATUM_UNSYNC 16U /* unsynchronised */

/** Reference identifiers, host order, written big-endian on the wire. */
#define NTP_REFID_MAKE(a, b, c, d)                                            \
	(((uint32_t)(uint8_t)(a) << 24) | ((uint32_t)(uint8_t)(b) << 16) |    \
	 ((uint32_t)(uint8_t)(c) << 8) | (uint32_t)(uint8_t)(d))

#define NTP_REFID_GPS  NTP_REFID_MAKE('G', 'P', 'S', 0)    /* stratum-1 source */
#define NTP_REFID_INIT NTP_REFID_MAKE('I', 'N', 'I', 'T')  /* not yet synced */
#define NTP_REFID_RATE NTP_REFID_MAKE('R', 'A', 'T', 'E')  /* KoD: rate exceeded */
#define NTP_REFID_NTSN NTP_REFID_MAKE('N', 'T', 'S', 'N')  /* KoD: NTS NAK */
#define NTP_REFID_DENY NTP_REFID_MAKE('D', 'E', 'N', 'Y')  /* KoD: access denied */

/* -------------------------------------------------------------- timestamps */

/**
 * Seconds between the NTP prime epoch (1900-01-01T00:00:00Z) and the Unix
 * epoch (1970-01-01T00:00:00Z).
 */
#define NTP_UNIX_EPOCH_OFFSET UINT64_C(2208988800)

/**
 * Convert a TAI instant to an NTP 64-bit timestamp (32.32 fixed point, seconds
 * since the prime epoch in the upper half).
 *
 * @param tai_ns         Nanoseconds since 1970-01-01T00:00:00 **TAI**. This is
 *                       the timebase the PPS/PTP capture path produces: it is
 *                       continuous and has no leap discontinuity.
 * @param tai_minus_utc  TAI−UTC in seconds (37 as of 2017-01-01). Subtracted to
 *                       reach UTC, which is what NTP carries.
 *
 * **Era.** NTP seconds are 32 bits and wrap every 2^32 s ≈ 136 years. Era 0 ends
 * at 2036-02-07T06:28:15Z; the next second is era 1, second 0. This function
 * returns the era-truncated value, which is precisely what RFC 5905 §6 puts on
 * the wire — the era is never transmitted. Both peers resolve it from their own
 * approximate time, so a server that has any idea what year it is stays correct
 * across the rollover without special casing. Anything in this firmware that
 * needs an unambiguous instant uses the TAI nanosecond value, not this one.
 */
uint64_t ntp_ts_from_tai(int64_t tai_ns, int32_t tai_minus_utc);

/**
 * Convert seconds in 16.16 fixed point to the NTP short format carried by the
 * root-delay and root-dispersion fields.
 *
 * The short format *is* 16.16, so the conversion is a range check: the argument
 * is 64-bit because the natural place to accumulate a growing holdover
 * dispersion is a wide counter, and the result saturates at the format maximum
 * of 65535.9999847 s. Saturating rather than wrapping matters — a wrapped
 * dispersion reads as a tiny one, and a client would trust a clock that has in
 * fact been free-running for a day.
 */
uint32_t ntp_short_from_q16(uint64_t q16);

/* ------------------------------------------------------------- quality view */

/**
 * The subset of the §3.8 service-quality block the NTP header needs.
 *
 * `core/quality` is the single source of truth (spec §3.8: "services never
 * compute their own"). This is a plain value struct the caller fills from a
 * quality snapshot; ntp derives nothing about clock health on its own.
 */
typedef struct {
	/** Leap Indicator to advertise (NTP_LI_*). Ignored when !synchronized. */
	uint8_t leap;
	/** Stratum to advertise. 1 when GNSS-locked. Ignored when !synchronized. */
	uint8_t stratum;
	/** Clock precision as a power of two seconds; −20 ≈ 1 µs. */
	int8_t precision;
	/**
	 * Reference identifier, host order. Zero means "derive": stratum 1 gets
	 * NTP_REFID_GPS, anything else gets 0. A secondary configuration
	 * (upstream NTP fallback, spec §3.6) sets the upstream-derived value here.
	 */
	uint32_t refid;
	/** Root delay, seconds in 16.16 fixed point. */
	uint32_t root_delay_q16;
	/** Root dispersion, seconds in 16.16 fixed point. */
	uint32_t root_disp_q16;
	/** TAI−UTC in seconds, for the timestamp conversion. */
	int32_t tai_minus_utc;
	/**
	 * Instant of the last clock update, TAI ns; 0 means "use the receive
	 * timestamp". Becomes the response's reference timestamp, so in holdover
	 * it correctly stops advancing and clients can see the staleness.
	 */
	int64_t ref_tai_ns;
	/**
	 * Holdover flag, informational only. `quality` has already folded holdover
	 * into stratum, leap and root_disp_q16 (spec §3.6/§3.8); ntp does not
	 * re-derive any of them from this bit. It is carried so callers can log
	 * and annunciate against the same snapshot the response was built from.
	 */
	bool holdover;
	/** False → advertise LI=3, stratum 16, refid 'INIT' whatever else says. */
	bool synchronized;
} ntp_quality_view_t;

/** Fill @p q with the unsynchronised defaults (LI 3, stratum 16, precision −20). */
void ntp_quality_view_default(ntp_quality_view_t *q);

/* --------------------------------------------------------- parsed request */

/** A parsed NTP datagram. Offsets index the caller's buffer. */
typedef struct {
	uint8_t li;
	uint8_t vn;
	uint8_t mode;
	uint8_t stratum;
	uint8_t poll;
	int8_t precision;
	uint32_t root_delay;
	uint32_t root_disp;
	uint32_t refid;
	uint64_t ref_ts;
	uint64_t org_ts;
	uint64_t rec_ts;
	uint64_t xmt_ts;
	/** Offset of the first extension field: always NTP_HDR_LEN. */
	size_t ext_off;
	/** Octets of well-formed extension fields following the header. */
	size_t ext_len;
	/** Offset of the MAC field, or 0 when there is none. */
	size_t mac_off;
	/**
	 * MAC field length: 0 (absent), or the length of a trailing field the
	 * disambiguation classified as a MAC — 4 (key id only, a crypto-NAK), 20
	 * (key id + 16-octet digest: AES-CMAC-128 or HMAC-SHA-256/128), 24 (key id
	 * + 20-octet digest: HMAC-SHA-256/160), or a longer MAC-shaped remainder
	 * (36/52/68) naming a digest length this server does not implement.
	 */
	size_t mac_len;
	/**
	 * The trailing field is MAC-shaped but of a length this server cannot
	 * verify — a crypto-NAK, or a 32/48/64-octet digest (SHA-256/384/512).
	 * The handler MUST reject such a request rather than serve it
	 * unauthenticated: a client that attached a MAC expects authentication or
	 * silence, never an unauthenticated answer it will trust.
	 */
	bool mac_unsupported;
	/** Key identifier from the MAC field; meaningless when mac_len == 0. */
	uint32_t keyid;
} ntp_pkt_t;

/**
 * Parse a datagram into @p out. Performs no policy: version, mode and
 * authentication are the caller's business.
 *
 * Tail disambiguation follows RFC 7822 §7.5.1 and long-standing practice
 * (ntpd, chrony): a trailing remainder whose length is that of a MAC field
 * (key id + a recognised digest length) is a MAC, not an extension field —
 * extension fields alongside a MAC are ≥ 28 octets, so the short MAC lengths
 * are unambiguous. A MAC-shaped remainder of a digest length this server does
 * not implement is still recognised, and flagged in @p mac_unsupported so the
 * handler rejects it instead of ignoring it and answering unauthenticated.
 *
 * The extension-field walk is otherwise tolerant: it stops at the first field
 * that is short, unaligned or overruns, reports the octets it did validate in
 * @p ext_len, and ignores the remainder. Nothing beyond ext_len is ever echoed.
 *
 * @retval 0         Parsed. @p out is fully populated.
 * @retval -EINVAL   @p pkt or @p out is NULL.
 * @retval -EBADMSG  Shorter than NTP_HDR_LEN or longer than NTP_PKT_MAX.
 */
int ntp_parse(const uint8_t *pkt, size_t len, ntp_pkt_t *out);

/** One RFC 7822 extension field located inside a packet buffer. */
typedef struct {
	uint16_t type;      /**< Field Type. */
	uint16_t len;       /**< Field Length, header included; a multiple of 4. */
	size_t offset;      /**< Offset of the field header within the packet. */
	const uint8_t *body;/**< Field Value, len − 4 octets. */
	size_t body_len;    /**< len − 4. */
} ntp_ef_t;

/** Extension-field walk state. All fields private. */
typedef struct {
	const uint8_t *pkt;
	size_t end;  /* one past the last extension-field octet */
	size_t off;  /* next field header */
} ntp_ef_iter_t;

/**
 * Bind an iterator to the extension-field region described by @p p.
 *
 * @retval 0        Bound (possibly to an empty region).
 * @retval -EINVAL  A NULL argument.
 */
int ntp_ef_iter_init(ntp_ef_iter_t *it, const uint8_t *pkt, const ntp_pkt_t *p);

/**
 * Advance to the next extension field.
 *
 * @retval 0        @p ef holds the next field.
 * @retval -ENOENT  End of the region.
 * @retval -EINVAL  A NULL argument.
 */
int ntp_ef_iter_next(ntp_ef_iter_t *it, ntp_ef_t *ef);

/* ------------------------------------------------------- symmetric-key MAC */

/**
 * Symmetric MAC algorithms (spec §4.3).
 *
 * Each key is bound to one algorithm, chosen by the operator per key id. The
 * on-wire MAC field is `u32 keyid || digest`; the digest length disambiguates
 * the algorithm from the packet's trailing length per RFC 7822 §7.5.1, and the
 * key's configured algorithm must agree with it.
 *
 *   NTP_MAC_AES_CMAC_128    RFC 8573 AES-CMAC (AES-128). 16-octet digest,
 *                           20-octet MAC field. The current standard NTP MAC
 *                           and the default for new deployments; the key must
 *                           be exactly 16 octets.
 *   NTP_MAC_HMAC_SHA256_128 HMAC-SHA-256 truncated to 16 octets. Interop with
 *                           peers configured for a 128-bit SHA-2 MAC; 20-octet
 *                           field, same length class as AES-CMAC, distinguished
 *                           by the key's algorithm.
 *   NTP_MAC_HMAC_SHA256_160 HMAC-SHA-256 truncated to 20 octets. 24-octet
 *                           field, the same size as a legacy SHA-1 MAC.
 *
 * MD5 (RFC 5905 Appendix A) is not implemented at any length: it is broken for
 * authentication. Autokey (RFC 5906) is not implemented; NTS (core/nts) is the
 * modern authenticated path.
 *
 * Config note for the cfg-schema owner (group 0x0A security): the key table
 * needs a per-entry algorithm selector alongside the key id and material. A
 * `u8` mapping {0: AES-CMAC-128, 1: HMAC-SHA-256/128, 2: HMAC-SHA-256/160}
 * matching this enum is the minimal addition; core does not read cfg itself.
 */
typedef enum {
	NTP_MAC_AES_CMAC_128 = 0,
	NTP_MAC_HMAC_SHA256_128,
	NTP_MAC_HMAC_SHA256_160,
} ntp_mac_alg_t;

/** Symmetric keys held per context (spec §4.3). */
#define NTP_MAC_KEYS 16U
/** Longest symmetric key accepted: the SHA-256 block size. */
#define NTP_MAC_KEY_MAX 64U
/** AES-CMAC-128 requires exactly a 16-octet key. */
#define NTP_MAC_AES_KEY_LEN 16U
/** Digest of the two 128-bit algorithms: AES-CMAC-128 and HMAC-SHA-256/128. */
#define NTP_MAC_DIGEST_128 16U
/** Digest of HMAC-SHA-256/160. */
#define NTP_MAC_DIGEST_160 20U
/** Longest digest this server emits or accepts. */
#define NTP_MAC_DIGEST_MAX 20U
/** Largest MAC field produced: 4-octet key id + 20-octet digest. */
#define NTP_MAC_FIELD_MAX (4U + NTP_MAC_DIGEST_160)

/**
 * Per-client state slots. Must be a power of two. Holds both the rate-limit
 * token buckets and the interleaved-mode timestamp cache — one table, one
 * eviction policy, so a client cannot be present for one purpose and absent
 * for the other.
 */
#ifndef NTP_CLIENT_SLOTS
#define NTP_CLIENT_SLOTS 256U
#endif

/** Set associativity of that table: the probe window, and the eviction set. */
#ifndef NTP_CLIENT_WAYS
#define NTP_CLIENT_WAYS 4U
#endif

/** Largest token-bucket depth accepted, in requests (keeps the fixed-point safe). */
#define NTP_BURST_MAX 4000000U

typedef struct {
	/** Sustained requests per second per client; 0 disables the per-client bucket. */
	uint32_t client_rate;
	/** Per-client bucket depth in requests; clamped to [1, NTP_BURST_MAX]. */
	uint32_t client_burst;
	/** Sustained requests per second, all clients; 0 disables the global bucket. */
	uint32_t global_rate;
	/** Global bucket depth in requests; clamped to [1, NTP_BURST_MAX]. */
	uint32_t global_burst;
	/**
	 * Smallest interval between Kiss-o'-Death responses to one client, ms.
	 * An over-limit client is otherwise answered one-for-one, which turns the
	 * server into a full-rate reflector for a spoofed source address. Damping
	 * the KoD makes the excess traffic cost the attacker everything and the
	 * victim nothing. Over-limit requests inside the interval are dropped.
	 */
	uint32_t kod_min_interval_ms;
	/** Over the limit: true sends a KoD RATE (damped), false drops silently. */
	bool kod_on_limit;
	/**
	 * Answer interleaved requests in interleaved mode (RFC 9769).
	 *
	 * Off by default. Interleaved detection distinguishes an interleaved
	 * request from an ordinary RFC 5905 one solely by which of the previous
	 * response's timestamps the client echoed as its origin; getting that
	 * distinction wrong serves a basic client wildly incorrect time, so the
	 * feature is opt-in and gated on the RFC 9769 Figure-1 vector test
	 * (test_ntp.c) which pins both the positive and the basic-mode negative
	 * case. Enabling it also relies on ntp_tx_complete() being called with the
	 * per-response token from ntp_result_t.
	 */
	bool interleave;
	/** Answer at all while unsynchronised (the reply carries LI 3 / stratum 16). */
	bool serve_unsync;
} ntp_cfg_t;

/**
 * Defaults: 8 req/s burst 16 per client, 20000 req/s burst 40000 aggregate
 * (twice the spec §4.1 capacity target, so the global bucket is a safety valve
 * and not a policy), KoD on limit damped to 1 per second per client, interleave
 * OFF (opt-in — see ntp_cfg_t.interleave), serve while unsynchronised.
 */
void ntp_cfg_default(ntp_cfg_t *cfg);

/* ------------------------------------------------------------------- stats */

typedef struct {
	uint64_t rx;          /**< Datagrams handed to ntp_handle_request(). */
	uint64_t served;      /**< Ordinary mode-4 responses produced. */
	/**
	 * Requests discarded after passing the mode/version filter: malformed,
	 * rate-limited, authentication failure, no room. Disjoint from `ignored`,
	 * so rx == served + kod + dropped + ignored always holds.
	 */
	uint64_t dropped;
	uint64_t kod;         /**< Kiss-o'-Death responses produced. */
	uint64_t auth_fail;   /**< MAC present and unacceptable. */
	uint64_t interleaved; /**< Responses sent in interleaved mode. */
	uint64_t rate_limited;/**< Requests over a token bucket (KoD or dropped). */
	uint64_t ignored;     /**< Wrong mode or version, including modes 6 and 7. */
} ntp_stats_t;

/* ------------------------------------------------------------------ result */

typedef enum {
	NTP_ACT_IGNORE = 0, /**< Send nothing. */
	NTP_ACT_RESPOND,    /**< Send the mode-4 response in the output buffer. */
	NTP_ACT_KOD,        /**< Send the Kiss-o'-Death in the output buffer. */
} ntp_action_t;

/** Why a request produced no response. */
typedef enum {
	NTP_DROP_NONE = 0,
	NTP_DROP_SHORT,     /**< Under NTP_HDR_LEN, or over NTP_PKT_MAX. */
	NTP_DROP_VERSION,   /**< VN outside [3, 4]. */
	NTP_DROP_MODE,      /**< Not mode 3, or mode 6/7 (deliberately unserved). */
	NTP_DROP_RATE,      /**< Token bucket empty and kod_on_limit is false. */
	NTP_DROP_AUTH,      /**< MAC present, unknown key or bad digest. */
	NTP_DROP_UNSYNC,    /**< Unsynchronised and serve_unsync is false. */
	NTP_DROP_EXT,       /**< The extension hook refused to build a response. */
	NTP_DROP_NOSPACE,   /**< Output buffer too small for the response. */
	NTP_DROP_INTERNAL,  /**< A port primitive failed. */
} ntp_drop_t;

typedef struct {
	ntp_action_t action;
	ntp_drop_t drop;      /**< Set when action == NTP_ACT_IGNORE. */
	size_t len;           /**< Response octets written. */
	uint64_t xmt;         /**< Transmit-timestamp field written. */
	/**
	 * Interleave pairing token for ntp_tx_complete(), or 0 when this response
	 * armed no interleave state (interleave disabled, or a Kiss-o'-Death).
	 * The caller passes it back with the measured hardware transmit timestamp.
	 * It is per-client and per-response, so a later request from the same
	 * client that arrives before this response's timestamp does cannot cause
	 * that timestamp to be paired with the wrong exchange — the stale token is
	 * rejected rather than committed. See ntp_tx_complete().
	 */
	uint32_t xl_token;
	bool interleaved;     /**< The response used interleaved-mode timestamps. */
	bool authenticated;   /**< A symmetric MAC was verified and appended. */
} ntp_result_t;

/* -------------------------------------------------------- extension hook */

/**
 * Response extension-field builder, supplied by the caller.
 *
 * Called once the 48-octet header is in place and before any MAC, with @p len
 * at NTP_HDR_LEN, so `pkt[0..*len)` is exactly the associated data an NTS
 * authenticator has to cover (RFC 8915 §5.3). The hook appends complete
 * RFC 7822 extension fields and advances @p len. @p cap already accounts for
 * the anti-amplification budget and for MAC octets held in reserve, so the hook
 * may fill it entirely.
 *
 * This is how core/nts protects the datapath without an ntp↔nts dependency
 * edge (ARCHITECTURE.md §4): the Zephyr glue — or a host test — binds
 * nts_append_response() here.
 *
 * @param kod_refid  In/out, pre-set to 0. Setting it turns the response into a
 *                   Kiss-o'-Death carrying that reference identifier (LI 3,
 *                   stratum 0) with the appended fields intact — the NTS NAK of
 *                   RFC 8915 §5.7, which is a KoD 'NTSN' plus the echoed Unique
 *                   Identifier field. Because ntp rewrites those three header
 *                   octets *after* the hook returns, a hook that sets this MUST
 *                   NOT have appended anything that authenticates the header.
 *
 * @retval 0   Fields appended (possibly none).
 * @retval <0  Abort: the request gets no response and is counted as
 *             NTP_DROP_EXT. This is the NTS "silently discard" path.
 */
typedef struct {
	int (*build)(void *ctx, const uint8_t *req, size_t req_len,
		     uint8_t *pkt, size_t *len, size_t cap, uint32_t *kod_refid);
	void *ctx;
} ntp_ext_hook_t;

/* -------------------------------------------------------------- the server */

/**
 * Per-client rate-limit and interleave state. All fields private.
 *
 * The interleave fields hold the RFC 9769 client/server pairing. `xl_rx_sent`
 * is the receive-timestamp field the server put in the last committed response
 * to this client; `xl_tx_actual` is that response's measured hardware transmit
 * instant. A request is interleaved when it echoes `xl_rx_sent` as its origin
 * (a basic RFC 5905 client echoes the transmit field instead, and the server
 * guarantees those two are never equal — see the never-emit-xmt==rec rule in
 * ntp.c — so the two cases never collide). The `xl_pend_*`/`xl_gen` fields are
 * the response awaiting its transmit timestamp; the generation token makes a
 * late timestamp for a superseded response reject rather than mispair (M5).
 */
typedef struct {
	uint32_t id;
	bool used;
	int64_t last_seen_ms;
	uint32_t tokens_milli; /* 1000 milli-tokens == one request */
	int64_t tokens_ms;
	int64_t kod_last_ms;      /* last Kiss-o'-Death emitted to this client */
	bool kod_seen;
	/* Committed interleave pair, usable for exactly one interleaved reply. */
	uint64_t xl_rx_sent;      /* rec field of the last committed response */
	uint64_t xl_tx_actual;    /* measured transmit instant of that response */
	bool xl_valid;
	/* Response awaiting its hardware transmit timestamp (one slot + token). */
	uint64_t xl_pend_rx_sent;
	uint32_t xl_pend_token;
	bool xl_pend_active;
	uint32_t xl_gen;          /* per-client response generation counter */
} ntp_client_t;

/** One symmetric key. All fields private. */
typedef struct {
	uint32_t keyid;
	uint8_t key[NTP_MAC_KEY_MAX];
	uint8_t key_len;
	uint8_t alg; /* ntp_mac_alg_t */
	bool used;
} ntp_key_t;

/** Server context. Caller-owned; all fields private. */
typedef struct {
	ntp_cfg_t cfg;
	port_crypto_t crypto;
	const ntp_ext_hook_t *ext;
	ntp_key_t keys[NTP_MAC_KEYS];
	ntp_client_t clients[NTP_CLIENT_SLOTS];
	uint32_t g_tokens_milli;
	int64_t g_tokens_ms;
	uint32_t hash_seed; /* per-boot client-table hash salt (L14) */
	ntp_stats_t stats;
} ntp_ctx_t;

/**
 * Initialise a context.
 *
 * @param ctx     Context, zeroed and configured in place.
 * @param cfg     Configuration; NULL takes ntp_cfg_default(). Bucket depths are
 *                clamped into [1, NTP_BURST_MAX].
 * @param crypto  Crypto port. Required only for symmetric authentication —
 *                pass NULL to run without it, and any MAC-bearing request is
 *                then an auth failure. When present its rand() also salts the
 *                per-client hash table, so an attacker cannot craft a set of
 *                source addresses that all collide into one bucket (L14); a
 *                rand() failure is non-fatal and leaves the salt zero.
 * @param now_ms  Monotonic milliseconds; seeds the global bucket.
 *
 * @retval 0        Initialised.
 * @retval -EINVAL  @p ctx is NULL.
 */
int ntp_init(ntp_ctx_t *ctx, const ntp_cfg_t *cfg, const port_crypto_t *crypto,
	     int64_t now_ms);

/**
 * Install the response extension-field hook, or NULL to remove it. The hook
 * struct is borrowed, not copied, and must outlive @p ctx.
 *
 * @retval 0        Installed.
 * @retval -EINVAL  @p ctx is NULL.
 */
int ntp_set_ext_hook(ntp_ctx_t *ctx, const ntp_ext_hook_t *hook);

/**
 * Install or replace a symmetric key (spec §4.3).
 *
 * @param keyid    Key identifier; 0 is reserved by RFC 5905 and rejected.
 * @param alg      MAC algorithm bound to this key (ntp_mac_alg_t).
 * @param key      Key octets.
 * @param key_len  For NTP_MAC_AES_CMAC_128, exactly NTP_MAC_AES_KEY_LEN (16).
 *                 For the HMAC algorithms, 1..NTP_MAC_KEY_MAX.
 *
 * @retval 0        Stored.
 * @retval -EINVAL  NULL argument, keyid 0, an unknown @p alg, or a key length
 *                  the algorithm does not permit.
 * @retval -ENOSPC  All NTP_MAC_KEYS slots are in use by other key ids.
 */
int ntp_key_set(ntp_ctx_t *ctx, uint32_t keyid, ntp_mac_alg_t alg,
		const uint8_t *key, size_t key_len);

/**
 * Remove a symmetric key. The slot is wiped, not just marked free.
 *
 * @retval 0        Removed.
 * @retval -EINVAL  @p ctx is NULL.
 * @retval -ENOENT  No such key id.
 */
int ntp_key_clear(ntp_ctx_t *ctx, uint32_t keyid);

/** One received datagram plus everything the datapath needs about it. */
typedef struct {
	const uint8_t *pkt; /**< Datagram octets. */
	size_t len;         /**< Datagram length. */
	/**
	 * Opaque per-client key for the rate limiter and interleave cache.
	 *
	 * The caller MUST derive it from the source IP address ALONE — never the
	 * source port, and never any packet contents. RFC 9769 §5 and RFC 9109
	 * both say a server SHOULD NOT distinguish clients by port: a NAT or a
	 * client that re-binds its socket changes port between requests, which
	 * would split one client's interleave state and, worse, split its rate
	 * bucket so a single host could multiply its budget by cycling ports.
	 * Deriving from the address alone keeps one client to one bucket.
	 */
	uint32_t client_id;
	int64_t rx_tai_ns;  /**< Hardware receive timestamp (t6), TAI ns. */
	/**
	 * Best estimate of the transmit instant, TAI ns. Goes in the transmit
	 * field of a basic-mode response. In interleaved mode this value is not
	 * transmitted; the cached measured timestamp of the *previous* response
	 * is sent instead, and this exchange's real transmit timestamp is learned
	 * later via ntp_tx_complete().
	 */
	int64_t tx_tai_ns;
	int64_t now_ms;     /**< Monotonic milliseconds, for the token buckets. */
} ntp_rx_t;

/**
 * Handle one received datagram: validate, rate-limit, authenticate, and build
 * the response into @p out.
 *
 * Anti-amplification: the response is never longer than the request. The
 * extension hook's capacity is clipped to the request length minus any MAC
 * reserve, so trimming happens before the authenticated bytes are computed
 * rather than by truncating them afterwards.
 *
 * @param ctx      Server context.
 * @param rx       The datagram and its metadata.
 * @param q        Quality view; NULL is treated as unsynchronised.
 * @param out      Response buffer.
 * @param out_cap  Capacity of @p out.
 * @param res      Outcome. Always written when the return value is 0.
 *
 * @retval 0        Handled; @p res says whether anything is to be sent.
 * @retval -EINVAL  NULL argument, or @p rx->pkt is NULL with a non-zero length.
 */
int ntp_handle_request(ntp_ctx_t *ctx, const ntp_rx_t *rx,
		       const ntp_quality_view_t *q,
		       uint8_t *out, size_t out_cap, ntp_result_t *res);

/**
 * Record the measured transmit timestamp of a response, committing the
 * interleave pair for that client's next request (RFC 9769).
 *
 * Call this once the driver reports the hardware TX timestamp of the datagram
 * ntp_handle_request() produced, passing the @p token from that call's
 * ntp_result_t. Until it is called, the client has no committed pair and a
 * subsequent interleaved request falls back to a basic response — correct,
 * because the server has no precise transmit timestamp to report yet.
 *
 * The token guards against the realistic async case (SO_TIMESTAMPING delivers
 * the transmit timestamp later, via the socket error queue): if a second
 * request from the same client is handled before this call arrives, it arms a
 * new pending response with a new token, and this call — carrying the old
 * token — is rejected as stale rather than pairing this timestamp with the
 * newer exchange. The measurement for the superseded response is simply
 * dropped; that exchange just does not become interleave-eligible.
 *
 * @param client_id  The same value passed in ntp_rx_t.
 * @param token      ntp_result_t.xl_token from the corresponding response.
 * @param xmt_ntp    Measured transmit timestamp, NTP 64-bit format.
 *
 * @retval 0        Committed.
 * @retval -EINVAL  @p ctx is NULL, or @p token is 0.
 * @retval -ENOENT  No pending response for that client, or @p token is stale
 *                  (the client was evicted, the response was a KoD, interleave
 *                  is disabled, or a later request superseded this one).
 */
int ntp_tx_complete(ntp_ctx_t *ctx, uint32_t client_id, uint32_t token,
		    uint64_t xmt_ntp);

/** Snapshot the counters. @p out is untouched on -EINVAL. */
int ntp_stats_get(const ntp_ctx_t *ctx, ntp_stats_t *out);

/** Zero the counters. */
int ntp_stats_reset(ntp_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_NTP_NTP_H_ */
