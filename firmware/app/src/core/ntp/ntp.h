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
#include "quality/quality.h"

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
#define NTP_STRATUM_SMEAR  2U  /* leap smear in progress: no longer primary */
#define NTP_STRATUM_UNSYNC 16U /* unsynchronised */

/** Reference identifiers, host order, written big-endian on the wire. */
#define NTP_REFID_MAKE(a, b, c, d)                                            \
	(((uint32_t)(uint8_t)(a) << 24) | ((uint32_t)(uint8_t)(b) << 16) |    \
	 ((uint32_t)(uint8_t)(c) << 8) | (uint32_t)(uint8_t)(d))

#define NTP_REFID_GPS  NTP_REFID_MAKE('G', 'P', 'S', 0)    /* stratum-1 source */
#define NTP_REFID_INIT NTP_REFID_MAKE('I', 'N', 'I', 'T')  /* not yet synced */
#define NTP_REFID_SMER NTP_REFID_MAKE('S', 'M', 'E', 'R')  /* leap smear active */
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

/**
 * ntp_ts_from_tai() with a leap-smear correction applied first.
 *
 * @param tai_ns         As ntp_ts_from_tai().
 * @param tai_minus_utc  As ntp_ts_from_tai().
 * @param smear_ns       quality_smear_t::offset_ns — **subtracted** from
 *                       @p tai_ns before the conversion. 0 makes this exactly
 *                       ntp_ts_from_tai(), which is why the whole datapath can
 *                       call this unconditionally.
 *
 * Every UTC-bearing field of a response has to go through the same correction,
 * or the exchange is self-inconsistent: a client that receives a smeared
 * receive timestamp and an unsmeared transmit timestamp measures the smear
 * offset as round-trip delay. That includes the *interleaved* transmit
 * timestamp, which the platform converts separately in ntp_tx_complete() — see
 * the smear field the glue carries in its pending-transmit table.
 */
uint64_t ntp_ts_from_tai_smeared(int64_t tai_ns, int32_t tai_minus_utc,
				 int32_t smear_ns);

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
	/**
	 * Live leap-smear state (spec §15.2), or all-zero when smearing is off.
	 *
	 * Filled by ntp_quality_view_from_block() from the same snapshot the rest
	 * of the view came from, so the header fields and the timestamp
	 * correction can never describe different instants. `smear.offset_ns` is
	 * already folded into the advertisement (see the degradation table at
	 * ntp_quality_view_from_block()); the datapath applies it to the served
	 * timestamps. Carried here rather than kept private so the glue can log
	 * it and management surfaces can annunciate it.
	 */
	quality_smear_t smear;
} ntp_quality_view_t;

/** Fill @p q with the unsynchronised defaults (LI 3, stratum 16, precision −20). */
void ntp_quality_view_default(ntp_quality_view_t *q);

/** How far ahead of the event the leap indicator is advertised (RFC 5905 §7.3). */
#define NTP_LEAP_ANNOUNCE_WINDOW_S UINT64_C(86400)

/**
 * Project a §3.8 quality block onto the NTP view.
 *
 * The counterpart of ptp_quality_view_from_block(): the one place the §3.8
 * layout and the NTP header meet, so the glue does not hand-roll it and the
 * mapping is under host test. The mapping, with its reasoning:
 *
 * | NTP field       | Source |
 * |---|---|
 * | stratum/refid/root_delay/root_disp | copied; `disc` has already folded holdover into all four |
 * | tai_minus_utc   | `leap_current_s` |
 * | leap            | `leap_pending` sign, and only inside NTP_LEAP_ANNOUNCE_WINDOW_S of `leap_at_tai_s`; LI 0 otherwise |
 * | ref_tai_ns      | `now_tai_ns` less the block's age, so it stops advancing in holdover and a client can see the staleness |
 * | synchronized    | stratum is primary **and** @p time_traceable |
 * | smear           | quality_leap_smear() over the block, at @p smear_window_s |
 *
 * ### Leap smear (spec §15.2), when @p smear_window_s is non-zero
 *
 * Smearing is an NTP-only opt-in and it is **off by default**; a
 * GPS-disciplined stratum-1 reference that smears is deliberately serving a
 * UTC it knows to be wrong, and the same box is a PTP grandmaster, where IEEE
 * 1588 has no smear concept and the grandmaster must step. Nothing in this
 * function or below it can reach the PTP path: `ptp_quality_view_from_block()`
 * projects the same block through its own function, which reads only
 * `leap_pending` / `leap_at_tai_s` / `leap_current_s` / `utc_valid` — the
 * un-smeared fields — and the correction itself is applied nowhere but in the
 * three UTC-bearing header fields of an NTP response.
 *
 * Two things change while a window is configured, and a third while the ramp is
 * actually running:
 *
 *  - **The leap is never announced.** LI stays 0 for the whole announcement
 *    window, not merely for the ramp. RFC 8633 §3.7.1: a smearing server must
 *    not set the leap indicator. Announcing a step and then smearing it away
 *    is contradictory, and a client that implements both handles it worst.
 *  - **The stratum-1 claim is forfeit while the ramp runs** — stratum 1
 *    becomes NTP_STRATUM_SMEAR (2) and the reference identifier becomes
 *    'SMER'. A server intentionally serving an offset UTC is not a traceable
 *    primary reference, and for NTP the traceability assertion *is* stratum 1
 *    with refid 'GPS'. It is deliberately not demoted to stratum 16: that
 *    means "unsynchronised", RFC 5905 clients discard such a server outright,
 *    and a smearing server the clients discard delivers no smear at all — they
 *    fall back to something else and take the step, which is the exact outcome
 *    the operator opted out of. Stratum 2 + 'SMER' keeps the server usable
 *    while withdrawing the primary claim, and puts the state in front of every
 *    operator on the network in `ntpq -p`.
 *  - **Root dispersion grows by the instantaneous deviation.** The served
 *    timescale is exactly |smear.offset_ns| away from UTC, and dispersion is
 *    RFC 5905's maximum-error bound, so the deviation is added to it
 *    (saturating). That is the same mechanism §3.6 uses for holdover, and it
 *    is what makes a client's selection and combining algorithms weight this
 *    server correctly instead of trusting it as if it were still exact.
 *
 * @param b              The snapshot to project. Not retained.
 * @param now_tai_ns     Current TAI nanoseconds; 0 when no time is available,
 *                       which suppresses the leap announcement, the smear and
 *                       the reference timestamp rather than inventing any.
 * @param now_mono_ms    Current monotonic milliseconds, to age @p b.
 * @param time_traceable Whether the served timescale is actually traceable to
 *                       the primary reference. **This is a hard gate**: false
 *                       forces LI 3 / stratum 16 / refid 'INIT' however healthy
 *                       the discipline loop believes it is. The clock counter
 *                       the timestamps come from is placed on TAI by a separate
 *                       mechanism from the one that locks the oscillator, and a
 *                       locked oscillator on an unplaced counter serves
 *                       confidently wrong time (F1).
 * @param precision      Clock precision as log2 seconds, for the header.
 * @param smear_window_s Leap-smear window in seconds; 0 disables smearing
 *                       entirely, which is the default and leaves every field
 *                       below bit-identical to a build without the feature.
 *                       Pass ntp_smear_window() so the clamp has one authority.
 * @param out            Receives the view.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p b or @p out is NULL.
 */
int ntp_quality_view_from_block(const quality_block_t *b, uint64_t now_tai_ns,
				uint64_t now_mono_ms, bool time_traceable,
				int8_t precision, uint32_t smear_window_s,
				ntp_quality_view_t *out);

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
 * Smallest extension-field length this parser accepts, octets.
 *
 * RFC 7822 §7.5.1: in a packet with no MAC an extension field is at least 16
 * octets; beside a MAC it is at least 28. 16 is therefore the floor for any
 * field, and enforcing it is not pedantry — every 4-aligned length below it
 * that the walk accepts is another value a chosen key id can alias to, and the
 * low 16 bits of a key id are exactly what the length field reads as when a MAC
 * is mistaken for an extension field (M4/F6).
 */
#define NTP_EF_LEN_MIN 16U

/**
 * Parse a datagram into @p out. Performs no policy: version, mode and
 * authentication are the caller's business.
 *
 * Tail disambiguation follows RFC 7822 §7.5.1 and long-standing practice
 * (ntpd, chrony): a trailing remainder whose length is that of a MAC field
 * (key id + a recognised digest length) is a MAC, not an extension field —
 * extension fields alongside a MAC are ≥ 28 octets, so the short MAC lengths
 * are unambiguous.
 *
 * That decision is taken **before** the extension-field parse, for every
 * recognised MAC length including the three this server cannot verify
 * (36/52/68 = key id + a 32/48/64-octet digest). Trying the extension-field
 * parse first was a real hole: the length field of a candidate extension field
 * sits exactly where the low 16 bits of the key id are, so a client choosing
 * `keyid = 36` produced `flen = 36` — 4-aligned, within the remainder, and
 * therefore consumed as one extension field. The MAC vanished, @p mac_len came
 * back 0, and the request was answered unauthenticated (M4/F6).
 *
 * The cost of resolving the ambiguity this way is that a *final* extension
 * field of exactly 36, 52 or 68 octets is read as an unsupported MAC and the
 * request is rejected. Nothing this firmware speaks is shaped that way: an NTS
 * request's last field is the authenticator (40 octets with an empty
 * plaintext), its Unique Identifier (36) is never last, and its cookie and
 * placeholder fields are 104. A client that really does put a 36-octet
 * extension field last is indistinguishable from one presenting a SHA-256 MAC,
 * and the safe reading of an ambiguous authenticator is "authentication
 * failed".
 *
 * The extension-field walk is otherwise tolerant: it stops at the first field
 * that is short (< NTP_EF_LEN_MIN), unaligned or overruns, reports the octets
 * it did validate in @p ext_len, and ignores the remainder. Nothing beyond
 * ext_len is ever echoed.
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
	/**
	 * Leap-smear window in seconds; **0 (the default) means step**.
	 *
	 * The opt-in of spec §15.2, and NTP-only: nothing here is reachable from
	 * the PTP grandmaster or from the discipline loop. A non-zero value is
	 * clamped by ntp_init() into
	 * [QUALITY_SMEAR_WINDOW_MIN_S, QUALITY_SMEAR_WINDOW_MAX_S] — never to 0,
	 * so an operator who asked for a smear cannot be silently given a step.
	 * Read it back with ntp_smear_window().
	 *
	 * What it costs while a ramp is running is not incidental: the server
	 * gives up its stratum-1 claim and its 'GPS' reference identifier, and
	 * grows root dispersion by the deviation it is deliberately introducing.
	 * See ntp_quality_view_from_block().
	 */
	uint32_t smear_window_s;
} ntp_cfg_t;

/**
 * Defaults: 8 req/s burst 16 per client, 20000 req/s burst 40000 aggregate
 * (twice the spec §4.1 capacity target, so the global bucket is a safety valve
 * and not a policy), KoD on limit damped to 1 per second per client, interleave
 * OFF (opt-in — see ntp_cfg_t.interleave), serve while unsynchronised, leap
 * smear OFF (step at the boundary — see ntp_cfg_t.smear_window_s).
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

/** Octets of per-boot key material behind ntp_client_id(). */
#define NTP_CLIENT_ID_KEY_LEN 16U

/**
 * Backward window over which the transmit field is kept strictly increasing,
 * in units of the NTP 32.32 fraction (2^32 == one second). 2^32/50 == 20 ms.
 *
 * Why the field must be distinct at all: the platform demultiplexes the
 * hardware egress timestamp of a response by the transmit field that went on
 * the wire, because that is the only per-response value the driver's callback
 * can see (there is no MSG_ERRQUEUE in Zephyr). Two responses carrying the same
 * transmit field are therefore indistinguishable, and one client's measured
 * egress instant can be paired with another client's exchange and reported to
 * it as its own t3. That is not a narrow race: it happens for every pair of
 * responses built inside one tick of whatever clock the transmit estimate came
 * from, and the software fallback clock ticks at 1 ms (F5).
 *
 * So the server nudges the field forward by one LSB (233 ps) whenever it would
 * otherwise repeat or move backwards. The nudge is bounded to this window so
 * that a *legitimate* backward move larger than it — an era rollover, or a
 * servo step, which is only ever taken for an offset above 20 ms — resets the
 * register instead of pinning the served time to the pre-step value.
 */
#define NTP_XMT_DISTINCT_WINDOW (((UINT64_C(1) << 32U) / UINT64_C(50)))

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
	uint32_t hash_seed; /* per-boot client-table slot salt (L14) */
	/* Per-boot key for ntp_client_id(). Separate from hash_seed because the
	 * two protect different things: hash_seed only randomises which set an
	 * identity lands in, this keys the identity itself (F8). */
	uint8_t id_key[NTP_CLIENT_ID_KEY_LEN];
	/* Transmit field of the previous response, for the distinctness rule
	 * documented at NTP_XMT_DISTINCT_WINDOW. 0 = none issued yet. */
	uint64_t xmt_last;
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
 *                then an auth failure. When present its rand() also keys
 *                ntp_client_id() and salts the per-client hash table, so an
 *                attacker can neither craft a source address that shares a
 *                victim's identity (F8) nor a set that all collide into one
 *                bucket (L14); a rand() failure is non-fatal and leaves both at
 *                a documented fixed value.
 * @param now_ms  Monotonic milliseconds; seeds the global bucket.
 *
 * @retval 0        Initialised.
 * @retval -EINVAL  @p ctx is NULL.
 */
int ntp_init(ntp_ctx_t *ctx, const ntp_cfg_t *cfg, const port_crypto_t *crypto,
	     int64_t now_ms);

/**
 * Derive the ntp_rx_t::client_id of a source address.
 *
 * A **keyed** 32-bit tag (SipHash-2-4 over @p addr under the context's per-boot
 * key), truncated to 32 bits.
 *
 * Keyed, not merely hashed, and that distinction is the whole point. The
 * previous derivation was an unkeyed CRC-32 over the address. CRC-32 is affine
 * over GF(2), so for a 16-octet IPv6 address a second address colliding with a
 * chosen victim is *solved*, not searched — and a colliding source shares the
 * victim's token bucket and its interleave cache, so anyone with a routable /64
 * could drain a specific customer's rate budget into Kiss-o'-Death and silent
 * drops (F8). Salting only the table slot did not help: mix32() is a bijection,
 * so a slot salt applied after the CRC leaves `crc(a) == crc(b)` exactly as
 * easy to solve as before. The identity itself had to become key-dependent.
 *
 * IPv4 was never solvable — four octets through CRC-32 is a bijection — but it
 * goes through the same function so there is one derivation to reason about.
 *
 * Truncation to 32 bits keeps ntp_rx_t and ntp_tx_complete() unchanged. A
 * *random* collision with one chosen victim still needs ~2^32 addresses tried
 * against a rate-limited server; without the key, forging one was free.
 *
 * @param ctx       Initialised context; supplies the key. NULL yields 0.
 * @param addr      Raw address octets — 4 for IPv4, 16 for IPv6. Pass the
 *                  address alone: no port, no packet contents.
 * @param addr_len  Length of @p addr in octets.
 *
 * @return The identity, or 0 when @p ctx or @p addr is NULL or @p addr_len is 0.
 */
uint32_t ntp_client_id(const ntp_ctx_t *ctx, const void *addr, size_t addr_len);

/**
 * The effective leap-smear window in seconds after ntp_init()'s clamp; 0 when
 * smearing is off (the default) or @p ctx is NULL.
 *
 * Feed it to ntp_quality_view_from_block() so the clamp is applied in exactly
 * one place and the view can never be built against a window the datapath is
 * not configured for.
 */
uint32_t ntp_smear_window(const ntp_ctx_t *ctx);

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
	 *
	 * Use ntp_client_id(): the identity has to be a *keyed* function of the
	 * address, not merely a hashed one (F8).
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
