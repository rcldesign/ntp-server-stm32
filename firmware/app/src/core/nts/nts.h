/*
 * STS1000 "Meridian" — core/nts: Network Time Security (RFC 8915).
 *
 * Platform-neutral C11, no dynamic allocation. Two halves that share a master
 * key ring:
 *
 *   NTS-KE   the TLS-carried record protocol (RFC 8915 §4). This module owns
 *            the record codec and the server-side negotiation; the TLS session
 *            itself, including the exporter, is glue — supplied as a callback.
 *
 *   NTS-NTP  the extension fields that protect an ordinary NTPv4 exchange
 *            (RFC 8915 §5): parse and authenticate a request, issue fresh
 *            cookies in an authenticated-and-encrypted response.
 *
 * Deliberate architectural boundary: ARCHITECTURE.md §4 fixes the dependency
 * map as `ntp→util,quality` and `nts→util`, with no edge between ntp and nts.
 * So this module never includes ntp.h. It re-declares the two NTP wire
 * constants it needs (the 48-octet header length, and the RFC 7822 extension
 * field header shape) and walks extension fields itself. The cost is a second
 * 20-line walker; the benefit is that neither protocol module can quietly grow
 * a dependency on the other's internals, and each is testable alone.
 *
 * nts_ntp_ext_build() is declared with exactly the signature of
 * ntp_ext_hook_t::build so the glue can bind one to the other. The two headers
 * are independent by design, so nothing enforces that at compile time in this
 * module; the integration test in tests/host does, by assigning it.
 *
 * Cookie format is entirely a server-side choice — RFC 8915 §6 only suggests
 * one, and clients treat cookies as opaque. This server uses a fixed 100-octet
 * layout, documented at NTS_COOKIE_LEN below.
 */

#ifndef STS1000_CORE_NTS_NTS_H_
#define STS1000_CORE_NTS_NTS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nts/aes_siv.h"
#include "port/port_crypto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------ NTP wire bits */

/**
 * Length of the NTPv4 header, the offset at which extension fields start.
 * Duplicates NTP_HDR_LEN on purpose — see the dependency note above.
 */
#define NTS_NTP_HDR_LEN 48U

/** NTS extension field types (RFC 8915 §5.3–§5.6). */
#define NTS_EF_UNIQUE_ID 0x0104U
#define NTS_EF_COOKIE 0x0204U
#define NTS_EF_COOKIE_PLACEHOLDER 0x0304U
#define NTS_EF_AUTH 0x0404U

/** Shortest Unique Identifier the RFC allows (§5.3: at least 32 octets). */
#define NTS_UNIQ_MIN 32U
/** Longest Unique Identifier accepted; anything longer is a malformed request. */
#define NTS_UNIQ_MAX 64U

/**
 * NTS NAK kiss code 'NTSN' (RFC 8915 §5.7) as a host-order reference
 * identifier. Must equal NTP_REFID_NTSN in core/ntp/ntp.h — the two modules
 * share no header by design (see above), so the constant is written out on both
 * sides and pinned to this literal by both test suites.
 */
#define NTS_KISS_NTSN 0x4E54534EU

/* ------------------------------------------------------------------- AEAD */

/** AEAD_AES_SIV_CMAC_256, IANA numeric id 15. The only algorithm negotiated. */
#define NTS_AEAD_AES_SIV_CMAC_256 15U

/** Length of a C2S or S2C key for that AEAD. */
#define NTS_KEY_LEN AES_SIV_KEY_LEN

/** SIV nonce length used when sealing cookies and responses. */
#define NTS_NONCE_LEN 16U

/* ---------------------------------------------------------- master key ring */

/** Master key length. Also an AES-SIV-CMAC-256 key. */
#define NTS_MASTER_KEY_LEN AES_SIV_KEY_LEN

/**
 * Live master keys. Three is the working minimum for graceful rotation: the
 * key currently sealing cookies, the one before it (whose cookies are still in
 * flight), and one more so a rotation never invalidates the previous
 * generation the instant it happens.
 */
#ifndef NTS_MASTER_KEY_SLOTS
#define NTS_MASTER_KEY_SLOTS 3U
#endif

/** Default rotation interval: 24 hours, in milliseconds (spec §4.2). */
#define NTS_ROTATE_DEFAULT_MS INT64_C(86400000)

/** One master key. All fields private. */
typedef struct {
	uint8_t key[NTS_MASTER_KEY_LEN];
	int64_t created_ms;
	uint16_t id;
	bool valid;
} nts_master_key_t;

/** Master key ring. Caller-owned; all fields private. */
typedef struct {
	port_crypto_t crypto;
	nts_master_key_t keys[NTS_MASTER_KEY_SLOTS];
	int64_t last_rotate_ms;
	int64_t rotate_ms;
	uint16_t next_id;
	uint8_t current;
	bool ready;
} nts_keyring_t;

/**
 * Create a ring and generate its first key.
 *
 * @param crypto     Needs aes_ecb_encrypt and rand.
 * @param rotate_ms  Rotation interval; 0 selects NTS_ROTATE_DEFAULT_MS.
 *
 * @retval 0        Ready.
 * @retval -EINVAL  NULL argument, or a port missing a required primitive.
 * @retval -EIO     The entropy source failed.
 */
int nts_keyring_init(nts_keyring_t *r, const port_crypto_t *crypto,
		     int64_t now_ms, int64_t rotate_ms);

/**
 * Retire the oldest slot, generate a new key there and make it current.
 *
 * Cookies sealed under the keys still in the ring keep decrypting, which is
 * what makes rotation graceful: a client holding a cookie from two rotations
 * ago is served normally and gets a fresh one back. Beyond that its cookie
 * fails to unseal and it receives an NTS NAK, whereupon it re-runs NTS-KE.
 *
 * @retval 0        Rotated.
 * @retval -EINVAL  @p r is NULL or uninitialised.
 * @retval -EIO     The entropy source failed; the ring is unchanged.
 */
int nts_keyring_rotate(nts_keyring_t *r, int64_t now_ms);

/**
 * Rotate if the interval has elapsed. Call from the housekeeping loop.
 *
 * @retval 1        Rotated.
 * @retval 0        Not due.
 * @retval -EINVAL  @p r is NULL or uninitialised.
 * @retval -EIO     The entropy source failed.
 */
int nts_keyring_tick(nts_keyring_t *r, int64_t now_ms);

/**
 * Install a key, for restoring the ring from sealed storage at boot so cookies
 * survive a reboot inside their validity window (spec §4.2).
 *
 * @param make_current  Seal new cookies under this key from now on.
 *
 * @retval 0        Installed, replacing any slot that already held @p id.
 * @retval -EINVAL  NULL argument, uninitialised ring, or @p id is 0.
 */
int nts_keyring_install(nts_keyring_t *r, uint16_t id,
			const uint8_t key[NTS_MASTER_KEY_LEN], int64_t created_ms,
			bool make_current);

/**
 * Read back slot @p slot, for persisting the ring.
 *
 * @retval 0        Exported.
 * @retval -EINVAL  NULL ring, uninitialised, or @p slot out of range.
 * @retval -ENOENT  The slot is empty.
 */
int nts_keyring_export(const nts_keyring_t *r, size_t slot, uint16_t *id,
		       uint8_t key[NTS_MASTER_KEY_LEN], int64_t *created_ms,
		       bool *is_current);

/* ----------------------------------------------------------------- cookies */

/**
 * Cookie length, octets. Fixed, because exactly one AEAD is negotiated and its
 * keys are a fixed size — which lets the request-side placeholder arithmetic be
 * exact.
 *
 *   offset  size  field
 *      0      2   key id          (big endian) — which master key sealed this
 *      2      2   AEAD id         (big endian) — 15, AEAD_AES_SIV_CMAC_256
 *      4     16   nonce           — from the entropy port, per cookie
 *     20     16   SIV             — RFC 5297 synthetic IV over the below
 *     36     64   ciphertext      — C2S key ‖ S2C key, 32 octets each
 *
 * The 4-octet cleartext prefix is the AES-SIV associated data and the nonce is
 * the trailing associated-data component (RFC 5297 §3), so both are
 * authenticated: flipping a bit in the AEAD id or the nonce fails the unseal
 * rather than silently selecting a different interpretation. The key id is in
 * the clear because the server has to know which key to try before it can
 * decrypt anything, which is exactly what makes rotation work.
 *
 * SIV is the right AEAD for this: cookies are minted at a high rate and a
 * repeated nonce here costs only the observation that two cookies are equal,
 * where a repeated GCM nonce would leak the authentication key.
 */
#define NTS_COOKIE_LEN 100U

/** A cookie carried as an RFC 7822 extension field. */
#define NTS_COOKIE_EF_LEN (4U + NTS_COOKIE_LEN)

/** The key pair a cookie carries. */
typedef struct {
	uint16_t aead_id;
	uint8_t c2s[NTS_KEY_LEN];
	uint8_t s2c[NTS_KEY_LEN];
} nts_cookie_keys_t;

/**
 * Seal @p k into a cookie under the ring's current key.
 *
 * @retval 0        Sealed.
 * @retval -EINVAL  NULL argument, uninitialised ring, or an AEAD id this
 *                  server does not implement.
 * @retval -EIO     The entropy source or the AES port failed.
 */
int nts_cookie_seal(nts_keyring_t *r, const nts_cookie_keys_t *k,
		    uint8_t out[NTS_COOKIE_LEN]);

/**
 * Recover the keys from a cookie.
 *
 * @retval 0         Authentic; @p k is valid.
 * @retval -EINVAL   NULL argument or uninitialised ring.
 * @retval -EBADMSG  Wrong length, or the SIV does not verify.
 * @retval -ENOKEY   No live master key with that id — the usual, benign case
 *                   of a cookie that outlived its generation.
 * @retval -ENOTSUP  A well-formed cookie naming an AEAD this server does not
 *                   implement.
 * @retval -EIO      The AES port failed.
 */
int nts_cookie_unseal(const nts_keyring_t *r, const uint8_t *cookie, size_t len,
		      nts_cookie_keys_t *k);

/* -------------------------------------------------- NTS on the NTP datapath */

/** Cookies this server will put in one response. */
#ifndef NTS_COOKIES_MAX
#define NTS_COOKIES_MAX 8U
#endif

/**
 * Largest encrypted plaintext accepted in a *request*.
 *
 * RFC 8915 defines no extension field a client must send encrypted, so a
 * conforming request's ciphertext holds nothing but the SIV. The allowance
 * exists so a client that encrypts something harmless still authenticates;
 * anything larger is refused rather than given a buffer.
 */
#define NTS_REQ_PLAINTEXT_MAX 256U

/** What nts_process_request() decided. Non-negative return values. */
typedef enum {
	NTS_ACT_NONE = 0, /**< No NTS extension fields: an ordinary NTP request. */
	NTS_ACT_OK,       /**< Authenticated; build a protected response. */
	NTS_ACT_NAK,      /**< Cookie unusable: answer with a KoD 'NTSN'. */
	NTS_ACT_DROP,     /**< Malformed or unauthenticated: answer nothing. */
} nts_action_t;

/** Everything the response builder needs from the request. */
typedef struct {
	/** Unique Identifier to echo (RFC 8915 §5.3). */
	uint8_t uniq[NTS_UNIQ_MAX];
	size_t uniq_len;
	/** Keys recovered from the cookie. */
	nts_cookie_keys_t keys;
	/** Cookies to return: one per placeholder plus one for the one spent. */
	uint8_t cookies_wanted;
	/** True once uniq[] holds a valid echo — a NAK needs this and nothing else. */
	bool has_uniq;
	/** True once keys[] is authentic. */
	bool has_keys;
} nts_req_t;

typedef struct {
	uint64_t rx;             /**< Requests carrying NTS extension fields. */
	uint64_t ok;             /**< Authenticated and answered. */
	uint64_t nak;            /**< NTS NAKs produced. */
	uint64_t dropped;        /**< Malformed or failed authentication. */
	uint64_t cookies_issued; /**< Fresh cookies handed out. */
} nts_stats_t;

/** Datapath context. Caller-owned; all fields private. */
typedef struct {
	nts_keyring_t *ring;
	nts_stats_t stats;
	uint8_t max_cookies;
	bool ready;
} nts_ctx_t;

/**
 * Bind a datapath context to a key ring.
 *
 * @param max_cookies  Cookies per response; 0 selects NTS_COOKIES_MAX, and
 *                     larger values are clamped to it.
 *
 * @retval 0        Ready.
 * @retval -EINVAL  NULL argument or an uninitialised ring.
 */
int nts_init(nts_ctx_t *ctx, nts_keyring_t *ring, uint8_t max_cookies);

/**
 * Parse and authenticate the NTS extension fields of a request.
 *
 * The associated data for the AEAD is the whole packet up to the authenticator
 * field — header and every preceding extension field (RFC 8915 §5.3) — so a
 * modified NTP header, a swapped cookie or a reordered field all fail.
 *
 * Fields *after* the authenticator are ignored rather than rejected: they are
 * outside the authenticated region, so honouring them would be honouring
 * attacker input, and rejecting them would let an off-path attacker deny
 * service by appending a byte.
 *
 * Split between NAK and DROP: an unusable cookie earns a NAK, because that is
 * the case NTS NAK exists for — the client's cookie has aged past the key ring
 * and it needs to be told to re-run NTS-KE. A cookie that unseals cleanly
 * followed by an authenticator that does not verify is a forgery, and answering
 * it at all would confirm to the forger that the cookie is live, so it is
 * dropped. (RFC 8915 §5.7 permits a NAK for both; this is the narrower reading
 * chrony also takes.) A failure of the crypto port itself is also a drop: it is
 * the server's fault, and sending a NAK would send a healthy client off to
 * re-key while hiding the real fault behind a protocol message.
 *
 * Encrypted extension fields in the *request* are decrypted — they have to be,
 * to authenticate the packet — but their contents are then discarded. RFC 8915
 * defines no field a client must send encrypted, and inventing an
 * interpretation for one would be inventing attack surface. Anything longer
 * than NTS_REQ_PLAINTEXT_MAX is refused outright.
 *
 * @param out  Populated for NTS_ACT_OK and, as far as the echo, for NTS_ACT_NAK.
 *
 * @retval >=0      An nts_action_t.
 * @retval -EINVAL  NULL argument, uninitialised context, or a packet shorter
 *                  than the NTP header.
 */
int nts_process_request(nts_ctx_t *ctx, const uint8_t *pkt, size_t len,
			nts_req_t *out);

/**
 * Append the response extension fields: the echoed Unique Identifier, then an
 * authenticator over the response so far whose ciphertext carries fresh NTS
 * Cookie fields (RFC 8915 §5.7).
 *
 * The cookie count is trimmed to what @p cap allows, which is how the
 * anti-amplification budget reaches the crypto: the caller clips @p cap to the
 * request length, and the response is built to fit rather than truncated after
 * the fact.
 *
 * @param pkt   Response buffer with the NTP header already written.
 * @param len   In/out: current response length, advanced by what is appended.
 * @param cap   Total capacity available for the response.
 *
 * On any failure @p len is left as it was but the octets past it are
 * unspecified — the builder writes cookies into the packet before sealing them.
 * The caller must discard the response, which is what it would do anyway.
 *
 * @retval 0        Appended.
 * @retval -EINVAL  NULL argument, uninitialised context, or a request that was
 *                  never authenticated.
 * @retval -ENOSPC  @p cap cannot hold even the echo and an empty authenticator.
 * @retval -EIO     The entropy source or the AES port failed.
 */
int nts_append_response(nts_ctx_t *ctx, const nts_req_t *req, uint8_t *pkt,
			size_t *len, size_t cap);

/**
 * Append the extension fields of an NTS NAK: the echoed Unique Identifier and
 * nothing else. RFC 8915 §5.7 forbids cookies or an authenticator in a
 * Kiss-o'-Death, so the caller pairs this with a KoD carrying the 'NTSN' kiss
 * code.
 *
 * @retval 0        Appended.
 * @retval -EINVAL  NULL argument, or no echo was recovered.
 * @retval -ENOSPC  @p cap is too small.
 */
int nts_append_nak(nts_ctx_t *ctx, const nts_req_t *req, uint8_t *pkt,
		   size_t *len, size_t cap);

/**
 * Whole datapath in one callback, shaped to bind to ntp_ext_hook_t::build.
 *
 * Processes the request and appends either a protected response or a NAK's echo
 * field, setting @p kod_refid to 'NTSN' in the latter case. A request with no
 * NTS fields appends nothing and succeeds, so a server with the hook installed
 * still answers plain NTP.
 *
 * Interaction with rate limiting: core/ntp decides to send a Kiss-o'-Death for
 * an over-limit client *before* it calls the hook, so such a reply carries no
 * Unique Identifier. An NTS client is required to ignore a Kiss-o'-Death it
 * cannot match to a request (RFC 8915 §5.7), so a rate-limited NTS client sees
 * a silent drop. That is the right outcome — the alternative is a reply an
 * off-path attacker could have solicited on the client's behalf.
 *
 * @param ctx  An nts_ctx_t.
 *
 * @retval 0   Nothing to send is not an outcome here: either fields were
 *             appended or the request had none.
 * @retval <0  Drop the request without a response.
 */
int nts_ntp_ext_build(void *ctx, const uint8_t *req, size_t req_len, uint8_t *pkt,
		      size_t *len, size_t cap, uint32_t *kod_refid);

/** Snapshot the counters. */
int nts_stats_get(const nts_ctx_t *ctx, nts_stats_t *out);

/* ---------------------------------------------------------------- NTS-KE */

/** NTS-KE record types (RFC 8915 §4.1). */
#define NTSKE_REC_EOM 0U         /**< End of Message. */
#define NTSKE_REC_NEXT_PROTO 1U  /**< NTS Next Protocol Negotiation. */
#define NTSKE_REC_ERROR 2U       /**< Error. */
#define NTSKE_REC_WARNING 3U     /**< Warning. */
#define NTSKE_REC_AEAD 4U        /**< AEAD Algorithm Negotiation. */
#define NTSKE_REC_COOKIE 5U      /**< New Cookie for NTPv4. */
#define NTSKE_REC_SERVER 6U      /**< NTPv4 Server Negotiation. */
#define NTSKE_REC_PORT 7U        /**< NTPv4 Port Negotiation. */

/** NTS-KE error codes (RFC 8915 §4.1.3). */
#define NTSKE_ERR_UNRECOGNIZED_CRITICAL 0U
#define NTSKE_ERR_BAD_REQUEST 1U
#define NTSKE_ERR_INTERNAL 2U

/** Sentinel for "no error record was emitted". Not a wire value. */
#define NTSKE_NO_ERROR 0xFFFFU

/** Protocol ID for NTPv4 in a Next Protocol Negotiation record. */
#define NTSKE_PROTO_NTPV4 0U

/** RFC 8915 §4.3 TLS exporter label. */
#define NTSKE_EXPORTER_LABEL "EXPORTER-network-time-security"

/** Length of the RFC 8915 §4.3 exporter context. */
#define NTSKE_EXPORTER_CONTEXT_LEN 5U

/** Cookies a successful negotiation issues by default (RFC 8915 §4.1.6 SHOULD 8; 7 leaves headroom under a 1500-octet TLS record with a long server name). */
#define NTSKE_COOKIES_DEFAULT 7U

/** Cookies one negotiation may issue. */
#define NTSKE_COOKIES_MAX 8U

/** Suggested response buffer size: eight cookies plus every optional record. */
#define NTSKE_RSP_RECOMMENDED 1024U

/** One parsed NTS-KE record; @p body points into the caller's buffer. */
typedef struct {
	const uint8_t *body;
	uint16_t body_len;
	uint16_t type;
	bool critical;
} ntske_rec_t;

/**
 * Parse the record at @p off.
 *
 * @param next  Receives the offset of the following record.
 *
 * @retval 0         Parsed.
 * @retval -EINVAL   NULL argument.
 * @retval -EBADMSG  Truncated header or a body that runs past @p len.
 */
int ntske_rec_parse(const uint8_t *buf, size_t len, size_t off, ntske_rec_t *rec,
		    size_t *next);

/**
 * Append a record.
 *
 * @retval 0        Appended.
 * @retval -EINVAL  NULL argument, or a NULL body with a non-zero length.
 * @retval -ENOSPC  @p cap is too small.
 */
int ntske_rec_put(uint8_t *buf, size_t cap, size_t *len, bool critical,
		  uint16_t type, const uint8_t *body, uint16_t body_len);

/** Append a record whose body is a single big-endian uint16. */
int ntske_rec_put_u16(uint8_t *buf, size_t cap, size_t *len, bool critical,
		      uint16_t type, uint16_t value);

/**
 * TLS exporter, supplied by the glue (mbedTLS
 * `mbedtls_ssl_export_keying_material()` on target).
 *
 * @param label        NUL-terminated exporter label.
 * @param context      Exporter context; RFC 8915 §4.3 defines it as five
 *                     octets: protocol id (2), AEAD id (2), and 0x00 for the
 *                     C2S key or 0x01 for the S2C key.
 *
 * @retval 0   @p out holds @p out_len octets of keying material.
 * @retval <0  Failed; the negotiation answers with an Internal Server Error.
 */
typedef int (*nts_tls_export_fn)(void *ctx, const char *label,
				 const uint8_t *context, size_t context_len,
				 uint8_t *out, size_t out_len);

typedef struct {
	nts_keyring_t *ring;
	nts_tls_export_fn export_fn;
	void *export_ctx;
	/** NTPv4 Server Negotiation record to emit; NULL omits it. */
	const char *server_name;
	/** NTPv4 Port Negotiation record to emit; 0 omits it. */
	uint16_t port;
	/** Cookies to issue; 0 selects NTSKE_COOKIES_DEFAULT, clamped to the max. */
	uint8_t cookies;
} ntske_cfg_t;

/** NTS-KE context. Caller-owned; all fields private. */
typedef struct {
	ntske_cfg_t cfg;
	bool ready;
} ntske_ctx_t;

/** Outcome of one negotiation. */
typedef struct {
	/** Error code emitted, or NTSKE_NO_ERROR. */
	uint16_t error;
	/** AEAD selected, or 0 when none could be. */
	uint16_t aead_id;
	/** Cookies issued. */
	uint8_t cookies;
	/** True when the client's request named NTPv4 and the server accepted. */
	bool ok;
} ntske_result_t;

/**
 * Initialise an NTS-KE context.
 *
 * @retval 0        Ready.
 * @retval -EINVAL  NULL argument, no key ring, or no exporter.
 */
int ntske_init(ntske_ctx_t *ctx, const ntske_cfg_t *cfg);

/**
 * Run the server side of one NTS-KE exchange over an already-established TLS
 * session: parse the client's records, negotiate, and build the response.
 *
 * A negotiation that fails still produces a response — an Error record, or an
 * empty Next Protocol or AEAD record, per RFC 8915 §4.1.2 and §4.1.5 — so the
 * caller sends @p rsp either way and then closes the session. @p res says what
 * happened.
 *
 * @retval 0        @p rsp holds the response.
 * @retval -EINVAL  NULL argument or an uninitialised context.
 * @retval -ENOSPC  @p rsp_cap cannot hold even a minimal error response.
 */
int ntske_handle(ntske_ctx_t *ctx, const uint8_t *req, size_t req_len,
		 uint8_t *rsp, size_t rsp_cap, size_t *rsp_len,
		 ntske_result_t *res);

/**
 * Build the five-octet RFC 8915 §4.3 exporter context for one direction.
 *
 * @param c2s  True for the client-to-server key, false for server-to-client.
 *
 * @retval 0        Written.
 * @retval -EINVAL  @p out is NULL.
 */
int ntske_exporter_context(uint16_t proto_id, uint16_t aead_id, bool c2s,
			   uint8_t out[NTSKE_EXPORTER_CONTEXT_LEN]);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_NTS_NTS_H_ */
