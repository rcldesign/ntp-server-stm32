/*
 * STS1000 "Meridian" — the remote-syslog transport decision, RFC 5425 framing
 * and reconnect cadence, as pure logic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/net/, and deliberately free of every Zephyr dependency
 * so tests/host can compile it — the same arrangement as sts_ldap_ca.h and
 * sts_secops_policy.h, for the same reason: sts_syslog.c is Zephyr glue that no
 * host suite links, so a decision left inside it is a decision nothing can
 * catch when it is wrong. Every rule below used to be an `if` in that glue, and
 * one of those `if`s shipped an operator's cleartext.
 *
 * ---------------------------------------------------------------------------
 * WHY THIS EXISTS: `log.syslog.tls` USED TO MEAN "PLEASE SEND CLEARTEXT"
 * ---------------------------------------------------------------------------
 *
 * `log.syslog.tls` is a real, reachable BOOL in the schema (0x0905,
 * CFG_F_RUNTIME_APPLY) that the shell, REST and MCP config setters all write.
 * Until this header existed, setting it produced:
 *
 *     LOG_WRN("syslog TLS requested but not implemented; using UDP");
 *
 * ...once, locally, and then the sender carried on emitting RFC 5424 datagrams
 * in the clear to the collector. The old file header defended that as better
 * than "silently sending cleartext or silently sending nothing". It was neither
 * better nor different: the warning is local and the exposure is remote, so an
 * operator who asked for encryption believed the audit trail was protected
 * while it crossed the network readable. The failure of an unimplemented
 * feature must not be a downgrade of the property the feature exists to
 * provide.
 *
 * So the rule this header owns is FAIL CLOSED:
 *
 *     tls = true and the transport cannot be established
 *       -> no datagram, no fallback, no "degraded mode". The records stay in
 *          the logring behind an unadvanced cursor, and an ALARM is raised.
 *
 * There is no configuration that turns the fallback back on, because a
 * fallback that an operator can leave enabled by accident is the same bug with
 * a switch on it. An operator who genuinely wants unencrypted remote logging
 * says so by clearing `log.syslog.tls` — which is one key, in the open, where
 * an auditor can see it.
 *
 * ---------------------------------------------------------------------------
 * WHY "REFUSED" IS A SEPARATE OUTCOME FROM "OFF"
 * ---------------------------------------------------------------------------
 *
 * mbedTLS's *client* default is MBEDTLS_SSL_VERIFY_REQUIRED and Zephyr's
 * socket layer leaves it in force (sockets_tls.c only calls
 * mbedtls_ssl_conf_authmode() when TLS_PEER_VERIFY was set, which this code
 * deliberately never sets — see sts_ldap_ca.h for the full argument). With no
 * TLS_SEC_TAG_LIST there is no CA chain at all, so the handshake cannot
 * succeed. That is fail-closed already, but it is fail-closed as a bare
 * handshake error every 30 seconds forever, and the operator is left reading
 * mbedTLS return codes to discover that they never installed an anchor.
 *
 * STS_SYSLOG_TX_REFUSED is that case named BEFORE the socket is opened. It
 * costs one comparison and it is the difference between "TLS syslog is broken"
 * and "TLS syslog has no trust anchor; POST one to /api/v1/log/syslog/ca".
 *
 * ---------------------------------------------------------------------------
 * WHAT IS NOT HERE
 * ---------------------------------------------------------------------------
 *
 * TLS_PEER_VERIFY, for the reason above. An "insecure/skip-verify" mode, at
 * all: unvalidated TLS on a log feed authenticates nothing and encrypts to
 * whoever answered the SYN, which is a worse lie than plaintext because it
 * looks like protection. And any notion of a spool buffer — see
 * sts_syslog_advance_ok() for why the logring already is one.
 */

#ifndef STS1000_ZEPHYR_NET_STS_SYSLOG_TLS_POLICY_H_
#define STS1000_ZEPHYR_NET_STS_SYSLOG_TLS_POLICY_H_

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------- trust anchor */

/**
 * Zephyr TLS credential tag for the syslog collector's trust anchor.
 *
 * Distinct from STS_LDAP_CA_SEC_TAG (0x4C444131, "LDA1", sts_ldap_ca.h) and
 * STS_WEB_SEC_TAG (0x57454231, "WEB1", sts_web.h), and deliberately so: the CA
 * that vouches for the directory which names this box's administrators is not
 * the CA that vouches for a log collector. Sharing one anchor between them
 * would mean an operator who trusts a log sink has, silently, also trusted it
 * to sign an LDAPS server.
 */
#define STS_SYSLOG_CA_SEC_TAG 0x534C4731 /* "SLG1" */

/** Same /lfs/tls directory sts_cert.c and sts_aaa.c use. */
#define STS_SYSLOG_CA_DIR "/lfs/tls"
#define STS_SYSLOG_CA_PATH STS_SYSLOG_CA_DIR "/syslog_ca.pem"

/**
 * Largest anchor accepted, in octets. Matched to STS_LDAP_CA_PEM_MAX so the two
 * anchors have the same ceiling and one screening function (sts_ldap_ca_check)
 * serves both; a two-certificate intermediate chain in PEM is ~2.1 KB.
 */
#define STS_SYSLOG_CA_PEM_MAX 3072U

/* ---------------------------------------------------------------- transport */

/** What `log.syslog.*` resolves to. Exactly one of these, never a blend. */
typedef enum {
	/** Not enabled, or no host configured. Nothing is sent, nothing is
	 *  wrong, no alarm. */
	STS_SYSLOG_TX_OFF = 0,

	/** `log.syslog.tls` clear: RFC 5424 over UDP, as before. The operator
	 *  asked for cleartext explicitly and gets exactly that. */
	STS_SYSLOG_TX_UDP,

	/** `log.syslog.tls` set and an anchor is installed: RFC 5425 over
	 *  TCP+TLS, octet-counted. */
	STS_SYSLOG_TX_TLS,

	/** `log.syslog.tls` set and something makes TLS impossible. NOTHING is
	 *  sent — this is the outcome that used to be STS_SYSLOG_TX_UDP. */
	STS_SYSLOG_TX_REFUSED,
} sts_syslog_tx_t;

/** Inputs to the transport decision, all from cfg plus one runtime fact. */
typedef struct {
	bool enabled;   /**< log.syslog.en */
	bool tls;       /**< log.syslog.tls */
	bool host_set;  /**< log.syslog.host is non-empty */
	bool ca_ready;  /**< an anchor is parsed and registered under
			 *   STS_SYSLOG_CA_SEC_TAG */
	bool tls_built; /**< this image has a TLS socket layer at all */
} sts_syslog_tx_in_t;

/**
 * Resolve the transport.
 *
 * The single rule worth stating in prose: there is no input combination for
 * which @c tls is true and the result is STS_SYSLOG_TX_UDP. That is the whole
 * fail-closed property, and test_syslog_tls.c enumerates the input space to
 * prove it rather than spot-checking cases.
 */
static inline sts_syslog_tx_t sts_syslog_tx(const sts_syslog_tx_in_t *in)
{
	if (in == NULL || !in->enabled || !in->host_set) {
		return STS_SYSLOG_TX_OFF;
	}
	if (!in->tls) {
		return STS_SYSLOG_TX_UDP;
	}
	if (!in->tls_built || !in->ca_ready) {
		return STS_SYSLOG_TX_REFUSED;
	}
	return STS_SYSLOG_TX_TLS;
}

/** Why a STS_SYSLOG_TX_REFUSED is refused; NULL for every other outcome. */
static inline const char *sts_syslog_refusal(const sts_syslog_tx_in_t *in)
{
	if (in == NULL || sts_syslog_tx(in) != STS_SYSLOG_TX_REFUSED) {
		return NULL;
	}
	if (!in->tls_built) {
		return "log.syslog.tls is set but this image has no TLS socket "
		       "layer; refusing to send cleartext";
	}
	return "log.syslog.tls is set but no collector trust anchor is "
	       "installed; POST one to /api/v1/log/syslog/ca";
}

/** True when the transport is a stream and therefore octet-counted. */
static inline bool sts_syslog_tx_is_stream(sts_syslog_tx_t t)
{
	return t == STS_SYSLOG_TX_TLS;
}

/**
 * True when this transport must raise the operator-visible alarm while it is
 * not carrying records.
 *
 * REFUSED alarms immediately and unconditionally: the operator asked for
 * encrypted remote logging and is not getting any remote logging, and the one
 * thing that must not happen is for that to be discoverable only by reading the
 * local log — which is the very facility that is broken.
 *
 * TLS alarms only once @p connected is false, because a session that is up is
 * doing its job. UDP never alarms: it has no delivery signal to alarm on, and
 * it was never promised to be anything.
 */
static inline bool sts_syslog_alarm(sts_syslog_tx_t t, bool connected)
{
	switch (t) {
	case STS_SYSLOG_TX_REFUSED:
		return true;
	case STS_SYSLOG_TX_TLS:
		return !connected;
	case STS_SYSLOG_TX_OFF:
	case STS_SYSLOG_TX_UDP:
	default:
		return false;
	}
}

/* ---------------------------------------------------------------- the port */

/**
 * The UDP syslog port, and the schema default of `log.syslog.port`.
 */
#define STS_SYSLOG_PORT_UDP 514U

/** RFC 5425 §4.1: "syslog-conn" over TLS is registered on 6514. */
#define STS_SYSLOG_PORT_TLS 6514U

/**
 * Effective destination port.
 *
 * A collector that speaks RFC 5425 does not listen on 514 — 514 is the UDP
 * datagram service, and RFC 5425 §4.1 assigns TLS its own port precisely
 * because the two protocols are not interchangeable on one port. But
 * `log.syslog.port` defaults to 514 and an operator who flips `log.syslog.tls`
 * will not usually think to change it, so honouring 514 literally would turn
 * "enable TLS" into "connect forever to a port nothing answers on" — a
 * fail-closed outcome that is technically correct and practically a support
 * call.
 *
 * So: TLS with the port left at its schema default means 6514. TLS with ANY
 * other port means that port, because an operator who typed a number meant it.
 * The substitution is confined to this function so it is stated once and tested
 * once, rather than being a surprise inside the connect path.
 */
static inline uint16_t sts_syslog_port(bool tls, uint16_t cfg_port)
{
	if (tls && cfg_port == (uint16_t)STS_SYSLOG_PORT_UDP) {
		return (uint16_t)STS_SYSLOG_PORT_TLS;
	}
	return cfg_port;
}

/* ------------------------------------------------------- RFC 5425 framing */

/**
 * Overhead sts_syslog_frame() adds: up to 5 digits of MSG-LEN plus one SP.
 * 5 digits covers any SYSLOG-MSG up to 99999 octets; LOGR_RENDER_MAX is far
 * below that and sts_syslog_frame() range-checks anyway.
 */
#define STS_SYSLOG_FRAME_OVERHEAD 6U

/**
 * Longest SYSLOG-MSG this framer will accept. RFC 5425 §4.3.1 requires a
 * receiver to support at least 2048 octets and permits more by agreement; a
 * sender that emits more than the guaranteed minimum is relying on agreement it
 * has not made, so the framer stops here and the caller truncates instead.
 */
#define STS_SYSLOG_MSG_MAX 2048U

/**
 * Octet-count a SYSLOG-MSG per RFC 5425 §4.3: `MSG-LEN SP SYSLOG-MSG`.
 *
 * MSG-LEN is the octet count of SYSLOG-MSG alone — it does not include itself,
 * it does not include the space, and it does not include any NUL. It is decimal
 * ASCII with NO leading zeros (§4.3.1), which is why this writes the digits by
 * division rather than through a printf-family conversion that a width or a
 * flag could quietly change.
 *
 * This exists as a function, tested on the host, because the transport-framing
 * bug class is silent at the sender and catastrophic at the receiver: a
 * MSG-LEN that is one too small leaves the last octet of message N sitting at
 * the front of message N+1's length field, and every subsequent record on that
 * connection is garbage or is rejected. Nothing in this firmware can observe
 * that. A test can.
 *
 * @param msg      SYSLOG-MSG octets (not NUL-terminated by requirement).
 * @param msg_len  Its length in octets. Must be non-zero — RFC 5424 has no
 *                 empty message and a "0 " frame is a receiver's parse error.
 * @param out      Destination.
 * @param cap      Capacity of @p out, in octets.
 * @param out_len  Receives the framed length on success.
 *
 * @retval 0        Framed.
 * @retval -EINVAL  NULL argument, zero @p msg_len, or @p msg_len over
 *                  STS_SYSLOG_MSG_MAX.
 * @retval -ENOSPC  @p cap cannot hold the frame.
 */
static inline int sts_syslog_frame(const uint8_t *msg, size_t msg_len,
				   uint8_t *out, size_t cap, size_t *out_len)
{
	char digits[8];
	size_t ndig = 0U;
	size_t v;
	size_t i;

	if (msg == NULL || out == NULL || out_len == NULL) {
		return -EINVAL;
	}
	if (msg_len == 0U || msg_len > (size_t)STS_SYSLOG_MSG_MAX) {
		return -EINVAL;
	}

	/* Decimal, most-significant first, no leading zero: build the digits
	 * backwards then reverse. msg_len is >= 1 so the do/while always emits
	 * at least one digit and never emits "0". */
	v = msg_len;
	do {
		digits[ndig++] = (char)('0' + (int)(v % 10U));
		v /= 10U;
	} while (v != 0U && ndig < sizeof(digits));

	if (cap < ndig + 1U + msg_len) {
		return -ENOSPC;
	}

	for (i = 0U; i < ndig; i++) {
		out[i] = (uint8_t)digits[ndig - 1U - i];
	}
	out[ndig] = (uint8_t)' ';
	for (i = 0U; i < msg_len; i++) {
		out[ndig + 1U + i] = msg[i];
	}

	*out_len = ndig + 1U + msg_len;
	return 0;
}

/* ------------------------------------------------------- cursor advancement */

/**
 * May the syslog reader's logring cursor advance past a record that was just
 * handed to the transport?
 *
 * THIS IS THE SPOOL, and it is why there is no spool buffer anywhere in this
 * feature. logring.h is a multi-reader ring — "a record stays until it is
 * overwritten, so several readers advance independently. A cursor is a sequence
 * number." The syslog sender owns one such cursor. Not advancing it IS
 * spooling: the records remain in the ring, they are re-read on the next drain,
 * and logr_tail() reports as a gap count anything the console area's writers
 * overwrote in the meantime. A separate spool would duplicate that storage out
 * of the same 640 KB and add a second, worse loss path.
 *
 * The asymmetry between the transports is not an oversight:
 *
 *   UDP    advances unconditionally. sendto() reports that the datagram left
 *          the stack, never that it arrived; holding the cursor on a UDP error
 *          would wedge the reader against a host that is simply gone, and would
 *          convert a lossy transport into a stalled one.
 *
 *   TLS    advances only on a completed write. A stream transport DOES have a
 *          delivery signal, so a record that was not written is a record that
 *          must be written again — and, crucially, a session that dropped
 *          mid-frame must not leave a half-frame acknowledged, because
 *          octet-counted framing has no resynchronisation point.
 */
static inline bool sts_syslog_advance_ok(sts_syslog_tx_t t, bool write_ok)
{
	if (sts_syslog_tx_is_stream(t)) {
		return write_ok;
	}
	return true;
}

/* -------------------------------------------------------- reconnect cadence */

/** First retry delay after a session drops or a connect fails, in ms. */
#define STS_SYSLOG_BACKOFF_MIN_MS 1000U

/**
 * Ceiling on the retry delay, in ms.
 *
 * 60 s: long enough that an unreachable collector costs a connect attempt a
 * minute rather than a TLS handshake every poll, short enough that a collector
 * coming back is picked up within a minute. The logring holds far more than a
 * minute of records at any plausible rate, so the ceiling does not by itself
 * cause loss.
 */
#define STS_SYSLOG_BACKOFF_MAX_MS 60000U

/**
 * Delay before connect attempt number @p fails + 1, in ms.
 *
 * Doubling from STS_SYSLOG_BACKOFF_MIN_MS, saturating at
 * STS_SYSLOG_BACKOFF_MAX_MS. @p fails is the count of consecutive failures so
 * far, so 0 -> the minimum, not zero: there is deliberately no "retry
 * immediately" case, because the first failure is exactly when an immediate
 * retry would spin. A TLS connect costs a TCP handshake, an ECDHE and a chain
 * verification; spinning on it against a refused port is a CPU-bound loop on a
 * box whose entire purpose is a disciplined oscillator.
 *
 * Deterministic — no jitter. One board is not a thundering herd, and a
 * reproducible cadence is one a bench test can assert.
 */
static inline uint32_t sts_syslog_backoff_ms(uint32_t fails)
{
	uint32_t ms = STS_SYSLOG_BACKOFF_MIN_MS;
	uint32_t i;

	for (i = 0U; i < fails; i++) {
		if (ms >= STS_SYSLOG_BACKOFF_MAX_MS / 2U) {
			return STS_SYSLOG_BACKOFF_MAX_MS;
		}
		ms *= 2U;
	}
	return ms;
}

/**
 * Has the backoff for @p fails failures elapsed?
 *
 * Subtraction on unsigned ms, so it is correct across the 32-bit wrap that
 * k_uptime_get_32() performs every 49.7 days — `now - since` is the true
 * elapsed time whatever the wrap, whereas `since + delay <= now` is not.
 */
static inline bool sts_syslog_retry_due(uint32_t now_ms, uint32_t since_ms,
				        uint32_t fails)
{
	return (uint32_t)(now_ms - since_ms) >= sts_syslog_backoff_ms(fails);
}

/* ------------------------------------------------------------ time budgets */

/**
 * Bound on how long one connect attempt may occupy its thread, in ms.
 *
 * Zephyr's ztls_connect_ctx() (subsys/net/lib/sockets/sockets_tls.c) is
 * TWO serialised blocking phases, each bounded by the same global Kconfig:
 *
 *     zsock_connect(ctx->sock, ...)                  <= NET_SOCKETS_CONNECT_TIMEOUT
 *     tls_mbedtls_handshake(ctx, K_MSEC(NET_SOCKETS_CONNECT_TIMEOUT))
 *
 * so the worst case is 2 x CONFIG_NET_SOCKETS_CONNECT_TIMEOUT, and SO_SNDTIMEO
 * does not shorten either one (sockets_inet.c uses the Kconfig value directly,
 * and the handshake's own TODO notes it blocks even a non-blocking socket).
 * At the Zephyr default of 3000 ms that is 6000 ms, which EXCEEDS
 * CONFIG_STS1000_LIVENESS_DEADLINE_MS (5000 ms).
 *
 * That is the reason connect runs on its own thread rather than on the syslog
 * drain thread: a liveness participant that blocks for 6 s stops the supervisor
 * kicking the TPS3430, and a grandmaster clock cold-cycles because a log
 * collector stopped answering. sts_syslog.c BUILD_ASSERTs this relationship so
 * that raising the Zephyr timeout, or lowering the liveness deadline, fails the
 * build instead of the board.
 */
#define STS_SYSLOG_CONNECT_BUDGET(connect_tmo_ms) (2U * (connect_tmo_ms))

/**
 * Is it safe to run a TLS connect inline on a liveness participant whose
 * deadline is @p deadline_ms, given Zephyr's connect timeout?
 *
 * Spelled out because the answer is arithmetic on two Kconfigs that different
 * people change for different reasons, and the consequence of it silently
 * becoming true is a board that reboots when a log collector goes away.
 *
 * The macro exists so sts_syslog.c can BUILD_ASSERT on it — a call to a
 * `static inline` is not a constant expression, so the check would have to be
 * dropped or deferred to runtime, which is exactly when it is useless. The
 * inline wrapper below is the same expression, so the host suite and the build
 * assert cannot drift apart.
 */
#define STS_SYSLOG_CONNECT_FITS_LIVENESS(connect_tmo_ms, poll_ms, deadline_ms) \
	((STS_SYSLOG_CONNECT_BUDGET(connect_tmo_ms) + (poll_ms)) <            \
	 (deadline_ms))

static inline bool sts_syslog_connect_fits_liveness(uint32_t connect_tmo_ms,
						    uint32_t poll_ms,
						    uint32_t deadline_ms)
{
	return STS_SYSLOG_CONNECT_FITS_LIVENESS(connect_tmo_ms, poll_ms,
						deadline_ms);
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_NET_STS_SYSLOG_TLS_POLICY_H_ */
