/*
 * STS1000 "Meridian" — core/auth: RADIUS client codec (RFC 2865, RFC 3579).
 *
 * Platform-neutral C11, no dynamic allocation. Builds Access-Request datagrams
 * and parses Access-Accept / Access-Reject / Access-Challenge; the UDP socket,
 * the retry timer and the destination live in the glue
 * (`src/zephyr/net/sts_aaa.c`).
 *
 * ---------------------------------------------------------------------------
 * Verification is not optional
 * ---------------------------------------------------------------------------
 *
 * A RADIUS reply is a bare UDP datagram, so anything on the path can forge an
 * Access-Accept. The only thing that makes the protocol usable is the Response
 * Authenticator — MD5(Code ‖ ID ‖ Length ‖ RequestAuth ‖ Attributes ‖ Secret) —
 * and @ref radius_parse_response *always* checks it. There is no flag to skip
 * it, because a client that can be configured to skip it eventually is.
 *
 * @ref radius_parse_response also verifies a Message-Authenticator (RFC 3579
 * §3.2, HMAC-MD5) when the reply carries one, and rejects a reply whose
 * attribute list is malformed rather than parsing what it can.
 *
 * ---------------------------------------------------------------------------
 * Role mapping
 * ---------------------------------------------------------------------------
 *
 * Filter-Id (attribute 11) is checked first and its string parsed by
 * @ref auth_role_parse, so a server can return "admin"/"operator"/"viewer" or
 * the usual synonyms. Failing that, Service-Type (attribute 6) maps:
 *
 *   Administrative(6)      -> admin
 *   NAS-Prompt(7)          -> operator
 *   Login(1) / Framed(2)   -> viewer
 *
 * A reply that says neither yields AUTH_ROLE_VIEWER: an authenticated user with
 * no stated authorisation gets the least privilege, never the most.
 */

#ifndef STS1000_CORE_AUTH_RADIUS_H_
#define STS1000_CORE_AUTH_RADIUS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "auth/auth.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ limits */

/** Fixed header: code, id, length(2), authenticator(16). */
#define RADIUS_HDR_LEN 20U
#define RADIUS_AUTH_LEN 16U

/** RFC 2865 §3: a datagram is 20..4096 octets. */
#define RADIUS_PKT_MIN 20U
#define RADIUS_PKT_MAX 4096U

/**
 * Largest datagram this client builds or accepts.
 *
 * An Access-Request from this device carries a username, a hidden password, a
 * NAS-Identifier and four small attributes — a few hundred octets. 512 keeps
 * the whole exchange inside one Ethernet frame and bounds the stack buffers the
 * glue needs; a server reply longer than that is refused rather than truncated.
 */
#ifndef RADIUS_BUF_MAX
#define RADIUS_BUF_MAX 512U
#endif

/** Longest State / Class blob echoed through a challenge round. */
#define RADIUS_STATE_MAX 64U
/** Longest Reply-Message captured, excluding the NUL. */
#define RADIUS_REPLY_MAX 63U

/* ------------------------------------------------------------------- codes */

#define RADIUS_CODE_ACCESS_REQUEST 1U
#define RADIUS_CODE_ACCESS_ACCEPT 2U
#define RADIUS_CODE_ACCESS_REJECT 3U
#define RADIUS_CODE_ACCESS_CHALLENGE 11U

/* -------------------------------------------------------------- attributes */

#define RADIUS_AT_USER_NAME 1U
#define RADIUS_AT_USER_PASSWORD 2U
#define RADIUS_AT_NAS_IP_ADDRESS 4U
#define RADIUS_AT_NAS_PORT 5U
#define RADIUS_AT_SERVICE_TYPE 6U
#define RADIUS_AT_FILTER_ID 11U
#define RADIUS_AT_REPLY_MESSAGE 18U
#define RADIUS_AT_STATE 24U
#define RADIUS_AT_CLASS 25U
#define RADIUS_AT_SESSION_TIMEOUT 27U
#define RADIUS_AT_NAS_IDENTIFIER 32U
#define RADIUS_AT_NAS_PORT_TYPE 61U
#define RADIUS_AT_MESSAGE_AUTHENTICATOR 80U

/* Service-Type values used by the role mapping (RFC 2865 §5.6). */
#define RADIUS_ST_LOGIN 1U
#define RADIUS_ST_FRAMED 2U
#define RADIUS_ST_ADMINISTRATIVE 6U
#define RADIUS_ST_NAS_PROMPT 7U

/* NAS-Port-Type values (RFC 2865 §5.41). Virtual is what a management session is. */
#define RADIUS_NPT_VIRTUAL 5U

/* ------------------------------------------------------------- primitives */

/**
 * RFC 2865 §5.2 User-Password hiding.
 *
 * `b1 = MD5(secret ‖ RequestAuth); c1 = p1 ⊕ b1;`
 * `bN = MD5(secret ‖ c(N-1));      cN = pN ⊕ bN`
 *
 * with the password zero-padded up to a whole number of 16-octet chunks. The
 * padding is why an empty password still produces 16 octets: the attribute is
 * never zero-length on the wire.
 *
 * @param out      Receives 16..128 octets.
 * @param out_len  Receives the length written.
 *
 * @retval 0        Written.
 * @retval -EINVAL  NULL argument, a password over AUTH_SECRET_MAX, or no
 *                  shared secret.
 * @retval -ENOSPC  @p cap too small.
 */
int radius_hide_password(const uint8_t *shared, size_t shared_len,
			 const uint8_t authenticator[RADIUS_AUTH_LEN],
			 const char *password, uint8_t *out, size_t cap,
			 size_t *out_len);

/**
 * Compute the Response Authenticator a reply should carry.
 *
 * MD5 over the reply's own Code, Identifier and Length, then the *request's*
 * authenticator, then the reply's attributes, then the shared secret.
 *
 * @param pkt   The reply as received; its authenticator field is ignored.
 * @param out   Receives RADIUS_AUTH_LEN octets.
 */
int radius_response_auth(const uint8_t *pkt, size_t len,
			 const uint8_t req_auth[RADIUS_AUTH_LEN],
			 const uint8_t *shared, size_t shared_len,
			 uint8_t out[RADIUS_AUTH_LEN]);

/**
 * Walk the attribute list, checking as it goes.
 *
 * @param type     Attribute type to find.
 * @param skip     Occurrences to pass over before returning one.
 * @param val      Receives a pointer into @p pkt.
 * @param val_len  Receives the value length (may be 0).
 *
 * @retval 0         Found.
 * @retval -ENOENT   No such attribute.
 * @retval -EBADMSG  A length octet under 2, or an attribute running past the
 *                   declared packet length.
 * @retval -EINVAL   NULL argument or a packet shorter than the header.
 */
int radius_attr_find(const uint8_t *pkt, size_t len, uint8_t type, size_t skip,
		     const uint8_t **val, size_t *val_len);

/** Validate the whole attribute list. @retval 0 well formed, else -EBADMSG. */
int radius_attrs_check(const uint8_t *pkt, size_t len);

/**
 * Derive an auth_role_t from a reply's attributes. Never fails: a reply with
 * nothing to say maps to AUTH_ROLE_VIEWER.
 *
 * @param out_explicit  Optional; set true when an attribute actually named a
 *                      role, so a caller can log the difference between
 *                      "server said viewer" and "server said nothing".
 */
uint8_t radius_role_from_attrs(const uint8_t *pkt, size_t len,
			       bool *out_explicit);

/* ------------------------------------------------------------------ request */

/** Everything an Access-Request needs. Strings are borrowed. */
typedef struct {
	const char *user;   /**< User-Name. Required.                        */
	const char *secret; /**< Password to hide. Required.                 */
	const uint8_t *shared; /**< Shared secret. Required.                 */
	size_t shared_len;
	uint8_t id;         /**< Identifier; the caller owns the sequence.   */
	/** 16 random octets. Required — a predictable Request Authenticator
	 *  makes both the password hiding and the reply check forgeable. */
	const uint8_t *authenticator;
	const char *nas_id; /**< NAS-Identifier, or NULL to omit.            */
	uint32_t nas_ip;    /**< NAS-IP-Address in host order; 0 omits it.   */
	uint32_t nas_port;  /**< NAS-Port; only emitted with @p nas_port_set */
	bool nas_port_set;
	uint8_t nas_port_type; /**< NAS-Port-Type; 0 omits.                  */
	uint8_t service_type;  /**< Service-Type to request; 0 omits.        */
	const uint8_t *state;  /**< State from a prior challenge, or NULL.   */
	size_t state_len;
	/** Append a Message-Authenticator (RFC 3579 §3.2). Recommended. */
	bool message_authenticator;
} radius_req_t;

/**
 * Build an Access-Request.
 *
 * @retval 0        @p out_len octets built.
 * @retval -EINVAL  A missing required field, or an over-long user/secret/state.
 * @retval -ENOSPC  @p cap too small.
 */
int radius_build_access_request(const radius_req_t *req, uint8_t *out,
				size_t cap, size_t *out_len);

/* ----------------------------------------------------------------- response */

typedef struct {
	uint8_t code;
	uint8_t id;
	uint8_t role;         /**< auth_role_t, for an Access-Accept          */
	bool role_explicit;   /**< an attribute named the role                */
	uint8_t state[RADIUS_STATE_MAX];
	size_t state_len;     /**< State from an Access-Challenge             */
	char reply[RADIUS_REPLY_MAX + 1U]; /**< Reply-Message, NUL-terminated */
	bool have_msg_auth;   /**< a Message-Authenticator was present and OK */
} radius_rsp_t;

/**
 * Parse and authenticate a reply.
 *
 * @param req_id    Identifier of the request this answers.
 * @param req_auth  Request Authenticator of that request.
 *
 * @retval 0         Authentic; @p out describes it. Inspect @p out->code.
 * @retval -EINVAL   NULL argument.
 * @retval -EBADMSG  Short, over-long, length-field mismatch, or a malformed
 *                   attribute list.
 * @retval -ENOMSG   Identifier mismatch: not an answer to this request.
 * @retval -EPROTO   A code this client does not expect.
 * @retval -EBADE    Response Authenticator or Message-Authenticator mismatch —
 *                   a forged or mis-keyed reply. Drop it and keep waiting.
 */
int radius_parse_response(const uint8_t *pkt, size_t len, const uint8_t *shared,
			  size_t shared_len, uint8_t req_id,
			  const uint8_t req_auth[RADIUS_AUTH_LEN],
			  radius_rsp_t *out);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_AUTH_RADIUS_H_ */
