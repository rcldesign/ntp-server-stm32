/*
 * STS1000 "Meridian" — core/auth: TACACS+ client codec (RFC 8907).
 *
 * Platform-neutral C11, no dynamic allocation. Builds authentication START and
 * CONTINUE packets and authorization REQUESTs, and parses the corresponding
 * REPLY / RESPONSE. The TCP connection to port 49 lives in the glue
 * (`src/zephyr/net/sts_aaa.c`).
 *
 * ---------------------------------------------------------------------------
 * "Encryption" is obfuscation, and this module says so
 * ---------------------------------------------------------------------------
 *
 * RFC 8907 §4.5 defines the body transform as an XOR against an MD5-derived
 * keystream. RFC 8907 §10.5.2 is explicit that this provides neither
 * confidentiality nor integrity to any modern standard, which is why the RFC
 * requires TACACS+ to be run over an already-secured channel. This
 * implementation is faithful to the protocol and names the transform
 * *obfuscation* throughout so nobody reads a security property into it. Deploy
 * it on a management VLAN (spec §9.7), not across an untrusted path.
 *
 * The transform is its own inverse, so @ref tacacs_obfuscate both applies and
 * removes it.
 *
 * ---------------------------------------------------------------------------
 * Wire format
 * ---------------------------------------------------------------------------
 *
 *   header (12 octets, never obfuscated)
 *     version(1) type(1) seq_no(1) flags(1) session_id(4, BE) length(4, BE)
 *   body (length octets, obfuscated unless the unencrypted flag is set)
 *
 * `seq_no` starts at 1 for the client's first packet and increments by one for
 * every packet in the session, so the client's are odd and the server's even.
 * A reply carrying the wrong sequence number, session id or a length that does
 * not match what arrived is rejected — those are the cheap checks that stop a
 * cross-session mix-up from being parsed as an authorisation.
 */

#ifndef STS1000_CORE_AUTH_TACACS_H_
#define STS1000_CORE_AUTH_TACACS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "auth/auth.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ limits */

#define TACACS_HDR_LEN 12U

/** Largest body this codec builds or accepts. */
#ifndef TACACS_BODY_MAX
#define TACACS_BODY_MAX 512U
#endif

/** Largest whole packet. */
#define TACACS_PKT_MAX (TACACS_HDR_LEN + TACACS_BODY_MAX)

/** Authorization arguments captured from a response. */
#define TACACS_ARGS_MAX 8U
/** Longest argument captured, excluding the NUL. */
#define TACACS_ARG_LEN_MAX 63U
/** Longest server message captured, excluding the NUL. */
#define TACACS_MSG_MAX 63U

/* ----------------------------------------------------------------- version */

#define TACACS_VER_MAJOR 0xCU
/** Minor 0 = ASCII login; minor 1 = PAP/CHAP/MS-CHAP login. */
#define TACACS_VER_MINOR_DEFAULT 0x0U
#define TACACS_VER_MINOR_ONE 0x1U
#define TACACS_VER(minor) ((uint8_t)((TACACS_VER_MAJOR << 4) | ((minor) & 0xFU)))

/* ------------------------------------------------------------------- types */

#define TACACS_TYPE_AUTHEN 0x01U
#define TACACS_TYPE_AUTHOR 0x02U
#define TACACS_TYPE_ACCT 0x03U

/* ------------------------------------------------------------------- flags */

#define TACACS_FLAG_UNENCRYPTED 0x01U
#define TACACS_FLAG_SINGLE_CONNECT 0x04U

/* ---------------------------------------------------- authentication fields */

#define TACACS_AUTHEN_LOGIN 0x01U
#define TACACS_AUTHEN_CHPASS 0x02U

#define TACACS_AUTHEN_TYPE_ASCII 0x01U
#define TACACS_AUTHEN_TYPE_PAP 0x02U
#define TACACS_AUTHEN_TYPE_CHAP 0x03U

#define TACACS_SVC_NONE 0x00U
#define TACACS_SVC_LOGIN 0x01U
#define TACACS_SVC_ENABLE 0x02U

#define TACACS_AUTHEN_STATUS_PASS 0x01U
#define TACACS_AUTHEN_STATUS_FAIL 0x02U
#define TACACS_AUTHEN_STATUS_GETDATA 0x03U
#define TACACS_AUTHEN_STATUS_GETUSER 0x04U
#define TACACS_AUTHEN_STATUS_GETPASS 0x05U
#define TACACS_AUTHEN_STATUS_RESTART 0x06U
#define TACACS_AUTHEN_STATUS_ERROR 0x07U
#define TACACS_AUTHEN_STATUS_FOLLOW 0x21U

/** CONTINUE flags. */
#define TACACS_CONTINUE_FLAG_ABORT 0x01U

/* ----------------------------------------------------- authorization fields */

#define TACACS_AUTHEN_METH_TACACSPLUS 0x06U

#define TACACS_AUTHOR_STATUS_PASS_ADD 0x01U
#define TACACS_AUTHOR_STATUS_PASS_REPL 0x02U
#define TACACS_AUTHOR_STATUS_FAIL 0x10U
#define TACACS_AUTHOR_STATUS_ERROR 0x11U
#define TACACS_AUTHOR_STATUS_FOLLOW 0x21U

/** Privilege levels (RFC 8907 §9). 15 is the conventional "enable" level. */
#define TACACS_PRIV_MIN 0U
#define TACACS_PRIV_USER 1U
#define TACACS_PRIV_ROOT 15U

/* ------------------------------------------------------------------ header */

typedef struct {
	uint8_t version;
	uint8_t type;
	uint8_t seq_no;
	uint8_t flags;
	uint32_t session_id;
	uint32_t length; /**< body length */
} tacacs_hdr_t;

/**
 * Serialise a header.
 *
 * @retval 0        Written (TACACS_HDR_LEN octets).
 * @retval -EINVAL  NULL argument, or a body length over TACACS_BODY_MAX.
 * @retval -ENOSPC  @p cap too small.
 */
int tacacs_hdr_build(uint8_t *out, size_t cap, const tacacs_hdr_t *h);

/**
 * Parse a header.
 *
 * @retval 0        Parsed.
 * @retval -EINVAL  NULL argument.
 * @retval -EAGAIN  Fewer than TACACS_HDR_LEN octets — read more.
 * @retval -EBADMSG A major version other than 0xC, or a body length over
 *                  TACACS_BODY_MAX.
 */
int tacacs_hdr_parse(const uint8_t *in, size_t len, tacacs_hdr_t *out);

/**
 * Apply (or remove) the RFC 8907 §4.5 body obfuscation in place.
 *
 * `pad_1 = MD5(session_id ‖ key ‖ version ‖ seq_no)` and each further block
 * appends the previous block's digest. A NULL or empty key is a no-op — the
 * caller is then responsible for having set TACACS_FLAG_UNENCRYPTED, which
 * @ref tacacs_build_authen_start and friends do.
 *
 * @retval 0        Transformed (or nothing to do).
 * @retval -EINVAL  NULL header/body with a non-zero length, or an over-long body.
 */
int tacacs_obfuscate(const tacacs_hdr_t *h, const uint8_t *key, size_t key_len,
		     uint8_t *body, size_t body_len);

/* --------------------------------------------------------- authentication */

/** Fields of an authentication START. Strings are borrowed; NULL means empty. */
typedef struct {
	uint32_t session_id;
	uint8_t action;         /**< TACACS_AUTHEN_LOGIN                     */
	uint8_t priv_lvl;       /**< requested privilege level               */
	uint8_t authen_type;    /**< TACACS_AUTHEN_TYPE_PAP or _ASCII        */
	uint8_t authen_service; /**< TACACS_SVC_LOGIN                        */
	const char *user;
	const char *port;      /**< e.g. "https" or "console"                */
	const char *rem_addr;  /**< client address, for the server's log     */
	/** PAP password; ignored for the ASCII flow (which sends it later). */
	const char *password;
	bool single_connect;
} tacacs_authen_start_t;

/**
 * Build an authentication START (sequence number 1).
 *
 * The minor version is set to 1 for PAP and 0 for ASCII, per RFC 8907 §5.1.
 *
 * @retval 0        Built.
 * @retval -EINVAL  NULL argument, no user, or an over-long field.
 * @retval -ENOSPC  @p cap too small.
 */
int tacacs_build_authen_start(const tacacs_authen_start_t *in,
			      const uint8_t *key, size_t key_len, uint8_t *out,
			      size_t cap, size_t *out_len);

/**
 * Build an authentication CONTINUE.
 *
 * @param seq_no    Sequence number to use; the caller tracks the session's.
 * @param user_msg  Reply to the server's prompt (the username or password), or
 *                  NULL when aborting.
 * @param abort     Set the abort flag; @p user_msg then carries the reason in
 *                  the data field, per RFC 8907 §5.4.
 */
int tacacs_build_authen_continue(uint32_t session_id, uint8_t seq_no,
				 const char *user_msg, bool abort,
				 const uint8_t *key, size_t key_len,
				 uint8_t *out, size_t cap, size_t *out_len);

typedef struct {
	uint8_t status;
	uint8_t flags;
	char server_msg[TACACS_MSG_MAX + 1U];
	size_t data_len;
	uint8_t seq_no;
} tacacs_authen_reply_t;

/**
 * Parse an authentication REPLY, removing the obfuscation into a scratch copy.
 *
 * @param session_id  Session this belongs to.
 * @param expect_seq  Sequence number expected; 0 accepts any even number.
 *
 * @retval 0         Parsed.
 * @retval -EINVAL   NULL argument.
 * @retval -EAGAIN   Short of a whole packet.
 * @retval -EBADMSG  Bad header, or a body whose internal lengths do not add up.
 * @retval -ENOMSG   Session-id or sequence-number mismatch.
 */
int tacacs_parse_authen_reply(const uint8_t *pkt, size_t len,
			      uint32_t session_id, uint8_t expect_seq,
			      const uint8_t *key, size_t key_len,
			      tacacs_authen_reply_t *out);

/* --------------------------------------------------------- authorization */

typedef struct {
	uint32_t session_id;
	uint8_t seq_no;
	uint8_t authen_method; /**< TACACS_AUTHEN_METH_TACACSPLUS            */
	uint8_t priv_lvl;
	uint8_t authen_type;
	uint8_t authen_service;
	const char *user;
	const char *port;
	const char *rem_addr;
	/** Arguments, e.g. "service=sts1000" and "cmd=". NULL-terminated list. */
	const char *const *args;
	size_t n_args;
	bool single_connect;
} tacacs_author_req_t;

/**
 * Build an authorization REQUEST.
 *
 * @retval 0        Built.
 * @retval -EINVAL  NULL argument, no user, too many args, or an over-long field.
 * @retval -ENOSPC  @p cap too small.
 */
int tacacs_build_author_request(const tacacs_author_req_t *in,
				const uint8_t *key, size_t key_len,
				uint8_t *out, size_t cap, size_t *out_len);

typedef struct {
	uint8_t status;
	uint8_t seq_no;
	char server_msg[TACACS_MSG_MAX + 1U];
	char args[TACACS_ARGS_MAX][TACACS_ARG_LEN_MAX + 1U];
	uint8_t n_args;
	/** Arguments the server sent that were too long or too many to keep. */
	uint8_t args_dropped;
} tacacs_author_rsp_t;

/**
 * Parse an authorization RESPONSE.
 *
 * Same return values as @ref tacacs_parse_authen_reply.
 */
int tacacs_parse_author_response(const uint8_t *pkt, size_t len,
				 uint32_t session_id, uint8_t expect_seq,
				 const uint8_t *key, size_t key_len,
				 tacacs_author_rsp_t *out);

/**
 * Map authorization arguments onto an auth_role_t.
 *
 * Checked in order: `role=` (parsed by @ref auth_role_parse), then `priv-lvl=`
 * with 15 → admin, 2..14 → operator, 0..1 → viewer. A response that names
 * neither maps to AUTH_ROLE_VIEWER, and @p out_explicit says which happened.
 */
uint8_t tacacs_role_from_args(const tacacs_author_rsp_t *rsp,
			      bool *out_explicit);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_AUTH_TACACS_H_ */
