/*
 * STS1000 "Meridian" — core/auth: LDAP v3 client codec (RFC 4511).
 *
 * Platform-neutral C11, no dynamic allocation. Builds simple BindRequests,
 * SearchRequests, UnbindRequest and the StartTLS ExtendedRequest, and parses
 * BindResponse / SearchResultEntry / SearchResultDone / ExtendedResponse. The
 * TCP connection (389), the TLS wrapper (636 or StartTLS) and the read loop
 * live in the glue (`src/zephyr/net/sts_aaa.c`).
 *
 * ---------------------------------------------------------------------------
 * A second BER codec, on purpose
 * ---------------------------------------------------------------------------
 *
 * core/snmp already contains a BER reader and writer, and this module contains
 * another. They are not shared, and that is deliberate: SNMP's codec is tuned
 * to SNMP's type set (Counter64, the varbind exceptions, a 24-arc OID limit) and
 * LDAP needs a different one (ENUMERATED, BOOLEAN, context-tagged constructed
 * choices, SET OF, no OIDs at all). Merging them would produce a codec that is
 * a superset of both protocols' grammars, which is exactly the thing you do not
 * want in the parser sitting in front of an authentication decision. Same
 * discipline — definite lengths only, no indefinite form, every length checked
 * against the buffer — separate code.
 *
 * ---------------------------------------------------------------------------
 * Bounded by construction
 * ---------------------------------------------------------------------------
 *
 * Every string this module extracts lands in a fixed field and is truncated
 * with a flag rather than allocated. A server can therefore return a 10 MB
 * diagnostic message or a DN with 4000 RDNs without the client doing anything
 * except ignoring most of it. Searches carry an explicit sizeLimit and
 * timeLimit so the *server* is also bounded, which is what keeps a group with
 * 50 000 members from being a denial of service against the login path.
 */

#ifndef STS1000_CORE_AUTH_LDAP_H_
#define STS1000_CORE_AUTH_LDAP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "auth/auth.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ limits */

/** Largest LDAPMessage this codec builds or accepts. */
#ifndef LDAP_BUF_MAX
#define LDAP_BUF_MAX 1024U
#endif

/** Longest distinguished name kept, excluding the NUL. */
#define LDAP_DN_MAX 191U
/** Longest attribute description kept. */
#define LDAP_ATTR_MAX 31U
/** Longest attribute value kept. */
#define LDAP_VALUE_MAX 127U
/** Values kept per attribute. */
#define LDAP_VALUES_MAX 4U
/** Longest diagnostic message kept. */
#define LDAP_DIAG_MAX 95U
/** Longest bind password accepted. */
#define LDAP_PW_MAX AUTH_SHARED_MAX

/* -------------------------------------------------------------- BER tags */

#define LDAP_T_BOOLEAN 0x01U
#define LDAP_T_INTEGER 0x02U
#define LDAP_T_OCTET_STRING 0x04U
#define LDAP_T_NULL 0x05U
#define LDAP_T_ENUMERATED 0x0AU
#define LDAP_T_SEQUENCE 0x30U
#define LDAP_T_SET 0x31U

/** protocolOp tags: [APPLICATION n], constructed unless noted. */
#define LDAP_OP_BIND_REQUEST 0x60U
#define LDAP_OP_BIND_RESPONSE 0x61U
#define LDAP_OP_UNBIND_REQUEST 0x42U /**< primitive, zero length */
#define LDAP_OP_SEARCH_REQUEST 0x63U
#define LDAP_OP_SEARCH_ENTRY 0x64U
#define LDAP_OP_SEARCH_DONE 0x65U
#define LDAP_OP_SEARCH_REFERENCE 0x73U
#define LDAP_OP_EXTENDED_REQUEST 0x77U
#define LDAP_OP_EXTENDED_RESPONSE 0x78U

/** AuthenticationChoice simple = [0] primitive. */
#define LDAP_AUTH_SIMPLE 0x80U
/** Filter equalityMatch = [3] constructed; and = [0]; present = [7] primitive. */
#define LDAP_FILTER_AND 0xA0U
#define LDAP_FILTER_EQUALITY 0xA3U
#define LDAP_FILTER_PRESENT 0x87U
/** ExtendedRequest requestName = [0] primitive. */
#define LDAP_EXT_REQUEST_NAME 0x80U

/* ------------------------------------------------------------------ scopes */

#define LDAP_SCOPE_BASE 0U
#define LDAP_SCOPE_ONELEVEL 1U
#define LDAP_SCOPE_SUBTREE 2U

#define LDAP_DEREF_NEVER 0U

/* ------------------------------------------------------------ result codes */

#define LDAP_RES_SUCCESS 0
#define LDAP_RES_OPERATIONS_ERROR 1
#define LDAP_RES_PROTOCOL_ERROR 2
#define LDAP_RES_TIME_LIMIT_EXCEEDED 3
#define LDAP_RES_SIZE_LIMIT_EXCEEDED 4
#define LDAP_RES_AUTH_METHOD_NOT_SUPPORTED 7
#define LDAP_RES_STRONGER_AUTH_REQUIRED 8
#define LDAP_RES_REFERRAL 10
#define LDAP_RES_NO_SUCH_OBJECT 32
#define LDAP_RES_INVALID_DN_SYNTAX 34
#define LDAP_RES_INAPPROPRIATE_AUTH 48
#define LDAP_RES_INVALID_CREDENTIALS 49
#define LDAP_RES_INSUFFICIENT_ACCESS 50
#define LDAP_RES_BUSY 51
#define LDAP_RES_UNAVAILABLE 52
#define LDAP_RES_UNWILLING_TO_PERFORM 53

/** OID of the StartTLS extended operation (RFC 4511 §4.14.1). */
#define LDAP_OID_STARTTLS "1.3.6.1.4.1.1466.20037"

/** Name of a result code, for logs. Never NULL. */
const char *ldap_result_name(int32_t code);

/**
 * Map a result code onto the errno-style value the AAA chain expects.
 *
 * @retval 0                success
 * @retval -EACCES          invalidCredentials / inappropriateAuthentication /
 *                          insufficientAccessRights — an authoritative "no"
 * @retval -EHOSTUNREACH    busy / unavailable / unwillingToPerform / referral —
 *                          the directory cannot answer, so try the next backend
 * @retval -EPROTO          anything else
 */
int ldap_result_to_errno(int32_t code);

/* ------------------------------------------------------------------ framing */

/**
 * Length of the first complete LDAPMessage in @p buf.
 *
 * A stream reader needs this before it can hand a whole message to a parser.
 *
 * @retval >0        Total octets of the message, tag and length included.
 * @retval -EAGAIN   Not yet a complete message.
 * @retval -EBADMSG  Not a SEQUENCE, an indefinite length, or a length over
 *                   LDAP_BUF_MAX.
 * @retval -EINVAL   NULL @p buf.
 */
int ldap_msg_len(const uint8_t *buf, size_t len);

/**
 * Peek the messageID and protocolOp tag without decoding the operation.
 *
 * @retval 0         Peeked.
 * @retval -EAGAIN   Incomplete.
 * @retval -EBADMSG  Malformed envelope.
 * @retval -EINVAL   NULL argument.
 */
int ldap_msg_peek(const uint8_t *buf, size_t len, uint32_t *out_msgid,
		  uint8_t *out_op);

/* ------------------------------------------------------------------ builders */

/**
 * Build a simple BindRequest.
 *
 * @param dn  Bind DN, or "" for an anonymous bind.
 * @param pw  Password. An empty password with a non-empty DN is refused: RFC
 *            4513 §5.1.2 makes that an *unauthenticated* bind that many servers
 *            answer with success, which would turn a blank password into a
 *            successful login.
 *
 * @retval 0        Built.
 * @retval -EINVAL  NULL argument, an over-long field, or an empty password with
 *                  a non-empty DN.
 * @retval -ENOSPC  @p cap too small.
 */
int ldap_build_bind_simple(uint32_t msgid, const char *dn, const char *pw,
			   uint8_t *out, size_t cap, size_t *out_len);

/** Build an UnbindRequest. */
int ldap_build_unbind(uint32_t msgid, uint8_t *out, size_t cap,
		      size_t *out_len);

/** Build the StartTLS ExtendedRequest. */
int ldap_build_starttls(uint32_t msgid, uint8_t *out, size_t cap,
			size_t *out_len);

/**
 * A bounded search.
 *
 * The filter is `(attr=value)` or, with the second pair set,
 * `(&(attr=value)(attr2=value2))`. That is enough for both group-membership
 * checks (`(&(objectClass=groupOfNames)(member=<userDN>))`) and user lookup
 * (`(uid=<name>)`), and it keeps the filter grammar this module has to parse
 * down to nothing — the client only ever *writes* filters.
 */
typedef struct {
	const char *base;  /**< baseObject; "" is the root DSE            */
	uint8_t scope;     /**< LDAP_SCOPE_*                              */
	uint32_t size_limit; /**< 0 = server default; always set it       */
	uint32_t time_limit; /**< seconds; 0 = server default             */
	const char *attr;  /**< first filter attribute. Required.         */
	const char *value; /**< first filter value. Required.             */
	const char *attr2; /**< optional second AND term                  */
	const char *value2;
	const char *want_attr; /**< attribute to return, or NULL for none */
} ldap_search_req_t;

/**
 * Build a SearchRequest.
 *
 * @retval 0        Built.
 * @retval -EINVAL  NULL argument, no filter attribute/value, a bad scope, or an
 *                  over-long field.
 * @retval -ENOSPC  @p cap too small.
 */
int ldap_build_search(uint32_t msgid, const ldap_search_req_t *s, uint8_t *out,
		      size_t cap, size_t *out_len);

/* ------------------------------------------------------------------ parsers */

/** A parsed LDAPResult (BindResponse, SearchResultDone, ExtendedResponse). */
typedef struct {
	uint32_t msgid;
	uint8_t op;
	int32_t code;
	char matched_dn[LDAP_DN_MAX + 1U];
	char diagnostic[LDAP_DIAG_MAX + 1U];
	bool truncated; /**< a field was longer than its buffer */
} ldap_result_t;

/**
 * Parse a result-bearing response.
 *
 * @param want_op  Required protocolOp tag, or 0 to accept any of the three
 *                 result-bearing operations.
 *
 * @retval 0         Parsed.
 * @retval -EAGAIN   Incomplete message.
 * @retval -EBADMSG  Malformed.
 * @retval -ENOMSG   A different operation than @p want_op.
 * @retval -EINVAL   NULL argument.
 */
int ldap_parse_result(const uint8_t *buf, size_t len, uint8_t want_op,
		      ldap_result_t *out);

/** One SearchResultEntry, reduced to its DN and (at most) one attribute. */
typedef struct {
	uint32_t msgid;
	char dn[LDAP_DN_MAX + 1U];
	char attr[LDAP_ATTR_MAX + 1U];
	char values[LDAP_VALUES_MAX][LDAP_VALUE_MAX + 1U];
	uint8_t n_values;
	bool truncated;
} ldap_entry_t;

/**
 * Parse a SearchResultEntry.
 *
 * Keeps the objectName and the first attribute whose type matches @p want_attr
 * — or the first attribute at all when @p want_attr is NULL. Later attributes
 * are skipped: a membership check needs the DN, and a role lookup needs one
 * named attribute, so there is nothing to gain from storing the rest.
 *
 * @retval 0         Parsed.
 * @retval -EAGAIN   Incomplete message.
 * @retval -EBADMSG  Malformed.
 * @retval -ENOMSG   Not a SearchResultEntry.
 * @retval -EINVAL   NULL argument.
 */
int ldap_parse_search_entry(const uint8_t *buf, size_t len,
			    const char *want_attr, ldap_entry_t *out);

/* --------------------------------------------------------- role mapping */

/** Group DNs (or role-attribute values) that grant each role. */
typedef struct {
	const char *admin;
	const char *oper;
	const char *viewer;
} ldap_role_map_t;

/**
 * Map a group DN — or a role attribute value — onto an auth_role_t.
 *
 * Compared case-insensitively, because DN comparison in LDAP is
 * case-insensitive for the attribute types and for the common string syntaxes,
 * and an operator who types `CN=Admins,DC=x` should not be defeated by a
 * server that answers `cn=admins,dc=x`. Whitespace around the commas is not
 * normalised — full DN normalisation (RFC 4514) is out of scope, so document
 * that the configured DN must match the directory's own spelling.
 *
 * Highest privilege wins when a user is in several groups.
 *
 * @param out_explicit  Optional; true when a configured group actually matched.
 * @return auth_role_t; AUTH_ROLE_NONE when nothing matched.
 */
uint8_t ldap_role_from_group(const ldap_role_map_t *map, const char *dn,
			     bool *out_explicit);

/**
 * Substitute `%s` in @p tmpl with @p user, once.
 *
 * Used to turn a configured bind template such as
 * `uid=%s,ou=people,dc=example,dc=com` into a user DN. The username is rejected
 * rather than escaped if it contains any of the DN special characters
 * (`,+"\<>;=` or a NUL) or a leading/trailing space — building a DN by
 * substitution is only safe when the substituted text cannot change the DN's
 * structure, and refusing is far easier to audit than escaping.
 *
 * @retval >=0      Length written.
 * @retval -EINVAL  NULL argument, no `%s` in @p tmpl, or an unsafe username.
 * @retval -ENOSPC  @p cap too small.
 */
int ldap_dn_from_template(const char *tmpl, const char *user, char *out,
			  size_t cap);

/** True when @p user is safe to substitute into a DN. */
bool ldap_user_is_dn_safe(const char *user);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_AUTH_LDAP_H_ */
