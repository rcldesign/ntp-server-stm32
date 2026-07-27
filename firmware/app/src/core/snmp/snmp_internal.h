/*
 * STS1000 "Meridian" — core/snmp: private interface between snmp.c and
 * snmp_v3.c.
 *
 * Not a public API. It exists because SNMPv2c and SNMPv3 differ *only* in the
 * envelope: both carry the identical RFC 3416 PDU, and both answer it from the
 * same OID catalogue and getter. Duplicating the PDU parser and the GETNEXT /
 * GETBULK walk into the v3 path would give an SNMPv3 manager subtly different
 * answers from an SNMPv2c one, which is the sort of divergence that shows up as
 * a monitoring bug two years later.
 *
 * So snmp.c owns the PDU layer and exports exactly two entry points here — one
 * to parse a PDU, one to emit a Response PDU — and snmp_v3.c wraps them in the
 * USM envelope. The v2c path is unchanged: it calls the same two functions
 * through its own envelope writer.
 */

#ifndef STS1000_CORE_SNMP_SNMP_INTERNAL_H_
#define STS1000_CORE_SNMP_SNMP_INTERNAL_H_

#include "snmp/snmp.h"

#ifdef __cplusplus
extern "C" {
#endif

/** One parsed varbind name plus the GETBULK walk cursor for it. */
typedef struct {
	uint32_t arcs[SNMP_OID_MAX_LEN];
	uint8_t n;
	bool over; /**< the encoded OID had more arcs than we store */
	int next;  /**< GETBULK: flat index of the next instance, or -1 */
} snmp_vb_t;

/** A parsed request PDU. */
typedef struct {
	uint8_t pdu;   /**< SNMP_PDU_* tag as received */
	int32_t reqid;
	int32_t f2;    /**< error-status, or non-repeaters   */
	int32_t f3;    /**< error-index, or max-repetitions  */
	size_t n_vb;
	snmp_vb_t vb[SNMP_MAX_VARBINDS];
	/** The request's varbind-list content, echoed verbatim in a SET reply. */
	const uint8_t *vbl;
	size_t vbl_len;
} snmp_req_t;

/**
 * Parse one request PDU.
 *
 * @param pdu_tag  The PDU's own tag, already read by the envelope parser.
 * @param body     The PDU's content octets (request-id onwards).
 *
 * @retval 0         Parsed.
 * @retval -ENOTSUP  A PDU tag this agent does not serve.
 * @retval -E2BIG    More varbinds than SNMP_MAX_VARBINDS; @p rq->reqid is
 *                   still valid so the caller can answer tooBig.
 * @retval -EBADMSG  Malformed.
 */
int snmp__parse_pdu(uint8_t pdu_tag, const uint8_t *body, size_t body_len,
		    snmp_req_t *rq);

/**
 * Emit a Response PDU (tag SNMP_PDU_RESPONSE) into @p w.
 *
 * @param err        Non-zero emits the error shape: an empty varbind list, or
 *                   the request's own list for a refused SET (RFC 3416 §4.2.5).
 * @param err_index  error-index to report.
 *
 * @retval 0        Written; `c->stats.varbinds` advanced.
 * @retval -ENOSPC  A GET/GETNEXT response did not fit. (A GETBULK instead stops
 *                  at the last varbind that fitted, per RFC 3416 §4.2.3.)
 */
int snmp__emit_response(snmp_ctx_t *c, snmp_req_t *rq, int32_t err,
			int32_t err_index, uint32_t uptime_cs, snmp_wr_t *w);

/**
 * Emit a bare PDU whose varbind list is supplied by @p fill.
 *
 * Used by snmp_v3.c for Report PDUs, whose single varbind is a usmStats
 * counter rather than anything in the OID catalogue.
 */
typedef int (*snmp__vbl_fn)(snmp_wr_t *w, void *ctx);

int snmp__emit_pdu(snmp_wr_t *w, uint8_t pdu_tag, int32_t reqid, int32_t err,
		   int32_t err_index, snmp__vbl_fn fill, void *fill_ctx);

/** Append one varbind: name then value. Shared by both envelopes. */
int snmp__put_varbind(snmp_wr_t *w, const uint32_t *arcs, size_t n,
		      const snmp_value_t *v);

/** Resolve (obj, inst) through the getter, with sysUpTime supplied locally. */
void snmp__resolve(snmp_ctx_t *c, uint16_t obj, uint16_t inst,
		   uint32_t uptime_cs, snmp_value_t *v);

/** OID of snmpTrapOID.0, for the mandatory second trap varbind. */
extern const uint32_t snmp__oid_trap_oid[11];

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_SNMP_SNMP_INTERNAL_H_ */
