/*
 * STS1000 "Meridian" — core/snmp: SNMPv2c read-only agent + traps.
 *
 * See snmp.h for the contract and the deliberate scope limits.
 *
 * Message layout served (RFC 3416 §3, RFC 1901):
 *
 *   SEQUENCE {
 *     INTEGER       version (1 == v2c)
 *     OCTET STRING  community
 *     [n] IMPLICIT SEQUENCE {              -- PDU, n = 0/1/3/5 in, 2 out
 *       INTEGER   request-id
 *       INTEGER   error-status | non-repeaters
 *       INTEGER   error-index  | max-repetitions
 *       SEQUENCE OF SEQUENCE { OID name, value }
 *     }
 *   }
 *
 * Writing definite lengths forward
 * --------------------------------
 * A constructed TLV's length is not known until its content is written, and
 * BER has no back-reference. snmp_wr_begin() therefore reserves the widest
 * length this agent can need (three octets, enough for SNMP_PKT_MAX) and
 * snmp_wr_end() shrinks it to the minimal encoding, memmove()ing the content
 * down. Nesting never exceeds four levels and the moves are of a partial
 * datagram, so the cost is a few hundred bytes of copying per response — far
 * cheaper than the usual alternative of encoding the whole message backwards,
 * which makes every helper read in reverse and is where hand-written BER
 * encoders acquire their off-by-ones.
 */

#include "snmp/snmp.h"

#include <errno.h>
#include <string.h>

#include "snmp/snmp_internal.h"

/* ========================================================================= */
/* BER codec                                                                 */
/* ========================================================================= */

/** Octets snmp_wr_begin() reserves for a constructed TLV's length. */
#define SNMP_LEN_RESERVE 3U

int snmp_rd_init(snmp_rd_t *r, const uint8_t *buf, size_t len)
{
	if (r == NULL) {
		return -EINVAL;
	}
	if (buf == NULL && len != 0U) {
		return -EINVAL;
	}
	r->buf = buf;
	r->len = len;
	r->off = 0U;
	return 0;
}

int snmp_wr_init(snmp_wr_t *w, uint8_t *buf, size_t cap)
{
	if (w == NULL) {
		return -EINVAL;
	}
	if (buf == NULL && cap != 0U) {
		return -EINVAL;
	}
	w->buf = buf;
	w->cap = cap;
	w->len = 0U;
	return 0;
}

size_t snmp_wr_len(const snmp_wr_t *w)
{
	return (w == NULL) ? 0U : w->len;
}

bool snmp_rd_done(const snmp_rd_t *r)
{
	return (r == NULL) || (r->off >= r->len);
}

int snmp_rd_skip(snmp_rd_t *r, size_t len)
{
	if (r == NULL) {
		return -EINVAL;
	}
	if (len > (r->len - r->off)) {
		return -EBADMSG;
	}
	r->off += len;
	return 0;
}

int snmp_rd_hdr(snmp_rd_t *r, uint8_t *tag, size_t *len)
{
	uint8_t t;
	uint8_t l0;
	size_t l;

	if (r == NULL || tag == NULL || len == NULL) {
		return -EINVAL;
	}
	if ((r->len - r->off) < 2U) {
		return -EBADMSG;
	}

	t = r->buf[r->off++];
	/* A low tag number of 0x1F introduces a multi-octet identifier. No SNMP
	 * type uses one, and accepting it would mean carrying a decoder that can
	 * only ever produce values this agent rejects later. */
	if ((t & 0x1FU) == 0x1FU) {
		return -EBADMSG;
	}

	l0 = r->buf[r->off++];
	if ((l0 & 0x80U) == 0U) {
		l = l0;
	} else {
		uint8_t nb = (uint8_t)(l0 & 0x7FU);
		uint8_t i;

		/* nb == 0 is the indefinite form: legal in BER, illegal in the
		 * DER-ish subset SNMP mandates, and a well-known parser trap. */
		if (nb == 0U || nb > 4U) {
			return -EBADMSG;
		}
		if (nb > (r->len - r->off)) {
			return -EBADMSG;
		}
		l = 0U;
		for (i = 0U; i < nb; i++) {
			l = (l << 8) | (size_t)r->buf[r->off++];
		}
	}

	if (l > (r->len - r->off)) {
		return -EBADMSG;
	}

	*tag = t;
	*len = l;
	return 0;
}

int snmp_rd_int(snmp_rd_t *r, int64_t *out)
{
	uint8_t tag;
	size_t len;
	int64_t v;
	size_t i;
	int rc;

	if (r == NULL || out == NULL) {
		return -EINVAL;
	}
	rc = snmp_rd_hdr(r, &tag, &len);
	if (rc != 0) {
		return rc;
	}
	if (tag != SNMP_TAG_INTEGER || len == 0U || len > 8U) {
		return -EBADMSG;
	}

	v = ((r->buf[r->off] & 0x80U) != 0U) ? -1 : 0;
	for (i = 0U; i < len; i++) {
		v = (int64_t)(((uint64_t)v << 8) | (uint64_t)r->buf[r->off + i]);
	}
	r->off += len;
	*out = v;
	return 0;
}

int snmp_rd_uint(snmp_rd_t *r, uint8_t want_tag, uint64_t *out)
{
	uint8_t tag;
	size_t len;
	uint64_t v = 0U;
	size_t i;
	int rc;

	if (r == NULL || out == NULL) {
		return -EINVAL;
	}
	rc = snmp_rd_hdr(r, &tag, &len);
	if (rc != 0) {
		return rc;
	}
	if (tag != want_tag || len == 0U || len > 9U) {
		return -EBADMSG;
	}
	/* Nine octets are only representable when the first is the zero pad the
	 * encoding inserts to keep a 64-bit value unsigned. */
	if (len == 9U && r->buf[r->off] != 0U) {
		return -EBADMSG;
	}
	for (i = 0U; i < len; i++) {
		v = (v << 8) | (uint64_t)r->buf[r->off + i];
	}
	r->off += len;
	*out = v;
	return 0;
}

int snmp_rd_octets(snmp_rd_t *r, const uint8_t **out, size_t *out_len)
{
	uint8_t tag;
	size_t len;
	int rc;

	if (r == NULL || out == NULL || out_len == NULL) {
		return -EINVAL;
	}
	rc = snmp_rd_hdr(r, &tag, &len);
	if (rc != 0) {
		return rc;
	}
	if (tag != SNMP_TAG_OCTET_STRING) {
		return -EBADMSG;
	}
	*out = &r->buf[r->off];
	*out_len = len;
	r->off += len;
	return 0;
}

/** Decode one base-128 subidentifier starting at @p *i. */
static int oid_subid(const uint8_t *p, size_t end, size_t *i, uint64_t *out)
{
	uint64_t v = 0U;
	uint8_t b;
	unsigned int nbytes = 0U;

	if (*i >= end) {
		return -EBADMSG;
	}
	/* A leading 0x80 makes the encoding non-minimal, which BER forbids and
	 * which is the standard way to smuggle two encodings of one OID past a
	 * comparison that only ever sees the decoded form. */
	if (p[*i] == 0x80U) {
		return -EBADMSG;
	}
	do {
		if (*i >= end) {
			return -EBADMSG;
		}
		b = p[(*i)++];
		nbytes++;
		if (nbytes > 5U) {
			return -EBADMSG; /* wider than 32 bits */
		}
		v = (v << 7) | (uint64_t)(b & 0x7FU);
	} while ((b & 0x80U) != 0U);

	if (v > 0xFFFFFFFFU) {
		return -EBADMSG;
	}
	*out = v;
	return 0;
}

int snmp_rd_oid(snmp_rd_t *r, uint32_t *arcs, size_t cap, size_t *out_n,
		bool *out_over)
{
	uint8_t tag;
	size_t len;
	size_t i;
	size_t end;
	size_t n = 0U;
	bool over = false;
	uint64_t sub;
	int rc;

	if (r == NULL || arcs == NULL || out_n == NULL || cap < 2U) {
		return -EINVAL;
	}
	rc = snmp_rd_hdr(r, &tag, &len);
	if (rc != 0) {
		return rc;
	}
	if (tag != SNMP_TAG_OID || len == 0U) {
		return -EBADMSG;
	}

	i = r->off;
	end = r->off + len;

	rc = oid_subid(r->buf, end, &i, &sub);
	if (rc != 0) {
		return rc;
	}
	if (sub < 80U) {
		arcs[0] = (uint32_t)(sub / 40U);
		arcs[1] = (uint32_t)(sub % 40U);
	} else {
		arcs[0] = 2U;
		arcs[1] = (uint32_t)(sub - 80U);
	}
	n = 2U;

	while (i < end) {
		rc = oid_subid(r->buf, end, &i, &sub);
		if (rc != 0) {
			return rc;
		}
		if (n < cap) {
			arcs[n] = (uint32_t)sub;
			n++;
		} else {
			over = true;
		}
	}

	r->off = end;
	*out_n = n;
	if (out_over != NULL) {
		*out_over = over;
	}
	return 0;
}

/* ------------------------------------------------------------------ writer */

static size_t len_size(size_t l)
{
	if (l < 0x80U) {
		return 1U;
	}
	if (l <= 0xFFU) {
		return 2U;
	}
	return 3U;
}

static void len_put(uint8_t *p, size_t l)
{
	if (l < 0x80U) {
		p[0] = (uint8_t)l;
	} else if (l <= 0xFFU) {
		p[0] = 0x81U;
		p[1] = (uint8_t)l;
	} else {
		p[0] = 0x82U;
		p[1] = (uint8_t)((l >> 8) & 0xFFU);
		p[2] = (uint8_t)(l & 0xFFU);
	}
}

static int wr_room(const snmp_wr_t *w, size_t n)
{
	if (w == NULL || w->buf == NULL) {
		return -EINVAL;
	}
	if (n > (w->cap - w->len)) {
		return -ENOSPC;
	}
	return 0;
}

int snmp_wr_tlv(snmp_wr_t *w, uint8_t tag, const uint8_t *val, size_t len)
{
	size_t ls;
	int rc;

	if (w == NULL || (val == NULL && len != 0U)) {
		return -EINVAL;
	}
	/* Longer than the widest length this encoder emits (see SNMP_LEN_RESERVE). */
	if (len > 0xFFFFU) {
		return -ENOSPC;
	}
	ls = len_size(len);
	rc = wr_room(w, 1U + ls + len);
	if (rc != 0) {
		return rc;
	}
	w->buf[w->len++] = tag;
	len_put(&w->buf[w->len], len);
	w->len += ls;
	if (len != 0U) {
		memcpy(&w->buf[w->len], val, len);
		w->len += len;
	}
	return 0;
}

int snmp_wr_empty(snmp_wr_t *w, uint8_t tag)
{
	return snmp_wr_tlv(w, tag, NULL, 0U);
}

int snmp_wr_int(snmp_wr_t *w, uint8_t tag, int64_t v)
{
	uint8_t tmp[8];
	size_t n = 0U;
	uint64_t u = (uint64_t)v;
	int i;

	/* Minimal two's-complement: drop leading 0x00 (or 0xFF) octets that the
	 * sign bit of the next octet already implies. */
	for (i = 7; i > 0; i--) {
		uint8_t hi = (uint8_t)((u >> (8 * (unsigned int)i)) & 0xFFU);
		uint8_t next = (uint8_t)((u >> (8 * ((unsigned int)i - 1U))) & 0xFFU);

		if (v < 0) {
			if (hi != 0xFFU || (next & 0x80U) == 0U) {
				break;
			}
		} else {
			if (hi != 0x00U || (next & 0x80U) != 0U) {
				break;
			}
		}
	}
	for (; i >= 0; i--) {
		tmp[n++] = (uint8_t)((u >> (8 * (unsigned int)i)) & 0xFFU);
	}

	return snmp_wr_tlv(w, tag, tmp, n);
}

int snmp_wr_uint(snmp_wr_t *w, uint8_t tag, uint64_t v)
{
	uint8_t tmp[9];
	size_t n = 0U;
	int i;

	for (i = 7; i > 0; i--) {
		if (((v >> (8 * (unsigned int)i)) & 0xFFU) != 0U) {
			break;
		}
	}
	/* An unsigned value whose top content bit is set needs a 0x00 pad, or a
	 * decoder reads it as negative. */
	if ((((v >> (8 * (unsigned int)i)) & 0xFFU) & 0x80U) != 0U) {
		tmp[n++] = 0x00U;
	}
	for (; i >= 0; i--) {
		tmp[n++] = (uint8_t)((v >> (8 * (unsigned int)i)) & 0xFFU);
	}

	return snmp_wr_tlv(w, tag, tmp, n);
}

int snmp_wr_ip(snmp_wr_t *w, uint32_t addr_be_host_order)
{
	uint8_t tmp[4];

	tmp[0] = (uint8_t)((addr_be_host_order >> 24) & 0xFFU);
	tmp[1] = (uint8_t)((addr_be_host_order >> 16) & 0xFFU);
	tmp[2] = (uint8_t)((addr_be_host_order >> 8) & 0xFFU);
	tmp[3] = (uint8_t)(addr_be_host_order & 0xFFU);
	return snmp_wr_tlv(w, SNMP_TAG_IPADDRESS, tmp, sizeof(tmp));
}

/** Octets a base-128 subidentifier needs. */
static size_t subid_size(uint64_t v)
{
	size_t n = 1U;

	while (v >= 0x80U) {
		v >>= 7;
		n++;
	}
	return n;
}

static void subid_put(uint8_t *p, size_t n, uint64_t v)
{
	size_t i;

	for (i = n; i > 0U; i--) {
		p[i - 1U] = (uint8_t)((v & 0x7FU) | ((i == n) ? 0x00U : 0x80U));
		v >>= 7;
	}
}

int snmp_wr_oid(snmp_wr_t *w, const uint32_t *arcs, size_t n)
{
	uint8_t tmp[1U + (SNMP_OID_MAX_LEN * 5U)];
	size_t used = 0U;
	uint64_t first;
	size_t i;
	size_t sz;

	if (w == NULL || arcs == NULL || n < 2U || n > SNMP_OID_MAX_LEN) {
		return -EINVAL;
	}
	if (arcs[0] > 2U) {
		return -EINVAL;
	}
	if (arcs[0] < 2U && arcs[1] >= 40U) {
		return -EINVAL;
	}

	first = ((uint64_t)arcs[0] * 40U) + (uint64_t)arcs[1];
	sz = subid_size(first);
	subid_put(&tmp[used], sz, first);
	used += sz;

	for (i = 2U; i < n; i++) {
		sz = subid_size(arcs[i]);
		subid_put(&tmp[used], sz, arcs[i]);
		used += sz;
	}

	return snmp_wr_tlv(w, SNMP_TAG_OID, tmp, used);
}

int snmp_wr_begin(snmp_wr_t *w, uint8_t tag, size_t *mark)
{
	int rc;

	if (w == NULL || mark == NULL) {
		return -EINVAL;
	}
	rc = wr_room(w, 1U + SNMP_LEN_RESERVE);
	if (rc != 0) {
		return rc;
	}
	w->buf[w->len++] = tag;
	*mark = w->len;
	w->len += SNMP_LEN_RESERVE;
	return 0;
}

int snmp_wr_end(snmp_wr_t *w, size_t mark)
{
	size_t content;
	size_t ls;

	if (w == NULL) {
		return -EINVAL;
	}
	if (mark + SNMP_LEN_RESERVE > w->len) {
		return -EINVAL;
	}
	content = w->len - (mark + SNMP_LEN_RESERVE);
	if (content > 0xFFFFU) {
		return -ENOSPC;
	}
	ls = len_size(content);
	if (ls < SNMP_LEN_RESERVE) {
		memmove(&w->buf[mark + ls], &w->buf[mark + SNMP_LEN_RESERVE],
			content);
		w->len -= (SNMP_LEN_RESERVE - ls);
	}
	len_put(&w->buf[mark], content);
	return 0;
}

int snmp_wr_rewind(snmp_wr_t *w, size_t len)
{
	if (w == NULL || len > w->len) {
		return -EINVAL;
	}
	w->len = len;
	return 0;
}

/* ========================================================================= */
/* values                                                                    */
/* ========================================================================= */

void snmp_val_int(snmp_value_t *v, int64_t x)
{
	if (v == NULL) {
		return;
	}
	memset(v, 0, sizeof(*v));
	v->type = SNMP_TAG_INTEGER;
	v->n.i = x;
}

void snmp_val_uint(snmp_value_t *v, uint8_t tag, uint64_t x)
{
	if (v == NULL) {
		return;
	}
	memset(v, 0, sizeof(*v));
	v->type = tag;
	v->n.u = x;
}

void snmp_val_ip(snmp_value_t *v, uint32_t addr)
{
	if (v == NULL) {
		return;
	}
	memset(v, 0, sizeof(*v));
	v->type = SNMP_TAG_IPADDRESS;
	v->n.u = addr;
}

void snmp_val_octets(snmp_value_t *v, const uint8_t *s, size_t n)
{
	if (v == NULL) {
		return;
	}
	memset(v, 0, sizeof(*v));
	v->type = SNMP_TAG_OCTET_STRING;
	if (n > SNMP_OCTET_MAX) {
		n = SNMP_OCTET_MAX;
	}
	if (n != 0U && s != NULL) {
		memcpy(v->octets, s, n);
		v->len = (uint16_t)n;
	}
}

void snmp_val_str(snmp_value_t *v, const char *s)
{
	snmp_val_octets(v, (const uint8_t *)s, (s == NULL) ? 0U : strlen(s));
}

void snmp_val_oid(snmp_value_t *v, const uint32_t *arcs, size_t n)
{
	if (v == NULL) {
		return;
	}
	memset(v, 0, sizeof(*v));
	v->type = SNMP_TAG_OID;
	if (n > SNMP_OID_MAX_LEN) {
		n = SNMP_OID_MAX_LEN;
	}
	if (n != 0U && arcs != NULL) {
		memcpy(v->oid, arcs, n * sizeof(arcs[0]));
		v->len = (uint16_t)n;
	}
}

int snmp_wr_value(snmp_wr_t *w, const snmp_value_t *v)
{
	if (w == NULL || v == NULL) {
		return -EINVAL;
	}

	switch (v->type) {
	case SNMP_TAG_INTEGER:
		return snmp_wr_int(w, SNMP_TAG_INTEGER, v->n.i);
	case SNMP_TAG_COUNTER32:
	case SNMP_TAG_GAUGE32:
	case SNMP_TAG_TIMETICKS:
	case SNMP_TAG_COUNTER64:
		return snmp_wr_uint(w, v->type, v->n.u);
	case SNMP_TAG_IPADDRESS:
		return snmp_wr_ip(w, (uint32_t)v->n.u);
	case SNMP_TAG_OCTET_STRING:
	case SNMP_TAG_OPAQUE:
		return snmp_wr_tlv(w, v->type, v->octets, v->len);
	case SNMP_TAG_OID:
		return snmp_wr_oid(w, v->oid, v->len);
	case SNMP_TAG_NULL:
	case SNMP_TAG_NO_SUCH_OBJECT:
	case SNMP_TAG_NO_SUCH_INSTANCE:
	case SNMP_TAG_END_OF_MIB_VIEW:
		return snmp_wr_empty(w, v->type);
	default:
		return snmp_wr_empty(w, SNMP_TAG_NO_SUCH_OBJECT);
	}
}

/* ========================================================================= */
/* the OID catalogue                                                          */
/* ========================================================================= */

/*
 * Every base below is written out in full so the table can be read as the MIB
 * it describes. The three shapes are:
 *
 *   OID_SYS(x)    1.3.6.1.2.1.1.x            MIB-II system group scalar
 *   OID_ENT(g,c)  1.3.6.1.4.1.<PEN>.1.g.c    enterprise scalar
 *   OID_RAIL(c)   1.3.6.1.4.1.<PEN>.1.3.1.1.c  column c of the rail table
 *
 * A scalar's instance arc is 0; a rail column's is 1..9. The table MUST stay
 * sorted by base; snmp_mib_check() is the enforcement and test_snmp calls it.
 */
#define OID_SYS(x)    { 1U, 3U, 6U, 1U, 2U, 1U, 1U, (x) }
#define OID_ENT(g, c) { 1U, 3U, 6U, 1U, 4U, 1U, SNMP_PEN, 1U, (g), (c) }
#define OID_RAIL(c)   { 1U, 3U, 6U, 1U, 4U, 1U, SNMP_PEN, 1U, 3U, 1U, 1U, (c) }

/* -- MIB-II system group --------------------------------------------------- */
static const uint32_t o_sys_descr[]     = OID_SYS(1U);
static const uint32_t o_sys_objectid[]  = OID_SYS(2U);
static const uint32_t o_sys_uptime[]    = OID_SYS(3U);
static const uint32_t o_sys_contact[]   = OID_SYS(4U);
static const uint32_t o_sys_name[]      = OID_SYS(5U);
static const uint32_t o_sys_location[]  = OID_SYS(6U);
static const uint32_t o_sys_services[]  = OID_SYS(7U);

/* -- enterprise .1.1 timing ------------------------------------------------ */
static const uint32_t o_stratum[]       = OID_ENT(1U, 1U);
static const uint32_t o_lock_state[]    = OID_ENT(1U, 2U);
static const uint32_t o_active_ref[]    = OID_ENT(1U, 3U);
static const uint32_t o_holdover[]      = OID_ENT(1U, 4U);
static const uint32_t o_ho_est_err[]    = OID_ENT(1U, 5U);
static const uint32_t o_ho_elapsed[]    = OID_ENT(1U, 6U);
static const uint32_t o_ho_demote[]     = OID_ENT(1U, 7U);
static const uint32_t o_pps_off[]       = OID_ENT(1U, 8U);
static const uint32_t o_pps_mean[]      = OID_ENT(1U, 9U);
static const uint32_t o_pps_sigma[]     = OID_ENT(1U, 10U);
static const uint32_t o_freq_err[]      = OID_ENT(1U, 11U);
static const uint32_t o_vc_cmd[]        = OID_ENT(1U, 12U);
static const uint32_t o_vc_sense[]      = OID_ENT(1U, 13U);
static const uint32_t o_adev_1s[]       = OID_ENT(1U, 14U);
static const uint32_t o_adev_10s[]      = OID_ENT(1U, 15U);
static const uint32_t o_adev_100s[]     = OID_ENT(1U, 16U);
static const uint32_t o_osc_temp[]      = OID_ENT(1U, 17U);

/* -- enterprise .1.2 gnss -------------------------------------------------- */
static const uint32_t o_gnss_fix[]      = OID_ENT(2U, 1U);
static const uint32_t o_gnss_used[]     = OID_ENT(2U, 2U);
static const uint32_t o_gnss_vis[]      = OID_ENT(2U, 3U);
static const uint32_t o_gnss_tacc[]     = OID_ENT(2U, 4U);
static const uint32_t o_gnss_ant[]      = OID_ENT(2U, 5U);
static const uint32_t o_leap_pending[]  = OID_ENT(2U, 6U);
static const uint32_t o_leap_current[]  = OID_ENT(2U, 7U);

/* -- enterprise .1.3.1.1 rail table ---------------------------------------- */
static const uint32_t o_rail_index[]    = OID_RAIL(1U);
static const uint32_t o_rail_name[]     = OID_RAIL(2U);
static const uint32_t o_rail_mv[]       = OID_RAIL(3U);
static const uint32_t o_rail_ua[]       = OID_RAIL(4U);
static const uint32_t o_rail_mw[]       = OID_RAIL(5U);

/* -- enterprise .1.4 thermal / environment --------------------------------- */
static const uint32_t o_temp_encl[]     = OID_ENT(4U, 1U);
static const uint32_t o_temp_osc[]      = OID_ENT(4U, 2U);
static const uint32_t o_temp_die[]      = OID_ENT(4U, 3U);
static const uint32_t o_humidity[]      = OID_ENT(4U, 4U);
static const uint32_t o_fan_rpm[]       = OID_ENT(4U, 5U);
static const uint32_t o_fan_duty[]      = OID_ENT(4U, 6U);

/* -- enterprise .1.5 time services ----------------------------------------- */
static const uint32_t o_ntp_rx[]        = OID_ENT(5U, 1U);
static const uint32_t o_ntp_served[]    = OID_ENT(5U, 2U);
static const uint32_t o_ntp_dropped[]   = OID_ENT(5U, 3U);
static const uint32_t o_ntp_kod[]       = OID_ENT(5U, 4U);
static const uint32_t o_ntp_authfail[]  = OID_ENT(5U, 5U);
static const uint32_t o_ntp_ratelim[]   = OID_ENT(5U, 6U);
static const uint32_t o_nts_rx[]        = OID_ENT(5U, 7U);
static const uint32_t o_nts_ok[]        = OID_ENT(5U, 8U);
static const uint32_t o_nts_nak[]       = OID_ENT(5U, 9U);
static const uint32_t o_nts_cookies[]   = OID_ENT(5U, 10U);
static const uint32_t o_ntske_hs[]      = OID_ENT(5U, 11U);

/* -- enterprise .1.6 ptp --------------------------------------------------- */
static const uint32_t o_ptp_state[]     = OID_ENT(6U, 1U);
static const uint32_t o_ptp_class[]     = OID_ENT(6U, 2U);
static const uint32_t o_ptp_accuracy[]  = OID_ENT(6U, 3U);
static const uint32_t o_ptp_domain[]    = OID_ENT(6U, 4U);
static const uint32_t o_ptp_tx[]        = OID_ENT(6U, 5U);
static const uint32_t o_ptp_rx[]        = OID_ENT(6U, 6U);
static const uint32_t o_ptp_anntmo[]    = OID_ENT(6U, 7U);
static const uint32_t o_ptp_alarms[]    = OID_ENT(6U, 8U);

/* -- enterprise .1.7 health ------------------------------------------------ */
static const uint32_t o_alarms[]        = OID_ENT(7U, 1U);
static const uint32_t o_fw_version[]    = OID_ENT(7U, 2U);
static const uint32_t o_gnss_fw[]       = OID_ENT(7U, 3U);
static const uint32_t o_uptime_s[]      = OID_ENT(7U, 4U);
static const uint32_t o_poe_class[]     = OID_ENT(7U, 5U);
static const uint32_t o_poe_draw[]      = OID_ENT(7U, 6U);
static const uint32_t o_cap_stm[]       = OID_ENT(7U, 7U);
static const uint32_t o_cap_gps[]       = OID_ENT(7U, 8U);
static const uint32_t o_serial[]        = OID_ENT(7U, 9U);

#define NODE(arr, tag, obj, ninst)                                             \
	{                                                                      \
		(arr), (uint8_t)(sizeof(arr) / sizeof((arr)[0])), (tag),        \
			(uint16_t)(obj), (uint16_t)(ninst)                      \
	}
#define SCALAR(arr, tag, obj) NODE(arr, tag, obj, 1U)
#define RAILCOL(arr, tag, obj) NODE(arr, tag, obj, SNMP_RAIL_COUNT)

static const snmp_node_t mib[] = {
	/* MIB-II system group */
	SCALAR(o_sys_descr, SNMP_TAG_OCTET_STRING, SNMP_OBJ_SYS_DESCR),
	SCALAR(o_sys_objectid, SNMP_TAG_OID, SNMP_OBJ_SYS_OBJECT_ID),
	SCALAR(o_sys_uptime, SNMP_TAG_TIMETICKS, SNMP_OBJ_SYS_UPTIME),
	SCALAR(o_sys_contact, SNMP_TAG_OCTET_STRING, SNMP_OBJ_SYS_CONTACT),
	SCALAR(o_sys_name, SNMP_TAG_OCTET_STRING, SNMP_OBJ_SYS_NAME),
	SCALAR(o_sys_location, SNMP_TAG_OCTET_STRING, SNMP_OBJ_SYS_LOCATION),
	SCALAR(o_sys_services, SNMP_TAG_INTEGER, SNMP_OBJ_SYS_SERVICES),

	/* timing */
	SCALAR(o_stratum, SNMP_TAG_INTEGER, SNMP_OBJ_STRATUM),
	SCALAR(o_lock_state, SNMP_TAG_INTEGER, SNMP_OBJ_LOCK_STATE),
	SCALAR(o_active_ref, SNMP_TAG_INTEGER, SNMP_OBJ_ACTIVE_REF),
	SCALAR(o_holdover, SNMP_TAG_INTEGER, SNMP_OBJ_HOLDOVER),
	SCALAR(o_ho_est_err, SNMP_TAG_INTEGER, SNMP_OBJ_HOLDOVER_EST_ERR_NS),
	SCALAR(o_ho_elapsed, SNMP_TAG_GAUGE32, SNMP_OBJ_HOLDOVER_ELAPSED_S),
	SCALAR(o_ho_demote, SNMP_TAG_GAUGE32, SNMP_OBJ_HOLDOVER_DEMOTE_S),
	SCALAR(o_pps_off, SNMP_TAG_INTEGER, SNMP_OBJ_PPS_OFFSET_NS),
	SCALAR(o_pps_mean, SNMP_TAG_INTEGER, SNMP_OBJ_PPS_MEAN_NS),
	SCALAR(o_pps_sigma, SNMP_TAG_GAUGE32, SNMP_OBJ_PPS_SIGMA_NS),
	SCALAR(o_freq_err, SNMP_TAG_INTEGER, SNMP_OBJ_FREQ_ERR_PPT),
	SCALAR(o_vc_cmd, SNMP_TAG_INTEGER, SNMP_OBJ_VC_CMD_MV),
	SCALAR(o_vc_sense, SNMP_TAG_INTEGER, SNMP_OBJ_VC_SENSE_MV),
	SCALAR(o_adev_1s, SNMP_TAG_GAUGE32, SNMP_OBJ_ADEV_1S_E18),
	SCALAR(o_adev_10s, SNMP_TAG_GAUGE32, SNMP_OBJ_ADEV_10S_E18),
	SCALAR(o_adev_100s, SNMP_TAG_GAUGE32, SNMP_OBJ_ADEV_100S_E18),
	SCALAR(o_osc_temp, SNMP_TAG_INTEGER, SNMP_OBJ_OSC_TEMP_MC),

	/* gnss */
	SCALAR(o_gnss_fix, SNMP_TAG_INTEGER, SNMP_OBJ_GNSS_FIX),
	SCALAR(o_gnss_used, SNMP_TAG_GAUGE32, SNMP_OBJ_GNSS_SV_USED),
	SCALAR(o_gnss_vis, SNMP_TAG_GAUGE32, SNMP_OBJ_GNSS_SV_VISIBLE),
	SCALAR(o_gnss_tacc, SNMP_TAG_GAUGE32, SNMP_OBJ_GNSS_TACC_NS),
	SCALAR(o_gnss_ant, SNMP_TAG_INTEGER, SNMP_OBJ_GNSS_ANT_STATE),
	SCALAR(o_leap_pending, SNMP_TAG_INTEGER, SNMP_OBJ_GNSS_LEAP_PENDING),
	SCALAR(o_leap_current, SNMP_TAG_INTEGER, SNMP_OBJ_GNSS_LEAP_CURRENT_S),

	/* power: the nine-row rail table, column-major, which is also
	 * lexicographic OID order */
	RAILCOL(o_rail_index, SNMP_TAG_INTEGER, SNMP_OBJ_RAIL_INDEX),
	RAILCOL(o_rail_name, SNMP_TAG_OCTET_STRING, SNMP_OBJ_RAIL_NAME),
	RAILCOL(o_rail_mv, SNMP_TAG_GAUGE32, SNMP_OBJ_RAIL_MV),
	RAILCOL(o_rail_ua, SNMP_TAG_INTEGER, SNMP_OBJ_RAIL_UA),
	RAILCOL(o_rail_mw, SNMP_TAG_GAUGE32, SNMP_OBJ_RAIL_MW),

	/* thermal / environment */
	SCALAR(o_temp_encl, SNMP_TAG_INTEGER, SNMP_OBJ_TEMP_ENCLOSURE_MC),
	SCALAR(o_temp_osc, SNMP_TAG_INTEGER, SNMP_OBJ_TEMP_OSC_MC),
	SCALAR(o_temp_die, SNMP_TAG_INTEGER, SNMP_OBJ_TEMP_DIE_MC),
	SCALAR(o_humidity, SNMP_TAG_GAUGE32, SNMP_OBJ_HUMIDITY_MPCT),
	SCALAR(o_fan_rpm, SNMP_TAG_GAUGE32, SNMP_OBJ_FAN_RPM),
	SCALAR(o_fan_duty, SNMP_TAG_GAUGE32, SNMP_OBJ_FAN_DUTY_PCT),

	/* time services */
	SCALAR(o_ntp_rx, SNMP_TAG_COUNTER64, SNMP_OBJ_NTP_RX),
	SCALAR(o_ntp_served, SNMP_TAG_COUNTER64, SNMP_OBJ_NTP_SERVED),
	SCALAR(o_ntp_dropped, SNMP_TAG_COUNTER64, SNMP_OBJ_NTP_DROPPED),
	SCALAR(o_ntp_kod, SNMP_TAG_COUNTER64, SNMP_OBJ_NTP_KOD),
	SCALAR(o_ntp_authfail, SNMP_TAG_COUNTER64, SNMP_OBJ_NTP_AUTH_FAIL),
	SCALAR(o_ntp_ratelim, SNMP_TAG_COUNTER64, SNMP_OBJ_NTP_RATE_LIMITED),
	SCALAR(o_nts_rx, SNMP_TAG_COUNTER64, SNMP_OBJ_NTS_RX),
	SCALAR(o_nts_ok, SNMP_TAG_COUNTER64, SNMP_OBJ_NTS_OK),
	SCALAR(o_nts_nak, SNMP_TAG_COUNTER64, SNMP_OBJ_NTS_NAK),
	SCALAR(o_nts_cookies, SNMP_TAG_COUNTER64, SNMP_OBJ_NTS_COOKIES),
	SCALAR(o_ntske_hs, SNMP_TAG_COUNTER64, SNMP_OBJ_NTSKE_HANDSHAKES),

	/* ptp */
	SCALAR(o_ptp_state, SNMP_TAG_INTEGER, SNMP_OBJ_PTP_PORT_STATE),
	SCALAR(o_ptp_class, SNMP_TAG_GAUGE32, SNMP_OBJ_PTP_CLOCK_CLASS),
	SCALAR(o_ptp_accuracy, SNMP_TAG_GAUGE32, SNMP_OBJ_PTP_CLOCK_ACCURACY),
	SCALAR(o_ptp_domain, SNMP_TAG_GAUGE32, SNMP_OBJ_PTP_DOMAIN),
	SCALAR(o_ptp_tx, SNMP_TAG_COUNTER64, SNMP_OBJ_PTP_TX),
	SCALAR(o_ptp_rx, SNMP_TAG_COUNTER64, SNMP_OBJ_PTP_RX),
	SCALAR(o_ptp_anntmo, SNMP_TAG_COUNTER32, SNMP_OBJ_PTP_ANN_TIMEOUTS),
	SCALAR(o_ptp_alarms, SNMP_TAG_GAUGE32, SNMP_OBJ_PTP_ALARMS),

	/* health */
	SCALAR(o_alarms, SNMP_TAG_GAUGE32, SNMP_OBJ_ALARMS),
	SCALAR(o_fw_version, SNMP_TAG_OCTET_STRING, SNMP_OBJ_FW_VERSION),
	SCALAR(o_gnss_fw, SNMP_TAG_OCTET_STRING, SNMP_OBJ_GNSS_FW_VERSION),
	SCALAR(o_uptime_s, SNMP_TAG_GAUGE32, SNMP_OBJ_UPTIME_S),
	SCALAR(o_poe_class, SNMP_TAG_INTEGER, SNMP_OBJ_POE_CLASS),
	SCALAR(o_poe_draw, SNMP_TAG_GAUGE32, SNMP_OBJ_POE_DRAW_MW),
	SCALAR(o_cap_stm, SNMP_TAG_GAUGE32, SNMP_OBJ_SUPERCAP_STM_MV),
	SCALAR(o_cap_gps, SNMP_TAG_GAUGE32, SNMP_OBJ_SUPERCAP_GPS_MV),
	SCALAR(o_serial, SNMP_TAG_OCTET_STRING, SNMP_OBJ_SERIAL),
};

#define MIB_COUNT (sizeof(mib) / sizeof(mib[0]))

/* Notification OIDs. coldStart is the standard snmpTraps.coldStart; the rest
 * live under <enterprise>.1.0.<n>, the SMIv2 placement that keeps them out of
 * the subtree a manager walks. */
static const uint32_t o_cold_start[] = { 1U, 3U, 6U, 1U, 6U, 3U, 1U, 1U, 5U, 1U };
static const uint32_t o_trap_base[] = { 1U, 3U, 6U, 1U, 4U, 1U, SNMP_PEN, 1U, 0U };
const uint32_t snmp__oid_trap_oid[11] = {
	1U, 3U, 6U, 1U, 6U, 3U, 1U, 1U, 4U, 1U, 0U
};

size_t snmp_mib_node_count(void)
{
	return MIB_COUNT;
}

const snmp_node_t *snmp_mib_node(size_t i)
{
	return (i < MIB_COUNT) ? &mib[i] : NULL;
}

size_t snmp_mib_instance_count(void)
{
	size_t total = 0U;
	size_t i;

	for (i = 0U; i < MIB_COUNT; i++) {
		total += mib[i].n_inst;
	}
	return total;
}

/** Numeric arc-by-arc comparison; a proper prefix sorts first. */
static int oid_cmp(const uint32_t *a, size_t na, const uint32_t *b, size_t nb)
{
	size_t n = (na < nb) ? na : nb;
	size_t i;

	for (i = 0U; i < n; i++) {
		if (a[i] != b[i]) {
			return (a[i] < b[i]) ? -1 : 1;
		}
	}
	if (na == nb) {
		return 0;
	}
	return (na < nb) ? -1 : 1;
}

/** Locate the node and instance arc of flat index @p idx. */
static const snmp_node_t *mib_at(size_t idx, uint16_t *out_inst)
{
	size_t i;

	for (i = 0U; i < MIB_COUNT; i++) {
		if (idx < mib[i].n_inst) {
			/* A scalar's single instance is arc 0; a table
			 * column's rows are arcs 1..n_inst. */
			*out_inst = (mib[i].n_inst == 1U)
					    ? 0U
					    : (uint16_t)(idx + 1U);
			return &mib[i];
		}
		idx -= mib[i].n_inst;
	}
	return NULL;
}

/** Materialise the full OID of a node/instance pair. */
static size_t node_oid(const snmp_node_t *n, uint16_t inst, uint32_t *arcs)
{
	memcpy(arcs, n->base, (size_t)n->base_len * sizeof(arcs[0]));
	arcs[n->base_len] = inst;
	return (size_t)n->base_len + 1U;
}

int snmp_mib_instance(size_t idx, uint16_t *out_obj, uint16_t *out_inst,
		      uint8_t *out_type, uint32_t *arcs)
{
	const snmp_node_t *n;
	uint16_t inst = 0U;

	if (arcs == NULL) {
		return -EINVAL;
	}
	n = mib_at(idx, &inst);
	if (n == NULL) {
		return -ENOENT;
	}
	if (out_obj != NULL) {
		*out_obj = n->obj;
	}
	if (out_inst != NULL) {
		*out_inst = inst;
	}
	if (out_type != NULL) {
		*out_type = n->type;
	}
	return (int)node_oid(n, inst, arcs);
}

int snmp_mib_find(const uint32_t *arcs, size_t n)
{
	uint32_t full[SNMP_OID_MAX_LEN];
	size_t total;
	size_t i;

	if (arcs == NULL) {
		return -EINVAL;
	}
	total = snmp_mib_instance_count();
	for (i = 0U; i < total; i++) {
		int len = snmp_mib_instance(i, NULL, NULL, NULL, full);
		int cmp;

		if (len < 0) {
			break;
		}
		cmp = oid_cmp(full, (size_t)len, arcs, n);
		if (cmp == 0) {
			return (int)i;
		}
		if (cmp > 0) {
			break; /* the table is sorted: no later entry can match */
		}
	}
	return -ENOENT;
}

int snmp_mib_find_next(const uint32_t *arcs, size_t n)
{
	uint32_t full[SNMP_OID_MAX_LEN];
	size_t total;
	size_t i;

	if (arcs == NULL) {
		return -EINVAL;
	}
	total = snmp_mib_instance_count();
	for (i = 0U; i < total; i++) {
		int len = snmp_mib_instance(i, NULL, NULL, NULL, full);

		if (len < 0) {
			break;
		}
		if (oid_cmp(full, (size_t)len, arcs, n) > 0) {
			return (int)i;
		}
	}
	return -ENOENT;
}

int snmp_mib_oid_of(uint16_t obj, uint16_t inst, uint32_t *arcs)
{
	size_t i;

	if (arcs == NULL) {
		return -EINVAL;
	}
	for (i = 0U; i < MIB_COUNT; i++) {
		if (mib[i].obj != obj) {
			continue;
		}
		if (mib[i].n_inst == 1U) {
			if (inst != 0U) {
				return -ENOENT;
			}
		} else if (inst == 0U || inst > mib[i].n_inst) {
			return -ENOENT;
		}
		return (int)node_oid(&mib[i], inst, arcs);
	}
	return -ENOENT;
}

int snmp_mib_check(void)
{
	size_t i;

	for (i = 0U; i < MIB_COUNT; i++) {
		if (mib[i].base == NULL || mib[i].base_len < 2U) {
			return -(int)(i + 1U);
		}
		if ((size_t)mib[i].base_len + 1U > SNMP_OID_MAX_LEN) {
			return -(int)(i + 1U);
		}
		if (mib[i].n_inst == 0U) {
			return -(int)(i + 1U);
		}
		if (mib[i].obj >= (uint16_t)SNMP_OBJ__COUNT) {
			return -(int)(i + 1U);
		}
		if (i > 0U &&
		    oid_cmp(mib[i - 1U].base, mib[i - 1U].base_len, mib[i].base,
			    mib[i].base_len) >= 0) {
			return -(int)(i + 1U);
		}
	}
	return 0;
}

/* ========================================================================= */
/* agent                                                                     */
/* ========================================================================= */

int snmp_init(snmp_ctx_t *c, const snmp_cfg_t *cfg)
{
	if (c == NULL || cfg == NULL || cfg->getter.get == NULL) {
		return -EINVAL;
	}
	memset(c, 0, sizeof(*c));
	c->cfg = *cfg;
	if (c->cfg.max_repetitions == 0U ||
	    c->cfg.max_repetitions > SNMP_MAX_REPETITIONS) {
		c->cfg.max_repetitions = SNMP_MAX_REPETITIONS;
	}
	c->ready = true;
	return 0;
}

int snmp_set_community(snmp_ctx_t *c, const char *community)
{
	if (c == NULL || !c->ready) {
		return -EINVAL;
	}
	c->cfg.community = community;
	return 0;
}

int snmp_stats_get(const snmp_ctx_t *c, snmp_stats_t *out)
{
	if (c == NULL || out == NULL) {
		return -EINVAL;
	}
	*out = c->stats;
	return 0;
}

int snmp_stats_reset(snmp_ctx_t *c)
{
	if (c == NULL) {
		return -EINVAL;
	}
	memset(&c->stats, 0, sizeof(c->stats));
	return 0;
}

/* -------------------------------------------------------------- resolution */

/**
 * The community string to emit, never NULL.
 *
 * `snmp_set_community(c, NULL)` is documented to disable the agent, and
 * parse_request() enforces that by refusing every request before a PDU is even
 * parsed. But the configuration pointer is replaced by a *different* thread (the
 * cfg-commit applier calls snmp_set_community() while the agent thread may be
 * mid-request), so a NULL can appear between the parse and the reply. Rather
 * than reason about that window, every read of the community goes through here.
 */
static const char *community_of(const snmp_ctx_t *c)
{
	return (c->cfg.community != NULL) ? c->cfg.community : "";
}

/** Resolve (obj, inst) through the getter, with sysUpTime supplied locally. */
void snmp__resolve(snmp_ctx_t *c, uint16_t obj, uint16_t inst,
		   uint32_t uptime_cs, snmp_value_t *v)
{
	memset(v, 0, sizeof(*v));

	if (obj == (uint16_t)SNMP_OBJ_SYS_UPTIME) {
		/* Owned here so a response and a trap can never disagree about
		 * the uptime the manager correlates them by. */
		snmp_val_uint(v, SNMP_TAG_TIMETICKS, uptime_cs);
		return;
	}
	if (c->cfg.getter.get(c->cfg.getter.ctx, obj, inst, v) != 0) {
		v->type = SNMP_TAG_NO_SUCH_INSTANCE;
		v->len = 0U;
	}
}

/**
 * Does @p arcs name an object that exists but an instance that does not?
 *
 * SNMPv2 distinguishes noSuchObject ("nothing in the view is called that") from
 * noSuchInstance ("the object exists; that instance does not"), and a manager
 * uses the difference to tell a MIB mismatch from an absent row.
 */
static bool names_known_object(const uint32_t *arcs, size_t n)
{
	size_t i;

	for (i = 0U; i < MIB_COUNT; i++) {
		if (n >= mib[i].base_len &&
		    oid_cmp(arcs, mib[i].base_len, mib[i].base,
			    mib[i].base_len) == 0) {
			return true;
		}
	}
	return false;
}

/* ------------------------------------------------------------ request state */

/* snmp_vb_t and snmp_req_t live in snmp_internal.h so snmp_v3.c can reuse the
 * PDU layer verbatim — see that header for why. */

/** Append one varbind: name then value. */
int snmp__put_varbind(snmp_wr_t *w, const uint32_t *arcs, size_t n,
		      const snmp_value_t *v)
{
	size_t mark;
	int rc;

	rc = snmp_wr_begin(w, SNMP_TAG_SEQUENCE, &mark);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_oid(w, arcs, n);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_value(w, v);
	if (rc != 0) {
		return rc;
	}
	return snmp_wr_end(w, mark);
}

/** GET: one varbind, exact match. */
static int emit_get(snmp_ctx_t *c, snmp_wr_t *w, snmp_vb_t *vb, uint32_t uptime_cs)
{
	snmp_value_t v;
	uint16_t obj = 0U;
	uint16_t inst = 0U;
	int idx;

	memset(&v, 0, sizeof(v));

	if (vb->over) {
		v.type = SNMP_TAG_NO_SUCH_OBJECT;
	} else {
		idx = snmp_mib_find(vb->arcs, vb->n);
		if (idx < 0) {
			v.type = names_known_object(vb->arcs, vb->n)
					 ? SNMP_TAG_NO_SUCH_INSTANCE
					 : SNMP_TAG_NO_SUCH_OBJECT;
		} else {
			uint32_t full[SNMP_OID_MAX_LEN];

			(void)snmp_mib_instance((size_t)idx, &obj, &inst, NULL,
						full);
			snmp__resolve(c, obj, inst, uptime_cs, &v);
		}
	}

	return snmp__put_varbind(w, vb->arcs, vb->n, &v);
}

/**
 * GETNEXT for one varbind, from flat index @p from.
 *
 * @param from  Index to emit, or negative for end-of-MIB.
 * @return the index emitted, or -1 once endOfMibView has been reported.
 */
static int emit_next(snmp_ctx_t *c, snmp_wr_t *w, const snmp_vb_t *vb, int from,
		     uint32_t uptime_cs, int *rc_out)
{
	snmp_value_t v;
	uint32_t full[SNMP_OID_MAX_LEN];
	uint16_t obj = 0U;
	uint16_t inst = 0U;
	int len;

	memset(&v, 0, sizeof(v));

	if (from < 0) {
		v.type = SNMP_TAG_END_OF_MIB_VIEW;
		*rc_out = snmp__put_varbind(w, vb->arcs, vb->n, &v);
		return -1;
	}

	len = snmp_mib_instance((size_t)from, &obj, &inst, NULL, full);
	if (len < 0) {
		v.type = SNMP_TAG_END_OF_MIB_VIEW;
		*rc_out = snmp__put_varbind(w, vb->arcs, vb->n, &v);
		return -1;
	}

	snmp__resolve(c, obj, inst, uptime_cs, &v);
	*rc_out = snmp__put_varbind(w, full, (size_t)len, &v);
	return from;
}

/**
 * Build a complete response message.
 *
 * @param err  When non-zero, the varbind list is emitted empty (the tooBig
 *             shape of RFC 3416 §4.2.1/§4.2.2) — except for a refused SET,
 *             which echoes the request's own list.
 *
 * A GET or GETNEXT that does not fit returns -ENOSPC so the caller can rebuild
 * it as tooBig; a GETBULK instead stops at the last varbind that fitted, which
 * RFC 3416 §4.2.3 explicitly allows and every manager expects.
 *
 * @retval 0        Built.
 * @retval -ENOSPC  A GET/GETNEXT response did not fit.
 */
static int build_response(snmp_ctx_t *c, snmp_req_t *rq, int32_t err,
			  int32_t err_index, uint32_t uptime_cs, uint8_t *rsp,
			  size_t cap, size_t *out_len)
{
	snmp_wr_t w;
	size_t m_msg;
	size_t m_pdu;
	size_t m_vbl = 0U;
	size_t emitted = 0U;
	size_t i;
	int rc;

	rc = snmp_wr_init(&w, rsp, cap);
	if (rc != 0) {
		return rc;
	}

	rc = snmp_wr_begin(&w, SNMP_TAG_SEQUENCE, &m_msg);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_int(&w, SNMP_TAG_INTEGER, SNMP_VERSION_2C);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_tlv(&w, SNMP_TAG_OCTET_STRING,
			 (const uint8_t *)c->cfg.community,
			 strlen(c->cfg.community));
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_begin(&w, SNMP_PDU_RESPONSE, &m_pdu);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_int(&w, SNMP_TAG_INTEGER, rq->reqid);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_int(&w, SNMP_TAG_INTEGER, err);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_int(&w, SNMP_TAG_INTEGER, err_index);
	if (rc != 0) {
		return rc;
	}

	/* An error response carries the request's own varbind list for SET
	 * (RFC 3416 §4.2.5 wants it identical, and the request bytes are the
	 * only way to be certain of that) and an empty one for tooBig. */
	if (err != SNMP_ERR_NO_ERROR) {
		if (rq->pdu == SNMP_PDU_SET && err != SNMP_ERR_TOO_BIG &&
		    rq->vbl != NULL) {
			rc = snmp_wr_tlv(&w, SNMP_TAG_SEQUENCE, rq->vbl,
					 rq->vbl_len);
		} else {
			rc = snmp_wr_tlv(&w, SNMP_TAG_SEQUENCE, NULL, 0U);
		}
		if (rc != 0) {
			return rc;
		}
		goto close;
	}

	rc = snmp_wr_begin(&w, SNMP_TAG_SEQUENCE, &m_vbl);
	if (rc != 0) {
		return rc;
	}

	if (rq->pdu == SNMP_PDU_GET) {
		for (i = 0U; i < rq->n_vb; i++) {
			size_t save = snmp_wr_len(&w);

			rc = emit_get(c, &w, &rq->vb[i], uptime_cs);
			if (rc != 0) {
				(void)snmp_wr_rewind(&w, save);
				return -ENOSPC;
			}
			emitted++;
		}
	} else if (rq->pdu == SNMP_PDU_GETNEXT) {
		for (i = 0U; i < rq->n_vb; i++) {
			size_t save = snmp_wr_len(&w);
			int from;
			int vrc = 0;

			from = rq->vb[i].over
				       ? -1
				       : snmp_mib_find_next(rq->vb[i].arcs,
							    rq->vb[i].n);
			(void)emit_next(c, &w, &rq->vb[i], from, uptime_cs,
					&vrc);
			if (vrc != 0) {
				(void)snmp_wr_rewind(&w, save);
				return -ENOSPC;
			}
			emitted++;
		}
	} else { /* GETBULK */
		size_t nr;
		size_t reps;
		size_t r;
		bool stop = false;

		nr = (rq->f2 <= 0) ? 0U : (size_t)rq->f2;
		if (nr > rq->n_vb) {
			nr = rq->n_vb;
		}
		reps = (rq->f3 <= 0) ? 0U : (size_t)rq->f3;
		if (reps > c->cfg.max_repetitions) {
			reps = c->cfg.max_repetitions;
		}

		for (i = 0U; i < nr && !stop; i++) {
			size_t save = snmp_wr_len(&w);
			int from;
			int vrc = 0;

			from = rq->vb[i].over
				       ? -1
				       : snmp_mib_find_next(rq->vb[i].arcs,
							    rq->vb[i].n);
			(void)emit_next(c, &w, &rq->vb[i], from, uptime_cs,
					&vrc);
			if (vrc != 0) {
				(void)snmp_wr_rewind(&w, save);
				stop = true;
				break;
			}
			emitted++;
		}

		/* Seed each repeater's cursor from its request OID. */
		for (i = nr; i < rq->n_vb; i++) {
			rq->vb[i].next =
				rq->vb[i].over
					? -1
					: snmp_mib_find_next(rq->vb[i].arcs,
							     rq->vb[i].n);
		}

		for (r = 0U; r < reps && !stop; r++) {
			bool any = false;

			for (i = nr; i < rq->n_vb; i++) {
				size_t save = snmp_wr_len(&w);
				int vrc = 0;
				int used;

				used = emit_next(c, &w, &rq->vb[i],
						 rq->vb[i].next, uptime_cs,
						 &vrc);
				if (vrc != 0) {
					(void)snmp_wr_rewind(&w, save);
					stop = true;
					break;
				}
				emitted++;
				if (used >= 0) {
					rq->vb[i].next = used + 1;
					if ((size_t)rq->vb[i].next >=
					    snmp_mib_instance_count()) {
						rq->vb[i].next = -1;
					}
					any = true;
				}
			}
			/* Every repeater has reported endOfMibView: further
			 * rounds would only repeat it (RFC 3416 §4.2.3). */
			if (!any) {
				break;
			}
		}
	}

	rc = snmp_wr_end(&w, m_vbl);
	if (rc != 0) {
		return rc;
	}

close:
	rc = snmp_wr_end(&w, m_pdu);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_end(&w, m_msg);
	if (rc != 0) {
		return rc;
	}

	c->stats.varbinds += emitted;
	*out_len = snmp_wr_len(&w);
	return 0;
}

/** Parse the message envelope and PDU into @p rq. */
static int parse_request(snmp_ctx_t *c, const uint8_t *req, size_t req_len,
			 snmp_req_t *rq)
{
	snmp_rd_t r;
	snmp_rd_t body;
	snmp_rd_t pdu;
	snmp_rd_t vbl;
	uint8_t tag;
	size_t len;
	int64_t v64;
	const uint8_t *comm;
	size_t comm_len;
	size_t clen;
	int rc;

	rc = snmp_rd_init(&r, req, req_len);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_rd_hdr(&r, &tag, &len);
	if (rc != 0 || tag != SNMP_TAG_SEQUENCE) {
		return -EBADMSG;
	}
	rc = snmp_rd_init(&body, &req[r.off], len);
	if (rc != 0) {
		return rc;
	}

	rc = snmp_rd_int(&body, &v64);
	if (rc != 0) {
		return -EBADMSG;
	}
	if (v64 != SNMP_VERSION_2C) {
		return -EPROTO;
	}

	rc = snmp_rd_octets(&body, &comm, &comm_len);
	if (rc != 0) {
		return -EBADMSG;
	}
	if (c->cfg.community == NULL) {
		return -EACCES;
	}
	clen = strlen(c->cfg.community);
	if (clen == 0U || clen != comm_len ||
	    memcmp(comm, c->cfg.community, clen) != 0) {
		return -EACCES;
	}

	rc = snmp_rd_hdr(&body, &tag, &len);
	if (rc != 0) {
		return -EBADMSG;
	}
	switch (tag) {
	case SNMP_PDU_GET:
	case SNMP_PDU_GETNEXT:
	case SNMP_PDU_GETBULK:
	case SNMP_PDU_SET:
		break;
	default:
		return -ENOTSUP;
	}
	rq->pdu = tag;

	rc = snmp_rd_init(&pdu, &body.buf[body.off], len);
	if (rc != 0) {
		return rc;
	}

	rc = snmp_rd_int(&pdu, &v64);
	if (rc != 0) {
		return -EBADMSG;
	}
	/* request-id is Integer32 (RFC 3416 §3); a wider value is a malformed
	 * PDU, and echoing a truncation would break the manager's matching. */
	if (v64 < INT32_MIN || v64 > INT32_MAX) {
		return -EBADMSG;
	}
	rq->reqid = (int32_t)v64;

	rc = snmp_rd_int(&pdu, &v64);
	if (rc != 0) {
		return -EBADMSG;
	}
	rq->f2 = (v64 < INT32_MIN) ? INT32_MIN
				   : ((v64 > INT32_MAX) ? INT32_MAX
							: (int32_t)v64);

	rc = snmp_rd_int(&pdu, &v64);
	if (rc != 0) {
		return -EBADMSG;
	}
	rq->f3 = (v64 < INT32_MIN) ? INT32_MIN
				   : ((v64 > INT32_MAX) ? INT32_MAX
							: (int32_t)v64);

	rc = snmp_rd_hdr(&pdu, &tag, &len);
	if (rc != 0 || tag != SNMP_TAG_SEQUENCE) {
		return -EBADMSG;
	}
	rq->vbl = &pdu.buf[pdu.off];
	rq->vbl_len = len;

	rc = snmp_rd_init(&vbl, rq->vbl, len);
	if (rc != 0) {
		return rc;
	}

	while (!snmp_rd_done(&vbl)) {
		snmp_rd_t one;
		size_t n = 0U;
		bool over = false;

		rc = snmp_rd_hdr(&vbl, &tag, &len);
		if (rc != 0 || tag != SNMP_TAG_SEQUENCE) {
			return -EBADMSG;
		}
		rc = snmp_rd_init(&one, &vbl.buf[vbl.off], len);
		if (rc != 0) {
			return rc;
		}
		rc = snmp_rd_skip(&vbl, len);
		if (rc != 0) {
			return -EBADMSG;
		}

		if (rq->n_vb >= SNMP_MAX_VARBINDS) {
			/* Reported as tooBig by the caller: the request is
			 * well formed, we simply will not answer that many. */
			return -E2BIG;
		}

		rc = snmp_rd_oid(&one, rq->vb[rq->n_vb].arcs, SNMP_OID_MAX_LEN,
				 &n, &over);
		if (rc != 0) {
			return -EBADMSG;
		}
		/* The value in a request is ignored (it is NULL in a Get and
		 * unusable in a Set this agent refuses), but it must parse. */
		rc = snmp_rd_hdr(&one, &tag, &len);
		if (rc != 0) {
			return -EBADMSG;
		}

		rq->vb[rq->n_vb].n = (uint8_t)n;
		rq->vb[rq->n_vb].over = over;
		rq->vb[rq->n_vb].next = -1;
		rq->n_vb++;
	}

	return 0;
}

int snmp_handle(snmp_ctx_t *c, const uint8_t *req, size_t req_len,
		uint32_t uptime_cs, uint8_t *rsp, size_t rsp_cap,
		size_t *rsp_len)
{
	snmp_req_t rq;
	size_t out = 0U;
	int rc;

	if (c == NULL || !c->ready || req == NULL || rsp == NULL ||
	    rsp_len == NULL) {
		return -EINVAL;
	}
	if (req_len == 0U || req_len > SNMP_PKT_MAX) {
		c->stats.rx++;
		c->stats.malformed++;
		return -EBADMSG;
	}
	if (rsp_cap < 32U) {
		return -ENOSPC;
	}

	c->stats.rx++;
	memset(&rq, 0, sizeof(rq));

	rc = parse_request(c, req, req_len, &rq);
	switch (rc) {
	case 0:
		break;
	case -EPROTO:
		c->stats.bad_version++;
		return -EPROTO;
	case -EACCES:
		c->stats.bad_community++;
		return -EACCES;
	case -ENOTSUP:
		c->stats.unsupported_pdu++;
		return -ENOTSUP;
	case -E2BIG:
		/* Too many varbinds to answer: a tooBig response is the
		 * standard way to say so, and it needs the request-id, which
		 * parse_request() has already recovered. */
		c->stats.too_big++;
		rc = build_response(c, &rq, SNMP_ERR_TOO_BIG, 0, uptime_cs, rsp,
				    rsp_cap, &out);
		if (rc != 0) {
			return -ENOSPC;
		}
		c->stats.responses++;
		*rsp_len = out;
		return 0;
	default:
		c->stats.malformed++;
		return -EBADMSG;
	}

	if (rq.pdu == SNMP_PDU_SET) {
		c->stats.set_refused++;
		rc = build_response(c, &rq, SNMP_ERR_NOT_WRITABLE,
				    (rq.n_vb > 0U) ? 1 : 0, uptime_cs, rsp,
				    rsp_cap, &out);
		if (rc != 0) {
			return -ENOSPC;
		}
		c->stats.responses++;
		*rsp_len = out;
		return 0;
	}

	if (rq.pdu == SNMP_PDU_GET) {
		c->stats.get++;
	} else if (rq.pdu == SNMP_PDU_GETNEXT) {
		c->stats.getnext++;
	} else {
		c->stats.getbulk++;
	}

	rc = build_response(c, &rq, SNMP_ERR_NO_ERROR, 0, uptime_cs, rsp,
			    rsp_cap, &out);
	if (rc == -ENOSPC) {
		/* RFC 3416 §4.2.1/§4.2.2: a Get or GetNext whose response does
		 * not fit is answered with tooBig and an empty varbind list.
		 * GetBulk instead returns what fitted, so it never lands here. */
		c->stats.too_big++;
		rc = build_response(c, &rq, SNMP_ERR_TOO_BIG, 0, uptime_cs, rsp,
				    rsp_cap, &out);
	}
	if (rc != 0) {
		return -ENOSPC;
	}

	c->stats.responses++;
	*rsp_len = out;
	return 0;
}

/* ========================================================================= */
/* traps                                                                     */
/* ========================================================================= */

static const char *const trap_names[SNMP_TRAP__COUNT] = {
	"coldStart",     "lockAcquired",  "lockLost",     "holdoverEnter",
	"holdoverExit",  "refSwitch",     "railAlarm",    "thermalAlarm",
	"antennaFault",  "authFailure",
};

const char *snmp_trap_name(snmp_trap_t t)
{
	if ((unsigned int)t >= (unsigned int)SNMP_TRAP__COUNT) {
		return "unknown";
	}
	return trap_names[t];
}

int snmp_trap_oid(snmp_trap_t t, uint32_t *arcs)
{
	size_t base_n;

	if (arcs == NULL || (unsigned int)t >= (unsigned int)SNMP_TRAP__COUNT) {
		return -EINVAL;
	}
	if (t == SNMP_TRAP_COLD_START) {
		base_n = sizeof(o_cold_start) / sizeof(o_cold_start[0]);
		memcpy(arcs, o_cold_start, base_n * sizeof(arcs[0]));
		return (int)base_n;
	}

	base_n = sizeof(o_trap_base) / sizeof(o_trap_base[0]);
	memcpy(arcs, o_trap_base, base_n * sizeof(arcs[0]));
	arcs[base_n] = (uint32_t)t; /* .0.1 .. .0.9, coldStart excluded */
	return (int)(base_n + 1U);
}

int snmp_make_trap(snmp_ctx_t *c, snmp_trap_t t, uint32_t uptime_cs,
		   const snmp_bind_t *binds, size_t n_binds, uint8_t *out,
		   size_t cap, size_t *out_len)
{
	snmp_wr_t w;
	snmp_value_t v;
	uint32_t arcs[SNMP_OID_MAX_LEN];
	size_t m_msg;
	size_t m_pdu;
	size_t m_vbl;
	size_t i;
	int n;
	int rc;

	if (c == NULL || !c->ready || out == NULL || out_len == NULL) {
		return -EINVAL;
	}
	if (binds == NULL && n_binds != 0U) {
		return -EINVAL;
	}
	if ((unsigned int)t >= (unsigned int)SNMP_TRAP__COUNT) {
		return -EINVAL;
	}
	if (c->cfg.community == NULL || c->cfg.community[0] == '\0') {
		return -EACCES;
	}

	rc = snmp_wr_init(&w, out, cap);
	if (rc != 0) {
		return rc;
	}

	rc = snmp_wr_begin(&w, SNMP_TAG_SEQUENCE, &m_msg);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_int(&w, SNMP_TAG_INTEGER, SNMP_VERSION_2C);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_tlv(&w, SNMP_TAG_OCTET_STRING,
			 (const uint8_t *)c->cfg.community,
			 strlen(c->cfg.community));
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_begin(&w, SNMP_PDU_TRAP_V2, &m_pdu);
	if (rc != 0) {
		return rc;
	}

	c->trap_reqid++;
	rc = snmp_wr_int(&w, SNMP_TAG_INTEGER,
			 (int64_t)(c->trap_reqid & 0x7FFFFFFFU));
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_int(&w, SNMP_TAG_INTEGER, 0);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_int(&w, SNMP_TAG_INTEGER, 0);
	if (rc != 0) {
		return rc;
	}

	rc = snmp_wr_begin(&w, SNMP_TAG_SEQUENCE, &m_vbl);
	if (rc != 0) {
		return rc;
	}

	/* Varbind 1: sysUpTime.0 (RFC 3416 §4.2.6). */
	snmp_val_uint(&v, SNMP_TAG_TIMETICKS, uptime_cs);
	n = snmp_mib_oid_of((uint16_t)SNMP_OBJ_SYS_UPTIME, 0U, arcs);
	if (n < 0) {
		return n;
	}
	rc = snmp__put_varbind(&w, arcs, (size_t)n, &v);
	if (rc != 0) {
		return rc;
	}

	/* Varbind 2: snmpTrapOID.0 = the notification. */
	n = snmp_trap_oid(t, arcs);
	if (n < 0) {
		return n;
	}
	snmp_val_oid(&v, arcs, (size_t)n);
	rc = snmp__put_varbind(&w, o_snmp_trap_oid,
			 sizeof(o_snmp_trap_oid) / sizeof(o_snmp_trap_oid[0]),
			 &v);
	if (rc != 0) {
		return rc;
	}

	for (i = 0U; i < n_binds; i++) {
		n = snmp_mib_oid_of(binds[i].obj, binds[i].inst, arcs);
		if (n < 0) {
			/* An unknown binding is a firmware bug, not a manager
			 * input; refusing the whole trap would hide the event
			 * the trap exists to report. */
			continue;
		}
		snmp__resolve(c, binds[i].obj, binds[i].inst, uptime_cs, &v);
		rc = snmp__put_varbind(&w, arcs, (size_t)n, &v);
		if (rc != 0) {
			return rc;
		}
	}

	rc = snmp_wr_end(&w, m_vbl);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_end(&w, m_pdu);
	if (rc != 0) {
		return rc;
	}
	rc = snmp_wr_end(&w, m_msg);
	if (rc != 0) {
		return rc;
	}

	c->stats.traps++;
	*out_len = snmp_wr_len(&w);
	return 0;
}
