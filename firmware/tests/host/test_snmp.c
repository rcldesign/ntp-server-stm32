/*
 * STS1000 "Meridian" — core/snmp unit tests.
 *
 * Provenance of the expectations (ARCHITECTURE.md §9 asks for known-good
 * vectors, not self-round-trips):
 *
 *   - The four whole-message vectors (`v_get_req` / `v_get_rsp`,
 *     `v_getnext_req` / `v_getnext_rsp`, `v_coldstart_trap`) are hand-derived
 *     from RFC 3416 §3 (message and PDU shape), RFC 1901 (the v2c community
 *     wrapper) and X.690 §8 (identifier, definite length, INTEGER and OBJECT
 *     IDENTIFIER encodings). They are literal octet arrays: nothing in them is
 *     produced by the code under test, so a swapped field or a non-minimal
 *     length shows as a byte difference instead of agreeing with itself.
 *
 *   - Worked example for the OID encoding, because it is the part everyone
 *     gets wrong: 1.3.6.1.2.1.1.3.0 encodes its first two arcs as one
 *     subidentifier, 1*40 + 3 = 43 = 0x2B, then one octet each for 6, 1, 2, 1,
 *     1, 3, 0 — `06 08 2B 06 01 02 01 01 03 00`. The enterprise arc 99999
 *     needs base-128 continuation: 99999 = 0b110_0001101_0011111, so
 *     `86 8D 1F`.
 *
 *   - Requests that are not fixed vectors are assembled by the tiny BER writer
 *     at the top of this file. It is deliberately a second, independent
 *     implementation — different structure, no shared helpers — so a bug in
 *     snmp.c's encoder cannot cancel out against the test input.
 *
 *   - The walk tests assert protocol *properties* the RFC states: GETNEXT
 *     returns a strictly greater OID (RFC 3416 §4.2.2), the walk terminates in
 *     endOfMibView exactly once per varbind, and a GETBULK emits
 *     non-repeaters + N x repeaters in round-major order (§4.2.3).
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "snmp/snmp.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* ------------------------------------------------------------------------ */
/* an independent BER writer, used to build request inputs                    */
/* ------------------------------------------------------------------------ */

typedef struct {
	uint8_t b[2048];
	size_t n;
} bld_t;

static void bld_reset(bld_t *b)
{
	memset(b, 0, sizeof(*b));
}

static void bld_raw(bld_t *b, const uint8_t *p, size_t n)
{
	TEST_ASSERT_TRUE(b->n + n <= sizeof(b->b));
	memcpy(&b->b[b->n], p, n);
	b->n += n;
}

static void bld_u8(bld_t *b, uint8_t v)
{
	bld_raw(b, &v, 1U);
}

/** Emit tag + definite length; returns nothing (content follows). */
static void bld_hdr(bld_t *b, uint8_t tag, size_t len)
{
	bld_u8(b, tag);
	if (len < 0x80U) {
		bld_u8(b, (uint8_t)len);
	} else if (len <= 0xFFU) {
		bld_u8(b, 0x81U);
		bld_u8(b, (uint8_t)len);
	} else {
		bld_u8(b, 0x82U);
		bld_u8(b, (uint8_t)(len >> 8));
		bld_u8(b, (uint8_t)len);
	}
}

static void bld_tlv(bld_t *b, uint8_t tag, const uint8_t *v, size_t n)
{
	bld_hdr(b, tag, n);
	if (n != 0U) {
		bld_raw(b, v, n);
	}
}

/** INTEGER, written big-endian over exactly @p width octets (no minimising:
 *  the agent must accept a legal non-minimal-looking but valid encoding). */
static void bld_int_w(bld_t *b, int32_t v, size_t width)
{
	uint8_t t[4];
	size_t i;

	for (i = 0U; i < width; i++) {
		t[i] = (uint8_t)(((uint32_t)v >> (8U * (width - 1U - i))) &
				 0xFFU);
	}
	bld_tlv(b, 0x02U, t, width);
}

static void bld_int(bld_t *b, int32_t v)
{
	if (v >= -128 && v <= 127) {
		bld_int_w(b, v, 1U);
	} else if (v >= -32768 && v <= 32767) {
		bld_int_w(b, v, 2U);
	} else {
		bld_int_w(b, v, 4U);
	}
}

static void bld_str(bld_t *b, const char *s)
{
	bld_tlv(b, 0x04U, (const uint8_t *)s, strlen(s));
}

static size_t bld_subid(uint8_t *out, uint32_t v)
{
	uint8_t tmp[5];
	size_t n = 0U;
	size_t i;

	do {
		tmp[n++] = (uint8_t)(v & 0x7FU);
		v >>= 7;
	} while (v != 0U);
	for (i = 0U; i < n; i++) {
		out[i] = (uint8_t)(tmp[n - 1U - i] |
				   ((i + 1U == n) ? 0x00U : 0x80U));
	}
	return n;
}

static void bld_oid(bld_t *b, const uint32_t *arcs, size_t n)
{
	uint8_t body[128];
	size_t used = 0U;
	size_t i;

	used += bld_subid(&body[used], (arcs[0] * 40U) + arcs[1]);
	for (i = 2U; i < n; i++) {
		used += bld_subid(&body[used], arcs[i]);
	}
	bld_tlv(b, 0x06U, body, used);
}

/** Wrap the bytes already in @p src under @p tag, into @p dst. */
static void bld_wrap(bld_t *dst, uint8_t tag, const bld_t *src)
{
	bld_hdr(dst, tag, src->n);
	bld_raw(dst, src->b, src->n);
}

/* ------------------------------------------------------------------------ */
/* well-known OIDs                                                            */
/* ------------------------------------------------------------------------ */

#define PEN 99999U

static const uint32_t oid_sys_descr[] = { 1, 3, 6, 1, 2, 1, 1, 1, 0 };
static const uint32_t oid_sys_objid[] = { 1, 3, 6, 1, 2, 1, 1, 2, 0 };
static const uint32_t oid_sys_uptime[] = { 1, 3, 6, 1, 2, 1, 1, 3, 0 };
static const uint32_t oid_internet[] = { 1, 3, 6, 1 };
static const uint32_t oid_ent_root[] = { 1, 3, 6, 1, 4, 1, PEN, 1 };
static const uint32_t oid_stratum[] = { 1, 3, 6, 1, 4, 1, PEN, 1, 1, 1, 0 };
static const uint32_t oid_rail_mv_1[] = { 1, 3, 6, 1, 4, 1, PEN, 1, 3, 1, 1, 3, 1 };
static const uint32_t oid_rail_mv_bad[] = {
	1, 3, 6, 1, 4, 1, PEN, 1, 3, 1, 1, 3, 99
};
static const uint32_t oid_unknown[] = { 1, 3, 6, 1, 4, 1, 12345, 7, 7, 0 };
static const uint32_t oid_last[] = { 1, 3, 6, 1, 4, 1, PEN, 1, 7, 9, 0 };

/* ------------------------------------------------------------------------ */
/* hand-derived message vectors                                               */
/* ------------------------------------------------------------------------ */

/* GetRequest, community "public", request-id 0x0A0B0C0D, one varbind naming
 * sysUpTime.0 with a NULL value. */
static const uint8_t v_get_req[] = {
	0x30, 0x29, 0x02, 0x01, 0x01, 0x04, 0x06, 0x70, 0x75, 0x62, 0x6C, 0x69,
	0x63, 0xA0, 0x1C, 0x02, 0x04, 0x0A, 0x0B, 0x0C, 0x0D, 0x02, 0x01, 0x00,
	0x02, 0x01, 0x00, 0x30, 0x0E, 0x30, 0x0C, 0x06, 0x08, 0x2B, 0x06, 0x01,
	0x02, 0x01, 0x01, 0x03, 0x00, 0x05, 0x00,
};

/* Its Response: same request-id, errorStatus 0, errorIndex 0, sysUpTime.0 =
 * TimeTicks 123456 (0x01E240). */
static const uint8_t v_get_rsp[] = {
	0x30, 0x2C, 0x02, 0x01, 0x01, 0x04, 0x06, 0x70, 0x75, 0x62, 0x6C, 0x69,
	0x63, 0xA2, 0x1F, 0x02, 0x04, 0x0A, 0x0B, 0x0C, 0x0D, 0x02, 0x01, 0x00,
	0x02, 0x01, 0x00, 0x30, 0x11, 0x30, 0x0F, 0x06, 0x08, 0x2B, 0x06, 0x01,
	0x02, 0x01, 0x01, 0x03, 0x00, 0x43, 0x03, 0x01, 0xE2, 0x40,
};

/* GetNextRequest for sysDescr.0, request-id 7. */
static const uint8_t v_getnext_req[] = {
	0x30, 0x26, 0x02, 0x01, 0x01, 0x04, 0x06, 0x70, 0x75, 0x62, 0x6C, 0x69,
	0x63, 0xA1, 0x19, 0x02, 0x01, 0x07, 0x02, 0x01, 0x00, 0x02, 0x01, 0x00,
	0x30, 0x0E, 0x30, 0x0C, 0x06, 0x08, 0x2B, 0x06, 0x01, 0x02, 0x01, 0x01,
	0x01, 0x00, 0x05, 0x00,
};

/* Its Response: sysObjectID.0 = 1.3.6.1.4.1.99999.1. */
static const uint8_t v_getnext_rsp[] = {
	0x30, 0x2F, 0x02, 0x01, 0x01, 0x04, 0x06, 0x70, 0x75, 0x62, 0x6C, 0x69,
	0x63, 0xA2, 0x22, 0x02, 0x01, 0x07, 0x02, 0x01, 0x00, 0x02, 0x01, 0x00,
	0x30, 0x17, 0x30, 0x15, 0x06, 0x08, 0x2B, 0x06, 0x01, 0x02, 0x01, 0x01,
	0x02, 0x00, 0x06, 0x09, 0x2B, 0x06, 0x01, 0x04, 0x01, 0x86, 0x8D, 0x1F,
	0x01,
};

/* SNMPv2-Trap coldStart, uptime 1234 (0x04D2), request-id 1: sysUpTime.0 then
 * snmpTrapOID.0 = 1.3.6.1.6.3.1.1.5.1 (RFC 3418 snmpTraps.coldStart). */
static const uint8_t v_coldstart_trap[] = {
	0x30, 0x41, 0x02, 0x01, 0x01, 0x04, 0x06, 0x70, 0x75, 0x62, 0x6C, 0x69,
	0x63, 0xA7, 0x34, 0x02, 0x01, 0x01, 0x02, 0x01, 0x00, 0x02, 0x01, 0x00,
	0x30, 0x29, 0x30, 0x0E, 0x06, 0x08, 0x2B, 0x06, 0x01, 0x02, 0x01, 0x01,
	0x03, 0x00, 0x43, 0x02, 0x04, 0xD2, 0x30, 0x17, 0x06, 0x0A, 0x2B, 0x06,
	0x01, 0x06, 0x03, 0x01, 0x01, 0x04, 0x01, 0x00, 0x06, 0x09, 0x2B, 0x06,
	0x01, 0x06, 0x03, 0x01, 0x01, 0x05, 0x01,
};

/* ------------------------------------------------------------------------ */
/* fixture                                                                    */
/* ------------------------------------------------------------------------ */

#define UPTIME_CS 123456U

static snmp_ctx_t g_ctx;
static bld_t g_in;
static uint8_t g_out[SNMP_PKT_MAX];
static size_t g_out_len;

/* Object the fake getter refuses, or SNMP_OBJ__COUNT for "none". */
static uint16_t g_fail_obj;
/* Octets the fake getter returns for every OCTET STRING leaf. */
static size_t g_str_len;

static uint8_t type_of_obj(uint16_t obj)
{
	size_t i;

	for (i = 0U; i < snmp_mib_node_count(); i++) {
		const snmp_node_t *n = snmp_mib_node(i);

		if (n->obj == obj) {
			return n->type;
		}
	}
	return 0U;
}

static int fake_get(void *ctx, uint16_t obj, uint16_t inst, snmp_value_t *out)
{
	uint8_t buf[SNMP_OCTET_MAX];
	uint8_t type;
	size_t i;

	(void)ctx;

	if (obj == g_fail_obj) {
		return -ENOENT;
	}

	if (obj == (uint16_t)SNMP_OBJ_SYS_OBJECT_ID) {
		snmp_val_oid(out, oid_ent_root, ARRAY_LEN(oid_ent_root));
		return 0;
	}

	type = type_of_obj(obj);
	switch (type) {
	case SNMP_TAG_INTEGER:
		snmp_val_int(out, (int64_t)obj - (int64_t)inst);
		return 0;
	case SNMP_TAG_COUNTER32:
	case SNMP_TAG_GAUGE32:
	case SNMP_TAG_TIMETICKS:
	case SNMP_TAG_COUNTER64:
		snmp_val_uint(out, type, ((uint64_t)obj * 1000U) + inst);
		return 0;
	case SNMP_TAG_OCTET_STRING:
		for (i = 0U; i < g_str_len && i < sizeof(buf); i++) {
			buf[i] = (uint8_t)('a' + (int)(i % 26U));
		}
		snmp_val_octets(out, buf, (g_str_len < sizeof(buf)) ? g_str_len
								   : sizeof(buf));
		return 0;
	default:
		return -ENOENT;
	}
}

void setUp(void)
{
	snmp_cfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.community = "public";
	cfg.getter.get = fake_get;
	cfg.getter.ctx = NULL;
	cfg.max_repetitions = 0U;

	g_fail_obj = (uint16_t)SNMP_OBJ__COUNT;
	g_str_len = 8U;

	memset(&g_ctx, 0, sizeof(g_ctx));
	TEST_ASSERT_EQUAL_INT(0, snmp_init(&g_ctx, &cfg));

	bld_reset(&g_in);
	memset(g_out, 0, sizeof(g_out));
	g_out_len = 0U;
}

void tearDown(void)
{
}

/** Assemble a request message around a PDU body already built in @p pdu_body. */
static void make_msg(uint8_t pdu_tag, const bld_t *pdu_body, const char *comm)
{
	bld_t pdu;
	bld_t body;

	bld_reset(&pdu);
	bld_wrap(&pdu, pdu_tag, pdu_body);

	bld_reset(&body);
	bld_int(&body, 1); /* version: v2c */
	bld_str(&body, comm);
	bld_raw(&body, pdu.b, pdu.n);

	bld_reset(&g_in);
	bld_wrap(&g_in, 0x30U, &body);
}

/** Build one varbind list from @p oids into @p out. */
static void make_vbl(bld_t *out, const uint32_t *const *oids,
		     const size_t *lens, size_t n)
{
	bld_t inner;
	bld_t one;
	size_t i;

	bld_reset(&inner);
	for (i = 0U; i < n; i++) {
		bld_reset(&one);
		bld_oid(&one, oids[i], lens[i]);
		bld_tlv(&one, 0x05U, NULL, 0U); /* NULL value */
		bld_wrap(&inner, 0x30U, &one);
	}
	bld_reset(out);
	bld_wrap(out, 0x30U, &inner);
}

/** Build a complete request for one PDU type over @p n OIDs. */
static void make_request(uint8_t pdu_tag, int32_t reqid, int32_t f2, int32_t f3,
			 const uint32_t *const *oids, const size_t *lens,
			 size_t n)
{
	bld_t vbl;
	bld_t pdu;

	make_vbl(&vbl, oids, lens, n);

	bld_reset(&pdu);
	bld_int(&pdu, reqid);
	bld_int(&pdu, f2);
	bld_int(&pdu, f3);
	bld_raw(&pdu, vbl.b, vbl.n);

	make_msg(pdu_tag, &pdu, "public");
}

static int run(void)
{
	return snmp_handle(&g_ctx, g_in.b, g_in.n, UPTIME_CS, g_out,
			   sizeof(g_out), &g_out_len);
}

/* ------------------------------------------------------------------------ */
/* response inspection                                                        */
/* ------------------------------------------------------------------------ */

typedef struct {
	int32_t reqid;
	int64_t err;
	int64_t err_index;
	size_t n_vb;
	uint32_t oid[SNMP_MAX_VARBINDS + 8U][SNMP_OID_MAX_LEN];
	size_t oid_n[SNMP_MAX_VARBINDS + 8U];
	uint8_t vtype[SNMP_MAX_VARBINDS + 8U];
	uint64_t vuint[SNMP_MAX_VARBINDS + 8U];
	int64_t vint[SNMP_MAX_VARBINDS + 8U];
} rsp_t;

/** Decode a Response message far enough for the assertions below. */
static void decode_rsp(const uint8_t *buf, size_t len, rsp_t *out)
{
	snmp_rd_t r;
	snmp_rd_t body;
	snmp_rd_t pdu;
	snmp_rd_t vbl;
	uint8_t tag;
	size_t l;
	int64_t v;
	const uint8_t *s;
	size_t sn;

	memset(out, 0, sizeof(*out));

	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, buf, len));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_hdr(&r, &tag, &l));
	TEST_ASSERT_EQUAL_HEX8(SNMP_TAG_SEQUENCE, tag);
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&body, &buf[r.off], l));

	TEST_ASSERT_EQUAL_INT(0, snmp_rd_int(&body, &v));
	TEST_ASSERT_EQUAL_INT64(SNMP_VERSION_2C, v);
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_octets(&body, &s, &sn));

	TEST_ASSERT_EQUAL_INT(0, snmp_rd_hdr(&body, &tag, &l));
	TEST_ASSERT_EQUAL_HEX8(SNMP_PDU_RESPONSE, tag);
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&pdu, &body.buf[body.off], l));

	TEST_ASSERT_EQUAL_INT(0, snmp_rd_int(&pdu, &v));
	out->reqid = (int32_t)v;
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_int(&pdu, &out->err));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_int(&pdu, &out->err_index));

	TEST_ASSERT_EQUAL_INT(0, snmp_rd_hdr(&pdu, &tag, &l));
	TEST_ASSERT_EQUAL_HEX8(SNMP_TAG_SEQUENCE, tag);
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&vbl, &pdu.buf[pdu.off], l));

	while (!snmp_rd_done(&vbl)) {
		snmp_rd_t one;
		size_t n = 0U;

		TEST_ASSERT_TRUE(out->n_vb < ARRAY_LEN(out->oid_n));
		TEST_ASSERT_EQUAL_INT(0, snmp_rd_hdr(&vbl, &tag, &l));
		TEST_ASSERT_EQUAL_HEX8(SNMP_TAG_SEQUENCE, tag);
		TEST_ASSERT_EQUAL_INT(0,
				      snmp_rd_init(&one, &vbl.buf[vbl.off], l));
		TEST_ASSERT_EQUAL_INT(0, snmp_rd_skip(&vbl, l));

		TEST_ASSERT_EQUAL_INT(0, snmp_rd_oid(&one, out->oid[out->n_vb],
						     SNMP_OID_MAX_LEN, &n,
						     NULL));
		out->oid_n[out->n_vb] = n;

		/* Peek the value tag without consuming, then decode by type. */
		TEST_ASSERT_TRUE(one.off < one.len);
		out->vtype[out->n_vb] = one.buf[one.off];
		switch (out->vtype[out->n_vb]) {
		case SNMP_TAG_INTEGER:
			TEST_ASSERT_EQUAL_INT(
				0, snmp_rd_int(&one, &out->vint[out->n_vb]));
			break;
		case SNMP_TAG_COUNTER32:
		case SNMP_TAG_GAUGE32:
		case SNMP_TAG_TIMETICKS:
		case SNMP_TAG_COUNTER64:
			TEST_ASSERT_EQUAL_INT(
				0, snmp_rd_uint(&one, out->vtype[out->n_vb],
						&out->vuint[out->n_vb]));
			break;
		default:
			TEST_ASSERT_EQUAL_INT(0, snmp_rd_hdr(&one, &tag, &l));
			TEST_ASSERT_EQUAL_INT(0, snmp_rd_skip(&one, l));
			break;
		}
		out->n_vb++;
	}
}

static int oid_equal(const uint32_t *a, size_t na, const uint32_t *b, size_t nb)
{
	return (na == nb) && (memcmp(a, b, na * sizeof(a[0])) == 0);
}

static int oid_less(const uint32_t *a, size_t na, const uint32_t *b, size_t nb)
{
	size_t n = (na < nb) ? na : nb;
	size_t i;

	for (i = 0U; i < n; i++) {
		if (a[i] != b[i]) {
			return a[i] < b[i];
		}
	}
	return na < nb;
}

/* ======================================================================== */
/* BER codec                                                                 */
/* ======================================================================== */

static void test_reader_argument_guards(void)
{
	snmp_rd_t r;
	uint8_t tag;
	size_t len;
	int64_t v;
	uint64_t u;
	const uint8_t *p;
	size_t n;
	uint32_t arcs[SNMP_OID_MAX_LEN];
	static const uint8_t some[] = { 0x02, 0x01, 0x00 };

	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_rd_init(NULL, some, sizeof(some)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_rd_init(&r, NULL, 4U));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, NULL, 0U));
	TEST_ASSERT_TRUE(snmp_rd_done(&r));
	TEST_ASSERT_TRUE(snmp_rd_done(NULL));

	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, some, sizeof(some)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_rd_hdr(NULL, &tag, &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_rd_hdr(&r, NULL, &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_rd_hdr(&r, &tag, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_rd_skip(NULL, 1U));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, snmp_rd_skip(&r, 99U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_rd_int(NULL, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_rd_int(&r, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_rd_uint(NULL, 0x41U, &u));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_rd_uint(&r, 0x41U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_rd_octets(NULL, &p, &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_rd_octets(&r, NULL, &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_rd_octets(&r, &p, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_rd_oid(NULL, arcs, sizeof(arcs), &n, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_rd_oid(&r, NULL, 4U, &n, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_rd_oid(&r, arcs, 4U, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_rd_oid(&r, arcs, 1U, &n, NULL));
}

static void test_header_rejects_illegal_encodings(void)
{
	struct {
		const char *why;
		uint8_t bytes[8];
		size_t len;
	} cases[] = {
		{ "truncated to one octet", { 0x30 }, 1U },
		{ "multi-octet identifier", { 0x1F, 0x81, 0x00 }, 3U },
		{ "indefinite length", { 0x30, 0x80, 0x00, 0x00 }, 4U },
		{ "length over four octets", { 0x04, 0x85, 1, 2, 3, 4, 5 }, 7U },
		{ "length octets truncated", { 0x04, 0x83, 0x00 }, 3U },
		{ "content runs past the end", { 0x04, 0x08, 0x00 }, 3U },
	};
	size_t i;

	for (i = 0U; i < ARRAY_LEN(cases); i++) {
		snmp_rd_t r;
		uint8_t tag;
		size_t len;

		TEST_ASSERT_EQUAL_INT(
			0, snmp_rd_init(&r, cases[i].bytes, cases[i].len));
		TEST_ASSERT_EQUAL_INT_MESSAGE(-EBADMSG,
					      snmp_rd_hdr(&r, &tag, &len),
					      cases[i].why);
	}
}

static void test_header_accepts_long_form_lengths(void)
{
	uint8_t buf[600];
	snmp_rd_t r;
	uint8_t tag;
	size_t len;

	memset(buf, 0, sizeof(buf));
	buf[0] = 0x04U;
	buf[1] = 0x81U;
	buf[2] = 0x80U; /* 128 octets */
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, buf, 3U + 128U));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_hdr(&r, &tag, &len));
	TEST_ASSERT_EQUAL_size_t(128U, len);

	buf[0] = 0x04U;
	buf[1] = 0x82U;
	buf[2] = 0x02U;
	buf[3] = 0x00U; /* 512 octets */
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, buf, 4U + 512U));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_hdr(&r, &tag, &len));
	TEST_ASSERT_EQUAL_size_t(512U, len);
}

static void test_integer_round_trip(void)
{
	static const int64_t vals[] = {
		0,     1,	-1,	127,	   128,	     -128,
		-129,  255,	256,	32767,	   -32768,   32768,
		-32769, 8388607, -8388608, 2147483647L, -2147483648LL,
		INT64_C(140737488355327), INT64_C(-140737488355328),
		INT64_MAX, INT64_MIN,
	};
	size_t i;

	for (i = 0U; i < ARRAY_LEN(vals); i++) {
		uint8_t buf[32];
		snmp_wr_t w;
		snmp_rd_t r;
		int64_t back = 0;

		TEST_ASSERT_EQUAL_INT(0, snmp_wr_init(&w, buf, sizeof(buf)));
		TEST_ASSERT_EQUAL_INT(
			0, snmp_wr_int(&w, SNMP_TAG_INTEGER, vals[i]));
		TEST_ASSERT_EQUAL_INT(0,
				      snmp_rd_init(&r, buf, snmp_wr_len(&w)));
		TEST_ASSERT_EQUAL_INT(0, snmp_rd_int(&r, &back));
		TEST_ASSERT_EQUAL_INT64(vals[i], back);
		/* Minimal encoding: never a redundant leading octet. */
		TEST_ASSERT_TRUE(buf[1] >= 1U && buf[1] <= 8U);
		if (buf[1] > 1U) {
			bool redundant = (buf[2] == 0x00U &&
					  (buf[3] & 0x80U) == 0U) ||
					 (buf[2] == 0xFFU &&
					  (buf[3] & 0x80U) != 0U);
			TEST_ASSERT_FALSE(redundant);
		}
	}
}

static void test_integer_known_answers(void)
{
	static const struct {
		int64_t v;
		uint8_t enc[6];
		size_t n;
	} cases[] = {
		{ 0, { 0x02, 0x01, 0x00 }, 3U },
		{ 127, { 0x02, 0x01, 0x7F }, 3U },
		{ 128, { 0x02, 0x02, 0x00, 0x80 }, 4U },
		{ -1, { 0x02, 0x01, 0xFF }, 3U },
		{ -128, { 0x02, 0x01, 0x80 }, 3U },
		{ -129, { 0x02, 0x02, 0xFF, 0x7F }, 4U },
		{ 168496141, { 0x02, 0x04, 0x0A, 0x0B, 0x0C, 0x0D }, 6U },
	};
	size_t i;

	for (i = 0U; i < ARRAY_LEN(cases); i++) {
		uint8_t buf[16];
		snmp_wr_t w;

		TEST_ASSERT_EQUAL_INT(0, snmp_wr_init(&w, buf, sizeof(buf)));
		TEST_ASSERT_EQUAL_INT(
			0, snmp_wr_int(&w, SNMP_TAG_INTEGER, cases[i].v));
		TEST_ASSERT_EQUAL_size_t(cases[i].n, snmp_wr_len(&w));
		TEST_ASSERT_EQUAL_HEX8_ARRAY(cases[i].enc, buf, cases[i].n);
	}
}

static void test_unsigned_round_trip_and_padding(void)
{
	static const uint64_t vals[] = {
		0U, 1U, 127U, 128U, 255U, 256U, 65535U, 0x7FFFFFFFU,
		0x80000000U, 0xFFFFFFFFU, UINT64_C(0x0102030405060708),
		UINT64_MAX,
	};
	static const uint8_t tags[] = { SNMP_TAG_COUNTER32, SNMP_TAG_GAUGE32,
					SNMP_TAG_TIMETICKS,
					SNMP_TAG_COUNTER64 };
	size_t i;
	size_t t;

	for (t = 0U; t < ARRAY_LEN(tags); t++) {
		for (i = 0U; i < ARRAY_LEN(vals); i++) {
			uint8_t buf[32];
			snmp_wr_t w;
			snmp_rd_t r;
			uint64_t back = 0U;

			TEST_ASSERT_EQUAL_INT(
				0, snmp_wr_init(&w, buf, sizeof(buf)));
			TEST_ASSERT_EQUAL_INT(
				0, snmp_wr_uint(&w, tags[t], vals[i]));
			TEST_ASSERT_EQUAL_INT(
				0, snmp_rd_init(&r, buf, snmp_wr_len(&w)));
			TEST_ASSERT_EQUAL_INT(
				0, snmp_rd_uint(&r, tags[t], &back));
			TEST_ASSERT_EQUAL_UINT64(vals[i], back);
			/* Never encoded so a reader could take it as negative. */
			TEST_ASSERT_TRUE((buf[2] & 0x80U) == 0U);
		}
	}

	/* 0xFFFFFFFF needs the pad: 42 05 00 FF FF FF FF. */
	{
		uint8_t buf[16];
		static const uint8_t want[] = { 0x42, 0x05, 0x00, 0xFF,
						0xFF, 0xFF, 0xFF };
		snmp_wr_t w;

		TEST_ASSERT_EQUAL_INT(0, snmp_wr_init(&w, buf, sizeof(buf)));
		TEST_ASSERT_EQUAL_INT(
			0, snmp_wr_uint(&w, SNMP_TAG_GAUGE32, 0xFFFFFFFFU));
		TEST_ASSERT_EQUAL_size_t(sizeof(want), snmp_wr_len(&w));
		TEST_ASSERT_EQUAL_HEX8_ARRAY(want, buf, sizeof(want));
	}
}

static void test_unsigned_rejects_bad_encodings(void)
{
	snmp_rd_t r;
	uint64_t u;
	/* Ten content octets cannot be a 64-bit unsigned. */
	static const uint8_t too_wide[] = { 0x46, 0x0A, 0, 0, 0, 0,
					    0,	  0,	0, 0, 0, 0 };
	/* Nine octets whose pad is not zero. */
	static const uint8_t bad_pad[] = { 0x46, 0x09, 0x01, 0, 0, 0,
					   0,	 0,    0,    0, 0 };
	static const uint8_t empty[] = { 0x46, 0x00 };
	static const uint8_t wrong_tag[] = { 0x02, 0x01, 0x05 };
	static const uint8_t good9[] = { 0x46, 0x09, 0x00, 0xFF, 0xFF, 0xFF,
					 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, too_wide, sizeof(too_wide)));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      snmp_rd_uint(&r, SNMP_TAG_COUNTER64, &u));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, bad_pad, sizeof(bad_pad)));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      snmp_rd_uint(&r, SNMP_TAG_COUNTER64, &u));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, empty, sizeof(empty)));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      snmp_rd_uint(&r, SNMP_TAG_COUNTER64, &u));
	TEST_ASSERT_EQUAL_INT(0,
			      snmp_rd_init(&r, wrong_tag, sizeof(wrong_tag)));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      snmp_rd_uint(&r, SNMP_TAG_COUNTER64, &u));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, good9, sizeof(good9)));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_uint(&r, SNMP_TAG_COUNTER64, &u));
	TEST_ASSERT_EQUAL_UINT64(UINT64_MAX, u);
}

static void test_integer_rejects_bad_encodings(void)
{
	snmp_rd_t r;
	int64_t v;
	static const uint8_t empty[] = { 0x02, 0x00 };
	static const uint8_t too_wide[] = { 0x02, 0x09, 0, 0, 0, 0,
					    0,	  0,	0, 0, 0 };
	static const uint8_t wrong_tag[] = { 0x04, 0x01, 0x05 };

	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, empty, sizeof(empty)));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, snmp_rd_int(&r, &v));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, too_wide, sizeof(too_wide)));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, snmp_rd_int(&r, &v));
	TEST_ASSERT_EQUAL_INT(0,
			      snmp_rd_init(&r, wrong_tag, sizeof(wrong_tag)));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, snmp_rd_int(&r, &v));
}

static void test_octets_round_trip(void)
{
	uint8_t buf[300];
	snmp_wr_t w;
	snmp_rd_t r;
	const uint8_t *p;
	size_t n;
	uint8_t payload[200];
	size_t i;

	for (i = 0U; i < sizeof(payload); i++) {
		payload[i] = (uint8_t)i;
	}

	TEST_ASSERT_EQUAL_INT(0, snmp_wr_init(&w, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_tlv(&w, SNMP_TAG_OCTET_STRING, payload,
					     sizeof(payload)));
	/* 200 octets forces the 0x81 long form. */
	TEST_ASSERT_EQUAL_HEX8(0x81U, buf[1]);
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, buf, snmp_wr_len(&w)));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_octets(&r, &p, &n));
	TEST_ASSERT_EQUAL_size_t(sizeof(payload), n);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, p, n);

	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, buf, snmp_wr_len(&w)));
	{
		uint8_t tag;
		size_t l;

		TEST_ASSERT_EQUAL_INT(0, snmp_rd_hdr(&r, &tag, &l));
		TEST_ASSERT_EQUAL_HEX8(SNMP_TAG_OCTET_STRING, tag);
	}
}

static void test_oid_round_trip_and_known_answers(void)
{
	static const uint32_t big[] = { 2, 999, 4294967295U, 1 };
	uint8_t buf[128];
	uint8_t tlvbuf[64];
	snmp_wr_t w;
	snmp_rd_t r;
	uint32_t back[SNMP_OID_MAX_LEN];
	size_t n = 0U;
	bool over = true;
	/* 1.3.6.1.2.1.1.3.0 -> 06 08 2B 06 01 02 01 01 03 00 (X.690 §8.19). */
	static const uint8_t want_uptime[] = { 0x06, 0x08, 0x2B, 0x06, 0x01,
					       0x02, 0x01, 0x01, 0x03, 0x00 };
	/* 1.3.6.1.4.1.99999.1: 99999 -> 86 8D 1F. */
	static const uint8_t want_ent[] = { 0x06, 0x09, 0x2B, 0x06, 0x01,
					    0x04, 0x01, 0x86, 0x8D, 0x1F,
					    0x01 };

	TEST_ASSERT_EQUAL_INT(0, snmp_wr_init(&w, tlvbuf, sizeof(tlvbuf)));
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_oid(&w, oid_sys_uptime,
					     ARRAY_LEN(oid_sys_uptime)));
	TEST_ASSERT_EQUAL_size_t(sizeof(want_uptime), snmp_wr_len(&w));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want_uptime, tlvbuf, sizeof(want_uptime));

	TEST_ASSERT_EQUAL_INT(0, snmp_wr_init(&w, tlvbuf, sizeof(tlvbuf)));
	TEST_ASSERT_EQUAL_INT(
		0, snmp_wr_oid(&w, oid_ent_root, ARRAY_LEN(oid_ent_root)));
	TEST_ASSERT_EQUAL_size_t(sizeof(want_ent), snmp_wr_len(&w));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(want_ent, tlvbuf, sizeof(want_ent));

	/* Arcs that need the widest subidentifier the format allows. */
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_init(&w, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_oid(&w, big, ARRAY_LEN(big)));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, buf, snmp_wr_len(&w)));
	TEST_ASSERT_EQUAL_INT(
		0, snmp_rd_oid(&r, back, SNMP_OID_MAX_LEN, &n, &over));
	TEST_ASSERT_FALSE(over);
	TEST_ASSERT_EQUAL_size_t(ARRAY_LEN(big), n);
	TEST_ASSERT_EQUAL_UINT32_ARRAY(big, back, n);
}

static void test_oid_writer_rejects_illegal_arcs(void)
{
	uint8_t buf[64];
	snmp_wr_t w;
	static const uint32_t one_arc[] = { 1 };
	static const uint32_t bad_root[] = { 3, 1, 2 };
	static const uint32_t bad_second[] = { 1, 40, 2 };
	uint32_t too_many[SNMP_OID_MAX_LEN + 1];
	size_t i;

	for (i = 0U; i < ARRAY_LEN(too_many); i++) {
		too_many[i] = (uint32_t)(i % 3U);
	}

	TEST_ASSERT_EQUAL_INT(0, snmp_wr_init(&w, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_oid(NULL, one_arc, 2U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_oid(&w, NULL, 2U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_oid(&w, one_arc, 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_wr_oid(&w, too_many, ARRAY_LEN(too_many)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_wr_oid(&w, bad_root, ARRAY_LEN(bad_root)));
	TEST_ASSERT_EQUAL_INT(
		-EINVAL, snmp_wr_oid(&w, bad_second, ARRAY_LEN(bad_second)));
}

static void test_oid_reader_rejects_illegal_encodings(void)
{
	struct {
		const char *why;
		uint8_t bytes[12];
		size_t len;
	} cases[] = {
		{ "empty content", { 0x06, 0x00 }, 2U },
		{ "wrong tag", { 0x04, 0x02, 0x2B, 0x06 }, 4U },
		{ "non-minimal subidentifier",
		  { 0x06, 0x03, 0x2B, 0x80, 0x06 }, 5U },
		{ "unterminated subidentifier", { 0x06, 0x02, 0x2B, 0x86 }, 4U },
		{ "subidentifier wider than 32 bits",
		  { 0x06, 0x07, 0x2B, 0x9F, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F },
		  9U },
	};
	size_t i;

	for (i = 0U; i < ARRAY_LEN(cases); i++) {
		snmp_rd_t r;
		uint32_t arcs[SNMP_OID_MAX_LEN];
		size_t n = 0U;

		TEST_ASSERT_EQUAL_INT(
			0, snmp_rd_init(&r, cases[i].bytes, cases[i].len));
		TEST_ASSERT_EQUAL_INT_MESSAGE(
			-EBADMSG,
			snmp_rd_oid(&r, arcs, SNMP_OID_MAX_LEN, &n, NULL),
			cases[i].why);
	}
}

static void test_oid_reader_reports_truncation(void)
{
	uint8_t buf[64];
	snmp_wr_t w;
	snmp_rd_t r;
	uint32_t arcs[SNMP_OID_MAX_LEN];
	uint32_t small[4];
	size_t n = 0U;
	bool over = false;

	TEST_ASSERT_EQUAL_INT(0, snmp_wr_init(&w, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_oid(&w, oid_sys_uptime,
					     ARRAY_LEN(oid_sys_uptime)));

	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, buf, snmp_wr_len(&w)));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_oid(&r, small, ARRAY_LEN(small), &n,
					     &over));
	TEST_ASSERT_TRUE(over);
	TEST_ASSERT_EQUAL_size_t(ARRAY_LEN(small), n);
	TEST_ASSERT_TRUE(snmp_rd_done(&r));

	/* The same OID with room reports no truncation. */
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, buf, snmp_wr_len(&w)));
	over = true;
	TEST_ASSERT_EQUAL_INT(
		0, snmp_rd_oid(&r, arcs, SNMP_OID_MAX_LEN, &n, &over));
	TEST_ASSERT_FALSE(over);
}

static void test_writer_argument_guards_and_capacity(void)
{
	uint8_t buf[8];
	uint8_t payload[4] = { 1, 2, 3, 4 };
	snmp_wr_t w;
	size_t mark;

	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_init(NULL, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_init(&w, NULL, 4U));
	TEST_ASSERT_EQUAL_size_t(0U, snmp_wr_len(NULL));

	TEST_ASSERT_EQUAL_INT(0, snmp_wr_init(&w, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_tlv(NULL, 0x04U, payload, 4U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_tlv(&w, 0x04U, NULL, 4U));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      snmp_wr_tlv(&w, 0x04U, payload, 0x10000U));
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_tlv(&w, 0x04U, payload, 4U));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, snmp_wr_tlv(&w, 0x04U, payload, 4U));

	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_begin(NULL, 0x30U, &mark));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_begin(&w, 0x30U, NULL));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, snmp_wr_begin(&w, 0x30U, &mark));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_end(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_end(&w, w.len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_rewind(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_rewind(&w, w.len + 1U));
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_rewind(&w, 0U));
	TEST_ASSERT_EQUAL_size_t(0U, snmp_wr_len(&w));
}

static void test_constructed_length_shrinks(void)
{
	uint8_t buf[1024];
	uint8_t filler[300];
	snmp_wr_t w;
	size_t mark;
	size_t i;

	memset(filler, 0xAA, sizeof(filler));

	/* Short form: the three reserved octets collapse to one and the content
	 * is moved down, so the first content octet lands at index 2. */
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_init(&w, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_begin(&w, SNMP_TAG_SEQUENCE, &mark));
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_tlv(&w, 0x04U, filler, 4U));
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_end(&w, mark));
	TEST_ASSERT_EQUAL_size_t(8U, snmp_wr_len(&w));
	TEST_ASSERT_EQUAL_HEX8(0x30U, buf[0]);
	TEST_ASSERT_EQUAL_HEX8(0x06U, buf[1]);
	TEST_ASSERT_EQUAL_HEX8(0x04U, buf[2]);

	/* 0x81 form. */
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_init(&w, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_begin(&w, SNMP_TAG_SEQUENCE, &mark));
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_tlv(&w, 0x04U, filler, 200U));
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_end(&w, mark));
	TEST_ASSERT_EQUAL_HEX8(0x81U, buf[1]);
	TEST_ASSERT_EQUAL_HEX8(203U, buf[2]);
	TEST_ASSERT_EQUAL_size_t(206U, snmp_wr_len(&w));

	/* 0x82 form: no move at all, the reservation is exact. */
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_init(&w, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_begin(&w, SNMP_TAG_SEQUENCE, &mark));
	for (i = 0U; i < 3U; i++) {
		TEST_ASSERT_EQUAL_INT(0,
				      snmp_wr_tlv(&w, 0x04U, filler, 100U));
	}
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_end(&w, mark));
	TEST_ASSERT_EQUAL_HEX8(0x82U, buf[1]);
	/* 3 x (1 tag + 1 short length + 100 octets) = 306 content octets, which
	 * needs the two-octet length the reservation already holds. */
	TEST_ASSERT_EQUAL_size_t(4U + 306U, snmp_wr_len(&w));
}

static void test_value_encoding_covers_every_type(void)
{
	uint8_t buf[128];
	snmp_wr_t w;
	snmp_value_t v;

	/* NULL guards on the setters must not fault. */
	snmp_val_int(NULL, 1);
	snmp_val_uint(NULL, SNMP_TAG_GAUGE32, 1U);
	snmp_val_ip(NULL, 0U);
	snmp_val_octets(NULL, NULL, 0U);
	snmp_val_oid(NULL, NULL, 0U);

	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_value(NULL, &v));
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_init(&w, buf, sizeof(buf)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_value(&w, NULL));

	snmp_val_int(&v, -5);
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_value(&w, &v));

	snmp_val_uint(&v, SNMP_TAG_COUNTER64, UINT64_MAX);
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_value(&w, &v));

	/* IpAddress 192.0.2.1 -> 40 04 C0 00 02 01. */
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_init(&w, buf, sizeof(buf)));
	snmp_val_ip(&v, 0xC0000201U);
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_value(&w, &v));
	{
		static const uint8_t want[] = { 0x40, 0x04, 0xC0,
						0x00, 0x02, 0x01 };

		TEST_ASSERT_EQUAL_size_t(sizeof(want), snmp_wr_len(&w));
		TEST_ASSERT_EQUAL_HEX8_ARRAY(want, buf, sizeof(want));
	}

	/* Truncation at the value limits. */
	{
		uint8_t big[SNMP_OCTET_MAX + 20];
		uint32_t many[SNMP_OID_MAX_LEN + 5];
		size_t i;

		memset(big, 'x', sizeof(big));
		for (i = 0U; i < ARRAY_LEN(many); i++) {
			many[i] = (uint32_t)(i % 3U);
		}
		snmp_val_octets(&v, big, sizeof(big));
		TEST_ASSERT_EQUAL_UINT16(SNMP_OCTET_MAX, v.len);
		snmp_val_oid(&v, many, ARRAY_LEN(many));
		TEST_ASSERT_EQUAL_UINT16(SNMP_OID_MAX_LEN, v.len);
		snmp_val_str(&v, NULL);
		TEST_ASSERT_EQUAL_UINT16(0U, v.len);
		snmp_val_octets(&v, NULL, 0U);
		TEST_ASSERT_EQUAL_UINT16(0U, v.len);
	}

	/* Exceptions, OPAQUE and an unknown tag. */
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_init(&w, buf, sizeof(buf)));
	memset(&v, 0, sizeof(v));
	v.type = SNMP_TAG_NULL;
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_value(&w, &v));
	v.type = SNMP_TAG_NO_SUCH_OBJECT;
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_value(&w, &v));
	v.type = SNMP_TAG_NO_SUCH_INSTANCE;
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_value(&w, &v));
	v.type = SNMP_TAG_END_OF_MIB_VIEW;
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_value(&w, &v));
	v.type = SNMP_TAG_OPAQUE;
	v.len = 0U;
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_value(&w, &v));
	v.type = 0x7FU; /* not a type this agent knows */
	TEST_ASSERT_EQUAL_INT(0, snmp_wr_value(&w, &v));
	TEST_ASSERT_EQUAL_HEX8(SNMP_TAG_NO_SUCH_OBJECT,
			       buf[snmp_wr_len(&w) - 2U]);

	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_ip(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_int(NULL, 0x02U, 0));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_uint(NULL, 0x41U, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_wr_empty(NULL, 0x05U));
}

/* ======================================================================== */
/* the catalogue                                                             */
/* ======================================================================== */

static void test_mib_is_sorted_and_consistent(void)
{
	uint32_t prev[SNMP_OID_MAX_LEN];
	uint32_t cur[SNMP_OID_MAX_LEN];
	size_t prev_n = 0U;
	size_t total;
	size_t i;
	size_t sum = 0U;

	TEST_ASSERT_EQUAL_INT(0, snmp_mib_check());
	TEST_ASSERT_NOT_NULL(snmp_mib_node(0U));
	TEST_ASSERT_NULL(snmp_mib_node(snmp_mib_node_count()));

	for (i = 0U; i < snmp_mib_node_count(); i++) {
		sum += snmp_mib_node(i)->n_inst;
	}
	total = snmp_mib_instance_count();
	TEST_ASSERT_EQUAL_size_t(sum, total);
	TEST_ASSERT_TRUE(total > snmp_mib_node_count());

	for (i = 0U; i < total; i++) {
		uint16_t obj = 0U;
		uint16_t inst = 0U;
		uint8_t type = 0U;
		int n = snmp_mib_instance(i, &obj, &inst, &type, cur);

		TEST_ASSERT_TRUE(n >= 3);
		TEST_ASSERT_TRUE((size_t)n <= SNMP_OID_MAX_LEN);
		TEST_ASSERT_TRUE(obj < (uint16_t)SNMP_OBJ__COUNT);
		TEST_ASSERT_TRUE(type != 0U);
		if (i > 0U) {
			TEST_ASSERT_TRUE_MESSAGE(
				oid_less(prev, prev_n, cur, (size_t)n),
				"catalogue is not in lexicographic order");
		}
		/* Round trip through the reverse lookup. */
		{
			uint32_t again[SNMP_OID_MAX_LEN];
			int m = snmp_mib_oid_of(obj, inst, again);

			TEST_ASSERT_EQUAL_INT(n, m);
			TEST_ASSERT_EQUAL_UINT32_ARRAY(cur, again, (size_t)n);
		}
		TEST_ASSERT_EQUAL_INT((int)i, snmp_mib_find(cur, (size_t)n));
		memcpy(prev, cur, (size_t)n * sizeof(cur[0]));
		prev_n = (size_t)n;
	}

	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      snmp_mib_instance(total, NULL, NULL, NULL, cur));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_mib_instance(0U, NULL, NULL, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_mib_find(NULL, 3U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_mib_find_next(NULL, 3U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_mib_oid_of(0U, 0U, NULL));
}

static void test_mib_lookup_edges(void)
{
	uint32_t arcs[SNMP_OID_MAX_LEN];

	/* An OID before everything finds the first instance. */
	TEST_ASSERT_EQUAL_INT(
		0, snmp_mib_find_next(oid_internet, ARRAY_LEN(oid_internet)));
	/* The very last instance has no successor. */
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      snmp_mib_find_next(oid_last, ARRAY_LEN(oid_last)));
	/* Unknown names resolve to nothing. */
	TEST_ASSERT_EQUAL_INT(
		-ENOENT, snmp_mib_find(oid_unknown, ARRAY_LEN(oid_unknown)));
	TEST_ASSERT_EQUAL_INT(
		-ENOENT,
		snmp_mib_find(oid_rail_mv_bad, ARRAY_LEN(oid_rail_mv_bad)));

	/* Instance-arc rules: a scalar takes 0 and nothing else; a rail column
	 * takes 1..9 and nothing else. */
	TEST_ASSERT_TRUE(snmp_mib_oid_of((uint16_t)SNMP_OBJ_STRATUM, 0U, arcs) >
			 0);
	TEST_ASSERT_EQUAL_INT(
		-ENOENT, snmp_mib_oid_of((uint16_t)SNMP_OBJ_STRATUM, 1U, arcs));
	TEST_ASSERT_EQUAL_INT(
		-ENOENT, snmp_mib_oid_of((uint16_t)SNMP_OBJ_RAIL_MV, 0U, arcs));
	TEST_ASSERT_TRUE(
		snmp_mib_oid_of((uint16_t)SNMP_OBJ_RAIL_MV, SNMP_RAIL_COUNT,
				arcs) > 0);
	TEST_ASSERT_EQUAL_INT(-ENOENT,
			      snmp_mib_oid_of((uint16_t)SNMP_OBJ_RAIL_MV,
					      SNMP_RAIL_COUNT + 1U, arcs));
	TEST_ASSERT_EQUAL_INT(
		-ENOENT,
		snmp_mib_oid_of((uint16_t)SNMP_OBJ__COUNT, 0U, arcs));

	/* The rail table really carries nine rows per column. */
	{
		size_t i;

		for (i = 1U; i <= SNMP_RAIL_COUNT; i++) {
			TEST_ASSERT_TRUE(
				snmp_mib_oid_of((uint16_t)SNMP_OBJ_RAIL_NAME,
						(uint16_t)i, arcs) > 0);
		}
	}

	/* And the first rail-voltage row is exactly where the MIB says. */
	TEST_ASSERT_TRUE(
		snmp_mib_find(oid_rail_mv_1, ARRAY_LEN(oid_rail_mv_1)) >= 0);
}

/* ======================================================================== */
/* lifecycle                                                                 */
/* ======================================================================== */

static void test_init_and_config(void)
{
	snmp_ctx_t c;
	snmp_cfg_t cfg;
	snmp_stats_t st;

	memset(&cfg, 0, sizeof(cfg));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_init(NULL, &cfg));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_init(&c, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_init(&c, &cfg)); /* no getter */

	cfg.getter.get = fake_get;
	cfg.community = "public";
	cfg.max_repetitions = 10000U; /* clamped */
	TEST_ASSERT_EQUAL_INT(0, snmp_init(&c, &cfg));

	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_stats_get(NULL, &st));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_stats_get(&c, NULL));
	TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&c, &st));
	TEST_ASSERT_EQUAL_UINT64(0U, st.rx);
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_stats_reset(NULL));
	TEST_ASSERT_EQUAL_INT(0, snmp_stats_reset(&c));

	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_set_community(NULL, "x"));
	TEST_ASSERT_EQUAL_INT(0, snmp_set_community(&c, "secret"));
}

static void test_handle_argument_guards(void)
{
	uint8_t buf[64];
	size_t n = 0U;
	snmp_ctx_t unready;

	memset(&unready, 0, sizeof(unready));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_handle(NULL, v_get_req,
						   sizeof(v_get_req), 0U, buf,
						   sizeof(buf), &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_handle(&unready, v_get_req,
					  sizeof(v_get_req), 0U, buf,
					  sizeof(buf), &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_handle(&g_ctx, NULL, 4U, 0U, buf,
						   sizeof(buf), &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_handle(&g_ctx, v_get_req, sizeof(v_get_req),
					  0U, NULL, sizeof(buf), &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_handle(&g_ctx, v_get_req, sizeof(v_get_req),
					  0U, buf, sizeof(buf), NULL));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      snmp_handle(&g_ctx, v_get_req, 0U, 0U, buf,
					  sizeof(buf), &n));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      snmp_handle(&g_ctx, v_get_req, SNMP_PKT_MAX + 1U,
					  0U, buf, sizeof(buf), &n));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      snmp_handle(&g_ctx, v_get_req, sizeof(v_get_req),
					  0U, buf, 8U, &n));
}

/* ======================================================================== */
/* GET                                                                       */
/* ======================================================================== */

static void test_get_known_answer(void)
{
	snmp_stats_t st;

	g_out_len = 0U;
	TEST_ASSERT_EQUAL_INT(0, snmp_handle(&g_ctx, v_get_req,
					     sizeof(v_get_req), UPTIME_CS,
					     g_out, sizeof(g_out), &g_out_len));
	TEST_ASSERT_EQUAL_size_t(sizeof(v_get_rsp), g_out_len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(v_get_rsp, g_out, g_out_len);

	TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.rx);
	TEST_ASSERT_EQUAL_UINT64(1U, st.get);
	TEST_ASSERT_EQUAL_UINT64(1U, st.responses);
	TEST_ASSERT_EQUAL_UINT64(1U, st.varbinds);
}

static void test_getnext_known_answer(void)
{
	rsp_t rsp;

	g_out_len = 0U;
	TEST_ASSERT_EQUAL_INT(0, snmp_handle(&g_ctx, v_getnext_req,
					     sizeof(v_getnext_req), UPTIME_CS,
					     g_out, sizeof(g_out), &g_out_len));
	TEST_ASSERT_EQUAL_size_t(sizeof(v_getnext_rsp), g_out_len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(v_getnext_rsp, g_out, g_out_len);

	/* The successor of sysDescr.0 is sysObjectID.0, which is what makes
	 * the vector above the right answer and not merely a stable one. */
	decode_rsp(g_out, g_out_len, &rsp);
	TEST_ASSERT_EQUAL_size_t(1U, rsp.n_vb);
	TEST_ASSERT_TRUE(oid_equal(rsp.oid[0], rsp.oid_n[0], oid_sys_objid,
				   ARRAY_LEN(oid_sys_objid)));
	TEST_ASSERT_EQUAL_HEX8(SNMP_TAG_OID, rsp.vtype[0]);
}

static void test_get_exception_varbinds(void)
{
	const uint32_t *oids[4];
	size_t lens[4];
	uint32_t deep[SNMP_OID_MAX_LEN + 2];
	rsp_t rsp;
	size_t i;

	for (i = 0U; i < ARRAY_LEN(deep); i++) {
		deep[i] = (i < 2U) ? (uint32_t)(i + 1U) : 1U;
	}

	oids[0] = oid_unknown; /* nothing in the view is called that */
	lens[0] = ARRAY_LEN(oid_unknown);
	oids[1] = oid_rail_mv_bad; /* the column exists, row 99 does not */
	lens[1] = ARRAY_LEN(oid_rail_mv_bad);
	oids[2] = oid_stratum; /* exists, but the getter will refuse it */
	lens[2] = ARRAY_LEN(oid_stratum);
	oids[3] = deep; /* longer than the agent stores */
	lens[3] = ARRAY_LEN(deep);

	g_fail_obj = (uint16_t)SNMP_OBJ_STRATUM;
	make_request(SNMP_PDU_GET, 42, 0, 0, oids, lens, 4U);
	TEST_ASSERT_EQUAL_INT(0, run());

	decode_rsp(g_out, g_out_len, &rsp);
	TEST_ASSERT_EQUAL_INT32(42, rsp.reqid);
	TEST_ASSERT_EQUAL_INT64(SNMP_ERR_NO_ERROR, rsp.err);
	TEST_ASSERT_EQUAL_size_t(4U, rsp.n_vb);
	TEST_ASSERT_EQUAL_HEX8(SNMP_TAG_NO_SUCH_OBJECT, rsp.vtype[0]);
	TEST_ASSERT_EQUAL_HEX8(SNMP_TAG_NO_SUCH_INSTANCE, rsp.vtype[1]);
	TEST_ASSERT_EQUAL_HEX8(SNMP_TAG_NO_SUCH_INSTANCE, rsp.vtype[2]);
	TEST_ASSERT_EQUAL_HEX8(SNMP_TAG_NO_SUCH_OBJECT, rsp.vtype[3]);
	/* Every varbind echoes the name it was asked about. */
	TEST_ASSERT_TRUE(oid_equal(rsp.oid[0], rsp.oid_n[0], oid_unknown,
				   ARRAY_LEN(oid_unknown)));
}

static void test_get_resolves_a_rail_row(void)
{
	const uint32_t *oids[1] = { oid_rail_mv_1 };
	size_t lens[1] = { ARRAY_LEN(oid_rail_mv_1) };
	rsp_t rsp;

	make_request(SNMP_PDU_GET, 3, 0, 0, oids, lens, 1U);
	TEST_ASSERT_EQUAL_INT(0, run());
	decode_rsp(g_out, g_out_len, &rsp);
	TEST_ASSERT_EQUAL_size_t(1U, rsp.n_vb);
	TEST_ASSERT_EQUAL_HEX8(SNMP_TAG_GAUGE32, rsp.vtype[0]);
	TEST_ASSERT_EQUAL_UINT64(((uint64_t)SNMP_OBJ_RAIL_MV * 1000U) + 1U,
				 rsp.vuint[0]);
}

static void test_get_too_big(void)
{
	const uint32_t *oids[1] = { oid_sys_descr };
	size_t lens[1] = { ARRAY_LEN(oid_sys_descr) };
	uint8_t small[48];
	size_t n = 0U;
	rsp_t rsp;
	snmp_stats_t st;

	g_str_len = SNMP_OCTET_MAX;
	make_request(SNMP_PDU_GET, 9, 0, 0, oids, lens, 1U);
	TEST_ASSERT_EQUAL_INT(0, snmp_handle(&g_ctx, g_in.b, g_in.n, UPTIME_CS,
					     small, sizeof(small), &n));
	decode_rsp(small, n, &rsp);
	TEST_ASSERT_EQUAL_INT64(SNMP_ERR_TOO_BIG, rsp.err);
	TEST_ASSERT_EQUAL_INT64(0, rsp.err_index);
	TEST_ASSERT_EQUAL_size_t(0U, rsp.n_vb);

	TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.too_big);
	/* A response that was thrown away must not inflate the varbind count. */
	TEST_ASSERT_EQUAL_UINT64(0U, st.varbinds);
}

static void test_too_many_varbinds_is_too_big(void)
{
	const uint32_t *oids[SNMP_MAX_VARBINDS + 1U];
	size_t lens[SNMP_MAX_VARBINDS + 1U];
	rsp_t rsp;
	snmp_stats_t st;
	size_t i;

	for (i = 0U; i < ARRAY_LEN(oids); i++) {
		oids[i] = oid_sys_uptime;
		lens[i] = ARRAY_LEN(oid_sys_uptime);
	}
	make_request(SNMP_PDU_GET, 11, 0, 0, oids, lens, ARRAY_LEN(oids));
	TEST_ASSERT_EQUAL_INT(0, run());
	decode_rsp(g_out, g_out_len, &rsp);
	TEST_ASSERT_EQUAL_INT32(11, rsp.reqid);
	TEST_ASSERT_EQUAL_INT64(SNMP_ERR_TOO_BIG, rsp.err);
	TEST_ASSERT_EQUAL_size_t(0U, rsp.n_vb);
	TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.too_big);
}

/* ======================================================================== */
/* GETNEXT / walk                                                            */
/* ======================================================================== */

static void test_getnext_walks_the_whole_mib(void)
{
	uint32_t cur[SNMP_OID_MAX_LEN];
	size_t cur_n;
	size_t steps = 0U;
	const size_t total = snmp_mib_instance_count();

	memcpy(cur, oid_internet, sizeof(oid_internet));
	cur_n = ARRAY_LEN(oid_internet);

	for (;;) {
		const uint32_t *oids[1] = { cur };
		size_t lens[1] = { cur_n };
		rsp_t rsp;

		make_request(SNMP_PDU_GETNEXT, (int32_t)steps + 1, 0, 0, oids,
			     lens, 1U);
		TEST_ASSERT_EQUAL_INT(0, run());
		decode_rsp(g_out, g_out_len, &rsp);
		TEST_ASSERT_EQUAL_size_t(1U, rsp.n_vb);

		if (rsp.vtype[0] == SNMP_TAG_END_OF_MIB_VIEW) {
			break;
		}
		TEST_ASSERT_TRUE_MESSAGE(
			oid_less(cur, cur_n, rsp.oid[0], rsp.oid_n[0]),
			"GETNEXT must return a strictly greater OID");
		memcpy(cur, rsp.oid[0], rsp.oid_n[0] * sizeof(cur[0]));
		cur_n = rsp.oid_n[0];
		steps++;
		TEST_ASSERT_TRUE(steps <= total + 1U);
	}

	TEST_ASSERT_EQUAL_size_t(total, steps);
}

static void test_getnext_from_an_overlong_name(void)
{
	uint32_t deep[SNMP_OID_MAX_LEN + 3];
	const uint32_t *oids[1] = { deep };
	size_t lens[1] = { ARRAY_LEN(deep) };
	rsp_t rsp;
	size_t i;

	deep[0] = 1U;
	deep[1] = 3U;
	for (i = 2U; i < ARRAY_LEN(deep); i++) {
		deep[i] = 1U;
	}
	make_request(SNMP_PDU_GETNEXT, 5, 0, 0, oids, lens, 1U);
	TEST_ASSERT_EQUAL_INT(0, run());
	decode_rsp(g_out, g_out_len, &rsp);
	TEST_ASSERT_EQUAL_size_t(1U, rsp.n_vb);
	TEST_ASSERT_EQUAL_HEX8(SNMP_TAG_END_OF_MIB_VIEW, rsp.vtype[0]);
}

static void test_getnext_too_big(void)
{
	const uint32_t *oids[1] = { oid_sys_descr };
	size_t lens[1] = { ARRAY_LEN(oid_sys_descr) };
	uint8_t small[46];
	size_t n = 0U;
	rsp_t rsp;

	/* sysDescr.0's successor is sysObjectID.0, whose value is an OID; make
	 * the buffer too small for the varbind but big enough for the shell. */
	make_request(SNMP_PDU_GETNEXT, 21, 0, 0, oids, lens, 1U);
	TEST_ASSERT_EQUAL_INT(0, snmp_handle(&g_ctx, g_in.b, g_in.n, UPTIME_CS,
					     small, sizeof(small), &n));
	decode_rsp(small, n, &rsp);
	TEST_ASSERT_EQUAL_INT64(SNMP_ERR_TOO_BIG, rsp.err);
	TEST_ASSERT_EQUAL_size_t(0U, rsp.n_vb);
}

/* ======================================================================== */
/* GETBULK                                                                   */
/* ======================================================================== */

static void test_getbulk_round_major_order(void)
{
	const uint32_t *oids[2] = { oid_sys_descr, oid_sys_descr };
	size_t lens[2] = { ARRAY_LEN(oid_sys_descr), ARRAY_LEN(oid_sys_descr) };
	rsp_t rsp;
	rsp_t walk;
	uint32_t chain[4][SNMP_OID_MAX_LEN];
	size_t chain_n[4];
	size_t i;
	snmp_stats_t st;

	/* Reference: what four successive GETNEXTs from sysDescr.0 return. */
	{
		uint32_t cur[SNMP_OID_MAX_LEN];
		size_t cur_n = ARRAY_LEN(oid_sys_descr);

		memcpy(cur, oid_sys_descr, sizeof(oid_sys_descr));
		for (i = 0U; i < 4U; i++) {
			const uint32_t *one[1] = { cur };
			size_t onelen[1] = { cur_n };

			make_request(SNMP_PDU_GETNEXT, 1, 0, 0, one, onelen,
				     1U);
			TEST_ASSERT_EQUAL_INT(0, run());
			decode_rsp(g_out, g_out_len, &walk);
			memcpy(chain[i], walk.oid[0],
			       walk.oid_n[0] * sizeof(cur[0]));
			chain_n[i] = walk.oid_n[0];
			memcpy(cur, walk.oid[0],
			       walk.oid_n[0] * sizeof(cur[0]));
			cur_n = walk.oid_n[0];
		}
	}

	TEST_ASSERT_EQUAL_INT(0, snmp_stats_reset(&g_ctx));
	make_request(SNMP_PDU_GETBULK, 77, 1 /* non-repeaters */,
		     3 /* max-repetitions */, oids, lens, 2U);
	TEST_ASSERT_EQUAL_INT(0, run());
	decode_rsp(g_out, g_out_len, &rsp);

	/* one non-repeater + three repetitions of the single repeater */
	TEST_ASSERT_EQUAL_size_t(4U, rsp.n_vb);
	for (i = 0U; i < 4U; i++) {
		TEST_ASSERT_TRUE(oid_equal(rsp.oid[i], rsp.oid_n[i],
					   chain[(i == 0U) ? 0U : (i - 1U)],
					   chain_n[(i == 0U) ? 0U : (i - 1U)]));
	}

	TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.getbulk);
	TEST_ASSERT_EQUAL_UINT64(4U, st.varbinds);
}

static void test_getbulk_stops_at_end_of_mib(void)
{
	const uint32_t *oids[1] = { oid_last };
	size_t lens[1] = { ARRAY_LEN(oid_last) };
	rsp_t rsp;

	make_request(SNMP_PDU_GETBULK, 1, 0, 5, oids, lens, 1U);
	TEST_ASSERT_EQUAL_INT(0, run());
	decode_rsp(g_out, g_out_len, &rsp);
	/* One endOfMibView and no repetition of it. */
	TEST_ASSERT_EQUAL_size_t(1U, rsp.n_vb);
	TEST_ASSERT_EQUAL_HEX8(SNMP_TAG_END_OF_MIB_VIEW, rsp.vtype[0]);
}

static void test_getbulk_negative_fields_are_clamped(void)
{
	const uint32_t *oids[1] = { oid_internet };
	size_t lens[1] = { ARRAY_LEN(oid_internet) };
	rsp_t rsp;

	make_request(SNMP_PDU_GETBULK, 2, -4, -9, oids, lens, 1U);
	TEST_ASSERT_EQUAL_INT(0, run());
	decode_rsp(g_out, g_out_len, &rsp);
	TEST_ASSERT_EQUAL_size_t(0U, rsp.n_vb);

	/* non-repeaters larger than the varbind count is also legal input. */
	make_request(SNMP_PDU_GETBULK, 3, 9, 2, oids, lens, 1U);
	TEST_ASSERT_EQUAL_INT(0, run());
	decode_rsp(g_out, g_out_len, &rsp);
	TEST_ASSERT_EQUAL_size_t(1U, rsp.n_vb);
}

static void test_getbulk_truncates_instead_of_failing(void)
{
	const uint32_t *oids[1] = { oid_internet };
	size_t lens[1] = { ARRAY_LEN(oid_internet) };
	uint8_t small[80];
	size_t n = 0U;
	rsp_t rsp;
	snmp_stats_t st;

	make_request(SNMP_PDU_GETBULK, 4, 0, 40, oids, lens, 1U);
	TEST_ASSERT_EQUAL_INT(0, snmp_handle(&g_ctx, g_in.b, g_in.n, UPTIME_CS,
					     small, sizeof(small), &n));
	TEST_ASSERT_TRUE(n <= sizeof(small));
	decode_rsp(small, n, &rsp);
	TEST_ASSERT_EQUAL_INT64(SNMP_ERR_NO_ERROR, rsp.err);
	TEST_ASSERT_TRUE(rsp.n_vb >= 1U);
	TEST_ASSERT_TRUE(rsp.n_vb < 40U);
	TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(0U, st.too_big);
}

static void test_getbulk_truncates_a_non_repeater(void)
{
	const uint32_t *oids[3] = { oid_internet, oid_internet, oid_internet };
	size_t lens[3] = { ARRAY_LEN(oid_internet), ARRAY_LEN(oid_internet),
			   ARRAY_LEN(oid_internet) };
	uint8_t small[48];
	size_t n = 0U;
	rsp_t rsp;

	make_request(SNMP_PDU_GETBULK, 5, 3, 0, oids, lens, 3U);
	TEST_ASSERT_EQUAL_INT(0, snmp_handle(&g_ctx, g_in.b, g_in.n, UPTIME_CS,
					     small, sizeof(small), &n));
	decode_rsp(small, n, &rsp);
	TEST_ASSERT_EQUAL_INT64(SNMP_ERR_NO_ERROR, rsp.err);
	TEST_ASSERT_TRUE(rsp.n_vb < 3U);
}

static void test_getbulk_repetitions_are_capped(void)
{
	snmp_ctx_t c;
	snmp_cfg_t cfg;
	const uint32_t *oids[1] = { oid_internet };
	size_t lens[1] = { ARRAY_LEN(oid_internet) };
	rsp_t rsp;
	size_t n = 0U;

	memset(&cfg, 0, sizeof(cfg));
	cfg.community = "public";
	cfg.getter.get = fake_get;
	cfg.max_repetitions = 2U;
	TEST_ASSERT_EQUAL_INT(0, snmp_init(&c, &cfg));

	make_request(SNMP_PDU_GETBULK, 6, 0, 50, oids, lens, 1U);
	TEST_ASSERT_EQUAL_INT(0, snmp_handle(&c, g_in.b, g_in.n, UPTIME_CS,
					     g_out, sizeof(g_out), &n));
	decode_rsp(g_out, n, &rsp);
	TEST_ASSERT_EQUAL_size_t(2U, rsp.n_vb);
}

/* ======================================================================== */
/* SET, auth, malformed input                                                */
/* ======================================================================== */

static void test_set_is_refused_and_echoes_the_varbinds(void)
{
	const uint32_t *oids[2] = { oid_stratum, oid_sys_uptime };
	size_t lens[2] = { ARRAY_LEN(oid_stratum), ARRAY_LEN(oid_sys_uptime) };
	rsp_t rsp;
	snmp_stats_t st;

	make_request(SNMP_PDU_SET, 55, 0, 0, oids, lens, 2U);
	TEST_ASSERT_EQUAL_INT(0, run());
	decode_rsp(g_out, g_out_len, &rsp);
	TEST_ASSERT_EQUAL_INT32(55, rsp.reqid);
	TEST_ASSERT_EQUAL_INT64(SNMP_ERR_NOT_WRITABLE, rsp.err);
	TEST_ASSERT_EQUAL_INT64(1, rsp.err_index);
	TEST_ASSERT_EQUAL_size_t(2U, rsp.n_vb);
	TEST_ASSERT_TRUE(oid_equal(rsp.oid[0], rsp.oid_n[0], oid_stratum,
				   ARRAY_LEN(oid_stratum)));
	TEST_ASSERT_TRUE(oid_equal(rsp.oid[1], rsp.oid_n[1], oid_sys_uptime,
				   ARRAY_LEN(oid_sys_uptime)));

	TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.set_refused);
	TEST_ASSERT_EQUAL_UINT64(1U, st.responses);
}

static void test_bad_community_is_dropped_and_counted(void)
{
	bld_t pdu;
	bld_t vbl;
	const uint32_t *oids[1] = { oid_sys_uptime };
	size_t lens[1] = { ARRAY_LEN(oid_sys_uptime) };
	snmp_stats_t st;

	make_vbl(&vbl, oids, lens, 1U);
	bld_reset(&pdu);
	bld_int(&pdu, 1);
	bld_int(&pdu, 0);
	bld_int(&pdu, 0);
	bld_raw(&pdu, vbl.b, vbl.n);
	make_msg(SNMP_PDU_GET, &pdu, "private");

	TEST_ASSERT_EQUAL_INT(-EACCES, run());
	TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.bad_community);
	TEST_ASSERT_EQUAL_UINT64(0U, st.responses);

	/* A community that is a prefix of the configured one must not pass. */
	make_msg(SNMP_PDU_GET, &pdu, "pub");
	TEST_ASSERT_EQUAL_INT(-EACCES, run());

	/* An agent configured with no community answers nobody. */
	TEST_ASSERT_EQUAL_INT(0, snmp_set_community(&g_ctx, ""));
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      snmp_handle(&g_ctx, v_get_req, sizeof(v_get_req),
					  0U, g_out, sizeof(g_out),
					  &g_out_len));
	TEST_ASSERT_EQUAL_INT(0, snmp_set_community(&g_ctx, NULL));
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      snmp_handle(&g_ctx, v_get_req, sizeof(v_get_req),
					  0U, g_out, sizeof(g_out),
					  &g_out_len));
}

static void test_wrong_version_is_dropped(void)
{
	bld_t body;
	bld_t pdu;
	bld_t inner;
	snmp_stats_t st;

	bld_reset(&inner);
	bld_int(&inner, 1);
	bld_int(&inner, 0);
	bld_int(&inner, 0);
	{
		bld_t vbl;
		const uint32_t *oids[1] = { oid_sys_uptime };
		size_t lens[1] = { ARRAY_LEN(oid_sys_uptime) };

		make_vbl(&vbl, oids, lens, 1U);
		bld_raw(&inner, vbl.b, vbl.n);
	}
	bld_reset(&pdu);
	bld_wrap(&pdu, SNMP_PDU_GET, &inner);

	bld_reset(&body);
	bld_int(&body, 0); /* SNMPv1 */
	bld_str(&body, "public");
	bld_raw(&body, pdu.b, pdu.n);
	bld_reset(&g_in);
	bld_wrap(&g_in, 0x30U, &body);

	TEST_ASSERT_EQUAL_INT(-EPROTO, run());
	TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.bad_version);
}

static void test_unserved_pdu_types_are_dropped(void)
{
	bld_t pdu;
	bld_t vbl;
	const uint32_t *oids[1] = { oid_sys_uptime };
	size_t lens[1] = { ARRAY_LEN(oid_sys_uptime) };
	snmp_stats_t st;
	static const uint8_t tags[] = { SNMP_PDU_RESPONSE, SNMP_PDU_TRAP_V2,
					SNMP_PDU_INFORM, SNMP_PDU_REPORT };
	size_t i;

	make_vbl(&vbl, oids, lens, 1U);
	bld_reset(&pdu);
	bld_int(&pdu, 1);
	bld_int(&pdu, 0);
	bld_int(&pdu, 0);
	bld_raw(&pdu, vbl.b, vbl.n);

	for (i = 0U; i < ARRAY_LEN(tags); i++) {
		make_msg(tags[i], &pdu, "public");
		TEST_ASSERT_EQUAL_INT(-ENOTSUP, run());
	}
	TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(ARRAY_LEN(tags), st.unsupported_pdu);
}

static void test_malformed_messages_are_dropped(void)
{
	static const uint8_t not_a_sequence[] = { 0x04, 0x03, 0x02, 0x01,
						  0x01 };
	static const uint8_t truncated[] = { 0x30, 0x10, 0x02, 0x01 };
	snmp_stats_t st;
	size_t i;

	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      snmp_handle(&g_ctx, not_a_sequence,
					  sizeof(not_a_sequence), 0U, g_out,
					  sizeof(g_out), &g_out_len));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      snmp_handle(&g_ctx, truncated, sizeof(truncated),
					  0U, g_out, sizeof(g_out),
					  &g_out_len));

	/* Every prefix of a valid request is either malformed or refused; none
	 * may produce a response. */
	for (i = 1U; i < sizeof(v_get_req); i++) {
		size_t n = 0U;
		int rc = snmp_handle(&g_ctx, v_get_req, i, 0U, g_out,
				     sizeof(g_out), &n);

		TEST_ASSERT_TRUE(rc < 0);
	}

	TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&g_ctx, &st));
	TEST_ASSERT_TRUE(st.malformed > 0U);
	TEST_ASSERT_EQUAL_UINT64(0U, st.responses);
}

static void test_malformed_pdu_bodies_are_dropped(void)
{
	bld_t pdu;

	/* request-id wider than Integer32. */
	bld_reset(&pdu);
	{
		static const uint8_t wide[] = { 0x01, 0x02, 0x03,
						0x04, 0x05, 0x06 };

		bld_tlv(&pdu, 0x02U, wide, sizeof(wide));
	}
	bld_int(&pdu, 0);
	bld_int(&pdu, 0);
	{
		bld_t vbl;
		const uint32_t *oids[1] = { oid_sys_uptime };
		size_t lens[1] = { ARRAY_LEN(oid_sys_uptime) };

		make_vbl(&vbl, oids, lens, 1U);
		bld_raw(&pdu, vbl.b, vbl.n);
	}
	make_msg(SNMP_PDU_GET, &pdu, "public");
	TEST_ASSERT_EQUAL_INT(-EBADMSG, run());

	/* varbind list is not a SEQUENCE. */
	bld_reset(&pdu);
	bld_int(&pdu, 1);
	bld_int(&pdu, 0);
	bld_int(&pdu, 0);
	bld_str(&pdu, "nope");
	make_msg(SNMP_PDU_GET, &pdu, "public");
	TEST_ASSERT_EQUAL_INT(-EBADMSG, run());

	/* a varbind that is not a SEQUENCE. */
	bld_reset(&pdu);
	bld_int(&pdu, 1);
	bld_int(&pdu, 0);
	bld_int(&pdu, 0);
	{
		bld_t inner;

		bld_reset(&inner);
		bld_str(&inner, "x");
		bld_wrap(&pdu, 0x30U, &inner);
	}
	make_msg(SNMP_PDU_GET, &pdu, "public");
	TEST_ASSERT_EQUAL_INT(-EBADMSG, run());

	/* a varbind whose name is not an OID. */
	bld_reset(&pdu);
	bld_int(&pdu, 1);
	bld_int(&pdu, 0);
	bld_int(&pdu, 0);
	{
		bld_t one;
		bld_t inner;

		bld_reset(&one);
		bld_str(&one, "not an oid");
		bld_tlv(&one, 0x05U, NULL, 0U);
		bld_reset(&inner);
		bld_wrap(&inner, 0x30U, &one);
		bld_wrap(&pdu, 0x30U, &inner);
	}
	make_msg(SNMP_PDU_GET, &pdu, "public");
	TEST_ASSERT_EQUAL_INT(-EBADMSG, run());

	/* a varbind with a name but no value. */
	bld_reset(&pdu);
	bld_int(&pdu, 1);
	bld_int(&pdu, 0);
	bld_int(&pdu, 0);
	{
		bld_t one;
		bld_t inner;

		bld_reset(&one);
		bld_oid(&one, oid_sys_uptime, ARRAY_LEN(oid_sys_uptime));
		bld_reset(&inner);
		bld_wrap(&inner, 0x30U, &one);
		bld_wrap(&pdu, 0x30U, &inner);
	}
	make_msg(SNMP_PDU_GET, &pdu, "public");
	TEST_ASSERT_EQUAL_INT(-EBADMSG, run());

	/* PDU truncated before error-status. */
	bld_reset(&pdu);
	bld_int(&pdu, 1);
	make_msg(SNMP_PDU_GET, &pdu, "public");
	TEST_ASSERT_EQUAL_INT(-EBADMSG, run());

	/* PDU truncated before error-index. */
	bld_reset(&pdu);
	bld_int(&pdu, 1);
	bld_int(&pdu, 0);
	make_msg(SNMP_PDU_GET, &pdu, "public");
	TEST_ASSERT_EQUAL_INT(-EBADMSG, run());

	/* community is not an OCTET STRING. */
	{
		bld_t body;
		bld_t inner;
		bld_t wrapped;

		bld_reset(&inner);
		bld_int(&inner, 1);
		bld_int(&inner, 0);
		bld_int(&inner, 0);
		bld_reset(&wrapped);
		bld_wrap(&wrapped, SNMP_PDU_GET, &inner);

		bld_reset(&body);
		bld_int(&body, 1);
		bld_int(&body, 7); /* not a community */
		bld_raw(&body, wrapped.b, wrapped.n);
		bld_reset(&g_in);
		bld_wrap(&g_in, 0x30U, &body);
		TEST_ASSERT_EQUAL_INT(-EBADMSG, run());
	}

	/* PDU tag is not a constructed context type at all. */
	{
		bld_t body;

		bld_reset(&body);
		bld_int(&body, 1);
		bld_str(&body, "public");
		bld_str(&body, "junk");
		bld_reset(&g_in);
		bld_wrap(&g_in, 0x30U, &body);
		TEST_ASSERT_EQUAL_INT(-ENOTSUP, run());
	}
}

static void test_empty_varbind_list_is_answered(void)
{
	bld_t pdu;
	rsp_t rsp;

	bld_reset(&pdu);
	bld_int(&pdu, 1);
	bld_int(&pdu, 0);
	bld_int(&pdu, 0);
	bld_tlv(&pdu, 0x30U, NULL, 0U);
	make_msg(SNMP_PDU_GET, &pdu, "public");

	TEST_ASSERT_EQUAL_INT(0, run());
	decode_rsp(g_out, g_out_len, &rsp);
	TEST_ASSERT_EQUAL_size_t(0U, rsp.n_vb);
	TEST_ASSERT_EQUAL_INT64(SNMP_ERR_NO_ERROR, rsp.err);
}

/* ======================================================================== */
/* traps                                                                     */
/* ======================================================================== */

static void test_trap_oids_and_names(void)
{
	uint32_t arcs[SNMP_OID_MAX_LEN];
	static const uint32_t want_cold[] = { 1, 3, 6, 1, 6, 3, 1, 1, 5, 1 };
	static const uint32_t want_lock[] = { 1, 3, 6, 1, 4,	1,
					      PEN, 1, 0, 1 };
	int n;

	TEST_ASSERT_EQUAL_INT(-EINVAL, snmp_trap_oid(SNMP_TRAP_COLD_START,
						     NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_trap_oid((snmp_trap_t)SNMP_TRAP__COUNT,
					    arcs));

	n = snmp_trap_oid(SNMP_TRAP_COLD_START, arcs);
	TEST_ASSERT_EQUAL_INT((int)ARRAY_LEN(want_cold), n);
	TEST_ASSERT_EQUAL_UINT32_ARRAY(want_cold, arcs, (size_t)n);

	n = snmp_trap_oid(SNMP_TRAP_LOCK_ACQUIRED, arcs);
	TEST_ASSERT_EQUAL_INT((int)ARRAY_LEN(want_lock), n);
	TEST_ASSERT_EQUAL_UINT32_ARRAY(want_lock, arcs, (size_t)n);

	TEST_ASSERT_EQUAL_STRING("coldStart",
				 snmp_trap_name(SNMP_TRAP_COLD_START));
	TEST_ASSERT_EQUAL_STRING("authFailure",
				 snmp_trap_name(SNMP_TRAP_AUTH_FAILURE));
	TEST_ASSERT_EQUAL_STRING("unknown",
				 snmp_trap_name((snmp_trap_t)SNMP_TRAP__COUNT));

	/* Every notification has a distinct OID. */
	{
		size_t i;
		size_t j;
		uint32_t all[SNMP_TRAP__COUNT][SNMP_OID_MAX_LEN];
		int lens[SNMP_TRAP__COUNT];

		for (i = 0U; i < (size_t)SNMP_TRAP__COUNT; i++) {
			lens[i] = snmp_trap_oid((snmp_trap_t)i, all[i]);
			TEST_ASSERT_TRUE(lens[i] > 0);
		}
		for (i = 0U; i < (size_t)SNMP_TRAP__COUNT; i++) {
			for (j = i + 1U; j < (size_t)SNMP_TRAP__COUNT; j++) {
				TEST_ASSERT_FALSE(oid_equal(
					all[i], (size_t)lens[i], all[j],
					(size_t)lens[j]));
			}
		}
	}
}

static void test_coldstart_trap_known_answer(void)
{
	size_t n = 0U;
	snmp_stats_t st;

	TEST_ASSERT_EQUAL_INT(0, snmp_make_trap(&g_ctx, SNMP_TRAP_COLD_START,
						1234U, NULL, 0U, g_out,
						sizeof(g_out), &n));
	TEST_ASSERT_EQUAL_size_t(sizeof(v_coldstart_trap), n);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(v_coldstart_trap, g_out, n);

	TEST_ASSERT_EQUAL_INT(0, snmp_stats_get(&g_ctx, &st));
	TEST_ASSERT_EQUAL_UINT64(1U, st.traps);
}

static void test_trap_with_bindings(void)
{
	snmp_bind_t binds[3];
	size_t n = 0U;
	snmp_rd_t r;
	snmp_rd_t body;
	snmp_rd_t pdu;
	snmp_rd_t vbl;
	uint8_t tag;
	size_t l;
	int64_t v;
	const uint8_t *s;
	size_t sn;
	size_t count = 0U;

	binds[0].obj = (uint16_t)SNMP_OBJ_LOCK_STATE;
	binds[0].inst = 0U;
	binds[1].obj = (uint16_t)SNMP_OBJ_RAIL_MV;
	binds[1].inst = 3U;
	/* Unknown binding: skipped rather than failing the whole trap. */
	binds[2].obj = (uint16_t)SNMP_OBJ__COUNT;
	binds[2].inst = 0U;

	TEST_ASSERT_EQUAL_INT(0,
			      snmp_make_trap(&g_ctx, SNMP_TRAP_RAIL_ALARM, 99U,
					     binds, 3U, g_out, sizeof(g_out),
					     &n));

	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&r, g_out, n));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_hdr(&r, &tag, &l));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&body, &g_out[r.off], l));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_int(&body, &v));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_octets(&body, &s, &sn));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_hdr(&body, &tag, &l));
	TEST_ASSERT_EQUAL_HEX8(SNMP_PDU_TRAP_V2, tag);
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&pdu, &body.buf[body.off], l));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_int(&pdu, &v));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_int(&pdu, &v));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_int(&pdu, &v));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_hdr(&pdu, &tag, &l));
	TEST_ASSERT_EQUAL_INT(0, snmp_rd_init(&vbl, &pdu.buf[pdu.off], l));
	while (!snmp_rd_done(&vbl)) {
		TEST_ASSERT_EQUAL_INT(0, snmp_rd_hdr(&vbl, &tag, &l));
		TEST_ASSERT_EQUAL_INT(0, snmp_rd_skip(&vbl, l));
		count++;
	}
	/* sysUpTime.0 + snmpTrapOID.0 + two resolvable bindings. */
	TEST_ASSERT_EQUAL_size_t(4U, count);
}

static void test_trap_argument_guards(void)
{
	size_t n = 0U;
	snmp_bind_t b;
	snmp_ctx_t unready;
	uint8_t tiny[16];
	size_t i;

	memset(&unready, 0, sizeof(unready));
	b.obj = (uint16_t)SNMP_OBJ_STRATUM;
	b.inst = 0U;

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_make_trap(NULL, SNMP_TRAP_COLD_START, 0U,
					     NULL, 0U, g_out, sizeof(g_out),
					     &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_make_trap(&unready, SNMP_TRAP_COLD_START, 0U,
					     NULL, 0U, g_out, sizeof(g_out),
					     &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_make_trap(&g_ctx, SNMP_TRAP_COLD_START, 0U,
					     NULL, 0U, NULL, sizeof(g_out),
					     &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_make_trap(&g_ctx, SNMP_TRAP_COLD_START, 0U,
					     NULL, 0U, g_out, sizeof(g_out),
					     NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_make_trap(&g_ctx, SNMP_TRAP_COLD_START, 0U,
					     NULL, 2U, g_out, sizeof(g_out),
					     &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      snmp_make_trap(&g_ctx,
					     (snmp_trap_t)SNMP_TRAP__COUNT, 0U,
					     NULL, 0U, g_out, sizeof(g_out),
					     &n));

	TEST_ASSERT_EQUAL_INT(0, snmp_set_community(&g_ctx, NULL));
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      snmp_make_trap(&g_ctx, SNMP_TRAP_COLD_START, 0U,
					     NULL, 0U, g_out, sizeof(g_out),
					     &n));
	TEST_ASSERT_EQUAL_INT(0, snmp_set_community(&g_ctx, ""));
	TEST_ASSERT_EQUAL_INT(-EACCES,
			      snmp_make_trap(&g_ctx, SNMP_TRAP_COLD_START, 0U,
					     NULL, 0U, g_out, sizeof(g_out),
					     &n));
	TEST_ASSERT_EQUAL_INT(0, snmp_set_community(&g_ctx, "public"));

	/* Every buffer size short of the full trap must fail cleanly, walking
	 * the -ENOSPC return of each nested writer call in turn. */
	for (i = 0U; i < sizeof(v_coldstart_trap); i++) {
		size_t m = 0U;

		TEST_ASSERT_EQUAL_INT(-ENOSPC,
				      snmp_make_trap(&g_ctx,
						     SNMP_TRAP_COLD_START,
						     1234U, &b, 1U, g_out, i,
						     &m));
	}
	TEST_ASSERT_EQUAL_INT(-ENOSPC, snmp_make_trap(&g_ctx,
						      SNMP_TRAP_COLD_START, 0U,
						      NULL, 0U, tiny,
						      sizeof(tiny), &n));
}

/* ======================================================================== */
/* robustness                                                                */
/* ======================================================================== */

static void test_random_input_never_crashes(void)
{
	uint32_t s = 0xC0FFEEU;
	size_t iter;

	for (iter = 0U; iter < 4000U; iter++) {
		uint8_t buf[128];
		size_t len;
		size_t i;
		size_t n = 0U;

		s ^= s << 13;
		s ^= s >> 17;
		s ^= s << 5;
		len = 1U + (s % sizeof(buf));

		for (i = 0U; i < len; i++) {
			s ^= s << 13;
			s ^= s >> 17;
			s ^= s << 5;
			buf[i] = (uint8_t)(s >> 11);
		}
		/* Half the corpus starts as a well-formed envelope so the
		 * fuzzing reaches past the first header check. */
		if ((iter & 1U) == 0U && len > sizeof(v_get_req)) {
			memcpy(buf, v_get_req, sizeof(v_get_req));
		}

		(void)snmp_handle(&g_ctx, buf, len, UPTIME_CS, g_out,
				  sizeof(g_out), &n);
	}
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_reader_argument_guards);
	RUN_TEST(test_header_rejects_illegal_encodings);
	RUN_TEST(test_header_accepts_long_form_lengths);
	RUN_TEST(test_integer_round_trip);
	RUN_TEST(test_integer_known_answers);
	RUN_TEST(test_integer_rejects_bad_encodings);
	RUN_TEST(test_unsigned_round_trip_and_padding);
	RUN_TEST(test_unsigned_rejects_bad_encodings);
	RUN_TEST(test_octets_round_trip);
	RUN_TEST(test_oid_round_trip_and_known_answers);
	RUN_TEST(test_oid_writer_rejects_illegal_arcs);
	RUN_TEST(test_oid_reader_rejects_illegal_encodings);
	RUN_TEST(test_oid_reader_reports_truncation);
	RUN_TEST(test_writer_argument_guards_and_capacity);
	RUN_TEST(test_constructed_length_shrinks);
	RUN_TEST(test_value_encoding_covers_every_type);

	RUN_TEST(test_mib_is_sorted_and_consistent);
	RUN_TEST(test_mib_lookup_edges);

	RUN_TEST(test_init_and_config);
	RUN_TEST(test_handle_argument_guards);

	RUN_TEST(test_get_known_answer);
	RUN_TEST(test_getnext_known_answer);
	RUN_TEST(test_get_exception_varbinds);
	RUN_TEST(test_get_resolves_a_rail_row);
	RUN_TEST(test_get_too_big);
	RUN_TEST(test_too_many_varbinds_is_too_big);

	RUN_TEST(test_getnext_walks_the_whole_mib);
	RUN_TEST(test_getnext_from_an_overlong_name);
	RUN_TEST(test_getnext_too_big);

	RUN_TEST(test_getbulk_round_major_order);
	RUN_TEST(test_getbulk_stops_at_end_of_mib);
	RUN_TEST(test_getbulk_negative_fields_are_clamped);
	RUN_TEST(test_getbulk_truncates_instead_of_failing);
	RUN_TEST(test_getbulk_truncates_a_non_repeater);
	RUN_TEST(test_getbulk_repetitions_are_capped);

	RUN_TEST(test_set_is_refused_and_echoes_the_varbinds);
	RUN_TEST(test_bad_community_is_dropped_and_counted);
	RUN_TEST(test_wrong_version_is_dropped);
	RUN_TEST(test_unserved_pdu_types_are_dropped);
	RUN_TEST(test_malformed_messages_are_dropped);
	RUN_TEST(test_malformed_pdu_bodies_are_dropped);
	RUN_TEST(test_empty_varbind_list_is_answered);

	RUN_TEST(test_trap_oids_and_names);
	RUN_TEST(test_coldstart_trap_known_answer);
	RUN_TEST(test_trap_with_bindings);
	RUN_TEST(test_trap_argument_guards);

	RUN_TEST(test_random_input_never_crashes);

	return UNITY_END();
}
