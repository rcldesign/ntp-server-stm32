/*
 * STS1000 "Meridian" — core/cfg unit tests.
 *
 * What these pin down, and why:
 *
 *  - The schema itself is an interface, not an implementation detail: MCP
 *    CFG_LIST pages through it in ID order and cfg.c binary-searches it, so
 *    sortedness, uniqueness and the per-group census are asserted directly.
 *    A key silently deleted or renumbered breaks a stored config; the census
 *    makes that a test failure rather than a field surprise.
 *
 *  - The TLV vectors are hand-built byte-by-byte from the layout documented in
 *    cfg.h, not produced by cfg_export_read() — a round-trip against the
 *    encoder proves only that the two halves agree with each other.
 *
 *  - Commit atomicity is checked by observing the *live* tree after a rejected
 *    commit, not by trusting the return code.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "cfg/cfg.h"
#include "util/crc.h"

/* ------------------------------------------------------------------------- */
/* Fake persistent store                                                     */
/* ------------------------------------------------------------------------- */

#define STORE_SLOTS 128U
#define STORE_REC   72U

typedef struct {
	struct {
		uint16_t id;
		uint8_t  buf[STORE_REC];
		uint8_t  len;
		bool     used;
	} e[STORE_SLOTS];
	int      fail_save_id;  /* -1 = never fail */
	int      fail_erase_id;
	int      fail_load_id;
	int      load_ret;      /* what load() returns for fail_load_id */
	uint32_t saves;
	uint32_t erases;
} fake_store_t;

static fake_store_t g_store;

static void store_clear(void)
{
	memset(&g_store, 0, sizeof(g_store));
	g_store.fail_save_id = -1;
	g_store.fail_erase_id = -1;
	g_store.fail_load_id = -1;
}

static int store_slot(uint16_t id, bool create)
{
	size_t i;

	for (i = 0U; i < STORE_SLOTS; i++) {
		if (g_store.e[i].used && (g_store.e[i].id == id)) {
			return (int)i;
		}
	}
	if (!create) {
		return -1;
	}
	for (i = 0U; i < STORE_SLOTS; i++) {
		if (!g_store.e[i].used) {
			g_store.e[i].used = true;
			g_store.e[i].id = id;
			return (int)i;
		}
	}
	return -1;
}

static int fs_load(void *ctx, uint16_t id, void *buf, size_t cap)
{
	int s;

	(void)ctx;
	if ((g_store.fail_load_id >= 0) && ((uint16_t)g_store.fail_load_id == id)) {
		return g_store.load_ret;
	}
	s = store_slot(id, false);
	if (s < 0) {
		return -2; /* ENOENT-like, per port_store.h */
	}
	if ((size_t)g_store.e[s].len > cap) {
		return -5;
	}
	memcpy(buf, g_store.e[s].buf, g_store.e[s].len);
	return (int)g_store.e[s].len;
}

static int fs_save(void *ctx, uint16_t id, const void *buf, size_t len)
{
	int s;

	(void)ctx;
	if ((g_store.fail_save_id >= 0) && ((uint16_t)g_store.fail_save_id == id)) {
		return -5;
	}
	if (len > STORE_REC) {
		return -28;
	}
	s = store_slot(id, true);
	if (s < 0) {
		return -28;
	}
	memcpy(g_store.e[s].buf, buf, len);
	g_store.e[s].len = (uint8_t)len;
	g_store.saves++;
	return 0;
}

static int fs_erase(void *ctx, uint16_t id)
{
	int s;

	(void)ctx;
	if ((g_store.fail_erase_id >= 0) &&
	    ((uint16_t)g_store.fail_erase_id == id)) {
		return -5;
	}
	s = store_slot(id, false);
	if (s < 0) {
		return -2;
	}
	g_store.e[s].used = false;
	g_store.erases++;
	return 0;
}

static const port_store_t g_store_port = {
	.load = fs_load,
	.save = fs_save,
	.erase = fs_erase,
	.ctx = NULL,
};

/* Plant a raw store record so the load path can be fed deliberate garbage. */
static void store_put_raw(uint16_t id, const uint8_t *rec, size_t len)
{
	int s = store_slot(id, true);

	TEST_ASSERT_GREATER_OR_EQUAL_INT(0, s);
	TEST_ASSERT_LESS_OR_EQUAL_size_t(STORE_REC, len);
	memcpy(g_store.e[s].buf, rec, len);
	g_store.e[s].len = (uint8_t)len;
}

/* ------------------------------------------------------------------------- */
/* Fixtures                                                                  */
/* ------------------------------------------------------------------------- */

/* ~11 KB of live+staged mirrors; keep it out of the stack. */
static cfg_ctx_t g_cfg;

void setUp(void)
{
	store_clear();
	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, &g_store_port));
}

/* ------------------------------------------------------------------------- */
/* Schema shape                                                              */
/* ------------------------------------------------------------------------- */

/*
 * The census. Changing it is a deliberate act: an export written by an older
 * firmware carries the older key set, so a key that vanishes needs a
 * migration, and this table is the reminder.
 */
static void test_schema_group_census(void)
{
	static const struct {
		uint8_t group;
		uint16_t expect;
	} census[] = {
		{ CFG_G_NET, 9 },   { CFG_G_NTP, 7 },  { CFG_G_NTS, 4 },
		{ CFG_G_PTP, 10 },  { CFG_G_GNSS, 7 }, { CFG_G_TIMING, 8 },
		{ CFG_G_POWER, 5 }, { CFG_G_UI, 3 },   { CFG_G_LOG, 5 },
		{ CFG_G_SEC, 4 },   { CFG_G_SNMP, 4 }, { CFG_G_CAL, 13 },
	};
	size_t i;
	uint16_t total = 0U;

	for (i = 0U; i < (sizeof(census) / sizeof(census[0])); i++) {
		uint16_t n = 0U;
		size_t j;

		for (j = 0U; j < cfg_key_count(); j++) {
			if (CFG_GROUP(cfg_key_at(j)->id) == census[i].group) {
				n++;
			}
		}
		TEST_ASSERT_EQUAL_UINT16(census[i].expect, n);
		total = (uint16_t)(total + n);
	}

	TEST_ASSERT_EQUAL_size_t(79U, cfg_key_count());
	TEST_ASSERT_EQUAL_UINT16(79U, total); /* no key outside a known group */
}

static void test_schema_is_sorted_and_well_formed(void)
{
	size_t i;

	for (i = 0U; i < cfg_key_count(); i++) {
		const cfg_key_t *k = cfg_key_at(i);

		TEST_ASSERT_NOT_NULL(k);
		TEST_ASSERT_NOT_NULL(k->name);
		TEST_ASSERT_TRUE(strlen(k->name) > 0U);
		TEST_ASSERT_LESS_OR_EQUAL_size_t(CFG_NAME_MAX, strlen(k->name));
		TEST_ASSERT_TRUE((k->type >= CFG_T_BOOL) &&
				 (k->type <= CFG_T_BLOB));
		/* Exactly one of the two apply flags, always. */
		TEST_ASSERT_TRUE(((k->flags & CFG_F_RUNTIME_APPLY) != 0U) !=
				 ((k->flags & CFG_F_REBOOT_REQUIRED) != 0U));

		if (i > 0U) {
			TEST_ASSERT_TRUE(cfg_key_at(i - 1U)->id < k->id);
		}
		if ((k->type == CFG_T_STR) || (k->type == CFG_T_BLOB)) {
			TEST_ASSERT_TRUE(k->maxlen > 0U);
			TEST_ASSERT_LESS_OR_EQUAL_UINT16(CFG_VAL_MAX,
							 k->maxlen);
		} else {
			TEST_ASSERT_EQUAL_UINT16(0U, k->maxlen);
		}
	}

	TEST_ASSERT_NULL(cfg_key_at(cfg_key_count()));
}

static void test_schema_defaults_pass_their_own_bounds(void)
{
	size_t i;

	/* A default outside its own range would make every fresh boot invalid
	 * and every commit that touches the key impossible to undo. */
	for (i = 0U; i < cfg_key_count(); i++) {
		const cfg_key_t *k = cfg_key_at(i);
		cfg_val_t v;

		TEST_ASSERT_EQUAL_INT(0, cfg_get(&g_cfg, k->id, &v));
		TEST_ASSERT_EQUAL_UINT8(k->type, v.type);
		TEST_ASSERT_EQUAL_INT(0, cfg_set(&g_cfg, k->id, &v));
	}
	TEST_ASSERT_EQUAL_UINT16((uint16_t)cfg_key_count(),
				 cfg_staged_count(&g_cfg));
}

/*
 * Defaults that are security- or availability-relevant, pinned so a later edit
 * has to argue with a test rather than slip through.
 *
 * A schema default is what a factory-reset box, a corrupt-NVS boot and every
 * first power-up actually run on, so "insecure but documented" is not a
 * position — nobody reads the doc before the box is on the network.
 */
static void test_schema_defaults_fail_safe(void)
{
	uint64_t u = 0U;
	uint8_t buf[CFG_VAL_MAX];
	size_t len = 1U;

	/* SNMPv2c's community string IS its authentication. "public" would have
	 * shipped a world-readable agent on every box; empty makes core/snmp
	 * answer -EACCES and the glue pass NULL, which disables the agent. */
	TEST_ASSERT_EQUAL_INT(0,
		cfg_get_bytes(&g_cfg, CFG_ID_SNMP_COMMUNITY, buf, sizeof(buf),
			      &len));
	TEST_ASSERT_EQUAL_size_t(0U, len);

	/* And the agent is off to begin with. */
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_SNMP_ENABLE, &u));
	TEST_ASSERT_EQUAL_UINT64(0U, u);

	/* Interleaved mode is opt-in: the glue reads this key rather than
	 * core/ntp's library default, so a 1 here is what puts the mode and its
	 * per-client timestamp cache live on a shipped box. */
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_NTP_INTERLEAVED, &u));
	TEST_ASSERT_EQUAL_UINT64(0U, u);

	/* The per-client token bucket must be armed. 0 disables it, which lets a
	 * single source drain the aggregate bucket and KoD every other client.
	 * core/ntp documents 8 req/s burst 16 as its own default. */
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_NTP_RATE_QPS, &u));
	TEST_ASSERT_EQUAL_UINT64(8U, u);
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_NTP_RATE_BURST, &u));
	TEST_ASSERT_EQUAL_UINT64(16U, u);

	/* Still reachable: 0 is a legal value an operator may ask for. */
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_NTP_RATE_QPS, 0U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));

	/* Auth is required and the console is read-only out of the box. */
	TEST_ASSERT_EQUAL_INT(0,
		cfg_get_u64(&g_cfg, CFG_ID_SEC_AUTH_REQUIRED, &u));
	TEST_ASSERT_EQUAL_UINT64(1U, u);
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_SEC_CONSOLE_RO, &u));
	TEST_ASSERT_EQUAL_UINT64(1U, u);
}

/*
 * cfg_schema_xvalidate(): the schema's own joint constraints. snmp.enable
 * requires a non-empty snmp.community, so the empty default cannot be turned
 * into an open agent by enabling the service and forgetting the community.
 */
static void test_schema_xvalidate_gates_snmp_on_its_community(void)
{
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_schema_xvalidate(NULL, NULL));

	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_validate_hook(&g_cfg, cfg_schema_xvalidate, NULL));

	/* Defaults are consistent: the agent is off, so the empty community is
	 * not a problem and an unrelated commit still works. */
	TEST_ASSERT_EQUAL_INT(0, cfg_schema_xvalidate(&g_cfg, NULL));
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_UI_BRIGHTNESS, 42U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));

	/* Enabling the agent alone is refused, and nothing is applied. */
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_SNMP_ENABLE, 1U));
	TEST_ASSERT_EQUAL_INT(-EPROTO, cfg_commit(&g_cfg, NULL));
	{
		uint64_t u = 1U;

		TEST_ASSERT_EQUAL_INT(0,
			cfg_get_u64(&g_cfg, CFG_ID_SNMP_ENABLE, &u));
		TEST_ASSERT_EQUAL_UINT64(0U, u);
	}
	/* Staging survives so the operator can supply the missing field. */
	TEST_ASSERT_TRUE(cfg_is_staged(&g_cfg, CFG_ID_SNMP_ENABLE));

	/* Enabling it together with a community in ONE commit is accepted: the
	 * hook reads the effective tree, not the live one. */
	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_bytes(&g_cfg, CFG_ID_SNMP_COMMUNITY,
			      (const uint8_t *)"n0tpublic", 9U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));

	/* And blanking the community while the agent is live is refused too —
	 * the rule is a property of the tree, not of one command. */
	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_bytes(&g_cfg, CFG_ID_SNMP_COMMUNITY, NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EPROTO, cfg_commit(&g_cfg, NULL));

	/* Disabling the agent in the same commit makes it acceptable again. */
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_SNMP_ENABLE, 0U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));

	/* The schema defaults themselves satisfy the hook, so a factory reset
	 * cannot leave a tree that refuses to commit. */
	TEST_ASSERT_EQUAL_INT(0, cfg_factory_reset(&g_cfg));
	TEST_ASSERT_EQUAL_INT(0, cfg_schema_xvalidate(&g_cfg, NULL));

	TEST_ASSERT_EQUAL_INT(0, cfg_set_validate_hook(&g_cfg, NULL, NULL));
}

static void test_schema_lookup(void)
{
	TEST_ASSERT_EQUAL_UINT16(CFG_ID_NET_DHCP,
				 cfg_key_find(CFG_ID_NET_DHCP)->id);
	TEST_ASSERT_NULL(cfg_key_find(0xFFFF));
	TEST_ASSERT_EQUAL_INT(-ENOENT, cfg_key_index(0xFFFF));
	TEST_ASSERT_EQUAL_INT(0, cfg_key_index(CFG_ID_NET_DHCP));

	/* lower_bound lands on the first key at or past an unused ID. */
	TEST_ASSERT_EQUAL_UINT16(CFG_ID_NTP_ENABLE,
		cfg_key_at((size_t)cfg_key_lower_bound(0x01FF))->id);
	TEST_ASSERT_EQUAL_INT(0, cfg_key_lower_bound(0x0000));
	TEST_ASSERT_EQUAL_INT(-ENOENT, cfg_key_lower_bound(0xFFFF));
}

static void test_type_widths(void)
{
	TEST_ASSERT_EQUAL_size_t(1U, cfg_type_width(CFG_T_BOOL));
	TEST_ASSERT_EQUAL_size_t(1U, cfg_type_width(CFG_T_U8));
	TEST_ASSERT_EQUAL_size_t(2U, cfg_type_width(CFG_T_U16));
	TEST_ASSERT_EQUAL_size_t(4U, cfg_type_width(CFG_T_U32));
	TEST_ASSERT_EQUAL_size_t(8U, cfg_type_width(CFG_T_U64));
	TEST_ASSERT_EQUAL_size_t(4U, cfg_type_width(CFG_T_I32));
	TEST_ASSERT_EQUAL_size_t(4U, cfg_type_width(CFG_T_F32));
	TEST_ASSERT_EQUAL_size_t(0U, cfg_type_width(CFG_T_STR));
	TEST_ASSERT_EQUAL_size_t(0U, cfg_type_width(CFG_T_BLOB));
	TEST_ASSERT_EQUAL_size_t(0U, cfg_type_width(200U));
}

/* ------------------------------------------------------------------------- */
/* Value codec                                                               */
/* ------------------------------------------------------------------------- */

static void test_val_codec_round_trip(void)
{
	static const uint8_t u64_le[8] = { 0x01, 0x23, 0x45, 0x67,
					   0x89, 0xAB, 0xCD, 0xEF };
	uint8_t out[CFG_VAL_MAX];
	cfg_val_t v;

	TEST_ASSERT_EQUAL_INT(0,
		cfg_val_decode(&v, CFG_T_U64, u64_le, sizeof(u64_le)));
	TEST_ASSERT_EQUAL_HEX64(0xEFCDAB8967452301ULL, v.v.u);
	TEST_ASSERT_EQUAL_INT(8, cfg_val_encode(&v, out, sizeof(out)));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(u64_le, out, 8);

	/* I32 sign extension: 0xFFFFFF9C little-endian is -100. */
	{
		static const uint8_t i32_le[4] = { 0x9C, 0xFF, 0xFF, 0xFF };

		TEST_ASSERT_EQUAL_INT(0,
			cfg_val_decode(&v, CFG_T_I32, i32_le, 4));
		TEST_ASSERT_EQUAL_INT32(-100, v.v.i);
		TEST_ASSERT_EQUAL_INT(4, cfg_val_encode(&v, out, sizeof(out)));
		TEST_ASSERT_EQUAL_HEX8_ARRAY(i32_le, out, 4);
	}

	/* F32: 0x3F800000 is 1.0f, little-endian on the wire. */
	{
		static const uint8_t f32_le[4] = { 0x00, 0x00, 0x80, 0x3F };

		TEST_ASSERT_EQUAL_INT(0,
			cfg_val_decode(&v, CFG_T_F32, f32_le, 4));
		TEST_ASSERT_EQUAL_FLOAT(1.0f, v.v.f);
		TEST_ASSERT_EQUAL_INT(4, cfg_val_encode(&v, out, sizeof(out)));
		TEST_ASSERT_EQUAL_HEX8_ARRAY(f32_le, out, 4);
	}

	/* BOOL / U8 / U16 / U32 widths. */
	TEST_ASSERT_EQUAL_INT(0, cfg_val_decode(&v, CFG_T_BOOL, out, 1));
	TEST_ASSERT_EQUAL_INT(1, cfg_val_encode(&v, out, sizeof(out)));
	TEST_ASSERT_EQUAL_INT(0, cfg_val_decode(&v, CFG_T_U8, out, 1));
	TEST_ASSERT_EQUAL_INT(1, cfg_val_encode(&v, out, sizeof(out)));
	TEST_ASSERT_EQUAL_INT(0, cfg_val_decode(&v, CFG_T_U16, out, 2));
	TEST_ASSERT_EQUAL_INT(2, cfg_val_encode(&v, out, sizeof(out)));
	TEST_ASSERT_EQUAL_INT(0, cfg_val_decode(&v, CFG_T_U32, out, 4));
	TEST_ASSERT_EQUAL_INT(4, cfg_val_encode(&v, out, sizeof(out)));

	/* STR is length-delimited and never NUL-terminated on the wire. */
	TEST_ASSERT_EQUAL_INT(0, cfg_val_decode(&v, CFG_T_STR,
						(const uint8_t *)"host", 4));
	TEST_ASSERT_EQUAL_UINT16(4U, v.len);
	TEST_ASSERT_EQUAL_INT(4, cfg_val_encode(&v, out, sizeof(out)));
	TEST_ASSERT_EQUAL_HEX8_ARRAY("host", out, 4);
}

static void test_val_codec_rejects(void)
{
	uint8_t out[CFG_VAL_MAX];
	uint8_t big[CFG_VAL_MAX + 1U];
	cfg_val_t v;

	memset(out, 0, sizeof(out));
	memset(big, 'x', sizeof(big));

	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_val_decode(NULL, CFG_T_U8, out, 1));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_val_decode(&v, CFG_T_U8, NULL, 1));
	TEST_ASSERT_EQUAL_INT(-EPROTO, cfg_val_decode(&v, CFG_T_U16, out, 1));
	TEST_ASSERT_EQUAL_INT(-EPROTO, cfg_val_decode(&v, 99U, out, 1));
	TEST_ASSERT_EQUAL_INT(-ERANGE,
		cfg_val_decode(&v, CFG_T_BLOB, big, sizeof(big)));

	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_val_encode(NULL, out, sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_val_encode(&v, NULL, 4));

	v.type = 99U;
	TEST_ASSERT_EQUAL_INT(-EPROTO, cfg_val_encode(&v, out, sizeof(out)));

	v.type = CFG_T_U32;
	TEST_ASSERT_EQUAL_INT(-ENOSPC, cfg_val_encode(&v, out, 3));

	v.type = CFG_T_BLOB;
	v.len = 8U;
	TEST_ASSERT_EQUAL_INT(-ENOSPC, cfg_val_encode(&v, out, 4));
	v.len = CFG_VAL_MAX + 1U;
	TEST_ASSERT_EQUAL_INT(-EPROTO, cfg_val_encode(&v, out, sizeof(out)));
}

/* ------------------------------------------------------------------------- */
/* Bounds and type enforcement                                               */
/* ------------------------------------------------------------------------- */

static void test_set_enforces_type(void)
{
	cfg_val_t v;

	memset(&v, 0, sizeof(v));
	v.type = CFG_T_U32; /* ptp.domain is U8 */
	v.v.u = 5U;
	TEST_ASSERT_EQUAL_INT(-EPROTO, cfg_set(&g_cfg, CFG_ID_PTP_DOMAIN, &v));
	TEST_ASSERT_EQUAL_UINT16(0U, cfg_staged_count(&g_cfg));

	TEST_ASSERT_EQUAL_INT(-ENOENT, cfg_set_u64(&g_cfg, 0xFFFF, 1U));
	TEST_ASSERT_EQUAL_INT(-EPROTO,
		cfg_set_u64(&g_cfg, CFG_ID_NET_HOSTNAME, 1U));
	TEST_ASSERT_EQUAL_INT(-EPROTO,
		cfg_set_u64(&g_cfg, CFG_ID_CAL_PPS_OFFSET_NS, 1U));
	TEST_ASSERT_EQUAL_INT(-EPROTO,
		cfg_set_bytes(&g_cfg, CFG_ID_PTP_DOMAIN,
			      (const uint8_t *)"x", 1U));
	TEST_ASSERT_EQUAL_INT(-ENOENT,
		cfg_set_bytes(&g_cfg, 0xFFFF, (const uint8_t *)"x", 1U));
}

static void test_set_enforces_bounds(void)
{
	uint8_t big[CFG_VAL_MAX];

	memset(big, 'h', sizeof(big));

	/* tim.tau.s is 10..1000. */
	TEST_ASSERT_EQUAL_INT(-ERANGE, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 9U));
	TEST_ASSERT_EQUAL_INT(-ERANGE,
		cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 1001U));
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 10U));
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 1000U));

	/* ptp.log.sync is -7..1. */
	TEST_ASSERT_EQUAL_INT(-ERANGE,
		cfg_set_i32(&g_cfg, CFG_ID_PTP_LOG_SYNC, -8));
	TEST_ASSERT_EQUAL_INT(-ERANGE,
		cfg_set_i32(&g_cfg, CFG_ID_PTP_LOG_SYNC, 2));
	TEST_ASSERT_EQUAL_INT(0, cfg_set_i32(&g_cfg, CFG_ID_PTP_LOG_SYNC, -7));

	/* cal.tempco is -100..100, and a NaN must not slip through the pair of
	 * ordered comparisons. */
	TEST_ASSERT_EQUAL_INT(-ERANGE,
		cfg_set_f32(&g_cfg, CFG_ID_CAL_TEMPCO_PPB_C, 100.5f));
	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_f32(&g_cfg, CFG_ID_CAL_TEMPCO_PPB_C, -12.5f));
	{
		float zero = 0.0f;
		float nan = zero / zero;

		TEST_ASSERT_EQUAL_INT(-ERANGE,
			cfg_set_f32(&g_cfg, CFG_ID_CAL_TEMPCO_PPB_C, nan));
	}

	/* net.hostname holds 63 bytes, not 64. */
	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_bytes(&g_cfg, CFG_ID_NET_HOSTNAME, big, 63U));
	TEST_ASSERT_EQUAL_INT(-ERANGE,
		cfg_set_bytes(&g_cfg, CFG_ID_NET_HOSTNAME, big, 64U));
	TEST_ASSERT_EQUAL_INT(-ERANGE,
		cfg_set_bytes(&g_cfg, CFG_ID_NET_HOSTNAME, big,
			      CFG_VAL_MAX + 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		cfg_set_bytes(&g_cfg, CFG_ID_NET_HOSTNAME, NULL, 4U));
}

static void test_typed_accessors(void)
{
	uint8_t buf[CFG_VAL_MAX];
	size_t n = 0U;
	uint64_t u = 0U;
	int32_t i = 0;
	float f = 0.0f;
	bool b = false;

	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(200U, u);
	TEST_ASSERT_EQUAL_INT(0, cfg_get_bool(&g_cfg, CFG_ID_NTP_ENABLE, &b));
	TEST_ASSERT_TRUE(b);
	TEST_ASSERT_EQUAL_INT(0,
		cfg_get_i32(&g_cfg, CFG_ID_PTP_LOG_ANNOUNCE, &i));
	TEST_ASSERT_EQUAL_INT32(1, i);
	TEST_ASSERT_EQUAL_INT(0,
		cfg_get_f32(&g_cfg, CFG_ID_CAL_TEMPCO_PPB_C, &f));
	TEST_ASSERT_EQUAL_FLOAT(0.0f, f);
	TEST_ASSERT_EQUAL_INT(0,
		cfg_get_bytes(&g_cfg, CFG_ID_NET_HOSTNAME, buf, sizeof(buf), &n));
	TEST_ASSERT_EQUAL_size_t(8U, n);
	TEST_ASSERT_EQUAL_HEX8_ARRAY("meridian", buf, 8);

	/* Wrong-shaped accessors must refuse rather than reinterpret. */
	TEST_ASSERT_EQUAL_INT(-EPROTO,
		cfg_get_u64(&g_cfg, CFG_ID_CAL_TEMPCO_PPB_C, &u));
	TEST_ASSERT_EQUAL_INT(-EPROTO, cfg_get_i32(&g_cfg, CFG_ID_TIM_TAU_S, &i));
	TEST_ASSERT_EQUAL_INT(-EPROTO, cfg_get_f32(&g_cfg, CFG_ID_TIM_TAU_S, &f));
	TEST_ASSERT_EQUAL_INT(-EPROTO,
		cfg_get_bytes(&g_cfg, CFG_ID_TIM_TAU_S, buf, sizeof(buf), &n));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
		cfg_get_bytes(&g_cfg, CFG_ID_NET_HOSTNAME, buf, 2U, &n));
	TEST_ASSERT_EQUAL_INT(-ENOENT, cfg_get_u64(&g_cfg, 0xFFFF, &u));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_get_bool(&g_cfg, CFG_ID_NTP_ENABLE, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_get_i32(&g_cfg, CFG_ID_PTP_LOG_SYNC, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		cfg_get_f32(&g_cfg, CFG_ID_CAL_TEMPCO_PPB_C, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		cfg_get_bytes(&g_cfg, CFG_ID_NET_HOSTNAME, NULL, 4U, &n));
}

static void test_null_arguments(void)
{
	cfg_val_t v;

	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_init(NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_load_all(NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_set_validate_hook(NULL, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_set_migrations(NULL, NULL, 0U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_set_migrations(&g_cfg, NULL, 3U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_factory_reset(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_get(NULL, 0, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_get(&g_cfg, 0, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_get_effective(NULL, 0, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_get_effective(&g_cfg, 0, NULL));
	TEST_ASSERT_EQUAL_INT(-ENOENT, cfg_get_effective(&g_cfg, 0xFFFF, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_set(NULL, 0, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_set(&g_cfg, 0, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_revert(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_commit(NULL, NULL));
	TEST_ASSERT_FALSE(cfg_is_staged(NULL, 0));
	TEST_ASSERT_FALSE(cfg_is_staged(&g_cfg, 0xFFFF));
	TEST_ASSERT_EQUAL_UINT16(0U, cfg_staged_count(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_export_begin(NULL, NULL, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_import_begin(NULL, NULL, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_import_finish(NULL, NULL, NULL));
}

/* ------------------------------------------------------------------------- */
/* Staging and commit                                                        */
/* ------------------------------------------------------------------------- */

static void test_staging_is_invisible_until_commit(void)
{
	cfg_val_t v;
	uint64_t u = 0U;

	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 500U));
	TEST_ASSERT_TRUE(cfg_is_staged(&g_cfg, CFG_ID_TIM_TAU_S));
	TEST_ASSERT_EQUAL_UINT16(1U, cfg_staged_count(&g_cfg));

	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(200U, u); /* live still the default */

	TEST_ASSERT_EQUAL_INT(0,
		cfg_get_effective(&g_cfg, CFG_ID_TIM_TAU_S, &v));
	TEST_ASSERT_EQUAL_UINT64(500U, v.v.u);

	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(500U, u);
	TEST_ASSERT_EQUAL_UINT16(0U, cfg_staged_count(&g_cfg));
}

static void test_revert_drops_staging(void)
{
	uint64_t u = 0U;

	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 500U));
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_UI_BRIGHTNESS, 5U));
	TEST_ASSERT_EQUAL_UINT16(2U, cfg_staged_count(&g_cfg));

	TEST_ASSERT_EQUAL_INT(0, cfg_revert(&g_cfg));
	TEST_ASSERT_EQUAL_UINT16(0U, cfg_staged_count(&g_cfg));
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(200U, u);
	TEST_ASSERT_EQUAL_UINT32(0U, g_store.saves);
}

static void test_commit_reports_reboot_scope(void)
{
	cfg_commit_res_t res;

	/* One runtime-apply key and two reboot keys in two different groups. */
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_UI_BRIGHTNESS, 5U));
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_NET_DHCP, 0U));
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_PTP_TRANSPORT, 0U));

	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, &res));
	TEST_ASSERT_EQUAL_UINT16(3U, res.staged);
	TEST_ASSERT_EQUAL_UINT16(3U, res.applied);
	TEST_ASSERT_EQUAL_UINT16(2U, res.reboot_keys);
	TEST_ASSERT_EQUAL_HEX32(CFG_GROUP_BIT(CFG_G_NET) |
					CFG_GROUP_BIT(CFG_G_PTP),
				res.reboot_groups);
	TEST_ASSERT_EQUAL_UINT16(0U, res.persist_errors);
	TEST_ASSERT_EQUAL_UINT32(3U, g_store.saves);
}

static void test_commit_skips_unchanged_values(void)
{
	cfg_commit_res_t res;

	/* Staging the value that is already live is legal and must not burn a
	 * flash write. */
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 200U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, &res));
	TEST_ASSERT_EQUAL_UINT16(1U, res.staged);
	TEST_ASSERT_EQUAL_UINT16(0U, res.applied);
	TEST_ASSERT_EQUAL_UINT32(0U, g_store.saves);
}

/* A cross-field rule: a static address must actually be configured. */
static int hook_static_needs_address(const cfg_ctx_t *c, void *user)
{
	cfg_val_t dhcp;
	cfg_val_t addr;

	(*(int *)user)++;

	if ((cfg_get_effective(c, CFG_ID_NET_DHCP, &dhcp) != 0) ||
	    (cfg_get_effective(c, CFG_ID_NET_IPV4_ADDR, &addr) != 0)) {
		return -EIO;
	}
	if ((dhcp.v.u == 0U) && (addr.v.u == 0U)) {
		return -EPROTO;
	}
	return 0;
}

static int hook_positive_error(const cfg_ctx_t *c, void *user)
{
	(void)c;
	(void)user;
	return 1; /* a hook that forgets the sign convention */
}

static void test_commit_is_all_or_nothing(void)
{
	cfg_commit_res_t res;
	int calls = 0;
	uint64_t u = 0U;
	bool b = true;

	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_validate_hook(&g_cfg, hook_static_needs_address,
				      &calls));

	/* Turning DHCP off without an address must fail, and the *other*
	 * staged key must not survive the failed transaction. */
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_NET_DHCP, 0U));
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_UI_BRIGHTNESS, 5U));

	TEST_ASSERT_EQUAL_INT(-EPROTO, cfg_commit(&g_cfg, &res));
	TEST_ASSERT_EQUAL_INT(1, calls);
	TEST_ASSERT_EQUAL_UINT16(0U, res.applied);
	TEST_ASSERT_EQUAL_UINT32(0U, g_store.saves);

	TEST_ASSERT_EQUAL_INT(0, cfg_get_bool(&g_cfg, CFG_ID_NET_DHCP, &b));
	TEST_ASSERT_TRUE(b);
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_UI_BRIGHTNESS, &u));
	TEST_ASSERT_EQUAL_UINT64(70U, u);

	/* Staging is intact so the operator can fix the offending field. */
	TEST_ASSERT_EQUAL_UINT16(2U, cfg_staged_count(&g_cfg));
	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_u64(&g_cfg, CFG_ID_NET_IPV4_ADDR, 0xC0A80105U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, &res));
	TEST_ASSERT_EQUAL_UINT16(3U, res.applied);
	TEST_ASSERT_EQUAL_INT(2, calls);

	/* Clearing the hook restores plain commits. */
	TEST_ASSERT_EQUAL_INT(0, cfg_set_validate_hook(&g_cfg, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_NET_IPV4_ADDR, 0U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, &res));
}

static void test_commit_normalises_a_positive_hook_error(void)
{
	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_validate_hook(&g_cfg, hook_positive_error, NULL));
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_UI_BRIGHTNESS, 5U));
	TEST_ASSERT_EQUAL_INT(-EPROTO, cfg_commit(&g_cfg, NULL));
}

static void test_commit_reports_persist_failure_but_keeps_ram(void)
{
	cfg_commit_res_t res;
	uint64_t u = 0U;

	g_store.fail_save_id = (int)CFG_ID_UI_BRIGHTNESS;

	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_UI_BRIGHTNESS, 5U));
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_UI_TIMEOUT_S, 30U));

	TEST_ASSERT_EQUAL_INT(-EIO, cfg_commit(&g_cfg, &res));
	TEST_ASSERT_EQUAL_UINT16(2U, res.applied);
	TEST_ASSERT_EQUAL_UINT16(1U, res.persist_errors);

	/* RAM is authoritative until the next boot — reporting "not applied"
	 * would be a lie the operator could not act on. */
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_UI_BRIGHTNESS, &u));
	TEST_ASSERT_EQUAL_UINT64(5U, u);
}

static void test_ram_only_registry_needs_no_store(void)
{
	cfg_ctx_t *c = &g_cfg;
	uint32_t corrupt = 1U;
	uint64_t u = 0U;

	TEST_ASSERT_EQUAL_INT(0, cfg_init(c, NULL));
	TEST_ASSERT_EQUAL_INT(0, cfg_load_all(c, &corrupt));
	TEST_ASSERT_EQUAL_UINT32(0U, corrupt);
	TEST_ASSERT_EQUAL_UINT32((uint32_t)cfg_key_count(), c->load_missing);

	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(c, CFG_ID_TIM_TAU_S, 42U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(c, NULL));
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(c, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(42U, u);
	TEST_ASSERT_EQUAL_INT(0, cfg_factory_reset(c));
}

/* ------------------------------------------------------------------------- */
/* Load / factory reset                                                      */
/* ------------------------------------------------------------------------- */

static void test_load_round_trips_through_the_store(void)
{
	uint32_t corrupt = 99U;
	uint64_t u = 0U;
	size_t n = 0U;
	uint8_t buf[CFG_VAL_MAX];
	float f = 0.0f;

	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 777U));
	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_bytes(&g_cfg, CFG_ID_NET_HOSTNAME,
			      (const uint8_t *)"obs-gm-1", 8U));
	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_f32(&g_cfg, CFG_ID_CAL_TEMPCO_PPB_C, -3.25f));
	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_i32(&g_cfg, CFG_ID_CAL_PPS_OFFSET_NS, -42));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));

	/* A fresh context over the same store must come back identical. */
	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, &g_store_port));
	TEST_ASSERT_EQUAL_INT(0, cfg_load_all(&g_cfg, &corrupt));
	TEST_ASSERT_EQUAL_UINT32(0U, corrupt);

	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(777U, u);
	TEST_ASSERT_EQUAL_INT(0,
		cfg_get_bytes(&g_cfg, CFG_ID_NET_HOSTNAME, buf, sizeof(buf), &n));
	TEST_ASSERT_EQUAL_size_t(8U, n);
	TEST_ASSERT_EQUAL_HEX8_ARRAY("obs-gm-1", buf, 8);
	TEST_ASSERT_EQUAL_INT(0,
		cfg_get_f32(&g_cfg, CFG_ID_CAL_TEMPCO_PPB_C, &f));
	TEST_ASSERT_EQUAL_FLOAT(-3.25f, f);
}

static void test_load_defaults_corrupt_entries(void)
{
	uint32_t corrupt = 0U;
	uint64_t u = 0U;

	/* Five distinct ways a stored record can be wrong. */
	{
		uint8_t wrong_type[3] = { CFG_T_U32, 0x10, 0x00 };
		uint8_t short_rec[2] = { CFG_T_U16, 0x10 };
		uint8_t empty[1] = { 0 };
		uint8_t out_of_range[3];

		store_put_raw(CFG_ID_TIM_TAU_S, wrong_type, sizeof(wrong_type));
		store_put_raw(CFG_ID_TIM_LOCK_HOLD_S, short_rec,
			      sizeof(short_rec));
		store_put_raw(CFG_ID_UI_TIMEOUT_S, empty, 0U);

		out_of_range[0] = CFG_T_U16;
		out_of_range[1] = 0xFF; /* 65535, above the 1000 s maximum */
		out_of_range[2] = 0xFF;
		store_put_raw(CFG_ID_TIM_HYSTERESIS_S, out_of_range,
			      sizeof(out_of_range));
	}
	g_store.fail_load_id = (int)CFG_ID_UI_BRIGHTNESS;
	g_store.load_ret = -5;

	TEST_ASSERT_EQUAL_INT(0, cfg_load_all(&g_cfg, &corrupt));
	TEST_ASSERT_EQUAL_UINT32(5U, corrupt);
	TEST_ASSERT_EQUAL_UINT32(5U, g_cfg.load_corrupt);
	TEST_ASSERT_EQUAL_UINT32((uint32_t)cfg_key_count() - 5U,
				 g_cfg.load_missing);

	/* Every one of them fell back to the schema default, and the box boots. */
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(200U, u);
	TEST_ASSERT_EQUAL_INT(0,
		cfg_get_u64(&g_cfg, CFG_ID_TIM_HYSTERESIS_S, &u));
	TEST_ASSERT_EQUAL_UINT64(60U, u);
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_UI_BRIGHTNESS, &u));
	TEST_ASSERT_EQUAL_UINT64(70U, u);
}

static void test_load_rejects_an_overlong_store_record(void)
{
	uint32_t corrupt = 0U;

	/* A store that reports having written more than it was handed. */
	g_store.fail_load_id = (int)CFG_ID_TIM_TAU_S;
	g_store.load_ret = 4096;

	TEST_ASSERT_EQUAL_INT(0, cfg_load_all(&g_cfg, &corrupt));
	TEST_ASSERT_EQUAL_UINT32(1U, corrupt);
}

static void test_factory_reset_clears_ram_and_store(void)
{
	uint64_t u = 0U;

	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 999U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));
	TEST_ASSERT_EQUAL_UINT32(1U, g_store.saves);

	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_UI_BRIGHTNESS, 1U));
	TEST_ASSERT_EQUAL_INT(0, cfg_factory_reset(&g_cfg));

	TEST_ASSERT_EQUAL_UINT16(0U, cfg_staged_count(&g_cfg));
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(200U, u);
	TEST_ASSERT_EQUAL_UINT32(1U, g_store.erases); /* only the stored key */

	/* Nothing survives into a fresh context. */
	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, &g_store_port));
	TEST_ASSERT_EQUAL_INT(0, cfg_load_all(&g_cfg, NULL));
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(200U, u);
}

static void test_factory_reset_reports_an_erase_failure(void)
{
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 999U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));

	g_store.fail_erase_id = (int)CFG_ID_TIM_TAU_S;
	TEST_ASSERT_EQUAL_INT(-EIO, cfg_factory_reset(&g_cfg));
}

/* ------------------------------------------------------------------------- */
/* TLV export / import                                                       */
/* ------------------------------------------------------------------------- */

#define EXPORT_BUF 8192U

static uint8_t g_blob[EXPORT_BUF];

/* Locate a record for @p id inside a TLV stream. Returns its offset or -1. */
static long tlv_find(const uint8_t *b, size_t n, uint16_t id)
{
	size_t o = CFG_EXPORT_HDR_LEN;

	while ((o + CFG_EXPORT_REC_HDR) <= (n - 4U)) {
		uint16_t rid = (uint16_t)(b[o] | ((uint16_t)b[o + 1U] << 8));
		uint16_t vlen =
			(uint16_t)(b[o + 3U] | ((uint16_t)b[o + 4U] << 8));

		if (rid == id) {
			return (long)o;
		}
		o += CFG_EXPORT_REC_HDR + vlen;
	}
	return -1;
}

static void test_export_header_and_trailer_are_as_documented(void)
{
	size_t n = 0U;
	uint16_t count;

	TEST_ASSERT_EQUAL_INT(0,
		cfg_export_all(&g_cfg, g_blob, sizeof(g_blob), false, &n));

	TEST_ASSERT_TRUE(n > CFG_EXPORT_HDR_LEN + 4U);
	TEST_ASSERT_EQUAL_HEX8_ARRAY("MCF1", g_blob, 4);
	TEST_ASSERT_EQUAL_UINT16(CFG_SCHEMA_VERSION,
				 (uint16_t)(g_blob[4] |
					    ((uint16_t)g_blob[5] << 8)));

	count = (uint16_t)(g_blob[6] | ((uint16_t)g_blob[7] << 8));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)cfg_key_count() - 2U, count);

	/* The trailer is a CRC-32/ISO-HDLC over every preceding byte. */
	TEST_ASSERT_EQUAL_HEX32(sts_crc32_ieee(g_blob, n - 4U),
				(uint32_t)(g_blob[n - 4U] |
					   ((uint32_t)g_blob[n - 3U] << 8) |
					   ((uint32_t)g_blob[n - 2U] << 16) |
					   ((uint32_t)g_blob[n - 1U] << 24)));
}

static void test_export_omits_secrets_unless_asked(void)
{
	size_t plain = 0U;
	size_t withs = 0U;

	/* A plain export carries neither secret. */
	TEST_ASSERT_EQUAL_INT(0,
		cfg_export_all(&g_cfg, g_blob, sizeof(g_blob), false, &plain));
	TEST_ASSERT_EQUAL_INT(-1, tlv_find(g_blob, plain, CFG_ID_SEC_ADMIN_PW));
	TEST_ASSERT_EQUAL_INT(-1,
		tlv_find(g_blob, plain, CFG_ID_SNMP_COMMUNITY));
	TEST_ASSERT_TRUE(tlv_find(g_blob, plain, CFG_ID_TIM_TAU_S) > 0);

	/* A secrets export carries the SNMP community, but the admin credential
	 * is CFG_F_NOEXPORT and never appears — not merely gated behind the
	 * flag. It is provisioned out-of-band and must never reach the wire. */
	TEST_ASSERT_EQUAL_INT(0,
		cfg_export_all(&g_cfg, g_blob, sizeof(g_blob), true, &withs));
	TEST_ASSERT_EQUAL_INT(-1, tlv_find(g_blob, withs, CFG_ID_SEC_ADMIN_PW));
	TEST_ASSERT_TRUE(tlv_find(g_blob, withs, CFG_ID_SNMP_COMMUNITY) > 0);
	TEST_ASSERT_TRUE(withs > plain);
}

static void test_noexport_key_never_leaves_even_with_secrets(void)
{
	const cfg_key_t *k = cfg_key_find(CFG_ID_SEC_ADMIN_PW);
	size_t withs = 0U;
	uint16_t hdr_count;
	uint16_t noexport_count = 0U;
	size_t i;

	/* The schema marks the credential NOEXPORT (implying SECRET). */
	TEST_ASSERT_NOT_NULL(k);
	TEST_ASSERT_TRUE((k->flags & CFG_F_NOEXPORT) != 0U);
	TEST_ASSERT_TRUE((k->flags & CFG_F_SECRET) != 0U);

	/* Provision a value, then confirm a secrets export still omits it and
	 * that its record count is exactly (keys - NOEXPORT count). */
	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_bytes(&g_cfg, CFG_ID_SEC_ADMIN_PW,
			      (const uint8_t *)"0123456789abcdef", 16U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));

	TEST_ASSERT_EQUAL_INT(0,
		cfg_export_all(&g_cfg, g_blob, sizeof(g_blob), true, &withs));
	TEST_ASSERT_EQUAL_INT(-1, tlv_find(g_blob, withs, CFG_ID_SEC_ADMIN_PW));

	for (i = 0U; i < cfg_key_count(); i++) {
		if ((cfg_key_at(i)->flags & CFG_F_NOEXPORT) != 0U) {
			noexport_count++;
		}
	}
	hdr_count = (uint16_t)(g_blob[6] | ((uint16_t)g_blob[7] << 8));
	TEST_ASSERT_EQUAL_UINT16((uint16_t)cfg_key_count() - noexport_count,
				 hdr_count);
}

static void test_export_chunks_at_the_minimum_size(void)
{
	uint8_t chunked[EXPORT_BUF];
	cfg_export_t ex;
	size_t total = 0U;
	size_t one = 0U;
	int rc;
	unsigned int rounds = 0U;

	TEST_ASSERT_EQUAL_INT(0,
		cfg_export_all(&g_cfg, g_blob, sizeof(g_blob), false, &one));

	/* Drive it at the smallest legal chunk so the record-splitting guard
	 * gets exercised on every boundary. */
	TEST_ASSERT_EQUAL_INT(0, cfg_export_begin(&g_cfg, &ex, false));
	do {
		size_t n = 0U;

		rc = cfg_export_read(&g_cfg, &ex, &chunked[total],
				     CFG_EXPORT_MIN_CHUNK, &n);
		TEST_ASSERT_GREATER_OR_EQUAL_INT(0, rc);
		TEST_ASSERT_TRUE(n <= CFG_EXPORT_MIN_CHUNK);
		total += n;
		rounds++;
		TEST_ASSERT_TRUE(rounds < 1000U);
	} while (rc == 0);

	TEST_ASSERT_TRUE(rounds > 5U); /* it really did chunk */
	TEST_ASSERT_EQUAL_size_t(one, total);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_blob, chunked, total);
}

static void test_export_argument_errors(void)
{
	cfg_export_t ex;
	size_t n = 0U;

	TEST_ASSERT_EQUAL_INT(0, cfg_export_begin(&g_cfg, &ex, false));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
		cfg_export_read(&g_cfg, &ex, g_blob,
				CFG_EXPORT_MIN_CHUNK - 1U, &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		cfg_export_read(NULL, &ex, g_blob, sizeof(g_blob), &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cfg_export_begin(&g_cfg, NULL, false));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		cfg_export_all(&g_cfg, NULL, 0U, false, &n));
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
		cfg_export_all(&g_cfg, g_blob, 128U, false, &n));
}

static void test_import_round_trip_restores_every_key(void)
{
	cfg_commit_res_t res;
	size_t n = 0U;
	uint64_t u = 0U;
	uint8_t buf[CFG_VAL_MAX];
	size_t blen = 0U;
	float f = 0.0f;

	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 321U));
	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_bytes(&g_cfg, CFG_ID_LOG_SYSLOG_HOST,
			      (const uint8_t *)"10.0.0.9", 8U));
	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_f32(&g_cfg, CFG_ID_CAL_TEMPCO_PPB_C, 7.5f));
	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_NTS_KE_PORT, 4461U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));

	TEST_ASSERT_EQUAL_INT(0,
		cfg_export_all(&g_cfg, g_blob, sizeof(g_blob), false, &n));

	/* Wipe, then restore from the blob. */
	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, &g_store_port));
	TEST_ASSERT_EQUAL_INT(0,
		cfg_import_all(&g_cfg, g_blob, n, true, &res));

	TEST_ASSERT_EQUAL_UINT16((uint16_t)cfg_key_count() - 2U, res.staged);
	TEST_ASSERT_EQUAL_UINT16(4U, res.applied);
	TEST_ASSERT_EQUAL_HEX32(CFG_GROUP_BIT(CFG_G_NTS), res.reboot_groups);

	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(321U, u);
	TEST_ASSERT_EQUAL_INT(0,
		cfg_get_bytes(&g_cfg, CFG_ID_LOG_SYSLOG_HOST, buf, sizeof(buf),
			      &blen));
	TEST_ASSERT_EQUAL_size_t(8U, blen);
	TEST_ASSERT_EQUAL_HEX8_ARRAY("10.0.0.9", buf, 8);
	TEST_ASSERT_EQUAL_INT(0,
		cfg_get_f32(&g_cfg, CFG_ID_CAL_TEMPCO_PPB_C, &f));
	TEST_ASSERT_EQUAL_FLOAT(7.5f, f);
}

static void test_import_detects_a_corrupt_crc(void)
{
	cfg_commit_res_t res;
	size_t n = 0U;
	uint64_t u = 0U;

	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 321U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));
	TEST_ASSERT_EQUAL_INT(0,
		cfg_export_all(&g_cfg, g_blob, sizeof(g_blob), false, &n));

	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, &g_store_port));

	/* Flip a value byte; the trailer no longer matches. */
	{
		long o = tlv_find(g_blob, n, CFG_ID_TIM_TAU_S);

		TEST_ASSERT_TRUE(o > 0);
		g_blob[(size_t)o + CFG_EXPORT_REC_HDR] ^= 0x01U;
	}

	TEST_ASSERT_EQUAL_INT(-EILSEQ, cfg_import_all(&g_cfg, g_blob, n, true, &res));
	TEST_ASSERT_EQUAL_UINT16(0U, cfg_staged_count(&g_cfg));
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(200U, u);
}

/* Build a minimal well-formed TLV stream with the records the caller lists. */
typedef struct {
	uint16_t id;
	uint8_t  type;
	uint16_t len;
	uint8_t  val[8];
} tlv_rec_t;

static size_t tlv_build(uint8_t *b, uint16_t ver, const tlv_rec_t *recs,
			uint16_t n, bool good_crc)
{
	size_t o = 0U;
	uint16_t i;
	uint32_t crc;

	b[o++] = (uint8_t)'M';
	b[o++] = (uint8_t)'C';
	b[o++] = (uint8_t)'F';
	b[o++] = (uint8_t)'1';
	b[o++] = (uint8_t)(ver & 0xFFU);
	b[o++] = (uint8_t)(ver >> 8);
	b[o++] = (uint8_t)(n & 0xFFU);
	b[o++] = (uint8_t)(n >> 8);

	for (i = 0U; i < n; i++) {
		b[o++] = (uint8_t)(recs[i].id & 0xFFU);
		b[o++] = (uint8_t)(recs[i].id >> 8);
		b[o++] = recs[i].type;
		b[o++] = (uint8_t)(recs[i].len & 0xFFU);
		b[o++] = (uint8_t)(recs[i].len >> 8);
		memcpy(&b[o], recs[i].val, recs[i].len);
		o += recs[i].len;
	}

	crc = sts_crc32_ieee(b, o);
	if (!good_crc) {
		crc ^= 0xFFFFFFFFU;
	}
	b[o++] = (uint8_t)(crc & 0xFFU);
	b[o++] = (uint8_t)((crc >> 8) & 0xFFU);
	b[o++] = (uint8_t)((crc >> 16) & 0xFFU);
	b[o++] = (uint8_t)((crc >> 24) & 0xFFU);
	return o;
}

static void test_import_unknown_id_strict_versus_lenient(void)
{
	tlv_rec_t recs[2] = {
		{ CFG_ID_TIM_TAU_S, CFG_T_U16, 2U, { 0x2C, 0x01, 0, 0, 0, 0, 0, 0 } },
		{ 0xFEED, CFG_T_U32, 4U, { 1, 2, 3, 4, 0, 0, 0, 0 } },
	};
	cfg_commit_res_t res;
	size_t n;
	uint64_t u = 0U;

	n = tlv_build(g_blob, CFG_SCHEMA_VERSION, recs, 2U, true);

	TEST_ASSERT_EQUAL_INT(-ENOENT, cfg_import_all(&g_cfg, g_blob, n, true, &res));
	TEST_ASSERT_EQUAL_UINT16(0U, cfg_staged_count(&g_cfg));

	TEST_ASSERT_EQUAL_INT(0, cfg_import_all(&g_cfg, g_blob, n, false, &res));
	TEST_ASSERT_EQUAL_UINT16(1U, res.applied);
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(300U, u); /* 0x012C */
}

static void test_import_rejects_malformed_streams(void)
{
	tlv_rec_t ok = { CFG_ID_TIM_TAU_S, CFG_T_U16, 2U,
			 { 0x2C, 0x01, 0, 0, 0, 0, 0, 0 } };
	cfg_commit_res_t res;
	size_t n;

	/* Bad magic. */
	n = tlv_build(g_blob, CFG_SCHEMA_VERSION, &ok, 1U, true);
	g_blob[1] = (uint8_t)'X';
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
		cfg_import_all(&g_cfg, g_blob, n, true, &res));

	/* Unknown schema version with no migration registered. */
	n = tlv_build(g_blob, 42U, &ok, 1U, true);
	TEST_ASSERT_EQUAL_INT(-ENOTSUP,
		cfg_import_all(&g_cfg, g_blob, n, true, &res));

	/* Absurd record count. */
	n = tlv_build(g_blob, CFG_SCHEMA_VERSION, &ok, 1U, true);
	g_blob[6] = 0xFF;
	g_blob[7] = 0xFF;
	TEST_ASSERT_EQUAL_INT(-EPROTO,
		cfg_import_all(&g_cfg, g_blob, n, true, &res));

	/* A value length no key could hold. */
	n = tlv_build(g_blob, CFG_SCHEMA_VERSION, &ok, 1U, true);
	g_blob[CFG_EXPORT_HDR_LEN + 3U] = 0xFF;
	g_blob[CFG_EXPORT_HDR_LEN + 4U] = 0x00;
	TEST_ASSERT_EQUAL_INT(-EPROTO,
		cfg_import_all(&g_cfg, g_blob, n, true, &res));

	/* Wrong encoded width for the declared type. */
	{
		tlv_rec_t wide = { CFG_ID_TIM_TAU_S, CFG_T_U16, 4U,
				   { 1, 2, 3, 4, 0, 0, 0, 0 } };

		n = tlv_build(g_blob, CFG_SCHEMA_VERSION, &wide, 1U, true);
		TEST_ASSERT_EQUAL_INT(-EPROTO,
			cfg_import_all(&g_cfg, g_blob, n, true, &res));
	}

	/* A value that decodes but violates the schema bounds. */
	{
		tlv_rec_t hot = { CFG_ID_TIM_TAU_S, CFG_T_U16, 2U,
				  { 0xFF, 0xFF, 0, 0, 0, 0, 0, 0 } };

		n = tlv_build(g_blob, CFG_SCHEMA_VERSION, &hot, 1U, true);
		TEST_ASSERT_EQUAL_INT(-ERANGE,
			cfg_import_all(&g_cfg, g_blob, n, true, &res));
	}

	/* Type mismatch against the schema row. */
	{
		tlv_rec_t mistyped = { CFG_ID_TIM_TAU_S, CFG_T_U32, 4U,
				       { 0x2C, 0x01, 0, 0, 0, 0, 0, 0 } };

		n = tlv_build(g_blob, CFG_SCHEMA_VERSION, &mistyped, 1U, true);
		TEST_ASSERT_EQUAL_INT(-EPROTO,
			cfg_import_all(&g_cfg, g_blob, n, true, &res));
	}

	/* Truncated stream. */
	n = tlv_build(g_blob, CFG_SCHEMA_VERSION, &ok, 1U, true);
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
		cfg_import_all(&g_cfg, g_blob, n - 2U, true, &res));

	/* A bad CRC over an otherwise perfect stream. */
	n = tlv_build(g_blob, CFG_SCHEMA_VERSION, &ok, 1U, false);
	TEST_ASSERT_EQUAL_INT(-EILSEQ,
		cfg_import_all(&g_cfg, g_blob, n, true, &res));
}

static void test_import_accepts_an_empty_record_set(void)
{
	cfg_commit_res_t res;
	size_t n = tlv_build(g_blob, CFG_SCHEMA_VERSION, NULL, 0U, true);

	TEST_ASSERT_EQUAL_INT(0, cfg_import_all(&g_cfg, g_blob, n, true, &res));
	TEST_ASSERT_EQUAL_UINT16(0U, res.applied);
}

static void test_import_accepts_a_zero_length_value(void)
{
	tlv_rec_t empty = { CFG_ID_NET_HOSTNAME, CFG_T_STR, 0U,
			    { 0, 0, 0, 0, 0, 0, 0, 0 } };
	cfg_commit_res_t res;
	size_t n = tlv_build(g_blob, CFG_SCHEMA_VERSION, &empty, 1U, true);
	size_t blen = 99U;
	uint8_t buf[CFG_VAL_MAX];

	TEST_ASSERT_EQUAL_INT(0, cfg_import_all(&g_cfg, g_blob, n, true, &res));
	TEST_ASSERT_EQUAL_UINT16(1U, res.applied);
	TEST_ASSERT_EQUAL_INT(0,
		cfg_get_bytes(&g_cfg, CFG_ID_NET_HOSTNAME, buf, sizeof(buf),
			      &blen));
	TEST_ASSERT_EQUAL_size_t(0U, blen);
}

static void test_import_feeds_one_byte_at_a_time(void)
{
	cfg_commit_res_t res;
	cfg_import_t im;
	size_t n = 0U;
	size_t i;
	int rc = 0;
	uint64_t u = 0U;

	TEST_ASSERT_EQUAL_INT(0, cfg_set_u64(&g_cfg, CFG_ID_TIM_TAU_S, 654U));
	TEST_ASSERT_EQUAL_INT(0, cfg_commit(&g_cfg, NULL));
	TEST_ASSERT_EQUAL_INT(0,
		cfg_export_all(&g_cfg, g_blob, sizeof(g_blob), false, &n));

	TEST_ASSERT_EQUAL_INT(0, cfg_init(&g_cfg, &g_store_port));
	TEST_ASSERT_EQUAL_INT(0, cfg_import_begin(&g_cfg, &im, true));

	/* Not complete until the very last byte of the trailer. */
	TEST_ASSERT_EQUAL_INT(-EAGAIN, cfg_import_finish(&g_cfg, &im, &res));

	for (i = 0U; i < n; i++) {
		size_t used = 0U;

		rc = cfg_import_feed(&g_cfg, &im, &g_blob[i], 1U, &used);
		TEST_ASSERT_EQUAL_size_t(1U, used);
		TEST_ASSERT_GREATER_OR_EQUAL_INT(0, rc);
		if (i + 1U < n) {
			TEST_ASSERT_EQUAL_INT(0, rc);
		}
	}
	TEST_ASSERT_EQUAL_INT(1, rc);
	TEST_ASSERT_EQUAL_UINT32((uint32_t)n, im.offset);

	TEST_ASSERT_EQUAL_INT(0, cfg_import_finish(&g_cfg, &im, &res));
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(654U, u);
}

static void test_import_latches_its_error(void)
{
	tlv_rec_t ok = { CFG_ID_TIM_TAU_S, CFG_T_U16, 2U,
			 { 0x2C, 0x01, 0, 0, 0, 0, 0, 0 } };
	cfg_import_t im;
	size_t n = tlv_build(g_blob, CFG_SCHEMA_VERSION, &ok, 1U, true);
	size_t used = 99U;

	g_blob[0] = (uint8_t)'Z';

	TEST_ASSERT_EQUAL_INT(0, cfg_import_begin(&g_cfg, &im, true));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
		cfg_import_feed(&g_cfg, &im, g_blob, n, NULL));

	/* Every later feed reports the same error and consumes nothing. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
		cfg_import_feed(&g_cfg, &im, g_blob, n, &used));
	TEST_ASSERT_EQUAL_size_t(0U, used);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, cfg_import_finish(&g_cfg, &im, NULL));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
		cfg_import_feed(&g_cfg, NULL, g_blob, n, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		cfg_import_feed(&g_cfg, &im, NULL, 4U, NULL));
}

/* ------------------------------------------------------------------------- */
/* Migration                                                                 */
/* ------------------------------------------------------------------------- */

static int mig_v0_to_v1(uint16_t *id, cfg_val_t *val, void *user)
{
	int *seen = (int *)user;

	(*seen)++;

	if (*id == 0x0BAD) {
		return 1; /* retired key: drop it */
	}
	if (*id == 0x0601) {
		/* v0 stored tau in deciseconds. */
		val->v.u /= 10U;
		return 0;
	}
	if (*id == 0x0DED) {
		return -EPROTO; /* a record the migration refuses to touch */
	}
	return 0;
}

static void test_import_runs_the_registered_migration(void)
{
	static const cfg_migration_t table[] = {
		{ .from_ver = 0U, .fn = mig_v0_to_v1 },
	};
	tlv_rec_t recs[2] = {
		/* 3000 deciseconds = 300 s once migrated. */
		{ 0x0601, CFG_T_U16, 2U, { 0xB8, 0x0B, 0, 0, 0, 0, 0, 0 } },
		{ 0x0BAD, CFG_T_U16, 2U, { 1, 0, 0, 0, 0, 0, 0, 0 } },
	};
	cfg_commit_res_t res;
	int seen = 0;
	size_t n;
	uint64_t u = 0U;

	TEST_ASSERT_EQUAL_INT(0,
		cfg_set_migrations(&g_cfg, table, 1U, &seen));

	n = tlv_build(g_blob, 0U, recs, 2U, true);
	TEST_ASSERT_EQUAL_INT(0, cfg_import_all(&g_cfg, g_blob, n, true, &res));
	TEST_ASSERT_EQUAL_INT(2, seen);
	TEST_ASSERT_EQUAL_UINT16(1U, res.applied);
	TEST_ASSERT_EQUAL_INT(0, cfg_get_u64(&g_cfg, CFG_ID_TIM_TAU_S, &u));
	TEST_ASSERT_EQUAL_UINT64(300U, u);

	/* A migration that aborts takes the whole import with it. */
	{
		tlv_rec_t bad = { 0x0DED, CFG_T_U16, 2U,
				  { 1, 0, 0, 0, 0, 0, 0, 0 } };

		n = tlv_build(g_blob, 0U, &bad, 1U, true);
		TEST_ASSERT_EQUAL_INT(-EPROTO,
			cfg_import_all(&g_cfg, g_blob, n, true, &res));
	}

	/* Current-version streams bypass the table entirely. */
	seen = 0;
	{
		tlv_rec_t cur = { CFG_ID_TIM_TAU_S, CFG_T_U16, 2U,
				  { 0x2C, 0x01, 0, 0, 0, 0, 0, 0 } };

		n = tlv_build(g_blob, CFG_SCHEMA_VERSION, &cur, 1U, true);
		TEST_ASSERT_EQUAL_INT(0,
			cfg_import_all(&g_cfg, g_blob, n, true, &res));
	}
	TEST_ASSERT_EQUAL_INT(0, seen);
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_schema_group_census);
	RUN_TEST(test_schema_is_sorted_and_well_formed);
	RUN_TEST(test_schema_defaults_pass_their_own_bounds);
	RUN_TEST(test_schema_defaults_fail_safe);
	RUN_TEST(test_schema_xvalidate_gates_snmp_on_its_community);
	RUN_TEST(test_schema_lookup);
	RUN_TEST(test_type_widths);

	RUN_TEST(test_val_codec_round_trip);
	RUN_TEST(test_val_codec_rejects);

	RUN_TEST(test_set_enforces_type);
	RUN_TEST(test_set_enforces_bounds);
	RUN_TEST(test_typed_accessors);
	RUN_TEST(test_null_arguments);

	RUN_TEST(test_staging_is_invisible_until_commit);
	RUN_TEST(test_revert_drops_staging);
	RUN_TEST(test_commit_reports_reboot_scope);
	RUN_TEST(test_commit_skips_unchanged_values);
	RUN_TEST(test_commit_is_all_or_nothing);
	RUN_TEST(test_commit_normalises_a_positive_hook_error);
	RUN_TEST(test_commit_reports_persist_failure_but_keeps_ram);
	RUN_TEST(test_ram_only_registry_needs_no_store);

	RUN_TEST(test_load_round_trips_through_the_store);
	RUN_TEST(test_load_defaults_corrupt_entries);
	RUN_TEST(test_load_rejects_an_overlong_store_record);
	RUN_TEST(test_factory_reset_clears_ram_and_store);
	RUN_TEST(test_factory_reset_reports_an_erase_failure);

	RUN_TEST(test_export_header_and_trailer_are_as_documented);
	RUN_TEST(test_export_omits_secrets_unless_asked);
	RUN_TEST(test_noexport_key_never_leaves_even_with_secrets);
	RUN_TEST(test_export_chunks_at_the_minimum_size);
	RUN_TEST(test_export_argument_errors);

	RUN_TEST(test_import_round_trip_restores_every_key);
	RUN_TEST(test_import_detects_a_corrupt_crc);
	RUN_TEST(test_import_unknown_id_strict_versus_lenient);
	RUN_TEST(test_import_rejects_malformed_streams);
	RUN_TEST(test_import_accepts_an_empty_record_set);
	RUN_TEST(test_import_accepts_a_zero_length_value);
	RUN_TEST(test_import_feeds_one_byte_at_a_time);
	RUN_TEST(test_import_latches_its_error);
	RUN_TEST(test_import_runs_the_registered_migration);

	return UNITY_END();
}
