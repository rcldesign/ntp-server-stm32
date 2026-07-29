/*
 * STS1000 "Meridian" — core/mp capability-manifest unit tests.
 *
 * The manifest is a data table, so most of these tests are self-consistency
 * proofs over it: every object id unique, every guard and group and kind in
 * range, every published interlock resolvable to a name, every envelope
 * coherent with its kind. A table like this rots silently, and a rotted manifest
 * is a tool that offers an operator a control the device does not have.
 *
 * The rest checks the manifest against the as-built hardware
 * (docs/sts1000_firmware_hardware_interface.md): the corrections that differ
 * from the FMT spec's examples are asserted here so a future edit cannot quietly
 * reintroduce the I/O expander or move the GNSS monitor back to 0x44.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "ina228/ina228.h"
#include "mp/mp_json.h"
#include "mp/mp_manifest.h"
#include "util/crc.h"

#define TOKS 128U

static mp_json_t g_p;
static mp_json_tok_t g_tok[TOKS];

/* ------------------------------------------------------------ enum coverage */

static void test_enum_names(void)
{
	TEST_ASSERT_EQUAL_STRING("G0", mp_guard_name(MP_GUARD_G0));
	TEST_ASSERT_EQUAL_STRING("G1", mp_guard_name(MP_GUARD_G1));
	TEST_ASSERT_EQUAL_STRING("G2", mp_guard_name(MP_GUARD_G2));
	TEST_ASSERT_EQUAL_STRING("G3", mp_guard_name(MP_GUARD_G3));
	TEST_ASSERT_EQUAL_STRING("G?", mp_guard_name(MP_GUARD_COUNT));

	TEST_ASSERT_EQUAL_STRING("bool", mp_kind_name(MP_KIND_BOOL));
	TEST_ASSERT_EQUAL_STRING("rail", mp_kind_name(MP_KIND_RAIL));
	TEST_ASSERT_EQUAL_STRING("unknown", mp_kind_name(MP_KIND_COUNT));

	TEST_ASSERT_EQUAL_STRING("power", mp_group_name(MP_GRP_POWER));
	TEST_ASSERT_EQUAL_STRING("sensor", mp_group_name(MP_GRP_SENSOR));
	TEST_ASSERT_EQUAL_STRING("unknown", mp_group_name(MP_GRP_COUNT));
}

static void test_every_interlock_bit_resolves(void)
{
	uint32_t bit;
	unsigned int found = 0U;

	for (bit = 1U; bit <= MP_ILK_ALL; bit <<= 1) {
		const char *name = mp_ilk_name(bit);

		TEST_ASSERT_NOT_NULL_MESSAGE(name, "interlock bit has no name");
		TEST_ASSERT_TRUE(strlen(name) > 0U);
		TEST_ASSERT_NOT_NULL(mp_ilk_reason(bit));
		TEST_ASSERT_TRUE(strlen(mp_ilk_reason(bit)) > 0U);
		found++;
	}
	TEST_ASSERT_EQUAL_UINT(MP_ILK_COUNT, found);

	/* Not a single defined bit: no name. */
	TEST_ASSERT_NULL(mp_ilk_name(0U));
	TEST_ASSERT_NULL(mp_ilk_name(1U << MP_ILK_COUNT));
	TEST_ASSERT_NULL(mp_ilk_name(MP_ILK_RB_OV | MP_ILK_RB_VMAX));
	TEST_ASSERT_EQUAL_STRING("unknown interlock",
				 mp_ilk_reason(1U << MP_ILK_COUNT));
}

static void test_interlock_names_are_unique(void)
{
	uint32_t a;
	uint32_t b;

	for (a = 1U; a <= MP_ILK_ALL; a <<= 1) {
		for (b = a << 1; b <= MP_ILK_ALL; b <<= 1) {
			TEST_ASSERT_TRUE_MESSAGE(strcmp(mp_ilk_name(a),
							mp_ilk_name(b)) != 0,
						 mp_ilk_name(a));
		}
	}
}

/* ------------------------------------------------------- table consistency */

static void test_table_is_not_empty(void)
{
	TEST_ASSERT_TRUE(mp_obj_count() > 60U);
	TEST_ASSERT_NOT_NULL(mp_obj_at(0U));
	TEST_ASSERT_NOT_NULL(mp_obj_at(mp_obj_count() - 1U));
	TEST_ASSERT_NULL(mp_obj_at(mp_obj_count()));
}

static void test_every_id_is_unique_and_well_formed(void)
{
	size_t i;
	size_t j;
	size_t n = mp_obj_count();

	for (i = 0U; i < n; i++) {
		const mp_obj_t *o = mp_obj_at(i);
		size_t k;
		size_t len;

		TEST_ASSERT_NOT_NULL(o->id);
		len = strlen(o->id);
		TEST_ASSERT_TRUE_MESSAGE(len > 2U, o->id);
		TEST_ASSERT_TRUE_MESSAGE(len < 48U, o->id);

		/* Ids are dotted lower-case identifiers: the tool builds UI paths
		 * from them, so no spaces, no upper case, no leading/trailing dot. */
		TEST_ASSERT_NOT_EQUAL_MESSAGE('.', o->id[0], o->id);
		TEST_ASSERT_NOT_EQUAL_MESSAGE('.', o->id[len - 1U], o->id);
		for (k = 0U; k < len; k++) {
			char ch = o->id[k];
			bool ok = ((ch >= 'a') && (ch <= 'z')) ||
				  ((ch >= '0') && (ch <= '9')) || (ch == '.') ||
				  (ch == '_');

			TEST_ASSERT_TRUE_MESSAGE(ok, o->id);
		}

		/* Every id is dotted: a group prefix and at least one segment. */
		TEST_ASSERT_NOT_NULL_MESSAGE(strchr(o->id, '.'), o->id);

		for (j = i + 1U; j < n; j++) {
			TEST_ASSERT_TRUE_MESSAGE(
				strcmp(o->id, mp_obj_at(j)->id) != 0, o->id);
		}
	}
}

static void test_every_field_is_in_range(void)
{
	size_t i;
	size_t n = mp_obj_count();

	for (i = 0U; i < n; i++) {
		const mp_obj_t *o = mp_obj_at(i);

		TEST_ASSERT_TRUE_MESSAGE(o->guard < (uint8_t)MP_GUARD_COUNT,
					 o->id);
		TEST_ASSERT_TRUE_MESSAGE(o->group < (uint8_t)MP_GRP_COUNT,
					 o->id);
		TEST_ASSERT_TRUE_MESSAGE(o->kind < (uint8_t)MP_KIND_COUNT,
					 o->id);

		/* No undefined interlock bits. */
		TEST_ASSERT_EQUAL_UINT32_MESSAGE(0U, o->ilk & ~MP_ILK_ALL,
						 o->id);

		/* Every declared interlock resolves. */
		{
			uint32_t bit;

			for (bit = 1U; bit <= MP_ILK_ALL; bit <<= 1) {
				if ((o->ilk & bit) != 0U) {
					TEST_ASSERT_NOT_NULL_MESSAGE(
						mp_ilk_name(bit), o->id);
				}
			}
		}

		/* Description carries something a technician can use. */
		TEST_ASSERT_NOT_NULL_MESSAGE(o->desc, o->id);
		TEST_ASSERT_TRUE_MESSAGE(strlen(o->desc) > 8U, o->id);

		/* Readable at minimum; nothing in the table is write-only. */
		TEST_ASSERT_TRUE_MESSAGE((o->flags & MP_OF_READ) != 0U, o->id);

		/* cfg_key and the MP_OF_CFG flag agree in both directions. */
		if ((o->flags & MP_OF_CFG) != 0U) {
			TEST_ASSERT_NOT_EQUAL_MESSAGE(0U, o->cfg_key, o->id);
		} else {
			TEST_ASSERT_EQUAL_UINT16_MESSAGE(0U, o->cfg_key, o->id);
		}
	}
}

static void test_ranged_kinds_have_coherent_envelopes(void)
{
	size_t i;
	size_t n = mp_obj_count();

	for (i = 0U; i < n; i++) {
		const mp_obj_t *o = mp_obj_at(i);

		switch (o->kind) {
		case MP_KIND_ENUM:
		case MP_KIND_PCT:
		case MP_KIND_MV:
		case MP_KIND_CODE:
		case MP_KIND_PULSE:
		case MP_KIND_SCALAR:
			TEST_ASSERT_TRUE_MESSAGE(o->min <= o->max, o->id);
			TEST_ASSERT_TRUE_MESSAGE(o->step >= 0, o->id);
			if (o->step > 0) {
				TEST_ASSERT_TRUE_MESSAGE(
					o->step <= (o->max - o->min), o->id);
			}
			break;
		default:
			break;
		}

		if (o->kind == (uint8_t)MP_KIND_PCT) {
			TEST_ASSERT_EQUAL_INT32_MESSAGE(0, o->min, o->id);
			TEST_ASSERT_EQUAL_INT32_MESSAGE(100, o->max, o->id);
			TEST_ASSERT_EQUAL_STRING_MESSAGE("%", o->unit, o->id);
		}
		if (o->kind == (uint8_t)MP_KIND_PULSE) {
			/* A pulse duration must be positive and bounded. */
			TEST_ASSERT_TRUE_MESSAGE(o->min >= 1, o->id);
			TEST_ASSERT_TRUE_MESSAGE(o->max <= 60000, o->id);
			TEST_ASSERT_EQUAL_STRING_MESSAGE("ms", o->unit, o->id);
			TEST_ASSERT_TRUE_MESSAGE(
				(o->flags & MP_OF_PULSE) != 0U, o->id);
		}
	}
}

static void test_enum_and_bits_kinds_name_their_values(void)
{
	size_t i;
	size_t n = mp_obj_count();

	for (i = 0U; i < n; i++) {
		const mp_obj_t *o = mp_obj_at(i);

		if ((o->kind == (uint8_t)MP_KIND_ENUM) ||
		    (o->kind == (uint8_t)MP_KIND_BITS)) {
			const char *p = o->enums;
			unsigned int count = 1U;

			TEST_ASSERT_NOT_NULL_MESSAGE(p, o->id);
			TEST_ASSERT_TRUE_MESSAGE(strlen(p) > 0U, o->id);
			TEST_ASSERT_NOT_EQUAL_MESSAGE(',', p[0], o->id);
			TEST_ASSERT_NOT_EQUAL_MESSAGE(
				',', p[strlen(p) - 1U], o->id);
			while (*p != '\0') {
				if (*p == ',') {
					count++;
					/* No empty names. */
					TEST_ASSERT_NOT_EQUAL_MESSAGE(
						',', p[1], o->id);
				}
				p++;
			}
			if (o->kind == (uint8_t)MP_KIND_ENUM) {
				/* The enum range must match the name count. */
				TEST_ASSERT_EQUAL_INT32_MESSAGE(
					(int32_t)count - 1, o->max, o->id);
				TEST_ASSERT_EQUAL_INT32_MESSAGE(0, o->min,
								o->id);
			} else {
				/* A bitmap must fit a 64-bit mask. */
				TEST_ASSERT_TRUE_MESSAGE(count <= 64U, o->id);
			}
		} else {
			/* Only ENUM and BITS carry a value-name list. */
			if (o->enums != NULL) {
				TEST_ASSERT_TRUE_MESSAGE(false, o->id);
			}
		}
	}
}

/* ------------------------------------------------------------ guard policy */

static void test_writable_objects_are_guarded(void)
{
	size_t i;
	size_t n = mp_obj_count();
	unsigned int mutable_count = 0U;

	for (i = 0U; i < n; i++) {
		const mp_obj_t *o = mp_obj_at(i);
		bool mutates = (o->flags & (MP_OF_WRITE | MP_OF_OVERRIDE |
					    MP_OF_PULSE)) != 0U;

		if (!mutates) {
			/* A read-only object is G0 by definition. */
			TEST_ASSERT_EQUAL_UINT8_MESSAGE((uint8_t)MP_GUARD_G0,
							o->guard, o->id);
			continue;
		}
		mutable_count++;
		/*
		 * Anything that changes board state needs at least a session.
		 * No exemptions, by name or otherwise: a G0 request is answered
		 * for MP_ROLE_NONE, so a G0 mutator routes actuation around the
		 * whole authentication check. `ui.identify` used to be exempted
		 * here and is now G1 — see the note on its manifest row.
		 */
		TEST_ASSERT_TRUE_MESSAGE(o->guard >= (uint8_t)MP_GUARD_G1, o->id);
		/* A sensor is never mutable. */
		TEST_ASSERT_NOT_EQUAL_MESSAGE((uint8_t)MP_GRP_SENSOR, o->group,
					      o->id);
	}
	TEST_ASSERT_TRUE(mutable_count > 25U);
}

/**
 * The rule above, stated the other way round so it cannot be satisfied vacuously.
 *
 * test_writable_objects_are_guarded() would still pass if the mutating-object
 * predicate were broken (nothing would be checked). This counts the G0 mutators
 * directly and requires the count to be exactly zero, and separately proves that
 * G0 objects exist at all — so "no G0 mutator" cannot be true because there are
 * no G0 rows left to look at.
 */
static void test_no_g0_object_can_mutate(void)
{
	size_t i;
	size_t n = mp_obj_count();
	unsigned int g0_mutators = 0U;
	unsigned int g0_total = 0U;
	const char *first = "none";

	for (i = 0U; i < n; i++) {
		const mp_obj_t *o = mp_obj_at(i);

		if (o->guard != (uint8_t)MP_GUARD_G0) {
			continue;
		}
		g0_total++;
		if ((o->flags & (MP_OF_WRITE | MP_OF_OVERRIDE | MP_OF_PULSE)) !=
		    0U) {
			if (g0_mutators == 0U) {
				first = o->id;
			}
			g0_mutators++;
		}
	}
	TEST_ASSERT_EQUAL_UINT_MESSAGE(0U, g0_mutators, first);
	TEST_ASSERT_TRUE(g0_total > 20U);
}

/**
 * The specific guard classes that matter.
 *
 * These are policy, not derivable from the table, so they are pinned by name:
 * anything that can stop the served clock or the board is G3, and the rubidium
 * chain and the calibration-adjacent rails are G2.
 */
static void test_guard_classes_of_the_dangerous_objects(void)
{
	static const struct {
		const char *id;
		uint8_t guard;
	} expect[] = {
		/* G3 — stops the clock or the board. */
		{ "ref.mux.sel", MP_GUARD_G3 },
		{ "ref.ocxo.vc_mv", MP_GUARD_G3 },
		{ "ref.ocxo.dac_code", MP_GUARD_G3 },
		{ "pwr.poe.kill", MP_GUARD_G3 },
		{ "pwr.rb.pot.code", MP_GUARD_G3 },
		{ "sys.wdt.en", MP_GUARD_G3 },
		{ "sys.wdt.kick", MP_GUARD_G3 },
		{ "gnss.safeboot", MP_GUARD_G3 },
		/* G2 — interrupts service or risks an attached instrument. */
		{ "pwr.rb.en", MP_GUARD_G2 },
		{ "pwr.rb.gate", MP_GUARD_G2 },
		{ "pwr.rb.vset_mv", MP_GUARD_G2 },
		{ "pwr.rb.vmax_mv", MP_GUARD_G2 },
		{ "pwr.gps.en", MP_GUARD_G2 },
		{ "ref.relay.hold", MP_GUARD_G2 },
		{ "sys.fan.duty", MP_GUARD_G2 },
		{ "sys.nor.reset", MP_GUARD_G2 },
		{ "sys.phy.reset", MP_GUARD_G2 },
		{ "gnss.reset", MP_GUARD_G2 },
		{ "ref.rb.tunnel", MP_GUARD_G2 },
		{ "gnss.tunnel", MP_GUARD_G2 },
		{ "sys.smp.tunnel", MP_GUARD_G2 },
		/* G1 — cosmetic or recoverable. */
		{ "ui.panel.duty", MP_GUARD_G1 },
		{ "ui.disp.bl", MP_GUARD_G1 },
		{ "ui.rgb.r", MP_GUARD_G1 },
		{ "pwr.disp.en", MP_GUARD_G1 },
		{ "ref.term.en", MP_GUARD_G1 },
		/*
		 * The locate pulse. G1 rather than G0: it is overridable and it
		 * outranks the fault colour, so it needs a session and a role
		 * like every other mutator.
		 */
		{ "ui.identify", MP_GUARD_G1 },
	};
	size_t i;

	for (i = 0U; i < (sizeof(expect) / sizeof(expect[0])); i++) {
		int idx = mp_obj_find(expect[i].id);

		TEST_ASSERT_TRUE_MESSAGE(idx >= 0, expect[i].id);
		TEST_ASSERT_EQUAL_UINT8_MESSAGE(expect[i].guard,
						mp_obj_at((size_t)idx)->guard,
						expect[i].id);
	}
}

/** The interlocks the spec §5.5 names, bound to the objects they must guard. */
static void test_interlocks_are_attached_where_they_must_be(void)
{
	static const struct {
		const char *id;
		uint32_t must;
	} expect[] = {
		{ "pwr.rb.vset_mv",
		  MP_ILK_RB_VMAX | MP_ILK_RB_OV | MP_ILK_RB_VERIFY },
		{ "pwr.rb.pot.code", MP_ILK_RB_VMAX | MP_ILK_RB_OV },
		{ "pwr.rb.en", MP_ILK_RB_WARM | MP_ILK_RB_OV },
		{ "pwr.rb.gate", MP_ILK_RB_OV | MP_ILK_RB_VERIFY },
		{ "sys.fan.duty", MP_ILK_FAN_FLOOR },
		{ "ref.relay.hold", MP_ILK_RELAY_OK },
		{ "pwr.disp.en", MP_ILK_DISP_OFF },
		{ "sys.wdt.en", MP_ILK_WDT_LIVE },
		{ "sys.wdt.kick", MP_ILK_WDT_LIVE },
		{ "ref.mux.sel", MP_ILK_MUX_GUARD },
		{ "ref.ocxo.vc_mv", MP_ILK_DAC_PARK },
		{ "ref.ocxo.dac_code", MP_ILK_DAC_PARK },
		{ "ref.rb.tunnel", MP_ILK_TUNNEL },
		{ "gnss.tunnel", MP_ILK_TUNNEL },
		{ "sys.smp.tunnel", MP_ILK_TUNNEL },
	};
	size_t i;

	for (i = 0U; i < (sizeof(expect) / sizeof(expect[0])); i++) {
		int idx = mp_obj_find(expect[i].id);
		const mp_obj_t *o;

		TEST_ASSERT_TRUE_MESSAGE(idx >= 0, expect[i].id);
		o = mp_obj_at((size_t)idx);
		TEST_ASSERT_EQUAL_UINT32_MESSAGE(expect[i].must,
						 o->ilk & expect[i].must,
						 expect[i].id);
	}
}

/**
 * A tunnel is a LEASED thing, so it may be overridden and never written.
 *
 * `gnss.tunnel` and `ref.rb.tunnel` carried MP_OF_WRITE, and m_obj_set() checks
 * only that flag — so `obj.set gnss.tunnel true` reached obj_apply(), called
 * sts_gnss_uart_suspend() and parked gnssmgr in GNSSMGR_ST_FW_UPDATE while
 * creating **no lease**. mp_ovr_revert_all() — the dead-man, the link drop,
 * `session.close`, mode exit — had nothing to revert, and MP_ILK_TUNNEL is a
 * consequence rather than a refusal, so nothing stopped it. The grandmaster
 * lost GNSS until a reboot.
 *
 * Stated over the INTERLOCK rather than over the three ids, because the rule is
 * about what MP_ILK_TUNNEL means: it suspends firmware's use of a port, and
 * only a lease can put that back. A fourth tunnel object added later is covered
 * the day it declares the interlock.
 */
static void test_a_tunnel_object_is_never_writable(void)
{
	static const char *const tunnels[] = { "gnss.tunnel", "ref.rb.tunnel",
					       "sys.smp.tunnel" };
	size_t i;
	unsigned int with_ilk = 0U;

	for (i = 0U; i < mp_obj_count(); i++) {
		const mp_obj_t *o = mp_obj_at(i);

		if ((o->ilk & MP_ILK_TUNNEL) == 0U) {
			continue;
		}
		with_ilk++;
		TEST_ASSERT_EQUAL_UINT_MESSAGE(
			0U, (unsigned int)(o->flags & MP_OF_WRITE), o->id);
		/* ...and it must still be reachable the leased way, or the
		 * narrowing would have removed the feature instead of the bug. */
		TEST_ASSERT_TRUE_MESSAGE((o->flags & MP_OF_OVERRIDE) != 0U,
					 o->id);
	}
	TEST_ASSERT_EQUAL_UINT((unsigned int)(sizeof(tunnels) /
					      sizeof(tunnels[0])),
			       with_ilk);

	for (i = 0U; i < (sizeof(tunnels) / sizeof(tunnels[0])); i++) {
		int idx = mp_obj_find(tunnels[i]);

		TEST_ASSERT_TRUE_MESSAGE(idx >= 0, tunnels[i]);
		TEST_ASSERT_TRUE_MESSAGE(
			(mp_obj_at((size_t)idx)->ilk & MP_ILK_TUNNEL) != 0U,
			tunnels[i]);
	}
}

/**
 * The honesty flag reaches the wire, on exactly the objects that carry it.
 *
 * MP_OF_DEFERRED was defined in the header, listed in emit_flags(), and set by
 * zero objects — so the manifest a host generates its UI from (FMT §4) could
 * not distinguish a control that works from one that answers MP_E_NOTSUP. The
 * set itself is proved against mp_glue.c's dispatch in test_mp_deferred.c; this
 * checks the serialisation, which is the half the host actually sees.
 */
static void test_the_deferred_flag_is_published(void)
{
	size_t i;
	unsigned int deferred = 0U;

	for (i = 0U; i < mp_obj_count(); i++) {
		char buf[MP_MANIFEST_OBJ_JSON_MAX];
		const mp_obj_t *o = mp_obj_at(i);
		bool want = (o->flags & MP_OF_DEFERRED) != 0U;
		int len = mp_manifest_obj_json(i, buf, sizeof(buf));
		int flags;
		uint16_t k;
		uint16_t n;
		bool seen = false;

		TEST_ASSERT_TRUE_MESSAGE(len > 0, o->id);
		TEST_ASSERT_TRUE(mp_json_parse(&g_p, buf, (size_t)len, g_tok,
					       TOKS, 0U) > 0);
		flags = mp_json_obj_get(&g_p, mp_json_root(&g_p), "flags");
		TEST_ASSERT_TRUE_MESSAGE(flags >= 0, o->id);

		n = mp_json_count(&g_p, flags);
		for (k = 0U; k < n; k++) {
			if (mp_json_streq(&g_p, mp_json_arr_at(&g_p, flags, k),
					  "deferred")) {
				seen = true;
			}
		}
		TEST_ASSERT_EQUAL_MESSAGE(want, seen, o->id);
		if (want) {
			deferred++;
		}
	}

	/* Neither all nor none: a manifest where every object is deferred, or
	 * none is, would satisfy the per-object check above and tell the host
	 * nothing. */
	TEST_ASSERT_TRUE(deferred > 20U);
	TEST_ASSERT_TRUE(deferred < mp_obj_count());
}

/* --------------------------------------------------------- as-built checks */

static void test_no_io_expander_anywhere(void)
{
	size_t i;
	size_t n = mp_obj_count();

	/*
	 * The FMT spec's examples reference an MCP23017 at U47 with an
	 * EXP_RESET line. This board has none: the aggregated inputs are a
	 * direct 1 kHz GPIOF/GPIOG scan. If either string ever appears in the
	 * manifest, someone has copied the spec's example instead of the board.
	 */
	for (i = 0U; i < n; i++) {
		const mp_obj_t *o = mp_obj_at(i);

		TEST_ASSERT_NULL_MESSAGE(strstr(o->id, "exp"), o->id);
		TEST_ASSERT_NULL_MESSAGE(strstr(o->desc, "U47"), o->id);
		TEST_ASSERT_NULL_MESSAGE(strstr(o->desc, "EXP_RESET"), o->id);
		TEST_ASSERT_NULL_MESSAGE(strstr(o->desc, "MCP23017"), o->id);
		if (o->net != NULL) {
			TEST_ASSERT_NULL_MESSAGE(strstr(o->net, "EXP_"), o->id);
		}
	}

	/* The replacement is there instead. */
	TEST_ASSERT_TRUE(mp_obj_find("sensor.pg") >= 0);
	TEST_ASSERT_TRUE(mp_obj_find("sensor.ina.alert") >= 0);
	TEST_ASSERT_TRUE(mp_obj_find("sensor.en.fault") >= 0);
	TEST_ASSERT_TRUE(mp_obj_find("sensor.buttons") >= 0);
}

static void test_rail_objects_match_the_as_built_monitor_table(void)
{
	static const struct {
		const char *id;
		uint8_t addr;
		const char *dev;
	} expect[] = {
		{ "sensor.rail.poe", 0x40U, "U10" },
		{ "sensor.rail.3v3_stm", 0x41U, "U31" },
		{ "sensor.rail.5v_disp", 0x42U, "U32" },
		{ "sensor.rail.3v3", 0x43U, "U30" },
		{ "sensor.rail.v_ant", 0x45U, "U26" },
		{ "sensor.rail.ocxo", 0x46U, "U37" },
		{ "sensor.rail.vcc_rb", 0x47U, "U44" },
		/* 0x4A, not 0x44: SHT45 U72 owns 0x44 on the same bus. */
		{ "sensor.rail.3v3_gps", 0x4AU, "U23" },
		{ "sensor.rail.panel_5v", 0x4CU, "U54" },
	};
	size_t i;
	unsigned int rail_objects = 0U;
	size_t n = mp_obj_count();

	for (i = 0U; i < (sizeof(expect) / sizeof(expect[0])); i++) {
		int idx = mp_obj_find(expect[i].id);
		const mp_obj_t *o;
		const ina228_rail_info_t *r;

		TEST_ASSERT_TRUE_MESSAGE(idx >= 0, expect[i].id);
		o = mp_obj_at((size_t)idx);
		TEST_ASSERT_EQUAL_UINT8_MESSAGE((uint8_t)MP_KIND_RAIL, o->kind,
						expect[i].id);
		TEST_ASSERT_TRUE_MESSAGE(o->min >= 0, expect[i].id);
		TEST_ASSERT_TRUE_MESSAGE(o->min < (int32_t)INA228_RAIL_COUNT,
					 expect[i].id);

		r = ina228_rail((ina228_rail_t)o->min);
		TEST_ASSERT_NOT_NULL_MESSAGE(r, expect[i].id);
		TEST_ASSERT_EQUAL_UINT8_MESSAGE(expect[i].addr, r->addr,
						expect[i].id);
		TEST_ASSERT_EQUAL_STRING_MESSAGE(expect[i].dev, r->designator,
						 expect[i].id);
	}

	/* Exactly nine rail objects, one per monitor, no duplicates. */
	for (i = 0U; i < n; i++) {
		if (mp_obj_at(i)->kind == (uint8_t)MP_KIND_RAIL) {
			rail_objects++;
		}
	}
	TEST_ASSERT_EQUAL_UINT((unsigned int)INA228_RAIL_COUNT, rail_objects);
}

static void test_as_built_designators_and_pins(void)
{
	static const struct {
		const char *id;
		const char *needle;
	} expect[] = {
		/* Status RGB is D5 on TIM4, not D36. */
		{ "ui.rgb.r", "D5" },
		{ "ui.rgb.r", "TIM4_CH1" },
		{ "ui.rgb.g", "TIM4_CH2" },
		{ "ui.rgb.b", "TIM4_CH3" },
		{ "ui.rgb.r", "PD12" },
		{ "ui.rgb.g", "PD13" },
		{ "ui.rgb.b", "PD14" },
		/* Panel-LED PWM is PE0 / LPTIM2_CH2. */
		{ "ui.panel.duty", "LPTIM2_CH2" },
		{ "ui.panel.duty", "PE0" },
		/* The VCC_RB transfer function, and the digipot part. */
		{ "pwr.rb.vset_mv", "24.45" },
		{ "pwr.rb.vset_mv", "6.645" },
		{ "pwr.rb.vset_mv", "MCP41U83" },
		{ "pwr.rb.pot.code", "terminal B" },
		/* Load switches and buck designators. */
		{ "pwr.rb.en", "MIC28516" },
		{ "pwr.ant.bias.en", "U27" },
		{ "pwr.disp.en", "U33" },
		{ "pwr.panel.led.en", "U55" },
		{ "pwr.gps.en", "LT3045" },
		/* Clock mux and watchdog parts. */
		{ "ref.mux.sel", "74LVC1G157" },
		{ "sys.wdt.en", "TPS3430" },
		{ "sys.nor.reset", "MX25L25645" },
		{ "sys.phy.reset", "LAN8742" },
		/* The relay's fail-safe polarity has to be discoverable. */
		{ "ref.relay.hold", "energized" },
		/* The fan's fail-safe state has to be discoverable. */
		{ "sys.fan.duty", "full speed" },
	};
	size_t i;

	for (i = 0U; i < (sizeof(expect) / sizeof(expect[0])); i++) {
		int idx = mp_obj_find(expect[i].id);
		const mp_obj_t *o;

		TEST_ASSERT_TRUE_MESSAGE(idx >= 0, expect[i].id);
		o = mp_obj_at((size_t)idx);
		TEST_ASSERT_NOT_NULL_MESSAGE(strstr(o->desc, expect[i].needle),
					     expect[i].needle);
	}
}

static void test_vcc_rb_envelope_is_the_electrical_range(void)
{
	int idx = mp_obj_find("pwr.rb.vset_mv");
	const mp_obj_t *o;

	TEST_ASSERT_TRUE(idx >= 0);
	o = mp_obj_at((size_t)idx);

	/* The published range is the supply's own 4.51-24.45 V; the configured
	 * ceiling (pwr.rb.vmax.mv) clamps it at set time via the interlock. */
	TEST_ASSERT_EQUAL_INT32(4510, o->min);
	TEST_ASSERT_EQUAL_INT32(24450, o->max);
	TEST_ASSERT_EQUAL_STRING("mV", o->unit);
	TEST_ASSERT_TRUE((o->ilk & MP_ILK_RB_VMAX) != 0U);

	idx = mp_obj_find("pwr.rb.vmax_mv");
	TEST_ASSERT_TRUE(idx >= 0);
	o = mp_obj_at((size_t)idx);
	/* Bound to the real cfg key (0x0703 pwr.rb.vmax.mv). */
	TEST_ASSERT_EQUAL_UINT16(0x0703U, o->cfg_key);
	TEST_ASSERT_TRUE((o->flags & MP_OF_CFG) != 0U);
}

static void test_group_populations(void)
{
	size_t total = 0U;
	uint8_t g;

	for (g = 0U; g < (uint8_t)MP_GRP_COUNT; g++) {
		size_t n = mp_obj_count_in(g);

		TEST_ASSERT_TRUE_MESSAGE(n > 0U, mp_group_name(g));
		total += n;
	}
	TEST_ASSERT_EQUAL_size_t(mp_obj_count(), total);
	TEST_ASSERT_EQUAL_size_t(0U, mp_obj_count_in(MP_GRP_COUNT));
}

/* ------------------------------------------------------------ lookup */

static void test_find(void)
{
	size_t i;
	size_t n = mp_obj_count();

	for (i = 0U; i < n; i++) {
		TEST_ASSERT_EQUAL_INT((int)i, mp_obj_find(mp_obj_at(i)->id));
	}
	TEST_ASSERT_EQUAL_INT(-ENOENT, mp_obj_find("no.such.object"));
	TEST_ASSERT_EQUAL_INT(-ENOENT, mp_obj_find(""));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_obj_find(NULL));
}

/* -------------------------------------------------------- serialisation */

static void test_every_object_serialises_to_valid_json(void)
{
	size_t i;
	size_t n = mp_obj_count();

	for (i = 0U; i < n; i++) {
		char buf[MP_MANIFEST_OBJ_JSON_MAX];
		const mp_obj_t *o = mp_obj_at(i);
		int len = mp_manifest_obj_json(i, buf, sizeof(buf));
		int root;
		int v;
		char s[64];

		TEST_ASSERT_TRUE_MESSAGE(len > 0, o->id);
		TEST_ASSERT_EQUAL_INT((int)strlen(buf), len);

		TEST_ASSERT_TRUE_MESSAGE(mp_json_parse(&g_p, buf, (size_t)len,
						       g_tok, TOKS, 0U) > 0,
					 buf);
		root = mp_json_root(&g_p);
		TEST_ASSERT_EQUAL_INT(MP_J_OBJ, mp_json_at(&g_p, root)->type);

		/* The five members every object must publish. */
		v = mp_json_obj_get(&g_p, root, "id");
		TEST_ASSERT_TRUE_MESSAGE(v >= 0, o->id);
		TEST_ASSERT_TRUE(mp_json_streq(&g_p, v, o->id));

		v = mp_json_obj_get(&g_p, root, "kind");
		TEST_ASSERT_TRUE(v >= 0);
		TEST_ASSERT_TRUE(mp_json_streq(&g_p, v, mp_kind_name(o->kind)));

		v = mp_json_obj_get(&g_p, root, "guard");
		TEST_ASSERT_TRUE(v >= 0);
		TEST_ASSERT_TRUE(
			mp_json_streq(&g_p, v, mp_guard_name(o->guard)));

		v = mp_json_obj_get(&g_p, root, "group");
		TEST_ASSERT_TRUE(v >= 0);
		TEST_ASSERT_TRUE(
			mp_json_streq(&g_p, v, mp_group_name(o->group)));

		v = mp_json_obj_get(&g_p, root, "flags");
		TEST_ASSERT_TRUE(v >= 0);
		TEST_ASSERT_EQUAL_INT(MP_J_ARR, mp_json_at(&g_p, v)->type);

		v = mp_json_obj_get(&g_p, root, "desc");
		TEST_ASSERT_TRUE(v >= 0);
		TEST_ASSERT_TRUE(mp_json_str(&g_p, v, s, sizeof(s)) != 0);

		/* Interlocks appear as names, and only when declared. */
		v = mp_json_obj_get(&g_p, root, "ilk");
		if (o->ilk != 0U) {
			uint16_t k;
			uint16_t cnt;

			TEST_ASSERT_TRUE_MESSAGE(v >= 0, o->id);
			cnt = mp_json_count(&g_p, v);
			TEST_ASSERT_TRUE(cnt > 0U);
			for (k = 0U; k < cnt; k++) {
				int e = mp_json_arr_at(&g_p, v, k);
				bool matched = false;
				uint32_t bit;

				for (bit = 1U; bit <= MP_ILK_ALL; bit <<= 1) {
					if (((o->ilk & bit) != 0U) &&
					    mp_json_streq(&g_p, e,
							  mp_ilk_name(bit))) {
						matched = true;
					}
				}
				TEST_ASSERT_TRUE_MESSAGE(matched, o->id);
			}
		} else {
			TEST_ASSERT_TRUE_MESSAGE(v < 0, o->id);
		}

		/* Rail objects carry the monitor's identity. */
		if (o->kind == (uint8_t)MP_KIND_RAIL) {
			const ina228_rail_info_t *r =
				ina228_rail((ina228_rail_t)o->min);

			v = mp_json_obj_get(&g_p, root, "dev");
			TEST_ASSERT_TRUE_MESSAGE(v >= 0, o->id);
			TEST_ASSERT_TRUE(
				mp_json_streq(&g_p, v, r->designator));

			v = mp_json_obj_get(&g_p, root, "addr");
			TEST_ASSERT_TRUE(v >= 0);
			{
				int64_t addr = 0;

				TEST_ASSERT_EQUAL_INT(0, mp_json_i64(&g_p, v,
								     &addr));
				TEST_ASSERT_EQUAL_INT64(r->addr, addr);
			}
			v = mp_json_obj_get(&g_p, root, "shunt_uohm");
			TEST_ASSERT_TRUE(v >= 0);
		}

		/* cfg-backed objects publish their key. */
		v = mp_json_obj_get(&g_p, root, "cfg");
		if ((o->flags & MP_OF_CFG) != 0U) {
			int64_t key = 0;

			TEST_ASSERT_TRUE_MESSAGE(v >= 0, o->id);
			TEST_ASSERT_EQUAL_INT(0, mp_json_i64(&g_p, v, &key));
			TEST_ASSERT_EQUAL_INT64(o->cfg_key, key);
		} else {
			TEST_ASSERT_TRUE_MESSAGE(v < 0, o->id);
		}
	}
}

static void test_object_json_reports_no_space(void)
{
	char tiny[16];

	TEST_ASSERT_EQUAL_INT(-ENOSPC,
			      mp_manifest_obj_json(0U, tiny, sizeof(tiny)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_manifest_obj_json(mp_obj_count(), tiny,
						   sizeof(tiny)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_manifest_obj_json(0U, NULL, 16U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_manifest_obj_json(0U, tiny, 0U));
}

static void test_paging_covers_the_whole_table_exactly_once(void)
{
	char buf[2048];
	size_t from = 0U;
	unsigned int pages = 0U;
	unsigned int objects = 0U;

	while (from < mp_obj_count()) {
		size_t len = 0U;
		size_t next = 0U;
		int n = mp_manifest_page(from, buf, sizeof(buf), &len, &next);

		TEST_ASSERT_TRUE(n > 0);
		TEST_ASSERT_TRUE(next > from);
		TEST_ASSERT_EQUAL_size_t(from + (size_t)n, next);
		TEST_ASSERT_EQUAL_size_t(strlen(buf), len);

		/* Each page is a comma-separated element list: it parses as an
		 * array once the brackets are put back on. */
		{
			static char wrapped[2200];
			mp_json_t p;
			static mp_json_tok_t toks[2048];

			wrapped[0] = '[';
			memcpy(&wrapped[1], buf, len);
			wrapped[1U + len] = ']';
			TEST_ASSERT_TRUE(mp_json_parse(&p, wrapped, len + 2U,
						       toks, 2048U, 0U) > 0);
			TEST_ASSERT_EQUAL_UINT16((uint16_t)n,
						 mp_json_count(&p, 0));
		}

		objects += (unsigned int)n;
		from = next;
		pages++;
		TEST_ASSERT_TRUE(pages < 200U);
	}

	TEST_ASSERT_EQUAL_UINT((unsigned int)mp_obj_count(), objects);
	TEST_ASSERT_TRUE(pages > 1U); /* the table does not fit in one page */
}

static void test_paging_edges(void)
{
	char buf[2048];
	char tiny[24];
	size_t len = 0U;
	size_t next = 0U;

	/* `from` at the end yields an empty page that is already done. */
	TEST_ASSERT_EQUAL_INT(0, mp_manifest_page(mp_obj_count(), buf,
						  sizeof(buf), &len, &next));
	TEST_ASSERT_EQUAL_size_t(0U, len);
	TEST_ASSERT_EQUAL_size_t(mp_obj_count(), next);

	/* Past the end is an argument error. */
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_manifest_page(mp_obj_count() + 1U,
							buf, sizeof(buf), &len,
							&next));

	/* A buffer too small for one object makes no progress and says so. */
	TEST_ASSERT_EQUAL_INT(0, mp_manifest_page(0U, tiny, sizeof(tiny), &len,
						  &next));
	TEST_ASSERT_EQUAL_size_t(0U, next);
	TEST_ASSERT_EQUAL_size_t(0U, len);

	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_manifest_page(0U, NULL, 16U, &len,
							&next));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_manifest_page(0U, buf, 0U, &len,
							&next));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      mp_manifest_page(0U, buf, sizeof(buf), NULL,
					       &next));
	TEST_ASSERT_EQUAL_INT(-EINVAL, mp_manifest_page(0U, buf, sizeof(buf),
							&len, NULL));
}

static void test_hash_is_stable_and_covers_the_table(void)
{
	static char scratch[MP_MANIFEST_OBJ_JSON_MAX];
	uint32_t h1 = mp_manifest_hash(scratch, sizeof(scratch));
	uint32_t h2 = mp_manifest_hash(scratch, sizeof(scratch));
	uint32_t crc;
	size_t i;
	const char ver = (char)('0' + (char)(MP_MANIFEST_VER % 10U));

	TEST_ASSERT_EQUAL_UINT32(h1, h2);
	TEST_ASSERT_NOT_EQUAL_UINT32(0U, h1);

	/* Recompute it independently: the version byte then every object's
	 * canonical JSON, in table order. */
	crc = sts_crc32_ieee_update(STS_CRC32_IEEE_SEED, &ver, 1U);
	for (i = 0U; i < mp_obj_count(); i++) {
		int n = mp_manifest_obj_json(i, scratch, sizeof(scratch));

		TEST_ASSERT_TRUE(n > 0);
		crc = sts_crc32_ieee_update(crc, scratch, (size_t)n);
	}
	TEST_ASSERT_EQUAL_UINT32(crc, h1);

	/* It is not simply the CRC of the version byte — the objects contribute. */
	TEST_ASSERT_NOT_EQUAL_UINT32(sts_crc32_ieee(&ver, 1U), h1);

	/* A scratch buffer that cannot hold the widest object is refused rather
	 * than producing a hash over truncated objects. */
	TEST_ASSERT_EQUAL_UINT32(0U, mp_manifest_hash(NULL, sizeof(scratch)));
	TEST_ASSERT_EQUAL_UINT32(0U, mp_manifest_hash(scratch, 16U));
}

/** Every object must fit the documented single-object bound. */
static void test_no_object_exceeds_the_json_bound(void)
{
	static char buf[MP_MANIFEST_OBJ_JSON_MAX];
	size_t i;
	int widest = 0;

	for (i = 0U; i < mp_obj_count(); i++) {
		int n = mp_manifest_obj_json(i, buf, sizeof(buf));

		TEST_ASSERT_TRUE_MESSAGE(n > 0, mp_obj_at(i)->id);
		if (n > widest) {
			widest = n;
		}
	}
	/* And the bound is not wildly oversized either. */
	TEST_ASSERT_TRUE(widest > 0);
	TEST_ASSERT_TRUE((size_t)widest <= MP_MANIFEST_OBJ_JSON_MAX);
	TEST_ASSERT_TRUE((size_t)widest > (MP_MANIFEST_OBJ_JSON_MAX / 2U));
}

/* ------------------------------------------------------------------- runner */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_enum_names);
	RUN_TEST(test_every_interlock_bit_resolves);
	RUN_TEST(test_interlock_names_are_unique);

	RUN_TEST(test_table_is_not_empty);
	RUN_TEST(test_every_id_is_unique_and_well_formed);
	RUN_TEST(test_every_field_is_in_range);
	RUN_TEST(test_ranged_kinds_have_coherent_envelopes);
	RUN_TEST(test_enum_and_bits_kinds_name_their_values);

	RUN_TEST(test_writable_objects_are_guarded);
	RUN_TEST(test_no_g0_object_can_mutate);
	RUN_TEST(test_guard_classes_of_the_dangerous_objects);
	RUN_TEST(test_interlocks_are_attached_where_they_must_be);
	RUN_TEST(test_a_tunnel_object_is_never_writable);
	RUN_TEST(test_the_deferred_flag_is_published);

	RUN_TEST(test_no_io_expander_anywhere);
	RUN_TEST(test_rail_objects_match_the_as_built_monitor_table);
	RUN_TEST(test_as_built_designators_and_pins);
	RUN_TEST(test_vcc_rb_envelope_is_the_electrical_range);
	RUN_TEST(test_group_populations);
	RUN_TEST(test_find);

	RUN_TEST(test_every_object_serialises_to_valid_json);
	RUN_TEST(test_object_json_reports_no_space);
	RUN_TEST(test_paging_covers_the_whole_table_exactly_once);
	RUN_TEST(test_paging_edges);
	RUN_TEST(test_hash_is_stable_and_covers_the_table);
	RUN_TEST(test_no_object_exceeds_the_json_bound);

	return UNITY_END();
}
