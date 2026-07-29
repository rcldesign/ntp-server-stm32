/*
 * STS1000 "Meridian" — the manifest's honesty flag: MP_OF_DEFERRED against the
 * dispatch that defines it.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * WHAT THIS SUITE EXISTS TO CATCH, stated so it cannot be diluted later.
 *
 * FMT §4 has the host download the manifest and generate its whole UI from it,
 * hardcoding nothing. The manifest publishes 91 objects and, for most of a
 * board's worth of controls, `obj_apply()`/`obj_pulse()` in
 * zephyr/console/mp_glue.c answer -ENOTSUP — so a maintenance tool rendered a
 * rack of working-looking controls and a technician found out which ones were
 * real by trying them at a bench. MP_OF_DEFERRED is how the device says so in
 * advance. It was defined, it was emitted by emit_flags(), and it was set by
 * ZERO objects: the flag existed and said nothing.
 *
 * A flag is not the fix. A flag plus a proof that it still matches the code is
 * the fix, because the failure mode is not "somebody wrote it down wrong once",
 * it is "somebody wires a setter and the manifest keeps saying it is deferred"
 * — or the reverse, which is worse: a cleared row and no implementation, i.e.
 * the exact state this change repairs, reintroduced silently.
 *
 * So this suite does not restate the flag. It DERIVES the wired set from the
 * dispatch and compares. mp_glue.c is a Zephyr translation unit no host suite
 * links, so the derivation is a source read — brace-matched and function-scoped,
 * the way test_mp_events.c and test_mp_veto.c read their seams — and it is
 * narrow on purpose: only `strcmp(o->id, "…")` inside a named function body
 * counts, so an id mentioned in prose or in a log message can never satisfy a
 * claim about the code. Comments are blanked; string literals are NOT, because
 * the ids ARE the dispatch here.
 *
 * The two policy-table branches (`sts_recov_bool_action`, `sts_recov_pulse_action`)
 * are not scanned as text at all — they are the real functions out of
 * console/sts_recovery_policy.h, compiled and called.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "mp/mp.h"
#include "mp/mp_manifest.h"
#include "zephyr/platform/sts_rbguard.h"
#include "zephyr/console/sts_recovery_policy.h"

#ifndef STS_APP_SRC_DIR
#error "STS_APP_SRC_DIR must be defined by tests/host/CMakeLists.txt"
#endif

void setUp(void)
{
}

void tearDown(void)
{
}

/* ===================================================== the source scanner == */

#define SRC_MAX (256U * 1024U)

static char g_src[SRC_MAX];
static size_t g_src_len;
static char g_src_name[128];

typedef struct {
	size_t begin;
	size_t end;
} span_t;

/**
 * Read @p rel into g_src, blanking comments in place so offsets stay 1:1.
 *
 * String literals are KEPT, unlike the loaders in test_mp_events.c and
 * test_mp_veto.c, because here the quoted object ids are the thing under test.
 * The narrowing that replaces "blank the strings" is the match pattern: nothing
 * counts unless it is literally `strcmp(o->id, "` inside the body of a named
 * function, which prose and log messages cannot be.
 */
static void load_source(const char *rel)
{
	char path[512];
	FILE *f;
	size_t n;
	size_t i;
	enum { CODE, BLOCK, LINE, STR, CHR } st = CODE;

	(void)snprintf(path, sizeof(path), "%s/%s", STS_APP_SRC_DIR, rel);
	f = fopen(path, "rb");
	TEST_ASSERT_NOT_NULL_MESSAGE(f, path);
	n = fread(g_src, 1U, sizeof(g_src) - 1U, f);
	(void)fclose(f);
	TEST_ASSERT_TRUE_MESSAGE(n < (sizeof(g_src) - 1U),
				 "source did not fit the scan buffer");
	TEST_ASSERT_TRUE_MESSAGE(n > 4096U, "source is implausibly small");
	g_src[n] = '\0';
	g_src_len = n;
	(void)snprintf(g_src_name, sizeof(g_src_name), "%s", rel);

	for (i = 0U; i < n; i++) {
		char c = g_src[i];
		char d = ((i + 1U) < n) ? g_src[i + 1U] : '\0';

		switch (st) {
		case CODE:
			if ((c == '/') && (d == '*')) {
				st = BLOCK;
				g_src[i] = ' ';
				g_src[i + 1U] = ' ';
				i++;
			} else if ((c == '/') && (d == '/')) {
				st = LINE;
				g_src[i] = ' ';
				g_src[i + 1U] = ' ';
				i++;
			} else if (c == '"') {
				st = STR;
			} else if (c == '\'') {
				st = CHR;
			}
			break;
		case BLOCK:
			g_src[i] = (c == '\n') ? '\n' : ' ';
			if ((c == '*') && (d == '/')) {
				g_src[i + 1U] = ' ';
				i++;
				st = CODE;
			}
			break;
		case LINE:
			if (c == '\n') {
				st = CODE;
			} else {
				g_src[i] = ' ';
			}
			break;
		case STR:
		case CHR:
			/* Kept verbatim; only the escape pair is stepped over so
			 * an escaped quote cannot end the literal early. */
			if (c == '\\') {
				i++;
				break;
			}
			if (((st == STR) && (c == '"')) ||
			    ((st == CHR) && (c == '\''))) {
				st = CODE;
			}
			break;
		}
	}
}

static unsigned int count_in(const span_t *s, const char *needle)
{
	size_t k = strlen(needle);
	unsigned int hits = 0U;
	size_t i;

	for (i = s->begin; (k != 0U) && ((i + k) <= s->end); i++) {
		if (memcmp(&g_src[i], needle, k) == 0) {
			hits++;
		}
	}
	return hits;
}

static size_t offset_in(const span_t *s, const char *needle)
{
	size_t k = strlen(needle);
	size_t i;
	char msg[256];

	for (i = s->begin; (i + k) <= s->end; i++) {
		if (memcmp(&g_src[i], needle, k) == 0) {
			return i;
		}
	}
	(void)snprintf(msg, sizeof(msg),
		       "%s: `%s` is gone — fix this scan, do not delete the "
		       "assertion it feeds",
		       g_src_name, needle);
	TEST_FAIL_MESSAGE(msg);
	return 0U;
}

/** The braced body of the function whose definition begins with @p sig. */
static span_t fn_body(const char *sig)
{
	size_t k = strlen(sig);
	span_t s = { 0U, 0U };
	size_t at = 0U;
	unsigned int hits = 0U;
	int depth = 0;
	size_t i;
	char msg[256];

	(void)snprintf(msg, sizeof(msg),
		       "%s: cannot locate exactly one body of `%s` — fix this "
		       "scan, do not delete the assertion it feeds",
		       g_src_name, sig);

	for (i = 0U; (i + k) <= g_src_len; i++) {
		if (memcmp(&g_src[i], sig, k) == 0) {
			hits++;
			at = i;
		}
	}
	TEST_ASSERT_EQUAL_UINT_MESSAGE(1U, hits, msg);

	while ((at < g_src_len) && (g_src[at] != '{')) {
		at++;
	}
	TEST_ASSERT_TRUE_MESSAGE(at < g_src_len, msg);
	s.begin = at + 1U;

	for (i = at; i < g_src_len; i++) {
		if (g_src[i] == '{') {
			depth++;
		} else if (g_src[i] == '}') {
			depth--;
			if (depth == 0) {
				s.end = i;
				break;
			}
		}
	}
	TEST_ASSERT_TRUE_MESSAGE(s.end > s.begin, msg);
	return s;
}

/* ============================================ the dispatch, as source read = */

/*
 * One entry per manifest object. Filled from the dispatch, never from the
 * manifest, so a mismatch is a real disagreement and not a tautology.
 */
#define OBJ_MAX 128U

static bool g_read_wired[OBJ_MAX];
static bool g_mut_wired[OBJ_MAX];

/** Mark the object named by every `strcmp(o->id, "…")` inside @p s. */
static unsigned int mark_strcmp_ids(const span_t *s, bool *set)
{
	static const char pat[] = "strcmp(o->id, \"";
	size_t k = sizeof(pat) - 1U;
	unsigned int marked = 0U;
	size_t i;

	for (i = s->begin; (i + k) < s->end; i++) {
		char id[64];
		size_t j = 0U;
		size_t p;
		int idx;
		char msg[320];

		if (memcmp(&g_src[i], pat, k) != 0) {
			continue;
		}
		for (p = i + k; (p < s->end) && (g_src[p] != '"'); p++) {
			TEST_ASSERT_TRUE_MESSAGE(j < (sizeof(id) - 1U),
						 "object id longer than the "
						 "manifest permits");
			id[j] = g_src[p];
			j++;
		}
		id[j] = '\0';

		idx = mp_obj_find(id);
		(void)snprintf(msg, sizeof(msg),
			       "%s dispatches on `%s`, which is not a manifest "
			       "object", g_src_name, id);
		TEST_ASSERT_TRUE_MESSAGE(idx >= 0, msg);
		TEST_ASSERT_TRUE((size_t)idx < OBJ_MAX);
		set[idx] = true;
		marked++;
	}
	return marked;
}

/**
 * Derive both wired sets from mp_glue.c and mp_rpc.c.
 *
 * Every non-`strcmp` route into the dispatch is asserted to still exist before
 * the objects it covers are marked, because each one covers a whole class:
 * dropping the MP_KIND_RAIL branch would silently un-wire nine rails, and
 * dropping the MP_OF_CFG branch would un-wire the two configured ceilings.
 */
static void derive_dispatch(void)
{
	span_t rd;
	span_t ap;
	span_t pu;
	size_t i;

	memset(g_read_wired, 0, sizeof(g_read_wired));
	memset(g_mut_wired, 0, sizeof(g_mut_wired));
	TEST_ASSERT_TRUE(mp_obj_count() <= OBJ_MAX);

	load_source("zephyr/console/mp_glue.c");

	/* ---------------------------------------------------------- reads -- */
	rd = fn_body("static int obj_read(void *user, size_t obj, mp_val_t *out)");
	(void)mark_strcmp_ids(&rd, g_read_wired);

	/* Kind- and flag-keyed branches, each covering a class of objects. */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&rd, "o->kind == (uint8_t)MP_KIND_RAIL"),
		"obj_read() no longer serves the INA228 rails by kind");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&rd, "(o->flags & MP_OF_CFG) != 0U"),
		"obj_read() no longer serves cfg-backed objects by flag");
	for (i = 0U; i < mp_obj_count(); i++) {
		const mp_obj_t *o = mp_obj_at(i);

		if ((o->kind == (uint8_t)MP_KIND_RAIL) ||
		    ((o->flags & MP_OF_CFG) != 0U)) {
			g_read_wired[i] = true;
		}
	}

	/*
	 * The two grouped blocks answer -EIO, not -ENOTSUP, when their snapshot
	 * has not landed. That is what makes "not deferred" mean "wired" at
	 * runtime and not merely in the table: a thermometer behind a busy I2C
	 * bus must not report itself as a feature the board does not have.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		3U, count_in(&rd, "return have ? 0 : -EIO;"),
		"obj_read()'s health/quality/pwrseq blocks no longer "
		"distinguish `no reading yet` from `no such feature`");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&rd, "return -ENOTSUP;"),
		"obj_read() has more than one unwired exit; the deferred set "
		"can no longer be derived from its dispatch");

	/* ------------------------------------------------------ mutations -- */
	ap = fn_body("static int obj_apply(void *user, size_t obj, const int32_t *value)");
	(void)mark_strcmp_ids(&ap, g_mut_wired);

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&ap, "sts_recov_bool_action(o->id)"),
		"obj_apply() no longer routes through the recovery policy");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&ap, "return -ENOTSUP;"),
		"obj_apply() has more than one unwired exit");
	/*
	 * The release short-circuit must stay BELOW every id branch. Above them
	 * it would answer 0 for a release of an object nothing implements, which
	 * is harmless, and — far from harmless — it would swallow the releases
	 * that DO something: `sys.wdt.en` re-arms the external watchdog on
	 * release and `ref.mux.sel` puts the pre-lease reference request back,
	 * and neither would ever run again.
	 *
	 * The needle is the short-circuit's whole shape rather than the bare
	 * `if (value == NULL)`, because those two branches now test the release
	 * direction inside themselves — that is what a protected release IS —
	 * and the first textual match would otherwise be one of them. Pinned to
	 * exactly one occurrence so the specificity cannot be traded for a
	 * second, earlier copy.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U,
		count_in(&ap, "if (value == NULL) {\n\t\treturn 0;\n\t}\n"
			      "\treturn -ENOTSUP;"),
		"obj_apply()'s release short-circuit changed shape; re-read "
		"what a release now has to reach before relaxing this");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&ap, "sts_recov_bool_action(o->id)") <
			offset_in(&ap, "if (value == NULL) {\n\t\treturn 0;\n\t}"
					"\n\treturn -ENOTSUP;"),
		"obj_apply()'s release short-circuit moved above the dispatch");

	pu = fn_body("static int obj_pulse(void *user, size_t obj, uint32_t ms)");
	(void)mark_strcmp_ids(&pu, g_mut_wired);
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&pu, "sts_recov_pulse_action(o->id)"),
		"obj_pulse() no longer routes through the recovery policy");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&pu, "case STS_RECOV_POE_KILL:"),
		"obj_pulse() no longer handles the board cold-cycle");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&pu, "case STS_RECOV_RB_OV_RESET:"),
		"obj_pulse() no longer handles the rubidium OV latch reset");

	/* The policy tables themselves, compiled and called — not read. */
	for (i = 0U; i < mp_obj_count(); i++) {
		const char *id = mp_obj_at(i)->id;

		if (sts_recov_bool_action(id) != STS_RECOV_NONE) {
			g_mut_wired[i] = true;
		}
		if (sts_recov_pulse_action(id) != STS_RECOV_NONE) {
			g_mut_wired[i] = true;
		}
	}

	/*
	 * cfg-backed writes do not reach obj_apply() at all: mp_rpc.c's
	 * m_obj_set() stages them through cfg_write() and commits. That is a
	 * wired mutation and the manifest must not call it deferred.
	 */
	load_source("core/mp/mp_rpc.c");
	{
		span_t set = fn_body("static int m_obj_set(mp_ctx_t *c, const "
				     "mp_json_t *p, int params, mp_jw_t *w)");

		TEST_ASSERT_EQUAL_UINT_MESSAGE(
			1U, count_in(&set, "rc = cfg_write(c, o, res.value);"),
			"m_obj_set() no longer writes cfg-backed objects "
			"through cfg_write()");
	}
	for (i = 0U; i < mp_obj_count(); i++) {
		const mp_obj_t *o = mp_obj_at(i);

		if (((o->flags & MP_OF_CFG) != 0U) &&
		    ((o->flags & MP_OF_WRITE) != 0U)) {
			g_mut_wired[i] = true;
		}
	}
}

/** True when @p o declares any mutating operation. */
static bool mutable_obj(const mp_obj_t *o)
{
	return (o->flags & (MP_OF_WRITE | MP_OF_OVERRIDE | MP_OF_PULSE)) != 0U;
}

/* ================================================== 1. the contract itself = */

/**
 * THE ASSERTION THE WHOLE SUITE EXISTS FOR.
 *
 * For every object: MP_OF_DEFERRED is set exactly when the operation that
 * object exists for is missing from the dispatch. A mutable object is judged on
 * its mutation, because that is the operation whose absence a technician
 * discovers by acting on the board; a read-only object is judged on its read.
 */
static void test_the_deferred_flag_is_the_dispatch(void)
{
	/* The objects mp_glue.c still does not dispatch, by name. See the
	 * argument below for why this replaced a numeric floor. */
	static const char *const still_deferred[] = {
		"gnss.dsel",         "gnss.reset",      "gnss.safeboot",
		"pwr.rb.en",         "pwr.rb.pot.code", "ref.relay.hold",
		"sensor.buttons",    "sensor.en.fault", "sensor.enc.pos",
		"sensor.gnss.ant",   "sensor.gps.ant_off",
		"sensor.gps.txrdy",  "sensor.ina.alert", "sensor.pfi",
		"sensor.pg",         "sensor.poe.status",
		"sensor.usb.vbus",   "sys.nor.reset",   "sys.smp.tunnel",
		"ui.disp.reset",
	};
	size_t i;
	size_t k;
	unsigned int deferred = 0U;
	unsigned int wired = 0U;

	derive_dispatch();

	for (i = 0U; i < mp_obj_count(); i++) {
		const mp_obj_t *o = mp_obj_at(i);
		bool expect = mutable_obj(o) ? !g_mut_wired[i]
					     : !g_read_wired[i];
		bool have = (o->flags & MP_OF_DEFERRED) != 0U;
		char msg[256];

		(void)snprintf(msg, sizeof(msg),
			       "`%s`: manifest says %s, mp_glue.c's dispatch "
			       "says %s. Whichever changed, the other has to.",
			       o->id, have ? "deferred" : "wired",
			       expect ? "deferred" : "wired");
		TEST_ASSERT_EQUAL_MESSAGE(expect, have, msg);

		if (have) {
			deferred++;
		} else {
			wired++;
		}
	}

	/*
	 * Not a vacuous pass: both sets have to be non-trivial, or a broken
	 * derivation that marked everything (or nothing) would agree with a
	 * manifest that did the same.
	 *
	 * The deferred side used to be a floor of 20. That number was written
	 * when the set was large and it has been shrinking with every wiring
	 * stage; wiring the OCXO steering group took it to exactly 20 and the
	 * floor started refusing the very outcome the work was for. A floor that
	 * has to be edited each time the thing it measures improves is not
	 * measuring anything, so it is replaced by the SET ITSELF, by name.
	 *
	 * That is strictly stronger than the inequality it replaces: it fails on
	 * an object that quietly becomes deferred again (an un-wiring) as well as
	 * on one that is wired without the list being updated, and it says which.
	 * Every member is deferred for a reason recorded in mp_glue.c's header —
	 * no accessor exists (`sys.nor.reset`, `gnss.dsel`), the interlock refuses
	 * it on purpose (`pwr.rb.pot.code`), the 1 kHz scan is not published
	 * object-by-object (the `sensor.*` group), or the seam is still open
	 * (`pwr.rb.en`, `ref.relay.hold`, `sys.smp.tunnel`, `gnss.reset`,
	 * `gnss.safeboot`, `ui.disp.reset`).
	 */
	for (k = 0U; k < (sizeof(still_deferred) / sizeof(still_deferred[0]));
	     k++) {
		int idx = mp_obj_find(still_deferred[k]);
		char msg[192];

		(void)snprintf(msg, sizeof(msg),
			       "`%s` is listed as still deferred but the "
			       "manifest no longer says so; shrink the list "
			       "rather than leaving it stale",
			       still_deferred[k]);
		TEST_ASSERT_TRUE_MESSAGE(idx >= 0, still_deferred[k]);
		TEST_ASSERT_TRUE_MESSAGE(
			(mp_obj_at((size_t)idx)->flags & MP_OF_DEFERRED) != 0U,
			msg);
	}
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		(unsigned int)(sizeof(still_deferred) /
			       sizeof(still_deferred[0])),
		deferred,
		"an object became deferred without joining this list");

	TEST_ASSERT_TRUE_MESSAGE(wired > 20U,
				 "the derivation marked almost everything "
				 "deferred");
	TEST_ASSERT_EQUAL_UINT((unsigned int)mp_obj_count(), deferred + wired);
}

/**
 * mp_glue.c's header enumerates the wired set. This is that enumeration, proved.
 *
 * The header comment says "Thirty-one manifest objects therefore have an
 * actuator behind them" and lists them by name, grouped by the seam each one
 * reaches. That claim rotted once already: it read "Twenty-eight" and omitted
 * `ref.mux.sel`, `sys.wdt.en` and `sys.wdt.kick` long after all three had live
 * branches in obj_apply()/obj_pulse() a few hundred lines below it. Nothing
 * failed, because no test had ever read the number — and a stale count in the
 * one file a reviewer trusts to describe the seam is how wrong numbers reach the
 * docs that quote it.
 *
 * A BARE COUNT WOULD NOT HAVE HELPED, and this file has already learnt that: the
 * deferred side's numeric floor of 20 is the cautionary tale recorded in
 * test_the_deferred_flag_is_the_dispatch(), where a number that had to be edited
 * every time the thing it measured improved was replaced by the set itself. A
 * count also passes on any equal-and-opposite edit — wire one object, un-wire
 * another — while saying nothing about which. So this is the SET, by name, exact
 * in both directions, derived from the dispatch by derive_dispatch() and never
 * from the prose.
 *
 * load_source() blanks comments before the scan, so the header's own list cannot
 * satisfy this test: the ids below have to be matched by real `strcmp(o->id,
 * "…")` branches, by a compiled recovery-policy table, or by the cfg flags.
 * The test therefore cannot check the header's WORDING — only that the set it
 * describes is the set that exists. If this fails, fix mp_glue.c's header in the
 * same change.
 */
static void test_the_wired_set_is_the_one_the_header_claims(void)
{
	/* Grouped and ordered as mp_glue.c's header groups them, so the two read
	 * against each other line by line. */
	static const char *const wired[] = {
		/* obj_apply, this area's own actuators */
		"ui.identify", "ref.rb.serial", "gnss.tunnel", "ref.rb.tunnel",
		"sys.wdt.en", "ref.mux.sel",
		/* obj_apply, posted to the sequencer mailbox */
		"ui.panel.duty", "pwr.panel.led.en", "pwr.gps.en",
		"pwr.ant.bias.en", "pwr.disp.en", "pwr.rb.gate",
		"pwr.rb.vset_mv", "ui.lamp.test", "ref.term.en", "sys.fan.duty",
		"ui.rgb.mode", "ui.rgb.r", "ui.rgb.g", "ui.rgb.b", "ui.disp.bl",
		"ref.disc.park", "ref.ocxo.vc_mv", "ref.ocxo.dac_code",
		/* obj_pulse */
		"pwr.poe.kill", "pwr.rb.ov.reset", "sys.phy.reset",
		"gnss.extint", "sys.wdt.kick",
		/* cfg_write, in mp_rpc.c rather than mp_glue.c */
		"pwr.rb.vmax_mv", "pwr.poe.budget_mw",
	};
	const size_t n = sizeof(wired) / sizeof(wired[0]);
	size_t i;
	size_t k;
	unsigned int found = 0U;

	derive_dispatch();

	for (i = 0U; i < mp_obj_count(); i++) {
		const mp_obj_t *o = mp_obj_at(i);
		bool listed = false;
		char msg[224];

		if (!mutable_obj(o) || !g_mut_wired[i]) {
			continue;
		}
		found++;
		for (k = 0U; k < n; k++) {
			if (strcmp(o->id, wired[k]) == 0) {
				listed = true;
			}
		}
		(void)snprintf(msg, sizeof(msg),
			       "`%s` gained an actuator but is missing from this "
			       "list and from mp_glue.c's header enumeration; "
			       "update both, including the count",
			       o->id);
		TEST_ASSERT_TRUE_MESSAGE(listed, msg);
	}

	/* The other direction, and the one a count alone would miss: a name here
	 * that the dispatch no longer reaches. */
	for (k = 0U; k < n; k++) {
		int idx = mp_obj_find(wired[k]);
		char msg[224];

		(void)snprintf(msg, sizeof(msg),
			       "`%s` is listed as wired but the dispatch no "
			       "longer reaches it; shrink this list and "
			       "mp_glue.c's header rather than leaving them "
			       "stale",
			       wired[k]);
		TEST_ASSERT_TRUE_MESSAGE(idx >= 0, wired[k]);
		TEST_ASSERT_TRUE_MESSAGE(g_mut_wired[(size_t)idx], msg);
	}

	/* Duplicates would let a dropped name hide inside the total. */
	for (k = 0U; k < n; k++) {
		size_t j;

		for (j = 0U; j < k; j++) {
			TEST_ASSERT_TRUE_MESSAGE(
				strcmp(wired[j], wired[k]) != 0, wired[k]);
		}
	}

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		(unsigned int)n, found,
		"the wired count changed; mp_glue.c's header says how many "
		"objects have an actuator behind them and that number is now "
		"wrong");
}

/**
 * The set of objects whose mutation works but whose read does not.
 *
 * One bit cannot say "actuates but cannot be read back", so the manifest's bit
 * describes the mutation and this residue is where it under-reports. Pinning
 * the residue by name is what keeps it from growing: each of the six has a
 * structural reason for having nothing to read — `ui.identify` is a write-only
 * beacon, and the other five are momentary pulses whose pin rests deasserted,
 * so the only thing a read could report is "not pulsing right now" — and a
 * seventh would mean somebody wired a setter without a read-back and let the
 * flag quietly become less true.
 *
 * `sys.phy.reset` and `gnss.extint` joined the pulse group when obj_pulse()
 * gained their branches. Neither has a rest state worth publishing: LAN_RST_N
 * is released within 1.5 ms of the call returning, and GPS_EXTINT is a
 * 1 ms edge whose EFFECT is reported by the receiver as UBX-TIM-TM2, not by
 * this seam.
 *
 * `sys.wdt.kick` joined for the same reason and is the clearest case of it:
 * WDI rests low between edges, so a read of PB2 would report "not pulsing"
 * forever. What a technician actually needs to confirm before pulsing it is
 * `sys.wdt.en` — the watchdog is not watching — and that object DOES read back.
 */
static void test_the_write_only_residue_is_exactly_these_five(void)
{
	static const char *const expect[] = {
		"ui.identify",
		"pwr.poe.kill",
		"pwr.rb.ov.reset",
		"sys.phy.reset",
		"gnss.extint",
		"sys.wdt.kick",
	};
	size_t i;
	unsigned int found = 0U;

	derive_dispatch();

	for (i = 0U; i < mp_obj_count(); i++) {
		const mp_obj_t *o = mp_obj_at(i);
		bool listed = false;
		size_t k;

		if (!mutable_obj(o) || !g_mut_wired[i] || g_read_wired[i]) {
			continue;
		}
		found++;
		for (k = 0U; k < (sizeof(expect) / sizeof(expect[0])); k++) {
			if (strcmp(o->id, expect[k]) == 0) {
				listed = true;
			}
		}
		TEST_ASSERT_TRUE_MESSAGE(
			listed,
			"a wired control gained no read-back; either wire "
			"obj_read() for it or add it to this list and to "
			"mp_manifest.h's note");
	}
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		(unsigned int)(sizeof(expect) / sizeof(expect[0])), found,
		"one of the named write-only objects now reads back; "
		"shrink the list rather than leaving it stale");
}

/**
 * THE CONSOLE NEVER WRITES PA4.
 *
 * Spec §3 makes the discipline thread the only writer of DAC1_OUT1 and the
 * owner of disc_ctx_t. `ref.disc.park`, `ref.ocxo.vc_mv` and `ref.ocxo.dac_code`
 * are the three objects that could break that, and the whole reason they are
 * mailbox rows rather than direct calls is that a console thread reaching the
 * actuator "works" on a bench every time and corrupts the loop only when the
 * two threads happen to interleave.
 *
 * Asserted as an ABSENCE over the whole of mp_glue.c, not just obj_apply(): a
 * helper called from obj_apply() would move the defect one frame down and pass
 * a body-scoped check. mp_glue.c is not linked by any host suite, so this reads
 * the source — the same technique the rest of this file uses.
 *
 * load_source() blanks comments before the scan, so the prose in mp_glue.c that
 * names these functions cannot satisfy or trip the assertions below.
 */
static void test_the_console_never_writes_the_dac(void)
{
	static const char *const forbidden[] = {
		"dac_write_value(",  /* the Zephyr DAC driver itself */
		"disc_write_dac(",   /* the discipline thread's own writer */
		"disc_apply_auto_dac(",
		"disc_park(",        /* mutating the loop's state machine */
		"disc_unpark(",
		"sts_discipline_park(",
		"sts_disc_handoff_park(",
	};
	span_t all;
	span_t ap;
	size_t k;

	load_source("zephyr/console/mp_glue.c");
	all.begin = 0U;
	all.end = g_src_len;

	for (k = 0U; k < (sizeof(forbidden) / sizeof(forbidden[0])); k++) {
		char msg[224];

		(void)snprintf(msg, sizeof(msg),
			       "mp_glue.c calls `%s`; the console thread has "
			       "reached the discipline loop's actuator and PA4 "
			       "now has two writers",
			       forbidden[k]);
		TEST_ASSERT_EQUAL_UINT_MESSAGE(0U, count_in(&all, forbidden[k]),
					       msg);
	}

	/* What it does instead: posts, and answers PENDING. Three objects, one
	 * post each — a shared post would make the outcome half attribute a
	 * settle or a veto to whichever lease it guessed. */
	ap = fn_body("static int obj_apply(void *user, size_t obj, const int32_t *value)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&ap, "STS_PWRSEQ_REQ_DISC_PARK"),
		"`ref.disc.park` no longer posts exactly one request");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&ap, "STS_PWRSEQ_REQ_OCXO_VC_MV"),
		"`ref.ocxo.vc_mv` no longer posts exactly one request");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&ap, "STS_PWRSEQ_REQ_OCXO_DAC_CODE"),
		"`ref.ocxo.dac_code` no longer posts exactly one request");
}

/**
 * ...and the discipline thread is where the actuation actually happens.
 *
 * The other half of the assertion above: proving mp_glue.c does not write PA4
 * is worth nothing if nobody else does either, which would make the objects
 * silently inert. The drain claims each row, and the Vc executor is the one
 * place disc_write_dac() is reached from a request.
 */
static void test_the_discipline_thread_drains_them(void)
{
	span_t mb;
	span_t dac;
	span_t park;
	span_t gate;
	span_t loop;

	load_source("zephyr/platform/disc_thread.c");

	mb = fn_body("static void disc_service_mailbox(void)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&mb, "STS_PWRSEQ_REQ_DISC_PARK"),
		"the discipline drain no longer claims the park row");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&mb, "STS_PWRSEQ_REQ_OCXO_VC_MV"),
		"the discipline drain no longer claims the Vc millivolt row");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&mb, "STS_PWRSEQ_REQ_OCXO_DAC_CODE"),
		"the discipline drain no longer claims the Vc code row");
	/*
	 * ORDER: both Vc rows are claimed before the park row. A lapse can drop
	 * a Vc lease and the park lease into the same window; taken park-first
	 * the release would withdraw a Vc lease that was expiring on its own and
	 * tell the technician it had been revoked.
	 */
	/*
	 * Anchored on the EXECUTOR CALLS, not on the row constants. The row ids
	 * also appear in the vc_rows[] initializer at the top of the body, which
	 * does not move when the claim loop does — an earlier version of this
	 * assertion compared those and passed with the order reversed.
	 */
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&mb, "disc_service_dac_row(") <
			offset_in(&mb, "disc_service_park_row("),
		"the park row is now drained before the Vc rows, so a lapse "
		"withdraws a lease that was expiring anyway");

	/*
	 * THE RESUME GATE READS BOTH LATCHES.
	 *
	 * There are two independent holders of the parked state — the PFI
	 * power-fail latch and the operator's lease — and each clears only its
	 * own. A gate that consulted one of them would let a PFI recovery resume
	 * the loop under a technician's Vc override, and let a maintenance
	 * release resume it into a browning-out rail. Both directions are the
	 * same one-line fact, which is why it is a gate and not two open-coded
	 * checks.
	 */
	gate = fn_body("static bool disc_resume_if_idle(void)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&gate, "disc_pfi_parked"),
		"the resume gate stopped consulting the PFI latch, so a "
		"maintenance release now defeats a power-fail park");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&gate, "disc_maint_parked"),
		"the resume gate stopped consulting the maintenance latch, so "
		"a PFI recovery now resumes the loop under a Vc override");

	/* The Vc executor writes the pin; the park executor never does. */
	dac = fn_body("static void disc_service_dac_row(uint8_t row, bool release, int32_t value,");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&dac, "disc_write_dac("),
		"the Vc drain no longer drives DAC1_OUT1");
	/*
	 * The park is re-checked AT THE ACTUATOR, not only at the interlock.
	 * MP_ILK_DAC_PARK is evaluated on the console thread at grant time;
	 * between that and this drain a refsel handoff or a PFI recovery can
	 * have resumed the loop, and a Vc write into a running loop is a control
	 * that reports success and is overwritten within the second.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&dac, "DISC_STATE_PARKED"),
		"the Vc drain trusts the console-side interlock and no longer "
		"re-checks that the loop is still parked when it executes");

	park = fn_body("static void disc_service_park_row(bool release, int32_t value)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&park, "disc_park("),
		"the park drain no longer parks the loop");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&park, "disc_resume_if_idle("),
		"the park drain resumes without consulting the other latch");
	/*
	 * THE OVERRIDE MUST STICK. A parked loop re-emits its frozen code every
	 * second, so the per-second writer has to be the auto path — which
	 * stands aside while an override is held — and never disc_write_dac()
	 * directly. This is the assertion that fails if anyone "simplifies" the
	 * split away, and the symptom in the field would be a Vc that reverts
	 * about one second after it is set.
	 */
	loop = fn_body("static void disc_entry(void *p1, void *p2, void *p3)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&loop, "disc_write_dac("),
		"the discipline loop writes the DAC directly again, so a "
		"maintenance override is undone within one PPS period");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&loop, "disc_apply_auto_dac("),
		"the discipline loop no longer drives the actuator at all");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&loop, "disc_service_mailbox()"),
		"the discipline loop no longer drains its mailbox, so the "
		"three objects are posted and never executed");
}

/** Every object the dispatch names is a manifest object, and vice versa. */
static void test_the_dispatch_names_nothing_the_manifest_lacks(void)
{
	span_t b;
	unsigned int n;

	/* mark_strcmp_ids() fails on an unknown id, so simply running it over
	 * each body is the assertion; the counts below stop it passing because
	 * it found nothing to check. */
	load_source("zephyr/console/mp_glue.c");

	b = fn_body("static int obj_read(void *user, size_t obj, mp_val_t *out)");
	n = mark_strcmp_ids(&b, g_read_wired);
	TEST_ASSERT_TRUE_MESSAGE(n >= 20U,
				 "obj_read()'s id dispatch shrank drastically; "
				 "re-derive the deferred set by hand before "
				 "trusting this suite");

	b = fn_body("static int obj_apply(void *user, size_t obj, const int32_t *value)");
	n = mark_strcmp_ids(&b, g_mut_wired);
	TEST_ASSERT_TRUE_MESSAGE(n >= 4U, "obj_apply()'s id dispatch shrank");
}

/* ============================================ 2. the flag is load-bearing == */

/**
 * The shipped glue answers `obj_supported` out of the flag, and the RPC layer
 * consults it BEFORE the guard.
 *
 * This is the half a behavioural test cannot reach: the host suites wire their
 * own complete apply, so `obj_supported` is NULL there by design and every
 * object actuates. What has to be true of the IMAGE is that the hook exists, is
 * bound, and reads the manifest bit rather than a second list of ids.
 */
static void test_the_image_gates_actuation_on_the_flag(void)
{
	span_t b;
	span_t st;

	load_source("zephyr/console/mp_glue.c");

	b = fn_body("static int prov_obj_supported(void *user, size_t obj)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "MP_OF_DEFERRED"),
		"prov_obj_supported() no longer answers from the manifest "
		"flag, so the flag and the gate are now two truths");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "-ENOTSUP"),
		"prov_obj_supported() no longer answers -ENOTSUP for a "
		"deferred object");

	st = fn_body("int sts_mp_start(void)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&st, "w.obj_supported = prov_obj_supported;"),
		"the image no longer binds obj_supported, so an unimplemented "
		"G3 object burns the arm again");
}

/**
 * `obj.set`, `obj.override` and `obj.pulse` each ask before the guard.
 *
 * The ordering IS the fix. mp_ovr_guard() consumes the G3 arming nonce on a
 * successful second phase, so a check placed after it spends the typed phrase
 * and the hold on an object that does nothing — and the arm cannot be handed
 * back.
 */
static void test_the_rpc_asks_before_the_guard(void)
{
	static const char *const sigs[] = {
		"static int m_obj_set(mp_ctx_t *c, const mp_json_t *p, int "
		"params, mp_jw_t *w)",
		"static int m_obj_override(mp_ctx_t *c, const mp_json_t *p, "
		"int params,",
		"static int m_obj_pulse(mp_ctx_t *c, const mp_json_t *p, int "
		"params, mp_jw_t *w)",
	};
	size_t i;

	load_source("core/mp/mp_rpc.c");

	for (i = 0U; i < (sizeof(sigs) / sizeof(sigs[0])); i++) {
		span_t b = fn_body(sigs[i]);

		TEST_ASSERT_EQUAL_UINT_MESSAGE(
			1U, count_in(&b, "impl_or_fail(c, (size_t)idx)"),
			"a mutating RPC no longer asks whether the object is "
			"implemented");
		TEST_ASSERT_TRUE_MESSAGE(
			offset_in(&b, "impl_or_fail(c, (size_t)idx)") <
				offset_in(&b, "guard_or_fail("),
			"the implemented-check moved below the guard, so a G3 "
			"object burns its arm before admitting it does nothing");
	}
}

/**
 * -ENOTSUP out of apply is not a veto.
 *
 * Behaviourally proved in test_mp_override.c; read here so the two claims that
 * cannot be separated behaviourally are both pinned — that the event is not
 * emitted and that the counter does not move. A future edit that folded
 * -ENOTSUP back into the -EACCES path would still pass a test which only
 * checked the return code.
 */
static void test_notsup_leaves_the_veto_machinery_alone(void)
{
	span_t b;
	size_t at;

	load_source("core/mp/mp_override.c");
	b = fn_body("int mp_ovr_grant(mp_ovr_ctx_t *c, size_t obj, int32_t "
		    "value, uint32_t ttl_ms,");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "if (rc == -ENOTSUP) {"),
		"mp_ovr_grant() no longer separates `nothing is wired` from a "
		"safety veto");
	at = offset_in(&b, "if (rc == -ENOTSUP) {");
	TEST_ASSERT_TRUE_MESSAGE(
		at < offset_in(&b, "c->vetoes++;"),
		"the -ENOTSUP exit moved below the veto counter");
	TEST_ASSERT_TRUE_MESSAGE(
		at < offset_in(&b, "MP_OVR_EV_VETO"),
		"the -ENOTSUP exit moved below the veto event");
}

/* ==================================== 3. the interlocks the flag sits beside */

/**
 * MP_ILK_MUX_GUARD's two terms come from two different signals.
 *
 * They were both derived from `quality_block_t::active_ref` — the reference the
 * discipline loop had SELECTED — so `extref_ok && rb_lock` demanded one field
 * hold two mutually exclusive values and `ref.mux.sel = 1` was unsatisfiable by
 * construction. The fix is that they now read RB_LOCK (PB13) and EXTREF_MON
 * (PB14/TIM12) out of the sequencer's published OBSERVED half, which is what
 * spec §3.5 names for the handoff.
 */
static void test_the_mux_guard_reads_two_hardware_signals(void)
{
	span_t b;

	load_source("zephyr/console/mp_glue.c");
	b = fn_body("static int prov_ilk(void *user, mp_ilk_state_t *out)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "out->rb_lock = ps.rb_lock_pin;"),
		"rb_lock no longer reads RB_LOCK");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "out->extref_ok = ps.extref_valid && "
				 "ps.extref_in_band;"),
		"extref_ok no longer reads EXTREF_MON");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "q.active_ref"),
		"an interlock term is derived from the loop's selected "
		"reference again; that is the defect, not a shortcut");
}

/**
 * The interlock terms this area still cannot measure say which they are.
 *
 * `rb_code_min > rb_code_max` and `liveness_ok = false` are both permanent
 * refusals, and the audit's question was whether each is a deliberate fail-safe
 * or a stub nobody had labelled. The answers are written into mp_glue.c beside
 * the code; this asserts the label exists, so a later reader is not left
 * guessing again — and, for the display, that the stub is GONE rather than
 * relabelled.
 */
static void test_the_permanent_refusals_are_labelled(void)
{
	span_t b;

	load_source("zephyr/console/mp_glue.c");
	b = fn_body("static int prov_ilk(void *user, mp_ilk_state_t *out)");

	/* The raw wiper code: refused on purpose, and it says so. */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "out->rb_code_min = 1U;"),
		"the digipot window changed shape; re-read the refusal's "
		"justification before trusting it");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(1U, count_in(&b, "out->rb_code_max = 0U;"),
				       "the digipot window is no longer "
				       "inverted, so a raw wiper code is now "
				       "accepted against an uncomputed bound");

	/* The watchdog liveness gate: still unobservable, still refusing. */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "out->liveness_ok = false;"),
		"the watchdog liveness term changed; it may only be reported "
		"true from a real measurement");

	/*
	 * The two Vc leases: read from the LEASE REGISTER, which is the fact the
	 * interlock asks about, and never from the pin.
	 *
	 * Not the same shortcut MP_ILK_MUX_GUARD was corrected for. That
	 * interlock asks a question about HARDWARE and was answered with the
	 * loop's own choice between two references. MP_ILK_DAC_SOLE and
	 * MP_ILK_DAC_IDLE ask whether a LEASE is held, and mp_ovr_lease() is
	 * where that lives. sts_disc_dac_state()'s `overridden` bit is downstream
	 * of it and lags by a drain, so reading the pin here would let a second
	 * grant through the window between a grant and the discipline thread's
	 * next pass — which is precisely the pair of writers the bit forbids.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		2U, count_in(&b, "mp_ovr_lease(&mp.ovr,"),
		"the OCXO Vc interlock terms stopped reading the lease "
		"register; a derived or lagging value must not stand in for it");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "sts_disc_dac_state("),
		"prov_ilk() is judging the Vc interlocks on the published pin "
		"state, which lags the lease it is supposed to be checking");

	/* The display off-time: no longer a stub — a real observation. */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "out->disp_on = disp_seen && disp_on_last;"),
		"disp_on stopped being an observation");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "out->disp_changed_ms = (uint32_t)"
				 "k_uptime_get_32();"),
		"the display change stamp is being re-taken as `now` on every "
		"evaluation again, which makes the off-time interlock refuse "
		"forever");
}

/** The change stamp is fed on every console pass, not only at request time. */
static void test_the_display_stamp_is_sampled_by_the_tick(void)
{
	span_t b;

	load_source("zephyr/console/mp_glue.c");
	b = fn_body("void sts_mp_tick(void)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "disp_observe_locked(&ps)"),
		"the tick no longer samples the display rail, so the off-time "
		"interlock has no history to measure against");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&b, "k_mutex_lock(&mp_lock") <
			offset_in(&b, "disp_observe_locked(&ps)"),
		"the display sample moved outside the engine lock it shares "
		"with prov_ilk()");
}

/* ============================================= 4. the tunnel is leased only */

/**
 * The K1 relay is wired, and to the setter that already existed.
 *
 * `sts_rb_serial_set_mode()` has been in sts_app.h with matching 0/1 semantics
 * the whole time and obj_apply() simply never called it, so the one control a
 * technician needs for the FE-5680A commissioning item answered "not
 * supported". Read rather than asserted behaviourally because mp_glue.c is not
 * linked by any host suite.
 */
static void test_the_rb_serial_relay_is_wired(void)
{
	span_t b;

	load_source("zephyr/console/mp_glue.c");
	b = fn_body("static int obj_apply(void *user, size_t obj, const int32_t *value)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_rb_serial_set_mode(mode)"),
		"`ref.rb.serial` no longer moves the K1 relay");
	/*
	 * A release must land on RS-232 — the relay's reset position and the
	 * documented fail-safe — so a lapsed lease cannot leave the port in the
	 * commissioning position nobody chose.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U,
		count_in(&b, "uint8_t mode = ((value != NULL) && (*value != 0)) "
			     "? 1U : 0U;"),
		"the K1 release no longer resolves to the RS-232 fail-safe");
}
/**
 * The VCC_RB ceiling the interlock clamps against is ASKED OF the sequencer.
 *
 * `prov_ilk()` filled `mp_ilk_state_t::rb_vmax_mv` from cfg `pwr.rb.vmax.mv`,
 * whose schema range is 4510..24450 mV, while the rail is actually held to
 * pwrseq_cfg_t::rb_vmax_mv. Re-applying STS_RB_VMAX_MV_CEILING here closed the
 * arm of sts_rb_vmax_decide() that LIED — an override to 20000 mV against a
 * 22000 mV cfg ceiling answered `value: 20000, clamped: false`, driven to
 * <=15000 by the drain and auto-reverted 250 ms later by MP_ILK_RB_VERIFY as
 * "VCC_RB out of window", a reply that told a technician his setpoint took
 * followed by a rail fault that did not exist.
 *
 * It could not close the other arm. decide() also REFUSES a ceiling below the
 * fixed operating setpoint and keeps pwrseq's own default, which is HIGHER than
 * cfg; a console quoting the constant then clamps tighter than the drain and
 * quotes a technician a ceiling lower than the board would take. Safe, still
 * untrue, and unmodellable from here — `default_mv` and `operating_mv` are
 * pwrseq internals. So the console stopped deriving the number and started
 * reading it.
 *
 * This half pins the CONSUMER: prov_ilk() takes the ceiling from
 * sts_pwrseq_rb_vmax_mv() and from nowhere else — no cfg key, no constant. The
 * producer is pinned separately below, so that Unity's abort-at-first-failure
 * cannot let one half hide the other.
 *
 * READ, NOT EXECUTED, for the reason the whole back half of this file gives:
 * mp_glue.c is linked by no host suite. The scan asserts the text of the fix;
 * it does not run it. It is paired with the two properties that keep the clamp
 * meaningful — that the hardware ceiling really does sit inside the published
 * envelope, so an in-force value can genuinely differ from what the manifest
 * advertises, and that it fits the u16 field it is narrowed into.
 */
static void test_the_rb_ceiling_is_the_one_in_force(void)
{
	span_t b;
	int vset = mp_obj_find("pwr.rb.vset_mv");

	/* The clamp is not a no-op: the hardware ceiling sits strictly inside
	 * the envelope the manifest publishes, so there is a band of cfg values
	 * for which the two numbers genuinely differ. */
	TEST_ASSERT_TRUE(vset >= 0);
	TEST_ASSERT_TRUE_MESSAGE(
		(int32_t)STS_RB_VMAX_MV_CEILING <
			mp_obj_at((size_t)vset)->max,
		"the hardware ceiling no longer bites inside the published "
		"VCC_RB envelope — this guard would pass vacuously");
	TEST_ASSERT_TRUE_MESSAGE(
		(int32_t)STS_RB_VMAX_MV_CEILING >=
			mp_obj_at((size_t)vset)->min,
		"the hardware ceiling fell below the setpoint floor; "
		"MP_ILK_RB_VMAX now refuses every request");
	TEST_ASSERT_TRUE(STS_RB_VMAX_MV_CEILING <= UINT16_MAX);

	load_source("zephyr/console/mp_glue.c");
	b = fn_body("static int prov_ilk(void *user, mp_ilk_state_t *out)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "uint32_t vmax = sts_pwrseq_rb_vmax_mv();"),
		"prov_ilk() no longer takes the VCC_RB ceiling from the "
		"sequencer that is enforcing it");
	/*
	 * And calls it exactly ONCE. Zephyr's MIN() is
	 * `(((a) < (b)) ? (a) : (b))`, so passing the call as `a` evaluates it
	 * twice — the comparison reads the sequencer, the result reads it again.
	 * `pwrseq_started` flips once, from the bring-up thread, while prov_ilk()
	 * runs on the console thread; two reads straddling that instant publish
	 * a ceiling neither read returned. Verified against the shipped ELF: the
	 * un-hoisted form really did emit two `bl sts_pwrseq_rb_vmax_mv`.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_pwrseq_rb_vmax_mv()"),
		"the sequencer is read more than once for one published "
		"ceiling — check for a MIN()/ternary double evaluation");
	/*
	 * And it does not ALSO derive one. Either of these coming back is the
	 * original defect returning: the cfg key is the request rather than the
	 * bound, and re-applying the constant models one of decide()'s two arms.
	 * Comments are blanked by load_source(), so the prose above the fix in
	 * mp_glue.c — which names both — cannot satisfy these.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "0x0703U"),
		"prov_ilk() reads cfg pwr.rb.vmax.mv again — that is the "
		"REQUEST, not the ceiling in force");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "STS_RB_VMAX_MV_CEILING"),
		"prov_ilk() re-applies the hardware ceiling, which models only "
		"the clamping arm of sts_rb_vmax_decide()");
	/* Exactly one write into the u16 field, and it is the bounded one. The
	 * bound is UINT16_MAX — the FIELD, not the envelope — because a second
	 * envelope bound here is the thing being removed. */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "out->rb_vmax_mv"),
		"a second write to rb_vmax_mv appeared");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U,
		count_in(&b, "out->rb_vmax_mv = (uint16_t)MIN(vmax, "
			     "(uint32_t)UINT16_MAX);"),
		"the write to rb_vmax_mv is no longer the bounded narrowing of "
		"the value read from the sequencer");
}

/**
 * The ceiling the platform PUBLISHES is the field the drain clamps with.
 *
 * The producer half of the property above, split from it because Unity aborts a
 * test at its first failure and these are two independent ways for the fix to
 * rot. sts_pwrseq_rb_vmax_mv() is only worth reading if it returns
 * pwrseq_cfg_t::rb_vmax_mv itself — the exact field pwrseq_mbox_apply_vset()
 * hands sts_rb_code_for_mv() — because that identity is what makes the
 * interlock's clamp and the sequencer's clamp one number by construction rather
 * than by two files agreeing on a constant.
 *
 * The second property is the FAIL-SAFE. Before pwrseq_start() the in-force
 * envelope is unknown, and the accessor must answer 0: MP_ILK_RB_VMAX refuses a
 * ceiling of 0 (`hi <= 0` -> -EPERM, executed in test_mp_override.c), so an
 * unknown envelope denies every VCC_RB request. Returning pwrseq_cfg_default()'s
 * value instead would be strictly MORE permissive than the cfg-derived number
 * this replaced, on a board whose sequencer has not run — and the FE-5680A is
 * the one load where a too-high rail is not recoverable (../CLAUDE.md, "Rb
 * digipot in a buck FB node").
 *
 * READ, NOT EXECUTED: pwrseq_exec.c is a Zephyr translation unit and is linked
 * by no host suite either. The scan proves the text, not the behaviour.
 */
static void test_the_published_rb_ceiling_is_the_drains_own_field(void)
{
	span_t acc;
	span_t drain;

	load_source("zephyr/platform/pwrseq_exec.c");

	acc = fn_body("uint32_t sts_pwrseq_rb_vmax_mv(void)");
	drain = fn_body("static bool pwrseq_mbox_apply_vset(bool release, "
			"int32_t value, uint8_t *out_err,");

	/* One field, both sides. */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&acc, "return pwrseq_cfg.rb_vmax_mv;"),
		"the published ceiling is no longer pwrseq's own in-force "
		"field");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&drain, "pwrseq_cfg.rb_vmax_mv"),
		"the drain no longer bounds the setpoint against the field the "
		"console publishes — the two clamps have separated");

	/* The unknown answer is 0, and it is 0 rather than anything wider. */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&acc, "if (!pwrseq_started) {"),
		"the accessor no longer distinguishes an unstarted sequencer "
		"from a known envelope");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&acc, "return 0U;"),
		"the unknown VCC_RB ceiling stopped answering 0, which is the "
		"only value MP_ILK_RB_VMAX refuses on");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&acc, "!pwrseq_started") <
			offset_in(&acc, "return pwrseq_cfg.rb_vmax_mv;"),
		"the in-force field is returned before the unstarted sequencer "
		"is ruled out");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&acc, "pwrseq_cfg_default"),
		"the unknown ceiling now answers the built-in default, which is "
		"MORE permissive than the cfg-derived value it replaced");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&acc, "STS_RB_VMAX_MV_CEILING"),
		"the accessor publishes the hardware constant instead of the "
		"envelope actually in force");
}

/* =========================== 5. the two objects that had to move a SEQUENCE = */

/**
 * `sys.wdt.en` moves a CADENCE, not a level, and its release is protected.
 *
 * sts_supervisor_arm() kicks once and seeds pwrseq's window from that same
 * instant so the first window opens already fed. A bare gpio_pin_set_dt() on
 * override release would raise WDT_EN mid-cadence with no seed, and the kicker's
 * next serviced kick would land on its 50 ms poll instead of a full period
 * later — a TPS3430 runaway fault, WDO_N -> POE_KILL, and the board drops its
 * own PoE port. So the glue may not touch the pin at all, and the supervisor's
 * enable path must replay the sequence rather than assert.
 *
 * Read rather than asserted behaviourally because neither file is linked by any
 * host suite. The pin-write assertion is the load-bearing one: it is the single
 * edit that would turn this object back into the hazard.
 */
static void test_the_watchdog_enable_replays_the_arm_sequence(void)
{
	span_t b;

	load_source("zephyr/console/mp_glue.c");
	b = fn_body("static int obj_apply(void *user, size_t obj, const int32_t *value)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_supervisor_wdt_enable(*value != 0)"),
		"`sys.wdt.en` no longer actuates through the supervisor");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "gpio_pin_set_dt"),
		"obj_apply() writes a GPIO directly; WDT_EN and MUX_SEL both "
		"carry a sequence the pin alone does not express");

	/*
	 * The release restores the state saved at grant, and the save happens
	 * only AFTER a successful apply — a refused or failed grant leaves
	 * nothing owed a restore. `sts_supervisor_wdt_enable(true)` must NOT
	 * appear: an unconditional re-arm would start the window against a
	 * liveness gate that cannot yet be satisfied when the lease was taken
	 * before stage 9, and the kicker would withhold into a cold cycle.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "sts_supervisor_wdt_enable(wdt_en_saved_armed)"),
		"the `sys.wdt.en` release stopped restoring the state the "
		"watchdog was in when the lease was taken");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "sts_supervisor_wdt_enable(true)"),
		"the release re-arms unconditionally; a lease taken before "
		"stage 9 would arm a watchdog firmware had not armed");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "wdt_en_saved_valid = true;"),
		"the pre-lease watchdog state is no longer recorded exactly "
		"once, after the apply that needed it succeeded");

	/* The pulse reaches the supervisor's own permission check. */
	b = fn_body("static int obj_pulse(void *user, size_t obj, uint32_t ms)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "return sts_supervisor_wdt_kick();"),
		"`sys.wdt.kick` no longer routes through the seam that refuses "
		"an edge against an armed watchdog");

	/* And the supervisor replays kick -> seed -> assert, in that order. */
	load_source("zephyr/platform/supervisor.c");
	b = fn_body("int sts_supervisor_wdt_enable(bool enable)");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&b, "wdt_kick_once();") <
			offset_in(&b, "pwrseq_wdt_arm(&super.wdt"),
		"the re-arm seeds the cadence before feeding the window");
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&b, "pwrseq_wdt_arm(&super.wdt") <
			offset_in(&b, "gpio_pin_set_dt(&wdt_en, 1)"),
		"WDT_EN is raised before the cadence is seeded — the runaway "
		"fault sts_supervisor_arm() exists to avoid");
	/* Disabling stops the kicker as well as dropping the pin: wdt_entry()
	 * polls `armed`, so leaving it set would keep driving WDI at cadence
	 * into a TPS3430 that is not watching, and the re-arm's own seed would
	 * be fighting a window pwrseq still believed was open. */
	TEST_ASSERT_TRUE_MESSAGE(
		offset_in(&b, "super.armed = false;") <
			offset_in(&b, "gpio_pin_set_dt(&wdt_en, 0)"),
		"the disable drops WDT_EN before it stops the kicker");

	b = fn_body("int sts_supervisor_wdt_kick(void)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U,
		count_in(&b, "if (!sts_super_wdi_pulse_allowed(super.ready, "
			     "super.armed)) {"),
		"the WDI edge lost its own permission check; the console "
		"interlock would then be the only thing between a keystroke "
		"and a cold cycle");
}

/**
 * `ref.mux.sel` never becomes a pin write, and its release restores AUTO.
 *
 * PB6 selects which 10 MHz reaches PH0, the HSE bypass input SYSCLK derives
 * from. Flipping it under a live HSE feed skips the HSI bridge and glitches
 * SYSCLK (spec §3.5), so the actuation is refsel's bracketed handoff —
 * PARK_DISCIPLINE, mux flip, UNPARK — reached by posting the standing request.
 *
 * The save/restore is the subtle half. The pre-lease state is three-valued
 * (AUTO / force-OCXO / force-EXTREF) while the object's value space is {0,1},
 * so core cannot hold it: there is no value of the object that means AUTO. The
 * glue saves sts_ref_override_get() VERBATIM, and a lapsed lease therefore
 * returns a box that was running AUTO to AUTO rather than pinning it to
 * whichever level happened to be active — the failure sts_app.h refuses to give
 * a cfg key for.
 */
static void test_the_clock_mux_goes_through_refsel_and_restores_verbatim(void)
{
	span_t b;

	load_source("zephyr/console/mp_glue.c");
	b = fn_body("static int obj_apply(void *user, size_t obj, const int32_t *value)");

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "uint8_t prev = sts_ref_override_get();"),
		"the pre-lease reference request is no longer sampled");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "rc = sts_ref_override_set(mux_sel_saved_req);"),
		"the `ref.mux.sel` release no longer replays the saved request "
		"verbatim, so a lease taken on AUTO would lapse to a level");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "mux_sel_saved_req = prev;"),
		"the saved request is not the value read before the apply");
	/*
	 * `1` is EXTREF, not a force-rubidium that does not exist: AUTO is what
	 * engages the Rb when the guards allow, and EXTREF is the only request
	 * that deterministically drives MUX_SEL to input B.
	 */
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "(uint8_t)STS_REF_REQ_EXTREF"),
		"`ref.mux.sel = 1` no longer selects input B explicitly");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "(uint8_t)STS_REF_REQ_OCXO"),
		"`ref.mux.sel = 0` no longer pins the OCXO");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "STS_REF_REQ_AUTO"),
		"an apply asks for AUTO; AUTO is a state the RELEASE restores, "
		"not a value this two-valued object can request");

	/* The read reports the ACTIVE reference, never the request. */
	b = fn_body("static int obj_read(void *user, size_t obj, mp_val_t *out)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		0U, count_in(&b, "sts_ref_override_get()"),
		"`ref.mux.sel` reads back its own request; \"requested rb, "
		"running ocxo\" is the normal reading and the object is named "
		"for the selector, not the intent");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, count_in(&b, "q.active_ref == (uint8_t)QUALITY_REF_OCXO"),
		"the mux read-back no longer resolves the active reference");
}

/* ------------------------------------------------------------------- runner */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_the_deferred_flag_is_the_dispatch);
	RUN_TEST(test_the_wired_set_is_the_one_the_header_claims);
	RUN_TEST(test_the_write_only_residue_is_exactly_these_five);
	RUN_TEST(test_the_console_never_writes_the_dac);
	RUN_TEST(test_the_discipline_thread_drains_them);
	RUN_TEST(test_the_dispatch_names_nothing_the_manifest_lacks);

	RUN_TEST(test_the_image_gates_actuation_on_the_flag);
	RUN_TEST(test_the_rpc_asks_before_the_guard);
	RUN_TEST(test_notsup_leaves_the_veto_machinery_alone);

	RUN_TEST(test_the_mux_guard_reads_two_hardware_signals);
	RUN_TEST(test_the_permanent_refusals_are_labelled);
	RUN_TEST(test_the_display_stamp_is_sampled_by_the_tick);

	RUN_TEST(test_the_rb_serial_relay_is_wired);
	RUN_TEST(test_the_rb_ceiling_is_the_one_in_force);
	RUN_TEST(test_the_published_rb_ceiling_is_the_drains_own_field);

	RUN_TEST(test_the_watchdog_enable_replays_the_arm_sequence);
	RUN_TEST(test_the_clock_mux_goes_through_refsel_and_restores_verbatim);

	return UNITY_END();
}
