/*
 * STS1000 "Meridian" — leap smear: containment fitness test (spec §15.2).
 *
 * The leap smear is an NTP-only serving policy. Two things must never happen,
 * and neither is the kind of defect a functional test can catch — both are
 * silent, and both ship a grandmaster that is subtly and confidently wrong:
 *
 *   1. **The PTP grandmaster must never smear.** IEEE 1588 has no smear
 *      concept; there is nowhere in an Announce or a Sync to say "the
 *      timescale you are being served is 400 ms from UTC on purpose". A
 *      smearing grandmaster is simply out of spec, and its clients — which
 *      are typically the ones that care most about absolute time — have no
 *      way to detect it.
 *   2. **The disciplined timebase must never be perturbed.** `disc` owns the
 *      DAC and the PTP hardware clock. A correction that reached either would
 *      convert a serving policy into a timing fault, and would do so for every
 *      consumer of the box at once.
 *
 * C has no module privacy, so neither property can be enforced by the
 * language. `core/ptp` does not link `core/ntp` (the module map in
 * CMakeLists.txt fixes `ptp -> util quality`), which makes ntp's smear symbols
 * unlinkable from the 1588 engine — but the *glue*, net/sts_ptp.c and
 * net/sts_ptpclk.c, includes net/sts_net.h, which includes ntp/ntp.h. And
 * quality_leap_smear() is reachable from anything at all, because everything
 * depends on `quality`.
 *
 * So the containment is enforced here instead, by scanning the tree: exactly
 * six files may mention the smear, and the PTP and discipline paths are named
 * individually so a violation reports which invariant it broke rather than
 * just that a list changed. This is an architecture-fitness test, not a unit
 * test; it is cheap, and it turns "nothing is wired to it today" into "wiring
 * something to it fails CI".
 *
 * It cannot pass vacuously: it asserts that the scan positively found the
 * files that are SUPPOSED to contain the smear, so a broken path or an empty
 * walk fails instead of reporting a clean tree.
 */

#define _POSIX_C_SOURCE 200809L

#include <dirent.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "unity.h"

#ifndef STS_APP_SRC_DIR
#error "STS_APP_SRC_DIR must be defined by tests/host/CMakeLists.txt"
#endif

#define NEEDLE "smear"
#define NEEDLE_LEN (sizeof(NEEDLE) - 1U)

#define MAX_HITS 64U
#define PATH_MAX_LEN 512U

static char g_hits[MAX_HITS][PATH_MAX_LEN];
static size_t g_hit_count;
static size_t g_files_scanned;
static bool g_overflow;

/* ------------------------------------------------------------------ scan */

static char lower(char c)
{
	return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
}

/**
 * Does @p path contain the needle, case-insensitively?
 *
 * Read in chunks with a (NEEDLE_LEN - 1) byte overlap carried between them, so
 * a match straddling a chunk boundary is still found. Chunked rather than
 * whole-file so the test has no opinion about how big a source file may get.
 */
static bool file_mentions_smear(const char *path)
{
	char buf[4096];
	size_t carry = 0U;
	FILE *f = fopen(path, "rb");
	bool found = false;

	TEST_ASSERT_NOT_NULL_MESSAGE(f, path);

	for (;;) {
		size_t got = fread(&buf[carry], 1U, sizeof(buf) - carry, f);
		size_t have = carry + got;
		size_t i;

		if (got == 0U) {
			break;
		}
		for (i = 0U; i < have; i++) {
			buf[i] = lower(buf[i]);
		}
		if (have >= NEEDLE_LEN) {
			for (i = 0U; i + NEEDLE_LEN <= have; i++) {
				if (memcmp(&buf[i], NEEDLE, NEEDLE_LEN) == 0) {
					found = true;
					break;
				}
			}
		}
		if (found) {
			break;
		}

		/* Carry the tail so a straddling match is not missed. */
		carry = (have < (NEEDLE_LEN - 1U)) ? have : (NEEDLE_LEN - 1U);
		memmove(buf, &buf[have - carry], carry);
	}

	(void)fclose(f);
	return found;
}

static bool is_source(const char *name)
{
	size_t n = strlen(name);

	return n > 2U && name[n - 2U] == '.' &&
	       (name[n - 1U] == 'c' || name[n - 1U] == 'h');
}

/** Recursively scan @p dir, recording every source file that mentions it. */
static void scan(const char *dir, const char *rel)
{
	struct dirent *de;
	DIR *d = opendir(dir);

	TEST_ASSERT_NOT_NULL_MESSAGE(d, dir);

	while ((de = readdir(d)) != NULL) {
		char child[PATH_MAX_LEN];
		char child_rel[PATH_MAX_LEN];
		struct stat st;

		if (strcmp(de->d_name, ".") == 0 ||
		    strcmp(de->d_name, "..") == 0) {
			continue;
		}
		(void)snprintf(child, sizeof(child), "%s/%s", dir, de->d_name);
		(void)snprintf(child_rel, sizeof(child_rel), "%s%s%s", rel,
			       (rel[0] == '\0') ? "" : "/", de->d_name);

		/* stat(), not dirent::d_type: the latter is a BSD extension
		 * that strict C11 + POSIX does not declare, and returns
		 * DT_UNKNOWN on some filesystems even where it does. */
		TEST_ASSERT_EQUAL_INT_MESSAGE(0, stat(child, &st), child);

		if (S_ISDIR(st.st_mode)) {
			scan(child, child_rel);
			continue;
		}
		if (!S_ISREG(st.st_mode) || !is_source(de->d_name)) {
			continue;
		}

		g_files_scanned++;
		if (!file_mentions_smear(child)) {
			continue;
		}
		if (g_hit_count >= MAX_HITS) {
			g_overflow = true;
			continue;
		}
		(void)snprintf(g_hits[g_hit_count], PATH_MAX_LEN, "%s",
			       child_rel);
		g_hit_count++;
	}

	(void)closedir(d);
}

static bool hit(const char *rel)
{
	size_t i;

	for (i = 0U; i < g_hit_count; i++) {
		if (strcmp(g_hits[i], rel) == 0) {
			return true;
		}
	}
	return false;
}

void setUp(void)
{
	if (g_files_scanned == 0U) {
		scan(STS_APP_SRC_DIR, "");
	}
}

void tearDown(void)
{
}

/* ----------------------------------------------------------------- tests */

/** The walk really walked. Everything below is meaningless without this. */
static void test_the_scan_is_not_vacuous(void)
{
	TEST_ASSERT_FALSE_MESSAGE(g_overflow, "hit table overflowed");
	/* The tree is ~200 source files; 100 is a floor that a broken walk
	 * (wrong root, no recursion, wrong extension test) cannot clear. */
	TEST_ASSERT_TRUE_MESSAGE(g_files_scanned > 100U,
				 "scan found implausibly few sources");

	/* And it positively found the files that MUST mention the smear. A
	 * scanner that silently matched nothing would otherwise report a
	 * beautifully clean tree. */
	TEST_ASSERT_TRUE_MESSAGE(hit("core/quality/quality.c"),
				 "scan missed the smear model");
	TEST_ASSERT_TRUE_MESSAGE(hit("core/quality/quality.h"),
				 "scan missed the smear model header");
	TEST_ASSERT_TRUE_MESSAGE(hit("core/ntp/ntp.c"),
				 "scan missed the smear policy");
	TEST_ASSERT_TRUE_MESSAGE(hit("core/ntp/ntp.h"),
				 "scan missed the smear policy header");
	TEST_ASSERT_TRUE_MESSAGE(hit("zephyr/net/sts_ntp.c"),
				 "scan missed the NTP glue");
	TEST_ASSERT_TRUE_MESSAGE(hit("core/cfg/cfg_schema.h"),
				 "scan missed the cfg keys");
}

/**
 * The PTP grandmaster cannot smear, because nothing on the PTP path so much as
 * names it. IEEE 1588 has no smear concept: a grandmaster that smeared would
 * be out of spec with no way to say so on the wire.
 */
static void test_the_ptp_path_cannot_reach_the_smear(void)
{
	static const char *const ptp_files[] = {
		"core/ptp/ptp_port.c",  "core/ptp/ptp.h",
		"zephyr/net/sts_ptp.c", "zephyr/net/sts_ptpclk.c",
	};
	size_t i;

	for (i = 0U; i < (sizeof(ptp_files) / sizeof(ptp_files[0])); i++) {
		char msg[PATH_MAX_LEN + 64U];

		(void)snprintf(msg, sizeof(msg),
			       "%s references the leap smear; PTP must step",
			       ptp_files[i]);
		TEST_ASSERT_FALSE_MESSAGE(hit(ptp_files[i]), msg);
	}

	/* Not just the four named above: the whole 1588 engine. */
	for (i = 0U; i < g_hit_count; i++) {
		char msg[PATH_MAX_LEN + 64U];

		(void)snprintf(msg, sizeof(msg),
			       "%s is on the PTP path and references the smear",
			       g_hits[i]);
		TEST_ASSERT_FALSE_MESSAGE(
			strncmp(g_hits[i], "core/ptp/", 9U) == 0, msg);
		TEST_ASSERT_FALSE_MESSAGE(
			strncmp(g_hits[i], "zephyr/net/sts_ptp", 18U) == 0,
			msg);
	}
}

/**
 * The correction never reaches the timebase. `disc` owns the DAC and the
 * discipline loop; a serving policy that perturbed either would be a timing
 * fault affecting every service at once, not just NTP.
 */
static void test_the_discipline_loop_cannot_reach_the_smear(void)
{
	size_t i;

	for (i = 0U; i < g_hit_count; i++) {
		char msg[PATH_MAX_LEN + 96U];

		(void)snprintf(msg, sizeof(msg),
			       "%s steers the timebase and references the "
			       "smear; the correction must stay in what NTP "
			       "SERVES", g_hits[i]);
		TEST_ASSERT_FALSE_MESSAGE(
			strncmp(g_hits[i], "core/disc/", 10U) == 0, msg);
		TEST_ASSERT_FALSE_MESSAGE(
			strncmp(g_hits[i], "core/refsel/", 12U) == 0, msg);
	}
}

/**
 * The containment list itself. Exactly six files, and adding a seventh is a
 * deliberate act that has to be argued for here rather than noticed later.
 */
static void test_only_the_ntp_path_mentions_the_smear(void)
{
	static const char *const allowed[] = {
		"core/cfg/cfg_schema.h",  /* the two opt-in keys */
		"core/ntp/ntp.c",         /* the policy */
		"core/ntp/ntp.h",         /* the policy contract */
		"core/quality/quality.c", /* the pure model */
		"core/quality/quality.h", /* the pure model contract */
		"zephyr/net/sts_ntp.c",   /* cfg wiring + annunciation */
	};
	size_t i;
	size_t j;

	for (i = 0U; i < g_hit_count; i++) {
		bool ok = false;
		char msg[PATH_MAX_LEN + 96U];

		for (j = 0U; j < (sizeof(allowed) / sizeof(allowed[0])); j++) {
			if (strcmp(g_hits[i], allowed[j]) == 0) {
				ok = true;
				break;
			}
		}
		(void)snprintf(msg, sizeof(msg),
			       "%s references the leap smear but is not on the "
			       "NTP serving path", g_hits[i]);
		TEST_ASSERT_TRUE_MESSAGE(ok, msg);
	}

	TEST_ASSERT_EQUAL_size_t(sizeof(allowed) / sizeof(allowed[0]),
				 g_hit_count);
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_the_scan_is_not_vacuous);
	RUN_TEST(test_the_ptp_path_cannot_reach_the_smear);
	RUN_TEST(test_the_discipline_loop_cannot_reach_the_smear);
	RUN_TEST(test_only_the_ntp_path_mentions_the_smear);

	return UNITY_END();
}
