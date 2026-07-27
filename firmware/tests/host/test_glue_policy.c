/*
 * STS1000 "Meridian" — the Zephyr glue's policy decisions, tested away from
 * Zephyr.
 *
 * Three pieces of glue logic are load-bearing enough that "it looked right" is
 * not good enough, and all three are invisible at runtime when they are wrong:
 *
 *   sts_confirm_gate.h  — whether a freshly-staged image may confirm itself. Too
 *                         weak and MCUboot's automatic revert never fires, which
 *                         is the only unattended protection a remote grandmaster
 *                         has against a bad update.
 *   sts_stage_geom.h    — how large an image the staging slot may accept. Too
 *                         large and the image stages, verifies and is marked
 *                         pending, then is silently declined by the bootloader's
 *                         swap on every boot, with logging compiled out.
 *   sts_cfg_applier.h   — who is told about a config change. A registry that
 *                         drops a subscriber and reports success makes every
 *                         later config apply a no-op with no diagnostic.
 *
 * Each header is deliberately free of Zephyr and MCUboot dependencies so it can
 * be compiled here. The staging tests model MCUboot's swap-using-move accept/
 * reject decision independently, from the bootloader sources, rather than
 * comparing the header against itself.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "zephyr/console/sts_confirm_gate.h"
#include "zephyr/console/sts_stage_geom.h"
#include "zephyr/sts_cfg_applier.h"

/* ------------------------------------------------------------------------- */
/* self-confirm gate (F2)                                                    */
/* ------------------------------------------------------------------------- */

/* The pre-fix gate, for contrast: uptime, cfg, NVS and a management link. No
 * timing content at all — which is what let a build with a dead GNSS UART, a
 * dead PPS capture or a dead DAC path confirm itself. */
static bool legacy_gate_pass(const sts_confirm_gate_t *g)
{
	return sts_confirm_min_age_met(g) && g->cfg_loaded && g->store_ready &&
	       g->link_ok;
}

/* A box that is healthy in every respect. */
static void healthy(sts_confirm_gate_t *g)
{
	memset(g, 0, sizeof(*g));
	g->min_age_s = 60U;
	g->deadline_s = sts_confirm_deadline_s(60U, 300U, 1200U, 600U);
	g->uptime_s = 900U;
	g->cfg_loaded = true;
	g->store_ready = true;
	g->link_ok = true;
	g->clock_locked = true;
	g->serving_primary = true;
}

static void test_confirm_gate_needs_every_term(void)
{
	sts_confirm_gate_t g;

	healthy(&g);
	TEST_ASSERT_TRUE(sts_confirm_gate_pass(&g));

	healthy(&g);
	g.uptime_s = 59U; /* one second short of the minimum age */
	TEST_ASSERT_FALSE(sts_confirm_min_age_met(&g));
	TEST_ASSERT_FALSE(sts_confirm_gate_pass(&g));

	healthy(&g);
	g.cfg_loaded = false;
	TEST_ASSERT_FALSE(sts_confirm_gate_pass(&g));

	healthy(&g);
	g.store_ready = false;
	TEST_ASSERT_FALSE(sts_confirm_gate_pass(&g));

	healthy(&g);
	g.link_ok = false;
	TEST_ASSERT_FALSE(sts_confirm_gate_pass(&g));
}

/*
 * The reversion case. A build that never reaches QUALITY_LOCK_LOCKED must NOT
 * confirm — yet it satisfies every term the old gate had, because USB is by
 * definition CONFIGURED during a DFU session and NVS mounts regardless of
 * whether the clock works.
 */
static void test_confirm_gate_refuses_an_undisciplined_image(void)
{
	sts_confirm_gate_t g;

	/* Broken timing path: uptime, cfg, NVS and the USB link are all fine. */
	healthy(&g);
	g.clock_locked = false;
	g.serving_primary = false;

	TEST_ASSERT_TRUE_MESSAGE(legacy_gate_pass(&g),
				 "the pre-fix gate is supposed to pass here — "
				 "that is the defect");
	TEST_ASSERT_FALSE(sts_confirm_timing_ok(&g));
	TEST_ASSERT_FALSE(sts_confirm_gate_pass(&g));

	/* Not even after a very long wait: the gate is a condition, not a timer. */
	g.uptime_s = 100000U;
	TEST_ASSERT_FALSE(sts_confirm_gate_pass(&g));

	/* Disciplined but not yet serving stratum 1: still not confirmable. */
	healthy(&g);
	g.serving_primary = false;
	TEST_ASSERT_FALSE(sts_confirm_gate_pass(&g));

	/* Serving stratum 1 without a declared lock: also refused. */
	healthy(&g);
	g.clock_locked = false;
	TEST_ASSERT_FALSE(sts_confirm_gate_pass(&g));

	/* And a build that DOES reach both confirms. */
	healthy(&g);
	TEST_ASSERT_TRUE(sts_confirm_gate_pass(&g));
	TEST_ASSERT_FALSE(sts_confirm_deadline_passed(&g)); /* before the deadline */
}

/*
 * The deadline has to sit beyond the lock-hold window, because the gate now
 * requires a lock and a lock cannot be declared until tim.lock.hold seconds of
 * criteria have elapsed. The old hardcoded 600 s is only 300 s past the default
 * hold time, so a legitimately slow first fix was abandoned before it could
 * possibly have succeeded.
 */
static void test_confirm_deadline_is_derived_from_the_lock_hold(void)
{
	const uint32_t min_age = 60U;
	const uint32_t margin = 1200U;
	const uint32_t floor = 600U;
	uint32_t d;

	/* Schema default hold: 60 + 300 + 1200. */
	d = sts_confirm_deadline_s(min_age, 300U, margin, floor);
	TEST_ASSERT_EQUAL_UINT32(1560U, d);
	TEST_ASSERT_TRUE_MESSAGE(d > 600U,
				 "the derived deadline must exceed the old "
				 "hardcoded 600 s");
	TEST_ASSERT_TRUE(d > (min_age + 300U));

	/* The maximum tim.lock.hold the schema allows is 3600 s; the deadline
	 * must still clear it rather than pin to the floor. */
	d = sts_confirm_deadline_s(min_age, 3600U, margin, floor);
	TEST_ASSERT_EQUAL_UINT32(4860U, d);
	TEST_ASSERT_TRUE(d > (min_age + 3600U));

	/* The schema minimum hold is 1 s; the configured floor then wins. */
	d = sts_confirm_deadline_s(min_age, 1U, margin, floor);
	TEST_ASSERT_EQUAL_UINT32(1261U, d);
	d = sts_confirm_deadline_s(5U, 1U, 60U, 600U);
	TEST_ASSERT_EQUAL_UINT32(600U, d);

	/* Saturating, never wrapping: no configured combination may produce a
	 * deadline in the past. */
	d = sts_confirm_deadline_s(UINT32_MAX, UINT32_MAX, UINT32_MAX, 600U);
	TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, d);
}

static void test_confirm_deadline_boundary(void)
{
	sts_confirm_gate_t g;

	healthy(&g);
	g.deadline_s = 1560U;

	g.uptime_s = 1559U;
	TEST_ASSERT_FALSE(sts_confirm_deadline_passed(&g));
	g.uptime_s = 1560U;
	TEST_ASSERT_TRUE(sts_confirm_deadline_passed(&g));
}

/* ------------------------------------------------------------------------- */
/* staging-slot geometry (F3)                                                */
/* ------------------------------------------------------------------------- */

/* This board: slot1 = 896 KiB, 8 KiB erase sectors, 16 B flash write block. */
#define SLOT1_SIZE   (896U * 1024U)  /* 917504 */
#define SLOT1_SECTOR (8U * 1024U)    /* 8192   */
#define SLOT1_WRITE  16U

/* The value the fix must produce, worked out by hand in sts_stage_geom.h. */
#define SLOT1_CEILING 901120U

/* What the pre-fix code advertised: the slot minus one erase sector. */
#define SLOT1_LEGACY (SLOT1_SIZE - SLOT1_SECTOR) /* 909312 */

/*
 * MCUboot's swap-using-move accept/reject decision, modelled from
 * bootloader/mcuboot/boot/bootutil/src/swap_move.c rather than from the header
 * under test:
 *
 *   find_last_idx(sz)  = ceil(sz / sector), minimum 1
 *   first_trailer_idx  = n_sectors - 1, decremented until the sectors from it to
 *                        the top of the slot cover boot_trailer_sz()
 *   swap_run() gives up (swap_type = NONE) when last_idx >= first_trailer_idx
 *
 * True means MCUboot would perform the upgrade.
 */
static bool mcuboot_would_swap(uint32_t image_size, uint32_t slot_size,
			       uint32_t sector, uint32_t write_block)
{
	uint32_t n_sectors = slot_size / sector;
	uint32_t trailer_sz = sts_mcuboot_trailer_sz(n_sectors, write_block);
	uint32_t first_trailer_idx = n_sectors - 1U;
	uint32_t last_idx;
	uint32_t sz = 0U;

	while (1) {
		sz += sector;
		if (sz >= trailer_sz) {
			break;
		}
		first_trailer_idx--;
	}

	last_idx = (image_size + sector - 1U) / sector;
	if (last_idx == 0U) {
		last_idx = 1U;
	}

	return last_idx < first_trailer_idx;
}

static void test_mcuboot_trailer_size_matches_the_bootloader(void)
{
	/* 112 sectors * 3 states * 16 B, plus BOOT_MAX_ALIGN*4 + magic(16). */
	TEST_ASSERT_EQUAL_UINT32(112U, SLOT1_SIZE / SLOT1_SECTOR);
	TEST_ASSERT_EQUAL_UINT32(5456U,
		sts_mcuboot_trailer_sz(112U, SLOT1_WRITE));

	/* BOOT_MAX_ALIGN is the write block, floored at 8. */
	TEST_ASSERT_EQUAL_UINT32(8U, sts_mcuboot_max_align(4U));
	TEST_ASSERT_EQUAL_UINT32(8U, sts_mcuboot_max_align(8U));
	TEST_ASSERT_EQUAL_UINT32(16U, sts_mcuboot_max_align(16U));
	TEST_ASSERT_EQUAL_UINT32(32U, sts_mcuboot_max_align(32U));
}

/*
 * The reversion case. The pre-fix ceiling over-advertised by exactly the one
 * erase sector swap-using-move needs to shift the image up into, so every image
 * of 901121..909312 B was accepted, staged, SHA-verified and marked pending — and
 * then declined by the swap forever, silently.
 */
static void test_staging_usable_matches_the_mcuboot_ceiling(void)
{
	uint32_t usable = sts_staging_usable(SLOT1_SIZE, SLOT1_SECTOR,
					     SLOT1_WRITE);

	TEST_ASSERT_EQUAL_UINT32(SLOT1_CEILING, usable);
	TEST_ASSERT_EQUAL_UINT32(110U * SLOT1_SECTOR, usable);

	/* One erase sector below what the old formula promised. */
	TEST_ASSERT_EQUAL_UINT32(SLOT1_LEGACY, usable + SLOT1_SECTOR);

	/* The advertised ceiling is exactly the boundary MCUboot enforces. */
	TEST_ASSERT_TRUE(mcuboot_would_swap(usable, SLOT1_SIZE, SLOT1_SECTOR,
					    SLOT1_WRITE));
	TEST_ASSERT_FALSE(mcuboot_would_swap(usable + 1U, SLOT1_SIZE,
					     SLOT1_SECTOR, SLOT1_WRITE));

	/* And the old value was on the wrong side of it — the defect, stated as
	 * an assertion so it cannot come back. */
	TEST_ASSERT_FALSE_MESSAGE(
		mcuboot_would_swap(SLOT1_LEGACY, SLOT1_SIZE, SLOT1_SECTOR,
				   SLOT1_WRITE),
		"the pre-fix ceiling was accepted by core/mcp and refused by "
		"MCUboot — that is the defect");
}

static void test_staging_usable_sweeps_the_boundary(void)
{
	uint32_t usable = sts_staging_usable(SLOT1_SIZE, SLOT1_SECTOR,
					     SLOT1_WRITE);
	uint32_t sz;

	/* Nothing at or below the ceiling may be refused by the swap, and nothing
	 * above it may be accepted. Sweep the sector boundaries plus the exact
	 * edge, which is where an off-by-one hides. */
	for (sz = SLOT1_SECTOR; sz <= SLOT1_SIZE; sz += SLOT1_SECTOR) {
		bool swappable = mcuboot_would_swap(sz, SLOT1_SIZE, SLOT1_SECTOR,
						    SLOT1_WRITE);

		TEST_ASSERT_EQUAL_INT((sz <= usable) ? 1 : 0, swappable ? 1 : 0);
	}

	TEST_ASSERT_TRUE(mcuboot_would_swap(usable - 1U, SLOT1_SIZE,
					    SLOT1_SECTOR, SLOT1_WRITE));
	TEST_ASSERT_TRUE(mcuboot_would_swap(1U, SLOT1_SIZE, SLOT1_SECTOR,
					    SLOT1_WRITE));
}

static void test_staging_usable_rejects_impossible_geometry(void)
{
	/* Degenerate arguments answer 0, which makes core/mcp refuse every
	 * FW_BEGIN with NOSPC rather than compute a wrapped ceiling. */
	TEST_ASSERT_EQUAL_UINT32(0U, sts_staging_usable(SLOT1_SIZE, 0U, 16U));
	TEST_ASSERT_EQUAL_UINT32(0U, sts_staging_usable(SLOT1_SIZE, 8192U, 0U));
	TEST_ASSERT_EQUAL_UINT32(0U, sts_staging_usable(0U, 8192U, 16U));

	/* A slot with only the trailer and the move sector in it. */
	TEST_ASSERT_EQUAL_UINT32(0U, sts_staging_usable(8192U, 8192U, 16U));
	TEST_ASSERT_EQUAL_UINT32(0U, sts_staging_usable(2U * 8192U, 8192U, 16U));
	TEST_ASSERT_EQUAL_UINT32(8192U,
		sts_staging_usable(3U * 8192U, 8192U, 16U));

	/* A geometry whose trailer spills over several sectors: 4096-byte pages
	 * with 1024 sectors give a 49232 B trailer = 13 pages, so 1024 - 13 - 1
	 * pages remain. Checked against the independent MCUboot model. */
	{
		uint32_t sector = 4096U;
		uint32_t slot = 1024U * sector;
		uint32_t u = sts_staging_usable(slot, sector, 16U);

		TEST_ASSERT_EQUAL_UINT32((1024U - 13U - 1U) * sector, u);
		TEST_ASSERT_TRUE(mcuboot_would_swap(u, slot, sector, 16U));
		TEST_ASSERT_FALSE(mcuboot_would_swap(u + 1U, slot, sector, 16U));
	}
}

/* ------------------------------------------------------------------------- */
/* config-applier registry (F5, F7)                                          */
/* ------------------------------------------------------------------------- */

#define TRACE_MAX 16U

static struct {
	const char *who;
	uint8_t group;
} g_trace[TRACE_MAX];
static size_t g_trace_n;

static void trace(const char *who, uint8_t group)
{
	if (g_trace_n < TRACE_MAX) {
		g_trace[g_trace_n].who = who;
		g_trace[g_trace_n].group = group;
		g_trace_n++;
	}
}

/* Stand-ins for the two real claimants of group 0x09 (log): the console area's
 * log.level -> ring severity push, and the net area's syslog reload. */
static void console_log_applier(void *ctx, uint8_t group)
{
	(void)ctx;
	trace("console", group);
}

static void net_log_applier(void *ctx, uint8_t group)
{
	(void)ctx;
	trace("net", group);
}

static size_t trace_count(const char *who)
{
	size_t n = 0U;

	for (size_t i = 0; i < g_trace_n; i++) {
		if (strcmp(g_trace[i].who, who) == 0) {
			n++;
		}
	}
	return n;
}

/*
 * The reversion case. Two areas claim group 0x09; with one slot per group the
 * second registration silently replaced the first AND returned 0, so nothing
 * warned and log.level became inert at runtime.
 */
static void test_applier_chain_keeps_every_subscriber(void)
{
	sts_cfg_applier_tbl_t t;

	memset(&t, 0, sizeof(t));
	g_trace_n = 0U;

	/* Registration order mirrors main.c: console starts before net. */
	TEST_ASSERT_EQUAL_INT(0,
		sts_cfg_applier_add(&t, 0x09U, console_log_applier, NULL));
	TEST_ASSERT_EQUAL_INT(0,
		sts_cfg_applier_add(&t, 0x09U, net_log_applier, NULL));

	TEST_ASSERT_EQUAL_size_t(2U, sts_cfg_applier_count(&t, 0x09U));

	sts_cfg_applier_dispatch(&t, 0x09U);

	/* BOTH ran, and in registration order. */
	TEST_ASSERT_EQUAL_size_t(2U, g_trace_n);
	TEST_ASSERT_EQUAL_STRING("console", g_trace[0].who);
	TEST_ASSERT_EQUAL_STRING("net", g_trace[1].who);
	TEST_ASSERT_EQUAL_HEX8(0x09U, g_trace[0].group);
	TEST_ASSERT_EQUAL_HEX8(0x09U, g_trace[1].group);
	TEST_ASSERT_EQUAL_size_t(1U, trace_count("console"));
	TEST_ASSERT_EQUAL_size_t(1U, trace_count("net"));
}

static void test_applier_groups_are_independent(void)
{
	sts_cfg_applier_tbl_t t;

	memset(&t, 0, sizeof(t));
	g_trace_n = 0U;

	TEST_ASSERT_EQUAL_INT(0,
		sts_cfg_applier_add(&t, 0x09U, console_log_applier, NULL));
	TEST_ASSERT_EQUAL_INT(0,
		sts_cfg_applier_add(&t, 0x01U, net_log_applier, NULL));

	sts_cfg_applier_dispatch(&t, 0x09U);
	TEST_ASSERT_EQUAL_size_t(1U, g_trace_n);
	TEST_ASSERT_EQUAL_STRING("console", g_trace[0].who);

	sts_cfg_applier_dispatch(&t, 0x01U);
	TEST_ASSERT_EQUAL_size_t(2U, g_trace_n);
	TEST_ASSERT_EQUAL_STRING("net", g_trace[1].who);

	/* An unclaimed group dispatches to nobody and is not an error. */
	sts_cfg_applier_dispatch(&t, 0x0CU);
	TEST_ASSERT_EQUAL_size_t(2U, g_trace_n);
}

static void test_applier_registration_contract(void)
{
	sts_cfg_applier_tbl_t t;
	static int ctx_a;
	static int ctx_b;

	memset(&t, 0, sizeof(t));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
		sts_cfg_applier_add(NULL, 0x09U, console_log_applier, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		sts_cfg_applier_add(&t, 0x09U, NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		sts_cfg_applier_add(&t, STS_CFG_GROUP_MAX, console_log_applier,
				    NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		sts_cfg_applier_add(&t, 0xFFU, console_log_applier, NULL));
	TEST_ASSERT_EQUAL_size_t(0U, sts_cfg_applier_count(&t, 0x09U));
	TEST_ASSERT_EQUAL_size_t(0U,
		sts_cfg_applier_count(&t, STS_CFG_GROUP_MAX));
	TEST_ASSERT_EQUAL_size_t(0U, sts_cfg_applier_count(NULL, 0x09U));

	/* Re-registering the identical (fn, ctx) pair is idempotent, not a
	 * second chain entry: an area whose start function runs twice must not
	 * double-dispatch. */
	TEST_ASSERT_EQUAL_INT(0,
		sts_cfg_applier_add(&t, 0x09U, console_log_applier, &ctx_a));
	TEST_ASSERT_EQUAL_INT(0,
		sts_cfg_applier_add(&t, 0x09U, console_log_applier, &ctx_a));
	TEST_ASSERT_EQUAL_size_t(1U, sts_cfg_applier_count(&t, 0x09U));

	/* Same function, different ctx, is a distinct subscriber. */
	TEST_ASSERT_EQUAL_INT(0,
		sts_cfg_applier_add(&t, 0x09U, console_log_applier, &ctx_b));
	TEST_ASSERT_EQUAL_size_t(2U, sts_cfg_applier_count(&t, 0x09U));

	/* Overflow is reported, and reporting it does not displace anyone. */
	TEST_ASSERT_EQUAL_INT(0,
		sts_cfg_applier_add(&t, 0x09U, net_log_applier, &ctx_a));
	TEST_ASSERT_EQUAL_INT(0,
		sts_cfg_applier_add(&t, 0x09U, net_log_applier, &ctx_b));
	TEST_ASSERT_EQUAL_size_t(STS_CFG_GROUP_SUBS_MAX,
				 sts_cfg_applier_count(&t, 0x09U));

	TEST_ASSERT_EQUAL_INT(-ENOSPC,
		sts_cfg_applier_add(&t, 0x09U, console_log_applier, NULL));
	TEST_ASSERT_EQUAL_size_t(STS_CFG_GROUP_SUBS_MAX,
				 sts_cfg_applier_count(&t, 0x09U));

	g_trace_n = 0U;
	sts_cfg_applier_dispatch(&t, 0x09U);
	TEST_ASSERT_EQUAL_size_t(STS_CFG_GROUP_SUBS_MAX, g_trace_n);
	sts_cfg_applier_dispatch(NULL, 0x09U); /* no crash, no dispatch */
	TEST_ASSERT_EQUAL_size_t(STS_CFG_GROUP_SUBS_MAX, g_trace_n);
}

/*
 * LOW-7: -EIO is "the live tree changed but some keys missed the store". Skipping
 * the applier dispatch for it reached the same silently-ineffective commit as
 * never dispatching at all, by a different route.
 */
static void test_commit_applied_includes_the_persist_failure(void)
{
	TEST_ASSERT_TRUE(sts_cfg_commit_applied(0));
	TEST_ASSERT_TRUE(sts_cfg_commit_applied(-EIO));

	/* Validation and cross-field rejections apply nothing, so they must not
	 * dispatch. */
	TEST_ASSERT_FALSE(sts_cfg_commit_applied(-EPROTO));
	TEST_ASSERT_FALSE(sts_cfg_commit_applied(-ERANGE));
	TEST_ASSERT_FALSE(sts_cfg_commit_applied(-EINVAL));
	TEST_ASSERT_FALSE(sts_cfg_commit_applied(-ENOENT));
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_confirm_gate_needs_every_term);
	RUN_TEST(test_confirm_gate_refuses_an_undisciplined_image);
	RUN_TEST(test_confirm_deadline_is_derived_from_the_lock_hold);
	RUN_TEST(test_confirm_deadline_boundary);

	RUN_TEST(test_mcuboot_trailer_size_matches_the_bootloader);
	RUN_TEST(test_staging_usable_matches_the_mcuboot_ceiling);
	RUN_TEST(test_staging_usable_sweeps_the_boundary);
	RUN_TEST(test_staging_usable_rejects_impossible_geometry);

	RUN_TEST(test_applier_chain_keeps_every_subscriber);
	RUN_TEST(test_applier_groups_are_independent);
	RUN_TEST(test_applier_registration_contract);
	RUN_TEST(test_commit_applied_includes_the_persist_failure);

	return UNITY_END();
}
