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
 *   sts_rollback.h      — whether a staged image is a downgrade, and how far to
 *                         step a ONE-WAY hardware counter. The decode is of a
 *                         byte layout owned by another project, and the action
 *                         it drives cannot be undone on the part.
 *   sts_ui_echo.h       — the front-panel button levels the MP mirror
 *                         publishes, reconstructed from a droppable event
 *                         stream. Get it wrong and a remote technician reads a
 *                         permanently-held key on a panel nobody is touching,
 *                         with nothing on the box to contradict it.
 *
 * Each header is deliberately free of Zephyr and MCUboot dependencies so it can
 * be compiled here. The staging tests model MCUboot's swap-using-move accept/
 * reject decision independently, from the bootloader sources, rather than
 * comparing the header against itself; the anti-rollback tests do the same for
 * check_downgrade_prevention(), and additionally rebuild imgtool's output byte
 * for byte so the decoder is checked against the producer's format and not
 * against its own idea of it.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "zephyr/console/sts_confirm_gate.h"
#include "zephyr/console/sts_rollback.h"
#include "zephyr/console/sts_stage_geom.h"
#include "zephyr/sts_cfg_applier.h"
#include "zephyr/ui/sts_ui_echo.h"

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
/* anti-rollback (sts_rollback.h)                                            */
/* ------------------------------------------------------------------------- */

/*
 * The constants restated in sts_rollback.h, quoted here from
 * bootloader/mcuboot/boot/bootutil/include/bootutil/image.h so the two are
 * compared rather than assumed. A silent drift in any of these makes every
 * image look like it carries no security counter, which reads as "downgrade
 * prevention is off" and is exactly the failure the feature exists to stop.
 */
static void test_mcuboot_image_constants_match_the_bootloader(void)
{
	TEST_ASSERT_EQUAL_HEX32(0x96f3b83dU, STS_ROLLBACK_IMAGE_MAGIC); /* IMAGE_MAGIC */
	TEST_ASSERT_EQUAL_UINT32(32U, STS_ROLLBACK_HDR_LEN);   /* IMAGE_HEADER_SIZE */
	TEST_ASSERT_EQUAL_HEX16(0x6908U, STS_ROLLBACK_TLV_PROT_MAGIC);
	TEST_ASSERT_EQUAL_HEX16(0x6907U, STS_ROLLBACK_TLV_INFO_MAGIC);
	TEST_ASSERT_EQUAL_HEX16(0x50U, STS_ROLLBACK_TLV_SEC_CNT);
	/* sizeof(struct image_tlv_info) == sizeof(struct image_tlv) == 4 */
	TEST_ASSERT_EQUAL_UINT32(4U, STS_ROLLBACK_TLV_HDR_LEN);

	/* ATECC608B datasheet §4.2: the monotonic counters stop at 2^21 - 1. */
	TEST_ASSERT_EQUAL_UINT32(2097151U, STS_ROLLBACK_COUNTER_MAX);
	TEST_ASSERT_EQUAL_UINT32((1U << 21) - 1U, STS_ROLLBACK_COUNTER_MAX);
}

/* --- little helpers that emit exactly what imgtool writes ---------------- */

static void put16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFU);
	p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFU);
	p[1] = (uint8_t)((v >> 8) & 0xFFU);
	p[2] = (uint8_t)((v >> 16) & 0xFFU);
	p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

/* struct image_header, the geometry this project actually signs: header size
 * 0x400 (CONFIG_ROM_START_OFFSET), version 0.1.0+0. */
static void make_hdr(uint8_t buf[32], uint16_t prot_tlv_size, uint32_t img_size)
{
	memset(buf, 0, 32);
	put32(&buf[0], STS_ROLLBACK_IMAGE_MAGIC); /* ih_magic */
	put32(&buf[4], 0U);                       /* ih_load_addr */
	put16(&buf[8], 0x400U);                   /* ih_hdr_size */
	put16(&buf[10], prot_tlv_size);           /* ih_protect_tlv_size */
	put32(&buf[12], img_size);                /* ih_img_size */
	put32(&buf[16], 0U);                      /* ih_flags */
	buf[20] = 0U;                             /* iv_major */
	buf[21] = 1U;                             /* iv_minor */
	put16(&buf[22], 0U);                      /* iv_revision */
	put32(&buf[24], 0U);                      /* iv_build_num */
}

/*
 * The exact protected TLV area imgtool emits for `--security-counter N` with no
 * boot record and no dependencies: a 4-octet info header declaring 12, then one
 * 4-octet TLV header, then the counter as a little-endian uint32
 * (imgtool/image.py: `prot_tlv.add('SEC_CNT', struct.pack(e + 'I', ...))`, and
 * `protected_tlv_size += TLV_SIZE + 4` then `+= TLV_INFO_SIZE`).
 */
static size_t make_prot_area(uint8_t *buf, uint32_t counter)
{
	put16(&buf[0], STS_ROLLBACK_TLV_PROT_MAGIC);
	put16(&buf[2], 12U); /* it_tlv_tot, INCLUDING this info header */
	put16(&buf[4], STS_ROLLBACK_TLV_SEC_CNT);
	put16(&buf[6], 4U);
	put32(&buf[8], counter);
	return 12U;
}

static void test_image_header_decode(void)
{
	sts_rollback_hdr_t h;
	uint8_t buf[32];

	make_hdr(buf, 12U, 0xA3F2CU); /* the real img_size of the 0.1.0 build */
	TEST_ASSERT_EQUAL_INT(0, sts_rollback_hdr_parse(buf, sizeof(buf), &h));
	TEST_ASSERT_EQUAL_UINT16(0x400U, h.hdr_size);
	TEST_ASSERT_EQUAL_UINT16(12U, h.prot_tlv_size);
	TEST_ASSERT_EQUAL_UINT32(0xA3F2CU, h.img_size);
	TEST_ASSERT_EQUAL_UINT32(0U, h.version[0]);
	TEST_ASSERT_EQUAL_UINT32(1U, h.version[1]);
	/* BOOT_TLV_OFF(hdr) = ih_hdr_size + ih_img_size */
	TEST_ASSERT_EQUAL_UINT64(0x400U + 0xA3F2CULL,
				 sts_rollback_prot_tlv_off(&h));

	/* An image with no protected TLVs at all — every build before this
	 * change. Decodes fine; the absence is reported by the walk, not here. */
	make_hdr(buf, 0U, 0x1000U);
	TEST_ASSERT_EQUAL_INT(0, sts_rollback_hdr_parse(buf, sizeof(buf), &h));
	TEST_ASSERT_EQUAL_UINT16(0U, h.prot_tlv_size);

	/* Erased flash, an unsigned image, or anything else that is not one. */
	memset(buf, 0xFF, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, sts_rollback_hdr_parse(buf, sizeof(buf), &h));
	make_hdr(buf, 12U, 0x1000U);
	put32(&buf[0], 0x96f3b83cU); /* IMAGE_MAGIC_V1 — not what we accept */
	TEST_ASSERT_EQUAL_INT(-EILSEQ, sts_rollback_hdr_parse(buf, sizeof(buf), &h));

	/* Impossible geometry: a header smaller than the header. */
	make_hdr(buf, 12U, 0x1000U);
	put16(&buf[8], 31U);
	TEST_ASSERT_EQUAL_INT(-EILSEQ, sts_rollback_hdr_parse(buf, sizeof(buf), &h));

	/* A protected area too small to hold even its own info header would
	 * underflow the walk. */
	make_hdr(buf, 3U, 0x1000U);
	TEST_ASSERT_EQUAL_INT(-EILSEQ, sts_rollback_hdr_parse(buf, sizeof(buf), &h));

	/* Arguments. */
	make_hdr(buf, 12U, 0x1000U);
	TEST_ASSERT_EQUAL_INT(-EINVAL, sts_rollback_hdr_parse(buf, 31U, &h));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sts_rollback_hdr_parse(NULL, 32U, &h));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sts_rollback_hdr_parse(buf, 32U, NULL));
}

static void test_sec_cnt_found_in_imgtools_layout(void)
{
	uint8_t area[64];
	uint32_t v = 0xDEADU;
	size_t n;

	memset(area, 0, sizeof(area));
	n = make_prot_area(area, 1U);
	TEST_ASSERT_EQUAL_size_t(12U, n);
	TEST_ASSERT_EQUAL_INT(0, sts_rollback_sec_cnt_find(area, n, &v));
	TEST_ASSERT_EQUAL_UINT32(1U, v);

	/* A large epoch, to prove the four octets are read little-endian and
	 * whole. 2097151 is the witness ceiling. */
	n = make_prot_area(area, 2097151U);
	TEST_ASSERT_EQUAL_INT(0, sts_rollback_sec_cnt_find(area, n, &v));
	TEST_ASSERT_EQUAL_UINT32(2097151U, v);
	TEST_ASSERT_EQUAL_HEX8(0xFF, area[8]);
	TEST_ASSERT_EQUAL_HEX8(0xFF, area[9]);
	TEST_ASSERT_EQUAL_HEX8(0x1F, area[10]);
	TEST_ASSERT_EQUAL_HEX8(0x00, area[11]);
}

static void test_sec_cnt_absent_is_not_malformed(void)
{
	uint8_t area[64];
	uint32_t v = 0xDEADU;

	/* An image signed without --security-counter has NO protected area at
	 * all: prot_tlv_size is 0. That is -ENOENT, and the caller turns it into
	 * "refuse this staged image", not into "the image is corrupt". */
	TEST_ASSERT_EQUAL_INT(-ENOENT, sts_rollback_sec_cnt_find(NULL, 0U, &v));

	/* A protected area holding only a BOOT_RECORD (0x60), which is what
	 * --boot-record produces without --security-counter. Well-formed, and
	 * still no counter. */
	memset(area, 0, sizeof(area));
	put16(&area[0], STS_ROLLBACK_TLV_PROT_MAGIC);
	put16(&area[2], 12U);
	put16(&area[4], 0x60U);
	put16(&area[6], 4U);
	put32(&area[8], 0x11223344U);
	TEST_ASSERT_EQUAL_INT(-ENOENT, sts_rollback_sec_cnt_find(area, 12U, &v));
	TEST_ASSERT_EQUAL_UINT32(0xDEADU, v); /* untouched */

	/* SEC_CNT after another protected TLV: the walk must not stop early. */
	memset(area, 0, sizeof(area));
	put16(&area[0], STS_ROLLBACK_TLV_PROT_MAGIC);
	put16(&area[2], 20U);
	put16(&area[4], 0x60U);   /* BOOT_RECORD */
	put16(&area[6], 4U);
	put32(&area[8], 0x11223344U);
	put16(&area[12], STS_ROLLBACK_TLV_SEC_CNT);
	put16(&area[14], 4U);
	put32(&area[16], 7U);
	TEST_ASSERT_EQUAL_INT(0, sts_rollback_sec_cnt_find(area, 20U, &v));
	TEST_ASSERT_EQUAL_UINT32(7U, v);
}

static void test_sec_cnt_rejects_a_malformed_area(void)
{
	uint8_t area[64];
	uint32_t v = 0xDEADU;

	/* Unprotected magic where the protected one belongs: reading the
	 * unprotected area would find TLVs nothing signed. */
	make_prot_area(area, 1U);
	put16(&area[0], STS_ROLLBACK_TLV_INFO_MAGIC);
	TEST_ASSERT_EQUAL_INT(-EILSEQ, sts_rollback_sec_cnt_find(area, 12U, &v));

	/* it_tlv_tot disagreeing with ih_protect_tlv_size — the same
	 * cross-check MCUboot's bootutil_tlv_iter_begin() makes before it will
	 * walk anything. */
	make_prot_area(area, 1U);
	put16(&area[2], 16U);
	TEST_ASSERT_EQUAL_INT(-EILSEQ, sts_rollback_sec_cnt_find(area, 12U, &v));

	/* A TLV whose length runs past the declared area. */
	make_prot_area(area, 1U);
	put16(&area[6], 5U);
	TEST_ASSERT_EQUAL_INT(-EILSEQ, sts_rollback_sec_cnt_find(area, 12U, &v));

	/* A SEC_CNT that is not a uint32. Reading four octets anyway would
	 * fabricate a counter out of neighbouring bytes. */
	memset(area, 0, sizeof(area));
	put16(&area[0], STS_ROLLBACK_TLV_PROT_MAGIC);
	put16(&area[2], 10U);
	put16(&area[4], STS_ROLLBACK_TLV_SEC_CNT);
	put16(&area[6], 2U);
	put16(&area[8], 1U);
	TEST_ASSERT_EQUAL_INT(-EILSEQ, sts_rollback_sec_cnt_find(area, 10U, &v));

	/* Truncated below even an info header. */
	make_prot_area(area, 1U);
	TEST_ASSERT_EQUAL_INT(-EILSEQ, sts_rollback_sec_cnt_find(area, 3U, &v));

	TEST_ASSERT_EQUAL_UINT32(0xDEADU, v); /* nothing wrote through */
	TEST_ASSERT_EQUAL_INT(-EINVAL, sts_rollback_sec_cnt_find(area, 12U, NULL));
}

/*
 * An independent model of MCUboot's check_downgrade_prevention()
 * (boot/bootutil/src/loader.c) for the security-counter variant, written from
 * the bootloader source rather than from sts_rollback.h. The point is to catch
 * the two implementations disagreeing, so it deliberately does not share code.
 */
static int mcuboot_would_swap_epoch(int slot0_rc, uint32_t slot0_cnt, int slot1_rc,
			      uint32_t slot1_cnt)
{
	if (slot0_rc != 0) {
		return 1; /* "If there was no security counter in slot 0, allow swap" */
	}
	if (slot1_rc != 0) {
		return 0;
	}
	return (slot0_cnt > slot1_cnt) ? 0 : 1;
}

static void test_staged_verdict_matches_the_bootloader(void)
{
	/* Equal counters swap: that is the whole reason for the counter variant
	 * over plain version comparison — routine releases keep the epoch, and a
	 * 0.1.0 -> 0.1.1 patch has to be installable. */
	TEST_ASSERT_EQUAL_INT(STS_ROLLBACK_STAGED_OK,
			      sts_rollback_staged_verdict(0, 3U, 3U));
	TEST_ASSERT_EQUAL_INT(1, mcuboot_would_swap_epoch(0, 3U, 0, 3U));

	/* Higher swaps. */
	TEST_ASSERT_EQUAL_INT(STS_ROLLBACK_STAGED_OK,
			      sts_rollback_staged_verdict(0, 4U, 3U));
	TEST_ASSERT_EQUAL_INT(1, mcuboot_would_swap_epoch(0, 3U, 0, 4U));

	/* Lower is the attack: a correctly-signed older image. */
	TEST_ASSERT_EQUAL_INT(STS_ROLLBACK_STAGED_OLDER,
			      sts_rollback_staged_verdict(0, 2U, 3U));
	TEST_ASSERT_EQUAL_INT(0, mcuboot_would_swap_epoch(0, 3U, 0, 2U));

	/* No counter in the staged image: refused, because the running one has
	 * one. */
	TEST_ASSERT_EQUAL_INT(STS_ROLLBACK_STAGED_NO_COUNTER,
			      sts_rollback_staged_verdict(-ENOENT, 0U, 3U));
	TEST_ASSERT_EQUAL_INT(0, mcuboot_would_swap_epoch(0, 3U, -1, 0U));

	/* A malformed area is refused too, and reported as its own thing. */
	TEST_ASSERT_EQUAL_INT(STS_ROLLBACK_STAGED_MALFORMED,
			      sts_rollback_staged_verdict(-EILSEQ, 0U, 3U));

	/* Sweep the boundary at the epoch this product ships with. */
	for (uint32_t staged = 0U; staged <= 4U; staged++) {
		sts_rollback_staged_t v =
			sts_rollback_staged_verdict(0, staged, 2U);
		int mb = mcuboot_would_swap_epoch(0, 2U, 0, staged);

		TEST_ASSERT_EQUAL_INT(mb, (v == STS_ROLLBACK_STAGED_OK) ? 1 : 0);
	}

	/* Every verdict has a distinct string; "unknown" is reserved for a
	 * value that is not one. */
	TEST_ASSERT_EQUAL_STRING("acceptable",
				 sts_rollback_staged_str(STS_ROLLBACK_STAGED_OK));
	TEST_ASSERT_EQUAL_STRING("older security epoch",
		sts_rollback_staged_str(STS_ROLLBACK_STAGED_OLDER));
	TEST_ASSERT_EQUAL_STRING("signed without a security counter",
		sts_rollback_staged_str(STS_ROLLBACK_STAGED_NO_COUNTER));
	TEST_ASSERT_EQUAL_STRING("malformed protected TLV area",
		sts_rollback_staged_str(STS_ROLLBACK_STAGED_MALFORMED));
	TEST_ASSERT_EQUAL_STRING("unknown",
		sts_rollback_staged_str((sts_rollback_staged_t)99));
}

/*
 * The one-way part. Every increment is irreversible on the part, so the step
 * count is the number that matters most in this file.
 */
static void test_witness_steps_are_bounded_and_one_way(void)
{
	const uint32_t cap = STS_ROLLBACK_MAX_STEPS;

	/* The steady state, and the reason this may hang off a confirmation:
	 * once the counter has reached the epoch, every later boot, every later
	 * confirmation and every `sts fw confirm` costs zero steps. */
	TEST_ASSERT_EQUAL_UINT32(0U, sts_rollback_witness_steps(1U, 1U, cap));
	TEST_ASSERT_EQUAL_UINT32(0U, sts_rollback_witness_steps(5U, 5U, cap));

	/* A factory-fresh part meeting the shipping epoch. */
	TEST_ASSERT_EQUAL_UINT32(1U, sts_rollback_witness_steps(1U, 0U, cap));

	/* One security epoch advanced: exactly one step. */
	TEST_ASSERT_EQUAL_UINT32(1U, sts_rollback_witness_steps(4U, 3U, cap));

	/* A unit that sat out several epochs catches up in one confirmation. */
	TEST_ASSERT_EQUAL_UINT32(4U, sts_rollback_witness_steps(7U, 3U, cap));

	/* Never backwards, and never past the epoch. A part that arrives
	 * pre-incremented is not a fault. */
	TEST_ASSERT_EQUAL_UINT32(0U, sts_rollback_witness_steps(1U, 900U, cap));
	TEST_ASSERT_EQUAL_UINT32(0U, sts_rollback_witness_steps(0U, 0U, cap));

	/* An implausible epoch cannot burn the counter: bounded, not obeyed. */
	TEST_ASSERT_EQUAL_UINT32(cap, sts_rollback_witness_steps(1000000U, 0U, cap));
	TEST_ASSERT_EQUAL_UINT32(cap,
		sts_rollback_witness_steps(STS_ROLLBACK_COUNTER_MAX, 0U, cap));

	/* Saturates at the part's ceiling rather than wrapping. */
	TEST_ASSERT_EQUAL_UINT32(0U,
		sts_rollback_witness_steps(UINT32_MAX, STS_ROLLBACK_COUNTER_MAX,
					   cap));
	TEST_ASSERT_EQUAL_UINT32(1U,
		sts_rollback_witness_steps(UINT32_MAX,
					   STS_ROLLBACK_COUNTER_MAX - 1U, cap));

	/* The cap is a real bound, not decoration. */
	TEST_ASSERT_TRUE(STS_ROLLBACK_MAX_STEPS > 0U);
	TEST_ASSERT_TRUE(STS_ROLLBACK_MAX_STEPS < STS_ROLLBACK_COUNTER_MAX);
}

/*
 * Applying the steps must reach a fixed point. This is the arithmetic behind
 * "a confirm that happens twice for the same image does not increment twice":
 * even with every runtime guard removed, the second pass asks for nothing.
 */
static void test_witness_converges_and_never_double_counts(void)
{
	uint32_t counter = 0U;
	uint32_t epoch = 3U;
	uint32_t steps;
	unsigned int rounds = 0U;

	steps = sts_rollback_witness_steps(epoch, counter, STS_ROLLBACK_MAX_STEPS);
	TEST_ASSERT_EQUAL_UINT32(3U, steps);
	counter += steps;
	TEST_ASSERT_EQUAL_UINT32(3U, counter);

	/* Re-running the whole witness — twice, ten times, on every boot for a
	 * decade — adds nothing. */
	for (unsigned int i = 0U; i < 10U; i++) {
		TEST_ASSERT_EQUAL_UINT32(0U,
			sts_rollback_witness_steps(epoch, counter,
						   STS_ROLLBACK_MAX_STEPS));
	}
	TEST_ASSERT_EQUAL_UINT32(3U, counter);

	/* A capped gap closes over successive confirmed upgrades instead of
	 * being abandoned; it must terminate. */
	counter = 0U;
	epoch = STS_ROLLBACK_MAX_STEPS * 3U;
	while ((steps = sts_rollback_witness_steps(epoch, counter,
						   STS_ROLLBACK_MAX_STEPS)) != 0U) {
		counter += steps;
		rounds++;
		TEST_ASSERT_TRUE_MESSAGE(rounds < 100U,
					 "witness stepping must terminate");
	}
	TEST_ASSERT_EQUAL_UINT32(epoch, counter);
	TEST_ASSERT_EQUAL_UINT32(3U, rounds);
}

/*
 * The budget claim in sts_rollback.h and app/conf/rollback.conf, asserted so it
 * cannot rot: at one increment per security epoch the part outlives the
 * product, and at one per boot it does not.
 */
static void test_witness_budget_arithmetic(void)
{
	const uint32_t budget = STS_ROLLBACK_COUNTER_MAX;

	/* Four security-relevant releases a year. */
	TEST_ASSERT_TRUE((budget / 4U) > 500000U);
	/* Twelve confirmed updates a year, if it were ever moved to per-update. */
	TEST_ASSERT_TRUE((budget / 12U) > 100000U);
	/* And the case sts_atecc.h forbids: a 10 s reboot loop, 8640 a day,
	 * exhausts a one-way counter inside a year. */
	TEST_ASSERT_TRUE((budget / 8640U) < 365U);
}

/* ------------------------------------------------------------------------- */
/* panel input echo (sts_ui_echo.h)                                          */
/* ------------------------------------------------------------------------- */

/* The signal numbers this echo actually carries (core/fault fault_sig_t). */
#define SIG_BUTTON_1 0U
#define SIG_BUTTON_6 5U
#define SIG_ENC_BUTTON 11U

/* Ordinary press/release traffic reconstructs the level exactly. */
static void test_echo_tracks_both_edges(void)
{
	sts_ui_echo_t e;

	memset(&e, 0, sizeof(e));

	sts_ui_echo_button(&e, SIG_BUTTON_1, true);
	sts_ui_echo_button(&e, SIG_ENC_BUTTON, true);
	TEST_ASSERT_TRUE(sts_ui_echo_is_down(&e, SIG_BUTTON_1));
	TEST_ASSERT_TRUE(sts_ui_echo_is_down(&e, SIG_ENC_BUTTON));
	TEST_ASSERT_EQUAL_UINT32((1U << SIG_BUTTON_1) | (1U << SIG_ENC_BUTTON),
				 e.down);

	sts_ui_echo_button(&e, SIG_BUTTON_1, false);
	TEST_ASSERT_FALSE(sts_ui_echo_is_down(&e, SIG_BUTTON_1));
	TEST_ASSERT_TRUE(sts_ui_echo_is_down(&e, SIG_ENC_BUTTON));

	/* A quiet queue must never resync: a spurious clear would drop a
	 * genuinely-held button out of the mirror every frame. */
	TEST_ASSERT_FALSE(sts_ui_echo_sync(&e, 0U));
	TEST_ASSERT_TRUE(sts_ui_echo_is_down(&e, SIG_ENC_BUTTON));
	TEST_ASSERT_EQUAL_UINT32(0U, e.resyncs);
}

/*
 * The defect this exists to prevent, reproduced end to end.
 *
 * Without the drop reconciliation, a release lost to the full queue latches its
 * bit forever: the level is only ever cleared by an event that no longer
 * arrives. A host on `mirror.get` then reads a permanently-held key and
 * diagnoses a stuck button on a panel nobody is touching.
 */
static void test_echo_clears_a_phantom_when_a_release_is_dropped(void)
{
	sts_ui_echo_t e;

	memset(&e, 0, sizeof(e));

	/* Press lands, release is dropped on the way in. */
	sts_ui_echo_button(&e, SIG_BUTTON_1, true);
	TEST_ASSERT_TRUE(sts_ui_echo_is_down(&e, SIG_BUTTON_1));

	/* Pre-fix behaviour, stated so a regression is unmistakable: nothing
	 * else in the event stream can ever clear this bit. */
	TEST_ASSERT_EQUAL_UINT32(1U << SIG_BUTTON_1, e.down);

	TEST_ASSERT_TRUE(sts_ui_echo_sync(&e, 1U));
	TEST_ASSERT_FALSE(sts_ui_echo_is_down(&e, SIG_BUTTON_1));
	TEST_ASSERT_EQUAL_UINT32(0U, e.down);
	TEST_ASSERT_EQUAL_UINT32(1U, e.resyncs);

	/* One drop, one resync: the same count must not clear the echo again,
	 * or a genuine press posted after the drop would never be reported. */
	sts_ui_echo_button(&e, SIG_ENC_BUTTON, true);
	TEST_ASSERT_FALSE(sts_ui_echo_sync(&e, 1U));
	TEST_ASSERT_TRUE(sts_ui_echo_is_down(&e, SIG_ENC_BUTTON));
	TEST_ASSERT_EQUAL_UINT32(1U, e.resyncs);
}

/* The lamp hold rides the same lossy stream, so the same clear must reach it —
 * a stranded "held" leaves the whole panel LED string full-on indefinitely. */
static void test_echo_resync_releases_the_lamp_hold(void)
{
	sts_ui_echo_t e;

	memset(&e, 0, sizeof(e));

	sts_ui_echo_button(&e, SIG_BUTTON_6, true);
	e.lamp = true;

	TEST_ASSERT_TRUE(sts_ui_echo_sync(&e, 7U));
	TEST_ASSERT_FALSE(e.lamp);
	TEST_ASSERT_EQUAL_UINT32(7U, e.drops_seen);

	/* Only a *change* matters, so the counter's 2^32 wrap needs no special
	 * case — the producer's total is never used as a magnitude. */
	TEST_ASSERT_FALSE(sts_ui_echo_sync(&e, 7U));
	TEST_ASSERT_TRUE(sts_ui_echo_sync(&e, 0U));
	TEST_ASSERT_EQUAL_UINT32(2U, e.resyncs);
}

/* Defensive edges: no NULL deref, and a signal past the word is ignored rather
 * than shifting out of range (UB) or aliasing bit 0. */
static void test_echo_rejects_out_of_range_input(void)
{
	sts_ui_echo_t e;

	memset(&e, 0, sizeof(e));

	sts_ui_echo_button(NULL, SIG_BUTTON_1, true);
	sts_ui_echo_button(&e, 32U, true);
	sts_ui_echo_button(&e, 255U, true);
	TEST_ASSERT_EQUAL_UINT32(0U, e.down);
	TEST_ASSERT_FALSE(sts_ui_echo_is_down(&e, 32U));
	TEST_ASSERT_FALSE(sts_ui_echo_is_down(NULL, SIG_BUTTON_1));
	TEST_ASSERT_FALSE(sts_ui_echo_sync(NULL, 5U));
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

	RUN_TEST(test_mcuboot_image_constants_match_the_bootloader);
	RUN_TEST(test_image_header_decode);
	RUN_TEST(test_sec_cnt_found_in_imgtools_layout);
	RUN_TEST(test_sec_cnt_absent_is_not_malformed);
	RUN_TEST(test_sec_cnt_rejects_a_malformed_area);
	RUN_TEST(test_staged_verdict_matches_the_bootloader);
	RUN_TEST(test_witness_steps_are_bounded_and_one_way);
	RUN_TEST(test_witness_converges_and_never_double_counts);
	RUN_TEST(test_witness_budget_arithmetic);

	RUN_TEST(test_echo_tracks_both_edges);
	RUN_TEST(test_echo_clears_a_phantom_when_a_release_is_dropped);
	RUN_TEST(test_echo_resync_releases_the_lamp_hold);
	RUN_TEST(test_echo_rejects_out_of_range_input);

	return UNITY_END();
}
