/*
 * STS1000 "Meridian" — the firmware-update seam's five decisions.
 *
 * sts_fwupd_seam_policy.h exists because every one of the five is silent when
 * it is wrong, and four of the five have already been wrong in this tree. This
 * suite is written against the failure modes, not against the code:
 *
 *   1. THE TRANSMIT CONTRACT IS NOT INVERTED. core/fwupd's ubx_fwupd_ops_t::tx
 *      is "0 on success"; the platform sink returns the octet count. Passing
 *      the count through made ubx_fwupd.c read every successful frame as a
 *      failure AND return the byte count where its caller expected an errno.
 *      Both halves are asserted: a full write must be exactly 0, and a real
 *      errno must survive unchanged rather than being flattened.
 *
 *   2. "NOT LINKED" AND "NOT STARTED" ARE DIFFERENT ANSWERS. The probe was
 *      `!= -ENOTSUP`, which called a strong symbol's -ENODEV "available" and so
 *      reported a usable USART3 before the GNSS thread had started. That turned
 *      a documented fail-safe into a refusal that could never fire.
 *
 *   3. AN UNAUTHENTICATED IDENTITY READ NEVER DRIVES A PORT SOMEBODY ELSE OWNS.
 *      `fw.inventory` is G0 and stays G0. What it must not do is write into a
 *      passthrough tunnel's live session. The refusal order is part of the
 *      contract and is asserted directly: the tunnel outranks every other
 *      reason, including one that would refuse anyway.
 *
 *   4. THE SUPERVISOR PASS BUDGET ADDS ITS WORK TERM ONCE. The dead-man
 *      inequality in sts_console.c is only as good as its arithmetic, and the
 *      once-not-per-pass choice is a modelling decision that a future edit
 *      could quietly reverse in either direction.
 *
 *   5. THE STAGING ERASE COVERS THE WRITE AND STOPS THERE. Nothing under
 *      port_image_t::staging_write() erases, so a window that falls short means
 *      programming flash that still holds the previous image — which the H5
 *      refuses, making the documented primary update path work exactly once per
 *      board. A window that runs long is worse in the other direction: past
 *      staging_size() lies the MCUboot trailer and the swap-using-move free
 *      sector, and erasing either turns a successful upload into an upgrade the
 *      bootloader silently declines. This half of the suite therefore asserts
 *      both edges — that the window always covers the chunk, and that it never
 *      passes the ceiling — plus the per-call bound the engine-lock budget in
 *      decision 4 rests on.
 *
 * Nothing here round-trips against fwupd_glue.c: all five are pure functions of
 * their arguments, which is why they live in a header of their own.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "zephyr/console/sts_fwupd_seam_policy.h"

/* ===================================================================== */
/* 1. the transmit contract                                              */
/* ===================================================================== */

/**
 * The whole point: a sink that wrote everything reports success as 0, not as
 * the number of octets it wrote.
 */
static void test_a_full_write_is_success_not_a_count(void)
{
	TEST_ASSERT_EQUAL_INT(0, sts_fwupd_tx_from_count(8, 8U));
	TEST_ASSERT_EQUAL_INT(0, sts_fwupd_tx_from_count(1, 1U));
	TEST_ASSERT_EQUAL_INT(0, sts_fwupd_tx_from_count(512, 512U));

	/* The UBX-MON-VER poll that started all this: an 8-octet frame whose
	 * sink answered 8 was being read as rc=8, i.e. "failed, errno 8". */
	TEST_ASSERT_EQUAL_INT(0, sts_fwupd_tx_from_count(8, 8U));
}

/** A zero-length burst that wrote zero octets is still a success. */
static void test_an_empty_burst_is_success(void)
{
	TEST_ASSERT_EQUAL_INT(0, sts_fwupd_tx_from_count(0, 0U));
}

/**
 * A short count is a failure, not a partial write to be resumed.
 *
 * Both sinks behind this are all-or-nothing uart_poll_out() loops, so a short
 * count means the sink is not the one the seam was written against. Accepting
 * it would drop octets out of the middle of a flash image and report success.
 */
static void test_a_short_write_fails(void)
{
	TEST_ASSERT_EQUAL_INT(-EIO, sts_fwupd_tx_from_count(7, 8U));
	TEST_ASSERT_EQUAL_INT(-EIO, sts_fwupd_tx_from_count(0, 8U));
	TEST_ASSERT_EQUAL_INT(-EIO, sts_fwupd_tx_from_count(511, 512U));
}

/**
 * A count LONGER than asked for is also a failure.
 *
 * It cannot happen from either sink, which is exactly why it must not be
 * rounded to success: it means the sink is doing something the seam does not
 * model, and guessing is how the byte count got propagated as an errno in the
 * first place.
 */
static void test_an_over_long_count_fails(void)
{
	TEST_ASSERT_EQUAL_INT(-EIO, sts_fwupd_tx_from_count(9, 8U));
	TEST_ASSERT_EQUAL_INT(-EIO, sts_fwupd_tx_from_count(1, 0U));
}

/**
 * A real errno survives unchanged.
 *
 * The caller has to keep telling -EPERM ("the port is not yours") from -ENODEV
 * ("there is no port") — sts_gnss_uart_raw_tx() answers the first outside a
 * suspended session and the second before the GNSS thread starts, and
 * flattening either to -EIO would hide which.
 */
static void test_a_negative_errno_passes_through(void)
{
	TEST_ASSERT_EQUAL_INT(-EPERM, sts_fwupd_tx_from_count(-EPERM, 8U));
	TEST_ASSERT_EQUAL_INT(-ENODEV, sts_fwupd_tx_from_count(-ENODEV, 8U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sts_fwupd_tx_from_count(-EINVAL, 8U));
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, sts_fwupd_tx_from_count(-ENOTSUP, 8U));

	/* And the length is irrelevant to a failure: the sink already said no. */
	TEST_ASSERT_EQUAL_INT(-EPERM, sts_fwupd_tx_from_count(-EPERM, 0U));
}

/** Nothing but an exact match is success, swept over a range of lengths. */
static void test_only_an_exact_match_succeeds(void)
{
	int len;
	int rc;

	for (len = 0; len <= 16; len++) {
		for (rc = 0; rc <= 20; rc++) {
			int got = sts_fwupd_tx_from_count(rc, (size_t)len);

			if (rc == len) {
				TEST_ASSERT_EQUAL_INT(0, got);
			} else {
				TEST_ASSERT_EQUAL_INT(-EIO, got);
			}
		}
	}
}

/* ===================================================================== */
/* 2. transport classification                                           */
/* ===================================================================== */

/** Only the __weak stub's -ENOTSUP means "this image has no transport". */
static void test_only_notsup_means_absent(void)
{
	TEST_ASSERT_EQUAL_INT(STS_FWUPD_XPORT_ABSENT,
			      sts_fwupd_xport_classify(-ENOTSUP));
}

/**
 * The regression: -ENODEV is the strong symbol saying "not started yet".
 *
 * It is NOT the same as absent and it is NOT ready. `!= -ENOTSUP` reported this
 * case as a wired, usable transport, so gnss_prepare()'s fail-safe never fired
 * and the boot log claimed a state it had not established.
 */
static void test_the_area_not_started_is_neither_absent_nor_ready(void)
{
	TEST_ASSERT_EQUAL_INT(STS_FWUPD_XPORT_DOWN,
			      sts_fwupd_xport_classify(-ENODEV));

	/* Any other refusal the platform might grow lands on the same side. */
	TEST_ASSERT_EQUAL_INT(STS_FWUPD_XPORT_DOWN,
			      sts_fwupd_xport_classify(-EPERM));
	TEST_ASSERT_EQUAL_INT(STS_FWUPD_XPORT_DOWN,
			      sts_fwupd_xport_classify(-EBUSY));
	TEST_ASSERT_EQUAL_INT(STS_FWUPD_XPORT_DOWN,
			      sts_fwupd_xport_classify(-EINVAL));
}

/** A non-negative probe — a drained octet count — is the only READY. */
static void test_a_non_negative_probe_is_ready(void)
{
	TEST_ASSERT_EQUAL_INT(STS_FWUPD_XPORT_READY,
			      sts_fwupd_xport_classify(0));
	TEST_ASSERT_EQUAL_INT(STS_FWUPD_XPORT_READY,
			      sts_fwupd_xport_classify(1));
	TEST_ASSERT_EQUAL_INT(STS_FWUPD_XPORT_READY,
			      sts_fwupd_xport_classify(64));
}

/** Every state has a distinct, non-empty name — the boot log renders it. */
static void test_every_transport_state_has_a_distinct_name(void)
{
	static const sts_fwupd_xport_t all[] = {
		STS_FWUPD_XPORT_ABSENT,
		STS_FWUPD_XPORT_DOWN,
		STS_FWUPD_XPORT_READY,
	};
	size_t i;
	size_t j;

	for (i = 0U; i < (sizeof(all) / sizeof(all[0])); i++) {
		const char *ni = sts_fwupd_xport_name(all[i]);

		TEST_ASSERT_NOT_NULL(ni);
		TEST_ASSERT_TRUE(strlen(ni) > 0U);
		TEST_ASSERT_NOT_EQUAL_INT(0, strcmp(ni, "?"));

		for (j = i + 1U; j < (sizeof(all) / sizeof(all[0])); j++) {
			TEST_ASSERT_NOT_EQUAL_INT(
				0, strcmp(ni, sts_fwupd_xport_name(all[j])));
		}
	}

	TEST_ASSERT_EQUAL_STRING("?", sts_fwupd_xport_name(
					      (sts_fwupd_xport_t)99));
}

/* ===================================================================== */
/* 3. what a G0 inventory read may do                                    */
/* ===================================================================== */

/** The permitted case, stated once: usable port, nobody else holding it. */
static void test_a_free_and_usable_port_may_be_polled(void)
{
	TEST_ASSERT_EQUAL_INT(STS_FWUPD_QUERY_POLL,
			      sts_fwupd_query_decide(true, false));
}

/**
 * THE finding. An open tunnel refuses the read outright.
 *
 * Without this, an unauthenticated `fw.inventory` injects a UBX-MON-VER poll
 * into a technician's live passthrough session — possibly into a receiver's
 * bootloader — and then spends its reply budget draining that session's bytes.
 * On UART7 the same read latches RB_CAP_NONE for the rest of the uptime.
 */
static void test_an_open_tunnel_refuses_the_read(void)
{
	TEST_ASSERT_EQUAL_INT(STS_FWUPD_QUERY_EMPTY_TUNNEL,
			      sts_fwupd_query_decide(true, true));
}

/** No usable transport is a refusal too, and a differently-named one. */
static void test_an_unusable_transport_refuses_the_read(void)
{
	TEST_ASSERT_EQUAL_INT(STS_FWUPD_QUERY_EMPTY_NO_TRANSPORT,
			      sts_fwupd_query_decide(false, false));
}

/**
 * The refusal ORDER is the contract: the tunnel outranks everything.
 *
 * "No transport" is a capability statement that can go stale in the safe
 * direction; "a tunnel owns this port" is a statement about another live
 * session. When both hold, the answer names the tunnel — so that a future
 * transport whose readiness probe is optimistic can never reach the wire under
 * an open tunnel by winning a race with the other test.
 */
static void test_the_tunnel_outranks_an_unusable_transport(void)
{
	TEST_ASSERT_EQUAL_INT(STS_FWUPD_QUERY_EMPTY_TUNNEL,
			      sts_fwupd_query_decide(false, true));
}

/**
 * Exhaustively: POLL is reachable from exactly one of the four inputs.
 *
 * The shape any weakened gate would take is "one more combination reaches the
 * bus", so the sweep asserts the conjunction rather than the three refusals.
 */
static void test_poll_is_reachable_from_exactly_one_input(void)
{
	unsigned int polls = 0U;
	int ready;
	int tunnel;

	for (ready = 0; ready <= 1; ready++) {
		for (tunnel = 0; tunnel <= 1; tunnel++) {
			sts_fwupd_query_act_t a = sts_fwupd_query_decide(
				ready != 0, tunnel != 0);

			if (a == STS_FWUPD_QUERY_POLL) {
				polls++;
				TEST_ASSERT_EQUAL_INT(1, ready);
				TEST_ASSERT_EQUAL_INT(0, tunnel);
			}
		}
	}
	TEST_ASSERT_EQUAL_UINT(1U, polls);
}

/** Every verdict has a distinct, non-empty name. */
static void test_every_query_verdict_has_a_distinct_name(void)
{
	static const sts_fwupd_query_act_t all[] = {
		STS_FWUPD_QUERY_POLL,
		STS_FWUPD_QUERY_EMPTY_TUNNEL,
		STS_FWUPD_QUERY_EMPTY_NO_TRANSPORT,
	};
	size_t i;
	size_t j;

	for (i = 0U; i < (sizeof(all) / sizeof(all[0])); i++) {
		const char *ni = sts_fwupd_query_act_name(all[i]);

		TEST_ASSERT_NOT_NULL(ni);
		TEST_ASSERT_TRUE(strlen(ni) > 0U);
		TEST_ASSERT_NOT_EQUAL_INT(0, strcmp(ni, "?"));

		for (j = i + 1U; j < (sizeof(all) / sizeof(all[0])); j++) {
			TEST_ASSERT_NOT_EQUAL_INT(
				0, strcmp(ni,
					  sts_fwupd_query_act_name(all[j])));
		}
	}

	TEST_ASSERT_EQUAL_STRING("?", sts_fwupd_query_act_name(
					      (sts_fwupd_query_act_t)99));
}

/* ===================================================================== */
/* 4. the supervisor pass budget                                         */
/* ===================================================================== */

/* The numbers sts_console.c feeds it, restated so this suite fails when they
 * drift rather than tracking them silently. Not #included: mp_glue.h and
 * sts_console.c are Zephyr-side, and the point of this header is that the
 * arithmetic can be checked without them. */
#define AS_BUILT_PERIOD_MS 250U
#define AS_BUILT_LOCK_MS   50U
#define AS_BUILT_MISSES    5U
#define AS_BUILT_DEADLINE  2000U

/** The as-built inequality, with the actual figure spelled out. */
static void test_the_as_built_budget_fits_the_deadman(void)
{
	unsigned int worst = STS_FWUPD_PASS_BUDGET_MS(AS_BUILT_PERIOD_MS,
						      AS_BUILT_LOCK_MS,
						      AS_BUILT_MISSES,
						      STS_FWUPD_STEP_BUDGET_MS);

	/* 6 * (250 + 50) + 150 */
	TEST_ASSERT_EQUAL_UINT(1950U, worst);
	TEST_ASSERT_TRUE_MESSAGE(worst <= AS_BUILT_DEADLINE,
				 "the supervisor pass budget no longer fits "
				 "MP_TICK_MAX_MS");
}

/**
 * The work term is added ONCE, not per pass, and that is a decision.
 *
 * core/fwupd runs restore() exactly once per prepared session, so at most one
 * pass inside any dead-man window carries the long work. Multiplying it through
 * would be arithmetically safer and would make the inequality unsatisfiable at
 * any useful budget; dropping it entirely is the bug this term was added to
 * fix. Both directions are pinned here.
 */
static void test_the_work_term_is_counted_exactly_once(void)
{
	unsigned int without = STS_FWUPD_PASS_BUDGET_MS(
		AS_BUILT_PERIOD_MS, AS_BUILT_LOCK_MS, AS_BUILT_MISSES, 0U);
	unsigned int with = STS_FWUPD_PASS_BUDGET_MS(AS_BUILT_PERIOD_MS,
						     AS_BUILT_LOCK_MS,
						     AS_BUILT_MISSES, 150U);

	/* The pre-existing inequality, unchanged when the work term is zero. */
	TEST_ASSERT_EQUAL_UINT(1800U, without);
	/* Exactly one work term, not (misses + 1) of them. */
	TEST_ASSERT_EQUAL_UINT(150U, with - without);
	TEST_ASSERT_NOT_EQUAL_UINT((AS_BUILT_MISSES + 1U) * 150U,
				   with - without);
}

/** The per-pass terms ARE multiplied by the tolerated run of misses. */
static void test_the_pass_terms_are_counted_per_pass(void)
{
	TEST_ASSERT_EQUAL_UINT(
		2U * (100U + 10U),
		STS_FWUPD_PASS_BUDGET_MS(100U, 10U, 1U, 0U));
	TEST_ASSERT_EQUAL_UINT(
		1U * (100U + 10U),
		STS_FWUPD_PASS_BUDGET_MS(100U, 10U, 0U, 0U));

	/* One more tolerated miss costs one more whole pass. */
	TEST_ASSERT_EQUAL_UINT(
		110U,
		STS_FWUPD_PASS_BUDGET_MS(100U, 10U, 2U, 0U) -
			STS_FWUPD_PASS_BUDGET_MS(100U, 10U, 1U, 0U));
}

/** Monotone in every argument: no term can be raised for free. */
static void test_the_budget_is_monotone_in_every_argument(void)
{
	unsigned int base = STS_FWUPD_PASS_BUDGET_MS(250U, 50U, 5U, 150U);

	TEST_ASSERT_TRUE(STS_FWUPD_PASS_BUDGET_MS(251U, 50U, 5U, 150U) > base);
	TEST_ASSERT_TRUE(STS_FWUPD_PASS_BUDGET_MS(250U, 51U, 5U, 150U) > base);
	TEST_ASSERT_TRUE(STS_FWUPD_PASS_BUDGET_MS(250U, 50U, 6U, 150U) > base);
	TEST_ASSERT_TRUE(STS_FWUPD_PASS_BUDGET_MS(250U, 50U, 5U, 151U) > base);
}

/**
 * The budget is big enough for the two long paths it was sized against.
 *
 * The GNSS restore path is the one that can be written down exactly:
 * ubx_fwupd_recover() holds UBX_FWUPD_SAFEBOOT_SETUP_MS then
 * UBX_FWUPD_RESET_HOLD_MS, both served by k_msleep(). The STM32 trailer write
 * has no datasheet figure this suite can assert, which is why fwupd_glue.c
 * measures it at runtime — but the budget must at least clear the path whose
 * cost IS known, with room for the flash write beside it.
 */
static void test_the_step_budget_clears_the_known_long_path(void)
{
	/* ubx_fwupd.h: 10 ms safeboot setup + 10 ms reset hold. */
	const unsigned int gnss_recover_ms = 10U + 10U;

	TEST_ASSERT_TRUE_MESSAGE(STS_FWUPD_STEP_BUDGET_MS > gnss_recover_ms,
				 "the step budget no longer covers "
				 "ubx_fwupd_recover()'s pin sequence");
	/* And with enough margin left over for the flash write beside it. */
	TEST_ASSERT_TRUE(STS_FWUPD_STEP_BUDGET_MS >= (4U * gnss_recover_ms));
}

/* ===================================================================== */
/* 5. the staging erase window                                           */
/* ===================================================================== */

/* The as-built slot 1 geometry, restated so this suite fails when it drifts.
 * 896 KiB of 8 KiB sectors, 16 B write block; sts_stage_geom.h derives 110
 * usable sectors from MCUboot's own arithmetic (root CLAUDE.md, sts_dfu.c). */
#define SLOT_GRAN   8192U
#define SLOT_USABLE (110U * SLOT_GRAN) /* 901120 */
/* fwupd.h FWUPD_CHUNK_MAX, and stm_chunk_max()'s own ceiling. */
#define STM_CHUNK   1024U

/** align_up is the identity on multiples and never rounds down. */
static void test_align_up_rounds_up_and_only_up(void)
{
	TEST_ASSERT_EQUAL_UINT32(0U, sts_fwupd_align_up(0U, SLOT_GRAN));
	TEST_ASSERT_EQUAL_UINT32(SLOT_GRAN, sts_fwupd_align_up(1U, SLOT_GRAN));
	TEST_ASSERT_EQUAL_UINT32(SLOT_GRAN,
				 sts_fwupd_align_up(SLOT_GRAN, SLOT_GRAN));
	TEST_ASSERT_EQUAL_UINT32(2U * SLOT_GRAN,
				 sts_fwupd_align_up(SLOT_GRAN + 1U, SLOT_GRAN));

	/* A granularity of 0 or 1 is the identity rather than a division by
	 * zero: the helper is total, so a caller cannot make it trap. */
	TEST_ASSERT_EQUAL_UINT32(1234U, sts_fwupd_align_up(1234U, 0U));
	TEST_ASSERT_EQUAL_UINT32(1234U, sts_fwupd_align_up(1234U, 1U));
}

/**
 * The saturation, which is the one arithmetic accident that would be silent.
 *
 * A wrapped round-up turns "erase up to here" into a target BELOW the cursor,
 * and the caller's `target <= erased_to` test then skips the erase entirely —
 * i.e. exactly the defect this whole decision exists to fix, reintroduced by an
 * overflow nobody would look for.
 */
static void test_align_up_saturates_instead_of_wrapping(void)
{
	TEST_ASSERT_EQUAL_UINT32(UINT32_MAX,
				 sts_fwupd_align_up(UINT32_MAX, SLOT_GRAN));
	TEST_ASSERT_EQUAL_UINT32(UINT32_MAX,
				 sts_fwupd_align_up(UINT32_MAX - 1U, SLOT_GRAN));
	TEST_ASSERT_TRUE(sts_fwupd_align_up(UINT32_MAX, SLOT_GRAN) >=
			 (UINT32_MAX - 1U));
}

/** The ceiling is the image rounded up, when the slot has room to spare. */
static void test_the_ceiling_is_the_image_rounded_up(void)
{
	/* A 740 KiB image in the 880 KiB slot: 93 sectors, not 110. */
	TEST_ASSERT_EQUAL_UINT32(93U * SLOT_GRAN,
				 sts_fwupd_erase_ceiling(740U * 1024U,
							 SLOT_USABLE, SLOT_GRAN));
	TEST_ASSERT_EQUAL_UINT32(SLOT_GRAN,
				 sts_fwupd_erase_ceiling(1U, SLOT_USABLE,
							 SLOT_GRAN));
	TEST_ASSERT_EQUAL_UINT32(0U,
				 sts_fwupd_erase_ceiling(0U, SLOT_USABLE,
							 SLOT_GRAN));
}

/**
 * …and never the whole slot, which is the point.
 *
 * A ceiling that ignored the image would blank ~160 KiB of sectors the transfer
 * never touches, at one engine-lock hold each, for no benefit whatsoever.
 */
static void test_the_ceiling_does_not_blank_sectors_the_image_never_reaches(void)
{
	uint32_t c = sts_fwupd_erase_ceiling(740U * 1024U, SLOT_USABLE,
					     SLOT_GRAN);

	TEST_ASSERT_TRUE_MESSAGE(c < SLOT_USABLE,
				 "the erase window covers the whole slot rather "
				 "than the image");
	TEST_ASSERT_EQUAL_UINT32(0U, c % SLOT_GRAN);
}

/**
 * THE dangerous edge: the ceiling never passes staging_size().
 *
 * Above it lie the MCUboot trailer region and the swap-using-move free sector
 * (sts_stage_geom.h). Erasing into them does not fail the upload — it produces
 * an image that verifies, marks pending, and is then silently declined by
 * swap_move() on every subsequent boot, in a bootloader built with
 * CONFIG_MCUBOOT_LOG_LEVEL_OFF.
 */
static void test_the_ceiling_never_passes_the_usable_region(void)
{
	uint32_t size;

	/* An image filling the slot exactly: the round-up must not add a sector
	 * that belongs to the trailer. */
	TEST_ASSERT_EQUAL_UINT32(SLOT_USABLE,
				 sts_fwupd_erase_ceiling(SLOT_USABLE,
							 SLOT_USABLE, SLOT_GRAN));

	/* Every size in the top sector, one octet at a time, is clamped. */
	for (size = SLOT_USABLE - SLOT_GRAN + 1U; size <= SLOT_USABLE; size++) {
		TEST_ASSERT_EQUAL_UINT32(SLOT_USABLE,
					 sts_fwupd_erase_ceiling(size,
								 SLOT_USABLE,
								 SLOT_GRAN));
	}

	/*
	 * And a capacity that is NOT a whole number of sectors is still clamped
	 * to the capacity, never rounded past it. sts_staging_usable() cannot
	 * produce one today, but "the clamp wins over the round-up" has to be
	 * the property rather than "the two happen to agree".
	 */
	TEST_ASSERT_EQUAL_UINT32(SLOT_USABLE - 1U,
				 sts_fwupd_erase_ceiling(SLOT_USABLE - 1U,
							 SLOT_USABLE - 1U,
							 SLOT_GRAN));
}

/** The ceiling always covers the image, so a clamp can never starve a chunk. */
static void test_the_ceiling_always_covers_the_declared_image(void)
{
	uint32_t size;

	for (size = 0U; size <= SLOT_USABLE; size += 997U) {
		TEST_ASSERT_TRUE_MESSAGE(
			sts_fwupd_erase_ceiling(size, SLOT_USABLE, SLOT_GRAN) >=
				size,
			"the erase ceiling fell below the declared image size");
	}
	TEST_ASSERT_TRUE(sts_fwupd_erase_ceiling(SLOT_USABLE, SLOT_USABLE,
						 SLOT_GRAN) >= SLOT_USABLE);
}

/** One granule of look-ahead: the target is always past what is needed. */
static void test_the_target_stands_one_granule_past_the_chunk(void)
{
	uint32_t ceiling = sts_fwupd_erase_ceiling(SLOT_USABLE, SLOT_USABLE,
						   SLOT_GRAN);

	/* prepare()'s pre-clear: nothing needed yet, one sector blanked. */
	TEST_ASSERT_EQUAL_UINT32(SLOT_GRAN,
				 sts_fwupd_erase_target(0U, SLOT_GRAN, ceiling));
	/* The first chunk lands inside sector 0 and pulls sector 1 forward. */
	TEST_ASSERT_EQUAL_UINT32(2U * SLOT_GRAN,
				 sts_fwupd_erase_target(STM_CHUNK, SLOT_GRAN,
							ceiling));
	/* Exactly on a boundary is still one whole granule beyond. */
	TEST_ASSERT_EQUAL_UINT32(2U * SLOT_GRAN,
				 sts_fwupd_erase_target(SLOT_GRAN, SLOT_GRAN,
							ceiling));
	TEST_ASSERT_EQUAL_UINT32(3U * SLOT_GRAN,
				 sts_fwupd_erase_target(SLOT_GRAN + 1U,
							SLOT_GRAN, ceiling));
}

/**
 * The transfer's whole walk: every chunk is covered, no call erases more than
 * one sector, and the cursor stops at the ceiling.
 *
 * The three properties together are the decision. Coverage alone would be
 * satisfied by erasing the slot at prepare(); the one-sector bound alone would
 * be satisfied by never erasing at all.
 */
static void test_a_whole_transfer_erases_one_sector_at_a_time(void)
{
	const uint32_t size = 740U * 1024U;
	uint32_t ceiling = sts_fwupd_erase_ceiling(size, SLOT_USABLE, SLOT_GRAN);
	uint32_t cursor = 0U;
	uint32_t off;
	unsigned int erases = 0U;

	/* prepare()'s pre-clear, exactly as stm_prepare() does it. */
	cursor = sts_fwupd_erase_target(0U, SLOT_GRAN, ceiling);
	TEST_ASSERT_EQUAL_UINT32(SLOT_GRAN, cursor);
	erases++;

	for (off = 0U; off < size; off += STM_CHUNK) {
		uint32_t len = ((size - off) < STM_CHUNK) ? (size - off)
							  : STM_CHUNK;
		uint32_t target = sts_fwupd_erase_target(off + len, SLOT_GRAN,
							 ceiling);
		char msg[128];

		if (target <= cursor) {
			/* Already blank: the common case, seven chunks in
			 * eight, and it must cost nothing. */
			TEST_ASSERT_TRUE_MESSAGE(
				(off + len) <= cursor,
				"a chunk was written into flash the cursor had "
				"not blanked");
			continue;
		}

		(void)snprintf(msg, sizeof(msg),
			       "one fw.data at off %u erased %u octets — more "
			       "than the one granule the engine-lock budget "
			       "assumes",
			       (unsigned int)off,
			       (unsigned int)(target - cursor));
		TEST_ASSERT_TRUE_MESSAGE(((target - cursor) <= SLOT_GRAN), msg);
		cursor = target;
		erases++;

		TEST_ASSERT_TRUE_MESSAGE(
			(off + len) <= cursor,
			"a chunk was written into flash the cursor had not "
			"blanked");
	}

	/* Every octet of the image is inside the blanked region... */
	TEST_ASSERT_TRUE(cursor >= size);
	/* ...and not one octet past the ceiling. */
	TEST_ASSERT_EQUAL_UINT32(ceiling, cursor);
	TEST_ASSERT_TRUE_MESSAGE(cursor <= SLOT_USABLE,
				 "the transfer erased into the MCUboot trailer");
	/* 93 sectors of image, blanked in 93 calls out of the 740 that carried
	 * it — the pre-clear plus one per sector boundary crossed. */
	TEST_ASSERT_EQUAL_UINT(93U, erases);
}

/**
 * A rewind erases nothing.
 *
 * core/fwupd absorbs a retransmit before transfer() is reached, but a session
 * restarted at the same size replays prepare() — and if that re-erased from 0
 * it would blank octets the tool had already been told were accepted.
 */
static void test_a_target_at_or_below_the_cursor_erases_nothing(void)
{
	uint32_t ceiling = sts_fwupd_erase_ceiling(SLOT_USABLE, SLOT_USABLE,
						   SLOT_GRAN);
	uint32_t cursor = sts_fwupd_erase_target(64U * SLOT_GRAN, SLOT_GRAN,
						 ceiling);

	TEST_ASSERT_EQUAL_UINT32(65U * SLOT_GRAN, cursor);
	/* Every offset already inside the blanked region asks for no erase. */
	TEST_ASSERT_TRUE(sts_fwupd_erase_target(0U, SLOT_GRAN, ceiling) <=
			 cursor);
	TEST_ASSERT_TRUE(sts_fwupd_erase_target(63U * SLOT_GRAN, SLOT_GRAN,
						ceiling) <= cursor);
	/* The first offset that does need one is the boundary itself. */
	TEST_ASSERT_TRUE(sts_fwupd_erase_target((64U * SLOT_GRAN) + 1U,
						SLOT_GRAN, ceiling) > cursor);
}

/** The look-ahead never carries the cursor past the ceiling. */
static void test_the_look_ahead_is_clamped_at_the_ceiling(void)
{
	uint32_t ceiling = sts_fwupd_erase_ceiling(SLOT_USABLE, SLOT_USABLE,
						   SLOT_GRAN);
	uint32_t need;

	/* The last sector of the image: the look-ahead wants one more and there
	 * is none to give. */
	for (need = SLOT_USABLE - SLOT_GRAN; need <= SLOT_USABLE; need++) {
		TEST_ASSERT_EQUAL_UINT32(ceiling,
					 sts_fwupd_erase_target(need, SLOT_GRAN,
								ceiling));
	}

	/* And an absurd request — which core/fwupd's range checks make
	 * unreachable — is still clamped rather than wrapped. */
	TEST_ASSERT_EQUAL_UINT32(ceiling,
				 sts_fwupd_erase_target(UINT32_MAX, SLOT_GRAN,
							ceiling));
	TEST_ASSERT_EQUAL_UINT32(ceiling,
				 sts_fwupd_erase_target(UINT32_MAX - 1U,
							SLOT_GRAN, ceiling));
}

/**
 * The per-call bound holds for every geometry, not just this board's.
 *
 * The engine-lock argument in fwupd_glue.c is "at most one sector per chunk",
 * and that rests on the chunk being no larger than a granule. A part with small
 * sectors breaks the premise, so the general bound — ceil(chunk/gran) + 1
 * granules — is what is asserted, across a sweep that includes granules both
 * larger and smaller than the chunk.
 */
static void test_the_per_call_erase_is_bounded_for_every_geometry(void)
{
	static const uint32_t grans[] = { 256U, 512U, 1024U, 2048U, 4096U,
					  8192U, 65536U, 131072U };
	size_t g;

	for (g = 0U; g < (sizeof(grans) / sizeof(grans[0])); g++) {
		uint32_t gran = grans[g];
		const uint32_t size = 300U * 1024U;
		uint32_t ceiling = sts_fwupd_erase_ceiling(size, SLOT_USABLE,
							   gran);
		uint32_t bound = (((STM_CHUNK + gran - 1U) / gran) + 1U) * gran;
		uint32_t cursor = sts_fwupd_erase_target(0U, gran, ceiling);
		uint32_t off;
		char msg[160];

		for (off = 0U; off < size; off += STM_CHUNK) {
			uint32_t len = ((size - off) < STM_CHUNK)
					       ? (size - off) : STM_CHUNK;
			uint32_t target = sts_fwupd_erase_target(off + len,
								 gran, ceiling);

			if (target <= cursor) {
				continue;
			}
			(void)snprintf(msg, sizeof(msg),
				       "gran %u: one call erased %u octets, "
				       "over the %u-octet bound",
				       (unsigned int)gran,
				       (unsigned int)(target - cursor),
				       (unsigned int)bound);
			TEST_ASSERT_TRUE_MESSAGE((target - cursor) <= bound,
						 msg);
			cursor = target;
			TEST_ASSERT_TRUE_MESSAGE((off + len) <= cursor,
						 "a chunk outran the cursor");
		}
		TEST_ASSERT_TRUE(cursor >= size);
		TEST_ASSERT_TRUE(cursor <= SLOT_USABLE);
	}
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_a_full_write_is_success_not_a_count);
	RUN_TEST(test_an_empty_burst_is_success);
	RUN_TEST(test_a_short_write_fails);
	RUN_TEST(test_an_over_long_count_fails);
	RUN_TEST(test_a_negative_errno_passes_through);
	RUN_TEST(test_only_an_exact_match_succeeds);

	RUN_TEST(test_only_notsup_means_absent);
	RUN_TEST(test_the_area_not_started_is_neither_absent_nor_ready);
	RUN_TEST(test_a_non_negative_probe_is_ready);
	RUN_TEST(test_every_transport_state_has_a_distinct_name);

	RUN_TEST(test_a_free_and_usable_port_may_be_polled);
	RUN_TEST(test_an_open_tunnel_refuses_the_read);
	RUN_TEST(test_an_unusable_transport_refuses_the_read);
	RUN_TEST(test_the_tunnel_outranks_an_unusable_transport);
	RUN_TEST(test_poll_is_reachable_from_exactly_one_input);
	RUN_TEST(test_every_query_verdict_has_a_distinct_name);

	RUN_TEST(test_the_as_built_budget_fits_the_deadman);
	RUN_TEST(test_the_work_term_is_counted_exactly_once);
	RUN_TEST(test_the_pass_terms_are_counted_per_pass);
	RUN_TEST(test_the_budget_is_monotone_in_every_argument);
	RUN_TEST(test_the_step_budget_clears_the_known_long_path);

	RUN_TEST(test_align_up_rounds_up_and_only_up);
	RUN_TEST(test_align_up_saturates_instead_of_wrapping);
	RUN_TEST(test_the_ceiling_is_the_image_rounded_up);
	RUN_TEST(test_the_ceiling_does_not_blank_sectors_the_image_never_reaches);
	RUN_TEST(test_the_ceiling_never_passes_the_usable_region);
	RUN_TEST(test_the_ceiling_always_covers_the_declared_image);
	RUN_TEST(test_the_target_stands_one_granule_past_the_chunk);
	RUN_TEST(test_a_whole_transfer_erases_one_sector_at_a_time);
	RUN_TEST(test_a_target_at_or_below_the_cursor_erases_nothing);
	RUN_TEST(test_the_look_ahead_is_clamped_at_the_ceiling);
	RUN_TEST(test_the_per_call_erase_is_bounded_for_every_geometry);

	return UNITY_END();
}
