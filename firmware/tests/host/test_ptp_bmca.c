/*
 * STS1000 "Meridian" — core/ptp best-master-clock unit tests.
 *
 * Provenance of the expectations:
 *
 *  - The part-1 truth table is IEEE 1588-2019 §9.3.4 Figure 34, walked rung by
 *    rung: priority1, clockClass, clockAccuracy, offsetScaledLogVariance,
 *    priority2, grandmasterIdentity. Each case sets exactly one rung in A's
 *    favour and, where it matters, a *lower* rung in B's, so a comparator that
 *    ordered the rungs wrongly fails rather than passes by luck.
 *
 *  - The part-2 cases are Figure 35, including the ±1 dead band on stepsRemoved
 *    and both error outcomes.
 *
 *  - The state-decision expectations are Figure 33, with the M1/M2/M3/P1/P2/S1
 *    labels the standard itself uses.
 *
 *  - The foreign-master table follows §9.3.2.4: FOREIGN_MASTER_THRESHOLD
 *    Announces inside FOREIGN_MASTER_TIME_WINDOW announce intervals qualify a
 *    master; silence for announceReceiptTimeout intervals drops it.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "ptp/ptp.h"
#include "test_support.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* --------------------------------------------------------------- fixtures -- */

static void set_id(ptp_clock_id_t *id, uint8_t last)
{
	static const uint8_t base[8] = { 0x00, 0x80, 0xE1, 0xFF, 0xFE, 0x00, 0x00, 0x00 };

	memcpy(id->id, base, sizeof(base));
	id->id[7] = last;
}

static void set_port(ptp_port_id_t *p, uint8_t last, uint16_t num)
{
	set_id(&p->clock_id, last);
	p->port_number = num;
}

/* A neutral dataset: identical on both sides until a test perturbs one rung. */
static void base_ds(ptp_dataset_t *d, uint8_t gm_last)
{
	memset(d, 0, sizeof(*d));
	d->priority1 = 128U;
	d->quality.clock_class = 6U;
	d->quality.clock_accuracy = 0x20U;
	d->quality.offset_scaled_log_variance = 0x4E5DU;
	d->priority2 = 128U;
	set_id(&d->gm_identity, gm_last);
	d->steps_removed = 0U;
	set_port(&d->sender, gm_last, 1U);
	set_port(&d->receiver, 0xEEU, 1U);
}

/* ------------------------------------------------------- part 1 truth table -- */

/*
 * Each row makes A better at `rung`, and simultaneously makes A *worse* at
 * every rung below it. If the comparator checked the rungs out of order, the
 * lower-priority attribute would flip the answer and the row would fail.
 */
typedef enum {
	RUNG_PRIORITY1 = 0,
	RUNG_CLASS,
	RUNG_ACCURACY,
	RUNG_VARIANCE,
	RUNG_PRIORITY2,
	RUNG_IDENTITY,
	RUNG__COUNT,
} rung_t;

static void apply_rung(ptp_dataset_t *a, ptp_dataset_t *b, rung_t rung)
{
	/* Below the decisive rung, A is deliberately the worse clock. */
	if (rung < RUNG_CLASS) {
		a->quality.clock_class = 7U;
		b->quality.clock_class = 6U;
	}
	if (rung < RUNG_ACCURACY) {
		a->quality.clock_accuracy = 0x21U;
		b->quality.clock_accuracy = 0x1FU;
	}
	if (rung < RUNG_VARIANCE) {
		a->quality.offset_scaled_log_variance = 0xF000U;
		b->quality.offset_scaled_log_variance = 0x1000U;
	}
	if (rung < RUNG_PRIORITY2) {
		a->priority2 = 200U;
		b->priority2 = 10U;
	}

	switch (rung) {
	case RUNG_PRIORITY1:
		a->priority1 = 10U;
		b->priority1 = 200U;
		break;
	case RUNG_CLASS:
		a->quality.clock_class = 6U;
		b->quality.clock_class = 7U;
		break;
	case RUNG_ACCURACY:
		a->quality.clock_accuracy = 0x1FU;
		b->quality.clock_accuracy = 0x21U;
		break;
	case RUNG_VARIANCE:
		a->quality.offset_scaled_log_variance = 0x1000U;
		b->quality.offset_scaled_log_variance = 0xF000U;
		break;
	case RUNG_PRIORITY2:
		a->priority2 = 10U;
		b->priority2 = 200U;
		break;
	case RUNG_IDENTITY:
		/* set_id() already made A's identity the numerically smaller. */
		break;
	case RUNG__COUNT:
	default:
		TEST_FAIL_MESSAGE("bad rung");
		break;
	}
}

static void test_part1_each_rung_decides_in_order(void)
{
	static const char *const names[RUNG__COUNT] = {
		"priority1", "clockClass", "clockAccuracy",
		"offsetScaledLogVariance", "priority2", "grandmasterIdentity",
	};
	rung_t rung;

	for (rung = RUNG_PRIORITY1; rung < RUNG__COUNT; rung++) {
		ptp_dataset_t a;
		ptp_dataset_t b;

		base_ds(&a, 0x01U);
		base_ds(&b, 0x02U);
		apply_rung(&a, &b, rung);

		TEST_ASSERT_EQUAL_INT_MESSAGE(PTP_DSCMP_A_BETTER,
					      ptp_bmca_compare(&a, &b), names[rung]);
		/* The relation must be antisymmetric. */
		TEST_ASSERT_EQUAL_INT_MESSAGE(PTP_DSCMP_B_BETTER,
					      ptp_bmca_compare(&b, &a), names[rung]);
		TEST_ASSERT_TRUE(ptp_dscmp_a_wins(ptp_bmca_compare(&a, &b)));
		TEST_ASSERT_TRUE(ptp_dscmp_b_wins(ptp_bmca_compare(&b, &a)));
	}
}

static void test_part1_identity_tiebreak_is_total(void)
{
	ptp_dataset_t a;
	ptp_dataset_t b;

	/* Everything equal except the grandmaster identity: lower wins. */
	base_ds(&a, 0x01U);
	base_ds(&b, 0x02U);
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_A_BETTER, ptp_bmca_compare(&a, &b));
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_B_BETTER, ptp_bmca_compare(&b, &a));

	/* Ordering is lexicographic over the whole EUI-64, not just the last octet. */
	base_ds(&a, 0x01U);
	base_ds(&b, 0x01U);
	a.gm_identity.id[0] = 0x00U;
	b.gm_identity.id[0] = 0x01U;
	a.gm_identity.id[7] = 0xFFU;
	b.gm_identity.id[7] = 0x00U;
	/* The identities now differ, so this is part 1 with the identity tiebreak. */
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_A_BETTER, ptp_bmca_compare(&a, &b));
}

static void test_part1_lower_rung_cannot_override_higher(void)
{
	ptp_dataset_t a;
	ptp_dataset_t b;

	/* A has a far better clockClass but a worse priority1: priority1 wins. */
	base_ds(&a, 0x01U);
	base_ds(&b, 0x02U);
	a.priority1 = 200U;
	b.priority1 = 128U;
	a.quality.clock_class = 6U;
	b.quality.clock_class = 248U;
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_B_BETTER, ptp_bmca_compare(&a, &b));

	/* A has priority2 10 vs 200 but a worse variance: variance wins. */
	base_ds(&a, 0x01U);
	base_ds(&b, 0x02U);
	a.priority2 = 10U;
	b.priority2 = 200U;
	a.quality.offset_scaled_log_variance = 0xFFFEU;
	b.quality.offset_scaled_log_variance = 0x0001U;
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_B_BETTER, ptp_bmca_compare(&a, &b));
}

static void test_compare_rejects_null(void)
{
	ptp_dataset_t a;

	base_ds(&a, 0x01U);
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_ERROR_2, ptp_bmca_compare(NULL, &a));
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_ERROR_2, ptp_bmca_compare(&a, NULL));
	TEST_ASSERT_FALSE(ptp_dscmp_a_wins(PTP_DSCMP_ERROR_2));
	TEST_ASSERT_FALSE(ptp_dscmp_b_wins(PTP_DSCMP_ERROR_2));
	TEST_ASSERT_FALSE(ptp_dscmp_a_wins(PTP_DSCMP_ERROR_1));
	TEST_ASSERT_FALSE(ptp_dscmp_b_wins(PTP_DSCMP_ERROR_1));
}

/* --------------------------------------------------------------- part 2 --- */

/*
 * Same grandmaster on both sides; only the topology differs.
 *
 * The receivers deliberately sort *below* both senders. Figure 35's
 * "stepsRemoved differ by one" leg compares the further dataset's receiver
 * against its own sender: receiver below sender gives better-by-topology,
 * receiver above sender gives an outright win, and equal is ERROR-1. All three
 * legs are exercised, this fixture just fixes the common one.
 */
static void same_gm(ptp_dataset_t *a, ptp_dataset_t *b)
{
	base_ds(a, 0x01U);
	base_ds(b, 0x01U);
	set_port(&a->sender, 0x10U, 1U);
	set_port(&b->sender, 0x20U, 1U);
	set_port(&a->receiver, 0x01U, 1U);
	set_port(&b->receiver, 0x01U, 1U);
}

static void test_part2_steps_removed_dead_band(void)
{
	ptp_dataset_t a;
	ptp_dataset_t b;

	/* Two or more hops apart: the closer one wins outright. */
	same_gm(&a, &b);
	a.steps_removed = 0U;
	b.steps_removed = 2U;
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_A_BETTER, ptp_bmca_compare(&a, &b));

	same_gm(&a, &b);
	a.steps_removed = 2U;
	b.steps_removed = 0U;
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_B_BETTER, ptp_bmca_compare(&a, &b));

	/*
	 * Exactly one hop apart is inside the dead band: the decision falls
	 * through to topology, not to the raw hop count. The further dataset's
	 * receiver (0x01) sorts below its sender, so the closer one wins by
	 * topology rather than outright.
	 */
	same_gm(&a, &b);
	a.steps_removed = 0U;
	b.steps_removed = 1U;
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_A_BETTER_TOPO, ptp_bmca_compare(&a, &b));

	same_gm(&a, &b);
	a.steps_removed = 1U;
	b.steps_removed = 0U;
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_B_BETTER_TOPO, ptp_bmca_compare(&a, &b));

	/* The boundary is strict: 3 vs 1 is outside, 2 vs 1 is inside. */
	same_gm(&a, &b);
	a.steps_removed = 3U;
	b.steps_removed = 1U;
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_B_BETTER, ptp_bmca_compare(&a, &b));

	same_gm(&a, &b);
	a.steps_removed = 2U;
	b.steps_removed = 1U;
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_B_BETTER_TOPO, ptp_bmca_compare(&a, &b));

	same_gm(&a, &b);
	a.steps_removed = 1U;
	b.steps_removed = 3U;
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_A_BETTER, ptp_bmca_compare(&a, &b));
}

static void test_part2_receiver_above_sender_is_not_topology(void)
{
	ptp_dataset_t a;
	ptp_dataset_t b;

	/*
	 * Figure 35's "receiver > sender" leg: when the further dataset's
	 * receiver sorts *below* its sender, the closer one is better outright
	 * rather than better-by-topology.
	 */
	same_gm(&a, &b);
	a.steps_removed = 0U;
	b.steps_removed = 1U;
	set_port(&b.sender, 0x20U, 1U);
	set_port(&b.receiver, 0xEEU, 1U); /* receiver above sender */
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_A_BETTER, ptp_bmca_compare(&a, &b));

	same_gm(&a, &b);
	a.steps_removed = 1U;
	b.steps_removed = 0U;
	set_port(&a.sender, 0x20U, 1U);
	set_port(&a.receiver, 0xEEU, 1U);
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_B_BETTER, ptp_bmca_compare(&a, &b));
}

static void test_part2_error_1_hearing_yourself(void)
{
	ptp_dataset_t a;
	ptp_dataset_t b;

	/* B's receiver *is* B's sender: it is listening to its own Announce. */
	same_gm(&a, &b);
	a.steps_removed = 0U;
	b.steps_removed = 1U;
	set_port(&b.sender, 0x20U, 3U);
	set_port(&b.receiver, 0x20U, 3U);
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_ERROR_1, ptp_bmca_compare(&a, &b));

	same_gm(&a, &b);
	a.steps_removed = 1U;
	b.steps_removed = 0U;
	set_port(&a.sender, 0x20U, 3U);
	set_port(&a.receiver, 0x20U, 3U);
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_ERROR_1, ptp_bmca_compare(&a, &b));
}

static void test_part2_equal_steps_orders_on_sender_then_receiver(void)
{
	ptp_dataset_t a;
	ptp_dataset_t b;

	same_gm(&a, &b);
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_A_BETTER_TOPO, ptp_bmca_compare(&a, &b));
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_B_BETTER_TOPO, ptp_bmca_compare(&b, &a));

	/* Identical senders: the receiving port number decides. */
	same_gm(&a, &b);
	set_port(&a.sender, 0x10U, 1U);
	set_port(&b.sender, 0x10U, 1U);
	a.receiver.port_number = 1U;
	b.receiver.port_number = 2U;
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_A_BETTER_TOPO, ptp_bmca_compare(&a, &b));
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_B_BETTER_TOPO, ptp_bmca_compare(&b, &a));

	/* The sender port number is part of the sender comparison. */
	same_gm(&a, &b);
	set_port(&a.sender, 0x10U, 1U);
	set_port(&b.sender, 0x10U, 2U);
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_A_BETTER_TOPO, ptp_bmca_compare(&a, &b));
}

static void test_part2_error_2_same_everything(void)
{
	ptp_dataset_t a;
	ptp_dataset_t b;

	same_gm(&a, &b);
	set_port(&a.sender, 0x10U, 1U);
	set_port(&b.sender, 0x10U, 1U);
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_ERROR_2, ptp_bmca_compare(&a, &b));
}

/* ------------------------------------------------------- state decision --- */

static void test_state_decision_listening_holds_without_announce(void)
{
	ptp_dataset_t d0;

	base_ds(&d0, 0x01U);
	TEST_ASSERT_EQUAL_INT(PTP_REC_LISTENING,
		ptp_bmca_state_decision(&d0, NULL, NULL, false, true));

	/* Once the receipt timeout expires the hold is lifted and we take over. */
	TEST_ASSERT_EQUAL_INT(PTP_REC_M1,
		ptp_bmca_state_decision(&d0, NULL, NULL, false, false));
}

static void test_state_decision_m1_and_p1(void)
{
	ptp_dataset_t d0;
	ptp_dataset_t other;

	/* clockClass 6 is inside the 1..127 "never a slave" band. */
	base_ds(&d0, 0x01U);
	base_ds(&other, 0x02U);
	TEST_ASSERT_EQUAL_INT(6U, d0.quality.clock_class);

	TEST_ASSERT_EQUAL_INT(PTP_REC_M1,
		ptp_bmca_state_decision(&d0, &other, &other, true, false));

	/* A better foreign master: P1, never S1, because we are class 1..127. */
	other.priority1 = 1U;
	TEST_ASSERT_EQUAL_INT(PTP_REC_P1,
		ptp_bmca_state_decision(&d0, &other, &other, true, false));

	/* The LISTENING hold does not apply once an Announce has been heard. */
	TEST_ASSERT_EQUAL_INT(PTP_REC_P1,
		ptp_bmca_state_decision(&d0, &other, &other, true, true));
}

static void test_state_decision_m2_and_s1(void)
{
	ptp_dataset_t d0;
	ptp_dataset_t other;

	/*
	 * Degradation alternative B (class 187) is the only way this appliance
	 * ever leaves the 1..127 band, and therefore the only way the M2/S1
	 * branch becomes reachable. Alternative A (52) stays below 128 and keeps
	 * the port in M1/P1 — see test_state_decision_degrade_a_never_slaves().
	 */
	base_ds(&d0, 0x01U);
	d0.quality.clock_class = 187U;
	base_ds(&other, 0x02U);
	other.quality.clock_class = 200U;

	TEST_ASSERT_EQUAL_INT(PTP_REC_M2,
		ptp_bmca_state_decision(&d0, &other, &other, true, false));

	/* Now the peer is genuinely better, and it is on this port. */
	other.quality.clock_class = 6U;
	TEST_ASSERT_EQUAL_INT(PTP_REC_S1,
		ptp_bmca_state_decision(&d0, &other, &other, true, false));
}

static void test_state_decision_m3_and_p2(void)
{
	ptp_dataset_t d0;
	ptp_dataset_t erbest;
	ptp_dataset_t ebest;

	/*
	 * Multi-port shape: the clock-wide best arrived on some other port. Not
	 * reachable on this single-port appliance, but the algorithm implements
	 * it, so it is tested directly.
	 */
	base_ds(&d0, 0x01U);
	d0.quality.clock_class = 187U;

	/* Same grandmaster on both sides so part 2 decides. */
	base_ds(&ebest, 0x02U);
	base_ds(&erbest, 0x02U);
	ebest.quality.clock_class = 6U;
	erbest.quality.clock_class = 6U;

	/* Ebest one hop closer, Erbest's receiver below its sender: topology. */
	ebest.steps_removed = 0U;
	erbest.steps_removed = 1U;
	set_port(&ebest.sender, 0x10U, 1U);
	set_port(&ebest.receiver, 0x01U, 2U);
	set_port(&erbest.sender, 0x20U, 1U);
	set_port(&erbest.receiver, 0x01U, 1U);
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_A_BETTER_TOPO,
			      ptp_bmca_compare(&ebest, &erbest));
	TEST_ASSERT_EQUAL_INT(PTP_REC_P2,
		ptp_bmca_state_decision(&d0, &erbest, &ebest, false, false));

	/* Two hops apart is outside the dead band: A_BETTER, so M3, not P2. */
	erbest.steps_removed = 2U;
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_A_BETTER, ptp_bmca_compare(&ebest, &erbest));
	TEST_ASSERT_EQUAL_INT(PTP_REC_M3,
		ptp_bmca_state_decision(&d0, &erbest, &ebest, false, false));
}

static void test_state_decision_degrade_a_never_slaves(void)
{
	ptp_dataset_t d0;
	ptp_dataset_t other;

	/*
	 * The consequence of §7.6.2.5 that picks between the two degradation
	 * ladders: at class 52 a strictly better peer still only gets us to P1,
	 * never S1, so the grandmaster-only deviation never even comes up.
	 */
	base_ds(&d0, 0x01U);
	d0.quality.clock_class = 52U;
	base_ds(&other, 0x02U);
	other.quality.clock_class = 6U;
	other.priority1 = 1U;

	TEST_ASSERT_EQUAL_INT(PTP_REC_P1,
		ptp_bmca_state_decision(&d0, &other, &other, true, false));

	/* At 187 the very same inputs reach S1. */
	d0.quality.clock_class = 187U;
	TEST_ASSERT_EQUAL_INT(PTP_REC_S1,
		ptp_bmca_state_decision(&d0, &other, &other, true, false));
}

static void test_state_decision_null_dataset_is_inert(void)
{
	ptp_dataset_t other;

	base_ds(&other, 0x02U);
	TEST_ASSERT_EQUAL_INT(PTP_REC_LISTENING,
		ptp_bmca_state_decision(NULL, &other, &other, true, false));
}

static void test_recommended_names(void)
{
	TEST_ASSERT_EQUAL_STRING("LISTENING", ptp_recommended_name(PTP_REC_LISTENING));
	TEST_ASSERT_EQUAL_STRING("M1", ptp_recommended_name(PTP_REC_M1));
	TEST_ASSERT_EQUAL_STRING("M2", ptp_recommended_name(PTP_REC_M2));
	TEST_ASSERT_EQUAL_STRING("M3", ptp_recommended_name(PTP_REC_M3));
	TEST_ASSERT_EQUAL_STRING("P1", ptp_recommended_name(PTP_REC_P1));
	TEST_ASSERT_EQUAL_STRING("P2", ptp_recommended_name(PTP_REC_P2));
	TEST_ASSERT_EQUAL_STRING("S1", ptp_recommended_name(PTP_REC_S1));
	TEST_ASSERT_EQUAL_STRING("?", ptp_recommended_name(PTP_REC_COUNT));
}

/* -------------------------------------------------- foreign-master table -- */

static const ptp_foreign_policy_t pol = {
	.receipt_timeout = 3U,
	.default_interval_ms = 2000U,
};

static void mk_announce(ptp_announce_t *a, uint8_t gm_last, uint8_t priority1)
{
	memset(a, 0, sizeof(*a));
	a->gm_priority1 = priority1;
	a->gm_quality.clock_class = 6U;
	a->gm_quality.clock_accuracy = 0x20U;
	a->gm_quality.offset_scaled_log_variance = 0x4E5DU;
	a->gm_priority2 = 128U;
	set_id(&a->gm_identity, gm_last);
	a->steps_removed = 1U;
	a->time_source = 0x20U;
}

static void test_foreign_qualification_needs_two_in_the_window(void)
{
	ptp_foreign_tbl_t t;
	ptp_port_id_t rx;
	ptp_port_id_t src;
	ptp_announce_t a;
	const ptp_foreign_t *f;

	ptp_foreign_init(&t);
	set_port(&rx, 0xEEU, 1U);
	set_port(&src, 0x10U, 1U);
	mk_announce(&a, 0x10U, 128U);

	/* First Announce: recorded but not yet a candidate. */
	f = ptp_foreign_update(&t, &pol, &src, &a, 0U, 1, 1000U);
	TEST_ASSERT_NOT_NULL(f);
	TEST_ASSERT_FALSE(f->qualified);
	TEST_ASSERT_EQUAL_UINT16(1U, f->count);
	TEST_ASSERT_NULL(ptp_foreign_best(&t, &rx, NULL));

	/* Second inside the window (4 x 2000 ms): qualified. */
	f = ptp_foreign_update(&t, &pol, &src, &a, 0U, 1, 3000U);
	TEST_ASSERT_TRUE(f->qualified);
	TEST_ASSERT_EQUAL_UINT16(PTP_FOREIGN_MASTER_THRESHOLD, f->count);
	TEST_ASSERT_EQUAL_PTR(f, ptp_foreign_best(&t, &rx, NULL));
	TEST_ASSERT_EQUAL_UINT32(1U, t.added);
}

static void test_foreign_window_slides_with_each_announce(void)
{
	ptp_foreign_tbl_t t;
	ptp_port_id_t src;
	ptp_announce_t a;
	const ptp_foreign_t *f;
	unsigned int k;

	/*
	 * Regression: the window is measured against the previous Announce, not
	 * against a fixed origin. A tumbling window would drop the count back to
	 * 1 every fourth interval — de-qualifying a master that is announcing
	 * perfectly on cadence, and taking it out of Erbest for an interval at a
	 * time. Announce on cadence for 40 intervals and the count must only ever
	 * rise.
	 */
	ptp_foreign_init(&t);
	set_port(&src, 0x10U, 1U);
	mk_announce(&a, 0x10U, 128U);

	for (k = 0U; k < 40U; k++) {
		f = ptp_foreign_update(&t, &pol, &src, &a, 0U, 1,
				       1000U + ((uint64_t)k * 2000U));
		TEST_ASSERT_NOT_NULL(f);
		TEST_ASSERT_EQUAL_UINT16((uint16_t)(k + 1U), f->count);
		TEST_ASSERT_EQUAL_INT(k >= (PTP_FOREIGN_MASTER_THRESHOLD - 1U),
				      f->qualified ? 1 : 0);
	}
	TEST_ASSERT_EQUAL_UINT32(1U, t.added);

	/* Exactly one window since the last Announce is still inside it. */
	f = ptp_foreign_update(&t, &pol, &src, &a, 0U, 1,
			       1000U + (39U * 2000U) + 8000U);
	TEST_ASSERT_EQUAL_UINT16(41U, f->count);
	TEST_ASSERT_TRUE(f->qualified);
}

static void test_foreign_gap_past_the_window_restarts_the_count(void)
{
	ptp_foreign_tbl_t t;
	ptp_port_id_t src;
	ptp_announce_t a;
	const ptp_foreign_t *f;

	ptp_foreign_init(&t);
	set_port(&src, 0x10U, 1U);
	mk_announce(&a, 0x10U, 128U);

	(void)ptp_foreign_update(&t, &pol, &src, &a, 0U, 1, 1000U);
	f = ptp_foreign_update(&t, &pol, &src, &a, 0U, 1, 3000U);
	TEST_ASSERT_TRUE(f->qualified);

	/* One millisecond past the 4 x 2000 ms window: the count starts over. */
	f = ptp_foreign_update(&t, &pol, &src, &a, 0U, 1, 3000U + 8001U);
	TEST_ASSERT_EQUAL_UINT16(1U, f->count);
	TEST_ASSERT_EQUAL_UINT32(1U, t.added); /* refreshed, not re-added */

	/*
	 * Qualification is not surrendered by the dip: only prune or clear takes
	 * it away. With any sane policy the receipt timeout (3 intervals) fires
	 * long before a 4-interval gap, so this record would already be gone —
	 * ptp_foreign_prune() is what actually retires a master.
	 */
	TEST_ASSERT_TRUE(f->qualified);
	/* The last Announce reset the clock, so the timeout runs from 11001 ms. */
	TEST_ASSERT_EQUAL_UINT32(0U, ptp_foreign_prune(&t, &pol, 11001U + 5999U));
	TEST_ASSERT_EQUAL_UINT32(1U, ptp_foreign_prune(&t, &pol, 11001U + 6000U));
	TEST_ASSERT_FALSE(t.rec[0].qualified);
	TEST_ASSERT_FALSE(t.rec[0].in_use);
}

static void test_foreign_uses_the_peers_own_interval(void)
{
	ptp_foreign_tbl_t t;
	ptp_port_id_t src;
	ptp_announce_t a;
	const ptp_foreign_t *f;

	ptp_foreign_init(&t);
	set_port(&src, 0x10U, 1U);
	mk_announce(&a, 0x10U, 128U);

	/* logAnnounceInterval 3 is 8 s, so the window is 32 s, not 8 s. */
	f = ptp_foreign_update(&t, &pol, &src, &a, 0U, 3, 0U);
	TEST_ASSERT_EQUAL_UINT16(1U, f->count);
	f = ptp_foreign_update(&t, &pol, &src, &a, 0U, 3, 30000U);
	TEST_ASSERT_EQUAL_UINT16(2U, f->count);

	/* It also governs the prune timeout: 3 x 8 s. */
	TEST_ASSERT_EQUAL_UINT32(0U, ptp_foreign_prune(&t, &pol, 30000U + 23999U));
	TEST_ASSERT_EQUAL_UINT32(1U, ptp_foreign_prune(&t, &pol, 30000U + 24000U));

	/* An unspecified (0x7F) interval falls back to the policy default. */
	ptp_foreign_init(&t);
	f = ptp_foreign_update(&t, &pol, &src, &a, 0U, PTP_LOG_INTERVAL_UNSPEC, 0U);
	TEST_ASSERT_NOT_NULL(f);
	TEST_ASSERT_EQUAL_UINT32(0U, ptp_foreign_prune(&t, &pol, 5999U));
	TEST_ASSERT_EQUAL_UINT32(1U, ptp_foreign_prune(&t, &pol, 6000U));
}

static void test_foreign_prune_expires_the_silent(void)
{
	ptp_foreign_tbl_t t;
	ptp_port_id_t rx;
	ptp_port_id_t src;
	ptp_announce_t a;

	ptp_foreign_init(&t);
	set_port(&rx, 0xEEU, 1U);
	set_port(&src, 0x10U, 1U);
	mk_announce(&a, 0x10U, 128U);

	(void)ptp_foreign_update(&t, &pol, &src, &a, 0U, 1, 0U);
	(void)ptp_foreign_update(&t, &pol, &src, &a, 0U, 1, 2000U);
	TEST_ASSERT_NOT_NULL(ptp_foreign_best(&t, &rx, NULL));

	/* 3 x 2000 ms of silence. */
	TEST_ASSERT_EQUAL_UINT32(0U, ptp_foreign_prune(&t, &pol, 2000U + 5999U));
	TEST_ASSERT_EQUAL_UINT32(1U, ptp_foreign_prune(&t, &pol, 2000U + 6000U));
	TEST_ASSERT_EQUAL_UINT32(1U, t.expired);
	TEST_ASSERT_NULL(ptp_foreign_best(&t, &rx, NULL));

	/* An empty table prunes nothing. */
	TEST_ASSERT_EQUAL_UINT32(0U, ptp_foreign_prune(&t, &pol, 1000000U));
}

static void test_foreign_eviction_is_least_recently_heard(void)
{
	ptp_foreign_tbl_t t;
	ptp_port_id_t rx;
	ptp_announce_t a;
	unsigned int i;

	ptp_foreign_init(&t);
	set_port(&rx, 0xEEU, 1U);

	/* Fill the table; record i was last heard at (i+1)*10 ms. */
	for (i = 0U; i < (unsigned int)PTP_MAX_FOREIGN_MASTERS; i++) {
		ptp_port_id_t src;

		set_port(&src, (uint8_t)(0x10U + i), 1U);
		mk_announce(&a, (uint8_t)(0x10U + i), 128U);
		TEST_ASSERT_NOT_NULL(
			ptp_foreign_update(&t, &pol, &src, &a, 0U, 1, (i + 1U) * 10U));
	}
	TEST_ASSERT_EQUAL_UINT32((uint32_t)PTP_MAX_FOREIGN_MASTERS, t.added);
	TEST_ASSERT_EQUAL_UINT32(0U, t.evicted);

	/* One more master evicts the oldest, which is the first one inserted. */
	{
		ptp_port_id_t src;
		ptp_port_id_t oldest;
		const ptp_foreign_t *f;

		set_port(&oldest, 0x10U, 1U);
		set_port(&src, 0x99U, 1U);
		mk_announce(&a, 0x99U, 128U);
		f = ptp_foreign_update(&t, &pol, &src, &a, 0U, 1, 1000U);
		TEST_ASSERT_NOT_NULL(f);
		TEST_ASSERT_EQUAL_UINT32(1U, t.evicted);
		TEST_ASSERT_EQUAL_UINT32((uint32_t)PTP_MAX_FOREIGN_MASTERS + 1U, t.added);

		/* The evicted record's slot now belongs to the new master. */
		TEST_ASSERT_EQUAL_INT(0, ptp_port_id_cmp(&f->source_port, &src));
		for (i = 0U; i < (unsigned int)PTP_MAX_FOREIGN_MASTERS; i++) {
			if (t.rec[i].in_use) {
				TEST_ASSERT_NOT_EQUAL_INT(0,
					ptp_port_id_cmp(&t.rec[i].source_port, &oldest));
			}
		}
	}
}

static void test_foreign_eviction_scans_the_whole_table(void)
{
	ptp_foreign_tbl_t t;
	ptp_announce_t a;
	ptp_port_id_t src;
	ptp_port_id_t oldest;
	unsigned int i;

	/*
	 * The companion test inserts in ascending age, so slot 0 is already the
	 * oldest and the scan's "found an older one" branch never runs. Insert
	 * in descending age instead: the victim is the last slot, which only the
	 * full scan can find.
	 */
	ptp_foreign_init(&t);
	for (i = 0U; i < (unsigned int)PTP_MAX_FOREIGN_MASTERS; i++) {
		set_port(&src, (uint8_t)(0x10U + i), 1U);
		mk_announce(&a, (uint8_t)(0x10U + i), 128U);
		TEST_ASSERT_NOT_NULL(ptp_foreign_update(&t, &pol, &src, &a, 0U, 1,
			(uint64_t)(PTP_MAX_FOREIGN_MASTERS - i) * 10U));
	}

	set_port(&oldest,
		 (uint8_t)(0x10U + (unsigned int)PTP_MAX_FOREIGN_MASTERS - 1U), 1U);
	set_port(&src, 0x99U, 1U);
	mk_announce(&a, 0x99U, 128U);
	TEST_ASSERT_NOT_NULL(ptp_foreign_update(&t, &pol, &src, &a, 0U, 1, 1000U));
	TEST_ASSERT_EQUAL_UINT32(1U, t.evicted);

	for (i = 0U; i < (unsigned int)PTP_MAX_FOREIGN_MASTERS; i++) {
		if (t.rec[i].in_use) {
			TEST_ASSERT_NOT_EQUAL_INT(0,
				ptp_port_id_cmp(&t.rec[i].source_port, &oldest));
		}
	}
}

static void test_foreign_best_picks_the_winner(void)
{
	ptp_foreign_tbl_t t;
	ptp_port_id_t rx;
	ptp_dataset_t ds;
	const ptp_foreign_t *best;
	unsigned int i;

	ptp_foreign_init(&t);
	set_port(&rx, 0xEEU, 1U);

	/* Three qualified masters; the middle one has the best priority1. */
	for (i = 0U; i < 3U; i++) {
		ptp_port_id_t src;
		ptp_announce_t a;
		static const uint8_t prio[3] = { 130U, 10U, 200U };

		set_port(&src, (uint8_t)(0x10U + i), 1U);
		mk_announce(&a, (uint8_t)(0x10U + i), prio[i]);
		(void)ptp_foreign_update(&t, &pol, &src, &a, 0U, 1, 0U);
		(void)ptp_foreign_update(&t, &pol, &src, &a, 0U, 1, 1000U);
	}

	best = ptp_foreign_best(&t, &rx, &ds);
	TEST_ASSERT_NOT_NULL(best);
	TEST_ASSERT_EQUAL_HEX8(10U, ds.priority1);
	TEST_ASSERT_EQUAL_HEX8(0x11U, best->source_port.clock_id.id[7]);

	/* The dataset carries our port as the receiver, per §9.3.4. */
	TEST_ASSERT_EQUAL_INT(0, ptp_port_id_cmp(&ds.receiver, &rx));
	TEST_ASSERT_EQUAL_INT(0, ptp_port_id_cmp(&ds.sender, &best->source_port));
	TEST_ASSERT_EQUAL_HEX16(1U, ds.steps_removed);
	TEST_ASSERT_EQUAL_HEX8(6U, ds.quality.clock_class);

	/* Unqualified records are invisible to the election. */
	{
		ptp_port_id_t src;
		ptp_announce_t a;

		set_port(&src, 0x50U, 1U);
		mk_announce(&a, 0x50U, 1U); /* would win if it counted */
		(void)ptp_foreign_update(&t, &pol, &src, &a, 0U, 1, 1000U);
		best = ptp_foreign_best(&t, &rx, &ds);
		TEST_ASSERT_EQUAL_HEX8(10U, ds.priority1);
	}
}

static void test_foreign_clear_keeps_the_counters(void)
{
	ptp_foreign_tbl_t t;
	ptp_port_id_t rx;
	ptp_port_id_t src;
	ptp_announce_t a;

	ptp_foreign_init(&t);
	set_port(&rx, 0xEEU, 1U);
	set_port(&src, 0x10U, 1U);
	mk_announce(&a, 0x10U, 128U);
	(void)ptp_foreign_update(&t, &pol, &src, &a, 0U, 1, 0U);
	(void)ptp_foreign_update(&t, &pol, &src, &a, 0U, 1, 1000U);
	TEST_ASSERT_NOT_NULL(ptp_foreign_best(&t, &rx, NULL));

	ptp_foreign_clear(&t);
	TEST_ASSERT_NULL(ptp_foreign_best(&t, &rx, NULL));
	TEST_ASSERT_EQUAL_UINT32(1U, t.added); /* lifetime counter is not rewound */

	/* init(), by contrast, zeroes everything. */
	ptp_foreign_init(&t);
	TEST_ASSERT_EQUAL_UINT32(0U, t.added);
}

static void test_foreign_null_arguments_are_inert(void)
{
	ptp_foreign_tbl_t t;
	ptp_port_id_t rx;
	ptp_port_id_t src;
	ptp_announce_t a;
	ptp_dataset_t ds;

	ptp_foreign_init(&t);
	set_port(&rx, 0xEEU, 1U);
	set_port(&src, 0x10U, 1U);
	mk_announce(&a, 0x10U, 128U);

	/* Core never traps; a NULL argument is a no-op with a NULL/0 answer. */
	ptp_foreign_init(NULL);
	ptp_foreign_clear(NULL);
	TEST_ASSERT_NULL(ptp_foreign_update(NULL, &pol, &src, &a, 0U, 1, 0U));
	TEST_ASSERT_NULL(ptp_foreign_update(&t, NULL, &src, &a, 0U, 1, 0U));
	TEST_ASSERT_NULL(ptp_foreign_update(&t, &pol, NULL, &a, 0U, 1, 0U));
	TEST_ASSERT_NULL(ptp_foreign_update(&t, &pol, &src, NULL, 0U, 1, 0U));
	TEST_ASSERT_EQUAL_UINT32(0U, ptp_foreign_prune(NULL, &pol, 0U));
	TEST_ASSERT_EQUAL_UINT32(0U, ptp_foreign_prune(&t, NULL, 0U));
	TEST_ASSERT_NULL(ptp_foreign_best(NULL, &rx, &ds));
	TEST_ASSERT_NULL(ptp_foreign_best(&t, NULL, &ds));

	memset(&ds, 0xA5, sizeof(ds));
	ptp_foreign_dataset(NULL, &rx, &ds);
	ptp_foreign_dataset(&t.rec[0], NULL, &ds);
	ptp_foreign_dataset(&t.rec[0], &rx, NULL);
	TEST_ASSERT_EQUAL_HEX8(0xA5U, ds.priority1); /* untouched */
}

static void test_foreign_count_saturates(void)
{
	ptp_foreign_tbl_t t;
	ptp_port_id_t src;
	ptp_announce_t a;
	const ptp_foreign_t *f = NULL;
	uint32_t i;

	ptp_foreign_init(&t);
	set_port(&src, 0x10U, 1U);
	mk_announce(&a, 0x10U, 128U);

	/*
	 * A pathological peer flooding Announces must not wrap the counter back
	 * through zero and un-qualify itself. Same millisecond throughout, so
	 * the window never lapses.
	 */
	for (i = 0U; i < 70000U; i++) {
		f = ptp_foreign_update(&t, &pol, &src, &a, 0U, 1, 5U);
	}
	TEST_ASSERT_NOT_NULL(f);
	TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, f->count);
	TEST_ASSERT_TRUE(f->qualified);
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_part1_each_rung_decides_in_order);
	RUN_TEST(test_part1_identity_tiebreak_is_total);
	RUN_TEST(test_part1_lower_rung_cannot_override_higher);
	RUN_TEST(test_compare_rejects_null);

	RUN_TEST(test_part2_steps_removed_dead_band);
	RUN_TEST(test_part2_receiver_above_sender_is_not_topology);
	RUN_TEST(test_part2_error_1_hearing_yourself);
	RUN_TEST(test_part2_equal_steps_orders_on_sender_then_receiver);
	RUN_TEST(test_part2_error_2_same_everything);

	RUN_TEST(test_state_decision_listening_holds_without_announce);
	RUN_TEST(test_state_decision_m1_and_p1);
	RUN_TEST(test_state_decision_m2_and_s1);
	RUN_TEST(test_state_decision_m3_and_p2);
	RUN_TEST(test_state_decision_degrade_a_never_slaves);
	RUN_TEST(test_state_decision_null_dataset_is_inert);
	RUN_TEST(test_recommended_names);

	RUN_TEST(test_foreign_qualification_needs_two_in_the_window);
	RUN_TEST(test_foreign_window_slides_with_each_announce);
	RUN_TEST(test_foreign_gap_past_the_window_restarts_the_count);
	RUN_TEST(test_foreign_uses_the_peers_own_interval);
	RUN_TEST(test_foreign_prune_expires_the_silent);
	RUN_TEST(test_foreign_eviction_is_least_recently_heard);
	RUN_TEST(test_foreign_eviction_scans_the_whole_table);
	RUN_TEST(test_foreign_best_picks_the_winner);
	RUN_TEST(test_foreign_clear_keeps_the_counters);
	RUN_TEST(test_foreign_null_arguments_are_inert);
	RUN_TEST(test_foreign_count_saturates);

	return UNITY_END();
}
