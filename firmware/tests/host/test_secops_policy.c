/*
 * STS1000 "Meridian" — the AAA offload gate, tested away from Zephyr.
 *
 * sts_secops_policy.h decides three things that are invisible at runtime when
 * they are wrong, which is why they are in a Zephyr-free header at all:
 *
 *   1. HOW LONG THE NEXT LIVENESS-FED SLEEP MAY BE. Too long starves the
 *      watchdog and cold-cycles a running grandmaster; a loop that can return
 *      0 while time remains, or a positive value forever, never terminates —
 *      and a non-terminating loop that keeps feeding liveness is the worst
 *      failure available here, because it disables the mechanism designed to
 *      recover the board from exactly that.
 *
 *   2. WHO RELEASES THE GATE when a submitter's budget expires while the
 *      worker is still inside the lookup. Release it twice and two lookups run
 *      concurrently in sts_aaa.c's shared static g_tx/g_rx buffers; release it
 *      never and every management plane is denied for the rest of the uptime.
 *      Neither shows up as a failing request in the moment.
 *
 *   3. HOW MUCH WALL CLOCK AN UNAUTHENTICATED CALLER MAY BUY. The number is a
 *      judgement call, so the tests pin it to the *reasons* given for it in the
 *      header — clears the shipped per-backend defaults, falls short of one
 *      full chain, far short of the ceiling — rather than restating 10 000.
 *
 * The gate tests are not assertions about a model of the code: model_t below
 * holds only the primitives sts_secops.c owns (a token count, an answer
 * semaphore) and every decision it takes comes from the real policy functions.
 * The invariant under test is the one the C compiler cannot state — the gate
 * token is released exactly once per acquisition, over every interleaving of
 * "the worker finished" and "the submitter gave up".
 */

#include <stdbool.h>
#include <stdint.h>

#include "unity.h"

#include "zephyr/net/sts_secops_policy.h"

/* ===================================================================== */
/* 1. the bounded wait                                                   */
/* ===================================================================== */

static void test_a_live_budget_yields_a_whole_slice(void)
{
	TEST_ASSERT_EQUAL_INT32((int32_t)STS_AAA_FED_SLICE_MS,
				sts_aaa_fed_slice_ms(0, 10000));
}

static void test_the_last_slice_is_only_the_remainder(void)
{
	/* 100 ms left: sleeping a whole slice would overshoot the budget. */
	TEST_ASSERT_EQUAL_INT32(100, sts_aaa_fed_slice_ms(9900, 10000));
	TEST_ASSERT_EQUAL_INT32(1, sts_aaa_fed_slice_ms(9999, 10000));
	/* Exactly one slice left is still one whole slice, not a truncation. */
	TEST_ASSERT_EQUAL_INT32((int32_t)STS_AAA_FED_SLICE_MS,
				sts_aaa_fed_slice_ms(9750, 10000));
}

static void test_a_spent_budget_yields_zero(void)
{
	TEST_ASSERT_EQUAL_INT32(0, sts_aaa_fed_slice_ms(10000, 10000));
	TEST_ASSERT_EQUAL_INT32(0, sts_aaa_fed_slice_ms(10001, 10000));
	/* Long past it — the caller was descheduled for a minute. */
	TEST_ASSERT_EQUAL_INT32(0, sts_aaa_fed_slice_ms(70000, 10000));
}

/**
 * Never negative and never longer than a slice, swept across the whole of the
 * pre-auth budget. A negative would become a k_sem_take() with a negative
 * timeout; anything over a slice is watchdog starvation.
 */
static void test_no_slice_is_negative_or_overlong(void)
{
	int64_t now;

	for (now = -1000; now <= (int64_t)STS_AAA_FED_PREAUTH_MS + 1000;
	     now += 7) {
		int32_t s = sts_aaa_fed_slice_ms(now,
						 (int64_t)STS_AAA_FED_PREAUTH_MS);

		TEST_ASSERT_GREATER_OR_EQUAL_INT32(0, s);
		TEST_ASSERT_LESS_OR_EQUAL_INT32((int32_t)STS_AAA_FED_SLICE_MS, s);
	}
}

/**
 * Consume a budget the way the loops in sts_secops.c do — a clock that advances
 * by whatever the function asked for, exactly as k_sem_take(K_MSEC(slice)) does
 * on a timeout — and report how many feeds it took.
 *
 * The iteration cap is the point of the helper, not housekeeping: the failure
 * being tested for is a loop that never terminates, and a test that hangs
 * instead of failing is no test at all. `cap` is set from the expected count so
 * a non-terminating slice function fails here rather than in ctest's timeout.
 */
static unsigned int drive_budget(int64_t budget, unsigned int cap,
				 int64_t *end_now)
{
	int64_t now = 0;
	unsigned int feeds = 0U;

	for (;;) {
		int32_t s = sts_aaa_fed_slice_ms(now, budget);

		if (s <= 0) {
			break;
		}
		now += s;
		feeds++;
		TEST_ASSERT_LESS_OR_EQUAL_INT64_MESSAGE(
			budget, now, "a slice overshot the budget");
		TEST_ASSERT_LESS_OR_EQUAL_UINT32_MESSAGE(
			cap, feeds, "the fed wait did not terminate");
	}
	*end_now = now;
	return feeds;
}

/**
 * The property the fed waits actually depend on: consuming slices terminates,
 * lands exactly on the budget, and never overshoots it.
 */
static void test_consuming_slices_terminates_on_the_budget(void)
{
	const int64_t budget = (int64_t)STS_AAA_FED_PREAUTH_MS;
	int64_t now = -1;
	unsigned int feeds = drive_budget(budget, 40U, &now);

	TEST_ASSERT_EQUAL_INT64(budget, now);
	/* 10 000 / 250 — the number of liveness feeds a stalled pre-auth
	 * lookup costs, and the proof that the count is finite. */
	TEST_ASSERT_EQUAL_UINT32(40U, feeds);
}

static void test_the_ceiling_is_also_consumed_in_finite_slices(void)
{
	const int64_t budget = (int64_t)STS_AAA_FED_CEILING_MS;
	int64_t now = -1;
	unsigned int feeds = drive_budget(budget, 2400U, &now);

	TEST_ASSERT_EQUAL_INT64(budget, now);
	TEST_ASSERT_EQUAL_UINT32(2400U, feeds);
}

/**
 * k_uptime_get() is milliseconds since boot in a signed 64-bit value. A unit
 * that has been up for fifty days is at 4.32e9 ms, past the point where a
 * 32-bit intermediate would have wrapped — and this appliance is a
 * grandmaster clock whose whole purpose is not to be rebooted.
 */
static void test_a_long_uptime_does_not_break_the_arithmetic(void)
{
	const int64_t up = INT64_C(4320000000); /* 50 days */

	TEST_ASSERT_EQUAL_INT32((int32_t)STS_AAA_FED_SLICE_MS,
				sts_aaa_fed_slice_ms(up, up + 10000));
	TEST_ASSERT_EQUAL_INT32(100, sts_aaa_fed_slice_ms(up + 9900, up + 10000));
	TEST_ASSERT_EQUAL_INT32(0, sts_aaa_fed_slice_ms(up + 10000, up + 10000));
}

/* ===================================================================== */
/* 2. the budgets, pinned to their stated reasons                        */
/* ===================================================================== */

/**
 * Long enough to be worth the wrapper at all: a budget of one or two feed
 * slices would deny before the offload thread had a chance to answer.
 */
static void test_the_preauth_budget_outlasts_several_feed_slices(void)
{
	TEST_ASSERT_GREATER_THAN_UINT32(STS_AAA_FED_SLICE_MS * 4U,
					STS_AAA_FED_PREAUTH_MS);
}

/**
 * Long enough for the shipped defaults. The cfg schema's defaults give a single
 * backend 3 000 ms x 3 retries (RADIUS) or 5 000 ms (TACACS+/LDAP); a budget
 * under that would deny logins on a working-but-retrying server.
 */
static void test_the_preauth_budget_clears_the_shipped_backend_defaults(void)
{
	TEST_ASSERT_GREATER_OR_EQUAL_UINT32(9000U, STS_AAA_FED_PREAUTH_MS);
}

/**
 * ...and short enough that an anonymous caller cannot buy even one configured
 * chain, which is the whole point of separating it from the ceiling.
 */
static void test_an_unauthenticated_caller_cannot_buy_a_whole_chain(void)
{
	TEST_ASSERT_LESS_THAN_UINT32(STS_AAA_FED_CHAIN_WORST_MS,
				     STS_AAA_FED_PREAUTH_MS);
	TEST_ASSERT_LESS_THAN_UINT32(STS_AAA_FED_CEILING_MS,
				     STS_AAA_FED_PREAUTH_MS);
	/* An order of magnitude of separation, not a rounding difference. */
	TEST_ASSERT_LESS_OR_EQUAL_UINT32(STS_AAA_FED_CEILING_MS / 10U,
					 STS_AAA_FED_PREAUTH_MS);
}

/**
 * The ceiling exists to bound "the worker never came back", not to truncate a
 * chain the operator legitimately configured.
 */
static void test_the_ceiling_does_not_truncate_a_legal_chain(void)
{
	TEST_ASSERT_GREATER_OR_EQUAL_UINT32(STS_AAA_FED_CHAIN_WORST_MS,
					    STS_AAA_FED_CEILING_MS);
}

/* ===================================================================== */
/* 3. the gate hand-off                                                  */
/* ===================================================================== */

static void test_the_worker_publishes_to_a_waiting_submitter(void)
{
	sts_aaa_fed_finish_t f =
		sts_aaa_fed_finish((uint8_t)STS_AAA_SLOT_RUNNING);

	TEST_ASSERT_TRUE(f.publish);
	TEST_ASSERT_FALSE(f.release); /* the submitter is awake and will do it */
	TEST_ASSERT_FALSE(f.discard); /* somebody is coming for this answer */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_AAA_SLOT_IDLE, f.next);
}

static void test_the_worker_cleans_up_after_a_submitter_that_left(void)
{
	sts_aaa_fed_finish_t f =
		sts_aaa_fed_finish((uint8_t)STS_AAA_SLOT_ABANDONED);

	TEST_ASSERT_FALSE(f.publish); /* nobody is waiting on the answer sem */
	TEST_ASSERT_TRUE(f.release);  /* ...so the gate is the worker's to free */
	TEST_ASSERT_TRUE(f.discard);  /* ...and the credential must not linger */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_AAA_SLOT_IDLE, f.next);
}

/**
 * The unreachable row, and the reason it is defined rather than left to fall
 * through: a spurious release is the failure that admits two lookups into
 * sts_aaa.c's shared buffers, so the defensive answer discards the credential
 * and keeps its hands off the gate.
 */
static void test_an_impossible_finish_never_touches_the_gate(void)
{
	sts_aaa_fed_finish_t f = sts_aaa_fed_finish((uint8_t)STS_AAA_SLOT_IDLE);

	TEST_ASSERT_FALSE(f.publish);
	TEST_ASSERT_FALSE(f.release);
	TEST_ASSERT_TRUE(f.discard);

	f = sts_aaa_fed_finish(0xFFU);
	TEST_ASSERT_FALSE(f.release);
	TEST_ASSERT_TRUE(f.discard);
}

static void test_giving_up_hands_the_gate_to_the_worker(void)
{
	sts_aaa_fed_expiry_t e =
		sts_aaa_fed_expiry((uint8_t)STS_AAA_SLOT_RUNNING, false);

	TEST_ASSERT_FALSE(e.take);
	TEST_ASSERT_FALSE(e.release); /* the worker still owns the slot */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_AAA_SLOT_ABANDONED, e.next);
}

/**
 * The race the hand-off lock exists for: the worker published between the last
 * failed take and the expiry decision. The answer is valid and must be used —
 * discarding it would deny a login that actually succeeded.
 */
static void test_an_answer_that_lands_in_the_gap_is_still_taken(void)
{
	sts_aaa_fed_expiry_t e =
		sts_aaa_fed_expiry((uint8_t)STS_AAA_SLOT_IDLE, true);

	TEST_ASSERT_TRUE(e.take);
	TEST_ASSERT_TRUE(e.release);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_AAA_SLOT_IDLE, e.next);
}

static void test_giving_up_with_no_worker_frees_the_gate_here(void)
{
	sts_aaa_fed_expiry_t e =
		sts_aaa_fed_expiry((uint8_t)STS_AAA_SLOT_IDLE, false);

	TEST_ASSERT_FALSE(e.take);
	TEST_ASSERT_TRUE(e.release); /* nobody else can */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_AAA_SLOT_IDLE, e.next);
}

/**
 * Exactly one party releases, for every reachable (slot, answered) pair. This
 * is the invariant; the two functions exist so it can be stated.
 */
static void test_the_gate_is_released_by_exactly_one_party(void)
{
	static const uint8_t slots[] = {
		(uint8_t)STS_AAA_SLOT_IDLE,
		(uint8_t)STS_AAA_SLOT_RUNNING,
		(uint8_t)STS_AAA_SLOT_ABANDONED,
	};
	size_t i;

	for (i = 0U; i < sizeof(slots) / sizeof(slots[0]); i++) {
		/* The submitter gave up: it releases, or it leaves a slot the
		 * worker's own decision releases. Never both, never neither. */
		sts_aaa_fed_expiry_t e = sts_aaa_fed_expiry(slots[i], false);
		sts_aaa_fed_finish_t f = sts_aaa_fed_finish(e.next);

		TEST_ASSERT_TRUE_MESSAGE(e.release != f.release,
					 "the gate must be released by exactly "
					 "one of the submitter and the worker");
	}
}

/* --------------------------------------------------------------------- */
/* the interleavings, driven through the real decisions                  */
/* --------------------------------------------------------------------- */

/**
 * Only what sts_secops.c owns: the gate semaphore's count, the answer
 * semaphore's count, the slot byte, and whether a submitter believes it holds
 * the gate. Every transition below comes from the policy functions.
 */
typedef struct {
	uint8_t slot;
	int     token;   /**< k_sem g_gate count; must never leave {0, 1} */
	bool    answer;  /**< k_sem g_done count */
	bool    held;    /**< a submitter has the gate */
} model_t;

static void m_init(model_t *m)
{
	m->slot = (uint8_t)STS_AAA_SLOT_IDLE;
	m->token = 1;
	m->answer = false;
	m->held = false;
}

static void m_check(const model_t *m)
{
	TEST_ASSERT_GREATER_OR_EQUAL_INT(0, m->token);
	TEST_ASSERT_LESS_OR_EQUAL_INT(1, m->token);
	/* Mutual exclusion, stated: the token is either in the semaphore or in
	 * a submitter's hand, never in both and never duplicated. */
	TEST_ASSERT_FALSE(m->token == 1 && m->held);
}

/** gate_take_fed(), collapsed to its decision. */
static bool m_acquire(model_t *m)
{
	if (!sts_aaa_fed_wait_ok(m->slot)) {
		return false; /* refused without waiting */
	}
	if (m->token == 0) {
		return false; /* would have waited, then timed out */
	}
	m->token--;
	m->held = true;
	m_check(m);
	return true;
}

static void m_submit(model_t *m)
{
	TEST_ASSERT_TRUE(m->held);
	m->slot = (uint8_t)STS_AAA_SLOT_RUNNING;
}

/** The worker's tail. */
static void m_finish(model_t *m)
{
	sts_aaa_fed_finish_t f = sts_aaa_fed_finish(m->slot);

	m->slot = f.next;
	if (f.publish) {
		m->answer = true;
	}
	if (f.release) {
		m->token++;
	}
	m_check(m);
}

/** The submitter taking the answer inside its budget. */
static void m_reap(model_t *m)
{
	TEST_ASSERT_TRUE(m->answer);
	m->answer = false;
	m->held = false;
	m->token++;
	m_check(m);
}

/** The submitter's budget expiring. Returns whether it got an answer anyway. */
static bool m_expire(model_t *m)
{
	bool answered = m->answer;
	sts_aaa_fed_expiry_t e;

	m->answer = false;
	e = sts_aaa_fed_expiry(m->slot, answered);
	m->slot = e.next;
	m->held = false;
	if (e.release) {
		m->token++;
	}
	m_check(m);
	return e.take;
}

/** The ordinary path: submit, worker answers, submitter reaps. */
static void test_a_completed_lookup_returns_the_gate(void)
{
	model_t m;

	m_init(&m);
	TEST_ASSERT_TRUE(m_acquire(&m));
	m_submit(&m);
	m_finish(&m);
	m_reap(&m);

	TEST_ASSERT_EQUAL_INT(1, m.token);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_AAA_SLOT_IDLE, m.slot);
	/* ...and the next caller gets straight in. */
	TEST_ASSERT_TRUE(m_acquire(&m));
}

/**
 * The abandoned path. The gate must come back — late, but exactly once — and
 * a caller arriving in the meantime must be refused rather than left waiting
 * on a holder that has already overrun a budget.
 */
static void test_an_abandoned_lookup_returns_the_gate_late_and_once(void)
{
	model_t m;

	m_init(&m);
	TEST_ASSERT_TRUE(m_acquire(&m));
	m_submit(&m);

	TEST_ASSERT_FALSE(m_expire(&m)); /* no answer; gate left to the worker */
	TEST_ASSERT_EQUAL_INT(0, m.token);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_AAA_SLOT_ABANDONED, m.slot);

	/* This is what stops a sustained attack: refused immediately, so the
	 * second attempt does not hold api_lock for another whole budget. */
	TEST_ASSERT_FALSE(sts_aaa_fed_wait_ok(m.slot));
	TEST_ASSERT_FALSE(m_acquire(&m));

	m_finish(&m);
	TEST_ASSERT_EQUAL_INT(1, m.token);
	TEST_ASSERT_FALSE(m.answer); /* no stale answer for the next caller */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_AAA_SLOT_IDLE, m.slot);
	TEST_ASSERT_TRUE(m_acquire(&m));
}

/** The worker wins the race by a hair: the answer is used, once. */
static void test_a_photo_finish_uses_the_answer_and_returns_the_gate(void)
{
	model_t m;

	m_init(&m);
	TEST_ASSERT_TRUE(m_acquire(&m));
	m_submit(&m);
	m_finish(&m);              /* published... */
	TEST_ASSERT_TRUE(m_expire(&m)); /* ...just as the budget ran out */

	TEST_ASSERT_EQUAL_INT(1, m.token);
	TEST_ASSERT_FALSE(m.answer);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_AAA_SLOT_IDLE, m.slot);
}

/**
 * Repeated abandonment, which is the shape of an attack against a blackholed
 * backend: every round must leave the token count exactly where it started.
 * A leak denies the plane forever; a duplicate admits two lookups at once.
 */
static void test_repeated_abandonment_neither_leaks_nor_duplicates(void)
{
	model_t m;
	int i;

	m_init(&m);
	for (i = 0; i < 64; i++) {
		TEST_ASSERT_TRUE(m_acquire(&m));
		m_submit(&m);
		TEST_ASSERT_FALSE(m_expire(&m));
		TEST_ASSERT_FALSE(m_acquire(&m)); /* refused while abandoned */
		m_finish(&m);
		TEST_ASSERT_EQUAL_INT(1, m.token);
	}
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_AAA_SLOT_IDLE, m.slot);
}

static void test_waiting_is_allowed_only_while_somebody_is_inside_a_budget(void)
{
	TEST_ASSERT_TRUE(sts_aaa_fed_wait_ok((uint8_t)STS_AAA_SLOT_IDLE));
	TEST_ASSERT_TRUE(sts_aaa_fed_wait_ok((uint8_t)STS_AAA_SLOT_RUNNING));
	TEST_ASSERT_FALSE(sts_aaa_fed_wait_ok((uint8_t)STS_AAA_SLOT_ABANDONED));
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_a_live_budget_yields_a_whole_slice);
	RUN_TEST(test_the_last_slice_is_only_the_remainder);
	RUN_TEST(test_a_spent_budget_yields_zero);
	RUN_TEST(test_no_slice_is_negative_or_overlong);
	RUN_TEST(test_consuming_slices_terminates_on_the_budget);
	RUN_TEST(test_the_ceiling_is_also_consumed_in_finite_slices);
	RUN_TEST(test_a_long_uptime_does_not_break_the_arithmetic);

	RUN_TEST(test_the_preauth_budget_outlasts_several_feed_slices);
	RUN_TEST(test_the_preauth_budget_clears_the_shipped_backend_defaults);
	RUN_TEST(test_an_unauthenticated_caller_cannot_buy_a_whole_chain);
	RUN_TEST(test_the_ceiling_does_not_truncate_a_legal_chain);

	RUN_TEST(test_the_worker_publishes_to_a_waiting_submitter);
	RUN_TEST(test_the_worker_cleans_up_after_a_submitter_that_left);
	RUN_TEST(test_an_impossible_finish_never_touches_the_gate);
	RUN_TEST(test_giving_up_hands_the_gate_to_the_worker);
	RUN_TEST(test_an_answer_that_lands_in_the_gap_is_still_taken);
	RUN_TEST(test_giving_up_with_no_worker_frees_the_gate_here);
	RUN_TEST(test_the_gate_is_released_by_exactly_one_party);

	RUN_TEST(test_a_completed_lookup_returns_the_gate);
	RUN_TEST(test_an_abandoned_lookup_returns_the_gate_late_and_once);
	RUN_TEST(test_a_photo_finish_uses_the_answer_and_returns_the_gate);
	RUN_TEST(test_repeated_abandonment_neither_leaks_nor_duplicates);
	RUN_TEST(test_waiting_is_allowed_only_while_somebody_is_inside_a_budget);

	return UNITY_END();
}
