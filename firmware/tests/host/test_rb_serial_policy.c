/*
 * STS1000 "Meridian" — the K1 relay's deferred fail-safe restore, executed.
 *
 * The defect this guards was real and silent. rb_serial_set_mode() refuses to
 * throw the K1 RS-232/CMOS relay while a raw tunnel holds UART7 — correctly; a
 * mechanical relay must not move under a live passthrough — and the maintenance
 * override engine reverts its leases in slot order, counting and then forgetting
 * a release that fails. Lease K1 to CMOS, open a tunnel, pull the cable: the
 * dead-man's first release arrives as set_mode(RS232), is refused, and the next
 * release closes the tunnel. Nothing owns the relay afterwards and nothing ever
 * moves it, so the board runs on with UART7 in a commissioning position no lease
 * holds and FE-5680A telemetry silently dead until somebody reboots it.
 *
 * The fix defers that one refusal. Its correctness is entirely in the asymmetry:
 * a refused move TO the fail-safe position is owed and paid at close, a refused
 * move AWAY from it is simply refused, and a close that was owed nothing must
 * leave the relay exactly where a standing lease put it.
 *
 * Until now that rule was pinned only by a source scan — one that read the text
 * of rb_serial.c without ever running it. This file runs it. The rule lives in
 * platform/sts_rb_serial_policy.h, which the firmware links and which needs no
 * devicetree, no UART ISR and no k_msleep(), so the code under test here is the
 * same code the board executes.
 *
 * WHAT THE RIG BELOW IS AND IS NOT. It is a state container: where the pin
 * stands, whether a tunnel is open, how many times the relay was driven. It
 * contains no rule of its own — every branch it takes is a switch on a verdict
 * the policy returned. That distinction is the whole point: a fixture that
 * decided anything for itself would be asserting against its own second copy of
 * the logic, which is how a guard goes green for the wrong reason.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "zephyr/platform/sts_rb_serial_policy.h"

/* ------------------------------------------------------------------- rig -- */

/**
 * Everything rb_serial.c owns that this suite cannot link: the relay pin, the
 * tunnel's presence, and a count of how often the pin was actually driven.
 *
 * `pin_writes` is the assertion that matters most often. "The relay did not
 * move" is not the same claim as "the relay ended up where it started", and the
 * bug this file guards against is exactly a relay that moves when nothing asked
 * it to.
 */
typedef struct {
	sts_rb_relay_t relay;
	bool tunnel_open;
	bool pin_active;	/**< last level driven onto RB_RS232_CMOS_SW */
	unsigned int pin_writes; /**< how many times it was driven at all */
	bool pin_fails;		/**< make the next GPIO write fail */
} rig_t;

/** Which of the two steps in rb_serial_tunnel_close() runs first. */
typedef enum {
	/** As built: retract the tunnel, THEN pay the deferred restore. */
	CLOSE_RETRACT_FIRST = 0,
	/** The inverted order the file header warns about. Never shipped; it
	 *  exists so this suite can show what it would cost. */
	CLOSE_DISCHARGE_FIRST,
} close_order_t;

/**
 * rb_serial_init(): reset the model, drive the pin to what it says, drop any
 * tunnel.
 *
 * It deliberately does NOT wipe the struct first. `rb` is a file-scope static,
 * so a re-init resets exactly the fields it names and nothing else — and a
 * fixture that cleared the latch on its own would mask a reset that had stopped
 * clearing it, which is precisely the class of green-for-the-wrong-reason this
 * suite exists to avoid. Use rig_new() for a rig that has never run.
 */
static void rig_init(rig_t *r)
{
	sts_rb_relay_reset(&r->relay);
	r->pin_active = sts_rb_relay_pin_active(r->relay.pos);
	r->tunnel_open = false;
}

/** A rig before anything has happened to it: zeroed counters, then init. */
static void rig_new(rig_t *r)
{
	memset(r, 0, sizeof(*r));
	rig_init(r);
}

/** rb_serial_set_mode(), with the GPIO write and the 20 ms settle stubbed. */
static int rig_set_mode(rig_t *r, uint8_t pos)
{
	switch (sts_rb_move_decide(pos, r->tunnel_open)) {
	case STS_RB_MOVE_INVALID:
		return -EINVAL;
	case STS_RB_MOVE_OWE:
		r->relay.restore_rs232_on_close = true;
		return -EBUSY;
	case STS_RB_MOVE_REFUSE:
		return -EBUSY;
	case STS_RB_MOVE_APPLY:
	default:
		break;
	}

	if (r->pin_fails) {
		/* rb.relay.pos is deliberately NOT advanced on a failed write. */
		return -EIO;
	}
	r->pin_active = sts_rb_relay_pin_active(pos);
	r->pin_writes++;
	r->relay.pos = pos;
	return 0;
}

/** rb_serial_tunnel_close(), with the step order left to the caller. */
static int rig_tunnel_close(rig_t *r, close_order_t order)
{
	sts_rb_close_act_t act;
	int rc = 0;

	act = sts_rb_close_decide(r->tunnel_open,
				  r->relay.restore_rs232_on_close);
	if (act == STS_RB_CLOSE_NOTHING) {
		return 0;
	}

	if ((order == CLOSE_DISCHARGE_FIRST) && (act == STS_RB_CLOSE_RESTORE)) {
		r->relay.restore_rs232_on_close = false;
		rc = rig_set_mode(r, (uint8_t)STS_RB_POS_RS232);
	}

	r->tunnel_open = false; /* the retraction */

	if ((order == CLOSE_RETRACT_FIRST) && (act == STS_RB_CLOSE_RESTORE)) {
		r->relay.restore_rs232_on_close = false;
		rc = rig_set_mode(r, (uint8_t)STS_RB_POS_RS232);
	}
	return rc;
}

/* --------------------------------------------------------- the positions -- */

/*
 * The reset state is both halves of "fail-safe", and the numbering that ties
 * this header to platform.h's rb_serial_mode_t (rb_serial.c carries the
 * BUILD_ASSERTs; a silent renumber here would invert the safe direction on a
 * board nobody is watching).
 */
static void test_the_reset_state_is_the_fail_safe_position(void)
{
	sts_rb_relay_t st;

	TEST_ASSERT_EQUAL_INT(0, (int)STS_RB_POS_RS232);
	TEST_ASSERT_EQUAL_INT(1, (int)STS_RB_POS_CMOS);
	TEST_ASSERT_EQUAL_INT(2, (int)STS_RB_POS__COUNT);

	/* A dirty prior state, written through the type rather than memset to a
	 * trap pattern: reading a _Bool that holds neither 0 nor 1 is undefined,
	 * and an assertion resting on undefined behaviour is one the compiler is
	 * free to satisfy without the code being right. */
	st.pos = (uint8_t)STS_RB_POS_CMOS;
	st.restore_rs232_on_close = true;

	sts_rb_relay_reset(&st);

	TEST_ASSERT_EQUAL_UINT8_MESSAGE(
		(uint8_t)STS_RB_POS_RS232, st.pos,
		"reset no longer lands on the level-shifter path, so a reboot "
		"leaves K1 in a commissioning position");
	TEST_ASSERT_FALSE_MESSAGE(
		st.restore_rs232_on_close,
		"reset leaves an obligation armed, inherited from a session "
		"that no longer exists");

	/* The direction of the pin and the direction of the reset must agree:
	 * GPIO_OUTPUT_INACTIVE at init is only the right drive because the
	 * fail-safe position is the inactive one. */
	TEST_ASSERT_FALSE_MESSAGE(
		sts_rb_relay_pin_active(st.pos),
		"the fail-safe position now wants the pin HIGH, which inverts "
		"both the reset drive and every move");
	TEST_ASSERT_TRUE(sts_rb_relay_pin_active((uint8_t)STS_RB_POS_CMOS));
}

/* -------------------------------------------------- the deferred restore -- */

/*
 * The defect, end to end. Commission K1 to CMOS, tunnel, then have the dead-man
 * ask for the fail-safe position while the tunnel still holds the port.
 */
static void test_a_refused_rs232_move_is_owed_and_paid_at_close(void)
{
	rig_t r;

	rig_new(&r);
	TEST_ASSERT_EQUAL_INT(0, rig_set_mode(&r, (uint8_t)STS_RB_POS_CMOS));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_RB_POS_CMOS, r.relay.pos);

	r.tunnel_open = true;

	TEST_ASSERT_EQUAL_INT_MESSAGE(
		-EBUSY, rig_set_mode(&r, (uint8_t)STS_RB_POS_RS232),
		"K1 can be thrown mid-passthrough again");
	TEST_ASSERT_EQUAL_UINT8_MESSAGE(
		(uint8_t)STS_RB_POS_CMOS, r.relay.pos,
		"the refusal moved the relay anyway");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, r.pin_writes,
		"the refused move still drove the pin");
	TEST_ASSERT_TRUE_MESSAGE(
		r.relay.restore_rs232_on_close,
		"a release refused by the tunnel is dropped on the floor "
		"again: K1 stays in the commissioning position with no lease "
		"left to move it");

	TEST_ASSERT_EQUAL_INT(0, rig_tunnel_close(&r, CLOSE_RETRACT_FIRST));

	TEST_ASSERT_EQUAL_UINT8_MESSAGE(
		(uint8_t)STS_RB_POS_RS232, r.relay.pos,
		"the close did not pay the deferred restore, so UART7 is "
		"stranded in the commissioning position until a reboot");
	TEST_ASSERT_FALSE_MESSAGE(
		sts_rb_relay_pin_active(r.relay.pos),
		"the restore left the pin on the CMOS level");
	TEST_ASSERT_FALSE_MESSAGE(
		r.relay.restore_rs232_on_close,
		"the latch survived the close it was paid by, so it will fire "
		"again in an unrelated session");
	TEST_ASSERT_EQUAL_UINT(2U, r.pin_writes);
}

/*
 * The other direction, and the reason it is not symmetric. Leaving the
 * fail-safe position is a deliberate commissioning act; a relay that threw
 * itself into the commissioning position minutes after the command asking for
 * it was rejected would be worse than the bug this replaces.
 */
static void test_a_refused_cmos_move_is_not_owed(void)
{
	rig_t r;

	rig_new(&r);
	TEST_ASSERT_EQUAL_INT(0, rig_set_mode(&r, (uint8_t)STS_RB_POS_CMOS));

	r.tunnel_open = true;

	/* The technician re-issues the commissioning move under the tunnel. */
	TEST_ASSERT_EQUAL_INT(-EBUSY,
			      rig_set_mode(&r, (uint8_t)STS_RB_POS_CMOS));
	TEST_ASSERT_FALSE_MESSAGE(
		r.relay.restore_rs232_on_close,
		"a refused CMOS move armed the deferred restore, so the close "
		"will throw K1 out of the position the technician asked for");

	TEST_ASSERT_EQUAL_INT(0, rig_tunnel_close(&r, CLOSE_RETRACT_FIRST));

	TEST_ASSERT_EQUAL_UINT8_MESSAGE(
		(uint8_t)STS_RB_POS_CMOS, r.relay.pos,
		"the close undid the technician's commissioning decision");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, r.pin_writes,
		"the close drove the relay although nothing was owed");
}

/*
 * A K1 lease and a tunnel lease are independent and simultaneous by design: set
 * CMOS *because* the variant needs CMOS, then tunnel to talk to it. Closing the
 * tunnel must not touch the relay, which is why the restore is conditional on
 * the latch rather than on the close.
 */
static void test_a_close_that_was_owed_nothing_moves_nothing(void)
{
	rig_t r;

	TEST_ASSERT_EQUAL_INT_MESSAGE(
		(int)STS_RB_CLOSE_PLAIN, (int)sts_rb_close_decide(true, false),
		"a close with nothing owed no longer just gives the port back");

	rig_new(&r);
	TEST_ASSERT_EQUAL_INT(0, rig_set_mode(&r, (uint8_t)STS_RB_POS_CMOS));

	r.tunnel_open = true;
	TEST_ASSERT_EQUAL_INT(0, rig_tunnel_close(&r, CLOSE_RETRACT_FIRST));

	TEST_ASSERT_EQUAL_UINT8_MESSAGE(
		(uint8_t)STS_RB_POS_CMOS, r.relay.pos,
		"the close yanked the relay out from under a standing CMOS "
		"lease, leaving the lease table claiming a position the pin no "
		"longer holds");
	TEST_ASSERT_EQUAL_UINT(1U, r.pin_writes);
	TEST_ASSERT_FALSE(r.tunnel_open);
}

/* Closing a port nobody holds is a no-op, latch or no latch — and the latched
 * variant is unreachable by construction, since only an open tunnel refuses. */
static void test_a_close_with_no_tunnel_open_does_nothing(void)
{
	rig_t r;

	TEST_ASSERT_EQUAL_INT((int)STS_RB_CLOSE_NOTHING,
			      (int)sts_rb_close_decide(false, false));
	TEST_ASSERT_EQUAL_INT_MESSAGE(
		(int)STS_RB_CLOSE_NOTHING, (int)sts_rb_close_decide(false, true),
		"an idempotent no-op call now throws the relay");

	rig_new(&r);
	TEST_ASSERT_EQUAL_INT(0, rig_set_mode(&r, (uint8_t)STS_RB_POS_CMOS));
	r.relay.restore_rs232_on_close = true; /* the unreachable pair */

	TEST_ASSERT_EQUAL_INT(0, rig_tunnel_close(&r, CLOSE_RETRACT_FIRST));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_RB_POS_CMOS, r.relay.pos);
	TEST_ASSERT_EQUAL_UINT(1U, r.pin_writes);
}

/*
 * An obligation must not outlive the session that incurred it. The hazard is
 * concrete: re-init, then a NEW commissioning decision, then an unrelated
 * tunnel — a surviving latch would throw K1 out of the position the new session
 * just asked for.
 */
static void test_the_deferred_restore_does_not_survive_a_reinit(void)
{
	rig_t r;

	rig_new(&r);
	TEST_ASSERT_EQUAL_INT(0, rig_set_mode(&r, (uint8_t)STS_RB_POS_CMOS));
	r.tunnel_open = true;
	TEST_ASSERT_EQUAL_INT(-EBUSY,
			      rig_set_mode(&r, (uint8_t)STS_RB_POS_RS232));
	TEST_ASSERT_TRUE(r.relay.restore_rs232_on_close);

	/* Whatever happened before the reset is not this session's problem. Note
	 * rig_init(), not rig_new(): rb_serial_init() runs against the state the
	 * previous session left behind, which is the whole question here. */
	rig_init(&r);
	TEST_ASSERT_FALSE_MESSAGE(
		r.relay.restore_rs232_on_close,
		"the deferred restore survived re-initialisation");

	/* The consequence, made visible. */
	TEST_ASSERT_EQUAL_INT(0, rig_set_mode(&r, (uint8_t)STS_RB_POS_CMOS));
	r.tunnel_open = true;
	TEST_ASSERT_EQUAL_INT(0, rig_tunnel_close(&r, CLOSE_RETRACT_FIRST));
	TEST_ASSERT_EQUAL_UINT8_MESSAGE(
		(uint8_t)STS_RB_POS_CMOS, r.relay.pos,
		"an obligation from a previous session fired into this one and "
		"undid its commissioning move");
}

/* -------------------------------------------------------- the ordering -- */

/*
 * Why rb_serial_tunnel_close() retracts before it discharges. The deferred move
 * is an ordinary set_mode(), so issued while the tunnel is still open it meets
 * the very refusal that armed the latch: it refuses itself, re-arms, and the
 * relay never moves — which is the original bug, reintroduced by a step order.
 *
 * This asserts the contract the policy encodes, not the order of two statements
 * in a C file: both orders are executed here and their outcomes compared.
 */
static void test_discharging_before_the_retraction_refuses_itself(void)
{
	rig_t built;
	rig_t inverted;

	/* The pair of verdicts the ordering rests on. */
	TEST_ASSERT_EQUAL_INT_MESSAGE(
		(int)STS_RB_MOVE_OWE,
		(int)sts_rb_move_decide((uint8_t)STS_RB_POS_RS232, true),
		"the restore no longer refuses under a live tunnel, so the "
		"step order stopped mattering — and so did the interlock");
	TEST_ASSERT_EQUAL_INT(
		(int)STS_RB_MOVE_APPLY,
		(int)sts_rb_move_decide((uint8_t)STS_RB_POS_RS232, false));

	rig_new(&built);
	TEST_ASSERT_EQUAL_INT(0,
			      rig_set_mode(&built, (uint8_t)STS_RB_POS_CMOS));
	built.tunnel_open = true;
	TEST_ASSERT_EQUAL_INT(
		-EBUSY, rig_set_mode(&built, (uint8_t)STS_RB_POS_RS232));

	inverted = built; /* identical state, opposite step order */

	TEST_ASSERT_EQUAL_INT(0, rig_tunnel_close(&built, CLOSE_RETRACT_FIRST));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_RB_POS_RS232, built.relay.pos);
	TEST_ASSERT_FALSE(built.relay.restore_rs232_on_close);

	TEST_ASSERT_EQUAL_INT_MESSAGE(
		-EBUSY, rig_tunnel_close(&inverted, CLOSE_DISCHARGE_FIRST),
		"a discharge issued before the retraction was accepted, so the "
		"refusal that protects a live passthrough is gone");
	TEST_ASSERT_EQUAL_UINT8_MESSAGE(
		(uint8_t)STS_RB_POS_CMOS, inverted.relay.pos,
		"the inverted order moved the relay, which is the thing the "
		"tunnel refusal exists to prevent");
	TEST_ASSERT_TRUE_MESSAGE(
		inverted.relay.restore_rs232_on_close,
		"the inverted order neither moved the relay nor re-armed the "
		"latch, so the obligation vanished silently");
}

/* ------------------------------------------------------------- refusals -- */

/*
 * Validity is decided before the tunnel is consulted, so a caller bug cannot
 * arm the deferral and move the relay some minutes later.
 */
static void test_an_out_of_range_position_arms_nothing(void)
{
	rig_t r;

	rig_new(&r);
	r.tunnel_open = true;

	TEST_ASSERT_EQUAL_INT_MESSAGE(
		(int)STS_RB_MOVE_INVALID,
		(int)sts_rb_move_decide((uint8_t)STS_RB_POS__COUNT, true),
		"an out-of-range position is now judged by the tunnel instead "
		"of by being out of range");
	TEST_ASSERT_EQUAL_INT((int)STS_RB_MOVE_INVALID,
			      (int)sts_rb_move_decide(255U, false));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      rig_set_mode(&r, (uint8_t)STS_RB_POS__COUNT));
	TEST_ASSERT_EQUAL_INT(-EINVAL, rig_set_mode(&r, 255U));
	TEST_ASSERT_FALSE_MESSAGE(
		r.relay.restore_rs232_on_close,
		"a bad mode armed the deferred restore");

	TEST_ASSERT_EQUAL_INT(0, rig_tunnel_close(&r, CLOSE_RETRACT_FIRST));
	TEST_ASSERT_EQUAL_UINT(0U, r.pin_writes);
}

/*
 * The whole move surface, written out by hand rather than derived from the
 * policy — six cells, and every one of them a different consequence. Anything
 * that queried sts_rb_move_decide() to work out what to expect here would agree
 * with itself no matter what the policy said.
 *
 * The second half is the same table's effect on the pin: only APPLY ever drives
 * the relay, whatever the direction, the tunnel, or the latch.
 */
static void test_the_move_decision_table(void)
{
	static const struct {
		uint8_t pos;
		bool tunnel_open;
		sts_rb_move_act_t want;
	} cases[] = {
		{ (uint8_t)STS_RB_POS_RS232, false, STS_RB_MOVE_APPLY },
		{ (uint8_t)STS_RB_POS_CMOS, false, STS_RB_MOVE_APPLY },
		{ (uint8_t)STS_RB_POS__COUNT, false, STS_RB_MOVE_INVALID },
		{ (uint8_t)STS_RB_POS_RS232, true, STS_RB_MOVE_OWE },
		{ (uint8_t)STS_RB_POS_CMOS, true, STS_RB_MOVE_REFUSE },
		{ (uint8_t)STS_RB_POS__COUNT, true, STS_RB_MOVE_INVALID },
	};
	size_t i;

	for (i = 0U; i < (sizeof(cases) / sizeof(cases[0])); i++) {
		rig_t r;
		char msg[96];

		(void)snprintf(msg, sizeof(msg),
			       "pos=%u tunnel=%d: wrong verdict",
			       (unsigned int)cases[i].pos,
			       (int)cases[i].tunnel_open);
		TEST_ASSERT_EQUAL_INT_MESSAGE(
			(int)cases[i].want,
			(int)sts_rb_move_decide(cases[i].pos,
						cases[i].tunnel_open),
			msg);

		rig_new(&r);
		r.tunnel_open = cases[i].tunnel_open;
		(void)rig_set_mode(&r, cases[i].pos);

		if (cases[i].want == STS_RB_MOVE_APPLY) {
			TEST_ASSERT_EQUAL_UINT(1U, r.pin_writes);
		} else {
			TEST_ASSERT_EQUAL_UINT_MESSAGE(
				0U, r.pin_writes,
				"a refused move drove the relay pin");
		}
	}
}

/* The close surface, likewise by hand. The (no tunnel, owed) cell is
 * unreachable — only an open tunnel refuses, and only a refusal arms — and it
 * is pinned anyway, because "unreachable" is a claim about today's callers. */
static void test_the_close_decision_table(void)
{
	TEST_ASSERT_EQUAL_INT((int)STS_RB_CLOSE_NOTHING,
			      (int)sts_rb_close_decide(false, false));
	TEST_ASSERT_EQUAL_INT((int)STS_RB_CLOSE_NOTHING,
			      (int)sts_rb_close_decide(false, true));
	TEST_ASSERT_EQUAL_INT((int)STS_RB_CLOSE_PLAIN,
			      (int)sts_rb_close_decide(true, false));
	TEST_ASSERT_EQUAL_INT((int)STS_RB_CLOSE_RESTORE,
			      (int)sts_rb_close_decide(true, true));
}

/* A GPIO error must leave the model behind the pin, not ahead of it: the next
 * attempt has to retry the write, and a model that had already advanced would
 * make a short-circuit refuse the retry that is the fix. */
static void test_a_failed_write_does_not_advance_the_model(void)
{
	rig_t r;

	rig_new(&r);
	r.pin_fails = true;
	TEST_ASSERT_EQUAL_INT(-EIO, rig_set_mode(&r, (uint8_t)STS_RB_POS_CMOS));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_RB_POS_RS232, r.relay.pos);

	r.pin_fails = false;
	TEST_ASSERT_EQUAL_INT_MESSAGE(
		(int)STS_RB_MOVE_APPLY,
		(int)sts_rb_move_decide((uint8_t)STS_RB_POS_CMOS, false),
		"the retry after a failed write is refused, so a relay that "
		"could not be driven once can never be driven again");
	TEST_ASSERT_EQUAL_INT(0, rig_set_mode(&r, (uint8_t)STS_RB_POS_CMOS));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)STS_RB_POS_CMOS, r.relay.pos);
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_the_reset_state_is_the_fail_safe_position);

	RUN_TEST(test_a_refused_rs232_move_is_owed_and_paid_at_close);
	RUN_TEST(test_a_refused_cmos_move_is_not_owed);
	RUN_TEST(test_a_close_that_was_owed_nothing_moves_nothing);
	RUN_TEST(test_a_close_with_no_tunnel_open_does_nothing);
	RUN_TEST(test_the_deferred_restore_does_not_survive_a_reinit);

	RUN_TEST(test_discharging_before_the_retraction_refuses_itself);

	RUN_TEST(test_an_out_of_range_position_arms_nothing);
	RUN_TEST(test_the_move_decision_table);
	RUN_TEST(test_the_close_decision_table);
	RUN_TEST(test_a_failed_write_does_not_advance_the_model);

	return UNITY_END();
}
