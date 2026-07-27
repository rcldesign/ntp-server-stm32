/*
 * STS1000 "Meridian" — the host->device passthrough routing decision.
 *
 * sts_mp_tunnel_policy.h exists because all three ways this can be wrong are
 * silent at runtime, and the file's own header says so. This suite is written
 * against those three, not against the switch statement:
 *
 *   1. A CHANNEL IS NEVER ROUTED TO THE WRONG PORT. Channel 0x07 is USART3 and
 *      the GNSS receiver; 0x08 is UART7 and the FE-5680A. A crossed pair puts
 *      rubidium commands into a timing receiver — which answers nothing, logs
 *      nothing, and leaves a technician debugging the wrong box. The sweep
 *      below asserts the mapping over every channel id in the octet, not just
 *      over the two the switch names.
 *
 *   2. A SHUT TUNNEL IS NEVER WRITTEN TO. The open flag is the record that
 *      firmware has been stood down on that UART (FMT §5.5). Writing anyway
 *      makes firmware and the maintenance host two writers on one port, which
 *      is the exact condition the tunnel object exists to prevent.
 *
 *   3. THE BURST IS BOUNDED. mp_tunnel.c calls this with the engine lock held
 *      and then blocks one character time per octet in a uart_poll_out() loop.
 *      An unbounded burst is therefore a host-controlled stall of the service
 *      tick, so the bound is checked at, one under and one over — and `max` is
 *      a parameter precisely so a test can pin the boundary rather than trust
 *      the constant.
 *
 * The refusal ORDER is part of the contract too, and is asserted directly: the
 * header argues each step of it, and a refusal that named the wrong obstacle
 * would send a well-behaved host looking for an override that does not exist
 * (or chunking a burst that was never going to be accepted).
 *
 * Nothing here is a round-trip against mp_tunnel.c: the decision is a pure
 * function of (channel, open flags, length, bound), which is why it lives in a
 * header of its own.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "mp/mp.h" /* MP_TUNNEL_TX_MAX — the bound the glue passes in */
#include "zephyr/console/sts_mp_tunnel_policy.h"

/* The two channels this device can write to, and the ports they name. */
#define CH_GNSS ((uint8_t)MP_CH_GNSS_PASS) /* 0x07, USART3 */
#define CH_RB   ((uint8_t)MP_CH_RB_PASS)   /* 0x08, UART7  */

#define MAXB ((size_t)MP_TUNNEL_TX_MAX)

/** Shorthand: both tunnels as named, default bound. */
static sts_mp_tx_act_t decide(uint8_t ch, bool gnss, bool rb, size_t len)
{
	return sts_mp_tunnel_tx_decide(ch, gnss, rb, len, MAXB);
}

/* ===================================================================== */
/* 1. the channel decides the port, and nothing else does                */
/* ===================================================================== */

/**
 * The pairing, stated once and directly. Everything else in this file is a
 * qualification of it.
 */
static void test_each_passthrough_names_its_own_port(void)
{
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_GNSS,
			      decide(CH_GNSS, true, true, 1U));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_RB, decide(CH_RB, true, true, 1U));

	/* And the pairing does not depend on the *other* tunnel's state, which
	 * is the shape a crossed lookup would take. */
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_GNSS,
			      decide(CH_GNSS, true, false, 1U));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_RB, decide(CH_RB, false, true, 1U));
}

/**
 * Every other channel id in the octet is refused as a channel — including the
 * ones a host might plausibly try.
 *
 * 0x02/0x03 are the NMEA and UBX tees: read-only mirrors of what the receiver
 * is already saying, so a host writing to them has misunderstood the protocol
 * and "closed" would be the wrong answer. 0x06 is the SMP tunnel, whose
 * manifest object has no apply binding at all, so nothing can open it and
 * routing bytes for it would be inventing a path.
 */
static void test_every_other_channel_is_refused_as_a_channel(void)
{
	unsigned int ch;

	for (ch = 0U; ch <= 0xFFU; ch++) {
		char msg[64];
		sts_mp_tx_act_t a;

		if ((ch == CH_GNSS) || (ch == CH_RB)) {
			continue;
		}
		(void)snprintf(msg, sizeof(msg),
			       "channel 0x%02X is not writable", ch);

		/* Both tunnels open and a legal burst: nothing about the
		 * *request* is wrong, so only the channel can refuse it. */
		a = decide((uint8_t)ch, true, true, 1U);
		TEST_ASSERT_EQUAL_INT_MESSAGE(STS_MP_TX_REFUSE_CHANNEL, a, msg);

		/* And the answer does not soften for an empty frame. */
		a = decide((uint8_t)ch, true, true, 0U);
		TEST_ASSERT_EQUAL_INT_MESSAGE(STS_MP_TX_REFUSE_CHANNEL, a, msg);
	}

	/* The three worth naming, so a regression reports which one. */
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_CHANNEL,
			      decide((uint8_t)MP_CH_NMEA, true, true, 4U));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_CHANNEL,
			      decide((uint8_t)MP_CH_UBX, true, true, 4U));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_CHANNEL,
			      decide((uint8_t)MP_CH_SMP, true, true, 4U));
	/* And the control/telemetry planes are not passthroughs either. */
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_CHANNEL,
			      decide((uint8_t)MP_CH_CONTROL, true, true, 4U));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_CHANNEL,
			      decide((uint8_t)MP_CH_MIRROR, true, true, 4U));
}

/* ===================================================================== */
/* 2. a shut tunnel is never written to                                  */
/* ===================================================================== */

static void test_a_closed_tunnel_refuses(void)
{
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_CLOSED,
			      decide(CH_GNSS, false, false, 1U));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_CLOSED,
			      decide(CH_RB, false, false, 1U));
}

/**
 * An unarmed channel is refused even while its neighbour is armed.
 *
 * This is the concrete crossed-lookup failure: one open tunnel authorising
 * writes to the other port. It reads as a working passthrough right up to the
 * point where the bytes land in the wrong IC.
 */
static void test_the_other_tunnel_being_open_authorises_nothing(void)
{
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_CLOSED,
			      decide(CH_RB, true, false, 8U));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_CLOSED,
			      decide(CH_GNSS, false, true, 8U));
}

/* ===================================================================== */
/* 3. the burst bound                                                    */
/* ===================================================================== */

/**
 * At the bound, one under, one over.
 *
 * `max` is inclusive: a burst of exactly MP_TUNNEL_TX_MAX is the largest the
 * glue will hand to uart_poll_out(), and it is the value the budget in
 * mp_tunnel.c's header is computed from. An off-by-one here is either a
 * needless refusal or 33 ms of extra lock time.
 */
static void test_the_burst_bound_is_inclusive(void)
{
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_GNSS,
			      decide(CH_GNSS, true, true, MAXB - 1U));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_GNSS,
			      decide(CH_GNSS, true, true, MAXB));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_OVERSIZE,
			      decide(CH_GNSS, true, true, MAXB + 1U));

	TEST_ASSERT_EQUAL_INT(STS_MP_TX_RB, decide(CH_RB, true, true, MAXB - 1U));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_RB, decide(CH_RB, true, true, MAXB));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_OVERSIZE,
			      decide(CH_RB, true, true, MAXB + 1U));
}

/** The bound is a parameter, so pin it away from the shipped constant too. */
static void test_the_bound_is_the_one_passed_in(void)
{
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_GNSS,
			      sts_mp_tunnel_tx_decide(CH_GNSS, true, true, 1U,
						      1U));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_OVERSIZE,
			      sts_mp_tunnel_tx_decide(CH_GNSS, true, true, 2U,
						      1U));

	/*
	 * A zero bound writes nothing at all. That is the fail-closed reading
	 * of "the platform has told me it can absorb no octets", and it matters
	 * because the alternative reading — 0 means unbounded — is the one that
	 * turns a mis-plumbed constant into a host-controlled stall.
	 */
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_OVERSIZE,
			      sts_mp_tunnel_tx_decide(CH_GNSS, true, true, 1U,
						      0U));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_OVERSIZE,
			      sts_mp_tunnel_tx_decide(CH_RB, true, true, 1U, 0U));
}

/* ===================================================================== */
/* 4. zero length                                                        */
/* ===================================================================== */

/**
 * An empty frame on an open tunnel is a keepalive, not an error.
 *
 * And it is answered *before* the size check, so it survives a zero bound: a
 * host holding a tunnel open with empty frames must not start seeing
 * -EMSGSIZE because the platform's write budget went to zero.
 */
static void test_a_zero_length_burst_on_an_open_tunnel_is_a_no_op(void)
{
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_NOTHING,
			      decide(CH_GNSS, true, true, 0U));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_NOTHING, decide(CH_RB, true, true, 0U));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_NOTHING,
			      sts_mp_tunnel_tx_decide(CH_GNSS, true, true, 0U,
						      0U));
}

/** But an empty frame does not open a shut tunnel, or a channel that is not one. */
static void test_a_zero_length_burst_is_still_refused_when_it_should_be(void)
{
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_CLOSED,
			      decide(CH_GNSS, false, true, 0U));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_CLOSED,
			      decide(CH_RB, true, false, 0U));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_CHANNEL,
			      decide((uint8_t)MP_CH_SMP, true, true, 0U));
}

/* ===================================================================== */
/* 5. the refusal names the real obstacle                                */
/* ===================================================================== */

/**
 * The documented precedence: channel, then tunnel, then emptiness, then size.
 *
 * Each pair below is a request that breaks two rules at once; the verdict must
 * be the outer one. A refusal that named the inner obstacle would tell the host
 * to fix something that was never going to help.
 */
static void test_the_refusal_order_is_outermost_first(void)
{
	/* Not a channel AND over the bound -> channel. */
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_CHANNEL,
			      decide((uint8_t)MP_CH_NMEA, true, true,
				     MAXB + 1U));
	/* Not a channel AND its notional tunnel shut -> channel. */
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_CHANNEL,
			      decide((uint8_t)MP_CH_SMP, false, false, 1U));
	/* Shut AND over the bound -> closed, because chunking will not help. */
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_CLOSED,
			      decide(CH_GNSS, false, false, MAXB + 1U));
	TEST_ASSERT_EQUAL_INT(STS_MP_TX_REFUSE_CLOSED,
			      decide(CH_RB, false, false, MAXB + 1U));
}

/* ===================================================================== */
/* 6. the whole input space                                              */
/* ===================================================================== */

/**
 * A verdict of "write it" appears only for the exact conjunction that permits
 * one, over every channel id, both tunnel states and a spread of lengths and
 * bounds that straddles each boundary.
 *
 * The point is not the arithmetic, which the cases above already pin. It is
 * that no *other* combination can reach a port: this is the assertion that a
 * new branch in the switch cannot quietly widen.
 */
static void test_no_input_outside_the_permitted_conjunction_reaches_a_port(void)
{
	static const size_t lens[] = { 0U, 1U, 2U, 127U, 128U, 129U, 4096U };
	static const size_t maxes[] = { 0U, 1U, 127U, 128U, 129U, MAXB };
	unsigned int ch;
	size_t li;
	size_t mi;

	for (ch = 0U; ch <= 0xFFU; ch++) {
		unsigned int g;

		for (g = 0U; g < 4U; g++) {
			bool gnss = (g & 1U) != 0U;
			bool rb = (g & 2U) != 0U;

			for (li = 0U; li < (sizeof(lens) / sizeof(lens[0]));
			     li++) {
				for (mi = 0U;
				     mi < (sizeof(maxes) / sizeof(maxes[0]));
				     mi++) {
					size_t len = lens[li];
					size_t max = maxes[mi];
					bool open = (ch == CH_GNSS) ? gnss
						  : ((ch == CH_RB) ? rb
								   : false);
					bool writable = (ch == CH_GNSS) ||
							(ch == CH_RB);
					bool permitted = writable && open &&
							 (len != 0U) &&
							 (max != 0U) &&
							 (len <= max);
					sts_mp_tx_act_t a =
						sts_mp_tunnel_tx_decide(
							(uint8_t)ch, gnss, rb,
							len, max);
					bool wrote = (a == STS_MP_TX_GNSS) ||
						     (a == STS_MP_TX_RB);
					char msg[128];

					(void)snprintf(
						msg, sizeof(msg),
						"ch 0x%02X gnss=%d rb=%d len=%zu "
						"max=%zu -> %s",
						ch, (int)gnss, (int)rb, len,
						max, sts_mp_tx_act_name(a));

					TEST_ASSERT_EQUAL_INT_MESSAGE(
						(int)permitted, (int)wrote, msg);
					if (wrote) {
						/* …and to the right port. */
						TEST_ASSERT_EQUAL_INT_MESSAGE(
							(ch == CH_GNSS)
								? STS_MP_TX_GNSS
								: STS_MP_TX_RB,
							a, msg);
					}
					/* Always one of the six verdicts. */
					TEST_ASSERT_TRUE_MESSAGE(
						(a >= STS_MP_TX_GNSS) &&
							(a <= STS_MP_TX_REFUSE_OVERSIZE),
						msg);
				}
			}
		}
	}
}

/* ===================================================================== */
/* 7. the verdict names                                                  */
/* ===================================================================== */

/**
 * Every verdict has its own non-empty name.
 *
 * mp_tunnel.c logs the refusal and the test failures above print it, so two
 * verdicts sharing a name would make the one log line an operator gets
 * ambiguous between "you cannot write here" and "you have not opened it".
 */
static void test_every_verdict_has_a_distinct_name(void)
{
	static const sts_mp_tx_act_t all[] = {
		STS_MP_TX_GNSS,	          STS_MP_TX_RB,
		STS_MP_TX_NOTHING,        STS_MP_TX_REFUSE_CHANNEL,
		STS_MP_TX_REFUSE_CLOSED,  STS_MP_TX_REFUSE_OVERSIZE,
	};
	size_t i;
	size_t j;

	for (i = 0U; i < (sizeof(all) / sizeof(all[0])); i++) {
		const char *ni = sts_mp_tx_act_name(all[i]);

		TEST_ASSERT_NOT_NULL(ni);
		TEST_ASSERT_TRUE(strlen(ni) > 0U);
		TEST_ASSERT_NOT_EQUAL_INT(0, strcmp(ni, "?"));

		for (j = i + 1U; j < (sizeof(all) / sizeof(all[0])); j++) {
			TEST_ASSERT_NOT_EQUAL_INT(
				0, strcmp(ni, sts_mp_tx_act_name(all[j])));
		}
	}

	/* An out-of-range verdict is legible rather than a NULL deref. */
	TEST_ASSERT_EQUAL_STRING("?", sts_mp_tx_act_name(
					      (sts_mp_tx_act_t)99));
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_each_passthrough_names_its_own_port);
	RUN_TEST(test_every_other_channel_is_refused_as_a_channel);

	RUN_TEST(test_a_closed_tunnel_refuses);
	RUN_TEST(test_the_other_tunnel_being_open_authorises_nothing);

	RUN_TEST(test_the_burst_bound_is_inclusive);
	RUN_TEST(test_the_bound_is_the_one_passed_in);

	RUN_TEST(test_a_zero_length_burst_on_an_open_tunnel_is_a_no_op);
	RUN_TEST(test_a_zero_length_burst_is_still_refused_when_it_should_be);

	RUN_TEST(test_the_refusal_order_is_outermost_first);
	RUN_TEST(test_no_input_outside_the_permitted_conjunction_reaches_a_port);
	RUN_TEST(test_every_verdict_has_a_distinct_name);

	return UNITY_END();
}
