/*
 * STS1000 "Meridian" — remote syslog over TLS: the fail-closed rule, the RFC
 * 5425 framing and the reconnect cadence, executed.
 *
 * THE DEFECT THIS GUARDS WAS SHIPPED. `log.syslog.tls` is a reachable BOOL in
 * the config schema (0x0905), writable from the shell, REST and MCP. Setting
 * it produced one local LOG_WRN and then RFC 5424 datagrams in the clear, to
 * the collector, over UDP — for the lifetime of the deployment. An operator who
 * asked for encryption got none and believed otherwise, which is strictly worse
 * than getting nothing, because "nothing" is visible and "cleartext you think
 * is ciphertext" is not.
 *
 * The reason it survived is structural, not careless: sts_syslog.c is Zephyr
 * glue — k_thread, zsock, logring — that no host suite links, so the branch
 * that chose UDP over TLS was unreachable from any test in the tree. That is
 * the same shape as the fan, K1-relay and watchdog fail-safes, and it gets the
 * same treatment: the DECISIONS move into net/sts_syslog_tls_policy.h, which is
 * Zephyr-free, which the firmware links verbatim, and which this file runs.
 *
 * WHAT IS AND IS NOT COVERED. Everything below is a rule with a wrong answer
 * that no runtime signal would reveal:
 *
 *   - the transport decision, ENUMERATED over its whole 2^5 input space rather
 *     than spot-checked, because the property being asserted is a universal
 *     ("no input yields UDP when tls is set") and a universal cannot be
 *     established by examples;
 *   - the octet count, byte-exact, because a MSG-LEN off by one desynchronises
 *     an octet-counted stream permanently and silently at the sender;
 *   - the cursor-advance rule, which is the entire spool;
 *   - the backoff, because "reconnect without spinning" is a claim about a
 *     number that is never printed anywhere;
 *   - the liveness arithmetic, because its failure mode is a grandmaster clock
 *     rebooting for reasons no log survives.
 *
 * What is NOT covered here: that zsock_connect() is actually called with the
 * sec tag, that tls_credential_add() succeeded, or that the collector is real.
 * Those are integration facts and this file does not pretend to have them.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "unity.h"

#include "zephyr/net/sts_syslog_tls_policy.h"
#include "zephyr/net/sts_ldap_ca.h" /* STS_LDAP_CA_SEC_TAG, for the collision test */
#include "zephyr/net/sts_web.h"     /* STS_WEB_SEC_TAG, likewise */

/* ------------------------------------------------------------------------- */
/* the transport decision — fail closed                                      */
/* ------------------------------------------------------------------------- */

/* Bit positions used to enumerate sts_syslog_tx_in_t exhaustively. */
#define B_EN 0x01U
#define B_TLS 0x02U
#define B_HOST 0x04U
#define B_CA 0x08U
#define B_BUILT 0x10U

static sts_syslog_tx_in_t in_from_bits(unsigned int b)
{
	sts_syslog_tx_in_t in;

	in.enabled = (b & B_EN) != 0U;
	in.tls = (b & B_TLS) != 0U;
	in.host_set = (b & B_HOST) != 0U;
	in.ca_ready = (b & B_CA) != 0U;
	in.tls_built = (b & B_BUILT) != 0U;
	return in;
}

/*
 * THE headline guard. Over every one of the 32 reachable input combinations,
 * asking for TLS must never produce the cleartext transport. This is the
 * assertion that fails if anyone reinstates the fallback.
 */
static void test_requesting_tls_never_yields_the_cleartext_transport(void)
{
	unsigned int b;

	for (b = 0U; b < 32U; b++) {
		sts_syslog_tx_in_t in = in_from_bits(b);
		sts_syslog_tx_t t = sts_syslog_tx(&in);
		char msg[96];

		if (!in.tls) {
			continue;
		}
		(void)snprintf(msg, sizeof(msg),
			       "bits=0x%02X yielded UDP with tls set", b);
		TEST_ASSERT_TRUE_MESSAGE(t != STS_SYSLOG_TX_UDP, msg);
	}
}

/*
 * The complement, and just as load-bearing: asking for TLS with everything in
 * place must actually give TLS. A "fail closed" implementation that refuses
 * unconditionally would pass the test above and be useless.
 */
static void test_tls_with_an_anchor_and_a_host_is_granted(void)
{
	sts_syslog_tx_in_t in = { .enabled = true,
				  .tls = true,
				  .host_set = true,
				  .ca_ready = true,
				  .tls_built = true };

	TEST_ASSERT_EQUAL_INT((int)STS_SYSLOG_TX_TLS, (int)sts_syslog_tx(&in));
	TEST_ASSERT_NULL(sts_syslog_refusal(&in));
}

static void test_tls_without_an_anchor_is_refused_not_downgraded(void)
{
	sts_syslog_tx_in_t in = { .enabled = true,
				  .tls = true,
				  .host_set = true,
				  .ca_ready = false,
				  .tls_built = true };

	TEST_ASSERT_EQUAL_INT((int)STS_SYSLOG_TX_REFUSED,
			      (int)sts_syslog_tx(&in));
	TEST_ASSERT_NOT_NULL(sts_syslog_refusal(&in));
	/* The refusal must name the missing thing, not the symptom. */
	TEST_ASSERT_NOT_NULL(strstr(sts_syslog_refusal(&in), "trust anchor"));
}

static void test_tls_in_a_build_without_a_tls_stack_is_refused(void)
{
	sts_syslog_tx_in_t in = { .enabled = true,
				  .tls = true,
				  .host_set = true,
				  .ca_ready = true,
				  .tls_built = false };

	TEST_ASSERT_EQUAL_INT((int)STS_SYSLOG_TX_REFUSED,
			      (int)sts_syslog_tx(&in));
	TEST_ASSERT_NOT_NULL(strstr(sts_syslog_refusal(&in), "TLS socket"));
}

static void test_cleartext_is_only_ever_reached_by_asking_for_it(void)
{
	unsigned int b;

	for (b = 0U; b < 32U; b++) {
		sts_syslog_tx_in_t in = in_from_bits(b);

		if (sts_syslog_tx(&in) == STS_SYSLOG_TX_UDP) {
			TEST_ASSERT_TRUE(in.enabled);
			TEST_ASSERT_TRUE(in.host_set);
			TEST_ASSERT_FALSE(in.tls);
		}
	}
}

static void test_no_host_or_not_enabled_is_off_and_silent(void)
{
	unsigned int b;

	for (b = 0U; b < 32U; b++) {
		sts_syslog_tx_in_t in = in_from_bits(b);

		if (!in.enabled || !in.host_set) {
			TEST_ASSERT_EQUAL_INT((int)STS_SYSLOG_TX_OFF,
					      (int)sts_syslog_tx(&in));
			TEST_ASSERT_NULL(sts_syslog_refusal(&in));
			TEST_ASSERT_FALSE(sts_syslog_alarm(
				sts_syslog_tx(&in), false));
		}
	}
}

static void test_a_null_input_is_off_rather_than_a_transport(void)
{
	TEST_ASSERT_EQUAL_INT((int)STS_SYSLOG_TX_OFF, (int)sts_syslog_tx(NULL));
	TEST_ASSERT_NULL(sts_syslog_refusal(NULL));
}

/*
 * Zephyr's tls_credential_add() keys on the sec tag alone. Two subsystems that
 * chose the same tag do not collide loudly — the second add returns -EEXIST or
 * silently reuses the first's chain, and the box ends up trusting a log
 * collector's CA to sign the directory that names its administrators. The tags
 * are literals in three different headers, so nothing but this checks them.
 */
static void test_the_syslog_anchor_does_not_share_a_tag_with_another(void)
{
	TEST_ASSERT_NOT_EQUAL_INT((int)STS_LDAP_CA_SEC_TAG,
				  (int)STS_SYSLOG_CA_SEC_TAG);
	TEST_ASSERT_NOT_EQUAL_INT((int)STS_WEB_SEC_TAG,
				  (int)STS_SYSLOG_CA_SEC_TAG);
}

/* Likewise for the on-disk anchor: two subsystems writing one path would have
 * the later install silently retract the earlier one. */
static void test_the_syslog_anchor_has_its_own_file(void)
{
	TEST_ASSERT_NOT_EQUAL_INT(0, strcmp(STS_SYSLOG_CA_PATH,
					    STS_LDAP_CA_PATH));
}

/* ------------------------------------------------------------------------- */
/* the alarm                                                                 */
/* ------------------------------------------------------------------------- */

/*
 * The whole point of the rewrite is that the failure is not only a LOG_WRN.
 * A refusal must ring whether or not anything is "connected", because there is
 * nothing to connect.
 */
static void test_a_refusal_alarms_regardless_of_connection_state(void)
{
	TEST_ASSERT_TRUE(sts_syslog_alarm(STS_SYSLOG_TX_REFUSED, false));
	TEST_ASSERT_TRUE(sts_syslog_alarm(STS_SYSLOG_TX_REFUSED, true));
}

static void test_tls_alarms_only_while_the_session_is_down(void)
{
	TEST_ASSERT_TRUE(sts_syslog_alarm(STS_SYSLOG_TX_TLS, false));
	TEST_ASSERT_FALSE(sts_syslog_alarm(STS_SYSLOG_TX_TLS, true));
}

static void test_udp_and_off_never_alarm(void)
{
	TEST_ASSERT_FALSE(sts_syslog_alarm(STS_SYSLOG_TX_UDP, false));
	TEST_ASSERT_FALSE(sts_syslog_alarm(STS_SYSLOG_TX_UDP, true));
	TEST_ASSERT_FALSE(sts_syslog_alarm(STS_SYSLOG_TX_OFF, false));
	TEST_ASSERT_FALSE(sts_syslog_alarm(STS_SYSLOG_TX_OFF, true));
}

/* ------------------------------------------------------------------------- */
/* RFC 5425 octet-counted framing                                            */
/* ------------------------------------------------------------------------- */

/*
 * Byte-exact. RFC 5425 §4.3 is `MSG-LEN SP SYSLOG-MSG`, where MSG-LEN counts
 * the octets of SYSLOG-MSG and nothing else — not itself, not the space, not a
 * terminator.
 */
static void test_a_frame_is_the_count_a_space_and_the_message(void)
{
	static const char msg[] = "<134>1 2026-07-30T00:00:00Z m s - - - hi";
	uint8_t out[128];
	size_t n = 0U;

	TEST_ASSERT_EQUAL_INT(0, sts_syslog_frame((const uint8_t *)msg,
						  strlen(msg), out, sizeof(out),
						  &n));

	/* strlen(msg) is 40 -> "40 " + 40 octets = 43. */
	TEST_ASSERT_EQUAL_size_t(40U, strlen(msg));
	TEST_ASSERT_EQUAL_size_t(43U, n);
	TEST_ASSERT_EQUAL_UINT8('4', out[0]);
	TEST_ASSERT_EQUAL_UINT8('0', out[1]);
	TEST_ASSERT_EQUAL_UINT8(' ', out[2]);
	TEST_ASSERT_EQUAL_MEMORY(msg, &out[3], 40U);
}

/*
 * The off-by-one that desynchronises a stream forever. Asserted as an identity
 * over many lengths rather than one case, because the digit-count boundaries
 * (9->10, 99->100, 999->1000) are exactly where a hand-rolled decimal
 * conversion goes wrong.
 */
static void test_the_count_equals_the_message_length_at_every_boundary(void)
{
	static const size_t lens[] = { 1U,   2U,   9U,   10U,	11U,
				       99U,  100U, 101U, 999U,	1000U,
				       1001U, 2047U, 2048U };
	uint8_t msg[STS_SYSLOG_MSG_MAX];
	uint8_t out[STS_SYSLOG_MSG_MAX + STS_SYSLOG_FRAME_OVERHEAD];
	size_t i;

	memset(msg, 'A', sizeof(msg));

	for (i = 0U; i < (sizeof(lens) / sizeof(lens[0])); i++) {
		size_t len = lens[i];
		size_t n = 0U;
		size_t sp;
		unsigned long parsed;
		char digits[8];
		char why[96];

		TEST_ASSERT_EQUAL_INT(0, sts_syslog_frame(msg, len, out,
							  sizeof(out), &n));

		/* Locate the single SP, parse the digits back, and require
		 * that the round trip returns the exact octet count. */
		for (sp = 0U; sp < n; sp++) {
			if (out[sp] == (uint8_t)' ') {
				break;
			}
		}
		(void)snprintf(why, sizeof(why), "len=%zu had no SP", len);
		TEST_ASSERT_TRUE_MESSAGE(sp < n, why);
		TEST_ASSERT_TRUE(sp < sizeof(digits));

		memcpy(digits, out, sp);
		digits[sp] = '\0';
		parsed = strtoul(digits, NULL, 10);

		(void)snprintf(why, sizeof(why),
			       "len=%zu framed as MSG-LEN=%lu", len, parsed);
		TEST_ASSERT_EQUAL_UINT64_MESSAGE((uint64_t)len,
						 (uint64_t)parsed, why);

		/* And the frame is exactly digits + SP + message. */
		TEST_ASSERT_EQUAL_size_t(sp + 1U + len, n);
		TEST_ASSERT_EQUAL_MEMORY(msg, &out[sp + 1U], len);
	}
}

/* RFC 5425 §4.3.1: MSG-LEN "MUST NOT have leading zeros". */
static void test_the_count_carries_no_leading_zero(void)
{
	uint8_t msg[64];
	uint8_t out[128];
	size_t len;

	memset(msg, 'A', sizeof(msg));

	for (len = 1U; len <= sizeof(msg); len++) {
		size_t n = 0U;

		TEST_ASSERT_EQUAL_INT(0, sts_syslog_frame(msg, len, out,
							  sizeof(out), &n));
		TEST_ASSERT_TRUE(out[0] != (uint8_t)'0');
	}
}

/* Exactly one SP separates the count from the message; a second would be
 * counted as message content by the receiver and shift every field. */
static void test_a_frame_contains_exactly_one_separator_before_the_message(void)
{
	static const char msg[] = "abc";
	uint8_t out[32];
	size_t n = 0U;

	TEST_ASSERT_EQUAL_INT(0, sts_syslog_frame((const uint8_t *)msg, 3U, out,
						  sizeof(out), &n));
	TEST_ASSERT_EQUAL_size_t(5U, n);
	TEST_ASSERT_EQUAL_UINT8('3', out[0]);
	TEST_ASSERT_EQUAL_UINT8(' ', out[1]);
	TEST_ASSERT_EQUAL_UINT8('a', out[2]);
}

/* The frame is never NUL-terminated into the octet count: a stray NUL inside a
 * counted frame is message content and corrupts the record. */
static void test_the_frame_length_excludes_any_terminator(void)
{
	static const char msg[] = "xy";
	uint8_t out[16];
	size_t n = 0U;

	memset(out, 0xEE, sizeof(out));
	TEST_ASSERT_EQUAL_INT(0, sts_syslog_frame((const uint8_t *)msg, 2U, out,
						  sizeof(out), &n));
	TEST_ASSERT_EQUAL_size_t(4U, n);
	/* Nothing was written past the reported length. */
	TEST_ASSERT_EQUAL_UINT8(0xEE, out[4]);
}

static void test_a_frame_that_does_not_fit_is_refused_not_truncated(void)
{
	static const char msg[] = "hello";
	uint8_t out[8];
	size_t n = 12345U;

	/* Needs 1 + 1 + 5 = 7; 6 is one short. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC, sts_syslog_frame((const uint8_t *)msg,
							5U, out, 6U, &n));
	TEST_ASSERT_EQUAL_INT(0, sts_syslog_frame((const uint8_t *)msg, 5U, out,
						  7U, &n));
	TEST_ASSERT_EQUAL_size_t(7U, n);
}

static void test_an_empty_or_oversized_message_is_rejected(void)
{
	uint8_t msg[STS_SYSLOG_MSG_MAX + 1U];
	uint8_t out[STS_SYSLOG_MSG_MAX + 16U];
	size_t n = 0U;

	memset(msg, 'A', sizeof(msg));

	/* RFC 5424 has no empty message, and "0 " is a receiver parse error. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      sts_syslog_frame(msg, 0U, out, sizeof(out), &n));
	/* Beyond the length a receiver is required to accept. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      sts_syslog_frame(msg, STS_SYSLOG_MSG_MAX + 1U,
					       out, sizeof(out), &n));
	/* The boundary itself is accepted. */
	TEST_ASSERT_EQUAL_INT(0, sts_syslog_frame(msg, STS_SYSLOG_MSG_MAX, out,
						  sizeof(out), &n));
}

static void test_null_arguments_are_rejected(void)
{
	uint8_t out[16];
	size_t n = 0U;

	TEST_ASSERT_EQUAL_INT(-EINVAL,
			      sts_syslog_frame(NULL, 4U, out, sizeof(out), &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sts_syslog_frame((const uint8_t *)"ab",
							2U, NULL, sizeof(out),
							&n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sts_syslog_frame((const uint8_t *)"ab",
							2U, out, sizeof(out),
							NULL));
}

/*
 * The overhead constant must actually bound the overhead, or the transmit
 * buffer sized from it overflows on the longest message.
 */
static void test_the_declared_overhead_bounds_the_worst_case(void)
{
	uint8_t msg[STS_SYSLOG_MSG_MAX];
	uint8_t out[STS_SYSLOG_MSG_MAX + STS_SYSLOG_FRAME_OVERHEAD];
	size_t n = 0U;

	memset(msg, 'A', sizeof(msg));
	TEST_ASSERT_EQUAL_INT(0, sts_syslog_frame(msg, sizeof(msg), out,
						  sizeof(out), &n));
	TEST_ASSERT_TRUE(n <= sizeof(msg) + STS_SYSLOG_FRAME_OVERHEAD);
}

/* ------------------------------------------------------------------------- */
/* the cursor: this is the spool                                             */
/* ------------------------------------------------------------------------- */

static void test_a_failed_stream_write_does_not_advance_the_cursor(void)
{
	TEST_ASSERT_FALSE(sts_syslog_advance_ok(STS_SYSLOG_TX_TLS, false));
	TEST_ASSERT_TRUE(sts_syslog_advance_ok(STS_SYSLOG_TX_TLS, true));
}

static void test_a_failed_datagram_send_still_advances(void)
{
	/* sendto() reports departure, not arrival: holding the cursor against
	 * a host that is simply gone would wedge the reader forever. */
	TEST_ASSERT_TRUE(sts_syslog_advance_ok(STS_SYSLOG_TX_UDP, false));
	TEST_ASSERT_TRUE(sts_syslog_advance_ok(STS_SYSLOG_TX_UDP, true));
}

static void test_only_the_tls_transport_is_octet_counted(void)
{
	TEST_ASSERT_TRUE(sts_syslog_tx_is_stream(STS_SYSLOG_TX_TLS));
	TEST_ASSERT_FALSE(sts_syslog_tx_is_stream(STS_SYSLOG_TX_UDP));
	TEST_ASSERT_FALSE(sts_syslog_tx_is_stream(STS_SYSLOG_TX_OFF));
	TEST_ASSERT_FALSE(sts_syslog_tx_is_stream(STS_SYSLOG_TX_REFUSED));
}

/*
 * The spool, played out. A stream transport that drops after two of five
 * records must resume at record three, not at record six.
 */
static void test_a_dropped_session_resumes_at_the_unsent_record(void)
{
	uint32_t cursor = 100U;
	unsigned int i;
	const bool write_ok[5] = { true, true, false, false, false };

	for (i = 0U; i < 5U; i++) {
		if (sts_syslog_advance_ok(STS_SYSLOG_TX_TLS, write_ok[i])) {
			cursor++;
		} else {
			break;
		}
	}
	TEST_ASSERT_EQUAL_UINT32(102U, cursor);
}

/* ------------------------------------------------------------------------- */
/* the port                                                                  */
/* ------------------------------------------------------------------------- */

static void test_tls_at_the_default_port_is_moved_to_6514(void)
{
	TEST_ASSERT_EQUAL_UINT16(6514U, sts_syslog_port(true, 514U));
}

static void test_an_operator_chosen_port_is_honoured_verbatim(void)
{
	TEST_ASSERT_EQUAL_UINT16(1514U, sts_syslog_port(true, 1514U));
	TEST_ASSERT_EQUAL_UINT16(6514U, sts_syslog_port(true, 6514U));
	TEST_ASSERT_EQUAL_UINT16(10514U, sts_syslog_port(true, 10514U));
}

static void test_the_cleartext_transport_never_has_its_port_moved(void)
{
	TEST_ASSERT_EQUAL_UINT16(514U, sts_syslog_port(false, 514U));
	TEST_ASSERT_EQUAL_UINT16(6514U, sts_syslog_port(false, 6514U));
}

/* ------------------------------------------------------------------------- */
/* reconnect cadence                                                         */
/* ------------------------------------------------------------------------- */

/* "Without spinning" is a claim about a number nothing ever prints. */
static void test_the_first_retry_is_never_immediate(void)
{
	TEST_ASSERT_EQUAL_UINT32(STS_SYSLOG_BACKOFF_MIN_MS,
				 sts_syslog_backoff_ms(0U));
	TEST_ASSERT_TRUE(sts_syslog_backoff_ms(0U) > 0U);
}

static void test_the_backoff_doubles_then_saturates(void)
{
	TEST_ASSERT_EQUAL_UINT32(1000U, sts_syslog_backoff_ms(0U));
	TEST_ASSERT_EQUAL_UINT32(2000U, sts_syslog_backoff_ms(1U));
	TEST_ASSERT_EQUAL_UINT32(4000U, sts_syslog_backoff_ms(2U));
	TEST_ASSERT_EQUAL_UINT32(8000U, sts_syslog_backoff_ms(3U));
	TEST_ASSERT_EQUAL_UINT32(16000U, sts_syslog_backoff_ms(4U));
	TEST_ASSERT_EQUAL_UINT32(32000U, sts_syslog_backoff_ms(5U));
	TEST_ASSERT_EQUAL_UINT32(60000U, sts_syslog_backoff_ms(6U));
	TEST_ASSERT_EQUAL_UINT32(60000U, sts_syslog_backoff_ms(7U));
}

/* Monotone and bounded for every failure count, including absurd ones: the
 * counter is uint32 and a long outage must not wrap the delay back to zero. */
static void test_the_backoff_is_monotone_and_bounded_forever(void)
{
	uint32_t prev = 0U;
	uint32_t f;

	for (f = 0U; f < 200U; f++) {
		uint32_t ms = sts_syslog_backoff_ms(f);

		TEST_ASSERT_TRUE(ms >= prev);
		TEST_ASSERT_TRUE(ms >= STS_SYSLOG_BACKOFF_MIN_MS);
		TEST_ASSERT_TRUE(ms <= STS_SYSLOG_BACKOFF_MAX_MS);
		prev = ms;
	}
	TEST_ASSERT_EQUAL_UINT32(STS_SYSLOG_BACKOFF_MAX_MS,
				 sts_syslog_backoff_ms(0xFFFFFFFFU));
}

static void test_a_retry_is_due_only_after_the_delay_elapses(void)
{
	TEST_ASSERT_FALSE(sts_syslog_retry_due(1000U, 500U, 0U)); /* 500 < 1000 */
	TEST_ASSERT_TRUE(sts_syslog_retry_due(1500U, 500U, 0U));  /* == 1000 */
	TEST_ASSERT_TRUE(sts_syslog_retry_due(9999U, 500U, 0U));
	TEST_ASSERT_FALSE(sts_syslog_retry_due(2000U, 500U, 1U)); /* needs 2000 */
	TEST_ASSERT_TRUE(sts_syslog_retry_due(2500U, 500U, 1U));
}

/*
 * k_uptime_get_32() wraps every 49.7 days. A comparison written as
 * `since + delay <= now` reads as "never due" for the whole first minute after
 * a wrap, which is precisely when a long-lived box is least supervised.
 */
static void test_the_retry_clock_survives_the_32_bit_wrap(void)
{
	const uint32_t since = 0xFFFFFF00U; /* 256 ms before wrap */

	/* 0xFFFFFF00 -> 0x00000000 is 0x100 = 256 ms elapsed. */
	TEST_ASSERT_FALSE(sts_syslog_retry_due(0x00000000U, since, 0U));
	/* -> 0x2E7 is 999 ms: one short. */
	TEST_ASSERT_FALSE(sts_syslog_retry_due(0x000002E7U, since, 0U));
	/* -> 0x2E8 is exactly 1000 ms. */
	TEST_ASSERT_TRUE(sts_syslog_retry_due(0x000002E8U, since, 0U));
}

/* ------------------------------------------------------------------------- */
/* the liveness budget                                                       */
/* ------------------------------------------------------------------------- */

/*
 * Zephyr's ztls_connect_ctx() blocks for up to 2 x NET_SOCKETS_CONNECT_TIMEOUT
 * (TCP connect, then the handshake, each given the same Kconfig). At the
 * defaults that is 6000 ms against a 5000 ms liveness deadline — so a TLS
 * connect must NOT run inline on the liveness-participant drain thread, and
 * sts_syslog.c BUILD_ASSERTs the same relationship this test states.
 *
 * If this ever returns true at the shipped numbers, the connector thread could
 * be deleted. It does not.
 */
static void test_a_tls_connect_does_not_fit_inside_the_liveness_deadline(void)
{
	/* CONFIG_NET_SOCKETS_CONNECT_TIMEOUT=3000, SYSLOG_POLL_MS=500,
	 * CONFIG_STS1000_LIVENESS_DEADLINE_MS=5000. */
	TEST_ASSERT_EQUAL_UINT32(6000U, STS_SYSLOG_CONNECT_BUDGET(3000U));
	TEST_ASSERT_FALSE(
		sts_syslog_connect_fits_liveness(3000U, 500U, 5000U));
}

static void test_the_budget_check_admits_a_genuinely_short_connect(void)
{
	/* Guards against a check that is simply always false. */
	TEST_ASSERT_TRUE(sts_syslog_connect_fits_liveness(1000U, 500U, 5000U));
	TEST_ASSERT_FALSE(sts_syslog_connect_fits_liveness(2500U, 500U, 5000U));
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_requesting_tls_never_yields_the_cleartext_transport);
	RUN_TEST(test_tls_with_an_anchor_and_a_host_is_granted);
	RUN_TEST(test_tls_without_an_anchor_is_refused_not_downgraded);
	RUN_TEST(test_tls_in_a_build_without_a_tls_stack_is_refused);
	RUN_TEST(test_cleartext_is_only_ever_reached_by_asking_for_it);
	RUN_TEST(test_no_host_or_not_enabled_is_off_and_silent);
	RUN_TEST(test_a_null_input_is_off_rather_than_a_transport);
	RUN_TEST(test_the_syslog_anchor_does_not_share_a_tag_with_another);
	RUN_TEST(test_the_syslog_anchor_has_its_own_file);

	RUN_TEST(test_a_refusal_alarms_regardless_of_connection_state);
	RUN_TEST(test_tls_alarms_only_while_the_session_is_down);
	RUN_TEST(test_udp_and_off_never_alarm);

	RUN_TEST(test_a_frame_is_the_count_a_space_and_the_message);
	RUN_TEST(test_the_count_equals_the_message_length_at_every_boundary);
	RUN_TEST(test_the_count_carries_no_leading_zero);
	RUN_TEST(test_a_frame_contains_exactly_one_separator_before_the_message);
	RUN_TEST(test_the_frame_length_excludes_any_terminator);
	RUN_TEST(test_a_frame_that_does_not_fit_is_refused_not_truncated);
	RUN_TEST(test_an_empty_or_oversized_message_is_rejected);
	RUN_TEST(test_null_arguments_are_rejected);
	RUN_TEST(test_the_declared_overhead_bounds_the_worst_case);

	RUN_TEST(test_a_failed_stream_write_does_not_advance_the_cursor);
	RUN_TEST(test_a_failed_datagram_send_still_advances);
	RUN_TEST(test_only_the_tls_transport_is_octet_counted);
	RUN_TEST(test_a_dropped_session_resumes_at_the_unsent_record);

	RUN_TEST(test_tls_at_the_default_port_is_moved_to_6514);
	RUN_TEST(test_an_operator_chosen_port_is_honoured_verbatim);
	RUN_TEST(test_the_cleartext_transport_never_has_its_port_moved);

	RUN_TEST(test_the_first_retry_is_never_immediate);
	RUN_TEST(test_the_backoff_doubles_then_saturates);
	RUN_TEST(test_the_backoff_is_monotone_and_bounded_forever);
	RUN_TEST(test_a_retry_is_due_only_after_the_delay_elapses);
	RUN_TEST(test_the_retry_clock_survives_the_32_bit_wrap);

	RUN_TEST(test_a_tls_connect_does_not_fit_inside_the_liveness_deadline);
	RUN_TEST(test_the_budget_check_admits_a_genuinely_short_connect);

	return UNITY_END();
}
