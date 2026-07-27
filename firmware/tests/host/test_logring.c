/*
 * STS1000 "Meridian" — core/logring unit tests.
 *
 * The interesting properties are the ones an operator relies on when the box
 * misbehaves, so they are tested directly rather than inferred:
 *
 *  - a full ring never refuses a record, and the loss it causes is counted;
 *  - a reader that falls behind is told how many records it missed, and its
 *    cursor lands on the oldest surviving record rather than silently skipping;
 *  - filtering advances the cursor, so a reader that filters out everything
 *    converges instead of rescanning the ring forever.
 *
 * The RFC 5424 expectations are literal strings checked against §6 of the RFC
 * (and its §6.5 examples for the shape), with the epoch → civil-date
 * conversion cross-checked against POSIX date(1) — not against this module's
 * own arithmetic.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "logring/logring.h"

#define RING_CAP 8U

static logr_rec_t g_slots[RING_CAP];
static logr_t g_ring;

void setUp(void)
{
	TEST_ASSERT_EQUAL_INT(0, logr_init(&g_ring, g_slots, RING_CAP));
}

/* Convenience: append "m<n>" at INFO/SYS with a synthetic timestamp. */
static void put_n(unsigned int n)
{
	unsigned int i;

	for (i = 0U; i < n; i++) {
		char msg[16];

		(void)snprintf(msg, sizeof(msg), "m%u", i);
		TEST_ASSERT_EQUAL_INT(0,
			logr_put(&g_ring, (uint8_t)LOGR_INFO,
				 (uint8_t)LOGR_SUB_SYS, 1000U + i, msg,
				 strlen(msg)));
	}
}

/* ------------------------------------------------------------------------- */
/* Lifecycle and arguments                                                   */
/* ------------------------------------------------------------------------- */

static void test_init_rejects_bad_arguments(void)
{
	logr_t l;

	TEST_ASSERT_EQUAL_INT(-EINVAL, logr_init(NULL, g_slots, RING_CAP));
	TEST_ASSERT_EQUAL_INT(-EINVAL, logr_init(&l, NULL, RING_CAP));
	TEST_ASSERT_EQUAL_INT(-EINVAL, logr_init(&l, g_slots, 0U));

	/* Accessors on a NULL ring answer instead of trapping the runner. */
	TEST_ASSERT_EQUAL_UINT32(0U, logr_head(NULL));
	TEST_ASSERT_EQUAL_UINT32(0U, logr_oldest(NULL));
	TEST_ASSERT_EQUAL_UINT16(0U, logr_count(NULL));
	TEST_ASSERT_EQUAL_UINT32(0U, logr_dropped(NULL));
	TEST_ASSERT_EQUAL_UINT32(0U, logr_filtered(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, logr_level_get(NULL, 0U));
	logr_reset(NULL);
	logr_filter_all(NULL);
}

static void test_put_rejects_bad_arguments(void)
{
	logr_t empty;

	memset(&empty, 0, sizeof(empty));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_put(NULL, LOGR_INFO, LOGR_SUB_SYS, 0U, "x", 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_put(&empty, LOGR_INFO, LOGR_SUB_SYS, 0U, "x", 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_put(&g_ring, 8U, LOGR_SUB_SYS, 0U, "x", 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_put(&g_ring, LOGR_INFO, LOGR_SUB_COUNT, 0U, "x", 1U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_put(&g_ring, LOGR_INFO, LOGR_SUB_SYS, 0U, NULL, 1U));

	/* A NULL message of length 0 is a legal empty record. */
	TEST_ASSERT_EQUAL_INT(0,
		logr_put(&g_ring, LOGR_INFO, LOGR_SUB_SYS, 0U, NULL, 0U));
	TEST_ASSERT_EQUAL_INT(0, logr_puts(&g_ring, LOGR_INFO, LOGR_SUB_SYS,
					   0U, NULL));
	TEST_ASSERT_EQUAL_UINT16(2U, logr_count(&g_ring));
}

/* ------------------------------------------------------------------------- */
/* Wrap and drop accounting                                                  */
/* ------------------------------------------------------------------------- */

static void test_wrap_counts_every_evicted_record(void)
{
	put_n(RING_CAP);
	TEST_ASSERT_EQUAL_UINT16(RING_CAP, logr_count(&g_ring));
	TEST_ASSERT_EQUAL_UINT32(0U, logr_dropped(&g_ring));
	TEST_ASSERT_EQUAL_UINT32(0U, logr_oldest(&g_ring));
	TEST_ASSERT_EQUAL_UINT32(RING_CAP, logr_head(&g_ring));

	put_n(5U);
	TEST_ASSERT_EQUAL_UINT16(RING_CAP, logr_count(&g_ring));
	TEST_ASSERT_EQUAL_UINT32(5U, logr_dropped(&g_ring));
	TEST_ASSERT_EQUAL_UINT32(5U, logr_oldest(&g_ring));
	TEST_ASSERT_EQUAL_UINT32(RING_CAP + 5U, logr_head(&g_ring));
}

static void test_sequence_numbers_are_dense_and_monotonic(void)
{
	logr_rec_t out[RING_CAP];
	uint16_t n = 0U;
	uint32_t next = 0U;
	uint32_t gap = 0U;
	uint16_t i;

	/* 200 records through an 8-slot ring: whatever survives must still be
	 * a contiguous run of the most recent sequence numbers. */
	put_n(200U);
	TEST_ASSERT_EQUAL_INT(0, logr_tail(&g_ring, logr_oldest(&g_ring), NULL,
					   out, RING_CAP, &n, &next, &gap));
	TEST_ASSERT_EQUAL_UINT16(RING_CAP, n);
	TEST_ASSERT_EQUAL_UINT32(0U, gap);
	TEST_ASSERT_EQUAL_UINT32(200U, next);

	for (i = 0U; i < n; i++) {
		TEST_ASSERT_EQUAL_UINT32(192U + i, out[i].seq);
		TEST_ASSERT_EQUAL_UINT64(1000U + 192U + i, out[i].mono_ms);
	}
	TEST_ASSERT_EQUAL_UINT32(192U, logr_dropped(&g_ring));
}

static void test_long_message_is_truncated_and_tail_is_scrubbed(void)
{
	char big[LOGR_MSG_MAX + 32U];
	logr_rec_t out[2];
	uint16_t n = 0U;
	uint32_t next = 0U;
	uint32_t gap = 0U;

	memset(big, 'A', sizeof(big));

	TEST_ASSERT_EQUAL_INT(0, logr_put(&g_ring, LOGR_INFO, LOGR_SUB_SYS, 1U,
					  big, sizeof(big)));
	/* A short record reusing the same slot must not expose the leftovers. */
	put_n(RING_CAP);

	TEST_ASSERT_EQUAL_INT(0, logr_tail(&g_ring, logr_oldest(&g_ring), NULL,
					   out, 1U, &n, &next, &gap));
	TEST_ASSERT_EQUAL_UINT16(1U, n);
	TEST_ASSERT_EQUAL_UINT8(2U, out[0].len); /* "m0" */
	TEST_ASSERT_EQUAL_HEX8_ARRAY("m0", out[0].msg, 2);
	TEST_ASSERT_EQUAL_HEX8(0U, out[0].msg[2]);
	TEST_ASSERT_EQUAL_HEX8(0U, out[0].msg[LOGR_MSG_MAX - 1U]);

	/* And the truncated one really was capped at LOGR_MSG_MAX. */
	logr_reset(&g_ring);
	TEST_ASSERT_EQUAL_INT(0, logr_put(&g_ring, LOGR_INFO, LOGR_SUB_SYS, 1U,
					  big, sizeof(big)));
	TEST_ASSERT_EQUAL_INT(0, logr_tail(&g_ring, 0U, NULL, out, 1U, &n,
					   &next, &gap));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)LOGR_MSG_MAX, out[0].len);
}

static void test_reset_clears_everything(void)
{
	put_n(20U);
	TEST_ASSERT_TRUE(logr_dropped(&g_ring) > 0U);

	logr_reset(&g_ring);
	TEST_ASSERT_EQUAL_UINT16(0U, logr_count(&g_ring));
	TEST_ASSERT_EQUAL_UINT32(0U, logr_head(&g_ring));
	TEST_ASSERT_EQUAL_UINT32(0U, logr_dropped(&g_ring));
	TEST_ASSERT_EQUAL_UINT32(0U, logr_filtered(&g_ring));
}

/* ------------------------------------------------------------------------- */
/* Cursors and gaps                                                          */
/* ------------------------------------------------------------------------- */

static void test_cursor_reports_the_gap_it_missed(void)
{
	logr_rec_t out[RING_CAP];
	uint16_t n = 0U;
	uint32_t next = 0U;
	uint32_t gap = 99U;

	put_n(4U);

	/* A reader that keeps up sees no gap and resumes exactly. */
	TEST_ASSERT_EQUAL_INT(0,
		logr_tail(&g_ring, 0U, NULL, out, RING_CAP, &n, &next, &gap));
	TEST_ASSERT_EQUAL_UINT16(4U, n);
	TEST_ASSERT_EQUAL_UINT32(0U, gap);
	TEST_ASSERT_EQUAL_UINT32(4U, next);

	/* Resuming with nothing new pending returns nothing, not an error. */
	TEST_ASSERT_EQUAL_INT(0,
		logr_tail(&g_ring, next, NULL, out, RING_CAP, &n, &next, &gap));
	TEST_ASSERT_EQUAL_UINT16(0U, n);
	TEST_ASSERT_EQUAL_UINT32(4U, next);

	/* Now overrun the reader by 12 records: 4..15 were produced, the ring
	 * holds 8..15, so the reader lost exactly 4. */
	put_n(12U);
	TEST_ASSERT_EQUAL_INT(0,
		logr_tail(&g_ring, 4U, NULL, out, RING_CAP, &n, &next, &gap));
	TEST_ASSERT_EQUAL_UINT32(4U, gap);
	TEST_ASSERT_EQUAL_UINT16(RING_CAP, n);
	TEST_ASSERT_EQUAL_UINT32(8U, out[0].seq);
	TEST_ASSERT_EQUAL_UINT32(16U, next);
}

static void test_cursor_from_the_future_is_clamped(void)
{
	logr_rec_t out[RING_CAP];
	uint16_t n = 1U;
	uint32_t next = 0U;
	uint32_t gap = 1U;

	put_n(3U);
	TEST_ASSERT_EQUAL_INT(0, logr_tail(&g_ring, 9999U, NULL, out, RING_CAP,
					   &n, &next, &gap));
	TEST_ASSERT_EQUAL_UINT16(0U, n);
	TEST_ASSERT_EQUAL_UINT32(0U, gap);
	TEST_ASSERT_EQUAL_UINT32(3U, next);
}

static void test_cursor_survives_a_sequence_wrap(void)
{
	logr_rec_t out[RING_CAP];
	uint16_t n = 0U;
	uint32_t next = 0U;
	uint32_t gap = 0U;

	/* Park the ring just below the 2^32 rollover. Signed-difference cursor
	 * arithmetic must keep working across it. */
	g_ring.head_seq = 0xFFFFFFFEU;
	put_n(4U);

	TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFEU, logr_oldest(&g_ring));
	TEST_ASSERT_EQUAL_UINT32(2U, logr_head(&g_ring));

	TEST_ASSERT_EQUAL_INT(0, logr_tail(&g_ring, 0xFFFFFFFEU, NULL, out,
					   RING_CAP, &n, &next, &gap));
	TEST_ASSERT_EQUAL_UINT16(4U, n);
	TEST_ASSERT_EQUAL_UINT32(0U, gap);
	TEST_ASSERT_EQUAL_UINT32(2U, next);
	TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFU, out[1].seq);
	TEST_ASSERT_EQUAL_UINT32(0U, out[2].seq);
}

static void test_tail_argument_checks(void)
{
	logr_rec_t out[2];
	logr_t empty;
	uint16_t n = 0U;
	uint32_t next = 0U;
	uint32_t gap = 0U;

	memset(&empty, 0, sizeof(empty));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_tail(NULL, 0U, NULL, out, 2U, &n, &next, &gap));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_tail(&empty, 0U, NULL, out, 2U, &n, &next, &gap));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_tail(&g_ring, 0U, NULL, out, 2U, NULL, &next, &gap));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_tail(&g_ring, 0U, NULL, out, 2U, &n, NULL, &gap));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_tail(&g_ring, 0U, NULL, out, 2U, &n, &next, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_tail(&g_ring, 0U, NULL, NULL, 2U, &n, &next, &gap));

	/* max == 0 is a legal probe of the cursor. */
	put_n(3U);
	TEST_ASSERT_EQUAL_INT(0,
		logr_tail(&g_ring, 0U, NULL, NULL, 0U, &n, &next, &gap));
	TEST_ASSERT_EQUAL_UINT16(0U, n);
	TEST_ASSERT_EQUAL_UINT32(0U, next);
}

/* ------------------------------------------------------------------------- */
/* Filters                                                                   */
/* ------------------------------------------------------------------------- */

static void test_reader_filter_by_level_and_subsystem(void)
{
	logr_rec_t out[RING_CAP];
	logr_filter_t f;
	uint16_t n = 0U;
	uint32_t next = 0U;
	uint32_t gap = 0U;

	TEST_ASSERT_EQUAL_INT(0, logr_level_set(&g_ring, 0xFFU, LOGR_DEBUG));
	TEST_ASSERT_EQUAL_INT(0, logr_puts(&g_ring, LOGR_ERR, LOGR_SUB_TIMING,
					   1U, "pps lost"));
	TEST_ASSERT_EQUAL_INT(0, logr_puts(&g_ring, LOGR_DEBUG, LOGR_SUB_TIMING,
					   2U, "tau 200"));
	TEST_ASSERT_EQUAL_INT(0, logr_puts(&g_ring, LOGR_ERR, LOGR_SUB_PWR, 3U,
					   "rail low"));

	/* Severity axis. */
	logr_filter_all(&f);
	f.max_level = (uint8_t)LOGR_WARN;
	TEST_ASSERT_EQUAL_INT(0,
		logr_tail(&g_ring, 0U, &f, out, RING_CAP, &n, &next, &gap));
	TEST_ASSERT_EQUAL_UINT16(2U, n);
	TEST_ASSERT_EQUAL_UINT32(3U, next); /* the filtered record still moved
					     * the cursor */

	/* Subsystem axis. */
	logr_filter_all(&f);
	f.sub_mask = (uint16_t)(1U << LOGR_SUB_PWR);
	TEST_ASSERT_EQUAL_INT(0,
		logr_tail(&g_ring, 0U, &f, out, RING_CAP, &n, &next, &gap));
	TEST_ASSERT_EQUAL_UINT16(1U, n);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)LOGR_SUB_PWR, out[0].subsys);

	/* Both axes at once, matching nothing. */
	f.max_level = (uint8_t)LOGR_EMERG;
	TEST_ASSERT_EQUAL_INT(0,
		logr_tail(&g_ring, 0U, &f, out, RING_CAP, &n, &next, &gap));
	TEST_ASSERT_EQUAL_UINT16(0U, n);
	TEST_ASSERT_EQUAL_UINT32(3U, next); /* still converges */
}

static void test_producer_level_is_per_subsystem(void)
{
	TEST_ASSERT_EQUAL_INT((int)LOGR_INFO,
			      logr_level_get(&g_ring, LOGR_SUB_GNSS));

	TEST_ASSERT_EQUAL_INT(0,
		logr_level_set(&g_ring, LOGR_SUB_GNSS, LOGR_WARN));
	TEST_ASSERT_EQUAL_INT((int)LOGR_WARN,
			      logr_level_get(&g_ring, LOGR_SUB_GNSS));
	TEST_ASSERT_EQUAL_INT((int)LOGR_INFO,
			      logr_level_get(&g_ring, LOGR_SUB_NTP));

	/* Below the threshold: counted as filtered, never stored, never fatal. */
	TEST_ASSERT_EQUAL_INT(1, logr_puts(&g_ring, LOGR_INFO, LOGR_SUB_GNSS,
					   1U, "sat 12"));
	TEST_ASSERT_EQUAL_UINT16(0U, logr_count(&g_ring));
	TEST_ASSERT_EQUAL_UINT32(1U, logr_filtered(&g_ring));

	TEST_ASSERT_EQUAL_INT(0, logr_puts(&g_ring, LOGR_ERR, LOGR_SUB_GNSS, 2U,
					   "antenna open"));
	TEST_ASSERT_EQUAL_UINT16(1U, logr_count(&g_ring));

	/* 0xFF is "all subsystems". */
	TEST_ASSERT_EQUAL_INT(0, logr_level_set(&g_ring, 0xFFU, LOGR_CRIT));
	TEST_ASSERT_EQUAL_INT((int)LOGR_CRIT,
			      logr_level_get(&g_ring, LOGR_SUB_NTP));

	TEST_ASSERT_EQUAL_INT(-EINVAL, logr_level_set(&g_ring, 0U, 8U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_level_set(&g_ring, LOGR_SUB_COUNT, LOGR_INFO));
	TEST_ASSERT_EQUAL_INT(-EINVAL, logr_level_set(NULL, 0U, LOGR_INFO));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_level_get(&g_ring, LOGR_SUB_COUNT));
}

/* ------------------------------------------------------------------------- */
/* Names                                                                     */
/* ------------------------------------------------------------------------- */

static void test_names(void)
{
	TEST_ASSERT_EQUAL_STRING("SYS", logr_sub_name(LOGR_SUB_SYS));
	TEST_ASSERT_EQUAL_STRING("TIMING", logr_sub_name(LOGR_SUB_TIMING));
	TEST_ASSERT_EQUAL_STRING("FAULT", logr_sub_name(LOGR_SUB_FAULT));
	TEST_ASSERT_EQUAL_STRING("UNKNOWN", logr_sub_name(LOGR_SUB_COUNT));

	TEST_ASSERT_EQUAL_STRING("emerg", logr_level_name(LOGR_EMERG));
	TEST_ASSERT_EQUAL_STRING("warning", logr_level_name(LOGR_WARN));
	TEST_ASSERT_EQUAL_STRING("debug", logr_level_name(LOGR_DEBUG));
	TEST_ASSERT_EQUAL_STRING("unknown", logr_level_name(8U));
}

/* ------------------------------------------------------------------------- */
/* RFC 5424 rendering                                                        */
/* ------------------------------------------------------------------------- */

static logr_rec_t mkrec(uint8_t level, uint8_t subsys, const char *msg)
{
	logr_rec_t r;

	memset(&r, 0, sizeof(r));
	r.seq = 7U;
	r.mono_ms = 1234U;
	r.level = level;
	r.subsys = subsys;
	r.len = (uint8_t)strlen(msg);
	memcpy(r.msg, msg, r.len);
	return r;
}

static void test_render_known_vector(void)
{
	/*
	 * PRI = local0 (16) * 8 + warning (4) = 132.
	 * 1785155696 is 2026-07-27T12:34:56Z (cross-checked with
	 * `date -u -d @1785155696`).
	 */
	logr_rec_t r = mkrec((uint8_t)LOGR_WARN, (uint8_t)LOGR_SUB_TIMING,
			     "holdover entered");
	char out[LOGR_RENDER_MAX];
	int n;

	n = logr_render_5424(&r, LOGR_FACILITY_LOCAL0, "meridian", "sts1000",
			     1785155696ULL, 250000U, out, sizeof(out));

	TEST_ASSERT_GREATER_THAN_INT(0, n);
	TEST_ASSERT_EQUAL_STRING(
		"<132>1 2026-07-27T12:34:56.250000Z meridian sts1000 - TIMING - "
		"holdover entered",
		out);
	TEST_ASSERT_EQUAL_INT((int)strlen(out), n);
}

static void test_render_epoch_edges(void)
{
	logr_rec_t r = mkrec((uint8_t)LOGR_EMERG, (uint8_t)LOGR_SUB_SYS, "x");
	char out[LOGR_RENDER_MAX];

	/* The epoch itself. PRI = 16*8 + 0 = 128. */
	TEST_ASSERT_GREATER_THAN_INT(0,
		logr_render_5424(&r, LOGR_FACILITY_LOCAL0, NULL, NULL, 0ULL, 0U,
				 out, sizeof(out)));
	TEST_ASSERT_EQUAL_STRING("<128>1 1970-01-01T00:00:00.000000Z - - - SYS "
				 "- x",
				 out);

	/* A leap day, and the last second before one. */
	TEST_ASSERT_GREATER_THAN_INT(0,
		logr_render_5424(&r, 0U, "h", "a", 1709164800ULL, 1U, out,
				 sizeof(out)));
	TEST_ASSERT_EQUAL_STRING("<0>1 2024-02-29T00:00:00.000001Z h a - SYS - x",
				 out);

	TEST_ASSERT_GREATER_THAN_INT(0,
		logr_render_5424(&r, 0U, "h", "a", 951868799ULL, 999999U, out,
				 sizeof(out)));
	TEST_ASSERT_EQUAL_STRING(
		"<0>1 2000-02-29T23:59:59.999999Z h a - SYS - x", out);

	/* The largest instant a 4-digit year can express. */
	TEST_ASSERT_GREATER_THAN_INT(0,
		logr_render_5424(&r, 0U, "h", "a", 253402300799ULL, 0U, out,
				 sizeof(out)));
	TEST_ASSERT_EQUAL_STRING(
		"<0>1 9999-12-31T23:59:59.000000Z h a - SYS - x", out);
}

static void test_render_sanitises_its_inputs(void)
{
	logr_rec_t r = mkrec((uint8_t)LOGR_INFO, (uint8_t)LOGR_SUB_NET,
			     "line1\nline2\ttab");
	char out[LOGR_RENDER_MAX];
	char longhost[LOGR_HOSTNAME_MAX + 16U];

	/* Control characters in the message would break line framing at the
	 * collector; a space in the hostname would shift every later field. */
	TEST_ASSERT_GREATER_THAN_INT(0,
		logr_render_5424(&r, LOGR_FACILITY_LOCAL0, "bad host", "app",
				 0ULL, 0U, out, sizeof(out)));
	TEST_ASSERT_EQUAL_STRING("<134>1 1970-01-01T00:00:00.000000Z bad_host "
				 "app - NET - line1 line2 tab",
				 out);

	/* HOSTNAME is truncated, not allowed to run away. */
	memset(longhost, 'h', sizeof(longhost));
	longhost[sizeof(longhost) - 1U] = '\0';
	TEST_ASSERT_GREATER_THAN_INT(0,
		logr_render_5424(&r, 0U, longhost, "a", 0ULL, 0U, out,
				 sizeof(out)));
	TEST_ASSERT_NOT_NULL(strstr(out, " a - NET - "));
	TEST_ASSERT_EQUAL_size_t(LOGR_HOSTNAME_MAX,
				 strcspn(&out[strlen("<0>1 "
						     "1970-01-01T00:00:00."
						     "000000Z ")],
					 " "));
}

static void test_render_argument_and_space_checks(void)
{
	logr_rec_t r = mkrec((uint8_t)LOGR_INFO, (uint8_t)LOGR_SUB_SYS, "hello");
	char out[LOGR_RENDER_MAX];
	char tiny[16];

	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_render_5424(NULL, 0U, NULL, NULL, 0ULL, 0U, out,
				 sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_render_5424(&r, 0U, NULL, NULL, 0ULL, 0U, NULL,
				 sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_render_5424(&r, 0U, NULL, NULL, 0ULL, 0U, out, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_render_5424(&r, 24U, NULL, NULL, 0ULL, 0U, out,
				 sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_render_5424(&r, 0U, NULL, NULL, 0ULL, 1000000U, out,
				 sizeof(out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_render_5424(&r, 0U, NULL, NULL, 253402300800ULL, 0U, out,
				 sizeof(out)));

	r.len = (uint8_t)(LOGR_MSG_MAX + 1U);
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		logr_render_5424(&r, 0U, NULL, NULL, 0ULL, 0U, out,
				 sizeof(out)));
	r.len = 5U;

	/* Too small: reported, and still NUL-terminated. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
		logr_render_5424(&r, 0U, NULL, NULL, 0ULL, 0U, tiny,
				 sizeof(tiny)));
	TEST_ASSERT_EQUAL_size_t(sizeof(tiny) - 1U, strlen(tiny));
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_init_rejects_bad_arguments);
	RUN_TEST(test_put_rejects_bad_arguments);

	RUN_TEST(test_wrap_counts_every_evicted_record);
	RUN_TEST(test_sequence_numbers_are_dense_and_monotonic);
	RUN_TEST(test_long_message_is_truncated_and_tail_is_scrubbed);
	RUN_TEST(test_reset_clears_everything);

	RUN_TEST(test_cursor_reports_the_gap_it_missed);
	RUN_TEST(test_cursor_from_the_future_is_clamped);
	RUN_TEST(test_cursor_survives_a_sequence_wrap);
	RUN_TEST(test_tail_argument_checks);

	RUN_TEST(test_reader_filter_by_level_and_subsystem);
	RUN_TEST(test_producer_level_is_per_subsystem);
	RUN_TEST(test_names);

	RUN_TEST(test_render_known_vector);
	RUN_TEST(test_render_epoch_edges);
	RUN_TEST(test_render_sanitises_its_inputs);
	RUN_TEST(test_render_argument_and_space_checks);

	return UNITY_END();
}
