/*
 * STS1000 "Meridian" — the GNSS NMEA/UBX run classifier, tested away from Zephyr.
 *
 * platform/gnss_tee.h splits the raw USART3 byte stream onto the maintenance
 * tool's two capture channels. Every defect it can carry is silent at runtime:
 * nothing crashes, the timing engine is unaffected, and the only symptom is a
 * .nmea file with UBX frames in it, or a sentence torn across two channels, or
 * a run that never gets emitted at all — none of which anyone notices until a
 * technician is trying to diagnose an outage from a capture that will not parse.
 * That is the whole reason the classifier is a Zephyr-free header.
 *
 * The vectors are written against the rule as specified in that header, not as
 * a round-trip of the implementation:
 *
 *   - the parser-state evidence (`hunting`) is supplied by the test, exactly as
 *     gnss.c samples it before ubx_parse_byte() consumes the byte;
 *   - the '$'-inside-a-UBX-payload case is built by construction, because that
 *     is the case a naive "default UBX, '$' switches" classifier gets wrong and
 *     is the reason `hunting` is threaded through at all;
 *   - the 64-byte run boundary is asserted as a number, since it is what bounds
 *     the interval the sink's lock masks interrupts for.
 */

#include <string.h>

#include "unity.h"

#include "zephyr/platform/gnss_tee.h"

/* ------------------------------------------------------------------ fixture */

#define CAP 512

typedef struct {
	unsigned int calls;
	bool nmea[32];
	size_t len[32];
	uint8_t nmea_bytes[CAP];
	size_t nmea_n;
	uint8_t ubx_bytes[CAP];
	size_t ubx_n;
} sink_log_t;

static sink_log_t log_;
static gnss_tee_t t;

static void sink(void *user, bool nmea, const uint8_t *data, size_t len)
{
	sink_log_t *l = (sink_log_t *)user;

	TEST_ASSERT_NOT_NULL(data);
	/* The header promises 1..GNSS_TEE_STAGE, never an empty run. */
	TEST_ASSERT_GREATER_THAN_UINT32(0U, (uint32_t)len);
	TEST_ASSERT_LESS_OR_EQUAL_UINT32((uint32_t)GNSS_TEE_STAGE, (uint32_t)len);

	if (l->calls < 32U) {
		l->nmea[l->calls] = nmea;
		l->len[l->calls] = len;
	}
	l->calls++;

	if (nmea) {
		TEST_ASSERT_LESS_OR_EQUAL_UINT32(CAP, (uint32_t)(l->nmea_n + len));
		memcpy(&l->nmea_bytes[l->nmea_n], data, len);
		l->nmea_n += len;
	} else {
		TEST_ASSERT_LESS_OR_EQUAL_UINT32(CAP, (uint32_t)(l->ubx_n + len));
		memcpy(&l->ubx_bytes[l->ubx_n], data, len);
		l->ubx_n += len;
	}
}

void setUp(void)
{
	memset(&log_, 0, sizeof(log_));
	memset(&t, 0, sizeof(t));
	gnss_tee_init(&t, sink, &log_);
}

void tearDown(void)
{
}

/** Feed a string as a UBX-payload run: the parser is never hunting. */
static void feed_in_frame(const char *s)
{
	size_t i;

	for (i = 0U; s[i] != '\0'; i++) {
		gnss_tee_byte(&t, (uint8_t)s[i], false);
	}
}

/** Feed a string with the parser hunting on every byte. */
static void feed_hunting(const char *s)
{
	size_t i;

	for (i = 0U; s[i] != '\0'; i++) {
		gnss_tee_byte(&t, (uint8_t)s[i], true);
	}
}

/* ------------------------------------------------------------ the basic rule */

static void test_hunting_dollar_starts_an_nmea_sentence(void)
{
	feed_hunting("$GPGGA,1\r\n");
	gnss_tee_flush(&t);

	TEST_ASSERT_EQUAL_UINT32(1U, log_.calls);
	TEST_ASSERT_TRUE(log_.nmea[0]);
	TEST_ASSERT_EQUAL_UINT32(10U, (uint32_t)log_.nmea_n);
	TEST_ASSERT_EQUAL_UINT8_ARRAY("$GPGGA,1\r\n", log_.nmea_bytes, 10);
	TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)log_.ubx_n);
}

static void test_sentence_ends_at_the_lf_not_the_cr(void)
{
	/* The LF closes the sentence; the next byte is UBX again. */
	feed_hunting("$A\r\n");
	gnss_tee_byte(&t, 0xB5U, true);
	gnss_tee_flush(&t);

	TEST_ASSERT_EQUAL_UINT32(2U, log_.calls);
	TEST_ASSERT_TRUE(log_.nmea[0]);
	TEST_ASSERT_EQUAL_UINT32(4U, (uint32_t)log_.len[0]);
	TEST_ASSERT_FALSE(log_.nmea[1]);
	TEST_ASSERT_EQUAL_UINT32(1U, (uint32_t)log_.ubx_n);
	TEST_ASSERT_EQUAL_UINT8(0xB5U, log_.ubx_bytes[0]);
}

static void test_bytes_inside_a_sentence_stay_nmea_even_when_not_hunting(void)
{
	/*
	 * Once a sentence is open the parser's state is irrelevant — it is hunting
	 * through the sentence anyway, but the classifier must not consult it.
	 */
	gnss_tee_byte(&t, (uint8_t)'$', true);
	feed_in_frame("GPRMC");
	gnss_tee_byte(&t, (uint8_t)'\n', false);
	gnss_tee_flush(&t);

	TEST_ASSERT_EQUAL_UINT32(1U, log_.calls);
	TEST_ASSERT_TRUE(log_.nmea[0]);
	TEST_ASSERT_EQUAL_UINT8_ARRAY("$GPRMC\n", log_.nmea_bytes, 7);
}

/* ------------------------------------------- the case `hunting` exists for */

static void test_dollar_inside_a_ubx_payload_is_not_nmea(void)
{
	/*
	 * THE case. 0x24 ('$') is an ordinary payload byte and appears constantly
	 * in NAV-SAT and MON-RF. A classifier that switched on '$' alone would
	 * derail here and dump the rest of the frame into the NMEA capture.
	 */
	feed_in_frame("ab$cd");
	gnss_tee_flush(&t);

	TEST_ASSERT_EQUAL_UINT32(1U, log_.calls);
	TEST_ASSERT_FALSE(log_.nmea[0]);
	TEST_ASSERT_EQUAL_UINT32(5U, (uint32_t)log_.ubx_n);
	TEST_ASSERT_EQUAL_UINT8_ARRAY("ab$cd", log_.ubx_bytes, 5);
	TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)log_.nmea_n);
}

static void test_sync_bytes_while_hunting_are_ubx(void)
{
	/* B5 62 arrives while hunting and must not be mistaken for anything. */
	gnss_tee_byte(&t, 0xB5U, true);
	gnss_tee_byte(&t, 0x62U, false);
	gnss_tee_flush(&t);

	TEST_ASSERT_EQUAL_UINT32(1U, log_.calls);
	TEST_ASSERT_FALSE(log_.nmea[0]);
	TEST_ASSERT_EQUAL_UINT32(2U, (uint32_t)log_.ubx_n);
}

/* --------------------------------------------------------------- batching */

static void test_a_run_is_closed_when_the_channel_changes(void)
{
	feed_in_frame("UUU");      /* UBX run */
	feed_hunting("$N\n");      /* NMEA run */
	feed_in_frame("VV");       /* UBX run */
	gnss_tee_flush(&t);

	TEST_ASSERT_EQUAL_UINT32(3U, log_.calls);
	TEST_ASSERT_FALSE(log_.nmea[0]);
	TEST_ASSERT_EQUAL_UINT32(3U, (uint32_t)log_.len[0]);
	TEST_ASSERT_TRUE(log_.nmea[1]);
	TEST_ASSERT_EQUAL_UINT32(3U, (uint32_t)log_.len[1]);
	TEST_ASSERT_FALSE(log_.nmea[2]);
	TEST_ASSERT_EQUAL_UINT32(2U, (uint32_t)log_.len[2]);

	/* Order is preserved within each channel. */
	TEST_ASSERT_EQUAL_UINT8_ARRAY("UUUVV", log_.ubx_bytes, 5);
	TEST_ASSERT_EQUAL_UINT8_ARRAY("$N\n", log_.nmea_bytes, 3);
}

static void test_run_is_emitted_at_exactly_the_stage_size(void)
{
	unsigned int i;

	/* 64 bytes must emit by themselves, with no flush from the caller. */
	for (i = 0U; i < GNSS_TEE_STAGE; i++) {
		gnss_tee_byte(&t, (uint8_t)'x', false);
	}
	TEST_ASSERT_EQUAL_UINT32(1U, log_.calls);
	TEST_ASSERT_EQUAL_UINT32((uint32_t)GNSS_TEE_STAGE, (uint32_t)log_.len[0]);

	/* The 65th starts a fresh run that is still pending. */
	gnss_tee_byte(&t, (uint8_t)'y', false);
	TEST_ASSERT_EQUAL_UINT32(1U, log_.calls);
	gnss_tee_flush(&t);
	TEST_ASSERT_EQUAL_UINT32(2U, log_.calls);
	TEST_ASSERT_EQUAL_UINT32(1U, (uint32_t)log_.len[1]);
	TEST_ASSERT_EQUAL_UINT32((uint32_t)GNSS_TEE_STAGE + 1U,
				 (uint32_t)log_.ubx_n);
}

static void test_a_sentence_longer_than_the_stage_splits_but_stays_nmea(void)
{
	unsigned int i;

	gnss_tee_byte(&t, (uint8_t)'$', true);
	for (i = 0U; i < GNSS_TEE_STAGE; i++) {
		gnss_tee_byte(&t, (uint8_t)'a', false);
	}
	gnss_tee_byte(&t, (uint8_t)'\n', false);
	gnss_tee_flush(&t);

	TEST_ASSERT_EQUAL_UINT32(2U, log_.calls);
	TEST_ASSERT_TRUE(log_.nmea[0]);
	TEST_ASSERT_TRUE(log_.nmea[1]);
	TEST_ASSERT_EQUAL_UINT32((uint32_t)GNSS_TEE_STAGE + 2U,
				 (uint32_t)log_.nmea_n);
	TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)log_.ubx_n);
}

static void test_flush_with_nothing_staged_emits_nothing(void)
{
	gnss_tee_flush(&t);
	gnss_tee_flush(&t);
	TEST_ASSERT_EQUAL_UINT32(0U, log_.calls);
}

/* ----------------------------------------------------------------- reset */

static void test_reset_drops_the_staged_run_rather_than_flushing_it(void)
{
	feed_in_frame("abc");
	gnss_tee_reset(&t);
	gnss_tee_flush(&t);

	/* Those bytes belong to a session that no longer exists. */
	TEST_ASSERT_EQUAL_UINT32(0U, log_.calls);
	TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)log_.ubx_n);
}

static void test_reset_clears_the_open_sentence(void)
{
	gnss_tee_byte(&t, (uint8_t)'$', true);
	gnss_tee_byte(&t, (uint8_t)'G', false);
	gnss_tee_reset(&t);

	/*
	 * Without the in_nmea clear, this byte would be classified NMEA on the
	 * strength of a '$' from before a receiver reset.
	 */
	gnss_tee_byte(&t, (uint8_t)'Z', false);
	gnss_tee_flush(&t);

	TEST_ASSERT_EQUAL_UINT32(1U, log_.calls);
	TEST_ASSERT_FALSE(log_.nmea[0]);
	TEST_ASSERT_EQUAL_UINT32(1U, (uint32_t)log_.ubx_n);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)'Z', log_.ubx_bytes[0]);
}

/* ------------------------------------------------------------- robustness */

static void test_null_arguments_are_ignored(void)
{
	/* The GNSS path must not fault on a mis-wired tee. */
	gnss_tee_init(NULL, sink, &log_);
	gnss_tee_byte(NULL, (uint8_t)'a', true);
	gnss_tee_flush(NULL);
	gnss_tee_reset(NULL);
	TEST_ASSERT_EQUAL_UINT32(0U, log_.calls);
}

static void test_a_null_sink_still_consumes_and_clears(void)
{
	gnss_tee_t bare;

	memset(&bare, 0, sizeof(bare));
	gnss_tee_init(&bare, NULL, NULL);
	gnss_tee_byte(&bare, (uint8_t)'a', false);
	gnss_tee_flush(&bare);
	TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)bare.len);
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_hunting_dollar_starts_an_nmea_sentence);
	RUN_TEST(test_sentence_ends_at_the_lf_not_the_cr);
	RUN_TEST(test_bytes_inside_a_sentence_stay_nmea_even_when_not_hunting);

	RUN_TEST(test_dollar_inside_a_ubx_payload_is_not_nmea);
	RUN_TEST(test_sync_bytes_while_hunting_are_ubx);

	RUN_TEST(test_a_run_is_closed_when_the_channel_changes);
	RUN_TEST(test_run_is_emitted_at_exactly_the_stage_size);
	RUN_TEST(test_a_sentence_longer_than_the_stage_splits_but_stays_nmea);
	RUN_TEST(test_flush_with_nothing_staged_emits_nothing);

	RUN_TEST(test_reset_drops_the_staged_run_rather_than_flushing_it);
	RUN_TEST(test_reset_clears_the_open_sentence);

	RUN_TEST(test_null_arguments_are_ignored);
	RUN_TEST(test_a_null_sink_still_consumes_and_clears);

	return UNITY_END();
}
