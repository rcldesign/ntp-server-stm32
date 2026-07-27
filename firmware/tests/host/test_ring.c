/*
 * STS1000 "Meridian" — core/util/ring unit tests.
 *
 * A ring buffer has no published vector set to check against, so the
 * expectations here are structural: the FIFO ordering property, the capacity
 * and wrap boundaries, and the free/len invariant. The long test at the end
 * runs the ring against a reference byte stream through twenty thousand random
 * put/peek/get operations and asserts the invariants on every one.
 *
 * One test reaches into ring_t's fields directly. That is deliberate and
 * annotated: the free-running indices wrap at SIZE_MAX, which on this target is
 * 2^32 — about ten hours of continuous traffic on the 921600-baud console — and
 * there is no way to reach that boundary through the public API inside a test.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"
#include "util/ring.h"

#include "test_support.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* Position-dependent byte stream: catches reordering, duplication and loss. */
static uint8_t stream_byte(size_t i)
{
	return (uint8_t)(i ^ (i >> 8) ^ (i >> 16) ^ 0x5AU);
}

/* ------------------------------------------------------------------ init - */

static void test_ring_init_requires_a_power_of_two_capacity(void)
{
	static const size_t good[] = {1U, 2U, 4U, 8U, 16U, 64U, 256U, 1024U};
	static const size_t bad[] = {0U, 3U, 5U, 6U, 7U, 9U, 100U, 255U, 1000U};
	uint8_t store[1024];
	ring_t r;
	size_t i;

	for (i = 0U; i < ARRAY_LEN(good); i++) {
		TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, good[i]));
		TEST_ASSERT_EQUAL_size_t(good[i], ring_cap(&r));
	}
	for (i = 0U; i < ARRAY_LEN(bad); i++) {
		TEST_ASSERT_EQUAL_INT(-EINVAL, ring_init(&r, store, bad[i]));
	}

	TEST_ASSERT_EQUAL_INT(-EINVAL, ring_init(NULL, store, 8U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ring_init(&r, NULL, 8U));
}

static void test_ring_starts_empty(void)
{
	uint8_t store[8];
	ring_t r;

	TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, sizeof(store)));
	TEST_ASSERT_EQUAL_size_t(0U, ring_len(&r));
	TEST_ASSERT_EQUAL_size_t(sizeof(store), ring_free(&r));
	TEST_ASSERT_EQUAL_size_t(sizeof(store), ring_cap(&r));
	TEST_ASSERT_TRUE(ring_empty(&r));
	TEST_ASSERT_FALSE(ring_full(&r));
}

static void test_ring_rejects_null_arguments(void)
{
	uint8_t store[8];
	uint8_t b = 0U;
	ring_t r;

	TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, sizeof(store)));

	TEST_ASSERT_EQUAL_size_t(0U, ring_cap(NULL));
	TEST_ASSERT_EQUAL_size_t(0U, ring_len(NULL));
	TEST_ASSERT_EQUAL_size_t(0U, ring_free(NULL));
	TEST_ASSERT_TRUE(ring_empty(NULL));
	TEST_ASSERT_FALSE(ring_full(NULL));

	TEST_ASSERT_EQUAL_size_t(0U, ring_put(NULL, &b, 1U));
	TEST_ASSERT_EQUAL_size_t(0U, ring_put(&r, NULL, 1U));
	TEST_ASSERT_EQUAL_size_t(0U, ring_get(NULL, &b, 1U));
	TEST_ASSERT_EQUAL_size_t(0U, ring_get(&r, NULL, 1U));
	TEST_ASSERT_EQUAL_size_t(0U, ring_peek(NULL, &b, 1U));
	TEST_ASSERT_EQUAL_size_t(0U, ring_peek(&r, NULL, 1U));
	TEST_ASSERT_EQUAL_size_t(0U, ring_discard(NULL, 1U));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ring_putc(NULL, 0x11U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ring_getc(NULL, &b));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ring_getc(&r, NULL));

	ring_reset(NULL); /* must not fault */
}

static void test_ring_zero_length_operations_are_no_ops(void)
{
	uint8_t store[8];
	ring_t r;

	TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, sizeof(store)));

	TEST_ASSERT_EQUAL_size_t(0U, ring_put(&r, NULL, 0U));
	TEST_ASSERT_EQUAL_size_t(0U, ring_get(&r, NULL, 0U));
	TEST_ASSERT_EQUAL_size_t(0U, ring_peek(&r, NULL, 0U));
	TEST_ASSERT_EQUAL_size_t(0U, ring_discard(&r, 0U));
	TEST_ASSERT_EQUAL_size_t(0U, ring_len(&r));

	/* And on an empty ring, discarding "everything" is still nothing. */
	ring_reset(&r);
	TEST_ASSERT_EQUAL_size_t(0U, ring_len(&r));
}

/* ------------------------------------------------------------ basic FIFO - */

static void test_ring_put_get_preserves_order(void)
{
	static const uint8_t src[] = {0x11, 0x22, 0x33, 0x44, 0x55};
	uint8_t store[8];
	uint8_t dst[8];
	ring_t r;

	TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, sizeof(store)));
	TEST_ASSERT_EQUAL_size_t(5U, ring_put(&r, src, 5U));
	TEST_ASSERT_EQUAL_size_t(5U, ring_len(&r));
	TEST_ASSERT_EQUAL_size_t(3U, ring_free(&r));

	TEST_ASSERT_EQUAL_size_t(5U, ring_get(&r, dst, sizeof(dst)));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(src, dst, 5U);
	TEST_ASSERT_TRUE(ring_empty(&r));
}

static void test_ring_uses_its_whole_capacity(void)
{
	uint8_t store[8];
	uint8_t src[8];
	uint8_t dst[8];
	ring_t r;

	/* Free-running indices mean no slot is sacrificed to distinguish full
	 * from empty: all eight bytes must be storable. */
	test_fill_seq(src, sizeof(src), 0xA0U);
	TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, sizeof(store)));
	TEST_ASSERT_EQUAL_size_t(8U, ring_put(&r, src, 8U));
	TEST_ASSERT_TRUE(ring_full(&r));
	TEST_ASSERT_FALSE(ring_empty(&r));
	TEST_ASSERT_EQUAL_size_t(0U, ring_free(&r));

	/* Nothing more fits. */
	TEST_ASSERT_EQUAL_size_t(0U, ring_put(&r, src, 1U));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ring_putc(&r, 0xFFU));

	TEST_ASSERT_EQUAL_size_t(8U, ring_get(&r, dst, sizeof(dst)));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(src, dst, 8U);
}

static void test_ring_put_is_partial_when_space_runs_out(void)
{
	uint8_t store[8];
	uint8_t src[16];
	uint8_t dst[16];
	ring_t r;

	test_fill_seq(src, sizeof(src), 0x01U);
	TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, sizeof(store)));

	TEST_ASSERT_EQUAL_size_t(5U, ring_put(&r, src, 5U));
	/* Only three bytes of room left, so a ten-byte offer takes three. */
	TEST_ASSERT_EQUAL_size_t(3U, ring_put(&r, &src[5], 10U));
	TEST_ASSERT_TRUE(ring_full(&r));

	TEST_ASSERT_EQUAL_size_t(8U, ring_get(&r, dst, sizeof(dst)));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(src, dst, 8U);
}

static void test_ring_get_is_partial_when_data_runs_out(void)
{
	static const uint8_t src[] = {0x11, 0x22, 0x33};
	uint8_t store[8];
	uint8_t dst[8];
	ring_t r;

	TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, sizeof(store)));
	TEST_ASSERT_EQUAL_size_t(3U, ring_put(&r, src, 3U));

	memset(dst, 0xEE, sizeof(dst));
	TEST_ASSERT_EQUAL_size_t(3U, ring_get(&r, dst, 8U));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(src, dst, 3U);
	TEST_ASSERT_EQUAL_HEX8(0xEEU, dst[3]); /* untouched past the data */
	TEST_ASSERT_EQUAL_size_t(0U, ring_get(&r, dst, 8U));
}

/* ------------------------------------------------------------------ wrap - */

static void test_ring_wraps_the_backing_store(void)
{
	uint8_t store[8];
	uint8_t src[8];
	uint8_t dst[8];
	ring_t r;
	size_t round;

	/* Fill and drain the whole ring at every possible starting phase. Phase 0
	 * takes the single-memcpy path; every other phase splits the copy across
	 * the end of the backing store. Thirty-two rounds cover all eight phases
	 * four times over. */
	test_fill_seq(src, sizeof(src), 0x10U);
	TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, sizeof(store)));

	for (round = 0U; round < 32U; round++) {
		memset(dst, 0, sizeof(dst));
		TEST_ASSERT_EQUAL_size_t(8U, ring_put(&r, src, 8U));
		TEST_ASSERT_TRUE(ring_full(&r));
		TEST_ASSERT_EQUAL_size_t(8U, ring_get(&r, dst, 8U));
		TEST_ASSERT_EQUAL_HEX8_ARRAY(src, dst, 8U);
		TEST_ASSERT_TRUE(ring_empty(&r));

		/* Advance the phase by one for the next round. */
		TEST_ASSERT_EQUAL_size_t(1U, ring_put(&r, src, 1U));
		TEST_ASSERT_EQUAL_size_t(1U, ring_get(&r, dst, 1U));
	}
}

static void test_ring_index_wraparound_at_size_max(void)
{
	uint8_t store[8];
	uint8_t src[8];
	uint8_t dst[8];
	ring_t r;
	size_t round;

	/*
	 * White-box: the public API cannot drive the free-running indices to
	 * SIZE_MAX inside a test, but on a 32-bit target they get there in about
	 * ten hours of saturated console traffic. Park both indices three bytes
	 * short of the wrap and run the ring across it.
	 */
	test_fill_seq(src, sizeof(src), 0x70U);
	TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, sizeof(store)));
	atomic_store(&r.head, SIZE_MAX - 3U);
	atomic_store(&r.tail, SIZE_MAX - 3U);

	TEST_ASSERT_EQUAL_size_t(0U, ring_len(&r));
	TEST_ASSERT_EQUAL_size_t(8U, ring_free(&r));

	for (round = 0U; round < 4U; round++) {
		memset(dst, 0, sizeof(dst));
		TEST_ASSERT_EQUAL_size_t(6U, ring_put(&r, src, 6U));
		TEST_ASSERT_EQUAL_size_t(6U, ring_len(&r));
		TEST_ASSERT_EQUAL_size_t(6U, ring_get(&r, dst, 6U));
		TEST_ASSERT_EQUAL_HEX8_ARRAY(src, dst, 6U);
		TEST_ASSERT_EQUAL_size_t(0U, ring_len(&r));
	}
}

/* ------------------------------------------------------------------ peek - */

static void test_ring_peek_does_not_consume(void)
{
	static const uint8_t src[] = {0x11, 0x22, 0x33, 0x44};
	uint8_t store[8];
	uint8_t dst[8];
	ring_t r;

	TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, sizeof(store)));
	TEST_ASSERT_EQUAL_size_t(4U, ring_put(&r, src, 4U));

	TEST_ASSERT_EQUAL_size_t(2U, ring_peek(&r, dst, 2U));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(src, dst, 2U);
	TEST_ASSERT_EQUAL_size_t(4U, ring_len(&r));

	/* Peeking more than is buffered returns what there is. */
	TEST_ASSERT_EQUAL_size_t(4U, ring_peek(&r, dst, 8U));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(src, dst, 4U);
	TEST_ASSERT_EQUAL_size_t(4U, ring_len(&r));

	TEST_ASSERT_EQUAL_size_t(4U, ring_get(&r, dst, 8U));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(src, dst, 4U);
	TEST_ASSERT_EQUAL_size_t(0U, ring_peek(&r, dst, 8U));
}

static void test_ring_peek_spans_the_wrap(void)
{
	uint8_t store[8];
	uint8_t src[8];
	uint8_t dst[8];
	ring_t r;

	test_fill_seq(src, sizeof(src), 0x30U);
	TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, sizeof(store)));

	/* Advance the read index to 6 so the next six bytes straddle the end. */
	TEST_ASSERT_EQUAL_size_t(6U, ring_put(&r, src, 6U));
	TEST_ASSERT_EQUAL_size_t(6U, ring_get(&r, dst, 6U));
	TEST_ASSERT_EQUAL_size_t(6U, ring_put(&r, src, 6U));

	memset(dst, 0, sizeof(dst));
	TEST_ASSERT_EQUAL_size_t(6U, ring_peek(&r, dst, 6U));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(src, dst, 6U);
	TEST_ASSERT_EQUAL_size_t(6U, ring_len(&r));
}

/* ------------------------------------------------------- byte-at-a-time -- */

static void test_ring_putc_getc(void)
{
	uint8_t store[4];
	uint8_t b = 0U;
	ring_t r;
	size_t i;

	TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, sizeof(store)));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, ring_getc(&r, &b));

	for (i = 0U; i < 4U; i++) {
		TEST_ASSERT_EQUAL_INT(0, ring_putc(&r, (uint8_t)(0x40U + i)));
	}
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ring_putc(&r, 0xFFU));

	for (i = 0U; i < 4U; i++) {
		TEST_ASSERT_EQUAL_INT(0, ring_getc(&r, &b));
		TEST_ASSERT_EQUAL_HEX8((uint8_t)(0x40U + i), b);
	}
	TEST_ASSERT_EQUAL_INT(-EAGAIN, ring_getc(&r, &b));
}

/* --------------------------------------------------------------- discard - */

static void test_ring_discard_drops_from_the_front(void)
{
	static const uint8_t src[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
	uint8_t store[8];
	uint8_t dst[8];
	ring_t r;

	TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, sizeof(store)));
	TEST_ASSERT_EQUAL_size_t(6U, ring_put(&r, src, 6U));

	TEST_ASSERT_EQUAL_size_t(2U, ring_discard(&r, 2U));
	TEST_ASSERT_EQUAL_size_t(4U, ring_len(&r));
	TEST_ASSERT_EQUAL_size_t(4U, ring_get(&r, dst, sizeof(dst)));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(&src[2], dst, 4U);

	/* Discarding more than is buffered drops what there is. */
	TEST_ASSERT_EQUAL_size_t(6U, ring_put(&r, src, 6U));
	TEST_ASSERT_EQUAL_size_t(6U, ring_discard(&r, 100U));
	TEST_ASSERT_TRUE(ring_empty(&r));
	TEST_ASSERT_EQUAL_size_t(0U, ring_discard(&r, 100U));
}

static void test_ring_reset_empties_the_ring(void)
{
	static const uint8_t src[] = {0x11, 0x22, 0x33};
	uint8_t store[8];
	ring_t r;

	TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, sizeof(store)));
	TEST_ASSERT_EQUAL_size_t(3U, ring_put(&r, src, 3U));
	ring_reset(&r);
	TEST_ASSERT_TRUE(ring_empty(&r));
	TEST_ASSERT_EQUAL_size_t(sizeof(store), ring_free(&r));

	/* The ring is still usable, and the stale bytes are gone. */
	TEST_ASSERT_EQUAL_size_t(3U, ring_put(&r, src, 3U));
	TEST_ASSERT_EQUAL_size_t(3U, ring_len(&r));
}

/* ------------------------------------------------------- degenerate case - */

static void test_ring_capacity_of_one(void)
{
	uint8_t store[1];
	uint8_t b = 0U;
	ring_t r;

	/* One is a power of two, so it is legal, and the boundary logic must
	 * hold with the ring simultaneously one byte from empty and from full. */
	TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, 1U));
	TEST_ASSERT_TRUE(ring_empty(&r));
	TEST_ASSERT_FALSE(ring_full(&r));

	TEST_ASSERT_EQUAL_INT(0, ring_putc(&r, 0x5AU));
	TEST_ASSERT_TRUE(ring_full(&r));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ring_putc(&r, 0xA5U));

	TEST_ASSERT_EQUAL_INT(0, ring_getc(&r, &b));
	TEST_ASSERT_EQUAL_HEX8(0x5AU, b);
	TEST_ASSERT_TRUE(ring_empty(&r));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, ring_getc(&r, &b));
}

/* ---------------------------------------------------------------- stress - */

static void test_ring_stress_against_a_reference_stream(void)
{
	uint8_t store[64];
	uint8_t chunk[80];
	uint8_t peeked[80];
	ring_t r;
	test_rng_t rng;
	size_t produced = 0U;
	size_t consumed = 0U;
	size_t iter;
	bool saw_full = false;
	bool saw_empty = false;

	TEST_ASSERT_EQUAL_INT(0, ring_init(&r, store, sizeof(store)));
	test_rng_init(&rng, 0xD15EA5EU);

	for (iter = 0U; iter < 20000U; iter++) {
		size_t n = test_rng_below(&rng, sizeof(chunk) + 1U);
		size_t k;

		/* Invariants that must hold at every point in the run. */
		TEST_ASSERT_EQUAL_size_t(produced - consumed, ring_len(&r));
		TEST_ASSERT_EQUAL_size_t(sizeof(store),
					 ring_len(&r) + ring_free(&r));
		TEST_ASSERT_TRUE(ring_empty(&r) == (ring_len(&r) == 0U));
		TEST_ASSERT_TRUE(ring_full(&r) == (ring_len(&r) == sizeof(store)));
		saw_full = saw_full || ring_full(&r);
		saw_empty = saw_empty || ring_empty(&r);

		if ((test_rng_u32(&rng) & 1U) != 0U) {
			size_t put;

			for (k = 0U; k < n; k++) {
				chunk[k] = stream_byte(produced + k);
			}
			put = ring_put(&r, chunk, n);
			TEST_ASSERT_LESS_OR_EQUAL_size_t(n, put);
			produced += put;
		} else {
			size_t peek_n = ring_peek(&r, peeked, n);
			size_t got = ring_get(&r, chunk, n);

			/* peek and get must agree on both count and content. */
			TEST_ASSERT_EQUAL_size_t(peek_n, got);
			for (k = 0U; k < got; k++) {
				TEST_ASSERT_EQUAL_HEX8(peeked[k], chunk[k]);
				TEST_ASSERT_EQUAL_HEX8(stream_byte(consumed + k),
						       chunk[k]);
			}
			consumed += got;
		}
	}

	/* If the run never hit either boundary it was not testing much. */
	TEST_ASSERT_TRUE_MESSAGE(saw_full, "stress run never filled the ring");
	TEST_ASSERT_TRUE_MESSAGE(saw_empty, "stress run never emptied the ring");
	TEST_ASSERT_GREATER_THAN_size_t(50000U, produced);
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_ring_init_requires_a_power_of_two_capacity);
	RUN_TEST(test_ring_starts_empty);
	RUN_TEST(test_ring_rejects_null_arguments);
	RUN_TEST(test_ring_zero_length_operations_are_no_ops);

	RUN_TEST(test_ring_put_get_preserves_order);
	RUN_TEST(test_ring_uses_its_whole_capacity);
	RUN_TEST(test_ring_put_is_partial_when_space_runs_out);
	RUN_TEST(test_ring_get_is_partial_when_data_runs_out);

	RUN_TEST(test_ring_wraps_the_backing_store);
	RUN_TEST(test_ring_index_wraparound_at_size_max);

	RUN_TEST(test_ring_peek_does_not_consume);
	RUN_TEST(test_ring_peek_spans_the_wrap);

	RUN_TEST(test_ring_putc_getc);
	RUN_TEST(test_ring_discard_drops_from_the_front);
	RUN_TEST(test_ring_reset_empties_the_ring);
	RUN_TEST(test_ring_capacity_of_one);
	RUN_TEST(test_ring_stress_against_a_reference_stream);

	return UNITY_END();
}
