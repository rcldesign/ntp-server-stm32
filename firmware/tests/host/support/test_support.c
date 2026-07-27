/*
 * STS1000 "Meridian" — shared helpers for the host unit tests.
 */

#include "test_support.h"

#include "unity.h"

/*
 * Unity requires setUp()/tearDown() in every test binary. Providing weak
 * no-ops here keeps a new test_<module>.c to just its test cases plus main();
 * a suite that needs a fixture defines its own strong versions.
 */
__attribute__((weak)) void setUp(void)
{
}

__attribute__((weak)) void tearDown(void)
{
}

void test_rng_init(test_rng_t *g, uint32_t seed)
{
	/* xorshift32 is stuck at zero; remap to a fixed non-zero state. */
	g->s = (seed != 0U) ? seed : 0x9E3779B9U;
}

uint32_t test_rng_u32(test_rng_t *g)
{
	uint32_t x = g->s;

	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	g->s = x;
	return x;
}

size_t test_rng_below(test_rng_t *g, size_t bound)
{
	if (bound == 0U) {
		return 0U;
	}
	return (size_t)test_rng_u32(g) % bound;
}

void test_rng_fill(test_rng_t *g, uint8_t *buf, size_t n)
{
	size_t i;

	for (i = 0U; i < n; i++) {
		buf[i] = (uint8_t)(test_rng_u32(g) >> 24);
	}
}

void test_rng_fill_nonzero(test_rng_t *g, uint8_t *buf, size_t n)
{
	size_t i;

	for (i = 0U; i < n; i++) {
		/* 1..255, uniform enough for a codec round-trip. */
		buf[i] = (uint8_t)(1U + (test_rng_u32(g) >> 24) % 255U);
	}
}

void test_fill_seq(uint8_t *buf, size_t n, uint8_t start)
{
	size_t i;

	for (i = 0U; i < n; i++) {
		buf[i] = (uint8_t)(start + i);
	}
}
