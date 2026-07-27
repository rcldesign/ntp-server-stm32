/*
 * STS1000 "Meridian" — shared helpers for the host unit tests.
 *
 * Everything in support/ is linked into every test binary (see
 * tests/host/CMakeLists.txt), so keep it small and dependency-free.
 *
 * Unity's setUp()/tearDown() are provided here as weak no-ops. A test file that
 * needs per-test fixture work simply defines its own strong setUp()/tearDown();
 * the strong definition wins at link time. Most suites need neither.
 */

#ifndef STS1000_TEST_SUPPORT_H_
#define STS1000_TEST_SUPPORT_H_

#include <stddef.h>
#include <stdint.h>

/**
 * Deterministic xorshift32 PRNG.
 *
 * Tests must be reproducible, so no time- or address-derived seeding: every
 * generator is explicitly seeded and a failing case can be replayed from the
 * seed printed in the assertion message.
 */
typedef struct {
	uint32_t s;
} test_rng_t;

/** Seed a generator. @p seed of 0 is remapped (xorshift has a zero fixpoint). */
void test_rng_init(test_rng_t *g, uint32_t seed);

/** Next pseudo-random 32-bit value. */
uint32_t test_rng_u32(test_rng_t *g);

/** Next pseudo-random value in [0, bound); returns 0 when @p bound is 0. */
size_t test_rng_below(test_rng_t *g, size_t bound);

/** Fill @p n bytes with pseudo-random data (zeros included). */
void test_rng_fill(test_rng_t *g, uint8_t *buf, size_t n);

/** Fill @p n bytes with pseudo-random data, never emitting 0x00. */
void test_rng_fill_nonzero(test_rng_t *g, uint8_t *buf, size_t n);

/**
 * Fill @p n bytes with the ascending run @p start, @p start+1, ... wrapping at
 * 256. Used to build the byte runs from the COBS paper's example table.
 */
void test_fill_seq(uint8_t *buf, size_t n, uint8_t start);

#endif /* STS1000_TEST_SUPPORT_H_ */
