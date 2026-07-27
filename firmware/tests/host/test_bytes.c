/*
 * STS1000 "Meridian" — core/util/bytes unit tests.
 *
 * The expectations are literal byte layouts, written out rather than computed,
 * so a test failure names the wrong byte rather than repeating whatever the
 * implementation did. The canonical value 0x0123456789ABCDEF is used
 * throughout: every byte differs, so a swapped or duplicated position shows up
 * immediately, which a palindromic pattern would hide.
 *
 * Two properties get particular attention:
 *
 *   Unaligned access. Every accessor is exercised at all of offsets 0..7 inside
 *   a buffer. On the target these land in packet buffers at arbitrary offsets —
 *   a UBX payload field, an NTP extension field — and an implementation that
 *   casts to uint32_t* would fault on some cores and silently mis-read on
 *   others. The byte-wise implementation must be indifferent.
 *
 *   Host independence. Nothing here may depend on the host's byte order; the
 *   little-endian and big-endian accessors must both produce their documented
 *   layout on any machine. The reversal tests below assert exactly that.
 */

#include <string.h>

#include "unity.h"
#include "util/bytes.h"

#include "test_support.h"

/* Canonical value and its two wire layouts. */
#define V16 ((uint16_t)0x0123U)
#define V32 ((uint32_t)0x01234567UL)
#define V64 ((uint64_t)0x0123456789ABCDEFULL)

static const uint8_t le16_bytes[2] = {0x23, 0x01};
static const uint8_t be16_bytes[2] = {0x01, 0x23};
static const uint8_t le32_bytes[4] = {0x67, 0x45, 0x23, 0x01};
static const uint8_t be32_bytes[4] = {0x01, 0x23, 0x45, 0x67};
static const uint8_t le64_bytes[8] = {0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01};
static const uint8_t be64_bytes[8] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};

/* A get and a put must be tested at every offset, so both share this scratch
 * buffer with guard bytes either side of the window under test. */
#define GUARD 0xA5U
#define SCRATCH 32U

static uint8_t scratch[SCRATCH];

static void scratch_reset(void)
{
	memset(scratch, GUARD, sizeof(scratch));
}

/* Assert that nothing outside [off, off+width) was disturbed. */
static void assert_guards_intact(size_t off, size_t width)
{
	size_t i;

	for (i = 0U; i < SCRATCH; i++) {
		if ((i >= off) && (i < (off + width))) {
			continue;
		}
		TEST_ASSERT_EQUAL_HEX8_MESSAGE(GUARD, scratch[i],
					       "write ran outside its width");
	}
}

/* ------------------------------------------------------------------- get - */

static void test_bytes_get_known_layouts(void)
{
	TEST_ASSERT_EQUAL_HEX16(V16, bytes_get_le16(le16_bytes));
	TEST_ASSERT_EQUAL_HEX16(V16, bytes_get_be16(be16_bytes));
	TEST_ASSERT_EQUAL_HEX32(V32, bytes_get_le32(le32_bytes));
	TEST_ASSERT_EQUAL_HEX32(V32, bytes_get_be32(be32_bytes));
	TEST_ASSERT_EQUAL_HEX64(V64, bytes_get_le64(le64_bytes));
	TEST_ASSERT_EQUAL_HEX64(V64, bytes_get_be64(be64_bytes));
}

static void test_bytes_get_is_endian_explicit_not_host_relative(void)
{
	/* Reading the same bytes both ways must give byte-reversed results,
	 * whatever the host's own order is. */
	TEST_ASSERT_EQUAL_HEX16(0x2301U, bytes_get_le16(be16_bytes));
	TEST_ASSERT_EQUAL_HEX16(0x2301U, bytes_get_be16(le16_bytes));
	TEST_ASSERT_EQUAL_HEX32(0x67452301UL, bytes_get_le32(be32_bytes));
	TEST_ASSERT_EQUAL_HEX32(0x67452301UL, bytes_get_be32(le32_bytes));
	TEST_ASSERT_EQUAL_HEX64(0xEFCDAB8967452301ULL, bytes_get_le64(be64_bytes));
	TEST_ASSERT_EQUAL_HEX64(0xEFCDAB8967452301ULL, bytes_get_be64(le64_bytes));
}

static void test_bytes_get_at_every_offset(void)
{
	size_t off;

	for (off = 0U; off < 8U; off++) {
		scratch_reset();

		memcpy(&scratch[off], le16_bytes, 2U);
		TEST_ASSERT_EQUAL_HEX16(V16, bytes_get_le16(&scratch[off]));
		memcpy(&scratch[off], be16_bytes, 2U);
		TEST_ASSERT_EQUAL_HEX16(V16, bytes_get_be16(&scratch[off]));

		memcpy(&scratch[off], le32_bytes, 4U);
		TEST_ASSERT_EQUAL_HEX32(V32, bytes_get_le32(&scratch[off]));
		memcpy(&scratch[off], be32_bytes, 4U);
		TEST_ASSERT_EQUAL_HEX32(V32, bytes_get_be32(&scratch[off]));

		memcpy(&scratch[off], le64_bytes, 8U);
		TEST_ASSERT_EQUAL_HEX64(V64, bytes_get_le64(&scratch[off]));
		memcpy(&scratch[off], be64_bytes, 8U);
		TEST_ASSERT_EQUAL_HEX64(V64, bytes_get_be64(&scratch[off]));
	}
}

/* ------------------------------------------------------------------- put - */

static void test_bytes_put_known_layouts(void)
{
	scratch_reset();
	bytes_put_le16(scratch, V16);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(le16_bytes, scratch, 2U);
	assert_guards_intact(0U, 2U);

	scratch_reset();
	bytes_put_be16(scratch, V16);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(be16_bytes, scratch, 2U);
	assert_guards_intact(0U, 2U);

	scratch_reset();
	bytes_put_le32(scratch, V32);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(le32_bytes, scratch, 4U);
	assert_guards_intact(0U, 4U);

	scratch_reset();
	bytes_put_be32(scratch, V32);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(be32_bytes, scratch, 4U);
	assert_guards_intact(0U, 4U);

	scratch_reset();
	bytes_put_le64(scratch, V64);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(le64_bytes, scratch, 8U);
	assert_guards_intact(0U, 8U);

	scratch_reset();
	bytes_put_be64(scratch, V64);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(be64_bytes, scratch, 8U);
	assert_guards_intact(0U, 8U);
}

static void test_bytes_put_at_every_offset(void)
{
	size_t off;

	for (off = 0U; off < 8U; off++) {
		scratch_reset();
		bytes_put_le16(&scratch[off], V16);
		TEST_ASSERT_EQUAL_HEX8_ARRAY(le16_bytes, &scratch[off], 2U);
		assert_guards_intact(off, 2U);

		scratch_reset();
		bytes_put_be16(&scratch[off], V16);
		TEST_ASSERT_EQUAL_HEX8_ARRAY(be16_bytes, &scratch[off], 2U);
		assert_guards_intact(off, 2U);

		scratch_reset();
		bytes_put_le32(&scratch[off], V32);
		TEST_ASSERT_EQUAL_HEX8_ARRAY(le32_bytes, &scratch[off], 4U);
		assert_guards_intact(off, 4U);

		scratch_reset();
		bytes_put_be32(&scratch[off], V32);
		TEST_ASSERT_EQUAL_HEX8_ARRAY(be32_bytes, &scratch[off], 4U);
		assert_guards_intact(off, 4U);

		scratch_reset();
		bytes_put_le64(&scratch[off], V64);
		TEST_ASSERT_EQUAL_HEX8_ARRAY(le64_bytes, &scratch[off], 8U);
		assert_guards_intact(off, 8U);

		scratch_reset();
		bytes_put_be64(&scratch[off], V64);
		TEST_ASSERT_EQUAL_HEX8_ARRAY(be64_bytes, &scratch[off], 8U);
		assert_guards_intact(off, 8U);
	}
}

/* ------------------------------------------------------------ round trip - */

static void test_bytes_round_trip_boundary_values(void)
{
	static const uint64_t vals[] = {
		0x0000000000000000ULL, 0x0000000000000001ULL,
		0x00000000000000FFULL, 0x000000000000FF00ULL,
		0x0000000080000000ULL, 0x00000000FFFFFFFFULL,
		0x0123456789ABCDEFULL, 0x8000000000000000ULL,
		0xFEDCBA9876543210ULL, 0xFFFFFFFFFFFFFFFFULL,
	};
	size_t i;
	size_t off;

	for (i = 0U; i < (sizeof(vals) / sizeof(vals[0])); i++) {
		uint64_t v64 = vals[i];
		uint32_t v32 = (uint32_t)v64;
		uint16_t v16 = (uint16_t)v64;

		for (off = 0U; off < 8U; off++) {
			scratch_reset();
			bytes_put_le16(&scratch[off], v16);
			TEST_ASSERT_EQUAL_HEX16(v16, bytes_get_le16(&scratch[off]));

			scratch_reset();
			bytes_put_be16(&scratch[off], v16);
			TEST_ASSERT_EQUAL_HEX16(v16, bytes_get_be16(&scratch[off]));

			scratch_reset();
			bytes_put_le32(&scratch[off], v32);
			TEST_ASSERT_EQUAL_HEX32(v32, bytes_get_le32(&scratch[off]));

			scratch_reset();
			bytes_put_be32(&scratch[off], v32);
			TEST_ASSERT_EQUAL_HEX32(v32, bytes_get_be32(&scratch[off]));

			scratch_reset();
			bytes_put_le64(&scratch[off], v64);
			TEST_ASSERT_EQUAL_HEX64(v64, bytes_get_le64(&scratch[off]));

			scratch_reset();
			bytes_put_be64(&scratch[off], v64);
			TEST_ASSERT_EQUAL_HEX64(v64, bytes_get_be64(&scratch[off]));
		}
	}
}

static void test_bytes_round_trip_random_values(void)
{
	test_rng_t rng;
	size_t iter;

	test_rng_init(&rng, 0xBEE5U);
	for (iter = 0U; iter < 4096U; iter++) {
		uint64_t v64 = ((uint64_t)test_rng_u32(&rng) << 32) |
			       (uint64_t)test_rng_u32(&rng);
		size_t off = (size_t)(test_rng_u32(&rng) % 8U);

		scratch_reset();
		bytes_put_le64(&scratch[off], v64);
		TEST_ASSERT_EQUAL_HEX64(v64, bytes_get_le64(&scratch[off]));
		assert_guards_intact(off, 8U);

		scratch_reset();
		bytes_put_be64(&scratch[off], v64);
		TEST_ASSERT_EQUAL_HEX64(v64, bytes_get_be64(&scratch[off]));
		assert_guards_intact(off, 8U);

		scratch_reset();
		bytes_put_le32(&scratch[off], (uint32_t)v64);
		TEST_ASSERT_EQUAL_HEX32((uint32_t)v64, bytes_get_le32(&scratch[off]));

		scratch_reset();
		bytes_put_be32(&scratch[off], (uint32_t)v64);
		TEST_ASSERT_EQUAL_HEX32((uint32_t)v64, bytes_get_be32(&scratch[off]));

		scratch_reset();
		bytes_put_le16(&scratch[off], (uint16_t)v64);
		TEST_ASSERT_EQUAL_HEX16((uint16_t)v64, bytes_get_le16(&scratch[off]));

		scratch_reset();
		bytes_put_be16(&scratch[off], (uint16_t)v64);
		TEST_ASSERT_EQUAL_HEX16((uint16_t)v64, bytes_get_be16(&scratch[off]));
	}
}

/* ----------------------------------------------------------- composition - */

static void test_bytes_64_is_consistent_with_two_32s(void)
{
	/* The 64-bit accessors are built from the 32-bit ones; assert the halves
	 * land where the wire format says they do rather than trusting that. */
	scratch_reset();
	bytes_put_le64(scratch, V64);
	TEST_ASSERT_EQUAL_HEX32(0x89ABCDEFUL, bytes_get_le32(&scratch[0]));
	TEST_ASSERT_EQUAL_HEX32(0x01234567UL, bytes_get_le32(&scratch[4]));

	scratch_reset();
	bytes_put_be64(scratch, V64);
	TEST_ASSERT_EQUAL_HEX32(0x01234567UL, bytes_get_be32(&scratch[0]));
	TEST_ASSERT_EQUAL_HEX32(0x89ABCDEFUL, bytes_get_be32(&scratch[4]));
}

static void test_bytes_matches_a_real_protocol_header(void)
{
	/*
	 * A concrete cross-check against the two wire formats this firmware
	 * actually speaks.
	 *
	 * MCP (ARCHITECTURE.md §7) is little-endian: ver=1, type=0 (REQ),
	 * cmd=0x11 (CFG_GET), flags=0, seq=0x0102, len=0x0004.
	 *
	 * The NTP short-format root delay (RFC 5905 §7.3) is big-endian
	 * 16.16 fixed point; 0x00010000 is exactly 1 second.
	 */
	static const uint8_t mcp_header[8] = {
		0x01, 0x00, 0x11, 0x00, 0x02, 0x01, 0x04, 0x00,
	};
	static const uint8_t ntp_root_delay[4] = {0x00, 0x01, 0x00, 0x00};

	TEST_ASSERT_EQUAL_HEX16(0x0102U, bytes_get_le16(&mcp_header[4]));
	TEST_ASSERT_EQUAL_HEX16(0x0004U, bytes_get_le16(&mcp_header[6]));
	TEST_ASSERT_EQUAL_HEX32(0x00010000UL, bytes_get_be32(ntp_root_delay));

	scratch_reset();
	bytes_put_le16(&scratch[4], 0x0102U);
	bytes_put_le16(&scratch[6], 0x0004U);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(&mcp_header[4], &scratch[4], 4U);
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_bytes_get_known_layouts);
	RUN_TEST(test_bytes_get_is_endian_explicit_not_host_relative);
	RUN_TEST(test_bytes_get_at_every_offset);
	RUN_TEST(test_bytes_put_known_layouts);
	RUN_TEST(test_bytes_put_at_every_offset);
	RUN_TEST(test_bytes_round_trip_boundary_values);
	RUN_TEST(test_bytes_round_trip_random_values);
	RUN_TEST(test_bytes_64_is_consistent_with_two_32s);
	RUN_TEST(test_bytes_matches_a_real_protocol_header);

	return UNITY_END();
}
