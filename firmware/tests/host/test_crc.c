/*
 * STS1000 "Meridian" — core/util/crc unit tests.
 *
 * Provenance of the expectations (ARCHITECTURE.md §9 asks for known-good
 * vectors, not self-round-trips):
 *
 *  - The two check values are the ones published in the CRC catalogue entries
 *    for CRC-32/ISO-HDLC and CRC-16/IBM-3740 (a.k.a. "CRC-16/CCITT-FALSE"):
 *    the CRC of the nine ASCII bytes "123456789" is 0xCBF43926 and 0x29B1.
 *
 *  - The remaining CRC-32 expectations were produced with zlib's crc32(), an
 *    implementation wholly independent of this one, and cross-checked against a
 *    textbook bit-at-a-time reference. The 32-byte all-zero / all-ones /
 *    ascending / descending vectors are the customary CRC regression set; they
 *    catch reflection and table-index mistakes that "123456789" alone does not.
 *
 *  - The CRC-16 expectations come from that same bit-at-a-time reference, which
 *    is the algorithm's definition rather than a second copy of the nibble
 *    table under test.
 *
 *  - 0x2144DF1C is the CRC-32 residue: running the CRC over a message with its
 *    own little-endian CRC appended always yields that constant. It is a
 *    structural property of the polynomial, verified here over the whole vector
 *    set, and it is exactly what the MCP frame check will rely on.
 */

#include <string.h>

#include "unity.h"
#include "util/crc.h"

#include "test_support.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* Longest buffer any vector needs. */
#define VEC_MAX 64U

/* How a vector's bytes are produced, so the table stays readable. */
typedef enum {
	VEC_LITERAL, /* bytes[] as given */
	VEC_FILL,    /* len copies of fill */
	VEC_SEQ,     /* len bytes ascending from fill */
	VEC_RSEQ,    /* len bytes descending from fill */
} vec_kind_t;

typedef struct {
	const char *name;
	vec_kind_t kind;
	const char *bytes; /* VEC_LITERAL only */
	uint8_t fill;
	size_t len;
	uint32_t crc32;
	uint16_t crc16;
} crc_vec_t;

static const crc_vec_t vectors[] = {
	{"catalogue check \"123456789\"", VEC_LITERAL, "123456789", 0, 9,
	 0xCBF43926U, 0x29B1U},
	{"empty", VEC_LITERAL, "", 0, 0, 0x00000000U, 0xFFFFU},
	{"\"a\"", VEC_LITERAL, "a", 0, 1, 0xE8B7BE43U, 0x9D77U},
	{"pangram", VEC_LITERAL,
	 "The quick brown fox jumps over the lazy dog", 0, 43,
	 0x414FA339U, 0x8FDDU},
	{"32 x 0x00", VEC_FILL, NULL, 0x00U, 32, 0x190A55ADU, 0xF14CU},
	{"32 x 0xFF", VEC_FILL, NULL, 0xFFU, 32, 0xFF6CAB0BU, 0x75F8U},
	{"0x00..0x1F", VEC_SEQ, NULL, 0x00U, 32, 0x91267E8AU, 0x23B3U},
	{"0x1F..0x00", VEC_RSEQ, NULL, 0x1FU, 32, 0x9AB0EF72U, 0x1086U},
	{"\"STS1000 Meridian\"", VEC_LITERAL, "STS1000 Meridian", 0, 16,
	 0xC27B9A37U, 0x560FU},
};

/* Materialise a vector into buf (VEC_MAX bytes); returns its length. */
static size_t vec_render(const crc_vec_t *v, uint8_t *buf)
{
	size_t i;

	TEST_ASSERT_LESS_OR_EQUAL_size_t(VEC_MAX, v->len);

	switch (v->kind) {
	case VEC_LITERAL:
		memcpy(buf, v->bytes, v->len);
		break;
	case VEC_FILL:
		memset(buf, v->fill, v->len);
		break;
	case VEC_SEQ:
		for (i = 0U; i < v->len; i++) {
			buf[i] = (uint8_t)(v->fill + i);
		}
		break;
	case VEC_RSEQ:
		for (i = 0U; i < v->len; i++) {
			buf[i] = (uint8_t)(v->fill - i);
		}
		break;
	default:
		TEST_FAIL_MESSAGE("bad vector kind");
		break;
	}

	return v->len;
}

/* --------------------------------------------------------------- CRC-32 -- */

static void test_crc32_catalogue_check_value(void)
{
	TEST_ASSERT_EQUAL_HEX32(0xCBF43926U, crc32_ieee("123456789", 9U));
}

static void test_crc32_known_answers(void)
{
	uint8_t buf[VEC_MAX];
	size_t i;

	for (i = 0U; i < ARRAY_LEN(vectors); i++) {
		size_t n = vec_render(&vectors[i], buf);

		TEST_ASSERT_EQUAL_HEX32_MESSAGE(vectors[i].crc32,
						crc32_ieee(buf, n),
						vectors[i].name);
	}
}

static void test_crc32_seed_is_the_identity(void)
{
	/* The zlib convention: seeding with the constant reproduces one-shot. */
	TEST_ASSERT_EQUAL_HEX32(0xCBF43926U,
				crc32_ieee_update(CRC32_IEEE_SEED, "123456789", 9U));
	TEST_ASSERT_EQUAL_HEX32(CRC32_IEEE_SEED,
				crc32_ieee_update(CRC32_IEEE_SEED, "", 0U));
}

static void test_crc32_null_is_no_data(void)
{
	/* Core must not trap: a NULL buffer is treated as contributing nothing,
	 * whatever length is claimed. */
	TEST_ASSERT_EQUAL_HEX32(0x00000000U, crc32_ieee(NULL, 0U));
	TEST_ASSERT_EQUAL_HEX32(0x00000000U, crc32_ieee(NULL, 16U));
	TEST_ASSERT_EQUAL_HEX32(0xCBF43926U,
				crc32_ieee_update(0xCBF43926U, NULL, 16U));
}

static void test_crc32_chaining_matches_one_shot(void)
{
	uint8_t buf[VEC_MAX];
	size_t i;

	/* Splitting the input at every offset must not change the result — this
	 * is what lets the MCP framer CRC a header and payload separately. */
	for (i = 0U; i < ARRAY_LEN(vectors); i++) {
		size_t n = vec_render(&vectors[i], buf);
		size_t split;

		for (split = 0U; split <= n; split++) {
			uint32_t c = crc32_ieee_update(CRC32_IEEE_SEED, buf, split);

			c = crc32_ieee_update(c, &buf[split], n - split);
			TEST_ASSERT_EQUAL_HEX32_MESSAGE(vectors[i].crc32, c,
							vectors[i].name);
		}
	}
}

static void test_crc32_byte_at_a_time_matches_one_shot(void)
{
	static const char msg[] = "123456789";
	uint32_t c = CRC32_IEEE_SEED;
	size_t i;

	for (i = 0U; i < 9U; i++) {
		c = crc32_ieee_update(c, &msg[i], 1U);
	}
	TEST_ASSERT_EQUAL_HEX32(0xCBF43926U, c);
}

static void test_crc32_residue_is_the_magic_constant(void)
{
	uint8_t buf[VEC_MAX + 4U];
	size_t i;

	for (i = 0U; i < ARRAY_LEN(vectors); i++) {
		size_t n = vec_render(&vectors[i], buf);
		uint32_t c = crc32_ieee(buf, n);

		/* Append the CRC little-endian, as MCP frames carry it. */
		buf[n + 0U] = (uint8_t)(c & 0xFFU);
		buf[n + 1U] = (uint8_t)((c >> 8) & 0xFFU);
		buf[n + 2U] = (uint8_t)((c >> 16) & 0xFFU);
		buf[n + 3U] = (uint8_t)((c >> 24) & 0xFFU);

		TEST_ASSERT_EQUAL_HEX32_MESSAGE(0x2144DF1CU,
						crc32_ieee(buf, n + 4U),
						vectors[i].name);
	}
}

static void test_crc32_detects_every_single_bit_flip(void)
{
	uint8_t buf[48];
	test_rng_t rng;
	uint32_t base;
	size_t bit;

	test_rng_init(&rng, 0xC0FFEEU);
	test_rng_fill(&rng, buf, sizeof(buf));
	base = crc32_ieee(buf, sizeof(buf));

	for (bit = 0U; bit < sizeof(buf) * 8U; bit++) {
		uint32_t flipped;

		buf[bit / 8U] ^= (uint8_t)(1U << (bit % 8U));
		flipped = crc32_ieee(buf, sizeof(buf));
		buf[bit / 8U] ^= (uint8_t)(1U << (bit % 8U));

		TEST_ASSERT_NOT_EQUAL_HEX32(base, flipped);
	}
}

/* --------------------------------------------------------------- CRC-16 -- */

static void test_crc16_catalogue_check_value(void)
{
	TEST_ASSERT_EQUAL_HEX16(0x29B1U, crc16_ccitt("123456789", 9U));
}

static void test_crc16_known_answers(void)
{
	uint8_t buf[VEC_MAX];
	size_t i;

	for (i = 0U; i < ARRAY_LEN(vectors); i++) {
		size_t n = vec_render(&vectors[i], buf);

		TEST_ASSERT_EQUAL_HEX16_MESSAGE(vectors[i].crc16,
						crc16_ccitt(buf, n),
						vectors[i].name);
	}
}

static void test_crc16_seed_is_the_init_value(void)
{
	/* CCITT-FALSE init is 0xFFFF and xorout is 0, so the empty message
	 * hashes to the init value itself. */
	TEST_ASSERT_EQUAL_HEX16(0xFFFFU, CRC16_CCITT_SEED);
	TEST_ASSERT_EQUAL_HEX16(0xFFFFU, crc16_ccitt("", 0U));
	TEST_ASSERT_EQUAL_HEX16(0x29B1U,
				crc16_ccitt_update(CRC16_CCITT_SEED, "123456789", 9U));
}

static void test_crc16_null_is_no_data(void)
{
	TEST_ASSERT_EQUAL_HEX16(0xFFFFU, crc16_ccitt(NULL, 0U));
	TEST_ASSERT_EQUAL_HEX16(0xFFFFU, crc16_ccitt(NULL, 16U));
	TEST_ASSERT_EQUAL_HEX16(0x29B1U, crc16_ccitt_update(0x29B1U, NULL, 16U));
}

static void test_crc16_chaining_matches_one_shot(void)
{
	uint8_t buf[VEC_MAX];
	size_t i;

	for (i = 0U; i < ARRAY_LEN(vectors); i++) {
		size_t n = vec_render(&vectors[i], buf);
		size_t split;

		for (split = 0U; split <= n; split++) {
			uint16_t c = crc16_ccitt_update(CRC16_CCITT_SEED, buf, split);

			c = crc16_ccitt_update(c, &buf[split], n - split);
			TEST_ASSERT_EQUAL_HEX16_MESSAGE(vectors[i].crc16, c,
							vectors[i].name);
		}
	}
}

static void test_crc16_byte_at_a_time_matches_one_shot(void)
{
	static const char msg[] = "123456789";
	uint16_t c = CRC16_CCITT_SEED;
	size_t i;

	for (i = 0U; i < 9U; i++) {
		c = crc16_ccitt_update(c, &msg[i], 1U);
	}
	TEST_ASSERT_EQUAL_HEX16(0x29B1U, c);
}

static void test_crc16_detects_every_single_bit_flip(void)
{
	/* Stay under the 16-bit birthday bound: a CRC-16 is guaranteed to catch
	 * a single bit flip only while the message is shorter than the code's
	 * Hamming distance limit, which 48 bytes comfortably is. */
	uint8_t buf[48];
	test_rng_t rng;
	uint16_t base;
	size_t bit;

	test_rng_init(&rng, 0x5EEDU);
	test_rng_fill(&rng, buf, sizeof(buf));
	base = crc16_ccitt(buf, sizeof(buf));

	for (bit = 0U; bit < sizeof(buf) * 8U; bit++) {
		uint16_t flipped;

		buf[bit / 8U] ^= (uint8_t)(1U << (bit % 8U));
		flipped = crc16_ccitt(buf, sizeof(buf));
		buf[bit / 8U] ^= (uint8_t)(1U << (bit % 8U));

		TEST_ASSERT_NOT_EQUAL_HEX16(base, flipped);
	}
}

static void test_crc16_differs_from_crc32_low_half(void)
{
	/* Guards against a copy-paste that wires both entry points to the same
	 * table: the two algorithms must not agree on a non-trivial message. */
	uint16_t c16 = crc16_ccitt("123456789", 9U);
	uint32_t c32 = crc32_ieee("123456789", 9U);

	TEST_ASSERT_NOT_EQUAL_HEX16(c16, (uint16_t)(c32 & 0xFFFFU));
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_crc32_catalogue_check_value);
	RUN_TEST(test_crc32_known_answers);
	RUN_TEST(test_crc32_seed_is_the_identity);
	RUN_TEST(test_crc32_null_is_no_data);
	RUN_TEST(test_crc32_chaining_matches_one_shot);
	RUN_TEST(test_crc32_byte_at_a_time_matches_one_shot);
	RUN_TEST(test_crc32_residue_is_the_magic_constant);
	RUN_TEST(test_crc32_detects_every_single_bit_flip);

	RUN_TEST(test_crc16_catalogue_check_value);
	RUN_TEST(test_crc16_known_answers);
	RUN_TEST(test_crc16_seed_is_the_init_value);
	RUN_TEST(test_crc16_null_is_no_data);
	RUN_TEST(test_crc16_chaining_matches_one_shot);
	RUN_TEST(test_crc16_byte_at_a_time_matches_one_shot);
	RUN_TEST(test_crc16_detects_every_single_bit_flip);
	RUN_TEST(test_crc16_differs_from_crc32_low_half);

	return UNITY_END();
}
