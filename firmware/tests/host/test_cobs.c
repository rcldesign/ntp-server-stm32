/*
 * STS1000 "Meridian" — core/util/cobs unit tests.
 *
 * The primary vectors are the nine worked examples in Table 1 of S. Cheshire
 * and M. Baker, "Consistent Overhead Byte Stuffing", IEEE/ACM Transactions on
 * Networking 7(2), April 1999 — the specification's own examples, reproduced
 * byte for byte. Rows 6 to 9 are the ones that matter most: they sit on the
 * 254-byte group boundary, which is where an encoder either handles the
 * "no implied zero after a full group" rule or silently emits a spurious
 * trailing 0x01 that a conforming decoder turns into an extra zero byte.
 *
 *   #  input                          encoded output
 *   1  00                             01 01
 *   2  00 00                          01 01 01
 *   3  11 22 00 33                    03 11 22 02 33
 *   4  11 22 33 44                    05 11 22 33 44
 *   5  11 00 00 00                    02 11 01 01 01
 *   6  01 02 .. FE          (254 B)   FF 01 02 .. FE            (255 B)
 *   7  00 01 02 .. FE       (255 B)   01 FF 01 02 .. FE         (256 B)
 *   8  01 02 .. FE FF       (255 B)   FF 01 02 .. FE 02 FF      (257 B)
 *   9  02 03 .. FF 00       (255 B)   FF 02 03 .. FF 01 01      (257 B)
 *
 * Round-trip and malformed-input tests sit on top of those; a round-trip alone
 * would only prove the encoder and decoder agree with each other.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"
#include "util/cobs.h"

#include "test_support.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* Room for the largest frame the codec accepts, plus slack for oversize tests. */
#define BUF_MAX (COBS_MAX_FRAME_ENCODED + 8U)

static uint8_t g_dec[BUF_MAX];
static uint8_t g_enc[BUF_MAX];
static uint8_t g_out[BUF_MAX];
static uint8_t g_tmp[BUF_MAX];

/* ------------------------------------------------- Cheshire/Baker Table 1 - */

typedef size_t (*cobs_build_fn)(uint8_t *buf);

typedef struct {
	const char *name;
	cobs_build_fn dec; /* the unencoded frame body */
	cobs_build_fn enc; /* the expected COBS encoding, delimiter excluded */
} cobs_vec_t;

static size_t lit(uint8_t *buf, const uint8_t *src, size_t n)
{
	memcpy(buf, src, n);
	return n;
}

/* Row 1: 00 -> 01 01 */
static size_t r1_dec(uint8_t *b)
{
	static const uint8_t v[] = {0x00};

	return lit(b, v, sizeof(v));
}
static size_t r1_enc(uint8_t *b)
{
	static const uint8_t v[] = {0x01, 0x01};

	return lit(b, v, sizeof(v));
}

/* Row 2: 00 00 -> 01 01 01 */
static size_t r2_dec(uint8_t *b)
{
	static const uint8_t v[] = {0x00, 0x00};

	return lit(b, v, sizeof(v));
}
static size_t r2_enc(uint8_t *b)
{
	static const uint8_t v[] = {0x01, 0x01, 0x01};

	return lit(b, v, sizeof(v));
}

/* Row 3: 11 22 00 33 -> 03 11 22 02 33 */
static size_t r3_dec(uint8_t *b)
{
	static const uint8_t v[] = {0x11, 0x22, 0x00, 0x33};

	return lit(b, v, sizeof(v));
}
static size_t r3_enc(uint8_t *b)
{
	static const uint8_t v[] = {0x03, 0x11, 0x22, 0x02, 0x33};

	return lit(b, v, sizeof(v));
}

/* Row 4: 11 22 33 44 -> 05 11 22 33 44 */
static size_t r4_dec(uint8_t *b)
{
	static const uint8_t v[] = {0x11, 0x22, 0x33, 0x44};

	return lit(b, v, sizeof(v));
}
static size_t r4_enc(uint8_t *b)
{
	static const uint8_t v[] = {0x05, 0x11, 0x22, 0x33, 0x44};

	return lit(b, v, sizeof(v));
}

/* Row 5: 11 00 00 00 -> 02 11 01 01 01 */
static size_t r5_dec(uint8_t *b)
{
	static const uint8_t v[] = {0x11, 0x00, 0x00, 0x00};

	return lit(b, v, sizeof(v));
}
static size_t r5_enc(uint8_t *b)
{
	static const uint8_t v[] = {0x02, 0x11, 0x01, 0x01, 0x01};

	return lit(b, v, sizeof(v));
}

/* Row 6: 01..FE (254) -> FF 01..FE (255). The exact-group boundary. */
static size_t r6_dec(uint8_t *b)
{
	test_fill_seq(b, 254U, 0x01U);
	return 254U;
}
static size_t r6_enc(uint8_t *b)
{
	b[0] = 0xFFU;
	test_fill_seq(&b[1], 254U, 0x01U);
	return 255U;
}

/* Row 7: 00 01..FE (255) -> 01 FF 01..FE (256) */
static size_t r7_dec(uint8_t *b)
{
	b[0] = 0x00U;
	test_fill_seq(&b[1], 254U, 0x01U);
	return 255U;
}
static size_t r7_enc(uint8_t *b)
{
	b[0] = 0x01U;
	b[1] = 0xFFU;
	test_fill_seq(&b[2], 254U, 0x01U);
	return 256U;
}

/* Row 8: 01..FF (255) -> FF 01..FE 02 FF (257) */
static size_t r8_dec(uint8_t *b)
{
	test_fill_seq(b, 255U, 0x01U);
	return 255U;
}
static size_t r8_enc(uint8_t *b)
{
	b[0] = 0xFFU;
	test_fill_seq(&b[1], 254U, 0x01U);
	b[255] = 0x02U;
	b[256] = 0xFFU;
	return 257U;
}

/* Row 9: 02..FF 00 (255) -> FF 02..FF 01 01 (257) */
static size_t r9_dec(uint8_t *b)
{
	test_fill_seq(b, 254U, 0x02U);
	b[254] = 0x00U;
	return 255U;
}
static size_t r9_enc(uint8_t *b)
{
	b[0] = 0xFFU;
	test_fill_seq(&b[1], 254U, 0x02U);
	b[255] = 0x01U;
	b[256] = 0x01U;
	return 257U;
}

static const cobs_vec_t paper[] = {
	{"table1 #1 00", r1_dec, r1_enc},
	{"table1 #2 00 00", r2_dec, r2_enc},
	{"table1 #3 11 22 00 33", r3_dec, r3_enc},
	{"table1 #4 11 22 33 44", r4_dec, r4_enc},
	{"table1 #5 11 00 00 00", r5_dec, r5_enc},
	{"table1 #6 254-byte non-zero run", r6_dec, r6_enc},
	{"table1 #7 zero + 254-byte run", r7_dec, r7_enc},
	{"table1 #8 255-byte non-zero run", r8_dec, r8_enc},
	{"table1 #9 254-byte run + zero", r9_dec, r9_enc},
};

/* ------------------------------------------------------------- block API - */

static void test_cobs_encode_matches_paper_table(void)
{
	size_t i;

	for (i = 0U; i < ARRAY_LEN(paper); i++) {
		size_t dec_len = paper[i].dec(g_dec);
		size_t exp_len = paper[i].enc(g_enc);
		size_t got_len = 0U;

		TEST_ASSERT_EQUAL_INT_MESSAGE(
			0, cobs_encode(g_dec, dec_len, g_out, sizeof(g_out), &got_len),
			paper[i].name);
		TEST_ASSERT_EQUAL_size_t_MESSAGE(exp_len, got_len, paper[i].name);
		TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(g_enc, g_out, exp_len,
						     paper[i].name);
	}
}

static void test_cobs_decode_matches_paper_table(void)
{
	size_t i;

	for (i = 0U; i < ARRAY_LEN(paper); i++) {
		size_t exp_len = paper[i].dec(g_dec);
		size_t enc_len = paper[i].enc(g_enc);
		size_t got_len = 0U;

		TEST_ASSERT_EQUAL_INT_MESSAGE(
			0, cobs_decode(g_enc, enc_len, g_out, sizeof(g_out), &got_len),
			paper[i].name);
		TEST_ASSERT_EQUAL_size_t_MESSAGE(exp_len, got_len, paper[i].name);
		TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(g_dec, g_out, exp_len,
						     paper[i].name);
	}
}

static void test_cobs_encode_of_empty_input_is_a_single_01(void)
{
	size_t n = 0xDEADU;

	TEST_ASSERT_EQUAL_INT(0, cobs_encode(NULL, 0U, g_out, sizeof(g_out), &n));
	TEST_ASSERT_EQUAL_size_t(1U, n);
	TEST_ASSERT_EQUAL_HEX8(0x01U, g_out[0]);
}

static void test_cobs_decode_of_a_single_01_is_empty(void)
{
	static const uint8_t enc[] = {0x01};
	size_t n = 0xDEADU;

	TEST_ASSERT_EQUAL_INT(0, cobs_decode(enc, sizeof(enc), g_out,
					     sizeof(g_out), &n));
	TEST_ASSERT_EQUAL_size_t(0U, n);

	/* A NULL/zero-capacity sink is legal when nothing decodes: this is the
	 * validate-only mode a framer uses to sanity-check before committing. */
	n = 0xDEADU;
	TEST_ASSERT_EQUAL_INT(0, cobs_decode(enc, sizeof(enc), NULL, 0U, &n));
	TEST_ASSERT_EQUAL_size_t(0U, n);
}

static void test_cobs_254_run_encodes_without_a_trailing_group(void)
{
	size_t n = 0U;

	/* The whole point of row 6: a full group carries no implied zero, so no
	 * 0x01 is appended and the encoding is exactly 1 + 254 bytes. Getting
	 * this wrong yields 256 bytes and a decoder that emits a phantom zero. */
	test_fill_seq(g_dec, 254U, 0x01U);
	TEST_ASSERT_EQUAL_INT(0, cobs_encode(g_dec, 254U, g_out, sizeof(g_out), &n));
	TEST_ASSERT_EQUAL_size_t(255U, n);
	TEST_ASSERT_EQUAL_HEX8(0xFFU, g_out[0]);
	TEST_ASSERT_EQUAL_HEX8(0xFEU, g_out[254]);

	/* One more byte and a second group must appear. */
	test_fill_seq(g_dec, 255U, 0x01U);
	TEST_ASSERT_EQUAL_INT(0, cobs_encode(g_dec, 255U, g_out, sizeof(g_out), &n));
	TEST_ASSERT_EQUAL_size_t(257U, n);
	TEST_ASSERT_EQUAL_HEX8(0xFFU, g_out[0]);
	TEST_ASSERT_EQUAL_HEX8(0x02U, g_out[255]);
	TEST_ASSERT_EQUAL_HEX8(0xFFU, g_out[256]);
}

static void test_cobs_encode_max_is_a_true_upper_bound(void)
{
	/* {input length, n + n/254 + 1}. 254 and 508 are the run boundaries. */
	static const size_t known[][2] = {
		{0U, 1U},     {1U, 2U},     {253U, 254U}, {254U, 256U},
		{255U, 257U}, {507U, 509U}, {508U, 511U}, {509U, 512U},
	};
	test_rng_t rng;
	size_t i;
	size_t len;

	for (i = 0U; i < ARRAY_LEN(known); i++) {
		TEST_ASSERT_EQUAL_size_t(known[i][1], cobs_encode_max(known[i][0]));
		TEST_ASSERT_EQUAL_size_t(known[i][1], COBS_ENCODE_MAX(known[i][0]));
	}

	/* The bound must hold for every length and every data profile, not just
	 * on paper: all-zero maximises group count, all-non-zero maximises run
	 * length, and the boundary lengths straddle 254. */
	test_rng_init(&rng, 0xB0BAU);
	for (len = 0U; len <= 600U; len++) {
		int profile;

		for (profile = 0; profile < 3; profile++) {
			size_t n = 0U;

			if (profile == 0) {
				memset(g_dec, 0, len);
			} else if (profile == 1) {
				test_rng_fill_nonzero(&rng, g_dec, len);
			} else {
				test_rng_fill(&rng, g_dec, len);
			}

			TEST_ASSERT_EQUAL_INT(0, cobs_encode(g_dec, len, g_out,
							     sizeof(g_out), &n));
			TEST_ASSERT_LESS_OR_EQUAL_size_t(cobs_encode_max(len), n);
			TEST_ASSERT_GREATER_OR_EQUAL_size_t(1U, n);
		}
	}
}

static void test_cobs_encoded_output_never_contains_a_zero(void)
{
	test_rng_t rng;
	size_t len;

	/* The delimiter must be unambiguous — this is the entire premise of the
	 * framing, so it is checked over the same sweep rather than spot-tested. */
	test_rng_init(&rng, 0x1234U);
	for (len = 0U; len <= 600U; len++) {
		size_t n = 0U;
		size_t i;

		test_rng_fill(&rng, g_dec, len);
		TEST_ASSERT_EQUAL_INT(0, cobs_encode(g_dec, len, g_out,
						     sizeof(g_out), &n));
		for (i = 0U; i < n; i++) {
			TEST_ASSERT_NOT_EQUAL_HEX8(0x00U, g_out[i]);
		}
	}
}

static void test_cobs_round_trip_over_length_and_data_sweep(void)
{
	test_rng_t rng;
	size_t len;

	test_rng_init(&rng, 0xACE1U);
	for (len = 0U; len <= 600U; len++) {
		int profile;

		for (profile = 0; profile < 4; profile++) {
			size_t enc_len = 0U;
			size_t dec_len = 0U;

			switch (profile) {
			case 0:
				memset(g_dec, 0, len);
				break;
			case 1:
				memset(g_dec, 0xA5, len);
				break;
			case 2:
				test_rng_fill_nonzero(&rng, g_dec, len);
				break;
			default:
				test_rng_fill(&rng, g_dec, len);
				break;
			}

			TEST_ASSERT_EQUAL_INT(0, cobs_encode(g_dec, len, g_enc,
							     sizeof(g_enc), &enc_len));
			TEST_ASSERT_EQUAL_INT(0, cobs_decode(g_enc, enc_len, g_out,
							     sizeof(g_out), &dec_len));
			TEST_ASSERT_EQUAL_size_t(len, dec_len);
			if (len != 0U) {
				TEST_ASSERT_EQUAL_HEX8_ARRAY(g_dec, g_out, len);
			}
		}
	}
}

static void test_cobs_round_trip_at_the_max_frame_size(void)
{
	test_rng_t rng;
	size_t enc_len = 0U;
	size_t dec_len = 0U;

	test_rng_init(&rng, 0x5A5AU);
	test_rng_fill(&rng, g_dec, COBS_MAX_FRAME);

	TEST_ASSERT_EQUAL_INT(0, cobs_encode(g_dec, COBS_MAX_FRAME, g_enc,
					     sizeof(g_enc), &enc_len));
	TEST_ASSERT_LESS_OR_EQUAL_size_t(COBS_MAX_FRAME_ENCODED, enc_len);
	TEST_ASSERT_EQUAL_INT(0, cobs_decode(g_enc, enc_len, g_out,
					     sizeof(g_out), &dec_len));
	TEST_ASSERT_EQUAL_size_t(COBS_MAX_FRAME, dec_len);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(g_dec, g_out, COBS_MAX_FRAME);
}

static void test_cobs_decode_max_bound(void)
{
	TEST_ASSERT_EQUAL_size_t(0U, cobs_decode_max(0U));
	TEST_ASSERT_EQUAL_size_t(0U, cobs_decode_max(1U));
	TEST_ASSERT_EQUAL_size_t(254U, cobs_decode_max(255U));
	TEST_ASSERT_EQUAL_size_t(COBS_MAX_FRAME_ENCODED - 1U,
				 cobs_decode_max(COBS_MAX_FRAME_ENCODED));
}

/* ------------------------------------------------------ argument checking - */

static void test_cobs_encode_rejects_bad_arguments(void)
{
	size_t n = 0U;

	TEST_ASSERT_EQUAL_INT(-EINVAL, cobs_encode(g_dec, 4U, NULL, 16U, &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cobs_encode(g_dec, 4U, g_out, 16U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cobs_encode(NULL, 4U, g_out, 16U, &n));
}

static void test_cobs_encode_rejects_oversize_input(void)
{
	size_t n = 0U;

	memset(g_dec, 0x42, COBS_MAX_FRAME + 1U);
	TEST_ASSERT_EQUAL_INT(0, cobs_encode(g_dec, COBS_MAX_FRAME, g_out,
					     sizeof(g_out), &n));
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE, cobs_encode(g_dec, COBS_MAX_FRAME + 1U,
						     g_out, sizeof(g_out), &n));
}

static void test_cobs_encode_reports_enospc(void)
{
	static const uint8_t nonzero[] = {0x11, 0x22, 0x33};
	static const uint8_t zeros[] = {0x00};
	size_t n = 0U;

	/* No room even for the first code byte. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC, cobs_encode(nonzero, 3U, g_out, 0U, &n));

	/* Room for the code byte but not the first data byte. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC, cobs_encode(nonzero, 3U, g_out, 1U, &n));

	/* Room for the data but not for the group that the zero forces open. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC, cobs_encode(zeros, 1U, g_out, 1U, &n));

	/* Exactly one byte short of the true encoded length, and exactly enough. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC, cobs_encode(nonzero, 3U, g_out, 3U, &n));
	TEST_ASSERT_EQUAL_INT(0, cobs_encode(nonzero, 3U, g_out, 4U, &n));
	TEST_ASSERT_EQUAL_size_t(4U, n);

	/* And on the 254-byte boundary, where the last group is not reserved. */
	test_fill_seq(g_dec, 254U, 0x01U);
	TEST_ASSERT_EQUAL_INT(-ENOSPC, cobs_encode(g_dec, 254U, g_out, 254U, &n));
	TEST_ASSERT_EQUAL_INT(0, cobs_encode(g_dec, 254U, g_out, 255U, &n));
	TEST_ASSERT_EQUAL_size_t(255U, n);
}

static void test_cobs_decode_rejects_bad_arguments(void)
{
	static const uint8_t enc[] = {0x02, 0x11};
	size_t n = 0U;

	TEST_ASSERT_EQUAL_INT(-EINVAL, cobs_decode(NULL, 2U, g_out, 16U, &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cobs_decode(enc, 2U, g_out, 16U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cobs_decode(enc, 2U, NULL, 16U, &n));
}

static void test_cobs_decode_rejects_malformed_input(void)
{
	/* Passed with a length of 0: a body must carry at least a code byte. */
	static const uint8_t well_formed[] = {0x01};
	/* A 0x00 where a code byte belongs — the delimiter must not appear. */
	static const uint8_t zero_code[] = {0x00};
	static const uint8_t zero_code_mid[] = {0x02, 0x11, 0x00, 0x11};
	/* A 0x00 inside a group's data. */
	static const uint8_t zero_in_body[] = {0x03, 0x11, 0x00};
	/* Codes claiming more data than the input holds. */
	static const uint8_t truncated[] = {0x05, 0x11};
	static const uint8_t truncated_tail[] = {0x02, 0x11, 0x04, 0x22};
	static const uint8_t truncated_ff[] = {0xFF, 0x11, 0x22};
	size_t n = 0U;

	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      cobs_decode(well_formed, 0U, g_out, sizeof(g_out), &n));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      cobs_decode(zero_code, sizeof(zero_code), g_out,
					  sizeof(g_out), &n));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      cobs_decode(zero_code_mid, sizeof(zero_code_mid), g_out,
					  sizeof(g_out), &n));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      cobs_decode(zero_in_body, sizeof(zero_in_body), g_out,
					  sizeof(g_out), &n));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      cobs_decode(truncated, sizeof(truncated), g_out,
					  sizeof(g_out), &n));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      cobs_decode(truncated_tail, sizeof(truncated_tail), g_out,
					  sizeof(g_out), &n));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
			      cobs_decode(truncated_ff, sizeof(truncated_ff), g_out,
					  sizeof(g_out), &n));

	/* n must be untouched by a failure. */
	TEST_ASSERT_EQUAL_size_t(0U, n);
}

static void test_cobs_decode_rejects_oversize_input(void)
{
	size_t n = 0U;

	memset(g_enc, 0x01, sizeof(g_enc));
	TEST_ASSERT_EQUAL_INT(0, cobs_decode(g_enc, COBS_MAX_FRAME_ENCODED, g_out,
					     sizeof(g_out), &n));
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE,
			      cobs_decode(g_enc, COBS_MAX_FRAME_ENCODED + 1U, g_out,
					  sizeof(g_out), &n));
}

static void test_cobs_decode_reports_enospc(void)
{
	static const uint8_t enc[] = {0x02, 0x11, 0x01}; /* -> 11 00 */
	size_t n = 0U;

	/* Short on the data byte. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC, cobs_decode(enc, sizeof(enc), g_out, 0U, &n));
	/* Room for the data byte but not the implied zero. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC, cobs_decode(enc, sizeof(enc), g_out, 1U, &n));
	/* Exactly enough. */
	TEST_ASSERT_EQUAL_INT(0, cobs_decode(enc, sizeof(enc), g_out, 2U, &n));
	TEST_ASSERT_EQUAL_size_t(2U, n);
	TEST_ASSERT_EQUAL_HEX8(0x11U, g_out[0]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, g_out[1]);
}

/* --------------------------------------------------------- streaming API - */

/* Feed a whole wire buffer; return the last non-zero result code. */
static int stream_feed(cobs_dec_t *d, const uint8_t *wire, size_t n)
{
	int last = 0;
	size_t i;

	for (i = 0U; i < n; i++) {
		int rc = cobs_dec_byte(d, wire[i]);

		if (rc != 0) {
			last = rc;
		}
	}
	return last;
}

static void test_cobs_stream_decodes_the_paper_table(void)
{
	cobs_dec_t d;
	size_t i;

	for (i = 0U; i < ARRAY_LEN(paper); i++) {
		size_t exp_len = paper[i].dec(g_dec);
		size_t enc_len = paper[i].enc(g_enc);
		size_t j;
		int rc = 0;

		TEST_ASSERT_EQUAL_INT(0, cobs_dec_init(&d, g_out, sizeof(g_out)));

		for (j = 0U; j < enc_len; j++) {
			TEST_ASSERT_EQUAL_INT_MESSAGE(0, cobs_dec_byte(&d, g_enc[j]),
						      paper[i].name);
		}
		rc = cobs_dec_byte(&d, 0x00U); /* delimiter */

		TEST_ASSERT_EQUAL_INT_MESSAGE(1, rc, paper[i].name);
		TEST_ASSERT_EQUAL_size_t_MESSAGE(exp_len, cobs_dec_len(&d),
						 paper[i].name);
		TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(g_dec, cobs_dec_buf(&d), exp_len,
						     paper[i].name);
	}
}

static void test_cobs_stream_skips_separator_runs(void)
{
	static const uint8_t wire[] = {0x00, 0x00, 0x00, 0x03, 0x11, 0x22, 0x00};
	cobs_dec_t d;
	size_t i;
	int rc = 0;

	/* Leading and repeated delimiters are separators, not malformed frames:
	 * a receiver that just came up mid-stream must not raise an error for
	 * every idle 0x00 it sees. */
	TEST_ASSERT_EQUAL_INT(0, cobs_dec_init(&d, g_out, sizeof(g_out)));
	for (i = 0U; i < sizeof(wire); i++) {
		rc = cobs_dec_byte(&d, wire[i]);
		if (i + 1U < sizeof(wire)) {
			TEST_ASSERT_EQUAL_INT(0, rc);
		}
	}
	TEST_ASSERT_EQUAL_INT(1, rc);
	TEST_ASSERT_EQUAL_size_t(2U, cobs_dec_len(&d));
	TEST_ASSERT_EQUAL_HEX8(0x11U, cobs_dec_buf(&d)[0]);
	TEST_ASSERT_EQUAL_HEX8(0x22U, cobs_dec_buf(&d)[1]);
}

static void test_cobs_stream_zero_length_payload_is_a_real_frame(void)
{
	cobs_dec_t d;

	/* 01 00 is a frame carrying nothing, which is distinct from a bare 00
	 * separator. MCP relies on the difference for payload-less commands. */
	TEST_ASSERT_EQUAL_INT(0, cobs_dec_init(&d, g_out, sizeof(g_out)));
	TEST_ASSERT_EQUAL_INT(0, cobs_dec_byte(&d, 0x01U));
	TEST_ASSERT_EQUAL_INT(1, cobs_dec_byte(&d, 0x00U));
	TEST_ASSERT_EQUAL_size_t(0U, cobs_dec_len(&d));
}

static void test_cobs_stream_reports_a_truncated_group_then_resyncs(void)
{
	static const uint8_t bad[] = {0x05, 0x11, 0x00};
	static const uint8_t good[] = {0x03, 0x11, 0x22, 0x00};
	cobs_dec_t d;

	TEST_ASSERT_EQUAL_INT(0, cobs_dec_init(&d, g_out, sizeof(g_out)));
	TEST_ASSERT_EQUAL_INT(0, cobs_dec_byte(&d, bad[0]));
	TEST_ASSERT_EQUAL_INT(0, cobs_dec_byte(&d, bad[1]));
	TEST_ASSERT_EQUAL_INT(-EBADMSG, cobs_dec_byte(&d, bad[2]));

	/* One corrupt frame must not desynchronise the stream. */
	TEST_ASSERT_EQUAL_INT(1, stream_feed(&d, good, sizeof(good)));
	TEST_ASSERT_EQUAL_size_t(2U, cobs_dec_len(&d));
	TEST_ASSERT_EQUAL_HEX8(0x11U, cobs_dec_buf(&d)[0]);
	TEST_ASSERT_EQUAL_HEX8(0x22U, cobs_dec_buf(&d)[1]);
}

static void test_cobs_stream_reports_overflow_then_resyncs(void)
{
	static const uint8_t good[] = {0x03, 0x11, 0x22, 0x00};
	uint8_t small[4];
	cobs_dec_t d;
	size_t enc_len = 0U;
	size_t i;
	int rc = 0;

	/* A frame larger than the sink: the decoder must latch -ENOSPC, swallow
	 * the rest of the frame, and report at the delimiter. */
	memset(g_dec, 0xAB, 32U);
	TEST_ASSERT_EQUAL_INT(0, cobs_encode(g_dec, 32U, g_enc, sizeof(g_enc), &enc_len));

	TEST_ASSERT_EQUAL_INT(0, cobs_dec_init(&d, small, sizeof(small)));
	for (i = 0U; i < enc_len; i++) {
		TEST_ASSERT_EQUAL_INT(0, cobs_dec_byte(&d, g_enc[i]));
	}
	rc = cobs_dec_byte(&d, 0x00U);
	TEST_ASSERT_EQUAL_INT(-ENOSPC, rc);

	TEST_ASSERT_EQUAL_INT(1, stream_feed(&d, good, sizeof(good)));
	TEST_ASSERT_EQUAL_size_t(2U, cobs_dec_len(&d));
}

static void test_cobs_stream_overflow_on_an_implied_zero(void)
{
	/* 02 11 01 decodes to {11, 00}: the second byte is the implied zero, so
	 * a capacity of exactly 1 exercises the overflow path in the code-byte
	 * branch rather than the data branch. */
	static const uint8_t wire[] = {0x02, 0x11, 0x01, 0x00};
	uint8_t one[1];
	cobs_dec_t d;

	TEST_ASSERT_EQUAL_INT(0, cobs_dec_init(&d, one, sizeof(one)));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, stream_feed(&d, wire, sizeof(wire)));
}

static void test_cobs_stream_handles_back_to_back_frames(void)
{
	test_rng_t rng;
	cobs_dec_t d;
	size_t frame;
	size_t wire_len = 0U;
	size_t lengths[6];
	size_t offsets[6];
	size_t i;

	/* Build six frames of assorted lengths into one wire buffer, then feed
	 * the whole thing a byte at a time without ever calling cobs_dec_reset()
	 * — the decoder must roll over to the next frame on its own. */
	test_rng_init(&rng, 0xFEEDU);
	for (frame = 0U; frame < ARRAY_LEN(lengths); frame++) {
		static const size_t sizes[] = {0U, 1U, 5U, 253U, 254U, 255U};
		size_t enc_len = 0U;

		lengths[frame] = sizes[frame];
		offsets[frame] = (frame == 0U) ? 0U
					       : offsets[frame - 1U] + lengths[frame - 1U];
		test_rng_fill(&rng, &g_dec[offsets[frame]], lengths[frame]);

		TEST_ASSERT_EQUAL_INT(0, cobs_encode(&g_dec[offsets[frame]],
						     lengths[frame], &g_enc[wire_len],
						     sizeof(g_enc) - wire_len, &enc_len));
		wire_len += enc_len;
		g_enc[wire_len++] = 0x00U;
	}

	TEST_ASSERT_EQUAL_INT(0, cobs_dec_init(&d, g_tmp, sizeof(g_tmp)));
	frame = 0U;
	for (i = 0U; i < wire_len; i++) {
		int rc = cobs_dec_byte(&d, g_enc[i]);

		if (rc == 1) {
			TEST_ASSERT_LESS_THAN_size_t(ARRAY_LEN(lengths), frame);
			TEST_ASSERT_EQUAL_size_t(lengths[frame], cobs_dec_len(&d));
			if (lengths[frame] != 0U) {
				TEST_ASSERT_EQUAL_HEX8_ARRAY(&g_dec[offsets[frame]],
							     cobs_dec_buf(&d),
							     lengths[frame]);
			}
			frame++;
		} else {
			TEST_ASSERT_EQUAL_INT(0, rc);
		}
	}
	TEST_ASSERT_EQUAL_size_t(ARRAY_LEN(lengths), frame);
}

static void test_cobs_stream_reset_discards_a_partial_frame(void)
{
	static const uint8_t good[] = {0x03, 0x11, 0x22, 0x00};
	cobs_dec_t d;

	TEST_ASSERT_EQUAL_INT(0, cobs_dec_init(&d, g_out, sizeof(g_out)));
	TEST_ASSERT_EQUAL_INT(0, cobs_dec_byte(&d, 0x03U));
	TEST_ASSERT_EQUAL_INT(0, cobs_dec_byte(&d, 0x99U));
	cobs_dec_reset(&d);
	TEST_ASSERT_EQUAL_size_t(0U, cobs_dec_len(&d));

	TEST_ASSERT_EQUAL_INT(1, stream_feed(&d, good, sizeof(good)));
	TEST_ASSERT_EQUAL_size_t(2U, cobs_dec_len(&d));
}

static void test_cobs_stream_rejects_bad_arguments(void)
{
	cobs_dec_t d;

	TEST_ASSERT_EQUAL_INT(-EINVAL, cobs_dec_init(NULL, g_out, sizeof(g_out)));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cobs_dec_init(&d, NULL, 8U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, cobs_dec_byte(NULL, 0x11U));

	/* NULL-tolerant accessors keep a caller's error path simple. */
	TEST_ASSERT_EQUAL_size_t(0U, cobs_dec_len(NULL));
	TEST_ASSERT_NULL(cobs_dec_buf(NULL));
	cobs_dec_reset(NULL);

	/* A zero-capacity sink is legal and still recognises an empty frame. */
	TEST_ASSERT_EQUAL_INT(0, cobs_dec_init(&d, NULL, 0U));
	TEST_ASSERT_EQUAL_INT(0, cobs_dec_byte(&d, 0x01U));
	TEST_ASSERT_EQUAL_INT(1, cobs_dec_byte(&d, 0x00U));
	TEST_ASSERT_EQUAL_size_t(0U, cobs_dec_len(&d));
}

static void test_cobs_stream_agrees_with_the_block_decoder(void)
{
	test_rng_t rng;
	cobs_dec_t d;
	size_t len;

	/* Two independent decoders, one incremental and one not, must produce
	 * identical output for every frame the encoder can emit. */
	test_rng_init(&rng, 0x2BADU);
	for (len = 0U; len <= 300U; len++) {
		size_t enc_len = 0U;
		size_t blk_len = 0U;

		test_rng_fill(&rng, g_dec, len);
		TEST_ASSERT_EQUAL_INT(0, cobs_encode(g_dec, len, g_enc,
						     sizeof(g_enc), &enc_len));
		TEST_ASSERT_EQUAL_INT(0, cobs_decode(g_enc, enc_len, g_out,
						     sizeof(g_out), &blk_len));

		TEST_ASSERT_EQUAL_INT(0, cobs_dec_init(&d, g_tmp, sizeof(g_tmp)));
		TEST_ASSERT_EQUAL_INT(0, stream_feed(&d, g_enc, enc_len));
		TEST_ASSERT_EQUAL_INT(1, cobs_dec_byte(&d, 0x00U));

		TEST_ASSERT_EQUAL_size_t(blk_len, cobs_dec_len(&d));
		if (blk_len != 0U) {
			TEST_ASSERT_EQUAL_HEX8_ARRAY(g_out, cobs_dec_buf(&d), blk_len);
		}
	}
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_cobs_encode_matches_paper_table);
	RUN_TEST(test_cobs_decode_matches_paper_table);
	RUN_TEST(test_cobs_encode_of_empty_input_is_a_single_01);
	RUN_TEST(test_cobs_decode_of_a_single_01_is_empty);
	RUN_TEST(test_cobs_254_run_encodes_without_a_trailing_group);
	RUN_TEST(test_cobs_encode_max_is_a_true_upper_bound);
	RUN_TEST(test_cobs_encoded_output_never_contains_a_zero);
	RUN_TEST(test_cobs_round_trip_over_length_and_data_sweep);
	RUN_TEST(test_cobs_round_trip_at_the_max_frame_size);
	RUN_TEST(test_cobs_decode_max_bound);

	RUN_TEST(test_cobs_encode_rejects_bad_arguments);
	RUN_TEST(test_cobs_encode_rejects_oversize_input);
	RUN_TEST(test_cobs_encode_reports_enospc);
	RUN_TEST(test_cobs_decode_rejects_bad_arguments);
	RUN_TEST(test_cobs_decode_rejects_malformed_input);
	RUN_TEST(test_cobs_decode_rejects_oversize_input);
	RUN_TEST(test_cobs_decode_reports_enospc);

	RUN_TEST(test_cobs_stream_decodes_the_paper_table);
	RUN_TEST(test_cobs_stream_skips_separator_runs);
	RUN_TEST(test_cobs_stream_zero_length_payload_is_a_real_frame);
	RUN_TEST(test_cobs_stream_reports_a_truncated_group_then_resyncs);
	RUN_TEST(test_cobs_stream_reports_overflow_then_resyncs);
	RUN_TEST(test_cobs_stream_overflow_on_an_implied_zero);
	RUN_TEST(test_cobs_stream_handles_back_to_back_frames);
	RUN_TEST(test_cobs_stream_reset_discards_a_partial_frame);
	RUN_TEST(test_cobs_stream_rejects_bad_arguments);
	RUN_TEST(test_cobs_stream_agrees_with_the_block_decoder);

	return UNITY_END();
}
