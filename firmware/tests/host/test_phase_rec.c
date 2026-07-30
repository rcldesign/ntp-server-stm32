/*
 * STS1000 "Meridian" — core/stats phase-record codec tests.
 *
 * The record is the only thing that crosses from the device to the bench, and
 * the field that matters on it is `gaps`: a consumer decides whether the
 * capture is fit for ADEV by reading that number. So the cases here are not
 * "does it round-trip" (though it must) — they are about the ways a record
 * could claim to be clean when it is not:
 *
 *   * a count that disagrees with the bitmap it is supposed to summarise;
 *   * a stale bit above the sample count, inherited from a producer whose ring
 *     is larger than the record it emitted;
 *   * a truncated buffer decoded as if it were whole.
 *
 * Each of those is rejected rather than repaired, because a repaired record is
 * indistinguishable from a real one and the operator has no way to know.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "stats/phase_rec.h"

#define BUF_CAP 8192u

static uint8_t buf[BUF_CAP];
static int64_t x_in[512];
static int64_t x_out[512];
static uint8_t map[64];

static void meta_defaults(phase_rec_meta_t *m, uint16_t n)
{
	memset(m, 0, sizeof(*m));
	m->ver = PHASE_REC_VER;
	m->n = n;
	m->tau0_ns = 1000000000u;
	m->gaps = 0u;
	m->mono_ms = 123456u;
}

/* ------------------------------------------------------------ geometry */

static void test_size_is_header_bitmap_and_samples(void)
{
	TEST_ASSERT_EQUAL_UINT32(24u, (uint32_t)phase_rec_size(0u));
	/* 1 sample: 24 header + 1 bitmap byte + 8 sample bytes. */
	TEST_ASSERT_EQUAL_UINT32(33u, (uint32_t)phase_rec_size(1u));
	/* 8 samples still fit one bitmap byte; 9 need two. */
	TEST_ASSERT_EQUAL_UINT32(24u + 1u + 64u, (uint32_t)phase_rec_size(8u));
	TEST_ASSERT_EQUAL_UINT32(24u + 2u + 72u, (uint32_t)phase_rec_size(9u));
	TEST_ASSERT_EQUAL_UINT32(24u + 64u + 4096u,
				 (uint32_t)phase_rec_size(512u));
}

/* ------------------------------------------------------------ round trip */

static void test_round_trip_preserves_every_field_and_sample(void)
{
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;
	size_t n_out = 0u;
	size_t i;

	for (i = 0u; i < 100u; i++) {
		/* Deliberately signed and large enough to exercise the full
		 * int64 path rather than a byte or two of it. */
		x_in[i] = ((int64_t)i - 50) * INT64_C(987654321);
	}

	meta_defaults(&m, 100u);
	m.flags = PHASE_REC_F_FULL;
	m.mono_ms = UINT64_C(0x0123456789ABCDEF);
	m.tau0_ns = 1000000000u;

	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_in, NULL, buf, BUF_CAP,
						  &len));
	TEST_ASSERT_EQUAL_UINT32((uint32_t)phase_rec_size(100u), (uint32_t)len);

	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(buf, len, &got));
	TEST_ASSERT_EQUAL_UINT8(PHASE_REC_VER, got.ver);
	TEST_ASSERT_EQUAL_UINT8(PHASE_REC_F_FULL, got.flags);
	TEST_ASSERT_EQUAL_UINT16(100u, got.n);
	TEST_ASSERT_EQUAL_UINT32(1000000000u, got.tau0_ns);
	TEST_ASSERT_EQUAL_UINT32(0u, got.gaps);
	TEST_ASSERT_TRUE(got.mono_ms == UINT64_C(0x0123456789ABCDEF));

	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_samples(buf, len, x_out,
							  512u, &n_out));
	TEST_ASSERT_EQUAL_UINT32(100u, (uint32_t)n_out);
	for (i = 0u; i < 100u; i++) {
		TEST_ASSERT_TRUE(x_out[i] == x_in[i]);
	}
}

static void test_empty_record_is_valid(void)
{
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;
	size_t n_out = 99u;

	meta_defaults(&m, 0u);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, NULL, NULL, buf, BUF_CAP,
						  &len));
	TEST_ASSERT_EQUAL_UINT32(24u, (uint32_t)len);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(buf, len, &got));
	TEST_ASSERT_EQUAL_UINT16(0u, got.n);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_samples(buf, len, x_out, 512u,
							  &n_out));
	TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)n_out);
}

/* --------------------------------------------------------------- the gate */

static void test_gap_count_and_bitmap_survive_the_round_trip(void)
{
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;
	size_t i;

	memset(map, 0, sizeof(map));
	/* Gaps at samples 3, 17 and 19. */
	map[0] |= (uint8_t)(1u << 3);
	map[2] |= (uint8_t)(1u << 1);
	map[2] |= (uint8_t)(1u << 3);

	for (i = 0u; i < 40u; i++) {
		x_in[i] = (int64_t)i;
	}

	meta_defaults(&m, 40u);
	m.gaps = 3u;

	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_in, map, buf, BUF_CAP,
						  &len));
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(buf, len, &got));
	TEST_ASSERT_EQUAL_UINT32(3u, got.gaps);

	for (i = 0u; i < 40u; i++) {
		bool want = (i == 3u) || (i == 17u) || (i == 19u);

		TEST_ASSERT_EQUAL_INT((int)want,
				      (int)phase_rec_gap_at(buf, len, i));
	}
	/* Out of range is not a gap, it is not a sample. */
	TEST_ASSERT_FALSE(phase_rec_gap_at(buf, len, 40u));
}

static void test_a_count_that_disagrees_with_the_bitmap_is_rejected(void)
{
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;

	memset(map, 0, sizeof(map));
	map[0] |= (uint8_t)(1u << 3);

	meta_defaults(&m, 40u);
	m.gaps = 1u;
	memset(x_in, 0, sizeof(x_in));
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_in, map, buf, BUF_CAP,
						  &len));
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(buf, len, &got));

	/*
	 * Now forge the headline: claim the record is clean while the bitmap
	 * still says otherwise. This is the exact shape of the failure the
	 * whole design exists to prevent — a consumer gates on `gaps`, so a
	 * record that lies in that field would be analysed as uniformly spaced.
	 */
	buf[12] = 0u;
	buf[13] = 0u;
	buf[14] = 0u;
	buf[15] = 0u;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, phase_rec_decode_hdr(buf, len, &got));

	/* And the converse: a count with no bits to back it. */
	buf[12] = 5u;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, phase_rec_decode_hdr(buf, len, &got));
}

static void test_stale_bits_above_the_sample_count_are_not_emitted(void)
{
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;

	/*
	 * A producer's bitmap is sized by its ring (512 bits), not by the
	 * record it emits. Emitting 5 samples must not carry bits 5..7 of the
	 * final byte with it, or the decoder's cross-check rejects a record
	 * that is in fact well formed.
	 */
	memset(map, 0xFF, sizeof(map));
	map[0] = 0xE0u; /* bits 5,6,7 set; bits 0..4 clear */

	memset(x_in, 0, sizeof(x_in));
	meta_defaults(&m, 5u);
	m.gaps = 0u;

	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_in, map, buf, BUF_CAP,
						  &len));
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(buf, len, &got));
	TEST_ASSERT_EQUAL_UINT32(0u, got.gaps);
	TEST_ASSERT_EQUAL_UINT8(0u, buf[24]);
}

static void test_longest_clean_run(void)
{
	phase_rec_meta_t m;
	size_t len = 0u;
	size_t start = 999u;

	memset(x_in, 0, sizeof(x_in));
	memset(map, 0, sizeof(map));
	/*
	 * 20 samples, gaps at 2 and 6. A gap bit on i means the interval
	 * ENDING at i is wrong, so i starts the next run:
	 *   [0,2) = 2,  [2,6) = 4,  [6,20) = 14  <- longest, starting at 6.
	 */
	map[0] |= (uint8_t)(1u << 2);
	map[0] |= (uint8_t)(1u << 6);

	meta_defaults(&m, 20u);
	m.gaps = 2u;
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_in, map, buf, BUF_CAP,
						  &len));

	TEST_ASSERT_EQUAL_UINT32(14u, (uint32_t)phase_rec_longest_clean(
					      buf, len, &start));
	TEST_ASSERT_EQUAL_UINT32(6u, (uint32_t)start);

	/* A clean record's longest run is the whole record. */
	memset(map, 0, sizeof(map));
	m.gaps = 0u;
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_in, map, buf, BUF_CAP,
						  &len));
	TEST_ASSERT_EQUAL_UINT32(20u, (uint32_t)phase_rec_longest_clean(
					      buf, len, &start));
	TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)start);
}

/* ------------------------------------------------------------ malformed */

static void test_malformed_records_are_rejected(void)
{
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;
	size_t n_out = 0u;

	memset(x_in, 0, sizeof(x_in));
	meta_defaults(&m, 10u);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_in, NULL, buf, BUF_CAP,
						  &len));

	/* Short of even a header. */
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE, phase_rec_decode_hdr(buf, 23u, &got));
	/* Header intact, body truncated: must not decode as a shorter record. */
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE,
			      phase_rec_decode_hdr(buf, len - 1u, &got));

	buf[0] ^= 0xFFu;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, phase_rec_decode_hdr(buf, len, &got));
	buf[0] ^= 0xFFu;

	buf[4] = (uint8_t)(PHASE_REC_VER + 1u);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, phase_rec_decode_hdr(buf, len, &got));
	buf[4] = (uint8_t)PHASE_REC_VER;

	/* tau0 == 0 would make every tau on the plot zero. */
	buf[8] = 0u;
	buf[9] = 0u;
	buf[10] = 0u;
	buf[11] = 0u;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, phase_rec_decode_hdr(buf, len, &got));

	/* Restore, then check gaps > n. */
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_in, NULL, buf, BUF_CAP,
						  &len));
	buf[12] = 11u;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, phase_rec_decode_hdr(buf, len, &got));

	/* A too-small output array must write nothing rather than truncate:
	 * a silently shortened phase series changes the answer invisibly. */
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_in, NULL, buf, BUF_CAP,
						  &len));
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE,
			      phase_rec_decode_samples(buf, len, x_out, 9u,
						       &n_out));

	/* Derived readers refuse an invalid record rather than guessing. */
	buf[0] ^= 0xFFu;
	TEST_ASSERT_FALSE(phase_rec_gap_at(buf, len, 0u));
	TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)phase_rec_longest_clean(buf, len,
								       NULL));
}

static void test_encode_argument_errors(void)
{
	phase_rec_meta_t m;
	size_t len = 0u;

	meta_defaults(&m, 10u);
	memset(x_in, 0, sizeof(x_in));

	TEST_ASSERT_EQUAL_INT(-EINVAL, phase_rec_encode(NULL, x_in, NULL, buf,
							BUF_CAP, &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, phase_rec_encode(&m, NULL, NULL, buf,
							BUF_CAP, &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, phase_rec_encode(&m, x_in, NULL, NULL,
							BUF_CAP, &len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, phase_rec_encode(&m, x_in, NULL, buf,
							BUF_CAP, NULL));
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE, phase_rec_encode(&m, x_in, NULL, buf,
							  32u, &len));

	m.tau0_ns = 0u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, phase_rec_encode(&m, x_in, NULL, buf,
							BUF_CAP, &len));
	m.tau0_ns = 1000000000u;

	m.gaps = 11u; /* > n */
	TEST_ASSERT_EQUAL_INT(-EINVAL, phase_rec_encode(&m, x_in, NULL, buf,
							BUF_CAP, &len));

	/* A gap count with no map is unresolvable, so it is not encodable. */
	m.gaps = 1u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, phase_rec_encode(&m, x_in, NULL, buf,
							BUF_CAP, &len));
}

/* ---------------------------------------------------------- ns -> ps */

static void test_ns_float_to_ps_conversion(void)
{
	TEST_ASSERT_TRUE(phase_rec_ns_f_to_ps(0.0f) == 0);
	TEST_ASSERT_TRUE(phase_rec_ns_f_to_ps(1.0f) == 1000);
	TEST_ASSERT_TRUE(phase_rec_ns_f_to_ps(-1.0f) == -1000);
	/* Rounds to nearest, symmetric about zero. */
	TEST_ASSERT_TRUE(phase_rec_ns_f_to_ps(0.0015f) == 2);
	TEST_ASSERT_TRUE(phase_rec_ns_f_to_ps(-0.0015f) == -2);
	TEST_ASSERT_TRUE(phase_rec_ns_f_to_ps(12.5f) == 12500);

	/* Saturates rather than invoking undefined behaviour. */
	TEST_ASSERT_TRUE(phase_rec_ns_f_to_ps(1.0e30f) ==
			 INT64_C(1000000000000000));
	TEST_ASSERT_TRUE(phase_rec_ns_f_to_ps(-1.0e30f) ==
			 -INT64_C(1000000000000000));

	/* A NaN is "no measurement", not a 1000-second phase error. */
	{
		float nan_v = 0.0f;
		float zero = 0.0f;

		nan_v = zero / zero;
		TEST_ASSERT_TRUE(phase_rec_ns_f_to_ps(nan_v) == 0);
	}
}

/* ------------------------------------------------------------- identity */

static void test_identity_round_trips_and_sets_the_flag(void)
{
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;

	memset(x_in, 0, sizeof(x_in));
	meta_defaults(&m, 10u);
	m.has_ident = true;
	m.epoch = 0xDEADBEEFu;
	m.seq0 = UINT64_C(0x0102030405060708);

	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_in, NULL, buf, BUF_CAP,
						  &len));
	/* The identity block is 12 bytes appended past the single series. */
	TEST_ASSERT_EQUAL_UINT32((uint32_t)(phase_rec_size(10u) + 12u),
				 (uint32_t)len);

	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(buf, len, &got));
	TEST_ASSERT_TRUE((got.flags & PHASE_REC_F_IDENT) != 0u);
	TEST_ASSERT_TRUE(got.has_ident);
	TEST_ASSERT_EQUAL_UINT32(0xDEADBEEFu, got.epoch);
	TEST_ASSERT_TRUE(got.seq0 == UINT64_C(0x0102030405060708));

	/*
	 * And a record WITHOUT identity reads back has_ident false with the
	 * fields zeroed — not "epoch 0, sample 0", which is a legitimate
	 * identity a consumer must not be able to infer from its absence.
	 */
	meta_defaults(&m, 10u);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_in, NULL, buf, BUF_CAP,
						  &len));
	TEST_ASSERT_EQUAL_UINT32((uint32_t)phase_rec_size(10u), (uint32_t)len);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(buf, len, &got));
	TEST_ASSERT_FALSE(got.has_ident);
	TEST_ASSERT_EQUAL_UINT32(0u, got.epoch);
	TEST_ASSERT_TRUE(got.seq0 == 0u);
}

static void test_identity_sits_past_both_series_when_paired(void)
{
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;
	size_t n_out = 0u;

	memset(x_in, 0, sizeof(x_in));
	meta_defaults(&m, 16u);
	m.has_ident = true;
	m.epoch = 7u;
	m.seq0 = 4096u;

	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode_pair(&m, x_in, x_in, NULL,
						       buf, BUF_CAP, &len));
	/* header + bitmap + two series + identity. */
	TEST_ASSERT_EQUAL_UINT32(24u + 2u + 128u + 128u + 12u, (uint32_t)len);
	TEST_ASSERT_EQUAL_UINT32((uint32_t)len,
				 (uint32_t)phase_rec_size_flags(
					 16u, (uint8_t)(PHASE_REC_F_RAW |
							PHASE_REC_F_IDENT)));

	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(buf, len, &got));
	TEST_ASSERT_EQUAL_UINT32(7u, got.epoch);
	TEST_ASSERT_TRUE(got.seq0 == 4096u);
	/*
	 * Both series still decode: the identity is appended, so it cannot
	 * have displaced anything the older layout put in front of it.
	 */
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_samples(buf, len, x_out,
							  512u, &n_out));
	TEST_ASSERT_EQUAL_UINT32(16u, (uint32_t)n_out);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_raw(buf, len, x_out, 512u,
						      &n_out));
	TEST_ASSERT_EQUAL_UINT32(16u, (uint32_t)n_out);
}

/*
 * The stale-decoder property, stated at the format's own level: a record that
 * DECLARES the identity block must be long enough to hold it, and a version
 * that predates the block cannot declare it. These are the same two guards
 * v2 armed for the raw series, which is the whole point of following the
 * pattern rather than inventing a new one.
 */
static void test_a_pre_v3_record_cannot_claim_the_identity_block(void)
{
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;

	memset(x_in, 0, sizeof(x_in));
	meta_defaults(&m, 10u);
	m.has_ident = true;
	m.epoch = 5u;
	m.seq0 = 99u;
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_in, NULL, buf, BUF_CAP,
						  &len));
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(buf, len, &got));
	TEST_ASSERT_EQUAL_UINT8(PHASE_REC_VER, got.ver);
	TEST_ASSERT_TRUE(got.ver >= 3u);

	/* Say it is v2 and the claim becomes impossible: F_IDENT did not exist,
	 * exactly as F_RAW did not exist at v1. */
	buf[4] = 2u;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, phase_rec_decode_hdr(buf, len, &got));
	buf[4] = 1u;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, phase_rec_decode_hdr(buf, len, &got));
	buf[4] = (uint8_t)PHASE_REC_VER;
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(buf, len, &got));

	/* And the length must back the claim: strip the block and keep the
	 * flag, and the record must be refused, not read 12 bytes short. */
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE,
			      phase_rec_decode_hdr(buf, len - 12u, &got));
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE,
			      phase_rec_decode_hdr(buf, len - 1u, &got));
}

/*
 * The other direction, and the one a stale build actually hits: THIS decoder
 * is v3, so it must refuse the v4 record it cannot lay out — the same refusal a
 * v2 build makes when handed one of ours. Proven by construction: a record
 * whose only change is a version this build does not know, and a record with an
 * unknown flag bit, are both rejected rather than read as far as they parse.
 */
static void test_an_unknown_version_or_flag_is_refused_not_partly_read(void)
{
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;

	memset(x_in, 0, sizeof(x_in));
	meta_defaults(&m, 10u);
	m.has_ident = true;
	m.epoch = 5u;
	m.seq0 = 99u;
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_in, NULL, buf, BUF_CAP,
						  &len));

	buf[4] = (uint8_t)(PHASE_REC_VER + 1u);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, phase_rec_decode_hdr(buf, len, &got));
	buf[4] = (uint8_t)PHASE_REC_VER;

	/* An unknown flag declares content of unknown length. 0x08 is the next
	 * bit a v4 would spend; this build must not guess at its layout. */
	buf[5] |= 0x08u;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, phase_rec_decode_hdr(buf, len, &got));
	buf[5] &= (uint8_t)~0x08u;
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(buf, len, &got));

	/* An encoder must not write one either: a record nothing can open is a
	 * worse outcome than the caller finding out at the call site. */
	meta_defaults(&m, 10u);
	m.flags = 0x08u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, phase_rec_encode(&m, x_in, NULL, buf,
							BUF_CAP, &len));
}

/*
 * The encoder decides the flag from what it wrote, not from what the caller
 * asked for — the same rule F_RAW follows. A caller cannot conjure an identity
 * block by setting the bit, and cannot suppress one it supplied values for.
 */
static void test_the_ident_flag_records_what_was_written(void)
{
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;

	memset(x_in, 0, sizeof(x_in));

	/* Bit set, has_ident false: no block, no flag, base length. */
	meta_defaults(&m, 10u);
	m.flags = PHASE_REC_F_IDENT;
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_in, NULL, buf, BUF_CAP,
						  &len));
	TEST_ASSERT_EQUAL_UINT32((uint32_t)phase_rec_size(10u), (uint32_t)len);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(buf, len, &got));
	TEST_ASSERT_FALSE(got.has_ident);

	/* Bit clear, has_ident true: the block is written and the flag set. */
	meta_defaults(&m, 10u);
	m.flags = 0u;
	m.has_ident = true;
	m.epoch = 42u;
	m.seq0 = 17u;
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_in, NULL, buf, BUF_CAP,
						  &len));
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(buf, len, &got));
	TEST_ASSERT_TRUE(got.has_ident);
	TEST_ASSERT_EQUAL_UINT32(42u, got.epoch);
	TEST_ASSERT_TRUE(got.seq0 == 17u);
}

static void test_series_pointer_reads_the_same_samples_as_the_decoder(void)
{
	phase_rec_meta_t m;
	phase_rec_meta_t got;
	size_t len = 0u;
	size_t i;

	for (i = 0u; i < 32u; i++) {
		x_in[i] = ((int64_t)i * INT64_C(-7919)) + INT64_C(11);
		x_out[i] = (int64_t)i * INT64_C(1000);
	}
	meta_defaults(&m, 32u);
	m.has_ident = true;
	m.epoch = 3u;
	m.seq0 = 1000u;
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode_pair(&m, x_in, x_out, NULL,
						       buf, BUF_CAP, &len));
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(buf, len, &got));

	{
		const uint8_t *sc = phase_rec_series(buf, &got, 0u);
		const uint8_t *sr = phase_rec_series(buf, &got, 1u);

		TEST_ASSERT_NOT_NULL(sc);
		TEST_ASSERT_NOT_NULL(sr);
		for (i = 0u; i < 32u; i++) {
			TEST_ASSERT_TRUE(phase_rec_sample(sc, i) == x_in[i]);
			TEST_ASSERT_TRUE(phase_rec_sample(sr, i) == x_out[i]);
		}
	}

	/* A single-series record has no second series to point at. */
	meta_defaults(&m, 32u);
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode(&m, x_in, NULL, buf, BUF_CAP,
						  &len));
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(buf, len, &got));
	TEST_ASSERT_NULL(phase_rec_series(buf, &got, 1u));
	TEST_ASSERT_NOT_NULL(phase_rec_series(buf, &got, 0u));
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_size_is_header_bitmap_and_samples);
	RUN_TEST(test_round_trip_preserves_every_field_and_sample);
	RUN_TEST(test_empty_record_is_valid);

	RUN_TEST(test_gap_count_and_bitmap_survive_the_round_trip);
	RUN_TEST(test_a_count_that_disagrees_with_the_bitmap_is_rejected);
	RUN_TEST(test_stale_bits_above_the_sample_count_are_not_emitted);
	RUN_TEST(test_longest_clean_run);

	RUN_TEST(test_malformed_records_are_rejected);
	RUN_TEST(test_encode_argument_errors);
	RUN_TEST(test_ns_float_to_ps_conversion);

	RUN_TEST(test_identity_round_trips_and_sets_the_flag);
	RUN_TEST(test_identity_sits_past_both_series_when_paired);
	RUN_TEST(test_a_pre_v3_record_cannot_claim_the_identity_block);
	RUN_TEST(test_an_unknown_version_or_flag_is_refused_not_partly_read);
	RUN_TEST(test_the_ident_flag_records_what_was_written);
	RUN_TEST(test_series_pointer_reads_the_same_samples_as_the_decoder);

	return UNITY_END();
}
