/*
 * STS1000 "Meridian" — core/stats phase-record JOIN tests.
 *
 * The device ring is 512 samples at tau0 = 1 s, so one export supports m up to
 * (n-1)/2 = 255 and its octave axis stops at tau = 128 s. Spec §14 asks about
 * tau of hundreds to thousands of seconds. The answer is not a bigger ring —
 * the part has no RAM for one — it is polling repeatedly and concatenating.
 *
 * Concatenation is where this goes wrong silently. Two windows that overlap
 * splice duplicated seconds into the series; two that straddle a dropped
 * stretch splice a hole out of existence. Either yields a phase array whose
 * time axis is wrong, from which stats_adev() returns a confident number that
 * looks entirely ordinary — the same failure the record's absorbed-gap count
 * exists to stop, one level up.
 *
 * So these are not round-trip tests. Every one of them is a way two records
 * could be spliced when they must not be, or a way the joined record could
 * misdescribe what it holds:
 *
 *   * records from either side of a ring reset (different epoch);
 *   * records whose overlap disagrees about a sample they both claim;
 *   * records with a hole between them;
 *   * records handed over out of order;
 *   * a joined gap bitmap that is not the union of its parts', or is the union
 *     placed at the wrong bit offset — which relocates every discontinuity the
 *     field exists to name;
 *   * a sample index that does not advance, which makes every window look like
 *     the same window.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "stats/phase_join.h"
#include "stats/phase_rec.h"

#define SRC_N 512u          /* one device-sized window */
#define DST_CAP 4096u       /* room for several of them */
#define REC_CAP 16384u

static uint8_t rec_a[REC_CAP];
static uint8_t rec_b[REC_CAP];
static uint8_t rec_c[REC_CAP];

static int64_t dst_x[DST_CAP];
static int64_t dst_raw[DST_CAP];
static uint8_t dst_map[(DST_CAP + 7u) / 8u];

static int64_t src_x[SRC_N];
static int64_t src_raw[SRC_N];
static uint8_t src_map[(SRC_N + 7u) / 8u];

/*
 * The synthetic "device": sample k of the capture has a deterministic value, so
 * two windows that overlap carry byte-identical samples exactly as the real
 * ring's do. A join that mis-aligns by even one sample fails the overlap check
 * against these, which is the property the whole scheme rests on.
 */
static int64_t sample_of(uint64_t k)
{
	return (int64_t)(k * UINT64_C(7919)) - INT64_C(31337);
}

static int64_t raw_of(uint64_t k)
{
	return sample_of(k) + INT64_C(500);
}

/* True when absolute sample k is marked as following a non-uniform interval. */
static bool gap_of(uint64_t k)
{
	return (k == 300u) || (k == 601u) || (k == 1024u);
}

/*
 * Encode one window [seq0, seq0+n) of that capture into @p buf.
 *
 * @p epoch, @p seq0 and @p n are the knobs every test turns; the samples and
 * gap bits follow from the absolute index, so nothing here can accidentally
 * make two windows agree that should not.
 */
static size_t make_window(uint8_t *buf, uint32_t epoch, uint64_t seq0,
			  uint16_t n, bool with_raw, bool with_ident)
{
	phase_rec_meta_t m;
	size_t len = 0u;
	size_t i;
	uint32_t gaps = 0u;

	TEST_ASSERT_TRUE(n <= SRC_N);
	memset(src_map, 0, sizeof(src_map));
	for (i = 0u; i < (size_t)n; i++) {
		uint64_t k = seq0 + (uint64_t)i;

		src_x[i] = sample_of(k);
		src_raw[i] = raw_of(k);
		if (gap_of(k)) {
			src_map[i >> 3] |= (uint8_t)(1u << (i & 7u));
			gaps++;
		}
	}

	memset(&m, 0, sizeof(m));
	m.ver = PHASE_REC_VER;
	m.flags = PHASE_REC_F_FULL;
	m.n = n;
	m.tau0_ns = 1000000000u;
	m.gaps = gaps;
	m.mono_ms = 1000u + seq0;
	m.has_ident = with_ident;
	m.epoch = epoch;
	m.seq0 = seq0;

	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode_pair(&m, src_x,
						       with_raw ? src_raw
								: NULL,
						       src_map, buf, REC_CAP,
						       &len));
	return len;
}

static void join_reset(phase_join_t *j)
{
	memset(dst_x, 0, sizeof(dst_x));
	memset(dst_raw, 0, sizeof(dst_raw));
	TEST_ASSERT_EQUAL_INT(0, phase_join_init(j, dst_x, dst_raw, dst_map,
						 DST_CAP));
}

/* ------------------------------------------------------- the happy path */

/*
 * Two overlapping windows join into one series that is exactly the capture.
 * The overlap is not tolerated, it is CONSUMED: 256 of the 512 samples in the
 * second record are checked against what the first one already said.
 */
static void test_overlapping_windows_join_into_the_underlying_capture(void)
{
	phase_join_t j;
	size_t la;
	size_t lb;
	size_t i;

	join_reset(&j);
	la = make_window(rec_a, 9u, 0u, 512u, true, true);
	lb = make_window(rec_b, 9u, 256u, 512u, true, true);

	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_b, lb));

	TEST_ASSERT_EQUAL_UINT32(2u, (uint32_t)j.parts);
	TEST_ASSERT_EQUAL_UINT32(768u, (uint32_t)j.n);
	TEST_ASSERT_EQUAL_UINT32(256u, (uint32_t)j.overlap);
	TEST_ASSERT_TRUE(j.seq0 == 0u);
	TEST_ASSERT_EQUAL_UINT32(9u, j.epoch);

	for (i = 0u; i < 768u; i++) {
		TEST_ASSERT_TRUE(dst_x[i] == sample_of((uint64_t)i));
		TEST_ASSERT_TRUE(dst_raw[i] == raw_of((uint64_t)i));
	}
}

/* Abutting windows — zero overlap — are contiguous and join. */
static void test_exactly_abutting_windows_join(void)
{
	phase_join_t j;
	size_t la;
	size_t lb;

	join_reset(&j);
	la = make_window(rec_a, 4u, 100u, 512u, true, true);
	lb = make_window(rec_b, 4u, 612u, 512u, true, true);

	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_b, lb));
	TEST_ASSERT_EQUAL_UINT32(1024u, (uint32_t)j.n);
	TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)j.overlap);
	TEST_ASSERT_TRUE(j.seq0 == 100u);
}

/* A window wholly inside the accumulated span adds nothing and must not
 * corrupt anything: it is 512 samples of pure verification. */
static void test_a_contained_window_verifies_and_adds_nothing(void)
{
	phase_join_t j;
	size_t la;
	size_t lb;
	size_t lc;

	join_reset(&j);
	la = make_window(rec_a, 2u, 0u, 512u, true, true);
	lb = make_window(rec_b, 2u, 400u, 512u, true, true);
	lc = make_window(rec_c, 2u, 100u, 200u, true, true);

	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_b, lb));
	TEST_ASSERT_EQUAL_UINT32(912u, (uint32_t)j.n);

	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_c, lc));
	TEST_ASSERT_EQUAL_UINT32(912u, (uint32_t)j.n);
	TEST_ASSERT_EQUAL_UINT32(112u + 200u, (uint32_t)j.overlap);
}

/* ------------------------------------------------------- the refusals */

/*
 * THE EPOCH GUARD. adev_reset() — a PPS gap past ADEV_MAX_GAP_S, entry to
 * holdover, disc_restart(), a reboot — restarts the numbering. Records from
 * either side describe unrelated series, and their indices may abut or overlap
 * perfectly plausibly. The epoch is what makes that impossible to miss.
 */
static void test_records_from_different_epochs_refuse_to_join(void)
{
	phase_join_t j;
	size_t la;
	size_t lb;

	join_reset(&j);
	/* Indices that would otherwise join without complaint: a clean abut. */
	la = make_window(rec_a, 9u, 0u, 512u, true, true);
	lb = make_window(rec_b, 10u, 512u, 512u, true, true);

	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(-EPROTO, phase_join_add(&j, rec_b, lb));

	/* Refused means untouched: the accumulator still holds only part one. */
	TEST_ASSERT_EQUAL_UINT32(512u, (uint32_t)j.n);
	TEST_ASSERT_EQUAL_UINT32(1u, (uint32_t)j.parts);

	/* And an overlap does not rescue it — the epoch is checked first,
	 * because agreeing samples across a reset would be a coincidence, not
	 * a proof. */
	join_reset(&j);
	lb = make_window(rec_b, 10u, 256u, 512u, true, true);
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(-EPROTO, phase_join_add(&j, rec_b, lb));
}

/*
 * THE OVERLAP GUARD. Two records claiming the same samples must agree about
 * them. They came from the same ring slots within the same epoch, so a
 * disagreement means the epoch is being reused or one record is corrupt —
 * and nothing here can tell which, so joining either version is a guess.
 */
static void test_a_disagreeing_overlap_refuses_to_join(void)
{
	phase_join_t j;
	size_t la;
	size_t lb;
	const uint8_t *series;
	phase_rec_meta_t m;
	size_t off;

	join_reset(&j);
	la = make_window(rec_a, 9u, 0u, 512u, true, true);
	lb = make_window(rec_b, 9u, 256u, 512u, true, true);

	/* Corrupt ONE corrected sample inside the overlap (absolute 300, which
	 * is index 44 of record B). One byte, one sample, deep in the middle. */
	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec_b, lb, &m));
	series = phase_rec_series(rec_b, &m, 0u);
	off = (size_t)(series - rec_b) + (44u * 8u);
	rec_b[off] ^= 0x01u;

	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, phase_join_add(&j, rec_b, lb));
	TEST_ASSERT_EQUAL_UINT32(512u, (uint32_t)j.n);
	rec_b[off] ^= 0x01u;

	/* The UNCORRECTED series is checked too: §14's sawtooth proof is a
	 * comparison of the pair, so a join that verified only the corrected
	 * series could splice two different uncorrected histories together. */
	join_reset(&j);
	series = phase_rec_series(rec_b, &m, 1u);
	off = (size_t)(series - rec_b) + (44u * 8u);
	rec_b[off] ^= 0x01u;
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, phase_join_add(&j, rec_b, lb));
	rec_b[off] ^= 0x01u;

	/* Sanity: with the corruption undone the same pair joins. */
	join_reset(&j);
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_b, lb));
}

/*
 * A gap bit that disagrees across the overlap is the same corruption seen from
 * the other side, and is refused for the same reason: the two records describe
 * the same second's spacing differently, and the joined record can only carry
 * one answer.
 */
static void test_a_disagreeing_overlap_gap_bit_refuses_to_join(void)
{
	phase_join_t j;
	size_t la;
	size_t lb;

	join_reset(&j);
	la = make_window(rec_a, 9u, 0u, 512u, true, true);
	lb = make_window(rec_b, 9u, 256u, 512u, true, true);

	/*
	 * Record B's index 44 is absolute 300, which gap_of() marks. Clear the
	 * bit and fix up the count so the record is still internally
	 * consistent — otherwise decode_hdr() would reject it for the gap
	 * count and this test would pass without ever reaching the join.
	 */
	rec_b[PHASE_REC_HDR_LEN + (44u >> 3)] &= (uint8_t)~(1u << (44u & 7u));
	TEST_ASSERT_TRUE(rec_b[12] > 0u);
	rec_b[12]--;
	{
		phase_rec_meta_t m;

		TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(rec_b, lb, &m));
	}

	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(-EILSEQ, phase_join_add(&j, rec_b, lb));
}

/*
 * THE HOLE GUARD, and the decision behind it: a non-contiguous boundary is
 * REFUSED, not marked as a gap.
 *
 * The bitmap says "the interval ending at sample i was not tau0". It has no
 * field for how long the discontinuity was. Marking a 512-sample hole as one
 * gap bit would leave --allow-gaps computing over a series whose time axis is
 * short by more than eight minutes, and would leave the record claiming a
 * duration it does not have. Two chains that cannot be joined are two records.
 */
static void test_a_hole_between_records_refuses_to_join(void)
{
	phase_join_t j;
	size_t la;
	size_t lb;

	join_reset(&j);
	la = make_window(rec_a, 9u, 0u, 512u, true, true);
	/* One single sample missing is still a hole. */
	lb = make_window(rec_b, 9u, 513u, 512u, true, true);

	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(-ERANGE, phase_join_add(&j, rec_b, lb));
	TEST_ASSERT_EQUAL_UINT32(512u, (uint32_t)j.n);

	/*
	 * The refusal changed nothing: the only gap in the accumulator is
	 * still part A's own (absolute 300), and in particular the hole was NOT
	 * quietly recorded as an absorbed gap at the boundary. Marking it there
	 * would let --allow-gaps compute over a series eight minutes short of
	 * the duration it claims.
	 */
	TEST_ASSERT_EQUAL_UINT32(1u, (uint32_t)j.gaps);
	TEST_ASSERT_TRUE(dst_map[300u >> 3] & (uint8_t)(1u << (300u & 7u)));
	TEST_ASSERT_FALSE(dst_map[511u >> 3] & (uint8_t)(1u << (511u & 7u)));

	/* A long hole is the same answer, not a worse one. */
	join_reset(&j);
	lb = make_window(rec_b, 9u, 5000u, 512u, true, true);
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(-ERANGE, phase_join_add(&j, rec_b, lb));
}

/* Out of order is refused rather than silently reordered: the joiner extends
 * forward only, and a caller that hands records over backwards is told so. */
static void test_out_of_order_records_refuse_to_join(void)
{
	phase_join_t j;
	size_t la;
	size_t lb;

	join_reset(&j);
	la = make_window(rec_a, 9u, 256u, 512u, true, true);
	lb = make_window(rec_b, 9u, 0u, 512u, true, true);

	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(-EALREADY, phase_join_add(&j, rec_b, lb));
	TEST_ASSERT_EQUAL_UINT32(512u, (uint32_t)j.n);
}

/* A record with no identity cannot be joined to anything: there is nothing in
 * it that says which samples it holds. */
static void test_a_record_without_identity_refuses_to_join(void)
{
	phase_join_t j;
	size_t la;
	size_t lb;

	join_reset(&j);
	la = make_window(rec_a, 9u, 0u, 512u, true, false);
	TEST_ASSERT_EQUAL_INT(-ENOENT, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)j.parts);

	/* Nor can one be appended to a chain that does have identity. */
	la = make_window(rec_a, 9u, 0u, 512u, true, true);
	lb = make_window(rec_b, 9u, 256u, 512u, true, false);
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(-ENOENT, phase_join_add(&j, rec_b, lb));
}

/* Mismatched shapes are refused rather than half-joined. */
static void test_mismatched_tau0_or_series_refuses_to_join(void)
{
	phase_join_t j;
	size_t la;
	size_t lb;

	/* One part paired, the next single-series: §14's proof needs both, and
	 * a join that dropped the second series would lose it silently. */
	join_reset(&j);
	la = make_window(rec_a, 9u, 0u, 512u, true, true);
	lb = make_window(rec_b, 9u, 256u, 512u, false, true);
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, phase_join_add(&j, rec_b, lb));

	/* And a paired record cannot be joined into a destination with nowhere
	 * to put the uncorrected series. */
	TEST_ASSERT_EQUAL_INT(0, phase_join_init(&j, dst_x, NULL, dst_map,
						 DST_CAP));
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, phase_join_add(&j, rec_a, la));

	/* tau0 must match: joining a 1 s series onto a 10 s one would put a
	 * factor of ten into every tau on the plot. */
	join_reset(&j);
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	lb = make_window(rec_b, 9u, 256u, 512u, true, true);
	rec_b[8] = 0x00u;
	rec_b[9] = 0xE4u;
	rec_b[10] = 0x0Bu;
	rec_b[11] = 0x54u; /* 1410065408 ns, simply not 1e9 */
	TEST_ASSERT_EQUAL_INT(-ENOTSUP, phase_join_add(&j, rec_b, lb));
}

/* The buffer bound is enforced, and enforced before anything is written. */
static void test_a_join_larger_than_the_buffer_refuses(void)
{
	phase_join_t j;
	size_t la;
	size_t lb;

	TEST_ASSERT_EQUAL_INT(0, phase_join_init(&j, dst_x, dst_raw, dst_map,
						 600u));
	la = make_window(rec_a, 9u, 0u, 512u, true, true);
	lb = make_window(rec_b, 9u, 256u, 512u, true, true);
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, phase_join_add(&j, rec_b, lb));
	TEST_ASSERT_EQUAL_UINT32(512u, (uint32_t)j.n);
}

/* --------------------------------------------------- the gap bitmap union */

/*
 * THE UNION. The joined bitmap must hold every gap of every part, at the
 * ABSOLUTE index the sample has in the join — and `gaps` must be its population
 * count, because phase_rec_decode_hdr() cross-checks the two and a joined
 * record that fails that check is unreadable.
 *
 * The offsets here are deliberately not multiples of 8 (509 and 1013): the ring
 * slides one sample a second, so a real poll never lands on a byte boundary
 * except by accident, and a byte-wise copy of a part's bitmap would displace
 * every gap in it by up to seven samples — relocating exactly the
 * discontinuities the field exists to name, while keeping the count right.
 */
static void test_the_joined_gap_bitmap_is_the_union_at_absolute_indices(void)
{
	phase_join_t j;
	phase_rec_meta_t jm;
	size_t la;
	size_t lb;
	size_t lc;
	size_t i;
	size_t counted = 0u;

	join_reset(&j);
	/* gap_of() marks absolute 300, 601 and 1024. The three windows below
	 * are chosen so that each part contributes a different one, and so
	 * that 601 falls in an overlap and is therefore asserted twice. */
	la = make_window(rec_a, 5u, 0u, 512u, true, true);
	lb = make_window(rec_b, 5u, 509u, 512u, true, true);
	lc = make_window(rec_c, 5u, 1013u, 512u, true, true);

	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_b, lb));
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_c, lc));
	TEST_ASSERT_EQUAL_UINT32(1525u, (uint32_t)j.n);

	/* Every bit, everywhere: set exactly where the capture had a gap. */
	for (i = 0u; i < j.n; i++) {
		bool got = (dst_map[i >> 3] & (uint8_t)(1u << (i & 7u))) != 0u;

		if (gap_of((uint64_t)i)) {
			TEST_ASSERT_TRUE_MESSAGE(got,
						 "a part's gap is missing from "
						 "the joined bitmap");
			counted++;
		} else {
			TEST_ASSERT_FALSE_MESSAGE(got,
						  "the joined bitmap invented "
						  "a gap");
		}
	}
	TEST_ASSERT_EQUAL_UINT32(3u, (uint32_t)counted);

	/* The count is the population count of that union — a part asserting a
	 * bit twice (601 is in an overlap) must not count twice. */
	TEST_ASSERT_EQUAL_UINT32(3u, (uint32_t)j.gaps);

	/* And the joined record is readable: decode_hdr() cross-checks the
	 * count against the bitmap, so a wrong union is fatal there too. */
	TEST_ASSERT_EQUAL_INT(0, phase_join_meta(&j, &jm));
	TEST_ASSERT_EQUAL_UINT32(3u, jm.gaps);
	TEST_ASSERT_EQUAL_UINT32(1525u, (uint32_t)jm.n);
	TEST_ASSERT_TRUE(jm.has_ident);
	TEST_ASSERT_EQUAL_UINT32(5u, jm.epoch);
	TEST_ASSERT_TRUE(jm.seq0 == 0u);
}

/*
 * The joined record re-encodes, decodes, and is itself joinable — which is
 * what makes a long capture something an operator can archive and extend
 * rather than a one-shot in-memory artefact.
 */
static void test_a_joined_record_round_trips_and_can_be_joined_again(void)
{
	phase_join_t j;
	phase_rec_meta_t jm;
	phase_rec_meta_t got;
	static uint8_t out[REC_CAP * 4u];
	size_t out_len = 0u;
	size_t la;
	size_t lb;
	size_t i;

	join_reset(&j);
	la = make_window(rec_a, 5u, 0u, 512u, true, true);
	lb = make_window(rec_b, 5u, 509u, 512u, true, true);
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_b, lb));
	TEST_ASSERT_EQUAL_INT(0, phase_join_meta(&j, &jm));
	TEST_ASSERT_EQUAL_INT(0, phase_rec_encode_pair(&jm, dst_x, dst_raw,
						       dst_map, out,
						       sizeof(out), &out_len));

	TEST_ASSERT_EQUAL_INT(0, phase_rec_decode_hdr(out, out_len, &got));
	TEST_ASSERT_EQUAL_UINT32(1021u, (uint32_t)got.n);
	TEST_ASSERT_EQUAL_UINT32(2u, got.gaps);
	TEST_ASSERT_TRUE(got.has_ident);
	TEST_ASSERT_EQUAL_UINT32(5u, got.epoch);
	TEST_ASSERT_TRUE(got.seq0 == 0u);
	/* The FIRST part's provenance, because seq0 names the first part's
	 * sample 0 and the two must describe the same capture. */
	TEST_ASSERT_TRUE(got.mono_ms == 1000u);

	/* Its samples are the capture's. */
	{
		const uint8_t *sc = phase_rec_series(out, &got, 0u);
		const uint8_t *sr = phase_rec_series(out, &got, 1u);

		for (i = 0u; i < got.n; i++) {
			TEST_ASSERT_TRUE(phase_rec_sample(sc, i) ==
					 sample_of((uint64_t)i));
			TEST_ASSERT_TRUE(phase_rec_sample(sr, i) ==
					 raw_of((uint64_t)i));
		}
	}

	/* And it extends: a later window of the same epoch joins onto it. */
	join_reset(&j);
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, out, out_len));
	lb = make_window(rec_b, 5u, 900u, 512u, true, true);
	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_b, lb));
	TEST_ASSERT_EQUAL_UINT32(1412u, (uint32_t)j.n);
	TEST_ASSERT_EQUAL_UINT32(121u, (uint32_t)j.overlap);
}

/*
 * THE INDEX ITSELF. If the sample index does not advance between polls, every
 * window looks like the same window: the second record claims the first
 * record's indices while holding different samples, and the overlap check is
 * the thing that catches it. This is the test that fails if a device stops
 * incrementing its counter.
 */
static void test_an_index_that_does_not_advance_is_caught(void)
{
	phase_join_t j;
	size_t la;
	size_t lb;

	join_reset(&j);
	la = make_window(rec_a, 6u, 0u, 512u, true, true);
	/* The device really moved on to samples [512, 1024) but reported seq0
	 * = 0 — a counter that never incremented. */
	lb = make_window(rec_b, 6u, 512u, 512u, true, true);
	rec_b[phase_rec_size_flags(512u, (uint8_t)(PHASE_REC_F_RAW)) + 4u] = 0u;
	{
		size_t base = phase_rec_size_flags(512u, PHASE_REC_F_RAW);
		size_t k;

		for (k = 4u; k < 12u; k++) {
			rec_b[base + k] = 0u;
		}
	}

	TEST_ASSERT_EQUAL_INT(0, phase_join_add(&j, rec_a, la));
	/* Same claimed indices, different samples: the overlap disagrees. */
	TEST_ASSERT_EQUAL_INT(-EILSEQ, phase_join_add(&j, rec_b, lb));
	TEST_ASSERT_EQUAL_UINT32(512u, (uint32_t)j.n);
}

/* --------------------------------------------------------------- plumbing */

static void test_argument_and_state_errors(void)
{
	phase_join_t j;
	phase_rec_meta_t jm;
	size_t la;

	TEST_ASSERT_EQUAL_INT(-EINVAL, phase_join_init(NULL, dst_x, dst_raw,
						       dst_map, DST_CAP));
	TEST_ASSERT_EQUAL_INT(-EINVAL, phase_join_init(&j, NULL, dst_raw,
						       dst_map, DST_CAP));
	TEST_ASSERT_EQUAL_INT(-EINVAL, phase_join_init(&j, dst_x, dst_raw, NULL,
						       DST_CAP));
	TEST_ASSERT_EQUAL_INT(-EINVAL, phase_join_init(&j, dst_x, dst_raw,
						       dst_map, 0u));

	join_reset(&j);
	TEST_ASSERT_EQUAL_INT(-EINVAL, phase_join_add(&j, NULL, 10u));
	TEST_ASSERT_EQUAL_INT(-ENODATA, phase_join_meta(&j, &jm));
	TEST_ASSERT_EQUAL_INT(-EINVAL, phase_join_meta(NULL, &jm));

	/* A malformed record is refused by the codec's own guards, not by a
	 * second copy of them here. */
	la = make_window(rec_a, 1u, 0u, 512u, true, true);
	rec_a[0] ^= 0xFFu;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, phase_join_add(&j, rec_a, la));
	rec_a[0] ^= 0xFFu;
	TEST_ASSERT_EQUAL_INT(-EMSGSIZE, phase_join_add(&j, rec_a, la - 1u));

	/* An empty record cannot extend anything. */
	la = make_window(rec_a, 1u, 0u, 0u, true, true);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, phase_join_add(&j, rec_a, la));

	/* Every refusal an operator can see carries a sentence. */
	TEST_ASSERT_NOT_NULL(phase_join_strerror(0));
	TEST_ASSERT_NOT_NULL(phase_join_strerror(-EPROTO));
	TEST_ASSERT_NOT_NULL(phase_join_strerror(-EILSEQ));
	TEST_ASSERT_NOT_NULL(phase_join_strerror(-ERANGE));
	TEST_ASSERT_NOT_NULL(phase_join_strerror(-ENOENT));
	TEST_ASSERT_NOT_NULL(phase_join_strerror(-12345));
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_overlapping_windows_join_into_the_underlying_capture);
	RUN_TEST(test_exactly_abutting_windows_join);
	RUN_TEST(test_a_contained_window_verifies_and_adds_nothing);

	RUN_TEST(test_records_from_different_epochs_refuse_to_join);
	RUN_TEST(test_a_disagreeing_overlap_refuses_to_join);
	RUN_TEST(test_a_disagreeing_overlap_gap_bit_refuses_to_join);
	RUN_TEST(test_a_hole_between_records_refuses_to_join);
	RUN_TEST(test_out_of_order_records_refuse_to_join);
	RUN_TEST(test_a_record_without_identity_refuses_to_join);
	RUN_TEST(test_mismatched_tau0_or_series_refuses_to_join);
	RUN_TEST(test_a_join_larger_than_the_buffer_refuses);

	RUN_TEST(test_the_joined_gap_bitmap_is_the_union_at_absolute_indices);
	RUN_TEST(test_a_joined_record_round_trips_and_can_be_joined_again);
	RUN_TEST(test_an_index_that_does_not_advance_is_caught);

	RUN_TEST(test_argument_and_state_errors);

	return UNITY_END();
}
