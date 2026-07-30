/*
 * STS1000 "Meridian" — core/stats: bench phase-record codec.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See phase_rec.h for the contract and the wire layout.
 */

#include "stats/phase_rec.h"

#include <errno.h>
#include <string.h>

/*
 * Saturation bound for the float-nanoseconds -> int64-picoseconds conversion:
 * 1e15 ps = 1000 s, far beyond any phase error a PPS gate would ever pass, and
 * far inside INT64_MAX. Note this is deliberately ABOVE adev.h's
 * STATS_PHASE_MAX_PS (1e12) — a saturated sample is out of the estimator's
 * range and stats_adev() answers -ERANGE for it, which is the honest outcome.
 * Clipping into the estimator's range instead would fabricate a plausible
 * sample out of a broken one.
 */
#define PS_SAT_LIMIT INT64_C(1000000000000000)

/* ------------------------------------------------------------ byte helpers */

static void put_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint16_t get_u16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void put_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
	p[2] = (uint8_t)((v >> 16) & 0xFFu);
	p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint32_t get_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
	       ((uint32_t)p[3] << 24);
}

static void put_u64(uint8_t *p, uint64_t v)
{
	put_u32(&p[0], (uint32_t)(v & UINT32_C(0xFFFFFFFF)));
	put_u32(&p[4], (uint32_t)((v >> 32) & UINT32_C(0xFFFFFFFF)));
}

static uint64_t get_u64(const uint8_t *p)
{
	return (uint64_t)get_u32(&p[0]) | ((uint64_t)get_u32(&p[4]) << 32);
}

static void put_i64(uint8_t *p, int64_t v)
{
	put_u64(p, (uint64_t)v);
}

static int64_t get_i64(const uint8_t *p)
{
	return (int64_t)get_u64(p);
}

/* --------------------------------------------------------------- geometry */

static size_t gapmap_bytes(uint16_t n)
{
	return ((size_t)n + 7u) / 8u;
}

size_t phase_rec_size(uint16_t n)
{
	return (size_t)PHASE_REC_HDR_LEN + gapmap_bytes(n) +
	       ((size_t)n * sizeof(int64_t));
}

static size_t popcount_bits(const uint8_t *map, size_t nbits)
{
	size_t i;
	size_t c = 0u;

	for (i = 0u; i < nbits; i++) {
		if ((map[i >> 3] & (uint8_t)(1u << (i & 7u))) != 0u) {
			c++;
		}
	}
	return c;
}

/* ---------------------------------------------------------------- encoding */

int phase_rec_encode(const phase_rec_meta_t *m, const int64_t *x_ps,
		     const uint8_t *gapmap, uint8_t *buf, size_t cap,
		     size_t *out_len)
{
	size_t gb;
	size_t need;
	size_t i;
	uint8_t *p;

	if ((m == NULL) || (buf == NULL) || (out_len == NULL)) {
		return -EINVAL;
	}
	if ((m->n > 0u) && (x_ps == NULL)) {
		return -EINVAL;
	}
	if (m->tau0_ns == 0u) {
		return -EINVAL;
	}
	if (m->gaps > (uint32_t)m->n) {
		return -EINVAL;
	}
	/*
	 * A gap count with no map to back it is not encodable: the decoder
	 * cross-checks the two, and a consumer that wants the longest clean run
	 * has nothing to compute it from. Declaring "clean" without a map is
	 * fine — an all-zero map is written for it.
	 */
	if ((m->gaps > 0u) && (gapmap == NULL)) {
		return -EINVAL;
	}

	gb = gapmap_bytes(m->n);
	need = phase_rec_size(m->n);
	if (cap < need) {
		return -EMSGSIZE;
	}

	p = buf;
	put_u32(&p[0], PHASE_REC_MAGIC);
	p[4] = (uint8_t)PHASE_REC_VER;
	p[5] = m->flags;
	put_u16(&p[6], m->n);
	put_u32(&p[8], m->tau0_ns);
	put_u32(&p[12], m->gaps);
	put_u64(&p[16], m->mono_ms);

	p = &buf[PHASE_REC_HDR_LEN];
	if (gb > 0u) {
		if (gapmap != NULL) {
			memcpy(p, gapmap, gb);
			/*
			 * Clear the bits past sample n-1 in the final byte. The
			 * producer's map is sized by its ring, not by n, so a
			 * stale bit above the sample count would fail the
			 * decoder's population-count cross-check on a record
			 * that is in fact well formed.
			 */
			if (((size_t)m->n & 7u) != 0u) {
				uint8_t keep =
					(uint8_t)((1u << ((size_t)m->n & 7u)) -
						  1u);
				p[gb - 1u] &= keep;
			}
		} else {
			memset(p, 0, gb);
		}
	}

	p = &buf[PHASE_REC_HDR_LEN + gb];
	for (i = 0u; i < (size_t)m->n; i++) {
		put_i64(&p[i * sizeof(int64_t)], x_ps[i]);
	}

	*out_len = need;
	return 0;
}

/* ---------------------------------------------------------------- decoding */

int phase_rec_decode_hdr(const uint8_t *buf, size_t len, phase_rec_meta_t *out)
{
	phase_rec_meta_t m;
	size_t need;

	if ((buf == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (len < (size_t)PHASE_REC_HDR_LEN) {
		return -EMSGSIZE;
	}
	if (get_u32(&buf[0]) != PHASE_REC_MAGIC) {
		return -EBADMSG;
	}

	m.ver = buf[4];
	m.flags = buf[5];
	m.n = get_u16(&buf[6]);
	m.tau0_ns = get_u32(&buf[8]);
	m.gaps = get_u32(&buf[12]);
	m.mono_ms = get_u64(&buf[16]);

	if (m.ver != (uint8_t)PHASE_REC_VER) {
		return -EBADMSG;
	}
	if (m.tau0_ns == 0u) {
		return -EBADMSG;
	}
	if (m.gaps > (uint32_t)m.n) {
		return -EBADMSG;
	}

	need = phase_rec_size(m.n);
	if (len < need) {
		return -EMSGSIZE;
	}

	/*
	 * The header's count and the bitmap must agree. This is the whole
	 * safety property of the format: a consumer gates on `gaps`, so a
	 * record whose map says otherwise must be rejected rather than analysed
	 * as clean.
	 */
	if (popcount_bits(&buf[PHASE_REC_HDR_LEN], (size_t)m.n) !=
	    (size_t)m.gaps) {
		return -EBADMSG;
	}

	*out = m;
	return 0;
}

int phase_rec_decode_samples(const uint8_t *buf, size_t len, int64_t *out_x,
			     size_t cap, size_t *out_n)
{
	phase_rec_meta_t m;
	const uint8_t *p;
	size_t i;
	int rc;

	if ((out_x == NULL) || (out_n == NULL)) {
		return -EINVAL;
	}
	rc = phase_rec_decode_hdr(buf, len, &m);
	if (rc != 0) {
		return rc;
	}
	if (cap < (size_t)m.n) {
		return -EMSGSIZE;
	}

	p = &buf[(size_t)PHASE_REC_HDR_LEN + gapmap_bytes(m.n)];
	for (i = 0u; i < (size_t)m.n; i++) {
		out_x[i] = get_i64(&p[i * sizeof(int64_t)]);
	}

	*out_n = (size_t)m.n;
	return 0;
}

bool phase_rec_gap_at(const uint8_t *buf, size_t len, size_t i)
{
	phase_rec_meta_t m;

	if (phase_rec_decode_hdr(buf, len, &m) != 0) {
		return false;
	}
	if (i >= (size_t)m.n) {
		return false;
	}
	return (buf[(size_t)PHASE_REC_HDR_LEN + (i >> 3)] &
		(uint8_t)(1u << (i & 7u))) != 0u;
}

size_t phase_rec_longest_clean(const uint8_t *buf, size_t len,
			       size_t *out_start)
{
	phase_rec_meta_t m;
	const uint8_t *map;
	size_t best = 0u;
	size_t best_start = 0u;
	size_t run = 0u;
	size_t run_start = 0u;
	size_t i;

	if (phase_rec_decode_hdr(buf, len, &m) != 0) {
		return 0u;
	}

	map = &buf[PHASE_REC_HDR_LEN];
	for (i = 0u; i < (size_t)m.n; i++) {
		bool gap = (map[i >> 3] & (uint8_t)(1u << (i & 7u))) != 0u;

		if (gap) {
			/*
			 * The interval ending at i is wrong, so i cannot extend
			 * the current run — but it is the first sample of the
			 * next one, whose interval to i+1 is a full tau0 again.
			 */
			run = 1u;
			run_start = i;
		} else {
			run++;
			if (run == 1u) {
				run_start = i;
			}
		}
		if (run > best) {
			best = run;
			best_start = run_start;
		}
	}

	if (out_start != NULL) {
		*out_start = best_start;
	}
	return best;
}

/* ------------------------------------------------------------- conversion */

int64_t phase_rec_ns_f_to_ps(float ns)
{
	double ps;

	/* NaN fails every comparison, so test for it explicitly rather than
	 * letting it fall through to a saturating branch that would turn "no
	 * measurement" into a 1000-second phase error. */
	if (!(ns == ns)) {
		return 0;
	}

	ps = (double)ns * 1000.0;
	if (ps >= (double)PS_SAT_LIMIT) {
		return PS_SAT_LIMIT;
	}
	if (ps <= -(double)PS_SAT_LIMIT) {
		return -PS_SAT_LIMIT;
	}
	return (int64_t)((ps < 0.0) ? (ps - 0.5) : (ps + 0.5));
}
