/*
 * STS1000 "Meridian" — core/stats: joining successive phase records.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See phase_join.h for the contract and the reasoning.
 */

#include "stats/phase_join.h"

#include <errno.h>
#include <string.h>

/* ------------------------------------------------------------ bit helpers */

static bool bit_get(const uint8_t *map, size_t i)
{
	return (map[i >> 3] & (uint8_t)(1u << (i & 7u))) != 0u;
}

static void bit_set(uint8_t *map, size_t i)
{
	map[i >> 3] |= (uint8_t)(1u << (i & 7u));
}

/* --------------------------------------------------------------- lifetime */

int phase_join_init(phase_join_t *j, int64_t *x, int64_t *x_raw,
		    uint8_t *gapmap, size_t cap)
{
	if ((j == NULL) || (x == NULL) || (gapmap == NULL) || (cap == 0u)) {
		return -EINVAL;
	}

	memset(j, 0, sizeof(*j));
	j->x = x;
	j->x_raw = x_raw;
	j->gapmap = gapmap;
	j->cap = cap;
	memset(gapmap, 0, (cap + 7u) / 8u);
	return 0;
}

/* ------------------------------------------------------------------- join */

/*
 * Everything that must be true before a single byte of @p buf is spliced in.
 * Split out so that the accumulator is provably untouched on refusal: this
 * function only reads, and the mutation below runs only after it has returned
 * 0 for every check.
 */
static int join_check(const phase_join_t *j, const phase_rec_meta_t *m,
		      const uint8_t *buf, uint64_t *out_end)
{
	uint64_t rec_end;
	uint64_t span_end;

	if (!m->has_ident) {
		return -ENOENT;
	}
	if (m->n == 0u) {
		return -EBADMSG;
	}
	if (((m->flags & PHASE_REC_F_RAW) != 0u) && (j->x_raw == NULL)) {
		return -ENOTSUP;
	}

	rec_end = m->seq0 + (uint64_t)m->n;
	*out_end = rec_end;

	if (!j->started) {
		return ((uint64_t)m->n > (uint64_t)j->cap) ? -ENOSPC : 0;
	}

	if (m->epoch != j->epoch) {
		return -EPROTO;
	}
	if (m->tau0_ns != j->tau0_ns) {
		return -ENOTSUP;
	}
	if (j->have_raw != ((m->flags & PHASE_REC_F_RAW) != 0u)) {
		return -ENOTSUP;
	}
	if (m->seq0 < j->seq0) {
		return -EALREADY;
	}

	span_end = j->seq0 + (uint64_t)j->n;
	if (m->seq0 > span_end) {
		return -ERANGE;
	}
	if (rec_end > (j->seq0 + (uint64_t)j->cap)) {
		return -ENOSPC;
	}

	/*
	 * The overlap is the proof, so it is checked in full — both series and
	 * the gap bits. These samples came out of the same ring slots on the
	 * same device within the same epoch, so they are bit-identical or the
	 * premise is false.
	 */
	{
		const uint8_t *rx = phase_rec_series(buf, m, 0u);
		const uint8_t *rr = phase_rec_series(buf, m, 1u);
		const uint8_t *rmap = &buf[PHASE_REC_HDR_LEN];
		uint64_t lo = m->seq0;
		uint64_t hi = (rec_end < span_end) ? rec_end : span_end;
		uint64_t s;

		if (rx == NULL) {
			return -EBADMSG;
		}
		if (j->have_raw && (rr == NULL)) {
			return -EBADMSG;
		}

		for (s = lo; s < hi; s++) {
			size_t di = (size_t)(s - j->seq0);
			size_t ri = (size_t)(s - m->seq0);

			if (j->x[di] != phase_rec_sample(rx, ri)) {
				return -EILSEQ;
			}
			if (j->have_raw &&
			    (j->x_raw[di] != phase_rec_sample(rr, ri))) {
				return -EILSEQ;
			}
			if (bit_get(j->gapmap, di) != bit_get(rmap, ri)) {
				return -EILSEQ;
			}
		}
	}

	return 0;
}

int phase_join_add(phase_join_t *j, const uint8_t *buf, size_t len)
{
	phase_rec_meta_t m;
	const uint8_t *rx;
	const uint8_t *rr;
	const uint8_t *rmap;
	uint64_t rec_end = 0u;
	uint64_t s;
	int rc;

	if ((j == NULL) || (buf == NULL)) {
		return -EINVAL;
	}
	if ((j->x == NULL) || (j->gapmap == NULL) || (j->cap == 0u)) {
		return -EINVAL;
	}

	rc = phase_rec_decode_hdr(buf, len, &m);
	if (rc != 0) {
		return rc;
	}

	rc = join_check(j, &m, buf, &rec_end);
	if (rc != 0) {
		return rc;
	}

	if (!j->started) {
		j->started = true;
		j->epoch = m.epoch;
		j->seq0 = m.seq0;
		j->tau0_ns = m.tau0_ns;
		j->mono_ms = m.mono_ms;
		j->have_raw = ((m.flags & PHASE_REC_F_RAW) != 0u);
		j->full = ((m.flags & PHASE_REC_F_FULL) != 0u);
		j->n = 0u;
	} else {
		uint64_t span_end = j->seq0 + (uint64_t)j->n;
		uint64_t ov_hi = (rec_end < span_end) ? rec_end : span_end;

		if (ov_hi > m.seq0) {
			j->overlap += (size_t)(ov_hi - m.seq0);
		}
	}

	rx = phase_rec_series(buf, &m, 0u);
	rr = phase_rec_series(buf, &m, 1u);
	rmap = &buf[PHASE_REC_HDR_LEN];

	/*
	 * Samples: only the part past the accumulated end is new. The overlap
	 * has already been proved identical, so re-copying it would change
	 * nothing and re-checking it here would be checking the copy.
	 */
	for (s = j->seq0 + (uint64_t)j->n; s < rec_end; s++) {
		size_t di = (size_t)(s - j->seq0);
		size_t ri = (size_t)(s - m.seq0);

		j->x[di] = phase_rec_sample(rx, ri);
		if (j->have_raw) {
			j->x_raw[di] = phase_rec_sample(rr, ri);
		}
	}

	/*
	 * Gap bits: the UNION over every sample this record carries, not just
	 * the new tail, at ABSOLUTE index. Written bit by bit because the
	 * record's origin within the join is `m.seq0 - j->seq0`, which is a
	 * multiple of 8 only by accident — the ring slides one sample a second,
	 * so a poll two minutes later starts 120 samples along. A byte-wise
	 * copy would displace every gap in this part by up to seven samples.
	 *
	 * `gaps` counts 0 -> 1 transitions, so it stays the population count of
	 * the union no matter how many parts assert the same bit. The joined
	 * record therefore satisfies the header/bitmap cross-check
	 * phase_rec_decode_hdr() applies to a device record.
	 */
	for (s = m.seq0; s < rec_end; s++) {
		size_t di = (size_t)(s - j->seq0);
		size_t ri = (size_t)(s - m.seq0);

		if (!bit_get(rmap, ri)) {
			continue;
		}
		if (!bit_get(j->gapmap, di)) {
			bit_set(j->gapmap, di);
			j->gaps++;
		}
	}

	if (rec_end > (j->seq0 + (uint64_t)j->n)) {
		j->n = (size_t)(rec_end - j->seq0);
	}
	j->parts++;
	return 0;
}

int phase_join_meta(const phase_join_t *j, phase_rec_meta_t *out)
{
	if ((j == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (!j->started || (j->n == 0u)) {
		return -ENODATA;
	}
	if (j->n > (size_t)UINT16_MAX) {
		return -EOVERFLOW;
	}

	memset(out, 0, sizeof(*out));
	out->ver = PHASE_REC_VER;
	out->flags = j->full ? (uint8_t)PHASE_REC_F_FULL : 0u;
	out->n = (uint16_t)j->n;
	out->tau0_ns = j->tau0_ns;
	out->gaps = (uint32_t)j->gaps;
	out->mono_ms = j->mono_ms;
	out->has_ident = true;
	out->epoch = j->epoch;
	out->seq0 = j->seq0;
	return 0;
}

const char *phase_join_strerror(int rc)
{
	switch (rc) {
	case 0:
		return "joined";
	case -EINVAL:
		return "bad argument, or the joiner was not initialised";
	case -EBADMSG:
		return "not a valid phase record, or it carries no samples";
	case -EMSGSIZE:
		return "the record is shorter than its header declares";
	case -ENOENT:
		return "the record carries no sample identity (pre-v3 export, "
		       "or a device that could not draw a capture epoch), so "
		       "there is nothing to prove it abuts anything";
	case -EPROTO:
		return "different capture epoch — the records straddle a ring "
		       "reset, a holdover entry or a reboot, and describe two "
		       "unrelated series";
	case -ENOTSUP:
		return "the records disagree about tau0 or about carrying the "
		       "uncorrected series";
	case -EALREADY:
		return "out of order — this record is older than the samples "
		       "already joined; add records oldest first";
	case -ERANGE:
		return "not contiguous — samples are missing between this "
		       "record and the ones already joined, and a joined "
		       "series cannot express how long the hole was";
	case -EILSEQ:
		return "the overlapping samples disagree — the epoch is being "
		       "reused or one of the records is corrupt";
	case -ENOSPC:
		return "the joined series is larger than the buffer allows";
	case -EOVERFLOW:
		return "the join holds more samples than a record's 16-bit "
		       "count can express";
	case -ENODATA:
		return "nothing has been joined";
	default:
		return "refused";
	}
}
