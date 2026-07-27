/*
 * STS1000 "Meridian" — core/quality implementation.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See quality.h for the contract, the seqlock rationale and the caveat.
 */

#include "quality/quality.h"

#include <errno.h>
#include <math.h>
#include <stdatomic.h>
#include <string.h>

/* NTP short format: 16.16 unsigned fixed-point seconds (RFC 5905 §6). */
#define NTP_SHORT_PER_S 65536u
#define NS_PER_S 1000000000u

/* Largest whole second representable in NTP short format. */
#define NTP_SHORT_MAX_S 65536

/* --------------------------------------------------------------- lifecycle */

void quality_block_init(quality_block_t *b)
{
	if (b == NULL) {
		return;
	}

	memset(b, 0, sizeof(*b));
	b->ver = (uint16_t)QUALITY_BLOCK_VER;
	b->size = (uint16_t)sizeof(*b);
	b->stratum = (uint8_t)QUALITY_STRATUM_UNSYNC;
	b->lock_state = (uint8_t)QUALITY_LOCK_UNKNOWN;
	b->active_ref = (uint8_t)QUALITY_REF_NONE;
	b->gnss_fix = (uint8_t)QUALITY_GNSS_NO_FIX;
	b->holdover_t_demote_s = UINT32_MAX;
}

int quality_state_init(quality_state_t *qs)
{
	if (qs == NULL) {
		return -EINVAL;
	}

	qs->seq = 0u;
	qs->race_hook = NULL;
	qs->hook_ctx = NULL;
	quality_block_init(&qs->blk);
	return 0;
}

/* ------------------------------------------------------------ publish/read */

int quality_publish(quality_state_t *qs, const quality_block_t *blk)
{
	uint32_t seq;

	if (qs == NULL || blk == NULL) {
		return -EINVAL;
	}

	seq = qs->seq;

	/* Odd sequence: a reader that sees this discards whatever it copied. */
	qs->seq = seq + 1u;
	atomic_thread_fence(memory_order_release);

	memcpy(&qs->blk, blk, sizeof(*blk));

	/* Stamp the identity fields here rather than trusting the caller, so a
	 * block built from a memset() scratch buffer is still well-formed. The
	 * tick is the slot's own publication ordinal (the sequence advances by
	 * two per publish), which is what lets a consumer notice a skipped
	 * update without the writer having to keep a second counter. */
	qs->blk.ver = (uint16_t)QUALITY_BLOCK_VER;
	qs->blk.size = (uint16_t)sizeof(*blk);
	qs->blk.tick = (seq / 2u) + 1u;

	atomic_thread_fence(memory_order_release);
	qs->seq = seq + 2u;

	return 0;
}

int quality_snapshot(const quality_state_t *qs, quality_block_t *out)
{
	unsigned int attempt;

	if (qs == NULL || out == NULL) {
		return -EINVAL;
	}

	for (attempt = 0u; attempt < QUALITY_SNAPSHOT_RETRIES; attempt++) {
		uint32_t s1 = qs->seq;
		uint32_t s2;

		if ((s1 & 1u) != 0u) {
			/* Writer mid-update; do not even copy. */
			continue;
		}

		atomic_thread_fence(memory_order_acquire);
		memcpy(out, &qs->blk, sizeof(*out));
		atomic_thread_fence(memory_order_acquire);

		if (qs->race_hook != NULL) {
			qs->race_hook(qs->hook_ctx);
		}

		s2 = qs->seq;
		if (s1 == s2) {
			return 0;
		}
	}

	return -EAGAIN;
}

/* ------------------------------------------------------------- conversions */

uint32_t quality_ntp_short_from_ns(int64_t ns)
{
	uint64_t scaled;

	if (ns <= 0) {
		return 0u;
	}
	if (ns >= (int64_t)NTP_SHORT_MAX_S * (int64_t)NS_PER_S) {
		return UINT32_MAX;
	}

	/* ns * 2^16 / 1e9, rounded to nearest. The bound above keeps the
	 * multiply inside uint64: 65536e9 * 65536 = 4.3e18 < 1.8e19. */
	scaled = (uint64_t)ns * NTP_SHORT_PER_S + (NS_PER_S / 2u);
	return (uint32_t)(scaled / NS_PER_S);
}

int64_t quality_ns_from_ntp_short(uint32_t q16)
{
	/* q16 * 1e9 / 2^16, rounded to nearest. Max 4.295e9 * 1e9 = 4.3e18. */
	uint64_t scaled = (uint64_t)q16 * NS_PER_S + (NTP_SHORT_PER_S / 2u);

	return (int64_t)(scaled / NTP_SHORT_PER_S);
}

uint32_t quality_refid(char a, char b, char c, char d)
{
	return ((uint32_t)(uint8_t)a << 24) | ((uint32_t)(uint8_t)b << 16) |
	       ((uint32_t)(uint8_t)c << 8) | (uint32_t)(uint8_t)d;
}

/* --------------------------------------------------------- holdover growth */

/* Ageing must never be negative: a model that predicts an improving oscillator
 * would under-report dispersion to clients. Clamped in one place. */
static float holdover_aging(const quality_holdover_model_t *m)
{
	return (m->aging_ns_per_s2 > 0.0f) ? m->aging_ns_per_s2 : 0.0f;
}

/* Linear drift term including the temperature contribution. */
static float holdover_rate(const quality_holdover_model_t *m, float dt_c)
{
	float mag = (dt_c < 0.0f) ? -dt_c : dt_c;

	return m->drift_ns_per_s + m->temp_ns_per_s_per_c * mag;
}

float quality_holdover_err_ns(const quality_holdover_model_t *m, float t_s,
			      float dt_c)
{
	float rate;
	float aging;
	float err;

	if (m == NULL) {
		return 0.0f;
	}
	/* Written as !(t_s > 0) so NaN takes the same path as a negative time. */
	if (!(t_s > 0.0f)) {
		return (m->base_ns > 0.0f) ? m->base_ns : 0.0f;
	}

	rate = holdover_rate(m, dt_c);
	aging = holdover_aging(m);
	err = m->base_ns + rate * t_s + 0.5f * aging * t_s * t_s;

	return (err > 0.0f) ? err : 0.0f;
}

float quality_holdover_time_to_ns(const quality_holdover_model_t *m, float dt_c,
				  float threshold_ns)
{
	float budget;
	float rate;
	float aging;

	if (m == NULL) {
		return INFINITY;
	}

	budget = threshold_ns - m->base_ns;
	if (!(budget > 0.0f)) {
		/* Already at or over the threshold the moment holdover starts
		 * (or a NaN threshold, which must not read as "plenty of time"). */
		return 0.0f;
	}

	rate = holdover_rate(m, dt_c);
	if (rate < 0.0f) {
		rate = 0.0f;
	}
	aging = holdover_aging(m);

	if (aging > 0.0f) {
		/* Positive root of 0.5*a*t^2 + rate*t - budget = 0. */
		float disc = rate * rate + 2.0f * aging * budget;

		return (quality_sqrtf(disc) - rate) / aging;
	}
	if (rate > 0.0f) {
		return budget / rate;
	}

	return INFINITY;
}

/* ------------------------------------------------------------------- misc */

float quality_sqrtf(float x)
{
	uint32_t bits;
	float y;
	float scale = 1.0f;

	/* Written as !(x > 0) so NaN and -0.0 land here too. */
	if (!(x > 0.0f)) {
		return 0.0f;
	}
	if (isinf(x)) {
		return x;
	}

	/* Subnormals defeat the exponent-halving seed, so scale them into the
	 * normal range first and undo the scale at the end. 2^64 lifts even
	 * FLT_TRUE_MIN (1.4e-45) to 2.6e-26. */
	if (x < 1.0e-30f) {
		x *= 0x1p64f;
		scale = 0x1p-32f;
	}

	/* Seed: halving the biased exponent approximates the square root to
	 * about 3.5 %. 0x1FBD1DF5 is the offset that minimises the worst-case
	 * relative error of that approximation. */
	memcpy(&bits, &x, sizeof(bits));
	bits = 0x1FBD1DF5u + (bits >> 1);
	memcpy(&y, &bits, sizeof(y));

	/* Newton–Raphson: each step squares the relative error.
	 * 3.5e-2 -> 6e-4 -> 2e-7 -> ulp. The fourth step is margin. */
	y = 0.5f * (y + x / y);
	y = 0.5f * (y + x / y);
	y = 0.5f * (y + x / y);
	y = 0.5f * (y + x / y);

	return y * scale;
}

const char *quality_lock_state_name(uint8_t state)
{
	static const char *const names[QUALITY_LOCK__COUNT] = {
		"unknown", "acquiring", "locking", "locked",
		"holdover", "recovering", "parked",
	};

	if (state >= (uint8_t)QUALITY_LOCK__COUNT) {
		return "invalid";
	}
	return names[state];
}

const char *quality_ref_name(uint8_t ref)
{
	static const char *const names[QUALITY_REF__COUNT] = {
		"none", "ocxo", "rb", "extref",
	};

	if (ref >= (uint8_t)QUALITY_REF__COUNT) {
		return "invalid";
	}
	return names[ref];
}
