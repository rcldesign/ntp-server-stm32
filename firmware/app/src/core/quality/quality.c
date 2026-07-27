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

	seq = atomic_load_explicit(&qs->seq, memory_order_relaxed);

	/* Odd sequence: a reader that sees this discards whatever it copied.
	 * The release fence keeps the payload writes below from being hoisted
	 * above this store; the single-writer contract makes the load/store of
	 * seq itself race-free without stronger ordering. */
	atomic_store_explicit(&qs->seq, seq + 1u, memory_order_relaxed);
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

	/* Release so a reader that observes this even value also observes every
	 * payload write above it. */
	atomic_store_explicit(&qs->seq, seq + 2u, memory_order_release);

	return 0;
}

int quality_snapshot(const quality_state_t *qs, quality_block_t *out)
{
	unsigned int attempt;

	if (qs == NULL || out == NULL) {
		return -EINVAL;
	}

	for (attempt = 0u; attempt < QUALITY_SNAPSHOT_RETRIES; attempt++) {
		/* Acquire so the payload read below observes the writer's
		 * release store from a completed publication. */
		uint32_t s1 = atomic_load_explicit(&qs->seq, memory_order_acquire);
		uint32_t s2;

		if ((s1 & 1u) != 0u) {
			/* Writer mid-update; do not even copy. */
			continue;
		}

		memcpy(out, &qs->blk, sizeof(*out));

		/* Acquire fence keeps the payload read above from sinking past
		 * the second sequence load, so a write that started during the
		 * copy is always detected. */
		atomic_thread_fence(memory_order_acquire);

		if (qs->race_hook != NULL) {
			qs->race_hook(qs->hook_ctx);
		}

		s2 = atomic_load_explicit(&qs->seq, memory_order_relaxed);
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
	uint64_t q;

	if (ns <= 0) {
		return 0u;
	}
	if (ns >= (int64_t)NTP_SHORT_MAX_S * (int64_t)NS_PER_S) {
		return UINT32_MAX;
	}

	/* ns * 2^16 / 1e9, rounded to nearest. The bound above keeps the
	 * multiply inside uint64: 65536e9 * 65536 = 4.3e18 < 1.8e19. */
	scaled = (uint64_t)ns * NTP_SHORT_PER_S + (NS_PER_S / 2u);
	q = scaled / NS_PER_S;

	/* Round-to-nearest can carry the quotient up to 2^32 for an ns just
	 * below the whole-second cap (e.g. 65535.9999995 s), which would wrap
	 * to 0 on the cast. Saturate in uint64 before narrowing. */
	return (q > UINT32_MAX) ? UINT32_MAX : (uint32_t)q;
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
		/* Positive root of 0.5*a*t^2 + rate*t - budget = 0. Written as
		 * 2b/(rate + sqrt(disc)) rather than (sqrt(disc) - rate)/a: the
		 * two forms are algebraically identical, but when rate dominates
		 * (small aging, large drift) sqrt(disc) ~= rate and the textbook
		 * subtraction loses most of its significant figures, while the
		 * conjugate form adds two like-signed quantities and stays
		 * accurate. */
		float disc = rate * rate + 2.0f * aging * budget;

		return (2.0f * budget) / (rate + quality_sqrtf(disc));
	}
	if (rate > 0.0f) {
		return budget / rate;
	}

	return INFINITY;
}

/* --------------------------------------------------------------- leap smear */

uint32_t quality_smear_window_clamp(uint32_t window_s)
{
	if (window_s == 0u) {
		return 0u;
	}
	if (window_s < QUALITY_SMEAR_WINDOW_MIN_S) {
		return QUALITY_SMEAR_WINDOW_MIN_S;
	}
	if (window_s > QUALITY_SMEAR_WINDOW_MAX_S) {
		return QUALITY_SMEAR_WINDOW_MAX_S;
	}
	return window_s;
}

int quality_leap_smear(const quality_block_t *b, uint32_t window_s,
		       uint64_t now_tai_ns, quality_smear_t *out)
{
	uint64_t now_s;
	uint64_t start_s;
	uint64_t elapsed_ns;
	uint64_t offset_ns;
	uint32_t w;

	if (out == NULL) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));
	if (b == NULL) {
		return -EINVAL;
	}

	w = quality_smear_window_clamp(window_s);
	out->window_s = w;

	if (w == 0u || b->leap_pending == 0 || now_tai_ns == 0u) {
		return 0;
	}
	/* A receiver reporting an event inside the first W seconds of the TAI
	 * epoch is reporting nonsense; refusing it here is what keeps the
	 * `leap_at_tai_s - w` below from wrapping into a colossal window. */
	if (b->leap_at_tai_s < (uint64_t)w) {
		return 0;
	}

	now_s = now_tai_ns / NS_PER_S;
	if (now_s >= b->leap_at_tai_s) {
		/* The event has fired (or the block is stale and still carries a
		 * pending flag for a past event). Either way the ramp is over and
		 * leap_current_s is the authority again. */
		return 0;
	}
	if ((b->leap_at_tai_s - now_s) > (uint64_t)w) {
		return 0; /* announced, but the ramp has not started */
	}

	start_s = b->leap_at_tai_s - (uint64_t)w;

	/*
	 * Elapsed time is accumulated as whole seconds plus the sub-second
	 * remainder rather than as `now_tai_ns - start_s * NS_PER_S`, because
	 * the latter multiplies an unvalidated receiver-supplied second count
	 * by 1e9. Here every term is bounded: `now_s - start_s` is under w
	 * (≤ 86400) by the two tests above, and the remainder is under 1e9.
	 */
	elapsed_ns = (now_s - start_s) * NS_PER_S + (now_tai_ns % NS_PER_S);

	/*
	 * The whole ramp, in one exact integer division.
	 *
	 * offset = 1e9 * elapsed_ns / (w * 1e9) = elapsed_ns / w. Non-decreasing
	 * in elapsed_ns (floor of a monotone linear function), and bounded by
	 * (w*1e9 - 1)/w = 1e9 - 1, so the correction reaches a full second only
	 * in the limit — at which instant the window closes and leap_current_s
	 * has moved. The two meet with no gap: at elapsed = w*1e9 - 1 ns the
	 * served time is (TAI + 1 ns) - leap_current_s - (1e9 - 1) ns, which is
	 * exactly the post-leap value one nanosecond later.
	 */
	offset_ns = elapsed_ns / (uint64_t)w;

	out->active = true;
	out->direction = (b->leap_pending > 0) ? (int8_t)1 : (int8_t)-1;
	out->offset_ns = (b->leap_pending > 0) ? (int32_t)offset_ns
					       : -(int32_t)offset_ns;
	out->elapsed_s = (uint32_t)(now_s - start_s);
	out->remaining_s = (uint32_t)(b->leap_at_tai_s - now_s);
	return 0;
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
