/*
 * STS1000 "Meridian" — publication of the power sequencer's state.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * pwrseq_exec.c owns a pwrseq_ctx_t that is stepped from the housekeeping
 * thread at 4 Hz and mutated from nowhere else — every operator recovery action
 * posts an atomic bit and is applied on that same thread, which is the mutual
 * exclusion pwrseq.h demands. That makes the sequencer a textbook single-writer
 * publisher, and this header is the publication.
 *
 * WHY A SEQLOCK AND NOT A MUTEX
 * -----------------------------
 * The reader is the console thread inside the Maintenance Protocol engine lock,
 * whose whole tick budget is accounted for in mp_glue.h (STS_MP_TICK_LOCK_MS 50
 * x STS_MP_TICK_MISS_MAX 5 inside MP_TICK_MAX_MS 2000). The writer is the
 * lowest-priority application thread on the board (housekeeping, priority 14),
 * and it must never queue behind a maintenance request — that rule is why this
 * area publishes snapshots rather than exposing its state directly. A mutex
 * would put each side on the other's critical path in one direction or the
 * other; a seqlock puts neither on the other's. The writer never waits at all,
 * and the reader's worst case is a bounded number of re-copies.
 *
 * The mechanism is core/quality's, which publishes the timing block from the
 * discipline thread at 1 Hz to exactly the same set of management readers
 * (quality.c quality_publish/quality_snapshot). It is reproduced here rather
 * than reused because quality_state_t's payload is a quality_block_t; the
 * ordering discipline — odd sequence brackets the payload, release on the way
 * out, acquire fence on the reader's way past the copy — is deliberately
 * identical, so a reader of one is a reader of the other.
 *
 * Zephyr-free by construction, like sts_rbguard.h and sts_super_policy.h beside
 * it, so that the mapping this file performs — which is one long field-by-field
 * copy, and therefore exactly the kind of code that silently loses a field — is
 * exercised by tests/host/test_mp_telem.c against a real pwrseq_ctx_t instead
 * of by inspection.
 */

#ifndef STS1000_ZEPHYR_PLATFORM_STS_PWRSEQ_PUB_H_
#define STS1000_ZEPHYR_PLATFORM_STS_PWRSEQ_PUB_H_

#include <errno.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pwrseq/pwrseq.h"
#include "zephyr/sts_app.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Snapshot attempts before sts_pwrseq_pub_read() gives up with -EAGAIN. */
#define STS_PWRSEQ_PUB_RETRIES 8u

/**
 * The publication slot.
 *
 * All members private; use the functions. Zero-initialised static storage is a
 * valid empty slot (seq 0, payload zero, `started` false), so no init call is
 * needed and a read that lands before the first publish reports "not started"
 * rather than a torn struct.
 */
typedef struct {
	/** Even = stable, odd = write in progress. C11 atomic so the writer and
	 *  a concurrent reader are a defined operation rather than a data race,
	 *  and so the reader's two loads cannot be folded into one. */
	_Atomic uint32_t seq;
	sts_pwrseq_snap_t slot;
} sts_pwrseq_pub_t;

/**
 * Map core/pwrseq's flattened status onto the DECIDED half of @p out.
 *
 * Zeroes @p out first: a snapshot assembled from a status read that failed must
 * not carry the previous tick's beliefs.
 */
static inline void sts_pwrseq_snap_from_status(sts_pwrseq_snap_t *out,
					       const pwrseq_status_t *st)
{
	if (out == NULL) {
		return;
	}

	memset(out, 0, sizeof(*out));
	if (st == NULL) {
		return;
	}

	out->started = true;
	out->stage = (uint8_t)st->stage;
	out->stage_entered_ms = st->stage_entered_ms;
	out->retries = st->retries;
	out->shed = (uint8_t)st->shed;
	out->alarms = st->alarms;
	out->halted = st->halted;
	out->rb_deferred = st->rb_deferred;
	out->rb_enabled = st->rb_enabled;
	out->rb_gated = st->rb_gated;
	out->rb_locked = st->rb_locked;
	out->rb_auto_retries = st->rb_auto_retries;
	out->ov_latched = st->ov_latched;
	out->wdt_armed = st->wdt_armed;
	out->relay_eligible = st->relay_eligible;
	out->display_on = st->display_on;
	out->panel_led_on = st->panel_led_on;
	out->gps_on = st->gps_on;
	out->ant_bias_on = st->ant_bias_on;
	out->disc_started = st->disc_started;
	out->phy_released = st->phy_released;
	out->pfi_seen = st->pfi_seen;
	out->pfi_expected = st->pfi_expected;
}

/**
 * Map the tick's own inputs onto the OBSERVED half of @p out.
 *
 * Additive: it does not clear the decided half, so the call order is
 * sts_pwrseq_snap_from_status() then this.
 *
 * @param in            The pwrseq_in_t just handed to pwrseq_step().
 * @param extref_hz     EXTREF_MON's measured frequency (sts_extref_mon_read()).
 * @param extref_valid  Whether that measurement is fresh and trustworthy.
 *
 * pwrseq_in_t carries only the in-band VERDICT, because that is all the stage
 * machine needs to decide with. The measurement itself is what a technician
 * needs to see — "the sequencer refused the external reference" and "the
 * external reference reads 9.9994 MHz" are different sentences — so it is
 * passed alongside rather than recovered from the verdict, which cannot be
 * done.
 */
static inline void sts_pwrseq_snap_observe(sts_pwrseq_snap_t *out,
					   const pwrseq_in_t *in,
					   uint32_t extref_hz,
					   bool extref_valid)
{
	if ((out == NULL) || (in == NULL)) {
		return;
	}

	out->rb_wanted = in->rb_wanted;
	out->rb_lock_pin = in->rb_lock;
	out->rb_ov_det = in->rb_ov_det;
	out->extref_hz = extref_hz;
	out->extref_valid = extref_valid;
	out->extref_in_band = in->extref_in_band;
	out->tick_mono_ms = in->mono_ms;
}

/**
 * Publish @p s. Single writer only — the housekeeping thread's sequencer pass.
 *
 * Never blocks and never fails: two relaxed stores and a copy between them.
 */
static inline void sts_pwrseq_pub_publish(sts_pwrseq_pub_t *p,
					  const sts_pwrseq_snap_t *s)
{
	uint32_t seq;

	if ((p == NULL) || (s == NULL)) {
		return;
	}

	seq = atomic_load_explicit(&p->seq, memory_order_relaxed);

	/* Odd sequence: a reader that sees this discards whatever it copied.
	 * The release fence keeps the payload writes below from being hoisted
	 * above the store; the single-writer contract makes the load/store of
	 * seq itself race-free without stronger ordering. */
	atomic_store_explicit(&p->seq, seq + 1u, memory_order_relaxed);
	atomic_thread_fence(memory_order_release);

	memcpy(&p->slot, s, sizeof(*s));

	/* Release so a reader that observes this even value also observes every
	 * payload write above it. */
	atomic_store_explicit(&p->seq, seq + 2u, memory_order_release);
}

/**
 * Copy the published snapshot. Any reader thread, lock-free.
 *
 * @retval 0        @p out written.
 * @retval -EINVAL  NULL argument.
 * @retval -EAGAIN  The writer won every attempt; @p out is zeroed rather than
 *                  left holding a torn copy.
 */
static inline int sts_pwrseq_pub_read(const sts_pwrseq_pub_t *p,
				      sts_pwrseq_snap_t *out)
{
	unsigned int attempt;

	if ((p == NULL) || (out == NULL)) {
		return -EINVAL;
	}

	for (attempt = 0u; attempt < STS_PWRSEQ_PUB_RETRIES; attempt++) {
		uint32_t s1 = atomic_load_explicit(&p->seq, memory_order_acquire);
		uint32_t s2;

		if ((s1 & 1u) != 0u) {
			/* Writer mid-update; do not even copy. */
			continue;
		}

		memcpy(out, &p->slot, sizeof(*out));

		/* Acquire fence keeps the payload read above from sinking past
		 * the second sequence load, so a write that started during the
		 * copy is always detected. */
		atomic_thread_fence(memory_order_acquire);

		s2 = atomic_load_explicit(&p->seq, memory_order_relaxed);
		if (s1 == s2) {
			return 0;
		}
	}

	memset(out, 0, sizeof(*out));
	return -EAGAIN;
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_PLATFORM_STS_PWRSEQ_PUB_H_ */
