/*
 * STS1000 "Meridian" — the parameterised sequencer request mailbox.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/platform/, and Zephyr-free so tests/host can drive it —
 * the same arrangement as sts_rbguard.h and sts_pwrseq_pub.h beside it. The
 * cross-area surface (the request ids, the result struct and the two entry
 * points) is in zephyr/sts_app.h, because that header is the ONLY interface
 * between the four glue areas; this one is the mechanism underneath it.
 *
 * ---------------------------------------------------------------------------
 * Why a second request path exists at all
 * ---------------------------------------------------------------------------
 * pwrseq_exec.c already has one: three operator recovery actions post a bit of
 * a single atomic word (PWRSEQ_REQ_RB_RETRY / _OV_CLEAR / _POE_KILL) and the
 * housekeeping thread performs them on its own 4 Hz pass. That file's
 * "operator requests" block is the long-form argument for why they may not
 * simply call in: emit() writes the action ring's head and pwrseq_drain() reads
 * and EXECUTES it, with no lock anywhere, so a console thread calling directly
 * would be appending to a queue whose consumer is mid-drain.
 *
 * Those three coalesce into single bits because each is IDEMPOTENT — retrying a
 * retry, clearing a cleared latch, killing an already-killed rail are all
 * no-ops, so two requests folding into one is correct rather than a lost
 * message. A general object setter cannot use that mechanism: it must carry
 * (object, value), which a bitmask cannot express, and two different values for
 * one object arriving before a drain is a real case that needs a defined answer
 * rather than an accident of bit arithmetic.
 *
 * ---------------------------------------------------------------------------
 * The concurrency discipline: spinlock-and-copy, not a seqlock
 * ---------------------------------------------------------------------------
 * Two precedents are in tree and they solve different problems.
 *
 *   sts_pwrseq_pub.h is a SEQLOCK. It is single-writer PUBLICATION: the
 *   housekeeping thread owns the payload and readers take a consistent copy.
 *   It cannot be used here because here the foreign thread is the PRODUCER —
 *   two writers with no owner is exactly what a seqlock does not do.
 *
 *   sts_mp_evq.h is SPINLOCK-AND-COPY, and it is the same shape as this: a
 *   thread that must not block stages a bounded record, and the owning thread
 *   drains it on its own pass. This header is modelled on it deliberately,
 *   including the split — the header is the queue (layout, bounds, the
 *   collision rule, the counters) and the .c owns the k_spinlock, with every
 *   function below documented as requiring it. That split is what lets the
 *   collision rule be tested on a host where there are no threads to race.
 *
 * A mutex is not an option in either direction. The producer is the console
 * thread inside the MP engine lock, whose whole tick budget is accounted for in
 * mp_glue.h (STS_MP_TICK_LOCK_MS 50 x STS_MP_TICK_MISS_MAX 5 inside
 * MP_TICK_MAX_MS 2000); the consumer is the lowest-priority application thread
 * on the board. A mutex would put each on the other's critical path. The
 * spinlock is held only across a bounded struct copy — no allocation, no
 * logging, no I/O, no call out of this header — so neither the 1 kHz io_scan
 * nor the priority-4 discipline thread can be delayed by more than that copy.
 * The ACTUATION happens outside the lock, claim-then-execute-then-settle, which
 * is sts_mp_evq.h's peek/send/discard for the same reason.
 *
 * ---------------------------------------------------------------------------
 * The collision rule: LAST WRITER WINS, per object, in a fixed slot table
 * ---------------------------------------------------------------------------
 * Two requests for the SAME object before a drain are not two events; they are
 * one intent revised. Executing both would drive the pin twice, and — worse —
 * would leave it at the OLDER value if the drain order were ever perturbed. So
 * the second overwrites the first and is counted in `coalesced`. Nothing is
 * dropped silently: the count is the record that a revision happened, and the
 * value that lands is the one the caller asked for last.
 *
 * Two requests for DIFFERENT objects never interact, because the table is
 * indexed BY object: one slot per sts_pwrseq_req_id_t. That is also why it
 * cannot overflow — its capacity is the number of requestable objects, a
 * compile-time constant — so there is no bounded-queue drop rule to get wrong
 * and no unbounded growth. Slot 0 (STS_PWRSEQ_REQ_NONE) is never used, the same
 * convention as sts_mp_veto_policy.h's "bit 0 is never set": it costs one slot
 * and makes an id of zero a rejected argument rather than a valid row.
 *
 * Keyed by a small platform enum and NOT by manifest index, so the platform
 * area stays free of the console's object table — "platform publishes, console
 * binds", the same layering sts_mp_veto_policy.h argues for in the other
 * direction. It is also what keeps the table one row long today instead of 91.
 *
 * ---------------------------------------------------------------------------
 * The outcome side, and what it deliberately does not preserve
 * ---------------------------------------------------------------------------
 * A drained request that the sequencer refuses must reach the caller; a request
 * that vanishes silently is the defect this whole path exists to close. So each
 * slot also carries an UNREAD OUTCOME, written by the drain and taken by the
 * console on its next pass, which maps a refusal onto mp_ovr_veto() — the
 * existing withdrawal path, with its channel-0x09 record and its audit line.
 *
 * The outcome is last-writer-wins too, and that is consistent rather than
 * lossy. If request A is refused and request B for the same object is then
 * applied before the console reads either, A's refusal describes a lease that
 * no longer exists — mp_ovr_grant() replaces the lease on a second grant for
 * the same object — so reporting it would withdraw the SUCCESSOR's lease for
 * its predecessor's reason. The reverse order (A applied, B refused) keeps the
 * refusal, which is the case that matters.
 */

#ifndef STS1000_ZEPHYR_PLATFORM_STS_PWRSEQ_REQ_H_
#define STS1000_ZEPHYR_PLATFORM_STS_PWRSEQ_REQ_H_

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "zephyr/sts_app.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * One object's mailbox slot.
 *
 * `posted` is the request half and `outcome` the reply half; they are
 * independent, so a slot may hold an unread outcome and a freshly posted
 * request at the same time — which is exactly what happens when a technician
 * revises an override faster than the console drains.
 */
typedef struct {
	/* --- request half: written by the producer, cleared by the drain --- */
	bool     posted;
	bool     release;  /**< the request is "return to firmware-automatic" */
	int32_t  value;    /**< meaningless when `release` */
	uint32_t posted_ms;

	/* --- reply half: written by the drain, cleared by the console ------ */
	uint8_t  outcome;    /**< sts_pwrseq_req_out_t; NONE = nothing unread */
	uint8_t  err;        /**< sts_pwrseq_req_err_t; NONE unless REFUSED */
	int32_t  applied;    /**< the value the drain actually applied */
	uint32_t settled_ms;
} sts_pwrseq_req_slot_t;

/**
 * The mailbox.
 *
 * Zero-initialised static storage is a valid empty mailbox, so there is no init
 * call: every slot reads "nothing posted, nothing unread".
 */
typedef struct {
	sts_pwrseq_req_slot_t slot[STS_PWRSEQ_REQ_COUNT];

	/** Requests accepted since boot; saturating. */
	uint32_t posted;
	/** Requests that overwrote an undrained one; saturating. */
	uint32_t coalesced;
	/** Requests the drain applied; saturating. */
	uint32_t applied;
	/** Requests the drain refused; saturating. */
	uint32_t refused;
} sts_pwrseq_reqq_t;

/** True when @p req names a usable slot. Slot 0 is never one. */
static inline bool sts_pwrseq_req_id_ok(uint8_t req)
{
	return (req > (uint8_t)STS_PWRSEQ_REQ_NONE) &&
	       (req < (uint8_t)STS_PWRSEQ_REQ_COUNT);
}

static inline void sts_pwrseq_req_bump(uint32_t *c)
{
	if (*c != UINT32_MAX) {
		(*c)++;
	}
}

/**
 * Post one request. Producer side; **call with the spinlock held.**
 *
 * @param release true returns the object to firmware-automatic control; @p
 *                value is then ignored. This is obj_apply()'s `value == NULL`.
 *
 * @retval 0        Posted. An undrained request for the same object was
 *                  replaced and counted in `coalesced` — see the collision rule
 *                  in the file header.
 * @retval -EINVAL  @p q is NULL or @p req is not a request id.
 */
static inline int sts_pwrseq_reqq_post(sts_pwrseq_reqq_t *q, uint8_t req,
				       bool release, int32_t value,
				       uint32_t now_ms)
{
	sts_pwrseq_req_slot_t *s;

	if ((q == NULL) || !sts_pwrseq_req_id_ok(req)) {
		return -EINVAL;
	}

	s = &q->slot[req];
	if (s->posted) {
		sts_pwrseq_req_bump(&q->coalesced);
	}
	s->posted = true;
	s->release = release;
	s->value = release ? 0 : value;
	s->posted_ms = now_ms;
	sts_pwrseq_req_bump(&q->posted);
	return 0;
}

/** True when @p req has a request awaiting the drain. **Spinlock held.** */
static inline bool sts_pwrseq_reqq_pending(const sts_pwrseq_reqq_t *q,
					   uint8_t req)
{
	if ((q == NULL) || !sts_pwrseq_req_id_ok(req)) {
		return false;
	}
	return q->slot[req].posted;
}

/**
 * Claim @p req's pending request for execution. Consumer side; **spinlock held.**
 *
 * Clears `posted` so a request arriving DURING the execution that follows is a
 * new request for the next pass rather than one this pass silently swallows.
 * The execution itself must happen outside the lock; the result comes back
 * through sts_pwrseq_reqq_settle().
 *
 * @retval true   @p out_release / @p out_value written; the caller owns it now.
 * @retval false  Nothing pending, or a bad argument.
 */
static inline bool sts_pwrseq_reqq_claim(sts_pwrseq_reqq_t *q, uint8_t req,
					 bool *out_release, int32_t *out_value)
{
	sts_pwrseq_req_slot_t *s;

	if ((q == NULL) || !sts_pwrseq_req_id_ok(req) || (out_release == NULL) ||
	    (out_value == NULL)) {
		return false;
	}

	s = &q->slot[req];
	if (!s->posted) {
		return false;
	}

	*out_release = s->release;
	*out_value = s->value;
	s->posted = false;
	return true;
}

/**
 * Record what became of a claimed request. Consumer side; **spinlock held.**
 *
 * @param outcome sts_pwrseq_req_out_t. STS_PWRSEQ_REQ_OUT_NONE is rejected:
 *                a drain that settles nothing is the silent disappearance this
 *                mailbox exists to prevent, so it cannot be expressed.
 * @param err     sts_pwrseq_req_err_t; ignored unless @p outcome is REFUSED.
 * @param applied The value that actually reached the actuator.
 *
 * @retval 0        Recorded. Any unread outcome is replaced — see the file
 *                  header for why that is consistent and not a loss.
 * @retval -EINVAL  Bad argument.
 */
static inline int sts_pwrseq_reqq_settle(sts_pwrseq_reqq_t *q, uint8_t req,
					 uint8_t outcome, uint8_t err,
					 int32_t applied, uint32_t now_ms)
{
	sts_pwrseq_req_slot_t *s;

	if ((q == NULL) || !sts_pwrseq_req_id_ok(req)) {
		return -EINVAL;
	}
	if ((outcome <= (uint8_t)STS_PWRSEQ_REQ_OUT_NONE) ||
	    (outcome >= (uint8_t)STS_PWRSEQ_REQ_OUT_COUNT)) {
		return -EINVAL;
	}
	if (err >= (uint8_t)STS_PWRSEQ_REQ_ERR_COUNT) {
		return -EINVAL;
	}

	s = &q->slot[req];
	s->outcome = outcome;
	s->err = (outcome == (uint8_t)STS_PWRSEQ_REQ_OUT_REFUSED)
			 ? err
			 : (uint8_t)STS_PWRSEQ_REQ_ERR_NONE;
	s->applied = applied;
	s->settled_ms = now_ms;

	if (outcome == (uint8_t)STS_PWRSEQ_REQ_OUT_REFUSED) {
		sts_pwrseq_req_bump(&q->refused);
	} else {
		sts_pwrseq_req_bump(&q->applied);
	}
	return 0;
}

/**
 * Take @p req's unread outcome. Console side; **spinlock held.**
 *
 * Take, not peek: an outcome reported twice would withdraw a lease the first
 * report already withdrew, and then withdraw its replacement.
 *
 * @retval true   @p out written and the slot's outcome cleared.
 * @retval false  Nothing unread, or a bad argument.
 */
static inline bool sts_pwrseq_reqq_take(sts_pwrseq_reqq_t *q, uint8_t req,
					sts_pwrseq_req_result_t *out)
{
	sts_pwrseq_req_slot_t *s;

	if ((q == NULL) || !sts_pwrseq_req_id_ok(req) || (out == NULL)) {
		return false;
	}

	s = &q->slot[req];
	if (s->outcome == (uint8_t)STS_PWRSEQ_REQ_OUT_NONE) {
		return false;
	}

	(void)memset(out, 0, sizeof(*out));
	out->outcome = s->outcome;
	out->err = s->err;
	out->value = s->applied;
	out->settled_ms = s->settled_ms;

	s->outcome = (uint8_t)STS_PWRSEQ_REQ_OUT_NONE;
	s->err = (uint8_t)STS_PWRSEQ_REQ_ERR_NONE;
	return true;
}

/**
 * The sentence a technician reads when the sequencer refuses a request.
 *
 * Constants here rather than strings passed by the caller, for the reason
 * sts_mp_veto_policy.h states about its own: the seam carries no pointer that
 * could dangle, the set is reviewable in one place against the 31-character
 * channel-0x09 budget, and the host suite checks every one of them fits.
 *
 * Never NULL for a real refusal reason — a refusal that cannot say what
 * happened is the shape of the defect this path exists to close.
 */
static inline const char *sts_pwrseq_req_reason_of(uint8_t err)
{
	switch ((sts_pwrseq_req_err_t)err) {
	case STS_PWRSEQ_REQ_ERR_STAGE:
		return "sequencer has not reached it";
	case STS_PWRSEQ_REQ_ERR_SHED:
		return "load shed (power/thermal)";
	case STS_PWRSEQ_REQ_ERR_HW:
		return "actuator refused the write";
	case STS_PWRSEQ_REQ_ERR_GATED:
		return "FE is connected to this rail";
	case STS_PWRSEQ_REQ_ERR_NONE:
	case STS_PWRSEQ_REQ_ERR_COUNT:
	default:
		return NULL;
	}
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_PLATFORM_STS_PWRSEQ_REQ_H_ */
