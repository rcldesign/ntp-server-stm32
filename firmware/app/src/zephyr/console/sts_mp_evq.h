/*
 * STS1000 "Meridian" — the MP event channel's staging queue (console area).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/console/, and Zephyr-free so tests/host can drive it —
 * the same arrangement as sts_mp_tunnel_policy.h and sts_mp_telem.h beside it.
 *
 * ---------------------------------------------------------------------------
 * Why a second queue exists at all
 * ---------------------------------------------------------------------------
 * core/mp already has an event queue (mp_stream_ctx_t::evq) and mp_tick()
 * already drains it onto channel 0x09. What it does not have is a way IN from a
 * thread that must not block: every entry into core/mp runs under the glue's
 * engine mutex (mp_glue.h's threading block), and the producers of the five
 * platform event kinds are exactly the threads that may not wait on it —
 *
 *   MP_EV_FAULT / BUTTON / PROX / TOUCH   the 1 kHz GPIOF/GPIOG scan
 *                                         (platform/io_scan.c, priority 11)
 *   MP_EV_ALARM                           whoever raised the alarm, which
 *                                         includes the priority-4 discipline
 *                                         loop and the priority-6 GNSS thread
 *
 * Parking the fault scan behind a console request is the priority inversion the
 * whole console area is built to prevent, and it would be doing it on the path
 * that carries the fault the technician is waiting for. So the producers stage
 * here — one bounded copy, no allocation, no wait — and the console supervisor
 * drains this queue into the engine on its own pass, inside the lock it already
 * holds. It is the same shape as mp_tunnel.c's byte tee and for the same reason;
 * that file's threading block is the long-form argument.
 *
 * ---------------------------------------------------------------------------
 * What this header is and is not
 * ---------------------------------------------------------------------------
 * It is the queue: the record layout, the bounds, the drop rule and the
 * counters. It is NOT the synchronisation — mp_events.c owns the k_spinlock
 * that serialises the producers against each other and against the drain, and
 * every function below is documented as requiring it. Keeping the two apart is
 * what makes the drop rule and the counters testable on the host, where there
 * are no threads to race.
 *
 * ---------------------------------------------------------------------------
 * Consequences, stated rather than hidden
 * ---------------------------------------------------------------------------
 *   - LOSS. The queue holds STS_MP_EVQ_MAX-bounded records and drops the
 *     NEWEST on overflow, counted in `dropped`. Drop-newest rather than
 *     drop-oldest because a cascade's first events are its root cause and the
 *     rest are consequences — the same argument core/fault makes for its own
 *     queue (fault.h) and core/mp for its. The count is not decoration: the
 *     drain folds it into mp_stream_event_drop_note(), so key 8 of the event
 *     record tells the host a gap happened. A dropped event that no one counts
 *     is the original defect in a new place.
 *
 *   - LATENCY. A staged event reaches the wire on the next console-supervisor
 *     pass that can take the engine lock: 250 ms typical, and at most
 *     (STS_MP_TICK_MISS_MAX + 1) passes when the shell thread is contending
 *     (mp_glue.h's budget). The record carries the *scan's* mono_ms, not the
 *     drain's, so the delay costs delivery time and not fidelity.
 */

#ifndef STS1000_ZEPHYR_CONSOLE_STS_MP_EVQ_H_
#define STS1000_ZEPHYR_CONSOLE_STS_MP_EVQ_H_

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "mp/mp_stream.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * The staging queue.
 *
 * Storage is caller-owned (core/mp's rule, and this project's: nothing
 * allocates). `head`/`len` are a plain ring over `ev`; both are written by the
 * consumer and the producer, which is why the caller must hold the spinlock
 * across every call that touches them.
 */
typedef struct {
	mp_ev_t *ev;
	uint16_t cap;
	uint16_t head;
	uint16_t len;

	/** Records accepted since boot. */
	uint32_t staged;
	/** Records refused for want of room since boot; saturating. */
	uint32_t dropped;
} sts_mp_evq_t;

/**
 * Bind @p slots as the queue's storage.
 *
 * @retval 0        Ready.
 * @retval -EINVAL  NULL argument or @p cap is 0.
 */
static inline int sts_mp_evq_init(sts_mp_evq_t *q, mp_ev_t *slots, uint16_t cap)
{
	if ((q == NULL) || (slots == NULL) || (cap == 0U)) {
		return -EINVAL;
	}
	(void)memset(q, 0, sizeof(*q));
	q->ev = slots;
	q->cap = cap;
	return 0;
}

/** True once sts_mp_evq_init() has bound storage. */
static inline bool sts_mp_evq_ready(const sts_mp_evq_t *q)
{
	return (q != NULL) && (q->ev != NULL) && (q->cap != 0U);
}

/**
 * Stage one event. Producer side; **call with the spinlock held**.
 *
 * The argument list is mp_stream_eventf()'s deliberately: this queue exists
 * only to move that call off the producer's thread, and a signature that
 * differed would be an invitation to lose a field crossing the seam. @p text is
 * copied and truncated to MP_EV_TEXT_MAX-1, so it may be a stack buffer.
 *
 * @retval 0        Staged.
 * @retval -EINVAL  @p q is not initialised, or @p kind is not an mp_ev_kind_t.
 * @retval -ENOSPC  Full; the NEW event is dropped and counted.
 */
static inline int sts_mp_evq_stage(sts_mp_evq_t *q, uint8_t kind, uint8_t sub,
				   uint16_t id, uint8_t edge, int32_t value,
				   uint32_t mono_ms, const char *text)
{
	mp_ev_t *slot;

	if (!sts_mp_evq_ready(q)) {
		return -EINVAL;
	}
	/*
	 * Refuse an out-of-range kind here rather than let mp_stream_event()
	 * refuse it at the drain: a producer that gets it wrong should fail
	 * where its own counter can show it, not silently occupy a slot that a
	 * real event then cannot have.
	 */
	if (kind >= (uint8_t)MP_EV_KIND_COUNT) {
		return -EINVAL;
	}
	if (q->len >= q->cap) {
		if (q->dropped != UINT32_MAX) {
			q->dropped++;
		}
		return -ENOSPC;
	}

	slot = &q->ev[(uint16_t)((q->head + q->len) % q->cap)];
	(void)memset(slot, 0, sizeof(*slot));
	slot->kind = kind;
	slot->sub = sub;
	slot->id = id;
	slot->edge = edge;
	slot->value = value;
	slot->mono_ms = mono_ms;
	if (text != NULL) {
		size_t n = strlen(text);

		if (n > (size_t)(MP_EV_TEXT_MAX - 1U)) {
			n = (size_t)(MP_EV_TEXT_MAX - 1U);
		}
		(void)memcpy(slot->text, text, n);
	}
	q->len++;
	if (q->staged != UINT32_MAX) {
		q->staged++;
	}
	return 0;
}

/**
 * Copy the oldest staged event without removing it. **Spinlock held.**
 *
 * Peek-then-pop rather than pop, so the drain can hand a record to the engine
 * and keep it staged if the engine refuses — the same shape, and the same
 * reason, as sts_mp_tunnel_drain()'s peek/send/discard.
 *
 * @retval true   @p out written.
 * @retval false  Empty, or @p q is not initialised.
 */
static inline bool sts_mp_evq_peek(const sts_mp_evq_t *q, mp_ev_t *out)
{
	if (!sts_mp_evq_ready(q) || (out == NULL) || (q->len == 0U)) {
		return false;
	}
	*out = q->ev[q->head];
	return true;
}

/** Discard the oldest staged event. **Spinlock held.** No-op when empty. */
static inline void sts_mp_evq_pop(sts_mp_evq_t *q)
{
	if (!sts_mp_evq_ready(q) || (q->len == 0U)) {
		return;
	}
	q->head = (uint16_t)((q->head + 1U) % q->cap);
	q->len--;
}

/**
 * Discard the backlog. **Spinlock held.**
 *
 * NOT counted as dropped, and the distinction is the point: the drain calls
 * this when no host is subscribed to channel 0x09, and events nobody was
 * listening for are not a loss to report. Overflow while a host IS listening
 * goes through sts_mp_evq_stage()'s -ENOSPC and IS reported.
 */
static inline void sts_mp_evq_reset(sts_mp_evq_t *q)
{
	if (!sts_mp_evq_ready(q)) {
		return;
	}
	q->head = 0U;
	q->len = 0U;
}

/** Records waiting. **Spinlock held.** */
static inline uint16_t sts_mp_evq_count(const sts_mp_evq_t *q)
{
	return sts_mp_evq_ready(q) ? q->len : 0U;
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_CONSOLE_STS_MP_EVQ_H_ */
