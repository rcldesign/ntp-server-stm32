/*
 * STS1000 "Meridian" — the MP event channel's producer seam (console area).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Channel 0x09 is the stream a Field Maintenance Tool subscribes to in order to
 * watch the board. FMT §7.5 says it carries edge-triggered faults, button
 * presses, reed-switch changes, override vetoes and diagnostic progress. Three
 * of its eight kinds — MP_EV_OVERRIDE, MP_EV_DIAG, MP_EV_MODE — are raised by
 * sinks inside core/mp itself. The other five describe the *board*, and until
 * this file existed none of them had a producer anywhere in the image:
 * mp_post_event(), the one seam into mp_stream_eventf() from outside core, was
 * dropped by --gc-sections because nothing called it. A technician subscribed,
 * got an open channel that reported diagnostic progress and their own override
 * grants back to them, and sat in silence through a power-good drop, an INA228
 * alert, an alarm assertion, a panel button press and a door event. Silence was
 * indistinguishable from a healthy box.
 *
 * This file is the seam. The *decisions* are elsewhere and were already there:
 * platform/sts_io_policy.h's sts_io_dispatch_plan() classifies every debounced
 * scan event, and sts_alarm_set() is where an alarm's active state transitions.
 * What was missing was a way for either to reach the engine without waiting on
 * it.
 *
 * ---------------------------------------------------------------------------
 * Threading (F11). Many producers, one consumer, and none of them may wait
 * ---------------------------------------------------------------------------
 *
 *   - the PRODUCER side (sts_mp_post_event) is called from the 1 kHz GPIOF/
 *     GPIOG scan thread (platform/io_scan.c, priority 11) and from every caller
 *     of sts_alarm_set() — which includes the priority-4 discipline loop, the
 *     priority-6 GNSS thread, housekeeping, and mp_tunnel.c's TAMPER refresh
 *     with the engine lock already held. None of them may take the engine mutex:
 *     the scan feeds the fault and alarm path, and parking it behind a console
 *     request is the priority inversion this whole area exists to prevent. A
 *     producer therefore does an arming test (one atomic read) and a bounded
 *     copy into `ev_slots` under a k_spinlock, and returns;
 *   - the CONSUMER side is mp_glue.c's sts_mp_tick(), which drains this queue
 *     into mp_post_event() *inside* the engine lock it already holds, before it
 *     calls mp_tick(). Draining first is what puts an event on the wire in the
 *     same pass it was drained in rather than the next one.
 *
 * The spinlock covers both sides rather than relying on a single-producer/
 * single-consumer invariant, because there is no such invariant here: two
 * threads can stage at once (the scan and the discipline loop are on different
 * priorities and either may preempt the other), and mp_ev_t is 48 bytes, so an
 * interleave would not merely reorder records — it would build one out of
 * halves of two. Held for one memcpy of that size, order 200 cycles at 250 MHz.
 *
 * The arming predicate stays lock-free (one atomic read of the mask mp_glue.c
 * republishes), so a producer asking "is anyone watching" never queues behind
 * the console, and an unsubscribed channel costs the 1 kHz scan a single load.
 *
 * NESTING. Producers may hold other locks — the engine mutex (mp_tunnel.c) or
 * nothing at all (io_scan) — and this file takes only the spinlock, never a
 * mutex, so it can never be the middle of a cycle. sts_alarm_set() deliberately
 * calls it OUTSIDE the fault mutex for the same reason.
 *
 * ---------------------------------------------------------------------------
 * Consequences, stated rather than hidden
 * ---------------------------------------------------------------------------
 *
 *   - LOSS. The queue is CONFIG_STS1000_MP_EVQ_STAGE records and drops the
 *     NEWEST on overflow, counted here and in `mp status`, and folded by the
 *     drain into the engine's own drop counter so key 8 of the event record
 *     tells the HOST a gap happened. An event channel that silently loses the
 *     event a technician is waiting for is the original defect in a new place.
 *     Drop-newest keeps a cascade's root cause, which is core/fault's argument
 *     for its own queue and core/mp's for its.
 *
 *   - LATENCY. Up to one console-supervisor pass (250 ms), and at most
 *     (STS_MP_TICK_MISS_MAX + 1) x (250 + STS_MP_TICK_LOCK_MS) = 1800 ms when
 *     the shell thread is contending — the bound sts_console.c's BUILD_ASSERT
 *     already proves for the override dead-man. The record carries the
 *     producer's own mono_ms, not the drain's, so the delay costs delivery time
 *     and not the ability to correlate one edge with another.
 *
 *   - TICK BUDGET. Nothing here adds a lock wait to the supervisor's pass.
 *     The drain runs inside the wait sts_mp_tick() already spends; the
 *     spinlock is not a wait. sts_console.c's BUILD_ASSERT budgets exactly one
 *     timed lock wait per pass and it is still the tick's own.
 */

#include <zephyr/kernel.h>

#if defined(CONFIG_STS1000_CONSOLE) && defined(CONFIG_STS1000_MP)

#include <errno.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "console/mp_glue.h"
#include "console/sts_mp_evq.h"
#include "fault/fault.h"

LOG_MODULE_DECLARE(sts_mp, CONFIG_STS1000_LOG_LEVEL);

/* --------------------------------------------------------------- tunables */

/*
 * As everywhere in the console area, the Kconfig symbol carries an
 * #if defined() fallback equal to its default, so the file is correct whether
 * or not app/Kconfig sources src/zephyr/console/Kconfig.mp.
 */
#if defined(CONFIG_STS1000_MP_EVQ_STAGE)
#define MP_EVQ_STAGE_LEN CONFIG_STS1000_MP_EVQ_STAGE
#else
#define MP_EVQ_STAGE_LEN 32
#endif

/*
 * FAULT_EVT_QUEUE_LEN is the real upstream bound: one scan can commit all 32
 * signals at once — a rail collapse takes its power-good, its INA alert and a
 * load-switch flag with it, and a board-wide brownout takes everything — and
 * io_scan.c drains core/fault's whole queue inside that same millisecond. A
 * staging queue shallower than that would drop exactly the burst a technician
 * most needs to see, on the 250 ms boundary where the drain has not run yet.
 */
BUILD_ASSERT(MP_EVQ_STAGE_LEN >= FAULT_EVT_QUEUE_LEN,
	     "the MP event staging queue cannot hold one full core/fault burst");

/* ------------------------------------------------------------------ state */

static mp_ev_t ev_slots[MP_EVQ_STAGE_LEN];
static sts_mp_evq_t ev_q;
static struct k_spinlock ev_lock;
static bool ev_ready;

/**
 * Staged drops already handed to the engine's counter.
 *
 * `ev_q.dropped` is cumulative so `mp status` can print a total that never goes
 * backwards; the engine wants the *delta* since the last drain, because
 * mp_enc_events() clears its own counter once the queue has emptied. Read and
 * written only by the drain, which is single-threaded on the console
 * supervisor, so it needs no lock of its own.
 */
static uint32_t ev_drops_reported;

void sts_mp_event_init(void)
{
	k_spinlock_key_t key;

	if (ev_ready) {
		return;
	}
	key = k_spin_lock(&ev_lock);
	if (sts_mp_evq_init(&ev_q, ev_slots, (uint16_t)ARRAY_SIZE(ev_slots)) ==
	    0) {
		ev_ready = true;
	}
	k_spin_unlock(&ev_lock, key);

	if (!ev_ready) {
		/* Only reachable if ARRAY_SIZE(ev_slots) were 0, which the
		 * Kconfig range and the BUILD_ASSERT above both forbid. */
		LOG_ERR("MP event staging queue rejected; channel 0x09 will "
			"carry only the engine's own events");
	}
}

void sts_mp_post_event(uint8_t kind, uint8_t sub, uint16_t id, uint8_t edge,
		       int32_t value, uint32_t mono_ms, const char *text)
{
	k_spinlock_key_t key;
	int rc;

	/*
	 * The arming test, and it is what keeps this free on the 1 kHz scan: with
	 * no technician subscribed to channel 0x09 a call is one atomic read and
	 * a return. sts_mp_ch_armed() lags a subscription change by at most one
	 * tick, and both directions are harmless — a stale "armed" stages one
	 * record the drain then discards because nobody is listening, a stale
	 * "not armed" costs one tick of events at subscribe time.
	 */
	if (!ev_ready || !sts_mp_ch_armed((uint8_t)MP_CH_EVENT)) {
		return;
	}

	key = k_spin_lock(&ev_lock);
	rc = sts_mp_evq_stage(&ev_q, kind, sub, id, edge, value, mono_ms, text);
	k_spin_unlock(&ev_lock, key);

	/*
	 * Deliberately silent on -ENOSPC. The overflow is already counted inside
	 * the queue and reaches both `mp status` and — through the drain — the
	 * host's own event record; logging it from here would put a log call on
	 * the scan thread at exactly the moment the board is producing events
	 * faster than they can be drained, which is the moment that thread has
	 * least to spare. -EINVAL is a caller bug that the BUILD_ASSERTs in
	 * io_scan.c and the enum bound in the queue between them make
	 * unreachable.
	 */
	ARG_UNUSED(rc);
}

bool sts_mp_event_peek(mp_ev_t *out)
{
	k_spinlock_key_t key;
	bool got;

	if (!ev_ready) {
		return false;
	}
	key = k_spin_lock(&ev_lock);
	got = sts_mp_evq_peek(&ev_q, out);
	k_spin_unlock(&ev_lock, key);
	return got;
}

void sts_mp_event_pop(void)
{
	k_spinlock_key_t key;

	if (!ev_ready) {
		return;
	}
	key = k_spin_lock(&ev_lock);
	sts_mp_evq_pop(&ev_q);
	k_spin_unlock(&ev_lock, key);
}

void sts_mp_event_purge(void)
{
	k_spinlock_key_t key;

	if (!ev_ready) {
		return;
	}
	key = k_spin_lock(&ev_lock);
	sts_mp_evq_reset(&ev_q);
	/*
	 * The cumulative counters are NOT reset: `mp status` reporting "0
	 * dropped" after a subscription ended would erase the record of an
	 * overflow that happened while a host was watching. What is reset is the
	 * engine's view — nothing was lost from a listener's point of view here,
	 * so the next subscription starts from a clean slate rather than
	 * inheriting a gap marker for events nobody wanted.
	 */
	ev_drops_reported = ev_q.dropped;
	k_spin_unlock(&ev_lock, key);
}

uint32_t sts_mp_event_take_drops(void)
{
	k_spinlock_key_t key;
	uint32_t n;

	if (!ev_ready) {
		return 0U;
	}
	key = k_spin_lock(&ev_lock);
	n = ev_q.dropped - ev_drops_reported;
	ev_drops_reported = ev_q.dropped;
	k_spin_unlock(&ev_lock, key);
	return n;
}

void sts_mp_event_stats(uint32_t *queued, uint32_t *staged, uint32_t *dropped)
{
	k_spinlock_key_t key;

	if (!ev_ready) {
		if (queued != NULL) {
			*queued = 0U;
		}
		if (staged != NULL) {
			*staged = 0U;
		}
		if (dropped != NULL) {
			*dropped = 0U;
		}
		return;
	}

	key = k_spin_lock(&ev_lock);
	if (queued != NULL) {
		*queued = (uint32_t)sts_mp_evq_count(&ev_q);
	}
	if (staged != NULL) {
		*staged = ev_q.staged;
	}
	if (dropped != NULL) {
		*dropped = ev_q.dropped;
	}
	k_spin_unlock(&ev_lock, key);
}

#endif /* CONFIG_STS1000_CONSOLE && CONFIG_STS1000_MP */
