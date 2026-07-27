/*
 * STS1000 "Meridian" — core/util: lock-free SPSC byte ring.
 *
 * Platform-neutral C11. No dynamic allocation; the caller owns the backing
 * store, so a ring can live in a driver's static buffer or a thread's BSS.
 *
 * Concurrency model: exactly one producer and one consumer, which may be an
 * ISR and a thread in either direction. No locks, no critical sections.
 *
 *   Producer side (only): ring_put(), ring_putc()
 *   Consumer side (only): ring_get(), ring_getc(), ring_peek(),
 *                         ring_discard(), ring_reset()
 *   Either side:          ring_len(), ring_free(), ring_cap(),
 *                         ring_empty(), ring_full()
 *
 * Calling a producer function from the consumer (or vice versa), or having two
 * producers, breaks the invariant — index ownership is what removes the need
 * for locking. Observers on either side see a consistent but possibly stale
 * snapshot: ring_len() read by the producer is a lower bound, read by the
 * consumer an upper bound. Both are the useful direction.
 *
 * Indices are free-running size_t values masked on access, so the full capacity
 * is usable (no sacrificed slot) and no separate "full" flag is needed. Ordering
 * is enforced with C11 acquire/release on the two indices; the data bytes
 * themselves are ordinary memory published by those fences.
 *
 * Capacity must be a power of two so the mask is a single AND.
 */

#ifndef STS1000_CORE_UTIL_RING_H_
#define STS1000_CORE_UTIL_RING_H_

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Ring state. All fields are private; use the ring_* functions.
 *
 * @p head is written only by the producer, @p tail only by the consumer.
 */
typedef struct {
	uint8_t *buf;        /* caller-owned backing store, cap bytes */
	size_t cap;          /* capacity in bytes, a power of two */
	size_t mask;         /* cap - 1 */
	atomic_size_t head;  /* free-running write index (producer owns) */
	atomic_size_t tail;  /* free-running read index (consumer owns) */
} ring_t;

/**
 * Bind a ring to its backing store and empty it.
 *
 * @param r    Ring.
 * @param buf  Backing store of @p cap bytes.
 * @param cap  Capacity in bytes; must be a non-zero power of two.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p r or @p buf is NULL, or @p cap is zero or not a power
 *                  of two.
 */
int ring_init(ring_t *r, uint8_t *buf, size_t cap);

/** Capacity in bytes, as passed to ring_init(). 0 for a NULL ring. */
size_t ring_cap(const ring_t *r);

/** Bytes currently buffered. */
size_t ring_len(const ring_t *r);

/** Bytes that can be written before the ring is full. */
size_t ring_free(const ring_t *r);

/** True when no bytes are buffered. */
bool ring_empty(const ring_t *r);

/** True when no further byte can be written. */
bool ring_full(const ring_t *r);

/**
 * Producer: write up to @p n bytes.
 *
 * Partial writes are normal — this never blocks and never overwrites unread
 * data. Compare the return value against @p n to detect back-pressure.
 *
 * @param r    Ring.
 * @param src  Source bytes; may be NULL only when @p n is 0.
 * @param n    Bytes offered.
 * @return     Bytes actually written (0 .. @p n). 0 for invalid arguments.
 */
size_t ring_put(ring_t *r, const uint8_t *src, size_t n);

/**
 * Producer: write one byte.
 *
 * @retval 0        Written.
 * @retval -EINVAL  @p r is NULL.
 * @retval -ENOSPC  Ring full.
 */
int ring_putc(ring_t *r, uint8_t b);

/**
 * Consumer: read and remove up to @p n bytes.
 *
 * @param r    Ring.
 * @param dst  Destination; may be NULL only when @p n is 0.
 * @param n    Bytes requested.
 * @return     Bytes actually read (0 .. @p n). 0 for invalid arguments.
 */
size_t ring_get(ring_t *r, uint8_t *dst, size_t n);

/**
 * Consumer: read one byte and remove it.
 *
 * @retval 0        Read; @p b holds the byte.
 * @retval -EINVAL  @p r or @p b is NULL.
 * @retval -EAGAIN  Ring empty.
 */
int ring_getc(ring_t *r, uint8_t *b);

/**
 * Consumer: copy up to @p n buffered bytes without removing them.
 *
 * @param r    Ring.
 * @param dst  Destination; may be NULL only when @p n is 0.
 * @param n    Bytes requested.
 * @return     Bytes actually copied (0 .. @p n). 0 for invalid arguments.
 */
size_t ring_peek(const ring_t *r, uint8_t *dst, size_t n);

/**
 * Consumer: drop up to @p n buffered bytes without copying them.
 *
 * @return Bytes actually dropped.
 */
size_t ring_discard(ring_t *r, size_t n);

/**
 * Consumer: drop everything currently buffered.
 *
 * Safe against a concurrent producer: it advances the read index to the write
 * index observed at entry, so bytes produced during the call survive.
 */
void ring_reset(ring_t *r);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_UTIL_RING_H_ */
