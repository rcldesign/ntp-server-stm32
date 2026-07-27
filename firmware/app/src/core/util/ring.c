/*
 * STS1000 "Meridian" — core/util: lock-free SPSC byte ring.
 *
 * See ring.h for the concurrency contract.
 *
 * Ordering: the producer publishes data with a release store to `head`; the
 * consumer observes it with an acquire load of `head`. Symmetrically the
 * consumer publishes freed space with a release store to `tail` and the
 * producer acquires it. Each side loads the index it owns with relaxed
 * ordering — nobody else writes it.
 */

#include "util/ring.h"

#include <errno.h>
#include <string.h>

int ring_init(ring_t *r, uint8_t *buf, size_t cap)
{
	if ((r == NULL) || (buf == NULL)) {
		return -EINVAL;
	}
	if ((cap == 0U) || ((cap & (cap - 1U)) != 0U)) {
		return -EINVAL; /* capacity must be a non-zero power of two */
	}

	r->buf = buf;
	r->cap = cap;
	r->mask = cap - 1U;
	atomic_init(&r->head, (size_t)0U);
	atomic_init(&r->tail, (size_t)0U);
	return 0;
}

size_t ring_cap(const ring_t *r)
{
	return (r != NULL) ? r->cap : 0U;
}

size_t ring_len(const ring_t *r)
{
	size_t head;
	size_t tail;

	if (r == NULL) {
		return 0U;
	}

	head = atomic_load_explicit(&r->head, memory_order_acquire);
	tail = atomic_load_explicit(&r->tail, memory_order_acquire);

	/* Free-running indices: the unsigned difference is exact across wrap. */
	return head - tail;
}

size_t ring_free(const ring_t *r)
{
	if (r == NULL) {
		return 0U;
	}
	return r->cap - ring_len(r);
}

bool ring_empty(const ring_t *r)
{
	return ring_len(r) == 0U;
}

bool ring_full(const ring_t *r)
{
	return (r != NULL) && (ring_len(r) == r->cap);
}

size_t ring_put(ring_t *r, const uint8_t *src, size_t n)
{
	size_t head;
	size_t tail;
	size_t space;
	size_t off;
	size_t first;

	if ((r == NULL) || ((src == NULL) && (n != 0U))) {
		return 0U;
	}

	head = atomic_load_explicit(&r->head, memory_order_relaxed); /* we own it */
	tail = atomic_load_explicit(&r->tail, memory_order_acquire);

	space = r->cap - (head - tail);
	if (n > space) {
		n = space;
	}
	if (n == 0U) {
		return 0U;
	}

	off = head & r->mask;
	first = r->cap - off;
	if (first > n) {
		first = n;
	}

	memcpy(&r->buf[off], src, first);
	if (n > first) {
		memcpy(&r->buf[0], &src[first], n - first);
	}

	atomic_store_explicit(&r->head, head + n, memory_order_release);
	return n;
}

int ring_putc(ring_t *r, uint8_t b)
{
	if (r == NULL) {
		return -EINVAL;
	}
	if (ring_put(r, &b, 1U) != 1U) {
		return -ENOSPC;
	}
	return 0;
}

/*
 * Copy up to n buffered bytes starting at the consumer's read index. Shared by
 * ring_get() and ring_peek(); does not move the index. `r` is const because
 * neither caller mutates the ring here — the index advance is the caller's job.
 */
static size_t ring_copy_out(const ring_t *r, uint8_t *dst, size_t n,
			    size_t tail, size_t avail)
{
	size_t off;
	size_t first;

	if (n > avail) {
		n = avail;
	}
	if (n == 0U) {
		return 0U;
	}

	off = tail & r->mask;
	first = r->cap - off;
	if (first > n) {
		first = n;
	}

	memcpy(dst, &r->buf[off], first);
	if (n > first) {
		memcpy(&dst[first], &r->buf[0], n - first);
	}

	return n;
}

size_t ring_get(ring_t *r, uint8_t *dst, size_t n)
{
	size_t head;
	size_t tail;
	size_t got;

	if ((r == NULL) || ((dst == NULL) && (n != 0U))) {
		return 0U;
	}

	tail = atomic_load_explicit(&r->tail, memory_order_relaxed); /* we own it */
	head = atomic_load_explicit(&r->head, memory_order_acquire);

	got = ring_copy_out(r, dst, n, tail, head - tail);
	if (got == 0U) {
		return 0U;
	}

	atomic_store_explicit(&r->tail, tail + got, memory_order_release);
	return got;
}

int ring_getc(ring_t *r, uint8_t *b)
{
	if ((r == NULL) || (b == NULL)) {
		return -EINVAL;
	}
	if (ring_get(r, b, 1U) != 1U) {
		return -EAGAIN;
	}
	return 0;
}

size_t ring_peek(const ring_t *r, uint8_t *dst, size_t n)
{
	size_t head;
	size_t tail;

	if ((r == NULL) || ((dst == NULL) && (n != 0U))) {
		return 0U;
	}

	tail = atomic_load_explicit(&r->tail, memory_order_relaxed);
	head = atomic_load_explicit(&r->head, memory_order_acquire);

	return ring_copy_out(r, dst, n, tail, head - tail);
}

size_t ring_discard(ring_t *r, size_t n)
{
	size_t head;
	size_t tail;
	size_t avail;

	if (r == NULL) {
		return 0U;
	}

	tail = atomic_load_explicit(&r->tail, memory_order_relaxed);
	head = atomic_load_explicit(&r->head, memory_order_acquire);

	avail = head - tail;
	if (n > avail) {
		n = avail;
	}
	if (n == 0U) {
		return 0U;
	}

	atomic_store_explicit(&r->tail, tail + n, memory_order_release);
	return n;
}

void ring_reset(ring_t *r)
{
	if (r == NULL) {
		return;
	}
	(void)ring_discard(r, SIZE_MAX);
}
