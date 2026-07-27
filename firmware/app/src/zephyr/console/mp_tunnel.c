/*
 * STS1000 "Meridian" — Maintenance Protocol passthrough tunnels (console area).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Channels 0x02/0x03 are read-only tees of the GNSS receiver's NMEA and UBX
 * traffic; 0x07 and 0x08 are bidirectional passthroughs to the ZED-F9T on USART3
 * and the FE-5680A on UART7.
 *
 * The safety rule (FMT §5.5) is the whole reason this is a separate file: while a
 * tunnel is open, firmware must not drive that port, and the reference it feeds
 * becomes **suspect**. Opening one is therefore a guarded override on
 * `gnss.tunnel` / `ref.rb.tunnel`. Core reports `reference_suspect` in the
 * override reply; this file is what makes it true.
 *
 * GNSS (0x07): the receiver is genuinely stood down. Opening the tunnel calls
 * sts_gnss_uart_suspend() (sts_app.h), which stops the UBX parser and gnssmgr
 * from consuming the port and parks gnssmgr in GNSSMGR_ST_FW_UPDATE, so firmware
 * is not a second writer and a configuration retry cannot be aimed at whatever
 * the host is talking to. The RX ISR keeps filling the ring, and the GNSS thread
 * drains it straight into sts_mp_tee_gnss() for the duration. Closing resumes
 * and re-runs the configuration walk. The TAMPER alarm annunciates the window.
 *
 * Rb (0x08): also genuinely stood down, since this revision.
 * rb_serial_tunnel_open() takes UART7 away from rb_serial_ops() — its transmit
 * path answers -EBUSY and its receive ring stops being filled — and routes every
 * received octet to the sink registered here. That sink runs in the UART7 ISR,
 * which is exactly why the tee below is a bounded copy into a ring rather than a
 * call into the engine; see the threading block.
 *
 * A tunnel is *not* a subscription: it is opened by the override and closed by
 * releasing it, by the dead-man, or by leaving MP mode — all of which arrive here
 * as an apply/release from core. The host must ALSO subscribe to the channel to
 * receive the bytes; the override says "firmware may not drive this port", the
 * subscription says "and I want to watch it".
 *
 * ---------------------------------------------------------------------------
 * Threading (F11). Three sides now, and none of them may wait on another
 * ---------------------------------------------------------------------------
 *
 *   - the open/close side (sts_mp_tunnel_set_*) is called from core's apply
 *     callback and therefore already holds the engine lock. It is the only
 *     writer of the tunnel flags;
 *   - the **producer** side (sts_mp_tee_*) is called from platform/gnss.c on the
 *     priority-6 GNSS thread and from the UART7 receive ISR. Neither may take a
 *     mutex — Zephyr forbids k_mutex_lock() in an ISR at any timeout, and the
 *     GNSS thread feeds gnssmgr, which feeds the discipline loop, so parking it
 *     behind a console request is the priority inversion this whole area exists
 *     to prevent. A producer therefore does an arming test (one atomic read) and
 *     a bounded memcpy into `tee_ring` under a k_spinlock, and returns;
 *   - the **consumer** side (sts_mp_tunnel_drain) runs once per sts_mp_tick() on
 *     the console supervisor, before the tick takes the engine lock, and frames
 *     each staged record through sts_mp_stream_raw().
 *
 * core/util's ring_t is already single-producer/single-consumer safe (free
 * running atomic head/tail, head written only by the producer and tail only by
 * the consumer), so the spinlock is there for exactly one reason: there are
 * **two** producers — a thread and an ISR — and they must not interleave halves
 * of a record. The consumer needs no lock at all.
 *
 * The `..._open()` predicates and sts_mp_ch_armed() stay lock-free — the flags
 * are atomic words — so a reader asking "may I drive this port" or "is anyone
 * watching" never queues behind the console.
 *
 * Consequences, stated rather than hidden:
 *
 *   - latency. A byte reaches the host after up to one GNSS pass (50 ms) plus up
 *     to one console pass (250 ms). This is a diagnostic view and a maintenance
 *     tunnel, not a terminal;
 *   - loss. The ring is CONFIG_STS1000_MP_TEE_RING bytes and drops the newest on
 *     overflow, counted in `mp status`. A tee that back-pressured its producer
 *     would be a console request throttling the timing path;
 *   - the host->device direction of 0x07/0x08 is still not wired: core/mp does
 *     not dispatch inbound bytes on a passthrough channel to a sink, so
 *     sts_gnss_uart_raw_tx() and rb_serial_tunnel_write() have no caller here.
 *     The tunnels carry device->host today.
 */

#include <zephyr/kernel.h>

#if defined(CONFIG_STS1000_CONSOLE) && defined(CONFIG_STS1000_MP)

#include <errno.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "console/mp_glue.h"
#include "fault/fault.h"
#include "util/ring.h"
#include "zephyr/platform/platform.h"
#include "zephyr/sts_app.h"

LOG_MODULE_DECLARE(sts_mp, CONFIG_STS1000_LOG_LEVEL);

/* --------------------------------------------------------------- tunables */

/*
 * As everywhere in the console area, the Kconfig symbol carries an #if defined()
 * fallback equal to its default, so the file is correct whether or not
 * app/Kconfig sources src/zephyr/console/Kconfig.mp.
 */
#if defined(CONFIG_STS1000_MP_TEE_RING)
#define MP_TEE_RING_BYTES CONFIG_STS1000_MP_TEE_RING
#else
#define MP_TEE_RING_BYTES 1024
#endif

BUILD_ASSERT((MP_TEE_RING_BYTES & (MP_TEE_RING_BYTES - 1)) == 0,
	     "CONFIG_STS1000_MP_TEE_RING must be a power of two (ring_init)");

/** Record header in the staging ring: channel, then payload length. */
#define TEE_HDR 2U

/**
 * Longest payload one record carries.
 *
 * It bounds the drain's stack frame (TEE_HDR + this) and the time the producer
 * spinlock is held (one memcpy of this many bytes). 64 also matches the natural
 * batch on both producers: platform/gnss.c stages a 64-byte run before it hands
 * one over, and the UART7 ISR reads its FIFO 32 bytes at a time.
 */
#define TEE_CHUNK 64U

BUILD_ASSERT(MP_TEE_RING_BYTES >= (int)(4U * (TEE_HDR + TEE_CHUNK)),
	     "the tee ring cannot hold four full records");

/**
 * Records one drain pass will frame before giving the supervisor its loop back.
 *
 * 16 * 64 = 1024 B, so a full default ring empties in a single 250 ms pass at
 * the expected record size — 4 kB/s of tee throughput against a UBX message set
 * of a few hundred bytes per second and a 960 B/s rubidium link. The cap is not
 * about the ring, it is about mp_tx(): Zephyr's cdc_acm_poll_out() spins at
 * 1 ms/byte while the host is not draining its endpoint, so an unbounded drain
 * would hand a stalled host a lever on the console supervisor's liveness feed.
 */
#define TEE_DRAIN_RECS 16U

/* ------------------------------------------------------------------ state */

/*
 * Written only under the engine lock (sts_mp_tunnel_set_*), read lock-free by
 * peripheral threads. atomic_t rather than bool so the lock-free read is defined
 * behaviour rather than a compiler's favour.
 */
static atomic_t tunnel_gnss = ATOMIC_INIT(0);
static atomic_t tunnel_rb = ATOMIC_INIT(0);

/*
 * Counters, for `mp status` and the support bundle. Updated on whichever
 * peripheral thread produced the bytes and read from the shell, so they are
 * atomic: an unsynchronised read-modify-write across threads is a defect even
 * when the datum is only a statistic, and the cost is nothing next to the
 * spinlock on the same path. The per-channel counts are bytes *staged*;
 * tee_dropped is bytes that never made it, whether the ring was full or the
 * frame could not be sent.
 */
static atomic_t tee_gnss_bytes = ATOMIC_INIT(0);
static atomic_t tee_rb_bytes = ATOMIC_INIT(0);
static atomic_t tee_nmea_bytes = ATOMIC_INIT(0);
static atomic_t tee_ubx_bytes = ATOMIC_INIT(0);
static atomic_t tee_dropped = ATOMIC_INIT(0);

/** The staging ring, and the spinlock that serialises its two producers. */
static uint8_t tee_ring_buf[MP_TEE_RING_BYTES];
static ring_t tee_ring;
static struct k_spinlock tee_lock;
static bool tee_ready;

bool sts_mp_tunnel_gnss_open(void)
{
	return atomic_get(&tunnel_gnss) != 0;
}

bool sts_mp_tunnel_rb_open(void)
{
	return atomic_get(&tunnel_rb) != 0;
}

void sts_mp_tunnel_init(void)
{
	if (tee_ready) {
		return;
	}
	if (ring_init(&tee_ring, tee_ring_buf, sizeof(tee_ring_buf)) != 0) {
		/* Only reachable if the BUILD_ASSERT above were removed. */
		LOG_ERR("MP tee ring rejected (%u B); passthrough disabled",
			(unsigned int)sizeof(tee_ring_buf));
		return;
	}
	tee_ready = true;
}

/**
 * TAMPER tracks "some port is diverted", not "the last tunnel to change state".
 *
 * Both tunnels raise the same alarm, so neither may clear it on its own: closing
 * the GNSS tunnel while the rubidium one is still open would tell the operator
 * the board was untampered with while a technician still owned UART7. Called
 * with the engine lock held, which is also what makes the two flag reads a
 * consistent pair.
 */
static void tunnel_alarm_refresh(void)
{
	(void)sts_alarm_set(FAULT_ALARM_TAMPER,
			    sts_mp_tunnel_gnss_open() || sts_mp_tunnel_rb_open());
}

/**
 * Open or close the GNSS tunnel.
 *
 * Called from core's apply callback once the G2 guard and the interlocks have
 * passed. Ordering is chosen so the port never has two owners and the reference
 * is never believed while bytes are being diverted: on OPEN the receiver is stood
 * down first, then the tee is enabled and the alarm raised; on CLOSE the tee is
 * shut first, then the receiver is resumed and the alarm cleared.
 *
 * Balance. `tunnel_gnss` is the single record of who owns USART3, and it is
 * advanced only after the platform has agreed:
 *
 *   - a repeated set to the same state early-returns, so suspend/resume are
 *     never called twice in a row;
 *   - a FAILED open leaves `tunnel_gnss` false and does NOT resume (nothing was
 *     suspended), so the tee stays shut and the next open retries cleanly;
 *   - a failed CLOSE still clears `tunnel_gnss` and still clears the alarm: the
 *     host has released the override either way, and refusing to close would
 *     leave the port owned by a tunnel nobody can reach. The failure is logged at
 *     error level, and gnssmgr's own config walk recovers the receiver.
 *
 * -ENOTSUP from a build with no platform GNSS thread (fwupd_glue.c's __weak
 * fallbacks) is not treated as a failure to open: there is no receiver driving
 * the port, so there is nothing to stand down.
 */
int sts_mp_tunnel_set_gnss(bool open)
{
	int rc;

	if (sts_mp_tunnel_gnss_open() == open) {
		return 0;
	}

	if (open) {
		rc = sts_gnss_uart_suspend();
		if ((rc != 0) && (rc != -ENOTSUP)) {
			/* Firmware still owns the port: refuse rather than let two
			 * writers share USART3 (FMT §5.5). */
			sts_log(LOGR_SUB_GNSS, LOGR_ERR,
				"MP GNSS tunnel refused: receiver stand-down failed (%d)",
				rc);
			return rc;
		}
		atomic_set(&tunnel_gnss, 1);
		tunnel_alarm_refresh();
		sts_log(LOGR_SUB_GNSS, LOGR_WARN,
			"MP GNSS tunnel open; receiver stood down, reference SUSPECT");
		return 0;
	}

	atomic_set(&tunnel_gnss, 0);
	rc = sts_gnss_uart_resume();
	if ((rc != 0) && (rc != -ENOTSUP)) {
		sts_log(LOGR_SUB_GNSS, LOGR_ERR,
			"MP GNSS tunnel closed but receiver resume failed (%d)", rc);
	}
	tunnel_alarm_refresh();
	sts_log(LOGR_SUB_GNSS, LOGR_NOTICE,
		"MP GNSS tunnel closed; receiver resumed, reference trusted again");
	return 0;
}

/**
 * The UART7 receive sink, called **from the UART7 ISR** for every octet the
 * FE-5680A sends while the tunnel holds the port.
 *
 * It exists only to bridge rb_serial_tunnel_cb_t's user-pointer signature onto
 * the tee. Everything downstream of here is ISR-safe by construction: an atomic
 * read, then a memcpy under a spinlock.
 */
static void rb_tunnel_sink(void *user, const uint8_t *data, size_t len)
{
	ARG_UNUSED(user);
	sts_mp_tee_rb(data, len);
}

/**
 * Open or close the rubidium serial tunnel.
 *
 * Symmetric with the GNSS path, and for the same reason: while the host owns
 * UART7 firmware must not be a second writer. rb_serial_tunnel_open() is what
 * makes that true — rb_serial_ops()'s transmit path answers -EBUSY for the
 * duration, its receive ring stops being filled (so core/fwupd's rb_fwupd reads
 * nothing rather than half a diverted frame), and rb_serial_set_mode() refuses
 * to throw the K1 relay under a live session.
 *
 * The sink it demands runs in the UART7 ISR. That used to be the blocker — the
 * only useful thing to do with those octets was sts_mp_tee_rb(), which reached
 * mp_stream_raw() and the console UART through a mutex — and it is why the tee
 * is now an ISR-safe bounded copy drained from sts_mp_tick().
 *
 * Balance is the GNSS function's, with one extra window worth naming: on OPEN
 * the platform is committed before `tunnel_rb` is published, so octets that
 * arrive in between are dropped by sts_mp_tee_rb()'s own gate. At 9600 8N1 that
 * window is shorter than one character time and dropping is the correct side to
 * err on — the alternative is a flag that claims a tunnel the platform then
 * refused to open.
 */
int sts_mp_tunnel_set_rb(bool open)
{
	int rc;

	if (sts_mp_tunnel_rb_open() == open) {
		return 0;
	}

	if (open) {
		rc = rb_serial_tunnel_open(rb_tunnel_sink, NULL);
		if (rc != 0) {
			sts_log(LOGR_SUB_TIMING, LOGR_ERR,
				"MP Rb tunnel refused: UART7 stand-down failed (%d)",
				rc);
			return rc;
		}
		atomic_set(&tunnel_rb, 1);
		tunnel_alarm_refresh();
		sts_log(LOGR_SUB_TIMING, LOGR_WARN,
			"MP Rb tunnel open; UART7 stood down, reference SUSPECT");
		return 0;
	}

	atomic_set(&tunnel_rb, 0);
	rc = rb_serial_tunnel_close();
	if (rc != 0) {
		sts_log(LOGR_SUB_TIMING, LOGR_ERR,
			"MP Rb tunnel closed but UART7 resume failed (%d)", rc);
	}
	tunnel_alarm_refresh();
	sts_log(LOGR_SUB_TIMING, LOGR_NOTICE,
		"MP Rb tunnel closed; UART7 resumed, reference trusted again");
	return 0;
}

/* -------------------------------------------------------------------- tees */

/**
 * Stage @p len bytes of @p ch, in records of at most TEE_CHUNK.
 *
 * The producer half of the ring, and the only code in this file that runs on a
 * timing-adjacent thread or in an ISR. Cost, per record: one k_spin_lock (an
 * irq_lock/unlock pair on this single-core build), a ring_free() compare, two
 * ring_putc() and one ring_put() memcpy of up to TEE_CHUNK bytes — order 350
 * cycles for a 64-byte record at 250 MHz, i.e. ~1.4 us, with interrupts masked
 * for the memcpy alone. Nothing here blocks, allocates or waits.
 *
 * A record is committed whole or not at all, so the consumer can never observe
 * half of one and the two producers can never interleave. When it does not fit,
 * the NEWEST bytes are dropped and counted: overwriting the oldest would corrupt
 * a capture that is already partly delivered, and back-pressuring the caller
 * would let a console subscription throttle the GNSS receive path.
 *
 * @return Bytes accepted (0 .. @p len).
 */
static size_t tee_enqueue(uint8_t ch, const uint8_t *data, size_t len)
{
	size_t off = 0U;

	while (off < len) {
		k_spinlock_key_t key;
		size_t n = MIN(len - off, (size_t)TEE_CHUNK);
		bool ok;

		key = k_spin_lock(&tee_lock);
		ok = ring_free(&tee_ring) >= (n + TEE_HDR);
		if (ok) {
			(void)ring_putc(&tee_ring, ch);
			(void)ring_putc(&tee_ring, (uint8_t)n);
			(void)ring_put(&tee_ring, &data[off], n);
		}
		k_spin_unlock(&tee_lock, key);

		if (!ok) {
			break;
		}
		off += n;
	}

	if (off < len) {
		(void)atomic_add(&tee_dropped, (atomic_val_t)(len - off));
	}
	return off;
}

/**
 * Stage @p len bytes of @p ch when someone is listening, counting what landed.
 *
 * The arming test is what keeps an unsubscribed channel free: with no consumer
 * this is one atomic read and a return, so leaving the tee calls in the GNSS
 * receive path costs nothing when no technician is attached.
 */
static void tee(uint8_t ch, const uint8_t *data, size_t len, atomic_t *counter)
{
	size_t took;

	if ((data == NULL) || (len == 0U) || !tee_ready ||
	    !sts_mp_ch_armed(ch)) {
		return;
	}
	took = tee_enqueue(ch, data, len);
	if (took != 0U) {
		(void)atomic_add(counter, (atomic_val_t)took);
	}
}

void sts_mp_tee_gnss(const uint8_t *data, size_t len)
{
	if (sts_mp_tunnel_gnss_open()) {
		tee(MP_CH_GNSS_PASS, data, len, &tee_gnss_bytes);
	}
}

void sts_mp_tee_rb(const uint8_t *data, size_t len)
{
	if (sts_mp_tunnel_rb_open()) {
		tee(MP_CH_RB_PASS, data, len, &tee_rb_bytes);
	}
}

void sts_mp_tee_nmea(const uint8_t *data, size_t len)
{
	tee(MP_CH_NMEA, data, len, &tee_nmea_bytes);
}

void sts_mp_tee_ubx(const uint8_t *data, size_t len)
{
	tee(MP_CH_UBX, data, len, &tee_ubx_bytes);
}

/**
 * Frame staged records onto the wire. Consumer side; see the threading block.
 *
 * Peek, send, then discard — rather than pop and send — so a record survives an
 * engine lock this pass could not get. sts_mp_stream_raw() is the locked entry
 * point; this file never holds a pointer to the engine context, because a caller
 * that does can mutate protocol state with no lock at all.
 *
 * -ENOENT is the normal "the host unsubscribed while this was queued" case and
 * -ENODEV is "the engine has stopped": both discard the record silently, because
 * neither is a loss the operator can act on.
 */
void sts_mp_tunnel_drain(void)
{
	unsigned int records;

	if (!tee_ready) {
		return;
	}

	for (records = 0U; records < TEE_DRAIN_RECS; records++) {
		uint8_t rec[TEE_HDR + TEE_CHUNK];
		size_t got;
		size_t n;
		int rc;

		got = ring_peek(&tee_ring, rec, sizeof(rec));
		if (got < TEE_HDR) {
			return; /* empty */
		}
		n = rec[1];
		if ((n == 0U) || (n > TEE_CHUNK) || (got < (TEE_HDR + n))) {
			/*
			 * Unreachable: tee_enqueue() commits a record whole under
			 * the producer spinlock, so the ring holds complete
			 * records or nothing. If the invariant ever broke, the
			 * only safe move is to drop the backlog rather than frame
			 * the ring's contents as if they were payload.
			 */
			(void)atomic_add(&tee_dropped,
					 (atomic_val_t)ring_len(&tee_ring));
			ring_reset(&tee_ring);
			LOG_ERR("MP tee ring desynchronised; backlog dropped");
			return;
		}

		rc = sts_mp_stream_raw(rec[0], &rec[TEE_HDR], n);
		if (rc == -EBUSY) {
			/* The engine lock is contended, or something called this
			 * from an ISR. Leave the record queued and end the pass:
			 * fifteen more attempts would only lengthen it. */
			return;
		}
		if ((rc < 0) && (rc != -ENOENT) && (rc != -ENODEV)) {
			(void)atomic_add(&tee_dropped, (atomic_val_t)n);
		}
		(void)ring_discard(&tee_ring, TEE_HDR + n);
	}
}

/** Tee byte counts, for `mp status`. Lock-free; needs no engine lock. */
void sts_mp_tunnel_stats(uint32_t *gnss, uint32_t *rb, uint32_t *nmea,
			 uint32_t *ubx, uint32_t *dropped)
{
	if (gnss != NULL) {
		*gnss = (uint32_t)atomic_get(&tee_gnss_bytes);
	}
	if (rb != NULL) {
		*rb = (uint32_t)atomic_get(&tee_rb_bytes);
	}
	if (nmea != NULL) {
		*nmea = (uint32_t)atomic_get(&tee_nmea_bytes);
	}
	if (ubx != NULL) {
		*ubx = (uint32_t)atomic_get(&tee_ubx_bytes);
	}
	if (dropped != NULL) {
		*dropped = (uint32_t)atomic_get(&tee_dropped);
	}
}

#endif /* CONFIG_STS1000_CONSOLE && CONFIG_STS1000_MP */
