/*
 * STS1000 "Meridian" — Maintenance Protocol glue (console area).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * `core/mp` is the protocol; this file is the wire under it, the shell hand-off,
 * and the provider callbacks that turn platform snapshots into the structs core
 * consumes.
 *
 * Mostly private to src/zephyr/console/, like sts_console.h — with one
 * deliberate exception, stated here so it is a contract rather than a leak. The
 * **byte tees and their arming predicates** (`sts_mp_tee_*`,
 * `sts_mp_gnss_tee_armed`, `sts_mp_ch_armed`, `sts_mp_tunnel_*_open`) are called
 * from the platform area, because that is where the bytes are:
 * platform/gnss.c owns the USART3 receive path and platform/rb_serial.c owns
 * UART7. sts_mp_mirror_publish() crosses the same seam in the other direction
 * and is declared in sts_app.h instead; these are not, because sts_app.h is not
 * this change's to extend. The dependency is made safe two ways:
 *
 *   - src/zephyr/platform/sts_area_weak.c carries a __weak no-op for **every**
 *     function declared below, so CONFIG_STS1000_MP=n (which drops mp_glue.c
 *     and mp_tunnel.c from the build, app/CMakeLists.txt) still links;
 *   - every one of them is safe to call from a hot loop or an ISR: the tees do
 *     a bounded copy into a staging ring behind a spinlock and nothing else,
 *     and the predicates are one atomic read. Nothing on this path takes the
 *     engine mutex or touches the console UART.
 *
 * Mode entry (FMT §2.3). MP shares the human console on CDC-ACM #0 rather than
 * taking a third endpoint, so entering it means taking the port away from the
 * Zephyr shell. That is done with `shell_set_bypass()`, which exists for exactly
 * this: while the bypass is installed every received byte goes to mp_input() and
 * the shell parses nothing. `mp exit`, the in-band exit magic, a BREAK and a DTR
 * drop all clear it.
 *
 * The engine is started by sts_console_start() and serviced by the console
 * supervisor; the panel mirror is published by the ui area. See the wiring block
 * at the top of mp_glue.c for the one hook still missing (the raw shell tap for
 * the autobaud entry magic) and for the object writes that remain unbound.
 *
 * ---------------------------------------------------------------------------
 * Threading (F11). The engine has two independent drivers
 * ---------------------------------------------------------------------------
 *
 * RX is byte-driven on the **Zephyr shell thread** (the bypass callback), and
 * the service tick is time-driven on the **console supervisor thread** (prio 14,
 * ARCHITECTURE.md §6). 14 preempts the shell's K_LOWEST_APPLICATION_THREAD_PRIO
 * (19 with CONFIG_NUM_PREEMPT_PRIORITIES=20), so the tick lands *inside*
 * mp_input() unless something stops it. It cannot be folded onto the RX side:
 * the dead-man exists for the case where the host stops sending, so a
 * byte-driven tick fails in its own design case.
 *
 * Every entry into core/mp therefore runs under one glue-owned mutex, and every
 * function below states which side of it the caller is on. core/mp itself stays
 * lock-free and platform-neutral — the lock is here, not there.
 */

#ifndef STS1000_ZEPHYR_CONSOLE_MP_GLUE_H_
#define STS1000_ZEPHYR_CONSOLE_MP_GLUE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mp/mp.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------- tick budget */

/**
 * How long sts_mp_tick() waits for the engine lock before skipping a pass.
 *
 * It must be a timeout and not K_FOREVER: the same supervisor loop feeds
 * sts_liveness_feed(), and a supervisor blocked past
 * CONFIG_STS1000_LIVENESS_DEADLINE_MS stops kicking the TPS3430 — i.e. an
 * unbounded wait here cold-cycles the board.
 */
#define STS_MP_TICK_LOCK_MS 50U

/**
 * Consecutive skipped passes the dead-man tolerates.
 *
 * At a period P the worst interval between two *successful* ticks with N
 * consecutive misses is `(N + 1) * (P + STS_MP_TICK_LOCK_MS)`, and that must
 * stay inside MP_TICK_MAX_MS (2000 ms). With P = 250 ms:
 * `(5 + 1) * (250 + 50) = 1800 <= 2000`. sts_console.c turns that inequality
 * into a BUILD_ASSERT against its own period, so changing either number breaks
 * the build rather than the dead-man.
 */
#define STS_MP_TICK_MISS_MAX 5U

/**
 * Initialise the MP engine and register the `mp` shell command.
 *
 * Called from the console area's start function, after cfg and the log ring are
 * available. Creates no thread of its own: MP is driven by the shell bypass on
 * the RX side and by sts_mp_tick() from the console supervisor.
 *
 * @retval 0        Ready (still in shell mode).
 * @retval -ENODEV  The console UART is not ready.
 * @retval <0       mp_init() failed.
 */
int sts_mp_start(void);

/**
 * Periodic service: the dead-man, lease expiry, diagnostics and due streams.
 *
 * Must be called at least every MP_TICK_MAX_MS (2 s) so an override can never
 * outlive its keepalive by more than the spec's revert budget. The console
 * supervisor calls it unconditionally on every pass of its 250 ms loop.
 *
 * Takes the engine lock with a STS_MP_TICK_LOCK_MS timeout and **skips the pass**
 * rather than waiting, so the caller's own deadlines (liveness, watchdog) are
 * never hostage to the shell thread. A skip is counted and reported through
 * `mp status`, and an excursion past STS_MP_TICK_MISS_MAX consecutive skips is
 * logged at error level — a persistently missed tick is an operational fault,
 * not a hiccup.
 *
 * A no-op before sts_mp_start(), so the caller needs no gate. Not ISR-safe.
 */
void sts_mp_tick(void);

/** True while the console is in MP mode (the shell is bypassed). Takes the lock. */
bool sts_mp_active(void);

/**
 * Report a DTR change; a drop is a dead-man failure and leaves MP mode.
 *
 * Takes the engine lock. Not ISR-safe: call it from the thread that polls the
 * line state, not from a USB callback.
 */
void sts_mp_notify_link(bool up);

/** Report a UART BREAK: leaves MP mode (FMT §2.3). Takes the lock; not ISR-safe. */
void sts_mp_notify_break(void);

/**
 * Feed one raw console byte while the shell owns the port, so the autobaud entry
 * magic (`\x01MP1\x02`) can be recognised without the shell cooperating.
 *
 * Returns true when the magic completed and MP has taken the port — the caller
 * must then stop handing the byte to the shell. See the TODO in mp_glue.c: the
 * console area cannot install this tap itself.
 *
 * Takes the engine lock, so it must be called from the shell's RX *thread*, not
 * from the UART ISR.
 */
bool sts_mp_shell_tap(uint8_t b);

/* ------------------------------------------------------------------ mirror */

/**
 * Publish the front-panel frame the mirror streams (project requirement).
 *
 * The ui area calls this after each render with its text-tile surface, lamp
 * state and input echo. Before the first render `mirror.get` and channel 0x0A
 * answer "not supported" rather than inventing a panel. The frame is copied, so
 * the caller may reuse its surface immediately.
 *
 * Publisher side only, and deliberately **outside** the engine lock: the ui
 * thread must never be able to block behind a console request. Safe from the ui
 * thread; takes a short mutex of its own shared with the MP tick's reader.
 * Neither side waits on the other: this drops the frame if the tick holds the
 * lock, and the tick's reader answers -EBUSY if this holds it.
 */
void sts_mp_mirror_publish(const mp_mirror_in_t *frame);

/* ------------------------------------------------------------------ tunnels */

/**
 * Frame raw bytes onto a passthrough channel (mp_tunnel.c's tee, and any other
 * producer that has to reach mp_stream_raw()).
 *
 * This is the *only* way into the engine from outside this area — the engine
 * context itself is not exported, because a caller holding `mp_ctx_t *` could
 * mutate protocol state with no lock at all.
 *
 * Tries the engine lock with **K_NO_WAIT** and refuses on contention rather than
 * waiting: a tee is best-effort passthrough, and its caller runs inside
 * sts_mp_tick()'s pass, which sts_console.c's BUILD_ASSERT budgets for exactly
 * one lock wait (the tick's own). See the argument at the call to
 * k_mutex_lock() in mp_glue.c — it is the dead-man's revert deadline, not a
 * preference.
 *
 * @retval >=0      Bytes accepted.
 * @retval -ENOENT  Channel not subscribed (normal; nothing was sent).
 * @retval -EBUSY   Lock held by another thread, or called from an ISR —
 *                  nothing was sent; the caller keeps the bytes.
 * @retval <0       As mp_stream_raw().
 */
int sts_mp_stream_raw(uint8_t ch, const uint8_t *data, size_t len);

/**
 * Tee bytes from a peripheral port onto its MP channel (mp_tunnel.c).
 *
 * The GNSS and Rb readers call these with the bytes they move; nothing is
 * staged unless the channel is subscribed *and*, for the two passthrough
 * channels, its tunnel object is overridden open.
 *
 * **Safe from an ISR and from the priority-6 GNSS thread**, which is the whole
 * point of them: a call is an arming test (one atomic read), then a bounded
 * memcpy into the staging ring under a k_spinlock, then return. It never takes
 * the engine mutex, never touches the console UART and never blocks — the
 * framing happens later, on the console supervisor, out of sts_mp_tick().
 * Anything that does not fit is dropped and counted, because a passthrough tee
 * is best-effort by nature and a peripheral reader that stalls is not.
 *
 * Cost, since one of these sits next to the timing path: ~15 instructions per
 * byte staged by the caller plus ~350 cycles per chunk handed over here (see
 * the accounting in mp_tunnel.c's tee_enqueue()).
 */
void sts_mp_tee_gnss(const uint8_t *data, size_t len);
void sts_mp_tee_rb(const uint8_t *data, size_t len);
void sts_mp_tee_nmea(const uint8_t *data, size_t len);
void sts_mp_tee_ubx(const uint8_t *data, size_t len);

/**
 * True when @p ch has a consumer: MP mode is up and the host is subscribed.
 *
 * Lock-free — a single atomic read of a bitmask the engine republishes under
 * its own lock on every sts_mp_tick() and after every input batch, so it lags a
 * subscription change by at most one tick. A stale "armed" costs one staged
 * chunk that the drain then discards; a stale "not armed" costs one tick of
 * missing tee data. Neither is worth a lock on a producer this hot.
 *
 * @p ch is an MP_CH_* value; anything above 31 reads as not armed.
 */
bool sts_mp_ch_armed(uint8_t ch);

/**
 * True when any GNSS-side tee has a consumer — 0x02 NMEA, 0x03 UBX or the 0x07
 * passthrough.
 *
 * The one predicate platform/gnss.c evaluates per drain pass to decide whether
 * the receive path pays for the copy at all. Same lock-free single atomic read
 * as sts_mp_ch_armed().
 */
bool sts_mp_gnss_tee_armed(void);

/**
 * True while the host holds a tunnel open on that port, in which case firmware
 * must not drive it and the reference it feeds is suspect (FMT §5.5).
 *
 * Lock-free by design: the callers are GNSS/Rb reader threads asking "may I
 * drive this port", and making them queue behind a console request to find out
 * would be the priority inversion this whole area exists to avoid. The flag is a
 * single atomic word, written only under the engine lock.
 */
bool sts_mp_tunnel_gnss_open(void);
bool sts_mp_tunnel_rb_open(void);

/**
 * Open or close a tunnel. Called from core's apply callback once the G2 guard
 * and the interlocks have passed; raises the reference-suspect state for the
 * duration (FMT §5.5).
 *
 * **Called with the engine lock already held** (obj_apply() runs inside the
 * engine). Do not call it from anywhere that does not hold the lock: the
 * open/close flag is a read-modify-write, and a second writer would desynchronise
 * it from the platform suspend/resume it is the sole record of.
 */
int sts_mp_tunnel_set_gnss(bool open);
int sts_mp_tunnel_set_rb(bool open);

/**
 * Put a host-supplied burst on the port behind a passthrough channel.
 *
 * The host->device half of channels 0x07 and 0x08. Core reaches it through
 * mp_wiring_t::raw_tx, having already established that the channel's tunnel
 * object holds a live override lease — i.e. that firmware has been stood down on
 * that UART (FMT §5.5). This function checks the same thing again from the
 * platform's side and refuses anything longer than MP_TUNNEL_TX_MAX.
 *
 * **Called with the engine lock held**, and it blocks for one character time per
 * octet on a `uart_poll_out()` loop. That is why the burst is bounded; see the
 * budget argument in mp_tunnel.c's header.
 *
 * @retval >=0        Octets written.
 * @retval -ENOTSUP   Not a writable passthrough channel.
 * @retval -EPERM     The tunnel is not open (or the platform says it is not).
 * @retval -EMSGSIZE  Over MP_TUNNEL_TX_MAX; nothing was written.
 */
int sts_mp_tunnel_write(uint8_t ch, const uint8_t *data, size_t len);

/** Tee byte counts, for `mp status` and the support bundle. */
void sts_mp_tunnel_stats(uint32_t *gnss, uint32_t *rb, uint32_t *nmea,
			 uint32_t *ubx, uint32_t *dropped);

/** Host->device passthrough counts: octets written per port, and refusals. */
void sts_mp_tunnel_tx_stats(uint32_t *gnss, uint32_t *rb, uint32_t *refused);

/**
 * Empty the tee staging ring onto the wire.
 *
 * Console-internal: sts_mp_tick() calls it once per pass, **before** it takes
 * the engine lock, so a contended engine costs a deferred drain and not a
 * deferred tick. Each record is framed through sts_mp_stream_raw(), which takes
 * the lock itself; the first -EBUSY ends the pass and leaves the rest queued.
 *
 * Thread context only, and only from one thread: the ring's consumer side is
 * single-consumer by construction.
 */
void sts_mp_tunnel_drain(void);

/**
 * Bind the tee staging ring. Console-internal; sts_mp_start() calls it before
 * it publishes the engine, i.e. before any producer can be armed.
 */
void sts_mp_tunnel_init(void);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_CONSOLE_MP_GLUE_H_ */
