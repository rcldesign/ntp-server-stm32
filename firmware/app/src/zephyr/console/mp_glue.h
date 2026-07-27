/*
 * STS1000 "Meridian" — Maintenance Protocol glue (console area).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/console/, like sts_console.h. `core/mp` is the protocol;
 * this file is the wire under it, the shell hand-off, and the provider callbacks
 * that turn platform snapshots into the structs core consumes.
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
 * Takes the engine lock with a STS_MP_TICK_LOCK_MS timeout and drops the bytes
 * on contention rather than waiting: a tee is best-effort passthrough, and a
 * peripheral reader must not be pinned behind a console request.
 *
 * @retval >=0      Bytes accepted.
 * @retval -ENOENT  Channel not subscribed (normal; nothing was sent).
 * @retval -EBUSY   Lock contended, or called from an ISR — bytes dropped.
 * @retval <0       As mp_stream_raw().
 */
int sts_mp_stream_raw(uint8_t ch, const uint8_t *data, size_t len);

/**
 * Tee bytes from a peripheral port onto its MP channel (mp_tunnel.c).
 *
 * The GNSS and Rb readers call these with every byte they receive; nothing is
 * sent unless the channel is subscribed or its tunnel object is overridden.
 *
 * **Thread context, not ISR context.** These reach mp_stream_raw() and the
 * console UART through the engine mutex, and Zephyr forbids k_mutex_lock() in an
 * ISR outright. A byte sink invoked from a UART ISR (rb_serial.c's is) must ring
 * the octets and drain them from a thread — see mp_tunnel.c. The ISR case is
 * detected and counted as a drop rather than left to an assertion that a release
 * build compiles out.
 */
void sts_mp_tee_gnss(const uint8_t *data, size_t len);
void sts_mp_tee_rb(const uint8_t *data, size_t len);
void sts_mp_tee_nmea(const uint8_t *data, size_t len);
void sts_mp_tee_ubx(const uint8_t *data, size_t len);

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

/** Tee byte counts, for `mp status` and the support bundle. */
void sts_mp_tunnel_stats(uint32_t *gnss, uint32_t *rb, uint32_t *nmea,
			 uint32_t *ubx, uint32_t *dropped);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_CONSOLE_MP_GLUE_H_ */
