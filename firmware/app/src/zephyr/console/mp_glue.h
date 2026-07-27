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
 * supervisor calls it unconditionally on every pass of its 250 ms loop, which is
 * 8x inside that. A no-op before sts_mp_start(), so the caller needs no gate.
 */
void sts_mp_tick(void);

/** True while the console is in MP mode (the shell is bypassed). */
bool sts_mp_active(void);

/** Report a DTR change; a drop is a dead-man failure and leaves MP mode. */
void sts_mp_notify_link(bool up);

/** Report a UART BREAK: leaves MP mode (FMT §2.3). */
void sts_mp_notify_break(void);

/** The engine context, for `sts diag` and the shell. NULL before start. */
const mp_ctx_t *sts_mp_ctx(void);

/**
 * Feed one raw console byte while the shell owns the port, so the autobaud entry
 * magic (`\x01MP1\x02`) can be recognised without the shell cooperating.
 *
 * Returns true when the magic completed and MP has taken the port — the caller
 * must then stop handing the byte to the shell. See the TODO in mp_glue.c: the
 * console area cannot install this tap itself.
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
 * Safe from the ui thread; takes a short mutex shared with the MP tick. Neither
 * side waits on the other: this drops the frame if the tick holds the lock, and
 * the tick's reader answers -EBUSY if this holds it.
 */
void sts_mp_mirror_publish(const mp_mirror_in_t *frame);

/* ------------------------------------------------------------------ tunnels */

/**
 * Tee bytes from a peripheral port onto its MP channel (mp_tunnel.c).
 *
 * The GNSS and Rb readers call these with every byte they receive; nothing is
 * sent unless the channel is subscribed or its tunnel object is overridden.
 */
void sts_mp_tee_gnss(const uint8_t *data, size_t len);
void sts_mp_tee_rb(const uint8_t *data, size_t len);
void sts_mp_tee_nmea(const uint8_t *data, size_t len);
void sts_mp_tee_ubx(const uint8_t *data, size_t len);

/**
 * True while the host holds a tunnel open on that port, in which case firmware
 * must not drive it and the reference it feeds is suspect (FMT §5.5).
 */
bool sts_mp_tunnel_gnss_open(void);
bool sts_mp_tunnel_rb_open(void);

/**
 * Open or close a tunnel. Called from core's apply callback once the G2 guard
 * and the interlocks have passed; raises the reference-suspect state for the
 * duration (FMT §5.5).
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
