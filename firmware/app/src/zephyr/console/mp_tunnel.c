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
 * the host is talking to. The RX ISR keeps filling the ring, which is what
 * sts_gnss_uart_raw_rx() drains, and sts_gnss_uart_raw_tx() only works inside a
 * suspended session — so the suspend is a prerequisite for the tunnel to carry
 * host->device bytes at all, not merely a courtesy. Closing resumes and re-runs
 * the configuration walk. The TAMPER alarm annunciates the whole window.
 *
 * Rb (0x08): NOT stood down. platform/rb_serial.c does have an equivalent
 * (rb_serial_tunnel_open/close), but it is declared in the platform area's
 * private platform.h and demands an ISR-context byte-sink callback rather than a
 * bare suspend, so wiring it needs a byte pump this area does not have. The Rb
 * path therefore keeps its alarm-only behaviour and says so; see
 * sts_mp_tunnel_set_rb().
 *
 * A tunnel is *not* a subscription: it is opened by the override and closed by
 * releasing it, by the dead-man, or by leaving MP mode — all of which arrive here
 * as an apply/release from core.
 *
 * Threading (F11). Two sides, and they are not symmetric:
 *
 *   - the open/close side (sts_mp_tunnel_set_*) is called from core's apply
 *     callback and therefore already holds the engine lock. It is the only
 *     writer of the tunnel flags;
 *   - the tee side is called by peripheral reader threads and reaches the engine
 *     through sts_mp_stream_raw(), which takes the lock with a timeout and drops
 *     the bytes on contention. A reader thread must not be pinned behind a
 *     console request, and a passthrough tee is best-effort by nature.
 *
 * The `..._open()` predicates stay lock-free — the flags are atomic words — so a
 * reader asking "may I drive this port" never queues behind the console. That is
 * the priority inversion this split exists to prevent.
 */

#include <zephyr/kernel.h>

#ifdef CONFIG_STS1000_CONSOLE

#include <errno.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "console/mp_glue.h"
#include "fault/fault.h"
#include "zephyr/sts_app.h"

LOG_MODULE_DECLARE(sts_mp, CONFIG_STS1000_LOG_LEVEL);

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
 * when the datum is only a statistic, and the cost is nothing next to the mutex
 * and the UART on the same path.
 */
static atomic_t tee_gnss_bytes = ATOMIC_INIT(0);
static atomic_t tee_rb_bytes = ATOMIC_INIT(0);
static atomic_t tee_nmea_bytes = ATOMIC_INIT(0);
static atomic_t tee_ubx_bytes = ATOMIC_INIT(0);
static atomic_t tee_dropped = ATOMIC_INIT(0);

bool sts_mp_tunnel_gnss_open(void)
{
	return atomic_get(&tunnel_gnss) != 0;
}

bool sts_mp_tunnel_rb_open(void)
{
	return atomic_get(&tunnel_rb) != 0;
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
		(void)sts_alarm_set(FAULT_ALARM_TAMPER, true);
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
	(void)sts_alarm_set(FAULT_ALARM_TAMPER, false);
	sts_log(LOGR_SUB_GNSS, LOGR_NOTICE,
		"MP GNSS tunnel closed; receiver resumed, reference trusted again");
	return 0;
}

/**
 * Open or close the rubidium serial tunnel.
 *
 * Alarm-and-flag only, unlike the GNSS path: firmware keeps reading UART7 while
 * this is open. That is a deliberate stop, not an oversight.
 * platform/rb_serial.c does expose a suspend/resume pair — rb_serial_tunnel_open()
 * / rb_serial_tunnel_close(), which take the port away from rb_serial_ops() and
 * make its transmit path answer -EBUSY — but it cannot be wired from here:
 *
 *   1. it is declared in src/zephyr/platform/platform.h, which is private to the
 *      platform area (ARCHITECTURE.md §2), so it needs an sts_app.h entry point;
 *   2. rb_serial_tunnel_open() takes a mandatory callback that the UART7 ISR
 *      invokes for every received octet. The only useful thing to do with those
 *      octets is sts_mp_tee_rb(), which reaches mp_stream_raw() and the console
 *      UART — not ISR-safe, and it mutates engine state shared with the shell
 *      bypass. Wiring it properly needs a ring plus a drain on the MP tick, i.e.
 *      a byte pump, which is a larger change than a stand-down.
 *
 * Until that exists the Rb reference is annunciated as suspect and the operator
 * is warned, which is what the log line below says — no more.
 */
int sts_mp_tunnel_set_rb(bool open)
{
	if (sts_mp_tunnel_rb_open() == open) {
		return 0;
	}
	atomic_set(&tunnel_rb, open ? 1 : 0);

	sts_log(LOGR_SUB_TIMING, open ? LOGR_WARN : LOGR_NOTICE,
		"MP Rb tunnel %s; reference %s (firmware still reads UART7)",
		open ? "open" : "closed", open ? "SUSPECT" : "trusted");
	return 0;
}

/* -------------------------------------------------------------------- tees */

/**
 * Frame @p len bytes onto @p ch, counting a refusal rather than retrying.
 *
 * sts_mp_stream_raw() is the locked entry point; this file never holds a pointer
 * to the engine context, because a caller that does can mutate protocol state
 * with no lock at all. -EBUSY (lock contended, or an ISR caller) is counted with
 * the other drops: a tee that waits is a peripheral reader that stalls.
 */
static void tee(uint8_t ch, const uint8_t *data, size_t len, atomic_t *counter)
{
	int rc;

	if ((data == NULL) || (len == 0U)) {
		return;
	}
	rc = sts_mp_stream_raw(ch, data, len);
	if (rc < 0) {
		if ((rc != -ENOENT) && (rc != -ENODEV)) {
			/* -ENOENT is the normal "nobody is listening" case, and
			 * -ENODEV is "the engine has not started". */
			(void)atomic_inc(&tee_dropped);
		}
		return;
	}
	(void)atomic_add(counter, (atomic_val_t)len);
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

#endif /* CONFIG_STS1000_CONSOLE */
