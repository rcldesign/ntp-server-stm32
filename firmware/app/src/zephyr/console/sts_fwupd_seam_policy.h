/*
 * STS1000 "Meridian" — console area: the firmware-update seam's decisions.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/console/, and deliberately free of every Zephyr
 * dependency so tests/host can compile it — the same arrangement as
 * console/sts_mp_tunnel_policy.h and net/sts_secops_policy.h, for the same
 * reason: every way these four decisions can be wrong is silent at runtime.
 *
 * fwupd_glue.c owns the wires (the ops tables, the mutex, the MP port). This
 * header owns the four judgements those wires are steered by:
 *
 *   1. WHAT A TRANSMIT RETURN MEANS. core/fwupd's ubx_fwupd_ops_t::tx is
 *      specified "0 on success"; the platform sink it is bound to,
 *      sts_gnss_uart_raw_tx(), returns the OCTET COUNT — and must keep doing
 *      so, because mp_tunnel.c's host->device path reports that count to the
 *      maintenance host. Two sinks, two conventions, one of which has to be
 *      converted at the seam. Getting it backwards reads every successful
 *      transmit as a failure AND propagates a byte count where an errno is
 *      expected, so the caller's `rc` is 8 rather than 0 or -EPERM.
 *
 *   2. WHETHER A TRANSPORT EXISTS. The USART3 raw hooks are __weak stubs in
 *      fwupd_glue.c that platform/gnss.c overrides. "Is the strong symbol
 *      linked" and "is the GNSS area running" are DIFFERENT questions with
 *      different answers (-ENOTSUP vs -ENODEV), and collapsing them into
 *      `!= -ENOTSUP` reports a wired transport before the receiver thread has
 *      started — which turns gnss_prepare()'s documented fail-safe into a
 *      refusal that never fires.
 *
 *   3. WHETHER AN UNAUTHENTICATED INVENTORY READ MAY TOUCH THE BUS. `fw.inventory`
 *      is G0 (FMT §5.1: observation is free) and that is correct — a board whose
 *      component list needs a credential is a board whose operator cannot tell
 *      what is wrong with it. But "observation" must not mean "writes to a port
 *      another session owns". While a passthrough tunnel holds USART3 or UART7,
 *      an identity read would inject its own frame into somebody else's session
 *      — possibly into a receiver's bootloader — and then spend its reply budget
 *      draining that session's bytes out of the tee ring.
 *
 *   4. HOW MUCH WALL CLOCK ONE SUPERVISOR PASS MAY SPEND ON THE ORCHESTRATOR.
 *      sts_console.c's dead-man BUILD_ASSERT budgets the worst interval between
 *      two successful sts_mp_tick() calls. sts_fwupd_step() shares that loop and
 *      spends no lock wait, but a lock wait is not the only way to run long:
 *      fwupd_step() fires the step and transfer-idle timeouts, and those call
 *      finish() -> restore(), which is an internal-flash trailer write for the
 *      STM32 target and 20 ms of k_msleep for the GNSS one.
 *
 * None of the four produces a crash, a log line or a failing request when it is
 * wrong, which is why all four are here as pure functions with one call site
 * each, pinned by tests/host/test_fwupd_seam_policy.c.
 */

#ifndef STS1000_ZEPHYR_CONSOLE_STS_FWUPD_SEAM_POLICY_H_
#define STS1000_ZEPHYR_CONSOLE_STS_FWUPD_SEAM_POLICY_H_

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================== *
 *  1. the transmit contract
 * ===================================================================== */

/**
 * Convert a count-returning transmit sink onto the "0 on success" ops contract.
 *
 * @param rc   What the sink returned: octets written, or a negative errno.
 * @param len  Octets the caller asked for.
 *
 * @retval 0        The whole burst went out.
 * @retval negative The sink's own errno, passed through unchanged so the caller
 *                  can still tell -EPERM ("the port is not yours") from -ENODEV
 *                  ("there is no port").
 * @retval -EIO     A non-negative return that is not @p len. Both sinks behind
 *                  this are all-or-nothing uart_poll_out() loops, so a short
 *                  count is not a partial write to be resumed — it means the
 *                  sink is not the one this seam was written against, and
 *                  continuing would silently drop octets out of a flash image.
 *
 * Note the direction: this normalises COUNT -> STATUS. Handing it a sink that
 * already returns 0 on success turns every non-empty success into -EIO, which
 * is why the Rb ops table (rb_serial.c op_tx, 0 on success) does not use it.
 */
static inline int sts_fwupd_tx_from_count(int rc, size_t len)
{
	if (rc < 0) {
		return rc;
	}
	return ((size_t)rc == len) ? 0 : -EIO;
}

/* ===================================================================== *
 *  2. transport classification
 * ===================================================================== */

/** What the raw-transport capability probe found. */
typedef enum {
	/** The __weak stub answered: platform/gnss.c is not in this build. */
	STS_FWUPD_XPORT_ABSENT = 0,
	/** The hooks are linked, but the owning area is not running yet. */
	STS_FWUPD_XPORT_DOWN,
	/** Linked and running: raw access is possible right now. */
	STS_FWUPD_XPORT_READY,
} sts_fwupd_xport_t;

/**
 * Classify the answer to sts_gnss_uart_raw_rx(NULL, 0) — the capability probe.
 *
 * Three outcomes, and they are three because two of them are not the same
 * thing:
 *
 *   -ENOTSUP  only the __weak stub can produce it, so it is the definitive
 *             "this image has no GNSS transport at all";
 *   negative  anything else is the strong symbol refusing for its own reason,
 *             today -ENODEV for "the GNSS thread has not started". The hooks
 *             exist; they are just not usable yet;
 *   >= 0      the probe drained zero octets from a running receiver, which is
 *             the only answer that means "you may use this now".
 *
 * Probing rather than a compile-time test is deliberate: the strong symbols may
 * be linked without this file knowing, and asking costs one call.
 */
static inline sts_fwupd_xport_t sts_fwupd_xport_classify(int probe_rc)
{
	if (probe_rc == -ENOTSUP) {
		return STS_FWUPD_XPORT_ABSENT;
	}
	if (probe_rc < 0) {
		return STS_FWUPD_XPORT_DOWN;
	}
	return STS_FWUPD_XPORT_READY;
}

/** Short name for a transport state, for the boot log. Never NULL. */
static inline const char *sts_fwupd_xport_name(sts_fwupd_xport_t x)
{
	switch (x) {
	case STS_FWUPD_XPORT_ABSENT:
		return "absent";
	case STS_FWUPD_XPORT_DOWN:
		return "wired, area not started";
	case STS_FWUPD_XPORT_READY:
		return "wired";
	default:
		return "?";
	}
}

/* ===================================================================== *
 *  3. what a G0 inventory read may do
 * ===================================================================== */

/** What an identity read is permitted to do with a shared serial port. */
typedef enum {
	/** Poll the device: the transport is usable and nobody else owns it. */
	STS_FWUPD_QUERY_POLL = 0,
	/** Report "present, nothing to read": a passthrough tunnel owns the port. */
	STS_FWUPD_QUERY_EMPTY_TUNNEL,
	/** Report "present, nothing to read": no usable transport. */
	STS_FWUPD_QUERY_EMPTY_NO_TRANSPORT,
} sts_fwupd_query_act_t;

/**
 * Decide whether a G0 identity read may drive the bus.
 *
 * The tunnel is tested FIRST and that ordering is the contract, not an
 * accident. "No transport" is a capability statement that can go stale in the
 * safe direction (the area starts, the probe starts saying READY); "a tunnel
 * owns this port" is a safety statement about another live session, and it must
 * win over anything that merely looks unavailable. Testing readiness first would
 * mean a future transport whose probe is optimistic could reach the wire under
 * an open tunnel.
 *
 * Both refusals produce the SAME observable result — an empty version string
 * with rc 0, which fwupd.h defines as "present, nothing to read" and the
 * maintenance tool renders differently from "absent". The distinction in this
 * enum is for the log line, not for the reply: telling an unauthenticated caller
 * that a technician currently holds a tunnel open is a disclosure, and inventing
 * a version string would be worse than admitting the gap.
 *
 * @param transport_ready  The port is usable by this thread right now.
 * @param tunnel_open      sts_mp_tunnel_gnss_open() / sts_mp_tunnel_rb_open().
 */
static inline sts_fwupd_query_act_t sts_fwupd_query_decide(bool transport_ready,
							   bool tunnel_open)
{
	if (tunnel_open) {
		return STS_FWUPD_QUERY_EMPTY_TUNNEL;
	}
	if (!transport_ready) {
		return STS_FWUPD_QUERY_EMPTY_NO_TRANSPORT;
	}
	return STS_FWUPD_QUERY_POLL;
}

/** Short name for a query verdict, for logs and test failures. Never NULL. */
static inline const char *sts_fwupd_query_act_name(sts_fwupd_query_act_t a)
{
	switch (a) {
	case STS_FWUPD_QUERY_POLL:
		return "poll";
	case STS_FWUPD_QUERY_EMPTY_TUNNEL:
		return "tunnel-owns-port";
	case STS_FWUPD_QUERY_EMPTY_NO_TRANSPORT:
		return "no-transport";
	default:
		return "?";
	}
}

/* ===================================================================== *
 *  4. the supervisor pass budget
 * ===================================================================== */

/**
 * Wall clock one supervisor pass may spend inside sts_fwupd_step().
 *
 * The orchestrator's pump takes its mutex with K_NO_WAIT, so it never spends
 * the pass's one lock wait — but that was only ever half the premise. What it
 * CAN do is run long without waiting for anything: fwupd_step() fires the step
 * and transfer-idle timeouts, and both land in finish() -> t->restore().
 *
 *   Reachable today. `fw.begin` for FWUPD_COMP_STM32_APP followed by silence:
 *   60 s later the supervisor's own step reaches stm_restore(), which is
 *   port_image_t::request_revert() — an internal-flash trailer-page erase plus
 *   a quad-word write, on the thread between two sts_liveness_feed() calls.
 *
 *   Not reachable today, and only because the allow bitmap excludes the GNSS
 *   receiver: gnss_restore() -> ubx_fwupd_recover() holds GPS_SAFEBOOT_N and
 *   GPS_RST_N for UBX_FWUPD_SAFEBOOT_SETUP_MS + UBX_FWUPD_RESET_HOLD_MS, which
 *   ubx_op_delay() serves with k_msleep(). 20 ms of sleep, in the loop that
 *   feeds the external watchdog.
 *
 * 150 ms covers both with room, and fwupd_glue.c measures the real figure and
 * complains at error level the first time a pass exceeds it — so the premise is
 * falsifiable on the bench instead of being an assumption nobody can check.
 */
#define STS_FWUPD_STEP_BUDGET_MS 150U

/**
 * Worst interval between two SUCCESSFUL sts_mp_tick() calls, in milliseconds.
 *
 * A macro rather than an inline function because sts_console.c evaluates it in
 * a BUILD_ASSERT, and a function call is not a constant expression in C11.
 *
 * The shape encodes two modelling choices, and both are load-bearing:
 *
 *   the (misses + 1) * (period + lock) term is PER PASS, because sts_mp_tick()
 *   can wait @p lock_ms for the engine and skip, and STS_MP_TICK_MISS_MAX
 *   consecutive skips are tolerated before the next success;
 *
 *   the work term is added ONCE, not per pass. fwupd's long work is finish()
 *   running restore(), which core/fwupd guarantees happens exactly once per
 *   prepared session — so at most one pass inside any dead-man window carries
 *   it. Multiplying it through would be arithmetically safer and would also
 *   make the inequality unsatisfiable at any useful budget; the honest statement
 *   is the one-shot, written down here where it can be argued with.
 */
#define STS_FWUPD_PASS_BUDGET_MS(period_ms, lock_ms, misses, work_ms)         \
	((((unsigned int)(misses) + 1U) *                                     \
	  ((unsigned int)(period_ms) + (unsigned int)(lock_ms))) +            \
	 (unsigned int)(work_ms))

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_CONSOLE_STS_FWUPD_SEAM_POLICY_H_ */
