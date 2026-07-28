/*
 * STS1000 "Meridian" — Maintenance Protocol glue (console area).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See mp_glue.h for the mode-entry design. This file is the wire, the shell
 * hand-off, and the provider callbacks.
 *
 * ---------------------------------------------------------------------------
 * As-built wiring, and what is still missing
 *
 * Engine. sts_console_start() calls sts_mp_start() alongside sts_mcp_start(),
 * and the console supervisor calls sts_mp_tick() every 250 ms — inside the 2 s
 * MP_TICK_MAX_MS dead-man budget. Without both the whole engine is not merely
 * idle: --gc-sections drops mp_init(), obj_apply() and the override/lease code
 * out of the image entirely.
 *
 * Concurrency (F11). Wiring the tick gave the engine a *second* driver, and the
 * two are on different threads at different priorities:
 *
 *   RX   shell thread, prio K_LOWEST_APPLICATION_THREAD_PRIO (19 here)
 *        mp_bypass() -> mp_input()
 *   tick console supervisor, prio 14
 *        sts_mp_tick() -> mp_tick()
 *
 * 14 preempts 19 on a single-CPU preemptible build, so before this file owned a
 * lock the tick landed *inside* mp_input() every 250 ms. Two mechanisms turned
 * that into corruption rather than a race in the abstract:
 *
 *   1. One shared TX scratch. mp_ctx_t holds a single mp_frame_tx_t, and
 *      mp_frame.h says the encoded bytes "point into tx->enc and stay valid
 *      until the next mp_frame_encode() on the same scratch". mp_tx() below
 *      walks that buffer one uart_poll_out() at a time. A tick that preempts
 *      mid-walk re-encodes into the same buffer, and the shell thread resumes
 *      emitting the *new* frame's bytes from the old offset: a spliced COBS
 *      frame, i.e. a CRC error at the tool or a decoder desync. `stream.sub` on
 *      telemetry plus any control request is enough; no timing luck is needed.
 *   2. One lease array. mp_ovr_tick() runs the dead-man revert and actuates
 *      through obj_apply() while the shell thread may be inside mp_ovr_grant()
 *      for the same object — and sts_mp_tunnel_set_gnss()'s entire balance
 *      argument (that `tunnel_gnss` is the single record of who owns USART3)
 *      assumes one writer.
 *
 * The fix is one mutex, `mp_lock`, owned here and taken across **every** entry
 * into the engine — including the whole encode-plus-transmit sequence, because a
 * lock released before mp_tx() drains fixes nothing. core/mp stays lock-free and
 * platform-neutral; see mp.h's threading block for the contract this satisfies.
 *
 * The lock is released at exactly one point, prov_auth(); the argument for why
 * that is safe is written out there.
 *
 * Panel mirror: WIRED. The ui area publishes a frame after each ui_render()
 * (src/zephyr/ui/sts_ui.c) through sts_mp_mirror_publish(), declared in
 * src/zephyr/sts_app.h because mp_glue.h is private to this area
 * (ARCHITECTURE.md §2) — the same seam sts_ui_post_input() crosses in the other
 * direction. `mirror.get` and channel 0x0A therefore serve a real panel; they
 * answer MP_E_NOTSUP only before the first render, and `mp status` prints the
 * mirror as "wired" once one has landed.
 *
 * Byte tees: WIRED, and on a ring rather than on this thread. platform/gnss.c
 * splits the USART3 receive stream onto channels 0x02/0x03 (or feeds 0x07 whole
 * while the tunnel owns the port) and platform/rb_serial.c's ISR feeds 0x08
 * through the sink mp_tunnel.c registers with rb_serial_tunnel_open(). Both
 * producers only stage; sts_mp_tick() frames what they staged, before it takes
 * this lock. The engine publishes an armed bitmask (mp_armed_refresh() below) so
 * neither producer has to ask the engine — under the lock — whether anyone is
 * listening.
 *
 * Link: WIRED, and it is the dead-man's outermost condition (FMT §5.4).
 * sts_usb.c samples CDC-ACM #0's DTR alongside the VBUS gate it already ran and
 * calls sts_mp_notify_link() on a transition; sts_console_link_policy.h owns the
 * decision and states what a cable pull used to leave behind. The notification
 * tries the lock with K_NO_WAIT and parks the transition for the tick on
 * contention — the argument is at sts_mp_notify_link() and is the supervisor's
 * pass budget, not a preference.
 *
 * Not offered, and deliberately not offered silently:
 *
 * 1. BREAK. It is undetectable on this console and the shell no longer claims
 *    otherwise. `zephyr,shell-uart` is cdc_acm_uart0 and the board has no
 *    physical console UART, so there are two places a BREAK could surface and
 *    neither does: Zephyr's cdc_acm_driver_api publishes no `.err_check`, which
 *    makes uart_err_check() answer -ENOSYS and puts UART_BREAK permanently out
 *    of reach; and cdc_acm_class_handle_req() implements only SET_LINE_CODING
 *    and SET_CONTROL_LINE_STATE, so a host's USB CDC SEND_BREAK is answered
 *    -ENOTSUP with nothing recorded and no callback. sts_mp_notify_break() is
 *    kept as the seam a physical console would use — it is correct, it is
 *    tested, and it is in scripts/reachability.allow with that evidence — but
 *    nothing on this board can call it. What changed is the *instruction*: the
 *    shell used to tell a technician BREAK would get them out.
 *
 * 2. Autobaud entry magic. sts_mp_shell_tap() implements it and nothing can
 *    feed it, on either interface:
 *
 *      ACM1 is the wrong port. MP's transport is DT_CHOSEN(zephyr_shell_uart)
 *      and every frame leaves through uart_poll_out(mp_uart, …), so a magic
 *      accepted on the MCP channel would put the device in MP mode and answer
 *      on a port the sender is not reading — while its five bytes desynchronise
 *      the COBS stream ACM1 exists to carry.
 *
 *      ACM0 is the right port and the Zephyr shell owns its bytes. The one seam
 *      the shell offers is shell_set_bypass(), which is all-or-nothing: while a
 *      bypass is installed the line editor sees nothing, and there is no API to
 *      hand a byte back. Zephyr has a byte-level diversion for exactly this
 *      shape of problem — smp_shell_rx_bytes() in shell_uart.c's
 *      uart_rx_handle() — but it is compiled in only under
 *      CONFIG_MCUMGR_TRANSPORT_SHELL and hardcoded to the SMP transport. The
 *      remaining routes (rewriting shell_transport_uart.api at runtime, or
 *      taking over the backend's UART callback and re-injecting into its RX
 *      ring) are writes into upstream internals with no compatibility promise,
 *      on the path that carries the recovery console.
 *
 *    So MP mode is entered with `mp enter` or the `sys.mode` request, docs/
 *    sts1000_field_maintenance_tool.md §2.2 says so, and mp_glue.h no longer
 *    points at a TODO here that had already been rewritten away.
 *
 * 3. The object write path. Most control objects live on GPIO/PWM/DAC that the
 *    *platform* area owns, and sts_app.h exposes only sts_panel_led_set(); the
 *    GNSS and Rb tunnels are the other two wired writes. Every remaining write
 *    answers MP_E_NOTSUP, but the manifest still publishes the object, its guard
 *    and its interlocks, so the tool discovers the surface and the safety model
 *    is already enforced. Wiring the rest is a platform-area change: one setter
 *    that takes a manifest object index, or a small table of per-object
 *    accessors in sts_app.h.
 * ---------------------------------------------------------------------------
 */

#include <zephyr/kernel.h>

#if defined(CONFIG_STS1000_CONSOLE) && defined(CONFIG_STS1000_MP)

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "auth/auth.h"
#include "console/mp_glue.h"
#include "console/sts_console.h"
#include "console/sts_mp_telem.h"
#include "console/sts_recovery_policy.h"
#include "fault/fault.h"
#include "ina228/ina228.h"
#include "zephyr/sts_app.h"

/*
 * The AAA lookup is a net-area implementation (src/zephyr/net/sts_secops.c).
 * Declared WEAK rather than reached through a cross-area header, exactly as
 * sts_mcp.c next door declares the same symbol: with CONFIG_STS1000_NET=n it
 * resolves to NULL, the maintenance plane falls back to a denial, and the image
 * still links. This replaces the console -> net include edge on net/sts_aaa.h
 * that the raw sts_aaa_check() call needed.
 */
extern int sts_aaa_check_fed(const char *user, const char *secret,
			     uint8_t *out_role, int live_id) __attribute__((weak));

LOG_MODULE_REGISTER(sts_mp, CONFIG_STS1000_LOG_LEVEL);

/* --------------------------------------------------------------- tunables */

/*
 * Tunables. Each has an #if defined() fallback so the area is correct whether or
 * not app/Kconfig sources src/zephyr/console/Kconfig.mp — the same convention
 * the rest of the console area uses (see console/Kconfig).
 */
#if defined(CONFIG_STS1000_MP_SCRATCH)
#define MP_SCRATCH_BYTES CONFIG_STS1000_MP_SCRATCH
#else
#define MP_SCRATCH_BYTES 4096
#endif

#if defined(CONFIG_STS1000_MP_FW_VERSION)
#define MP_FW_VERSION CONFIG_STS1000_MP_FW_VERSION
#else
#define MP_FW_VERSION "0.1.0-dev"
#endif

#if defined(CONFIG_STS1000_MP_REASM)
#define MP_REASM_BYTES CONFIG_STS1000_MP_REASM
#else
#define MP_REASM_BYTES 3072
#endif

/* The as-built panel is a 480x320 ST7796 with an 8x16 font: 60x20 cells. */
#define MP_MIRROR_ROWS 20
#define MP_MIRROR_COLS 60
#define MP_MIRROR_CELLS (MP_MIRROR_ROWS * MP_MIRROR_COLS)

BUILD_ASSERT(MP_SCRATCH_BYTES >= (int)MP_SCRATCH_MIN,
	     "CONFIG_STS1000_MP_SCRATCH cannot hold one mirror keyframe");

/*
 * core/mp carries its own MP_ROLE_* constants because its dependency set does
 * not include core/auth (ARCHITECTURE.md §4). This file is the seam where both
 * are visible, so it is where the numeric identity is proved. If auth_role_t is
 * ever reordered, the build stops here instead of silently promoting a viewer.
 */
BUILD_ASSERT((int)AUTH_ROLE_NONE == (int)MP_ROLE_NONE, "role enum drift");
BUILD_ASSERT((int)AUTH_ROLE_VIEWER == (int)MP_ROLE_VIEWER, "role enum drift");
BUILD_ASSERT((int)AUTH_ROLE_OPERATOR == (int)MP_ROLE_OPERATOR, "role enum drift");
BUILD_ASSERT((int)AUTH_ROLE_ADMIN == (int)MP_ROLE_ADMIN, "role enum drift");
BUILD_ASSERT(MP_USER_MAX >= AUTH_USER_MAX,
	     "a session cannot record the longest user AAA accepts");
BUILD_ASSERT(MP_SECRET_MAX >= AUTH_SECRET_MAX,
	     "the control plane would truncate a credential AAA would accept");

/* ------------------------------------------------------------------ state */

static const struct device *const mp_uart =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));

static mp_ctx_t mp;
static bool mp_started;

static uint8_t mp_scratch[MP_SCRATCH_BYTES];
static uint8_t mp_reasm_buf[2][MP_REASM_BYTES / 2];
static mp_reasm_t mp_reasm[2];

static char mirror_prev_ch[MP_MIRROR_CELLS];
static uint8_t mirror_prev_attr[MP_MIRROR_CELLS];

/* The published panel frame, double-buffered under a short mutex. */
static K_MUTEX_DEFINE(mirror_lock);
static mp_mirror_in_t mirror_frame;
static char mirror_ch[MP_MIRROR_CELLS];
static uint8_t mirror_attr[MP_MIRROR_CELLS];
static ui_hint_t mirror_hints[UI_SURF_MAX_HINTS];
static bool mirror_valid;

/*
 * The engine's private copy of the published frame.
 *
 * prov_mirror() cannot hold mirror_lock across the encode: it is a provider
 * callback, core/mp encodes *after* it returns, and the worker vtable has no
 * release hook to unlock on the way back out. Handing core pointers into
 * mirror_ch/mirror_attr therefore let mp_mirror_encode() read 1200 cells with
 * the lock down, so a keyframe issued exactly as the ui thread published could
 * carry the top half of frame N and the bottom half of N+1. Self-correcting on
 * the next frame, but a torn keyframe is precisely the frame a host trusts.
 *
 * So the provider snapshots under the lock and hands core the snapshot. One
 * buffer is enough because every entry into core/mp is serialised by mp_lock
 * (F11) and both callers — m_mirror_get() and pump_mirror() — are inside it.
 */
static char mirror_snap_ch[MP_MIRROR_CELLS];
static uint8_t mirror_snap_attr[MP_MIRROR_CELLS];
static ui_hint_t mirror_snap_hints[UI_SURF_MAX_HINTS];

/*
 * Frames sts_mp_mirror_publish() did not accept, cumulative.
 *
 * `mp status` used to report mirror_valid plus the *encoder's* frame counts,
 * and neither of those moves when a publish is refused — so "the mirror is
 * healthy" and "every frame since boot was dropped" printed identically.
 *
 * `mirror_drops` is the designed case: the 5 ms lock timeout expired because
 * the MP tick held it, which is normal at 10 Hz and self-healing.
 * `mirror_rejects` must stay zero — it counts a frame refused for its geometry
 * or a NULL surface, which for the ui area is a build error caught by the
 * BUILD_ASSERT in sts_ui.c, and for any future publisher is a bug that would
 * otherwise be perfectly silent.
 *
 * Atomic because sts_app.h advertises sts_mp_mirror_publish() as callable from
 * any cooperative thread, and both increments sit outside mirror_lock by
 * necessity — the contended path is the one that could not take it.
 */
static atomic_t mirror_drops;
static atomic_t mirror_rejects;

static char mp_serial[MP_CONFIRM_MAX];
static const struct shell *mp_shell;

/* --------------------------------------------------------- the engine lock */

/*
 * Serialises every entry into core/mp (F11; rationale in the file header).
 *
 * Held across encode *and* transmit, so a frame on the wire is never spliced
 * with another. Zephyr's k_mutex carries priority inheritance, so the console
 * supervisor waiting here lifts the shell thread to prio 14 for the duration
 * instead of leaving it at 19 behind other work.
 */
static K_MUTEX_DEFINE(mp_lock);

/*
 * Set only while prov_auth() has released mp_lock across a blocking credential
 * check. It is what lets the two mode-leaving entry points below tell "the
 * engine is idle" from "the shell thread is suspended with an inbound frame
 * half-decoded", which they must not disturb.
 */
static bool mp_auth_window;
/** A BREAK / DTR drop that arrived during that window, honoured by mp_bypass(). */
static bool mp_exit_pending;

/*
 * A link transition sts_mp_notify_link() could not apply where it was reported,
 * because the engine lock was held. 0 = nothing pending, 1 = up, 2 = down.
 *
 * The caller is sts_usb_poll(), on the console supervisor — the same thread and
 * the same pass as sts_mp_tick(), whose lock wait is the *one* the BUILD_ASSERT
 * in sts_console.c budgets. So the notification tries K_NO_WAIT and parks the
 * transition here rather than adding a second timed wait, exactly as
 * sts_mp_stream_raw() does and for the same arithmetic. The next successful tick
 * applies it, which puts the revert inside the deadline that assertion proves
 * (1950 ms, against MP_TICK_MAX_MS = 2000) instead of outside it.
 *
 * Atomic because it is written by whichever thread notices the port and read by
 * both engine drivers; atomic_set() returns the previous value, so the read is a
 * swap and a transition can never be applied twice or lost to a racing write.
 */
#define MP_LINK_PEND_NONE 0
#define MP_LINK_PEND_UP   1
#define MP_LINK_PEND_DOWN 2
static atomic_t mp_link_pending = ATOMIC_INIT(MP_LINK_PEND_NONE);

/* Tick-skip accounting; reported by `mp status`. */
static uint32_t mp_tick_misses;
static uint8_t mp_tick_miss_run;
static uint8_t mp_tick_miss_worst;
/*
 * Link transitions that had to be deferred to a tick; `mp status`.
 *
 * Atomic, unlike the tick-miss counters beside it, because this one is
 * incremented on the path that *failed* to take the engine lock — so it is the
 * one counter here with no mutual exclusion available to it.
 */
static atomic_t mp_link_defers;

/*
 * Firmware-veto staging (mp_glue.h sts_mp_veto()).
 *
 * One bit per sts_mp_veto_t subject, OR-ed in by the platform area and swapped
 * to zero by the drain under the engine lock. A bitmap rather than a queue
 * because a veto is a COMMAND and not a record: two requests for the same
 * subject must coalesce (mp_ovr_veto() is idempotent), and nothing may ever be
 * dropped — a lost veto leaves an override standing against a rail that is
 * already down, which is the whole defect.
 *
 * The counters are separate from mp.ovr.vetoes, which counts leases actually
 * withdrawn by anyone (including a grant whose apply refused). These two count
 * the seam: what platform asked for, and how many of those requests the drain
 * has consumed. `raised` is incremented on the producer's thread, so it is
 * atomic; `applied` is written only by the drain, under the lock.
 */
static atomic_t mp_veto_pending;
static atomic_t mp_veto_raised;
static uint32_t mp_veto_applied;

/** Enter the engine from a thread that may wait. Never call from an ISR. */
static void mp_engine_lock(void)
{
	(void)k_mutex_lock(&mp_lock, K_FOREVER);
}

static void mp_engine_unlock(void)
{
	(void)k_mutex_unlock(&mp_lock);
}

/* ---------------------------------------------------------- armed channels */

/*
 * Which passthrough channels currently have a consumer, as a bitmask of MP_CH_*
 * indices — the lock-free projection of engine state that the byte tees test.
 *
 * The producers are the priority-6 GNSS thread and the UART7 receive ISR. Asking
 * the engine directly would mean a mutex on both, which is the priority
 * inversion this area exists to prevent (and is illegal outright in the ISR), so
 * the engine publishes the answer instead: mp_armed_refresh() is called from
 * every locked section that can change a subscription or leave MP mode. A
 * producer then pays one atomic read to find out that nobody is watching.
 *
 * Staleness is bounded by the tick period, and both directions are harmless: a
 * stale "armed" stages one chunk the drain discards with -ENOENT, a stale "not
 * armed" costs one tick of tee data at subscribe time.
 */
static atomic_t mp_ch_armed;

BUILD_ASSERT(MP_CH_COUNT <= 32, "the armed mask is one 32-bit atomic word");

/** Recompute the armed mask. **Call with mp_lock held.** */
static void mp_armed_refresh(void)
{
	atomic_val_t m = 0;

	if (mp_mode(&mp) == (uint8_t)MP_MODE_MP) {
		static const uint8_t ch[] = {
			(uint8_t)MP_CH_NMEA,
			(uint8_t)MP_CH_UBX,
			(uint8_t)MP_CH_GNSS_PASS,
			(uint8_t)MP_CH_RB_PASS,
			/*
			 * Not a tee, but the same problem: the event channel's
			 * producers are the 1 kHz scan and every caller of
			 * sts_alarm_set(), none of which may ask the engine —
			 * under the lock — whether a technician is watching.
			 * sts_mp_gnss_tee_armed() below tests a fixed mask, so
			 * adding a channel here does not widen it.
			 */
			(uint8_t)MP_CH_EVENT,
		};
		size_t i;

		for (i = 0U; i < ARRAY_SIZE(ch); i++) {
			if (mp_stream_is_sub(&mp.st, ch[i])) {
				m |= (atomic_val_t)BIT(ch[i]);
			}
		}
	}

	atomic_set(&mp_ch_armed, m);
}

bool sts_mp_ch_armed(uint8_t ch)
{
	if (ch >= 32U) {
		return false;
	}
	return (atomic_get(&mp_ch_armed) & (atomic_val_t)BIT(ch)) != 0;
}

bool sts_mp_gnss_tee_armed(void)
{
	static const atomic_val_t gnss_mask =
		(atomic_val_t)(BIT(MP_CH_NMEA) | BIT(MP_CH_UBX) |
			       BIT(MP_CH_GNSS_PASS));

	return (atomic_get(&mp_ch_armed) & gnss_mask) != 0;
}

/* ------------------------------------------------------------------ ports */

static uint32_t mono_ms(void *user)
{
	ARG_UNUSED(user);
	return (uint32_t)k_uptime_get_32();
}

/**
 * Transmit one framed message.
 *
 * Polled output: MP owns the port exclusively while it is active (the shell is
 * bypassed), the frames are small, and a poll loop cannot desynchronise the COBS
 * stream the way a partially-accepted ring write could.
 *
 * `wire` points into the engine's single mp_frame_tx_t scratch and is only valid
 * until the next mp_frame_encode(), so this loop **must** run inside mp_lock —
 * which it does, because every path that reaches mp_send() is entered under it.
 * A lock released before this drain would leave the splice described in the file
 * header exactly as it was.
 */
static int mp_tx(void *user, const uint8_t *wire, size_t len)
{
	size_t i;

	ARG_UNUSED(user);
	if (!device_is_ready(mp_uart)) {
		return -ENODEV;
	}
	for (i = 0U; i < len; i++) {
		uart_poll_out(mp_uart, wire[i]);
	}
	return 0;
}

/* ------------------------------------------------------------- providers */

/** Map a health snapshot's INA array onto the record's rail array. */
static void fill_rails(mp_health_t *h, const sts_health_t *s)
{
	size_t i;

	for (i = 0U; i < MP_RAIL_COUNT; i++) {
		if (i >= s->ina_count) {
			break;
		}
		h->rail[i].bus_mv = s->ina[i].bus_mv;
		h->rail[i].current_ua = s->ina[i].current_ua;
		h->rail[i].power_uw = s->ina[i].power_uw / 1000U;
		h->rail[i].diag = s->ina[i].diag_alrt;
		h->rail[i].valid = s->ina[i].valid;
	}
}

static void fill_health(mp_health_t *h, const sts_health_t *s)
{
	memset(h, 0, sizeof(*h));
	fill_rails(h, s);
	h->tmp_osc_mc = s->tmp_osc_mc;
	h->tmp_osc_valid = s->tmp_osc_valid;
	h->tmp_amb_mc = s->tmp_amb_mc;
	h->tmp_amb_valid = s->tmp_amb_valid;
	h->die_mc = s->die_mc;
	h->die_valid = s->die_valid;
	h->humidity_mpct = s->humidity_mpct;
	h->humidity_valid = s->humidity_valid;
	h->fan_rpm = s->fan_rpm;
	h->fan_duty_pct = s->fan_duty_pct;
	h->poe_class = s->poe_class;
	h->poe_draw_mw = s->poe_draw_mw;
	h->poe_budget_mw = s->poe_budget_mw;
	h->bkp_stm_pg = s->bkp_stm_pg;
	h->bkp_gps_pg = s->bkp_gps_pg;
}

/**
 * Assemble one telemetry record (FMT §7.1).
 *
 * Console-thread context, inside mp_lock. Every read here is a snapshot
 * accessor and none of them is a timing lock:
 *
 *   sts_quality_snapshot()  lock-free seqlock (core/quality)
 *   sts_pwrseq_snapshot()   lock-free seqlock (platform/sts_pwrseq_pub.h)
 *   sts_health_snapshot()   bounded mutex; the housekeeping thread holds it for
 *                           one struct copy per sweep
 *   sts_gnss_detail()       bounded mutex; the gnss thread holds it for three
 *                           struct copies per second
 *
 * so the whole provider is bounded by a handful of memcpys and cannot park this
 * thread behind a producer, nor a producer behind it — which is the rule that
 * put snapshots in this architecture in the first place (ARCHITECTURE.md §10).
 *
 * Note what is NOT in that list: sts_fault_lock(). `scan_state` (key 45) is
 * core/fault's debounced bitmap, and taking the fault mutex from the console
 * thread to read it would put this provider on the io_scan thread's path for a
 * word the housekeeping thread already reads every tick. It arrives inside
 * sts_pwrseq_snapshot() instead — see sts_mp_telem.h.
 *
 * How each key is filled, and from which half of which snapshot, is stated in
 * sts_mp_telem.h rather than here, so the reasoning sits beside the binding it
 * qualifies.
 */
static int prov_telem(void *user, mp_telem_t *out)
{
	sts_health_t hs;
	sts_pwrseq_snap_t ps;
	sts_gnss_detail_t gd;
	bool have_ps;
	bool have_gd;

	ARG_UNUSED(user);
	memset(out, 0, sizeof(*out));

	if (sts_quality_snapshot(&out->q) != 0) {
		quality_block_init(&out->q);
	}
	if (sts_health_snapshot(&hs) == 0) {
		fill_health(&out->h, &hs);
	}
	out->alarms = sts_alarms_active();
	out->alarms_latched = sts_alarms_latched();
	out->uptime_s = (uint32_t)(k_uptime_get() / 1000);
	(void)sts_time_tai_ns(&out->tai_ns);
	out->time_fallback = sts_time_is_fallback();

	have_ps = (sts_pwrseq_snapshot(&ps) == 0);
	have_gd = (sts_gnss_detail(&gd) == 0);

	sts_mp_telem_bind_pwrseq(out, have_ps ? &ps : NULL, out->q.active_ref);
	sts_mp_telem_bind_gnss(out, have_gd ? &gd : NULL);
	return 0;
}

static int prov_pps(void *user, mp_pps_t *out)
{
	quality_block_t q;

	ARG_UNUSED(user);
	memset(out, 0, sizeof(*out));
	if (sts_quality_snapshot(&q) != 0) {
		return -EIO;
	}

	/*
	 * What the quality block carries. The per-capture raw phases and the
	 * pre-correction residual live in `disc`, which publishes only the
	 * corrected result; pa0_ns/pc6_ns/qerr therefore stay zero and
	 * pc6_valid false until the timing glue offers a capture accessor.
	 */
	out->residual_corr_ns = q.last_pps_off_ns;
	out->vc_cmd_mv = q.vc_cmd_mv;
	out->vc_sense_mv = q.vc_sense_mv;
	out->dac_code = q.dac_code;
	out->loop_state = q.lock_state;
	out->active_ref = q.active_ref;
	out->ref_flags = q.flags;
	out->accepted = ((q.flags & (QUALITY_FLAG_PPS_REJECT |
				     QUALITY_FLAG_NO_PPS)) == 0U);
	return 0;
}

/**
 * Interlock state.
 *
 * Fail-safe by construction: anything this area cannot observe is left in the
 * state that makes the interlock *refuse*. A missing accessor must never read as
 * "permission granted".
 */
static int prov_ilk(void *user, mp_ilk_state_t *out)
{
	sts_health_t hs;
	quality_block_t q;
	uint64_t alarms = sts_alarms_active();
	uint64_t vmax = 15000U;

	ARG_UNUSED(user);
	memset(out, 0, sizeof(*out));

	if (sts_quality_snapshot(&q) == 0) {
		out->ocxo_warm = ((q.flags & QUALITY_FLAG_OCXO_WARM) != 0U);
		out->disc_parked = ((q.flags & QUALITY_FLAG_PARKED) != 0U);
		out->rb_lock = (q.active_ref == (uint8_t)QUALITY_REF_RB);
		out->extref_ok = (q.active_ref == (uint8_t)QUALITY_REF_EXTREF);
	}

	if (sts_health_snapshot(&hs) == 0) {
		/* Both supercap rails good is the §5.5 precondition for the
		 * rubidium chain, alongside a warm oven. */
		out->supercaps_ok = hs.bkp_stm_pg && hs.bkp_gps_pg;
		if (hs.ina_count > (size_t)INA228_RAIL_VCC_RB) {
			out->vcc_rb_mv = hs.ina[INA228_RAIL_VCC_RB].bus_mv;
			out->vcc_rb_valid = hs.ina[INA228_RAIL_VCC_RB].valid;
		}
		out->fan_floor_pct = (uint8_t)MIN(hs.fan_duty_pct, 100U);
	}

	/* The configured ceiling, from cfg — this is what bounds every VCC_RB
	 * request (the manifest publishes the wider electrical range). */
	if ((sts_cfg() != NULL) &&
	    (cfg_get_u64(sts_cfg(), 0x0703U, &vmax) == 0)) {
		out->rb_vmax_mv = (uint16_t)vmax;
	} else {
		out->rb_vmax_mv = 0U; /* unknown -> rb.vmax refuses */
	}
	out->rb_expected_mv = (int32_t)out->rb_vmax_mv;

	/*
	 * The digipot code window would come from pwrseq's own VCC_RB transfer
	 * function. Without an accessor the window is left inverted
	 * (min > max), which mp_ilk_eval() treats as "cannot compute" and
	 * refuses — the correct answer for a raw wiper write.
	 */
	out->rb_code_min = 1U;
	out->rb_code_max = 0U;

	out->rb_ov_latched = (alarms & FAULT_ALARM_BIT(FAULT_ALARM_RB_OV)) != 0U;
	out->relay_blocked = (alarms & FAULT_RELAY_DISQUALIFY_DEFAULT) != 0U;

	/* DISP_EN and the supervisor's liveness gate are platform state. Leaving
	 * disp_on false with a fresh change stamp makes the off-time interlock
	 * hold, and liveness_ok false makes the WDT interlock refuse. */
	out->disp_on = false;
	out->disp_changed_ms = (uint32_t)k_uptime_get_32();
	out->liveness_ok = false;
	return 0;
}

static int prov_mirror(void *user, mp_mirror_in_t *out)
{
	size_t cells;
	uint8_t hints;

	ARG_UNUSED(user);

	/*
	 * A timeout means the ui thread is mid-memcpy into mirror_ch/mirror_attr,
	 * so copying anyway would serve cells from two different renders — and
	 * unlocking a mutex this thread does not own returns -EPERM, i.e. the
	 * release would silently not happen either. Bail, like
	 * sts_mp_mirror_publish() does on the other side.
	 *
	 * -EBUSY specifically: mp_map_errno() turns it into MP_E_BUSY ("another
	 * operation holds the resource"), which is the retryable answer the tool
	 * needs. -EAGAIN has no case in that map and would land on
	 * MP_E_INTERNAL; -ENOTSUP would claim the mirror is unwired for good.
	 * pump_mirror() treats any non-zero as "skip this frame", so a subscribed
	 * stream simply resyncs on the next tick.
	 */
	if (k_mutex_lock(&mirror_lock, K_MSEC(20)) != 0) {
		return -EBUSY;
	}
	/*
	 * mirror_valid is tested *inside* the lock, not before it: the publisher
	 * sets it at the end of its memcpy sequence, so an unlocked read is a read
	 * of the one word that says whether the other words are complete. It is
	 * benign today (a single aligned bool, false->true exactly once) and would
	 * stop being benign the moment the mirror is ever invalidated.
	 */
	if (!mirror_valid) {
		(void)k_mutex_unlock(&mirror_lock);
		return -ENOTSUP;
	}

	/*
	 * Snapshot the cells here, not just the descriptor: core/mp reads them
	 * after this returns, with the lock down (see mirror_snap_ch above).
	 * sts_mp_mirror_publish() has already bounded both counts, so the clamps
	 * are belt-and-braces against a future publisher rather than live checks.
	 */
	*out = mirror_frame;
	cells = (size_t)mirror_frame.rows * (size_t)mirror_frame.cols;
	if (cells > MP_MIRROR_CELLS) {
		cells = MP_MIRROR_CELLS;
	}
	hints = mirror_frame.hint_count;
	if (hints > UI_SURF_MAX_HINTS) {
		hints = (uint8_t)UI_SURF_MAX_HINTS;
	}
	memcpy(mirror_snap_ch, mirror_ch, cells);
	memcpy(mirror_snap_attr, mirror_attr, cells);
	memcpy(mirror_snap_hints, mirror_hints,
	       (size_t)hints * sizeof(ui_hint_t));
	(void)k_mutex_unlock(&mirror_lock);

	out->ch = mirror_snap_ch;
	out->attr = mirror_snap_attr;
	out->hint = mirror_snap_hints;
	out->hint_count = hints;
	return 0;
}

static int prov_bundle(void *user, mp_bundle_t *out)
{
	static uint8_t i2c_map[16];

	ARG_UNUSED(user);
	memset(out, 0, sizeof(*out));

	/*
	 * Both words, not just the live one. `fault_active` answers "what is
	 * wrong now"; `fault_latched` (key 11) is the sticky record of what has
	 * gone wrong since boot, and it is the half a support bundle exists for —
	 * a transient that has already cleared appears in the latched word and
	 * nowhere else. Filling only `fault_active` after the memset left key 11
	 * shipping a constant zero, which reads as "nothing has ever faulted"
	 * rather than as "not collected".
	 */
	out->fault_active = sts_alarms_active();
	out->fault_latched = sts_alarms_latched();
	if (sts_diag_i2c_scan(i2c_map) >= 0) {
		out->i2c_scan = i2c_map;
	}
	out->notes = "mp diag.snapshot";
	return 0;
}

static int prov_time(void *user, uint64_t *tai_ns, bool *fallback)
{
	ARG_UNUSED(user);
	*fallback = sts_time_is_fallback();
	return sts_time_tai_ns(tai_ns);
}

static int prov_cfg_commit(void *user, cfg_commit_res_t *res)
{
	ARG_UNUSED(user);
	/* sts_cfg_commit(), not cfg_commit(): the group appliers must run. */
	return sts_cfg_commit(res);
}

/**
 * The host->device half of passthrough channels 0x07 and 0x08.
 *
 * An adapter only, for the `void *user` core passes and this side does not
 * need: mp_tunnel.c holds the channel-to-UART map and the platform's own
 * refusals, and duplicating either here would give the device two answers.
 *
 * Core has already established that the channel names a passthrough holding a
 * live override lease before it calls (mp_rpc.c:3307), and sts_mp_tunnel_write()
 * checks the same thing again from the platform's side — deliberately, because a
 * sink that trusts its caller to have checked is one refactor away from writing
 * into a port firmware still owns.
 */
static int prov_raw_tx(void *user, uint8_t ch, const uint8_t *data, size_t len)
{
	ARG_UNUSED(user);
	return sts_mp_tunnel_write(ch, data, len);
}

/**
 * The maintenance credential check (FMT §5.3).
 *
 * One line of substance, and that is the point: the throttle, the hard lockout
 * that reconnecting does not reset, the positive cache and the role mapping all
 * live in sts_aaa_check(), shared with the web and shell planes. Every non-zero
 * return is a denial — including -EHOSTUNREACH, which means no authority could
 * answer and is never an allow-on-failure (sts_aaa.h).
 *
 * ---------------------------------------------------------------------------
 * Why sts_aaa_check_fed() and not sts_aaa_check()
 * ---------------------------------------------------------------------------
 *
 * NOT for the watchdog. This runs on the shell thread, which is not a liveness
 * participant, so `live_id` is -1 and no feeding happens — the wrapper costs
 * this plane nothing on that axis.
 *
 * It is for the gate. sts_secops.h states the invariant as "one lookup at a
 * time, board-wide", and a raw sts_aaa_check() here made that false: an MP
 * login and a web login could be inside sts_aaa.c's shared static g_tx/g_rx
 * buffers at the same time, serialised only by that file's own mutex, while
 * every sts_aaa_check_fed() caller queued behind whichever of them got there
 * first. Going through the wrapper restores the invariant and additionally
 * bounds this call at STS_AAA_FED_CEILING_MS, so a blackholed backend cannot
 * park the maintenance plane — or the gate every other plane needs — forever.
 *
 * Only -EBUSY is passed through with its identity intact, because core reports a
 * lockout distinctly; every other refusal reaches the host as one
 * indistinguishable answer, so this is not an account oracle.
 *
 * ---------------------------------------------------------------------------
 * Why the engine lock is dropped here, and only here
 * ---------------------------------------------------------------------------
 *
 * sts_aaa.h states its own cost: sts_aaa_check() "blocks (DNS, a UDP round trip,
 * or a TCP+TLS handshake) for up to the configured per-backend timeout". Against
 * an unreachable chain that is not milliseconds. From the cfg schema
 * (cfg_schema.h, group 0x0A), per login attempt:
 *
 *   RADIUS   sec.radius.tmo.ms  <= 30 000, x (sec.radius.retries <= 5) + 1
 *                                             ->  180 000 ms
 *   TACACS+  sec.tacacs.tmo.ms  <= 30 000, >= 4 blocking recv()s per login
 *                                             ->  120 000 ms
 *   LDAP     sec.ldap.tmo.ms    <= 30 000, >= 8 blocking recv()s (bind, service
 *                                bind, three membership searches)
 *                                             ->  240 000 ms
 *
 * i.e. a worst-case configured chain of local->radius->tacacs->ldap is about
 * **540 s**, and 69 s at the shipped defaults (3000x3 + 5000x4 + 5000x8). Those
 * are floors, not ceilings: read_exact()/ldap_recv() loop per-recv, so a server
 * that dribbles one octet just inside each timeout extends them without bound.
 *
 * Holding mp_lock across that would suspend the dead-man for the same duration —
 * the tick-miss budget in mp_glue.h tolerates 1800 ms, so *any* remote backend
 * blows it — and would leave a granted override commanding hardware minutes past
 * its keepalive. Bounding the wait instead of releasing it does not help: no
 * bound both fits 1800 ms and lets a real RADIUS server answer.
 *
 * So the lock is released across exactly this call, and the release is safe
 * because the engine is *between* operations at this point. m_session_open()
 * authenticates before mp_ovr_session_open() (deliberately, so a bad credential
 * cannot kick a working tool off the board), so the only mp_ctx_t state alive on
 * this thread's stack is:
 *
 *   c->rx           the inbound frame being decoded
 *   c->tok, c->reply, c->err_*   the parsed request and half-built response
 *
 * and nothing else reaches any of them: mp_tick(), mp_stream_raw() and the
 * status readers touch c->ovr, c->st, c->diag, c->mirror, c->txf and w.scratch
 * only. c->txf in particular is *not* live — mp_rpc_handle() has not transmitted
 * anything yet; on_msg() sends the reply after it returns — so the TX-scratch
 * splice this lock exists to prevent cannot occur through this window.
 *
 * The one exception is mp_mode_exit(), which resets c->rx. It is reachable from
 * another thread only through sts_mp_notify_break()/sts_mp_notify_link(false),
 * and both defer it while mp_auth_window is set; mp_bypass() honours the
 * deferral once mp_input() has returned. Their safety half (reverting every
 * lease) still runs immediately.
 *
 * A tick that lands in the window and finds the *previous* session stale will
 * revert its overrides and zero it. That is correct, not a race:
 * mp_ovr_session_open() then opens a fresh session from a cleared struct.
 */
static int prov_auth(void *user, const char *user_name, const char *secret,
		     uint8_t *out_role)
{
	bool released;
	int rc;

	ARG_UNUSED(user);

	*out_role = (uint8_t)AUTH_ROLE_NONE;
	if ((user_name == NULL) || (secret == NULL)) {
		return -EACCES;
	}
	if (sts_aaa_check_fed == NULL) {
		/* No net area: no authority could answer, which is a denial.
		 * Checked before the lock dance below so the window is never
		 * opened for a call that cannot happen. */
		return -EACCES;
	}

	/* Raised before the release and cleared after the re-acquisition, so it
	 * is observable to another thread exactly while the window is open. */
	mp_auth_window = true;
	released = (k_mutex_unlock(&mp_lock) == 0);
	if (!released) {
		/*
		 * Not this thread's to release, i.e. prov_auth() was somehow
		 * reached without the engine lock. Re-taking it below would then
		 * deadlock the console, so leave the lock state alone and just do
		 * the check. Unreachable through mp_bypass(); handled rather than
		 * asserted because an assert is compiled out of a release image.
		 */
		mp_auth_window = false;
	}

	/* live_id -1: the shell thread feeds no liveness participant, so there
	 * is nothing to keep alive across the wait. The wrapper is here for the
	 * board-wide gate and the budget, not for the watchdog. */
	rc = sts_aaa_check_fed(user_name, secret, out_role, -1);

	if (released) {
		mp_engine_lock();
		mp_auth_window = false;
	}

	if (rc != 0) {
		*out_role = (uint8_t)AUTH_ROLE_NONE;
		return (rc == -EBUSY) ? -EBUSY : -EACCES;
	}
	return 0;
}

/* ------------------------------------------------------- object accessors */

static int obj_read(void *user, size_t obj, mp_val_t *out)
{
	const mp_obj_t *o = mp_obj_at(obj);
	sts_health_t hs;
	quality_block_t q;

	ARG_UNUSED(user);
	if (o == NULL) {
		return -EINVAL;
	}
	out->kind = o->kind;
	out->valid = false;

	/* Rails come straight from the housekeeping sweep. */
	if (o->kind == (uint8_t)MP_KIND_RAIL) {
		size_t rail = (size_t)o->min;

		if (sts_health_snapshot(&hs) != 0) {
			return -EIO;
		}
		if (rail >= hs.ina_count) {
			return -EINVAL;
		}
		out->rail[0] = hs.ina[rail].bus_mv;
		out->rail[1] = hs.ina[rail].current_ua;
		out->rail[2] = (int32_t)(hs.ina[rail].power_uw / 1000U);
		out->rail[3] = (int32_t)hs.ina[rail].diag_alrt;
		out->valid = hs.ina[rail].valid;
		return 0;
	}

	if (strcmp(o->id, "sensor.alarms") == 0) {
		out->u = sts_alarms_active();
		out->valid = true;
		return 0;
	}
	if (strcmp(o->id, "sensor.uptime_s") == 0) {
		out->i = (int32_t)(k_uptime_get() / 1000);
		out->valid = true;
		return 0;
	}
	if (strcmp(o->id, "sensor.serial") == 0) {
		(void)strncpy(out->text, mp_serial, sizeof(out->text) - 1U);
		out->text[sizeof(out->text) - 1U] = '\0';
		out->valid = true;
		return 0;
	}
	if (strcmp(o->id, "sensor.fw_version") == 0) {
		(void)strncpy(out->text, MP_FW_VERSION,
			      sizeof(out->text) - 1U);
		out->text[sizeof(out->text) - 1U] = '\0';
		out->valid = true;
		return 0;
	}
	if (strcmp(o->id, "ui.panel.duty") == 0) {
		out->i = (int32_t)sts_panel_led_get();
		out->valid = true;
		return 0;
	}

	if (sts_health_snapshot(&hs) == 0) {
		if (strcmp(o->id, "sensor.temp.osc") == 0) {
			out->i = hs.tmp_osc_mc;
			out->valid = hs.tmp_osc_valid;
			return 0;
		}
		if (strcmp(o->id, "sensor.temp.amb") == 0) {
			out->i = hs.tmp_amb_mc;
			out->valid = hs.tmp_amb_valid;
			return 0;
		}
		if (strcmp(o->id, "sensor.temp.die") == 0) {
			out->i = hs.die_mc;
			out->valid = hs.die_valid;
			return 0;
		}
		if (strcmp(o->id, "sensor.humidity") == 0) {
			out->i = hs.humidity_mpct;
			out->valid = hs.humidity_valid;
			return 0;
		}
		if (strcmp(o->id, "sensor.fan.rpm") == 0) {
			out->i = (int32_t)hs.fan_rpm;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.fan.duty") == 0) {
			out->i = (int32_t)hs.fan_duty_pct;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.poe.class") == 0) {
			out->i = (int32_t)hs.poe_class;
			out->valid = (hs.poe_class != 0U);
			return 0;
		}
		if (strcmp(o->id, "sensor.poe.draw_mw") == 0) {
			out->i = (int32_t)hs.poe_draw_mw;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.bkp.pg") == 0) {
			out->u = (hs.bkp_stm_pg ? 1U : 0U) |
				 (hs.bkp_gps_pg ? 2U : 0U);
			out->valid = true;
			return 0;
		}
	}

	if (sts_quality_snapshot(&q) == 0) {
		if (strcmp(o->id, "sensor.timing.stratum") == 0) {
			out->i = (int32_t)q.stratum;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.lock") == 0) {
			out->i = (int32_t)q.lock_state;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.ref") == 0) {
			out->i = (int32_t)q.active_ref;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.pps_ns") == 0) {
			out->i = q.last_pps_off_ns;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.freq_ppb") == 0) {
			out->f = q.freq_err_ppb;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.adev_1s") == 0) {
			out->f = q.adev_1s;
			out->valid = (q.adev_1s != 0.0f);
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.adev_10s") == 0) {
			out->f = q.adev_10s;
			out->valid = (q.adev_10s != 0.0f);
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.adev_100s") == 0) {
			out->f = q.adev_100s;
			out->valid = (q.adev_100s != 0.0f);
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.holdover_s") == 0) {
			out->i = (int32_t)q.holdover_elapsed_s;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.timing.root_disp_ns") == 0) {
			out->i = (int32_t)quality_ns_from_ntp_short(
				q.root_disp_q16);
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.gnss.fix") == 0) {
			out->i = (int32_t)q.gnss_fix;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.gnss.sv_used") == 0) {
			out->i = (int32_t)q.gnss_sv_used;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.gnss.sv_visible") == 0) {
			out->i = (int32_t)q.gnss_sv_visible;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.gnss.tacc_ns") == 0) {
			out->i = (int32_t)q.gnss_tacc_ns;
			out->valid = true;
			return 0;
		}
		if (strcmp(o->id, "sensor.ocxo.vc") == 0) {
			out->i = q.vc_sense_mv;
			out->valid = ((q.flags &
				       QUALITY_FLAG_VC_SENSE_VALID) != 0U);
			return 0;
		}
	}

	/* Everything else needs a platform accessor that does not exist yet. */
	return -ENOTSUP;
}

static int obj_apply(void *user, size_t obj, const int32_t *value)
{
	const mp_obj_t *o = mp_obj_at(obj);

	ARG_UNUSED(user);
	if (o == NULL) {
		return -EINVAL;
	}

	/* The panel dimmer is the one output sts_app.h exposes today. */
	if (strcmp(o->id, "ui.panel.duty") == 0) {
		uint8_t duty = (value != NULL)
				       ? (uint8_t)CLAMP(*value, 0, 100)
				       : 0U;

		/* A release restores the UI's own brightness policy; with no
		 * accessor for it, 0 %% is the safe resting state. */
		return sts_panel_led_set(duty);
	}

	if (strcmp(o->id, "gnss.tunnel") == 0) {
		return sts_mp_tunnel_set_gnss((value != NULL) && (*value != 0));
	}
	if (strcmp(o->id, "ref.rb.tunnel") == 0) {
		return sts_mp_tunnel_set_rb((value != NULL) && (*value != 0));
	}

	/*
	 * The locate beacon. A release (value == NULL) and an explicit 0 both
	 * stop it, so the LED cannot outlive the lease that asked for it — which
	 * is what makes MP_MIRROR_F_IDENTIFY, the flag the mirror already
	 * reports, mean something.
	 */
	if (sts_recov_bool_action(o->id) == STS_RECOV_IDENTIFY) {
		bool on = (value != NULL) && (*value != 0);

		sts_supervisor_identify(on ? STS_RECOV_IDENTIFY_MS : 0U);
		return 0;
	}

	/*
	 * A release of an object we never applied must succeed, or the dead-man
	 * would count a release error on every revert.
	 */
	if (value == NULL) {
		return 0;
	}
	return -ENOTSUP;
}

/*
 * The two pulsable recovery objects. Both are platform-owned pins, so both are
 * REQUESTS posted to the housekeeping sequencer rather than writes from this
 * thread (sts_app.h, "operator recovery actions").
 *
 * The pulse WIDTH the manifest carries is not passed on, and that is deliberate
 * for both. RB_OV_RESET's width is fixed by the OV-latch datasheet and asserted
 * by pwrseq_exec.c's own busy-wait; POE_KILL is a latch, not a pulse — Q3
 * sustains it and recovery is the PSE's, so a duration would be a number
 * nothing could honour. mp_rpc.c has already range-checked `ms` against the
 * manifest envelope by the time we are called, so accepting and ignoring it is
 * the honest reading of an object whose min == max would be a lie.
 *
 * Guard classes come from the manifest and are enforced by m_obj_pulse() before
 * this runs: `pwr.poe.kill` is G3 (typed phrase + hold, admin floor) per FMT
 * §5.2, `pwr.rb.ov.reset` is G2 (typed device serial, admin floor).
 */
static int obj_pulse(void *user, size_t obj, uint32_t ms)
{
	const mp_obj_t *o = mp_obj_at(obj);

	ARG_UNUSED(user);
	ARG_UNUSED(ms);

	if (o == NULL) {
		return -EINVAL;
	}

	switch (sts_recov_pulse_action(o->id)) {
	case STS_RECOV_POE_KILL:
		return sts_pwrseq_poe_kill();
	case STS_RECOV_RB_OV_RESET:
		return sts_pwrseq_ov_clear();
	default:
		/* Every other pulsable object is a platform-owned pin with no
		 * seam yet; see the TODO block. */
		return -ENOTSUP;
	}
}

static int diag_action(void *user, uint8_t test, uint8_t step,
		       mp_diag_step_res_t *out)
{
	ARG_UNUSED(user);
	ARG_UNUSED(step);

	if (test == (uint8_t)MP_DIAG_I2C_SCAN) {
		uint8_t found[16];
		int n = sts_diag_i2c_scan(found);

		if (n < 0) {
			return n;
		}
		out->verdict = (n >= 10) ? (uint8_t)MP_DIAG_PASS
					 : (uint8_t)MP_DIAG_FAIL;
		out->value = n;
		(void)snprintf(out->text, sizeof(out->text), "%d devices", n);
		return 0;
	}

	/* The rest need platform-area actuation. */
	return -ENOTSUP;
}

/* ------------------------------------------------------------------ mirror */

void sts_mp_mirror_publish(const mp_mirror_in_t *frame)
{
	size_t cells;

	if ((frame == NULL) || (frame->ch == NULL) || (frame->attr == NULL)) {
		(void)atomic_inc(&mirror_rejects);
		return;
	}
	cells = (size_t)frame->rows * (size_t)frame->cols;
	if ((cells == 0U) || (cells > MP_MIRROR_CELLS)) {
		(void)atomic_inc(&mirror_rejects);
		return;
	}
	if (k_mutex_lock(&mirror_lock, K_MSEC(5)) != 0) {
		/* The tick has it; drop this frame rather than block ui. Counted
		 * so `mp status` can tell a live mirror from one that has been
		 * dropping every frame — nothing else moves when this happens. */
		(void)atomic_inc(&mirror_drops);
		return;
	}

	mirror_frame = *frame;
	memcpy(mirror_ch, frame->ch, cells);
	memcpy(mirror_attr, frame->attr, cells);
	mirror_frame.hint_count = 0U;
	if (frame->hint != NULL) {
		uint8_t n = frame->hint_count;

		if (n > UI_SURF_MAX_HINTS) {
			n = (uint8_t)UI_SURF_MAX_HINTS;
		}
		memcpy(mirror_hints, frame->hint, (size_t)n * sizeof(ui_hint_t));
		mirror_frame.hint_count = n;
	}
	mirror_frame.ch = mirror_ch;
	mirror_frame.attr = mirror_attr;
	mirror_frame.hint = mirror_hints;
	mirror_valid = true;

	(void)k_mutex_unlock(&mirror_lock);
}

/* --------------------------------------------------------- shell hand-off */

/**
 * Apply a link transition. **Called with the engine lock held.**
 *
 * The one place mp_set_link() is reached from, so the auth-window rule is stated
 * once: while prov_auth() has the lock released the shell thread is suspended
 * inside a credential check with an inbound frame half-decoded, and
 * mp_mode_exit() resets that decoder. So the half that must not wait — reverting
 * every lease, because the link *is* the dead-man's outermost condition
 * (mp_override.c §5.3) — runs here and now, and only the mode change is left to
 * mp_bypass(), which reaches it once mp_input() has returned.
 */
static void mp_link_apply_locked(bool up)
{
	if (!up && mp_auth_window) {
		(void)mp_ovr_set_link(&mp.ovr, false, (uint32_t)k_uptime_get_32());
		mp_exit_pending = true;
	} else {
		(void)mp_set_link(&mp, up);
	}
}

/** Drain a deferred link transition. **Called with the engine lock held.** */
static void mp_link_pending_apply_locked(void)
{
	atomic_val_t pend = atomic_set(&mp_link_pending, MP_LINK_PEND_NONE);

	if (pend != MP_LINK_PEND_NONE) {
		mp_link_apply_locked(pend == MP_LINK_PEND_UP);
	}
}

/* ------------------------------------------------- event channel 0x09 drain */

/**
 * Move staged board events into the engine. **Call with mp_lock held.**
 *
 * The consumer half of mp_events.c. It runs inside the wait sts_mp_tick()
 * already spends rather than taking a lock of its own, which is why it costs
 * the supervisor's pass budget nothing: sts_console.c's BUILD_ASSERT budgets
 * exactly one *timed* lock wait per pass and this adds none. Contrast the
 * passthrough drain, which cannot do this — its producers hand over whole COBS
 * frames that go out through mp_stream_raw(), so it runs before the lock and
 * refuses on contention.
 *
 * Called before mp_tick() so pump_events() puts this pass's events on the wire
 * in this pass, not the next one.
 *
 * Subscription is read from the engine (mp_stream_is_sub) rather than from the
 * lock-free armed mask, because here we hold the lock and can have the exact
 * answer. With no subscriber the backlog is discarded: leaving it would fill
 * the engine's queue with events nobody asked for and then deliver them, stale,
 * to whoever subscribed next.
 *
 * Peek/post/pop rather than pop/post, so an event the engine's own queue has no
 * room for stays staged for the next pass instead of being lost between two
 * queues — the same shape as sts_mp_tunnel_drain().
 */
static void mp_event_drain_locked(void)
{
	unsigned int n;
	uint32_t lost;

	if (!mp_stream_is_sub(&mp.st, (uint8_t)MP_CH_EVENT)) {
		sts_mp_event_purge();
		return;
	}

	/*
	 * The staging queue's overflow is the host's loss too, and key 8 of the
	 * event record is the only field that says a gap happened. Folded before
	 * the drain so the marker cannot arrive after the batch it belongs to.
	 */
	lost = sts_mp_event_take_drops();
	if (lost != 0U) {
		(void)mp_stream_event_drop_note(&mp.st, lost);
	}

	/*
	 * MP_EVQ_LEN is the engine queue's depth and therefore the most this
	 * loop could ever place there; the operative limits are the empty queue
	 * and the engine's -ENOSPC. The cap is here so the work inside the lock
	 * is bounded by a constant a reader can check rather than by two
	 * conditions in the body.
	 */
	for (n = 0U; n < (unsigned int)MP_EVQ_LEN; n++) {
		mp_ev_t ev;

		/*
		 * Room FIRST, and it is not an optimisation: mp_stream_event()
		 * counts a drop on -ENOSPC, so offering an event to a full
		 * engine queue and then keeping it staged would report a loss
		 * to the host on key 8 for an event that was never lost. The
		 * one counter a technician has to be able to believe is the one
		 * that says "there is a gap here".
		 */
		if (mp_stream_event_count(&mp.st) >= (size_t)MP_EVQ_LEN) {
			break;
		}
		if (!sts_mp_event_peek(&ev)) {
			break;
		}
		if (mp_post_event(&mp, ev.kind, ev.sub, ev.id, ev.edge,
				  ev.value, ev.mono_ms,
				  (ev.text[0] != '\0') ? ev.text : NULL) != 0) {
			/* Unreachable with the check above; if it ever happens,
			 * stopping loses nothing that is not already counted. */
			break;
		}
		sts_mp_event_pop();
	}
}

/* --------------------------------------------------------- firmware vetoes */

void sts_mp_veto(sts_mp_veto_t subject)
{
	/*
	 * No arming test, unlike sts_mp_post_event(). A veto's consumer is the
	 * lease table, not a subscriber: it has to run whether or not anyone is
	 * watching channel 0x09, and gating it on a subscription would make the
	 * safety property depend on a technician's stream being open. The call
	 * is one atomic OR either way, and its producers are 4 Hz (the
	 * sequencer's action drain) and 1 Hz (the antenna supervisor), not the
	 * 1 kHz scan.
	 */
	if ((subject <= STS_MP_VETO_NONE) || (subject >= STS_MP_VETO_COUNT)) {
		return;
	}

	(void)atomic_or(&mp_veto_pending, (atomic_val_t)STS_MP_VETO_BIT(subject));
	(void)atomic_inc(&mp_veto_raised);
}

/**
 * Withdraw the overrides the board has made untrue. **Call with mp_lock held.**
 *
 * The consumer half of the veto seam. Runs inside the wait sts_mp_tick() already
 * spends, so it adds no timed lock wait to the supervisor's pass — the same
 * arrangement, and the same reason, as mp_event_drain_locked() above.
 *
 * Claimed with atomic_set(), not read-then-clear: a subject raised while this
 * function is running belongs to the NEXT pass, and clearing after the work
 * would swallow it. The cost of the swap-first order is at worst one redundant
 * veto next pass, which answers -ENOENT.
 */
static void mp_veto_drain_locked(void)
{
	atomic_val_t pend = atomic_set(&mp_veto_pending, 0);
	unsigned int s;

	if (pend == 0) {
		return;
	}

	for (s = (unsigned int)STS_MP_VETO_NONE + 1U;
	     s < (unsigned int)STS_MP_VETO_COUNT; s++) {
		const char *const *ids;
		const char *reason;
		size_t n = 0U;
		size_t i;

		if ((pend & (atomic_val_t)STS_MP_VETO_BIT(s)) == 0) {
			continue;
		}
		mp_veto_applied++;

		reason = sts_mp_veto_reason((sts_mp_veto_t)s);
		ids = sts_mp_veto_objects((sts_mp_veto_t)s, &n);

		for (i = 0U; i < n; i++) {
			int obj = mp_obj_find(ids[i]);
			int rc;

			if (obj < 0) {
				/* A manifest rename that missed this table.
				 * Loud, because the veto it was meant to carry
				 * did not happen — tests/host/test_mp_veto.c
				 * resolves every id in the table for exactly
				 * this reason, so reaching here means the two
				 * were changed apart. */
				LOG_ERR("MP veto: manifest has no object `%s`; "
					"an override on it cannot be withdrawn",
					ids[i]);
				continue;
			}

			rc = mp_veto(&mp, (size_t)obj, reason);
			/*
			 * -ENOENT is the ordinary answer and is not an error:
			 * almost every veto is raised with no lease on the
			 * object, because the board sheds loads far more often
			 * than a technician overrides one. Logging it would put
			 * a line in the operator's log on every boot's
			 * bring-up. rc == 0 is already audited at LOGR_WARN and
			 * streamed as MP_OVR_EV_VETO by the engine's own event
			 * sink, so there is nothing to add here either.
			 */
			if ((rc != 0) && (rc != -ENOENT)) {
				LOG_ERR("MP veto on `%s` failed (%d)", ids[i],
					rc);
			}
		}
	}
}

void sts_mp_veto_stats(uint32_t *raised, uint32_t *applied)
{
	if (raised != NULL) {
		*raised = (uint32_t)atomic_get(&mp_veto_raised);
	}
	if (applied != NULL) {
		*applied = mp_veto_applied;
	}
}

/**
 * The shell bypass callback: every console byte, on the shell thread.
 *
 * shell_set_bypass() is called *outside* the lock. It touches shell state, not
 * engine state, and calling it under mp_lock would put the console's own
 * bookkeeping inside a critical section the tick contends for.
 */
static void mp_bypass(const struct shell *sh, uint8_t *data, size_t len)
{
	bool left;

	ARG_UNUSED(sh);

	mp_engine_lock();
	(void)mp_input(&mp, data, len);

	/*
	 * Deferred work, at the first point on this stack where no engine state
	 * is live — mp_input() has returned, so resetting the frame decoder is
	 * safe here and was not safe where either of these was reported.
	 *
	 * The link first: this thread is the *reason* a notification deferred
	 * (it is the only other contender for the lock), so draining it here
	 * rather than waiting for the tick is what keeps a busy host from
	 * postponing its own dead-man. mp_link_apply_locked() cannot re-defer
	 * from here — mp_auth_window is false once prov_auth() has returned, and
	 * prov_auth() is reached through mp_input().
	 */
	mp_link_pending_apply_locked();

	/*
	 * Then the mode change a BREAK or DTR drop left behind because it landed
	 * while prov_auth() had the lock released.
	 */
	if (mp_exit_pending) {
		mp_exit_pending = false;
		(void)mp_mode_exit(&mp);
	}

	left = (mp_mode(&mp) != (uint8_t)MP_MODE_MP);
	/* A `stream.sub` arrives on this path, so the tees learn about it here
	 * rather than up to a tick later. */
	mp_armed_refresh();
	mp_engine_unlock();

	if (left) {
		/* The exit magic, sys.mode, or a deferred link drop brought us
		 * back. */
		shell_set_bypass(mp_shell, NULL);
		LOG_INF("MP mode left; shell restored");
	}
}

bool sts_mp_active(void)
{
	bool active;

	if (!mp_started) {
		return false;
	}
	mp_engine_lock();
	active = (mp_mode(&mp) == (uint8_t)MP_MODE_MP);
	mp_engine_unlock();
	return active;
}

bool sts_mp_shell_tap(uint8_t b)
{
	bool entered;

	if (!mp_started) {
		return false;
	}
	mp_engine_lock();
	entered = (mp_shell_byte(&mp, b) == 1);
	mp_armed_refresh();
	mp_engine_unlock();

	if (entered) {
		shell_set_bypass(mp_shell, mp_bypass);
		LOG_INF("MP mode entered by magic");
	}
	return entered;
}

void sts_mp_notify_link(bool up)
{
	if (!mp_started) {
		return;
	}

	/*
	 * K_NO_WAIT, and it is the same load-bearing argument as
	 * sts_mp_stream_raw()'s rather than an optimisation.
	 *
	 * The caller is sts_usb_poll(), which sts_console.c runs on the console
	 * supervisor immediately before sts_mp_tick() — the same pass, and
	 * sts_console.c's BUILD_ASSERT budgets exactly one timed lock wait in it.
	 * A 50 ms wait here would make the worst interval between two successful
	 * ticks (5 + 1) * (250 + 50 + 50) + 150 = 2250 ms, past the 2000 ms
	 * MP_TICK_MAX_MS an override's dead-man is allowed to take to revert: a
	 * wait added to *protect* the revert would be what broke its deadline.
	 * And the supervisor also feeds sts_liveness_feed(), so an unbounded wait
	 * would stop the TPS3430 kick and cold-cycle the board — a technician
	 * closing a terminal window would reboot the grandmaster.
	 *
	 * So: apply it here when the engine is free, which is the ordinary case
	 * (a link that just went away is not sending bytes, and the shell thread
	 * is the only other contender), and park it otherwise. The tick two lines
	 * later in the same pass, or mp_bypass() on the thread that caused the
	 * contention, drains it — inside the deadline the BUILD_ASSERT proves.
	 */
	if (k_is_in_isr() || (k_mutex_lock(&mp_lock, K_NO_WAIT) != 0)) {
		atomic_set(&mp_link_pending,
			   up ? MP_LINK_PEND_UP : MP_LINK_PEND_DOWN);
		(void)atomic_inc(&mp_link_defers);
	} else {
		mp_link_apply_locked(up);
		mp_armed_refresh();
		(void)k_mutex_unlock(&mp_lock);
	}

	/*
	 * Outside the lock either way, and unconditional on a drop — including
	 * the deferred path. Taking the port back from mp_bypass() is what makes
	 * the console reachable again, it touches shell state and not engine
	 * state, and it must not be the thing that waits: a host that has closed
	 * the port is not going to send the exit magic. mp_bypass() is harmless
	 * if it is already running (it re-reads the engine's mode and clears the
	 * bypass itself), and clearing a bypass that is not installed is a no-op.
	 */
	if (!up && (mp_shell != NULL)) {
		shell_set_bypass(mp_shell, NULL);
	}
}

void sts_mp_notify_break(void)
{
	if (!mp_started) {
		return;
	}
	mp_engine_lock();
	if (mp_auth_window) {
		/* Same deferral as the link drop, and the same reason. A BREAK is
		 * not a dead-man failure, so there is no revert to hoist. */
		mp_exit_pending = true;
	} else {
		(void)mp_mode_exit(&mp);
	}
	mp_armed_refresh();
	mp_engine_unlock();

	if (mp_shell != NULL) {
		shell_set_bypass(mp_shell, NULL);
	}
}

int sts_mp_stream_raw(uint8_t ch, const uint8_t *data, size_t len)
{
	int rc;

	if (!mp_started) {
		return -ENODEV;
	}
	if (k_is_in_isr()) {
		/* k_mutex_lock() is illegal in an ISR at any timeout, and the
		 * assertion that says so is compiled out of a release build. */
		return -EBUSY;
	}
	/*
	 * K_NO_WAIT, deliberately, and it is load-bearing rather than an
	 * optimisation.
	 *
	 * The only caller is sts_mp_tunnel_drain(), which sts_mp_tick() runs
	 * before it takes the lock for the tick proper. Any timeout here would
	 * therefore add a second wait to the same supervisor pass, and
	 * sts_console.c's BUILD_ASSERT budgets exactly one: at the as-built
	 * numbers a 50 ms wait here would make the worst interval between two
	 * successful ticks (5 + 1) * (250 + 50 + 50) = 2100 ms, past the 2000 ms
	 * MP_TICK_MAX_MS an override's dead-man is allowed to take to revert. Not
	 * waiting at all keeps the pass at (250 + 50) and the assertion's premise
	 * true.
	 *
	 * It also matches what this function already promised: "drops the bytes on
	 * contention rather than waiting: a tee is best-effort passthrough". The
	 * bytes stay queued in the staging ring and the next pass retries them.
	 */
	if (k_mutex_lock(&mp_lock, K_NO_WAIT) != 0) {
		return -EBUSY;
	}
	rc = mp_stream_raw(&mp, ch, data, len);
	(void)k_mutex_unlock(&mp_lock);
	return rc;
}

void sts_mp_tick(void)
{
	if (!mp_started) {
		return;
	}

	/*
	 * Passthrough bytes first, and deliberately *outside* the engine lock.
	 *
	 * The producers — platform/gnss.c on the priority-6 GNSS thread and the
	 * UART7 receive ISR — may not take a mutex, so they only stage into
	 * mp_tunnel.c's ring; this is the thread that frames the result. Doing it
	 * before the lock attempt means a contended engine costs a deferred drain
	 * and not a dropped one, and it keeps the tee out of the tick's own
	 * timeout budget: sts_mp_tunnel_drain() bounds itself by record count and
	 * ends the pass on the first -EBUSY.
	 */
	sts_mp_tunnel_drain();

	/*
	 * Bounded, and a miss is tolerated. An unbounded wait here would let a
	 * shell thread stuck in a long call stall the console supervisor past
	 * CONFIG_STS1000_LIVENESS_DEADLINE_MS, at which point the supervisor
	 * stops kicking the TPS3430 and the board cold-cycles — a maintenance
	 * login would reboot the grandmaster. The budget arithmetic is in
	 * mp_glue.h and is BUILD_ASSERTed against the caller's period in
	 * sts_console.c.
	 */
	if (k_mutex_lock(&mp_lock, K_MSEC(STS_MP_TICK_LOCK_MS)) != 0) {
		mp_tick_misses++;
		if (mp_tick_miss_run < UINT8_MAX) {
			mp_tick_miss_run++;
		}
		if (mp_tick_miss_run > mp_tick_miss_worst) {
			mp_tick_miss_worst = mp_tick_miss_run;
		}
		/*
		 * Once per excursion, not once per pass: a run this long means the
		 * dead-man's revert deadline is no longer guaranteed, which is an
		 * operational fault worth a record even though the next successful
		 * tick still reverts.
		 */
		if (mp_tick_miss_run == (uint8_t)(STS_MP_TICK_MISS_MAX + 1U)) {
			LOG_ERR("MP tick starved: %u consecutive misses, dead-man "
				"revert may exceed %u ms",
				(unsigned int)mp_tick_miss_run,
				(unsigned int)MP_TICK_MAX_MS);
			sts_log((uint8_t)LOGR_SUB_MCP, (uint8_t)LOGR_ERR,
				"MP service tick starved (%u misses); override "
				"dead-man latency is no longer bounded",
				(unsigned int)mp_tick_misses);
		}
		return;
	}

	mp_tick_miss_run = 0U;
	/*
	 * Firmware vetoes first, ahead of both the link transition and the tick.
	 *
	 * All three can revert the same lease, and whichever runs first owns the
	 * reason a technician is left with. A veto's reason is a statement about
	 * the BOARD — "Rb 26 V OV latch tripped" — and the other two are
	 * statements about the console session ("keepalive or link lost", "lease
	 * expired"). When the hardware acted and the cable was pulled in the same
	 * 250 ms, the hardware is the answer to "why did my override go away".
	 */
	mp_veto_drain_locked();
	/*
	 * Before mp_tick(), not after: a parked link drop is a dead-man failure
	 * that has already happened, and applying it first means this pass's
	 * mp_ovr_tick() runs against the true link state instead of reverting the
	 * same leases one pass later for the weaker keepalive reason.
	 */
	mp_link_pending_apply_locked();
	mp_event_drain_locked();
	(void)mp_tick(&mp);
	/*
	 * The unconditional republication of the armed mask. Every other call
	 * site is an event; this one is what bounds how stale the mask can get
	 * when the host stops sending — a lease that expires here, or a dead-man
	 * that drops the session, silences the tees within one tick.
	 */
	mp_armed_refresh();
	(void)k_mutex_unlock(&mp_lock);
}

/* ------------------------------------------------------------ shell command */

static int cmd_mp_enter(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t hash;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mp_started) {
		shell_error(sh, "MP engine is not running");
		return -ENODEV;
	}
	mp_shell = sh;

	mp_engine_lock();
	(void)mp_set_link(&mp, true);
	(void)mp_mode_enter(&mp);
	hash = mp_manifest_hash_cached(&mp);
	mp_armed_refresh();
	mp_engine_unlock();

	shell_print(sh, "MP mode: proto %u, manifest %u objects, hash 0x%08x",
		    (unsigned int)MP_PROTO_VER, (unsigned int)mp_obj_count(),
		    (unsigned int)hash);
	/*
	 * Both escapes named here are ones this image actually implements, which
	 * the previous wording was not: it offered "BREAK", and no BREAK is
	 * detectable on this console. The port is CDC-ACM (DT_CHOSEN(zephyr_shell_uart)
	 * is cdc_acm_uart0; the board has no physical console UART), Zephyr's
	 * cdc_acm_driver_api publishes no .err_check — so uart_err_check() answers
	 * -ENOSYS and UART_BREAK can never be read — and cdc_acm_class_handle_req()
	 * handles only SET_LINE_CODING and SET_CONTROL_LINE_STATE, so the USB CDC
	 * SEND_BREAK request is refused with no application-visible surface at all.
	 * Naming an escape that does not exist is worse than naming none: it is
	 * the instruction a technician follows while the shell is unreachable.
	 *
	 * The exit magic is spelled out in keystrokes because a human is reading
	 * this on a terminal, and "\x01" is not something one types.
	 */
	shell_print(sh, "to return to the shell: type Ctrl-A M P 0 Ctrl-B "
			"(\\x01MP0\\x02), or close the port");
	shell_set_bypass(sh, mp_bypass);
	return 0;
}

static int cmd_mp_exit(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mp_started) {
		return -ENODEV;
	}
	mp_engine_lock();
	(void)mp_mode_exit(&mp);
	mp_armed_refresh();
	mp_engine_unlock();

	shell_set_bypass(sh, NULL);
	shell_print(sh, "MP mode left");
	return 0;
}

/**
 * Counters, printed from one consistent snapshot.
 *
 * The whole read runs under the engine lock rather than sampling field by field:
 * `mp status` reads engine state the tick mutates, and a report that mixes a
 * pre-revert lease count with a post-revert dead-man counter is a report that
 * misleads whoever is diagnosing the box. shell_print() is called inside the
 * lock, which costs the tick a few skipped passes on a slow console — bounded by
 * the tick's timeout, counted, and cheaper than a shadow copy of the context.
 */
static int cmd_mp_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mp_started) {
		shell_error(sh, "MP engine is not running");
		return -ENODEV;
	}

	mp_engine_lock();
	/*
	 * The link is printed next to the mode because it is now an input the
	 * board changes underneath the operator: a closed console port reverts
	 * every override (FMT §5.4), and "my override went away by itself" has
	 * exactly one honest first question. `defer` counts transitions that had
	 * to wait for a tick because the engine was busy — normally 0.
	 */
	shell_print(sh, "mode         %s (link %s, defer %u)",
		    (mp_mode(&mp) == (uint8_t)MP_MODE_MP) ? "mp" : "shell",
		    mp.ovr.link_up ? "up" : "down",
		    (unsigned int)atomic_get(&mp_link_defers));
	shell_print(sh, "manifest     %u objects, hash 0x%08x, ver %u",
		    (unsigned int)mp_obj_count(),
		    (unsigned int)mp_manifest_hash_cached(&mp),
		    (unsigned int)MP_MANIFEST_VER);
	shell_print(sh, "requests     %u (errors %u, notifications %u)",
		    mp.requests, mp.rpc_errors, mp.notifications);
	shell_print(sh, "frames       rx %u, msgs %u, crc-err %u, cobs-err %u",
		    mp.rx.frames, mp.rx.msgs, mp.rx.crc_errors,
		    mp.rx.cobs_errors);
	shell_print(sh, "records      %u sent (tx errors %u)", mp.records_sent,
		    mp.tx_errors);
	shell_print(sh, "session      %u, overrides %u/%u",
		    mp.ovr.sess.id, (unsigned int)mp_ovr_active(&mp.ovr),
		    (unsigned int)MP_LEASE_MAX);
	{
		const char *who = mp_ovr_session_user(&mp.ovr);

		shell_print(sh, "auth         role %s, user %s",
			    mp_role_name(mp_ovr_session_role(&mp.ovr)),
			    (who[0] != '\0') ? who : "-");
	}
	shell_print(sh, "safety       grants %u, vetoes %u, deadman %u, "
			"verify-fail %u, refusals %u",
		    mp.ovr.grants, mp.ovr.vetoes, mp.ovr.deadman_reverts,
		    mp.ovr.verify_failures, mp.ovr.refusals);
	/*
	 * Tick health. `late` is core's own count of gaps past MP_TICK_MAX_MS;
	 * `miss` is this file's count of passes that could not take the engine
	 * lock. A non-zero worst run at or above STS_MP_TICK_MISS_MAX means the
	 * dead-man's revert deadline was not guaranteed over that excursion.
	 */
	shell_print(sh, "tick         %u miss (run %u, worst %u/%u), %u late",
		    (unsigned int)mp_tick_misses, (unsigned int)mp_tick_miss_run,
		    (unsigned int)mp_tick_miss_worst,
		    (unsigned int)STS_MP_TICK_MISS_MAX,
		    (unsigned int)mp.ovr.late_ticks);
	/*
	 * The ui area publishes after every ui_render(), so "no frame yet" means
	 * it has not rendered one — CONFIG_STS1000_UI=n, or a boot this early —
	 * not that the hook is missing.
	 *
	 * mirror_valid is read without mirror_lock here, unlike in prov_mirror():
	 * this call site only picks a label, it does not go on to copy the cells
	 * the flag vouches for.
	 */
	shell_print(sh,
		    "mirror       %s (%u frames, %u keyframes, "
		    "%u publish-drop, %u rejected)",
		    mirror_valid ? "live" : "no frame yet (ui has not rendered)",
		    mp.mirror.frames, mp.mirror.keyframes,
		    (unsigned int)atomic_get(&mirror_drops),
		    (unsigned int)atomic_get(&mirror_rejects));
	shell_print(sh, "events       %u queued, %u dropped",
		    (unsigned int)mp_stream_event_count(&mp.st),
		    mp_stream_event_dropped(&mp.st));
	{
		uint32_t raised = 0U;
		uint32_t applied = 0U;

		/*
		 * The firmware-veto seam, and the two numbers do different jobs.
		 * `raised` counts what the platform area asked for — it climbs
		 * on every shed and every rail drop whether or not a lease
		 * existed, so a steady climb is the sequencer working, not a
		 * fault. The gap between it and `applied` is the backlog waiting
		 * for the next drain, normally zero. `safety … vetoes` above is
		 * the one that counts leases actually withdrawn.
		 */
		sts_mp_veto_stats(&raised, &applied);
		shell_print(sh, "veto seam    %u raised, %u drained",
			    (unsigned int)raised, (unsigned int)applied);
	}
	mp_engine_unlock();

	{
		uint32_t queued = 0U;
		uint32_t staged = 0U;
		uint32_t dropped = 0U;

		sts_mp_event_stats(&queued, &staged, &dropped);
		/*
		 * The stage in front of the engine's queue: what the 1 kHz scan
		 * and the alarm table handed over, and what would not fit.
		 * Printed separately from the line above because they fail
		 * separately — a non-zero `dropped` here means the board
		 * produced edges faster than a 250 ms drain could move them,
		 * which is a different diagnosis from an engine queue that
		 * overflowed because the host stopped reading. Both are folded
		 * into the number the host itself sees.
		 */
		shell_print(sh,
			    "event stage  %u queued, %u staged, %u dropped "
			    "(armed %u)",
			    queued, staged, dropped,
			    sts_mp_ch_armed((uint8_t)MP_CH_EVENT) ? 1U : 0U);
	}

	{
		uint32_t gnss = 0U;
		uint32_t rb = 0U;
		uint32_t nmea = 0U;
		uint32_t ubx = 0U;
		uint32_t dropped = 0U;

		sts_mp_tunnel_stats(&gnss, &rb, &nmea, &ubx, &dropped);
		/*
		 * Bytes staged per channel, and bytes that never made it —
		 * either the staging ring was full when a producer offered them
		 * or the frame could not be sent. Both are the tee being
		 * best-effort on purpose; see mp_tunnel.c.
		 */
		shell_print(sh,
			    "tees         gnss %u B, rb %u B, nmea %u B, ubx %u B "
			    "(dropped %u B)",
			    gnss, rb, nmea, ubx, dropped);
		shell_print(sh, "tee armed    nmea %u, ubx %u, gnss %u, rb %u",
			    sts_mp_ch_armed((uint8_t)MP_CH_NMEA) ? 1U : 0U,
			    sts_mp_ch_armed((uint8_t)MP_CH_UBX) ? 1U : 0U,
			    sts_mp_ch_armed((uint8_t)MP_CH_GNSS_PASS) ? 1U : 0U,
			    sts_mp_ch_armed((uint8_t)MP_CH_RB_PASS) ? 1U : 0U);
		shell_print(sh, "tunnels      gnss %s, rb %s",
			    sts_mp_tunnel_gnss_open() ? "OPEN (suspect)" : "closed",
			    sts_mp_tunnel_rb_open() ? "OPEN (suspect)" : "closed");
	}

	{
		uint32_t gnss = 0U;
		uint32_t rb = 0U;
		uint32_t refused = 0U;

		/*
		 * The other direction. mp_tunnel.c keeps these precisely so a
		 * refused host->device burst is visible somewhere — the protocol
		 * sends no reply for one — and this is the "somewhere" its
		 * comments name.
		 */
		sts_mp_tunnel_tx_stats(&gnss, &rb, &refused);
		shell_print(sh, "host->dev    gnss %u B, rb %u B (refused %u)",
			    gnss, rb, refused);
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	mp_sub,
	SHELL_CMD(enter, NULL, "Hand the console to the Maintenance Protocol",
		  cmd_mp_enter),
	SHELL_CMD(exit, NULL, "Return the console to the shell", cmd_mp_exit),
	SHELL_CMD(status, NULL, "Maintenance Protocol counters", cmd_mp_status),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(mp, &mp_sub, "Field Maintenance Tool protocol", NULL);

/* --------------------------------------------------------------------- init */

/** Derive the G2 confirmation string from the board identity. */
static void fill_serial(void)
{
	uint8_t uid[12];
	ssize_t n = hwinfo_get_device_id(uid, sizeof(uid));
	size_t i;
	size_t w = 0U;

	(void)strcpy(mp_serial, "STS1000-");
	w = strlen(mp_serial);
	if (n <= 0) {
		(void)strcpy(&mp_serial[w], "UNPROVISIONED");
		return;
	}
	for (i = 0U; (i < (size_t)n) && ((w + 2U) < sizeof(mp_serial)); i++) {
		static const char hexd[] = "0123456789ABCDEF";

		mp_serial[w++] = hexd[(uid[i] >> 4) & 0x0FU];
		mp_serial[w++] = hexd[uid[i] & 0x0FU];
	}
	mp_serial[w] = '\0';
}

int sts_mp_start(void)
{
	mp_wiring_t w;
	uint32_t hash;
	unsigned int i;
	int rc;

	if (mp_started) {
		return 0;
	}
	if (!device_is_ready(mp_uart)) {
		LOG_ERR("console UART not ready; MP unavailable");
		return -ENODEV;
	}

	fill_serial();

	/*
	 * The two staging rings, before anything can be armed: the producers gate
	 * on sts_mp_ch_armed(), which stays 0 until mp_armed_refresh() runs under
	 * the lock below, so binding them here cannot race a producer.
	 */
	sts_mp_tunnel_init();
	sts_mp_event_init();

	for (i = 0U; i < 2U; i++) {
		mp_reasm[i].buf = mp_reasm_buf[i];
		mp_reasm[i].cap = sizeof(mp_reasm_buf[i]);
	}

	memset(&w, 0, sizeof(w));
	w.tx = mp_tx;
	w.mono_ms = mono_ms;
	w.model = "STS1000";
	w.serial = mp_serial;
	w.fw_version = MP_FW_VERSION;
	w.boot_version = "mcuboot";
	w.board_id = mp_serial;
	w.telem = prov_telem;
	w.pps = prov_pps;
	w.ilk = prov_ilk;
	w.mirror = prov_mirror;
	w.bundle = prov_bundle;
	w.time_get = prov_time;
	w.apply = obj_apply;
	w.obj_read = obj_read;
	w.pulse = obj_pulse;
	w.diag = diag_action;
	w.cfg_commit = prov_cfg_commit;
	w.auth = prov_auth;
	w.img = sts_dfu_port();
	/*
	 * The multi-IC orchestrator behind the six FMT §9 `fw.*` methods. A NULL
	 * here is not an error — mp.h makes every `fw.*` method answer
	 * MP_E_NOTSUP — which is precisely why it has to be set deliberately:
	 * the failure mode of forgetting is a protocol that politely reports the
	 * feature does not exist, on a build that contains all of it.
	 */
	w.fwupd = sts_fwupd_mp_port();
	/*
	 * The host->device direction of channels 0x07/0x08. Leaving this NULL is
	 * how the tunnels stayed device-to-host for as long as they did: core
	 * null-checks it (mp_rpc.c:3307) and answers politely, so the whole
	 * inbound path — sink, policy header and its suite — sat in the tree with
	 * no caller and was collected out of the image. mp_tunnel.c's header used
	 * to describe that as "still not wired"; this is the line that changed it.
	 */
	w.raw_tx = prov_raw_tx;
	w.raw_tx_user = NULL;
	w.cfg = sts_cfg();
	w.log = sts_logring();
	w.scratch = mp_scratch;
	w.scratch_len = sizeof(mp_scratch);
	w.reasm = mp_reasm;
	w.reasm_n = 2U;
	w.mirror_prev_ch = mirror_prev_ch;
	w.mirror_prev_attr = mirror_prev_attr;
	w.mirror_cells = MP_MIRROR_CELLS;

	/*
	 * Under the lock even though this runs single-threaded before the console
	 * supervisor exists: "every touch of `mp` is inside mp_lock" is a rule a
	 * reviewer can check mechanically, and one documented exception is how it
	 * stops being one. mp_started is published last, so a shell command racing
	 * bring-up sees "not running" rather than a half-built context.
	 */
	mp_engine_lock();
	rc = mp_init(&mp, &w);
	hash = (rc == 0) ? mp_manifest_hash_cached(&mp) : 0U;
	if (rc == 0) {
		mp_armed_refresh();
	}
	mp_engine_unlock();
	if (rc != 0) {
		LOG_ERR("mp_init failed (%d)", rc);
		return rc;
	}

	mp_started = true;
	LOG_INF("MP ready: %u objects, hash 0x%08x, serial %s",
		(unsigned int)mp_obj_count(), (unsigned int)hash, mp_serial);
	return 0;
}

#endif /* CONFIG_STS1000_CONSOLE && CONFIG_STS1000_MP */
