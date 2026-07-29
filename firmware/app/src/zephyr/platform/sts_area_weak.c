/*
 * STS1000 "Meridian" — __weak no-op fallbacks for the glue-area entry points.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * The four glue areas (platform, net, console, ui) are developed in parallel
 * and are individually switchable via CONFIG_STS1000_{NET,CONSOLE,UI} — plus
 * CONFIG_STS1000_MP, which switches one sub-area of console on its own. Every
 * cross-area entry point declared in sts_app.h, storage/sts_store.h or
 * console/mp_glue.h that is *not* implemented by the platform area gets a weak
 * no-op here, so the image links whether or not the owning area is in the build.
 *
 * Overriding works because Zephyr links every zephyr_library — `app` included
 * — inside --whole-archive (zephyr/cmake/linker/ld/target.cmake), so a strong
 * definition in an area's object file always wins over the weak one here. It
 * does NOT depend on archive member ordering.
 *
 * A fallback must be a *safe* no-op, not a stub that pretends to succeed at
 * something observable. Hence:
 *   - the area start functions return 0 (nothing to start is not an error);
 *   - sts_ui_post_input() drops the event (no consumer);
 *   - sts_update_*() report "confirmed", because with no image manager there
 *     is nothing to confirm and reporting "pending" would make the supervisor
 *     retry forever.
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/toolchain.h>

#include "console/mp_glue.h"
#include "zephyr/sts_app.h"
#include "storage/sts_store.h"

__weak int sts_net_start(void)
{
	return 0;
}

__weak int sts_console_start(void)
{
	return 0;
}

__weak int sts_ui_start(void)
{
	return 0;
}

__weak void sts_ui_post_input(const sts_input_evt_t *evt)
{
	ARG_UNUSED(evt);
}

/*
 * The read-back for `ui.disp.bl`, which the console area calls. 0 is the honest
 * answer with no ui area in the image: TIM15_CH2's compare is whatever the PWM
 * driver left it at and nothing has programmed a backlight duty, so reporting
 * any other number would invent one. sts_app.h names this stub as the
 * before-the-ui-area value.
 */
__weak uint8_t sts_ui_backlight_pct(void)
{
	return 0U;
}

__weak bool sts_update_pending_confirm(void)
{
	return false;
}

__weak int sts_update_self_confirm(void)
{
	return 0;
}

/*
 * PFI fast-save hooks. The console area's storage backend (src/zephyr/storage/)
 * owns the strong definitions; these weak no-ops let the platform's PFI ISR,
 * the discipline loop, the gnss path and the shutdown sequencer call them
 * unconditionally, so a CONFIG_STS1000_CONSOLE=n image still links (nothing is
 * persisted, which is the correct behaviour with no storage backend).
 *
 * Keep this set complete against sts_store.h: every storage entry point the
 * always-compiled platform area calls needs a stub here, or the claim above is
 * false for exactly one symbol and only a `=n` build finds out.
 */
__weak void sts_store_critical_flush_from_isr(void)
{
}

/*
 * The blocking counterpart. pwrseq_exec.c:512 calls it on the shutdown path and
 * that file is in the always-compiled platform area, so its absence broke the
 * CONFIG_STS1000_CONSOLE=n link that the comment above claims works — the ISR
 * variant was stubbed and this one was not. Returns -ENODEV rather than 0: with
 * no storage backend nothing was persisted, and reporting success for a flush
 * that did not happen is the kind of lie a later caller would build on. Both
 * present call sites discard the value.
 */
__weak int sts_store_critical_flush(sts_critical_reason_t reason)
{
	ARG_UNUSED(reason);
	return -ENODEV;
}

__weak void sts_store_note_dac_code(uint16_t code)
{
	ARG_UNUSED(code);
}

__weak void sts_store_note_leap(int16_t current, int16_t pending, bool valid)
{
	ARG_UNUSED(current);
	ARG_UNUSED(pending);
	ARG_UNUSED(valid);
}

__weak void sts_store_note_log_cursor(uint32_t cursor)
{
	ARG_UNUSED(cursor);
}

/*
 * ---------------------------------------------------------------------------
 * Maintenance Protocol (console/mp_glue.h)
 * ---------------------------------------------------------------------------
 *
 * MP has a switch of its own inside the console area — CONFIG_STS1000_MP,
 * src/zephyr/console/Kconfig.mp — because it is a physical-access surface that
 * answers at role `none` before any credential is presented, and an operator has
 * to be able to take it out of the image. app/CMakeLists.txt drops mp_glue.c and
 * mp_tunnel.c when it is n, which leaves callers in three always-compiled
 * places:
 *
 *   src/zephyr/console/sts_console.c   sts_mp_start(), sts_mp_tick()
 *   src/zephyr/console/sts_usb.c       sts_mp_notify_link(), from the VBUS/DTR
 *                                      sampler that is compiled either way
 *   src/zephyr/platform/gnss.c         the byte tees and their arming predicates
 *   src/zephyr/platform/io_scan.c      sts_mp_post_event(), from the 1 kHz scan
 *   src/zephyr/platform/pwrseq_exec.c  sts_mp_veto(), from the action executor
 *                                      and the antenna-bias write
 *   src/zephyr/sts_app.c               sts_mp_post_event(), from sts_alarm_set()
 *
 * Same completeness rule as the storage set above, and the same reason: the set
 * below is complete against **every** function console/mp_glue.h declares, not
 * merely against today's call sites, so adding a call somewhere cannot break a
 * CONFIG_STS1000_MP=n link that nothing else in CI builds. The one omission is
 * deliberate and is not an omission: sts_mp_mirror_publish() is declared in
 * sts_app.h rather than mp_glue.h and already has its __weak fallback in
 * src/zephyr/sts_app.c — defining it a second time here would be two weak
 * definitions of one symbol.
 *
 * Every stub is a *safe* no-op in the sense the file header requires. Note in
 * particular what the three predicates do NOT do: they report "nothing is
 * listening", so the GNSS receive path's tee call sites collapse to a predicted
 * branch and stage nothing, and the tunnel setters report -ENOTSUP rather than
 * success, because reporting that a port was diverted when no engine exists to
 * divert it is exactly the lie the header warns about.
 */
__weak int sts_mp_start(void)
{
	return 0;
}

__weak void sts_mp_tick(void)
{
}

__weak bool sts_mp_active(void)
{
	return false;
}

__weak void sts_mp_notify_link(bool up)
{
	ARG_UNUSED(up);
}

__weak void sts_mp_notify_break(void)
{
}

/* False = "the entry magic did not complete", so the caller keeps handing the
 * byte to the shell — which is the whole behaviour of a build without MP. */
__weak bool sts_mp_shell_tap(uint8_t b)
{
	ARG_UNUSED(b);
	return false;
}

/* mp_glue.h's own name for "the engine has not started". */
__weak int sts_mp_stream_raw(uint8_t ch, const uint8_t *data, size_t len)
{
	ARG_UNUSED(ch);
	ARG_UNUSED(data);
	ARG_UNUSED(len);
	return -ENODEV;
}

__weak void sts_mp_tee_gnss(const uint8_t *data, size_t len)
{
	ARG_UNUSED(data);
	ARG_UNUSED(len);
}

__weak void sts_mp_tee_rb(const uint8_t *data, size_t len)
{
	ARG_UNUSED(data);
	ARG_UNUSED(len);
}

__weak void sts_mp_tee_nmea(const uint8_t *data, size_t len)
{
	ARG_UNUSED(data);
	ARG_UNUSED(len);
}

__weak void sts_mp_tee_ubx(const uint8_t *data, size_t len)
{
	ARG_UNUSED(data);
	ARG_UNUSED(len);
}

__weak bool sts_mp_ch_armed(uint8_t ch)
{
	ARG_UNUSED(ch);
	return false;
}

__weak bool sts_mp_gnss_tee_armed(void)
{
	return false;
}

__weak bool sts_mp_tunnel_gnss_open(void)
{
	return false;
}

__weak bool sts_mp_tunnel_rb_open(void)
{
	return false;
}

__weak int sts_mp_tunnel_set_gnss(bool open)
{
	ARG_UNUSED(open);
	return -ENOTSUP;
}

__weak int sts_mp_tunnel_set_rb(bool open)
{
	ARG_UNUSED(open);
	return -ENOTSUP;
}

__weak void sts_mp_tunnel_stats(uint32_t *gnss, uint32_t *rb, uint32_t *nmea,
				uint32_t *ubx, uint32_t *dropped)
{
	if (gnss != NULL) {
		*gnss = 0U;
	}
	if (rb != NULL) {
		*rb = 0U;
	}
	if (nmea != NULL) {
		*nmea = 0U;
	}
	if (ubx != NULL) {
		*ubx = 0U;
	}
	if (dropped != NULL) {
		*dropped = 0U;
	}
}

/* No engine means no tunnel, so there is nothing that may drive the port. */
__weak int sts_mp_tunnel_write(uint8_t ch, const uint8_t *data, size_t len)
{
	ARG_UNUSED(ch);
	ARG_UNUSED(data);
	ARG_UNUSED(len);
	return -ENOTSUP;
}

__weak void sts_mp_tunnel_tx_stats(uint32_t *gnss, uint32_t *rb,
				   uint32_t *refused)
{
	if (gnss != NULL) {
		*gnss = 0U;
	}
	if (rb != NULL) {
		*rb = 0U;
	}
	if (refused != NULL) {
		*refused = 0U;
	}
}

__weak void sts_mp_tunnel_drain(void)
{
}

__weak void sts_mp_tunnel_init(void)
{
}

/*
 * The event channel's producer and its drain. Same discipline as the tees: the
 * producer drops the record (there is no engine to stage it for) and the
 * counters report nothing rather than inventing activity, so `mp status` on a
 * CONFIG_STS1000_MP=n build cannot claim a stream that does not exist.
 */
__weak void sts_mp_post_event(uint8_t kind, uint8_t sub, uint16_t id,
			      uint8_t edge, int32_t value, uint32_t mono_ms,
			      const char *text)
{
	ARG_UNUSED(kind);
	ARG_UNUSED(sub);
	ARG_UNUSED(id);
	ARG_UNUSED(edge);
	ARG_UNUSED(value);
	ARG_UNUSED(mono_ms);
	ARG_UNUSED(text);
}

__weak bool sts_mp_event_peek(mp_ev_t *out)
{
	ARG_UNUSED(out);
	return false;
}

__weak void sts_mp_event_pop(void)
{
}

__weak void sts_mp_event_purge(void)
{
}

__weak uint32_t sts_mp_event_take_drops(void)
{
	return 0U;
}

__weak void sts_mp_event_stats(uint32_t *queued, uint32_t *staged,
			       uint32_t *dropped)
{
	if (queued != NULL) {
		*queued = 0U;
	}
	if (staged != NULL) {
		*staged = 0U;
	}
	if (dropped != NULL) {
		*dropped = 0U;
	}
}

__weak void sts_mp_event_init(void)
{
}

/*
 * The firmware veto. Dropping the request is the correct no-op: with no engine
 * there is no lease table, so there is no override to withdraw — and unlike the
 * tunnel setters, nothing here is being claimed to a caller that could act on
 * the answer, because the producer is fire-and-forget by design.
 */
__weak void sts_mp_veto(sts_mp_veto_t subject)
{
	ARG_UNUSED(subject);
}

__weak void sts_mp_veto_stats(uint32_t *raised, uint32_t *applied)
{
	if (raised != NULL) {
		*raised = 0U;
	}
	if (applied != NULL) {
		*applied = 0U;
	}
}
