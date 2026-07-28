/*
 * STS1000 "Meridian" — console area: when the maintenance link is up.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * FMT §5.4 makes the console link one of the dead-man's conditions, and
 * mp_override.c:mp_ovr_set_link() calls it "the dead-man's outermost condition"
 * — a drop reverts every override there and then rather than waiting for the
 * keepalive to age out. The engine cannot observe that condition itself: MP
 * runs on CDC-ACM #0 through shell_set_bypass() and never touches the USB
 * device stack, so *something else* has to look at the port and say so.
 *
 * That something is sts_usb.c, and this header is the decision it makes. It is
 * Zephyr-free by construction for the same reason `sts_mp_tunnel_policy.h` is:
 * every way it can be wrong is silent, and one of them shipped.
 *
 *   THE DEFECT THIS EXISTS TO PIN. sts_usb.c polled DTR (for
 *   sts_usb_configured()) and tracked VBUS edges (for the attach gate), and
 *   neither told MP anything. A technician who pulled the cable mid-session
 *   left the board with `mp_bypass` still installed on the shell UART and the
 *   overrides still asserted: the port came back with the Zephyr shell
 *   unreachable, escapable only by a byte sequence nobody at a terminal was
 *   going to guess. The keepalive dead-man still expired the leases a few
 *   seconds later, so the failure was bounded — and completely invisible until
 *   somebody plugged in again.
 *
 * Three rules, and each of them is a way to get it wrong:
 *
 *   EDGE, NOT LEVEL. sts_mp_notify_link() is cheap but not free (it tries the
 *   engine mutex), and the supervisor calls this at 4 Hz forever. Re-reporting
 *   a level would put a lock attempt on every pass of a loop that also feeds
 *   the watchdog.
 *
 *   VBUS LOSS FORCES THE LINK DOWN, WITHOUT CONSULTING DTR. DTR on a CDC-ACM
 *   port is `line_state`, a host-written byte the class driver only clears when
 *   the *controller* reports USB_DC_DISCONNECTED (cdc_acm_reset_port(), Zephyr
 *   subsys/usb/device/class/cdc_acm.c). A board that gates its own attach on
 *   PE2 tears the stack down with usb_disable() the moment VBUS goes, and there
 *   is no guarantee that callback ever runs first — so the last DTR sample can
 *   read "asserted" across a cable pull. Sampling DTR and believing it would
 *   reproduce the exact bug above.
 *
 *   AND IT HAPPENS BEFORE THE DETACH. The revert path actuates hardware
 *   through obj_apply() and logs; running it after usb_disable() would be
 *   running it with the diagnostic surface already gone. sts_usb.c already
 *   orders sts_mcp_notify_link_down() ahead of usb_detach() for the weaker
 *   reason that a held response would wedge the input path — the same order,
 *   with more at stake.
 *
 * The decision is a pure function of (previous sample, VBUS now, DTR now), so
 * sts_usb_poll() is a switch over the verdict and this header is what the host
 * suite exercises — against the real mp_ctx_t, so "the caller decides to call"
 * and "the call reverts the lease" are proved as one chain rather than
 * separately.
 */

#ifndef STS1000_ZEPHYR_CONSOLE_STS_CONSOLE_LINK_POLICY_H_
#define STS1000_ZEPHYR_CONSOLE_STS_CONSOLE_LINK_POLICY_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** The MP link transition a pass produced, if any. */
typedef enum {
	/** Nothing changed; do not call sts_mp_notify_link(). */
	STS_LINK_MP_NONE = 0,
	/** A host opened the console port: sts_mp_notify_link(true). */
	STS_LINK_MP_UP,
	/** DTR dropped or VBUS went: sts_mp_notify_link(false). */
	STS_LINK_MP_DOWN,
} sts_link_mp_t;

/** What sts_usb_poll() must do this pass, in the order the fields are listed. */
typedef struct {
	/** MP link transition to report, before any detach below. */
	sts_link_mp_t mp;
	/** Call sts_mcp_notify_link_down(); VBUS-driven, ACM1's own session. */
	bool mcp_down;
	/** Call usb_enable() — VBUS appeared. */
	bool attach;
	/** Call usb_disable() — VBUS went. Always last. */
	bool detach;
} sts_link_act_t;

/** The sampler's memory. Zero-initialised is "no VBUS, no host". */
typedef struct {
	bool vbus; /**< last VBUS sample */
	bool dtr;  /**< last *believed* console DTR; forced false with VBUS */
} sts_link_state_t;

/**
 * Seed the sampler from the boot-time VBUS reading.
 *
 * sts_usb_start() attaches directly rather than through a poll, so without this
 * the first sts_console_link_step() would see a rising VBUS edge that already
 * happened and call usb_enable() a second time. DTR is deliberately left false:
 * the engine has not started yet (sts_mp_start() runs after sts_usb_start(), see
 * sts_console_start()), so a link-up reported here would be dropped on the floor
 * by mp_glue.c's `mp_started` gate and never repeated. Leaving it false makes
 * the first poll after bring-up publish the truth as an edge.
 */
static inline void sts_console_link_seed(sts_link_state_t *s, bool vbus)
{
	s->vbus = vbus;
	s->dtr = false;
}

/**
 * Fold one (VBUS, DTR) sample into @p s and say what the caller must do.
 *
 * @param s     Sampler state; updated in place.
 * @param vbus  USB_VBUS_SENSE (PE2) this pass.
 * @param dtr   Console CDC-ACM DTR this pass. Ignored unless @p vbus.
 */
static inline sts_link_act_t sts_console_link_step(sts_link_state_t *s, bool vbus,
						   bool dtr)
{
	sts_link_act_t a = { STS_LINK_MP_NONE, false, false, false };
	bool dtr_now;

	/*
	 * With no VBUS there is no port, so there is no DTR to believe — see the
	 * header. This is the single line that makes a cable pull a link drop.
	 */
	dtr_now = vbus && dtr;

	if (dtr_now != s->dtr) {
		a.mp = dtr_now ? STS_LINK_MP_UP : STS_LINK_MP_DOWN;
		s->dtr = dtr_now;
	}

	if (vbus != s->vbus) {
		s->vbus = vbus;
		if (vbus) {
			a.attach = true;
		} else {
			a.mcp_down = true;
			a.detach = true;
		}
	}

	return a;
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_CONSOLE_STS_CONSOLE_LINK_POLICY_H_ */
