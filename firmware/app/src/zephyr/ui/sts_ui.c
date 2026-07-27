/*
 * STS1000 "Meridian" — local-UI area entry point.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * sts_app.h: "ui area: display, touch, encoder, panel". This file is the glue
 * that binds the platform-neutral core/ui to the board:
 *
 *   - it owns the one ui_ctx_t and the tile surface core/ui renders into;
 *   - it implements the strong sts_ui_post_input(), the drop point for the
 *     button / touch / proximity events the platform's 1 kHz scan dispatches
 *     (sts_app.h), translating each onto a core/ui input;
 *   - the render thread (ui_local, priority 15, 10 Hz per ARCHITECTURE.md §6)
 *     drains that queue, folds in the TIM1 encoder delta, ticks the timeout,
 *     builds the health snapshot, renders, and blits the dirty tiles;
 *   - it publishes each rendered frame to the Maintenance Protocol panel mirror
 *     (sts_mp_mirror_publish(), see mirror_publish() below), which is what makes
 *     channel 0x0A and the `mirror.get` RPC answer with a real panel;
 *   - it drains the core/ui action queue: brightness/timeout edits are pushed
 *     back into cfg (and persisted), the lamp test drives the panel LED string,
 *     and reboot / factory-reset are executed here.
 *
 * Config mutations from this file (the brightness/timeout persist and the
 * factory reset) go through sts_cfg_lock() and sts_cfg_factory_reset() per the
 * sts_app.h contract: the tree is shared with the MCP engine, the shell backend
 * and the web plane, and none of them is internally locked.
 *
 * INPUT INTEGRATION STATUS (sts_app.h contract, as it now stands)
 * ---------------------------------------------------------------
 * The platform area already wires the scan to us: io_scan.c calls
 * sts_ui_post_input() with STS_INPUT_BUTTON / _LONG / _REPEAT / _TOUCH / _PROX.
 * We provide the strong definition, so no marker TODO is needed for the input
 * path — it is live. Encoder rotation is read here from TIM1 (the scan does not
 * carry it, by the sts_app.h note).
 *
 * WHAT THE HEALTH SNAPSHOT CAN AND CANNOT SEE
 * -------------------------------------------
 * sts_app.h is the ONLY cross-area interface, and it exposes the §3.8 quality
 * block, the alarm mask, cfg and time — but no aggregate health struct. So the
 * Home, Clocks and Alarms pages populate fully (their data is in the quality
 * block and the alarm mask), while the per-rail INA readings, fan/PoE figures,
 * per-satellite az/el/CN0, and the network/PTP counters have no typed cross-area
 * source and render as zero/empty. Wiring those needs a health getter added to
 * sts_app.h by the platform/net owners (that header is out of this area's
 * scope). Marked TODO(ui-health) at build_health().
 */

#include <zephyr/kernel.h>

#ifdef CONFIG_STS1000_UI

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/app_version.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/reboot.h>

#include "cfg/cfg.h"
#include "fault/fault.h"
#include "logring/logring.h"
#include "quality/quality.h"
#include "ui/ui.h"
#include "zephyr/sts_app.h"
#include "zephyr/ui/sts_factory_policy.h"
#include "zephyr/ui/sts_sky_policy.h"
#include "zephyr/ui/sts_ui.h"
#include "zephyr/ui/sts_ui_echo.h"

/*
 * The factory-reset key wipe is a net-area implementation
 * (src/zephyr/net/sts_web.c). Declared WEAK rather than reached through
 * net/sts_secops.h, exactly as console/sts_mcp.c declares it and as sts_web.c
 * declares sts_dfu_port(): with CONFIG_STS1000_NET=n the symbol resolves to
 * NULL, the panel reports the reset as config-only, and the image still links.
 */
extern int sts_sec_factory_wipe(void) __attribute__((weak));

LOG_MODULE_REGISTER(sts_ui, CONFIG_STS1000_LOG_LEVEL);

/** ARCHITECTURE.md §6: ui_local runs at priority 15, 10 Hz. */
#define UI_PRIO 15
#define UI_PERIOD_MS (1000U / STS_UI_RENDER_HZ)

/* ------------------------------------------------------------------ state */

static ui_ctx_t g_ui;

/* The live tile surface core/ui renders into; the display shadow is separate
 * and owned by ui_display.c. */
static char surf_ch[STS_UI_ROWS * STS_UI_COLS];
static uint8_t surf_attr[STS_UI_ROWS * STS_UI_COLS];
static ui_surface_t g_surf;

/*
 * The MP panel mirror's previous-frame store is sized for the as-built 60x20
 * grid (MP_MIRROR_ROWS/COLS in console/mp_glue.c, which is private to that area
 * so the constant cannot be referenced from here). A surface larger than that is
 * rejected by sts_mp_mirror_publish() SILENTLY — every frame dropped, `mp status`
 * still reporting the mirror wired. Fail the build instead; keep the two in step.
 */
BUILD_ASSERT((size_t)(STS_UI_ROWS * STS_UI_COLS) <= (size_t)(20u * 60u),
	     "panel surface exceeds the MP mirror's 60x20 cell store "
	     "(MP_MIRROR_CELLS in src/zephyr/console/mp_glue.c)");

/* Input queue fed by the io_scan thread through sts_ui_post_input(). Deep
 * enough to absorb a burst between two render ticks; drop-on-full so the
 * non-blocking scan thread is never delayed. */
#define UI_INQ_DEPTH 16
K_MSGQ_DEFINE(ui_inq, sizeof(sts_input_evt_t), UI_INQ_DEPTH, 4);

/*
 * Events sts_ui_post_input() could not enqueue, cumulative.
 *
 * Written by the io_scan thread (priority 11), read by the render thread, so it
 * is an atomic rather than a plain word. It is the *only* evidence that the
 * level state reconstructed below is incomplete — see sts_ui_echo.h for why the
 * reconstruction is lossy and what the consumer does about it.
 */
static atomic_t ui_inq_drops;

static struct k_thread ui_thread;
static K_THREAD_STACK_DEFINE(ui_stack, STS_UI_RENDER_STACK);

static int ui_liveness_id = -1;

/* --------------------------------------------------------------- skyplot */

/*
 * Oldest UBX-NAV-SAT frame the plot will draw, and oldest e-compass sweep the
 * rotation will trust.
 *
 * sts_app.h is explicit that the sky cache is a last-known-good that a stopped
 * receiver leaves in place indefinitely — "the right thing to draw for a second
 * or two and the wrong thing to draw for an hour". Both sources are 1 Hz, so
 * five seconds rides out four consecutive misses (a busy parse, a survey-in
 * reconfiguration, an I2C retry) and still blanks the plot long before an
 * operator could mistake a frozen sky for a live one.
 */
#define SKY_STALE_MS 5000u

/*
 * The §10.4 hard/soft-iron fit and the site declination, cached from cfg group
 * 0x0C by the applier below.
 *
 * Cached rather than read per frame for coherency, not speed: the seven keys
 * are one calibration and a render tick that read three of them from before a
 * commit and four from after would rotate the plot by an angle that was never
 * configured. sts_app.h dispatches appliers after the commit completes, so the
 * set this holds is always one that was committed together.
 *
 * Written by the committing thread (a web worker or the shell), read by
 * ui_local. Both are management threads and the failure mode of a torn read is
 * one frame at a wrong rotation, so the words are not individually atomic — but
 * `valid` is published LAST and cleared FIRST, so a reader never sees a partly
 * written calibration marked usable.
 */
static struct {
	int32_t off[3];  /**< cal.mag.off.*, milligauss */
	int32_t scl[3];  /**< cal.mag.scl.*, Q12 */
	uint32_t ref;    /**< cal.mag.ref, milligauss; 0 = never fitted */
	int16_t decl_ddeg;
	bool decl_valid; /**< cal.decl.ddeg is not STS_SKY_DECL_UNSET */
	bool valid;      /**< cal.mag.ref is non-zero */
} sky_cal;

/** What the rasteriser draws; rebuilt once per render tick. */
static sts_ui_sky_t sky_frame;

/*
 * Input echo for the panel mirror (sts_app.h sts_mp_mirror_publish).
 *
 * Written and read only by the render thread — handle_scan_event() and the
 * render tick both run there — so no lock is involved. These are the UI area's
 * own view of the scan it is fed; core/fault's debounced bitmap itself is behind
 * the platform's fault lock and is not exposed cross-area.
 *
 * `mirror_echo` carries the button levels and the lamp-test hold, both
 * reconstructed from an event stream that can drop; sts_ui_echo_sync() below
 * folds ui_inq_drops in once per frame and clears the echo whenever one was
 * lost, so a dropped release cannot latch a phantom held key into the mirror.
 */
static sts_ui_echo_t mirror_echo;
static int32_t mirror_enc_pos; /**< detents accumulated since encoder init */
static uint16_t mirror_touch_x;
static uint16_t mirror_touch_y;
static uint32_t mirror_touch_ms; /**< 0 = no coordinate has ever been read */

/* ------------------------------------------------------- input translation */

/*
 * Panel button -> logical UI event, on press (docs/sts1000_panel_controls.md
 * §3): 1 MENU, 2 BACK, 3 DISPLAY, 4 ACK, 5 INFO, 6 LAMP, 7 RESET (guarded),
 * ENC_BUTTON = Enter. RESET does nothing on a short press — only its long-press
 * arms the reboot dialog — so it maps to NONE here.
 */
static uint8_t button_press_kind(uint8_t sig)
{
	switch (sig) {
	case FAULT_SIG_BUTTON_1:
		return (uint8_t)UI_IN_FN;
	case FAULT_SIG_BUTTON_2:
		return (uint8_t)UI_IN_LEFT;
	case FAULT_SIG_BUTTON_3:
		return (uint8_t)UI_IN_DISPLAY;
	case FAULT_SIG_BUTTON_4:
		return (uint8_t)UI_IN_ACK;
	case FAULT_SIG_BUTTON_5:
		return (uint8_t)UI_IN_INFO;
	case FAULT_SIG_BUTTON_6:
		return (uint8_t)UI_IN_LAMP; /* handled specially (edge) */
	case FAULT_SIG_ENC_BUTTON:
		return (uint8_t)UI_IN_ENTER;
	case FAULT_SIG_BUTTON_7:
	default:
		return (uint8_t)UI_IN_NONE;
	}
}

static void ui_feed(uint8_t kind, int16_t delta, uint16_t x, uint16_t y)
{
	ui_input_t in;

	memset(&in, 0, sizeof(in));
	in.kind = kind;
	in.delta = delta;
	in.x = x;
	in.y = y;
	(void)ui_input(&g_ui, &in);
}

/** Handle one queued scan event (render-thread context). */
static void handle_scan_event(const sts_input_evt_t *e)
{
	switch (e->type) {
	case STS_INPUT_BUTTON: {
		uint8_t kind = button_press_kind(e->id);

		/* Mirror echo: the scan posts press (1) and release (0) for every
		 * FAULT_CLASS_BUTTON signal, so the bitmap tracks both edges —
		 * subject to the drop reconciliation in sts_ui_echo.h. */
		if (e->id < (uint8_t)FAULT_SIG_COUNT) {
			sts_ui_echo_button(&mirror_echo, e->id, e->value != 0);
		}

		if (e->id == FAULT_SIG_BUTTON_6) {
			/* LAMP is level: press = held, release = released. */
			mirror_echo.lamp = (e->value != 0);
			ui_feed((uint8_t)UI_IN_LAMP, mirror_echo.lamp ? 1 : 0,
				0u, 0u);
		} else if (e->value != 0 && kind != (uint8_t)UI_IN_NONE) {
			ui_feed(kind, 0, 0u, 0u); /* act on press only */
		}
		break;
	}
	case STS_INPUT_BUTTON_LONG:
		/* Only RESET's long-press is meaningful: it offers the reboot
		 * dialog (docs §3, §4 — guarded). */
		if (e->id == FAULT_SIG_BUTTON_7) {
			ui_feed((uint8_t)UI_IN_RESET_HOLD, 0, 0u, 0u);
		}
		break;
	case STS_INPUT_BUTTON_REPEAT:
		/* Panel buttons are discrete actions; nothing auto-repeats. The
		 * encoder is the scroll surface, and it is read separately. */
		break;
	case STS_INPUT_TOUCH: {
		uint16_t x = 0u;
		uint16_t y = 0u;
		int rc = ui_input_touch_read(&x, &y);

		if (rc == 1) {
			mirror_touch_x = x;
			mirror_touch_y = y;
			/* The scan's own timestamp, not now: this is when the
			 * controller asserted INT. A zero stamp means "never", so
			 * a coordinate captured in the first millisecond after boot
			 * is aged by one tick rather than reported as absent. */
			mirror_touch_ms = (e->mono_ms != 0u) ? e->mono_ms : 1u;
			ui_feed((uint8_t)UI_IN_TOUCH, 0, x, y);
		} else if (rc < 0) {
			/* Rail down or bus busy: still wake so a stale panel
			 * lights up for the operator who just touched it. */
			ui_feed((uint8_t)UI_IN_TOUCH, 0, 0u, 0u);
		}
		break;
	}
	case STS_INPUT_PROX:
		if (e->value != 0) {
			ui_feed((uint8_t)UI_IN_PROX, 0, 0u, 0u);
		}
		break;
	case STS_INPUT_ENCODER:
		/* The scan does not normally post this (TIM1 is read here), but
		 * honour it if some future source does. */
		mirror_enc_pos += (int32_t)e->value;
		ui_feed((uint8_t)UI_IN_ENCODER, e->value, 0u, 0u);
		break;
	default:
		break;
	}
}

/* sts_app.h: strong override of the weak drop-stub. Called from io_scan
 * (priority 11) and MUST NOT block, so it only enqueues. */
void sts_ui_post_input(const sts_input_evt_t *evt)
{
	if (evt == NULL) {
		return;
	}
	/*
	 * Drop-on-full: a stuck button must never delay the 1 kHz scan. Count
	 * what was lost — k_msgq_put()'s own return code, not a separate
	 * k_msgq_num_free_get() probe, because only the return code is exact
	 * and free of a window between the check and the put. The render thread
	 * reads this to decide whether its reconstructed level state can still
	 * be trusted (sts_ui_echo.h).
	 */
	if (k_msgq_put(&ui_inq, evt, K_NO_WAIT) != 0) {
		(void)atomic_inc(&ui_inq_drops);
	}
}

/* ----------------------------------------------------------- action drain */

/**
 * Persist one numeric UI cfg key and commit (runs the appliers).
 *
 * sts_app.h: sts_cfg() is not internally locked and the MCP engine, the shell
 * backend and the web plane write the same tree, so the STAGE must be bracketed
 * by sts_cfg_lock()/sts_cfg_unlock(). The lock is then RELEASED before
 * sts_cfg_commit(), which takes the mutex itself and then dispatches the
 * appliers with it RELEASED again (sts_app.c drops it before the dispatch loop).
 * Holding ours across the commit would undo exactly that: the mutex is recursive,
 * so it would not deadlock — it would quietly run every applier inside the config
 * critical section, parking every other cfg user behind display and socket I/O.
 */
static void ui_cfg_persist(uint16_t id, uint64_t val)
{
	cfg_ctx_t *c = sts_cfg();
	int rc;

	if (c == NULL) {
		return;
	}

	sts_cfg_lock();
	rc = cfg_set_u64(c, id, val);
	sts_cfg_unlock();

	if (rc != 0) {
		LOG_WRN("cfg_set 0x%04x failed (%d)", id, rc);
		return;
	}
	rc = sts_cfg_commit(NULL);
	if (rc != 0) {
		LOG_WRN("cfg_commit for 0x%04x failed (%d)", id, rc);
	}
}

static void handle_action(const ui_action_t *a)
{
	switch (a->kind) {
	case UI_ACTION_SET_BRIGHTNESS:
		ui_display_backlight_permille(ui_backlight_permille(&g_ui));
		ui_cfg_persist((uint16_t)CFG_ID_UI_BRIGHTNESS,
			       (uint64_t)a->arg);
		sts_log((uint8_t)LOGR_SUB_UI, (uint8_t)LOGR_INFO,
			"brightness set to %d%%", a->arg);
		break;
	case UI_ACTION_SET_TIMEOUT:
		ui_cfg_persist((uint16_t)CFG_ID_UI_TIMEOUT_S, (uint64_t)a->arg);
		sts_log((uint8_t)LOGR_SUB_UI, (uint8_t)LOGR_INFO,
			"blank timeout set to %d s", a->arg);
		break;
	case UI_ACTION_IDENTIFY:
		/*
		 * TODO(ui-identify): spec §6.4 identify is a blue RGB pulse, but
		 * the status RGB is platform-owned (TIM4) and sts_app.h exposes
		 * no request path. Until one is added we only log it; the panel
		 * LED string is deliberately NOT borrowed, to keep it free for
		 * the lamp test.
		 */
		sts_log((uint8_t)LOGR_SUB_UI, (uint8_t)LOGR_NOTICE,
			"identify %s (RGB request unavailable; see TODO)",
			a->arg ? "on" : "off");
		break;
	case UI_ACTION_LAMP_TEST:
		/* Drive the front-panel LED string full-on while held. */
		(void)sts_panel_led_set(a->arg ? 100u : 0u);
		sts_log((uint8_t)LOGR_SUB_UI, (uint8_t)LOGR_INFO,
			"lamp test %s", a->arg ? "on" : "off");
		break;
	case UI_ACTION_ACK_ALARMS:
		/*
		 * TODO(ui-ack): acknowledging a latched alarm needs
		 * fault_alarm_clear(), which is behind the platform's fault
		 * lock and not exposed by sts_app.h. Log the intent; the alarm
		 * list still shows live state from sts_alarms_active().
		 */
		sts_log((uint8_t)LOGR_SUB_UI, (uint8_t)LOGR_NOTICE,
			"alarm ack requested (id %d); clear path not exposed",
			a->arg);
		break;
	case UI_ACTION_REBOOT:
		sts_log((uint8_t)LOGR_SUB_UI, (uint8_t)LOGR_WARN,
			"operator requested reboot from the panel");
		(void)sts_panel_led_set(0u);
		k_sleep(K_MSEC(50)); /* let the log line drain */
		sys_reboot(SYS_REBOOT_WARM);
		break;
	case UI_ACTION_FACTORY_RESET: {
		/*
		 * Both halves, in the order sts_factory_policy.h fixes, then the
		 * reboot unconditionally.
		 *
		 * The panel plane used to run only the first half, so a factory
		 * reset driven from the front panel left the persisted TLS server
		 * key, its certificate and the ACME account key on /lfs and every
		 * outstanding web session valid — while reporting a clean unit.
		 * The web plane (sts_web.c pv_factory_reset) and the MCP plane
		 * (sts_mcp.c mcp_cfg_factory_reset) always did both; this is the
		 * third plane catching up to them, and
		 * tests/host/test_factory_policy.c now fails if any of the three
		 * regresses.
		 */
		sts_factory_steps_t st = {
			.cfg_reset_ok = false,
			.wipe_linked = (sts_sec_factory_wipe != NULL),
			.wipe_ok = false,
		};
		int rc;

		sts_log((uint8_t)LOGR_SUB_UI, (uint8_t)LOGR_WARN,
			"operator requested factory reset from the panel");
		/*
		 * sts_cfg_factory_reset(), not cfg_factory_reset(): it takes the
		 * cfg mutex and then runs EVERY group's appliers, so no subsystem
		 * is left serving pre-reset configuration if the cold reboot below
		 * does not happen (a failed sys_reboot, or a supervisor that gets
		 * there first). The bare core call reset the tree and told nobody.
		 */
		rc = sts_cfg_factory_reset();
		st.cfg_reset_ok = (rc == 0);
		if (rc != 0) {
			sts_log((uint8_t)LOGR_SUB_UI, (uint8_t)LOGR_ERR,
				"factory reset: config reset returned %d", rc);
		}

		/*
		 * AFTER the config reset, because that is what runs the appliers
		 * that push the emptied credential keys into the subsystems
		 * holding RAM copies of them (net/sts_secops.h). Run even when the
		 * config reset failed: failures accumulate, they do not
		 * short-circuit — a half-wiped unit reported as clean is the
		 * outcome this whole path exists to prevent.
		 */
		if (st.wipe_linked) {
			rc = sts_sec_factory_wipe();
			st.wipe_ok = (rc == 0);
			if (rc != 0) {
				sts_log((uint8_t)LOGR_SUB_UI, (uint8_t)LOGR_ERR,
					"factory reset: key wipe returned %d",
					rc);
			}
		} else {
			LOG_WRN("no net area in this image: factory reset is "
				"config-only, TLS key material is not erased");
			sts_log((uint8_t)LOGR_SUB_UI, (uint8_t)LOGR_WARN,
				"factory reset is config-only; no key wipe in "
				"this image");
		}

		if (sts_factory_result(&st) != 0) {
			sts_log((uint8_t)LOGR_SUB_UI, (uint8_t)LOGR_ERR,
				"factory reset INCOMPLETE; do not treat this "
				"unit as decommissioned");
		}

		/*
		 * Unconditional. The reboot is what clears RAM-only key material
		 * and mints the replacement TLS identity, so it is load-bearing
		 * even — especially — when a step above failed.
		 */
		if (sts_factory_should_reboot(&st)) {
			(void)sts_panel_led_set(0u);
			k_sleep(K_MSEC(50)); /* let the log lines drain */
			sys_reboot(SYS_REBOOT_COLD);
		}
		break;
	}
	default:
		break;
	}
}

static void drain_actions(void)
{
	ui_action_t a;

	while (ui_action_get(&g_ui, &a) == 0) {
		handle_action(&a);
	}
}

/* --------------------------------------------------------- health snapshot */

/** Days since 1970-01-01 -> civil date (Hinnant's algorithm, proleptic). */
static void civil_from_days(int64_t z, uint16_t *y, uint8_t *m, uint8_t *d)
{
	int64_t era;
	uint64_t doe;
	uint64_t yoe;
	uint64_t doy;
	uint64_t mp;
	uint64_t day;
	uint64_t mon;
	int64_t year;

	z += 719468;
	era = (z >= 0 ? z : z - 146096) / 146097;
	doe = (uint64_t)(z - era * 146097);              /* [0, 146096] */
	yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
	year = (int64_t)yoe + era * 400;
	doy = doe - (365 * yoe + yoe / 4 - yoe / 100);   /* [0, 365] */
	mp = (5 * doy + 2) / 153;                        /* [0, 11] */
	day = doy - (153 * mp + 2) / 5 + 1;              /* [1, 31] */
	mon = mp < 10 ? mp + 3 : mp - 9;                 /* [1, 12] */

	*y = (uint16_t)(year + (mon <= 2 ? 1 : 0));
	*m = (uint8_t)mon;
	*d = (uint8_t)day;
}

static void fill_time(ui_health_t *h, const quality_block_t *q)
{
	uint64_t tai_ns = 0u;
	int64_t utc_s;

	h->uptime_s = (uint32_t)(sts_mono_ms() / 1000U);

	if (sts_time_tai_ns(&tai_ns) != 0 || sts_time_is_fallback()) {
		h->utc.valid = false;
		return;
	}

	/* TAI seconds since the PTP/Unix epoch, less the tracked leap offset. */
	utc_s = (int64_t)(tai_ns / 1000000000ULL) - (int64_t)q->leap_current_s;
	if (utc_s < 0) {
		h->utc.valid = false;
		return;
	}

	civil_from_days(utc_s / 86400, &h->utc.year, &h->utc.mon, &h->utc.day);
	h->utc.hour = (uint8_t)((utc_s / 3600) % 24);
	h->utc.min = (uint8_t)((utc_s / 60) % 60);
	h->utc.sec = (uint8_t)(utc_s % 60);
	h->utc.valid = true;
}

static void fill_ident(ui_health_t *h)
{
	uint8_t id[8];
	ssize_t n;
	size_t hlen = 0u;

	/*
	 * Deliberately NOT under sts_cfg_lock(). sts_app.h requires the lock for
	 * every mutation and for "every read that must not see a half-applied
	 * commit"; a hostname drawn on the panel is not one of those. This runs on
	 * the 10 Hz render path from the lowest-priority thread in the system, so
	 * taking the config mutex here would put every other cfg user behind a
	 * render tick for no gain. The worst case is one frame of a torn string,
	 * and it is bounded: cfg_get() copies a whole fixed-size cfg_val_t, and
	 * cfg_get_bytes() rejects any length past the caller's capacity (leaving
	 * hlen 0 and the "meridian" fallback below), so a torn read cannot
	 * over-read.
	 */
	(void)cfg_get_bytes(sts_cfg(), (uint16_t)CFG_ID_NET_HOSTNAME,
			    (uint8_t *)h->hostname, sizeof(h->hostname) - 1u,
			    &hlen);
	if (hlen < sizeof(h->hostname)) {
		h->hostname[hlen] = '\0';
	}
	if (h->hostname[0] == '\0') {
		strcpy(h->hostname, "meridian");
	}

	(void)snprintf(h->fw_version, sizeof(h->fw_version), "%s",
		       APP_VERSION_EXTENDED_STRING);

	n = hwinfo_get_device_id(id, sizeof(id));
	if (n > 0) {
		char *p = h->serial;
		size_t cap = sizeof(h->serial);
		ssize_t i;

		for (i = 0; i < n && (size_t)(2 * i + 1) < cap - 1u; i++) {
			static const char hex[] = "0123456789abcdef";

			p[2 * i] = hex[(id[i] >> 4) & 0xF];
			p[2 * i + 1] = hex[id[i] & 0xF];
		}
		p[2 * i] = '\0';
	} else {
		strcpy(h->serial, "unknown");
	}
}

/** Resolve an alarm id to a display name for the Alarms page. */
static const char *ui_alarm_name(uint16_t id)
{
	static const char *const soft[] = {
		"REFERENCE_LOST", "GNSS_LOST",       "ANTENNA_OPEN",
		"ANTENNA_SHORT",  "OCXO_UNHEALTHY",  "DAC_FAULT",
		"RB_FAULT",       "RB_OV",           "THERMAL_WARN",
		"THERMAL_CRIT",   "FAN_FAULT",       "POE_BUDGET",
		"I2C_WEDGE",      "NOR_FAULT",       "DISPLAY_FAULT",
		"PFI",            "TAMPER",
	};

	if (id < (uint16_t)FAULT_SIG_COUNT) {
		return fault_sig_name((fault_sig_t)id);
	}
	if (id >= (uint16_t)FAULT_ALARM_REFERENCE_LOST &&
	    id < (uint16_t)FAULT_ALARM_COUNT) {
		return soft[id - (uint16_t)FAULT_ALARM_REFERENCE_LOST];
	}
	return "ALARM";
}

static void fill_alarms(ui_health_t *h, uint64_t mask)
{
	uint8_t n = 0u;
	unsigned int bit;

	for (bit = 0u; bit < 64u && n < (uint8_t)UI_MAX_ALARMS; bit++) {
		if ((mask & (UINT64_C(1) << bit)) == 0u) {
			continue;
		}
		h->alarm[n].id = (uint16_t)bit;
		h->alarm[n].active = true;
		h->alarm[n].latched = true; /* it is true right now */
		h->alarm[n].count = 1u;
		h->alarm[n].first_s = h->uptime_s;
		n++;
	}
	h->alarm_count = n;
	h->alarm_name = ui_alarm_name;
}

/**
 * Fill the per-SV markers and the true-north orientation for this frame.
 *
 * Both halves land in the same place because they are one picture: the numeric
 * table on the sky page (h->sv[]) and the polar plot (sky_frame) MUST show the
 * same satellites, or the operator reading "GPS 14, elevation 8" off the table
 * cannot find it on the plot.
 *
 * The selection itself is sts_sky_policy.h's. UBX reports azimuth and elevation
 * as zero for a satellite it knows only from the almanac, so admitting every
 * NAV-SAT record would paint a stack of phantom markers at due north on the
 * horizon — precisely where an operator looks for an obstruction.
 */
static void build_sky(ui_health_t *h)
{
	static sts_gnss_sky_t sky; /* ~280 B; the ui thread's stack is 3 KiB */
	sts_ecompass_t ec;
	sts_sky_orient_in_t in;
	uint64_t now = sts_mono_ms();
	bool sky_fresh;
	uint8_t n = 0u;
	uint8_t i;

	memset(&sky_frame, 0, sizeof(sky_frame));

	if (sts_gnss_sky(&sky) != 0) {
		/* -EBUSY: the GNSS thread is mid-parse. sts_app.h zeroes the
		 * output, and a render tick skips the frame rather than waits. */
		memset(&sky, 0, sizeof(sky));
	}

	/* sts_app.h: mono_ms is when the frame was DECODED and a receiver that
	 * has stopped talking leaves the last good one in place. Age it. */
	sky_fresh = (sky.mono_ms != 0u) && (now >= sky.mono_ms) &&
		    ((now - sky.mono_ms) <= (uint64_t)SKY_STALE_MS);

	if (sky_fresh) {
		for (i = 0u; i < sky.count && i < (uint8_t)STS_GNSS_SKY_MAX_SV &&
			     n < (uint8_t)UI_MAX_SV;
		     i++) {
			const sts_gnss_sv_t *s = &sky.sv[i];

			if (sts_sky_sv_from_ubx(s->gnss_id, s->sv_id,
						s->elev_deg, s->azim_deg,
						s->cno_dbhz, s->used,
						&h->sv[n])) {
				sky_frame.sv[n] = h->sv[n];
				n++;
			}
		}
	}
	h->sv_count = n;
	sky_frame.sv_count = n;

	/* --- orientation ------------------------------------------------- */

	memset(&in, 0, sizeof(in));
	memset(&ec, 0, sizeof(ec));
	(void)sts_ecompass(&ec);

	/*
	 * mono_ms here is k_uptime_get_32()'s width, not sts_mono_ms()'s, so the
	 * comparison is done in 32 bits and wraps correctly with it.
	 */
	in.sample_valid = ec.valid &&
			  (((uint32_t)now - ec.mono_ms) <= SKY_STALE_MS);
	for (i = 0u; i < 3u; i++) {
		in.mag[i] = ec.mag_mgauss[i];
		in.acc[i] = ec.acc_mg[i];
		in.mag_offset[i] = sky_cal.off[i];
		in.mag_scale[i] = sky_cal.scl[i];
	}
	in.cal_valid = sky_cal.valid;
	in.field_ref = sky_cal.ref;
	in.decl_site_valid = sky_cal.decl_valid;
	in.decl_site_ddeg = sky_cal.decl_ddeg;

	/*
	 * Position for the dipole declination fallback comes from the SAME
	 * snapshot as the satellites, and is trusted even when the NAV-SAT frame
	 * behind it has aged out: a fixed-site grandmaster's last known position
	 * does not go stale in five seconds, and a 10-15 degree model correction
	 * is not the thing that will be wrong about a plot whose receiver has
	 * stopped reporting.
	 */
	in.pos_valid = sky.pos_valid;
	in.lat_1e7 = sky.lat_1e7;
	in.lon_1e7 = sky.lon_1e7;

	sky_frame.north_reason =
		sts_sky_orient(&in, &sky_frame.orient, NULL);
}

/**
 * Build the health snapshot core/ui renders from.
 *
 * TODO(ui-health): the fields below the quality-derived block have no typed
 * cross-area source in sts_app.h (per-rail INA, fan/PoE, the network/PTP
 * counters and the Rb detail). They stay zero until a health getter is added to
 * sts_app.h by the platform/net owners. Home/Clocks/Alarms are fully populated
 * from the quality block and the alarm mask, and the Sky page's per-SV
 * az/el/CN0 now comes from sts_gnss_sky() through build_sky().
 */
static void build_health(ui_health_t *h, const quality_block_t *q,
			 uint64_t alarms)
{
	memset(h, 0, sizeof(*h));
	fill_time(h, q);
	fill_ident(h);
	fill_alarms(h, alarms);
	build_sky(h);

	/* Antenna state is not directly exposed; approximate from the GNSS
	 * time-lock flag so the Home/Sky badge is not permanently "UNKNOWN". */
	h->ant_state = ((q->flags & QUALITY_FLAG_GNSS_TIME_LOCKED) != 0u)
			       ? (uint8_t)UI_ANT_OK
			       : (uint8_t)UI_ANT_UNKNOWN;

	/* Reflect the active reference into the Rb/ext view where the quality
	 * block already tells us. */
	if (q->active_ref == (uint8_t)QUALITY_REF_RB) {
		h->rb_present = true;
		h->rb_powered = true;
		h->rb_locked = true;
	} else if (q->active_ref == (uint8_t)QUALITY_REF_EXTREF) {
		h->extref_ok = true;
	}
}

/* -------------------------------------------------------------- panel mirror */

/**
 * Fold the scan queue's drop count into the input echo, once per render tick.
 *
 * Runs before ui_tick()/ui_render() rather than beside mirror_publish(), so the
 * lamp release a lost edge implies is acted on in the *same* frame the mirror
 * reports it in — otherwise the panel LED string would stay lit for one more
 * tick than the mirror says it is.
 *
 * The counter is only ever compared, never trusted as a magnitude, so its 2^32
 * wrap needs no handling (sts_ui_echo.h).
 */
static void echo_reconcile(void)
{
	bool was_lamp = mirror_echo.lamp;

	if (!sts_ui_echo_sync(&mirror_echo,
			      (uint32_t)atomic_get(&ui_inq_drops))) {
		return;
	}

	if (was_lamp) {
		/* The lost edge may have been this button's release. Command the
		 * release rather than only forgetting it: the LAMP level drives
		 * UI_ACTION_LAMP_TEST, and a stranded "held" leaves the whole
		 * panel LED string full-on indefinitely. */
		ui_feed((uint8_t)UI_IN_LAMP, 0, 0u, 0u);
	}

	LOG_WRN("panel input queue overflowed (%u events lost, %u resyncs); "
		"button echo cleared",
		(unsigned int)mirror_echo.drops_seen,
		(unsigned int)mirror_echo.resyncs);
	sts_log((uint8_t)LOGR_SUB_UI, (uint8_t)LOGR_WARN,
		"panel input dropped (%u total); mirror button echo resynced",
		(unsigned int)mirror_echo.drops_seen);
}

/**
 * Publish the just-rendered frame for the Maintenance Protocol panel mirror
 * (channel 0x0A / `mirror.get`).
 *
 * Called from the render tick with the freshly rendered g_surf. Everything here
 * is a copy out of state this thread owns; sts_mp_mirror_publish() deep-copies
 * the cells, the attributes and the hint list before it returns, so the
 * stack-local frame and the pointers into g_surf do not outlive the call. It
 * takes a 5 ms mutex and drops the frame on contention rather than parking the
 * priority-15 ui thread — which is why there is no return code to check and no
 * log line here: a dropped frame at 10 Hz is normal and self-healing (the next
 * frame carries the same rows, and core/mp only records a row in its
 * previous-frame store once it has actually been emitted).
 *
 * FIELDS WITH NO TRUTHFUL SOURCE, left zero rather than guessed
 * -------------------------------------------------------------
 *  - dialog_item / dialog_stage: ui_frame_t::item / ::stage of the top stack
 *    frame. core/ui exposes ui_page()/ui_depth()/ui_selection() but no accessor
 *    for either, and reaching into ui_ctx_t from the glue is not this area's
 *    business. `dialog` itself IS truthful (the top page is EDIT or CONFIRM), so
 *    the host still learns a dialog is up, and the dialog's text is mirrored in
 *    full by the cells.
 *  - led_logical: the UI's per-lamp logical bitmap. There is no such model in
 *    core/ui — the seven panel lamps are one series string on a single PWM
 *    (mp_mirror.h "As-built note on (b)"), so the electrical truth is carried by
 *    panel_rail_on + panel_duty_pct + panel_fault and a fabricated bitmap would
 *    be strictly worse than an empty one.
 *  - rgb_state / rgb_r / rgb_g / rgb_b: the status RGB D5 is driven by the
 *    platform's supervisor (TIM4, PD12-14). Its commanded pattern and per-channel
 *    duties are private to supervisor.c and sts_app.h exposes no getter.
 */
static void mirror_publish(uint64_t alarms)
{
	mp_mirror_in_t f;
	uint8_t page = (uint8_t)ui_page(&g_ui);

	memset(&f, 0, sizeof(f));

	/* (a) screen — straight from the surface core/ui just rendered. */
	f.rows = g_surf.rows;
	f.cols = g_surf.cols;
	f.cell_w = g_surf.cell_w; /* 8, per ui_font8x16.h UI_FONT_W */
	f.cell_h = g_surf.cell_h; /* 16, per UI_FONT_H */
	f.ch = g_surf.ch;
	f.attr = g_surf.attr;
	f.hint = g_surf.hint;
	f.hint_count = g_surf.hint_count;

	f.page = page;
	f.depth = ui_depth(&g_ui);
	f.sel = ui_selection(&g_ui);
	f.dialog = (page == (uint8_t)UI_PAGE_EDIT) ||
		   (page == (uint8_t)UI_PAGE_CONFIRM);

	f.awake = ui_awake(&g_ui);
	f.identify = ui_identify(&g_ui);
	/* The debounced LAMP level this area forwarded to core. It equals
	 * ui_ctx_t::lamp_test except across a wake edge, where core consumes the
	 * waking event; core exposes no ui_lamp_test(), and for a mirror of "what
	 * the operator is doing to the box" the held key is the honest answer. */
	f.lamp_test = mirror_echo.lamp;

	/* (b) indicators. */
	f.panel_duty_pct = sts_panel_led_get();
	/* sts_app.h: duty 0 drops PANEL_LED_EN (PC0) and any non-zero duty asserts
	 * it, so the commanded duty IS the rail state — one writer, no readback. */
	f.panel_rail_on = (f.panel_duty_pct != 0u);
	/*
	 * The unmasked panel-LED fault ALARM, which is deliberately not the same
	 * thing as "PF12 asserted".
	 *
	 * PF12 (U55 RT9742 nFLG) is scanned signal 12 and alarm ids 0..31 mirror
	 * fault_sig_t one-for-one (fault.h), so the alarm mask is this area's only
	 * cross-area view of that pin. But sts_alarms_active() is fault_alarms(),
	 * which drops anything in core/fault's expected-off set, and pwrseq_exec.c
	 * puts FAULT_SIG_PANEL_LED_FAULT there on every PANEL_LED_DIS. With the
	 * rail deliberately gated off this therefore reports false while the pin
	 * is asserted — the useful answer (a rail firmware turned off is not a
	 * fault), and the reason mp_mirror.h documents the field as an alarm.
	 *
	 * @p alarms is the tick's single sts_alarms_active() reading, shared with
	 * build_health() — that call takes the platform's fault mutex, and the
	 * render path takes it once per frame, not twice.
	 */
	f.panel_fault = (alarms &
			 FAULT_ALARM_BIT(FAULT_SIG_PANEL_LED_FAULT)) != 0u;
	f.bl_permille = ui_backlight_permille(&g_ui);

	/* (c) input echo. Reconciled against the scan-queue drop count by the
	 * caller (echo_reconcile()) immediately before this runs, so a lost
	 * release cannot present as a stuck key. */
	f.buttons_down = mirror_echo.down;
	f.enc_pos = mirror_enc_pos;
	f.touch_x = mirror_touch_x;
	f.touch_y = mirror_touch_y;
	f.touch_ms = mirror_touch_ms;

	sts_mp_mirror_publish(&f);
}

/* --------------------------------------------------------------- cfg apply */

/**
 * Pull ui.brightness / ui.timeout.s from cfg into the core ui context.
 *
 * The pair is read under sts_cfg_lock(): these two keys are applied together and
 * this runs on the *committing* thread with the cfg mutex released (sts_app.h),
 * so a second commit racing in between could otherwise hand the panel one key
 * from before it and one from after. sts_app.h scopes read-locking to exactly
 * this case — "every read that must not see a half-applied commit" — and taking
 * the mutex here is free, since an applier only runs on a commit.
 *
 * No deadlock, and not because the mutex is recursive: sts_cfg_commit(),
 * sts_cfg_factory_reset() and sts_cfg_register_store() all release it before
 * they dispatch, precisely so an applier may block on sockets, DNS or display
 * I/O. Taking it here is therefore an ordinary, normally-uncontended acquire.
 */
static void apply_ui_group(void *ctx, uint8_t group)
{
	uint64_t bright = 60u;
	uint64_t timeout = 120u;

	ARG_UNUSED(ctx);
	ARG_UNUSED(group);

	sts_cfg_lock();
	(void)cfg_get_u64(sts_cfg(), (uint16_t)CFG_ID_UI_BRIGHTNESS, &bright);
	(void)cfg_get_u64(sts_cfg(), (uint16_t)CFG_ID_UI_TIMEOUT_S, &timeout);
	sts_cfg_unlock();

	/* The schema allows 0..100 but core/ui floors brightness at 10 % so a
	 * mis-set value cannot black out the field-service panel. */
	if (bright < UI_BRIGHTNESS_MIN_PCT) {
		bright = UI_BRIGHTNESS_MIN_PCT;
	}
	if (bright > UI_BRIGHTNESS_MAX_PCT) {
		bright = UI_BRIGHTNESS_MAX_PCT;
	}
	g_ui.cfg.brightness_pct = (uint8_t)bright;
	g_ui.cfg.timeout_s = (uint16_t)timeout;

	ui_display_backlight_permille(ui_backlight_permille(&g_ui));
	sts_log((uint8_t)LOGR_SUB_UI, (uint8_t)LOGR_INFO,
		"ui config: brightness %u%%, blank %u s", (unsigned int)bright,
		(unsigned int)timeout);
}

/**
 * Pull the §10.4 e-compass fit and the site declination out of cfg group 0x0C.
 *
 * Group 0x0C already has a subscriber (platform/hk.c, for the nine INA228
 * SHUNT_CAL trims). sts_app.h dispatches EVERY subscriber for a group, in
 * registration order, precisely so a shared group does not become first-come:
 * these seven keys belong to the panel and none of them means anything to
 * housekeeping.
 *
 * Read under sts_cfg_lock() for the same reason apply_ui_group() takes it — the
 * keys are a set, and a commit racing in between would hand the plot half of an
 * old calibration and half of a new one.
 */
static void apply_cal_group(void *ctx, uint8_t group)
{
	static const uint16_t off_id[3] = {
		(uint16_t)CFG_ID_CAL_MAG_OFF_X,
		(uint16_t)CFG_ID_CAL_MAG_OFF_Y,
		(uint16_t)CFG_ID_CAL_MAG_OFF_Z,
	};
	static const uint16_t scl_id[3] = {
		(uint16_t)CFG_ID_CAL_MAG_SCL_X,
		(uint16_t)CFG_ID_CAL_MAG_SCL_Y,
		(uint16_t)CFG_ID_CAL_MAG_SCL_Z,
	};
	int32_t off[3] = { 0, 0, 0 };
	int32_t scl[3] = { 4096, 4096, 4096 };
	uint64_t ref = 0u;
	int32_t decl = STS_SKY_DECL_UNSET;
	unsigned int i;

	ARG_UNUSED(ctx);
	ARG_UNUSED(group);

	sts_cfg_lock();
	for (i = 0u; i < 3u; i++) {
		uint64_t s = 4096u;

		(void)cfg_get_i32(sts_cfg(), off_id[i], &off[i]);
		(void)cfg_get_u64(sts_cfg(), scl_id[i], &s);
		scl[i] = (int32_t)s;
	}
	(void)cfg_get_u64(sts_cfg(), (uint16_t)CFG_ID_CAL_MAG_FIELD_REF, &ref);
	(void)cfg_get_i32(sts_cfg(), (uint16_t)CFG_ID_CAL_DECLINATION_DDEG,
			  &decl);
	sts_cfg_unlock();

	/* Cleared first, published last: a render tick that interleaves with
	 * this must never read half a calibration and believe it. */
	sky_cal.valid = false;
	sky_cal.decl_valid = false;

	for (i = 0u; i < 3u; i++) {
		sky_cal.off[i] = off[i];
		sky_cal.scl[i] = scl[i];
	}
	sky_cal.ref = (uint32_t)ref;
	sky_cal.decl_ddeg = (int16_t)decl;

	/*
	 * `cal.mag.ref` doubles as the "the fit has been performed" flag
	 * (cfg_schema.h): zero leaves the plot in GNSS-north behind the "north
	 * unverified" badge, which is what an uncommissioned unit MUST show.
	 */
	sky_cal.decl_valid = (decl != STS_SKY_DECL_UNSET);
	sky_cal.valid = (ref != 0u);

	sts_log((uint8_t)LOGR_SUB_UI, (uint8_t)LOGR_INFO,
		"skyplot orientation: compass fit %s, site declination %s",
		sky_cal.valid ? "loaded" : "absent",
		sky_cal.decl_valid ? "set" : "unset (dipole model)");
}

/* ------------------------------------------------------------------ thread */

static void ui_thread_entry(void *p1, void *p2, void *p3)
{
	int64_t next_render = k_uptime_get();

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		sts_input_evt_t e;
		int64_t now = k_uptime_get();
		int64_t wait = next_render - now;

		if (wait < 0) {
			wait = 0;
		}

		/* Handle input the moment it arrives; otherwise wake to render
		 * at the 10 Hz cadence. */
		if (k_msgq_get(&ui_inq, &e, K_MSEC((uint32_t)wait)) == 0) {
			handle_scan_event(&e);
			continue;
		}

		/* Render tick. */
		next_render += UI_PERIOD_MS;
		{
			quality_block_t q;
			ui_health_t h;
			uint64_t alarms;
			int32_t detents = ui_input_encoder_delta();

			echo_reconcile();

			if (detents != 0) {
				mirror_enc_pos += detents;
				ui_feed((uint8_t)UI_IN_ENCODER,
					(int16_t)detents, 0u, 0u);
			}

			(void)ui_tick(&g_ui, UI_PERIOD_MS);

			if (sts_quality_snapshot(&q) != 0) {
				quality_block_init(&q);
			}
			/* One reading per frame: it takes the platform's fault
			 * mutex, and both the health snapshot and the mirror want
			 * the same answer anyway. */
			alarms = sts_alarms_active();
			build_health(&h, &q, alarms);

			if (ui_render(&g_ui, &q, &h, &g_surf) == 0) {
				/* Mirror the frame the panel is about to show,
				 * before the blit: the surface is what the host
				 * replicates, and a display that is absent or
				 * still powering up must not stop the mirror. */
				mirror_publish(alarms);
			}
			/*
			 * Hand the rasteriser the satellites and the rotation
			 * build_health() just resolved. Unconditional, and not
			 * only when the sky page is up: the console dump renders
			 * from the same publication, and an operator asking for
			 * the plot over USB must not have to walk the panel to
			 * the sky page first.
			 */
			ui_display_sky_set(&sky_frame);
			(void)ui_display_blit(&g_surf);
			ui_display_backlight_permille(
				ui_backlight_permille(&g_ui));
			drain_actions();
		}

		if (ui_liveness_id >= 0) {
			sts_liveness_feed(ui_liveness_id);
		}
	}
}

/* ------------------------------------------------------------------ start */

int sts_ui_start(void)
{
	ui_cfg_t cfg;
	k_tid_t tid;
	int rc;

	/* Seed the ui config from the schema defaults, then bind the context. */
	ui_cfg_default(&cfg);
	{
		uint64_t bright = cfg.brightness_pct;
		uint64_t timeout = cfg.timeout_s;

		/* Same coherency argument as apply_ui_group(): the console and web
		 * planes are already running by the time this area starts. */
		sts_cfg_lock();
		(void)cfg_get_u64(sts_cfg(), (uint16_t)CFG_ID_UI_BRIGHTNESS,
				  &bright);
		(void)cfg_get_u64(sts_cfg(), (uint16_t)CFG_ID_UI_TIMEOUT_S,
				  &timeout);
		sts_cfg_unlock();

		if (bright < UI_BRIGHTNESS_MIN_PCT) {
			bright = UI_BRIGHTNESS_MIN_PCT;
		}
		if (bright > UI_BRIGHTNESS_MAX_PCT) {
			bright = UI_BRIGHTNESS_MAX_PCT;
		}
		cfg.brightness_pct = (uint8_t)bright;
		cfg.timeout_s = (uint16_t)timeout;
	}

	rc = ui_init(&g_ui, &cfg);
	if (rc != 0) {
		LOG_ERR("ui_init failed (%d)", rc);
		return rc;
	}

	rc = ui_surface_init(&g_surf, (uint8_t)STS_UI_ROWS, (uint8_t)STS_UI_COLS,
			     surf_ch, surf_attr, sizeof(surf_ch));
	if (rc != 0) {
		LOG_ERR("ui_surface_init failed (%d)", rc);
		return rc;
	}

	rc = ui_display_init();
	if (rc != 0) {
		LOG_WRN("display init deferred/failed (%d); UI runs headless",
			rc);
	}

	(void)ui_input_encoder_init();

	/* React to later cfg commits touching the UI group (0x08). */
	rc = sts_cfg_register_applier(CFG_G_UI, apply_ui_group, NULL);
	if (rc != 0) {
		LOG_WRN("ui cfg applier not registered (%d)", rc);
	}

	/*
	 * The skyplot's true-north inputs live in the calibration group (0x0C).
	 * Seeded here as well as registered, because an applier only runs on a
	 * commit: without the seeding call a unit that has been commissioned and
	 * then rebooted would draw its first frames in GNSS-north behind the
	 * badge despite holding a perfectly good fit in NVS.
	 */
	rc = sts_cfg_register_applier(CFG_G_CAL, apply_cal_group, NULL);
	if (rc != 0) {
		LOG_WRN("ui cal applier not registered (%d)", rc);
	}
	apply_cal_group(NULL, (uint8_t)CFG_G_CAL);

	ui_liveness_id = sts_liveness_register("ui");
	if (ui_liveness_id < 0) {
		LOG_WRN("no liveness slot for ui (%d)", ui_liveness_id);
	}

	tid = k_thread_create(&ui_thread, ui_stack,
			      K_THREAD_STACK_SIZEOF(ui_stack), ui_thread_entry,
			      NULL, NULL, NULL, UI_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(tid, "ui_local");

	LOG_INF("ui area up: %ux%u tiles, %u Hz render", STS_UI_COLS,
		STS_UI_ROWS, STS_UI_RENDER_HZ);
	return 0;
}

#endif /* CONFIG_STS1000_UI */
