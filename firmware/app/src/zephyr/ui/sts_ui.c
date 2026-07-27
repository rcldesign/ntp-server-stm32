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
 *   - it drains the core/ui action queue: brightness/timeout edits are pushed
 *     back into cfg (and persisted), the lamp test drives the panel LED string,
 *     and reboot / factory-reset are executed here.
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
#include <zephyr/sys/reboot.h>

#include "cfg/cfg.h"
#include "fault/fault.h"
#include "logring/logring.h"
#include "quality/quality.h"
#include "ui/ui.h"
#include "zephyr/sts_app.h"
#include "zephyr/ui/sts_ui.h"

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

/* Input queue fed by the io_scan thread through sts_ui_post_input(). Deep
 * enough to absorb a burst between two render ticks; drop-on-full so the
 * non-blocking scan thread is never delayed. */
#define UI_INQ_DEPTH 16
K_MSGQ_DEFINE(ui_inq, sizeof(sts_input_evt_t), UI_INQ_DEPTH, 4);

static struct k_thread ui_thread;
static K_THREAD_STACK_DEFINE(ui_stack, STS_UI_RENDER_STACK);

static int ui_liveness_id = -1;
static bool lamp_on;

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

		if (e->id == FAULT_SIG_BUTTON_6) {
			/* LAMP is level: press = held, release = released. */
			lamp_on = (e->value != 0);
			ui_feed((uint8_t)UI_IN_LAMP, lamp_on ? 1 : 0, 0u, 0u);
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
	/* Drop-on-full: a stuck button must never delay the 1 kHz scan. */
	(void)k_msgq_put(&ui_inq, evt, K_NO_WAIT);
}

/* ----------------------------------------------------------- action drain */

/** Persist one numeric UI cfg key and commit (runs the appliers). */
static void ui_cfg_persist(uint16_t id, uint64_t val)
{
	cfg_ctx_t *c = sts_cfg();
	int rc;

	if (c == NULL) {
		return;
	}
	rc = cfg_set_u64(c, id, val);
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
	case UI_ACTION_FACTORY_RESET:
		sts_log((uint8_t)LOGR_SUB_UI, (uint8_t)LOGR_WARN,
			"operator requested factory reset from the panel");
		if (sts_cfg() != NULL) {
			(void)cfg_factory_reset(sts_cfg());
		}
		(void)sts_panel_led_set(0u);
		k_sleep(K_MSEC(50));
		sys_reboot(SYS_REBOOT_COLD);
		break;
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

static void fill_alarms(ui_health_t *h)
{
	uint64_t mask = sts_alarms_active();
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
 * Build the health snapshot core/ui renders from.
 *
 * TODO(ui-health): the fields below the quality-derived block have no typed
 * cross-area source in sts_app.h (per-rail INA, fan/PoE, per-SV az/el/CN0, the
 * network/PTP counters and the Rb detail). They stay zero until a health getter
 * is added to sts_app.h by the platform/net owners. The Sky page already falls
 * back to the quality SV counts, and Home/Clocks/Alarms are fully populated
 * from the quality block and the alarm mask.
 */
static void build_health(ui_health_t *h, const quality_block_t *q)
{
	memset(h, 0, sizeof(*h));
	fill_time(h, q);
	fill_ident(h);
	fill_alarms(h);

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

/* --------------------------------------------------------------- cfg apply */

/** Pull ui.brightness / ui.timeout.s from cfg into the core ui context. */
static void apply_ui_group(void *ctx, uint8_t group)
{
	uint64_t bright = 60u;
	uint64_t timeout = 120u;

	ARG_UNUSED(ctx);
	ARG_UNUSED(group);

	(void)cfg_get_u64(sts_cfg(), (uint16_t)CFG_ID_UI_BRIGHTNESS, &bright);
	(void)cfg_get_u64(sts_cfg(), (uint16_t)CFG_ID_UI_TIMEOUT_S, &timeout);

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
			int32_t detents = ui_input_encoder_delta();

			if (detents != 0) {
				ui_feed((uint8_t)UI_IN_ENCODER,
					(int16_t)detents, 0u, 0u);
			}

			(void)ui_tick(&g_ui, UI_PERIOD_MS);

			if (sts_quality_snapshot(&q) != 0) {
				quality_block_init(&q);
			}
			build_health(&h, &q);

			(void)ui_render(&g_ui, &q, &h, &g_surf);
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

		(void)cfg_get_u64(sts_cfg(), (uint16_t)CFG_ID_UI_BRIGHTNESS,
				  &bright);
		(void)cfg_get_u64(sts_cfg(), (uint16_t)CFG_ID_UI_TIMEOUT_S,
				  &timeout);
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
