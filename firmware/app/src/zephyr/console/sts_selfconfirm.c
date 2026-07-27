/*
 * STS1000 "Meridian" — spec §8.3 / §8.2 test-image self-confirmation.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * A freshly-staged image boots in MCUboot's TEST mode: unless the application
 * calls boot_write_img_confirmed() before the next reset, MCUboot swaps the
 * previous image back. Spec §8.2 states the rule — "the app must self-confirm
 * after passing post-update health (network up, clock disciplining, services
 * answering) within a watchdog window, else MCUboot reverts."
 *
 * The gate itself is pure arithmetic in sts_confirm_gate.h, which is where the
 * reasoning about each term lives and where the host tests pin it down. This file
 * is the Zephyr side: sample the observations, apply the gate, own the deadline.
 *
 * ONE confirming caller, and it is the supervisor
 * ----------------------------------------------
 * sts_update_self_confirm() (sts_app.h) is called by the platform supervisor once
 * its own liveness AND-gate is clean, no fault is active and the clock is locked.
 * It is the only path that may write the confirmation flag.
 *
 * The console supervisor's 5 s sts_selfconfirm_poll() is a REPORTER. It does not
 * confirm, and it must not. It used to be a second, much weaker gate — uptime
 * >= 60 s plus "cfg loaded, NVS mounted, USB or an interface up" — running in
 * parallel with the strict one, and because 60 s is far below the time needed to
 * declare a lock (tim.lock.hold alone defaults to 300 s), the weak gate always
 * won the race. USB is by definition CONFIGURED during a DFU session, so an image
 * with a dead GNSS UART, a dead PPS capture or a dead DAC path confirmed itself a
 * minute after boot and the automatic revert — the only unattended protection a
 * remote grandmaster has — never fired. The poll's job now is to say why the gate
 * is still open, and to say loudly when the window closes.
 *
 * Consequence worth stating plainly: an update applied while the antenna is
 * disconnected will NOT self-confirm and will revert on the next reboot. That is
 * the intent of §8.2, not a regression. The deliberate override is
 * `sts fw confirm` on the local shell — a human decision, made with the box in
 * front of them.
 */

#include <zephyr/kernel.h>

#ifdef CONFIG_STS1000_CONSOLE

#include <errno.h>
#include <string.h>

#include <zephyr/dfu/mcuboot.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_NETWORKING)
#include <zephyr/net/net_if.h>
#endif

#include "cfg/cfg.h"
#include "console/sts_confirm_gate.h"
#include "console/sts_console.h"
#include "quality/quality.h"
#include "storage/sts_store.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_selfconfirm, CONFIG_STS1000_LOG_LEVEL);

#if defined(CONFIG_STS1000_SELF_CONFIRM_MIN_S)
#define SELF_CONFIRM_MIN_S CONFIG_STS1000_SELF_CONFIRM_MIN_S
#else
#define SELF_CONFIRM_MIN_S 60
#endif

#if defined(CONFIG_STS1000_SELF_CONFIRM_DEADLINE_S)
#define SELF_CONFIRM_DEADLINE_FLOOR_S CONFIG_STS1000_SELF_CONFIRM_DEADLINE_S
#else
#define SELF_CONFIRM_DEADLINE_FLOOR_S 600
#endif

#if defined(CONFIG_STS1000_SELF_CONFIRM_LOCK_MARGIN_S)
#define SELF_CONFIRM_LOCK_MARGIN_S CONFIG_STS1000_SELF_CONFIRM_LOCK_MARGIN_S
#else
#define SELF_CONFIRM_LOCK_MARGIN_S 1200
#endif

BUILD_ASSERT(SELF_CONFIRM_DEADLINE_FLOOR_S > SELF_CONFIRM_MIN_S,
	     "the self-confirm deadline floor must leave room after the minimum age");

/** Fallback tim.lock.hold when cfg cannot be read; the schema default. */
#define SELF_CONFIRM_LOCK_HOLD_FALLBACK_S 300U

static K_MUTEX_DEFINE(sc_lock);
static bool sc_done;      /* confirmed, or was already confirmed at boot */
static bool sc_abandoned; /* deadline passed without the gate closing */
static uint32_t sc_reports;

/* ------------------------------------------------------------------------- */

static bool link_alive(void)
{
	if (sts_usb_configured()) {
		return true;
	}

#if defined(CONFIG_NETWORKING)
	{
		struct net_if *iface = net_if_get_default();

		if ((iface != NULL) && net_if_is_up(iface)) {
			return true;
		}
	}
#endif

	return false;
}

/*
 * tim.lock.hold, read live: an operator who lengthens the hold window has
 * lengthened the time to a legitimate first lock, and the deadline has to follow
 * or it would abandon them. Read under the cfg lock like every other access to
 * the shared tree (sts_app.h).
 */
static uint32_t lock_hold_s(void)
{
	uint64_t hold = SELF_CONFIRM_LOCK_HOLD_FALLBACK_S;
	cfg_ctx_t *c = sts_cfg();

	if (c == NULL) {
		return SELF_CONFIRM_LOCK_HOLD_FALLBACK_S;
	}

	sts_cfg_lock();
	if (cfg_get_u64(c, (uint16_t)CFG_ID_TIM_LOCK_HOLD_S, &hold) != 0) {
		hold = SELF_CONFIRM_LOCK_HOLD_FALLBACK_S;
	}
	sts_cfg_unlock();

	if (hold > (uint64_t)UINT32_MAX) {
		hold = (uint64_t)UINT32_MAX;
	}
	return (uint32_t)hold;
}

static void gate_sample(sts_confirm_gate_t *g)
{
	quality_block_t q;

	memset(g, 0, sizeof(*g));

	g->uptime_s = (uint32_t)(k_uptime_get() / 1000);
	g->min_age_s = (uint32_t)SELF_CONFIRM_MIN_S;
	g->deadline_s = sts_confirm_deadline_s(
		(uint32_t)SELF_CONFIRM_MIN_S, lock_hold_s(),
		(uint32_t)SELF_CONFIRM_LOCK_MARGIN_S,
		(uint32_t)SELF_CONFIRM_DEADLINE_FLOOR_S);

	g->image_confirmed = boot_is_img_confirmed();
	g->cfg_loaded = (sts_cfg() != NULL);
	g->store_ready = sts_store_ready();
	g->link_ok = link_alive();

	if (sts_quality_snapshot(&q) == 0) {
		g->clock_locked = (q.lock_state == (uint8_t)QUALITY_LOCK_LOCKED);
		g->serving_primary = (q.stratum == QUALITY_STRATUM_PRIMARY);
	}
}

void sts_selfconfirm_status(sts_selfconfirm_status_t *out)
{
	sts_confirm_gate_t g;

	if (out == NULL) {
		return;
	}

	gate_sample(&g);

	memset(out, 0, sizeof(*out));
	out->image_confirmed = g.image_confirmed;
	out->min_age_met = sts_confirm_min_age_met(&g);
	out->cfg_loaded = g.cfg_loaded;
	out->store_ready = g.store_ready;
	out->link_ok = g.link_ok;
	out->clock_locked = g.clock_locked;
	out->serving_primary = g.serving_primary;
	out->deadline_passed = sc_abandoned;
	out->uptime_s = g.uptime_s;
	out->deadline_s = g.deadline_s;
}

bool sts_update_pending_confirm(void)
{
	return !boot_is_img_confirmed();
}

/* ------------------------------------------------------------------------- */

/*
 * The one confirming path. The gate below re-establishes the health terms
 * independently of the supervisor's own checks, so a supervisor that is ever
 * changed to call this more eagerly still cannot confirm an undisciplined image.
 */
int sts_update_self_confirm(void)
{
	sts_confirm_gate_t g;
	int rc;

	k_mutex_lock(&sc_lock, K_FOREVER);

	gate_sample(&g);

	if (g.image_confirmed) {
		sc_done = true;
		rc = 0;
		goto out;
	}
	if (!sts_confirm_gate_pass(&g)) {
		rc = -EAGAIN;
		goto out;
	}

	rc = boot_write_img_confirmed();
	if (rc != 0) {
		LOG_ERR("self-confirm: boot_write_img_confirmed failed (%d)", rc);
		goto out;
	}

	sc_done = true;
	LOG_INF("self-confirm: image confirmed at %u s uptime (cfg loaded, NVS "
		"mounted, management link up, clock LOCKED and serving stratum 1)",
		g.uptime_s);
	sts_log((uint8_t)LOGR_SUB_SYS, (uint8_t)LOGR_NOTICE,
		"firmware image self-confirmed at %u s", g.uptime_s);

out:
	k_mutex_unlock(&sc_lock);
	return rc;
}

/* ------------------------------------------------------------------------- */

int sts_selfconfirm_start(void)
{
	sts_confirm_gate_t g;

	if (boot_is_img_confirmed()) {
		sc_done = true;
		LOG_DBG("running a confirmed image; no self-confirm needed");
		return 0;
	}

	gate_sample(&g);

	LOG_WRN("running an UNCONFIRMED image: it must pass the §8.3 health gate "
		"(cfg, NVS, management link, clock LOCKED and serving stratum 1) "
		"within %u s or MCUboot reverts on the next reboot",
		(unsigned int)g.deadline_s);
	sts_log((uint8_t)LOGR_SUB_SYS, (uint8_t)LOGR_WARN,
		"unconfirmed image: self-confirm deadline %u s",
		(unsigned int)g.deadline_s);
	return 0;
}

void sts_selfconfirm_poll(void)
{
	sts_confirm_gate_t g;

	if (sc_done || sc_abandoned) {
		return;
	}

	gate_sample(&g);

	/*
	 * Reporter only — deliberately no boot_write_img_confirmed() here. The
	 * supervisor owns confirmation; see the file header for why a second,
	 * weaker gate is worse than no second gate at all.
	 */
	if (g.image_confirmed) {
		sc_done = true;
		return;
	}

	if (!sts_confirm_deadline_passed(&g)) {
		/* Report the outstanding conditions occasionally, not every
		 * poll — this runs while an operator is probably watching. */
		if ((sc_reports++ % 12U) == 0U) {
			LOG_WRN("self-confirm pending at %u/%u s: age%s cfg%s "
				"nvs%s link%s lock%s strat1%s",
				(unsigned int)g.uptime_s,
				(unsigned int)g.deadline_s,
				sts_confirm_min_age_met(&g) ? "+" : "-",
				g.cfg_loaded ? "+" : "-",
				g.store_ready ? "+" : "-",
				g.link_ok ? "+" : "-",
				g.clock_locked ? "+" : "-",
				g.serving_primary ? "+" : "-");
		}
		return;
	}

	sc_abandoned = true;
	LOG_ERR("self-confirm ABANDONED after %u s (age%s cfg%s nvs%s link%s "
		"lock%s strat1%s): MCUboot will revert to the previous image on "
		"the next reboot",
		(unsigned int)g.deadline_s,
		sts_confirm_min_age_met(&g) ? "+" : "-",
		g.cfg_loaded ? "+" : "-", g.store_ready ? "+" : "-",
		g.link_ok ? "+" : "-", g.clock_locked ? "+" : "-",
		g.serving_primary ? "+" : "-");
	sts_log((uint8_t)LOGR_SUB_SYS, (uint8_t)LOGR_CRIT,
		"self-confirm abandoned; image will revert on reboot");
}

#endif /* CONFIG_STS1000_CONSOLE */
