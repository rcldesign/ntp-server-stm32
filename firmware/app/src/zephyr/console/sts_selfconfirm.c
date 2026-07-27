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
 * The gate
 * --------
 *   image is unconfirmed   otherwise there is nothing to do at all;
 *   uptime >= MIN_S        the system has survived long enough for every
 *                          thread to have registered and fed its liveness
 *                          slot at least once (ARCHITECTURE.md §6 lists the
 *                          slowest at 1 Hz, so 60 s is many cycles);
 *   cfg is loaded          sts_cfg() answers, i.e. the platform area finished
 *                          building the configuration tree;
 *   config persists        the NVS backend mounted — an image that cannot
 *                          save its configuration is not a healthy image;
 *   a management path      the USB device reached CONFIGURED, or a network
 *   is alive               interface is up. Confirming an image that answers
 *                          on neither would strand the box with no way in.
 *
 * Two callers reach the same gate. The supervisor calls
 * sts_update_self_confirm() when its own liveness AND-gate is satisfied
 * (sts_app.h: "called by supervisor when healthy"), which is the authoritative
 * path and skips the MIN_S wait. The console thread also polls, so a build
 * whose supervisor never calls still confirms a genuinely healthy image
 * instead of reverting every update.
 *
 * If the gate has still not passed by DEADLINE_S the attempt is abandoned with
 * a CRIT log line. That is deliberately a loud non-event: doing nothing is the
 * safe outcome (MCUboot restores the last known-good image on the next reset),
 * but silence would make a reverting box look like a mystery.
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

#include "console/sts_console.h"
#include "storage/sts_store.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_selfconfirm, CONFIG_STS1000_LOG_LEVEL);

#if defined(CONFIG_STS1000_SELF_CONFIRM_MIN_S)
#define SELF_CONFIRM_MIN_S CONFIG_STS1000_SELF_CONFIRM_MIN_S
#else
#define SELF_CONFIRM_MIN_S 60
#endif

#if defined(CONFIG_STS1000_SELF_CONFIRM_DEADLINE_S)
#define SELF_CONFIRM_DEADLINE_S CONFIG_STS1000_SELF_CONFIRM_DEADLINE_S
#else
#define SELF_CONFIRM_DEADLINE_S 600
#endif

BUILD_ASSERT(SELF_CONFIRM_DEADLINE_S > SELF_CONFIRM_MIN_S,
	     "the self-confirm deadline must leave room after the minimum age");

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

static void gate_eval(sts_selfconfirm_status_t *st)
{
	uint32_t up_s = (uint32_t)(k_uptime_get() / 1000);

	memset(st, 0, sizeof(*st));
	st->uptime_s = up_s;
	st->image_confirmed = boot_is_img_confirmed();
	st->min_age_met = (up_s >= (uint32_t)SELF_CONFIRM_MIN_S);
	st->cfg_loaded = (sts_cfg() != NULL);
	st->store_ready = sts_store_ready();
	st->link_ok = link_alive();
	st->deadline_passed = sc_abandoned;
}

void sts_selfconfirm_status(sts_selfconfirm_status_t *out)
{
	if (out == NULL) {
		return;
	}
	gate_eval(out);
}

bool sts_update_pending_confirm(void)
{
	return !boot_is_img_confirmed();
}

/*
 * @param honour_min_age  false for the supervisor path, which has already
 *                        established liveness and does not need the proxy.
 */
static int self_confirm(bool honour_min_age)
{
	sts_selfconfirm_status_t st;
	int rc;

	k_mutex_lock(&sc_lock, K_FOREVER);

	gate_eval(&st);

	if (st.image_confirmed) {
		sc_done = true;
		rc = 0;
		goto out;
	}
	if (honour_min_age && !st.min_age_met) {
		rc = -EAGAIN;
		goto out;
	}
	if (!st.cfg_loaded || !st.store_ready || !st.link_ok) {
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
		"mounted, management link up)",
		st.uptime_s);
	sts_log((uint8_t)LOGR_SUB_SYS, (uint8_t)LOGR_NOTICE,
		"firmware image self-confirmed at %u s", st.uptime_s);

out:
	k_mutex_unlock(&sc_lock);
	return rc;
}

int sts_update_self_confirm(void)
{
	return self_confirm(false);
}

/* ------------------------------------------------------------------------- */

int sts_selfconfirm_start(void)
{
	if (boot_is_img_confirmed()) {
		sc_done = true;
		LOG_DBG("running a confirmed image; no self-confirm needed");
		return 0;
	}

	LOG_WRN("running an UNCONFIRMED image: it must pass the §8.3 health "
		"gate within %d s or MCUboot reverts on the next reboot",
		(int)SELF_CONFIRM_DEADLINE_S);
	sts_log((uint8_t)LOGR_SUB_SYS, (uint8_t)LOGR_WARN,
		"unconfirmed image: self-confirm deadline %d s",
		(int)SELF_CONFIRM_DEADLINE_S);
	return 0;
}

void sts_selfconfirm_poll(void)
{
	sts_selfconfirm_status_t st;
	uint32_t up_s;

	if (sc_done || sc_abandoned) {
		return;
	}

	if (self_confirm(true) == 0) {
		return;
	}

	up_s = (uint32_t)(k_uptime_get() / 1000);
	if (up_s < (uint32_t)SELF_CONFIRM_DEADLINE_S) {
		/* Report the outstanding conditions occasionally, not every
		 * poll — this runs while an operator is probably watching. */
		if ((sc_reports++ % 12U) == 0U) {
			gate_eval(&st);
			LOG_WRN("self-confirm pending at %u s: age%s cfg%s nvs%s "
				"link%s",
				st.uptime_s, st.min_age_met ? "+" : "-",
				st.cfg_loaded ? "+" : "-",
				st.store_ready ? "+" : "-",
				st.link_ok ? "+" : "-");
		}
		return;
	}

	gate_eval(&st);
	sc_abandoned = true;
	LOG_ERR("self-confirm ABANDONED after %d s (age%s cfg%s nvs%s link%s): "
		"MCUboot will revert to the previous image on the next reboot",
		(int)SELF_CONFIRM_DEADLINE_S, st.min_age_met ? "+" : "-",
		st.cfg_loaded ? "+" : "-", st.store_ready ? "+" : "-",
		st.link_ok ? "+" : "-");
	sts_log((uint8_t)LOGR_SUB_SYS, (uint8_t)LOGR_CRIT,
		"self-confirm abandoned; image will revert on reboot");
}

#endif /* CONFIG_STS1000_CONSOLE */
