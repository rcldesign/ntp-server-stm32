/*
 * STS1000 "Meridian" — console area entry point.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * sts_app.h: "console area: usb, shell, mcp, storage". This file is the
 * ordering and the supervisor loop; every subsystem it starts lives in its own
 * translation unit.
 *
 * Start order, and why it is this one
 * -----------------------------------
 *   1. cfg appliers, then the persistent store. sts_cfg_register_store() runs
 *      cfg_load_all() and then calls every registered applier once, so an
 *      applier registered afterwards would miss the values it exists to react
 *      to.
 *   2. USB. Everything downstream writes to a CDC-ACM endpoint, and attaching
 *      first means the first log lines are visible to a host that is already
 *      plugged in.
 *   3. The logger. It arms the Zephyr LOG bridge, so from here on driver
 *      warnings reach MCP LOG_TAIL and the NOR spool as well as the console.
 *   4. MCP. It needs cfg (auth policy), the log ring, and the DFU port.
 *   5. The Maintenance Protocol. Same dependency set as MCP — cfg, the log
 *      ring, the DFU port — plus the console UART, which it shares with the
 *      shell through shell_set_bypass() and which it refuses to run without
 *      (-ENODEV). Starting it after the USB gate keeps that ordering explicit
 *      even though the CDC-ACM device is ready before usb_enable().
 *   6. Self-confirm. Reporting the unconfirmed-image warning last puts it at
 *      the end of the boot log where an operator will see it.
 *
 * The supervisor thread runs at priority 14 alongside `mcp` and
 * `housekeeping` (ARCHITECTURE.md §6). It owns four slow duties that do not
 * deserve threads of their own: the USB VBUS gate, the MP service tick, the
 * §8.3 self-confirm poll, and the periodic fast-save flush.
 */

#include <zephyr/kernel.h>

#ifdef CONFIG_STS1000_CONSOLE

#include <errno.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "cfg/cfg.h"
#include "console/mp_glue.h"
#include "console/sts_console.h"
#include "console/sts_fwupd_seam_policy.h"
#include "logring/logring.h"
#include "storage/sts_store.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_console, CONFIG_STS1000_LOG_LEVEL);

#if defined(CONFIG_STS1000_CONSOLE_THREAD_STACK)
#define CONSOLE_STACK CONFIG_STS1000_CONSOLE_THREAD_STACK
#else
#define CONSOLE_STACK 2048
#endif

#if defined(CONFIG_STS1000_FASTSAVE_PERIOD_S)
#define FASTSAVE_PERIOD_S CONFIG_STS1000_FASTSAVE_PERIOD_S
#else
#define FASTSAVE_PERIOD_S 60
#endif

/** ARCHITECTURE.md §6: the console supervisor shares priority 14. */
#define CONSOLE_PRIO 14
/** Loop period. Sets the USB VBUS gate's reaction time. */
#define CONSOLE_PERIOD_MS 250
/** Self-confirm gate re-evaluation interval. */
#define SELFCONFIRM_PERIOD_MS 5000

/*
 * The MP dead-man budget, as an inequality the compiler checks.
 *
 * sts_mp_tick() takes the engine lock with a timeout and skips the pass rather
 * than waiting (it shares this loop with sts_liveness_feed(), and a supervisor
 * that blocks past CONFIG_STS1000_LIVENESS_DEADLINE_MS stops kicking the
 * TPS3430 — an unbounded wait here would cold-cycle the board on a slow
 * maintenance login). Skipping means the worst interval between two *successful*
 * ticks is (STS_MP_TICK_MISS_MAX + 1) whole loop passes, each of which can run
 * long by the lock timeout. That interval is what MP_TICK_MAX_MS bounds.
 *
 * THE PREMISE HAS TWO HALVES, and both are load-bearing:
 *
 *   ONE LOCK WAIT PER PASS. sts_mp_tick() has two things that want the engine:
 *   the passthrough drain and the tick proper. That is why sts_mp_stream_raw()
 *   — the drain's only route in — tries with K_NO_WAIT rather than a timeout,
 *   and why sts_fwupd_step() does the same with the orchestrator's mutex. If
 *   either is ever given a timeout instead, this becomes
 *   (5 + 1) * (250 + 50 + 50) + 150 = 2250 ms and the dead-man's revert
 *   deadline is no longer met; change the numbers here in the same edit or do
 *   not make it.
 *
 *   BOUNDED WORK PER PASS. Not waiting on a lock does not make a call short. A
 *   pass runs long if anything in it blocks, sleeps or writes flash, and
 *   sts_fwupd_step() can do the last two: fwupd_step() fires the step and
 *   transfer-idle timeouts, which reach finish() -> t->restore(). Reachable
 *   today, with the allow bitmap as it is: `fw.begin` for the STM32 image
 *   followed by silence puts an internal-flash trailer erase and write on this
 *   thread 60 s later. Not reachable today, but one allow bit away: the GNSS
 *   restore path holds two GPIO pulses with 20 ms of k_msleep. So the budget
 *   carries an explicit work term rather than pretending the pass is free, and
 *   fwupd_glue.c measures the real figure and complains above it. A k_msleep()
 *   inside anything called from this loop is the thing to look for.
 *
 * The work term is added once rather than per pass, because core/fwupd runs
 * restore() exactly once per prepared session; STS_FWUPD_PASS_BUDGET_MS's own
 * comment argues that choice where it can be challenged.
 *
 * As built: (5 + 1) * (250 + 50) + 150 = 1950 <= 2000 ms.
 */
BUILD_ASSERT(STS_FWUPD_PASS_BUDGET_MS(CONSOLE_PERIOD_MS, STS_MP_TICK_LOCK_MS,
				      STS_MP_TICK_MISS_MAX,
				      STS_FWUPD_STEP_BUDGET_MS) <=
		     MP_TICK_MAX_MS,
	     "console period + MP tick-miss budget + the fwupd step's bounded "
	     "work exceeds the override dead-man deadline (MP_TICK_MAX_MS)");

static struct k_thread console_thread;
static K_THREAD_STACK_DEFINE(console_stack, CONSOLE_STACK);

static int console_liveness_id = -1;
static bool console_started;

/* ------------------------------------------------------------------------- */
/* cfg appliers                                                              */
/* ------------------------------------------------------------------------- */

/*
 * Group 0x09 (log). log.level is the ring's global severity floor; the
 * syslog keys belong to the net area's sender and are read there.
 */
static void apply_log_group(void *ctx, uint8_t group)
{
	uint64_t level = (uint64_t)LOGR_INFO;

	ARG_UNUSED(ctx);
	ARG_UNUSED(group);

	if (cfg_get_u64(sts_cfg(), (uint16_t)CFG_ID_LOG_LEVEL, &level) != 0) {
		return;
	}
	if (level >= LOGR_LEVEL_COUNT) {
		level = (uint64_t)LOGR_DEBUG;
	}

	/* 0xFF = every subsystem. Individual subsystems are then raised or
	 * lowered at runtime by MCP LOG_LEVEL. */
	(void)logr_level_set(sts_logring(), 0xFFU, (uint8_t)level);
	LOG_DBG("log ring level set to %u", (unsigned int)level);
}

/*
 * Group 0x0A (security). Nothing needs pushing anywhere: core/mcp reads
 * sec.auth.req, sec.session.s and sec.admin.pw from the registry on every use,
 * and the shell reads sec.console.ro the same way. The applier exists so the
 * change is auditable, which spec §9.4 requires of every mutating action.
 */
static void apply_sec_group(void *ctx, uint8_t group)
{
	bool auth_req = true;
	bool console_ro = true;
	uint64_t session_s = 0U;

	ARG_UNUSED(ctx);
	ARG_UNUSED(group);

	(void)cfg_get_bool(sts_cfg(), (uint16_t)CFG_ID_SEC_AUTH_REQUIRED,
			   &auth_req);
	(void)cfg_get_bool(sts_cfg(), (uint16_t)CFG_ID_SEC_CONSOLE_RO,
			   &console_ro);
	(void)cfg_get_u64(sts_cfg(), (uint16_t)CFG_ID_SEC_SESSION_S,
			  &session_s);

	sts_log((uint8_t)LOGR_SUB_SEC, (uint8_t)LOGR_NOTICE,
		"security policy: auth %s, console %s, session %u s",
		auth_req ? "required" : "open",
		console_ro ? "read-only" : "read-write",
		(unsigned int)session_s);
}

/* ------------------------------------------------------------------------- */
/* Supervisor thread                                                         */
/* ------------------------------------------------------------------------- */

static void console_thread_entry(void *p1, void *p2, void *p3)
{
	uint32_t since_selfconfirm = 0U;
	uint32_t since_fastsave = 0U;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		k_sleep(K_MSEC(CONSOLE_PERIOD_MS));

		sts_usb_poll();

		/*
		 * Unconditionally, every iteration — not behind one of the
		 * accumulators below. mp_glue.h requires this at least every
		 * MP_TICK_MAX_MS (2 s) because it is what expires an override's
		 * dead-man: a tick that is skipped is a lease that outlives its
		 * keepalive. CONSOLE_PERIOD_MS (250) is 8x inside that budget,
		 * and the call is a no-op until sts_mp_start() has run.
		 *
		 * It is also a bounded call: it waits at most
		 * STS_MP_TICK_LOCK_MS for the engine lock and returns having
		 * skipped the pass, so the shell thread cannot delay
		 * sts_liveness_feed() below. The BUILD_ASSERT above is what keeps
		 * the skips inside the dead-man's deadline.
		 */
		sts_mp_tick();

		/*
		 * The orchestrator's pump: what advances a transfer whose
		 * target needs time (an erase, a receiver reboot, a VERIFY)
		 * rather than answering inline, and what fires its timeouts.
		 *
		 * It does NOT spend the pass's one lock wait — sts_fwupd_step()
		 * takes the fwupd mutex with K_NO_WAIT and returns on
		 * contention, because contention here means a request is inside
		 * the orchestrator right now, which is exactly when nothing
		 * needs pumping. So sts_mp_tick() remains the only waiter.
		 *
		 * It can still run long, though: a fired timeout reaches
		 * restore(), which writes internal flash. That is the second
		 * term in the BUILD_ASSERT above, STS_FWUPD_STEP_BUDGET_MS, and
		 * sts_fwupd_step() measures itself against it.
		 */
		(void)sts_fwupd_step();

		since_selfconfirm += CONSOLE_PERIOD_MS;
		if (since_selfconfirm >= SELFCONFIRM_PERIOD_MS) {
			since_selfconfirm = 0U;
			sts_selfconfirm_poll();
		}

		since_fastsave += CONSOLE_PERIOD_MS;
		if (since_fastsave >= ((uint32_t)FASTSAVE_PERIOD_S * 1000U)) {
			since_fastsave = 0U;
			/*
			 * Only writes when something actually changed, and only
			 * from this preemptible thread — an NVS garbage
			 * collection can take tens of milliseconds and must not
			 * land on the cooperative system workqueue except in
			 * the PFI case, where power is going away anyway.
			 */
			(void)sts_store_critical_flush(
				STS_CRITICAL_REASON_PERIODIC);
		}

		if (console_liveness_id >= 0) {
			sts_liveness_feed(console_liveness_id);
		}
	}
}

/* ------------------------------------------------------------------------- */
/* Entry point                                                               */
/* ------------------------------------------------------------------------- */

int sts_console_start(void)
{
	sts_critical_t crit;
	k_tid_t tid;
	int rc;

	if (console_started) {
		return 0;
	}
	console_started = true;

	LOG_INF("console area starting");

	/*
	 * Appliers before the store: sts_cfg_register_store() invokes every
	 * registered applier once with the freshly loaded values.
	 *
	 * Group 0x08 (ui) is deliberately not claimed here even though this
	 * area could read it — sts_cfg_register_applier() is one applier per
	 * group, and the ui area owns the display, backlight and RGB that
	 * those keys drive.
	 */
	rc = sts_cfg_register_applier(CFG_G_LOG, apply_log_group, NULL);
	if (rc != 0) {
		LOG_WRN("log cfg applier rejected (%d)", rc);
	}
	rc = sts_cfg_register_applier(CFG_G_SEC, apply_sec_group, NULL);
	if (rc != 0) {
		LOG_WRN("security cfg applier rejected (%d)", rc);
	}

	rc = sts_cfg_register_store(sts_port_store());
	if (rc != 0) {
		LOG_ERR("configuration will not persist (%d): running on schema "
			"defaults",
			rc);
	} else {
		LOG_INF("configuration loaded from NVS");
	}

	/* Report what the previous power cycle managed to save, before
	 * anything overwrites the shadow. */
	if (sts_store_critical_load(&crit) == 0) {
		LOG_INF("fast-save from the previous run: dac %u, leap %d%s, "
			"log cursor %u (reason %u)",
			crit.dac_code, (int)crit.leap_current,
			crit.leap_valid ? "" : " (unvalidated)",
			(unsigned int)crit.log_cursor, crit.reason);
	}

	(void)sts_usb_start();

	rc = sts_logspool_start();
	if (rc != 0) {
		LOG_ERR("logger thread failed to start (%d)", rc);
	}

	rc = sts_mcp_start();
	if (rc != 0) {
		LOG_ERR("MCP channel failed to start (%d)", rc);
	}

	/*
	 * Before sts_mp_start(), which reads sts_fwupd_mp_port() into its
	 * wiring: the port is a static vtable and would be valid either way, but
	 * an orchestrator that is ready before the plane that drives it is one
	 * fewer ordering question.
	 *
	 * Non-fatal for the same reason as MCP and MP below. A box that cannot
	 * accept a firmware update must still serve time and must still be
	 * manageable — refusing to bring the console up would remove the very
	 * surface an operator would use to work out why.
	 */
	rc = sts_fwupd_init();
	if (rc != 0) {
		LOG_ERR("firmware-update orchestrator failed to start (%d): "
			"`fw.*` and the `fw` shell group will report "
			"unavailable; nothing else is affected",
			rc);
	}

	/*
	 * Non-fatal, like MCP above: MP shares CDC-ACM #0 with the shell, so if
	 * its engine will not initialise the shell and MCP are still there — and
	 * they are the recovery path. Aborting console bring-up here would take
	 * the recovery path away to punish the loss of a maintenance surface.
	 */
	rc = sts_mp_start();
	if (rc != 0) {
		LOG_ERR("Maintenance Protocol failed to start (%d): `mp enter` "
			"will refuse; shell and MCP unaffected",
			rc);
	}

	(void)sts_selfconfirm_start();
	sts_shell_announce();

	console_liveness_id = sts_liveness_register("console");
	if (console_liveness_id < 0) {
		LOG_WRN("no liveness slot for the console supervisor (%d)",
			console_liveness_id);
	}

	tid = k_thread_create(&console_thread, console_stack,
			      K_THREAD_STACK_SIZEOF(console_stack),
			      console_thread_entry, NULL, NULL, NULL,
			      CONSOLE_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(tid, "console");

	LOG_INF("console area up: shell on CDC-ACM #0, MCP on CDC-ACM #1");
	return 0;
}

#endif /* CONFIG_STS1000_CONSOLE */
