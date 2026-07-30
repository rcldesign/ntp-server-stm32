/*
 * STS1000 "Meridian" — HTTPS management plane: listener, workers, providers.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * ---------------------------------------------------------------------------
 * Shape
 * ---------------------------------------------------------------------------
 * All the policy lives in core/web (routing, authentication, roles, CSRF, audit,
 * JSON). This file is transport and wiring only:
 *
 *   - a TLS 1.3 listening socket on 443 through Zephyr's TLS sockets, bound to
 *     the credential sts_cert.c registered;
 *   - STS_WEB_WORKERS identical threads at priority 12, each polling the
 *     listener(s) and running one connection to completion;
 *   - an optional plain-HTTP listener on 80 that answers nothing but a 301 to
 *     https:// (and, when ACME is ever compiled in, the HTTP-01 challenge);
 *   - the rest_providers_t bindings, which are thin adapters over
 *     sts_app.h / net area snapshots;
 *   - the firmware-upload bridge (see "DFU" below).
 *
 * Why Zephyr TLS sockets here, when sts_ntske.c drives mbedTLS directly:
 * NTS-KE needs the RFC 8446 exporter, which the socket layer does not expose.
 * HTTPS needs no exporter, so the socket layer is the right level — it gets
 * session handling, the credential store and the record layer for free.
 *
 * ---------------------------------------------------------------------------
 * Why the area starts itself
 * ---------------------------------------------------------------------------
 * sts_net_start() belongs to another author and is not edited by this wave, so
 * the workers are static threads that wait for what they need (a cfg context and
 * an interface) before binding. See sts_web.h.
 *
 * ---------------------------------------------------------------------------
 * DFU: the web upload drives core/mcp's engine, it does not reimplement it
 * ---------------------------------------------------------------------------
 * ARCHITECTURE.md §7 already contains a resumable, idempotent, progressively
 * erasing, SHA-256-verifying staging engine in core/mcp, exercised by
 * tests/host/test_mcp_dfu.c against a RAM slot with NOR write semantics. Writing
 * a second one for the web would double the ways an image can be staged wrong.
 *
 * So the web upload is an MCP *client*: this file owns a private mcp_ctx_t wired
 * to the same port_image_t, and each REST call is turned into a real FW_BEGIN /
 * FW_DATA / FW_END / FW_CONFIRM / FW_REVERT frame fed through mcp_input(), with
 * the response captured from the tx callback. Byte-identical state machine,
 * byte-identical verification, one implementation. The cost is a COBS encode and
 * a CRC per 1 KB chunk on a path that is not timing-critical.
 *
 * That private engine is NOT externally reachable: it has no cfg registry, so it
 * neither holds nor checks a credential. Authentication and the role check happen
 * in core/web before a frame is ever synthesised.
 *
 * Threading: every REST/DFU/auth call is made under `api_lock`, so core/cfg,
 * core/web's auth registry and the DFU engine — none of which are internally
 * locked — see one caller at a time. ARCHITECTURE.md §10 invariant 10 is
 * respected: the timing state is only ever read through sts_quality_snapshot().
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/app_version.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/tls_credentials.h>
#include <zephyr/sys/fdtable.h>
#include <zephyr/toolchain.h>

#include "cfg/cfg.h"
#include "fault/fault.h"
#include "ina228/ina228.h"
#include "mcp/mcp.h"
#include "mcp/mcp_wire.h"
#include "net/sts_aaa.h"
#include "net/sts_net.h"
#include "net/sts_secops.h"
#include "net/sts_web.h"
#include "net/sts_web_sky.h"
#include "storage/sts_store.h"
#include "util/cobs.h"
#include "web/auth_web.h"
#include "web/http_parse.h"
#include "web/rest.h"
#include "web/web.h"
#include "web/wss.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_web, CONFIG_STS1000_LOG_LEVEL);

/*
 * The console area owns the staging-slot port (src/zephyr/console/sts_dfu.c) and
 * its header is private to that area, so the one symbol needed is declared here
 * as a WEAK reference instead of reaching across the boundary. With
 * CONFIG_STS1000_CONSOLE=n the symbol resolves to NULL, the DFU providers are
 * left unbound, and the firmware routes answer 501 — which is the truth for an
 * image with no image manager.
 */
extern const port_image_t *sts_dfu_port(void) __attribute__((weak));

/* ========================================================================= */
/* state                                                                     */
/* ========================================================================= */

static struct k_mutex api_lock;

static auth_web_ctx_t auth_ctx;
static auth_kdf_t     auth_kdf;
static rest_ctx_t     rest_ctx;
static rest_providers_t providers;

static int listen_tls = -1;
static int listen_plain = -1;

static struct k_spinlock st_lock;
static sts_web_stats_t st;

static int live_id = -1;

/* Per-worker scratch. Static, one set per worker: no allocation anywhere. */
struct worker {
	uint8_t rx[STS_WEB_RX_SIZE];
	char    resp[STS_WEB_RESP_SIZE];
	uint8_t ws_msg[STS_WEB_WS_MSG_SIZE];
	uint8_t frame[WSS_HDR_MAX + WSS_CONTROL_MAX];
	wss_txq_t txq;
};

static struct worker workers[STS_WEB_WORKERS];
static K_THREAD_STACK_ARRAY_DEFINE(worker_stacks, STS_WEB_WORKERS,
				   STS_WEB_STACK_SIZE);
static struct k_thread worker_threads[STS_WEB_WORKERS];

static void stat_add(uint32_t *field, uint32_t n)
{
	K_SPINLOCK(&st_lock) {
		*field += n;
	}
}

void sts_web_stats(sts_web_stats_t *out)
{
	if (out == NULL) {
		return;
	}
	K_SPINLOCK(&st_lock) {
		*out = st;
	}
}

bool sts_web_running(void)
{
	return listen_tls >= 0;
}

/* ========================================================================= */
/* providers                                                                 */
/* ========================================================================= */

static int pv_quality(void *u, quality_block_t *out)
{
	ARG_UNUSED(u);
	return sts_quality_snapshot(out);
}

static int pv_health(void *u, rest_health_t *out)
{
	sts_health_t h;
	size_t i;

	ARG_UNUSED(u);
	if (sts_health_snapshot(&h) != 0) {
		return -EIO;
	}
	memset(out, 0, sizeof(*out));
	out->n_rails = (uint8_t)((h.ina_count < REST_RAIL_MAX) ? h.ina_count
							       : REST_RAIL_MAX);
	for (i = 0U; i < out->n_rails; i++) {
		const ina228_rail_info_t *ri = ina228_rail((ina228_rail_t)i);

		out->rail[i].name = (ri != NULL) ? ri->name : "";
		out->rail[i].designator = (ri != NULL) ? ri->designator : "";
		out->rail[i].shunt_ref = (ri != NULL) ? ri->shunt_ref : "";
		out->rail[i].addr = (ri != NULL) ? ri->addr : 0U;
		out->rail[i].nominal_mv = (ri != NULL) ? ri->nominal_mv : 0;
		out->rail[i].design_max_ma = (ri != NULL) ? ri->design_max_ma : 0U;
		out->rail[i].bus_mv = h.ina[i].bus_mv;
		out->rail[i].current_ua = h.ina[i].current_ua;
		out->rail[i].power_uw = h.ina[i].power_uw;
		out->rail[i].diag_alrt = h.ina[i].diag_alrt;
		out->rail[i].valid = h.ina[i].valid;
	}
	out->tmp_osc_mc = h.tmp_osc_mc;
	out->tmp_osc_valid = h.tmp_osc_valid;
	out->tmp_amb_mc = h.tmp_amb_mc;
	out->tmp_amb_valid = h.tmp_amb_valid;
	out->die_mc = h.die_mc;
	out->die_valid = h.die_valid;
	out->humidity_mpct = h.humidity_mpct;
	out->humidity_valid = h.humidity_valid;
	out->fan_rpm = h.fan_rpm;
	out->fan_duty_pct = h.fan_duty_pct;
	out->poe_class = h.poe_class;
	out->poe_draw_mw = h.poe_draw_mw;
	out->poe_budget_mw = h.poe_budget_mw;
	out->bkp_stm_pg = h.bkp_stm_pg;
	out->bkp_gps_pg = h.bkp_gps_pg;
	out->age_ms = (uint32_t)(sts_mono_ms() - (uint64_t)h.mono_ms);
	return 0;
}

/*
 * GNSS detail: the §3.8 quality block, plus the per-satellite frame the GNSS
 * thread caches for the skyplot.
 *
 * WHAT FEEDS THE SKYPLOT, AND WHY IT IS THIS AND NOT THE QUALITY BLOCK.
 * core/gnssmgr reduces UBX-NAV-SAT to counts on purpose, so the quality block
 * has `gnss_sv_used`/`gnss_sv_visible` and no records. The records live in
 * platform/gnss.c's sky cache, published through sts_gnss_sky(), and that is
 * the SAME cache the front panel's build_sky() renders from — which is what
 * spec §336 ("the skyplot mirrors the local-UI renderer") and §371 ("identical
 * data feeds the web skyplot so both views agree") require. Not a second
 * source, not a second decoder.
 *
 * The three decisions on top of that cache — is the frame fresh, which records
 * are worth serving, and what `detail_available` means — are net/sts_web_sky.h's
 * and are host-tested there, including an assertion that the admission rule
 * still matches the panel's. Read that header before changing anything here;
 * in particular the staleness window is the net area's own number and must not
 * be re-pointed at the ui area's.
 *
 * SURVEY-IN, STORED POSITION AND THE ANTENNA come from sts_gnss_detail(), the
 * platform area's once-a-second publication of gnssmgr_svin() /
 * gnssmgr_position() / gnssmgr_rf(). A gnssmgr_t belongs to the GNSS thread and
 * is not internally synchronised, so those getters may not be called from here;
 * the published snapshot is the only legal read. -ENODEV (no GNSS thread) is
 * not an error for this document — the survey reads idle, the position invalid,
 * and the antenna falls back to what the alarm mask alone can say.
 *
 * LOCKING. sts_gnss_sky() takes the sky cache's own 5 ms-bounded mutex and
 * sts_gnss_detail() takes the snapshot mutex; NEITHER is a timing lock —
 * ARCHITECTURE.md §10 invariant 10 is about the discipline loop's state, still
 * reached only through sts_quality_snapshot(). Both are struct copies held for
 * microseconds by threads that inherit this worker's priority. On -EBUSY from
 * the sky cache the output is zeroed and this second's document reports no
 * detail rather than waiting on a receiver mid-parse.
 */
static int pv_gnss(void *u, rest_gnss_t *out)
{
	quality_block_t q;
	sts_gnss_sky_t sky;
	sts_gnss_detail_t detail;
	bool have_detail;

	ARG_UNUSED(u);
	if (sts_quality_snapshot(&q) != 0) {
		return -EIO;
	}
	memset(out, 0, sizeof(*out));
	out->fix_type = q.gnss_fix;
	out->sv_used = q.gnss_sv_used;
	out->sv_visible = q.gnss_sv_visible;
	out->tacc_ns = q.gnss_tacc_ns;
	out->utc_valid = q.utc_valid;
	out->leap_current_s = q.leap_current_s;
	out->leap_pending = q.leap_pending;
	out->ant_bias_on = sts_net_cfg_bool(CFG_ID_GNSS_ANT_BIAS_EN, true);

	/*
	 * Stack, not static: the worker stacks are already reserved
	 * (STS_WEB_WORKERS x STS_WEB_STACK_SIZE), every provider runs under
	 * api_lock so at most one worker is ever in here, and a static would
	 * add the same bytes to BSS for no gain. pv_health() builds a
	 * comparably sized sts_health_t the same way. This is nowhere near the
	 * 1 KB+ that made pv_fw_data()'s chunk buffer static.
	 */
	have_detail = (sts_gnss_detail(&detail) == 0);
	sts_web_sky_survey(have_detail ? &detail : NULL, out);
	out->ant_state = sts_web_ant_state(sts_alarms_active(), q.flags,
					   have_detail && detail.ant_short_latched,
					   out->ant_bias_on);

	if (sts_gnss_sky(&sky) != 0) {
		/* -EBUSY. sts_app.h zeroes the output; belt and braces, because
		 * a partially written snapshot must not reach a client. */
		memset(&sky, 0, sizeof(sky));
	}
	sts_web_sky_fill(&sky, sts_mono_ms(), out);
	return 0;
}

static int pv_net(void *u, rest_net_t *out)
{
	sts_net_link_t link;
	sts_ptpclk_stats_t cs;
	const uint8_t *mac;

	ARG_UNUSED(u);
	memset(out, 0, sizeof(*out));
	sts_net_link_get(&link);
	sts_ptpclk_stats(&cs);
	mac = sts_net_mac();

	out->link_up = link.link_up;
	out->ipv4_ok = link.ipv4_ok;
	out->ipv6_ok = link.ipv6_ok;
	out->dhcp_bound = link.dhcp_bound;
	out->ipv4_addr = link.ipv4_addr;
	out->ipv4_mask = link.ipv4_mask;
	out->ipv4_gw = link.ipv4_gw;
	out->link_changes = link.link_changes;
	if (mac != NULL) {
		memcpy(out->mac, mac, sizeof(out->mac));
	}
	(void)sts_net_cfg_str(CFG_ID_NET_HOSTNAME, out->hostname,
			      sizeof(out->hostname));
	out->ptp_clock_ok = cs.clock_ok;
	out->ptp_clock_synced = cs.synced;
	out->ptp_clock_off_ns = cs.last_off_ns;
	out->ptp_clock_rate_ppb = cs.rate_ppb;
	return 0;
}

static int pv_ptp(void *u, rest_ptp_t *out)
{
	sts_ptp_stats_t ps;
	size_t i;

	ARG_UNUSED(u);
	memset(out, 0, sizeof(*out));
	sts_ptp_stats(&ps);
	out->running = ps.running;
	out->port_state = ps.port_state;
	out->clock_class = ps.clock_class;
	out->clock_accuracy = ps.clock_accuracy;
	out->domain = ps.domain;
	out->transport = ps.transport;
	out->alarms = ps.alarms;
	/*
	 * The profile conformance disclosure. Both strings are core/ptp
	 * constants with static storage duration, so handing the pointers to the
	 * encoder is safe past the end of this call and needs no copy.
	 */
	out->profile = ps.profile;
	out->deviations = ptp_profile_desc(ps.profile)->deviations;
	out->profile_name = ptp_profile_name(ps.profile);
	out->deviation_text = ptp_profile_deviation_text(ps.profile);
	for (i = 0U; i < PTP_MSG_TYPE_COUNT; i++) {
		out->tx_total += ps.counters.tx[i];
		out->rx_total += ps.counters.rx[i];
	}
	out->announce_timeouts = ps.counters.announce_timeouts;
	out->followup_missed = ps.counters.followup_missed;
	return 0;
}

static int pv_services(void *u, rest_services_t *out)
{
	sts_ntp_stats_t ns;
	sts_ntske_stats_t ks;
	snmp_stats_t ss;
	snmp_v3_stats_t vs;
	sts_ptp_stats_t ps;
	sts_web_stats_t ws;

	ARG_UNUSED(u);
	memset(out, 0, sizeof(*out));
	sts_ntp_stats(&ns);
	sts_ntske_stats(&ks);
	memset(&ss, 0, sizeof(ss));
	sts_snmp_stats(&ss);
	/*
	 * The USM counters come through sts_snmp_v3_stats(), which serves the
	 * SNMP thread's published snapshot. Calling snmp_v3_stats_get() on the
	 * live context from this worker would violate snmp_v3.h's one-context-
	 * one-thread rule (the context carries the decrypt scratch), so do not
	 * "simplify" it back to a direct read.
	 */
	memset(&vs, 0, sizeof(vs));
	sts_snmp_v3_stats(&vs);
	sts_ptp_stats(&ps);
	sts_web_stats(&ws);

	out->ntp_rx = ns.ntp.rx;
	out->ntp_served = ns.ntp.served;
	out->ntp_kod = ns.ntp.kod;
	out->ntp_dropped = ns.ntp.dropped;
	out->nts_served = ns.nts.ok;
	out->ntske_ok = ks.negotiations_ok;
	out->ntske_fail = ks.handshakes_failed;
	out->snmp_gets = (uint32_t)(ss.get + ss.getnext);
	out->snmp_v3_unknown_users = vs.unknown_user_names;
	out->snmp_v3_wrong_digests = vs.wrong_digests;
	out->snmp_v3_time_windows = vs.not_in_time_windows;
	out->snmp_v3_decrypt_errors = vs.decryption_errors;
	out->snmp_v3_unknown_engines = vs.unknown_engine_ids;
	out->snmp_v3_bad_sec_levels = vs.unsupported_sec_levels;
	out->snmp_v3_authenticated = vs.authenticated;
	out->web_requests = ws.requests;
	out->ntp_running = ns.running;
	out->nts_running = ks.running;
	out->ptp_running = ps.running;
	out->snmp_running = sts_net_cfg_bool(CFG_ID_SNMP_ENABLE, false) ||
			    (ss.rx != 0U);
	out->uptime_s = (uint32_t)(sts_mono_ms() / 1000ULL);
	(void)snprintf(out->fw_version, sizeof(out->fw_version), "%s",
		       APP_VERSION_STRING);
	(void)web_span_copy(out->model, sizeof(out->model), "STS1000", 7U);
	return 0;
}

static uint64_t pv_alarms(void *u)
{
	ARG_UNUSED(u);
	return sts_alarms_active();
}

static const char *pv_alarm_name(void *u, uint8_t bit)
{
	/*
	 * Ids 0..31 mirror fault_sig_t and core/fault names them; 32.. are raised
	 * by other modules and have no name table in core, so they are named
	 * here. Keeping the table next to the enum it mirrors is the least-bad
	 * option until core/fault grows fault_alarm_name().
	 */
	static const char *const soft[] = {
		"REFERENCE_LOST", "GNSS_LOST",       "ANTENNA_OPEN",
		"ANTENNA_SHORT",  "OCXO_UNHEALTHY",  "DAC_FAULT",
		"RB_FAULT",       "RB_OV",           "THERMAL_WARN",
		"THERMAL_CRITICAL", "FAN_FAULT",     "POE_BUDGET",
		"I2C_WEDGE",      "NOR_FAULT",       "DISPLAY_FAULT",
		"PFI",            "TAMPER",
	};

	ARG_UNUSED(u);
	if (bit < 32U) {
		return fault_sig_name((fault_sig_t)bit);
	}
	if ((size_t)(bit - 32U) < (sizeof(soft) / sizeof(soft[0]))) {
		return soft[bit - 32U];
	}
	return NULL;
}

/*
 * PTP alarm bits are a namespace of their own — core/ptp's, not core/fault's —
 * so they get their own renderer rather than being passed through pv_alarm_name()
 * above, which would name bit 3 "ANTENNA_SHORT" when it means
 * PTP_ALARM_PROFILE_UNSUPPORTED.
 */
static const char *pv_ptp_alarm_name(void *u, uint8_t bit)
{
	ARG_UNUSED(u);
	return ptp_alarm_name(bit);
}

/*
 * Control providers.
 *
 * Each of these runs on a web worker at priority 12 and must not touch a timing
 * thread's state. The platform area therefore exposes REQUEST entry points
 * (sts_app.h) rather than the underlying core APIs: the GNSS pair post into a
 * spinlock-guarded single-slot mailbox the priority-6 gnss thread drains, and
 * the reference override is one atomic word the priority-4 discipline thread
 * reads once a second. No provider here waits on either thread, so a hung or
 * busy timing loop cannot stall the management plane and a management plane
 * cannot perturb the timing loop (firmware/CLAUDE.md hard rule).
 *
 * All four return promptly with an ACCEPTANCE verdict, never a completion one.
 * A survey runs for gnss.survey.dur seconds; a reference handoff waits out
 * refsel's hysteresis window. The SPA polls telemetry for what actually
 * happened.
 */
static int pv_gnss_survey(void *u, bool start)
{
	ARG_UNUSED(u);

	/* -ENOTSUP on stop: core/gnssmgr has no survey abort. See
	 * sts_gnss_request_survey(). */
	return sts_gnss_request_survey(start);
}

static int pv_gnss_fixed(void *u, int64_t x, int64_t y, int64_t z)
{
	ARG_UNUSED(u);

	/* Centimetres straight through: the REST contract and gnssmgr_ecef_t use
	 * the same unit (rest.h gnss_fixed, gnssmgr.h gnssmgr_ecef_t::x_cm).
	 * Only the accuracy field is in 0.1 mm, and the caller supplies none. */
	return sts_gnss_set_fixed_ecef(x, y, z);
}

static int pv_ref_override(void *u, uint8_t mode)
{
	ARG_UNUSED(u);

	/*
	 * REST_REF_RB maps to STS_REF_REQ_EXTREF, not to a distinct "rubidium"
	 * request, because the two name the same thing: input B of the clock mux,
	 * reached over the SMA front end. refsel's AUTO already engages the
	 * rubidium whenever the guards allow, so an explicit selection of it *is*
	 * refsel.h's "explicit operator selection of input B".
	 *
	 * Written as a switch rather than a cast even though the enumerations
	 * agree numerically today: a silent divergence between rest.h and
	 * sts_app.h would put the wrong 10 MHz on PH0, and out-of-range values
	 * must fail rather than land on whatever enumerator they happen to hit.
	 */
	switch (mode) {
	case (uint8_t)REST_REF_AUTO:
		return sts_ref_override_set((uint8_t)STS_REF_REQ_AUTO);
	case (uint8_t)REST_REF_OCXO:
		return sts_ref_override_set((uint8_t)STS_REF_REQ_OCXO);
	case (uint8_t)REST_REF_RB:
		return sts_ref_override_set((uint8_t)STS_REF_REQ_EXTREF);
	default:
		return -EINVAL;
	}
}

/*
 * Calibration. Only REST_CAL_INA_TRIM is a procedure a running unit can perform
 * on itself; the other three are bench operations whose results are entered
 * through the group 0x0C cfg keys, and they answer -ENOTSUP rather than
 * pretending to start (see sts_cal_run() and core/cal/cal.h).
 *
 * `arg` for the trim is the packed word STS_CAL_INA_ARG(rail, i_ref_ua) — rail
 * index in the top byte, the reference instrument's current in microamps in the
 * low 24 bits. One uint32_t is all the REST contract carries, and the procedure
 * needs both numbers: the board cannot measure its own reference.
 *
 * KNOWN GAP, reported to the web layer: firmware/web/app.js sends `arg` as a
 * bare rail index (0..8), which decodes here as rail 0 with a reference current
 * of 0..8 uA. That fails the plausibility band and is refused — the honest
 * outcome, but the SPA control cannot succeed until it sends the packed word.
 */
static int pv_cal_run(void *u, uint8_t proc, uint32_t arg)
{
	ARG_UNUSED(u);

	switch (proc) {
	case (uint8_t)REST_CAL_HOLDOVER:
		return sts_cal_run((uint8_t)STS_CAL_HOLDOVER, arg);
	case (uint8_t)REST_CAL_OCXO_TUNE:
		return sts_cal_run((uint8_t)STS_CAL_OCXO_TUNE, arg);
	case (uint8_t)REST_CAL_INA_TRIM:
		return sts_cal_run((uint8_t)STS_CAL_INA_TRIM, arg);
	case (uint8_t)REST_CAL_COMPASS:
		return sts_cal_run((uint8_t)STS_CAL_COMPASS, arg);
	default:
		return -EINVAL;
	}
}

/*
 * Service enable/disable is expressed as a config change, which is exactly what
 * the underlying keys are: the service reads its enable key at start, and most of
 * them are CFG_F_REBOOT_REQUIRED. So this stages the key and the operator commits
 * it — the response tells them a commit is required, and the router reports
 * `commit_required`.
 *
 * Staged under sts_cfg_lock(). sts_app.h: sts_cfg() is not internally locked and
 * the MCP engine, the shell backend and the ui_local thread write the same tree,
 * so every mutation MUST be bracketed. This runs on a web worker, the one plane
 * that was still writing it bare. No commit here — the operator commits through
 * /api/v1/config/commit, which reaches sts_cfg_commit() via commit_via_app() and
 * takes the mutex itself.
 */
static int pv_service_enable(void *u, uint8_t svc, bool enable)
{
	static const uint16_t keys[REST_SVC_COUNT] = {
		(uint16_t)CFG_ID_NTP_ENABLE,  (uint16_t)CFG_ID_NTS_ENABLE,
		(uint16_t)CFG_ID_PTP_ENABLE,  (uint16_t)CFG_ID_SNMP_ENABLE,
		(uint16_t)CFG_ID_LOG_SYSLOG_EN,
	};
	int rc;

	ARG_UNUSED(u);
	if (svc >= (uint8_t)REST_SVC_COUNT) {
		return -EINVAL;
	}

	sts_cfg_lock();
	rc = cfg_set_u64(sts_cfg(), keys[svc], enable ? 1U : 0U);
	sts_cfg_unlock();

	return (rc != 0) ? -EIO : 0;
}

/*
 * Reboot. The response has to reach the browser before the reset, and this runs
 * on the worker that still owes it, so the reset is deferred onto the system work
 * queue — the same trick MCP uses with MCP_REBOOT_DELAY_MS.
 */
static uint8_t pending_reboot_mode;

static void reboot_handler(struct k_work *w)
{
	const port_image_t *img;

	ARG_UNUSED(w);
	if (sts_dfu_port == NULL) {
		return;
	}
	img = sts_dfu_port();
	if (img != NULL && img->reboot != NULL) {
		LOG_WRN("reboot requested over the web interface (mode %u)",
			pending_reboot_mode);
		img->reboot(img->ctx, (int)pending_reboot_mode);
	}
}

static K_WORK_DELAYABLE_DEFINE(reboot_work, reboot_handler);

static int pv_reboot(void *u, uint8_t mode)
{
	const port_image_t *img;

	ARG_UNUSED(u);
	if (sts_dfu_port == NULL) {
		return -ENOSYS;
	}
	img = sts_dfu_port();
	if (img == NULL || img->reboot == NULL) {
		return -ENOSYS;
	}
	pending_reboot_mode = mode;
	(void)k_work_reschedule(&reboot_work, K_MSEC(500));
	return 0;
}

/*
 * ---------------------------------------------------------------------------
 * Factory reset: config AND key material (spec §9.6 secure-erase)
 * ---------------------------------------------------------------------------
 *
 * What a factory reset now ERASES:
 *
 *   1. Every cfg key, back to its schema default, in the live tree and in NVS —
 *      sts_cfg_factory_reset(). That covers every CFG_F_SECRET blob: the three
 *      management credentials (sec.admin.pw, sec.operator.pw, sec.viewer.pw),
 *      the SNMPv3 USM auth/priv keys, the four NTP symmetric MAC keys, and the
 *      RADIUS/TACACS+ shared secrets and LDAP bind password. cfg.c's
 *      val_default() memsets the WHOLE cfg_val_t, so the secret bytes go, not
 *      just the length.
 *   2. Every subsystem's RAM copy of those, because the same call then runs
 *      every registered group's appliers — which is how the emptied keys reach
 *      core/snmp's USM table, core/ntp's key slots and sts_aaa.c's local blob.
 *   3. The web plane's own credential blobs and every live session, token and
 *      CSRF token — auth_web_wipe(), through a volatile pointer.
 *   4. The persisted TLS server identity: private key and certificate on
 *      /lfs, the ACME account key, the Zephyr credential-store entries, and the
 *      in-RAM PEM buffers — sts_cert_reset(). This is the one place sts_cert.c's
 *      deliberate persistence is reversed.
 *   4b. The LDAPS trust anchor — sts_aaa_ldap_ca_erase(). It is NOT a secret,
 *      which is exactly why the decision to erase it needs stating: what it is
 *      is the operator's answer to "which directory may tell this box who its
 *      administrators are". A wiped unit that still trusts the previous
 *      operator's CA has kept the most consequential half of an AAA
 *      configuration whose other half — the `sec.ldap.*` keys in (1) — has just
 *      been cleared, leaving a trust decision with nothing left to explain it.
 *      It also lives in /lfs/tls beside the material in (4) and was installed
 *      the same way, through the same admin-only route, so treating the two
 *      differently would be the surprise. Erase.
 *
 *      What it erases and what it defers is not symmetric, and the asymmetry
 *      is deliberate: the /lfs file is unlinked with no lock held, because the
 *      AAA exchange mutex is held for the length of a directory bind and
 *      waiting for it here would blow this thread's liveness deadline and
 *      cold-cycle the board mid-reset; the in-RAM copy is dropped under a
 *      bounded wait and, if an exchange owns it, left for (5) to clear.
 *      sts_ldap_ca.h states why that is sound and what it depends on.
 *   5. Everything that lives only in RAM — the NTS cookie master keyring
 *      (src/zephyr/net/sts_net.c), the per-boot NTS-KE server key, mbedTLS and
 *      PSA volatile keys, and any in-RAM LDAPS anchor (4b) could not take under
 *      its bounded wait — by REBOOTING. That reboot is not a convenience: it
 *      is what makes the claim in (5) true, and it is also what mints the fresh
 *      TLS identity that (4) deleted. It is scheduled unconditionally below,
 *      and (4b) depends on that.
 *
 * What it deliberately does NOT erase, and why:
 *
 *   - the ATECC608B device key and its slots. That is the hardware root of
 *     identity and attestation (spec §9.1); its key is non-exportable and was
 *     provisioned at manufacture, not by the operator. Erasing it would not
 *     un-deploy a secret an operator put there — it would permanently destroy
 *     the board's ability to attest to what it is, with no way back in the
 *     field. Spec §9.6 puts secure-erase of the secure element behind a
 *     separate, physically-gated policy for exactly that reason;
 *   - the MCUboot signing trust anchor and the anti-rollback security counter.
 *     Neither is a secret (one is a public key, one is a monotonic counter), and
 *     rolling the counter back would re-enable installing a firmware image that
 *     was withdrawn for a security defect;
 *   - the log ring and the NOR syslog spool. They are an audit record, and the
 *     factory reset itself is one of the events in it. They hold no key
 *     material: core/logring stores messages, and every credential path logs
 *     outcomes rather than values.
 */
static int pv_factory_reset(void *u)
{
	int rc = 0;

	ARG_UNUSED(u);
	/*
	 * sts_cfg_factory_reset() clears the tree, erases the store and then runs
	 * every registered group's appliers, so no subsystem is left serving
	 * pre-reset configuration until the next reboot — the bare
	 * cfg_factory_reset() this used to call told nobody.
	 */
	if (sts_cfg_factory_reset() != 0) {
		rc = -EIO;
	}
	if (sts_sec_factory_wipe() != 0) {
		rc = -EIO;
	}

	/*
	 * Deferred well past the response, like pv_reboot(): the browser is owed
	 * the answer to the request that caused this, and the worker that owes it
	 * is this one.
	 */
	pending_reboot_mode = 0U;
	(void)k_work_reschedule(&reboot_work, K_MSEC(1500));

	return rc;
}

int sts_sec_factory_wipe(void)
{
	int rc = 0;

	/* The credential blobs and every session. Under api_lock, because the
	 * auth registry is not internally locked and a worker may be in it. */
	k_mutex_lock(&api_lock, K_FOREVER);
	auth_web_wipe(&auth_ctx);
	k_mutex_unlock(&api_lock);

	if (sts_cert_reset() != 0) {
		rc = -EIO;
	}
	/* The operator's LDAPS trust anchor — see (4b) above. */
	if (sts_aaa_ldap_ca_erase() != 0) {
		rc = -EIO;
	}
	/* And the syslog collector's anchor, for the same reason and with the
	 * same non-secret status: it is the operator's answer to "which host
	 * may receive this box's audit trail". Leaving it behind would have a
	 * wiped unit keep opening an encrypted session to the previous
	 * operator's log collector as soon as `log.syslog.*` was set again. */
	if (sts_syslog_ca_erase() != 0) {
		rc = -EIO;
	}

	sts_log((uint8_t)LOGR_SUB_SEC, (uint8_t)LOGR_ALERT,
		"factory reset: key material zeroized (credentials, sessions, "
		"TLS identity, LDAPS and syslog trust anchors); rebooting to "
		"clear RAM-only keys");
	return rc;
}

/* ------------------------------------------------------------------------- */
/* firmware: the MCP-client bridge                                           */
/* ------------------------------------------------------------------------- */

static mcp_ctx_t dfu_mcp;
static bool dfu_mcp_ready;

/* Last response frame the engine emitted, captured from its tx callback. */
static struct {
	uint8_t payload[64];
	uint16_t len;
	uint8_t cmd;
	bool have;
} dfu_rsp;

static int dfu_tx(void *user, const uint8_t *buf, size_t len)
{
	uint8_t frame[MCP_MAX_FRAME];
	size_t dn = 0U;
	mcp_frame_t f;

	ARG_UNUSED(user);
	dfu_rsp.have = false;
	if (len < 2U) {
		return 0;
	}
	/* The engine hands over a COBS-encoded, 0x00-delimited frame. */
	if (cobs_decode(buf, len - 1U, frame, sizeof(frame), &dn) != 0) {
		return 0;
	}
	if (mcp_wire_parse(frame, dn, &f) != 0) {
		return 0;
	}
	if (f.type != (uint8_t)MCP_T_RSP) {
		return 0; /* an EVT; the bridge subscribes to none */
	}
	dfu_rsp.cmd = f.cmd;
	dfu_rsp.len = (f.len > sizeof(dfu_rsp.payload))
			      ? (uint16_t)sizeof(dfu_rsp.payload)
			      : f.len;
	memcpy(dfu_rsp.payload, f.payload, dfu_rsp.len);
	dfu_rsp.have = true;
	return 0;
}

static int dfu_bridge_init(void)
{
	mcp_wiring_t w;
	const port_image_t *img;

	if (dfu_mcp_ready) {
		return 0;
	}
	if (sts_dfu_port == NULL) {
		return -ENOSYS;
	}
	img = sts_dfu_port();
	if (img == NULL) {
		return -ENOSYS;
	}

	memset(&w, 0, sizeof(w));
	w.crypto = sts_port_crypto();
	w.sha = sts_port_sha256_stream();
	w.img = img;
	w.log = sts_logring();
	/*
	 * No cfg on purpose: with no credential store the engine treats auth as
	 * not required, which is correct because this instance is unreachable
	 * from outside — core/web has already authenticated and authorised the
	 * caller. See the header comment.
	 */
	w.cfg = NULL;
	w.tx = dfu_tx;
	w.tx_user = NULL;
	(void)web_span_copy(w.ident.model, sizeof(w.ident.model), "STS1000", 7U);

	if (mcp_init(&dfu_mcp, &w) != 0) {
		return -EIO;
	}
	dfu_mcp_ready = true;
	return 0;
}

/*
 * Synthesise one MCP request and run it through the engine. Returns the RSP
 * status byte (MCP_OK or an mcp_err_t), or a negative errno if the frame could
 * not be built or produced no response.
 */
static int dfu_call(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
	uint8_t frame[MCP_MAX_FRAME];
	uint8_t enc[COBS_ENCODE_MAX(MCP_MAX_FRAME) + 1U];
	size_t frame_len = 0U;
	size_t enc_len = 0U;
	size_t consumed = 0U;
	static uint16_t seq;
	int rc;

	rc = dfu_bridge_init();
	if (rc != 0) {
		return rc;
	}
	rc = mcp_wire_build((uint8_t)MCP_T_REQ, cmd, 0U, seq++, payload, len,
			    frame, sizeof(frame), &frame_len);
	if (rc != 0) {
		return rc;
	}
	rc = cobs_encode(frame, frame_len, enc, sizeof(enc) - 1U, &enc_len);
	if (rc != 0) {
		return rc;
	}
	enc[enc_len++] = 0x00U;

	(void)mcp_tick(&dfu_mcp, sts_mono_ms());
	dfu_rsp.have = false;
	rc = mcp_input(&dfu_mcp, enc, enc_len, &consumed);
	if (rc != 0 && rc != -EAGAIN) {
		return rc;
	}
	if (!dfu_rsp.have || dfu_rsp.len == 0U) {
		return -EIO;
	}
	if (dfu_rsp.cmd != cmd) {
		return -EPROTO;
	}
	return (int)dfu_rsp.payload[0];
}

/* Map an MCP status onto the rest_providers_t error contract (rest.h). */
static int dfu_map(int status)
{
	if (status < 0) {
		return status;
	}
	switch (status) {
	case MCP_OK:
		return 0;
	case MCP_ERR_NOSPC:
		return -ENOSPC;
	case MCP_ERR_OFFSET:
		return -EPROTO;
	case MCP_ERR_VERIFY:
		return -EBADMSG;
	case MCP_ERR_NOTSUP:
		return -ENOSYS;
	case MCP_ERR_STATE:
	case MCP_ERR_BUSY:
		return -EBUSY;
	case MCP_ERR_ARG:
		return -EINVAL;
	default:
		return -EIO;
	}
}

static uint32_t rsp_le32(size_t off)
{
	if ((off + 4U) > dfu_rsp.len) {
		return 0U;
	}
	return (uint32_t)dfu_rsp.payload[off] |
	       ((uint32_t)dfu_rsp.payload[off + 1U] << 8) |
	       ((uint32_t)dfu_rsp.payload[off + 2U] << 16) |
	       ((uint32_t)dfu_rsp.payload[off + 3U] << 24);
}

static int pv_fw_info(void *u, rest_fw_t *out)
{
	const mcp_dfu_t *d;
	uint8_t slot;

	ARG_UNUSED(u);
	if (dfu_bridge_init() != 0) {
		return -ENOSYS;
	}
	memset(out, 0, sizeof(*out));
	out->n_slots = 2U;
	for (slot = 0U; slot < 2U; slot++) {
		port_image_info_t info;
		const port_image_t *img = sts_dfu_port();

		memset(&info, 0, sizeof(info));
		if (img != NULL && img->image_info != NULL) {
			(void)img->image_info(img->ctx, slot, &info);
		}
		out->slot[slot].slot = slot;
		out->slot[slot].size = info.size;
		memcpy(out->slot[slot].version, info.version,
		       sizeof(out->slot[slot].version));
		out->slot[slot].valid = info.valid;
		out->slot[slot].active = info.active;
		out->slot[slot].pending = info.pending;
		out->slot[slot].confirmed = info.confirmed;
	}
	d = mcp_dfu_status(&dfu_mcp);
	if (d != NULL) {
		out->dfu_state = d->state;
		out->dfu_total = d->total;
		out->dfu_written = d->written;
		out->write_block = d->write_block;
	}
	out->chunk_max = (uint32_t)MCP_FW_CHUNK_MAX;
	out->pending_confirm = sts_update_pending_confirm();
	return 0;
}

static int pv_fw_begin(void *u, uint32_t size, const uint8_t sha[32],
		       uint32_t *out_next)
{
	uint8_t payload[36];
	int status;

	ARG_UNUSED(u);
	payload[0] = (uint8_t)(size & 0xFFU);
	payload[1] = (uint8_t)((size >> 8) & 0xFFU);
	payload[2] = (uint8_t)((size >> 16) & 0xFFU);
	payload[3] = (uint8_t)((size >> 24) & 0xFFU);
	memcpy(&payload[4], sha, 32U);

	status = dfu_call((uint8_t)MCP_CMD_FW_BEGIN, payload, sizeof(payload));
	if (status == MCP_OK && out_next != NULL) {
		*out_next = rsp_le32(1U);
	}
	return dfu_map(status);
}

static int pv_fw_data(void *u, uint32_t off, const uint8_t *data, size_t len,
		      uint32_t *out_next)
{
	/* Static, not stack: 1 KB+ of locals would blow the worker stack. Safe
	 * because every provider call is made under api_lock. */
	static uint8_t payload[4U + MCP_FW_CHUNK_MAX];
	int status;

	ARG_UNUSED(u);
	if (len == 0U || len > MCP_FW_CHUNK_MAX) {
		return -EINVAL;
	}
	payload[0] = (uint8_t)(off & 0xFFU);
	payload[1] = (uint8_t)((off >> 8) & 0xFFU);
	payload[2] = (uint8_t)((off >> 16) & 0xFFU);
	payload[3] = (uint8_t)((off >> 24) & 0xFFU);
	memcpy(&payload[4], data, len);

	status = dfu_call((uint8_t)MCP_CMD_FW_DATA, payload,
			  (uint16_t)(4U + len));
	/* FW_DATA always carries the frontier, success or offset error. */
	if (status >= 0 && out_next != NULL) {
		*out_next = rsp_le32(1U);
	}
	return dfu_map(status);
}

static int pv_fw_end(void *u, uint32_t *out_size)
{
	int status;

	ARG_UNUSED(u);
	status = dfu_call((uint8_t)MCP_CMD_FW_END, NULL, 0U);
	if (status == MCP_OK && out_size != NULL) {
		*out_size = rsp_le32(1U);
	}
	return dfu_map(status);
}

static int pv_fw_confirm(void *u)
{
	ARG_UNUSED(u);
	return dfu_map(dfu_call((uint8_t)MCP_CMD_FW_CONFIRM, NULL, 0U));
}

static int pv_fw_revert(void *u)
{
	ARG_UNUSED(u);
	return dfu_map(dfu_call((uint8_t)MCP_CMD_FW_REVERT, NULL, 0U));
}

/* ------------------------------------------------------------------------- */
/* TLS providers                                                             */
/* ------------------------------------------------------------------------- */

static int pv_cert_info(void *u, rest_cert_t *out)
{
	ARG_UNUSED(u);
	return sts_cert_info(out);
}

static int pv_cert_install(void *u, const char *pem, size_t len)
{
	ARG_UNUSED(u);
	return sts_cert_install(pem, len);
}

static int pv_csr(void *u, const char *subject, char *out, size_t cap,
		  size_t *out_len)
{
	ARG_UNUSED(u);
	return sts_cert_csr(subject, out, cap, out_len);
}

/*
 * The LDAPS trust anchor lives in the AAA area, not here — sts_aaa.c owns the
 * socket that consumes it. These two are the same shape as the certificate
 * providers above so the router's error contract is one contract, and they are
 * bound unconditionally: sts_aaa.c answers -ENOTSUP in a build with no TLS
 * socket layer, which the route turns into a 501 that says so.
 */
static int pv_ldap_ca_info(void *u, rest_trust_t *out)
{
	sts_aaa_ldap_ca_t ca;
	int rc;

	ARG_UNUSED(u);
	rc = sts_aaa_ldap_ca_info(&ca);
	if (rc != 0) {
		return rc;
	}
	out->present = ca.present;
	out->persisted = ca.persisted;
	(void)web_span_copy(out->subject, sizeof(out->subject), ca.subject,
			    strlen(ca.subject));
	(void)web_span_copy(out->not_after, sizeof(out->not_after),
			    ca.not_after, strlen(ca.not_after));
	(void)web_span_copy(out->sha256_fp, sizeof(out->sha256_fp),
			    ca.sha256_fp, strlen(ca.sha256_fp));
	return 0;
}

static int pv_ldap_ca_install(void *u, const char *pem, size_t len)
{
	ARG_UNUSED(u);
	return sts_aaa_ldap_ca_install(pem, len);
}

static int pv_syslog_ca_present(void *u, bool *out)
{
	ARG_UNUSED(u);
	if (out == NULL) {
		return -EINVAL;
	}
	*out = sts_syslog_ca_present();
	return 0;
}

static int pv_syslog_ca_install(void *u, const char *pem, size_t len)
{
	ARG_UNUSED(u);
	return sts_syslog_ca_install(pem, len);
}

static void providers_bind(void)
{
	memset(&providers, 0, sizeof(providers));
	providers.quality = pv_quality;
	providers.health = pv_health;
	providers.gnss = pv_gnss;
	providers.net = pv_net;
	providers.ptp = pv_ptp;
	providers.services = pv_services;
	providers.alarms = pv_alarms;
	providers.alarm_name = pv_alarm_name;
	providers.ptp_alarm_name = pv_ptp_alarm_name;
	providers.gnss_survey = pv_gnss_survey;
	providers.gnss_fixed = pv_gnss_fixed;
	providers.ref_override = pv_ref_override;
	providers.service_enable = pv_service_enable;
	providers.cal_run = pv_cal_run;
	providers.reboot = pv_reboot;
	providers.factory_reset = pv_factory_reset;
	providers.cert_info = pv_cert_info;
	providers.cert_install = pv_cert_install;
	providers.csr_make = pv_csr;
	providers.ldap_ca_info = pv_ldap_ca_info;
	providers.ldap_ca_install = pv_ldap_ca_install;
	providers.syslog_ca_present = pv_syslog_ca_present;
	providers.syslog_ca_install = pv_syslog_ca_install;
	if (sts_dfu_port != NULL) {
		providers.fw_info = pv_fw_info;
		providers.fw_begin = pv_fw_begin;
		providers.fw_data = pv_fw_data;
		providers.fw_end = pv_fw_end;
		providers.fw_confirm = pv_fw_confirm;
		providers.fw_revert = pv_fw_revert;
	}
	providers.u = NULL;
}

/* ========================================================================= */
/* cfg commit hook                                                           */
/* ========================================================================= */

/* The router must not call cfg_commit() directly — the appliers have to run. */
static int commit_via_app(void *u, cfg_commit_res_t *res)
{
	ARG_UNUSED(u);
	return sts_cfg_commit(res);
}

/* Re-read the credential and the session policy after a security commit. */
static void cfg_applied(void *ctx, uint8_t group)
{
	ARG_UNUSED(ctx);
	if (group != CFG_G_SEC) {
		return;
	}
	k_mutex_lock(&api_lock, K_FOREVER);
	(void)auth_web_reload(&auth_ctx);
	k_mutex_unlock(&api_lock);
	LOG_INF("security config applied; credentials reloaded");
}

/*
 * ---------------------------------------------------------------------------
 * The remote authority, and the two hazards of reaching it from here
 * ---------------------------------------------------------------------------
 *
 * core/web asks this when the local account table could not accept a
 * credential. One line of substance, and that is the point: the backend chain,
 * the positive cache, the hard lockout and the role mapping all live in
 * sts_aaa_check(), shared with the MCP console and the maintenance planes
 * rather than reimplemented three times.
 *
 * Every non-zero return is a denial, -EHOSTUNREACH included — that one means no
 * authority could answer, and it is never an allow-on-failure (sts_aaa.h).
 * Only -EBUSY keeps its identity, because core/web reports a lockout distinctly
 * so the route can send Retry-After; every other refusal reaches the browser as
 * one indistinguishable 401.
 *
 * Hazard 1 — the watchdog. This runs on a web worker, and the workers feed a
 * liveness participant. sts_aaa_check() blocks for the operator's whole
 * configured chain timeout (up to ~540 s; see sts_secops.h for the arithmetic)
 * against an unreachable server, and CONFIG_STS1000_LIVENESS_DEADLINE_MS is
 * 5 000. A direct call would therefore let one unauthenticated POST cold-cycle
 * the board. sts_aaa_check_fed() runs the lookup on its own thread and feeds
 * `live_id` while this worker sleeps, which is why it is used here and a bare
 * sts_aaa_check() must never be.
 *
 * Hazard 2 — api_lock, which this thread holds across rest_dispatch() and
 * therefore across this call. It is held ON PURPOSE and is NOT released here,
 * unlike the maintenance plane's engine lock:
 *
 *   - it is the mutual exclusion for auth_web_ctx_t AND rest_ctx_t, neither of
 *     which is internally locked. Dropping it mid-login would let a second
 *     worker enter auth_web_login(), auth_web_reload() or auth_web_set_password()
 *     while this one is suspended inside the first — memory corruption in the
 *     credential store, which is a far worse outcome than the delay;
 *   - the watchdog does not need it released, because hazard 1 is already
 *     solved: the STS_WEB_WORKERS share ONE liveness id, and this worker keeps
 *     feeding it throughout.
 *
 * Hazard 3 — the one hazard 1 and hazard 2 create between them. Keeping
 * api_lock is right, and feeding liveness is right, but together they mean one
 * UNAUTHENTICATED POST /api/login could park the entire HTTPS management plane
 * for the operator's whole configured chain timeout (~540 s) with nothing
 * noticing: the watchdog was being fed the whole time, on purpose. A detectable
 * hang had been converted into an undetectable one.
 *
 * The fix is a tight budget on this specific call — not on the console planes'
 * — because this is the caller holding something everyone else needs. The other
 * two candidates were rejected: dropping api_lock trades a bounded stall for
 * credential-store corruption (see above), and capping concurrent lookups is a
 * no-op because api_lock and the AAA gate each already cap them at one. The
 * full argument, and why 10 s, is written down at STS_AAA_FED_PREAUTH_MS in
 * net/sts_secops_policy.h.
 *
 * The cost is now honest AND numbered: while a remote lookup is outstanding,
 * REST requests queue behind api_lock for at most STS_AAA_FED_PREAUTH_MS plus
 * one feed slice, and a second attempt made while the first is still stuck is
 * refused immediately rather than buying the budget again. That happens only on
 * a unit configured for remote AAA whose server is unreachable — the default
 * chain is `local`, which never blocks at all — and a briefly slow management
 * plane is the correct trade against either a corrupted credential table or a
 * rebooting grandmaster.
 */
static int web_remote_auth(void *user, const char *name, const char *secret,
			   uint8_t *out_role)
{
	int rc;

	ARG_UNUSED(user);

	rc = sts_aaa_check_fed_budgeted(name, secret, out_role, live_id,
					STS_AAA_FED_PREAUTH_MS);
	if (rc != 0) {
		*out_role = (uint8_t)WEB_ROLE_NONE;
		return (rc == -EBUSY) ? -EBUSY : -EACCES;
	}
	/*
	 * auth_role_t and web_role_t are the same four numbers by construction
	 * (auth.h, web.h), so the value passes straight through; core/web clamps
	 * anything outside the range to the least privilege regardless.
	 */
	return 0;
}

/* ========================================================================= */
/* transport helpers                                                         */
/* ========================================================================= */

static ssize_t send_all(int fd, const void *buf, size_t len)
{
	const uint8_t *p = buf;
	size_t off = 0U;

	while (off < len) {
		ssize_t n = zsock_send(fd, &p[off], len - off, 0);

		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				struct zsock_pollfd pfd = {
					.fd = fd,
					.events = ZSOCK_POLLOUT,
					.revents = 0,
				};

				if (zsock_poll(&pfd, 1, 2000) <= 0) {
					return -1;
				}
				continue;
			}
			return -1;
		}
		if (n == 0) {
			return -1;
		}
		off += (size_t)n;
	}
	return (ssize_t)off;
}

/* Common security headers. HSTS is only meaningful on the TLS listener. */
#define SEC_HEADERS                                                            \
	"X-Content-Type-Options: nosniff\r\n"                                  \
	"X-Frame-Options: DENY\r\n"                                            \
	"Referrer-Policy: no-referrer\r\n"                                     \
	"Content-Security-Policy: default-src 'self'; script-src 'self'; "     \
	"style-src 'self'; img-src 'self' data:; connect-src 'self' wss:; "    \
	"object-src 'none'; base-uri 'none'; frame-ancestors 'none'\r\n"       \
	"Strict-Transport-Security: max-age=31536000\r\n"

static int send_response(int fd, const web_resp_t *r, bool head_only)
{
	char hdr[1024];
	size_t o = 0U;

	o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o,
			      "HTTP/1.1 %u %s\r\n", r->status,
			      rest_status_text(r->status));
	o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o,
			      "Content-Type: %s\r\nContent-Length: %u\r\n",
			      (r->content_type != NULL) ? r->content_type
							: "application/json",
			      (unsigned int)r->body_len);
	o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o, SEC_HEADERS);
	if (r->no_store) {
		o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o,
				      "Cache-Control: no-store\r\n");
	}
	if (r->etag[0] != '\0') {
		o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o, "ETag: %s\r\n",
				      r->etag);
	}
	if (r->extra_len != 0U && (o + r->extra_len) < (sizeof(hdr) - 32U)) {
		memcpy(&hdr[o], r->extra, r->extra_len);
		o += r->extra_len;
	}
	o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o, "Connection: %s\r\n\r\n",
			      r->close ? "close" : "keep-alive");

	if (send_all(fd, hdr, o) < 0) {
		return -EIO;
	}
	if (!head_only && r->body_len != 0U) {
		if (send_all(fd, r->body, r->body_len) < 0) {
			return -EIO;
		}
	}
	return 0;
}

static int send_simple(int fd, uint16_t status, const char *ctype,
		       const char *body, const char *extra)
{
	char hdr[768];
	size_t blen = (body == NULL) ? 0U : strlen(body);
	size_t o = 0U;

	o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o,
			      "HTTP/1.1 %u %s\r\nContent-Type: %s\r\n"
			      "Content-Length: %u\r\n",
			      status, rest_status_text(status),
			      (ctype != NULL) ? ctype : "text/plain",
			      (unsigned int)blen);
	o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o, SEC_HEADERS);
	if (extra != NULL) {
		o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o, "%s", extra);
	}
	o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o,
			      "Connection: close\r\n\r\n");
	if (send_all(fd, hdr, o) < 0) {
		return -EIO;
	}
	if (blen != 0U && send_all(fd, body, blen) < 0) {
		return -EIO;
	}
	return 0;
}

/* ========================================================================= */
/* static assets                                                             */
/* ========================================================================= */

static int serve_static(struct worker *wk, int fd, const http_req_t *req)
{
	sts_webfs_asset_t a;
	char hdr[768];
	size_t o = 0U;
	size_t off = 0U;
	int rc;

	rc = sts_webfs_resolve(req->path.p, req->path.n,
			       (req->flags & HTTP_F_GZIP_OK) != 0U, &a);
	if (rc != 0) {
		return send_simple(fd, 404, "text/plain", "not found\n", NULL);
	}

	/* Conditional GET: the SPA is cache-busted by its ETag. */
	if (a.etag[0] != '\0' && req->if_none_match.n != 0U &&
	    web_span_eq(req->if_none_match.p, req->if_none_match.n, a.etag)) {
		char nm[512];

		(void)snprintf(nm, sizeof(nm),
			       "HTTP/1.1 304 Not Modified\r\nETag: %s\r\n"
			       SEC_HEADERS "Connection: keep-alive\r\n\r\n",
			       a.etag);
		return (send_all(fd, nm, strlen(nm)) < 0) ? -EIO : 0;
	}

	o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o,
			      "HTTP/1.1 200 OK\r\nContent-Type: %s\r\n"
			      "Content-Length: %u\r\n",
			      a.content_type, (unsigned int)a.len);
	if (a.gzip) {
		o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o,
				      "Content-Encoding: gzip\r\nVary: "
				      "Accept-Encoding\r\n");
	}
	if (a.etag[0] != '\0') {
		o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o, "ETag: %s\r\n",
				      a.etag);
	}
	o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o, SEC_HEADERS);
	o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o,
			      "Cache-Control: public, max-age=300\r\n"
			      "Connection: keep-alive\r\n\r\n");
	if (send_all(fd, hdr, o) < 0) {
		return -EIO;
	}
	if (req->method == (uint8_t)WEB_METHOD_HEAD) {
		return 0;
	}

	while (off < a.len) {
		int n = sts_webfs_read(&a, off, wk->rx, sizeof(wk->rx));

		if (n <= 0) {
			return (n == 0) ? 0 : -EIO;
		}
		if (send_all(fd, wk->rx, (size_t)n) < 0) {
			return -EIO;
		}
		off += (size_t)n;
	}
	return 0;
}

/* ========================================================================= */
/* WebSocket                                                                 */
/* ========================================================================= */

static int ws_flush(struct worker *wk, int fd)
{
	const uint8_t *p;
	size_t n;

	while (wss_txq_peek(&wk->txq, &p, &n) == 0) {
		if (send_all(fd, p, n) < 0) {
			return -EIO;
		}
		(void)wss_txq_pop(&wk->txq);
		stat_add(&st.ws_frames_tx, 1U);
	}
	return 0;
}

/* Control frames only; see the wss_txq_t contract. */
static int ws_queue(struct worker *wk, uint8_t op, const uint8_t *payload,
		    size_t len, bool urgent)
{
	int n = wss_encode(op, true, payload, len, wk->frame, sizeof(wk->frame));

	if (n < 0) {
		return n;
	}
	if (wss_txq_push(&wk->txq, wk->frame, (size_t)n, urgent) == 1) {
		stat_add(&st.ws_dropped, 1U);
	}
	return 0;
}

/*
 * Send an application text message straight from wk->resp: the header goes out
 * first, then the payload. Nothing is staged, so a 6 KB telemetry record costs
 * no extra SRAM — which is why these do not go through the queue.
 */
static int ws_send_text(struct worker *wk, int fd, const char *body, size_t len)
{
	uint8_t hdr[WSS_HDR_MAX];
	int hn;

	if (ws_flush(wk, fd) != 0) {
		return -EIO;
	}
	hn = wss_encode_header((uint8_t)WSS_OP_TEXT, true, len, hdr,
			       sizeof(hdr));
	if (hn < 0) {
		return hn;
	}
	if (send_all(fd, hdr, (size_t)hn) < 0) {
		return -EIO;
	}
	if (send_all(fd, body, len) < 0) {
		return -EIO;
	}
	stat_add(&st.ws_frames_tx, 1U);
	return 0;
}

static void ws_run(struct worker *wk, int fd, const rest_authctx_t *actx)
{
	wss_rx_t rx;
	wss_sub_t sub;
	size_t have = 0U;
	uint64_t last_rx_ms = sts_mono_ms();

	ARG_UNUSED(actx);
	(void)wss_rx_init(&rx, wk->ws_msg, sizeof(wk->ws_msg));
	wss_sub_init(&sub);
	wss_txq_init(&wk->txq);
	stat_add(&st.ws_upgrades, 1U);

	for (;;) {
		struct zsock_pollfd pfd = {
			.fd = fd,
			.events = ZSOCK_POLLIN,
			.revents = 0,
		};
		uint64_t now;
		int pr;

		if (ws_flush(wk, fd) != 0) {
			break;
		}
		pr = zsock_poll(&pfd, 1, 250);
		now = sts_mono_ms();
		if (live_id >= 0) {
			sts_liveness_feed(live_id);
		}

		if (pr > 0 && (pfd.revents & ZSOCK_POLLIN) != 0) {
			ssize_t n = zsock_recv(fd, &wk->rx[have],
					       sizeof(wk->rx) - have, 0);

			if (n <= 0) {
				if (n < 0 && (errno == EAGAIN ||
					      errno == EWOULDBLOCK)) {
					continue;
				}
				break;
			}
			have += (size_t)n;
			last_rx_ms = now;

			for (;;) {
				size_t consumed = 0U;
				uint8_t op = 0U;
				const uint8_t *p = NULL;
				size_t plen = 0U;
				uint16_t code = WSS_CLOSE_PROTOCOL_ERROR;
				int frc = wss_rx_feed(&rx, wk->rx, have,
						      &consumed, &op, &p, &plen,
						      &code);

				if (consumed != 0U) {
					memmove(wk->rx, &wk->rx[consumed],
						have - consumed);
					have -= consumed;
				}
				if (frc < 0) {
					int cn = wss_encode_close(code, NULL,
								  wk->frame,
								  sizeof(wk->frame));

					if (cn > 0) {
						(void)send_all(fd, wk->frame,
							       (size_t)cn);
					}
					goto done;
				}
				if (frc == 0) {
					break;
				}
				if (op == (uint8_t)WSS_OP_CLOSE) {
					int cn = wss_encode_close(
						WSS_CLOSE_NORMAL, NULL,
						wk->frame, sizeof(wk->frame));

					if (cn > 0) {
						(void)send_all(fd, wk->frame,
							       (size_t)cn);
					}
					goto done;
				}
				if (op == (uint8_t)WSS_OP_PING) {
					(void)ws_queue(wk,
						       (uint8_t)WSS_OP_PONG, p,
						       plen, true);
					continue;
				}
				if (op == (uint8_t)WSS_OP_PONG) {
					continue;
				}
				if (op == (uint8_t)WSS_OP_TEXT) {
					(void)wss_sub_apply(&sub,
							    (const char *)p,
							    plen);
				}
				/* Binary frames carry nothing in this protocol. */
			}
		} else if (pr < 0) {
			break;
		}

		/* Idle disconnect: a dead peer must not hold a worker. */
		if ((now - last_rx_ms) >
		    ((uint64_t)STS_WEB_WS_PING_S * 3U * 1000ULL)) {
			break;
		}
		if ((now - sub.last_ping_ms) >
		    ((uint64_t)STS_WEB_WS_PING_S * 1000ULL)) {
			sub.last_ping_ms = now;
			(void)ws_queue(wk, (uint8_t)WSS_OP_PING, NULL, 0U, true);
		}

		if (now < sub.next_ms) {
			continue;
		}
		sub.next_ms = now + (1000ULL / (sub.rate_hz ? sub.rate_hz : 1U));

		/* Telemetry. The snapshot APIs are lock-free or bounded-mutex;
		 * no timing lock is taken here (ARCHITECTURE.md §10 rule 10). */
		if ((sub.groups & (uint8_t)~WSS_GRP_LOGS) != 0U) {
			int n;

			k_mutex_lock(&api_lock, K_FOREVER);
			rest_ctx.now_ms = now;
			n = rest_encode_telemetry(&rest_ctx,
						  (uint8_t)(sub.groups &
							    (uint8_t)~WSS_GRP_LOGS),
						  ++sub.seq, wk->resp,
						  sizeof(wk->resp));
			k_mutex_unlock(&api_lock);
			if (n > 0 &&
			    ws_send_text(wk, fd, wk->resp, (size_t)n) != 0) {
				break;
			}
		}
		if ((sub.groups & WSS_GRP_LOGS) != 0U) {
			int n;

			k_mutex_lock(&api_lock, K_FOREVER);
			rest_ctx.now_ms = now;
			n = rest_encode_logs(&rest_ctx, &sub.log_cursor, 8U,
					     (uint8_t)LOGR_DEBUG, wk->resp,
					     sizeof(wk->resp));
			k_mutex_unlock(&api_lock);
			if (n > 0 &&
			    ws_send_text(wk, fd, wk->resp, (size_t)n) != 0) {
				break;
			}
		}
	}

done:
	stat_add(&st.ws_closed, 1U);
}

static int ws_handshake(int fd, const http_req_t *req)
{
	char accept[WSS_ACCEPT_LEN + 2U];
	char hdr[352];
	size_t o = 0U;

	if (wss_accept_key(req->ws_key.p, req->ws_key.n, accept,
			   sizeof(accept)) != 0) {
		return send_simple(fd, 400, "text/plain", "bad key\n", NULL);
	}
	o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o,
			      "HTTP/1.1 101 Switching Protocols\r\n"
			      "Upgrade: websocket\r\nConnection: Upgrade\r\n"
			      "Sec-WebSocket-Accept: %s\r\n",
			      accept);
	if (req->ws_protocol.n != 0U &&
	    web_span_eq(req->ws_protocol.p, req->ws_protocol.n,
			"sts.telemetry.v1")) {
		o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o,
				      "Sec-WebSocket-Protocol: "
				      "sts.telemetry.v1\r\n");
	}
	o += (size_t)snprintf(&hdr[o], sizeof(hdr) - o, "\r\n");
	return (send_all(fd, hdr, o) < 0) ? -EIO : 0;
}

/* ========================================================================= */
/* one connection                                                            */
/* ========================================================================= */

/*
 * Read a request head, then its body, then answer. Returns true to keep the
 * connection, false to close it.
 */
static bool serve_request(struct worker *wk, int fd, size_t *have, bool tls)
{
	http_req_t req;
	web_resp_t resp;
	rest_authctx_t actx;
	http_chunked_t chunk;
	const char *body = NULL;
	size_t body_len = 0U;
	/*
	 * A chunked body is decoded into the RESPONSE buffer. The two lifetimes
	 * do not overlap — the body is fully decoded before rest_dispatch() is
	 * called, and the response is only built after that — so this costs no
	 * SRAM. It must be per-worker, not static: two workers decoding chunked
	 * bodies at once would otherwise corrupt each other's request.
	 */
	uint8_t *body_buf = (uint8_t *)wk->resp;
	const size_t body_cap = (sizeof(wk->resp) < HTTP_BODY_MAX)
					? sizeof(wk->resp)
					: HTTP_BODY_MAX;
	int rc;

	rc = http_parse_request((const char *)wk->rx, *have, &req);
	if (rc == -EAGAIN) {
		if (*have >= HTTP_HEAD_MAX) {
			(void)send_simple(fd, 431, "text/plain",
					  "header too large\n", NULL);
			return false;
		}
		return true; /* need more bytes */
	}
	if (rc != 0) {
		stat_add(&st.rejected_oversize, (rc == -E2BIG) ? 1U : 0U);
		(void)send_simple(fd, (rc == -E2BIG) ? 431 : 400, "text/plain",
				  "bad request\n", NULL);
		return false;
	}
	stat_add(&st.requests, 1U);

	/* --- body ------------------------------------------------------ */
	if ((req.flags & HTTP_F_CHUNKED) != 0U) {
		size_t off = req.head_len;
		size_t out_len = 0U;

		http_chunked_init(&chunk, (uint32_t)body_cap);
		for (;;) {
			size_t consumed = 0U;
			int frc = http_chunked_feed(&chunk,
						    (const char *)&wk->rx[off],
						    *have - off, &consumed,
						    body_buf, sizeof(body_buf),
						    &out_len);

			off += consumed;
			if (frc == 1) {
				break;
			}
			if (frc < 0) {
				(void)send_simple(fd,
						  (frc == -E2BIG) ? 413 : 400,
						  "text/plain",
						  "bad chunked body\n", NULL);
				return false;
			}
			if (off >= *have) {
				ssize_t n;

				if (*have >= sizeof(wk->rx)) {
					(void)send_simple(fd, 413, "text/plain",
							  "body too large\n",
							  NULL);
					return false;
				}
				n = zsock_recv(fd, &wk->rx[*have],
					       sizeof(wk->rx) - *have, 0);
				if (n <= 0) {
					return false;
				}
				*have += (size_t)n;
			}
		}
		body = (const char *)body_buf;
		body_len = out_len;
		*have = 0U; /* the pipeline cannot be resynchronised cheaply */
	} else if (req.content_length != 0U) {
		if (req.content_length > body_cap) {
			(void)send_simple(fd, 413, "text/plain",
					  "body too large\n", NULL);
			return false;
		}
		if ((req.flags & HTTP_F_EXPECT_100) != 0U) {
			static const char cont[] = "HTTP/1.1 100 Continue\r\n\r\n";

			(void)send_all(fd, cont, sizeof(cont) - 1U);
		}
		while ((*have - req.head_len) < req.content_length) {
			ssize_t n;

			if (*have >= sizeof(wk->rx)) {
				(void)send_simple(fd, 413, "text/plain",
						  "body too large\n", NULL);
				return false;
			}
			n = zsock_recv(fd, &wk->rx[*have],
				       sizeof(wk->rx) - *have, 0);
			if (n <= 0) {
				return false;
			}
			*have += (size_t)n;
		}
		body = (const char *)&wk->rx[req.head_len];
		body_len = req.content_length;
	}

	/* --- plain-HTTP listener: redirect and nothing else ------------- */
	if (!tls) {
		char loc[192];
		char host[96];

		(void)http_span_str(req.host, host, sizeof(host));
		if (host[0] == '\0') {
			(void)web_span_copy(host, sizeof(host), "localhost", 9U);
		}
		(void)snprintf(loc, sizeof(loc), "Location: https://%s%.*s\r\n",
			       host, (int)req.target.n, req.target.p);
		stat_add(&st.redirects, 1U);
		(void)send_simple(fd, 301, "text/plain",
				  "use https\n", loc);
		return false;
	}

	/* --- WebSocket -------------------------------------------------- */
	if ((req.flags & HTTP_F_UPGRADE_WS) != 0U) {
		bool ok;

		k_mutex_lock(&api_lock, K_FOREVER);
		rest_ctx.now_ms = sts_mono_ms();
		rest_resolve_auth(&rest_ctx, &req, &actx);
		ok = actx.have_session ||
		     !auth_web_required(&auth_ctx);
		k_mutex_unlock(&api_lock);

		if (!ok) {
			(void)send_simple(fd, 401, "text/plain",
					  "authenticate first\n",
					  "WWW-Authenticate: Bearer "
					  "realm=\"meridian\"\r\n");
			return false;
		}
		if (ws_handshake(fd, &req) != 0) {
			return false;
		}
		*have = 0U;
		ws_run(wk, fd, &actx);
		return false;
	}

	/* --- REST ------------------------------------------------------- */
	if (rest_is_api_path(&req)) {
		k_mutex_lock(&api_lock, K_FOREVER);
		rest_ctx.now_ms = sts_mono_ms();
		(void)auth_web_tick(&auth_ctx, rest_ctx.now_ms);
		rest_resp_init(&resp, wk->resp, sizeof(wk->resp));
		(void)rest_dispatch(&rest_ctx, &req, body, body_len, &resp);
		k_mutex_unlock(&api_lock);

		if (resp.status >= 500U) {
			stat_add(&st.responses_5xx, 1U);
		} else if (resp.status >= 400U) {
			stat_add(&st.responses_4xx, 1U);
		} else {
			stat_add(&st.responses_2xx, 1U);
		}
		rc = send_response(fd, &resp,
				   req.method == (uint8_t)WEB_METHOD_HEAD);
		if (rc != 0 || resp.close) {
			return false;
		}
	} else if (req.method == (uint8_t)WEB_METHOD_GET ||
		   req.method == (uint8_t)WEB_METHOD_HEAD) {
		if (serve_static(wk, fd, &req) != 0) {
			return false;
		}
	} else {
		(void)send_simple(fd, 405, "text/plain", "method not allowed\n",
				  "Allow: GET, HEAD\r\n");
		return false;
	}

	if ((req.flags & HTTP_F_KEEPALIVE) == 0U) {
		return false;
	}
	/* Drop anything already read past this request: HTTP pipelining is not
	 * supported, and silently mis-associating a pipelined second request
	 * with this one's authorisation would be a security bug. */
	*have = 0U;
	return true;
}

static void handle_conn(struct worker *wk, int fd, bool tls)
{
	size_t have = 0U;
	uint64_t deadline = sts_mono_ms() +
			    (uint64_t)STS_WEB_IDLE_TIMEOUT_S * 1000ULL;

	for (;;) {
		struct zsock_pollfd pfd = {
			.fd = fd,
			.events = ZSOCK_POLLIN,
			.revents = 0,
		};
		ssize_t n;
		int pr;

		if (sts_mono_ms() > deadline) {
			break;
		}
		pr = zsock_poll(&pfd, 1, 1000);
		if (live_id >= 0) {
			sts_liveness_feed(live_id);
		}
		if (pr == 0) {
			continue;
		}
		if (pr < 0) {
			break;
		}
		if (have >= sizeof(wk->rx)) {
			(void)send_simple(fd, 431, "text/plain",
					  "header too large\n", NULL);
			break;
		}
		n = zsock_recv(fd, &wk->rx[have], sizeof(wk->rx) - have, 0);
		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				continue;
			}
			break;
		}
		if (n == 0) {
			break;
		}
		have += (size_t)n;

		if (!serve_request(wk, fd, &have, tls)) {
			break;
		}
		deadline = sts_mono_ms() +
			   (uint64_t)STS_WEB_IDLE_TIMEOUT_S * 1000ULL;
	}
	(void)zsock_close(fd);
}

/* ========================================================================= */
/* listeners                                                                 */
/* ========================================================================= */

static int open_listener(uint16_t port, bool tls)
{
	static const sec_tag_t tags[] = { STS_WEB_SEC_TAG };
	struct sockaddr_in6 a6;
	int fd;
	int on = 1;
	int rc;

	fd = zsock_socket(AF_INET6, SOCK_STREAM,
			  tls ? IPPROTO_TLS_1_2 : IPPROTO_TCP);
	if (fd < 0) {
		return -errno;
	}
	(void)zsock_setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

	if (tls) {
		rc = zsock_setsockopt(fd, SOL_TLS, TLS_SEC_TAG_LIST, tags,
				      sizeof(tags));
		if (rc < 0) {
			rc = -errno;
			(void)zsock_close(fd);
			return rc;
		}
		/*
		 * A server does not verify a client certificate here: there is no
		 * client PKI on this box and mutual TLS is not in scope for
		 * spec §5.1. Authentication is the session layer's job.
		 */
		{
			int verify = TLS_PEER_VERIFY_NONE;

			(void)zsock_setsockopt(fd, SOL_TLS, TLS_PEER_VERIFY,
					       &verify, sizeof(verify));
		}
	}

	memset(&a6, 0, sizeof(a6));
	a6.sin6_family = AF_INET6;
	a6.sin6_port = htons(port);
	a6.sin6_addr = in6addr_any;

	if (zsock_bind(fd, (struct sockaddr *)&a6, sizeof(a6)) < 0) {
		rc = -errno;
		(void)zsock_close(fd);
		return rc;
	}
	if (zsock_listen(fd, 2) < 0) {
		rc = -errno;
		(void)zsock_close(fd);
		return rc;
	}
	/*
	 * Non-blocking, so several workers may race on accept() and the losers
	 * get EAGAIN instead of parking inside accept(). The flags come from
	 * <zephyr/sys/fdtable.h> (ZVFS_*): zsock_fcntl() is the socket-facing
	 * name for the same call, but the O_/F_ constants live with the fd table,
	 * not in socket.h.
	 */
	(void)zsock_fcntl(fd, ZVFS_F_SETFL, ZVFS_O_NONBLOCK);
	return fd;
}

static void worker_loop(void *a, void *b, void *c)
{
	struct worker *wk = a;

	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		struct zsock_pollfd pfd[2];
		int nfd = 0;
		int tls_idx = -1;
		int plain_idx = -1;
		int pr;

		if (listen_tls >= 0) {
			pfd[nfd].fd = listen_tls;
			pfd[nfd].events = ZSOCK_POLLIN;
			pfd[nfd].revents = 0;
			tls_idx = nfd;
			nfd++;
		}
		if (listen_plain >= 0) {
			pfd[nfd].fd = listen_plain;
			pfd[nfd].events = ZSOCK_POLLIN;
			pfd[nfd].revents = 0;
			plain_idx = nfd;
			nfd++;
		}
		if (nfd == 0) {
			k_sleep(K_SECONDS(1));
			continue;
		}

		pr = zsock_poll(pfd, nfd, 500);
		if (live_id >= 0) {
			sts_liveness_feed(live_id);
		}
		if (pr <= 0) {
			continue;
		}

		if (tls_idx >= 0 && (pfd[tls_idx].revents & ZSOCK_POLLIN) != 0) {
			int cfd = zsock_accept(listen_tls, NULL, NULL);

			if (cfd >= 0) {
				stat_add(&st.accepted, 1U);
				/*
				 * Zephyr completes the TLS handshake inside the
				 * first recv() on the accepted socket, so a
				 * receive timeout here is the handshake timeout.
				 */
				{
					struct zsock_timeval tv = {
						.tv_sec = STS_WEB_HANDSHAKE_TIMEOUT_MS /
							  1000,
						.tv_usec = 0,
					};

					(void)zsock_setsockopt(cfd, SOL_SOCKET,
							       SO_RCVTIMEO, &tv,
							       sizeof(tv));
				}
				handle_conn(wk, cfd, true);
			}
		}
		if (plain_idx >= 0 &&
		    (pfd[plain_idx].revents & ZSOCK_POLLIN) != 0) {
			int cfd = zsock_accept(listen_plain, NULL, NULL);

			if (cfd >= 0) {
				handle_conn(wk, cfd, false);
			}
		}
	}
}

/* ========================================================================= */
/* start-up                                                                  */
/* ========================================================================= */

static int web_bring_up(void)
{
	int rc;
	unsigned int i;

	k_mutex_init(&api_lock);
	providers_bind();

	rc = auth_kdf_hmac_sha256(&auth_kdf, sts_port_crypto());
	if (rc != 0) {
		LOG_ERR("no HMAC in the crypto port; web auth impossible");
		return rc;
	}
	rc = auth_web_init(&auth_ctx, sts_port_crypto(), &auth_kdf, sts_cfg(),
			   sts_logring());
	if (rc != 0) {
		return rc;
	}
	/*
	 * All three roles, each bound to a cfg key of its own, each holding the
	 * SAME {salt,HMAC} blob core/mcp uses (see auth_web.h). `admin` shares
	 * sec.admin.pw with the console deliberately; operator and viewer got
	 * schema keys of their own (sec.operator.pw, sec.viewer.pw) so they are
	 * real persistent accounts rather than the RAM-only ghosts they would
	 * have had to be before.
	 *
	 * They ship UNPROVISIONED, which fails closed: an account with no
	 * credential cannot be logged into, and — once any account is set up —
	 * is refused with exactly the answer a wrong password gets, so the
	 * login reply does not report which of the three have been commissioned.
	 * An administrator provisions them over POST /api/v1/security/password.
	 */
	static const struct {
		const char *name;
		uint8_t     role;
		uint16_t    key;
	} accounts[] = {
		{ "admin", (uint8_t)WEB_ROLE_ADMIN,
		  (uint16_t)CFG_ID_SEC_ADMIN_PW },
		{ "operator", (uint8_t)WEB_ROLE_OPERATOR,
		  (uint16_t)CFG_ID_SEC_OPERATOR_PW },
		{ "viewer", (uint8_t)WEB_ROLE_VIEWER,
		  (uint16_t)CFG_ID_SEC_VIEWER_PW },
	};

	for (i = 0U; i < ARRAY_SIZE(accounts); i++) {
		rc = auth_web_user_add(&auth_ctx, accounts[i].name,
				       accounts[i].role, accounts[i].key);
		if (rc < 0) {
			LOG_ERR("account '%s': %d", accounts[i].name, rc);
			return rc;
		}
	}
	if (auth_web_reload(&auth_ctx) == 0) {
		LOG_WRN("no management credential provisioned; the web plane "
			"is read-only until one is set over the console or the "
			"local UI");
	}

	/*
	 * The remote authority. Without this the web plane could reach only the
	 * local blob, so RADIUS / TACACS+ / LDAP — built, host-tested and
	 * configured through cfg group 0x0A — were reachable from exactly one of
	 * the three management surfaces.
	 *
	 * Wired to sts_aaa_check_fed_budgeted(), NOT sts_aaa_check(): this runs
	 * on a web worker, under api_lock, and the workers share one liveness
	 * participant — so it needs both the offload thread and a budget an
	 * anonymous caller cannot exceed. See web_remote_auth() and
	 * sts_secops_policy.h.
	 */
	(void)auth_web_set_remote(&auth_ctx, web_remote_auth, NULL);

	rc = rest_init(&rest_ctx, sts_cfg(), sts_logring(), &auth_ctx,
		       &providers);
	if (rc != 0) {
		return rc;
	}
	rest_ctx.commit = commit_via_app;
	rest_ctx.commit_u = NULL;

	(void)sts_cfg_register_applier(CFG_G_SEC, cfg_applied, NULL);

	sts_webfs_init();

	rc = sts_cert_init();
	if (rc != 0) {
		LOG_ERR("no TLS credential (%d); HTTPS not started", rc);
		return rc;
	}
	K_SPINLOCK(&st_lock) {
		st.tls_ready = 1U;
	}

	listen_tls = open_listener(STS_WEB_TLS_PORT, true);
	if (listen_tls < 0) {
		LOG_ERR("HTTPS listen on :%u failed: %d", STS_WEB_TLS_PORT,
			listen_tls);
		return listen_tls;
	}
#if STS_WEB_REDIRECT_PORT != 0
	listen_plain = open_listener(STS_WEB_REDIRECT_PORT, false);
	if (listen_plain < 0) {
		LOG_WRN("HTTP redirect listener on :%u failed: %d",
			STS_WEB_REDIRECT_PORT, listen_plain);
		listen_plain = -1;
	}
#endif

	live_id = sts_liveness_register("web");
	for (i = 0U; i < STS_WEB_WORKERS; i++) {
		wss_txq_init(&workers[i].txq);
		k_thread_create(&worker_threads[i], worker_stacks[i],
				K_THREAD_STACK_SIZEOF(worker_stacks[i]),
				worker_loop, &workers[i], NULL, NULL,
				STS_WEB_PRIORITY, 0, K_NO_WAIT);
		(void)k_thread_name_set(&worker_threads[i], "web");
	}

	K_SPINLOCK(&st_lock) {
		st.running = true;
	}
	LOG_INF("HTTPS on :%u (%u worker(s), SPA %s)", STS_WEB_TLS_PORT,
		(unsigned int)STS_WEB_WORKERS,
		sts_webfs_have_bundle() ? "from /lfs/www" : "fallback page");
	return 0;
}

/*
 * The supervisor thread. It waits for the platform and net areas to come up,
 * brings the listener online, and then does nothing but retry if that failed —
 * a box whose NOR was not mounted at first attempt gets another chance rather
 * than losing its management plane until the next reboot.
 */
static void web_supervisor(void *a, void *b, void *c)
{
	unsigned int tries = 0U;

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	/* Wait for the platform's cfg context and for an interface. */
	while (sts_cfg() == NULL || sts_net_iface() == NULL) {
		k_sleep(K_MSEC(250));
	}
	/* Let the link settle so the first bind sees an address family up. */
	k_sleep(K_SECONDS(1));

	for (;;) {
		if (web_bring_up() == 0) {
			return;
		}
		tries++;
		if (tries >= 10U) {
			LOG_ERR("giving up on the HTTPS listener after %u "
				"attempts; the console remains available",
				tries);
			return;
		}
		LOG_WRN("HTTPS bring-up attempt %u failed; retrying", tries);
		k_sleep(K_SECONDS(5));
	}
}

K_THREAD_DEFINE(sts_web_sup, 2048, web_supervisor, NULL, NULL, NULL,
		STS_WEB_PRIORITY, 0, 1500);
