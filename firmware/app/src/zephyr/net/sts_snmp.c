/*
 * STS1000 "Meridian" — SNMPv2c/v3 agent glue + trap sender (spec §5.3, prio 12).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * core/snmp is the codec and the walk; this file is the socket, the value
 * getter, and the trap sender.
 *
 * ---------------------------------------------------------------------------
 * SNMPv3 / USM
 * ---------------------------------------------------------------------------
 *
 * Spec §5.3 makes SNMPv3/USM the primary interface and v2c an explicit opt-in,
 * and core/snmp already implements both behind one PDU layer. The receive path
 * therefore goes through snmp_dispatch(), which routes by msgVersion; passing it
 * a NULL v3 context is its documented way of making every v3 datagram a counted
 * drop, which is exactly what `sec.snmpv3.en = 0` should mean. Calling
 * snmp_handle() directly — as this file used to — refused every v3 datagram and
 * left the whole USM implementation unreachable.
 *
 * Three things the engine needs and only the glue can supply:
 *
 *   engine id     RFC 3411 §5 from SNMP_PEN plus a device-unique value: the
 *                 ATECC608B serial when the part answers, else the SoC device
 *                 id. Which one was used is logged, because a software-rooted
 *                 identity is unique per die but not tamper-resistant. If the
 *                 id cannot be built, v3 stays off — two boards sharing an
 *                 engine id present to a manager as random authentication
 *                 errors months later (snmp_v3.h).
 *   engineBoots   RFC 3414 §2.2.2: >= 1 once the engine has ever run, and
 *                 strictly increasing across restarts, or a manager's 150 s
 *                 timeliness window accepts messages replayed from a previous
 *                 boot. Kept in cfg (`sec.snmpv3.boots`), incremented and
 *                 persisted here. 2147483647 is terminal: the RFC requires a
 *                 new engine id at that point, so v3 refuses to start rather
 *                 than wrap into a count a manager has already seen.
 *   engineTime    Seconds since *this engine* booted — NOT sysUpTime. The two
 *                 differ by however long bring-up took, and against a 150 s
 *                 window that is not academic, so it is measured from its own
 *                 monotonic base taken at snmp_v3_init().
 *
 * Users come from `sec.snmpv3.uN.*`; `sec.snmpv3.local` says whether the stored
 * key blobs are already-localised keys or passphrases. A failed slot is logged
 * with its name and errno and does not take the other slot down with it.
 *
 * ---------------------------------------------------------------------------
 * Where the values come from — and what this area cannot reach
 * ---------------------------------------------------------------------------
 *
 * The getter resolves every leaf from data this area can legitimately read
 * through sts_app.h (ARCHITECTURE.md §2 forbids reaching into another area's
 * private state):
 *
 *   timing / gnss / osc-temp / leap : the §3.8 quality snapshot
 *   ntp / nts / ptp / net counters  : this area's own service stats
 *   alarms / uptime                 : sts_alarms_active(), k_uptime
 *   sysDescr / sysName              : build constants + net.hostname
 *
 * The platform-owned health metrics — the nine INA228 rails, the enclosure and
 * die temperatures, humidity, fan RPM/duty, PoE class/draw, supercap voltages —
 * are NOT exposed through sts_app.h as scalars (only as opaque MCP STATUS_GET
 * blobs), so the getter returns "no value" for them and core/snmp answers
 * noSuchInstance. That is a truthful SNMPv2 response, and the OIDs still walk.
 *
 * TODO(cross-area telemetry): add an sts_app.h accessor for the §10.5 health
 * scalars (a small read-only struct, or per-metric getters) so the rail/thermal
 * columns resolve. Until then a manager sees the timing MIB fully and the power
 * MIB as present-but-unpopulated. This is the one material scope gap in the
 * SNMP surface and it is a boundary limitation, not a codec one.
 *
 * ---------------------------------------------------------------------------
 * Traps
 * ---------------------------------------------------------------------------
 *
 * sts_app.h exposes no event hook, so transitions are derived by polling:
 * successive quality snapshots give lock acquired/lost and holdover enter/exit
 * and reference switch; the alarm bitmask diff gives rail/thermal/antenna
 * alarms. snmp_dispatch() returning -EACCES (a bad community, or a v3 digest
 * that did not verify on a non-reportable request) offers an
 * authentication-failure trap to snmp_notify_gate(), which coalesces it. A
 * coldStart is sent once at start. The queue is a small ring so a burst cannot
 * block the producer.
 *
 * Notifications go out as USM messages when `sec.snmpv3.trap.user` names a
 * provisioned user, at the level in `sec.snmpv3.trap.lvl`; otherwise as v2c
 * traps. See send_trap() for why a *configured but broken* trap user is not
 * silently downgraded to v2c.
 *
 * Enqueueing one authentication-failure trap per bad-community datagram, as this
 * file used to, turned an attack on this box into an attack on the trap
 * receiver, and buried every genuine event behind the flood — each one also
 * carrying a LOG_INF into the RAM ring and the NOR spool (F9).
 *
 * ---------------------------------------------------------------------------
 * Source ACL (spec §5.3: "SNMPv2c optional behind ACL")
 * ---------------------------------------------------------------------------
 *
 * cfg key `net.mgmt.acl` gates who may reach the agent at all. It existed in the
 * schema and was read by nothing anywhere in the tree, so the agent answered the
 * whole Internet — and a 36-octet spoofed GETBULK draws a 1391-octet response,
 * a 38.6x reflector aimed at whatever source address the attacker writes (F9).
 *
 * The key is a 32-octet BLOB, so this file defines a compact binary encoding —
 * a list of CIDR prefixes, each:
 *
 *     kind(1) | address(4 or 16, network order) | prefix_len(1)
 *     kind 4 = IPv4 (6 octets total), kind 6 = IPv6 (18 octets total)
 *
 * 32 octets therefore hold five IPv4 prefixes, or one IPv6 plus two IPv4. A
 * trailing zero octet ends the list, as does the end of the blob. An empty blob
 * means "no ACL configured": every source is allowed, which is the historical
 * behaviour and is why start-up logs a warning when the agent is enabled without
 * one. The agent itself is disabled by default, and core/snmp refuses v2c by
 * default, so a shipped box is not exposed by that default.
 *
 * TODO(cross-area): the encoding belongs in cfg_schema.h's comment for
 * CFG_ID_NET_MGMT_ACL, and the 32-octet cap is tight for an IPv6 deployment.
 * Both are core/cfg's to change.
 */

#include <errno.h>
#include <string.h>

#include <mbedtls/platform_util.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>

#include "cfg/cfg.h"
#include "fault/fault.h"
#include "net/sts_net.h"
#include "quality/quality.h"
#include "snmp/snmp.h"
#include "snmp/snmp_v3.h"
/*
 * Second inbound exception to the net-area boundary, alongside
 * storage/sts_store.h (see sts_net.h): the device-unique value behind the RFC
 * 3411 engine id has exactly one correct source on this board and it is the
 * secure element. sts_atecc.h is a thread-safe façade whose every entry point
 * degrades to the software path, so consuming it costs no ordering discipline.
 */
#include "storage/sts_atecc.h"
#include "storage/sts_store.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_snmp, CONFIG_STS1000_LOG_LEVEL);

#define SNMP_STACK_SIZE 4096
#define SNMP_PRIORITY 12
#define SNMP_AGENT_PORT 161
#define SNMP_POLL_MS 500
#define TRAP_QUEUE_LEN 16U

/**
 * Datagrams handled per pass over one ready socket before yielding.
 *
 * Same reasoning as the NTP and PTP loops: zsock_poll() returns immediately
 * while data is queued and Zephyr does not timeslice across priorities, so an
 * uncapped drain lets an SNMP flood starve `housekeeping` and cold-cycle the
 * board (F12). Small, because each iteration can walk the whole MIB.
 */
#define SNMP_RX_BUDGET 4U

/**
 * Fallback minimum gap between two notifications of the same type, ms.
 *
 * The live value is cfg key `sec.snmp.notify.ms` (schema default 5000); this is
 * only what an unreadable key falls back to. Coalescing turns an
 * authentication-failure flood into one trap per interval carrying a suppressed
 * count, while still delivering a genuine lockLost promptly — the gate is per
 * notification type, so the two do not compete.
 */
#define SNMP_NOTIFY_MIN_INTERVAL_MS 5000U

/**
 * Terminal snmpEngineBoots (RFC 3414 §2.2.2), and the schema max of
 * `sec.snmpv3.boots`. Reaching it is legal; exceeding it requires a new engine
 * id, which this firmware cannot mint on its own.
 */
#define V3_BOOTS_MAX 2147483647U

/**
 * Largest snmpEngineTime the RFC allows. Reaching it should roll boots and reset
 * the clock; at one second per second that is 68 years of unbroken uptime, so it
 * is clamped instead and the roll is deliberately not implemented.
 */
#define V3_ENGINE_TIME_MAX 2147483647U

/** Bytes of `net.mgmt.acl` blob (the cfg schema's declared size). */
#define SNMP_ACL_MAX 32U

/* net.mgmt.acl entry encoding; see the file header. */
#define ACL_KIND_END 0U
#define ACL_KIND_V4 4U
#define ACL_KIND_V6 6U
#define ACL_ENTRY_V4_LEN 6U
#define ACL_ENTRY_V6_LEN 18U

/* Firmware identity strings for the system group. */
#define SNMP_SYS_DESCR "STS1000 Meridian GPS-disciplined Stratum-1 NTP/PTP grandmaster"
#define SNMP_FW_VERSION "1.0.0-dev"

static snmp_ctx_t agent;

/*
 * Double-buffered community string (F10).
 *
 * sts_snmp_reapply() runs on whichever thread committed the configuration, and
 * it used to rewrite the single static buffer that the SNMP thread was
 * concurrently memcmp-ing inside snmp_handle() — core/snmp borrows the pointer
 * rather than copying it. A commit could therefore make a request compare
 * against a half-written string (accept the wrong community, or reject the right
 * one) and, when the new value was empty, hand core a NULL mid-request.
 *
 * So: build the new value in the buffer that is NOT live, then publish the
 * pointer in one store. A reader either sees the old buffer or the new one, and
 * both are complete NUL-terminated strings at all times.
 */
static char community_buf[2][SNMP_COMMUNITY_MAX + 1U];
static uint8_t community_live;
static char hostname[64];

/*
 * SNMPv3 engine. File-scope static and touched by exactly one thread.
 *
 * snmp_v3_ctx_t carries a full-datagram scratch buffer — a privacy-protected
 * request has to be decrypted somewhere and the received datagram is const — so
 * it is ~1.6 KiB, far too large for the 4 kB SNMP thread stack, and its header
 * states that one context serves one thread because two would share that
 * scratch. Only snmp_loop() reaches it: snmp_dispatch() from serve_one() and
 * snmp_v3_make_notification() from send_trap(), both on that thread.
 */
static snmp_v3_ctx_t v3;
static uint8_t v3_engine_id[SNMP_V3_ENGINEID_MAX];
static bool v3_ready;

/**
 * Monotonic base for snmpEngineTime, taken at snmp_v3_init().
 *
 * snmpEngineTime is seconds since *this engine* booted, which sts_net_uptime_cs()
 * is not: that counts hundredths from kernel boot, so reusing it would overstate
 * the engine's age by however long bring-up took. RFC 3414 §2.2.3's timeliness
 * window is 150 s wide, so the difference is a real interoperability error and
 * not a cosmetic one.
 */
static uint64_t v3_boot_ms;

/*
 * v3 notification target, double-buffered for the same reason as the community
 * (F10): send_trap() runs on the SNMP thread and hands this pointer straight to
 * snmp_v3_make_notification(), which borrows it for a user lookup, while the
 * cfg applier runs on whichever thread committed. Rewriting the live buffer in
 * place would let a lookup see a half-written name — matching nobody, or the
 * wrong user. `trap_level` is a single byte and `trap_v3_warned` a single bool,
 * so both publish in one store.
 */
static char trap_user_buf[2][SNMP_V3_USER_MAX + 1U];
static uint8_t trap_user_live;
static uint8_t trap_level = (uint8_t)SNMP_SEC_AUTH_NOPRIV;
static bool trap_v3_warned;

static uint8_t acl[SNMP_ACL_MAX];
static size_t acl_len;
static bool acl_deny_all; /* the configured value could not be parsed */
static uint32_t acl_refused; /* datagrams dropped by the source ACL */
static struct k_spinlock acl_lock;

static int sock4 = -1;
static int sock6 = -1;
static uint8_t rx[SNMP_PKT_MAX];
static uint8_t tx[SNMP_PKT_MAX];

/* trap destination */
static int trap_sock = -1;
static struct sockaddr_storage trap_dest;
static socklen_t trap_dest_len;
static bool trap_resolved;
static uint8_t trap_buf[SNMP_PKT_MAX];

/* trap queue */
static snmp_trap_t trap_q[TRAP_QUEUE_LEN];
static uint8_t trap_head;
static uint8_t trap_tail;
static struct k_spinlock trap_lock;

/* transition tracking */
static bool have_prev;
static uint8_t prev_lock;
static uint8_t prev_ref;
static bool prev_holdover;
static uint64_t prev_alarms;

static bool started;
static K_THREAD_STACK_DEFINE(snmp_stack, SNMP_STACK_SIZE);
static struct k_thread snmp_thread;
static int live_id = -1;

/* ------------------------------------------------------------------------- */
/* getter                                                                    */
/* ------------------------------------------------------------------------- */

/** Clamp a float to a non-negative Gauge32, treating NaN/inf as 0. */
static uint32_t gauge_of_float(float f)
{
	if (!(f == f) || f < 0.0f) { /* NaN or negative */
		return 0U;
	}
	if (f > 4.294967e9f) {
		return 0xFFFFFFFFU;
	}
	return (uint32_t)f;
}

static int getter(void *ctx, uint16_t obj, uint16_t inst, snmp_value_t *out)
{
	quality_block_t q;
	sts_ntp_stats_t ns;
	sts_ptp_stats_t ps;
	bool have_q;

	ARG_UNUSED(ctx);
	ARG_UNUSED(inst);

	have_q = (sts_quality_snapshot(&q) == 0);

	switch (obj) {
	/* -- system group ------------------------------------------------- */
	case SNMP_OBJ_SYS_DESCR:
		snmp_val_str(out, SNMP_SYS_DESCR);
		return 0;
	case SNMP_OBJ_SYS_OBJECT_ID: {
		/* sysObjectID.0 = the vendor's registration OID, i.e. the
		 * enterprise root this MIB hangs under (1.3.6.1.4.1.<PEN>.1). */
		static const uint32_t objid[] = { 1U, 3U, 6U, 1U, 4U,
						  1U, SNMP_PEN, 1U };

		snmp_val_oid(out, objid, sizeof(objid) / sizeof(objid[0]));
		return 0;
	}
	case SNMP_OBJ_SYS_NAME:
		snmp_val_str(out, hostname[0] ? hostname : "meridian");
		return 0;
	case SNMP_OBJ_SYS_CONTACT:
	case SNMP_OBJ_SYS_LOCATION:
		snmp_val_str(out, "");
		return 0;
	case SNMP_OBJ_SYS_SERVICES:
		/* RFC 3418: bit for the highest layer offered. Application (7)
		 * -> 1<<(7-1) = 64, plus transport (4) -> 8. */
		snmp_val_int(out, 72);
		return 0;

	/* -- timing (from quality) ---------------------------------------- */
	case SNMP_OBJ_STRATUM:
		snmp_val_int(out, have_q ? q.stratum : 16);
		return 0;
	case SNMP_OBJ_LOCK_STATE:
		snmp_val_int(out, have_q ? q.lock_state : 0);
		return 0;
	case SNMP_OBJ_ACTIVE_REF:
		snmp_val_int(out, have_q ? q.active_ref : 0);
		return 0;
	case SNMP_OBJ_HOLDOVER:
		snmp_val_int(out, (have_q && q.holdover) ? 1 : 0);
		return 0;
	case SNMP_OBJ_HOLDOVER_EST_ERR_NS:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_int(out, (int64_t)q.holdover_est_err_ns);
		return 0;
	case SNMP_OBJ_HOLDOVER_ELAPSED_S:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_uint(out, SNMP_TAG_GAUGE32, q.holdover_elapsed_s);
		return 0;
	case SNMP_OBJ_HOLDOVER_DEMOTE_S:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_uint(out, SNMP_TAG_GAUGE32, q.holdover_t_demote_s);
		return 0;
	case SNMP_OBJ_PPS_OFFSET_NS:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_int(out, q.last_pps_off_ns);
		return 0;
	case SNMP_OBJ_PPS_MEAN_NS:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_int(out, (int64_t)q.pps_off_mean_ns);
		return 0;
	case SNMP_OBJ_PPS_SIGMA_NS:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_uint(out, SNMP_TAG_GAUGE32,
			      gauge_of_float(q.pps_off_sigma_ns));
		return 0;
	case SNMP_OBJ_FREQ_ERR_PPT:
		if (!have_q) {
			return -ENOENT;
		}
		/* ppb * 1000 = parts-per-trillion, so a sub-ppb figure is not
		 * quantised to zero by the integer transport. */
		snmp_val_int(out, (int64_t)(q.freq_err_ppb * 1000.0f));
		return 0;
	case SNMP_OBJ_VC_CMD_MV:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_int(out, q.vc_cmd_mv);
		return 0;
	case SNMP_OBJ_VC_SENSE_MV:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_int(out, q.vc_sense_mv);
		return 0;
	case SNMP_OBJ_ADEV_1S_E18:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_uint(out, SNMP_TAG_GAUGE32,
			      gauge_of_float(q.adev_1s * 1e18f));
		return 0;
	case SNMP_OBJ_ADEV_10S_E18:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_uint(out, SNMP_TAG_GAUGE32,
			      gauge_of_float(q.adev_10s * 1e18f));
		return 0;
	case SNMP_OBJ_ADEV_100S_E18:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_uint(out, SNMP_TAG_GAUGE32,
			      gauge_of_float(q.adev_100s * 1e18f));
		return 0;
	case SNMP_OBJ_OSC_TEMP_MC:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_int(out, q.osc_temp_mc);
		return 0;

	/* -- gnss (from quality) ------------------------------------------ */
	case SNMP_OBJ_GNSS_FIX:
		snmp_val_int(out, have_q ? q.gnss_fix : 0);
		return 0;
	case SNMP_OBJ_GNSS_SV_USED:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_uint(out, SNMP_TAG_GAUGE32, q.gnss_sv_used);
		return 0;
	case SNMP_OBJ_GNSS_SV_VISIBLE:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_uint(out, SNMP_TAG_GAUGE32, q.gnss_sv_visible);
		return 0;
	case SNMP_OBJ_GNSS_TACC_NS:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_uint(out, SNMP_TAG_GAUGE32, q.gnss_tacc_ns);
		return 0;
	case SNMP_OBJ_GNSS_LEAP_PENDING:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_int(out, q.leap_pending);
		return 0;
	case SNMP_OBJ_GNSS_LEAP_CURRENT_S:
		if (!have_q) {
			return -ENOENT;
		}
		snmp_val_int(out, q.leap_current_s);
		return 0;

	/* -- time services (this area's own counters) --------------------- */
	case SNMP_OBJ_NTP_RX:
	case SNMP_OBJ_NTP_SERVED:
	case SNMP_OBJ_NTP_DROPPED:
	case SNMP_OBJ_NTP_KOD:
	case SNMP_OBJ_NTP_AUTH_FAIL:
	case SNMP_OBJ_NTP_RATE_LIMITED:
	case SNMP_OBJ_NTS_RX:
	case SNMP_OBJ_NTS_OK:
	case SNMP_OBJ_NTS_NAK:
	case SNMP_OBJ_NTS_COOKIES:
		sts_ntp_stats(&ns);
		switch (obj) {
		case SNMP_OBJ_NTP_RX:
			snmp_val_uint(out, SNMP_TAG_COUNTER64, ns.ntp.rx);
			break;
		case SNMP_OBJ_NTP_SERVED:
			snmp_val_uint(out, SNMP_TAG_COUNTER64, ns.ntp.served);
			break;
		case SNMP_OBJ_NTP_DROPPED:
			snmp_val_uint(out, SNMP_TAG_COUNTER64, ns.ntp.dropped);
			break;
		case SNMP_OBJ_NTP_KOD:
			snmp_val_uint(out, SNMP_TAG_COUNTER64, ns.ntp.kod);
			break;
		case SNMP_OBJ_NTP_AUTH_FAIL:
			snmp_val_uint(out, SNMP_TAG_COUNTER64,
				      ns.ntp.auth_fail);
			break;
		case SNMP_OBJ_NTP_RATE_LIMITED:
			snmp_val_uint(out, SNMP_TAG_COUNTER64,
				      ns.ntp.rate_limited);
			break;
		case SNMP_OBJ_NTS_RX:
			snmp_val_uint(out, SNMP_TAG_COUNTER64, ns.nts.rx);
			break;
		case SNMP_OBJ_NTS_OK:
			snmp_val_uint(out, SNMP_TAG_COUNTER64, ns.nts.ok);
			break;
		case SNMP_OBJ_NTS_NAK:
			snmp_val_uint(out, SNMP_TAG_COUNTER64, ns.nts.nak);
			break;
		case SNMP_OBJ_NTS_COOKIES:
			snmp_val_uint(out, SNMP_TAG_COUNTER64,
				      ns.nts.cookies_issued);
			break;
		default:
			break;
		}
		return 0;
	case SNMP_OBJ_NTSKE_HANDSHAKES: {
		sts_ntske_stats_t ks;

		sts_ntske_stats(&ks);
		snmp_val_uint(out, SNMP_TAG_COUNTER64, ks.handshakes_ok);
		return 0;
	}

	/* -- ptp (this area's engine) ------------------------------------- */
	case SNMP_OBJ_PTP_PORT_STATE:
	case SNMP_OBJ_PTP_CLOCK_CLASS:
	case SNMP_OBJ_PTP_CLOCK_ACCURACY:
	case SNMP_OBJ_PTP_DOMAIN:
	case SNMP_OBJ_PTP_TX:
	case SNMP_OBJ_PTP_RX:
	case SNMP_OBJ_PTP_ANN_TIMEOUTS:
	case SNMP_OBJ_PTP_ALARMS: {
		uint32_t tx_sum = 0U;
		uint32_t rx_sum = 0U;
		size_t i;

		sts_ptp_stats(&ps);
		for (i = 0U; i < PTP_MSG_TYPE_COUNT; i++) {
			tx_sum += ps.counters.tx[i];
			rx_sum += ps.counters.rx[i];
		}
		switch (obj) {
		case SNMP_OBJ_PTP_PORT_STATE:
			snmp_val_int(out, ps.port_state);
			break;
		case SNMP_OBJ_PTP_CLOCK_CLASS:
			snmp_val_uint(out, SNMP_TAG_GAUGE32, ps.clock_class);
			break;
		case SNMP_OBJ_PTP_CLOCK_ACCURACY:
			snmp_val_uint(out, SNMP_TAG_GAUGE32,
				      ps.clock_accuracy);
			break;
		case SNMP_OBJ_PTP_DOMAIN:
			snmp_val_uint(out, SNMP_TAG_GAUGE32, ps.domain);
			break;
		case SNMP_OBJ_PTP_TX:
			snmp_val_uint(out, SNMP_TAG_COUNTER64, tx_sum);
			break;
		case SNMP_OBJ_PTP_RX:
			snmp_val_uint(out, SNMP_TAG_COUNTER64, rx_sum);
			break;
		case SNMP_OBJ_PTP_ANN_TIMEOUTS:
			snmp_val_uint(out, SNMP_TAG_COUNTER32,
				      ps.counters.announce_timeouts);
			break;
		case SNMP_OBJ_PTP_ALARMS:
			snmp_val_uint(out, SNMP_TAG_GAUGE32, ps.alarms);
			break;
		default:
			break;
		}
		return 0;
	}

	/* -- health ------------------------------------------------------- */
	case SNMP_OBJ_ALARMS:
		snmp_val_uint(out, SNMP_TAG_GAUGE32,
			      (uint32_t)(sts_alarms_active() & 0xFFFFFFFFU));
		return 0;
	case SNMP_OBJ_FW_VERSION:
		snmp_val_str(out, SNMP_FW_VERSION);
		return 0;
	case SNMP_OBJ_UPTIME_S:
		snmp_val_uint(out, SNMP_TAG_GAUGE32,
			      (uint32_t)(k_uptime_get() / 1000));
		return 0;

	/* -- platform-owned health scalars: not reachable from this area -- */
	default:
		/* Includes the rail table, enclosure/die temp, humidity, fan,
		 * PoE, supercaps, gnss antenna/fw, serial — see the file
		 * comment's TODO(cross-area telemetry). */
		return -ENOENT;
	}
}

/* ------------------------------------------------------------------------- */
/* SNMPv3 / USM engine                                                       */
/* ------------------------------------------------------------------------- */

/** snmpEngineTime: seconds since snmp_v3_init(). See v3_boot_ms. */
static uint32_t engine_time_s(void)
{
	uint64_t s = (sts_mono_ms() - v3_boot_ms) / 1000U;

	return (s > (uint64_t)V3_ENGINE_TIME_MAX) ? V3_ENGINE_TIME_MAX
						  : (uint32_t)s;
}

/**
 * Publish the v3 notification user and level from cfg.
 *
 * The name goes into whichever buffer is not live and the pointer is published
 * by one store of `trap_user_live`; a reader sees the old complete string or the
 * new one (F10).
 */
static void publish_trap_user(void)
{
	uint8_t next = (uint8_t)(trap_user_live ^ 1U);
	uint8_t lvl;

	(void)sts_net_cfg_str(CFG_ID_SEC_SNMPV3_TRAP_USER, trap_user_buf[next],
			      sizeof(trap_user_buf[next]));
	trap_user_live = next;

	lvl = (uint8_t)sts_net_cfg_u64(CFG_ID_SEC_SNMPV3_TRAP_LVL,
				       (uint64_t)SNMP_SEC_AUTH_NOPRIV);
	if (lvl > (uint8_t)SNMP_SEC_AUTH_PRIV) {
		LOG_ERR("sec.snmpv3.trap.lvl %u is not a USM security level; "
			"using authNoPriv", (unsigned int)lvl);
		lvl = (uint8_t)SNMP_SEC_AUTH_NOPRIV;
	}
	trap_level = lvl;

	/* A retargeted trap user earns a fresh diagnostic (see send_trap). */
	trap_v3_warned = false;
}

/**
 * Read, increment and persist snmpEngineBoots (RFC 3414 §2.2.2).
 *
 * @retval 0       @p out holds the value to run this engine with.
 * @retval -EPERM  The counter is exhausted; v3 must not start.
 */
static int v3_next_boots(uint32_t *out)
{
	uint64_t boots = 0U;
	int rc;

	(void)cfg_get_u64(sts_cfg(), CFG_ID_SEC_SNMPV3_BOOTS, &boots);
	if (boots >= V3_BOOTS_MAX) {
		LOG_ERR("sec.snmpv3.boots has reached %u; RFC 3414 §2.2.2 requires "
			"a new engine id before it may be reset, so SNMPv3 stays "
			"off rather than reuse a boot count a manager has already "
			"cached", (unsigned int)V3_BOOTS_MAX);
		return -EPERM;
	}
	boots++;
	*out = (uint32_t)boots;

	/*
	 * Stage under the cfg mutex, commit outside it: sts_cfg_commit() takes
	 * the mutex itself and then dispatches the group appliers with it
	 * released. Those appliers re-enter this area (sts_snmp_on_cfg_sec() is
	 * a no-op until `started`), so the ordering matters.
	 */
	sts_cfg_lock();
	rc = cfg_set_u64(sts_cfg(), CFG_ID_SEC_SNMPV3_BOOTS, boots);
	sts_cfg_unlock();
	if (rc == 0) {
		rc = sts_cfg_commit(NULL);
	}

	/*
	 * A failure here is annunciated but not fatal. Refusing to serve v3 on a
	 * box whose NVS is unavailable would take away the primary management
	 * interface (spec §5.3) over a replay window that only matters for the
	 * first 150 s after a reset — but the operator has to be told, because the
	 * next boot will reuse this count.
	 */
	if (!sts_cfg_is_persistent()) {
		LOG_WRN("snmpEngineBoots %u: no persistent cfg store this boot, so "
			"the count repeats across the next reset and a manager may "
			"accept a message replayed from this one",
			(unsigned int)boots);
	} else if (rc != 0) {
		LOG_ERR("snmpEngineBoots %u could not be persisted (%d); it will "
			"repeat across the next reset", (unsigned int)boots, rc);
	}
	return 0;
}

/**
 * Load one USM user slot from cfg.
 *
 * A slot with an empty name is unconfigured and skipped. Everything else is
 * reported: a slot that fails is the difference between "the manager cannot
 * poll" and "the manager cannot poll and nothing said why", and one bad slot
 * must not take the other down with it.
 */
static void v3_load_user(uint16_t id_name, uint16_t id_auth, uint16_t id_akey,
			 uint16_t id_priv, uint16_t id_pkey, bool localized)
{
	char name[SNMP_V3_USER_MAX + 1U];
	uint8_t akey[CFG_VAL_MAX];
	uint8_t pkey[CFG_VAL_MAX];
	size_t akey_len = 0U;
	size_t pkey_len = 0U;
	uint8_t auth;
	uint8_t priv;
	int rc;

	if (sts_net_cfg_str(id_name, name, sizeof(name)) == 0U) {
		return;
	}

	auth = (uint8_t)sts_net_cfg_u64(id_auth,
					(uint64_t)SNMP_AUTH_HMAC_SHA256_192);
	priv = (uint8_t)sts_net_cfg_u64(id_priv, (uint64_t)SNMP_PRIV_AES128_CFB);

	if (cfg_get_bytes(sts_cfg(), id_akey, akey, sizeof(akey), &akey_len) != 0) {
		akey_len = 0U;
	}
	if (cfg_get_bytes(sts_cfg(), id_pkey, pkey, sizeof(pkey), &pkey_len) != 0) {
		pkey_len = 0U;
	}

	if (!localized) {
		/*
		 * RFC 3414 §2.6 expansion hashes 1 MiB per key by design, which
		 * is ~100 ms each on this part and happens on the bring-up
		 * thread. Say so, so the pause in the boot log has a cause, and
		 * so the localised-key route (which skips it entirely and keeps
		 * the passphrase off the device) is visible as the alternative.
		 */
		LOG_INF("SNMPv3 user '%s': expanding passphrases (RFC 3414 §2.6, "
			"1 MiB per key); provision localised keys to avoid it",
			name);
	}

	rc = snmp_v3_user_set(&v3, name, auth, (akey_len != 0U) ? akey : NULL,
			      akey_len, localized, priv,
			      (pkey_len != 0U) ? pkey : NULL, pkey_len,
			      localized);

	/*
	 * Drop the plaintext copies before this frame is reused. mbedTLS's
	 * zeroize, not memset: a plain memset to a dead local is exactly what
	 * dead-store elimination removes, and a wipe that the compiler deletes is
	 * worse than none because it reads as a control that is not there. This
	 * does not make the secret gone — cfg's live tree holds the same bytes for
	 * the life of the box, and key zeroization on factory reset is a
	 * documented deferral (ARCHITECTURE.md §5) — it just keeps it off a stack
	 * that later carries unrelated data.
	 */
	mbedtls_platform_zeroize(akey, sizeof(akey));
	mbedtls_platform_zeroize(pkey, sizeof(pkey));

	if (rc == -ENOTSUP) {
		LOG_ERR("SNMPv3 user '%s': this build cannot expand a %s "
			"passphrase (no streaming SHA-256 port); provision an "
			"already-localised key and set sec.snmpv3.local = 1",
			name, snmp_auth_proto_name(auth));
		return;
	}
	if (rc != 0) {
		LOG_ERR("SNMPv3 user '%s' rejected (%d): auth %s, priv %s, %s "
			"secrets of %u/%u octets", name, rc,
			snmp_auth_proto_name(auth), snmp_priv_proto_name(priv),
			localized ? "localised" : "passphrase",
			(unsigned int)akey_len, (unsigned int)pkey_len);
		return;
	}

	LOG_INF("SNMPv3 user '%s': auth %s, priv %s", name,
		snmp_auth_proto_name(auth), snmp_priv_proto_name(priv));
}

/**
 * Bring the USM engine up.
 *
 * Any failure leaves v3_ready false, which makes serve_one() pass NULL to
 * snmp_dispatch() and every v3 datagram a counted drop. That is the right
 * outcome for a missing identity: an agent that answers under a default or
 * duplicated engine id is worse than one that does not answer v3 at all.
 */
static void v3_start(void)
{
	snmp_v3_cfg_t vcfg;
	/* The engine id is 4 octets of PEN plus a format octet plus the unique
	 * value, capped at SNMP_V3_ENGINEID_MAX — so this is all of the unique
	 * value that can ever be used. */
	uint8_t uniq[SNMP_V3_ENGINEID_MAX - 5U];
	size_t uniq_len = 0U;
	bool from_atecc = false;
	uint32_t boots = 0U;
	int len;
	int rc;

	if (!sts_net_cfg_bool(CFG_ID_SEC_SNMPV3_EN, true)) {
		LOG_INF("SNMPv3/USM off by configuration (sec.snmpv3.en)");
		return;
	}

	rc = sts_atecc_device_unique(uniq, sizeof(uniq), &uniq_len, &from_atecc);
	if (rc != 0) {
		LOG_ERR("no device-unique value for the SNMP engine id (%d); "
			"SNMPv3 stays off", rc);
		return;
	}
	/* Which source answered is operationally relevant: the SoC device id is
	 * per-die and immutable but not tamper-resistant. */
	LOG_INF("SNMP engine id: %u octets from %s", (unsigned int)uniq_len,
		from_atecc ? "the ATECC608B serial" : "the SoC device id");

	len = snmp_v3_engine_id_build(SNMP_PEN, uniq, uniq_len, v3_engine_id,
				      sizeof(v3_engine_id));
	if (len < 0) {
		LOG_ERR("snmp_v3_engine_id_build: %d; SNMPv3 stays off", len);
		return;
	}

	if (v3_next_boots(&boots) != 0) {
		return;
	}

	memset(&vcfg, 0, sizeof(vcfg));
	vcfg.ports.crypto = sts_port_crypto();
	/* Needed only to expand a SHA-256 passphrase (1 MiB of streaming hash);
	 * without it snmp_v3_user_set() answers -ENOTSUP and the operator's fix
	 * is to provision localised keys. */
	vcfg.ports.sha256_stream = sts_port_sha256_stream();
	vcfg.engine_boots = boots;
	vcfg.engine_id = v3_engine_id;
	vcfg.engine_id_len = (size_t)len;
	vcfg.max_msg_size = 0U; /* selects SNMP_PKT_MAX */

	v3_boot_ms = sts_mono_ms();
	rc = snmp_v3_init(&v3, &vcfg);
	if (rc != 0) {
		LOG_ERR("snmp_v3_init: %d; SNMPv3 stays off", rc);
		return;
	}
	v3_ready = true;

	{
		bool localized = sts_net_cfg_bool(CFG_ID_SEC_SNMPV3_LOCALIZED,
						  false);

		v3_load_user(CFG_ID_SEC_SNMPV3_U1_NAME, CFG_ID_SEC_SNMPV3_U1_AUTH,
			     CFG_ID_SEC_SNMPV3_U1_AKEY, CFG_ID_SEC_SNMPV3_U1_PRIV,
			     CFG_ID_SEC_SNMPV3_U1_PKEY, localized);
		v3_load_user(CFG_ID_SEC_SNMPV3_U2_NAME, CFG_ID_SEC_SNMPV3_U2_AUTH,
			     CFG_ID_SEC_SNMPV3_U2_AKEY, CFG_ID_SEC_SNMPV3_U2_PRIV,
			     CFG_ID_SEC_SNMPV3_U2_PKEY, localized);
	}

	if (snmp_v3_user_count(&v3) == 0U) {
		/* Not a failure: the engine still answers a discovery Report, so
		 * a manager can read the engine id back and be provisioned
		 * against it. It just cannot authenticate anybody yet. */
		LOG_WRN("SNMPv3 is enabled with no USM user provisioned: the engine "
			"answers discovery only. Set sec.snmpv3.u1.*");
	}

	LOG_INF("SNMPv3/USM ready: engineID %u octets, engineBoots %u, %u user(s)",
		(unsigned int)len, (unsigned int)boots,
		(unsigned int)snmp_v3_user_count(&v3));
}

/* ------------------------------------------------------------------------- */
/* trap queue                                                                */
/* ------------------------------------------------------------------------- */

void sts_snmp_notify(snmp_trap_t trap)
{
	K_SPINLOCK(&trap_lock) {
		uint8_t nxt = (uint8_t)((trap_head + 1U) % TRAP_QUEUE_LEN);

		if (nxt != trap_tail) { /* drop silently when full */
			trap_q[trap_head] = trap;
			trap_head = nxt;
		}
	}
}

static bool trap_dequeue(snmp_trap_t *out)
{
	bool got = false;

	K_SPINLOCK(&trap_lock) {
		if (trap_tail != trap_head) {
			*out = trap_q[trap_tail];
			trap_tail = (uint8_t)((trap_tail + 1U) % TRAP_QUEUE_LEN);
			got = true;
		}
	}
	return got;
}

/**
 * Resolve the trap destination and open a socket of the matching family.
 *
 * The socket used to be created once, unconditionally as AF_INET6, while the
 * resolver was left at AF_UNSPEC — so an IPv4-resolving trap host produced an
 * AF_INET destination that every sendto() on the v6 socket rejected, silently.
 * No trap ever arrived and nothing said so (F9). Binding the socket's family to
 * the address actually resolved is the fix; the socket is recreated on reapply
 * because the family can change with the host.
 */
static void resolve_trap_dest(void)
{
	char h[64];
	char port_s[8];
	struct zsock_addrinfo hints;
	struct zsock_addrinfo *res = NULL;
	uint16_t port;
	int fd;

	trap_resolved = false;
	if (trap_sock >= 0) {
		(void)zsock_close(trap_sock);
		trap_sock = -1;
	}

	(void)sts_net_cfg_str(CFG_ID_SNMP_TRAP_HOST, h, sizeof(h));
	if (h[0] == '\0') {
		return;
	}
	port = (uint16_t)sts_net_cfg_u64(CFG_ID_SNMP_TRAP_PORT, 162U);
	(void)snprintk(port_s, sizeof(port_s), "%u", port);

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;
	if (zsock_getaddrinfo(h, port_s, &hints, &res) != 0 || res == NULL) {
		LOG_WRN("SNMP trap host '%s' unresolved", h);
		return;
	}
	if ((size_t)res->ai_addrlen > sizeof(trap_dest)) {
		LOG_ERR("SNMP trap host '%s' resolved to an oversized address", h);
		zsock_freeaddrinfo(res);
		return;
	}
	memcpy(&trap_dest, res->ai_addr, res->ai_addrlen);
	trap_dest_len = res->ai_addrlen;

	fd = zsock_socket(res->ai_family, SOCK_DGRAM, IPPROTO_UDP);
	zsock_freeaddrinfo(res);
	if (fd < 0) {
		LOG_ERR("SNMP trap socket for '%s': %d", h, errno);
		return;
	}
	trap_sock = fd;
	trap_resolved = true;
}

/* ------------------------------------------------------------------------- */
/* source ACL                                                                */
/* ------------------------------------------------------------------------- */

/** Do the first @p bits of @p a and @p b agree? */
static bool prefix_match(const uint8_t *a, const uint8_t *b, unsigned int bits)
{
	unsigned int whole = bits / 8U;
	unsigned int rest = bits % 8U;

	if (whole != 0U && memcmp(a, b, whole) != 0) {
		return false;
	}
	if (rest != 0U) {
		uint8_t mask = (uint8_t)(0xFFU << (8U - rest));

		if (((a[whole] ^ b[whole]) & mask) != 0U) {
			return false;
		}
	}
	return true;
}

/** Is @p sa permitted by `net.mgmt.acl`? An empty ACL permits everything. */
static bool acl_allows(const struct sockaddr *sa)
{
	uint8_t local[SNMP_ACL_MAX];
	size_t len;
	size_t off = 0U;
	bool deny_all;

	K_SPINLOCK(&acl_lock) {
		len = acl_len;
		deny_all = acl_deny_all;
		memcpy(local, acl, sizeof(local));
	}
	if (deny_all) {
		return false;
	}
	if (len == 0U) {
		return true; /* not configured; see the file header */
	}

	/* load_acl() has already validated the walk, so every entry here is whole
	 * and of a known kind; a blob that could not be parsed sets acl_deny_all
	 * above rather than being re-diagnosed on every datagram. */
	while ((off + ACL_ENTRY_V4_LEN) <= len) {
		uint8_t kind = local[off];

		if (kind == ACL_KIND_V4) {
			if (sa->sa_family == AF_INET) {
				const struct sockaddr_in *s4 =
					(const struct sockaddr_in *)sa;

				if (prefix_match((const uint8_t *)&s4->sin_addr,
						 &local[off + 1U],
						 local[off + 5U])) {
					return true;
				}
			}
			off += ACL_ENTRY_V4_LEN;
		} else if (kind == ACL_KIND_V6) {
			if (sa->sa_family == AF_INET6) {
				const struct sockaddr_in6 *s6 =
					(const struct sockaddr_in6 *)sa;

				if (prefix_match((const uint8_t *)&s6->sin6_addr,
						 &local[off + 1U],
						 local[off + 17U])) {
					return true;
				}
			}
			off += ACL_ENTRY_V6_LEN;
		} else {
			break; /* ACL_KIND_END */
		}
	}
	return false;
}

/**
 * Read and validate `net.mgmt.acl`.
 *
 * Validation happens here, once per apply, so the per-datagram path is a plain
 * walk with no diagnostics and no way to log per packet. A blob that cannot be
 * walked denies everything: an operator who set an ACL meant to restrict access,
 * and silently ignoring their unparseable value would leave the agent wide open.
 */
static void load_acl(void)
{
	uint8_t next[SNMP_ACL_MAX];
	size_t n = 0U;
	size_t off = 0U;
	bool bad = false;

	memset(next, 0, sizeof(next));
	if (cfg_get_bytes(sts_cfg(), CFG_ID_NET_MGMT_ACL, next, sizeof(next),
			  &n) != 0) {
		n = 0U;
	}

	while (off < n) {
		uint8_t kind = next[off];
		size_t need;

		if (kind == ACL_KIND_END) {
			n = off; /* everything after the terminator is padding */
			break;
		}
		if (kind == ACL_KIND_V4) {
			need = ACL_ENTRY_V4_LEN;
		} else if (kind == ACL_KIND_V6) {
			need = ACL_ENTRY_V6_LEN;
		} else {
			bad = true;
			break;
		}
		if ((n - off) < need) {
			bad = true;
			break;
		}
		if ((kind == ACL_KIND_V4 && next[off + 5U] > 32U) ||
		    (kind == ACL_KIND_V6 && next[off + 17U] > 128U)) {
			bad = true;
			break;
		}
		off += need;
	}

	if (bad) {
		LOG_ERR("net.mgmt.acl is malformed at octet %u; refusing every "
			"SNMP source until it is corrected", (unsigned int)off);
	}

	K_SPINLOCK(&acl_lock) {
		acl_deny_all = bad;
		memcpy(acl, next, sizeof(acl));
		acl_len = bad ? 0U : n;
	}
}

static void send_trap(snmp_trap_t t)
{
	const char *user = trap_user_buf[trap_user_live];
	size_t len = 0U;
	bool as_v3 = false;

	if (!trap_resolved || trap_sock < 0) {
		return;
	}

	if (v3_ready && user[0] != '\0') {
		/* Our own engine id, boots and time: RFC 3414 §3.1 makes the
		 * notification originator the authoritative engine, which is why a
		 * receiver never has to discover anything to accept one. */
		int rc = snmp_v3_make_notification(&agent, &v3, user, trap_level,
						   t, engine_time_s(),
						   sts_net_uptime_cs(), NULL, 0U,
						   false, trap_buf,
						   sizeof(trap_buf), &len, NULL);

		if (rc != 0) {
			/*
			 * Deliberately NOT a fall-back to v2c. The operator named
			 * a USM user, so -ENOENT (no such user) or -EPERM (that
			 * user cannot reach the requested level) is a
			 * configuration error; downgrading would put the event on
			 * the wire unauthenticated, to a manager that is almost
			 * certainly configured to discard it. Logged once, not
			 * per trap — the transitions that produce traps repeat,
			 * and each log line costs a RAM-ring and NOR-spool record
			 * (F9). publish_trap_user() re-arms it.
			 */
			if (!trap_v3_warned) {
				trap_v3_warned = true;
				LOG_ERR("SNMPv3 notification as '%s' at %s failed "
					"(%d); notifications stay suppressed until "
					"sec.snmpv3.trap.user/.lvl is corrected",
					user, snmp_sec_level_name(trap_level), rc);
			}
			return;
		}
		as_v3 = true;
	} else if (snmp_make_trap(&agent, t, sts_net_uptime_cs(), NULL, 0U,
				  trap_buf, sizeof(trap_buf), &len) != 0) {
		return;
	}

	(void)zsock_sendto(trap_sock, trap_buf, len, 0,
			   (struct sockaddr *)&trap_dest, trap_dest_len);
	LOG_INF("SNMP%s trap: %s", as_v3 ? "v3" : "v2c", snmp_trap_name(t));
}

/* ------------------------------------------------------------------------- */
/* transition detection                                                      */
/* ------------------------------------------------------------------------- */

static void detect_transitions(void)
{
	quality_block_t q;
	uint64_t alarms = sts_alarms_active();

	if (sts_quality_snapshot(&q) != 0) {
		return;
	}

	if (!have_prev) {
		have_prev = true;
		prev_lock = q.lock_state;
		prev_ref = q.active_ref;
		prev_holdover = q.holdover;
		prev_alarms = alarms;
		return;
	}

	if (q.lock_state == QUALITY_LOCK_LOCKED &&
	    prev_lock != QUALITY_LOCK_LOCKED) {
		sts_snmp_notify(SNMP_TRAP_LOCK_ACQUIRED);
	} else if (prev_lock == QUALITY_LOCK_LOCKED &&
		   q.lock_state != QUALITY_LOCK_LOCKED) {
		sts_snmp_notify(SNMP_TRAP_LOCK_LOST);
	}

	if (q.holdover && !prev_holdover) {
		sts_snmp_notify(SNMP_TRAP_HOLDOVER_ENTER);
	} else if (!q.holdover && prev_holdover) {
		sts_snmp_notify(SNMP_TRAP_HOLDOVER_EXIT);
	}

	if (q.active_ref != prev_ref) {
		sts_snmp_notify(SNMP_TRAP_REF_SWITCH);
	}

	/* Alarm rising edges. The rail/thermal/antenna families map onto the
	 * three notification OIDs; a manager reads the alarms bitmask leaf for
	 * the detail. */
	if (alarms != prev_alarms) {
		uint64_t rising = alarms & ~prev_alarms;

		if (rising &
		    (FAULT_ALARM_BIT(FAULT_ALARM_THERMAL_WARN) |
		     FAULT_ALARM_BIT(FAULT_ALARM_THERMAL_CRITICAL))) {
			sts_snmp_notify(SNMP_TRAP_THERMAL_ALARM);
		}
		if (rising &
		    (FAULT_ALARM_BIT(FAULT_ALARM_ANTENNA_OPEN) |
		     FAULT_ALARM_BIT(FAULT_ALARM_ANTENNA_SHORT))) {
			sts_snmp_notify(SNMP_TRAP_ANTENNA_FAULT);
		}
		if (rising &
		    (FAULT_ALARM_BIT(FAULT_ALARM_RB_FAULT) |
		     FAULT_ALARM_BIT(FAULT_ALARM_RB_OV) |
		     FAULT_ALARM_BIT(FAULT_ALARM_POE_BUDGET))) {
			sts_snmp_notify(SNMP_TRAP_RAIL_ALARM);
		}
	}

	prev_lock = q.lock_state;
	prev_ref = q.active_ref;
	prev_holdover = q.holdover;
	prev_alarms = alarms;
}

/* ------------------------------------------------------------------------- */
/* config                                                                    */
/* ------------------------------------------------------------------------- */

/**
 * Load the configured community into the spare buffer and publish it.
 *
 * The publish is the single store of `community_live`; core/snmp borrows the
 * pointer we hand it, so the buffer it points at must never be written again
 * while it is live (F10).
 */
static void publish_community(void)
{
	uint8_t next = (uint8_t)(community_live ^ 1U);
	char *buf = community_buf[next];

	(void)sts_net_cfg_str(CFG_ID_SNMP_COMMUNITY, buf,
			      SNMP_COMMUNITY_MAX + 1U);
	if (buf[0] == '\0') {
		/*
		 * core/snmp documents NULL as "refuse every request", which is the
		 * right reading of an unconfigured community — but handing it a
		 * NULL is also what tripped the missing guard in its
		 * build_response(). Pass the empty string instead: core compares
		 * it and refuses just the same, with nothing to dereference.
		 */
		LOG_WRN("snmp.community is empty; every request will be refused");
	}
	(void)snmp_set_community(&agent, buf);
	community_live = next;
}

void sts_snmp_reapply(void)
{
	if (!started) {
		return;
	}
	publish_community();
	(void)sts_net_cfg_str(CFG_ID_NET_HOSTNAME, hostname, sizeof(hostname));
	load_acl();
	resolve_trap_dest();
}

/**
 * Runtime-apply the 0x0A security keys this agent owns.
 *
 * Only the keys the schema flags CFG_F_RUNTIME_APPLY are here — `sec.snmp.v2c`,
 * `sec.snmp.notify.ms`, `sec.snmpv3.trap.user/.lvl`. The USM users,
 * `sec.snmpv3.local` and `sec.snmpv3.en` are CFG_F_REBOOT_REQUIRED and stay
 * that way on purpose: localised keys are bound to the engine id, so
 * re-initialising a live engine would clear every user
 * (snmp_v3_set_engine_id()) and leave the agent looking configured while
 * authenticating nobody. cfg reports the group in `reboot_groups`, which is how
 * the operator learns a reboot is needed.
 *
 * Every publish below is one aligned store or the double-buffered string, so the
 * SNMP thread always sees a whole value.
 */
void sts_snmp_on_cfg_sec(void)
{
	if (!started) {
		return;
	}

	(void)snmp_set_v2c_enabled(&agent,
				   sts_net_cfg_bool(CFG_ID_SEC_SNMP_V2C_EN, false));

	/*
	 * core/snmp has no setter for the notification interval and is outside
	 * this change's boundary. The field is a single aligned uint32_t that
	 * snmp_notify_gate() only ever reads as a comparison, so the gate sees
	 * either the old or the new value — never a torn one.
	 * TODO(core/snmp): add snmp_set_notify_interval() and use it here.
	 */
	agent.cfg.notify_min_interval_ms =
		(uint32_t)sts_net_cfg_u64(CFG_ID_SEC_SNMP_NOTIFY_MS,
					  SNMP_NOTIFY_MIN_INTERVAL_MS);

	publish_trap_user();
}

/* ------------------------------------------------------------------------- */
/* agent socket                                                              */
/* ------------------------------------------------------------------------- */

static int open_agent(sa_family_t fam)
{
	struct sockaddr_in a4;
	struct sockaddr_in6 a6;
	struct sockaddr *sa;
	socklen_t slen;
	int fd = zsock_socket(fam, SOCK_DGRAM, IPPROTO_UDP);

	if (fd < 0) {
		return -errno;
	}
	if (fam == AF_INET) {
		memset(&a4, 0, sizeof(a4));
		a4.sin_family = AF_INET;
		a4.sin_port = htons(SNMP_AGENT_PORT);
		a4.sin_addr.s_addr = INADDR_ANY;
		sa = (struct sockaddr *)&a4;
		slen = sizeof(a4);
	} else {
		memset(&a6, 0, sizeof(a6));
		a6.sin6_family = AF_INET6;
		a6.sin6_port = htons(SNMP_AGENT_PORT);
		a6.sin6_addr = in6addr_any;
		sa = (struct sockaddr *)&a6;
		slen = sizeof(a6);
	}
	if (zsock_bind(fd, sa, slen) < 0) {
		(void)zsock_close(fd);
		return -errno;
	}
	return fd;
}

/** @return true when a datagram was handled, false when the socket ran dry. */
static bool serve_one(int fd)
{
	struct sockaddr_storage peer;
	socklen_t plen = sizeof(peer);
	size_t rsp_len = 0U;
	ssize_t n;
	int rc;

	n = zsock_recvfrom(fd, rx, sizeof(rx), 0, (struct sockaddr *)&peer,
			   &plen);
	if (n <= 0) {
		return false;
	}

	/*
	 * The ACL is checked before the codec runs, so a source that is not
	 * permitted costs a compare and nothing else — no parse, no MIB walk, and
	 * above all no response to an address it may have forged (F9).
	 */
	if (!acl_allows((const struct sockaddr *)&peer)) {
		acl_refused++;
		return true;
	}

	/*
	 * snmp_dispatch(), not snmp_handle(): it routes on msgVersion so a v3
	 * manager reaches the USM engine and a v2c manager still reaches the same
	 * PDU layer. A NULL context is its documented way of counting and dropping
	 * v3 datagrams, which is what `sec.snmpv3.en = 0` (or a v3 engine that
	 * could not be identified) should mean.
	 */
	rc = snmp_dispatch(&agent, v3_ready ? &v3 : NULL, rx, (size_t)n,
			   engine_time_s(), sts_net_uptime_cs(), tx, sizeof(tx),
			   &rsp_len);
	if (rc == 0) {
		/* May be a Report PDU rather than a Response — that is a normal
		 * success and must be sent, because provoking one is how a manager
		 * discovers the engine id and synchronises its clock. */
		(void)zsock_sendto(fd, tx, rsp_len, 0,
				   (struct sockaddr *)&peer, plen);
		return true;
	}
	if (rc == -EACCES) {
		/*
		 * A bad v2c community, or a v3 digest that did not verify on a
		 * request that was not reportable: RFC-silent to the sender, but a
		 * security event worth a trap (spec §5.3) — offered to the gate,
		 * not enqueued unconditionally, so a flood becomes one trap per
		 * interval instead of one per datagram.
		 */
		if (snmp_notify_gate(&agent, SNMP_TRAP_AUTH_FAILURE,
				     sts_mono_ms(), NULL)) {
			sts_snmp_notify(SNMP_TRAP_AUTH_FAILURE);
		}
	}
	return true;
}

/* ------------------------------------------------------------------------- */
/* thread                                                                    */
/* ------------------------------------------------------------------------- */

static void snmp_loop(void *a, void *b, void *c)
{
	uint64_t last_txn_ms = 0U;
	uint64_t last_acl_log_ms = 0U;
	uint32_t acl_logged = 0U;
	bool cold_start_sent = false;

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		struct zsock_pollfd fds[2];
		snmp_trap_t t;
		uint64_t now;
		int nfds = 0;
		int rc;

		if (sock4 >= 0) {
			fds[nfds].fd = sock4;
			fds[nfds].events = ZSOCK_POLLIN;
			fds[nfds].revents = 0;
			nfds++;
		}
		if (sock6 >= 0) {
			fds[nfds].fd = sock6;
			fds[nfds].events = ZSOCK_POLLIN;
			fds[nfds].revents = 0;
			nfds++;
		}

		rc = (nfds > 0) ? zsock_poll(fds, nfds, SNMP_POLL_MS)
				: (k_sleep(K_MSEC(SNMP_POLL_MS)), 0);
		if (rc > 0) {
			int i;

			for (i = 0; i < nfds; i++) {
				unsigned int k;

				if ((fds[i].revents & ZSOCK_POLLIN) == 0) {
					continue;
				}
				/* Bounded drain; see SNMP_RX_BUDGET. */
				for (k = 0U; k < SNMP_RX_BUDGET; k++) {
					if (!serve_one(fds[i].fd)) {
						break;
					}
				}
			}
		}

		/* A coldStart is only meaningful once we can actually reach a
		 * manager; defer it until the trap host resolves. */
		if (!cold_start_sent && trap_resolved) {
			send_trap(SNMP_TRAP_COLD_START);
			cold_start_sent = true;
		}

		now = k_uptime_get();
		if (now - last_txn_ms >= 1000U) {
			last_txn_ms = now;
			detect_transitions();
		}

		/* An ACL that is dropping traffic is worth annunciating once a
		 * minute — an operator whose poller stopped working needs to see
		 * it — but never once per datagram. */
		if (acl_refused != acl_logged && (now - last_acl_log_ms) >= 60000U) {
			last_acl_log_ms = now;
			LOG_WRN("net.mgmt.acl refused %u SNMP datagrams",
				(unsigned int)(acl_refused - acl_logged));
			acl_logged = acl_refused;
		}

		while (trap_dequeue(&t)) {
			send_trap(t);
		}

		if (live_id >= 0) {
			sts_liveness_feed(live_id);
		}

		/* Unconditional yield: see SNMP_RX_BUDGET (F12). */
		k_yield();
	}
}

/* ------------------------------------------------------------------------- */
/* public                                                                    */
/* ------------------------------------------------------------------------- */

void sts_snmp_stats(snmp_stats_t *out)
{
	if (out == NULL) {
		return;
	}
	(void)snmp_stats_get(&agent, out);
}

void sts_snmp_v3_stats(snmp_v3_stats_t *out)
{
	if (out == NULL) {
		return;
	}
	memset(out, 0, sizeof(*out));
	if (v3_ready) {
		(void)snmp_v3_stats_get(&v3, out);
	}
}

int sts_snmp_start(void)
{
	snmp_cfg_t cfg;
	bool v2c;
	int rc;

	if (!sts_net_cfg_bool(CFG_ID_SNMP_ENABLE, false)) {
		LOG_INF("SNMP agent disabled by configuration");
		return 0;
	}

	(void)sts_net_cfg_str(CFG_ID_SNMP_COMMUNITY, community_buf[0],
			      SNMP_COMMUNITY_MAX + 1U);
	community_live = 0U;
	(void)sts_net_cfg_str(CFG_ID_NET_HOSTNAME, hostname, sizeof(hostname));
	load_acl();

	memset(&cfg, 0, sizeof(cfg));
	/* Never NULL: core/snmp treats an empty string the same way (refuse
	 * every request) with nothing to dereference. */
	cfg.community = community_buf[0];
	cfg.getter.get = getter;
	cfg.getter.ctx = NULL;
	cfg.max_repetitions = 0U;
	cfg.notify_min_interval_ms =
		(uint32_t)sts_net_cfg_u64(CFG_ID_SEC_SNMP_NOTIFY_MS,
					 SNMP_NOTIFY_MIN_INTERVAL_MS);

	rc = snmp_init(&agent, &cfg);
	if (rc != 0) {
		LOG_ERR("snmp_init: %d", rc);
		return rc;
	}

	/*
	 * `sec.snmp.v2c` defaults to 0 (spec §9.5: v2c only when explicitly
	 * enabled), and core/snmp spells its gate negatively so a zeroed config
	 * keeps the historical behaviour — which means the gate has to be driven
	 * from cfg here or the default never takes effect.
	 */
	v2c = sts_net_cfg_bool(CFG_ID_SEC_SNMP_V2C_EN, false);
	(void)snmp_set_v2c_enabled(&agent, v2c);

	v3_start();
	publish_trap_user();

	if (!v2c && !v3_ready) {
		LOG_ERR("SNMP is enabled but both versions are off: v2c by "
			"sec.snmp.v2c and v3 by sec.snmpv3.en (or a failed engine "
			"id). The agent will answer nothing.");
	}
	if (community_buf[0][0] == '\0') {
		/* Both halves matter: the community authenticates v2c requests AND
		 * is what snmp_make_trap() needs, so an empty one also silences
		 * every v2c trap. */
		LOG_WRN("snmp.community is empty; every v2c request will be refused "
			"and no v2c trap can be built");
	}
	if (acl_len == 0U && !acl_deny_all) {
		LOG_WRN("SNMP is enabled with no net.mgmt.acl: every source may "
			"query the agent, and a spoofed GETBULK makes it a "
			"reflector. Set net.mgmt.acl.");
	}

	sock4 = open_agent(AF_INET);
	sock6 = open_agent(AF_INET6);
	if (sock4 < 0 && sock6 < 0) {
		LOG_ERR("no SNMP agent socket");
		return -ENOTCONN;
	}

	/* resolve_trap_dest() opens the socket in the family the host actually
	 * resolved to; see its comment. */
	resolve_trap_dest();

	started = true;
	live_id = sts_liveness_register("snmp");

	k_thread_create(&snmp_thread, snmp_stack,
			K_THREAD_STACK_SIZEOF(snmp_stack), snmp_loop, NULL,
			NULL, NULL, SNMP_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&snmp_thread, "snmp");

	LOG_INF("SNMP agent on :%d (v2c %s, v3/USM %s), notifications as %s",
		SNMP_AGENT_PORT, v2c ? "on" : "off", v3_ready ? "on" : "off",
		(v3_ready && trap_user_buf[trap_user_live][0] != '\0') ? "v3"
								       : "v2c");
	return 0;
}
