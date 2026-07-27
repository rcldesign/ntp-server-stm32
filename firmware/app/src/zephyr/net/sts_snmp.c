/*
 * STS1000 "Meridian" — SNMPv2c agent glue + trap sender (spec §5.3, prio 12).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * core/snmp is the codec and the walk; this file is the socket, the value
 * getter, and the trap sender.
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
 * alarms. snmp_handle() returning -EACCES (bad community) enqueues an
 * authentication-failure trap. A coldStart is sent once at start. The queue is
 * a small ring so a burst cannot block the producer.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>

#include "cfg/cfg.h"
#include "fault/fault.h"
#include "net/sts_net.h"
#include "quality/quality.h"
#include "snmp/snmp.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_snmp, CONFIG_STS1000_LOG_LEVEL);

#define SNMP_STACK_SIZE 4096
#define SNMP_PRIORITY 12
#define SNMP_AGENT_PORT 161
#define SNMP_POLL_MS 500
#define TRAP_QUEUE_LEN 16U

/* Firmware identity strings for the system group. */
#define SNMP_SYS_DESCR "STS1000 Meridian GPS-disciplined Stratum-1 NTP/PTP grandmaster"
#define SNMP_FW_VERSION "1.0.0-dev"

static snmp_ctx_t agent;
static char community[SNMP_COMMUNITY_MAX + 1U];
static char hostname[64];

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
	case SNMP_OBJ_SYS_OBJECT_ID: /* handled by core/snmp getter for OID? no */
	default:
		/* Includes the rail table, enclosure/die temp, humidity, fan,
		 * PoE, supercaps, gnss antenna/fw, serial — see the file
		 * comment's TODO(cross-area telemetry). */
		return -ENOENT;
	}
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

static void resolve_trap_dest(void)
{
	char h[64];
	char port_s[8];
	struct zsock_addrinfo hints;
	struct zsock_addrinfo *res = NULL;
	uint16_t port;

	trap_resolved = false;
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
	memcpy(&trap_dest, res->ai_addr, res->ai_addrlen);
	trap_dest_len = res->ai_addrlen;
	zsock_freeaddrinfo(res);
	trap_resolved = true;
}

static void send_trap(snmp_trap_t t)
{
	size_t len = 0U;

	if (!trap_resolved || trap_sock < 0) {
		return;
	}
	if (snmp_make_trap(&agent, t, sts_net_uptime_cs(), NULL, 0U, trap_buf,
			   sizeof(trap_buf), &len) != 0) {
		return;
	}
	(void)zsock_sendto(trap_sock, trap_buf, len, 0,
			   (struct sockaddr *)&trap_dest, trap_dest_len);
	LOG_INF("SNMP trap: %s", snmp_trap_name(t));
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

void sts_snmp_reapply(void)
{
	if (!started) {
		return;
	}
	(void)sts_net_cfg_str(CFG_ID_SNMP_COMMUNITY, community,
			      sizeof(community));
	(void)snmp_set_community(&agent, community[0] ? community : NULL);
	(void)sts_net_cfg_str(CFG_ID_NET_HOSTNAME, hostname, sizeof(hostname));
	resolve_trap_dest();
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

static void serve_one(int fd)
{
	struct sockaddr_storage peer;
	socklen_t plen = sizeof(peer);
	size_t rsp_len = 0U;
	ssize_t n;
	int rc;

	n = zsock_recvfrom(fd, rx, sizeof(rx), 0, (struct sockaddr *)&peer,
			   &plen);
	if (n <= 0) {
		return;
	}

	rc = snmp_handle(&agent, rx, (size_t)n, sts_net_uptime_cs(), tx,
			 sizeof(tx), &rsp_len);
	if (rc == 0) {
		(void)zsock_sendto(fd, tx, rsp_len, 0,
				   (struct sockaddr *)&peer, plen);
		return;
	}
	if (rc == -EACCES) {
		/* Bad community: RFC-silent to the sender, but a security event
		 * worth a trap (spec §5.3). */
		sts_snmp_notify(SNMP_TRAP_AUTH_FAILURE);
	}
}

/* ------------------------------------------------------------------------- */
/* thread                                                                    */
/* ------------------------------------------------------------------------- */

static void snmp_loop(void *a, void *b, void *c)
{
	uint64_t last_txn_ms = 0U;
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
				if ((fds[i].revents & ZSOCK_POLLIN) != 0) {
					serve_one(fds[i].fd);
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

		while (trap_dequeue(&t)) {
			send_trap(t);
		}

		if (live_id >= 0) {
			sts_liveness_feed(live_id);
		}
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

int sts_snmp_start(void)
{
	snmp_cfg_t cfg;
	int rc;

	if (!sts_net_cfg_bool(CFG_ID_SNMP_ENABLE, false)) {
		LOG_INF("SNMP agent disabled by configuration");
		return 0;
	}

	(void)sts_net_cfg_str(CFG_ID_SNMP_COMMUNITY, community,
			      sizeof(community));
	(void)sts_net_cfg_str(CFG_ID_NET_HOSTNAME, hostname, sizeof(hostname));

	memset(&cfg, 0, sizeof(cfg));
	cfg.community = community[0] ? community : NULL;
	cfg.getter.get = getter;
	cfg.getter.ctx = NULL;
	cfg.max_repetitions = 0U;

	rc = snmp_init(&agent, &cfg);
	if (rc != 0) {
		LOG_ERR("snmp_init: %d", rc);
		return rc;
	}

	sock4 = open_agent(AF_INET);
	sock6 = open_agent(AF_INET6);
	if (sock4 < 0 && sock6 < 0) {
		LOG_ERR("no SNMP agent socket");
		return -ENOTCONN;
	}

	trap_sock = zsock_socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	resolve_trap_dest();

	started = true;
	live_id = sts_liveness_register("snmp");

	k_thread_create(&snmp_thread, snmp_stack,
			K_THREAD_STACK_SIZEOF(snmp_stack), snmp_loop, NULL,
			NULL, NULL, SNMP_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&snmp_thread, "snmp");

	LOG_INF("SNMPv2c agent on :%d", SNMP_AGENT_PORT);
	return 0;
}
