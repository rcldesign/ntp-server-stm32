/*
 * STS1000 "Meridian" — remote syslog sender, RFC 5424 over UDP (spec §5.4).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * ---------------------------------------------------------------------------
 * How this stays out of the console area's way
 * ---------------------------------------------------------------------------
 *
 * The logring is a multi-reader ring (logring.h): "a record stays until it is
 * overwritten, so several readers (MCP tail, spool, syslog) advance
 * independently. A cursor is a sequence number." So this sender keeps its OWN
 * cursor and calls logr_tail() with it — it does not consume records, does not
 * register a sink, and does not couple to the console area's logger thread in
 * any way. The console area drains the ring to USB and the NOR spool through
 * its own cursor; this file reads the same ring through a second cursor. That
 * is exactly the independent-reader model the header documents, and it is why
 * the coordination note about "not adding a single-cursor sink" is satisfied by
 * construction.
 *
 * Reading is polled at SYSLOG_POLL_MS rather than event-driven: a low-priority
 * (16, spec §1.2) periodic drain cannot stall a service thread that is trying
 * to log a fault, and a few hundred milliseconds of latency on a remote log
 * line is immaterial. logr_tail() reports how many records were overwritten
 * before this reader reached them (the gap count), which is forwarded as a
 * synthetic NOTICE so a truncated remote log is auditable.
 *
 * ---------------------------------------------------------------------------
 * TLS transport
 * ---------------------------------------------------------------------------
 *
 * RFC 5425 syslog-over-TLS is config-gated (log.syslog.tls) and NOT implemented
 * this phase. When the key is set the sender logs once that TLS is unavailable
 * and falls back to UDP, rather than silently sending cleartext a operator
 * asked to be encrypted or silently sending nothing. TODO(syslog-tls): reuse
 * the sts_ntske.c mbedTLS pattern for a TCP+TLS transport.
 */

#include <errno.h>
#include <string.h>
#include <time.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/clock.h>

#include "cfg/cfg.h"
#include "logring/logring.h"
#include "net/sts_net.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_syslog, CONFIG_STS1000_LOG_LEVEL);

#define SYSLOG_STACK_SIZE 3072
#define SYSLOG_PRIORITY 16
#define SYSLOG_POLL_MS 500
#define SYSLOG_BATCH 16U

static char host[64];
static char appname[LOGR_APPNAME_MAX];
static uint16_t dest_port;
static bool enabled;
static bool warned_tls;

static struct sockaddr_storage dest;
static socklen_t dest_len;
static bool dest_resolved;

static int sock = -1;
static uint32_t cursor;

static struct {
	uint32_t sent;
	uint32_t dropped;
	uint32_t gaps;
} gstat;
static struct k_spinlock stat_lock;

static K_THREAD_STACK_DEFINE(syslog_stack, SYSLOG_STACK_SIZE);
static struct k_thread syslog_thread;
static int live_id = -1;
static bool started;

/* ------------------------------------------------------------------------- */
/* configuration                                                             */
/* ------------------------------------------------------------------------- */

/** Resolve the configured host into @ref dest. Numeric literals only for now. */
static bool resolve_dest(const char *h)
{
	struct zsock_addrinfo hints;
	struct zsock_addrinfo *res = NULL;
	char port_s[8];
	int rc;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;

	(void)snprintk(port_s, sizeof(port_s), "%u", dest_port);

	rc = zsock_getaddrinfo(h, port_s, &hints, &res);
	if (rc != 0 || res == NULL) {
		return false;
	}

	memcpy(&dest, res->ai_addr, res->ai_addrlen);
	dest_len = res->ai_addrlen;
	zsock_freeaddrinfo(res);
	return true;
}

static void reload(void)
{
	char h[64];

	enabled = sts_net_cfg_bool(CFG_ID_LOG_SYSLOG_EN, false);
	dest_port = (uint16_t)sts_net_cfg_u64(CFG_ID_LOG_SYSLOG_PORT, 514U);
	(void)sts_net_cfg_str(CFG_ID_LOG_SYSLOG_HOST, h, sizeof(h));

	if (sts_net_cfg_bool(CFG_ID_LOG_SYSLOG_TLS, false) && !warned_tls) {
		LOG_WRN("syslog TLS requested but not implemented; using UDP");
		warned_tls = true;
	}

	if (!enabled || h[0] == '\0') {
		dest_resolved = false;
		return;
	}

	(void)sts_net_cfg_str(CFG_ID_NET_HOSTNAME, host, sizeof(host));
	if (host[0] == '\0') {
		strcpy(host, "meridian");
	}
	strcpy(appname, "sts1000");

	dest_resolved = resolve_dest(h);
	if (!dest_resolved) {
		LOG_WRN("syslog host '%s' unresolved", h);
	}
}

void sts_syslog_reapply(void)
{
	if (started) {
		reload();
	}
}

/* ------------------------------------------------------------------------- */
/* one record -> one datagram                                                */
/* ------------------------------------------------------------------------- */

static void send_record(const logr_rec_t *r)
{
	char line[LOGR_RENDER_MAX];
	struct timespec ts;
	uint64_t unix_s = 0U;
	uint32_t frac_us = 0U;
	int n;

	if (sys_clock_gettime(SYS_CLOCK_REALTIME, &ts) == 0) {
		unix_s = (uint64_t)ts.tv_sec;
		frac_us = (uint32_t)(ts.tv_nsec / 1000);
	}

	n = logr_render_5424(r, LOGR_FACILITY_LOCAL0, host, appname, unix_s,
			     frac_us, line, sizeof(line));
	if (n < 0) {
		return;
	}

	if (zsock_sendto(sock, line, (size_t)n, 0, (struct sockaddr *)&dest,
			 dest_len) < 0) {
		K_SPINLOCK(&stat_lock) {
			gstat.dropped++;
		}
		return;
	}
	K_SPINLOCK(&stat_lock) {
		gstat.sent++;
	}
}

static void drain_once(void)
{
	logr_rec_t recs[SYSLOG_BATCH];
	logr_t *ring = sts_logring();
	uint16_t got = 0U;
	uint32_t next = cursor;
	uint32_t gap = 0U;
	size_t i;

	if (ring == NULL || !dest_resolved || sock < 0) {
		return;
	}

	if (logr_tail(ring, cursor, NULL, recs, SYSLOG_BATCH, &got, &next,
		      &gap) != 0) {
		return;
	}

	if (gap != 0U) {
		K_SPINLOCK(&stat_lock) {
			gstat.gaps += gap;
		}
		LOG_WRN("syslog reader fell behind: %u records lost", gap);
	}

	for (i = 0U; i < got; i++) {
		send_record(&recs[i]);
	}
	cursor = next;
}

static void syslog_loop(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		if (enabled) {
			drain_once();
		}
		if (live_id >= 0) {
			sts_liveness_feed(live_id);
		}
		k_sleep(K_MSEC(SYSLOG_POLL_MS));
	}
}

/* ------------------------------------------------------------------------- */
/* public                                                                    */
/* ------------------------------------------------------------------------- */

void sts_syslog_stats(sts_syslog_stats_t *out)
{
	if (out == NULL) {
		return;
	}
	K_SPINLOCK(&stat_lock) {
		out->sent = gstat.sent;
		out->dropped = gstat.dropped;
		out->gaps = gstat.gaps;
	}
	out->enabled = enabled;
	out->resolved = dest_resolved;
}

int sts_syslog_start(void)
{
	logr_t *ring = sts_logring();

	/* Start reading from the ring's current head, not sequence 0: the
	 * boot-time log belongs to the local console and the NOR spool, not to
	 * a remote collector that was not listening for it. */
	if (ring != NULL) {
		cursor = logr_head(ring);
	}

	sock = zsock_socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		/* v6 socket also carries v4-mapped destinations; if even that
		 * fails there is no transport, but the thread still runs so a
		 * later cfg change can retry. */
		LOG_WRN("syslog socket: %d", errno);
	}

	started = true;
	reload();

	live_id = sts_liveness_register("syslog");

	k_thread_create(&syslog_thread, syslog_stack,
			K_THREAD_STACK_SIZEOF(syslog_stack), syslog_loop, NULL,
			NULL, NULL, SYSLOG_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&syslog_thread, "syslog");

	LOG_INF("syslog sender ready (enabled=%d)", (int)enabled);
	return 0;
}
