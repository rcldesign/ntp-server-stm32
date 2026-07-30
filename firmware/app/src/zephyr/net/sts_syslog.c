/*
 * STS1000 "Meridian" — remote syslog sender: RFC 5424 over UDP, or RFC 5425
 * over TCP+TLS (spec §5.4, §5.5).
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
 * TLS transport, and what it replaced
 * ---------------------------------------------------------------------------
 *
 * `log.syslog.tls` used to log one local warning and then send the operator's
 * records in the clear over UDP anyway. That is now gone and cannot be
 * reinstated by configuration: net/sts_syslog_tls_policy.h owns the decision,
 * a host suite enumerates its whole input space, and there is no combination of
 * inputs for which asking for TLS yields the datagram transport. The reasoning
 * — why a downgrade is worse than a refusal, why "REFUSED" is distinct from
 * "OFF" — lives in that header and is not repeated here.
 *
 * Mechanically, TLS syslog is:
 *
 *   transport   RFC 5425: TCP, port 6514 by default, TLS 1.3 (web.conf pins
 *               MBEDTLS_TLS_VERSION_1_2=n board-wide).
 *   framing     octet-counted, `MSG-LEN SP SYSLOG-MSG`, from
 *               sts_syslog_frame(). NOT the UDP datagram format: on a stream
 *               there is no message boundary to infer.
 *   trust       the operator's anchor under STS_SYSLOG_CA_SEC_TAG, plus
 *               TLS_HOSTNAME so the certificate's NAME is checked and not
 *               merely its chain. No anchor -> refuse before the socket.
 *   spool       the logring, via an unadvanced cursor. See
 *               sts_syslog_advance_ok().
 *
 * ---------------------------------------------------------------------------
 * WHY THERE ARE TWO THREADS
 * ---------------------------------------------------------------------------
 *
 * This is not fashion, and it is the one part of the design that is not
 * obvious from the spec.
 *
 * Zephyr's ztls_connect_ctx() (subsys/net/lib/sockets/sockets_tls.c) is two
 * serialised blocking phases — zsock_connect(), then
 * tls_mbedtls_handshake(ctx, K_MSEC(CONFIG_NET_SOCKETS_CONNECT_TIMEOUT)) —
 * and BOTH are bounded by the same Kconfig. SO_SNDTIMEO does not shorten
 * either: sockets_inet.c uses the Kconfig value directly, and the handshake
 * carries an explicit TODO that it "blocks the socket even for non-blocking
 * socket". So one connect attempt to a host that is dark can occupy its caller
 * for 2 x 3000 ms.
 *
 * The drain thread is a liveness participant (sts_liveness_register), and
 * CONFIG_STS1000_LIVENESS_DEADLINE_MS is 5000. Doing the connect inline would
 * therefore mean: log collector stops answering -> drain thread blocks 6 s ->
 * supervisor stops kicking the TPS3430 -> the grandmaster cold-cycles. A clock
 * that reboots because a syslog server went away is a far worse bug than the
 * one this change set out to fix.
 *
 * So `syslog` drains, renders and writes (bounded by SO_SNDTIMEO, which DOES
 * apply to zsock_send on an established session) and feeds liveness every
 * pass; `syslog_tls` does nothing but connect, is not a liveness participant,
 * and is allowed to block for as long as Zephyr makes it. The BUILD_ASSERT
 * below fails the build if anyone later makes the inline case look safe by
 * changing either Kconfig, rather than letting the board discover it.
 *
 * Ownership across the two threads is deliberately asymmetric so no fd is ever
 * closed twice: the connector is the only opener and only ever publishes into
 * an empty slot; the drain thread is the only closer of a published fd. The
 * mutex is held for pointer-sized state transitions only, never across a
 * connect or a write.
 */

#include <errno.h>
#include <string.h>
#include <time.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/clock.h>

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
#include <zephyr/fs/fs.h>
#include <zephyr/net/tls_credentials.h>
#include <mbedtls/x509_crt.h>
#endif

#include "cfg/cfg.h"
#include "fault/fault.h"
#include "logring/logring.h"
#include "net/sts_ldap_ca.h" /* sts_ldap_ca_check(): the PEM screen, reused */
#include "net/sts_net.h"
#include "net/sts_syslog_tls_policy.h"
#include "storage/sts_store.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_syslog, CONFIG_STS1000_LOG_LEVEL);

#define SYSLOG_STACK_SIZE 3072
#define SYSLOG_PRIORITY 16
#define SYSLOG_POLL_MS 500
#define SYSLOG_BATCH 16U

/*
 * Connector thread. Priority 16 like the drain — it is not timing-critical and
 * must never preempt a service thread — but NOT a liveness participant, for the
 * reason in the file header. 4 KB: it runs zsock_connect() and the socket
 * layer's handshake, whose record buffers and bignums come from
 * CONFIG_MBEDTLS_HEAP_SIZE rather than this stack; what is on the stack is
 * mbedTLS's X.509 chain verification, which is bounded by the two-certificate
 * chain STS_SYSLOG_CA_PEM_MAX admits.
 */
#define SYSLOG_CONN_STACK_SIZE 4096
#define SYSLOG_CONN_PRIORITY 16

/**
 * SO_SNDTIMEO/SO_RCVTIMEO on an ESTABLISHED session, in ms.
 *
 * This bounds the drain thread, which IS a liveness participant, so it must
 * leave room for a whole poll period inside the deadline. It does not bound the
 * handshake — nothing this code can set does (see the file header).
 */
#define SYSLOG_IO_TIMEOUT_MS 1000

/*
 * The reason `syslog_tls` exists. If this ever becomes true, the connector
 * thread and its stack can be deleted and the connect folded back into
 * drain_once(). It is false at the shipped numbers and this fails the build
 * rather than the board if someone changes one of them.
 */
BUILD_ASSERT(!STS_SYSLOG_CONNECT_FITS_LIVENESS(
		     CONFIG_NET_SOCKETS_CONNECT_TIMEOUT, SYSLOG_POLL_MS,
		     CONFIG_STS1000_LIVENESS_DEADLINE_MS),
	     "a TLS connect now fits inside the liveness deadline; the "
	     "separate connector thread is no longer required");

/* The drain thread's own bounded work must still fit, with a poll period to
 * spare. SYSLOG_BATCH writes at SYSLOG_IO_TIMEOUT_MS each cannot: the drain
 * stops at the first failed write (sts_syslog_advance_ok), so the worst case
 * is one timeout per pass, not SYSLOG_BATCH of them. */
BUILD_ASSERT((SYSLOG_IO_TIMEOUT_MS + SYSLOG_POLL_MS) <
		     CONFIG_STS1000_LIVENESS_DEADLINE_MS,
	     "one blocked syslog write outlasts the liveness deadline");

static char host[64];
static char appname[LOGR_APPNAME_MAX];
static char dest_host[64];
static uint16_t dest_port;
static bool enabled;
static bool want_tls;

static struct sockaddr_storage dest;
static socklen_t dest_len;
static bool dest_resolved;

/** Resolved transport. Written by the drain thread, read by both. */
static sts_syslog_tx_t tx = STS_SYSLOG_TX_OFF;

static int udp_sock = -1;
static uint32_t cursor;

/* Connector/drain shared state. See the ownership note in the file header. */
static struct k_mutex link_lock;
static int tls_sock = -1;    /* published by the connector, closed by drain */
static bool link_up;         /* a usable session exists */
static uint32_t conn_fails;  /* consecutive failures, drives the backoff */
static uint32_t last_try_ms; /* uptime of the last connect attempt */

/** Set by sts_syslog_reapply(); consumed by the drain thread. */
static atomic_t cfg_dirty = ATOMIC_INIT(0);

static struct {
	uint32_t sent;
	uint32_t dropped;
	uint32_t gaps;
	uint32_t reconnects;
	uint32_t handshake_fails;
} gstat;
static struct k_spinlock stat_lock;

static K_THREAD_STACK_DEFINE(syslog_stack, SYSLOG_STACK_SIZE);
static struct k_thread syslog_thread;
static K_THREAD_STACK_DEFINE(syslog_conn_stack, SYSLOG_CONN_STACK_SIZE);
static struct k_thread syslog_conn_thread;
static int live_id = -1;
static bool started;

/* Edge-detect for the operator-visible alarm, so a persistent refusal costs one
 * log line and not one every poll — the sender must not flood the very ring it
 * is failing to drain. */
static bool alarm_on;

/* ------------------------------------------------------------------------- */
/* trust anchor                                                              */
/* ------------------------------------------------------------------------- */
/*
 * The operator-installed CA that lets the collector's certificate be verified.
 * Storage and Zephyr binding are modelled on sts_aaa.c's LDAPS anchor —
 * including the requirement that the PEM buffer be file-scope and live for the
 * credential's lifetime, because tls_credentials.c stores the POINTER and
 * copies nothing.
 *
 * The screen is sts_ldap_ca_check() from sts_ldap_ca.h, called verbatim. Its
 * name says LDAP but what it decides is "is this a PEM certificate blob, of a
 * sane size, with no private key smuggled into it" — which is the same question
 * here, and is already covered by test_ldap_ca.c. A second copy of that logic
 * under a syslog-flavoured name would be a second copy to get wrong.
 */

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)

BUILD_ASSERT(STS_SYSLOG_CA_PEM_MAX <= STS_LDAP_CA_PEM_MAX,
	     "sts_ldap_ca_check() screens against its own ceiling; a larger "
	     "syslog ceiling would silently reject anchors this file accepts");

static char g_ca_pem[STS_SYSLOG_CA_PEM_MAX + 1U];
static size_t g_ca_len;
static bool g_ca_ready;
static bool g_ca_scanned;
static struct k_mutex ca_lock;

static int ca_file_read(char *buf, size_t cap, size_t *out_len)
{
	struct fs_file_t f;
	ssize_t n;
	int rc;

	if (!sts_fs_ready()) {
		return -EROFS;
	}
	fs_file_t_init(&f);
	rc = fs_open(&f, STS_SYSLOG_CA_PATH, FS_O_READ);
	if (rc != 0) {
		return rc;
	}
	n = fs_read(&f, buf, cap);
	(void)fs_close(&f);
	if (n < 0) {
		return (int)n;
	}
	*out_len = (size_t)n;
	return 0;
}

static int ca_file_write(const char *buf, size_t len)
{
	struct fs_file_t f;
	ssize_t n;
	int rc;

	if (!sts_fs_ready()) {
		return -EROFS;
	}
	rc = fs_mkdir(STS_SYSLOG_CA_DIR);
	if (rc != 0 && rc != -EEXIST) {
		LOG_WRN("mkdir %s: %d", STS_SYSLOG_CA_DIR, rc);
	}
	fs_file_t_init(&f);
	rc = fs_open(&f, STS_SYSLOG_CA_PATH,
		     FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	if (rc != 0) {
		return rc;
	}
	n = fs_write(&f, buf, len);
	if (n >= 0) {
		rc = fs_sync(&f);
	}
	(void)fs_close(&f);
	if (n < 0) {
		return (int)n;
	}
	if ((size_t)n != len) {
		return -EIO;
	}
	return rc;
}

/** Drop the anchor from the credential store, then from RAM. Order matters. */
static void ca_forget(void)
{
	(void)tls_credential_delete(STS_SYSLOG_CA_SEC_TAG,
				    TLS_CREDENTIAL_CA_CERTIFICATE);
	memset(g_ca_pem, 0, sizeof(g_ca_pem));
	g_ca_len = 0U;
	g_ca_ready = false;
}

/**
 * Screen, parse, adopt. Caller holds ca_lock.
 *
 * The screen runs on the CALLER's buffer so a rejected blob never displaces a
 * working anchor; only a blob that both screens and parses reaches g_ca_pem.
 */
static int ca_adopt(const char *pem, size_t len)
{
	mbedtls_x509_crt crt;
	sts_ldap_ca_verdict_t v;
	int rc;

	v = sts_ldap_ca_check(pem, len);
	if (v != STS_LDAP_CA_OK) {
		LOG_ERR("syslog ca: refused — %s", sts_ldap_ca_reason(v));
		switch (v) {
		case STS_LDAP_CA_TOO_BIG:
			return -EFBIG;
		case STS_LDAP_CA_HAS_KEY:
			return -EPERM;
		default:
			return -EBADMSG;
		}
	}

	ca_forget();
	memcpy(g_ca_pem, pem, len);
	g_ca_pem[len] = '\0';
	g_ca_len = len;

	/* mbedTLS wants the NUL counted for PEM input. */
	mbedtls_x509_crt_init(&crt);
	rc = mbedtls_x509_crt_parse(&crt, (const unsigned char *)g_ca_pem,
				    g_ca_len + 1U);
	if (rc != 0) {
		if (rc > 0) {
			LOG_ERR("syslog ca: %d certificate block(s) did not "
				"parse",
				rc);
		} else {
			LOG_ERR("syslog ca: parse: -0x%04x", (unsigned int)-rc);
		}
		mbedtls_x509_crt_free(&crt);
		ca_forget();
		return -EBADMSG;
	}
	mbedtls_x509_crt_free(&crt);

	rc = tls_credential_add(STS_SYSLOG_CA_SEC_TAG,
				TLS_CREDENTIAL_CA_CERTIFICATE, g_ca_pem,
				g_ca_len + 1U);
	if (rc != 0) {
		LOG_ERR("syslog ca: tls_credential_add: %d", rc);
		ca_forget();
		return rc;
	}
	g_ca_ready = true;
	return 0;
}

/** Read the persisted anchor, once /lfs is available. Caller holds ca_lock. */
static void ca_load(void)
{
	char buf[STS_SYSLOG_CA_PEM_MAX];
	size_t n = 0U;
	int rc;

	if (!sts_fs_ready()) {
		return; /* not scanned: /lfs may still mount */
	}
	g_ca_scanned = true;

	rc = ca_file_read(buf, sizeof(buf), &n);
	if (rc != 0) {
		if (rc != -ENOENT) {
			LOG_WRN("syslog ca: %s unreadable (%d)",
				STS_SYSLOG_CA_PATH, rc);
		}
		return;
	}
	if (ca_adopt(buf, n) != 0) {
		LOG_ERR("syslog ca: %s is unusable; TLS syslog will refuse "
			"until a good anchor is installed",
			STS_SYSLOG_CA_PATH);
		return;
	}
	LOG_INF("syslog ca: %s loaded (%zu B)", STS_SYSLOG_CA_PATH, n);
}

/** Is an anchor live? Loads it lazily the first time /lfs is available. */
static bool ca_ready(void)
{
	bool r;

	k_mutex_lock(&ca_lock, K_FOREVER);
	if (!g_ca_scanned) {
		ca_load();
	}
	r = g_ca_ready;
	k_mutex_unlock(&ca_lock);
	return r;
}

int sts_syslog_ca_install(const char *pem, size_t len)
{
	int rc;

	if (pem == NULL || len == 0U) {
		return -EBADMSG;
	}

	k_mutex_lock(&ca_lock, K_FOREVER);
	rc = ca_adopt(pem, len);
	if (rc != 0) {
		/* Nothing is live now, so nothing on /lfs should claim to be:
		 * leaving the old file behind would resurrect the rejected
		 * operator's previous anchor at the next boot. */
		if (sts_fs_ready()) {
			int frc = fs_unlink(STS_SYSLOG_CA_PATH);

			if (!sts_ldap_ca_unlink_ok(frc)) {
				LOG_WRN("unlink %s: %d", STS_SYSLOG_CA_PATH,
					frc);
			}
		}
		g_ca_scanned = true;
		k_mutex_unlock(&ca_lock);
		return rc;
	}

	g_ca_scanned = true;
	if (ca_file_write(g_ca_pem, g_ca_len) != 0) {
		LOG_WRN("syslog ca: accepted and live, but NOT persisted; it "
			"is gone at the next reboot");
		rc = -EROFS;
	}
	k_mutex_unlock(&ca_lock);

	sts_log((uint8_t)LOGR_SUB_SEC, (uint8_t)LOGR_NOTICE,
		"syslog ca installed (%u B)", (unsigned int)len);

	/* A refusal may have just become a transport. Re-resolve promptly
	 * rather than at the next config write. */
	atomic_set(&cfg_dirty, 1);
	return rc;
}

int sts_syslog_ca_erase(void)
{
	int frc = 0;

	k_mutex_lock(&ca_lock, K_FOREVER);
	if (sts_fs_ready()) {
		frc = fs_unlink(STS_SYSLOG_CA_PATH);
		if (!sts_ldap_ca_unlink_ok(frc)) {
			LOG_WRN("unlink %s: %d", STS_SYSLOG_CA_PATH, frc);
		} else {
			frc = 0;
		}
	}
	ca_forget();
	g_ca_scanned = true;
	k_mutex_unlock(&ca_lock);

	atomic_set(&cfg_dirty, 1);
	return frc;
}

bool sts_syslog_ca_present(void)
{
	return ca_ready();
}

#else /* !CONFIG_NET_SOCKETS_SOCKOPT_TLS */

static inline bool ca_ready(void)
{
	return false;
}

int sts_syslog_ca_install(const char *pem, size_t len)
{
	ARG_UNUSED(pem);
	ARG_UNUSED(len);
	return -ENOTSUP;
}

int sts_syslog_ca_erase(void)
{
	return 0;
}

bool sts_syslog_ca_present(void)
{
	return false;
}

#endif /* CONFIG_NET_SOCKETS_SOCKOPT_TLS */

/* ------------------------------------------------------------------------- */
/* link teardown / alarm                                                     */
/* ------------------------------------------------------------------------- */

/**
 * Tear down a published session. DRAIN THREAD ONLY.
 *
 * The connector never closes a published fd, so there is no window in which
 * both threads hold the same descriptor number — which, once the number is
 * recycled by the next zsock_socket(), is how a "harmless" double close becomes
 * a write into somebody else's connection.
 */
static void link_drop(const char *why)
{
	int fd;

	k_mutex_lock(&link_lock, K_FOREVER);
	fd = tls_sock;
	tls_sock = -1;
	link_up = false;
	/* The attempt clock restarts here so the backoff is measured from the
	 * loss, not from the last connect that succeeded. */
	last_try_ms = k_uptime_get_32();
	k_mutex_unlock(&link_lock);

	if (fd >= 0) {
		(void)zsock_close(fd);
		LOG_WRN("syslog: TLS session down (%s)", why);
	}
}

static bool link_is_up(void)
{
	bool up;

	k_mutex_lock(&link_lock, K_FOREVER);
	up = link_up;
	k_mutex_unlock(&link_lock);
	return up;
}

/**
 * Drive the operator-visible alarm from the policy, on edges only.
 *
 * FAULT_ALARM_SYSLOG_DOWN is what makes the failure something other than a
 * LOG_WRN nobody reads: it appears in the alarm bitmap the panel, the REST
 * status document and the SNMP agent all render, and it latches.
 */
static void alarm_update(void)
{
	bool want = sts_syslog_alarm(tx, link_is_up());

	if (want == alarm_on) {
		return;
	}
	alarm_on = want;
	(void)sts_alarm_set((uint8_t)FAULT_ALARM_SYSLOG_DOWN, want);
	if (want) {
		sts_log((uint8_t)LOGR_SUB_NET, (uint8_t)LOGR_ERR,
			"remote syslog is NOT delivering: %s",
			(tx == STS_SYSLOG_TX_REFUSED) ? "refused" :
							"session down");
	} else {
		sts_log((uint8_t)LOGR_SUB_NET, (uint8_t)LOGR_NOTICE,
			"remote syslog delivering again");
	}
}

/* ------------------------------------------------------------------------- */
/* configuration                                                             */
/* ------------------------------------------------------------------------- */

/** Resolve the configured host into @ref dest for the given transport. */
static bool resolve_dest(const char *h, int socktype)
{
	struct zsock_addrinfo hints;
	struct zsock_addrinfo *res = NULL;
	char port_s[8];
	int rc;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = socktype;

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

/**
 * Re-read cfg and re-resolve the transport. DRAIN THREAD ONLY.
 *
 * sts_syslog_reapply() used to run this on whatever thread applied the config
 * write, mutating the destination and the enable flag under the drain thread's
 * feet. That was survivable when the worst outcome was one datagram to a stale
 * address; it is not survivable now that the same state decides whether a
 * socket is a TLS socket. So the setter only raises a flag.
 */
static void reload(void)
{
	sts_syslog_tx_in_t in;
	sts_syslog_tx_t prev = tx;
	uint16_t cfg_port;

	enabled = sts_net_cfg_bool(CFG_ID_LOG_SYSLOG_EN, false);
	want_tls = sts_net_cfg_bool(CFG_ID_LOG_SYSLOG_TLS, false);
	cfg_port = (uint16_t)sts_net_cfg_u64(CFG_ID_LOG_SYSLOG_PORT,
					     STS_SYSLOG_PORT_UDP);
	(void)sts_net_cfg_str(CFG_ID_LOG_SYSLOG_HOST, dest_host,
			      sizeof(dest_host));

	in.enabled = enabled;
	in.tls = want_tls;
	in.host_set = (dest_host[0] != '\0');
	in.ca_ready = ca_ready();
	in.tls_built = IS_ENABLED(CONFIG_NET_SOCKETS_SOCKOPT_TLS);

	/* Every reload invalidates a live session, not only a transport change:
	 * the host and the port may have moved under a transport that did not,
	 * and a session that survived that would keep shipping the operator's
	 * records to the collector they just stopped trusting. Reloads happen
	 * on a config write or a CA install, so the cost is irrelevant. */
	if (link_is_up()) {
		link_drop("configuration changed");
	}

	/* The connector thread reads the whole target atomically under
	 * link_lock; publishing it field-by-field would let a handshake go out
	 * with the new port and the old TLS_HOSTNAME. */
	k_mutex_lock(&link_lock, K_FOREVER);
	tx = sts_syslog_tx(&in);
	dest_port = sts_syslog_port(want_tls, cfg_port);
	dest_resolved = false;
	k_mutex_unlock(&link_lock);

	if (tx == STS_SYSLOG_TX_OFF || tx == STS_SYSLOG_TX_REFUSED) {
		if (tx == STS_SYSLOG_TX_REFUSED && prev != tx) {
			const char *why = sts_syslog_refusal(&in);

			/* ERR, not WRN: the operator asked for encrypted
			 * remote logging and is getting no remote logging. */
			LOG_ERR("syslog: %s", why);
			sts_log((uint8_t)LOGR_SUB_SEC, (uint8_t)LOGR_ERR,
				"syslog: %s", why);
		}
		alarm_update();
		return;
	}

	(void)sts_net_cfg_str(CFG_ID_NET_HOSTNAME, host, sizeof(host));
	if (host[0] == '\0') {
		strcpy(host, "meridian");
	}
	strcpy(appname, "sts1000");

	/* Resolution writes `dest`, which the connector also reads, so it is
	 * published under the same lock and only once it is complete. */
	{
		bool ok = resolve_dest(dest_host, sts_syslog_tx_is_stream(tx) ?
							  SOCK_STREAM :
							  SOCK_DGRAM);

		k_mutex_lock(&link_lock, K_FOREVER);
		dest_resolved = ok;
		k_mutex_unlock(&link_lock);
		if (!ok) {
			LOG_WRN("syslog host '%s' unresolved", dest_host);
		}
	}
	alarm_update();
}

void sts_syslog_reapply(void)
{
	if (started) {
		atomic_set(&cfg_dirty, 1);
	}
}

/* ------------------------------------------------------------------------- */
/* the connector thread                                                      */
/* ------------------------------------------------------------------------- */

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)

static void set_timeout(int fd, uint32_t ms)
{
	struct zsock_timeval tv;

	tv.tv_sec = (long)(ms / 1000U);
	tv.tv_usec = (long)((ms % 1000U) * 1000U);
	(void)zsock_setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	(void)zsock_setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

/**
 * Open, arm and connect one TLS session. Returns the fd or a negative errno.
 *
 * The two setsockopt calls are the entire difference between encrypted
 * transport and encrypted-looking transport, so neither is best-effort:
 *
 *   TLS_SEC_TAG_LIST  supplies the CA chain. Without it Zephyr's
 *                     tls_mbedtls_set_credentials() never calls
 *                     tls_set_ca_chain(), and since mbedTLS's client default is
 *                     MBEDTLS_SSL_VERIFY_REQUIRED the handshake cannot
 *                     complete — a hard failure, but an unattributable one.
 *   TLS_HOSTNAME      makes the certificate's NAME checked, not merely its
 *                     chain. A chain check alone accepts ANY host the CA ever
 *                     signed, which on an internal CA is every server in the
 *                     estate. Without it sockets_tls.c forces
 *                     mbedtls_ssl_set_hostname(ssl, "").
 *
 * TLS_PEER_VERIFY is deliberately absent: Zephyr leaves verify_level at -1 and
 * only overrides mbedtls_ssl_conf_authmode() when it was set, so the client
 * default of REQUIRED already stands. Any value passed here would be an
 * opportunity to pass the wrong one. Same argument as sts_ldap_ca.h.
 */
typedef struct {
	struct sockaddr_storage addr;
	socklen_t addr_len;
	char host[sizeof(dest_host)];
	uint16_t port;
} conn_target_t;

static int tls_connect_once(const conn_target_t *t)
{
	static const sec_tag_t tags[] = { STS_SYSLOG_CA_SEC_TAG };
	int fd;
	int rc;

	/* IPPROTO_TLS_1_2 names the socket TYPE, not a version floor: Zephyr's
	 * protocol_check() maps every IPPROTO_TLS_1_* to IPPROTO_TCP and the
	 * negotiated version comes from the mbedTLS config, which web.conf pins
	 * to 1.3 only. Same call shape as sts_aaa.c and sts_web.c. */
	fd = zsock_socket(t->addr.ss_family, SOCK_STREAM, IPPROTO_TLS_1_2);
	if (fd < 0) {
		return -errno;
	}

	if (zsock_setsockopt(fd, SOL_TLS, TLS_SEC_TAG_LIST, tags,
			     sizeof(tags)) < 0) {
		rc = -errno;
		LOG_ERR("syslog: TLS_SEC_TAG_LIST: %d", -rc);
		(void)zsock_close(fd);
		return rc;
	}
	if (zsock_setsockopt(fd, SOL_TLS, TLS_HOSTNAME, t->host,
			     strlen(t->host) + 1U) < 0) {
		rc = -errno;
		LOG_ERR("syslog: TLS_HOSTNAME: %d", -rc);
		(void)zsock_close(fd);
		return rc;
	}

	set_timeout(fd, SYSLOG_IO_TIMEOUT_MS);

	if (zsock_connect(fd, (struct sockaddr *)&t->addr, t->addr_len) < 0) {
		rc = -errno;
		(void)zsock_close(fd);
		return rc;
	}
	return fd;
}

static void conn_loop(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		conn_target_t target;
		bool due;
		uint32_t fails;
		int fd;

		k_sleep(K_MSEC(SYSLOG_POLL_MS));

		/* One critical section takes the WHOLE target plus the
		 * retry decision. Reading tx, dest and dest_host separately
		 * would let a reload land between them and send a handshake
		 * out with the new address and the previous host name — a
		 * certificate check against the wrong identity, which is the
		 * one failure this transport exists to prevent. */
		k_mutex_lock(&link_lock, K_FOREVER);
		fails = conn_fails;
		due = (tx == STS_SYSLOG_TX_TLS) && dest_resolved && !link_up &&
		      (tls_sock < 0) &&
		      sts_syslog_retry_due(k_uptime_get_32(), last_try_ms,
					   fails);
		if (due) {
			memcpy(&target.addr, &dest, sizeof(target.addr));
			target.addr_len = dest_len;
			(void)strncpy(target.host, dest_host,
				      sizeof(target.host) - 1U);
			target.host[sizeof(target.host) - 1U] = '\0';
			target.port = dest_port;
			last_try_ms = k_uptime_get_32();
		}
		k_mutex_unlock(&link_lock);

		if (!due) {
			continue;
		}

		fd = tls_connect_once(&target);

		k_mutex_lock(&link_lock, K_FOREVER);
		/* The drain thread may have re-pointed the transport while the
		 * handshake was in flight; publishing into that would leak a
		 * session nothing owns. */
		if (fd >= 0 && tx == STS_SYSLOG_TX_TLS && tls_sock < 0) {
			tls_sock = fd;
			link_up = true;
			conn_fails = 0U;
			fd = -1;
		} else if (fd >= 0) {
			/* Not published: this thread still owns it. */
			(void)zsock_close(fd);
			fd = -1;
			conn_fails++;
		} else {
			conn_fails++;
		}
		/* The retry clock runs from the END of a failed attempt, not
		 * its start: a 6 s handshake timeout would otherwise consume
		 * the whole first backoff and turn the first three retries
		 * into a spin. */
		last_try_ms = k_uptime_get_32();
		k_mutex_unlock(&link_lock);

		if (link_is_up()) {
			K_SPINLOCK(&stat_lock) {
				gstat.reconnects++;
			}
			LOG_INF("syslog: TLS session to %s:%u established",
				target.host, (unsigned int)target.port);
		} else {
			K_SPINLOCK(&stat_lock) {
				gstat.handshake_fails++;
			}
			/* Deliberately LOG_DBG and not LOG_WRN: the alarm and
			 * the one-shot NOTICE from alarm_update() carry this
			 * to the operator. Warning on every retry would have
			 * the sender fill the ring it cannot drain. */
			LOG_DBG("syslog: TLS connect failed (retry in %u ms)",
				(unsigned int)sts_syslog_backoff_ms(
					conn_fails));
		}
	}
}

#endif /* CONFIG_NET_SOCKETS_SOCKOPT_TLS */

/* ------------------------------------------------------------------------- */
/* one record -> the wire                                                    */
/* ------------------------------------------------------------------------- */

/**
 * Render, frame and send one record.
 *
 * @return true when the cursor may advance past this record. The distinction
 *         between "delivered" and "may advance" belongs to
 *         sts_syslog_advance_ok(), not here.
 */
static bool send_record(const logr_rec_t *r)
{
	char line[LOGR_RENDER_MAX];
	struct timespec ts;
	uint64_t unix_s = 0U;
	uint32_t frac_us = 0U;
	int n;
	bool ok;

	if (sys_clock_gettime(SYS_CLOCK_REALTIME, &ts) == 0) {
		unix_s = (uint64_t)ts.tv_sec;
		frac_us = (uint32_t)(ts.tv_nsec / 1000);
	}

	n = logr_render_5424(r, LOGR_FACILITY_LOCAL0, host, appname, unix_s,
			     frac_us, line, sizeof(line));
	if (n < 0) {
		/* Unrenderable: dropping it is the only option, and holding the
		 * cursor on it would wedge the reader on one bad record. */
		K_SPINLOCK(&stat_lock) {
			gstat.dropped++;
		}
		return true;
	}

	if (sts_syslog_tx_is_stream(tx)) {
#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
		uint8_t frame[LOGR_RENDER_MAX + STS_SYSLOG_FRAME_OVERHEAD];
		size_t flen = 0U;
		size_t off = 0U;
		int fd;

		if (sts_syslog_frame((const uint8_t *)line, (size_t)n, frame,
				     sizeof(frame), &flen) != 0) {
			K_SPINLOCK(&stat_lock) {
				gstat.dropped++;
			}
			return true;
		}

		k_mutex_lock(&link_lock, K_FOREVER);
		fd = link_up ? tls_sock : -1;
		k_mutex_unlock(&link_lock);
		if (fd < 0) {
			return false;
		}

		/* A short write is not a failure, it is the rest of the frame.
		 * Abandoning it here would leave a partial octet-counted record
		 * on the wire, and octet counting has no resynchronisation
		 * point — the collector would misparse every record after it
		 * for the life of the session. */
		while (off < flen) {
			ssize_t w = zsock_send(fd, &frame[off], flen - off, 0);

			if (w <= 0) {
				link_drop("write failed");
				K_SPINLOCK(&stat_lock) {
					gstat.dropped++;
				}
				return false;
			}
			off += (size_t)w;
		}
		ok = true;
#else
		ok = false;
#endif
	} else {
		if (udp_sock < 0) {
			return true;
		}
		ok = zsock_sendto(udp_sock, line, (size_t)n, 0,
				  (struct sockaddr *)&dest, dest_len) >= 0;
		if (!ok) {
			K_SPINLOCK(&stat_lock) {
				gstat.dropped++;
			}
		}
	}

	if (ok) {
		K_SPINLOCK(&stat_lock) {
			gstat.sent++;
		}
	}
	return sts_syslog_advance_ok(tx, ok);
}

static void drain_once(void)
{
	logr_rec_t recs[SYSLOG_BATCH];
	logr_t *ring = sts_logring();
	uint16_t got = 0U;
	uint32_t next = cursor;
	uint32_t gap = 0U;
	uint32_t base;
	size_t i;

	if (ring == NULL || !dest_resolved) {
		return;
	}
	if (sts_syslog_tx_is_stream(tx)) {
		if (!link_is_up()) {
			return; /* spooling: the cursor stays put */
		}
	} else if (udp_sock < 0) {
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

	/* logr_tail() may have skipped forward past overwritten records, so the
	 * first record returned is `next - got`, NOT `cursor`. Resuming from
	 * `cursor` after a partial send would re-read records the ring no
	 * longer holds and re-report the same gap every poll. */
	base = next - (uint32_t)got;

	for (i = 0U; i < got; i++) {
		if (!send_record(&recs[i])) {
			/* Stop at the first record the transport did not take
			 * and leave the cursor ON it, so the next drain starts
			 * there: the ring IS the spool. */
			cursor = base + (uint32_t)i;
			return;
		}
	}
	cursor = next;
}

static void syslog_loop(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		if (atomic_cas(&cfg_dirty, 1, 0)) {
			reload();
		}
		if (tx == STS_SYSLOG_TX_UDP || tx == STS_SYSLOG_TX_TLS) {
			drain_once();
		}
		alarm_update();
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
		out->reconnects = gstat.reconnects;
		out->handshake_fails = gstat.handshake_fails;
	}
	out->enabled = enabled;
	out->resolved = dest_resolved;
	out->tls = (tx == STS_SYSLOG_TX_TLS);
	out->refused = (tx == STS_SYSLOG_TX_REFUSED);
	out->connected = link_is_up();
	out->ca_present = sts_syslog_ca_present();
}

int sts_syslog_start(void)
{
	logr_t *ring = sts_logring();

	k_mutex_init(&link_lock);
#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	k_mutex_init(&ca_lock);
#endif

	/* Start reading from the ring's current head, not sequence 0: the
	 * boot-time log belongs to the local console and the NOR spool, not to
	 * a remote collector that was not listening for it. */
	if (ring != NULL) {
		cursor = logr_head(ring);
	}

	/* The datagram socket is opened unconditionally and used only when the
	 * transport is UDP. It costs one context and keeps the failure of
	 * socket() out of the per-poll path. */
	udp_sock = zsock_socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	if (udp_sock < 0) {
		/* v6 socket also carries v4-mapped destinations; if even that
		 * fails there is no datagram transport, but the thread still
		 * runs so a later cfg change can retry. */
		LOG_WRN("syslog socket: %d", errno);
	}

	started = true;
	reload();

	live_id = sts_liveness_register("syslog");

	k_thread_create(&syslog_thread, syslog_stack,
			K_THREAD_STACK_SIZEOF(syslog_stack), syslog_loop, NULL,
			NULL, NULL, SYSLOG_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&syslog_thread, "syslog");

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	/* Not a liveness participant, by design — see the file header. */
	k_thread_create(&syslog_conn_thread, syslog_conn_stack,
			K_THREAD_STACK_SIZEOF(syslog_conn_stack), conn_loop,
			NULL, NULL, NULL, SYSLOG_CONN_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&syslog_conn_thread, "syslog_tls");
#else
	ARG_UNUSED(syslog_conn_stack);
	ARG_UNUSED(syslog_conn_thread);
#endif

	LOG_INF("syslog sender ready (enabled=%d, transport=%d)", (int)enabled,
		(int)tx);
	return 0;
}
