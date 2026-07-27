/*
 * STS1000 "Meridian" — AAA transport: RADIUS / TACACS+ / LDAP clients (spec §9.4).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * core/auth owns the decision (the backend chain, the role mapping, the cache
 * and the lockout) and core/auth's radius.h / tacacs.h / ldap.h own the wire
 * formats. This file owns exactly the part that cannot be platform-neutral: the
 * sockets, the timeouts and the retries.
 *
 * The whole surface is one function pointer — `auth_remote_fn` — which
 * core/auth calls once per remote backend in the chain. Everything else here is
 * private.
 *
 * ---------------------------------------------------------------------------
 * The three-valued answer, and why it matters
 * ---------------------------------------------------------------------------
 *
 * Every backend here returns one of three things, and confusing them is a
 * security bug:
 *
 *   0                accept — the server said yes and *_role holds the role
 *   -EACCES          reject — the server said no. The chain stops here.
 *   -EHOSTUNREACH    unavailable — not configured, no answer, or a reply we
 *                    could not authenticate. The chain tries the next backend.
 *
 * A forged or mis-keyed reply is *unavailable*, never accept and never reject:
 * an attacker who can inject UDP must not be able to produce either verdict.
 *
 * ---------------------------------------------------------------------------
 * Bounded by construction
 * ---------------------------------------------------------------------------
 *
 * One exchange at a time, under a mutex, with a per-attempt receive timeout and
 * a hard cap on retries — so a slow or hostile server bounds the login latency
 * instead of pinning a thread. Buffers are file-scope statics rather than stack
 * arrays because the caller may be the web thread with a modest stack; the mutex
 * is what makes that safe.
 *
 * ---------------------------------------------------------------------------
 * TLS for LDAP
 * ---------------------------------------------------------------------------
 *
 * `sec.ldap.mode` selects plain (389), StartTLS or LDAPS (636). The two TLS
 * modes are compiled only when Zephyr's TLS socket option is available
 * (CONFIG_NET_SOCKETS_SOCKOPT_TLS); the project does not enable the TLS 1.3
 * stack by default (see the block at the end of conf/net.conf), so a build
 * without it reports LDAPS/StartTLS as unavailable rather than silently binding
 * in the clear. Falling back to plaintext would send the bind password over the
 * wire, which is exactly the failure the operator selected TLS to avoid.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>

#include "auth/auth.h"
#include "auth/ldap.h"
#include "auth/radius.h"
#include "auth/tacacs.h"
#include "cfg/cfg.h"
#include "net/sts_aaa.h"
/*
 * Deliberately NOT including net/sts_net.h.
 *
 * That header offers cfg convenience readers this file would otherwise use, but
 * it currently also typedefs `sts_gnss_wallclock_t`, which zephyr/sts_app.h
 * typedefs as well — so any translation unit including both fails to compile.
 * This file needs sts_app.h (for sts_cfg(), sts_log() and sts_mono_ms()), so it
 * reads cfg through core/cfg directly via the three small helpers below instead.
 * That is a handful of lines and it keeps this file building while the duplicate
 * typedef is resolved by the areas that own those two headers.
 *
 * sts_store.h is the one header this area legitimately takes from another (its
 * own comment declares it a public façade): it is where the net area gets its
 * port_crypto_t rather than standing up a second mbedTLS binding.
 */
#include "storage/sts_store.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_aaa, CONFIG_STS1000_LOG_LEVEL);

/* Longest host name / DN / attribute read from cfg. */
#define HOST_MAX 64U
#define DN_MAX 64U
#define ATTR_MAX 32U

/* One exchange at a time; these buffers are shared under g_lock. */
static uint8_t g_tx[RADIUS_BUF_MAX > TACACS_PKT_MAX
			    ? (RADIUS_BUF_MAX > LDAP_BUF_MAX ? RADIUS_BUF_MAX
							     : LDAP_BUF_MAX)
			    : (TACACS_PKT_MAX > LDAP_BUF_MAX ? TACACS_PKT_MAX
							     : LDAP_BUF_MAX)];
static uint8_t g_rx[sizeof(g_tx)];
static struct k_mutex g_lock;

static auth_ctx_t g_auth;
static uint8_t g_admin_blob[AUTH_PW_BLOB_LEN];
static size_t g_admin_blob_len;
static bool g_started;

/* Cached backend configuration, refreshed by sts_aaa_reapply(). */
typedef struct {
	char host[HOST_MAX];
	uint16_t port;
	uint8_t secret[AUTH_SHARED_MAX];
	size_t secret_len;
	uint16_t timeout_ms;
	uint8_t retries;
} remote_cfg_t;

static remote_cfg_t g_radius;
static remote_cfg_t g_tacacs;
static remote_cfg_t g_ldap;

static struct {
	uint8_t mode; /* 0 plain, 1 StartTLS, 2 LDAPS */
	char base[DN_MAX];
	char user_dn[DN_MAX];
	char bind_dn[DN_MAX];
	uint8_t bind_pw[AUTH_SHARED_MAX];
	size_t bind_pw_len;
	char member_attr[ATTR_MAX];
	char grp_admin[DN_MAX];
	char grp_oper[DN_MAX];
	char grp_view[DN_MAX];
} g_ldap_cfg;

static char g_nas_id[HOST_MAX];

/* ------------------------------------------------------------------------- */
/* cfg plumbing                                                              */
/* ------------------------------------------------------------------------- */

/** A cfg STR key as a NUL-terminated string; empty when absent. */
static void load_str(uint16_t id, char *out, size_t cap)
{
	cfg_ctx_t *c = sts_cfg();
	size_t n = 0U;

	out[0] = '\0';
	if (c == NULL || cap == 0U) {
		return;
	}
	/* cfg stores a STR without a terminator, so leave room for one. */
	if (cfg_get_bytes(c, id, (uint8_t *)out, cap - 1U, &n) != 0) {
		return;
	}
	out[n] = '\0';
}

/** A cfg BLOB key. @p out_len is 0 when the key is absent or empty. */
static void load_blob(uint16_t id, uint8_t *out, size_t cap, size_t *out_len)
{
	cfg_ctx_t *c = sts_cfg();
	size_t n = 0U;

	*out_len = 0U;
	if (c == NULL) {
		return;
	}
	if (cfg_get_bytes(c, id, out, cap, &n) == 0) {
		*out_len = n;
	}
}

/** A cfg numeric key, with a fallback when cfg is not up yet. */
static uint64_t load_u64(uint16_t id, uint64_t dflt)
{
	cfg_ctx_t *c = sts_cfg();
	uint64_t v = dflt;

	if (c != NULL && cfg_get_u64(c, id, &v) != 0) {
		v = dflt;
	}
	return v;
}

void sts_aaa_reapply(void)
{
	auth_cfg_t acfg;
	char order[32];
	int n;

	if (!g_started) {
		return;
	}

	k_mutex_lock(&g_lock, K_FOREVER);

	load_str((uint16_t)CFG_ID_SEC_AAA_NAS_ID, g_nas_id, sizeof(g_nas_id));
	if (g_nas_id[0] == '\0') {
		load_str((uint16_t)CFG_ID_NET_HOSTNAME, g_nas_id,
			 sizeof(g_nas_id));
	}

	load_str((uint16_t)CFG_ID_SEC_RADIUS_HOST, g_radius.host,
		 sizeof(g_radius.host));
	g_radius.port =
		(uint16_t)load_u64((uint16_t)CFG_ID_SEC_RADIUS_PORT,
					  1812U);
	load_blob((uint16_t)CFG_ID_SEC_RADIUS_SECRET, g_radius.secret,
		  sizeof(g_radius.secret), &g_radius.secret_len);
	g_radius.timeout_ms =
		(uint16_t)load_u64((uint16_t)CFG_ID_SEC_RADIUS_TMO_MS,
					  3000U);
	g_radius.retries =
		(uint8_t)load_u64((uint16_t)CFG_ID_SEC_RADIUS_RETRIES,
					 2U);

	load_str((uint16_t)CFG_ID_SEC_TACACS_HOST, g_tacacs.host,
		 sizeof(g_tacacs.host));
	g_tacacs.port =
		(uint16_t)load_u64((uint16_t)CFG_ID_SEC_TACACS_PORT, 49U);
	load_blob((uint16_t)CFG_ID_SEC_TACACS_SECRET, g_tacacs.secret,
		  sizeof(g_tacacs.secret), &g_tacacs.secret_len);
	g_tacacs.timeout_ms =
		(uint16_t)load_u64((uint16_t)CFG_ID_SEC_TACACS_TMO_MS,
					  5000U);

	load_str((uint16_t)CFG_ID_SEC_LDAP_HOST, g_ldap.host,
		 sizeof(g_ldap.host));
	g_ldap.port =
		(uint16_t)load_u64((uint16_t)CFG_ID_SEC_LDAP_PORT, 389U);
	g_ldap.timeout_ms =
		(uint16_t)load_u64((uint16_t)CFG_ID_SEC_LDAP_TMO_MS,
					  5000U);
	g_ldap_cfg.mode =
		(uint8_t)load_u64((uint16_t)CFG_ID_SEC_LDAP_MODE, 0U);
	load_str((uint16_t)CFG_ID_SEC_LDAP_BASE_DN, g_ldap_cfg.base,
		 sizeof(g_ldap_cfg.base));
	load_str((uint16_t)CFG_ID_SEC_LDAP_USER_DN, g_ldap_cfg.user_dn,
		 sizeof(g_ldap_cfg.user_dn));
	load_str((uint16_t)CFG_ID_SEC_LDAP_BIND_DN, g_ldap_cfg.bind_dn,
		 sizeof(g_ldap_cfg.bind_dn));
	load_blob((uint16_t)CFG_ID_SEC_LDAP_BIND_PW, g_ldap_cfg.bind_pw,
		  sizeof(g_ldap_cfg.bind_pw), &g_ldap_cfg.bind_pw_len);
	load_str((uint16_t)CFG_ID_SEC_LDAP_MEMBER_ATTR, g_ldap_cfg.member_attr,
		 sizeof(g_ldap_cfg.member_attr));
	if (g_ldap_cfg.member_attr[0] == '\0') {
		(void)strncpy(g_ldap_cfg.member_attr, "member", ATTR_MAX - 1U);
	}
	load_str((uint16_t)CFG_ID_SEC_LDAP_GRP_ADMIN, g_ldap_cfg.grp_admin,
		 sizeof(g_ldap_cfg.grp_admin));
	load_str((uint16_t)CFG_ID_SEC_LDAP_GRP_OPER, g_ldap_cfg.grp_oper,
		 sizeof(g_ldap_cfg.grp_oper));
	load_str((uint16_t)CFG_ID_SEC_LDAP_GRP_VIEW, g_ldap_cfg.grp_view,
		 sizeof(g_ldap_cfg.grp_view));

	load_blob((uint16_t)CFG_ID_SEC_ADMIN_PW, g_admin_blob,
		  sizeof(g_admin_blob), &g_admin_blob_len);

	memset(&acfg, 0, sizeof(acfg));
	acfg.crypto = sts_port_crypto();
	acfg.remote = sts_aaa_remote;
	acfg.remote_ctx = NULL;
	acfg.local_blob = g_admin_blob;
	acfg.local_blob_len = g_admin_blob_len;
	acfg.local_role = (uint8_t)AUTH_ROLE_ADMIN;
	acfg.cache_ttl_s =
		(uint16_t)load_u64((uint16_t)CFG_ID_SEC_AAA_CACHE_S,
					  300U);
	acfg.lockout_fails =
		(uint8_t)load_u64((uint16_t)CFG_ID_SEC_AAA_LOCK_N, 5U);
	acfg.lockout_s =
		(uint16_t)load_u64((uint16_t)CFG_ID_SEC_AAA_LOCK_S, 300U);

	load_str((uint16_t)CFG_ID_SEC_AAA_ORDER, order, sizeof(order));
	n = auth_order_parse(order, acfg.order, sizeof(acfg.order));
	acfg.n_order = (n > 0) ? (uint8_t)n : 0U;

	(void)auth_reconfigure(&g_auth, &acfg);
	k_mutex_unlock(&g_lock);
}

/* ------------------------------------------------------------------------- */
/* socket helpers                                                            */
/* ------------------------------------------------------------------------- */

/** Resolve @p host / @p port into @p dst. */
static int resolve(const char *host, uint16_t port, int socktype,
		   struct sockaddr_storage *dst, socklen_t *dst_len)
{
	struct zsock_addrinfo hints;
	struct zsock_addrinfo *res = NULL;
	char port_s[8];

	if (host == NULL || host[0] == '\0') {
		return -EHOSTUNREACH;
	}
	(void)snprintf(port_s, sizeof(port_s), "%u", (unsigned int)port);

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = socktype;
	if (zsock_getaddrinfo(host, port_s, &hints, &res) != 0 || res == NULL) {
		LOG_WRN("AAA: '%s' unresolved", host);
		return -EHOSTUNREACH;
	}
	memcpy(dst, res->ai_addr, res->ai_addrlen);
	*dst_len = res->ai_addrlen;
	zsock_freeaddrinfo(res);
	return 0;
}

static void set_timeout(int fd, uint16_t ms)
{
	struct zsock_timeval tv;

	tv.tv_sec = (long)(ms / 1000U);
	tv.tv_usec = (long)((ms % 1000U) * 1000U);
	(void)zsock_setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	(void)zsock_setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

/** Read exactly @p want octets from a stream socket, or fail. */
static int read_exact(int fd, uint8_t *buf, size_t want)
{
	size_t got = 0U;

	while (got < want) {
		ssize_t n = zsock_recv(fd, &buf[got], want - got, 0);

		if (n <= 0) {
			return -EHOSTUNREACH;
		}
		got += (size_t)n;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/* RADIUS                                                                    */
/* ------------------------------------------------------------------------- */

static int do_radius(const char *user, const char *secret, uint8_t *out_role)
{
	const port_crypto_t *crypto = sts_port_crypto();
	struct sockaddr_storage dst;
	socklen_t dst_len = 0U;
	radius_req_t req;
	radius_rsp_t rsp;
	uint8_t authenticator[RADIUS_AUTH_LEN];
	size_t tx_len = 0U;
	int fd = -1;
	int rc;
	unsigned int try_n;
	static uint8_t next_id;

	if (g_radius.host[0] == '\0' || g_radius.secret_len == 0U) {
		return -EHOSTUNREACH;
	}
	if (crypto == NULL || crypto->rand == NULL) {
		return -EHOSTUNREACH;
	}
	/* A predictable Request Authenticator makes both the password hiding and
	 * the reply check forgeable, so no entropy means no RADIUS. */
	if (crypto->rand(crypto->ctx, authenticator, sizeof(authenticator)) !=
	    0) {
		return -EHOSTUNREACH;
	}

	rc = resolve(g_radius.host, g_radius.port, SOCK_DGRAM, &dst, &dst_len);
	if (rc != 0) {
		return rc;
	}

	memset(&req, 0, sizeof(req));
	req.user = user;
	req.secret = secret;
	req.shared = g_radius.secret;
	req.shared_len = g_radius.secret_len;
	req.id = next_id++;
	req.authenticator = authenticator;
	req.nas_id = (g_nas_id[0] != '\0') ? g_nas_id : NULL;
	req.nas_port_type = RADIUS_NPT_VIRTUAL;
	req.service_type = RADIUS_ST_ADMINISTRATIVE;
	req.message_authenticator = true;

	rc = radius_build_access_request(&req, g_tx, RADIUS_BUF_MAX, &tx_len);
	if (rc != 0) {
		LOG_WRN("radius: build %d", rc);
		return -EHOSTUNREACH;
	}

	fd = zsock_socket(dst.ss_family, SOCK_DGRAM, IPPROTO_UDP);
	if (fd < 0) {
		return -EHOSTUNREACH;
	}
	set_timeout(fd, g_radius.timeout_ms);

	rc = -EHOSTUNREACH;
	for (try_n = 0U; try_n <= g_radius.retries; try_n++) {
		ssize_t n;

		if (zsock_sendto(fd, g_tx, tx_len, 0, (struct sockaddr *)&dst,
				 dst_len) < 0) {
			continue;
		}
		n = zsock_recv(fd, g_rx, sizeof(g_rx), 0);
		if (n <= 0) {
			continue;
		}

		/* radius_parse_response() always verifies the Response
		 * Authenticator; a reply that fails it is treated as noise on
		 * the wire and we keep waiting for a real one. */
		if (radius_parse_response(g_rx, (size_t)n, g_radius.secret,
					  g_radius.secret_len, req.id,
					  authenticator, &rsp) != 0) {
			continue;
		}
		if (rsp.code == RADIUS_CODE_ACCESS_ACCEPT) {
			*out_role = rsp.role;
			rc = 0;
		} else if (rsp.code == RADIUS_CODE_ACCESS_REJECT) {
			rc = -EACCES;
		} else {
			/*
			 * Access-Challenge: a multi-round conversation this
			 * client does not conduct. Treating it as unavailable
			 * lets the next backend answer; treating it as a reject
			 * would deny a user the server never denied.
			 */
			LOG_INF("radius: challenge not supported");
			rc = -EHOSTUNREACH;
		}
		break;
	}

	(void)zsock_close(fd);
	return rc;
}

/* ------------------------------------------------------------------------- */
/* TACACS+                                                                   */
/* ------------------------------------------------------------------------- */

/** Receive one TACACS+ packet: the 12-octet header, then its body. */
static int tacacs_recv(int fd, size_t *out_len)
{
	tacacs_hdr_t h;
	int rc;

	rc = read_exact(fd, g_rx, TACACS_HDR_LEN);
	if (rc != 0) {
		return rc;
	}
	rc = tacacs_hdr_parse(g_rx, TACACS_HDR_LEN, &h);
	if (rc != 0) {
		return -EHOSTUNREACH;
	}
	if (h.length != 0U) {
		if ((TACACS_HDR_LEN + h.length) > sizeof(g_rx)) {
			return -EHOSTUNREACH;
		}
		rc = read_exact(fd, &g_rx[TACACS_HDR_LEN], h.length);
		if (rc != 0) {
			return rc;
		}
	}
	*out_len = TACACS_HDR_LEN + h.length;
	return 0;
}

static int do_tacacs(const char *user, const char *secret, uint8_t *out_role)
{
	const port_crypto_t *crypto = sts_port_crypto();
	struct sockaddr_storage dst;
	socklen_t dst_len = 0U;
	tacacs_authen_start_t start;
	tacacs_authen_reply_t reply;
	tacacs_author_req_t areq;
	tacacs_author_rsp_t arsp;
	static const char *const args[] = { "service=sts1000" };
	uint32_t session;
	size_t tx_len = 0U;
	size_t rx_len = 0U;
	int fd = -1;
	int rc;

	if (g_tacacs.host[0] == '\0' || g_tacacs.secret_len == 0U) {
		return -EHOSTUNREACH;
	}
	if (crypto == NULL || crypto->rand == NULL) {
		return -EHOSTUNREACH;
	}
	/* RFC 8907 §4.3: the session id must be unpredictable — it is an input
	 * to the body obfuscation keystream. */
	if (crypto->rand(crypto->ctx, (uint8_t *)&session, sizeof(session)) !=
	    0) {
		return -EHOSTUNREACH;
	}

	rc = resolve(g_tacacs.host, g_tacacs.port, SOCK_STREAM, &dst, &dst_len);
	if (rc != 0) {
		return rc;
	}

	fd = zsock_socket(dst.ss_family, SOCK_STREAM, IPPROTO_TCP);
	if (fd < 0) {
		return -EHOSTUNREACH;
	}
	set_timeout(fd, g_tacacs.timeout_ms);
	if (zsock_connect(fd, (struct sockaddr *)&dst, dst_len) < 0) {
		(void)zsock_close(fd);
		return -EHOSTUNREACH;
	}

	/* --- authentication: a single-round PAP login --- */
	memset(&start, 0, sizeof(start));
	start.session_id = session;
	start.action = TACACS_AUTHEN_LOGIN;
	start.priv_lvl = TACACS_PRIV_USER;
	start.authen_type = TACACS_AUTHEN_TYPE_PAP;
	start.authen_service = TACACS_SVC_LOGIN;
	start.user = user;
	start.port = (g_nas_id[0] != '\0') ? g_nas_id : "mgmt";
	start.rem_addr = "";
	start.password = secret;
	start.single_connect = true;

	rc = tacacs_build_authen_start(&start, g_tacacs.secret,
				       g_tacacs.secret_len, g_tx, sizeof(g_tx),
				       &tx_len);
	if (rc != 0) {
		(void)zsock_close(fd);
		return -EHOSTUNREACH;
	}
	if (zsock_send(fd, g_tx, tx_len, 0) < 0) {
		(void)zsock_close(fd);
		return -EHOSTUNREACH;
	}
	if (tacacs_recv(fd, &rx_len) != 0) {
		(void)zsock_close(fd);
		return -EHOSTUNREACH;
	}
	if (tacacs_parse_authen_reply(g_rx, rx_len, session, 2U,
				      g_tacacs.secret, g_tacacs.secret_len,
				      &reply) != 0) {
		(void)zsock_close(fd);
		return -EHOSTUNREACH;
	}

	if (reply.status == TACACS_AUTHEN_STATUS_FAIL) {
		(void)zsock_close(fd);
		return -EACCES;
	}
	if (reply.status != TACACS_AUTHEN_STATUS_PASS) {
		/*
		 * GETDATA / GETUSER / GETPASS / RESTART / ERROR / FOLLOW. A PAP
		 * login should not produce any of them; a server that does is
		 * asking for a conversation this client does not conduct, so it
		 * counts as unavailable rather than as a denial.
		 */
		LOG_INF("tacacs: authen status 0x%02x unsupported",
			reply.status);
		(void)zsock_close(fd);
		return -EHOSTUNREACH;
	}

	/* --- authorization: ask for the role --- */
	memset(&areq, 0, sizeof(areq));
	areq.session_id = session;
	areq.seq_no = 1U; /* a new exchange on the same connection */
	areq.authen_method = TACACS_AUTHEN_METH_TACACSPLUS;
	areq.priv_lvl = TACACS_PRIV_USER;
	areq.authen_type = TACACS_AUTHEN_TYPE_PAP;
	areq.authen_service = TACACS_SVC_LOGIN;
	areq.user = user;
	areq.port = start.port;
	areq.rem_addr = "";
	areq.args = args;
	areq.n_args = 1U;

	*out_role = (uint8_t)AUTH_ROLE_VIEWER;
	if (tacacs_build_author_request(&areq, g_tacacs.secret,
					g_tacacs.secret_len, g_tx, sizeof(g_tx),
					&tx_len) == 0 &&
	    zsock_send(fd, g_tx, tx_len, 0) >= 0 &&
	    tacacs_recv(fd, &rx_len) == 0 &&
	    tacacs_parse_author_response(g_rx, rx_len, session, 2U,
					 g_tacacs.secret, g_tacacs.secret_len,
					 &arsp) == 0) {
		if (arsp.status == TACACS_AUTHOR_STATUS_PASS_ADD ||
		    arsp.status == TACACS_AUTHOR_STATUS_PASS_REPL) {
			*out_role = tacacs_role_from_args(&arsp, NULL);
		} else if (arsp.status == TACACS_AUTHOR_STATUS_FAIL) {
			/* Authenticated but not authorised for this service. */
			(void)zsock_close(fd);
			return -EACCES;
		}
	}
	/* An authorization step that did not complete leaves the least
	 * privilege, which is the safe reading of "authenticated, role
	 * unknown". */

	(void)zsock_close(fd);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* LDAP                                                                      */
/* ------------------------------------------------------------------------- */

/** Receive one LDAPMessage into g_rx. */
static int ldap_recv(int fd, size_t *out_len)
{
	size_t got = 0U;
	int want;

	/* Read enough to learn the length, then the rest. */
	while (got < 6U) {
		ssize_t n = zsock_recv(fd, &g_rx[got], 6U - got, 0);

		if (n <= 0) {
			return -EHOSTUNREACH;
		}
		got += (size_t)n;
	}
	want = ldap_msg_len(g_rx, got);
	if (want < 0) {
		return -EHOSTUNREACH;
	}
	if ((size_t)want > sizeof(g_rx)) {
		return -EHOSTUNREACH;
	}
	if (got < (size_t)want) {
		if (read_exact(fd, &g_rx[got], (size_t)want - got) != 0) {
			return -EHOSTUNREACH;
		}
	}
	*out_len = (size_t)want;
	return 0;
}

/** Send one built message and read one reply. */
static int ldap_xchg(int fd, size_t tx_len, size_t *rx_len)
{
	if (zsock_send(fd, g_tx, tx_len, 0) < 0) {
		return -EHOSTUNREACH;
	}
	return ldap_recv(fd, rx_len);
}

/**
 * Bind as @p dn / @p pw.
 *
 * @retval 0               bound
 * @retval -EACCES         the directory said no
 * @retval -EHOSTUNREACH   could not ask
 */
static int ldap_bind(int fd, uint32_t msgid, const char *dn, const char *pw)
{
	ldap_result_t r;
	size_t tx_len = 0U;
	size_t rx_len = 0U;

	if (ldap_build_bind_simple(msgid, dn, pw, g_tx, sizeof(g_tx),
				   &tx_len) != 0) {
		return -EHOSTUNREACH;
	}
	if (ldap_xchg(fd, tx_len, &rx_len) != 0) {
		return -EHOSTUNREACH;
	}
	if (ldap_parse_result(g_rx, rx_len, LDAP_OP_BIND_RESPONSE, &r) != 0) {
		return -EHOSTUNREACH;
	}
	if (r.msgid != msgid) {
		return -EHOSTUNREACH;
	}
	return ldap_result_to_errno(r.code);
}

/**
 * Is @p user_dn a member of @p group? One base-scoped search, so the server's
 * work is bounded whatever the group's size.
 *
 * @retval 1  member
 * @retval 0  not a member
 * @retval <0 could not ask
 */
static int ldap_is_member(int fd, uint32_t msgid, const char *group,
			  const char *user_dn)
{
	ldap_search_req_t s;
	ldap_result_t done;
	uint32_t id = 0U;
	uint8_t op = 0U;
	size_t tx_len = 0U;
	size_t rx_len = 0U;
	bool found = false;

	if (group == NULL || group[0] == '\0') {
		return 0;
	}

	memset(&s, 0, sizeof(s));
	s.base = group;
	s.scope = LDAP_SCOPE_BASE;
	s.size_limit = 1U;
	s.time_limit = 5U;
	s.attr = g_ldap_cfg.member_attr;
	s.value = user_dn;
	s.want_attr = NULL;

	if (ldap_build_search(msgid, &s, g_tx, sizeof(g_tx), &tx_len) != 0) {
		return -EHOSTUNREACH;
	}
	if (zsock_send(fd, g_tx, tx_len, 0) < 0) {
		return -EHOSTUNREACH;
	}

	/* Entries then a SearchResultDone; bounded by sizeLimit above. */
	for (;;) {
		if (ldap_recv(fd, &rx_len) != 0) {
			return -EHOSTUNREACH;
		}
		if (ldap_msg_peek(g_rx, rx_len, &id, &op) != 0) {
			return -EHOSTUNREACH;
		}
		if (id != msgid) {
			return -EHOSTUNREACH;
		}
		if (op == LDAP_OP_SEARCH_ENTRY) {
			found = true;
			continue;
		}
		if (op == LDAP_OP_SEARCH_REFERENCE) {
			continue; /* referrals are not chased */
		}
		if (op == LDAP_OP_SEARCH_DONE) {
			if (ldap_parse_result(g_rx, rx_len,
					      LDAP_OP_SEARCH_DONE, &done) != 0) {
				return -EHOSTUNREACH;
			}
			break;
		}
		return -EHOSTUNREACH;
	}

	return found ? 1 : 0;
}

static int do_ldap(const char *user, const char *secret, uint8_t *out_role)
{
	ldap_role_map_t map;
	struct sockaddr_storage dst;
	socklen_t dst_len = 0U;
	char user_dn[DN_MAX + AUTH_USER_MAX + 2U];
	uint32_t msgid = 1U;
	int fd = -1;
	int rc;
	int m;

	if (g_ldap.host[0] == '\0' || g_ldap_cfg.user_dn[0] == '\0') {
		return -EHOSTUNREACH;
	}
	/* Refuse rather than escape: see ldap_dn_from_template(). */
	if (ldap_dn_from_template(g_ldap_cfg.user_dn, user, user_dn,
				  sizeof(user_dn)) < 0) {
		LOG_WRN("ldap: username not safe to place in a DN");
		return -EHOSTUNREACH;
	}

#if !defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	if (g_ldap_cfg.mode != 0U) {
		/* Binding in the clear when the operator asked for TLS would
		 * put the bind password on the wire. Refuse instead. */
		LOG_ERR("ldap: mode %u needs TLS support in this build",
			g_ldap_cfg.mode);
		return -EHOSTUNREACH;
	}
#endif

	rc = resolve(g_ldap.host, g_ldap.port, SOCK_STREAM, &dst, &dst_len);
	if (rc != 0) {
		return rc;
	}

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	if (g_ldap_cfg.mode == 2U) {
		fd = zsock_socket(dst.ss_family, SOCK_STREAM, IPPROTO_TLS_1_2);
	} else {
		fd = zsock_socket(dst.ss_family, SOCK_STREAM, IPPROTO_TCP);
	}
#else
	fd = zsock_socket(dst.ss_family, SOCK_STREAM, IPPROTO_TCP);
#endif
	if (fd < 0) {
		return -EHOSTUNREACH;
	}
	set_timeout(fd, g_ldap.timeout_ms);
	if (zsock_connect(fd, (struct sockaddr *)&dst, dst_len) < 0) {
		(void)zsock_close(fd);
		return -EHOSTUNREACH;
	}

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	if (g_ldap_cfg.mode == 1U) {
		ldap_result_t r;
		size_t tx_len = 0U;
		size_t rx_len = 0U;

		/*
		 * StartTLS: the ExtendedRequest goes over the plain connection
		 * and the socket is upgraded afterwards. Zephyr's TLS is a
		 * socket *type*, not an upgrade, so this cannot be completed
		 * without a socket-layer facility the platform does not offer.
		 * Refuse rather than continue in the clear.
		 */
		if (ldap_build_starttls(msgid, g_tx, sizeof(g_tx), &tx_len) ==
			    0 &&
		    ldap_xchg(fd, tx_len, &rx_len) == 0 &&
		    ldap_parse_result(g_rx, rx_len, LDAP_OP_EXTENDED_RESPONSE,
				      &r) == 0 &&
		    r.code == LDAP_RES_SUCCESS) {
			LOG_ERR("ldap: StartTLS accepted but this socket layer "
				"cannot upgrade in place; use LDAPS (mode 2)");
		} else {
			LOG_ERR("ldap: StartTLS refused by the server");
		}
		(void)zsock_close(fd);
		return -EHOSTUNREACH;
	}
#endif

	/* The user's own bind is the authentication. */
	rc = ldap_bind(fd, msgid++, user_dn, secret);
	if (rc != 0) {
		(void)zsock_close(fd);
		return rc;
	}

	/*
	 * Authorisation: three base-scoped membership checks, highest privilege
	 * first, so a user in several groups gets the strongest role and the
	 * server never has to enumerate a large group.
	 *
	 * The membership search runs as the *service* account when one is
	 * configured, because a directory commonly denies a plain user the right
	 * to read group membership.
	 */
	if (g_ldap_cfg.bind_dn[0] != '\0' && g_ldap_cfg.bind_pw_len != 0U) {
		char pw[AUTH_SHARED_MAX + 1U];
		size_t n = g_ldap_cfg.bind_pw_len;

		if (n > AUTH_SHARED_MAX) {
			n = AUTH_SHARED_MAX;
		}
		memcpy(pw, g_ldap_cfg.bind_pw, n);
		pw[n] = '\0';
		if (ldap_bind(fd, msgid++, g_ldap_cfg.bind_dn, pw) != 0) {
			LOG_WRN("ldap: service bind failed; "
				"group lookup will use the user's own rights");
		}
		memset(pw, 0, sizeof(pw));
	}

	memset(&map, 0, sizeof(map));
	map.admin = g_ldap_cfg.grp_admin;
	map.oper = g_ldap_cfg.grp_oper;
	map.viewer = g_ldap_cfg.grp_view;

	*out_role = (uint8_t)AUTH_ROLE_NONE;
	m = ldap_is_member(fd, msgid++, map.admin, user_dn);
	if (m == 1) {
		*out_role = (uint8_t)AUTH_ROLE_ADMIN;
	} else {
		m = ldap_is_member(fd, msgid++, map.oper, user_dn);
		if (m == 1) {
			*out_role = (uint8_t)AUTH_ROLE_OPERATOR;
		} else {
			m = ldap_is_member(fd, msgid++, map.viewer, user_dn);
			if (m == 1) {
				*out_role = (uint8_t)AUTH_ROLE_VIEWER;
			}
		}
	}

	{
		size_t tx_len = 0U;

		if (ldap_build_unbind(msgid, g_tx, sizeof(g_tx), &tx_len) == 0) {
			(void)zsock_send(fd, g_tx, tx_len, 0);
		}
	}
	(void)zsock_close(fd);

	if (*out_role == (uint8_t)AUTH_ROLE_NONE) {
		/*
		 * The bind succeeded, so the credential is good — but no
		 * configured group matched, which means the directory has not
		 * authorised this user for this box. That is a denial, not a
		 * fall-through: granting a default role to anybody who can bind
		 * would make every directory account an operator here.
		 */
		LOG_INF("ldap: bound but in no configured group");
		return -EACCES;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/* the one hook core/auth calls                                              */
/* ------------------------------------------------------------------------- */

int sts_aaa_remote(void *ctx, uint8_t backend, const char *user,
		   const char *secret, uint8_t *out_role)
{
	int rc;

	ARG_UNUSED(ctx);

	if (user == NULL || secret == NULL || out_role == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&g_lock, K_FOREVER);
	switch (backend) {
	case AUTH_BE_RADIUS:
		rc = do_radius(user, secret, out_role);
		break;
	case AUTH_BE_TACACS:
		rc = do_tacacs(user, secret, out_role);
		break;
	case AUTH_BE_LDAP:
		rc = do_ldap(user, secret, out_role);
		break;
	default:
		rc = -ENOTSUP;
		break;
	}
	/* Never leave a shared buffer holding a credential. */
	memset(g_tx, 0, sizeof(g_tx));
	memset(g_rx, 0, sizeof(g_rx));
	k_mutex_unlock(&g_lock);

	if (rc == 0) {
		LOG_INF("%s: accepted '%s' as %s", auth_backend_name(backend),
			user, auth_role_name(*out_role));
	} else if (rc == -EACCES) {
		LOG_WRN("%s: rejected '%s'", auth_backend_name(backend), user);
	}
	return rc;
}

/* ------------------------------------------------------------------------- */
/* public                                                                    */
/* ------------------------------------------------------------------------- */

int sts_aaa_check(const char *user, const char *secret, uint8_t *out_role)
{
	auth_out_t out;
	int rc;

	if (!g_started) {
		return -EHOSTUNREACH;
	}
	rc = auth_check(&g_auth, user, secret, sts_mono_ms(), &out);
	if (out_role != NULL) {
		*out_role = out.role;
	}
	if (rc == 0) {
		sts_log((uint8_t)LOGR_SUB_SEC, (uint8_t)LOGR_NOTICE,
			"auth: '%s' -> %s via %s%s", user,
			auth_role_name(out.role),
			auth_backend_name(out.backend),
			out.cached ? " (cached)" : "");
	} else if (rc == -EACCES) {
		sts_log((uint8_t)LOGR_SUB_SEC, (uint8_t)LOGR_WARN,
			"auth: '%s' rejected by %s", user,
			auth_backend_name(out.backend));
	} else if (rc == -EBUSY) {
		sts_log((uint8_t)LOGR_SUB_SEC, (uint8_t)LOGR_WARN,
			"auth: '%s' locked out for %us", user,
			(unsigned int)out.retry_after_s);
	} else if (rc == -EHOSTUNREACH) {
		sts_log((uint8_t)LOGR_SUB_SEC, (uint8_t)LOGR_ERR,
			"auth: no backend could answer for '%s' — denied", user);
	}
	return rc;
}

int sts_aaa_unlock(const char *user)
{
	if (!g_started) {
		return -EHOSTUNREACH;
	}
	return auth_unlock(&g_auth, user);
}

void sts_aaa_flush(void)
{
	if (g_started) {
		auth_cache_flush(&g_auth, NULL);
	}
}

int sts_aaa_stats(auth_stats_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}
	if (!g_started) {
		memset(out, 0, sizeof(*out));
		return -EHOSTUNREACH;
	}
	return auth_stats_get(&g_auth, out);
}

int sts_aaa_start(void)
{
	auth_cfg_t acfg;
	int rc;

	k_mutex_init(&g_lock);

	memset(&acfg, 0, sizeof(acfg));
	acfg.crypto = sts_port_crypto();
	acfg.remote = sts_aaa_remote;
	acfg.local_role = (uint8_t)AUTH_ROLE_ADMIN;
	acfg.order[0] = (uint8_t)AUTH_BE_LOCAL;
	acfg.n_order = 1U;

	rc = auth_init(&g_auth, &acfg);
	if (rc != 0) {
		LOG_ERR("auth_init: %d", rc);
		return rc;
	}
	g_started = true;

	/* Now pull the real configuration in, which also picks up the local
	 * credential blob and the backend chain. */
	sts_aaa_reapply();

	LOG_INF("AAA ready");
	return 0;
}
