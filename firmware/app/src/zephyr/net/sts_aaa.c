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
 * `sec.ldap.mode` selects plain (389), StartTLS or LDAPS (636).
 *
 * CONFIG_NET_SOCKETS_SOCKOPT_TLS and CONFIG_TLS_CREDENTIALS ARE SET in every
 * image this project builds: app/conf/web.conf turns them on for HTTPS, Kconfig
 * symbols are global, and app/CMakeLists.txt globs every conf fragment into
 * every build. So the TLS blocks below are compiled in today — the `#if` is a
 * guard for a hypothetical HTTPS-less build, not a description of the shipped
 * one. (An older comment here claimed the opposite, and pointed at the
 * commented-out block at the end of conf/net.conf; that block is NTS-KE's
 * mbedTLS *user-config header*, which is a separate and still-open item.)
 *
 * LDAPS therefore needs a trust anchor, not a Kconfig change. The whole
 * decision — which transport, and what is missing when there is none — is in
 * net/sts_ldap_ca.h, together with the reasoning for each branch. In outline:
 *
 *   mode 0   plain TCP;
 *   mode 1   refused. Zephyr's TLS is a socket *type* chosen at socket() time,
 *            not an in-place upgrade, so the StartTLS ExtendedRequest has
 *            nowhere to go. Refused BEFORE the socket, because probing a
 *            directory to learn which flavour of impossible applies is an
 *            outbound connection an unauthenticated login attempt should not
 *            get to make;
 *   mode 2   a TLS socket armed with TLS_SEC_TAG_LIST (the operator's anchor)
 *            and TLS_HOSTNAME (`sec.ldap.host`), refused before the socket when
 *            no anchor is installed.
 *
 * None of the three ever falls back to plaintext: binding in the clear would
 * send the bind password over the wire, which is exactly the failure the
 * operator selected TLS to avoid.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
#include <zephyr/fs/fs.h>
#include <zephyr/net/tls_credentials.h>

#include <mbedtls/x509_crt.h>
#endif

#include "auth/auth.h"
#include "auth/ldap.h"
#include "auth/radius.h"
#include "auth/tacacs.h"
#include "cfg/cfg.h"
#include "net/sts_aaa.h"
#include "net/sts_ldap_ca.h"
#include "net/sts_net.h"
/* sts_store.h is the one header this area takes from another (its own comment
 * declares it a public façade): it is where the net area gets its port_crypto_t
 * rather than standing up a second mbedTLS binding. */
#include "storage/sts_store.h"
#include "web/web.h" /* web_hex_encode / web_span_copy, as sts_cert.c uses them */
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

void sts_aaa_reapply(void)
{
	auth_cfg_t acfg;
	char order[32];
	int n;

	if (!g_started) {
		return;
	}

	k_mutex_lock(&g_lock, K_FOREVER);

	(void)sts_net_cfg_str((uint16_t)CFG_ID_SEC_AAA_NAS_ID, g_nas_id,
			      sizeof(g_nas_id));
	if (g_nas_id[0] == '\0') {
		(void)sts_net_cfg_str((uint16_t)CFG_ID_NET_HOSTNAME, g_nas_id,
				      sizeof(g_nas_id));
	}

	(void)sts_net_cfg_str((uint16_t)CFG_ID_SEC_RADIUS_HOST, g_radius.host,
			      sizeof(g_radius.host));
	g_radius.port =
		(uint16_t)sts_net_cfg_u64((uint16_t)CFG_ID_SEC_RADIUS_PORT,
					  1812U);
	load_blob((uint16_t)CFG_ID_SEC_RADIUS_SECRET, g_radius.secret,
		  sizeof(g_radius.secret), &g_radius.secret_len);
	g_radius.timeout_ms =
		(uint16_t)sts_net_cfg_u64((uint16_t)CFG_ID_SEC_RADIUS_TMO_MS,
					  3000U);
	g_radius.retries =
		(uint8_t)sts_net_cfg_u64((uint16_t)CFG_ID_SEC_RADIUS_RETRIES,
					 2U);

	(void)sts_net_cfg_str((uint16_t)CFG_ID_SEC_TACACS_HOST, g_tacacs.host,
			      sizeof(g_tacacs.host));
	g_tacacs.port =
		(uint16_t)sts_net_cfg_u64((uint16_t)CFG_ID_SEC_TACACS_PORT, 49U);
	load_blob((uint16_t)CFG_ID_SEC_TACACS_SECRET, g_tacacs.secret,
		  sizeof(g_tacacs.secret), &g_tacacs.secret_len);
	g_tacacs.timeout_ms =
		(uint16_t)sts_net_cfg_u64((uint16_t)CFG_ID_SEC_TACACS_TMO_MS,
					  5000U);

	(void)sts_net_cfg_str((uint16_t)CFG_ID_SEC_LDAP_HOST, g_ldap.host,
			      sizeof(g_ldap.host));
	g_ldap.port =
		(uint16_t)sts_net_cfg_u64((uint16_t)CFG_ID_SEC_LDAP_PORT, 389U);
	g_ldap.timeout_ms =
		(uint16_t)sts_net_cfg_u64((uint16_t)CFG_ID_SEC_LDAP_TMO_MS,
					  5000U);
	g_ldap_cfg.mode =
		(uint8_t)sts_net_cfg_u64((uint16_t)CFG_ID_SEC_LDAP_MODE, 0U);
	(void)sts_net_cfg_str((uint16_t)CFG_ID_SEC_LDAP_BASE_DN, g_ldap_cfg.base,
			      sizeof(g_ldap_cfg.base));
	(void)sts_net_cfg_str((uint16_t)CFG_ID_SEC_LDAP_USER_DN, g_ldap_cfg.user_dn,
			      sizeof(g_ldap_cfg.user_dn));
	(void)sts_net_cfg_str((uint16_t)CFG_ID_SEC_LDAP_BIND_DN, g_ldap_cfg.bind_dn,
			      sizeof(g_ldap_cfg.bind_dn));
	load_blob((uint16_t)CFG_ID_SEC_LDAP_BIND_PW, g_ldap_cfg.bind_pw,
		  sizeof(g_ldap_cfg.bind_pw), &g_ldap_cfg.bind_pw_len);
	(void)sts_net_cfg_str((uint16_t)CFG_ID_SEC_LDAP_MEMBER_ATTR, g_ldap_cfg.member_attr,
			      sizeof(g_ldap_cfg.member_attr));
	if (g_ldap_cfg.member_attr[0] == '\0') {
		(void)strncpy(g_ldap_cfg.member_attr, "member", ATTR_MAX - 1U);
	}
	(void)sts_net_cfg_str((uint16_t)CFG_ID_SEC_LDAP_GRP_ADMIN, g_ldap_cfg.grp_admin,
			      sizeof(g_ldap_cfg.grp_admin));
	(void)sts_net_cfg_str((uint16_t)CFG_ID_SEC_LDAP_GRP_OPER, g_ldap_cfg.grp_oper,
			      sizeof(g_ldap_cfg.grp_oper));
	(void)sts_net_cfg_str((uint16_t)CFG_ID_SEC_LDAP_GRP_VIEW, g_ldap_cfg.grp_view,
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
		(uint16_t)sts_net_cfg_u64((uint16_t)CFG_ID_SEC_AAA_CACHE_S,
					  300U);
	acfg.lockout_fails =
		(uint8_t)sts_net_cfg_u64((uint16_t)CFG_ID_SEC_AAA_LOCK_N, 5U);
	acfg.lockout_s =
		(uint16_t)sts_net_cfg_u64((uint16_t)CFG_ID_SEC_AAA_LOCK_S, 300U);

	(void)sts_net_cfg_str((uint16_t)CFG_ID_SEC_AAA_ORDER, order, sizeof(order));
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
/* LDAP trust anchor                                                         */
/* ------------------------------------------------------------------------- */
/*
 * The operator-installed CA that makes `sec.ldap.mode = 2` able to complete a
 * handshake at all. sts_ldap_ca.h carries the reasoning; this is the storage
 * and the Zephyr binding.
 *
 * Modelled on sts_cert.c: a PEM file under /lfs/tls, loaded once into a
 * file-scope buffer and handed to tls_credential_add(). The buffer must be
 * file-scope and must outlive every socket that uses the tag, because
 * tls_credential_add() stores the POINTER — subsys/net/lib/tls_credentials/
 * tls_credentials.c copies nothing. That is also why ldap_ca_forget() deletes
 * the credential entry BEFORE it zeroes the buffer.
 *
 * Everything below runs under g_lock, which is what keeps do_ldap() — the only
 * reader of the credential — out of the buffer while it changes.
 */

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)

/* +1 for the NUL: mbedTLS detects PEM by the trailing NUL inside the length it
 * is given, and Zephyr passes cred->len straight through to it. */
static char g_ca_pem[STS_LDAP_CA_PEM_MAX + 1U];
static size_t g_ca_len;
static bool g_ca_ready;
static bool g_ca_persisted;
/** /lfs has already been consulted once; a missing file is not re-read. */
static bool g_ca_scanned;
static char g_ca_subject[72];
static char g_ca_not_after[24];
static char g_ca_fp[68];

static int ca_file_read(char *buf, size_t cap, size_t *out_len)
{
	struct fs_file_t f;
	ssize_t n;
	int rc;

	if (!sts_fs_ready()) {
		return -EROFS;
	}
	fs_file_t_init(&f);
	rc = fs_open(&f, STS_LDAP_CA_PATH, FS_O_READ);
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
	rc = fs_mkdir(STS_LDAP_CA_DIR);
	if (rc != 0 && rc != -EEXIST) {
		LOG_WRN("mkdir %s: %d", STS_LDAP_CA_DIR, rc);
	}
	fs_file_t_init(&f);
	rc = fs_open(&f, STS_LDAP_CA_PATH, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
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
static void ldap_ca_forget(void)
{
	(void)tls_credential_delete(STS_LDAP_CA_SEC_TAG,
				    TLS_CREDENTIAL_CA_CERTIFICATE);
	memset(g_ca_pem, 0, sizeof(g_ca_pem));
	g_ca_len = 0U;
	g_ca_ready = false;
	g_ca_persisted = false;
	g_ca_subject[0] = '\0';
	g_ca_not_after[0] = '\0';
	g_ca_fp[0] = '\0';
}

/** Subject DN, expiry and DER fingerprint of the first certificate. */
static void ldap_ca_describe(const mbedtls_x509_crt *crt)
{
	const port_crypto_t *cr = sts_port_crypto();
	uint8_t digest[32];

	if (mbedtls_x509_dn_gets(g_ca_subject, sizeof(g_ca_subject),
				 &crt->subject) < 0) {
		g_ca_subject[0] = '\0';
	}
	(void)snprintf(g_ca_not_after, sizeof(g_ca_not_after),
		       "%04d-%02d-%02dT%02d:%02d:%02dZ", crt->valid_to.year,
		       crt->valid_to.mon, crt->valid_to.day, crt->valid_to.hour,
		       crt->valid_to.min, crt->valid_to.sec);

	g_ca_fp[0] = '\0';
	if (cr != NULL && cr->sha256 != NULL &&
	    cr->sha256(cr->ctx, crt->raw.p, crt->raw.len, digest) == 0) {
		(void)web_hex_encode(digest, sizeof(digest), g_ca_fp,
				     sizeof(g_ca_fp));
	}
}

/**
 * Validate @p pem and, if it is good, make it the live anchor. Touches no file.
 *
 * The screening in sts_ldap_ca_check() runs on the CALLER's buffer, so an
 * operator's bad paste is refused before the live anchor is disturbed. Only an
 * X.509 defect that survives the framing check can cost the previous anchor,
 * and the header explains why that outcome is preferred to a stale one.
 */
static int ldap_ca_adopt(const char *pem, size_t len)
{
	mbedtls_x509_crt crt;
	sts_ldap_ca_verdict_t v;
	int rc;

	v = sts_ldap_ca_check(pem, len);
	if (v != STS_LDAP_CA_OK) {
		LOG_ERR("ldap ca: refused — %s", sts_ldap_ca_reason(v));
		switch (v) {
		case STS_LDAP_CA_TOO_BIG:
			return -EFBIG;
		case STS_LDAP_CA_HAS_KEY:
			return -EPERM;
		default:
			return -EBADMSG;
		}
	}

	ldap_ca_forget();
	memcpy(g_ca_pem, pem, len);
	g_ca_pem[len] = '\0';
	g_ca_len = len;

	mbedtls_x509_crt_init(&crt);
	/* +1: mbedTLS decides PEM-vs-DER by the NUL at the end of the span. A
	 * positive return is a PARTIAL parse (that many blocks failed), which is
	 * not good enough for a trust anchor. */
	rc = mbedtls_x509_crt_parse(&crt, (const unsigned char *)g_ca_pem,
				    g_ca_len + 1U);
	if (rc != 0) {
		if (rc > 0) {
			LOG_ERR("ldap ca: %d certificate block(s) did not parse",
				rc);
		} else {
			LOG_ERR("ldap ca: x509 parse -0x%04x", (unsigned int)-rc);
		}
		mbedtls_x509_crt_free(&crt);
		ldap_ca_forget();
		return -EBADMSG;
	}
	ldap_ca_describe(&crt);
	mbedtls_x509_crt_free(&crt);

	rc = tls_credential_add(STS_LDAP_CA_SEC_TAG, TLS_CREDENTIAL_CA_CERTIFICATE,
				g_ca_pem, g_ca_len + 1U);
	if (rc != 0) {
		LOG_ERR("ldap ca: tls_credential_add: %d", rc);
		ldap_ca_forget();
		return rc;
	}
	g_ca_ready = true;
	return 0;
}

/** Read the persisted anchor, once. Sets g_ca_scanned whatever the outcome. */
static void ldap_ca_load(void)
{
	char buf[STS_LDAP_CA_PEM_MAX];
	size_t n = 0U;
	int rc;

	if (!sts_fs_ready()) {
		return; /* not scanned: /lfs may still mount */
	}
	g_ca_scanned = true;

	rc = ca_file_read(buf, sizeof(buf), &n);
	if (rc != 0) {
		if (rc != -ENOENT) {
			LOG_WRN("ldap ca: %s unreadable (%d)", STS_LDAP_CA_PATH,
				rc);
		}
		return;
	}
	if (ldap_ca_adopt(buf, n) != 0) {
		LOG_ERR("ldap ca: %s is unusable; LDAPS will refuse until a "
			"good anchor is installed",
			STS_LDAP_CA_PATH);
		return;
	}
	g_ca_persisted = true;
	LOG_INF("ldap ca: %s loaded (%s, expires %s)", STS_LDAP_CA_PATH,
		g_ca_subject, g_ca_not_after);
}

/**
 * Is an anchor live? Loads it lazily the first time /lfs is available, so this
 * does not depend on sts_aaa_start() running after the volume is mounted.
 *
 * Caller must hold g_lock.
 */
static bool ldap_ca_ready(void)
{
	if (!g_ca_scanned) {
		ldap_ca_load();
	}
	return g_ca_ready;
}

/**
 * How long a read-only CA query will wait for the exchange mutex.
 *
 * NOT K_FOREVER, and the difference matters. g_lock is held for the WHOLE of
 * do_ldap() — resolve, connect, TLS handshake, bind, three membership searches —
 * bounded only by `sec.ldap.tmo.ms` per exchange. sts_aaa_ldap_ca_info()'s only
 * caller is a REST provider (sts_web.c:990) running on one of two web workers
 * at priority 12, so a K_FOREVER here lets a single slow or hostile directory
 * server park a worker for the length of a bind, and two status polls wedge the
 * management plane outright.
 *
 * That is precisely the availability failure the AAA serialisation gate was
 * built to prevent, and it would have walked straight back in through a status
 * query — which needs none of the exchange state it was queueing behind.
 *
 * 200 ms is far longer than any legitimate holder of this mutex outside an
 * in-flight exchange (the longest is ldap_ca_adopt(), a parse and a memcpy over
 * at most 3 KiB), so a timeout means "an authentication is running", which is
 * exactly what -EBUSY tells the operator.
 */
#define LDAP_CA_INFO_WAIT_MS 200

/**
 * How long the ERASE will wait for the exchange mutex before giving up on the
 * in-RAM half and letting the caller's reboot finish the job.
 *
 * Same mutex, same unbounded holder, opposite response to a timeout — and the
 * asymmetry is the point. sts_aaa_ldap_ca_info() answers a status poll it can
 * honestly decline (-EBUSY, retry). sts_aaa_ldap_ca_erase() is the retraction
 * half of a factory reset, running INLINE on a liveness participant with
 * CONFIG_STS1000_LIVENESS_DEADLINE_MS to spend, so it can neither wait for the
 * exchange nor refuse the work: it does the persisted half without this mutex
 * at all, and treats the wait below as the only thing the RAM half is allowed
 * to cost. sts_ldap_ca.h carries the reasoning in full.
 *
 * 250 ms clears every legitimate holder of this mutex outside an in-flight
 * exchange — the longest is sts_aaa_ldap_ca_install(), a parse and a memcpy
 * over at most 3 KiB followed by a LittleFS write of the same — and is a 20x
 * margin against the 5 000 ms deadline this call is one step of. The rest of
 * that budget is not this function's to spend: sts_cfg_factory_reset() erases
 * NVS and fans out every applier, and sts_cert_reset() unlinks three files,
 * before and after it on the same thread.
 */
#define LDAP_CA_ERASE_WAIT_MS 250

BUILD_ASSERT(LDAP_CA_ERASE_WAIT_MS * 4U < CONFIG_STS1000_LIVENESS_DEADLINE_MS,
	     "the factory-reset erase must leave the watchdog wide margin: it "
	     "runs inline on a liveness participant");

int sts_aaa_ldap_ca_info(sts_aaa_ldap_ca_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));

	if (k_mutex_lock(&g_lock, K_MSEC(LDAP_CA_INFO_WAIT_MS)) != 0) {
		return -EBUSY;
	}
	out->present = ldap_ca_ready();
	out->persisted = g_ca_persisted;
	(void)web_span_copy(out->subject, sizeof(out->subject), g_ca_subject,
			    strlen(g_ca_subject));
	(void)web_span_copy(out->not_after, sizeof(out->not_after),
			    g_ca_not_after, strlen(g_ca_not_after));
	(void)web_span_copy(out->sha256_fp, sizeof(out->sha256_fp), g_ca_fp,
			    strlen(g_ca_fp));
	k_mutex_unlock(&g_lock);
	return 0;
}

int sts_aaa_ldap_ca_install(const char *pem, size_t len)
{
	char subject[sizeof(g_ca_subject)];
	char fp[sizeof(g_ca_fp)];
	int rc;

	k_mutex_lock(&g_lock, K_FOREVER);
	rc = ldap_ca_adopt(pem, len);
	if (rc != 0) {
		/* Nothing is live now, so nothing on /lfs should claim to be:
		 * leaving the old file behind would resurrect the rejected
		 * operator's previous anchor at the next boot. */
		if (sts_fs_ready()) {
			int frc = fs_unlink(STS_LDAP_CA_PATH);

			if (!sts_ldap_ca_unlink_ok(frc)) {
				LOG_WRN("unlink %s: %d", STS_LDAP_CA_PATH, frc);
			}
		}
		g_ca_scanned = true;
		k_mutex_unlock(&g_lock);
		return rc;
	}

	g_ca_scanned = true;
	g_ca_persisted = (ca_file_write(g_ca_pem, g_ca_len) == 0);
	if (!g_ca_persisted) {
		LOG_WRN("ldap ca: accepted and live, but NOT persisted; it is "
			"gone at the next reboot");
		rc = -EROFS;
	}
	(void)web_span_copy(subject, sizeof(subject), g_ca_subject,
			    strlen(g_ca_subject));
	(void)web_span_copy(fp, sizeof(fp), g_ca_fp, strlen(g_ca_fp));
	k_mutex_unlock(&g_lock);

	sts_log((uint8_t)LOGR_SUB_SEC, (uint8_t)LOGR_NOTICE,
		"ldap ca installed: %s (sha256 %s)", subject, fp);
	return rc;
}

/**
 * Remove the persisted anchor, and fold the outcome into @p e.
 *
 * Takes no lock and must not grow one: the whole point of splitting the erase
 * is that this half cannot be made to wait behind an authentication exchange.
 * Nothing else on the board reads or writes STS_LDAP_CA_PATH under g_lock —
 * ca_file_read()/ca_file_write() are called with it held, but the file is not
 * shared STATE the mutex protects, and the volatile objects that are (g_ca_pem,
 * the credential entry) are dealt with separately by the caller.
 *
 * Idempotent and monotone, so the caller can run it twice.
 */
static void ca_persist_erase(sts_ldap_ca_erase_t *e)
{
	int frc;

	if (!sts_fs_ready()) {
		/* Nothing persisted this call can reach; see sts_ldap_ca.h. */
		return;
	}
	e->fs_ready = true;

	frc = fs_unlink(STS_LDAP_CA_PATH);
	if (!sts_ldap_ca_unlink_ok(frc)) {
		LOG_ERR("unlink %s: %d", STS_LDAP_CA_PATH, frc);
		e->unlinked = false;
	}
}

int sts_aaa_ldap_ca_erase(void)
{
	sts_ldap_ca_erase_t e = {
		.fs_ready = false,
		.unlinked = true, /* "nothing to unlink" is the same state */
		.ram_erased = false,
	};

	/*
	 * (1) The persisted half, FIRST and WITHOUT g_lock.
	 *
	 * This is the half that has to survive, because it is the only one the
	 * reboot at the end of a factory reset cannot do by itself — and it is
	 * the half that must never queue behind a backend exchange, which holds
	 * g_lock for minutes. sts_ldap_ca.h has the full account of what taking
	 * the mutex here used to cost.
	 */
	ca_persist_erase(&e);

	/*
	 * (2) The volatile half, under a BOUNDED wait. do_ldap() is the
	 * credential's only reader and ldap_ca_forget() deletes the credential
	 * entry before zeroing the buffer it points at, so this genuinely needs
	 * the mutex — but it does not need it badly enough to wait for a bind.
	 */
	if (k_mutex_lock(&g_lock, K_MSEC(LDAP_CA_ERASE_WAIT_MS)) == 0) {
		ldap_ca_forget();
		g_ca_scanned = true; /* erased on purpose: no lazy reload */

		/*
		 * Repeat the unlink now that nothing else can be inside this
		 * mutex. sts_aaa_ldap_ca_install() persists the anchor while
		 * holding g_lock, so an install that completed between step (1)
		 * and this lock would otherwise have re-created the file behind
		 * a factory reset that had already passed it.
		 */
		ca_persist_erase(&e);

		k_mutex_unlock(&g_lock);
		e.ram_erased = true;
	}

	if (sts_ldap_ca_erase_ram_deferred(&e)) {
		/*
		 * An exchange is in flight and owns the buffer. Not a failure:
		 * every caller of this function reboots unconditionally (see
		 * sts_ldap_ca.h), and the reboot is what clears RAM. Say so
		 * anyway — the claim is only true while that stays true.
		 */
		LOG_WRN("ldap ca: an AAA exchange holds the anchor buffer; the "
			"/lfs copy is erased and the in-RAM copy is left for "
			"the reboot to clear");
	}

	return sts_ldap_ca_erase_result(&e);
}

/**
 * Arm a mode-2 socket: the anchor, and the name the certificate must carry.
 *
 * TLS_PEER_VERIFY is deliberately absent. Zephyr leaves verify_level at -1
 * unless it is set, and mbedTLS's client default is then
 * MBEDTLS_SSL_VERIFY_REQUIRED — which is what this wants. Setting it explicitly
 * would only create the opportunity to set it wrong, and there is no "verify
 * none" escape hatch here on purpose: an unverified LDAPS bind hands the
 * directory password to whoever answered the TCP connection.
 *
 * TLS_HOSTNAME matters as much as the anchor. Without it sockets_tls.c calls
 * mbedtls_ssl_set_hostname(ssl, "") and the handshake checks only that SOME
 * certificate the CA ever issued was presented — including one for a different
 * host, which an attacker inside the CA's namespace can obtain legitimately.
 * The name is `sec.ldap.host` verbatim: if the operator configured a bare IP,
 * the server certificate needs a matching iPAddress SAN, and if it has none the
 * handshake fails closed rather than silently skipping the check.
 */
static int ldap_tls_arm(int fd)
{
	static const sec_tag_t tags[] = { STS_LDAP_CA_SEC_TAG };

	if (zsock_setsockopt(fd, SOL_TLS, TLS_SEC_TAG_LIST, tags,
			     sizeof(tags)) < 0) {
		LOG_ERR("ldap: TLS_SEC_TAG_LIST: %d", errno);
		return -EHOSTUNREACH;
	}
	if (zsock_setsockopt(fd, SOL_TLS, TLS_HOSTNAME, g_ldap.host,
			     strlen(g_ldap.host) + 1U) < 0) {
		LOG_ERR("ldap: TLS_HOSTNAME: %d", errno);
		return -EHOSTUNREACH;
	}
	return 0;
}

#else /* !CONFIG_NET_SOCKETS_SOCKOPT_TLS */

/*
 * No TLS socket layer, so no anchor can be used. The entry points still exist
 * because the REST provider table binds them unconditionally; each reports the
 * build limitation rather than pretending to have stored something.
 */
static bool ldap_ca_ready(void)
{
	return false;
}

int sts_aaa_ldap_ca_info(sts_aaa_ldap_ca_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));
	return -ENOTSUP;
}

int sts_aaa_ldap_ca_install(const char *pem, size_t len)
{
	ARG_UNUSED(pem);
	ARG_UNUSED(len);
	return -ENOTSUP;
}

int sts_aaa_ldap_ca_erase(void)
{
	return 0;
}

#endif /* CONFIG_NET_SOCKETS_SOCKOPT_TLS */

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
	sts_ldap_go_t go;
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

	/*
	 * The transport is decided BEFORE the resolver and the socket, so a
	 * configuration that cannot work costs nothing and — the point of the
	 * exercise — reports the missing thing rather than an mbedTLS error code
	 * from a handshake that never had a chance. Never falls back to
	 * plaintext: sts_ldap_go_opens_socket() is false for every refusal.
	 */
	go = sts_ldap_transport(g_ldap_cfg.mode,
				IS_ENABLED(CONFIG_NET_SOCKETS_SOCKOPT_TLS),
				ldap_ca_ready());
	if (!sts_ldap_go_opens_socket(go)) {
		LOG_ERR("ldap: mode %u refused: %s", g_ldap_cfg.mode,
			sts_ldap_go_reason(go));
		return -EHOSTUNREACH;
	}

	rc = resolve(g_ldap.host, g_ldap.port, SOCK_STREAM, &dst, &dst_len);
	if (rc != 0) {
		return rc;
	}

	/*
	 * IPPROTO_TLS_1_2 names the socket *type*, not a version floor: Zephyr's
	 * protocol_check() maps every IPPROTO_TLS_1_* to IPPROTO_TCP and keeps
	 * the value only so getsockopt(TLS_PROTOCOL_VERSION) can report it. The
	 * negotiated version comes from the mbedTLS Kconfig, which conf/web.conf
	 * pins to 1.3 only. sts_web.c opens its listener the same way.
	 */
	fd = zsock_socket(dst.ss_family, SOCK_STREAM,
			  sts_ldap_go_is_tls(go) ? IPPROTO_TLS_1_2 : IPPROTO_TCP);
	if (fd < 0) {
		return -EHOSTUNREACH;
	}
#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	if (sts_ldap_go_is_tls(go) && ldap_tls_arm(fd) != 0) {
		(void)zsock_close(fd);
		return -EHOSTUNREACH;
	}
#endif
	set_timeout(fd, g_ldap.timeout_ms);
	/* For a TLS socket this also runs the handshake, inline on this thread —
	 * which is what sizes STS_AAA_WORKER_STACK in sts_secops.c. */
	if (zsock_connect(fd, (struct sockaddr *)&dst, dst_len) < 0) {
		(void)zsock_close(fd);
		return -EHOSTUNREACH;
	}

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

	/*
	 * Fail closed before anything else. An unstarted context holds a zeroed
	 * auth_ctx_t (no crypto port, no chain), so this is the difference between
	 * "AAA is down, nobody can log in" and "AAA is down, so the caller reads
	 * an uninitialised role out of *out_role". The role is cleared on this
	 * path too, honouring the header's contract that *out_role is
	 * AUTH_ROLE_NONE unless the return value is 0 — a caller that trusted its
	 * own stack garbage would be the whole bug.
	 */
	if (out_role != NULL) {
		*out_role = (uint8_t)AUTH_ROLE_NONE;
	}
	if (!g_started) {
		return -EHOSTUNREACH;
	}
	memset(&out, 0, sizeof(out));
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

	/*
	 * The LDAPS trust anchor. Best-effort on purpose: a /lfs that is not
	 * mounted yet is not an error here, because ldap_ca_ready() retries the
	 * load the first time mode 2 is actually used. Doing it now anyway means
	 * the boot log names the anchor next to the mode it serves, instead of
	 * the operator discovering at the first login that there is none.
	 */
	k_mutex_lock(&g_lock, K_FOREVER);
	(void)ldap_ca_ready();
	k_mutex_unlock(&g_lock);

	LOG_INF("AAA ready");
	return 0;
}
