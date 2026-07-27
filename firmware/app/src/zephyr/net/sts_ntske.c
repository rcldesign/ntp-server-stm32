/*
 * STS1000 "Meridian" — NTS-KE server: TLS 1.3 + RFC 8915 §4 (spec §4.2).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * ---------------------------------------------------------------------------
 * Why this drives mbedTLS directly instead of a Zephyr TLS socket
 * ---------------------------------------------------------------------------
 *
 * NTS-KE's entire purpose is to run the RFC 8446 §7.5 TLS exporter with the
 * label "EXPORTER-network-time-security" (RFC 8915 §4.3) and hand the two
 * derived keys to the NTP datapath. Zephyr's TLS socket layer keeps its
 * `mbedtls_ssl_context` inside a private static `tls_contexts[]` array in
 * subsys/net/lib/sockets/sockets_tls.c and exposes no exporter, no key-export
 * callback and no handle to the context — so mbedtls_ssl_export_keying_material()
 * is simply not reachable through IPPROTO_TLS_1_3.
 *
 * Every production NTS-KE server (chrony, ntpsec) therefore drives its TLS
 * stack directly, and so does this one: a plain Zephyr TCP socket carries the
 * bytes, mbedTLS runs the TLS 1.3 handshake over it through mbedtls_ssl_set_bio(),
 * and the exporter is called on the context this file owns. That is a
 * *deviation from the task's "using Zephyr TLS sockets" wording*, forced by the
 * exporter requirement and documented here and in the report.
 *
 * ---------------------------------------------------------------------------
 * The exporter macro, and why the whole body is guarded
 * ---------------------------------------------------------------------------
 *
 * mbedtls_ssl_export_keying_material() exists only when mbedTLS was built with
 * MBEDTLS_SSL_KEYING_MATERIAL_EXPORT. Zephyr's mbedTLS config does not enable
 * it and offers no Kconfig for it; sts_mbedtls_user.h turns it on, but wiring
 * that header onto the mbedTLS include path is a one-line app/CMakeLists.txt
 * edit owned by the platform area (see that header and the W3b report). Until
 * it lands the macro is undefined, so this entire TLS implementation is behind
 * `#if defined(...)`: the file always compiles, and sts_ntske_start() cleanly
 * reports the listener unavailable rather than standing up a server that could
 * not derive a single key.
 *
 * ---------------------------------------------------------------------------
 * Cert / key persistence (spec §9.2)
 * ---------------------------------------------------------------------------
 *
 * A self-signed P-256 certificate and its key are generated once per boot, in
 * RAM. They are NOT persisted this phase. Two reasons, both structural:
 *
 *   1. Persistence belongs to the console area (src/zephyr/storage); the area
 *      boundary (ARCHITECTURE.md §2) forbids this area reaching into it and
 *      sts_app.h exposes no key-blob store.
 *   2. NTS clients treat the KE server's cert as a pinned / operator-provisioned
 *      trust anchor, not a web-PKI leaf, so a per-boot self-signed cert only
 *      means a client re-pins after a reboot — acceptable for bring-up and
 *      exactly what spec §9.2 defers.
 *
 * TODO(persistence): once sts_app.h grows a sealed-blob store (PSA ITS or an
 * ATECC608B-wrapped NVS record, spec §4.2/§9.1), generate the key on first ever
 * boot, seal it, and reload it in make_self_signed_cert().
 *
 * ---------------------------------------------------------------------------
 * Threading and DoS posture
 * ---------------------------------------------------------------------------
 *
 * One acceptor thread (priority 12, spec §1.2 web/TLS band), one connection at
 * a time. NTS-KE is rare and bursty — a client runs it once, then serves itself
 * cookies over NTP for the cookie lifetime — so serialising handshakes trades
 * negligible throughput for a bounded RAM footprint (one ~16 KB session, not
 * N). A per-handshake wall-clock cap keeps a stalled client off the slot.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>

#include "cfg/cfg.h"
#include "net/sts_net.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_ntske, CONFIG_STS1000_LOG_LEVEL);

/* Statistics are visible in every build so the status/SNMP layers can report
 * "NTS-KE not running" uniformly. */
static struct {
	uint32_t accepted;
	uint32_t handshakes_ok;
	uint32_t handshakes_failed;
	uint32_t negotiations_ok;
	uint32_t cookies_issued;
	bool running;
	bool cert_ready;
	bool exporter_available;
} st;
static struct k_spinlock st_lock;

void sts_ntske_stats(sts_ntske_stats_t *out)
{
	if (out == NULL) {
		return;
	}
	K_SPINLOCK(&st_lock) {
		out->accepted = st.accepted;
		out->handshakes_ok = st.handshakes_ok;
		out->handshakes_failed = st.handshakes_failed;
		out->negotiations_ok = st.negotiations_ok;
		out->cookies_issued = st.cookies_issued;
		out->running = st.running;
		out->cert_ready = st.cert_ready;
		out->exporter_available = st.exporter_available;
	}
}

#if defined(MBEDTLS_SSL_KEYING_MATERIAL_EXPORT)

#include <mbedtls/ctr_drbg.h>
#include <mbedtls/ecp.h>
#include <mbedtls/entropy.h>
#include <mbedtls/pk.h>
#include <mbedtls/ssl.h>
#include <mbedtls/x509_crt.h>

#include "storage/sts_store.h"

#define NTSKE_STACK_SIZE 8192
#define NTSKE_PRIORITY 12
#define NTSKE_BACKLOG 2
#define NTSKE_HANDSHAKE_TIMEOUT_MS 8000
#define NTSKE_REQ_MAX 2048U
#define NTSKE_RSP_MAX NTSKE_RSP_RECOMMENDED

/** ALPN protocol id required by RFC 8915 §4. */
static const char *const alpn_list[] = { "ntske/1", NULL };

static ntske_ctx_t ke;
static mbedtls_ssl_config tls_conf;
static mbedtls_x509_crt srv_cert;
static mbedtls_pk_context srv_key;
static mbedtls_entropy_context entropy;
static mbedtls_ctr_drbg_context drbg;

static int listen_fd = -1;
static uint16_t ke_port;

static uint8_t req_buf[NTSKE_REQ_MAX];
static uint8_t rsp_buf[NTSKE_RSP_MAX];

static K_THREAD_STACK_DEFINE(ke_stack, NTSKE_STACK_SIZE);
static struct k_thread ke_thread;
static int live_id = -1;

/* ------------------------------------------------------------------------- */
/* mbedTLS plumbing                                                          */
/* ------------------------------------------------------------------------- */

static int bio_send(void *ctx, const unsigned char *buf, size_t len)
{
	int fd = (int)(intptr_t)ctx;
	ssize_t n = zsock_send(fd, buf, len, 0);

	if (n < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			return MBEDTLS_ERR_SSL_WANT_WRITE;
		}
		return MBEDTLS_ERR_NET_SEND_FAILED;
	}
	return (int)n;
}

static int bio_recv(void *ctx, unsigned char *buf, size_t len)
{
	int fd = (int)(intptr_t)ctx;
	ssize_t n = zsock_recv(fd, buf, len, 0);

	if (n < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			return MBEDTLS_ERR_SSL_WANT_READ;
		}
		return MBEDTLS_ERR_NET_RECV_FAILED;
	}
	if (n == 0) {
		return MBEDTLS_ERR_SSL_CONN_EOF;
	}
	return (int)n;
}

/** RFC 8915 §4.3 TLS exporter, bound into ntske_cfg_t. */
static int exporter_cb(void *ctx, const char *label, const uint8_t *context,
		       size_t context_len, uint8_t *out, size_t out_len)
{
	mbedtls_ssl_context *ssl = ctx;
	int rc;

	rc = mbedtls_ssl_export_keying_material(ssl, out, out_len, label,
						strlen(label), context,
						context_len, 1);
	if (rc != 0) {
		LOG_WRN("exporter failed: -0x%04x", (unsigned int)-rc);
		return -EIO;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/* self-signed certificate                                                   */
/* ------------------------------------------------------------------------- */

static int make_self_signed_cert(void)
{
	mbedtls_x509write_cert w;
	mbedtls_mpi serial;
	uint8_t der[768];
	int len;
	int rc;

	mbedtls_pk_init(&srv_key);
	mbedtls_x509_crt_init(&srv_cert);
	mbedtls_x509write_crt_init(&w);
	mbedtls_mpi_init(&serial);

	rc = mbedtls_pk_setup(&srv_key,
			      mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY));
	if (rc != 0) {
		goto out;
	}
	rc = mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1,
				 mbedtls_pk_ec(srv_key),
				 mbedtls_ctr_drbg_random, &drbg);
	if (rc != 0) {
		goto out;
	}

	mbedtls_x509write_crt_set_version(&w, MBEDTLS_X509_CRT_VERSION_3);
	mbedtls_x509write_crt_set_md_alg(&w, MBEDTLS_MD_SHA256);
	mbedtls_x509write_crt_set_subject_key(&w, &srv_key);
	mbedtls_x509write_crt_set_issuer_key(&w, &srv_key);

	rc = mbedtls_x509write_crt_set_subject_name(&w, "CN=STS1000 Meridian");
	if (rc != 0) {
		goto out;
	}
	rc = mbedtls_x509write_crt_set_issuer_name(&w, "CN=STS1000 Meridian");
	if (rc != 0) {
		goto out;
	}
	rc = mbedtls_mpi_lset(&serial, 1);
	if (rc != 0) {
		goto out;
	}
	rc = mbedtls_x509write_crt_set_serial(&w, &serial);
	if (rc != 0) {
		goto out;
	}
	/* A device with no wall clock at first boot cannot honour a tight
	 * validity window; NTS clients pin the key rather than chain to a CA
	 * clock, so a wide window is correct rather than lax. */
	rc = mbedtls_x509write_crt_set_validity(&w, "20260101000000",
						"20360101000000");
	if (rc != 0) {
		goto out;
	}

	len = mbedtls_x509write_crt_der(&w, der, sizeof(der),
					mbedtls_ctr_drbg_random, &drbg);
	if (len < 0) {
		rc = len;
		goto out;
	}
	/* _der writes to the END of the buffer and returns the length. */
	rc = mbedtls_x509_crt_parse_der(&srv_cert, der + sizeof(der) - len,
					(size_t)len);

out:
	mbedtls_mpi_free(&serial);
	mbedtls_x509write_crt_free(&w);
	return rc;
}

static int tls_setup(void)
{
	int rc;

	mbedtls_ssl_config_init(&tls_conf);
	mbedtls_entropy_init(&entropy);
	mbedtls_ctr_drbg_init(&drbg);

	rc = mbedtls_ctr_drbg_seed(&drbg, mbedtls_entropy_func, &entropy,
				   (const uint8_t *)"sts1000-ntske", 13);
	if (rc != 0) {
		LOG_ERR("ctr_drbg_seed: -0x%04x", (unsigned int)-rc);
		return -EIO;
	}

	rc = make_self_signed_cert();
	if (rc != 0) {
		LOG_ERR("cert generation: -0x%04x", (unsigned int)-rc);
		return -EIO;
	}

	rc = mbedtls_ssl_config_defaults(&tls_conf, MBEDTLS_SSL_IS_SERVER,
					 MBEDTLS_SSL_TRANSPORT_STREAM,
					 MBEDTLS_SSL_PRESET_DEFAULT);
	if (rc != 0) {
		return -EIO;
	}

	mbedtls_ssl_conf_min_tls_version(&tls_conf, MBEDTLS_SSL_VERSION_TLS1_3);
	mbedtls_ssl_conf_max_tls_version(&tls_conf, MBEDTLS_SSL_VERSION_TLS1_3);
	mbedtls_ssl_conf_authmode(&tls_conf, MBEDTLS_SSL_VERIFY_NONE);
	mbedtls_ssl_conf_rng(&tls_conf, mbedtls_ctr_drbg_random, &drbg);

	rc = mbedtls_ssl_conf_own_cert(&tls_conf, &srv_cert, &srv_key);
	if (rc != 0) {
		LOG_ERR("conf_own_cert: -0x%04x", (unsigned int)-rc);
		return -EIO;
	}
	rc = mbedtls_ssl_conf_alpn_protocols(&tls_conf, alpn_list);
	if (rc != 0) {
		return -EIO;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/* one connection                                                            */
/* ------------------------------------------------------------------------- */

static void handle_conn(int fd)
{
	mbedtls_ssl_context ssl;
	ntske_result_t res;
	const char *alpn;
	size_t total = 0U;
	size_t rsp_len = 0U;
	int rc;
	int64_t deadline = (int64_t)sts_mono_ms() + NTSKE_HANDSHAKE_TIMEOUT_MS;

	mbedtls_ssl_init(&ssl);

	rc = mbedtls_ssl_setup(&ssl, &tls_conf);
	if (rc != 0) {
		goto done;
	}
	mbedtls_ssl_set_bio(&ssl, (void *)(intptr_t)fd, bio_send, bio_recv,
			    NULL);

	ke.cfg.export_ctx = &ssl; /* the exporter needs this exact context */

	do {
		rc = mbedtls_ssl_handshake(&ssl);
		if ((int64_t)sts_mono_ms() > deadline) {
			rc = MBEDTLS_ERR_SSL_TIMEOUT;
			break;
		}
	} while (rc == MBEDTLS_ERR_SSL_WANT_READ ||
		 rc == MBEDTLS_ERR_SSL_WANT_WRITE);

	if (rc != 0) {
		K_SPINLOCK(&st_lock) {
			st.handshakes_failed++;
		}
		goto close_notify;
	}

	alpn = mbedtls_ssl_get_alpn_protocol(&ssl);
	if (alpn == NULL || strcmp(alpn, "ntske/1") != 0) {
		/* RFC 8915 §4: the client MUST offer "ntske/1". */
		K_SPINLOCK(&st_lock) {
			st.handshakes_failed++;
		}
		goto close_notify;
	}

	K_SPINLOCK(&st_lock) {
		st.handshakes_ok++;
	}

	for (;;) {
		rc = mbedtls_ssl_read(&ssl, &req_buf[total],
				      sizeof(req_buf) - total);
		if (rc == MBEDTLS_ERR_SSL_WANT_READ ||
		    rc == MBEDTLS_ERR_SSL_WANT_WRITE) {
			if ((int64_t)sts_mono_ms() > deadline) {
				goto close_notify;
			}
			continue;
		}
		if (rc <= 0) {
			break;
		}
		total += (size_t)rc;
		if (total >= sizeof(req_buf) || total >= 4U) {
			/* A well-formed request ends in an End-of-Message
			 * record; stop once one flight is in rather than
			 * waiting for a close the client sends only after our
			 * response. */
			break;
		}
	}

	rc = ntske_handle(&ke, req_buf, total, rsp_buf, sizeof(rsp_buf),
			  &rsp_len, &res);
	if (rc != 0) {
		goto close_notify;
	}
	if (res.ok) {
		K_SPINLOCK(&st_lock) {
			st.negotiations_ok++;
			st.cookies_issued += res.cookies;
		}
	}

	{
		size_t off = 0U;

		while (off < rsp_len) {
			rc = mbedtls_ssl_write(&ssl, &rsp_buf[off],
					       rsp_len - off);
			if (rc == MBEDTLS_ERR_SSL_WANT_READ ||
			    rc == MBEDTLS_ERR_SSL_WANT_WRITE) {
				if ((int64_t)sts_mono_ms() > deadline) {
					break;
				}
				continue;
			}
			if (rc <= 0) {
				break;
			}
			off += (size_t)rc;
		}
	}

close_notify:
	(void)mbedtls_ssl_close_notify(&ssl);
done:
	ke.cfg.export_ctx = NULL;
	mbedtls_ssl_free(&ssl);
	(void)zsock_close(fd);
}

/* ------------------------------------------------------------------------- */
/* acceptor                                                                  */
/* ------------------------------------------------------------------------- */

static int open_listener(void)
{
	struct sockaddr_in6 a6;
	int fd;
	int on = 1;

	fd = zsock_socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);
	if (fd < 0) {
		return -errno;
	}
	(void)zsock_setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

	memset(&a6, 0, sizeof(a6));
	a6.sin6_family = AF_INET6;
	a6.sin6_port = htons(ke_port);
	a6.sin6_addr = in6addr_any;

	if (zsock_bind(fd, (struct sockaddr *)&a6, sizeof(a6)) < 0) {
		int rc = -errno;

		(void)zsock_close(fd);
		return rc;
	}
	if (zsock_listen(fd, NTSKE_BACKLOG) < 0) {
		int rc = -errno;

		(void)zsock_close(fd);
		return rc;
	}
	return fd;
}

static void ke_loop(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	K_SPINLOCK(&st_lock) {
		st.running = true;
	}

	for (;;) {
		struct zsock_pollfd pfd = {
			.fd = listen_fd,
			.events = ZSOCK_POLLIN,
			.revents = 0,
		};
		int rc = zsock_poll(&pfd, 1, 500);
		int cfd;

		if (live_id >= 0) {
			sts_liveness_feed(live_id);
		}
		if (rc <= 0 || (pfd.revents & ZSOCK_POLLIN) == 0) {
			continue;
		}

		cfd = zsock_accept(listen_fd, NULL, NULL);
		if (cfd < 0) {
			continue;
		}
		K_SPINLOCK(&st_lock) {
			st.accepted++;
		}
		handle_conn(cfd);
	}
}

int sts_ntske_start(void)
{
	ntske_cfg_t cfg;
	int rc;

	if (!sts_net_cfg_bool(CFG_ID_NTS_ENABLE, true)) {
		LOG_INF("NTS disabled by configuration");
		return 0;
	}
	if (sts_net_keyring() == NULL) {
		LOG_WRN("NTS-KE: no master-key ring; NTS not started");
		return 0;
	}

	ke_port = (uint16_t)sts_net_cfg_u64(CFG_ID_NTS_KE_PORT, 4460U);

	rc = tls_setup();
	if (rc != 0) {
		return rc;
	}
	K_SPINLOCK(&st_lock) {
		st.cert_ready = true;
		st.exporter_available = true;
	}

	memset(&cfg, 0, sizeof(cfg));
	cfg.ring = sts_net_keyring();
	cfg.export_fn = exporter_cb;
	cfg.export_ctx = NULL; /* set per connection to the live ssl context */
	cfg.server_name = NULL;
	cfg.port = 0U;
	cfg.cookies = NTSKE_COOKIES_DEFAULT;

	rc = ntske_init(&ke, &cfg);
	if (rc != 0) {
		LOG_ERR("ntske_init: %d", rc);
		return rc;
	}

	listen_fd = open_listener();
	if (listen_fd < 0) {
		LOG_ERR("NTS-KE listen on :%u failed: %d", ke_port, listen_fd);
		return listen_fd;
	}

	live_id = sts_liveness_register("ntske");
	k_thread_create(&ke_thread, ke_stack, K_THREAD_STACK_SIZEOF(ke_stack),
			ke_loop, NULL, NULL, NULL, NTSKE_PRIORITY, 0,
			K_NO_WAIT);
	k_thread_name_set(&ke_thread, "ntske");

	LOG_INF("NTS-KE (TLS 1.3) on :%u", ke_port);
	return 0;
}

#else /* !MBEDTLS_SSL_KEYING_MATERIAL_EXPORT */

int sts_ntske_start(void)
{
	/*
	 * Without the RFC 8446 exporter NTS-KE cannot derive C2S/S2C keys, so
	 * standing up a listener that answers every handshake with an Internal
	 * Server Error would be worse than not listening. Enable it by wiring
	 * sts_mbedtls_user.h onto the mbedTLS include path — see that header.
	 */
	LOG_WRN("NTS-KE disabled: mbedTLS built without the TLS exporter "
		"(MBEDTLS_SSL_KEYING_MATERIAL_EXPORT); see sts_mbedtls_user.h");
	return 0;
}

#endif /* MBEDTLS_SSL_KEYING_MATERIAL_EXPORT */
