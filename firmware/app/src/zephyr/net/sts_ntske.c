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
 * N). Concurrency is therefore structurally bounded at one; NTSKE_BACKLOG
 * bounds what waits behind it and the kernel refuses the rest.
 *
 * ---------------------------------------------------------------------------
 * Why the per-connection deadline needs a non-blocking socket to exist at all
 * ---------------------------------------------------------------------------
 *
 * A Zephyr socket is blocking by default, so zsock_recv() inside bio_recv()
 * waits K_FOREVER. mbedtls_ssl_handshake() then never returns to us, the
 * deadline check after it is unreachable code, and one TCP connection that
 * completes the handshake's first flight and stops writing parks this thread
 * permanently.
 *
 * On this appliance that is not a stalled service, it is a reset. handle_conn()
 * runs inline from ke_loop(), so a parked connection also stops this area's
 * liveness feed; sts_liveness_stale_mask() goes stale at
 * CONFIG_STS1000_LIVENESS_DEADLINE_MS, the supervisor withholds the external
 * watchdog kick, the TPS3430 asserts WDO_N, and POE_KILL cold-cycles the board.
 * The attacker reconnects and the box boot-loops (F4). Worse, the original
 * handshake budget (8 s) was *longer* than the 5 s liveness deadline, so a
 * merely slow but legitimate client did it too.
 *
 * Three things fix it together, and all three are needed:
 *
 *   1. SO_RCVTIMEO/SO_SNDTIMEO on the accepted socket, so a stalled peer turns
 *      into MBEDTLS_ERR_SSL_WANT_READ and control comes back here. Preferred
 *      over O_NONBLOCK because it paces the retry loop instead of spinning.
 *   2. Liveness fed from inside every loop in handle_conn(), so the acceptor's
 *      liveness no longer depends on any connection making progress.
 *   3. A total connection budget comfortably inside the liveness deadline, so
 *      even the pathological case closes the socket long before the supervisor
 *      could notice.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>

#if defined(CONFIG_MBEDTLS)
/*
 * MUST come before the #if that tests MBEDTLS_SSL_KEYING_MATERIAL_EXPORT.
 *
 * That macro is not a Kconfig symbol: it reaches this translation unit only
 * through mbedTLS's own configuration chain (MBEDTLS_CONFIG_FILE →
 * config-mbedtls.h → CONFIG_MBEDTLS_USER_CONFIG_FILE → sts_mbedtls_user.h), and
 * build_info.h is what starts that chain. Every mbedTLS header this file needs
 * used to be included *inside* the guarded block, so the guard was evaluated
 * before any mbedTLS configuration existed and was therefore false no matter how
 * the tree was configured — the NTS-KE server could not be compiled in even with
 * the exporter fully wired. Pulling build_info.h in first is what makes the
 * guard mean what it says.
 */
#include <mbedtls/build_info.h>
#endif

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

bool sts_ntske_supported(void)
{
#if defined(MBEDTLS_SSL_KEYING_MATERIAL_EXPORT)
	return true;
#else
	return false;
#endif
}

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
#if !defined(MBEDTLS_ECP_C)
#include <psa/crypto.h>
#endif

#include "storage/sts_store.h"

#define NTSKE_STACK_SIZE 8192
#define NTSKE_PRIORITY 12
#define NTSKE_BACKLOG 2
#define NTSKE_REQ_MAX 2048U
#define NTSKE_RSP_MAX NTSKE_RSP_RECOMMENDED

/**
 * Total wall clock one connection may consume — handshake, request and
 * response together.
 *
 * Deliberately a fraction of CONFIG_STS1000_LIVENESS_DEADLINE_MS: the budget
 * has to expire, the socket has to close, and this thread has to be back in
 * ke_loop() well before the supervisor could conclude the area is wedged. A
 * TLS 1.3 handshake to a P-256 server is one round trip, so 1.5 s is generous
 * for any client that is actually trying.
 */
#define NTSKE_CONN_BUDGET_MS (CONFIG_STS1000_LIVENESS_DEADLINE_MS / 3)

/*
 * Both halves of the budget rule, checked at build time rather than trusted:
 * the connection budget must leave the supervisor room, and the socket timeout
 * must be short enough that one blocking call cannot overshoot the budget.
 */
BUILD_ASSERT(NTSKE_CONN_BUDGET_MS < CONFIG_STS1000_LIVENESS_DEADLINE_MS,
	     "a connection may not outlive the liveness deadline");

/**
 * SO_RCVTIMEO/SO_SNDTIMEO on the accepted socket.
 *
 * Short enough that the deadline is honoured promptly and the liveness feed
 * inside handle_conn() runs often, long enough that a healthy handshake almost
 * never sees it.
 */
#define NTSKE_SOCK_TIMEOUT_MS 200

BUILD_ASSERT(NTSKE_SOCK_TIMEOUT_MS * 4 < NTSKE_CONN_BUDGET_MS,
	     "the socket timeout must be small beside the connection budget");

/** ALPN protocol id required by RFC 8915 §4. */
/* Not `const char *const`: mbedtls_ssl_conf_alpn_protocols() takes
 * `const char **`, and it only reads the list. */
static const char *alpn_list[] = { "ntske/1", NULL };

static ntske_ctx_t ke;
static mbedtls_ssl_config tls_conf;
static mbedtls_x509_crt srv_cert;
static mbedtls_pk_context srv_key;
#if !defined(MBEDTLS_ECP_C)
/* The PSA key behind srv_key. mbedtls_pk_free() does not destroy it (pk.h
 * §mbedtls_pk_free), so tls_teardown() must. */
static mbedtls_svc_key_id_t srv_key_id = MBEDTLS_SVC_KEY_ID_INIT;
#endif
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
		/* MBEDTLS_ERR_NET_* live in mbedtls/net_sockets.h, which Zephyr does
		 * not build (it has its own socket layer), so the idiomatic fatal
		 * BIO error here is the one Zephyr's own sockets_tls.c returns. */
		return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
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
		return MBEDTLS_ERR_SSL_INTERNAL_ERROR; /* see bio_send() */
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

/**
 * Generate the server's P-256 key into @ref srv_key.
 *
 * Two paths, chosen by what mbedTLS was actually built with rather than by
 * assumption. The legacy path (mbedtls_ecp_gen_key over mbedtls_pk_ec()) only
 * exists when MBEDTLS_ECP_C is enabled; Zephyr's PSA-backed configuration —
 * which is the one that comes with TLS 1.3 — turns ECP_C off and keeps EC keys
 * inside PSA, where mbedtls_pk_ec() is not merely deprecated but absent. The
 * file previously used the legacy call unconditionally, which is one of the
 * reasons the guarded body had never been compiled (F13).
 */
static int gen_p256_key(void)
{
#if defined(MBEDTLS_ECP_C)
	int rc = mbedtls_pk_setup(&srv_key,
				  mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY));

	if (rc != 0) {
		return rc;
	}
	return mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1,
				   mbedtls_pk_ec(srv_key),
				   mbedtls_ctr_drbg_random, &drbg);
#else
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t st_psa;
	int rc;

	psa_set_key_type(&attr,
			 PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
	psa_set_key_bits(&attr, 256U);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_SIGN_HASH);
	psa_set_key_algorithm(&attr, PSA_ALG_ECDSA(PSA_ALG_SHA_256));

	st_psa = psa_generate_key(&attr, &srv_key_id);
	if (st_psa != PSA_SUCCESS) {
		LOG_ERR("psa_generate_key: %d", (int)st_psa);
		return MBEDTLS_ERR_PK_ALLOC_FAILED;
	}
	rc = mbedtls_pk_setup_opaque(&srv_key, srv_key_id);
	if (rc != 0) {
		(void)psa_destroy_key(srv_key_id);
		srv_key_id = MBEDTLS_SVC_KEY_ID_INIT;
	}
	return rc;
#endif
}

static int make_self_signed_cert(void)
{
	mbedtls_x509write_cert w;
	mbedtls_mpi serial;
	uint8_t der[768];
	int len;
	int rc;

	/* srv_key / srv_cert are initialised by tls_setup(), so tls_teardown()
	 * can free them however early this fails. */
	mbedtls_x509write_crt_init(&w);
	mbedtls_mpi_init(&serial);

	rc = gen_p256_key();
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

/**
 * Release everything tls_setup() may have initialised.
 *
 * Every tls_setup() failure path used to return straight to sts_ntske_start()
 * leaving the entropy source, the DRBG, the generated key and the certificate
 * behind. One boot's worth is not a leak that grows, but it is ~2 KB of heap and
 * a live entropy context held by a subsystem that then reports itself
 * unavailable, and the key material outlives its only user (F13).
 */
static void tls_teardown(void)
{
	mbedtls_ssl_config_free(&tls_conf);
	mbedtls_x509_crt_free(&srv_cert);
	mbedtls_pk_free(&srv_key);
#if !defined(MBEDTLS_ECP_C)
	if (mbedtls_svc_key_id_is_null(srv_key_id) == 0) {
		(void)psa_destroy_key(srv_key_id);
		srv_key_id = MBEDTLS_SVC_KEY_ID_INIT;
	}
#endif
	mbedtls_ctr_drbg_free(&drbg);
	mbedtls_entropy_free(&entropy);
}

static int tls_setup(void)
{
	int rc;

	mbedtls_ssl_config_init(&tls_conf);
	mbedtls_entropy_init(&entropy);
	mbedtls_ctr_drbg_init(&drbg);
	/* Initialised here too, so tls_teardown() is safe on every failure path
	 * including one taken before make_self_signed_cert() runs. */
	mbedtls_pk_init(&srv_key);
	mbedtls_x509_crt_init(&srv_cert);

	rc = mbedtls_ctr_drbg_seed(&drbg, mbedtls_entropy_func, &entropy,
				   (const uint8_t *)"sts1000-ntske", 13);
	if (rc != 0) {
		LOG_ERR("ctr_drbg_seed: -0x%04x", (unsigned int)-rc);
		goto fail;
	}

	rc = make_self_signed_cert();
	if (rc != 0) {
		LOG_ERR("cert generation: -0x%04x", (unsigned int)-rc);
		goto fail;
	}

	rc = mbedtls_ssl_config_defaults(&tls_conf, MBEDTLS_SSL_IS_SERVER,
					 MBEDTLS_SSL_TRANSPORT_STREAM,
					 MBEDTLS_SSL_PRESET_DEFAULT);
	if (rc != 0) {
		goto fail;
	}

	mbedtls_ssl_conf_min_tls_version(&tls_conf, MBEDTLS_SSL_VERSION_TLS1_3);
	mbedtls_ssl_conf_max_tls_version(&tls_conf, MBEDTLS_SSL_VERSION_TLS1_3);
	mbedtls_ssl_conf_authmode(&tls_conf, MBEDTLS_SSL_VERIFY_NONE);
	mbedtls_ssl_conf_rng(&tls_conf, mbedtls_ctr_drbg_random, &drbg);

	rc = mbedtls_ssl_conf_own_cert(&tls_conf, &srv_cert, &srv_key);
	if (rc != 0) {
		LOG_ERR("conf_own_cert: -0x%04x", (unsigned int)-rc);
		goto fail;
	}
	rc = mbedtls_ssl_conf_alpn_protocols(&tls_conf, alpn_list);
	if (rc != 0) {
		goto fail;
	}
	return 0;

fail:
	tls_teardown();
	return -EIO;
}

/* ------------------------------------------------------------------------- */
/* one connection                                                            */
/* ------------------------------------------------------------------------- */

/** Feed the area's liveness bit; safe to call as often as we like. */
static void conn_alive(void)
{
	if (live_id >= 0) {
		sts_liveness_feed(live_id);
	}
}

/**
 * Has the client's whole request flight arrived?
 *
 * NTS-KE frames a request as a sequence of records terminated by End of Message
 * (RFC 8915 §4.1.1), and TLS record boundaries have nothing to do with those
 * boundaries. Stopping after the first mbedtls_ssl_read() — which returns at
 * most one TLS record — therefore lost the AEAD and EOM records of any client
 * that split its flight, and the negotiation failed with BAD_REQUEST through no
 * fault of the client (F11). So walk what we have and only stop when the
 * terminator is actually present.
 *
 * @return true once a complete record sequence ending in EOM is buffered.
 */
static bool request_complete(const uint8_t *buf, size_t len)
{
	size_t off = 0U;

	while (off < len) {
		ntske_rec_t rec;
		size_t next = off;

		if (ntske_rec_parse(buf, len, off, &rec, &next) != 0) {
			return false; /* truncated: more to come */
		}
		if (rec.type == NTSKE_REC_EOM) {
			return true;
		}
		if (next <= off) {
			return false; /* no forward progress; treat as truncated */
		}
		off = next;
	}
	return false;
}

static void handle_conn(int fd)
{
	mbedtls_ssl_context ssl;
	ntske_result_t res;
	const char *alpn;
	size_t total = 0U;
	size_t rsp_len = 0U;
	int rc;
	int64_t deadline = (int64_t)sts_mono_ms() + NTSKE_CONN_BUDGET_MS;
	struct zsock_timeval tv = {
		.tv_sec = NTSKE_SOCK_TIMEOUT_MS / 1000,
		.tv_usec = (NTSKE_SOCK_TIMEOUT_MS % 1000) * 1000,
	};

	mbedtls_ssl_init(&ssl);

	/*
	 * Before anything reads or writes: without these the socket blocks
	 * K_FOREVER, mbedTLS never hands control back, and every deadline below
	 * is dead code. See the DoS note in the file header.
	 */
	if (zsock_setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0 ||
	    zsock_setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
		LOG_ERR("NTS-KE: socket timeouts unavailable (%d); refusing the "
			"connection rather than risking a blocking stall", errno);
		K_SPINLOCK(&st_lock) {
			st.handshakes_failed++;
		}
		goto done;
	}

	rc = mbedtls_ssl_setup(&ssl, &tls_conf);
	if (rc != 0) {
		goto done;
	}
	mbedtls_ssl_set_bio(&ssl, (void *)(intptr_t)fd, bio_send, bio_recv,
			    NULL);

	ke.cfg.export_ctx = &ssl; /* the exporter needs this exact context */

	do {
		conn_alive();
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

	while (total < sizeof(req_buf)) {
		conn_alive();
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
			break; /* close_notify, or a fatal record error */
		}
		total += (size_t)rc;
		if (request_complete(req_buf, total)) {
			/* The flight is in. Do not wait for the client's close:
			 * it sends that only after reading our response. */
			break;
		}
		if ((int64_t)sts_mono_ms() > deadline) {
			goto close_notify;
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
			conn_alive();
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
			if ((int64_t)sts_mono_ms() > deadline) {
				break;
			}
		}
	}

close_notify:
	(void)mbedtls_ssl_close_notify(&ssl);
done:
	ke.cfg.export_ctx = NULL;
	mbedtls_ssl_free(&ssl);
	(void)zsock_close(fd);
	conn_alive();
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

		conn_alive();
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
		/* Serialised by construction: exactly one handshake is ever in
		 * flight, and handle_conn() feeds liveness from inside its own
		 * loops so this thread's liveness no longer depends on the peer
		 * making progress (F4). */
		handle_conn(cfd);
		k_yield();
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
		tls_teardown();
		return rc;
	}

	listen_fd = open_listener();
	if (listen_fd < 0) {
		LOG_ERR("NTS-KE listen on :%u failed: %d", ke_port, listen_fd);
		tls_teardown();
		return listen_fd;
	}

	/* Registered only now that the thread is about to exist: a registered
	 * participant that never feeds withholds the watchdog kick. */
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
