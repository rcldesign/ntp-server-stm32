/*
 * STS1000 "Meridian" — web management plane (spec §5.1/§5.2, §9.2): glue header.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/net/. Three files sit behind it:
 *
 *   sts_web.c    TLS listener on 443, the worker pool, the REST/WSS pump, the
 *                provider bindings and the DFU bridge.
 *   sts_cert.c   certificate lifecycle: operator PEM, persisted self-signed,
 *                CSR, fingerprints, and the (compiled-out) ACME skeleton.
 *   sts_webfs.c  the packed SPA out of /lfs/www, with a compiled-in fallback.
 *
 * Why the area is self-starting
 * -----------------------------
 * sts_net_start() is owned by another author and is not edited by this wave, so
 * the web area cannot be called from it. Instead sts_web.c defines its own
 * static threads and each one waits for the two things it needs — a live cfg
 * context and a network interface — before binding a socket. That is strictly
 * more robust than an ordering assumption: a build where the platform area
 * never comes up simply never opens port 443.
 *
 * Tunables are #define, not Kconfig
 * ---------------------------------
 * The net area has no Kconfig fragment of its own, and app/Kconfig (which would
 * have to `rsource` one) belongs to the platform area. Rather than reach into
 * another owner's file, every tunable here is an overridable #define. Promoting
 * them is a one-line app/Kconfig edit for whoever owns it; see the report.
 */

#ifndef STS1000_ZEPHYR_NET_STS_WEB_H_
#define STS1000_ZEPHYR_NET_STS_WEB_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "web/rest.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ tunables */

/** HTTPS port. */
#ifndef STS_WEB_TLS_PORT
#define STS_WEB_TLS_PORT 443
#endif

/** Plain-HTTP port used only to redirect to HTTPS. 0 disables the listener. */
#ifndef STS_WEB_REDIRECT_PORT
#define STS_WEB_REDIRECT_PORT 80
#endif

/**
 * Concurrent connection workers.
 *
 * Each worker costs a stack plus a receive and a response buffer, and each live
 * TLS connection costs an mbedTLS session (its in/out content buffers dominate).
 * Two is the smallest number that lets the SPA hold its telemetry WebSocket open
 * *and* still answer a REST call, which is the actual requirement; three would
 * add ~30 KB of SRAM for a second concurrent operator. `CONFIG_NET_SOCKETS_TLS_
 * MAX_CONTEXTS` in web.conf must be at least this + 1 (the listener).
 */
#ifndef STS_WEB_WORKERS
#define STS_WEB_WORKERS 2
#endif

/** Worker thread stack, bytes. TLS handshake + REST encoding, no recursion. */
#ifndef STS_WEB_STACK_SIZE
#define STS_WEB_STACK_SIZE 6144
#endif

/** Thread priority. ARCHITECTURE.md §6: web/tls share band 12 with SNMP. */
#ifndef STS_WEB_PRIORITY
#define STS_WEB_PRIORITY 12
#endif

/** Receive buffer per worker: the request head plus the largest body. */
#ifndef STS_WEB_RX_SIZE
#define STS_WEB_RX_SIZE (HTTP_HEAD_MAX + HTTP_BODY_MAX + 64U)
#endif

/**
 * Response buffer per worker.
 *
 * Sized for the largest single document the API emits in one shot: a page of
 * config keys (see REST_CONFIG_PAGE_DEFAULT), a power snapshot with nine rails,
 * or a log page. Larger collections page through `next_id` / `next_cursor`
 * rather than growing this.
 */
#ifndef STS_WEB_RESP_SIZE
#define STS_WEB_RESP_SIZE 8192U
#endif

/** WebSocket message reassembly buffer per worker. */
#ifndef STS_WEB_WS_MSG_SIZE
#define STS_WEB_WS_MSG_SIZE 1024U
#endif

/** Seconds a keep-alive connection may sit idle before it is closed. */
#ifndef STS_WEB_IDLE_TIMEOUT_S
#define STS_WEB_IDLE_TIMEOUT_S 30
#endif

/** Milliseconds a TLS handshake may take before the worker gives up. */
#ifndef STS_WEB_HANDSHAKE_TIMEOUT_MS
#define STS_WEB_HANDSHAKE_TIMEOUT_MS 10000
#endif

/** Seconds between WebSocket keepalive pings. */
#ifndef STS_WEB_WS_PING_S
#define STS_WEB_WS_PING_S 20
#endif

/** TLS credential security tag the listener binds. */
#ifndef STS_WEB_SEC_TAG
#define STS_WEB_SEC_TAG 0x57454231 /* "WEB1" */
#endif

/* ------------------------------------------------------------------ sts_web.c */

/** Counters, for the shell and the status encoders. */
typedef struct {
	uint32_t accepted;
	uint32_t handshakes_ok;
	uint32_t handshakes_failed;
	uint32_t requests;
	uint32_t responses_2xx;
	uint32_t responses_4xx;
	uint32_t responses_5xx;
	uint32_t ws_upgrades;
	uint32_t ws_closed;
	uint32_t ws_frames_tx;
	uint32_t ws_dropped;
	uint32_t redirects;
	uint32_t rejected_oversize;
	uint32_t tls_ready;   /* 1 once a credential is loaded */
	bool     running;
} sts_web_stats_t;

void sts_web_stats(sts_web_stats_t *out);

/** True once the HTTPS listener is bound. */
bool sts_web_running(void);

/* ----------------------------------------------------------------- sts_cert.c */

/** Directory the certificate and key live in. */
#define STS_CERT_DIR "/lfs/tls"
/** Operator-supplied or generated certificate, PEM. */
#define STS_CERT_CRT_PATH STS_CERT_DIR "/server.crt.pem"
/** Matching private key, PEM. */
#define STS_CERT_KEY_PATH STS_CERT_DIR "/server.key.pem"
/** ACME account key, PEM (only used when ACME is compiled in). */
#define STS_CERT_ACME_KEY_PATH STS_CERT_DIR "/acme_account.key.pem"

/** Largest PEM blob accepted for a certificate or a key. */
#ifndef STS_CERT_PEM_MAX
#define STS_CERT_PEM_MAX 3072U
#endif

/**
 * Load or create the server credential and register it under STS_WEB_SEC_TAG.
 *
 * Order of preference:
 *   1. an operator-supplied PEM pair already on /lfs;
 *   2. a self-signed pair this firmware previously generated and persisted;
 *   3. a freshly generated ECDSA P-256 self-signed pair, persisted to /lfs so
 *      the fingerprint survives a reboot (a per-boot certificate would break
 *      every pin and re-prompt the operator on every restart);
 *   4. if /lfs is unavailable, the same pair held only in RAM, with the
 *      "not persisted" flag set and a warning logged.
 *
 * @retval 0        A credential is registered.
 * @retval -ENOTSUP This build has no X.509 write support.
 * @retval <0       Generation or registration failed; HTTPS must not start.
 */
int sts_cert_init(void);

/** Fill @p out with the live certificate's description (rest_cert_t). */
int sts_cert_info(rest_cert_t *out);

/**
 * Install an operator-supplied PEM bundle (certificate then private key, in
 * either order, concatenated).
 *
 * The pair is validated — the certificate must parse, the key must parse, and
 * the key must match the certificate's public key — and only then written to
 * /lfs and re-registered.
 *
 * @retval 0        Installed.
 * @retval -EBADMSG Not a usable PEM certificate + key pair.
 * @retval -EPERM   The key does not match the certificate.
 * @retval -EROFS   Accepted and live, but /lfs could not persist it.
 */
int sts_cert_install(const char *pem, size_t len);

/**
 * Generate a PKCS#10 CSR for the live key.
 *
 * @param subject  Subject DN, or NULL for a default built from the hostname.
 * @retval 0        @p out holds a NUL-terminated PEM CSR of @p out_len bytes.
 * @retval -ENOSPC  @p cap too small.
 */
int sts_cert_csr(const char *subject, char *out, size_t cap, size_t *out_len);

/* ---------------------------------------------------------------- sts_webfs.c */

/** Root of the packed SPA on the external NOR volume. */
#define STS_WEBFS_ROOT "/lfs/www"

/** A resolved static asset. */
typedef struct {
	const char *content_type;
	const char *etag;         /* quoted; "" when unknown */
	bool        gzip;         /* body is gzip-encoded */
	bool        from_flash;   /* body is @ref data, not a file */
	const uint8_t *data;      /* valid when from_flash */
	size_t      len;          /* body length in bytes */
	char        path[64];     /* file to stream when !from_flash */
} sts_webfs_asset_t;

/** Read the /lfs/www manifest, if there is one. Safe to call repeatedly. */
void sts_webfs_init(void);

/**
 * Resolve a request path to an asset.
 *
 * "/" maps to index.html, and any path that is not a known file also maps to
 * index.html so the SPA's hash router survives a deep-link reload. Path
 * traversal ("..", "//", a leading '.') is refused outright.
 *
 * @param gzip_ok  The client sent `Accept-Encoding: gzip`.
 * @retval 0        Resolved.
 * @retval -EINVAL  Bad argument or a rejected path.
 * @retval -ENOENT  No file and no compiled-in fallback for it.
 */
int sts_webfs_resolve(const char *path, size_t path_len, bool gzip_ok,
		      sts_webfs_asset_t *out);

/**
 * Copy up to @p cap bytes of @p a's body starting at @p off.
 *
 * @retval >=0      Bytes copied (0 at end of body).
 * @retval <0       I/O error.
 */
int sts_webfs_read(const sts_webfs_asset_t *a, size_t off, uint8_t *buf,
		   size_t cap);

/** True when the packed SPA was found on /lfs (as opposed to the fallback). */
bool sts_webfs_have_bundle(void);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_NET_STS_WEB_H_ */
