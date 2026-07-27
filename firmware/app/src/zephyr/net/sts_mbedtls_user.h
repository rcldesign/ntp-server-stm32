/*
 * STS1000 "Meridian" — mbedTLS user configuration overlay.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Included at the end of Zephyr's config-mbedtls.h when
 * CONFIG_MBEDTLS_USER_CONFIG_ENABLE=y and CONFIG_MBEDTLS_USER_CONFIG_FILE names
 * this file. It turns on exactly one feature the stock Zephyr mbedTLS config
 * omits and offers no Kconfig for:
 *
 *   MBEDTLS_SSL_KEYING_MATERIAL_EXPORT — mbedtls_ssl_export_keying_material(),
 *       the RFC 8446 §7.5 / RFC 5705 TLS exporter. RFC 8915 §4.3 derives the
 *       NTS C2S and S2C keys from it with the label
 *       "EXPORTER-network-time-security"; without it NTS-KE cannot mint keys
 *       and the whole NTS path is inert.
 *
 * WIRING (cross-area — see sts_ntske.c and the W3b report): Zephyr's mbedTLS
 * library is compiled with the module's own include paths, not the app's, so
 * this header must be reachable from there. app/CMakeLists.txt (owned by the
 * platform/W3a area) needs one line before find_package(Zephyr):
 *
 *     zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/zephyr/net)
 *
 * and app/conf/net.conf carries the two Kconfig lines that select this file.
 * sts_ntske.c guards every use of the exporter with
 * `#if defined(MBEDTLS_SSL_KEYING_MATERIAL_EXPORT)`, so the image links and the
 * NTS-KE listener cleanly reports "exporter unavailable" if this overlay is not
 * wired — the build is never broken by its absence.
 */

#ifndef STS1000_ZEPHYR_NET_STS_MBEDTLS_USER_H_
#define STS1000_ZEPHYR_NET_STS_MBEDTLS_USER_H_

#if !defined(MBEDTLS_SSL_KEYING_MATERIAL_EXPORT)
#define MBEDTLS_SSL_KEYING_MATERIAL_EXPORT
#endif

#endif /* STS1000_ZEPHYR_NET_STS_MBEDTLS_USER_H_ */
