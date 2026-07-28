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
 * WIRING (cross-area — see sts_ntske.c): Zephyr's mbedTLS library is compiled
 * with the module's own include paths, not the app's, so this header must be
 * reachable from there. app/CMakeLists.txt carries one line, AFTER
 * find_package(Zephyr):
 *
 *     zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/zephyr/net)
 *
 * and app/conf/net.conf carries the two Kconfig lines that select this file
 * (CONFIG_MBEDTLS_USER_CONFIG_ENABLE / _FILE). Both are in place; NTS-KE is
 * live.
 *
 * "After", not before: this comment used to say before, and that placement
 * cannot work — zephyr_include_directories is defined by Zephyr's
 * extensions.cmake and appends to the `zephyr_interface` target, neither of
 * which exists until find_package(Zephyr) has run, so the configure step fails
 * with `Unknown CMake command`. Appending after still reaches the mbedTLS
 * libraries because CMake resolves INTERFACE usage requirements at generate
 * time, not at target-creation time.
 *
 * sts_ntske.c still guards every use of the exporter with
 * `#if defined(MBEDTLS_SSL_KEYING_MATERIAL_EXPORT)`, so the image links and the
 * NTS-KE listener cleanly reports "exporter unavailable" if this overlay is ever
 * unwired — the build is never broken by its absence. That guard is also the
 * check to run after touching any of this: if
 * `objdump -d --disassemble=sts_ntske_supported zephyr.elf` is `movs r0,#0`,
 * the header did not reach the compile and NTS-KE is silently off.
 */

#ifndef STS1000_ZEPHYR_NET_STS_MBEDTLS_USER_H_
#define STS1000_ZEPHYR_NET_STS_MBEDTLS_USER_H_

#if !defined(MBEDTLS_SSL_KEYING_MATERIAL_EXPORT)
#define MBEDTLS_SSL_KEYING_MATERIAL_EXPORT
#endif

#endif /* STS1000_ZEPHYR_NET_STS_MBEDTLS_USER_H_ */
