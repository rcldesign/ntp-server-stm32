/*
 * STS1000 "Meridian" — ATECC608B secure-element services (spec §9.1).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Thin, thread-safe façade over core/atecc and I²C1. Every entry point takes
 * the bus mutex, wakes the part, runs one operation and idles it again, so
 * callers need no ordering discipline of their own.
 *
 * **Every entry point returns -ENODEV when the part is absent, unprovisioned or
 * disabled by `sec.atecc.en`.** That is not an error path to propagate: it means
 * "use the software key" (port_crypto over PSA/mbedTLS). The absence is logged
 * once and never blocks boot.
 *
 * Not callable from an ISR: these block on a mutex and on I²C.
 */

#ifndef STS1000_ZEPHYR_STORAGE_STS_ATECC_H_
#define STS1000_ZEPHYR_STORAGE_STS_ATECC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/drivers/hwinfo.h>

#include "atecc/atecc.h"

#ifdef __cplusplus
extern "C" {
#endif

/** True when the part answered a wake handshake. Probes on first call. */
bool sts_atecc_present(void);

/** Apply `sec.atecc.en` / `sec.atecc.slot` from cfg. Safe to call repeatedly. */
void sts_atecc_reapply(void);

/** Enable/disable and select the device key slot directly (for tests/shell). */
void sts_atecc_configure(bool enable, uint8_t slot);

/** The nine-octet factory serial number. */
int sts_atecc_serial(uint8_t out[ATECC_SERIAL_LEN]);

/**
 * The serial number as 18 upper-case hex characters plus a NUL.
 *
 * @param cap  At least 19.
 * @retval >0  Characters written (18).
 */
int sts_atecc_serial_string(char *out, size_t cap);

/** Hardware TRNG bytes. Zeroes @p out on failure so a partial fill is never used. */
int sts_atecc_random(uint8_t *out, size_t len);

/** ECDSA P-256 signature over @p digest with the device key. @p sig is R‖S. */
int sts_atecc_sign(const uint8_t digest[32], uint8_t sig[64]);

/** The device key's public point, X‖Y, 32 octets each. */
int sts_atecc_pubkey(uint8_t pub[64]);

/** Verify @p sig over @p digest against @p pub. @retval -EBADE on miscompare. */
int sts_atecc_verify(const uint8_t pub[64], const uint8_t digest[32],
		     const uint8_t sig[64]);

/** ECDH: X coordinate of @p peer × the device key. */
int sts_atecc_ecdh(const uint8_t peer[64], uint8_t secret[32]);

/** Read monotonic counter @p idx (0..1). Anti-rollback evidence. */
int sts_atecc_counter_read(uint8_t idx, uint32_t *out);

/**
 * Increment monotonic counter @p idx and return the new value.
 *
 * One-way and finite. The only intended caller is the anti-rollback step of a
 * confirmed firmware upgrade — never anything that runs on every boot.
 */
int sts_atecc_counter_increment(uint8_t idx, uint32_t *out);

/** Collect the `sec.attest` report (serial, revision, counters, locks, keys). */
int sts_atecc_attest(atecc_attest_t *out);

/** Transport counters, for `diag`. @retval -ENODEV before the first use. */
int sts_atecc_stats(atecc_stats_t *out);

/**
 * A device-unique value for the SNMP engine id (RFC 3411 §5) and the USB serial
 * string.
 *
 * Prefers the ATECC serial number; falls back to the SoC's hardware device id
 * when the part is absent. @p out_from_atecc says which, so a caller can log
 * that the identity is not hardware-rooted.
 *
 * @retval 0        @p out_len octets written.
 * @retval -EINVAL  NULL argument or zero capacity.
 * @retval -ENODEV  Neither source produced anything.
 */
int sts_atecc_device_unique(uint8_t *out, size_t cap, size_t *out_len,
			    bool *out_from_atecc);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_STORAGE_STS_ATECC_H_ */
