/*
 * STS1000 "Meridian" — core/util: CRC primitives.
 *
 * Platform-neutral C11. No dynamic allocation, no platform headers.
 *
 * Two catalogue CRCs, both nibble-table driven (16-entry tables, 64 B / 32 B of
 * rodata) so the cost is bounded on the STM32H5 without pulling in a 1 KB table:
 *
 *   sts_crc32_ieee   CRC-32/ISO-HDLC ("IEEE 802.3", zlib crc32)
 *                width=32 poly=0x04C11DB7 init=0xFFFFFFFF refin=true
 *                refout=true xorout=0xFFFFFFFF  check("123456789")=0xCBF43926
 *                Used for the MCP frame trailer (ARCHITECTURE.md §7) and image
 *                integrity checks.
 *
 *   sts_crc16_ccitt  CRC-16/IBM-3740, commonly "CRC-16/CCITT-FALSE"
 *                width=16 poly=0x1021 init=0xFFFF refin=false refout=false
 *                xorout=0x0000  check("123456789")=0x29B1
 *
 * Both come in a one-shot form and a seeded "update" form so a caller can run
 * the CRC across a scattered buffer list or a byte stream without staging.
 */

#ifndef STS1000_CORE_UTIL_CRC_H_
#define STS1000_CORE_UTIL_CRC_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Seed for a fresh sts_crc32_ieee_update() chain.
 *
 * This is the zlib convention: the seed and the returned value are both the
 * *finalised* CRC, so chaining composes naturally and
 * sts_crc32_ieee_update(STS_CRC32_IEEE_SEED, d, n) == sts_crc32_ieee(d, n).
 */
#define STS_CRC32_IEEE_SEED 0x00000000U

/** Seed for a fresh sts_crc16_ccitt_update() chain (the algorithm's init value). */
#define STS_CRC16_CCITT_SEED 0xFFFFU

/**
 * Continue a CRC-32/ISO-HDLC over @p len bytes at @p data.
 *
 * @param crc   Result of the previous call, or STS_CRC32_IEEE_SEED to start.
 * @param data  Input bytes. May be NULL only when @p len is 0; a NULL pointer
 *              is treated as "no data" and returns @p crc unchanged rather than
 *              faulting, because core code must not trap the host test runner.
 * @param len   Number of bytes at @p data.
 * @return      The running CRC after the supplied bytes.
 */
uint32_t sts_crc32_ieee_update(uint32_t crc, const void *data, size_t len);

/**
 * One-shot CRC-32/ISO-HDLC. Equivalent to
 * sts_crc32_ieee_update(STS_CRC32_IEEE_SEED, data, len).
 */
uint32_t sts_crc32_ieee(const void *data, size_t len);

/**
 * Continue a CRC-16/CCITT-FALSE over @p len bytes at @p data.
 *
 * @param crc   Result of the previous call, or STS_CRC16_CCITT_SEED to start.
 * @param data  Input bytes; NULL is treated as "no data" (see sts_crc32_ieee_update).
 * @param len   Number of bytes at @p data.
 * @return      The running CRC after the supplied bytes.
 */
uint16_t sts_crc16_ccitt_update(uint16_t crc, const void *data, size_t len);

/**
 * One-shot CRC-16/CCITT-FALSE. Equivalent to
 * sts_crc16_ccitt_update(STS_CRC16_CCITT_SEED, data, len).
 */
uint16_t sts_crc16_ccitt(const void *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_UTIL_CRC_H_ */
