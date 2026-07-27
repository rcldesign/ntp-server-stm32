/*
 * STS1000 "Meridian" — core/util: endian-explicit byte accessors.
 *
 * Platform-neutral C11, header-only. No dynamic allocation, no platform headers.
 *
 * Every wire format this firmware touches names its own byte order — MCP is
 * little-endian (ARCHITECTURE.md §7), UBX is little-endian, NTP/NTS/PTP/SNMP are
 * big-endian — so the accessors are explicit rather than host-relative. There is
 * deliberately no htons()/ntohs() analogue: nothing here depends on the host
 * byte order, which also makes the host unit tests representative of the target.
 *
 * All access is byte-wise, so any alignment is legal and no strict-aliasing or
 * unaligned-load rule is bent. On Cortex-M33 at -O2 GCC folds the common
 * 16/32-bit cases back into a single load plus REV where it is allowed to.
 *
 * Contract: pointers must be non-NULL and address at least the accessed width.
 * These are leaf accessors on per-packet paths and do not validate.
 */

#ifndef STS1000_CORE_UTIL_BYTES_H_
#define STS1000_CORE_UTIL_BYTES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- get, LE */

/** Read a little-endian uint16 from @p p. */
static inline uint16_t bytes_get_le16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/** Read a little-endian uint32 from @p p. */
static inline uint32_t bytes_get_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/** Read a little-endian uint64 from @p p. */
static inline uint64_t bytes_get_le64(const uint8_t *p)
{
	return (uint64_t)bytes_get_le32(p) |
	       ((uint64_t)bytes_get_le32(&p[4]) << 32);
}

/* ---------------------------------------------------------------- get, BE */

/** Read a big-endian uint16 from @p p. */
static inline uint16_t bytes_get_be16(const uint8_t *p)
{
	return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/** Read a big-endian uint32 from @p p. */
static inline uint32_t bytes_get_be32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
	       ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

/** Read a big-endian uint64 from @p p. */
static inline uint64_t bytes_get_be64(const uint8_t *p)
{
	return ((uint64_t)bytes_get_be32(p) << 32) |
	       (uint64_t)bytes_get_be32(&p[4]);
}

/* ---------------------------------------------------------------- put, LE */

/** Write @p v to @p p as a little-endian uint16. */
static inline void bytes_put_le16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFU);
	p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

/** Write @p v to @p p as a little-endian uint32. */
static inline void bytes_put_le32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFU);
	p[1] = (uint8_t)((v >> 8) & 0xFFU);
	p[2] = (uint8_t)((v >> 16) & 0xFFU);
	p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

/** Write @p v to @p p as a little-endian uint64. */
static inline void bytes_put_le64(uint8_t *p, uint64_t v)
{
	bytes_put_le32(p, (uint32_t)(v & 0xFFFFFFFFU));
	bytes_put_le32(&p[4], (uint32_t)((v >> 32) & 0xFFFFFFFFU));
}

/* ---------------------------------------------------------------- put, BE */

/** Write @p v to @p p as a big-endian uint16. */
static inline void bytes_put_be16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)((v >> 8) & 0xFFU);
	p[1] = (uint8_t)(v & 0xFFU);
}

/** Write @p v to @p p as a big-endian uint32. */
static inline void bytes_put_be32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)((v >> 24) & 0xFFU);
	p[1] = (uint8_t)((v >> 16) & 0xFFU);
	p[2] = (uint8_t)((v >> 8) & 0xFFU);
	p[3] = (uint8_t)(v & 0xFFU);
}

/** Write @p v to @p p as a big-endian uint64. */
static inline void bytes_put_be64(uint8_t *p, uint64_t v)
{
	bytes_put_be32(p, (uint32_t)((v >> 32) & 0xFFFFFFFFU));
	bytes_put_be32(&p[4], (uint32_t)(v & 0xFFFFFFFFU));
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_UTIL_BYTES_H_ */
