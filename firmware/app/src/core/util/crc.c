/*
 * STS1000 "Meridian" — core/util: CRC primitives.
 *
 * Nibble-at-a-time table implementations. Two table lookups per input byte;
 * 16 entries each, so 64 B (CRC-32) + 32 B (CRC-16) of rodata versus the 1 KB a
 * byte-wide CRC-32 table would cost.
 */

#include "util/crc.h"

/*
 * CRC-32/ISO-HDLC, reflected form. tab32[i] is the reflected remainder of the
 * 4-bit value i under the reflected polynomial 0xEDB88320.
 */
static const uint32_t tab32[16] = {
	0x00000000U, 0x1DB71064U, 0x3B6E20C8U, 0x26D930ACU,
	0x76DC4190U, 0x6B6B51F4U, 0x4DB26158U, 0x5005713CU,
	0xEDB88320U, 0xF00F9344U, 0xD6D6A3E8U, 0xCB61B38CU,
	0x9B64C2B0U, 0x86D3D2D4U, 0xA00AE278U, 0xBDBDF21CU,
};

/*
 * CRC-16/CCITT-FALSE, non-reflected form. tab16[i] is the remainder of i << 12
 * under the polynomial 0x1021.
 */
static const uint16_t tab16[16] = {
	0x0000U, 0x1021U, 0x2042U, 0x3063U,
	0x4084U, 0x50A5U, 0x60C6U, 0x70E7U,
	0x8108U, 0x9129U, 0xA14AU, 0xB16BU,
	0xC18CU, 0xD1ADU, 0xE1CEU, 0xF1EFU,
};

uint32_t sts_crc32_ieee_update(uint32_t crc, const void *data, size_t len)
{
	const uint8_t *p = (const uint8_t *)data;
	uint32_t c;

	if (p == NULL) {
		return crc;
	}

	/* Undo the output XOR to recover the running register. */
	c = ~crc;

	while (len-- != 0U) {
		c ^= (uint32_t)*p++;
		c = (c >> 4) ^ tab32[c & 0x0FU];
		c = (c >> 4) ^ tab32[c & 0x0FU];
	}

	return ~c;
}

uint32_t sts_crc32_ieee(const void *data, size_t len)
{
	return sts_crc32_ieee_update(STS_CRC32_IEEE_SEED, data, len);
}

uint16_t sts_crc16_ccitt_update(uint16_t crc, const void *data, size_t len)
{
	const uint8_t *p = (const uint8_t *)data;
	uint16_t c = crc;

	if (p == NULL) {
		return crc;
	}

	while (len-- != 0U) {
		uint8_t b = *p++;

		c = (uint16_t)((c << 4) ^ tab16[((c >> 12) ^ (b >> 4)) & 0x0FU]);
		c = (uint16_t)((c << 4) ^ tab16[((c >> 12) ^ (b & 0x0FU)) & 0x0FU]);
	}

	return c;
}

uint16_t sts_crc16_ccitt(const void *data, size_t len)
{
	return sts_crc16_ccitt_update(STS_CRC16_CCITT_SEED, data, len);
}
