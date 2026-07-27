/*
 * STS1000 "Meridian" — anti-rollback policy and MCUboot image-header decoding,
 * as pure arithmetic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/console/, and — like sts_confirm_gate.h next to it —
 * deliberately free of every Zephyr, MCUboot and hardware dependency so
 * tests/host can compile it and pin the decisions down. Everything here is
 * either a byte-layout decode that is wrong silently or a one-way hardware
 * action that cannot be undone, which is the wrong combination to leave
 * untested.
 *
 * What anti-rollback is made of on this product
 * ---------------------------------------------
 * Three parts, in decreasing order of how much they matter:
 *
 *   1. THE GATE. MCUboot refuses to swap in an image whose IMAGE_TLV_SEC_CNT is
 *      lower than the running image's (sysbuild/mcuboot.conf:
 *      CONFIG_MCUBOOT_DOWNGRADE_PREVENTION_SECURITY_COUNTER). Both counters sit
 *      in the PROTECTED TLV area, so both are covered by the ECDSA signature.
 *      This is the part that actually stops a downgrade, and it needs nothing
 *      from the application.
 *
 *   2. THE PRE-FLIGHT REFUSAL. The gate runs at the NEXT BOOT, in a bootloader
 *      whose logging is compiled out (mcuboot.conf CONFIG_MCUBOOT_LOG_LEVEL_OFF).
 *      An operator who uploads a downgrade therefore sees "staged, pending",
 *      reboots, and finds the old version still running with no explanation.
 *      sts_rollback_staged_verdict() lets the update path refuse at upload time
 *      instead, with a reason (console/fwupd_glue.c).
 *
 *   3. THE WITNESS. The ATECC608B monotonic counter is stepped up to the
 *      running image's epoch once that image self-confirms. It is EVIDENCE, not
 *      a gate: hardware-one-way, so nothing with write access to flash can
 *      rewind it, and a verifier that sees a witness above the epoch an image
 *      claims knows the unit has run something newer than what it is being told
 *      is running. Deliberately not MCUBOOT_HW_DOWNGRADE_PREVENTION — that
 *      would put I²C and CryptoAuthLib inside the bootloader and let a wedged
 *      bus brick the boot.
 *
 * The epoch is not a version
 * --------------------------
 * CONFIG_STS1000_SECURITY_EPOCH advances only when a release fixes something
 * that must never be reachable again by re-installing an older signed image.
 * app/VERSION moves independently. This is why the counter variant of
 * downgrade prevention is used rather than plain version comparison: MCUboot's
 * version comparison demands a greater major or minor, so 0.1.0 -> 0.1.1 would
 * be refused and no patch release could ship. app/conf/rollback.conf carries
 * the release procedure.
 */

#ifndef STS1000_ZEPHYR_CONSOLE_STS_ROLLBACK_H_
#define STS1000_ZEPHYR_CONSOLE_STS_ROLLBACK_H_

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* MCUboot image layout (bootutil/image.h), restated so this header needs no  */
/* bootloader include. The host suite asserts these against the values quoted */
/* from that file.                                                           */
/* ------------------------------------------------------------------------- */

/** `IMAGE_MAGIC` — first word of a signed image header. */
#define STS_ROLLBACK_IMAGE_MAGIC 0x96f3b83dU
/** `IMAGE_HEADER_SIZE` — the fixed header is 32 octets, little-endian. */
#define STS_ROLLBACK_HDR_LEN 32U
/** `IMAGE_TLV_PROT_INFO_MAGIC` — opens the protected (signed) TLV area. */
#define STS_ROLLBACK_TLV_PROT_MAGIC 0x6908U
/** `IMAGE_TLV_INFO_MAGIC` — opens the unprotected TLV area. */
#define STS_ROLLBACK_TLV_INFO_MAGIC 0x6907U
/** `IMAGE_TLV_SEC_CNT` — the security counter, a little-endian uint32. */
#define STS_ROLLBACK_TLV_SEC_CNT 0x50U
/** Both `struct image_tlv_info` and `struct image_tlv` are 4 octets. */
#define STS_ROLLBACK_TLV_HDR_LEN 4U

/**
 * Ceiling on how much of the protected TLV area is read from flash.
 *
 * imgtool writes at most the info header plus SEC_CNT, a boot record, image
 * dependencies and any custom protected TLVs; for this product it is the info
 * header plus one 4-octet SEC_CNT, i.e. 12 octets. 256 is room for two orders
 * of magnitude of growth and still a stack buffer, and a declared area larger
 * than this is rejected rather than truncated — a truncated walk could miss the
 * SEC_CNT and report "absent", which is a materially different answer.
 */
#define STS_ROLLBACK_PROT_TLV_MAX 256U

/* ------------------------------------------------------------------------- */
/* The witness                                                               */
/* ------------------------------------------------------------------------- */

/**
 * ATECC608B monotonic counter ceiling: 2^21 - 1 (datasheet §4.2). The counter
 * is one-way and cannot be reset, so this is a lifetime budget for the part.
 *
 * Budget arithmetic, which is the whole reason the witness is driven from the
 * epoch and not from anything else:
 *
 *   per security epoch   2 097 151 / 4 per year   = 524 287 years
 *   per confirmed update 2 097 151 / 12 per year  =  174 762 years
 *   per boot             2 097 151 / 365 per year =    5 745 years, but a
 *                        10 s reboot loop is 8 640/day = 243 days to
 *                        exhaustion, permanently, with no way back.
 *
 * The first is what this code does. The third is what sts_atecc.h warns
 * against ("never anything that runs on every boot"), and the reason
 * sts_rollback_witness_steps() returns 0 for the steady state: after the first
 * confirmation at a given epoch the counter already equals it, so every later
 * boot and every later confirmation performs zero increments.
 */
#define STS_ROLLBACK_COUNTER_MAX 2097151U

/**
 * Most increments one confirmation may perform.
 *
 * The normal gap is 0 or 1. A unit that has sat out several security epochs can
 * legitimately need a few. A gap of thousands means a wrong number was signed
 * into an image, and burning thousands of one-way counter steps on it would
 * destroy the witness permanently — so the step count is bounded, the excess is
 * reported, and the remainder is made up by the next confirmed upgrade rather
 * than acted on all at once.
 */
#define STS_ROLLBACK_MAX_STEPS 8U

/* ------------------------------------------------------------------------- */
/* Little-endian accessors                                                   */
/* ------------------------------------------------------------------------- */

/** Read a little-endian uint16 from @p p. */
static inline uint16_t sts_rollback_rd16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/** Read a little-endian uint32 from @p p. */
static inline uint32_t sts_rollback_rd32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
	       ((uint32_t)p[3] << 24);
}

/* ------------------------------------------------------------------------- */
/* Image header                                                              */
/* ------------------------------------------------------------------------- */

/** The fields of `struct image_header` this module needs. */
typedef struct {
	uint16_t hdr_size;      /**< `ih_hdr_size` — payload starts here. */
	uint16_t prot_tlv_size; /**< `ih_protect_tlv_size`, INCLUDING its info
				 *   header; 0 when the image carries no
				 *   protected TLVs at all. */
	uint32_t img_size;      /**< `ih_img_size` — payload length. */
	uint32_t version[4];    /**< major, minor, revision, build. */
} sts_rollback_hdr_t;

/**
 * Decode an MCUboot image header.
 *
 * @param buf  At least STS_ROLLBACK_HDR_LEN octets read from the slot base.
 * @param len  Octets available in @p buf.
 * @param out  Filled on success; untouched otherwise.
 *
 * @retval 0        Decoded.
 * @retval -EINVAL  NULL argument or @p len below the header size.
 * @retval -EILSEQ  Wrong magic (erased slot, unsigned image, or not an image),
 *                  or a header that describes an impossible geometry.
 */
static inline int sts_rollback_hdr_parse(const uint8_t *buf, size_t len,
					 sts_rollback_hdr_t *out)
{
	sts_rollback_hdr_t h;

	if ((buf == NULL) || (out == NULL) || (len < (size_t)STS_ROLLBACK_HDR_LEN)) {
		return -EINVAL;
	}
	if (sts_rollback_rd32(&buf[0]) != STS_ROLLBACK_IMAGE_MAGIC) {
		return -EILSEQ;
	}

	h.hdr_size = sts_rollback_rd16(&buf[8]);
	h.prot_tlv_size = sts_rollback_rd16(&buf[10]);
	h.img_size = sts_rollback_rd32(&buf[12]);
	h.version[0] = buf[20];
	h.version[1] = buf[21];
	h.version[2] = sts_rollback_rd16(&buf[22]);
	h.version[3] = sts_rollback_rd32(&buf[24]);

	/*
	 * The header must at least contain itself, and the protected area — when
	 * present — must have room for its own info header. Both are cheap to
	 * check here and awkward to recover from later: hdr_size + img_size is
	 * used as a flash offset, and a prot_tlv_size of 1..3 would underflow
	 * the walk below.
	 */
	if (h.hdr_size < (uint16_t)STS_ROLLBACK_HDR_LEN) {
		return -EILSEQ;
	}
	if ((h.prot_tlv_size != 0U) &&
	    (h.prot_tlv_size < (uint16_t)STS_ROLLBACK_TLV_HDR_LEN)) {
		return -EILSEQ;
	}

	*out = h;
	return 0;
}

/**
 * Offset of the protected TLV area from the slot base: `BOOT_TLV_OFF(hdr)`.
 *
 * Returned as uint64 because both terms are uint32 in the header and a corrupt
 * image can make them sum past 2^32; the caller compares against the slot size
 * and rejects it, rather than reading from a wrapped offset.
 */
static inline uint64_t sts_rollback_prot_tlv_off(const sts_rollback_hdr_t *h)
{
	return (uint64_t)h->hdr_size + (uint64_t)h->img_size;
}

/* ------------------------------------------------------------------------- */
/* The protected TLV walk                                                    */
/* ------------------------------------------------------------------------- */

/**
 * Find IMAGE_TLV_SEC_CNT in a protected TLV area.
 *
 * @p buf must start at the protected area's own info header — i.e. at
 * sts_rollback_prot_tlv_off() — and hold `prot_tlv_size` octets, which is what
 * that field counts (imgtool adds the info header into it; MCUboot's
 * bootutil_tlv_iter_begin() cross-checks the two and refuses a mismatch).
 *
 * @param buf  The protected area.
 * @param len  Octets in @p buf; must equal the header's `prot_tlv_size`.
 * @param out  The counter, on success.
 *
 * @retval 0        Found; @p out set.
 * @retval -EINVAL  NULL argument.
 * @retval -ENOENT  Well-formed area with no SEC_CNT in it, or @p len is 0
 *                  (an image signed without --security-counter).
 * @retval -EILSEQ  Wrong info magic, a declared size that disagrees with @p len,
 *                  a TLV that runs past the end, or a SEC_CNT that is not four
 *                  octets. Never treated as "absent": a malformed area is a
 *                  different fact from an unsigned-counter image, and only one
 *                  of the two is routine.
 */
static inline int sts_rollback_sec_cnt_find(const uint8_t *buf, size_t len,
					    uint32_t *out)
{
	size_t off;

	if ((out == NULL) || ((buf == NULL) && (len != 0U))) {
		return -EINVAL;
	}
	if (len == 0U) {
		return -ENOENT; /* no protected area: no counter was signed */
	}
	if (len < (size_t)STS_ROLLBACK_TLV_HDR_LEN) {
		return -EILSEQ;
	}
	if (sts_rollback_rd16(&buf[0]) != STS_ROLLBACK_TLV_PROT_MAGIC) {
		return -EILSEQ;
	}
	if ((size_t)sts_rollback_rd16(&buf[2]) != len) {
		return -EILSEQ;
	}

	for (off = (size_t)STS_ROLLBACK_TLV_HDR_LEN;
	     off + (size_t)STS_ROLLBACK_TLV_HDR_LEN <= len;) {
		uint16_t type = sts_rollback_rd16(&buf[off]);
		size_t tlv_len = (size_t)sts_rollback_rd16(&buf[off + 2U]);

		off += (size_t)STS_ROLLBACK_TLV_HDR_LEN;
		if (tlv_len > (len - off)) {
			return -EILSEQ; /* runs past the declared area */
		}
		if (type == (uint16_t)STS_ROLLBACK_TLV_SEC_CNT) {
			if (tlv_len != sizeof(uint32_t)) {
				return -EILSEQ;
			}
			*out = sts_rollback_rd32(&buf[off]);
			return 0;
		}
		off += tlv_len;
	}

	return -ENOENT;
}

/* ------------------------------------------------------------------------- */
/* Decisions                                                                 */
/* ------------------------------------------------------------------------- */

/** What sts_rollback_staged_verdict() concluded about a staged image. */
typedef enum {
	/** Counter >= the running image's: MCUboot will accept the swap. */
	STS_ROLLBACK_STAGED_OK = 0,
	/** Counter < the running image's: MCUboot would erase it at next boot. */
	STS_ROLLBACK_STAGED_OLDER,
	/** No SEC_CNT at all; refused because slot 0 has one. */
	STS_ROLLBACK_STAGED_NO_COUNTER,
	/** The image's protected TLV area does not decode. */
	STS_ROLLBACK_STAGED_MALFORMED,
} sts_rollback_staged_t;

/**
 * Reproduce MCUboot's check_downgrade_prevention() decision for a staged image,
 * before it is marked pending.
 *
 * Modelled from the bootloader source rather than shared with it, so that a
 * change on either side shows up as a disagreement instead of both moving
 * together:
 *
 *     if slot-0 has no counter                 -> allow  (see below)
 *     else if slot-1 has no counter            -> prevent
 *     else if slot0_counter > slot1_counter    -> prevent
 *     else                                        allow
 *
 * @param staged_rc       0, -ENOENT or -EILSEQ, exactly as
 *                        sts_rollback_sec_cnt_find() returned for slot 1.
 * @param staged_counter  The staged counter when @p staged_rc is 0.
 * @param running_counter The running image's counter.
 *
 * Note the asymmetry with the bootloader: this function is only ever reached
 * from an image that HAS a counter, because the running image's counter is the
 * compiled-in epoch, and app/CMakeLists.txt refuses to build an image whose
 * epoch is not also signed in. The "slot 0 has no counter, allow anything"
 * branch therefore has no application-side equivalent — it exists in the
 * bootloader to make the first upgrade from a pre-anti-rollback image work, and
 * that upgrade is staged by the OLD firmware, which has none of this code.
 */
static inline sts_rollback_staged_t
sts_rollback_staged_verdict(int staged_rc, uint32_t staged_counter,
			    uint32_t running_counter)
{
	if (staged_rc == -ENOENT) {
		return STS_ROLLBACK_STAGED_NO_COUNTER;
	}
	if (staged_rc != 0) {
		return STS_ROLLBACK_STAGED_MALFORMED;
	}
	if (staged_counter < running_counter) {
		return STS_ROLLBACK_STAGED_OLDER;
	}
	return STS_ROLLBACK_STAGED_OK;
}

/** Human-readable form of a verdict, for logs and the console. */
static inline const char *sts_rollback_staged_str(sts_rollback_staged_t v)
{
	switch (v) {
	case STS_ROLLBACK_STAGED_OK:
		return "acceptable";
	case STS_ROLLBACK_STAGED_OLDER:
		return "older security epoch";
	case STS_ROLLBACK_STAGED_NO_COUNTER:
		return "signed without a security counter";
	case STS_ROLLBACK_STAGED_MALFORMED:
		return "malformed protected TLV area";
	default:
		return "unknown";
	}
}

/**
 * How many times to increment the witness counter.
 *
 * Steps the counter up to @p epoch, never past it and never backwards, bounded
 * by @p max_steps and by the part's own ceiling.
 *
 * @param epoch      The running image's security epoch (>= 1).
 * @param counter    The witness counter's current value.
 * @param max_steps  Cap for one call; STS_ROLLBACK_MAX_STEPS in production.
 *
 * @return Increments to perform. 0 means the witness is already at or above
 *         the epoch, which is the steady state on every boot after the first
 *         confirmation at a given epoch.
 *
 * A counter ABOVE the epoch is left alone rather than treated as an error: the
 * counters are shared with whatever else a provisioning flow chose to use them
 * for, and a part that arrives from the factory pre-incremented is not a fault.
 * The invariant this function maintains is one-directional — the witness never
 * decreases, and it never exceeds the highest epoch this unit has confirmed
 * unless it started out that way.
 */
static inline uint32_t sts_rollback_witness_steps(uint32_t epoch,
						  uint32_t counter,
						  uint32_t max_steps)
{
	uint32_t want;

	if (epoch > STS_ROLLBACK_COUNTER_MAX) {
		epoch = STS_ROLLBACK_COUNTER_MAX;
	}
	if (counter >= epoch) {
		return 0U;
	}

	want = epoch - counter;
	return (want > max_steps) ? max_steps : want;
}

/* ========================================================================= */
/* Runtime API — implemented in sts_rollback.c against flash_map and the     */
/* ATECC608B. Declared here so everything about anti-rollback is in one file; */
/* tests/host compiles the pure half above and never references these.       */
/* ========================================================================= */

/** Everything the console and the attestation report say about anti-rollback. */
typedef struct {
	/** CONFIG_STS1000_SECURITY_EPOCH: what the firmware believes it is. */
	uint32_t epoch;
	/** IMAGE_TLV_SEC_CNT actually read back from slot 0. */
	uint32_t signed_epoch;
	/** signed_epoch was decoded; false means slot 0 carries no counter. */
	bool signed_epoch_valid;
	/**
	 * signed_epoch == epoch. False is the failure mode that matters most:
	 * the build's imgtool argument and its compiled-in constant disagree, so
	 * the gate MCUboot enforces is not the one the firmware reasons about.
	 */
	bool corroborated;

	/** CONFIG_STS1000_ROLLBACK_WITNESS. */
	bool witness_enabled;
	/** CONFIG_STS1000_ROLLBACK_ATECC_COUNTER. */
	uint8_t witness_index;
	/** The monotonic counter's value at the last read. */
	uint32_t witness_counter;
	/** The counter was readable; false means no secure element. */
	bool witness_valid;
	/** Increments performed since boot. Expected to be 0 or 1, ever. */
	uint32_t witness_steps;
	/** The witness ran to completion for this image. */
	bool witnessed;
} sts_rollback_status_t;

/**
 * Read a slot's signed security counter.
 *
 * @param slot  0 = running, 1 = staged.
 * @param out   The counter, on success.
 *
 * @retval 0        Decoded.
 * @retval -ENOENT  The image carries no security counter, or the slot holds no
 *                  image at all.
 * @retval -EILSEQ  The image or its protected TLV area does not decode.
 * @retval -EINVAL  Bad @p slot or NULL @p out.
 * @retval <0       Whatever the flash layer returned.
 */
int sts_rollback_slot_epoch(uint8_t slot, uint32_t *out);

/**
 * Refuse a staged image the bootloader would silently decline.
 *
 * Called before the staged image is marked pending. MCUboot's own check runs at
 * the next boot with its logging compiled out, so without this an operator sees
 * a successful upload followed by a reboot that changes nothing.
 *
 * @retval 0       The staged image may be marked pending.
 * @retval -EPERM  Anti-rollback refuses it; the reason is logged.
 * @retval -EIO    Slot 1 could not be read.
 */
int sts_rollback_check_staged(void);

/**
 * Step the ATECC608B witness counter up to the running image's epoch.
 *
 * THE ONLY CALLERS ARE THE TWO PATHS THAT CONFIRM AN IMAGE: the §8.3 supervisor
 * gate in sts_selfconfirm.c and the deliberate `sts fw confirm` override. It
 * must never be reachable from anything that runs per boot — the counter is
 * one-way and finite (see STS_ROLLBACK_COUNTER_MAX for the arithmetic).
 *
 * Idempotent four times over: a once-per-boot guard inside; the callers both
 * check boot_is_img_confirmed() first; the counter equals the epoch after the
 * first success so the step count is then 0; and the epoch is corroborated
 * against slot 0's signed TLV before anything is incremented.
 *
 * A missing, unprovisioned or disabled secure element is NOT a failure — every
 * sts_atecc_*() entry point answers -ENODEV there and this returns 0 after
 * logging once (sts_atecc.h). Confirmation must never depend on it.
 *
 * @retval 0       Nothing to do, done, or no secure element.
 * @retval -EIO    The running image's epoch could not be corroborated; no
 *                 counter step was taken.
 */
int sts_rollback_witness(void);

/** Snapshot for `sts sec attest` and the log line. Safe from any thread. */
void sts_rollback_status(sts_rollback_status_t *out);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_CONSOLE_STS_ROLLBACK_H_ */
