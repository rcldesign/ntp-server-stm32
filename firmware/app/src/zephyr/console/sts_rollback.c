/*
 * STS1000 "Meridian" — anti-rollback: image-epoch readback, staged-image
 * refusal, and the ATECC608B monotonic-counter witness.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * The policy and the byte layouts live in sts_rollback.h, where they are pure
 * and host-tested. This file is the Zephyr side: reach the two slots through
 * flash_map, reach the secure element through sts_atecc.h, own the once-per-boot
 * state.
 *
 * Why the running epoch is read back from flash at all
 * ----------------------------------------------------
 * CONFIG_STS1000_SECURITY_EPOCH is a compiled-in constant, and
 * CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS is a string that has to survive a Kconfig
 * fragment, a CMake variable, `separate_arguments()` and a post-build command
 * before it reaches imgtool. app/CMakeLists.txt checks the two agree at
 * configure time, but that check reasons about the *intent*; it cannot observe
 * whether the argument actually landed in the signed artefact. Reading
 * IMAGE_TLV_SEC_CNT back out of the running slot does observe it, on the device,
 * every time an image is confirmed. If the two disagree the witness is refused
 * outright and the mismatch is logged at error level, because at that point the
 * epoch MCUboot enforces is not the epoch this firmware is reasoning about and
 * neither number can be trusted to describe the other.
 *
 * Cost: two flash reads of 32 and <= 256 octets, on a path that runs at most
 * once per boot.
 */

#include <zephyr/kernel.h>

#ifdef CONFIG_STS1000_CONSOLE

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/util.h>

#include "console/sts_console.h"
#include "console/sts_rollback.h"
#include "storage/sts_atecc.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_rollback, CONFIG_STS1000_LOG_LEVEL);

#define SLOT0_ID FIXED_PARTITION_ID(slot0_partition)
#define SLOT1_ID FIXED_PARTITION_ID(slot1_partition)

BUILD_ASSERT(FIXED_PARTITION_EXISTS(slot0_partition) &&
		     FIXED_PARTITION_EXISTS(slot1_partition),
	     "slot0_partition/slot1_partition are missing from the devicetree");

#if defined(CONFIG_STS1000_SECURITY_EPOCH)
#define SECURITY_EPOCH ((uint32_t)CONFIG_STS1000_SECURITY_EPOCH)
#else
/*
 * app/Kconfig always provides this. The fallback exists so the area still
 * compiles if that menu is dropped, and 1 is the lowest legal epoch — never 0,
 * which would make "no counter signed" and "epoch zero" indistinguishable in a
 * log line.
 */
#define SECURITY_EPOCH 1U
#endif

BUILD_ASSERT(SECURITY_EPOCH >= 1U && SECURITY_EPOCH <= STS_ROLLBACK_COUNTER_MAX,
	     "CONFIG_STS1000_SECURITY_EPOCH must fit the ATECC608B witness counter");

#if defined(CONFIG_STS1000_ROLLBACK_ATECC_COUNTER)
#define WITNESS_INDEX ((uint8_t)CONFIG_STS1000_ROLLBACK_ATECC_COUNTER)
#else
#define WITNESS_INDEX 0U
#endif

#if defined(CONFIG_STS1000_ROLLBACK_WITNESS)
#define WITNESS_ENABLED 1
#else
#define WITNESS_ENABLED 0
#endif

/* ------------------------------------------------------------------------- */
/* State                                                                     */
/* ------------------------------------------------------------------------- */

static K_MUTEX_DEFINE(rb_lock);

static struct {
	bool witnessed;      /* the witness ran to completion this boot */
	bool attempted;      /* ... or ran and gave up; either way, once only */
	bool signed_valid;
	bool corroborated;
	bool counter_valid;
	bool absent_logged;
	uint32_t signed_epoch;
	uint32_t counter;
	uint32_t steps;
} rb;

/* ------------------------------------------------------------------------- */
/* Slot readback                                                             */
/* ------------------------------------------------------------------------- */

int sts_rollback_slot_epoch(uint8_t slot, uint32_t *out)
{
	uint8_t buf[STS_ROLLBACK_PROT_TLV_MAX];
	const struct flash_area *fa = NULL;
	sts_rollback_hdr_t hdr;
	uint64_t tlv_off;
	size_t tlv_len;
	int rc;

	if (out == NULL) {
		return -EINVAL;
	}
	if (slot > 1U) {
		return -EINVAL;
	}

	rc = flash_area_open((slot == 0U) ? SLOT0_ID : SLOT1_ID, &fa);
	if (rc != 0) {
		LOG_ERR("slot%u: flash_area_open failed (%d)", slot, rc);
		return rc;
	}

	rc = flash_area_read(fa, 0, buf, (size_t)STS_ROLLBACK_HDR_LEN);
	if (rc != 0) {
		LOG_ERR("slot%u: header read failed (%d)", slot, rc);
		goto out;
	}

	rc = sts_rollback_hdr_parse(buf, (size_t)STS_ROLLBACK_HDR_LEN, &hdr);
	if (rc != 0) {
		/*
		 * An erased or partially-written slot 1 lands here routinely, so
		 * it is reported as "no image" rather than as an error. Slot 0
		 * landing here means the running image was not produced by
		 * imgtool, which the caller escalates.
		 */
		rc = (rc == -EILSEQ) ? -ENOENT : rc;
		goto out;
	}

	if (hdr.prot_tlv_size == 0U) {
		rc = -ENOENT; /* signed without --security-counter */
		goto out;
	}
	if ((size_t)hdr.prot_tlv_size > sizeof(buf)) {
		LOG_ERR("slot%u: protected TLV area is %u B, over the %zu B "
			"ceiling — refusing to read it partially",
			slot, (unsigned int)hdr.prot_tlv_size, sizeof(buf));
		rc = -EILSEQ;
		goto out;
	}

	tlv_off = sts_rollback_prot_tlv_off(&hdr);
	tlv_len = (size_t)hdr.prot_tlv_size;
	/* fa->fa_size, not flash_area_get_size(): Zephyr 4.2 exposes the size as
	 * a struct field only — there is a getter for the device, not the size,
	 * and sts_dfu.c reads it the same way. */
	if ((tlv_off + (uint64_t)tlv_len) > (uint64_t)fa->fa_size) {
		LOG_ERR("slot%u: protected TLV area at 0x%llx +%zu B runs past "
			"the %zu B slot",
			slot, (unsigned long long)tlv_off, tlv_len,
			(size_t)fa->fa_size);
		rc = -EILSEQ;
		goto out;
	}

	rc = flash_area_read(fa, (off_t)tlv_off, buf, tlv_len);
	if (rc != 0) {
		LOG_ERR("slot%u: protected TLV read failed (%d)", slot, rc);
		goto out;
	}

	rc = sts_rollback_sec_cnt_find(buf, tlv_len, out);

out:
	flash_area_close(fa);
	return rc;
}

/* ------------------------------------------------------------------------- */
/* Staged-image pre-flight                                                   */
/* ------------------------------------------------------------------------- */

int sts_rollback_check_staged(void)
{
	sts_rollback_staged_t verdict;
	uint32_t staged = 0U;
	uint32_t running = 0U;
	int rc;

	/*
	 * Compare against slot 0's SIGNED counter, not the compiled-in
	 * SECURITY_EPOCH.
	 *
	 * MCUboot compares the staged image's IMAGE_TLV_SEC_CNT against the
	 * running image's, and this pre-flight exists solely to reach the same
	 * verdict earlier and with logging. Using the compiled-in constant makes
	 * that true only while the imgtool argument and the Kconfig agree — and
	 * on a unit where the argument silently did not land, the two disagree
	 * and the pre-flight answers a different question from the one MCUboot
	 * will answer at the next boot. The configure-time check in
	 * app/CMakeLists.txt makes that build hard to produce; it does not make
	 * this comparison correct.
	 */
	rc = sts_rollback_slot_epoch(0U, &running);
	if (rc != 0) {
		/*
		 * Refuse rather than fall back to SECURITY_EPOCH. A slot 0 whose
		 * counter cannot be read is exactly the case where the constant
		 * is least trustworthy, so substituting it would be guessing at
		 * the moment guessing is most expensive.
		 */
		LOG_ERR("anti-rollback: slot 0 security counter unreadable "
			"(%d); refusing to mark slot 1 pending", rc);
		return -EIO;
	}

	rc = sts_rollback_slot_epoch(1U, &staged);
	if ((rc != 0) && (rc != -ENOENT) && (rc != -EILSEQ)) {
		/* A flash failure is not a verdict. Say so rather than
		 * inventing one in either direction. */
		LOG_ERR("anti-rollback: slot 1 unreadable (%d); refusing to "
			"mark it pending", rc);
		return -EIO;
	}

	verdict = sts_rollback_staged_verdict(rc, staged, running);
	if (verdict == STS_ROLLBACK_STAGED_OK) {
		LOG_INF("anti-rollback: staged image security epoch %u >= "
			"running %u — acceptable",
			(unsigned int)staged, (unsigned int)SECURITY_EPOCH);
		return 0;
	}

	LOG_ERR("anti-rollback: REFUSING the staged image (%s). Running epoch "
		"%u, staged %s%u. MCUboot would erase it at the next boot with "
		"its logging compiled out, so it is refused here instead.",
		sts_rollback_staged_str(verdict), (unsigned int)running,
		(rc == 0) ? "" : "n/a, rc ", (rc == 0) ? staged : (uint32_t)(-rc));
	sts_log((uint8_t)LOGR_SUB_SYS, (uint8_t)LOGR_CRIT,
		"anti-rollback refused a staged image: %s",
		sts_rollback_staged_str(verdict));
	return -EPERM;
}

/* ------------------------------------------------------------------------- */
/* The witness                                                               */
/* ------------------------------------------------------------------------- */

/**
 * Read the witness counter, tolerating an absent part.
 *
 * @retval 0        @p out holds the counter.
 * @retval -ENODEV  No usable secure element; logged once per boot.
 */
static int witness_read(uint32_t *out)
{
	int rc = sts_atecc_counter_read(WITNESS_INDEX, out);

	if (rc == 0) {
		return 0;
	}
	if (!rb.absent_logged) {
		rb.absent_logged = true;
		/*
		 * sts_atecc.h: -ENODEV means "absent, unprovisioned or disabled
		 * by sec.atecc.en", which is a configuration, not a fault. It
		 * must not block a confirmation, so this is informational.
		 */
		if (rc == -ENODEV) {
			LOG_INF("anti-rollback: no secure element; the epoch-%u "
				"witness is not recorded (MCUboot's own "
				"downgrade gate is unaffected)",
				(unsigned int)SECURITY_EPOCH);
		} else {
			LOG_WRN("anti-rollback: witness counter %u unreadable "
				"(%d)", (unsigned int)WITNESS_INDEX, rc);
		}
	}
	/*
	 * Every failure collapses to -ENODEV on purpose. The caller's only
	 * decision is "record a witness or do not", and there is no failure of
	 * the secure element that should change whether a healthy image stays
	 * confirmed.
	 */
	return -ENODEV;
}

int sts_rollback_witness(void)
{
	uint32_t counter = 0U;
	uint32_t signed_epoch = 0U;
	uint32_t steps;
	uint32_t i;
	int rc;

	k_mutex_lock(&rb_lock, K_FOREVER);

	if (rb.attempted) {
		rc = rb.corroborated ? 0 : -EIO;
		goto out;
	}
	rb.attempted = true;

	/* --- corroborate the epoch against what was actually signed ------- */
	rc = sts_rollback_slot_epoch(0U, &signed_epoch);
	if (rc == 0) {
		rb.signed_valid = true;
		rb.signed_epoch = signed_epoch;
		rb.corroborated = (signed_epoch == SECURITY_EPOCH);
	}

	if (!rb.corroborated) {
		if (rc == -ENOENT) {
			LOG_ERR("anti-rollback: the running image carries NO "
				"security counter, but this firmware was built "
				"for epoch %u. --security-counter did not reach "
				"imgtool; MCUboot is enforcing nothing.",
				(unsigned int)SECURITY_EPOCH);
		} else if (rc != 0) {
			LOG_ERR("anti-rollback: slot 0 security counter "
				"unreadable (%d); refusing to record a witness "
				"for an epoch that cannot be corroborated", rc);
		} else {
			LOG_ERR("anti-rollback: the running image is signed for "
				"epoch %u but this firmware was built for %u. "
				"The gate MCUboot enforces is not the one this "
				"firmware reasons about.",
				(unsigned int)signed_epoch,
				(unsigned int)SECURITY_EPOCH);
		}
		sts_log((uint8_t)LOGR_SUB_SYS, (uint8_t)LOGR_CRIT,
			"security epoch not corroborated (built %u)",
			(unsigned int)SECURITY_EPOCH);
		rc = -EIO;
		goto out;
	}

	if (!WITNESS_ENABLED) {
		LOG_INF("anti-rollback: epoch %u corroborated; witness disabled "
			"by configuration", (unsigned int)SECURITY_EPOCH);
		rb.witnessed = true;
		rc = 0;
		goto out;
	}

	/* --- step the counter up to the epoch ----------------------------- */
	if (witness_read(&counter) != 0) {
		rb.witnessed = true; /* nothing more will be attempted */
		rc = 0;              /* an absent part never fails a confirm */
		goto out;
	}
	rb.counter_valid = true;
	rb.counter = counter;

	steps = sts_rollback_witness_steps(SECURITY_EPOCH, counter,
					   STS_ROLLBACK_MAX_STEPS);
	if ((counter + steps) < SECURITY_EPOCH) {
		LOG_WRN("anti-rollback: witness counter %u is %u below epoch %u; "
			"stepping by %u only. A gap this large means an "
			"implausible epoch was signed, and the steps are "
			"one-way — the remainder is left to the next confirmed "
			"upgrade.",
			(unsigned int)counter,
			(unsigned int)(SECURITY_EPOCH - counter),
			(unsigned int)SECURITY_EPOCH, (unsigned int)steps);
	}

	for (i = 0U; i < steps; i++) {
		uint32_t now = 0U;

		rc = sts_atecc_counter_increment(WITNESS_INDEX, &now);
		if (rc != 0) {
			LOG_WRN("anti-rollback: witness increment %u/%u failed "
				"(%d); counter left at %u",
				(unsigned int)(i + 1U), (unsigned int)steps, rc,
				(unsigned int)rb.counter);
			break;
		}
		rb.counter = now;
		rb.steps++;
	}

	rb.witnessed = true;
	if (rb.steps > 0U) {
		LOG_INF("anti-rollback: witness counter %u advanced to %u for "
			"security epoch %u (%u step(s))",
			(unsigned int)WITNESS_INDEX, (unsigned int)rb.counter,
			(unsigned int)SECURITY_EPOCH, (unsigned int)rb.steps);
		sts_log((uint8_t)LOGR_SUB_SYS, (uint8_t)LOGR_NOTICE,
			"anti-rollback witness at %u for epoch %u",
			(unsigned int)rb.counter, (unsigned int)SECURITY_EPOCH);
	} else {
		LOG_INF("anti-rollback: witness counter %u already at %u for "
			"security epoch %u; no step needed",
			(unsigned int)WITNESS_INDEX, (unsigned int)rb.counter,
			(unsigned int)SECURITY_EPOCH);
	}
	rc = 0;

out:
	k_mutex_unlock(&rb_lock);
	return rc;
}

void sts_rollback_status(sts_rollback_status_t *out)
{
	if (out == NULL) {
		return;
	}

	memset(out, 0, sizeof(*out));
	out->epoch = SECURITY_EPOCH;
	out->witness_enabled = (WITNESS_ENABLED != 0);
	out->witness_index = WITNESS_INDEX;

	k_mutex_lock(&rb_lock, K_FOREVER);
	out->signed_epoch = rb.signed_epoch;
	out->signed_epoch_valid = rb.signed_valid;
	out->corroborated = rb.corroborated;
	out->witness_counter = rb.counter;
	out->witness_valid = rb.counter_valid;
	out->witness_steps = rb.steps;
	out->witnessed = rb.witnessed;
	k_mutex_unlock(&rb_lock);

	/*
	 * Before the first confirmation nothing above has been sampled, so fill
	 * the two readable facts on demand. Deliberately NOT the increment path:
	 * this runs from the console whenever an operator asks.
	 */
	if (!out->witnessed) {
		uint32_t v = 0U;

		if (sts_rollback_slot_epoch(0U, &v) == 0) {
			out->signed_epoch = v;
			out->signed_epoch_valid = true;
			out->corroborated = (v == SECURITY_EPOCH);
		}
		if (out->witness_enabled &&
		    (sts_atecc_counter_read(WITNESS_INDEX, &v) == 0)) {
			out->witness_counter = v;
			out->witness_valid = true;
		}
	}
}

#endif /* CONFIG_STS1000_CONSOLE */
