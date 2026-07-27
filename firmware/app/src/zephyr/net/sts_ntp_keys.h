/*
 * STS1000 "Meridian" — NTP symmetric-key reconciliation, as pure logic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/net/, and deliberately free of every Zephyr, cfg and
 * logging dependency so tests/host can compile it — the same arrangement as
 * console/sts_confirm_gate.h, for the same reason: what it decides is invisible
 * at runtime when it is wrong.
 *
 * What it decides is which key ids core/ntp holds, and the direction that
 * matters is *revocation*. ntp_key_set() validates its arguments and returns
 * before it locates or clears a slot (core/ntp/ntp.c), so a rejected call
 * leaves any existing entry for that key id completely intact. Rotating a live
 * id to parameters core refuses therefore used to leave the OLD key
 * authenticating while the glue recorded the slot as empty — which also made
 * the id unwithdrawable for the rest of the boot, because withdrawal walks the
 * recorded ids. A rotation prompted by compromise of the old key left it in
 * service until the next reboot, having told the operator "rejected".
 *
 * Two rules make that unreachable:
 *
 *   1. sts_ntp_key_want_set() rejects everything ntp_key_set() rejects,
 *      including RFC 8573's exactly-16-octet rule for AES-CMAC-128, so an
 *      unusable slot reads as *invalid* rather than merely un-installable. The
 *      withdrawal pass then sees its id as no longer wanted and clears it.
 *   2. sts_ntp_keys_reconcile() ends with a sweep that clears the id of every
 *      slot ntp_key_set() still rejected and that no other slot installed.
 *      After rule 1 only -ENOSPC can reach it, and NTP_MAC_KEYS (16) exceeds
 *      STS_NTP_KEY_SLOTS (4) so it cannot — but the post-condition is worth
 *      holding unconditionally rather than by arithmetic coincidence:
 *
 *          after reconcile, core's key table holds exactly the ids
 *          `installed[]` names.
 *
 * Removal is resolved against the whole *new* set rather than slot-by-slot,
 * because two slots may legally name one key id: clearing per slot would let
 * slot 1 delete a key slot 0 had just re-installed.
 */

#ifndef STS1000_ZEPHYR_NET_STS_NTP_KEYS_H_
#define STS1000_ZEPHYR_NET_STS_NTP_KEYS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "ntp/ntp.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Symmetric-key slots in the schema: `sec.ntpkey0..3.*`. */
#define STS_NTP_KEY_SLOTS 4U

/** Why a configured slot cannot be used. */
typedef enum {
	STS_NTP_KEY_OK = 0,
	/** Key id 0: RFC 5905 §7.5 reserves it, so the slot is simply unused. */
	STS_NTP_KEY_UNUSED,
	/** An id with no key material behind it. */
	STS_NTP_KEY_NO_MATERIAL,
	/** More than NTP_MAC_KEY_MAX octets of key material. */
	STS_NTP_KEY_OVERLONG,
	/** The algorithm selector is not one of core/ntp's MAC algorithms. */
	STS_NTP_KEY_BAD_ALG,
	/** AES-CMAC-128 keyed by other than exactly 16 octets (RFC 8573). */
	STS_NTP_KEY_BAD_LEN,
} sts_ntp_key_reject_t;

/** One slot as cfg currently describes it. */
typedef struct {
	uint16_t keyid; /* 0 = the slot is unused */
	uint8_t alg;
	uint8_t key_len;
	uint8_t key[NTP_MAC_KEY_MAX];
	bool valid; /* usable: core/ntp accepts exactly these arguments */
} sts_ntp_key_want_t;

/** What one reconciliation did, for the caller to annunciate. */
typedef struct {
	/** Slots whose key is now installed. */
	unsigned int live;
	/** Key ids revoked, indexed by the slot that used to hold them; 0 = none. */
	uint16_t withdrawn[STS_NTP_KEY_SLOTS];
	/** ntp_key_set() result per slot; 0 also means "not attempted". */
	int set_rc[STS_NTP_KEY_SLOTS];
} sts_ntp_keys_res_t;

/**
 * Validate one slot's raw configured values into a want entry.
 *
 * @p s is always fully written. On any rejection @p s->valid stays false and
 * @p s->keyid still carries the configured id, so the caller can name it in a
 * diagnostic and so reconciliation can withdraw it.
 *
 * @param s        Destination, never NULL.
 * @param keyid    `sec.ntpkeyN.id`.
 * @param alg      `sec.ntpkeyN.alg`, taken wide so an out-of-domain value from
 *                 a future schema is rejected rather than truncated into range.
 * @param key      `sec.ntpkeyN.key` bytes; may be NULL when @p key_len is 0.
 * @param key_len  Octets of key material.
 * @return Why the slot is unusable, or STS_NTP_KEY_OK.
 */
static inline sts_ntp_key_reject_t sts_ntp_key_want_set(sts_ntp_key_want_t *s,
							uint16_t keyid,
							uint64_t alg,
							const uint8_t *key,
							size_t key_len)
{
	memset(s, 0, sizeof(*s));
	if (keyid == 0U) {
		return STS_NTP_KEY_UNUSED;
	}
	s->keyid = keyid;

	if (key == NULL || key_len == 0U) {
		return STS_NTP_KEY_NO_MATERIAL;
	}
	if (key_len > NTP_MAC_KEY_MAX) {
		return STS_NTP_KEY_OVERLONG;
	}
	if (alg > (uint64_t)NTP_MAC_HMAC_SHA256_160) {
		return STS_NTP_KEY_BAD_ALG;
	}
	/*
	 * RFC 8573: AES-CMAC-128 is keyed by exactly one AES-128 key. core/ntp
	 * enforces this too, but from inside ntp_key_set(), which returns before
	 * it touches the table — so the check has to be repeated here or a slot
	 * that fails it reads as "wanted" and its id is never withdrawn.
	 */
	if (alg == (uint64_t)NTP_MAC_AES_CMAC_128 &&
	    key_len != NTP_MAC_AES_KEY_LEN) {
		return STS_NTP_KEY_BAD_LEN;
	}

	s->alg = (uint8_t)alg;
	s->key_len = (uint8_t)key_len;
	memcpy(s->key, key, key_len);
	s->valid = true;
	return STS_NTP_KEY_OK;
}

/**
 * Make core/ntp's key table match @p want, and @p installed match the table.
 *
 * @param ctx        core/ntp context, never NULL.
 * @param want       STS_NTP_KEY_SLOTS entries from sts_ntp_key_want_set().
 * @param installed  STS_NTP_KEY_SLOTS ids this function last installed, updated
 *                   in place. cfg alone cannot say what to *remove*; this is
 *                   the memory that makes revocation possible.
 * @param res        Outcome, for the caller's diagnostics. Never NULL.
 */
static inline void sts_ntp_keys_reconcile(ntp_ctx_t *ctx,
					  const sts_ntp_key_want_t *want,
					  uint16_t *installed,
					  sts_ntp_keys_res_t *res)
{
	size_t i;
	size_t j;

	memset(res, 0, sizeof(*res));

	/* Withdrawals first, so a slot whose id changed cannot leave the old key
	 * behind and a re-used id is not cleared after being re-installed. */
	for (i = 0U; i < STS_NTP_KEY_SLOTS; i++) {
		bool still_wanted = false;

		if (installed[i] == 0U) {
			continue;
		}
		for (j = 0U; j < STS_NTP_KEY_SLOTS; j++) {
			if (want[j].valid && want[j].keyid == installed[i]) {
				still_wanted = true;
				break;
			}
		}
		if (!still_wanted) {
			(void)ntp_key_clear(ctx, installed[i]);
			res->withdrawn[i] = installed[i];
			installed[i] = 0U;
		}
	}

	for (i = 0U; i < STS_NTP_KEY_SLOTS; i++) {
		int rc;

		if (!want[i].valid) {
			installed[i] = 0U;
			continue;
		}
		rc = ntp_key_set(ctx, want[i].keyid, (ntp_mac_alg_t)want[i].alg,
				 want[i].key, want[i].key_len);
		res->set_rc[i] = rc;
		if (rc != 0) {
			installed[i] = 0U;
			continue;
		}
		installed[i] = want[i].keyid;
		res->live++;
	}

	/* Post-condition sweep: an id core rejected must not still be
	 * authenticating from an earlier apply. Skipped when another slot
	 * installed the same id — that one is the operator's live key. */
	for (i = 0U; i < STS_NTP_KEY_SLOTS; i++) {
		bool held = false;

		if (res->set_rc[i] == 0) {
			continue;
		}
		for (j = 0U; j < STS_NTP_KEY_SLOTS; j++) {
			if (installed[j] == want[i].keyid) {
				held = true;
				break;
			}
		}
		if (!held) {
			(void)ntp_key_clear(ctx, want[i].keyid);
		}
	}
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_NET_STS_NTP_KEYS_H_ */
