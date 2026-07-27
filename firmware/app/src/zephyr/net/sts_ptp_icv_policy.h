/*
 * STS1000 "Meridian" — arming IEEE 1588-2019 Annex P integrity, as pure logic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/net/, and deliberately free of every Zephyr, cfg and
 * logging dependency so tests/host can compile it — the same arrangement as
 * net/sts_ntp_keys.h and net/sts_secops_policy.h, for the same reason: what it
 * decides is invisible at runtime when it is wrong.
 *
 * ---------------------------------------------------------------------------
 * What was wrong
 * ---------------------------------------------------------------------------
 *
 * core/ptp/ptp_icv.c is a complete Annex-P implementation and ptp_port.c calls
 * into it on both the transmit and the receive path — but nothing ever created
 * a ptp_icv_ctx_t or handed one to a port. ptp_port_ctx_t::icv was permanently
 * NULL, so every append and every verify was a no-op and the feature could not
 * be switched on at all. `--gc-sections` then discarded ptp_icv_init(),
 * ptp_icv_cfg_defaults(), ptp_icv_key_set() and five more from the image, which
 * is the machine agreeing that the feature did not exist.
 *
 * This header owns the decision the glue was missing: given the operator's
 * `ptp.icv.*` configuration, produce the ptp_icv_cfg_t to load and say whether
 * the port should be holding a context at all.
 *
 * ---------------------------------------------------------------------------
 * Four states, and why each behaves as it does
 * ---------------------------------------------------------------------------
 *
 *   OFF      `ptp.icv.policy` is 0, which is the SHIPPED DEFAULT. No context is
 *            attached; ptp_port_ctx_t::icv stays NULL and the engine behaves
 *            exactly as it did before Annex P existed. A grandmaster that
 *            starts signing to peers that cannot verify is worse than one that
 *            does not, so the feature is opt-in even though core's own
 *            ptp_icv_cfg_defaults() starts at VERIFY_IF_PRESENT — core's
 *            default is what a caller who has already decided to use the
 *            feature should start from, which is a different question.
 *
 *   NO_KEY   A policy is selected but `ptp.icv.key` is empty. The context IS
 *            attached, and the consequences differ by policy:
 *              VERIFY_IF_PRESENT  unsigned messages are accepted; a peer that
 *                                 does send a TLV is rejected as rx_no_key,
 *                                 because "I cannot check this" is not
 *                                 "there was nothing to check";
 *              REQUIRE            every message is rejected (rx_absent_rej) and
 *                                 PTP stops working.
 *            Attaching anyway is the deliberate choice. Detaching would be
 *            quieter, but it would silently downgrade a REQUIRE deployment to
 *            unauthenticated operation, and an unauthenticated grandmaster that
 *            an operator believes is authenticated is the precise failure this
 *            feature exists to prevent. A dead PTP port is loud; a forged
 *            Announce is not. The state is annunciated so the cause is one log
 *            line away.
 *
 *   ARMED    A policy and a usable key. TX signs, RX enforces.
 *
 *   REFUSED  The configuration cannot be turned into a usable association — an
 *            over-long key, an unknown suite, a window out of range, or
 *            anything else core rejects. **Nothing is attached and the produced
 *            cfg is empty**, so pushing it into a live context DISARMS and
 *            zeroizes rather than leaving the previous key in service.
 *
 *            That last point is the same lesson net/sts_ntp_keys.h records: a
 *            rejected key rotation that leaves the old key authenticating is
 *            worse than one that leaves the box unauthenticated, because the
 *            rotation was probably prompted by the old key's compromise and the
 *            operator has been told "rejected".
 *
 * ---------------------------------------------------------------------------
 * Key lifecycle, and the factory reset
 * ---------------------------------------------------------------------------
 *
 * `ptp.icv.key` is CFG_F_SECRET|CFG_F_NOEXPORT, so a factory reset clears it
 * along with every other secret blob and it never appears in an export or a
 * read-back. What that does NOT do by itself is remove the copy inside the live
 * ptp_icv_ctx_t. The glue therefore re-runs this planner from the CFG_G_PTP
 * applier: after a reset the key reads back empty, the plan comes out OFF or
 * NO_KEY with an empty key table, and ptp_icv_set_cfg() overwrites the context's
 * whole ptp_icv_cfg_t — key material included — with it. The reboot is still
 * load-bearing, but the RAM copy does not wait for it.
 *
 * The same path is what lets an operator clear one key without clearing the
 * config tree: emptying `ptp.icv.key` and committing disarms on the next
 * applier run.
 */

#ifndef STS1000_ZEPHYR_NET_STS_PTP_ICV_POLICY_H_
#define STS1000_ZEPHYR_NET_STS_PTP_ICV_POLICY_H_

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "ptp/ptp_icv.h"

#ifdef __cplusplus
extern "C" {
#endif

/** What the glue should do with the port's ICV context. */
typedef enum {
	/** Policy OFF: detach, and hold no key. */
	STS_PTP_ICV_OFF = 0,
	/** Attached with no association: TX unsigned, RX per policy. */
	STS_PTP_ICV_NO_KEY,
	/** Attached and keyed. */
	STS_PTP_ICV_ARMED,
	/** Unusable configuration: detach, and hold no key. */
	STS_PTP_ICV_REFUSED,
	STS_PTP_ICV_STATE_COUNT,
} sts_ptp_icv_state_t;

/** The `ptp.icv.*` keys, exactly as cfg holds them. */
typedef struct {
	uint8_t policy;  /**< ptp_icv_policy_t; 0 = OFF */
	uint8_t suite;   /**< ptp_icv_suite_t */
	uint8_t spp;
	uint32_t key_id;
	uint8_t key[PTP_ICV_KEY_MAX];
	size_t key_len; /**< 0 = no key provisioned */
	bool replay_protect;
	uint8_t replay_window;
	bool mask_mutable;
} sts_ptp_icv_want_t;

/** The planner's verdict. */
typedef struct {
	uint8_t state; /**< sts_ptp_icv_state_t */
	/** Attach the context to the port. False for OFF and REFUSED. */
	bool attach;
	/**
	 * Why the plan is REFUSED: the core error that rejected it
	 * (-EINVAL, -ENOSPC, -ENOENT). 0 for every other state.
	 */
	int rc;
} sts_ptp_icv_plan_t;

/** State name for a log line. Never NULL. */
static inline const char *sts_ptp_icv_state_name(uint8_t s)
{
	switch (s) {
	case (uint8_t)STS_PTP_ICV_OFF:
		return "off";
	case (uint8_t)STS_PTP_ICV_NO_KEY:
		return "armed-no-key";
	case (uint8_t)STS_PTP_ICV_ARMED:
		return "armed";
	case (uint8_t)STS_PTP_ICV_REFUSED:
		return "refused";
	default:
		return "?";
	}
}

/**
 * Turn the configured `ptp.icv.*` keys into a loadable ptp_icv_cfg_t.
 *
 * Always writes @p cfg, including on REFUSED and OFF — in both of those cases
 * it is the *disarmed* configuration (policy OFF, empty key table), which is
 * what makes ptp_icv_set_cfg() a zeroization step rather than a no-op.
 *
 * Pure: no globals, no allocation, no clock. @p w is not modified.
 *
 * @param w    Configured values. NULL is treated as REFUSED(-EINVAL).
 * @param cfg  Receives the configuration to load. Must not be NULL.
 * @param out  Receives the verdict. Must not be NULL.
 */
static inline void sts_ptp_icv_plan(const sts_ptp_icv_want_t *w,
				    ptp_icv_cfg_t *cfg, sts_ptp_icv_plan_t *out)
{
	int rc;

	if ((cfg == NULL) || (out == NULL)) {
		return;
	}

	/* Start disarmed, so every early return below is a zeroizing one. */
	ptp_icv_cfg_defaults(cfg);
	cfg->policy = (uint8_t)PTP_ICV_POLICY_OFF;
	out->state = (uint8_t)STS_PTP_ICV_OFF;
	out->attach = false;
	out->rc = 0;

	if (w == NULL) {
		out->state = (uint8_t)STS_PTP_ICV_REFUSED;
		out->rc = -EINVAL;
		return;
	}
	if (w->policy == (uint8_t)PTP_ICV_POLICY_OFF) {
		return; /* OFF, and the key table is already empty. */
	}
	if ((unsigned int)w->policy >= (unsigned int)PTP_ICV_POLICY_COUNT) {
		out->state = (uint8_t)STS_PTP_ICV_REFUSED;
		out->rc = -EINVAL;
		return;
	}

	cfg->policy = w->policy;
	cfg->tx_spp = w->spp;
	cfg->tx_key_id = w->key_id;
	cfg->replay_protect = w->replay_protect;
	/*
	 * 0 is not "no window" — ptp_icv.h treats it as 1 so a zeroed config
	 * fails closed. Clamp here as well so the value the glue logs is the
	 * value the engine enforces.
	 */
	cfg->replay_window = (w->replay_window == 0U) ? 1U : w->replay_window;
	cfg->mask_mutable = w->mask_mutable;

	if (w->key_len == 0U) {
		/*
		 * Attached but unkeyed. ptp_icv_cfg_validate() permits this
		 * (an empty table is legal and inert), so validate and return
		 * rather than treating it as a refusal.
		 */
		rc = ptp_icv_cfg_validate(cfg);
		if (rc != 0) {
			ptp_icv_cfg_defaults(cfg);
			cfg->policy = (uint8_t)PTP_ICV_POLICY_OFF;
			out->state = (uint8_t)STS_PTP_ICV_REFUSED;
			out->rc = rc;
			return;
		}
		out->state = (uint8_t)STS_PTP_ICV_NO_KEY;
		out->attach = true;
		return;
	}

	/*
	 * ptp_icv_key_set() is the single authority on what a usable
	 * association looks like — key length 1..PTP_ICV_KEY_MAX and a suite
	 * with a defined tag length. Deferring to it, rather than re-deriving
	 * the rules here, is what keeps the glue from installing something core
	 * would silently reinterpret.
	 */
	rc = ptp_icv_key_set(cfg, w->spp, w->key_id, w->suite, w->key,
			     w->key_len);
	if (rc == 0) {
		rc = ptp_icv_cfg_validate(cfg);
	}
	if (rc != 0) {
		/* Refused: disarm and drop the key rather than leaving whatever
		 * was running in service. */
		ptp_icv_cfg_defaults(cfg);
		cfg->policy = (uint8_t)PTP_ICV_POLICY_OFF;
		out->state = (uint8_t)STS_PTP_ICV_REFUSED;
		out->rc = rc;
		return;
	}

	out->state = (uint8_t)STS_PTP_ICV_ARMED;
	out->attach = true;
}

/**
 * True when @p cfg holds no key material at all.
 *
 * The post-condition the disarming paths above must satisfy, exposed so the
 * test can assert it directly instead of inferring it from the state enum.
 */
static inline bool sts_ptp_icv_cfg_is_keyless(const ptp_icv_cfg_t *cfg)
{
	size_t i;

	if (cfg == NULL) {
		return true;
	}
	for (i = 0U; i < (size_t)PTP_ICV_MAX_KEYS; i++) {
		size_t j;

		if (cfg->key[i].in_use) {
			return false;
		}
		if (cfg->key[i].key_len != 0U) {
			return false;
		}
		for (j = 0U; j < (size_t)PTP_ICV_KEY_MAX; j++) {
			if (cfg->key[i].key[j] != 0U) {
				return false;
			}
		}
	}
	return true;
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_NET_STS_PTP_ICV_POLICY_H_ */
