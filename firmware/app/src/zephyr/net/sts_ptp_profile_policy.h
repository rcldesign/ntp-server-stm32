/*
 * STS1000 "Meridian" — which value a PTP profile-owned field takes, as pure
 * logic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/net/, and free of every Zephyr dependency so tests/host
 * can compile it — the same arrangement as sts_ldap_ca.h and
 * sts_secops_policy.h, for the same reason: what it decides is invisible at
 * runtime when it is wrong.
 *
 * ---------------------------------------------------------------------------
 * WHY THIS EXISTS: A PROFILE THAT WAS A LABEL, NOT A PARAMETER SET
 * ---------------------------------------------------------------------------
 *
 * ptp_cfg_apply_profile() (core/ptp/ptp_port.c) exists to set the twelve fields
 * a profile actually dictates — domain, priority1/2, local priority, the three
 * log intervals, announce receipt timeout, transport, l2_mac and not_slave —
 * from the profile descriptor. It had **no caller**. sts_ptp_start() instead
 * read each of those fields from its own cfg key, whose schema defaults are the
 * *Default profile's* values, and then assigned `.profile` last.
 *
 * Two things followed, and the second is worse than the first:
 *
 *   1. Selecting G.8275.1 applied none of G.8275.1. The unit kept domain 0
 *      instead of 24, UDP instead of L2, and 1 Sync/s instead of 16 (log_sync 0
 *      rather than -4) with 0.5 Announce/s instead of 8 (log_announce 1 rather
 *      than -3). Descriptor: core/ptp/ptp_profile.c desc_g8275_1.
 *   2. Because ptp_cfg_validate() enforces each profile's *normative* ranges —
 *      G.8275.1's are normative, unlike Annex I.3's recommendations — that
 *      combination was then REJECTED, and the rejection path called
 *      ptp_cfg_defaults(), which resets `.profile` too. So asking for a telecom
 *      profile silently produced a Default-profile grandmaster.
 *
 * The fix is to apply the profile first and let the operator override it. That
 * needs a rule for "did the operator override this", and cfg has no such query:
 * cfg_get_u64() returns the stored value, and a key nobody ever wrote holds its
 * schema default rather than reading as absent.
 *
 * ---------------------------------------------------------------------------
 * THE RULE, AND WHAT IT COSTS
 * ---------------------------------------------------------------------------
 *
 * A stored value that differs from its schema default is an operator decision
 * and wins. A stored value equal to its schema default is indistinguishable
 * from "never set", so the profile's value wins.
 *
 * The cost, stated rather than hidden: an operator who deliberately sets a
 * profile-owned key BACK to its schema default, under a non-Default profile,
 * gets the profile's value instead of the one they typed. That case is narrow
 * and mostly self-correcting — the schema defaults are the Default profile's
 * values, and for the fields where it would matter most they are outside the
 * selected profile's normative range, so ptp_cfg_validate() would have refused
 * the configuration anyway. (Domain is the clearest example: the schema default
 * is 0, and G.8275.1 permits only 24..43.)
 *
 * The alternative — a per-key "is set" bit in the cfg store — is the honest fix
 * and a schema change, which is a larger and more disruptive edit than the
 * defect warrants. If cfg ever grows that query, delete this header and use it.
 */

#ifndef STS1000_ZEPHYR_NET_STS_PTP_PROFILE_POLICY_H_
#define STS1000_ZEPHYR_NET_STS_PTP_PROFILE_POLICY_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Pick between an operator's stored unsigned value and the profile's.
 *
 * @param stored      what cfg holds for the key right now.
 * @param schema_def  the key's schema default (cfg_key_t::def.u).
 * @param profile_val what ptp_cfg_apply_profile() put in the field.
 *
 * @return @p stored when it differs from @p schema_def, otherwise
 *         @p profile_val.
 */
static inline uint64_t sts_ptp_prof_pick_u64(uint64_t stored,
					     uint64_t schema_def,
					     uint64_t profile_val)
{
	return (stored != schema_def) ? stored : profile_val;
}

/** Signed form of sts_ptp_prof_pick_u64(), for the three log intervals. */
static inline int32_t sts_ptp_prof_pick_i32(int32_t stored, int32_t schema_def,
					    int32_t profile_val)
{
	return (stored != schema_def) ? stored : profile_val;
}

/* ------------------------------------------------- overrides vs the profile */

/**
 * How a profile-owned field got the value it has.
 *
 * Exists so the caller can log the third case, which used to be
 * indistinguishable from the second and cost the operator their whole profile.
 */
typedef enum {
	STS_PTP_PICK_PROFILE = 0, /**< operator never moved it off the default */
	STS_PTP_PICK_OPERATOR,    /**< operator's value, inside the profile */
	STS_PTP_PICK_REFUSED,     /**< operator's value is outside the profile */
} sts_ptp_pick_t;

/**
 * Pick a profile-owned field, refusing an override the profile forbids.
 *
 * ---------------------------------------------------------------------------
 * WHY REFUSING ONE FIELD BEATS REJECTING THE CONFIGURATION
 * ---------------------------------------------------------------------------
 *
 * ptp_cfg_validate() checks every profile-owned field against the descriptor's
 * normative range and returns -ERANGE if any single one is outside it
 * (ptp_port.c:141-157). sts_ptp_start()'s only answer to -ERANGE is
 * ptp_cfg_defaults(), which erases `.profile` — so ONE stale operator key
 * demotes a G.8275.1 grandmaster to the Default profile.
 *
 * That is not a corner case. G.8275.1's ranges are mostly single values:
 * priority1 {128,128}, log_announce {-3,-3}, log_sync {-4,-4}. Any commissioned
 * unit that ever had a log interval or priority1 set — for the Default profile,
 * where those are free — hits it the moment somebody selects the telecom
 * profile. The unit then serves Default while the operator believes it is
 * serving G.8275.1: for a telecom deployment, the wrong domain on the wrong
 * transport at the wrong rate.
 *
 * So an override the profile forbids is dropped, that one field takes the
 * profile's value, and the caller names the field in a log line. A
 * standards-conformant grandmaster plus a message beats a conformant
 * grandmaster of the wrong standard plus a message about "defaults".
 *
 * This cannot mask a genuinely invalid configuration. Every value substituted
 * comes from the profile descriptor, so ptp_cfg_validate() accepts it by
 * construction; anything still rejected afterwards is a field this function
 * does not own — portNumber, the SDO ids, the C37.238 payload — and the
 * fallback for those is unchanged.
 *
 * @param how  Receives which of the three cases applied. May be NULL.
 */
static inline uint64_t sts_ptp_prof_pick_u64_r(uint64_t stored,
					       uint64_t schema_def,
					       uint64_t profile_val, uint64_t lo,
					       uint64_t hi, sts_ptp_pick_t *how)
{
	sts_ptp_pick_t verdict;
	uint64_t out;

	if (stored == schema_def) {
		verdict = STS_PTP_PICK_PROFILE;
		out = profile_val;
	} else if ((stored < lo) || (stored > hi)) {
		verdict = STS_PTP_PICK_REFUSED;
		out = profile_val;
	} else {
		verdict = STS_PTP_PICK_OPERATOR;
		out = stored;
	}

	if (how != NULL) {
		*how = verdict;
	}
	return out;
}

/** Signed form of sts_ptp_prof_pick_u64_r(), for the three log intervals. */
static inline int32_t sts_ptp_prof_pick_i32_r(int32_t stored, int32_t schema_def,
					      int32_t profile_val, int32_t lo,
					      int32_t hi, sts_ptp_pick_t *how)
{
	sts_ptp_pick_t verdict;
	int32_t out;

	if (stored == schema_def) {
		verdict = STS_PTP_PICK_PROFILE;
		out = profile_val;
	} else if ((stored < lo) || (stored > hi)) {
		verdict = STS_PTP_PICK_REFUSED;
		out = profile_val;
	} else {
		verdict = STS_PTP_PICK_OPERATOR;
		out = stored;
	}

	if (how != NULL) {
		*how = verdict;
	}
	return out;
}

/**
 * Transport form: a profile carries a bit MASK, not a range.
 *
 * @param mask  ptp_profile_desc_t::transport_mask, one bit per transport.
 */
static inline uint64_t sts_ptp_prof_pick_xport(uint64_t stored,
					       uint64_t schema_def,
					       uint64_t profile_val,
					       uint32_t mask,
					       sts_ptp_pick_t *how)
{
	sts_ptp_pick_t verdict;
	uint64_t out;

	if (stored == schema_def) {
		verdict = STS_PTP_PICK_PROFILE;
		out = profile_val;
	} else if ((stored >= 32U) ||
		   ((mask & (1UL << (unsigned int)stored)) == 0UL)) {
		/* The width test comes first on purpose: shifting by >= the
		 * width of the type is undefined behaviour, and `stored` is a
		 * configuration value an operator picks. */
		verdict = STS_PTP_PICK_REFUSED;
		out = profile_val;
	} else {
		verdict = STS_PTP_PICK_OPERATOR;
		out = stored;
	}

	if (how != NULL) {
		*how = verdict;
	}
	return out;
}

/** Whether this outcome is worth a log line. */
static inline bool sts_ptp_pick_notable(sts_ptp_pick_t how)
{
	return how == STS_PTP_PICK_REFUSED;
}

/**
 * Whether a stored `ptp.profile` is one the engine knows.
 *
 * Out of range is clamped to Default rather than refused, because a profile id
 * from a newer firmware surviving a downgrade is a configuration this unit must
 * still boot from. @p count is PTP_PROFILE_COUNT, passed in so this header does
 * not have to include the PTP engine's.
 */
static inline uint8_t sts_ptp_prof_clamp(uint64_t stored, uint8_t count,
					 uint8_t dflt)
{
	if ((count == 0U) || (stored >= (uint64_t)count)) {
		return dflt;
	}
	return (uint8_t)stored;
}

/**
 * Whether a validate failure should discard the operator's profile selection.
 *
 * It should not, silently. The caller falls back to a configuration the engine
 * will accept — it has no other option — but the profile the operator asked for
 * is the single most useful thing to name in that log line, and
 * ptp_cfg_defaults() erases it before anyone can. Capture it first; this
 * predicate exists so a test can assert the log path is taken for every
 * non-Default profile and not for Default, where "fell back to Default" would
 * be noise.
 */
static inline bool sts_ptp_prof_report_discard(uint8_t requested, uint8_t dflt)
{
	return requested != dflt;
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_NET_STS_PTP_PROFILE_POLICY_H_ */
