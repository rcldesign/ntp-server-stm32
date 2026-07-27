/*
 * STS1000 "Meridian" — core/ptp: PTP profile descriptors and profile TLVs.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11, no allocation, no globals holding state
 * (ARCHITECTURE.md §4). Behaviour comes from ntp_server_software_spec.md §4.4:
 * "Profiles: Default (1588), and configurable Telecom (G.8275.1/.2) and Power
 * (C37.238) profiles — domain, priority1/2, log intervals, transport (L2/UDP)".
 *
 * ---------------------------------------------------------------------------
 * What a "profile" is, here
 * ---------------------------------------------------------------------------
 *
 * A PTP profile is a named set of *restrictions and defaults* on an otherwise
 * unchanged protocol engine, plus (sometimes) a small number of genuine
 * behavioural deltas. This module is the parameterisation: one descriptor per
 * profile carrying
 *
 *   - domainNumber default and permitted range,
 *   - priority1/priority2 defaults and permitted values,
 *   - localPriority default and permitted range (telecom profiles),
 *   - logAnnounce/logSync/logMinDelayReq defaults and permitted ranges,
 *   - which transports the profile allows and which it defaults to,
 *   - whether the profile's BMCA uses priority1 and/or localPriority,
 *   - the clockClass ladder the profile assigns to our sync states,
 *   - whether Announce carries a mandatory organization TLV.
 *
 * The engine in ptp_port.c reads the descriptor; it does not branch on the
 * profile enum. Adding a profile is a table entry plus its clockClass ladder.
 *
 * ---------------------------------------------------------------------------
 * Scope limits that a reader must not mistake for conformance
 * ---------------------------------------------------------------------------
 *
 * 1. **C37.238 mandates the peer-delay mechanism** and a network of transparent
 *    clocks. This engine implements E2E only (ptp.h scope note 4). Selecting
 *    PTP_PROFILE_POWER_C37_238 therefore configures the profile's domain,
 *    intervals, transport and Announce TLV, and leaves the delay mechanism at
 *    E2E. That is a *documented deviation*, not conformance, and it is reported:
 *    the descriptor's @ref ptp_profile_desc_t::deviations field names it and
 *    ptp_profile_deviation_text() renders it for the operator.
 *
 * 2. **G.8275.2 is a unicast profile** and unicast message negotiation
 *    (Signaling REQUEST/GRANT_UNICAST_TRANSMISSION, §16.1) is not implemented.
 *    The descriptor marks the profile unicast so the glue opens unicast sockets
 *    and the engine answers unicast Delay_Req, but a peer that expects to
 *    negotiate its Announce/Sync grants will not be served. Also reported as a
 *    deviation.
 *
 * 3. **The alternate BMCA's localPriority is implemented; notSlave is
 *    structural.** G.8275.1 §6.3 replaces priority1 with localPriority as a
 *    tiebreak and adds a notSlave attribute. localPriority is implemented in
 *    the dataset comparison (see ptp_bmca_compare_profile()). notSlave needs no
 *    code: this appliance never enters SLAVE at all (ptp.h scope note 2), which
 *    is strictly stronger than notSlave.
 *
 * ---------------------------------------------------------------------------
 * VERIFY markers
 * ---------------------------------------------------------------------------
 *
 * Constants this project cannot check against a primary source carry a
 * "VERIFY vs <source>" comment, the same convention core/ubx/ubx.h uses for the
 * un-published CFG key IDs. Everything so marked is either (a) a default that
 * an operator can override through cfg, or (b) gated behind an explicit
 * "verified" configuration bit. Nothing marked VERIFY can put the timing path
 * or a peripheral at risk on its own.
 */

#ifndef STS1000_CORE_PTP_PTP_PROFILE_H_
#define STS1000_CORE_PTP_PTP_PROFILE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ptp/ptp_msg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================== *
 *  Generic TLV suffix utilities
 *
 *  A PTP message is fixed fields followed by a TLV suffix (§13.4). Both the
 *  profile TLVs below and the Annex-P AUTHENTICATION TLV in ptp_icv.h need to
 *  append to and walk that suffix, so the mechanics live here rather than being
 *  written twice.
 * ===================================================================== */

/** tlvType + lengthField, §14.1. */
#define PTP_TLV_HDR_LEN 4U

/** tlvType values used by this firmware (§14.1.1, Table 52). */
#define PTP_TLV_TYPE_ORGANIZATION_EXTENSION 0x0003U
/**
 * AUTHENTICATION, IEEE 1588-2019 §16.14.
 *
 * VERIFY vs IEEE 1588-2019 Table 52 — this project has no copy of the 2019
 * revision's tlvType table. 0x8000 is the value the 2019 revision is understood
 * to have assigned; IEEE 1588-2008 used 0x2000 for its (different, withdrawn)
 * AUTHENTICATION TLV. Both are selectable at runtime through
 * ptp_icv_cfg_t::tlv_type precisely because this one is unverified, and a wrong
 * value is inert: it makes our TLV unrecognised by a peer, and a peer's TLV
 * unrecognised by us, which the ICV policy then handles as "absent".
 */
#define PTP_TLV_TYPE_AUTHENTICATION 0x8000U
/** The IEEE 1588-2008 AUTHENTICATION tlvType, offered as an alternative. */
#define PTP_TLV_TYPE_AUTHENTICATION_2008 0x2000U

/**
 * Append a TLV to an encoded message and update its messageLength.
 *
 * @param buf        Encoded message. Must already hold @p *len valid octets and
 *                   a decodable common header (only octets 2..3 are rewritten).
 * @param cap        Capacity of @p buf.
 * @param len        In: current message length. Out: length including the TLV.
 * @param type       tlvType.
 * @param value      TLV value, or NULL when @p value_len is 0.
 * @param value_len  Octets of value. §14.1 requires an even length; an odd
 *                   length is rejected rather than silently padded, because a
 *                   pad octet would change the ICV coverage of a following
 *                   AUTHENTICATION TLV.
 * @param val_off    Receives the offset of the value within @p buf. May be NULL.
 *
 * @retval 0        Appended.
 * @retval -EINVAL  NULL argument, odd @p value_len, or @p *len below PTP_HDR_LEN.
 * @retval -ENOSPC  @p cap cannot hold the TLV.
 * @retval -EOVERFLOW The resulting messageLength exceeds 0xFFFF.
 */
int ptp_tlv_append(uint8_t *buf, size_t cap, size_t *len, uint16_t type,
		   const uint8_t *value, size_t value_len, size_t *val_off);

/** Cursor over the TLV suffix of a received message. */
typedef struct {
	const uint8_t *buf;
	size_t len;   /**< usable extent: min(len, messageLength) */
	size_t pos;   /**< offset of the next TLV header */
} ptp_tlv_iter_t;

/**
 * Start walking the TLV suffix of @p buf.
 *
 * The suffix begins at ptp_msg_min_len(messageType) — the end of the fixed
 * fields — and ends at messageLength. Trailing octets beyond messageLength (an
 * 802.3 pad) are excluded, so a pad can never be mistaken for a TLV.
 *
 * @retval 0        Iterator ready (it may immediately be exhausted).
 * @retval -EINVAL  @p it or @p buf is NULL.
 * @retval -EBADMSG @p len is below the header, or messageLength is inconsistent.
 */
int ptp_tlv_iter_begin(ptp_tlv_iter_t *it, const uint8_t *buf, size_t len);

/**
 * Produce the next TLV.
 *
 * @param type     Receives tlvType. May be NULL.
 * @param value    Receives a pointer into @p it->buf. May be NULL.
 * @param val_len  Receives the value length. May be NULL.
 * @param tlv_off  Receives the offset of the TLV header. May be NULL.
 *
 * @retval 0        A TLV was produced.
 * @retval -ENOENT  The suffix is exhausted.
 * @retval -EBADMSG A truncated TLV header or a lengthField past the end.
 * @retval -EINVAL  @p it is NULL.
 */
int ptp_tlv_iter_next(ptp_tlv_iter_t *it, uint16_t *type, const uint8_t **value,
		      size_t *val_len, size_t *tlv_off);

/* ===================================================================== *
 *  clockClass ladders
 * ===================================================================== */

/**
 * Sentinel meaning "fall back to ptp_cfg_t::degradation" (class 52 or 187).
 *
 * clockClass 0 is reserved by §7.6.2.5, so it can never be a real answer and is
 * safe as an in-band sentinel. The Default profile uses it for its two degraded
 * rungs, which is what keeps Default-profile behaviour bit-identical to the
 * pre-profile engine.
 */
#define PTP_CLASS_USE_DEGRADATION 0U

/**
 * IEEE 1588-2019 Table 4 "default" clockClass: not traceable to any reference.
 *
 * The honest answer for a clock that has never been disciplined since boot. See
 * the never-locked gate in ptp_port.c.
 */
#define PTP_CLASS_DEFAULT_NOT_TRACEABLE 248U

/**
 * Which frequency-source category is behind a holdover interval.
 *
 * G.8275.1 grades holdover by the quality of the frequency source the clock is
 * flywheeling on, and gives each grade its own clockClass. This appliance can
 * tell the three apart from the quality view's timeSource:
 *
 *   CAT1  an atomic standard — the FE-5680A rubidium on the mux (PRC-class).
 *   CAT2  the onboard OH300 VC-OCXO, whose holdover is characterised (§3.6).
 *   CAT3  the external house 10 MHz, real but of provenance unknown to us.
 */
typedef enum {
	PTP_FREQ_CAT1 = 0,
	PTP_FREQ_CAT2,
	PTP_FREQ_CAT3,
	PTP_FREQ_CAT_COUNT,
} ptp_freq_cat_t;

/** clockClass to advertise for each (sync state, frequency category) pair. */
typedef struct {
	uint8_t locked;               /**< disciplined and inside the lock window */
	uint8_t holdover_cat1;        /**< in-spec holdover on an atomic reference */
	uint8_t holdover_cat2;        /**< in-spec holdover on the OCXO */
	uint8_t holdover_cat3;        /**< in-spec holdover on the house reference */
	uint8_t holdover_out_of_spec; /**< holdover past the characterised window */
	uint8_t freerun;              /**< never disciplined, or discipline lost */
} ptp_class_ladder_t;

/* ===================================================================== *
 *  Profile descriptor
 * ===================================================================== */

/** Inclusive uint8 range. min > max marks the field as "not used". */
typedef struct {
	uint8_t min;
	uint8_t max;
} ptp_range_u8_t;

/** Inclusive int8 range. */
typedef struct {
	int8_t min;
	int8_t max;
} ptp_range_i8_t;

/** Documented deviations from the profile as published. */
#define PTP_DEV_NONE            0x0000U
/** The profile mandates the peer-delay mechanism; this engine is E2E only. */
#define PTP_DEV_E2E_ONLY        0x0001U
/** The profile is unicast; unicast message negotiation is not implemented. */
#define PTP_DEV_NO_UNICAST_NEG  0x0002U
/** The profile expects one-step Sync; this engine is two-step (ptp.h note 3). */
#define PTP_DEV_TWO_STEP_ONLY   0x0004U
/** The profile defines slave/boundary behaviour this grandmaster never enters. */
#define PTP_DEV_GM_ONLY         0x0008U

/**
 * Transport encapsulation selector; the glue owns the framing.
 *
 * Declared here rather than in ptp.h because a profile descriptor has to name
 * the transports its profile permits, and ptp.h includes this header.
 */
typedef enum {
	PTP_TRANSPORT_UDP_IPV4 = 0, /* IEEE 1588-2019 Annex C */
	PTP_TRANSPORT_UDP_IPV6,     /* Annex D */
	PTP_TRANSPORT_L2,           /* Annex E, EtherType 0x88F7 */
	PTP_TRANSPORT_COUNT,
} ptp_transport_t;

/** Which multicast MAC an L2 profile uses (IEEE 1588-2019 Annex E, Table E.1). */
typedef enum {
	/** 01-1B-19-00-00-00 — forwarded by ordinary bridges. */
	PTP_L2_MAC_FORWARDABLE = 0,
	/** 01-80-C2-00-00-0E — reserved, not forwarded (G.8275.1 default). */
	PTP_L2_MAC_NON_FORWARDABLE,
	PTP_L2_MAC_COUNT,
} ptp_l2_mac_t;

/** Bit per ptp_transport_t, for ptp_profile_desc_t::transport_mask. */
#define PTP_XPORT_BIT(t) ((uint16_t)1U << (unsigned int)(t))

/**
 * Everything the engine needs to know about a profile.
 *
 * Descriptors are static const; ptp_profile_desc() hands out a pointer.
 */
typedef struct {
	const char *name;      /**< "Default", "G.8275.1", … — never NULL */
	const char *reference; /**< the document the numbers come from */

	uint8_t domain_default;
	ptp_range_u8_t domain_range;

	uint8_t priority1_default;
	ptp_range_u8_t priority1_range;
	uint8_t priority2_default;
	ptp_range_u8_t priority2_range;
	uint8_t local_priority_default;
	ptp_range_u8_t local_priority_range;

	int8_t log_announce_default;
	ptp_range_i8_t log_announce_range;
	int8_t log_sync_default;
	ptp_range_i8_t log_sync_range;
	int8_t log_min_delay_req_default;
	ptp_range_i8_t log_min_delay_req_range;

	uint8_t announce_receipt_timeout_default;
	ptp_range_u8_t announce_receipt_timeout_range;

	uint8_t transport_default; /**< ptp_transport_t */
	uint16_t transport_mask;   /**< PTP_XPORT_BIT() union of what is allowed */
	uint8_t l2_mac_default;    /**< ptp_l2_mac_t; only read for PTP_TRANSPORT_L2 */

	bool unicast;              /**< the profile's normal delivery is unicast */
	bool use_priority1;        /**< false: the profile's BMCA ignores priority1 */
	bool use_local_priority;   /**< true: localPriority is a BMCA tiebreak */
	bool not_slave_default;    /**< the profile's notSlave default */
	bool announce_org_tlv;     /**< Announce carries a mandatory org TLV */

	const ptp_class_ladder_t *ladder;
	uint16_t deviations;       /**< PTP_DEV_* */
} ptp_profile_desc_t;

/**
 * Descriptor for @p profile.
 *
 * Never NULL: an out-of-range selector returns the Default-profile descriptor,
 * so a corrupted configuration degrades to the profile that always works
 * rather than to a NULL dereference.
 */
const ptp_profile_desc_t *ptp_profile_desc(uint8_t profile);

/** Profile name, e.g. "G.8275.1". Never NULL. */
const char *ptp_profile_name(uint8_t profile);

/**
 * Human-readable list of @p profile's documented deviations.
 *
 * Returns "none" when the profile is implemented as published. The string is
 * static storage owned by this module; it is stable for the life of the program.
 */
const char *ptp_profile_deviation_text(uint8_t profile);

/**
 * Map a sync state and frequency category onto a clockClass.
 *
 * @param ladder     The profile's ladder.
 * @param locked     True when the clock is disciplined and inside the window.
 * @param holdover   True when in holdover.
 * @param exceeded   True when holdover has passed its characterised window.
 * @param cat        Frequency-source category behind the holdover.
 *
 * @return The clockClass, or PTP_CLASS_USE_DEGRADATION when the caller must
 *         fall back to its configured degradation alternative.
 */
uint8_t ptp_class_from_ladder(const ptp_class_ladder_t *ladder, bool locked,
			      bool holdover, bool exceeded, ptp_freq_cat_t cat);

/**
 * Classify a timeSource (§7.6.2.8) into a frequency-source category.
 *
 * ATOMIC_CLOCK -> CAT1. GNSS and INTERNAL_OSC -> CAT2: while GNSS is the *time*
 * source the flywheel underneath it is the OCXO, and that is what a holdover
 * class describes. Everything else, the house 10 MHz included, -> CAT3.
 */
ptp_freq_cat_t ptp_freq_cat_from_time_source(uint8_t time_source);

/* ===================================================================== *
 *  IEEE C37.238 organization extension TLV
 * ===================================================================== */

/**
 * organizationId / organizationSubType of the two published C37.238 TLVs.
 *
 * VERIFY vs IEEE C37.238-2011 §5.11 and IEEE C37.238-2017 §5.13 — this project
 * has no copy of either standard. The values below are the ones interoperating
 * implementations use. They are inert if wrong: a peer simply does not
 * recognise the TLV, and this clock does not act on a received one. Both
 * variants are selectable through ptp_c37238_t::v2017, and the whole TLV is
 * off unless the Power profile is selected.
 */
#define PTP_C37238_ORG_ID_2011_0 0x00U /* VERIFY vs IEEE C37.238-2011 §5.11 */
#define PTP_C37238_ORG_ID_2011_1 0x1BU /* VERIFY vs IEEE C37.238-2011 §5.11 */
#define PTP_C37238_ORG_ID_2011_2 0x19U /* VERIFY vs IEEE C37.238-2011 §5.11 */
#define PTP_C37238_SUBTYPE_2011  0x000001UL /* VERIFY vs IEEE C37.238-2011 §5.11 */

#define PTP_C37238_ORG_ID_2017_0 0x1CU /* VERIFY vs IEEE C37.238-2017 §5.13 */
#define PTP_C37238_ORG_ID_2017_1 0x12U /* VERIFY vs IEEE C37.238-2017 §5.13 */
#define PTP_C37238_ORG_ID_2017_2 0x9DU /* VERIFY vs IEEE C37.238-2017 §5.13 */
#define PTP_C37238_SUBTYPE_2017  0x000002UL /* VERIFY vs IEEE C37.238-2017 §5.13 */

/**
 * Octets in the TLV *value* of either variant.
 *
 * organizationId 3 + organizationSubType 3 + two 16/32-bit inaccuracy fields
 * + reserved, laid out below. §14.1 requires an even value length and this is
 * 18, so no padding is involved.
 */
#define PTP_C37238_VALUE_LEN 18U
/** Total wire octets: header + value. */
#define PTP_C37238_TLV_LEN (PTP_TLV_HDR_LEN + PTP_C37238_VALUE_LEN)

/**
 * The C37.238 Announce payload.
 *
 * 2011 layout (value octets, after organizationId and organizationSubType):
 *   grandmasterID              u16   an operator-assigned 0..255-ish tag
 *   grandmasterTimeInaccuracy  u32   ns, this grandmaster's own inaccuracy
 *   networkTimeInaccuracy      u32   ns, accumulated network inaccuracy
 *   reserved                   2 octets, zero
 *
 * 2017 layout replaces the first two with reserved fields and carries a single
 * totalTimeInaccuracy in the third position. @ref total_inaccuracy_ns is used
 * for that, so one struct serves both.
 */
typedef struct {
	uint16_t grandmaster_id;
	uint32_t gm_inaccuracy_ns;
	uint32_t network_inaccuracy_ns;
	uint32_t total_inaccuracy_ns; /**< 2017 only */
	bool v2017;
} ptp_c37238_t;

/** Defaults: grandmasterID 0, inaccuracies "unknown" (0). */
void ptp_c37238_defaults(ptp_c37238_t *p);

/**
 * Encode the TLV value of @p p into @p buf (PTP_C37238_VALUE_LEN octets).
 *
 * @retval 0        Encoded.
 * @retval -EINVAL  @p buf or @p p is NULL.
 * @retval -ENOSPC  @p cap is below PTP_C37238_VALUE_LEN.
 */
int ptp_c37238_value_encode(uint8_t *buf, size_t cap, const ptp_c37238_t *p);

/**
 * Decode a C37.238 organization-extension TLV value.
 *
 * @retval 0        Decoded; @p out->v2017 reports which variant it was.
 * @retval -EINVAL  @p buf or @p out is NULL.
 * @retval -EBADMSG @p len is not PTP_C37238_VALUE_LEN.
 * @retval -EPROTO  organizationId / organizationSubType is neither variant.
 */
int ptp_c37238_value_decode(const uint8_t *buf, size_t len, ptp_c37238_t *out);

/**
 * Append the C37.238 TLV to an encoded Announce.
 *
 * @retval 0        Appended and messageLength updated.
 * @retval -EINVAL  NULL argument, or @p *len is not a plausible message.
 * @retval -ENOSPC  @p cap cannot hold the TLV.
 */
int ptp_c37238_append(uint8_t *buf, size_t cap, size_t *len,
		      const ptp_c37238_t *p);

/**
 * Find and decode the C37.238 TLV of a received Announce.
 *
 * @retval 0        Found and decoded.
 * @retval -ENOENT  No C37.238 organization-extension TLV present.
 * @retval -EINVAL  @p buf or @p out is NULL.
 * @retval -EBADMSG A malformed TLV suffix.
 * @retval -EPROTO  A matching organizationId with an unusable body.
 */
int ptp_c37238_find(const uint8_t *buf, size_t len, ptp_c37238_t *out);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_PTP_PTP_PROFILE_H_ */
