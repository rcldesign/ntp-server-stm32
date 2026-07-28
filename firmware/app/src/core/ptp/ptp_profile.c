/*
 * STS1000 "Meridian" — core/ptp: profile descriptor table, TLV suffix
 * mechanics, and the IEEE C37.238 organization extension TLV.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See ptp_profile.h for the design and for the scope limits that must not be
 * mistaken for conformance.
 */

#include "ptp/ptp_profile.h"

#include <errno.h>
#include <string.h>

#include "util/bytes.h"

/* ===================================================================== *
 *  TLV suffix mechanics
 * ===================================================================== */

/* messageLength occupies octets 2..3 of the common header (§13.3.2.4). */
#define MSG_LENGTH_OFF 2U

/*
 * organizationSubType is a 24-bit big-endian field (§14.3.2) and util/bytes.h
 * has no 24-bit accessor — it is a PTP oddity, not a general one, so the two
 * helpers live here rather than widening the shared header.
 */
static void put_be24(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)((v >> 16) & 0xFFU);
	p[1] = (uint8_t)((v >> 8) & 0xFFU);
	p[2] = (uint8_t)(v & 0xFFU);
}

static uint32_t get_be24(const uint8_t *p)
{
	return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | (uint32_t)p[2];
}

int ptp_tlv_append(uint8_t *buf, size_t cap, size_t *len, uint16_t type,
		   const uint8_t *value, size_t value_len, size_t *val_off)
{
	size_t base;
	size_t total;

	if ((buf == NULL) || (len == NULL)) {
		return -EINVAL;
	}
	if ((value == NULL) && (value_len != 0U)) {
		return -EINVAL;
	}
	/*
	 * §14.1: "lengthField shall be an even number". An odd value would have
	 * to be padded, and a pad octet inserted between two TLVs changes the
	 * byte range an AUTHENTICATION ICV covers — so this refuses rather than
	 * pads, and every caller here builds an even-length value.
	 */
	if ((value_len % 2U) != 0U) {
		return -EINVAL;
	}
	base = *len;
	if (base < (size_t)PTP_HDR_LEN) {
		return -EINVAL;
	}

	total = base + (size_t)PTP_TLV_HDR_LEN + value_len;
	if (total > cap) {
		return -ENOSPC;
	}
	if (total > 0xFFFFU) {
		return -EOVERFLOW;
	}

	bytes_put_be16(&buf[base], type);
	bytes_put_be16(&buf[base + 2U], (uint16_t)value_len);
	if (value_len != 0U) {
		(void)memcpy(&buf[base + PTP_TLV_HDR_LEN], value, value_len);
	}

	/*
	 * messageLength must count the TLV. A verifier recomputes the ICV over
	 * the message as received, so if this were left stale the two sides
	 * would hash different bytes and every ICV would fail.
	 */
	bytes_put_be16(&buf[MSG_LENGTH_OFF], (uint16_t)total);

	if (val_off != NULL) {
		*val_off = base + (size_t)PTP_TLV_HDR_LEN;
	}
	*len = total;
	return 0;
}

int ptp_tlv_iter_begin(ptp_tlv_iter_t *it, const uint8_t *buf, size_t len)
{
	uint16_t msg_len;
	size_t fixed;

	if ((it == NULL) || (buf == NULL)) {
		return -EINVAL;
	}
	(void)memset(it, 0, sizeof(*it));

	if (len < (size_t)PTP_HDR_LEN) {
		return -EBADMSG;
	}

	msg_len = bytes_get_be16(&buf[MSG_LENGTH_OFF]);
	fixed = ptp_msg_min_len(buf[0] & 0x0FU);

	if (((size_t)msg_len > len) || ((size_t)msg_len < fixed)) {
		return -EBADMSG;
	}

	it->buf = buf;
	it->len = (size_t)msg_len;
	it->pos = fixed;
	return 0;
}

int ptp_tlv_iter_next(ptp_tlv_iter_t *it, uint16_t *type, const uint8_t **value,
		      size_t *val_len, size_t *tlv_off)
{
	uint16_t t;
	uint16_t vl;

	if (it == NULL) {
		return -EINVAL;
	}
	if (it->buf == NULL) {
		return -ENOENT;
	}
	if (it->pos >= it->len) {
		return -ENOENT;
	}
	if ((it->len - it->pos) < (size_t)PTP_TLV_HDR_LEN) {
		/* Trailing octets that are too short to be a TLV header. */
		return -EBADMSG;
	}

	t = bytes_get_be16(&it->buf[it->pos]);
	vl = bytes_get_be16(&it->buf[it->pos + 2U]);

	if ((size_t)vl > (it->len - it->pos - (size_t)PTP_TLV_HDR_LEN)) {
		return -EBADMSG;
	}

	if (type != NULL) {
		*type = t;
	}
	if (value != NULL) {
		*value = &it->buf[it->pos + PTP_TLV_HDR_LEN];
	}
	if (val_len != NULL) {
		*val_len = (size_t)vl;
	}
	if (tlv_off != NULL) {
		*tlv_off = it->pos;
	}

	it->pos += (size_t)PTP_TLV_HDR_LEN + (size_t)vl;
	return 0;
}

/* ===================================================================== *
 *  clockClass ladders
 * ===================================================================== */

/*
 * Default profile (IEEE 1588-2019 Table 4).
 *
 * Reproduces the pre-profile engine exactly: locked -> 6, any in-spec holdover
 * -> 7 regardless of what is flywheeling, and both degraded rungs deferring to
 * ptp_cfg_t::degradation (52 for alternative A, 187 for B). The three holdover
 * categories collapsing onto one value is not an oversight — Table 4 has no
 * per-category holdover codes; grading holdover by source is a telecom-profile
 * idea.
 */
static const ptp_class_ladder_t ladder_default = {
	.locked = 6U,
	.holdover_cat1 = 7U,
	.holdover_cat2 = 7U,
	.holdover_cat3 = 7U,
	.holdover_out_of_spec = PTP_CLASS_USE_DEGRADATION,
	.freerun = PTP_CLASS_USE_DEGRADATION,
};

/*
 * Telecom profiles (ITU-T G.8275.1 Table 1 / G.8275.2, "clockClass values").
 *
 * 6    T-GM locked to a PRTC (phase and frequency traceable to a primary
 *      reference; for this appliance, GNSS-disciplined and inside the window).
 * 7    T-GM in holdover, within the holdover specification, traceable to a
 *      Category 1 frequency source — the rubidium.
 * 140  ditto, Category 2 — the characterised OCXO.
 * 150  ditto, Category 3 — the external house standard, provenance unknown.
 * 160  T-GM in holdover, out of holdover specification.
 * 248  default: not traceable, i.e. free-run.
 *
 * Note what these do *not* do: 140/150/160/248 are all above the 127 boundary
 * of §7.6.2.5's "shall never be a slave" band, so a telecom-profile unit that
 * degrades past Category 1 becomes eligible for the S1 branch of the state
 * decision — which this appliance executes as PASSIVE (ptp.h scope note 2).
 * That is the correct telecom outcome: a T-GM whose own reference is gone
 * should defer to a healthy peer rather than keep grandmastering.
 */
static const ptp_class_ladder_t ladder_telecom = {
	.locked = 6U,
	.holdover_cat1 = 7U,
	.holdover_cat2 = 140U,
	.holdover_cat3 = 150U,
	.holdover_out_of_spec = 160U,
	.freerun = 248U,
};

/*
 * Power profile. IEEE C37.238 does not define its own clockClass semantics; it
 * inherits IEEE 1588's Table 4, so this is the Default ladder. The distinct
 * entry exists so a future C37.238 delta has somewhere to live without
 * disturbing Default.
 */
static const ptp_class_ladder_t ladder_power = {
	.locked = 6U,
	.holdover_cat1 = 7U,
	.holdover_cat2 = 7U,
	.holdover_cat3 = 7U,
	.holdover_out_of_spec = PTP_CLASS_USE_DEGRADATION,
	.freerun = PTP_CLASS_USE_DEGRADATION,
};

uint8_t ptp_class_from_ladder(const ptp_class_ladder_t *ladder, bool locked,
			      bool holdover, bool exceeded, ptp_freq_cat_t cat)
{
	if (ladder == NULL) {
		return PTP_CLASS_USE_DEGRADATION;
	}
	if (locked) {
		return ladder->locked;
	}
	if (holdover) {
		if (exceeded) {
			return ladder->holdover_out_of_spec;
		}
		switch (cat) {
		case PTP_FREQ_CAT1:
			return ladder->holdover_cat1;
		case PTP_FREQ_CAT2:
			return ladder->holdover_cat2;
		case PTP_FREQ_CAT3:
		case PTP_FREQ_CAT_COUNT:
		default:
			return ladder->holdover_cat3;
		}
	}
	return ladder->freerun;
}

ptp_freq_cat_t ptp_freq_cat_from_time_source(uint8_t time_source)
{
	switch (time_source) {
	case 0x10U: /* PTP_TIME_SRC_ATOMIC_CLOCK */
		return PTP_FREQ_CAT1;
	case 0x20U: /* PTP_TIME_SRC_GNSS */
	case 0xA0U: /* PTP_TIME_SRC_INTERNAL_OSC */
		/*
		 * GNSS is the *time* source; the thing that keeps time when GNSS
		 * goes away is the OCXO underneath it, and a holdover class
		 * describes the flywheel. Both therefore land on Category 2.
		 */
		return PTP_FREQ_CAT2;
	default:
		return PTP_FREQ_CAT3;
	}
}

/* ===================================================================== *
 *  Profile descriptors
 * ===================================================================== */

/*
 * Default profile, IEEE 1588-2019 Annex I.3 (delayRequest-response default
 * profile). Every value here is the pre-profile engine's own default, so
 * selecting PTP_PROFILE_DEFAULT changes nothing.
 */
static const ptp_profile_desc_t desc_default = {
	.name = "Default",
	.reference = "IEEE 1588-2019 Annex I.3",

	.domain_default = 0U,
	.domain_range = { 0U, 255U },

	.priority1_default = 128U,
	.priority1_range = { 0U, 255U },
	.priority2_default = 128U,
	.priority2_range = { 0U, 255U },
	/* Not used by the Default BMCA; kept in range so cfg validation passes. */
	.local_priority_default = 128U,
	.local_priority_range = { 1U, 255U },

	.log_announce_default = 1,  /* 2 s   */
	.log_announce_range = { 0, 4 },
	.log_sync_default = 0,      /* 1 s   */
	.log_sync_range = { -1, 1 },
	.log_min_delay_req_default = 0,
	.log_min_delay_req_range = { 0, 5 },

	.announce_receipt_timeout_default = 3U,
	.announce_receipt_timeout_range = { 2U, 10U },

	.transport_default = (uint8_t)PTP_TRANSPORT_UDP_IPV4,
	.transport_mask = PTP_XPORT_BIT(PTP_TRANSPORT_UDP_IPV4) |
			  PTP_XPORT_BIT(PTP_TRANSPORT_UDP_IPV6) |
			  PTP_XPORT_BIT(PTP_TRANSPORT_L2),
	.l2_mac_default = (uint8_t)PTP_L2_MAC_FORWARDABLE,

	.unicast = false,
	.use_priority1 = true,
	.use_local_priority = false,
	.not_slave_default = false,
	.announce_org_tlv = false,

	.ladder = &ladder_default,
	.deviations = PTP_DEV_TWO_STEP_ONLY | PTP_DEV_GM_ONLY,
};

/*
 * ITU-T G.8275.1: full timing support from the network, PTP over Ethernet
 * multicast, 128 Announce/s is not a thing — 8 Announce/s and 16 Sync/s are
 * (logAnnounceInterval -3, logSyncInterval -4).
 *
 * priority1 is fixed at 128 and *not used* by the alternate BMCA (§6.3); the
 * range is pinned to {128,128} so a configuration that tries to tune it is
 * rejected rather than silently ignored. localPriority (1..255, default 128)
 * takes its place as the operator's tiebreak.
 *
 * Domain 24 with the 24..43 range is G.8275.1 §6.5. The default destination MAC
 * is the non-forwardable 01-80-C2-00-00-0E, which is what keeps PTP inside a
 * link in a full-on-path-support deployment; the forwardable
 * 01-1B-19-00-00-00 is selectable for deployments that need it.
 */
static const ptp_profile_desc_t desc_g8275_1 = {
	.name = "G.8275.1",
	.reference = "ITU-T G.8275.1",

	.domain_default = 24U,
	.domain_range = { 24U, 43U },

	.priority1_default = 128U,
	.priority1_range = { 128U, 128U },
	.priority2_default = 128U,
	.priority2_range = { 0U, 255U },
	.local_priority_default = 128U,
	.local_priority_range = { 1U, 255U },

	.log_announce_default = -3, /* 8 Announce/s */
	.log_announce_range = { -3, -3 },
	.log_sync_default = -4,     /* 16 Sync/s */
	.log_sync_range = { -4, -4 },
	.log_min_delay_req_default = -4,
	.log_min_delay_req_range = { -4, -4 },

	.announce_receipt_timeout_default = 3U,
	.announce_receipt_timeout_range = { 2U, 3U },

	.transport_default = (uint8_t)PTP_TRANSPORT_L2,
	.transport_mask = PTP_XPORT_BIT(PTP_TRANSPORT_L2),
	.l2_mac_default = (uint8_t)PTP_L2_MAC_NON_FORWARDABLE,

	.unicast = false,
	.use_priority1 = false,
	.use_local_priority = true,
	.not_slave_default = true,
	.announce_org_tlv = false,

	.ladder = &ladder_telecom,
	.deviations = PTP_DEV_TWO_STEP_ONLY | PTP_DEV_GM_ONLY,
};

/*
 * ITU-T G.8275.2: partial timing support, PTP over UDP unicast. Domain 44 with
 * the 44..63 range; 1 Announce/s and 16 Sync/s are the nominal rates. The
 * alternate BMCA is G.8275.1's, so priority1 is again unused and localPriority
 * is the tiebreak.
 *
 * Unicast *message negotiation* is not implemented — see PTP_DEV_NO_UNICAST_NEG
 * and the header's scope note 2.
 */
static const ptp_profile_desc_t desc_g8275_2 = {
	.name = "G.8275.2",
	.reference = "ITU-T G.8275.2",

	.domain_default = 44U,
	.domain_range = { 44U, 63U },

	.priority1_default = 128U,
	.priority1_range = { 128U, 128U },
	.priority2_default = 128U,
	.priority2_range = { 0U, 255U },
	.local_priority_default = 128U,
	.local_priority_range = { 1U, 255U },

	.log_announce_default = 0,  /* 1 Announce/s */
	.log_announce_range = { -3, 4 },
	.log_sync_default = -4,     /* 16 Sync/s */
	.log_sync_range = { -6, 4 },
	.log_min_delay_req_default = -4,
	.log_min_delay_req_range = { -6, 4 },

	.announce_receipt_timeout_default = 3U,
	.announce_receipt_timeout_range = { 2U, 10U },

	.transport_default = (uint8_t)PTP_TRANSPORT_UDP_IPV4,
	.transport_mask = PTP_XPORT_BIT(PTP_TRANSPORT_UDP_IPV4) |
			  PTP_XPORT_BIT(PTP_TRANSPORT_UDP_IPV6),
	.l2_mac_default = (uint8_t)PTP_L2_MAC_FORWARDABLE, /* unused */

	.unicast = true,
	.use_priority1 = false,
	.use_local_priority = true,
	.not_slave_default = true,
	.announce_org_tlv = false,

	.ladder = &ladder_telecom,
	.deviations = PTP_DEV_TWO_STEP_ONLY | PTP_DEV_GM_ONLY |
		      PTP_DEV_NO_UNICAST_NEG,
};

/*
 * IEEE C37.238 power-system profile. Domain 0, 1 Announce/s and 1 Sync/s, PTP
 * over Ethernet multicast with the forwardable group MAC, and a mandatory
 * organization-extension TLV on every Announce.
 *
 * The profile mandates the *peer-delay* mechanism and a transparent-clock
 * network. This engine is E2E only, which PTP_DEV_E2E_ONLY records and
 * ptp_profile_deviation_text() renders. Selecting the profile is therefore
 * "C37.238 parameterisation and TLV", not "C37.238 conformance", and the
 * operator is told so on two surfaces: sts_ptp.c logs the text once at start,
 * and REST `/api/v1/status/ptp` carries `deviations` + `deviation_text` on every
 * poll. PTP_DEV_E2E_ONLY is also in PTP_DEV_MATERIAL, so ptp_port_init() raises
 * PTP_ALARM_PROFILE_UNSUPPORTED alongside. SNMP is NOT one of those surfaces:
 * enterprise .1.6 carries ptpAlarms as a bare Gauge32 and no profile or
 * deviation object — a poller still cannot decode it. See the open item in
 * scripts/reachability.allow's fmt block.
 */
static const ptp_profile_desc_t desc_c37_238 = {
	.name = "C37.238",
	.reference = "IEEE C37.238 (Power profile)",

	.domain_default = 0U,
	.domain_range = { 0U, 255U },

	.priority1_default = 128U,
	.priority1_range = { 0U, 255U },
	.priority2_default = 128U,
	.priority2_range = { 0U, 255U },
	.local_priority_default = 128U,
	.local_priority_range = { 1U, 255U },

	.log_announce_default = 0,  /* 1 Announce/s */
	.log_announce_range = { 0, 0 },
	.log_sync_default = 0,      /* 1 Sync/s */
	.log_sync_range = { 0, 0 },
	.log_min_delay_req_default = 0,
	.log_min_delay_req_range = { 0, 5 },

	.announce_receipt_timeout_default = 3U,
	.announce_receipt_timeout_range = { 2U, 10U },

	.transport_default = (uint8_t)PTP_TRANSPORT_L2,
	.transport_mask = PTP_XPORT_BIT(PTP_TRANSPORT_L2),
	.l2_mac_default = (uint8_t)PTP_L2_MAC_FORWARDABLE,

	.unicast = false,
	.use_priority1 = true,
	.use_local_priority = false,
	.not_slave_default = false,
	.announce_org_tlv = true,

	.ladder = &ladder_power,
	.deviations = PTP_DEV_TWO_STEP_ONLY | PTP_DEV_GM_ONLY |
		      PTP_DEV_E2E_ONLY,
};

static const ptp_profile_desc_t *const profile_tbl[] = {
	&desc_default,  /* PTP_PROFILE_DEFAULT          */
	&desc_g8275_1,  /* PTP_PROFILE_TELECOM_G8275_1  */
	&desc_g8275_2,  /* PTP_PROFILE_TELECOM_G8275_2  */
	&desc_c37_238,  /* PTP_PROFILE_POWER_C37_238    */
};

#define PROFILE_TBL_LEN (sizeof(profile_tbl) / sizeof(profile_tbl[0]))

const ptp_profile_desc_t *ptp_profile_desc(uint8_t profile)
{
	if ((size_t)profile >= PROFILE_TBL_LEN) {
		/* Degrade to the profile that always works, never to NULL. */
		return &desc_default;
	}
	return profile_tbl[profile];
}

const char *ptp_profile_name(uint8_t profile)
{
	return ptp_profile_desc(profile)->name;
}

const char *ptp_profile_deviation_text(uint8_t profile)
{
	uint16_t d = ptp_profile_desc(profile)->deviations;

	/*
	 * A small fixed set of combinations, returned as pointers to string
	 * literals rather than assembled into a shared buffer. That is what
	 * makes this re-entrant: there is no per-call state and no static
	 * scratch, so the two callers — sts_ptp.c's start-up log line on the
	 * caller of sts_ptp_start(), and sts_web.c's pv_ptp() on the web worker
	 * thread — need no lock between them, and the returned pointer stays
	 * valid for the life of the image rather than until the next call.
	 */
	if ((d & PTP_DEV_E2E_ONLY) != 0U) {
		return "E2E only (profile mandates peer-delay); two-step only; "
		       "grandmaster-only";
	}
	if ((d & PTP_DEV_NO_UNICAST_NEG) != 0U) {
		return "no unicast message negotiation; two-step only; "
		       "grandmaster-only";
	}
	if ((d & (PTP_DEV_TWO_STEP_ONLY | PTP_DEV_GM_ONLY)) != 0U) {
		return "two-step only; grandmaster-only";
	}
	return "none";
}

/* ===================================================================== *
 *  IEEE C37.238 organization extension TLV
 * ===================================================================== */

void ptp_c37238_defaults(ptp_c37238_t *p)
{
	if (p == NULL) {
		return;
	}
	(void)memset(p, 0, sizeof(*p));
	/*
	 * Zero inaccuracy reads as "0 ns", which no clock can honestly claim.
	 * C37.238 has no "unknown" code for these fields, so the operator must
	 * set them from the site's measured budget; leaving them zero is the
	 * visible-and-wrong default rather than an invisible-and-wrong one, and
	 * the maintenance tool reports it.
	 */
	p->v2017 = false;
}

int ptp_c37238_value_encode(uint8_t *buf, size_t cap, const ptp_c37238_t *p)
{
	if ((buf == NULL) || (p == NULL)) {
		return -EINVAL;
	}
	if (cap < (size_t)PTP_C37238_VALUE_LEN) {
		return -ENOSPC;
	}

	(void)memset(buf, 0, (size_t)PTP_C37238_VALUE_LEN);

	if (p->v2017) {
		buf[0] = PTP_C37238_ORG_ID_2017_0;
		buf[1] = PTP_C37238_ORG_ID_2017_1;
		buf[2] = PTP_C37238_ORG_ID_2017_2;
		put_be24(&buf[3], (uint32_t)PTP_C37238_SUBTYPE_2017);
		/* octets 6..7 and 8..11 reserved (already zero) */
		bytes_put_be32(&buf[12], p->total_inaccuracy_ns);
		/* octets 16..17 reserved */
	} else {
		buf[0] = PTP_C37238_ORG_ID_2011_0;
		buf[1] = PTP_C37238_ORG_ID_2011_1;
		buf[2] = PTP_C37238_ORG_ID_2011_2;
		put_be24(&buf[3], (uint32_t)PTP_C37238_SUBTYPE_2011);
		bytes_put_be16(&buf[6], p->grandmaster_id);
		bytes_put_be32(&buf[8], p->gm_inaccuracy_ns);
		bytes_put_be32(&buf[12], p->network_inaccuracy_ns);
		/* octets 16..17 reserved */
	}
	return 0;
}

int ptp_c37238_value_decode(const uint8_t *buf, size_t len, ptp_c37238_t *out)
{
	uint32_t subtype;

	if ((buf == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (len != (size_t)PTP_C37238_VALUE_LEN) {
		return -EBADMSG;
	}

	(void)memset(out, 0, sizeof(*out));
	subtype = get_be24(&buf[3]);

	if ((buf[0] == PTP_C37238_ORG_ID_2011_0) &&
	    (buf[1] == PTP_C37238_ORG_ID_2011_1) &&
	    (buf[2] == PTP_C37238_ORG_ID_2011_2) &&
	    (subtype == (uint32_t)PTP_C37238_SUBTYPE_2011)) {
		out->v2017 = false;
		out->grandmaster_id = bytes_get_be16(&buf[6]);
		out->gm_inaccuracy_ns = bytes_get_be32(&buf[8]);
		out->network_inaccuracy_ns = bytes_get_be32(&buf[12]);
		return 0;
	}

	if ((buf[0] == PTP_C37238_ORG_ID_2017_0) &&
	    (buf[1] == PTP_C37238_ORG_ID_2017_1) &&
	    (buf[2] == PTP_C37238_ORG_ID_2017_2) &&
	    (subtype == (uint32_t)PTP_C37238_SUBTYPE_2017)) {
		out->v2017 = true;
		out->total_inaccuracy_ns = bytes_get_be32(&buf[12]);
		return 0;
	}

	return -EPROTO;
}

int ptp_c37238_append(uint8_t *buf, size_t cap, size_t *len,
		      const ptp_c37238_t *p)
{
	uint8_t value[PTP_C37238_VALUE_LEN];
	int rc;

	if ((buf == NULL) || (len == NULL) || (p == NULL)) {
		return -EINVAL;
	}

	rc = ptp_c37238_value_encode(value, sizeof(value), p);
	if (rc != 0) {
		return rc;
	}

	return ptp_tlv_append(buf, cap, len,
			      (uint16_t)PTP_TLV_TYPE_ORGANIZATION_EXTENSION,
			      value, sizeof(value), NULL);
}

int ptp_c37238_find(const uint8_t *buf, size_t len, ptp_c37238_t *out)
{
	ptp_tlv_iter_t it;
	uint16_t type;
	const uint8_t *val;
	size_t val_len;
	int rc;
	bool saw_candidate = false;

	if ((buf == NULL) || (out == NULL)) {
		return -EINVAL;
	}

	rc = ptp_tlv_iter_begin(&it, buf, len);
	if (rc != 0) {
		return rc;
	}

	for (;;) {
		rc = ptp_tlv_iter_next(&it, &type, &val, &val_len, NULL);
		if (rc == -ENOENT) {
			break;
		}
		if (rc != 0) {
			return rc;
		}
		if (type != (uint16_t)PTP_TLV_TYPE_ORGANIZATION_EXTENSION) {
			continue;
		}
		if (val_len < 3U) {
			continue;
		}
		/*
		 * Match the organizationId before insisting on a length: a
		 * different vendor's organization TLV is not an error, but one
		 * of *ours* with the wrong length is.
		 */
		if (!(((val[0] == PTP_C37238_ORG_ID_2011_0) &&
		       (val[1] == PTP_C37238_ORG_ID_2011_1) &&
		       (val[2] == PTP_C37238_ORG_ID_2011_2)) ||
		      ((val[0] == PTP_C37238_ORG_ID_2017_0) &&
		       (val[1] == PTP_C37238_ORG_ID_2017_1) &&
		       (val[2] == PTP_C37238_ORG_ID_2017_2)))) {
			continue;
		}
		saw_candidate = true;
		rc = ptp_c37238_value_decode(val, val_len, out);
		if (rc == 0) {
			return 0;
		}
		/* Keep looking: a peer may carry more than one org TLV. */
	}

	return saw_candidate ? -EPROTO : -ENOENT;
}
