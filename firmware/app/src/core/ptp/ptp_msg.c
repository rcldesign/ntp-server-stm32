/*
 * STS1000 "Meridian" — core/ptp: IEEE 1588-2019 message codecs.
 *
 * See ptp_msg.h for the contract. Layout references are to IEEE 1588-2019 §13.
 *
 * Common header, octets 0..33:
 *
 *   0      majorSdoId (7..4) | messageType (3..0)
 *   1      minorVersionPTP (7..4) | versionPTP (3..0)
 *   2-3    messageLength
 *   4      domainNumber
 *   5      minorSdoId
 *   6-7    flagField
 *   8-15   correctionField          Integer64, nanoseconds scaled by 2^16
 *   16-19  messageTypeSpecific
 *   20-27  sourcePortIdentity.clockIdentity
 *   28-29  sourcePortIdentity.portNumber
 *   30-31  sequenceId
 *   32     controlField             legacy, per Table 39
 *   33     logMessageInterval       Integer8
 */

#include "ptp/ptp_msg.h"

#include <errno.h>
#include <string.h>

#include "util/bytes.h"

/* Body offsets, from the start of the message. */
#define OFF_BODY            PTP_HDR_LEN                 /* 34 */
#define OFF_ANN_UTC_OFFSET  (OFF_BODY + PTP_TS_LEN)     /* 44 */
#define OFF_ANN_RESERVED    (OFF_ANN_UTC_OFFSET + 2U)   /* 46 */
#define OFF_ANN_PRIORITY1   (OFF_ANN_RESERVED + 1U)     /* 47 */
#define OFF_ANN_QUALITY     (OFF_ANN_PRIORITY1 + 1U)    /* 48 */
#define OFF_ANN_PRIORITY2   (OFF_ANN_QUALITY + 4U)      /* 52 */
#define OFF_ANN_GM_IDENTITY (OFF_ANN_PRIORITY2 + 1U)    /* 53 */
#define OFF_ANN_STEPS       (OFF_ANN_GM_IDENTITY + 8U)  /* 61 */
#define OFF_ANN_TIME_SOURCE (OFF_ANN_STEPS + 2U)        /* 63 */

#define OFF_DRESP_REQ_PORT  (OFF_BODY + PTP_TS_LEN)     /* 44 */

#define NS_PER_SEC 1000000000ULL

/* Largest TAI nanosecond count representable in a uint64. */
#define NS_MAX_SECONDS (UINT64_MAX / NS_PER_SEC)
#define NS_MAX_REMAINDER (UINT64_MAX % NS_PER_SEC)

/* 48-bit seconds field mask. */
#define SECONDS_MASK 0x0000FFFFFFFFFFFFULL

/* ------------------------------------------------------------- internals -- */

/*
 * Reinterpret the wire's two's-complement Integer64 as int64_t without relying
 * on the implementation-defined behaviour of an out-of-range unsigned-to-signed
 * conversion (C11 6.3.1.3p3).
 */
static int64_t u64_to_i64(uint64_t u)
{
	if (u <= (uint64_t)INT64_MAX) {
		return (int64_t)u;
	}
	return -(int64_t)(~u) - 1;
}

static uint64_t i64_to_u64(int64_t v)
{
	if (v >= 0) {
		return (uint64_t)v;
	}
	return ~(uint64_t)(-(v + 1));
}

static int16_t u16_to_i16(uint16_t u)
{
	if (u <= (uint16_t)INT16_MAX) {
		return (int16_t)u;
	}
	return (int16_t)-(int32_t)((uint32_t)(uint16_t)(~u) + 1U);
}

static void port_id_decode(const uint8_t *p, ptp_port_id_t *out)
{
	memcpy(out->clock_id.id, p, PTP_CLOCK_ID_LEN);
	out->port_number = bytes_get_be16(&p[PTP_CLOCK_ID_LEN]);
}

static void port_id_encode(uint8_t *p, const ptp_port_id_t *id)
{
	memcpy(p, id->clock_id.id, PTP_CLOCK_ID_LEN);
	bytes_put_be16(&p[PTP_CLOCK_ID_LEN], id->port_number);
}

/* ---------------------------------------------------------- small helpers -- */

int ptp_clock_id_from_mac(ptp_clock_id_t *out, const uint8_t *mac)
{
	if ((out == NULL) || (mac == NULL)) {
		return -EINVAL;
	}

	/* EUI-48 -> EUI-64 per §7.5.2.2.2: OUI, 0xFF, 0xFE, extension. */
	out->id[0] = mac[0];
	out->id[1] = mac[1];
	out->id[2] = mac[2];
	out->id[3] = 0xFFU;
	out->id[4] = 0xFEU;
	out->id[5] = mac[3];
	out->id[6] = mac[4];
	out->id[7] = mac[5];
	return 0;
}

int ptp_clock_id_cmp(const ptp_clock_id_t *a, const ptp_clock_id_t *b)
{
	return memcmp(a->id, b->id, PTP_CLOCK_ID_LEN);
}

int ptp_port_id_cmp(const ptp_port_id_t *a, const ptp_port_id_t *b)
{
	int d = ptp_clock_id_cmp(&a->clock_id, &b->clock_id);

	if (d != 0) {
		return d;
	}
	if (a->port_number < b->port_number) {
		return -1;
	}
	if (a->port_number > b->port_number) {
		return 1;
	}
	return 0;
}

size_t ptp_msg_min_len(uint8_t msg_type)
{
	switch (msg_type) {
	case PTP_MSG_SYNC:
	case PTP_MSG_DELAY_REQ:
	case PTP_MSG_FOLLOW_UP:
		return PTP_TSMSG_LEN;                /* header + Timestamp */
	case PTP_MSG_PDELAY_REQ:
	case PTP_MSG_PDELAY_RESP:
	case PTP_MSG_PDELAY_RESP_FOLLOW_UP:
	case PTP_MSG_DELAY_RESP:
		return PTP_DELAY_RESP_LEN;           /* header + Timestamp + PortIdentity */
	case PTP_MSG_ANNOUNCE:
		return PTP_ANNOUNCE_LEN;
	case PTP_MSG_SIGNALING:
		return PTP_HDR_LEN + PTP_PORT_ID_LEN;                  /* 44 */
	case PTP_MSG_MANAGEMENT:
		return PTP_HDR_LEN + PTP_PORT_ID_LEN + 4U;             /* 48 */
	default:
		return PTP_HDR_LEN;
	}
}

uint8_t ptp_msg_control_field(uint8_t msg_type)
{
	switch (msg_type) {
	case PTP_MSG_SYNC:
		return 0U;
	case PTP_MSG_DELAY_REQ:
		return 1U;
	case PTP_MSG_FOLLOW_UP:
		return 2U;
	case PTP_MSG_DELAY_RESP:
		return 3U;
	case PTP_MSG_MANAGEMENT:
		return 4U;
	default:
		return 5U; /* "all others" */
	}
}

const char *ptp_msg_type_name(uint8_t msg_type)
{
	switch (msg_type) {
	case PTP_MSG_SYNC:
		return "Sync";
	case PTP_MSG_DELAY_REQ:
		return "Delay_Req";
	case PTP_MSG_PDELAY_REQ:
		return "Pdelay_Req";
	case PTP_MSG_PDELAY_RESP:
		return "Pdelay_Resp";
	case PTP_MSG_FOLLOW_UP:
		return "Follow_Up";
	case PTP_MSG_DELAY_RESP:
		return "Delay_Resp";
	case PTP_MSG_PDELAY_RESP_FOLLOW_UP:
		return "Pdelay_Resp_Follow_Up";
	case PTP_MSG_ANNOUNCE:
		return "Announce";
	case PTP_MSG_SIGNALING:
		return "Signaling";
	case PTP_MSG_MANAGEMENT:
		return "Management";
	default:
		return "reserved";
	}
}

/* -------------------------------------------------------------- timestamp -- */

void ptp_ts_decode(const uint8_t *buf, ptp_timestamp_t *out)
{
	out->seconds = ((uint64_t)bytes_get_be16(buf) << 32) |
		       (uint64_t)bytes_get_be32(&buf[2]);
	out->nanoseconds = bytes_get_be32(&buf[6]);
}

void ptp_ts_encode(uint8_t *buf, const ptp_timestamp_t *ts)
{
	uint64_t s = ts->seconds & SECONDS_MASK;

	bytes_put_be16(buf, (uint16_t)((s >> 32) & 0xFFFFU));
	bytes_put_be32(&buf[2], (uint32_t)(s & 0xFFFFFFFFU));
	bytes_put_be32(&buf[6], ts->nanoseconds);
}

ptp_timestamp_t ptp_ts_from_ns(uint64_t tai_ns)
{
	ptp_timestamp_t ts;

	ts.seconds = tai_ns / NS_PER_SEC;
	ts.nanoseconds = (uint32_t)(tai_ns % NS_PER_SEC);
	return ts;
}

uint64_t ptp_ts_to_ns(const ptp_timestamp_t *ts)
{
	uint64_t s = ts->seconds & SECONDS_MASK;

	if (s > NS_MAX_SECONDS) {
		return UINT64_MAX;
	}
	if ((s == NS_MAX_SECONDS) && (ts->nanoseconds > NS_MAX_REMAINDER)) {
		return UINT64_MAX;
	}
	return (s * NS_PER_SEC) + (uint64_t)ts->nanoseconds;
}

/* ----------------------------------------------------------------- header -- */

int ptp_hdr_decode(const uint8_t *buf, size_t len, ptp_hdr_t *out)
{
	ptp_hdr_t h;

	if ((buf == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (len < PTP_HDR_LEN) {
		return -EBADMSG;
	}

	h.major_sdo_id = (uint8_t)(buf[0] >> 4);
	h.msg_type = (uint8_t)(buf[0] & 0x0FU);
	h.minor_version = (uint8_t)(buf[1] >> 4);
	h.version = (uint8_t)(buf[1] & 0x0FU);

	if (h.version != PTP_VERSION) {
		return -EPROTO;
	}

	h.msg_length = bytes_get_be16(&buf[2]);
	h.domain = buf[4];
	h.minor_sdo_id = buf[5];
	h.flags = bytes_get_be16(&buf[6]);
	h.correction = u64_to_i64(bytes_get_be64(&buf[8]));
	h.msg_type_specific = bytes_get_be32(&buf[16]);
	port_id_decode(&buf[20], &h.source_port);
	h.seq_id = bytes_get_be16(&buf[30]);
	h.control = buf[32];
	h.log_msg_interval = (int8_t)buf[33];

	/*
	 * messageLength is the authoritative message extent (§13.3.2.4). It must
	 * cover the fixed body of its type and must not claim more than the
	 * caller actually holds. Trailing octets are legal — 802.3 pads short
	 * frames to 60 octets, and suffix TLVs are permitted — and are ignored.
	 */
	if ((size_t)h.msg_length < ptp_msg_min_len(h.msg_type)) {
		return -EBADMSG;
	}
	if ((size_t)h.msg_length > len) {
		return -EBADMSG;
	}

	*out = h;
	return 0;
}

int ptp_hdr_encode(uint8_t *buf, size_t cap, const ptp_hdr_t *h)
{
	if ((buf == NULL) || (h == NULL)) {
		return -EINVAL;
	}
	if (cap < PTP_HDR_LEN) {
		return -ENOSPC;
	}

	buf[0] = (uint8_t)(((h->major_sdo_id & 0x0FU) << 4) | (h->msg_type & 0x0FU));
	buf[1] = (uint8_t)(((h->minor_version & 0x0FU) << 4) | (h->version & 0x0FU));
	bytes_put_be16(&buf[2], h->msg_length);
	buf[4] = h->domain;
	buf[5] = h->minor_sdo_id;
	bytes_put_be16(&buf[6], h->flags);
	bytes_put_be64(&buf[8], i64_to_u64(h->correction));
	bytes_put_be32(&buf[16], h->msg_type_specific);
	port_id_encode(&buf[20], &h->source_port);
	bytes_put_be16(&buf[30], h->seq_id);
	buf[32] = h->control;
	buf[33] = (uint8_t)h->log_msg_interval;
	return 0;
}

/* ------------------------------------------------------ timestamp messages -- */

int ptp_tsmsg_encode(uint8_t *buf, size_t cap, const ptp_hdr_t *h,
		     const ptp_timestamp_t *ts, size_t *out_len)
{
	int rc;

	if ((buf == NULL) || (h == NULL) || (ts == NULL)) {
		return -EINVAL;
	}
	if (cap < PTP_TSMSG_LEN) {
		return -ENOSPC;
	}

	rc = ptp_hdr_encode(buf, cap, h);
	if (rc != 0) {
		return rc;
	}
	bytes_put_be16(&buf[2], (uint16_t)PTP_TSMSG_LEN);
	ptp_ts_encode(&buf[OFF_BODY], ts);

	if (out_len != NULL) {
		*out_len = PTP_TSMSG_LEN;
	}
	return 0;
}

int ptp_tsmsg_decode(const uint8_t *buf, size_t len, ptp_timestamp_t *out)
{
	if ((buf == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (len < PTP_TSMSG_LEN) {
		return -EBADMSG;
	}

	ptp_ts_decode(&buf[OFF_BODY], out);
	return 0;
}

/* --------------------------------------------------------------- announce -- */

int ptp_announce_encode(uint8_t *buf, size_t cap, const ptp_hdr_t *h,
			const ptp_announce_t *a, size_t *out_len)
{
	int rc;

	if ((buf == NULL) || (h == NULL) || (a == NULL)) {
		return -EINVAL;
	}
	if (cap < PTP_ANNOUNCE_LEN) {
		return -ENOSPC;
	}

	rc = ptp_hdr_encode(buf, cap, h);
	if (rc != 0) {
		return rc;
	}
	bytes_put_be16(&buf[2], (uint16_t)PTP_ANNOUNCE_LEN);

	ptp_ts_encode(&buf[OFF_BODY], &a->origin_ts);
	bytes_put_be16(&buf[OFF_ANN_UTC_OFFSET],
		       (uint16_t)(uint32_t)(int32_t)a->current_utc_offset);
	buf[OFF_ANN_RESERVED] = 0U;
	buf[OFF_ANN_PRIORITY1] = a->gm_priority1;
	buf[OFF_ANN_QUALITY + 0U] = a->gm_quality.clock_class;
	buf[OFF_ANN_QUALITY + 1U] = a->gm_quality.clock_accuracy;
	bytes_put_be16(&buf[OFF_ANN_QUALITY + 2U],
		       a->gm_quality.offset_scaled_log_variance);
	buf[OFF_ANN_PRIORITY2] = a->gm_priority2;
	memcpy(&buf[OFF_ANN_GM_IDENTITY], a->gm_identity.id, PTP_CLOCK_ID_LEN);
	bytes_put_be16(&buf[OFF_ANN_STEPS], a->steps_removed);
	buf[OFF_ANN_TIME_SOURCE] = a->time_source;

	if (out_len != NULL) {
		*out_len = PTP_ANNOUNCE_LEN;
	}
	return 0;
}

int ptp_announce_decode(const uint8_t *buf, size_t len, ptp_announce_t *out)
{
	if ((buf == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (len < PTP_ANNOUNCE_LEN) {
		return -EBADMSG;
	}

	ptp_ts_decode(&buf[OFF_BODY], &out->origin_ts);
	out->current_utc_offset = u16_to_i16(bytes_get_be16(&buf[OFF_ANN_UTC_OFFSET]));
	out->gm_priority1 = buf[OFF_ANN_PRIORITY1];
	out->gm_quality.clock_class = buf[OFF_ANN_QUALITY + 0U];
	out->gm_quality.clock_accuracy = buf[OFF_ANN_QUALITY + 1U];
	out->gm_quality.offset_scaled_log_variance =
		bytes_get_be16(&buf[OFF_ANN_QUALITY + 2U]);
	out->gm_priority2 = buf[OFF_ANN_PRIORITY2];
	memcpy(out->gm_identity.id, &buf[OFF_ANN_GM_IDENTITY], PTP_CLOCK_ID_LEN);
	out->steps_removed = bytes_get_be16(&buf[OFF_ANN_STEPS]);
	out->time_source = buf[OFF_ANN_TIME_SOURCE];
	return 0;
}

/* ------------------------------------------------------------- delay_resp -- */

int ptp_delay_resp_encode(uint8_t *buf, size_t cap, const ptp_hdr_t *h,
			  const ptp_timestamp_t *rx_ts,
			  const ptp_port_id_t *requesting, size_t *out_len)
{
	int rc;

	if ((buf == NULL) || (h == NULL) || (rx_ts == NULL) || (requesting == NULL)) {
		return -EINVAL;
	}
	if (cap < PTP_DELAY_RESP_LEN) {
		return -ENOSPC;
	}

	rc = ptp_hdr_encode(buf, cap, h);
	if (rc != 0) {
		return rc;
	}
	bytes_put_be16(&buf[2], (uint16_t)PTP_DELAY_RESP_LEN);

	ptp_ts_encode(&buf[OFF_BODY], rx_ts);
	port_id_encode(&buf[OFF_DRESP_REQ_PORT], requesting);

	if (out_len != NULL) {
		*out_len = PTP_DELAY_RESP_LEN;
	}
	return 0;
}

int ptp_delay_resp_decode(const uint8_t *buf, size_t len,
			  ptp_timestamp_t *rx_ts, ptp_port_id_t *requesting)
{
	if (buf == NULL) {
		return -EINVAL;
	}
	if (len < PTP_DELAY_RESP_LEN) {
		return -EBADMSG;
	}

	if (rx_ts != NULL) {
		ptp_ts_decode(&buf[OFF_BODY], rx_ts);
	}
	if (requesting != NULL) {
		port_id_decode(&buf[OFF_DRESP_REQ_PORT], requesting);
	}
	return 0;
}
