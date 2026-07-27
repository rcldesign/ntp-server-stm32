/*
 * STS1000 "Meridian" — core/ptp: IEEE 1588-2019 message codecs.
 *
 * Platform-neutral C11, no allocation. All wire access is byte-wise big-endian
 * through util/bytes.h, so the codecs are alignment- and host-endianness-
 * independent and the host unit tests are representative of the target.
 *
 * Scope is the PTP **PDU only**. Encapsulation — UDP/IPv4 (IEEE 1588-2019
 * Annex C), UDP/IPv6 (Annex D) and IEEE 802.3 (Annex E) — belongs to the glue:
 * this layer hands out PDU bytes plus an addressing hint (event vs general
 * port, which multicast group), and consumes PDU bytes plus a hardware
 * timestamp. See ptp.h for the port engine that drives these codecs.
 *
 * Field layouts follow IEEE 1588-2019 §13; the octet offsets in the comments
 * are measured from the first octet of the PTP message.
 *
 * Robustness contract: every decoder validates its length before reading and
 * returns a negative errno-style code rather than trapping. No decoder reads
 * outside [buf, buf + len). Garbage in never means a crash — the port engine
 * feeds these functions straight from the network.
 */

#ifndef STS1000_CORE_PTP_PTP_MSG_H_
#define STS1000_CORE_PTP_PTP_MSG_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------- constants -- */

/** Octets in a ClockIdentity (§7.5.2.2). */
#define PTP_CLOCK_ID_LEN 8U

/** Octets in the common message header (§13.3). */
#define PTP_HDR_LEN 34U

/** Octets in a wire Timestamp: 48-bit seconds + 32-bit nanoseconds (§5.3.3). */
#define PTP_TS_LEN 10U

/** Octets in a wire PortIdentity: ClockIdentity + portNumber (§5.3.5). */
#define PTP_PORT_ID_LEN 10U

/** Wire length of Sync / Follow_Up / Delay_Req. */
#define PTP_TSMSG_LEN 44U

/** Wire length of Delay_Resp (§13.8). */
#define PTP_DELAY_RESP_LEN 54U

/** Wire length of Announce (§13.5). */
#define PTP_ANNOUNCE_LEN 64U

/**
 * Largest PDU this module will build or accept for parsing.
 *
 * Sized by the worst case this firmware actually emits: a 64-octet Announce,
 * plus the 22-octet IEEE C37.238 organization TLV the Power profile mandates
 * (ptp_profile.h), plus an Annex-P AUTHENTICATION TLV carrying a full 32-octet
 * ICV — 4 header + 6 fixed + 4 sequenceNo + 32 = 46 octets (ptp_icv.h). That is
 * 132; 160 leaves room for one more suffix TLV without another audit of every
 * buffer that is declared with this bound.
 */
#define PTP_MSG_MAX_LEN 160U

/** versionPTP of IEEE 1588-2008 and -2019 (§13.3.2.3). */
#define PTP_VERSION 2U

/** minorVersionPTP of IEEE 1588-2019 (§13.3.2.3); 0 identifies a -2008 peer. */
#define PTP_MINOR_VERSION_2019 1U

/** logMessageInterval value meaning "unspecified" (§13.3.2.14). */
#define PTP_LOG_INTERVAL_UNSPEC ((int8_t)0x7F)

/** Widest log message interval this engine accepts: 2^-7 s .. 2^7 s. */
#define PTP_LOG_INTERVAL_MIN (-7)
#define PTP_LOG_INTERVAL_MAX (7)

/* --------------------------------------------------------- message types -- */

/** messageType field, low nibble of octet 0 (§13.3.2.2, Table 36). */
typedef enum {
	PTP_MSG_SYNC                  = 0x0,
	PTP_MSG_DELAY_REQ             = 0x1,
	PTP_MSG_PDELAY_REQ            = 0x2,
	PTP_MSG_PDELAY_RESP           = 0x3,
	PTP_MSG_FOLLOW_UP             = 0x8,
	PTP_MSG_DELAY_RESP            = 0x9,
	PTP_MSG_PDELAY_RESP_FOLLOW_UP = 0xA,
	PTP_MSG_ANNOUNCE              = 0xB,
	PTP_MSG_SIGNALING             = 0xC,
	PTP_MSG_MANAGEMENT            = 0xD,
} ptp_msg_type_t;

/** Number of distinct messageType codes; sizes the per-type counter arrays. */
#define PTP_MSG_TYPE_COUNT 16U

/** True for the four event message types, which are hardware-timestamped. */
static inline bool ptp_msg_is_event(uint8_t msg_type)
{
	return msg_type <= (uint8_t)PTP_MSG_PDELAY_RESP;
}

/* ---------------------------------------------------------------- flags --- */

/*
 * flagField (§13.3.2.8, Table 37) is carried here as a uint16 in wire order:
 * octet 0 of the field occupies bits 15..8, octet 1 bits 7..0. Bit numbering
 * within each octet is LSB-first, as in the standard.
 */
#define PTP_FLAG_ALTERNATE_MASTER  0x0100U /* octet 0 bit 0 */
#define PTP_FLAG_TWO_STEP          0x0200U /* octet 0 bit 1 */
#define PTP_FLAG_UNICAST           0x0400U /* octet 0 bit 2 */
#define PTP_FLAG_PROFILE_1         0x2000U /* octet 0 bit 5 */
#define PTP_FLAG_PROFILE_2         0x4000U /* octet 0 bit 6 */
#define PTP_FLAG_LEAP61            0x0001U /* octet 1 bit 0 */
#define PTP_FLAG_LEAP59            0x0002U /* octet 1 bit 1 */
#define PTP_FLAG_UTC_OFFSET_VALID  0x0004U /* octet 1 bit 2 */
#define PTP_FLAG_PTP_TIMESCALE     0x0008U /* octet 1 bit 3 */
#define PTP_FLAG_TIME_TRACEABLE    0x0010U /* octet 1 bit 4 */
#define PTP_FLAG_FREQ_TRACEABLE    0x0020U /* octet 1 bit 5 */
#define PTP_FLAG_SYNC_UNCERTAIN    0x0040U /* octet 1 bit 6 */

/* ------------------------------------------------------ primitive types --- */

/** ClockIdentity (§5.3.4): an EUI-64. */
typedef struct {
	uint8_t id[PTP_CLOCK_ID_LEN];
} ptp_clock_id_t;

/** PortIdentity (§5.3.5). portNumber is 1-based; 0 is reserved. */
typedef struct {
	ptp_clock_id_t clock_id;
	uint16_t port_number;
} ptp_port_id_t;

/**
 * Timestamp (§5.3.3): 48-bit unsigned seconds plus 32-bit nanoseconds, both
 * relative to the PTP epoch (1970-01-01 00:00:00 TAI).
 */
typedef struct {
	uint64_t seconds;     /* only the low 48 bits are on the wire */
	uint32_t nanoseconds; /* 0 .. 999999999 */
} ptp_timestamp_t;

/** ClockQuality (§5.3.7). */
typedef struct {
	uint8_t clock_class;
	uint8_t clock_accuracy;
	uint16_t offset_scaled_log_variance;
} ptp_clock_quality_t;

/* ------------------------------------------------------- decoded header --- */

/** Common message header, §13.3. */
typedef struct {
	uint8_t major_sdo_id;      /* octet 0, high nibble (2008: transportSpecific) */
	uint8_t msg_type;          /* octet 0, low nibble */
	uint8_t minor_version;     /* octet 1, high nibble */
	uint8_t version;           /* octet 1, low nibble; must be 2 */
	uint16_t msg_length;       /* octets 2-3 */
	uint8_t domain;            /* octet 4 */
	uint8_t minor_sdo_id;      /* octet 5 (2008: reserved) */
	uint16_t flags;            /* octets 6-7, see PTP_FLAG_* */
	int64_t correction;        /* octets 8-15, nanoseconds scaled by 2^16 */
	uint32_t msg_type_specific;/* octets 16-19 (2008: reserved) */
	ptp_port_id_t source_port; /* octets 20-29 */
	uint16_t seq_id;           /* octets 30-31 */
	uint8_t control;           /* octet 32, legacy controlField */
	int8_t log_msg_interval;   /* octet 33 */
} ptp_hdr_t;

/** Announce body, §13.5 (octets 34..63). */
typedef struct {
	ptp_timestamp_t origin_ts;    /* reserved in -2019; transmitted as zero */
	int16_t current_utc_offset;   /* TAI - UTC, seconds */
	uint8_t gm_priority1;
	ptp_clock_quality_t gm_quality;
	uint8_t gm_priority2;
	ptp_clock_id_t gm_identity;
	uint16_t steps_removed;
	uint8_t time_source;
} ptp_announce_t;

/* -------------------------------------------------------- small helpers --- */

/**
 * Build a ClockIdentity from an EUI-48 MAC address, per the IEEE EUI-64
 * encapsulation in §7.5.2.2.2: OUI, 0xFF, 0xFE, extension identifier.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p out or @p mac is NULL.
 */
int ptp_clock_id_from_mac(ptp_clock_id_t *out, const uint8_t *mac);

/** Lexicographic compare of two ClockIdentities. <0, 0 or >0. */
int ptp_clock_id_cmp(const ptp_clock_id_t *a, const ptp_clock_id_t *b);

/** Compare PortIdentities: ClockIdentity first, then portNumber. <0, 0 or >0. */
int ptp_port_id_cmp(const ptp_port_id_t *a, const ptp_port_id_t *b);

/**
 * Minimum wire length of @p msg_type, in octets. Unknown types give the header.
 *
 * These are the fixed-field minima only. Management (§13.11) and Signaling
 * (§13.12) additionally carry at least one mandatory TLV, so a conforming
 * message of either type is always longer than the figure returned here. That
 * is deliberate and safe for this engine, which parses neither: both are
 * accepted, counted and dropped, so the number is used purely as a lower bound
 * for memory safety. A future Management responder must check the TLV extent
 * for itself.
 */
size_t ptp_msg_min_len(uint8_t msg_type);

/** Legacy controlField value for @p msg_type (§13.3.2.10, Table 39). */
uint8_t ptp_msg_control_field(uint8_t msg_type);

/** Short name of @p msg_type for logs; "reserved" for unassigned codes. */
const char *ptp_msg_type_name(uint8_t msg_type);

/**
 * Milliseconds in 2^@p log_interval seconds.
 *
 * logMessageInterval is a signed power-of-two exponent (§13.3.2.14). The
 * exponent is clamped to [-7, +7] — 7.8125 ms to 128 s, which brackets every
 * interval any profile in scope uses — and fractional-millisecond results are
 * rounded to nearest, so 2^-7 s reports 8 ms.
 */
uint32_t ptp_log_interval_ms(int8_t log_interval);

/* ------------------------------------------------------------ timestamp --- */

/** Decode the 10-octet Timestamp at @p buf. @p buf must hold PTP_TS_LEN octets. */
void ptp_ts_decode(const uint8_t *buf, ptp_timestamp_t *out);

/**
 * Encode @p ts as a 10-octet Timestamp at @p buf.
 *
 * seconds is truncated to its low 48 bits, matching the wire field width.
 */
void ptp_ts_encode(uint8_t *buf, const ptp_timestamp_t *ts);

/** Split TAI nanoseconds into a Timestamp. */
ptp_timestamp_t ptp_ts_from_ns(uint64_t tai_ns);

/**
 * Recombine a Timestamp into TAI nanoseconds.
 *
 * The 48-bit seconds field spans ~8.9 million years, far more than 2^64 ns can
 * hold, so a timestamp beyond UINT64_MAX nanoseconds saturates at UINT64_MAX
 * rather than wrapping. Nanoseconds are used as-is; a malformed field ≥ 1e9 is
 * carried through so the caller can see it.
 */
uint64_t ptp_ts_to_ns(const ptp_timestamp_t *ts);

/* --------------------------------------------------------------- header --- */

/**
 * Decode and sanity-check the common header of a received message.
 *
 * Validation performed here, so that no body decoder can be reached with a
 * length it cannot satisfy:
 *   - @p len must be at least PTP_HDR_LEN;
 *   - versionPTP must be 2 (minorVersionPTP is not constrained);
 *   - messageLength must be at least the minimum for the messageType and must
 *     not exceed @p len. Trailing octets beyond messageLength are legal —
 *     802.3 pads short frames to 60 octets — and are ignored.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p buf or @p out is NULL.
 * @retval -EBADMSG Truncated, or messageLength inconsistent with the type.
 * @retval -EPROTO  versionPTP is not 2.
 */
int ptp_hdr_decode(const uint8_t *buf, size_t len, ptp_hdr_t *out);

/**
 * Encode @p h as a 34-octet common header.
 *
 * Every field is written verbatim, including messageLength — the message
 * encoders below overwrite it with the true length, so a caller only has to
 * get it right when building a header on its own.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p buf or @p h is NULL.
 * @retval -ENOSPC  @p cap is below PTP_HDR_LEN.
 */
int ptp_hdr_encode(uint8_t *buf, size_t cap, const ptp_hdr_t *h);

/* ---------------------------------------------------- timestamp messages -- */

/**
 * Encode Sync, Follow_Up or Delay_Req: common header plus one Timestamp.
 *
 * The three share a layout; the header's messageType selects which one it is.
 * messageLength is written as PTP_TSMSG_LEN.
 *
 * @param out_len  Receives the encoded length. May be NULL.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p buf, @p h or @p ts is NULL.
 * @retval -ENOSPC  @p cap is below PTP_TSMSG_LEN.
 */
int ptp_tsmsg_encode(uint8_t *buf, size_t cap, const ptp_hdr_t *h,
		     const ptp_timestamp_t *ts, size_t *out_len);

/**
 * Decode the Timestamp of a Sync, Follow_Up or Delay_Req.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p buf or @p out is NULL.
 * @retval -EBADMSG @p len is below PTP_TSMSG_LEN.
 */
int ptp_tsmsg_decode(const uint8_t *buf, size_t len, ptp_timestamp_t *out);

/* ------------------------------------------------------------- announce --- */

/**
 * Encode an Announce. messageLength is written as PTP_ANNOUNCE_LEN.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p buf, @p h or @p a is NULL.
 * @retval -ENOSPC  @p cap is below PTP_ANNOUNCE_LEN.
 */
int ptp_announce_encode(uint8_t *buf, size_t cap, const ptp_hdr_t *h,
			const ptp_announce_t *a, size_t *out_len);

/**
 * Decode the body of an Announce.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p buf or @p out is NULL.
 * @retval -EBADMSG @p len is below PTP_ANNOUNCE_LEN.
 */
int ptp_announce_decode(const uint8_t *buf, size_t len, ptp_announce_t *out);

/* ----------------------------------------------------------- delay_resp --- */

/**
 * Encode a Delay_Resp. messageLength is written as PTP_DELAY_RESP_LEN.
 *
 * @param rx_ts       receiveTimestamp: when the Delay_Req arrived.
 * @param requesting  requestingPortIdentity, copied from the Delay_Req header.
 *
 * @retval 0        Success.
 * @retval -EINVAL  A pointer argument is NULL.
 * @retval -ENOSPC  @p cap is below PTP_DELAY_RESP_LEN.
 */
int ptp_delay_resp_encode(uint8_t *buf, size_t cap, const ptp_hdr_t *h,
			  const ptp_timestamp_t *rx_ts,
			  const ptp_port_id_t *requesting, size_t *out_len);

/**
 * Decode the body of a Delay_Resp. Either output pointer may be NULL.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p buf is NULL.
 * @retval -EBADMSG @p len is below PTP_DELAY_RESP_LEN.
 */
int ptp_delay_resp_decode(const uint8_t *buf, size_t len,
			  ptp_timestamp_t *rx_ts, ptp_port_id_t *requesting);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_PTP_PTP_MSG_H_ */
