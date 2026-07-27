/*
 * STS1000 "Meridian" — core/ubx: u-blox UBX protocol codec.
 *
 * Platform-neutral C11 (ARCHITECTURE.md §4). No dynamic allocation, no globals,
 * no platform headers: every buffer is caller-owned and every parser keeps its
 * whole state in a caller-owned ubx_parser_t. Dependency edge: ubx -> util.
 *
 * Scope: the wire format only. Nothing here knows about the ZED-F9T's lifecycle,
 * UARTs, or timing — that is core/gnssmgr and the Zephyr glue.
 *
 *   Framing        B5 62 | class | id | len(LE16) | payload[len] | CK_A CK_B
 *   Checksum       8-bit Fletcher over class..payload (u-blox ICD "UBX checksum")
 *   Byte order     little-endian throughout (util/bytes.h accessors)
 *
 * Three groups of API:
 *
 *   1. Streaming parser — ubx_parse_byte() consumes the UART byte stream one
 *      byte at a time and reports a validated frame. It never blocks, never
 *      allocates, and resynchronises without losing a byte that could start the
 *      next frame (see ubx_parse_byte()).
 *   2. Typed decoders — ubx_parse_nav_pvt() and friends turn a validated frame
 *      into a portable struct. No packed structs are overlaid on the wire: the
 *      decoders read field by field so alignment, padding and endianness are
 *      all explicit and the host tests are representative of the target.
 *   3. Builders — ubx_frame()/ubx_poll() for arbitrary frames, and the
 *      ubx_valset_* group for the UBX-CFG-VALSET configuration writes that
 *      gnssmgr uses to set up the receiver.
 *
 * Return convention: int, 0 or negative errno-style, except where a byte count
 * is more useful than a bare 0 (ubx_frame(), ubx_valset_end()) and except for
 * the "produced something" reporters (ubx_parse_byte(), the iterators) which
 * return 1. Each function documents its own values.
 */

#ifndef STS1000_CORE_UBX_UBX_H_
#define STS1000_CORE_UBX_UBX_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------- framing -- */

#define UBX_SYNC1 0xB5U
#define UBX_SYNC2 0x62U

/** Bytes a frame costs on the wire beyond its payload: 2 sync, 4 header, 2 ck. */
#define UBX_FRAME_OVERHEAD 8U

/** Largest payload the 16-bit length field can express. */
#define UBX_PAYLOAD_MAX 0xFFFFU

/**
 * Suggested parser payload capacity.
 *
 * Everything this firmware enables fits far below it — the largest is NAV-SAT at
 * 8 + 12·numSvs bytes, ~500 B for a full sky on four constellations. The cap is
 * a parser parameter rather than a constant so a caller with tighter RAM (or a
 * caller that wants to accept a large MON-* dump) can size its own buffer.
 */
#define UBX_PAYLOAD_CAP_DEFAULT 1024U

/* Message classes. */
#define UBX_CLASS_NAV 0x01U
#define UBX_CLASS_ACK 0x05U
#define UBX_CLASS_CFG 0x06U
#define UBX_CLASS_MON 0x0AU
#define UBX_CLASS_TIM 0x0DU

/* Message IDs, by class. */
#define UBX_ID_NAV_PVT    0x07U
#define UBX_ID_NAV_TIMELS 0x26U
#define UBX_ID_NAV_SAT    0x35U
#define UBX_ID_NAV_SVIN   0x3BU
#define UBX_ID_ACK_NAK    0x00U
#define UBX_ID_ACK_ACK    0x01U
#define UBX_ID_CFG_VALSET 0x8AU
#define UBX_ID_MON_RF     0x38U
#define UBX_ID_MON_VER    0x04U
#define UBX_ID_TIM_TP     0x01U

/* Fixed payload lengths of the fixed-length messages this module decodes. */
#define UBX_LEN_NAV_PVT    92U
#define UBX_LEN_NAV_TIMELS 24U
#define UBX_LEN_NAV_SVIN   40U
#define UBX_LEN_TIM_TP     16U
#define UBX_LEN_ACK        2U

/* Variable-length message geometry. */
#define UBX_NAV_SAT_HDR_LEN 8U
#define UBX_NAV_SAT_SV_LEN  12U
#define UBX_MON_RF_HDR_LEN  4U
#define UBX_MON_RF_BLK_LEN  24U

/* ---------------------------------------------------------------- decode -- */

/** A validated frame. @p payload points into the parser's caller-owned buffer. */
typedef struct {
	uint8_t        cls;
	uint8_t        id;
	uint16_t       len;
	const uint8_t *payload;
} ubx_msg_t;

/** Streaming-parser states. Exposed only because ubx_parser_t embeds one. */
typedef enum {
	UBX_PS_SYNC1 = 0,
	UBX_PS_SYNC2,
	UBX_PS_CLASS,
	UBX_PS_ID,
	UBX_PS_LEN_LO,
	UBX_PS_LEN_HI,
	UBX_PS_PAYLOAD,
	UBX_PS_CK_A,
	UBX_PS_CK_B,
} ubx_pstate_t;

/** Parser counters. Diagnostic only; never affects decoding. */
typedef struct {
	uint32_t frames;      /**< frames delivered to the caller */
	uint32_t hunt_bytes;  /**< bytes discarded while hunting for B5 62 */
	uint32_t len_errors;  /**< frames rejected for a length above the cap */
	uint32_t ck_errors;   /**< frames rejected for a checksum mismatch */
} ubx_stats_t;

/** Streaming parser. All fields are private; use the ubx_parser_* functions. */
typedef struct {
	uint8_t    *buf;     /**< caller-owned payload store, cap bytes */
	uint16_t    cap;     /**< largest payload accepted */
	uint8_t     state;   /**< ubx_pstate_t */
	uint8_t     cls;
	uint8_t     id;
	uint16_t    len;
	uint16_t    idx;
	uint8_t     ck_a;    /**< running Fletcher A over class..payload */
	uint8_t     ck_b;    /**< running Fletcher B */
	uint8_t     rx_ck_a; /**< CK_A as received, held until CK_B arrives */
	bool        complete;
	ubx_stats_t stats;
} ubx_parser_t;

/**
 * Bind a parser to its payload buffer and reset it.
 *
 * @param p    Parser.
 * @param buf  Payload store of @p cap bytes (UBX_PAYLOAD_CAP_DEFAULT is a
 *             sensible size for this receiver's message set).
 * @param cap  Largest payload accepted; a frame declaring more is rejected with
 *             -EMSGSIZE rather than overrunning @p buf.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p p or @p buf is NULL, or @p cap is 0.
 */
int ubx_parser_init(ubx_parser_t *p, uint8_t *buf, uint16_t cap);

/** Drop any partial frame and return to hunting for the sync pair. */
void ubx_parser_reset(ubx_parser_t *p);

/**
 * Feed one received byte.
 *
 * Resynchronisation: a byte that fails the sync pattern, the length bound or
 * the checksum is re-examined once as a possible new UBX_SYNC1, so the stream
 * "B5 B5 62 ..." and a frame that begins immediately after a corrupt one are
 * both picked up without loss. A frame truncated mid-payload is inherently only
 * detectable at its checksum — that is what a length-prefixed framing costs, and
 * the bytes of the following frame consumed as payload are the price of the
 * one bad frame, not of every frame after it.
 *
 * @retval 1         A frame completed and validated; read it with
 *                   ubx_parser_msg() before the next call.
 * @retval 0         Byte consumed, no frame yet (includes non-frame garbage).
 * @retval -EINVAL   @p p is NULL or uninitialised.
 * @retval -EMSGSIZE Declared payload length exceeds the parser's cap; the frame
 *                   is abandoned.
 * @retval -EBADMSG  Checksum mismatch; the frame is discarded.
 */
int ubx_parse_byte(ubx_parser_t *p, uint8_t b);

/**
 * View the frame reported by the last ubx_parse_byte() == 1.
 *
 * The payload pointer aliases the parser's buffer and is valid only until the
 * next ubx_parse_byte() call.
 *
 * @retval 0        @p out filled.
 * @retval -EINVAL  NULL argument.
 * @retval -EAGAIN  No completed frame is pending.
 */
int ubx_parser_msg(const ubx_parser_t *p, ubx_msg_t *out);

/** Copy the parser's counters. @retval 0 / -EINVAL. */
int ubx_parser_stats(const ubx_parser_t *p, ubx_stats_t *out);

/** True when @p m is the given class/id. NULL-safe (returns false). */
bool ubx_msg_is(const ubx_msg_t *m, uint8_t cls, uint8_t id);

/* ---------------------------------------------------------------- encode -- */

/**
 * Compute the UBX 8-bit Fletcher checksum.
 *
 * @param data  First byte of the checksummed range — the class byte, i.e. the
 *              frame *without* its two sync bytes.
 * @param len   Bytes covered: 4 header bytes + payload.
 * @param ck_a  Out: CK_A. @param ck_b Out: CK_B.
 */
void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b);

/**
 * Build a complete frame.
 *
 * @param buf      Destination.
 * @param cap      Bytes available at @p buf.
 * @param cls,id   Message class and id.
 * @param payload  Payload; may be NULL only when @p len is 0.
 * @param len      Payload bytes.
 *
 * @return         Frame length in bytes (len + UBX_FRAME_OVERHEAD) on success.
 * @retval -EINVAL   @p buf is NULL, or @p payload is NULL with a non-zero @p len.
 * @retval -EMSGSIZE @p len exceeds UBX_PAYLOAD_MAX.
 * @retval -ENOSPC   @p cap is too small for the frame.
 */
int ubx_frame(uint8_t *buf, size_t cap, uint8_t cls, uint8_t id,
	      const uint8_t *payload, size_t len);

/** Build a zero-length poll request for @p cls / @p id. Same returns as ubx_frame(). */
int ubx_poll(uint8_t *buf, size_t cap, uint8_t cls, uint8_t id);

/* ------------------------------------------------------- NAV-PVT (01 07) -- */

/** UBX-NAV-PVT fixType. */
enum {
	UBX_FIX_NONE      = 0,
	UBX_FIX_DR_ONLY   = 1,
	UBX_FIX_2D        = 2,
	UBX_FIX_3D        = 3,
	UBX_FIX_GNSS_DR   = 4,
	UBX_FIX_TIME_ONLY = 5,
};

/**
 * UBX-NAV-PVT, 92-byte payload.
 *
 * Velocity, heading and magnetic-declination fields are deliberately not
 * decoded: this is a stationary timing receiver and nothing in the firmware
 * consumes them. Add them here, not with a second decoder, if that changes.
 */
typedef struct {
	uint32_t itow_ms;        /**< GPS time of week of the nav epoch */
	uint16_t year;
	uint8_t  month;          /**< 1..12 */
	uint8_t  day;            /**< 1..31 */
	uint8_t  hour;           /**< 0..23 */
	uint8_t  min;            /**< 0..59 */
	uint8_t  sec;            /**< 0..60 (60 during a leap insertion) */
	uint8_t  valid;          /**< raw validity bitfield */
	bool     valid_date;     /**< valid bit 0 */
	bool     valid_time;     /**< valid bit 1 */
	bool     fully_resolved; /**< valid bit 2 — UTC offset known, no ambiguity */
	bool     valid_mag;      /**< valid bit 3 */
	uint32_t tacc_ns;        /**< time accuracy estimate */
	int32_t  nano_ns;        /**< fraction of second, -1e9..1e9 */
	uint8_t  fix_type;       /**< UBX_FIX_* */
	uint8_t  flags;          /**< raw flags byte */
	bool     gnss_fix_ok;    /**< flags bit 0 — fix passes the DOP/accuracy mask */
	bool     diff_soln;      /**< flags bit 1 */
	uint8_t  carr_soln;      /**< flags bits 6..7: 0 none, 1 float, 2 fixed */
	uint8_t  flags2;         /**< raw */
	uint8_t  num_sv;         /**< satellites used in the nav solution */
	int32_t  lon_1e7;        /**< 1e-7 deg */
	int32_t  lat_1e7;        /**< 1e-7 deg */
	int32_t  height_mm;      /**< above ellipsoid */
	int32_t  hmsl_mm;        /**< above mean sea level */
	uint32_t hacc_mm;
	uint32_t vacc_mm;
	uint16_t pdop;           /**< 0.01 */
	uint16_t flags3;         /**< raw */
	bool     invalid_llh;    /**< flags3 bit 0 */
} ubx_nav_pvt_t;

/**
 * Decode UBX-NAV-PVT.
 *
 * @retval 0        Decoded.
 * @retval -EINVAL  NULL argument.
 * @retval -ENOMSG  @p m is not NAV-PVT.
 * @retval -EBADMSG Payload length is not UBX_LEN_NAV_PVT.
 */
int ubx_parse_nav_pvt(const ubx_msg_t *m, ubx_nav_pvt_t *out);

/* ------------------------------------------------------- NAV-SAT (01 35) -- */

/** One satellite record from UBX-NAV-SAT. */
typedef struct {
	uint8_t  gnss_id;   /**< 0 GPS, 1 SBAS, 2 Galileo, 3 BeiDou, 5 QZSS, 6 GLONASS */
	uint8_t  sv_id;
	uint8_t  cno_dbhz;
	int8_t   elev_deg;  /**< -90..90 */
	int16_t  azim_deg;  /**< 0..360 */
	int16_t  pr_res;    /**< pseudorange residual, 0.1 m */
	uint32_t flags;     /**< raw */
	uint8_t  quality;   /**< flags bits 0..2, signal quality indicator */
	bool     used;      /**< flags bit 3 — used in the nav solution */
	uint8_t  health;    /**< flags bits 4..5: 0 unknown, 1 healthy, 2 unhealthy */
} ubx_nav_sat_sv_t;

/** UBX-NAV-SAT iterator. Avoids a several-hundred-byte flat struct. */
typedef struct {
	uint32_t       itow_ms;
	uint8_t        version;
	uint8_t        num_svs;
	uint8_t        idx;
	const uint8_t *rec;  /**< next record, or NULL when exhausted */
} ubx_nav_sat_iter_t;

/**
 * Start iterating a UBX-NAV-SAT frame.
 *
 * @retval 0        @p it is armed; num_svs may be 0.
 * @retval -EINVAL  NULL argument.
 * @retval -ENOMSG  @p m is not NAV-SAT.
 * @retval -EBADMSG Length is not 8 + 12·numSvs.
 */
int ubx_nav_sat_begin(const ubx_msg_t *m, ubx_nav_sat_iter_t *it);

/**
 * Produce the next satellite record.
 *
 * @retval 1        @p sv filled.
 * @retval 0        Iteration complete.
 * @retval -EINVAL  NULL argument.
 */
int ubx_nav_sat_next(ubx_nav_sat_iter_t *it, ubx_nav_sat_sv_t *sv);

/* ---------------------------------------------------- NAV-TIMELS (01 26) -- */

/** UBX-NAV-TIMELS, 24-byte payload. Source of the leap-second schedule. */
typedef struct {
	uint32_t itow_ms;
	uint8_t  version;
	uint8_t  src_of_curr_ls;      /**< 0 default, 1 GLONASS, 2 GPS, ... 255 unknown */
	int8_t   curr_ls;             /**< current GPS-UTC leap seconds */
	uint8_t  src_of_ls_change;    /**< 0 none, 2 GPS, 3 SBAS, 4 BeiDou, 5 Galileo, 6 GLONASS */
	int8_t   ls_change;           /**< +1 insertion, -1 deletion, 0 none scheduled */
	int32_t  time_to_ls_event_s;  /**< seconds until the event; negative once past */
	uint16_t date_of_ls_gps_wn;   /**< GPS week number of the event */
	uint16_t date_of_ls_gps_dn;   /**< GPS day of week of the event, 1..7 */
	uint8_t  valid;               /**< raw */
	bool     valid_curr_ls;       /**< valid bit 0 */
	bool     valid_time_to_ls_event; /**< valid bit 1 */
} ubx_nav_timels_t;

/** Decode UBX-NAV-TIMELS. Returns as ubx_parse_nav_pvt(). */
int ubx_parse_nav_timels(const ubx_msg_t *m, ubx_nav_timels_t *out);

/* ------------------------------------------------------ NAV-SVIN (01 3B) -- */

/** UBX-NAV-SVIN, 40-byte payload. Survey-in progress and result. */
typedef struct {
	uint8_t  version;
	uint32_t itow_ms;
	uint32_t dur_s;            /**< elapsed survey-in observation time */
	int32_t  mean_x_cm;
	int32_t  mean_y_cm;
	int32_t  mean_z_cm;
	int8_t   mean_x_hp;        /**< 0.1 mm, -99..99 */
	int8_t   mean_y_hp;
	int8_t   mean_z_hp;
	uint32_t mean_acc_0p1mm;   /**< 3D position accuracy estimate */
	uint32_t obs;              /**< observations used */
	bool     valid;            /**< survey-in position is valid */
	bool     active;           /**< survey-in still running */
} ubx_nav_svin_t;

/** Decode UBX-NAV-SVIN. Returns as ubx_parse_nav_pvt(). */
int ubx_parse_nav_svin(const ubx_msg_t *m, ubx_nav_svin_t *out);

/* -------------------------------------------------------- TIM-TP (0D 01) -- */

/**
 * UBX-TIM-TP, 16-byte payload.
 *
 * The message describes the time pulse *about to be produced* — the receiver
 * emits it in the second before the pulse it refers to, so @p tow_ms is the
 * time-of-week of the NEXT pulse, not the last one. gnssmgr tags its cached
 * copy with that target ToW so the discipline glue pairs a captured edge with
 * the qErr that actually belongs to it.
 */
typedef struct {
	uint32_t tow_ms;         /**< ToW of the target pulse, ms */
	uint32_t tow_sub_ms;     /**< sub-ms remainder, 2^-32 ms */
	int32_t  qerr_ps;        /**< quantisation error, picoseconds — SIGNED */
	uint16_t week;           /**< GPS week number of the target pulse */
	uint8_t  flags;          /**< raw */
	bool     time_base_utc;  /**< flags bit 0: 0 = GNSS timebase, 1 = UTC */
	bool     utc_available;  /**< flags bit 1 */
	uint8_t  raim;           /**< flags bits 2..3: 0 n/a, 1 inactive, 2 active */
	bool     qerr_invalid;   /**< flags bit 4 — qErr must not be applied */
	uint8_t  ref_info;       /**< raw */
	uint8_t  time_ref_gnss;  /**< ref_info bits 0..3 */
	uint8_t  utc_standard;   /**< ref_info bits 4..7 */
} ubx_tim_tp_t;

/** Decode UBX-TIM-TP. Returns as ubx_parse_nav_pvt(). */
int ubx_parse_tim_tp(const ubx_msg_t *m, ubx_tim_tp_t *out);

/* -------------------------------------------------------- MON-RF (0A 38) -- */

/** MON-RF antStatus. */
enum {
	UBX_ANT_STATUS_INIT     = 0,
	UBX_ANT_STATUS_DONTKNOW = 1,
	UBX_ANT_STATUS_OK       = 2,
	UBX_ANT_STATUS_SHORT    = 3,
	UBX_ANT_STATUS_OPEN     = 4,
};

/** MON-RF antPower. */
enum {
	UBX_ANT_POWER_OFF      = 0,
	UBX_ANT_POWER_ON       = 1,
	UBX_ANT_POWER_DONTKNOW = 2,
};

/** MON-RF jammingState (flags bits 0..1). */
enum {
	UBX_JAMMING_UNKNOWN  = 0,
	UBX_JAMMING_OK       = 1,
	UBX_JAMMING_WARNING  = 2,
	UBX_JAMMING_CRITICAL = 3,
};

/** One RF block from UBX-MON-RF. */
typedef struct {
	uint8_t  block_id;
	uint8_t  flags;         /**< raw */
	uint8_t  jamming_state; /**< flags bits 0..1, UBX_JAMMING_* */
	uint8_t  ant_status;    /**< UBX_ANT_STATUS_* */
	uint8_t  ant_power;     /**< UBX_ANT_POWER_* */
	uint32_t post_status;
	uint16_t noise_per_ms;
	uint16_t agc_cnt;       /**< 0..8191 */
	uint8_t  jam_ind;       /**< 0..255 CW jamming indicator */
	int8_t   ofs_i;
	uint8_t  mag_i;
	int8_t   ofs_q;
	uint8_t  mag_q;
} ubx_mon_rf_block_t;

/** UBX-MON-RF iterator. */
typedef struct {
	uint8_t        version;
	uint8_t        n_blocks;
	uint8_t        idx;
	const uint8_t *rec;
} ubx_mon_rf_iter_t;

/** Start iterating a UBX-MON-RF frame. Returns as ubx_nav_sat_begin(). */
int ubx_mon_rf_begin(const ubx_msg_t *m, ubx_mon_rf_iter_t *it);

/** Produce the next RF block. Returns as ubx_nav_sat_next(). */
int ubx_mon_rf_next(ubx_mon_rf_iter_t *it, ubx_mon_rf_block_t *blk);

/* ------------------------------------------------------ MON-VER (0A 04) -- */

/*
 * UBX-MON-VER payload: swVersion[30] and hwVersion[10], both NUL-padded ASCII,
 * followed by zero or more 30-octet extension strings. Poll it with a
 * zero-length request; the receiver answers with its own lengths, so the number
 * of extensions is (payload_len - 40) / 30.
 */
#define UBX_MON_VER_SW_LEN  30U
#define UBX_MON_VER_HW_LEN  10U
#define UBX_MON_VER_EXT_LEN 30U
#define UBX_MON_VER_MIN_LEN (UBX_MON_VER_SW_LEN + UBX_MON_VER_HW_LEN)

/** Extension strings kept. A ZED-F9T reports around five. */
#define UBX_MON_VER_MAX_EXT 8U

/**
 * Decoded UBX-MON-VER.
 *
 * Every string is NUL-terminated here even when the wire field used all its
 * octets, so the fields are safe to pass to string functions. This is the
 * receiver's own statement of what firmware it is running, and therefore the
 * post-update verification step (see core/fwupd/ubx_fwupd.h).
 */
typedef struct {
	char    sw_version[UBX_MON_VER_SW_LEN + 1U];
	char    hw_version[UBX_MON_VER_HW_LEN + 1U];
	uint8_t n_ext;
	char    ext[UBX_MON_VER_MAX_EXT][UBX_MON_VER_EXT_LEN + 1U];
} ubx_mon_ver_t;

/**
 * Decode UBX-MON-VER.
 *
 * @retval 0        Decoded.
 * @retval -EINVAL  NULL argument.
 * @retval -ENOMSG  @p m is not UBX-MON-VER.
 * @retval -EBADMSG Payload shorter than swVersion + hwVersion, or a trailing
 *                  fragment that is not a whole extension string.
 */
int ubx_parse_mon_ver(const ubx_msg_t *m, ubx_mon_ver_t *out);

/**
 * Find the value of a "KEY=VALUE" extension string, e.g. "FWVER".
 *
 * The ZED-F9T reports its firmware version as an extension string of the form
 * "FWVER=TIM 2.20", which is the field a firmware update has to move. Returns a
 * pointer into @p v, or NULL when no extension has that key.
 */
const char *ubx_mon_ver_ext(const ubx_mon_ver_t *v, const char *key);

/* ------------------------------------------------- ACK-ACK / ACK-NAK (05) -- */

/** UBX-ACK-ACK / UBX-ACK-NAK, 2-byte payload. */
typedef struct {
	bool    ack;     /**< true = ACK-ACK, false = ACK-NAK */
	uint8_t cls_id;  /**< class of the acknowledged message */
	uint8_t msg_id;  /**< id of the acknowledged message */
} ubx_ack_t;

/**
 * Decode UBX-ACK-ACK or UBX-ACK-NAK.
 *
 * @retval 0        Decoded.
 * @retval -EINVAL  NULL argument.
 * @retval -ENOMSG  @p m is neither ACK-ACK nor ACK-NAK.
 * @retval -EBADMSG Payload length is not UBX_LEN_ACK.
 */
int ubx_parse_ack(const ubx_msg_t *m, ubx_ack_t *out);

/* ---------------------------------------------------------- CFG-VALSET -- */

/*
 * ===========================================================================
 * Configuration key IDs — the ONE table. Do not scatter these.
 *
 * u-blox generation-9 configuration interface (ZED-F9T interface description,
 * UBX-19003606). Key ID layout:
 *
 *     bit 31      reserved (0)
 *     bits 30..28 storage size id
 *     bits 27..16 group id
 *     bits 15..0  item id
 *
 *     size id 0x1  one bit    (L)          -> 1 byte on the wire, 0 or 1
 *     size id 0x2  one byte   (U1/I1/E1)   -> 1 byte
 *     size id 0x3  two bytes  (U2/I2/E2)   -> 2 bytes
 *     size id 0x4  four bytes (U4/I4/R4)   -> 4 bytes
 *     size id 0x5  eight bytes(U8/I8/R8)   -> 8 bytes
 *
 * ubx_cfg_key_bytes() decodes that size field and every ubx_valset_add_*()
 * refuses a key whose declared width does not match the setter, so a transposed
 * digit below fails loudly at the call site instead of silently emitting a
 * malformed VALSET.
 *
 * VERIFY status. Four IDs are anchors, quoted verbatim in the Wave-2a work
 * order and cross-checked against the encoding above:
 *
 *     CFG-UART1-BAUDRATE             0x40520001
 *     CFG-UART1OUTPROT-NMEA          0x10740002
 *     CFG-MSGOUT-UBX_NAV_PVT_UART1   0x20910007
 *     CFG-TP-TP1_ENA                 0x10050007
 *
 * Every other ID is reproduced from the generation-9 interface description and
 * is marked VERIFY until checked against the ICD PDF or a live receiver. The
 * failure mode is safe and loud rather than silent: a wrong key makes the F9T
 * answer UBX-ACK-NAK, which gnssmgr retries and then escalates to the
 * CONFIG_FAILED alarm. tests/host/test_ubx.c additionally asserts the group and
 * size field of every entry, which catches a transposed digit without the ICD.
 * ===========================================================================
 */

/* CFG-UART1 (group 0x052) — baud rate is owned by the glue, which must change
 * the STM32 side in step; the rest of the port setup is gnssmgr's. */
#define UBX_CFG_UART1_BAUDRATE      0x40520001UL /* U4, anchor */
#define UBX_CFG_UART1_ENABLED       0x10520005UL /* L,  VERIFY */

/* CFG-UART1INPROT (0x073) / CFG-UART1OUTPROT (0x074) — UBX only, NMEA off. */
#define UBX_CFG_UART1INPROT_UBX     0x10730001UL /* L,  VERIFY */
#define UBX_CFG_UART1INPROT_NMEA    0x10730002UL /* L,  VERIFY */
#define UBX_CFG_UART1INPROT_RTCM3X  0x10730004UL /* L,  VERIFY */
#define UBX_CFG_UART1OUTPROT_UBX    0x10740001UL /* L,  VERIFY */
#define UBX_CFG_UART1OUTPROT_NMEA   0x10740002UL /* L,  anchor */
#define UBX_CFG_UART1OUTPROT_RTCM3X 0x10740004UL /* L,  VERIFY */

/* CFG-MSGOUT (group 0x091) — per-port output rate in navigation epochs. */
#define UBX_CFG_MSGOUT_NAV_PVT_UART1    0x20910007UL /* U1, anchor */
#define UBX_CFG_MSGOUT_NAV_SAT_UART1    0x20910016UL /* U1, VERIFY */
#define UBX_CFG_MSGOUT_NAV_TIMELS_UART1 0x20910061UL /* U1, VERIFY */
#define UBX_CFG_MSGOUT_NAV_SVIN_UART1   0x20910089UL /* U1, VERIFY */
#define UBX_CFG_MSGOUT_NAV_SIG_UART1    0x20910346UL /* U1, VERIFY (unused, Wave 2a) */
#define UBX_CFG_MSGOUT_TIM_TP_UART1     0x2091017EUL /* U1, VERIFY */
#define UBX_CFG_MSGOUT_MON_RF_UART1     0x2091035AUL /* U1, VERIFY */
#define UBX_CFG_MSGOUT_MON_HW_UART1     0x209101B5UL /* U1, VERIFY (unused, Wave 2a) */

/* CFG-RATE (group 0x021). */
#define UBX_CFG_RATE_MEAS    0x30210001UL /* U2, ms,     VERIFY */
#define UBX_CFG_RATE_NAV     0x30210002UL /* U2, cycles, VERIFY */
#define UBX_CFG_RATE_TIMEREF 0x20210003UL /* E1,         VERIFY */

/* CFG-NAVSPG (group 0x011). */
#define UBX_CFG_NAVSPG_DYNMODEL      0x20110021UL /* E1, VERIFY */
#define UBX_CFG_NAVSPG_INFIL_MINELEV 0x201100A4UL /* I1, deg, VERIFY */

/** CFG-NAVSPG-DYNMODEL values (only the one this board uses is named). */
#define UBX_DYNMODEL_STATIONARY 2U

/* CFG-SIGNAL (group 0x031) — constellation enables, spec §3.7. */
#define UBX_CFG_SIGNAL_GPS_ENA  0x1031001FUL /* L, VERIFY */
#define UBX_CFG_SIGNAL_SBAS_ENA 0x10310020UL /* L, VERIFY */
#define UBX_CFG_SIGNAL_GAL_ENA  0x10310021UL /* L, VERIFY */
#define UBX_CFG_SIGNAL_BDS_ENA  0x10310022UL /* L, VERIFY */
#define UBX_CFG_SIGNAL_QZSS_ENA 0x10310024UL /* L, VERIFY */
#define UBX_CFG_SIGNAL_GLO_ENA  0x10310025UL /* L, VERIFY */

/* CFG-TP (group 0x005) — TIMEPULSE 1 (PA0/TIM2) and 2 (PC6/TIM3). */
#define UBX_CFG_TP_PULSE_DEF        0x20050023UL /* E1, VERIFY */
#define UBX_CFG_TP_PULSE_LENGTH_DEF 0x20050030UL /* E1, VERIFY */
#define UBX_CFG_TP_ANT_CABLEDELAY   0x30050001UL /* I2, ns, VERIFY */
#define UBX_CFG_TP_PERIOD_TP1       0x40050002UL /* U4, us, VERIFY */
#define UBX_CFG_TP_PERIOD_LOCK_TP1  0x40050003UL /* U4, us, VERIFY */
#define UBX_CFG_TP_LEN_TP1          0x40050004UL /* U4, us, VERIFY */
#define UBX_CFG_TP_LEN_LOCK_TP1     0x40050005UL /* U4, us, VERIFY */
#define UBX_CFG_TP_USER_DELAY_TP1   0x40050006UL /* I4, ns, VERIFY */
#define UBX_CFG_TP_TP1_ENA          0x10050007UL /* L,  anchor */
#define UBX_CFG_TP_SYNC_GNSS_TP1    0x10050008UL /* L,  VERIFY */
#define UBX_CFG_TP_USE_LOCKED_TP1   0x10050009UL /* L,  VERIFY */
#define UBX_CFG_TP_ALIGN_TO_TOW_TP1 0x1005000AUL /* L,  VERIFY */
#define UBX_CFG_TP_POL_TP1          0x1005000BUL /* L,  VERIFY */
#define UBX_CFG_TP_TIMEGRID_TP1     0x2005000CUL /* E1, VERIFY */
#define UBX_CFG_TP_PERIOD_TP2       0x4005000DUL /* U4, us, VERIFY */
#define UBX_CFG_TP_PERIOD_LOCK_TP2  0x4005000EUL /* U4, us, VERIFY */
#define UBX_CFG_TP_LEN_TP2          0x4005000FUL /* U4, us, VERIFY */
#define UBX_CFG_TP_LEN_LOCK_TP2     0x40050010UL /* U4, us, VERIFY */
#define UBX_CFG_TP_USER_DELAY_TP2   0x40050011UL /* I4, ns, VERIFY */
#define UBX_CFG_TP_TP2_ENA          0x10050012UL /* L,  VERIFY */
#define UBX_CFG_TP_SYNC_GNSS_TP2    0x10050013UL /* L,  VERIFY */
#define UBX_CFG_TP_USE_LOCKED_TP2   0x10050014UL /* L,  VERIFY */
#define UBX_CFG_TP_ALIGN_TO_TOW_TP2 0x10050015UL /* L,  VERIFY */
#define UBX_CFG_TP_POL_TP2          0x10050016UL /* L,  VERIFY */
#define UBX_CFG_TP_TIMEGRID_TP2     0x20050017UL /* E1, VERIFY */

/** CFG-TP-PULSE_DEF values. */
#define UBX_TP_PULSE_DEF_PERIOD 0U
#define UBX_TP_PULSE_DEF_FREQ   1U
/** CFG-TP-PULSE_LENGTH_DEF values. */
#define UBX_TP_LENGTH_DEF_RATIO  0U
#define UBX_TP_LENGTH_DEF_LENGTH 1U
/** CFG-TP-TIMEGRID_TPx values. */
#define UBX_TP_TIMEGRID_UTC 0U
#define UBX_TP_TIMEGRID_GPS 1U

/* CFG-TXREADY (group 0x0A2) — remaps F9T pin 19 (GEOFENCE_STAT by default) to
 * TX_READY so PD5/EXTI5 can wake the parser. Interface ref §10 caution 2. */
#define UBX_CFG_TXREADY_ENABLED   0x10A20001UL /* L,  VERIFY */
#define UBX_CFG_TXREADY_POLARITY  0x10A20002UL /* L,  VERIFY (0 = active high) */
#define UBX_CFG_TXREADY_PIN       0x20A20003UL /* U1, VERIFY */
#define UBX_CFG_TXREADY_THRESHOLD 0x30A20004UL /* U2, ×8 bytes, VERIFY */
#define UBX_CFG_TXREADY_INTERFACE 0x20A20005UL /* E1, VERIFY */

/** CFG-TXREADY-INTERFACE values. */
#define UBX_TXREADY_IF_I2C   0U
#define UBX_TXREADY_IF_SPI   1U
#define UBX_TXREADY_IF_UART1 2U
#define UBX_TXREADY_IF_UART2 3U

/* CFG-TMODE (group 0x003) — survey-in / fixed-position timing mode. */
#define UBX_CFG_TMODE_MODE           0x20030001UL /* E1, VERIFY */
#define UBX_CFG_TMODE_POS_TYPE       0x20030002UL /* E1, VERIFY */
#define UBX_CFG_TMODE_ECEF_X         0x40030003UL /* I4, cm,     VERIFY */
#define UBX_CFG_TMODE_ECEF_Y         0x40030004UL /* I4, cm,     VERIFY */
#define UBX_CFG_TMODE_ECEF_Z         0x40030005UL /* I4, cm,     VERIFY */
#define UBX_CFG_TMODE_ECEF_X_HP      0x20030006UL /* I1, 0.1 mm, VERIFY */
#define UBX_CFG_TMODE_ECEF_Y_HP      0x20030007UL /* I1, 0.1 mm, VERIFY */
#define UBX_CFG_TMODE_ECEF_Z_HP      0x20030008UL /* I1, 0.1 mm, VERIFY */
#define UBX_CFG_TMODE_FIXED_POS_ACC  0x4003000FUL /* U4, 0.1 mm, VERIFY */
#define UBX_CFG_TMODE_SVIN_MIN_DUR   0x40030010UL /* U4, s,      VERIFY */
#define UBX_CFG_TMODE_SVIN_ACC_LIMIT 0x40030011UL /* U4, 0.1 mm, VERIFY */

/** CFG-TMODE-MODE values. */
#define UBX_TMODE_DISABLED  0U
#define UBX_TMODE_SURVEY_IN 1U
#define UBX_TMODE_FIXED     2U
/** CFG-TMODE-POS_TYPE values. */
#define UBX_TMODE_POS_ECEF 0U
#define UBX_TMODE_POS_LLH  1U

/* VALSET layer bitmap. */
#define UBX_CFG_LAYER_RAM   0x01U
#define UBX_CFG_LAYER_BBR   0x02U
#define UBX_CFG_LAYER_FLASH 0x04U
/** Persist everywhere — spec §3.7 requires config to survive a power cycle. */
#define UBX_CFG_LAYER_ALL (UBX_CFG_LAYER_RAM | UBX_CFG_LAYER_BBR | UBX_CFG_LAYER_FLASH)

/** VALSET payload prefix: version, layers, 2 reserved. */
#define UBX_VALSET_HDR_LEN 4U

/**
 * Wire width of the value belonging to @p key, decoded from its size field.
 *
 * @retval 1,2,4,8  Value bytes.
 * @retval -EINVAL  The size field is not a defined storage size.
 */
int ubx_cfg_key_bytes(uint32_t key);

/** Storage-size id of @p key (1..5), or -EINVAL. Distinguishes L from U1. */
int ubx_cfg_key_size_id(uint32_t key);

/** Group id of @p key, 0..0xFFF. */
uint16_t ubx_cfg_key_group(uint32_t key);

/**
 * Transaction-less UBX-CFG-VALSET builder.
 *
 * The frame is assembled in place in the caller's buffer, so there is no second
 * copy and no payload staging area. Errors are latched: the first failure is
 * remembered, later add_* calls become no-ops, and ubx_valset_end() reports it.
 * That lets a builder emit a long key list without an if() per key.
 */
typedef struct {
	uint8_t *buf;
	size_t   cap;
	size_t   len;    /**< bytes written so far, including the frame header */
	uint16_t items;  /**< key/value pairs added */
	int      err;    /**< first latched error, 0 while healthy */
	bool     open;   /**< begun and not yet ended */
} ubx_valset_t;

/**
 * Begin a VALSET frame.
 *
 * @param v       Builder state.
 * @param buf     Destination for the whole frame.
 * @param cap     Bytes available at @p buf.
 * @param layers  Bitmap of UBX_CFG_LAYER_*; at least one bit must be set.
 *
 * @retval 0        Ready for add_*.
 * @retval -EINVAL  NULL argument, or @p layers has no known bit set.
 * @retval -ENOSPC  @p cap cannot hold even an empty VALSET frame.
 */
int ubx_valset_begin(ubx_valset_t *v, uint8_t *buf, size_t cap, uint8_t layers);

/**
 * Append a one-byte value.
 *
 * @retval 0         Appended.
 * @retval -EINVAL   NULL @p v, builder not open, or the key is not 1 byte wide.
 * @retval -ENOSPC   Buffer full.
 * @retval <0        A previously latched error, re-reported.
 */
int ubx_valset_add_u1(ubx_valset_t *v, uint32_t key, uint8_t val);

/** Append a two-byte value. Returns as ubx_valset_add_u1(). */
int ubx_valset_add_u2(ubx_valset_t *v, uint32_t key, uint16_t val);

/** Append a four-byte value. Returns as ubx_valset_add_u1(). */
int ubx_valset_add_u4(ubx_valset_t *v, uint32_t key, uint32_t val);

/** Append an eight-byte value. Returns as ubx_valset_add_u1(). */
int ubx_valset_add_u8(ubx_valset_t *v, uint32_t key, uint64_t val);

/** Append a signed one-byte value (I1). Returns as ubx_valset_add_u1(). */
int ubx_valset_add_i1(ubx_valset_t *v, uint32_t key, int8_t val);

/** Append a signed two-byte value (I2). Returns as ubx_valset_add_u1(). */
int ubx_valset_add_i2(ubx_valset_t *v, uint32_t key, int16_t val);

/** Append a signed four-byte value (I4). Returns as ubx_valset_add_u1(). */
int ubx_valset_add_i4(ubx_valset_t *v, uint32_t key, int32_t val);

/**
 * Append a boolean (L) value.
 *
 * Stricter than add_u1(): the key must have storage size id 1 (one bit), so a
 * U1 key cannot be set from a bool by accident.
 */
int ubx_valset_add_bool(ubx_valset_t *v, uint32_t key, bool val);

/**
 * Close the frame: patch the length field and append the checksum.
 *
 * @return         Total frame length in bytes.
 * @retval -EINVAL The builder is NULL or was never begun.
 * @retval <0      The first latched add_* error.
 */
int ubx_valset_end(ubx_valset_t *v);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_UBX_UBX_H_ */
