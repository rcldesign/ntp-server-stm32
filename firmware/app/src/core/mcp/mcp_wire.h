/*
 * STS1000 "Meridian" — core/mcp: Meridian Console Protocol wire format.
 *
 * Platform-neutral C11. This header is the normative statement of the byte
 * layout defined in ARCHITECTURE.md §7; `tools/meridian_ctl.py` mirrors it and
 * `tests/host/test_mcp.c` pins it with byte vectors. Change nothing here
 * without changing all three.
 *
 * Transport
 * ---------
 * A frame is COBS-encoded (core/util/cobs.h) and terminated by a single 0x00
 * delimiter, so the channel resynchronises after any corruption. The decoded
 * frame is:
 *
 *   offset  size  field
 *   0       1     ver     protocol version, MCP_VER
 *   1       1     type    0 REQ, 1 RSP, 2 EVT
 *   2       1     cmd     command code (mcp_cmd_t)
 *   3       1     flags   per-command; EVT uses bit0 as the subscription id
 *   4       2     seq     little-endian; RSP echoes the REQ's seq
 *   6       2     len     little-endian payload length
 *   8       len   payload
 *   8+len   4     crc32   little-endian CRC-32/ISO-HDLC over bytes 0..8+len-1
 *
 * Every multibyte field in this protocol is little-endian, payloads included.
 *
 * A RSP payload always begins with a u8 status (mcp_err_t); the per-command
 * layouts below describe what follows a status of MCP_OK. On any other status
 * the payload is the status byte alone — no partial results — with one
 * documented exception: FW_DATA answering MCP_ERR_OFFSET still carries the u32
 * next-expected offset, because that value is exactly what lets the tool
 * recover without restarting the transfer.
 *
 * Frames that fail COBS decoding, the CRC, or the length check are dropped
 * without a response: nothing in a frame that failed its own integrity check
 * is trustworthy enough to echo, including its sequence number.
 */

#ifndef STS1000_CORE_MCP_MCP_WIRE_H_
#define STS1000_CORE_MCP_MCP_WIRE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Protocol version carried in byte 0. */
#define MCP_VER 1U

/** Fixed header bytes ahead of the payload. */
#define MCP_HDR_LEN 8U
/** Trailer bytes after the payload. */
#define MCP_CRC_LEN 4U
/** Largest payload, per ARCHITECTURE.md §7. */
#define MCP_MAX_PAYLOAD 1040U
/** Largest decoded frame: header + payload + CRC. */
#define MCP_MAX_FRAME (MCP_HDR_LEN + MCP_MAX_PAYLOAD + MCP_CRC_LEN)
/** Smallest legal decoded frame: an empty payload. */
#define MCP_MIN_FRAME (MCP_HDR_LEN + MCP_CRC_LEN)

/** Frame type (byte 1). */
typedef enum {
	MCP_T_REQ = 0,
	MCP_T_RSP = 1,
	MCP_T_EVT = 2,
} mcp_type_t;

/** Command codes (byte 2). */
typedef enum {
	MCP_CMD_HELLO         = 0x01,
	MCP_CMD_REBOOT        = 0x02,
	MCP_CMD_AUTH          = 0x08,
	MCP_CMD_CFG_LIST      = 0x10,
	MCP_CMD_CFG_GET       = 0x11,
	MCP_CMD_CFG_SET       = 0x12,
	MCP_CMD_CFG_COMMIT    = 0x13,
	MCP_CMD_CFG_REVERT    = 0x14,
	MCP_CMD_CFG_EXPORT    = 0x15,
	MCP_CMD_CFG_IMPORT    = 0x16,
	MCP_CMD_FACTORY_RESET = 0x17,
	MCP_CMD_STATUS_GET    = 0x20,
	MCP_CMD_TELEM_SUB     = 0x21,
	MCP_CMD_TELEM_UNSUB   = 0x22,
	MCP_CMD_LOG_TAIL      = 0x30,
	MCP_CMD_LOG_LEVEL     = 0x31,
	MCP_CMD_FW_INFO       = 0x40,
	MCP_CMD_FW_BEGIN      = 0x41,
	MCP_CMD_FW_DATA       = 0x42,
	MCP_CMD_FW_END        = 0x43,
	MCP_CMD_FW_CONFIRM    = 0x44,
	MCP_CMD_FW_REVERT     = 0x45,
	MCP_CMD_DIAG          = 0x50,
} mcp_cmd_t;

/**
 * Response status, the first byte of every RSP payload.
 *
 * MCP_ERR_CRC is not reachable for a frame CRC — a frame whose CRC fails is
 * dropped, not answered. It reports the CRC-32 trailer of a *config import*
 * stream (CFG_IMPORT), which is checked at application level.
 */
typedef enum {
	MCP_OK            = 0,
	MCP_ERR_AUTH      = 1,  /* no session, or the wrong password */
	MCP_ERR_ARG       = 2,  /* malformed payload or out-of-range field */
	MCP_ERR_STATE     = 3,  /* command not legal in the current state */
	MCP_ERR_CRC       = 4,  /* CFG_IMPORT trailer mismatch */
	MCP_ERR_OFFSET    = 5,  /* FW_DATA/CFG_* offset gap; next expected returned */
	MCP_ERR_VERIFY    = 6,  /* image hash or MCUboot header rejected */
	MCP_ERR_NOSPC     = 7,  /* image larger than the staging slot */
	MCP_ERR_NOTSUP    = 8,  /* command or sub-function not wired on this build */
	MCP_ERR_INTERNAL  = 9,  /* a port returned an error we cannot classify */
	MCP_ERR_BUSY      = 10, /* another session-scoped operation is in progress */
} mcp_err_t;

/* ---------------------------------------------------------- capabilities */

/** HELLO capability bits. */
#define MCP_CAP_DFU    0x00000001U /* FW_* implemented (port_image wired) */
#define MCP_CAP_TELEM  0x00000002U /* STATUS_GET / TELEM_SUB have a source */
#define MCP_CAP_LOGS   0x00000004U /* LOG_TAIL / LOG_LEVEL have a ring */
#define MCP_CAP_CFG    0x00000008U /* CFG_* bound to a registry */
#define MCP_CAP_AUTH   0x00000010U /* AUTH can be evaluated (crypto wired) */
#define MCP_CAP_DIAG   0x00000020U /* DIAG sub-functions 1..4 available */

/* -------------------------------------------------------------- sub-ids */

/** EVT flags bit0: which subscription produced the event. */
#define MCP_EVT_SUB_TELEM 0U
#define MCP_EVT_SUB_LOG   1U
#define MCP_EVT_SUB_MASK  0x01U

/* ------------------------------------------------------------- constants */

/** STATUS_GET / TELEM_SUB groups. */
typedef enum {
	MCP_GRP_SUMMARY = 0,
	MCP_GRP_TIMING  = 1,
	MCP_GRP_GNSS    = 2,
	MCP_GRP_POWER   = 3,
	MCP_GRP_NET     = 4,
	MCP_GRP_PTP     = 5,
	MCP_GRP_ALARMS  = 6,
	MCP_GRP_COUNT   = 7,
} mcp_group_t;

/** Every defined bit of a TELEM_SUB group mask. */
#define MCP_GRP_MASK_ALL ((uint8_t)((1U << MCP_GRP_COUNT) - 1U))

/** REBOOT modes. */
#define MCP_REBOOT_NORMAL   0U
#define MCP_REBOOT_RECOVERY 1U
#define MCP_REBOOT_HALT     2U

/** FACTORY_RESET magic: ASCII "FACT" read as a little-endian u32. */
#define MCP_FACTORY_MAGIC 0x54434146U

/** Largest FW_DATA chunk. */
#define MCP_FW_CHUNK_MAX 1024U

/** MCUboot image header magic, first 4 bytes of a valid image (little-endian). */
#define MCP_MCUBOOT_MAGIC 0x96F3B83DU

/** DIAG sub-function count (0 = health snapshot, 1..4 vendor diagnostics). */
#define MCP_DIAG_SUB_COUNT 5U

/** port_image_info_t flag bits as they appear in HELLO / FW_INFO. */
#define MCP_SLOT_VALID     0x01U
#define MCP_SLOT_ACTIVE    0x02U
#define MCP_SLOT_PENDING   0x04U
#define MCP_SLOT_CONFIRMED 0x08U

/* --------------------------------------------------------------- codec */

/** A parsed frame. @p payload points into the caller's decoded buffer. */
typedef struct {
	uint8_t  ver;
	uint8_t  type;
	uint8_t  cmd;
	uint8_t  flags;
	uint16_t seq;
	uint16_t len;
	const uint8_t *payload;
} mcp_frame_t;

/**
 * Validate and parse a decoded (post-COBS) frame.
 *
 * @retval 0        Parsed; @p out is filled and the CRC verified.
 * @retval -EINVAL  Bad argument.
 * @retval -EBADMSG @p n outside [MCP_MIN_FRAME, MCP_MAX_FRAME].
 * @retval -EPROTO  The length field disagrees with @p n.
 * @retval -EILSEQ  CRC mismatch.
 */
int mcp_wire_parse(const uint8_t *frame, size_t n, mcp_frame_t *out);

/**
 * Fill in the header and CRC around a payload already sitting at
 * `out[MCP_HDR_LEN .. MCP_HDR_LEN + len)`.
 *
 * This is the in-place form the engine uses: response handlers build straight
 * into the frame buffer, so there is nothing to copy.
 *
 * @param out_n    Receives the frame length.
 *
 * @retval 0        Sealed.
 * @retval -EINVAL  Bad argument.
 * @retval -EMSGSIZE @p len exceeds MCP_MAX_PAYLOAD.
 * @retval -ENOSPC  @p cap too small.
 */
int mcp_wire_seal(uint8_t *out, size_t cap, uint8_t type, uint8_t cmd,
		  uint8_t flags, uint16_t seq, uint16_t len, size_t *out_n);

/**
 * Build a decoded (pre-COBS) frame from a separate payload buffer.
 *
 * @param out      Destination, at least MCP_HDR_LEN + @p len + MCP_CRC_LEN.
 *                 Must not overlap @p payload.
 * @param out_n    Receives the frame length.
 *
 * @retval 0        Built.
 * @retval -EINVAL  Bad argument.
 * @retval -EMSGSIZE @p len exceeds MCP_MAX_PAYLOAD.
 * @retval -ENOSPC  @p cap too small.
 */
int mcp_wire_build(uint8_t type, uint8_t cmd, uint8_t flags, uint16_t seq,
		   const uint8_t *payload, uint16_t len,
		   uint8_t *out, size_t cap, size_t *out_n);

/*
 * ---------------------------------------------------------------------------
 * Payload layouts. "REQ:" is what the tool sends, "RSP:" what comes back after
 * the leading u8 status == MCP_OK. Field widths in bytes; all little-endian.
 * ---------------------------------------------------------------------------
 *
 * HELLO 0x01
 *   REQ: (empty)
 *   RSP: u8 proto_ver, u16 max_payload, u32 caps, u8 model[16] (NUL-padded),
 *        u8 board_id[8],
 *        u32 run_ver[4], u32 run_size, u8 run_flags,
 *        u32 stg_ver[4], u32 stg_size, u8 stg_flags
 *        (slot fields are zero when no image port is wired; *_flags use
 *         MCP_SLOT_*)
 *
 * REBOOT 0x02
 *   REQ: u8 mode (MCP_REBOOT_*)
 *   RSP: (status only) — the reset happens MCP_REBOOT_DELAY_MS later so the
 *        response can drain out of the USB endpoint first.
 *
 * AUTH 0x08
 *   REQ: u8 password[1..64] (raw bytes, no NUL)
 *   RSP: u8 authed, u16 session_seconds
 *        A brute-force backoff (mcp.h) answers MCP_ERR_BUSY, without testing
 *        the password, while a lockout window is armed.
 *
 * CFG_LIST 0x10
 *   REQ: u16 start_id, u8 max_entries
 *   RSP: u16 next_id (0 = listing complete), u8 n,
 *        n × { u16 id, u8 type, u8 flags, u8 name_len, u8 name[name_len] }
 *
 * CFG_GET 0x11
 *   REQ: u16 id [, u8 flags]   flags bit0 = read the staged value if any
 *   RSP: u16 id, u8 type, u8 key_flags, u16 len, u8 value[len]
 *
 * CFG_SET 0x12
 *   REQ: u16 id, u8 type, u16 len, u8 value[len]
 *   RSP: u16 id
 *
 * CFG_COMMIT 0x13
 *   REQ: (empty)
 *   RSP: u16 applied, u16 reboot_keys, u32 reboot_groups (bit per config group),
 *        u16 persist_errors
 *        status is MCP_OK when the staged set validated and was applied to the
 *        live tree. persist_errors > 0 means that many applied keys could not
 *        be written to non-volatile storage: the change is live now but will
 *        not survive a reboot, so the tool must warn rather than report clean
 *        success. A validation/cross-field failure instead returns a bare
 *        error status and leaves the staged set intact.
 *
 * CFG_REVERT 0x14
 *   REQ: (empty)
 *   RSP: u16 dropped
 *
 * CFG_EXPORT 0x15
 *   REQ: u32 offset, u8 flags   flags bit0 = include CFG_F_SECRET keys
 *        (CFG_F_NOEXPORT keys — the admin credential — are never included)
 *   RSP: u32 offset, u8 more, u8 data[...]   (data is the rest of the payload)
 *        offset 0 restarts the export. Any other value must equal either the
 *        byte offset the previous chunk ended at (the normal advance) or the
 *        offset of the previous chunk (a retransmit after a lost response, in
 *        which case the identical chunk is re-emitted); anything else is
 *        MCP_ERR_OFFSET. Requesting secrets needs an authenticated session.
 *
 * CFG_IMPORT 0x16
 *   REQ: u32 offset, u8 flags, u8 data[...]
 *        flags bit0 = strict (unknown key IDs are an error), bit1 = final
 *        chunk (commit after this one)
 *   RSP: u32 next_offset, u8 complete, u16 applied, u32 reboot_groups,
 *        u16 persist_errors
 *        offset 0 restarts the import. A repeat of the previous chunk's offset
 *        is absorbed — the engine re-acknowledges the current frontier without
 *        re-applying — and a repeat of a committed final chunk re-emits the
 *        cached result, so a lost response never forces a restart. complete==1
 *        marks the committed final chunk; persist_errors carries the same
 *        meaning as in CFG_COMMIT.
 *
 * FACTORY_RESET 0x17
 *   REQ: u32 magic == MCP_FACTORY_MAGIC
 *   RSP: (status only)
 *
 * STATUS_GET 0x20
 *   REQ: u8 group (mcp_group_t)
 *   RSP: u8 group, u8 data[...]   data[0] is the group struct's own version
 *        byte, so a tool can decode an older firmware's layout.
 *
 * TELEM_SUB 0x21
 *   REQ: u8 group_mask (bit per mcp_group_t), u8 rate_hz (1..4)
 *   RSP: u8 group_mask, u8 rate_hz
 *   EVT: cmd 0x21, flags bit0 = MCP_EVT_SUB_TELEM, payload u8 group,
 *        u8 data[...] — same encoding as the STATUS_GET body.
 *
 * TELEM_UNSUB 0x22
 *   REQ: (empty)   RSP: (status only)
 *
 * LOG_TAIL 0x30
 *   REQ: u32 cursor, u8 follow, u8 max_records, u8 max_level, u16 sub_mask
 *   RSP: u32 next_cursor, u32 gap, u8 n, n × log record
 *   EVT: cmd 0x30, flags bit0 = MCP_EVT_SUB_LOG, payload u32 next_cursor,
 *        u32 gap, u8 n, n × log record
 *   log record: u32 seq, u64 mono_ms, u8 level, u8 subsys, u8 len, u8 msg[len]
 *
 * LOG_LEVEL 0x31
 *   REQ: u8 subsys (0xFF = all), u8 level
 *   RSP: (status only)
 *
 * FW_INFO 0x40
 *   REQ: (empty)
 *   RSP: u8 n_slots, n × { u8 slot, u8 flags, u32 size, u32 ver[4] },
 *        u8 dfu_state, u32 dfu_total, u32 dfu_written, u32 chunk_max,
 *        u32 write_block
 *
 * FW_BEGIN 0x41
 *   REQ: u32 size, u8 sha256[32]
 *        size is the image length and MUST NOT exceed staging_size() (the slot
 *        capacity minus the MCUboot trailer, per port_image.h); a larger image
 *        is MCP_ERR_NOSPC.
 *   RSP: u32 next_expected, u32 chunk_max, u32 write_block, u8 resumed
 *        Every non-final FW_DATA chunk length must be a multiple of write_block
 *        (the port buffers only the final short block); the final chunk may be
 *        any length.
 *
 * FW_DATA 0x42
 *   REQ: u32 offset, u8 data[1..1024]
 *   RSP: u32 next_expected      (also present on MCP_ERR_OFFSET)
 *
 * FW_END 0x43
 *   REQ: (empty)
 *   RSP: u32 size
 *
 * FW_CONFIRM 0x44 / FW_REVERT 0x45
 *   REQ: (empty)   RSP: (status only)
 *
 * DIAG 0x50
 *   REQ: u8 sub (0 health snapshot, 1 I2C scan, 2 PPS residuals,
 *        3 thread/CPU stats, 4 INA dump)
 *   RSP: u8 sub, u8 data[...]
 */

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_MCP_MCP_WIRE_H_ */
