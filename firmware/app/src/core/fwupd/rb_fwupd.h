/*
 * STS1000 "Meridian" — core/fwupd: FE-5680A rubidium serial client, capability
 * classification and guarded EFC trim.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11, no allocation. UART7 arrives as byte callbacks, so the
 * framing, the capability probe and the trim bounds are all host-testable.
 *
 * Sources:
 *   docs/rb_rs232_interface.md   the serial path (SN65C3221E, the K1 RS-232/CMOS
 *                                relay, the opto lock line) and the documented
 *                                command set
 *   spec §3.4                    "Optional fine EFC trim over UART7 only if a
 *                                GPS-PPS cross-check shows a residual the Rb
 *                                isn't removing; default is hands-off."
 *   spec §8.6                    "Rb (FE-5680A) firmware is vendor-flashed over
 *                                UART7 only if the module supports it (guarded,
 *                                rare)."
 *
 * ===========================================================================
 * Framing
 * ===========================================================================
 *
 * The FE-5680A "Option 2" serial protocol is a short binary frame:
 *
 *   [id] [len_lo] [len_hi] [hdr_cksum] { [data...] [data_cksum] }
 *
 *   id          command identifier
 *   len         TOTAL frame length, little-endian, including every octet shown
 *   hdr_cksum   XOR of id, len_lo, len_hi
 *   data_cksum  XOR of the data octets; absent when there are no data octets
 *
 * So a command with no data is 4 octets and len = 4; the frequency-offset
 * commands carry a 32-bit value and are 9 octets with len = 9.
 *
 * The documented command set is exactly three members of the 0x2C..0x2E family:
 *
 *   0x2D  read the frequency offset            4-octet request, 9-octet reply
 *   0x2E  set the frequency offset, volatile   9 octets, unlimited writes
 *   0x2C  set the frequency offset and SAVE    9 octets, EEPROM — rate-limited
 *
 * The offset itself is a 32-bit **big-endian signed** value in the data field
 * (the length is little-endian; the payload is not — an inconsistency in the
 * protocol, not a typo here). Full scale +-0x7FFFFFFF is about +-383 Hz at the
 * 60 MHz internal reference, i.e. roughly 1.79e-7 Hz per count.
 *
 * ===========================================================================
 * Capability classes, and why the default is "telemetry only"
 * ===========================================================================
 *
 * These units are surplus. Variants differ in which commands they answer, in
 * whether the EEPROM write works at all, and — per docs/rb_rs232_interface.md —
 * even in the J6.8/J6.9 Tx/Rx pin direction. There is no reliable way to know
 * what is on the end of the cable except to ask it, so rb_fwupd_probe()
 * classifies the fitted unit:
 *
 *   RB_CAP_NONE            nothing answers. Either not powered (note that the
 *                          SN65C3221E is only powered when RB_PWR_EN is high, so
 *                          this is the expected answer before the Rb sequence
 *                          runs) or the Tx/Rx pair is swapped for this variant.
 *   RB_CAP_TELEMETRY_ONLY  0x2D answers. The offset can be read; nothing is
 *                          written. **This is what a healthy unit reports until
 *                          an operator asks for more**, because probing further
 *                          means writing to a running reference.
 *   RB_CAP_EFC_TRIMMABLE   a volatile 0x2E write was accepted and read back.
 *                          Only reached when rb_probe_req_t::probe_trim is set.
 *   RB_CAP_FW_UPDATABLE    a vendor loader answered. Only reachable when
 *                          rb_fwupd_cfg_t::loader_verified is set, which it is
 *                          not by default — see below.
 *
 * ===========================================================================
 * Firmware update: NOT_SUPPORTED, deliberately
 * ===========================================================================
 *
 * No FE-5680A firmware-update protocol is documented anywhere this project can
 * reach. rb_fwupd_begin() therefore returns -ENOTSUP, and the inventory reports
 * the component as not updatable, unless rb_fwupd_cfg_t::loader_verified is set
 * *and* a loader actually answers the probe. That is the honest answer to spec
 * §8.6's "only if the module supports it": for every variant this project has
 * documentation for, it does not.
 *
 * ===========================================================================
 * The EFC trim is an operator action with bounds, readback and an audit trail
 * ===========================================================================
 *
 * §3.4 says hands-off by default, and the reason is that the Rb is a
 * self-disciplined reference: steering it fights its own loop. rb_fwupd_trim()
 * is therefore never called by any automatic path in this firmware. It
 * additionally
 *
 *   - refuses an absolute offset outside [min_offset, max_offset],
 *   - refuses a single step larger than max_step, so a decimal-point error
 *     cannot move the reference by full scale,
 *   - reads the value back with 0x2D and fails if it did not take,
 *   - refuses an EEPROM-backed write unless allow_eeprom is set, and then
 *     rate-limits it to one per eeprom_min_interval_s (the part's EEPROM is
 *     rated for a bounded number of writes),
 *   - emits an audit event for every attempt, accepted or refused.
 */

#ifndef STS1000_CORE_FWUPD_RB_FWUPD_H_
#define STS1000_CORE_FWUPD_RB_FWUPD_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================== *
 *  Wire protocol
 * ===================================================================== */

/** Set frequency offset and store it in EEPROM. Rate-limited. */
#define RB_CMD_SET_OFFSET_SAVE 0x2CU
/** Read the current frequency offset. */
#define RB_CMD_REQ_OFFSET 0x2DU
/** Set frequency offset, volatile (lost on power cycle). */
#define RB_CMD_SET_OFFSET_VOL 0x2EU

/** id + len_lo + len_hi + header checksum. */
#define RB_FRAME_HDR_LEN 4U
/** Data octets carried by the 0x2C/0x2D/0x2E family. */
#define RB_OFFSET_DATA_LEN 4U
/** Wire length of an offset command or reply: header + data + data checksum. */
#define RB_OFFSET_FRAME_LEN (RB_FRAME_HDR_LEN + RB_OFFSET_DATA_LEN + 1U)
/** Largest frame this module builds or accepts. */
#define RB_FRAME_MAX 32U

/**
 * Frequency-offset full scale, in counts.
 *
 * +-0x7FFFFFFF is about +-383 Hz on the internal 60 MHz reference, so one count
 * is roughly 1.79e-7 Hz — about 3e-15 fractional, which is why the trim is a
 * fine adjustment and not a steering input.
 */
#define RB_OFFSET_MAX 0x7FFFFFFFL

/** A decoded frame. */
typedef struct {
	uint8_t id;
	uint16_t len;      /**< total frame length as advertised */
	uint8_t data[RB_FRAME_MAX];
	uint8_t data_len;
} rb_frame_t;

/**
 * Encode a frame.
 *
 * @param data      Data octets, or NULL when @p data_len is 0.
 * @param out_len   Receives the encoded length.
 *
 * @retval 0        Encoded.
 * @retval -EINVAL  NULL argument, or @p data_len over RB_FRAME_MAX.
 * @retval -ENOSPC  @p cap too small.
 */
int rb_frame_encode(uint8_t *buf, size_t cap, uint8_t id, const uint8_t *data,
		    size_t data_len, size_t *out_len);

/**
 * Decode and validate a complete frame.
 *
 * Both checksums are verified, and the advertised length must match @p len
 * exactly — a frame whose length field disagrees with what arrived is refused
 * rather than trimmed, because the alternative is acting on a truncated offset.
 *
 * @retval 0        Decoded.
 * @retval -EINVAL  NULL argument.
 * @retval -EBADMSG Too short, length mismatch, or a checksum failure.
 */
int rb_frame_decode(const uint8_t *buf, size_t len, rb_frame_t *out);

/** Encode a signed offset as the 4-octet big-endian data field. */
int rb_offset_encode(int32_t offset, uint8_t out[RB_OFFSET_DATA_LEN]);

/** Decode the 4-octet big-endian data field into a signed offset. */
int rb_offset_decode(const uint8_t in[RB_OFFSET_DATA_LEN], int32_t *out);

/* ---------------------------------------------------------- byte parser --- */

/**
 * Incremental frame assembler for the UART receive path.
 *
 * The protocol has no sync byte, so resynchronisation after a corrupt frame is
 * inherently heuristic: this parser reads a 4-octet header, validates the header
 * checksum, and on failure discards one octet and retries from the next. That is
 * the best available and it is why every accepted frame is checksum-verified
 * twice.
 */
typedef struct {
	uint8_t buf[RB_FRAME_MAX];
	uint8_t n;
	uint16_t want;   /**< total frame length once the header is validated */
	uint32_t frames; /**< accepted */
	uint32_t resyncs;/**< octets discarded looking for a valid header */
} rb_parser_t;

/** Reset a parser. */
void rb_parser_init(rb_parser_t *p);

/**
 * Feed one octet.
 *
 * @retval 1  A complete, checksum-valid frame is available in @p out.
 * @retval 0  More octets needed.
 * @retval -EINVAL NULL argument.
 */
int rb_parse_byte(rb_parser_t *p, uint8_t b, rb_frame_t *out);

/* ===================================================================== *
 *  Capability classes
 * ===================================================================== */

typedef enum {
	RB_CAP_UNKNOWN = 0,       /**< not probed yet */
	RB_CAP_NONE,              /**< nothing answered */
	RB_CAP_TELEMETRY_ONLY,    /**< 0x2D answers; nothing is written */
	RB_CAP_EFC_TRIMMABLE,     /**< a volatile 0x2E write took */
	RB_CAP_FW_UPDATABLE,      /**< a vendor loader answered */
	RB_CAP__COUNT,
} rb_cap_t;

/** Capability-class name, never NULL. */
const char *rb_cap_name(uint8_t cap);

/* ===================================================================== *
 *  Ops, config
 * ===================================================================== */

typedef struct {
	/**
	 * Queue @p len octets for transmission. **0 on success**, negative errno
	 * otherwise — a COUNT is not a success value here.
	 *
	 * Said explicitly because the board's two serial sinks disagree and the
	 * disagreement is invisible at the call site: rb_serial.c's op_tx()
	 * answers 0 and matches this directly, while platform/gnss.c's
	 * sts_gnss_uart_raw_tx() returns the octet count (mp_tunnel.c reports
	 * that count to the host) and has to be converted at its adapter. A sink
	 * bound here that returns a count makes exchange() read every successful
	 * frame as a failure and then hand the byte count back as if it were an
	 * errno.
	 */
	int (*tx)(void *user, const uint8_t *data, size_t len);
	/** Non-blocking read; count, 0 for none, or negative. */
	int (*rx)(void *user, uint8_t *data, size_t cap);
	uint64_t (*now_ms)(void *user);
	void (*delay_ms)(void *user, uint32_t ms);
	/**
	 * True when VCC_RB is up, i.e. when the SN65C3221E transceiver has power.
	 *
	 * Optional but strongly advised: docs/rb_rs232_interface.md notes the
	 * transceiver is powered from the Rb rail, so probing with the rail down
	 * produces RB_CAP_NONE and would otherwise be indistinguishable from a
	 * dead unit or a swapped Tx/Rx pair.
	 */
	bool (*rail_up)(void *user);
	/** Audit sink. @p what is a short static string. */
	void (*audit)(void *user, const char *what, int32_t value, int rc);
	void *user;
} rb_fwupd_ops_t;

/** EFC trim bounds. Deliberately narrow by default. */
typedef struct {
	int32_t min_offset;
	int32_t max_offset;
	/** Largest change one rb_fwupd_trim() call may make. */
	int32_t max_step;
	/** Permit EEPROM-backed (0x2C) writes at all. */
	bool allow_eeprom;
	/** Minimum seconds between EEPROM writes. */
	uint32_t eeprom_min_interval_s;
} rb_trim_cfg_t;

typedef struct {
	/** Per-command reply budget. */
	uint32_t reply_timeout_ms;
	/**
	 * A vendor firmware loader for this variant has been verified.
	 *
	 * **Default false.** While false, rb_fwupd_begin() returns -ENOTSUP and
	 * the probe never classifies a unit as RB_CAP_FW_UPDATABLE, because no
	 * FE-5680A loader protocol is documented anywhere this project can reach.
	 */
	bool loader_verified;
	rb_trim_cfg_t trim;
} rb_fwupd_cfg_t;

/**
 * Defaults: 1 s reply timeout, loader NOT verified, trim bounded to
 * +-1,000,000 counts (about +-0.18 Hz) with a 100,000-count step ceiling, EEPROM
 * writes disallowed, and a one-hour EEPROM interval if they are enabled.
 *
 * The +-1e6 bound is two orders of magnitude inside full scale: a fine trim that
 * corrects a residual the Rb's own loop is not removing needs parts in 1e13, not
 * parts in 1e9, and a bound that cannot express a gross error cannot cause one.
 */
void rb_fwupd_cfg_defaults(rb_fwupd_cfg_t *cfg);

/** What to attempt during a probe. */
typedef struct {
	/**
	 * Attempt a volatile 0x2E write/readback to establish EFC_TRIMMABLE.
	 *
	 * Off by default: it writes to a running frequency reference. The write
	 * used is the unit's *current* offset, so a successful probe is a no-op
	 * and a failed one has not moved anything.
	 */
	bool probe_trim;
	/** Attempt the (unverified) loader handshake. Requires loader_verified. */
	bool probe_loader;
} rb_probe_req_t;

/* ===================================================================== *
 *  Status
 * ===================================================================== */

/**
 * What the unit reports.
 *
 * Only @ref offset is available from the documented command set. Lock state
 * comes from the opto-isolated RB_LOCK line (PB13), not from the serial port,
 * and is filled in by the glue rather than by this module; temperature is not
 * exposed by any documented command, so @ref temp_valid is false unless a
 * variant-specific status frame has been configured and decoded.
 */
typedef struct {
	bool offset_valid;
	int32_t offset;
	bool lock_valid;
	bool locked;
	bool temp_valid;
	int32_t temp_mc;
	/** Frames received whose id this module does not recognise. */
	uint32_t unknown_frames;
	/** The most recent unrecognised frame, for the maintenance tool. */
	rb_frame_t last_unknown;
} rb_status_t;

/* ===================================================================== *
 *  Context
 * ===================================================================== */

typedef struct {
	rb_fwupd_cfg_t cfg;
	rb_fwupd_ops_t ops;
	rb_parser_t parser;

	uint8_t cap;        /**< rb_cap_t */
	rb_status_t status;

	int32_t last_offset_written;
	uint64_t last_eeprom_s;
	bool eeprom_written;

	uint32_t tx_frames;
	uint32_t rx_frames;
	uint32_t timeouts;
	uint32_t bad_frames;
	uint32_t trims_ok;
	uint32_t trims_refused;
} rb_ctx_t;

/* ===================================================================== *
 *  API
 * ===================================================================== */

/**
 * @retval 0        Ready.
 * @retval -EINVAL  NULL argument or a missing mandatory callback.
 */
int rb_fwupd_init(rb_ctx_t *c, const rb_fwupd_cfg_t *cfg,
		  const rb_fwupd_ops_t *ops);

/**
 * Read the frequency offset (command 0x2D).
 *
 * @retval 0         @p out written, and rb_ctx_t::status updated.
 * @retval -EINVAL   NULL argument.
 * @retval -ENODEV   The rail is down, so the transceiver is unpowered.
 * @retval -ETIMEDOUT No reply inside the configured budget.
 * @retval -EBADMSG  A reply arrived but was not a valid 0x2D response.
 */
int rb_read_offset(rb_ctx_t *c, int32_t *out);

/**
 * Classify the fitted unit.
 *
 * Always starts with the read-only 0x2D probe and stops there unless @p req asks
 * for more. The resulting class is available from rb_capability() and is what
 * the fwupd inventory reports.
 *
 * @retval 0        Classified (including as RB_CAP_NONE — that is an answer).
 * @retval -EINVAL  NULL argument.
 */
int rb_fwupd_probe(rb_ctx_t *c, const rb_probe_req_t *req);

/** Last classification; RB_CAP_UNKNOWN before a probe. */
uint8_t rb_capability(const rb_ctx_t *c);

/** Snapshot of what the unit has reported. */
int rb_fwupd_status(const rb_ctx_t *c, rb_status_t *out);

/**
 * One-line identity/version summary for the fwupd inventory.
 *
 * Renders the capability class and the offset, e.g.
 * "FE-5680A telemetry-only offset=-1234". Never fails to produce something: a
 * unit that did not answer renders as "FE-5680A absent".
 */
int rb_fwupd_identify(rb_ctx_t *c, char *out, size_t cap);

/**
 * Apply a bounded EFC fine trim. **Operator action only** (spec §3.4).
 *
 * @param new_offset  Absolute offset in counts.
 * @param persist     Write through to EEPROM (0x2C) instead of volatile (0x2E).
 * @param now_s       Monotonic seconds, for the EEPROM rate limit.
 *
 * @retval 0         Written and read back.
 * @retval -EINVAL   NULL argument.
 * @retval -ENODEV   Rail down.
 * @retval -ERANGE   @p new_offset outside the configured bounds, or the step
 *                   from the current value exceeds max_step.
 * @retval -EACCES   @p persist requested but allow_eeprom is false.
 * @retval -EAGAIN   EEPROM rate limit not yet expired.
 * @retval -ENOTSUP  The unit is not classified EFC_TRIMMABLE or better.
 * @retval -EIO      The readback did not match what was written.
 * @retval other     Transport error.
 */
int rb_fwupd_trim(rb_ctx_t *c, int32_t new_offset, bool persist, uint64_t now_s);

/**
 * Begin a firmware update.
 *
 * @retval -ENOTSUP  Always, unless rb_fwupd_cfg_t::loader_verified is set AND
 *                   the probe classified the unit RB_CAP_FW_UPDATABLE. No
 *                   FE-5680A loader protocol is documented; see the header
 *                   comment.
 * @retval -EINVAL   NULL argument.
 */
int rb_fwupd_begin(rb_ctx_t *c, uint32_t image_size);

/**
 * Feed received octets into the frame parser.
 *
 * Used by the glue's tunnel/monitor path so unsolicited frames are counted and
 * the most recent unrecognised one is retained for the maintenance tool.
 *
 * @return Frames completed by this call, or negative on a bad argument.
 */
int rb_fwupd_feed(rb_ctx_t *c, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_FWUPD_RB_FWUPD_H_ */
