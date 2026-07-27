/*
 * STS1000 "Meridian" — core/fwupd: u-blox ZED-F9T safeboot firmware-update
 * transport.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11, no allocation, no platform headers. The UART and the two
 * GPIOs arrive as callbacks (ubx_fwupd_ops_t), so the whole state machine —
 * including the safeboot dance and the retry logic — runs on the host.
 *
 * Spec reference: ntp_server_software_spec.md §8.5 —
 *
 *   "F9T firmware updated over USART3 (PD8/PD9) using GPS_SAFEBOOT_N (PD15) +
 *    GPS_RST_N (PD11) for safeboot recovery. The app exposes a guarded `gnss fw`
 *    flow (web + console) that streams the u-blox image, verifies the receiver
 *    reports the new version, and restores timing config afterward."
 *
 * ===========================================================================
 * THE OPCODES ARE NOT VERIFIED, AND THAT IS WHY THIS IS DISARMED BY DEFAULT
 * ===========================================================================
 *
 * u-blox does not publish the safeboot flash-loader protocol. The interface
 * documentation this project has (`docs/sts1000_firmware_hardware_interface.md`
 * §9) gives the *pins* and the *UART*, and says nothing about what to send over
 * it. Everything in the opcode table below is therefore marked
 * "VERIFY vs u-blox flash.xml / update tooling" and is an informed
 * reconstruction, not a specification.
 *
 * The safety consequence is handled structurally rather than with a warning:
 *
 *   **ubx_fwupd_begin() returns -ENOTSUP unless ubx_fwupd_cfg_t::opcodes_verified
 *   is set.** With it clear — the default — the module never asserts
 *   GPS_SAFEBOOT_N at all, so it cannot leave the receiver in safeboot. The bit
 *   exists so that whoever confirms the protocol against real u-blox tooling
 *   turns it on deliberately, in one place, with the table they verified.
 *
 * The rest of the module is complete and tested: framing, the pin sequence and
 * its timing, baud negotiation, chunking, per-chunk acknowledgement and retry,
 * the completion handshake, the return to normal mode and the UBX-MON-VER
 * readback. Only the four opcode *values* are unverified.
 *
 * ===========================================================================
 * FAIL-SAFE CONTRACT
 * ===========================================================================
 *
 * A receiver left with GPS_SAFEBOOT_N asserted does not boot into its
 * application, which means no PPS, no time, and a unit that looks dead. Every
 * failure path in this module therefore converges on ubx_fwupd_recover(), which
 *
 *   1. releases GPS_SAFEBOOT_N (drives it high / inactive),
 *   2. pulses GPS_RST_N so the receiver reboots out of safeboot,
 *   3. restores the UART to the configured operating baud,
 *
 * in that order, and it is called from the abort path, from every error
 * transition and from the timeout path. ubx_fwupd_in_safeboot() lets the caller
 * assert the invariant, and the host tests do exactly that after every injected
 * failure.
 */

#ifndef STS1000_CORE_FWUPD_UBX_FWUPD_H_
#define STS1000_CORE_FWUPD_UBX_FWUPD_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ubx/ubx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================== *
 *  The unverified opcode table — one place, all marked
 * ===================================================================== */

/**
 * UBX class of the firmware-update/loader messages.
 *
 * VERIFY vs u-blox flash.xml / update tooling. 0x09 is the class u-blox uses for
 * UPD-family messages (UBX-UPD-SOS is 0x09 0x14, which *is* documented in the
 * F9 interface description), so the class is the best-supported value in this
 * table. The message ids below are not.
 */
#define UBX_FWUPD_CLASS 0x09U /* VERIFY vs u-blox flash.xml / update tooling */

/** Loader identify/handshake. VERIFY vs u-blox flash.xml / update tooling. */
#define UBX_FWUPD_ID_IDENT 0x01U /* VERIFY vs u-blox flash.xml / update tooling */
/** Erase the application flash region. VERIFY vs u-blox flash.xml / tooling. */
#define UBX_FWUPD_ID_ERASE 0x02U /* VERIFY vs u-blox flash.xml / update tooling */
/** Write one image chunk at an offset. VERIFY vs u-blox flash.xml / tooling. */
#define UBX_FWUPD_ID_WRITE 0x03U /* VERIFY vs u-blox flash.xml / update tooling */
/** Finalise/commit the image. VERIFY vs u-blox flash.xml / update tooling. */
#define UBX_FWUPD_ID_FINALISE 0x04U /* VERIFY vs u-blox flash.xml / tooling */

/**
 * Octets of image payload per WRITE.
 *
 * VERIFY vs u-blox flash.xml / update tooling. 512 is a conservative choice: it
 * is a flash-page multiple on every u-blox flash part in the family and stays
 * well inside the receiver's UBX input buffer at 115200 baud.
 */
#define UBX_FWUPD_CHUNK 512U /* VERIFY vs u-blox flash.xml / update tooling */

/* ===================================================================== *
 *  Timing
 *
 *  docs/sts1000_firmware_hardware_interface.md §9 names GPS_SAFEBOOT_N (PD15)
 *  and GPS_RST_N (PD11) and their polarity, but gives no pulse widths. The
 *  values below are derived from the ZED-F9T data sheet's reset timing and are
 *  deliberately generous — every one of them is a *minimum* hold, so erring long
 *  costs a few milliseconds of update time and erring short costs a receiver
 *  that did not enter safeboot and then gets sent flash-write commands.
 * ===================================================================== */

/** GPS_RST_N asserted low for at least this long. */
#define UBX_FWUPD_RESET_HOLD_MS 10U
/**
 * GPS_SAFEBOOT_N must be stable *before* the reset is released and held after
 * it: the receiver samples the pin as it comes out of reset.
 */
#define UBX_FWUPD_SAFEBOOT_SETUP_MS 10U
#define UBX_FWUPD_SAFEBOOT_HOLD_MS 100U
/** Time for the ROM loader to be listening after reset release. */
#define UBX_FWUPD_BOOT_MS 200U
/** Time for the application firmware to come up after leaving safeboot. */
#define UBX_FWUPD_APP_BOOT_MS 2000U
/** Per-command acknowledgement budget. */
#define UBX_FWUPD_ACK_MS 2000U
/** Erase can take a while on a full application region. */
#define UBX_FWUPD_ERASE_MS 20000U
/** Attempts per chunk before the session is abandoned. */
#define UBX_FWUPD_CHUNK_RETRIES 3U
/** Attempts per baud rate during negotiation. */
#define UBX_FWUPD_PROBE_RETRIES 2U

/**
 * Baud rates tried, in order, when looking for the safeboot loader.
 *
 * The ROM loader comes up at 9600 on every u-blox part this applies to, so that
 * is first; the rest cover a receiver whose loader inherited a previously
 * configured rate.
 */
#define UBX_FWUPD_BAUD_COUNT 4U
extern const uint32_t ubx_fwupd_baud_list[UBX_FWUPD_BAUD_COUNT];

/** Baud used for the bulk transfer once the loader has answered. */
#define UBX_FWUPD_XFER_BAUD 115200U

/* ===================================================================== *
 *  Ops
 * ===================================================================== */

/**
 * The platform, as this module sees it.
 *
 * All callbacks are mandatory except @ref delay_ms; a NULL delay makes the
 * module rely purely on @ref now_ms, which is what the host tests do.
 */
typedef struct {
	/** Queue @p len octets for transmission. 0 on success. */
	int (*tx)(void *user, const uint8_t *data, size_t len);
	/**
	 * Read up to @p cap octets. Returns the count (0 when none available), or
	 * negative on error. Must not block.
	 */
	int (*rx)(void *user, uint8_t *data, size_t cap);
	/** Reconfigure the UART. 0 on success. */
	int (*set_baud)(void *user, uint32_t baud);
	/**
	 * Drive GPS_SAFEBOOT_N (PD15). @p assert_low true = pin low = safeboot
	 * requested. The pin is active-low, hence the parameter's name.
	 */
	int (*set_safeboot)(void *user, bool assert_low);
	/** Drive GPS_RST_N (PD11). @p assert_low true = pin low = in reset. */
	int (*set_reset)(void *user, bool assert_low);
	/** Monotonic milliseconds. */
	uint64_t (*now_ms)(void *user);
	/** Optional blocking delay, for the short pin-sequence holds. */
	void (*delay_ms)(void *user, uint32_t ms);
	void *user;
} ubx_fwupd_ops_t;

/* ===================================================================== *
 *  Config, state
 * ===================================================================== */

typedef struct {
	/**
	 * The operator has verified the opcode table above against real u-blox
	 * tooling for this receiver.
	 *
	 * **Default false, and while false ubx_fwupd_begin() refuses with
	 * -ENOTSUP without touching a pin.** Nothing else in this module is gated
	 * on it, because nothing else is unverified.
	 */
	bool opcodes_verified;
	/** Baud to leave the UART at when the session ends. */
	uint32_t operating_baud;
	/** Chunk size; 0 selects UBX_FWUPD_CHUNK. */
	uint16_t chunk;
} ubx_fwupd_cfg_t;

/** Defaults: not verified (so disarmed), 38400 operating baud, 512-octet chunks. */
void ubx_fwupd_cfg_defaults(ubx_fwupd_cfg_t *cfg);

/** Update phases. */
typedef enum {
	UBX_FWUPD_ST_IDLE = 0,
	UBX_FWUPD_ST_ENTER_SAFEBOOT, /**< driving the pin sequence */
	UBX_FWUPD_ST_NEGOTIATE,      /**< probing baud rates for the loader */
	UBX_FWUPD_ST_ERASE,          /**< erase issued, awaiting completion */
	UBX_FWUPD_ST_WRITE,          /**< accepting chunks */
	UBX_FWUPD_ST_FINALISE,       /**< commit issued */
	UBX_FWUPD_ST_LEAVE_SAFEBOOT, /**< releasing safeboot and rebooting */
	UBX_FWUPD_ST_VERIFY,         /**< polling UBX-MON-VER */
	UBX_FWUPD_ST_DONE,
	UBX_FWUPD_ST_FAILED,
	UBX_FWUPD_ST__COUNT,
} ubx_fwupd_state_t;

/** Short state name, never NULL. */
const char *ubx_fwupd_state_name(uint8_t st);

/** Receive scratch: the largest loader reply plus a MON-VER with extensions. */
#define UBX_FWUPD_RXBUF 512U

typedef struct {
	ubx_fwupd_cfg_t cfg;
	ubx_fwupd_ops_t ops;

	uint8_t state;
	int last_rc;

	/** True while GPS_SAFEBOOT_N is being held asserted by this module. */
	bool safeboot_asserted;
	uint32_t baud;        /**< baud currently configured */
	uint8_t baud_idx;     /**< index into ubx_fwupd_baud_list */
	uint8_t probe_tries;
	uint8_t chunk_tries;

	uint32_t total;
	uint32_t done;
	uint64_t deadline_ms;

	/** UBX parser over the receive path. */
	ubx_parser_t parser;
	uint8_t parse_buf[UBX_FWUPD_RXBUF];

	/** Frame assembly scratch: header + chunk + offset field + checksum. */
	uint8_t txbuf[UBX_FRAME_OVERHEAD + 8U + UBX_FWUPD_CHUNK];

	ubx_mon_ver_t ver;
	bool ver_valid;

	uint32_t frames_sent;
	uint32_t acks;
	uint32_t naks;
	uint32_t retries;
	uint32_t timeouts;
	uint32_t recoveries;
} ubx_fwupd_t;

/* ===================================================================== *
 *  API
 * ===================================================================== */

/**
 * Initialise @p u.
 *
 * @retval 0        Ready.
 * @retval -EINVAL  NULL argument, or an ops struct missing a mandatory callback.
 */
int ubx_fwupd_init(ubx_fwupd_t *u, const ubx_fwupd_cfg_t *cfg,
		   const ubx_fwupd_ops_t *ops);

/**
 * Read the receiver's current version by polling UBX-MON-VER.
 *
 * Usable at any time outside a session — it is the inventory read for
 * FWUPD_COMP_GNSS_ZED_F9T. Blocks (via poll + now_ms) for at most
 * UBX_FWUPD_ACK_MS.
 *
 * @param out  NUL-terminated "swVersion / FWVER=..." summary.
 *
 * @retval 0         @p out written.
 * @retval -EINVAL   NULL argument.
 * @retval -EBUSY    A session is in progress.
 * @retval -ETIMEDOUT No MON-VER arrived.
 */
int ubx_fwupd_query_version(ubx_fwupd_t *u, char *out, size_t cap);

/**
 * Start an update: enter safeboot, find the loader, erase.
 *
 * @retval 0         Ready for chunks (or still working; poll ubx_fwupd_step()).
 * @retval -EINVAL   NULL argument or zero @p image_size.
 * @retval -EBUSY    Already in a session.
 * @retval -ENOTSUP  ubx_fwupd_cfg_t::opcodes_verified is clear. **No pin was
 *                   touched**, so the receiver is still running its application.
 * @retval other     A step failed; ubx_fwupd_recover() has already run and
 *                   ubx_fwupd_in_safeboot() is false.
 */
int ubx_fwupd_begin(ubx_fwupd_t *u, uint32_t image_size);

/**
 * Write one chunk at @p off. Offsets must be sequential.
 *
 * Retries internally up to UBX_FWUPD_CHUNK_RETRIES on a NAK or a timeout.
 *
 * @retval 0        Written and acknowledged.
 * @retval -EPERM   Not in the write phase.
 * @retval -EINVAL  NULL argument, bad length, or a non-sequential offset.
 * @retval -EIO     The chunk failed every retry; the session is FAILED and
 *                  recovery has run.
 */
int ubx_fwupd_write(ubx_fwupd_t *u, uint32_t off, const uint8_t *data, size_t len);

/**
 * Finish: commit the image, leave safeboot, reboot, confirm with UBX-MON-VER.
 *
 * @retval 0        Complete; the receiver reported a version (see
 *                  ubx_fwupd_version()).
 * @retval -EPERM   Not in the write phase.
 * @retval -EIO     Commit failed or the receiver did not come back. Recovery has
 *                  run either way.
 */
int ubx_fwupd_finish(ubx_fwupd_t *u);

/**
 * Abandon the session and return the receiver to normal operation.
 *
 * Safe from any state and idempotent. Always leaves safeboot released.
 */
int ubx_fwupd_abort(ubx_fwupd_t *u);

/**
 * Drive timeouts and the asynchronous phases.
 *
 * @retval 0         Progress made, or nothing to do.
 * @retval -EAGAIN   Still working on the current phase.
 * @retval negative  The session failed; recovery has run.
 */
int ubx_fwupd_step(ubx_fwupd_t *u);

/**
 * Release safeboot and reboot the receiver into its application.
 *
 * The fail-safe primitive. Called automatically on every failure path; exposed
 * so the glue can call it during its own bring-up if a previous session was
 * interrupted by a power cut.
 *
 * @retval 0        The pins are released and the reset pulse was issued.
 * @retval -EINVAL  @p u is NULL.
 * @retval negative The GPIO or UART callback failed — the only case in which
 *                  the receiver may still be held, and it is reported.
 */
int ubx_fwupd_recover(ubx_fwupd_t *u);

/**
 * True while this module is holding GPS_SAFEBOOT_N asserted.
 *
 * The invariant every failure test asserts: after any error, abort or timeout,
 * this must be false.
 */
bool ubx_fwupd_in_safeboot(const ubx_fwupd_t *u);

/** Current phase. */
uint8_t ubx_fwupd_state(const ubx_fwupd_t *u);

/** Last decoded UBX-MON-VER, or NULL if none has been decoded. */
const ubx_mon_ver_t *ubx_fwupd_version(const ubx_fwupd_t *u);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_FWUPD_UBX_FWUPD_H_ */
