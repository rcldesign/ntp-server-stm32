/*
 * STS1000 "Meridian" — core/mp: the Maintenance Protocol (FMT device side).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation; all state in the caller's ctx.
 *
 * This is the protocol the Field Maintenance Tool speaks over USB CDC-ACM. It is
 * a **second** protocol alongside the Meridian Console Protocol
 * (ARCHITECTURE.md §7, `core/mcp`), not a replacement: MCP stays the production
 * PC-tool channel on ACM1 and owns DFU; MP is a maintenance mode entered from
 * the human console on ACM0.
 *
 *   frame layer     mp_frame.h   COBS(channel|flags|payload|crc16) + 0x00
 *   control plane   mp_rpc.c     JSON-RPC 2.0 over channel 0x00
 *   manifest        mp_manifest.h build-time object table -> JSON
 *   safety          mp_override.h leases, guards G0-G3, device-side interlocks
 *   streams         mp_stream.h  CBOR telemetry / PPS / log / event records
 *   diagnostics     mp_diag.h    named tests with progress and results
 *   panel mirror    mp_mirror.h  live front-panel replica (project requirement)
 *   firmware update fwupd.h      the `fw.*` methods, over the mp_fwupd_t port
 *
 * Threading and budget (FMT §10). Core never takes a lock and never reads
 * hardware: it consumes snapshot structs the glue fills, which is what keeps the
 * protocol off the timing path (ARCHITECTURE.md §10 invariant 10). It is
 * **not** re-entrant and it is **not** internally synchronised — one mp_ctx_t
 * carries one frame decoder, one reply buffer, one transmit scratch, one lease
 * array and one session, and every entry point below mutates several of them.
 *
 * That matters because the engine has *two* drivers, on purpose:
 *
 *   byte-driven   mp_input() / mp_shell_byte(), from whatever context the glue
 *                 receives console bytes in;
 *   time-driven   mp_tick(), from a periodic context, which must keep running
 *                 when the host has gone quiet — the dead-man exists for
 *                 exactly that case, so folding the tick onto the byte path
 *                 would make the safety property fail in its own design case.
 *
 * **The glue must serialise every entry into one mp_ctx_t under a single lock**,
 * and that lock must span encode *and* transmit: mp_frame_encode() returns a
 * pointer into the ctx's one mp_frame_tx_t (mp_frame.h), so a caller that
 * releases before the sink has drained lets a second caller re-encode over the
 * bytes still on the wire — a spliced COBS frame. The same applies to the lease
 * array, which mp_tick()'s dead-man revert and mp_obj_override()'s grant both
 * write. The reference implementation of that rule is `mp_lock` in
 * src/zephyr/console/mp_glue.c, which also documents the one point where it is
 * released (a blocking credential check) and why that release is safe.
 *
 * Nothing here may be called from an interrupt: the transmit sink, the log ring
 * and the state providers are all thread-context facilities, and the lock above
 * cannot be taken in an ISR.
 *
 * Mode entry (FMT §2.3). The console starts in shell mode. MP mode is entered by
 * the `mp enter` shell command or, autobaud-safe, by the in-band magic
 * MP_MAGIC_ENTER. It is left by the in-band MP_MAGIC_EXIT, by a UART BREAK (the
 * glue calls mp_mode_exit()), or by the `sys.mode` request. Leaving MP mode
 * reverts every override, drops every subscription and closes the session.
 */

#ifndef STS1000_CORE_MP_MP_H_
#define STS1000_CORE_MP_MP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cfg/cfg.h"
#include "fwupd/fwupd.h"
#include "logring/logring.h"
#include "mp/mp_diag.h"
#include "mp/mp_frame.h"
#include "mp/mp_json.h"
#include "mp/mp_manifest.h"
#include "mp/mp_mirror.h"
#include "mp/mp_override.h"
#include "mp/mp_stream.h"
#include "port/port_image.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Protocol version reported by `hello`. */
#define MP_PROTO_VER 1U

/**
 * Mode-entry magic (FMT §2.3), autobaud-safe: SOH 'M' 'P' '1' STX.
 *
 * Neither byte pair occurs in shell input, and the sequence survives a baud
 * mismatch because a framing error simply fails to match.
 */
#define MP_MAGIC_ENTER "\x01MP1\x02"

/**
 * In-band exit sequence, symmetrical with the entry magic.
 *
 * The spec's `mp exit` is a *shell* command, which cannot be typed while the
 * console is in MP mode, so an in-band exit is required for a host that has no
 * way to assert BREAK. A BREAK leaves MP mode through mp_mode_exit() on any
 * platform whose glue can detect one — which is a real qualification and not a
 * formality: the STS1000's own console is CDC-ACM, where Zephyr surfaces no
 * BREAK at all, so this in-band sequence is the exit that exists there. See
 * sts_mp_notify_break() in src/zephyr/console/mp_glue.h.
 */
#define MP_MAGIC_EXIT "\x01MP0\x02"

/** Length of both magics. */
#define MP_MAGIC_LEN 5U

/** Console mode. */
typedef enum {
	MP_MODE_SHELL = 0, /**< the Zephyr shell owns the port */
	MP_MODE_MP,        /**< MP owns the port */
} mp_mode_t;

/* ------------------------------------------------------ JSON-RPC error codes */

/* Standard (JSON-RPC 2.0 §5.1). */
#define MP_E_PARSE (-32700)
#define MP_E_INVALID_REQ (-32600)
#define MP_E_NO_METHOD (-32601)
#define MP_E_BAD_PARAMS (-32602)
#define MP_E_INTERNAL (-32603)

/* Application codes, inside the reserved -32000..-32099 implementation range. */
#define MP_E_NO_SESSION (-32000) /**< G1+ with no open session, or stale */
#define MP_E_GUARD (-32001)      /**< confirmation missing or wrong */
#define MP_E_INTERLOCK (-32002)  /**< a device-side interlock refused */
#define MP_E_RANGE (-32003)      /**< outside the manifest envelope */
#define MP_E_BUSY (-32004)       /**< another operation holds the resource */
#define MP_E_NOTSUP (-32005)     /**< not wired on this build */
#define MP_E_STATE (-32006)      /**< not legal in the current state */
#define MP_E_HOLD (-32007)       /**< G3 armed; the hold has not elapsed */
#define MP_E_VETO (-32008)       /**< firmware refused or withdrew */
#define MP_E_IO (-32009)         /**< a port or subsystem returned an error */
/**
 * The credential was not accepted.
 *
 * Deliberately one code for every non-lockout refusal: a wrong password, an
 * unknown user, an over-long field and "no authority could answer" are
 * indistinguishable in the reply, so the control plane is not an account
 * oracle.
 */
#define MP_E_AUTH (-32010)
/**
 * Locked out by the credential store's brute-force throttle.
 *
 * Reported distinctly from MP_E_AUTH on purpose. Telling an operator "you are
 * locked out, wait" is operationally necessary — otherwise a technician at the
 * board reads a lockout as a typo and keeps extending it — and it reveals
 * nothing: the lockout is keyed on the name the caller just supplied, and
 * whoever caused it already knows they were failing. It says nothing about
 * whether the account exists, because the throttle counts attempts, not users.
 */
#define MP_E_LOCKED (-32011)
/** The session is valid but its role is below the guard class (FMT §5.3). */
#define MP_E_ROLE (-32012)

/** Human-readable message for an MP error code. Never NULL. */
const char *mp_err_msg(int code);

/* ------------------------------------------------------------- object values */

/** A read object's value, tagged by the manifest kind that produced it. */
typedef struct {
	uint8_t kind; /**< mp_kind_t */
	int32_t i;    /**< BOOL/ENUM/PCT/MV/CODE/PULSE/SCALAR */
	uint64_t u;   /**< BITS, and SCALAR values wider than 32 bits */
	float f;      /**< REAL */
	/** RAIL: bus mV, current µA, power µW, DIAG_ALRT. */
	int32_t rail[4];
	bool valid; /**< the reading is trustworthy */
	char text[32];
} mp_val_t;

/** Read the live (firmware-authoritative) value of @p obj. */
typedef int (*mp_obj_read_fn)(void *user, size_t obj, mp_val_t *out);

/** Momentarily assert @p obj for @p ms milliseconds. */
typedef int (*mp_pulse_fn)(void *user, size_t obj, uint32_t ms);

/* ---------------------------------------------------------- firmware update */

/** Everything the `fw.*` replies report about the orchestrator in one read. */
typedef struct {
	fwupd_event_t progress;      /**< comp/state/reason/done/total/rc */
	char before[FWUPD_VER_LEN];  /**< version read before the update */
	char after[FWUPD_VER_LEN];   /**< version read back after it */
	uint32_t allow;              /**< fwupd_cfg_t::allow, bit per component */
	uint32_t chunk_max;          /**< largest chunk `fw.data` will accept */
	uint32_t chunks_duplicate;
	uint32_t chunks_rejected;
	uint32_t sessions;
	uint32_t sessions_ok;
	uint32_t sessions_failed;
} mp_fw_status_t;

/**
 * The multi-IC firmware-update orchestrator (`core/fwupd`), reached through a
 * port rather than as a bare `fwupd_ctx_t *`.
 *
 * The indirection buys exactly one thing, and it is load-bearing: the
 * orchestrator has **two** drivers on **two** threads — the `fw.*` handlers here
 * (console RX thread, inside the engine lock) and the platform's periodic
 * `fwupd_step()` (console supervisor). `fwupd_ctx_t` is no more internally
 * synchronised than `mp_ctx_t` is, so somebody has to serialise them, and core
 * never takes a lock (mp.h threading block). The glue therefore owns one mutex
 * and wraps each call below in it; the only nesting is engine lock -> fwupd
 * lock, never the reverse, so the order cannot invert.
 *
 * Every member may be NULL only by leaving the whole port NULL: a partially
 * filled port is a programming error the handlers do not defend against beyond
 * the usual MP_E_NOTSUP.
 */
typedef struct {
	/**
	 * Rows [@p first, @p first + @p max) of the component inventory.
	 *
	 * @param n      Receives the rows written.
	 * @param total  Receives FWUPD_COMP__COUNT, so the caller can page.
	 *
	 * The board is probed when @p first is 0; later pages are served from
	 * that snapshot, so walking the inventory reads each component once.
	 */
	int (*inventory)(void *ctx, size_t first, fwupd_inv_row_t *out,
			 size_t max, size_t *n, size_t *total);
	/** fwupd_begin(): guards, allow-list, QUERY and PREPARE. */
	int (*begin)(void *ctx, uint8_t comp, const fwupd_req_t *req);
	/**
	 * fwupd_data(): one chunk; @p next_off is always set.
	 *
	 * @param duplicate  Optional. Set true when the chunk was accepted and
	 *                   advanced nothing — i.e. core/fwupd counted it in
	 *                   `chunks_duplicate` and did not call transfer().
	 *                   False on every other outcome, including refusals.
	 *
	 * The flag is an OUT-PARAM rather than something the caller derives,
	 * because it cannot be derived. `(off, len, next_off)` alone cannot tell
	 * a retransmit that ends exactly at the write offset — the commonest
	 * retransmit there is — from a fresh write: both leave
	 * `next_off == off + len`. Only the value of `done` BEFORE the call
	 * separates them, and the only place that can be read without a race is
	 * inside whatever mutual exclusion the implementation already holds
	 * around fwupd_data(). A `status()` round trip from the caller is a
	 * second, separate acquisition of that lock, so the `done` it returns is
	 * not provably the one fwupd_data() then saw.
	 */
	int (*data)(void *ctx, uint32_t off, const uint8_t *d, size_t len,
		    uint32_t *next_off, bool *duplicate);
	/** fwupd_end(): length + SHA-256, then VERIFY and RESTORE. */
	int (*end)(void *ctx);
	/** fwupd_abort() then fwupd_reset(): RESTORE runs, state returns to IDLE. */
	int (*abort)(void *ctx);
	/** fwupd_reset(): acknowledge a finished session. -EBUSY while one runs. */
	int (*reset)(void *ctx);
	/** Snapshot for the reply. Never fails on a wired port. */
	int (*status)(void *ctx, mp_fw_status_t *out);
	void *ctx;
} mp_fwupd_t;

/* ------------------------------------------------------------------- wiring */

/**
 * Everything core/mp needs from the platform. Filled once by the glue.
 *
 * Any state provider may be NULL; the affected request then answers
 * MP_E_NOTSUP rather than crashing, so a partially wired build is diagnosable
 * instead of dead.
 */
typedef struct {
	/* --- transport ---------------------------------------------------- */
	/** Hand one complete framed message to the wire. 0 on success. */
	mp_frame_sink_fn tx;
	void *tx_user;

	/* --- clock -------------------------------------------------------- */
	uint32_t (*mono_ms)(void *user);
	void *clock_user;

	/* --- identity ----------------------------------------------------- */
	const char *model;
	const char *serial; /**< the G2 confirmation string */
	const char *fw_version;
	const char *boot_version;
	const char *board_id;

	/* --- state providers --------------------------------------------- */
	int (*telem)(void *user, mp_telem_t *out);
	int (*pps)(void *user, mp_pps_t *out);
	int (*ilk)(void *user, mp_ilk_state_t *out);
	int (*mirror)(void *user, mp_mirror_in_t *out);
	int (*bundle)(void *user, mp_bundle_t *out);
	int (*time_get)(void *user, uint64_t *tai_ns, bool *fallback);
	void *state_user;

	/* --- actuation ---------------------------------------------------- */
	mp_ovr_apply_fn apply;
	void *apply_user;
	mp_obj_read_fn obj_read;
	void *obj_read_user;
	mp_pulse_fn pulse;
	void *pulse_user;
	mp_diag_action_fn diag;
	void *diag_user;

	/**
	 * Commit the staged cfg set and run the group appliers.
	 *
	 * The glue must route this to its own commit wrapper (sts_cfg_commit)
	 * rather than cfg_commit(), or a change would apply to the RAM tree
	 * without any subsystem being told.
	 */
	int (*cfg_commit)(void *user, cfg_commit_res_t *res);
	void *cfg_user;

	/* --- authentication ------------------------------------------------ */
	/**
	 * Authenticate a maintenance credential and return the granted role.
	 *
	 * Optional. When NULL the console cannot authenticate anybody, so every
	 * session is capped at MP_GUARD_G0 — observation only. That is the safe
	 * default: an unwired hook must degrade to read-only, never to full
	 * access.
	 *
	 * The glue routes this to the one credential store the web and console
	 * planes already share (sts_aaa_check()), so the lockout counters and the
	 * role mapping are the same on every surface (FMT §5.3). Core neither
	 * stores nor caches the credential: @p secret is borrowed for the
	 * duration of the call and wiped by the caller immediately afterwards.
	 *
	 * @param out_role  auth_role_t (== MP_ROLE_*); MP_ROLE_NONE unless the
	 *                  return value is 0.
	 * @retval 0        Accepted.
	 * @retval -EBUSY   Locked out by the brute-force throttle. This one
	 *                  reason *is* reported to the caller (see MP_E_LOCKED).
	 * @retval <0       Refused. The reason is NOT reported to the caller.
	 */
	int (*auth)(void *user, const char *user_name, const char *secret,
		    uint8_t *out_role);
	void *auth_user;

	/* --- passthrough (host -> device) ---------------------------------- */
	/**
	 * Put host-supplied bytes on the port behind a passthrough channel
	 * (0x06 SMP, 0x07 GNSS/USART3, 0x08 rubidium/UART7).
	 *
	 * Optional; without it the host->device direction of those channels
	 * answers -ENOTSUP and only the device->host tee runs. Core does not own
	 * any UART, so this is the whole seam: it decides *whether* the bytes may
	 * flow (the channel must be a passthrough, and its tunnel object must be
	 * held open by a live override lease — FMT §5.5), and the glue decides
	 * where they go.
	 *
	 * Called from mp_input()'s context with the engine lock held, so it must
	 * be bounded: see the burst limit in the glue.
	 *
	 * @retval >=0  Accepted.
	 * @retval <0   Refused; counted in `raw_rx_refused`.
	 */
	int (*raw_tx)(void *user, uint8_t ch, const uint8_t *data, size_t len);
	void *raw_tx_user;

	/* --- firmware update ----------------------------------------------- */
	/** The multi-IC orchestrator; NULL makes every `fw.*` method MP_E_NOTSUP. */
	const mp_fwupd_t *fwupd;

	/** Reboot / stay-in-bootloader. Modes as port_image_t::reboot. */
	const port_image_t *img;

	/* --- subsystems --------------------------------------------------- */
	cfg_ctx_t *cfg;
	logr_t *log;

	/* --- caller-owned buffers ---------------------------------------- */
	/** Record assembly (mirror keyframes, support bundles, log batches).
	 *  Must be at least MP_SCRATCH_MIN bytes. */
	uint8_t *scratch;
	size_t scratch_len;

	/** Reassembly pool for fragmented inbound messages; may be NULL. */
	mp_reasm_t *reasm;
	uint8_t reasm_n;

	/** Mirror previous-frame store; may be NULL to disable the mirror. */
	char *mirror_prev_ch;
	uint8_t *mirror_prev_attr;
	size_t mirror_cells;
} mp_wiring_t;

/** Smallest scratch buffer mp_init() accepts (one mirror keyframe). */
#define MP_SCRATCH_MIN MP_MIRROR_MAX

/** JSON token table depth for one control request. */
#ifndef MP_RPC_TOKENS
#define MP_RPC_TOKENS 96U
#endif

/**
 * Reply assembly buffer. One complete JSON-RPC response must fit.
 *
 * Larger than MP_TX_PAYLOAD_MAX on purpose: `hello`, `diag.list` and a manifest
 * page all exceed one frame, and mp_frame_send() fragments them. A reply that
 * still does not fit is answered as an internal error rather than truncated —
 * see mp_rpc_handle().
 */
#ifndef MP_REPLY_MAX
#define MP_REPLY_MAX 3072U
#endif

/** Longest `data` detail an error reply carries. */
#define MP_ERR_DATA_MAX 96U

/**
 * Host->device octets one passthrough frame may carry.
 *
 * Not a buffer size — nothing is copied — but a **time** bound. Both sinks
 * behind channels 0x07 and 0x08 are `uart_poll_out()` loops, and they run on
 * the console RX thread with the engine lock held, so the burst is how long a
 * host can stop the service tick from getting that lock. 128 octets is 33 ms at
 * the receiver's 38400 and 133 ms at the rubidium's 9600 — at most one missed
 * tick pass per frame, comfortably inside the STS_MP_TICK_MISS_MAX run the
 * dead-man budget allows.
 *
 * It is a real limit rather than a chunking loop because these tunnels are an
 * interactive maintenance path — a UBX config message, an FE-5680A command —
 * and bulk images go through `fw.*`, which is offset-driven and resumable.
 * A longer frame is refused whole and counted: truncating a byte stream aimed
 * at a bootloader would corrupt it silently.
 */
#define MP_TUNNEL_TX_MAX 128U

/* --------------------------------------------------------------------- ctx */

typedef struct {
	mp_wiring_t w;

	uint8_t mode; /**< mp_mode_t */

	mp_frame_rx_t rx;
	mp_frame_tx_t txf;
	mp_ovr_ctx_t ovr;
	mp_stream_ctx_t st;
	mp_diag_ctx_t diag;
	mp_mirror_ctx_t mirror;

	/* control-plane scratch */
	mp_json_tok_t tok[MP_RPC_TOKENS];
	char reply[MP_REPLY_MAX];

	/* per-request error detail */
	int err_code;
	const char *err_msg;
	char err_data[MP_ERR_DATA_MAX];
	/*
	 * Optional rewind point carried by a REFUSAL, not just a success.
	 *
	 * `fw.data` refuses a gap, an over-long chunk or a chunk past the end
	 * rather than trimming it, and leaves the transfer open — so the one
	 * thing the tool needs from the refusal is where the orchestrator
	 * actually is. Without this the error reply is `{code, message,
	 * data.reason}` and the tool has to guess or re-derive it from a
	 * separate request.
	 */
	bool err_has_rewind;
	uint32_t err_next_off;
	const char *err_state; /**< fwupd_state_name(), or NULL */

	/* magic detectors (shell mode: enter; MP mode: exit) */
	uint8_t magic_n;

	/*
	 * Deferred effects of a request. A handler cannot reboot or drop out of
	 * MP mode itself: its reply has to reach the host first. Both are
	 * honoured by mp_input()/mp_tick() once the reply has been transmitted.
	 */
	uint8_t pending_reboot; /**< 0 = none, else port_image reboot mode + 1 */
	bool pending_exit;      /**< leave MP mode after the reply */

	/* cached manifest content hash */
	uint32_t manifest_hash;

	/*
	 * Manifest indices of the three tunnel objects, resolved once at
	 * mp_init(). A held lease on one of them is what "the tunnel is open"
	 * means, and it is the gate on inbound passthrough bytes; resolving the
	 * id by string on every received frame would put a linear manifest walk
	 * on the byte path. -1 when the object is not in the manifest.
	 */
	int obj_gnss_tunnel;
	int obj_rb_tunnel;
	int obj_smp_tunnel;

	/*
	 * The firmware-update transfer's owner.
	 *
	 * fw.begin records the session that passed the guard; fw.data and fw.end
	 * refuse any other. A takeover opens a new session id, so an update
	 * cannot be continued — still less completed — by whoever arrived after
	 * the operator who authorised it. Zero means no transfer is open.
	 */
	uint32_t fw_sid;
	uint8_t fw_comp;  /**< fwupd_comp_t of the open transfer */
	uint8_t fw_guard; /**< mp_guard_t that authorised it */

	/* cfg export/import cursors, one of each per session */
	cfg_export_t cfg_ex;
	bool cfg_ex_active;
	cfg_import_t cfg_im;
	bool cfg_im_active;

	/* counters */
	uint32_t requests;
	uint32_t replies;
	uint32_t rpc_errors;
	uint32_t notifications;
	uint32_t records_sent;
	uint32_t tx_errors;
	uint32_t mode_enters;
	uint32_t mode_exits;
	/** Host->device passthrough: octets handed to a port, and refusals. */
	uint32_t raw_rx_bytes;
	uint32_t raw_rx_refused;
} mp_ctx_t;

/* --------------------------------------------------------------- lifecycle */

/**
 * Initialise.
 *
 * @retval 0        Initialised, in MP_MODE_SHELL.
 * @retval -EINVAL  @p c or @p w is NULL, or a mandatory member (tx, mono_ms,
 *                  apply) is missing.
 * @retval -ENOMEM  @p w->scratch is smaller than MP_SCRATCH_MIN.
 */
int mp_init(mp_ctx_t *c, const mp_wiring_t *w);

/** Current mode. */
uint8_t mp_mode(const mp_ctx_t *c);

/**
 * Enter MP mode (the `mp enter` shell command, or the entry magic).
 *
 * @retval 0        Entered (or already in MP mode).
 * @retval -EINVAL  @p c is NULL.
 */
int mp_mode_enter(mp_ctx_t *c);

/**
 * Leave MP mode (the exit magic, a UART BREAK, or `sys.mode`).
 *
 * Reverts every override, drops every subscription and closes the session — a
 * console returning to the shell must not leave the board commanded.
 */
int mp_mode_exit(mp_ctx_t *c);

/**
 * Report the physical link state (CDC DTR).
 *
 * A drop is a dead-man failure and also leaves MP mode: a host that closed the
 * port is not driving the box any more.
 */
int mp_set_link(mp_ctx_t *c, bool up);

/* ------------------------------------------------------------------- input */

/**
 * Feed one received byte while in **shell** mode, to watch for the entry magic.
 *
 * The glue passes every console byte here *in addition* to the shell, so the
 * magic works without the shell having to cooperate.
 *
 * @retval 1        The magic completed; the ctx is now in MP mode and the glue
 *                  must stop giving bytes to the shell.
 * @retval 0        Not (yet) the magic.
 * @retval -EINVAL  @p c is NULL.
 */
int mp_shell_byte(mp_ctx_t *c, uint8_t b);

/**
 * Feed received bytes while in **MP** mode.
 *
 * Drives the frame decoder and the control plane, and watches for the exit
 * magic on the raw byte stream (which is why it is checked here rather than
 * inside the framer: the exit must work even if framing has desynchronised).
 *
 * @retval >=0      Number of complete messages processed.
 * @retval -EINVAL  Bad argument.
 * @retval -EPERM   Not in MP mode.
 */
int mp_input(mp_ctx_t *c, const uint8_t *data, size_t len);

/* -------------------------------------------------------------------- tick */

/**
 * Periodic work: the dead-man, lease expiry, diagnostic stepping and every due
 * stream. Call at least every MP_TICK_MAX_MS while in MP mode; calling it in
 * shell mode is a cheap no-op except for the dead-man, which still runs so a
 * lease can never outlive a mode change.
 *
 * @retval >=0      Number of records transmitted.
 * @retval -EINVAL  @p c is NULL.
 */
int mp_tick(mp_ctx_t *c);

/* ------------------------------------------------------------ stream feeds */

/**
 * Tee raw bytes onto a passthrough channel (NMEA, UBX, GNSS, Rb, SMP).
 *
 * Core only frames them: the glue decides what to tee and when. Nothing is sent
 * when the channel is not subscribed.
 *
 * @retval >=0      Bytes accepted.
 * @retval -EINVAL  Bad argument or a non-raw channel.
 * @retval -ENOENT  Channel not subscribed (not an error; nothing was sent).
 */
int mp_stream_raw(mp_ctx_t *c, uint8_t ch, const uint8_t *data, size_t len);

/**
 * Post an event onto channel 0x09. Events are queued and drained by mp_tick().
 *
 * @p mono_ms is the moment the event HAPPENED, not the moment it was posted,
 * and it is a parameter for that reason. The producers of the five platform
 * event kinds — the 1 kHz GPIOF/GPIOG scan, the discipline loop, housekeeping —
 * run on threads that may not take the engine lock, so the glue stages their
 * events and drains them here up to a console-supervisor pass later. Stamping
 * the record at post time would put a 250 ms-wide uncertainty on every edge and
 * make the one thing this channel is for — correlating a button press with a
 * rail collapse, or an alarm with the log line that explains it — unreliable.
 *
 * Everything else is as mp_stream_eventf(): the event is copied, so no argument
 * has to outlive the call.
 *
 * @retval 0        Queued.
 * @retval -EINVAL  @p c is NULL, or @p kind is not an mp_ev_kind_t.
 * @retval -ENOSPC  The queue is full; the new event is dropped and counted.
 */
int mp_post_event(mp_ctx_t *c, uint8_t kind, uint8_t sub, uint16_t id,
		  uint8_t edge, int32_t value, uint32_t mono_ms,
		  const char *text);

/**
 * Firmware veto of an override (FMT §5.4). Reverts the lease and reports it on
 * the event channel.
 */
int mp_veto(mp_ctx_t *c, size_t obj, const char *reason);

/* ----------------------------------------------------------------- helpers */

/** Cached manifest content hash (computed at mp_init()). */
uint32_t mp_manifest_hash_cached(const mp_ctx_t *c);

/**
 * Handle one decoded control-plane message. Exposed for the unit tests, which
 * drive the RPC layer directly rather than through COBS framing.
 *
 * @param msg      Request bytes (a JSON document).
 * @param len      Request length.
 * @param out      Optional; receives the reply bytes (NUL-terminated).
 * @param out_len  Optional; receives the reply length. 0 for a notification.
 *
 * @retval 0        Handled (a reply may or may not have been produced).
 * @retval -EINVAL  Bad argument.
 */
int mp_rpc_handle(mp_ctx_t *c, const uint8_t *msg, size_t len, const char **out,
		  size_t *out_len);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_MP_MP_H_ */
