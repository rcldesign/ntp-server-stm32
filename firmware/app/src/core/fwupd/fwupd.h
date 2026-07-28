/*
 * STS1000 "Meridian" — core/fwupd: multi-IC firmware-update orchestrator and
 * board component inventory.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11, no allocation, all state in a caller-owned context
 * (ARCHITECTURE.md §4). Behaviour comes from ntp_server_software_spec.md §8
 * (§8.3/§8.4 the STM32 application image, §8.5 the ZED-F9T passthrough, §8.6 the
 * FE-5680A), plus the customer requirement that **every IC on the board that has
 * firmware must be updatable through the maintenance tool**.
 *
 * ---------------------------------------------------------------------------
 * Why an orchestrator rather than three commands
 * ---------------------------------------------------------------------------
 *
 * Three updatable components on this board have nothing in common at the
 * transport layer: the STM32 application goes into an MCUboot slot through
 * port_image, the GNSS receiver goes over USART3 after a safeboot dance, and the
 * rubidium — if its variant supports it at all — goes over UART7. Exposing that
 * asymmetry to the maintenance tool would mean three protocols, three progress
 * models and three ways to get the destructive parts wrong.
 *
 * This module presents ONE lifecycle for all of them:
 *
 *   QUERY_VERSION -> PREPARE -> TRANSFER -> VERIFY -> RESTORE
 *
 * with a single chunked, resumable, integrity-checked data path, one progress
 * event stream, and one abort path that is guaranteed to run RESTORE. A target
 * plugs in as a fwupd_target_ops_t; the orchestrator never knows what a safeboot
 * pin is.
 *
 * ---------------------------------------------------------------------------
 * Inventory: the components that are NOT updatable are part of the contract
 * ---------------------------------------------------------------------------
 *
 * A maintenance tool that lists only the three updatable parts invites the
 * question "what about the PHY?" every time. fwupd_inventory() therefore
 * enumerates every firmware-bearing or identity-bearing IC on the board,
 * updatable or not, with how its version or identity is read and an explicit
 * `updatable` flag. FWUPD_ERR_NOT_SUPPORTED on a read-only component is a
 * *designed* answer, not a failure.
 *
 * ---------------------------------------------------------------------------
 * Everything here is destructive
 * ---------------------------------------------------------------------------
 *
 * A mis-issued update bricks a soldered-down part. The guards are therefore
 * layered and none of them is optional:
 *
 *   1. fwupd_begin() requires FWUPD_MAGIC in the request. A stray or corrupted
 *      frame cannot start a session.
 *   2. fwupd_cfg_t::allow is a per-component bitmap. A component the operator
 *      has not enabled cannot be started even with the magic.
 *   3. A target whose capability probe says NOT_SUPPORTED is refused before any
 *      pin moves.
 *   4. PREPARE raises the timing-degraded flag through the callback, so the rest
 *      of the firmware knows the GNSS receiver or the rubidium is about to
 *      disappear. RESTORE clears it — and RESTORE runs on *every* exit path,
 *      including abort and timeout.
 *   5. Every transition emits an audit event.
 */

#ifndef STS1000_CORE_FWUPD_FWUPD_H_
#define STS1000_CORE_FWUPD_FWUPD_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "port/port_crypto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------- constants -- */

/**
 * Required in fwupd_req_t::magic: "FWUP" big-endian.
 *
 * The same idea as the MCP FACTORY_RESET magic (ARCHITECTURE.md §7): a
 * destructive operation should not be reachable by a single corrupted opcode.
 */
#define FWUPD_MAGIC 0x46575550UL

/** Largest chunk the orchestrator accepts in one fwupd_data() call. */
#define FWUPD_CHUNK_MAX 1024U

/** Bound on an image, so a bad size cannot make the progress maths overflow. */
#define FWUPD_IMAGE_MAX (16U * 1024U * 1024U)

/** Fixed-width strings in the inventory. */
#define FWUPD_VER_LEN 40U

/* ----------------------------------------------------------- components --- */

/**
 * Every firmware- or identity-bearing IC on the board.
 *
 * Order is the inventory order the maintenance tool renders, which is why the
 * updatable components come first. The numeric values are part of the MCP
 * contract: append, never renumber.
 */
typedef enum {
	/* --- updatable ---------------------------------------------------- */
	/** U12 STM32H563ZIT6 application image, via MCUboot slot 1. */
	FWUPD_COMP_STM32_APP = 0,
	/** U20 u-blox ZED-F9T-00B, via USART3 + safeboot. */
	FWUPD_COMP_GNSS_ZED_F9T,
	/** External FE-5680A rubidium, via UART7. Capability-gated. */
	FWUPD_COMP_RB_FE5680A,

	/* --- read-only: no field-update path exists ----------------------- */
	/** U13 LAN8742AI RMII PHY. */
	FWUPD_COMP_PHY_LAN8742,
	/** U60 ATECC608B secure element. */
	FWUPD_COMP_SE_ATECC608B,
	/** Nine INA228 current/voltage monitors. */
	FWUPD_COMP_INA228,
	/** U58/U57 TMP117 temperature sensors. */
	FWUPD_COMP_TMP117,
	/** Display controller (ST7796) on SPI4. */
	FWUPD_COMP_DISPLAY_ST7796,
	/** FT6336U capacitive touch controller. */
	FWUPD_COMP_TOUCH_FT6336U,
	/** U43 MCP41U83 digipot (Rb rail trim). */
	FWUPD_COMP_DIGIPOT_MCP41U83,
	/** U9 NCP1095 PoE PD controller. */
	FWUPD_COMP_PD_NCP1095,

	FWUPD_COMP__COUNT,
} fwupd_comp_t;

/** How a component is reached. */
typedef enum {
	FWUPD_XPORT_NONE = 0,  /**< no electrical identity path at all */
	FWUPD_XPORT_INTERNAL,  /**< internal flash via port_image */
	FWUPD_XPORT_UART_UBX,  /**< USART3, UBX + safeboot loader */
	FWUPD_XPORT_UART_RB,   /**< UART7, FE-5680A serial */
	FWUPD_XPORT_I2C,       /**< I²C1 register read */
	FWUPD_XPORT_SPI,       /**< SPI4 register read */
	FWUPD_XPORT_MDIO,      /**< RMII management interface */
	FWUPD_XPORT__COUNT,
} fwupd_xport_t;

/** Static description of one component. All strings are non-NULL literals. */
typedef struct {
	uint8_t comp;             /**< fwupd_comp_t */
	const char *name;         /**< "u-blox ZED-F9T-00B" */
	const char *designator;   /**< "U20", or "U10/U31/..." for a group */
	/**
	 * How the version or identity is obtained, in words.
	 *
	 * Carried in the inventory because "updatable: no" without "and here is
	 * what we can read instead" is what makes an operator open the case.
	 */
	const char *version_how;
	bool updatable;
	uint8_t xport;            /**< fwupd_xport_t */
	uint8_t count;            /**< devices behind this row; 9 for INA228 */
} fwupd_comp_desc_t;

/** Static descriptor for @p comp, or NULL when @p comp is out of range. */
const fwupd_comp_desc_t *fwupd_comp_desc(uint8_t comp);

/** Component name, never NULL ("?" for an unknown id). */
const char *fwupd_comp_name(uint8_t comp);

/** Live inventory row: the descriptor plus what was actually read. */
typedef struct {
	const fwupd_comp_desc_t *desc;
	/** NUL-terminated version or identity, "" when unread. */
	char version[FWUPD_VER_LEN];
	bool version_valid;
	/** The identity read back matched something, i.e. the part answered. */
	bool present;
	/** Result of the last query, 0 or negative errno. */
	int last_rc;
} fwupd_inv_row_t;

/* ----------------------------------------------------------- target ops --- */

/**
 * One updatable (or merely identifiable) component's driver.
 *
 * Only @ref query_version is required. A component with no transfer path leaves
 * the programming callbacks NULL and is refused at fwupd_begin() with
 * -ENOTSUP — which is how the read-only components are modelled: they are real
 * inventory entries with a real identity read and no update path.
 *
 * Every callback returns 0 on success or a negative errno. -EAGAIN from
 * @ref poll means "still working"; the orchestrator will call it again.
 */
typedef struct {
	/** Read the running version/identity into @p out (NUL-terminated). */
	int (*query_version)(void *user, char *out, size_t cap);
	/**
	 * Put the component into a programming state: suspend the firmware's own
	 * use of it, assert safeboot/reset as required. Called once per session.
	 */
	int (*prepare)(void *user, uint32_t image_size);
	/** Write one chunk. @p last is true for the final chunk of the image. */
	int (*transfer)(void *user, uint32_t off, const uint8_t *data, size_t len,
			bool last);
	/** Finish programming and read back the new version into @p out. */
	int (*verify)(void *user, char *out, size_t cap);
	/**
	 * Return the component to service: re-apply configuration, resume normal
	 * use, release safeboot/reset.
	 *
	 * **Called on every exit path**, success or failure, exactly once per
	 * successful prepare(). It must be safe to call after a failure at any
	 * earlier step, and it must leave the component in a serviceable state or
	 * say why. This is the callback that guarantees a GNSS receiver is never
	 * left sitting in safeboot.
	 */
	int (*restore)(void *user, bool after_failure);
	/**
	 * Poll a long-running step: 0 done, -EAGAIN busy, negative on error.
	 *
	 * **No target on this board defines it, and that is the intended state.**
	 * All four vtables in console/fwupd_glue.c answer their prepare(),
	 * transfer() and verify() calls inline: the MCUboot slot engine and the
	 * two serial clients block for the milliseconds they need rather than
	 * returning -EAGAIN. So fwupd_step()'s poll branch and fwupd_begin()'s
	 * "stay in PREPARE" branch are unreachable in this image.
	 *
	 * Kept, rather than deleted, because it is the only shape that lets a
	 * future target with a genuinely long step — a receiver erase measured
	 * in tens of seconds — avoid parking the console supervisor for the
	 * duration, and removing it would mean rebuilding the PREPARE/VERIFY
	 * states around it later. Both branches are covered by tests/host, which
	 * is what stops them rotting while nothing in the image uses them.
	 */
	int (*poll)(void *user);
	/** Largest chunk this target accepts; 0 means FWUPD_CHUNK_MAX. */
	uint32_t (*chunk_max)(void *user);
	void *user;
} fwupd_target_ops_t;

/* --------------------------------------------------------------- events --- */

/** Session states. */
typedef enum {
	FWUPD_ST_IDLE = 0,
	FWUPD_ST_QUERY,    /**< reading the version that is about to be replaced */
	FWUPD_ST_PREPARE,  /**< prepare() issued, possibly still polling */
	FWUPD_ST_TRANSFER, /**< accepting chunks */
	FWUPD_ST_VERIFY,   /**< verify() issued, possibly still polling */
	FWUPD_ST_RESTORE,  /**< restore() issued, possibly still polling */
	FWUPD_ST_DONE,     /**< finished successfully; call fwupd_reset() */
	FWUPD_ST_FAILED,   /**< finished unsuccessfully; call fwupd_reset() */
	FWUPD_ST__COUNT,
} fwupd_state_t;

/** Short state name, never NULL. */
const char *fwupd_state_name(uint8_t st);

/** Why a session ended. */
typedef enum {
	FWUPD_END_NONE = 0,
	FWUPD_END_OK,
	FWUPD_END_ABORTED,     /**< the operator called fwupd_abort() */
	FWUPD_END_TIMEOUT,     /**< a step exceeded its budget */
	FWUPD_END_HASH,        /**< the image did not match its declared SHA-256 */
	FWUPD_END_SHORT,       /**< fwupd_end() with fewer bytes than declared */
	FWUPD_END_TARGET_ERROR,/**< a target callback failed */
	FWUPD_END_VERIFY,      /**< the component reports a version we did not expect */
	FWUPD_END__COUNT,
} fwupd_end_t;

/** Short reason name, never NULL. */
const char *fwupd_end_name(uint8_t reason);

/** Progress/state event. */
typedef struct {
	uint8_t comp;    /**< fwupd_comp_t */
	uint8_t state;   /**< fwupd_state_t */
	uint8_t reason;  /**< fwupd_end_t, meaningful in DONE/FAILED */
	uint32_t done;   /**< octets transferred */
	uint32_t total;  /**< octets declared */
	uint16_t permille; /**< 0..1000 of the transfer step */
	int rc;          /**< the errno that caused a FAILED, else 0 */
} fwupd_event_t;

/** Orchestrator callbacks. All optional. */
typedef struct {
	/** State change or transfer progress. */
	void (*event)(void *user, const fwupd_event_t *e);
	/**
	 * Audit trail. @p what is a short static string ("prepare", "abort", …).
	 * Every state transition and every guarded refusal produces one.
	 */
	void (*audit)(void *user, uint8_t comp, const char *what, int rc);
	/**
	 * Timing is (or is no longer) degraded because a reference component is
	 * being reprogrammed. Raised at PREPARE, cleared after RESTORE.
	 */
	void (*degraded)(void *user, uint8_t comp, bool on);
	void *user;
} fwupd_cb_t;

/* --------------------------------------------------------------- config --- */

typedef struct {
	/**
	 * Bit per fwupd_comp_t: 1 = updates permitted.
	 *
	 * Zero by default, so a freshly initialised orchestrator can update
	 * nothing at all. An operator (or the commissioning configuration) has to
	 * name each component explicitly.
	 */
	uint32_t allow;
	/** Milliseconds a PREPARE, VERIFY or RESTORE step may take. */
	uint32_t step_timeout_ms;
	/** Milliseconds without a chunk before a TRANSFER is abandoned. */
	uint32_t transfer_idle_timeout_ms;
} fwupd_cfg_t;

/** policy defaults: nothing allowed, 30 s per step, 60 s transfer idle. */
void fwupd_cfg_defaults(fwupd_cfg_t *cfg);

/** Set or clear the allow bit for @p comp. */
int fwupd_cfg_allow(fwupd_cfg_t *cfg, uint8_t comp, bool on);

/* -------------------------------------------------------------- request --- */

typedef struct {
	uint32_t magic;        /**< must be FWUPD_MAGIC */
	uint32_t size;         /**< image octets, 1..FWUPD_IMAGE_MAX */
	uint8_t sha256[32];    /**< SHA-256 over the whole image */
	/**
	 * Version the component must report after the update, or "" to accept
	 * whatever it reports.
	 *
	 * A substring match: "TIM 2.30" matches the F9T's
	 * "FWVER=TIM 2.30" extension string. When set and not matched, the
	 * session ends FWUPD_END_VERIFY even though the transfer succeeded —
	 * which is the difference between "we wrote bytes" and "the update took".
	 */
	char expect_version[FWUPD_VER_LEN];
} fwupd_req_t;

/* -------------------------------------------------------------- context --- */

typedef struct {
	fwupd_cfg_t cfg;
	fwupd_cb_t cb;
	port_sha256_stream_t sha;
	/** Per-component target drivers, indexed by fwupd_comp_t. */
	fwupd_target_ops_t target[FWUPD_COMP__COUNT];
	bool target_set[FWUPD_COMP__COUNT];

	uint8_t state;   /**< fwupd_state_t */
	uint8_t comp;    /**< component of the active session */
	uint8_t reason;  /**< fwupd_end_t */
	int last_rc;

	uint32_t total;
	uint32_t done;
	uint64_t step_started_ms;
	uint64_t last_chunk_ms;
	bool prepared;   /**< prepare() succeeded, so restore() is owed */
	bool degraded;   /**< the degraded callback has been raised */

	char version_before[FWUPD_VER_LEN];
	char version_after[FWUPD_VER_LEN];
	char expect_version[FWUPD_VER_LEN];

	uint8_t want_sha[32];
	uint8_t sha_state[PORT_SHA256_CTX_SIZE];
	bool sha_active;

	/** Lifetime counters, for telemetry. */
	uint32_t sessions;
	uint32_t sessions_ok;
	uint32_t sessions_failed;
	uint32_t chunks_duplicate;
	uint32_t chunks_rejected;
} fwupd_ctx_t;

/* ------------------------------------------------------------ lifecycle --- */

/**
 * Initialise @p c.
 *
 * @param sha  Incremental SHA-256 port, used to hash the image as it streams.
 *             Required: an unverified image is not an update path.
 *
 * @retval 0        Ready, state FWUPD_ST_IDLE.
 * @retval -EINVAL  NULL argument, or @p sha is missing a callback.
 */
int fwupd_init(fwupd_ctx_t *c, const fwupd_cfg_t *cfg, const fwupd_cb_t *cb,
	       const port_sha256_stream_t *sha);

/**
 * Register (or with NULL @p ops, deregister) the driver for @p comp.
 *
 * @retval 0        Registered.
 * @retval -EINVAL  @p c is NULL, @p comp out of range, or @p ops has no
 *                  query_version.
 * @retval -EBUSY   A session is in progress.
 */
int fwupd_set_target(fwupd_ctx_t *c, uint8_t comp,
		     const fwupd_target_ops_t *ops);

/** Replace the policy. Refused while a session is running. */
int fwupd_set_cfg(fwupd_ctx_t *c, const fwupd_cfg_t *cfg);

/* ------------------------------------------------------------ inventory --- */

/**
 * Enumerate every component, updatable or not.
 *
 * Each row's version is read through the component's registered
 * query_version(), so a part that does not answer is reported as
 * `present = false` with the errno in `last_rc` rather than omitted. A component
 * with no registered driver still appears, with its descriptor and an empty
 * version — the tool can then say "not probed" instead of "absent".
 *
 * @param out  Array of at least @p max rows.
 * @param n    Receives the number of rows written.
 *
 * @retval 0        Written.
 * @retval -EINVAL  NULL argument.
 * @retval -ENOSPC  @p max is below FWUPD_COMP__COUNT; @p n is still set to the
 *                  number written, so a small buffer truncates rather than
 *                  failing outright.
 */
int fwupd_inventory(fwupd_ctx_t *c, fwupd_inv_row_t *out, size_t max, size_t *n);

/**
 * Read one component's version without starting a session.
 *
 * @retval 0        @p out written.
 * @retval -EINVAL  NULL argument or bad @p comp.
 * @retval -ENODEV  No driver registered for @p comp.
 * @retval other    Whatever the driver returned.
 */
int fwupd_query(fwupd_ctx_t *c, uint8_t comp, char *out, size_t cap);

/* -------------------------------------------------------------- session --- */

/**
 * Begin an update session for @p comp.
 *
 * Runs QUERY_VERSION and then PREPARE. On return the state is FWUPD_ST_TRANSFER
 * (ready for chunks) or FWUPD_ST_PREPARE (prepare is still polling — call
 * fwupd_step() until it leaves).
 *
 * @retval 0        Session open.
 * @retval -EINVAL  NULL argument, bad @p comp, bad magic, or a size outside
 *                  1..FWUPD_IMAGE_MAX.
 * @retval -EBUSY   A session is already in progress.
 * @retval -ENODEV  No driver registered for @p comp.
 * @retval -EACCES  fwupd_cfg_t::allow does not permit @p comp.
 * @retval -ENOTSUP The component has no transfer path (a read-only part, or a
 *                  variant whose capability probe found no loader).
 * @retval other    A target callback failed; the session is FAILED and RESTORE
 *                  has already run.
 */
int fwupd_begin(fwupd_ctx_t *c, uint8_t comp, const fwupd_req_t *req,
		uint64_t now_ms);

/**
 * Feed one chunk.
 *
 * Resumability, precisely: a chunk at exactly the expected offset is written and
 * hashed; a chunk wholly inside the already-written region is a duplicate
 * retransmit and is accepted as a no-op; anything else is refused with
 * @p next_off set so the tool can rewind. Partial overlap is refused rather than
 * trimmed — accepting the tail would leave the streaming hash covering bytes
 * that were never written.
 *
 * @param next_off  Always set to the next offset the orchestrator wants, on
 *                  success and on refusal alike. May be NULL.
 *
 * @retval 0        Accepted (written, or recognised as a duplicate).
 * @retval -EINVAL  NULL argument, zero length, or a chunk over FWUPD_CHUNK_MAX
 *                  or over the target's own limit.
 * @retval -EPERM   No session, or the session is not in TRANSFER.
 * @retval -ENOSPC  The chunk would run past the declared image size.
 * @retval -EPROTO  Out-of-order offset; @p next_off says where to resume.
 * @retval other    The target's transfer() failed; the session is FAILED and
 *                  RESTORE has run.
 */
int fwupd_data(fwupd_ctx_t *c, uint32_t off, const uint8_t *data, size_t len,
	       uint32_t *next_off, uint64_t now_ms);

/**
 * Declare the transfer complete: checks the length and the SHA-256, then runs
 * VERIFY and RESTORE.
 *
 * @retval 0        The session has entered VERIFY (poll with fwupd_step()).
 * @retval -EPERM   Not in TRANSFER.
 * @retval -EINVAL  @p c is NULL.
 * @retval -EBADMSG Fewer octets than declared (FWUPD_END_SHORT), or the
 *                  SHA-256 does not match (FWUPD_END_HASH). In both cases the
 *                  session is FAILED, nothing was verified, and RESTORE has run.
 */
int fwupd_end(fwupd_ctx_t *c, uint64_t now_ms);

/**
 * Abandon the session.
 *
 * Always safe, from any state, including one where a target callback has already
 * failed. RESTORE runs if PREPARE succeeded. Idempotent.
 *
 * @retval 0        Aborted (or there was nothing to abort).
 * @retval -EINVAL  @p c is NULL.
 */
int fwupd_abort(fwupd_ctx_t *c, uint64_t now_ms);

/**
 * Pump a polling step and enforce the timeouts.
 *
 * Call from the maintenance thread while a session is open. Advances
 * PREPARE -> TRANSFER, VERIFY -> RESTORE and RESTORE -> DONE/FAILED, and fails
 * a session whose step or transfer has stalled.
 *
 * @retval 0        Nothing to do, or progress was made.
 * @retval -EINVAL  @p c is NULL.
 */
int fwupd_step(fwupd_ctx_t *c, uint64_t now_ms);

/** Return a finished session to IDLE. Refused while one is still running. */
int fwupd_reset(fwupd_ctx_t *c);

/* ------------------------------------------------------------ accessors --- */

/** Current state; FWUPD_ST_IDLE for a NULL context. */
uint8_t fwupd_state(const fwupd_ctx_t *c);

/** Component of the active or last session. */
uint8_t fwupd_component(const fwupd_ctx_t *c);

/** Why the last session ended. */
uint8_t fwupd_end_reason(const fwupd_ctx_t *c);

/** Fill @p out with the current progress. */
int fwupd_progress(const fwupd_ctx_t *c, fwupd_event_t *out);

/** Version read before the update; "" when none. Never NULL. */
const char *fwupd_version_before(const fwupd_ctx_t *c);

/** Version read back after the update; "" when none. Never NULL. */
const char *fwupd_version_after(const fwupd_ctx_t *c);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_FWUPD_FWUPD_H_ */
