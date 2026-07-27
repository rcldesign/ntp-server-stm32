/*
 * STS1000 "Meridian" — core/mcp: Meridian Console Protocol engine.
 *
 * Platform-neutral C11. No dynamic allocation; the caller owns mcp_ctx_t.
 * ARCHITECTURE.md §7 fixes the protocol; mcp_wire.h fixes the bytes; this
 * header is the engine's API.
 *
 * Shape
 * -----
 * The engine is a pure byte pump. Glue feeds it whatever the CDC-ACM endpoint
 * delivered (mcp_input()), calls it on a timer (mcp_tick()), and hands it one
 * transmit callback. Everything else — the config registry, the log ring, the
 * status source, the staging flash, crypto — arrives as ports or callbacks at
 * init, so a host test wires fakes and gets the real state machine.
 *
 * Transmit contract (this is the part that bites)
 * -----------------------------------------------
 * The tx callback returns 0 when it took the frame, or a negative errno when
 * the link is back-pressured. The two frame classes are then treated
 * differently, deliberately:
 *
 *   RSP — never dropped. A response that cannot be transmitted is held in the
 *         context and mcp_input() stops consuming input; it reports how many
 *         bytes it took so the caller can re-offer the rest. Call
 *         mcp_poll_tx() when the link drains. A request whose answer was
 *         silently discarded would leave the tool waiting for a reply that is
 *         never coming, which is worse than back-pressure.
 *
 *   EVT — always droppable. Telemetry and log-follow events are snapshots of
 *         state that will be re-sampled; blocking the engine for them would
 *         let a slow reader stall the command channel. A dropped event bumps
 *         mcp_stats_t::evt_dropped, and the log stream additionally reports
 *         the resulting sequence gap to the tool, so loss is visible rather
 *         than silent.
 *
 * Sessions
 * --------
 * One session, because the channel is a point-to-point serial link: "the
 * session" is the connection, and authentication is a boolean on the context.
 * It clears on idle timeout (cfg key sec.session.s, default 600 s) and on
 * mcp_reset(). Mutating commands require it whenever cfg key sec.auth.req is
 * set.
 *
 * The stored credential is {salt[16], mac[32]} in cfg key sec.admin.pw, where
 * mac = HMAC-SHA-256(key = salt, msg = password). Compare is constant-time.
 * TODO(wave 3): the spec (§9.4) asks for a memory-hard KDF; move to Argon2id
 * in glue and keep this as the verifier of the derived key. HMAC-SHA-256 over
 * a salt is not password-stretching and is only defensible because the
 * credential never leaves a physically-present serial port.
 *
 * When no cfg registry is wired the engine has neither a policy nor a
 * credential store: AUTH answers MCP_ERR_NOTSUP and auth is treated as not
 * required. That configuration is for unit tests and minimal builds; the
 * production wiring always passes a cfg_ctx_t.
 */

#ifndef STS1000_CORE_MCP_MCP_H_
#define STS1000_CORE_MCP_MCP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cfg/cfg.h"
#include "logring/logring.h"
#include "mcp/mcp_wire.h"
#include "util/cobs.h"

#include "port/port_crypto.h"
#include "port/port_image.h"
#include "port/port_time.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Idle seconds before an authenticated session lapses, when cfg is absent. */
#define MCP_DEFAULT_SESSION_S 600U

/** Delay between answering REBOOT and actually resetting, in ms. */
#define MCP_REBOOT_DELAY_MS 250U

/** DFU session idle timeout: no FW_DATA for this long suspends the session. */
#define MCP_DFU_TIMEOUT_MS 30000U

/** Erase-ahead granularity, in bytes. STM32H5 internal flash sector size. */
#ifndef MCP_DFU_ERASE_GRAN
#define MCP_DFU_ERASE_GRAN 8192U
#endif

/**
 * Flash write-block size, in bytes. Every non-final FW_DATA chunk must be a
 * multiple of this so the port never has to buffer a partial block except the
 * final short one (port_image_t::staging_write buffers only that one). The
 * default is the STM32H5 128-bit programming quadword; the glue overrides it
 * with the real flash write-block-size via mcp_wiring_t::dfu_write_block, and
 * the value is echoed to the tool in the FW_BEGIN/FW_INFO responses.
 */
#ifndef MCP_DFU_WRITE_BLOCK
#define MCP_DFU_WRITE_BLOCK 16U
#endif

/*
 * AUTH brute-force throttle (spec §9.4). Failed password attempts are counted
 * per engine instance (state survives a link drop — resetting it on reconnect
 * would let an attacker toggle DTR to erase the counter). The first
 * MCP_AUTH_FREE_TRIES mismatches answer immediately; after that each further
 * mismatch arms a backoff window during which AUTH returns ERR_BUSY without
 * testing the password, doubling from MCP_AUTH_THROTTLE_MS up to
 * MCP_AUTH_THROTTLE_MAX_MS; reaching MCP_AUTH_LOCK_TRIES arms the longer
 * MCP_AUTH_LOCKOUT_MS window. A correct password (or a reboot) clears it.
 */
#ifndef MCP_AUTH_FREE_TRIES
#define MCP_AUTH_FREE_TRIES 3U
#endif
#ifndef MCP_AUTH_LOCK_TRIES
#define MCP_AUTH_LOCK_TRIES 8U
#endif
#ifndef MCP_AUTH_THROTTLE_MS
#define MCP_AUTH_THROTTLE_MS 2000U
#endif
#ifndef MCP_AUTH_THROTTLE_MAX_MS
#define MCP_AUTH_THROTTLE_MAX_MS 30000U
#endif
#ifndef MCP_AUTH_LOCKOUT_MS
#define MCP_AUTH_LOCKOUT_MS 60000U
#endif

/** Chunk size used when streaming the staged image back for SHA-256 verify. */
#ifndef MCP_DFU_VERIFY_CHUNK
#define MCP_DFU_VERIFY_CHUNK 256U
#endif

/** Log records carried by one LOG EVT. */
#ifndef MCP_LOG_EVT_RECS
#define MCP_LOG_EVT_RECS 8U
#endif

/**
 * Bytes of TLV payload CFG_EXPORT emits per chunk. Kept well below the frame
 * limit so the export streams in small, individually-acknowledged pieces over
 * a slow serial link (and so the resumable/retry path is the normal case, not
 * a corner one). Must be at least CFG_EXPORT_MIN_CHUNK.
 */
#ifndef MCP_CFG_EXPORT_CHUNK
#define MCP_CFG_EXPORT_CHUNK 256U
#endif

/** Stored admin credential size: salt[16] || HMAC-SHA-256[32]. */
#define MCP_PW_SALT_LEN 16U
#define MCP_PW_MAC_LEN  32U
#define MCP_PW_BLOB_LEN (MCP_PW_SALT_LEN + MCP_PW_MAC_LEN)
/** Longest password AUTH accepts. */
#define MCP_PW_MAX 64U

/* ----------------------------------------------------------- callbacks */

/**
 * Transmit one fully-framed (COBS-encoded, delimited) buffer.
 *
 * @retval 0   Accepted; the engine may reuse its buffer.
 * @retval <0  Back-pressured. RSPs are retried, EVTs are dropped.
 */
typedef int (*mcp_tx_fn)(void *user, const uint8_t *buf, size_t len);

/**
 * Produce the packed status struct for @p group (STATUS_GET / telemetry).
 *
 * The first byte written MUST be the struct's own version number, so a newer
 * tool can decode an older firmware. Everything after it is the group's
 * layout, owned by whoever implements this callback — in the production build
 * that is the `quality` module's publisher, which is why the encoding lives
 * behind a callback instead of inside core/mcp.
 *
 * @retval >0        Bytes written to @p buf.
 * @retval -ENOTSUP  This build does not produce that group.
 * @retval <0        Any other error, reported as MCP_ERR_INTERNAL.
 */
typedef int (*mcp_status_fn)(void *user, uint8_t group, uint8_t *buf,
			     size_t cap);

/**
 * Produce DIAG sub-function @p sub (1..4). Sub 0 is answered from
 * mcp_status_fn with the summary group and never reaches this callback.
 *
 * @retval >0        Bytes written.
 * @retval -ENOTSUP  Unimplemented sub-function.
 */
typedef int (*mcp_diag_fn)(void *user, uint8_t sub, uint8_t *buf, size_t cap);

/* ------------------------------------------------------------- wiring */

/** Device identity reported by HELLO. */
typedef struct {
	char    model[16];   /* NUL-padded ASCII, e.g. "STS1000" */
	uint8_t board_id[8]; /* unique per board (STM32 UID digest, serial, …) */
} mcp_ident_t;

/** Everything the engine needs, supplied once at mcp_init(). */
typedef struct {
	const port_clock_t         *clock;  /* optional; see mcp_tick() */
	const port_crypto_t        *crypto; /* optional; required for AUTH */
	const port_sha256_stream_t *sha;    /* optional; required for FW_END */
	const port_image_t         *img;    /* optional; gates MCP_CAP_DFU */
	cfg_ctx_t                  *cfg;    /* optional; gates MCP_CAP_CFG */
	logr_t                     *log;    /* optional; gates MCP_CAP_LOGS */

	mcp_status_fn status_cb;
	void         *status_user;
	mcp_diag_fn   diag_cb;
	void         *diag_user;

	mcp_tx_fn tx;       /* required */
	void     *tx_user;

	mcp_ident_t ident;

	/** Erase granularity override for the staging slot; 0 = default. */
	uint32_t dfu_erase_gran;
	/** Flash write-block override; 0 = MCP_DFU_WRITE_BLOCK. */
	uint32_t dfu_write_block;
} mcp_wiring_t;

/* -------------------------------------------------------------- state */

/** DFU session state, reported by FW_INFO. */
typedef enum {
	MCP_DFU_IDLE      = 0, /* nothing staged in this session */
	MCP_DFU_ACTIVE    = 1, /* FW_BEGIN accepted, receiving FW_DATA */
	MCP_DFU_SUSPENDED = 2, /* timed out; resumable by an identical FW_BEGIN */
	MCP_DFU_VERIFIED  = 3, /* FW_END passed, staged slot marked pending */
} mcp_dfu_state_t;

/**
 * DFU session.
 *
 * Resume semantics, precisely: a FW_BEGIN whose size *and* SHA-256 match the
 * session already in ACTIVE or SUSPENDED state does not erase anything and
 * answers with the current next-expected offset, so a tool that lost the link
 * continues from where it stopped. Any other FW_BEGIN — different size,
 * different hash, or a session in IDLE/VERIFIED — starts over from offset 0
 * and re-erases. A failed FW_END clears the session entirely: the staged bytes
 * are known bad, so resuming into them must not be possible.
 */
typedef struct {
	uint8_t  state;       /* mcp_dfu_state_t */
	uint32_t total;       /* image size from FW_BEGIN */
	uint32_t written;     /* next expected offset */
	uint32_t erased;      /* end of the erased region, exclusive */
	uint32_t erase_gran;
	uint32_t write_block; /* non-final chunks must be a multiple of this */
	uint8_t  sha[32];
	uint64_t last_ms;     /* last FW_BEGIN/FW_DATA, for the idle timeout */
	uint32_t chunks;      /* FW_DATA writes performed */
	uint32_t retransmits; /* duplicate FW_DATA absorbed idempotently */
	uint32_t timeouts;    /* sessions suspended by the idle timeout */
} mcp_dfu_t;

/** Engine counters. Read-only to the caller; useful in telemetry and tests. */
typedef struct {
	uint32_t rx_frames;
	uint32_t rx_bad_cobs;
	uint32_t rx_bad_crc;
	uint32_t rx_bad_hdr;    /* short/long frame, or a length field mismatch */
	uint32_t rx_bad_ver;
	uint32_t rx_ignored;    /* well-formed but not a REQ */
	uint32_t rsp_tx;
	uint32_t evt_tx;
	uint32_t evt_dropped;
	uint32_t tx_stalls;     /* transmit attempts the link refused */
	uint32_t auth_ok;
	uint32_t auth_fail;
	uint32_t auth_denied;   /* mutating command refused for lack of a session */
	uint32_t auth_throttled;/* AUTH refused by the brute-force backoff window */
	uint32_t session_expired;
} mcp_stats_t;

/** Engine state. Caller-owned; zeroed and populated by mcp_init(). */
typedef struct {
	mcp_wiring_t w;
	uint32_t     caps;

	/* Framing. */
	cobs_dec_t dec;
	uint8_t    rx[MCP_MAX_FRAME];
	uint8_t    frame[MCP_MAX_FRAME];
	uint8_t    enc[COBS_ENCODE_MAX(MCP_MAX_FRAME) + 1U];
	size_t     enc_len;
	bool       tx_pending;

	/* Session. */
	bool     authed;
	uint64_t now_ms;
	uint64_t last_activity_ms;
	uint16_t evt_seq;

	/* AUTH brute-force throttle. */
	uint32_t auth_fails;        /* consecutive password mismatches */
	uint64_t auth_lock_until_ms;/* AUTH refused until this instant */

	/* Telemetry subscription. */
	uint8_t  telem_mask;
	uint8_t  telem_rate;
	uint64_t telem_next_ms;

	/* Log follow subscription. */
	bool          log_follow;
	uint32_t      log_cursor;
	logr_filter_t log_filter;

	/* CFG_EXPORT / CFG_IMPORT streaming state. Each keeps enough history to
	 * absorb a retransmit of the previous chunk (see the retry contract in
	 * mcp_wire.h): export snapshots its cursor so it can re-emit, import
	 * remembers the last chunk's start offset so it can re-ack the frontier
	 * without re-applying, and a completed import caches its result so a
	 * repeat of the final chunk returns the same answer. */
	cfg_export_t exp;
	bool         exp_active;
	cfg_export_t exp_prev;      /* cursor at the start of the last chunk */
	bool         exp_prev_valid;

	cfg_import_t imp;
	bool         imp_active;
	uint32_t     imp_prev_off;  /* start offset of the last accepted chunk */
	bool         imp_prev_valid;
	bool         imp_done;      /* final chunk committed */
	uint32_t     imp_done_off;  /* start offset of the committed final chunk */
	cfg_commit_res_t imp_res;   /* cached commit result for a final repeat */

	/* Deferred reboot. */
	bool     reboot_pending;
	uint8_t  reboot_mode;
	uint64_t reboot_at_ms;

	mcp_dfu_t   dfu;
	mcp_stats_t stats;
} mcp_ctx_t;

/* ---------------------------------------------------------------- API */

/**
 * Initialise the engine.
 *
 * @retval 0        Ready.
 * @retval -EINVAL  @p c or @p w is NULL, or no tx callback was supplied.
 */
int mcp_init(mcp_ctx_t *c, const mcp_wiring_t *w);

/**
 * Drop the session: de-authenticate, cancel subscriptions, abandon any
 * CFG_EXPORT/CFG_IMPORT stream, discard the staged config overlay, and reset
 * the frame decoder. The DFU session survives — a link drop mid-upload is
 * exactly when resume must work.
 *
 * Call this on USB disconnect.
 */
void mcp_reset_session(mcp_ctx_t *c);

/**
 * Feed received bytes.
 *
 * @param consumed  Optional; bytes actually taken. Fewer than @p len means a
 *                  response is waiting for the link — call mcp_poll_tx() and
 *                  re-offer the remainder.
 *
 * @retval 0        All input consumed.
 * @retval -EAGAIN  Stopped early on transmit back-pressure.
 * @retval -EINVAL  Bad argument.
 */
int mcp_input(mcp_ctx_t *c, const uint8_t *buf, size_t len, size_t *consumed);

/**
 * Retry a held response.
 *
 * @retval 0   Nothing pending (the link is free).
 * @retval 1   Still pending.
 * @retval -EINVAL Bad argument.
 */
int mcp_poll_tx(mcp_ctx_t *c);

/**
 * Advance time: emit telemetry and log events, expire the session and the DFU
 * window, and execute a deferred reboot.
 *
 * @p now_ms is authoritative even when a clock port is wired, so a test drives
 * the engine deterministically. When no clock port is wired this is the *only*
 * thing that advances the engine's notion of time.
 *
 * @retval 0        Done.
 * @retval -EINVAL  Bad argument.
 */
int mcp_tick(mcp_ctx_t *c, uint64_t now_ms);

/** True when a session is authenticated right now. */
bool mcp_authenticated(const mcp_ctx_t *c);

/** Counters snapshot. */
const mcp_stats_t *mcp_stats(const mcp_ctx_t *c);

/** DFU session snapshot. */
const mcp_dfu_t *mcp_dfu_status(const mcp_ctx_t *c);

/**
 * Derive the stored admin credential blob for @p pw.
 *
 * Writes MCP_PW_BLOB_LEN bytes: the salt followed by
 * HMAC-SHA-256(key = salt, msg = pw). Provisioning code (local UI, shell,
 * factory jig) uses this and stores the result in cfg key sec.admin.pw.
 *
 * @param salt  MCP_PW_SALT_LEN random bytes, or NULL to draw them from
 *              @p crypto->rand.
 *
 * @retval 0        Written.
 * @retval -EINVAL  Bad argument, or a password outside 1..MCP_PW_MAX bytes.
 * @retval -ENOTSUP @p crypto lacks hmac_sha256 (or rand, when @p salt is NULL).
 * @retval -EIO     A crypto port call failed.
 */
int mcp_auth_make_blob(const port_crypto_t *crypto, const uint8_t *salt,
		       const uint8_t *pw, size_t pw_len, uint8_t *out,
		       size_t cap);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_MCP_MCP_H_ */
