/*
 * STS1000 "Meridian" — core/mp: override leases, guards, interlocks (FMT §5).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation; every table is in the ctx.
 *
 * This is the safety core of the Maintenance Protocol and the one module whose
 * failure modes matter more than its features:
 *
 *   §5.1-5.3  An override is a **lease**, never a write. It is held only while
 *             the owning session is valid, its keepalive is fresh and the
 *             physical link is asserted. The moment any of those stops being
 *             true every override in the table reverts to firmware-automatic
 *             control and the reversion is logged. Nothing here can leave the
 *             board in a commanded state after the tool walks away.
 *   §5.4      The owning subsystem may **veto**: firmware always wins, and the
 *             veto is reported on the event channel rather than swallowed.
 *   §5.2/5.3  Guard classes G0..G3 escalate cumulatively (see mp_manifest.h) and
 *             each carries a **minimum role**: G1 needs operator, G2 and G3 need
 *             admin. The role is granted once, at session open, by the
 *             platform's credential store (mp_wiring_t::auth); a session that
 *             presented no credential carries MP_ROLE_NONE and can therefore do
 *             nothing but observe. There is no way to raise a session's role
 *             after the fact — re-authenticating means opening a new session,
 *             which reverts everything the old one held.
 *   §5.5      Interlocks are evaluated **here**, on the device, from a state
 *             struct the glue fills. None of them is host-overridable. Making
 *             them a pure function of (mask, state, value) is what lets the
 *             refusals be unit-tested without a board.
 *
 * Timing contract for the dead-man (the property the spec puts a number on):
 * mp_ovr_tick() reverts everything on the first call that observes a failed
 * dead-man condition, so worst-case revert latency is
 * `keepalive TTL + tick period`. The glue must therefore tick at no slower than
 * MP_TICK_MAX_MS; mp_ovr_tick() reports a tick gap larger than that in
 * mp_ovr_ctx_t::late_ticks so a too-slow caller is visible rather than silent.
 */

#ifndef STS1000_CORE_MP_MP_OVERRIDE_H_
#define STS1000_CORE_MP_MP_OVERRIDE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mp/mp_manifest.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ limits */

/** Concurrent overrides. One per object; a second grant replaces the first. */
#ifndef MP_LEASE_MAX
#define MP_LEASE_MAX 16U
#endif

/** Keepalive freshness window (FMT §5.3: overrides held only while <= 5 s). */
#define MP_KEEPALIVE_TTL_MS 5000U

/** Deadline the spec puts on reverting after a dead-man failure. */
#define MP_DEADMAN_REVERT_MS 2000U

/** Slowest mp_ovr_tick() cadence that still meets MP_DEADMAN_REVERT_MS. */
#define MP_TICK_MAX_MS MP_DEADMAN_REVERT_MS

/** Default and maximum lease lifetime. A lease always has one. */
#define MP_LEASE_TTL_DEFAULT_MS 60000U
#define MP_LEASE_TTL_MAX_MS 600000U

/** G3: how long the operator must hold, and how long an arm stays valid. */
#define MP_G3_HOLD_MS 3000U
#define MP_G3_WINDOW_MS 30000U

/**
 * Default G3 phrase.
 *
 * The FMT spec requires "a typed phrase" without fixing its text, so this is
 * the device's default and the glue may replace it (mp_ovr_set_phrase()). It is
 * deliberately long and upper-case: a G3 action stops the clock or the board,
 * and the phrase exists to make that impossible to type by reflex.
 */
#define MP_G3_PHRASE_DEFAULT "CONFIRM UNSAFE ACTION"

/** Longest accepted confirmation string (serial or phrase). */
#define MP_CONFIRM_MAX 48U

/**
 * Longest user name recorded on a session, excluding the NUL.
 *
 * Matches AUTH_USER_MAX (core/auth) so an audit record never names a truncated
 * user — the whole point of storing it is that the name in the log is the name
 * the operator typed.
 */
#define MP_USER_MAX 63U

/**
 * Longest credential the control plane will carry, excluding the NUL.
 *
 * Matches AUTH_SECRET_MAX. Anything longer is refused as a bad credential, not
 * truncated: silently hashing a prefix would make two different passwords the
 * same password.
 */
#define MP_SECRET_MAX 64U

/* --------------------------------------------------------------------- roles */

/**
 * Authorisation roles, ordered so a numeric comparison is a privilege
 * comparison: a guard needing operator rights accepts `role >= MP_ROLE_OPERATOR`.
 *
 * These are numerically identical to `auth_role_t` (`core/auth/auth.h`), which
 * is deliberately **not** included here: core/mp's dependency edge set
 * (ARCHITECTURE.md §4) does not contain `auth`, and taking one for three
 * integers would couple the protocol to the AAA module it must stay independent
 * of. The glue that bridges the two asserts the identity at build time
 * (mp_glue.c) and tests/host/test_mp_override.c pins it against the real enum.
 */
#define MP_ROLE_NONE 0U
#define MP_ROLE_VIEWER 1U
#define MP_ROLE_OPERATOR 2U
#define MP_ROLE_ADMIN 3U

/** Role name ("none"/"viewer"/"operator"/"admin"). Never NULL. */
const char *mp_role_name(uint8_t role);

/** Display-rail minimum off-time before it may be re-enabled. */
#ifndef MP_DISP_MIN_OFF_MS
#define MP_DISP_MIN_OFF_MS 1000U
#endif

/** Grace before the VCC_RB read-back is judged, and its tolerance. */
#ifndef MP_RB_VERIFY_DELAY_MS
#define MP_RB_VERIFY_DELAY_MS 250U
#endif
#ifndef MP_RB_VERIFY_TOL_PCT
#define MP_RB_VERIFY_TOL_PCT 10U
#endif

/**
 * How long a lease may stay UNSETTLED before it is reverted as failed.
 *
 * A glue whose actuator lives on another thread answers MP_APPLY_PENDING and
 * confirms later with mp_ovr_settled(); until it does, the lease exists but the
 * pin has not moved, and the reply says so (see MP_APPLY_PENDING). This is the
 * deadline on "later", and past it the lease is dropped exactly as a failed
 * VCC_RB read-back is — same event, same fail-safe reading of silence.
 *
 * 3000 ms is set from the worst case the shipped glue can actually produce, not
 * picked round: the request is drained on the platform's 250 ms sequencer pass,
 * and the confirmation is taken on the console supervisor's own pass, which
 * mp_glue.h budgets at up to `(STS_MP_TICK_MISS_MAX + 1) * (250 + 50)` = 1800 ms
 * between two SUCCESSFUL ticks when the shell thread is contending. 250 + 1800 =
 * 2050 ms worst case, so a deadline at 3000 ms cannot fire on a merely busy
 * board — which matters, because a spurious drop here would revert a lease that
 * was working.
 */
#ifndef MP_APPLY_SETTLE_MS
#define MP_APPLY_SETTLE_MS 3000U
#endif

/**
 * mp_ovr_apply_fn's "accepted, but the pin has not moved yet".
 *
 * Positive, because both call sites have always tested `rc < 0` for failure and
 * therefore already treat any positive value as success; this gives that unused
 * range one documented meaning rather than adding a parameter.
 *
 * WHY IT EXISTS. Most of this board's control objects are GPIO/PWM the platform
 * area owns, and that area is single-writer by construction — every rail is
 * driven from the housekeeping thread's 4 Hz sequencer pass, because the action
 * ring it feeds has no lock and a second writer would be appending to a queue
 * whose consumer is mid-drain. So a glue that routes a write to that thread
 * cannot report, at return, that the pin moved. It has not.
 *
 * Answering 0 anyway would make `obj.override`'s reply claim an effect that has
 * not happened, which is precisely the class of untruth this protocol's replies
 * are supposed to be free of. Answering an error would be worse: the override
 * WAS accepted, and reporting a veto sends a technician looking for an interlock
 * that did not fire.
 *
 * So the lease is granted — the authorisation is immediate and real — and it is
 * marked unsettled. `mp_ilk_res_t::pending` carries that out to the RPC layer,
 * which folds it into the reply's existing `verify_pending` key: that key
 * already means "not confirmed; a check is scheduled and this lease may yet be
 * dropped", which is exactly the situation. The glue then either calls
 * mp_ovr_settled() when its drain confirms, or mp_ovr_veto() when the owning
 * subsystem refuses; silence past MP_APPLY_SETTLE_MS is treated as failure.
 * That is MP_ILK_RB_VERIFY's read-back-and-revert shape, reused rather than
 * reinvented.
 */
#define MP_APPLY_PENDING 1

/* ------------------------------------------------------- interlock inputs */

/**
 * Live device state the interlocks are evaluated against.
 *
 * Filled by the glue from snapshots — never by reading hardware from inside
 * core, and never while holding a timing lock (spec §10). Every field is a
 * plain value so a unit test can construct any interlock scenario.
 *
 * `rb_code_min`/`rb_code_max` are the *permitted digipot code window*, computed
 * by the glue from `pwrseq`'s own VCC_RB transfer function and the configured
 * ceiling. Handing core a window rather than the transfer function keeps the
 * single source of that math in `pwrseq` and makes this module independent of
 * the code-to-voltage polarity.
 */
typedef struct {
	/* rubidium chain */
	bool ocxo_warm;      /**< oven at temperature (INA 0x46 + TMP117) */
	bool supercaps_ok;   /**< both backup rails good, PoE budget covered */
	bool rb_ov_latched;  /**< RB_OV_DET (PE3) autonomous 26 V latch */
	int32_t vcc_rb_mv;   /**< INA228 0x47 bus reading */
	bool vcc_rb_valid;   /**< that reading is trustworthy */
	int32_t rb_expected_mv; /**< rail voltage the programmed code implies */
	uint16_t rb_vmax_mv; /**< cfg pwr.rb.vmax.mv */
	uint16_t rb_code_min;
	uint16_t rb_code_max;

	/* thermal */
	uint8_t fan_floor_pct; /**< thermal loop's minimum duty */

	/* relay / faults */
	bool relay_blocked; /**< a disqualifying alarm is active */

	/* display */
	bool disp_on;            /**< DISP_EN currently asserted */
	uint32_t disp_changed_ms;/**< when DISP_EN last changed */

	/* supervisor */
	bool liveness_ok; /**< every registered liveness participant is fed */

	/* timing */
	bool disc_parked; /**< the discipline loop is parked (DAC free) */
	bool extref_ok;   /**< EXTREF_MON in band */
	bool rb_lock;     /**< RB_LOCK asserted */
} mp_ilk_state_t;

/** Outcome of an interlock evaluation. */
typedef struct {
	int32_t value;   /**< the value to apply, possibly clamped */
	bool clamped;    /**< an interlock reduced the request */
	bool tunnel;     /**< MP_ILK_TUNNEL applies: reference becomes suspect */
	bool verify;     /**< MP_ILK_RB_VERIFY applies: schedule a read-back */
	/**
	 * The apply answered MP_APPLY_PENDING: the lease is granted but the pin
	 * has not moved yet.
	 *
	 * NOT set by mp_ilk_eval() — it is not an interlock verdict and no
	 * manifest row declares it. It is written by mp_ovr_grant() from the
	 * apply callback's return, and it rides in this struct because this is
	 * already what mp_ovr_grant() hands back to the RPC layer for the reply.
	 */
	bool pending;
	uint32_t failed; /**< the interlock bit that refused, or 0 */
} mp_ilk_res_t;

/**
 * Evaluate every interlock declared by object @p obj against @p st.
 *
 * @param obj      Manifest index.
 * @param req      Requested value.
 * @param st       Live state; NULL is treated as "nothing is known", which
 *                 fails every interlock that has a positive precondition.
 * @param now_ms   Monotonic milliseconds (for the display off-time).
 * @param out      Receives the outcome.
 *
 * @retval 0        Permitted; @p out->value is what to apply.
 * @retval -EINVAL  Bad argument or object index.
 * @retval -EPERM   Refused; @p out->failed names the interlock.
 */
int mp_ilk_eval(size_t obj, int32_t req, const mp_ilk_state_t *st,
		uint32_t now_ms, mp_ilk_res_t *out);

/* ---------------------------------------------------------------- session */

/** One maintenance session. Exactly one is supported at a time (FMT §5.1). */
typedef struct {
	uint32_t id; /**< 0 = closed */
	uint32_t opened_ms;
	uint32_t keepalive_ms; /**< last keepalive or successful request */
	uint32_t ttl_ms;       /**< negotiated, <= MP_KEEPALIVE_TTL_MS */
	bool serial_ok;        /**< G2 device-serial confirmation satisfied */

	/**
	 * Role granted at open (MP_ROLE_*), and the name it was granted to.
	 *
	 * Both are set by mp_ovr_session_open() and by nothing else, so a
	 * takeover cannot inherit the previous caller's privilege. `user` is
	 * empty when no credential was presented; it is what an audit record
	 * names (FMT §5.3), and it is never the credential itself.
	 */
	uint8_t role;
	char user[MP_USER_MAX + 1U];

	/* G3 arm state: one armed action per session. */
	uint32_t arm_nonce;
	uint16_t arm_obj1;    /**< manifest index + 1, or 0 */
	uint8_t arm_tag;      /**< caller-defined action tag for non-object G3 */
	uint32_t arm_ready_ms;/**< earliest completion */
	uint32_t arm_expire_ms;
} mp_session_t;

/* -------------------------------------------------------------- leases */

typedef struct {
	uint16_t obj1; /**< manifest index + 1; 0 = free */
	uint32_t sid;
	int32_t value;
	uint32_t granted_ms;
	uint32_t deadline_ms;
	/* MP_ILK_RB_VERIFY bookkeeping */
	uint32_t verify_at_ms; /**< 0 = nothing pending */
	int32_t verify_expect_mv;
	/**
	 * MP_APPLY_PENDING bookkeeping: the instant past which an unconfirmed
	 * apply is treated as failed. 0 = settled (the ordinary case).
	 */
	uint32_t settle_by_ms;
} mp_lease_t;

/* -------------------------------------------------------------- callbacks */

/**
 * Apply or release an override.
 *
 * @param obj    Manifest index.
 * @param value  Value to apply, or NULL to return the object to
 *               firmware-automatic control.
 * @retval 0     Applied/released.
 * @retval <0    Refused. On an apply this becomes a veto; on a release it is
 *               counted (mp_ovr_ctx_t::release_errors) and the lease is dropped
 *               anyway — a lease core cannot release is worse than one the glue
 *               failed to clear.
 */
typedef int (*mp_ovr_apply_fn)(void *user, size_t obj, const int32_t *value);

/** Event kinds emitted by this module. */
typedef enum {
	MP_OVR_EV_GRANT = 0,  /**< a lease was granted or replaced */
	MP_OVR_EV_RELEASE,    /**< released by the owner */
	MP_OVR_EV_EXPIRE,     /**< the lease TTL ran out */
	MP_OVR_EV_DEADMAN,    /**< dead-man reverted everything */
	MP_OVR_EV_VETO,       /**< firmware refused or withdrew */
	MP_OVR_EV_VERIFY_FAIL,/**< post-set read-back mismatch, auto-reverted */
	MP_OVR_EV_COUNT,
} mp_ovr_ev_t;

/** Stable event name for the wire and the log. Never NULL. */
const char *mp_ovr_ev_name(uint8_t ev);

/**
 * Event sink. Called from mp_ovr_* with the ctx unlocked; must not re-enter
 * this module. @p reason may be NULL.
 */
typedef void (*mp_ovr_evt_fn)(void *user, uint8_t ev, size_t obj, uint32_t sid,
			      const char *reason);

/* ------------------------------------------------------------------- ctx */

typedef struct {
	mp_session_t sess;
	mp_lease_t lease[MP_LEASE_MAX];

	mp_ovr_apply_fn apply;
	void *apply_user;
	mp_ovr_evt_fn evt;
	void *evt_user;

	/** Device serial; the G2 confirmation string. Empty = G2 unusable. */
	char serial[MP_CONFIRM_MAX];
	/** G3 phrase. Defaults to MP_G3_PHRASE_DEFAULT. */
	char phrase[MP_CONFIRM_MAX];

	bool link_up; /**< physical link asserted (CDC DTR / MP mode entered) */
	uint32_t next_sid;
	uint32_t last_tick_ms;
	bool ticked;

	/* counters */
	uint32_t grants;
	uint32_t releases;
	uint32_t expiries;
	uint32_t deadman_reverts;
	uint32_t vetoes;
	uint32_t verify_failures;
	uint32_t refusals;      /**< guard or interlock refusals */
	uint32_t release_errors;
	uint32_t late_ticks;    /**< ticks later than MP_TICK_MAX_MS */
	/** Leases that reached the actuator after an MP_APPLY_PENDING grant. */
	uint32_t settled;
	/** Leases dropped because an MP_APPLY_PENDING grant never landed. */
	uint32_t settle_failures;
} mp_ovr_ctx_t;

/**
 * Initialise. @p serial may be NULL (G2 then always refuses, which is the
 * fail-safe answer for an unprovisioned board).
 *
 * @retval 0        Initialised.
 * @retval -EINVAL  @p c or @p apply is NULL.
 */
int mp_ovr_init(mp_ovr_ctx_t *c, mp_ovr_apply_fn apply, void *apply_user,
		mp_ovr_evt_fn evt, void *evt_user, const char *serial);

/** Replace the G3 phrase. NULL or "" restores MP_G3_PHRASE_DEFAULT. */
int mp_ovr_set_phrase(mp_ovr_ctx_t *c, const char *phrase);

/**
 * Report the physical link state.
 *
 * A transition to false is a dead-man failure: every override reverts on the
 * spot, not at the next tick, because a lost link is unambiguous.
 */
int mp_ovr_set_link(mp_ovr_ctx_t *c, bool up, uint32_t now_ms);

/* ------------------------------------------------------------- session API */

/**
 * Open the single session with an already-granted role.
 *
 * Authentication happens **before** this call, in the control plane, against the
 * platform's credential store; this function only records the verdict. Taking
 * the role as an argument rather than exposing a later "authorize" call is what
 * makes it impossible to leave a session open carrying the previous caller's
 * privilege: the role and the user are written in the same operation that clears
 * the old session, so there is no window in between.
 *
 * @param ttl_ms   Requested keepalive TTL; clamped to MP_KEEPALIVE_TTL_MS.
 * @param role     Granted role (MP_ROLE_*). MP_ROLE_NONE for an unauthenticated
 *                 session, which is legal and useful — it can still observe.
 *                 A value above MP_ROLE_ADMIN is not a super-admin: it is an
 *                 unrecognised role and is recorded as MP_ROLE_NONE.
 * @param user     Authenticated user name, or NULL/"" when none. Truncated to
 *                 MP_USER_MAX. Never a credential.
 * @param out_sid  Receives the new session id (never 0).
 *
 * @retval 0        Opened. Any previous session is closed and its overrides
 *                  reverted first — a stale session must not keep the board
 *                  commanded once a new tool has taken over.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOLINK The physical link is not asserted.
 */
int mp_ovr_session_open(mp_ovr_ctx_t *c, uint32_t ttl_ms, uint8_t role,
			const char *user, uint32_t now_ms, uint32_t *out_sid);

/** The open session's granted role, or MP_ROLE_NONE when there is none. */
uint8_t mp_ovr_session_role(const mp_ovr_ctx_t *c);

/**
 * The open session's user name. Never NULL; "" when unauthenticated or closed.
 *
 * Borrowed from the ctx, so it is only valid until the session changes. This is
 * the `user` field of the §5.3 audit record.
 */
const char *mp_ovr_session_user(const mp_ovr_ctx_t *c);

/**
 * Refresh the keepalive.
 *
 * @retval 0        Refreshed.
 * @retval -EINVAL  @p c is NULL.
 * @retval -ENOENT  No such session (closed, or a different id).
 */
int mp_ovr_keepalive(mp_ovr_ctx_t *c, uint32_t sid, uint32_t now_ms);

/** Close @p sid, reverting its overrides. -ENOENT for an unknown id. */
int mp_ovr_session_close(mp_ovr_ctx_t *c, uint32_t sid, uint32_t now_ms);

/** True when @p sid is the open session and its keepalive is fresh. */
bool mp_ovr_session_valid(const mp_ovr_ctx_t *c, uint32_t sid, uint32_t now_ms);

/** Milliseconds until the open session goes stale; 0 when it already has. */
uint32_t mp_ovr_session_remaining(const mp_ovr_ctx_t *c, uint32_t now_ms);

/* --------------------------------------------------------------- guards */

/** Guard-check outcome. */
typedef enum {
	MP_GC_OK = 0,        /**< proceed */
	MP_GC_NEED_SESSION,  /**< G1+: no open session */
	MP_GC_STALE,         /**< the session's keepalive has expired */
	MP_GC_NEED_SERIAL,   /**< G2: confirmation absent or wrong */
	MP_GC_NEED_PHRASE,   /**< G3: phrase absent or wrong */
	MP_GC_ARMED,         /**< G3: armed; repeat with the nonce after the hold */
	MP_GC_NEED_HOLD,     /**< G3: armed but the hold has not elapsed */
	MP_GC_ARM_MISMATCH,  /**< G3: nonce does not match the armed action */
	/**
	 * The session is valid but its role is below the class minimum (§5.3).
	 *
	 * Appended rather than inserted next to MP_GC_STALE so the existing
	 * outcome numbers — which appear in the wire error `data.reason` through
	 * mp_gc_name() — do not shift.
	 */
	MP_GC_NEED_ROLE,
	MP_GC_COUNT,
} mp_gc_t;

/** Stable name for a guard outcome, for the error `data`. Never NULL. */
const char *mp_gc_name(uint8_t gc);

/** Extra output from a G3 arming. */
typedef struct {
	uint32_t nonce;
	uint32_t hold_ms;
	uint32_t expires_ms; /**< milliseconds from now */
} mp_gc_arm_t;

/**
 * Evaluate the guard for an action.
 *
 * The escalation is cumulative, so G3 needs **both** confirmations: the device
 * serial in @p confirm (the G2 requirement, which does not go away) and the
 * phrase in @p phrase. They are separate fields rather than one, because one
 * field could only ever carry one of them and the spec asks for both.
 *
 * The role requirement is checked **before** either confirmation, so an operator
 * attempting a G2 action is told its role is insufficient rather than being sent
 * to find a serial number that would not have helped. Neither string is a
 * secret — the serial is printed on the chassis and the phrase is in the manual
 * — so ordering the checks for a useful message leaks nothing:
 *
 *   G0  nothing.
 *   G1  a fresh session, role >= MP_ROLE_OPERATOR.
 *   G2  G1 + role >= MP_ROLE_ADMIN + the typed device serial.
 *   G3  G2 + the typed phrase + the arm/hold.
 *
 * A @p guard value above MP_GUARD_G3 is treated as G3, which is the fail-safe
 * reading of an unknown class.
 *
 * @param guard    Required class (mp_guard_t).
 * @param sid      Caller's session id (0 when it presented none).
 * @param obj1     Manifest index + 1 for an object action, else 0.
 * @param tag      Action tag for a non-object G3 action (sys.reboot, a G3
 *                 diagnostic). Ignored when @p obj1 is non-zero.
 * @param confirm  The request's `confirm` string (device serial), or NULL.
 * @param phrase   The request's `phrase` string (G3 arming only), or NULL. Not
 *                 required again on the second phase: it has been typed, and
 *                 the nonce is what binds the completion to it.
 * @param nonce    The request's `nonce` (G3 second phase), or 0.
 * @param arm      Optional; filled when the result is MP_GC_ARMED.
 *
 * @return An mp_gc_t. Only MP_GC_OK permits the action. A successful check also
 *         refreshes the session keepalive: a tool that is issuing commands is
 *         demonstrably alive.
 */
int mp_ovr_guard(mp_ovr_ctx_t *c, uint8_t guard, uint32_t sid, uint16_t obj1,
		 uint8_t tag, const char *confirm, const char *phrase,
		 uint32_t nonce, uint32_t now_ms, mp_gc_arm_t *arm);

/** Drop any armed G3 action on the open session. */
void mp_ovr_disarm(mp_ovr_ctx_t *c);

/* --------------------------------------------------------------- leases */

/**
 * Grant (or replace) an override lease on @p obj.
 *
 * The guard must already have been satisfied by mp_ovr_guard(); this function
 * enforces the manifest's MP_OF_OVERRIDE flag, the value envelope and every
 * declared interlock, then calls the apply callback.
 *
 * @param ttl_ms  Requested lifetime; 0 selects MP_LEASE_TTL_DEFAULT_MS, and
 *                anything above MP_LEASE_TTL_MAX_MS is clamped down.
 * @param st      Interlock state.
 * @param res     Optional; receives the interlock outcome (clamping, tunnel).
 *
 * @retval 0         Granted.
 * @retval -EINVAL   Bad argument or object index.
 * @retval -ENOTSUP  The object is not overridable, OR the apply callback
 *                   answered -ENOTSUP — nothing is wired behind the object.
 *                   Deliberately NOT reported as a veto: no MP_OVR_EV_VETO is
 *                   emitted and `vetoes` does not move, because telling a
 *                   technician that safety supervision refused them sends them
 *                   looking for an interlock that does not exist. See the
 *                   comment on the apply-failure path in mp_override.c.
 * @retval -ERANGE   Value outside the object's envelope.
 * @retval -EPERM    An interlock refused; @p res->failed names it.
 * @retval -ENOLINK  Link down or session invalid.
 * @retval -ENOSPC   Lease table full.
 * @retval -EACCES   The apply callback vetoed a value it understood.
 */
int mp_ovr_grant(mp_ovr_ctx_t *c, size_t obj, int32_t value, uint32_t ttl_ms,
		 uint32_t sid, const mp_ilk_state_t *st, uint32_t now_ms,
		 mp_ilk_res_t *res);

/**
 * Release the lease on @p obj held by @p sid.
 *
 * @retval 0        Released.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOENT  No lease on that object.
 * @retval -EACCES  The lease belongs to a different session.
 */
int mp_ovr_release(mp_ovr_ctx_t *c, size_t obj, uint32_t sid, uint32_t now_ms);

/**
 * Confirm that an MP_APPLY_PENDING grant reached the actuator.
 *
 * Called by the glue when its own drain reports the write landed. Clears the
 * settle deadline so mp_ovr_tick() stops watching the lease; the lease itself,
 * its value and its TTL are untouched — this reports an observation, it does not
 * re-grant anything.
 *
 * Idempotent, and deliberately so: the glue's outcome queue is last-writer-wins
 * per object, so a confirmation may legitimately arrive for a lease that has
 * already settled, and that must not be an error.
 *
 * @retval 0        The lease is settled (whether or not it already was).
 * @retval -EINVAL  @p c is NULL or @p obj is not a manifest index.
 * @retval -ENOENT  No lease on that object — the usual answer when a lease was
 *                  released or vetoed between the request and its confirmation.
 */
int mp_ovr_settled(mp_ovr_ctx_t *c, size_t obj, uint32_t now_ms);

/** True when @p obj's lease exists and has not yet been confirmed applied. */
bool mp_ovr_pending(const mp_ovr_ctx_t *c, size_t obj);

/**
 * Firmware veto: the owning subsystem withdraws an override.
 *
 * Always succeeds when a lease exists; firmware outranks the tool by
 * construction. @p reason is carried to the event sink.
 *
 * @retval 0        Vetoed and reverted.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOENT  No lease on that object.
 */
int mp_ovr_veto(mp_ovr_ctx_t *c, size_t obj, const char *reason,
		uint32_t now_ms);

/** Revert every lease. @p ev is the event kind reported for the batch. */
int mp_ovr_revert_all(mp_ovr_ctx_t *c, uint8_t ev, const char *reason,
		      uint32_t now_ms);

/** The lease on @p obj, or NULL when the object is not overridden. */
const mp_lease_t *mp_ovr_lease(const mp_ovr_ctx_t *c, size_t obj);

/** Number of live leases. */
size_t mp_ovr_active(const mp_ovr_ctx_t *c);

/* ----------------------------------------------------------------- tick */

/**
 * Periodic maintenance: the dead-man, lease expiry and read-back verification.
 *
 * Must be called at least every MP_TICK_MAX_MS (see the file header).
 *
 * @param st  Interlock state, used for the VCC_RB read-back check. May be NULL,
 *            in which case a pending verification is treated as failed —
 *            "cannot verify" and "verified bad" get the same fail-safe answer.
 *
 * @retval >=0      Number of leases reverted by this tick.
 * @retval -EINVAL  @p c is NULL.
 */
int mp_ovr_tick(mp_ovr_ctx_t *c, const mp_ilk_state_t *st, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_MP_MP_OVERRIDE_H_ */
