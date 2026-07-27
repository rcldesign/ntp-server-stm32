/*
 * STS1000 "Meridian" — core/refsel: the reference-selection state machine.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation, no platform headers, no globals.
 *
 * Implements spec §3.5. The clock mux (74LVC1G157 U52) forwards one of two
 * 10 MHz inputs to PH0: input A is the always-on OCXO, input B is whatever the
 * external SMA front end delivers — the FE-5680A rubidium in the normal build,
 * or a house 10 MHz standard. `MUX_SEL` (PB6) is the only control bit and is
 * 0 = A at reset (R188 100 k pull-down).
 *
 * ARCHITECTURE.md §10 invariant 1: PB6 is written only by this module's action
 * executor, and only through the HSI-bridge sequence. This module is pure
 * decision logic — it never touches hardware. It emits an ordered action list
 * that the Zephyr glue executes:
 *
 *     BRIDGE_TO_HSI -> SET_MUX(target) -> WAIT_SETTLE_MS(n)
 *                   -> RESELECT_HSE -> VERIFY_PLL -> DONE
 *
 * A plain selector is used rather than a glitch-free mux IC on purpose (see
 * docs/sts1000_clock_mux.md §0): a glitch-free part completes its handoff on
 * the *outgoing* clock's edges and hangs when that source has stopped, which is
 * exactly the Rb-died case this machine exists to survive.
 *
 * Guard policy (§3.5), stated as the machine implements it:
 *
 *   OCXO -> B   only when EXTREF_MON reports a real, in-band ~10 MHz AND
 *               (for the rubidium) RB_LOCK is asserted, both continuously true
 *               for the hysteresis window, and no lockout is running.
 *   B -> OCXO   the same tick either guard drops. Fail-safe, no debounce, no
 *               conditions. A revert starts a lockout so the machine cannot
 *               flap, and counts towards the flap alarm.
 *
 * The glue reports a failed handoff (HSERDY never came, PLL would not lock, CSS
 * fired) through refsel_in_t::switch_failed; the machine falls back to the
 * OCXO, raises an alarm flag and starts a lockout.
 */

#ifndef STS1000_CORE_REFSEL_REFSEL_H_
#define STS1000_CORE_REFSEL_REFSEL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Longest action list the machine can emit (the sequence above). */
#define REFSEL_MAX_STEPS 6u

/** Revert timestamps retained for the flap window. */
#define REFSEL_FLAP_HISTORY 8u

/* ------------------------------------------------------------------ states */

/** Selected reference (spec §3.5, §11). */
typedef enum {
	REFSEL_OCXO_ACTIVE = 0, /**< mux input A, MUX_SEL = 0, boot default */
	REFSEL_RB_ACTIVE,       /**< mux input B, FE-5680A rubidium */
	REFSEL_EXTREF_ACTIVE,   /**< mux input B, external house standard */
	REFSEL_STATE__COUNT
} refsel_state_t;

/** Operator intent. AUTO is the normal setting. */
typedef enum {
	REFSEL_REQ_AUTO = 0,   /**< engage the Rb whenever the guards allow */
	REFSEL_REQ_FORCE_OCXO, /**< pin the OCXO; revert now if not already */
	REFSEL_REQ_EXTREF,     /**< explicit operator selection of input B as a
				*   house standard; never entered automatically */
	REFSEL_REQ__COUNT
} refsel_request_t;

/** One step of the glitchless handoff, for the glue to execute in order. */
typedef enum {
	REFSEL_ACT_NONE = 0,
	REFSEL_ACT_BRIDGE_TO_HSI, /**< SYSCLK <- HSI before touching the mux */
	REFSEL_ACT_SET_MUX,       /**< write PB6; arg 0 = input A, 1 = input B */
	REFSEL_ACT_WAIT_SETTLE_MS,/**< arg = milliseconds to let the mux settle */
	REFSEL_ACT_RESELECT_HSE,  /**< re-enable HSE bypass, poll HSERDY */
	REFSEL_ACT_VERIFY_PLL,    /**< re-lock the PLL, re-arm CSS, SYSCLK <- PLL */
	REFSEL_ACT_DONE,
	REFSEL_ACT__COUNT
} refsel_action_t;

/** Argument value for REFSEL_ACT_SET_MUX. */
#define REFSEL_MUX_A 0u /**< OCXO */
#define REFSEL_MUX_B 1u /**< external / rubidium */

typedef struct {
	refsel_action_t act;
	uint32_t arg;
} refsel_step_t;

/* Published condition flags. */
#define REFSEL_FLAG_EXTREF_OK     (1u << 0) /**< in-band 10 MHz on input B */
#define REFSEL_FLAG_RB_OK         (1u << 1) /**< RB_LOCK asserted */
#define REFSEL_FLAG_GUARDS_STABLE (1u << 2) /**< guards held past hysteresis */
#define REFSEL_FLAG_LOCKOUT       (1u << 3) /**< re-engage inhibited */
#define REFSEL_FLAG_FLAP_ALARM    (1u << 4) /**< sticky; refsel_clear_flap() */
#define REFSEL_FLAG_SWITCH_FAILED (1u << 5) /**< glue reported a failed handoff */
#define REFSEL_FLAG_FORCED_OCXO   (1u << 6) /**< operator pinned the OCXO */

/* ------------------------------------------------------------------ config */

typedef struct {
	/** Nominal input-B frequency. Both references are 10 MHz. */
	uint32_t extref_nominal_hz;
	/**
	 * Acceptance half-band in hertz. The default 20 Hz is +-2 ppm of
	 * 10 MHz; expressed in Hz rather than ppm because TIM12 measures a
	 * frequency, and the achievable granularity is a property of the gate
	 * time the glue uses, not of the specification.
	 */
	uint32_t extref_band_hz;
	/** Both guards must hold this long before input B may be engaged. */
	uint32_t hysteresis_ms;
	/** After a revert, input B may not be re-engaged for this long. */
	uint32_t lockout_ms;
	/** Settle time handed to the glue in the WAIT_SETTLE_MS step. */
	uint32_t settle_ms;
	/** Window over which reverts are counted for the flap alarm. */
	uint32_t flap_window_ms;
	/** Reverts within the window that raise REFSEL_FLAG_FLAP_ALARM. */
	uint8_t flap_alarm_n;
	/**
	 * Whether REFSEL_REQ_EXTREF also requires RB_LOCK.
	 *
	 * Default false. Input B is shared: the rubidium reaches it over coax
	 * through the same SMA front end a house standard would use, and a
	 * house standard has no lock line to assert. The guard machinery
	 * (band check, hysteresis, immediate revert, lockout, flap counting)
	 * is identical either way; this bit only decides whether RB_LOCK is
	 * one of the guarded conditions. Set it true on a build where input B
	 * is always the FE-5680A.
	 */
	bool extref_require_rb_lock;
} refsel_cfg_t;

/* ----------------------------------------------------------------- context */

typedef struct {
	refsel_cfg_t cfg;
	bool initialised;

	refsel_state_t state;

	/* Guard debounce. The window belongs to a particular target, because
	 * REFSEL_REQ_AUTO and REFSEL_REQ_EXTREF may guard on different
	 * conditions; changing the request restarts it. */
	refsel_state_t tracked_target;
	bool guards_have_since;
	uint64_t guards_since_ms;

	/* Lockout */
	bool lockout;
	uint64_t lockout_until_ms;

	/* Flap accounting */
	uint64_t revert_ms[REFSEL_FLAP_HISTORY];
	uint8_t revert_n;
	uint8_t revert_head;
	uint16_t revert_total;
	bool flap_alarm;

	bool switch_failed_latched;

	/* Action list produced by the last transition decision */
	refsel_step_t steps[REFSEL_MAX_STEPS];
	size_t n_steps;
} refsel_ctx_t;

/* ------------------------------------------------------------------ inputs */

typedef struct {
	uint64_t mono_ms;

	/** TIM12_CH1 sees edges arriving on input B at all. */
	bool extref_edges_advancing;
	/** Measured input-B frequency, hertz. */
	uint32_t extref_freq_hz;
	/** The frequency measurement is fresh; false fails the guard. */
	bool extref_valid;

	/**
	 * FE-5680A lock, already polarity-corrected by the caller. The opto
	 * (U48) inverts and the polarity is a config bit (interface ref §9), so
	 * this module never has to know which way round the board is wired.
	 */
	bool rb_lock;

	refsel_request_t request;

	/**
	 * The glue could not complete the last handoff: HSERDY timed out, the
	 * PLL would not re-lock, or CSS fired. Edge-style — assert it for one
	 * step call. Forces an immediate fall back to the OCXO.
	 */
	bool switch_failed;
} refsel_in_t;

/* ----------------------------------------------------------------- outputs */

typedef struct {
	refsel_state_t state;      /**< state after this step */
	bool transition;           /**< a handoff was decided this step */
	refsel_state_t from;       /**< state before, valid when transition */
	uint32_t flags;            /**< REFSEL_FLAG_* */
	uint16_t revert_count;     /**< reverts since init (or clear_flap) */
	uint32_t lockout_remain_ms;/**< 0 when no lockout is running */
} refsel_out_t;

/* --------------------------------------------------------------------- API */

/**
 * Fill @p cfg with the documented defaults: 10 MHz +-20 Hz, 30 s hysteresis,
 * 300 s lockout, 10 ms settle, 3 reverts in 1 h raises the flap alarm.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p cfg is NULL.
 */
int refsel_cfg_defaults(refsel_cfg_t *cfg);

/**
 * Initialise the machine in REFSEL_OCXO_ACTIVE, matching the hardware default
 * that R188 forces before firmware runs.
 *
 * @param ctx  Context.
 * @param cfg  Configuration, or NULL for refsel_cfg_defaults().
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p ctx is NULL, or @p cfg fails validation.
 */
int refsel_init(refsel_ctx_t *ctx, const refsel_cfg_t *cfg);

/**
 * Evaluate the guards and decide whether to hand the mux over.
 *
 * Call at a steady rate (the housekeeping thread's 1-4 Hz is ample; the
 * hysteresis and lockout are measured in wall time, not ticks). When
 * @p out->transition is true the caller must fetch the action list with
 * refsel_actions() and execute every step in order before the next call.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p ctx or @p in is NULL, or the context is not initialised.
 */
int refsel_step(refsel_ctx_t *ctx, const refsel_in_t *in, refsel_out_t *out);

/**
 * Action list produced by the most recent transition decision.
 *
 * Valid until the next refsel_step(). Empty (@p *n == 0) when the last step
 * decided nothing.
 *
 * @param ctx    Context.
 * @param steps  Receives a pointer to the internal array.
 * @param n      Receives the number of valid steps.
 *
 * @retval 0        Success.
 * @retval -EINVAL  Any argument NULL, or the context is not initialised.
 */
int refsel_actions(const refsel_ctx_t *ctx, const refsel_step_t **steps,
		   size_t *n);

/**
 * Clear the sticky flap alarm and its revert history (an operator ack).
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p ctx is NULL or not initialised.
 */
int refsel_clear_flap(refsel_ctx_t *ctx);

/** Current state. REFSEL_STATE__COUNT for a NULL/uninitialised context. */
refsel_state_t refsel_state(const refsel_ctx_t *ctx);

/** Human-readable state name, for logs and the UI. Never NULL. */
const char *refsel_state_name(refsel_state_t s);

/** Human-readable action name, for logs. Never NULL. */
const char *refsel_action_name(refsel_action_t a);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_REFSEL_REFSEL_H_ */
