/*
 * STS1000 "Meridian" — the K1 RS-232/CMOS relay rule, as pure logic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/platform/, and deliberately free of every Zephyr
 * dependency so tests/host can compile it — the same arrangement as
 * sts_rbguard.h, for the same reason.
 *
 * platform/rb_serial.c keeps what only Zephyr can do: the GPIO write, the 20 ms
 * contact settle, the UART7 ISR and its ring. What it must not also keep is the
 * decision below, because there the decision has no test at all: the .c needs a
 * devicetree, an ISR and k_msleep() to link, so a host suite cannot execute it,
 * and a fixture that simulated the relay instead would end up asserting against
 * its own second copy of this rule. The state and the verdicts therefore live
 * here, where the suite runs the same code the firmware runs.
 *
 * ---------------------------------------------------------------------------
 * The rule: a refused move TO the fail-safe position is owed, not cancelled
 * ---------------------------------------------------------------------------
 *
 * rb_serial_set_mode() refuses while a raw tunnel holds UART7: throwing a
 * mechanical relay under a live passthrough breaks whatever the host is
 * mid-transaction with, and the host is the one party here nobody can ask. That
 * refusal is correct and it stays. Its consequence is what this file exists for.
 *
 * The maintenance override engine reverts its leases in slot order and a
 * release that fails is counted and then forgotten (core/mp lease_drop()). So:
 * lease K1 to CMOS, open a tunnel, pull the USB cable. The dead-man releases the
 * K1 lease first, that release arrives as set_mode(RS232), it is refused because
 * the tunnel is still open, and the *next* release closes the tunnel. Nothing
 * then owns the relay and nothing ever moves it: the board runs on with UART7 in
 * a commissioning position no lease holds, and FE-5680A housekeeping telemetry
 * is silently dead until somebody reboots it.
 *
 * So a refused move to the fail-safe position is remembered, and paid by the
 * close. The asymmetry between the two directions is the whole design:
 *
 *   RS-232 refused  ->  STS_RB_MOVE_OWE. Deferred, and applied on close. It is
 *                       the reset position, the documented fail-safe
 *                       (docs/rb_rs232_interface.md) and what rb_serial_init()
 *                       leaves the relay in. Something asked for the safe state
 *                       and could not have it *yet*; honouring it at the first
 *                       moment it is possible is what "fail-safe" means.
 *
 *   CMOS refused    ->  STS_RB_MOVE_REFUSE. Just refused. Leaving the fail-safe
 *                       position is a deliberate commissioning act for a
 *                       specific FE variant. A technician who was told -EBUSY
 *                       must re-issue it knowingly; a relay that quietly threw
 *                       itself into the commissioning position some minutes
 *                       after the command that asked for it was rejected would
 *                       be worse than the bug this replaces.
 *
 * Restoring UNCONDITIONALLY on close was the obvious alternative and it is
 * wrong, because a K1 lease and a tunnel lease are independent and simultaneous
 * by design — set CMOS *because* the variant needs CMOS, then tunnel to talk to
 * it. Closing only the tunnel would then yank the relay back to RS-232 while
 * that lease still stood, undoing the technician's commissioning decision in the
 * middle of their session and leaving core/mp's lease table claiming a position
 * the pin no longer holds. Nothing on this side of the seam can see the lease
 * table, so "restore only what was actually asked for" is the strongest
 * invariant it can carry — and it needs no help from the layer or the ordering
 * that releases the lease.
 *
 * ---------------------------------------------------------------------------
 * The ordering the close must honour, and why it is encoded rather than stated
 * ---------------------------------------------------------------------------
 *
 * STS_RB_CLOSE_RESTORE is "give the port back, THEN move the relay", and the
 * order is the mechanism rather than a tidiness preference: the deferred move is
 * an ordinary sts_rb_move_decide() like any other, so issued while the tunnel is
 * still open it hits the same refusal that armed the latch in the first place —
 * it refuses itself, re-arms, and the relay never moves. That is not a comment
 * asking to be believed: feed sts_rb_move_decide() the two tunnel states in turn
 * and it answers STS_RB_MOVE_OWE before the retraction and STS_RB_MOVE_APPLY
 * after it, which is the contract, executable.
 */

#ifndef STS1000_ZEPHYR_PLATFORM_STS_RB_SERIAL_POLICY_H_
#define STS1000_ZEPHYR_PLATFORM_STS_RB_SERIAL_POLICY_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Where the K1 DPDT relay can stand.
 *
 * Numerically identical to platform.h's rb_serial_mode_t, which is the type the
 * rest of the firmware passes around; rb_serial.c carries the BUILD_ASSERTs that
 * hold the two together. They are restated here rather than included because
 * platform.h pulls in devicetree and the GPIO API, and this header must stay
 * compilable off-target.
 */
typedef enum {
	/** Through the SN65C3221E level shifter (U46). The fail-safe position:
	 *  the reset state, and what the pin holds LOW. */
	STS_RB_POS_RS232 = 0,
	/** Direct CMOS, for an FE-5680A variant that needs it. A commissioning
	 *  position, never a reset state. */
	STS_RB_POS_CMOS = 1,
	STS_RB_POS__COUNT = 2,
} sts_rb_pos_t;

/** What rb_serial_set_mode() must do with a requested position. */
typedef enum {
	/** Drive the relay. The only verdict that touches the pin. */
	STS_RB_MOVE_APPLY = 0,
	/** Not a position at all; -EINVAL. Owes nothing — an out-of-range mode
	 *  is a caller bug, and letting one arm the deferred restore would let a
	 *  typo move the relay minutes later. */
	STS_RB_MOVE_INVALID,
	/** A tunnel holds the port; -EBUSY, and nothing is owed. The CMOS half
	 *  of the asymmetry above. */
	STS_RB_MOVE_REFUSE,
	/** A tunnel holds the port; -EBUSY, and the fail-safe restore is now
	 *  owed by the close that gives the port back. The RS-232 half. */
	STS_RB_MOVE_OWE,
} sts_rb_move_act_t;

/** What rb_serial_tunnel_close() must do. */
typedef enum {
	/** No tunnel is open: the port is already free and there is nothing to
	 *  give back. Idempotent, by contract. */
	STS_RB_CLOSE_NOTHING = 0,
	/** Give the port back. Nothing is owed, so the relay stays where the
	 *  standing lease put it. */
	STS_RB_CLOSE_PLAIN,
	/** Give the port back, clear the latch, and only THEN command the relay
	 *  to STS_RB_POS_RS232. The latch is cleared whether or not that move
	 *  succeeds: a relay that could not be driven is a hardware fault worth
	 *  one loud line, not a standing obligation to fire at an arbitrary later
	 *  close in some unrelated session. */
	STS_RB_CLOSE_RESTORE,
} sts_rb_close_act_t;

/**
 * The relay as this side of the seam models it: where the pin is believed to
 * stand, and whether a fail-safe restore is owed.
 *
 * Held by rb_serial.c inside its own state, which is why `pos` is only ever
 * advanced after a *successful* GPIO write — a failed write must leave the model
 * and the pin disagreeing in the direction that makes the next attempt retry.
 */
typedef struct {
	/** sts_rb_pos_t. The position the pin is believed to hold. */
	uint8_t pos;
	/**
	 * A move to STS_RB_POS_RS232 was asked for and refused because the tunnel
	 * held the port. Set only on the refusal path, so it can never be armed
	 * while no tunnel is open.
	 */
	bool restore_rs232_on_close;
} sts_rb_relay_t;

/**
 * The state rb_serial_init() must leave behind: the fail-safe position, and no
 * outstanding obligation.
 *
 * Both halves matter. RS-232 is the documented reset path through U46 and a
 * CMOS-direct variant is an explicit commissioning decision, never something a
 * reboot lands in. And a latch that survived re-initialisation would be an
 * obligation inherited from a session that no longer exists, which is the one
 * thing the deferral must never become.
 *
 * @param st  Never NULL; fully written.
 */
static inline void sts_rb_relay_reset(sts_rb_relay_t *st)
{
	st->pos = (uint8_t)STS_RB_POS_RS232;
	st->restore_rs232_on_close = false;
}

/**
 * The level RB_RS232_CMOS_SW (PE4) must be driven to for @p pos.
 *
 * The pin is active-high and LOW is the level shifter, so the fail-safe position
 * is the *inactive* one — which is what makes GPIO_OUTPUT_INACTIVE at init the
 * right reset drive, and what an inverted mapping here would silently undo in
 * both places at once.
 *
 * @param pos  sts_rb_pos_t. Anything that is not STS_RB_POS_CMOS answers false,
 *             so a corrupt value fails towards the fail-safe path.
 */
static inline bool sts_rb_relay_pin_active(uint8_t pos)
{
	return pos == (uint8_t)STS_RB_POS_CMOS;
}

/**
 * Decide a requested relay move.
 *
 * Validity is checked before the tunnel, so a bad mode is -EINVAL whether or not
 * a tunnel happens to be open and can never arm the deferral.
 *
 * There is deliberately no "already in this position" short-circuit. It would
 * save one contact settle on a redundant command — 20 ms on an action a human
 * initiates — and would cost the one case where the write has to happen anyway:
 * sts_rb_relay_t::pos only advances after a successful GPIO write, so a failed
 * write leaves the model and the pin disagreeing, and a short-circuit keyed on
 * the model would then refuse to retry the write that is the fix.
 *
 * @param pos          Requested position, as passed to rb_serial_set_mode().
 * @param tunnel_open  rb_serial_tunnel_active(): a raw tunnel holds UART7.
 */
static inline sts_rb_move_act_t sts_rb_move_decide(uint8_t pos, bool tunnel_open)
{
	if (pos >= (uint8_t)STS_RB_POS__COUNT) {
		return STS_RB_MOVE_INVALID;
	}
	if (!tunnel_open) {
		return STS_RB_MOVE_APPLY;
	}
	return (pos == (uint8_t)STS_RB_POS_RS232) ? STS_RB_MOVE_OWE
						  : STS_RB_MOVE_REFUSE;
}

/**
 * Decide what a tunnel close owes.
 *
 * @p owed is consulted only when a tunnel is actually open. The pair
 * (no tunnel, owed) is unreachable — the latch is armed only by a refusal, and
 * only an open tunnel refuses — and answering anything but STS_RB_CLOSE_NOTHING
 * for it would make an idempotent no-op call throw the relay.
 *
 * @param tunnel_open  A raw tunnel holds UART7 right now.
 * @param owed         sts_rb_relay_t::restore_rs232_on_close.
 */
static inline sts_rb_close_act_t sts_rb_close_decide(bool tunnel_open, bool owed)
{
	if (!tunnel_open) {
		return STS_RB_CLOSE_NOTHING;
	}
	return owed ? STS_RB_CLOSE_RESTORE : STS_RB_CLOSE_PLAIN;
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_PLATFORM_STS_RB_SERIAL_POLICY_H_ */
