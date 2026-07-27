/*
 * STS1000 "Meridian" — what a factory reset must do, on every plane, as pure
 * logic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/ui/, and deliberately free of every Zephyr, cfg and
 * logging dependency so tests/host can compile it — the same arrangement as
 * console/sts_confirm_gate.h and net/sts_ntp_keys.h, for the same reason: what
 * it decides is invisible at runtime when it is wrong.
 *
 * ---------------------------------------------------------------------------
 * Why this exists at all
 * ---------------------------------------------------------------------------
 *
 * A factory reset is offered by three planes — the web UI, the MCP/console
 * command and the front panel's Menu -> Factory reset dialog — and it has to
 * mean the same thing on all three, because an operator decommissioning a unit
 * picks whichever one is in front of them. It did not: the panel plane called
 * sts_cfg_factory_reset() and nothing else, so a reset driven from the front
 * panel cleared the config tree (every CFG_F_SECRET blob with it) and then left
 * the persisted TLS server key, its certificate and the ACME account key sitting
 * on /lfs, plus every outstanding web session still valid. The unit reported a
 * clean factory state and kept its old identity.
 *
 * The defect was not in any of the three implementations taken alone — each one
 * is a short, obvious function. It was in the *set* of them, which nothing
 * described and nothing checked. So the shape of the operation lives here, the
 * planes are written against it, and tests/host/test_factory_policy.c asserts
 * both the arithmetic and — by scanning the sources — that all three planes
 * still perform both halves.
 *
 * ---------------------------------------------------------------------------
 * The shape of the operation
 * ---------------------------------------------------------------------------
 *
 *   1. sts_cfg_factory_reset()  clears the live tree, erases the store and runs
 *                               every registered group's appliers, so no
 *                               subsystem is left serving pre-reset config and
 *                               every subsystem's RAM copy of a secret is
 *                               overwritten by the applier fan-out.
 *   2. sts_sec_factory_wipe()   erases what does not live in the config tree:
 *                               the web sessions and CSRF tokens, and the
 *                               persisted TLS identity on /lfs and in the
 *                               Zephyr credential store.
 *   3. reboot                   load-bearing, not a courtesy. It is what clears
 *                               RAM-only key material and mints the replacement
 *                               identity.
 *
 * Three rules bind them, and all three are testable:
 *
 *   FAILURES ACCUMULATE.  Step 2 runs even when step 1 failed, and the result is
 *   -EIO if *either* failed. Short-circuiting on the first failure is how a
 *   half-wiped unit gets reported as a clean one.
 *
 *   AN ABSENT WIPE IS NOT A FAILURE.  With CONFIG_STS1000_NET=n there is no TLS
 *   identity to erase and sts_sec_factory_wipe() does not exist; the weak symbol
 *   resolves to NULL. That is a config-only reset and it is honest, so it
 *   returns 0 — but it MUST be annunciated, because "config-only" and "complete"
 *   are different states of the unit and the operator is entitled to know which
 *   one they got.
 *
 *   THE REBOOT IS UNCONDITIONAL.  An operator who asked for a factory reset and
 *   got neither a wipe nor a reboot is the worst outcome available: the unit is
 *   in an undefined half-reset state and nothing forces it out. sts_factory_
 *   should_reboot() therefore takes the outcome as an argument and ignores it —
 *   the signature exists so that a future caller cannot reintroduce the
 *   conditional without deleting a documented invariant and a test.
 */

#ifndef STS1000_ZEPHYR_UI_STS_FACTORY_POLICY_H_
#define STS1000_ZEPHYR_UI_STS_FACTORY_POLICY_H_

#include <errno.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** What one plane's factory reset actually managed to do. */
typedef struct {
	/** sts_cfg_factory_reset() returned 0. */
	bool cfg_reset_ok;
	/**
	 * sts_sec_factory_wipe() is present in this image.
	 *
	 * False only for a build without the net area, where the weak symbol
	 * resolves to NULL. Not a failure — see the header comment.
	 */
	bool wipe_linked;
	/** sts_sec_factory_wipe() returned 0. Meaningless when !wipe_linked. */
	bool wipe_ok;
} sts_factory_steps_t;

/**
 * The result the plane reports to whoever asked for the reset.
 *
 * @retval 0     Everything this image could erase is gone.
 * @retval -EIO  At least one step failed; the unit must NOT be described as
 *               factory-clean.
 */
static inline int sts_factory_result(const sts_factory_steps_t *s)
{
	if (s == NULL) {
		return -EIO;
	}
	if (!s->cfg_reset_ok) {
		return -EIO;
	}
	if (s->wipe_linked && !s->wipe_ok) {
		return -EIO;
	}
	return 0;
}

/**
 * Whether the reset erased only the config tree because no key-zeroization
 * path is linked into this image.
 *
 * True is not an error; it is a fact the plane must log, because the TLS
 * identity such a unit keeps is not visible in the config tree the operator
 * just watched being cleared.
 */
static inline bool sts_factory_wipe_skipped(const sts_factory_steps_t *s)
{
	return (s != NULL) && !s->wipe_linked;
}

/**
 * Whether to reboot. Always true — including after a failure, and including
 * when the config reset itself failed.
 *
 * @p s is accepted and ignored on purpose: the invariant is "the reboot does
 * not depend on the outcome", and a function that cannot see the outcome could
 * not be shown to honour it.
 */
static inline bool sts_factory_should_reboot(const sts_factory_steps_t *s)
{
	(void)s;
	return true;
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_UI_STS_FACTORY_POLICY_H_ */
