/*
 * STS1000 "Meridian" — front-panel input echo, as pure logic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Implementation detail of the ui area's Maintenance Protocol panel mirror
 * (sts_ui.c; `buttons_down`, key 17, in mp_mirror.h). Split out and made
 * Zephyr-free so tests/host can compile it — the same arrangement as
 * sts_cfg_applier.h and sts_confirm_gate.h — because the defect it exists to
 * prevent is invisible on the box and actively misleading off it.
 *
 * Why the reconstruction is lossy
 * -------------------------------
 * The mirror publishes a *level* bitmap ("which buttons are physically down"),
 * but this area only ever sees an *event* stream: the 1 kHz scan posts one
 * press and one release per button into a 16-deep, drop-on-full queue
 * (sts_ui_post_input()). Drop-on-full is correct — the scan must never be
 * parked behind the lowest-priority thread in the system — but it makes the
 * level reconstruction lossy in the one direction that matters. Lose a
 * *release* and the bit latches until that button is pressed and released
 * again: a technician reading `mirror.get` sees a permanently-held key and
 * concludes the panel has a stuck button. The same lost edge strands the lamp
 * test on.
 *
 * There is no second opinion available inside this area that would settle it.
 * core/fault holds the debounced level bitmap (fault_state()) behind the
 * platform's fault mutex; sts_pwrseq_snap_t::scan_state now republishes that
 * word lock-free, so a reader does exist — but it is a **4 Hz sample of a 1 kHz
 * signal**, taken on the sequencer's tick. A 60 ms press falls between two
 * samples entirely, so substituting it would trade this reconstruction's
 * self-correcting false negative for a sampling one that no drop counter can
 * even detect. Reconciling the two — edges from the event stream, the snapshot
 * consulted only to clear bits after a drop — is a real option and a separate
 * change; it is not what this header does today.
 *
 * So the reconstruction is made *fail-safe* rather than pretending to be exact:
 * the producer counts every event it had to drop, the consumer folds that count
 * in before it publishes, and any advance clears the echo to "nothing held".
 *
 * A genuinely-held button then under-reports until its next press — a false
 * negative that self-corrects on the very next edge — instead of over-reporting
 * a key nobody is touching, which is a false positive that looks exactly like a
 * hardware fault and never clears on its own.
 *
 * Only *level* state is recoverable this way. The encoder position is a
 * free-running accumulator, so a dropped detent is a permanent offset with
 * nothing to reconcile against; the scan does not normally post encoder events
 * (TIM1 is read on the render thread, where nothing can be dropped), which is
 * what keeps that exposure theoretical.
 */

#ifndef STS1000_ZEPHYR_UI_STS_UI_ECHO_H_
#define STS1000_ZEPHYR_UI_STS_UI_ECHO_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Signals the echo can represent: one 32-bit word, indexed by `fault_sig_t`. */
#define STS_UI_ECHO_SIGNALS 32U

/**
 * Reconstructed panel input echo.
 *
 * Zero-initialised storage is a valid empty echo. Single-threaded by contract:
 * every entry point below runs on the ui render thread.
 */
typedef struct {
	uint32_t down;       /**< level bitmap, bit n = fault_sig_t n */
	uint32_t drops_seen; /**< producer drop total already folded in */
	uint32_t resyncs;    /**< times a drop forced the echo clear */
	bool lamp;           /**< LAMP button held (level, same event stream) */
} sts_ui_echo_t;

/** Fold one button edge in. Signals past the word are ignored, not clamped. */
static inline void sts_ui_echo_button(sts_ui_echo_t *e, uint8_t sig,
				      bool pressed)
{
	if ((e == NULL) || ((uint32_t)sig >= STS_UI_ECHO_SIGNALS)) {
		return;
	}
	if (pressed) {
		e->down |= (uint32_t)1U << sig;
	} else {
		e->down &= ~((uint32_t)1U << sig);
	}
}

/** True when @p sig is currently reconstructed as held. */
static inline bool sts_ui_echo_is_down(const sts_ui_echo_t *e, uint8_t sig)
{
	if ((e == NULL) || ((uint32_t)sig >= STS_UI_ECHO_SIGNALS)) {
		return false;
	}
	return (e->down & ((uint32_t)1U << sig)) != 0U;
}

/**
 * Reconcile the echo against the producer's cumulative drop count.
 *
 * Call once per published frame, before reading @ref sts_ui_echo_t::down.
 *
 * @param e      Echo.
 * @param drops  sts_ui_post_input()'s running total of events it could not
 *               enqueue. Only *change* is significant, so the 2^32 wrap needs
 *               no handling.
 *
 * @retval true   At least one event was lost since the last call. @p e has been
 *                cleared to "nothing held", and the caller must actively
 *                release anything the lost edge might have left asserted — the
 *                lamp test, whose `lamp` flag this call also clears.
 * @retval false  Nothing was lost; @p e is untouched.
 */
static inline bool sts_ui_echo_sync(sts_ui_echo_t *e, uint32_t drops)
{
	if ((e == NULL) || (drops == e->drops_seen)) {
		return false;
	}
	e->drops_seen = drops;
	e->down = 0U;
	e->lamp = false;
	e->resyncs++;
	return true;
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_UI_STS_UI_ECHO_H_ */
