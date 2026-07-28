/*
 * STS1000 "Meridian" — the stage-8 rubidium envelope and the pwrseq input
 * conditioning, as pure logic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/platform/, and deliberately free of every Zephyr
 * dependency so tests/host can compile it — the same arrangement as
 * net/sts_ppscorr.h, for the same reason.
 *
 * ---------------------------------------------------------------------------
 * Why the VCC_RB ceiling is the most consequential number in this tree
 * ---------------------------------------------------------------------------
 * The root CLAUDE.md states it directly: "Rb digipot in a buck FB node can
 * destroy the Rb". The MIC28516 rail is programmable from 4.51 V to 24.43 V,
 * the schema bounds `pwr.rb.vmax.mv` only by that buck range, and the FE-5680A
 * this board is built around is a 15 V-class part. So an operator — or a bad
 * config import — can raise the "FE ceiling" to 24 V, and pwrseq's
 * measured <= vmax gate then waves through a rail that destroys the FE. The
 * hardware ceiling below is the thing that stops it, and until now nothing
 * asserted that it exists, that it clamps rather than rejects, or that it
 * cannot be raised.
 *
 * The second half of the same decision is subtler and its failure is worse in a
 * different direction: a ceiling configured *below* the fixed operating
 * setpoint makes pwrseq_init() reject the whole configuration, and a sequencer
 * that never starts means no GPS, no display, no watchdog and no relay. A
 * config typo aimed at the rubidium would brick the grandmaster. The policy is
 * therefore "clamp down, refuse up, never reject the sequencer".
 *
 * ---------------------------------------------------------------------------
 * The rest: input conditioning for pwrseq_step()
 * ---------------------------------------------------------------------------
 * core/pwrseq is the stage machine and tests/host/test_pwrseq.c covers it. What
 * was untested is the layer that builds its inputs — and each item here has a
 * failure that the stage machine cannot detect because it is downstream of it:
 *
 *   pg_mask       an off-by-one turns one rail's power-good into another's, and
 *                 the sequencer then gates the wrong load.
 *   ina_age_ms    the field is a *timestamp*, not an age. Passing it through
 *                 unconverted made the rubidium rail windows judge a reading up
 *                 to a second older than the rail change they were watching.
 *   extref band   a window that is too wide accepts a rubidium that is not on
 *                 frequency as the system clock reference.
 *   rb_lock       the opto inverts, and which level means "locked" is a
 *                 per-unit commissioning bit for a surplus FE variant. Inverted,
 *                 the sequencer either never engages the Rb or engages an
 *                 unlocked one.
 *
 * The GPIOs, the cfg reads and the digipot SPI stay in pwrseq_exec.c.
 */

#ifndef STS1000_ZEPHYR_PLATFORM_STS_RBGUARD_H_
#define STS1000_ZEPHYR_PLATFORM_STS_RBGUARD_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "fault/fault.h"
#include "ina228/ina228.h"
#include "pwrseq/pwrseq.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------- the VCC_RB ceiling */

/**
 * Hard ceiling on the configurable VCC_RB limit, millivolts.
 *
 * Not a preference and not derived from cfg: it is what the FE-5680A this board
 * is built around can survive. cfg may lower it for a lower-voltage unit; it may
 * never raise it past what the hardware was designed for. Documented in
 * docs/sts1000_vcc_rb_supply.md and the root CLAUDE.md digipot gotcha.
 */
#define STS_RB_VMAX_MV_CEILING 15000U

/** What the configured VCC_RB ceiling resolved to, and why. */
typedef struct {
	/** The value to put in pwrseq_cfg_t::rb_vmax_mv. Always set. */
	uint32_t vmax_mv;
	/** The configured value exceeded the hardware ceiling and was clamped. */
	bool clamped;
	/**
	 * The (possibly clamped) value sits below the fixed operating setpoint,
	 * so it was refused and the built-in default kept. Refusing beats
	 * accepting: pwrseq_init() would reject the whole configuration, and a
	 * sequencer that never starts costs GPS, display, watchdog and relay.
	 */
	bool refused;
} sts_rb_vmax_t;

/**
 * Resolve the configured `pwr.rb.vmax.mv` against the hardware.
 *
 * @param cfg_mv        The configured value, as read from cfg (u64, unbounded
 *                      here on purpose — the point is that this function is the
 *                      bound).
 * @param default_mv    pwrseq_cfg_default()'s rb_vmax_mv, kept on a refusal.
 * @param operating_mv  pwrseq_rb_expected_mv() for the operating digipot code.
 *                      Non-positive means "unknown", in which case the
 *                      below-setpoint check is skipped rather than guessed.
 * @param out           Never NULL; always fully written.
 */
static inline void sts_rb_vmax_decide(uint64_t cfg_mv, uint32_t default_mv,
				      int32_t operating_mv, sts_rb_vmax_t *out)
{
	uint32_t vmax;

	memset(out, 0, sizeof(*out));

	if (cfg_mv > (uint64_t)STS_RB_VMAX_MV_CEILING) {
		vmax = STS_RB_VMAX_MV_CEILING;
		out->clamped = true;
	} else {
		vmax = (uint32_t)cfg_mv;
	}

	if ((operating_mv > 0) && (vmax < (uint32_t)operating_mv)) {
		out->vmax_mv = default_mv;
		out->refused = true;
		return;
	}

	out->vmax_mv = vmax;
}

/* ------------------------------------------------------------- PG decode --- */

/**
 * Build pwrseq_in_t::pg_mask from core/fault's debounced bitmap.
 *
 * core/fault normalises every scanned line so that "asserted" means the
 * condition worth reacting to; for a power-good line, high = good, so asserted
 * means NOT good. pwrseq wants the opposite convention — a set bit is a rail
 * that reads good — so this is an inversion as well as a re-index, and both
 * halves are silent when wrong.
 *
 * The eight rails are contiguous from FAULT_SIG_PG_3V3_GPS_LDO (PG0) through
 * FAULT_SIG_PG_POE (PG7), and PWRSEQ_PG(n) numbers them the same way.
 *
 * The expected-off mask (fault_expected_off_mask()) is deliberately NOT applied:
 * a rail firmware has gated off must still report itself as not-good to the
 * sequencer, which is the thing that gated it. Suppressing it here would let
 * pwrseq read its own commanded-off rails as healthy.
 *
 * @param asserted  fault_state(): bit n set = signal n is asserted.
 */
static inline uint8_t sts_rb_pg_mask(uint32_t asserted)
{
	uint8_t mask = 0;

	for (unsigned int n = 0; n < 8U; n++) {
		fault_sig_t sig = (fault_sig_t)((unsigned int)FAULT_SIG_PG_3V3_GPS_LDO + n);

		if ((asserted & FAULT_SIG_BIT(sig)) == 0U) {
			mask |= PWRSEQ_PG(n);
		}
	}

	return mask;
}

/**
 * Are both supercap-backed backup rails good?
 *
 * Same inversion as above: FAULT_SIG_BKP_*_PG asserted means the rail is NOT
 * good. Both are required — the STM backup holds the RTC and the GPS backup
 * holds the ephemeris, and the stage-8 gate wants a board that can ride out a
 * rubidium inrush without losing either.
 */
static inline bool sts_rb_supercaps_charged(uint32_t asserted)
{
	return (asserted & (FAULT_SIG_BIT(FAULT_SIG_BKP_STM_PG) |
			    FAULT_SIG_BIT(FAULT_SIG_BKP_GPS_PG))) == 0U;
}

/* ------------------------------------------------------------ INA inputs --- */

/**
 * Age of a cached INA228 reading, milliseconds.
 *
 * sts_ina_reading_t::age_ms is the monotonic *timestamp* the reading was taken
 * at, not an age. Handing the timestamp straight to pwrseq made the rubidium
 * rail windows judge whatever was in the cache — on a 1 Hz sweep against a
 * 250 ms tick, a reading up to a second older than the rail change it was
 * supposed to observe.
 *
 * An invalid reading has no age; UINT32_MAX is the "infinitely old" sentinel
 * every pwrseq freshness test already treats as a refusal.
 */
static inline uint32_t sts_rb_ina_age_ms(bool valid, uint32_t now_ms,
					 uint32_t stamp_ms)
{
	return valid ? (uint32_t)(now_ms - stamp_ms) : UINT32_MAX;
}

/**
 * PoE draw in milliwatts from the input monitor's bus voltage and current.
 *
 * INA228 0x40 reads the 54 V bus directly with no divider, so both terms are
 * true rail quantities. Non-positive readings yield 0 rather than a negative or
 * wrapped power: the monitor is signed and reads slightly negative at no load,
 * and a wrapped "4 gigawatt" draw would trip the budget gate and shed loads on
 * a healthy board.
 */
static inline uint32_t sts_rb_poe_mw(bool valid, int32_t bus_uv,
				     int32_t current_ua)
{
	int32_t mv = bus_uv / 1000;
	int32_t ma = current_ua / 1000;

	if (!valid || mv <= 0 || ma <= 0) {
		return 0U;
	}

	return (uint32_t)(((int64_t)mv * (int64_t)ma) / 1000);
}

/* --------------------------------------------------------- external ref --- */

/**
 * Acceptance band for the external 10 MHz reference, hertz.
 *
 * +-20 Hz, i.e. +-2 ppm. Wide enough for the EXTREF_MON gate's own +-1-count
 * quantisation over its measurement window, and far tighter than any fault
 * mode that leaves the rubidium producing a carrier at all: an unlocked
 * FE-5680A free-runs tens of hertz away, and a dead one produces no edges and
 * fails the validity flag instead.
 */
#define STS_RB_EXTREF_MIN_HZ 9999800U
#define STS_RB_EXTREF_MAX_HZ 10000200U

/** Is the measured external reference in band and trustworthy? */
static inline bool sts_rb_extref_in_band(uint32_t hz, bool valid)
{
	return valid && (hz >= STS_RB_EXTREF_MIN_HZ) && (hz <= STS_RB_EXTREF_MAX_HZ);
}

/* -------------------------------------------------------------- RB_LOCK --- */

/**
 * Decode the RB_LOCK pin.
 *
 * The opto (U48) inverts: FE lock line high -> LED on -> transistor on ->
 * RB_LOCK pulled low (rb_rs232_interface §6A.3). Which level means "locked"
 * cannot be assumed for a surplus FE variant, so it is a commissioning bit
 * rather than a hard-coded polarity.
 *
 * @param level       gpio_pin_get_dt(): 0, 1, or negative on error.
 * @param active_low  CONFIG_STS1000_RB_LOCK_ACTIVE_LOW.
 *
 * A read error is NOT locked. Reporting "locked" for a pin that could not be
 * read would let the reference state machine hand the system clock to a
 * rubidium nobody can see the status of.
 */
static inline bool sts_rb_lock_from_level(int level, bool active_low)
{
	if (level < 0) {
		return false;
	}

	return active_low ? (level == 0) : (level == 1);
}

/**
 * Whether the FE-5680A can answer on UART7 right now — BOTH pins, not one.
 *
 * ---------------------------------------------------------------------------
 * WHY THIS IS A FUNCTION AND NOT TWO gpio_pin_get_dt() CALLS IN rb_serial.c
 * ---------------------------------------------------------------------------
 *
 * RB_PWR_EN (pwrseq step 8.12, pwrseq.c:920) brings up the buck. RB_VCC_GATE
 * (step 8.14, pwrseq.c:986) is what connects VCC_RB to the FE. Between them sit
 * the soft-start delay, the rail-safety window, two digipot write/verify rows
 * with up to two retries each, and the operating-rail ramp. The buck is live
 * throughout and the rubidium is not connected.
 *
 * rb_serial_rail_up() tested RB_PWR_EN alone, so it answered "yes" during a
 * window in which nothing could reply — and rb_fwupd_probe() takes a rail-up
 * silence as licence to conclude RB_CAP_NONE, "absent, check the cable", and
 * latch it for the rest of the uptime (cap returns to UNKNOWN only in
 * rb_fwupd_init()). An unauthenticated G0 `fw.inventory` lands in that window
 * during ordinary bring-up.
 *
 * A previous fix closed the RB_PWR_EN-low half of this and its commit message
 * claimed all of it. It did not — this half is the longer one. An adversarial
 * reviewer found that by reading pwrseq's step table against the predicate;
 * no test could have, because the predicate lived in Zephyr glue and the core
 * test fixture models the ANSWER (a single `rail` bool) rather than the pins.
 * That is why the decision now lives here, where a host test can hold it.
 *
 * Gating on both pins also keeps the probe one-shot, which matters: while the
 * gate is shut the probe declines with no bus traffic at all, and once it is
 * open, silence is a real verdict worth latching. A probe that merely stopped
 * latching would leave one unauthenticated inventory able to hold the MP engine
 * lock for rb_fwupd's full 1 s reply timeout, repeatably.
 *
 * @param pwr_en_level    gpio_pin_get_dt(RB_PWR_EN): 0, 1, or negative.
 * @param vcc_gate_level  gpio_pin_get_dt(RB_VCC_GATE): 0, 1, or negative.
 *
 * A read error on either pin is NOT up, for the same reason a read error is not
 * locked above: concluding "the rubidium is absent" from a pin nobody could
 * read is a verdict this code has no business reaching.
 */
static inline bool sts_rb_serial_rail_up(int pwr_en_level, int vcc_gate_level)
{
	return (pwr_en_level == 1) && (vcc_gate_level == 1);
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_PLATFORM_STS_RBGUARD_H_ */
