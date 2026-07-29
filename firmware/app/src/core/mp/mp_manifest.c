/*
 * STS1000 "Meridian" — core/mp: capability manifest (FMT §4).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * The object inventory is derived from the as-built pin contract
 * (docs/sts1000_firmware_hardware_interface.md §1 pin table, §4 I2C map, §4.2
 * INA rail table) — see mp_manifest.h for the corrections that differ from the
 * FMT spec's illustrative examples.
 *
 * Objects deliberately absent, with reasons:
 *   - SPI4 chip-selects (SPI_NSS/PE11, DISP_CS/PE9, SPI_DPOT_CS/PD2) and the
 *     RMII/UART bus pins. These are driver-owned per-transfer state, not
 *     commandable board state; exposing them would let the tool desynchronise a
 *     bus mid-transaction with no way for firmware to notice.
 *   - RTC_TAMPER (PC13). Input to the RTC tamper block, not readable as GPIO.
 *   - Anything on an I/O expander: there is none (see the header).
 */

#include "mp/mp_manifest.h"

#include <errno.h>
#include <string.h>

#include "ina228/ina228.h"
#include "mp/mp_json.h"
#include "util/crc.h"

/* ------------------------------------------------------------------- names */

static const char *const guard_names[MP_GUARD_COUNT] = { "G0", "G1", "G2",
							 "G3" };

const char *mp_guard_name(uint8_t g)
{
	return (g < (uint8_t)MP_GUARD_COUNT) ? guard_names[g] : "G?";
}

static const char *const kind_names[MP_KIND_COUNT] = {
	"bool", "enum", "pct",  "mv",   "code", "pulse",
	"scalar", "real", "rail", "bits", "text",
};

const char *mp_kind_name(uint8_t k)
{
	return (k < (uint8_t)MP_KIND_COUNT) ? kind_names[k] : "unknown";
}

static const char *const group_names[MP_GRP_COUNT] = {
	"power", "reference", "gnss", "panel", "system", "sensor",
};

const char *mp_group_name(uint8_t g)
{
	return (g < (uint8_t)MP_GRP_COUNT) ? group_names[g] : "unknown";
}

/* -------------------------------------------------------------- interlocks */

typedef struct {
	uint32_t bit;
	const char *name;
	const char *reason;
} ilk_info_t;

static const ilk_info_t ilks[MP_ILK_COUNT] = {
	{ MP_ILK_RB_VMAX, "rb.vmax",
	  "VCC_RB clamped to cfg pwr.rb.vmax.mv" },
	{ MP_ILK_RB_OV, "rb.ov", "rubidium 26 V OV latch is set" },
	{ MP_ILK_RB_VERIFY, "rb.verify",
	  "VCC_RB read-back on INA228 0x47 must match, else auto-revert" },
	{ MP_ILK_RB_WARM, "rb.warm",
	  "OCXO must be warm and the supercaps charged (PoE budget)" },
	{ MP_ILK_FAN_FLOOR, "fan.floor",
	  "fan duty may not go below the thermal loop's floor" },
	{ MP_ILK_RELAY_OK, "relay.ok",
	  "K2 may not be force-energized while a disqualifying fault is active" },
	{ MP_ILK_DISP_OFF, "disp.offtime",
	  "display rail minimum off-time not yet elapsed" },
	{ MP_ILK_WDT_LIVE, "wdt.liveness",
	  "external watchdog may be armed only with a healthy liveness gate" },
	{ MP_ILK_TUNNEL, "tunnel",
	  "suspends firmware's use of the port; the reference becomes suspect" },
	{ MP_ILK_MUX_GUARD, "mux.guard",
	  "clock handoff needs EXTREF_MON in band and RB_LOCK asserted" },
	{ MP_ILK_DAC_PARK, "dac.park",
	  "the discipline loop must be parked before the DAC may be driven" },
	{ MP_ILK_WDT_OFF, "wdt.off",
	  "WDI may be pulsed only with the external watchdog disabled" },
	{ MP_ILK_DAC_SOLE, "dac.sole",
	  "the other view of DAC1_OUT1 is already leased" },
	{ MP_ILK_DAC_IDLE, "dac.idle",
	  "the park may not be released while a Vc override is held" },
};

static const ilk_info_t *ilk_info(uint32_t bit)
{
	size_t i;

	for (i = 0U; i < MP_ILK_COUNT; i++) {
		if (ilks[i].bit == bit) {
			return &ilks[i];
		}
	}
	return NULL;
}

const char *mp_ilk_name(uint32_t bit)
{
	const ilk_info_t *info = ilk_info(bit);

	return (info != NULL) ? info->name : NULL;
}

const char *mp_ilk_reason(uint32_t bit)
{
	const ilk_info_t *info = ilk_info(bit);

	return (info != NULL) ? info->reason : "unknown interlock";
}

/* ================================================================ objects */

/* Units, pooled so the table reads as a table. */
#define U_MV "mV"
#define U_PCT "%"
#define U_MS "ms"
#define U_MC "mC"
#define U_NS "ns"
#define U_HZ "Hz"
#define U_MW "mW"
#define U_RPM "rpm"
#define U_S "s"
#define U_PPB "ppb"
#define U_MPCT "m%RH"

/*
 * Flag shorthands.
 *
 * The `_D` forms add MP_OF_DEFERRED — published, but nothing is wired behind
 * it, so the device refuses. See mp_manifest.h for what the bit is exactly the
 * complement of; tests/host/test_mp_deferred.c derives the set from
 * mp_glue.c's dispatch and fails this table if the two ever disagree, which is
 * the only thing that stops the bit rotting the first time someone wires a
 * setter and forgets the row.
 *
 * Spelled as names rather than `F_RW | MP_OF_DEFERRED` on the thirty-five rows
 * that still carry it, so the table stays a table and the deferred rows read in
 * the same column as everything else.
 *
 * F_LEASE is the tunnel flag set: readable and OVERRIDABLE, deliberately NOT
 * writable. A tunnel is a leased thing — mp_tunnel.c: "it is opened by the
 * override and closed by releasing it, by the dead-man, or by leaving MP mode"
 * — and `obj.set` creates no lease, so a set that reached
 * sts_mp_tunnel_set_gnss(true) would suspend the GNSS receiver with nothing in
 * the system able to resume it: mp_ovr_revert_all() has no lease to revert, and
 * MP_ILK_TUNNEL is a consequence rather than a refusal (mp_override.c). The
 * grandmaster would lose GNSS until a reboot or a deliberate `obj.set … false`.
 * Dropping MP_OF_WRITE is what makes the lease mandatory and the dead-man
 * effective; test_mp_manifest.c asserts no MP_ILK_TUNNEL object is writable.
 */
#define F_RO MP_OF_READ
#define F_RO_D (F_RO | MP_OF_DEFERRED)
#define F_RW (MP_OF_READ | MP_OF_WRITE | MP_OF_OVERRIDE)
#define F_RW_D (F_RW | MP_OF_DEFERRED)
#define F_RWL (F_RW | MP_OF_ACTIVE_LOW)
#define F_RWL_D (F_RWL | MP_OF_DEFERRED)
#define F_PULSE (MP_OF_READ | MP_OF_PULSE)
#define F_PULSE_D (F_PULSE | MP_OF_DEFERRED)
#define F_CFG (MP_OF_READ | MP_OF_WRITE | MP_OF_CFG)
#define F_LEASE (MP_OF_READ | MP_OF_OVERRIDE)
#define F_LEASE_D (F_LEASE | MP_OF_DEFERRED)

/*
 * VCC_RB envelope. The supply's own range is 4.51-24.45 V over
 * VCC_RB = 24.45 - 6.645*VCTRL (hardware_design_reference); the manifest
 * publishes the full electrical range and the `rb.vmax` interlock clamps every
 * request to cfg pwr.rb.vmax.mv (default 15000 mV) at set time. Publishing the
 * electrical range rather than the configured one is deliberate: the tool has to
 * be able to show an operator that the ceiling is a *setting*, and the ceiling
 * itself is an object (`pwr.rb.vmax_mv`).
 */
#define RB_MV_MIN 4510
#define RB_MV_MAX 24450

/* MCP41U83 is an 8-bit (257-tap) part; code 0 = terminal B = safe-low. */
#define RB_POT_MAX 255

const mp_obj_t mp_objs[] = {
	/* ---------------------------------------------------- §6.1 power --- */
	/*
	 * THE MAILBOX RAILS. Every one of these is driven by the housekeeping
	 * thread's 4 Hz sequencer pass through zephyr/platform/sts_pwrseq_req.h,
	 * so every one of them is LEASE-ONLY — see the block over
	 * `pwr.panel.led.en` below, which states the two reasons in full.
	 */
	{ .id = "pwr.gps.en", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G2,
	  .group = MP_GRP_POWER, .flags = F_LEASE, .net = "GPS_PWR_EN",
	  .desc = "U22 LT3045 3V3_GPS rail enable (PC8)" },
	{ .id = "pwr.ant.bias.en", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G1,
	  .group = MP_GRP_POWER, .flags = F_LEASE, .net = "ANT_BIAS_EN",
	  .desc = "U27 RT9742 antenna bias-T enable (PC9)" },
	{ .id = "pwr.disp.en", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G1,
	  .group = MP_GRP_POWER, .flags = F_LEASE, .ilk = MP_ILK_DISP_OFF,
	  .net = "DISP_EN",
	  .desc = "U33 RT9742 5V_DISP + PCA9306 enable (PC11)" },
	/*
	 * The first rail wired through the parameterised sequencer mailbox
	 * (zephyr/platform/sts_pwrseq_req.h), and therefore LEASE-ONLY: no
	 * MP_OF_WRITE, for the reason test_mp_manifest.c states over the tunnel
	 * objects and for one more that is specific to a rail.
	 *
	 * The shared reason: `obj.set` creates no lease, so nothing in the
	 * system can put the rail back — not the dead-man, not a link drop, not
	 * `session.close`. FMT §5.1.2 makes an override a lease precisely so
	 * that nothing stays commanded after the tool walks away, and a power
	 * rail is the last object that should be exempt from it.
	 *
	 * The specific reason: the write is asynchronous. It is posted to the
	 * housekeeping thread, which owns every rail pin, and answers
	 * MP_APPLY_PENDING. `obj.override`'s reply can say so — that is what
	 * `verify_pending` means and mp_ovr_tick() drops the lease if the write
	 * never lands — but `obj.set`'s reply has no such field, so a `set`
	 * could only report an effect that had not happened yet. Refusing the
	 * method is honest; answering it would not be.
	 */
	{ .id = "pwr.panel.led.en", .kind = MP_KIND_BOOL,
	  .guard = MP_GUARD_G1, .group = MP_GRP_POWER, .flags = F_LEASE,
	  .net = "PANEL_LED_EN",
	  .desc = "U55 RT9742 panel-LED 5 V rail enable (PC0)" },
	/*
	 * `pwr.rb.en` STAYS DEFERRED, and it is the one rail of this group that
	 * does. ARCHITECTURE.md §10 invariant 3 fixes the order the rubidium may
	 * be brought up in — safe digipot code (verify readback) -> RB_PWR_EN ->
	 * INA 0x47 window -> RB_VCC_GATE — and a lease on PB7 cannot honour it
	 * in either direction: raising it energises the FE from a rail nothing
	 * re-verified while RB_VCC_GATE is already asserted, and lowering it
	 * drops the supply under a connected load, inverting the order
	 * rb_shutdown() and sts_pwrseq_rb_quiesce_from_isr() both document. A
	 * release has nowhere honest to go either — the only path back is the
	 * full guarded stage-8 re-entry, which is an operator action that clears
	 * the rubidium alarms and the deferral, not something a dead-man may do.
	 * The full argument is in zephyr/platform/pwrseq_exec.c, over the
	 * mailbox. `pwr.rb.gate` is the half of the pair that respects the
	 * order, so it is the half that is wired.
	 */
	{ .id = "pwr.rb.en", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G2,
	  .group = MP_GRP_POWER, .flags = F_RW_D,
	  .ilk = MP_ILK_RB_WARM | MP_ILK_RB_OV, .net = "RB_PWR_EN",
	  .desc = "U40 MIC28516 rubidium rail enable (PB7)" },
	{ .id = "pwr.rb.gate", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G2,
	  .group = MP_GRP_POWER, .flags = F_LEASE,
	  .ilk = MP_ILK_RB_OV | MP_ILK_RB_VERIFY, .net = "RB_VCC_GATE",
	  .desc = "Q25 gate: connects VCC_RB_G to the FE-5680A (PB1)" },
	{ .id = "pwr.rb.vset_mv", .kind = MP_KIND_MV, .guard = MP_GUARD_G2,
	  .group = MP_GRP_POWER, .flags = F_LEASE, .min = RB_MV_MIN,
	  .max = RB_MV_MAX, .step = 78, .unit = U_MV,
	  .ilk = MP_ILK_RB_VMAX | MP_ILK_RB_OV | MP_ILK_RB_VERIFY,
	  .net = "VCC_RB",
	  .desc = "U43 MCP41U83 digipot setpoint; 24.45 - 6.645*VCTRL" },
	{ .id = "pwr.rb.pot.code", .kind = MP_KIND_CODE, .guard = MP_GUARD_G3,
	  .group = MP_GRP_POWER, .flags = F_RW_D, .min = 0, .max = RB_POT_MAX,
	  .step = 1, .ilk = MP_ILK_RB_VMAX | MP_ILK_RB_OV,
	  .net = "SPI_DPOT_CS",
	  .desc = "U43 raw wiper code; 0 = terminal B = safe-low (PD2 CS)" },
	{ .id = "pwr.rb.vmax_mv", .kind = MP_KIND_MV, .guard = MP_GUARD_G2,
	  .group = MP_GRP_POWER, .flags = F_CFG, .min = RB_MV_MIN,
	  .max = RB_MV_MAX, .step = 1, .unit = U_MV, .cfg_key = 0x0703U,
	  .desc = "configured VCC_RB ceiling; bounds every rb.vset request" },
	{ .id = "pwr.rb.ov.reset", .kind = MP_KIND_PULSE, .guard = MP_GUARD_G2,
	  .group = MP_GRP_POWER, .flags = F_PULSE, .min = 1, .max = 100,
	  .step = 1, .unit = U_MS, .net = "RB_OV_RESET",
	  .desc = "clears the autonomous 26 V OV latch (PD3)" },
	{ .id = "pwr.poe.budget_mw", .kind = MP_KIND_SCALAR,
	  .guard = MP_GUARD_G2, .group = MP_GRP_POWER, .flags = F_CFG,
	  .min = 0, .max = 90000, .step = 1, .unit = U_MW,
	  .cfg_key = 0x0704U, .desc = "granted PoE budget (cfg pwr.poe.mw)" },
	{ .id = "pwr.poe.kill", .kind = MP_KIND_PULSE, .guard = MP_GUARD_G3,
	  .group = MP_GRP_POWER, .flags = F_PULSE, .min = 1, .max = 1000,
	  .step = 1, .unit = U_MS, .net = "POE_KILL",
	  .desc = "Q3 sustain latch: board cold-cycle, PSE-driven recovery (PE15)" },

	/* ------------------------------------------------ §6.2 reference --- */
	/*
	 * THE CLOCK MUX, AND WHY IT IS LEASED RATHER THAN WRITABLE.
	 *
	 * PB6 selects which 10 MHz reaches PH0, which is the HSE bypass input
	 * SYSCLK is derived from. Moving it under a live HSE feed skips the HSI
	 * bridge and glitches the system clock (spec §3.5), so the actuation is
	 * refsel's bracketed handoff — PARK_DISCIPLINE, mux flip, UNPARK — and
	 * never a pin write.
	 *
	 * F_LEASE for a reason specific to this object's value space. The
	 * standing request refsel consults is THREE-valued (AUTO / force-OCXO /
	 * force-EXTREF) while the manifest object is {ocxo, rb}, so there is no
	 * value of this object that means AUTO. `obj.set` creates no lease, so a
	 * bare write would pin the reference to a level with nothing able to
	 * restore the automatic behaviour — a box repaired months ago still
	 * running on its OCXO because nobody remembered, which is the failure
	 * sts_app.h refuses to give a cfg key for. A lease can be released, and
	 * mp_glue.c saves the pre-lease request verbatim so the release puts
	 * AUTO back when AUTO is what was there.
	 */
	{ .id = "ref.mux.sel", .kind = MP_KIND_ENUM, .guard = MP_GUARD_G3,
	  .group = MP_GRP_REF, .flags = F_LEASE, .min = 0, .max = 1, .step = 1,
	  .enums = "ocxo,rb", .ilk = MP_ILK_MUX_GUARD, .net = "MUX_SEL",
	  .desc = "U52 74LVC1G157 clock mux -> PH0 HSE bypass (PB6)" },
	/*
	 * F_LEASE, not F_RW, for the reason `pwr.panel.led.en` gives above: the
	 * apply is posted to the sequencer mailbox and answers
	 * MP_APPLY_PENDING, which `obj.set`'s reply has no field to admit.
	 *
	 * The RELEASE direction is the unusual part — it re-terminates. R176's
	 * 100k pull-up boots PC10 asserted and pwrseq drives it active at start,
	 * so terminated is firmware's automatic level and un-terminating is the
	 * direction a technician has to ask for.
	 */
	{ .id = "ref.term.en", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G1,
	  .group = MP_GRP_REF, .flags = F_LEASE, .net = "REF_TERM_EN",
	  .desc = "external-reference SMA 50 ohm termination (PC10)" },
	/*
	 * The maintenance park, and the object that makes the two below
	 * reachable at all.
	 *
	 * MP_ILK_DAC_PARK gates them on `disc_parked`, which prov_ilk() takes
	 * from QUALITY_FLAG_PARKED — a real measurement. Until this row existed
	 * the only producers of that state were the PFI power-fail ISR and
	 * refsel's own handoff bracket, so the interlock was satisfiable only
	 * during a power failure. EXPLICIT rather than implied by a Vc write,
	 * because interlocks are evaluated BEFORE apply: a park implied by the
	 * write could not satisfy the interlock gating that write.
	 *
	 * F_LEASE, not writable, for the reason every asynchronous actuator on
	 * this board is: the park lands on the discipline thread's next pass and
	 * `obj.set` has no field in which to admit a write that has not happened
	 * yet. A lease also gives it a dead-man — a console that walks away
	 * leaves the loop parked for at most the lease, not until the next boot.
	 *
	 * Its READ-BACK is the loop's actual state and not this object's request,
	 * the same choice `ref.mux.sel` makes: a PFI park reads back parked with
	 * no lease held, which is the truth a technician needs.
	 */
	{ .id = "ref.disc.park", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G3,
	  .group = MP_GRP_REF, .flags = F_LEASE, .ilk = MP_ILK_DAC_IDLE,
	  .net = "OCXO_VC",
	  .desc = "park the discipline loop; freezes Vc and frees the DAC" },
	{ .id = "ref.ocxo.vc_mv", .kind = MP_KIND_MV, .guard = MP_GUARD_G3,
	  .group = MP_GRP_REF, .flags = F_LEASE, .min = 0, .max = 3300,
	  .step = 1, .unit = U_MV,
	  .ilk = MP_ILK_DAC_PARK | MP_ILK_DAC_SOLE, .net = "OCXO_VC",
	  .desc = "DAC1_OUT1 -> U36 OPA320 -> OH300 Vc, centre 1650 mV (PA4)" },
	{ .id = "ref.ocxo.dac_code", .kind = MP_KIND_CODE,
	  .guard = MP_GUARD_G3, .group = MP_GRP_REF, .flags = F_LEASE, .min = 0,
	  .max = 4095, .step = 1,
	  .ilk = MP_ILK_DAC_PARK | MP_ILK_DAC_SOLE, .net = "OCXO_VC",
	  .desc = "raw 12-bit DAC1_OUT1 code (PA4)" },
	{ .id = "ref.relay.hold", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G2,
	  .group = MP_GRP_REF, .flags = F_RW_D, .ilk = MP_ILK_RELAY_OK,
	  .net = "HOLDOVER_ALARM_RELAY",
	  .desc = "K2 holdover/alarm relay; high = energized = healthy (PA6)" },
	/*
	 * LEASE-ONLY, for the shared reason the rail rows above give: `obj.set`
	 * creates no lease, so nothing in the system can put K1 back — not the
	 * dead-man, not a link drop, not `session.close`, not `mp_mode_exit`.
	 * Nothing re-asserts PE4 after rb_serial_init(), so a `set` to CMOS
	 * stranded the RS-232 fail-safe until somebody typed the opposite `set`.
	 *
	 * The deferred fail-safe restore in platform/rb_serial.c does NOT cover
	 * this: that latch only arms when a move to RS-232 is REFUSED by an open
	 * tunnel, and a `set` to CMOS with no tunnel open simply succeeds.
	 *
	 * Not an asynchronous apply — sts_rb_serial_set_mode() blocks for the
	 * 20 ms relay settle and answers 0 or -EBUSY — so this row is lease-only
	 * for the un-doability reason alone. test_mp_manifest.c's
	 * test_a_lease_only_actuator_is_never_writable is the guard, and it is
	 * scoped to that rule rather than to MP_ILK_TUNNEL for exactly this
	 * reason: this row carries no interlock and sat outside the old one.
	 */
	{ .id = "ref.rb.serial", .kind = MP_KIND_ENUM, .guard = MP_GUARD_G2,
	  .group = MP_GRP_REF, .flags = F_LEASE, .min = 0, .max = 1, .step = 1,
	  .enums = "rs232,cmos", .net = "RB_RS232_CMOS_SW",
	  .desc = "K1 relay: FE-5680A serial level select, RS-232 fail-safe (PE4)" },
	{ .id = "ref.rb.tunnel", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G2,
	  .group = MP_GRP_REF, .flags = F_LEASE, .ilk = MP_ILK_TUNNEL,
	  .net = "RB_TX/RB_RX",
	  .desc = "UART7 passthrough on channel 0x08 (PE7/PB4)" },

	/* ----------------------------------------------------- §6.3 gnss --- */
	{ .id = "gnss.reset", .kind = MP_KIND_PULSE, .guard = MP_GUARD_G2,
	  .group = MP_GRP_GNSS, .flags = F_PULSE_D | MP_OF_ACTIVE_LOW, .min = 1,
	  .max = 1000, .step = 1, .unit = U_MS, .net = "GPS_RST_N",
	  .desc = "ZED-F9T reset, active low (PD11)" },
	{ .id = "gnss.safeboot", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G3,
	  .group = MP_GRP_GNSS, .flags = F_RWL_D, .net = "GPS_SAFEBOOT_N",
	  .desc = "ZED-F9T safeboot, active low; with reset enters recovery (PD15)" },
	{ .id = "gnss.dsel", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G2,
	  .group = MP_GRP_GNSS, .flags = F_RW_D, .net = "GPS_DSEL",
	  .desc = "ZED-F9T interface select (PD7)" },
	{ .id = "gnss.extint", .kind = MP_KIND_PULSE, .guard = MP_GUARD_G1,
	  .group = MP_GRP_GNSS, .flags = F_PULSE, .min = 1, .max = 1000,
	  .step = 1, .unit = U_MS, .net = "GPS_EXTINT",
	  .desc = "ZED-F9T time-mark / aiding trigger (PD6)" },
	{ .id = "gnss.tunnel", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G2,
	  .group = MP_GRP_GNSS, .flags = F_LEASE, .ilk = MP_ILK_TUNNEL,
	  .net = "GPS_TX/GPS_RX",
	  .desc = "USART3 passthrough on channel 0x07 (PD8/PD9)" },

	/* ---------------------------------------------------- §6.4 panel --- */
	/*
	 * The dimmer, migrated onto the sequencer mailbox and therefore no
	 * longer writable. It DID work through `obj.set` — and that is the
	 * point: it worked by writing PE0 from the console thread, which is the
	 * single-writer violation `pwr.panel.led.en` was routed through the
	 * mailbox to close, on the very same setter. Losing MP_OF_WRITE is what
	 * an asynchronous apply costs; the host uses `obj.override` and reads
	 * `verify_pending`, and the write is now also refused when it would
	 * re-light a panel the shed ladder has dropped.
	 */
	{ .id = "ui.panel.duty", .kind = MP_KIND_PCT, .guard = MP_GUARD_G1,
	  .group = MP_GRP_PANEL, .flags = F_LEASE, .min = 0, .max = 100,
	  .step = 1, .unit = U_PCT, .net = "PANEL_LED_PWM",
	  .desc = "front-panel LED string dimmer, LPTIM2_CH2 (PE0)" },
	/*
	 * The four panel controls below are lease-only for the reason
	 * `pwr.panel.led.en` gives: each is posted to the sequencer mailbox and
	 * answers MP_APPLY_PENDING.
	 *
	 * `ui.disp.bl` is the one row the HOUSEKEEPING pass does not drain.
	 * ui_display.c rewrites TIM15_CH2 on every render frame, so the ui
	 * thread is PE6's single writer and claims the row itself
	 * (sts_pwrseq_req_is_foreign). `ui.lamp.test` gets a row of its own
	 * rather than sharing `ui.panel.duty`'s, because the mailbox's
	 * last-writer-wins rule is per OBJECT and a shared row would settle and
	 * withdraw the other object's leases.
	 */
	{ .id = "ui.disp.bl", .kind = MP_KIND_PCT, .guard = MP_GUARD_G1,
	  .group = MP_GRP_PANEL, .flags = F_LEASE, .min = 0, .max = 100,
	  .step = 1, .unit = U_PCT, .net = "DISP_BL",
	  .desc = "ST7796 module backlight duty (PE6)" },
	{ .id = "ui.disp.reset", .kind = MP_KIND_PULSE, .guard = MP_GUARD_G2,
	  .group = MP_GRP_PANEL, .flags = F_PULSE_D, .min = 1, .max = 200,
	  .step = 1, .unit = U_MS, .net = "DISP_RST",
	  .desc = "ST7796 reset, active low (PA10)" },
	{ .id = "ui.rgb.mode", .kind = MP_KIND_ENUM, .guard = MP_GUARD_G1,
	  .group = MP_GRP_PANEL, .flags = F_LEASE, .min = 0, .max = 5, .step = 1,
	  .enums = "auto,off,green,amber,red,blue-pulse", .net = "LED_R/G/B",
	  .desc = "D5 status RGB pattern; auto returns it to the fault policy" },
	{ .id = "ui.rgb.r", .kind = MP_KIND_PCT, .guard = MP_GUARD_G1,
	  .group = MP_GRP_PANEL, .flags = F_LEASE, .min = 0, .max = 100,
	  .step = 1, .unit = U_PCT, .net = "LED_R",
	  .desc = "D5 red leg, TIM4_CH1, common anode + low-side NPN (PD12)" },
	{ .id = "ui.rgb.g", .kind = MP_KIND_PCT, .guard = MP_GUARD_G1,
	  .group = MP_GRP_PANEL, .flags = F_LEASE, .min = 0, .max = 100,
	  .step = 1, .unit = U_PCT, .net = "LED_G",
	  .desc = "D5 green leg, TIM4_CH2 (PD13)" },
	{ .id = "ui.rgb.b", .kind = MP_KIND_PCT, .guard = MP_GUARD_G1,
	  .group = MP_GRP_PANEL, .flags = F_LEASE, .min = 0, .max = 100,
	  .step = 1, .unit = U_PCT, .net = "LED_B",
	  .desc = "D5 blue leg, TIM4_CH3 (PD14)" },
	{ .id = "ui.lamp.test", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G1,
	  .group = MP_GRP_PANEL, .flags = F_LEASE,
	  .desc = "all panel indicators on, for a lamp test" },
	/*
	 * G1, not G0, despite being "which box is this in the rack".
	 *
	 * It carries F_RW, i.e. MP_OF_OVERRIDE, so at G0 an unauthenticated
	 * session could take a lease — consuming one of MP_LEASE_MAX slots and
	 * engaging the dead-man and the actuation path — which is not "a change
	 * with no service consequence" (mp_manifest.h's own definition of G0).
	 * And its whole behaviour is to outrank the fault colour: suppressing the
	 * board's primary annunciation is a service consequence by definition.
	 * The cost of the promotion is nil — the tool opens a session for
	 * everything else anyway, and MP mode already requires physical access to
	 * the console port.
	 */
	{ .id = "ui.identify", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G1,
	  .group = MP_GRP_PANEL, .flags = F_RW,
	  .desc = "D5 identify pulse; outranks the fault colour by policy" },

	/* --------------------------------------------------- §6.5 system --- */
	/*
	 * Lease-only like the rest of the mailbox, and the only row whose
	 * override is a FLOOR rather than a level: housekeeping programs
	 * max(thermal loop, override), and a release resolves to full airflow
	 * rather than to the loop's last answer (ARCHITECTURE.md §10.9).
	 * MP_ILK_FAN_FLOOR clamps the request up at grant; the executor is what
	 * keeps it there as the box heats up.
	 */
	{ .id = "sys.fan.duty", .kind = MP_KIND_PCT, .guard = MP_GUARD_G2,
	  .group = MP_GRP_SYSTEM, .flags = F_LEASE, .min = 0, .max = 100,
	  .step = 1, .unit = U_PCT, .ilk = MP_ILK_FAN_FLOOR,
	  .net = "FAN_PWM",
	  .desc = "25 kHz fan PWM, TIM15_CH1; idle/fault state is full speed (PE5)" },
	/*
	 * THE EXTERNAL WATCHDOG, AND WHY THE TWO ROWS DO NOT SHARE AN INTERLOCK.
	 *
	 * `sys.wdt.en` is F_LEASE rather than F_RW for the reason the tunnels
	 * are: `obj.set` creates no lease, so a bare write that dropped WDT_EN
	 * would leave the TPS3430 disarmed with nothing in the system able to
	 * re-arm it — not the dead-man, not a link drop, not `session.close`.
	 * The watchdog's fail-safe direction is ON, so the release direction is
	 * the protected one and it has to be reachable.
	 *
	 * MP_ILK_WDT_LIVE belongs here and only here: "do not arm a watchdog you
	 * cannot prove you are feeding". It guards the ASSERT direction only, so
	 * disarming stays available while the liveness gate is unknown — which
	 * is exactly the state a technician needs to reach before pulsing WDI.
	 *
	 * `sys.wdt.kick` carries MP_ILK_WDT_OFF INSTEAD. MP_ILK_WDT_LIVE on this
	 * row was inverted in effect: m_obj_pulse() evaluates every pulse
	 * against a hardcoded request of 1, so the liveness bit refused the
	 * edge while the supervisor was deliberately withholding kicks (the pin
	 * idle — a harmless pulse) and GRANTED it while the supervisor was
	 * kicking on the 920-1360 ms cadence, where an arbitrary-phase edge
	 * inside tWDL(min) = 680 ms is a runaway fault that drives WDO_N ->
	 * POE_KILL and drops the board's own PoE port. See mp_manifest.h over
	 * MP_ILK_WDT_OFF and docs/sts1000_external_wdt.md §4.
	 */
	{ .id = "sys.wdt.en", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G3,
	  .group = MP_GRP_SYSTEM, .flags = F_LEASE, .ilk = MP_ILK_WDT_LIVE,
	  .net = "WDT_EN",
	  .desc = "U64 TPS3430 external windowed watchdog enable (PC12)" },
	{ .id = "sys.wdt.kick", .kind = MP_KIND_PULSE, .guard = MP_GUARD_G3,
	  .group = MP_GRP_SYSTEM, .flags = F_PULSE, .min = 1, .max = 10,
	  .step = 1, .unit = U_MS, .ilk = MP_ILK_WDT_OFF, .net = "WDT_KICK",
	  .desc = "U64 WDI refresh; normally only the supervisor drives it (PB2)" },
	{ .id = "sys.nor.reset", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G2,
	  .group = MP_GRP_SYSTEM, .flags = F_RWL_D, .net = "NOR_RST_N",
	  .desc = "U62 MX25L25645 reset, active low (PE10)" },
	{ .id = "sys.phy.reset", .kind = MP_KIND_PULSE, .guard = MP_GUARD_G2,
	  .group = MP_GRP_SYSTEM, .flags = F_PULSE | MP_OF_ACTIVE_LOW,
	  .min = 1, .max = 1000, .step = 1, .unit = U_MS,
	  .net = "LAN_RST_N",
	  .desc = "LAN8742AI reset, active low, 100 us minimum (PD10)" },
	{ .id = "sys.smp.tunnel", .kind = MP_KIND_BOOL, .guard = MP_GUARD_G2,
	  .group = MP_GRP_SYSTEM, .flags = F_LEASE_D, .ilk = MP_ILK_TUNNEL,
	  .desc = "MCUmgr/SMP tunnel on channel 0x06" },

	/* --------------------------------------------------- §7.1 sensors -- */
	/* Nine INA228 rails; identity comes from ina228_rail_tbl. */
	{ .id = "sensor.rail.poe", .kind = MP_KIND_RAIL, .group = MP_GRP_SENSOR,
	  .flags = F_RO, .min = INA228_RAIL_POE,
	  .desc = "PoE input bus and current" },
	{ .id = "sensor.rail.3v3_stm", .kind = MP_KIND_RAIL,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = INA228_RAIL_3V3_STM,
	  .desc = "always-on housekeeping rail" },
	{ .id = "sensor.rail.5v_disp", .kind = MP_KIND_RAIL,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = INA228_RAIL_5V_DISP,
	  .desc = "display 5 V rail" },
	{ .id = "sensor.rail.3v3", .kind = MP_KIND_RAIL,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = INA228_RAIL_3V3_MAIN,
	  .desc = "main/general 3V3 rail" },
	{ .id = "sensor.rail.v_ant", .kind = MP_KIND_RAIL,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = INA228_RAIL_V_ANT,
	  .desc = "GNSS antenna bias; open shows as under-current" },
	{ .id = "sensor.rail.ocxo", .kind = MP_KIND_RAIL,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = INA228_RAIL_OCXO,
	  .desc = "OCXO rail; oven current is the warm-up indicator" },
	{ .id = "sensor.rail.vcc_rb", .kind = MP_KIND_RAIL,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = INA228_RAIL_VCC_RB,
	  .desc = "programmable rubidium rail; the rb.verify interlock reads it" },
	{ .id = "sensor.rail.3v3_gps", .kind = MP_KIND_RAIL,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = INA228_RAIL_3V3_GPS,
	  .desc = "GNSS receiver rail (0x4A: SHT45 owns 0x44)" },
	{ .id = "sensor.rail.panel_5v", .kind = MP_KIND_RAIL,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = INA228_RAIL_PANEL_5V,
	  .desc = "panel-LED 5 V; average over the PWM window" },

	/* Environment. */
	{ .id = "sensor.temp.osc", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = -60000, .max = 150000,
	  .unit = U_MC, .desc = "U57 TMP117 0x49 oscillator temperature" },
	{ .id = "sensor.temp.amb", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = -60000, .max = 150000,
	  .unit = U_MC, .desc = "U58 TMP117 0x48 enclosure temperature" },
	{ .id = "sensor.temp.die", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = -60000, .max = 150000,
	  .unit = U_MC, .desc = "STM32H563 internal temperature sensor" },
	{ .id = "sensor.humidity", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 100000,
	  .unit = U_MPCT, .desc = "U72 SHT45 0x44 relative humidity" },
	{ .id = "sensor.fan.rpm", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 30000,
	  .unit = U_RPM, .desc = "FAN_TACH edge count over 1 s (PA15)" },
	{ .id = "sensor.fan.duty", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 100,
	  .unit = U_PCT, .desc = "commanded fan duty" },
	{ .id = "sensor.poe.class", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 8,
	  .desc = "U9 NCP1095 negotiated class; 0 = unknown" },
	{ .id = "sensor.poe.draw_mw", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 100000,
	  .unit = U_MW, .desc = "measured PoE draw (INA228 0x40)" },

	/* Direct-scan bitmaps — the replacement for the absent I/O expander. */
	{ .id = "sensor.pg", .kind = MP_KIND_BITS, .group = MP_GRP_SENSOR,
	  .flags = F_RO_D,
	  .enums = "3v3_gps_ldo,ocxo_ldo,3v0_rf_ldo,5v_psu,3v3_psu,ocxo_psu,"
		   "rb_psu,poe",
	  .desc = "GPIOG[0:7] rail power-good, 1 = asserted-low = not good" },
	{ .id = "sensor.ina.alert", .kind = MP_KIND_BITS,
	  .group = MP_GRP_SENSOR, .flags = F_RO_D,
	  .enums = "v_poe,3v3_stm,5v_disp,3v3,3v3_gps,v_ant,ocxo,vcc_rb,panel",
	  .desc = "INA228 ALERT lines: GPIOG[8:15] plus PF13 for U54" },
	{ .id = "sensor.en.fault", .kind = MP_KIND_BITS,
	  .group = MP_GRP_SENSOR, .flags = F_RO_D,
	  .enums = "v_ant,v_disp,panel_led",
	  .desc = "RT9742 nFLG flags U27/U33/U55 (PF8, PF9, PF12)" },
	{ .id = "sensor.bkp.pg", .kind = MP_KIND_BITS, .group = MP_GRP_SENSOR,
	  .flags = F_RO, .enums = "stm,gps",
	  .desc = "TPS61094 supercap backup power-good (PF14, PF15)" },
	{ .id = "sensor.buttons", .kind = MP_KIND_BITS,
	  .group = MP_GRP_SENSOR, .flags = F_RO_D,
	  .enums = "b1,b2,b3,b4,b5,b6,b7,encoder",
	  .desc = "panel buttons PF0..PF6 and the encoder switch PF11" },
	{ .id = "sensor.poe.status", .kind = MP_KIND_BITS,
	  .group = MP_GRP_SENSOR, .flags = F_RO_D, .enums = "ncm,lcf,ncl",
	  .desc = "U9 NCP1095 open-drain status (PC2, PC3, PC7)" },

	/* Discrete inputs outside the scan. */
	{ .id = "sensor.rb.lock", .kind = MP_KIND_BOOL, .group = MP_GRP_SENSOR,
	  .flags = F_RO, .net = "RB_LOCK",
	  .desc = "FE-5680A lock via U48 opto (PB13)" },
	{ .id = "sensor.rb.ov", .kind = MP_KIND_BOOL, .group = MP_GRP_SENSOR,
	  .flags = F_RO, .net = "RB_OV_DET",
	  .desc = "autonomous 26 V OV latch state (PE3)" },
	{ .id = "sensor.extref.hz", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 20000000,
	  .unit = U_HZ, .net = "EXTREF_MON",
	  .desc = "TIM12_CH1 capture of the mux B input (PB14)" },
	{ .id = "sensor.pfi", .kind = MP_KIND_BOOL, .group = MP_GRP_SENSOR,
	  .flags = F_RO_D, .net = "PFI",
	  .desc = "U2A power-fail early warning, 38.1 V trip (PE8)" },
	{ .id = "sensor.usb.vbus", .kind = MP_KIND_BOOL,
	  .group = MP_GRP_SENSOR, .flags = F_RO_D, .net = "USB_VBUS_SENSE",
	  .desc = "USB-C VBUS presence via divider (PE2)" },
	{ .id = "sensor.gps.txrdy", .kind = MP_KIND_BOOL,
	  .group = MP_GRP_SENSOR, .flags = F_RO_D, .net = "GPS_TXRDY",
	  .desc = "ZED-F9T TX-ready; valid only after the CFG-TXREADY ACK (PD5)" },
	{ .id = "sensor.gps.ant_off", .kind = MP_KIND_BOOL,
	  .group = MP_GRP_SENSOR, .flags = F_RO_D, .net = "GPS_ANT_OFF_MON",
	  .desc = "ZED-F9T LNA-off indication (PD4)" },
	{ .id = "sensor.ocxo.vc", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 3300,
	  .unit = U_MV, .net = "OCXO_V",
	  .desc = "ADC read-back of the OCXO Vc loop filter (PA3)" },
	{ .id = "sensor.enc.pos", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO_D, .min = -2147483647,
	  .max = 2147483647, .net = "ENC_A/ENC_B",
	  .desc = "TIM1 hardware quadrature position (PA8/PA9)" },

	/* Timing and GNSS observables (from the §3.8 quality block). */
	{ .id = "sensor.timing.stratum", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 16,
	  .desc = "served NTP stratum; 16 = unsynchronised" },
	{ .id = "sensor.timing.lock", .kind = MP_KIND_ENUM,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 6,
	  .enums = "unknown,acquiring,locking,locked,holdover,recovering,parked",
	  .desc = "discipline loop state" },
	{ .id = "sensor.timing.ref", .kind = MP_KIND_ENUM,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 3,
	  .enums = "none,ocxo,rb,extref",
	  .desc = "reference the clock mux is forwarding to PH0" },
	{ .id = "sensor.timing.pps_ns", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = -1000000000,
	  .max = 1000000000, .unit = U_NS,
	  .desc = "last accepted fully corrected PPS offset" },
	{ .id = "sensor.timing.freq_ppb", .kind = MP_KIND_REAL,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .unit = U_PPB,
	  .desc = "filtered residual fractional frequency" },
	{ .id = "sensor.timing.adev_1s", .kind = MP_KIND_REAL,
	  .group = MP_GRP_SENSOR, .flags = F_RO,
	  .desc = "overlapping ADEV, tau = 1 s" },
	{ .id = "sensor.timing.adev_10s", .kind = MP_KIND_REAL,
	  .group = MP_GRP_SENSOR, .flags = F_RO,
	  .desc = "overlapping ADEV, tau = 10 s" },
	{ .id = "sensor.timing.adev_100s", .kind = MP_KIND_REAL,
	  .group = MP_GRP_SENSOR, .flags = F_RO,
	  .desc = "overlapping ADEV, tau = 100 s" },
	{ .id = "sensor.timing.root_disp_ns", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 2147483647,
	  .unit = U_NS, .desc = "RFC 5905 root dispersion; grows in holdover" },
	{ .id = "sensor.timing.holdover_s", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 2147483647,
	  .unit = U_S, .desc = "seconds since holdover entry" },
	{ .id = "sensor.gnss.fix", .kind = MP_KIND_ENUM,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 3,
	  .enums = "none,2d,3d,time-only",
	  .desc = "receiver fix classification" },
	{ .id = "sensor.gnss.sv_used", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 64,
	  .desc = "satellites in the timing solution" },
	{ .id = "sensor.gnss.sv_visible", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 64,
	  .desc = "satellites tracked" },
	{ .id = "sensor.gnss.tacc_ns", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 2147483647,
	  .unit = U_NS, .desc = "receiver time-accuracy estimate" },
	{ .id = "sensor.gnss.ant", .kind = MP_KIND_ENUM,
	  .group = MP_GRP_SENSOR, .flags = F_RO_D, .min = 0, .max = 4,
	  .enums = "unknown,ok,open,short,off",
	  .desc = "antenna supervisor: MON-RF + PD4 + INA228 0x45 fused" },
	{ .id = "sensor.pwrseq.stage", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 15,
	  .desc = "bring-up stage machine position (interface ref §2)" },
	/*
	 * The active-alarm mask. Bits 0..31 are the scanned signals (fault_sig_t)
	 * and 32..48 the software-raised alarms (fault_alarm_id_t); the names are
	 * published here rather than left to the tool because an unlabelled 49-bit
	 * mask is useless at a bench. This is the only place the full list
	 * appears — the bitmap objects above are per-port groupings of the same
	 * signals.
	 */
	{ .id = "sensor.alarms", .kind = MP_KIND_BITS,
	  .group = MP_GRP_SENSOR, .flags = F_RO,
	  .enums = "button_1,button_2,button_3,button_4,button_5,button_6,"
		   "button_7,touch_int,v_ant_en_fault,v_disp_en_fault,"
		   "prox_wake,enc_button,panel_led_fault,ina_alert_panel,"
		   "bkp_stm_pg,bkp_gps_pg,pg_3v3_gps_ldo,pg_ocxo_ldo,"
		   "pg_3v0_rf_ldo,pg_5v_psu,pg_3v3_psu,pg_ocxo_psu,pg_rb_psu,"
		   "pg_poe,ina_alert_v_poe,ina_alert_3v3_stm,"
		   "ina_alert_5v_disp,ina_alert_3v3,ina_alert_3v3_gps,"
		   "ina_alert_v_ant,ina_alert_ocxo,ina_alert_vcc_rb,"
		   "reference_lost,gnss_lost,antenna_open,antenna_short,"
		   "ocxo_unhealthy,dac_fault,rb_fault,rb_ov,thermal_warn,"
		   "thermal_critical,fan_fault,poe_budget,i2c_wedge,nor_fault,"
		   "display_fault,pfi,tamper",
	  .desc = "active alarm mask: fault_sig_t 0..31, fault_alarm_id_t 32..48" },
	{ .id = "sensor.uptime_s", .kind = MP_KIND_SCALAR,
	  .group = MP_GRP_SENSOR, .flags = F_RO, .min = 0, .max = 2147483647,
	  .unit = U_S, .desc = "seconds since boot" },
	{ .id = "sensor.serial", .kind = MP_KIND_TEXT,
	  .group = MP_GRP_SENSOR, .flags = F_RO,
	  .desc = "device serial; the G2 confirmation string" },
	{ .id = "sensor.fw_version", .kind = MP_KIND_TEXT,
	  .group = MP_GRP_SENSOR, .flags = F_RO,
	  .desc = "running application version" },
};

#define MP_OBJ_N (sizeof(mp_objs) / sizeof(mp_objs[0]))

size_t mp_obj_count(void)
{
	return MP_OBJ_N;
}

const mp_obj_t *mp_obj_at(size_t i)
{
	return (i < MP_OBJ_N) ? &mp_objs[i] : NULL;
}

int mp_obj_find(const char *id)
{
	size_t i;

	if (id == NULL) {
		return -EINVAL;
	}
	for (i = 0U; i < MP_OBJ_N; i++) {
		if (strcmp(mp_objs[i].id, id) == 0) {
			return (int)i;
		}
	}
	return -ENOENT;
}

size_t mp_obj_count_in(uint8_t group)
{
	size_t i;
	size_t n = 0U;

	if (group >= (uint8_t)MP_GRP_COUNT) {
		return 0U;
	}
	for (i = 0U; i < MP_OBJ_N; i++) {
		if (mp_objs[i].group == group) {
			n++;
		}
	}
	return n;
}

int mp_obj_check_value(size_t idx, int32_t v)
{
	const mp_obj_t *o = mp_obj_at(idx);

	if (o == NULL) {
		return -EINVAL;
	}
	switch (o->kind) {
	case MP_KIND_BOOL:
		return ((v == 0) || (v == 1)) ? 0 : -ERANGE;
	case MP_KIND_ENUM:
	case MP_KIND_PCT:
	case MP_KIND_MV:
	case MP_KIND_CODE:
	case MP_KIND_PULSE:
	case MP_KIND_SCALAR:
		return ((v >= o->min) && (v <= o->max)) ? 0 : -ERANGE;
	default:
		/* REAL/RAIL/BITS/TEXT are observations, not settings. */
		return -ENOTSUP;
	}
}

/* --------------------------------------------------------------- emitting */

/** True for kinds whose min/max/step envelope is meaningful. */
static bool kind_has_range(uint8_t kind)
{
	switch (kind) {
	case MP_KIND_ENUM:
	case MP_KIND_PCT:
	case MP_KIND_MV:
	case MP_KIND_CODE:
	case MP_KIND_PULSE:
	case MP_KIND_SCALAR:
		return true;
	default:
		return false;
	}
}

/** Emit `enums` as a JSON array of strings. */
static void emit_enums(mp_jw_t *w, const char *csv)
{
	const char *p = csv;

	(void)mp_jw_arr_open(w);
	while (*p != '\0') {
		const char *comma = strchr(p, ',');
		size_t n = (comma != NULL) ? (size_t)(comma - p) : strlen(p);

		(void)mp_jw_strn(w, p, n);
		if (comma == NULL) {
			break;
		}
		p = comma + 1;
	}
	(void)mp_jw_arr_close(w);
}

static void emit_flags(mp_jw_t *w, uint8_t flags)
{
	static const struct {
		uint8_t bit;
		const char *name;
	} tbl[] = {
		{ MP_OF_READ, "read" },
		{ MP_OF_WRITE, "write" },
		{ MP_OF_OVERRIDE, "override" },
		{ MP_OF_PULSE, "pulse" },
		{ MP_OF_ACTIVE_LOW, "active-low" },
		{ MP_OF_CFG, "cfg" },
		{ MP_OF_DEFERRED, "deferred" },
	};
	size_t i;

	(void)mp_jw_arr_open(w);
	for (i = 0U; i < (sizeof(tbl) / sizeof(tbl[0])); i++) {
		if ((flags & tbl[i].bit) != 0U) {
			(void)mp_jw_str(w, tbl[i].name);
		}
	}
	(void)mp_jw_arr_close(w);
}

static void emit_ilk(mp_jw_t *w, uint32_t mask)
{
	size_t i;

	(void)mp_jw_arr_open(w);
	for (i = 0U; i < MP_ILK_COUNT; i++) {
		if ((mask & ilks[i].bit) != 0U) {
			(void)mp_jw_str(w, ilks[i].name);
		}
	}
	(void)mp_jw_arr_close(w);
}

int mp_manifest_obj_json(size_t idx, char *buf, size_t cap)
{
	const mp_obj_t *o = mp_obj_at(idx);
	mp_jw_t w;
	size_t len = 0U;
	int rc;

	if ((o == NULL) || (buf == NULL) || (cap == 0U)) {
		return -EINVAL;
	}

	rc = mp_jw_init(&w, buf, cap);
	if (rc != 0) {
		return rc;
	}

	(void)mp_jw_obj_open(&w);
	(void)mp_jw_kv_str(&w, "id", o->id);
	(void)mp_jw_kv_str(&w, "kind", mp_kind_name(o->kind));
	(void)mp_jw_kv_str(&w, "group", mp_group_name(o->group));
	(void)mp_jw_kv_str(&w, "guard", mp_guard_name(o->guard));

	(void)mp_jw_key(&w, "flags");
	emit_flags(&w, o->flags);

	if (kind_has_range(o->kind)) {
		(void)mp_jw_kv_i64(&w, "min", o->min);
		(void)mp_jw_kv_i64(&w, "max", o->max);
		if (o->step != 0) {
			(void)mp_jw_kv_i64(&w, "step", o->step);
		}
	}
	if (o->unit != NULL) {
		(void)mp_jw_kv_str(&w, "unit", o->unit);
	}
	if (o->enums != NULL) {
		(void)mp_jw_key(&w, "enum");
		emit_enums(&w, o->enums);
	}
	if ((o->flags & MP_OF_CFG) != 0U) {
		(void)mp_jw_kv_u64(&w, "cfg", o->cfg_key);
	}
	if (o->kind == (uint8_t)MP_KIND_RAIL) {
		const ina228_rail_info_t *r =
			ina228_rail((ina228_rail_t)o->min);

		if (r != NULL) {
			(void)mp_jw_kv_str(&w, "net", r->name);
			(void)mp_jw_kv_str(&w, "dev", r->designator);
			(void)mp_jw_kv_str(&w, "shunt", r->shunt_ref);
			(void)mp_jw_kv_u64(&w, "addr", r->addr);
			(void)mp_jw_kv_u64(&w, "shunt_uohm", r->r_shunt_uohm);
			(void)mp_jw_kv_u64(&w, "fs_ua", r->fs_current_ua);
		}
	} else if (o->net != NULL) {
		(void)mp_jw_kv_str(&w, "net", o->net);
	} else {
		/* no net: a logical object such as ui.lamp.test */
	}
	if (o->ilk != 0U) {
		(void)mp_jw_key(&w, "ilk");
		emit_ilk(&w, o->ilk);
	}
	(void)mp_jw_kv_str(&w, "desc", o->desc);
	(void)mp_jw_obj_close(&w);

	rc = mp_jw_finish(&w, &len);
	if (rc != 0) {
		return rc;
	}
	return (int)len;
}

int mp_manifest_page(size_t from, char *buf, size_t cap, size_t *out_len,
		     size_t *out_next)
{
	size_t used = 0U;
	size_t i = from;
	int n = 0;

	if ((buf == NULL) || (cap == 0U) || (out_len == NULL) ||
	    (out_next == NULL)) {
		return -EINVAL;
	}
	if (from > MP_OBJ_N) {
		return -EINVAL;
	}

	buf[0] = '\0';

	while (i < MP_OBJ_N) {
		size_t sep = (n > 0) ? 1U : 0U;
		int rc;

		/* Try the object into the remaining space, leaving room for the
		 * separator and the NUL. */
		if ((used + sep + 2U) >= cap) {
			break;
		}
		rc = mp_manifest_obj_json(i, &buf[used + sep],
					  cap - used - sep);
		if (rc == -ENOSPC) {
			break;
		}
		if (rc < 0) {
			return rc;
		}
		if (sep != 0U) {
			buf[used] = ',';
		}
		used += sep + (size_t)rc;
		buf[used] = '\0';
		i++;
		n++;
	}

	*out_len = used;
	*out_next = i;
	return n;
}

uint32_t mp_manifest_hash(char *scratch, size_t cap)
{
	uint32_t crc = STS_CRC32_IEEE_SEED;
	const char ver = (char)('0' + (char)(MP_MANIFEST_VER % 10U));
	size_t i;

	if ((scratch == NULL) || (cap < MP_MANIFEST_OBJ_JSON_MAX)) {
		return 0U;
	}

	crc = sts_crc32_ieee_update(crc, &ver, 1U);

	for (i = 0U; i < MP_OBJ_N; i++) {
		int n = mp_manifest_obj_json(i, scratch, cap);

		if (n < 0) {
			/* A table entry that cannot be serialised would make the
			 * hash meaningless; fold the index in so the mismatch is
			 * visible rather than silently skipped. */
			crc = sts_crc32_ieee_update(crc, &i, sizeof(i));
			continue;
		}
		crc = sts_crc32_ieee_update(crc, scratch, (size_t)n);
	}
	return crc;
}
