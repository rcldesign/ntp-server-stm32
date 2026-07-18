# STS1000 "Meridian" — External Window Watchdog & Board Cold-Cycle Arming

External windowed watchdog that backstops firmware liveness and, on a window
violation, arms the board cold-cycle (`POE_KILL`). Also documents the role of the
I²C bus accelerator (`LTC4311`) and the I²C-wedge recovery procedure. Datasheet-verified
against TI SBVS366A (TPS3430) and ADI 4311fa (LTC4311).

---

## 1. Role & division of labor

| Layer | Mechanism | Action on fault | Covers |
|---|---|---|---|
| On-chip | **IWDG** (hardware-start) | MCU reset (NRST-class) | MCU hang, incl. early boot before the external WDT is armed |
| Board | **External WDT (U64, TPS3430)** | assert `POE_KILL` → full board cold-cycle | operational hangs that survive an MCU reset; runaway loops |

Principle: the cheapest sufficient reset is used first (IWDG resets the core); the
external WDT escalates to a full power cold-cycle only when the core-level reset is
not enough. The external WDT is **disabled at boot** and armed by firmware once the
supervisor task is confirmed live (§5); the IWDG covers the pre-arm window.

---

## 2. Part selection

| Desig. | Part (orderable) | Pkg | Notes |
|---|---|---|---|
| **U64** | TPS3430WDRCR (std) / TPS3430-Q1¹ | VSON-10 DRC, 3×3 mm | Standalone **window** watchdog; VDD 1.6–6.5 V; IDD ~10 µA; open-drain `WDO`. Active/Production (marking 430AA). |

¹ Automotive AEC-Q100 grade qualified version exists ("TPS3430-Q1"); confirm exact orderable suffix on ti.com if used.

Why a dedicated window WDT (not a supervisor + simple WDT): the window catches a
**runaway/babbling** firmware (kicks too fast) as well as a stall (kicks too slow).
Both are real failure modes for the timing/network supervisor.

---

## 3. U64 connections (every pin)

| Pin | Name | Net | Strap / note |
|---|---|---|---|
| 1 | VDD1 | 3V3_STM | always-on logic rail; **C180 = 0.1 µF** bypass close to pin |
| 2 | CWD | **NC (float)** | selects factory window preset; keep-out, no copper fill, minimize parasitic C |
| 3 | SET0 | **3V3_STM** | hard-tied high |
| 4 | CRST | **NC (float)** | selects tRST = 200 ms preset; no fill |
| 5 | GND | GND | |
| 6 | SET1 | **WDT_EN (PC12)** | enable/ratio control (§5); **R213 = 100 kΩ pulldown** so it boots disabled |
| 7 | WDI | **WDT_KICK (PB2)** | falling-edge triggered; **no pulldown** (net is a 2-node PB2↔WDI link; PB2 push-pull holds a defined level) |
| 8 | WDO | **WDO_N** | open-drain active-low; pull-up **R23 = 10 kΩ** (§6) |
| 9 | NC | float | must not connect |
| 10 | VDD2 | 3V3_STM | **must** tie to VDD1 — part is non-functional otherwise |
| Pad | — | GND | thermal pad to ground plane |

U64 sits on `3V3_STM` (always-on relative to the gated peripheral rails). It is
downstream of the `POE_KILL` pass-FET, so a cold-cycle de-powers U64 and it
re-arms fresh on restore — the desired behavior.

---

## 4. Window timing

**Strap:** SET0 = 1, SET1 = 1 (enabled), CWD = NC, CRST = NC → factory preset, **zero timing caps.**

| Parameter | min | typ | max |
|---|---|---|---|
| tWDL (lower / too-early boundary) | 680 | 800 | 920 ms |
| tWDU (upper / too-late boundary) | 1360 | 1600 | 1840 ms |
| tRST (WDO assert duration) | 170 | 200 | 230 ms |

Derived behavior:

- **Valid kick window** = (tWDL(max), tWDU(min)) = **920–1360 ms** between consecutive WDI falling edges.
- **Nominal kick cadence: 1.10 s** (+180 ms above the early bound, −260 ms below the late bound — ample for a software timer).
- **Stall → fault** by ≤ tWDU(max) = **1.84 s** → cold-cycle.
- **Runaway → fault** if kicks arrive faster than tWDL(min) = **0.68 s** → cold-cycle.
- tRST (≈200 ms WDO assert) is far longer than needed to arm the kill RC; once the
  board collapses the on-board RC owns the off-time, so the exact tRST is non-critical.

**Programmable alternative (not required for this design):** a CWD cap sets
`tWDU(typ) = 77.4·C_CWD + 0.055` (s, µF), with the 1:2 ratio giving
`tWDL = 0.5·tWDU`. Example: C_CWD = 39 nF C0G → tWDU ≈ 3.07 s, valid window ≈ 1.68–2.78 s.
Reserve for a future cadence change; the no-cap preset is the baseline.

> The TPS3430 power-up "first-kick" deadline (tRST + first-window) does **not** gate
> the secure-boot chain in this design, because the part boots **disabled** (§5) and
> is armed only after firmware is up. The first WDI edge is issued by firmware
> immediately after it asserts `WDT_EN`, well inside the first window.

---

## 5. Enable / disable control — `WDT_EN` (PC12)

The TPS3430 is enabled/disabled by the SET pins. SET0 is tied high; **SET1 is driven
directly by the MCU on PC12 (`WDT_EN`, GPIO push-pull).**

| SET0 | SET1 (PC12) | Mode |
|---|---|---|
| 1 | 0 | **Disabled** — WDO held high, all WDI activity ignored |
| 1 | 1 | **Enabled**, 1:2 ratio, window per §4 |

- **Boot-disabled by default.** At MCU reset PC12 is Hi-Z; **R213 (100 kΩ pulldown)**
  holds SET1 low → disabled. Firmware drives PC12 high to arm, after the supervisor
  task is confirmed running.
- **Enable transition** takes effect immediately, with a 150 µs (tWD-setup) blanking
  before the device responds to WDI; the first WDI falling edge after arming must
  occur before tWDU(max) — trivially met since firmware arms then kicks.
- **Single-pin change.** Only SET1 moves (SET0 fixed high); the disable↔enable
  transition is immediate and does not require the 500 µs (tSET) inter-pin spacing
  that applies only when both SET pins are changed.
- **Disabled-state hygiene.** Per datasheet, WDI must not float when disabled. `WDT_KICK`
  is a direct 2-node link (PB2 ↔ U64 WDI) with **no pulldown**; R169 is not on this net (it
  is the Q21 relay-driver base pulldown in the Rb-IO block). PB2 is push-pull and holds a
  defined level once GPIO is configured; at the pre-GPIO-config boot instant WDI is
  briefly Hi-Z, harmless because the WDT is held disabled by R213 until firmware arms it.

Direct-GPIO control (rather than an I/O-expander bit) is deliberate: the board watchdog's
arm/disarm line must not depend on the I²C bus it helps protect, and it must remain
controllable when that bus is wedged. No I/O expander sits in this path — every such signal
is direct GPIO.

---

## 6. `WDO` → `POE_KILL` interface

`WDO` is **open-drain active-low** (asserts low on a window violation for tRST). This
is the correct polarity to join the board's active-low fault collection directly.

### Primary: `WDO_N` → POE_KILL WDT input stage

**`WDO_N` is WDO-only** — not a three-way wire-OR. The net has exactly three
nodes: `U64` pin 8 (`WDO`, open-drain), the pull-up **R23 (10 kΩ → 3V3)**, and **R24**
(series into the `POE_KILL` WDT input stage, Q4 base). There are no thermal-cutoff or Rb-OV
taps on this node: the Rb OV event reaches the MCU on its own `RB_OV_DET` path (Rb-supply
doc §4), and there is no separate thermal open-collector here. `WDO_N` low drives the double inversion
(Q4→Q5) inside the `POE_KILL` kill-driver, which pulls `POE_KILL` high and arms the kill
RC that opens the pass switch. Firmware (`POE_KILL`, PE15) ORs in on the high side.

```
WDO(U64.8) ──┬── WDO_N ── R24 ──► POE_KILL WDT input stage (Q4→Q5) ──► KILL_N low ──► AUX assert ──► NCP1095 offline
             R23 10k
             (to 3V3)
```

Trip sequence: supervisor stops kicking → window violated → `WDO` pulls `WDO_N`
low → kill-driver forces the NCP1095 offline (POE_KILL doc) → 3V3_STM decays over the
buck output caps (ms-class), during which U64 stays powered and `WDO` holds the assert →
the on-board kill RC latches the off-time; the PSE then re-detects → board
reboots, U64 re-arms.

### Fallback: active-high collection

Not used in this design (the POE_KILL WDT input stage takes `WDO_N` active-low
directly). Documented only in case the kill-driver collection node is ever changed to
accept **active-high** asserts: add a 74LVC1G04GW-Q100H inverter on 3V3_STM (matches the
project's 74LVC1G157 / 74LVC1G34 family), `WDO` → inverter → a Schottky (BAT54 /
RB521S-30, anode at inverter output) → `POE_KILL`, with `WDO` pulled up to 3V3 via R23,
plus a 0.1 µF inverter bypass. **None of these parts are populated and none carry a
refdes.**

---

## 7. Firmware implications

### 7.1 Kick (`WDT_KICK`, PB2)
- Hold PB2 high; emit one falling edge per kick (≥ 50 ns low), once per **1.10 s**, from
  the supervisor task — **only** when timing, network, and housekeeping threads all
  report healthy. A windowed WDT means kicking unconditionally on a timer defeats
  the purpose; the kick must be gated on liveness.

### 7.2 Arm sequence (`WDT_EN`, PC12)
1. Boot with PC12 left to its reset Hi-Z (R213 holds the WDT disabled).
2. Start IWDG (hardware-start, §7.3) so the MCU is covered from instruction zero.
3. Bring up the supervisor; confirm all monitored threads report in.
4. **If no debugger attached** (§7.4): drive PC12 high to arm, then begin the 1.10 s kick loop.

### 7.3 Internal watchdog
- **IWDG hardware-start** via the flash option byte (IWDG software/hardware select —
  confirm field name in RM0481) so the IWDG runs before any firmware executes,
  closing the pre-arm coverage gap left by the boot-disabled external WDT.
- Set the **DBGMCU debug-freeze** bits for IWDG/WWDG (DBGMCU APBx freeze register —
  confirm exact field in RM0481) so the internal watchdog pauses while the core is
  halted at a breakpoint and resumes on run.

### 7.4 Interactive debugging
The external WDT cannot freeze on a core halt (no freeze input), so it is **disabled**
for a debug session rather than frozen:

- **Connect-under-reset** is the standard flow: the probe attaches with `C_DEBUGEN`
  set before release of reset. Firmware reads `CoreDebug->DHCSR & C_DEBUGEN` early
  and, if a debugger is present, **skips arming** the external WDT (leaves PC12 low →
  WDT disabled). No mid-boot race.
- **Live toggle:** a console command (e.g. `dev wd ext off|on`) drives PC12 directly.
  Because it is a direct GPIO, this works even with the I²C bus wedged. Gate the
  command behind **debug-auth** (`sec` group) so a fielded unit cannot have its board
  watchdog switched off without authorization.
- While disabled, `WDO` stays high and `POE_KILL` is not asserted regardless of WDI,
  so halts/single-stepping are safe.

### 7.5 State summary
| Condition | External WDT | IWDG |
|---|---|---|
| Boot, pre-supervisor | disabled (R213) | running (HW-start) |
| Operational, no debugger | armed, kicked at 1.10 s | running, kicked |
| Debugger attached | disabled (PC12 low) | frozen-on-halt (DBGMCU) |
| Thread stall / runaway | trips → `POE_KILL` cold-cycle | (may also trip MCU reset) |

---

## 8. I²C bus accelerator (`LTC4311`) & wedge-recovery procedure

### 8.1 The accelerator is not a buffer
`U56` **LTC4311** (SC70-6) is a **dual I²C active pull-up / rise-time accelerator**, not a
segmenting buffer. It taps onto the bus lines (BUS1/BUS2) and boosts positive
transitions; it has **no IN/OUT isolation**. Its ENABLE pin only gates the
accelerator (≈1 V threshold; >1.5 V = on; <0.4 V = <5 µA shutdown with BUS pins
high-impedance), and the part **does not load the bus when disabled** — so disabling
it cannot isolate or recover a wedged bus.

| Pin | Net | Strap |
|---|---|---|
| VCC | 3V3_STM | bus supply rail; 0.1 µF bypass |
| GND | GND | ground plane |
| BUS1 | I2C_SCL (PB8) | active pull-up tap |
| BUS2 | I2C_SDA (PB9) | active pull-up tap |
| ENABLE | **3V3_STM** | **tied on, always enabled** (auto-standby handles idle current) |

There is no `I2C_BUF_EN` control: the LTC4311 ENABLE is tied on and is not a recovery
lever. `WDT_EN` is on **PC12** (§5); PA8/PA9 carry the encoder. The static bus pull-ups
are independent of the LTC4311, which provides no internal pull-ups.

### 8.2 Bus-wedge recovery ladder
None of these involve the LTC4311.

1. **Controller recovery:** reconfigure SCL (PB8) as GPIO output, clock out ≥ 9
   pulses to release a slave stuck mid-byte, generate a STOP, re-init I2C1. Clears a
   slave holding SDA low.
2. **`POE_KILL`** full cold-cycle as last resort. (There is no I/O-expander reset lever —
   no MCP23017 expanders exist, so there is no `EXP_RST_N`; and housekeeping is on
   always-on 3V3_STM, so there is no rail-cycle recovery path either.)

---

## 9. Layout notes (per TI SBVS366A §10)
- 0.1 µF bypass (C180, with C181 on VDD2) as close as possible to VDD1; keep the VDD connection low-impedance.
- CWD and CRST are floated (NC): minimize parasitic capacitance on both pins (short
  traces, no plane fill under the pins) so the factory presets evaluate correctly.
- Place the WDO pull-up (R23) close to pin 8.
- Thermal pad to a large-area ground plane (internally GND).

---

## 10. BOM

| Desig. | Part / value | Pkg | Function |
|---|---|---|---|
| U64 | TPS3430WDRCR (or TPS3430-Q1) | VSON-10 | window watchdog |
| C180 | 0.1 µF X7R | 0402 | U64 VDD1 bypass |
| C181 | 0.1 µF X7R | 0402 | U64 VDD2 bypass |
| R23 | 10 kΩ (E96) | 0402 | WDO pull-up → WDO_N |
| R213 | 100 kΩ (E96) | 0402 | SET1/`WDT_EN` (PC12) pulldown (boot-disabled default) |
| *inverter* | *74LVC1G04GW-Q100H* | *SC-70* | *fallback only (active-high collection) — no refdes; not populated* |
| *bypass* | *0.1 µF X7R* | *0402* | *inverter bypass — fallback only; not populated* |
| *Schottky* | *BAT54 / RB521S-30* | *SOD-323* | *OR-diode — fallback only; not populated* |
| *CWD cap* | *39 nF C0G* | *0402* | *window-stretch — optional, not in baseline* |
| — | U56 LTC4311 ENABLE → 3V3_STM | — | strap-on (no new part) |

Italic rows are conditional and not populated in the baseline.

---

## 11. Open items / bench verify
- [ ] Measure STiRoT → MCUboot → Zephyr → supervisor-running time; confirm the
      arm-then-kick sequence (§7.2) lands the first WDI edge within the first window
      after `WDT_EN` is asserted (decoupled from total boot time, but verify).
- [ ] Confirm the `POE_KILL` collection-node polarity: active-low `WDO_N` wire-OR
      (primary, WDO direct) vs active-high (fallback inverter + OR-diode).
- [ ] Confirm IWDG hardware-start option-byte field and the DBGMCU IWDG/WWDG
      freeze register fields in RM0481.
- [ ] Bench-confirm a window violation (stall and runaway) drives `WDO_N` low and
      cold-cycles the board; confirm `WDO` tRST assert overlaps the rail collapse.

---

## 12. Cross-references
- **Pin map / software spec:** `WDT_EN (PC12) → TPS3430 (U64) SET1`; there is no `I2C_BUF_EN`. PA8/PA9 are the encoder (`ENC_A`/`ENC_B`).
- **Reliability/monitoring:** `WDT_KICK (PB2)` is the kick line; PC12 is the arm line (boot-disabled). `WDO_N` is WDO-only (no thermal/Rb-OV wire-OR).
- **Checklist:** LTC4311 (U56) ENABLE tied to 3V3_STM (always on); U64 per §1/§9.
- **Recovery matrix:** the I²C-wedge ladder has no expander-reset step (no MCP23017 expanders, no `EXP_RST_N`); use the §8.2 ladder.
