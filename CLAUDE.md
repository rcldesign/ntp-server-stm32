# STS1000 "Meridian" — Project Context for Claude Code

GPS-disciplined Stratum-1 NTP/NTS/PTP grandmaster clock. Single-board, PoE-powered,
built around an **STM32H563ZIT6** (LQFP144) and a **u-blox ZED-F9T-00B** GNSS timing
receiver, with **dual permanent timing references**: an onboard **OH300 VC-OCXO**
(always on, STM32-steered) and an **external FE-5680A rubidium** (10 MHz over coax to
an SMA front end; power/serial/lock/PPS over a housekeeping connector). Full NTP/NTS/PTP
stack, local TFT+touch UI, RS-232 Rb telemetry, hardware-rooted secure boot, and a
complete power-monitoring/fault-aggregation architecture. Target deployment: the
Observatory at Williams Manor.

This file orients an agent picking up the project. Electrical and behavioral detail
lives in `docs/` — this is the index and the rulebook, not a substitute for the docs.

---

## Repository layout

```
/                      project root (this file lives here)
  CLAUDE.md            ← you are here
  docs/                canonical design documents (authoritative reference)
  firmware/            Zephyr application + boot chain (see Software stack)
  hardware/            KiCad schematic / layout / BOM
```

`docs/` is the source of truth for the design. Treat each document as a **unified
authoritative reference**, not a changelog — when a decision changes, edit the relevant
doc(s) to reflect the new as-built state and **propagate the delta across every doc it
touches in the same change**. Do not leave docs disagreeing with each other.

---

## Document map (`docs/`)

| Document | Scope | Status |
|---|---|---|
| `ntp_server_peripheral_map.md` | **Master hardware reference.** Every STM32 pin/net, all buses (RMII, I²C1, SPI4, UARTs), power domains, PoE budget, ADC, EXTI allocation, reset matrix, spare pins (none), open items. | Authoritative |
| `ntp_server_software_spec.md` | **Master firmware spec.** Zephyr stack, thread model, timing engine, NTP/NTS/PTP services, web/SNMP/console, secure boot, security, telemetry/calibration, state machines, fault tree. Prescriptive (MUST/SHOULD/MAY). | Authoritative |
| `sts1000_schematic_implementation_checklist.md` | IC/circuit inventory by block with designators; INA228 address map; standing KiCad actions. **Best starting point for a schematic/BOM review.** | Active checklist |
| `sts1000_clock_mux.md` | OCXO/ext-ref → PH0 selector (74LVC1G157 + bench fanout). Schematic-verified, as-built. | As-built |
| `ntp_server_rf_frontend.md` | External-reference SMA front end (single LTC6752xS5 slicer, switched 50 Ω term). | Design ref |
| `gnss_antenna_bias_supervisor.md` | Active-antenna bias-T, current limiter, open/short supervisor. | Design ref |
| `sts1000_vcc_rb_supply.md` | Digipot-trimmed Rb buck (MIC28516, VCC_RB 5–24 V), bounding resistors, autonomous 26 V OV latch. | Design ref |
| `rb_rs232_interface.md` | FE-5680A serial/control path: SN65C3221E, RS-232/CMOS DPDT relay, opto lock-status. | Design ref |
| `sts1000_fault_aggregation.md` | Direct-GPIO 1 kHz fault/UI scan model (GPIOF HMI/faults, GPIOG PG-rails + INA228 ALERTs). | Design ref |
| `sts1000_3v3p_i2c_peripherals.md` | I²C peripheral group + display/touch subsystem + e-compass; records the 3V3_P-rail elimination. | Design ref |
| `sts1000_net_naming.md` | **Net/rail naming convention** + conventions-in-practice + decision log (plain `3V3`, `_N` active-low, `<rail>_PG`). | Authoritative |
| `sts1000_bom.md` | **BOM standardization rules**, the final current-sense shunt table, and the remaining sourcing/footprint items. CSV: `hardware/ntp-server-stm32/sts1000_bom.csv` — 184 grouped lines / 635 placed parts, every line with a manufacturer **and** DigiKey PN (fields written back into KiCad symbols). | Active |
| `sts1000_firmware_hardware_interface.md` | **Authoritative firmware pin contract.** All 144 U12 pins, bring-up order of operations, timing-engine I/O, I²C map, 1 kHz GPIO scan model, EXTI, monitoring loops, alarms, peripheral methods. | Authoritative (FW) |
| `sts1000_hardware_design_reference.md` | **Master hardware design reference.** Power tree, per-rail derivations, 9× INA228 monitoring architecture, per-subsystem circuit design with formulas. | Authoritative (HW) |
| `sts1000_datasheet_pinout_verification.md` | Per-IC pinout verification vs datasheets, pull-up/level/divider audits, 5 V-tolerance map, connector ESD matrix. | Review |
| `sts1000_pcb_layout_guidance.md` | PCB routing: power-trace/copper sizing, HV clearance, controlled-Z, matched-length, placement sensitivities, RF/shielding, ESD placement, net-class table. | Layout ref |
| `sts1000_bench_tuning_procedures.md` | 43 bench-tune/measure items with procedures + equipment + acceptance criteria; rework prerequisites. | Active |
| `sts1000_schematic_design_review.md` | Netlist-driven design review: connectivity/pinout/termination/ESD/power-tree verification vs datasheets, the current open items, and the decision log for non-obvious choices. | Review |
| `sts1000_layout_readiness_review.md` | **Historical** schematic-to-layout readiness review (multi-agent, 2026-06-21), superseded but retained as a point-in-time record. Its header tracks the disposition of the original 68 blockers: most are resolved (SDA/SCL swap, address collisions, BOOT0, shunt values, cap packages, footprints); **`fp-lib-table` and 36 unassigned footprints remain open**. Do not "fix docs" against this file. | Historical |
| `sts1000_external_wdt.md` | External windowed watchdog U64 TPS3430 → `WDO_N` → POE_KILL; window timing, SET0/SET1 straps, kick cadence. | Design ref |
| `sts1000_poe_kill.md` | Board cold-cycle path: PE15 / WDO_N → NCP1095 AUX, V_KBIAS bias rail, Q3 sustain latch, PSE-driven recovery. | Design ref |
| `sts1000_power_fail_input.md` | PFI early-warning comparator (U2A) off VOUT_P: trip/hysteresis math, hold-up window, EXTI8 handling. | Design ref |
| `sts1000_panel_controls.md` | Front-panel control set: 7 buttons, rotary encoder (TIM1), reed-switch wake, cap-touch INT — electrical + firmware contract. | Design ref |
| `sts1000_panel_switch_inputs.md` | Panel switch input conditioning: pull-ups, debounce caps, scan polarity. | Design ref |
| `sts1000_panel_ui_inputs_esd.md` | ESD strategy for the J17 panel harness (TPD4E02B04 / TPD4E05U06 allocation per line class). | Design ref |
| `sts1000_rgb_indicator.md` | Status RGB D5: common-anode 5 V, low-side NPN per colour, ballast/current table, TIM4 PWM. | Design ref |
| `usb_console_interface.md` | USB-C console port: CC advertisement, VBUS sense divider, ESD, CDC-ACM contract. | Design ref |
| `sts1000_product_page.html` | Customer-facing product/spec page (feature framing, positioning). | Reference |

**Authority rule:** the peripheral map wins on electrical facts; the software spec owns
firmware behavior. Subsystem docs are canonical for their own block's schematic/BOM and
must stay consistent with the two masters.

---

## Core architecture snapshot

- **MCU:** STM32H563ZIT6, Cortex-M33 @ 250 MHz, 2 MB flash, 640 KB SRAM, **LQFP144**,
  −40/+85 °C. All aggregated inputs are direct GPIO on GPIOF/GPIOG (no I/O expanders).
  **There are no spare GPIO.** `PE1` and `PB11` do **not** exist on LQFP144 — ST uses those die pads
  for the two **VCAP** pins on this package (pins 70 and 142); they only appear on LQFP176/UFBGA169.
  The single uncommitted pin is **PH1/OSC_OUT (pin 24)**, free because PH0 runs in **HSE-bypass**
  mode (OSC_IN only), so PH1 is available as a plain GPIO. Full 144-pin contract
  in `sts1000_firmware_hardware_interface.md`.
- **Timing references (both permanent, both always monitored):**
  - OCXO **OH300-61003CV-010.0M** — 10 MHz, always on, steered by DAC1_OUT1 (PA4) →
    loop filter → Vc (center **1.65 V**, never floating). Vc sensed on PA3.
  - External **FE-5680A Rb** — 10 MHz via SMA front end; rail gated by `RB_PWR_EN` (PB7).
  - **Clock mux** (74LVC1G157, `MUX_SEL`=PB6) feeds PH0 (HSE bypass). Glitchless handoff
    is firmware HSI-bridge + STM32 CSS, **not** a glitch-free mux IC.
- **GNSS:** ZED-F9T-00B on USART3 (PD8/9), TIMEPULSE→PA0 (TIM2), TIMEPULSE2→PC6 (TIM3),
  full reset/safeboot/dsel/extint control. V_BCKP held through VCC cycles.
- **Network/PTP:** LAN8742AI RMII PHY (AF11), MAC IEEE-1588 hardware timestamping.
- **Power:** 802.3bt-capable PoE → FDMQ8205A bridge → NCP1095 PD → bucks. Classify
  **Type 2 (at) minimum, Type 3 (bt) for margin** — Rb is permanent and can run
  concurrently with the OCXO (~12–14 W steady both-on, ~25–30 W cold-start peak).
  Supercap backup via TPS61094 ×2 (strapped autonomous).
- **Monitoring:** **9× INA228** (all ADCRANGE=1, shunt values final — table below), 2× TMP117 (U58=0x48 / U57=0x49), ATECC608B (0x60),
  IIS2MDC (0x1E) + LIS2DH12 (**0x19**) e-compass, SHT45 humidity (**0x44**), all on I²C1
  (PB8/PB9) with an LTC4311 rise-time accelerator (EN strapped high). All PG rails, INA228
  ALERTs, buttons, and faults are read by a 1 kHz direct-GPIO scan of GPIOF/GPIOG — no I/O
  expander (see `sts1000_firmware_hardware_interface.md`). All housekeeping peripherals sit
  on **always-on 3V3_STM** (no gated 3V3_P rail).

### INA228 rail map + shunts (as-built, final)
All nine run **ADCRANGE=1** (±40.96 mV shunt FS). Shunts are Vishay Dale **WSL** metal-strip 1 %
AEC-Q200, 1206/0.25 W except R159 (2512/1 W).

| # | Addr | Rail (ref) | Shunt | FS current | CURRENT_LSB |
|---|---|---|---|---|---|
| 1 | 0x40 | PoE input (U10) | **R30 25 mΩ** | ±1.6384 A | 3.125 µA |
| 2 | 0x41 | STM 3V3 (U31) | **R106 100 mΩ** | ±409.6 mA | 781.25 nA |
| 3 | 0x42 | 5V_DISP (U32) | **R107 100 mΩ** | ±409.6 mA | 781.25 nA |
| 4 | 0x43 | main/general 3V3 (U30) | **R102 50 mΩ** | ±819.2 mA | 1.5625 µA |
| 5 | 0x45 | Antenna bias (U26) | **R89 150 mΩ** | ±273.1 mA | 520.833 nA |
| 6 | 0x46 | OCXO (U37) | **R126 25 mΩ** | ±1.6384 A | 3.125 µA |
| 7 | 0x47 | Rb VCC_RB (U44) | **R159 7 mΩ** (2512) | ±5.8514 A | 11.1607 µA |
| 8 | **0x4A** | **GPS VCC (U23)** | **R72 75 mΩ** | ±546.1 mA | 1.04167 µA |
| 9 | **0x4C** | **Panel-LED 5V (U54)** | **R199 150 mΩ** | ±273.1 mA | 520.833 nA |

> **GPS INA228 is 0x4A, not 0x44** — SHT45 (U72) owns 0x44 on the same bus, so re-strapping
> U23 to the historically-documented 0x44 would collide. Firmware must use 0x4A.
>
> **`SHUNT_CAL` = 4096 on all nine.** Setting `Max_Expected_Current` to each device's own full
> scale makes the R term cancel out of the calibration formula, so one constant serves the whole
> board; per-rail resolution comes from `CURRENT_LSB = 78.125 nV / R_SHUNT`. VBUS LSB is
> 195.3125 µV over 0–85 V, so the 54 V PoE bus and the 24 V VCC_RB read **directly, with no
> divider**. Sizing math: `hardware_design_reference §2.1.1`; firmware constants and alert
> thresholds: `firmware_hardware_interface §4.2`. **R16 25 mΩ** (Ohmite MCS1632, 1 W) is the
> NCP1095 hot-swap RSNS — *not* an INA228 shunt.

### Software stack
Zephyr RTOS + MCUboot chained off STM32H5 **STiRoT** secure-boot root; PSA Crypto over
mbedTLS 3.x with **ATECC608B** (CryptoAuthLib) for keys/identity. NTP/NTS server and PTP
grandmaster are **custom application modules** (Zephyr has clients/gPTP but no production
NTS server or turnkey 1588 grandmaster) — this is the largest firmware effort. SNMP agent
also custom. Fallback platform documented: FreeRTOS + lwIP + mbedTLS + MCUboot.

---

## Hard constraints

- **Sourcing policy (China-origin restriction is narrow).** Only two categories are
  banned: (1) IT/networking products with security exposure (network switches, routers,
  networking silicon); (2) software that provides answers or content (LLMs, reference/
  design tools). Ordinary Chinese-origin passives, power ICs, analog parts, and connectors
  are **fully acceptable**. Concrete consequence already in the BOM: SPI-NOR must be
  Macronix/Winbond (TW) or ISSI (US) — **not GigaDevice (PRC)**.
- **Automotive grade:** AEC-Q100/Q101 parts used throughout where available.
- **No DNP / no fit-jumper policy:** all hardware is populated; the MCU adapts paths at
  runtime via GPIO/analog switches. Do not introduce build-option straps.
- **Datasheet-verified parts only.** Exact values with math justification. When ST pin
  data is inaccessible, the AF table is sourced from `modm-io/modm-devices` (develop
  branch, `stm32h5-62_63_73.xml`).

---

## Key design principles & gotchas (do not relearn these the hard way)

- **Back-powering via ESD clamps:** gating a rail while its I²C pull-ups sit on an
  always-on rail back-powers the gated devices through their SDA/SCL clamp diodes. This is
  why all peripherals sit on always-on 3V3_STM. An LTC4311 accelerator does **not**
  isolate and does not solve this.
- **Glitch-free clock mux:** dedicated glitch-free mux ICs complete handoff on the
  outgoing clock's edges and can hang if that source has stopped (the exact Rb-failure
  case). Plain selector + firmware HSI bridge + CSS is more robust.
- **OCXO Vc-path caps:** C0G/NP0 or film only — **never X7R** (piezoelectric microphonics
  FM-modulate the carrier).
- **Ground pours under the magnetometer:** continuous current-free GND pour is correct
  (copper is diamagnetic/magnetically silent); a void routes return current into a loop
  under the sensor. Balanced go+return gives 1/r² falloff vs 1/r single-ended.
- **Direct-GPIO fault scan (rationale):** an MCP23017 expander is the obvious way to aggregate
  fault/UI inputs, but its Rev-D GPA7/GPB7 output-only erratum caps usable inputs and its INT has no
  open-source mode (only a fragile open-drain wire-OR). The LQFP144 instead carries all aggregated
  inputs on a 1 kHz direct-GPIO scan of GPIOF/GPIOG — bus-free, wedge-immune, per-signal identity
  preserved.
- **Source termination placement:** series termination belongs at the driver (e.g.
  R123 at the OCXO output). Remove redundant receiver-end series resistors on the same net.
- **Sense-shunt sizing is a resolution/drop trade, not one rule:** target 50–75 % of the INA228
  ADCRANGE=1 full scale at the rail's *design maximum*. Two rails deliberately sit outside that
  band — VCC_RB (7 mΩ, 36 %) is sized for the **low-voltage** end of its 4.5–24.45 V programmable
  range, not the 15 V nominal; 3V3_GPS (75 mΩ, 24 %) is sized by **insertion drop** because the
  shunt is in series with an LDO feeding a receiver with a 2.7 V floor. Never size a shunt from the
  nominal operating point alone.
- **Kelvin taps matter more at low shunt values:** at a 78.125 nV shunt LSB, **70 µΩ** of tap
  asymmetry is 1 % of R159 (7 mΩ) — less than 0.5 mm of narrow 1 oz copper. The per-board
  `SHUNT_CAL` trim silently absorbs such an error and then lets it drift with temperature.
- **GPS backup sizing:** ZED-F9T has no useful warm-start state beyond ephemeris validity
  (~4 h). Size the supercap to the ephemeris window, not arbitrarily large.
- **ESD on the GPS SMA (RF + bias):** carries 1.5 GHz RF (needs Cj ≤ 0.2–0.3 pF) *and*
  5 V antenna bias (needs V_RWM ≥ ~6.5 V). Use a polymer ESD suppressor + a separate
  coaxial gas-discharge arrestor for outdoor runs; standard clock-line ESD parts fail both.
- **Rb digipot in a buck FB node** can destroy the Rb: hard-bound the range with fixed
  series resistors so no wiper code (POR/mid-scale/SPI-fault) exits the safe envelope; use
  the NV-wiper for safe-low power-up; verify the rail on INA228 0x47 (U44) before trusting the Rb.
  Autonomous 26 V OV latch is the firmware-independent backstop.

---

## Open items before layout / fab

Full evidence in `sts1000_schematic_design_review.md`. Genuinely-open work only:

- [ ] **`fp-lib-table` is missing** from the KiCad project (only `sym-lib-table` exists), so the custom
      footprints in `hardware/libraries/` are unreachable from the board editor. Create it before layout.
- [ ] **Library tables are fixed; footprint *assignment* is the remaining layout task.** `fp-lib-table` now exists (9 nicknames → `hardware/libraries/sts1000.pretty`, 54 footprints) and `sym-lib-table` is portable (`${KIPRJMOD}` relative, 38 entries, all resolving). Of 648 symbols, **41** now carry a resolvable `library:footprint`; **571** still hold a vendor package *description* in the Footprint property (`0402 (1005 Metric)`, `SOT-323`, …) and **36** are empty (J1, J2, J6–J10, J15–J17, K1, K2, L2–L7, U53, U71, Y1, Y2 + mounting holes H1–H14). Two `Library:` refs point at footprints that do not exist in the repo: **J3** `GCT_BG055-06A-1-0450-0530-0350-L-D` and **D5** `LED_IN-P55TATRGB`.
- [ ] **0402 element-voltage / power margin on the PoE dividers.** ERJ-2RK / ERJ-U02 0402 parts are rated
      **50 V, 0.1 W**. `R20` (10 k, VOUT_P→POE_PG) carries **54 V and 292 mW whenever NCP1095 PGO is low**
      — i.e. the whole start-up window and indefinitely if the PSE never delivers; `R2` (1.2 M, PFI
      divider) sees 52–55 V; `R264` (174 k, PG7 divider) sees 51–54 V.
- [ ] **C125 dielectric (optional).** As-built 47 nF **X7R** 50 V feedforward on the 24 V VCC_RB node —
      meets the ≥50 V rule; a C0G part would hold its value under DC bias.

**Retracted:** the long-standing "GPS RF_IN needs a 47 pF series DC-block per the u-blox reference"
open item was **never supported by UBX-21040375** — u-blox injects the antenna bias directly on the RF
trace and relies on the receiver's internal DC block. The part briefly added for it (`C204`) is
**deleted**; the as-built `node A → L1 → GPS_RF_IN` matches the reference. Do not re-open it.
See `sts1000_schematic_design_review.md §1.1`.

**Closed since the last revision** (so stale references are recognizable): **R77** re-sized to a
**0.25 W** part (109 mW at the antenna foldback clamp = 44 % of rating); all nine **INA228 shunt
values** are final and sized; **C98/C99/C101** → 0.1 µF C0G **1210**; **C1/C42/C186** → 4.7 nF 2 kV
**1812**; **C37** → 0.1 µF C0G 1210; **C125–C128** → all 50 V; **R112/R113** → 13.0 k (VCHG 2.7 V);
**R266** 3V0_RF power-good pull-up fitted (PG2 needs no firmware masking); **L7** → 39 µH
(7447709390); **PHY straps R49/R228/R229/R230** → 4.99 k E96; PoE HV caps C6–C11 sourced at 100 V;
footprint *fields* populated board-wide — but with vendor package text, not land-pattern links (above).

**Bench verifications (before fab):**
- [ ] **AP3441 PG active-drive (do early — high consequence):** DS39754 says PG pulls to VIN (=5 V) when
      good but specs **no PG drive impedance**. Divider ratios are correct (all driven inputs get valid
      levels), but the OCXO-chain nodes were mis-documented: node N (`OCXO_PSU_PG`, which enables OCXO LDO
      U39 EN) is loaded by **two** parallel legs (R124 and R232+R233), so node N ≈ **2.98 V** (not 3.68 V)
      and PG5 ≈ **2.66 V** (not 3.28 V) — both still valid (U39 EN VIH 1.1 V; PG5 > VIH 2.31 V). PG4 =
      2.98 V. Scope node N / PG5 / PG4 with the 5 V rail up; if PG5 sags near VIH raise R233. See
      `hardware_design_reference §2.3` ⁷.
- [ ] MCP41U83 (U43): code 0 = Terminal B = **safe-low** (VOUT min 4.51 V); POR = midscale (~14.5 V,
      within the 26 V OV); SPI = **Mode 0,0**. Firmware pre-programs the NV wiper safe-low + verifies
      VCC_RB on INA228 0x47.
- [ ] Supercap DSF305Q3R0 (3.0 V to 65 °C / 2.5 V at 85 °C): enclosure Tmax = **125 °F (51.7 °C)** is
      below the 65 °C corner → full 3.0 V rating; VCHG termination set to **2.7 V** (R112/R113 = 13.0k,
      highest TPS61094 option under the cap, ~1.5× backup energy vs 2.2 V). Confirm BKP_STM_PG/BKP_GPS_PG
      thresholds. (Was 2.2 V/85 °C; also fixed a stale sch-4.75k/BOM-6.65k mismatch.)
- [ ] ZED-F9T V_BCKP current at max enclosure temperature (datasheet only specs 45 µA @ 25 °C).
- [ ] FE-5680A J6.8/J6.9 Tx/Rx direction + full connector pinout for the specific surplus variant.
- [x] NCP1095 NCM/NCL/LCF (PC2/PC7/PC3): **resolved** — open-drain, RTN-referenced (U9.12 = GND),
      +72 V abs-max; safe direct to 3.3 V GPIO but **float without a pull-up**. Add pull-up to 3V3_STM
      (3× external 10 kΩ, or STM32 internal pull-ups on PC2/PC3/PC7). Non-isolated PD → no isolator.
- [ ] Per-board INA228 `SHUNT_CAL` trim (×9) against a reference load — removes the 1 % shunt
      tolerance, which dominates the uncalibrated error budget. Values themselves are final.
- [ ] RT9742 (U33) inrush vs I_LIM, and whether that limit exceeds the U32 **±409.6 mA** full scale
      across R107 100 mΩ (if so, a display short clips the current reading — annunciation still valid
      via the INA alert + nFLG pair). Measure module backlight-full current + Cin.

See `peripheral_map §14` and `software_spec §15` for the complete open-items lists.

---

## How to work in this repo
- **Style:** terse, dense, conclusions first, table-driven. No filler, no design-history
  framing in docs.
- **Schematic reviews:** reference components by **designator and pin** (R200, U55 GPB2,
  PA10), not node labels. Flag issues against the relevant datasheet; re-verify rather
  than defend a first claim if corrected.
- **Edits to docs:** keep them as unified authoritative references and propagate every
  decision across all affected docs in the same change. Track open work as checklists with
  explicit owner/action.
- **Firmware:** Zephyr devicetree should express the pin map 1:1; net names in code/DT
  match the canonical net names in the peripheral map.

> Optional: as `firmware/` and `hardware/` grow, add a scoped `CLAUDE.md` in each
> (Claude Code auto-loads nested ones) — e.g. build/flash commands and the Zephyr board
> overlay in `firmware/`, KiCad version + library/3D-model paths in `hardware/`.

# Claude Code operating model

## Model responsibility map

The main session is the technical lead, architect, orchestrator, and final reviewer. It must remain on Fable unless the user explicitly selects another model. Workers run on cheaper models sized to their task:

| Responsibility                    | Agent                  | Model     | Reason                                                        |
| --------------------------------- | ---------------------- | --------- | ------------------------------------------------------------- |
| Architecture and orchestration    | main session           | Fable 5   | Highest-value reasoning and sustained task management          |
| Final acceptance review           | main session           | Fable 5   | Independently validates worker output                          |
| Implementation and review fixes   | `implementer`          | Opus 5, max effort | Strong coding and agentic execution at lower cost than Fable |
| First-pass code review            | `code-reviewer`        | Opus 5, xhigh effort | Strong enough to catch implementation defects independently |
| Adversarial second-pass review    | `adversarial-reviewer` | Opus 5, max effort | Attempts to refute the implementation and first-pass approval |
| AWS deployment execution          | `aws-deployer`         | Opus 5, high effort | High-stakes tool use requiring strong judgment          |
| Repository exploration            | `explore`              | Sonnet 5  | Good search and comprehension at lower cost                    |
| Tests and failure triage          | `test-runner`          | Sonnet 5  | Mostly tool use and bounded diagnosis                          |
| Documentation and mechanical work | `documentation-writer` | Haiku     | Low reasoning requirement and lowest cost                      |

Do not use the main Fable context for lengthy mechanical implementation, bulk file reading, or test execution that an appropriate worker can perform. Reserve Fable turns for investigation synthesis, planning, arbitration of conflicting worker reports, and final review.

## Standard feature workflow

For non-trivial implementation work:

1. Investigate the repository and requirements (delegate searches to `explore`).
2. Establish explicit acceptance criteria and state them to the user.
3. Produce a technically coherent implementation plan.
4. Delegate the implementation to `implementer` with the plan, acceptance criteria, and file/subsystem boundaries.
5. Delegate the first-pass review to `code-reviewer`, giving it the plan, the acceptance criteria, and the explicit file list or feature boundary under review (much of this project's work is uncommitted, so a bare `git diff` is not the review unit). Run `test-runner` in parallel when the review and test scopes do not overlap in interpretation.
6. Send BLOCKER, HIGH, and justified MEDIUM findings back to `implementer`; require a disposition for every finding. Repeat implementation and first-pass review until no blocking findings remain.
7. Delegate the adversarial second pass to `adversarial-reviewer`, giving it the plan, the implementation report, the first-pass verdict, and the same file list. Route its output as follows: CONFIRMED blocking findings go back to `implementer`; SUSPECTED blocking findings are arbitrated by the main agent, which must disprove them itself, send them to `implementer`, or accept the risk with a recorded rationale — never drop one silently. On a WOUNDED verdict, the main agent decides which non-blocking findings are fixed now versus recorded as known risks in the completion report. After fixes, re-run `adversarial-reviewer` on the changed code until it returns SURVIVED or only findings the main agent has adjudicated remain.
8. Fable final acceptance review — see below. Findings from this review also go back to `implementer`, never fixed inline by the main session unless trivial.
9. Confirm tests and acceptance criteria (via `test-runner` output, not worker assertions).
10. Report completion, remaining risks, and important decisions to the user.

Skip steps proportionally for trivial work (a typo fix does not need an adversarial review), but never skip step 8: the main agent always inspects the diff before declaring success.

Loop discipline — the review loops must terminate:

- "Blocking" means BLOCKER and HIGH findings, plus MEDIUM findings the main agent explicitly endorses.
- Each review stage gets at most two fix/re-review cycles. If findings remain disputed after that, the main agent adjudicates each one directly and its ruling is final.
- When an `implementer` disputes a finding, the main agent — not another review round — resolves the dispute, by inspecting the code itself.
- On every re-review, hand the reviewer the prior findings with their dispositions and instruct it not to re-raise adjudicated items. Rulings by the main agent bind all subsequent review passes.
- "Read-only" agents are read-only by instruction, not by technical enforcement (they have Bash); the main agent should treat an unexplained tree mutation reported by a reviewer as an incident, not noise.

## Final-review responsibilities

The main agent must not merely repeat a worker's report. Before declaring success:

- Inspect the actual diff.
- Verify that the implementation matches the approved intent.
- Check the most consequential files directly.
- Assess reviewer findings independently — reviewers can be wrong in both directions.
- Confirm that relevant tests were actually run, from command output, not from a worker's summary.
- Identify any residual risk or unverified assumption.

The main agent has final authority to approve, reject, simplify, or redirect worker output.

## AWS deployment workflow

Deployment is never part of the feature workflow. It happens only when the user explicitly requests a deployment in their own words.

1. Confirm with the user (or from their explicit instruction): target environment, AWS account/profile, region, what is being deployed, whether rollback on failure is authorized, and any anticipated resource replacements or deletions. Never infer "production" — authorization for environment, destruction, and rollback comes from the user's words, not the orchestrator's interpretation.
2. Ensure the code being deployed has passed the standard feature workflow, or tell the user what review it has skipped. Record the reviewed commit SHA.
3. Delegate to `aws-deployer` with the user's deployment request quoted verbatim, the environment, account ID, profile, region, stack/service scope, expected changes, the reviewed commit SHA, and whether rollback is pre-authorized on failure. The deployer will abort if the verbatim quote does not name the environment.
4. The deployer verifies identity, previews changes, applies, waits for a terminal state, and verifies health. Treat a deploy without passing verification as a failed deploy.
5. Relay to the user: what changed, verification results, environment state, and any manual follow-ups or cost-relevant resources.

Infrastructure-as-code changes (templates, CDK, Terraform, pipeline config) are ordinary implementation work and go through the standard feature workflow; only their execution against AWS goes through `aws-deployer`.

## Delegation constraints

- Give every worker a bounded assignment and explicit acceptance criteria.
- Tell workers which files or subsystem they own when parallel work is used.
- Avoid having multiple implementation agents edit overlapping files concurrently; when overlap is unavoidable, serialize or use worktree isolation.
- Prefer read-only agents (`explore`, `code-reviewer`, `adversarial-reviewer`, `test-runner`) for exploration and review.
- Reviewers must receive the plan and acceptance criteria, not just the diff, so they can judge compliance rather than style.
- Never allow a worker report to substitute for evidence from the repository.
- Do not commit, push, publish, deploy, or open a pull request unless the user explicitly requests that action.
