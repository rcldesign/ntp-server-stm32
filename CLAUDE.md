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
| `sts1000_bom.md` | **BOM standardization rules** + the lines still needing a sourcing decision. CSV: `hardware/ntp-server-stm32/sts1000_bom_v2.csv` (fields written back into KiCad symbols). | Active |
| `sts1000_firmware_hardware_interface.md` | **Authoritative firmware pin contract.** All 144 U12 pins, bring-up order of operations, timing-engine I/O, I²C map, 1 kHz GPIO scan model, EXTI, monitoring loops, alarms, peripheral methods. | Authoritative (FW) |
| `sts1000_hardware_design_reference.md` | **Master hardware design reference.** Power tree, per-rail derivations, 9× INA228 monitoring architecture, per-subsystem circuit design with formulas. | Authoritative (HW) |
| `sts1000_datasheet_pinout_verification.md` | Per-IC pinout verification vs datasheets, pull-up/level/divider audits, 5 V-tolerance map, connector ESD matrix. | Review |
| `sts1000_pcb_layout_guidance.md` | PCB routing: power-trace/copper sizing, HV clearance, controlled-Z, matched-length, placement sensitivities, RF/shielding, ESD placement, net-class table. | Layout ref |
| `sts1000_bench_tuning_procedures.md` | 43 bench-tune/measure items with procedures + equipment + acceptance criteria; rework prerequisites. | Active |
| `sts1000_schematic_design_review.md` | Netlist-driven design review: connectivity/pinout/termination/ESD/power-tree verification vs datasheets, the current open items, and the decision log for non-obvious choices. | Review |
| `sts1000_layout_readiness_review.md` | **Schematic-to-layout readiness review** (multi-agent, 2026-06-21). 15-section: IC-by-IC datasheet/footprint verification, netlist defects, power tree, interfaces, net-class/constraint package. Verdict: **NEEDS CORRECTIONS BEFORE LAYOUT** — 68 blockers (incl. board-wide INA228 SDA/SCL swap, missing `fp-lib-table`, many unassigned footprints, BOOT0 1:1 divider). | Review |
| `sts1000_product_page.html` | Customer-facing product/spec page (feature framing, positioning). | Reference |

**Authority rule:** the peripheral map wins on electrical facts; the software spec owns
firmware behavior. Subsystem docs are canonical for their own block's schematic/BOM and
must stay consistent with the two masters.

---

## Core architecture snapshot

- **MCU:** STM32H563ZIT6, Cortex-M33 @ 250 MHz, 2 MB flash, 640 KB SRAM, **LQFP144**,
  −40/+85 °C. All aggregated inputs are direct GPIO on GPIOF/GPIOG (no I/O expanders).
  **PE1 and PB11 are bonded but unrouted — the only uncommitted GPIO.** Full 144-pin contract
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
- **Monitoring:** **9× INA228**, 2× TMP117 (U58=0x48 / U57=0x49), ATECC608B (0x60),
  IIS2MDC (0x1E) + LIS2DH12 (**0x19**) e-compass, SHT45 humidity (**0x44**), all on I²C1
  (PB8/PB9) with an LTC4311 rise-time accelerator (EN strapped high). All PG rails, INA228
  ALERTs, buttons, and faults are read by a 1 kHz direct-GPIO scan of GPIOF/GPIOG — no I/O
  expander (see `sts1000_firmware_hardware_interface.md`). All housekeeping peripherals sit
  on **always-on 3V3_STM** (no gated 3V3_P rail).

### INA228 rail map (as-built, address-strap-decoded)
| # | Addr | Rail (ref) | # | Addr | Rail (ref) |
|---|---|---|---|---|---|
| 1 | 0x40 | PoE input (U10) | 6 | 0x47 | Rb VCC_RB (U44) |
| 2 | 0x41 | STM 3V3 (U31) | 7 | **0x4A** | **GPS VCC (U23)** |
| 3 | 0x42 | 5V_DISP (U32) | 8 | 0x45 | Antenna bias (U26) |
| 4 | 0x43 | main/general 3V3 (U30) | 9 | **0x4C** | **Panel-LED 5V (U54)** |
| 5 | 0x46 | OCXO (U37) | | | |

> **GPS INA228 is 0x4A, not 0x44** — SHT45 (U72) owns 0x44 on the same bus, so re-strapping
> U23 to the historically-documented 0x44 would collide. Firmware must use 0x4A.

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
- **GPS backup sizing:** ZED-F9T has no useful warm-start state beyond ephemeris validity
  (~4 h). Size the supercap to the ephemeris window, not arbitrarily large.
- **ESD on the GPS SMA (RF + bias):** carries 1.5 GHz RF (needs Cj ≤ 0.2–0.3 pF) *and*
  5 V antenna bias (needs V_RWM ≥ ~6.5 V). Use a polymer ESD suppressor + a separate
  coaxial gas-discharge arrestor for outdoor runs; standard clock-line ESD parts fail both.
- **Rb digipot in a buck FB node** can destroy the Rb: hard-bound the range with fixed
  series resistors so no wiper code (POR/mid-scale/SPI-fault) exits the safe envelope; use
  the NV-wiper for safe-low power-up; verify the rail on INA228 #6 before trusting the Rb.
  Autonomous 26 V OV latch is the firmware-independent backstop.

---

## Open items before layout / fab

Full evidence in `sts1000_schematic_design_review.md`. Genuinely-open work only:

- [ ] **GPS RF_IN series DC-block.** The u-blox ZED-F9T reference antenna-bias design (Integration
      Manual UBX-21040375) places a 47 pF C0G series DC-block between the bias-T node and RF_IN. As-built
      `GPS_RF_IN` ties the 5 V-biased node directly to U21.2. Add a ~47 pF C0G series DC-block (so 5 V bias
      reaches the antenna only), or confirm the ZED-F9T internal block tolerates continuous 5 V.
- [ ] **Rb buck output caps C125/C126/C127/C128 ≥ 50 V** — they sit on VCC_RB (24.45 V pedestal / 26 V
      OV); source ≥ 50 V parts (47 µF 50 V won't fit 1210 → 1210/1812 or split; C0G for C125).
- [ ] **Cap packages:** `C98/C99/C101` 0.1 µF C0G → 1210 C0G / PPS film; `C1/C42/C186` 4.7 nF 2 kV →
      1808/1812 Y-cap.
- [ ] **`C37` (VREF+) 1 nF → 100 nF** (optional ADC-ENOB improvement for OCXO steering).

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
- [ ] Per-rail INA228 shunt final values.
- [ ] RT9742 inrush vs I_LIM + 0.1 Ω shunt headroom (measure module backlight-full current + Cin).

See `peripheral_map §14` and `software_spec §15` for the complete open-items lists.

---

## How to work in this repo

- **Editor:** use **vi** for any CLI text editing — never pico/nano.
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
