# STS1000 "Meridian" — Project Context for Claude Code

GPS-disciplined Stratum-1 NTP/NTS/PTP grandmaster clock. Single-board, PoE-powered,
built around an **STM32H563VIT6** (LQFP100) and a **u-blox ZED-F9T-00B** GNSS timing
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
| `sts1000_fault_aggregation.md` | MCP23017 ×2 fault/UI aggregation, three active-low wire-OR interrupts. As-built. | As-built |
| `sts1000_3v3p_i2c_peripherals.md` | I²C peripheral group + display/touch subsystem + e-compass; records the 3V3_P-rail elimination. | Design ref |
| `sts1000_net_naming.md` | **Net/rail naming convention** + the applied rename map (plain `3V3`, `_N` active-low, `<rail>_PG`). | Authoritative |
| `sts1000_bom.md` | **BOM standardization rules** + the lines still needing a sourcing decision. CSV: `hardware/ntp-server-stm32/sts1000_bom.csv`. | Active |
| `sts1000_schematic_error_review.md` | Netlist-driven error/consistency review (wiring defects, ESD-scheme gaps, doc-vs-as-built deltas, reliability items). | Review |
| `sts1000_product_page.html` | Customer-facing product/spec page (feature framing, positioning). | Reference |

**Authority rule:** the peripheral map wins on electrical facts; the software spec owns
firmware behavior. Subsystem docs are canonical for their own block's schematic/BOM and
must stay consistent with the two masters.

---

## Core architecture snapshot

- **MCU:** STM32H563VIT6, Cortex-M33 @ 250 MHz, 2 MB flash, 640 KB SRAM, LQFP100,
  −40/+85 °C. Package note: **PE1 and PB11 are not bonded — do not use.** Board is
  GPIO-full; no uncommitted pins remain (`peripheral_map §13`).
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
- **Monitoring:** 8× INA228 (0x40–0x47), 2× TMP117 (0x48/0x49), ATECC608B (0x60),
  IIS2MDC (0x1E) + LIS2DH12 (0x18/0x19) e-compass, 2× MCP23017 (0x20/0x21), all on I²C1
  (PB8/PB9) with an LTC4311 rise-time accelerator. All housekeeping peripherals are on
  **always-on 3V3_STM** (the gated 3V3_P rail was eliminated).

### INA228 rail map
| # | Addr | Rail | # | Addr | Rail |
|---|---|---|---|---|---|
| 1 | 0x40 | PoE input | 5 | 0x46 | OCXO |
| 2 | 0x41 | STM 3V3 | 6 | 0x47 | Rb (VCC_RB) |
| 3 | 0x44 | GPS VCC | 7 | 0x42 | 5V_DISP |
| 4 | 0x45 | Antenna bias | 8 | 0x43 | main/general 3V3 |

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
  always-on rail back-powers the gated devices through their SDA/SCL clamp diodes. Fixed
  here by moving all peripherals to always-on 3V3_STM. An LTC4311 accelerator does **not**
  isolate and does not solve this.
- **Glitch-free clock mux:** dedicated glitch-free mux ICs complete handoff on the
  outgoing clock's edges and can hang if that source has stopped (the exact Rb-failure
  case). Plain selector + firmware HSI bridge + CSS is more robust.
- **OCXO Vc-path caps:** C0G/NP0 or film only — **never X7R** (piezoelectric microphonics
  FM-modulate the carrier).
- **Ground pours under the magnetometer:** continuous current-free GND pour is correct
  (copper is diamagnetic/magnetically silent); a void routes return current into a loop
  under the sensor. Balanced go+return gives 1/r² falloff vs 1/r single-ended.
- **MCP23017 INT:** no open-source mode exists; use open-drain active-low wire-OR
  (active-high push-pull with pull-down does not work).
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

## Standing open items

**KiCad actions (verify done):**
- [ ] Drop legacy **R44** at PC12 (R199 is the `INA_ALERT_INT_N` pull-up; else 10k∥10k = 5k). <!-- TODO verify against re-annotated schematic -->
- [ ] Fix net label **`PG_IN_N` → `PG_INT_N`** (one-pin-net otherwise).
- [ ] Confirm INT nets + `EXP_RST_N` land on the correct MCU pins (PA10 / PE0 / PC12 / PC0).
- [ ] Cross-doc: peripheral-map §2 wording "OCXO → PH0 directly" → "→ PH0 via §2.1 clock mux".

**Design verifications before layout / fab:**
- [ ] MCP41U83T-503E/ST: pin numbers, A0/A1 tie in SPI mode, wiper-write opcode/frame,
      CRC default, SPI CPOL/CPHA, power-on wiper value.
- [ ] Commit LIS2DH12 **SA0 address** (0x18 vs 0x19); confirm consumer −40/+85 °C rating
      is adequate for the enclosure thermal environment.
- [ ] Confirm PIR (Panasonic PaPIRs EKMB) orderable PN + output structure (rail = 3V3_STM).
- [ ] ZED-F9T V_BCKP current at max enclosure temperature (datasheet only specs 45 µA @ 25 °C).
- [ ] DSF305Q3R0 supercap placement keeps it ≤ ~75 °C (2.5 V/85 °C derating at 2.55 V peak).
- [ ] FE-5680A J1-8/J1-9 Tx/Rx direction (FE manual §2-3.1 label convention is ambiguous);
      confirm full connector pinout for the specific surplus variant before layout.
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
