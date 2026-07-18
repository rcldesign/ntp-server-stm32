# STS1000 — Schematic Design Review & Open Items

A datasheet-grounded review of the STS1000 schematic: connectivity (from the KiCad netlist,
`ref.pin → net`), pinouts, terminations, signal levels, ESD, and the power tree verified against
manufacturer datasheets. This document states the **current open items** and the **design
decisions** behind the non-obvious choices; the verified-correct portions are described in the
subsystem docs.

---

## 1. Open items

Grouped by what closes them. None are blocking for board bring-up except where noted; the panel
power, PoE power-good, current-monitoring, transistor orientations, and pull-up terminations are all
verified correct as-built.

### 1.1 Design change to implement
| Item | Detail | Action |
|---|---|---|
| **GPS RF_IN series DC-block** | The u-blox ZED-F9T reference antenna-bias design (Integration Manual UBX-21040375) places a **47 pF C0G series DC-block between the bias-T node and RF_IN**, so the 5 V antenna bias reaches the antenna only. As-built, `GPS_RF_IN` ties the 5 V-biased node directly to U21.2. | Add a ~47 pF C0G series DC-block between the antenna/bias node and U21.2, or obtain u-blox confirmation the internal RF_IN block tolerates continuous 5 V. Supervisor/current-sense (U24/U26) are on the V_ANT side, unaffected. |

### 1.2 Part sourcing (values fixed; orderable parts to be selected — flagged in BOM descriptions)
| Refs | Requirement |
|---|---|
| C126 / C127 / C128 (47 µF), C125 (47 nF) | ≥ 50 V — Rb buck output on VCC_RB (24.45 V pedestal, 26 V OV). 47 µF 50 V will not fit 1210 → 1210/1812 or split. |
| C98 / C99 / C101 (0.1 µF) | C0G/NP0 or PPS film, **1210 or film** package (0.1 µF C0G does not exist in 0402). OCXO Vc path — no class-II (X7R) substitution. |
| C1 / C42 / C186 (4.7 nF 2 kV) | 1808/1812 safety/AEC-Q200 Y-cap (2 kV does not exist below 1808). Chassis-shield surge bridge. |

### 1.3 Optional improvement
| Item | Detail |
|---|---|
| **C37 (VREF+) 1 nF → 100 nF** | VREF+ feeds the OCXO-steering DAC/ADC; 100 nF at the pin (keeping R48/C38 as the reference-noise filter) improves ADC ENOB per ST AN5711. Non-blocking. |

### 1.4 Bench verification / per-unit
1. **AP3441 PG dividers — logic levels confirmed; absolute levels bench-only.** Both AP3441 bucks
   (U29 3V3, U38 OCXO pre-reg) run off the **5 V** rail, and their PG pins pull to VIN = 5 V when in
   regulation, scaled by dividers to GND (no external pull-up). All **driven inputs receive valid
   levels**, but the OCXO-chain node voltages were previously mis-documented:
   - **U29 → PG4:** R94 6.81k / R92 10k, PG4 the only load → **2.98 V** (> STM32 VIH 2.31 V, < 3.6 V). ✓
   - **U38 → node N (`OCXO_PSU_PG`):** R125 3.57k top, but node N is loaded by **both** R124 10k→GND
     **and** the series R232 1.21k+R233 10k→GND, so node N = **2.98 V** (not the "3.68 V" that ignored
     the second leg). Node N drives **U39 (TPS7A52) EN** — VIH(max) 1.1 V, abs-max 7 V → reliably
     enables the OCXO LDO. ✓
   - **U38 → PG5:** node N · R233/(R232+R233) = **2.66 V** (not "3.28 V"); > STM32 VIH 2.31 V
     (margin 0.35 V), < 3.6 V. ✓
   - **Contingency:** AP3441 DS39754 gives the PG pin **no drive-impedance spec**, so all three
     absolute levels depend on the internal pull-up under the ~0.57 mA divider load. **Scope node N and
     PG5 (and PG4) with the 5 V rail up.** If PG5 sags toward VIH, raise R233 (e.g. 22k) to pull PG5
     up toward node N (2.98 V). Divider **ratios** are otherwise correct.
2. **ZED-F9T V_BCKP current at enclosure Tmax** — datasheet specifies only 45 µA @ 25 °C; measure to
   size the supercap ephemeris-hold window.
3. **FE-5680A serial direction** — J6.8/J6.9 Tx/Rx convention varies by surplus variant; confirm per
   unit. VCC_RB operating point is set per unit (the variable supply also services other external Rb
   references, so the 24.45 V pedestal is by design, not bounded to one FE's Vmax).
4. **NCP1095 NCM/NCL/LCF — resolved (pull-up action required).** All four status pins (LCF, PGO, NCM,
   NCL) are **open-drain, referenced to RTN** = U9.12 → **GND** (board ground), abs-max **+72 V** to
   RTN. NCM/NCL (Class MSB/LSB) → PC2/PC7, LCF → PC3, each **direct to the MCU with no divider** —
   correct and **safe on a 3.3 V GPIO** (GND-referenced open-drain; it can only sink to GND, so nothing
   drives 54 V onto the pin — unlike PGO, whose VPP-side pull-up requires the R264/R263 divider). But
   they **float without a pull-up**, and as-built POE_NCM/POE_NCL/POE_LCF have none. **Action:** add a
   pull-up to 3V3_STM — 3× external **10 kΩ** (on-pattern with the INA228 ALERT pull-ups) or the STM32
   **internal pull-ups** on PC2/PC3/PC7 (zero-BOM; fine for these slow latched status lines). VOL ≤
   0.5 V @ 2 mA → either reads valid. *(No digital isolator needed: the PD front end is non-isolated,
   RTN = board GND.)*
5. **Per-rail INA228 shunt final values** — size to each rail's expected full-scale current.

---

## 2. Verification coverage (confirmed correct)

| Area | Result |
|---|---|
| **Pinouts** | All ICs verified against datasheets (STM32H563 DS14258, INA228, NCP1095, MIC28516, AP3441, TPS61094, LT3045, LAN8742, ZED-F9T, MCP41U83, SI7469DP, BC857W, BZX84C12, LTC6752, PCA9306, …). |
| **PoE power-good** | NCP1095 PGO (open-drain to RTN) → R20 → R264 174 k / R263 10 k → PG7 = 2.93 V; PG3/PG7 are 5 V-tolerant (FT). |
| **Current monitoring** | 9× INA228; PoE shunt R30 in the series V_POE feed; all ALERT pins have 10 k pull-ups. |
| **Power sequencing** | 5V buck PG (open-drain, R95→5V) gates the downstream AP3441 bucks and LDOs; AP3441 PG (→VIN) divided to the MCU. |
| **Transistor/diode orientation** | POE_KILL Q3, GPS ANT_OFF Q13 (BC857W high-side PNP, E on the high rail); D25 gate clamp (cathode→VCC_RB). |
| **Pull-ups / open-drain** | All PG / ALERT / nFLG / reset nets terminated (incl. `INA_ALERT_VCC_RB` R265, `3V0_RF_LDO_PG` R266). |
| **Signal levels / dividers** | USB_VBUS_SENSE 2.87 V, RB_OV_DET 3.24 V, PG5 2.66 V (loaded OCXO PG divider — §1.4-1), PG4 2.98 V, antenna DETECT/SHORT thresholds — all in range. |
| **5 V tolerance** | Encoder buffered by U68 (5 V-safe); display via PCA9306; PG3/PG7 FT. No 3.3 V-only pin sees 5 V. |
| **ESD** | Ribbon/panel → TPD4E0x; RF/clock coax → SZESD7410 / PGB1010603MR; power/shield → SMF5V0A / 1.5SMCJ28A / 2 kV Y-cap; RS-232/DE-9 → SMAJ15CA + D26. |
| **I²C addressing** | 15 devices, no collisions (GPS INA 0x4A; SHT45 0x44; TMP117 0x49/0x48; LIS2DH12 0x19). |
| **Front-panel power** | J17.16 → 5V_DISP (display); J17.28 → 5 V (encoder/panel logic); reed switch passive (no Vcc). |

---

## 3. Decision log

Rationale for the non-obvious choices, so a future reviewer does not re-open them.

- **LQFP144 + direct-GPIO input scan** — the MCP23017 Rev-D GPA7/GPB7 output-only erratum capped
  usable expander inputs, and its INT has no open-source mode (only a fragile wire-OR); moving all
  aggregated inputs to direct GPIO on Ports F/G (1 kHz scan) removes the I²C-wedge exposure. The
  144-pin package supplies the I/O.
- **GPS INA228 at 0x4A** — SHT45 (U72) occupies 0x44 on the same I²C1 bus; 0x4A avoids the collision.
- **OCXO Vc caps C0G/NP0 or film** — X7R piezoelectric microphonics FM-modulate the 10 MHz carrier;
  the loop-filter/Vc-pin/1.65 V-ref caps must not be class-II.
- **VCC_RB hard-bounded by fixed resistors** — no digipot wiper code (POR / mid-scale / SPI-fault) may
  exit the Rb-safe envelope; the digipot only trims within the bound, and the 26 V OV latch is the
  firmware-independent backstop. The 24.45 V pedestal is intentional (the supply also serves other
  external Rb references; the operating point is set per unit).
- **PG7 divider (R264 174 k / R263 10 k)** — NCP1095 PGO releases to the 54 V VPP rail when power-good;
  the divider brings the power-good level into the MCU input range (and PG7 is FT regardless).
- **U10 shunt in the series V_POE feed** — the shunt must carry the load current to measure PoE input.
- **BC857W high-side PNP orientation (Q3, Q13)** — emitter on the high rail, collector driving the
  load node, so pulling the base low turns the switch on.
- **D25 gate-clamp orientation (cathode → VCC_RB)** — Zener-clamps the P-FET gate to keep |Vgs| within
  the SI7469DP ±20 V limit at the 24 V rail.
- **Clock mux = plain 74LVC1G157 + firmware HSI bridge + CSS** — a dedicated glitch-free mux completes
  handoff on the outgoing clock's edges and can hang if that source has stopped (the exact Rb-failure
  case); a plain selector plus the firmware bridge is more robust.
- **Supercap VCHG = 2.7 V** (VCHG R112/R113 = 13.0k 1%) — enclosure Tmax = 125 °F (51.7 °C) is below
  the DSF305Q3R0 65 °C corner, so its full 3.0 V rating applies; 2.7 V is the highest TPS61094 table
  option under that cap (next step is 3.6 V) and stores ~1.5× the backup energy of the former 2.2 V.
  Supersedes the 2.2 V/85 °C assumption; also resolves the stale schematic-4.75k vs BOM-6.65k conflict.
- **All housekeeping peripherals on always-on 3V3_STM** — gating a rail while its I²C pull-ups sit on
  an always-on rail back-powers the parts through their SDA/SCL clamp diodes.
- **GPS SMA ESD = polymer suppressor (+ external GDT for outdoor runs)** — the line carries 1.5 GHz RF
  (Cj ≤ ~0.3 pF) and 5 V bias (V_RWM ≥ ~6.5 V); ordinary clock-line ESD parts meet neither.
- **L1 = 47 nH bias-T choke** — passes the ≤ 182 mA foldback-limited bias while presenting high
  impedance at L-band; the design relies on the active foldback limiter rather than the u-blox
  reference's fixed series resistor for short protection.
