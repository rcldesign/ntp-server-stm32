# STS1000 Hardware — Claude Code Context (`hardware/`)

Scoped context for the KiCad project under `hardware/`. The root `../CLAUDE.md` carries the
project-wide rules. `../docs/ntp_server_peripheral_map.md` is the **authoritative pin/net/
power reference**; `../docs/sts1000_schematic_implementation_checklist.md` is the
**block-by-block IC inventory and the best entry point for a schematic/BOM review**. Each
subsystem doc is canonical for its own block's schematic and BOM. This file is the working
preamble, not a re-statement of those docs.

---

## Tooling & project hygiene

- KiCad project (schematic + layout + BOM). **Record and pin the KiCad version here** once
  confirmed, plus the symbol/footprint/3D-model library paths so reviews are reproducible.
- Keep `sym-lib-table` / `fp-lib-table` pointing at in-repo libraries where parts are custom;
  note any external library dependencies.
- **Net names must match the canonical names** in the peripheral map (`OSC_IN`, `MUX_SEL`,
  `RB_PWR_EN`, `INA_ALERT_VCC_RB`, `PROX_WAKE`, `PANEL_LED_EN`, `5V_DISP`, `VCC_RB`, …). A
  schematic net that drifts from the doc name is a defect. **There are no I/O expanders** — all
  INA228 ALERTs, PG rails, buttons, encoder, touch and presence are **direct MCU GPIO** (Ports F/G);
  there are no `_INT_N` expander-interrupt or `EXP_RST_N` nets.

---

## Open KiCad / BOM actions before layout

Genuinely-open work; full evidence in `../docs/sts1000_schematic_design_review.md`. Everything else in
the schematic is verified as-built.

- [ ] **GPS RF_IN series DC-block.** Add a ~47 pF C0G series DC-block between the antenna/bias node and
      U21.2 (the u-blox ZED-F9T reference antenna-bias design places one there so 5 V bias reaches the
      antenna only), or confirm the ZED-F9T internal RF_IN block tolerates continuous 5 V.
- [ ] **Rb buck output caps** C125/C126/C127/C128 → ≥ 50 V (they sit on VCC_RB, 24.45 V pedestal /
      26 V OV); 47 µF 50 V won't fit 1210 → 1210/1812 or split; C0G for C125.
- [ ] **Cap packages:** C98/C99/C101 (0.1 µF C0G → 1210 C0G / PPS film), C1/C42/C186 (4.7 nF 2 kV →
      1808/1812 Y-cap).
- [ ] **C37 VREF+ 1 nF → 100 nF** (optional ADC-ENOB improvement for OCXO steering).

---

## Subsystem → doc → key designators

| Block | Canonical doc | Anchor designators |
|---|---|---|
| MCU & core | peripheral_map | **U12** STM32H563ZIT6 (LQFP144), **U64** TPS3430 WDT |
| OCXO + supply | checklist §2 | **Y3** OH300-61003CV-010.0M, **U39** TPS7A52 (LDO), **U38** AP3441 buck, **U36** OPA320 Vc buf, **R123 22 Ω** source term, **U37** INA228 #5 @0x46 |
| Clock mux | `sts1000_clock_mux.md` | **U52** 74LVC1G157, **U53** 74LVC1G34, R187/R188/R189, C160, D16 <!-- TODO verify designators: C141→C160?, D16 --> |
| RF front end | `ntp_server_rf_frontend.md` | **U49** TMUX1101, **U50** LTC6752xS5, **U51** LT3045, D18/D19/D20/D21, R184 33 Ω <!-- TODO verify clamp-diode designators: old D11/D12/D13/D14/D15 --> |
| Rb buck (VCC_RB) | `sts1000_vcc_rb_supply.md` | **U40A** MIC28516, **U43** MCP41U83 digipot, **U42** OPA320, **U45** MCP1502-30E (3.0 V ref), **U41A** LMV393 OV latch, **U44** INA228 @0x47, 1.5SMCJ28A TVS; output caps C125–C128 ≥ 50 V (open item) |
| FE-5680A serial | `rb_rs232_interface.md` | **U46** SN65C3221E, **K1** G6K-2F-Y DC3 relay, **U48** APC-817C1 opto, **U47** PESD15VL2BT |
| GNSS | checklist §6 | **U21** ZED-F9T-00B, **U22** LT3045 GPS LDO, **U23** INA228 #3 @0x4A (0x44=SHT45 U72) |
| Antenna bias/supervisor | `gnss_antenna_bias_supervisor.md` | **U24** INA181A1, **U25** LMV393, Q11/Q12/Q14, L1 47 nH, R77 3.3 Ω, **U26** INA228 #4 @0x45 <!-- TODO verify transistor designators: old Q1/Q2/Q4/Q5 --> |
| Ethernet/PTP | checklist §8 | **U11** LAN8742AI-CZ-TR + Y1 25 MHz xtal, magjack |
| PoE + power tree | checklist §9 | **U7/U8** FDMQ8205A bridge, **U9** NCP1095 PD, **U28** MIC28516 / **U29** AP3441 bucks, INA228 **U10**/#1 **U31**/#2 **U30**/#8 |
| Supercap backup | checklist §10 | **U34/U35** TPS61094 ×2, DSF305Q3R0 |
| I²C housekeeping | checklist §11 | **U56** LTC4311ISC6, **U57** TMP117 @0x49 / **U58** TMP117 @0x48, **U60** ATECC608B @0x60, **U61** IIS2MDC @0x1E, **U59** LIS2DH12 @0x19, **U72** SHT45 @0x44 |
| Fault/UI aggregation | checklist §12 (**direct GPIO** — no expander) | INA ALERTs on PG8–15 + PF13; PG rails PG0–7; buttons PF0–6; encoder PA8/PA9; `PROX_WAKE` reed PF10 |
| Panel-LED indicator | checklist §16 | **U54** INA228 #9 @0x4C (panel-LED 5 V), **U55** RT9742 load switch, Q22 PNP high-side, Q23 PWM |
| Display/touch | `sts1000_3v3p_i2c_peripherals.md` | LCDwiki MSP4030, **U33** RT9742, **U63** PCA9306, **U17/U18** TPD4E05U06DQAR, **U32** INA228 #7 @0x42, R104/R210/R211 |

### INA228 address map — 9 monitors, direct-scanned (GPS at 0x4A; panel at 0x4C)
| # | Addr | Rail (designator) | # | Addr | Rail (designator) |
|---|---|---|---|---|---|
| 1 | 0x40 | PoE input (U10) | 6 | 0x47 | Rb / VCC_RB (U44) |
| 2 | 0x41 | STM 3V3 (U31) | 7 | 0x42 | 5V_DISP (U32) |
| 3 | **0x4A** | GPS VCC (U23) — **0x44 = SHT45 U72** | 8 | 0x43 | main/general 3V3 (U30) |
| 4 | 0x45 | Antenna bias (U26) | 9 | **0x4C** | panel-LED 5 V (U54), ALERT→PF13 |
| 5 | 0x46 | OCXO (U37) | | | |

---

## Constraints that bind the BOM

- **Sourcing policy (narrow).** Banned: (1) IT/networking products with security exposure
  (switches, routers, networking silicon); (2) software that provides answers/content. Ordinary
  Chinese-origin passives, power ICs, analog parts, connectors are fine. Concrete BOM rule:
  **SPI-NOR = Macronix/Winbond (TW) or ISSI (US), NOT GigaDevice (PRC).** The external Rb and
  any replacement are non-Chinese-origin.
- **AEC-Q100/Q101 grade** throughout where available.
- **No DNP / no fit-jumper.** All hardware is populated; the MCU adapts paths at runtime via
  GPIO / analog switches. Do not add build-option straps.
- **Datasheet-verified parts only**, with exact values and math justification. AF data, when
  ST sources are unavailable, comes from `modm-io/modm-devices` (`stm32h5-62_63_73.xml`).
- **Package note:** STM32H563ZIT6 **LQFP144** (U12). Board is GPIO-full — the direct-GPIO
  fan-out (9 INA ALERTs, 8 PG rails, 7 buttons, encoder/touch/reed/faults on Ports F/G)
  commits the package's I/O; no spare routed GPIO (peripheral_map §13).

---

## Layout-critical rules (these are placement/routing, not schematic)

- **Continuous current-free GND pour under IIS2MDC.** Copper is diamagnetic/magnetically
  silent; a void routes return current into a loop under the sensor. Keep-out sizing
  `B = (μ₀/2π)·I/r`; balanced go+return gives 1/r² falloff vs 1/r single-ended. Place the
  e-compass as far as possible from the Rb physics package, OCXO oven, bucks, PoE magnetics,
  fan motor, and DC-rail currents (board edge/corner or external puck).
- **OCXO Vc-path caps: C0G/NP0 or film — never X7R** (X7R microphonics FM-modulate the carrier).
- **Rb FB node:** keep wiper/SPI switching noise off the high-impedance feedback node. The
  fixed bounding resistors (not the digipot) set the absolute min/max VCC_RB — verify no wiper
  code (POR/mid-scale/SPI-fault) can exit the Rb-safe envelope.
- **Source termination at the driver** (e.g. R123 at OCXO output → mux I0). No redundant
  receiver-end series resistor on the same net.
- **GPS SMA ESD (RF + bias):** the line carries 1.5 GHz RF (needs Cj ≤ 0.2–0.3 pF) **and** 5 V
  antenna bias (needs V_RWM ≥ ~6.5 V). Use a polymer ESD suppressor + a separate coaxial
  gas-discharge surge arrestor for the outdoor antenna run — standard clock-line ESD parts
  meet neither requirement alone.
- **Supercap thermal:** keep DSF305Q3R0 placement ≤ ~75 °C. Case Tmax 51.7 °C (< 65 °C corner) →
  full 3.0 V rating; 2.7 V charge term (13.0k VCHG) sits 10 % under it.
- RF island, display ESD arrays, and connector TVS placed at their connectors per the
  subsystem docs.

---

## Power / PoE facts (peripheral_map §6)

- Chain: 802.3bt-capable PoE → magjack center taps (all 4 pairs, Mode A+B) → FDMQ8205A bridge
  → NCP1095 PD → bucks → 3.3 V main + 5 V intermediate.
- **Classify Type 2 (802.3at, ~25.5 W) minimum, Type 3 (802.3bt) for margin** — Rb is
  permanent and can run concurrently with the OCXO (~12–14 W steady both-on, ~25–30 W
  cold-start peak). Class 3 (~13 W) does not cover it.
- `POE_KILL` (PE15) opens the buck-input pass FET; default-RUN via external pulldown;
  WDT/thermal/latched faults OR into the same driver.
- Backup managers (TPS61094 ×2) run strapped/autonomous (EN/MODE tied) — no MCU control pins;
  backup readiness is reported digitally on `BKP_STM_PG` (PF14) / `BKP_GPS_PG` (PF15), not an
  analog SOC channel.

---

## Open items before layout / fab (peripheral_map §14)

- LAN8742 nINTSEL/REGOFF straps + LED polarity for REF_CLK-Out; PHY addr vs PB10 strap.
- NCL/NCM/LCF (PC7/PC2/PC3): **resolved** — NCP1095 status pins are open-drain, RTN-referenced
  (U9.12 = GND), +72 V abs-max → safe direct to the 3.3 V GPIO but **float with no pull-up**. As-built
  POE_NCM/POE_NCL/POE_LCF have none: add 3× 10 kΩ → 3V3_STM (on-pattern) or use STM32 internal pull-ups.
  PD front end is non-isolated (RTN = board GND) → no digital isolator needed.
- INA228 shunt values per expected per-rail current (incl. OCXO rail #5; display #7 = 0.1 Ω,
  ADCRANGE=1).
- OCXO loop-filter design: center Vc 1.65 V, scale DAC 0–3.3 V to ±0.4 ppm, set loop τ.
- MCP41U83 (U43): code 0 = Terminal B = safe-low (VOUT min 4.51 V); POR = midscale ~14.5 V (within
  26 V OV); SPI Mode 0,0. Firmware pre-programs the NV wiper safe-low and verifies VCC_RB on INA228
  U44 (0x47) before trusting the Rb.
- Clock mux 74LVC1G157 (S=PB6): confirm EXTREF_MON acceptance band + switch hysteresis.
- RF front end: size Rseries + island clamp; R_VCC 33 Ω + D5 3.3 V VCC guard (S5 base = 3.6 V
  supply abs-max); source D4/D5 TVS; 1.5 V bias on the 3.0 V island; `REF_TERM_EN` safe reset.
- FE-5680A connector pinout per the specific variant (J6.8/J6.9 Tx/Rx direction ambiguous in
  FE manual §2-3.1). **10 MHz does NOT enter this connector — it goes to the SMA over coax.**
  Per-unit: a 15 V-class FE is over-volted by the 24.45 V full-scale pedestal (OV latch trips
  only at 26 V) — bound the digipot code / OV per the specific FE.
- LIS2DH12 **SA0 = 0x19**; confirm consumer −40/+85 °C adequacy.
- ZED-F9T V_BCKP current at max enclosure temp (datasheet only specs 45 µA @ 25 °C).
- **Supercap thermal:** DSF305Q3R0 is 3.0 V to 65 °C / 2.5 V @ 85 °C. Enclosure Tmax = 125 °F
  (51.7 °C) is below the 65 °C corner → full 3.0 V rating; VCHG termination set to **2.7 V**
  (R112/R113 = **13.0k 1%**, highest TPS61094 table option under the cap, ~1.5× backup energy vs 2.2 V).
  Keep supercaps ≤ ~75 °C by placement.
- RT9742 inrush vs I_LIM + 0.1 Ω shunt headroom (measure module backlight-full current + Cin).

---

## Conventions

- **Editor: vi only** for CLI text editing — never pico/nano.
- Review by **designator and pin** (R199, U54 GPB2, PA10), not node labels.
- When a part or value changes, update the owning subsystem doc **and** propagate the delta to
  the peripheral map / checklist in the same change; keep docs mutually consistent.
