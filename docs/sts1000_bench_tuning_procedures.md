# STS1000 "Meridian" — Bench-Tuning & Unknown-Values Procedures

Consolidated register of every electrical item on the STS1000 that is **unknown**, **must be
measured**, or **should be tuned** before/at first bring-up. Sourced from the as-built netlist
verification (subsystem fact-sheets) + the remaining `sts1000_bom.md` sourcing items + `CLAUDE.md` standing open
items. Netlist is source of truth for designators.

**Scope:** first-article bring-up and per-board calibration. This is a procedures doc, not a
design reference — for *why* a circuit exists see the subsystem docs cited per item.

**Read order for bring-up:** do all **P0 (SAFETY-CRITICAL)** items, in order, on a
current-limited bench supply with the load disconnected, **before** any rail is allowed to
reach its target voltage or before enabling the Rb.

**Open PCB items to land before fab:** J17 panel-power (N-1, `PW-7`), R264 MPN check (N-4), and the
**0402 50 V element-voltage margin** on R2 / R20 / R264 (N-8). The KiCad project is also missing
`fp-lib-table` and 36 parts have no footprint (N-9). Design-review context and the full derating
table are in `sts1000_schematic_design_review.md §1.2/§1.5`.

**Retracted:** the "GPS RF_IN needs a 47 pF series DC-block per the u-blox reference" item
(ex-`GN-8`/`N-6`) was **not supported by UBX-21040375**; `C204` is deleted and the as-built
`node A → L1 → GPS_RF_IN` matches the u-blox reference. Do not re-open.

**Closed since the last revision** (no longer bench items): **R77** re-sized to a **0.25 W** part
(109 mW at the foldback clamp = 44 % of rating); all nine INA228 **shunt values** are final (BM-1 is
now a per-board gain trim, not a value selection); Rb buck output caps C125–C128 are **50 V**; OCXO
Vc caps C98/C99/C101 are **0.1 µF C0G 1210**; Y-caps C1/C42/C186 are **1812**; C37 is **0.1 µF C0G
1210**; the `3V0_RF_LDO_PG` pull-up **R266 is fitted** (PG2 needs no masking); L7 is **39 µH**
(7447709390); PHY straps are **4.99 k**.

---

## 0. Equipment inventory

| ID | Equipment | Key spec |
|---|---|---|
| E1 | Bench PSU, dual, **current-limit + current readback** | 0–30 V / 0–5 A, ≤1 mA setpoint on limit |
| E2 | PoE PSE / injector **or** HV bench PSU | 802.3bt Type-3 (51 W) or 0–60 V / 2 A isolated |
| E3 | 6½-digit DMM | DCV, 4-wire, µA current range |
| E4 | Oscilloscope ≥200 MHz, 4-ch | 10× probes + **HV differential probe** (≥100 V) |
| E5 | VNA or TDR | DC–3 GHz (L1 bias-T, MDI return loss, RF ESD) |
| E6 | Frequency counter, ≥10 digit/s | external-reference input, or phase comparator |
| E7 | Reference standard | GPSDO / house Rb **10 MHz**, ≤1e-11 |
| E8 | Function / arbitrary generator | slow ramp + 10 MHz sine/square, level-adjustable |
| E9 | Precision current source / SMU | 1 µA–300 mA, compliance ≥12 V |
| E10 | DC electronic load | CC/CR, 0–60 V, ≥5 A |
| E11 | Thermal chamber **or** hot-air + T/C | −40…+85 °C, or spot-heat + thermocouple |
| E12 | Logic / protocol analyzer | SPI + I²C decode |
| E13 | SWD debugger (ST-LINK) + FW scratch harness | register poke / GPIO / DAC / INA228 read |
| E14 | RS-232 + TTL-serial adapter + terminal | FE-5680A telemetry, level-selectable |
| E15 | Phase-noise / spectrum analyzer *(optional)* | OCXO carrier microphonics, close-in |
| E16 | LCR meter | cap value/dielectric, inductor, relay-coil R |

---

## 1. Priority-ranked master summary (SAFETY-CRITICAL first)

| ID | Pri | Subsystem | Item | Prereq | Accept criterion (short) |
|---|---|---|---|---|---|
| **RB-1** | **P0** | Rb | MCP41U83 (U43) wiper **code-0 parks safe-low** | — | Code 0 → wiper at terminal B (VREF_3V0), VCTRL→24.45 V **NOT** reached; verify BEFORE `RB_PWR_EN` |
| **RB-3** | **P0** | Rb | 26 V OV-latch trip + latch + clear | — | Fires at VCC_RB = 26.0 ±0.3 V, latches EN low, `RB_OV_RESET` HIGH clears |
| **PW-2** | **P0** | Power | STM32 **PG7** on POE_PG — verify R264/R263 divider | — | PG7 = 2.75–3.13 V at power-good over a 50–57 V bus (≤3.6 V); PG3/PG7 are FT |
| **PW-7** | **P0** | Power/HMI | **J17 panel power — 5 V (J17.16) + 3 V3 (J17.28)** (N-1) | **PCB item** | Rails present on J17 before plugging the panel |
| **RB-2** | **P1** | Rb | VCC_RB setpoint sweep vs `24.45 − 6.645·VCTRL` | RB-1/RB-3 | INA228 0x47 matches curve ±3 %; trim R134/R148 |
| **RB-4** | P1 | Rb | Q25 disconnect-gate enhancement + inrush | RB-1..3 | Vgs clamps ≤ −12 V; VCC_RB_G inrush non-destructive |
| **RB-7** | **P1** | Rb | Rb output-cap **effective C under 24 V DC bias** (rating closed: 50 V fitted) | E16 w/ DC bias | Ripple + load step inside MIC28516 budget with derated bulk |
| **GN-4** | P1 | GNSS | ANT_OFF disable (Q13) | — | On `ANT_OFF` assert node-A → ~0 V |
| **OX-4** | P2 | OCXO/RF | `3V0_RF_LDO_PG` functional check (R266 pull-up **fitted**) | — | PG2 reads good when U51 LT3045 is in regulation |
| **PW-4** | P1 | Power | Supercap C90/C91 Vmax vs 2.7 V VCHG term @ Tmax 51.7 °C | — | Cell V ≤ 3.0 V (≤65 °C); ~2.7 V term (R112/R113 13.0k) |
| **RB-5** | P2 | Rb | Ref-gate VREF_3V0 settle vs buck soft-start | — | VREF_3V0 stable before U40 SS ramp |
| **RB-6** | P2 | Rb | FE-5680A serial dir/baud/lock + Vmax vs pedestal | — | Telemetry decodes; FE Vmax ≥ 24.45 V or EN-interlocked |
| **OX-1** | P2 | OCXO | OH300 EFC pull range vs 1.65 V center | — | Pull ≥ temp+aging budget within 0–3.3 V EFC |
| **OX-2** | P2 | OCXO | Loop-filter corner verification | — | Corners near 15.9 Hz / 1.6 kHz first-cut |
| **OX-3** | P2 | OCXO | **DAC→Hz slope per-board calibration** | — | Slope logged; lock to GPS/Rb within budget |
| **RF-1** | P2 | Ref FE | LTC6752 (U50) hysteresis/chatter | — | No chatter on lowest-level source |
| **GN-1** | P2 | GNSS | Antenna DETECT ~5 mA threshold | — | Asserts ≈5 mA; holds 10–30 mA; trim R86 |
| **GN-2** | P2 | GNSS | Antenna SHORT (node-A < 1.52 V) | — | `GPS_ANT_SHORT` asserts; INA228 0x45 ≈ clamp |
| **GN-3** | P2 | GNSS | Foldback clamp ~182 mA | — | Hard short → ≈182 mA; Q11 within power; **R77 0402/0.1 W sees 109 mW — see N-7** |
| **GN-5** | P2 | GNSS | V_BCKP current @ max enclosure temp | — | U34+supercap holds ~4 h ephemeris; `BKP_GPS_PG` trips |
| **PW-1** | P2 | Power | U28 FREQ program + actual fSW | — | fSW at target; FREQ pin within abs-max |
| **PW-3** | P2 | Power | NCP1095 NCM/NCL/LCF pull-ups + DET/R15 (structure resolved: OD/RTN/72 V) | — | Pull-up present (10k or int-PU); clean HIGH + valid LOW |
| **PW-6** | P2 | Power | TPS61094 cold-start EN/MODE thresholds | — | Cold-start from supercap at min V |
| **MC-1** | P3 | MCU | PROX_WAKE magnetic-reed conditioning | — | Magnet-close = LOW wake; pull-up + close-to-GND correct |
| **MC-2** | P2 | MCU | Display 5V I²C pull-up presence | — | Module carries pull-ups, else fit 2×4.7k→5V_DISP |
| **MC-3** | P2 | MCU | OCXO_PSU_PG voltage domain (PG5 not FT) | — | PG5 node ≤ 3.3 V |
| **MC-4** | P2 | MCU | VREF+ decoupling vs ENOB | — | DAC/ADC noise floor meets timing-loop budget |
| **HM-1** | P2 | HMI | Panel-LED ballast final I vs Vf bins | — | I ≤ 75 % INA228 FS; rail sag acceptable |
| **HM-3** | P2 | HMI | Encoder quadrature + TIM1 filter (push-pull, no PU) | PW-7 (Vcc) | Clean quadrature at PA8/PA9; set TIM1 ICxF |
| **BM-1** | P2 | CAL | INA228 per-board `SHUNT_CAL` trim (×9) | reference load / DMM | Each rail within ±0.2 % of the reference after trim |
| **BM-2** | P3 | OCXO | OCXO Vc caps microphonic proof (C0G 1210 **fitted**) | E15, E16 | No FM sidebands under tap test |
| **RB-3b** | P2 | Rb | (see RB-3) RB_OV_DET level | — | ≈3.24 V (PE3-safe) |
| **GN-6** | P3 | GNSS | LT3045 I_LIM (R70=374 Ω) | — | Limit > F9T peak; no cold-start foldback |
| **GN-7** | P3 | GNSS | L1 47 nH bias-T RF (VNA) — 47 nH intentional | — | Insertion loss/isolation acceptable at 47 nH |
| **PW-5** | P3 | Power | PFI hold-up cap (C_port) | — | Hold-up ≥ POE_KILL/save window |
| **MC-5** | P3 | MCU | ATECC608B provisioned I²C address | — | Confirm 0x60 as shipped/provisioned |
| **HM-2** | P3 | HMI | RGB D5 Vf-bin trim + blue derating | — | Per-color I at target; blue within derate |
| **HM-4** | P3 | HMI | K2 DC3 relay coil resistance | — | Coil I < Q24 100 mA rating |
| **NW-1** | P3 | Net | Xtal load caps vs stray → REF_CLK 50.000 MHz | — | REF_CLK = 50.000 MHz; move 27→33 pF if ppm matters |
| **NW-2** | P3 | Net | RBIAS (R37 12.1k) TX amplitude/return loss | — | TX amplitude + return loss in DS window |
| **NW-3** | P3 | Net | PHY strap-detect budget (nINTSEL/REGOFF) | — | 4.75k pd (~5.08k) overrides internal pu |
| **NW-4** | P3 | Net | J1 magjack CT current rating (Type-2/3) | — | CT rated for Alt-A+Alt-B PoE |
| **RF-2** | P3 | Ref FE | Post-layout trim R187/R189/R177 | post-layout | PH0 damp / 50 Ω fanout / term return-loss |
| **BM-3** | P3 | BOM | C1/C42/C186 2 kV Y-cap package | — | 1808/1812 HV part; footprint fixed |
| **BM-4** | P3 | BOM | PoE HV caps (C8/C10; C6/C7/C9/C11) | — | Working-voltage-verified PNs, package fits |
| **BM-5** | P3 | BOM | Power inductors L2–L7 | — | Isat > peak; DCR/thermal OK at layout |
| **BM-6** | P3 | BOM | Connector / zener PNs | — | Orderable PNs assigned + verified |

**Design points to confirm at bring-up** (orientation/value verification, no rework expected):
- **D25** gate clamp K→VCC_RB, A→gate — Q25 enhances. Confirm orientation before RB-2/RB-4/RB-6.
- **Q3** high-side sustain PNP: E=V_KBIAS, C=HOLD — POE_KILL latches.
- **Q13** high-side PNP: E=V_ANT, C=Q11-B — ANT_OFF disable. Confirm before GN-4.
- **POE_PG→PG7** chain R20 10k + R264 162k / R263 10k → **2.97 V @54 V** (2.75 V @50 V, 3.13 V @57 V) — safe MCU tap. Confirm at PW-2.
- **INA_ALERT_VCC_RB** pull-up R265 10k→3V3_STM.
- **U10 PoE shunt** — loads on V_POE; **R30 25 mΩ** in the series feed.

**Open PCB items to land before fab:**
- **N-1 (CRITICAL)** — J17 panel power: J17.16→`5V_DISP`, J17.28→`3V3_STM`, encoder Vcc. Gates PW-7.
- **N-8 (MED)** — **0402 element-voltage / power margin on the PoE dividers**: R20 (292 mW and 54 V
  across a 0.1 W / 50 V part whenever PGO is low), R2 (52–55 V), R264 (51–54 V). Gates PW-2.
- **N-4 (BOM)** — R264 MPN `ERJ-8ENF1623V` (162k, 1206) — re-verify before ordering.
- **N-9 (LAYOUT)** — KiCad project has no `fp-lib-table`, and 36 parts have no footprint
  (H1–H14, J1, J2, J6–J10, J15–J17, K1, K2, L2–L7, U53, U71, Y1, Y2).

**Closed PCB items** (were N-2/N-3/N-5): Rb output caps C125–C128 are **50 V**; `3V0_RF_LDO_PG`
pull-up **R266 is fitted**; C37 is **0.1 µF C0G 1210**. OCXO Vc caps C98/C99/C101 are **C0G 1210**
and Y-caps C1/C42/C186 are **1812**.

---

## 2. Rubidium (SAFETY-CRITICAL) — `sts1000_vcc_rb_supply.md`, `rb_rs232_interface.md`

> **The Rb is destroyable by an over-voltage rail.** Do RB-1 and RB-3 before `RB_PWR_EN` is
> ever asserted with an FE-5680A connected. Until both pass, keep VCC_RB_G open (Q25 off) or
> the FE disconnected. INA228 0x47 (U44) is the rail witness — never trust the rail without it.

### RB-1 — MCP41U83 (U43) wiper code-0 direction parks safe-low  *(P0 SAFETY-CRITICAL)*
- **What/why:** The digipot wiper injects VCTRL into the MIC28516 (U40) FB node. The transfer
  is `VOUT = 24.45 − 6.645·VCTRL`, so **low VCTRL = high VCC_RB**. Per the MCP41U83 DS20007000B:
  **code 0x000 = Terminal B (VREF_3V0, VCTRL max) = safe-low** (VOUT min
  4.5 V); **POR forces midscale** (~14.5 V, inside the 26 V OV envelope) then loads NV-Wiper0;
  **SPI = Mode 0,0**. The dangerous 24.45 V pedestal is code 3FFh (Terminal A). Because the fixed
  bounding resistors keep even the extreme codes ≤ 24.45 V < 26 V OV trip, the mandatory POR
  midscale excursion is safe. Bench-confirm the direction on the actual part before enabling Rb.
- **Procedure:** Power only 3V3/5V logic; keep `RB_PWR_EN` LOW (U40 disabled, VREF_3V0=0 via
  ref-gate). Over SWD/SPI harness, read POR wiper code and drive codes 0 / mid / max. With the
  buck disabled you cannot read VCC_RB, so instead inject a known VREF_3V0 (bench 3.0 V into the
  P0B node via test point) and measure the wiper (P0W) voltage vs code on the DMM. Confirm
  code 0 → wiper ≈ VREF_3V0 (safe-low). Also capture: POR/default code, MTP/NV-wiper write
  frame + read-back, CRC default, SPI CPOL/CPHA that ACKs, wiper-write opcode.
- **Equipment:** E1, E3, E12, E13, plus 3.0 V ref source (E1 second channel).
- **Acceptance:** Code 0 and POR both park wiper at terminal B (VCTRL = VREF_3V0). Any code
  (POR / mid-scale / SPI-fault / open) yields VOUT ≤ 24.45 V. If code 0 = terminal A → **STOP**,
  re-map the FB network before enabling Rb.
- **Interlock:** `RB_PWR_EN` (PB7) stays LOW; FE-5680A disconnected until pass. Confirm safe
  code is written to NV-wiper so POR is safe-low.

### RB-2 — VCC_RB setpoint sweep + FB trim  *(P1; after RB-1/RB-3)*
- **What/why:** Verify the real rail follows `24.45 − 6.645·VCTRL` (VCTRL 0…3.0 V) and land the
  operating setpoint (15 V @ VCTRL≈1.42 for a 15 V FE; ~top for a 24 V FE). FB network is
  R147=20.0k, R148=604 Ω, R134=3.01k, R136=4.99k, **R159=7 mΩ (2512, 1 W)**, L7=39 µH.
- **Procedure:** With D25 orientation confirmed and RB-1 passed, enable U40 into an **electronic load** (not
  the FE). Sweep wiper code, log VCC_RB on DMM and INA228 0x47 in parallel; fit measured vs
  `24.45 − 6.645·VCTRL`. Trim R134 (slope) / R148 (pedestal) if the curve is off. Confirm INA228
  0x47 rail read matches DMM within tolerance (calibrates the witness).
- **Equipment:** E1, E3, E10, E13.
- **Acceptance:** Measured curve within ±3 % of formula across range; INA228 0x47 agrees with
  DMM ±1 %; setpoint reaches target FE voltage with ≥1.5 V guard below the 26 V OV trip.
- **Interlock:** Load, not Rb. Never write a code above the FE Vmax until RB-6 confirms it.

### RB-3 — 26 V OV-latch trip / latch / clear  *(P0 SAFETY-CRITICAL)*
- **What/why:** Firmware-independent backstop (U41A). Sense `0.0769·VCC_RB` vs 2.001 V → trip
  at VCC_RB > 26.0 V, pull `RB_PSU_PWR_EN` low (U40 off), latch (R151, U41 on 5V). `RB_OV_DET`
  (PE3) = 3.24 V witness; `RB_OV_RESET` (PD3, default LOW) pulse HIGH clears.
- **Procedure:** Disconnect Rb; drive VCC_RB node from a current-limited HV bench supply through
  the sense divider (or ramp the buck output via load). Slowly ramp; capture trip voltage on
  scope, confirm U40 EN goes low and *stays* low after ramp-down (latched). Read `RB_OV_DET`
  ≈3.24 V at trip. Pulse `RB_OV_RESET` HIGH → confirm latch clears and EN re-arms.
- **Equipment:** E2 (or E1 to 30 V), E3, E4, E8, E13.
- **Acceptance:** Trip = 26.0 ±0.3 V; latch holds; `RB_OV_DET` = 3.24 ±0.15 V; PD3 pulse clears.
- **Interlock:** FE disconnected. This is the guard that makes RB-2 safe — do it first.

### RB-4 — Q25 disconnect-gate enhancement + inrush  *(P1; after RB-1/RB-3)*
- **What/why:** SI7469DP P-ch (Q25) gates VCC_RB→VCC_RB_G to the clock connector. Intended
  Vgs = −0.909·VCC_RB (−13.6 V @15 V; −21.8 V @24 V > ±20 V abs-max → BZX84C12 clamp, D25).
  No dv/dt limit on VCC_RB_G — inrush TBD.
- **Procedure:** Assert `RB_VCC_GATE` (PB1); scope Vgs and confirm clamp
  ≤ −12 V at 24 V rail. Scope VCC_RB_G rising edge into a representative Cload + FE inrush;
  measure di/dt and Q25 SOA. If inrush excessive, add gate R/C to slow turn-on.
- **Equipment:** E1/E2, E4 (HV diff probe), E10.
- **Acceptance:** Vgs clamped to −12 V; Q25 within SOA; VCC_RB_G inrush non-destructive and
  below any downstream OV transient.
- **Interlock:** RB-1/RB-2/RB-3 passed.

### RB-5 — Reference-gate VREF_3V0 settle vs buck soft-start  *(P2)*
- **What/why:** `RB_PWR_EN` low → VREF_3V0 = 0 (digipot dead). On enable, VREF_3V0 (U45
  MCP1502-30, nSHDN gated by Q19/Q20) must settle **before** U40 soft-start ramps, or the FB
  injection is undefined during SS.
- **Procedure:** Scope VREF_3V0 and VCC_RB (or U40 SS pin) simultaneously on `RB_PWR_EN`
  rising edge. Confirm VREF_3V0 reaches final 3.0 V before the buck SS reaches the FB-active
  region.
- **Equipment:** E4, E13.
- **Acceptance:** VREF_3V0 stable ≥ (a few SS time-constants) before VCC_RB regulates; no rail
  overshoot during the sequencing window.
- **Interlock:** Load, not Rb, until confirmed.

### RB-6 — FE-5680A serial/lock + Vmax vs pedestal  *(P2)*
- **What/why:** FE J6.8/J6.9 Tx/Rx direction and baud are surplus-variant-dependent (manual
  §2-3.1 ambiguous); lock is inverting through opto U48 (firmware polarity bit). Must confirm
  the specific FE's Vmax vs the 24.45 V pedestal, and the K1 DPDT NC/NO map (de-energized must
  = RS-232 default).
- **Procedure:** With RB-1/2/3 passed and rail set to a *safe-low* value, connect FE via E14
  adapter; probe J6.8 (RB_RS232_TX→U46 RIN) and J6.9 (RB_RS232_RX). Identify Tx/Rx, baud, and
  frame; confirm telemetry decodes. Scope `RB_LOCK` polarity vs FE lock state. Confirm FE
  datasheet/label Vmax ≥ intended setpoint; if a 15 V FE, verify EN-interlock + soft-start keep
  it below its limit (pedestal has no hard guard below the 26 V trip). Verify K1 de-energized =
  RS-232 path.
- **Equipment:** E14, E4, E3, E13.
- **Acceptance:** Serial decodes at identified baud/direction; `RB_LOCK` polarity recorded for
  firmware; setpoint ≤ FE Vmax with margin; K1 map correct.
- **Interlock:** Rail proven safe (RB-2) and OV latch proven (RB-3) before FE is powered.

### RB-7 — Rb buck output-cap effective capacitance under DC bias  *(P1)*
- **Status:** the **voltage-rating item is closed** — C126/C127/C128 are `KCM55WR71H476MH13L`
  (47 µF **50 V** X7R, stacked SMD 2 J-lead) and C125 is `GCM155R71H473KE02J` (47 nF **50 V** X7R
  0402), all on `Net-(U44-IN+)` = the MIC28516 (U40) output = **VCC_RB** (24.45 V pedestal, 26 V OV
  trip). Input caps C133–C136 remain 100 V X7T.
- **What/why (remaining):** these are class-II dielectrics. At ~24 V bias a 50 V X7R typically holds
  **roughly half** its nameplate capacitance, so the loop compensation and ripple must be checked
  against the *derated* bulk, not 3×47 µF.
- **Procedure:** Measure effective C at the operating bias (E16 with DC bias, or infer from ripple).
  Sweep VCC_RB across the 4.5–24.45 V range and measure ripple + load-step response at each end;
  confirm the MIC28516 stays inside its stability/PSRR budget with the derated bulk. Consider C0G
  for C125 if the feedforward zero moves materially.
- **Equipment:** E16, E4, E10.
- **Acceptance:** ripple and transient response within the MIC28516 budget across the full output
  range with measured (derated) capacitance.
- **Interlock:** do as part of RB-2 while sweeping the rail toward the pedestal.

---

## 3. OCXO — `sts1000_clock_mux.md`, peripheral map §2

> Prerequisite: OCXO Vc-path caps C98/C99/C101 must be **C0G/NP0 or film** (BM-2 / findings_timing
> E1) before phase-noise-sensitive OCXO tuning — X7R microphonics FM-modulate the carrier.

### OX-1 — OH300 EFC pull range vs 1.65 V center  *(P2)*
- **What/why:** Vc centers at 1.65 V (R118/R119 off VDDA), buffered 0–3.3 V span (VDDA-limited).
  Must confirm the EFC pull authority spans the required temperature + aging budget within that
  span (not clipping at the rails).
- **Procedure:** Drive DAC1_OUT1 (PA4) across 0→3.3 V; at each point measure OCXO output on E6
  against the E7 reference. Record Hz vs Vc; identify pull at Vc=0 and Vc=3.3 V. Compare span to
  OH300 datasheet pull spec and to the aging + temp budget over the enclosure range (E11).
- **Equipment:** E6, E7, E13, E3, E11 (for temp sweep).
- **Acceptance:** Total EFC authority ≥ (initial tolerance + aging over service life + temp
  excursion) with ≥20 % margin, center near 1.65 V (Vc not near a rail at nominal lock).
- **Interlock:** none (OCXO safe to steer full-range).

### OX-2 — Loop-filter corner verification  *(P2)*
- **What/why:** First-cut analog corners: R120·C98 ≈ 15.9 Hz, R122·C101 ≈ 1.6 kHz. Firmware owns
  the slow disciplining loop; the analog filter must not fight it or pass DAC step noise.
- **Procedure:** With C0G caps fitted, inject a small DAC step; scope OCXO_V (PA3 node) settling
  and ringing. Confirm dominant corner ≈15.9 Hz and buffer-output corner ≈1.6 kHz. Adjust
  R120/R122 or caps only if step response is unacceptable.
- **Equipment:** E4, E13.
- **Acceptance:** Corners within ~30 % of design; clean settle, no overshoot into the carrier.
- **Interlock:** none.

### OX-3 — DAC→Hz slope per-board calibration  *(P2, per-board)*
- **What/why:** DAC-code-to-frequency slope varies part-to-part; firmware needs a per-board
  calibration constant to discipline against GPS/Rb.
- **Procedure:** Lock the board's OCXO output to E7 via E6 (or phase comparator). Step the DAC by
  known increments; measure Δf per ΔLSB; compute Hz/LSB and store as a per-board cal constant.
  Cross-check by closing the firmware loop against GPS PPS and against the Rb 10 MHz.
- **Equipment:** E6, E7, E13.
- **Acceptance:** Slope logged; closed-loop disciplining converges to target holdover/lock spec.
- **Interlock:** none.

---

## 4. Reference front end — `ntp_server_rf_frontend.md`

### RF-1 — LTC6752 (U50) hysteresis / chatter on lowest source  *(P2)*
- **What/why:** U50 is a single slicer with fixed internal hysteresis (S5 grade). At the lowest
  expected external-reference input level it must not chatter (double-edge) around the slice.
- **Procedure:** Inject a 10 MHz sine at the lowest spec'd amplitude into J8; scope U50 Q
  (10MHz_CLK_OUT node) for clean single edges. Add noise/attenuation to find the chatter floor.
  Verify U50 VCC ~2.85 V and input-B Voh ~2.65 V margins over temp (E11).
- **Equipment:** E8, E4, E11.
- **Acceptance:** No double-edging at minimum input level over temp; margin to slice threshold.
- **Interlock:** none.

### RF-2 — Post-layout trim R187 / R189 / R177  *(P3, post-layout)*
- **What/why:** R187 (PH0 series damper), R189 (fanout to 50 Ω SMA), R177 (switched-term return).
  Final values depend on trace impedance/length — post-layout tune.
- **Procedure:** After layout, scope PH0 edge (overshoot/ringing) → tune R187; measure 10MHz_RF_OUT
  into 50 Ω → tune R189 for match; TDR/VNA the switched termination → tune R177 for return loss.
- **Equipment:** E4, E5.
- **Acceptance:** PH0 clean edge (no double-clock), fanout matched to 50 Ω, term return loss
  meets spec.
- **Interlock:** none.

### OX-4 — `3V0_RF_LDO_PG` (PG2) functional check  *(P2)*
- **Status:** the pull-up item is **closed** — **R266 10 k → 3V3_STM** is fitted on
  `3V0_RF_LDO_PG` = {U12.87 (PG2), U51.4, R266.2}, mirroring R215 on the GPS LDO. **Firmware no
  longer masks PG2.**
- **What/why (remaining):** confirm the open-collector LT3045 PWRGD actually swings correctly with
  the fitted pull-up.
- **Procedure:** With the 3V0_RF island in regulation, scope PG2 → confirm HIGH (≈3.3 V) when good
  and LOW when U51 is forced out of regulation. Confirm R266 lands on 3V3_STM (not 3V0_RF).
- **Equipment:** E3, E4, E13.
- **Acceptance:** PG2 = HIGH when 3V0_RF is good, LOW on fault; firmware LDO-good telemetry tracks.
- **Interlock:** none.

---

## 5. GNSS + active antenna — `gnss_antenna_bias_supervisor.md`

> `GPS_ANT_DETECT`/`GPS_ANT_SHORT` = LMV393 U25; **presence via INA181 U24 across R77 (3.3 Ω)**,
> precise current via **INA228 U26 across R89 (150 mΩ, 520.833 nA/LSB)** — do not swap these in test scripts.

### GN-1 — Antenna DETECT threshold (~5 mA)  *(P2)*
- **What/why:** DETECT ref R85/R86 (100k/11k) → 0.327 V → ≈5 mA assert. Must assert for a
  low-draw active antenna and hold through 10–30 mA.
- **Procedure:** Replace antenna with an E9 current source on the coax. Sweep 0→30 mA; record the
  current at which `GPS_ANT_DETECT` asserts and confirm it holds to 30 mA. Trim R86 if a target
  antenna draws below the assert point.
- **Equipment:** E9, E4/E3, E13.
- **Acceptance:** Assert ≈5 mA (±1 mA); stable through 10–30 mA; matches chosen antenna's
  minimum draw with margin.
- **Interlock:** none (current-limited source).

### GN-2 — Antenna SHORT threshold (node-A < 1.52 V)  *(P2)*
- **What/why:** SHORT ref R82/R83 (100k/30k) → 0.762 V; node-A divider ÷2 → asserts when
  node-A < 1.52 V. INA228 0x45 (U26) should read the ≈182 mA foldback clamp during short.
- **Procedure:** Apply a current-limited short at the coax; confirm `GPS_ANT_SHORT` asserts and
  INA228 0x45 reads the foldback clamp. If comparator is slow/oscillates near threshold, add MΩ
  hysteresis on U25B.
- **Equipment:** E9/E10, E13, E4.
- **Acceptance:** SHORT asserts on hard short; INA228 0x45 ≈ 182 mA; no oscillation at threshold.
- **Interlock:** current-limit the short.

### GN-3 — Foldback clamp (~182 mA)  *(P2)*
- **What/why:** R77 3.3 Ω + Q12 VBE foldback limits antenna current to ≈182 mA (~20 % over
  150 mA). Verify the clamp value and the sustained-short dissipation. R77 is now a **0.25 W** part,
  so the 0.182² × 3.3 = **109 mW** clamp dissipation is 44 % of rating even when a shorted cable
  holds it there indefinitely. Q11 (~0.9 W ≪ 2 W) is fine.
- **Procedure:** Short the bias output through E10 in CC; measure clamped current on E3; verify
  R77/Q11 temperature with E11 T/C during sustained short.
- **Equipment:** E10, E3, E11.
- **Acceptance:** Clamp ≈182 mA; R77 ≤ 0.25 W and Q11 ≤ 2 W under a sustained short, no thermal runaway.
- **Interlock:** none.

### GN-4 — ANT_OFF disable (Q13)  *(P1)*
- **What/why:** Q13 (BC857W) is the high-side PNP (E=V_ANT, C=Q11-B) that lets the F9T `ANT_OFF`
  command shut the bias LNA. The firmware hard-cut via `ANT_BIAS_EN`→U27 is the independent
  short-protection path; this item verifies the F9T-commanded LNA-off route.
- **Procedure:** Assert the F9T `ANT_OFF` output (U21 pin 5, net `GPS_ANT_OFF_MON`); scope node-A → must collapse to ~0 V. Also
  confirm firmware gate of SHORT/OPEN alarms while commanded-off.
- **Equipment:** E4, E13.
- **Acceptance:** Node-A → ~0 V on `ANT_OFF` assert; bias fully disabled.
- **Interlock:** none.

### GN-5 — V_BCKP current @ max enclosure temp  *(P1)*
- **What/why:** ZED-F9T V_BCKP (GPS_VBAT) is fed only by TPS61094 U34 + supercap. Datasheet
  specs 45 µA @25 °C only; must confirm the backup current at max enclosure temp still lets the
  supercap hold the ~4 h ephemeris window, and that `BKP_GPS_PG` (PF15) trips on collapse.
- **Procedure:** In E11 at +85 °C (or spot-heat), measure GPS_VBAT current draw; cut VIN to
  U34 and time how long the supercap holds V_BCKP above the F9T retention minimum; confirm
  ≥4 h and that `BKP_GPS_PG` asserts when it falls. Also DSF305Q3R0 supercap placement ≤ ~75 °C.
- **Equipment:** E11, E3, E13, stopwatch.
- **Acceptance:** Hold-up ≥ ephemeris window (~4 h) at max temp; PF15 trips on collapse; supercap
  ≤ 75 °C.
- **Interlock:** none.

### GN-6 — LT3045 I_LIM (R70 = 374 Ω)  *(P3)*
- **What/why:** U22 LT3045 current limit must exceed F9T peak with no foldback at cold start.
- **Procedure:** Load 3V3_GPS with E10 up to the current limit; confirm limit > F9T peak draw
  and that 3V3_GPS does not foldback during F9T cold-start inrush.
- **Equipment:** E10, E4, E3.
- **Acceptance:** I_LIM > F9T peak with margin; no cold-start foldback. **R72 is now 75 mΩ** — drop
  is 9.75 mV at the 130 mA acquisition peak (≈3.31 V delivered); confirm on the bench.
- **Interlock:** none.

### GN-7 — L1 47 nH bias-T RF verify (VNA)  *(P3)*
- **What/why:** L1 = 47 nH (u-blox's own reference is 120 nH). 47 nH passes the DC bias current
  (300 mA-rated > 182 mA foldback) with the board relying on active foldback rather than a series R;
  Z ≈ 465 Ω is an accepted lower-isolation RF trade. VNA-verify only.
- **Procedure:** VNA the bias-T: measure insertion loss on the RF path and isolation to the bias
  node at 1.5 GHz; confirm L1 rated ≥182 mA (+ saturation). Accept 47 nH unless loss/isolation is
  out of budget.
- **Equipment:** E5.
- **Acceptance:** Insertion loss/isolation acceptable at 47 nH.
- **Interlock:** none.

### GN-8 — *(retracted — no longer an item)*
The "GPS RF_IN needs a 47 pF series DC-block" premise came from this repo's own `CLAUDE.md`
open-items list and was **not supported by UBX-21040375**: the u-blox active-antenna reference
circuits inject the bias directly on the RF trace and rely on the receiver's internal DC block.
`C204`, added on that premise, is **deleted**; the as-built `node A → L1 47 nH → GPS_RF_IN
{J7.1, U21.2, L10.1}` matches the reference. Normal antenna bring-up applies (GN-1/GN-2/GN-3).

---

## 6. Power / PoE — `sts1000_poe_kill.md`, `sts1000_power_fail_input.md`, peripheral map §PoE

> **PW-7 (J17 panel power, N-1)** is an open PCB item that must land before the panel is plugged.

### PW-2 — Verify POE_PG→PG7 divider + PG3/PG7 5V-tolerance  *(P0)*
- **What/why:** NCP1095 PGO is open-drain pulled to VOUT_P (54–57 V) via R20. The divider
  **R20 10 k + R264 162 k / R263 10 k → PG7 = 54 V·10/182 = 2.97 V** (2.75 V @50 V, 3.13 V @57 V), safe on a 3.6 V
  GPIO and a valid HIGH (>2.0 V); on fault PGO sinks → PG7 ≈ 0 V. Separately 5V_PSU_PG pulls PG3 to
  5 V; PG3 and PG7 are 5V-tolerant (FT).
- **Procedure:** At PoE power-good, scope PG7 = **2.9–3.1 V** (not 54 V) and confirm it tracks the
  NCP1095 PGO fault state (goes low on fault). Confirm from the STM32H563 DS14258 pin table that
  PG3/PG7 are **FT**; scope PG3 = 5 V at 5V power-good with no clamp current.
- **Equipment:** E2, E4 (HV diff probe), E3, datasheet.
- **Acceptance:** PG7 = 2.93 ±0.2 V at power-good, low on fault; PG3 FT, sits at 5 V without clamping.
- **Interlock:** confirm R264 = 174 k (N-4) **before** first PoE power — a 1.21 k there puts 48 V on
  PG7 and kills the MCU.

### PW-2b — AP3441 PG active-drive check (U29 3V3, U38 OCXO)  *(P0 — high consequence)*
- **What/why:** Both AP3441 bucks tap PG through a **divider to GND with NO external pull-up**
  (U29: R94 6.81k/R92 10k; U38: R125 3.57k/R124 10k). This is correct **only if** the AP3441 PG
  actively drives to VIN (5 V) when good, per Diodes DS39754 ("open drain … **pulled up to VIN**
  when good"). If PG were truly high-Z sink-only, `OCXO_PSU_PG` → 0 V → **U39 OCXO LDO never
  enables → OCXO rail dead** (loss of the primary reference), and PG4 telemetry would stick low.
  Verify empirically before trusting the OCXO enable path.
- **Procedure:** Power the 5 V rail with both bucks in regulation. Scope U29 PG and U38 PG pins and
  their divider nodes: confirm **3V3_PSU_PG (PG4) ≈ 2.98 V**, **OCXO_PSU_PG (node N) ≈ 2.98 V** (→ U39
  EN asserted) and **PG5 ≈ 2.66 V**. *(Node N is loaded by both R124 and the R232+R233 leg → ≈ 2.98 V,
  not the 3.68 V that ignores the second leg; PG5 = 0.89·node N ≈ 2.66 V. Both > VIH.)* Then pull each
  buck out of regulation and confirm the PG nodes drop low.
- **Equipment:** E1, E4, E3, E13.
- **Acceptance:** PG-good nodes reach the computed divider voltages (PG4 2.98 V, node N 2.98 V, PG5
  2.66 V; PG actively pulls to VIN); U39 OCXO LDO EN asserts and the OCXO rail comes up. If any PG node
  sits at 0 V when good → the part is sink-only; add a ~10k pull-up to 5 V and re-scale before relying
  on OCXO enable. If PG5 measures near VIH (2.31 V) — DS39754 specs no PG drive-Z, so node N can sag
  under the ~0.57 mA divider load — raise R233 (e.g. 22k) to pull PG5 toward node N.
- **Interlock:** do this early — a dead OCXO enable blocks all OCXO tuning (OX-1…OX-3).

### PW-7 — J17 panel power (5 V + 3 V3)  *(P0; PCB item, N-1)*
- **What/why:** The 36-pin J17 panel connector carries **J17.16 (display 5 V)** and **J17.28
  (logic 3 V3)** to the display, touch controller, and encoder Vcc. Confirm both supply pins land
  on their rails.
- **Procedure:** Confirm J17.16 = `5V_DISP` (so U54/U32 also monitor the display load) and
  J17.28 = `3V3_STM`, and that the encoder Vcc pin is fed. Ohm/scope each J17 supply pin to its
  rail; then plug the panel and confirm display, touch (via PCA9306), and encoder quadrature all
  come up.
- **Equipment:** E3, E4, E13.
- **Acceptance:** J17.16 = 5 V, J17.28 = 3 V3, encoder Vcc present; panel + touch + encoder
  functional. Also completes MC-2 (5 V-side I²C pull-ups) once J17.16 5 V is live.
- **Interlock:** the panel-power routing must be on the fabricated board — this is not a bench tune.

### PW-1 — U28 FREQ programming + actual fSW  *(P2)*
- **What/why:** MIC28516 U28 FREQ set by R90 (200k→VOUT_P)/R91 (100k→GND) — unloaded ~18 V,
  self-consistent only if FREQ internally clamps low. Must confirm the programming method,
  FREQ-pin abs-max, and actual switching frequency.
- **Procedure:** Measure FREQ-pin voltage vs datasheet program table; confirm within abs-max.
  Scope the switch node for actual fSW; compare to intended. Confirm EXTVDD=GND strap validity
  and MIC28516 EN abs-max ≥ 60 V.
- **Equipment:** E4, E3, datasheet.
- **Acceptance:** fSW at target; FREQ pin within abs-max; EN rating ≥ 60 V.
- **Interlock:** none once PW-2 clear.

### PW-3 — NCP1095 NCM/NCL/LCF pull-ups + DET/R15  *(P2 — output structure RESOLVED)*
- **What/why:** NCM/NCL/LCF (→PC2/PC7/PC3) are **open-drain, RTN(=GND)-referenced, +72 V abs-max**
  (datasheet, U9.12=GND) — **safe direct to the 3.3 V GPIO** (no VPP hazard, unlike PGO). But they
  **float without a pull-up**, and as-built there is none: add 3× 10 kΩ→3V3_STM **or** enable STM32
  internal pull-ups on PC2/PC3/PC7. Also confirm DET/COSC (R15 bridges COSC↔DET).
- **Procedure:** Verify a pull-up source is present (external part or firmware internal-PU config).
  Scope each net idle (should read HIGH = pull-up rail) and active (NCP1095 sinks LOW, VOL ≤ 0.5 V @
  2 mA). Confirm valid class-code read on NCM/NCL. Confirm R15 COSC↔DET function.
- **Equipment:** E4, E3, datasheet.
- **Acceptance:** Each line reads a clean HIGH (pull-up present) and a valid LOW when asserted; class
  code decodes; R15 role confirmed.
- **Interlock:** ensure the pull-up (external or internal) is provisioned before relying on these reads.

### PW-4 — Supercap C90/C91 Vmax vs VCHG term at temp  *(P1)*
- **What/why:** TPS61094 VCHG sets the supercap termination to **2.7 V** (R112/R113 = **13.0k 1%**).
  Enclosure Tmax = **125 °F (51.7 °C)** is below the DSF305Q3R0 65 °C corner, so its full 3.0 V rating
  applies and 2.7 V sits 10 % under it. This item confirms the cell voltage stays within the derated
  max at the enclosure Tmax.
- **Procedure:** Charge the supercaps; measure terminal voltage at the enclosure Tmax (~52 °C, E11);
  compare to the cell's temperature-derated max (3.0 V at ≤65 °C). If the enclosure spec ever rises
  above 65 °C, drop VCHG back down the table (13.0k→9.53k = 2.6 V, →6.65k = 2.5 V, →4.75k = 2.2 V).
- **Equipment:** E11, E3.
- **Acceptance:** Cell voltage ≤ 3.0 V (≤65 °C) with margin; ~2.7 V nominal term (±2 % → ≤2.75 V).
- **Interlock:** none.

### PW-5 — PFI hold-up cap (C_port)  *(P3)*
- **What/why:** U2A PFI trips at 38.1 V (R2/R3, 36.8/38.8 V hyst). The port hold-up capacitance
  must carry the POE_KILL / context-save window after PFI asserts.
- **Procedure:** Trigger PFI (ramp input down via E2); scope VOUT_P decay and measure hold-up time
  vs the firmware save/kill window. Size C_port if insufficient.
- **Equipment:** E2, E4, E13.
- **Acceptance:** Hold-up ≥ save/kill window with margin.
- **Interlock:** none.

### PW-6 — TPS61094 cold-start EN/MODE thresholds  *(P2)*
- **What/why:** U34/U35 EN/MODE are self-referenced; must confirm the board cold-starts (boosts
  from supercap) at the minimum supercap voltage.
- **Procedure:** Discharge supercap to minimum; apply/remove VIN and confirm EN/MODE logic
  boots the boost path and holds VBAT=3.0 V; scope the cold-start threshold.
- **Equipment:** E1, E4, E10.
- **Acceptance:** Reliable cold-start from min supercap V; VBAT holds 3.0 V.
- **Interlock:** none.

---

## 7. MCU + peripherals — `sts1000_3v3p_i2c_peripherals.md`, peripheral map §13

### MC-1 — PROX_WAKE magnetic-reed conditioning  *(P3 — confirm only)*
- **What/why:** The proximity sensor is a **passive magnetic reed switch** (dry contact) on
  J17.18. `PROX_WAKE` (PF10) = R254 10k **pull-UP** to 3V3_STM + C202 debounce, reed closes to
  GND → **close = active-low wake**. A reed is passive, so **no Vcc pin is needed** and pull-up +
  close-to-GND is the correct, fail-safe conditioning.
- **Procedure:** Bring a magnet to the reed; scope PROX_WAKE goes LOW on close, returns HIGH
  (3V3_STM) on release; confirm C202 debounces contact bounce. Firmware treats LOW as wake.
- **Equipment:** E4, E13.
- **Acceptance:** Idle = HIGH (no wake); magnet-close = LOW (wake); clean debounced edge.
- **Interlock:** none.

### MC-2 — Display-module 5 V I²C pull-up presence  *(P2)*
- **What/why:** PCA9306 U63 5V-side (I2C_DISP) has **no on-board pull-ups** — works only if the
  display module carries its own 5 V I²C pull-ups.
- **Procedure:** Ohm/scope I2C_DISP_SCL/SDA with module attached; confirm pull-ups to 5V_DISP
  present. If absent, fit 2×~4.7k → 5V_DISP.
- **Equipment:** E3, E4.
- **Acceptance:** Both lines pulled to 5V_DISP; I²C to display ACKs.
- **Interlock:** none.

### MC-3 — OCXO_PSU_PG voltage domain (PG5 not FT)  *(P2)*
- **What/why:** PG5 is `Net-(U12B-PG5)` = R232 (1.21k)/R233 (10k) divider from OCXO_PSU_PG →
  PG5 ≈ 0.89·OCXO_PSU_PG. PG5 is **not 5V-tolerant** — confirm OCXO_PSU_PG ≤ 3.3 V or that the
  divider keeps PG5 ≤ 3.3 V.
- **Procedure:** Measure OCXO_PSU_PG source rail; compute/measure PG5 node voltage at power-good.
- **Equipment:** E3, E13.
- **Acceptance:** PG5 ≤ 3.3 V under all conditions.
- **Interlock:** none.

### MC-4 — VREF+ decoupling vs ENOB  *(P3)*
- **Status:** the cap item is **closed** — C37 is now **0.1 µF C0G 1210**
  (`C1210C104J5GACAUTO`) at U12.32, with C38 2.2 µF behind R48 50 Ω as the reference-noise filter.
- **What/why (remaining):** confirm the achieved ADC/DAC noise floor actually meets the OCXO
  steering-loop budget.
- **Procedure:** Histogram a static Vc read on PA3 and measure the DAC output noise on PA4; compare
  against the loop budget.
- **Equipment:** E13, E4, E15 (optional).
- **Acceptance:** DAC/ADC noise meets the timing-loop budget.
- **Interlock:** none.

### MC-5 — ATECC608B provisioned I²C address  *(P3)*
- **What/why:** U60 default 0x60 on I²C1, but ATECC608B address can be provisioned — confirm the
  shipped/provisioned address matches firmware.
- **Procedure:** Scan I²C1; confirm U60 ACKs at 0x60 (or record provisioned address).
- **Equipment:** E13/E12.
- **Acceptance:** Address matches firmware config (0x60 expected).
- **Interlock:** none.

---

## 8. HMI / panel — `sts1000_panel_controls.md`, `sts1000_rgb_indicator.md`

### HM-1 — Panel-LED ballast final current vs Vf bins  *(P2)*
- **What/why:** 6× white R247–R252 = 91 Ω, 1× red R253 = 150 Ω on V_PANEL_LED ≈ 4.7 V →
  ~17–18 mA each, all-on ≈126–132 mA. INA228 U54 shunt **R199 = 150 mΩ** (19.8 mV = **48.3 % FS**).
  Final current depends on LED Vf bin and rail sag — must stay ≤ 75 % of INA228 FS.
- **Procedure:** With final LED bins fitted, measure per-string current and total on INA228 0x4C;
  measure V_PANEL_LED sag all-on. Confirm total ≤ 75 % of ±40.96 mV FS across R199 (48 % as-built, so a
  brighter ballast has room). Adjust ballast
  if a bin shifts current out of range.
- **Equipment:** E3, E13.
- **Acceptance:** Per-LED current at target brightness; INA228 shunt drop ≤ 75 % FS; no visible
  dimming from rail sag.
- **Interlock:** none.

### HM-2 — RGB D5 Vf-bin trim + blue derating  *(P3)*
- **What/why:** Common-anode 5 V: blue LED_B R59 60.4 Ω → 24.3 mA; red R60 174 Ω → 16.7 mA;
  green R61 221 Ω → 8.4 mA. Per-color current depends on Vf bin; blue may need derating.
- **Procedure:** Measure each color's current with final bins; verify balanced perceived
  brightness and blue within its derated max. Trim R59/R60/R61 as needed.
- **Equipment:** E3.
- **Acceptance:** Per-color current at target; blue within derating.
- **Interlock:** none.

### HM-3 — Encoder quadrature + TIM1 filter  *(P2 — no pull-up needed)*
- **What/why:** The optical encoder is a **push-pull (driven) output**, so ENC_A_IN/ENC_B_IN need
  **no pull-ups**; they drive U68 74LVC2G17 Schmitt inputs → PA8/PA9 (TIM1). The dependency is the
  encoder Vcc from J17 (N-1 / PW-7) — quadrature can't be driven until that rail is present.
- **Procedure:** With encoder Vcc present (PW-7), turn the encoder; scope ENC_A/ENC_B quadrature
  into TIM1; set the TIM1 input-capture ICxF digital filter to debounce without missing counts.
- **Equipment:** E4, E13.
- **Acceptance:** Clean push-pull quadrature at PA8/PA9; count tracks detents; no missed steps.
- **Interlock:** PW-7 (encoder Vcc) done first.

### HM-4 — K2 DC3 relay coil + fail-safe  *(P3)*
- **What/why:** K2 (G6K-2F-Y DC3, holdover alarm, normally-energized fail-safe) coil is **91 Ω /
  3 VDC → 33 mA** (~100 mW) — within Q24 (BC847W, 100 mA). At 3V3 the coil sees ~110 % of rated
  voltage; contacts are 2 Form C, de-energized COM→NC = alarm. The check is the fail-safe + coil
  derating at max enclosure temp.
- **Procedure:** Confirm coil R (91 Ω) with E16; energize/de-energize and confirm fail-safe
  (de-energized → NC = alarm). If a warm enclosure eats coil-voltage margin, consider a ~30 Ω
  series resistor to bring the coil to ~3.0 V.
- **Equipment:** E16, E3, E13.
- **Acceptance:** Coil I < 100 mA with margin; relay toggles; fail-safe on power loss.
- **Interlock:** none.

---

## 9. Networking — `findings_ethernet` (Ethernet input + LAN8742 PHY)

### NW-1 — Xtal load caps vs stray → REF_CLK 50.000 MHz  *(P3)*
- **What/why:** Y1 CL=20 pF; C14=C16=27 pF → CL_eff ≈13.5 pF + Cstray (needs ~6.5 pF stray to
  hit 20 pF) → a few ppm fast. Harmless (RMII decoupled from disciplined timing) but confirm.
- **Procedure:** Scope/counter REF_CLK (PA1) = 50.000 MHz. If ppm matters, move C14/C16 → 33 pF.
- **Equipment:** E6, E4.
- **Acceptance:** REF_CLK = 50.000 MHz within RMII tolerance (±50 ppm ok).
- **Interlock:** none.

### NW-2 — RBIAS (R37 12.1k) TX amplitude / return loss  *(P3)*
- **What/why:** RBIAS = 12.1k 1% = exact LAN8742A DS value; verify TX amplitude and MDI return
  loss are in the DS window.
- **Procedure:** VNA/scope MDI TX amplitude and return loss per 100BASE-TX template.
- **Equipment:** E5, E4.
- **Acceptance:** TX amplitude + return loss within DS/template.
- **Interlock:** none.

### NW-3 — PHY strap-detect budget (nINTSEL/REGOFF)  *(P3)*
- **What/why:** nINTSEL=0/REGOFF=0 use 4.75k pull-downs behind 330 Ω LED series R (~5.08k at pin)
  to override internal pull-ups; confirm the latch is reliable.
- **Procedure:** Confirm PHY latches REFCLKO + reg-on modes at reset (read config via MDIO). If
  marginal, tie R40/R41 to the LED pin or drop to ~2.2k.
- **Equipment:** E13, E4.
- **Acceptance:** Straps latch correctly every power-up; PHY ID reads at addr 0.
- **Interlock:** none.

### NW-4 — J1 magjack CT current rating (Type-2/3 PoE)  *(P3)*
- **What/why:** Reconcile J1 orderable PN (value 0826-1X1T-HS-F vs footprint lib -GH-F) and
  confirm magjack center-tap current rating for Alt-A+Alt-B PoE (VC1..VC4).
- **Procedure:** Confirm orderable PN; from magjack datasheet confirm CT current rating ≥ Type-3
  per-pair current. Thermally check under full PoE load (E11).
- **Equipment:** datasheet, E11.
- **Acceptance:** Orderable PN fixed; CT rating ≥ PoE per-pair; no CT heating.
- **Interlock:** none.

---

## 10. Current-monitor calibration & remaining BOM selects — `sts1000_bom.md`

> Shunt **values are final** (see the table below). What remains is a per-board gain trim and a
> handful of measure-then-commit sourcing checks.

### BM-1 — INA228 per-board `SHUNT_CAL` trim (×9)  *(P2, per board)*
- **What/why:** all nine shunts are 1 % parts, and that 1 % dominates the uncalibrated current
  error (the INA228's own gain error is an order of magnitude smaller). One multiply per device
  removes it. Values, packages and full-scale figures are **final as-built**:

  | Ref | Value | Package / rating | Rail / INA228 | FS current | CURRENT_LSB |
  |---|---|---|---|---|---|
  | R30 | **25 m** | 1206 / 0.25 W | U10 PoE-input (0x40) — in the series `VOUT_P`→`V_POE` feed | ±1.6384 A | 3.125 µA |
  | R106 | **100 m** | 1206 / 0.25 W | U31 STM 3V3 (0x41) Kelvin split | ±409.6 mA | 781.25 nA |
  | R107 | **100 m** | 1206 / 0.25 W | U32 5V_DISP (0x42) | ±409.6 mA | 781.25 nA |
  | R102 | **50 m** | 1206 / 0.25 W | U30 main 3V3 (0x43) | ±819.2 mA | 1.5625 µA |
  | R89 | **150 m** | 1206 / 0.25 W | U26 antenna-bias (0x45) | ±273.1 mA | 520.833 nA |
  | R126 | **25 m** | 1206 / 0.25 W | U37 OCXO (0x46) | ±1.6384 A | 3.125 µA |
  | R159 | **7 m** | **2512 / 1 W** | U44 Rb/VCC_RB (0x47) | ±5.8514 A | 11.1607 µA |
  | R72 | **75 m** | 1206 / 0.25 W | U23 GPS-VCC (0x4A) | ±546.1 mA | 1.04167 µA |
  | R199 | **150 m** | 1206 / 0.25 W | U54 panel-LED (0x4C) | ±273.1 mA | 520.833 nA |
  | R16 | 25 m | 1206 / **1 W** (Ohmite MCS1632) | PD hot-swap RSNS (Q2/NCP1095) | — | **not** an INA228 shunt |

  All nine run **ADCRANGE=1** and start from **`SHUNT_CAL` = 4096** (see
  `sts1000_firmware_hardware_interface.md §4.2`).

- **Procedure (per rail, per board):**
  1. Bring the rail up and settle it at a **steady mid-range load** — ideally 40–60 % of that
     monitor's full scale, so neither offset nor clipping dominates. Where the natural load is too
     light (3V3_GPS, V_ANT), substitute a bench electronic load in place of the normal one.
  2. Measure the true current with a calibrated DMM in series (or a calibrated shunt + 6½-digit
     meter). Record `I_ref`.
  3. Read the INA228 `CURRENT` register with `SHUNT_CAL` = 4096 and convert with that rail's
     CURRENT_LSB. Record `I_reported`.
  4. Compute `SHUNT_CAL_trim = round(4096 × I_ref / I_reported)`, clamp to 15 bits, write it, and
     re-read to confirm the rail now agrees with the DMM.
  5. Store the nine trims in NOR with the calibration record. **Firmware must re-apply them after
     any INA228 reset** — the part reverts to its POR calibration silently.
  6. Also record the **no-load offset** on each rail (rail up, load removed): it is the floor for
     any under-current / open-load threshold.
- **Equipment:** E10 (DMM), E3 (bench supply), E13 (electronic load).
- **Acceptance:** each rail reads within **±0.2 %** of the DMM after trim; no-load offset recorded;
  the nine trims persisted and re-applied across a power cycle.
- **Interlock:** run **after** the rail is verified safe by its own P0/P1 item; the VCC_RB trim
  (0x47) comes after RB-1/RB-2, not before.

### BM-1a — Alert-threshold commissioning  *(P3)*
- **What/why:** `SOVL`/`SUVL` are shunt-voltage registers (**1.25 µV/LSB** at ADCRANGE=1, 16-bit
  signed, FS code 32767 = 40.96 mV) and are **not** scaled by `SHUNT_CAL`. Defaults at 125 % of each
  rail's design maximum are tabulated in `sts1000_firmware_hardware_interface.md §4.2`.
- **Procedure:** after BM-1, load each rail to its real worst case (measured, not assumed), confirm
  the alert does **not** fire, then force an over-load and confirm the ALERT GPIO asserts and
  `DIAG_ALRT` names the right cause. Commit the final codes to the calibration record.
- **Special cases:**
  - **U26 (V_ANT, 0x45):** the R77 foldback clamps at ≈182 mA autonomously, so the 227 mA `SOVL`
    can never fire in normal operation — it is a foldback-failure backstop. Use `SUVL` against the
    BM-1 no-load offset to detect a **disconnected** antenna.
  - **U32 (5V_DISP, 0x42):** the RT9742 current limit may sit **above** the ±409.6 mA full scale, so
    a display short clips the `CURRENT` reading. Verify the *pair* (INA alert PG10 +
    `V_DISP_EN_FAULT` PF9) annunciates, and do not rely on the magnitude.
  - **U54 (panel, 0x4C):** set `AVG` so the averaging window is ≫ the 1 ms PWM period before
    commissioning the threshold, then duty-normalize.
- **Acceptance:** every alert fires on a real over-load and stays quiet at worst-case normal load.
- **Interlock:** after BM-1.

### BM-2 — OCXO Vc caps microphonic proof (C98/C99/C101)  *(P3)*
- **Status:** the dielectric/package item is **closed** — all three are
  `C1210C104J5GACAUTO`, **0.1 µF 50 V C0G in 1210** (0.1 µF C0G is unbuildable below 1210).
- **What/why (remaining):** confirm on a real board that the C0G parts leave no microphonic
  signature on the carrier.
- **Procedure:** Verify the fitted dielectric with E16. Tap/vibrate the board while watching OCXO
  close-in phase noise and spurs (E15); confirm no FM sidebands appear.
- **Equipment:** E16, E15.
- **Acceptance:** no microphonic FM sidebands under tap test.
- **Interlock:** do before OX-2/OX-3 phase-noise-sensitive tuning.

### BM-3 — C1/C42/C186 2 kV Y-cap package  *(P3)*
- **What/why:** As-built 4700 pF but 0402 impossible at 2 kV — shield-to-chassis HV Y-caps need a
  1808/1812 HV part + footprint fix.
- **Procedure:** Assign a 4.7 nF 2 kV Y-cap PN in 1808/1812; update footprint; confirm creepage.
- **Equipment:** datasheet.
- **Acceptance:** HV-rated PN + correct footprint assigned.
- **Interlock:** none.

### BM-4 — PoE HV caps (C8, C10; REVIEW-HV C6/C7/C9/C11)  *(P3)*
- **What/why:** C8 0.1 µF 100 V + C10 10 µF 100 V on VOUT_P are undersized in 0402/0805 →
  1210/1812 PNs. C6/C7/C9/C11 on VOUT_N assigned at 100 V — confirm the node's true working
  voltage.
- **Procedure:** Measure VOUT_P / VOUT_N working voltage under load; assign package/PN with ≥2×
  derating; verify footprints fit.
- **Equipment:** E4 (HV diff probe), E2.
- **Acceptance:** All PoE HV caps voltage-verified and package-fit.
- **Interlock:** none.

### BM-5 — Power inductors L2–L7  *(P3)*
- **What/why:** L2 6.8 µH ≥12 A (5 V buck), L3/L6 2.2 µH ~4 A (3V3/OCXO bucks), L4/L5 2.2 µH
  (TPS61094 backup — size Isat for boost peak), **L7 39 µH (Würth 7447709390, 4.1 A, 56 mΩ)** (Rb high-Vout buck). Specific PN is a
  layout/thermal call.
- **Procedure:** Confirm each buck's peak inductor current under worst-case load; select Isat >
  peak with margin, acceptable DCR/thermal at the layout footprint. Verify with E16 + thermal
  under load (E11).
- **Equipment:** E10, E16, E11.
- **Acceptance:** Isat > peak (incl. TPS61094 boost peak for L4/L5); DCR/thermal acceptable.
- **Interlock:** none.

### BM-6 — Connector / zener PNs  *(P3)*
- **Status:** **closed** — every BOM line now carries a manufacturer PN and a DigiKey PN.
  D16/D20/D21 = `BZX84C3V3LT1G`; J2/J10/J16 = On Shore `OSTVN0*A150`; J6 = EDAC `622-009-260-033`;
  J7/J8/J9/J15 = Samtec `SMA-J-P-H-RA-TH1`; J17 = Phoenix `1787179`; J1 = Bel/Stewart
  `0826-1X1T-HS-F`; U50 = `LTC6752IS5#TRMPBF` (**I** grade); U27/U33/U55 = `RT9742VGJ5` (standardized).
- **What/why (remaining):** confirm the LTC6752 **I** grade is fast enough against the RF-1 chatter
  result, and re-verify **R264** = `ERJ-8ENF1623V` (162 k, 1206) before ordering — a 1.21 k part there
  would put ~48 V on PG7.
- **Equipment:** datasheets.
- **Acceptance:** LTC6752 grade confirmed against RF-1; R264 PN re-verified.
- **Interlock:** none.

---

## 11. Cross-references

| Source | Items |
|---|---|
| `findings_rb` §8 / `sts1000_vcc_rb_supply.md` | RB-1…RB-6 |
| `findings_timing` §8 / `sts1000_clock_mux.md` | OX-1…OX-3, RF-1, RF-2, BM-2 |
| `findings_gnss` §8 / `gnss_antenna_bias_supervisor.md` | GN-1…GN-8 |
| `findings_power` / `sts1000_poe_kill.md`, `sts1000_power_fail_input.md` | PW-1…PW-6 |
| `findings_mcu` §9 | MC-1…MC-5 |
| `findings_hmi` §9 / `sts1000_panel_controls.md`, `sts1000_rgb_indicator.md` | HM-1…HM-4 |
| `findings_ethernet` §8 | NW-1…NW-4 |
| `sts1000_bom.md` (sourcing + shunt table) | BM-1, BM-1a, BM-2…BM-6 |
| `CLAUDE.md` standing open items | RB-1, MC-5, GN-5, PW-4, RB-6, GN-6, HM-1 |

**Total items: 48** (P0: 5 · P1: 7 · P2: 21 · P3: 15) across 9 subsystems. The five open PCB items
(N-1…N-5) gate PW-7 / RB-7 / OX-4 / MC-4; the rest are bench-verify and per-board calibration.
