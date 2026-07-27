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

### 1.1 Retracted finding — GPS RF_IN "missing DC-block"

**This finding was wrong and is withdrawn.** Earlier revisions of this repo's `CLAUDE.md` open-items
list asserted that "the u-blox ZED-F9T reference antenna-bias design places a 47 pF C0G series
DC-block between the bias-T node and RF_IN." **UBX-21040375 contains no such block** — the u-blox
reference circuits inject the antenna bias directly on the RF trace and rely on the receiver's
internal DC block, including where the bias comes from an external supply above VCC. (The
`gnss_antenna_bias_supervisor` subsystem doc said exactly that all along; the two docs contradicted
each other and the wrong one was propagated.)

A 47 pF part (`C204`) was added on the strength of that claim and, as drawn, landed in the **bias**
leg (`node A → L1 → C204 → GPS_RF_IN`) — which would have DC-blocked the antenna itself and left the
open/short supervisor reading permanent *open*. **C204 has been deleted.** The as-built topology is
`node A → L1 47 nH → GPS_RF_IN {J7.1, U21.2, L10.1}`, matching the u-blox reference. No action
outstanding; do not re-introduce the part.

### 1.2 Component derating to verify or correct

Netlist values cross-checked against the as-built part ratings in `sts1000_bom.csv`. These were not
previously flagged.

| Ref | As-built part | Stress | Assessment |
|---|---|---|---|
| **R77** 3.3 Ω (sets the antenna foldback limit) | **`RMCF1206FT3R30`** — **1206, 0.25 W** | At the 182 mA foldback clamp, P = 0.182² × 3.3 = **109 mW** — a *sustained* state on a shorted antenna cable, not a transient. | **RESOLVED** — 109 mW is 44 % of 0.25 W. (Was a 0402 / 0.1 W part at 109 % of rating.) Confirm the MPN/footprint on the next BOM export. |
| **R20** → 10 kΩ (`VOUT_P` → `POE_PG` pull-up) | **`ERJ-14NF1002U`** — **1210, 0.5 W, 200 V** | NCP1095 PGO (U9.14) is open-drain and sits low while the pass FET is on but the output is not yet good — the soft-start window on every power-up, plus any sustained output fault with the FET on. R20 then carries 54 V / 10 kΩ = 5.4 mA → **292 mW at 54 V, 325 mW at 57 V**, with the full bus voltage across the element. (When the pass FET is *off*, RTN rises toward VPP so R20 sees almost nothing — this is pulse/fault duty, not continuous.) | **RESOLVED** — 292 mW = 58 % of 0.5 W, 54–57 V = 27–29 % of 200 V. (Was a 0402 / 0.1 W / 50 V part at ~3× power and over its element voltage.) |
| **R2** → 1.2 MΩ (PFI divider, `V_POE` → U2A+) | **`ERJ-8ENF1204V`** — **1206, 0.25 W, 200 V** | R2/R3 = 1.2 M / 39.2 k → **52.3 V across R2** at 54 V, **55.2 V at the 57 V PoE max**; power 2.5 mW. | **RESOLVED** — 55.2 V = 28 % of 200 V, 2.5 mW = 1 % of 0.25 W. (Was `ERJ-U02F1204X`, an 0402 rated 50 V.) |
| **R264** → **162 kΩ** (`POE_PG` → PG7) | **`ERJ-8ENF1623V`** — **1206, 0.25 W, 200 V** | 50.7 V across the element = **25 %** of rating; 15.9 mW = 6.4 %. | **RESOLVED.** Value also moved 174 k → 162 k, widening PG7 to 2.75–3.13 V (margin over VIH 0.44–0.82 V). |
| **R18** 24.9 kΩ (`VOUT_P` → D4 zener, V_KBIAS) | `ERJ-2RKF2492X` — 0402, 0.1 W | 42 V drop at 1.69 mA → **71 mW** continuous (81 mW at 57 V). Element voltage 42 V < 50 V. | **Inside rating but at 71–81 % continuously** — the only PoE-side 0402 still near its limit. Acceptable; account for the self-heating in placement. |

### 1.3 Closed since the last revision
| Item | Resolution |
|---|---|
| Rb buck output caps C125–C128 | All **50 V** as-built: C126–C128 `KCM55WR71H476MH13L` (47 µF 50 V X7R, stacked SMD 2 J-lead), C125 `GCM155R71H473KE02J` (47 nF 50 V X7R 0402). Class-II derating at 24 V bias is now a loop-stability bench check (RB-2), not a sourcing item. |
| OCXO Vc caps C98/C99/C101 | `C1210C104J5GACAUTO` — 0.1 µF 50 V **C0G, 1210**. |
| Y-caps C1/C42/C186 | `C1812C472KGRACAUTO` — 4.7 nF 2 kV X7R **1812**. |
| C37 (VREF+) | 1 nF → **0.1 µF C0G 1210** at U12.32; R48/C38 unchanged as the reference filter. |
| Per-rail INA228 shunt values | **All nine final** — see `hardware_design_reference §2.1.1`. Only the per-board `SHUNT_CAL` trim remains (bench BM-1). |
| 3V0_RF LDO power-good | R266 10 kΩ → 3V3_STM fitted; PG2 is a normal power-good input, no firmware masking. |
| L7 (Rb buck inductor) | 40 µH → **39 µH**, Würth `7447709390` (4.1 A, 56 mΩ). |
| PHY straps R49/R228/R229/R230 | Non-E96 5 k → **4.99 k** (`ERJ-2RKF4991X`). |
| PoE HV caps C6–C11 | All 100 V with orderable PNs. |
| Footprint assignment | **Not achieved** — see §1.5. The Footprint property carries vendor package text for most parts. |

### 1.5 Layout-package gaps
| Item | Detail |
|---|---|
| **`fp-lib-table` missing** | The project has `sym-lib-table` but no `fp-lib-table`, so the custom footprints in `hardware/libraries/` are unreachable from the board editor. **Blocks opening the PCB.** |
| **Footprint fields are not land-pattern links** | Of 648 symbols only **25** carry a resolvable `library:footprint`; **587** hold a vendor package *description* in the Footprint property (`0402 (1005 Metric)`, `SOT-323`, `10-USON`, `Stacked SMD, 2 J-Lead`, …) that KiCad cannot resolve; **36** are empty (J1, J2, J6–J10, J15–J17, K1, K2, L2–L7, U53, U71, Y1, Y2 + mounting holes H1–H14). **31 parts lost a valid link during the BOM passes** (e.g. D10 `Diode_SMD:D_SOD-323F` → `SOD-323F`, D12 `Package_TO_SOT_SMD:SOT-23` → `SOT-23-3 (TO-236)`); those are recoverable from commit `78853e8`. **Hard blocker for layout.** |
| **R159 footprint name** | The library name reads `2515 (6332 Metric)`; 6332 metric **is** 2512 imperial, and both the MPN (`WSL25127L000FEA`) and the Description say 2512. Confirm the land pattern is a true 2512. |

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
5. **Per-board INA228 `SHUNT_CAL` trim** — the nine shunt *values* are final; what remains is
   removing the 1 % shunt tolerance by trimming `SHUNT_CAL` (default **4096**) against a reference
   load per rail and storing the result in NOR (bench BM-1).
6. **OH300 warm-up current** — U37 (0x46) across R126 25 mΩ gives ±1.638 A full scale, sized for the
   warm-up surge. Confirm the actual OH300 surge stays inside that (and inside U39's 2 A LDO limit).

---

## 2. Verification coverage (confirmed correct)

| Area | Result |
|---|---|
| **Pinouts** | All ICs verified against datasheets (STM32H563 DS14258, INA228, NCP1095, MIC28516, AP3441, TPS61094, LT3045, LAN8742, ZED-F9T, MCP41U83, SI7469DP, BC857W, BZX84C12, LTC6752, PCA9306, …). |
| **PoE power-good** | NCP1095 PGO (open-drain to RTN) → R20 10 k → R264 162 k / R263 10 k → PG7 = 2.97 V @54 V (2.75–3.13 V over the PoE range); PG3/PG7 are 5 V-tolerant (FT). |
| **Current monitoring** | 9× INA228, all ADCRANGE=1; shunt values final (R30 25 m, R106 100 m, R107 100 m, R102 50 m, R89 150 m, R126 25 m, R159 7 m/2512, R72 75 m, R199 150 m); PoE shunt R30 in the series V_POE feed; all ALERT pins have 10 k pull-ups. |
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
- **PG7 divider (R264 162 k / R263 10 k)** — NCP1095 PGO releases to the 54 V VPP rail when power-good;
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
- **INA228 shunts sized to 50–75 % of the ADCRANGE=1 full scale** — the ±40.96 mV range gives a 4×
  finer shunt LSB than ±163.84 mV and no rail here needs the wide range; landing each rail's design
  maximum in that band leaves transient headroom without wasting resolution. Setting
  `Max_Expected_Current` to each device's own full scale then makes **SHUNT_CAL = 4096 on all nine**,
  independent of R. Two deliberate exceptions: VCC_RB (7 mΩ, 36 % FS) is sized for the *low-voltage*
  end of its 4.5–24.45 V programmable range rather than the 15 V nominal, and 3V3_GPS (75 mΩ, 24 % FS)
  is sized by insertion drop (9.75 mV at peak) rather than by resolution.
- **Metal-strip (Vishay WSL) 1 % shunts rather than tighter thin-film** — tolerance is removed once
  per board by the firmware `SHUNT_CAL` trim; TCR and pulse-withstand are not, and metal strip wins
  on both.
