# STS1000 BOM — Standardization & Sourcing Status

Authoritative parts data lives in the **KiCad symbol fields** and is exported to
`hardware/ntp-server-stm32/sts1000_bom_v2.csv` (636 BOM lines = 647 as-built components
− 14 mounting holes, plus the R263/R264/R265 divider/pull-up parts). Each symbol carries
`Value`, `Description`, `MANUFACTURER`, `MANUFACTURER_PART_NUMBER` (MPN) plus the spec
columns `VOLTAGE`, `DIELECTRIC`, `TOLERANCE`, `POWER`, and a `Status` tag
(STD / KEEP / SELECT / SHUNT-REVIEW / REVIEW-HV / SPECIAL-KEEP). This doc records the
standardization rules and the **lines that still need a sourcing decision**.

Keyed to the as-built designators. Key ICs: **U12 = STM32H563ZIT6** (LQFP144);
**U54 = INA228** (panel-LED 9th monitor); **U55 = RT9742** panel-LED load switch;
**U45 = MCP1502-30E** (3.0 V VREF_3V0). All aggregated inputs are direct GPIO — there are
no MCP23017 expanders. Each part's fields are derived from its `Value` + circuit context.

## Standardization rules

| Class | Rule |
|---|---|
| **Resistors (general)** | Panasonic **ERJ-2RKF**, 0402, **1 %, 1/10 W**, AEC-Q200. One part per value (MPN generated deterministically from the value, E96). Consolidates the prior mix of duplicate MPNs per value. |
| **Resistors (sense)** | Milliohm shunts left as **SHUNT-REVIEW** — value is TBD per INA228 rail (project open item); 0.5 % 0805 once the rail current is fixed. |
| **Resistors (precision)** | 0.1 % dividers (OCXO) keep their TE **RQ73C** 0603 parts. |
| **Capacitors (MLCC)** | Murata **GRM** series. Verified MPN reused only where the value truly matches; 16 V X7R default for logic decoupling, 100 V X7T on the PoE/Rb high-voltage nodes, C0G for ≤ 1 nF and the OCXO Vc/loop-filter path. |
| **Value notation** | `0.1uF` (not 100nF), `10nF` (not 0.01uF), `µF ≥ 1`, voltage tag dropped into the spec, not the value. |
| **Footprints** | IPC metric names (`R_0402_1005Metric`, `C_0805_2012Metric`, …). |
| **Actives / ICs** | MPN = the part-number value where the schematic left the field blank (e.g. `INA228AQDGSRQ1`, `ZED-F9T-00B`). |

A `Status` column in the CSV tags every line: `STD` (standardized), `KEEP` /
`MPN-FROM-VALUE` (active with known PN), `SPECIAL-KEEP` (precision/power part kept),
`SHUNT-REVIEW`, `REVIEW-HV`, `SELECT` (needs a part chosen — full spec provided).

## Lines needing a sourcing decision

Status tally: **STD 460 · KEEP 126 · SELECT 34 · SHUNT-REVIEW 10 · REVIEW-HV 4 · SPECIAL-KEEP 2.**
A full CSV re-export is pending for the most recently added parts (e.g. D26 SMAJ15CA on RB_LOCK_IN).

### OCXO Vc-path caps — dielectric-critical (SELECT — top priority)
- **C98, C99, C101** currently 0.1 µF X7R 0402 on the OCXO steering/loop-filter path
  (PA4→`OCXO_VC`→OPA320 U36 buffer→`OCXO_V`→Y3.1). **Must be C0G/NP0 or film** — X7R
  microphonics FM-modulate the 10 MHz carrier. 0.1 µF C0G does **not** exist in 0402 → needs
  film or a **1210 C0G** (package/layout change). C98/C101 confirmed on the Vc path; C99 is
  the 1.65 V bias node (via 10 M — also move to C0G).
- **C109, C117** 22 µF on the OCXO sheet are confirmed **bulk** (X7R OK) — not the Vc path.

### 2 kV shield/chassis Y-caps — package (SELECT)
- **C1, C42, C186** 4700 pF **2 kV** shield-to-chassis Y-caps (RJ45 shield, USB shield, DE-9
  shield). The MLCC minimum case for 2 kV is 1808 — assign a 1808/1812 safety/AEC-Q200 HV Y-cap
  PN with the matching footprint.

### Rb buck OUTPUT caps — ≥ 50 V rating (SELECT — HIGH)
- **C125** (47 nF feedforward) and **C126, C127, C128** (47 µF X6S bulk) sit on `Net-(U44-IN+)`
  = the MIC28516 (U40) buck output = **VCC_RB**, which the digipot steers to a **24.45 V**
  pedestal (26 V OV-latch trip). These **must be rated ≥ 50 V** (X7S/X7T; C0G for the small C125)
  — X6S also loses most of its C at ~24 V DC bias. At 50 V + DC-bias derating a 47 µF will not fit
  1210 → expect 1210/1812 and/or split parts to hold effective bulk. The **input** caps C133–C136
  are 100 V X7T. KiCad VOLTAGE/Description fields are set to 50 V; PN selection pending.

### R264 PG7-divider resistor — verify MPN before ordering
- **R264** is the 174 kΩ upper leg of the `POE_PG`→PG7 divider (R264 174 k / R263 10 k → 2.93 V).
  MPN = **`ERJ-2RKF1743X`** ("RES 174 k 1% 1/10 W 0402"). Re-verify the ordered PN matches 174 kΩ
  before fab — a 1.21 kΩ part here would put ~48 V on PG7 (board-lethal).

### Caps on high-voltage nodes — confirm voltage rating (REVIEW-HV)
- **C6** 1 nF, **C7** 1 µF, **C9** 1 µF, **C11** 47 pF — sit on `VOUT_N` (PoE primary return).
  PNs assigned at 100 V; confirm the node's true working voltage before committing.

### PoE high-voltage caps (SELECT)
- **C8** 0.1 µF 100 V and **C10** 10 µF 100 V on `VOUT_P` — 0402/0805 undersized for 100 V;
  assign 1210/1812 PNs.

### Current-sense shunts — value TBD per INA228 rail (SHUNT-REVIEW)
| Ref | Value | INA228 rail (addr) |
|---|---|---|
| R30 | 150 m | U10 PoE-input (0x40) — in the series `VOUT_P`→`V_POE` feed |
| R106 | 15 m | U31 STM 3V3 (0x41) |
| R107 | 100 m | U32 5V_DISP (0x42) |
| R102 | 75 m | U30 main 3V3 (0x43) |
| R89 | 100 m | U26 antenna-bias (0x45) |
| R126 | 25 m | U37 OCXO (0x46) |
| R159 | 20 m | U44 Rb/VCC_RB (0x47) |
| R72 | 500 m | U23 GPS-VCC (0x4A) — consider 0.1 Ω (500 m drops ~60–75 mV) |
| R199 | 220 m | U54 panel-LED (0x4C) |
| R16 | 25 m | PD hot-swap sense (Q2/NCP1095) — **not** an INA228 shunt |

Pick 0.5 % metal-strip (or 4-terminal Kelvin) per the chosen full-scale current of each rail.

### Power / precision resistors (SPECIAL-KEEP — verify MPN)
- **R77** 3.3 Ω 1 % 0.25 W 1206 (antenna-bias bounding) — needs an orderable PN (ERJ-8ENF3R30V suggested).
- **R129** 12.1 k, **R130** 3.83 k — 0.1 % TE RQ73C 0603 (OCXO divider), kept.

### Inductors (SELECT — size Isat/current)
- **L2** 6.8 µH ≥12 A (main 5 V buck) · **L3, L6** 2.2 µH ~4 A (3V3 / OCXO bucks) · **L4, L5**
  2.2 µH (TPS61094 backup — size Isat for boost peak) · **L7** 40 µH (Rb high-Vout buck).

### Diodes / comparator grade (SELECT)
- **D16, D20, D21** 3.3 V Zener (package blank as-built — BZX84C3V3 suggested); **D7** 1.5SMCxxA
  (pick clamp voltage) — assign orderable PNs.
- **U50** LTC6752**x**S5 — confirm speed grade (H vs I).

### Connectors (SELECT)
- **J2, J4, J10, J11, J13** 2.54 mm headers; **J6** DE-9 socket; **J7, J8, J9, J15** SMA coax —
  assign connector PNs.
