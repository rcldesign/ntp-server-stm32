# STS1000 BOM — Standardization & Sourcing Status

Authoritative parts data lives in the **KiCad symbol fields** and is exported to
**`hardware/ntp-server-stm32/sts1000_bom.csv`** — 184 grouped lines covering **635 placed
components** (649 as-built symbols − 14 mounting holes). *(The former `sts1000_bom_v2.csv` and
`ntp-server-stm32.csv` exports are deleted; `sts1000_bom.csv` is the only BOM file.)*

CSV columns: `Reference, Qty, Value, DNP, Exclude from BOM, Exclude from Board, Footprint,
Datasheet, DIGIKEY_PART_NUMBER, MANUFACTURER, MANUFACTURER_PART_NUMBER, Description`.
**Every line now carries both a manufacturer PN and a DigiKey PN** — the earlier
`Status` / `VOLTAGE` / `DIELECTRIC` / `TOLERANCE` / `POWER` tag columns and the
`STD / KEEP / SELECT / SHUNT-REVIEW / REVIEW-HV / SPECIAL-KEEP` workflow are retired; the
voltage/dielectric/tolerance/power spec now lives in the `Description` text of each line.

Keyed to the as-built designators. Key ICs: **U12 = STM32H563ZIT6** (LQFP144);
**U54 = INA228** (panel-LED 9th monitor); **U55 = RT9742** panel-LED load switch;
**U45 = MCP1502-30E** (3.0 V VREF_3V0); **U62 = MX25L25645GM2I-08G** (Macronix SPI-NOR,
sourcing-policy compliant). All aggregated inputs are direct GPIO — there are no MCP23017 expanders.

## Standardization rules

| Class | Rule |
|---|---|
| **Resistors (general)** | Panasonic **ERJ-2RKF**, 0402, **1 %, 1/10 W**, AEC-Q200. One part per value (MPN generated deterministically from the value, E96). KOA **RK73H1E** appears where a value predates the consolidation. Non-E96 values were normalized (PHY straps 5 k → **4.99 k**). |
| **Resistors (anti-surge)** | Panasonic **ERJ-U02** on the PoE-side high-voltage dividers (R2, R4, R6, R101, R162, R103, R163). |
| **Resistors (sense)** | **Vishay Dale WSL metal-strip, 1 %, AEC-Q200** — 1206 (0.25 W) for the eight low-current INA228 rails, **2512 (1 W)** for R159. The PD hot-swap RSNS R16 is an **Ohmite MCS1632** 1206/1 W. Values are **final** — see below. |
| **Resistors (precision)** | 0.1 % dividers keep their TE **RQ73C** 0603 parts (R129/R130, OCXO LDO). R147 is a Panasonic **ERJ-H2RD** 0.5 % (Rb FB). |
| **Capacitors (MLCC)** | Murata **GCM/GRM/GRT** and KEMET **C-series** AEC-Q200. 16 V X7R default for logic decoupling, **100 V** on the PoE primary nodes, **50 V** on the VCC_RB output nodes, **C0G** for ≤ 1 nF, the OCXO Vc path, and the RF DC-block. |
| **Value notation** | `0.1uF` (not 100nF), `10nF` (not 0.01uF), `µF ≥ 1`; milliohm shunts as `25m`/`7m`. Voltage/dielectric live in `Description`, not `Value`. |
| **Footprints** | Vendor/package names as exported by the symbol library (`1206 (3216 Metric)`, `0402 (1005 Metric)`, `TSOT-23-5`, …). |
| **Actives / ICs** | MPN = the part-number value where the schematic left the field blank (e.g. `INA228AQDGSRQ1`, `ZED-F9T-00B`). |

## Current-sense shunts — final values

All nine INA228 monitors run **ADCRANGE=1** (±40.96 mV shunt full-scale). Sizing math and the
per-rail derivation live in `sts1000_hardware_design_reference.md §2.1.1`; firmware register
constants in `sts1000_firmware_hardware_interface.md §4.2`.

| Ref | Value | Package / rating | MPN | INA228 rail (addr) |
|---|---|---|---|---|
| R30 | **25 mΩ** | 1206, 0.25 W | WSL1206R0250FEA | U10 PoE-input (0x40) — in the series `VOUT_P`→`V_POE` feed |
| R106 | **100 mΩ** | 1206, 0.25 W | WSL1206R1000FEA | U31 STM 3V3 (0x41) — Kelvin split |
| R107 | **100 mΩ** | 1206, 0.25 W | WSL1206R1000FEA | U32 5V_DISP (0x42) |
| R102 | **50 mΩ** | 1206, 0.25 W | WSL1206R0500FEA | U30 main 3V3 (0x43) |
| R89 | **150 mΩ** | 1206, 0.25 W | WSL1206R1500FEA | U26 antenna-bias (0x45) |
| R126 | **25 mΩ** | 1206, 0.25 W | WSL1206R0250FEA | U37 OCXO (0x46) |
| R159 | **7 mΩ** | **2512, 1 W** | WSL25127L000FEA | U44 Rb/VCC_RB (0x47) |
| R72 | **75 mΩ** | 1206, 0.25 W | WSL1206R0750FEA | U23 GPS-VCC (0x4A) |
| R199 | **150 mΩ** | 1206, 0.25 W | WSL1206R1500FEA | U54 panel-LED (0x4C) |
| R16 | 25 mΩ | 1206, **1 W** | MCS1632R025DER (Ohmite) | PD hot-swap RSNS (Q2/NCP1095) — **not** an INA228 shunt |

Purchasing note: **R16 and R30/R126 share the value `25m` on one grouped CSV line but are two
different parts** (Ohmite 1 W vs Vishay 0.25 W) — split the line at order time.

## Grouped lines carrying more than one MPN

Deliberate voltage-grade splits inside a single `Value`. Each must be split at purchase; the
reference designators tell you which is which:

| Line | Value | Parts |
|---|---|---|
| C3, C6, C62, C200, C201 | 1 nF | KEMET `C0402C102K1GECAUTO` (100 V, PoE nodes) + `C0402C102J5GECAUTO` (50 V) |
| C11, C141, C142, C146, C147 | 47 pF | Vishay `GA0402A470JXBAP31G` (100 V, PoE) + Murata `GCM1555C1H470FA16D` (50 V C0G, RS-232 line shunts) |
| C63, C106, C110, C116, C119, C148, C151, C155, C190, C192, C206 | 10 nF | Murata `GCM155R71H103KA55D` (50 V) + `GRT155R71C103KE01J` (16 V) |
| R16, R30, R126 | 25 mΩ | Ohmite `MCS1632R025DER` (1 W) + Vishay `WSL1206R0250FEA` (0.25 W) |

*(Nexperia PNs such as `BC847W,135` and `74LVC1G34GW,125` contain a literal comma — those are
single parts, not split lines.)*

## Resolved sourcing items

Every line in the CSV now has an orderable manufacturer PN **and** a DigiKey PN. The following
previously-open selections are closed:

| Item | Resolution |
|---|---|
| OCXO Vc-path caps **C98, C99, C101** | KEMET **C1210C104J5GACAUTO** — 0.1 µF 50 V **C0G, 1210**. (0.1 µF C0G is unbuildable below 1210; no X7R substitution — microphonics FM-modulate the carrier.) |
| **C37** (VREF+) | 1 nF → **0.1 µF C0G 1210** (same part), at U12.32; C38 2.2 µF stays behind R48 50 Ω. |
| 2 kV shield Y-caps **C1, C42, C186** | KEMET **C1812C472KGRACAUTO** — 4.7 nF 2 kV X7R **1812**. |
| Rb buck output caps **C126, C127, C128** | Murata **KCM55WR71H476MH13L** — 47 µF **50 V** X7R, stacked SMD 2 J-lead (47 µF/50 V does not fit 1210). |
| **C125** | **GCM155R71H473KE02J** — 47 nF **50 V** X7R 0402. Meets the ≥50 V rule; C0G would hold value better under the 24 V bias (optional). |
| PoE HV caps **C6–C11** | All **100 V**: C6 1 nF C0G, C7/C9 1 µF 0805, C8 0.1 µF 0603 `GCJ188R72A104KA01D`, C10 10 µF `KCM55QR72A106KH01L`, C11 47 pF `GA0402A470JXBAP31G`. |
| **All nine INA228 shunts** | Values final; Vishay WSL parts assigned (table above). |
| **L7** (Rb buck) | 40 µH → **39 µH**, Würth **7447709390** (4.1 A, 56 mΩ). |
| PHY straps **R49, R228, R229, R230** | Non-E96 5 k → **4.99 k** `ERJ-2RKF4991X`; same LAN8742A strap window. |
| Supercap VCHG **R112, R113** | 4.75 k → **13.0 k** `ERJ-2RKF1302X` → 2.7 V termination. |
| **R264** (PG7 divider) | 174 k 0402 → **`ERJ-8ENF1623V`** 162 kΩ **1206 / 0.25 W / 200 V**. Re-verify the ordered PN before fab — a 1.21 kΩ part here would put ~48 V on PG7 (board-lethal). |
| **R2** (PFI divider) | 1.2 M 0402 → **`ERJ-8ENF1204V`** 1206 / 0.25 W / **200 V** (was a 50 V-rated 0402 carrying 52–55 V). |
| **R20** (POE_PG pull-up) | 10 k 0402 → **`ERJ-14NF1002U`** **1210 / 0.5 W / 200 V** — carries 292 mW at 54 V (325 mW at 57 V) and the full bus voltage whenever the pass FET is on and PGO is low. |
| **R77** (antenna foldback set) | 0402 / 0.1 W → **`RMCF1206FT3R30`** 1206 / **0.25 W**; 109 mW at the 182 mA clamp = 44 % of rating. *(Stackpole general-purpose thick film — `ERJ-8ENF3R30V` would keep the AEC-Q200 ERJ-8 family used elsewhere.)* |
| **R159** footprint | Library name corrected `2515 (6332 Metric)` → **`2512 (6332 Metric)`**, matching the WSL2512 part. |
| Zeners **D16, D20, D21** | `BZX84C3V3LT1G` (onsemi, SOT-23). |
| **D7** | `1.5SMCJ28A` (Diotec), 28 V standoff on VCC_RB_G. |
| **U50** | `LTC6752IS5#TRMPBF` — **I** grade selected. |
| Connectors | J1 Bel/Stewart `0826-1X1T-HS-F`; J2/J10/J16 On Shore `OSTVN0*A150`; J3 GCT `BG055-06A-…`; J5 GCT `USB4120-03-C`; J6 EDAC `622-009-260-033`; J7/J8/J9/J15 Samtec `SMA-J-P-H-RA-TH1`; J17 Phoenix `1787179`. |
| Inductors | L2 `SRP1265A-6R8M` 6.8 µH; L3/L4/L5/L6 `DR73-2R2-R` 2.2 µH 4.15 A; L7 `7447709390` 39 µH. |

## Remaining BOM / footprint items

- **Library tables are fixed; footprint *assignment* is the remaining layout task.** `fp-lib-table` now exists (9 nicknames → `hardware/libraries/sts1000.pretty`, 54 footprints) and `sym-lib-table` is portable (`${KIPRJMOD}` relative, 38 entries, all resolving). Of 648 symbols, **41** now carry a resolvable `library:footprint`; **571** still hold a vendor package *description* in the Footprint property (`0402 (1005 Metric)`, `SOT-323`, …) and **36** are empty (J1, J2, J6–J10, J15–J17, K1, K2, L2–L7, U53, U71, Y1, Y2 + mounting holes H1–H14). Two `Library:` refs point at footprints that do not exist in the repo: **J3** `GCT_BG055-06A-1-0450-0530-0350-L-D` and **D5** `LED_IN-P55TATRGB`.
  The `Footprint` column of `sts1000_bom.csv` is therefore a *packaging* column, not a layout input.
- **R159 footprint name reads `2515 (6332 Metric)`.** 6332 metric = **2512 imperial**, and the part
  (`WSL25127L000FEA`) and Description both say 2512 — the library name is the only inconsistency.
  Confirm the land pattern is a true 2512 before layout.
- **0402 working-voltage margin on the PoE dividers** (R2, R20, R264) — see the design review; the
  ERJ-2RK/ERJ-U02 0402 element rating is 50 V and these nodes sit at 52–55 V.
