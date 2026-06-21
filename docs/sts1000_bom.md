# STS1000 BOM — Standardization & Sourcing Status

Authoritative parts data lives in the **KiCad symbol fields** (Value, Footprint,
Description, MANUFACTURER, MANUFACTURER_PART_NUMBER, DIGIKEY_PART_NUMBER) and is exported
to `hardware/ntp-server-stm32/sts1000_bom.csv`. This doc records the standardization rules
and the **lines that still need a sourcing decision**.

## Why the BOM was rebuilt

The source BOM's `Value` field is authoritative, but `MANUFACTURER_PART_NUMBER`,
`Footprint`, and `Description` were heavily corrupted by copy-paste — e.g. the 4.7 µF 0805
part `C2012X7R1E475K125AB` was attached to 0.1 µF / 1 nF / 10 µF caps, and the 0.025 Ω
sense resistor `RL1206FR-070R025L` to 10 k / 100 k / 2 M resistors. Every part was
therefore **re-derived from `Value` + circuit context**, not from the corrupted fields.

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

### Caps on high-voltage nodes — confirm voltage rating (REVIEW-HV)
- **C6** 1 nF, **C7/C9** 1 µF, **C11** 47 pF — sit on `VOUT_N` (PoE primary return). Confirm
  working voltage; rate for the PoE line (likely 100 V) if they see it.

### OCXO caps — confirm dielectric (OCXO-C0G?)
- **C109, C117** 22 µF on the OCXO sheet — confirm these are bulk (X7R OK) and **not** on
  the Vc/loop-filter path, which must be **C0G/NP0 or film** (X7R microphonics FM the carrier).

### Current-sense shunts — value TBD per INA228 rail (SHUNT-REVIEW)
- **R16, R126** 25 m; **R72, R106** 500 m; **R89, R107** 100 m; **R30** 150 m; **R102** 75 m;
  **R159** 20 m. Pick 0.5 % 0805 (or 4-terminal) to match the chosen full-scale current of
  each INA228 (see `peripheral_map` / `software_spec` shunt open items).

### Power / precision resistors (SPECIAL-KEEP — verify MPN)
- **R77** 3.3 Ω 1 % 0.25 W (antenna-bias bounding) — needs an orderable 0805 PN.
- **R129** 12.1 k, **R130** 3.83 k — 0.1 % RQ73C, kept.

### Caps with no verified MPN yet (SELECT — spec given, pick a Murata/TDK equivalent)
- **0.1 µF 100 V** C8 · **10 µF 100 V** C10 · **0.022 µF 50 V** C2 · **0.47 µF 16 V** C49 ·
  **47 nF 16 V** C125 · **220 nF 16 V** C176 · **1 µF 16 V** (C18, C28, C40, C45, C47, C52,
  C57, C84, C85, C130, C143, C152, C157) · **22 µF 16 V** (C92–C97) · **470 pF C0G** C17 ·
  **47 pF C0G** (C141, C142, C146, C147) · **27 pF C0G** (C14, C16) · **12 pF C0G** (C29, C31).

### Diodes / inductors / connectors (SELECT)
- **D4** 12 V Zener, **D16/D20/D21** 3 V3 Zener, **D7** 1.5SMCxxA (pick clamp voltage) —
  need orderable PNs.
- **L4, L5** 2.2 µH (backup-supply inductors) — confirm current rating / PN.
- **J2** (1×4), **J4** (1×14), **J10** (1×2), **J11** (1×8) headers; **J6** DE-9; **J7/J8/J9**
  coaxial (SMA) — assign connector PNs.
