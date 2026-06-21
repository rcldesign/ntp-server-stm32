# STS1000 Schematic Error & Consistency Review

Findings from a netlist-driven review of the re-annotated schematic (2026-06). Each item
is rated **confidence** (High = verified in the netlist; Med = strong inference; Low =
worth checking). Items are deliberately conservative — design choices that look unusual but
are justified are listed under *Not-a-defect* so they aren't "fixed" by mistake.

---

## A. Connectivity / wiring errors (fix before layout)

### A1. `INA_ALERT_N` is a 1-pin dangling net — MCU can't see the INA alert wire-OR  **(High)**
- `INA_ALERT_N` has a **single node**: `U12.80` (STM32 **PC12**). Nothing else is on it.
- The aggregated INA228 alert is `INA_ALERT_INT_N` (`U54.15` INTB + pull-up `R199`); the 8
  INA228 `ALERT` pins feed `U54` GPIO inputs, whose INTB is `INA_ALERT_INT_N`.
- So the MCU's intended INA-alert input (PC12) is on the wrong net and is **not connected**
  to the aggregate. **Fix:** relabel PC12's net `INA_ALERT_N` → `INA_ALERT_INT_N` (or wire
  the two together). This matches the historical intent (`peripheral_map`: INA_ALERT_INT_N
  = PC12).

### A2. USB-C VBUS sense divider is mis-wired — ~5 V into a 3.3 V ADC pin  **(High)**
- `R68` (100 k): `USB_VBUS` ↔ `USB_VBUS_SENSE`. `R67` (121 k): `USB_VBUS` ↔ **GND**.
- The sense node `USB_VBUS_SENSE` (→ `U12` **PE2**, + `C43` 100 nF) therefore has **no
  resistive leg to GND** — only a 100 k series feed from VBUS. DC at PE2 ≈ VBUS (~5 V),
  exceeding the 3.3 V ADC max; `R67` is just a VBUS→GND bleeder doing nothing useful.
- **Fix:** move `R67`'s grounded end so the divider is `VBUS –R68(100k)→ SENSE –R67(121k)→
  GND` (Vsense ≈ 5 V·121/221 ≈ 2.74 V). I.e. `R67` should be `USB_VBUS_SENSE`↔`GND`, not
  `USB_VBUS`↔`GND`.

### A3. Duplicate pull-up on `DISP_TOUCH_INT` (10 k ∥ 10 k = 5 k)  **(High)**
- Both `R197` (I²C-GPIO sheet) and `R208` (SPI-peripherals sheet) are 10 k from `3V3_STM`
  to `DISP_TOUCH_INT`. This is exactly the condition the open-item warned against ("R208
  must *replace* the old pull-up, not add to it"). **Fix:** remove one (keep a single 10 k).

### A4. `VREF_4V096` (U45 MCP1502-40 reference) has no output capacitor  **(Med)**
- No capacitor sits on the `VREF_4V096` net. The MCP1502 datasheet requires an output
  cap (typ. 1 µF) for stability/noise. **Fix:** add the datasheet output cap (and any input
  cap) at `U45`.

---

## B. ESD-protection consistency (the house style is TI TPD4E0xx flow-through arrays)

House pattern, all consistent and correct:
- Ethernet diff pairs (J1) → `U1` TPD4EUSB30; USB-C DM/DP/VBUS (J5) → `U18` TPD4E05U06;
  display flex (J4) → `U15`/`U16`/`U17`; panel buttons + tamper (J11/J10) → `U19`/`U20`.

### B1. GPS antenna SMA (J7) has **no** ESD/surge device  **(High — and contradicts our own rule)**
- `GPS_RF_IN`: `J7` → `L1` (bias-T) → `U21` RF_IN, with **nothing** clamping the port.
- Both *other* coax ports get protection: `J8` `10MHz_RF_IN` → `D8` SZESD7410, `J9`
  `10MHz_RF_OUT` → `D9` SZESD7410.
- The GPS SMA is the **most** exposed port (outdoor antenna run, 1.5 GHz RF **and** 5 V
  bias) and `CLAUDE.md` explicitly requires a polymer ESD suppressor **plus** a coaxial
  gas-discharge arrestor here. **Fix:** add the polymer-ESD + GDT protection at `J7` (a
  plain clock-line ESD part won't meet the Cj ≤ 0.2–0.3 pF RF and V_RWM ≥ 6.5 V bias
  requirements simultaneously).

### B2. USB-C CC1/CC2 lines (J5) are unprotected  **(Med)**
- `Net-(J5-CC1)`/`CC2` go only to `R64`/`R66`. CC pins are user-exposed on the receptacle;
  `U18` (TPD4E05U06, 6-ch) has spare channels — route CC1/CC2 through it, or add a 2-ch
  array, for consistency with the rest of the connector ESD.

### Not-a-defect (justified deviations, do **not** "normalize")
- **RS-232 DE-9 (J6)** uses discrete `D13`/`D14` **SMAJ15CA** (15 V) on `RB_RS232_TX/RX`
  and `U47` PESD15VL2BT on the post-transceiver `_P` lines — correct, because ±15 V RS-232
  swing would forward-clamp a 5 V TPD array. Keep the discretes here.
- **10 MHz coax (J8/J9)** uses single-line `SZESD7410` rather than a multi-line TPD array —
  correct for a single low-capacitance RF/clock line.

---

## C. Doc-vs-schematic / value discrepancies (verify against design intent)

| # | Item | Conf |
|---|---|---|
| C1 | **MCP23017 addresses swapped roles**: netlist straps give `U54`=**0x21** (A0=3V3_STM, buttons/INA-alert/touch, INTA/INTB separate → MIRROR=0) and `U55`=**0x20** (A0=GND, power-good, INTA+INTB tied → MIRROR=1). Earlier docs had the fault aggregator at 0x20/MIRROR=1. Docs updated to as-built; confirm this is intended. | High |
| C2 | **Rb digipot `U43` VDD on `3V3`**, but `vcc_rb` doc described +5 V. Confirm pot supply rail. | Med |
| C3 | **Rb buck FREQ vs EN**: `R132`/`R133` (200 k/100 k) divide `VOUT_P`→`U40A` **FREQ** pin, but the doc describes an EN sense divider. EN (pin 24) ties directly to `RB_PSU_PWR_EN`. Reconcile. | Med |
| C4 | **`V_DISP_EN_FAULT` on `U55` GPB1** (pin 26), not GPB2 (doc says GPB2; GPB2 is unconnected). | High |
| C5 | **`U54` GPA7 = `DISP_TOUCH_INT`**, not `BUTTON_8` → the panel is **7 buttons + touch-INT**, not 8 buttons. Update the button-mask prose. | High |
| C6 | **L1** antenna bias-T inductor value: BOM `47 nH` vs doc `56 nH`. | Med |
| C7 | **Q11** antenna pass device: symbol library is `BCP51` but Value/intended part is `NSS40300MZ4T1G` — symbol/value mismatch; confirm the footprint matches the intended part. | Med |
| C8 | **PoE clamp topology vs cited reference**: docs cite `SMBJ58A` (per NCP1095 datasheet); as-built VPP clamp is `D2`/`D3` (DF6A6.8FUT1G arrays) + `C8` 0.1 µF/100 V, `C10` 10 µF/100 V. Reconcile the citation with the as-built. | Low |

---

## D. Reliability / performance suggestions

1. **A1/A2/A3 first** — those are functional defects (alert blind, VBUS over-range, doubled
   pull-up), not just cleanup.
2. **Finalize the INA228 shunt values** (`R16/R30/R72/R89/R102/R106/R107/R126/R159`, all
   left as `SHUNT-REVIEW`). Pick each 4-terminal/0.5 % shunt to the chosen full-scale
   current per rail so the INA228 ADCRANGE/LSB is optimal; today they carry a copy-pasted
   0.15 Ω part that doesn't match their values. (See `sts1000_bom.md`.)
3. **Confirm VOUT_N caps' voltage rating** (`C6` 1 nF, `C7`/`C9` 1 µF, `C11` 47 pF) — they
   sit on the PoE primary return; rate for the line if they can see it (REVIEW-HV).
4. **OCXO loop-filter / Vc-path caps must be C0G/NP0 or film** — verify `C109`/`C117`
   (22 µF, OCXO sheet) are bulk only and **not** on the Vc node; any cap on `OCXO_VC` must
   be C0G/film (X7R microphonics FM the carrier).
5. **GPS SMA protection (B1)** is both a consistency and a field-reliability item — the
   outdoor antenna run is the highest-energy ESD/surge path in the system.
6. **Standardize the remaining sourcing flags** in `sts1000_bom.md` (the `SELECT` caps,
   12 V/3 V3 zeners `D4`/`D16`/`D20`/`D21`, `D7` clamp voltage, header PNs) so the BOM is
   fully orderable.
7. **External WDT `WDT_KICK` (WDI) has no defined-level resistor** — if the watchdog window
   requires a known idle level on WDI, add the pull; otherwise drop the doc's claim.
