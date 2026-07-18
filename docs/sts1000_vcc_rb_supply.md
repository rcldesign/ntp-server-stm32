# STS1000 — Variable Clock Supply (VCC_RB, 5–24 V)

Digitally controlled buck rail powering the external 10 MHz clock standard.
Nominal 15 V @ 1.5 A; full range 5–24 V. Setpoint is commanded over SPI by a
digital potentiometer that drives a ratiometric reference into the buck feedback
node through a buffered fixed resistor. Built on the MIC28516 (U40A) running as a
standard adaptive on-time synchronous buck; output voltage is set by the FB
divider plus the injected control voltage.

---

## 1. Signal flow

```
VREF_3V0 ──► MCP41U83 (B term) ──► wiper W ──► OPA320 follower ──► VCTRL ──► Rctrl ──► FB
                                                                                       ▲
SW ─► L7 ─► buck-out ─► R_shunt(R159) ─► VCC_RB ─► [Q25 disconnect §6a] ─► VCC_RB_G ─► J6 (FE)
              │                                                         R1 (VCC_RB→FB) │
              ├─ R1 (R147) ─────────────────────────────────────────────────────────┤
              └─ Cff (C125) across R1                        R2 (FB→GND), R144/C123 (SW→FB)
INA228 (U44) senses across R159 (VCC_RB I/V); TVS (D7) clamps VCC_RB_G at the connector.
```

FB is a 5-way node: R1 (R147), R2 (R148), RINJ+CINJ (R144/C123), Cff (C125), Rctrl (R134).
The loop holds FB = 0.600 V; VOUT follows the divider and the injected VCTRL.

---

## 2. Bill of materials

### Control front end
| Ref | Part | Value / spec | Function |
|---|---|---|---|
| U43 | MCP41U83T-503E/ST | 50 kΩ, 1024-pos, 10-bit, 14-TSSOP | Digital pot, potentiometer mode — output setpoint. **VDD = 3V3** (native 2.7–5.5 V) |
| U42 | OPA320AIDBVT | RRIO, SOT-23-5 | Unity buffer for wiper → VCTRL (loads FB, not the pot string). V+ = 5 V |
| U45 | **MCP1502T-30E/CHY** | **3.0 V** series reference | `VREF_3V0` (scales VOUT; precision/tempco critical). **nSHDN gated by Q19/Q20** off `RB_PWR_EN` (§3/§4) — not always-on |
| R134 | resistor | **3.01 kΩ 1%** | Rctrl: control injection, U42 OUT → FB |
| C182 | capacitor | 0.1 µF | `VREF_3V0` reference output cap (as-built; on the VREF_3V0 net) |
| C121 | capacitor | 0.1 µF | U43 VDD bypass (on 3V3) |
| C122 | capacitor | 0.1 µF | U42 supply bypass (on 5V) |

### Output filter + FB network
| Ref | Part | Value / spec | Function |
|---|---|---|---|
| L7 | inductor | 40 µH, Isat ≥3 A, low DCR | Output inductor, SW → VCC_RB |
| R147 | resistor | 20.0 kΩ 1% | R1: VCC_RB → FB (divider top) |
| R148 | resistor | **604 Ω 1%** | R2: FB → GND (divider bottom; sets **24.45 V pedestal**) |
| R144 | resistor | 10.7 kΩ 1% *(bench-trim)* | RINJ: SW → FB ripple injection |
| C123 | capacitor | 0.1 µF, X7R, ≥100 V | CINJ: series with R144 (sees full SW swing) |
| C125 | capacitor | 47 nF, C0G, ≥50 V | Cff: feed-forward across R147. Rating set by the VCC_RB node (≥50 V; see §6 sourcing item). |
| C126, C127, C128 | capacitor | 47 µF, X7R, **≥63 V** each | COUT bulk (rating set by TVS clamp; derate for DC bias). ≥50 V minimum, ≥63 V target; see §6 sourcing item. |

### Protection + monitor
| Ref | Part | Value / spec | Function |
|---|---|---|---|
| D7 | 1.5SMCJ28A | 28 V standoff, uni, SMC | transient/ESD clamp on **VCC_RB_G** at clock connector (J6, load side of Q25) |
| D26 | **SMAJ15CA** | 15 V standoff, bidir, SMA | TVS on `RB_LOCK_IN` at the FE connector (D26.2 = RB_LOCK_IN, D26.1 = GND); surge-clamps the FE lock input like its RS-232 siblings (15 V standoff / 24.4 V clamp) |
| U44 | INA228AQDGSRQ1 | I²C addr 0x47 | VCC_RB voltage + current monitor (VS/VDD on 3V3); ALERT open-drain has **R265 10 kΩ pull-up to 3V3_STM** (matches the sibling INA228 alerts) |
| R_shunt (R159) | shunt | **20 mΩ** (ADCRANGE=1, ±2 A FS — see notes) | VCC_RB current-sense element (buck-out → R159 → VCC_RB) |
| C131 | capacitor | 0.1 µF | U44 supply bypass (on 3V3) |

### Overvoltage latch (see §4)
| Ref | Part | Value / spec | Function |
|---|---|---|---|
| U41A | LMV393 (½) | +5 V supply | OV comparator (detector) |
| R138 | resistor | 120 kΩ 1% | Sense divider top: VCC_RB → comparator (+) |
| R139 | resistor | 10.0 kΩ 1% | Sense divider bottom → GND (ratio 0.0769) |
| R136 | resistor | **4.99 kΩ 1%** | Ref divider top: `VREF_3V0` → comparator (−) |
| R137 | resistor | 10.0 kΩ 1% | Ref divider bottom → GND (3.0·10/(4.99+10) → 2.00 V) |
| C120* | capacitor | 0.1 µF | Comparator (−) / ref-node bypass |
| R141 | resistor | 1 MΩ | Hysteresis: comparator OUT → (+) |
| R142 | resistor | 47 kΩ | Open-drain pull-up on comparator OUT (D11 anode) to +5 V |
| D11 | BAS16W | — | Isolates comparator from latch (anode = OUT, cathode = Q16 base node) |
| Q16 | BC847W (NPN) | — | Latch lower half: collector → R149/R150, emitter → GND |
| Q17 | BC857W (PNP) | — | Latch upper half / OV flag: emitter → +5 V, base ← R150, collector → R151/R153/R155 |
| Q18 | BC847W (NPN) | — | EN disable actuator: collector → RB_PSU_PWR_EN, emitter → GND, base ← R155 |
| Q15 | BC847W (NPN) | — | Latch reset: collector → Q16 base node, emitter → GND, base ← R145 |
| R152 | resistor | 100 kΩ | Q16 base pulldown (Q16 base node) |
| R149 | resistor | 10 kΩ | Q16 collector pull-up to +5 V |
| R150 | resistor | 10 kΩ | Q16 collector → Q17 base |
| R151 | resistor | 4.7 kΩ | Regenerative hold: Q17 collector → Q16 base node |
| R153 / R154 | resistor | 18 k / 33 k | RB_OV_DET divider off Q17 collector |
| R155 | resistor | 10 kΩ | Q17 collector → Q18 base |
| R156 | resistor | 4.7 kΩ | Series isolation: RB_PWR_EN (GPIO) → RB_PSU_PWR_EN (SMPS EN) |
| R145 | resistor | 10 kΩ | RB_OV_RESET → Q15 base |
| R146 | resistor | 100 kΩ | Q15 base pulldown |

*C120 is the OV ref-node bypass (the MCP41U83 has no EXT_CAP pin).

### Buck support (standard MIC28516 config)
| Ref | Value | Function |
|---|---|---|
| C119 | 10 nF | SS — soft-start cap |
| C124 | 0.1 µF | BST — bootstrap cap (SW↔BST) |
| R140 | 2.21 kΩ | ILIM / RCL — current-limit set |
| R143 | 10 kΩ | PG pull-up to 3V3 (RB_PSU_PG) |
| R132 / R133 | 200 k / 100 k | EN sense divider, VOUT_P → EN |
| Q19, Q20 | BC847W | RB_PWR_EN gate / level-shift (power-up interlock) |
| R157, R158, R160, R161 | 10 kΩ | EN gate network |

---

## 3. U43 (MCP41U83T-503E/ST) connections

**Pin-number assignment is TBD** — fill from the 14-TSSOP pinout. The logical
connections below are fixed by the design; only the pin numbers are pending.

| Signal / terminal | Net | Notes |
|---|---|---|
| VDD | **3V3** | Native 2.7–5.5 V; runs on always-on 3V3 (C121 0.1 µF bypass) |
| VSS / DGND | GND | Single-supply, VSS = DGND = 0 V |
| **SPI2C** (interface select) | **DGND** | Straps the part to **SPI** mode |
| Terminal A (P0A) | **GND** | Low end of divider |
| Terminal B (P0B) | **VREF_3V0** | High end — code 0 parks wiper here (safe-low) |
| Wiper W | U42 +IN | Wiper → buffer |
| SDI/MOSI | SPI_MOSI | |
| SDO/MISO | SPI_MISO | Add pull-up only if datasheet requires for readback |
| SCK | SPI_SCK | ≤20 MHz |
| CS | SPI_DPOT_CS (PD2) | Chip select, active low |
| A0 / A1 | **TBD** | I²C address pins; in SPI mode tie per datasheet (don't-care vs. defined level — confirm) |

Terminal orientation (A = GND, B = VREF, code 0 → wiper at B = VREF → **min VOUT** with
the new transfer function, since VOUT falls as VCTRL rises) preserves the safe-low
power-up direction. Confirm the MCP41U83 wiper code 0 connects to terminal B (safety-
critical — §6) when finalizing.

### Other active devices
- **U42 OPA320:** V+ → **+5 V** (C122 bypass), V− → GND, +IN ← U43 W, −IN ↔ OUT (unity follower), OUT = VCTRL → R134.
- **U45 MCP1502 (3.0 V):** VDD → +5 V, OUT = `VREF_3V0` (C182), **nSHDN gated by the Q19/Q20 pair off `RB_PWR_EN`** — the reference (and thus the digipot string) is dead until `RB_PWR_EN` is asserted, so pre-enable VCTRL = 0 forces the buck to its 24.45 V pedestal only if the buck itself is enabled (it is not; RB_PSU_PWR_EN is separately gated).
- **U44 INA228 (0x47):** IN+/IN− across R159 (20 mΩ) — IN+ = buck-out, IN− = VCC_RB; VBUS senses VCC_RB; I²C_SCL/SDA; ALERT (open-drain) → INA_ALERT_VCC_RB (PG15, polled) with **R265 10 kΩ pull-up → 3V3_STM**; VS/VDD → **3V3** (always-on), C131 bypass.

---

## 4. Overvoltage latch

Standalone hardware OV shutdown protecting the external clock (FE-5680A) against a
runaway VCC_RB (e.g. open-FB fault). Comparator detects; a regenerative BJT pair
latches and clamps the SMPS enable; firmware resets it. Independent of VCC_RB once
tripped (held from the always-on +5 V rail), so it latches off rather than hiccuping.

### Threshold ladder
```
24.45 V (max commanded) < 26.0 V (OV trip) < 28 V (TVS standoff) < ~31 V (TVS V_BR) < ~45 V (TVS clamp)
```
The latch fires at 26 V — before the TVS conducts — so the load is protected at the
trip point, not at the 45 V clamp.

### Detector (U41A, spare LMV393 half, on +5 V)
- Non-inverting input: VCC_RB → R138 (120 k) → (+) → R139 (10 k) → GND. Divider ratio 0.0769.
- Inverting input: `VREF_3V0` → R136 (**4.99 k**) → (−) → R137 (10 k) → GND → 2.00 V; C120 bypass on the ref node.
- Trip when sense > 2.00 V → VCC_RB > 26.0 V.  (RB_OV_DET tap = 5·33/51 ≈ **3.24 V** in fault, PE3-safe.)
- Hysteresis R141 (1 M) OUT → (+). R142 (47 k) is the open-drain pull-up, on the **D11 anode** node.

### Latch + actuator
- Comparator OUT → **D11 (BAS16W)** → Q16 base node (D11 isolates the comparator so collapse of VCC_RB cannot re-arm the latch).
- Q16 base node = D11 cathode + R152 (100 k → GND) + R151 bottom + Q15 collector.
- Q16 (NPN): collector → R149 (10 k → +5 V) and R150 (10 k → Q17 base); emitter → GND.
- Q17 (PNP): emitter → +5 V; base ← R150; collector → R151 (4.7 k, back to Q16 base node = regenerative hold) + R153 (18 k) + R155 (10 k).
- Q18 (NPN): base ← R155 (from Q17 collector node); collector → RB_PSU_PWR_EN (SMPS-side of R156); emitter → GND. This is the EN-disable actuator.
- RB_OV_DET: Q17 collector → R153 (18 k) → R154 (33 k) → GND; tap = RB_OV_DET (MCU status).
- Reset: RB_OV_RESET → R145 (10 k) → Q15 base; R146 (100 k) base pulldown; Q15 collector → Q16 base node.
- Enable path: STM32 RB_PWR_EN → R156 (4.7 k) → RB_PSU_PWR_EN → SMPS EN pin (Q18 collector taps the SMPS side).

### State logic
| Condition | Comparator | Q16 | Q17 | Q18 | EN pin | RB_OV_DET |
|---|---|---|---|---|---|---|
| VCC_RB good (≤24.45 V) | low (D11 off) | off | off | off | ~3.3 V (enabled) | low |
| VCC_RB > 26 V (fault) | high (via D11) | on | on | on | ~0.1 V (disabled) | high |
| Latched, VCC_RB collapsed | low again | **held on** by R151 | **held on** | on | ~0.1 V | high |
| RB_OV_RESET pulse | — | forced off (Q15) | off | off | released | low |

Q17 on is simultaneously the OV flag (RB_OV_DET) and the latch hold (R151 feeds Q16
base). Once Q17/Q16 are on, R151 (4.7 k) sustains the base independent of the
comparator, so the latch holds with VCC_RB at 0 V. Q18 saturates hard because its
base is driven from the Q17-collector node (~3–4 V in fault), not the ~0.7 V Q16-base
node — guarantees EN < 0.6 V.

### STM32 pins
| Signal | Dir | Pin | Config |
|---|---|---|---|
| RB_OV_RESET | out → Q15 base (R145) | **PD3** | GPIO push-pull, default LOW; pulse HIGH ≥10 µs to clear |
| RB_OV_DET | in ← Q17 collector divider | **PE3** | GPIO input, polled (no EXTI) |

Both are GPIO/polled — compliant with the EXTI2/3/4 restriction on PD3/PE3/PE4.
No interrupt needed: OV disable is hardware-automatic and instant; the MCU only
logs/recovers.

### EN threshold check (SMPS: ≥1.6 V enable, ≤0.6 V disable)
| State | EN pin | Margin |
|---|---|---|
| Enabled (Q18 off) | ~3.3 V | +1.7 V over enable threshold |
| OV-latched (Q18 sat) | ~0.1 V | −0.5 V under disable threshold |
| Commanded off (GPIO low, Q18 off) | 0 V (through R156) | ✓ |

---


## 5. Control transfer function

FB summing node, V_FB = 0.600 V, R1 = 20.0 k, R2 = **604 Ω**, Rctrl = **3.01 k**
(R1/R2 = 33.11, R1/Rctrl = 6.645):

```
VOUT = 0.6·(1 + R1/R2 + R1/Rctrl) − (R1/Rctrl)·VCTRL
     = 24.45 − 6.645·VCTRL          (VCTRL = 0…3.0 V)
```

Buffered wiper, divider mode, B = `VREF_3V0` (**3.0 V**), A = GND:

```
VCTRL = ((1024 − D) / 1024) · 3.0        (D = RDAC code, 0…1023)
```

Combined (VOUT rises with code — code 0 = safe-low):

```
VOUT ≈ 4.52 + 0.01947·D
```

| Target | Code D | Note |
|---|---|---|
| Min (code 0) | 0 | **4.51 V** (wiper at B = VREF → VCTRL = 3.0 → min VOUT; safe-low) |
| 5 V | 25 | low end |
| 15 V (nominal) | 539 | VCTRL ≈ 1.42 V |
| 24 V | 1001 | |
| Max (code 1023) | 1023 | 24.43 V |
| **Power-up preset** | **TBD** | MCP41U83 power-on wiper value — confirm from datasheet (§6) |

LSB = 19.5 mV (10-bit over the 19.9 V span, 4.51–24.45 V). Monotonic; DNL ±1 LSB per datasheet.

> **Code-to-VOUT sense:** with the 3.0 V reference and Rctrl = 3.01 k, VOUT falls as VCTRL
> rises, so **code 0 (wiper at VREF, VCTRL = 3.0 V) is minimum VOUT (safe-low)** — the safe
> power-up direction. Firmware wiper-code lookups must be generated against the
> `VOUT ≈ 4.52 + 0.01947·D` relation.

---

## 6. Operating & firmware notes

**Supply.** U43 MCP41U83 runs natively on 2.7–5.5 V (single rail, VSS = DGND = 0 V),
powered from the always-on **3V3** rail (U43 VDD = 3V3); no dedicated pot supply rail is
needed. `VREF_3V0` (U45 MCP1502-30E) is the ladder reference and is **gated by Q19/Q20 off
`RB_PWR_EN`**, so the digipot string is dead until the Rb path is enabled.

**Interface select.** SPI2C pin **tied to DGND → SPI mode.** Confirm A0/A1 tie level
(I²C-address pins, function in SPI mode per datasheet).

**Potentiometer mode.** Configure the part as a potentiometer (all three terminals on
the ladder); buffered + ratiometric, so the 50 kΩ RAB absolute value and its tolerance
do not affect VOUT — only string current (~82 µA at VREF/RAB) and bandwidth. **TBD:**
exact mode/config register write — confirm from datasheet.

**Wiper write sequence — TBD (datasheet needed).** Confirm and document from the MCP41U83
sheet: (a) any config-lock / write-protect that must be cleared before the volatile wiper
register accepts writes;
(b) the volatile-wiper write opcode and frame format (byte count, bit positions for
the 10-bit value); (c) **CRC** default state — if CRC is on by default, either disable
it in software or supply valid CRC bytes or writes are rejected; (d) **SPI mode**
(CPOL/CPHA).

**Power-on wiper state — TBD (safety-critical).** Confirm the factory/erased power-on
wiper value (zero-scale, mid-scale, or last MTP-stored). This sets the rail's pre-
firmware voltage. MTP is rewritable (1,000 cycles), so a safe code can be stored and
updated freely — preferred over relying on a default. Until confirmed, treat the
EN-gating interlock below as mandatory.

**EN-gating interlock.** Keep **RB_PWR_EN deasserted** (buck off) until the wiper is
written to a safe code and U42 has settled. Optionally pre-store a safe code (e.g.
5 V → code 25, or nominal 15 V → code 539) to MTP so the power-on preset is safe regardless.
Note the **§6a disconnect gate (Q25)** is a second, independent interlock: `VCC_RB_G`
(and hence the FE) is dead until `RB_VCC_GATE` (PB1) is asserted.

**Protection layering.** **D7** (on `VCC_RB_G`) clamps fast transients/ESD only; it cannot limit the
24.45 V pedestal (that is the commanded maximum) and cannot guard the FE-5680A against
a slow open-FB runaway. The **§4 OV latch** (U41A, 26 V trip → clamps RB_PSU_PWR_EN,
latched) is the DC-overvoltage guard; the TVS only handles the sub-µs surge/ESD window.

**COUT rating — open BOM sourcing item.** The Rb buck **output** caps (`C125` feed-forward,
`C126/C127/C128` bulk) sit on `Net-(U44-IN+)` = the MIC28516 output = the **VCC_RB rail**,
which the digipot steers to a **24.45 V pedestal** (26 V OV trip). They must be rated
**≥50 V** (X7R/X7S bulk; C0G for the C125 feed-forward), ≥63 V preferred to match the input
side (C133–C136 are 100 V X7T) and the TVS clamp (≈45 V). At 50 V + DC-bias derating, 47 µF
will not fit 1210 → expect 1210/1812 and/or split caps. Verify total effective capacitance
after DC-bias derating at 24 V still satisfies loop stability across the full 5–24 V range.

**Ripple injection.** R144 = 10.7 kΩ is a starting value. Adding Rctrl makes the FB
AC load R2 ∥ Rctrl ≈ 505 Ω, which attenuates injected ripple. **Bench-trim R144** for
20–100 mV pp at FB across the full 5–24 V output range (depends on VOUT_P / SW swing).

**Current-sense shunt (R159 = 20 mΩ, as-built).** With ADCRANGE = 1 (±40.96 mV FS),
20 mΩ gives ±2.0 A FS — a good fit for the 1.5 A nominal load and low self-heating
(45 mW at 1.5 A). IN+ = buck-out, IN− = VCC_RB.

---

## 6a. VCC_RB disconnect gate (Q25)

A firmware-commanded high-side P-FET disconnect sits between the buck output (`VCC_RB`)
and the FE connector rail (`VCC_RB_G` → J6.1). It is a **third, independent interlock**
in series with the buck EN (RB_PSU_PWR_EN) and the §4 OV latch: even with the buck
running, the FE gets no power until `RB_VCC_GATE` (PB1) is asserted.

| Ref | Part | Connection |
|---|---|---|
| Q25 | **SI7469DP** P-ch | S = `VCC_RB` (buck out), D = `VCC_RB_G` (→ J6.1, FE), G = gate node `Net-(D25-A)` |
| Q26 | BC847W (NPN) | gate driver: B ← R261, E = GND, C → R258 → gate |
| D25 | zener (BZX84C12-class) | gate-source clamp (protects Vgs > ±20 V abs-max) |
| R257 | 100 kΩ | gate → source (`VCC_RB`): default-off pull-up |
| R258 | resistor | gate → Q26 collector |
| R261 | resistor | `RB_VCC_GATE` (PB1) → Q26 base |
| R262 | resistor | Q26 base pulldown → GND |
| D7 | 1.5SMCJ28A | TVS on `VCC_RB_G` at J6 (load-side) |

**Control (`RB_VCC_GATE` = PB1).** Default (PB1 Hi-Z/low at reset): Q26 off → R257 holds
gate = source → Vgs = 0 → Q25 **off** → `VCC_RB_G` unpowered (FE off). Assert PB1 high →
Q26 on → gate pulled toward GND through R258 → Vgs < 0 → Q25 **on** → `VCC_RB` →
`VCC_RB_G`. Unclamped Vgs = −0.909·VCC_RB (≈ −21.8 V at 24 V) would exceed the SI7469DP
±20 V gate abs-max, so the D25 gate-source zener clamp is **required**.

**D25 orientation.** `D25.1 (K) → VCC_RB (source)`, `D25.2 (A) → gate` (`Net-(D25-A)`) —
cathode on the source, anode on the gate. In the OFF/normal region D25 reverse-blocks; when
the gate is pulled below source to enhance Q25 it Zener-clamps |Vgs| to ~12 V at 24 V,
protecting the SI7469DP ±20 V gate abs-max.

---

## 6b. Decision Log

- **VCC_RB hard-bounded by fixed resistors (R147/R148/R134).** No digipot wiper code
  (POR, mid-scale, or an SPI fault) may drive the rail outside the Rb-safe envelope; the
  digipot only trims within the fixed-resistor bound. The 26 V OV latch (§4) is the
  firmware-independent backstop.
- **24.45 V pedestal is intentional.** The variable supply also services other external
  Rb references, not just one FE-5680A, so the rail is deliberately not bounded to a single
  unit's Vmax; firmware/digipot set the per-unit operating point. The output caps are
  therefore rated for the full pedestal (§6 sourcing item), not for a 15 V nominal.
- **D25 clamp orientation (K on source, A on gate).** This orientation reverse-blocks in
  the normal region and Zener-clamps |Vgs| only when the gate is pulled below the source to
  turn Q25 on. The opposite orientation would forward-conduct at ≈0.7 V as soon as the gate
  dropped below the source, pinning Vgs ≈ −0.7 V below the SI7469DP enhancement threshold so
  the P-FET could never turn on. Verified against the BZX84C12 / SI7469DP pinouts.
- **Three independent interlocks.** The FE gets power only when the buck EN
  (`RB_PSU_PWR_EN`, gated through the §4 OV latch) **and** the disconnect gate
  (`RB_VCC_GATE` = PB1) are both asserted — a single stuck bit cannot power the Rb.
