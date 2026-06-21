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
VREF_4V096 ──► MCP41U83 (B term) ──► wiper W ──► OPA320 follower ──► VCTRL ──► Rctrl ──► FB
                                                                                        ▲
SW ──► L7 ──► VCC_RB ──► COUT bank ──► R_shunt ──► clock connector                      │
              │                                                          R1 (VCC_RB→FB) │
              ├─ R1 (R147) ──────────────────────────────────────────────────────────┤
              └─ Cff (C125) across R1                          R2 (FB→GND), R144/C123 (SW→FB)
INA228 senses VCC_RB (I/V); TVS clamps VCC_RB at the connector.
```

FB is a 5-way node: R1 (R147), R2 (R148), RINJ+CINJ (R144/C123), Cff (C125), Rctrl (R134).
The loop holds FB = 0.600 V; VOUT follows the divider and the injected VCTRL.

---

## 2. Bill of materials

### Control front end
| Ref | Part | Value / spec | Function |
|---|---|---|---|
| U43 | MCP41U83T-503E/ST | 50 kΩ, 1024-pos, 10-bit, 14-TSSOP | Digital pot, potentiometer mode — output setpoint. Native 2.7–5.5 V supply |
| U42 | OPA320AIDBVT | RRIO, SOT-23-5 | Unity buffer for wiper → VCTRL (loads FB, not the pot string) |
| U45 | MCP1502T-40E/CHY | 4.096 V series reference | VREF_4V096 (scales VOUT 1:1; precision/tempco critical) |
| R134 | resistor | 4.12 kΩ 1% | Rctrl: control injection, U42 OUT → FB |
| C115 | capacitor | 2.2 µF | U45 reference output cap <!-- TODO verify designator: C115 (no cap on VREF_4V096 net in netlist) --> |
| C121 | capacitor | 0.1 µF | U43 VDD bypass <!-- TODO verify designator: C116 (C121/C131 are interchangeable 3V3 bypass caps) --> |
| C122 | capacitor | 0.1 µF | U42 supply bypass <!-- TODO verify designator: C117 (C122/C129 are interchangeable 5V bypass caps) --> |
| ~~C118~~ | — | — | EXT_CAP cap not required by MCP41U83 (no EXT_CAP pin — **confirm from datasheet**); refdes freed. C120 now used only as the §4 OV ref-node bypass |

### Output filter + FB network
| Ref | Part | Value / spec | Function |
|---|---|---|---|
| L7 | inductor | 40 µH, Isat ≥3 A, low DCR | Output inductor, SW → VCC_RB |
| R147 | resistor | 20.0 kΩ 1% | R1: VCC_RB → FB (divider top) |
| R148 | resistor | 576 Ω 1% | R2: FB → GND (divider bottom; sets 24.34 V pedestal) |
| R144 | resistor | 10.7 kΩ 1% *(bench-trim)* | RINJ: SW → FB ripple injection |
| C123 | capacitor | 0.1 µF, X7R, ≥100 V | CINJ: series with R144 (sees full SW swing) |
| C125 | capacitor | 47 nF, C0G/X7R, ≥50 V | Cff: feed-forward across R147 |
| C126, C127, C128 | capacitor | 47 µF, X7R, **≥63 V** each | COUT bulk (rating set by TVS clamp; derate for DC bias) |

### Protection + monitor
| Ref | Part | Value / spec | Function |
|---|---|---|---|
| D7 | 1.5SMCJ28A | 28 V standoff, uni, SMC | VCC_RB transient/ESD clamp at clock connector |
| U44 | INA228AQDGSRQ1 | I²C addr 0x47 | VCC_RB voltage + current monitor |
| R_shunt (R159) | shunt | size per load — see notes | VCC_RB current-sense element |
| C131 | capacitor | 0.1 µF | U44 supply bypass <!-- TODO verify designator: C120 (C121/C131 are interchangeable 3V3 bypass caps) --> |

### Overvoltage latch (see §4)
| Ref | Part | Value / spec | Function |
|---|---|---|---|
| U41A | LMV393 (½) | +5 V supply | OV comparator (detector) |
| R138 | resistor | 120 kΩ 1% | Sense divider top: VCC_RB → comparator (+) |
| R139 | resistor | 10.0 kΩ 1% | Sense divider bottom → GND (ratio 0.0769) |
| R136 | resistor | 10.5 kΩ 1% | Ref divider top: VREF_4V096 → comparator (−) |
| R137 | resistor | 10.0 kΩ 1% | Ref divider bottom → GND (→ 2.00 V) |
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

*C120 is the OV ref-node bypass. With the MCP41U83 (no EXT_CAP pin) the earlier
refdes collision is resolved — C120 is now this part only.

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
| VDD | +5 V | Native 2.7–5.5 V (C121 0.1 µF bypass) |
| VSS / DGND | GND | Single-supply, VSS = DGND = 0 V |
| **SPI2C** (interface select) | **DGND** | Straps the part to **SPI** mode |
| Terminal A | **GND** | Low end of divider |
| Terminal B | **VREF_4V096** | High end — code 0 parks wiper here (safe-low) |
| Wiper W | U42 +IN | Wiper → buffer |
| SDI/MOSI | SPI_MOSI | |
| SDO/MISO | SPI_MISO | Add pull-up only if datasheet requires for readback |
| SCK | SPI_SCK | ≤20 MHz |
| CS | SPI_DPOT_CS (PD2) | Chip select, active low |
| A0 / A1 | **TBD** | I²C address pins; in SPI mode tie per datasheet (don't-care vs. defined level — confirm) |

Terminal orientation (A = GND, B = VREF, code 0 → wiper at B → min VOUT) is unchanged
from the prior design and preserves the safe-low power-up direction. Confirm the
MCP41U83 wiper code 0 connects to terminal B (same convention) when finalizing.

### Other active devices
- **U42 OPA320:** V+ → +5 V (C122 bypass), V− → GND, +IN ← U43 W, −IN ↔ OUT (unity follower), OUT = VCTRL → R134.
- **U45 MCP1502 (4.096 V):** VDD → +5 V, OUT = VREF_4V096 (C115 2.2 µF), shutdown pin tied for always-on.
- **U44 INA228 (0x47):** IN+/IN− across R_shunt on VCC_RB, VBUS senses VCC_RB, I²C_SCL/SDA, ALERT → INA_ALERT_VCC_RB (EXTI), VS/VDD → +3.3 V (always-on rail), C131 bypass.

---

## 4. Overvoltage latch

Standalone hardware OV shutdown protecting the external clock (FE-5680A) against a
runaway VCC_RB (e.g. open-FB fault). Comparator detects; a regenerative BJT pair
latches and clamps the SMPS enable; firmware resets it. Independent of VCC_RB once
tripped (held from the always-on +5 V rail), so it latches off rather than hiccuping.

### Threshold ladder
```
24.34 V (max commanded) < 26.0 V (OV trip) < 28 V (TVS standoff) < ~31 V (TVS V_BR) < ~45 V (TVS clamp)
```
The latch fires at 26 V — before the TVS conducts — so the load is protected at the
trip point, not at the 45 V clamp.

### Detector (U41A, spare LMV393 half, on +5 V)
- Non-inverting input: VCC_RB → R138 (120 k) → (+) → R139 (10 k) → GND. Divider ratio 0.0769.
- Inverting input: VREF_4V096 → R136 (10.5 k) → (−) → R137 (10 k) → GND → 2.00 V; C120 bypass on the ref node.
- Trip when sense > 2.00 V → VCC_RB > 26.0 V.
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
| VCC_RB good (≤24.34 V) | low (D11 off) | off | off | off | ~3.3 V (enabled) | low |
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

FB summing node, V_FB = 0.600 V, R1 = 20.0 k, R2 = 576 Ω, Rctrl = 4.12 k:

```
VOUT = 0.6·(1 + R1/R2 + R1/Rctrl) − (R1/Rctrl)·VCTRL
     = 24.34 − 4.854·VCTRL
```

Buffered wiper, divider mode, B = VREF (4.096 V), A = GND:

```
VCTRL = ((1024 − D) / 1024) · 4.096        (D = RDAC code, 0…1023)
```

Combined (non-inverting in code):

```
VOUT ≈ 4.46 + 0.01941·D
```

| Target | Code D | Note |
|---|---|---|
| Min (code 0) | 0 | 4.46 V |
| 5 V | 28 | low end |
| 15 V (nominal) | 543 | |
| 24 V | 1007 | |
| Max (code 1023) | 1023 | 24.32 V |
| **Power-up preset** | **TBD** | MCP41U83 power-on wiper value — confirm from datasheet (§6) |

LSB = 19.4 mV (10-bit over the 19.9 V span). Monotonic; DNL ±1 LSB per datasheet.

---

## 6. Operating & firmware notes

**Supply (resolved).** U43 MCP41U83 runs natively on 2.7–5.5 V (single rail, VSS =
DGND = 0 V), powered from the existing +5 V. The prior AD5292 ≥9 V requirement no
longer applies; no dedicated pot supply rail is needed.

**Interface select.** SPI2C pin **tied to DGND → SPI mode.** Confirm A0/A1 tie level
(I²C-address pins, function in SPI mode per datasheet).

**Potentiometer mode.** Configure the part as a potentiometer (all three terminals on
the ladder); buffered + ratiometric, so the 50 kΩ RAB absolute value and its tolerance
do not affect VOUT — only string current (~82 µA at VREF/RAB) and bandwidth. **TBD:**
exact mode/config register write — confirm from datasheet.

**Wiper write sequence — TBD (datasheet needed).** The AD5292 command set no longer
applies. Confirm and document from the MCP41U83 sheet: (a) any config-lock / write-
protect that must be cleared before the volatile wiper register accepts writes;
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
5 V → code 28, or nominal 15 V) to MTP so the power-on preset is safe regardless.

**Protection layering.** **D7** clamps fast transients/ESD only; it cannot limit the
24.34 V pedestal (that is the commanded maximum) and cannot guard the FE-5680A against
a slow open-FB runaway. The **§4 OV latch** (U41A, 26 V trip → clamps RB_PSU_PWR_EN,
latched) is the DC-overvoltage guard; the TVS only handles the sub-µs surge/ESD window.

**COUT rating.** TVS clamp ≈45 V → COUT must be ≥63 V X7R. Verify total effective
capacitance after DC-bias derating at 24 V still satisfies loop stability across the
full 5–24 V range.

**Ripple injection.** R144 = 10.7 kΩ is a starting value. Adding Rctrl makes the FB
AC load R2 ∥ Rctrl ≈ 505 Ω, which attenuates injected ripple. **Bench-trim R144** for
20–100 mV pp at FB across the full 5–24 V output range (depends on VOUT_P / SW swing).

**Current-sense shunt (R159).** Size for the load and the INA228 full-scale. At 1.5 A
nominal, 100 mΩ drops 150 mV (225 mW) — large. With ADCRANGE = 1 (±40.96 mV FS),
~20 mΩ gives ±2.0 A FS, a better fit.
