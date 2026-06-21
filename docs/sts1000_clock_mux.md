# STS1000 "Meridian" — System Clock-Source Mux (OCXO / external-ref → PH0)

**Comprehensive design, control, and firmware reference.**
This block selects between the onboard OCXO (`OCXO_CLK_OUT`, mux input **A**) and the conditioned external/Rb reference (`10MHz_CLK_OUT`, mux input **B**), re-drives a clean 3.3 V CMOS square into the STM32 HSE-bypass pin **PH0** (`CLK_OUT`), and provides an isolated buffered copy to the bench 10 MHz fanout (`10MHz_RF_OUT`). It is the authoritative source for the clock-mux schematic, BOM, control interface, and firmware contract. Implements peripheral-map §2.1.

**Status: schematic-verified, as-built.** Both logic pinouts confirmed against the Nexperia datasheets (§A); select polarity, default state, and net routing confirmed on the captured schematic.

-----

## 0. TL;DR for the firmware engineer

- **One control line:** `MUX_SEL` = **PB6**, GPIO push-pull. **0 = OCXO (A, boot default)**, **1 = external/Rb (B)**. A **100 kΩ pull-down (R149)** on the SEL pin forces **A (OCXO)** before firmware runs (PB6 is Hi-Z at reset). Polarity matches the 74LVC1G157 function table directly (S=L→I0, S=H→I1) — no inversion.
- **Switching is glitchless by firmware, not by the part.** Before changing `MUX_SEL`, bridge SYSCLK to HSI, switch, wait `HSERDY`, re-lock the PLL on the new HSE (§4.2). A logic 2:1 selector is intentionally chosen over a "glitch-free" clock-mux IC: those complete their handoff on the *outgoing* clock and **hang if the source you are leaving has stopped** — precisely the Rb-died-return-to-OCXO case. A plain selector cannot hang. (The dedicated parts — IDT/Renesas ICS580/581 — are also EOL.)
- **Hardware failsafe:** enable the **Clock Security System (CSS)**. If the live reference dies while selected, CSS → NMI → firmware drops to HSI and re-selects OCXO. This replaces the in-mux clock-detect that the EOL ICS580 would have provided.
- **Rail discipline:** the mux/buffer run on **+3.3V_CLK = the MCU VDD domain** (ferrite-isolated via FB9), **never** the 3.327 V OCXO LDO. PH0 abs-max is VDD+0.3; a >VDD Voh would over-drive it.
- **Always-on:** both references are permanently powered and present at A and B at all times (§2.1). The mux only *forwards* one; it never powers a source down. The 74LVC1G157 **IOFF** feature also prevents a live source back-powering the part if +3.3V_CLK is down.

-----

## 1. Design targets

| Parameter | Value | Source |
|---|---|---|
| Inputs | two 10 MHz CMOS squares, A = OCXO, B = external/Rb front end | §2 OCXO, ext-ref front-end doc |
| Input A level | ~3.0 V Voh CMOS, 0 V Vol, **R98 = 22 Ω source term at the OCXO**, 50 Ω DC-coupled trace | OCXO buffer |
| Input B level | 3.0 V CMOS (3.0 V island), **R144 = 33 Ω source term at the LTC6752 Q** | LTC6752xS5 Q |
| Output to PH0 (`CLK_OUT`) | full-swing **3.3 V** CMOS square, ≤ VDD, ~50 % duty, 10 MHz | HSE bypass spec |
| PH0 HSE-bypass thresholds | VIH > 0.7·VDD = 2.31 V; VIL < 0.3·VDD = 0.99 V; **must not exceed VDD or go below VSS**; 1–32 MHz, ~40–60 % duty | STM32H5 datasheet (digital bypass) |
| Bench fanout (`10MHz_RF_OUT`) | isolated buffered copy, source-matched to 50 Ω, AC-coupled, ESD-protected | follows the active reference |
| Switch behavior | glitchless **at the system level** via HSI bridge; CSS failover on loss | §2.1 |

**Level margins (VCC = 3.3 V on the mux):** 74LVC1G157 VIH (2.7–3.6 V) = 2.0 V; both 3.0 V sources clear it (margin ≥ 1.0 V). Mux Voh ≈ 3.3 V clears PH0's 2.31 V with ~1.0 V margin and stays ≤ VDD. No level translation needed. The '157 also has **Schmitt-trigger inputs**, easing any slow-edge concern on the input clocks.

-----

## 2. Architecture (as-built)

```
 OCXO_CLK_OUT ─(R98 22Ω @OCXO, 50Ω trace)──────► I0(3) ┐
 (A: 3.0V CMOS, 10 MHz; src-terminated upstream)        │  U45            Y(4)
                                                        │ 74LVC1G157 ──┬─ R150 22Ω ─► CLK_OUT ─► PH0 (OSC_IN, HSE bypass; PH1/OSC_OUT = NC)
 10MHz_CLK_OUT ─(R144 33Ω @LTC6752)────────────► I1(1) ┘  GW-Q100H     │
 (B: 3.0V CMOS, 10 MHz; src-terminated upstream)   S(6)                └─► A(2) U46 ─Y(4)─ R151 22Ω ─ C141 100nF ─► 10MHz_RF_OUT ─► SMA
                                                    │                       74LVC1G34GW                                  │
 MUX_SEL (PB6) ─────────────────────────────────────┤                                                                  D16 SZESD7410
   R149 100kΩ ↓ GND  (default S=L → I0 = OCXO at boot)                                                         (low-C, at connector, off-page)

 Rail:  +3.3V ─ FB9 (≈600Ω@100MHz) ─► +3.3V_CLK  (VDD domain; ≤ VDD so PH0 is never over-driven)
        decoupling: C142 1 µF bulk + C140 0.1 µF (U45 VCC, pin 5) + C143 0.1 µF (U46 VCC, pin 5), tight to each pin
```

**Decisions of record:**

- **Logic 2:1 selector (74LVC1G157GW-Q100H), not a glitch-free clock-mux IC.** (a) The dedicated parts (IDT/Renesas ICS580/581) are EOL. (b) A glitch-free mux completes its handoff on the outgoing clock's edges and can hang if that clock has stopped — the worst case is exactly "Rb stopped, return to OCXO." A selector forwards combinationally and cannot stall. (c) At 10 MHz LVCMOS, a logic mux's additive jitter is sub-ps and swamped by the source oscillators and the STM32 PLL.
- **Glitchlessness lives in firmware (HSI bridge) + CSS.** §2.1 already mandates the HSI bridge for the live-HSE swap; with SYSCLK on HSI during the flip, any select-edge runt at the mux is irrelevant to the core. CSS gives automatic, hardware-timed failover on reference loss.
- **One gate on the PH0 path.** The mux output (Y, pin 4) drives PH0 through a single 22 Ω series damper (R150). The bench fanout is taken from the **Y node before R150** through a *separate* buffer (U46) so its 50 Ω load and the SMA port never disturb the PH0 node.
- **No series R at the mux inputs.** Source termination lives at each driver (R98 at the OCXO, R144 at the LTC6752); a receiver-end series R into the high-Z, Schmitt CMOS input (CI = 2.5 pF) does nothing for the match and would be asymmetric between A and B. (The earlier R148 on I0 was removed for this reason.)
- **VDD-domain supply.** +3.3V_CLK is the MCU 3.3 V rail behind FB9, so the mux Voh tracks VDD and never exceeds the PH0 abs-max. The 3.327 V OCXO LDO is deliberately not used here.
- **All hardware always populated;** runtime selection only, via the single `MUX_SEL` bit (consistent with the program-wide no-DNP/no-jumper rule).

-----

## 3. Stage-by-stage design

### 3.1 Selector — `U45` (74LVC1G157GW-Q100H)
TSSOP6 / SOT363-2, AEC-Q100 Grade 1, −40/+125 °C. VCC 1.65–5.5 V, 2:1 non-inverting multiplexer, **Schmitt inputs**, **IOFF** partial-power-down, ±24 mA drive @ 3.0 V, tpd ≈ 2.7–3.1 ns @ 3.0–3.6 V, CI = 2.5 pF.

| Pin | Symbol | Net | Connection |
|---|---|---|---|
| 1 | I1 | `10MHz_CLK_OUT` | input B; R144 (33 Ω) source-terminated at the LTC6752 Q. Short stub. |
| 2 | GND | GND | continuous plane. |
| 3 | I0 | `OCXO_CLK_OUT` | input A; R98 (22 Ω) source-terminated at the OCXO over its 50 Ω trace. Direct to I0 (no receiver-end R). |
| 4 | Y | (mux-out node) | → R150 → `CLK_OUT`, and → U46 A (pin 2). |
| 5 | VCC | `+3.3V_CLK` | C140 0.1 µF at the pin. |
| 6 | S | `MUX_SEL` (PB6) | S=0 → I0 (OCXO); S=1 → I1 (ext/Rb). R149 100 kΩ pull-down → default OCXO. |

Function table (Nexperia DS): S=L → Y=I0; S=H → Y=I1. Non-inverting.

### 3.2 Output to PH0 — `R150`
| Item | Value | Notes |
|---|---|---|
| `R150` | 22 Ω 1% 0402 | Series source-damp on the short `(Y node) → CLK_OUT → PH0` trace. PH0 (OSC_IN, bypass) is a high-Z digital input; 22 Ω + U45 Zo (~25–35 Ω) damps the line without a parallel terminator. **Keep `CLK_OUT` short and over continuous ground to the STM32.** |
| PH1 (OSC_OUT) | NC | left high-Z in bypass (peripheral map §2). |

### 3.3 Bench fanout — `U46`, `R151`, `C141`, `D16`
Isolated so the SMA/cable load and any hot-plug event never reach PH0.

| Item | Value / PN | Notes |
|---|---|---|
| `U46` | 74LVC1G34GW-Q100, TSSOP5 / SOT353-1 | single non-inverting buffer. High-Z input (pin 2) taps the **Y node (before R150)** — negligible load; re-drives the fanout. Place close to U45. Q100 grade matches U45. |
| `R151` | 22 Ω 1% (trim 22–33) | Source-match: U46 Zo (~30 Ω) + R151 ≈ 50 Ω. Delivers ~1.5–1.65 Vpp into a 50 Ω bench load (≈ +8 dBm-class square). |
| `C141` | 100 nF C0G/X7R, 50 V | DC-block to bench gear; HPF corner = 1/(2π·50·100 n) ≈ 32 kHz → transparent at 10 MHz. |
| `D16` | **SZESD7410MXWT5G** (≤ 0.7 pF, bidir, IEC 61000-4-2) | output-port ESD at the SMA pads (off-page, at the connector). Low C preserves the 10 MHz edges. Same family as the RF-input ESD (D11). |

> If a higher-drive or lower-additive-jitter fanout is wanted later (sine-shaped, +13 dBm, or distribution to several ports), U46 → a dedicated clock-fanout/driver block; the mux interface here is unchanged.

### 3.4 Select default — `R149`
| Item | Value | Notes |
|---|---|---|
| `R149` | 100 kΩ 1%, `MUX_SEL` → GND | Forces S=0 (OCXO/A) at power-up/reset before PB6 is driven (PB6 resets Hi-Z). When firmware drives PB6 high (select B), it sources only 33 µA into R149 — negligible. SEL is static (set on a source change), so no series damping needed. |

### 3.5 Supply island — `FB9`, decoupling
| Item | Value | Notes |
|---|---|---|
| `FB9` | ferrite ≈ 600 Ω @ 100 MHz, 0603 | `+3.3V → +3.3V_CLK`. Keeps digital/SMPS spurs off the clock rail while staying in the VDD domain (so Voh ≤ VDD at PH0). |
| `C142` | 1 µF X7R 0402 | bulk on +3.3V_CLK at FB9 output. |
| `C140` | 0.1 µF 0402 | at U45 VCC (pin 5), tight. |
| `C143` | 0.1 µF 0402 | at U46 VCC (pin 5), tight. |

-----

## 4. Control & firmware interface

### 4.1 MCU resources
| Signal | Pin | Mode | Role |
|---|---|---|---|
| `MUX_SEL` | PB6 | GPIO out, push-pull, low speed | 0 = OCXO (A, boot), 1 = ext/Rb (B). Owned by the reference state machine. R149 → default A. |
| `EXTREF_MON` | PB14 / TIM12_CH1 | timer input capture/counter | Validates input B (`10MHz_CLK_OUT`): edges advancing **and** f ∈ 10 MHz ± band. Precondition for selecting B. |
| `RB_LOCK` | PB13 / EXTI13 | GPIO in | FE-5680A lock-good. Second precondition for B. |
| (HSE/CSS) | — | RCC | `HSERDY`, `HSECSS` (NMI) own the live-clock health on whatever the mux forwards. |

No new MCU pin is consumed by this block — it reuses the §2.1 set.

### 4.2 Glitchless switch sequence (firmware-owned)
Both same-frequency (10 MHz), so the PLL config is identical across the switch.

```
Select B (OCXO → ext/Rb), only when EXTREF_MON valid AND RB_LOCK asserted, stable past debounce:
  1. SYSCLK ← HSI            (bridge: nothing locked to PH0 now)
  2. (optional) disable HSE CSS for the transition window
  3. MUX_SEL = 1            (mux forwards B; any select-edge runt is harmless on HSI)
  4. wait for HSE re-stabilize → poll HSERDY
  5. re-enable CSS; PLL re-lock on HSE; SYSCLK ← PLL

Revert B → A: trigger on EITHER RB_LOCK de-assert OR EXTREF_MON fail OR CSS/NMI.
  Same steps with MUX_SEL = 0. Re-engage to B uses hysteresis to prevent flapping; fallback to OCXO is fail-safe.
```

`CSS` is the catch-all: if the *selected* reference stops between firmware polls, the NMI forces HSI immediately; the handler sets `MUX_SEL = 0` and re-locks on the always-warm OCXO.

-----

## 5. BOM (as-built)

| Ref | Part / value | Pkg | Purpose |
|---|---|---|---|
| U45 | 74LVC1G157GW-Q100H | TSSOP6 (SOT363-2) | 2:1 clock selector; Schmitt in, IOFF |
| U46 | 74LVC1G34GW-Q100 | TSSOP5 (SOT353-1) | bench-fanout buffer |
| R149 | 100 kΩ 1% | 0402 | MUX_SEL pull-down → default OCXO |
| R150 | 22 Ω 1% | 0402 | Y → PH0 (`CLK_OUT`) series damp |
| R151 | 22 Ω 1% (trim 22–33) | 0402 | 50 Ω source-match, fanout |
| C140 | 0.1 µF | 0402 | U45 VCC decoupling (pin 5) |
| C141 | 100 nF C0G/X7R 50 V | 0402 | fanout AC-couple / DC block |
| C142 | 1 µF X7R | 0402 | +3.3V_CLK bulk (at FB9) |
| C143 | 0.1 µF | 0402 | U46 VCC decoupling (pin 5) |
| D16 | SZESD7410MXWT5G (≤ 0.7 pF) | X2-DFN | SMA fanout ESD (at connector, off-page) |
| FB9 | ferrite ≈ 600 Ω @ 100 MHz | 0603 | +3.3V → +3.3V_CLK |

-----

## 6. Net reference (as-built)

| Net | Nodes | Notes |
|---|---|---|
| `OCXO_CLK_OUT` | OCXO buffer (R98) → U45 I0 (pin 3) | input A, 3.0 V CMOS, source-terminated upstream (R98) |
| `10MHz_CLK_OUT` | LTC6752 Q (R144) → U45 I1 (pin 1) | input B, 3.0 V CMOS, source-terminated upstream (R144); also → EXTREF_MON (PB14) at the front end |
| (mux-out / Y node) | U45 Y (pin 4) → R150, U46 A (pin 2) | selected reference; fanout tap is here, **before** R150 |
| `CLK_OUT` | R150 → PH0 (OSC_IN) | HSE bypass, full-swing 3.3 V CMOS; keep short to STM32 |
| `10MHz_RF_OUT` | U46 Y (pin 4) → R151 → C141 → SMA, D16 | bench 10 MHz fanout, follows the active reference |
| `MUX_SEL` (PB6) | U45 S (pin 6), R149 → GND | static select, default A |
| `+3.3V_CLK` | FB9 → U45/U46 VCC, C140/C142/C143 | VDD-domain clock rail (≤ VDD) |

-----

## 7. Grounding & layout
- Continuous ground plane under `OCXO_CLK_OUT`, `10MHz_CLK_OUT`, the Y node, and the `CLK_OUT` trace — consistent with the RF-front-end grounding philosophy (isolation on the rail via FB9, not the return).
- U45 GND is **pin 2 only**; route it to the plane. (Confirmed the GND drop does not tie to the I0/`OCXO_CLK_OUT` net.)
- Place **U45 adjacent to PH0**; keep `(Y node) → R150 → CLK_OUT → PH0` short and direct. Keep both input stubs (I0, I1) short; the real source termination is already at each source (R98, R144).
- Place **U46 next to U45** so the Y-node tap to the buffer is a short stub.
- Steer the bench-fanout 50 Ω trace and SMA away from PH0; keep its return on the same plane. D16 at the SMA pads.
- FB9 + C142 at the rail entry; C140/C143 hard against each VCC pin.
- **Test points:** Y node, `CLK_OUT` (PH0), `10MHz_RF_OUT`, `+3.3V_CLK`.

-----

## 8. Bring-up & test
1. **Rail:** +3.3V_CLK = 3.30 ± 0.1 V; ripple low after FB9.
2. **Default select (no firmware):** PB6 Hi-Z → R149 holds S=0 → confirm `CLK_OUT` = OCXO 10 MHz at PH0, full 3.3 V swing, ~50 % duty. Verify VIH/VIL margins against 0.7/0.3·VDD.
3. **Manual select:** drive PB6=1 → `CLK_OUT` follows input B (`10MHz_CLK_OUT`); PB6=0 → back to A. Confirm clean full-swing in both states.
4. **Bench fanout:** terminate `10MHz_RF_OUT` in 50 Ω; confirm ~1.5–1.65 Vpp, in phase with the active source; trim R151 if amplitude/match needs it.
5. **HSE bring-up:** HSEBYP set, confirm `HSERDY`, PLL locks on `CLK_OUT`; SYSCLK = PLL.
6. **Glitchless switch:** run the §4.2 sequence A↔B on a scope/counter; confirm no SYSCLK disturbance (core on HSI during flip) and PLL re-lock.
7. **CSS failover:** kill the selected reference; confirm CSS/NMI fires, firmware drops to HSI and re-selects OCXO, system recovers.

-----

## 9. Failure modes & diagnostics
| Symptom | Likely cause | Check |
|---|---|---|
| Boots on B (ext) instead of A | R149 open / wrong-variant part ('158) | meter S at power-up = 0 V; confirm part is '157 |
| PH0 not toggling / HSE won't ready | U45 Y dead, R150 open, wrong rail | scope Y node and `CLK_OUT`; confirm +3.3V_CLK; confirm input present at I0 |
| HSE marginal / intermittent HSERDY | Voh < 0.7·VDD (rail sag) or duty out of band | confirm +3.3V_CLK ≥ 3.3 V; check source duty |
| Switch upsets SYSCLK | HSI bridge not used / CSS not handled | verify §4.2 sequence order |
| No failover when reference dies | CSS disabled, or stuck masked after a switch | confirm CSS enabled in steady state, masked only in the switch window |
| Fanout wrong amplitude/reflections | R151 mismatch, far-end not 50 Ω, C141 open | trim R151; verify termination |

-----

## 10. Open items
1. Trim R150 / R151 against the actual `CLK_OUT` and fanout trace lengths post-layout.
2. Firmware: minimize the CSS masking window in §4.2 — don't blind the failsafe longer than the switch.
3. Confirm bench-equipment 10 MHz input impedance / level expectation to finalize R151 (and whether DC-block C141 alone suffices).
4. *(Optional)* if bench-fanout phase noise is ever found marginal, swap U46 → a dedicated low-additive-jitter clock buffer; for U45, 74AUP1G157 is a lower-noise drop-in. Interface unchanged.

-----

## A. Verification record (datasheet-confirmed)
- **U45 74LVC1G157-Q100 (Nexperia, Rev. 6, 2025-09-09):** pinning 1=I1, 2=GND, 3=I0, 4=Y, 5=VCC, 6=S — matches schematic exactly. Function table S=L→I0 / S=H→I1 (non-inverting) — confirms R149 pull-down boots to OCXO. Schmitt inputs, IOFF, ±24 mA @ 3.0 V noted.
- **U46 74LVC1G34 (Nexperia, SOT353-1):** pinning 1=n.c., 2=A, 3=GND, 4=Y, 5=VCC — matches schematic exactly. Non-inverting buffer (A→Y), fanout in-phase with the reference.
- **STM32H5 HSE bypass:** OSC_IN is a digital input requiring >0.7·VDD / <0.3·VDD, ≤ VDD; 1–32 MHz, ~40–60 % duty; OSC_OUT NC. The 3.3 V re-buffered square satisfies this with ~1 V margin.
- **Removed:** R148 (redundant receiver-end series R on I0; OCXO source term is R98). **Ruled out:** OCXO-to-GND short at the I0 node (U45 GND is pin 2 only).
- **EOL note:** dedicated glitch-free clock-mux ICs (IDT/Renesas ICS580-01, ICS581-0x) are obsolete; not used.

-----

## 11. System interface & cross-references
- **Inputs:** A = OCXO `OCXO_CLK_OUT` (peripheral map §2); B = `10MHz_CLK_OUT` from the external-reference front end (RF-front-end doc; supersedes the older multi-slicer §2.2 — single LTC6752xS5 now).
- **Output:** `CLK_OUT` → PH0 HSE bypass + `10MHz_RF_OUT` bench fanout (peripheral map §2.1).
- **Control:** `MUX_SEL` (PB6); validation via `EXTREF_MON` (PB14/TIM12) and `RB_LOCK` (PB13); failover via HSE CSS.
- **Doc note:** peripheral-map §2 wording "OCXO … → PH0 directly" should be updated to "→ PH0 **via the §2.1 clock mux** (`CLK_OUT`)."
