# STS1000 "Meridian" — External-Reference RF Front End

**Comprehensive design, control, and firmware reference.**
This block conditions any common 10 MHz external reference into a clean 3.0 V CMOS square and drives **input B** of the system clock-source mux (peripheral map §2.1). It is the authoritative source for this subsystem's schematic, BOM, control interface, and firmware contract.

-----

## 0. TL;DR for the firmware engineer

- **One control line:** `REF_TERM_EN` = **PC10**, GPIO push-pull. `1` = 50 Ω termination engaged (sine / 50 Ω sources), `0` = termination lifted (high-Z logic sources). A 100 kΩ pull-up to the always-on +3.3 V rail defaults it **terminated** (safe) before firmware runs. Set-once from the per-source profile; never toggled in the hot path.
- **One validation line:** `EXTREF_MON` = **PB14 / TIM12_CH1**, watches the conditioned output `10MHz_CLK_OUT` (= mux input B). The reference state machine may select input B only after this reports *edges advancing AND frequency ∈ 10 MHz ± band*.
- **Selection is elsewhere:** the glitchless mux is driven by `MUX_SEL` (**PB6**); switching the live HSE source uses the HSI-bridge sequence in peripheral-map §2.1. This block only *produces and validates* the square; it does not select it.
- **Always-on hardware:** the comparator and island are powered whenever +5 V is good (island enable = `5V_PSU_PG`); there is no per-block MCU enable. The front end is damage-proof in any power state and either termination setting (§6), so firmware never protects it — it only validates the signal.

-----

## 1. Design targets

### 1.1 Operating input envelope (conditioned and served)

|Parameter|Value|Source|
|---|---|---|
|Frequency|10 MHz (gate band ±100 ppm, enforced by EXTREF_MON)|all|
|Dominant type|Sine, 50 Ω|FE-5680A (via its SMA output), lab/distribution refs|
|Level range|+6…+15 dBm sine **or** TTL/CMOS square|FE-5680A base +7 dBm and Opt 21 +13 dBm both covered|
|Square sources|TTL/CMOS, 50 Ω-driving or high-Z|supported|

Level → amplitude into 50 Ω: +7 dBm = 0.50 Vrms = 1.41 Vpp; +10 dBm = 2.0 Vpp; +13 dBm = 2.83 Vpp; +15 dBm = 3.56 Vpp.

**One comparator covers the whole range.** The LTC6752 has rail-to-rail inputs that extend beyond both rails; the N2 clamp holds any over-range signal within bounds; the comparator slices on the zero-crossings, which a clipped peak does not move. No level routing, no attenuator, no slicer select.

### 1.2 Protection / abuse envelope (survive, no damage)

|Stress|Requirement|Any power state?|Any setting?|
|---|---|---|---|
|ESD at connector|IEC 61000-4-2: ±8 kV contact, ±15 kV air|yes|yes|
|Continuous over-power|+20 dBm (100 mW) indefinitely|yes (incl. fully off)|yes|
|Transient over-power|+27 dBm (0.5 W) brief|yes|yes|
|DC offset on signal|any DC blocked from all active parts|yes|yes|
|Reverse / back-power|hot input must not pull any rail above an IC abs-max, nor back-power logic|yes|yes|

Out of scope: connecting a DC power supply directly to the SMA centre. C149 blocks it from everything downstream and is rated for it, but sustained HV DC on the RF port is misuse (reference power arrives on the DB9, never here).

-----

## 2. Architecture & key decisions

```
                                                  3V0_RF (island)
                                                       |
 SMA ─D8─||─C149─┬─N1─R178─┬─N2─||─C151─R181─┬─(+IN) ┌────────┐
(RF_IN) (ESD)(DCblk)│       │ (clamp │  (Rin) │   U50 │LTC6752 │ Q ─R184─► 10MHz_CLK_OUT
                   │        │  D19)  │        │ (−IN) │  xS5   │           ├─► mux input B
              R174/R175   D20(isl  REF_VMID──┴───────└────────┘           └─► EXTREF_MON
              47k/47k     clamp)   (1.5V, R179/R180)                            (PB14/TIM12)
              →N1=1.5V       │       R182 100k +IN bias
                   │     term leg:  VCC: 3V0_RF─R183(33Ω)─┬─VCC(5)
        3V3─R176─┤     N1─R177─C150─U49(SEL)   33Ω        D21(3V3)──┤
         (100k     │        49.9 0.1µF (TMUX1101            0.1µF+1µF──┤
         pull-up)  │  D18(BAT54S)──┘  VDD=3V0_RF,                    └─GND
                   │   pin-2 clamp    → GND)
              N1───┘

 Island: 5V ─FB9(600Ω)─ U51 LT3045 (3.0V) ─► 3V0_RF ;  EN/UV ← 5V_PSU_PG
```

**Decisions of record:**

- **Single LTC6752xS5 squarer.** One comparator covers the full input envelope — its rail-to-rail-beyond input plus the N2 clamp handle +6…+15 dBm sine, TTL square, and high-Z logic. CMOS output drives mux input B directly.
- **3.0 V island.** Set by the LT3045; matches the OCXO's 3.0 V Voh so both references present identical logic levels to the mux, and respects the base LTC6752's **3.6 V supply abs-max** with headroom.
- **VCC abs-max guard.** Base LTC6752 supply abs-max is 3.6 V (the 5.25 V figure applies only to the separate-supply −2/−3 variants, not offered in S5). R183 + D21 hold the comparator VCC ≤ ~3.5 V even if the island lifts in a fault.
- **Continuous ground plane** under the whole path (no split, no star tie, no bridge component) — isolation lives on the supply rail (LT3045 + FB9), not in the return. See §7.
- **All hardware always populated**; the MCU adapts the path at runtime via the single `REF_TERM_EN` bit.

-----

## 3. Protection philosophy — power-state-independent

The governing failure mode: a Schottky clamp to the 3 V rail **fails when the board is off**, because a dead rail is at 0 V and positive excursions then dump current into it, back-powering logic in an undefined state. Defeated by cooperating elements, none of which assume a live rail:

1. **ESD array (D8, SZESD7410)** at the connector — kills the kV transient first.
2. **DC block (C149)** — isolates every active part from input DC; a DC fault cannot reach the switch, clamp, or comparator.
3. **Rail clamp (D19 at N2)**, current-limited by **R178** — holds the comparator-input node within (−0.3 V … island+0.3 V) in every fault/setting case.
4. **Island shunt clamp (D20)** — sinks back-fed current the LT3045 cannot (an LDO can't sink), *defining* the island rather than letting it float; the comparator VCC is independently guarded by **R183 + D21**, so the comparator's survival does not depend on D20's exact clamp voltage.
5. **Termination-switch clamp (D18 at U49 pin 2)** — holds the switch's signal pin to the rails when the termination is lifted, so the open-switch state is safe for any source.

**N1 carries no voltage-sensitive load** (divider, C149, R177, R178 — all rated for the full ±6.3 V / ±14 V transient swing), so it needs no clamp; leaving the 50 Ω node clamp-free also removes shunt capacitance and preserves the match.

-----

## 4. Stage-by-stage design

### 4.1 Connector & ESD — SMA, `D8`
| Item | Value / PN | Notes |
|---|---|---|
| SMA | edge-mount, 50 Ω | reference input `10MHz_RF_IN` |
| `D8` | SZESD7410MXWT5G | bidirectional, 0.7 pF, V_BR 10 V; IEC 61000-4-2. Low C preserves the 50 Ω node. At the connector pads, before C149. |

### 4.2 DC block — `C149`
| Item | Value | Notes |
|---|---|---|
| `C149` | 100 nF C0G/X7R, 50 V, 0402 | 0.16 Ω at 10 MHz; HPF into 50 Ω ≈ 32 kHz. 50 V rating survives a +24 V-class miswire on the centre pin without conducting it downstream. |

### 4.3 Bias node N1 — `R174`, `R175`
| Item | Value | Notes |
|---|---|---|
| `R174`,`R175` | 47 kΩ 1% each: R174 = 3V0_RF→N1, R175 = N1→GND | Midpoint = N1 = **1.5 V** (half the island). 23.5 kΩ Thevenin ≫ 50 Ω → does not load the signal or detune the match. **No bypass on N1** (would short the signal). DC bias of N1 is always defined regardless of termination state. |

### 4.4 Switched termination — `R177`, `C150`, `U49`, `D18`, `R176`
Leg: **N1 → R177 → C150 → U49 → GND.**
| Item | Value / PN | Notes |
|---|---|---|
| `R177` | 49.9 Ω 0.1% thin-film | 50 Ω line termination (with C150 + U49 Ron ≈ 50.5 Ω → RL > 30 dB). |
| `C150` | 0.1 µF C0G/X7R | DC-blocks the leg so N1's 1.5 V draws no standing current; 0.16 Ω at 10 MHz → transparent. |
| `U49` | TI **TMUX1101DCK** SPST, SC70-6, VDD = 3V0_RF | `REF_TERM_EN` (PC10) on SEL: **1 = ON → terminated**, 0 = OFF → lifted. Grounded-end placement. 0.1 µF VDD decoupling. |
| `D18` | **BAT54S** at U49 pin 2 → 3V0_RF/GND | Clamps the switch's signal pin to the rails. With the switch open (term lifted), pin 2 sees the N1 swing through R177; D18 holds it to the rails so the lifted state is safe for any source — sine, logic, or a hot 50 Ω source. |
| `R176` | 100 kΩ → 3V3 (always-on) | Pull-up on `REF_TERM_EN`/SEL → defaults **terminated** (safe) before the island or MCU come up. SEL is fail-safe (may be driven above VDD and before VDD), so the 3.3 V idle level into a 3.0 V-powered part is valid. |

**Why U49 is on the 3.0 V island, not the 3.3 V logic rail:** SEL is 1.8 V-logic-compatible and fail-safe, so a 3.3 V control HIGH into a 3.0 V-powered part is valid with no level concern. Keeping VDD + GND + the termination return all in the island/analog-ground domain keeps the 10 MHz return local. The line is static (µA), so it does not dirty the low-noise rail.

### 4.5 Clamp & current limit — `R178`, `D19`, `D20`
| Item | Value / PN | Notes |
|---|---|---|
| `R178` | 100 Ω 1%, N1 → N2 | Sole current limiter into the N2 clamp; ≈ 100 MHz RC with the comparator input C → passes 10 MHz flat. |
| `D19` | **BAT54S**: A(1)→GND, COM(3)→N2, K(2)→3V0_RF | Clamps N2 to (−0.3 V … island+0.3 V), current-limited by R178. Legit swing never reaches it → no distortion/leakage. |
| `D20` | 3.3 V clamp on 3V0_RF (TVS preferred ≥ 1 W; 3V3 zener acceptable) | Island shunt clamp — sinks D19's back-fed current (~27 mA at +20 dBm), defining the island. V_RWM ≥ 3.0 V (no conduction at setpoint). Clamp voltage non-critical (≤ ~4.5 V fine) because R183+D21 guard the comparator. A TVS gives more +27 dBm surge margin. |

### 4.6 Squarer — `U50` (LTC6752xS5)
S5 = 5-lead TSOT-23. Pinout: **1 = Q, 2 = VEE, 3 = +IN, 4 = −IN, 5 = VCC.**
Specs: 2.9 ns prop delay, 280 MHz, 4.5 ps RMS jitter (100 mVpp/100 MHz), rail-to-rail-beyond inputs, CMOS output to 200 mV of rail, **fixed internal hysteresis** (no LE/HYST pin in S5).

| Net | Pin | Connection |
|---|---|---|
| +IN | 3 | N2 → **C151 10 nF** (AC-couple) → **R181 1 kΩ** (Rin) → +IN; plus **R182 100 kΩ** +IN → REF_VMID (self-bias). R181 limits +IN ESD-clamp current in a fault (≈ 1 mA), so almost no fault current reaches VCC. |
| −IN | 4 | REF_VMID. |
| Q | 1 | → **R184 33 Ω** → `10MHz_CLK_OUT` (= mux input B; also tapped to EXTREF_MON). |
| VEE | 2 | GND. |
| VCC | 5 | 3V0_RF → **R183 33 Ω** → VCC; **D21 (3V3)** VCC→GND at the pin; **0.1 µF + 1 µF** at the pin. |

`REF_VMID` = R179/R180 (10 kΩ/10 kΩ off 3V0_RF) = **1.5 V**, 0.1 µF bypass; feeds both the +IN bias (R182) and −IN.

**VCC guard (R183 + D21).** R183 in series from 3V0_RF to pin 5; D21 on the **pin-5 side** of R183. Under an off-state fault the island can lift to ~4 V; R183 drops the difference and D21 pins VCC ≤ ~3.4 V < 3.6 V abs-max. Normal Iq (4.5 mA) drops ~0.15 V → VCC ≈ 2.85 V (> 2.45 V min); the 0.1 µF + 1 µF supplies switching transients through the 33 Ω.

**Hysteresis:** fixed internal (S5). Adequate for the high-slew 10 MHz sine. For more immunity on marginal sources, add positive feedback Q→+IN (~1 MΩ) — no package change.

### 4.7 Supply island — `U51` (LT3045)
| Item | Value | Notes |
|---|---|---|
| `U51` | LT3045xDD | ultralow-noise LDO (0.8 µVRMS, 1 nV/√Hz). Dedicated island for U49 + U50; keeps SMPS spurs off the comparator and the bench fanout. |
| VOUT | **3.0 V** via `R186` = 30.1 kΩ 0.1% on SET (100 µA × 30.1k = 3.01 V) | matches OCXO Voh; respects LTC6752 3.6 V abs-max. |
| `C155` | 10 nF on SET | low-noise mode (bypasses SET-resistor + reference noise); increases start-up time. |
| `R185` | 1.5 kΩ ILIM→GND | current limit = 150 mA·kΩ / 1.5 kΩ = **100 mA** (≫ ~5 mA load). |
| `C154`,`C156` | 10 µF X7R (Cin / Cout) | — |
| `FB9` | ferrite ≈ 600 Ω @ 100 MHz | island input filter; ~260 mV dropout from +5 V → ample headroom. |
| EN/UV | ← `5V_PSU_PG` (defined logic level) | island enabled when +5 V is good. |
| OUTS (9) | tied to OUT (10) at the output node | regulation sense point. |
| PGFB (6) | R218/R219 divider | sets the PG threshold; configures the PG comparator and fast-start. |
| PG (4) | `3V0_RF_LDO_PG` → **PG2** (U12.87), **R266 10 kΩ → 3V3_STM** | open-collector power-good routed to the MCU; the pull-up presents a valid HIGH when the LDO is in regulation (§4.8). |

Island current: U50 ≈ 4.5 mA + U49 negligible ≪ 100 mA limit.

### 4.8 Power-good pull-up — `R266`

The LT3045 (U51) `PG` pin is **open-collector**: it pulls low when the LDO is out of
regulation and goes high-impedance (de-asserts) when power is good, so it requires an
external pull-up to present a valid HIGH. Net `3V0_RF_LDO_PG` carries **R266 (10 kΩ →
3V3_STM)** alongside `U51.4 (PG)` and `U12.87 (PG2)`; PG2 therefore reads power-good HIGH
in regulation and low on fault. This matches the sibling GPS LDO (U22, same LT3045), whose
`3V3_GPS_LDO_PG` net uses R215 10 kΩ → 3V3_STM. PG is monitor-only (not an enable), so the
island runs independently of this line.

-----

## 5. Control & firmware interface

### 5.1 MCU resources

| Signal | Pin | Peripheral / mode | Role |
|---|---|---|---|
| `REF_TERM_EN` | PC10 | GPIO output, push-pull, low speed | 1 = terminate (50 Ω/sine), 0 = lift (high-Z logic). Drives U49 SEL. R176 pull-up to +3.3 V → defaults terminated. |
| `EXTREF_MON` | PB14 | TIM12_CH1 (AF input capture / counter) | Validates `10MHz_CLK_OUT`: edges advancing AND f ∈ 10 MHz ± band. |
| `MUX_SEL` | PB6 | GPIO output | System mux select: 0 = OCXO (A, boot default), 1 = external ref (B). Owned by the reference state machine (peripheral map §2.1/§3.5). |
| `RB_LOCK` | PB13 | GPIO in, EXTI13 | FE-5680A lock-good (when the external source is the Rb); second precondition before selecting B. |

Reference-source power (FE-5680A) is gated by `RB_PWR_EN` (PB7) on the Rb buck — a separate subsystem, cross-referenced only.

**Boot/reset state:** PC10 resets to Hi-Z; R176 (100 kΩ to +3.3 V) then holds SEL high → **terminated** (the safe default — presents 50 Ω, and the lifted state is safe anyway via D18). Firmware drives PC10 explicitly early in init.

### 5.2 Power & enable sequencing

1. **+5 V good → `5V_PSU_PG` asserts → LT3045 enables → 3V0_RF live.** The comparator and front end power up independently of MCU boot; no MCU action required. The front end is passive-safe in all states.
2. **MCU boot:** set `MUX_SEL` = 0 (OCXO), drive `REF_TERM_EN` to the stored per-source default (or leave the R176 default = terminated).
3. **External source power** (if FE-5680A): `RB_PWR_EN` = 1 → Rb buck up → warm-up (cross-ref Rb buck). The conditioned square appears at `10MHz_CLK_OUT` once the source outputs 10 MHz.

### 5.3 Configuration, validation, and selection

`REF_TERM_EN` is **set-once** from the stored per-source profile and re-applied only on a source/clock-type change — never in the hot path. A wrong setting is non-destructive (the hardware is safe in either state); it merely fails EXTREF_MON validation, so the state machine never selects B.

Per-source profile = {source-supply voltage (Rb buck, cross-ref), REF_TERM_EN}:

| Source | Rb buck | REF_TERM_EN |
|---|---|---|
| FE-5680A (base or Opt 21 sine) | 15–18 V | 1 |
| Generic sine, +6…+15 dBm, 50 Ω | n/a (self-powered) | 1 |
| TTL/CMOS square, 50 Ω-driving | n/a | 1 |
| CMOS/TTL square, high-Z | n/a | 0 |

**Sequence to bring input B online:**

```
configure_external_ref(profile):
    REF_TERM_EN = profile.term            # set-once
    settle(>= 10 ms)                      # island/bias/AC-couple/Vmid settling (§5.4)
    if not extref_valid(): return INVALID
    if source_is_FE5680A and not RB_LOCK: return WAIT_LOCK
    glitchless_select_B()                 # MUX_SEL=1 via HSI-bridge (periph map §2.1)
    return OK

extref_valid():                           # EXTREF_MON gate, PB14/TIM12
    n = count_extref_edges(GATE)          # gate from a stable timebase (LSE or OCXO-derived)
    f = n / GATE
    return edges_advancing(n) and (|f - 10e6| <= BAND)
```

**Autorange (unknown source):** try `REF_TERM_EN = 1` first (correct for every 50 Ω/sine source and the common case); if `extref_valid()` fails after settle, try `REF_TERM_EN = 0` (high-Z logic). Lock and store the first setting that validates. Two states → a two-try search.

**Runtime monitoring & revert:** once on B, `EXTREF_MON` is polled continuously; if it fails (frequency out of band or edges stop) — or `RB_LOCK` de-asserts for an Rb source — the reference SM reverts to OCXO immediately (glitchless), with hysteresis on the re-engage path to prevent flapping. The OCXO is always live on input A.

**`EXTREF_MON` measurement (TIM12_CH1 on PB14):**
- *Gated edge count (recommended):* count `10MHz_CLK_OUT` rising edges over a known gate (e.g. 10 ms from LSE or an OCXO-derived timer) → expect ~100 000 ± BAND·gate. Robust against duty-cycle/jitter; yields presence + frequency.
- *Input capture:* capture consecutive edges, derive period (~100 ns). Faster but more sensitive to jitter/aliasing.
Set BAND from the application (±100 ppm typical; widen for autorange acquisition, narrow for the in-lock check). Define an "edges advancing" timeout so an absent clock fails deterministically.

### 5.4 Timing & settling budget

| Element | Time constant | Note |
|---|---|---|
| LT3045 start-up | ms-class | increased by Cset (C155); only at power-on |
| REF_VMID divider | 5 kΩ × 0.1 µF = 0.5 ms | threshold settle |
| AC-couple HPF | ~159 Hz (C151 with R181+R182) | edge-coupling; ms-class envelope settle |
| VCC RC | 33 Ω × 1 µF = 33 µs | negligible |
| Comparator prop delay | 2.9 ns | instantaneous vs. the above |

Use **≥ 10 ms** settle after changing `REF_TERM_EN` (or after the source starts) before trusting `EXTREF_MON`.

-----

## 6. Protection analysis (worst-case math)

### 6.1 Over-power node voltages
A source rated "+P dBm into 50 Ω" has open-circuit EMF = 2× its into-50 Ω voltage. **Termination lifted** (worst case) presents the doubled voltage at N1.

| Input | Into-50 Ω Vpk | Open-circuit at N1 (term lifted) |
|---|---|---|
| +15 dBm | 1.78 V | 3.56 V |
| +20 dBm (continuous spec) | 3.16 V | **6.32 V** |
| +27 dBm (transient spec) | 7.07 V | 14.1 V |

### 6.2 Clamp currents (device OFF, +20 dBm, term lifted)
- N1 swings to ~6.32 Vpk, loading only the 47 kΩ divider (134 µA), C149, the term leg, and R178.
- R178 (100 Ω) → N2; D19 clamps to the island: I ≈ (6.32 − ~3.6)/100 ≈ **27 mA** (half-wave) into the island.
- D20 sinks it, defining the island at ≤ ~4 V; dissipation ≈ 27 mA × 4 V ≈ 0.1 W → trivial for a ≥ 1 W TVS.
- **Comparator VCC is independently guarded:** even with the island at ~4 V, R183 + D21 hold VCC ≤ ~3.4 V < 3.6 V abs-max; D21 sinks ≈ (4 − 3.4)/33 ≈ 18 mA ≈ 0.06 W.
- Comparator +IN guarded by R181 (1 kΩ): internal input clamp draws ≈ 1 mA, harmless.
- **Termination leg:** with the switch lifted, U49 pin 2 sees the N1 swing through R177; D18 clamps it to the rails → safe.

Transient +27 dBm: island-path I ≈ (14.1 − 3.6)/100 ≈ 105 mA brief → D20 ≈ 0.4 W within TVS surge; BAT54S IFSM ok; VCC still pinned ≤ ~3.5 V by D21.

### 6.3 Setting-combination safety
| TERM | Source | Outcome |
|---|---|---|
| 1 | 50 Ω sine/square | normal |
| 1 | high-Z logic | source sags into 50 Ω (its problem); our parts unstressed |
| 0 | high-Z logic | intended use; U49 pin 2 clamped by D18 → safe |
| 0 | hot 50 Ω source | N1 ±6.3 V; R178 limits the N2/island path; U49 pin 2 clamped by D18 → safe (a wrong setting only fails validation) |
| any | +15 dBm / 5 V into comparator | N2 rail-clamped via D19; R181 protects +IN → no damage; slices the crossings |
| any | OFF + input ≤ +20 dBm | D8/C149/R178/D19/D20/D18 + R183/D21 cooperate; no rail back-powered above abs-max |

Conclusion: every power state × either termination setting × ≤ +20 dBm continuous (≤ +27 dBm transient) is non-destructive, with no residual stress point.

-----

## 7. Grounding & layout

**One continuous ground plane under the entire path — SMA → comparator → mux. No split, no star tie, no bridge component (no ferrite / 0 Ω / via neck) between an "RF ground" and digital ground.** A 1.2 ns CMOS edge has content to ~300 MHz; HF return current images directly under the trace following least *inductance*. A plane split with a single star tie forces that return into a large loop → radiation, EMI pickup, and jitter. Star/single-point grounding works only at DC/audio and is wrong here.

**The isolation is on the supply rail, not the return:** LT3045 + FB9 keep SMPS spurs off the comparator. The ground stays common and continuous.

**The only Kelvin point is local at the LT3045:** route the SET (R186) and ILIM (R185) resistor grounds directly back to the U51 GND pin (datasheet Kelvin requirement). That is device-level routing on the *same* plane.

**Placement, not cuts, provides separation:**
- Keep the RF front end in its own physical region; steer SMPS, Ethernet PHY, and STM32 digital returns elsewhere so they never share copper under the RF traces.
- Continuous plane under SMA → comparator → mux; stitch with ground vias along the 50 Ω trace and around U50.
- Short, controlled-impedance 50 Ω trace SMA → N1; R177 close to N1; D8 at the connector pads; C149 immediately after.
- D19 returns to the plane with minimal loop; D20 adjacent to U51 output; D18 at U49 pin 2.
- R183 + D21 + the VCC bypass tight to U50 pin 5; R181 + C151 tight to +IN; REF_VMID bypass close to U50.
- `REF_TERM_EN`, `10MHz_CLK_OUT`, and the EXTREF_MON tap cross as ordinary signals referenced to the same plane; route them away from the RF node.

**Test points:** N1, N2, REF_VMID, 3V0_RF, U50 VCC (pin 5), 10MHz_CLK_OUT.

-----

## 8. Consolidated BOM

| Ref | Part / value | Pkg | Purpose |
|---|---|---|---|
| SMA | edge-mount 50 Ω | — | reference input |
| D8 | SZESD7410MXWT5G | X2-DFN | ESD (IEC 61000-4-2) |
| C149 | 100 nF C0G/X7R 50 V | 0402 | DC / fault block |
| R174,R175 | 47 kΩ 1% | 0402 | N1 bias → 1.5 V |
| R178 | 100 Ω 1% | 0402 | N2 clamp current limit |
| D19 | BAT54S | SOT-23 | N2 clamp (A→GND, COM→N2, K→3V0_RF) |
| D20 | 3.0–3.3 V TVS, V_RWM ≥ 3.0 V, ≥ 1 W (or 3V3 zener) | SOD | island shunt clamp *(user-selected)* |
| R177 | 49.9 Ω 0.1% thin-film | 0402 | 50 Ω termination |
| C150 | 100 nF C0G/X7R | 0402 | term-leg DC block |
| U49 | TI TMUX1101DCK | SC70-6 | termination switch (SEL = PC10), grounded-end |
| — | 0.1 µF | 0402 | U49 VDD decoupling |
| D18 | BAT54S | SOT-23 | U49 pin-2 clamp → 3V0_RF/GND |
| R176 | 100 kΩ 1% | 0402 | REF_TERM_EN pull-up → +3.3 V (default terminated) |
| C151 | 10 nF C0G | 0402 | comparator input AC-couple |
| R181 | 1 kΩ 1% | 0402 | +IN clamp-current limiter (Rin) |
| R179,R180 | 10 kΩ 1% | 0402 | REF_VMID divider (1.5 V) |
| — | 0.1 µF | 0402 | REF_VMID bypass |
| R182 | 100 kΩ 1% | 0402 | +IN self-bias to REF_VMID |
| U50 | **LTC6752xS5** (base, S5) | TSOT-23-5 | squarer → mux input B |
| R183 | 33 Ω 1% | 0402 | VCC isolation (abs-max guard) |
| D21 | 3.3 V TVS/Zener, ≥ 0.5 W | SOD | VCC local clamp (pin-5 side of R183) |
| — | 0.1 µF + 1 µF | 0402 | U50 VCC decoupling (at pin 5) |
| R184 | 33 Ω 1% | 0402 | mux-B / EXTREF_MON edge damping |
| U51 | LT3045xDD | DFN | island LDO 3.0 V |
| R186 | 30.1 kΩ 0.1% | 0402 | LT3045 SET (3.0 V) |
| C155 | 10 nF | 0402 | LT3045 Cset (low-noise) |
| R185 | 1.5 kΩ 1% | 0402 | LT3045 ILIM (~100 mA) |
| C154,C156 | 10 µF X7R | 0603 | LT3045 Cin / Cout |
| FB9 | ferrite ≈ 600 Ω @ 100 MHz | 0603 | island input filter |

-----

## 9. Net reference

| Net | Nodes | Notes |
|---|---|---|
| `10MHz_RF_IN` | SMA center, D8, C149 | 50 Ω, controlled-Z |
| `N1` | C149, R174/R175 mid, R177, R178 | bias = 1.5 V; no clamp, no bypass |
| `N2` | R178, D19 COM, C151 | clamped to (−0.3 … island+0.3) V |
| `REF_VMID` | R179/R180 mid, R182, U50 −IN, 0.1 µF | 1.5 V threshold |
| `3V0_RF` | U51 OUT, D20, D19 K, D18, R174, R179, R183, U49 VDD, bypass | 3.0 V island |
| `10MHz_CLK_OUT` | U50 Q via R184 → mux input B **and** EXTREF_MON (PB14) | 3.0 V CMOS square |
| `REF_TERM_EN` | PC10 → U49 SEL, R176 pull-up to +3.3 V | static control, default terminated |
| `5V_PSU_PG` | → U51 EN/UV | island enable |
| `3V0_RF_LDO_PG` | U51 PG (4) → PG2 (U12.87), R266 10 kΩ → 3V3_STM | island power-good (open-collector), pulled up to present a valid HIGH — §4.8 |
| `5V` | FB9 → U51 IN | island input |
| `3V3` | R176 | always-on rail (pull-up only) |

-----

## 10. Bring-up & test procedure

1. **Island:** apply +5 V, assert `5V_PSU_PG`. Verify 3V0_RF = 3.00 ± 0.03 V; REF_VMID = 1.50 V; U50 VCC ≈ 2.85 V (pin 5, after R183).
2. **Bias/clamp (no signal):** N1 = 1.5 V, N2 = 1.5 V. With PC10 undriven, confirm R176 holds SEL high → terminated.
3. **Signal:** inject +7 dBm 10 MHz sine, term engaged (PC10=1): clean swing at N1, D19 not conducting, `10MHz_CLK_OUT` is a 3.0 V 10 MHz square.
4. **EXTREF_MON:** confirm TIM12 counts ~100 000 edges / 10 ms gate; verify in-band pass and out-of-band/absent fail.
5. **Termination logic:** toggle PC10 → confirm U49 switches (50 Ω present when 1, lifted when 0) via return-loss or N1 amplitude; confirm D18 clamps U49 pin 2 in the lifted state under a hot input.
6. **Hot-input / off-state:** with the board **off**, apply +20 dBm 10 MHz; confirm no rail exceeds abs-max (probe 3V0_RF and U50 VCC ≤ 3.5 V). Repeat powered.
7. **End-to-end:** run `configure_external_ref()`, confirm `EXTREF_MON` validates, then glitchless `MUX_SEL`=1; verify PH0 tracks and the bench fanout follows.

-----

## 11. Failure modes & diagnostics

| Symptom | Likely cause | Check |
|---|---|---|
| EXTREF_MON never validates, signal present | term setting wrong for source type; source level too low; AC-couple/bias fault | scope N2 and 10MHz_CLK_OUT; try the other REF_TERM_EN state |
| Comparator output stuck high/low | REF_VMID wrong (R179/R180), +IN bias open (R182), no input | meter REF_VMID = 1.5 V; check C151/R181 continuity |
| Island > 3.0 V or noisy | LT3045 OUTS open, FB9 open, Cout fault | verify OUTS=OUT, 3V0_RF ripple |
| VCC > 3.5 V in fault | D21 not at the pin-5 node of R183 | confirm D21 cathode at the R183–pin5 node |
| Termination never engages | R176/PC10 drive conflict; U49 SEL open | confirm SEL toggles 0/1 at U49 |
| Reference flaps OCXO↔ext | BAND too tight, no re-engage hysteresis | widen/hysteresis in EXTREF_MON gate (§5.3) |

-----

## 12. Open items
1. `D20` (user-selecting): TVS V_RWM ≥ 3.0 V, ≥ 1 W; clamp ≤ ~4.5 V acceptable (R183+D21 guard the IC).
2. LTC6752 S5 internal hysteresis value (LE/HYST not bonded) — confirm adequate for the 10 MHz sine; decide on optional Q→+IN positive feedback.
3. Confirm U50 VCC ≈ 2.85 V stays > 2.45 V min over temp/Iq spread; trim R183 if marginal.
4. Confirm the `10MHz_CLK_OUT` → EXTREF_MON (PB14) tap is damped/routed cleanly alongside the mux-B route.
5. FE-5680A Loop-Lock polarity (base: >3 V unlocked / <1 V locked; Opt 26 inverts) → RB_LOCK sense (DB9 side; cross-reference).

-----

## 13. System interface & cross-references
- **Clock mux (peripheral map §2.1):** `10MHz_CLK_OUT` drives mux input B; `MUX_SEL` (PB6) selects it; the glitchless HSI-bridge sequence switches the live HSE source.
- **EXTREF_MON (PB14 / TIM12_CH1):** validates `10MHz_CLK_OUT` (edges advancing + in band) before selection and continuously thereafter.
- **Software spec:** the reference state machine owns `MUX_SEL` and consumes `EXTREF_MON`; `REF_TERM_EN` is the only front-end control it drives.
- **Rb housekeeping (separate subsystem):** `RB_PWR_EN` (PB7) gates the FE-5680A buck; `RB_LOCK` (PB13) and `RB_UART` (PB4/PE7) carry lock/telemetry. Only the 10 MHz (via SMA) enters this chain.
