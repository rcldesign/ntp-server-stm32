# STS1000 — RGB Status Indicator (D5)

Single **IN-P55TATRGB** PLCC-6 RGB LED, low-side switched, driven from the always-on
5 V rail with 3.3 V STM32 PWM. One BC847W transistor and one resistor set per color.
Ballast values set each die to the same full-on luminous intensity (~750 mcd), so equal
PWM duty produces equal brightness across colors with no firmware balancing required.

---

## 1. Topology

- **6-pin independent-die part** (3 anodes, 3 cathodes; no internal common). Wired
  **common-anode**: all three anodes to +5 V through per-color ballast resistors;
  each cathode switched to GND by an NPN.
- **NPN low-side switch (BC847W)** per color, saturated. NPN is the single-transistor
  answer for low-side off a 5 V rail with 3.3 V logic — a PNP high-side switch cannot be
  turned off by a 3.3 V GPIO when its emitter sits at 5 V (Vbe stays ≈ −1.7 V), so it
  would need a second transistor as a level shifter.
- LED current never flows through the MCU; the GPIO sources only base current.

```
  5V_AON ──[R_ballast]── anode(die)            (one R per die)
  cathode(die) ──┬── collector
                 │
   LED_x ──[Rb]──┴── base   Q (BC847W:  B=1, C=3, E=2)
                 │
              [10k]→GND   (base pulldown — off at boot / GPIO Hi-Z)
              emitter ──→ GND
```

Active-high: GPIO high → base drive → transistor saturates → LED on. Duty = brightness.

---

## 2. LED parameters (datasheet, IN-P55TATRGB)

| Color | Tech | V_F @20 mA (typ) | Iv @20 mA (typ) | Efficacy (Iv/I) | λ_D |
|---|---|---|---|---|---|
| Red | AlInGaP | 2.0 V (1.6–2.4) | 900 mcd | 45 mcd/mA | 624 nm |
| Green | InGaN | 3.2 V (2.8–3.4) | 1800 mcd | 90 mcd/mA | 520 nm |
| Blue | InGaN | 3.2 V (2.8–3.4) | 600 mcd | 30 mcd/mA | 468 nm |

Abs-max (per color, 25 °C): **I_F = 25 mA DC**, I_FP = 100 mA (1/10 duty, 0.1 ms only),
V_R = 5 V, T_OP −40…+80 °C. I_F derates linearly to 0 at the 80 °C T_OP
(Forward-Current Derating Curve).

Pinout: 1 = Cath-Blue, 2 = Cath-Red, 3 = Cath-Green, 4 = An-Green, 5 = An-Red,
6 = An-Blue.

---

## 3. Brightness-balance theory

PWM scales time-averaged luminance linearly (luminance ≈ duty × full-on luminance), so
"equal duty → equal brightness" requires the **full-on luminous intensity to be matched
across the three dies**. Iv is approximately linear in I_F over this range, so matching
mcd means setting each die's current inversely to its efficacy:

```
I_red : I_green : I_blue  =  1/45 : 1/90 : 1/30  =  2 : 1 : 3
```

**Balanced current ratio Blue : Red : Green = 3 : 2 : 1.**

Blue is the least efficient die, so it sets the ceiling: the brightest *balanced* output
is blue at its current ceiling with red and green throttled to match. Choosing blue at
its 25 °C abs-max (≈ 24.8 mA) gives a common target of **~750 mcd/die**:

| Color | Target = 30 mcd/mA × I_blue | I_die | Iv @ I_die |
|---|---|---|---|
| Blue | (ceiling) | 24.8 mA | 24.8 × 30 = 744 mcd |
| Red | 744 / 45 | 16.7 mA | 16.7 × 45 = 752 mcd |
| Green | 744 / 90 | 8.4 mA | 8.4 × 90 = 756 mcd |

This is **Option A (hardware-balanced)**: equal PWM = equal brightness in hardware, no
firmware scaling. The cost is that red and green are permanently capped at ~750 mcd and
cannot reach their individual maxima (1125 / 2250 mcd). If full per-channel brightness is
ever needed (e.g. a max-intensity single-color effect), the ballasts must be re-sized to
each die's own max and the balance moved into firmware — see §8.

---

## 4. Ballast resistor calculation

Per die, with V_ce(sat) ≈ 0.15 V (BC847W in forced-β saturation) and V_F estimated at the
*operating* current (not the 20 mA datasheet point — green/red run below 20 mA):

```
R_ballast = (5.0 − V_F(I_die) − V_ce(sat)) / I_die
```

| Color | I_die | V_F @ I (est) | R calc | **R (E96)** | P_R | Footprint |
|---|---|---|---|---|---|---|
| Blue | 24.8 mA | 3.35 V | (5−3.35−0.15)/0.0248 = 60.5 Ω | **60.4 Ω** | 37 mW | 0603 |
| Red | 16.7 mA | 1.95 V | (5−1.95−0.15)/0.0167 = 173.7 Ω | **174 Ω** | 48 mW | 0603 |
| Green | 8.4 mA | 3.00 V | (5−3.00−0.15)/0.0084 = 220.2 Ω | **221 Ω** | 16 mW | 0603 |

0603 (100 mW) on all for margin and BOM consistency; do not use 0402 on red/blue.

---

## 5. Base resistor calculation

BC847W as a saturated switch, forced β ≈ 10 for low, stable V_ce(sat). V_be(sat) ≈ 0.8 V:

```
I_b = I_die / 10 ;   R_b = (3.3 − 0.8) / I_b = 2.5 / I_b
```

| Color | I_b (β_f≈10) | R_b calc | **R_b (E96)** | β_f actual |
|---|---|---|---|---|
| Blue | 2.48 mA | 2.5/0.00248 = 1008 Ω | **1.0 kΩ** | 24.8/2.5 = 9.9 |
| Red | 1.67 mA | 2.5/0.00167 = 1497 Ω | **1.5 kΩ** | 16.7/1.67 = 10.0 |
| Green | 0.84 mA | 2.5/0.00084 = 2976 Ω | **3.01 kΩ** | 8.4/0.83 = 10.1 |

BC847W min hFE is ≫ 10 at these collector currents, so all three are deeply saturated.
**Base pulldown 10 kΩ → GND** on each holds the transistor off when the GPIO is Hi-Z
(boot/reset), matching the Q21 relay-driver pattern.

Total base current with all three on = 2.5 + 1.67 + 0.83 ≈ **5 mA** across PD12–14.
Unlike a MOSFET gate (~0 static), the BJT base is a real per-pin load while on — within
the STM32H5 per-pin and port budget, but accounted for.

---

## 6. Designators

LED: **D5 — IN-P55TATRGB**.

| Color | Ballast → anode | Cathode → C | Transistor | Base R → B | Pulldown | Net |
|---|---|---|---|---|---|---|
| Blue | R59 60.4 Ω → D5.6 | D5.1 → Q8.C | Q8 BC847W | R55 1.0 kΩ | R56 10 kΩ | LED_B |
| Red | R60 174 Ω → D5.5 | D5.2 → Q9.C | Q9 BC847W | R57 1.5 kΩ | R58 10 kΩ | LED_R |
| Green | R61 221 Ω → D5.4 | D5.3 → Q10.C | Q10 BC847W | R62 3.01 kΩ | R63 10 kΩ | LED_G |

All emitters → GND. BC847W pin map B=1 / C=3 / E=2 applied on all three.

Total 5 V load (all on) ≈ 50 mA / 0.25 W. Reverse stress: with a die off its cathode
pulls to ≈ 5 V through the ballast while the anode is at 5 V, so V across the off die ≈ 0
— well inside the 5 V V_R abs-max.

---

## 7. Thermal derating (sets the real current ceiling)

25 mA is the **25 °C** abs-max; I_F derates to 0 at the 80 °C T_OP. In a warm enclosure
the sustainable maximum is below 25 mA, so blue's 24.8 mA design point is only valid if
the LED's **local** ambient is near 25 °C. Two actions:

- **Bench-confirm the LED's local ambient** at the front panel under worst-case
  enclosure conditions; read I_F(max) off the derating curve at that temperature.
- If warm, scale I_blue down and re-derive R/G via the 3:2:1 ratio (same procedure,
  lower target mcd). At I_blue = 20 mA the set becomes I_red = 13.3 mA, I_green = 6.7 mA,
  target ~600 mcd/die — a safe default if local ambient is uncharacterized.

I_FP (100 mA pulsed) is for 1/10 duty / 0.1 ms only and is **not** usable for steady
full-brightness; ignore it for this design.

---

## 8. Tolerance, trim, and the Option-A/B trade

**Ballasts are bench-trim starting points.** The dominant error is **V_F bin spread**:
green/blue V_F is specced 2.8–3.4 V over only a ~1.5 V resistor headroom, so ±0.3 V V_F ≈
±20 % current ≈ ±20 % brightness, and it breaks the inter-color balance unit-to-unit.
Mitigations, in order of preference:

1. **Per-unit firmware white-balance trim at commissioning** (recommended; integrates
   with the existing brightness/gamma policy — see §9). Robust to V_F spread.
2. Specify V_F bins on D5 (datasheet page-3 bins, e.g. F2/V2) — tightens spread but
   constrains procurement.
3. Bench-select ballast values against measured V_F per build.

Option A (this design) hard-codes the balance in the ballast ratio. If the product later
needs each color at its individual maximum, switch to **Option B**: size all three
ballasts to the chosen per-die max current and apply the 3:2:1 balance as a fixed
per-channel duty scale (R 66.7 %, G 33.3 %, B 100 %) folded into the brightness LUT.
Option B also makes the inter-unit V_F spread a firmware-cal problem rather than a BOM
problem.

---

## 9. Firmware implications

- **3 PWM channels on LED_R / LED_G / LED_B → PD12 / PD13 / PD14.** Expected TIM4
  CH1/CH2/CH3 on the LQFP144 (STM32H563ZIT6, U12) — **verify the AF pairing** before layout. Use one timer so
  all three share frequency/phase.
- **Polarity active-high.** Duty 0 % = off, 100 % = full (~750 mcd/color). Initialize the
  three pins **low** (or leave Hi-Z; the 10 kΩ pulldowns hold the transistors off) and
  ensure no glitch-on during clock/timer bring-up.
- **PWM frequency ≥ ~300 Hz**, 1–2 kHz typical, for flicker-free indication. BJT storage
  time (ns–µs) is negligible at these rates — no duty/edge error.
- **Linear vs perceptual.** Luminance is linear in duty, but perceived brightness is not;
  apply a gamma LUT if perceptual brightness steps are wanted. Balance (§3) holds in
  luminance regardless of gamma, since the same curve applies to all three channels.
- **Balance.** With Option A, equal duty already gives equal brightness; an optional
  per-channel scale (3 multipliers, default 1.0) provides the per-unit white-balance trim
  from §8 without hardware change.
- **Identify** pulses blue to locate the unit in a rack; blue is the channel at its
  ceiling so it has full available output. Brightness follows the ambient/backlight
  policy.
- The indicator must remain functional when the display sleeps — so its rail is
  **always-on 5 V, not the DISP_EN-gated 5V_DISP** (§10).

---

## 10. Verify before layout

- **5V net = always-on 5 V** (MIC28516 5 V intermediate), **not** the DISP_EN-gated
  5V_DISP — status/identify must survive display sleep.
- **D5 symbol pin# ↔ footprint pad# mapping** against the PLCC-6 datasheet
  (1 CathB / 2 CathR / 3 CathG / 4 AnG / 5 AnR / 6 AnB). Schematic color-routing is
  correct *iff* the symbol numbering matches the footprint.
- **LED_R/G/B land on the intended TIM4 PWM channels** (PD12–14), AF confirmed.
- **0603 ballast footprints** (R59 37 mW, R60 48 mW).
- **Local-ambient / derating** confirmed for the blue 24.8 mA design point (§7).
- **Ballast values re-trimmed** against measured V_F, or balance handled per §8.
