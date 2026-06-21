# GNSS Active-Antenna Feed, Current Limiter & Antenna Supervisor

**Design documentation — operation, control signals, and parameters**

| | |
|---|---|
| **Subsystem** | Active GNSS antenna feed (bias-T), current-limited supply, and antenna-fault supervisor |
| **Host receiver** | u-blox ZED-F9T-00B (multi-band timing receiver) |
| **Host MCU** | STM32H563VIT6 |
| **Antenna rail** | 5 V, 150 mA maximum |
| **Logic domain** | +3.3V_GPS (F9T VCC_IO); comparators powered from +5V_LDO |

---

## 1. Overview

This subsystem powers an active (LNA-equipped) GNSS antenna over the same coaxial line that carries the received RF, protects the system against antenna/cable faults, and reports antenna health to both the GNSS receiver and the host MCU.

It performs four functions on one signal path:

1. **RF/DC combining (bias-T):** delivers DC bias to the antenna while passing the received GNSS RF through to the receiver.
2. **Current-limited 5 V supply:** feeds the antenna LNA with a hard foldback current limit, protecting the rail and antenna against a short.
3. **Antenna supervision:** detects *open* (no antenna), *normal*, and *short* conditions and drives the ZED-F9T's antenna-supervisor inputs.
4. **Commanded shutdown:** allows the receiver (and, indirectly, firmware) to switch antenna power off.

The supervisor uses **two independent fault paths** for defense in depth: the analog supervisor described here drives the F9T's ANT_DETECT / ANT_SHORT_N pins (surfaced over UBX-MON-RF), while a separate antenna-rail INA228 read by the STM32 over I²C provides a precise, digital current measurement. Firmware fuses the two (§9).

---

## 2. System Block Diagram

```
                          GPS_RF_IN  ──►  ZED-F9T RF_IN
                              ▲
                              │  (RF received signal)
                             L1  56 nH  ← bias-T choke (DC pass / RF block)
                              │
   V_ANT ──FB2──┬──[ Q2 ]─────┴───────────────►  node A  ──► antenna (RF + 5 V DC)
   (5 V)        │   PNP pass        (antenna DC feed point / Q2 collector)
                ├──[ R30 ]───┐
                │   3.3 Ω    │  current-limit foldback:  Q1 (PNP) + R29
                │  (sense)   │
                │            └──► INA181 current sense ──► U12A ──► GPS_ANT_DETECT ──► F9T pin 4
                │                  (across R30)            comparator
                │
              node A ──► R33/R34 divider ──► U12B comparator ──► GPS_ANT_SHORT ──► F9T pin 6

   F9T ANT_OFF (pin 5) / GPS_ANT_OFF_MON ──► Q4 (NPN) ──► Q5 (PNP) ──► forces Q2 base to V_ANT (antenna off)
                                                  │
                                                  └──► observed by STM32 PD4 (EXTI4)
```

Two physical domains coexist on the line: a **DC path** (V_ANT → current limiter → L1 → antenna) and an **RF path** (antenna → L1 node → GPS_RF_IN). L1 and C26 separate them.

---

## 3. RF/DC Combining (Bias-T)

| Ref | Value | Function |
|-----|-------|----------|
| L1 | 56 nH (250 mA, 820 mΩ DCR) | Bias-T choke. Passes DC bias to the antenna; presents high impedance to GNSS RF so the signal stays on the line to the receiver. |
| C26 | 1 µF | DC-feed-node bypass; provides an AC return so RF does not enter the bias network. |
| FB2 | 120 Ω @ 100 MHz | Conditions the incoming V_ANT rail (suppresses conducted noise). |
| C27 | 0.1 µF | V_ANT local decoupling. |

The choke is the only series element between the antenna DC feed (node A) and the receiver RF input. At the GPS L1 band (1.575 GHz) the 56 nH choke presents:

```
Z = 2πfL ≈ 2π × 1.575e9 × 56e-9 ≈ 554 Ω
```

This clears u-blox's ">500 Ω at GNSS frequencies" guideline with margin across the multi-band range. The choke's 250 mA saturation rating comfortably exceeds the ~180 mA current limit, and its 820 mΩ DCR adds only ~0.15 V of drop at the limit current (this drop is downstream of node A, so it affects the voltage delivered to the antenna but not the supervisor thresholds).

---

## 4. Antenna Power Supply & Current Limiting

The antenna is fed from the 5 V rail through a high-side PNP pass transistor with a transistor-based foldback current limiter.

| Ref | Part | Value | Function |
|-----|------|-------|----------|
| Q2 | NSS40300MZ4T1G | PNP power | High-side pass element delivering 5 V to the antenna. V_CEO 40 V, I_C(max) 3 A, P_D(max) 2 W, V_CE(sat) ≤ 400 mV @ I_C = 3 A. |
| Q1 | BC857W | PNP | Current-limit sense transistor. |
| R30 | — | 3.3 Ω | Current-sense / limit resistor in series with the antenna current. |
| R29 | — | 220 Ω | Q1 base path. |
| R28 | — | 2.2 kΩ | Q2 base drive. |

### How the limit works

R30 carries the full antenna current and develops a voltage proportional to it. Q1's base–emitter junction sits across R30; when the current produces approximately one V_BE across R30, Q1 turns on, diverts Q2's base drive, and folds the pass current back to a fixed ceiling:

```
I_limit ≈ V_BE(Q1) / R30 ≈ 0.6 V / 3.3 Ω ≈ 180 mA
```

This is set ~20 % above the 150 mA antenna maximum, giving operating headroom while still clamping well below any level that would stress the rail or a shorted cable. **The limit point is defined solely by R30** — changing it requires re-checking the supervisor thresholds in §6 and §8, which reference the same resistor and feed node.

In normal operation Q2 stays in saturation, so the DC drop from V_ANT to the antenna is small (R30 + V_CE(sat) ≈ a few hundred mV at full load). At the ≤180 mA operating range Q2 runs far below its 3 A / 2 W ratings, so its actual V_CE(sat) is well under the 400 mV datasheet figure (specified at 3 A) and self-heating is negligible. On a short, the limiter holds the current at ~180 mA and Q2 comes out of saturation, collapsing the feed-node voltage — the signature the supervisor uses to flag a short (§6.2).

---

## 5. Antenna Supervisor — Detection Principle

The supervisor must distinguish three states: **open** (no antenna / no current), **normal**, and **short** (over-current). Because the supply uses an *active* foldback limiter rather than a large passive series resistor, the antenna feed-node voltage alone cannot separate open from normal:

- With Q2 saturated, the feed node (node A) sits near 5 V whether the antenna draws 0 mA (open) or only a few mA (light-normal).
- Therefore **open vs. present must be decided by measuring current**, while **short is decided by feed-node collapse**.

The supervisor is built as two independent legs accordingly:

| Leg | Senses | Drives | Decides |
|-----|--------|--------|---------|
| DETECT | Antenna current via INA181 across R30 | GPS_ANT_DETECT (F9T pin 4, active-high = present) | Open vs. present |
| SHORT | Feed-node (node A) voltage via divider | GPS_ANT_SHORT (F9T pin 6, active-low = short) | Short vs. ok |

### State table

| Condition | I through R30 | Node A | INA181 out | GPS_ANT_DETECT | GPS_ANT_SHORT |
|-----------|---------------|--------|------------|----------------|---------------|
| **Open** | ~0 mA | ~5 V | ~0 V | **low** (absent) | high (ok) |
| **Normal** | 5–150 mA | 4.3–5 V | 0.33 V → rail | **high** (present) | high (ok) |
| **Short** | ~180 mA (clamped) | ~0 V | railed high | high (present) | **low** (short) |

In the short state the limiter holds ~180 mA, so the DETECT leg still reads "present." This is intentional and unambiguous: the SHORT leg's low output is the signal the receiver acts on, and open vs. short is fully captured by the SHORT leg.

---

## 6. Supervisor Circuit Detail

### 6.1 DETECT leg (current sense)

| Ref | Part | Value | Function |
|-----|------|-------|----------|
| U11 | INA181A1IDBVR | gain 20 | High-side current-sense amp across R30. V+ = +3.3V_GPS; IN+ = V_ANT (upstream) side of R30; IN− = load side; REF = GND (unidirectional). |
| C28 | — | 0.1 µF | U11 decoupling. |
| U12A | ½ LMV393 | — | DETECT comparator: +in = U11 OUT, −in = reference. Open-collector output. |
| R31 / R32 | — | 100 k / 11 k | DETECT reference divider from +3.3V_GPS → **0.327 V**. |
| C29 | — | 0.1 µF | Reference / supply decoupling. |
| R40 | — | 10 kΩ | Open-collector pull-up to +3.3V_GPS → GPS_ANT_DETECT. |

**Transfer function:** `V_out = gain × I × R30 = 20 × I × 3.3 = 66 V/A`.

- **Threshold:** 0.327 V ÷ 66 V/A ≈ **5 mA**. DETECT asserts (high) for any antenna drawing ≥ ~5 mA — above idle leakage, below any real active antenna's draw.
- **Linear range:** U11's output rails at ~3.0 V (≈ 45 mA); above this it simply stays high, which is fine because this leg only decides present/absent. Precision current is provided separately by the INA228 (§9).
- **Gain choice:** gain 20 (the INA181**A1**) keeps the threshold linear with margin; the gain-100 A3 variant would rail at ~9 mA, leaving no usable range.
- **Polarity:** present → U11 OUT > ref → output high-Z → R40 pulls to 3.3 V → DETECT high. Absent → output sinks → DETECT low. Matches the F9T (active-high = present).

### 6.2 SHORT leg (feed-node collapse)

| Ref | Part | Value | Function |
|-----|------|-------|----------|
| U12B | ½ LMV393 | — | SHORT comparator: +in = node A ÷ 2, −in = reference. Open-collector output. |
| R33 / R34 | — | 100 k / 100 k | Node-A divider → node A ÷ 2 (draws ~25 µA — negligible vs. antenna current). |
| R36 / R37 | — | 100 k / 30 k | SHORT reference divider from +3.3V_GPS → **0.762 V**. |
| C30 | — | 0.1 µF | Reference / supply decoupling. |
| R39 | — | 10 kΩ | Open-collector pull-up to +3.3V_GPS → GPS_ANT_SHORT. |

**Threshold:** node A ÷ 2 is compared to 0.762 V, so SHORT asserts when **node A < ~1.52 V**. Normal node A is 4.3–5 V (÷2 = 2.1–2.5 V, above ref → released); a short pulls node A to ~0 V → asserted.

**Polarity:** +in = node A ÷ 2, −in = ref. Normal → +in > ref → high-Z → SHORT_N high (ok). Short → +in < ref → output sinks → SHORT_N low (fault). Matches the F9T (active-low = short).

### 6.3 Comparator supply & level shifting

| Ref | Part | Value | Function |
|-----|------|-------|----------|
| U12C | LMV393 (power-pin unit of the dual) | — | V+ = +5V_LDO, V− = GND. |
| C31 | — | 0.1 µF | LMV393 supply decoupling. |

The LMV393 is powered from **+5 V** (not 3.3 V) so its input common-mode range (0 to V+ − 1.5 V ≈ 0–3.5 V) spans every supervisor input: U11 OUT (0–3.0 V), node A ÷ 2 (0–2.5 V), and both references (≤0.76 V).

Both comparator outputs are **open-collector, pulled up to +3.3V_GPS**. This pull-up is the level shift from the 5 V analog domain to the F9T 3.3 V logic domain. It is also inherently fail-safe: when +3.3V_GPS is off, the lines are simply undriven, so the F9T's input pins are never back-driven from the 5 V side. This follows u-blox's guidance to use open-drain buffering whenever the antenna voltage (5 V) exceeds the receiver IO voltage (3.3 V).

---

## 7. Antenna-Disable Control (ANT_OFF)

The F9T's ANT_OFF pin (5) is an **output that goes high to command the antenna off**. Disabling a high-side PNP requires pulling Q2's base **up to V_ANT** (its emitter), so a two-stage inverting/high-side driver is used.

| Ref | Part | Value/role | Connection |
|-----|------|------------|------------|
| Q4 | BC847W (NPN) | Level shift / invert | E→GND; B← ANT_OFF via R38; C→ Q5 base via R41. |
| Q5 | BC857W (PNP) | High-side disable switch | E→V_ANT; C→ Q2 base node (where R28 lands); B← R41 and R42. |
| R38 | 10 kΩ | ANT_OFF → Q4 base. |
| R41 | 4.7 kΩ | Q4 collector → Q5 base (drive limit). |
| R42 | 10 kΩ | Q5 base → V_ANT (holds Q5 off when Q4 is off). |
| R43 | 100 kΩ | Q4 base → GND — sets the default state when the ANT_OFF line is undriven (see below). |

**Behavior:**

- ANT_OFF **high** → Q4 on → Q5 base pulled low (~0.8 mA via R42) → Q5 on → Q2 base tied to V_ANT → **Q2 off, antenna unpowered.**
- ANT_OFF **low** → Q4 off → Q5 off (held by R42) → **Q2 runs normally.** Q5 easily overrides R28's 2.2 kΩ to hold Q2 off when commanded.
- ANT_OFF **undriven** (F9T unpowered / pin hi-Z) → R43 holds Q4 off → Q5 off → **antenna defaults on.** This matters because V_ANT can remain up while the F9T is power-cycled; defaulting the antenna *on* preserves a fast warm restart. (To default *off* instead, R43 would pull high — but that rail is also down during an F9T cycle, so the GND default is the practical choice.)

Because the only connection to the F9T pin is Q4's base through R38, the 5 V rail never reaches the receiver pin.

**Shared monitoring net:** the ANT_OFF line is also the `GPS_ANT_OFF_MON` net observed by the STM32 on **PD4 (EXTI4)**, letting firmware see when the antenna has been commanded off. PD4 is configured **input-only** (the F9T drives this net push-pull; an output on PD4 would contend with it).

---

## 8. Control & Status Signals + Operating Parameters

### 8.1 Signal interface

| Net | Direction | Domain | Meaning |
|-----|-----------|--------|---------|
| GPS_ANT_DETECT → F9T pin 4 | supervisor → F9T | 3.3 V, open-collector + pull-up | High = antenna present (I ≥ ~5 mA). |
| GPS_ANT_SHORT → F9T pin 6 | supervisor → F9T | 3.3 V, open-collector + pull-up | Low = antenna short (node A < ~1.5 V). |
| ANT_OFF / GPS_ANT_OFF_MON ← F9T pin 5 | F9T → supervisor; also → MCU PD4 | 3.3 V push-pull | High = command antenna off. |
| V_ANT | rail input | 5 V | Antenna supply rail. |
| GPS_RF_IN | RF | — | Combined RF to receiver. |

### 8.2 Operating parameters

| Parameter | Value | Set/limited by |
|-----------|-------|----------------|
| Antenna supply voltage | 5 V | V_ANT rail |
| Antenna operating current (max) | 150 mA | Antenna spec |
| Current limit (foldback) | ~180 mA | R30 (3.3 Ω) + Q1 V_BE |
| DETECT (antenna-present) threshold | ~5 mA | R31/R32 → 0.327 V, ÷ 66 V/A |
| SHORT threshold | node A < ~1.5 V | R33/R34 ÷2 vs. R36/R37 → 0.762 V |
| Current-sense linear ceiling | ~45 mA | INA181A1 gain 20 on 3.3 V (present/absent only) |
| Bias-T choke impedance @ 1.575 GHz | ~554 Ω | L1 = 56 nH (250 mA, 820 mΩ DCR) |
| Supervisor logic level | 3.3 V | +3.3V_GPS pull-ups |
| Comparator supply | 5 V | +5V_LDO |

Tolerance: the dividers are 1 % resistors feeding an LMV393 (few-mV offset) and an INA181A1 (low offset, ~±1 % gain error). The DETECT threshold (5 mA) has wide margin against any real antenna (≥ ~10 mA); the SHORT threshold (1.5 V) sits far from both normal (≥ 4.3 V) and fault (~0 V), so hysteresis is not required for clean switching.

---

## 9. ZED-F9T & STM32 Integration

### 9.1 Pin map

| F9T pin | Signal | Dir (F9T) | Net |
|---------|--------|-----------|-----|
| 4 | ANT_DETECT | input | GPS_ANT_DETECT (U12A) |
| 5 | ANT_OFF | output | ANT_OFF / GPS_ANT_OFF_MON |
| 6 | ANT_SHORT_N | input | GPS_ANT_SHORT (U12B) |

| STM32 pin | Net | Role |
|-----------|-----|------|
| PD4 (EXTI4) | GPS_ANT_OFF_MON | Observes LNA-disable / commanded-off state. Input-only. |
| PC9 | ANT_BIAS_EN | Antenna-bias enable; dropped on persistent short. |
| PC8 | GPS VCC switch | Gates F9T VCC. |
| PD2 / PD3 | V_BCKP backup | Must stay powered through GPS VCC cycles (warm start). |
| I²C1 (PB8/PB9) | — | Antenna-rail INA228 #4 (precision V/I/P). |

### 9.2 Antenna-state fusion

Firmware determines antenna state by fusing three independent sources:

1. **UBX-MON-RF** from the F9T (driven by the analog supervisor in this document).
2. **GPS_ANT_OFF_MON (PD4)** — whether the antenna is currently commanded off.
3. **Antenna-rail INA228 #4** over I²C — precise current → OK / open / short.

On a **persistent** short, firmware drops `ANT_BIAS_EN (PC9)` and raises an alarm (SNMP trap, log entry, GUI indication). The redundancy ensures a single-source transient does not by itself trigger a shutdown, and gives the operator a precise current reading for diagnostics.

### 9.3 Commanded-off fault masking (firmware requirement)

When ANT_OFF is asserted (by the F9T or by firmware via ANT_BIAS_EN), Q2 turns off, node A collapses, and antenna current goes to zero — so **the analog supervisor reports SHORT (low) and DETECT (absent) even though nothing is faulty.** The F9T masks its own internal flags while it holds ANT_OFF; the firmware fusion logic must likewise **gate the SHORT/OPEN flags on the commanded-off state (PD4 / ANT_BIAS_EN)** to avoid logging phantom shorts on every commanded antenna cycle.

### 9.4 Backup-domain interaction

V_BCKP (F9T pin 36) stays powered across GPS VCC power-cycles for warm start. V_ANT is on the STM32-gated antenna rail, independent of the F9T VCC switch, so the antenna can remain powered while the F9T is cycled — which is why the ANT_OFF stage defaults the antenna *on* when the receiver pin is undriven (§7).

---

## 10. Design Rationale & Constraints

**Why an active foldback limiter (Q1/Q2) rather than a passive series resistor.** Lower dropout and a well-defined current limit independent of antenna impedance. The trade-off is that the feed-node voltage no longer separates open from normal, which is why the DETECT decision is made by current sensing (§5).

**Why split detection (current for open, feed-node for short).** Each fault has a clean, well-separated signature in exactly one domain. A single window comparator on the feed node would fail to detect open, and a single current threshold cannot cleanly distinguish high-normal from the clamped short current.

**Why INA181A1 (gain 20).** It keeps the present/absent threshold in the amplifier's linear range with margin; the precise current measurement is delegated to the INA228, so this leg needs only coarse presence detection.

**Why open-collector outputs to 3.3 V.** A single-resistor level shift from the 5 V analog domain to F9T logic, inherently fail-safe (lines undriven when receiver IO is off), and aligned with u-blox's open-drain recommendation for antenna voltage > IO voltage.

**Why the LMV393 runs on 5 V.** To give input common-mode headroom across all supervisor signals; the open-collector outputs still serve the 3.3 V logic domain cleanly.

**Why a discrete ANT_OFF stage (BC847W + BC857W).** Keeps the F9T pin isolated from the 5 V rail, uses a small-signal transistor pair common across the board, and sets a defined default state with one resistor (R43).

**Why dual-path supervision (analog + INA228).** Defense in depth — no single glitch trips a shutdown — plus a precise digital current value for telemetry and diagnostics.

### Engineering constraints to honor at layout/BOM

- **R30** dissipates ~130 mW at the ~180 mA limit; specify 0805, 1 %, ≥0.25 W.
- **INA181 input orientation** is critical: IN+ on the V_ANT/FB2 (upstream) side of R30, IN− on the load side, current flowing + → −. With REF = GND the device is unidirectional, so reversed inputs read zero.
- **PD4** must remain input-only in firmware (shared with the F9T's push-pull ANT_OFF driver).
- If a real short produces a slow node-A collapse, light hysteresis (a few-MΩ feedback resistor on U12B) can be added; with the limiter's fast action this is normally unnecessary.

---

## 11. Bill of Materials

**Bias-T & RF/DC combining**

| Ref | Part | Value |
|-----|------|-------|
| L1 | RF choke | 56 nH, 250 mA, 820 mΩ DCR |
| C26 | capacitor | 1 µF |
| FB2 | ferrite bead | 120 Ω @ 100 MHz |
| C27 | capacitor | 0.1 µF |

**Antenna supply & current limit**

| Ref | Part | Value |
|-----|------|-------|
| Q2 | NSS40300MZ4T1G | PNP power pass transistor (40 V, 3 A, 2 W) |
| Q1 | BC857W | PNP sense transistor |
| R30 | resistor | 3.3 Ω, 1 %, ≥0.25 W (0805) |
| R29 | resistor | 220 Ω |
| R28 | resistor | 2.2 kΩ |

**Supervisor**

| Ref | Part | Value |
|-----|------|-------|
| U11 | INA181A1IDBVR | gain 20 current-sense amp |
| U12 | LMV393 | dual comparator (U12A DETECT, U12B SHORT, U12C power) |
| R31 / R32 | resistors | 100 k / 11 k (DETECT ref, 0.33 V) |
| R33 / R34 | resistors | 100 k / 100 k (node-A ÷2) |
| R36 / R37 | resistors | 100 k / 30 k (SHORT ref, 0.76 V) |
| R39 | resistor | 10 k (SHORT pull-up → 3.3V_GPS) |
| R40 | resistor | 10 k (DETECT pull-up → 3.3V_GPS) |
| C28–C31 | capacitors | 0.1 µF (decoupling) |

**ANT_OFF disable stage**

| Ref | Part | Value |
|-----|------|-------|
| Q4 | BC847W | NPN |
| Q5 | BC857W | PNP |
| R38 | resistor | 10 k (ANT_OFF → Q4 base) |
| R41 | resistor | 4.7 k (Q4 collector → Q5 base) |
| R42 | resistor | 10 k (Q5 base → V_ANT) |
| R43 | resistor | 100 k (Q4 base → GND, default state) |
