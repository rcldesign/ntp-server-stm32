# STS1000 — Power-Fail Input (PFI) Comparator

Early-warning detector for loss of input (PoE) power. A single LMV393 half
(U2A) monitors the upstream PD bus (`VOUT_P`) through a high-impedance divider
and asserts **PFI** (PE8 / EXTI8) low when the bus droops below ~37 V — well
before the on-chip PVD or any 3.3 V-domain supervisor would react. Because
`VOUT_P` carries the largest bulk hold-up energy in the system and collapses
first when the PSE removes power, tripping here buys the firmware the full
hold-up window of the port capacitance to flush state and park the disciplining
loop before the rails fall out of regulation.

PFI is annunciation/interrupt only — it takes **no autonomous action**. It is
distinct from POE_KILL (a deliberate, firmware-commanded PoE cold-cycle) and
from the external watchdog; see §7.

---

## 1. Signal flow

```
 VOUT_P ──R2(1.2M)───┬───────────────► +IN (U2A pin 3)
   (PD bus, ~54 V)    │                     │
                      R3(39.2k)           C3(1nF)──GND
                       │                   R6(2M)──► OUT (hysteresis)
                      GND
 3V3 ──R4(174k)──────┬───────────────────► −IN (U2A pin 2)
                      R5(100k)             │
                       │                   C4(0.1µF)──GND
                      GND
                 Vref = 1.204 V

 U2A OUT (pin1, open-drain) ──R7(1k)──┬──► PFI ──► PE8 / EXTI8
   (supplied 5V)                      R8(100k)──► 3V3   (pull-up = logic level + boot default)

 high = power good   |   low = power fail   |   EXTI8 falling edge = fail
```

`VOUT_P` good → +IN > Vref → open-drain output released → R8 holds PFI = 3.3 V
(good). `VOUT_P` droops below trip → +IN < Vref → output sinks → PFI low (fail).

---

## 2. Bill of materials

| Ref | Value / spec | Function |
|---|---|---|
| U2A | LMV393 (½), **V+ = 5 V** | Power-fail comparator. Open-drain output. |
| U2B | LMV393 (½), unused | Spare half — inputs strapped to defined levels (pin 5 → GND, pin 6 → 5 V), pin 7 open. |
| U2C | LMV393 power unit | V+ (pin 8) = 5 V, V− (pin 4) = GND. |
| C5 | 0.1 µF | U2 supply bypass. |
| R2 | **1.2 MΩ** 1%, 0805/1206, ≥100 V WV | Sense divider top (VOUT_P → +IN). |
| R3 | **39.2 kΩ** 1% | Sense divider bottom (+IN → GND). |
| C3 | 1 nF | +IN noise filter (RC ≈ 39 µs with R3). |
| R4 | **174 kΩ** 1% | Reference divider top (3V3 → −IN). |
| R5 | **100 kΩ** 1% | Reference divider bottom (−IN → GND). |
| C4 | 0.1 µF | Reference-node bypass. |
| R6 | **2.0 MΩ** 1% | Hysteresis, OUT → +IN. |
| R7 | **1.0 kΩ** | Series isolation / current-limit, OUT → PE8. |
| R8 | **100 kΩ** | Pull-up, PFI → 3V3. Mandatory (open-drain). Sets logic-high level and boot default. |

---

## 3. Operation

### 3.1 Sense divider and trip point

Divider ratio:

  k = R3 / (R2 + R3) = 39.2k / 1.2392M = **0.031633**

Reference (from 3V3):

  Vref = 3.3 × R5 / (R4 + R5) = 3.3 × 100 / 274 = **1.204 V**

Nominal trip (no hysteresis): VOUT_P = Vref / k = 1.204 / 0.031633 = **38.1 V**.

+IN node voltage across the operating range (sets the comparator common-mode
point):

| VOUT_P | +IN |
|---|---|
| 54 V (nominal) | 1.71 V |
| 57 V (PoE max) | 1.80 V |
| 60 V (SELV ceiling) | 1.90 V |

### 3.2 Common-mode range — why 5 V supply

The LMV393 input common-mode range is **0 to (V+ − 1.5 V)**. At 5 V that is
0–3.5 V, which contains the +IN operating point (≤1.90 V) with margin. **At
3.3 V the ceiling would be 1.8 V** — exceeded by +IN at the top of the PoE
range (1.80 V @57 V, 1.90 V @60 V), risking output phase-reversal and a spurious
fail. The comparator is therefore supplied from **5 V** even though its logic
output must reach a 3.3 V pin; the open-drain output stage makes the supply rail
and the output-high level independent (§3.4). This matches the U25 / U41 LMV393
convention elsewhere in the design (5 V supply, output pulled to a 3.3 V rail).

### 3.3 Hysteresis

R6 (2 MΩ) injects positive feedback from OUT to +IN. With the open-drain
output swinging 0 ↔ 3.3 V (set by R8):

  ΔVOUT_P ≈ R2 × (V_OH − V_OL) / R6 = 1.2M × 3.3 / 2M ≈ **2.0 V**

Resulting thresholds (KCL at +IN, crossing at Vref):

| Event | Output state before | VOUT_P threshold |
|---|---|---|
| **Assert (fail)** | high (good) | **≈ 36.8 V** |
| **De-assert (re-arm, good)** | low (fail) | **≈ 38.8 V** |

Both sit below the IEEE 802.3bt Type-3 PD operating floor (~42.5 V), so no
false trip on a long-cable / low-PSE-output installation, and the 2 V band
prevents chatter on the slow bus decay.

### 3.4 Output stage and level translation

U2A is **open-drain**. The output transistor only sinks; the high level is set
entirely by R8's pull-up rail (3V3). Pulling the drain to 3.3 V while the
chip is supplied from 5 V is safe — there is no conduction path from the 5 V
supply to the output node. R7 (1 kΩ) sits between the output pin and the PFI
net for ESD/fault current-limiting at PE8; a CMOS GPIO input draws negligible
current, so it introduces no divider error. With R7 ahead of the pull-up node,
the asserted-low level is 3.3 V × R7/(R7+R8) ≈ 33 mV — a solid logic low.

### 3.5 Why VOUT_P (node choice and hold-up)

`VOUT_P` is the NCP1095 post-hotswap PD bus — the common input to all downstream
bucks and the node carrying the port bulk capacitance. It collapses first and
fastest when the PSE removes power, while the 5 V and 3.3 V bucks hold regulation
off the sagging bus until it nears their dropout (~5–6 V). Detecting the droop
here, far above any buck dropout, yields the maximum lead time. The comparator's
own rails (5 V supply, 3.3 V reference + pull-up) remain valid throughout the
decay because their bucks stay in regulation until VOUT_P is nearly exhausted —
long after PFI has fired at ~37 V.

Constant-power hold-up after assertion:

  t = C_port × (V₁² − V₂²) / (2·P)

At the IEEE inrush-limited C_port ceiling of 180 µF, V₁ = 37 V, V₂ = 6 V
(buck min-Vin), P = 25 W: **t ≈ 4.8 ms** of MCU run-time after PFI asserts. This
is the firmware's orderly-shutdown budget (§7) and scales linearly with actual
C_port — confirm by measurement (§8).

### 3.6 Standby current

Sense divider at 54 V: 54 / 1.2392 MΩ = 43.6 µA. Reference divider: 3.3 / 274 kΩ
= 12.0 µA. Total ≈ **56 µA** — negligible against the 25 W budget.

---

## 4. Reference: IEEE 802.3bt (Type 3) anchors

| Parameter | Value |
|---|---|
| PSE port voltage | 50–57 V |
| PD operating voltage | ~42.5–57 V |
| SELV ceiling (design max) | 60 V |
| Class 6 power | 60 W PSE / 51 W PD (board budget ~25 W) |
| Port capacitance (PSE-limited inrush) | < 180 µF |

Trip is set below 42.5 V; divider is rated to 60 V continuous (R2 single 1.2 MΩ
at ≥100 V WV clears 60 V with ≥1.6× margin and survives a post-TVS transient
toward the ~63 V clamp).

---

## 5. Firmware implications

**Pin / EXTI.** PFI = PE8, EXTI line 8. Configure as input, **falling-edge**
trigger. Direct MCU pin — **not** routed through the U54/U55 I/O expanders (it
must remain valid while the expanders and their bus may be browning out).

**Boot sequence (avoid the start-up race).**
1. As VOUT_P rises, the 5 V and 3.3 V rails come up and R8 holds PFI high
   (good) before the comparator output is valid — so the line is never floating.
2. In init, configure PE8 as input with internal pull-up enabled.
3. **Read the PFI level and confirm "good" (high) before arming EXTI8.** Do not
   arm the interrupt on a level that is already low. This prevents latching a
   phantom fail from the supply-ramp window.

**ISR (orderly shutdown), in priority order.** The ~4.8 ms (at 180 µF) budget is
tight; keep the handler short and pre-staged:
- Freeze / hold the OCXO disciplining loop (latch last good Vc; stop integrating).
- Flush volatile timing state, holdover/aging estimates, and the event log to NOR.
- Quiesce the Rb subsystem if enabled (RB_PWR_EN low) — its warm-up surge is the
  largest concurrent load and shedding it extends hold-up.
- Set a clean shutdown flag for the next boot, then spin/halt; let the rails
  decay. Do **not** attempt long flash writes or network teardown that exceed the
  budget.

**Relationship to other supervisors.**
- **PFI is the primary early warning.** It fires on the 54 V bus, milliseconds
  before the 3.3 V domain moves.
- **On-chip PVD** (programmable voltage detector, VDD domain) is the backstop if
  PFI is missed or the decay is unusually fast; treat a PVD event as a hard,
  abbreviated shutdown.
- **BOR** resets the MCU at the floor — last resort.
- **External watchdog (TPS3430)** is unrelated to power-fail; do not gate watchdog
  servicing on PFI state.
- **POE_KILL** is a deliberate cold-cycle, not a fault — firmware drives it and
  expects the resulting power loss; the PFI ISR should distinguish a commanded
  POE_KILL from an unexpected line drop (e.g., via a "kill armed" flag) so the two
  paths log differently.

**De-assert / recovery.** On re-arm (PFI returns high at ~38.8 V), power was
restored before full collapse. Firmware may treat this as a brown-out recovery:
re-validate rails (INA228 reads), resume disciplining, and clear the pending
shutdown flag if the MCU never actually lost power.

---

## 6. Design rules captured

- **Supply the comparator from 5 V**, output pulled to 3V3 — the open-drain
  stage decouples supply rail from logic level and the 5 V CMR (0–3.5 V) clears
  the +IN operating point that a 3.3 V supply would not.
- **Single 1.2 MΩ top resistor** at ≥100 V WV — splitting for derating is
  unnecessary on this node (one part clears 60 V with margin; voltage-coefficient
  error is a fraction of a percent, irrelevant to a ±2 V trip).
- **Trip below the PD operating floor (42.5 V), not into it** — to buy more
  hold-up, add capacitance on the 3.3 V/5 V rail; never raise the trip into the
  operating band.
- **High = good, low = fail, driven asserting edge** — the comparator actively
  sinks the fail edge rather than relying on the (also-sagging) pull-up rail; the
  pull-up only defines the benign default.
- **Direct MCU pin, not the expanders** — the annunciation path must survive a
  browning-out bus.

---

## 7. Open items / bench verification

- [ ] Confirm `VOUT_P` resolves to the **NCP1095 post-hotswap PD bus** and the
      divider tap is **downstream of the PoE input TVS/surge clamp**.
- [ ] Measure actual **C_port** and recompute the hold-up window; confirm it
      exceeds the worst-case ISR shutdown time with margin. If short, add 3.3 V/5 V
      rail hold-up capacitance.
- [ ] Confirm **U2B** pin 5 / pin 6 land on **opposite** rails (5 V / GND) and
      are not commoned (no rail-to-rail short through the strap).
- [ ] Verify buck **min-Vin** (V₂ in the hold-up calc) for the 3.3 V rail.
- [ ] Bench-confirm no false trip at the **low end of the PoE range under a load
      transient** (long-cable + Rb warm-up step); adjust R6 if the 2 V band is
      insufficient.
- [ ] Measure **boot-to-EXTI-arm** time and confirm the read-before-arm guard
      eliminates any ramp-window false trip.
