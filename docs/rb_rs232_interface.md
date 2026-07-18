# Rubidium Reference — Serial & Control Interface Design Notes

Covers the FE-5680A digital interface subsystem: the RS-232 ⇄ logic transceiver, the
dual-mode (RS-232 / direct-CMOS) DPDT routing, the DB9-edge ESD/EMI front end, the
relay driver, and the firmware behavior for both the serial link and its control
signals.

This is the **digital telemetry/control path** to the Rb, plus the **lock-status
input** (§6A). The Rb's 10 MHz output is handled separately (RF conditioning chain)
and is out of scope here except where it interacts with grounding.

---

## 1. Purpose

The FE-5680A Option 2 exposes a serial port (frequency offset get/set, EEPROM save)
on its DB9 power connector, pins J1-8 and J1-9, referenced to J1-5. The MCU reaches it
on **UART7** — `RB_UART_TX = PB4`, `RB_UART_RX = PE7`. This subsystem connects those
two domains while solving three problems specific to surplus FE-5680A hardware:

1. **Level uncertainty.** The manual specifies RS-232C levels, but surplus Option 2
   units frequently ship with logic-level (CMOS/TTL) UART on the same J1-8/9 pins, and
   occasionally with inverted polarity. The hardware must support **both** RS-232 and
   direct-CMOS without a rebuild.
2. **Field-port robustness.** The DB9 is an externally cabled connector to a separate
   chassis, so it needs proper ESD and surge protection on both data lines.
3. **Ground integrity.** The Rb runs on its own high-current supply with a large
   warm-up surge. The serial return must be the single, controlled bond between Rb
   ground and board ground — see §6.

---

## 2. Naming convention (read this first)

Net names are **FE-relative** — named for the FE-5680A's own port direction, *not* the
host's:

| Net | Meaning | Physical direction |
|---|---|---|
| `RB_RS232_TX` | the **FE's transmit** (its serial output) | FE → board |
| `RB_RS232_RX` | the **FE's receive** (its serial input) | board → FE |
| `RB_UART_TX` (PB4) | the **MCU's transmit** | MCU → board |
| `RB_UART_RX` (PE7) | the **MCU's receive** | board → MCU |

`_P` suffix (`RB_RS232_TX_P`, `RB_RS232_RX_P`) = the **protected** node, i.e. the same
signal after the DB9 front-end ESD/filter stage.

> On the schematic, `RB_UART_TX`/`RB_UART_RX` may appear abbreviated as `RB_RX`/`RB_TX`
> at the transceiver/relay block. These follow the **same FE-relative logic**:
> the net feeding the MCU's transmit into the board is the FE's receive path, and the
> net the MCU listens on carries the FE's transmit. `RB_RX` (into DIN) = MCU TX =
> data toward the Rb; `RB_TX` (from the relay COM) = MCU RX = data from the Rb.

---

## 3. Data-direction mapping (authoritative)

The FE manual labels J1-8/J1-9 as "Rx/Tx" without stating whose perspective, and the
prose (§2-3.1) does not disambiguate the label letters. What **is** unambiguous is the
data direction on each wire:

- **J1-9 carries commands INTO the FE** (host→FE). The FE listens here.
- **J1-8 carries responses OUT of the FE** (FE→host). The FE drives here.

The board is wired to that direction fact, independent of the table's Tx/Rx text:

| FE J1 pin | Data direction | Board net | Transceiver pin |
|---|---|---|---|
| J1-9 | board → FE (FE input) | `RB_RS232_RX` | driven by **DOUT (13)** |
| J1-8 | FE → board (FE output) | `RB_RS232_TX` | feeds **RIN (8)** |
| J1-5 | reference | board GND (single bond) | GND (14) |

**If a specific unit measures reversed** relative to §2-3.1 on the bench, fix it in the
**mating DB9 cable** (swap pins 8↔9), not on the board — DOUT/RIN are hardwired.

---

## 4. Transceiver — SN65C3221E (U46)

Single-channel RS-232 driver/receiver with charge pump and ±15 kV IEC ESD on the line
pins. Operates from 3.3 V; accepts 3.3 V logic in and produces ±5.4 V RS-232 swing.

### 4.1 Strapping

| Pin | Name | Connection | Rationale |
|---|---|---|---|
| 15 | VCC | always-on 3.3 V | powered whenever the board is up |
| 14 | GND | board GND | the single Rb↔board bond point (§6) |
| 1 | EN̄ | **GND** | receiver permanently enabled — MCU must always be able to hear the FE |
| 16 | FORCEOFF̄ | `RB_PWR_EN` net, 100 kΩ pulldown (R164) | active-high enable; default-off until firmware asserts |
| 12 | FORCEON | 3.3 V | **auto-powerdown disabled** — no dependence on sensing a valid RS-232 level |
| 10 | INVALID̄ | NC | unused (auto-powerdown off) |
| 11 | DIN | `RB_RX` (= MCU TX, PB4 path) | data toward the Rb |
| 13 | DOUT | relay Pole A:NC | RS-232 driver output → FE input |
| 8 | RIN | `RB_RS232_TX_P` | FE output → receiver |
| 9 | ROUT | relay Pole B:NC | received data → MCU (RS-232 mode) |

**Auto-powerdown is deliberately off.** FORCEON tied high disables it; the part's
on/off is controlled solely by FORCEOFF̄ via `RB_PWR_EN`. This avoids the receiver
gating itself off when the FE link is briefly idle, and removes any dependence on the
FE presenting a "valid" RS-232 level (it may be CMOS).

**EN̄ tied to GND** keeps the receiver enabled at all times the part is powered. (EN̄
high would tri-state ROUT — the MCU would go deaf to the FE. It is *not* used for
power control; FORCEOFF̄ does that.)

### 4.2 Charge pump & supply caps

| Designator | Value | Pin(s) | Note |
|---|---|---|---|
| C139 | 0.1 µF | C1+ / C1− (2,4) | flying cap |
| C140 | 0.1 µF | C2+ / C2− (5,6) | flying cap |
| C144 | 0.1 µF | V− (7) → GND | negative reservoir |
| C145 | 0.1 µF | **V+ (3) → GND** | positive-doubler reservoir — the standard SN65C3221E doubler reservoir (V+→GND). |
| C143 | **1 µF** | VCC (15) → GND | bulk/bypass — see note |

> **The 1 µF on VCC is mandatory, not optional.** U46 is the **PW (TSSOP)** package.
> The datasheet IEC-ESD spec (§7.3) requires a minimum 1 µF between VCC and GND for the
> PW package to meet the rated ±8 kV contact / ±15 kV air discharge on the line pins.
> Dropping below 1 µF silently forfeits the field-port ESD rating that is the entire
> reason for choosing the "E" (ESD) variant.

---

## 5. Dual-mode routing — DPDT relay K1 (G6K-2F-Y DC3)

A 2-coil... no — a single-coil **3 V** DPDT signal relay selects between the RS-232
path (through the transceiver) and a direct-CMOS path (transceiver bypassed). The
design principle: **switch the two dual-source nodes, not the two line wires
symmetrically.**

- **Pole A** selects which driver feeds the FE input (J1-9 / `RB_RS232_RX_P`):
  the transceiver's DOUT, or the MCU's raw TX.
- **Pole B** selects which source feeds the MCU input (`RB_TX` → PE7):
  the transceiver's ROUT, or the FE's raw output.

`DIN` and `RIN` stay **permanently** wired to the MCU TX path and `RB_RS232_TX_P`
respectively — they are never switched. In CMOS mode the transceiver still toggles
DOUT into an open NC contact (harmless) and still drives a dangling ROUT (harmless).

### 5.1 Contact map (de-energized = NC = RS-232 = power-on default)

| Pole | COM | NC (RS-232) | NO (CMOS) | Selects |
|---|---|---|---|---|
| A | J1-9 (`RB_RS232_RX_P`) | DOUT (13) | `RB_RX` via R167 220 Ω | driver feeding FE input |
| B | `RB_TX` (→ PE7) | ROUT (9) | `RB_RS232_TX_P` via R168 10 kΩ | source feeding MCU input |

**De-energized defaults to RS-232**, which matches the FE manual's documented level, so
the link is in its safest/most-likely-correct state before firmware runs.

### 5.2 Series resistors

| Designator | Value | Branch | Purpose |
|---|---|---|---|
| R167 | 220 Ω | Pole A:NO (MCU TX → FE input, CMOS) | edge/ringing & current limit; no MCU hazard (J1-9 is an FE input) |
| R168 | 10 kΩ | Pole B:NO (FE output → MCU, CMOS) | **fault current limit** — see below |

R168 is the protection-critical part. In CMOS mode the FE's raw output reaches PE7
through this 10 kΩ. If the unit is actually RS-232 and the relay is energized to CMOS
by mistake, ±5.4 V appears at R168's input; the 10 kΩ limits the current the clamp
(§5.3) must shunt to ≈ (5.4 − 3.6)/10 k ≈ 0.18 mA — well inside both the Schottky and
the STM32 pin-injection limits.

### 5.3 MCU-pin clamp — BAT54S (D12)

`RB_TX` connects **directly to the STM32 (PE7)** — no series resistor on the RB_TX net
itself. The clamp sits right at the pin:

- BAT54S series pair: A (pin 1) → GND, K (pin 2) → +3.3 V, COM (pin 3) → `RB_TX`.
- Negative excursions below GND−0.3 V → lower diode conducts → clamped to ≈ −0.3 V.
- Positive excursions above 3.3 V+0.3 V → upper diode conducts → clamped to ≈ 3.6 V.

The only over-rail energy that can reach `RB_TX` arrives through the Pole B:NO branch
(R168 10 kΩ); the NC branch (ROUT) is a 0–3.3 V logic output and never a hazard. So the
current limit lives on the relay branch (R168) and the clamp lives on the pin (D12) —
together they bound PE7 to 0–3.3 V in every mode and fault combination. No additional
series R on `RB_TX` is wanted (it would only add delay/loading to the normal ROUT
path).

### 5.4 Polarity outcome

Routing the dual-source nodes (rather than the line wires) makes **both modes resolve
to the same UART polarity**, so firmware does not have to flip inversion between modes:

| Mode | FE-input driver | MCU-input source | TXINV | RXINV |
|---|---|---|---|---|
| RS-232 (NC) | DOUT, ±5.4 V | ROUT (receiver de-inverts) | 0 | 0 |
| CMOS (NO) | MCU TX, 0/3.3 V | FE output direct (idle-high) | 0 | 0 |

Idle trace: MCU TX idles high → "mark" in both modes (negative through the driver in
RS-232, 3.3 V direct in CMOS). FE idle-mark → high at PE7 in both (the receiver inverts
the RS-232 negative; a CMOS idle-high passes straight through). This holds for standard
non-inverted variants of each type. **Keep firmware inversion bits available** for
oddball units (see §7).

### 5.5 Relay driver

`RB_RS232_CMOS_SW` (MCU GPIO) → R170 470 Ω → base of Q21 (BC847W, NPN) → R169 10 kΩ
base pulldown → emitter direct to GND → collector to one coil terminal; other coil
terminal to +3.3 V with D15 (BAT54J) flyback across the coil (cathode to +3.3 V).

- **Emitter is directly grounded** (no degeneration) so the transistor saturates and
  the coil sees the full rail.
- **R169 base pulldown** holds Q21 off (relay de-energized → RS-232) during reset and
  any time the GPIO is high-Z — fail-safe to the documented default.
- High on `RB_RS232_CMOS_SW` → energize → CMOS mode.

---

## 6. ESD / EMI front end & grounding (DB9 edge)

Each data line (`RB_RS232_TX`, `RB_RS232_RX`) carries, from connector inward:

| Stage | Designators | Value | Role |
|---|---|---|---|
| Primary TVS | D13, D14 | SMAJ15CA | bidirectional surge clamp at the connector (15 V standoff, passes ±5.4 V RS-232) |
| Connector-side shunt | C146, C147 | 47 pF C0G | EMI filter / edge shaping at the port |
| Series | R165, R166 | 51 Ω | isolates the connector-side cap from the transceiver node; forms RC/ferrite filter |
| Ferrite | L8, L9 | 600 Ω @ 100 MHz | HF/EMI suppression |
| Secondary TVS | U47 | PESD15VL2BT | low-cap ESD clamp at the transceiver side |
| Transceiver-side shunt | C141, C142 | 47 pF C0G | EMI filter at DOUT/RIN node |

**Shunt sizing.** Shunts are 47 pF C0G (not 150 pF) to preserve edges if the link is
ever run fast. At 3.3 V the SN65C3221E holds 1 Mbit/s only up to **CL ≤ 250 pF** at the
driver (datasheet §7.8); the dominant capacitance at high rates is the TVS junction
capacitance, then cable length. For the FE-5680A's normal serial rate (commonly
9600 8N1) none of this matters and 47 pF is comfortable. C0G is specified so the filter
corner does not move with bias/temperature.

**Grounding — single bond.** The serial return (J1-5 → transceiver GND → board GND) is
the **one intended galvanic bond** between the Rb domain and board ground. To keep it
the *only* bond:

- The Rb's high-current supply return must go back to its own buck/brick, **not** share
  the J1-5 path (the warm-up surge must never flow in the signal return).
- The 10 MHz path is isolated by its RF transformer (separate chain), so it adds no
  bond.
- Audit that nothing else (SMA shield clamp at the bulkhead, chassis/mounting, shield
  drain) silently re-bonds Rb ground to board ground — that would form a loop.

> **FE J1-2 vs J1-5:** on the FE-5680A these are presented as two returns at the
> connector ("+15V Return" and "GROUND"), but on a single sealed module they are
> typically **internally common**. Treat as one domain bonded at J1-5; keep the J1-2
> power-return path physically separate so surge current does not ride the signal
> return. Confirm with an ohmmeter (J1-2 ↔ J1-5, power off) on the actual unit.

---

## 6A. Lock-status input — opto-isolated, voltage-tolerant (U48)

The FE-5680A loop-lock indicator (J1-3) is brought in on a separate, **opto-isolated**
path — not through the serial transceiver, and not directly to a GPIO. This serves two
goals at once: it tolerates an **unknown "high" voltage** (the FE asserts something
above ~3 V, but the exact level is not specified and varies by unit — the design covers
the full **3–24 V** range), and it keeps the lock signal off the RS-232 single-bond
ground (§6) — the optocoupler is the galvanic break, so the lock line adds no second
ground bond.

The optocoupler is an **APC-817C1** (American Bright), CTR rank **C1 = 200–400%**. The
rank matters: at the 3 V input corner the LED current is small (~0.3 mA) and CTR is
depressed (Fig 7), so a low-CTR rank (A1, 80–160%) may not reliably pull the output
low. C1 is the minimum acceptable rank.

A slow DC status line has no timing/jitter sensitivity, so an optocoupler is the right
tool here (unlike the 10 MHz path, which must never be opto-coupled).

### 6A.1 Signal chain

`RB_LOCK_IN` (from FE J1-3) → input-protection → opto LED → opto output → `RB_LOCK`
(to MCU GPIO, **PB13 / EXTI13**).

| Stage | Designator | Value | Role |
|---|---|---|---|
| Upstream series R | R171 | 4.7 kΩ, 0.25 W | limits total input current so D16 is never over-stressed across 3–24 V; ~91 mW worst-case at 24 V |
| Input clamp | D16 | 3V3 zener → GND | clamps the unknown-high input node to ~3.3 V; sinks only the residual current R171 lets through (≤0.4 mA → ~1.3 mW) |
| LED series R | R172 | 470 Ω | sets opto LED forward current (~4 mA) from the clamped node |
| LED-side catch | D17 | BAT54J → GND | reverse-protection for the LED — APC-817 V_R abs max is only **6 V**, so this clamp is **required**, not optional |
| Optocoupler | U48 | APC-817C1 (CTR 200–400%) | galvanic isolation + level translation; C1 rank min |
| Output pullup | R173 | 22 kΩ → +3.3 V | pulls `RB_LOCK` high when the opto transistor is off; high value eases the 3 V-corner pull-down |
| Output filter | C148 | 10 nF → GND | debounce (≈ R173·C148 ≈ 220 µs) for a slow status line |

Connections: `RB_LOCK_IN` → R171 → [D16 clamp node] → R172 → LED_A (1); LED_C (2) → GND.
Collector (4) → R173 pullup; emitter (3) → GND; collector node → C148 → `RB_LOCK`.

### 6A.2 Why this tolerates an unknown high voltage (3–24 V)

`RB_LOCK_IN` may be asserted at any voltage from ~3 V up to ~24 V — the FE's high level
is not specified and varies by unit. Two elements handle this:

- **R171 (4.7 kΩ upstream)** limits the *total* input current, so the FE's drive is
  bounded regardless of how high it swings.
- **D16 (3V3 zener)** clamps the LED-side node to ~3.3 V, sinking only the residual
  current that R171 lets past.

The APC-817 CTR is specified at **I_F = 5 mA**, and Fig 7 shows CTR rolling off above
~10 mA and below ~1 mA. The target is therefore to keep I_F in the ~1–5 mA band across
the whole input range; the clamp is what makes that possible (without it, a single
resistor cannot span the 14× current ratio between 3 V and 24 V — see note).

**High-voltage regime (input ≥ ~3.3 V, D16 clamping).** The LED current is set by R172
from the *clamped* node, independent of input voltage (V_F ≈ 1.4 V max from the
APC-817 EOC table):

> I_F ≈ (3.3 − 1.4) / 470 ≈ **4.0 mA** — right at the 5 mA CTR spec point.

R171 bounds the total current; D16 sinks the small remainder:

| V_in | I_total = (V_in − 3.3)/4.7 kΩ | I_D16 = I_total − I_F | D16 dissipation (~3.3 V · I_D16) |
|---|---|---|---|
| 5 V | 0.36 mA | LED not yet at 4 mA; D16 idle | ~0 |
| 12 V | 1.85 mA | LED dominates; D16 ~0 | ~0 |
| 15 V | 2.49 mA | LED dominates; D16 ~0 | ~0 |
| **24 V** | **4.40 mA** | **~0.4 mA** | **~1.3 mW** |

At 24 V the total (4.4 mA) barely exceeds the LED's 4.0 mA, so D16 only ever sinks
~0.4 mA → ~1.3 mW, trivial vs. its rating. **R171 dissipation at 24 V** ≈
(4.4 mA)² · 4.7 kΩ ≈ **91 mW** → spec R171 at **0.25 W** (0805/1206 body). LED current
~4 mA is far under the 60 mA / 100 mW input abs-max.

**Low-voltage regime (input < ~3.3 V, D16 not clamping).** The LED current is set by
R171 + R172 in series:

> I_F ≈ (V_in − 1.4) / (R171 + R172) = (V_in − 1.4) / 5.17 kΩ

| V_in | I_F |
|---|---|
| 3.0 V | **0.31 mA** |
| 3.3 V | 0.37 mA |

This is the weak corner. At ~0.3 mA, Fig 7 shows normalized CTR depressed to roughly
0.5–0.6× the 5 mA value; with the **C1 rank** (200% min) that still gives effective
CTR ≈ 100–120%, so I_C ≈ 0.3–0.4 mA. The transistor only needs to sink ~3.3 V / 22 kΩ
≈ **0.15 mA** to pull `RB_LOCK` to V_CE(sat) (≤0.2 V), so there is ~2× margin even at
3.0 V. This is why **R173 = 22 kΩ** (not 10 kΩ) and why the **C1 rank is the minimum** —
the two together secure the 3 V corner.

> **Single-resistor approach does not work over 3–24 V.** Without the clamp, I_F =
> (V_in − 1.4)/R. Sizing for ~3 mA at 24 V gives ~0.2 mA at 3 V (opto won't turn on);
> sizing for ~1 mA at 3 V gives ~14 mA at 24 V (past the CTR sweet spot, ~100 mW in R).
> The 14× span is irreconcilable with one resistor — the zener clamp is what decouples
> LED current from input voltage.

D16 is also bidirectional: a negative input excursion forward-biases the zener to
≈ −0.7 V, with R171 + R172 limiting current. Reverse protection of the **LED** itself is
the job of D17 — the APC-817 LED reverse-voltage abs-max is only **6 V**, so D17 is
required to catch any reverse swing before it reaches V_R.

### 6A.3 Output polarity & sense

The optocoupler **inverts**: FE lock line high → LED on → transistor on → `RB_LOCK`
pulled **low**. Combined with the FE's own ambiguous lock-sense convention (Table 3:
< 1 V = locked, > 3 V = unlocked — **reversed on Option 26 units**), the absolute
meaning of a `RB_LOCK` level cannot be assumed.

> **`RB_LOCK` polarity must be a firmware config bit.** Do not hard-code which level
> means "locked." Determine it at commissioning (correlate `RB_LOCK` against the
> serial-reported lock state, §7.6) and persist it.

### 6A.4 Verify (per physical unit)

- **FE J1-3 source impedance / drive.** R171 (4.7 kΩ upstream) bounds the input current
  so D16 sinks only the residual — sized to cover the full 3–24 V range. Confirm the
  measured J1-3 high lands inside that range; D16 dissipation stays ≤~1.3 mW and R171
  ≤~91 mW for any input up to 24 V (§6A.2). If the measured high is known and stable
  (e.g. a clean 5 V), R172 could be nudged for a touch more I_F, but the as-built set
  already works across the whole range — no change required.
- **Opto rank / CTR at the 3 V corner.** Confirm the fitted part is **APC-817C1**
  (CTR 200–400%), not A1/B1. At the 3 V input corner I_F ≈ 0.3 mA with depressed CTR;
  the C1 rank + R173 = 22 kΩ give ~2× pull-down margin. A lower rank may not pull low
  reliably at 3 V.
- **Lock polarity** (per 6A.3) → set the `RB_LOCK` sense config bit.

---

## 7. Firmware notes & requirements

### 7.1 UART configuration

- **Peripheral:** UART7, `RB_UART_TX = PB4`, `RB_UART_RX = PE7`.
- **PB4 caveat:** PB4 boots as JTAG **NJTRST**. The board must run **SWD only** (not
  full JTAG) so PB4 is free for UART7 alternate function. Ensure nothing asserts PB4
  during reset. The relay default (RS-232) routes PB4 into DIN through the transceiver,
  harmless in either reset state.
- **Verify the AF pairing:** confirm PB4 and PE7 both map to the same **UART7** TX/RX
  alternate function on the LQFP144 (ZIT6) package before layout. Note **PB4 = NJTRST**, so
  the board must be **SWD-only** (no JTAG).
- **Baud/format:** the FE manual does not state it. FE-5680A Option 2 units are commonly
  **9600 8N1** — **confirm on the actual unit.** Make baud a configurable parameter.

### 7.2 Mode selection (`RB_RS232_CMOS_SW`)

- GPIO output. **Low = RS-232** (relay de-energized, default). **High = CMOS** (relay
  energized).
- Default to **low** at boot and hold low until the link level has been confirmed.
- This is a **set-once-at-integration** selection in practice — the FE is a fixed
  external unit characterized once. There is no operational reason to toggle it at
  runtime; treat it as a commissioning/config setting, not a live control.
- **Determine the correct mode at commissioning,** not by guesswork: with the relay in
  RS-232 (default), attempt a known transaction (e.g. Request Frequency Offset, 0x2D).
  If it fails, the unit may be CMOS — switch the relay and retry. Persist the result.

### 7.3 Inversion handling

- For standard variants of each type, run **TXINV=0, RXINV=0** in both modes (§5.4).
- Keep `USART_CR2` TXINV/RXINV bits as **configurable fallbacks** for inverted variants
  (some surplus CMOS units idle low / invert data). If a unit fails to communicate in
  both relay positions at the expected baud, try toggling RXINV before concluding the
  link is dead.
- Data-order/swap anomalies (rare TX/RX swap on the DB9) are **not** fixable in firmware
  here (DOUT/RIN are hardwired to specific DB9 pins) — they are handled in the mating
  cable.

### 7.4 Enable sequencing (`RB_PWR_EN`, PB7)

`RB_PWR_EN` drives **both** the Rb-rail buck EN **and** the transceiver FORCEOFF̄
(via R164). Consequences:

- The transceiver is **only powered when the Rb rail is enabled.** Do not attempt serial
  transactions while `RB_PWR_EN` is low — the transceiver is in its 1 µA shutdown and
  ROUT is not driven.
- Follow the warm-up staging already defined for the Rb rail: bring up the OCXO first,
  defer `RB_PWR_EN` until the OCXO is warm and supercaps are charged. The serial link
  becomes available only after `RB_PWR_EN` asserts and the transceiver's supply-enable
  time elapses (≈100 µs, datasheet `ten`).
- After asserting `RB_PWR_EN`, allow the transceiver enable time **and** the FE's own
  warm-up (manual: < 5 min to lock) before trusting frequency/health telemetry.

### 7.5 FE-5680A serial protocol (Option 2)

For reference — the command set firmware must implement over this link:

| Command | ID | Length | Effect |
|---|---|---|---|
| Set freq offset, save to EEPROM | 0x2C | 9 bytes | persistent offset; **limit to ≤ 1 write/hour** (100k-write EEPROM endurance) |
| Set freq offset, no save | 0x2E | 9 bytes | volatile offset; unlimited writes — use this for any closed-loop trimming |
| Request freq offset | 0x2D | 4-byte cmd / 9-byte resp | read current offset |

- Message format: `[Command ID][Msg Len LO][Msg Len HI][Cmd checksum][Data…][Data checksum]`.
  Data checksum = XOR of data bytes. Offset is a 32-bit **signed** integer, MSB first;
  full-scale ±0x7FFFFFFF ≈ ±383 Hz; resolution ≈ 1.79 × 10⁻⁷ Hz (≈1.79 × 10⁻¹⁴
  fractional at 10 MHz).
- **Do not use 0x2C (EEPROM save) for routine discipline.** A timing-grade Rb
  self-disciplines from its GPS 1PPS input; if firmware ever trims it, use 0x2E
  (volatile) to protect EEPROM endurance. Reserve 0x2C for infrequent aging
  corrections, rate-limited to ≤ 1/hour.

### 7.6 Relation to the rest of the Rb subsystem

This serial link carries **control + telemetry only** (lock state, frequency offset,
health). It is independent of:

- **`RB_LOCK` (PB13, EXTI13)** — the hardware lock-good status used for the OCXO↔Rb
  auto-switch decision, brought in on the **opto-isolated, voltage-tolerant input
  (§6A)**. (Lock polarity is ambiguous across FE variants *and* inverted once by the
  optocoupler — keep `RB_LOCK` sense a config bit.)
- **The 10 MHz RF chain** — conditioned separately into clock-mux input B.

Firmware cross-checks: serial-reported lock/offset should agree with the hardware
`RB_LOCK` line and with the GPS-PPS cross-check of the active reference; disagreement is
a fault flag.

---

## 8. Bench-verification checklist (per physical FE unit)

| Item | How | Why |
|---|---|---|
| Serial **level** (RS-232 vs CMOS, and rail) | scope J1-8 idle: defined negative mark = RS-232; 3.3 V high = CMOS | sets relay default mode; a 5 V-CMOS input needs VIH ≈ 3.5 V → 3.3 V drive marginal |
| Serial **data direction** | scope idle with host disconnected: the FE's output sits at a defined mark | confirms J1-8/J1-9 vs §2-3.1; if reversed, fix in the **cable** |
| **Polarity** | attempt 0x2D transaction; toggle RXINV if needed | sets TX/RXINV bits |
| **Baud/format** | try 9600 8N1 first | not stated in manual |
| **Lock polarity** | measure J1-3 vs lock state | sets `RB_LOCK` config bit |
| **Lock J1-3 drive range** | measure J1-3 high voltage & source impedance | confirm it lands in 3–24 V; R171/D16 dissipation in range for the whole span (§6A.2) |
| **Opto rank = APC-817C1** | confirm fitted part CTR 200–400% | A1/B1 may not pull low at the 3 V corner |
| **J1-2 ↔ J1-5 continuity** | ohmmeter, power off | confirms single-domain grounding assumption |
| **No second ground bond** | inspect SMA shield clamp, chassis, drains | preserve single-bond integrity |
| **EN̄ = 0 V** | scope/ohm pin 1 | receiver must stay enabled |
| **G6K-2F-Y NC/NO map** | vs Omron datasheet | confirm de-energized = RS-232 throws |

---

## 9. Designator summary

| Ref | Part / value | Function |
|---|---|---|
| U46 | SN65C3221EPWR | RS-232 transceiver (TSSOP/PW package) |
| U47 | PESD15VL2BT | secondary (transceiver-side) ESD array |
| U48 | APC-817C1 (CTR 200–400%) | opto-isolated lock-status input |
| K1 | G6K-2F-Y DC3 | DPDT mode-select relay (3 V coil) |
| Q21 | BC847W | relay coil driver (NPN) |
| D13, D14 | SMAJ15CA | primary line TVS (connector side) |
| D12 | BAT54S | `RB_TX` / PE7 rail clamp |
| D16 | 3V3 zener | lock-input clamp (before LED resistor) |
| D17 | BAT54J | lock LED reverse-V protection (APC-817 V_R max = 6 V — required) |
| D15 | BAT54J | relay coil flyback |
| L8, L9 | 600 Ω @ 100 MHz ferrite | line EMI suppression |
| R165, R166 | 51 Ω | line series (filter isolation) |
| R164 | 100 kΩ | FORCEOFF̄ pulldown (default-off) |
| R172 | 470 Ω | lock opto LED series resistor (sets I_F ≈ 4 mA) |
| R173 | 22 kΩ | lock opto output pullup (eases 3 V-corner pull-down) |
| R171 | 4.7 kΩ, 0.25 W | lock-input upstream limiter (3–24 V; ~91 mW at 24 V) |
| R169 | 10 kΩ | Q21 base pulldown (relay default-off) |
| R170 | 470 Ω | Q21 base series |
| R168 | 10 kΩ | CMOS-path fault current limit (Pole B:NO) |
| R167 | 220 Ω | CMOS-path series (Pole A:NO) |
| C141, C142 | 47 pF C0G | transceiver-side line shunts |
| C146, C147 | 47 pF C0G | connector-side line shunts |
| C139, C140 | 0.1 µF | charge-pump flying caps |
| C143 | 1 µF | VCC bypass (**required for PW-package IEC ESD**) |
| C144 | 0.1 µF | V− reservoir (→ GND) |
| C145 | 0.1 µF | V+ doubler reservoir (**V+ → GND**) |
| C148 | 10 nF | lock opto output debounce (≈220 µs with R173 = 22 kΩ) |
