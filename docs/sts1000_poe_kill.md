# STS1000 POE_KILL — AUX-Actuated PD Cold-Cycle

## Function

`POE_KILL` is the last-resort hardware recovery for a wedged board: it forces a full PoE power cold-cycle by commanding the NCP1095 (U8) pass switch off. Triggered by the MCU (`POE_KILL`, PE15) or the external watchdog (`WDO_N`, TPS3430WDRC `nWDO`). Self-sustaining across MCU death; default-RUN if all sources are inactive.

The actuator is the NCP1095 **AUX pin** (the datasheet-sanctioned pass-switch disable), not the PGATE gate. The PGATE-clamp approach is rejected — it fights the gate driver, exceeds the PGATE 11 V abs-max risk envelope on release, and triggers an overcurrent latch into the discharged port cap.

## Recovery model (the governing constraint)

The NCP1095 has **no in-band restart**. Every off-path — UVLO, overcurrent, thermal, and AUX-disable — drops the device to the *offline* state and holds there until the port voltage falls below `Vrst` (2.81 / 3.85 / 4.9 V). Recovery is therefore always via the PSE removing power, then re-detecting:

```
AUX asserted (VAUX > AUX_H)
  → pass switch off → device offline (NCM/NCL/LCF float), GBR driven low
  → buck input return opens → 3V3 collapses → MCU dies
  → PD draw < MPS (10 mA) → PSE disconnects after MPS dropout (≤310 ms, LCF low)
  → Vport < Vrst → device resets to idle
  → PSE re-detect / classify / inrush / DC-DC start → board cold-starts
```

Consequence: the on-board hold cap (C169) does **not** need to span the MPS window. It only needs to (a) hold `VAUX` high long enough to latch the device offline, and (b) outlive MCU collapse so the assert completes after PE15 goes Hi-Z. Once offline is latched, deasserting AUX does not restore — the PSE owns recovery.

## Connections

Net aliases: `VOUT_P` = VPP (bridge +, buck Vin+); `VOUT_N` = VPN (bridge −, **kill-driver reference**); `GND` = RTN (buck return, logic ground — **rises to VPP when the pass switch is off**); `V_KBIAS` = VOUT_N + 12 V (local); `VAUX` = NCP1095 AUX (pin 4) drive net.

### Bias rail + KILL_N (VOUT_N-referenced; alive whenever PoE present)

| Ref | Value | Connection | Note |
|---|---|---|---|
| R177 | 24.9 kΩ, ≥¼ W | VOUT_P → V_KBIAS | ~38 V drop at ~1.8 mA; 81 mW at VOUT_P = 57 V |
| D20 | 12 V Zener | K = V_KBIAS, A = VOUT_N | sets V_KBIAS = VOUT_N + 12 V |
| C165 | 1 µF | V_KBIAS → VOUT_N | bias hold |
| R194 | 100 kΩ | V_KBIAS → KILL_N | pull-up; KILL_N high = RUN |

### MCU input stage (GND/RTN-referenced)

| Ref | Value | Connection | Note |
|---|---|---|---|
| R199 | 10 kΩ | POE_KILL (PE15) → Q22.B | |
| R201 | 100 kΩ | Q22.B → GND | default-RUN pulldown; **no +3V3 pull-up** (PE15 is push-pull) |
| Q22 | BC847W (NPN) | B; E = GND; C = KILL_N | POE_KILL high → KILL_N low |

### WDT input stage (GND/RTN-referenced; double inversion for active-low nWDO)

| Ref | Value | Connection | Note |
|---|---|---|---|
| R205 | 10 kΩ | +3V3 → WDO_N | open-drain pull-up; **taps WDO_N, ahead of R202** (keeps nWDO ≤ 3V3) |
| R202 | 10 kΩ | WDO_N → Q23.B | series base |
| Q23 | BC847W (NPN) | B; E = GND; C = WDT_REQ | inverter |
| R203 | 10 kΩ | +3V3 → WDT_REQ | inverter collector pull-up |
| R204 | 10 kΩ | WDT_REQ → Q24.B | series base |
| Q24 | BC847W (NPN) | B; E = GND; C = KILL_N | WDO_N low → KILL_N low |

### Sustain + AUX driver (V_KBIAS / VOUT_N-referenced)

| Ref | Value | Connection | Note |
|---|---|---|---|
| R195 | 10 kΩ | KILL_N → Q21.B | base series |
| Q21 | BC857W (PNP) | E = V_KBIAS; B; C = HOLD | KILL_N low → HOLD → V_KBIAS |
| C169 | 1 µF | HOLD → VOUT_N | off-time / survives MCU death |
| R196 | 180 kΩ | HOLD → VOUT_N | bleed; default-RUN |
| R197 | 10 kΩ | HOLD → VAUX | divider top |
| R198 | 100 kΩ | VAUX → VOUT_N | divider bottom (parallels AUX 265 k internal pd) |
| C170 | 47 pF | VAUX → VOUT_N | AUX filter (per datasheet Cb) |

### NCP1095 front end (context; on the PD sheet)

| Ref | Value / Part | Key connections |
|---|---|---|
| U8 | NCP1095DBR2 | AUX(4) ← VAUX; PGATE(10) → Q1.G; PSNS(9) → Q1.S / R8 top; RTN(12) → GND; VPP(1) → VOUT_P; VPN(8) → VOUT_N; ACS(6) → VPN; GBR(11) → FDMQ8205A GDC; PGO(14) open-drain → buck EN (pull-up to VPP) |
| Q1 | FDMC8622 (100 V / 40 mΩ) | D = RTN(GND), S = R8 top / PSNS, G = PGATE |
| R8 | 25 mΩ | Q1.S → VOUT_N (RSNS) |
| R5 / R6 | 232 Ω / 909 Ω | CLA(2)→VPN / CLB(3)→VPN — Class 6 |
| CPD | 10 µF / 80 V | VPP → RTN (PD bulk) |
| C1 | 100 nF / 100 V | VPP → VPN |
| D1 | SMBJ58A | VPP → VPN (TVS) |

## Reference domains

Two domains meet only at `KILL_N`:

- **VOUT_N (VPN) domain** — bias rail, Q21, HOLD network, VAUX. Stable through a kill (VPN is fixed; VOUT_P survives → V_KBIAS survives).
- **GND (RTN) domain** — Q22 / Q23 / Q24 inputs, POE_KILL, WDO_N, +3V3. **RTN floats to VPP when the pass switch is off** (datasheet off-state: RTN = VPP).

All input-stage transistors are emitter-to-GND with base pulldowns to GND. This is mandatory: it makes base and emitter track the floating MCU domain together, so when RTN rises during the kill the stages do **not** spuriously re-assert. (A VOUT_N-referenced input stage would latch on via the POE_KILL→R199 path as RTN rises.) KILL_N pulled to RTN (≈ VOUT_N at the assert instant) is close enough to turn Q21 on.

## Operation

| State | KILL_N | Q21 | HOLD | VAUX | Result |
|---|---|---|---|---|---|
| RUN | high (R194) | off | low (R196) | ~0 | AUX low → NCP1095 normal → Q1 on → buck runs |
| KILL (MCU) | low (Q22) | on | V_KBIAS | ~10.5 V | AUX > AUX_H → pass switch off → offline |
| KILL (WDT) | low (Q24) | on | V_KBIAS | ~10.5 V | same |
| Restore | — | — | decays | → 0 | offline persists; PSE disconnects on MPS loss → Vrst → re-detect |

`VAUX` divide: HOLD (≈ 11.9 V) × R198∥265 k / (R197 + R198∥265 k) ≈ **10.5 V** — above AUX_H (2.6 V max), far below AUX abs-max (72 V vs VPN). At rest HOLD is pulled to VOUT_N, so VAUX ≈ 0 during detection/classification (no false assert).

## Timing

| Parameter | Value | Basis |
|---|---|---|
| HOLD decay τ | ~57 ms | C169 1 µF × (R196 180 k ∥ [R197 + R198∥265 k] ≈ 82.6 k) |
| VAUX assert hold (to AUX_L) | ~150 ms | τ · ln(11.9 / 0.85) |
| MPS dropout (LCF low / Type 3) | ≤ 310 ms | datasheet Table 3 |
| MPS dropout (LCF open / Type 1-2) | ≤ 250 ms | datasheet Table 3 |
| PSE re-detect → DC-DC start | up to ~1.5 s | datasheet Fig 7 (TPON 0-400 ms + class + inrush + DC-DC) |

~150 ms VAUX assert comfortably outlives MCU collapse (<10 ms) and any reasonable AUX debounce; the device is offline well before the PSE disconnects, so exact off-time is non-critical.

## NCP1095 facts that constrain the design (datasheet-verified, Rev 2, Aug 2024)

| Fact | Value / cite |
|---|---|
| AUX disables pass switch → offline; needs Vport < Vrst to re-enter idle | p.15 |
| AUX_H / AUX_L / hysteresis / internal pulldown / abs-max | 1.7–2.6 V / 0.5–1.05 V / 1.4 V / 265 k / 72 V (vs VPN) — p.5, p.7 |
| AUX high → GBR low immediately (GreenBridge disable) | p.15 |
| All off-paths latch offline until Vport < Vrst | UVLO/OC/thermal p.12; AUX p.15 |
| Vrst | 2.81 / 3.85 / 4.9 V (VPORT falling) — p.5 |
| Overcurrent: hard VOC 1.2 V / 18 µs, soft IOC 6.4 A / 960 µs | p.12 |
| Off-state: RTN = VPP | p.7 |
| PGATE abs-max | 11 V vs VPN — p.5 |
| Class 6 = RCLASSA 232 Ω / RCLASSB 909 Ω, assigned 51 W | Table 1, p.9 |
| Pass FET FDMC8622 = 100 V / 40 mΩ; RSNS 25 mΩ; CPD 10 µF/80 V; C1 100 nF/100 V; D1 SMBJ58A | BOM p.8 |
| PGO open-drain MUST gate buck EN; pull-up referenced to RTN | p.10, Fig 5 |
| ACS to VPN disables Autoclass (single-signature) | p.3, p.12 |

## Design rules

1. **AUX is the kill actuator** — never clamp PGATE. PGATE-hijack fights the gate driver and OC-latches on release.
2. **Input stages are GND-referenced** (emitter + base pulldown to GND) so they track the floating RTN domain and do not re-assert during the kill.
3. **No +3V3 pull-up on POE_KILL** — PE15 is push-pull; a pull-up forces default-KILL/boot-loop when the pin is Hi-Z at power-up.
4. **WDT pull-up taps WDO_N, not the Q23 base node** — open-drain nWDO must be cleanly pulled to 3V3; a pull-up after the series resistor leaves Q23 half-on during a fault and blocks the kill.
5. Recovery is PSE-driven (MPS dropout → re-detect). On-board RC only triggers + survives MCU death.
6. Reuse standard BJTs: BC847W (NPN), BC857W (PNP). 10 kΩ base series, 100 kΩ base pulldown.

## Open bench items

1. **GBR / VOUT_P survival (critical).** AUX-assert drives GBR low. Confirm the FDMQ8205A reverts to body-diode rectification so VOUT_P (hence V_KBIAS) survives the kill and the driver stays biased. If GBR-low instead collapses VOUT_P, the result is an offline↔reset oscillation rather than a clean MPS-dropout disconnect. Confirm GBR → FDMQ8205A GDC wiring and disabled-state behavior.
2. **AUX assert duration.** Datasheet specifies AUX must stay high "a sufficient amount of time" without a number. Confirm the ~150 ms VAUX hold reliably latches the device offline on the actual unit.
3. **CPD bulk present.** Confirm the 10 µF / 80 V CPD bulk exists at VPP→RTN (the schematic's C3 0.1 µF is the C1 decouple, not the bulk).
4. **PGO → buck EN.** Confirm PGO gates the main buck enable with a cold-start-valid pull-up referenced to VPP/pre-reg (not 3V3).
5. **ACS → VPN.** Confirm ACS tied to VPN (Autoclass disabled).
6. R177 rated ≥ ¼ W.
