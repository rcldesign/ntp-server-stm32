# STS1000 "Meridian" — Hardware Design Reference

Master power-architecture and per-subsystem circuit reference: topology, governing
formulas, and the derivation of every load-bearing resistor/cap value. Keyed to the
schematic netlist + component values.

**Authority:** electrical facts here are derived from the netlist + component values +
device datasheets. Design *intent* (why a block exists, target behavior, firmware
contract) lives in the subsystem docs cross-referenced per section. Open verifications and
the schematic-to-layout package are tracked in `sts1000_schematic_design_review.md` and
`sts1000_layout_readiness_review.md`.

The MCU is **U12 STM32H563ZIT6 (LQFP144)**; all pin references below use the LQFP144 map.

---

## 1. System power architecture

### 1.1 Power tree

```
 PoE (802.3bt, Class 6, 50–57 V)
   │  RJ45 magjack J1 — all 4 pairs, Alt-A + Alt-B via center taps TR0..TR3_TAP
   ▼
 FDMQ8205A bridges  U7 + U8   (GreenBridge active rectifiers, both polarities)
   ▼  VOUT_P (VPP, ~54 V)  ───────────────────────── PD bus / bulk hold-up node
 NCP1095 U9  (PD hot-swap controller + pass FET Q2 FDMC8622, RSNS R16 25 mΩ)
   │  Class 6: RCLASSA R13 232 Ω / RCLASSB R14 909 Ω → 51 W PD signature
   ▼
 VOUT_P  ────────────────────────────────────────────────────────────────────┐
   ├─► U28 MIC28516 sync buck ───► 5V (4.99 V)                                 │
   │       ├─► U33 RT9742 load switch ─► 5V_DISP (gated, DISP_EN)             │
   │       ├─► U39 TPS7A5201 LDO ─────► OCXO 3.327 V rail (from OCXO buck U38)│
   │       ├─► U22 LT3045 LDO ────────► 3V3_GPS (3.32 V)                       │
   │       ├─► U51 LT3045 LDO ────────► 3V0_RF (3.01 V, RF island)            │
   │       ├─► U27 RT9742 load switch ─► V_ANT (gated, ANT_BIAS_EN)           │
   │       └─► U55 RT9742 load switch ─► V_PANEL_LED (gated, PANEL_LED_EN)    │
   ├─► U29 AP3441 sync buck ─────► 3V3 (3.33 V) ─► 3V3_STM (always-on) + 3V3_LAN
   │       └─► U13 LT3045 LDO ───► VDDA (MCU analog, OCXO-loop island)        │
   ├─► U38 AP3441 sync buck ─────► 3.70 V pre-reg ─► U39 LDO ─► OCXO 3.327 V  │
   └─► U40 MIC28516 sync buck ───► VCC_RB (5–24 V, digipot-trimmed) ─► Q25 gate ─► VCC_RB_G → FE-5680A

 Supercap backup (parallel to 3V3/3V3_STM):
   U34 TPS61094 buck/boost ─► STM_VBAT / GPS_VBAT (VBAT 3.0 V) ; supercaps C90/C91 (3 F)
   U35 TPS61094 (second)   ─► second backup domain ; charge term 2.5 V, 25 mA
```

Design rule of record (`net_naming`, `3v3p_i2c_peripherals`): **the gated 3V3_P rail was
eliminated.** All housekeeping peripherals (INA228 ×9, TMP117 ×2, ATECC608B, e-compass,
SHT45, LTC4311) sit on **always-on 3V3_STM** to avoid back-powering through I²C ESD clamps
(§4).

### 1.2 Rail table (setpoints + derivations)

| Rail | Source (ref) | Setpoint | Derivation | Monitor (INA228) | PG net |
|---|---|---|---|---|---|
| VOUT_P (VPP) | Bridge U7/U8 → NCP1095 U9 | ~54 V (50–57 V) | PoE bus, PSE-set; SELV ceiling 60 V | U10 **0x40**, shunt R30 **25 mΩ**¹ | POE_PG (PGO, U9.14)² |
| 5V | U28 MIC28516 | **4.99 V** | V_FB 0.6 V; 0.6·(1+R97 15k / R98 2.05k) | (upstream of gated 5V loads) | 5V_PSU_PG (R95→5V) |
| 5V_DISP | U33 RT9742 (from 5V) | 4.99 V (switched) | load switch, EN=DISP_EN | U32 **0x42**, shunt R107 **100 mΩ** | V_DISP_EN_FAULT (nFLG) |
| 3V3 | U29 AP3441 | **3.33 V** | 0.6·(1+R99 10k / R100 2.2k) | U30 **0x43**, shunt R102 **50 mΩ** | 3V3_PSU_PG (R92)³ |
| 3V3_STM | = 3V3 (Kelvin split R106 **100 mΩ**) | 3.33 V | same buck; R106 splits STM branch for U31 telemetry | U31 **0x41**, shunt R106 **100 mΩ** | (shares 3V3) |
| 3V3_LAN | 3V3 via FB1 600 Ω | 3.33 V | ferrite-isolated PHY analog rail | (via 3V3) | — |
| 3V3_CLK | 3V3 via FB10 600 Ω | 3.33 V | ferrite-isolated mux rail (VDD-domain, ≤VDD for PH0) | — | — |
| VDDA | U13 LT3045 (from 3V3) | 3.3 V | LT3045 SET; MCU analog island for OCXO loop | — | — |
| 3V3_GPS | U22 LT3045 (from 5V) | **3.32 V** | 100 µA × R71 33.2k = 3.32 V | U23 **0x4A**⁴, shunt R72 **75 mΩ** | 3V3_GPS_LDO_PG (R215→PG0) |
| OCXO rail | U38 AP3441 → U39 TPS7A5201 | **3.327 V** | LDO: 0.8 V·(1+R129 12.1k / R130 3.83k); buck pre-reg 3.70 V (R127 12.4k/R128 2.4k, Vref 0.6) | U37 **0x46**, shunt R126 **25 mΩ** | OCXO_LDO_PG, OCXO_PSU_PG (R232→PG5) |
| 3V0_RF | U51 LT3045 (from 5V via FB9) | **3.01 V** | 100 µA × R186 30.1k = 3.01 V; I_LIM R185 1.5k → ~100 mA | — | 3V0_RF_LDO_PG (R266→PG2)⁵ |
| V_ANT | U27 RT9742 (from 5V) | ~5 V (switched, foldback ~182 mA) | load switch, EN=ANT_BIAS_EN; foldback set by R77 (§3.8) | U26 **0x45**, shunt R89 **150 mΩ** | V_ANT_EN_FAULT (nFLG) |
| VCC_RB | U40 MIC28516 | **24.45−6.645·VCTRL** (5–24 V) | digipot-injected FB; pedestal 24.45 V (§3.9) | U44 **0x47**, shunt R159 **7 mΩ** (2512) | RB_PSU_PG (R143→3V3) |
| VCC_RB_G | Q25 SI7469DP P-FET gate (from VCC_RB) | = VCC_RB (gated) | disconnect gate, RB_VCC_GATE=PB1 | (via U44) | — |
| STM_VBAT / GPS_VBAT | U34 TPS61094 | **3.0 V** (VBAT) | OSEL R116/R117 3.09k → Table 7-1 | — | BKP_STM_PG (PF14), BKP_GPS_PG (PF15) |
| supercap term | U34/U35 charge control | **2.7 V** / 25 mA | VCHG R112/R113 **13.0k 1%** (Tbl 7-2 → 2.7 V); ICHG R114/R115 9.53k (Tbl 7-3) | — | (U67 PG comparator) |

¹ All PoE loads draw from `V_POE`, so R30 25 mΩ sits in the series feed and U10 reads real load current. Shunt values for all nine INA228 rails are **final as-built** — sizing math in §2.1.1.
² POE_PG reaches PG7 through the R264 162 k / R263 10 k divider → **2.75 V at a 50 V bus / 2.97 V at 54 V / 3.13 V at 57 V** (chain R20 10 k + R264 162 k + R263 10 k = 182 k; the R20 pull-up is in series and must be included), keeping the MCU input off the ~54 V node. All three corners clear VIH (2.31 V) by ≥0.44 V and sit under 3.6 V.
³ AP3441 PG **pulls up to VIN (5 V) when good** (Diodes DS39754); the R94 6.81k / R92 10k divider scales it to PG4 ≈ 2.98 V — no external pull-up needed.
⁴ GPS INA228 is strapped **0x4A** (A0=A1=SDA) because SHT45 (U72) owns 0x44 on the same I²C1 bus. Do NOT re-strap.
⁵ LT3045 U51 PWRGD is open-collector; `3V0_RF_LDO_PG` (PG2) carries **R266 10 k → 3V3_STM** (mirrors R215 on the GPS LDO) to present a valid HIGH in regulation. **R266 is fitted as-built** — PG2 is a normal power-good input, no firmware masking required.

### 1.3 PoE power budget

| Item | Value | Basis |
|---|---|---|
| PoE class | **Class 6** | R13 232 Ω / R14 909 Ω → 51 W PD signature (Type 3 / bt) |
| PD available (Class 6) | 51 W at PD | IEEE 802.3bt, 60 W PSE |
| Board steady draw (OCXO + Rb both on) | ~12–14 W | OCXO always-on + FE-5680A concurrent |
| Cold-start peak | ~25–30 W | Rb warm-up surge concurrent with OCXO |
| Classification requirement | **Type 2 (at) min, Type 3 (bt) for margin** | Class 3 (~13 W) insufficient with concurrent Rb |
| PD bus TVS | CR1 SMCJ58A | Vwm 58 V > 57 V; VBR(min) 64.4 V > 60 V SELV; Vc ~93 V < 100 V |
| PD bulk (CPD) | C10 10 µF / 80 V (VOUT_P→RTN) | hot-swap hold-up; C8 0.1 µF/100 V = VPP decouple |
| Port hold-up window | ~4.8 ms @ 180 µF C_port | PFI reserve, t = C(V₁²−V₂²)/2P (§3.3, PFI) |

---

## 2. Current-monitoring & fault architecture

### 2.1 INA228 map (9 monitors, all I²C1 @ PB8/PB9)

Address = 0x40 + 4·f(A1) + f(A0), pin tie GND=0/VS=1/SDA=2/SCL=3. Straps read back from the
netlist (U*.1 = A1, U*.2 = A0) and confirmed against the address table.

**All nine shunt values are final as-built** — Vishay Dale **WSL** metal-strip, **1 %**, AEC-Q200,
1206 (0.25 W) except R159 (2512, 1 W). Sizing math and the per-rail derivation are in §2.1.1;
firmware register constants in §2.1.2.

| # | Ref | Addr | A1/A0 strap | Rail | Shunt (MPN) | FS current¹ | ALERT net → MCU pin |
|---|---|---|---|---|---|---|---|
| 1 | U10 | 0x40 | GND / GND | PoE input (V_POE) | R30 **25 mΩ** (WSL1206R0250FEA) | ±1.638 A² | INA_ALERT_V_POE → PG8 |
| 2 | U31 | 0x41 | GND / VS | STM 3V3 (Kelvin split) | R106 **100 mΩ** (WSL1206R1000FEA) | ±409.6 mA | INA_ALERT_3V3_STM → PG9 |
| 3 | U32 | 0x42 | GND / SDA | 5V_DISP | R107 **100 mΩ** (WSL1206R1000FEA) | ±409.6 mA | INA_ALERT_5V_DISP → PG10 |
| 4 | U30 | 0x43 | GND / SCL | main 3V3 | R102 **50 mΩ** (WSL1206R0500FEA) | ±819.2 mA | INA_ALERT_3V3 → PG11 |
| 5 | U26 | 0x45 | VS / VS | Antenna bias (V_ANT) | R89 **150 mΩ** (WSL1206R1500FEA) | ±273.1 mA | INA_ALERT_V_ANT → PG13 |
| 6 | U37 | 0x46 | VS / SDA | OCXO 3.327 V | R126 **25 mΩ** (WSL1206R0250FEA) | ±1.638 A | INA_ALERT_OCXO → PG14 |
| 7 | U44 | 0x47 | VS / SCL | Rb (VCC_RB) | R159 **7 mΩ** (WSL25127L000FEA, 2512 1 W) | ±5.851 A | INA_ALERT_VCC_RB → PG15³ |
| 8 | U23 | 0x4A | SDA / SDA | GPS VCC (3V3_GPS) | R72 **75 mΩ** (WSL1206R0750FEA) | ±546.1 mA | INA_ALERT_3V3_GPS → PG12 |
| 9 | U54 | 0x4C | SCL / GND | Panel-LED 5 V | R199 **150 mΩ** (WSL1206R1500FEA) | ±273.1 mA | INA_ALERT_5V_PANEL → PF13 |

¹ **All nine run ADCRANGE=1** (±40.96 mV shunt FS); FS current = 40.96 mV / R_shunt. No rail needs
the ±163.84 mV range — see the headroom column in §2.1.1.
² All PoE loads draw from `V_POE`; R30 is in the series `VOUT_P`→`V_POE` feed → U10 reads real load current.
³ INA_ALERT_VCC_RB (PG15) pull-up is R265 10 k → 3V3_STM, matching the other eight ALERT lines.

A tenth milliohm resistor, **R16 25 mΩ** (Ohmite MCS1632R025DER, 1206, 1 W), is the **NCP1095
hot-swap sense (RSNS)** in the PD return — *not* an INA228 shunt and not firmware-readable (§3.1).

#### 2.1.1 Shunt sizing — rules and per-rail derivation

**Selection rules applied to every rail:**

1. **Fit the worst-case rail current inside ADCRANGE=1 (±40.96 mV) with margin.** Target
   **50–75 % of FS at the design maximum** — enough headroom that a transient does not clip the
   reading, while keeping resolution. Using the ±40.96 mV range rather than ±163.84 mV buys a **4×
   finer shunt LSB (78.125 nV vs 312.5 nV)** for the same shunt.
2. **Keep the insertion drop negligible against the rail.** ≤ ~1 % of the rail voltage at maximum
   load, and — for the LDO-fed rails — small enough that the delivered voltage stays inside the load's
   supply window.
3. **Keep dissipation ≪ the package rating** (1206 WSL = 0.25 W, 2512 WSL = 1 W); ≤ ~25 % of rating
   at the design maximum so self-heating does not add TCR error.
4. **Prefer 1 % metal-strip (WSL) with low TCR** over thin-film; the 1 % tolerance is removed by the
   per-board `SHUNT_CAL` trim (§2.1.2, `bench_tuning` BM-1), the TCR is not.

| Rail (INA) | Shunt | Design-max current (basis) | V_shunt @ max | % of FS | P_shunt @ max | Insertion drop | % of rail |
|---|---|---|---|---|---|---|---|
| V_POE (U10) | 25 mΩ | **1.20 A** — Class-6 51 W at the 42.5 V Type-3 PD floor | 30.0 mV | 73 % | 36 mW | 30 mV | 0.06 % |
| 3V3_STM (U31) | 100 mΩ | **0.30 A** — MCU @250 MHz + NOR + I²C cluster, peak | 30.0 mV | 73 % | 9.0 mW | 30 mV | 0.90 % |
| 5V_DISP (U32) | 100 mΩ | **0.25 A** — TFT logic + backlight at full duty | 25.0 mV | 61 % | 6.2 mW | 25 mV | 0.50 % |
| 3V3 main (U30) | 50 mΩ | **0.45 A** — 3V3_STM + 3V3_LAN + 3V3_CLK + VDDA + relay + supercap charge | 22.5 mV | 55 % | 10.1 mW | 22.5 mV | 0.68 % |
| V_ANT (U26) | 150 mΩ | **0.182 A** — the R77 foldback limit itself (hard ceiling) | 27.3 mV | 67 % | 5.0 mW | 27.3 mV | 0.55 % |
| OCXO (U37) | 25 mΩ | **1.20 A** — OH300 warm-up surge (bench item OS-2) | 30.0 mV | 73 % | 36 mW | 30 mV | 0.90 % |
| VCC_RB (U44) | 7 mΩ | **2.10 A** — FE-5680A warm-up at 15 V (≈30 W) | 14.7 mV | 36 % | 31 mW | 14.7 mV | 0.10 % |
| 3V3_GPS (U23) | 75 mΩ | **0.13 A** — ZED-F9T acquisition peak | 9.75 mV | 24 % | 1.3 mW | 9.75 mV | 0.29 % |
| Panel-LED (U54) | 150 mΩ | **0.132 A** — all 7 LEDs on (6×91 Ω + 1×150 Ω ballast) | 19.8 mV | 48 % | 2.6 mW | 19.8 mV | 0.42 % |

**Why the two rails with the loosest fit are deliberate:**
- **VCC_RB at 7 mΩ / 36 % FS.** VCC_RB is programmable 4.51–24.45 V. At the *low* end of that range
  the same Rb power draw is several times the current, so the shunt is sized for the low-voltage
  worst case (5.85 A FS ≈ 29 W at 5 V), not for the 15 V nominal. 2512/1 W is required: at 5.85 A the
  shunt dissipates 240 mW, which a 1206 (0.25 W) would not carry.
- **3V3_GPS at 75 mΩ / 24 % FS.** Sized by rule 2, not rule 1 — the LT3045 delivers 3.32 V and the
  ZED-F9T floor is 2.7 V, but the shunt is the only series element and its drop is subtracted at the
  module pins. 75 mΩ keeps the drop under 10 mV at peak (≈3.31 V delivered). Resolution is still
  1.04 µA/LSB, ~30 000 counts at the F9T's ~30 mA idle — resolution is not the constraint here.

**Superseded values** (documented so stale numbers are recognizable): R30 150 mΩ → 25 mΩ (150 mΩ
would have put 180 mV across the shunt at 1.2 A — past even the ±163.84 mV range); R106 15 mΩ →
100 mΩ (15 mΩ wasted 85 % of the range); R102 75 mΩ → 50 mΩ; R72 500 mΩ → 75 mΩ (500 mΩ dropped
65 mV of a 3.32 V rail); R89 100 mΩ → 150 mΩ; R159 20 mΩ → 7 mΩ; R199 220 mΩ → 150 mΩ.

#### 2.1.2 Firmware register constants (identical form for all nine)

With **ADCRANGE=1** and `Max_Expected_Current` set to the device's own full-scale, the INA228
calibration collapses to one constant for every monitor on the board:

```
CURRENT_LSB   = FS_current / 2^19 = (40.96 mV / R_SHUNT) / 524288 = 78.125 nV / R_SHUNT
SHUNT_CAL     = 4 · 13107.2e6 · CURRENT_LSB · R_SHUNT
              = 4 · 13107.2e6 · 78.125e-9            (R cancels)
              = 4096   ← same for all nine devices
```
(The ×4 is the ADCRANGE=1 term from the INA228 datasheet; SHUNT_CAL is 15-bit, 4096 ≪ 32767.)

| INA | Addr | R_SHUNT | CURRENT_LSB | POWER_LSB (=3.2·I_LSB) | FS current | FS power at rail |
|---|---|---|---|---|---|---|
| U10 | 0x40 | 25 mΩ | **3.125 µA** | 10.0 µW | 1.6384 A | 88.5 W @54 V |
| U31 | 0x41 | 100 mΩ | **781.25 nA** | 2.5 µW | 0.4096 A | 1.36 W @3.33 V |
| U32 | 0x42 | 100 mΩ | **781.25 nA** | 2.5 µW | 0.4096 A | 2.04 W @4.99 V |
| U30 | 0x43 | 50 mΩ | **1.5625 µA** | 5.0 µW | 0.8192 A | 2.73 W @3.33 V |
| U26 | 0x45 | 150 mΩ | **520.833 nA** | 1.667 µW | 0.2731 A | 1.37 W @5 V |
| U37 | 0x46 | 25 mΩ | **3.125 µA** | 10.0 µW | 1.6384 A | 5.45 W @3.327 V |
| U44 | 0x47 | 7 mΩ | **11.1607 µA** | 35.71 µW | 5.8514 A | 87.8 W @15 V |
| U23 | 0x4A | 75 mΩ | **1.04167 µA** | 3.333 µW | 0.5461 A | 1.81 W @3.32 V |
| U54 | 0x4C | 150 mΩ | **520.833 nA** | 1.667 µW | 0.2731 A | 1.28 W @4.7 V |

Range-independent constants (same silicon on all nine): VBUS LSB **195.3125 µV** (0–85 V, so every
rail including the 54 V PoE bus and the 24 V VCC_RB reads directly, no divider); DIETEMP LSB
**7.8125 m°C**; ENERGY LSB = 16·POWER_LSB; CHARGE LSB = CURRENT_LSB. The `SOVL`/`SUVL` over/under
shunt-voltage alert registers use a **1.25 µV LSB** at ADCRANGE=1 (16-bit signed, FS 40.96 mV) —
threshold codes per rail are in `firmware_hardware_interface §4.2`.

**Accuracy.** Before calibration the **1 % shunt tolerance dominates** the error budget (the INA228's
own gain error is an order of magnitude smaller). The per-board trim is a single multiply on
SHUNT_CAL:
```
SHUNT_CAL_trimmed = round( 4096 · I_reference / I_reported )
```
measured at a steady mid-range load per rail (`bench_tuning` BM-1). Store the nine trimmed values in
NOR with the calibration record; they are per-board, not per-design.

Full I²C1 map (15 devices, no collisions): 0x19 LIS2DH12 (U59), 0x1E IIS2MDC (U61),
0x40–0x47/0x4A/0x4C INA228 (above), 0x44 SHT45 (U72), 0x48 TMP117 (U58), 0x49 TMP117 (U57),
0x60 ATECC608B (U60). Bus pull-ups R202/R203 4.7k→3V3_STM; LTC4311 U56 rise-time
accelerator ENABLE tied on (3V3_STM) — accelerator only, not a segmenting buffer.

### 2.2 ALERT routing

All 9 INA228 ALERT outputs are open-drain, wired to **direct MCU GPIO** (PG8–PG15 + PF13),
each with a 10 k pull-up to 3V3_STM (R220–R227, plus R265 for PG15/VCC_RB). There is no I/O
expander in the path and no open-drain wire-OR; firmware polls/EXTIs each alert line directly.

### 2.3 Rail power-good supervision

| PG net | Source | MCU pin | Notes |
|---|---|---|---|
| 5V_PSU_PG | U28 PG (OD), R95→5V pull-up | PG3⁶ | working (pull-up to 5V) |
| 3V3_PSU_PG | U29 AP3441 PG (**pulls to VIN=5V when good**) | PG4 | R94 6.81k / R92 10k divider → **2.98 V**; no pull-up needed |
| 3V3_GPS_LDO_PG | U22 PG, R215→PG0 | PG0 | LT3045 PG |
| OCXO_LDO_PG / OCXO_PSU_PG | U39 PG / U38 AP3441 PG → R125 3.57k → node N (→R124 10k→GND) | — / PG5 | See ⁷ — node N ≈ **2.98 V** drives **U39 EN**; further R232 1.21k/R233 10k → PG5 ≈ **2.66 V** |
| 3V0_RF_LDO_PG | U51 LT3045 PG (**open-collector**) | PG2 | R266 10 k → 3V3_STM pull-up (mirrors R215 on the GPS LDO) |
| BKP_STM_PG / BKP_GPS_PG | U67 LMV393 comparators | PF14 / PF15 | backup-rail good; threshold ≈2.0 V rising / 1.83 falling (Vref 3.3·150k/260k) |
| POE_PG | NCP1095 PGO → R264 162k / R263 10k divider | PG7 | PG7 = **2.75 / 2.97 / 3.13 V** at 50 / 54 / 57 V (R20 10 k is in series — total chain 182 k); PG7 is FT |

⁶ Confirm STM32 PG3/PG7 are 5V-tolerant (FT) — 5V_PSU_PG pulls PG3 to 5 V; both Port-G, expected FT.
⁷ **OCXO PG divider — loaded, corrected.** U38 PG pulls to VIN = **5 V** when good (AP3441 buck runs off the 5 V rail, not VOUT_P). Node N (`OCXO_PSU_PG`) is loaded by **two parallel legs** — R124 10k→GND *and* the series R232 1.21k+R233 10k→GND — so the earlier per-divider figures (3.68 V / 3.28 V) were computed in isolation and are wrong. Actual: node N = 5·(R124‖(R232+R233))/(R125+…) = **2.98 V** (not 3.68 V); PG5 = node N·R233/(R232+R233) = **2.66 V** (not 3.28 V). Both remain valid: U39 (TPS7A52) EN VIH(max) = 1.1 V, abs-max 7 V → 2.98 V reliably enables the OCXO LDO; PG5 2.66 V > STM32 VIH 2.31 V (margin 0.35 V) and < 3.6 V. **Caveat:** AP3441 DS39754 does **not** spec the PG pull-up impedance, so both absolute levels are contingent on that internal pull-up under the ~0.57 mA divider load — **bench-confirm** node N and PG5 with the 5 V rail up (§5). If PG5 sags near VIH, raise R233 (e.g. 22k) to pull PG5 toward node N. The 3V3 divider (R94 6.81k/R92 10k → PG4 = 2.98 V) has PG4 as its only load and is unaffected.

### 2.4 PFI / POE_KILL / external WDT / holdover

- **PFI (Power-Fail Input, PE8/EXTI8):** LMV393 half U2A senses VOUT_P through R2 1.2M /
  R3 39.2k (k = 0.031633) vs Vref = 3.3·R5 100k/(R4 174k + R5) = **1.204 V** → trip at
  **38.1 V** nominal, hysteresis R6 2M → assert 36.8 V / re-arm 38.8 V (both < 42.5 V Type-3
  floor). Open-drain, R7 1k series + R8 100k pull-up→3V3. Annunciation only, no autonomous
  action. Buys ~4.8 ms of MCU run-time at 180 µF C_port. See `power_fail_input`.
- **POE_KILL (PE15) / WDO_N:** last-resort board cold-cycle. MCU (PE15) or external WDT
  (WDO_N low) drives the NCP1095 **AUX** pin (datasheet-sanctioned pass-switch disable, not
  a PGATE clamp). Bias rail V_KBIAS = VOUT_N + 12 V (R18 24.9k drop + D4 12 V zener). Input
  stages GND-referenced (Q1/Q4/Q5 emitters=GND) so they track floating RTN during a kill.
  Sustain PNP Q3 + HOLD net (C9 1 µF, τ ≈ 57 ms; VAUX assert ~150 ms). Recovery is
  PSE-driven (MPS dropout → Vrst → re-detect). Q3 (BC857W) is high-side: E(2)=V_KBIAS,
  C(3)=HOLD. See `poe_kill`.
- **External WDT (U64 TPS3430WDRC):** windowed watchdog, VSON-10, factory preset (CWD/CRST
  = NC). Window 920–1360 ms valid kick (WDT_KICK=PB2 falling edge), 0.68 s runaway / 1.84 s
  stall → fault. SET0=3V3_STM, SET1=WDT_EN (**PC12**; R213 100k pull-down → boot-disabled).
  WDO open-drain active-low, R23 10k pull-up → WDO_N. WDO_N is WDO-only (no thermal/Rb-OV
  wire-OR; Rb-OV uses a separate RB_OV_DET path). See `external_wdt`.
- **Holdover alarm relay (K2 G6K-2F-Y, WDT sheet):** normally-energized fail-safe. PA6
  (HOLDOVER_ALARM_RELAY) → R260 470 Ω → Q24 → coil (K2.1→3V3, D24 BAT54J flyback). Healthy →
  energized; fault/reset/power-loss → de-energized → NC closes = ALARM. Both Form-C poles →
  J16. (K1, the *other* G6K-2F-Y, is the Rb RS-232 DPDT relay — §3.11.)

---

## 3. Per-subsystem circuit design

### 3.1 PoE front end

**Purpose:** rectify both PoE polarities, present a single 51 W Class-6 PD signature,
hot-swap the port cap, and hand off a clean ~54 V VOUT_P bus to the bucks.

**Topology:** magjack J1 (all 4 pairs, Alt-A + Alt-B via center taps TR0..TR3_TAP) → two
FDMQ8205A GreenBridge active-rectifier bridges (U7, U8) → VOUT_P/VOUT_N → NCP1095 U9 PD
controller with external pass FET Q2 (FDMC8622, 100 V / 40 mΩ) and RSNS R16 25 mΩ.

**Derivations:**
- **Class 6 signature:** RCLASSA = R13 232 Ω, RCLASSB = R14 909 Ω → 51 W (NCP1095 Table 1).
- **Hot-swap current limit:** hard VOC 1.2 V / R16 25 mΩ = 48 A instantaneous clamp; soft
  IOC 6.4 A / 960 µs — datasheet-fixed, R16 sizes the sense.
- **V_KBIAS:** R18 24.9k from VOUT_P drops ~38 V at ~1.8 mA (81 mW at 57 V), D4 12 V zener
  clamps → VOUT_N + 12 V for the kill driver (§2.4).
- **TVS CR1 SMCJ58A:** Vwm 58 V clears 57 V PoE max; VBR(min) 64.4 V > 60 V SELV; Vc ~93 V
  < 100 V FET/cap rating.
- **PGO → buck EN:** NCP1095 PGO (open-drain, VPP-referenced pull-up) gates the buck enables
  for cold-start. The MCU tap (PG7) is divided — R264 162 k / R263 10 k drop POE_PG to **2.97 V** at a
  54 V bus (2.75 V at 50 V, 3.13 V at 57 V), keeping the MCU input off the VPP-referenced node.
  **R20 10 k is the POE_PG pull-up to VOUT_P and is in series with that divider**, so the real chain is
  10 k + 162 k + 10 k = 182 k — computing PG7 from R264/R263 alone overstates it by ~5 %. U28 EN also
  sits on the undivided POE_PG.
- **NCM/NCL/LCF → MCU (resolved).** All four NCP1095 status pins (LCF 13, PGO 14, NCM 15, NCL 16)
  are **open-drain, referenced to RTN** (U9.12 → **GND**, the PD-return / hot-swap drain node = board
  ground), abs-max **+72 V** to RTN. NCM/NCL (Class result MSB/LSB) → **PC2/PC7**, LCF (long-class
  finger) → **PC3**, each wired **direct to the MCU, no divider** — correct, because these are
  GND-referenced (unlike PGO, whose pull-up sits on the 54 V VPP node and *needs* the R264/R263
  divider). **They are safe direct to a 3.3 V GPIO (open-drain can only sink to GND; nothing drives
  54 V onto them).** But being open-drain they **float without a pull-up** — as-built there is none on
  POE_NCM/POE_NCL/POE_LCF. **Action:** provide a pull-up to 3V3_STM — either 3× external **10 kΩ**
  (on-pattern with the nine INA228 ALERT pull-ups) or the STM32 **internal pull-ups** on PC2/PC3/PC7
  (zero-BOM, adequate for these slow latched status lines). Datasheet VOL ≤ 0.5 V @ 2 mA → either
  choice reads valid. Firmware pin contract must set the pull if external parts are not added.

### 3.2 Main bucks (U28 5 V, U29 3V3)

**Purpose:** convert VOUT_P (~54 V) to the two primary rails.

**U28 MIC28516 (5 V):** adaptive-on-time synchronous buck, V_FB = 0.6 V.
```
V_OUT = 0.6 · (1 + R97/R98) = 0.6 · (1 + 15k/2.05k) = 4.99 V ✓
```
- L2 6.8 µH power inductor (≥12 A, SELECT); C71/C72 47 µF output bulk; FREQ set by R90 200k
  (→VOUT_P) / R91 100k divider — **verify method + abs-max + fSW** (unloaded ~18 V on FREQ;
  self-consistent only if FREQ internally clamps).
- EN sense divider off VOUT_P; confirm MIC28516 EN abs-max ≥ 60 V.

**U29 AP3441 (3V3):** synchronous buck, V_FB = 0.6 V.
```
V_OUT = 0.6 · (1 + R99/R100) = 0.6 · (1 + 10k/2.2k) = 3.33 V ✓
```
- L3 2.2 µH; 3V3 = 3V3_STM is a *single* rail (not independent) — the **R106 100 mΩ** Kelvin split
  isolates the STM branch so U31 (0x41) reads STM-only current while U30 (0x43) reads total. The
  split costs 30 mV at the 300 mA STM peak (3.33 V → 3.30 V at the MCU), far above the H5 VDD floor.
- U13 LT3045 hangs off 3V3 → VDDA (MCU analog island for the OCXO DAC/ADC loop).

### 3.3 Battery / supercap backup (U34, U35 TPS61094)

**Purpose:** ride-through for STM_VBAT (RTC/backup domain) and GPS_VBAT (ZED-F9T V_BCKP,
ephemeris hold) across PoE loss; sized to the ZED-F9T ephemeris window (~4 h), not
arbitrarily large.

**Topology:** each TPS61094 is a bidirectional buck/boost with an integrated supercap
charger; supercaps C90/C91 (3 F) on the storage node. In-regulation it bucks 3V3→VBAT and
trickle-charges the cap; on VIN loss it boosts from the cap to hold VBAT.

**Derivations (datasheet strap tables):**
- **VBAT = 3.0 V:** OSEL set by R116/R117 = 3.09k → Table 7-1.
- **Supercap termination 2.7 V:** VCHG R112/R113 = **13.0k 1%** → Table 7-2 (2.7 V). The enclosure
  Tmax is specified **125 °F = 51.7 °C**, below the DSF305Q3R0's 65 °C corner, so its full **3.0 V**
  rating applies; 2.7 V sits 10 % under that (±2 % TPS61094 term accuracy → ~2.75 V worst-case, still
  <3.0 V). The TPS61094 VCHG table has no option between 2.7 V (13.0k) and 3.6 V (17.4k), so **2.7 V is
  the highest usable setting under the 3.0 V cap**. Energy stored scales (2.7/2.2)² ≈ **1.5×** vs the
  former 2.2 V, extending the ephemeris-hold window. VBAT (OSEL, 3.0 V) and boost operation are
  unchanged, so the U67 backup-PG thresholds need no retune. (Supersedes the earlier 2.2 V/85 °C
  assumption; resolves the schematic-4.75k vs BOM-6.65k conflict — both were stale.)
- **Charge current 25 mA:** ICHG R114/R115 = 9.53k → Table 7-3 → ~5 min to top 3 F.
- **Backup-PG comparators (U67 LMV393):** threshold Vref 3.3·150k/(150k+110k) → 1.904 V
  → ≈2.0 V rising / 1.83 V falling; outputs BKP_STM_PG (PF14) / BKP_GPS_PG (PF15).

### 3.4 OCXO discipline (Y3 OH300 + steering loop)

**Purpose:** always-on 10 MHz reference, STM32-steered via a DAC → loop filter → Vc, with
the control node biased to a defined 1.65 V center that never floats.

**Topology / signal flow:**
```
PA4 (DAC1_OUT1, OCXO_VC) ─R120 100k─┬─ U36 OPA320 (+IN) ── unity buffer ── R122 1k ─ OCXO_V ─┬─ Y3.1 Vcontrol
                                     │                                                        ├─ C101 0.1µF → GND
              C98 0.1µF → GND ───────┤                                                        └─ PA3 (ADC Vc sense)
              R121 10M → 1.65V ──────┘
 1.65V ref = R118/R119 (100k/100k off VDDA), C99 bypass
```

**Derivations:**
- **Vc center 1.65 V:** R118/R119 = 100k/100k divide VDDA (3.3 V) → 1.65 V. R121 10M pulls
  the OPA320 +IN to 1.65 V so Vc defaults to mid-range if PA4 is Hi-Z — **confirmed never
  floats.**
- **Loop-filter corners:** R120·C98 = 100k·0.1µF → **15.9 Hz**; R122·C101 = 1k·0.1µF →
  **1.6 kHz**. Firmware owns the slow disciplining loop; these set the analog roll-off.
- **INA228 U37 (0x46):** shunt R126 25 mΩ, IN+ upstream (D10 BAT54J side) — correct
  polarity for the OCXO rail.
- **OCXO rail 3.327 V:** buck U38 pre-regs to 3.70 V (R127 12.4k/R128 2.4k, Vref 0.6),
  U39 TPS7A5201 LDO → 0.8·(1 + R129 12.1k / R130 3.83k) = 3.327 V. R129/R130 are **0.1%**
  precision (TE RQ73C, SPECIAL-KEEP).

**DIELECTRIC RULE (critical):** C98, C101 (and C99 on the ref) sit on the Vc steering path
and are specified **C0G/NP0 or film** — X7R is piezoelectric; microphonics FM-modulate the
10 MHz carrier. **Resolved as-built:** all three are **KEMET C1210C104J5GACAUTO, 0.1 µF 50 V C0G,
1210** (0.1 µF C0G does not exist below case 1210, so the footprint is 1210, not 0402). No class-II
(X7R) substitution. C109/C117 22 µF are bulk (not on the Vc path) → X7R OK.

**Bench-tuning:** OH300 EFC pull range vs the 1.65 V center (op-amp span 0–3.3 V VDDA); DAC
→ Hz slope is a per-board calibration against GPS/Rb. See `findings_timing` §8.

### 3.5 Clock mux + 1PPS fanout (U52/U53/U71)

**Purpose:** select OCXO (A) or external/Rb ref (B) into PH0 (HSE bypass); provide isolated
bench 10 MHz + 1PPS fanouts. See `sts1000_clock_mux` (as-built).

**Topology:**
```
OCXO_CLK_OUT (R123 22Ω src-term @ Y3) ─► U52 I0(3) ┐
10MHz_CLK_OUT (R184 33Ω src-term @ U50) ─► U52 I1(1)┘ 74LVC1G157 Y(4) ─R187 22Ω─ CLK_OUT → PH0
MUX_SEL (PB6, R188 100k↓) ─► U52 S(6)              └─► U53 74LVC1G34 ─R189 22Ω─ C160 ─ 10MHz_RF_OUT (J9)
ETH_PPS_OUT (PB5) ─► U71 74LVC1G34 (5V) ─R255/R256 33Ω─ 1PPS_OUT (J15)
Rail: 3V3 ─FB10─ 3V3_CLK (VDD domain, ≤VDD so PH0 never over-driven)
```

**Derivations / rules:**
- **Source termination at the driver:** R123 22 Ω at the OCXO output, R184 33 Ω at the
  LTC6752 Q. **No receiver-end series R** at the mux inputs (would be asymmetric into the
  2.5 pF Schmitt input). R187 22 Ω is the single PH0-path damper (U52 Zo ~25–35 Ω + 22 Ω).
- **Boot default OCXO:** R188 100k pull-down on MUX_SEL → S=0 → I0 (OCXO) before PB6 is
  driven. Matches the '157 function table (non-inverting, no glitch-free IC — a plain
  selector cannot hang when the *outgoing* clock has stopped; §4).
- **3V3_CLK = MCU VDD domain** (FB10-isolated), deliberately **not** the 3.327 V OCXO LDO,
  so mux Voh ≤ VDD keeps PH0 within abs-max (VDD+0.3).
- **Fanout tap is before R187** (at the Y node) via U53 so the SMA 50 Ω load never disturbs
  PH0. R189 22 Ω (trim 22–33) source-matches U53 Zo ~30 Ω to 50 Ω; C160 100 nF DC-block
  (HPF ~32 kHz).
- U71 buffers 1PPS at 5 V into the SMA only (MCU side stays 3.3 V).

### 3.6 External-reference RF slicer (U50 LTC6752 + switched term)

**Purpose:** condition any 10 MHz external reference (+6…+15 dBm sine or TTL/CMOS) into a
clean 3.0 V CMOS square (mux input B), power-state-independent and abuse-proof. See
`ntp_server_rf_frontend`.

**Topology:**
```
SMA ─D8 ESD─||─C149─┬─N1(R174/R175 47k→1.5V)─R178 100Ω─N2(D19 clamp)─||─C151 10nF─R181 1k─┬─ U50 +IN
                    │                                                                      │  LTC6752xS5
             switched term: N1─R177 49.9Ω─C150─U49 TMUX1101(COM)⇄GND, SEL=REF_TERM_EN     └─ Q ─R184 33Ω─ 10MHz_CLK_OUT
 −IN = REF_VMID (R179/R180 10k/10k → 1.5V);  VCC: 3V0_RF ─R183 33Ω─┬─ D20 3V3 clamp
```
(Old-doc designators R134/135→R174/175, R136→R177, R137→R178, R138→R181, R143/144→R183/184,
D12→D18, D13→D20 per rf_frontend TODO; as-built netlist refs used above.)

**Derivations:**
- **N1 bias 1.5 V:** R174/R175 47k/47k halve the 3.0 V island; 23.5 kΩ Thevenin ≫ 50 Ω → no
  detune, no bypass on N1 (would short the signal).
- **REF_VMID 1.5 V:** R179/R180 10k/10k; feeds both +IN self-bias and −IN.
- **50 Ω termination:** R177 49.9 Ω 0.1% + C150 DC-block + U49 Ron → ~50.5 Ω, RL > 30 dB;
  switched by REF_TERM_EN (PC10, R176 100k pull-up → default terminated = safe).
- **VCC guard:** R183 33 Ω + D20 3V3 clamp hold LTC6752 VCC ≤ ~3.4 V < 3.6 V abs-max even if
  the island lifts; normal Iq 4.5 mA drops ~0.15 V → VCC ≈ 2.85 V (> 2.45 V min).
- **3V0_RF island (U51 LT3045):** 100 µA × R186 30.1k = 3.01 V (matches OCXO Voh);
  I_LIM R185 1.5k → 100 mA; FB9 600 Ω input filter; EN ← 5V_PSU_PG.
- Clamp current at +20 dBm, term lifted: R178 100 Ω → D19 → island, I ≈ (6.32−3.6)/100 ≈
  27 mA, sunk by D20 (~0.1 W). Comparator +IN limited by R181 1k to ~1 mA.

### 3.7 GNSS receiver (U21 ZED-F9T-00B)

**Purpose:** multi-band timing GNSS on USART3 with TIMEPULSE capture and full supervisor
control. See `gnss_antenna_bias_supervisor` for the antenna front end (§3.8).

**Key connections / derivations:**
- **3V3_GPS LDO (U22 LT3045):** V_OUT = 100 µA × R71 33.2k = **3.32 V**; I_LIM R70 374 Ω;
  series **R72 75 mΩ** is the U23 INA228 (0x4A) shunt — drops **9.75 mV at the 130 mA acquisition
  peak** → **≈3.31 V delivered** at U21.33/34, well above the F9T 2.7 V floor. (Was 500 mΩ, which cost
  65 mV; resized per §2.1.1 rule 2.) EN=GPS_PWR_EN (PC8, R69 100k pull-down → default off).
- **V_BCKP:** GPS_VBAT sourced from U34 TPS61094 (supercap-backed) → warm-start preserved
  across VCC cycles; PG → BKP_GPS_PG (PF15).
- **TIMEPULSE → PA0 (TIM2_CH1); TIMEPULSE2 → PC6 (TIM3_CH1).** RXD/TXD → PD9/PD8 (USART3);
  RXD2/TXD2 → PD1/PD0 (UART4). All single-rail 3.3 V, no level shift.
- pin19 GEOFENCE_STAT → GPS_TXRDY (PD5): wiring correct but firmware must remap to TX_READY
  via UBX CFG.

### 3.8 Antenna bias-T + current limiter + supervisor

**Purpose:** deliver switched 5 V bias to an active antenna over the coax, hard-foldback
current-limit it, and report open/normal/short to both the F9T and the MCU.

**Topology:** 5 V → U27 RT9742 → **R89 150 mΩ** (U26 INA228 shunt) → FB6 → V_ANT → FB4 120 Ω → **R77
3.3 Ω / 0.25 W** (foldback sense / U24 INA181 shunt) → Q11 NSS40300 PNP pass → node A (`Net-(Q11-C)`)
→ L1 47 nH → `GPS_RF_IN` (J7 SMA + U21.2 RF_IN + L10 ESD). **No series DC-block** — see below.

**Derivations:**
- **Foldback limit ~182 mA:** Q12 (BC857W) V_BE across R77: I = 0.6 V / 3.3 Ω ≈ 182 mA
  (~20% over the 150 mA antenna max). Limit set *solely* by R77.
- **INA181 DETECT transfer:** gain 20 × R77 3.3 Ω = **66 V/A**. DETECT ref R85/R86
  100k/11k off 3V3_GPS → 0.329 V → threshold 0.329/66 ≈ **5 mA** (present vs open). The INA181A1
  output saturates near its 3.32 V rail → **≈50 mA** ceiling; it is coarse presence detection only.
  Precise current comes from U26 INA228 (0x45) across **R89 150 mΩ** (520.833 nA/LSB, ±273 mA FS).
- **SHORT leg:** node A ÷2 (R80/R81 100k/100k) vs SHORT ref R82/R83 100k/30k → 0.762 V →
  assert when **node A < 1.52 V** (normal 4.3–5 V, short ~0 V).
- **Bias-T choke:** L1 = 47 nH is **intentional** (Z ≈ 465 Ω at 1.575 GHz; u-blox's own reference
  is 120 nH). It passes the DC bias (300 mA-rated > 182 mA foldback); the board relies on active
  foldback rather than a series R. Accepted lower-isolation RF deviation, **not** an error.
- **Series drop to the antenna:** R89 150 mΩ + R77 3.3 Ω + FB4 DCR + Q11 V_CE(sat). At the 30 mA
  nominal antenna draw the resistive drop is ~104 mV; at the 182 mA foldback ceiling, ~628 mV. An
  active antenna spec'd 3–5 V still sees ≥4.2 V at the foldback point. Raising R89 from 100 mΩ to
  150 mΩ added only 9 mV at full foldback.
- Comparators U25 LMV393 on **5 V** (CMR 0–3.5 V spans all inputs); OD outputs pull to
  3V3_GPS (R84/R87) — fail-safe level shift.
- ANT_OFF disable stage (Q14 NPN → Q13 PNP): Q13 (BC857W) is high-side — E(2)=V_ANT,
  C(3)=Q11-B — so its collector sources into the Q11 base to disable bias on ANT_OFF.
- **No series DC-block on RF_IN (settled).** The ~5 V bias is injected by L1 directly onto
  `GPS_RF_IN` = {J7.1, U21.2, L10.1}; the ZED-F9T's **internal DC block** handles it, which is what
  the u-blox active-antenna reference circuits do (UBX-21040375), including with an external bias
  supply above VCC. A 47 pF part (`C204`) was briefly fitted and has been **removed** — see
  `gnss_antenna_bias_supervisor §3.1` for why the "u-blox places a 47 pF block" claim was retracted.

### 3.9 Rb supply (U40 MIC28516, digipot-trimmed VCC_RB)

**Purpose:** digitally-set 5–24 V buck for the FE-5680A, hard-bounded so no wiper fault can
exit the safe envelope, with an autonomous 26 V OV latch and a disconnect gate. See
`sts1000_vcc_rb_supply`.

**Reference & FB network:** VREF = **MCP1502-30E (3.0 V, VREF_3V0)**; FB network R147 20.0k **0.5 %**,
R148 **604 Ω**, R134 **3.01k**, R136 **4.99k**. Power inductor **L7 39 µH** (Würth 7447709390, 4.1 A,
56 mΩ). Series sense **R159 7 mΩ** (WSL2512, 1 W — §2.1.1). Digipot U43 MCP41U83 VDD1 = **3V3**.

**Transfer function:**
```
VOUT = 0.6·(1 + R147/R148 + R147/R134) − (R147/R134)·VCTRL
Pedestal = 0.6·(1 + 33.11 + 6.645) = 24.45 V
VOUT = 24.45 − 6.645·VCTRL          (VCTRL = 0…3.0 V)
→ VOUT_MAX = 24.45 V (VCTRL=0),  VOUT_MIN = 4.51 V (VCTRL=3.0),  15 V at VCTRL ≈ 1.42 V
```
**Bounding (PASS):** the 24.45 V pedestal is fixed by R147/R148/R134; the digipot can only
pull the FB summing node further to lower VOUT. Every wiper state (POR, mid-scale, SPI-fault,
open) yields ≤ 24.45 V. The only path above is an FB fault → caught by the OV latch.

**OV latch (U41A LMV393, autonomous/latching, PASS):** sense 0.0769·VCC_RB (R138 120k/R139
10k) vs 2.001 V ref (R136 4.99k/R137 10k off VREF_3V0) → trip at **VCC_RB > 26.0 V**. Fires
→ pulls RB_PSU_PWR_EN low → U40 off; held by regenerative Q16/Q17 pair + R151, powered from
the always-on 5 V so it latches (not hiccups). RB_OV_DET = 5·33/51 = 3.24 V (PE3-safe);
RB_OV_RESET (PD3) default LOW, pulse HIGH clears. Threshold ladder: 24.45 V max < 26.0 V
trip < 28 V TVS standoff (D7 1.5SMCJ28A) < ~45 V clamp.

**Disconnect gate (Q25 SI7469DP P-FET):** gate = 0.0909·VCC_RB → Vgs = −0.909·VCC_RB
(−21.8 V at 24 V > ±20 V abs-max → zener D25 BZX84C12 required to clamp −12 V). Default-OFF
via R257 100k gate→source + R262 pull-down. D25 (BZX84C12) is oriented K(1)=VCC_RB, A(2)=gate:
it reverse-blocks in the OFF/normal region and Zener-clamps |Vgs| to ~12 V, so Q25 enhances and
powers VCC_RB_G.

**Output-cap voltage (resolved as-built):** the buck output node `Net-(U44-IN+)` (upstream of R159)
carries C125 **47 nF 50 V X7R 0402** (GCM155R71H473KE02J, feedforward) + C126/C127/C128 **47 µF 50 V
X7R** (Murata KCM55WR71H476MH13L, stacked SMD 2 J-lead). All ≥ 50 V against the 24.45 V pedestal /
26 V OV trip — **2× voltage derating**. Input caps C133–C136 remain 100 V. Note the bulk caps are
class-II: at 24 V bias, expect roughly half the nameplate capacitance — the loop compensation and
ripple numbers must be checked against the *derated* value on the bench (RB-2).

**SAFETY-CRITICAL bench item:** confirm MCP41U83 wiper code-0 parks at terminal B
(VREF_3V0) = safe-low; if it parks at A (GND), VCTRL=0 → 24.45 V pedestal. Never assert
RB_PWR_EN before a safe code is written; verify VCC_RB on INA228 0x47 first.

### 3.10 Rb RS-232 interface (U46, K1, U48)

**Purpose:** FE-5680A serial telemetry + lock status over the DE-9, RS-232/CMOS switchable,
opto-isolated lock. See `rb_rs232_interface`.

- **U46 SN65C3221E** charge-pump RS-232 transceiver, VCC = 3V3; ~EN=GND, FORCEON=3V3,
  ~FORCEOFF=RB_PWR_EN (R164 pull-down). Charge-pump caps C139/C140 (C1±/C2±), C143 1 µF VCC,
  C144 (V−), C145 (V+ → **GND**, doubler configuration).
- **K1 G6K-2F-Y DPDT relay:** de-energized = NC = RS-232 default (fail-safe); switches
  RS-232 ⇄ CMOS direct path. Q21 driver defaults de-energized.
- **Opto lock (U48 APC-817):** RB_LOCK_IN → R171 4.7k → D16 3V3 zener → R172 470 → LED;
  collector = RB_LOCK, R173 22k pull-up→3V3, C148 debounce. Inverting → firmware polarity
  bit. **RB_RX = PB4 = NJTRST → board must be SWD-only.**
- ESD: D13/D14 SMAJ15CA, U47 PESD15VL2BT, L8/L9 + R165/R166 + 47 pF C0G on the RS-232 lines.

### 3.11 I²C peripheral cluster + level shift

**Purpose:** all housekeeping sensors on one always-on I²C1 bus; a level-shifted branch for
the 5 V display touch controller.

- **Bus:** PB8 (SCL) / PB9 (SDA), pull-ups R202/R203 4.7k→3V3_STM; LTC4311 U56 accelerator
  ENABLE tied on. 15 devices, no collisions (§2.1). **All on 3V3_STM** (back-power avoidance,
  §4).
- **Display level shift (U63 PCA9306):** VREF1=3V3_STM, VREF2=5V_DISP, EN=DISP_EN.
  The 5V_DISP-side I²C pull-ups are supplied by the display module (per its datasheet); J17.16
  provides 5V_DISP to the module.

### 3.12 Display subsystem (SPI4 TFT + touch)

- SPI4 shared bus: NOR U62 (nCS=PE11), display (DISP_CS=PE9), digipot U43 (SPI_DPOT_CS=PD2)
  — distinct chip selects. Display MOSI/SCK re-buffered through R210/R211 22 Ω series.
- DISP_RST=PA10 with R209 10k pull-down (held in reset by default); DISP_DC=PB0, DISP_BL=PE6.
- 5V_DISP gated by U33 RT9742 (EN=DISP_EN, R104 10k pull-down default-off; nFLG pull-up
  R105→3V3_STM).
- **NOR U62 (MX25L25645, Macronix — sourcing-compliant):** nRESET=PE10 with R205 10k
  pull-down → **held in reset by default; firmware must drive PE10 high early.**

### 3.13 Panel-LED rail (high-side PWM)

**Purpose:** drive 6 white + 1 red panel indicator, current-monitored, high-side PWM,
default-off. See `findings_hmi` §1.

**Topology:** 5V → U55 RT9742 → **R199 150 mΩ** (U54 INA228 0x4C shunt) → FB12 600 Ω →
V_PANEL_LED → Q22 NSS40300 PNP (E→C=PANEL_LEDS_P common anode) → J17 → panel; cathodes →
ballast → GND. (Panel connectors J4/J11/J12/J13/J14 are consolidated into the single 36-pin J17.)

**Derivations:**
- **Ballast:** rail ≈ 4.7 V. White R247–R252 = **91 Ω** → (4.7 − ~2.9 V Vf)/91 ≈ 17–18 mA
  each; red R253 = **150 Ω** → ~18 mA. All-on ≈ 126–132 mA.
- **Shunt R199 = 150 mΩ:** all-on 132 mA × 0.15 = **19.8 mV = 48.3 % of ±40.96 mV FS** — comfortably
  inside the 50–75 % target band with room for a brighter ballast respin, at 520.833 nA/LSB
  (≈0.4 % of one LED's current per LSB, so a single dead LED is unambiguous). Shunt drop 19.8 mV,
  dissipation 2.6 mW. (Was 220 mΩ / 29.0 mV / 70.9 % FS.)
- **High-side PWM:** PANEL_LED_PWM (PE0) → R216 4.7k → Q23 (R235 10k pull-down) → R217 820 Ω
  → Q22 base (R234 4.7k off-hold). Non-inverting active-high; PE0 Hi-Z at reset →
  double-default OFF. EN=PANEL_LED_EN (PC0, R198 10k pull-down → RT9742 default-off).
  nFLG → PANEL_LED_FAULT (PF12, R201 10k pull-up).

### 3.14 RGB status indicator (D5)

Common-anode 5 V, low-side NPN per color, active-high default-off (10k base pull-downs +
Hi-Z reset). Ballast sets per-color current:

| Color | Net (pin) | Ballast | Current |
|---|---|---|---|
| Blue | LED_B (PD14), Q8 | R59 60.4 Ω | 24.3 mA |
| Red | LED_R (PD12), Q9 | R60 174 Ω | 16.7 mA |
| Green | LED_G (PD13), Q10 | R61 221 Ω | 8.4 mA |

### 3.15 Optical encoder (U68)

ENC_A_IN/ENC_B_IN (J17) → U68 74LVC2G17 Schmitt buffer (VCC 3V3_STM, 5V-tolerant inputs) →
ENC_A/ENC_B → PA8 (TIM1_CH1) / PA9 (TIM1_CH2). Encoder is a **push-pull (driven) output** at
5 V; U68 outputs 3.3 V so PA8/PA9 need not be FT. ENC_BUTTON → PF11 + R236 10k pull-up + C187.
**No pull-ups needed on ENC_A_IN/ENC_B_IN** — a driven push-pull output requires none. J17.28
supplies the 5 V encoder/panel-logic rail.

### 3.16 USB console (J5 USB-C)

- CC1 (A5) → R66 5.1k → GND, CC2 (B5) → R64 5.1k → GND (separate → UFP advertisement).
- **VBUS sense:** USB_VBUS → R68 100k → USB_VBUS_SENSE → R67 121k → GND → k = 0.547 →
  V(PE2) = 2.87 V at 5.25 V (≤ 3.3 V ✓). (Verify R67's grounded end lands on USB_VBUS_SENSE,
  not USB_VBUS.)
- DP=PA12 / DM=PA11 (USB-FS), no series R; shield R65 1M + C42 4.7 nF **2 kV** Y-cap. **Resolved
  as-built:** C1/C42/C186 are **KEMET C1812C472KGRACAUTO, 4.7 nF 2 kV X7R, 1812** (a 2 kV MLCC has no
  package below 1808/1812). ESD U18 (DP/DM/VBUS) + U65 (CC).

### 3.17 Ethernet PHY (U11 LAN8742AI, RMII)

**Purpose:** 10/100 RMII PHY with IEEE-1588 hardware timestamping in the MAC. See
`findings_ethernet`.

**Derivations / straps:**
- **RBIAS R37 = 12.1k 1% → GND** — exact LAN8742A datasheet value (sets internal bias).
- **REF_CLK direction:** 25 MHz xtal Y1 → PHY PLL ×2 → 50 MHz REFCLKO (pin14) → PA1.
  nINTSEL = 0 (R41 4.75k pull-down) forces pin14 = REFCLKO out (not nINT). Matches STM32
  RMII master-clock-from-PHY topology.
- **Straps:** MODE[2:0] = 111 (R49/R228/R229 **4.99 k** pull-ups) = all-capable, auto-neg on;
  PHYAD0 = 0 (R230 **4.99 k** pull-down) → PHY address 0; REGOFF = 0 (R40 4.75k pull-down) →
  internal 1.2 V regulator on (C17 470 pF + C18 1 µF on VDDCR). The strap resistors were moved
  from the non-standard 5 kΩ to the **E96 4.99 kΩ** (ERJ-2RKF4991X) — same LAN8742A strap window,
  orderable 1 % part, and it merges with the existing 4.99 k line in the BOM.
- **MDI termination:** R42–R45 49.9 Ω 1% each TRD line → 3V3_LAN, with PHY-side center taps
  at 3V3_LAN → 100 Ω differential.
- **nRST:** LAN_RST_N (PD10) with R35 10k pull-down → PHY held in reset until firmware
  drives PD10 high (atypical but deliberate; rails stable before strap latch).
- **Supply:** 3V3 → FB1 600 Ω → 3V3_LAN (ferrite-isolated analog rail).
- Xtal load caps C14/C16 = 27 pF for a 20 pF-CL crystal are a few pF light (~ppm fast) —
  harmless since the RMII domain is intentionally decoupled from the disciplined timing ref.

---

## 4. Design rules honored

| Rule | Where applied | Rationale |
|---|---|---|
| **OCXO Vc caps C0G/film only, never X7R** | C98/C99/C101 (§3.4) | X7R piezoelectric microphonics FM-modulate the 10 MHz carrier. (0.1 µF C0G needs a 1210/film package, not 0402 — §3.4.) |
| **Magnetometer GND: continuous current-free pour** | IIS2MDC U61 (layout) | Copper is diamagnetic/magnetically silent; a void routes return current into a loop under the sensor. Balanced go+return → 1/r² falloff vs 1/r. |
| **Source termination at the driver** | R123 @ OCXO, R184 @ LTC6752, R187 PH0 damper (§3.5) | Series term belongs at the driver; receiver-end series R into a high-Z Schmitt input does nothing and is asymmetric. Redundant receiver-end R removed. |
| **Back-power avoidance: all periph on always-on 3V3_STM** | §3.11, whole I²C1 cluster | Gating a rail while its pull-ups sit on an always-on rail back-powers devices through SDA/SCL ESD clamp diodes. The gated 3V3_P rail was eliminated. LTC4311 does not isolate — it is not a fix. |
| **Glitchless mux = firmware HSI bridge + CSS, not a glitch-free IC** | U52 74LVC1G157 (§3.5) | Dedicated glitch-free mux ICs complete handoff on the *outgoing* clock's edges and hang if that source stopped (exact Rb-death case). Plain selector + HSI bridge + CSS cannot hang. |
| **3V3_CLK on the VDD domain, not the OCXO LDO** | FB10 → 3V3_CLK (§3.5) | Mux Voh must stay ≤ VDD so PH0 (abs-max VDD+0.3) is never over-driven. |
| **Rb FB node hard-bounded by fixed resistors** | R147/R148/R134 (§3.9) | No digipot wiper code (POR/mid/SPI-fault/open) can exceed the 24.45 V pedestal; 26 V OV latch is the firmware-independent backstop. |
| **Comparators on 5 V, outputs pulled to 3.3 V** | PFI U2A, OV U41A, antenna U25, backup U67 | LMV393 CMR = 0…(V+−1.5 V); 5 V gives 0–3.5 V headroom; open-drain output decouples supply rail from logic level. |
| **GPS SMA dual-duty ESD** | J7 (RF 1.5 GHz + 5 V bias) | Needs Cj ≤ 0.2–0.3 pF *and* V_RWM ≥ 6.5 V — L10 PGB1010603MR polymer suppressor (Cj≈0.05 pF, standoff ~20–24 V) satisfies both; outdoor runs still need an inline coax GDT (deferred to install). |
| **No DNP / no fit-jumper** | whole board | All hardware populated; the MCU adapts paths at runtime via GPIO/analog switches. |

---

## 5. Open items & bench verification

**Open hardware items (source/select before fab).** The full schematic-to-layout package is in
`sts1000_schematic_design_review.md` and `sts1000_layout_readiness_review.md`.

| Item | Action | §ref |
|---|---|---|
| **`fp-lib-table` missing** | The KiCad project has `sym-lib-table` but **no `fp-lib-table`**; custom footprints in `hardware/libraries/` are unreachable from the board editor. Create it before opening the PCB. | layout |
| **Footprint fields are not land-pattern links** | Only **25** of 648 symbols carry a resolvable `library:footprint`; **587** hold vendor package text (`0402 (1005 Metric)`, `SOT-323`, …) that KiCad cannot resolve; **36** are empty (J1, J2, J6–J10, J15–J17, K1, K2, L2–L7, U53, U71, Y1, Y2, H1–H14). 31 valid links were lost during the BOM passes and are recoverable from `78853e8`. | layout |
| C125 dielectric | As-built 47 nF **X7R** 50 V (feedforward on a 24 V node). Meets the ≥50 V rule; a C0G part would hold its value under DC bias. Optional improvement, not a blocker. | §3.9 |
| R264 MPN | Confirm the 162 kΩ part (`ERJ-8ENF1623V`, 1206) on the PG7 divider before ordering — a 1.21 kΩ value would drive PG7 board-lethal. | §3.1 |

**Resolved since the last revision** (kept so stale references are recognizable):
**shunt values** — all nine INA228 shunts finalized and sized (§2.1.1); **C98/C99/C101** — 0.1 µF C0G
1210 (C1210C104J5GACAUTO); **C1/C42/C186** — 4.7 nF 2 kV 1812 (C1812C472KGRACAUTO); **C37** — 1 nF →
0.1 µF C0G 1210; **C125–C128** — all 50 V; **R112/R113** — 13.0 k (VCHG 2.7 V); **R266** — 3V0_RF PG
pull-up fitted, PG2 no longer needs masking; **L7** — 39 µH orderable part (7447709390); **PHY straps**
— 4.99 k E96.

**Bench-tuning unknowns** (finalize on a board): MIC28516 U28 FREQ programming + fSW; FE-5680A Vmax
vs the 24.45 V pedestal (per-unit); ZED-F9T RF_IN 5 V-bias tolerance; V_BCKP current at Tmax; OCXO
DAC→Hz slope + EFC pull range; OH300 warm-up current vs the 1.638 A U37 full-scale; per-board
INA228 `SHUNT_CAL` trim (BM-1). Full lists in `sts1000_bench_tuning_procedures.md`,
`peripheral_map §14`, `software_spec §15`.

**MCP41U83 wiper safety (§3.9):** code 0 parks at Terminal B (VREF_3V0 = safe-low), POR is
mid-scale, Mode (0,0). Never assert RB_PWR_EN before a safe code is written; verify VCC_RB on
INA228 0x47 first.

---

## 6. Decision Log

Rationale for the non-obvious choices, keyed to the sections that implement them.

- **MCU on LQFP144 with a direct-GPIO input scan (§2.1–§2.4).** All aggregated fault/UI inputs are
  direct GPIO on Ports F/G rather than through I/O expanders: the MCP23017 GPA7/GPB7 output-only
  erratum and its lack of an open-source INT (a fragile wire-OR) drove the change; the 144-pin
  package supplies the I/O.
- **GPS INA228 at 0x4A (§2.1).** SHT45 (U72) occupies 0x44 on the same I²C1 bus, so the GPS monitor
  is strapped 0x4A to avoid the collision.
- **PG7 telemetry divider R264/R263 (§3.1).** NCP1095 PGO releases to the ~54 V VPP rail when
  power-good; the divider brings that level into the MCU input range (2.75–3.13 V over the PoE range) so a single GPIO reads
  PoE power-good without rail exposure.
- **U10 shunt in the series feed / V_POE (§3.1).** The PoE-input INA228 shunt (R30) must carry the
  full load current to measure input power, so every buck draws from `V_POE` downstream of R30.
- **All nine INA228 monitors on ADCRANGE=1 with per-rail shunts sized to 50–75 % of FS (§2.1.1).**
  The ±40.96 mV range gives a 4× finer shunt LSB than ±163.84 mV, and no rail on this board needs
  the wide range. Sizing each shunt so the *design maximum* lands in the 50–75 % band leaves transient
  headroom without wasting resolution — and because `Max_Expected_Current` is then the device's own
  full-scale, **SHUNT_CAL is 4096 for all nine**, independent of R (§2.1.2). Two rails sit outside
  the band on purpose: VCC_RB (7 mΩ, 36 %) is sized for the *low-voltage* end of its 4.5–24.45 V
  programmable range, and 3V3_GPS (75 mΩ, 24 %) is sized by insertion drop rather than resolution.
- **Metal-strip 1 % shunts, calibrated in firmware (§2.1.2).** Vishay WSL rather than a tighter
  thin-film part: tolerance is removed once per board by trimming SHUNT_CAL against a reference, but
  TCR and pulse-withstand are not, and metal strip wins on both.
- **Q3 / Q13 PNP high-side orientation (§2.4, §3.8).** Each sustain/disable PNP has its emitter on
  the high rail and collector driving the load node, so the drive turns the device on and sources
  into the load.
- **D25 gate-clamp orientation (§3.9).** Cathode on VCC_RB so the BZX84C12 Zener-clamps |Vgs| of the
  Q25 P-FET gate and reverse-blocks in the OFF/normal region.
- **VCC_RB hard-bounded by fixed resistors (§3.9).** No digipot wiper code (POR/mid-scale/SPI-fault/
  open) may exit the Rb-safe envelope; the digipot only trims within the fixed bound, and the 26 V
  OV latch is the firmware-independent backstop.
- **OCXO Vc caps C0G/NP0 or film (§3.4).** X7R piezoelectric microphonics FM-modulate the 10 MHz
  carrier.
- **Clock mux = plain 74LVC1G157 + firmware HSI bridge + CSS (§3.5).** A dedicated glitch-free mux
  completes its handoff on the outgoing clock's edges and can hang if that source has stopped (the
  Rb-failure case); a plain selector cannot hang.
- **3V3_CLK on the VDD domain (§3.5).** Mux Voh must stay ≤ VDD so PH0 (abs-max VDD+0.3) is never
  over-driven, so the mux runs on the FB10-isolated VDD rail, not the 3.327 V OCXO LDO.
- **Supercap VCHG 2.7 V (§3.3).** Enclosure Tmax = 125 °F (51.7 °C) is below the DSF305Q3R0 65 °C
  corner, so the full 3.0 V rating applies; 2.7 V (VCHG R112/R113 = 13.0k) is the highest TPS61094
  table option under that cap and stores ~1.5× the backup energy of the former 2.2 V. (Supersedes the
  2.2 V/85 °C assumption.)
- **All housekeeping peripherals on always-on 3V3_STM (§3.11, §4).** Gating a rail while its I²C
  pull-ups sit on an always-on rail back-powers the parts through their SDA/SCL clamp diodes; a
  single always-on rail avoids this. The LTC4311 accelerator does not isolate and is not a fix.
- **Panel harness consolidated to one 36-pin J17 (§3.13, §3.15).** A single panel connector carries
  buttons, encoder, panel LEDs, cap-touch, and reed-switch wake; J17.16 = 5V_DISP (display),
  J17.28 = 5 V (encoder/panel logic).
- **GPS SMA ESD = polymer suppressor + external GDT for outdoor (§4).** The line carries 1.5 GHz RF
  (Cj ≤ ~0.3 pF) and 5 V bias (V_RWM ≥ ~6.5 V); ordinary clock-line ESD parts meet neither.
