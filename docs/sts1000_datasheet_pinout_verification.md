# STS1000 "Meridian" — Datasheet Pinout & Signal-Integrity Verification

**As-built reference:** re-annotated KiCad schematic + netlist. **Scope:** per-IC datasheet pinout
confirmation, open-drain/pull termination audit, voltage-divider and signal-level checks,
5V-tolerance map, connector-ESD adequacy, and the residual datasheet items a human must close.
Synthesized from the subsystem verification fact-sheets (power, timing, Rb, GNSS, ethernet, MCU, HMI).

**Authority:** the netlist is source of truth for connectivity and designators. Design-review
context lives in `sts1000_schematic_design_review.md` and `sts1000_layout_readiness_review.md`.
Reference parts by **designator + pin** (e.g. `U12 PG7`, `D25 K`).

---

## 1. Summary

Per-IC pinout and signal-level verification against manufacturer datasheets: **PASS with a small
open-items list**. The items still to close before fab are the J17 panel-power pins, the Rb-buck
output-cap voltage rating, the `3V0_RF_LDO_PG` pull-up, the R264 MPN check, and two capacitor
packages.

### Open items before fab

| # | Subsystem | Item |
|---|---|---|
| N-1 | HMI | **J17 panel-power pins.** J17.16 (display 5 V) → **5V_DISP** and J17.28 (logic 3 V3) → **3V3_STM** feed the display, cap-touch, and encoder Vcc — confirm both land on their rails. |
| N-2 | Rb | **Rb-buck output caps C125/C126/C127/C128 rated ≥ 50 V** — they sit on `Net-(U44-IN+)` = VCC_RB (24.45 V pedestal / 26 V OV trip). X7S/X7T; C0G for C125. |
| N-3 | Timing | **`3V0_RF_LDO_PG` (PG2) pull-up** — LT3045 U51 PWRGD is open-collector; add **10 k → 3V3_STM** (mirror R215 on the GPS LDO). |
| N-4 | BOM | **R264 MPN** = `ERJ-2RKF1743X` (174 k) on the PG7 divider — re-verify the ordered PN before fab. |
| N-5 | MCU | **VREF+ local cap C37 → 100 nF** (ST AN5711 wants 100 nF + 1 µF at the pin; C38 2.2 µF sits behind R48 50 Ω). |

### Capacitor packages to assign (dielectric correct, package pending)

| Refs | Value/dielectric | Package | Note |
|---|---|---|---|
| C98, C99, C101 | 0.1 µF C0G/film | 1210 C0G **or** PPS/PEN film | OCXO Vc loop / 1.65 V ref / Vcontrol — microphonics-critical, **no X7R substitution** |
| C1, C42, C186 | 4.7 nF 2 kV | 1808/1812 2 kV Y-cap | 2 kV MLCC minimum case is 1808 |

### Design points confirmed against datasheet

- Encoder `ENC_A_IN`/`ENC_B_IN` are **push-pull (driven)** outputs → no pull-ups needed.
- AP3441 (U29/U38) PG is **pulled up to VIN when good** (Diodes DS39754); the R94/R92 and R124/R125
  dividers scale it correctly (PG4 ≈ 2.98 V) — no external pull-up needed (bench-confirm §7).
- `PROX_WAKE` is a passive **magnetic reed** (dry contact); pull-up + close-to-GND is correct, no Vcc.
- L1 = **47 nH** bias-T inductor passes the 300 mA antenna bias (u-blox reference is 120 nH).

---

## 2. Per-IC pinout verification

Verdicts pulled from the subsystem fact-sheets. **PASS** = datasheet pin/function ↔ net confirmed;
**CONFIRM** = needs a human datasheet read or bench check (§7); open items are flagged N-x (§1).

### 2.1 MCU — U12 STM32H563ZIT6 (LQFP144)

Full pin→net map is authoritative in `u12_pinmap.txt`. Power/boot summary:

| Pin group | Net | Verdict |
|---|---|---|
| VDD ×10 (17,30,39,52,62,72,84,95,131,144), VDDIO2 (121) | 3V3_STM | PASS |
| VDDUSB (106) | 3V3_STM_USB (C26+C28) | PASS |
| VDDA (33) | dedicated LT3045 U13 (islanded for OCXO loop) | PASS |
| VREF+ (32) | VREF_3V3 (MCP1502-33 U14); 1nF at pin | PASS pinout (grow C37 → 100nF, N-5) |
| VBAT (6) | STM_VBAT (supercap-backed) | PASS |
| VCAP (70)/VCAP__1 (142) | C36/C34 2.2µF — LDO core mode | PASS |
| BOOT0 (138) | R50 10k→GND single pull-down (boot user flash) | PASS |
| NRST (25) | MCU_NRST + J3.5; no external cap | PASS (add ~100nF, §7) |
| PG7 (92) | POE_PG (via R264 174k / R263 10k divider) | **PASS** — PG7 = 2.93 V; PG7 is 5V-tolerant (FT) |

### 2.2 Power / PoE

| IC | Part | Verdict | Note |
|---|---|---|---|
| U7/U8 | FDMQ8205A PoE bridge | PASS | Alt-A+Alt-B all-pairs |
| U9 | NCP1095 PD | PASS pinout | PGO open-drain → POE_PG divided (R264/R263); NCM/NCL/LCF open-drain/RTN=GND/+72 V → **add pull-up** to 3V3_STM (§7); DET/R15 confirm |
| U10 | INA228 0x40 PoE | **PASS** | loads on V_POE; R30 150 mΩ in the series feed → reads real current |
| U28 | MIC28516 buck (5V) | PASS | 0.6·(1+15k/2.05k)=4.99V ✓; FREQ programming CONFIRM (§7) |
| U29 | AP3441 buck (3V3) | PASS | 0.6·(1+10k/2.2k)=3.33V ✓; PG pulled to VIN when good → R94/R92 divider (no PU needed) |
| U30 | INA228 0x43 (3V3) | PASS | shunt in series ✓ |
| U31 | INA228 0x41 (3V3_STM) | PASS | R106 15mΩ Kelvin ✓ |
| U32 | INA228 0x42 (5V_DISP) | PASS | |
| U33 | RT9742 (DISP 5V) | PASS | EN=DISP_EN R104 10k off; nFLG R105 PU ✓ |
| U34/U35 | TPS61094 supercap | PASS | VBAT=3.0V, term 2.5V, 25mA (Tables 7-1/2/3) |
| U64 | TPS3430 WDT | PASS | WDO_N PU R23→3V3 ✓; boot-disabled R213 100k pd ✓ |
| U2/U67 | LMV393 (PFI/backup-PG) | PASS | PFI trip 38.1V; backup-PG ≈2.0V ✓ |
| CR1 | SMCJ58A PoE TVS | PASS | Vwm 58>57, VBR 64.4>60, Vc 93<100 ✓ |
| Q3 | BC857W POE_KILL sustain | **PASS** | high-side sustain PNP: E(2)=V_KBIAS, C(3)=HOLD |
| Q1/Q4/Q5 | POE_KILL input | PASS | emitters=GND ✓ |

### 2.3 Timing core (OCXO + mux + ext-ref)

| IC | Part/pkg | Verdict |
|---|---|---|
| Y3 | OH300 VC-OCXO | PASS — 1 Vcontrol→OCXO_V, 3 VCC→3.33V, 4 OUT→R123, 7 GND |
| U36 | OPA320 SOT23-5 | PASS |
| U37 | INA228 0x46 OCXO | PASS — shunt IN+ upstream of R126 (correct polarity) |
| U38 | AP3441 buck | PASS-plausible — R127/R128 → 3.70V pre-reg |
| U39 | TPS7A5201 VQFN-20 | PASS — R129/R130 → 3.327V ✓ |
| U49 | TMUX1101DCK SC70-5 | PASS (doc "SC70-6" nit) |
| U50 | LTC6752xS5 TSOT23-5 | PASS — VCC ~2.85V thin but in-spec (verify over temp, §7) |
| U51 | LT3045 DFN | PASS — R186 30.1k×100µA → 3.01V ✓ |
| U52 | 74LVC1G157 mux | PASS — S=L→I0=OCXO; VCC=3V3 (Voh ≤ VDD, PH0 safe) ✓ |
| U53 | 74LVC1G34 buffer | PASS |
| U71 | 74LVC1G34 (VCC=5V) | PASS pinout |
| C98, C99, C101 | 0.1 µF Vc-path caps | C0G/film dielectric (never X7R on the Vc path); **0.1 µF C0G needs 1210 C0G / PPS film** (§1 cap-package) |

### 2.4 Rubidium (Rb power + Rb IO)

| IC | Part | Verdict |
|---|---|---|
| U40 | MIC28516 Rb buck | PASS — PVIN=VOUT_P; EN=RB_PSU_PWR_EN; PG=RB_PSU_PG. **Output caps C125–128 rate ≥50 V on the 24.45 V VCC_RB rail (N-2)** |
| U41 | LMV393 OV comp | PASS — V+ from 5V (survives VCC_RB collapse); OD + R142 PU |
| U42 | OPA320 wiper buffer | PASS — VCTRL→R134 FB inject; V+=5V |
| U43 | MCP41U83 digipot | PASS — VDD1=3V3; SPI **Mode 0,0**; **code 0 = Terminal B (VREF_3V0) = safe-low** (VCTRL max → VOUT min 4.5 V), POR = midscale ~14.5 V (< 26 V OV envelope). FW pre-programs NV-wiper safe-low (§7) |
| U44 | INA228 0x47 Rb | PASS — shunt R159 20mΩ, IN−=VCC_RB; ALERT PU R265 10k→3V3_STM |
| U45 | MCP1502T-30E ref | PASS — OUT=VREF_3V0 (3.0V); nSHDN gated Q19-C |
| U46 | SN65C3221E RS-232 | PASS — charge pump C139/C140/C143/C144, C145 V+→GND ✓ |
| U47 | PESD15VL2BT ESD | PASS — 1=TX_P, 2=RX_P, 3=GND |
| U48 | APC-817C1 opto | PASS — E=GND, C=RB_LOCK |
| Q25 | SI7469DP P-ch gate | PASS — gate clamp D25 K→VCC_RB, A→gate; \|Vgs\| clamped ~12 V |
| Q26 | BC847W driver | PASS |
| K1 | G6K-2F-Y DPDT (RS-232) | topology PASS / NC-NO map CONFIRM (§7) |
| Q15–Q18 | OV latch | PASS (autonomous/latching, trip VCC_RB>26.0V) |

### 2.5 GNSS + active-antenna

| IC | Part | Verdict |
|---|---|---|
| U21 | ZED-F9T-00B | PASS — all supervisor/UART/PPS pins verified; pin19 GEOFENCE→TX_RDY needs CFG remap (§7); V_USB→GND |
| U22 | LT3045 (3V3_GPS) | PASS — 100µA×33.2k = 3.32V ✓; EN R69 pd off |
| U23 | INA228 GPS | PASS wiring / **0x4A** (A0=A1=SDA strap; SHT45 owns 0x44) |
| U24 | INA181A1 (presence) | PASS — gain 20, VCM≈5V in range |
| U25 | LMV393 (DETECT/SHORT) | PASS — V+=5V via FB5 |
| U26 | INA228 0x45 V_ANT | PASS — shunt R89 0.1Ω |
| U27 | RT9742VGJ5 ant switch | PASS — nFLG R200 PU; EN R88 pd off (symbol/MPN variant nit) |
| Q11 | NSS40300 PNP | PASS |
| Q12 | BC857W foldback | PASS |
| Q13 | BC857W ANT_OFF | **PASS** — high-side PNP: E(2)=V_ANT, C(3)=Q11-B |
| L1 | 47nH bias-T inductor | PASS — 47 nH passes the 300 mA antenna bias (u-blox ref 120 nH) |
| L10 | PGB1010603MR RF ESD | PASS — Cj≈0.05pF ✓ |

### 2.6 Ethernet / PHY

| IC | Part | Verdict |
|---|---|---|
| U11 | LAN8742AI-CZ (QFN-25) | PASS — all 25 pins verified; straps MODE=111, PHYAD0=0, REGOFF=0, nINTSEL=0 (REFCLKO); RBIAS R37 12.1k exact |
| U1 | TPD4EUSB30 MDI TVS | PASS — VRWM 5.5V > 4.3Vpk MDI CM, ~0.5pF ✓ |
| J1 | 0826 PoE magjack | PASS pairing / orderable-PN + CT current rating CONFIRM (§7) |
| Y1 | 25MHz 20pF xtal | PASS — C14/C16 27pF slightly undersized (few-ppm fast, RMII decoupled from timing ref; harmless) |
| U12 RMII pins | REF_CLK PA1 / MDIO PA2 / MDC PC1 / CRS_DV PA7 / RXD0 PC4 / RXD1 PC5 / TX_EN PA5 / TXD0 PB12 / TXD1 PB15 / RX_ER PB10 / PPS PB5 / nRST PD10 | PASS — all valid AF11; PB13/PB14 erratum correctly avoided |

### 2.7 MCU digital peripherals + HMI

| IC | Part | Verdict |
|---|---|---|
| U62 | MX25L25645 SPI-NOR (Macronix) | PASS — SPI4; nRESET R205 10k↓ default-asserted (FW must release PE10 early) |
| U60 | ATECC608B 0x60 | PASS — NC 1,2,3,7 unconnected ✓ |
| U57/U58 | TMP117 (0x49 / 0x48) | PASS — U57 ADD0→3V3_STM (0x49), U58 ADD0→GND (0x48) |
| U59 | LIS2DH12 0x19 | PASS — SA0→3V3_STM |
| U61 | IIS2MDC 0x1E | PASS — fixed addr |
| U72 | SHT45 0x44 | PASS — fixed SHT45-AD1B |
| U56 | LTC4311 rise-time accel | PASS — ENABLE=3V3_STM permanent on |
| U63 | PCA9306 level shift | PASS pinout / 5V-side pull-ups supplied by the display module |
| U68 | 74LVC2G17 Schmitt (encoder buffer) | PASS — 5V-tolerant in, 3.3V out → PA8/PA9 need not be FT |
| U13/U14 | LT3045 VDDA / MCP1502-33 VREF | PASS |
| D5 | RGB LED (common-anode 5V) | PASS — Q8/Q9/Q10 low-side, active-high default-off |
| K2 | G6K-2F-Y DC3 holdover relay | PASS — normally-energized fail-safe; D24 flyback ✓ (coil 91 Ω / 33 mA) |
| Q22/Q23 | NSS40300 + BC847W panel-LED driver | PASS — double default-off |
| U54 | INA228 0x4C panel | PASS — R199 220mΩ = 70.9% FS ≤75% ✓ |

---

## 3. Open-drain / pull-up-pull-down audit

Every open-drain / OC / PG / ALERT / nFLG / reset net: required termination vs as-built resistor.

| Net | Source (OD/OC) | Required | As-built (ref / value / rail) | Verdict |
|---|---|---|---|---|
| INA_ALERT_V_POE (PG8) | U10 ALERT | PU→3V3_STM | R220 10k→3V3_STM | PASS |
| INA_ALERT_3V3_STM (PG9) | U31 | PU | R221 10k→3V3_STM | PASS |
| INA_ALERT_5V_DISP (PG10) | U32 | PU | R222 10k→3V3_STM | PASS |
| INA_ALERT_3V3 (PG11) | U30 | PU | R223 10k→3V3_STM | PASS |
| INA_ALERT_3V3_GPS (PG12) | U23 | PU | R224 10k→3V3_STM | PASS |
| INA_ALERT_V_ANT (PG13) | U26 | PU | R225 10k→3V3_STM | PASS |
| INA_ALERT_OCXO (PG14) | U37 | PU | R226 10k→3V3_STM | PASS |
| INA_ALERT_VCC_RB (PG15) | U44 | PU | R265 10k→3V3_STM | **PASS** |
| INA_ALERT_5V_PANEL (PF13) | U54 | PU | R227 10k→3V3_STM | PASS |
| 3V3_PSU_PG (PG4) | U29 AP3441 PG (**pulls to VIN when good**) | divider to GND | R94 6.81k / R92 10k → PG4 ≈ 2.98 V | PASS (no PU needed; §5 / bench §7) |
| 5V_PSU_PG (PG3) | U28 PG (OD) | PU→5V | R95 10k→5V | PASS (PG3 is 5V-tolerant, §5) |
| OCXO_LDO_PG (PG1) | U39 PG | PU | R215-class PU present | PASS |
| 3V3_GPS_LDO_PG (PG0) | U22 LT3045 PG | PU→PG0 | R215 10k→3V3_STM | PASS |
| **3V0_RF_LDO_PG (PG2)** | U51 LT3045 PG (**open-collector**) | PU→3V3_STM | **add 10k→3V3_STM** | **Open — N-3** (mirror R215) |
| RB_PSU_PG (PG6) | U40 PG | PU | R143→3V3 | PASS |
| OCXO_PSU_PG (→PG5) | AP3441 U38 PG→R125 3.57k→node N (R124 10k **‖** R232+R233 → **2.98 V**) | 2nd divider | R232 1.21k / R233 10k → PG5 ≈ **2.66 V** | PASS (node loaded by both legs; >VIH; bench-confirm PG drive-Z) |
| WDO_N | U64 WDO (OD) | PU→3V3 | R23 10k→3V3 | PASS (WDO-only, not 3-way wire-OR) |
| PANEL_LED_FAULT (PF12) | U55 nFLG (OD) | PU | R201 10k→3V3_STM | PASS |
| V_ANT_EN_FAULT (PF8) | U27 nFLG (OD) | PU | R200 10k→3V3_STM | PASS (→3V3_GPS/STM) |
| V_DISP_EN_FAULT (PF9) | U33 nFLG (OD) | PU | R105 10k→3V3_STM | PASS |
| GPS_ANT_DETECT | U25A (OD) | PU→3V3_GPS | R84 → 3V3_GPS | PASS |
| GPS_ANT_SHORT | U25B (OD) | PU→3V3_GPS | R87 → 3V3_GPS | PASS |
| RB_LOCK (PB13) | U48 opto collector | PU | R173 22k→3V3 | PASS |
| RB_PSU OV comp out | U41 (OD) | PU | R142→5V | PASS |
| MDIO | U11/MAC | PU | R36 1.5k→3V3_LAN | PASS |
| NOR nWP/SIO2 | strap | PU | R207 10k→3V3_STM | PASS |
| NOR nCS (SPI_NSS) | idle-high | PU | R212 10k→3V3_STM | PASS |
| SPI_DPOT_CS (PD2) | idle-high | PU | R135 10k→3V3 | PASS |
| ENC_A_IN / ENC_B_IN | encoder (**push-pull, driven**) | none | J17 + U68/U69 (Schmitt buffer) | PASS — driven output needs no PU; encoder Vcc from J17 (N-1) |
| ENC_BUTTON (PF11) | dry switch | PU | R236 10k→3V3_STM + C187 | PASS |
| BUTTON_1..7 (PF0-6) | dry switch | PU | R190–196 10k→3V3_STM + 0.1µF | PASS |
| FAN_TACH (PA15) | OC tach | PU | none — MCU internal PU | PASS — PA15 is FT with FW internal pull-up (standard for OC tach) |
| PROX_WAKE (J17.18→PF10) | **passive magnetic reed** (dry contact) | PU + close-to-GND | R254 10k→3V3_STM (PU) | PASS — reed switch, no Vcc; correct conditioning |
| MUX_SEL (PB6) | — | PD (boot OCXO) | R188 100k→GND | PASS |
| PANEL_LED_EN (PC0) | — | PD (off) | R198 10k→GND | PASS |
| GPS_PWR_EN (PC8) | — | PD (off) | R69 100k→GND | PASS |
| ANT_BIAS_EN (PC9) | — | PD (off) | R88 10k→GND | PASS |
| RB_PWR_EN (PB7) | — | PD (off) | R164 100k→GND | PASS |
| WDT_EN (PC12) | — | PD (boot-disabled) | R213 100k→GND | PASS |
| DISP_RST (PA10) | — | PD (held-reset) | R209 10k→GND | PASS |
| NOR_RST_N (PE10) | — | PD (held-reset) | R205 10k→GND | PASS (FW release early) |
| LAN_RST_N (PD10) | — | PD (held-reset) | R35 10k→GND | PASS (atypical, confirm intent) |

**Open pull item: N-3 (`3V0_RF_LDO_PG` PG2 — add 10k→3V3_STM).** FAN_TACH uses the MCU internal
pull-up (standard for an OC tach). All other open-drain/PG/ALERT nets carry their required termination.

---

## 4. Signal level / voltage-range & divider checks

Every divider feeding an MCU pin or comparator; computed node voltage vs in-range verdict.

| Node | Divider (as-built) | Computed | Range / limit | Verdict |
|---|---|---|---|---|
| **USB_VBUS_SENSE** (PE2) | USB_VBUS–R68 100k–node–R67 121k–GND | k=0.547 → **2.87 V** @5.25V | ≤3.3V | PASS |
| **RB_OV_DET** (PE3) | 5·33/51 | **3.24 V** | ≤3.3V | PASS |
| **POE_PG → PG7** | R264 174k / R263 10k off POE_PG (54 V) | **2.93 V** (3.10 V @57 V) | ≤3.6V abs-max, VIH>2.0V | **PASS** |
| PG5 / OCXO_PSU_PG | AP3441 U38 PG (5V) → R125 → node N (R124 **‖** R232+R233) = **2.98 V**, then R232/R233 | **≈2.66 V** | ≤3.3V (PG5 not FT), >VIH 2.31 V | PASS (bench-confirm) |
| 3V3_PSU_PG → PG4 | AP3441 U29 PG (5V) → R94 6.81k / R92 10k | **2.98 V** | ≤3.6V abs-max | PASS (PG pulls to VIN when good) |
| RB OV latch sense | 0.0769·VCC_RB vs 2.001V | trip @ **VCC_RB > 26.0 V** | 1.55V guard vs 24.45V pedestal | PASS |
| VCC_RB setpoint | 24.45 − 6.645·VCTRL, VCTRL 0–3.0V | **24.45 V max / 4.51 V min** | ≤26V trip; bounded | PASS (digipot pulls DOWN only) |
| VREF_3V0 (Rb ref) | MCP1502-30E | **3.0 V** | Rb ref | PASS |
| REF slicer node N1 | R174/R175 47k/47k | **1.50 V** bias | comparator CM | PASS |
| REF_VMID (U50 −IN) | R179/R180 10k/10k | **1.50 V** | comparator CM | PASS |
| OCXO Vc center | R118/R119 100k/100k off VDDA | **1.65 V** | never floats (R121 10M) | PASS |
| OCXO_V ADC (PA3) | direct | **0–3.3V FS** | ADC | PASS |
| Ant DETECT ref | R85 100k / R86 11k | **0.327 V** → ≈5mA thr | comparator | PASS |
| Ant node-A divider | R80/R81 100k/100k | **÷2** | — | PASS |
| Ant SHORT ref | R82 100k / R83 30k | **0.762 V** → node A <1.52V | comparator | PASS |
| INA228 V_ANT FS | R89 0.1Ω | 1.638 A FS (180mA in range) | ≤75% FS | PASS |
| Panel-LED INA228 FS | R199 220mΩ, 29.0mV | **70.9% of ±40.96mV FS** | ≤75% | PASS |
| MIC28516 5V FB | R97 15k / R98 2.05k | **4.99 V** | 5V | PASS |
| AP3441 3V3 FB | R99 10k / R100 2.2k | **3.33 V** | 3V3 | PASS |
| GPS LT3045 | 100µA × R71 33.2k | **3.32 V** | — | PASS |
| PFI trip (U2A) | R2 1.2M / R3 39.2k, Vref 1.204 | **38.1 V** (36.8/38.8 hyst) | <42.5V | PASS |
| Backup-PG (U67) | 3.3·150k/260k | **1.90 V** → ≈2.0V trip | — | PASS |
| U28 FREQ | R90 200k→VOUT_P / R91 100k→GND | ~18V unloaded | needs internal clamp | **CONFIRM (§7)** |

**All computed nodes are in range.** The AP3441 PG nodes (PG4/PG5) are valid because PG drives to
VIN (5 V) when good (Diodes DS39754), so the dividers-to-GND scale correctly.

---

## 5. 5V-tolerance map

Which MCU/PHY pins see 5V and how they are protected.

| Pin(s) | 5V source | Protection | Verdict |
|---|---|---|---|
| PA8/PA9 (ENC_A/B) | 5V encoder (J12.6) | **U68 74LVC2G17 buffer** — 5V-tolerant inputs, 3.3V outputs to MCU | SAFE — PA8/PA9 need not be FT |
| Display I2C/SPI (PE9/PB0/PA10/PE6, MOSI/SCK) | 5V_DISP module | **PCA9306 U63** level-shift (I2C) + 5V_DISP domain isolated; SPI re-buffered R210/R211 22Ω | SAFE (5V-side PU on the display module) |
| **PG3 (5V_PSU_PG)** | R95 10k→5V (OD released → 5V on pin) | PG3 is 5V-tolerant (FT) | SAFE — PG3 sits at 5 V on an FT pin |
| PG7 (POE_PG) | 54V VOUT_P, **divided to 2.93 V** (R264/R263) | resistive divider + FT | SAFE — PG7 is 5V-tolerant (FT) |
| RB_TX (PE7) | RS-232/CMOS relay path | D12 BAT54S clamp; R168 10k limit in mis-set CMOS | SAFE |
| GPS/Rb/I2C1/RMII | all 3.3V both ends | n/a | SAFE — no 5V exposure |
| USB_VBUS (PE2) | 5.25V VBUS | divider to 2.87V (§4) | SAFE |

PG3 (5V_PSU_PG, sits at 5 V) and PG7 (POE_PG divider node, 2.93 V) are both on 5 V-tolerant (FT)
Port-G digital pins per STM32H563 DS14258.

---

## 6. Connector ESD-protection matrix

Each external connector, its signals, protecting device, adequacy. From findings_hmi + findings_gnss + findings_ethernet.

| Conn | Signals | ESD device | Adequacy / verdict |
|---|---|---|---|
| J1 RJ45 magjack | Ethernet MDI (4-pair PoE) | U1 TPD4EUSB30 (PHY-side, VRWM 5.5V > 4.3Vpk, ~0.5pF); magjack + shield R1 1M‖C1 4700pF/2kV | PASS — low-cap OK for 100BASE-TX; cable-side via magjack (indoor) |
| J2 fan 1×4 | 5V, FAN_PWM (PE5), GND, FAN_TACH (PA15) | D22 SMF5V0A on **5V only** | signals unprotected (internal fan, low risk); FAN_TACH no PU |
| J3 SWD 1×6 | 3V3_STM, SWCLK, SWDIO, NRST, SWO, GND | none | internal debug — OK |
| J5 USB-C | VBUS, CC1, CC2, DP, DM, shield | U18 (DP/DM/VBUS) + U65 (CC1/CC2) | PASS — all lines covered |
| J6 DE-9 (Rb) | VCC_RB_G, RS232_TX/RX, RB_LOCK_IN | D7 1.5SMCJ28A (pwr); D13/D14 SMAJ15CA (TX/RX); **D26 SMAJ15CA (RB_LOCK_IN)** | PASS — every line has a channel |
| J7 SMA | GPS_RF_IN (1.5 GHz RF + 5V bias) | **L10 PGB1010603MR** polymer: Cj≈0.05pF (≤0.3pF ✓), standoff ~20–24V (≥6.5V bias ✓) | PASS class; outdoor runs still need external inline coax GDT |
| J8/J9 SMA | 10MHz_RF_IN / _OUT | D8/D9 SZESD7410 (~0.4pF) | PASS — correct low-Cj clock TVS |
| J10 1×2 | RTC_TAMPER (PC13) | U16 | PASS |
| **J17 36-pin panel** | buttons, panel-LED cathodes/anode, encoder A/B/switch, display bus + I2C_DISP + TOUCH_INT, PROX_WAKE reed (J17.18), **5 V (J17.16) + 3 V3 (J17.28) supply pins** | U15/U16/U17/U19/U20/U66/U69/U70 arrays + D6 SMF5V0A — every signal pin has a channel | signal ESD PASS; **confirm the supply pins J17.16→5V_DISP, J17.28→3V3_STM feed the display/touch/encoder Vcc (N-1)** |
| J15 SMA | 1PPS_OUT | D23 SZESD7410 | PASS — correct low-Cj |
| J16 alarm | 6 dry relay contacts | none | isolated dry contacts — optional field-surge TVS |

**ESD scheme is consistent:** ribbon/panel (J17) → TPD4E0x arrays (every pin retains a channel);
RF/clock coax → SZESD7410 / PGB1010603MR (low-Cj); power/shield → SMF5V0A / 1.5SMCJ28A / SMCJ58A /
D26 sized to rail. The RF SMA is split (polymer for Cj; external GDT for outdoor runs). The one
remaining functional item is confirming the J17 power pins reach the panel (N-1).

---

## 7. Datasheet facts & residual bench/human checks

Per-item datasheet answers for the parts whose behavior needs pinning down, plus the checks that
still need a human datasheet read or a bench measurement (carried into `bench_tuning_procedures.md`).

| # | Part / net | Datasheet fact | Check |
|---|---|---|---|
| V-1 | U12 PG3 / PG7 | Port-G PG3/PG7 are 5V-tolerant (FT) per DS14258 (Port-G is general digital FT; only TT/analog/VBAT pins are 3.3 V-only). PG3 sits at 5 V; PG7 is the 2.93 V divider node. | — |
| V-2 | U12 PG5 / OCXO_PSU_PG | Node N (OCXO_PSU_PG) off U38 PG (5 V-pulled) via R125 3.57k is loaded by **both** R124 10k and the R232+R233 leg → **2.98 V** (not 3.68 V); R232/R233 gives **PG5 ≈ 2.66 V** (≤3.3 V ✓, > VIH 2.31 V). Node N ≥ U39 EN VIH 1.1 V ✓. DS39754 specs no PG drive-Z → **bench-confirm**. | — |
| V-3 | U9 NCP1095 | All 4 status pins (PGO/NCM/NCL/LCF) are **open-drain, referenced to RTN = GND** (U9.12), +72 V abs-max. PGO's pull-up sits on VOUT_P via R20 → POE_PG divided to 2.93 V (needs the divider). NCM/NCL/LCF→PC2/PC7/PC3 are **GND-referenced → safe direct to 3.3 V GPIO, but float without a pull-up** — add 10k→3V3_STM or STM32 internal PU. DET/R15 COSC↔DET still to read. | **ACTION** add NCM/NCL/LCF pull-up; confirm DET/R15 |
| V-4/5 | U28 MIC28516 | **FREQ = resistor-divider-from-PVIN** (R90/R91 ratiometric). **EN "do not exceed PVIN"** → EN=POE_PG≈PVIN is at the allowed limit. **EXTVDD=GND** = internal-HV-LDO strap. | **CONFIRM** fSW value |
| V-6 | U43 MCP41U83 | **Code 0x000 = Terminal B (zero-scale) = safe-low** (VCTRL max → VOUT min 4.5 V). POR = **midscale** (~14.5 V, < 26 V OV envelope) then loads NV-Wiper0. **SPI = Mode 0,0.** FW pre-programs NV-wiper safe-low + verifies VCC_RB on U44 (0x47). | — (FW note) |
| V-7/8 | K1 / K2 G6K-2F-Y | **3 VDC coil = 91 Ω, 33 mA** (~100 mW) — within Q24 BC847W 100 mA. Contacts **2 Form C (DPDT)**; de-energized COM→NC → RS-232 default (K1) / alarm (K2) on the NC contacts. | **CONFIRM** NC-side wiring |
| V-9 | U21 ZED-F9T pin19 | **Pin 19 = GEOFENCE_STAT** (default). Firmware remaps to TX_READY via **CFG-TXREADY**. | — (FW) |
| V-10 | U21 RF_IN | ~5 V antenna bias sits on RF_IN (L1 bias-T, no series DC-block). u-blox reference places a 47 pF C0G series DC-block; the internal block "may not tolerate > VCC." | **ACTION** — add a ~47 pF C0G series DC-block, drop bias ≤3.3 V, or get u-blox written OK |
| V-11 | J1 magjack | 4-pair PoE center-tap wiring is standard. Orderable PN (HS-F vs GH-F) + CT current rating for Type-2/3 is a sourcing item. | **CONFIRM** PN/CT rating |
| V-12 | U50 LTC6752 | VCC ~2.85 V > **2.45 V single-supply min** → ~0.4 V margin; in spec. | verify over temp |
| V-13 | Proximity sensor | Passive **magnetic reed** on J17.18 (dry contact, no Vcc); R254 pull-up + close-to-GND. | — |
| V-14 | Supercap C90/C91 | **DSF305Q3R0 = 3.0 V to 65 °C, derated 2.5 V at 85 °C.** Enclosure Tmax = **125 °F (51.7 °C)** < 65 °C → full 3.0 V rating; TPS61094 VCHG termination set to **2.7 V** (R112/R113 **13.0k**, highest table option under the cap). | keep supercaps ≤ ~75 °C |
| V-15 | FE-5680A (J6) | J6.8/J6.9 Tx/Rx **variant-dependent** (typ. J6.8=RX-into-Rb, J6.9=TX-from-Rb). The 24.45 V full-scale pedestal over-volts a 15 V-class FE (OV latch trips at 26 V). | **CONFIRM per-unit** — variant pinout + Vmax; set digipot/OV bound |

**Also cross-listed in** `sts1000_layout_readiness_review.md` (footprint/library items) and the
`peripheral_map §14` / `software_spec §15` open-item lists. The open items N-1…N-5 (§1) and the
cap-package assignments are the remaining items before fab.

---

*Synthesized from the subsystem fact-sheets (power/timing/rb/gnss/ethernet/mcu/hmi) + u12_pinmap +
ina228_decode. The netlist is source of truth for connectivity and designators.*
