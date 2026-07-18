# STS1000 "Meridian" — Schematic Implementation Checklist

High-level IC / circuit inventory for verifying schematic completeness. One line per active part or distinct circuit block; key support passives folded in. The authoritative pin↔net contract is `sts1000_firmware_hardware_interface.md`; the design detail is `sts1000_hardware_design_reference.md`; the netlist-driven consistency review is `sts1000_schematic_design_review.md`.

> **Open items before layout / fab** (genuinely-open work; everything else in this checklist is the current design):
> - [ ] **GPS RF_IN series DC-block.** The u-blox ZED-F9T reference antenna-bias design (Integration Manual UBX-21040375) places a 47 pF C0G series DC-block between the bias-T node and RF_IN. As-built `GPS_RF_IN` ties the 5 V-biased node directly to U21.2. Add a ~47 pF C0G series DC-block (so the 5 V bias reaches the antenna only), or confirm the ZED-F9T internal block tolerates continuous 5 V. Supervisor/current-sense sit on the V_ANT side and are unaffected.
> - [ ] **Rb buck output caps C125/C126/C127/C128 ≥ 50 V.** They sit on VCC_RB (24.45 V pedestal / 26 V OV); source ≥ 50 V parts (47 µF 50 V won't fit 1210 → 1210/1812 or split; C0G for feedforward C125).
> - [ ] **Cap packages:** C98/C99/C101 0.1 µF C0G → **1210 C0G / PPS film** (0.1 µF C0G is unbuildable in 0402); C1/C42/C186 4.7 nF 2 kV → **1808/1812** Y-cap.
> - [ ] **C37 (VREF+) 1 nF → 100 nF** (optional ADC-ENOB improvement for the OCXO steering path; ST AN5711 wants 100 nF + 1 µF at VREF+, keep the R48/C38 filter).
> - [ ] **Bench-verify:** AP3441 PG active-drive to VIN (confirm the PG-divider nodes reach ~2.98 V when good — node N 2.98 V / PG5 2.66 V, both loaded, both > VIH; the OCXO LDO enable depends on it); ZED-F9T V_BCKP current at enclosure Tmax; FE-5680A J6.8/J6.9 Tx/Rx direction per surplus variant; **NCP1095 NCM/NCL/LCF** resolved (open-drain/RTN=GND/+72 V — add 10k→3V3_STM or STM32 internal pull-up); per-rail INA228 shunt final values.

---

## 1. MCU & core
- [ ] **U12 STM32H563ZIT6** (LQFP144) — MCU. **SWD-only** (PB4/NJTRST reused as `RB_RX`)
- [ ] HSE **bypass** on PH0 ← clock-mux `CLK_OUT`; PH1/OSC_OUT = NC
- [ ] **LSE Y2 32.768 kHz** crystal on PC14/PC15 (RTC) + load caps
- [ ] VDD/VDDA decoupling, VREF+ (**C37 → 100 nF**, see open items), VBAT backup-domain feed
- [ ] Boot/SWD, MCU_NRST, WDT_KICK (PB2), PFI (PE8); **U64 TPS3430** window WDT
- [ ] **U13 LT3045** VDDA LDO; **U14 MCP1502-33** 3.3 V STM ref

## 2. Timing reference — OCXO
- [ ] **Y3 OH300-61003CV-010.0M** VC-OCXO (10 MHz, 3.0 V CMOS)
- [ ] OCXO low-noise supply LDO (**U39 TPS7A52**), always-on, dedicated; buck **U38 AP3441**
- [ ] DAC loop filter: PA4 (DAC1_OUT1) → filter → Vc (pin 1), **centered ~1.65 V**, Vc never floating
  - [ ] Vc-path caps C98/C99/C101 **C0G/NP0 or film — never X7R** (microphonic FM); 0.1 µF C0G needs a **1210 C0G or PPS/PEN film** package (unbuildable in 0402)
- [ ] **R123 22 Ω** source termination at OCXO output (→ mux I0)
- [ ] PA3 (ADC INP15) Vc sense — no divider
- [ ] **INA228 #5 @0x46 (U37)** OCXO-rail monitor

## 3. Clock-source mux (`sts1000_clock_mux.md`)
- [ ] **U52 74LVC1G157GW-Q100H** — 2:1 LVCMOS selector (A=OCXO, B=ext)
- [ ] **U53 74LVC1G34GW-Q100** — bench-fanout buffer
- [ ] **R188 100 k** MUX_SEL (PB6) pull-down → default OCXO
- [ ] **R187 22 Ω** Y→PH0 damp; **R189 22 Ω** fanout source-match
- [ ] **C160 100 nF** fanout DC-block; **D16 SZESD7410** at SMA (off-page) <!-- TODO verify designator: C141, D16 -->
- [ ] **FB10** ferrite 3V3→3V3_CLK (VDD domain); C158/C159 decoupling <!-- TODO verify designator: C140/C142/C143 -->


## 4. External-reference RF front end (`ntp_server_rf_frontend.md`)
- [ ] SMA input (`10MHz_RF_IN`)
- [ ] **D18 SZESD7410** ESD at connector <!-- TODO verify designator: D11 -->
- [ ] **C150 100 nF** DC block (50 V) <!-- TODO verify designator: C132 -->
- [ ] **R174/R175 47 k** bias node N1 (1.5 V)
- [ ] **U49 TMUX1101** switched 50 Ω termination + **D19 BAT54S** pin-2 clamp (REF_TERM_EN = PC10) <!-- TODO verify designator: D15 -->
- [ ] **U50 LTC6752xS5** comparator slicer; REF_VMID = R179/R180; R181/R182/C151 input net <!-- TODO verify designator: C136 -->
- [ ] Protection: island/VCC clamp diodes D20/D21 (3V3 zener) <!-- TODO verify designators: D12/D13/D14, R137/R143 -->
- [ ] **U51 LT3045** 3.0 V island LDO + **FB9** ferrite <!-- TODO verify designator: FB8 -->
- [ ] **R184 33 Ω** source term at U50 Q (→ mux I1, EXTREF_MON PB14)

## 5. Rb variable supply (`sts1000_vcc_rb_supply.md`)
- [ ] **U40A MIC28516** adaptive-on-time buck (VCC_RB 5–24 V) + **L7 40 µH** + COUT bank — **output caps C125/C126/C127/C128 ≥ 50 V** (VCC_RB pedestal 24.45 V; see open items)
- [ ] **U43 MCP41U83T-503E/ST** SPI digipot (FB trim; SPI2C→DGND; CS=PD2)
- [ ] **U42 OPA320** wiper buffer; **U45 MCP1502-30E** **3.0 V** reference (`VREF_3V0`, gated by RB_PWR_EN via Q19/Q20)
- [ ] FB network R147/R148/R134 → **`VCC_RB = 24.45 − 6.645·VCTRL`** (VCTRL 0…3.0 V); pedestal 24.45 V fixed, digipot pulls **down** only
- [ ] **Fixed bounding resistors** so no wiper code (POR/mid/SPI-fault/open) exits the Rb-safe envelope; **verify code-0 parks wiper safe-low before any RB_PWR_EN**
- [ ] **D_TVS 1.5SMCJ28A** at clock connector
- [ ] **U44 INA228 @0x47** VCC_RB monitor + shunt <!-- TODO verify designator: R108 shunt -->
- [ ] **OV latch:** U41A LMV393 + Q transistors + D (26 V trip); RB_OV_DET (PE3) / RB_OV_RESET (PD3) <!-- TODO verify designators: Q10/Q11/Q12/Q13, D4 -->
- [ ] EN interlock (RB_PWR_EN = PB7 → buck EN) <!-- TODO verify designators: Q8/Q9 -->


## 6. GNSS — ZED-F9T
- [ ] **U21 u-blox ZED-F9T-00B** receiver + **U22 LT3045** GPS LDO
- [ ] **V_BCKP** feed (stays up through GPS VCC cycles)
- [ ] USART3 (PD8/9) + UART4 RTCM (PD0/1) + TIMEPULSE (PA0) / TIMEPULSE2 (PC6)
- [ ] RESET_N (PD11), SAFEBOOT_N (PD15), DSEL (PD7), EXTINT (PD6), TXRDY (PD5)
- [ ] **GPS VCC switch (PC8)**; **INA228 #3 @0x4A (U23)** GPS-rail monitor (A0=A1=SDA strap; **0x44 is the SHT45 U72** — do not re-strap)

## 7. GNSS antenna bias + supervisor (`gnss_antenna_bias_supervisor.md`)
- [ ] Bias-T: **L1 47 nH** choke + C <!-- TODO verify designator: C26 -->
- [ ] **Q11** PNP pass + foldback **Q12** + **R77 3.3 Ω** sense (across U24 IN+) <!-- TODO verify foldback designator: old Q1/R29 -->
- [ ] **U24 INA181A1** current sense (gain 20)
- [ ] **U25 LMV393** dual comparator (DETECT + SHORT), powered from **5V_LDO**, O-C → 3V3_GPS
- [ ] ANT_OFF high-side disable **Q13 (BC857W PNP)**: E=V_ANT, C=Q11-B (collector sources into Q11 base to remove antenna bias on ANT_OFF); GPS_ANT_OFF_MON (PD4)
- [ ] **INA228 #4 @0x45 (U26)** antenna-rail monitor; ANT_BIAS_EN (PC9)

## 8. Ethernet / PTP
- [ ] **U11 LAN8742AI-CZ-TR** RMII PHY + **Y1 25 MHz crystal**
- [ ] PoE-capable **magjack** (integrated magnetics, all 4 pairs)
- [ ] Straps: nINTSEL low (REF_CLK-out), PHYAD (PB10/RXER); **LAN_RST_N (PD10)**
- [ ] RMII bus nets; **ETH_PPS_OUT (PB5)**

## 9. PoE input + power tree
- [ ] **U7/U8 FDMQ8205A** bridge rectifier (Mode A+B)
- [ ] **U9 NCP1095** PD controller (classify **Type 2 min / Type 3 margin**)
- [ ] Main buck (**U28 MIC28516**) → **3.3 V main** (**U29 AP3441** → 3V3)
- [ ] **5 V intermediate** buck/rail (feeds 5V_LDO domains, display rail, RF island)
- [ ] **POE_KILL** pass-FET (PE15, default-RUN pulldown)
- [ ] **INA228 #1 @0x40 (U10)** PoE-in; **#2 @0x41 (U31)** STM 3V3; **#8 @0x43 (U30)** main/general 3V3

## 10. Supercap backup
- [ ] **U34/U35 TPS61094 ×2** backup managers (strapped autonomous — EN/MODE tied)
- [ ] **DSF305Q3R0** supercaps (placement ≤ ~75 °C; 2.55 V peak charge)
- [ ] SOC sense: BKP_GPS_SOC_V (PA6), BKP_STM_SOC_V (PB1)

## 11. I²C1 housekeeping bus (PB8/PB9)
- [ ] **U56 LTC4311** rise-time accelerator (**EN tied permanently to 3V3_STM** — always on, nothing to sequence)
- [ ] **U57 TMP117 @0x49** oscillator temp (ADD0→3V3_STM); **U58 TMP117 @0x48** ambient/enclosure (ADD0→GND, fan loop)
- [ ] **U72 SHT45 @0x44** humidity/temp (**owns 0x44** — reason GPS INA228 U23 is at 0x4A)
- [ ] **U60 ATECC608B @0x60** secure element
- [ ] **U61 IIS2MDC @0x1E** magnetometer (continuous GND pour keep-out under it)
- [ ] **U59 LIS2DH12 @0x19** accelerometer (SA0→3V3_STM — committed)
- [ ] **9× INA228** also share this bus (see §5/§9/§12/§13); **all INA ALERTs / PG / buttons are direct GPIO — no I/O expander (§12)**

## 12. Fault / UI aggregation — **direct GPIO** (no I/O expander)
- [ ] **GPIOF scan (1 kHz):** PF0–6 `BUTTON_1..7` (10k↑ +0.1µF), PF7 `DISP_TOUCH_INT`, PF8 `V_ANT_EN_FAULT_N`, PF9 `V_DISP_EN_FAULT_N`, PF10 `PROX_WAKE` (EXTI10), PF11 `ENC_BUTTON`, PF12 `PANEL_LED_FAULT_N`, PF13 `INA_ALERT_5V_PANEL` (INA228 #9 0x4C), PF14 `BKP_STM_PG`, PF15 `BKP_GPS_PG`
- [ ] **GPIOG scan (1 kHz):** PG0–7 PG rails (3V3_GPS_LDO_PG, OCXO_LDO_PG, **3V0_RF_LDO_PG (PG2 — LT3045 U51 PG open-collector, R266 10k→3V3_STM)**, 5V_PSU_PG, 3V3_PSU_PG (PG4, AP3441 PG→VIN, R94/R92 → 2.98 V), OCXO_PSU_PG(÷), RB_PSU_PG, **POE_PG (PG7 — divider R264 174k/R263 10k → 2.93 V)**); PG8–15 INA228 ALERTs (V_POE, 3V3_STM, 5V_DISP, 3V3, 3V3_GPS/0x4A, V_ANT, OCXO, **VCC_RB (PG15 — R265 10k pull-up)**)
- [ ] **Encoder** `ENC_A/ENC_B` (PA8/PA9, TIM1 hardware quadrature) via **U68 74LVC2G17** Schmitt buffer — encoder output is **push-pull (driven), no pull-up needed**; encoder/panel logic Vcc = J17.28 (5 V)
- [ ] **7-button keypad** (PF0–6, ESD arrays U19/U20 at J17)
- [ ] **Magnetic reed** presence sensor = discrete GPIO `PROX_WAKE` PF10 (**J17.18**) — passive dry contact, R254 10k pull-up + close-to-GND (active-low); no Vcc pin needed

## 13. Display / touch (`sts1000_3v3p_i2c_peripherals.md`)
- [ ] **LCDwiki MSP4030 module:** ST7796S driver + FT6336U touch (**0x38**) + onboard 74LVC245 level-shift + BSS138 backlight
- [ ] **U33 RT9742** load switch — gates 5V_DISP only; nFLG→`V_DISP_EN_FAULT_N` (PF9, R105 10k↑); **DISP_EN (PC11) + R104 pull-down**
- [ ] **U63 PCA9306** touch I²C level translator (gated 5 V side; one low-side pull-up pair to 3V3_STM)
- [ ] **U17/U18 TPD4E05U06DQAR** ESD (5 V touch lines); **TPD4E02B04** (3.3 V lines) <!-- TODO verify designators: 3.3 V-line ESD arrays U15/U16/U19/U20 -->
- [ ] **R210/R211 22 Ω** series in display branch only
- [ ] **INA228 #7 @0x42 (U32)** 5V_DISP monitor (0.1 Ω, ADCRANGE=1)
- [ ] Control: CS (PE9), DC (PB0), BL PWM (PE6, wake/sleep), DISP_RST (PA10, R209 10k↓ = held-in-reset default)

## 14. FE-5680A housekeeping / RS-232 (`rb_rs232_interface.md`)
- [ ] **U46 SN65C3221E** RS-232 transceiver + charge-pump caps C139–C147 <!-- TODO verify designators: old C126–C130 -->
- [ ] **K1 G6K-2F-Y** DPDT RS-232/CMOS mode-select relay + **Q21** driver; `RB_RS232_CMOS_SW` (PE4, default low = RS-232, fail-safe)
- [ ] **U48 APC-817C1** opto lock-status input (inverting → FW polarity bit) + D17; `RB_LOCK` (PB13)
- [ ] ESD/TVS: **U47 PESD15VL2BT**, **D13/D14 SMAJ15CA**, D12 BAT54S, D16 3V3 zener; **D26 SMAJ15CA on RB_LOCK_IN** (J6.3 DE-9 lock input)
- [ ] L8/L9 line ferrites; UART7 (`RB_RX` PB4=**NJTRST → SWD-only** / `RB_TX` PE7)
- [ ] **Rb VCC disconnect gate Q25 (SI7469DP)** — `RB_VCC_GATE` PB1 → `VCC_RB_G` to J6; **D25 gate-clamp** (K=VCC_RB, A=gate — reverse-blocks OFF, Zener-clamps |Vgs| to ~12 V); 26 V OV latch U41A → `RB_OV_DET` PE3 / `RB_OV_RESET` PD3
- [ ] Connector pinout per the specific FE-5680A variant (10 MHz is **not** on this connector)

## 15. SPI4 bus (PE11–14)
- [ ] **U62 SPI NOR flash** (CS PE11, NOR_RST_N PE10) — Macronix/Winbond/ISSI, **not GigaDevice**
- [ ] ST7796 TFT (CS PE9, write-only) — see §13
- [ ] **U43** MCP41U83 digipot (CS PD2) — see §5

## 16. Misc / front-panel / housekeeping
- [ ] **USB FS device** connector + ESD (console / SMP recovery); HSI48 + CRS; PE2 VBUS-sense divider
- [ ] **RGB LED** (PD12–PD14, common-anode low-side NPN) — status indication
- [ ] **Fan** + tach (driven off TMP117 #2 loop) — `FAN_PWM` PE5, `FAN_TACH` PA15
- [ ] **Panel-LED string** (6 white + 1 red): **U55 RT9742** 5V load switch → R199 shunt (**INA228 #9 U54 @0x4C**) → FB12 → Q22 (NSS40300 PNP) high-side + Q23 (BC847W) PWM; `PANEL_LED_EN` PC0 (R198↓), `PANEL_LED_PWM` PE0, `PANEL_LED_FAULT_N` PF12
- [ ] **Holdover alarm relay K2** (G6K-2F-Y, Form-C → J16) — `HOLDOVER_ALARM_RELAY` PA6 → Q24 driver + D24 flyback; **normally-energized fail-safe** (de-energized = ALARM)
- [ ] Bench-fanout **SMA** output (`10MHz_RF_OUT`) — see §3

---

### INA228 map (sanity) — 9 monitors, as-built
| # | Addr | Rail | Designator | Shunt | ALERT (U12) |
|---|------|------|------|------|------|
| 1 | 0x40 | PoE input | U10 | R30 150m (in series V_POE feed) | PG8 |
| 2 | 0x41 | STM 3V3 | U31 | R106 15m | PG9 |
| 3 | **0x4A** | GPS VCC | U23 | R72 500m | PG12 |
| 4 | 0x45 | Antenna bias | U26 | R89 100m | PG13 |
| 5 | 0x46 | OCXO | U37 | R126 25m | PG14 |
| 6 | 0x47 | Rb (VCC_RB) | U44 | R159 20m | PG15 (R265 pull-up) |
| 7 | 0x42 | 5V_DISP | U32 | R107 100m | PG10 |
| 8 | 0x43 | main/general 3V3 | U30 | R102 75m | PG11 |
| 9 | **0x4C** | panel-LED 5V | U54 | R199 220m | **PF13** |

Non-INA228 addresses on the same I²C1 bus: 0x44 SHT45 (U72), 0x48/0x49 TMP117 (U58/U57), 0x19 LIS2DH12 (U59), 0x1E IIS2MDC (U61), 0x60 ATECC608B (U60), 0x38 FT6336U touch. **15 devices, no collisions.**
