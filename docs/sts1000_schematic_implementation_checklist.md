# STS1000 "Meridian" — Schematic Implementation Checklist

High-level IC / circuit inventory for verifying schematic completeness. One line per active part or distinct circuit block; key support passives folded in. Designators shown where fixed in the canonical docs; blank where not yet assigned.

> Standing KiCad actions (verify these are done):
> - [ ] Drop legacy **R44** at PC12 (R199 is the `INA_ALERT_INT_N` pull-up; 10k∥10k = 5k otherwise) <!-- TODO verify against re-annotated schematic: R44 is now a 49.9 Ω Ethernet PHY term; no second INA_ALERT_INT_N pull-up remains -->
> - [ ] Fix net label **`PG_IN_N` → `PG_INT_N`** (one-pin-net otherwise) <!-- TODO verify against re-annotated schematic: net is `PG_INT_N` -->
> - [ ] Confirm INT nets + `EXP_RST_N` land on the correct MCU pins (PA10 / PE0 / PC12 / PC0)
> - [ ] Cross-doc: peripheral-map §2 wording "OCXO → PH0 directly" → "→ PH0 via §2.1 clock mux"

---

## 1. MCU & core
- [ ] **U12 STM32H563VIT6** (LQFP100) — MCU
- [ ] HSE **bypass** on PH0 ← clock-mux `CLK_OUT`; PH1/OSC_OUT = NC
- [ ] **LSE Y2 32.768 kHz** crystal on PC14/PC15 (RTC) + load caps
- [ ] VDD/VDDA decoupling, VREF+, VBAT backup-domain feed
- [ ] Boot/SWD, MCU_NRST, WDT_KICK (PB2), PFI (PE8); **U64 TPS3430** window WDT
- [ ] **U13 LT3045** VDDA LDO; **U14 MCP1502-33** 3.3 V STM ref

## 2. Timing reference — OCXO
- [ ] **Y3 OH300-61003CV-010.0M** VC-OCXO (10 MHz, 3.0 V CMOS)
- [ ] OCXO low-noise supply LDO (**U39 TPS7A52**), always-on, dedicated; buck **U38 AP3441**
- [ ] DAC loop filter: PA4 (DAC1_OUT1) → filter → Vc (pin 1), **centered ~1.65 V**, Vc never floating
  - [ ] Vc-path caps **C0G/NP0 or film — never X7R** (microphonic FM)
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
- [ ] **U40A MIC28516** adaptive-on-time buck (VCC_RB 5–24 V) + **L7 40 µH** + COUT bank (≥63 V X7R)
- [ ] **U43 MCP41U83T-503E/ST** SPI digipot (FB trim; SPI2C→DGND; CS=PD2)
- [ ] **U42 OPA320** wiper buffer; **U45 MCP1502-40** 4.096 V reference
- [ ] FB network divider + Rctrl + ripple-inj + Cff <!-- TODO verify designators: R92/R93/R102/R91/C99/C101 (Rb Power sheet re-annotated) -->
- [ ] **Fixed bounding resistors** so no wiper code exits Rb-safe envelope
- [ ] **D_TVS 1.5SMCJ28A** at clock connector
- [ ] **U44 INA228 @0x47** VCC_RB monitor + shunt <!-- TODO verify designator: R108 shunt -->
- [ ] **OV latch:** U41A LMV393 + Q transistors + D (26 V trip); RB_OV_DET (PE3) / RB_OV_RESET (PD3) <!-- TODO verify designators: Q10/Q11/Q12/Q13, D4 -->
- [ ] EN interlock (RB_PWR_EN = PB7 → buck EN) <!-- TODO verify designators: Q8/Q9 -->


## 6. GNSS — ZED-F9T
- [ ] **U21 u-blox ZED-F9T-00B** receiver + **U22 LT3045** GPS LDO
- [ ] **V_BCKP** feed (stays up through GPS VCC cycles)
- [ ] USART3 (PD8/9) + UART4 RTCM (PD0/1) + TIMEPULSE (PA0) / TIMEPULSE2 (PC6)
- [ ] RESET_N (PD11), SAFEBOOT_N (PD15), DSEL (PD7), EXTINT (PD6), TXRDY (PD5)
- [ ] **GPS VCC switch (PC8)**; **INA228 #3 @0x44 (U23)** GPS-rail monitor

## 7. GNSS antenna bias + supervisor (`gnss_antenna_bias_supervisor.md`)
- [ ] Bias-T: **L1 47 nH** choke + C <!-- TODO verify designator: C26 -->
- [ ] **Q11** PNP pass + foldback **Q12** + **R77 3.3 Ω** sense (across U24 IN+) <!-- TODO verify foldback designator: old Q1/R29 -->
- [ ] **U24 INA181A1** current sense (gain 20)
- [ ] **U25 LMV393** dual comparator (DETECT + SHORT), powered from **5V_LDO**, O-C → 3V3_GPS
- [ ] ANT_OFF driver **Q14 (NPN)** + R79 default-on; GPS_ANT_OFF_MON (PD4) <!-- TODO verify PNP designator: old Q5 -->
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
- [ ] **U56 LTC4311ISC6** rise-time accelerator (EN = I2C_BUF_EN PA9)
- [ ] **U57 TMP117 #1 @0x48** oscillator temp; **U58 TMP117 #2 @0x49** ambient (fan loop)
- [ ] **U60 ATECC608B @0x60** secure element
- [ ] **U61 IIS2MDC @0x1E** magnetometer (continuous GND pour keep-out under it)
- [ ] **U59 LIS2DH12 @0x18/0x19** accelerometer (**commit SA0**)
- [ ] (INA228 ×8 and MCP23017 ×2 also share this bus — see §5/§9/§12/§13)

## 12. Fault / UI aggregation (`sts1000_fault_aggregation.md`)
- [ ] **U55 MCP23017 @0x20 (MIRROR=1)** — PortA 8×PG, PortB EN-faults/PROX → **PG_INT_N (PA10, R200)** (INTA+INTB tied)
- [ ] **U54 MCP23017 @0x21 (MIRROR=0)** — PortA 7 buttons + DISP_TOUCH_INT → **BTN_INT_N (PE0, R201)**; PortB 8×INA ALERT → **INA_ALERT_INT_N (PC12, R199)** (INTA/INTB separate)
- [ ] **EXP_RST_N (PC0, R198 pull-down)** shared /RESET (open-drain, active-low INTs)
- [ ] **7-button keypad**
- [ ] **PIR (Panasonic PaPIRs EKMB)** on U55 PortB — confirm orderable PN + output structure <!-- TODO verify designator: PortB[3] mapping -->

## 13. Display / touch (`sts1000_3v3p_i2c_peripherals.md`)
- [ ] **LCDwiki MSP4030 module:** ST7796S driver + FT6336U touch (**0x38**) + onboard 74LVC245 level-shift + BSS138 backlight
- [ ] **U33 RT9742** load switch — gates 5V_DISP only; nFLG→U55 GPB1 (V_DISP_EN_FAULT); **DISP_EN (PC11) + R104 pull-down**
- [ ] **U63 PCA9306** touch I²C level translator (gated 5 V side; one low-side pull-up pair to 3V3_STM)
- [ ] **U17/U18 TPD4E05U06DQAR** ESD (5 V touch lines); **TPD4E02B04** (3.3 V lines) <!-- TODO verify designators: 3.3 V-line ESD arrays U15/U16/U19/U20 -->
- [ ] **R210/R211 22 Ω** series in display branch only
- [ ] **INA228 #7 @0x42 (U32)** 5V_DISP monitor (0.1 Ω, ADCRANGE=1)
- [ ] Control: CS (PE9), DC, BL PWM (wake/sleep), DISP_RST (PA8)

## 14. FE-5680A housekeeping / RS-232 (`rb_rs232_interface.md`)
- [ ] **U46 SN65C3221E** RS-232 transceiver + charge-pump caps C139–C147 <!-- TODO verify designators: old C126–C130 -->
- [ ] **K1 G6K-2F-Y DC3** DPDT mode-select relay + **Q21** driver + **D15** flyback; RB_SER_MODE (PE4) <!-- TODO verify designators: old Q14 driver, D10 flyback -->
- [ ] **U48 APC-817C1** opto lock-status input + D17 reverse-V; RB_LOCK (PB13) <!-- TODO verify designator: old D9 -->
- [ ] ESD/TVS: **U47 PESD15VL2BT**, **D13/D14 SMAJ15CA**, D12 BAT54S, D16 3V3 zener
- [ ] L8/L9 line ferrites; UART7 (RB_UART_TX PB4 / RX PE7)
- [ ] Connector pinout per the specific FE-5680A variant (10 MHz is **not** on this connector)

## 15. SPI4 bus (PE11–14)
- [ ] **U62 SPI NOR flash** (CS PE11, NOR_RST_N PE10) — Macronix/Winbond/ISSI, **not GigaDevice**
- [ ] ST7796 TFT (CS PE9, write-only) — see §13
- [ ] **U43** MCP41U83 digipot (CS PD2) — see §5

## 16. Misc / front-panel / housekeeping
- [ ] **USB FS device** connector + ESD (console / SMP recovery); HSI48 + CRS; PE2 VBUS-sense divider
- [ ] **RGB LED ×2** (PD12–PD14) — status indication
- [ ] **Fan** + tach (driven off TMP117 #2 loop)
- [ ] Bench-fanout **SMA** output (`10MHz_RF_OUT`) — see §3

---

### INA228 map (sanity)
| # | Addr | Rail | Designator |
|---|------|------|------|
| 1 | 0x40 | PoE input | U10 |
| 2 | 0x41 | STM 3V3 | U31 |
| 3 | 0x44 | GPS VCC | U23 |
| 4 | 0x45 | Antenna bias | U26 |
| 5 | 0x46 | OCXO | U37 |
| 6 | 0x47 | Rb (VCC_RB) | U44 |
| 7 | 0x42 | 5V_DISP | U32 |
| 8 | 0x43 | main/general 3V3 | U30 |
