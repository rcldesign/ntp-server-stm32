# STS1000 "Meridian" — Schematic Implementation Checklist

High-level IC / circuit inventory for verifying schematic completeness. One line per active part or distinct circuit block; key support passives folded in. Designators shown where fixed in the canonical docs; blank where not yet assigned.

> Standing KiCad actions (verify these are done):
> - [ ] Drop legacy **R44** at PC12 (R153 is the `INA_ALERT_INT_N` pull-up; 10k∥10k = 5k otherwise)
> - [ ] Fix net label **`PG_IN_N` → `PG_INT_N`** (one-pin-net otherwise)
> - [ ] Confirm INT nets + `EXP_RESET` land on the correct MCU pins (PA10 / PE0 / PC12 / PC0)
> - [ ] Cross-doc: peripheral-map §2 wording "OCXO → PH0 directly" → "→ PH0 via §2.1 clock mux"

---

## 1. MCU & core
- [ ] **STM32H563VIT6** (LQFP100) — MCU
- [ ] HSE **bypass** on PH0 ← clock-mux `CLK_OUT`; PH1/OSC_OUT = NC
- [ ] **LSE 32.768 kHz** crystal on PC14/PC15 (RTC) + load caps
- [ ] VDD/VDDA decoupling, VREF+, VBAT backup-domain feed
- [ ] Boot/SWD, NRST, WDT_KICK (PB2), PFI (PE8)

## 2. Timing reference — OCXO
- [ ] **OH300-61003CV-010.0M** VC-OCXO (10 MHz, 3.0 V CMOS)
- [ ] OCXO low-noise supply LDO (**TPS7A52** — confirm PN/rail), always-on, dedicated
- [ ] DAC loop filter: PA4 (DAC1_OUT1) → filter → Vc (pin 1), **centered ~1.65 V**, Vc never floating
  - [ ] Vc-path caps **C0G/NP0 or film — never X7R** (microphonic FM)
- [ ] **R98 22 Ω** source termination at OCXO output (→ mux I0)
- [ ] PA3 (ADC INP15) Vc sense — no divider
- [ ] **INA228 #5 @0x46** OCXO-rail monitor

## 3. Clock-source mux (`sts1000_clock_mux.md`)
- [ ] **U45 74LVC1G157GW-Q100H** — 2:1 LVCMOS selector (A=OCXO, B=ext)
- [ ] **U46 74LVC1G34GW-Q100** — bench-fanout buffer
- [ ] **R149 100 k** MUX_SEL (PB6) pull-down → default OCXO
- [ ] **R150 22 Ω** Y→PH0 damp; **R151 22 Ω** fanout source-match
- [ ] **C141 100 nF** fanout DC-block; **D16 SZESD7410** at SMA (off-page)
- [ ] **FB9** ferrite +3.3V→+3.3V_CLK (VDD domain); C140/C142/C143 decoupling

## 4. External-reference RF front end (`ntp_server_rf_frontend.md`)
- [ ] SMA input (`10MHz_RF_IN`)
- [ ] **D11 SZESD7410** ESD at connector
- [ ] **C132 100 nF** DC block (50 V)
- [ ] **R134/R135 47 k** bias node N1 (1.5 V)
- [ ] **U42 TMUX1101** switched 50 Ω termination + **D15 BAT54S** pin-2 clamp (REF_TERM_EN = PC10)
- [ ] **U43 LTC6752xS5** comparator slicer; REF_VMID = R140/R141; R142/R138/C136 input net
- [ ] Protection: **D12**+R137 (N2 clamp), **D13** island clamp, **R143+D14** VCC guard
- [ ] **U44 LT3045** 3.0 V island LDO + **FB8** ferrite
- [ ] **R144 33 Ω** source term at U43 Q (→ mux I1, EXTREF_MON PB14)

## 5. Rb variable supply (`sts1000_vcc_rb_supply.md`)
- [ ] **U31A MIC28516** adaptive-on-time buck (VCC_RB 5–24 V) + **L7 40 µH** + COUT bank (≥63 V X7R)
- [ ] **U35 MCP41U83T-503E/ST** SPI digipot (FB trim; SPI2C→DGND; CS=PD2)
- [ ] **U36 OPA320** wiper buffer; **U34 MCP1502-40** 4.096 V reference
- [ ] FB network: R92/R93 divider + Rctrl R102 + ripple-inj R91/C99 + Cff C101
- [ ] **Fixed bounding resistors** so no wiper code exits Rb-safe envelope
- [ ] **D_TVS 1.5SMCJ28A** at clock connector
- [ ] **U37 INA228 @0x47** VCC_RB monitor + R108 shunt
- [ ] **OV latch:** U38A LMV393 + Q10/Q11/Q12/Q13 + D4 (26 V trip); RB_OV_DET (PE3) / RB_OV_RESET (PD3)
- [ ] EN interlock Q8/Q9 (RB_PWR_EN = PB7 → buck EN)

## 6. GNSS — ZED-F9T
- [ ] **u-blox ZED-F9T-00B** receiver
- [ ] **V_BCKP** feed (stays up through GPS VCC cycles)
- [ ] USART3 (PD8/9) + UART4 RTCM (PD0/1) + TIMEPULSE (PA0) / TIMEPULSE2 (PC6)
- [ ] RESET_N (PD11), SAFEBOOT_N (PD15), DSEL (PD7), EXTINT (PD6), TXRDY (PD5)
- [ ] **GPS VCC switch (PC8)**; **INA228 #3 @0x44** GPS-rail monitor

## 7. GNSS antenna bias + supervisor (`gnss_antenna_bias_supervisor.md`)
- [ ] Bias-T: **L1 56 nH** choke + C26
- [ ] **Q2** PNP pass + foldback **Q1/R29** + **R30 3.3 Ω** sense
- [ ] **U11 INA181A1** current sense (gain 20)
- [ ] **U12 LMV393** dual comparator (DETECT + SHORT), powered from **+5V_LDO**, O-C → 3V3_GPS
- [ ] ANT_OFF driver **Q4 (NPN) + Q5 (PNP)** + R43 default-on; GPS_ANT_OFF_MON (PD4)
- [ ] **INA228 #4 @0x45** antenna-rail monitor; ANT_BIAS_EN (PC9)

## 8. Ethernet / PTP
- [ ] **LAN8742AI-CZ-TR** RMII PHY + **25 MHz crystal**
- [ ] PoE-capable **magjack** (integrated magnetics, all 4 pairs)
- [ ] Straps: nINTSEL low (REF_CLK-out), PHYAD (PB10/RXER); **PHY_nRST (PD10)**
- [ ] RMII bus nets; **ETH_PPS_OUT (PB5)**

## 9. PoE input + power tree
- [ ] **FDMQ8205A** bridge rectifier (Mode A+B)
- [ ] **NCP1095** PD controller (classify **Type 2 min / Type 3 margin**)
- [ ] Main buck → **3.3 V main**
- [ ] **5 V intermediate** buck/rail (feeds +5V_LDO domains, display rail, RF island)
- [ ] **POE_KILL** pass-FET (PE15, default-RUN pulldown)
- [ ] **INA228 #1 @0x40** PoE-in; **#2 @0x41** STM 3V3; **#8 @0x43** main/general 3V3

## 10. Supercap backup
- [ ] **TPS61094 ×2** backup managers (strapped autonomous — EN/MODE tied)
- [ ] **DSF305Q3R0** supercaps (placement ≤ ~75 °C; 2.55 V peak charge)
- [ ] SOC sense: BKP_GPS_SOC_V (PA6), BKP_STM_SOC_V (PB1)

## 11. I²C1 housekeeping bus (PB8/PB9)
- [ ] **LTC4311ISC6** rise-time accelerator (EN = I2C_BUF_EN PA9)
- [ ] **TMP117 #1 @0x48** oscillator temp; **TMP117 #2 @0x49** ambient (fan loop)
- [ ] **ATECC608B @0x60** secure element
- [ ] **IIS2MDC @0x1E** magnetometer (continuous GND pour keep-out under it)
- [ ] **LIS2DH12 @0x18/0x19** accelerometer (**commit SA0**)
- [ ] (INA228 ×8 and MCP23017 ×2 also share this bus — see §5/§9/§12/§13)

## 12. Fault / UI aggregation (`sts1000_fault_aggregation.md`)
- [ ] **U47 MCP23017 @0x20 (MIRROR=1)** — PortA 8×PG, PortB EN-faults/PROX → **PG_INT_N (PA10, R152)**
- [ ] **U48 MCP23017 @0x21 (MIRROR=0)** — PortA 7 buttons + DISP_TOUCH_INT → **BTN_INT_N (PE0, R148)**; PortB 8×INA ALERT → **INA_ALERT_INT_N (PC12, R153)**
- [ ] **EXP_RESET (PC0, R154 pull-down)** shared /RESET (open-drain, active-low INTs)
- [ ] **7-button keypad**
- [ ] **PIR (Panasonic PaPIRs EKMB)** on U47 PortB[3] — confirm orderable PN + output structure

## 13. Display / touch (`sts1000_3v3p_i2c_peripherals.md`)
- [ ] **LCDwiki MSP4030 module:** ST7796S driver + FT6336U touch (**0x38**) + onboard 74LVC245 level-shift + BSS138 backlight
- [ ] **U56 RT9742** load switch — gates V_DISP_5V only; nFLG→U47 GPB2; **DISP_EN (PC11) + R161 pull-down**
- [ ] **U57 PCA9306** touch I²C level translator (gated 5 V side; one low-side pull-up pair to 3V3_STM)
- [ ] **U60 TPD4E05U06DQAR** ESD (5 V touch lines); **U58/U59 TPD4E02B04** (3.3 V lines)
- [ ] **R163/R164 22 Ω** series in display branch only
- [ ] **INA228 #7 @0x42** V_DISP_5V monitor (0.1 Ω, ADCRANGE=1)
- [ ] Control: CS (PE9), DC, BL PWM (wake/sleep), DISP_RST (PA8)

## 14. FE-5680A housekeeping / RS-232 (`rb_rs232_interface.md`)
- [ ] **U40 SN65C3221E** RS-232 transceiver + charge-pump caps C126–C130
- [ ] **K1 G6K-2F-Y DC3** DPDT mode-select relay + **Q14** driver + **D10** flyback; RB_SER_MODE (PE4)
- [ ] **U41 APC-817C1** opto lock-status input + D9 reverse-V; RB_LOCK (PB13)
- [ ] ESD/TVS: **U39 PESD15VL2BT**, **D5/D6 SMAJ15CA**, D7 BAT54S, D8 3V3 zener
- [ ] L9/L10 line ferrites; UART7 (RB_UART_TX PB4 / RX PE7)
- [ ] Connector pinout per the specific FE-5680A variant (10 MHz is **not** on this connector)

## 15. SPI4 bus (PE11–14)
- [ ] **SPI NOR flash** (CS PE11, NOR_RESET PE10) — Macronix/Winbond/ISSI, **not GigaDevice**
- [ ] ST7796 TFT (CS PE9, write-only) — see §13
- [ ] MCP41U83 digipot (CS PD2) — see §5

## 16. Misc / front-panel / housekeeping
- [ ] **USB FS device** connector + ESD (console / SMP recovery); HSI48 + CRS; PE2 VBUS-sense divider
- [ ] **RGB LED ×2** (PD12–PD14) — status indication
- [ ] **Fan** + tach (driven off TMP117 #2 loop)
- [ ] Bench-fanout **SMA** output (`10MHz_RF_OUT`) — see §3

---

### INA228 map (sanity)
| # | Addr | Rail |
|---|------|------|
| 1 | 0x40 | PoE input |
| 2 | 0x41 | STM 3V3 |
| 3 | 0x44 | GPS VCC |
| 4 | 0x45 | Antenna bias |
| 5 | 0x46 | OCXO |
| 6 | 0x47 | Rb (VCC_RB) |
| 7 | 0x42 | V_DISP_5V |
| 8 | 0x43 | main/general 3V3 |
