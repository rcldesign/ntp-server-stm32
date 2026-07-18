# STS1000 "Meridian" — Firmware ↔ Hardware Interface Reference

**Scope.** The operational contract a firmware engineer uses to drive every hardware signal
on the STS1000 correctly: the complete 144-pin map of **U12 (STM32H563ZIT6, LQFP144)**,
bring-up sequencing, the timing engine I/O, the single-bus I²C device map, the GPIO scan /
interrupt model, per-subsystem monitoring loops, alarms/fail-safe, and the firmware-critical
cautions.

**Authority.** Pin↔net facts are taken from the as-built re-annotated netlist. All aggregated
inputs (buttons, load-switch fault flags, rail power-goods, INA228 ALERTs) are **direct GPIO**
on Ports F/G — there are no MCP23017 expanders. This document is authoritative for the
firmware↔hardware pin contract; behavioral intent (loop rates, state machines, refids) is from
the software spec. Reference parts by **designator + pin**.

**Conventions.** `_N` suffix = active-low. "Default/reset state" = the state the pin sits in
at MCU power-on reset (all GPIO are Hi-Z inputs at reset; the listed state is what the external
pull/circuit forces). MUST/SHOULD/MAY are prescriptive.

## Hardware notes & open items

These items are tracked for the next board spin or bench verification; none blocks writing
firmware against this contract:

- **`3V0_RF_LDO_PG` (PG2).** The LT3045 (U51) PWRGD is open-collector and needs a 10 kΩ pull-up
  to 3V3_STM (mirror R215 on the GPS LDO). Until fitted, firmware **masks PG2** and derives the
  3V0_RF LDO health from its INA/rail telemetry.
- **J17 panel power.** The panel harness carries display-5 V (**J17.16 → 5V_DISP**) and logic-3 V3
  (**J17.28 → 3V3_STM**) to the display, cap-touch, and rotary encoder. Confirm both supply pins
  land on their rails before relying on panel bring-up (Stage 6). The panel-LED string has its own
  Q22-switched anode and is independent.
- **VREF+ decoupling (C37).** Grow C37 to 100 nF at U12.32 for ADC ENOB on the OCXO steering path
  (a 2.2 µF bulk C38 sits behind R48 50 Ω).
- **Rb-buck output caps C125–C128.** Rate ≥ 50 V; they sit on the 24.45 V VCC_RB rail.
- **Bench verify:** AP3441 PG active-drive to VIN (the OCXO LDO enable depends on the PG-divider
  nodes reaching ~2.98 V when good — node N ≈ 2.98 V / PG5 ≈ 2.66 V, both loaded, both > VIH; DS39754
  specs no PG drive-Z); ZED-F9T V_BCKP current at enclosure Tmax; FE-5680A J6.8/J6.9 Tx/Rx direction
  per surplus variant.

## Firmware requirements

- **SWD-only board.** `RB_RX` uses PB4/NJTRST, so JTAG is unavailable. Debug via SWD
  (PA13/PA14/PB3); do not enable the JTAG pins.
- **`GPS_TXRDY` re-`CFG`.** F9T pin 19 defaults to GEOFENCE_STAT; firmware MUST send CFG-TXREADY to
  route TX_READY onto pin 19 before trusting PD5.
- **`NOR_RST_N` (PE10).** Boots asserted (R205 10k↓); drive HIGH early in bring-up or the SPI-NOR
  stays held in reset.
- **Digipot safe code.** MCP41U83 code 0x000 = Terminal B = safe-low VCTRL (VOUT min ~4.5 V); POR =
  midscale (~14.5 V, inside the 26 V OV envelope). Pre-program the NV wiper safe-low and verify
  `VCC_RB` on INA228 0x47 before `RB_PWR_EN` / `RB_VCC_GATE`.

---

## 1. Pin map — all 144 pins of U12 (as-built)

Pin↔net is verbatim from `u12_pinmap.txt`. AF column: confirmed where load-bearing (RMII=AF11
confirmed via PHY verification); "GPIO" = plain digital I/O; analog pins marked. Peripheral
assignments follow the STM32H563 AF table.

### 1.1 Signal pins

| Pin | Net | Dir | Peripheral / AF | Function | Default / reset state | Active pol. | Notes |
|----:|-----|-----|-----------------|----------|-----------------------|-------------|-------|
| 1 | USB_VBUS_SENSE | AIN | ADC (PE2) | USB-C VBUS present (÷ divider, k≈0.547) | follows VBUS | high=present | **Polled**, not EXTI (line shares w/ PC2). ≈2.87 V @ 5.25 V. |
| 2 | RB_OV_DET | IN | GPIO (PE3) | Rb 26 V OV-latch tripped indicator | low (latch idle) | high=OV tripped | =3.24 V when tripped (÷ from 5 V). **Polled.** |
| 3 | RB_RS232_CMOS_SW | OUT | GPIO (PE4) | Rb serial DPDT relay mode select | **low = RS-232** (R169 pd) | high=CMOS | De-energized relay = RS-232 (fail-safe). |
| 4 | FAN_PWM | OUT | TIM PWM (PE5) | Fan speed PWM, 25 kHz | Hi-Z → **max fan** | duty | **Fail-safe: undriven = full speed.** |
| 5 | DISP_BL | OUT | TIM PWM (PE6) | Display backlight PWM | Hi-Z (off) | duty | RT9742-independent; module backlight. |
| 6 | STM_VBAT | PWR | — | RTC/backup domain supply | supercap-backed | — | VBAT well; not GPIO. |
| 7 | RTC_TAMPER | IN | RTC TAMP (PC13) | Tamper input (J10) | 1 nF only, no pull | — | Add pull if used; no ext pull as-built. |
| 8 | Net-(…OSC32_IN) | AIN | LSE (PC14) | 32.768 kHz RTC crystal in | — | — | LSE osc. |
| 9 | Net-(…OSC32_OUT) | AIN | LSE (PC15) | 32.768 kHz RTC crystal out | — | — | LSE osc. |
| 10 | BUTTON_1 | IN | GPIO (PF0) | Keypad button 1 | high (R190 10k↑ +0.1µF) | **low**=pressed | Scanned (GPIOF). |
| 11 | BUTTON_2 | IN | GPIO (PF1) | Keypad button 2 | high (R191↑) | low=pressed | Scanned. |
| 12 | BUTTON_3 | IN | GPIO (PF2) | Keypad button 3 | high (R192↑) | low=pressed | Scanned. |
| 13 | BUTTON_4 | IN | GPIO (PF3) | Keypad button 4 | high (R193↑) | low=pressed | Scanned. |
| 14 | BUTTON_5 | IN | GPIO (PF4) | Keypad button 5 | high (R194↑) | low=pressed | Scanned. |
| 15 | BUTTON_6 | IN | GPIO (PF5) | Keypad button 6 | high (R195↑) | low=pressed | Scanned. |
| 18 | BUTTON_7 | IN | GPIO (PF6) | Keypad button 7 | high (R196↑) | low=pressed | Scanned. |
| 19 | DISP_TOUCH_INT | IN | GPIO/EXTI7 (PF7) | Cap-touch controller INT | high (module PU) | low=touch | May use EXTI7 or scan; no debounce cap. |
| 20 | V_ANT_EN_FAULT | IN | GPIO (PF8) | Antenna-bias RT9742 nFLG | high (R200 10k↑) | **low**=fault | Open-drain fault. Scanned. |
| 21 | V_DISP_EN_FAULT | IN | GPIO (PF9) | Display-5V RT9742 nFLG | high (R105 10k↑) | **low**=fault | Open-drain fault. Scanned. |
| 22 | PROX_WAKE | IN | GPIO/EXTI10 (PF10) | Presence/proximity wake — passive magnetic **reed** switch (J17.18) | high (R254 10k↑) | **low**=magnet present | **EXTI wake** + scan. Dry reed contact (no Vcc); pull-up + close-to-GND is correct → active-low on magnet. |
| 23 | CLK_OUT | AIN | HSE bypass (PH0) | Disciplined 10 MHz into HSE (from clock mux) | — | — | System reference clock input. R187 22Ω damper. |
| 24 | (unconnected) | — | PH1 | — | Hi-Z | — | Not bonded/used. |
| 25 | MCU_NRST | I/O | NRST | Reset (J3.5 SWD) | internal PU | low=reset | No ext cap (SHOULD add 100 nF). |
| 26 | PANEL_LED_EN | OUT | GPIO (PC0) | Panel-LED 5V RT9742 enable | **low = off** (R198 10k↓) | high=on | Default OFF. |
| 27 | RMII_MDC | OUT | ETH AF11 (PC1) | RMII management clock | Hi-Z | — | PHY addr 0. |
| 28 | POE_NCM | IN | GPIO/EXTI2 (PC2) | NCP1095 status (class MSB) | pull-up | high=idle | Open-drain/RTN=GND/+72 V. **Enable internal pull-up** (no external fitted); EXTI2. |
| 29 | POE_LCF | IN | GPIO/EXTI3 (PC3) | NCP1095 long-class finger | pull-up | high=idle | Open-drain/RTN=GND. **Enable internal pull-up**; EXTI3. |
| 34 | GPS_PPS | IN | **TIM2_CH1** (PA0) | GNSS TIMEPULSE (1 PPS) | driven by F9T | rising | Primary PPS capture. |
| 35 | RMII_REF_CLK | IN | ETH AF11 (PA1) | 50 MHz RMII ref clock (from PHY REFCLKO) | driven by PHY | — | REFCLKO mode (nINTSEL=0). |
| 36 | RMII_MDIO | I/O | ETH AF11 (PA2) | RMII management data | R36 1.5k↑ | — | Link status via MDIO polling (no PHY IRQ). |
| 37 | OCXO_V | AIN | **ADC (PA3)** | OCXO Vc sense | ≈1.65 V | — | Vc read-back for the discipline loop. |
| 40 | OCXO_VC | AOUT | **DAC1_OUT1 (PA4)** | OCXO steering (control voltage) | ≈1.65 V (R121 10M holds center) | — | Written **only** by `discipline`. Never floats. |
| 41 | RMII_TX_EN | OUT | ETH AF11 (PA5) | RMII transmit enable | Hi-Z | high | — |
| 42 | HOLDOVER_ALARM_RELAY | OUT | GPIO (PA6) | Holdover alarm relay K2 drive | **low = de-energized = ALARM** (R259 10k↓) | **high=healthy** | Normally-energized fail-safe. Drive HIGH only when healthy. |
| 43 | RMII_CRS_DV | IN | ETH AF11 (PA7) | RMII carrier-sense/data-valid | driven by PHY | — | Also PHY MODE2 strap (PHY-side). |
| 44 | RMII_RXD0 | IN | ETH AF11 (PC4) | RMII RX data 0 | driven by PHY | — | — |
| 45 | RMII_RXD1 | IN | ETH AF11 (PC5) | RMII RX data 1 | driven by PHY | — | — |
| 46 | DISP_DC | OUT | GPIO (PB0) | Display data/command | Hi-Z | — | ST7796 D/C. |
| 47 | RB_VCC_GATE | OUT | GPIO (PB1) | Rb VCC disconnect gate (Q25) drive | **low = disconnected** (R262 pd) | high=connect | Governs `VCC_RB_G` to J6. |
| 48 | WDT_KICK | OUT | GPIO (PB2) | External windowed WDT refresh | Hi-Z (no pull) | edge/pulse | Kick **only** when all liveness passes. |
| 49 | ENC_BUTTON | IN | GPIO (PF11) | Rotary-encoder push switch | high (R236 10k↑ +C187) | **low**=pressed | Scanned. |
| 50 | PANEL_LED_FAULT | IN | GPIO (PF12) | Panel-LED RT9742 nFLG | high (R201 10k↑) | **low**=fault | Open-drain. Scanned. |
| 53 | INA_ALERT_5V_PANEL | IN | GPIO (PF13) | INA228 U54 (panel 5V, 0x4C) ALERT | high (R220-series↑) | **low**=alert | On **GPIOF**, not GPIOG. Scanned. |
| 54 | BKP_STM_PG | IN | GPIO (PF14) | STM backup-supply (TPS61094) power-good | driven | high=good | U67 comparator. Scanned. |
| 55 | BKP_GPS_PG | IN | GPIO (PF15) | GPS backup-supply (V_BCKP) power-good | driven | high=good | U67. Scanned. Watches ephemeris-hold rail. |
| 56 | 3V3_GPS_LDO_PG | IN | GPIO (PG0) | GPS LT3045 U22 power-good | high (R215 10k↑) | high=good | Scanned (GPIOG). |
| 57 | OCXO_LDO_PG | IN | GPIO (PG1) | OCXO LDO U39 power-good | driven | high=good | Scanned. |
| 58 | RB_TX | IN | UART7_RX (PE7) | FE-5680A serial TX → MCU RX | D12 BAT54S clamp | — | Rb telemetry in. |
| 59 | PFI | IN | GPIO/**EXTI8** (PE8) | Power-fail early warning (U2A, 38.1 V trip) | — | falling=fail | ISR persists state, parks DAC. Ahead of PVD. |
| 60 | DISP_CS | OUT | GPIO (PE9) | Display SPI chip-select | Hi-Z (deselect) | **low**=selected | SPI4 shared bus, distinct CS. |
| 63 | NOR_RST_N | OUT | GPIO (PE10) | SPI-NOR U62 reset | **low = RESET asserted** (R205 10k↓) | low=reset | **MUST drive HIGH early in bring-up** or NOR stays held in reset. |
| 64 | SPI_NSS | OUT | GPIO (PE11) | **SPI-NOR U62 chip-select** | high (R212 10k↑, deselect) | **low**=selected | Despite name, this is NOR CS on SPI4. |
| 65 | SPI_SCK | OUT | SPI4 AF5 (PE12) | SPI4 clock (NOR + digipot + display) | Hi-Z | — | Shared bus. |
| 66 | SPI_MISO | IN | SPI4 AF5 (PE13) | SPI4 MISO | Hi-Z | — | Shared bus. |
| 67 | SPI_MOSI | OUT | SPI4 AF5 (PE14) | SPI4 MOSI (22Ω series to display) | Hi-Z | — | Shared bus. |
| 68 | POE_KILL | OUT | GPIO (PE15) | Board cold-cycle (opens input FET) | **low = RUN** (default) | high=kill | Also HW-OR'd by WDT/thermal/latched faults. Q3 sustain latch holds the kill. |
| 69 | RMII_RXER | IN | ETH AF11 (PB10) | RMII receive error | driven by PHY | — | Also PHY PHYAD0 strap (PHY-side). Keep Hi-Z through reset. |
| 73 | RMII_TXD0 | OUT | ETH AF11 (PB12) | RMII TX data 0 | Hi-Z | — | — |
| 74 | RB_LOCK | IN | GPIO/EXTI13 (PB13) | Rb lock-good (opto U48, inverting) | high (R173 22k↑) | **FW polarity bit** | Opto inverts; polarity is a firmware config bit. De-assert → immediate revert to OCXO. |
| 75 | 10MHz_CLK_OUT | I/O | **TIM12_CH1** (PB14) | EXTREF_MON — measure external ~10 MHz | driven | — | Net named CLK_OUT; firmware role = frequency monitor (input capture) for the Rb/ext-ref band-check. |
| 76 | RMII_TXD1 | OUT | ETH AF11 (PB15) | RMII TX data 1 | Hi-Z | — | — |
| 77 | GPS_RX | OUT | **USART3_TX** (PD8) | MCU→GPS (F9T RXD) | Hi-Z | — | Net named from GPS side: PD8 drives F9T RX. |
| 78 | GPS_TX | IN | **USART3_RX** (PD9) | GPS→MCU (F9T TXD) | driven by F9T | — | Primary UBX/NMEA. |
| 79 | LAN_RST_N | OUT | GPIO (PD10) | LAN8742 PHY reset | **low = held in reset** (R35 10k↓) | low=reset | **Drive HIGH after rails + 25 MHz stable**, width ≥ DS min. |
| 80 | GPS_RST_N | OUT | GPIO (PD11) | F9T RESET_N | high (F9T internal PU) | low=reset | Deasserted by default. |
| 81 | LED_R | OUT | TIM4 PWM (PD12) | Status RGB — red | low (R57 base pd) | high=on | Common-anode, low-side NPN. |
| 82 | LED_G | OUT | TIM4 PWM (PD13) | Status RGB — green | low (base pd) | high=on | Green = locked stratum-1. |
| 85 | LED_B | OUT | TIM4 PWM (PD14) | Status RGB — blue | low (base pd) | high=on | Blue pulse = activity/identify. |
| 86 | GPS_SAFEBOOT_N | OUT | GPIO (PD15) | F9T SAFEBOOT_N | high (F9T PU) | low=safeboot | FW-recovery path w/ RESET. |
| 87 | 3V0_RF_LDO_PG | IN | GPIO (PG2) | Ext-ref 3V0 RF LDO U51 power-good | driven | high=good | LT3045 PWRGD open-collector; **needs a 10k→3V3_STM pull-up** (open item) — mask PG2 until fitted. |
| 88 | 5V_PSU_PG | IN | GPIO (PG3) | 5 V buck power-good | high (R95↑ to 5V) | high=good | PG3 is 5 V-tolerant (FT) — pulled to 5 V. Scanned. |
| 89 | 3V3_PSU_PG | IN | GPIO (PG4) | 3V3 (AP3441) power-good | high ≈2.98 V when good | high=good | AP3441 PG drives to VIN(5V); R94/R92 divide → ~2.98 V. Usable power-good input. |
| 90 | Net-(U12B-PG5) | AIN/IN | GPIO (PG5) | OCXO PSU/EN monitor (÷ from OCXO_PSU_PG) | ≈0.89·OCXO_PSU_PG | high=good | Not spare. **Verify OCXO_PSU_PG ≤3.3 V** — PG5 not 5V-tolerant. |
| 91 | RB_PSU_PG | IN | GPIO (PG6) | Rb buck U40 power-good | driven | high=good | Scanned. |
| 92 | POE_PG | IN | GPIO (PG7) | PoE (NCP1095 PGO) power-good | high ≈2.93 V when good | high=good | R264 174k/R263 10k divider → 2.93 V (safe on a 5 V-tolerant pin). Usable power-good input. |
| 93 | INA_ALERT_V_POE | IN | GPIO (PG8) | INA228 U10 (PoE 0x40) ALERT | high (R221↑) | **low**=alert | Scanned. U10 reads real current (R30 in the series feed). |
| 96 | GPS_TP2 | IN | **TIM3_CH1** (PC6) | GNSS TIMEPULSE2 | driven by F9T | rising | Secondary PPS; cross-check vs PA0. |
| 97 | POE_NCL | IN | GPIO/EXTI7 (PC7) | NCP1095 status (class LSB) | pull-up | high=idle | Open-drain/RTN=GND. **Enable internal pull-up** (unless 10k ext fitted); EXTI7. |
| 98 | GPS_PWR_EN | OUT | GPIO (PC8) | GPS LT3045 U22 enable | **low = off** (R69 100k↓) | high=on | Default OFF; enable in bring-up. |
| 99 | ANT_BIAS_EN | OUT | GPIO (PC9) | Antenna-bias RT9742 U27 enable | **low = off** (R88 10k↓) | high=on | Default OFF. Hard-cut for short protection. |
| 100 | ENC_A | IN | **TIM1_CH1** (PA8) | Encoder quadrature A | via U68 Schmitt | — | Hardware encoder mode; **not scanned**. |
| 101 | ENC_B | IN | **TIM1_CH2** (PA9) | Encoder quadrature B | via U68 Schmitt | — | Hardware encoder mode. |
| 102 | DISP_RST | OUT | GPIO (PA10) | Display ST7796 reset | **low = held in reset** (R209 10k↓) | low=reset | Drive HIGH to release display. |
| 103 | USB_DM | I/O | USB FS (PA11) | USB-C D− | — | — | CDC-ACM console. |
| 104 | USB_DP | I/O | USB FS (PA12) | USB-C D+ | — | — | Console. |
| 105 | SWDIO | I/O | SWD (PA13) | Debug data | SWD | — | **SWD-only board.** |
| 109 | SWCLK | I/O | SWD (PA14) | Debug clock | SWD | — | — |
| 110 | FAN_TACH | IN | GPIO/**EXTI15** (PA15) | Fan tachometer | no pull (use internal PU) | edge | Edge-count over 1 s → RPM. |
| 111 | REF_TERM_EN | OUT | GPIO (PC10) | Ext-ref 50 Ω termination select | **high = terminated** (R176 100k↑) | high=term | Default terminated (safe). |
| 112 | DISP_EN | OUT | GPIO (PC11) | Display 5V RT9742 U33 enable + PCA9306 EN | **low = off** (R104 10k↓) | high=on | Default OFF; deferrable under PoE pressure. |
| 113 | WDT_EN | OUT | GPIO (PC12) | External WDT (TPS3430) enable | **low = WDT disabled** (R213 100k↓) | high=enable | Boot-disabled; FW enables after init. |
| 114 | GPS_TXD2 | OUT | UART4_TX (PD0) | F9T RXD2 (2nd port) | Hi-Z | — | Secondary/RTCM path. |
| 115 | GPS_RXD2 | IN | UART4_RX (PD1) | F9T TXD2 (2nd port) | driven | — | — |
| 116 | SPI_DPOT_CS | OUT | GPIO (PD2) | Rb digipot U43 chip-select | high (R135 10k↑, deselect) | **low**=selected | SPI4 shared bus. |
| 117 | RB_OV_RESET | OUT | GPIO (PD3) | Rb OV-latch reset | **low** (R146 pd) | pulse **high**=clear | Default low; pulse HIGH clears latch. |
| 118 | GPS_ANT_OFF_MON | IN | GPIO/EXTI4 (PD4) | F9T ANT_OFF (LNA-disable) observe | driven by F9T | high=LNA off | Corroborates antenna supervisor. |
| 119 | GPS_TXRDY | IN | GPIO/EXTI5 (PD5) | F9T TX-ready | driven | — | F9T pin-19 default fn = GEOFENCE_STAT; re-`CFG` (CFG-TXREADY) to TX_READY before trusting. |
| 122 | GPS_EXTINT | OUT | GPIO (PD6) | F9T EXTINT (time-mark/aiding trigger) | — | — | MCU→F9T. |
| 123 | GPS_DSEL | OUT | GPIO (PD7) | F9T D_SEL (interface select) | high (F9T PU) | — | — |
| 124 | INA_ALERT_3V3_STM | IN | GPIO (PG9) | INA228 U31 (0x41) ALERT | high (↑) | **low**=alert | Scanned. |
| 125 | INA_ALERT_5V_DISP | IN | GPIO (PG10) | INA228 U32 (0x42) ALERT | high (↑) | **low**=alert | Scanned. |
| 126 | INA_ALERT_3V3 | IN | GPIO (PG11) | INA228 U30 (0x43) ALERT | high (↑) | **low**=alert | Scanned. |
| 127 | INA_ALERT_3V3_GPS | IN | GPIO (PG12) | INA228 U23 (**0x4A**) ALERT | high (↑) | **low**=alert | Scanned. |
| 128 | INA_ALERT_V_ANT | IN | GPIO (PG13) | INA228 U26 (0x45) ALERT | high (↑) | **low**=alert | Scanned. |
| 129 | INA_ALERT_OCXO | IN | GPIO (PG14) | INA228 U37 (0x46) ALERT | high (↑) | **low**=alert | Scanned. |
| 132 | INA_ALERT_VCC_RB | IN | GPIO (PG15) | INA228 U44 (0x47) ALERT | high (R265 10k↑) | **low**=alert | Scanned. |
| 133 | SWO | I/O | TRACE (PB3) | SWO trace | SWD | — | Bring-up trace/log. |
| 134 | RB_RX | OUT | UART7_TX (PB4/NJTRST) | MCU→FE-5680A serial | R168 fault-limit | — | PB4 = NJTRST → SWD-only board (JTAG unavailable). |
| 135 | ETH_PPS_OUT | OUT | ETH PTP (PB5) | MAC IEEE-1588 PPS output | Hi-Z | rising | Buffered by U71 → 1PPS_OUT (J15). |
| 136 | MUX_SEL | OUT | GPIO (PB6) | Clock-mux select (74LVC1G157) | **low = OCXO** (R188 100k↓) | 0=OCXO,1=Rb | Driven only by reference state machine w/ glitchless sequence. |
| 137 | RB_PWR_EN | OUT | GPIO (PB7) | Rb rail buck (U40) enable | **low = off** (R164 100k↓) | high=on | **Enable ONLY after safe digipot code + INA228 0x47 verify.** |
| 139 | I2C_SCL | OD | **I2C1 AF4** (PB8) | I²C1 clock | high (R202 4.7k↑) | — | 400 kHz, LTC4311 accelerated. |
| 140 | I2C_SDA | OD | **I2C1 AF4** (PB9) | I²C1 data | high (R203 4.7k↑) | — | Shared 15-device bus. |
| 141 | PANEL_LED_PWM | OUT | TIM PWM (PE0) | Panel-LED brightness PWM | **low = off** (Q23 base pd) | duty | Hi-Z reset = double default-OFF. Window ≫ PWM period (§4). |

### 1.2 Power / ground / analog-reference pins

| Pins | Net | Role |
|------|-----|------|
| 17, 30, 39, 52, 62, 72, 84, 95, 131, 144 | 3V3_STM | VDD core/IO (always-on housekeeping rail) |
| 121 | 3V3_STM | VDDIO2 |
| 106 | 3V3_STM_USB | VDDUSB (C26/C28) |
| 33 | VDDA | Analog supply — **dedicated LT3045 U13**, islanded for OCXO loop |
| 32 | VREF_3V3 | ADC/DAC reference (MCP1502-33 U14). **SHOULD add 100 nF at pin** (feeds OCXO DAC/ADC). |
| 6 | STM_VBAT | Backup domain (supercap-backed) |
| 70, 142 | VCAP / VCAP__1 | Core LDO caps (2.2 µF each) — H5 LDO mode |
| 16, 31(VSSA), 38, 51, 61, 71, 83, 94, 107, 120, 130, 143 | GND | Grounds |
| 138 | BOOT0 (R50 10k↓) | **Boot from user flash** (single pull-down; no divider defect). |

---

## 2. Power-up / bring-up order of operations

Fail-safe principle: every gated load and every held-in-reset device defaults to the **safe/off**
state by external pull; firmware releases them deliberately and in order. **Never** enable a load
before its rail is verified. **Never** enable Rb before the digipot code is proven safe.

**Stage 0 — hardware autonomous (no MCU).** PoE negotiates (Class 6/Type 3), FDMQ8205A bridge →
NCP1095 → bucks bring up **3V3_STM** (always-on housekeeping) and 5 V. Supercap managers
(TPS61094 U34/U35, strapped autonomous) charge C90/C91 to 2.5 V. OCXO rail (U38/U39) energizes;
OCXO begins warming. The Rb OV latch and PoE-kill sustain circuits arm autonomously.

**Stage 1 — MCU reset exit.** Boot from user flash (BOOT0 pull-down) → STiRoT → MCUboot → app.
At this instant **all GPIO are Hi-Z**; the external pulls in §1 hold every gated load OFF, every
resettable device IN RESET, MUX_SEL=OCXO, RB_PWR_EN=off, HOLDOVER_ALARM_RELAY=de-energized (ALARM).

**Stage 2 — release held-in-reset digital devices (early).**
1. **`NOR_RST_N` (PE10) → HIGH** immediately (R205 defaults it asserted). NOR is needed for
   NVS/config/calibration; nothing else can proceed correctly until it is out of reset.
2. Bring up **I²C1** (PB8/PB9); the bus and LTC4311 are always powered. Probe the 15-device map
   (§4). Read config/calibration from NOR.
3. **`DISP_RST` (PA10) → HIGH** to release the display controller when UI is wanted (rail still off).

**Stage 3 — verify main rails via telemetry + PG lines.**
- Confirm 3V3 via **INA228 0x43** (corroborated by PG4, ~2.98 V), 3V3_STM via 0x41.
- Read **POE_PG/PG7** (~2.93 V divided) and **INA228 0x40** (current + voltage both valid).
- Read the usable PG lines (PG0/1/3/4/6/7, PF14/PF15) and INA228 rails for a health baseline.
  **Mask PG2** until the `3V0_RF_LDO_PG` pull-up is fitted.

**Stage 4 — bring up PHY (network).**
4. After rails stable and the 25 MHz PHY clock is up, **`LAN_RST_N` (PD10) → HIGH** (reset width
   ≥ datasheet min). Then MDIO-poll PHY addr 0 for link.

**Stage 5 — GNSS + antenna.**
5. **`GPS_PWR_EN` (PC8) → HIGH**; wait for 3V3_GPS good (PG0 / INA228 0x4A).
6. Release/pulse **`GPS_RST_N` (PD11)** as needed; F9T boots.
7. **`ANT_BIAS_EN` (PC9) → HIGH**; watch antenna supervisor (§7): DETECT, SHORT_N, INA228 0x45.
8. **Re-`CFG` the F9T** to route `TX_READY` onto pin 19, and configure TIMEPULSE/TIMEPULSE2.

**Stage 6 — display / panel (deferrable).** Requires the J17 panel-power pins (J17.16 → 5V_DISP,
J17.28 → 3V3_STM) to feed the display, touch, and encoder. The panel-LED string is independent (its
own Q22-switched anode).
9. **`DISP_EN` (PC11) → HIGH** (5 V display rail + PCA9306) only when UI is needed; first item to
   shed under PoE pressure.
10. **`PANEL_LED_EN` (PC0) → HIGH**, then drive **`PANEL_LED_PWM` (PE0)**.

**Stage 7 — OCXO discipline.** Once OCXO is warm, start the `discipline` loop (§3). MUX_SEL stays
OCXO. Bring status RGB/relay to reflect real state.

**Stage 8 — Rb (last, guarded).** Only when PoE budget covers it AND OCXO is warm AND supercaps
charged:
11. Write a **known safe-low digipot code** to U43 over SPI4 (code 0x000 = Terminal B = safe-low).
12. **`RB_PWR_EN` (PB7) → HIGH** → soft-start.
13. **Verify `VCC_RB` on INA228 0x47** is inside the safe window (24.45 V − 6.645·VCTRL envelope)
    **before trusting the rail**.
14. **`RB_VCC_GATE` (PB1) → HIGH** to connect `VCC_RB_G` to the FE-5680A.
15. Wait for `RB_LOCK` (PB13, polarity bit) + `EXTREF_MON` (PB14/TIM12) in-band before allowing
    the reference state machine to consider Rb.
> **Stage-8 caveats.** Rate the Rb-buck output caps C125–C128 ≥ 50 V (they sit on the 24 V VCC_RB
> rail). For a 15 V-class FE, set the digipot bound / OV per the unit — the 24.45 V full-scale
> pedestal over-volts a 15 V unit and the OV latch only trips at 26 V. Confirm FE J6-8/J6-9 Tx/Rx
> direction per surplus variant before trusting the serial link.

**Stage 9 — arm watchdog & alarms.**
16. Start the supervisor; once timing/network/housekeeping liveness all pass, **`WDT_EN` (PC12) →
    HIGH** and begin the windowed **`WDT_KICK` (PB2)** cadence (§7).
17. Drive **`HOLDOVER_ALARM_RELAY` (PA6) → HIGH** (energize = healthy) once service quality is met.

---

## 3. Timing engine I/O

| Signal | Pin | Peripheral | Role |
|--------|-----|-----------|------|
| OCXO steer | PA4 | DAC1_OUT1 | Control voltage `OCXO_VC`; 0–3.3 V scaled to OCXO ±0.4 ppm pull, **center 1.65 V**. Slew-limited. |
| OCXO Vc sense | PA3 | ADC | `OCXO_V` read-back of the loop-filter output. |
| Clock mux | PB6 | GPIO `MUX_SEL` | 0=OCXO (boot default), 1=Rb; drives 74LVC1G157 U52. |
| GPS PPS | PA0 | TIM2_CH1 | Primary TIMEPULSE capture. |
| GPS PPS2 | PC6 | TIM3_CH1 | Secondary TIMEPULSE2 capture (cross-check). |
| Ext-ref freq monitor | PB14 | TIM12_CH1 | `EXTREF_MON` — measures the ~10 MHz on mux input B for in-band check. |
| ETH PPS out | PB5 | MAC PTP | 1588 hardware PPS → U71 buffer → J15. |
| Ref term select | PC10 | GPIO `REF_TERM_EN` | Ext-ref SMA 50 Ω termination (default terminated). |

**OCXO steering loop.** `discipline` (4 Hz thread, PPS-paced at 1 Hz) is the **only** writer of PA4.
Topology: PA4 → R120 100k → OPA320 U36 (+in, with C98 0.1 µF and R121 10M to a 1.65 V bias) → unity
buffer → R122 1k → `OCXO_V` → OCXO Y3.1 + C101 + PA3 ADC. R121 (10 M) forces the Vc node to the
1.65 V center if PA4 ever goes Hi-Z — **Vc never floats**. Loop-filter poles: R120·C98 ≈ 15.9 Hz,
R122·C101 ≈ 1.6 kHz; the slow PI/FLL loop lives in firmware. On holdover the last good DAC value is
frozen; when Rb is the active reference the DAC holds OCXO at its last disciplined value (keeps the
fallback warm). C98/C99/C101 are **C0G/film** on the Vc path (X7R would FM-modulate the carrier); a
0.1 µF C0G needs a 1210/film package — a BOM/layout item, not a firmware constraint.

**PPS handling.** Cross-check PA0 vs PC6; reject outliers with a median/MAD gate before the edge
enters the loop. Full-scale DAC = OCXO ±0.4 ppm pull; 1 LSB ≪ holdover spec.

**Glitchless reference handoff (OCXO ⇄ Rb).** This is **not** a glitch-free mux IC (deliberately —
those hang if the outgoing clock has stopped, the exact Rb-failure case). Sequence: firmware
bridges the system clock to HSI, changes `MUX_SEL` (PB6), and relies on the STM32 **CSS** to catch a
dead source. Switch **OCXO→Rb only when** `EXTREF_MON` (PB14/TIM12) confirms an in-band ~10 MHz
**and** `RB_LOCK` (PB13) is asserted, held past a debounce window. Switch **Rb→OCXO immediately** if
**either** fails; hysteresis on re-engage prevents flap. `CLK_OUT` (PH0) is the disciplined 10 MHz
into the HSE-bypass input. `10MHz_RF_OUT` (J9) and `1PPS_OUT` (J15) are the buffered distribution.

---

## 4. I²C1 device map & usage (PB8/PB9, single bus)

15 devices, one bus, **no address collisions**. Pull-ups R202/R203 (4.7k→3V3_STM); LTC4311 U56
rise-time accelerator is **permanently enabled** (EN tied to 3V3_STM) — nothing to sequence. All
devices sit on always-on **3V3_STM** (no gated peripheral rail — prevents back-powering via ESD
clamps). Bus is 400 kHz.

| Addr | Device | Ref | Firmware notes |
|------|--------|-----|----------------|
| 0x18 | (LIS2DH12 alt) | — | not used; U59 strapped 0x19 |
| **0x19** | LIS2DH12 accel | U59 | SA0→3V3_STM. e-compass. INT unconnected → **poll**. |
| **0x1E** | IIS2MDC magnetometer | U61 | Fixed addr. e-compass. Poll. |
| **0x40** | INA228 PoE | U10 | R30 in the series feed → voltage + current both usable. |
| **0x41** | INA228 3V3_STM | U31 | Kelvin split of 3V3 for STM telemetry. |
| **0x42** | INA228 5V_DISP | U32 | Display rail. |
| **0x43** | INA228 3V3 (main) | U30 | 3V3-good telemetry; corroborates PG4. |
| **0x44** | **SHT45 humidity** | U72 | **Owns 0x44** — this is why GPS INA is 0x4A. |
| **0x45** | INA228 antenna bias | U26 | Precise antenna current (R89 0.1Ω, 1.638 A FS). |
| **0x46** | INA228 OCXO | U37 | OCXO rail. |
| **0x47** | INA228 VCC_RB | U44 | **Rb rail verify before/after enable.** |
| **0x48** | TMP117 | U58 | ADD0→GND. Enclosure temp (thermal loop). |
| **0x49** | TMP117 | U57 | ADD0→3V3_STM. Oscillator temp. |
| **0x4A** | INA228 GPS | U23 | **0x4A, not 0x44** — A0=A1=SDA strap (SHT45 owns 0x44). |
| **0x4C** | INA228 panel 5V | U54 | Panel-LED health (§4.1). ALERT on **PF13**. |
| **0x60** | ATECC608B secure element | U60 | PSA keys/identity (CryptoAuthLib). |

> **Firmware I²C map MUST adopt:** GPS INA = **0x4A**, SHT45 = **0x44**, LIS2DH12 = **0x19**,
> TMP117 U57=**0x49** / U58=**0x48**. Do **not** attempt to "correct" U23 to 0x44 (SHT45 collision).

**INA228 configuration.** Program shunt cal per each device's R_shunt; enable the ALERT on the
**averaged** conversion result with a suitable averaging window (`AVG`). Alerts are open-drain →
active-low on the GPIO listed in §1. On any INA alert, **read that
INA's diagnostic/flags register for the cause** (over/under current, bus over/under voltage,
conversion-ready), then run alarm evaluation.

### 4.1 Panel-LED current averaging
The panel LEDs are PWM-dimmed (`PANEL_LED_PWM`, PE0). INA228 U54 (0x4C) must average over a
window **≫ the 1 ms PWM period** so the ALERT fires on the averaged (not instantaneous) current.
Firmware **duty-normalizes**: I_true ≈ I_avg / duty. Health check = duty-normalized current within
the expected all-on band (≈126–132 mA at full duty), corroborated by `PANEL_LED_FAULT` (PF12).

---

## 5. GPIO scan model (1 kHz)

Everything that is neither a bus, a timer channel, nor an edge-wake source is serviced by a **1 kHz
port scan** that snapshots **GPIOF IDR** and **GPIOG IDR**, XOR-diffs against the previous sample,
and dispatches only changed bits. The rotary encoder is **not** scanned (hardware TIM1 encoder
mode on PA8/PA9); its button (PF11) is.

### 5.1 GPIOF bit masks (`IDR`)
| Bits | Mask | Signals | Polarity |
|------|------|---------|----------|
| 0–6 | `0x007F` | BUTTON_1..7 | active-low (pressed=0) |
| 7 | `0x0080` | DISP_TOUCH_INT | low=touch |
| 8 | `0x0100` | V_ANT_EN_FAULT | low=fault |
| 9 | `0x0200` | V_DISP_EN_FAULT | low=fault |
| 10 | `0x0400` | PROX_WAKE | see §6 (also EXTI) |
| 11 | `0x0800` | ENC_BUTTON | low=pressed |
| 12 | `0x1000` | PANEL_LED_FAULT | low=fault |
| 13 | `0x2000` | **INA_ALERT_5V_PANEL** (U54 0x4C) | low=alert |
| 14 | `0x4000` | **BKP_STM_PG** | high=good |
| 15 | `0x8000` | **BKP_GPS_PG** | high=good |

> Note PF14/PF15 are backup-rail power-good (not spare); PF13 carries the 9th INA alert (panel),
> the one INA alert that is **not** in GPIOG.

### 5.2 GPIOG bit masks (`IDR`)
| Bits | Mask | Group | Signals (bit order) |
|------|------|-------|---------------------|
| 0–7 | `0x00FF` | **PG rails** | PG0 3V3_GPS_LDO_PG, PG1 OCXO_LDO_PG, PG2 3V0_RF_LDO_PG *(mask until pull-up fitted)*, PG3 5V_PSU_PG, PG4 3V3_PSU_PG, PG5 OCXO_PSU_PG(÷), PG6 RB_PSU_PG, PG7 POE_PG |
| 8–15 | `0xFF00` | **INA alerts** | PG8 V_POE, PG9 3V3_STM, PG10 5V_DISP, PG11 3V3, PG12 3V3_GPS(0x4A), PG13 V_ANT, PG14 OCXO, PG15 VCC_RB |

PG-rail bits are active-high "good"; INA-alert bits are active-low. **Mask off PG2 (0x0004)** in
firmware until the `3V0_RF_LDO_PG` pull-up is fitted (open-drain LDO PWRGD reads low without it).

### 5.3 Dispatch
- **Button/UI bits (PF0–7, PF11):** debounce ~20–30 ms; emit press/long-press/repeat to the nav
  state machine. DISP_TOUCH_INT (PF7) → service the touch controller over I²C (no debounce).
- **EN-fault bits (PF8, PF9, PF12):** any low → log + alarm the corresponding RT9742 load.
- **PG-rail bits (PG0/1/3/4/6/7, PF14/PF15; **not** PG2 while masked):** any high→low transition → rail-fault alarm.
- **INA-alert bits (PG8–15 + PF13):** on assert, **read that INA228's flags** for cause (§4).

---

## 6. Interrupts / EXTI

| Signal | Pin | Mechanism | Why |
|--------|-----|-----------|-----|
| `GPS_PPS` | PA0 | **TIM2_CH1 input capture** | Sub-cycle PPS timestamp; not EXTI. |
| `GPS_TP2` | PC6 | **TIM3_CH1 input capture** | Secondary PPS. |
| `ETH_PPS_OUT` | PB5 | MAC PTP hardware | 1588 PPS out. |
| `EXTREF_MON` | PB14 | **TIM12_CH1** | Frequency measure of ext ref. |
| Encoder A/B | PA8/PA9 | **TIM1 encoder mode** | Hardware quadrature counter. |
| `PFI` | PE8 | **EXTI8** (falling) | Power-fail ISR: persist critical state, park DAC. |
| `FAN_TACH` | PA15 | **EXTI15** edge count | RPM over 1 s. |
| `RB_LOCK` | PB13 | **EXTI13** (debounced) | Immediate fail-safe revert on de-assert (polarity bit). |
| `PROX_WAKE` | PF10 | **EXTI10** wake + scan | Wake-from-low-power on presence; also visible in the scan. |
| `GPS_TXRDY` | PD5 | EXTI5 | Wake parser when F9T has data (after CFG-TXREADY re-map). |
| `GPS_ANT_OFF_MON` | PD4 | EXTI4 | LNA-state change corroboration. |
| `POE_NCM/LCF/NCL` | PC2/PC3/PC7 | EXTI2/3/7 | NCP1095 status/fault (open-drain/RTN=GND; **internal pull-up required** unless 10k ext fitted). |
| `USB_VBUS_SENSE` | PE2 | **Polled** | EXTI line shared with PC2 → cannot use EXTI. |

Everything in §5 (buttons, EN-faults, PG rails, INA alerts, backup PG) is **scanned, not EXTI** —
the 1 kHz scan is fast enough and avoids EXTI-line contention.

---

## 7. Monitoring & feedback loops

| Subsystem | Read | Expected | Fault action |
|-----------|------|----------|--------------|
| **OCXO discipline** | PPS offset (PA0/PC6), Vc sense (PA3), INA228 0x46, TMP117 0x49 | Vc ≈ center, offset in gate | Outlier reject; on GNSS loss → holdover (freeze DAC). |
| **Rb rail** | INA228 **0x47** *before trusting rail*; `RB_LOCK` (PB13); `EXTREF_MON` (PB14) | VCC_RB in 24.45−6.645·VCTRL envelope; lock asserted | Never set RB_PWR_EN before a safe digipot code. Rail out of band → don't gate VCC_RB. Rate C125–C128 ≥50 V; set OV/digipot per FE Vmax. |
| **Rb OV latch** | `RB_OV_DET` (PE3, polled) | low (idle) | On trip: latch already killed U40 autonomously; log, then pulse `RB_OV_RESET` (PD3) HIGH to clear after cause cleared. |
| **PoE** | INA228 0x40 *(current + voltage)*; `POE_PG`/PG7; `POE_NCM/LCF/NCL` (PC2/3/7); PFI (PE8) | class covers load | If budget can't cover Rb → defer/deny RB_PWR_EN. `PFI`→persist+park. `POE_KILL` latches via Q3 sustain. |
| **Watchdog** | supervisor liveness (timing+network+housekeeping) | all healthy | Kick `WDT_KICK` (PB2) **only** if all pass; windowed WDT catches both stall and runaway. `WDT_EN` (PC12) HIGH after init. Timeout → HW `POE_KILL`. |
| **Antenna** | `GPS_ANT_DETECT`, `GPS_ANT_SHORT` (via F9T + comparators U25), INA228 0x45 (R89), `V_ANT_EN_FAULT` (PF8) | ~15–30 mA present, no short | Foldback ≈182 mA autonomous; on short/open alarm + optionally hard-cut `ANT_BIAS_EN` (PC9). |
| **Panel LED** | INA228 0x4C duty-normalized; `PANEL_LED_FAULT` (PF12) | I_avg/duty ≈ nominal | Deviation → LED-string fault (open/short). |
| **Rails (general)** | INA228 0x41/0x42/0x43/0x46; PG lines (PG0/1/3/4/6/7, PF14/15) | good | Rail drop → alarm; **PG2 masked** — 3V0_RF LDO health via INA/other until pull-up added. |
| **Thermal** | TMP117 0x48 (enclosure) + 0x49 (osc) + die temp | in band | PI fan loop on PE5; over-temp → alarm; loop stall → fan full-speed (fail-safe). |
| **Supercaps** | BKP_STM_PG (PF14), BKP_GPS_PG (PF15) digital PG only | charged | On VIN loss, managers boost from supercap (autonomous); firmware watches PG + ephemeris window (~4 h). |

---

## 8. Alarms & fail-safe

- **Holdover alarm relay K2 (PA6).** Normally-**energized** fail-safe: `HOLDOVER_ALARM_RELAY`=HIGH
  keeps K2 energized when the clock is healthy. Any of {fault, reset, power loss, boot} →
  de-energized → NC contacts close = **ALARM** at J16 (both Form-C poles). Firmware MUST drive PA6
  HIGH only once service quality is actually met, and drop it on any disqualifying fault.
- **`POE_KILL` (PE15).** Commanded board cold-cycle; also hardware-OR'd from WDT/thermal/latched
  faults. Default RUN. Q3 (BC857W) is the high-side sustain PNP that holds the kill latched.
- **Status RGB D5 (PD12/13/14, TIM4).** green = locked stratum-1; amber = warming/holdover;
  red = unlocked/fault; blue pulse = activity/identify. PWM for brightness/blend. Default-OFF at
  reset (base pull-downs + Hi-Z).
- **Panel LEDs.** 6 white + 1 red (nets `PANEL_LED_WHITE_N_1..6`, `PANEL_LED_RED_N_1`) on
  `V_PANEL_LED`; enable PC0, PWM PE0, fault PF12. Both enable stages default OFF.
- **Local display + skyplot/dashboard** for detailed annunciation (never blocks timing).

---

## 9. Peripheral operating methods

**SPI4 (PE12 SCK / PE13 MISO / PE14 MOSI).** Three devices share the bus, each with a distinct CS:
- **NOR U62 (MX25L25645, Macronix):** CS = `SPI_NSS`/PE11 (R212 10k↑). **Held in reset by default**
  via `NOR_RST_N`/PE10 (R205 10k↓) — drive PE10 HIGH before first access. nWP/SIO2 pulled up.
- **Rb digipot U43 (MCP41U83):** CS = `SPI_DPOT_CS`/PD2 (R135 10k↑). SPI **mode 0,0**. Code 0x000 =
  Terminal B = **safe-low VCTRL** (VOUT min ~4.5 V); POR = midscale (~14.5 V, inside the 26 V OV
  envelope). Firmware pre-programs the NV wiper safe-low and verifies VCC_RB (0x47) before
  `RB_PWR_EN`. Wiper-write opcode/CRC per DS20007000B.
- **Display (ST7796):** CS = `DISP_CS`/PE9; D/C = `DISP_DC`/PB0; RST = `DISP_RST`/PA10 (held in
  reset by R209 10k↓); BL = `DISP_BL`/PE6. MOSI/SCK 22Ω series-isolated (R210/R211).

Firmware MUST arbitrate the shared bus (mutex) and reconfigure CPOL/CPHA/speed per device.

**RS-232 Rb path.** SN65C3221E U46 transceiver + DPDT relay K1 (G6K-2F-Y) selects RS-232 vs CMOS:
- `RB_RS232_CMOS_SW` (PE4): default **low = RS-232** (relay de-energized, fail-safe). Set HIGH for
  CMOS-level operation only if the FE variant needs it.
- Serial: MCU TX = `RB_RX`/PB4 (UART7_TX, **NJTRST → SWD-only**); MCU RX = `RB_TX`/PE7
  (UART7_RX). Confirm FE J6-8/J6-9 direction per surplus variant before trusting.
- `RB_LOCK` (PB13): opto U48 (APC-817) inverting → **lock polarity is a firmware config bit**;
  debounced (C148) + EXTI13.

**UART assignments.**
| UART | Pins | Use |
|------|------|-----|
| USART3 | PD8 (TX→GPS_RX) / PD9 (RX←GPS_TX) | Primary GNSS UBX/NMEA; also FW-upgrade path w/ RESET_N/SAFEBOOT_N. |
| UART4 | PD0 (TX) / PD1 (RX) | GNSS 2nd port (RTCM/aux). |
| UART7 | PB4 (TX) / PE7 (RX) | FE-5680A Rb telemetry. |

**USB console.** PA11/PA12 (D−/D+), self-powered device gated by `USB_VBUS_SENSE` (PE2, polled).
Driverless CDC-ACM console with full command/control parity to the web API; trace mirrored on SWO.

---

## 10. Firmware-critical cautions (summary)

1. **SWD-only.** `RB_RX` uses PB4/NJTRST → JTAG unavailable. All debug via SWD
   (PA13/PA14/PB3). Do not enable JTAG pins.
2. **`GPS_TXRDY` re-CFG.** F9T pin 19 defaults to GEOFENCE_STAT; firmware MUST send CFG-TXREADY
   to make it TX_READY before trusting PD5.
3. **Digipot safe code.** Code 0x000 = Terminal B = safe-low VCTRL; POR = midscale (~14.5 V, inside
   the 26 V OV envelope). Write a proven-safe NV code **before** `RB_PWR_EN` and verify `VCC_RB` on
   INA228 0x47 before `RB_VCC_GATE`.
4. **Held-in-reset defaults.** `NOR_RST_N` (PE10), `LAN_RST_N` (PD10), `DISP_RST` (PA10) all boot
   **asserted** — release in the §2 order.
5. **Gated-load defaults OFF.** `GPS_PWR_EN`, `ANT_BIAS_EN`, `DISP_EN`, `PANEL_LED_EN`,
   `RB_PWR_EN`, `RB_VCC_GATE`, `WDT_EN` all default OFF/safe; enable deliberately.
6. **J17 panel power.** The display, cap-touch, and rotary encoder are fed from J17.16 (5V_DISP) and
   J17.28 (3V3_STM); confirm both rails reach the panel before relying on Stage-6 bring-up. The
   panel-LED string is independent (Q22-switched anode).
7. **Mask PG2** until the `3V0_RF_LDO_PG` 10k→3V3_STM pull-up is fitted (open-drain LDO PWRGD reads
   low without it). Derive that rail's health from its INA/rail telemetry meanwhile.
8. **Use the power-good/telemetry signals normally:** POE_PG/PG7 (~2.93 V divided), INA228 0x40
   current, PG4 (~2.98 V), INA_ALERT_VCC_RB/PG15, GPS ANT_OFF. AP3441 PG drives to VIN when good
   (bench-confirm).
9. **Rb bring-up caveats.** Rate the Rb-buck output caps C125–C128 ≥50 V (they sit on the 24.45 V
   VCC_RB rail). For a 15 V-class FE, set the digipot bound / OV per the unit (the 24.45 V full-scale
   pedestal over-volts a 15 V unit; the OV latch only trips at 26 V). Confirm FE J6-8/J6-9 Tx/Rx
   direction per surplus variant.
10. **OCXO Vc dielectric.** C98/C99/C101 are C0G/film (never X7R — microphonics FM-modulate the
    carrier); a 0.1 µF C0G needs a 1210/film package — a BOM/layout item, not a loop constraint.
11. **`MUX_SEL` handoff.** Never write PB6 outside the reference state machine; always use the
    HSI-bridge + CSS glitchless sequence and the dual guard (EXTREF_MON + RB_LOCK).
12. **`OCXO_VC` (PA4) single-writer.** Only the `discipline` thread writes the DAC; it must never
    float (R121 holds 1.65 V, but firmware must not tri-state it during normal operation).
13. **5V-tolerance.** PG3 (pulled to 5 V) and PG7 (2.93 V divider node) are on 5 V-tolerant Port-G
    pins; the PG5 divider keeps its node ≈2.66 V from node N ≈2.98 V (loaded OCXO_PSU_PG; PG5 not FT — in range, > VIH).
    Grow VREF+ C37 to 100 nF at the pin (ADC ENOB for OCXO steering).

---

*As-built from the re-annotated netlist. This document is authoritative for the firmware↔hardware
pin contract.*
