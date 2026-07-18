# PoE GPS-Disciplined Stratum-1 NTP Server — Peripheral & Pin Map

**MCU:** STM32H563ZIT6 (U12) — Cortex-M33 @ 250 MHz, 2 MB flash, 640 KB SRAM, **LQFP144**, LDO core supply, –40/+85 °C, ~114 GPIO
**PHY:** Microchip LAN8742AI-CZ-TR (U11) — RMII, 10/100
**GNSS:** u-blox ZED-F9T-00B (U21) — multi-band timing receiver
**Reference oscillator:** Connor-Winfield **OH300-61003CV-010.0M** (Y3) — 10 MHz VC-OCXO, ±10 ppb, 3.3 V CMOS, voltage-controlled (US-origin)

> Pin assignments below are the **as-built KiCad netlist** for U12, cross-checked against ST pin data for `stm32h563zitx` (LQFP144). Ethernet uses AF11. **PB11 and PE1 are bonded on the LQFP144** but are **not routed** in the schematic — they are the only uncommitted GPIO. Non-signal pins: **PH1/OSC_OUT** (NC in HSE-bypass), **BOOT0** (R50 10 kΩ pull-down → user-flash boot), **VCAP×2** (core LDO). Every routed GPIO is committed (§13). The fault/UI fan-out is a **direct-GPIO** scan on GPIOF/GPIOG — there are no I/O expanders on the board.

-----

## 1. Ethernet — RMII (AF11) + LAN8742A (U11) control

|STM32 pin|Net           |Dir|Note                                                   |
|---------|--------------|---|-------------------------------------------------------|
|PA1      |RMII_REF_CLK  |in |50 MHz from PHY REFCLKO (REF_CLK-Out mode)             |
|PA2      |RMII_MDIO     |I/O|1.5 kΩ pull-up (R36) to 3V3_LAN                        |
|PC1      |RMII_MDC      |out|                                                       |
|PA7      |RMII_CRS_DV   |in |                                                       |
|PC4      |RMII_RXD0     |in |                                                       |
|PC5      |RMII_RXD1     |in |                                                       |
|PA5      |RMII_TX_EN    |out|only TX_EN pin on this package                         |
|PB12     |RMII_TXD0     |out|                                                       |
|PB15     |RMII_TXD1     |out|only TXD1 pin on this package                          |
|PB10     |RMII_RXER     |in |LAN8742 RXER / PHYAD0 strap — keep high-Z through reset|
|PB5      |ETH_PPS_OUT   |out|PTP pulse output → U71 buffer → 1PPS_OUT (J15) + scope tap|
|PD10     |LAN_RST_N     |out|LAN8742 reset — R35 10 kΩ pull-**down**: PHY held in reset until firmware drives PD10 high|

**Clock strategy:** PHY runs a 25 MHz crystal (Y1) and drives 50 MHz out on nINT/REFCLKO into PA1 (REF_CLK-Out mode; LAN8742 nINTSEL strapped **low** at reset via R41). RMII clock domain is intentionally separate from the disciplined timing reference. **PHY address = 0** (PHYAD0 strapped low, R230) — firmware MDIO driver must use addr 0. MODE[2:0]=111 (auto-neg, all-capable). Link is polled over MDIO (no PHY IRQ — pin 14 is REFCLKO).

-----

## 2. Timing reference & oscillators

|STM32 pin|Net                      |Note                                                               |
|---------|-------------------------|-------------------------------------------------------------------|
|PH0      |CLK_OUT ← clock-mux output|HSE **bypass**; mux (U52) selects OCXO (A) or external ref (B) — see §2.1|
|PH1      |(OSC_OUT)                |NC in bypass                                                       |
|PC14     |OSC32_IN                 |LSE 32.768 kHz (RTC)                                               |
|PC15     |OSC32_OUT                |LSE                                                                |
|PA4      |OCXO_VC (DAC1_OUT1)      |steering → loop filter → OCXO Vc (Y3.1)                            |
|PA3      |OCXO_V (ADC1)            |Vc monitor at Y3.1 — loop self-test + oscillator-aging trend       |
|PA0      |GPS_PPS (TIM2_CH1 capture)|← GPS TIMEPULSE (PPS) — 32-bit timer                              |
|PC6      |GPS_TP2 (TIM3_CH1 capture)|← GPS TIMEPULSE2 (redundant/aux)                                  |

**Reference = OH300-61003CV-010.0M VC-OCXO (Y3):**

- 10 MHz CMOS output → U52 mux input A → PH0 (via R123 22 Ω source term at Y3, R187 22 Ω PH0 damper). OCXO powered from a dedicated **3.327 V** LDO rail (U39 TPS7A5201, off a 3.70 V pre-buck U38).
- DAC1_OUT1 (PA4) → loop filter (R120/C98, U36 OPA320 buffer, R122) → OCXO Vc (Y3.1). **Vc is biased to ~1.65 V nominal** (R118/R119 100k/100k off VDDA; R121 10 MΩ holds 1.65 V if PA4 is Hi-Z) — never floating. Center the filter at 1.65 V so the pull range is symmetric.
- PA3 monitors Vc → loop self-test + oscillator-aging trend. Vc range is within VDDA, so **no divider** on PA3.
  > **Vc-path caps C98 / C99 / C101 are C0G/NP0** (piezoelectric X7R microphonics FM-modulate the carrier). 0.1 µF C0G needs a **1210 C0G or PPS/PEN film** package (unbuildable in 0402) — see §14.
- **Warm-up:** ~3.0–3.8 W turn-on, ~1.1–1.5 W steady, ~5 min to 0.1 ppm. Firmware must **hold off advertising stratum-1 / locked until OCXO warm AND GPS-locked**.
- Dedicated power monitor: INA228 U37 @ **0x46** (§4). OCXO pre-regulator power-good = **OCXO_PSU_PG** (drives U39 EN) monitored on **PG5** through a 0.89× divider (R232/R233); OCXO LDO PG on **PG1** (OCXO_LDO_PG).

### 2.1 Clock-source mux & dual reference (OCXO + Rb)

Both references are **permanent** — the **OCXO (Y3) is onboard**, the **Rb is external** (10 MHz in via the §2.2 SMA front end) — each on its own power rail, both powered and monitored at all times. A 2:1 LVCMOS clock mux (**74LVC1G157, U52**, S=PB6; glitchlessness delivered by the firmware HSI bridge + STM32 CSS rather than a glitch-free mux IC — see `sts1000_clock_mux.md`) sits in front of PH0: **input A (I0) = OCXO**, **input B (I1) = external 10 MHz reference** conditioned by the front end (§2.2). Mux output → PH0 (net `CLK_OUT`) **and**, via buffer U53, the bench 10 MHz fanout (J9). Both inputs are 10 MHz, so the PLL config is identical across a switch. **The STM32 never powers a reference down to use the other** — it only selects which one the mux forwards; both rails stay live so each is an instantly-available, already-warm source. Mux VCC = **3V3** (MCU VDD domain, not the OCXO LDO) so Voh ≤ VDD keeps PH0 abs-max safe.

|STM32 pin|Net           |Dir        |Function                                                           |
|---------|--------------|-----------|-------------------------------------------------------------------|
|PB6      |MUX_SEL       |out        |0 = OCXO (A, default/boot via R188 100k pull-down), 1 = Rb (B)     |
|PB14     |10MHz_CLK_OUT (TIM12 capture)|in|Rb-clock presence + frequency check (EXTREF_MON) — TIM12 counts the slicer's conditioned square (same net that feeds mux I1)|
|PB13     |RB_LOCK       |in (EXTI13)|Rb lock-good status (via opto U48)                                 |
|PB7      |RB_PWR_EN     |out        |**master Rb enable** — gates the Rb buck EN (through R156 → RB_PSU_PWR_EN → U40) and the RS-232 transceiver (U46 ~FORCEOFF)|
|PB1      |RB_VCC_GATE   |out        |**VCC_RB disconnect gate** (Q25 P-FET) — isolates the FE-5680A rail at the clock connector (R261 → Q25 gate)|

**Rb subsystem connections:** EXT_REF_IN (Rb 10 MHz → SMA → front end → mux I1), buffered GPS PPS (F9T TIMEPULSE → Rb 1PPS-in for self-discipline — no new STM32 pin), **Rb serial (UART7)** RB_TX = PE7 (← FE TXD) / RB_RX = PB4 (→ FE RXD). The Rb runs on its **own adjustable-buck rail, enabled by RB_PWR_EN (PB7) and monitored by INA228 U44 @ 0x47** — independent of the OCXO rail (INA228 U37 @ 0x46). Both INA228s report continuously.

> **JTAG caveat:** RB_RX is on **PB4 (NJTRST)** — the board is therefore **SWD-only** (no 4-wire JTAG). SWD (PA13/PA14/PB3) is unaffected.

**Auto-switch logic (firmware-owned) — two independent preconditions before using the Rb:**

1. Boot on OCXO (MUX_SEL = A).
1. **Check the actual Rb clock:** TIM12 on PB14 watches the conditioned square (`10MHz_CLK_OUT`) and declares input B *valid* only when edges advance **and** the measured frequency is in band (~10 MHz ± tolerance).
1. **Check the lock signal:** RB_LOCK (PB13) must be asserted (Rb warm + phase-locked).
1. **Switch OCXO → Rb** only when **both** #2 and #3 hold together, stable past a debounce/hysteresis window.
1. **Switch Rb → OCXO** immediately if **either** RB_LOCK de-asserts **or** the EXTREF_MON clock check fails. Hysteresis on re-engage prevents flapping; fallback is fail-safe.
1. **Glitchless switching:** U52 is not a glitch-free part, so bridge SYSCLK to HSI across the changeover, then re-lock the PLL on the new HSE source (CSS as hardware failsafe). Same-frequency inputs make this near-seamless.

**Because the Rb is always present** (and may be enabled concurrently with the OCXO), the PoE budget must cover both rails — see §6. The Rb runs on a **digipot-trimmed buck** (U40 MIC28516, VCC_RB ~15–24 V from the PoE/PD rail); **RB_PWR_EN drives that buck’s EN**, so enabling the Rb also stages its warm-up.

### 2.2 External-reference front end (single SMA input) — jumperless, firmware-configured

The source for **mux input B** is a single **SMA** (J8, `10MHz_RF_IN`) feeding a conditioning front end that swallows a wide range of 10 MHz references and hands the mux a clean **3.3 V CMOS square**. There is no DB9 clock source and no source-select bit. The **Rb is always external**: its 10 MHz reaches this front end over coax into the SMA, while its power/serial/lock/PPS run on the separate Rb housekeeping connector (J6, §15). **Every configuration choice is made by GPIO / analog switch — no jumpers, no fit/DNP straps.**

The one SMA input accepts, from the same connector, **both** input classes:
- **50 Ω sine** sources (e.g. an external FE-5680A Rb, ~0.5–1.0 Vrms into 50 Ω) — termination engaged.
- **TTL / CMOS high-impedance** logic clocks — termination lifted.

`REF_TERM_EN` (PC10) selects between the two; the **LTC6752xS5 slicer (U50)** self-biases and squares either.

**Chain (SMA → mux input B):**

`J8 SMA → D8 ESD → AC-couple → self-bias (1.5 V) → switched 50 Ω term (U49 TMUX1101, REF_TERM_EN) → Rseries + clamp → LTC6752xS5 (U50) → R184 33 Ω → 10MHz_CLK_OUT → mux I1`

> **Canonical: `ntp_server_rf_frontend.md`.** The front end is a **single LTC6752xS5** (5-lead TSOT-23) comparator slicer accepting the full input envelope; the only front-end control bit is **`REF_TERM_EN` (PC10)**. Slicer detail (1.5 V bias on a 3.0 V island U51; R_VCC guard) lives in the RF doc.

**Confirmation is free:** `EXTREF_MON` (PB14, TIM12) already gates input B on *edges advancing AND frequency ∈ 10 MHz ± band* — that same check validates the front end’s output. No new MCU pin verifies the front end.

|MCU pin|Net        |Dir|Function                                                            |
|-------|-----------|---|--------------------------------------------------------------------|
|PC10   |REF_TERM_EN|out|1 = 50 Ω shunt engaged (sine/50 Ω source, default via R176 pull-up); 0 = lifted (TTL/CMOS high-Z)|

**External-Rb housekeeping connector (FE-5680A, J6):** the **10 MHz does *not* run on this connector** — it goes to the SMA over coax. J6 (DE-9) carries only the Rb’s power/serial/lock/PPS: **lock → RB_LOCK (PB13)** via opto U48, **RS-232/CMOS telemetry → UART7 (RB_TX/PE7, RB_RX/PB4), mode-switched by the RS-232/CMOS relay K1 on PE4 (§15)**, **+15 V / +5 V supply ← the Rb adjustable buck (RB_PWR_EN, §6), gated out to the connector by the Q25 disconnect FET (RB_VCC_GATE/PB1)**. FE-5680A connector pinouts vary by variant — **verify against the specific unit before layout** (§14).

-----

## 3. GNSS — ZED-F9T (U21; LGA pin numbers in last column)

|STM32 pin|Net                      |Dir       |F9T pin                        |
|---------|-------------------------|----------|-------------------------------|
|PD8      |GPS_RX (USART3_TX → GPS RXD)|out     |43 (RXD)                       |
|PD9      |GPS_TX (USART3_RX ← GPS TXD)|in      |42 (TXD)                       |
|PA0      |GPS_PPS (TIM2_CH1 ← TIMEPULSE)|in    |53                             |
|PC6      |GPS_TP2 (TIM3_CH1 ← TIMEPULSE2)|in   |54                             |
|PD11     |GPS_RST_N                |out       |49                             |
|PD15     |GPS_SAFEBOOT_N           |out       |50 (firmware recovery)         |
|PD7      |GPS_DSEL                 |out       |47 (hold UART+I2C mode)        |
|PD6      |GPS_EXTINT               |out       |51                             |
|PD5      |GPS_TXRDY                |in (EXTI5)|19 (default fn = GEOFENCE_STAT — firmware must CFG to TX_READY)|
|PD4      |GPS_ANT_OFF_MON          |in (EXTI4)|5 (observe LNA-disable)        |
|PD0      |GPS_TXD2 (UART4_RX ← GPS TXD2)|in    |27 (RTCM/corrections, optional)|
|PD1      |GPS_RXD2 (UART4_TX → GPS RXD2)|out    |26 (RTCM/corrections, optional)|

**Supply & antenna:** GPS VCC = **3V3_GPS** (LT3045 U22, 3.32 V; EN = GPS_PWR_EN/PC8) monitored by INA228 U23 @ **0x4A** (§4). Antenna bias-T: high-side FET enabled by **ANT_BIAS_EN (PC9)**; presence sensed by INA181 U24 (across R77), precise current by INA228 U26 @ **0x45**; open/short flagged by comparators U25 (nets GPS_ANT_DETECT / GPS_ANT_SHORT). Status also reaches the STM32 via UBX-MON-RF over UART. V_BCKP (36) = **GPS_VBAT**, held by TPS61094 U34 + supercap through GPS VCC power-cycles; backup power-good on **PF15 (BKP_GPS_PG)** via comparator U67.
**Firmware upgrade path:** USART3 (PD8/PD9) + RESET_N (PD11) + SAFEBOOT_N (PD15) → full update incl. safeboot recovery.

-----

## 4. I²C1 bus — PB8 (SCL) / PB9 (SDA)

Shared bus, 3.3 V, Fast-mode. Pull-ups R202/R203 (4.7 kΩ → 3V3_STM); rise-time accelerator **LTC4311 (U56)**, ENABLE tied to 3V3_STM (always on). **15 devices, no collisions** (verified against A0/A1 straps):

|Device (ref)     |Function                                   |Addr |ALERT net → MCU pin |Notes                                          |
|-----------------|-------------------------------------------|-----|--------------------|-----------------------------------------------|
|INA228 (U10)     |PoE input rail V/I/P                        |0x40 |INA_ALERT_V_POE → PG8|A1=A0=GND                                      |
|INA228 (U31)     |STM 3.3 V (3V3_STM, Kelvin split of 3V3)    |0x41 |INA_ALERT_3V3_STM → PG9|A1=GND A0=VS                                  |
|INA228 (U32)     |5 V display rail (5V_DISP, RT9742 U33)      |0x42 |INA_ALERT_5V_DISP → PG10|A1=GND A0=SDA                               |
|INA228 (U30)     |main/general 3.3 V (3V3)                    |0x43 |INA_ALERT_3V3 → PG11|A1=GND A0=SCL                                  |
|**SHT45 (U72)**  |**Humidity / temperature**                 |0x44 |—                   |SHT45-AD1B; **owns 0x44** → GPS INA228 is at 0x4A       |
|INA228 (U26)     |Antenna bias rail (V_ANT)                   |0x45 |INA_ALERT_V_ANT → PG13|A1=A0=VS                                     |
|INA228 (U37)     |OCXO rail                                   |0x46 |INA_ALERT_OCXO → PG14|A1=VS A0=SDA                                   |
|INA228 (U44)     |Rb rail (VCC_RB, adjustable buck)           |0x47 |INA_ALERT_VCC_RB → PG15|A1=VS A0=SCL — pull-up R265 10 kΩ         |
|TMP117 (U58)     |Oscillator temperature (±0.1 °C)            |0x48 |—                   |ADD0 → GND                                     |
|TMP117 (U57)     |Environment/ambient temperature            |0x49 |—                   |ADD0 → 3V3_STM; drives fan loop                |
|**INA228 (U23)** |**GPS VCC rail (3V3_GPS)**                  |**0x4A**|INA_ALERT_3V3_GPS → PG12|A1=A0=SDA — **0x44 is taken by SHT45; do NOT re-strap**|
|INA228 (U54)     |Panel-LED 5 V rail (#9 — §8)                |0x4C |INA_ALERT_5V_PANEL → PF13|A1=SCL A0=GND                              |
|LIS2DH12 (U59)   |e-compass accel (tilt)                      |0x19 |—                   |SA0 → 3V3_STM = 0x19                           |
|IIS2MDC (U61)    |e-compass mag (heading)                     |0x1E |—                   |LIS2MDL register-compatible                    |
|ATECC608B (U60)  |Secure element (keys/identity, NTS/TLS)     |0x60 |—                   |Microchip default 0x60                         |

**Nine INA228 current monitors, direct-scanned.** Each ALERT is a **direct MCU GPIO** — eight on **PG8–PG15**, the ninth (panel LED) on **PF13** — read by firmware; there is **no I/O expander** in the alert path. Address encoding uses INA228 4-level A0/A1 straps (each → GND/VS/SDA/SCL). The map now spans **0x40,0x41,0x42,0x43,0x45,0x46,0x47,0x4A,0x4C**; 0x44 is SHT45, 0x48/0x49 are TMP117.

> **FT6336U capacitive touch** (0x38) is **not** on I²C1 — it rides the gated **5 V display I²C** (`I2C_DISP_SCL/SDA`) behind level-shifter **PCA9306 (U63)**; its INT is the direct GPIO **DISP_TOUCH_INT (PF7)**, not an I²C address on this bus. See §8.

**E-compass (heading) note:** the magnetometer shares the enclosure with strong/dynamic field sources (Rb physics package, OCXO oven, bucks, PoE magnetics, fan). For usable heading: (1) place it at a board edge/corner or external puck; (2) in-enclosure hard/soft-iron calibration once (fixed install); (3) apply **WMM/IGRF declination from the GPS fix** for true north; (4) tilt-compensate with the accelerometer (U59). Both e-compass INT lines are left unconnected — firmware **polls**.

-----

## 5. SPI4 bus — PE12 (SCK) / PE13 (MISO) / PE14 (MOSI) / PE11 (NSS)

|Device (ref)               |Function                                    |Note                                                                                             |
|---------------------------|--------------------------------------------|-------------------------------------------------------------------------------------------------|
|SPI NOR flash (U62)        |Discipline/temp logs, web assets, FW staging|MX25L25645 (Macronix — TW). CS = SPI_NSS (PE11, R212 pull-up); nRESET = NOR_RST_N (PE10, R205 pull-**down** → held in reset, firmware must release early)|
|ST7796 TFT (3.5”, 320×480) (display module)|Status/UI display               |CS = DISP_CS (PE9); write-only (module SDO NC — off MISO). MOSI/SCK re-buffered (R210/R211 22 Ω series → SPI_MOSI_DISP / SPI_SCK_DISP). Control lines §8|
|MCP41U83T-503E/ST digipot (U43)|Rb-buck FB-divider trim (VCC_RB set-point)|CS = SPI_DPOT_CS (PD2); SPI mode; VDD = **3V3**; NV-wiper, readback on MISO (PE13); bounded by fixed resistors (§6, §15)|

|STM32 pin|Net        |Note                                        |
|---------|-----------|--------------------------------------------|
|PE10     |NOR_RST_N  |firmware-controlled NOR reset (default-asserted — release early)|
|PD2      |SPI_DPOT_CS|Rb-buck FB digipot chip select (GPIO output, R135 idle-high)|

-----

## 6. Power, domains & backup

**Input chain:** PoE (802.3bt-capable front end) → magjack center taps (all 4 pairs, Alt-A+Alt-B) → FDMQ8205A bridges (U7/U8) → NCP1095 PD (U9) → bucks. PoE input TVS = **CR1 SMCJ58A**; PD bulk CPD = C10. See classification below.

**Rails (as-built regulators & setpoints):**

| Rail | Reg | Setpoint | Monitor |
|---|---|---|---|
| **3V3 / 3V3_STM** (single rail) | AP3441 buck **U29** | 3.33 V | INA228 U30 (0x43, main 3V3) + U31 (0x41, STM Kelvin split R106) |
| **5V_MAIN** | MIC28516 **U28** | 4.99 V | 5V_PSU_PG → PG3 |
| **5V_DISP** (display) | RT9742 **U33**, EN = DISP_EN | switched 5 V | INA228 U32 (0x42) |
| **5V panel-LED** | RT9742 **U55**, EN = PANEL_LED_EN | switched 5 V | INA228 U54 (0x4C) |
| **OCXO** | pre-buck U38 (3.70 V) → LDO **U39** | 3.327 V | INA228 U37 (0x46); OCXO_LDO_PG → PG1; OCXO_PSU_PG → PG5 (÷0.89) |
| **3V0_RF** (slicer island) | LT3045 **U51** | 3.01 V | 3V0_RF_LDO_PG → PG2 |
| **3V3_GPS** | LT3045 **U22**, EN = GPS_PWR_EN | 3.32 V | INA228 U23 (0x4A); 3V3_GPS_LDO_PG → PG0 |
| **VDDA** (OCXO-loop island) | LT3045 **U13** | 3.3 V | — |
| **VREF+** | MCP1502-33 **U14** | 3.3 V | — |
| **VCC_RB** (Rb) | MIC28516 **U40**, EN = RB_PSU_PWR_EN (← RB_PWR_EN via R156) | pedestal **24.45 V**, digipot-trimmed down (§15) | INA228 U44 (0x47); RB_PSU_PG → PG6 |

> **`3V3_STM` is the always-on housekeeping rail and is electrically the same rail as `3V3`** — a single AP3441 buck (U29) feeds both; U31 (0x41) meters the STM portion via a Kelvin split (R106), U30 (0x43) meters the main 3V3. **All housekeeping peripherals sit on always-on 3V3_STM** so nothing back-powers through I²C clamps.

**Power-good monitors — direct GPIO on Port G (PG0–PG7):**

|Pin |Net           |Rail / source                               |
|----|--------------|--------------------------------------------|
|PG0 |3V3_GPS_LDO_PG|GPS LDO U22                                 |
|PG1 |OCXO_LDO_PG   |OCXO LDO U39                                |
|PG2 |3V0_RF_LDO_PG |RF-island LDO U51 (LT3045 PG open-collector; R266 10 kΩ pull-up → 3V3_STM)|
|PG3 |5V_PSU_PG     |5 V buck U28 (PG3 is FT/5 V-tolerant — pulled to 5 V, in spec)|
|PG4 |3V3_PSU_PG    |3V3 buck U29 (AP3441 PG pulls to VIN when good; R94/R92 divider → 2.98 V)|
|PG5 |OCXO_PSU_PG (÷0.89)|OCXO pre-reg/EN monitor via R232/R233 (node N loaded by R124 **and** R232/R233 → source ≈ 2.98 V → PG5 ≈ 2.66 V, > VIH, safe)|
|PG6 |RB_PSU_PG     |Rb buck U40                                 |
|PG7 |POE_PG        |NCP1095 PGO via R264 174 k / R263 10 k divider → 2.93 V (PG7 is FT/5 V-tolerant)|

**Switched load domains (STM32-controlled, INA228-monitored):**

|STM32 pin|Net        |Domain                                                                               |
|---------|-----------|-------------------------------------------------------------------------------------|
|PC8      |GPS_PWR_EN |GPS VCC LDO enable (U22)                                                              |
|PC9      |ANT_BIAS_EN|Antenna bias-T high-side FET                                                          |
|PC11     |DISP_EN    |5 V display rail (RT9742 U33) + touch PCA9306 (U63) EN. Default-off via R104 pulldown |
|PC0      |PANEL_LED_EN|5 V panel-LED rail (RT9742 U55) EN. Default-off via R198 pulldown                    |
|PB7      |RB_PWR_EN  |**Rb-rail buck EN (via R156) + Rb RS-232 transceiver** — independent enable / warm-up staging|
|PB1      |RB_VCC_GATE|**VCC_RB disconnect FET (Q25)** — isolates the FE-5680A rail at J6                    |

**Rb rail design rules** (a digipot in a buck FB node can otherwise destroy the Rb — see §15 / `sts1000_vcc_rb_supply.md`):
- **Fixed bounding resistors** set a hard **24.45 V pedestal** (R147/R148/R134); the digipot (U43) can only trim *down* from there. No wiper state (POR/mid-scale/SPI-fault) exceeds the pedestal.
- **NV-wiper** powers the rail up to a stored safe-low value; **VREF_3V0** (MCP1502-30E **U45**, 3.0 V — *not* 4.096 V) sets the wiper transfer via OPA320 buffer U42.
- **Autonomous 26 V OV latch** (LMV393 U41 + regenerative BJT) trips with no MCU action; firmware observes via **RB_OV_DET (PE3, polled)** and clears via **RB_OV_RESET (PD3)**.
- **Closed-loop verify:** enable RB_PWR_EN, let the buck soft-start, read back VCC_RB on INA228 U44 (0x47), confirm in range before trusting the Rb.

**Backup managers (TPS61094 ×2 — U34 GPS-domain, U35 STM-domain):** both run **strapped/autonomous** (EN/MODE tied to enabled/auto-backup), so each enters backup on input collapse without waiting for firmware. Supercaps (C90/C91, 3 F) hold V_BCKP/VBAT through outages → GPS warm start + RTC retention. Setpoints: VBAT/backup **3.0 V**, VCHG termination **2.7 V** (R112/R113 **13.0 kΩ** — enclosure Tmax 51.7 °C < 65 °C corner → full 3.0 V rating, 2.7 V is the highest TPS61094 option under it), charge **~25 mA**. **No EN/MODE MCU pins.** Backup state is reported **digitally** via power-good comparators (U67): **BKP_STM_PG (PF14)** and **BKP_GPS_PG (PF15)**; there is **no analog supercap-SOC ADC channel** (PA6 carries HOLDOVER_ALARM_RELAY, PB1 carries RB_VCC_GATE).

**PoE hard-reset (board-level cold cycle):**

|STM32 pin|Net     |Note                                                        |
|---------|--------|------------------------------------------------------------|
|PE15     |POE_KILL|opens buck-input pass FET; default-RUN via external pulldown. WDT timeout, thermal cutoff, and latched faults OR into this driver.|

**Power budget & recommended PoE classification**

|Block                                         |Steady      |Notes                                      |
|----------------------------------------------|------------|-------------------------------------------|
|OCXO                                          |~1.1–1.5 W  |~3.0–3.8 W during ~5-min warm-up           |
|**Rb (RB_PWR_EN-gated, may run concurrently)**|**~6–10 W** |**~15–20 W during physics-package warm-up**|
|GPS (F9T) + active antenna                    |~0.5–0.7 W  |higher during acquisition                  |
|Ethernet PHY                                  |~0.3 W      |                                           |
|STM32 @ 250 MHz                               |~0.2 W      |                                           |
|Display + panel LEDs + RGB + fan + I²C/SPI    |~0.5–2 W    |fan ramps with temperature                 |
|Buck/PD conversion loss                       |~15 %       |                                           |
|**Steady-state, OCXO only**                   |**~3.5–4 W**|Rb disabled                                |
|**Steady-state, OCXO + Rb both on**           |**~12–14 W**|                                           |
|**Worst-case peak**                           |**~25–30 W**|concurrent Rb + OCXO cold-start warm-up    |

**As-built PoE class = Class 6 (Type 3 / 802.3bt, ~51 W PD)** — set by CLA/CLB (R13 232 Ω / R14 909 Ω); board draw ~25 W with generous margin. This comfortably covers concurrent OCXO+Rb operation. **Firmware should still stagger warm-ups** (OCXO first, defer RB_PWR_EN until OCXO warm and supercaps charged) to keep the cold-start peak inside the granted budget.

-----

## 7. ADC & analog monitors

**External:**

|STM32 pin|Net             |Measures                  |Note                                                            |
|---------|----------------|--------------------------|----------------------------------------------------------------|
|PA3      |OCXO_V          |OCXO steering voltage (Vc)|ADC1; loop self-test + aging trend; no divider (Vc ≤ 3.0 V)     |
|PE2      |USB_VBUS_SENSE  |USB 5 V VBUS present       |R68/R67 divider (k≈0.547 → ~2.87 V @ 5.25 V); D+ pull-up gate   |
|PE3      |RB_OV_DET       |Rb OV-latch state          |÷ (5·33/51 ≈ 3.24 V); polled                                    |
|PG5      |OCXO_PSU_PG     |OCXO pre-reg power-good    |R232/R233 ÷0.89 (confirm source ≤ 3.3 V — PG5 not 5 V-tolerant) |

**Internal (no pins):** VREFINT (true VDDA + droop), die temperature sensor, VBAT sense channel (STM32 backup rail).

> **Supercap backup readiness is digital, not analog:** the **BKP_STM_PG (PF14) / BKP_GPS_PG (PF15)** power-good bits report it (§6). There is no supercap state-of-charge ADC channel — PA6 carries HOLDOVER_ALARM_RELAY and PB1 carries RB_VCC_GATE.

-----

## 8. Indicators, fan & local UI (direct-GPIO scan)

**RGB status LED (D5) + fan + display control:**

|STM32 pin         |Net               |Timer/Note                                                                                |
|------------------|------------------|------------------------------------------------------------------------------------------|
|PD12 / PD13 / PD14|LED_R / LED_G / LED_B|TIM4 — common-anode 5 V, low-side NPN per color (active-high, default-off)              |
|PE5               |FAN_PWM           |out, TIM15 — 25 kHz 4-wire fan control (fail-safe: floating = full airflow)                |
|PA15              |FAN_TACH          |in, GPIO/EXTI15 — edge-count for RPM (open-collector; use MCU internal pull-up)            |
|PE9               |DISP_CS           |out, GPIO — ST7796 chip select on SPI4                                                    |
|PB0               |DISP_DC           |out, GPIO — ST7796 data/command                                                           |
|PE6               |DISP_BL           |out, TIM15 PWM — backlight dimming/blanking                                                |
|PA10              |DISP_RST          |out, GPIO — ST7796 + touch reset (R209 pull-down = held-in-reset default)                  |

**Panel-LED indicator array:** 5 V → RT9742 **U55** (EN = PANEL_LED_EN/PC0) → R199 220 mΩ shunt (INA228 **U54** @ 0x4C) → FB12 → V_PANEL_LED → **Q22 PNP high-side** → PANEL_LEDS_P common anode → J17 → panel. Cathodes = `PANEL_LED_WHITE_N_1..6` (6× white, 91 Ω ballast) + `PANEL_LED_RED_N_1` (red, 150 Ω). Dimming: **PANEL_LED_PWM (PE0, LPTIM2)** → Q23 → Q22 (non-inverting, active-high; Hi-Z reset = double default-OFF). Fault: **PANEL_LED_FAULT_N (PF12)** = U55 nFLG, R201 pull-up → 3V3_STM.

**Buttons, encoder, touch & presence — all direct GPIO (no I/O expander):**

|STM32 pin|Net           |Function                                                        |
|---------|--------------|----------------------------------------------------------------|
|PF0–PF6  |BUTTON_1 … BUTTON_7|7-button keypad (J17); each 10 kΩ → 3V3_STM + 0.1 µF, active-low; connector TVS (U19/U20)|
|PF7      |DISP_TOUCH_INT|FT6336U capacitive-touch INT (10 kΩ → 3V3_STM, **no debounce cap**) — primary wake-on-touch|
|PA8      |ENC_A         |Optical encoder A (TIM1_CH1) — via U68 74LVC2G17 Schmitt buffer (5 V-tolerant in, 3.3 V out); encoder output is **push-pull (driven)** — no pull-up needed|
|PA9      |ENC_B         |Optical encoder B (TIM1_CH2) — via U68                          |
|PF11     |ENC_BUTTON    |Encoder push-switch (J17); R236 10 kΩ pull-up + 0.1 µF          |
|PF10     |PROX_WAKE     |Magnetic **reed** presence wake (J17.18) — passive dry contact; R254 10 kΩ pull-up + C202, close-to-GND = active-low (no Vcc needed)|

**Fault flags — direct GPIO (RT9742 nFLG, open-drain active-low, renamed `_N`):**

|STM32 pin|Net            |Source                                     |
|---------|---------------|-------------------------------------------|
|PF8      |V_ANT_EN_FAULT_N|Antenna-bias RT9742 U27 nFLG (R200 pull-up)|
|PF9      |V_DISP_EN_FAULT_N|Display RT9742 U33 nFLG (R105 pull-up)    |
|PF12     |PANEL_LED_FAULT_N|Panel-LED RT9742 U55 nFLG (R201 pull-up)  |

**Display module — LCDwiki MSP4030 (ST7796S, 320×480 IPS, FT6336U touch):** pixel/control on SPI4 (SCK/MOSI via 22 Ω series buffers, write-only, SDO NC). Module 74LVC245 level-shifts control at 3.3 V; onboard BSS138 backlight low-side switch driven by DISP_BL PWM. **VCC = 5 V `5V_DISP`, gated by DISP_EN via RT9742 U33**, monitored by INA228 U32 @ 0x42. Touch (FT6336U @ 0x38) rides the gated 5 V I²C behind **PCA9306 (U63)** (EN = DISP_EN); CTP_RST shares DISP_RST (PA10); CTP_INT → **DISP_TOUCH_INT (PF7)**. ESD at J17: U15/U16 (3.3 V control/SPI) + U17 TPD4E05U06 (5.5 V touch lines). **Verify:** 5 V-side I²C pull-ups are on the module (PCA9306 secondary carries none on-board). Full subsystem: `sts1000_3v3p_i2c_peripherals.md`.

**Fan / thermal loop:** firmware drives FAN_PWM from a thermal loop using TMP117 U57 (ambient), TMP117 U58 (oscillator), and the die-temp channel. TACH edge-counting over 1 s suffices. 4-wire fan resting state = full airflow (fail-safe). TACH open-collector → MCU internal pull-up to 3.3 V.

-----

## 9. Reliability / monitoring

|STM32 pin|Net     |Dir       |Note                                                           |
|---------|--------|----------|---------------------------------------------------------------|
|PB2      |WDT_KICK|out       |external windowed watchdog refresh (TPS3430 U64 WDI); timeout → POE_KILL (HW)|
|PC12     |WDT_EN  |out       |WDT enable/arm (U64 SET1) — R213 100 kΩ pull-down = boot-disabled |
|PE8      |PFI     |in (EXTI8)|external power-fail comparator (U2, ~38 V trip — earlier than on-chip PVD)|
|PC13     |RTC_TAMPER|in      |intrusion, backup-domain, timestamped (RTC/TAMP peripheral)    |
|PA6      |HOLDOVER_ALARM_RELAY|out|drives normally-energized fail-safe relay **K2** (Q24 driver) → dry contacts at J16; de-energizes (NC closes = ALARM) on fault/reset/power-loss|

> Watchdog coverage = external WDT (U64) → POE_KILL, refreshed by WDT_KICK (PB2), armed by WDT_EN (PC12). **WDO_N is the WDT output only** — it is *not* a 3-way thermal/Rb-OV wire-OR (those paths are separate; Rb-OV = RB_OV_DET). There is no firmware WDT-fault observe GPIO.

**PoE controller monitor (NCP1095 U9 status):**

|STM32 pin|Net    |Note                                                                    |
|---------|-------|------------------------------------------------------------------------|
|PC7      |POE_NCL|in (EXTI7) — NCP1095 open-drain, **RTN(=GND)-referenced**, +72 V abs-max → safe direct (no divider); **needs pull-up to 3V3_STM** (10 kΩ ext or STM32 internal) — §14|
|PC2      |POE_NCM|in (EXTI2) — as PC7 (pull-up required)                                   |
|PC3      |POE_LCF|in (EXTI3) — as PC7 (pull-up required)                                   |

-----

## 10. Debug & USB console

|STM32 pin|Net                                   |
|---------|--------------------------------------|
|PA13     |SWDIO                                 |
|PA14     |SWCLK                                 |
|PB3      |SWO (TRACESWO)                        |
|NRST     |MCU_NRST (dedicated reset, J3.5)      |
|PA11     |USB_DM                                |
|PA12     |USB_DP                                |
|PE2      |USB_VBUS_SENSE (divider from 5 V VBUS)|

**SWD-only debug:** because **PB4 (NJTRST) carries RB_RX** (§2.1), 4-wire JTAG is unavailable — use SWD (PA13/PA14 + PB3 SWO). Out-of-band console = **USB CDC-ACM** (driverless virtual COM), USB-FS on HSI48 + CRS (no USB crystal); VBUS sensed on PE2 to gate the D+ pull-up. The only free UART-capable pins would collide with the GPS corrections UART4 (PD0/PD1), so the console is USB CDC.

-----

## 11. EXTI line allocation — direct-GPIO model

The fault/alert/UI fan-out is entirely on direct GPIO. The **INA228 ALERTs (PG8–PG15 + PF13), the power-good rails (PG0–PG7), and the 7 buttons (PF0–PF6)** each occupy an EXTI *line number* that is shared across ports (only one port’s pin per line can raise an interrupt), so these groups are **polled**, not interrupt-driven. EXTI is reserved for wake and latency-critical edges. Representative EXTI users:

|Line|Pin |Source          |Use            |
|----|----|----------------|---------------|
|2   |PC2 |POE_NCM         |PoE status     |
|3   |PC3 |POE_LCF         |PoE status     |
|4   |PD4 |GPS_ANT_OFF_MON |antenna LNA-off|
|5   |PD5 |GPS_TXRDY       |GPS TX-ready   |
|7   |PC7 |POE_NCL *or* PF7 DISP_TOUCH_INT|PoE status **/** wake-on-touch (mutually exclusive — pick one; the other polls)|
|8   |PE8 |PFI             |power-fail     |
|10  |PF10|PROX_WAKE       |presence wake  |
|11  |PF11|ENC_BUTTON      |UI wake        |
|13  |PB13|RB_LOCK         |Rb lock edge   |
|15  |PA15|FAN_TACH        |RPM edge-count |

**Polled groups (no EXTI):** INA228 ALERTs (PG8–15, PF13), PG rails (PG0–7), buttons (PF0–6), fault flags (PF8/PF9/PF12), backup PG (PF14/PF15). RTC_TAMPER (PC13) is handled by the RTC/TAMP peripheral. PPS inputs (PA0, PC6) and EXTREF_MON (PB14) are timer-capture (AF), not EXTI. **Line-number contention (e.g. line 7 PC7↔PF7, line 13 PB13↔PF13↔PG13, line 15 PA15↔PB15↔PF15↔PG15) is inherent to the direct-GPIO fan-out** and is resolved in firmware by enabling EXTI on only one pin per line and polling the rest.

-----

## 12. Reset / recovery matrix

|Target            |Mechanism                                   |Pin(s) / net    |
|------------------|--------------------------------------------|----------------|
|PHY               |dedicated reset (held low until FW)          |PD10 (LAN_RST_N)|
|GPS — soft        |RESET_N                                      |PD11 (GPS_RST_N)|
|GPS — hard        |VCC LDO enable cycle                         |PC8 (GPS_PWR_EN)|
|GPS — FW recovery |SAFEBOOT_N + RESET                           |PD15 + PD11     |
|Antenna           |bias-T FET                                   |PC9 (ANT_BIAS_EN)|
|TMP117 / ATECC608B / e-compass|none — on always-on 3V3_STM (no rail-cycle path); rely on device soft-reset|—     |
|NOR flash         |dedicated reset (default-asserted, FW releases)|PE10 (NOR_RST_N)|
|ST7796 display + touch|dedicated reset                          |PA10 (DISP_RST) |
|Display 5 V rail  |load-switch cycle                            |PC11 (DISP_EN)  |
|Panel-LED 5 V rail|load-switch cycle                            |PC0 (PANEL_LED_EN)|
|Rb rail           |buck EN / disconnect FET                     |PB7 (RB_PWR_EN), PB1 (RB_VCC_GATE)|
|Rb OV latch       |hardware-autonomous trip @ 26 V; FW reset after clear|PD3 (RB_OV_RESET)|
|Whole board (cold)|PoE kill (single-signal)                     |PE15 (POE_KILL) |
|MCU               |NRST / internal IWDG / external WDT → POE_KILL|NRST, PB2, PC12 |

-----

## 13. Spare pins

**Routed GPIO: none free.** The board is GPIO-full on the LQFP144 — the direct-GPIO fan-out (9 INA ALERTs, 8 PG rails, 7 buttons, touch/encoder/reed/faults on Ports F/G) commits the package's I/O.

- **Bonded-but-unrouted:** **PB11** and **PE1** are bonded on the LQFP144 and are the **only uncommitted GPIO**, but they are **not pinned into the schematic symbol** — expansion would require routing to them.
- **Non-signal / dedicated pins:** PH1/OSC_OUT (NC in HSE-bypass), BOOT0 (R50 10 kΩ pull-down → user-flash boot), VCAP×2 (core LDO, C34/C36 2.2 µF).
- **PG5 is used** (OCXO_PSU_PG monitor, not spare).

Status indication is the ST7796 display + RGB (PD12–14) + panel LEDs. Firmware has dedicated reset control of the display (PA10), NOR (PE10), GPS (PD11), PHY (PD10), plus rail power-cycles and POE_KILL. Further expansion is I²C/SPI-bus-only (I²C1 has address headroom) or requires routing PB11/PE1.

-----

## 14. Open items & verifications before layout / fab

The full netlist-driven consistency review is `sts1000_schematic_design_review.md`; the panel power, PoE power-good, current-monitoring, transistor orientations, and pull-up terminations are verified correct as-built.

**Open work items:**
- **GPS RF_IN series DC-block.** The u-blox ZED-F9T reference antenna-bias design (Integration Manual UBX-21040375) places a **47 pF C0G series DC-block between the bias-T node and RF_IN**, so the 5 V antenna bias reaches the antenna only. As-built, `GPS_RF_IN` ties the 5 V-biased node directly to U21.2. Add a ~47 pF C0G series DC-block between the antenna/bias node and U21.2, or obtain u-blox confirmation the internal RF_IN block tolerates continuous 5 V. Supervisor/current-sense (U24/U26) sit on the V_ANT side, unaffected.
- **Rb buck output caps ≥ 50 V.** C126/C127/C128 (47 µF) and C125 (47 nF) sit on the MIC28516 (U40) output = **VCC_RB** (24.45 V pedestal, 26 V OV). Source ≥ 50 V parts (X7R/X7S; C0G for feedforward C125); 47 µF 50 V won't fit 1210 → 1210/1812 or split. Input caps C133–C136 are 100 V X7T.
- **Cap packages:** C98/C99/C101 0.1 µF C0G (OCXO Vc loop / 1.65 V ref / Vcontrol — microphonics-critical, no X7R substitution) → **1210 C0G or PPS/PEN film** (0.1 µF C0G is unbuildable in 0402); C1/C42/C186 4.7 nF 2 kV (RJ45/USB/DE-9 shield→GND Y-caps) → **1808/1812** AEC-Q200 Y-caps (2 kV MLCC ≥ case 1808).
- **C37 (VREF+) 1 nF → 100 nF (optional).** VREF_3V3 (U12.32 VREF+ + U14 MCP1502 OUT) has C37 = 1 nF at the pin and C38 2.2 µF behind R48 50 Ω. ST AN5711 wants 100 nF + 1 µF at VREF+; change C37 → 100 nF, keep R48/C38 as the reference filter. Improves ADC ENOB for OCXO steering.

**Design verifications:**
- **AP3441 PG active-drive (bench-confirm early, high consequence).** DS39754: PG is pulled up to VIN (=5 V) when good but specs **no PG drive impedance**. Divider ratios are correct; PG4 ≈ 2.98 V. **Correction:** node N (`OCXO_PSU_PG`, → U39 EN) is loaded by **both** R124 10k and the R232+R233 leg, so it is ≈ **2.98 V** (not 3.68 V) and PG5 ≈ **2.66 V** (not 3.28 V) — both still valid (U39 EN VIH 1.1 V; PG5 > VIH 2.31 V). If PG were high-Z, node N→0 → U39 OCXO LDO never enables. Scope node N / PG5 / PG4 with the 5 V rail up; if PG5 sags near VIH raise R233.
- LAN8742 (U11): strap-detect budget for nINTSEL=0/REGOFF=0 behind 4.75 kΩ; xtal load caps (27 pF vs 20 pF CL → ~33 pF ideal, harmless as RMII is decoupled from the timing ref); J1 orderable magjack PN + PoE CT current rating. PHY addr = 0.
- ETH PTP auxiliary-snapshot internal trigger (RM0481) vs software TIM2↔PTP correlation.
- INA228 shunt values per expected per-rail currents.
- OCXO loop-filter: center Vc at 1.65 V; scale DAC 0–3.3 V to the pull range; set loop time constant.
- Clock mux (74LVC1G157 U52, S=PB6): confirm EXTREF_MON acceptance band + switch hysteresis.
- External-ref front end (single LTC6752xS5 U50, one SMA J8): size Rseries + clamp; R_VCC guard; 1.5 V island bias; source the D8/island TVS; confirm REF_TERM_EN safe reset default. Canonical: `ntp_server_rf_frontend.md`.
- PoE controller NCP1095 (U9): NCM/NCL/LCF (PC2/PC7/PC3) **resolved** — open-drain, RTN(=GND)-referenced, +72 V abs-max → safe direct to the 3.3 V GPIO but **float without a pull-up**; add 10 kΩ→3V3_STM (or STM32 internal pull-ups). No isolator (PD non-isolated, RTN=GND).
- External-Rb connector (FE-5680A, J6): confirm 1PPS / lock / RS-232 / +15 V / +5 V pin map per variant (10 MHz enters the SMA, **not** J6); verify K1 NC/NO map (de-energized must = RS-232). **Per-unit:** a 15 V-class FE is over-volted by the 24.45 V full-scale pedestal (OV latch only trips at 26 V) — bound the digipot code / OV per the specific FE variant; a 24 V-class unit is hard-protected by the bounding resistors.
- Rb-rail digipot (MCP41U83T-503E/ST U43, VDD=3V3, SPI_DPOT_CS=PD2): code 0 = Terminal B = VREF_3V0 = max VCTRL = min VOUT (4.51 V) = safe-low; POR = midscale (~14.5 V, within the 26 V OV envelope); SPI = Mode 0,0. Firmware pre-programs the NV wiper safe-low and verifies VCC_RB on INA228 U44 (0x47) before trusting the Rb. VREF = **3.0 V** (VREF_3V0, MCP1502-30E U45); transfer fn ≈ 24.45 − 6.645·VCTRL.
- USB: HSI48 + CRS config; PE2 VBUS-sense divider (~2.87 V @ 5.25 V); CDC-ACM descriptor; USB-connector ESD.
- Display/touch (MSP4030): backlight BSS138 (no external MOSFET); confirm 5 V-side I²C pull-ups on the module (PCA9306 secondary has none on-board); RT9742 inrush vs I_LIM + shunt headroom.
- E-compass: maximize distance from Rb/OCXO/bucks/PoE magnetics/fan; hard/soft-iron cal once; WMM/IGRF declination from GPS; tilt-comp via U59. Both INT lines unconnected → polling.
- Holdover relay K2 (G6K-2F-Y DC3): confirm coil resistance vs Q24 100 mA rating; normally-energized fail-safe.
- Backup: supercap C90/C91 DSF305Q3R0 is 3.0 V to 65 °C / 2.5 V at 85 °C; enclosure Tmax = **125 °F (51.7 °C)** < 65 °C → full 3.0 V rating, so VCHG termination is set to **2.7 V** (R112/R113 = 13.0k, highest TPS61094 option under the cap, ~1.5× backup energy vs 2.2 V). Keep the supercaps ≤ ~75 °C by placement; confirm BKP_STM_PG/BKP_GPS_PG thresholds.
- ZED-F9T V_BCKP current at max enclosure temperature (datasheet specifies only 45 µA @ 25 °C).

-----

## 15. Rubidium (Rb) reference — external FE-5680A, independently enabled

**There is no onboard Rb option.** The system has exactly two references: the **onboard OCXO (Y3)** (always on, STM32-steered) and an **external FE-5680A Rb**. The Rb’s **10 MHz enters via the single SMA front end (§2.2)**; its power/serial/lock/PPS run on the FE-5680A housekeeping connector (J6). The Rb rail is gated by **RB_PWR_EN (PB7)** and disconnected at the connector by **RB_VCC_GATE (PB1, Q25 P-FET)**; when enabled it runs concurrently and both references are monitored at all times (INA228 U37 = OCXO 0x46, U44 = Rb 0x47). The clock mux (§2.1) selects which reference feeds PH0.

### 15.1 Concept

Unlike the OCXO — which the STM32 actively steers through the PA4 DAC loop — the external FE-5680A is a self-contained **GPS-disciplined flywheel (GPSDRO)**: it takes a GPS 1PPS input and locks its own 10 MHz over a long time constant. When the Rb is enabled and selected:

- The Rb’s 10 MHz becomes the *selected* system reference (mux **input B** → PH0 + bench fanout).
- The **OCXO keeps running as a live, already-warm fallback** (mux input A) — never powered down.
- The **STM32 demotes** from “steer the oscillator” to “feed PPS, read health, cross-check GPS, and select the mux.” The PA4 DAC loop goes idle.

### 15.2 What it buys / when to enable it

|                 |Holdover < 1 µs|Holdover < 1 ms|Bench 10 MHz |
|-----------------|---------------|---------------|-------------|
|OCXO (always on) |~hours         |~1 day–2 wk    |very good    |
|Rb (when enabled)|**~1 day**     |**~months**    |**lab-grade**|

The Rb rail can be left off to save power and enabled only for a genuine multi-day GNSS-denied holdover need, or when a lab-grade 10 MHz is wanted on the bench outputs. For normal LAN-NTP service and minutes-to-hours outages, the OCXO alone keeps holdover error invisible to clients.

### 15.3 Rb subsystem connections (external FE-5680A)

|Signal / net   |STM32 pin               |Purpose                                                              |
|---------------|------------------------|---------------------------------------------------------------------|
|EXT_REF_IN     |SMA (J8) → §2.2 → mux I1 |Rb 10 MHz over coax into the SMA front end, then the clock mux       |
|(buffered PPS) |(F9T TIMEPULSE)         |→ Rb 1PPS-in for self-discipline (no new STM32 pin)                  |
|RB_TX          |PE7 (UART7 RX ← FE TXD) |Rb telemetry (lock, freq offset, health)                             |
|RB_RX          |PB4 (UART7 TX → FE RXD) |Rb control — **PB4 = NJTRST → board is SWD-only**                     |
|RB_RS232_CMOS_SW|PE4                    |FE-5680A RS-232/CMOS serial relay (K1) mode-select                   |
|RB_LOCK        |PB13 (EXTI13)           |lock-good status (via opto U48) → drives auto-switch                  |
|RB_PWR_EN      |PB7                     |Rb-rail buck EN (via R156) + transceiver enable / warm-up staging    |
|RB_VCC_GATE    |PB1                     |VCC_RB disconnect FET (Q25) at J6                                     |
|RB_OV_DET / RB_OV_RESET|PE3 / PD3       |autonomous 26 V OV-latch observe (polled) / reset                    |
|INA228 U44     |I²C1 0x47 (PB8/PB9)     |independent Rb-rail power monitor (always reporting)                  |

### 15.4 Power / PoE

- The Rb runs on a **digipot-trimmed buck (MIC28516 U40)** from the PoE/PD rail to the FE-5680A’s voltage. **Pedestal 24.45 V** (fixed by R147/R148/R134); the SPI digipot **U43** (MCP41U83, VDD=3V3, `SPI_DPOT_CS`=PD2) trims *down* only: VOUT ≈ 24.45 − 6.645·VCTRL (VCTRL 0–3.0 V from VREF_3V0). ~6–10 W steady, ~15–20 W physics-package warm-up. **RB_PWR_EN (PB7)** enables the buck (via R156 → RB_PSU_PWR_EN); the autonomous 26 V OV latch (U41) is the firmware-independent backstop.
- **PoE classification:** board is **Class 6 (Type 3 / bt)** — covers concurrent OCXO+Rb. Firmware staggers warm-ups to keep the cold-start peak inside budget.
- **Independent monitoring:** OCXO on **INA228 U37 (0x46)**, Rb on **INA228 U44 (0x47)** — both report continuously.

### 15.5 Operation (firmware)

1. OCXO always on; Rb rail enabled via RB_PWR_EN when wanted. Boot on OCXO (mux A); both references monitored whenever powered.
1. **Switch OCXO → Rb only when both pass:** EXTREF_MON (TIM12 / PB14) confirms a real, in-band ~10 MHz on input B **and** RB_LOCK (PB13) is asserted — stable past a debounce window.
1. **Switch Rb → OCXO immediately if either fails.** Fallback is fail-safe; hysteresis on re-engage prevents flapping.
1. Both references are 10 MHz → PLL config unchanged; 74LVC1G157 mux + firmware HSI bridge (CSS failsafe) across the switch (§2.1).
1. STM32 keeps capturing GPS PPS (PA0) to cross-check the active reference and flag drift via UART7.

### 15.6 Reference unit

- **External FE-5680A** (Frequency Electronics) — 10 MHz Rb, 1PPS-disciplinable, RS-232/CMOS serial, ~+15 V supply (+5 V on some variants). Connects via coax (10 MHz → SMA, §2.2) plus J6 (power/serial/lock/PPS). Surplus variants differ in connector pinout and serial level — **verify the specific unit before layout** (§14).
- No onboard Rb/atomic module is fitted; the only onboard reference is the OCXO (§2).
- Per sourcing policy, the external unit and any replacement are non-Chinese-origin.
