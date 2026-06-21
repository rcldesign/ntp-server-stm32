# PoE GPS-Disciplined Stratum-1 NTP Server — Peripheral & Pin Map

**MCU:** STM32H563VIT6 — Cortex-M33 @ 250 MHz, 2 MB flash, 640 KB SRAM, LQFP100, LDO supply, –40/+85 °C
**PHY:** Microchip LAN8742AI-CZ-TR (RMII, 10/100)
**GNSS:** u-blox ZED-F9T-00B (multi-band timing receiver)
**Reference oscillator:** Connor-Winfield **OH300-61003CV-010.0M** — 10 MHz VC-OCXO, ±10 ppb, 3.3 V CMOS, voltage-controlled (US-origin)

> All pin assignments validated against ST’s machine-readable pin data for the `stm32h563vitx` (LQFP100) package. Ethernet uses AF11. Pins not bonded on this package: **PE1, PB11** (do not use). Port E present = PE0, PE2–PE15. Port B skips PB11.

-----

## 1. Ethernet — RMII (AF11) + LAN8742A control

|STM32 pin|Signal          |Dir|Note                                                   |
|---------|----------------|---|-------------------------------------------------------|
|PA1      |ETH_RMII_REF_CLK|in |50 MHz from PHY REFCLKO (REF_CLK-Out mode)             |
|PA2      |ETH_MDIO        |I/O|1.5 kΩ pull-up to VDDIO                                |
|PC1      |ETH_MDC         |out|                                                       |
|PA7      |ETH_RMII_CRS_DV |in |                                                       |
|PC4      |ETH_RMII_RXD0   |in |                                                       |
|PC5      |ETH_RMII_RXD1   |in |                                                       |
|PA5      |ETH_RMII_TX_EN  |out|only TX_EN pin on this package                         |
|PB12     |ETH_RMII_TXD0   |out|                                                       |
|PB15     |ETH_RMII_TXD1   |out|only TXD1 pin on this package                          |
|PB10     |ETH_RMII_RX_ER  |in |LAN8742 RXER / PHYAD0 strap — keep high-Z through reset|
|PB5      |ETH_PPS_OUT     |out|PTP pulse output — verification/scope tap              |
|PD10     |PHY_nRST        |out|LAN8742 reset                                          |

**Clock strategy:** PHY runs a 25 MHz crystal and drives 50 MHz out on nINT/REFCLKO into PA1 (REF_CLK-Out mode → strap LAN8742 nINTSEL **low** at reset). RMII clock domain is intentionally separate from the disciplined timing reference.

-----

## 2. Timing reference & oscillators

|STM32 pin|Signal                     |Note                                                               |
|---------|---------------------------|-------------------------------------------------------------------|
|PH0      |OSC_IN ← clock-mux output  |HSE **bypass**; mux selects OCXO (A) or external ref (B) — see §2.1|
|PH1      |OSC_OUT                    |NC in bypass                                                       |
|PC14     |OSC32_IN                   |LSE 32.768 kHz (RTC)                                               |
|PC15     |OSC32_OUT                  |LSE                                                                |
|PA4      |DAC1_OUT1 → OCXO Vc (pin 1)|steering via loop filter                                           |
|PA0      |TIM2_CH1 (input capture)   |← GPS TIMEPULSE (PPS) — 32-bit timer                               |
|PC6      |TIM3_CH1 (input capture)   |← GPS TIMEPULSE2 (redundant/aux)                                   |

**Reference = OH300-61003CV-010.0M VC-OCXO:**

- 10 MHz CMOS output → PH0 directly (3.0 V Voh, HSE bypass).
- DAC1_OUT1 (PA4) → loop filter → OCXO Vc (pin 1). **Vc must be biased to ~1.65 V nominal** — never leave it floating (datasheet: floating Vc → unstable output). Center the filter at 1.65 V so the ±0.4 ppm pull range is symmetric.
- PA3 (ADC INP15) monitors Vc → loop self-test + oscillator-aging trend. Vc range 0.30–3.00 V is within VDDA, so **no divider needed** on PA3.
- **Warm-up:** ~3.0–3.8 W turn-on, ~1.1–1.5 W steady, ~5 min to 0.1 ppm. Firmware must **hold off advertising stratum-1 / locked until OCXO warm AND GPS-locked**.
- Dedicated power monitor: see INA228 #5 (§4) on the OCXO rail.

### 2.1 Clock-source mux & dual reference (OCXO + Rb)

Both references are **permanent** — the **OCXO is onboard**, the **Rb is external** (10 MHz in via the §2.2 SMA front end) — each on its own power rail, both powered and monitored at all times. A 2:1 LVCMOS clock mux (**74LVC1G157**, S=PB6; glitchlessness delivered by the firmware HSI bridge + STM32 CSS rather than a glitch-free mux IC — see `sts1000_clock_mux.md`) sits in front of PH0: **input A = OCXO**, **input B = external 10 MHz reference**, conditioned by the **external-reference front end (§2.2)** — a single **SMA** input taking the always-external Rb’s 10 MHz (or any sine/TTL/CMOS source). Mux output → PH0 **and** the bench 10 MHz fanout, so the bench output follows the active reference. Both inputs are 10 MHz, so the PLL config is identical across a switch. **The STM32 never powers a reference down to use the other** — it only selects which one the mux forwards; both rails stay live so each is an instantly-available, already-warm source.

|STM32 pin|Net                   |Dir        |Function                                                           |
|---------|----------------------|-----------|-------------------------------------------------------------------|
|PB6      |MUX_SEL               |out        |0 = OCXO (A, default/boot), 1 = Rb (B)                             |
|PB14     |EXTREF_MON (TIM12_CH1)|in         |Rb-clock presence + frequency check (timer counts/captures input B)|
|PB13     |RB_LOCK               |in (EXTI13)|Rb lock-good status                                                |
|PB7      |RB_PWR_EN             |out        |**independent enable of the Rb power rail**                        |

**Rb subsystem connections:** EXT_REF_IN (Rb 10 MHz → mux input B), buffered GPS_PPS_OUT (fan-off the F9T TIMEPULSE → Rb 1PPS-in for self-discipline — no new STM32 pin), **UART7** RB_UART_TX = PB4 / RB_UART_RX = PE7 (Rb telemetry/health). The Rb runs on its **own rail, separately enabled (RB_PWR_EN) and separately monitored (INA228 #6, 0x47)** — independent of the OCXO rail (INA228 #5, 0x46). Both INA228s report continuously, so the STM32 always sees the health of both references regardless of which the mux is forwarding.

**Auto-switch logic (firmware-owned) — two independent preconditions before using the Rb:**

1. Boot on OCXO (MUX_SEL = A).
1. **Check the actual Rb clock:** TIM12 on PB14 watches input B and declares it *valid* only when edges are advancing **and** the measured frequency is in band (~10 MHz ± tolerance). A present-but-wrong or absent clock does not qualify.
1. **Check the lock signal:** RB_LOCK (PB13) must be asserted (Rb warm + phase-locked).
1. **Switch OCXO → Rb** only when **both** #2 (real, in-band clock) **and** #3 (lock asserted) hold true together, stable past a debounce/hysteresis window.
1. **Switch Rb → OCXO** immediately if **either** RB_LOCK de-asserts **or** the EXTREF_MON clock check fails (frequency out of band or edges stop). Hysteresis on the re-engage path prevents flapping; fallback is fail-safe.
1. **Glitchless switching:** the 74LVC1G157 is not a glitch-free part, so bridge SYSCLK to HSI during the changeover, then re-lock the PLL on the new HSE source (CSS as hardware failsafe). Same-frequency inputs make this near-seamless; never switch the live HSE feed without the HSI bridge.

**Because the Rb is always present** (and may be enabled concurrently with the OCXO), the PoE budget must cover both rails — see §6 for the revised classification (Type 2 / bt). The Rb runs on a **digipot-adjustable buck** (set to the module’s voltage, commonly +15–24 V, bucked from the PoE/PD rail); **RB_PWR_EN drives that buck’s EN**, so enabling the Rb also stages its warm-up independently of the always-on OCXO.

### 2.2 External-reference front end (single SMA input) — jumperless, firmware-configured

The source for **mux input B** is a single **SMA** input feeding a conditioning front end that swallows a wide range of 10 MHz references and hands the mux a clean **3.3 V CMOS square**. **There is no DB9 clock source** — the earlier DB9-vs-SMA source-select concept is gone (which is why `REF_SRC_SEL`/PA10 is freed). The **Rb is always external**: its 10 MHz reaches this front end over coax into the SMA, while its power/serial/lock/PPS run on the separate Rb housekeeping connector (below) — **no 10 MHz on that connector**. **Every configuration choice is made by GPIO / analog switch — no jumpers, no fit/DNP straps.**

The one SMA input accepts, from the same connector, **both** input classes:
- **50 Ω sine** sources (e.g. an external FE-5680A Rb, ~0.5–1.0 Vrms into 50 Ω) — termination engaged.
- **TTL / CMOS high-impedance** logic clocks — termination lifted (must not be loaded into 50 Ω).

`REF_TERM_EN` (PC10) selects between the two; the LTC6752xS5 slicer self-biases and squares either.

**Chain (SMA → mux input B):**

`SMA → 50 Ω term (switched, REF_TERM_EN) → AC-couple → Rseries + clamp → LTC6752xS5 comparator → mux input B`

> **Canonical: `ntp_server_rf_frontend.md`.** The LTC6957 family went EOL; the front end is a
> **single LTC6752xS5** (5-lead TSOT-23) comparator slicer, which accepts the full input
> envelope (sine to rail-to-rail-and-beyond) and so **deletes the second slicer, the ÷2/fixed
> pad, and the 2:1 output mux**. Consequences for this pin map:
> - `REF_SLICER_SEL` is **gone** (single slicer) → **PC0 freed**.
> - `REF_SRC_SEL` is **gone** (single SMA input, no source to select) → **PA10 freed**.
> - The **only** remaining front-end control bit is **`REF_TERM_EN` (PC10)**.
> - Slicer detail (1.5 V bias on a 3.0 V island; R_VCC 33 Ω + D5 3.3 V VCC guard; S5 pinout
>   1=Q/2=VEE/3=+IN/4=−IN/5=VCC; D4/D5 TVS sourcing) lives in the RF doc.

- **Switched 50 Ω termination** — an analog switch grounds a 50 Ω shunt for a 50 Ω sine source and lifts it for a high-Z logic source. `REF_TERM_EN` (PC10).
- **AC-couple + self-bias** — series ~0.1 µF strips DC and lets the slicer self-bias to its threshold (sine or square, any DC offset, no reconfiguration).
- **Protection** — series R into a Schottky clamp to the island rail/GND at the slicer input, plus the local R_VCC + D5 guard at the LTC6752 VCC pin (the S5 base variant has a 3.6 V supply abs-max). Makes an unknown SMA input (hot sine / 5 V logic) safe to plug in. See RF doc.

**Confirmation is free:** `EXTREF_MON` (PB14, TIM12) already gates input B on *edges advancing AND frequency ∈ 10 MHz ± band* — that same check validates the front end’s **output**. A mis-conditioned input simply fails the gate; the reference state machine never switches to B (and reverts if it was on B). No new MCU pin verifies the front end.

**Control — one direct MCU GPIO, no build options.** All hardware is **always populated**; the MCU adapts the path at runtime. The runtime config is **one push-pull output**:

|MCU pin|Net        |Dir|Function                                                            |
|-------|-----------|---|--------------------------------------------------------------------|
|PC10   |REF_TERM_EN|out|1 = 50 Ω shunt engaged (sine/50 Ω source); 0 = lifted (TTL/CMOS high-Z)|

`REF_TERM_EN` is a direct output (no EXTI), defaulting to a safe boot state via the GPIO reset state plus the slicer’s self-bias. **PA10 and PC0 are freed** by this collapse (§13).

**External-Rb housekeeping connector (FE-5680A):** the **10 MHz does *not* run on this connector** — it goes to the SMA above over coax. The connector (DB9 on a typical FE-5680A) carries only the Rb’s power/serial/lock/PPS, mapping onto the existing Rb nets — **lock → RB_LOCK (PB13)**, **RS-232/CMOS telemetry → RB_UART (UART7: PB4/PE7), mode-switched by the RS-232/CMOS relay on PE4 (§6)**, **+15 V / +5 V supply ← the Rb adjustable buck (RB_PWR_EN, §6)** — and the unit self-disciplines from the **buffered GPS PPS (GPS_PPS_OUT → 1PPS-in)**. FE-5680A connector pinouts vary by variant — **verify against the specific unit before layout** (§14). The “15 V” of the FE-5680A is its **supply**, not a signal; it never touches the RF (SMA) path.

-----

## 3. GNSS — ZED-F9T (LGA pin numbers in last column)

|STM32 pin|Signal                   |Dir       |F9T pin                        |
|---------|-------------------------|----------|-------------------------------|
|PD8      |USART3_TX → GPS RXD      |out       |43 (RXD)                       |
|PD9      |USART3_RX ← GPS TXD      |in        |42 (TXD)                       |
|PA0      |TIM2_CH1 ← TIMEPULSE     |in        |53                             |
|PC6      |TIM3_CH1 ← TIMEPULSE2    |in        |54                             |
|PD11     |GPS_RESET_N              |out       |49                             |
|PD15     |GPS_SAFEBOOT_N           |out       |50 (firmware recovery)         |
|PD7      |GPS_DSEL                 |out       |47 (hold UART+I2C mode)        |
|PD6      |GPS_EXTINT               |out       |51                             |
|PD5      |GPS_TXRDY / GEOFENCE_STAT|in (EXTI5)|19                             |
|PD4      |GPS_ANT_OFF_MON          |in (EXTI4)|5 (observe LNA-disable)        |
|PD1      |UART4_TX → GPS RXD2      |out       |26 (RTCM/corrections, optional)|
|PD0      |UART4_RX ← GPS TXD2      |in        |27 (RTCM/corrections, optional)|

**Antenna supervisor:** F9T ANT_DETECT (4) / ANT_SHORT_N (6) / ANT_OFF (5) wire to the bias-T sense/control network; status reaches the STM32 via UBX-MON-RF over UART **and** independently via the antenna-rail INA228 (current → open/short/OK). V_BCKP (36) must stay powered during GPS VCC power-cycles (see backup tree).
**Firmware upgrade path:** UART1 (PD8/PD9) + RESET_N (PD11) + SAFEBOOT_N (PD15) → full update incl. safeboot recovery.

-----

## 4. I²C1 bus — PB8 (SCL) / PB9 (SDA)

Shared bus, 3.3 V, Fast-mode. Suggested 7-bit addresses (collision-free):

|Device             |Function                                                   |Addr (7-bit)             |Notes                                                                         |
|-------------------|-----------------------------------------------------------|-------------------------|------------------------------------------------------------------------------|
|TMP117 #1          |Oscillator temperature (±0.1 °C)                           |0x48                     |ADD0 → GND                                                                    |
|TMP117 #2          |Environment/ambient temperature                            |0x49                     |ADD0 → V+; drives fan loop                                                    |
|ATECC608B          |Secure element (keys/identity, NTS/TLS)                    |0x60                     |Microchip; default 0x60, configurable                                         |
|INA228 #1          |PoE input rail V/I/P                                       |0x40                     |A0/A1 straps                                                                  |
|INA228 #2          |STM32 3.3 V rail                                           |0x41                     |                                                                              |
|INA228 #3          |GPS VCC rail                                               |0x44                     |                                                                              |
|INA228 #4          |Antenna bias rail                                          |0x45                     |                                                                              |
|INA228 #5          |OCXO rail (fixed, dedicated low-noise)                     |0x46                     |warm-up vs steady; oven-fault detection                                       |
|INA228 #6          |Rb rail (adjustable buck, RB_PWR_EN = buck EN)             |0x47                     |independent; **closed-loop verify of the digipot-set voltage** (INA_ALERT_VCC_RB)|
|INA228 #7          |**V_DISP_5V (display rail, RT9742-switched)**             |0x42                     |INA_ALERT_DISP; 0.1 Ω shunt, ADCRANGE=1, output side; panel-health + backlight draw|
|INA228 #8          |main/general 3.3 V rail (3V3_STM)                          |0x43                     |INA_ALERT_3V3 (former "peripheral 3.3 V" assignment retired with the 3V3_P rail)|
|FT6336U (cap-touch)|Touchscreen — wake-on-touch + UI input                     |0x38 (fixed)             |on **V_DISP_5V** (gated) behind **PCA9306 (U57)**; INT→U48 GPA7; reuses retired TCA9534A slot|
|MCP23017 (U47)     |PG + EN-fault + UI aggregator (RB_OV excluded — on PE3/PD3): PortA 8×PG, PortB EN-faults + discrete inputs|0x20|**MIRROR=1**; INTA+INTB tied → **PG_INT_N** → PA10/EXTI10 (R152 10k); A0/A1/A2→GND; /RESET=EXP_RESET (PC0, R154 pull-down); **VDD 3V3_STM**. PortB: [0] V_ANT_EN_FAULT, [1] **freed** (3V3_P EN-fault retired with the rail), [2] **V_DISP_EN_FAULT** (RT9742 nFLG, pull-up to 3V3_STM), [3] **PROX_WAKE** (PIR), [4–7] spare. See sts1000_fault_aggregation.md|
|MCP23017 (U48)     |7-button keypad (PortA GPA0–6) + CTP_INT (GPA7) + 8×INA ALERT (PortB)|0x21|**MIRROR=0**; A0→VS; PortA INTA→**BTN_INT_N**→PE0/EXTI0 (on-change, R148 10k) — GPA7=**DISP_TOUCH_INT** (FT6336U, R167 10k→3V3_STM, **no debounce cap**); PortB INTB→**INA_ALERT_INT_N**→PC12/EXTI12 (INA, level, R153 10k); /RESET=EXP_RESET (PC0); **VDD 3V3_STM**|
|IIS2MDC (e-compass mag)|Heading (mag) — device facing for sat-plot N (vertical mount)|0x1E (fixed)|industrial; LIS2MDL register-compatible; CS→Vdd_IO; **3V3_STM**; keep-out/cal note below. *Fusion-IMU fallback: BNO086 0x4A/0x4B.*|
|LIS2DH12 (e-compass accel)|Tilt (gravity vector) for tilt-compensated heading|0x18 / 0x19 (SA0)|LIS2DH/LIS3DH reg family; WHO_AM_I 0x33; CS→Vdd_IO; SA0→Vdd_IO=0x19; **3V3_STM**|

|STM32 pin|Signal          |Note                                                                              |
|---------|----------------|----------------------------------------------------------------------------------|
|PC12     |INA_ALERT_INT_N |EXTI12 — INA-alert interrupt = U48 **INTB** (8 INA228 ALERTs, individual bits on U48 PortB, latched; read PortB to identify, then that INA228 for cause). Pull-up = **R153** on U48 sheet — **drop legacy R44** to avoid a double pull-up|
|PA10     |PG_INT_N        |EXTI10 — PG+EN-fault interrupt = U47 INTA+INTB tied (8 PG on PortA + 2 EN-fault on PortB[0:1]). R152 10k pull-up on U47 sheet|
|PE0      |BTN_INT_N       |EXTI0 — button interrupt = U48 INTA (8 buttons). R148 10k pull-up on U48 sheet|

**Addressing capacity:** INA228 uses 4-level encoding on A0/A1 (each pin → GND/VS/SDA/SCL) = **16 addresses (0x40–0x4F)**, so all eight INA228s live on this one bus — no second bus needed. INA228s occupy **0x40–0x47** and TMP117s **0x48–0x4B** to avoid the INA/TMP address-window overlap. (FT6336U 0x38, MCP23017 U47/U48 0x20/0x21, ATECC608B 0x60, e-compass IIS2MDC 0x1E + LIS2DH12 0x18/0x19 — all clear of the INA/TMP addresses. The proximity wake left I²C entirely — it is a discrete PIR on U47 Port B. The former 0x2C/0x2D digipot addresses are freed — the Rb-buck FB digipot moved to SPI4, §5. The former TCA9534A 0x38 is retired — the keypad is on MCP23017 U48; FT6336U reuses 0x38.)

**Bus buffer / speed:** with ~15 nodes (eight INA228 + two TMP117 + ATECC608B + IIS2MDC + LIS2DH12 + two MCP23017; the FB digipot moved to SPI, the proximity wake is an off-bus PIR, and the FT6336U sits behind the PCA9306 on the gated 5 V side) the bus capacitance approaches the 400 kHz limit, so an **I²C rise-time accelerator** is fitted — **LTC4311ISC6 (locked)** — to restore clean edges and run **Fast-mode 400 kHz**. **No segmenting buffer is needed:** the 3V3_P rail was removed, so all bus devices are on always-on 3V3_STM and there is nothing gated to back-power (the old PCA9517/TCA4311 segmenting requirement is retired — see sts1000 i2c-peripherals doc §2). The accelerator EN is wired to **I2C_BUF_EN (PA9)** so firmware can isolate/recover a stuck bus (or tie EN high if unused). Zero MCU-bus pins beyond the optional EN.

**E-compass (heading) note:** the magnetometer shares the enclosure with strong/dynamic field sources (Rb physics package, OCXO oven, switching bucks, PoE magnetics, fan motor, DC rail currents). For usable heading: (1) place it as far from those as possible — board edge/corner or an external puck; (2) do an in-enclosure hard/soft-iron calibration — since the install is fixed, calibrate once and store the offset; (3) it reports *magnetic* heading, so apply the **WMM/IGRF declination computed from the GPS fix** to get *true* north for the sat plot. Use the accelerometer for tilt-compensation if the mount isn’t level.

-----

## 5. SPI4 bus — PE12 (SCK) / PE13 (MISO) / PE14 (MOSI) / PE11 (NSS)

|Device                    |Function                                    |Note                                                                                             |
|--------------------------|--------------------------------------------|-------------------------------------------------------------------------------------------------|
|SPI NOR flash             |Discipline/temp logs, web assets, FW staging|CS = PE11; origin: Macronix/Winbond (TW) or ISSI (US) — **not** GigaDevice (PRC)                 |
|ST7796 TFT (3.5”, 320×480)|Status/UI display                           |CS = PE9; write-only (MISO NC); control lines in §8. Reconfigure SPI baud/mode per CS; ~20–40 MHz|
|MCP41U83T-503E/ST digipot |Rb-buck FB-divider trim (VCC_RB set-point)  |CS = PD2 (SPI_DPOT_CS); SPI mode (SPI2C→DGND); NV-wiper, readback on MISO PE13; bounded by fixed resistors (§6). See sts1000_vcc_rb_supply.md|

|STM32 pin|Signal   |Note                         |
|---------|---------|-----------------------------|
|PE10     |NOR_RESET|firmware-controlled NOR reset|
|PD2      |SPI_DPOT_CS|Rb-buck FB digipot chip select (GPIO output, idle high)|

-----

## 6. Power, domains & backup

**Input chain:** PoE (front end is 802.3bt-capable) → magjack center taps (all 4 pairs, Mode A+B) → FDMQ8205A bridge → NCP1095 PD → buck → **3.3 V main**. See PoE classification below.

**Clock rails — fixed OCXO + adjustable Rb buck:**

- **OCXO rail:** fixed, dedicated low-noise rail (3.3 V for the OH300), **always on**, monitored by INA228 #5. No digipots — the OCXO voltage is known and fixed.
- **Rb rail:** a buck (from the PoE/PD intermediate rail down to the Rb module’s voltage, commonly +15–24 V) whose feedback divider is trimmed by a **single SPI digipot (MCP41U83T-503E/ST, `SPI_DPOT_CS` = PD2 on SPI4, §5)**, so the rail is set to match the installed Rb. **Its EN pin is driven by RB_PWR_EN (PB7)** — so firmware enables the Rb rail and thereby *stages its warm-up* separately from the OCXO. Monitored by INA228 #6. Design rules, because a digipot in a buck FB node can otherwise destroy the Rb:
  - **Bound the range with fixed series resistors** so that *no* wiper position — power-on default, mid-scale, or an SPI-fault state — can command a voltage outside the Rb’s safe envelope. The digipot only *trims* within that hard-bounded window; the fixed resistors set the absolute min/max. Primary, firmware-independent safeguard.
  - **NV-wiper digipot** (MCP41U83, terminal B=VREF / A=GND for safe-low power-up) so the rail powers up to a stored known-good value, not mid-scale; OPA320 buffer + MCP1502T-40E 4.096 V reference set the wiper-to-VOUT transfer (VOUT ≈ 4.46 + 0.01941·D). See sts1000_vcc_rb_supply.md.
  - **Hardware OV latch (autonomous):** an LMV393 + regenerative BJT latch trips at **26 V**, disables the buck, and clamps the output (1.5SMCJ28A TVS) with **no MCU action**. Firmware observes via **RB_OV_DET (PE3, polled)** and clears via **RB_OV_RESET (PD3)** — both wired direct to the MCU; **RB_OV is not on the I/O expander** (its annunciation path is PE3). The expander/MCU path is annunciation only — protection is autonomous.
  - **Closed-loop verify:** firmware enables RB_PWR_EN, lets the buck soft-start, then reads back the rail on INA228 #6 and confirms it’s in range before trusting the Rb; out-of-range → drop RB_PWR_EN and flag.
  - Keep wiper / SPI switching noise off the high-impedance FB node (layout).
- **Warm-up staging:** OCXO (fixed rail) comes up first; firmware defers RB_PWR_EN until the OCXO is warm and supercaps are charged, so the Rb’s ~15–20 W warm-up surge doesn’t stack on the OCXO’s — keeping the cold-start peak inside the PoE budget.

**Switched domains (load switch / FET, STM32-controlled, INA228-monitored):**

|STM32 pin|Signal     |Domain                                                                               |
|---------|-----------|-------------------------------------------------------------------------------------|
|PC8      |GPS_PWR_EN |GPS VCC load switch                                                                  |
|PC9      |ANT_BIAS_EN|Antenna bias-T high-side FET                                                         |
|PC11     |DISP_EN    |5 V display rail (RT9742 U56) + touch PCA9306 (U57) EN. **Ex-PERIPH_EN** — the 3V3_P housekeeping rail was removed (those devices now on always-on 3V3_STM); this signal now gates only the display/touch. Default-off via R161 pulldown.|
|PB7      |RB_PWR_EN  |**Rb-rail adjustable buck EN (digipot-set V) — independent enable / warm-up staging**|


> The OCXO rail (fixed, INA228 #5) is **always on**. The Rb rail (adjustable buck, INA228 #6) is **separately enabled** via RB_PWR_EN driving its EN, so it powers up only when wanted — but when on it runs concurrently with the OCXO, and both are monitored at all times.

**Backup managers (TI TPS61094 ×2, auto-backup on input collapse):** both run **strapped/autonomous** — EN and MODE externally tied to the enabled / auto-backup state, so each enters backup automatically on input collapse (the STM32-domain manager can’t wait for firmware while the MCU browns out). Supercaps hold V_BCKP/VBAT through outages → GPS warm start + RTC retention. **No EN/MODE control pins**, but firmware **observes supercap state-of-charge** via ADC: **BKP_GPS_SOC_V (PA6)** and **BKP_STM_SOC_V (PB1)**. The former `BKP_*_EN/MODE` control lines are dropped, freeing PD2/PD3/PE3/PE4 — now reassigned:

|Freed pin|Now            |Function                                                    |
|---------|---------------|------------------------------------------------------------|
|PD2      |SPI_DPOT_CS    |Rb-buck FB digipot chip select (SPI4, §5)                   |
|PD3      |RB_OV_RESET    |Rb OV-latch reset (output)                                  |
|PE3      |RB_OV_DET      |Rb OV-latch state (input, polled)                           |
|PE4      |RB_SER_MODE    |FE-5680A RS-232/CMOS serial relay mode-select (output)      |

**PoE hard-reset (board-level cold cycle):**

|STM32 pin|Signal  |Note                                                        |
|---------|--------|------------------------------------------------------------|
|PE15     |POE_KILL|opens buck-input pass FET; default-RUN via external pulldown|


> Single-signal POE_KILL. Off-time owned by a gate RC biased from the still-energized upstream rail; auto-restores with soft-start. WDT timeout, thermal cutoff, and latched faults OR into this same driver.

**Power budget & recommended PoE classification**

Approximate load (at the 3.3 V rail):

|Block                                         |Steady      |Notes                                      |
|----------------------------------------------|------------|-------------------------------------------|
|OCXO                                          |~1.1–1.5 W  |~3.0–3.8 W during ~5-min warm-up           |
|**Rb (RB_PWR_EN-gated, may run concurrently)**|**~6–10 W** |**~15–20 W during physics-package warm-up**|
|GPS (F9T) + active antenna                    |~0.5–0.7 W  |higher during acquisition                  |
|Ethernet PHY                                  |~0.3 W      |                                           |
|STM32 @ 250 MHz                               |~0.2 W      |                                           |
|RGB ×2, fan, I²C/SPI housekeeping             |~0.5–2 W    |fan ramps with temperature                 |
|Buck/PD conversion loss                       |~15 %       |                                           |
|**Steady-state, OCXO only**                   |**~3.5–4 W**|Rb disabled                                |
|**Steady-state, OCXO + Rb both on**           |**~12–14 W**|                                           |
|**Worst-case peak**                           |**~25–30 W**|concurrent Rb + OCXO cold-start warm-up    |

**Request Type 2 (IEEE 802.3at, ~25.5 W at the PD) as the baseline; Type 3 (802.3bt) for margin.** Because the Rb is permanent and can be enabled concurrently with the OCXO, Class 3 (~13 W) does not cover the load — OCXO+Rb steady is already ~12–14 W and concurrent warm-up peaks ~25–30 W. The bt-capable front end supports this directly; set the NCP1095 classification accordingly (at needs 2-event/LLDP). **Firmware should stagger warm-ups** — bring up the OCXO first, defer RB_PWR_EN until the OCXO is warm and supercaps are charged — to keep the cold-start peak inside the granted budget; if you can’t guarantee staggering, classify **Type 3** so even a simultaneous cold start is covered.

-----

## 7. ADC channels

**External (12-bit, ADC1/2):**

|STM32 pin|Channel|Measures                  |Note                                                            |
|---------|-------|--------------------------|----------------------------------------------------------------|
|PA6      |INP3   |GPS supercap voltage      |/2 divider; state-of-charge/holdup                              |
|PB1      |INP5   |STM32 supercap voltage    |/2 divider                                                      |
|PA3      |INP15  |OCXO steering voltage (Vc)|loop self-test + oscillator-aging trend; no divider (Vc ≤ 3.0 V)|

**Internal (no pins):** VREFINT (true VDDA + droop), die temperature sensor, VBAT sense channel (STM32 backup rail).

-----

## 8. Indicators, fan & local UI

|STM32 pin         |Signal            |Timer/Note                                                                                |
|------------------|------------------|------------------------------------------------------------------------------------------|
|PD12 / PD13 / PD14|RGB LED #1 (R/G/B)|TIM4_CH1/2/3                                                                              |
|PE0               |BTN_INT_N         |in (EXTI0) — button interrupt from MCP23017 U48 INTA (0x21, 8 buttons); see sts1000_fault_aggregation.md|
|PE5               |FAN_PWM           |out, TIM15_CH1 — 25 kHz 4-wire fan control                                                |
|PA15              |FAN_TACH          |in, GPIO/EXTI15 — edge-count for RPM (open-collector, pull-up to 3.3 V)                   |
|PE9               |DISP_CS           |out, GPIO — ST7796 chip select on SPI4                                                    |
|PB0               |DISP_DC           |out, GPIO — ST7796 data/command                                                           |
|PE6               |DISP_BL           |out, TIM15_CH2 PWM — backlight dimming/blanking                                           |
|PA8               |DISP_RST          |out, GPIO — firmware-controlled ST7796 reset; pull resistor for a defined power-on state  |
|PA9               |I2C_BUF_EN        |out, GPIO — enable/recover the I²C buffer (firmware bus isolate/reset); tie high if unused|

**Keypad, touch & presence wake:** the 7 buttons hang off **MCP23017 U48 (0x21)** Port A (GPA0–6) — INTA drives **PE0/EXTI0 (`BTN_INT_N`)** on-change, so a press/release wakes the MCU and firmware reads/debounces over I²C. **GPA7 = `DISP_TOUCH_INT`** (FT6336U) on the same on-change path → **wake-on-touch** is the primary wake. U48 Port B carries the 8 INA228 ALERTs → INTB → **PC12/EXTI12 (`INA_ALERT_INT_N`)**. The PG (8) + EN-fault flags and the **PIR proximity wake** (secondary/ambient) sit on **U47 (0x20)** Port B, INTA+INTB → **PA10/EXTI10 (`PG_INT_N`)**. UI carries **switch + wire only**; per-button **10 kΩ to 3V3 + 100 nF** (~1 ms RC) + connector TVS — **but GPA7 (touch INT) gets the 10 kΩ pull-up to 3V3_STM and NO 100 nF** (the cap would swallow the INT pulse). Both expanders on **3V3_STM** with `EXP_RESET` (PC0) shared. See sts1000_fault_aggregation.md.

**Display module — LCDwiki MSP4030 (ST7796S, 320×480 IPS, FT6336U touch):** pixel/control on SPI4 (SCK PE12 / MOSI PE14), write-only (module SDO left NC, off the NOR MISO). 320×480×16 bpp ≈ 307 KB/frame → 4 Hz ≈ 1.2 MB/s, fine at 20–40 MHz SPI with DMA; partial-region updates bound traffic. The module level-shifts all write/control lines via an onboard **74LVC245 (U2)** — drive them at 3.3 V; LVC Ioff means they don't back-feed the module when its rail is gated. Backlight is an onboard **BSS138** low-side switch (gate pull-up R7 → backlight defaults full-on when DISP_BL floats); **DISP_BL PWM (TIM15_CH2)** drives the gate directly — no external MOSFET needed. **VCC = 5 V `V_DISP_5V`, gated by `DISP_EN` via RT9742 (U56)**, monitored by INA228 @0x42. Touch (FT6336U @0x38) rides the gated 5 V rail behind **PCA9306 (U57)** (EN=DISP_EN, self-isolating); CTP_RST shares DISP_RST (PA8), CTP_INT → U48 GPA7. ESD arrays at the connector: U59 (control) + U58 (SPI) = TPD4E02B04 (3.3 V, 0.25 pF); **U60 (touch I²C + INT) = TPD4E05U06DQAR** (5.5 V VRWM for the 5 V touch lines, DQA drop-in). Full subsystem: sts1000 i2c-peripherals/display doc §6.

**Fan / thermal loop:** firmware drives FAN_PWM from a thermal loop using TMP117 #2 (ambient/enclosure), TMP117 #1 (oscillator), and the STM32 die-temp channel. TACH is a low-frequency signal (~100–200 Hz), so EXTI edge-counting over a 1 s window is sufficient for RPM — no timer-capture channel needed. Notes:

- 4-wire fan: PWM input takes the 25 kHz signal; the STM32’s 3.3 V PWM usually drives it directly (level-shift to 5 V only if a specific fan needs it). Fan motor power comes from a separate 5 V/12 V rail, not the logic pins.
- **Fail-safe to cooling:** a 4-wire fan runs full-speed with no/100 % PWM, so a floating or un-driven FAN_PWM defaults to maximum airflow — a hung MCU can’t let the enclosure cook. Keep that the resting state.
- TACH open-collector output → pull-up to 3.3 V (not 5 V) so PA15 sees clean logic levels.

-----

## 9. Reliability / monitoring

|STM32 pin|Signal  |Dir       |Note                                                           |
|---------|--------|----------|---------------------------------------------------------------|
|PB2      |WDT_KICK|out       |external windowed watchdog refresh; WDT timeout → POE_KILL (HW)|
|PE8      |PFI     |in (EXTI8)|external power-fail comparator (earlier than on-chip PVD)      |
|PC13     |RTC_TAMP|in        |intrusion, backup-domain, timestamped                          |


> Watchdog coverage is the hardware external-WDT → POE_KILL path plus WDT_KICK (PB2); there is no separate firmware WDT-fault observe GPIO.

**PoE controller monitor (NCP1095 status):**

|STM32 pin|Signal |Note                                                                    |
|---------|-------|------------------------------------------------------------------------|
|PC7      |POE_NCL|in (EXTI7) — confirm direction & logic level; level-shift if isolated PD|
|PC2      |POE_NCM|in (EXTI2)                                                              |
|PC3      |POE_LCF|in (EXTI3)                                                              |

-----

## 10. Debug & USB console

|STM32 pin|Signal                                |
|---------|--------------------------------------|
|PA13     |SWDIO                                 |
|PA14     |SWCLK                                 |
|PB3      |SWO (TRACESWO)                        |
|NRST     |dedicated reset                       |
|PA11     |USB_DM                                |
|PA12     |USB_DP                                |
|PE2      |USB_VBUS_SENSE (divider from 5 V VBUS)|

**Out-of-band console = USB CDC-ACM (driverless virtual serial port).** The STM32H563 USB-FS device enumerates as a standard CDC virtual COM port — no host driver on Windows/macOS/Linux — giving a plug-in serial console/management channel independent of the network. Notes:

- USB-FS clock is **HSI48 + CRS** (clock-recovery trimmed against USB SOF) → no external USB crystal.
- Self-powered device (PoE), so VBUS is sensed on PE2 (via divider) to gate the D+ pull-up for compliant enumeration / host-connect detection.
- The console is USB CDC rather than a hardware UART: the only free UART-capable pins (PA11/PA12 → UART4) would collide with the GPS corrections UART4 (PD0/PD1), and CDC is driverless.

-----

## 11. EXTI line allocation (one port per line — verified collision-free)

|Line|Pin |Source         |
|----|----|---------------|
|0   |PE0 |BTN_INT_N (U48 INTA)|
|2   |PC2 |POE_NCM        |
|3   |PC3 |POE_LCF        |
|4   |PD4 |GPS_ANT_OFF_MON|
|5   |PD5 |GPS_TXRDY      |
|7   |PC7 |POE_NCL        |
|8   |PE8 |PFI            |
|10  |PA10|PG_INT_N (U47 INTA+INTB)|
|12  |PC12|INA_ALERT_INT_N (U48 INTB)|
|13  |PB13|RB_LOCK        |
|15  |PA15|FAN_TACH       |

RTC_TAMP (PC13) handled by the RTC/TAMP peripheral, not the EXTI GPIO line. PPS inputs (PA0, PC6) and EXTREF_MON (PB14) are timer-capture (AF), not EXTI. Free EXTI lines: 1, 6, 9, 11, 14. **PA10/EXTI10 = PG_INT_N** (U47 PG+EN-fault) — the former spare is now used. **PC0** = EXP_RESET (GPIO output) sits on EXTI0; as an output it does not contend with PE0. The Rb-input lines PD2 (SPI_DPOT_CS), PD3 (RB_OV_RESET), PE3 (RB_OV_DET, polled), PE4 (RB_SER_MODE) are GPIO/output/polled — none are EXTI sources (PD3/PE3 deliberately avoid the occupied EXTI3, PE4 the occupied EXTI4).

-----

## 12. Reset / recovery matrix

|Target            |Mechanism                                   |Pin(s)          |
|------------------|--------------------------------------------|----------------|
|PHY               |dedicated reset                             |PD10            |
|GPS — soft        |RESET_N                                     |PD11            |
|GPS — hard        |VCC load-switch cycle                       |PC8             |
|GPS — FW recovery |SAFEBOOT_N + RESET                          |PD15 + PD11     |
|Antenna           |bias-T FET                                  |PC9             |
|TMP117 / ATECC608B|rail power-cycle                            |PC11            |
|NOR flash         |dedicated reset (firmware)                  |PE10 (NOR_RESET)|
|ST7796 display    |dedicated reset (firmware)                  |PA8 (DISP_RST)  |
|Whole board (cold)|PoE kill (single-signal)                    |PE15            |
|MCU               |NRST / internal IWDG / external WDT→POE_KILL|NRST, PB2       |
|I/O expanders     |EXP_RESET (both MCP23017); I2C_BUF_EN bus clear. VDD=3V3_STM (not gated → no rail-cycle recovery)|PC0 (PA9)|
|Rb OV latch       |hardware-autonomous trip @26 V; firmware reset after clear  |PD3 (RB_OV_RESET)|

-----

## 13. Spare pins

**None.** The pin map churned through several changes; the last freed pin (PA10) is now used for the PG interrupt, so the board is GPIO-full again:

- **RF front-end collapse (§2.2):** the single-LTC6752xS5 slicer deleted `REF_SLICER_SEL` (freed **PC0**) and the source/termination simplification freed **PA10**. Only `REF_TERM_EN` (PC10) remains committed to the front end.
- **Backup-manager strapping (§6):** the TPS61094 managers run strapped/autonomous, freeing PD2/PD3/PE3/PE4 — all reassigned: **PD2**→SPI_DPOT_CS (§5), **PD3**→RB_OV_RESET, **PE3**→RB_OV_DET, **PE4**→RB_SER_MODE (§6). (SOC sense remains on PA6/PB1.)
- **Fault aggregation (§4):** **PC12**→INA_ALERT_INT_N (U48 INTB) and **PE0**→BTN_INT_N (U48 INTA) — reused interrupt pins; **PA10**→PG_INT_N (U47 PG+EN-fault, R152 pull-up); **PC0**→EXP_RESET (shared expander /RESET — its EXTI0 collision with PE0 is irrelevant for an output).

So both freed pins are now spent — PA10 on the PG interrupt, PC0 on EXP_RESET — and **no uncommitted GPIO remains**.

Status indication is the ST7796 display plus RGB #1 (PD12–14). Firmware has dedicated reset control of the display (PA8), NOR (PE10), GPS (PD11), PHY (PD10), and the I/O expanders (PC0), plus the rail power-cycles and POE_KILL. Further expansion is I²C/SPI-bus-only (I²C1 has address headroom behind the buffer, §4) or requires the 144-pin package.

-----

## 14. Open items to verify before layout

- LAN8742 nINTSEL/REGOFF strap values & LED1/LED2 polarity for REF_CLK-Out, PHY addr matching PB10 strap.
- Whether the ETH PTP auxiliary-snapshot trigger can be driven internally (RM0481) for direct PTP-timebase PPS capture; otherwise correlate TIM2 capture ↔ PTP in software.
- NCL/NCM/LCF direction & logic level; digital isolator required only if PD front end is isolated.
- TPS61094 MODE/EN truth table & auto-backup behavior (TI datasheet).
- INA228 shunt values per expected per-rail currents (incl. OCXO rail #5).
- OCXO loop-filter design: center Vc at 1.65 V nominal, scale DAC 0–3.3 V to the ±0.4 ppm pull, set loop time constant for GPS discipline.
- Set NCP1095 classification for **Type 2 (802.3at) minimum, Type 3 (bt) for margin** (Rb is permanent); confirm staggered-warm-up firmware keeps the cold-start peak within the granted budget.
- Clock mux: **74LVC1G157** selected (S=PB6); glitchlessness via firmware HSI-bridge + CSS (not a glitch-free IC) — see `sts1000_clock_mux.md`; confirm EXTREF_MON frequency-acceptance band and switch hysteresis.
- **External-ref front end (§2.2) — single LTC6752xS5, single SMA input (canonical: `ntp_server_rf_frontend.md`):** the LTC6957 EOL collapsed the two-branch design to one LTC6752xS5 slicer (deletes the second slicer, pad, 2:1 mux) and there is **one SMA clock input** — no DB9 source, no source-select. Remaining work: size Rseries + island clamp and the **R_VCC (33 Ω) + D5 (3.3 V) VCC guard** (S5 base variant = 3.6 V supply abs-max); **source the D4/D5 TVS** (STN061050B101 5 V rejected; STN061033B101 3.3 V conditionally accepted); set the 1.5 V bias on the 3.0 V island; confirm `REF_TERM_EN` (PC10) safe reset default and the 50 Ω-vs-high-Z switch handles both sine/50 Ω and TTL/CMOS from the one SMA. **PA10 and PC0 are freed** (§13).
- **External-Rb connector pinout per FE-5680A variant:** confirm the 1PPS / lock / RS-232 / +15 V / +5 V pin map for the specific unit before layout — these vary across surplus variants. **The 10 MHz does not enter this connector** — it goes to the SMA over coax; route power/PPS/lock/serial to their own nets only.
- **Rb topology — resolved:** external FE-5680A only; no onboard Rb option. 10 MHz via the SMA front end (§2.2), power/serial/lock/PPS via the FE-5680A housekeeping connector. §15 rewritten to match; OCXO is the only onboard reference.
- Rb-rail buck: compute fixed bounding resistors so the digipot-set FB range can never exit the Rb’s safe-voltage envelope (incl. POR / mid-scale / SPI-fault); the FB trim is a **single SPI digipot (MCP41U83T-503E/ST, `SPI_DPOT_CS` = PD2, §5)** — resolve the remaining datasheet TBDs (pin numbers, A0/A1 tie in SPI mode, wiper-write opcode/frame, CRC default, SPI CPOL/CPHA, power-on wiper value); tie buck EN to RB_PWR_EN; define firmware read-back-verify thresholds on INA228 #6. Hardware OV latch is autonomous (26 V); RB_OV_DET=PE3 (polled), RB_OV_RESET=PD3.
- USB: confirm HSI48 + CRS config for the FS device; PE2 VBUS-sense divider values; CDC-ACM descriptor (driverless on host); ESD protection on the USB connector.
- Display/touch (MSP4030): backlight is an onboard BSS138 (no external MOSFET); module 74LVC245 level-shifts control lines (drive 3.3 V). **Resolved:** U60 = TPD4E05U06DQAR (5.5 V, 5 V touch lines); DISP_EN default-off pulldown R161 present; PCA9306 SCL1/SDA1 share the one main-bus pull-up pair; FT6336U 0x38 (no strap dependence). **Open:** RT9742 inrush vs I_LIM + 0.1 Ω shunt headroom (measure module backlight-full current + Cin).
- I²C buffer/accelerator: pick LTC4311 (accelerator) vs PCA9517/TCA4311 (segmenting buffer) per measured bus Cb; confirm 400 kHz timing margin; decide whether I2C_BUF_EN (PA9) is driven or tied high.
- E-compass: maximize distance from Rb/OCXO/bucks/PoE magnetics/fan; in-enclosure hard/soft-iron calibration (store once for the fixed install); WMM/IGRF declination from the GPS fix for true north; tilt-comp via the accelerometer.
- Keypad/touch & power-fault aggregation (`sts1000_fault_aggregation.md`), as-built: **U47 @0x20** PortA 8×PG; PortB [0] V_ANT_EN_FAULT, [1] freed (3V3_P EN-fault retired), [2] V_DISP_EN_FAULT, [3] PROX_WAKE (PIR), [4–7] spare; **MIRROR=1**, INTA+INTB tied → **PG_INT_N**→PA10 (R152). **U48 @0x21** PortA GPA0–6 = 7 buttons, **GPA7 = DISP_TOUCH_INT** → INTA→**BTN_INT_N**→PE0 (R148, GPA7 no debounce cap); PortB 8×INA ALERT → INTB→**INA_ALERT_INT_N**→PC12 (R153). Both VDD 3V3_STM; shared EXP_RESET (PC0, R154 pull-down). **Drop legacy R44 at PC12**. Fix net label `PG_IN_N`→`PG_INT_N`. Per-bit IPOL asserted=1; latched-ALERT on all 8 INA228; faults level (compare-to-DEFVAL); buttons + touch on-change. RB_OV excluded (PE3/PD3). Proximity wake is the PIR on U47 PortB; touch is the primary UI wake.
- **PE3 owner conflict:** §6 now assigns PE3 = RB_OV_DET; confirm no residual reference to PE3 as a backup-manager line elsewhere in firmware/schematic.
- NOR is now on always-on 3V3_STM (3V3_P rail removed) — recovery is via `NOR_RESET` (PE10) only, no rail-cycle path. Confirm that's acceptable for the chosen NOR.

-----

## 15. Rubidium (Rb) reference — external FE-5680A, independently enabled

**There is no onboard Rb option.** The system has exactly two references: the **onboard OCXO** (always on, STM32-steered) and an **external FE-5680A Rb**. The Rb's **10 MHz enters via the single SMA front end (§2.2)**; its power/serial/lock/PPS run on the FE-5680A housekeeping connector (§2.2). The Rb rail is gated by **RB_PWR_EN (PB7)** so firmware brings it up independently of the OCXO; when enabled it runs concurrently and both are monitored at all times (INA228 #5 = OCXO, #6 = Rb). The clock mux (§2.1) selects which reference feeds PH0.

### 15.1 Concept

Unlike the OCXO — which the STM32 actively steers through the PA4 DAC loop — the external FE-5680A is a self-contained **GPS-disciplined flywheel (GPSDRO)**: it takes a GPS 1PPS input and locks its own 10 MHz to it over a long time constant. When the Rb is enabled and selected:

- The Rb’s 10 MHz becomes the *selected* system reference (mux **input B** → PH0 + bench fanout).
- The **OCXO keeps running as a live, already-warm fallback** (mux input A) — it is never powered down; the STM32 only changes which source the mux forwards.
- The **STM32 demotes** from “steer the oscillator” to “feed PPS, read health, cross-check GPS, and select the mux.” The PA4 DAC loop goes **idle** (optionally it can trim the Rb’s analog EFC, but a good Rb self-disciplines better than a firmware loop).

### 15.2 What it buys / when to enable it

|                 |Holdover < 1 µs|Holdover < 1 ms|Bench 10 MHz |
|-----------------|---------------|---------------|-------------|
|OCXO (always on) |~hours         |~1 day–2 wk    |very good    |
|Rb (when enabled)|**~1 day**     |**~months**    |**lab-grade**|

The external Rb, when connected, can have its rail left off by firmware to save power and enabled only when warranted: a genuine multi-day GNSS-denied holdover need, or when a lab-grade phase-noise/Allan-deviation 10 MHz is wanted on the bench outputs. For normal LAN-NTP service (network jitter is the floor) and minutes-to-hours outages, the OCXO alone keeps holdover error invisible to clients — running the Rb then is holdover you won’t spend (and ~6–10 W you needn’t burn).

### 15.3 Rb subsystem connections (external FE-5680A)

Pins are allocated in §2.1; the Rb subsystem uses:

|Signal         |STM32 pin               |Purpose                                                              |
|---------------|------------------------|---------------------------------------------------------------------|
|EXT_REF_IN     |SMA → §2.2 → mux input B |Rb 10 MHz over coax into the SMA front end, then the clock mux       |
|GPS_PPS_OUT    |(buffered F9T TIMEPULSE)|→ Rb 1PPS-in for self-discipline (no new STM32 pin)                  |
|RB_UART_TX / RX|PB4 / PE7 (UART7)       |Rb control + telemetry (lock, freq offset, health)                   |
|RB_LOCK        |PB13 (EXTI13)           |lock-good status → drives auto-switch                                |
|RB_PWR_EN      |PB7                     |**drives the Rb-rail buck EN — independent enable / warm-up staging**|
|INA228 #6      |I²C1 0x47 (PB8/PB9)     |independent Rb-rail power monitor (always reporting)                 |

### 15.4 Power / PoE

- The Rb runs on a **digipot-adjustable buck** (from the PoE/PD rail down to the FE-5680A's voltage — **+15 V**, plus **+5 V** on some variants; the buck's 5–24 V range covers variant spread), ~6–10 W steady and ~15–20 W during physics-package warm-up. **RB_PWR_EN (PB7) drives the buck’s EN**, so the same signal that enables the Rb stages its warm-up. A **single SPI digipot (MCP41U83, `SPI_DPOT_CS` = PD2, §5)** sets the FB-divider output (bounded by fixed resistors; autonomous 26 V OV latch; verified on INA228 #6 — see §6).
- **PoE classification (see §6):** because the Rb is permanent and can run concurrently with the OCXO, the board classifies **Type 2 (802.3at) minimum, Type 3 (bt) for margin** — the bt-capable front end supports this. Firmware staggers warm-ups (OCXO first, defer RB_PWR_EN) to keep the cold-start peak inside the granted budget.
- **Independent monitoring:** OCXO on **INA228 #5 (0x46)**, Rb on **INA228 #6 (0x47)** — both report continuously, so the STM32 always sees the health of both references and only manages mux selection.

### 15.5 Operation (firmware)

1. OCXO always on; Rb rail enabled via RB_PWR_EN when wanted. Boot on OCXO (mux A); both references monitored (INA228 #5 + #6) whenever powered.
1. **Switch OCXO → Rb only when both checks pass:** EXTREF_MON (TIM12 / PB14) confirms a real, in-band ~10 MHz on input B **and** RB_LOCK (PB13) is asserted — held stable past a debounce window.
1. **Switch Rb → OCXO immediately if either fails** — RB_LOCK de-asserts (Rb unlocked) **or** the EXTREF_MON clock check drops out. Fallback is fail-safe; hysteresis on re-engage prevents flapping.
1. Both references are 10 MHz → PLL config unchanged; **74LVC1G157 mux + firmware HSI bridge (CSS failsafe)** across the switch (§2.1).
1. STM32 keeps capturing GPS PPS (PA0) to cross-check the active reference against GPS and flag drift via UART7; advertises stratum/holdup based on the active reference and its lock/warm state.

### 15.6 Reference unit

- **External FE-5680A** (Frequency Electronics) — 10 MHz Rb, 1PPS-disciplinable, RS-232/CMOS serial control/telemetry, ~+15 V supply (+5 V on some variants). Connects via coax (10 MHz → SMA, §2.2) plus the housekeeping connector (power/serial/lock/PPS). Surplus variants differ in connector pinout and serial level — **verify the specific unit before layout** (§14).
- No onboard Rb/atomic module is fitted; the only onboard reference is the OCXO (§2).
- Per sourcing policy, the external unit and any replacement are non-Chinese-origin.