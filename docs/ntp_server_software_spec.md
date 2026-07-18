# PoE GPS-Disciplined Stratum-1 NTP/PTP Server — Software & Firmware Specification

**Target:** STM32H563ZIT6 (Cortex-M33 @ 250 MHz, 2 MB flash, **LQFP144**, TrustZone + PSA crypto, ETH-MAC with IEEE-1588 hardware timestamping).
**Companion documents:** *Firmware ↔ Hardware Interface Reference* (`sts1000_firmware_hardware_interface.md`) is the **authoritative pin contract** — the complete as-built 144-pin map of U12, bring-up order, the GPIO scan model, and the confirmed hardware defects that gate firmware. *Peripheral & Pin Map* (`ntp_server_peripheral_map.md`) and *Hardware Design Reference* (`sts1000_hardware_design_reference.md`) cover electrical facts. Every pin/net cited here uses the canonical net names. Where any disagree on a designator or pin, **the interface reference (as-built netlist) wins**; this document owns software behavior.

This spec is prescriptive: it states the chosen design, then the rationale. Section 2 maps the software role of **every** STM32 connection. Sections 3–10 define behavior. Section 15 lists the decisions still open.

-----

## 0. Conventions

- **MUST / SHALL** = mandatory; **SHOULD** = strong default, deviation requires justification; **MAY** = optional.
- Pin references are `NET_NAME (Pxx)` per the pin map.
- “The loop” = the clock-disciplining control loop (§3.3); “the active reference” = whichever oscillator the clock mux is forwarding to `PH0`.
- Times in UTC; all internal timestamps are TAI-based with a tracked leap-second offset.

-----

## 1. System architecture

### 1.1 Platform stack (chosen)

|Layer                  |Selection                                                                                                                |Rationale                                                                                                                                                                                                                                                         |
|-----------------------|-------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|RTOS / platform        |**Zephyr RTOS**                                                                                                          |Devicetree expresses hardware intent 1:1 with the pin map; first-class MCUboot, settings/NVS, LittleFS, USB CDC-ACM, shell, mbedTLS/PSA, native IP stack with gPTP, and in-tree drivers for most of the BOM (TMP117, INA2xx, IIS2MDC, LIS2DH12, ST7796, FT5xx6/FT6336, SPI-NOR).|
|Secure boot root       |**STM32H5 STiRoT** (immutable ST root of trust) → **OEMuRoT/MCUboot**                                                    |Hardware-anchored chain to a signed, anti-rollback application.                                                                                                                                                                                                   |
|Bootloader / image mgmt|**MCUboot** (swap-move, two slots) + **MCUmgr/SMP**                                                                      |Signed A/B images, confirm/revert, serial recovery over the USB console.                                                                                                                                                                                          |
|Crypto                 |**PSA Crypto** over **mbedTLS 3.x**, H5 AES/PKA/HASH accelerators, **ATECC608B** secure element (Microchip CryptoAuthLib)|TLS 1.3, NTS AEAD, SNMPv3, firmware-signature verify; ECC P-256 keys held in the ATECC608B where exportability must be denied.                                                                                                                                    |
|TCP/IP                 |Zephyr native stack (dual-stack IPv4/IPv6), `SO_TIMESTAMPING` to the ETH-MAC PTP unit                                    |Hardware RX/TX timestamps are the basis of NTP/PTP accuracy.                                                                                                                                                                                                      |
|Filesystem             |**LittleFS** on SPI-NOR (§2.5) + **NVS** in internal flash for small critical settings                                   |Power-loss-safe; NOR holds web assets, logs, almanac, calibration, NTS keys.                                                                                                                                                                                      |
|TLS web server         |Embedded HTTPS (Zephyr `http_server`) serving a static SPA + REST + WSS                                                  |§5.                                                                                                                                                                                                                                                               |

**Two honest gaps and how they’re filled (custom application modules, not off-the-shelf):**

1. **NTP/NTS server and PTP grandmaster** — Zephyr has clients and gPTP (802.1AS) but no production NTS-capable NTP *server* nor a turnkey 1588 grandmaster. These are implemented as application services (§4) over the socket API with hardware timestamping. This is the largest software effort in the project; budget for it.
1. **SNMP agent** — no maintained Zephyr agent. Implement a compact SNMPv2c/v3 agent as an app module (§5.3), or port the lwIP `apps/snmp` agent.

**Fallback platform:** FreeRTOS + lwIP + mbedTLS + MCUboot. Trade devicetree elegance and Zephyr’s driver set for lwIP’s *built-in* SNMP agent and ST’s reference H5 ETH/PTP examples. Choose this only if the SNMP/NTP-server custom effort on Zephyr proves larger than the lwIP path; the rest of this spec is platform-neutral at the behavior level.

### 1.2 Thread/task model (real-time partitioning)

The timing path is isolated from the management/network load. Hardware timestamping in the MAC means served-time accuracy is **not** hostage to TLS/web CPU load, but the disciplining control must still never be starved.

|Thread                  |Prio (Zephyr, lower = higher)|Period / trigger               |Role                                                                             |
|------------------------|-----------------------------|-------------------------------|---------------------------------------------------------------------------------|
|`pps_capture` ISR + work|ISR + cooperative            |GPS PPS edge (PA0 TIM2 capture)|Latch capture, apply sawtooth correction, post phase sample. Must complete in µs.|
|`discipline`            |4                            |1 Hz (PPS-paced)               |PI/FLL loop; DAC update (PA4); reference/holdover state machine.                 |
|`gnss`                  |6                            |UART3 RX + 1 Hz                |Parse UBX; survey-in; NAV-SAT; antenna supervisor; leap-second.                  |
|`ptp`                   |5                            |event                          |1588 engine; BMCA; Sync/Follow_Up/Delay. Time-critical, above net-app.           |
|`ntp_server`            |8                            |socket                         |NTP/NTS request service; RX/TX HW timestamps.                                    |
|`net_mgmt`              |10                           |event                          |DHCP, mDNS, link, ND/ARP.                                                        |
|`web` / `tls`           |12                           |socket                         |HTTPS, REST, WSS. Lowest network prio.                                           |
|`snmp`                  |12                           |socket                         |Agent + traps.                                                                   |
|`io_scan`               |11                           |1 kHz                          |Direct-GPIO port scan (GPIOF/GPIOG IDR snapshot + XOR-diff): buttons, EN-faults, PG rails, INA228 ALERTs. Replaces the deleted MCP23017 expanders. See interface ref §5.|
|`housekeeping`          |14                           |1–4 Hz                         |I²C sensors (INA228/TMP117/e-compass), fan loop, supercaps.                 |
|`ui_local`              |15                           |4–10 Hz                        |Display render, button/nav, RGB, proximity.                                      |
|`console`               |14                           |CDC/UART RX                    |Shell, log tail, diagnostics, SMP recovery.                                      |
|`logger`                |16                           |queue                          |Async structured logging to RAM ring + NOR + syslog.                             |

Rule: no `web`/`snmp`/`console`/`ui` thread may hold a mutex on timing state; they read lock-free snapshots (double-buffered telemetry block published by `discipline` at 1 Hz).

### 1.3 Memory & boot

- Internal 2 MB flash: STiRoT region → MCUboot → **slot-0 (active app)** / **slot-1 (staged)**; NVS partition for critical settings; protected region for anti-rollback counters and the device key handle.
- SPI-NOR (§2.5): LittleFS — web SPA bundle, GNSS almanac/ephemeris cache, calibration tables, discipline/health log ring, NTS master keys (sealed), syslog spool, MCUboot image staging if internal slot-1 is undersized.
- Boot sequence: STiRoT verifies MCUboot → MCUboot verifies app signature + anti-rollback → app. A failed verify halts in MCUboot recovery (SMP over USB). First boot of a new image is **test** until firmware self-confirms (§8.3), else automatic revert.

-----

## 2. Hardware-intent mapping (software role of every connection)

This is the contract between firmware and the board: for each net, what the software does with it and how it behaves. Grouped by subsystem; pin facts per the pin map.

### 2.1 Timing core

|Net (pin)                                                            |Peripheral                     |Software role & behavior                                                                                                                                                                                                                                                                                                      |
|---------------------------------------------------------------------|-------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|`OSC_IN` (PH0)                                                       |HSE bypass ← mux output        |Disciplined 10 MHz system reference. PLL configured **identically** for OCXO or Rb (both 10 MHz). On a mux switch, firmware bridges SYSCLK to HSI, reselects HSE, re-locks PLL (§3.5).                                                                                                                                        |
|`OCXO Vc` (PA4)                                                      |DAC1_OUT1                      |Control output of the OCXO loop. 0–3.3 V scaled to the ±0.4 ppm pull; **center 1.65 V**. Slew-limited; never written from any thread but `discipline`. On holdover the last good value is frozen. When Rb is the active reference the DAC holds OCXO at its last disciplined value (keeps the fallback warm and near-correct).|
|`OCXO Vc sense` (PA3)                                                |ADC1_INP15                     |Loop self-test + aging trend. Read each control tick; compare commanded vs sensed Vc (open-loop/DAC-fault detection); long-term drift of Vc-at-lock = oscillator aging metric.                                                                                                                                                |
|`GPS PPS` (PA0)                                                      |TIM2_CH1 input capture (32-bit)|Primary phase reference. Capture timestamp of each GPS 1PPS against the reference-clock-derived timer; sawtooth-corrected (§3.2). The single most important measurement in the system.                                                                                                                                        |
|`GPS TIMEPULSE2` (PC6)                                               |TIM3_CH1 input capture         |Redundant/aux PPS: cross-check against PA0, detect receiver PPS anomalies, and serve as the F9T cable-delay differential reference.                                                                                                                                                                                           |
|`MUX_SEL` (PB6)                                                      |GPIO out                       |Reference select: 0 = OCXO (boot default), 1 = Rb. Driven only by the reference state machine (§3.5) with the glitchless sequence.                                                                                                                                                                                            |
|`EXTREF_MON` (PB14)                                                  |TIM12_CH1 capture              |Rb-clock presence + frequency gate. Declares input-B “valid” only when edges advance **and** measured frequency ∈ 10 MHz ± band. Precondition for switching to Rb; loss forces immediate revert.                                                                                                                              |
|`RB_LOCK` (PB13)                                                     |GPIO in, EXTI13                |Rb lock-good. Second precondition for using Rb; de-assert → immediate fail-safe revert to OCXO. Debounced.                                                                                                                                                                                                                    |
|`RB_PWR_EN` (PB7)                                                    |GPIO out → buck EN             |Enables/stages the Rb rail. Sequenced after OCXO warm + supercaps charged (PoE budget, §10.3). Drives closed-loop rail verify on INA228 #6 before the Rb is trusted.                                                                                                                                                          |
|`RB_UART_TX/RX` (PB4/PE7)                                            |UART7                          |Rb control + telemetry: poll lock, frequency offset, EFC, temperature, health; optionally trim EFC. Device-side net naming — PB4 is the MCU’s TX into the Rb’s RX.                                                                                                                                                            |
|`10MHz_RF_IN` (SMA J8) → single LTC6752 slicer (U50) → mux input B, `1PPS_OUT` (J15, buffered ETH PTP)|(board nets)                   |Input B is driven by the **external-reference front end**: the FE-5680A's 10 MHz over coax to the SMA, sliced by U50 to 3.3 V CMOS (switched 50 Ω term, §2.1 `REF_TERM_EN`). The FE-5680A runs standalone; firmware does not steer it (§3.4). `EXTREF_MON` (PB14/TIM12) validates the conditioned output before the reference SM may select B.|
|`REF_TERM_EN` (PC10)                                                 |GPIO out                       |Direct control of the §2.2 front end (no expander, no build options — all hardware populated). As-built the front end is a **single LTC6752 slicer (U50)** on the `10MHz_RF_IN` SMA with a **switched 50 Ω termination** (U49 TMUX1101, `REF_TERM_EN`=PC10, default **terminated** via R176 100k↑) — there is no runtime source-select or dual-slicer route. **Set-once** from the stored profile at boot, re-applied only on a source/clock-type change; never in the hot path. Apply → wait settle → require `EXTREF_MON` valid before the reference SM may select B.|

### 2.2 GNSS — ZED-F9T

|Net (pin)                                                        |Peripheral    |Software role                                                                                                                                                                           |
|-----------------------------------------------------------------|--------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|`USART3_TX/RX` (PD8/PD9)                                         |USART3        |Primary control/telemetry @ 460800 8N1. UBX binary protocol (NMEA disabled). Carries config, NAV-PVT/NAV-SAT/NAV-SIG/TIM-TP/MON-RF, and is the **F9T firmware-update transport** (§8.5).|
|`UART4_TX/RX` (PD1/PD0)                                          |UART4         |RTCM3 corrections input (RTK/precise timing) — optional; from an NTRIP client over the network or a local base.                                                                         |
|`GPS PPS` (PA0) / `TIMEPULSE2` (PC6)                             |TIM2/TIM3     |See §2.1. Configured via UBX-CFG-TP5: PPS1 = 1 Hz aligned to UTC, locked-only; PPS2 = higher-rate aux if useful.                                                                        |
|`GPS_RST_N` (PD11)                                               |GPIO out      |Soft reset; part of FW-recovery combo.                                                                                                                                                  |
|`GPS_SAFEBOOT_N` (PD15)                                          |GPIO out      |Hold low + reset → F9T safeboot for firmware recovery (§8.5).                                                                                                                           |
|`GPS_DSEL` (PD7)                                                 |GPIO out      |Hold the receiver in UART+I²C interface mode at boot.                                                                                                                                   |
|`GPS_EXTINT` (PD6)                                               |GPIO out      |External interrupt/time-mark trigger to the F9T (e.g., aiding, time-mark requests).                                                                                                     |
|`GPS_TXRDY` (PD5)                                                |GPIO in, EXTI5|TX-ready wake for the parser. **F9T pin-19 default function = GEOFENCE_STAT; firmware MUST re-`CFG` it to TX_READY before trusting PD5.**                                                  |
|`GPS_ANT_OFF_MON` (PD4)                                          |GPIO in, EXTI4|Observes LNA-disable; corroborates UBX-MON-RF antenna state and the antenna-rail INA228 for the supervisor (§3.7).                                                                      |
|GPS VCC switch (PC8), antenna bias (PC9), V_BCKP backup           |—             |See §2.6. V_BCKP is held by the autonomous TPS61094 U34 (+supercap) and **must** stay up through GPS VCC cycles for warm start; firmware monitors it via `BKP_GPS_PG` (PF15), it has no MCU enable.|

### 2.3 Ethernet & PTP timestamping

|Net (pin)                                                                                                     |Peripheral                                 |Software role                                                                                                                                                                                                                                                                                                                                                               |
|--------------------------------------------------------------------------------------------------------------|-------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|RMII bus (PA1 REF_CLK, PA2 MDIO, PC1 MDC, PA7 CRS_DV, PC4/PC5 RXD0/1, PA5 TX_EN, PB12/PB15 TXD0/1, PB10 RX_ER)|ETH-MAC (RMII)                             |L2 transport. MDIO driver manages LAN8742 (link, autoneg, energy-detect). RMII 50 MHz domain is **independent** of the disciplined reference — by design.                                                                                                                                                                                                                   |
|`ETH_PPS_OUT` (PB5)                                                                                           |ETH PTP PPS                                |MAC PTP pulse-per-second output: scope/verification tap and a hardware cross-check that the PTP clock tracks the disciplined timebase.                                                                                                                                                                                                                                      |
|`LAN_RST_N` (PD10)                                                                                            |GPIO out                                   |PHY reset/recovery.                                                                                                                                                                                                                                                                                                                                                         |
|(internal)                                                                                                    |ETH PTP timestamp unit + auxiliary snapshot|Hardware RX/TX timestamps for NTP/PTP; PTP clock is **syntonized** to the disciplined reference and **synchronized** (offset/skew steered) to GPS time via the discipline engine. Auxiliary-snapshot capture of the GPS PPS is the preferred PPS↔PTP correlation if RM0481 allows internal trigger (§14 open item); otherwise correlate TIM2 capture ↔ PTP-time in software.|

### 2.4 I²C1 sensor / secure / HMI bus (PB8 SCL / PB9 SDA)

400 kHz, behind the **LTC4311 U56** rise-time accelerator (EN tied permanently to 3V3_STM — nothing to sequence). **15 devices, one bus, no address collisions.** All housekeeping peripherals sit on always-on **3V3_STM** (no gated peripheral rail — gating a rail whose I²C pull-ups sit on an always-on rail would back-power the parts through their SDA/SCL clamp diodes). Each INA228 ALERT is a **separate GPIO** (GPIOG/PF13, active-low, scanned — see §2.4a and interface ref §5), **not** a shared open-drain line.

|Device (addr, ref)              |Software role & cadence                                                                                                                                                    |
|--------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|INA228 #1 PoE in (0x40, U10)    |Input power/PoE budget accounting; class-vs-draw sanity; brownout precursor. 4 Hz. R30 sits in the series path → voltage + current both usable.|
|INA228 #2 STM 3V3 (0x41, U31)   |MCU rail health. 1 Hz.                                                                                                                                                     |
|INA228 #7 5V_DISP (0x42, U32)   |Display 5 V rail. 1 Hz.                                                                                                                                                    |
|INA228 #8 main 3V3 (0x43, U30)  |Main/general 3V3 rail; 3V3-good telemetry corroborating PG4 (AP3441 PG drives to VIN when good → ~2.98 V). 1 Hz.                                    |
|INA228 #4 Antenna (0x45, U26)   |**Antenna supervisor**: open (low I) / short (high I) / OK; cross-checks UBX-MON-RF + PD4. 4 Hz.                                                                           |
|INA228 #5 OCXO (0x46, U37)      |Warm-up vs steady; oven-fault detection; always reporting. 4 Hz during warm-up, 1 Hz steady.                                                                               |
|INA228 #6 Rb (0x47, U44)        |**Closed-loop verify** of the digipot-set Rb rail before trusting Rb; continuous Rb health. 4 Hz. ALERT (PG15) pull-up R265 10k→3V3_STM.           |
|SHT45 humidity (0x44, U72)      |Enclosure humidity/temp. **Owns 0x44** — this is why the GPS INA228 is at 0x4A. 1 Hz.                                                                                       |
|INA228 #3 GPS (0x4A, U23)       |GPS load; acquisition-vs-tracking signature. **0x4A, not 0x44** (A0=A1=SDA strap; 0x44 is the SHT45). Do **not** re-strap. 1 Hz.                                            |
|INA228 #9 panel 5V (0x4C, U54)  |Panel-LED string health; average over ≫ the PWM period, duty-normalize (I_true ≈ I_avg/duty). ALERT on **PF13**. 4 Hz. §6.4.                                                |
|TMP117 #1 oscillator (0x49, U57)|OCXO/Rb local temperature → aging compensation, thermal alarms. ADD0→3V3_STM. 1 Hz.                                                                                        |
|TMP117 #2 ambient (0x48, U58)   |Enclosure temp → fan loop setpoint. ADD0→GND. 1 Hz.                                                                                                                        |
|ATECC608B (0x60, U60)           |Device identity, TLS/NTS ECC P-256 private keys (non-exportable), ECDSA sign, ECDH, SHA-256/HMAC, AES-128, hardware TRNG, two monotonic counters, attestation. §9.         |
|IIS2MDC (0x1E, U61)             |Magnetometer heading for skyplot true-north; polled 1–5 Hz; calibrated constants applied. INT unconnected → poll. §6.3.                                                    |
|LIS2DH12 (0x19, U59)            |Accel tilt for skyplot tilt-compensation. **SA0→3V3_STM ⇒ 0x19.** Poll. §6.3.                                                                                              |
|FT6336U touch (0x38)            |Capacitive touchscreen; INT is **direct on `DISP_TOUCH_INT` (PF7)** (scanned/EXTI7) — read+dispatch. On the `DISP_EN`-gated 5 V rail behind PCA9306 U63. §6.5.              |

The Rb digipot **U43 (MCP41U83)** is **not** on this bus — it is on **SPI4** (SPI mode, CS `SPI_DPOT_CS`/PD2); see §2.5/§3.4 and interface ref §9. The presence sensor is a **passive magnetic reed switch** (dry contact, no Vcc) read as a **discrete GPIO** on `PROX_WAKE`/PF10 (EXTI10, J17.18) with pull-up R254 to always-on 3V3_STM, not on I²C. §6.4.

### 2.4a Direct-GPIO scan model

All HMI inputs, load-switch fault flags, rail power-goods, and INA228 ALERTs are **direct GPIO**
(no I/O expanders), serviced by the 1 kHz `io_scan` thread
(§1.2) that snapshots **GPIOF IDR** and **GPIOG IDR**, XOR-diffs against the previous sample, and
dispatches changed bits. The rotary encoder is **not** scanned (hardware TIM1 mode). Authoritative
bit maps live in `sts1000_firmware_hardware_interface.md` §5; summary:

- **GPIOF[0:6]** = BUTTON_1..7 (active-low); **[7]** DISP_TOUCH_INT; **[8]** `V_ANT_EN_FAULT_N`,
  **[9]** `V_DISP_EN_FAULT_N`, **[12]** `PANEL_LED_FAULT_N` (RT9742 open-drain nFLG, active-low);
  **[10]** PROX_WAKE (also EXTI10); **[11]** ENC_BUTTON; **[13]** `INA_ALERT_5V_PANEL` (INA228 #9
  panel 0x4C — the one INA alert **not** on GPIOG); **[14]** BKP_STM_PG, **[15]** BKP_GPS_PG.
- **GPIOG[0:7]** = PG rails (active-high "good"): PG0 3V3_GPS_LDO_PG, PG1 OCXO_LDO_PG, PG2
  3V0_RF_LDO_PG *(mask until the LT3045 open-collector pull-up is fitted)*, PG3 5V_PSU_PG,
  PG4 3V3_PSU_PG *(~2.98 V)*, PG5 OCXO_PSU_PG (÷ divider), PG6 RB_PSU_PG, PG7 POE_PG
  *(divided to 2.93 V)*.
- **GPIOG[8:15]** = INA228 ALERTs (active-low): PG8 V_POE, PG9 3V3_STM, PG10 5V_DISP, PG11 3V3,
  PG12 3V3_GPS (0x4A), PG13 V_ANT, PG14 OCXO, PG15 VCC_RB *(pull-up R265 10k→3V3_STM)*.

Firmware **MUST mask PG2** in the scan until the `3V0_RF_LDO_PG` pull-up is fitted. On any INA-alert
bit asserting, read that INA228's diagnostic/flags register for the cause, then run alarm evaluation.

### 2.5 SPI4 (PE12 SCK / PE13 MISO / PE14 MOSI / PE11 NSS)

|Device                                                                     |Software role                                                                                                                                                            |
|---------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|SPI-NOR U62 (CS `SPI_NSS` PE11, `NOR_RST_N` PE10)                          |MX25L25645 (Macronix). LittleFS backing store (web assets, logs, almanac, calibration, NTS keys, FW staging). DMA reads for asset serving. **`NOR_RST_N` boots asserted (R205 10k↓) — drive PE10 HIGH early in bring-up** or the NOR stays held in reset.|
|Rb digipot U43 (CS `SPI_DPOT_CS` PD2)                                      |MCP41U83, **SPI mode 0,0**. Rb-buck FB trim, **hard-bounded by fixed resistors** (§3.4). Written only during commissioning/calibration with INA228 #6 read-back verify. Code 0x000 = Terminal B = safe-low VCTRL; POR = midscale (~14.5 V, inside the 26 V OV envelope). Pre-program the NV wiper safe-low before any `RB_PWR_EN`. Wiper-write opcode/CRC per DS20007000B.|
|ST7796 TFT (CS PE9, `DISP_DC` PB0, `DISP_BL` PE6, `DISP_RST` PA10)         |Local GUI on MSP4030 module. Write-only (MISO NC). Separate SPI baud/mode profile (20–40 MHz) per-CS; MOSI/SCK 22 Ω series-isolated (R210/R211). Partial-region updates; DMA frame pushes. Backlight PWM (onboard BSS138) for dim/blank. VCC = `5V_DISP`, gated by `DISP_EN`; touch (FT6336U) shares the rail (§2.4/§6.5). `DISP_RST` (PA10, held low at boot by R209 10k↓) also resets the touch controller. §6.5.|

### 2.6 Power, domains, backup, kill

|Net (pin)                  |Software role                                                                                                                                                                                                 |
|---------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|`GPS_PWR_EN` (PC8)         |GPS VCC load switch — hard power-cycle path for the receiver (warm start preserved by V_BCKP).                                                                                                                |
|`ANT_BIAS_EN` (PC9)        |Antenna bias-T high-side enable; part of the antenna fault response (disable on persistent short).                                                                                                            |
|`DISP_EN` (PC11)           |5 V display rail (`5V_DISP`, RT9742 U33) + touch PCA9306 U63 enable — gates only display+touch (TMP117/ATECC608B/NOR/e-compass are on always-on 3V3_STM). Default-off at boot (R104 10k↓); cycle to cold-restart a wedged panel/touch. Mask `V_DISP_EN_FAULT_N` during soft-start. The display/touch/encoder are fed from the J17 panel-power pins (J17.16→5V_DISP, J17.28→3V3_STM); confirm both rails reach the panel before Stage-6 bring-up.|
|`PANEL_LED_EN` (PC0), `PANEL_LED_PWM` (PE0)|Panel-LED 5 V RT9742 U55 enable (default-off, R198 10k↓) + brightness PWM. Both stages default OFF (PE0 Hi-Z = Q23 base pull-down). Health via INA228 #9 (0x4C) duty-normalized + `PANEL_LED_FAULT_N` (PF12). §6.4.|
|`RB_PWR_EN` (PB7)          |Rb-rail buck (U40) EN — default-off (R164 100k↓); see §2.1/§3.4/§10.3 sequencing and INA228 #6 verify.                                                                                                        |
|`RB_VCC_GATE` (PB1)        |Rb VCC disconnect gate (Q25 SI7469DP) drive — default-off (R262 pd). Connects `VCC_RB_G` to the FE-5680A **only after** `VCC_RB` (INA228 0x47) verifies in-window. Gate clamp D25 (BZX84C12) bounds \|Vgs\|.|
|`RB_OV_DET` (PE3, polled) / `RB_OV_RESET` (PD3)|Autonomous **26 V OV latch** status/reset. `RB_OV_DET` high = tripped (latch already killed U40 in hardware); pulse `RB_OV_RESET` HIGH to clear after the cause is gone. §3.4.|
|Supercap backup (autonomous)|TPS61094 U34 (GPS V_BCKP) / U35 (STM VBAT) are **strapped autonomous** (EN/MODE tied) — they back up in hardware on input collapse, **no firmware enable**. Firmware monitors only via `BKP_GPS_PG` (PF15) / `BKP_STM_PG` (PF14).|
|`POE_KILL` (PE15)          |Board cold-cycle (opens buck-input FET). Asserted by firmware for a commanded full reset; also OR-driven by WDT/thermal/latched faults in hardware. Default-RUN via pulldown. Q3 (BC857W) is the high-side sustain PNP that holds the kill latched.|

### 2.7 Reliability / monitoring

|Net (pin)                                 |Software role                                                                                                                                                                                                                                                       |
|------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|`WDT_KICK` (PB2)                          |Refresh the external **windowed** watchdog from a supervisor task that itself checks liveness of the timing, network, and housekeeping threads (kick only if all are healthy). Windowed → both stalls and runaway loops trip it. WDT timeout → POE_KILL in hardware.|
|`PFI` (PE8, EXTI8)                        |Power-fail early warning (ahead of on-chip PVD). ISR commits volatile critical state (last Vc, leap, calibration deltas, log cursor) to NVS and parks the DAC; the supercap/PVD covers the write window.                                                            |
|`RTC_TAMP` (PC13)                         |Tamper/intrusion → timestamped event in the audit log, optional secure-erase policy hook, SNMP trap. Handled by RTC/TAMP peripheral.                                                                                                                                |
|`POE_NCL/NCM/LCF` (PC7/PC2/PC3, EXTI7/2/3)|NCP1095 PD status (class/event/fault). Track PoE state, detected class vs measured draw, and fault latches; gate `RB_PWR_EN` if the granted budget can’t cover the Rb. `POE_PG`/PG7 (divided to 2.93 V) and INA228 voltage/current corroborate these status lines.|
|`HOLDOVER_ALARM_RELAY` (PA6)              |Holdover alarm relay **K2** (G6K-2F-Y, Form-C → J16). **Normally-energized fail-safe:** drive PA6 HIGH (energize) only when service quality is met; any {fault, reset, power loss, boot} de-energizes → NC closes = **ALARM**. Default de-energized (R259 10k↓ = ALARM at boot). §3.6/§8-alarms.|

### 2.8 Local UI

|Net (pin)                 |Software role                                                                                                                                          |
|--------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
|7-button keypad `BUTTON_1..7` (PF0–PF6) + `DISP_TOUCH_INT` (PF7) + `ENC_BUTTON` (PF11)|**Direct GPIO**, serviced by the 1 kHz `io_scan` (§2.4a/§1.2): debounce buttons (~20–30 ms), service FT6336U touch on PF7 (no debounce), dispatch to the nav state machine (§6.1).|
|Rotary encoder `ENC_A/ENC_B` (PA8/PA9)|**Hardware quadrature via TIM1 encoder mode** (through U68 Schmitt buffer) — a counter, **not scanned**. Menu scroll / value edit. §6.1.|
|RGB LED (PD12/13/14, TIM4)|At-a-glance status: green = locked stratum-1, amber = warming/holdover, red = unlocked/fault, blue pulse = activity/identify. PWM for brightness/blend.|
|`FAN_PWM` (PE5, TIM15_CH1)|25 kHz fan drive from the thermal loop (§10.2). **Fail-safe**: undriven = full speed.                                                                  |
|`FAN_TACH` (PA15, EXTI15) |Edge-count over 1 s → RPM; stall/again alarm.                                                                                                          |
|Display + touch + proximity|ST7796 GUI (§2.5/§6.5), FT6336U touch wake (primary, §6.5), magnetic-reed proximity wake (secondary, §6.4). Display/touch fed from the J17 panel-power pins (§2.6).|

### 2.9 USB & debug

|Net (pin)                             |Software role                                                                                                                             |
|--------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------|
|`USB_DM/DP` (PA11/PA12)               |USB-FS device, **CDC-ACM** console (driverless). HSI48 + CRS clock. Carries shell, live logs, and **MCUmgr/SMP** firmware recovery (§8.4).|
|`USB_VBUS_SENSE` (PE2)                |Gate the D+ pull-up / host-connect detect (self-powered device). Polled (not EXTI — line shared with PC2).                                |
|`SWDIO/SWCLK` (PA13/PA14), `SWO` (PB3)|Development debug + `SWO` for high-rate trace/log during bring-up. **SWD-only board:** PB4 (NJTRST) carries `RB_RX` (UART7_TX), so **JTAG is unavailable** — all debug is SWD; firmware MUST NOT enable the JTAG pins. Locked out in production via H5 debug-authentication (§9.1).|

-----

## 3. Timing engine

The core competency. Everything else is plumbing around this.

### 3.1 Timebase architecture

The disciplined 10 MHz on `PH0` clocks the PLL → 250 MHz SYSCLK and, through the timer tree, the capture timers. The **ETH PTP clock** is the canonical system time: it is *syntonized* to `PH0` (so its rate follows the disciplined reference) and *synchronized* (offset corrected) to GPS-UTC by the discipline engine. NTP/PTP timestamps are taken from this PTP clock in hardware. A monotonic TAI counter underlies UTC; the leap-second offset is tracked from UBX-NAV-TIMELS.

### 3.2 PPS capture & timestamping

1. Each GPS 1PPS edge is hardware-captured on `PA0` (TIM2_CH1) and, redundantly, `PC6` (TIM3). Capture latches the reference-clock count → sub-ns resolution relative to the disciplined timebase.
1. **Sawtooth correction is mandatory.** Read UBX-TIM-TP each second; apply its quantization-error (`qErr`) to the capture, removing the receiver’s few-ns PPS sawtooth. Uncorrected, this dominates short-term error.
1. Subtract the calibrated **antenna cable delay** and any board PPS-routing delay (§10.4) to refer the edge to the antenna phase center / UTC.
1. Cross-check PA0 vs PC6; reject outliers (multipath/holdover glitches) with a median/MAD gate before the edge enters the loop.

### 3.3 OCXO disciplining loop

- **Controller:** combined FLL (frequency) + PLL (phase), PI form, operating on the sawtooth-corrected PPS phase error. Loop time constant configurable 10–1000 s (default ~100–300 s) — long enough to filter GPS PPS noise, short enough to track OCXO aging/thermal.
- **Actuator:** DAC1_OUT1 (PA4), slew-limited, scaled so full-scale = the OCXO ±0.4 ppm pull with 1.65 V center. Resolution and dither chosen so 1 LSB ≪ the holdover spec.
- **Feed-forward:** TMP117 #1 oscillator temperature drives a learned tempco correction (improves warm-up and post-transient settling).
- **Telemetry:** publish phase error, frequency error, Vc (commanded + sensed via PA3), loop state, and Allan-deviation estimates (τ = 1, 10, 100 s) at 1 Hz.
- **Locked criteria:** |phase error| and its variance under thresholds for N consecutive seconds, OCXO warm (INA228 #5 + TMP117 #1), GPS time-locked. Only then advertise stratum-1.

### 3.4 Rb management

- The Rb is an **FE-5680A** standalone rubidium reference. Firmware does **not** steer it; it sequences power, reads health over UART7, and watches `RB_LOCK` + `EXTREF_MON` for a valid, in-band 10 MHz before the mux may select it.
- **Variable rail (VCC_RB).** The MIC28516 buck (U40) FB is trimmed by digipot U43 (MCP41U83) buffered through OPA320 U42, referenced to **VREF_3V0 = 3.0 V** (MCP1502-30E U45, gated by RB_PWR_EN). Transfer function (netlist-derived FB network R147/R148/R134): **`VCC_RB = 24.45 − 6.645·VCTRL`**, VCTRL = 0…3.0 V → 24.45 V (code parks VCTRL=0) down to 4.51 V (VCTRL=3.0). The 24.45 V pedestal is **fixed by the resistors**; the digipot can only pull the rail **down**, so no wiper state (POR / mid-scale / SPI-fault / open) can exceed the pedestal.
- **Enable sequence (MUST, in order):** (1) write a **known safe-low digipot code** over SPI4 — code 0x000 parks the wiper at terminal B = safe-low VCTRL (POR = midscale ~14.5 V, inside the 26 V OV envelope); (2) `RB_PWR_EN` (PB7) → soft-start; (3) **verify `VCC_RB` on INA228 #6 (0x47)** is inside the `24.45 − 6.645·VCTRL` window **before trusting the rail**; (4) `RB_VCC_GATE` (PB1) → HIGH to connect `VCC_RB_G` to the FE-5680A; (5) wait for `RB_LOCK` + `EXTREF_MON` in-band. **Never** assert `RB_PWR_EN` before the safe code is written and read back. Warm-up gated by §10.3 PoE budget (after OCXO warm + supercaps charged).
- **Autonomous 26 V OV latch** (U41A, firmware-independent) trips VCC_RB > 26.0 V → kills U40; `RB_OV_DET` (PE3) high = tripped; clear by pulsing `RB_OV_RESET` (PD3) HIGH once the cause is gone. This is the backstop against any FB fault above the pedestal.
- **Rb gate path.** Gate-clamp zener **D25** (K=VCC_RB) bounds the Q25 SI7469DP gate so Q25 enhances and `VCC_RB_G` powers the FE-5680A. Two hardware caveats (not firmware blockers): rate the Rb-buck output caps C125–C128 ≥50 V (they sit on the 24.45 V VCC_RB rail), and for a 15 V-class FE set the digipot bound / OV per the unit (the 24.45 V full-scale pedestal over-volts a 15 V unit; the OV latch trips only at 26 V). Confirm FE J6-8/J6-9 Tx/Rx direction per surplus variant.
- Optional fine EFC trim over UART7 only if a GPS-PPS cross-check shows a residual the Rb isn’t removing; default is hands-off.

### 3.5 Reference selection / mux state machine

States: `OCXO_ACTIVE` (boot default) ⇄ `RB_ACTIVE`, plus `EXTREF_ACTIVE` (house standard on input B).

- **OCXO → Rb** only when **both** `EXTREF_MON` reports a real, in-band ~10 MHz **and** `RB_LOCK` is asserted, stable past the debounce/hysteresis window.
- **Rb → OCXO** immediately if **either** condition drops (fail-safe). Hysteresis on re-engage prevents flapping.
- **Glitchless switch (firmware, not a glitch-free mux IC):** the mux is a **plain 74LVC1G157 selector (U52)** — deliberately, because a dedicated glitch-free mux completes on the *outgoing* clock's edges and hangs if that source has stopped (the exact Rb-failure case). Sequence: bridge SYSCLK to HSI, command `MUX_SEL` (PB6), let the selector switch, reselect HSE, re-lock PLL, resume; the STM32 **CSS** catches a dead source. Both inputs are 10 MHz so PLL config is unchanged; the disciplined PTP clock free-wheels on HSI for the few ms of changeover and is re-aligned after.
- The OCXO is never powered down; on `RB_ACTIVE` its DAC holds last-good so it remains an instantly-available warm fallback.

### 3.6 Holdover engine

On GNSS loss (no fix / PPS outliers / antenna fault):

- Freeze the loop actuator at the last good value (OCXO) or trust the Rb flywheel (if active).
- Switch service quality: increment NTP **root dispersion** per the active oscillator’s characterized drift (§10.4), advance **stratum** only if dispersion exceeds policy, set the PTP `clockClass`/`clockAccuracy`/`offsetScaledLogVariance` to holdover values, and assert the holdover flag in all telemetry.
- Maintain an **error budget estimate** = ∫(characterized drift rate × time) + temperature term; expose “estimated time error” and “time until stratum demotion” to the GUI/SNMP.
- If holdover exceeds policy and peer NTP servers are configured, fall back to disciplining against them (degraded, clearly flagged) rather than free-running indefinitely.
- On GNSS recovery, re-converge with a rate-limited phase pull-in (no step) to avoid serving a time jump.

### 3.7 GNSS engine

- **Receiver config (UBX, persisted to F9T BBR + flash):** timing mode, constellations (GPS+Galileo+GLONASS+BeiDou), elevation mask, UBX messages enabled (NAV-PVT, NAV-SAT, NAV-SIG, NAV-TIMELS, TIM-TP, MON-RF, MON-HW), NMEA off, rate 1 Hz (+ on-demand NAV-SAT for skyplot).
- **Survey-in / fixed position:** on first install, run survey-in to a configured accuracy/duration, store the surveyed ECEF position, then operate in **fixed-position (stationary) timing mode** for best timing performance. Position is a stored calibration constant; re-survey is an explicit operator action.
- **Skyplot data:** UBX-NAV-SAT (az/el/CNR/used) + NAV-SIG → per-SV records for both GUIs (§6.3).
- **Antenna supervisor:** fuse UBX-MON-RF, `GPS_ANT_OFF_MON` (PD4), and antenna INA228 #4 → OK/open/short; on persistent short, drop `ANT_BIAS_EN` and alarm.
- **Leap second:** track from NAV-TIMELS; schedule and apply at the announced epoch; expose pending-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality export

`discipline` publishes a single authoritative quality block each second consumed by NTP, PTP, GUI, SNMP, console: stratum, lock state, reference in use, holdover flag + estimate, root delay/dispersion, last PPS offset, frequency error, Vc, ADEV, GNSS fix/SV counts, leap state. This is the one source of truth; services never compute their own.

-----

## 4. Network time services

### 4.1 NTP server (RFC 5905, v4)

- Hardware RX/TX timestamps via `SO_TIMESTAMPING` against the PTP clock → low-µs server-side accuracy independent of CPU load.
- Fills LI/stratum/precision/root delay/root dispersion/reference ID from §3.8. Stratum-1 with refid `GPS` (or `GNSS`) when locked; demotes per holdover policy.
- Capacity target ≥ 10,000 req/s sustained (matches commercial class); rate-limiting and KoD (RATE) on abuse; per-client and aggregate counters.
- Modes: unicast server, optional broadcast/multicast. Interleaved mode supported for best symmetry where clients use it.

### 4.2 NTS (RFC 8915)

- **NTS-KE** over TLS 1.3 (mbedTLS/PSA), server cert per §9.2; AEAD = AES-SIV-CMAC-256 (RFC 5297). Issues cookies; client presents a cookie per NTP request, gets a fresh one back.
- **Master keys** rotate on a configurable interval; sealed in NOR (wrapped by an ATECC608B/PSA key) so cookies survive reboot within the validity window. Key IDs tracked for graceful rotation.
- NTS request handling shares the §4.1 timestamp path; crypto runs in the `ntp_server` thread using H5 AES acceleration.

### 4.3 Authenticated NTP (symmetric)

- Legacy symmetric-key MAC (per-peer key, SHA-256/AES-CMAC) for clients that can’t do NTS. Keys managed in the security config. Autokey is **not** implemented (deprecated/insecure); NTS is the recommended path.

### 4.4 PTP grandmaster (IEEE 1588-2019)

- Hardware-timestamped Sync/Follow_Up/Delay_Req-Resp (and P2P where profile dictates) off the ETH PTP unit, syntonized to the disciplined reference.
- **Profiles:** Default (1588), and configurable **Telecom (G.8275.1/.2)** and **Power (C37.238)** profiles — domain, priority1/2, log intervals, transport (L2/UDP), one-step preferred (hardware one-step if the MAC supports it, else two-step).
- **BMCA** with `clockClass`/`clockAccuracy` driven by §3.8 (GPS-locked vs holdover vs free-run). Grandmaster-capable on all LAN ports/VLANs per config.
- **Security:** IEEE 1588-2019 Annex P integrity (ICV/TLV) where peers support it; otherwise document reliance on network-layer protection (MACsec/VLAN isolation). PTP management messages gated.

### 4.5 Network configuration

- Dual-stack IPv4/IPv6; static or DHCP/SLAAC; optional 802.1Q VLAN(s); per-service bind (e.g., PTP on a timing VLAN, management on another).
- mDNS/DNS-SD advertisement for discovery; NTP peers/pool for holdover fallback (§3.6); NTRIP client for RTCM corrections (§2.2) optional.
- All management services (HTTPS/SNMP/SSH-console-if-enabled) bindable to a management interface/VLAN and ACL-restricted, separable from the time-serving interface.

-----

## 5. Management & control plane

### 5.1 HTTPS web server

- TLS 1.3 only (1.2 configurable for legacy), strong cipher policy, HSTS, secure cookies, CSRF tokens, per-route RBAC. Server private key in the ATECC608B where possible; certificate either operator-uploaded (PEM/PKCS#12), generated CSR for an external CA, or self-signed with a downloadable root for pinning. Optional ACME (later).
- **API:** versioned REST (`/api/v1/...`) for config/control/calibration/firmware; **WSS** stream for real-time telemetry (clock/lock, skyplot, rails, logs) at 1–4 Hz. JSON throughout; schema-validated.
- Static SPA served from NOR; gzip/brotli pre-compressed; cache-busted. All mutating routes require auth + role + audit-log entry.

### 5.2 Web GUI (pages)

|Page            |Content / controls                                                                                                                                            |
|----------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
|**Dashboard**   |Live stratum/lock/reference, holdover flag + estimate, GPS fix + SV counts, **skyplot**, served-time stats (NTP/NTS req/s, PTP state), key rail/temps, alarms.|
|**Timing**      |Loop state, phase/freq error trends, Vc (commanded/sensed), ADEV plot, reference state machine, manual reference override (guarded), loop-constant tuning.    |
|**Calibration** |Antenna cable delay, PPS offset, OCXO tuning characterization, Rb EFC trim, e-compass hard/soft-iron, sensor offsets, holdover characterization run. §10.4.   |
|**GNSS**        |Constellations, elevation mask, survey-in / fixed-position control + stored position, antenna supervisor, RTCM/NTRIP config, F9T firmware update.             |
|**Network**     |Interfaces/VLAN/IP, NTP server + peers, NTS, PTP profile/domain, mDNS, ACLs.                                                                                  |
|**Security**    |Users/roles, RADIUS/LDAP/TACACS+ (optional), TLS certs/keys, NTS keys + rotation, SNMPv3 users, firmware-signing trust, debug-auth, audit log, secure erase.  |
|**Power/Health**|INA228 ×6 (V/I/P), TMP117 ×2, die temp, fan RPM + curve, supercaps, PoE class/draw, fault latches.                                                            |
|**Logs**        |Live tail (WSS), filter by subsystem/level, download, syslog config.                                                                                          |
|**Firmware**    |Running/standby versions, signed image upload, stage→verify→swap→confirm/revert, MCU + GNSS + (optional) Rb firmware.                                         |

Every real-time element on Dashboard/Timing/GNSS reads the §3.8 quality block via WSS; the skyplot mirrors the local-UI renderer (§6.3).

### 5.3 SNMP agent (Zabbix-ready)

- **SNMPv3** (USM: SHA-256 auth, AES-128/256 priv) primary; SNMPv2c optional behind ACL.
- **Enterprise MIB** exposing: stratum, lock/reference/holdover state + estimate, phase/freq error, Vc, ADEV, GNSS fix/SV counts/antenna state, NTP/NTS/PTP counters and PTP port state, all rails (V/I/P), temps, fan RPM, supercaps, PoE class/draw, fault flags, firmware versions, uptime.
- **Traps/informs** on: lock acquired/lost, holdover enter/exit, reference switch, antenna fault, rail/thermal alarm, PoE fault, tamper, auth failure, firmware change.
- Ship a **Zabbix template** (`Template_NTP_Server.yaml`) mapping the MIB to items/triggers/graphs (lock-lost, holdover>threshold, antenna fault, temp/rail out of range, time-error estimate) so it drops into Zabbix with discovery.

### 5.4 Logging / syslog

- Structured events (RFC 5424 syslog), levels, subsystem tags; RAM ring (fast) + NOR spool (persistent) + remote syslog (TLS optional). Time-quality and security events always persisted. Audit log (who/what/when for every config/control/firmware action) is separate and tamper-evident.

### 5.5 Config model

- Single typed, versioned config tree (network, timing, gnss, ntp/nts, ptp, snmp, security, ui, logging). Stored in NVS (critical) + NOR (bulk); schema-migrated across firmware versions. Atomic apply with validate-then-commit and rollback. **Import/export** (signed JSON) via web and console. **Factory reset** clears config + keys (policy-gated) and re-runs first-boot.

-----

## 6. Local UI (TFT + touch + 7-button)

### 6.1 Buttons & navigation

- Seven buttons `BUTTON_1..7` on **direct GPIO PF0–PF6** (active-low, each 10k↑ + 0.1 µF); the 1 kHz `io_scan` (§2.4a) diffs GPIOF, debounces (~20–30 ms), and emits press/long-press/repeat events. `DISP_TOUCH_INT` (PF7) carries the FT6336U touch INT (serviced over I²C, no debounce). A **rotary encoder** (`ENC_A/ENC_B` PA8/PA9, TIM1 hardware quadrature; push-switch `ENC_BUTTON` PF11) provides scroll/edit.
- Logical map: **Up, Down, Left/Back, Right/Enter, Function/Menu, Power/Identify** (final assignment in §15). Nav is a screen-stack state machine: list scroll, value edit (where permitted), confirm dialogs for guarded actions.
- Locally editable: network basics (DHCP/static, IP), display brightness/timeout, identify (blink RGB + WSS hint), reference override (guarded), reboot, factory-reset (double-confirm). Deep config stays on the web/console — the panel is for status + field service.

### 6.2 Screen hierarchy

`Home` (time, stratum, lock, reference, SV count, holdover) → `Skyplot` → `Clocks` (OCXO Vc/temp/ADEV, Rb lock/health, active reference, holdover estimate) → `Network` (IP, NTP/PTP state, served rate) → `Power/Health` (rails, temps, fan, supercaps, PoE) → `Menu` (the editable items above) → `Alarms`. Left/Back pops; Right/Enter descends.

### 6.3 Skyplot rendering

- Polar plot, zenith center, horizon edge; per-SV marker placed by az/el from UBX-NAV-SAT, colored by constellation, sized/labelled by CNR, hollow = visible-not-used, filled = used in solution.
- **True-north orientation:** rotate the plot by the magnetometer heading (IIS2MDC), corrected by hard/soft-iron calibration (§10.4) and by **magnetic declination computed from the GPS fix** (WMM/IGRF); tilt-compensate with the accelerometer (LIS2DH12) — mandatory given the vertical board mount. If the compass is uncalibrated/disturbed, fall back to “GNSS-north” (az as reported) with a clear “north unverified” badge.
- Identical data feeds the web skyplot (§5.2) so both views agree.

### 6.4 Touch / proximity wake & RGB

- **Touch is the primary wake.** FT6336U INT on `DISP_TOUCH_INT` (PF7, scanned/EXTI7) wakes to Home. Sleep/wake is a **`DISP_BL` PWM duty change only** — blank = duty→0, wake = duty→on — not a power or reset event: `DISP_EN`, the 5 V rail, and the rendered ST7796 frame all persist. Because touch rides the `DISP_EN`-gated rail, **DISP_EN stays asserted through the dimmed/blanked idle state**; drop it only for a deliberate display-off/recovery.
- **A magnetic reed switch is the secondary/ambient wake** (`PROX_WAKE` PF10, EXTI10, J17.18, discrete GPIO): a magnet approach closes the dry contact and re-asserts `DISP_EN` (if dropped), waking to Home. The reed is passive (no Vcc); pull-up R254 to always-on 3V3_STM + close-to-GND is the correct conditioning → **active-low on magnet present**, always live regardless of DISP_EN. Thresholds/timeouts configurable.
- RGB semantics per §2.8; brightness follows ambient/backlight policy; an “identify” command pulses blue to locate the unit in a rack.

### 6.5 Display driver

- ST7796 over SPI4, write-only, dedicated baud/mode profile (20–40 MHz), DMA frame/region pushes, partial-region updates for changing fields to bound traffic (≈1.2 MB/s @ 4 Hz full-frame worst case). Render thread is lowest-priority and pre-emptible; never blocks timing.
- **Power/reset sequencing:** assert `DISP_EN` (PC11) → RT9742 soft-starts `5V_DISP` and the PCA9306 enables; mask `V_DISP_EN_FAULT_N` (PF9, RT9742 U33 nFLG) during the soft-start window; release `DISP_RST` (PA10, held low at boot by R209 10k↓ → panel + touch reset) with the FT6336U reset-timing margin; init ST7796, then init FT6336U over the (now-translated) touch I²C. A `DISP_EN` low→high cycle fully cold-restarts a wedged panel/touch (module 74LVC245 has Ioff → no sneak path keeps it alive); drive DISP_CS/DC/BL to a defined state during the off window.

-----

## 7. Console & command/control (USB CDC-ACM + SWO)

### 7.1 Shell

Full command/control parity with the web API for headless/field use, over the driverless CDC-ACM port (and mirrored on SWO for trace during bring-up). Command groups:

|Group   |Examples                                                                                                           |
|--------|-------------------------------------------------------------------------------------------------------------------|
|`status`|`status`, `status timing`, `status gnss`, `status power`, `status ptp` — one-shot or `--watch`.                    |
|`clock` |show loop/Vc/ADEV; `clock ref ocxo                                                                                 |
|`gnss`  |constellations, survey-in start/stop, fixed-position set, `gnss sat` (text skyplot), antenna status, `gnss fw ...`.|
|`net`   |interface/IP/VLAN, `net ntp ...`, `net nts ...`, `net ptp ...`, peers, ACL.                                        |
|`cal`   |antenna delay, pps offset, ocxo characterize, rb trim, compass calibrate, sensor offsets.                          |
|`sec`   |users/roles, certs, nts keys, snmp users, debug-auth, `sec audit tail`.                                            |
|`log`   |`log tail [subsystem] [level]`, level set, syslog config, download cursor.                                         |
|`fw`    |versions, `fw upload` (kick SMP), `fw confirm` / `fw revert`.                                                      |
|`sys`   |reboot, factory-reset (double-confirm), config export/import, `sys ident`.                                         |

- Auth required for mutating commands (local credential or, if enabled, the same RADIUS/TACACS+ backend); read-only status available pre-auth on the physical port if policy permits. Every mutating command writes the audit log.
- Scriptable: stable, line-oriented, machine-parseable output mode (`--json`) for automation.

### 7.2 Live logging

`log tail` streams the structured log with filters; the same ring buffer feeds web WSS and syslog. Diagnostics commands dump loop history, PPS residual histograms, I²C bus scans, and a one-shot health snapshot for support.

-----

## 8. Firmware update & integrity

### 8.1 Secure boot chain

`STiRoT (immutable ST RoT)` → verifies → `MCUboot` → verifies app signature + **anti-rollback counter** → app. Debug is closed in production via H5 debug authentication (§9.1). A verification failure drops to MCUboot **serial recovery** (SMP over CDC-ACM). No unsigned image ever executes.

### 8.2 Image format & signing

- MCUboot image format; signature **ECDSA P-256** (default) or ed25519; hash SHA-256. Signing key held offline by the operator; the **public trust anchor** is provisioned at manufacture (and pinned by STiRoT/MCUboot). Anti-rollback monotonic counter prevents downgrade.
- Two slots (swap-move). New image boots in **test** mode; the app must self-confirm after passing post-update health (network up, clock disciplining, services answering) within a watchdog window, else MCUboot reverts automatically.

### 8.3 Update via web

`Firmware` page: upload signed image → staged to slot-1 (or NOR if internal slot-1 is undersized) → MCUboot verifies signature/version → operator confirms swap → reboot into test → auto-confirm on health or auto-revert. Progress and result over WSS; full audit entry. Rollback is one click (until confirmed).

### 8.4 Update via USB/UART

- **Primary:** MCUmgr/SMP over CDC-ACM — `fw upload` from the shell or an `mcumgr` host pushes the signed image into MCUboot serial recovery; same signature/anti-rollback checks; same test/confirm/revert. Works even if the application is unbootable.
- **Last resort:** STM32 system-memory ROM bootloader via BOOT0 (DFU/UART) for unbricking at the factory; gated by debug-auth state.

### 8.5 GNSS firmware update passthrough

F9T firmware updated over USART3 (PD8/PD9) using `GPS_SAFEBOOT_N (PD15)` + `GPS_RST_N (PD11)` for safeboot recovery. The app exposes a guarded `gnss fw` flow (web + console) that streams the u-blox image, verifies the receiver reports the new version, and restores timing config afterward.

### 8.6 Asset / Rb / config updates

Web SPA bundle and MIB/Zabbix template update with the firmware image (versioned together). Rb (FE-5680A) firmware is vendor-flashed over UART7 only if the module supports it (guarded, rare). Config import is a signed JSON applied via validate-then-commit (§5.5).

-----

## 9. Security architecture

### 9.1 Root of trust & key hierarchy

- Hardware RoT = STiRoT (immutable) → MCUboot trust anchor → application. Debug ports closed in production via H5 **debug authentication** (re-open only with a signed challenge). Readout protection (RDP/product state) at the highest level consistent with field-update needs. The board is **SWD-only** (PB4/NJTRST reused as `RB_RX`); production debug-auth governs SWD, not JTAG.
- **ATECC608B** holds the device identity ECC P-256 keypair and (where exportability must be denied) the TLS server key; performs ECDSA sign, ECDH, SHA-256/HMAC and AES-128, has a hardware TRNG, and provides two hardware monotonic counters plus attestation. PSA Crypto fronts both the ATECC608B (via Microchip CryptoAuthLib’s mbedTLS integration) and the on-die AES/PKA/HASH.

### 9.2 TLS

TLS 1.3 (1.2 optional), ECDHE + AES-GCM/CHACHA, server cert from an ATECC608B-backed key (ECDSA P-256) or operator-supplied (RSA/ECDSA, in software); CSR generation, PKCS#12 import, self-signed + downloadable root for pinning. Used by HTTPS (§5.1), NTS-KE (§4.2), and optional TLS-syslog.

### 9.3 Time-protocol auth

NTS (§4.2) is the recommended authenticated-NTP path; symmetric-key NTP (§4.3) for legacy; PTP integrity per 1588-2019 Annex P where supported (§4.4). All key material in the security config, sealed at rest.

### 9.4 Management auth

Local users with Argon2id-hashed credentials; roles **admin / operator / viewer**; optional **RADIUS / LDAP / TACACS+** (commercial parity, later phase). Session tokens, idle/absolute timeouts, lockout on brute force, CSRF protection, per-action authorization, full audit log. Console mutating commands obey the same policy.

### 9.5 SNMPv3

USM with SHA-256 auth + AES priv; per-user engine; v2c only if explicitly enabled and ACL-scoped. Traps carry no secrets.

### 9.6 Tamper & secure storage

`RTC_TAMP (PC13)` events timestamped and trapped; optional policy to zeroize keys on tamper. Secrets (NTS master keys, symmetric keys, credentials) stored wrapped by an ATECC608B/PSA key. Secure-erase (factory reset with key zeroization) is policy-gated and audited.

### 9.7 Hardening checklist

Closed debug in production; signed-only firmware + anti-rollback; least-privilege service binding + ACLs + VLAN separation of timing vs management; rate-limiting on NTP and on auth; no plaintext secrets in logs; constant-time crypto via PSA; default-deny management exposure; minimal attack surface (no telnet/ftp; SSH console optional and key-based if enabled).

-----

## 10. Health, telemetry & calibration

### 10.1 Sensor acquisition

`housekeeping` polls the I²C devices (§2.4) and the internal ADC (VREFINT for true VDDA, die temp, VBAT, OCXO Vc `OCXO_V` PA3) on the cadences listed, applies per-sensor calibration offsets, and publishes a health block consumed by GUI/SNMP/console/display. Supercap state is read from the backup power-good lines `BKP_STM_PG` (PF14) / `BKP_GPS_PG` (PF15), not a dedicated ADC channel (the TPS61094 managers are autonomous). INA228 ALERTs are **separate per-rail GPIOs** serviced by the 1 kHz `io_scan` (§2.4a); on any assert, immediately re-read that INA228's flags register and run alarm evaluation.

### 10.2 Thermal / fan loop

PI loop on enclosure temp (TMP117 #2) with oscillator temp (TMP117 #1) and die temp as inputs; drives `FAN_PWM (PE5)` 25 kHz; RPM from `FAN_TACH (PA15)`. Hysteresis + minimum duty for bearing life; **fail-safe full-speed** if the loop or MCU stalls (undriven PWM = max). Fan stall/under-speed → alarm + trap.

### 10.3 Power sequencing & faults

- **Boot order:** main 3.3 V (3V3_STM — housekeeping I²C, NOR all come up here, no gating) → release held-in-reset digital devices in order (`NOR_RST_N` PE10 → HIGH first; `LAN_RST_N` PD10 after rails+25 MHz; `DISP_RST` PA10 when UI wanted) → GPS (`GPS_PWR_EN`) + antenna bias (`ANT_BIAS_EN`) → display/touch (`DISP_EN`, when UI is needed) → OCXO warm → (when warranted) the **guarded Rb sequence** (safe digipot code → `RB_PWR_EN` → INA228 #6 0x47 verify → `RB_VCC_GATE`, §3.4). Sequencing keeps the cold-start peak inside the granted PoE budget (§ pin map 6); if PoE class can’t cover the Rb, defer/deny `RB_PWR_EN` and flag. The display rail is deferrable/low-priority — drop `DISP_EN` first under PoE pressure. Full ordered bring-up: interface ref §2.
- **`PFI (PE8)`** ISR: persist volatile critical state, park the DAC, prepare for brownout (supercap covers the window).
- **PoE monitor** (`POE_NCL/NCM/LCF`): track class/event/fault; gate Rb enable; trap on fault.
- **`POE_KILL (PE15)`** for commanded cold cycle; also hardware-OR’d from WDT/thermal/latched faults. The supervisor task kicks `WDT_KICK (PB2)` only when timing/network/housekeeping liveness all pass.

### 10.4 Calibration (procedures + stored constants)

|Calibration                 |Method                                                                                  |Stored as                        |
|----------------------------|----------------------------------------------------------------------------------------|---------------------------------|
|Antenna cable delay         |Enter known cable length/velocity factor or measure against a reference; applied in §3.2|ns offset                        |
|PPS / board routing offset  |Differential PA0 vs PC6, or against a calibrated PPS source                             |ns offset                        |
|OCXO tuning characterization|Sweep DAC, record Vc→frequency, derive gain/center; learn tempco vs TMP117 #1           |curve + tempco                   |
|Rb EFC trim                 |Optional fine-trim vs GPS over UART7                                                    |EFC value                        |
|Holdover characterization   |Run controlled GNSS-denied intervals, record drift vs time & temp per oscillator        |drift model (feeds §3.6 estimate)|
|E-compass hard/soft-iron    |In-enclosure rotation cal (once, fixed install)                                         |3×3 + offset                     |
|Sensor offsets              |Per-INA228 shunt/zero, TMP117 trim                                                      |per-channel                      |

All calibration constants live in NOR/NVS, are export/importable (signed), and are shown/edited on the Calibration page and `cal` console group. Changing one writes the audit log and re-evaluates affected telemetry.

### 10.5 Metrics catalog

The canonical metric set (one definition, exposed everywhere): stratum; lock state; active reference; holdover flag + estimated time error + time-to-demotion; PPS offset (last/mean/σ); frequency error; OCXO Vc (cmd/sense) + tempco; ADEV(1/10/100 s); GNSS fix type, SVs visible/used per constellation, CNR stats, antenna state; NTP req/s + drops + KoD; NTS handshakes/cookies; PTP port state + offsetFromMaster + meanPathDelay + clockClass; per-rail V/I/P (×6); temps (×2 + die); fan RPM + duty; supercap voltages; PoE class + draw; fault flags; firmware versions (MCU/GNSS); uptime; audit/security counters. GUI, SNMP MIB, console `--json`, and the local display all bind to this set.

-----

## 11. Operational state machines (consolidated)

- **Boot:** STiRoT → MCUboot(verify/anti-rollback) → app(test|confirmed) → services up.
- **Oscillator (per OCXO/Rb):** OFF → WARMUP → READY → (OCXO: DISCIPLINING → LOCKED) / (Rb: SELF-LOCKING → LOCKED) → HOLDOVER → re-converge.
- **Reference:** OCXO_ACTIVE ⇄ RB_ACTIVE (guards: EXTREF_MON in-band AND RB_LOCK; fail-safe revert) ; EXTREF_ACTIVE for house standard.
- **GNSS:** COLD → ACQUIRE → SURVEY_IN → FIXED_POSITION/TIME_LOCKED → (DEGRADED on antenna/fix loss).
- **Service quality:** STRATUM1_LOCKED → HOLDOVER(dispersion-growing) → DEMOTED/PEER-FALLBACK → recover.
- **Update:** IDLE → STAGED → VERIFIED → TEST-BOOT → CONFIRMED | REVERTED.

Each transition emits a log event + (where defined) an SNMP trap and a GUI/RGB state change.

-----

## 12. Persistence & data layout

- **NVS (internal flash):** network identity, security policy handles, anti-rollback counter, last-good Vc/leap/cal deltas (PFI fast-save), config schema version.
- **SPI-NOR / LittleFS:** web SPA bundle, GNSS almanac/ephemeris cache, full config tree + signed export, calibration tables, NTS master keys (sealed), discipline/health log ring (bounded, oldest-evicted), syslog spool, audit log (append-only, tamper-evident), staged firmware image.
- Retention policies configurable; logs survive reboot; secrets always sealed; wear-leveling via LittleFS.

-----

## 13. Fault tree & recovery

|Fault                |Detection                      |Response                                                             |
|---------------------|-------------------------------|---------------------------------------------------------------------|
|GNSS fix/PPS loss    |parser + PPS outlier gate      |Holdover (§3.6); alarm/trap; GUI/RGB amber.                          |
|Antenna open/short   |INA228 #4 + MON-RF + PD4       |Flag; on persistent short drop `ANT_BIAS_EN`; trap.                  |
|Rb unlock / bad clock|RB_LOCK / EXTREF_MON           |Fail-safe revert to OCXO; trap.                                      |
|Rb rail over-voltage |`RB_OV_DET (PE3)`; INA228 #6 (0x47)|Autonomous 26 V OV latch already killed U40; log/trap; clear via `RB_OV_RESET (PD3)` pulse after cause gone (§3.4).|
|Panel-LED string open/short|INA228 #9 (0x4C) duty-normalized; `PANEL_LED_FAULT_N (PF12)`|Flag LED fault; UI-only, never blocks timing.                    |
|Rail power-good drop |PG scan (PG0/1/3/4/6/7, PF14/15) high→low|Rail-fault alarm/trap. **PG2 masked** until its LDO-PWRGD pull-up is fitted.|
|Ext-ref front-end misconfig / no output|EXTREF_MON invalid after config|Re-apply stored profile once; if still invalid, hold on OCXO, do not select B, flag config fault + trap. Never trust an unverified front-end output.|
|OCXO oven/DAC fault  |INA228 #5; Vc cmd≠sense (PA3)  |Mark OCXO unhealthy; prefer Rb if available; alarm.                  |
|I²C bus wedge        |transaction timeout / SCL stuck|Software bus-recovery (9 SCL clocks / STOP) + re-init; LTC4311 EN is hardwired on. Housekeeping is on always-on 3V3_STM (no rail-cycle path); no I/O-expander reset (expanders deleted — direct GPIO).|
|NOR hang             |LittleFS error                 |`NOR_RST_N (PE10)`; degrade to RAM-only logging; alarm.              |
|Display hang         |SPI/driver watchdog            |`DISP_RST (PA10)` re-init; UI optional, never blocks timing.          |
|Over-temp            |TMP117/die                     |Fan max; if exceeded, shed Rb, then thermal `POE_KILL`.              |
|Power-fail           |`PFI (PE8)`                    |Fast-save + park (§10.3).                                            |
|Thread stall         |supervisor liveness            |Withhold `WDT_KICK` → external WDT → `POE_KILL` cold cycle.          |
|Bad firmware         |post-update health             |MCUboot auto-revert to confirmed image.                              |
|Tamper               |`RTC_TAMP (PC13)`              |Audit + trap; optional key zeroize.                                  |

Principle: timing service degrades gracefully (holdover, reference fallback, peer NTP) before it ever serves wrong time; unrecoverable states cold-cycle rather than hang.

**Firmware-relevant hardware notes** (full detail in `sts1000_firmware_hardware_interface.md`).
**Standing firmware requirements:** SWD-only (PB4 = NJTRST → JTAG unavailable); `GPS_TXRDY` needs an
F9T CFG-TXREADY re-map before PD5 is trusted; digipot code 0 = Terminal B safe-low (still
pre-program the NV wiper + verify VCC_RB before `RB_PWR_EN`). **Open items:** add the
`3V0_RF_LDO_PG`/PG2 pull-up (10k→3V3_STM) — mask PG2 until fitted; confirm the J17 panel-power pins
(J17.16→5V_DISP, J17.28→3V3_STM) reach the panel; rate the Rb-buck output caps C125–C128 ≥50 V; grow
VREF+ C37 to 100 nF. The power-good/telemetry signals `POE_PG`/PG7 (divided to 2.93 V), INA228 0x40
current (R30 in series), `INA_ALERT_VCC_RB`/PG15, and PG4 (~2.98 V) are all usable as read.

-----

## 14. Build, test & validation requirements

- **Timing validation:** measure served accuracy against a reference grandmaster/UTC source; ADEV/MDEV vs τ for OCXO and Rb (locked and holdover); PPS residual histograms with/without sawtooth correction (prove the qErr path); holdover drift vs the characterized model.
- **Protocol interop:** NTP (chrony/ntpd/w32time clients), NTS (ntpsec/chrony NTS), PTP (linuxptp `ptp4l`/`phc2sys`, telecom/power profile testers), SNMP (Zabbix discovery + the shipped template), syslog collector.
- **Security:** signed-image enforcement + downgrade-rejection; debug-auth lockout; TLS scan (cipher/cert); NTS handshake/rotation; SNMPv3 auth/priv; brute-force lockout; secure-erase.
- **Robustness:** PoE class negotiation incl. staggered warm-up within budget; brownout/PFI save-restore; I²C/NOR/display wedge recovery; GNSS spoof/jam degradation; reference flap resistance (hysteresis); 10k+ NTP req/s soak with concurrent TLS/web/SNMP load proving the timing path is unperturbed.
- **CI:** unit tests for loop math, sawtooth, holdover estimator, config schema/migration; HIL rig for PPS injection and fault simulation.

-----

## 15. Open items / decisions

- Confirm ETH PTP **auxiliary-snapshot internal trigger** (RM0481) for direct PPS↔PTP capture; else lock in the TIM2↔PTP software correlation (pin map §14).
- One-step vs two-step PTP (does the H5 MAC do hardware one-step for the chosen profiles?).
- Final **button-function mapping** (the 6 logical keys) and long-press semantics.
- NTP-server + NTS-server + PTP-grandmaster implementation: build on Zephyr (custom services) vs adopt the FreeRTOS+lwIP fallback for lwIP-native SNMP — decide before committing the platform, as it sizes the largest effort.
- SNMP agent: custom compact agent vs ported lwIP `apps/snmp`.
- Web SPA framework/size budget (must fit NOR with assets + logs); pick a small framework (Preact/Svelte) and a brotli budget.
- RADIUS/LDAP/TACACS+ and TLS-syslog: phase-2 or phase-1?
- Rb EFC steering: hands-off vs optional firmware fine-trim — default hands-off; confirm FE-5680A control surface (serial command set + J6-8/J6-9 Tx/Rx direction per surplus variant).
- External-ref front-end (§2.1/§2.2) config policy: the as-built front end is a **single LTC6752 slicer (U50)** on the `10MHz_RF_IN` SMA with only a switched 50 Ω termination (`REF_TERM_EN` PC10, default terminated). Decide the stored default (terminated vs high-Z) per source impedance and the auto-probe policy (apply term → check `EXTREF_MON` → flag if invalid). No source-select or dual-slicer routing exists on this board.
- Leap-second handling policy at the second of insertion for NTP (smear vs step) and PTP (step) — define per service.
- Certificate lifecycle: manual/CSR only, or add ACME — phase decision.
- Anti-rollback counter budget and field-update policy (how many versions, recovery if exhausted).