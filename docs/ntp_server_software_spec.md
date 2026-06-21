# PoE GPS-Disciplined Stratum-1 NTP/PTP Server — Software & Firmware Specification

**Target:** STM32H563VIT6 (Cortex-M33 @ 250 MHz, 2 MB flash, TrustZone + PSA crypto, ETH-MAC with IEEE-1588 hardware timestamping).
**Companion document:** *Peripheral & Pin Map* (`ntp_server_peripheral_map.md`) — the authoritative hardware reference. Every pin/net cited here uses that document’s canonical net names. Where the two ever disagree, the pin map wins for electrical facts; this document owns software behavior.

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
|`EXT_REF_IN` (front end §2.2) → mux B, `GPS_PPS_OUT` (buffered TIMEPULSE) → ext-Rb 1PPS-in|(board nets)                   |Input B is driven by the **external-reference front end** (DB9→FE-5680A Rb, or SMA→future clock), conditioned to 3.3 V CMOS. An external Rb self-disciplines to the GPS 1PPS; firmware does not steer it (§3.4). `EXTREF_MON` (above) validates the conditioned output.|
|`REF_SRC_SEL` (PA10), `REF_TERM_EN` (PC10), `REF_SLICER_SEL` (PC0)   |GPIO out ×3                    |Direct control of the §2.2 front end (no expander, no build options — all hardware populated): source select (DB9/SMA), 50 Ω termination engage/lift, and slicer-branch route (LTC6957 low-jitter vs comparator tolerant/attenuated). **Set-once** from the stored per-source profile at boot and re-applied only on a source/clock-type change; never in the hot path. Apply → wait settle → require `EXTREF_MON` valid before the reference SM may select B.|

### 2.2 GNSS — ZED-F9T

|Net (pin)                                                        |Peripheral    |Software role                                                                                                                                                                           |
|-----------------------------------------------------------------|--------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|`USART3_TX/RX` (PD8/PD9)                                         |USART3        |Primary control/telemetry @ 460800 8N1. UBX binary protocol (NMEA disabled). Carries config, NAV-PVT/NAV-SAT/NAV-SIG/TIM-TP/MON-RF, and is the **F9T firmware-update transport** (§8.5).|
|`UART4_TX/RX` (PD1/PD0)                                          |UART4         |RTCM3 corrections input (RTK/precise timing) — optional; from an NTRIP client over the network or a local base.                                                                         |
|`GPS PPS` (PA0) / `TIMEPULSE2` (PC6)                             |TIM2/TIM3     |See §2.1. Configured via UBX-CFG-TP5: PPS1 = 1 Hz aligned to UTC, locked-only; PPS2 = higher-rate aux if useful.                                                                        |
|`GPS_RESET_N` (PD11)                                             |GPIO out      |Soft reset; part of FW-recovery combo.                                                                                                                                                  |
|`GPS_SAFEBOOT_N` (PD15)                                          |GPIO out      |Hold low + reset → F9T safeboot for firmware recovery (§8.5).                                                                                                                           |
|`GPS_DSEL` (PD7)                                                 |GPIO out      |Hold the receiver in UART+I²C interface mode at boot.                                                                                                                                   |
|`GPS_EXTINT` (PD6)                                               |GPIO out      |External interrupt/time-mark trigger to the F9T (e.g., aiding, time-mark requests).                                                                                                     |
|`GPS_TXRDY` (PD5)                                                |GPIO in, EXTI5|TX-ready / geofence status; wake the parser when the receiver has data.                                                                                                                 |
|`GPS_ANT_OFF_MON` (PD4)                                          |GPIO in, EXTI4|Observes LNA-disable; corroborates UBX-MON-RF antenna state and the antenna-rail INA228 for the supervisor (§3.7).                                                                      |
|GPS VCC switch (PC8), antenna bias (PC9), V_BCKP backup (PD2/PD3)|—             |See §2.6. V_BCKP **must** stay up through GPS VCC cycles for warm start.                                                                                                                |

### 2.3 Ethernet & PTP timestamping

|Net (pin)                                                                                                     |Peripheral                                 |Software role                                                                                                                                                                                                                                                                                                                                                               |
|--------------------------------------------------------------------------------------------------------------|-------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|RMII bus (PA1 REF_CLK, PA2 MDIO, PC1 MDC, PA7 CRS_DV, PC4/PC5 RXD0/1, PA5 TX_EN, PB12/PB15 TXD0/1, PB10 RX_ER)|ETH-MAC (RMII)                             |L2 transport. MDIO driver manages LAN8742 (link, autoneg, energy-detect). RMII 50 MHz domain is **independent** of the disciplined reference — by design.                                                                                                                                                                                                                   |
|`ETH_PPS_OUT` (PB5)                                                                                           |ETH PTP PPS                                |MAC PTP pulse-per-second output: scope/verification tap and a hardware cross-check that the PTP clock tracks the disciplined timebase.                                                                                                                                                                                                                                      |
|`PHY_nRST` (PD10)                                                                                             |GPIO out                                   |PHY reset/recovery.                                                                                                                                                                                                                                                                                                                                                         |
|(internal)                                                                                                    |ETH PTP timestamp unit + auxiliary snapshot|Hardware RX/TX timestamps for NTP/PTP; PTP clock is **syntonized** to the disciplined reference and **synchronized** (offset/skew steered) to GPS time via the discipline engine. Auxiliary-snapshot capture of the GPS PPS is the preferred PPS↔PTP correlation if RM0481 allows internal trigger (§14 open item); otherwise correlate TIM2 capture ↔ PTP-time in software.|

### 2.4 I²C1 sensor / secure / HMI bus (PB8 SCL / PB9 SDA)

400 kHz, behind the rise-time accelerator; `I2C_BUF_EN (PA9)` lets firmware isolate/recover a wedged bus (toggle on SCL-stuck or transaction-timeout). `INA_ALERT (PC12, EXTI12)` is the shared open-drain alert.

|Device (addr)              |Software role & cadence                                                                                                                                                    |
|---------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|INA228 #1 PoE in (0x40)    |Input power/PoE budget accounting; class-vs-draw sanity; brownout precursor. 4 Hz.                                                                                         |
|INA228 #2 STM 3V3 (0x41)   |MCU rail health. 1 Hz.                                                                                                                                                     |
|INA228 #3 GPS (0x44)       |GPS load; acquisition-vs-tracking signature. 1 Hz.                                                                                                                         |
|INA228 #4 Antenna (0x45)   |**Antenna supervisor**: open (low I) / short (high I) / OK; cross-checks UBX-MON-RF + PD4. 4 Hz.                                                                           |
|INA228 #5 OCXO (0x46)      |Warm-up vs steady; oven-fault detection; always reporting. 4 Hz during warm-up, 1 Hz steady.                                                                               |
|INA228 #6 Rb (0x47)        |**Closed-loop verify** of the digipot-set Rb rail before trusting Rb; continuous Rb health. 4 Hz.                                                                          |
|TMP117 #1 oscillator (0x48)|OCXO/Rb local temperature → aging compensation, thermal alarms. 1 Hz.                                                                                                      |
|TMP117 #2 ambient (0x49)   |Enclosure temp → fan loop setpoint. 1 Hz.                                                                                                                                  |
|Digipot #1/#2 (0x2C/0x2D)  |Rb-rail buck FB trim, **bounded by fixed resistors**; NV-wiper. Written only during commissioning/calibration with INA228 #6 read-back verify; never to an unbounded value.|
|ATECC608B (0x60)           |Device identity, TLS/NTS ECC P-256 private keys (non-exportable), ECDSA sign, ECDH, SHA-256/HMAC, AES-128, hardware TRNG, two monotonic counters, attestation. §9.         |
|FT6336U touch (0x38)       |Capacitive touchscreen; INT → MCP23017 U48 GPA7 → `BTN_INT_N`/PE0; read+dispatch on interrupt. On gated 5 V rail behind PCA9306. §6.5.                                       |
|MCP23017 U48 (0x21)        |7-button keypad (GPA0–6) + touch INT (GPA7) on PortA; 8 INA228 ALERTs on PortB. INT → PE0 / PC12. §6.1.                                                                     |
|IIS2MDC + LIS2DH12 (0x1E / 0x18-0x19)|E-compass: magnetometer heading + accel tilt for skyplot true-north; polled 1–5 Hz; calibrated constants applied. §6.3.                                          |
|PIR proximity (off-bus)    |Presence/approach wake — discrete digital input on MCP23017 U47 PortB (GPB3), not on I²C. Secondary wake (touch is primary). §6.4.                                          |

### 2.5 SPI4 (PE12 SCK / PE13 MISO / PE14 MOSI / PE11 NSS)

|Device                                                                     |Software role                                                                                                                                                            |
|---------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|SPI-NOR (CS PE11, `NOR_RESET` PE10)                                        |LittleFS backing store (web assets, logs, almanac, calibration, NTS keys, FW staging). DMA reads for asset serving. NOR_RESET for recovery from a hung NOR.              |
|ST7796 TFT (CS PE9, `DISP_DC` PB0, `DISP_BL` PE6/TIM15_CH2, `DISP_RST` PA8)|Local GUI on MSP4030 module. Write-only (MISO NC). Separate SPI baud/mode profile (20–40 MHz) per-CS. Partial-region updates; DMA frame pushes. Backlight PWM (onboard BSS138) for dim/blank. VCC = `V_DISP_5V`, gated by `DISP_EN`; touch (FT6336U) shares the rail (§2.4/§6.5). `DISP_RST` (PA8) also resets the touch controller. §6.5.|

### 2.6 Power, domains, backup, kill

|Net (pin)                  |Software role                                                                                                                                                                                                 |
|---------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|`GPS_PWR_EN` (PC8)         |GPS VCC load switch — hard power-cycle path for the receiver (warm start preserved by V_BCKP).                                                                                                                |
|`ANT_BIAS_EN` (PC9)        |Antenna bias-T high-side enable; part of the antenna fault response (disable on persistent short).                                                                                                            |
|`DISP_EN` (PC11)           |5 V display rail (`V_DISP_5V`, RT9742) + touch PCA9306 enable. **Ex-`PERIPH_EN`** — the 3V3_P housekeeping rail was removed (TMP117/ATECC608B/NOR/e-compass now on always-on 3V3_STM); this gates only display+touch. Default-off at boot (R161); cycle to cold-restart a wedged panel/touch. Mask `V_DISP_EN_FAULT` during soft-start.|
|`RB_PWR_EN` (PB7)          |Rb-rail buck EN — see §2.1/§10.3 sequencing and INA228 #6 verify.                                                                                                                                             |
|`BKP_GPS_EN/MODE` (PD2/PD3)|GPS V_BCKP supercap manager (TPS61094): keep enabled; firmware monitors mode/state; GPS warm-start retention.                                                                                                 |
|`BKP_STM_EN/MODE` (PE3/PE4)|STM32 VBAT supercap manager: **auto-backup in hardware** on input collapse (must not wait for firmware). Firmware monitors only.                                                                              |
|`POE_KILL` (PE15)          |Board cold-cycle (opens buck-input FET). Asserted by firmware for a commanded full reset; also OR-driven by WDT/thermal/latched faults in hardware. Default-RUN via pulldown. Use as the last-resort recovery.|

### 2.7 Reliability / monitoring

|Net (pin)                                 |Software role                                                                                                                                                                                                                                                       |
|------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|`WDT_KICK` (PB2)                          |Refresh the external **windowed** watchdog from a supervisor task that itself checks liveness of the timing, network, and housekeeping threads (kick only if all are healthy). Windowed → both stalls and runaway loops trip it. WDT timeout → POE_KILL in hardware.|
|`PFI` (PE8, EXTI8)                        |Power-fail early warning (ahead of on-chip PVD). ISR commits volatile critical state (last Vc, leap, calibration deltas, log cursor) to NVS and parks the DAC; the supercap/PVD covers the write window.                                                            |
|`RTC_TAMP` (PC13)                         |Tamper/intrusion → timestamped event in the audit log, optional secure-erase policy hook, SNMP trap. Handled by RTC/TAMP peripheral.                                                                                                                                |
|`POE_NCL/NCM/LCF` (PC7/PC2/PC3, EXTI7/2/3)|NCP1095 PD status (class/event/fault). Track PoE state, detected class vs measured draw, and fault latches; gate `RB_PWR_EN` if the granted budget can’t cover the Rb.                                                                                              |

### 2.8 Local UI

|Net (pin)                 |Software role                                                                                                                                          |
|--------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
|`BTN_INT_N` (PE0, EXTI0)  |MCP23017 U48 INTA: 7-button keypad (GPA0–6) + touch INT (GPA7) → read U48, debounce buttons / service FT6336U, dispatch to the nav state machine (§6.1).             |
|RGB LED (PD12/13/14, TIM4)|At-a-glance status: green = locked stratum-1, amber = warming/holdover, red = unlocked/fault, blue pulse = activity/identify. PWM for brightness/blend.|
|`FAN_PWM` (PE5, TIM15_CH1)|25 kHz fan drive from the thermal loop (§10.2). **Fail-safe**: undriven = full speed.                                                                  |
|`FAN_TACH` (PA15, EXTI15) |Edge-count over 1 s → RPM; stall/again alarm.                                                                                                          |
|Display + touch + proximity|ST7796 GUI (§2.5/§6.5), FT6336U touch wake (primary, §6.5), PIR proximity wake (secondary, §6.4).                                                     |

### 2.9 USB & debug

|Net (pin)                             |Software role                                                                                                                             |
|--------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------|
|`USB_DM/DP` (PA11/PA12)               |USB-FS device, **CDC-ACM** console (driverless). HSI48 + CRS clock. Carries shell, live logs, and **MCUmgr/SMP** firmware recovery (§8.4).|
|`USB_VBUS_SENSE` (PE2)                |Gate the D+ pull-up / host-connect detect (self-powered device). Polled (not EXTI — line shared with PC2).                                |
|`SWDIO/SWCLK` (PA13/PA14), `SWO` (PB3)|Development debug + `SWO` for high-rate trace/log during bring-up. Locked out in production via H5 debug-authentication (§9.1).           |

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

- The Rb (PRS10-class) is a **self-disciplined GPSDRO**: it locks its own 10 MHz to the buffered GPS 1PPS. Firmware does **not** run a steering loop for it; it sequences power, reads health over UART7, and watches `RB_LOCK`.
- Warm-up sequencing per §10.3 (after OCXO warm + supercaps charged) to stay within PoE budget; `RB_PWR_EN` → soft-start → INA228 #6 read-back verify → wait for `RB_LOCK` + EXTREF_MON in-band.
- Optional fine EFC trim over UART7 only if cross-checking against GPS PPS shows a residual the Rb’s own loop isn’t removing; default is hands-off.

### 3.5 Reference selection / mux state machine

States: `OCXO_ACTIVE` (boot default) ⇄ `RB_ACTIVE`, plus `EXTREF_ACTIVE` (house standard on input B).

- **OCXO → Rb** only when **both** `EXTREF_MON` reports a real, in-band ~10 MHz **and** `RB_LOCK` is asserted, stable past the debounce/hysteresis window.
- **Rb → OCXO** immediately if **either** condition drops (fail-safe). Hysteresis on re-engage prevents flapping.
- **Glitchless switch:** bridge SYSCLK to HSI, command `MUX_SEL`, allow the glitchless mux to cross over, reselect HSE, re-lock PLL, resume. Both inputs are 10 MHz so PLL config is unchanged; the disciplined PTP clock free-wheels on HSI for the few ms of changeover and is re-aligned after.
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

- Seven buttons on MCP23017 U48 Port A (GPA0–6); `BTN_INT_N (PE0/EXTI0)` fires on any Port A change → firmware reads the expander register, debounces (~20–30 ms), and emits press/long-press/repeat events. GPA7 on the same interrupt carries the FT6336U touch INT (serviced over I²C, no debounce).
- Logical map: **Up, Down, Left/Back, Right/Enter, Function/Menu, Power/Identify** (final assignment in §15). Nav is a screen-stack state machine: list scroll, value edit (where permitted), confirm dialogs for guarded actions.
- Locally editable: network basics (DHCP/static, IP), display brightness/timeout, identify (blink RGB + WSS hint), reference override (guarded), reboot, factory-reset (double-confirm). Deep config stays on the web/console — the panel is for status + field service.

### 6.2 Screen hierarchy

`Home` (time, stratum, lock, reference, SV count, holdover) → `Skyplot` → `Clocks` (OCXO Vc/temp/ADEV, Rb lock/health, active reference, holdover estimate) → `Network` (IP, NTP/PTP state, served rate) → `Power/Health` (rails, temps, fan, supercaps, PoE) → `Menu` (the editable items above) → `Alarms`. Left/Back pops; Right/Enter descends.

### 6.3 Skyplot rendering

- Polar plot, zenith center, horizon edge; per-SV marker placed by az/el from UBX-NAV-SAT, colored by constellation, sized/labelled by CNR, hollow = visible-not-used, filled = used in solution.
- **True-north orientation:** rotate the plot by the magnetometer heading (IIS2MDC), corrected by hard/soft-iron calibration (§10.4) and by **magnetic declination computed from the GPS fix** (WMM/IGRF); tilt-compensate with the accelerometer (LIS2DH12) — mandatory given the vertical board mount. If the compass is uncalibrated/disturbed, fall back to “GNSS-north” (az as reported) with a clear “north unverified” badge.
- Identical data feeds the web skyplot (§5.2) so both views agree.

### 6.4 Touch / proximity wake & RGB

- **Touch is the primary wake.** FT6336U INT (U48 GPA7 → `BTN_INT_N`) wakes to Home. Sleep/wake is a **`DISP_BL` PWM duty change only** — blank = duty→0, wake = duty→on — not a power or reset event: `DISP_EN`, the 5 V rail, and the rendered ST7796 frame all persist. Because touch rides the `DISP_EN`-gated rail, **DISP_EN stays asserted through the dimmed/blanked idle state**; drop it only for a deliberate display-off/recovery.
- **PIR is the secondary/ambient wake** (U47 GPB3, discrete): approach re-asserts `DISP_EN` (if dropped) and wakes to Home. Thresholds/timeouts configurable. PIR is on always-on 3V3_STM, so it is always live regardless of DISP_EN.
- RGB semantics per §2.8; brightness follows ambient/backlight policy; an “identify” command pulses blue to locate the unit in a rack.

### 6.5 Display driver

- ST7796 over SPI4, write-only, dedicated baud/mode profile (20–40 MHz), DMA frame/region pushes, partial-region updates for changing fields to bound traffic (≈1.2 MB/s @ 4 Hz full-frame worst case). Render thread is lowest-priority and pre-emptible; never blocks timing.
- **Power/reset sequencing:** assert `DISP_EN` (PC11) → RT9742 soft-starts `V_DISP_5V` and the PCA9306 enables; mask `V_DISP_EN_FAULT` (U47 GPB2) during the soft-start window; release `DISP_RST` (PA8, held low at boot by R161-class pulldown→ panel + touch reset) with the FT6336U reset-timing margin; init ST7796, then init FT6336U over the (now-translated) touch I²C. A `DISP_EN` low→high cycle fully cold-restarts a wedged panel/touch (module 74LVC245 has Ioff → no sneak path keeps it alive); drive DISP_CS/DC/BL to a defined state during the off window.

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

F9T firmware updated over USART3 (PD8/PD9) using `GPS_SAFEBOOT_N (PD15)` + `GPS_RESET_N (PD11)` for safeboot recovery. The app exposes a guarded `gnss fw` flow (web + console) that streams the u-blox image, verifies the receiver reports the new version, and restores timing config afterward.

### 8.6 Asset / Rb / config updates

Web SPA bundle and MIB/Zabbix template update with the firmware image (versioned together). Rb (PRS10) firmware is vendor-flashed over UART7 only if the module supports it (guarded, rare). Config import is a signed JSON applied via validate-then-commit (§5.5).

-----

## 9. Security architecture

### 9.1 Root of trust & key hierarchy

- Hardware RoT = STiRoT (immutable) → MCUboot trust anchor → application. Debug ports closed in production via H5 **debug authentication** (re-open only with a signed challenge). Readout protection (RDP/product state) at the highest level consistent with field-update needs.
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

`housekeeping` polls the I²C devices (§2.4) and the internal ADC (VREFINT for true VDDA, die temp, VBAT, plus supercap channels PA6/PB1, OCXO Vc PA3) on the cadences listed, applies per-sensor calibration offsets, and publishes a health block consumed by GUI/SNMP/console/display. INA228 alerts via `INA_ALERT (PC12)` trigger immediate re-read + alarm evaluation.

### 10.2 Thermal / fan loop

PI loop on enclosure temp (TMP117 #2) with oscillator temp (TMP117 #1) and die temp as inputs; drives `FAN_PWM (PE5)` 25 kHz; RPM from `FAN_TACH (PA15)`. Hysteresis + minimum duty for bearing life; **fail-safe full-speed** if the loop or MCU stalls (undriven PWM = max). Fan stall/under-speed → alarm + trap.

### 10.3 Power sequencing & faults

- **Boot order:** main 3.3 V (3V3_STM — housekeeping I²C, NOR, expanders all come up here, no gating) → GPS (`GPS_PWR_EN`) + antenna bias (`ANT_BIAS_EN`) → display/touch (`DISP_EN`, when UI is needed) → OCXO warm → (when warranted) `RB_PWR_EN` with INA228 #6 read-back verify. Sequencing keeps the cold-start peak inside the granted PoE budget (§ pin map 6); if PoE class can’t cover the Rb, defer/deny `RB_PWR_EN` and flag. The display rail is deferrable/low-priority — drop `DISP_EN` first under PoE pressure.
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
|Ext-ref front-end misconfig / no output|EXTREF_MON invalid after config|Re-apply stored profile once; if still invalid, hold on OCXO, do not select B, flag config fault + trap. Never trust an unverified front-end output.|
|OCXO oven/DAC fault  |INA228 #5; Vc cmd≠sense (PA3)  |Mark OCXO unhealthy; prefer Rb if available; alarm.                  |
|I²C bus wedge        |transaction timeout / SCL stuck|Toggle `I2C_BUF_EN (PA9)`, re-init; if persistent, `EXP_RESET (PC0)` hard-reset the expanders (housekeeping is on always-on 3V3_STM — no rail-cycle path).|
|NOR hang             |LittleFS error                 |`NOR_RESET (PE10)`; degrade to RAM-only logging; alarm.              |
|Display hang         |SPI/driver watchdog            |`DISP_RST (PA8)` re-init; UI optional, never blocks timing.          |
|Over-temp            |TMP117/die                     |Fan max; if exceeded, shed Rb, then thermal `POE_KILL`.              |
|Power-fail           |`PFI (PE8)`                    |Fast-save + park (§10.3).                                            |
|Thread stall         |supervisor liveness            |Withhold `WDT_KICK` → external WDT → `POE_KILL` cold cycle.          |
|Bad firmware         |post-update health             |MCUboot auto-revert to confirmed image.                              |
|Tamper               |`RTC_TAMP (PC13)`              |Audit + trap; optional key zeroize.                                  |

Principle: timing service degrades gracefully (holdover, reference fallback, peer NTP) before it ever serves wrong time; unrecoverable states cold-cycle rather than hang.

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
- Rb EFC steering: hands-off (self-discipline only) vs optional firmware fine-trim — default hands-off; confirm PRS10 control surface.
- External-ref front-end (§2.2) config policy: per-source default routing (DB9/FE-5680A = 50 Ω term + LTC6957 branch; SMA = comparator branch + term per source impedance until characterized), persisted in NVS; auto-probe (apply route → check EXTREF_MON → if invalid, try the comparator branch → flag if still invalid) vs operator-selected source only. Both slicers are always populated; selection is runtime via `REF_SLICER_SEL` — no build option.
- Leap-second handling policy at the second of insertion for NTP (smear vs step) and PTP (step) — define per service.
- Certificate lifecycle: manual/CSR only, or add ACME — phase decision.
- Anti-rollback counter budget and field-update policy (how many versions, recovery if exhausted).