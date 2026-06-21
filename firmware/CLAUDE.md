# STS1000 Firmware — Claude Code Context (`firmware/`)

Scoped context for work under `firmware/`. The root `../CLAUDE.md` carries the
project-wide rules; `../docs/ntp_server_software_spec.md` is the **authoritative firmware
spec** and `../docs/ntp_server_peripheral_map.md` is the **authoritative hardware/pin
reference**. This file is the working preamble for the firmware tree, not a re-statement
of those docs. Where this file and the spec disagree, the spec wins.

> Status: greenfield. The application is not yet scaffolded — treat the commands and
> layout below as the intended target and adjust this file to match reality as it lands.

---

## Platform stack (decided)

| Layer | Selection |
|---|---|
| RTOS | **Zephyr RTOS** (devicetree expresses the pin map 1:1) |
| Secure-boot root | **STM32H5 STiRoT** (immutable ST RoT) → OEMuRoT/MCUboot |
| Bootloader / image mgmt | **MCUboot** swap-move, two slots, + **MCUmgr/SMP** (serial recovery over USB) |
| Crypto | **PSA Crypto** over **mbedTLS 3.x**; H5 AES/PKA/HASH; **ATECC608B** secure element via Microchip **CryptoAuthLib** (ECC P-256 device key, non-exportable) |
| TCP/IP | Zephyr native dual-stack IPv4/IPv6, `SO_TIMESTAMPING` → ETH-MAC PTP unit |
| Filesystem | **LittleFS** on SPI-NOR + **NVS** in internal flash for small critical settings |
| Web | Embedded HTTPS (`http_server`) serving a static SPA + REST + WSS |

**Fallback platform (documented, not chosen):** FreeRTOS + lwIP + mbedTLS + MCUboot —
adopt only if the custom NTS-server/SNMP effort on Zephyr proves larger than the lwIP
path. The behavior-level spec is platform-neutral.

### Custom-effort gaps (largest work in the project)
1. **NTS-capable NTP *server* + PTP grandmaster.** Zephyr ships NTP/PTP *clients* and gPTP
   (802.1AS) but no production NTS server nor a turnkey 1588 grandmaster. Both are
   application services over the socket API with MAC hardware timestamping. Budget for it.
2. **SNMP agent.** No maintained Zephyr agent — implement a compact SNMPv2c/v3 module or
   port lwIP `apps/snmp`.

---

## Board bring-up

- **MCU:** STM32H563VIT6, LQFP100. **PE1 / PB11 are not bonded — never reference them in DT.**
- A **custom board definition** is required (`boards/.../sts1000_meridian/`): `.dts`, board
  `Kconfig`, `_defconfig`, and pinctrl. DT net names **must** match the canonical net names
  in the peripheral map (`OSC_IN`, `MUX_SEL`, `RB_PWR_EN`, `INA_ALERT_INT_N`, …).
- **System clock:** PH0 is **HSE bypass** fed by the clock mux → PLL → 250 MHz SYSCLK. PLL
  config is identical for OCXO and Rb (both 10 MHz). USB-FS clock is **HSI48 + CRS** (no USB
  crystal). LSE 32.768 kHz on PC14/PC15 for the RTC.
- **In-tree drivers exist** for most of the BOM: TMP117, INA2xx, IIS2MDC, LIS2DH12, ST7796,
  FT5xx6/FT6336, SPI-NOR. ATECC608B is CryptoAuthLib, not a Zephyr sensor driver.

---

## Thread model (spec §1.2) — timing path is privileged

Lower number = higher Zephyr priority.

| Thread | Prio | Trigger | Role |
|---|---|---|---|
| `pps_capture` | ISR + coop | PPS edge (PA0/TIM2) | latch, sawtooth-correct, post phase sample (µs budget) |
| `discipline` | 4 | 1 Hz PPS-paced | PI/FLL loop; DAC (PA4); reference/holdover state machine |
| `ptp` | 5 | event | 1588 engine, BMCA, Sync/Follow_Up/Delay |
| `gnss` | 6 | UART3 + 1 Hz | UBX parse, survey-in, NAV-SAT, antenna supervisor, leap |
| `ntp_server` | 8 | socket | NTP/NTS service, RX/TX HW timestamps |
| `net_mgmt` | 10 | event | DHCP, mDNS, link, ND/ARP |
| `web`/`tls`, `snmp` | 12 | socket | lowest network prio |
| `housekeeping` | 14 | 1–4 Hz | I²C sensors, fan loop, supercaps |
| `console` | 14 | CDC/UART | shell, log tail, SMP recovery |
| `ui_local` | 15 | 4–10 Hz | display, buttons, RGB, proximity |
| `logger` | 16 | queue | async log → RAM ring + NOR + syslog |

**Hard rule:** no `web`/`snmp`/`console`/`ui` thread may hold a mutex on timing state. They
read a lock-free, double-buffered telemetry snapshot that `discipline` publishes at 1 Hz
(spec §3.8 — the single source of truth; services never compute their own quality).

---

## Timing engine invariants (spec §3) — get these right

- **Sawtooth correction is mandatory.** Read UBX-TIM-TP each second and apply `qErr` to the
  PPS capture; uncorrected sawtooth dominates short-term error. Subtract calibrated antenna
  cable + board PPS-routing delay. Median/MAD-gate PA0 vs PC6 before the edge enters the loop.
- **OCXO loop:** FLL+PLL, PI form; τ configurable 10–1000 s (default ~100–300 s). Actuator =
  DAC1_OUT1 (PA4), **slew-limited, center 1.65 V, full-scale = ±0.4 ppm pull**. Written by
  **`discipline` only** — never any other thread. TMP117 #1 tempco feed-forward.
- **Advertise stratum-1 only when** phase error + variance are under threshold for N seconds,
  **OCXO warm** (INA228 #5 + TMP117 #1), **and** GPS time-locked. Hold off until all true.
- **Reference state machine (§3.5):** `OCXO_ACTIVE` (boot) ⇄ `RB_ACTIVE`.
  - OCXO→Rb only when **both** `EXTREF_MON` (PB14/TIM12) reports a real in-band ~10 MHz
    **and** `RB_LOCK` (PB13) is asserted, stable past hysteresis.
  - Rb→OCXO **immediately** if **either** drops (fail-safe); hysteresis on re-engage stops flap.
  - **Glitchless switch:** bridge SYSCLK→HSI, command `MUX_SEL` (PB6), cross over, reselect
    HSE, re-lock PLL. Never switch the live HSE feed without the HSI bridge; CSS is the HW
    failsafe. OCXO is never powered down — its DAC holds last-good as a warm fallback.
- **Rb is a self-disciplined GPSDRO.** Firmware does **not** steer it: sequence power, read
  health over UART7, watch `RB_LOCK`. Default EFC trim = hands-off.
- **Holdover (§3.6):** freeze actuator / trust Rb flywheel; grow NTP root dispersion per the
  characterized drift; set PTP `clockClass`/`clockAccuracy`/`offsetScaledLogVariance`; expose
  estimated time error + time-to-demotion; re-converge with rate-limited pull-in (no step).

---

## Power-sequencing & fault rules firmware owns

- **Stagger warm-ups** to stay in the PoE budget: OCXO first → wait OCXO warm + supercaps
  charged → assert `RB_PWR_EN` (PB7) → soft-start → INA228 #6 read-back verify in range →
  wait `RB_LOCK` + EXTREF_MON in-band. Out-of-range → drop `RB_PWR_EN`, flag.
- **Rb OV latch is autonomous (26 V).** Firmware only observes `RB_OV_DET` (PE3, polled) and
  clears via `RB_OV_RESET` (PD3). Not on the I/O expander.
- **Fan fail-safe to cooling:** 4-wire fan runs full speed on float/100 % PWM. Keep
  `FAN_PWM` (PE5/TIM15_CH1) resting state = max airflow so a hung MCU can't cook the box.
- **Fault interrupts (open-drain wire-OR, active-low):** `PG_INT_N` (PA10, U55 PG+EN-fault),
  `BTN_INT_N` (PE0, U54 buttons + touch INT), `INA_ALERT_INT_N` (PC12, U54 INA ALERTs). On
  IRQ, read the expander port to identify the bit, then the implicated device for cause.
- **I²C wedge recovery:** toggle `I2C_BUF_EN` (PA9, LTC4311 EN) to isolate/recover a stuck bus.
- Front-end control reduced to one bit: **`REF_TERM_EN` (PC10)** (single LTC6752xS5 slicer;
  `REF_SLICER_SEL`/`REF_SRC_SEL` no longer exist — see `../docs/ntp_server_rf_frontend.md`).

---

## GNSS engine (spec §3.7)

UBX only (NMEA off), persisted to F9T BBR + flash: timing mode, GPS+Galileo+GLONASS+BeiDou,
elevation mask, NAV-PVT/NAV-SAT/NAV-SIG/NAV-TIMELS/TIM-TP/MON-RF/MON-HW @ 1 Hz. **Survey-in**
on first install → store ECEF → run fixed-position timing mode (re-survey is an explicit
operator action). Antenna supervisor fuses UBX-MON-RF + `GPS_ANT_OFF_MON` (PD4) + antenna
INA228 #4; persistent short → drop `ANT_BIAS_EN` (PC9) + alarm. Track leap from NAV-TIMELS;
expose LI bits (NTP) and PTP leap flags.

---

## Memory & boot (spec §1.3)

- Internal 2 MB flash: STiRoT → MCUboot → **slot-0 (active)** / **slot-1 (staged)** + NVS +
  anti-rollback counters + device-key handle.
- SPI-NOR (LittleFS): web SPA bundle, almanac/ephemeris cache, calibration tables, discipline/
  health log ring, sealed NTS master keys, syslog spool, MCUboot staging if slot-1 undersized.
- First boot of a new image is **test** until firmware self-confirms (§8.3), else auto-revert.

---

## Intended toolchain & commands (adjust once scaffolded)

```sh
# Zephyr west workspace + SDK assumed; custom board under boards/
west build -b sts1000_meridian firmware/app
west flash                      # via on-board SWD (PA13/PA14/PB3)
west build -t menuconfig        # Kconfig
# MCUboot signing + SMP recovery over the USB CDC console (PA11/PA12) for field update
```

Provide a `west.yml` manifest pinning Zephyr + MCUboot + CryptoAuthLib revisions before the
first real build, and record the Zephyr SDK version here.

---

## Open firmware decisions (spec §15 — resolve before sizing)

- ETH PTP **auxiliary-snapshot internal trigger** (RM0481) for direct PPS↔PTP capture, else
  lock the TIM2↔PTP software correlation.
- **One-step vs two-step PTP** (does the H5 MAC do HW one-step for the chosen profiles?).
- Platform commit: Zephyr custom services vs FreeRTOS+lwIP fallback (sizes the biggest effort).
- SNMP: custom compact agent vs ported lwIP `apps/snmp`.
- Web SPA framework + brotli size budget (must fit NOR with assets + logs).
- Leap-second at the second of insertion: NTP **smear vs step**, PTP step — per service.
- Final 6-key button mapping + long-press semantics.
- Anti-rollback counter budget + field-update/recovery policy; cert lifecycle (manual/CSR vs ACME).

---

## Conventions

- **Editor: vi only** for CLI text editing — never pico/nano.
- Spec keywords are normative: **MUST/SHALL**, **SHOULD**, **MAY** (spec §0).
- Net names in DT and code match the canonical peripheral-map names exactly.
- Validation/CI expectations live in spec §14 (ADEV/MDEV, sawtooth proof, holdover model,
  protocol interop, 10k+ req/s soak proving the timing path is unperturbed, HIL PPS injection).
