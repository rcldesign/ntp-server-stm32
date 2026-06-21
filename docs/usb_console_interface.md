# USB Console — Connector, Protection & Device Interface Design Notes

Covers the front-panel **USB 2.0 Type-C** console port: the receptacle, the CC sink
presentation, the data-pair routing into the STM32H563 embedded FS PHY, the
flow-through ESD array, the VBUS-sense divider, the device-supply decoupling, and the
firmware behavior for enumeration, the shell/log channel, and firmware recovery.

This is the **out-of-band management path** — a driverless CDC-ACM virtual serial port
that carries the console shell, live logs, and MCUmgr/SMP firmware recovery,
independent of the Ethernet/PoE network. It is **not** a power input: the unit is
self-powered from PoE, and VBUS is sensed only, never drawn from.

Cross-refs: peripheral-map §9/§10/§11; software-spec §2.9/§8.4/§9.1.

---

## 1. Purpose

The STM32H563 integrates a USB 2.0 **full-speed device** controller with an embedded
transceiver. This subsystem exposes it on a USB-C receptacle and solves four
problems:

1. **Driverless host attach.** CDC-ACM enumerates as a standard virtual COM port on
   Windows/macOS/Linux with no host driver, giving a guaranteed management channel
   even when the network is down or misconfigured.
2. **Self-powered enumeration compliance.** A self-powered device must not present its
   D+ pull-up until VBUS is present. VBUS is divided down and sensed on a GPIO; the
   pull-up (internal to the PHY) is enabled in firmware only after VBUS is detected.
3. **Field-port robustness.** The connector is externally cabled, so D+/D−/VBUS need
   ESD protection at the receptacle.
4. **Source-side power negotiation.** A USB-C source applies VBUS only after it sees a
   sink advertisement. Discrete Rd resistors on both CC lines present a default-USB
   sink so VBUS appears without any PD/UCPD firmware.

No hardware UART is used: the only free UART-capable pins (PA11/PA12 → UART4) collide
with the GPS-corrections UART4 (PD0/PD1), and CDC-ACM is driverless.

**Sourcing policy:** the receptacle and ESD array are ordinary connector/protection
parts — not networking silicon and not answer-providing software — so they are outside
both China-origin bans.

---

## 2. MCU pin map

| STM32 pin | Net | Dir | Role |
|---|---|---|---|
| PA12 | `USB_DP` | bidir | USB-FS D+, embedded PHY |
| PA11 | `USB_DM` | bidir | USB-FS D− |
| PE2 | `USB_VBUS_SENSE` | in | divided VBUS; **polled, not EXTI** |

`USB_VBUS_SENSE` is polled because EXTI2 is occupied by `POE_NCM` (PC2). SWD debug
(PA13 SWDIO / PA14 SWCLK / PB3 SWO) is the separate development interface and is locked
out in production via H5 debug authentication (software-spec §9.1).

---

## 3. Connector — J10 (USB4120-03)

USB 2.0 Type-C receptacle, 16 contacts, vertical SMT. The USB-2.0 variant omits the
SuperSpeed pairs (A2/A3/A10/A11, B2/B3/B10/B11), leaving VBUS, CC, D±, SBU, GND, and
the four shell tabs.

### 3.1 Pin consolidation

| Net | J10 pins | Notes |
|---|---|---|
| `USB_VBUS` | A4, A9, B4, B9 | all four VBUS contacts tied |
| `GND` | A1, A12, B1, B12 | all four GND contacts → board GND |
| `USB_DP` | A6 (D1+) **+** B6 (D2+) | both orientations paralleled |
| `USB_DM` | A7 (D1−) **+** B7 (D2−) | both orientations paralleled |
| CC1 | A5 | → R211 5.1 kΩ → GND |
| CC2 | B5 | → R212 5.1 kΩ → GND |
| SBU1 / SBU2 | A8 / B8 | NC (unused on USB 2.0) |
| SHIELD | S1, S2, S3, S4 | → shell bond network (§7) |

The device has no orientation mux, so the two D pairs are simply paralleled: a cable
presents only one pair per insertion orientation, and the un-mated pair floats. This is
the standard USB-2.0 Type-C device connection.

Connector ratings (datasheet): 5 A collective VBUS, 6.25 A collective GND, 48 V, op-temp
**−25 to +85 °C**, 20 k mating cycles.

---

## 4. CC — sink presentation

A USB-C source enables VBUS only after detecting a sink. Each CC line carries its own
Rd pulldown to advertise a default-USB sink (Type-C `Rp`/`Rd` model).

| Designator | Value | Connection |
|---|---|---|
| R211 | 5.1 kΩ 1% | CC1 (A5) → GND |
| R212 | 5.1 kΩ 1% | CC2 (B5) → GND |

- **Separate Rd per CC** — CC1 and CC2 are **not** tied together (tying them defeats
  orientation detection at the source).
- **5.1 kΩ is the Type-C sink value**, not the 56 kΩ legacy pull-up — confirm the value
  in BOM.
- No VCONN, no Ra, no PD stack. CC is not routed to the MCU; the H563 UCPD peripheral is
  unused. This keeps the console robust and stateless.

---

## 5. Data pair — D+/D−

`USB_DP` (PA12) and `USB_DM` (PA11) route from J10 through the ESD array (§6) to the MCU
as a **90 Ω differential pair**, kept short.

The embedded FS PHY makes the external circuit minimal:

- **No external series resistors** — the FS driver matching impedance is embedded in the
  pad. Adding external series R degrades the eye.
- **No external 1.5 kΩ D+ pull-up** — the pull-up is internal to the PHY and is enabled
  by firmware (§9.2). There is no pull-up gating hardware; "gate the D+ pull-up" is a
  software action keyed off `USB_VBUS_SENSE`.

---

## 6. ESD — U64 (TPD4E05U06DQAR), flow-through

Quad-channel, ultra-low-cap (0.3 pF I/O–I/O), 5.5 V working, leadless **flow-through**
DFN. Placed hard against the receptacle, before any branch.

### 6.1 Channel map

| Channel | Pins (in ↔ out) | Net |
|---|---|---|
| 1 | 1 ↔ 10 | `USB_DP` |
| 2 | 2 ↔ 9 | `USB_DM` |
| 3 | 4 ↔ 7 | `USB_VBUS` |
| 4 | 5 ↔ 6 | unused (open) |
| GND | 3, 8 | GND |

### 6.2 Flow-through note (critical for layout)

The datasheet labels pins 6/7/9/10 as **NC** in the package pin table, but they are the
flow-through partners of pins 1/2/4/5 — TI's documentation and applications support
confirm these pins are to be shorted to the corresponding line. The signal must
physically **pass through** the part: receptacle pad → one side → through U64 → other
side → MCU/divider. A net-list short alone (both pins on the same net) satisfies ERC but
forfeits the protection benefit if the layout stubs the part off to the side.

- Channel 4 (pins 5/6) is one channel, correctly left fully open.
- **VBUS headroom is tight:** VRWM 5.5 V vs VBUS_max 5.25 V → 0.25 V margin. Valid only
  for default-USB power (5 V). If CC is ever rerouted to a PD sink that could request
  9/15/20 V, this clamp must be re-rated.
- CC1/CC2 are unprotected (channel 4 is a single channel, cannot cover two CC pins);
  acceptable for a console, with the 5.1 kΩ Rd limiting any event.

---

## 7. SHIELD / shell bond

The four shell tabs (S1–S4) are both the EMI/ESD shield and the SMT mechanical
retention. They are kept **separate from the four GND signal contacts** — the GND pins
are the D±/VBUS signal return to board GND; the shell is bonded to chassis through a
high-impedance network so a host-side ground loop is broken at DC while ESD/HF still has
a low-impedance return.

| Designator | Value | Connection |
|---|---|---|
| R213 | 1 MΩ | SHIELD → GND |
| C178 | 4.7 nF | SHIELD → GND (‖ R213) |

---

## 8. VBUS sense & supply

### 8.1 VBUS-sense divider → PE2

VBUS divided to a 3.3 V-safe level for the GPIO. Target: clean logic-high at VBUS_min,
≤ VDD at VBUS_max.

| Designator | Value | Connection |
|---|---|---|
| R209 | 100 kΩ 1% | `USB_VBUS` → `USB_VBUS_SENSE` |
| R210 | 121 kΩ 1% | `USB_VBUS_SENSE` → GND |
| C179 | 100 nF | `USB_VBUS_SENSE` → GND (filter/debounce) |

Divider ratio k = 121 / 221 = **0.547**. V(PE2) = VBUS · 0.547:

| VBUS | V(PE2) | |
|---|---|---|
| 5.25 V | 2.87 V | ≤ 3.0, safe |
| 5.00 V | 2.74 V | high |
| 4.40 V | 2.41 V | > VIH (~2.31 V at VDD = 3.3 V) ✓ |
| 0 V | 0 V | absent |

Load ≈ 24 µA at 5.25 V (negligible; self-powered). Filter τ = (R209‖R210)·C179 =
54.7 kΩ · 100 nF ≈ **5.5 ms**. The 100 kΩ top resistor also limits injection into PE2
when VBUS is present while the MCU is unpowered (cable plugged into a depowered unit) —
current is held within the U64 clamp's capacity. *(C179 designator assigned here —
confirm against the schematic refdes.)*

### 8.2 Device supply — VDD33USB

The STM32H563**V**IT6 is LQFP100 (high pin count) and therefore has a dedicated
**VDD33USB** pin powering the USB transceiver. Tied to **3V3_STM** (always-on) and
decoupled per AN4879: **100 nF + 1 µF**. Tying to the same 3.3 V rail auto-satisfies the
sequencing rule (VDD33USB ≤ VDD during ramp; last-on / first-off). *(Decoupling sits on
the MCU power sheet — confirm placement adjacent to the pin.)*

---

## 9. Firmware implications

### 9.1 Clocking

USB-FS clock is **HSI48 + CRS** — the on-chip 48 MHz RC oscillator, frequency-trimmed by
the Clock Recovery System against USB SOF packets. **No external USB crystal.** This is
independent of the 250 MHz SYSCLK/timing tree. Enable CRS with USB SOF as the sync
source before/at `usb_enable()`.

### 9.2 VBUS-gated enumeration

The device is self-powered, so the D+ pull-up must not assert until VBUS is present:

1. Configure PE2 as a digital input; poll it (no EXTI — EXTI2 is taken by `POE_NCM`).
   Apply a debounce in firmware in addition to the ~5.5 ms RC.
2. On VBUS-present, bring up the USB stack / enable the internal DP pull-up (in Zephyr,
   gate `usb_enable()` on the VBUS-detect state).
3. On VBUS-absent, tear down the stack / release the pull-up so the device de-enumerates
   cleanly.

Treat PE2 as a **digital present/absent** signal. If continuous VBUS *measurement* is
ever wanted (telemetry), verify whether PE2 maps to an ADC channel before relying on it;
otherwise keep it digital. VBUS = V(PE2) · 221/121 = V(PE2) · 1.826 if measured.

### 9.3 CDC-ACM console

- Class: USB CDC-ACM (driverless virtual COM). Single device function.
- Carries: interactive shell, live log stream, and the firmware-recovery transport.
- DTR/RTS line-state and baud are cosmetic for a virtual COM but should be accepted and
  ignored gracefully; do not gate output on DTR.

### 9.4 Firmware recovery (MCUmgr / SMP)

The same CDC-ACM endpoint carries the **MCUmgr/SMP** transport for image upload and swap
under MCUboot (software-spec §8.4). This is the recovery path when the network image is
bad: attach USB, push a signed image over SMP, reset into the new slot. Keep the SMP
transport available early in boot (before the full application), so a bricked
application is still recoverable over USB.

### 9.5 Production lockout

SWD (PA13/PA14/PB3) is development-only and is disabled in production via H5 debug
authentication (software-spec §9.1). USB CDC-ACM remains the production management/
recovery channel; ensure the shell's privileged operations are authenticated
independently of the debug port.

---

## 10. Designator summary

| Ref | Part / value | Function |
|---|---|---|
| J10 | USB4120-03 (USB 2.0 Type-C, vertical SMT) | console receptacle |
| U64 | TPD4E05U06DQAR | flow-through ESD on D+, D−, VBUS (3 of 4 ch) |
| R211 | 5.1 kΩ 1% | CC1 Rd (sink advertise) |
| R212 | 5.1 kΩ 1% | CC2 Rd (sink advertise) |
| R209 | 100 kΩ 1% | VBUS-sense divider top |
| R210 | 121 kΩ 1% | VBUS-sense divider bottom |
| C179 | 100 nF | `USB_VBUS_SENSE` filter/debounce |
| R213 | 1 MΩ | shell → GND DC isolation |
| C178 | 4.7 nF | shell → GND HF bond |
| — | 100 nF + 1 µF | VDD33USB decoupling (MCU power sheet) |

---

## 11. Open items / verify

- [ ] Confirm **C179** refdes against the schematic (assigned here as next-free).
- [ ] Confirm **Rd = 5.1 kΩ** sink value in BOM (not 56 kΩ).
- [ ] Layout: route U64 as a **physical flow-through** (connector → through part → MCU),
      not a stub; place hard against J10.
- [ ] Confirm VDD33USB decoupling (100 nF + 1 µF) is placed adjacent to the pin on the
      power sheet, tied to 3V3_STM.
- [ ] D+/D− as a 90 Ω differential pair; no series R, no external pull-up.
- [ ] Confirm VBUS stays ≤ 5.25 V (default-USB only; CC not routed to a PD sink) so
      U64's 5.5 V VRWM keeps margin.
- [ ] Decide whether PE2 needs ADC mapping for VBUS telemetry, or stays digital-only.
