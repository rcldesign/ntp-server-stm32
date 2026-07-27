# STS1000 "Meridian" — Field Maintenance Tool (FMT) Specification

**Scope.** Cross-platform host application (Windows / macOS / Linux) + the device-side
maintenance protocol it speaks, over the USB CDC-ACM console port. Covers full peripheral
control, live monitoring/diagnostics, GNSS observation (NMEA/UBX, satellite state),
calibration, log capture, **a live mirror of the device's own front panel**, and
**firmware update for every updatable IC on the board** — everything needed to commission,
maintain, and debug an STS1000 with nothing but a laptop and a USB cable.

**Status.** The as-built firmware capabilities the tool surfaces are catalogued in §13 and
are in tree. The MP device-side layer of §3–§9 is under active implementation; §11 tracks
its per-item state. The host application is specified here and not yet built;
`firmware/tools/meridian_ctl.py` is the reference client for the subset it covers (§13.2).

**Authority.** `sts1000_firmware_hardware_interface.md` wins on pins and addresses;
`ntp_server_software_spec.md` owns firmware behavior; `firmware/ARCHITECTURE.md` owns
firmware structure. **At runtime the device-served manifest (§4) is authoritative** — the
host hardcodes nothing.

**Conclusions first:**

1. **Transport is the USB-FS CDC-ACM port** (PA11/PA12, HSI48; PE2 VBUS sense). No new
   hardware. USB Full-Speed is the H563's ceiling (no HS PHY) — 12 Mbit/s wire rate. The
   CDC line coding is a no-op (virtual). This is the fastest link this silicon offers.
2. **One physical port, several logical planes**, multiplexed by a lightweight frame layer
   (§3): command/response, subscription streams (telemetry, NMEA, UBX, logs, PPS
   residuals, events, panel mirror), and firmware-transfer tunnels. The human shell
   remains reachable as a plain-text fallback mode.
3. **The host hardcodes nothing about the board.** At connect it downloads a **capability
   manifest** (§4) — the device's self-description of every controllable and observable
   object, with types, units, limits, guard class and schematic designator. The UI is
   generated from it, so firmware changes surface without a tool update.
4. **Safety model = guard classes + maintenance session + dead-man revert** (§5).
   Observation is free; mutation is gated. Loss of the link reverts every override to
   firmware-automatic within 2 s.
5. **Firmware update covers the whole board** (§9): the STM32 application, the ZED-F9T
   GNSS receiver, and the FE-5680A rubidium where the variant supports it — plus a
   complete component inventory that enumerates the ICs which have no field-update path,
   so the operator sees the real state of every device.
6. **Host stack: single-binary, no runtime dependencies** (§10). Reference
   implementation: Rust + Tauri (GUI) over a shared `sts1000ctl` CLI/library crate;
   serial via `serialport-rs`. Complies with the project sourcing policy (US/EU-origin
   OSS only).

---

## 1. Definitions

| Term | Meaning |
|---|---|
| FMT | Field Maintenance Tool — the host application (GUI + CLI) |
| MP | Maintenance Protocol — the framed protocol of §3 |
| MCP | Meridian Console Protocol — the as-built binary console protocol (§13.2); MP's predecessor and still supported |
| Manifest | Device-served self-description of the control/monitor surface (§4) |
| Object | One controllable/observable entity (rail, GPIO, PWM, sensor channel, service) |
| Override | Host-commanded state that supersedes firmware's automatic control of an object |
| Session | Authenticated maintenance context; owns all overrides; bounded by dead-man |
| Guard class | Per-object safety tier G0–G3 (§5.2) |
| Mirror | Live replica of the device's own front panel — screen contents and lamp state (§8) |

> **As-built correction.** Earlier drafts referenced a `sts1000_h563zi_migration.md`
> migration document. No such document exists; the LQFP144 pin contract is in
> `sts1000_firmware_hardware_interface.md`, which is authoritative.

---

## 2. Transport layer

### 2.1 USB device

| Item | Spec |
|---|---|
| Controller | STM32H563 USB-FS device, HSI48 (no crystal; **no CRS** — Zephyr 4.2 has no STM32H5 CRS support, so the clock runs on the ±1 % factory trim, which is inside USB-FS tolerance) |
| Class | CDC-ACM — driverless on Win10+/macOS/Linux |
| Composition | **Two CDC-ACM interfaces as built:** ACM0 = human shell + log console, ACM1 = binary maintenance channel (MCP today, MP per §3). MCUboot's serial-recovery build presents its own single CDC. |
| VID/PID | Obtain via pid.codes or a purchased VID; do not ship a vendor default. Distinct PIDs for application (0x1000) and recovery (0x1001) so the host can tell them apart |
| Strings | Manufacturer, Product "STS1000 Meridian", Serial = device serial (ATECC608B identity when present, else the STM32 unique ID) |
| VBUS | Sensed on **PE2 as a digital input** (polled). PE2 has **no ADC channel** — presence detect only, not VBUS measurement |
| Speed | FS, 12 Mbit/s, 64-byte bulk endpoints |
| Line coding | Ignored by the device (virtual). Host may open at any rate |
| DTR | DTR assert = host attached. DTR drop / suspend / disconnect = dead-man trigger (§5.4) |

> **Why two CDC interfaces, not one framed port.** An earlier draft chose a single CDC
> with everything multiplexed. As built there are two, because the human shell and the
> binary plane have genuinely different framing and a shared port forces a mode switch
> that strands a human operator mid-session. The frame mux of §3 still applies to the
> binary interface, which carries all the streams. Endpoint budget on the H563 accommodates
> two ACMs comfortably.

### 2.2 Modes

| Mode | Entered by | Content |
|---|---|---|
| Shell | default on ACM0 | Human line shell (`sts …` commands, §13.3) |
| MCP | default on ACM1 | As-built binary protocol (§13.2) |
| MP | `mp enter` on the shell, or the magic `\x01MP1\x02` on either binary interface | Framed protocol §3 |
| SMP recovery | MCUboot when no valid application boots | Raw MCUmgr/SMP over MCUboot's own CDC |

FMT connect sequence: enumerate by VID/PID → open the binary interface → send the MP
magic → on `hello`, proceed; else try MCP `HELLO`; else probe SMP (device is in the
bootloader → offer recovery flash); else fall back to shell scraping.

---

## 3. Maintenance Protocol (MP)

### 3.1 Frame layer

COBS-encoded frames, `0x00` delimiter (unambiguous resync on a byte stream):

```
[COBS( channel:u8 | flags:u8 | payload[≤2048] | crc16-ccitt:u16 )] 0x00
```

| Channel | Direction | Content |
|---|---|---|
| 0x00 | H↔D | Control plane: JSON-RPC 2.0, one message per frame |
| 0x01 | D→H | Telemetry stream (CBOR, §7.1) |
| 0x02 | D→H | NMEA stream (raw ASCII sentences, verbatim) |
| 0x03 | D→H | UBX stream (raw binary UBX frames, verbatim) |
| 0x04 | D→H | Log stream (structured records, CBOR) |
| 0x05 | D→H | PPS/discipline stream (per-second loop record, CBOR, §7.2) |
| 0x06 | H↔D | SMP tunnel (MCUmgr frames verbatim) |
| 0x07 | H↔D | GNSS passthrough (bidirectional raw USART3 tunnel, §9.3) |
| 0x08 | H↔D | Rb passthrough (bidirectional raw UART7 tunnel, §9.4) |
| 0x09 | D→H | Event/alarm stream (CBOR, edge-triggered) |
| 0x0A | D→H | **Panel mirror stream** (CBOR, §8) — *added; not in the original draft* |
| 0x0B–0x1F | — | Reserved |

`flags` bit0 = more-fragments; reassembly is per-channel. CRC failures are dropped
silently — the control plane has request IDs and host retry, streams tolerate loss.

### 3.2 Control plane (channel 0)

JSON-RPC 2.0; every mutating call carries the session token (§5.3).

| Method | Args | Returns |
|---|---|---|
| `hello` | host tool version | fw version, hw rev, serial, manifest hash, MP version, session requirements |
| `manifest.get` | cursor | manifest fragment (§4) |
| `session.open` / `keepalive` / `close` | credential+level / token / token | token+ttl+level / ttl / — (all overrides revert) |
| `obj.get` | id \| glob | value(s) + source (auto/override) + timestamp |
| `obj.set` / `obj.override` | id, value, [confirm nonce] | applied value \| guard challenge |
| `obj.pulse` | id, value, duration_ms | momentary actions (lamp test, identify, relay exercise) |
| `stream.sub` / `stream.unsub` | channel, [rate, filter] | ack |
| `diag.run` | test id, [params] | handle → progress/result on channel 9 |
| `cal.*` | §9.6 | before/after values |
| `cfg.export` / `cfg.import` | — / signed blob | config (validate-then-commit) |
| `log.fetch` | cursor, count | historical records |
| `fw.inventory` | — | **every IC with firmware/ID (§9.1)** — *added* |
| `fw.begin` / `fw.data` / `fw.end` / `fw.confirm` / `fw.revert` | target + image | multi-IC update (§9) — *added* |
| `mirror.get` | — | full panel-mirror frame (§8) — *added* |
| `sys.reboot` / `sys.bootloader` | token, confirm | — |
| `time.get` | — | device UTC/TAI, leap state, clock status |

Values are SI-unit numbers or enumerated strings. `debug.reg` (raw register access)
exists at G3 and is compiled out of release builds.

---

## 4. Capability manifest

Served by the device, versioned by content hash, cached by FMT keyed on (fw version,
hash). Abridged shape:

```json
{
  "schema": 1, "fw": "0.2.0", "hw": "sts1000-lqfp144",
  "objects": [
    { "id": "led.rgb.status", "kind": "rgb_pwm", "guard": "G0",
      "caps": {"channels":["r","g","b"], "modes":["auto","override"]},
      "desc": "Front-panel status RGB (D5, TIM4 CH1-3, common-anode low-side NPN)" },
    { "id": "rail.vcc_rb", "kind": "rail", "guard": "G2",
      "caps": {"transfer":"24.45 - 6.645*Vctrl", "set_range_mv":[4510,15000],
               "step_code":1, "monitor":"ina228.vcc_rb"},
      "interlocks": ["rb_vmax","rb_ov_clear","ocxo_warm","supercap_ok","measured_ceiling"] },
    { "id": "gpio.rb_vcc_gate", "kind": "gpio_out", "guard": "G2", "default": "off" }
  ],
  "sensors": [ {"id":"ina228.panel_led","addr":"0x4C","rail":"panel_led_5v",
                "channels":["v","i","p","dietemp"],"shunt_mohm":150,
                "current_lsb_na":520.833,"fs_ma":273.1} ],
  "streams": [...], "diags": [...], "buttons": [...], "components": [...]
}
```

Rules:
- Every object the firmware can touch **must** appear. FMT renders unknown `kind`s as
  generic get/set widgets, so new firmware features are usable without a tool release.
- `guard`, `interlocks` and limits are **enforced device-side**; the manifest copy lets
  the UI annotate before the user tries.
- Strings carry designators (D5, K2, U44…) so the GUI cross-links to the schematic.
- `set_range_mv` for `rail.vcc_rb` is **derived at runtime** from the configured
  `pwr.rb.vmax_mv` — it is not a fixed 10–18 V window.

---

## 5. Safety model

### 5.1 Principles

1. Observation is free; mutation is gated.
2. Firmware remains the authority. An override is a lease, not a transfer: fault
   supervision (INA alerts, OV latch, thermal ladder, watchdog) keeps running and **may
   veto or revert any override**; the veto is reported on channel 9.
3. Nothing persists. Every override reverts on session end, dead-man, or reboot.

### 5.2 Guard classes

| Class | Meaning | UI behavior | Examples |
|---|---|---|---|
| G0 | Cosmetic / safe | direct | RGB colour, panel-LED brightness, backlight duty, lamp test, identify pulse, fan duty (floor enforced) |
| G1 | Service-affecting, hw-safe | one-click confirm + "service degraded" banner | DISP_EN cycle, GPS_PWR_EN cycle, ANT_BIAS_EN, REF_TERM_EN, MUX_SEL, button/encoder capture mode, K1 serial-mode relay, PHY reset |
| G2 | Hardware-risk, interlocked | typed device-serial confirmation + interlock check | RB_PWR_EN, RB_VCC_GATE, **VCC_RB setpoint**, K2 holdover-relay force, WDT_EN, NOR_RST_N |
| G3 | Destructive / last resort | typed phrase + hold | POE_KILL, factory reset, `sys.bootloader`, `debug.reg`, GNSS/Rb firmware flash |

### 5.3 Sessions & auth

`session.open` against the same credential store the web and console planes use
(role ≥ operator for G0–G1, admin for G2–G3). Read-only monitoring needs no session.
Token TTL 15 min, extended by keepalive; one session at a time, takeover audited.
Repeated failures escalate through a throttle to a hard lockout (shared discipline with
MCP/web — reconnecting does not reset it). Every mutating call is audit-logged
(user, object, old→new, source).

### 5.4 Dead-man

Overrides hold only while **all** of: session valid, keepalive fresh (≤5 s), DTR
asserted, USB not suspended. Any failure reverts every override to automatic within 2 s
and logs it. FMT sends keepalive at 1 Hz.

### 5.5 Hard interlocks (device-enforced, non-overridable)

| Interlock | Rule |
|---|---|
| **VCC_RB setpoint** | Three independent defenses: (a) an out-of-range wiper code resolves to the **safe-low** end, never maximum; (b) a commanded setpoint whose *expected* rail exceeds `pwr.rb.vmax_mv` is refused; (c) the **measured** rail on INA228 0x47 must be ≤ `pwr.rb.vmax_mv` — a check that consults no digipot code, so a runaway rail cannot validate itself. Refused outright if RB_OV_DET is latched. Post-set read-back verify with auto-revert. |
| RB_PWR_EN on | Refused unless OCXO warm and supercap PG good and PoE headroom covers the Rb cold start. |
| RB_VCC_GATE | Only after the safe-precharge rail *and* the operating-setpoint rail have both verified in-window. Teardown is always gate-before-supply. |
| POE_KILL | UI must display the latch consequence (recovery may require a PSE port cycle). No pulse shorter than the documented off-time. |
| Fan override | Duty floor from the thermal loop; the over-temp ladder always wins; a stalled loop forces 100 %. |
| K2 holdover relay | Force-de-energize (assert alarm downstream) allowed; force-energized-while-fault refused. |
| DISP_EN | Soft-start fault-mask window enforced; minimum off-time before re-enable. |
| Watchdog | Arming requires the supervisor's liveness gate healthy. `diag.wdt_test` uses the sanctioned window-violation test, never a raw disable. |
| GNSS/Rb passthrough | Suspends firmware's own use of that UART, flags the reference suspect, and auto-restores on tunnel close. |

---

## 6. Control surface

Guard class in brackets. All objects are also readable.

### 6.1 Indicators & panel

| Object | Control |
|---|---|
| Status RGB **D5** [G0] | auto/override; colour + brightness; per-channel duty; blink/pulse patterns; identify (blue pulse). *Correction: designator is D5, not D36.* |
| Panel illumination [G0] | PANEL_LED_EN on/off (RT9742) + duty on **PE0/LPTIM2_CH2**; ramp test; live current from INA228 **0x4C**, duty-normalized |
| Display backlight [G0] | duty 0–100 % (PE6, TIM15_CH2 — shares TIM15's 25 kHz period with the fan), blank/wake |
| Display panel [G1] | DISP_EN cycle, DISP_RST pulse, test patterns (bars/gradient/pixel-walk/white/black), draw text |
| Lamp test [G0] | `obj.pulse` — all lamps + backlight full for N s |
| Buttons/encoder/touch [G1] | live capture: every press/release/detent/touch coordinate streamed; test mode suppresses normal UI navigation |
| Reed switch (PROX_WAKE) | read-only, live |

### 6.2 Power & rails

| Object | Control |
|---|---|
| RB_PWR_EN [G2] | on/off with staged sequencing + interlock state display |
| RB_VCC_GATE (Q25) [G2] | open/close; default off |
| VCC_RB setpoint [G2] | value within the runtime-derived bound; wiper read-back; INA228 0x47 verify plot |
| Rb OV latch [G2] | view RB_OV_DET (PE3); pulse RB_OV_RESET (PD3) |
| GPS_PWR_EN [G1] | receiver power-cycle (warm start preserved by V_BCKP) |
| ANT_BIAS_EN [G1] | antenna bias with INA228 0x45 live current + open/short classifier |
| DISP_EN [G1] | display 5 V rail |
| WDT_EN [G2] | external TPS3430 arm/disarm (boot-disabled default) |
| POE_KILL [G3] | commanded cold cycle, with latch warning |
| NOR_RST_N [G2] | SPI-NOR reset |
| Supercap backup | **read-only digital power-good only** (BKP_STM_PG PF14, BKP_GPS_PG PF15). *Correction: there is no supercap SoC ADC — the TPS61094 managers are autonomous and firmware sees PG bits.* |
| Rail power-good + EN faults | read-only, from the **1 kHz direct-GPIO scan of GPIOF/GPIOG**. *Correction: there is no I/O expander and therefore no `EXP_RESET` object — the MCP23017s were deleted from the design.* |

### 6.3 Timing & references

| Object | Control |
|---|---|
| Discipline loop [G1] | auto/hold; bounded manual Vc; loop time-constant select; re-lock; park/unpark |
| Reference select (MUX_SEL) [G1] | OCXO / external with the firmware HSI-bridge sequence (never a bare pin write); shows EXTREF_MON measured frequency + validity |
| REF_TERM_EN [G1] | 50 Ω terminate / high-Z (default terminated) |
| 1PPS output | ETH_PPS_OUT (PB5) from the MAC PTP unit, buffered by U71 to J15; PTP-clock configurable, no separate buffer enable |
| Holdover relay K2 [G2] | view; force-alarm test (fail-safe semantics displayed) |
| Rb (FE-5680A) [G2] | power (§6.2); K1 RS-232/CMOS relay (PE4, default RS-232); RB_LOCK polarity config bit; frequency-offset read + guarded EFC trim (§9.4) |

### 6.4 GNSS

| Object | Control |
|---|---|
| Constellations/config [G1] | GPS/GLONASS/Galileo/BeiDou enable, rates, message set (UBX-CFG-VALSET through the firmware API) |
| Survey-in [G1] | start/stop with target accuracy + duration; fixed-position set/clear; progress plotted. Results are **gated on the configured duration and accuracy** before they are stored |
| Receiver reset [G1] | hot/warm/cold; GPS_RST_N hard reset |
| Antenna supervisor [G1] | bias enable, fault acknowledge; fused from UBX-MON-RF + GPS_ANT_OFF_MON + INA228 0x45 |
| GNSS firmware [G3] | §9.3 |

### 6.5 Environment, network & system

| Object | Control |
|---|---|
| Fan [G0] | auto/override duty (floor-clamped), tach RPM live, stall alarm |
| E-compass [G1] | in-enclosure hard/soft-iron calibration; heading + raw XYZ live |
| Ethernet | read-only PHY link/speed/duplex + counters; [G1] PHY reset |
| Services [G1] | NTP / NTS / PTP / SNMP / web enable/disable/restart + client counters |
| Config [G2] | export / import (validate-then-commit) / factory reset [G3] |
| System | reboot [G1], bootloader [G3], set RTC [G1] |

---

## 7. Monitor surface

Read-only, session-free, subscription-driven.

### 7.1 Telemetry stream (channel 1, 1–10 Hz)

| Group | Contents |
|---|---|
| Power | **INA228 ×9 (V/I/P + die temp): PoE 0x40, 3V3_STM 0x41, 5V_DISP 0x42, 3V3 main 0x43, V_ANT 0x45, OCXO 0x46, VCC_RB 0x47, GPS 0x4A, panel-LED 0x4C.** *Correction: the draft listed "GPS VCC 0x43" and "3V3 bulk 0x44" — 0x43 is the main 3V3 rail and **0x44 is the SHT45 humidity sensor**, which is precisely why the GPS monitor is strapped to **0x4A**. The draft's open item about an OCXO/Rb address swap is **closed**: OCXO is 0x46, VCC_RB is 0x47.* |
| PoE | negotiated class, NCP1095 status (NCM/NCL/LCF), PFI comparator state |
| Thermal | TMP117 ×2 (oscillator 0x49, enclosure 0x48), SHT45 humidity 0x44, STM32 die temp, fan duty + RPM, INA die temps |
| Rails | all eight PG bits + EN-fault flags + INA alert lines from the 1 kHz GPIOF/GPIOG scan; backup PG ×2; latched fault causes with DIAG_ALRT decode |
| Timing | loop state, active reference, Vc commanded + sensed (each with a validity flag), phase/frequency error, holdover elapsed + estimated error + time-to-demotion, ADEV(1/10/100 s), stratum, leap state |
| GNSS | fix type, SVs used/visible, accuracy estimate, survey-in progress, antenna state, leap info |
| Rb | powered, lock (opto + serial cross-check), warm-up timer, measured VCC_RB |
| UI | reed switch, button states, display on/off, heading (tilt-compensated), accel XYZ |
| System | uptime, per-thread stack/CPU, heap, filesystem usage, USB/Ethernet stats, liveness participants |

### 7.2 PPS/discipline stream (channel 5, 1 Hz)

Per-second record: both capture channels (PA0/TIM2 and PC6/TIM3), raw and
sawtooth-corrected residual (UBX-TIM-TP `qErr` applied, paired by **GPS time-of-week**),
cable/board delay applied, MAD-gate verdict, Vc, loop state, reference flags. Feeds the
host's strip charts and an on-host ADEV/TDEV computation.

### 7.3 NMEA view (channel 2) and 7.4 UBX view (channel 3)

Raw tees, capturable to `.nmea` / `.ubx` for offline analysis. The UBX decoder drives the
satellite panel: skyplot (az/el, constellation-coloured, used vs visible), per-SV C/N₀
bars, jamming/AGC indicators from MON-RF. The device renders the same skyplot locally
(§8), so both views agree.

### 7.5 Log & event views (channels 4, 9)

Live structured tail with subsystem/level filters, cursor-based history, export; the
event channel carries edge-triggered faults, button presses, reed-switch changes,
override vetoes, and diagnostic progress.

---

## 8. Panel mirror — live replica of the device UI

*New section: the host must be able to see exactly what the operator at the rack sees.*

Channel 0x0A (or `mirror.get` for a one-shot frame) carries a compact, versioned,
delta-encoded record at up to 10 Hz:

| Group | Contents |
|---|---|
| **Screen** | The active page id, the text-tile surface the device renders (rows × cols with per-cell attributes), any open dialog/confirm state, and the scroll/selection cursor. Deltas only — unchanged tiles are omitted, so a 10 Hz mirror costs a few hundred bytes/s. |
| **Lamp state** | Status RGB commanded r/g/b + pattern (locked/warming/holdover/fault/identify) and its *source* (auto vs override); panel-LED rail enable + PWM duty + duty-normalized measured current from INA228 0x4C; display backlight duty; the derived per-lamp logical states so the host can draw lit/unlit buttons faithfully. |
| **Inputs** | Which of the seven buttons and the encoder switch are physically down, the encoder's accumulated position, the last touch coordinate, and the reed-switch state — so the mirror shows the operator's own key presses live. |

Host rendering: FMT draws a to-scale replica of the front panel — bezel, seven buttons
with their lit state, the encoder, and the TFT contents — beside the live telemetry. Two
uses: remote support (the field engineer sees the panel without describing it) and
verification (the mirror is the fastest way to confirm a UI change or a lamp fault).
Because the lamp state carries its `source`, the mirror also makes an active override
visually obvious.

---

## 9. Firmware update — every IC on the board

*Expanded: the original draft covered only the STM32 application and a GNSS passthrough.
The requirement is a single uniform interface for every component that has firmware, and
a complete inventory of the ones that do not.*

### 9.1 Component inventory (`fw.inventory`)

The device enumerates every IC with firmware or a readable identity, so the tool can show
the whole board and never leaves the operator guessing:

| Component | Designator | Updatable | Version/ID source |
|---|---|---|---|
| STM32H563 application | U12 | **yes** — signed image, A/B slots | MCUboot image header (running + staged) |
| MCUboot bootloader | U12 | via a signed application update of the boot partition (factory/guarded) | image header |
| ZED-F9T GNSS receiver | U21 | **yes** — u-blox loader over USART3 | UBX-MON-VER |
| FE-5680A rubidium | J6 module | **variant-dependent** — probed and reported | serial identification |
| ATECC608B secure element | U60 | no (immutable) | Info command, serial number |
| LAN8742AI PHY | U6 | no | MDIO PHY ID registers |
| INA228 ×9 | U10/U23/U26/U30/U31/U32/U37/U44/U54 | no | MANUFACTURER_ID / DEVICE_ID |
| TMP117 ×2 | U57/U58 | no | device ID register |
| SHT45 | U72 | no | serial number command |
| IIS2MDC / LIS2DH12 | U61 / U59 | no | WHO_AM_I |
| ST7796S display / FT6336U touch | panel | no | controller ID registers |
| MCP41U83 digipot | U43 | no | — (SPI, no ID) |
| NCP1095 PD controller | U9 | no | — (status pins only) |

### 9.2 STM32 application

Two paths, both enforcing MCUboot signature verification and anti-rollback — the tool can
never bypass them:

1. **Application running:** `fw.begin/data/end` (or MCP `FW_*`, or SMP over channel 6)
   streams the signed image into slot 1. Chunks are idempotent and resumable; non-final
   chunks are write-block aligned (16 B on this part); the image is SHA-256 verified **by
   reading it back out of flash** before the slot is marked pending; the trailer region is
   reserved so `staging_size` never collides with the swap metadata. Then reboot → test
   boot → firmware self-confirms after its health gate, else MCUboot reverts.
2. **Application unbootable:** MCUboot serial recovery — hold **BUTTON_1 (PF0)** through
   reset for ≥1 s; the device enumerates as recovery (PID 0x1001); `mcumgr image upload`.
3. **Factory unbrick:** BOOT0/DFU via the ROM bootloader, gated by debug-authentication
   state.

### 9.3 ZED-F9T GNSS receiver [G3]

Channel 7 tunnels USART3. The firmware drives the guarded sequence: flag timing
degraded → suspend its own UBX use → assert `GPS_SAFEBOOT_N` (PD15) with a
`GPS_RST_N` (PD11) pulse to enter the loader → negotiate → stream the vendor image in
acknowledged chunks with retry → reset back to normal mode → verify the reported version
via UBX-MON-VER → **re-apply the full timing configuration** (constellations, rates,
timepulse, TX-ready remap, TMODE/fixed position) → clear degraded. Every unknown or
failed step aborts and restores; the receiver is never left in safeboot.

### 9.4 FE-5680A rubidium [G2/G3]

Channel 8 tunnels UART7 with the K1 RS-232/CMOS relay (PE4) selectable live, for
commissioning surplus units of unknown variant. The device probes and classifies the
unit as *telemetry-only*, *EFC-trimmable*, or *firmware-updatable* and reports that in the
inventory, so the tool offers only what the hardware supports. EFC fine-trim is an
explicit, bounded, read-back-verified operator action with an audit entry — never
automatic (the Rb is a self-disciplined reference; firmware does not steer it).

### 9.5 Diagnostics & tests (`diag.run`, results on channel 9)

`i2c.scan` (**one bus** — I²C1; the touch controller sits behind the PCA9306 translator
on the DISP_EN-gated rail, so it appears only when the display rail is up. *Correction:
the draft's "both segments" and the expander step in `i2c.recover` do not apply — there is
a single bus and no expander; recovery is controller clock-out plus device-level reset*),
`ina.selftest`, `display.pattern`, `touch.target`, `ui.exercise`, `relay.exercise`,
`fan.sweep`, `wdt.test` [G3], `eth.loopback`, `nor.verify`, `sec.attest` (ATECC608B
attestation + monotonic counters), `gnss.antenna`, `pps.selfcheck`, `compass.cal`, and
`diag.snapshot` — the one-shot support bundle (full telemetry, config, versions, fault
latches, I²C scan, PPS residual histogram, log tail, NAV-SAT dump). Burn-in mode loops
the non-destructive tests with 10 Hz telemetry logging and reports per-channel min/max/σ.

### 9.6 Calibration

Antenna cable delay, PPS/board routing offset, OCXO characterization sweep (Vc vs
frequency + tempco fit), holdover characterization run, Rb EFC trim, e-compass
hard/soft-iron, per-channel sensor zeros, and the **nine per-board INA228 `SHUNT_CAL`
trims** — the last of these dominates the uncalibrated current-measurement error budget
and must be re-applied after any INA228 reset. Each calibration is a guided flow with
before/after values, persist-on-confirm, and an audit entry.

---

## 10. Host application

### 10.1 Platforms & distribution

Windows 10+ (x64/ARM64), macOS 12+ (universal), Linux glibc 2.31+ (x64/ARM64). Signed
single-file installer/DMG/AppImage plus a portable archive; no admin rights needed and no
kernel modules anywhere (Linux serial access via the `dialout` group or a shipped udev
rule). `sts1000ctl` is the same engine headless, with JSON output for CI and bench
automation. Updates are manual; FMT checks compatibility against the device manifest and
never auto-updates.

### 10.2 Architecture

```
core (Rust lib): transport ─ framing ─ MP client ─ manifest model ─ SMP ─ capture/replay
       │                                             │
  sts1000ctl (CLI)                          FMT GUI (Tauri, TS front-end)
```

The core library owns all protocol logic; GUI and CLI are thin. Every session can be
recorded across all channels and replayed offline, so a support bundle can be analysed
without hardware. A simulated-device backend implements MP against a behavioural model
for tool development.

### 10.3 GUI structure

Dashboard (annunciator strip, clock state, GNSS summary, nine rail tiles, thermal,
holdover) · **Panel Mirror** (§8) · Peripherals (manifest-generated control tree with
auto/override source and guard badges) · GNSS (skyplot, C/N₀, NMEA, UBX decoder,
survey-in) · Timing (phase/Vc strip charts, ADEV, reference selector) · Power (per-rail
plots, PG/fault matrix, PoE) · Logs · Diagnostics (test catalog, burn-in, support bundle)
· **Firmware** (component inventory + per-target update) · Console (shell tab + raw MP
inspector).

UI rules: light/dark, resizable panes, every plotted value exportable (CSV/PNG), full
keyboard reachability, colour-blind-safe status palettes, timestamps in both device and
host time.

### 10.4 Performance & robustness targets

| Metric | Target |
|---|---|
| Connect to dashboard | ≤ 3 s cached manifest, ≤ 6 s cold |
| Telemetry latency (device tick → pixel) | ≤ 250 ms |
| Sustained capture | ≥ 1 h at 10 Hz telemetry + full NMEA/UBX + mirror without drop |
| Application image upload | ≤ 90 s for ~400 KB over FS CDC |
| Surprise unplug | no crash; overrides reverted device-side; clean reconnect |
| Host resources | < 200 MB RAM, < 5 % CPU while monitoring |

### 10.5 Host security posture

Credentials never stored plaintext (OS keychain optional, off by default); capture
archives never embed credentials and support bundles scrub secrets; FMT verifies device
identity via ATECC608B attestation and warns on a serial/identity change between
sessions; firmware images are trusted only by the device — FMT's local header check is a
fail-fast courtesy, not a trust anchor.

---

## 11. Device-side firmware deltas

This table is the authoritative build-state tracker for the MP layer. "In tree" means the
code exists, is unit-tested on the host, and links into the signed image.

| Item | State |
|---|---|
| Frame mux (COBS + CRC16, channel dispatch, `mp enter/exit`) | in tree |
| Manifest generator (build-time table → runtime JSON + content hash) | in tree |
| Override engine (lease table, dead-man, revert hooks, veto reporting) | in tree |
| Sessions + guard/interlock evaluation | in tree, sharing the credential store and lockout discipline with the console and web planes |
| Streams (telemetry/PPS/log/event/mirror CBOR; NMEA/UBX tees) | in tree |
| Tunnels (USART3, UART7) with firmware-suspend handshake | in tree |
| Diag runner + support bundle | in tree |
| Multi-IC update orchestrator + inventory | in tree |
| Host application (§10) | **not started** — specified only |
| Budget rule | MP threads run at console priority and **never hold a timing mutex** — they read the lock-free quality/health snapshots only, so the discipline loop is unaffected |

Items marked "in tree" are verified against the repository at the commit that introduced
this document's §11; anything the firmware has not yet landed is called out explicitly
rather than implied.

---

## 12. Compatibility & versioning

MP version is reported in `hello`; the frame layer is stable across MP minor versions.
The manifest carries an integer `schema`; FMT supports the current and previous schema.
FMT works read-only against unknown-newer firmware via generic widgets; mutation requires
a schema match so guard fidelity is guaranteed. A compatibility matrix is maintained
alongside the firmware, and the `sts1000ctl` smoke suite runs against the simulated device
on all three host targets per release.

---

## 13. As-built firmware capabilities the FMT surfaces

*Added: features present in the firmware that the original draft did not mention. The FMT
must expose these, and they are the reason several sections above differ from the draft.*

### 13.1 Timing engine

Sawtooth-corrected PPS (UBX-TIM-TP `qErr`, paired by GPS time-of-week), dual-channel
capture cross-check with a median/MAD outlier gate, FLL+PI discipline with a configurable
10–1000 s time constant, DAC actuator scaled to the OCXO's ±0.4 ppm pull with slew
limiting and anti-windup, learned temperature feed-forward, overlapping-Allan-deviation
estimates at τ = 1/10/100 s, a holdover estimator whose error and served root dispersion
are **monotonic by construction** (they cannot improve while unlocked), rate-limited
re-convergence with no time step, and a reference state machine whose OCXO⇄Rb handoff
brackets the clock switch with a discipline park/unpark so a healthy reference upgrade
never demotes the served stratum.

### 13.2 Console protocols

**MCP** — the as-built binary console protocol on ACM1: COBS + CRC32 framing, session
auth with escalating lockout, the full typed configuration registry (staged
validate-then-commit, signed export/import, write-only secret keys), status groups,
subscribable telemetry, cursor-based log tail, diagnostics, and the resumable DFU engine.
`firmware/tools/meridian_ctl.py` is the reference client (`info`, `status`, `watch`,
`cfg-*`, `log-tail`, `fw-*`, `reboot`). MP (§3) supersedes it for the FMT and reuses the
same underlying engines; MCP remains supported.

### 13.3 Shell

`sts status|quality|alarms|log tail|cfg|fw|reboot|diag` on ACM0, plus the credential
provisioning path used after a factory reset.

### 13.4 Network services the FMT can observe and control

NTP server (rate limiting, Kiss-o'-Death, optional interleaved mode, symmetric-key MAC
with a per-key algorithm selector), NTS (AES-SIV-CMAC-256 cookies, master-key ring with
rotation, NTS-KE on 4460), PTP grandmaster (Default plus Telecom G.8275.1/.2 and Power
C37.238 profiles, Annex-P integrity with a policy switch), SNMP (v2c and v3 USM with
HMAC-SHA-256 auth and AES-128 privacy, traps/informs, shipped Zabbix template), the
HTTPS/REST/WSS web plane, syslog, mDNS, DHCP/static dual-stack addressing, and enterprise
AAA (RADIUS, TACACS+, LDAP) with role mapping.

### 13.5 Platform behaviours worth surfacing

The staged bring-up sequence and its per-stage interlocks; the 1 kHz direct-GPIO fault
scan with per-class debounce and button long-press/repeat semantics; the watchdog
supervisor's liveness participants (a kick is withheld the moment any registered
participant goes stale); the power-fail (PFI) fast-save that persists the critical set and
parks the DAC; the thermal ladder with its secondary-sensor fallback and fail-safe
full-speed fan; the expected-off rail mask (a deliberately deferred Rb does not paint the
UI red); LittleFS `/lfs` for the log spool and bulk config with graceful degradation when
the NOR is absent; and the full 640 KB SRAM enablement.

---

## 14. Open items

- [ ] VID/PID acquisition (pid.codes application or purchased VID) — blocks release descriptors
- [ ] Measure sustained CDC throughput on H563 FS with DMA; sizes the NMEA+UBX+10 Hz+mirror headroom claim
- [x] ~~INA228 address↔rail discrepancy~~ — **closed**: the map in §7.1 is authoritative (GPS is 0x4A because SHT45 owns 0x44; OCXO 0x46, VCC_RB 0x47)
- [x] ~~I/O-expander objects (`EXP_RESET`, "U47 PortA" PG bits)~~ — **closed**: no expander exists; PG/fault inputs are a direct 1 kHz GPIOF/GPIOG scan
- [ ] Decide whether `debug.reg` ships in release firmware or bring-up builds only (current: compiled out of release)
- [ ] K2 contact-verify method — coil current only, or add a downstream sense net (schematic question)
- [ ] Enumerate the FE-5680A opcode set to wrap beyond the 0x2D frequency-offset family, per surplus variant
- [ ] Capture archive container for the host tool (length-prefixed records vs SQLite)
- [ ] Windows ARM64 CDC behaviour on surprise removal — verify on hardware
- [ ] Confirm whether MCUboot serial recovery should also accept the channel-6 SMP tunnel, or remain raw-SMP-only (current: raw-only, FMT auto-detects)
- [ ] Host application implementation (this document specifies it; only the device side and the reference CLI exist today)
