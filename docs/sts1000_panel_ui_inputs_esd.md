# STS1000 — Panel / UI Inputs & ESD (Buttons, Tamper, Touch INT)

**Scope:** every off-board human-interface input that reaches a logic pin, plus its connector
ESD: the **7 keypad buttons** and **touch INT** (→ STM32 **GPIOF PF0–PF7**, direct GPIO), and the
**enclosure tamper switch** (→ STM32 **PC13 / TAMP**, backup domain). TVS arrays cover the cable
entries.

**Aggregation:** all these inputs land on STM32 U12 GPIO directly and are gathered by a ~1 kHz
`GPIOF` port scan (see `sts1000_fault_aggregation.md`). **U54 = panel INA228 (0x4C)** and
**U55 = panel-LED RT9742**. The touch-INT pull-up is **R208**.

Cross-refs: `ntp_server_peripheral_map.md` §8/§9/§11; `sts1000_fault_aggregation.md` §4 (direct-GPIO
scan); `sts1000_firmware_hardware_interface.md`; `sts1000_3v3p_i2c_peripherals.md`;
`ntp_server_software_spec.md` §2.7/§2.8/§6/§9.6.

---

## 1. Topology summary

| Input | Count | Entry connector | TVS array | Conditioning | Logic pin |
|---|---|---|---|---|---|
| Buttons | 7 | **J17** (36-pin, consolidated) | U19 (4 ch) + U20 (3 ch) — TPD4E02B04DQAR | 10 kΩ→3V3_STM + 100 nF | PF0–PF6 |
| Tamper | 1 | J10 (1×02) | U16 ch — TPD4E02B04DQAR | C201 1 nF, **no external PU** | PC13 / TAMP |
| Touch INT | 1 | **J17.33** (display bus) | **U17 spare ch — TPD4E05U06DQAR** | R208 10 kΩ→3V3_STM, **no cap** | PF7 |

All three are **active-low, idle-high**. Buttons/tamper short to GND on actuation; touch INT is a
driven pulse from the FT6336U.

---

## 2. Buttons — PF0–PF6 (direct GPIO)

```
[Panel SPST]──J11──TVS(U19/U20)──●node●──PFn (U12)
                                 │      │
                         10k→3V3_STM  100nF→GND
```

| Net | TVS pin | Pull-up | Debounce C | U12 pin |
|---|---|---|---|---|
| BUTTON_1 | U19 D1+ (p1) | R190 10 kΩ | C161 0.1 µF | PF0 / 10 |
| BUTTON_2 | U19 D1− (p2) | R191 | C162 | PF1 / 11 |
| BUTTON_3 | U19 D2+ (p4) | R192 | C163 | PF2 / 12 |
| BUTTON_4 | U19 D2− (p5) | R193 | C164 | PF3 / 13 |
| BUTTON_5 | U20 D1+ (p1) | R194 | C165 | PF4 / 14 |
| BUTTON_6 | U20 D1− (p2) | R195 | C166 | PF5 / 15 |
| BUTTON_7 | U20 D2+ (p4) | R196 | C167 | PF6 / 18 |

- **Idle (open):** 10 kΩ holds the line at 3V3_STM → high.
- **Pressed:** SPST ties the line to GND → low.
- **Debounce:** τ = R·C = 10 kΩ × 100 nF = **1 ms** (release edge; the dominant filter). Press-edge
  cap dump is limited by cable series R, so no dedicated series resistor is fitted.
- **TVS flow-through:** each net lands on its protected D-pin (1/2/4/5) **and** the diagonal NC pin
  (6/7/9/10) as a routing feed-through. The NC pins are internally isolated — harmless, intended use.
- All pull-ups on **3V3_STM** (always-on), so buttons read valid whenever the MCU runs.

---

## 3. Tamper — PC13 / TAMP (backup domain)

```
[Lid SPST, NC]──J10──TVS(U16 D2−)──●RTC_TAMPER●──PC13
                                   │
                              C201 1nF→GND   (NO external pull-up)
```

| Element | Value | Purpose |
|---|---|---|
| TVS | U16 D2− (p5), TPD4E02B04DQAR | Enclosure-cable ESD |
| Cap | C201 **1 nF** | Holds the line between TAMP precharge samples; debounce |
| Pull-up | **none external** | Provided by the TAMP internal precharge pull-up (backup-domain powered) |
| Connector | J10 pin 1 = RTC_TAMPER, pin 2 = GND | Lid/intrusion contact across 1–2 |

**Why no external pull-up:** tamper is in the **backup (VBAT/supercap) domain** so intrusion is
detected with main power off. A continuous external 10 kΩ on a backup rail would bleed ~300 µA and
flatten the supercap in hours. The STM32 TAMP block provides an **internal pull-up + periodic
precharge** instead: the pull-up is connected only for a short precharge window before each sample,
giving nA–µA average draw while keeping a defined high level.

**Why 1 nF (not 100 nF):** the external cap must charge to a valid high **inside** the precharge
window. Precharge is 1/2/4/8 RTCCLK cycles; at LSE 32.768 kHz, 8 cycles ≈ 244 µs. With internal
R_PU ≈ 40 kΩ (confirm from the STM32H563 datasheet), τ(1 nF) ≈ 40 µs → charges in ~4 cycles. 100 nF
would need ~10 ms → permanent false-tamper. 1 nF is the right value; keep ≤ ~2 nF.

**Switch polarity:** use a **normally-closed** contact held closed by the secured lid. Closed =
line at GND (secure); lid-open **or a cut cable** = pull-up takes it high = tamper. Fails secure
against tampering with the switch wiring. Set the TAMP active level/edge to match.

**Not an EXTI source** — PC13 is serviced by the RTC/TAMP peripheral, leaving EXTI13 free.

---

## 4. Touch INT — PF7 (direct GPIO)

```
Touch INT──J17.33──TVS(U17 spare ch)──●DISP_TOUCH_INT●──PF7 (U12)
 (3.3V, push-pull, active-low)        │
                               R208 10k→3V3_STM   (NO cap)
```

| Element | Value | Purpose |
|---|---|---|
| TVS | **U17 spare channel, TPD4E05U06DQAR** (5.5 V V_RWM) | Display-flex ESD; touch cable is high-touch |
| Pull-up | R208 10 kΩ → **3V3_STM** | Defines "no touch" = high, and holds a defined level when the display 5 V rail is gated off |
| Cap | **none** | A debounce cap would swallow the INT pulse |
| Pin | U12 PF7 / 19; touch INT arrives on J17.33 (display bus) | Read on the GPIOF scan (or EXTI) |

- **3.3 V line.** Although touch I²C (`I2C_DISP_SCL/SDA`, J17.32/J17.15) idles at 5 V behind the
  PCA9306 (U63), the INT (J17.33) is a **3.3 V** signal direct from the touch controller. It fits the
  U17 5.5 V array trivially; the array is 5.5 V-rated because of whatever else it protects, not
  because the INT needs it.
- **Module supply:** the touch/display module is powered from **J17.16 = 5V_DISP** (display) and
  **J17.28 = 5 V** (panel logic); this INT path is the signal return from that powered module.
- **Pull-up rail rationale:** on 3V3_STM (always-on) so the line is defined high when the display
  rail (5V_DISP, gated by DISP_EN) is off — a gated-rail pull-up would let PF7 float into a phantom
  wake. When the display rail is gated off, ~260 µA flows through R208 into the touch controller's
  INT pin; harmless, current-limited, into a part held in reset.
- **No debounce cap** — same rule as any INT line.

---

## 5. TVS arrays — verified specs

| Array | Part | V_RWM | V_BR(min) | V_clamp | Cap/ch | Lines |
|---|---|---|---|---|---|---|
| U19, U20 | TPD4E02B04DQAR | ±3.6 V | 5.5 V | ~8.8 V @ Ipp | 0.25 pF | 7 buttons + tamper (3.3 V logic) |
| U17 | TPD4E05U06DQAR | 5.5 V | 6.5 V | ~14 V @ Ipp | 0.5 pF | touch INT on spare ch (+ its own lines) |

**Common DQA-10 pinout (both parts):** D1+ = 1, D1− = 2, GND = 3, D2+ = 4, D2− = 5, NC = 6/7/9/10,
GND = 8. Protected I/O only on pins 1/2/4/5; NC pins are straight-through routing pads. Place every
array at its connector; tie both GND pins (3, 8) to the plane with short, low-impedance vias.

**Voltage fit:** all four protected nets idle ≤ 3.3 V; tamper idles ≤ ~3.3 V (precharge) or ≤
supercap (~3.0 V). All inside both arrays' standoff. The TPD4E02B04's 8.8 V clamp is the better
match for 3.3 V logic; the U17 5.5 V part's ~14 V clamp lets through more during a strike, but the
MCU PF7 input survives the residual on its own ESD cells — acceptable for a spare-channel reuse.

---

## 6. Firmware implications

### 6.1 Direct GPIO (PF0–PF7 = buttons + touch INT)

- **No expander.** Firmware reads `GPIOF->IDR` on a ~1 kHz scan and XORs against the last snapshot
  to find changed bits. Buttons/touch INT may **additionally** be armed as EXTI on their PF pins for
  low-latency wake.
- **Buttons (PF0–PF6):** active-low, a press reads **0**. On change → debounce (~20–30 ms, software
  timer or re-sample) → emit press/long-press/repeat to the nav state machine.
- **Touch INT (PF7):** active-low, push-pull; **no debounce** — on assert, service the touch
  controller over the display I²C (the gated 5 V bus behind PCA9306 U63).
- **Wake:** touch (PF7) is the **primary** UI wake; any button also wakes. Wake from blank is a
  `DISP_BL` PWM duty change only — DISP_EN and the 5 V rail stay asserted through dim/blank, so the
  touch controller stays powered and its INT stays live.
- **Touch bring-up sequencing:** on DISP_EN enable, allow rail soft-start, release DISP_RST past the
  touch-controller reset margin, then I²C-init touch. Mask `V_DISP_EN_FAULT_N` (PF9, U33 RT9742
  nFLG) for the soft-start window so inrush current-limit isn't logged as a fault.

### 6.2 STM32 TAMP (PC13)

- Configure **TAMPxE** (enable), **TAMPFLT ≠ 00** (2/4/8-sample level filter — **required for
  precharge to apply**), **TAMPPRCH** (precharge cycles; start at 4 and verify against the 1 nF +
  R_PU charge time), **TAMPPUDIS = 0** (internal precharge pull-up enabled), **TAMPFREQ** (lower =
  lower current, slower detect; ~1 Hz gives ≤1 s latency, fine for a lid switch).
- Set active level/edge to match the **normally-closed** contact (secure = low).
- Runs in the **backup domain** — armed with main power off, powered by the VBAT/supercap rail.
- **On tamper:** RTC-timestamp the event → tamper-evident audit log → SNMP trap; **optional,
  policy-gated** secure-erase (zeroize NTS master / symmetric keys / credentials wrapped by the
  ATECC608B/PSA key). No EXTI handler — the event is latched in TAMP status.

### 6.3 Bus/recovery interactions

- The panel button/touch/tamper inputs are **direct MCU GPIO** — no I²C dependency, no expander to
  wedge or reset. (The display I²C behind PCA9306 U63 is a separate segment; the touch-controller
  service path lives there.) The former `I2C_BUF_EN`/`EXP_RST_N` recovery levers no longer exist.

---

## 7. Purpose recap (per element)

| Element | Purpose |
|---|---|
| 10 kΩ pull-up (R190–R196, R208) | Define idle-high; set active-low sense |
| 100 nF (C161–C167) | 1 ms RC button debounce |
| 1 nF (C201) | Precharge-compatible tamper hold/debounce |
| U19/U20 TPD4E02B04 | Connector ESD, 3.3 V-class (low clamp) for the 7 buttons |
| U16 TPD4E02B04 ch | Connector ESD for the tamper line (shared array) |
| U17 TPD4E05U06 spare ch | Connector ESD for the 3.3 V touch INT, reusing a 5.5 V array's free channel |

---

## 8. Open items / flags

1. **U17 ESD locality (priority).** ESD must clamp at the cable entry. DISP_TOUCH_INT enters on the
   consolidated **J17 (pin 33)** — confirm **U17 sits at J17**. If U17 is a remote array (e.g.,
   the USB connector), a J17 strike propagates across the board before clamping and the protection is
   ineffective; in that case keep the INT on the local display array (U17 has spare channels) or add
   a local single-channel TVS at J17. **Confirm U17's location and which of its channels
   DISP_TOUCH_INT uses.**
2. **U17 channel count.** If DISP_TOUCH_INT moved off U17, U17 now carries only the two 5 V touch
   I²C lines (2 of 4 used). Update `sts1000_3v3p_i2c_peripherals.md` §6.5 accordingly.
3. **Single pull-up on DISP_TOUCH_INT.** R208 (here) must *replace* R167 (display sheet), not add to
   it — two parallel 10 kΩ = 5 kΩ and ~doubled back-feed (~520 µA) into the unpowered touch INT pin.
4. **TAMP internal R_PU.** Confirm the STM32H563 tamper internal pull-up value (datasheet) and set
   TAMPPRCH so C201 1 nF reaches valid V_IH inside the precharge window.
5. **Tamper power-off scope.** Confirm the supercap/VBAT budget covers the chosen TAMPFREQ precharge
   draw for the required power-off detection duration.
6. **Confirm tamper TVS locality.** `RTC_TAMPER` shares U16 (with `PROX_WAKE` and a display SPI
   line); confirm U16 sits at the J10 tamper entry so the clamp is at the cable, not mid-board.
