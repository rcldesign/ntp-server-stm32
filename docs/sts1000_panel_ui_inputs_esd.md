# STS1000 — Panel / UI Inputs & ESD (Buttons, Tamper, Touch INT)

**Scope:** every off-board human-interface input that reaches a logic pin, plus its connector
ESD: the **7 keypad buttons** and **touch INT** (→ MCP23017 **U54 @0x21** Port A), and the
**enclosure tamper switch** (→ STM32 **PC13 / TAMP**, backup domain). Three TVS arrays cover the
three cable entries.

Designator note: the schematic labels the button/INA expander **U54**; older docs call it **U48**/**U51**.
Same part/role/address (MCP23017 @0x21). The touch-INT pull-up is **R208** here (was **R167** in the
peripheral map). Reconcile docs to the schematic.

Cross-refs: `ntp_server_peripheral_map.md` §8/§9/§11; `sts1000_fault_aggregation.md` §1–2;
`sts1000_3v3p_i2c_peripherals.md` §6; `ntp_server_software_spec.md` §2.7/§2.8/§6/§9.6.

---

## 1. Topology summary

| Input | Count | Entry connector | TVS array | Conditioning | Logic pin |
|---|---|---|---|---|---|
| Buttons | 7 | J11 (1×08) | U19 (4 ch) + U20 (3 ch) — TPD4E02B04DQAR | 10 kΩ→3V3_STM + 100 nF | U54 GPA0–GPA6 |
| Tamper | 1 | J10 (1×02) | U20 ch4 — TPD4E02B04DQAR | 1 nF, **no external PU** | PC13 / TAMP |
| Touch INT | 1 | J4 pin 13 (display flex) | **U17 spare ch — TPD4E05U06DQAR** | 10 kΩ→3V3_STM, **no cap** | U54 GPA7 |

All three are **active-low, idle-high**. Buttons/tamper short to GND on actuation; touch INT is a
driven pulse from the FT6336U.

---

## 2. Buttons — U54 GPA0–GPA6

```
[Panel SPST]──J11──TVS(U19/U20)──●node●──GPAn
                                 │      │
                         10k→3V3_STM  100nF→GND
```

| Net | TVS pin | Pull-up | Debounce C | GPA / U54 pin |
|---|---|---|---|---|
| BUTTON_1 | U19 D1+ (p1) | R190 10 kΩ | C161 0.1 µF | GPA0 / 17 |
| BUTTON_2 | U19 D1− (p2) | R191 | C162 | GPA1 / 18 |
| BUTTON_3 | U19 D2+ (p4) | R192 | C163 | GPA2 / 19 |
| BUTTON_4 | U19 D2− (p5) | R193 | C164 | GPA3 / 20 |
| BUTTON_5 | U20 D1+ (p1) | R194 | C165 | GPA4 / 21 |
| BUTTON_6 | U20 D1− (p2) | R195 | C166 | GPA5 / 22 |
| BUTTON_7 | U20 D2+ (p4) | R196 | C167 | GPA6 / 23 |

- **Idle (open):** 10 kΩ holds the line at 3V3_STM → high.
- **Pressed:** SPST ties the line to GND → low.
- **Debounce:** τ = R·C = 10 kΩ × 100 nF = **1 ms** (release edge; the dominant filter). Press-edge
  cap dump is limited by cable series R, so no dedicated series resistor is fitted.
- **TVS flow-through:** each net lands on its protected D-pin (1/2/4/5) **and** the diagonal NC pin
  (6/7/9/10) as a routing feed-through. The NC pins are internally isolated — harmless, intended use.
- All pull-ups on **3V3_STM** (always-on; the expander rail), so buttons read valid whenever the MCU runs.

---

## 3. Tamper — PC13 / TAMP (backup domain)

```
[Lid SPST, NC]──J10──TVS(U20 ch4)──●RTC_TAMPER●──PC13
                                   │
                               C44 1nF→GND   (NO external pull-up)
```

| Element | Value | Purpose |
|---|---|---|
| TVS | U20 D2− (p5), TPD4E02B04DQAR | Enclosure-cable ESD |
| Cap | C44 **1 nF** | Holds the line between TAMP precharge samples; debounce |
| Pull-up | **none external** | Provided by the TAMP internal precharge pull-up (backup-domain powered) |
| Connector | J10 pin 1 = RTC_TAMPER, pin 2 = GND | Lid/intrusion contact across 1–2 |

**Why no external pull-up:** tamper moved to the **backup (VBAT/supercap) domain** so intrusion is
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

## 4. Touch INT — U54 GPA7

```
FT6336U INT──J4.13──TVS(U17 spare ch)──●DISP_TOUCH_INT●──GPA7
   (3.3V, push-pull, active-low)        │
                                  R208 10k→3V3_STM   (NO cap)
```

| Element | Value | Purpose |
|---|---|---|
| TVS | **U17 spare channel, TPD4E05U06DQAR** (5.5 V V_RWM) | Display-flex ESD; touch cable is high-touch |
| Pull-up | R208 10 kΩ → **3V3_STM** | Defines "no touch" = high while the FT6336U is unpowered (gated 5 V rail) |
| Cap | **none** | A debounce cap would swallow the INT pulse |
| Pin | U54 GPA7 / 24; FT6336U INT arrives on J4 pin 13 | On-change → INTA → BTN_INT_N |

- **3.3 V line.** Although touch I²C (CTP_SCL/SDA, J4 p10/p12) idles at 5 V behind the PCA9306, the
  INT (J4 p13) is a **3.3 V** signal direct from the FT6336U (not through the module's 74LVC245). It
  fits the U17 5.5 V array trivially; the array is 5.5 V-rated because of whatever else it protects,
  not because the INT needs it.
- **Pull-up rail rationale:** on 3V3_STM (always-on) so the line is defined high when the display
  rail (5V_DISP, gated by DISP_EN) is off — a gated-rail pull-up would let GPA7 float into a
  phantom wake. Residual: ~260 µA flows through R208 into the FT6336U's unpowered INT pin; harmless,
  current-limited, into a part held in reset.
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
MCP23017 input survives the residual on its own ESD cells — acceptable for a spare-channel reuse.

---

## 6. Firmware implications

### 6.1 MCP23017 U54 @0x21 (Port A = buttons + touch INT)

- **MIRROR = 0** (Port A and Port B INTs separate). Port A INTA → `BTN_INT_N` → PE0/EXTI0
  (R201 10 kΩ pull-up). Port B INTB → `INA_ALERT_INT_N` → PC12/EXTI12 (R199).
- **GPINTEN** set on GPA0–GPA7 (interrupt-on-change for all 8).
- **INTCON = 0** on Port A → compare-to-previous (on-change), not compare-to-DEFVAL.
- **IPOL:** set buttons asserted = 1 (a press reads as 1 in GPIOA/INTCAPA). Touch INT is edge-served,
  so IPOL on GPA7 is cosmetic.
- **ISR flow (PE0 falling):** read **INTCAPA** (latched state at the interrupt, also clears INTA) or
  GPIOA; XOR against the last snapshot to find which bit(s) changed.
  - GPA0–6 changed → debounce (~20–30 ms, software timer or re-sample) → emit press/long-press/repeat
    to the nav state machine.
  - GPA7 changed → **no debounce**; service the FT6336U over I²C (the gated 5 V bus behind PCA9306).
- **Shared reset:** `EXP_RST_N` (PC0, R198 pull-down) resets U55+U54; re-init both after any pulse.
- **Wake:** touch (GPA7) is the **primary** UI wake; any button also wakes. Wake from blank is a
  `DISP_BL` PWM duty change only — DISP_EN and the 5 V rail stay asserted through dim/blank, so the
  FT6336U stays powered and its INT stays live.
- **Touch bring-up sequencing:** on DISP_EN enable, allow rail soft-start, release DISP_RST (PA8
  high) past the FT6336U reset margin, then I²C-init touch. Mask `V_DISP_EN_FAULT` (U55 GPB2) for the
  soft-start window so inrush current-limit isn't logged as a fault.

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

- I²C wedge recovery: toggle `I2C_BUF_EN` (PA9, LTC4311 EN); if persistent, pulse `EXP_RST_N` (PC0)
  to hard-reset U55/U54. Housekeeping is on always-on 3V3_STM, so there is no rail-cycle path — the
  reset pin is the recovery lever.

---

## 7. Purpose recap (per element)

| Element | Purpose |
|---|---|
| 10 kΩ pull-up (R190–R196, R208) | Define idle-high; set active-low sense |
| 100 nF (C161–C167) | 1 ms RC button debounce |
| 1 nF (C44) | Precharge-compatible tamper hold/debounce |
| U19/U20 TPD4E02B04 | Connector ESD, 3.3 V-class (low clamp) for buttons + tamper |
| U17 TPD4E05U06 spare ch | Connector ESD for the 3.3 V touch INT, reusing a 5.5 V array's free channel |
| R201 (BTN_INT_N PU) | Single pull-up on the open-drain INTA net to PE0 |
| EXP_RST_N / R198 | Shared expander reset, default-asserted via pull-down |

---

## 8. Open items / flags

1. **U17 ESD locality (priority).** ESD must clamp at the cable entry. DISP_TOUCH_INT enters at the
   display flex **J4 pin 13** — confirm **U17 sits at J4**. If U17 is a remote array (e.g., the USB
   connector), a J4 strike propagates across the board before clamping and the protection is
   ineffective; in that case keep the INT on the local display array (U17 has spare channels) or add
   a local single-channel TVS at J4. **Confirm U17's location and which of its channels DISP_TOUCH_INT
   uses.**
2. **U17 channel count.** If DISP_TOUCH_INT moved off U17, U17 now carries only the two 5 V touch
   I²C lines (2 of 4 used). Update `sts1000_3v3p_i2c_peripherals.md` §6.5 accordingly.
3. **Single pull-up on DISP_TOUCH_INT.** R208 (here) must *replace* R167 (display sheet), not add to
   it — two parallel 10 kΩ = 5 kΩ and ~doubled back-feed (~520 µA) into the unpowered FT6336U pin.
4. **TAMP internal R_PU.** Confirm the STM32H563 tamper internal pull-up value (datasheet) and set
   TAMPPRCH so 1 nF reaches valid V_IH inside the precharge window.
5. **Tamper power-off scope.** Confirm the supercap/VBAT budget covers the chosen TAMPFREQ precharge
   draw for the required power-off detection duration.
6. **Designator/doc sync:** U54↔U48/U51, R208↔R167; R207 gap (confirm used elsewhere); A0→VS / A1,A2→GND
   strap for 0x21; INTA(R201)/INTB(R199) pull-ups and U54 VDD=3V3_STM + 0.1 µF — all off-crop, verify
   on the full sheet.
