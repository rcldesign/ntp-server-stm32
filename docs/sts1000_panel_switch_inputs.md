# STS1000 — Panel SPST Switch Inputs (Buttons + Tamper)

**Scope:** the off-board SPST contacts that route through a panel connector to a logic input —
the **7 keypad buttons** (→ STM32 **GPIOF PF0–PF6**, direct GPIO) and the **enclosure
tamper/intrusion switch** (→ STM32 **PC13 / `RTC_TAMPER`**, TAMP peripheral). Both use the same
conditioning chain: **connector TVS → 10 kΩ pull-up → debounce cap → input**. Active-low: switch
shorts the line to GND on actuation; the pull-up defines the idle-high state. (Buttons use 100 nF;
tamper uses 1 nF — see §3.)

Cross-refs: `ntp_server_peripheral_map.md` §8/§9/§11; `sts1000_fault_aggregation.md` §4 (direct-GPIO
scan); `sts1000_firmware_hardware_interface.md`; `ntp_server_software_spec.md` §2.7/§6.1/§9.6.

---

## 1. Signal chain (per line)

```
[Panel SPST] ──cable──┤ HDR ├── TVS→GND ──● node A ●── input pin
                                          │         │
                                  10k → 3V3_STM   100nF → GND
```

- **Idle (open):** 10 kΩ holds node A at 3V3_STM → logic high.
- **Actuated (closed):** switch ties node A to GND through the cable → logic low.
- **Debounce:** τ = R·C = 10 kΩ × 100 nF = **1 ms** (release edge; the dominant filter). Cable
  series R limits the press-edge cap dump, so no dedicated series resistor is fitted.
- **ESD:** TVS array at the connector — first thing the external cable sees, return to chassis/GND.

---

## 2. Button lines — U12 PF0–PF6 (`BUTTON_1`…`BUTTON_7`), direct GPIO

| Net | U12 pin | Pull-up | Debounce C | TVS |
|---|---|---|---|---|
| `BUTTON_1` | PF0 (10) | R190 10 kΩ→3V3_STM | C161 0.1 µF | U19 |
| `BUTTON_2` | PF1 (11) | R191 | C162 | U19 |
| `BUTTON_3` | PF2 (12) | R192 | C163 | U19 |
| `BUTTON_4` | PF3 (13) | R193 | C164 | U19 |
| `BUTTON_5` | PF4 (14) | R194 | C165 | U20 |
| `BUTTON_6` | PF5 (15) | R195 | C166 | U20 |
| `BUTTON_7` | PF6 (18) | R196 | C167 | U20 |

Each line: momentary SPST → GND (active-low), 10 kΩ → **3V3_STM** idle-high, 100 nF → GND
(1 ms RC), connector TVS (TPD4E02B04, U19 for BUTTON_1–4, U20 for BUTTON_5–7).

**Panel entry:** the seven buttons enter on the consolidated **36-pin J17** —
`BUTTON_1/2/3/4/5/6/7` = J17.**1/19/2/20/21/4/22**. J17 also carries the encoder, panel LEDs,
display bus, touch INT, the reed-switch wake, and the two panel supply rails (J17.16 = 5V_DISP,
J17.28 = 5 V); the buttons themselves are passive contacts and draw no rail.

**Register/INT config:** direct GPIO. Firmware reads GPIOF on a ~1 kHz scan and
diffs against the previous snapshot; a pressed line reads **low**. Buttons may additionally be
armed as EXTI on their PF pins for low-latency wake. Firmware adds ~20–30 ms debounce on top of
the 1 ms RC.

**PF7 (not a button):** `DISP_TOUCH_INT` (touch controller) gets the 10 kΩ → 3V3_STM pull-up
(R208) but **no cap** — a debounce cap would swallow the push-pull touch INT pulse. Read on the
same GPIOF scan (or EXTI).

---

## 3. Tamper line — PC13 (`RTC_TAMPER`)

Same external pattern (SPST + pull-up + cap + TVS), but the input is the STM32 **TAMP** peripheral
pin in the **backup (VBAT) domain**, not an expander and not an EXTI GPIO line. Net = `RTC_TAMPER`
(J10.1 → PC13); TVS = U16 (TPD4E02B04, shared channel).

| Item | Value / part | Function |
|---|---|---|
| Pull-up R | **none external** — TAMP internal precharge pull-up (backup domain) | Idle level (see §3.1) |
| Debounce C | **C201 = 1 nF** → GND | precharge-compatible hold; see §3.1 |
| TVS | U16 ch (TPD4E02B04, 3.3 V) | Connector/enclosure-cable ESD |
| Switch | SPST enclosure/lid intrusion contact | See §3.2 polarity |

> `RTC_TAMPER` carries only C201 (1 nF), J10, PC13 and the U16 TVS — **no external pull-up**. It
> relies on the STM32 TAMP block's **internal pull-up + periodic precharge** (§3.1). The 1 nF C0G is
> the correct value for precharge sampling: a 100 nF cap would not charge inside the precharge window
> and would false-trip. `sts1000_panel_ui_inputs_esd.md` §3 is the detailed reference for the tamper
> input.

**Function:** `RTC_TAMP` events are timestamped in the backup domain, written to the tamper-evident
audit log, raise an SNMP trap, and (policy-gated) can zeroize ATECC608B/PSA-wrapped key material.
Handled by the RTC/TAMP peripheral — **not** an EXTI source (EXTI13 stays free).

### 3.1 Precharge-mode rationale (confirm against RM0481 TAMP config)

The tamper input runs in the TAMP block's **internal pull-up + periodic precharge** mode, powered
from the **backup (VBAT/supercap) domain**, so intrusion is sensed with main power off. Two facts
drive the topology:

1. **Power-off detection sets the rail.** A main-rail (3V3_STM) pull-up would sense tamper only while
   the board is powered; the backup-domain TAMP internal pull-up detects lid-open with the unit off.
2. **Precharge sets the cap.** In precharge-sampling mode the internal pull-up connects only for a
   short window before each sample. A 100 nF cap would not charge inside that window → false tamper.
   The **1 nF (C201)** external cap charges inside the precharge window and gives valid ESD/debounce
   hold. Set TAMPPRCH/TAMPFLT per RM0481; use the TAMP digital filter for debounce.

### 3.2 Switch polarity (security choice)

Use a **normally-closed** contact held closed by the secured lid: closed = line pulled to GND
(secure); lid open **or a cut/disconnected cable** = pull-up takes the line high = tamper. This
fails secure against tampering with the switch wiring itself. A normally-open contact is simpler but
does not flag a cut wire. Set TAMP active edge/level to match the chosen contact.

---

## 4. TVS selection

| Net group | Lines | Part | Notes |
|---|---|---|---|
| Buttons | 7 | **U19 + U20** TPD4E02B04DQAR | 3.3 V working, ~0.25 pF, 4 ch/array; U19 = BUTTON_1–4, U20 = BUTTON_5–7 |
| Tamper | 1 | **U16** ch (TPD4E02B04DQAR, shared) | 3.3 V working; single channel |

All TVS placed at the **connector**, clamp return to chassis/GND, ahead of the pull-up node. These
are 3.3 V-class logic lines (unlike the display touch I²C lines, which idle at 5 V and require the
5.5 V-class TPD4E05U06DQAR on U17) — the lower-voltage 3.3 V arrays are correct here.

---

## 5. Summary table

| Parameter | Buttons (×7) | Tamper (×1) |
|---|---|---|
| Input | STM32 PF0–PF6 (direct GPIO, R190–R196) | STM32 PC13 / TAMP (backup domain) |
| Idle level | High (10 kΩ → 3V3_STM) | High (TAMP internal precharge pull-up) |
| Actuated | Low (SPST → GND) | Low (SPST → GND) |
| Debounce | 100 nF (C161–C167), 1 ms RC + FW 20–30 ms | 1 nF (C201), precharge-compatible (§3.1) |
| TVS | TPD4E02B04 (U19/U20, 3.3 V) | TPD4E02B04 (U16, 3.3 V) |
| Interrupt path | ~1 kHz GPIOF scan (+ optional EXTI) | TAMP peripheral (no EXTI) |
| Polarity | NO momentary, reads low pressed | NC intrusion preferred (§3.2) |

**Open items:** (1) confirm the tamper precharge cycles (TAMPPRCH) + internal R_PU value against
RM0481 so C201 (1 nF) reaches valid V_IH inside the precharge window (§3.1); (2) confirm panel
connector pinout/keying — the 7 buttons are on the consolidated **36-pin J17** (J17.1/19/2/20/21/4/22);
the tamper is on its own **J10** (backup domain, not routed through J17).
