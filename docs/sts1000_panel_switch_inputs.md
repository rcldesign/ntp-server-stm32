# STS1000 — Panel SPST Switch Inputs (Buttons + Tamper)

**Scope:** the off-board SPST contacts that route through a panel connector to a logic input —
the **7 keypad buttons** (→ MCP23017 **U54 @0x21** Port A, GPA0–GPA6) and the **enclosure
tamper/intrusion switch** (→ STM32 **PC13 / `RTC_TAMP`**). Both use the same conditioning chain:
**connector TVS → 10 kΩ pull-up → 100 nF debounce → input**. Active-low: switch shorts the line
to GND on actuation; the pull-up defines the idle-high state.

Cross-refs: `ntp_server_peripheral_map.md` §8/§9/§11; `sts1000_fault_aggregation.md` §1; `ntp_server_software_spec.md` §2.7/§6.1/§9.6.

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

## 2. Button lines — U54 GPA0–GPA6 (`BUTTON_1`…`BUTTON_7`)

| Item | Value / part | Per line | Function |
|---|---|---|---|
| Pull-up R | 10 kΩ → **3V3_STM** | ×7 | Idle-high; sets the un-pressed level |
| Debounce C | 100 nF → GND | ×7 | 1 ms RC with the 10 kΩ |
| TVS array | TPD4E02B04DQA (3.3 V, ~0.25 pF) | 2 arrays (4 ch ea., 7 used) | Connector ESD on 3.3 V logic |
| Switch | SPST, line→GND on press | ×7 (panel) | Momentary, normally-open |

**Designators:** assign per-line R/C in KiCad (the 10 kΩ and 100 nF reuse existing value lines — no
new BOM parts). **R201 (10 kΩ → 3V3_STM) is *not* one of these** — it is the single pull-up on the
open-drain `BTN_INT_N` net (U54 INTA → PE0/EXTI0), not a per-button part.

**Register/INT config:** **IPOL=1** (pressed reads as 1); on-change interrupt (`GPINTEN`,
`INTCON`=0) → INTA → `BTN_INT_N` → PE0/EXTI0. Firmware adds ~20–30 ms debounce on top of the 1 ms RC.

**GPA7 exception (not a button):** GPA7 = `DISP_TOUCH_INT` (FT6336U) gets the 10 kΩ → 3V3_STM
pull-up (R208) but **no 100 nF** — the cap would swallow the push-pull touch INT pulse. GPA7 rides
the same INTA/`BTN_INT_N` path.

---

## 3. Tamper line — PC13 (`RTC_TAMP`)

Same external pattern (SPST + pull-up + cap + TVS), but the input is the STM32 **TAMP** peripheral
pin in the **backup (VBAT) domain**, not an expander and not an EXTI GPIO line.

| Item | Value / part | Function |
|---|---|---|
| Pull-up R | 10 kΩ → **3V3_STM** (see §3.1 backup-domain note) | Idle level |
| Debounce C | 100 nF → GND | 1 ms RC; see §3.1 TAMP-precharge caveat |
| TVS | 1 ch of a 3.3 V low-cap array (TPD4E02B04 spare ch or TPD1E10B06) | Connector/enclosure-cable ESD |
| Switch | SPST enclosure/lid intrusion contact | See §3.2 polarity |

**Function:** `RTC_TAMP` events are timestamped in the backup domain, written to the tamper-evident
audit log, raise an SNMP trap, and (policy-gated) can zeroize ATECC608B/PSA-wrapped key material.
Handled by the RTC/TAMP peripheral — **not** an EXTI source (EXTI13 stays free).

### 3.1 Two open hardware decisions — confirm against RM0481 TAMP config

1. **Pull-up rail vs. power-off detection.** A 3V3_STM pull-up means tamper is sensed **only while
   the board is powered**. To detect lid-open while the unit is off, the pull-up must sit on the
   **VBAT-backed supercap rail** *or* use the **TAMP internal pull-up + precharge** in VBAT mode.
   Decide whether power-off intrusion detection is in scope.
2. **External 100 nF vs. internal precharge.** If the TAMP input is run in **passive/precharge
   sampling** mode (low-power, periodic precharge through an internal resistor), an external 100 nF
   may not charge inside the precharge window → **false tamper**. Two clean options:
   - **Level/continuous mode, external pull-up:** the external 10 kΩ + 100 nF debounce is valid;
     disable internal precharge. (Matches the as-specified button topology.)
   - **Internal pull-up + precharge:** drop the external cap, lengthen the internal precharge/
     sample interval per RM0481, and use the TAMP digital filter for debounce instead.

   The as-drawn design assumes the first option (external R+C, continuous level detect).

### 3.2 Switch polarity (security choice)

Use a **normally-closed** contact held closed by the secured lid: closed = line pulled to GND
(secure); lid open **or a cut/disconnected cable** = pull-up takes the line high = tamper. This
fails secure against tampering with the switch wiring itself. A normally-open contact is simpler but
does not flag a cut wire. Set TAMP active edge/level to match the chosen contact.

---

## 4. TVS selection

| Net group | Lines | Part | Notes |
|---|---|---|---|
| Buttons | 7 | TPD4E02B04DQA ×2 | 3.3 V working, ~0.25 pF, 4 ch/array; reuses the display-cable ESD part |
| Tamper | 1 | spare TPD4E02B04 ch, or TPD1E10B06DPL | 3.3 V working; single channel acceptable |

All TVS placed at the **connector**, clamp return to chassis/GND, ahead of the pull-up node. These
are 3.3 V-class logic lines (unlike the display touch I²C lines, which idle at 5 V and require the
5.5 V-class TPD4E05U06DQAR on U17) — the lower-voltage 3.3 V arrays are correct here.

---

## 5. Summary table

| Parameter | Buttons (×7) | Tamper (×1) |
|---|---|---|
| Input | MCP23017 U54 GPA0–6 (I²C) | STM32 PC13 / TAMP (backup domain) |
| Idle level | High (10 kΩ → 3V3_STM) | High (10 kΩ → rail per §3.1) |
| Actuated | Low (SPST → GND) | Low (SPST → GND) |
| Debounce | 100 nF, 1 ms RC + FW 20–30 ms | 100 nF, 1 ms RC (see §3.1) |
| TVS | TPD4E02B04 (3.3 V) | TPD4E02B04 / TPD1E10B06 (3.3 V) |
| Interrupt path | INTA → `BTN_INT_N` → PE0/EXTI0 | TAMP peripheral (no EXTI) |
| Polarity | NO momentary, IPOL=1 | NC intrusion preferred (§3.2) |

**Open items:** (1) tamper pull-up rail + precharge mode (§3.1); (2) per-line R/C designator
assignment in KiCad; (3) confirm panel connector pinout/keying for the 7 buttons + tamper bundle.
