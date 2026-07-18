# STS1000 "Meridian" — Front-Panel Controls (7 buttons + optical encoder)

**Subsystem design record.** Defines the front-panel human-input set: one **optical incremental
encoder** (quadrature A/B + integral push switch) and **seven momentary buttons**. The
capacitive touchscreen (FT6336U) remains the **primary** interaction and wake surface; the
physical controls are the **no-look / dead-screen / dark-site** path and must remain useful when
the panel is blanked or the touch layer has failed.

**Canonical authority:** `ntp_server_peripheral_map.md` §8 (UI pins/rails) and §6 (power domains)
are authoritative; fault/UI aggregation is a direct-GPIO scan per `sts1000_fault_aggregation.md`.
This record holds the control-map decisions and rationale.

**Design intent:** no buzzer exists in the BOM — all alarming is **visual** (RGB status LED, PD12–
PD14 / TIM4) plus network (SNMP). No "silence" function is assigned. Holdover-force and reference-
source override are **not** front-panel functions; they live in the touchscreen menu and web UI
only, leaving **RESET** as the sole timing-perturbing physical control.

---

## 0. Decisions (conclusions first)

| # | Decision | Rationale |
|---|----------|-----------|
| D1 | 7 buttons + encoder (quad + switch) | Touchscreen carries rich/occasional config; physical set kept lean and purposeful |
| D2 | Encoder push = **Enter/Select**; dedicated **BACK** button is its partner | Encoder forward/commit + tactile reverse = classic eyes-free pairing; frees a button for RESET |
| D3 | Quadrature A/B → **STM32 TIM1 encoder-interface mode**, committed **PA8/PA9** (via U68 5 V-safe buffer) | Hardware timer decode avoids the I²C round-trip latency that would drop counts at optical edge rates |
| D4 | Encoder **switch** (`ENC_BUTTON`/PF11) + 7 buttons (`BUTTON_1..7`/PF0–PF6) → **direct STM32 GPIO** (GPIOF), firmware ~1 kHz scan (+ optional EXTI wake) | Human-rate inputs read directly on GPIO, zero I²C latency |
| D5 | **DISPLAY** is a dedicated physical key (not menu-only dimming) | Dark-site: you cannot navigate a menu you cannot see — un-blank must be one known key |
| D6 | **RESET** guarded (long-press + confirm); cold-cycle behind ≥10 s / combo | Resetting re-disciplines the OCXO; never a single tap |
| D7 | A/B edge conditioning via **TIM input filter (ICxF)**, not RC | Avoids rounding optical edges below decode margin; pull-up set by noise/current, not bandwidth |
| D8 | All panel I/O + both panel rails (`5V_DISP`, `5V`) on **one 36-pin J17** | Single panel harness; display draws 5 V on J17.16, encoder/panel logic on J17.28 |

---

## 1. Interaction model

- **Touchscreen** — primary rich UI (config, GNSS sky plot, dashboards). Wake-on-touch is primary.
- **Encoder** — eyes-free navigation + value adjust; push = Enter/Select. On the Home screen,
  rotation cycles dashboard pages.
- **7 buttons** — dedicated, single-purpose, no-look functions that survive a dead or blanked screen.

Dashboard page order (encoder rotation on Home): **Time/sync → GNSS sky/SVs → References
(GPS/OCXO/Rb, holdover) → Network (IP/NTP/PTP) → Faults/log**.

---

## 2. Optical encoder

### 2.1 Function

| Action | Function |
|--------|----------|
| Rotate CW/CCW | Move selection / increment–decrement field. On Home: cycle dashboard pages |
| Push (short) | **Enter / Select / commit** |
| Push (long ≥1 s) | Item context action / jump Home |

### 2.2 Electrical & routing

The encoder connects through the consolidated **36-pin panel connector J17**, powered from the
panel logic rail on **J17.28 = 5 V**. Its raw quadrature lines `ENC_A_IN`/`ENC_B_IN` (J17.11/J17.27)
pass through **U68 74LVC2G17** (dual Schmitt buffer, Vcc = 3V3_STM, 5 V-tolerant inputs) to the
conditioned nets `ENC_A`/`ENC_B` at the MCU. PA8/PA9 are the committed encoder pins and never see 5 V.

| Line | Net | Destination | Notes |
|------|-----|-------------|-------|
| A | `ENC_A_IN` → `ENC_A` | U12 **PA8** = TIM1_CH1, via U68 buffer | J17.11 raw → U68 (5 V-safe Schmitt) → PA8 |
| B | `ENC_B_IN` → `ENC_B` | U12 **PA9** = TIM1_CH2, via U68 buffer | J17.27 raw → U68 → PA9 |
| SW | `ENC_BUTTON` | U12 **PF11** (direct GPIO) | J17.9; plain button; R236 10 kΩ→3V3_STM + C187 0.1 µF (~1 ms RC) + TVS U69 |

- **Output type: push-pull (driven) quadrature.** The optical encoder drives `ENC_A_IN`/`ENC_B_IN`
  actively both ways, so **no pull-up is required**. U68's inputs are high-Z Schmitt; a driven
  encoder output feeds them directly.
- **No pull-up, no series R/C on A/B.** A driven push-pull source needs neither; edge conditioning
  stays in the timer (`IC1F`/`IC2F`, see below). Only J17, the U65/U69 TVS channels, and the high-Z
  U68 Schmitt inputs sit on `ENC_A_IN`/`ENC_B_IN`.
- **Edge conditioning:** use the **TIM input filter** (`IC1F`/`IC2F`, N-sample at f_DTS) for
  glitch/jitter rejection rather than series-R/C, which would soften optical edges. A few-µs
  effective filter is ample at panel rates.

### 2.3 Firmware

- Encoder counter in HW (timer), read as a delta; **commit on switch *release*** and apply a
  ~50 ms rotation lockout around push edges so a press does not nudge the count.
- Switch debounce: RC (~1 ms) + firmware (~10–20 ms).

---

## 3. Button purpose map

| # | Label | Short press | Long press / guard |
|---|-------|-------------|--------------------|
| 1 | **MENU** | Open menu / Home (wakes panel) | — |
| 2 | **BACK** | Up one level / cancel edit / Esc | — |
| 3 | **DISPLAY** | Cycle backlight: Full → Dim → Night → Off | Instant blackout ↔ restore (dark-site) |
| 4 | **ACK** | Acknowledge active fault → clear latched RGB alarm to "acked," open fault list | — |
| 5 | **INFO** | Identity card: hostname, IPv4/IPv6, lock state, active reference, NTP/PTP/PTP-GM status | Full network detail |
| 6 | **LAMP TEST** | Momentary while held: drive RGB through R/G/B + screen test pattern | — |
| 7 | **RESET** | (no action — guarded) | ≥3 s + on-screen confirm → warm NVIC reset; ≥10 s (or MENU+RESET) → PoE cold-cycle |

Enter/Select is on the encoder push (D2), so it is **not** duplicated as a button.

---

## 4. Guarded / soft-power actions

| Control | Tier | Mechanism | Cross-ref |
|---------|------|-----------|-----------|
| DISPLAY long | none (non-destructive) | `DISP_BL` PWM (PE6/TIM15_CH2) → 0 %; panel rail (`DISP_EN`/PC11 → RT9742 U33) stays up for instant restore; MCU/timing never sleeps | peripheral map §8, §6 |
| RESET ≥3 s | warm | SCB AIRCR system reset (NVIC); confirm dialog first; OCXO re-disciplines on reboot | — |
| RESET ≥10 s / MENU+RESET | cold | `POE_KILL` (PE15, push-pull) → NCP1095 AUX kill → PSE port collapse < Vrst → re-power → cold boot | `sts1000_poe_kill.md` |

> `POE_KILL` (PE15) is push-pull with **no +3V3 pull-up** (a pull-up creates default-KILL/boot-loop
> during MCU reset). Cold-cycle is latch-recovery via port-voltage collapse — there is no in-band
> NCP1095 restart. Bench item: GBR/VOUT_P survival under AUX-assert (`sts1000_poe_kill.md`).

---

## 5. Electrical & GPIO allocation (direct GPIO)

**Per-button stage:** momentary to GND (active-low), **10 kΩ → 3V3_STM** + **100 nF** (≈ 1 ms RC),
plus connector TVS (TPD4E0x arrays U19/U20 on the keypad ribbon, U69 on the encoder). All seven
buttons + the touch INT + the encoder switch land on **STM32 GPIOF** directly; firmware adds
software debounce.

**Button/encoder map (GPIOF):**

| Net | U12 pin | Pull-up (→3V3_STM) | Debounce C | TVS |
|-----|---------|--------------------|------------|-----|
| `BUTTON_1` (MENU)    | PF0 (10) | R190 10 kΩ | C161 0.1 µF | U19 |
| `BUTTON_2` (BACK)    | PF1 (11) | R191 | C162 | U19 |
| `BUTTON_3` (DISPLAY) | PF2 (12) | R192 | C163 | U19 |
| `BUTTON_4` (ACK)     | PF3 (13) | R193 | C164 | U19 |
| `BUTTON_5` (INFO)    | PF4 (14) | R194 | C165 | U20 |
| `BUTTON_6` (LAMP)    | PF5 (15) | R195 | C166 | U20 |
| `BUTTON_7` (RESET)   | PF6 (18) | R196 | C167 | U20 |
| `DISP_TOUCH_INT`     | PF7 (19) | R208 10 kΩ | — (no cap) | U17 |
| `ENC_BUTTON`         | PF11 (49)| R236 10 kΩ | C187 0.1 µF | U69 |

The old GPA7 `BUTTON_8`-vs-`DISP_TOUCH_INT` conflict is moot: with direct GPIO, touch INT has
its own pin (PF7) and the encoder switch its own (PF11). The button→function assignment (MENU/
BACK/DISPLAY/ACK/INFO/LAMP/RESET) is a firmware label over `BUTTON_1..7`; confirm the physical
legend order against §3.

### 5.1 Consolidated panel connector J17 (36-pin)

The panel headers are consolidated into a single **36-pin J17** carrying the buttons, panel-LED
anode/cathodes, encoder, display bus, touch INT, and the reed-switch wake, plus both panel supply
rails: **J17.16 = 5V_DISP** (display 5 V, bypass C40 + TVS D6) and **J17.28 = 5 V** (encoder/panel
logic supply, bypass + TVS).

| Pin | Net | Pin | Net |
|---|---|---|---|
| 1 | `BUTTON_1` | 19 | `BUTTON_2` |
| 2 | `BUTTON_3` | 20 | `BUTTON_4` |
| 3 | GND | 21 | `BUTTON_5` |
| 4 | `BUTTON_6` | 22 | `BUTTON_7` |
| 5 | `PANEL_LED_WHITE_N_1` | 23 | `PANEL_LED_WHITE_N_2` |
| 6 | `PANEL_LED_WHITE_N_3` | 24 | `PANEL_LED_WHITE_N_4` |
| 7 | `PANEL_LED_WHITE_N_5` | 25 | `PANEL_LED_WHITE_N_6` |
| 8 | `PANEL_LED_RED_N_1` | 26 | `PANEL_LEDS_P` (LED anode) |
| 9 | `ENC_BUTTON` | 27 | `ENC_B_IN` |
| 10 | GND | 28 | `5V` (encoder/panel logic) |
| 11 | `ENC_A_IN` | 29 | (no-connect) |
| 12 | `DISP_CS` | 30 | `DISP_RST` |
| 13 | GND | 31 | `DISP_DC` |
| 14 | `DISP_BL` | 32 | `I2C_DISP_SCL` |
| 15 | `I2C_DISP_SDA` | 33 | `DISP_TOUCH_INT` |
| 16 | `5V_DISP` (display) | 34 | `SPI_MOSI_DISP` |
| 17 | `SPI_SCK_DISP` | 35 | GND |
| 18 | `PROX_WAKE` (reed switch) | 36 | GND |

---

## 6. Wake & display behavior

- Any button (`BUTTON_1..7`/PF0–PF6), the touch INT (PF7), and the proximity-wake input
  (`PROX_WAKE`/PF10) can wake the panel — read by the ~1 kHz GPIOF scan, and optionally armed as
  EXTI for low-latency wake. Primary wake remains **touch**; secondary is the **passive magnetic
  reed switch** (dry contact on J17.18, active-low → PF10; see `sts1000_3v3p_i2c_peripherals.md` §3).
  Encoder rotation does **not** wake (A/B go to TIM1, no wake path); a knob bump should not wake
  the panel — use the switch or any button.
- **Blackout blanks the panel only.** The MCU, disciplining loop, and all timing outputs keep
  running. Restore is instant (backlight PWM up), not a panel cold-start.
- Deep "panel off" (rail gated via `DISP_EN`) is reserved for idle-timeout/menu; it requires
  ST7796S/FT6336U re-init on wake and is **not** the DISPLAY-key path.

---

## 7. Cross-references

`ntp_server_peripheral_map.md` §6/§8 (rails, UI pins, display) · `sts1000_fault_aggregation.md`
(direct-GPIO fault/HMI scan on GPIOF/GPIOG) · `sts1000_firmware_hardware_interface.md` (button/
encoder register behavior) · `sts1000_poe_kill.md` (cold-cycle, PE15) ·
`sts1000_3v3p_i2c_peripherals.md` (display/touch subsystem, `DISP_BL`, `DISP_EN`/RT9742 U33) ·
`ntp_server_software_spec.md` (UI/menu, holdover & source override now web/touch-only).

---

## 8. Open items

- [ ] **Quadrature AF:** confirm TIM1_CH1/CH2 = PA8/PA9 AF for the encoder-interface mode (committed
      pins; U68 74LVC2G17 buffer makes the 5 V encoder safe into 3.3 V PA8/PA9).
- [ ] **TIM input filter:** set `IC1F`/`IC2F` divider + N for A/B glitch reject (target few µs).
- [ ] **Encoder-switch debounce:** `ENC_BUTTON` (PF11) = R236 10 kΩ + C187 0.1 µF (~1 ms RC) + FW.
- [ ] **RESET cold-cycle:** confirm `POE_KILL` long-hold path against the GBR/VOUT_P survival bench
      item (`sts1000_poe_kill.md`).
- [ ] **Legend order:** confirm the physical button legend (MENU/BACK/DISPLAY/ACK/INFO/LAMP/RESET)
      maps to `BUTTON_1..7` = PF0..PF6 as tabled in §5.

---

## 9. Panel indicator / backlight LED rail

Separate from the RGB status LED (D5, see `sts1000_rgb_indicator.md`): a dimmable string of
**6 white + 1 red** panel indicators driven common-anode off a switched, current-monitored 5 V
rail. High-side PNP pass with a discrete PWM level-shifter.

**Power/monitor chain:** `5V` → **U55 RT9742** (IN, EN = `PANEL_LED_EN`/PC0) → OUT → **R199
0.22 Ω** shunt (INA228 **U54 @0x4C** IN+/IN−) → **FB12** (600 Ω) → `V_PANEL_LED` → **Q22
NSS40300 PNP** emitter → collector = `PANEL_LEDS_P` (common anode) → **J17.26** → panel; cathodes
`PANEL_LED_WHITE_N_1..6` (J17.5/6/7/23/24/25) / `PANEL_LED_RED_N_1` (J17.8) return via J17 → ballast
→ sink → GND. (The LED string is on the consolidated J17.)

| Function | Net / ref | Detail |
|----------|-----------|--------|
| Rail enable | `PANEL_LED_EN` = PC0 | R198 10 kΩ pull-**down** → RT9742 U55 EN active-high ⇒ default OFF |
| Fault flag | `PANEL_LED_FAULT_N` = PF12 | U55 nFLG open-drain, R201 10 kΩ → 3V3_STM (renamed `_N`) |
| Current monitor | INA228 **U54 @0x4C** | shunt R199 0.22 Ω on the 5 V panel rail (`INA_ALERT_5V_PANEL` → PF13) |
| Brightness PWM | `PANEL_LED_PWM` = PE0 (LPTIM2) | PE0 → R216 4.7 kΩ → **Q23 BC847W** base (R235 10 kΩ pull-down); Q23 collector → R217 820 Ω → Q22 base (R234 4.7 kΩ off-hold). Non-inverting: PE0 high → Q23 sat → Q22 on. Hi-Z reset ⇒ double default-OFF |
| Ballast | white R247–R252 = **91 Ω** (×6); red R253 = **150 Ω** | rail ≈4.7 V ⇒ ~17–18 mA/LED, all-on ≈130 mA |

Shunt check: ~29 mV at all-on ≈ 71 % of the INA228 ±40.96 mV FS (≤75 %) — OK.
