# STS1000 Net / Rail Naming Convention

Canonical naming rules for schematic net labels (KiCad **global labels**) and power
rails (KiCad **power-symbol Value**). Net names in firmware/devicetree and in every doc
**must** match these. A net that drifts from this convention is a defect.

---

## Rules

| Class | Rule | Examples |
|---|---|---|
| **Supply rails** | Voltage-first, **no `+`**, **no decimal point** (`3V3`, `3V0`, `5V`). Optional `_<domain>` suffix. | `3V3`, `3V3_STM`, `3V0_RF`, `5V`, `5V_DISP` |
| **Functional rails** | Keep the functional name where the rail is a distinct domain, not a generic logic supply. | `VCC_RB`, `V_ANT`, `V_POE`, `VDDA`, `VAUX`, `VREF_3V0`, `VREF_3V3`, `GPS_VBAT`, `STM_VBAT`, `OCXO_V`, `OCXO_VC`, `VOUT_P/N`, `GND` |
| **Active-low** | `_N` **suffix** (never `n`-prefix, never `~{}`), applied where the part datasheet defines the pin active-low. STM32 core reset keeps the ST name `MCU_NRST`. | `GPS_RST_N`, `LAN_RST_N`, `NOR_RST_N`, `GPS_SAFEBOOT_N`, `WDO_N`, `KILL_N`, `V_ANT_EN_FAULT_N`, `V_DISP_EN_FAULT_N`, `PANEL_LED_FAULT_N`, `MCU_NRST` |
| **Reset** | `RST`, not `RESET`. | `GPS_RST_N`, `LAN_RST_N`, `NOR_RST_N` |
| **Power-good** | `<rail>[_<stage>]_PG`. Keep the `_PSU_` (switcher) / `_LDO_` (linear) stage qualifier **only** where a rail has two distinct PG nets. | `3V3_PSU_PG`, `5V_PSU_PG`, `OCXO_PSU_PG`, `OCXO_LDO_PG`, `3V3_GPS_LDO_PG`, `3V0_RF_LDO_PG`, `RB_PSU_PG`, `POE_PG` |
| **INA228 alert** | `INA_ALERT_<rail>` using the unified rail name; **no `_N` suffix** (deliberate). Each INA228 alert lands on its own MCU GPIO — there is **no** wire-OR aggregate net. | `INA_ALERT_3V3`, `INA_ALERT_3V3_STM`, `INA_ALERT_5V_DISP`, `INA_ALERT_5V_PANEL`, `INA_ALERT_OCXO`, `INA_ALERT_VCC_RB`, `INA_ALERT_V_ANT`, `INA_ALERT_V_POE`, `INA_ALERT_3V3_GPS` |
| **Differential / split pairs** | `_P` / `_N` suffix. | `TRD0_P/N`, `TRD1_P/N`, `VOUT_P/N` |
| **Bus prefixes** | Subsystem prefix retained. | `RMII_*`, `SPI_*`, `I2C_*`, `GPS_*`, `RB_RS232_*`, `DISP_*`, `POE_*`, `WDT_*` |

---

## Conventions in practice — exceptions and near-collisions

These are the deliberate departures from a naive reading of the rules; each is intentional
and must be preserved:

- **Stage qualifier kept where a rail has two PG nets.** `OCXO_PSU_PG` (switcher pre-reg) and
  `OCXO_LDO_PG` (LDO) are **distinct nets**; collapsing either to `<rail>_PG` would merge them.
  `RB_PSU_PG`, `3V3_PSU_PG`, `5V_PSU_PG` keep the `_PSU_` qualifier for the same reason.
- **`OCXO_VC` vs `OCXO_V` are different nodes, not a collision.** `OCXO_VC` = PA4 DAC-out
  (steering command); `OCXO_V` = the filtered Vcontrol at Y3.1 (sensed on PA3). Keep both.
- **`RB_OV_RESET` carries no `_N`.** The OV-latch reset pin polarity is not datasheet-confirmed,
  so the active-low suffix is not asserted.
- **`_P`/`_N` for polarity, not differential pairs, in two blocks:**
  - Panel-LED: `PANEL_LEDS_P` = common-anode rail; `PANEL_LED_WHITE_N_1..6` and
    `PANEL_LED_RED_N_1` = individual cathode returns.
  - Rb-IO RS-232: `RB_RS232_RX_P` / `RB_RS232_TX_P` = transceiver/relay-side nodes
    (U46/U47/K1), vs the line-side `RB_RS232_RX` / `RB_RS232_TX` at J6.
- **`_IN` suffix for conditioned connector signals.** A raw connector-side signal conditioned
  to an internal net keeps the `_IN` form on the connector side (`ENC_A_IN`→`ENC_A`,
  `ENC_B_IN`→`ENC_B`, `RB_LOCK_IN`→`RB_LOCK`).
- **RT9742 fault flags are `_N`.** `V_ANT_EN_FAULT_N` (U27.3), `V_DISP_EN_FAULT_N` (U33.3),
  `PANEL_LED_FAULT_N` (U55.3) — each is an open-drain `nFLG` (active-low) with a pull-up.
- **No aggregate ALERT/interrupt nets.** Each INA228 ALERT and each button/PG lands on its own
  MCU GPIO; there is no wire-OR aggregate net.

---

## Decision Log

- **Voltage-first rail names, no `+`, no decimal point.** `3V3` / `3V0` / `5V` sort and parse
  cleanly, avoid the KiCad `.`-in-label ambiguity, and give a single canonical token per rail
  that firmware/devicetree and every doc can match verbatim.
- **`_N` suffix (never `n`-prefix or `~{}`) for active-low.** A trailing token survives net
  mangling in netlists/DT identifiers and reads unambiguously; it is asserted only where the
  part datasheet defines the pin active-low. STM32 core reset keeps ST's `MCU_NRST` name.
- **`RST`, not `RESET`.** One reset token across the board.
- **`<rail>[_<stage>]_PG` for power-good,** with the `_PSU_`/`_LDO_` stage qualifier retained
  only where a rail has two distinct PG nets — so no two power-goods ever share a name.
- **`INA_ALERT_<rail>` with no `_N`.** Deliberate: the alert net is named for its rail and each
  lands on a dedicated GPIO, so the active-low sense lives in the pin table, not the net name.
