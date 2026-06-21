# STS1000 Net / Rail Naming Convention

Canonical naming rules for schematic net labels (KiCad **global labels**) and power
rails (KiCad **power-symbol Value**). Net names in firmware/devicetree and in every doc
**must** match these. A net that drifts from this convention is a defect.

---

## Rules

| Class | Rule | Examples |
|---|---|---|
| **Supply rails** | Voltage-first, **no `+`**, **no decimal point** (`3V3`, `3V0`, `5V`). Optional `_<domain>` suffix. | `3V3`, `3V3_STM`, `3V0_RF`, `5V`, `5V_DISP` |
| **Functional rails** | Keep the functional name where the rail is a distinct domain, not a generic logic supply. | `VCC_RB`, `V_ANT`, `V_POE`, `VDDA`, `VAUX`, `VREF_4V096`, `GPS_VBAT`, `STM_VBAT`, `OCXO_V`, `OCXO_VC`, `VOUT_P/N`, `GND` |
| **Active-low** | `_N` **suffix** (never `n`-prefix, never `~{}`), applied where the part datasheet defines the pin active-low. STM32 core reset keeps the ST name `MCU_NRST`. | `GPS_RST_N`, `LAN_RST_N`, `EXP_RST_N`, `NOR_RST_N`, `BTN_INT_N`, `PG_INT_N`, `INA_ALERT_INT_N`, `WDO_N`, `KILL_N`, `MCU_NRST` |
| **Reset** | `RST`, not `RESET`. | `GPS_RST_N`, `EXP_RST_N` |
| **Power-good** | `<rail>[_<stage>]_PG`. Keep the `_PSU_` (switcher) / `_LDO_` (linear) stage qualifier **only** where a rail has two distinct PG nets. | `3V3_PSU_PG`, `5V_PSU_PG`, `OCXO_PSU_PG`, `OCXO_LDO_PG`, `3V3_GPS_LDO_PG`, `3V0_RF_LDO_PG`, `RB_PSU_PG`, `POE_PG` |
| **INA228 alert** | `INA_ALERT_<rail>` using the unified rail name; the wire-OR aggregate is `INA_ALERT_INT_N`. | `INA_ALERT_3V3`, `INA_ALERT_3V3_STM`, `INA_ALERT_5V_DISP`, `INA_ALERT_OCXO`, `INA_ALERT_VCC_RB`, `INA_ALERT_V_ANT`, `INA_ALERT_V_POE` |
| **Differential / split pairs** | `_P` / `_N` suffix. | `TRD0_P/N`, `TRD1_P/N`, `VOUT_P/N` |
| **Bus prefixes** | Subsystem prefix retained. | `RMII_*`, `SPI_*`, `I2C_*`, `GPS_*`, `RB_RS232_*`, `DISP_*`, `POE_*`, `WDT_*` |

---

## Applied renames (2026-06, schematic-verified)

22 nets were renamed across all sheets; the netlist was re-parsed to confirm the
removed/added net-name sets match exactly and **all node connectivity was preserved**.

### Rails (power-symbol Value)
| Old | New | Old | New |
|---|---|---|---|
| `+3.3V` | `3V3` | `+3.3V_CLK` | `3V3_CLK` |
| `+3.3V_STM` | `3V3_STM` | `+3.0V_RF` | `3V0_RF` |
| `+3.3V_STM_USB` | `3V3_STM_USB` | `+5V` | `5V` |
| `+3.3V_GPS` | `3V3_GPS` | `V_DISP_5V` | `5V_DISP` |
| `+3.3V_LAN` | `3V3_LAN` | | |

### Signals (global labels)
| Old | New | Old | New |
|---|---|---|---|
| `GPS_nRST` | `GPS_RST_N` | `POE_PGO` | `POE_PG` |
| `GPS_nSAFEBOOT` | `GPS_SAFEBOOT_N` | `3.0V_RF_LDO_PG` | `3V0_RF_LDO_PG` |
| `LAN_nRST` | `LAN_RST_N` | `3V3_LDO_GPS_PG` | `3V3_GPS_LDO_PG` |
| `nRST` | `MCU_NRST` | `INA_ALERT_3.3V` | `INA_ALERT_3V3` |
| `NOR_RESET` | `NOR_RST_N` | `INA_ALERT_3.3V_GPS` | `INA_ALERT_3V3_GPS` |
| `EXP_RESET` | `EXP_RST_N` | `INA_ALERT_3.3V_STM` | `INA_ALERT_3V3_STM` |
| | | `INA_ALERT_V_DISP_5V` | `INA_ALERT_5V_DISP` |

### Deliberately **not** renamed (with reason)
- `RB_OV_RESET` — OV-latch reset; pin polarity not datasheet-confirmed, so `_N` is not
  asserted. Revisit once U41 latch polarity is fixed.
- `OCXO_PSU_PG` / `OCXO_LDO_PG`, `RB_PSU_PG`, `3V3_PSU_PG`, `5V_PSU_PG` — the
  `_PSU_`/`_LDO_` stage qualifier is **kept** because OCXO has two distinct PG nets;
  collapsing to `<rail>_PG` would merge them.
- `INA_ALERT_N` — a **1-pin dangling net** on the MCU (see error report); it is a defect,
  not a name to standardize. Should be merged into `INA_ALERT_INT_N`.
- Functional rails (`VCC_RB`, `V_ANT`, `V_POE`, `OCXO_V`, …) — kept by rule.
