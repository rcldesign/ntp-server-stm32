# STS1000 — Fault Aggregation (direct-GPIO scan)

All fault/status and human-interface signals land on **STM32 U12 (STM32H563ZIT6,
LQFP144) GPIO directly** — no port expander, no I²C round-trip, **no open-drain wire-OR
interrupt tree**. Fault state is gathered by a firmware **port-scan** (read whole GPIO
input registers at a fixed rate), not by EXTI aggregation. The LQFP144 package provides
the I/O headroom to land every rail, ALERT, and HMI line on its own pin.

Canonical firmware/register behavior lives in **`sts1000_firmware_hardware_interface.md`**;
electrical pin facts in `ntp_server_peripheral_map.md` §8/§9/§12/§13. This doc is the
map of *what* is aggregated and *where*.

---

## 1. Aggregation model

- **All PG (power-good) rails → `GPIOG[0:7]`** (one pin per rail, read as a byte).
- **All 9 INA228 ALERT lines → `GPIOG[8:15]` (8) + `PF13` (panel INA228, the 9th).**
- **HMI + RT9742 EN-fault + backup-PG inputs → `GPIOF`** (buttons, touch INT, encoder
  button, proximity wake, the three load-switch fault flags, backup-rail PGs).
- **No expander, no MIRROR/INTCON config, no shared interrupt nets.** Firmware reads
  `GPIOF`/`GPIOG` input data registers on a **~1 kHz port scan** and diffs against the
  previous snapshot.

Each INA228 ALERT remains individually meaningful (the scan identifies which rail), and
each ALERT is an open-drain output pulled to 3V3_STM at the MCU pin — but they land on
**separate GPIO pins**, so there is no shared wire-OR line and no missed-edge race.

---

## 2. Power-good rails — `GPIOG[0:7]`

| Pin (#) | Net | Source |
|---------|-----|--------|
| PG0 (56) | `3V3_GPS_LDO_PG` | U22 LT3045 PG (R215 pull-up) |
| PG1 (57) | `OCXO_LDO_PG`    | U39 LDO PG (R131) |
| PG2 (87) | `3V0_RF_LDO_PG`  | U51 LDO PG |
| PG3 (88) | `5V_PSU_PG`      | 5 V buck PG (U13/U28/U29 chain) |
| PG4 (89) | `3V3_PSU_PG`     | 3V3 buck PG (R92/R94) |
| PG5 (90) | `OCXO_PSU_PG`    | via R232/R233 divider off `OCXO_PSU_PG` (U39 EN node) — sense, **not** spare |
| PG6 (91) | `RB_PSU_PG`      | U40 MIC28516 PG |
| PG7 (92) | `POE_PG`         | U9 NCP1095 PGO (also gates U28 EN) |

> PG5 divides `OCXO_PSU_PG` (≈0.89×) into PG5 — confirm the source stays ≤3.3 V (PG5 is
> not 5 V-tolerant). Two additional backup-rail PGs land on GPIOF, not GPIOG: `BKP_STM_PG`
> (PF14) and `BKP_GPS_PG` (PF15), from the U67 comparator pair.

---

## 3. INA228 ALERT lines — `GPIOG[8:15]` + `PF13`

| Pin (#) | Net | INA228 | Addr | Rail |
|---------|-----|--------|------|------|
| PG8 (93)  | `INA_ALERT_V_POE`   | U10 | 0x40 | PoE input |
| PG9 (124) | `INA_ALERT_3V3_STM` | U31 | 0x41 | STM 3V3 |
| PG10 (125)| `INA_ALERT_5V_DISP` | U32 | 0x42 | 5V_DISP |
| PG11 (126)| `INA_ALERT_3V3`     | U30 | 0x43 | main/general 3V3 |
| PG12 (127)| `INA_ALERT_3V3_GPS` | U23 | **0x4A** | GPS VCC (0x44 = SHT45 U72) |
| PG13 (128)| `INA_ALERT_V_ANT`   | U26 | 0x45 | antenna bias |
| PG14 (129)| `INA_ALERT_OCXO`    | U37 | 0x46 | OCXO |
| PG15 (132)| `INA_ALERT_VCC_RB`  | U44 | 0x47 | Rb (VCC_RB) |
| PF13 (53) | `INA_ALERT_5V_PANEL`| U54 | 0x4C | panel-LED 5 V |

Each of the nine ALERTs pulls up 10 kΩ → 3V3_STM (INA228 ALERT is open-drain): R220–R226
on PG8–PG14, **R265 on `INA_ALERT_VCC_RB` (PG15)**, and R227 on the panel ALERT (PF13).

---

## 4. HMI / EN-fault / backup-PG inputs — `GPIOF`

| Pin (#) | Net | Source | Conditioning |
|---------|-----|--------|--------------|
| PF0–PF6 (10–18) | `BUTTON_1`…`BUTTON_7` | J17 keypad | R190–R196 10 kΩ→3V3_STM + C161–C167 0.1 µF; TVS U19/U20 |
| PF7 (19) | `DISP_TOUCH_INT` | FT-series touch INT (J17.33) | R208 10 kΩ→3V3_STM, no cap; TVS U17 |
| PF8 (20) | `V_ANT_EN_FAULT_N` | U27 RT9742 nFLG (antenna) | R200 10 kΩ→3V3_STM |
| PF9 (21) | `V_DISP_EN_FAULT_N` | U33 RT9742 nFLG (display 5 V) | R105 10 kΩ→3V3_STM |
| PF10 (22) | `PROX_WAKE` | magnetic reed switch (J17.18) | R254 10 kΩ→3V3_STM + C202; TVS U16; passive dry contact, active-low on magnet |
| PF11 (49) | `ENC_BUTTON` | encoder switch (J17.9) | R236 10 kΩ→3V3_STM + C187; TVS U69 |
| PF12 (50) | `PANEL_LED_FAULT_N` | U55 RT9742 nFLG (panel LED) | R201 10 kΩ→3V3_STM |
| PF13 (53) | `INA_ALERT_5V_PANEL` | U54 INA228 @0x4C | R227 pull-up (see §3) |
| PF14 (54) | `BKP_STM_PG` | U67 backup-PG comparator | R246 |
| PF15 (55) | `BKP_GPS_PG` | U67 backup-PG comparator | R241 |

All 16 GPIOF pins are used. Buttons/encoder are active-low (idle-high); the RT9742 nFLG
flags are open-drain active-low (`_N`); INA ALERT is open-drain active-low.

> **Not aggregated here (direct dedicated pins elsewhere):** `RB_OV_DET` (PE3, autonomous
> Rb OV latch), `PFI` power-fail (PE8), NCP1095 `POE_NCL/POE_NCM/POE_LCF` (PC7/PC2/PC3),
> `RB_LOCK` (PB13), `WDT_EN` (PC12). These are safety/latency paths that must not wait on a
> port scan.

---

## 5. Firmware — port-scan (see `sts1000_firmware_hardware_interface.md`)

```
scan_task (~1 kHz):
  g = read GPIOG->IDR                 # PG[0:7] rails, PG[8:15] INA ALERTs
  f = read GPIOF->IDR                 # buttons/touch/encoder/EN-fault/backup-PG
  pg_bad   = ~g[0:7]                  # PG low = rail not good (per-rail polarity: confirm)
  ina_alrt = ~g[8:15]; if f.PF13 low: ina_alrt |= panel
  dispatch each changed bit vs. previous snapshot
  for each asserted INA ALERT: read that INA228 diag/flags (V/I/P cause) over I²C1, clear latch
  buttons: debounce ~20–30 ms in firmware on top of the 1 ms RC
```

- **No read-acknowledge race:** ALERTs/PGs are levels on dedicated pins; a persistent fault
  simply keeps its bit asserted until cleared. A missed transition self-heals on the next
  ~1 ms scan.
- **Storm guard:** a stuck ALERT is handled by reading/clearing its INA228 over I²C; the
  bit clears when the underlying condition clears. No masking of a shared interrupt is
  needed (there is no shared interrupt).
- **Wake:** buttons/touch/encoder can additionally be armed as EXTI on their GPIOF pins for
  low-latency panel wake if desired; the fault scan itself does not depend on EXTI.

---

## 6. Reserved / absent nets & designators (do not introduce)

There is no shared-interrupt or expander topology in this design: nets `EXP_RST_N`,
`BTN_INT_N`, `INA_ALERT_INT_N`, `PG_INT_N`, `INA_ALERT_N` do not exist. **U54 = panel
INA228 (0x4C)** and **U55 = panel-LED RT9742 load switch** — neither is an I/O expander.

---

## 7. Open items

1. **PG polarity per rail** — confirm each PG is good=high/fault=low; set scan mask accordingly.
2. **PG5 source voltage** — confirm `OCXO_PSU_PG` (÷ R232/R233) stays ≤3.3 V at PG5.
3. **INA228 latched ALERT** enabled on all nine; scan clears via I²C diag read.
4. **Optional EXTI wake** on the GPIOF button/touch/encoder subset (latency vs. scan).
