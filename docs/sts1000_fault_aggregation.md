# STS1000 — Fault Aggregation & Local-UI Expanders (MCP23017 ×2)

**As-built** per schematic (U47 @0x20, U48 @0x21). Complete fault/UI coverage on two
MCP23017s, every signal individually readable on I²C, on **three active-low interrupts**:

| Net (`_N`, open-drain) | STM32 | Source | Carries |
|------------------------|-------|--------|---------|
| `BTN_INT_N`      | PE0/EXTI0  | U48 INTA | 8 buttons |
| `INA_ALERT_INT_N`| PC12/EXTI12| U48 INTB | 8 INA228 ALERTs |
| `PG_INT_N`       | PA10/EXTI10| U47 INTA+INTB | 8 PG + 2 EN-fault |

VDD = **3V3_STM** on both (matches the I²C master + INT pull-up rail). Cross-refs: peripheral
map §4/§6/§8/§9/§11/§12/§13; `sts1000_vcc_rb_supply.md`.

---

## 1. As-built wiring

### U47 @0x20 — PG + EN-fault → one interrupt (`PG_INT_N`, MIRROR=1)

| Bit | Net | | Bit | Net |
|-----|-----|-|-----|-----|
| GPA0 | `3V3_LDO_GPS_PG` | | GPB0 | `V_ANT_EN_FAULT` |
| GPA1 | `OCXO_LDO_PG`    | | GPB1 | `3.3V_P_EN_FAULT` |
| GPA2 | `3.0V_RF_LDO_PG` | | GPB2–7 | NC (6 spare) |
| GPA3 | `5V_PSU_PG`      | | | |
| GPA4 | `3V3_PSU_PG`     | | | |
| GPA5 | `OCXO_PSU_PG`    | | | |
| GPA6 | `RB_PSU_PG`      | | | |
| GPA7 | `POE_PGO`        | | | |

INTA+INTB tied → `PG_INT_N` (R152 10k → 3V3_STM). A0/A1/A2→GND. RESET←`EXP_RESET` (R154 10k pull-down). VDD 3V3_STM (C144 0.1µF).

### U48 @0x21 — buttons + INA → two interrupts (MIRROR=0)

| Bit | Net | | Bit | Net |
|-----|-----|-|-----|-----|
| GPA0 | `BUTTON_1` | | GPB0 | `INA_ALERT_V_POE` |
| GPA1 | `BUTTON_2` | | GPB1 | `INA_ALERT_3.3V` |
| GPA2 | `BUTTON_3` | | GPB2 | `INA_ALERT_3.3V_STM` |
| GPA3 | `BUTTON_4` | | GPB3 | `INA_ALERT_3.3V_P` |
| GPA4 | `BUTTON_5` | | GPB4 | `INA_ALERT_3.3V_GPS` |
| GPA5 | `BUTTON_6` | | GPB5 | `INA_ALERT_V_ANT` |
| GPA6 | `BUTTON_7` | | GPB6 | `INA_ALERT_OCXO` |
| GPA7 | `BUTTON_8` | | GPB7 | `INA_ALERT_VCC_RB` |

INTA→`BTN_INT_N` (R148 10k), INTB→`INA_ALERT_INT_N` (R153 10k), both →3V3_STM. A0→3V3_STM (A1/A2→GND ⇒ 0x21). RESET←`EXP_RESET`. VDD 3V3_STM (C145 0.1µF). 16/16 bits used.

> **Not on either expander:** `RB_OV` (PE3/PD3, autonomous latch), `PFI` (PE8), NCP1095
> `POE_NCL/NCM/LCF` (PC7/PC2/PC3), `RB_LOCK` (PB13). Direct MCU pins.

---

## 2. Interrupt topology

```
   U47 @0x20 (MIRROR=1, INT open-drain)
     PortA 8×PG ─┐
     PortB 2×EN_FAULT ─┘── INTA+INTB (tied) → PG_INT_N → R152 → PA10/EXTI10

   U48 @0x21 (MIRROR=0, INT open-drain)
     PortA 8×button ── INTA → BTN_INT_N        → R148 → PE0/EXTI0
     PortB 8×INA ──── INTB → INA_ALERT_INT_N   → R153 → PC12/EXTI12
```

All `_N` active-low, idle-high, asserted-low. One pull-up per net on the expander sheet
(R148/R152/R153). **PC12: remove the legacy R44** if still placed (R153 is now the pull-up).
Scale a domain past its port by wire-ORing another open-drain INT onto the same `_N` line.

---

## 3. STM32 pin map (verify the three nets land here on the MCU sheet)

| Pin (#) | Net | Source |
|---------|-----|--------|
| PE0 (97)  | `BTN_INT_N`       | U48 INTA |
| PC12 (80) | `INA_ALERT_INT_N` | U48 INTB (drop R44; keep R153) |
| PA10 (69) | `PG_INT_N`        | U47 INTA+INTB (R152 = the PA10 pull-up — earlier "add 10k" item is satisfied) |
| PC0 (15)  | `EXP_RESET`       | shared to both RESET pins (R154 pull-down → default-asserted) |

Board is GPIO-full (PA10 and PC0 both now used). All three INT lines + reset are on existing pins.

---

## 4. Register configuration (IOCON.BANK=0)

| Reg | U47 @0x20 (PG+EN) | U48 @0x21 (btn+INA) | Meaning |
|-----|-------------------|---------------------|---------|
| IODIR | 0xFF/0xFF | 0xFF/0xFF | inputs |
| IPOL | per-bit | PortA 0x00; PortB per-bit | invert active-low so asserted=1 (PG/INA active-low; confirm POE_PGO, EN-fault) |
| GPINTEN | 0xFF/**0x03** | 0xFF/0xFF | U47: 8 PG + 2 EN (GPB0/1); U48: 8 btn + 8 INA |
| DEFVAL | 0x00/0x00 | —/0x00 | idle reads 0 after IPOL |
| INTCON | 0xFF/0xFF | **0x00**/0xFF | U47 both level; U48 PortA on-change, PortB level |
| IOCON | **0x44** (MIRROR=1) | **0x04** (MIRROR=0) | ODR=1 (open-drain) both |
| GPPU | per source | per source | internal 100k OK for clean ALERT/PG; external where noisy |

U47 MIRROR=1 OR's both ports onto INTA/INTB (the external tie then parallels them — harmless).
U48 MIRROR=0 keeps buttons (INTA) and INA (INTB) on separate pins.

**Init (arm last):** IOCON → IODIR → IPOL → GPPU → DEFVAL → INTCON → dummy read GPIOA+GPIOB →
GPINTEN → enable MCU EXTI. Drive `EXP_RESET` (PC0) high first.

---

## 5. Read-acknowledge race — mitigation

1. Read INTF first; if zero, exit.
2. Faults are level (compare-to-DEFVAL) → persistent fault keeps its INT asserted; missed edge self-heals; transient captured in INTCAP.
3. Poll backstop (§7) reads GPIO regardless.
4. Storm guard: stuck fault → mask its GPINTEN bit, hand to poll, re-arm on clear. Buttons (on-change) can't storm.

On U48, read each port's INTCAP to clear only that port's INT (INTA vs INTB independent). On U47
(MIRROR=1) reading either port's INTCAP/GPIO clears the shared INT — read both GPIOA (PG) and
GPIOB (EN) each entry.

---

## 6. Firmware — three workers

```
EXTI(PE0,  BTN_INT_N):        button_worker
EXTI(PC12, INA_ALERT_INT_N):  ina_worker
EXTI(PA10, PG_INT_N):         pg_worker

button_worker:                 # U48 Port A
  do: fa=rd(0x21,INTFA); if fa==0: break
      ca=rd(0x21,INTCAPA); debounce_confirm(~ca & 0xFF)   # 8 buttons
  while pe0_low()

ina_worker:                    # U48 Port B
  do: fb=rd(0x21,INTFB); if fb==0: break
      cb=rd(0x21,INTCAPB)
      for bit in fb: read that INA228 diag/flags for V/I/P cause, clear its latch
      stuck=rd(0x21,GPIOB) & 0xFF; if stuck: mask GPINTENB bits, poll owns them
  while pc12_low()

pg_worker:                     # U47 Port A (PG) + Port B (EN-fault)
  do: fa=rd(0x20,INTFA); fb=rd(0x20,INTFB); if fa==0 and fb==0: break
      ca=rd(0x20,INTCAPA); cb=rd(0x20,INTCAPB)
      dispatch_pg(fa,ca)            # RB/5V first, then 3.3V rails, OCXO, RF, PoE
      dispatch_enfault(fb & 0x03,cb)  # V_ANT_EN_FAULT, 3.3V_P_EN_FAULT
      stuck=(rd(0x20,GPIOA)&0xFF) | ((rd(0x20,GPIOB)&0x03)<<8)
      if stuck: mask those GPINTEN bits
  while pa10_low()
```

EN-fault is now interrupt-driven (shares `PG_INT_N`), no longer polled. Antenna faults
cross-check `GPS_ANT_OFF_MON` (PD4) and the ZED-F9T antenna status.

---

## 7. Periodic poll backstop

```
poll_task (500 ms): for chip,ports in {0x20:(A,B), 0x21:(A,B)}:
  read GPIOA/GPIOB & fault masks; update state; re-arm cleared masked bits
  if i2c_error or implausible: recover(chip)
```

---

## 8. Power, /RESET, recovery

- **VDD = 3V3_STM** (both): MCU's own always-on rail → expanders share the I²C-master/INT-line domain and the fault monitor is alive whenever the MCU can read it. I²C bus pull-ups and R148/R152/R153 are on 3V3_STM. **Not** 3V3_P (gated; and a rail U48 monitors → circular).
- **/RESET = `EXP_RESET` (PC0)**, shared; R154 10k pull-down holds both chips in reset until firmware drives PC0 high.
- **Recovery:** I2C_BUF_EN (PA9) bus clear → assert `EXP_RESET` (PC0) for a POR-equivalent hard reset. No I²C soft-reset; no rail power-cycle (3V3_STM not gated).

> OV (Rb latch + PE3), WDT→POE_KILL, fan fail-safe act in hardware; PFI (PE8) covers fast brownout. Expander latency isn't a safety path.

---

## 9. Address & bus (peripheral map §4)

- U47 @0x20, U48 @0x21 (A0→VS); on the buffered segment (`I2C_SCL_BUF`/`I2C_SDA_BUF`).
- INA228 ×8 at 0x40–0x47; TMP117 0x48–0x4B; VL53L1X 0x29; ATECC608B 0x60 — all clear.
- ~16 nodes behind the LTC4311/PCA9517 — re-confirm 400 kHz Cb.

---

## 10. Open items

1. **Drop R44 at PC12** (or R153) — one pull-up per `INA_ALERT_INT_N` net; both = 5k.
2. **Net label** — Image 1 reads `PG_IN_N`; intended `PG_INT_N`. Fix in KiCad so it isn't a one-pin net.
3. **STM32-side connections** — confirm `BTN_INT_N`→PE0, `INA_ALERT_INT_N`→PC12, `PG_INT_N`→PA10, `EXP_RESET`→PC0 on the MCU sheet.
4. **Polarity** — confirm `POE_PGO`, `V_ANT_EN_FAULT`, `3.3V_P_EN_FAULT` (and PGs good=high/fault=low); set per-bit IPOL so asserted=1.
5. **INA228 latched ALERT** on all eight.
6. **U47 MIRROR=1** in firmware (matches the external INTA/INTB tie); U48 MIRROR=0.
7. **Masks** — U47 fault mask 0x03FF (PG 0x00FF + EN 0x0300); U48 button 0x00FF, INA 0xFF00.
