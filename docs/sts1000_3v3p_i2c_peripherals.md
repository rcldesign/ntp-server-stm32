# STS1000 "Meridian" — I²C1 Peripherals & UI Sensors

**Subsystem design record.** Covers the I²C1 peripheral group, its display level-shifter, and the
two UI sensors: the **proximity-wake sensor** and the **e-compass**.

> **Rail topology:** *all* I²C1 peripherals sit on the **always-on `3V3_STM`** rail as one flat
> segment; there is no gated peripheral rail. All aggregated digital inputs are **direct STM32
> GPIO** (no I/O expanders). This is deliberate: gating a peripheral rail while the I²C master and
> pull-ups sit on an always-on rail back-powers the "off" parts through their SDA/SCL ESD clamps
> (§2), so the clean topology is a single always-on segment.

**Canonical bus/pin map:** `ntp_server_peripheral_map.md` §4 (I²C1) and §6 (power domains) are
authoritative for the global address table and rail assignments. This document records the
*decisions and rationale*.

**Mounting context (drives the e-compass choice):** the main PCB / control panel is mounted **vertically** in the enclosure. Any heading function must therefore be tilt-compensated at an arbitrary orientation — see §4.

---

## 0. Summary of decisions (conclusions first)

| # | Decision | Rationale |
|---|----------|-----------|
| D1 | Proximity-wake sensor = **passive magnetic reed switch** (dry SPST contact), panel/housing-mounted, wired back to the PCB | A magnet on the access door/panel actuates it; no I²C proximity part is panel-mount/THT, and a dry contact needs no Vcc |
| D2 | Proximity-wake interface = **discrete digital input on STM32 `PROX_WAKE` = PF10** (J17.18) | Off the I²C bus entirely; direct GPIO, read on the ~1 kHz GPIOF scan (+ optional EXTI wake) |
| D3 | E-compass = **IIS2MDC (mag) + LIS2DH12 (accel)**, 2-chip | The single-chip 6-axis e-compass category is dead (LSM303AGR and FXOS8700CQ both EOL); a mag+accel pair covers the tilt-comp need |
| D4 | Magnetometer = **IIS2MDC** (industrial grade) | Longevity + industrial qual for a fixed-install instrument |
| D5 | Accelerometer = **LIS2DH12** | Availability; lands in the LIS2DH/LIS3DH register family, keeping the accel firmware path simple |
| D6 | **Continuous, current-free ground pour under the magnetometer**; control *current*, not copper | Copper is magnetically silent; only currents and ferromagnets produce field, and a plane void can route return current under the sensor |
| D7 | **One flat I²C1 segment, every device on always-on 3V3_STM** (no gated peripheral rail) | Gating a rail while the bus stays live back-powers the "off" devices through their SDA/SCL ESD clamps; a single always-on segment needs no isolating buffer |

---

## 1. I²C1 device inventory (15 devices, one flat segment on 3V3_STM)

Master = STM32 U12 on **PB8 (SCL) / PB9 (SDA)**. Pull-ups **R202 (SCL) / R203 (SDA) = 4.7 kΩ →
3V3_STM** (the only pull-ups on the bus). Rise-time accelerator **LTC4311 U56** with **ENABLE (pin 3)
strapped to 3V3_STM** (permanently on). Every device below is powered from **always-on 3V3_STM** —
there is no gated segment.

| Addr (7-bit) | Device | Ref | Strap / note |
|---|---|---|---|
| 0x19 | LIS2DH12 (e-compass accel) | U59 | SA0 → 3V3_STM (**committed 0x19**); CS → Vdd_IO for I²C |
| 0x1E | IIS2MDC (e-compass mag) | U61 | fixed address; CS → Vdd_IO for I²C |
| 0x40 | INA228 — PoE input | U10 | A1/A0 = GND |
| 0x41 | INA228 — STM 3V3 | U31 | |
| 0x42 | INA228 — 5V_DISP | U32 | |
| 0x43 | INA228 — main/general 3V3 | U30 | |
| **0x44** | **SHT45 (humidity)** | **U72** | fixed (SHT45-AD1B); **owns 0x44** |
| 0x45 | INA228 — antenna bias (V_ANT) | U26 | |
| 0x46 | INA228 — OCXO | U37 | |
| 0x47 | INA228 — Rb (VCC_RB) | U44 | |
| 0x48 | TMP117 (osc temp) | U58 | ADD0 → GND |
| 0x49 | TMP117 (ambient) | U57 | ADD0 → 3V3_STM |
| **0x4A** | INA228 — GPS VCC | U23 | A1 = A0 = SDA → 0x4A (SHT45 U72 owns 0x44) |
| 0x4C | INA228 — panel-LED 5 V | U54 | |
| 0x60 | ATECC608B (secure element) | U60 | address in config zone |

**No collisions.** 9× INA228 + 2× TMP117 + SHT45 + LIS2DH12 + IIS2MDC + ATECC608B = 15 devices.
The proximity-wake sensor is off the bus entirely (D1/D2 → PF10).

> **Address notes:** GPS INA228 **U23 = 0x4A** because **SHT45 U72 owns 0x44** on this bus (do not
> re-strap U23 to 0x44 — head-on collision). TMP117 **U58 = 0x48 / U57 = 0x49**; LIS2DH12
> **U59 = 0x19** (SA0 strapped high).

---

## 2. Bus topology — one flat always-on segment (D7)

Every I²C1 device is on always-on **3V3_STM**, one flat segment, no gating — so nothing is ever
back-powered. The hazard this avoids: gating a peripheral rail while the I²C **master and pull-ups
sit on always-on 3V3_STM** back-powers the "off" parts — their SDA/SCL pins stay held at 3.3 V by
the 3V3_STM pull-ups, so current flows into those pins through the devices' bus-pin ESD clamp diodes,
back-powering the dead devices and loading the live bus. A single always-on segment removes the
hazard with no isolating buffer.

> An **LTC4311 only accelerates edges — it does NOT isolate.** **U56 (LTC4311)** is present purely
> as a rise-time accelerator with **ENABLE strapped to 3V3_STM** (always on); it is not solving
> back-powering (there is none to solve).

### 2.1 Display-side I²C level-shift (PCA9306 U63)

The display/touch module runs I²C at **5 V**; the MCU I²C1 domain is 3.3 V. **PCA9306 U63** bridges
them: VREF1 (pin 2) = **3V3_STM** + main `I2C_SCL/I2C_SDA` (SCL1/SDA1); VREF2 (pin 7) = **5V_DISP** +
the display-side `I2C_DISP_SCL/I2C_DISP_SDA` (SCL2/SDA2); **EN (pin 8) = `DISP_EN` (PC11)** so the
display segment level-shifter is enabled with the display rail. The 3.3 V main side is pulled up by
R202/R203 (§1).

> **5 V-side pull-ups:** the PCA9306 is a passive pass-gate and provides no pull-ups, so the
> **display module supplies its own 5 V I²C pull-ups** on `I2C_DISP_SCL/SDA` (J17.32/J17.15). The
> display module is powered from **J17.16 = 5V_DISP** (so INA228 U54/U32 also see the display load)
> and the panel logic from **J17.28 = 5 V**.

---

## 3. Proximity-wake sensor — passive magnetic reed switch (D1, D2)

### Why a reed switch

The wake sensor must mount on the **housing / control panel** with flying leads back to the PCB. **No catalog I²C proximity sensor is through-hole or panel-mount** — every I²C proximity part (VL53xx, VCNL40xx, APDS-9960) is a tiny optical SMD module meant to sit behind a window on the main PCB. A panel/THT mount rules out I²C; the part is a **passive magnetic reed switch** — a dry SPST contact actuated by a magnet on the access door/panel. It needs no supply, no window, and no warm-up, and conditions exactly like the panel buttons.

### Selected part

**Passive magnetic reed switch (dry SPST contact).** Two-terminal, unpowered: a magnet brought near the contact closes it. Panel/housing-mountable with flying leads; immune to optical/thermal false triggers and to the EMI environment near the GNSS/RF section. Actuation = door/panel magnet present.

### Interface — `PROX_WAKE` = PF10, J17.18

- **Read on STM32 `PROX_WAKE` = PF10** (direct GPIO, TVS U16), gathered by the ~1 kHz GPIOF scan and
  optionally armed as EXTI for wake. Conditioning: **R254 10 kΩ → 3V3_STM + C202 0.1 µF** — the same
  active-low chain as the panel buttons.
- **Dry contact, active-low.** Idle (contact open) the 10 kΩ holds PF10 high; magnet present closes
  the reed to GND → low. A passive contact draws no current. The topology is **fail-safe**: an
  unplugged or cut lead reads idle-high (no false wake); a closed reed reads low (wake).
- **No Vcc pin needed.** The reed is passive — `PROX_WAKE` + GND on **J17.18** (plus a J17 GND) is the
  complete connection.
- **Debounce:** R254·C202 = 10 kΩ × 0.1 µF ≈ 1 ms RC + firmware debounce, as for the buttons. Reed
  contacts bounce on close; the RC + FW debounce covers it.
- **Connector / wiring:** a couple of feet of wire to the panel is fine. TVS at the connector (U16) as
  for the button array.

### Application note

- **Magnet-actuated, not motion/approach.** Wake fires when the door/panel magnet is presented (lid
  opened, or a keeper magnet moved), not on a warm body. Firmware treats a close edge as a wake event;
  apply a wake-hold timer to keep the panel lit N s after the last edge. **No warm-up mask is
  required** — a passive contact is valid immediately at boot.

---

## 4. E-compass — IIS2MDC + LIS2DH12 (D3, D4, D5)

### Why a pair, and why these parts

- **Vertical mount forces tilt compensation.** A magnetometer reads all three field components at any orientation, but converting to heading needs a gravity reference (3-axis accel). A mag-only replacement cannot produce a heading on a vertical board. So the replacement must provide **mag + accel**.
- **Single-chip 6-axis e-compass is dead.** LSM303AGR is NLA; **FXOS8700CQ is EOL** (NXP lists no drop-in 6-axis successor). The realistic choices are a fusion IMU or a mag+accel pair.
- **Pair chosen over fusion IMU.** The unit is bolted in place and static, so a fusion IMU's gyro buys nothing; a static mag+accel tilt-comp is sufficient and lower cost/power, and stays in the ST tooling (X-CUBE-MEMS1, MotionEC e-compass, MotionMC calibration). Fusion-IMU fallbacks if ever wanted: BNO086 (0x4A/0x4B, onboard fusion) or BNO055 (0x28/0x29, aging/power-hungry).

### Parts

| Function | Part | Ref | Addr (7-bit) | Key facts |
|----------|------|-----|--------------|-----------|
| Magnetometer | **IIS2MDC** (TR) | **U61** | **0x1E, fixed** (no select pin) | Industrial-grade sibling of LIS2MDL, **register-compatible**; ±50 G; I²C to 3.4 MHz + SPI; LGA; −40/+85 °C. Same register family as the LSM303AGR mag. |
| Accelerometer | **LIS2DH12** (TR) | **U59** | **0x19 (SA0 → 3V3_STM)** | ±2/±4/±8/±16 g; 12-bit HR; **LIS2DH/LIS3DH register family** (NOT the LIS2DW12 "femto" map); `WHO_AM_I` = **0x33**; −40/+85 °C. |

- **0x1E is fixed** for the IIS2MDC (no select pin); the **0x18/0x19 strap belongs to the accelerometer, not the magnetometer.**
- **Accel address = 0x19** — LIS2DH12 U59 SA0 strapped to 3V3_STM; free on the bus.
- **Firmware:** LIS2DH12 sits in the LIS2DH register family, keeping the accel firmware path simple. On Zephyr, the `st,lis2dh` driver covers LIS2DH12; the magnetometer uses the `st,lis2mdl`-class driver given IIS2MDC register identity (confirm the devicetree `compatible`/alias).

### Configuration & assembly notes

- **CS → Vdd_IO on both parts** to select I²C (each supports SPI on the same pads).
- **Axis alignment is now firmware's job.** Two separate packages replace the factory-aligned LSM303AGR; place them with a known fixed relative orientation (ideally same orientation, close together) and bake the rotation/axis-mapping into the tilt-comp math before computing heading.
- **Tilt-comp math** handles the vertical mount: accel → roll/pitch from gravity (gravity lies in the board plane when vertical), de-rotate the mag vector into the local-level frame, `heading = atan2(...)`. Use ST MotionEC (e-compass) + MotionMC (mag cal) or roll your own (ST design tip DT0131 / NXP AN4248).
- **Resolution is adequate.** LIS2DH12 is lower-res/noisier than LIS2DW12, but only a static 1 g gravity vector is being resolved — oversample/average; non-issue for a fixed install.
- **Grade is mixed** (industrial mag, consumer accel). LIS2DH12 still operates −40/+85 °C, so temperature is covered; it just lacks the IIS longevity guarantee. Acceptable given availability — **verify worst-case local temp at the sensor location** stays in range.
- **Decoupling:** 100 nF on Vdd and Vdd_IO of each part; the IIS2MDC datasheet also calls for a 10 µF bulk near the supply pin.

---

## 5. Magnetometer layout & keep-out (D6)

### Principle: control current, not copper

Copper is diamagnetic (χ ≈ −10⁻⁵) — **magnetically invisible to a DC magnetometer.** The IIS2MDC senses field from (a) **currents** (Ampère) and (b) **ferromagnetic materials**. A ground pour carrying **no net current** produces no field and is magnetically silent.

**Do NOT void the plane under the part.** A bare-FR4 cutout buys nothing magnetically and:
- removes the SCL/SDA/CS return/reference plane (SI + EMI hit), and
- can force a return current to **detour around the void**, possibly into a loop directly under the sensor — the one geometry to avoid.

**Keep a continuous, current-free ground pour under the sensor** and steer current elsewhere.

### The field math that should size the keep-out

B = (μ₀/2π)·(I/r) = 2×10⁻⁷ · I/r.
Reference: San Diego horizontal field ≈ 20 µT (200 mgauss); ~1° heading ≈ **3.5 mgauss** of error budget.

| Current | Distance | Field | ≈ heading error if uncorrected |
|---------|----------|-------|--------------------------------|
| 10 mA | 5 mm | 4 mgauss | ~1° |
| 100 mA | 5 mm | 40 mgauss | ~11° |
| 100 mA | 20 mm | 10 mgauss | ~3° |
| 1 A | 5 mm | 400 mgauss | swamps Earth field |

Takeaways: a single 100 mA trace at 5 mm already costs ~11°, so **5 mm of plain distance is weak**. Routing a current's **go + return as a tight pair** (or stacked on adjacent layers) collapses the field to a dipole that falls as **1/r² instead of 1/r** — **balancing beats distance.**

### Static vs. dynamic (what calibration can and can't fix)

A one-time, in-enclosure **hard/soft-iron calibration** (stored once, fixed install) removes **constant** offsets — nearby permanently-magnetized metal and steady DC currents just become a calibrated offset. It **cannot** remove **time-varying** field: buck switching, OCXO oven-heater PWM, fan motor, **G6K relay actuation**, and VCC_RB load swings. Layout priority is therefore keeping **switching/variable** loops away and quiet, plus distance from dynamic sources.

### Concrete layout rules

1. **Continuous ground pour under the part**, with no net/return current crossing the region — no plane split or neck that diverts current under the footprint.
2. **Refine the trace rule:** the sensor's own SCL/SDA/CS/supply must reach the pads — route them short/direct (their currents are tiny). Apply the real keep-out to **power/high-current** traces; route them out of the area and as **balanced go+return pairs**. For hundreds of mA, 5 mm is not enough — balance + more distance.
3. **Non-ferromagnetic finish/parts in the keep-out:** ENIG's nickel underlayer is mildly ferromagnetic — prefer immersion silver/OSP locally, or keep ENIG out of the immediate keep-out. Keep ferrite beads, inductor cores, magnetic-core MLCCs, steel standoffs/hardware away.
4. **Distance from the big offenders:** G6K relay coil, bucks, PoE magnetics, Rb physics package, OCXO oven, fan motor — several are dynamic, so distance (not calibration) is the lever.

---

## 6. I²C address map — canonical assignments for peripheral-map §4/§6

The full I²C1 map is the table in **§1**. The canonical assignments peripheral-map §4/§6 must carry:

- **E-compass:** `IIS2MDC 0x1E` mag (U61); `LIS2DH12 0x19` accel (U59, SA0→3V3_STM); CS→Vdd_IO for I²C.
- **Humidity:** `SHT45 0x44` (U72).
- **Current monitors:** GPS INA228 = **0x4A** (U23); TMP117 **U58 = 0x48 / U57 = 0x49**; panel INA228
  `U54 = 0x4C`.
- **Proximity wake** is a discrete GPIO on PF10 (D1/D2), not an I²C address.
- **Power domain:** every I²C1 device (the §1 table), plus the SPI-NOR (U62) and display/touch, is on
  **always-on 3V3_STM**. No gated peripheral rail, no segmenting buffer; U56 (LTC4311) is an
  always-on accelerator only. Fusion-IMU alternatives (BNO055 0x28 etc.) apply only if the e-compass
  pair is ever replaced.

---

## 7. Firmware notes

- **Proximity wake:** read `PROX_WAKE` (PF10) on the GPIOF scan; optionally arm EXTI on PF10 for
  on-change wake. Apply a wake-hold timer on the close edge. The sensor is a **passive magnetic reed
  switch** (active-low, R254 pull-up → 3V3_STM), so **no warm-up mask is needed** and the level is
  valid immediately (§3).
- **E-compass:** `st,lis2dh` driver for LIS2DH12; `st,lis2mdl`-class for IIS2MDC (verify compatible string). Apply the package-to-board rotation/axis map first, then tilt-comp (MotionEC) and stored hard/soft-iron cal (MotionMC). Apply WMM/IGRF declination from the GPS fix for **true** north. Heading is for orienting the satellite plot — low rate is fine; oversample the accel to beat LIS2DH12 noise.
- **Calibration is one-shot for the fixed install** — run in-enclosure, store, reuse. Re-run only if the unit is re-housed or nearby ferromagnetics change.

---

## 8. Open items / to verify

- [ ] **Display 5V-side I²C pull-ups (U63)**: the PCA9306 provides none — confirm the display module
      carries its own 2× ~4.7 kΩ → 5V_DISP on `I2C_DISP_SCL/SDA` (§2.1).
- [ ] **Zephyr bindings**: confirm `st,lis2dh` (LIS2DH12 U59 @0x19) and the IIS2MDC (U61 @0x1E)
      magnetometer `compatible`/alias; firmware I²C map must adopt GPS INA=0x4A, SHT45=0x44,
      TMP117 U58=0x48 / U57=0x49.
- [ ] **Local temperature** at the e-compass location vs LIS2DH12 −40/+85 °C (consumer grade).
- [ ] **Magnetometer keep-out** captured as layout rules: current-loop exclusion vs trace current,
      balanced power returns, non-ferromagnetic finish/parts zone, distances to relay/bucks/magnetics/Rb/OCXO/fan.

---

## 9. Cross-references

`ntp_server_peripheral_map.md` (§4 I²C, §6 power domains), `ina228_decode` map,
`sts1000_fault_aggregation.md` (direct-GPIO fault/HMI scan), `sts1000_firmware_hardware_interface.md`,
`sts1000_vcc_rb_supply.md` (VCC_RB dynamics relevant to mag interference),
`sts1000_schematic_design_review.md` (netlist-driven consistency review).
