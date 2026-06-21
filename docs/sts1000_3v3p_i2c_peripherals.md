# STS1000 "Meridian" — 3V3_P I²C Peripherals & UI Sensors

**Subsystem design record.** Covers the I²C peripheral group on the gated peripheral rail (3V3_P), the bus-topology consequences of gating that rail, and the two UI sensors that were re-specified in this design cycle: the **proximity-wake sensor** and the **e-compass**.

**Canonical bus/pin map:** `ntp_server_peripheral_map.md` §4 (I²C1) and §6 (power domains) remain authoritative for the global address table and rail assignments. This document records the *decisions and rationale* behind the changes; propagate the deltas in §6 below into §4/§6 of the peripheral map.

**Mounting context (drives the e-compass choice):** the main PCB / control panel is mounted **vertically** in the enclosure. Any heading function must therefore be tilt-compensated at an arbitrary orientation — see §4.

---

## 0. Summary of decisions (conclusions first)

| # | Decision | Was | Now | Driver |
|---|----------|-----|-----|--------|
| D1 | Proximity-wake sensor | VL53L1X (I²C ToF, SMD) | **Panasonic PaPIRs PIR** (EKMB 6 µA digital), panel/housing-mounted, wired back to PCB | Needs panel/through-hole part on the housing; no THT/panel-mount I²C proximity part exists |
| D2 | Proximity-wake interface | I²C @0x29, polled 2–4 Hz | **Discrete digital input on a spare MCP23017 U55 Port B bit** (GPB2–7), interrupt via `PG_INT_N` | Leaves I²C entirely; zero MCU pins; gets a real wake interrupt |
| D3 | E-compass | LSM303AGR (NLA) | **IIS2MDC (mag) + LIS2DH12 (accel)**, 2-chip | LSM303AGR and FXOS8700CQ both EOL; single-chip 6-axis e-compass category is dead |
| D4 | Magnetometer grade | (consumer LIS2MDL considered) | **IIS2MDC** (industrial) | Longevity + industrial qual for a fixed-install instrument |
| D5 | Accelerometer part | (LIS2DW12 considered) | **LIS2DH12** | Availability; also lands in the LIS2DH register family that the LSM303AGR accel already used |
| D6 | Magnetometer copper treatment | (proposed: void plane under part, 5 mm trace keep-out) | **Keep a continuous, current-free ground pour under the part**; control *current*, not copper | Copper is magnetically silent; only currents and ferromagnets produce field; a void breaks the return path and can route current under the sensor |
| D7 | 3V3_P I²C bus topology | flat segment, pull-ups on 3V3_STM | **Buffer-isolate the gated devices** (PCA9517/TCA4311) with far-side pull-ups on 3V3_P **or** treat PERIPH_EN as effectively always-on | Gating 3V3_P while the bus stays live back-powers the dead devices through their SDA/SCL ESD clamps |

---

## 1. 3V3_P I²C device inventory

Devices powered from the **gated peripheral/housekeeping rail** (`PERIPH_EN` = PC11), excluding the two GPIO expanders and the eight INA228s (those are on always-on **3V3_STM**, by design).

| Device | Addr (7-bit) | Rail status | Strap / mode | Analog beyond bypass |
|--------|--------------|-------------|--------------|----------------------|
| TMP117 #1 (osc temp) | 0x48 | **Confirmed** on PERIPH_EN | ADD0 → GND | none |
| TMP117 #2 (ambient) | 0x49 | **Confirmed** on PERIPH_EN | ADD0 → V+ | none |
| ATECC608B (secure element) | 0x60 | **Confirmed** on PERIPH_EN | I²C variant; addr in config zone | none |
| IIS2MDC (e-compass mag) | 0x1E (fixed) | UI peripheral — **confirm rail in §6** | CS → Vdd_IO for I²C | mag is sensitive analog; keep-out (§5) |
| LIS2DH12 (e-compass accel) | 0x18 or 0x19 (SA0) | UI peripheral — **confirm rail in §6** | CS → Vdd_IO; SA0 strap | none |

> **Open assumption:** `ntp_server_peripheral_map.md` §6 currently lists `PERIPH_EN` as feeding only **TMP117 / ATECC608B / NOR**. The e-compass parts (and previously the VL53L1X) are treated as 3V3_P UI peripherals throughout this work but are **not explicitly enumerated on that rail in §6**. Confirm and add them, or assign them deliberately to a different rail. NOR flash and the ST7796 TFT are also on PERIPH_EN but are SPI, hence out of scope here.

The proximity-wake sensor (formerly VL53L1X @0x29) **leaves the I²C bus entirely** under D1/D2.

---

## 2. Bus topology — 3V3_P gating & back-powering (D7)

`PERIPH_EN` (PC11) gates 3V3_P, while the I²C **master, pull-ups, and both MCP23017 expanders sit on always-on 3V3_STM**. When PERIPH_EN drops:

- The 3V3_P I²C devices lose VDD, but their SDA/SCL pins are still held at 3.3 V by the 3V3_STM pull-ups.
- Current flows into those pins through the devices' bus-pin ESD clamp diodes → **back-powers the "off" parts and loads the live bus**.

### Resolution options

1. **Buffer-isolated segment (preferred).** Put the 3V3_P devices behind a **PCA9517 / TCA4311** segmenting buffer, with the **far-side pull-ups on 3V3_P**. Gating PERIPH_EN then drops the whole segment cleanly and isolates it from the master side. (`I2C_BUF_EN` = PA9 already exists as an open item in peripheral-map §13.)
2. **Flat segment + treat PERIPH_EN as always-on.** Keep one flat bus, add series protection on SDA/SCL, and never actually gate 3V3_P in normal operation.

> An **LTC4311 only accelerates edges — it does NOT isolate**, so it does not solve back-powering. If segmenting is required for back-powering, the part must be a real buffer (PCA9517/TCA4311), not the LTC4311 accelerator. Resolve the LTC4311-vs-buffer choice (peripheral-map §13) with this constraint in mind.

---

## 3. Proximity-wake sensor — VL53L1X → PaPIRs PIR (D1, D2)

### Why the change

The wake sensor must mount on the **housing / control panel** with flying leads back to the PCB. **No catalog I²C proximity sensor is through-hole or panel-mount** — every I²C proximity part (VL53xx, VCNL40xx, APDS-9960) is a tiny optical SMD module meant to sit behind a window on the main PCB. Requiring panel/THT mount therefore drops I²C and moves to a discrete-output sensor.

### Technology trade (panel/THT, single sensor, "wake on approach")

| Tech | Representative part | Mount | Supply | Output | Range | Verdict |
|------|--------------------|-------|--------|--------|-------|---------|
| **Pyroelectric PIR** | Panasonic PaPIRs **EKMB** (6 µA) / EKMC | TO-5 leaded can; lens at panel aperture | ~1.8–6 V | digital open-drain on/off | meters | **Selected** |
| Capacitive proximity (industrial barrel) | M12/M18 threaded | true panel-mount + lead | usually 10–30 V | NPN/PNP OC | mm–few cm | range too short; wrong supply |
| Diffuse photoelectric | bracket/panel sensor | bracket | 10–30 V | OC discrete | OK | false-triggers in enclosure; wrong supply |
| Analog IR distance | Sharp GP2Y0A | module + leads | 5 V | analog (needs ADC) | OK | no spare ADC; obsolescence |

### Selected part

**Panasonic PaPIRs, digital type — recommend EKMB 6 µA high-noise-immunity grade.** TO-5 metal-can leaded (through-hole), integrated amp/comparator/ASIC, fixed preset threshold, open-drain on/off output, temp/offset compensation, high EMI immunity (6 µA grade has a differential design with noise resistance into the GHz range — relevant near GNSS/RF). Standard or wide lens. Runs directly off 3V3_P. Choose the final lens/finish for the aperture geometry.

### Interface (the parts that bite)

- **Output drive is tiny: ≤ 100 µA.** Exceeding it makes the part unstable or damages it. Pull-up sized for ≤100 µA → **R ≥ 33 kΩ; use 100 kΩ.** Optional 1–10 nF for edge cleanup. **Do not** hang an LED or low-value pull-up on the output.
- **Pull-up rail = 3V3_STM** (so the expander input is always defined even when 3V3_P is gated; PIR off → output transistor off → line reads high = "no motion", safe default). Back-feed through a ≥100 kΩ pull-up is negligible.
- **Confirm output structure per orderable PN** — EU lit says open-drain, NA lit describes a limited-drive voltage output. Same 100 µA ceiling either way; verify whether the pull-up is mandatory.
- **Read via spare MCP23017 U55 Port B bit (GPB2–7).** U55 is MIRROR=1 with INTA+INTB tied → `PG_INT_N` → PA10/EXTI10. The wake input therefore raises an interrupt with **zero new MCU pins**. Set the bit's IPOL/INTCON for on-change wake.
- **Connector / wiring:** 3 conductors (V+, GND, OUT). Keep OUT high-impedance; a couple of feet of wire to the panel is fine at these currents. Add connector-pin TVS on the main PCB (as for the button array).

### Application caveats

- **Motion ≠ presence.** PIR fires on movement; "approach" wakes it, but it won't hold while a person stands still. Handle in firmware with a wake-hold timer (stay lit N s after last edge) — standard PIR display-wake pattern.
- **Warm-up vs. rail gating.** PIR needs ~30 s to stabilize after power-up; output is unreliable until then. If 3V3_P is gated by PERIPH_EN, each cycle restarts that settle. **Option:** put just this sensor on 3V3_STM (a few µA) so wake is instantly reliable; otherwise firmware must mask the sensor ~30 s after any PERIPH_EN cycle.
- **Optical path.** Lens must see out an aperture — will not work behind glass or most plastics (IR-opaque). Provide a clear/IR-transmissive window or open slot. **Aim it out the front, away from the fan exhaust, OCXO oven, and Rb package** — moving warm air is the classic PIR false-trigger source in an instrument enclosure.

---

## 4. E-compass — LSM303AGR → IIS2MDC + LIS2DH12 (D3, D4, D5)

### Why a pair, and why these parts

- **Vertical mount forces tilt compensation.** A magnetometer reads all three field components at any orientation, but converting to heading needs a gravity reference (3-axis accel). A mag-only replacement cannot produce a heading on a vertical board. So the replacement must provide **mag + accel**.
- **Single-chip 6-axis e-compass is dead.** LSM303AGR is NLA; **FXOS8700CQ is EOL** (NXP lists no drop-in 6-axis successor). The realistic choices are a fusion IMU or a mag+accel pair.
- **Pair chosen over fusion IMU.** The unit is bolted in place and static, so a fusion IMU's gyro buys nothing; a static mag+accel tilt-comp is sufficient and lower cost/power, and stays in the ST tooling (X-CUBE-MEMS1, MotionEC e-compass, MotionMC calibration). Fusion-IMU fallbacks if ever wanted: BNO086 (0x4A/0x4B, onboard fusion) or BNO055 (0x28/0x29, aging/power-hungry).

### Parts

| Function | Part | Addr (7-bit) | Key facts |
|----------|------|--------------|-----------|
| Magnetometer | **IIS2MDC** (TR) | **0x1E, fixed** (no select pin) | Industrial-grade sibling of LIS2MDL, **register-compatible**; ±50 G; I²C to 3.4 MHz + SPI; LGA; −40/+85 °C. Same register family as the LSM303AGR mag. |
| Accelerometer | **LIS2DH12** (TR) | **0x18 (SA0=0) / 0x19 (SA0=1)** | ±2/±4/±8/±16 g; 12-bit HR; **LIS2DH/LIS3DH register family** (NOT the LIS2DW12 "femto" map); `WHO_AM_I` = **0x33**; −40/+85 °C. |

- **0x1E is fixed** for the IIS2MDC — confirms that the **0x18/0x19 strap belongs to the accelerometer, not the magnetometer.** (Recorded because it was a point of confusion.)
- Pick the accel address: **0x19 (SA0 → Vdd_IO)** reuses the LSM303AGR accel's old slot, or 0x18 (SA0 → GND). Both free on the bus. **Strap SA0 — never float it.**
- **Firmware bonus:** the LSM303AGR's accelerometer was register-compatible with the LIS2DH family, so LIS2DH12 keeps the accel firmware path close to the original design. On Zephyr, the `st,lis2dh` driver covers LIS2DH12; the magnetometer uses the `st,lis2mdl`-class driver given IIS2MDC register identity (confirm the devicetree `compatible`/alias).

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

## 6. I²C address map — delta to propagate into peripheral-map §4/§6

**Remove:**
- `VL53L1X 0x29` (proximity moves off I²C — D1/D2).
- `LSM303AGR 0x1E / 0x19` (e-compass replaced — D3).
- The `BNO055 0x28` alt note (unless retained as a fusion-IMU fallback).

**Add:**
- `IIS2MDC 0x1E (mag)` — fixed address; on 3V3_P; CS→Vdd_IO.
- `LIS2DH12 0x19 (accel, SA0→Vdd_IO)` — or 0x18; on 3V3_P; CS→Vdd_IO.

**Net:** the 0x1E slot is unchanged; 0x18/0x19 is a new accel slot (no collision with the existing map). The 3V3_P I²C UI group becomes **IIS2MDC + LIS2DH12** only; the proximity wake is a **discrete input on U55 GPB2–7**.

**§6 power-domain note:** add the e-compass pair (and decide the PIR's rail per §3) to the `PERIPH_EN` enumeration, or assign deliberately. If the buffered-segment topology (§2/D7) is adopted, document the far-side pull-ups on 3V3_P and the buffer part.

---

## 7. Firmware notes

- **Proximity wake:** read the U55 Port B bit; configure on-change interrupt (IPOL/INTCON/GPINTEN) so motion raises `PG_INT_N`. Apply a wake-hold timer. If the PIR is on 3V3_P, mask its input for ~30 s after any PERIPH_EN cycle (warm-up).
- **E-compass:** `st,lis2dh` driver for LIS2DH12; `st,lis2mdl`-class for IIS2MDC (verify compatible string). Apply the package-to-board rotation/axis map first, then tilt-comp (MotionEC) and stored hard/soft-iron cal (MotionMC). Apply WMM/IGRF declination from the GPS fix for **true** north. Heading is for orienting the satellite plot — low rate is fine; oversample the accel to beat LIS2DH12 noise.
- **Calibration is one-shot for the fixed install** — run in-enclosure, store, reuse. Re-run only if the unit is re-housed or nearby ferromagnetics change.

---

## 8. Open items / to verify

- [ ] **PIR orderable PN**: confirm digital output structure (open-drain vs limited-drive push-pull) and the ≤100 µA ceiling; finalize lens/finish for the panel aperture.
- [ ] **PIR rail**: 3V3_P (gated, needs warm-up mask) vs 3V3_STM (always ready). Decide and document.
- [ ] **3V3_P bus topology (D7)**: choose buffer-isolated segment (PCA9517/TCA4311, far-side pull-ups on 3V3_P) vs flat-bus-with-PERIPH_EN-always-on; reconcile with the LTC4311-vs-buffer open item (peripheral-map §13). LTC4311 does not isolate.
- [ ] **§6 rail enumeration**: explicitly assign IIS2MDC, LIS2DH12 (and PIR) to a rail; §6 currently lists only TMP117/ATECC608B/NOR on PERIPH_EN.
- [ ] **Accel address**: commit 0x18 vs 0x19 and strap SA0 accordingly.
- [ ] **Zephyr bindings**: confirm `st,lis2dh` (LIS2DH12) and the IIS2MDC magnetometer `compatible`/alias.
- [ ] **Local temperature** at the e-compass location vs LIS2DH12 −40/+85 °C (consumer grade).
- [ ] **Magnetometer keep-out** captured as layout rules: current-loop exclusion vs trace current, balanced power returns, non-ferromagnetic finish/parts zone, distances to relay/bucks/magnetics/Rb/OCXO/fan.

---

## 9. Cross-references & change log

**Cross-references:** `ntp_server_peripheral_map.md` (§4 I²C, §6 power domains, §13 open items — I²C buffer/accelerator, e-compass), `sts1000_fault_aggregation.md` (MCP23017 U55/U54 wiring, `PG_INT_N`), `sts1000_vcc_rb_supply.md` (VCC_RB dynamics relevant to mag interference).

**Change log:**
- E-compass: LSM303AGR (NLA) → **IIS2MDC + LIS2DH12** pair; tilt-comp mandatory due to vertical mount.
- Proximity wake: VL53L1X (I²C ToF) → **PaPIRs PIR** (panel/THT), discrete input on U55 GPB2–7; removed from I²C bus.
- Magnetometer copper: **continuous current-free pour under the part** (rejected: plane void); keep-out sized by Ampère-law field budget, balanced returns over distance.
- Bus topology: flagged 3V3_P gating back-power; **buffer-isolated segment** recommended.
