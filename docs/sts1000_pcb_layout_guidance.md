# STS1000 "Meridian" — PCB Layout & Routing Guidance

Actionable layout/routing rules for the PCB designer, derived from the as-built KiCad
schematic (netlist = source of truth) and the datasheet-level subsystem verification.
Standalone; the routing constraints here are authoritative for layout.

**See `sts1000_layout_readiness_review.md`** for the footprint/library package and the
net-class/constraint starter set, and `sts1000_schematic_design_review.md` for design-review
context. This doc is *physical layout*.

## 0. Global assumptions (state these to the board house before starting)

| Parameter | Assumed value | Action if changed |
|---|---|---|
| Stackup | ≥4-layer; recommend 6-layer (Sig/GND/Pwr/Pwr/GND/Sig) for the mixed HV+RF+timing mix | Re-run every Z and width below with the real stackup in a field solver |
| Copper weight | 1 oz (35 µm) outer, 0.5–1 oz inner | Widths scale ≈ inversely with Cu weight |
| Width basis | IPC-2221, **external** trace, **≤20 °C rise** | Halve rise → widen ~1.5×; inner-layer traces need ~2× the outer width for the same rise |
| RF/diff Z basis | Placeholder microstrip/CPWG; **must be confirmed by the fab's impedance calculator** on the committed stackup | Adjust trace/gap to hit target |
| Timing partition | RMII 50 MHz domain is **intentionally decoupled** from the disciplined 10 MHz timing reference — keep them on separate ground regions | — |

All widths below are **minimums with margin**; pours/polygons are always preferred to
traces for the power nets. Where a net is both HV and current-bearing, the HV clearance
rule (§1.3) dominates over the width rule.

---

## 1. Power trace / copper sizing

### 1.1 Rail current + width table

Current estimates from `docs/ntp_server_peripheral_map.md §6` power budget, the regulator
setpoints in `findings_power.md`/`findings_rb.md`, and the Class-6 PoE PD (51 W).
Widths are IPC-2221 external, 1 oz, 20 °C rise (see §1.2 for the calc), rounded up.

| Power net | Voltage | Steady I | Peak / inrush | IPC min width (20 °C) | **Recommended (with margin)** | Notes |
|---|---|---|---|---|---|---|
| **VOUT_P** (PD primary bus, bridge U7/U8 → NCP1095 U9 → bucks U28/U40) | ~48–57 V | ~0.5 A | ~1.1 A (Class-6 51 W); Rb+OCXO cold-start peak draws through here | 0.22 mm @1.1 A | **≥0.75 mm (30 mil) pour** | HV — see §1.3. Route as a short wide pour bridge→bulk→buck PVINs. The **R30 150 mΩ INA228 shunt sits in the series feed** so U10 reads current. |
| **VOUT_N** (PD primary return) | 0 V (primary) | = VOUT_P | = VOUT_P | — | **wide pour, one node** | This is the *primary* return, **not** chassis/SELV GND. Keep it a distinct pour tied to GND only at the single designed point; HV clearance to everything SELV. |
| **5V bus** (U28 MIC28516 out, 8 A device) | 5.0 V | ~2–3 A | ~4 A (OCXO warm-up + 3V3-buck input + display) | 0.9 mm @3 A | **≥1.5 mm (60 mil) pour** | Main secondary distribution. Feeds U29 (3V3), U38 (OCXO pre-reg), U33/U55 (disp/panel), U22 (GPS LDO), ref front-end. |
| **3V3** (U29 AP3441 out, 3 A device) | 3.3 V | ~1 A | ~2 A | 0.51 mm @2 A | **≥0.6 mm (24 mil)** | General digital + PHY (3V3_LAN via FB1) + INA228 VS pins. |
| **3V3_STM** (= 3V3 through R106 15 mΩ Kelvin split) | 3.3 V | ~0.5–0.8 A | ~1 A | 0.30 mm @1 A | **≥0.5 mm (20 mil)** | Always-on housekeeping rail (all I²C peripherals, MCU I/O). R106 is a **Kelvin telemetry split, not a real separate rail** — keep the two Kelvin taps to U31 short & symmetric (§4.4). |
| **VCC_RB** (U40 MIC28516 out, digipot-trimmed) | up to **24.45 V** (26 V OV trip) | ~0.4–1.3 A (Rb 6–20 W @ 15–24 V) | inrush > steady (no dv/dt limit on Q25 gate — see §4.7) | 0.26 mm @1.3 A | **≥0.6 mm (24 mil)** | HV — see §1.3. Gated to VCC_RB_G by SI7469DP **Q25** (gate clamp D25). Provide for inrush: widen to ~1 mm near J6. |
| **V_ANT** (antenna bias, U27 RT9742 out) | 5.0 V | 15–30 mA | 182 mA foldback (hard short) | — | **≥0.3 mm (12 mil)** | Small; but the bias-T injection into the RF path is layout-critical (§5.1). |
| **OCXO rail** (U39 TPS7A5201 LDO out, 3.327 V) | 3.327 V | ~0.4 A | ~1.14 A (3.8 W oven warm-up, ~5 min) | 0.28 mm @1.14 A | **≥0.5 mm (20 mil)** | Star-feed the oven from the LDO with a short wide trace; oven current is the dominant OCXO load. Do **not** neck this down near Y3. |
| **V_PANEL_LED** (U55 RT9742 → FB12 → common anode) | ~4.7 V | ~0.13 A all-on | — | — | **≥0.3 mm (12 mil)** | Low current; keep the 6×91 Ω/1×150 Ω ballast return clean. |
| **Supercap charge/discharge** (U34/U35 TPS61094 SW ↔ L4/L5 ↔ SUP C90/C91) | 3.0 V nom / 2.5 V cap | 25 mA charge | boost SW peak ~0.3–0.5 A | — | **≥0.5 mm SW/SUP, tight loop** | This is the "TPS61094 high-current" path — **minimize the SW–L–SUP–IC loop area**, not width-limited. EP thermal vias. Supercap thermal keep-out §4.8. |
| **GPS_VBAT** (V_BCKP, U34 boost out) | ~3.0 V | ~mA charge | µA backup | — | **≥0.3 mm** | Backup only; low current but keep leakage low (guard the node). |
| **STM_VBAT** (U35 → U12 VBAT pin 6) | ~3.0 V | µA | µA | — | **≥0.25 mm** | RTC/backup domain, µA. |
| **3V3_GPS** (U22 LT3045 out, 3.32 V) | 3.32 V | ~0.2 A | higher on acq | — | **≥0.4 mm** | Low-noise LDO for F9T; series R72 (**0.5 Ω** — consider 0.1 Ω, N4) is the U23 INA228 shunt. |
| **3V0_RF** (U51 LT3045 out, 3.01 V) | 3.01 V | ~100 mA (ILIM R185) | — | — | **≥0.4 mm** | RF-island rail for U50 slicer — keep on the island (§5.2). |
| **VDDA** (U13 LT3045, MCU analog) | 3.3 V | ~mA | — | — | **≥0.4 mm, islanded** | Feeds the 1.65 V OCXO Vc reference divider — keep quiet (§4.1). |

### 1.2 IPC-2221 basis (so widths can be re-derived)

`I = k · ΔT^0.44 · A^0.725` (A in mil², I in A, external k=0.048). Solved at ΔT=20 °C,
1 oz (1.378 mil thick): 1 A→0.20 mm, 2 A→0.51 mm, 3 A→0.90 mm, 5 A→1.82 mm, 8 A→3.46 mm.
At ΔT=10 °C the same currents need ~1.5× the width (1 A→0.30 mm, 3 A→1.37 mm, 5 A→2.76 mm).
**Inner-layer traces:** derate to k≈0.024 → roughly 2× width for the same rise.
For the buck output rails and VOUT_P, **use polygon pours** and let via stitching carry
current between layers (≥0.3 mm vias, multiple in parallel at each rail transition).

### 1.3 High-voltage clearance / creepage

Two HV domains exist. Both are **functional** clearances within the primary side; the true
galvanic isolation barrier is the **RJ45 magjack J1** (1500 Vrms), not on-PCB copper.

| Domain | Working V | Transient | Min clearance (air, IPC-2221 B4 uncoated) | **Recommended** | Rationale |
|---|---|---|---|---|---|
| PoE primary: VOUT_P ↔ VOUT_N ↔ SELV/chassis | 57 V | surge to CR1 SMCJ58A clamp (~93 V) | ~0.6 mm (51–100 V row) | **≥1.0 mm** VOUT_P↔SELV; ≥0.6 mm within primary | Higher margin because a coupling/surge event can push the node; keep the whole PD front end (U7/U8/U9/CR1/R30/C10) in one HV keep-in region. |
| VCC_RB rail | 24.45 V (26 V OV trip) | 28 V TVS D7 | ~0.25 mm (10–30 V row) | **≥0.5 mm** | Use the 30 V row (rail can sit at 24.45 V pedestal and transiently reach 26 V before the OV latch fires). |
| Magjack isolation barrier (J1 line side ↔ board GND) | 1500 Vrms | ESD/surge | — | **≥2.0 mm + routed slot** under J1 | Cut a slot in all copper layers under the magjack isolation line; chassis-GND island under the RJ45 shell, single HV-cap couple (C1 — 4.7 nF **2 kV** Y-cap, needs the 1808/1812 package, see §6). |

Add solder-mask dam / conformal-coat note for the VOUT_P region if the enclosure sees
condensation (Observatory deployment). Do not route SELV signals under the PoE HV pour.

---

## 2. Controlled-impedance nets

Confirm all targets on the committed stackup with the fab's calculator (§0). Reference
plane must be **continuous and unbroken** under every controlled-Z trace — a plane split
under an impedance-controlled net is a defect.

| Net / group | Target Z | Reference | Routing rule |
|---|---|---|---|
| **RMII** RMII_TXD0/1, RXD0/1, TX_EN, CRS_DV, RX_ER, MDC/MDIO (U11 ↔ U12) | 50 Ω single-ended | solid GND directly beneath | Keep the group short; U11 close to U12. Route over one continuous GND plane; no plane splits. |
| **RMII_REF_CLK** (U11 pin14 REFCLKO → U12 PA1, 50 MHz) | 50 Ω single-ended | solid GND | **Most timing-critical RMII net** — shortest, no stubs, no vias if avoidable; it clocks the whole MAC. Length-match to the data group (§3). |
| **USB2.0** USB_DP/USB_DM (U12 PA12/PA11 ↔ J5 USB-C, via U18 TVS) | **90 Ω differential** | solid GND | Tight-coupled pair, TVS U18 stub minimized (§6). Both connector orientations are paralleled at the header — keep the paralleled fan-out symmetric and short. |
| **MDI pairs** TRD0_N/P, TRD1_N/P (U11 pins 20–23 ↔ R42–R45 term ↔ U1 ESD ↔ J1) | **100 Ω differential** | solid GND, clear-out into magnetics | Route PHY→term→ESD→magjack tight and short; ½-plane clear-out into the magjack; **no GND under the RJ45/line side** of the isolation barrier. Only TRD0/TRD1 used (10/100 PHY on 4-pair jack). |
| **GPS RF** GPS_RF_IN (J7 SMA → L10 ESD → L1 bias-T → U21 pin2, 1.2–1.6 GHz) | **50 Ω** CPWG or microstrip | continuous GND, via-fenced | See §5.1. Short, guarded, ground-via fence both sides; L10 shunt at connector, bias-T injection near U21. |
| **10 MHz coax feeds** 10MHz_RF_IN (J8→U50), 10MHz_RF_OUT (U53→J9), 1PPS_OUT (U71→J15) | **50 Ω** | continuous GND | Match the SMA launch to 50 Ω (§5.4). Source terms already at drivers (R184 slicer, R189 fanout, R255/R256 PPS). |
| **PH0 clock** CLK_OUT (U52 mux Y → R187 22 Ω → U12 PH0/pin23) | keep short, ~50 Ω | continuous GND | §3/§4.1. R187 is the *only* series damper — at the mux, short run to PH0. |
| INA228 ALERT / I²C / SPI control | DEFAULT (no Z control) | — | Standard; keep I²C1 (PB8/PB9) short with the R202/R203 pull-ups near the MCU end. |

---

## 3. Matched-length / timing-critical routing

| Group | Match to | Tolerance | Notes |
|---|---|---|---|
| **RMII data** (TXD0/1, RXD0/1, TX_EN, CRS_DV, RX_ER) ↔ **REF_CLK** | all referenced to RMII_REF_CLK | **±5 mm** (≈±34 ps) group skew | RMII is source-synchronous to the 50 MHz REF_CLK; loose vs USB/MDI but keep the group tight to REF_CLK. MDC/MDIO not skew-critical. |
| **USB DP/DM** intra-pair | DP↔DM | **≤2.5 mm** (≈±5 mil) intra-pair | 90 Ω diff; match at the paralleled connector fan-out too. |
| **MDI TRD0_N/P** intra-pair | N↔P | **≤2.5 mm** intra-pair skew | 100 Ω diff. |
| **MDI TRD1_N/P** intra-pair | N↔P | **≤2.5 mm** intra-pair skew | Pair-to-pair (TRD0↔TRD1) match not critical (independent TX/RX). |
| **PH0 clock path** (mux Y → R187 → PH0) | — | **shortest possible**, <~25 mm | Single-ended 10 MHz HSE-bypass into U12 PH0; length adds jitter budget. R187 22 Ω source term at the mux output only (no receiver-end R). |
| **OCXO_CLK_OUT / 10MHz_CLK_OUT into mux** | — | short, symmetric | Source terms R123 (22 Ω at Y3 output) and R184 (33 Ω at U50 Q) stay **at their drivers**; do not add a second series R at the mux inputs. |
| **PPS capture** GPS_PPS→PA0 (TIM2), GPS_TP2→PC6 (TIM3), ETH_PPS_OUT→PB5 | — | keep short, controlled length; **document trace delay** | These are the timestamp references — trace propagation delay is a fixed offset the firmware calibrates, but keep them short and route over solid GND so the delay is stable over temperature. Avoid vias/layer changes that add uncharacterized delay. |

---

## 4. Placement sensitivities

### 4.1 OCXO Y3 + Vc loop filter (highest timing-quality risk)
- **Vc-path caps C98, C101 (and bias-ref C99) are C0G/NP0 or film — never X7R** (piezoelectric microphonics FM-modulate the carrier). The 0.1 µF C0G may force a 1210 body — reserve the pad area.
- The Vc node (OCXO_VC / OCXO_V) is **high-impedance** (R120 100 kΩ, R121 10 MΩ, R122 1 kΩ). **Guard it:** keep U36 (OPA320 buffer), R120/R121/R122, C98/C101 as a tight cluster next to Y3 pin 1; ground-guard the trace; keep all digital/switching nets (DAC1 out PA4, bucks, RMII) away.
- **R123 22 Ω source term at the Y3 output only.** No receiver-end series R on OCXO_CLK_OUT.
- Place Y3 **away from heat sources (bucks, Rb, PoE magnetics) and mechanical/vibration** (board flex, mounting stress). The oven self-heats; give it copper for thermal mass but not a thermal gradient from a neighbor.
- The 1.65 V Vc center reference (R118/R119 off **VDDA**, C99) must stay on the quiet islanded VDDA — do not source it from a switching rail.

### 4.2 25 MHz crystal Y1 (PHY) at U11
- Y1 + load caps C14/C16 (27 pF) tight to U11 pins 4/5; short symmetric traces; local GND guard.
- **Note:** C14/C16 at 27 pF are slightly undersized for a 20 pF-CL crystal (wants ~33 pF); a few-ppm fast REF_CLK is harmless (RMII decoupled from the timing ref). If ppm matters, bump to 33 pF — layout should leave both feasible.
- Keep Y1 away from the MDI pairs and the disciplined-timing region.

### 4.3 Magnetometer U61 (IIS2MDC)
- Place over a **continuous, current-free GND pour** (copper is diamagnetic / magnetically silent). **Do not void** the pour under the sensor — a void routes return current into a loop under it.
- Ensure any current that must pass nearby is **balanced go+return** (1/r² falloff) rather than single-ended (1/r). Keep high-current rails (5V, VCC_RB, panel-LED) and the RGB drivers well away.
- Keep U61 away from Y3 oven heat and any ferrous connector shells.

### 4.4 INA228 Kelvin shunt sensing (all 9)
Every INA228 must Kelvin-sense its shunt: IN+ and IN− tap **at the shunt pads**, routed as
a tight differential pair back to the IC, symmetric, not tapping the power pour.

| INA228 | Shunt | Rail | Placement note |
|---|---|---|---|
| U10 (0x40) | R30 150 mΩ | PoE / VOUT_P | R30 sits in the series VOUT_P→V_POE feed; Kelvin-sense at the shunt pads. |
| U31 (0x41) | R106 15 mΩ | 3V3↔3V3_STM | Kelvin split is *also* the rail feed — symmetric taps critical. |
| U32 (0x42) | R107 100 mΩ | 5V_DISP | senses RT9742 output side. |
| U30 (0x43) | R102 75 mΩ | 3V3 | |
| U23 (0x4A) | R72 0.5 Ω | 3V3_GPS | 0.5 Ω is also the LDO series R (consider 0.1 Ω, N4). |
| U26 (0x45) | R89 **0.15 Ω** | V_ANT | value-field says 100 m; part is 150 mΩ — resolve before setting SHUNT_CAL. |
| U37 (0x46) | R126 25 mΩ | OCXO | IN+ upstream of R126 (polarity correct). |
| U44 (0x47) | R159 20 mΩ | VCC_RB | HV side — keep sense pair inside HV clearance. |
| U54 (0x4C) | R199 220 mΩ | panel-LED 5V | 29 mV = 70.9 % FS ✓. |

I²C1 addressing is final (15 devices, no collisions): **GPS INA U23 = 0x4A, SHT45 U72 =
0x44 — do NOT re-strap U23.**

### 4.5 Secure element U60 (ATECC608B)
- Place close to U12, short I²C1 run; keep the trace away from board edge and probe-accessible
  test points (anti-tamper). Local 0.1 µF (C205) at the pin.

### 4.6 Supercaps C90/C91 (DSF305Q3R0, 3 F) — thermal
- **Keep ≤ ~75 °C** (2.2 V charge term is under the 2.5 V/85 °C derating with margin). Place
  **away from the bucks (U28/U29/U38/U40), the Rb section, the OCXO oven, and the PoE
  magnetics** — put the supercaps on the cool side of the board.
- Reserve the 3.5 mm-pitch can land pattern and the vertical/keep-out volume.

### 4.7 TPS61094 hot loops (U34/U35)
- Minimize the **SW–L4/L5–SUP–IC** loop area (this is the high-di/dt boost loop). EP thermal
  vias to the GND plane. Local 22 µF near VOUT/VBCKP, ≥2.2 µF at VIN.

### 4.8 Rb digipot U43 (MCP41U83) near the FB node
- Place U43 + OPA320 buffer U42 close to the U40 MIC28516 FB divider so the VCTRL injection
  (wiper → R134) trace is short and quiet — it directly sets VCC_RB. Keep it away from the
  24.45 V VCC_RB pour and the SW node. The D25 gate-clamp zener (K→VCC_RB) bounds the Q25 gate.

---

## 5. RF / shielding

### 5.1 GPS 1.5 GHz path (J7 → L10 → L1 bias-T → U21 pin2)
- **50 Ω CPWG**, as short as possible, straight; ground-via fence on both sides of the trace,
  continuous GND reference, no plane breaks.
- L10 (PGB1010603MR polymer ESD, Cj≈0.05 pF) **shunt at the SMA connector**, ahead of any stub.
- Bias-T: L1 (47 nH) choke + series current-limit inject the 5 V antenna bias **near U21**, not at
  the connector (Z ≈ 465 Ω; the board relies on active foldback rather than a series R).
- **Shield can** over the GPS RF front end (J7/L10/L1/U21 RF corner) recommended for the
  Observatory RF environment. Reserve the can keep-out and ground-ring fence now.
- The u-blox reference places a **47 pF C0G series DC-block** between the bias-T node and RF_IN
  (open item) — reserve the pad in the injection path so only the antenna sees the 5 V bias.

### 5.2 10 MHz slicer / ext-ref island (U50 LTC6752 + U51 LDO)
- Partition U50/U51 and the front-end (D8/R174–R186) onto a **3V0_RF island** with its own
  local GND region, tied to main GND at one point; keep it isolated from the RMII 50 MHz domain
  and the bucks. This is the phase-sensitive slicer that conditions the external reference.
- 50 Ω from J8 SMA to the slicer; switched 50 Ω term (R177 49.9 Ω via U49) near the input.

### 5.3 Keep switchers away from OCXO and RF
- The bucks U28/U29/U38/U40 and their SW nodes are the board's noise sources. **Physically
  separate** them from: Y3/OCXO Vc cluster (§4.1), the GPS RF corner (§5.1), the ext-ref island
  (§5.2), and the magnetometer (§4.3). Route buck SW nodes short/wide/tight-loop for EMI (their
  width is loop-driven, not current-driven).

### 5.4 SMA launches (J7 GPS, J8 10MHz-in, J9 10MHz-out, J15 1PPS)
- Tune each launch to 50 Ω (pad clearance + ground-via ring). Post-layout, the trim resistors
  R187 (PH0 damp), R189 (fanout 50 Ω match), R177 (term return-loss) are the knobs — leave them
  accessible. Each coax connector gets its low-Cj TVS at the connector edge (§6).

---

## 6. ESD / protection placement

**Rule: every connector's TVS/ESD device sits at the connector edge, ahead of any trace stub
or series element.** ESD scheme is consistent (from `findings_hmi.md §3`): ribbon/button/
encoder → TPD4E0x arrays; RF/clock coax → SZESD7410 / PGB1010603MR low-Cj; power pins →
SMF5V0A / SMCJ / SMAJ sized to the rail.

| Connector | Signals | ESD device(s) | Placement |
|---|---|---|---|
| **J1 RJ45** | Ethernet MDI | (in-magjack); U1 TPD4EUSB30 across TRD0/1 (PHY side) | U1 at the PHY-side pairs; **Bob-Smith term is internal to the magjack** (4×75 Ω + 1000 pF/2 kV — none external). Chassis-gap slot under J1 (§1.3). **C1 is a 4.7 nF 2 kV Y-cap → 1808/1812 package** (safety/AEC); same for C42/C186. |
| **J5 USB-C** | DP/DM, VBUS, CC1/CC2 | U18 (DP/DM/VBUS) + U65 (CC1/CC2) | At the connector, ahead of the DP/DM pair fan-out — minimize the TVS stub off the 90 Ω pair. |
| **J17 36-pin panel** | display 5 V + SPI/I²C/TOUCH_INT, 7 buttons, encoder A/B/switch, panel-LED cathodes+anode, PROX_WAKE (reed), 5 V (J17.16) + 3 V3 (J17.28) supply pins | U15/U16/U17 + U19/U20 (buttons) + U65/U66/U69/U70 | ESD arrays at the connector edge, ahead of the fan-out. **Route the supply pins: J17.16→5V_DISP, J17.28→3V3_STM, and the encoder Vcc.** Encoder is push-pull → no A/B pull-ups needed. PROX_WAKE (J17.18) is a passive magnetic **reed** switch (no Vcc). |
| **J6 DE-9 (Rb)** | VCC_RB_G, RS232_TX/RX, RB_LOCK_IN | D7 1.5SMCJ28A (pwr), D13/D14 SMAJ15CA (TX/RX), **D26 SMAJ15CA (RB_LOCK_IN)** | At the connector. Every line, including RB_LOCK_IN (D26), has a TVS channel. |
| **J7 SMA (GPS RF)** | GPS_RF_IN | L10 PGB1010603MR (low-Cj polymer) | At the SMA, ahead of the bias-T. **Outdoor run also needs an inline coaxial GDT** (polymer alone fails the combined 1.5 GHz Cj + 5 V-bias-standoff requirement over a long outdoor cable). |
| **J8/J9 SMA (10 MHz)** | 10MHz_RF_IN/OUT | D8 / D9 SZESD7410 (~0.4 pF) | At each SMA. |
| **J15 SMA (1PPS)** | 1PPS_OUT | D23 SZESD7410 | At the SMA. |
| **J10 (RTC_TAMPER)** | single signal | U16 | At the connector. (Buttons/encoder/panel-LED/PROX_WAKE are now all on J17 above.) |
| **J2 fan 1×4** | 5V, PWM, TACH | D22 SMF5V0A on 5V only | Internal fan, signals unprotected — low risk. |
| **J16 alarm** | 6 dry relay contacts | none | Isolated dry contacts; optional field-surge TVS if the alarm wiring leaves the enclosure. |
| **J3 SWD** | debug | none | Internal — OK. |

---

## 7. Thermal & mechanical

**Heat sources (rank):** U28 MIC28516 (5 V, up to 8 A) and U40 MIC28516 (VCC_RB) → U9
NCP1095 PD + bridges U7/U8 → U38 AP3441 (OCXO pre-reg) → Y3 OCXO oven (self-heating,
~3.8 W warm-up) → the Rb module itself (external, but its buck is on-board).

- Give each buck a **copper thermal pour + EP via array** (≥9 vias, 0.3 mm) to an internal
  plane / bottom pour. MIC28516 EP is the primary thermal path.
- **Group the heat on one side** of the board and put the temperature-sensitive parts on the
  other: supercaps C90/C91 (≤75 °C, §4.6), the two TMP117 (U57/U58 — place one near the hot
  cluster, one in ambient for a gradient read), the magnetometer, and the OCXO Vc cluster.
- **Y3 oven:** give it copper for thermal mass but avoid a steep gradient from a neighboring
  hot part (gradient = frequency drift). Isolate it from board-flex mounting stress.
- **Fan (J2):** place the intake over the buck/Rb cluster; FAN_TACH (PA15) has no pull-up —
  use the MCU internal pull-up.

**Mounting holes H1–H14** (0.125"): treat as fixed keep-outs. Confirm which are chassis-GND
tie points vs isolated. Keep the magjack chassis-GND island (§1.3) tied to the nearest
mounting hole under the RJ45 shell. Reserve annular keep-out around each; do not route HV
(VOUT_P, VCC_RB) or RF adjacent to a mounting screw head.

---

## 8. Net-class / constraint starter table (KiCad)

Suggested KiCad net classes. **Widths/clearances are minimums-with-margin at the §0
assumptions**; re-solve on the committed stackup. Assign every net to a class; the leftovers
fall to DEFAULT.

| Net class | Trace width | Clearance | Diff pair (w/g) | Target Z | Member nets |
|---|---|---|---|---|---|
| **POWER_HV** | ≥0.75 mm (pour) | **≥1.0 mm** | — | — | VOUT_P, VOUT_N, V_POE (VCC_RB → its own sub-rule ≥0.5 mm width / ≥0.5 mm clr) |
| **POWER_RB_HV** | ≥0.6 mm | ≥0.5 mm | — | — | VCC_RB, VCC_RB_G, RB_OV_DET path |
| **POWER** | ≥0.6 mm (pour preferred) | 0.2 mm | — | — | 5V, 3V3, 3V3_STM, 3V3_GPS, 3V0_RF, OCXO rail, V_ANT, 5V_DISP, V_PANEL_LED, VDDA, backup rails |
| **RMII** | 0.15 mm (Z-driven) | 0.15 mm | — | 50 Ω SE | RMII_TXD0/1, RXD0/1, TX_EN, CRS_DV, RX_ER, MDC, MDIO, **REF_CLK** (group-match ±5 mm) |
| **USB** | Z-driven | 0.2 mm | 90 Ω pair | 90 Ω diff | USB_DP, USB_DM |
| **DIFF100** | Z-driven | 0.2 mm | 100 Ω pair, ≤2.5 mm intra-skew | 100 Ω diff | TRD0_N/P, TRD1_N/P |
| **RF50** | Z-driven (CPWG) | via-fence | — | 50 Ω SE | GPS_RF_IN, 10MHz_RF_IN, 10MHz_RF_OUT, 1PPS_OUT, CLK_OUT (PH0), OCXO_CLK_OUT, 10MHz_CLK_OUT |
| **CLK_SENSITIVE** | 0.2 mm, short | guard | — | ~50 Ω | OCXO_VC, OCXO_V (high-Z, guarded — §4.1), GPS_PPS, GPS_TP2, ETH_PPS_OUT |
| **DEFAULT** | 0.2 mm | 0.2 mm | — | — | all I²C/SPI/GPIO/ALERT control nets |

**Do not route** DEFAULT/POWER nets under RF50, CLK_SENSITIVE, RMII, USB, or DIFF100 traces
where it breaks their reference plane. HV classes get their own keep-in region.

---

## Open items feeding layout (resolve before or during placement)

- **Footprint/library package** (NOR U62, SMA/DE-9 connectors, U71, L8/L9, the C90/C91 can,
  `fp-lib-table`): see `sts1000_layout_readiness_review.md` before placement.
- **GPS RF_IN 47 pF C0G series DC-block** (§5.1) — reserve the pad in the injection path.
- **R89 value** 100 m vs 150 mΩ part — settle before INA228 SHUNT_CAL and before placing the shunt.
- **C1/C42/C186 2 kV Y-caps** → 1808/1812 package.
- **Rb-buck output caps C125–C128** → ≥50 V (reserve the larger body).
- **Stackup + Cu weight commit** → re-run every Z and width here in a field solver / IPC-2152.
- Vc caps C98/C101/C99 → C0G/NP0 (may force 1210 body; reserve pad area).
- Confirm PoE PD isolated vs non-isolated front end → sets the final magjack barrier + creepage.
```
