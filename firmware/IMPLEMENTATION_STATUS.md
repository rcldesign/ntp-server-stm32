# STS1000 Firmware — Implementation Status

Point-in-time record, firmware scope only. Branch `claude/stm32-firmware-impl-aool3z`
at `5b5a6eb`. This is **not** an authoritative reference — `docs/ntp_server_software_spec.md`
owns firmware behaviour and `ARCHITECTURE.md` owns structure. This file answers one
question: what is done and verified, and what is not.

**Summary:** every implementable requirement in the spec is built, reviewed and
verified — including all of spec §14 that does not require a physical board. What
remains is §14 **bench measurement**, which produces a dataset rather than a firmware
artifact, and is gated on fab.

---

## 1. Verified state

| Check | Result |
|---|---|
| Host test suites | **84/84 passing**, 83 test files |
| Target build | clean, signed images produced (app + MCUboot) |
| FLASH | 760 195 B — 84.4 % of the 900 970 B slot |
| RAM | 580 868 B — 88.6 % of 640 KB (~74 KB free) |
| Reachability gate | **1339 defined, 226 absent, all 226 accounted for** |
| Coverage gate | `core/` ≥ 80 % enforced; actual **96 %** |
| Source size | ~136 k lines across `app/src/core` + `app/src/zephyr` |

The reachability gate is load-bearing here: `--gc-sections` silently drops a function
whose only reference is its own prototype, and host tests cannot see it because they
link `core/` directly. Every "absent" symbol is audited or asserted in
`scripts/reachability.allow` — and entries **leave** it when a real caller appears
(`disc_state` did, when stage 4c wired it).

---

## 2. Complete and reviewed

### Core modules (`app/src/core/`, platform-neutral C11, no allocation)

`atecc` `auth` `cal` `cfg` `disc` `fault` `fwupd` `gnssmgr` `ina228` `logring` `mcp`
`mp` `ntp` `nts` `ptp` `pwrseq` `quality` `refsel` `snmp` `stats` `thermal` `ubx` `ui`
`util` `web`

### Subsystem status

| Subsystem | State |
|---|---|
| Board definition, MCUboot/sysbuild, STiRoT chain | built, signed images |
| Timing engine — PI/FLL, sawtooth correction, holdover, reference state machine | built + tested |
| GNSS — UBX codec, survey-in, antenna supervisor, leap tracking | built + tested |
| NTP server + NTS-KE | built + tested, NTS enabled in the shipped image |
| PTP grandmaster incl. telecom/power profiles | built + tested |
| SNMP v2c/v3 agent + traps | built + tested |
| Web — HTTPS, REST, WSS, SPA | built + tested |
| Maintenance Protocol (MP) — manifest, overrides, leases, vetoes, tunnels, events | built + tested |
| AAA — local, RADIUS, TACACS+, LDAP/LDAPS | built + tested |
| Firmware update — MP/MCP/web paths, anti-rollback | built + tested |
| Power sequencing, fault scan, INA228 monitoring | built + tested |
| UI — display, buttons, encoder, RGB, skyplot | built + tested |
| Secure boot, key zeroization, factory reset | built + tested |
| **§14 stability-analysis chain** — see below | built + tested |

### Spec §14 — everything implementable is done

**CI bullet** — host-side, satisfied:

| §14 CI requirement | Covering suite |
|---|---|
| loop math | `test_disc.c` |
| sawtooth / qErr path | `test_ppscorr.c`, `test_ppsjoin.c`, `test_ubx.c`, `test_disc.c` |
| holdover estimator | `test_quality.c` — 5 dedicated tests |
| config schema + migration | `test_cfg.c` incl. `test_import_runs_the_registered_migration` |

**Security bullet**, host-testable parts: `test_fwupd.c`, `test_fwupd_seam_policy.c`
(signed-image enforcement, downgrade rejection), `test_nts.c`, `test_snmpv3.c`, AAA suites.

**Analysis chain** — the arithmetic and plumbing §14's measurements feed, built so it is
trustworthy *before* bench time rather than written under it:

| Piece | What it is | Proof |
|---|---|---|
| `core/stats/adev.c` | overlapping ADEV, MDEV, residual histogram | closed forms: frequency offset → ADEV **exactly 0**; drift → D·τ/√2; white PM → MDEV τ^−3/2 vs ADEV τ^−1 |
| `core/stats/saw.c` | with/without sawtooth verdict | z = (var(c)−var(r))/var(s) → −1 correct, +1 mispaired, **+3 sign-inverted** |
| `core/stats/phase_rec.c` | versioned record (v3), gap bitmap, identity | version / length-vs-flags / unknown-bit guards; count cross-checked against bitmap |
| `core/stats/phase_join.c` | multi-poll join | epoch, contiguity, overlap-agreement, union bitmap |
| `core/mcp` `PHASE_EXPORT` | offset-chunked export | transport, encoder, and **composed** end-to-end suites |
| `tools/meridian_phase.c` | bench CLI | links the same proved C; refuses records it cannot trust |

### What τ a bench operator reaches

The ring is `DISC_ADEV_CAP` = 512 at τ₀ = 1 s, so one export covers 8 min 32 s and the
octave axis stops at **τ = 128 s** — short of the region §14 asks about.

**The ring is not enlarged, and cannot be.** ~32 B per sample across two rings, the
exporter's staging snapshot and the export blob: a 2048-sample ring reaches τ = 1024 s
*and nothing further* for ~65 KB against ~74 KB free (98.9 % SRAM), and quadruples the
per-second `memmove` on the timing thread. τ = 8192 s is not expressible on the part.

Instead the record carries a capture `epoch` and the monotonic index `seq0`
(`PHASE_REC_F_IDENT`, v3, **+48 B**), and the bench joins polls it can prove:

| polls at 240 s | polling time | joined n | largest m | τ reached |
|---|---|---|---|---|
| 1 | 0 | 512 | 128 | 128 s |
| 12 | 44 min | 3152 | 1024 | **1024 s** |
| 35 | 2.3 h | 8672 | 4096 | 4096 s |
| 273 | 18.1 h | 65535 | 16384 | 16384 s |

The last row is the format ceiling (`n` is `uint16`); `phase_join_meta()` returns
`-EOVERFLOW` rather than truncating.

---

## 3. Not done — hardware-gated

Every remaining item is a **measurement**. No code completes them; the deliverable is a
dataset from a bench.

| §14 requirement | Needs |
|---|---|
| Served accuracy vs a reference grandmaster / UTC source | board + reference clock |
| ADEV/MDEV vs τ, OCXO and Rb, locked and holdover | board + a reference 3–5× more stable than the DUT at every τ |
| PPS residual histograms with/without sawtooth correction | board + GNSS signal |
| Holdover drift vs the characterized model | board + temperature chamber |
| Protocol interop — chrony/ntpd/w32time, ntpsec NTS, `ptp4l`/`phc2sys`, Zabbix | board + live peers |
| PoE class negotiation, staggered warm-up in budget | board + PSE |
| Brownout/PFI save-restore, wedge recovery, GNSS spoof/jam | board |
| 10k+ NTP req/s soak proving the timing path is unperturbed | board + load generator |
| HIL rig — PPS injection, fault simulation | rig |
| TLS cipher/cert scan, debug-auth lockout, secure-erase | board |

**First-session preconditions** (task #74). The one that matters most:
`disc_cfg_t::qerr_sign` defaults to `+1` on the gpsd/chrony convention, which is
**asserted, never measured**. The first capture *is* that measurement — a `SIGN-INVERTED`
verdict there is the feature working, not a fault. Expected good result: sdev uncorrected
~1605 ps, corrected ~61 ps, ratio ~0.04, z = −1.000, verdict `CORRECTED`.

---

## 4. How this was verified

Relevant because several defects on this branch were invisible to a green suite.

- **Mutation testing on every change.** Break the fix, confirm a *named* test fails,
  restore byte-identically under `sha256sum`, confirm green. A fix without a
  mutation-verified test was treated as unverified.
- **First-pass + adversarial review** per change, findings adjudicated rather than
  accepted or dropped.
- **Eleven guards were found green for the wrong reason** — a test mirroring production
  logic instead of exercising it, a fake that never refused, a scan counting lock *calls*
  when the defect was lock *scope*, a comment asserting a property the code did not have
  and the test did not check, a numeric floor that started refusing the outcome the work
  was for, and an assertion anchored on identifiers that do not move with the code.

### The structural lesson

A decision living in Zephyr glue cannot be executed by a host suite, so it gets guarded
by something that *reads* code rather than *runs* it — and a scan cannot distinguish a
correct implementation from a plausible wrong one. Three fail-safes were found this way,
by mutation rather than by reading:

- the fan's release-to-full-airflow (deletable with every test green);
- the K1 RS-232/CMOS relay's deferred restore (scanned, not executed);
- the watchdog's unknown-state report.

Each was fixed by lifting the decision into a Zephyr-free policy header
(`sts_fan_policy.h`, `sts_rb_serial_policy.h`, `sts_super_policy.h`) with its own host
suite. **That is the house pattern for any new decision in glue.**

---

## 5. Residual risks and known limits

| Item | Status |
|---|---|
| `prov_ilk()` hardcodes `liveness_ok = false` | `sys.wdt.en` cannot be re-armed *through its interlock* while a lease is held; the route back is releasing the lease, whose restore path is not interlock-evaluated. |
| pwrseq stage 9 can override a held `sys.wdt.en` lease | Fail-safe direction, window is seconds of bring-up behind a G3 ceremony. Closing it needs a new veto subject. |
| `sts_disc_dac_state()` false until the discipline thread starts | `obj_read` answers `-EIO` for the two Vc objects in a window where the manifest advertises them wired. |
| A Vc lease held across a PFI park | The fast-save records the override, not the loop's last good Vc. Arguably correct; nobody explicitly decided it. |
| `g_writable_objects[]` named half | Judgement-maintained. No test can decide whether a *newly* writable object is safe; the roll-call only refuses to let one become writable silently. |
| Glue-level source scans | `mp_glue.c`, `pwrseq_exec.c`, `disc_thread.c`, `supervisor.c` link into no host suite; assertions about them read text rather than execute it. |
| `sys_csrand_get()` at `disc_thread` init | Compile-verified only. A failed draw leaves the epoch seed 0 — loud (`LOG_WRN`, `identity: none`) and exports become un-joinable rather than wrongly joinable — but only observable on a board. |
| `PHASE_BLOB_CAP` headroom | 8300 B against a real record's 8292. A `BUILD_ASSERT` ties the buffer to the record's parts, so outgrowing it fails the build rather than truncating silently. |
| Never run on hardware | All validation is host tests, disassembly and static analysis. |

---

## 6. Firmware decisions settled on this branch

Recorded so they are not re-litigated. Full argument in each commit message.

- **PA4 has one writer and two paths.** The DAC single-writer rule holds; a maintenance
  override is posted from the console and drained *by* the discipline thread.
- **Two independent park latches** (PFI, maintenance) — the loop resumes only when both
  are clear. refsel's bracket is a third, transient, park.
- **An async actuator is `F_LEASE`, never writable.** `obj.set` has no field that can
  admit a write landing a sequencer pass later.
- **A refused move *to* a fail-safe position is owed; a refused move *away* is just
  refused.** The K1 asymmetry — an unconditional restore would yank the relay from under
  a standing lease.
- **The fan override is a floor, not a level**; a release resolves to full airflow.
- **A console WDI edge is only safe with the watchdog disabled** (`MP_ILK_WDT_OFF`).
- **An object is guarded by default**: the writable set is enumerated by name, so making
  one writable requires saying what puts it back.
- **The runtime absorbs short sample gaps; the bench must not.** `ADEV_MAX_GAP_S` trades
  purity for availability so the τ = 100 s estimate exists on a real antenna. The export
  therefore carries an absorbed-gap count and a bitmap, and the CLI refuses by default.
- **The capture epoch seed is random, and that is load-bearing.** With a constant seed a
  reboot restarts both epoch and index, so records either side present as perfectly
  abutting with no overlap to check — the silent splice the scheme exists to stop.
