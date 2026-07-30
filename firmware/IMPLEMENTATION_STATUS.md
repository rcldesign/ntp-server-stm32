# STS1000 Firmware — Implementation Status

Point-in-time record, firmware scope only. Branch `claude/stm32-firmware-impl-aool3z`
at `9d9bd3b`. This is **not** an authoritative reference — `docs/ntp_server_software_spec.md`
owns firmware behaviour and `ARCHITECTURE.md` owns structure. This file answers one
question: what is done and verified, and what is not.

**Summary:** every implementable requirement in the spec is built, reviewed and
verified. What remains is spec §14 bench validation, which is measurement of a
physical board against reference equipment, and is gated on fab.

---

## 1. Verified state

| Check | Result |
|---|---|
| Host test suites | **76/76 passing**, 76 test files |
| Target build | clean, signed images produced (app + MCUboot) |
| FLASH | 84.1 % of 900 970 B slot |
| RAM | 86.4 % of 640 KB |
| Reachability gate | **1310 defined, 204 absent, all 204 accounted for** |
| Coverage gate | `core/` ≥ 80 % enforced by `scripts/coverage.sh` |
| Source size | ~133 k lines across `app/src/core` + `app/src/zephyr` |

The reachability gate is load-bearing on this project: `--gc-sections` silently drops
a function whose only reference is its own prototype, and host tests cannot see it
because they link `core/` directly. Every "absent" symbol is either audited or asserted
in `scripts/reachability.allow`.

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

### Spec §14 — the implementable half is done

§14's **CI** bullet is host-side and satisfied:

| §14 CI requirement | Covering suite |
|---|---|
| loop math | `test_disc.c` |
| sawtooth / qErr path | `test_ppscorr.c`, `test_ppsjoin.c`, `test_ubx.c`, `test_disc.c` |
| holdover estimator | `test_quality.c` — 5 dedicated tests |
| config schema + migration | `test_cfg.c` incl. `test_import_runs_the_registered_migration` |

§14's **security** bullet, host-testable parts: `test_fwupd.c`,
`test_fwupd_seam_policy.c` (signed-image enforcement, downgrade rejection),
`test_nts.c`, `test_snmpv3.c`, plus the AAA suites.

---

## 3. Not done — hardware-gated

Every remaining item is a **measurement**, not a firmware artifact. No code completes
them; the deliverable is a dataset from a bench.

| §14 requirement | Needs |
|---|---|
| Served accuracy vs a reference grandmaster / UTC source | board + reference clock |
| ADEV/MDEV vs τ, OCXO and Rb, locked and holdover | board + counter, long runs (see the τ note below) |
| PPS residual histograms with/without sawtooth correction | board + time-interval counter |
| Holdover drift vs the characterized model | board + temperature chamber |
| Protocol interop — chrony/ntpd/w32time, ntpsec NTS, `ptp4l`/`phc2sys`, Zabbix | board + live network peers |
| PoE class negotiation, staggered warm-up in budget | board + PSE |
| Brownout/PFI save-restore, wedge recovery, GNSS spoof/jam | board |
| 10k+ NTP req/s soak proving the timing path is unperturbed | board + load generator |
| HIL rig — PPS injection, fault simulation | rig |
| TLS cipher/cert scan, debug-auth lockout, secure-erase | board |

**In progress:** a Zephyr-free `core/stats` module computing overlapping ADEV, MDEV and
PPS residual histograms, verified against closed-form results (pure frequency offset →
ADEV exactly 0; drift → D·τ/√2; white PM → MDEV τ^−3/2 where ADEV is τ^−1). This is the
*analysis* the above measurements feed. Building it now means it is trustworthy before
bench time rather than written under bench-time pressure.

### What τ a bench operator can actually reach

The ADEV phase ring is `DISC_ADEV_CAP` = **512 samples at τ₀ = 1 s**, so one
`PHASE_EXPORT` covers 8 min 32 s. Overlapping ADEV needs n ≥ 2m+1, so m ≤ 255, and the
octave axis `meridian_phase` prints therefore **stops at τ = 128 s**. That is short of the
region §14 is asking about for an OCXO and a rubidium.

**The ring is not enlarged to fix this, and cannot be.** Each extra sample costs 4 B in
`disc_ctx_t`'s corrected ring, 4 B in its uncorrected ring, the same again in the
`disc_phase_snap_t` the exporter stages through, and 16 B in the export blob — ~32 B per
sample plus two bitmaps. A 2048-sample ring reaches τ = 1024 s and nothing further, for
about **65 KB** against the **~73 KB** of SRAM this build has left (88.62 % of 640 KB used),
and it quadruples the per-second `memmove` on the timing thread. τ = 8192 s is not
expressible on the part at all.

Instead the record carries **sample identity** — a capture `epoch` and the monotonic index
`seq0` of its first sample (`PHASE_REC_F_IDENT`, format v3) — and the bench joins successive
polls:

```
meridian_ctl.py phase-export run.phr --repeat 12    # 44 min of polling, 240 s apart
meridian_phase run.*.phr                            # τ axis to 1024 s
```

`stats/phase_join.c` refuses any join it cannot prove: a differing epoch (a ring reset,
holdover entry, `disc_restart()` or a reboot), a hole between records, or an overlap whose
samples disagree. Overlap is the *mechanism*, not a nuisance — polls taken faster than the
ring empties share samples, and those shared samples are what verify the splice.

| polls at 240 s | polling time | joined n | record span | largest octave m | τ reached |
|---|---|---|---|---|---|
| 1 | 0 | 512 | 8.5 min | 128 | 128 s |
| 4 | 12 min | 1232 | 20.5 min | 512 | 512 s |
| 12 | 44 min | 3152 | 52.5 min | 1024 | 1024 s |
| 35 | 2.27 h | 8672 | 2.41 h | 4096 | 4096 s |
| 273 | 18.1 h | 65535 | 18.2 h | 16384 | 16384 s |

Polling time is `(polls − 1) × 240 s`; the span is longer because the first poll already
carries the 512 s the ring held when the operator started. n = 512 + (polls − 1) × 240,
and the largest octave is the greatest power of two with 2m + 1 ≤ n.

The last row is the format's own ceiling: `n` is a `uint16`, so 65535 samples is the
longest series a single record can express, and `phase_join_meta()` refuses to describe a
larger join rather than truncate one.

---

## 4. How this was verified

Relevant because several defects on this branch were invisible to a green suite.

- **Mutation testing on every change.** Break the fix, confirm a named test fails,
  restore byte-identically under `sha256sum`, confirm green. A fix without a
  mutation-verified test was treated as unverified.
- **First-pass + adversarial review** per change, with findings adjudicated rather than
  accepted or dropped.
- **Ten separate guards were found green for the wrong reason** — a test mirroring
  production logic instead of exercising it, a fake that never refused, a scan counting
  lock *calls* when the defect was lock *scope*, a comment asserting a property the code
  did not have and the test did not check, and a numeric floor that started refusing the
  outcome the work was for.

### The structural lesson

A decision living in Zephyr glue cannot be executed by a host suite, so it gets guarded
by something that *reads* code rather than *runs* it — and a scan cannot distinguish a
correct implementation from a plausible wrong one. Three fail-safes were found this way,
by mutation rather than by reading:

- the fan's release-to-full-airflow (deletable with every test green);
- the K1 RS-232/CMOS relay's deferred restore (structurally scanned, not executed);
- the watchdog's unknown-state report.

Each was fixed by lifting the decision into a Zephyr-free policy header
(`sts_fan_policy.h`, `sts_rb_serial_policy.h`, `sts_super_policy.h`) with its own host
suite. **That is the house pattern for any new decision in glue.**

---

## 5. Residual risks and known limits

Carried deliberately, each with a stated reason.

| Item | Status |
|---|---|
| `prov_ilk()` hardcodes `liveness_ok = false` | `sys.wdt.en` cannot be re-armed *through its interlock* while a lease is held; the route back is releasing the lease, whose restore path is not interlock-evaluated. Publishing the liveness gate is a platform change. |
| pwrseq stage 9 can override a held `sys.wdt.en` lease | Fail-safe direction (watchdog on), window is seconds of bring-up behind a G3 typed-phrase ceremony. Closing it needs a new veto subject. |
| `sts_disc_dac_state()` false until the discipline thread starts | `obj_read` answers `-EIO` for the two Vc objects in a window where the manifest advertises them wired. |
| A Vc lease held across a PFI park | The power-fail fast-save records the override, not the loop's last good Vc. Arguably correct — it is what the oven sees — but nobody explicitly decided it. |
| `g_writable_objects[]` named half | Judgement-maintained. No test can decide whether a *newly* writable object is safe; the roll-call only refuses to let one become writable silently. |
| Glue-level source scans | `mp_glue.c`, `pwrseq_exec.c`, `disc_thread.c`, `supervisor.c` link into no host suite, so assertions about them read text rather than execute it. Mutation-verified, but shape not behaviour. |
| Never run on hardware | All validation is host tests, disassembly and static analysis. |

---

## 6. Firmware decisions settled on this branch

Recorded so they are not re-litigated. Full argument in each commit message.

- **PA4 has one writer and two paths.** The DAC single-writer rule holds; a maintenance
  override is posted from the console and drained *by* the discipline thread.
- **Two independent park latches** (PFI, maintenance) — the loop resumes only when both
  are clear, so neither direction can defeat the other. refsel's bracket is a third,
  transient, park.
- **An async actuator is `F_LEASE`, never writable.** `obj.set` has no field that can
  admit a write landing a sequencer pass later.
- **A refused move *to* a fail-safe position is owed; a refused move *away* is just
  refused.** The K1 asymmetry — an unconditional restore would yank the relay from under
  a standing lease.
- **The fan override is a floor, not a level**; a release resolves to full airflow.
- **A console WDI edge is only safe with the watchdog disabled** (`MP_ILK_WDT_OFF`).
- **An object is guarded by default**: the writable set is enumerated by name, so making
  one writable requires saying what puts it back.
