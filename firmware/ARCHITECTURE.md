# STS1000 Firmware — Architecture Contract

Binding implementation contract for the `firmware/` tree. Behavior comes from
`../docs/ntp_server_software_spec.md` (spec) and the pin contract from
`../docs/sts1000_firmware_hardware_interface.md` (interface ref). This file fixes the
*structure*: module boundaries, APIs, partitions, protocols, and the test/coverage policy.
Where this file and the spec disagree on behavior, the spec wins; where it disagrees with
the interface ref on pins, the interface ref wins.

---

## 1. Toolchain (pinned, proven)

| Item | Value |
|---|---|
| Zephyr | **v4.2.2** via `firmware/west.yml` (T2 manifest; module allowlist: cmsis, cmsis_6, hal_stm32, hal_st, mbedtls, mcuboot, littlefs, zcbor, picolibc) |
| SDK | zephyr-sdk **0.17.2**, `arm-zephyr-eabi` |
| Build | `west build -b sts1000_meridian --sysbuild firmware/app` (sysbuild builds MCUboot + app, signs app) |
| Host tests | CMake + CTest + Unity (vendored), `gcc`, coverage via `gcovr` |

Workspace topology: workspace root is the **parent** of this repository
(`west init -l ntp-server-stm32 --mf firmware/west.yml`).

---

## 2. Repository layout (`firmware/`)

```
firmware/
  west.yml                      manifest (pinned)
  ARCHITECTURE.md               this file
  CLAUDE.md                     working preamble
  boards/rcldesign/sts1000_meridian/
    board.yml  Kconfig.sts1000_meridian  sts1000_meridian_defconfig
    sts1000_meridian.dts  sts1000_meridian-pinctrl.dtsi  board.cmake  support/openocd.cfg
  app/
    CMakeLists.txt  prj.conf  Kconfig  VERSION  sysbuild.conf
    sysbuild/mcuboot.conf  sysbuild/mcuboot.overlay
    keys/mcuboot_dev_ec_p256.pem      DEV-ONLY signing key (marked, never for production)
    src/
      main.c                          bring-up sequencer entry (stages per interface ref §2)
      core/                           platform-neutral, host-testable (rule §4)
        util/  cfg/  quality/  disc/  refsel/  ubx/  gnssmgr/  ntp/  nts/  ptp/
        ina228/  fault/  pwrseq/  thermal/  mcp/  logring/  snmp/  ui/
      port/                           port-interface headers (core → world)
        port.h  port_time.h  port_crypto.h ...
      zephyr/                         Zephyr glue (threads, drivers, port impls)
        threads/  drv/  portz/
  tests/host/
    CMakeLists.txt  unity/  support/  test_*.c
  tools/
    meridian_ctl.py                   reference PC tool (MCP client: config/telemetry/logs/DFU)
  scripts/
    build.sh  test.sh  coverage.sh
```

---

## 3. Flash partitioning & boot chain

Internal flash: 2 MB, 8 KB sectors (STM32H563ZIT6). STiRoT (ROM-anchored, OBK-configured)
verifies and launches MCUboot; MCUboot (swap-move, ECDSA P-256, SHA-256, anti-rollback)
manages A/B application slots.

| Partition | Offset | Size | Contents |
|---|---|---|---|
| `boot_partition` | 0x000000 | 128 KB | MCUboot (+ serial recovery over USB CDC-ACM) |
| `slot0_partition` | 0x020000 | 896 KB | Active application (signed) |
| `slot1_partition` | 0x100000 | 896 KB | Staged application (DFU target) |
| `storage_partition` | 0x1E0000 | 64 KB | NVS (settings backend: config, PFI fast-save, cal deltas) |
| *reserved* | 0x1F0000 | 64 KB | anti-rollback/provisioning scratch (unused by app) |

External SPI-NOR (MX25L25645, 32 MB, SPI4): LittleFS mount `/lfs` — bulk config export,
calibration tables, log spool, GNSS cache, staged assets. NOR is **not** required to boot.

Upgrade paths (both serial, per user requirement):
1. **Application alive:** PC tool → MCP `FW_*` commands over CDC-ACM → image streamed into
   `slot1` (flash_img), SHA-256 verified, `boot_request_upgrade(TEST)`, reboot; app
   self-confirms after health checks (`boot_write_img_confirmed()`), else MCUboot reverts.
2. **Application dead:** MCUboot **serial recovery** (MCUmgr/SMP over CDC-ACM) accepts a
   signed image directly. Entry: held button / retention flag / absent valid image.
3. First-ever load: ST-LINK (SWD) of the merged `boot+app` image.

---

## 4. Core / glue split (the testability rule)

**`src/core/` is platform-neutral C11.** No Zephyr headers, no HAL, no globals holding
hardware state. Each module talks to the world only through the `port/` interfaces or
through caller-supplied function pointers/context structs. This is what makes ≥80 % unit
coverage achievable on the host.

**`src/zephyr/` implements the ports** (I²C/SPI/UART transfers, time, flash, crypto via
mbedTLS/PSA, sockets) and owns threads, ISRs, devicetree, and driver instances.

Conventions:
- Module prefix per directory (`ntp_`, `mcp_`, `disc_`, …). All state in a `*_ctx` struct
  owned by the caller. No dynamic allocation in core (fixed pools provided by glue).
- Return `int` (0 / negative errno-style). No asserts that kill the host test runner.
- Headers under `core/<mod>/<mod>.h` are the module's public API; internal headers stay
  private. Cross-core-module dependencies are fixed by this map (also encoded in the
  host-test CMake; adding an edge requires updating both):
  `ubx→util`; `gnssmgr→ubx,util`; `disc→util,quality`; `refsel→util`; `quality→util`;
  `thermal→util`; `ntp→util,quality`; `nts→util`; `ptp→util,quality`; `ina228→util`;
  `fault→util`; `pwrseq→ina228,util`; `mcp→util,cfg,logring,quality`; `cfg→util`;
  `logring→util`; `snmp→util,quality`; `ui→util,quality`;
  `web→util,cfg,logring,quality`; `mp→util,cfg,logring,quality,ina228`;
  `atecc→util`; `auth→util`; `fwupd→util,ubx`.
- Two qualifications on that map, both recorded in `tests/host/CMakeLists.txt` because the
  build is what enforces them:
  - **Types-only includes are not edges.** `mp` includes `ui/ui.h` and `port/port_image.h`
    for `ui_hint_t`/`UI_ATTR_*` and the reboot-mode enum, but references no `ui` symbol, so
    `ui` is deliberately not a link dependency of `mp`. Port headers are never edges in this
    map — they are the platform seam, not core modules.
  - **A module may own several suites.** `snmp` carries a second suite, `snmpv3`
    (`snmpv3→snmp,util,quality`), because USM is a distinct layer over the shared PDU codec.

---

## 5. Module inventory & ownership

| Module | Purpose (spec ref) | Key API surface |
|---|---|---|
| `util` | CRC32 (IEEE), CRC16-CCITT, COBS enc/dec, ring buffer, be/le helpers | `crc32()`, `cobs_encode/decode()`, `ring_*` |
| `cfg` | Typed config registry: numeric key IDs `0xGGII` (group/item), types u8..u64/i32/f32/bool/str/blob; defaults, bounds validation, dirty/commit, TLV export/import, schema version + migration | `cfg_get/set/commit/export/import`, `cfg_iter` |
| `quality` | §3.8 single source of truth: quality block struct, publish/snapshot (seqlock pattern; glue provides memory), root-dispersion growth in holdover | `quality_publish()`, `quality_snapshot()` |
| `disc` | §3.2–3.3, §3.6: PPS sample conditioning (sawtooth qErr apply, cable-delay offset, PA0-vs-PC6 cross-check, median/MAD gate), FLL+PI loop (τ 10–1000 s), DAC scaling ±0.4 ppm FS center 1.65 V + slew limit, tempco feed-forward, lock criteria, holdover estimator + freeze, rate-limited re-converge | `disc_tick_pps()`, `disc_tick_no_pps()` → `disc_out{dac_code, state}` |
| `refsel` | §3.5 reference SM: OCXO_ACTIVE⇄RB_ACTIVE(+EXTREF), guards (`extref_ok && rb_lock`), debounce/hysteresis, emits action list (`BRIDGE_HSI`, `SET_MUX(x)`, `UNBRIDGE`) executed by glue | `refsel_input()`, `refsel_step()` |
| `ubx` | UBX frame codec (sync/class/id/len/ck), builders: CFG-VALSET (UART, rate, constellations, TP5, TXREADY, TMODE3), MON/NAV/TIM pollers; parsers: NAV-PVT, NAV-SAT, NAV-TIMELS, TIM-TP (qErr), MON-RF, ACK | `ubx_frame()`, `ubx_parse_byte()`, typed msg structs |
| `gnssmgr` | §3.7 receiver lifecycle SM: init config sequence w/ ACK tracking, survey-in → fixed-position, leap-second tracker, antenna supervisor fusion (MON-RF + `ANT_OFF` + INA 0x45 bands → OK/OPEN/SHORT + persistent-short bias-drop) | `gnssmgr_step()`, `gnssmgr_on_msg()` |
| `ntp` | §4.1/4.3: RFC 5905 server datapath: parse/validate request, build response from quality block (LI/stratum/refid/root delay+disp, precision), NTP↔TAI timestamp conversion, rate-limit + KoD (RATE), per-client token buckets, interleaved mode, symmetric-key MAC (SHA-256 via port) | `ntp_handle_request(pkt, rx_ts, quality) → rsp` |
| `nts` | §4.2: NTS extension-field parse/build, cookie AEAD (AES-SIV-CMAC-256 built on port-supplied AES-ECB), master-key ring + rotation, NTS-KE record protocol SM (TLS transport supplied by glue) | `nts_process_ext()`, `ntske_step()` |
| `ptp` | §4.4: 1588-2019 GM: message codecs (Announce/Sync/Follow_Up/Delay_Req/Resp, header TLV), BMCA dataset comparison, per-port SM (only MASTER/PASSIVE roles needed + LISTENING), clockClass/accuracy/variance mapping from quality, log-interval schedulers, E2E; L2 + UDPv4/v6 transports via port | `ptp_port_step()`, `ptp_rx()`, `ptp_make_*()` |
| `ina228` | Register codec + conversions (per-rail CURRENT_LSB table from interface ref §4.2, SHUNT_CAL=4096+trim, SOVL/SUVL codes, DIAG_ALRT decode), rail table for all nine devices | `ina228_decode_*`, `ina228_rail_tbl[]` |
| `fault` | §5 scan model: 1 kHz GPIOF/GPIOG snapshot diff → debounce (per-class) → events (button press/long/repeat, EN-fault, PG-drop, INA alert); alarm aggregator + fault latch table → RGB/relay/trap policy | `fault_scan_input(f_idr, g_idr, ms)`, `fault_alarms()` |
| `pwrseq` | Interface ref §2 stage machine (Stages 2–9) incl. guarded Rb sequence (digipot safe code → RB_PWR_EN → INA 0x47 window verify → RB_VCC_GATE → lock wait) + PoE budget gate + OV-latch observe/clear; emits port actions, consumes fault/INA state | `pwrseq_step()` |
| `thermal` | §10.2 fan PI loop (25 kHz PWM duty out, tach RPM in, hysteresis, min duty, fail-safe max), over-temp shed ladder (fan max → shed Rb → POE_KILL) | `thermal_step_1hz()` |
| `mcp` | §7 + user req: **Meridian Console Protocol** (framing + command router + DFU session SM + telemetry/log event encoder + auth gate) — see §7 below | `mcp_input()`, `mcp_poll_tx()` |
| `logring` | Structured log ring (RAM), RFC 5424 render, cursor-based tail, drop counters; NOR spool + syslog senders live in glue | `logr_put/tail` |
| `snmp` | Compact **read-only** agent + traps: BER codec, OID table walk bound to quality/health metrics catalog (§10.5 subset). **SNMPv3/USM is no longer deferred** — `snmp_v3.c` adds HMAC-SHA-256 auth, AES-128-CFB privacy, engine discovery/Report PDUs and authenticated notifications; `snmp_dispatch()` routes by msgVersion and is the live receive path, with v2c behind `sec.snmp.v2c` | `snmp_dispatch(pkt)`, `snmp_v3_handle()`, `snmp_make_trap()`, `snmp_v3_make_notification()` |
| `ui` | Screen-stack nav SM (§6.1/6.2), button/encoder event consumption, page render into an abstract "text/tile" surface (glue rasterizes to ST7796), RGB pattern selector (§2.8), backlight/wake policy | `ui_input()`, `ui_render(surface)` |

### Phase-2 modules (all landed; see the FMT spec for the maintenance surface)

| Module | Purpose | Key API surface |
|---|---|---|
| `web` | HTTPS management plane: HTTP/1.1 parser, versioned REST router (`/api/v1`), RFC 6455 WSS telemetry/log push, session + CSRF auth with lockout. Glue adds the TLS listener, persisted self-signed cert/CSR and LittleFS asset serving. DFU reuses `mcp_dfu` over the same `port_image` — one state machine | `http_parse()`, `rest_dispatch()`, `wss_*`, `auth_web_*` |
| `mp` | Maintenance Protocol for the Field Maintenance Tool: COBS+CRC16 channel mux, JSON-RPC control plane, capability manifest, override/lease engine with dead-man revert and G0–G3 guards, CBOR telemetry/PPS/log/event streams, UART tunnels, diag runner, **panel mirror** (screen tiles + lamp state + live inputs) | `mp_input()`, `mp_manifest_*`, `mp_override_*`, `mp_mirror_*` |
| `snmp` | SNMPv2c **and v3 USM** (HMAC-SHA-256 auth, AES-128-CFB priv, RFC 3414 key localization, engineID from the ATECC serial), traps/informs, notification gating | `snmp_handle()`, `snmp_make_trap()` |
| `auth` | AAA chain local → RADIUS → TACACS+ → LDAP, role mapping (admin/operator/viewer), TTL cache, lockout shared with the local path | `auth_check()` |
| `atecc` | ATECC608B protocol layer (ATCA framing/CRC16, wake/idle/sleep, Info/Random/Sign/Verify/ECDH/SHA/HMAC/Counter/…). Transport via callback; tolerates an absent part by falling back to software keys | `atecc_cmd_*`, `atecc_resp_parse()` |
| `fwupd` | **Multi-IC firmware update orchestrator + component inventory** (11 rows: STM32, ZED-F9T, FE-5680A variant-gated, plus the read-only-ID parts). Per-target QUERY→PREPARE→TRANSFER→VERIFY→RESTORE with `restore()` guaranteed once per `prepare()` on every exit path | `fwupd_inventory()`, `fwupd_begin/data/end/abort()` |
| `ptp` (extended) | Telecom **G.8275.1/.2** and Power **C37.238** profiles (Default provably unchanged), Annex-P AUTHENTICATION TLV with replay window verified *after* the ICV, `ever_locked` gate so a never-locked unit advertises clockClass 248 | `ptp_cfg_apply_profile()`, `ptp_icv_*` |
| `ui` (extended) | Polar skyplot renderer (az/el, constellation colour, used-vs-visible fill, true-north with declination + tilt, "north unverified" badge) with golden-image tests | `skyplot_render()` |

**Still deferred, with the reason:** ACME (compiled-out skeleton; needs an HTTPS
client, base64url/JWS-ES256 and a trustworthy clock at first boot — the gaps are
enumerated in `sts_cert.c`); Argon2id credential stretching (plumbed via `auth_kdf_t`,
but switching it is a coordinated flag day with MCP because the 48-byte envelope cannot
record which KDF produced the tag); FE-5680A firmware update (no loader protocol is
documented anywhere reachable — reported as `NOT_SUPPORTED` rather than attempted, and
the only item on this list that no amount of work closes); **NTS-KE** (`sts_ntske.c` is a
complete TLS 1.3 key-exchange server driving mbedTLS directly, because Zephyr's TLS socket
layer exposes no RFC 8446 exporter — but its whole body compiles only under
`MBEDTLS_SSL_KEYING_MATERIAL_EXPORT`, which Zephyr's mbedTLS config omits and offers no
Kconfig for. `sts_ntske_supported()` therefore returns false, `sts_ntp.c:1058` gates
`nts_enabled` on it, and the appliance advertises what the build can actually do rather
than NAKing cookies no client could have been issued. The three enabling steps —
a `zephyr_include_directories()` line for `sts_mbedtls_user.h`, the two user-config
Kconfigs, the TLS 1.3 block — are written out and commented in `app/conf/net.conf`);
an **ATECC-backed TLS server key** (the HTTPS identity is still a software PEM on NOR —
routing it to the secure element needs an mbedTLS `PK_OPAQUE`/PSA driver over
CryptoAuthLib, which is a driver, not wiring; the part is already the root for the SNMP
engine id, attestation and the anti-rollback counters. It is **not** the entropy source:
the CSPRNG is `sys_csrand_get()` on the STM32H5 RNG peripheral (`portz_crypto.c:137`).
`sts_atecc_random()` exists, works, and has no caller — see `scripts/reachability.allow`).

**No longer deferred** — struck from the list above as they landed, recorded here because
a deferral list that quietly loses entries is indistinguishable from one nobody maintains:
operator/viewer accounts (now `sec.operator.pw` 0x0A3E and `sec.viewer.pw` 0x0A3F, so the
roles the MP guard floor keys off exist locally instead of being RAM-only ghosts); key
zeroization on factory reset (every `CFG_F_SECRET` blob, each subsystem's RAM copy via the
applier fan-out, the web plane's sessions through a volatile pointer, and the persisted TLS
identity — the reboot is load-bearing, not a convenience, since it is what clears
RAM-only key material and mints the replacement identity); the **MP UART7/rubidium tunnel**
and the **NMEA/UBX tee producers**; and **anti-rollback**, which was never on this list
because nobody had noticed it was absent — see `app/conf/rollback.conf`.

> **Reachability is a build property, not a source property.** `--gc-sections` silently
> discards any function nothing calls, so an entry point with no caller produces code that
> compiles, tests green and is absent from the image. This bit the project once, at the
> largest possible scale: `sts_mp_start()`/`sts_mp_tick()` had no callers, so the whole
> Maintenance Protocol engine — `mp_init`, the override/lease engine, the guard classes,
> the mirror provider — was stripped from every image built before it was wired into
> `sts_console.c`. When a module is declared done, confirm its symbols survive into
> `zephyr.elf` (`nm`), not merely that a call chain exists in the source.

---

## 6. Thread wiring (Zephyr glue)

Per spec §1.2 (priorities as listed there). Static threads, static stacks:

| Thread | Prio | Notes |
|---|---|---|
| `pps_capture` | ISR + k_work | TIM2/TIM3 capture ISR → sawtooth pairing with TIM-TP in `gnss` context → `disc_pps_input` |
| `discipline` | 4 | 1 Hz paced by PPS semaphore (timeout 1.2 s → missed-PPS path); sole DAC writer; publishes quality |
| `ptp` | 5 | socket/L2 RX + timer events |
| `gnss` | 6 | UART3 DMA/int RX → `ubx_parse_byte`; drives `gnssmgr` |
| `ntp_server` | 8 | UDP 123 RX loop; HW timestamps via MAC PTP clock |
| `net_mgmt` | 10 | DHCP/mDNS/link events |
| `io_scan` | 11 | 1 kHz k_timer → GPIOF/GPIOG IDR read → `fault_scan_input` |
| `snmp` | 12 | UDP 161 + traps |
| `console`/`mcp` | 14 | CDC-ACM #0 shell (Zephyr shell); CDC-ACM #1 binary MCP |
| `housekeeping` | 14 | 1–4 Hz I²C sensor sweep, `pwrseq_step`, `thermal_step`, supervisor + WDT kick |
| `ui_local` | 15 | 10 Hz render/nav |
| `logger` | 16 | drain logring → NOR spool / syslog |

Rule (spec §1.2): management threads never take timing locks — they read the seqlock
quality snapshot.

USB: **composite device, 2× CDC-ACM** — ACM0 = Zephyr shell + log console (human),
ACM1 = MCP binary channel (PC tool). MCUboot serial recovery enumerates its own CDC.

---

## 7. MCP — Meridian Console Protocol (binary, PC-tool channel)

Framing: **COBS**-encoded frames, `0x00` delimiter. Decoded frame:

```
u8  ver (=1) | u8 type (0=REQ 1=RSP 2=EVT) | u8 cmd | u8 flags
u16 seq (LE) | u16 len (LE)  | payload[len] | u32 crc32 (LE, over ver..payload)
```

Max payload 1040 B. RSP carries same `cmd`/`seq`; payload starts with `u8 status`
(0=OK, else `mcp_err_t`). EVTs use their own `seq` space, `flags.bit0` = subscription id.
Mutating commands require a prior `AUTH` in the session unless auth is disabled by config.

| cmd | Name | Payload → response |
|---|---|---|
| 0x01 | HELLO | → proto ver, fw ver (running+staged), board id, capabilities bitmap |
| 0x02 | REBOOT | u8 mode (0=normal 1=bootloader/recovery 2=halt-to-test) |
| 0x08 | AUTH | pw/token → session grant |
| 0x10 | CFG_LIST | u16 start-id, u8 max → array of {id, type, flags, name} |
| 0x11 | CFG_GET | u16 id → TLV value |
| 0x12 | CFG_SET | u16 id + TLV (staged) |
| 0x13 | CFG_COMMIT | validate-then-commit staged set (atomic) |
| 0x14 | CFG_REVERT | drop staged |
| 0x15 | CFG_EXPORT | cursor → TLV stream chunks |
| 0x16 | CFG_IMPORT | TLV stream chunks + final commit |
| 0x17 | FACTORY_RESET | magic u32 0x46414354 ("FACT") required |
| 0x20 | STATUS_GET | u8 group (0=summary 1=timing 2=gnss 3=power 4=net 5=ptp 6=alarms) → packed struct (versioned) |
| 0x21 | TELEM_SUB | u8 group-mask, u8 rate-Hz (≤4) → EVT stream |
| 0x22 | TELEM_UNSUB | — |
| 0x30 | LOG_TAIL | u32 cursor, u8 follow → records (+EVT stream if follow) |
| 0x31 | LOG_LEVEL | u8 subsystem, u8 level |
| 0x40 | FW_INFO | → slot table {version, hash, flags(active/pending/confirmed)} |
| 0x41 | FW_BEGIN | u32 size, u8 sha256[32] → grants DFU session (erases as it goes) |
| 0x42 | FW_DATA | u32 offset + chunk (≤1024) → u8 status, u32 next-expected (idempotent, resumable) |
| 0x43 | FW_END | → verify SHA-256 + MCUboot magic → mark slot1 pending(TEST) |
| 0x44 | FW_CONFIRM | confirm running image |
| 0x45 | FW_REVERT | request revert on next boot |
| 0x50 | DIAG | u8 sub (0=health snapshot 1=I²C scan 2=PPS residual histogram 3=thread/CPU stats 4=INA dump) |

DFU session rules: single session; `FW_DATA` out-of-order → status=ERR_OFFSET with
`next-expected` (tool rewinds); flash writes page-aligned via port; `FW_END` fails if
SHA-256 mismatch or image header invalid → slot marked unusable. All of this is in
`core/mcp` and unit-tested against a RAM flash port.

`tools/meridian_ctl.py` implements the client side (pyserial): `info`, `status`,
`watch`, `cfg get/set/commit/export/import`, `log tail`, `fw upload/confirm/revert`,
`reboot`. This is the reference for "the proprietary tool".

---

## 8. Config keys (groups)

Group IDs (`0xGG` high byte): 0x01 net, 0x02 ntp, 0x03 nts, 0x04 ptp, 0x05 gnss,
0x06 timing (loop τ, thresholds, cable delay), 0x07 power (Rb enable policy, digipot
bound, PoE budget), 0x08 ui, 0x09 log/syslog, 0x0A security (auth mode, pw hash),
0x0B snmp, 0x0C cal (INA trims ×9, PPS offsets, tempco). Persisted via Zephyr settings/NVS
(`cfg` core is storage-agnostic; glue registers a settings handler). PFI fast-save writes
the volatile-critical subset (last Vc, leap, log cursor) through a dedicated NVS id.

---

## 9. Test & coverage policy (gate)

- **Unit tests (host):** every `core/` module has `tests/host/test_<mod>.c` (Unity).
  `ctest` runs them; `scripts/coverage.sh` runs gcovr over `app/src/core` with
  `--fail-under-line 80`. **The 80 % line-coverage gate applies to `app/src/core/`** —
  the platform-neutral logic, which is where correctness lives. Zephyr glue/drivers are
  compile-verified by the target build (west) and exercised on hardware.
- **Target build:** `scripts/build.sh` must produce signed `zephyr.signed.bin` +
  `mcuboot.bin` with zero warnings from app code (`-Wall -Wextra` on app sources).
- Protocol vectors: NTP/NTS/PTP/UBX/SNMP tests include known-good byte vectors
  (hand-derived from the RFCs/ICDs) — not just self-round-trips.
- Determinism: core modules take time/entropy via ports → tests inject.

---

## 10. Key hardware-behavior invariants (enforced in code review)

1. `MUX_SEL` (PB6) written only by `refsel` action executor with HSI-bridge sequence.
2. DAC (PA4) written only by `discipline` thread; park via explicit `disc` API (PFI).
3. Rb order: safe digipot code (verify readback) → `RB_PWR_EN` → INA 0x47 window →
   `RB_VCC_GATE`; any out-of-window → drop `RB_PWR_EN`, latch fault.
4. `NOR_RST_N` HIGH before any SPI4 NOR access; SPI4 bus mutex + per-CS reconfig.
5. WDT kick only from supervisor after liveness AND-gate; `WDT_EN` last in bring-up.
6. Relay PA6 HIGH only when service quality met; any fault path drops it first.
7. INA228: SHUNT_CAL(4096×trim) re-applied after any device reset before readings are
   trusted; GPS INA at **0x4A**; SHT45 owns 0x44.
8. `GPS_TXRDY` trusted only after CFG-TXREADY remap ACK.
9. Fan PWM idle/fault state = full speed.
10. No timing-state mutex in management threads (seqlock snapshot only).
