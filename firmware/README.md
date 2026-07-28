# STS1000 "Meridian" — Firmware

GPS-disciplined Stratum-1 NTP/NTS/PTP grandmaster firmware for the STM32H563ZIT6.
Zephyr RTOS + MCUboot, custom board definition, platform-neutral core logic with
host unit tests.

- **Structure contract:** [`ARCHITECTURE.md`](ARCHITECTURE.md) — module boundaries, flash
  map, MCP console protocol, thread wiring, coverage policy. Read this first.
- **Behavior spec:** [`../docs/ntp_server_software_spec.md`](../docs/ntp_server_software_spec.md)
- **Pin contract:** [`../docs/sts1000_firmware_hardware_interface.md`](../docs/sts1000_firmware_hardware_interface.md)

## Quick start

```sh
# Workspace bootstrap (workspace root = this repo's parent)
cd <workspace>
west init -l ntp-server-stm32 --mf firmware/west.yml
west update --narrow -o=--depth=1
west zephyr-export

# Build MCUboot + signed application
firmware/scripts/build.sh -p

# Host unit tests + coverage gate
firmware/scripts/test.sh
firmware/scripts/coverage.sh
```

Requires Zephyr SDK 0.17.2 (`arm-zephyr-eabi`); Zephyr v4.2.2 is pinned in `west.yml`.

## Current build figures

Measured on a pristine sysbuild with NTS-KE enabled. Absolute bytes, not
rounded KB — re-measure rather than trust this table, which has been stale
before by a margin that mattered: it read 46 % / 45 % while the image was at
81 % / 81 %, the difference between "plenty of room" and "budget the next
feature".

| Image | FLASH | RAM |
|---|---|---|
| Application (slot 0) | 742,600 B / 900,970 B (**82.4 %**) | 563,200 B / 655,360 B (**85.9 %**) |
| MCUboot | 51,068 B / 131,072 B (39.0 %) | — |

The application's FLASH region is 900,970 B, not the full 896 KiB slot0
(917,504 B): MCUboot's header and image trailer take the difference. Against
raw slot0 the application is **80.9 %**.

Enabling NTS-KE cost **+5,040 B flash** and **+29,320 B SRAM** — measured as
the difference between this image and the one immediately before it
(737,560 B / 533,880 B), both pristine builds of the same tree. The SRAM is
mostly two lines: `MBEDTLS_HEAP_SIZE` 32768 → 49152 in `app/conf/web.conf` (a
fourth concurrent TLS 1.3 session) and the listener's 8 KiB thread stack.
Raising `STS_LIVENESS_MAX` 12 → 20 in `app/src/zephyr/sts_app.c` adds a further
72 B of SRAM on top.

Host tests: **69 suites, all passing**. Core line coverage: **96 %** (gate: 80 %).

**Both budgets are tight enough to plan around.**

- *RAM.* The largest single discretionary allocation is the skyplot's 224×224
  indexed canvas at 50 KiB (7.8 % of SRAM), held permanently for a page that is
  on screen only when someone has walked the panel to it. `STS_UI_SKY_MAX_SIDE`
  in `src/zephyr/ui/sts_ui.h` is the one knob; band-rendering or nibble-packing
  the canvas would recover ~42 KiB or ~25 KiB respectively, at the cost of
  touching golden-image-tested code in `core/ui/skyplot.c`.
- *The web response buffer.* `STS_WEB_RESP_SIZE` is 8192 B and the worst-case
  telemetry frame is now **6657 B** — 1535 B of headroom, pinned by an assertion
  in `tests/host/test_rest.c`. The next field added to any REST group should
  check it rather than assume.

## Layout

```
west.yml                  pinned manifest (Zephyr + module allowlist)
boards/rcldesign/sts1000_meridian/   custom board: DTS, pinctrl, defconfig
app/
  src/core/               platform-neutral C11 logic — host-tested, no Zephyr headers
  src/port/               port interfaces (time, crypto, flash/image, store)
  src/zephyr/             Zephyr glue in four areas: platform, net, console, ui
  conf/, dts/             per-area Kconfig fragments and DT overlays (auto-merged)
  keys/                   DEV-ONLY MCUboot signing key (production key is offline)
tests/host/               Unity + CTest suites, one per core module
tools/meridian_ctl.py     reference PC client (config, telemetry, logs, firmware upload)
scripts/                  build.sh, test.sh, coverage.sh
```

## Flash map

| Partition | Offset | Size | Contents |
|---|---|---|---|
| `boot_partition` | 0x000000 | 128 KB | MCUboot (+ USB serial recovery) |
| `slot0_partition` | 0x020000 | 896 KB | active application (signed) |
| `slot1_partition` | 0x100000 | 896 KB | staged application (DFU target) |
| `storage_partition` | 0x1E0000 | 64 KB | NVS: config, PFI fast-save, calibration |

External 32 MB SPI-NOR carries LittleFS `/lfs` (log spool, bulk config, caches) and is
not required to boot.

## Firmware update

**Normal (application running)** — the PC tool speaks the binary MCP protocol on the
second USB CDC-ACM port:

```sh
tools/meridian_ctl.py --port /dev/ttyACM1 fw-info
tools/meridian_ctl.py --port /dev/ttyACM1 --password <pw> fw-upload build/app/zephyr/zephyr.signed.bin
tools/meridian_ctl.py --port /dev/ttyACM1 --password <pw> fw-confirm     # or fw-revert
```

Uploads are chunked, idempotent and resumable; the image is SHA-256 verified in flash
before the slot is marked pending. The new image boots as *test* and auto-reverts unless
the firmware self-confirms after its health gate passes.

**Recovery (unbootable image)** — MCUboot serial recovery over USB:

1. Hold **BUTTON_1 (PF0)**, reset/power-cycle, keep holding ≥ 1 s.
2. The device enumerates as "STS1000 Meridian recovery".
3. `mcumgr -c <conn> image upload zephyr.signed.bin`

**First load / bring-up** — ST-LINK over SWD (`west flash`).

## Console

Two USB CDC-ACM ports: **ACM0** is the human shell (Zephyr shell + live logs, `sts status`,
`sts quality`, `sts alarms`, `sts log tail`, `sts cfg`, `sts fw`, `sts diag`); **ACM1** is
the binary MCP channel for `meridian_ctl.py` (full configuration, real-time telemetry
streaming, log tail, firmware upload). Both offer the same data; the protocol is specified
in `ARCHITECTURE.md` §7 and `app/src/core/mcp/mcp_wire.h`.

## Testing

`app/src/core/**` is platform-neutral and fully host-testable — that is where the
correctness-critical logic lives (timing loop, protocol codecs, state machines, power
sequencing). `scripts/coverage.sh` enforces ≥ 80 % line coverage over it; the tree is
currently at 96 % (24,579 of 25,552 lines). Protocol modules are pinned to published test vectors (RFC 5297
AES-SIV, RFC 4493 CMAC, RFC 4231 HMAC, FIPS-197/180-4, IEEE 1588 §13 layouts, u-blox
UBX frames) rather than self-round-trips.

Hardware-dependent behavior (bring-up sequencing against real rails, PPS capture, PHY
link, display) is exercised by the target build and awaits the HIL rig; constants needing
bench characterization are marked `BENCH` in the relevant `*_cfg_t`.
