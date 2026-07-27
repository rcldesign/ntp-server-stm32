# STS1000 "Meridian" — Maintenance Protocol tunables (console area).
# SPDX-License-Identifier: Apache-2.0
#
# Sourced by app/Kconfig:
#
#     rsource "src/zephyr/console/Kconfig.mp"
#
# STS1000_MP is a switch, not documentation. app/CMakeLists.txt drops
# src/zephyr/console/mp_glue.c and mp_tunnel.c from the build when it is n, and
# src/zephyr/platform/sts_area_weak.c carries a __weak no-op for every function
# console/mp_glue.h declares, so the image still links: sts_console.c calls
# sts_mp_start()/sts_mp_tick() unconditionally and platform/gnss.c calls the
# byte tees from the USART3 receive path.
#
# It exists because MP is a physical-access surface that answers at role `none`
# before any credential is presented (FMT §5.3), so an operator deploying a
# board where the USB port is not physically controlled has to be able to take
# it out of the image rather than merely leave it unused.
#
# The tunables keep their #if defined() fallbacks in mp_glue.c/mp_tunnel.c — the
# console-area convention — and every fallback equals the default here, so
# sourcing this file changes no value.

if STS1000_CONSOLE

config STS1000_MP
	bool "Maintenance Protocol (Field Maintenance Tool)"
	default y
	help
	  Builds src/zephyr/console/mp_glue.c and mp_tunnel.c and registers the
	  `mp` shell command. The protocol itself is core/mp, which is always
	  compiled; this switch only controls whether anything drives it — and
	  with nothing driving it --gc-sections discards core/mp from the image.

	  Say n to remove the maintenance surface: `mp enter` disappears, the
	  GNSS/rubidium passthrough tunnels cannot be opened, and the NMEA/UBX
	  tees stop being fed (their call sites resolve to the __weak no-ops in
	  src/zephyr/platform/sts_area_weak.c).

if STS1000_MP

config STS1000_MP_FW_VERSION
	string "Firmware version string reported by MP hello"
	default "0.1.0-dev"

config STS1000_MP_SCRATCH
	int "MP record-assembly scratch (bytes)"
	default 4096
	range 3072 16384
	help
	  Holds one mirror keyframe (MP_MIRROR_MAX = 3072 B for the as-built
	  60x20 panel), one log batch, or one diag.snapshot support bundle. A
	  build assertion enforces the mirror minimum.

config STS1000_MP_REASM
	int "MP inbound reassembly pool (bytes, split across 2 slots)"
	default 3072
	range 1024 16384
	help
	  Fragmented inbound messages are reassembled here, two channels at a
	  time. A cfg.import chunk and a full JSON-RPC request both fit in half
	  of the default.

config STS1000_MP_TEE_RING
	int "MP passthrough tee staging ring (bytes, power of two)"
	default 1024
	range 256 8192
	help
	  Bytes buffered between the byte producers and the drain on
	  sts_mp_tick(). The producers are the GNSS thread (priority 6, on the
	  timing path's critical route) and the UART7 receive ISR, neither of
	  which may take the engine mutex or push a COBS frame down the console
	  UART; they do a bounded copy into this ring and the console supervisor
	  frames it 250 ms later.

	  Size it for one drain period of the fastest producer. The as-built
	  UBX message set is a few hundred bytes per second and the FE-5680A
	  link is 9600 8N1 (240 B per period), so the default holds roughly four
	  periods of both together. Overflow drops the newest bytes and counts
	  them in `mp status`; a tee is best-effort passthrough by nature.

	  Must be a power of two (core/util ring_init()); a build assertion
	  enforces it.

endif # STS1000_MP

endif # STS1000_CONSOLE
