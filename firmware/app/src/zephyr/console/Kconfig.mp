# STS1000 "Meridian" — Maintenance Protocol tunables (console area).
# SPDX-License-Identifier: Apache-2.0
#
# NOT sourced by app/Kconfig yet: adding the `rsource` line is a one-line edit to
# a file this change does not own. Every symbol below has an #if defined()
# fallback in mp_glue.c, so the area builds correctly without it — except
# STS1000_MP itself, which is a bool defaulting to y and therefore cannot be
# defaulted by the preprocessor. Until app/Kconfig sources this file, define
# CONFIG_STS1000_MP through conf/mp.conf or the command line.
#
#     rsource "src/zephyr/console/Kconfig.mp"

if STS1000_CONSOLE

config STS1000_MP
	bool "Maintenance Protocol (Field Maintenance Tool)"
	default y
	help
	  Builds src/zephyr/console/mp_glue.c and mp_tunnel.c and registers the
	  `mp` shell command. The protocol itself is core/mp, which is always
	  compiled; this switch only controls whether anything drives it.

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

endif # STS1000_MP

endif # STS1000_CONSOLE
