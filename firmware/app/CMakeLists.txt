# STS1000 "Meridian" application — SPDX-License-Identifier: Apache-2.0
#
# Build:  west build -b sts1000_meridian --sysbuild firmware/app
# See firmware/scripts/build.sh for the canonical invocation.

cmake_minimum_required(VERSION 3.20.0)

# ---------------------------------------------------------------------------
# Drop-in configuration fragments  (MUST precede find_package(Zephyr))
#
# The Zephyr glue is split into four ownership areas (platform, net, console,
# ui) worked on in parallel. So that adding an area never means editing a
# shared file, each area drops its own Kconfig fragment into `conf/` and its
# own devicetree overlay into `dts/`; both directories are globbed here and
# merged into EXTRA_CONF_FILE / EXTRA_DTC_OVERLAY_FILE.
#
# Ordering is the sorted filename, so a fragment can rely on being applied
# after an alphabetically earlier one. `prj.conf` is always applied first
# (Zephyr applies CONF_FILE before EXTRA_CONF_FILE), and a later fragment
# overrides an earlier one — that is the intended override direction.
#
# configuration_files.cmake reads both variables with `zephyr_get(... MERGE)`,
# which unions the local scope value set here with any CACHE/env/sysbuild
# value, so a `-DEXTRA_CONF_FILE=...` on the west command line still applies.
#
# CONFIGURE_DEPENDS makes CMake re-glob on every build, so a newly added
# fragment is picked up without a manual `cmake` re-run. Changing an already
# merged Kconfig default still needs a pristine build.
# ---------------------------------------------------------------------------
file(GLOB sts1000_conf_fragments CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.conf)
if(sts1000_conf_fragments)
  list(SORT sts1000_conf_fragments)
  list(APPEND EXTRA_CONF_FILE ${sts1000_conf_fragments})
endif()

file(GLOB sts1000_dts_overlays CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/dts/*.overlay)
if(sts1000_dts_overlays)
  list(SORT sts1000_dts_overlays)
  list(APPEND EXTRA_DTC_OVERLAY_FILE ${sts1000_dts_overlays})
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sts1000_meridian LANGUAGES C)

# ---------------------------------------------------------------------------
# Sources
#
# ARCHITECTURE.md §4 splits the application in two:
#   src/core/    platform-neutral C11, host-testable, no Zephyr headers
#   src/zephyr/  Zephyr glue (threads, ISRs, driver + port implementations)
#
# Both are recursive globs. During the wave-2 build-out core/ was listed per
# module so a half-written module could not break an unrelated build; every
# ARCHITECTURE.md §5 module now compiles for the target, so that exception is
# gone and adding a module (or an area's glue file) is a no-edit operation.
# ---------------------------------------------------------------------------
file(GLOB_RECURSE sts1000_core_sources CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.c)

file(GLOB_RECURSE sts1000_glue_sources CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/src/zephyr/*.c)

target_sources(app PRIVATE
  src/main.c
  ${sts1000_core_sources}
  ${sts1000_glue_sources}
)

# Include roots:
#   src/        the cross-area API as "zephyr/sts_app.h", and an area's headers
#               with the zephyr/ prefix ("zephyr/platform/platform.h")
#   src/core/   core modules include each other as "util/crc.h" — the same root
#               tests/host/CMakeLists.txt uses, which is what keeps a core module
#               compiling identically on the host and on the target
#   src/port/   bare "port_time.h" for glue that implements a port
#   src/zephyr/ an area's own headers, area-relative ("console/sts_console.h",
#               "platform/platform.h"). Both this and the zephyr/-prefixed form
#               from src/ resolve; the four glue areas are authored in parallel
#               and settled on the area-relative style, so the glue root is
#               carried explicitly rather than forcing every include to be
#               rewritten. It adds src/zephyr/<area>/ as search dirs, none of
#               which collides with a <zephyr/...> path (those live under the
#               toolchain's zephyr/include, and there is no src/zephyr/zephyr/).
#
# NOTE: having src/ on the include path means <zephyr/...> would resolve against
# src/zephyr/ first if such a header existed. Do not create a header directly
# under src/zephyr/ whose name matches a Zephyr header (device.h, kernel.h, ...)
# — glue headers belong in src/zephyr/<area>/ per ARCHITECTURE.md §2.
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core
  ${CMAKE_CURRENT_SOURCE_DIR}/src/port
  ${CMAKE_CURRENT_SOURCE_DIR}/src/zephyr
)

# ARCHITECTURE.md §9: zero warnings from application code.
target_compile_options(app PRIVATE -Wall -Wextra)
