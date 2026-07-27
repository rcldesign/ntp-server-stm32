# STS1000 "Meridian" application — SPDX-License-Identifier: Apache-2.0
#
# Build:  west build -b sts1000_meridian --sysbuild firmware/app
# See firmware/scripts/build.sh for the canonical invocation.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sts1000_meridian LANGUAGES C)

# ---------------------------------------------------------------------------
# Sources
#
# ARCHITECTURE.md §4 splits the application in two:
#   src/core/    platform-neutral C11, host-testable, no Zephyr headers
#   src/zephyr/  Zephyr glue (threads, ISRs, driver + port implementations)
#
# core/ is listed PER MODULE, deliberately — not with a recursive glob. The §5
# modules land across several waves and are written in parallel, so a recursive
# glob would drag a half-written module into the target link and break a build
# that has nothing to do with it. Adding a module is a one-line edit here.
# ---------------------------------------------------------------------------

# --- core/ modules: add one file(GLOB ...) + list(APPEND ...) pair per module
#     as each ARCHITECTURE.md §5 module lands (WAVE 3 EXTENSION POINT) --------
set(sts1000_core_sources)

file(GLOB sts1000_core_util_sources CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/src/core/util/*.c)
list(APPEND sts1000_core_sources ${sts1000_core_util_sources})

# WAVE 3 EXTENSION POINT — append the remaining ARCHITECTURE.md §5 modules here
# in the same shape, e.g.:
#
#   file(GLOB sts1000_core_cfg_sources CONFIGURE_DEPENDS
#        ${CMAKE_CURRENT_SOURCE_DIR}/src/core/cfg/*.c)
#   list(APPEND sts1000_core_sources ${sts1000_core_cfg_sources})
#
# Remaining modules: cfg quality disc refsel ubx gnssmgr ntp nts ptp ina228
#                    fault pwrseq thermal mcp logring snmp ui
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# TEMPORARY EXCLUSION — src/core/util/crc.c
#
# That file exports crc32_ieee(), crc32_ieee_update(), crc16_ccitt() and
# crc16_ccitt_update(): the exact global symbol names of Zephyr's own
# lib/crc (<zephyr/sys/crc.h>). CONFIG_CRC is hard-`select`ed by the SHT4X
# driver, which the I2C device map (interface ref §4, U72 at 0x44) requires, so
# both objects are always pulled into the link:
#
#   ld.bfd: zephyr/libzephyr.a(crc32_sw.c.obj): in function 'crc32_ieee':
#   multiple definition of 'crc32_ieee'; app/libapp.a(crc.c.obj): first defined
#
# core/util owns those symbols and must prefix them (sts_crc32_ieee, ...); that
# file and its host test are outside this change's boundary. Until the rename
# lands, the module is excluded from the TARGET image only — tests/host builds
# core/ directly, so unit coverage is unaffected.
# ---------------------------------------------------------------------------
list(REMOVE_ITEM sts1000_core_sources
     ${CMAKE_CURRENT_SOURCE_DIR}/src/core/util/crc.c)

# --- Zephyr glue: recursive glob is safe here, this tree is owned by the
#     platform waves and every file in it is expected to compile. -------------
file(GLOB_RECURSE sts1000_glue_sources CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/src/zephyr/*.c)

target_sources(app PRIVATE
  src/main.c
  ${sts1000_core_sources}
  ${sts1000_glue_sources}
)

# Include roots:
#   src/       "port/port_time.h", "zephyr/threads/threads.h"
#   src/core/  core modules include each other as "util/crc.h" — the same root
#              tests/host/CMakeLists.txt uses, which is what keeps a core module
#              compiling identically on the host and on the target
#   src/port/  bare "port_time.h" for glue that implements a port
#
# NOTE: having src/ on the include path means <zephyr/...> resolves against
# src/zephyr/ first. Do not create a header directly under src/zephyr/ whose
# name matches a Zephyr header (device.h, kernel.h, ...) — glue headers belong
# in src/zephyr/{threads,drv,portz}/ per ARCHITECTURE.md §2.
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core
  ${CMAKE_CURRENT_SOURCE_DIR}/src/port
)

# ARCHITECTURE.md §9: zero warnings from application code.
target_compile_options(app PRIVATE -Wall -Wextra)
