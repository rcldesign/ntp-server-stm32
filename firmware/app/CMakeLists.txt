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
# mbedTLS user-config header (NTS-KE)
#
# CONFIG_MBEDTLS_USER_CONFIG_FILE="sts_mbedtls_user.h" makes Zephyr's
# configs/config-mbedtls.h end with `#include "sts_mbedtls_user.h"` (that file,
# line 552). The header therefore has to be findable from every translation unit
# that pulls the mbedTLS configuration in — which is the whole mbedTLS module,
# not just this application, because the module is compiled with its own include
# paths and knows nothing about app/. Without this line the mbedTLS library
# fails to compile with "sts_mbedtls_user.h: No such file or directory"; with it,
# MBEDTLS_SSL_KEYING_MATERIAL_EXPORT reaches the library and the application
# identically, which is the only way the exporter can be both compiled into
# ssl_tls.c and visible to the `#if` in src/zephyr/net/sts_ntske.c.
#
# PLACEMENT: this must come AFTER find_package(Zephyr), not before it, and the
# note that used to say otherwise (in sts_mbedtls_user.h and conf/net.conf) was
# wrong — `zephyr_include_directories` is defined by cmake/modules/extensions.cmake
# and operates on the `zephyr_interface` target, and neither exists until
# find_package(Zephyr) runs kernel.cmake's add_subdirectory(${ZEPHYR_BASE}).
# Placed earlier the configure step dies with
#     CMake Error: Unknown CMake command "zephyr_include_directories".
# Placing it after is not a workaround: zephyr_include_directories() appends to
# an INTERFACE target's INTERFACE_INCLUDE_DIRECTORIES, and CMake resolves usage
# requirements at generate time, so every zephyr_library() — including the four
# the mbedTLS module defines — picks it up even though those targets were
# created during find_package(Zephyr).
#
# Scope note: this puts src/zephyr/net/ on the include path of every Zephyr
# library in the image, not only mbedTLS. That is safe because every header in
# that directory is `sts_`-prefixed, so none can shadow a Zephyr, mbedTLS or
# libc header. Keep it that way — an unprefixed header added there would be
# picked up by upstream code.
# ---------------------------------------------------------------------------
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/zephyr/net)

# ---------------------------------------------------------------------------
# Anti-rollback: the SIGNED security counter must equal the COMPILED-IN epoch
#
# Two symbols carry one number (app/conf/rollback.conf explains why Kconfig
# cannot express it as one):
#
#   CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS  reaches imgtool and lands in the image's
#                                      protected TLV area, which is what
#                                      MCUboot compares.
#   CONFIG_STS1000_SECURITY_EPOCH      reaches the C code, which uses it to
#                                      refuse a staged downgrade and to step
#                                      the ATECC608B witness counter.
#
# Letting those drift is the interesting failure: the image would be gated on
# one number and the firmware would reason about another, and both halves would
# look fine in isolation. So this is a hard error, not a warning. It also
# catches the case the whole feature is silently absent — a fragment that got
# dropped, or an -DEXTRA_CONF_FILE that replaced rather than extended the set —
# because then the imgtool argument is empty and no counter is signed at all.
#
# Only meaningful for an image that MCUboot chain-loads and that is actually
# signed; a raw or unsigned build has no TLV area to put a counter in.
# ---------------------------------------------------------------------------
if(CONFIG_BOOTLOADER_MCUBOOT AND NOT CONFIG_MCUBOOT_GENERATE_UNSIGNED_IMAGE)
  if(NOT CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS MATCHES "--security-counter[ \t]+([0-9]+|0[xX][0-9a-fA-F]+)")
    message(FATAL_ERROR
      "anti-rollback: CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS carries no "
      "'--security-counter <n>', so this image would be signed without an "
      "IMAGE_TLV_SEC_CNT and could never be installed on a unit that has "
      "already confirmed a counter-bearing image. Restore app/conf/rollback.conf.\n"
      "  CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS = '${CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS}'")
  endif()
  set(sts1000_signed_epoch "${CMAKE_MATCH_1}")

  if(NOT DEFINED CONFIG_STS1000_SECURITY_EPOCH)
    message(FATAL_ERROR
      "anti-rollback: CONFIG_STS1000_SECURITY_EPOCH is not defined, but the "
      "image is being signed with --security-counter ${sts1000_signed_epoch}. "
      "The firmware would have no idea what epoch it is running.")
  endif()

  # EQUAL, not STREQUAL: 0x2 and 2 are the same epoch, and imgtool accepts both.
  if(NOT sts1000_signed_epoch EQUAL CONFIG_STS1000_SECURITY_EPOCH)
    message(FATAL_ERROR
      "anti-rollback: the epoch signed into the image and the epoch compiled "
      "into it disagree. Both live in app/conf/rollback.conf and must be the "
      "same number.\n"
      "  signed   (--security-counter)      = ${sts1000_signed_epoch}\n"
      "  compiled (CONFIG_STS1000_SECURITY_EPOCH) = ${CONFIG_STS1000_SECURITY_EPOCH}")
  endif()

  message(STATUS
    "sts1000 anti-rollback: security epoch ${CONFIG_STS1000_SECURITY_EPOCH} "
    "(signed into IMAGE_TLV_SEC_CNT and compiled in)")
endif()

# ---------------------------------------------------------------------------
# Sources
#
# ARCHITECTURE.md §4 splits the application in two:
#   src/core/    platform-neutral C11, host-testable, no Zephyr headers
#   src/zephyr/  Zephyr glue (threads, ISRs, driver + port implementations)
#
# core/ is one recursive glob. During the wave-2 build-out it was listed per
# module so a half-written module could not break an unrelated build; every
# ARCHITECTURE.md §5 module now compiles for the target, so that exception is
# gone and adding a module is a no-edit operation.
# ---------------------------------------------------------------------------
file(GLOB_RECURSE sts1000_core_sources CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.c)

# --- Zephyr glue, gated per ownership area ---------------------------------
#
# The glue is four ownership areas developed in parallel (ARCHITECTURE.md §2):
# platform, net, console, ui. Each area's sources are globbed from its own
# subdirectory, so adding a file to an area is still a no-edit operation — the
# STEP-0 goal. The three non-platform areas are compiled only when their
# CONFIG_STS1000_<AREA> switch is on (all default y).
#
# Gating the compile — not just the entry-point call in main.c — is what makes
# the switches actually compose the image: CONFIG_STS1000_NET=n produces a
# timing-only bring-up image with no network code linked (the motivation in
# app/Kconfig), and it lets one area be excluded while it is mid-refactor
# without editing another area's files. main.c's __weak fallbacks
# (src/zephyr/platform/sts_area_weak.c) stand in for a disabled area's entry
# point, so the image always links.
#
#   platform  src/zephyr/*.c (sts_app.c) + src/zephyr/platform/  — always built
#   net       src/zephyr/net/                                    — CONFIG_STS1000_NET
#   console   src/zephyr/console/ + src/zephyr/storage/          — CONFIG_STS1000_CONSOLE
#   ui        src/zephyr/ui/                                     — CONFIG_STS1000_UI
#
# The four area directories are fixed by ARCHITECTURE.md §2; a fifth would be a
# structural change that warrants editing this file.
file(GLOB sts1000_glue_platform_top CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/src/zephyr/*.c)
file(GLOB_RECURSE sts1000_glue_platform CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/src/zephyr/platform/*.c)
set(sts1000_glue_sources ${sts1000_glue_platform_top} ${sts1000_glue_platform})

if(CONFIG_STS1000_NET)
  file(GLOB_RECURSE sts1000_glue_net CONFIGURE_DEPENDS
       ${CMAKE_CURRENT_SOURCE_DIR}/src/zephyr/net/*.c)
  list(APPEND sts1000_glue_sources ${sts1000_glue_net})
endif()

if(CONFIG_STS1000_CONSOLE)
  file(GLOB_RECURSE sts1000_glue_console CONFIGURE_DEPENDS
       ${CMAKE_CURRENT_SOURCE_DIR}/src/zephyr/console/*.c
       ${CMAKE_CURRENT_SOURCE_DIR}/src/zephyr/storage/*.c)
  # The Maintenance Protocol is the one sub-area with a switch of its own
  # (src/zephyr/console/Kconfig.mp). It is an unauthenticated-to-role-`none`
  # physical-access surface, so CONFIG_STS1000_MP=n has to actually remove it
  # from the image rather than leave it compiled and merely unused — which
  # means naming its two files here, because the area glob above would
  # otherwise pull them back in. Their call sites resolve to the __weak no-ops
  # in src/zephyr/platform/sts_area_weak.c.
  if(NOT CONFIG_STS1000_MP)
    list(REMOVE_ITEM sts1000_glue_console
         ${CMAKE_CURRENT_SOURCE_DIR}/src/zephyr/console/mp_glue.c
         ${CMAKE_CURRENT_SOURCE_DIR}/src/zephyr/console/mp_tunnel.c)
  endif()
  list(APPEND sts1000_glue_sources ${sts1000_glue_console})
endif()

if(CONFIG_STS1000_UI)
  file(GLOB_RECURSE sts1000_glue_ui CONFIGURE_DEPENDS
       ${CMAKE_CURRENT_SOURCE_DIR}/src/zephyr/ui/*.c)
  list(APPEND sts1000_glue_sources ${sts1000_glue_ui})
endif()

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
