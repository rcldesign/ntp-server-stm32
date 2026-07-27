# ---------------------------------------------------------------------------
# STS1000 "Meridian" — host unit-test harness (CMake + CTest + Unity).
#
# ZERO-TOUCH TEST REGISTRATION — adding a test needs no edit here.
#
#   * Drop `test_<module>.c` into this directory. It is discovered
#     automatically, built into its own executable, and registered with CTest
#     under the name `<module>`.
#
#   * A test binary compiles ONLY its own module's sources plus the transitive
#     closure of that module's declared dependencies. Nothing else. A module
#     that is mid-refactor and does not compile therefore breaks only the
#     suites that actually depend on it — everyone else's tests keep running.
#     Declare dependencies in the map below; that is the one edit a new module
#     needs, and only when its dependency set is not already listed.
#
#   * Shared helpers live in `support/`; every .c there is linked into every
#     test binary. `setUp`/`tearDown` come from there as weak no-ops.
#
#   * All globs use CONFIGURE_DEPENDS, so `cmake --build` re-runs the configure
#     step when a test, core source, or helper appears or disappears — including
#     the first time a module directory lands.
#
# Include path for every test target:
#   app/src/core        -> #include "util/crc.h", "quality/quality.h"
#   app/src             -> #include "port/port_time.h", "port/port_crypto.h"
#   tests/host/support  -> #include "test_support.h"
#
# Run with `scripts/test.sh` (all suites, or `TESTS='ntp ptp' scripts/test.sh`
# to build and run only yours); coverage gate with `scripts/coverage.sh`.
#
# See README.md in this directory for the full convention.
# ---------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.20)
project(sts1000_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

option(COVERAGE "Instrument the core and test sources for gcov" OFF)

enable_testing()

get_filename_component(FW_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(APP_SRC_DIR "${FW_ROOT}/app/src")
set(CORE_DIR "${APP_SRC_DIR}/core")

if(NOT IS_DIRECTORY "${CORE_DIR}")
  message(FATAL_ERROR "core source tree not found at ${CORE_DIR}")
endif()

# ---------------------------------------------------------------------------
# Module dependency map
#
# `module_deps(<module> [<direct dep> ...])`. Dependencies resolve transitively,
# so list only the direct ones — gnssmgr declares ubx, and util arrives through
# it.
#
# Entries exist for every planned module (ARCHITECTURE.md §5) before the module
# lands. A directory that does not exist yet contributes no sources and is
# skipped silently, so this map is safe to populate ahead of the code.
#
# A suite whose name has no entry here is an error the harness recovers from:
# it falls back to compiling all of core and CMake warns. That escape hatch
# exists so a typo or a brand-new module never blocks a test run — it is not a
# supported configuration, because it re-couples the suite to every module.
# ---------------------------------------------------------------------------

macro(module_deps mod)
  set(MODULE_DEPS_${mod} "${ARGN}")
endmacro()

module_deps(util)
module_deps(cfg      util)
module_deps(quality  util)
module_deps(logring  util)
module_deps(ubx      util)
module_deps(gnssmgr  ubx util)
module_deps(disc     util quality)
module_deps(refsel   util)
module_deps(thermal  util)
module_deps(ntp      util quality)
module_deps(nts      util)
module_deps(ptp      util quality)
module_deps(ina228   util)
module_deps(fault    util)
module_deps(pwrseq   ina228 util)
module_deps(mcp      util cfg logring quality)
module_deps(snmp     util quality)
module_deps(ui       util quality)
module_deps(atecc    util)
module_deps(auth     util)
# `cal` adds one edge to the ARCHITECTURE.md §4 map: `ina228`. The trim's
# arithmetic and the register's own 15-bit clamp belong to core/ina228
# (ina228_shunt_cal_trim); core/cal owns the plausibility policy on top of it,
# and reusing the codec keeps one implementation of the formula.
module_deps(cal      ina228 util)
# `mp` (the Maintenance Protocol, FMT device side) adds one edge to the
# ARCHITECTURE.md §4 map: `ina228`, for the nine-monitor rail table the
# capability manifest publishes (name, designator, address, shunt) rather than
# copying. It also includes ui/ui.h and port/port_image.h for *types only* — no
# ui symbol is referenced, so `ui` is deliberately not a link dependency and
# stays out of these suites' coverage.
module_deps(mp       util cfg logring quality ina228)

# A module may own several focused suites. Each suite name needs its own entry
# naming the module it belongs to. `util` is split across four because CRC,
# COBS, the ring buffer and the byte accessors have nothing to say to each
# other; `core/crc/` and friends do not exist and glob to nothing.
module_deps(crc      util)
module_deps(cobs     util)
module_deps(ring     util)
module_deps(bytes    util)
module_deps(aes_siv  nts)
module_deps(mcp_dfu  mcp util cfg logring quality)

# `glue_policy` covers the Zephyr glue's decision logic — the §8.3 self-confirm
# gate, the MCUboot staging-slot ceiling, the config-applier registry — which
# lives in dependency-free headers under app/src/zephyr precisely so it can be
# tested here. It compiles no core module, hence the empty dependency list.
module_deps(glue_policy)

# `ppscorr` is the same idea for the PPS <-> ETH-PTP correlation
# (app/src/zephyr/net/sts_ppscorr.h): the arithmetic that places the served
# timescale on TAI to a few hundred nanoseconds instead of tens of milliseconds.
# Every defect it can carry — a sign slip, a 19-second timescale error, a
# mishandled 32-bit TIM2 wrap — is silent at runtime and ships a grandmaster
# that is stable and wrong, which is exactly why it is Zephyr-free and tested
# here. It compiles no core module, hence the empty dependency list.
module_deps(ppscorr)

# `gnss_tee` is the same idea for the platform area's NMEA/UBX run classifier
# (app/src/zephyr/platform/gnss_tee.h): the split that decides which of the
# maintenance tool's two capture channels each received byte belongs to. It runs
# on the priority-6 GNSS thread, so it is written to be countable rather than
# clever, and every way it can be wrong is silent — a '$' inside a NAV-SAT
# payload misread as a sentence start does not fail, it just quietly corrupts a
# technician's capture. Zephyr-free by construction, hence the empty dependency
# list.
module_deps(gnss_tee)

# `ntp_keys` is the same idea for the net area's key reconciliation
# (app/src/zephyr/net/sts_ntp_keys.h): Zephyr-free by construction so the
# revocation decision can be tested here. It drives core/ntp's real key table
# rather than a model, because the defect it guards against lived exactly in the
# disagreement between the two — ntp_key_set() validates before it clears, so a
# rejected rotation left the previous key live.
module_deps(ntp_keys ntp util quality)

# The Wave-4c security cluster. `core/auth` holds the AAA chain plus the RADIUS,
# TACACS+ and LDAP codecs, so all four suites compile the same source set; the
# LDAP codec has no suite of its own and is covered from test_auth.c, next to
# the chain that drives it. `snmpv3` is the USM envelope over the same PDU layer
# the `snmp` suite already exercises, hence the `snmp` edge.
module_deps(radius   auth)
module_deps(tacacs   auth)
module_deps(snmpv3   snmp util quality)

# `mp` is split by layer: the frame codec, the JSON codec, the RPC dispatch, the
# override/guard/interlock engine, the manifest, the stream records and the
# panel mirror fail independently and are tested independently.
module_deps(mp_frame    mp)
module_deps(mp_json     mp)
module_deps(mp_rpc      mp)
module_deps(mp_override mp)
module_deps(mp_manifest mp)
module_deps(mp_stream   mp)
module_deps(mp_mirror   mp)

# `ptp` is split by layer: the wire codecs, the best-master algorithm and the
# port state machine are independent enough to fail independently.
# core/web (Wave 4b): the web management plane. One module directory, four
# focused suites — the HTTP parser, the REST router, the WebSocket codec and the
# session/auth layer fail independently and are fuzzed independently.
module_deps(web        util cfg logring quality)
module_deps(http_parse web)
module_deps(rest       web)
module_deps(wss        web)
module_deps(auth_web   web)

module_deps(ptp_msg  ptp)
module_deps(ptp_bmca ptp)
module_deps(ptp_port ptp)

# Profile descriptors/TLVs and the Annex-P AUTHENTICATION TLV. Both live in
# core/ptp/, so they arrive with the module rather than as new dependency edges.
module_deps(ptp_profile ptp)
module_deps(ptp_icv     ptp)

# `fwupd` is the multi-IC firmware-update orchestrator. Its three translation
# units — the orchestrator, the u-blox safeboot loader client and the FE-5680A
# client — all live in core/fwupd/, so each suite names the same module set. The
# `ubx` edge is real: ubx_fwupd.c frames UBX and reads back UBX-MON-VER.
module_deps(fwupd     ubx util)
# The two transport suites name `fwupd` so the core/fwupd/ directory is globbed;
# `fwupd` in turn carries the ubx and util edges.
module_deps(ubx_fwupd fwupd)
module_deps(rb_fwupd  fwupd)

# The polar skyplot renderer lives in core/ui/ and consumes ui_sv_t.
module_deps(skyplot ui)

# ---------------------------------------------------------------------------
# Dependency resolution
# ---------------------------------------------------------------------------

# Transitive closure of `root`'s dependencies, including `root` itself.
# Iterative worklist rather than recursion: this terminates on a cyclic map
# instead of recursing until CMake dies.
function(resolve_deps out_var root)
  set(_pending "${root}")
  set(_seen "")

  while(NOT "${_pending}" STREQUAL "")
    list(POP_FRONT _pending _cur)
    if(_cur IN_LIST _seen)
      continue()
    endif()
    list(APPEND _seen "${_cur}")

    if(DEFINED MODULE_DEPS_${_cur})
      foreach(_dep IN LISTS MODULE_DEPS_${_cur})
        if(NOT _dep IN_LIST _seen)
          list(APPEND _pending "${_dep}")
        endif()
      endforeach()
    endif()
  endwhile()

  set(${out_var} "${_seen}" PARENT_SCOPE)
endfunction()

# Sources for one test binary, plus the module directories that supplied them.
#
# The glob is unconditional: a directory that does not exist yet yields nothing,
# and globbing it anyway registers it with CONFIGURE_DEPENDS so the build
# reconfigures itself the moment the module lands.
function(module_sources out_srcs out_mods module)
  resolve_deps(_mods "${module}")
  set(_srcs "")
  set(_present "")

  foreach(_m IN LISTS _mods)
    file(GLOB_RECURSE _found CONFIGURE_DEPENDS "${CORE_DIR}/${_m}/*.c")
    if(_found)
      list(SORT _found)
      list(APPEND _srcs ${_found})
      list(APPEND _present "${_m}")
    endif()
  endforeach()

  set(${out_srcs} "${_srcs}" PARENT_SCOPE)
  set(${out_mods} "${_present}" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------
# Interface targets
# ---------------------------------------------------------------------------
# Warnings are an error for first-party code (ARCHITECTURE.md §9). Vendored
# Unity is deliberately excluded — third-party warning cleanliness is not a
# gate on this project.
add_library(sts_warnings INTERFACE)
target_compile_options(sts_warnings INTERFACE -Wall -Wextra -Werror)

add_library(sts_coverage INTERFACE)
if(COVERAGE)
  # -O0 keeps line attribution honest; without it gcov reports folded lines.
  target_compile_options(sts_coverage INTERFACE --coverage -O0 -g)
  target_link_options(sts_coverage INTERFACE --coverage)
endif()

# ---------------------------------------------------------------------------
# Vendored Unity
# ---------------------------------------------------------------------------
add_library(unity STATIC "${CMAKE_CURRENT_LIST_DIR}/unity/unity.c")
target_include_directories(unity PUBLIC "${CMAKE_CURRENT_LIST_DIR}/unity")
target_compile_options(unity PRIVATE -Wall)

# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS "${CORE_DIR}/*.c")
file(GLOB SUPPORT_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/support/*.c")
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/test_*.c")

list(SORT CORE_SOURCES)
list(SORT SUPPORT_SOURCES)
list(SORT TEST_SOURCES)

if(NOT CORE_SOURCES)
  message(FATAL_ERROR "no core sources found under ${CORE_DIR}")
endif()
if(NOT TEST_SOURCES)
  message(FATAL_ERROR "no test_*.c found in ${CMAKE_CURRENT_LIST_DIR}")
endif()

# Census of module directories that currently hold sources. Used below to spot
# modules that no test binary compiles.
set(CORE_MODULE_DIRS "")
foreach(_src IN LISTS CORE_SOURCES)
  file(RELATIVE_PATH _rel "${CORE_DIR}" "${_src}")
  if(_rel MATCHES "^([^/]+)/")
    if(NOT CMAKE_MATCH_1 IN_LIST CORE_MODULE_DIRS)
      list(APPEND CORE_MODULE_DIRS "${CMAKE_MATCH_1}")
    endif()
  endif()
endforeach()

list(LENGTH CORE_SOURCES _n_core)
list(LENGTH TEST_SOURCES _n_test)
message(STATUS
  "sts1000 host tests: ${_n_test} suite(s), ${_n_core} core source(s), coverage=${COVERAGE}")

# ---------------------------------------------------------------------------
# One executable + one CTest entry per test_*.c
# ---------------------------------------------------------------------------
set(COMPILED_MODULE_DIRS "")

foreach(test_src IN LISTS TEST_SOURCES)
  get_filename_component(test_target "${test_src}" NAME_WE)      # test_crc
  string(REGEX REPLACE "^test_" "" suite_name "${test_target}")  # crc

  if(DEFINED MODULE_DEPS_${suite_name})
    module_sources(_srcs _mods "${suite_name}")
  else()
    message(WARNING
      "tests/host: no module_deps() entry for '${suite_name}', so "
      "${test_target} falls back to compiling all of core. Add "
      "`module_deps(${suite_name} <deps>)` to this file, otherwise an "
      "unrelated module's build error will break this suite.")
    set(_srcs ${CORE_SOURCES})
    set(_mods ${CORE_MODULE_DIRS})
  endif()

  add_executable(${test_target} "${test_src}" ${SUPPORT_SOURCES} ${_srcs})
  target_include_directories(${test_target} PRIVATE
    "${CORE_DIR}"
    "${APP_SRC_DIR}"
    "${CMAKE_CURRENT_LIST_DIR}/support")
  target_link_libraries(${test_target} PRIVATE unity sts_warnings sts_coverage)

  add_test(NAME ${suite_name} COMMAND ${test_target})
  set_tests_properties(${suite_name} PROPERTIES TIMEOUT 120)

  if(_mods)
    string(REPLACE ";" " " _mods_pretty "${_mods}")
    list(APPEND COMPILED_MODULE_DIRS ${_mods})
  else()
    set(_mods_pretty "(no core sources yet)")
  endif()
  message(STATUS "  ${suite_name} <- ${_mods_pretty}")
endforeach()

# ---------------------------------------------------------------------------
# Coverage-gate visibility
#
# The gate measures every core file present in the coverage data, which is the
# union of what the test binaries compile. A module that no binary compiles is
# not measured at 0 % — it is absent, and the gate cannot see it. That is the
# one thing per-module compilation gives up relative to building all of core
# into every suite, so name the gap at configure time rather than letting the
# percentage quietly describe a shrinking fraction of the tree.
# ---------------------------------------------------------------------------
if(COMPILED_MODULE_DIRS)
  list(REMOVE_DUPLICATES COMPILED_MODULE_DIRS)
endif()

set(UNMEASURED_MODULE_DIRS ${CORE_MODULE_DIRS})
if(COMPILED_MODULE_DIRS AND UNMEASURED_MODULE_DIRS)
  list(REMOVE_ITEM UNMEASURED_MODULE_DIRS ${COMPILED_MODULE_DIRS})
endif()

if(UNMEASURED_MODULE_DIRS)
  string(REPLACE ";" ", " _unmeasured "${UNMEASURED_MODULE_DIRS}")
  message(WARNING
    "tests/host: core module(s) compiled into no test binary: ${_unmeasured}. "
    "They contribute no coverage records at all, so the ARCHITECTURE.md §9 "
    "gate cannot see them — it is not counting them as 0 %. Add "
    "tests/host/test_<mod>.c for each.")
endif()
