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
# `mp` (the Maintenance Protocol, FMT device side) adds two edges to the
# ARCHITECTURE.md §4 map. `ina228`, for the nine-monitor rail table the
# capability manifest publishes (name, designator, address, shunt) rather than
# copying; and `fwupd`, because FMT §3.2 puts the six `fw.*` methods on the MP
# control plane — mp_rpc.c drives the orchestrator through the mp_fwupd_t port
# and renders its state/component/reason names. `fwupd` carries `ubx` with it.
# mp also includes ui/ui.h and port/port_image.h for *types only* — no ui symbol
# is referenced, so `ui` is deliberately not a link dependency and stays out of
# these suites' coverage.
module_deps(mp       util cfg logring quality ina228 fwupd)

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

# `ppsjoin` covers the step between core/disc's pairing primitives and the
# `ppscorr` arithmetic: naming the pulse a hardware capture belongs to from
# receiver evidence that arrives at its own times. `ppscorr` takes epoch_valid
# as an input and `disc` drives the primitives from a caller sitting on the PPS
# edge, so the join between them was covered by neither — and that is exactly
# where a defect lived, silently deciding at boot whether a unit would ever
# serve sub-microsecond time. disc_name_pulse() lives in core/disc, hence the
# same dependency set as the `disc` suite.
module_deps(ppsjoin disc util quality)

# The platform area's safety policy, all four in Zephyr-free headers beside
# their .c for the same reason `ppscorr` is: these decisions are silent when
# they are wrong and dangerous when silent, and none of them is reachable from a
# host test through the .c, which is devicetree tables, timers and threads.
#
# `io_policy` (platform/sts_io_policy.h) — the 1 kHz scan's signal-to-pin split,
# the ALERT-to-monitor lookup and the event dispatch. It resolves rails through
# ina228_rail_tbl[] and names signals from core/fault, hence both edges.
module_deps(io_policy   fault ina228 logring util)

# `fan_policy` (platform/sts_fan_policy.h) — the fan actuator and the thermal
# ladder's rungs 2 and 3. core/thermal decides the duty; this decides what
# happens to it, including the ARCHITECTURE.md §10.9 invariant that the resting
# state is full speed.
module_deps(fan_policy  thermal fault util)

# `super_policy` (platform/sts_super_policy.h) — the liveness AND-gate feeding
# the TPS3430 kick, the holdover relay's three gates, the status-RGB inputs and
# the §8.3 self-confirm trigger. The cadence arithmetic itself is core/pwrseq's
# (pwrseq_wdt_service), already covered by the `pwrseq` suite.
module_deps(super_policy pwrseq fault quality ina228 util)

# `rbguard` (platform/sts_rbguard.h) — the stage-8 rubidium envelope: the
# hardware ceiling on the configurable VCC_RB limit, the refusal of a ceiling
# below the operating setpoint, the PG mask decode, the EXTREF band and the
# RB_LOCK opto polarity. The root CLAUDE.md is explicit that an error here can
# destroy the rubidium.
module_deps(rbguard     pwrseq fault ina228 util)

# `pwrseq_req` (platform/sts_pwrseq_req.h) — the parameterised request mailbox
# that lets a management thread command a rail without writing its pin. It is
# the seam between the console area and the single writer of every rail, so the
# suite links `mp` for the real lease table, RPC layer and event stream, and
# drives them against the real mailbox header. `mp` carries cfg, logring,
# quality, ina228 and fwupd with it; `pwrseq` is NOT a link dependency — the
# suite mirrors pwrseq_exec.c's drain rather than running the stage machine,
# which test_mp_veto.c already does.
module_deps(pwrseq_req  mp util)

# `gnss_tee` is the same idea for the platform area's NMEA/UBX run classifier
# (app/src/zephyr/platform/gnss_tee.h): the split that decides which of the
# maintenance tool's two capture channels each received byte belongs to. It runs
# on the priority-6 GNSS thread, so it is written to be countable rather than
# clever, and every way it can be wrong is silent — a '$' inside a NAV-SAT
# payload misread as a sentence start does not fail, it just quietly corrupts a
# technician's capture. Zephyr-free by construction, hence the empty dependency
# list.
module_deps(gnss_tee)

# `smear_isolation` is an architecture-fitness test, not a unit test: it scans
# the source tree to enforce that the spec §15.2 leap smear stays confined to
# the NTP serving path — never the PTP grandmaster (IEEE 1588 has no smear
# concept) and never the discipline loop. C has no module privacy and the PTP
# glue transitively includes ntp/ntp.h, so the containment cannot be expressed
# in the language; it is checked here instead. It compiles no core module, hence
# the empty dependency list — it reads the sources, it does not link them.
module_deps(smear_isolation)

# `ntp_keys` is the same idea for the net area's key reconciliation
# (app/src/zephyr/net/sts_ntp_keys.h): Zephyr-free by construction so the
# revocation decision can be tested here. It drives core/ntp's real key table
# rather than a model, because the defect it guards against lived exactly in the
# disagreement between the two — ntp_key_set() validates before it clears, so a
# rejected rotation left the previous key live.
module_deps(ntp_keys ntp util quality)

# `secops_policy` is the same idea for the net area's AAA offload gate
# (app/src/zephyr/net/sts_secops_policy.h): the feed-slice arithmetic, the
# budgets, and who releases the gate token when a submitter's budget expires
# while the worker is still inside the lookup. Every way it can be wrong is
# silent — a double release puts two logins inside sts_aaa.c's shared static
# buffers, a lost release denies the management plane for the rest of the
# uptime, and a wait that never terminates keeps feeding the watchdog that
# exists to recover the board from exactly that. Zephyr-free by construction,
# hence the empty dependency list.
module_deps(secops_policy)

# `ldap_ca` is the same idea for the net area's LDAPS transport decision
# (app/src/zephyr/net/sts_ldap_ca.h): which socket `sec.ldap.mode` opens, and
# what is missing when it opens none. It is Zephyr-free because the failure it
# guards against is silent in both directions — mode 2 used to open a TLS socket
# that could never complete a handshake (no CA, no hostname, mbedTLS's client
# default of VERIFY_REQUIRED), and the one way to make that worse is a refusal
# path that falls through to a cleartext bind and puts the directory password on
# the wire. It compiles no core module, hence the empty dependency list; it
# includes core/web/rest.h only through zephyr/net/sts_web.h, for the sec-tag
# collision check, and links nothing from it.
#
# The same header also owns the anchor's ERASE, and that half of the suite reads
# zephyr/net/sts_aaa.c the way `factory_policy` and `smear_isolation` read their
# sources: the rule is that the /lfs unlink runs before — and independently of —
# the AAA exchange mutex, which is a property of the statement order and cannot
# be expressed in C. Taking that mutex with K_FOREVER, as it once did, made a
# factory reset wait on a directory bind and cold-cycle the board mid-wipe with
# the previous operator's CA still installed.
module_deps(ldap_ca)

# `ptp_profile_policy` (app/src/zephyr/net/sts_ptp_profile_policy.h) is the
# precedence rule between a PTP profile's parameter set and the operator's cfg
# keys. Zephyr-free and compiling no core module — it takes PTP_PROFILE_COUNT
# and the descriptor's numbers as plain values rather than including the engine,
# so the suite states what G.8275.1 requires instead of asking the code that
# implements it. Hence the empty dependency list.
module_deps(ptp_profile_policy cfg ptp util quality)

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
# `mp_fw` is the six FMT §9 `fw.*` methods: the paged inventory, the per-target
# guard class, the allow-list refusal, the offset arithmetic of a resumable
# chunked transfer and the abort paths. It drives the real core/fwupd through
# the mp_fwupd_t port rather than a model, because the property that matters —
# restore() once per prepare() on every exit — is core/fwupd's and the seam
# between the two is where a missed abort would live.
module_deps(mp_fw       mp)

# `mp_tunnel_policy` (app/src/zephyr/console/sts_mp_tunnel_policy.h) is the
# console area's host->device passthrough routing decision: which port a channel
# names, whether its tunnel is open, and the burst bound that keeps a blocking
# uart_poll_out() from holding the engine lock past the service tick's budget.
# Zephyr-free by construction so the refusals can be tested here, hence the
# empty dependency list.
module_deps(mp_tunnel_policy)

# `console_link_policy` (app/src/zephyr/console/sts_console_link_policy.h) is the
# other half of that area's MP wiring: when the console port's (VBUS, DTR) sample
# is a *link transition*, which FMT §5.4 makes the override dead-man's outermost
# condition. The `mp` edge is real and is the point of the suite — it drives the
# policy's verdict into a live mp_ctx_t holding an actual override lease, because
# the defect it guards was never in the seam (mp_set_link() always worked) but in
# nothing deciding to call it, and a suite that reached the seam directly would
# have passed throughout.
module_deps(console_link_policy mp)

# `fwupd_seam_policy` (app/src/zephyr/console/sts_fwupd_seam_policy.h) is the
# firmware-update seam's four decisions: converting the platform's count-returning
# transmit onto core/fwupd's "0 on success" contract, telling an absent raw
# transport from one whose area has not started, refusing a G0 identity read on a
# port a passthrough tunnel owns, and the supervisor-pass budget the MP dead-man
# BUILD_ASSERT is built from. All four are silent when wrong and three have been.
# Zephyr-free by construction, hence the empty dependency list.
module_deps(fwupd_seam_policy)

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

# `sky_policy` (app/src/zephyr/ui/sts_sky_policy.h) is the layer between the
# renderer and the panel: where the square canvas goes inside a UI_HINT_SKYPLOT
# band, which UBX-NAV-SAT records become markers, whether the plot may be
# rotated to true north at all, and when the picture has to be pushed again.
# Zephyr-free by construction so all of it is reachable here — and it needs to
# be, because every one of those decisions is silent when it is wrong: an
# oversized canvas is an out-of-bounds framebuffer write, an unfiltered record
# paints a phantom satellite exactly where an operator looks for an obstruction,
# and spec §6.3 is explicit that a plausible-but-wrong rotation is worse than an
# honest un-rotated one. It drives the real core/ui/skyplot.c rather than a
# model — sts_sky_orient() calls sky_heading_from_ecompass() and
# sky_declination_dipole_ddeg() directly — hence the same `ui` edge as `skyplot`.
module_deps(sky_policy ui)

# `ptp_icv_policy` (app/src/zephyr/net/sts_ptp_icv_policy.h) is the arming
# decision for IEEE 1588-2019 Annex P: whether the port holds an ICV context at
# all, and what ptp_icv_cfg_t to load into it. It defers to core/ptp's own
# ptp_icv_key_set()/ptp_icv_cfg_validate() rather than re-deriving what a usable
# association looks like, hence the real `ptp` edge — a model would let the glue
# install something core silently reinterprets, which is the class of defect
# that left ptp_port_ctx_t::icv permanently NULL in the first place.
module_deps(ptp_icv_policy ptp)

# `factory_policy` (app/src/zephyr/ui/sts_factory_policy.h) is the shape of a
# factory reset, shared by the web, MCP and panel planes. Half the suite is the
# arithmetic; the other half reads those three sources and asserts each plane
# still runs both the config reset and the key wipe, because the defect the
# header records lived in the *set* of the three and no C construct can express
# it. It compiles no core module, hence the empty dependency list — like
# `smear_isolation` it reads the sources, it does not link them.
module_deps(factory_policy)

# `web_sky` (app/src/zephyr/net/sts_web_sky.h) is the web plane's GNSS-detail
# decision: whether the cached NAV-SAT frame is fresh enough to serve, which
# records are worth serving, what `detail_available` is allowed to mean, and the
# antenna verdict the net area can honestly reach. Every one of them is silent
# when it is wrong — sts_app.h leaves the last good frame in the cache when a
# receiver stops talking, so an unaged read serves an hour-old sky as current
# with no indication anywhere.
#
# The `web` edge is real rather than type-only: the suite drives rest.c's
# encoder to prove the worst-case satellite list still fits STS_WEB_RESP_SIZE,
# because rest_encode_telemetry()'s -ENOSPC makes ws_run() send nothing at all.
# It also includes ui/sts_sky_policy.h to pin its admission rule against the
# panel's (spec §371) — static inline only, so no `ui` link edge.
module_deps(web_sky web)

# `mp_telem` is the join between the platform area's published state and the
# Maintenance Protocol telemetry record — sts_pwrseq_pub.h, sts_mp_telem.h and
# sts_ui_refs.h. It exists because `mp_stream` proved the wire and the defect was
# upstream of it: mp_telem_t declared the sequencer's stage, the receiver's state
# and the survey progress, mp_stream.c encoded them, and nothing wrote them, so
# every unit in the field reported zeros with every test passing.
#
# Every edge is a real one, no models. `pwrseq` because the suite drives a real
# pwrseq_ctx_t through pwrseq_step()/pwrseq_shed_step()/pwrseq_ov_observe() and
# reads it with pwrseq_status() — a hand-set ctx would still pass against a
# status accessor that had stopped reading the field. `fault` for the same
# reason on key 45: the scan bitmap is produced by feeding real GPIOF/GPIOG
# words to fault_scan_input() and letting the debounce commit them, so the word
# that reaches the wire was computed by the module that owns it rather than
# poked into a struct. `mp` for the encoder and the CBOR reader that decodes
# what it produced. `gnssmgr` and `refsel` for the enumerations the record
# carries raw. `ui` and `web` because the survey-accuracy rule is shared by
# three planes and is pinned across all three here.
module_deps(mp_telem mp pwrseq fault gnssmgr refsel ui web)

# `recovery_policy` (app/src/zephyr/console/sts_recovery_policy.h) is the
# operator-recovery surface's three decisions: which latch may be acknowledged
# and what the answer means, whether a tempco fit may be believed, and which
# recovery action an MP object id names. Each is silent when wrong and each
# guards something a technician only reaches after something else has already
# failed.
#
# Three real edges, no models. `fault` because the clear-outcome map is a
# translation of fault_alarm_clear()'s errnos and is only correct against them;
# `disc` because the acceptance rule is judged on numbers from the real
# disc_tempco_fit(), including the two-point R^2 = 1.0 case it exists to catch;
# `mp` because the object router is asserted against the shipped manifest —
# `pwr.poe.kill` must still be G3 with MP_OF_PULSE and without MP_OF_WRITE, and
# a downgrade there is exactly the change no reviewer would notice.
module_deps(recovery_policy fault disc mp)

# `mp_events` is the seam that puts the BOARD on MP channel 0x09: the 1 kHz
# scan's edges and the alarm table's transitions. It exists because
# `mp_stream` proved the encoder and `mp_rpc` proved mp_post_event(), and the
# defect was upstream of both — nothing called mp_post_event() anywhere, so
# --gc-sections dropped it and five of the channel's eight event kinds had no
# producer in the image at all. A technician subscribed and heard silence
# through a power-good drop, an alarm, a button press and a door opening.
#
# Every edge is a real one, no models. `fault` because each event starts as a
# GPIOF/GPIOG word fed to fault_scan_input() and is debounced by the module that
# owns the decision; `ina228` because sts_io_policy.h resolves an ALERT to its
# monitor through ina228_rail_tbl[]; `mp` for the staging queue's record type,
# the engine the drain feeds, the frame decoder that reads the wire back and the
# CBOR reader that decodes it.
module_deps(mp_events mp fault ina228 logring util)

# `mp_veto` is the seam that lets FIRMWARE withdraw an override (FMT §5.1.2,
# §5.4). It exists because test_mp_override.c proved mp_ovr_veto() and
# test_mp_rpc.c proved mp_veto(), and the defect was upstream of both — nothing
# raised one, so both symbols were absent from zephyr.elf and an override the
# board had made untrue stood until its keepalive lapsed, with no veto on
# channel 0x09 and no audit line.
#
# Real edges, no models. `pwrseq` because the condition is the real stage
# machine deciding to drop the rubidium — the suite raises RB_OV_DET on a real
# pwrseq_in_t and takes the actions pwrseq_step() emits, so a change to that
# decision changes this suite; `ina228` for the rail table the sequencer's
# inputs are built from; `mp` for the lease table, the manifest the subject map
# resolves against, the event stream and the frame decoder that reads the veto
# back off the wire; `logring` for the audit line, which is the half FMT §5.3
# requires and a stream subscriber does not provide.
module_deps(mp_veto mp pwrseq ina228 logring util)

# `mp_deferred` is the manifest's honesty flag, MP_OF_DEFERRED, against the
# dispatch that defines it. FMT §4 has the host generate its whole UI from the
# manifest, so an object that is published and then answers -ENOTSUP is a
# control a technician tries at a bench and watches fail — and the flag that
# exists to say so in advance was set by zero objects while 31 of 39 mutable
# objects were refused. A flag alone would rot the first time somebody wired a
# setter, so this suite DERIVES the wired set by reading mp_glue.c's dispatch
# (brace-matched, function-scoped, `strcmp(o->id, "…")` only) and fails the
# manifest if the two disagree.
#
# `mp` alone: the manifest is the thing under test, and the two policy-table
# branches come from the header-only console/sts_recovery_policy.h, which is on
# the include path and compiles anywhere. Everything else this suite knows it
# learns by reading source it deliberately does not link.
module_deps(mp_deferred mp)

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
  # For suites that read the source tree rather than link it — see
  # test_smear_isolation.c. Harmless everywhere else.
  target_compile_definitions(${test_target} PRIVATE
    STS_APP_SRC_DIR="${APP_SRC_DIR}")
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
