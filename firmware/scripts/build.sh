#!/bin/sh
# Build the STS1000 "Meridian" target firmware (MCUboot + signed application).
#
# SPDX-License-Identifier: Apache-2.0
#
# Usage:
#   scripts/build.sh [-p] [-d BUILD_DIR] [-- <extra west build args>]
#
#   -p            pristine build (west build -p always)
#   -n            skip the reachability gate (see the note at the end)
#   -d BUILD_DIR  output directory (default: <repo>/firmware/build)
#
# Produces, per ARCHITECTURE.md §9:
#   <BUILD_DIR>/app/zephyr/zephyr.signed.bin   application, signed for slot0
#   <BUILD_DIR>/mcuboot/zephyr/zephyr.bin      bootloader, for boot_partition
#
# The west workspace root is the PARENT of this repository (T2 topology, see
# firmware/west.yml). west itself is happy to run from anywhere inside the
# workspace, so this script only has to resolve the application path.

set -eu

BOARD=sts1000_meridian

script_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
firmware_dir=$(dirname -- "${script_dir}")
app_dir="${firmware_dir}/app"

pristine=
skip_reach=
build_dir="${firmware_dir}/build"

usage() {
	sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
	exit "${1:-0}"
}

while [ $# -gt 0 ]; do
	case "$1" in
	-p|--pristine)
		pristine="always"
		shift
		;;
	-n|--no-reachability)
		skip_reach=1
		shift
		;;
	-d|--build-dir)
		[ $# -ge 2 ] || { echo "$0: -d needs an argument" >&2; exit 2; }
		build_dir="$2"
		shift 2
		;;
	-h|--help)
		usage 0
		;;
	--)
		shift
		break
		;;
	*)
		echo "$0: unknown option '$1'" >&2
		usage 2
		;;
	esac
done

if ! command -v west >/dev/null 2>&1; then
	echo "$0: 'west' not found on PATH." >&2
	echo "    Activate the Zephyr virtualenv, e.g. PATH=/opt/zvenv/bin:\$PATH" >&2
	exit 127
fi

set -- west build \
	${pristine:+-p "${pristine}"} \
	-b "${BOARD}" \
	--sysbuild "${app_dir}" \
	-d "${build_dir}" \
	"$@"

echo "+ $*"
"$@"

echo
echo "Artifacts:"
for artifact in \
	"${build_dir}/app/zephyr/zephyr.signed.bin" \
	"${build_dir}/app/zephyr/zephyr.signed.hex" \
	"${build_dir}/mcuboot/zephyr/zephyr.bin" \
	"${build_dir}/mcuboot/zephyr/zephyr.hex"
do
	if [ -f "${artifact}" ]; then
		printf '  %s\n' "${artifact}"
	else
		printf '  MISSING: %s\n' "${artifact}" >&2
	fi
done

# ---------------------------------------------------------------------------
# Reachability gate.
#
# A successful link is not the same as a shipped feature. --gc-sections drops
# any function whose only reference is its own prototype: it costs no flash,
# raises no warning, and passes every host test, because the host suites link
# core/ directly and never see the target image. This project lost an entire
# firmware-update orchestrator, the skyplot renderer, PTP Annex-P integrity and
# the whole Rb serial layer that way, each of them written, tested and
# documented as delivered.
#
# So it runs here, on the artifact, and its exit status is this script's. Skip
# it with -n while mid-refactor if you must; do not skip it in CI.
# ---------------------------------------------------------------------------
if [ -n "${skip_reach}" ]; then
	echo
	echo "reachability: SKIPPED (-n)"
	exit 0
fi

echo
"${script_dir}/reachability.sh" -d "${build_dir}"
