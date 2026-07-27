#!/usr/bin/env bash
#
# STS1000 "Meridian" — configure, build and run the host unit tests.
#
#   firmware/scripts/test.sh [extra cmake args...]
#
# Working on one module while others are mid-flight? Name your suites and the
# script builds only those targets, so a neighbouring module that does not
# currently compile cannot fail your run:
#
#   TESTS='ntp ptp'  firmware/scripts/test.sh      # space- or comma-separated
#
# Without TESTS every suite is built and run, which is what CI wants.
#
# The build tree is always outside the repository. It defaults to a temp
# directory; override with BUILD_DIR when you want it somewhere specific:
#
#   BUILD_DIR=/path/to/build-tests firmware/scripts/test.sh
#   CTEST_ARGS='-R cobs -V'        firmware/scripts/test.sh
#
# Coverage lives in the sibling coverage.sh, which uses its own build tree so
# an instrumented build never shadows the fast one.

set -euo pipefail

here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
fw_root=$(cd -- "${here}/.." && pwd)
src_dir="${fw_root}/tests/host"

BUILD_DIR=${BUILD_DIR:-${TMPDIR:-/tmp}/sts1000-host-tests}

cmake_args=(-S "${src_dir}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Debug)
if command -v ninja >/dev/null 2>&1; then
	cmake_args+=(-G Ninja)
fi

build_args=(--build "${BUILD_DIR}" --parallel)
ctest_args=(--test-dir "${BUILD_DIR}" --output-on-failure)

if [[ -n ${TESTS:-} ]]; then
	read -r -a wanted <<<"${TESTS//,/ }"
	targets=()
	filter=""
	for suite in "${wanted[@]}"; do
		targets+=("test_${suite}")
		filter+="${filter:+|}^${suite}\$"
	done
	# --target must come last: it consumes every following non-option word.
	build_args+=(--target "${targets[@]}")
	ctest_args+=(-R "${filter}")
	echo "== subset: ${wanted[*]}"
fi

if [[ -n ${CTEST_ARGS:-} ]]; then
	read -r -a extra_ctest <<<"${CTEST_ARGS}"
	ctest_args+=("${extra_ctest[@]}")
fi

echo "== configure  (${src_dir} -> ${BUILD_DIR})"
cmake "${cmake_args[@]}" "$@"

echo "== build"
cmake "${build_args[@]}"

echo "== test"
ctest "${ctest_args[@]}"
