#!/usr/bin/env bash
#
# STS1000 "Meridian" — configure, build and run the host unit tests.
#
#   firmware/scripts/test.sh [extra cmake args...]
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

ctest_args=(--test-dir "${BUILD_DIR}" --output-on-failure)
if [[ -n ${CTEST_ARGS:-} ]]; then
	read -r -a extra_ctest <<<"${CTEST_ARGS}"
	ctest_args+=("${extra_ctest[@]}")
fi

echo "== configure  (${src_dir} -> ${BUILD_DIR})"
cmake "${cmake_args[@]}" "$@"

echo "== build"
cmake --build "${BUILD_DIR}" --parallel

echo "== test"
ctest "${ctest_args[@]}"
