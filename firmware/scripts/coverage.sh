#!/usr/bin/env bash
#
# STS1000 "Meridian" — host unit tests with the gcov line-coverage gate.
#
#   firmware/scripts/coverage.sh [extra cmake args...]
#
# Builds tests/host with --coverage, runs ctest, then runs gcovr restricted to
# firmware/app/src/core. Per ARCHITECTURE.md §9 the 80 % line gate applies to
# app/src/core only — the platform-neutral logic, which is where correctness
# lives. The test harness, vendored Unity and the Zephyr glue are outside the
# measurement on purpose: instrumenting them would inflate the number without
# telling anyone anything.
#
# Artifacts land in ${BUILD_DIR}/coverage/:
#   coverage.txt    per-file line/branch table (also echoed to stdout)
#   coverage.xml    Cobertura, for CI ingestion
#   coverage.html   browsable report, per-line detail in coverage.*.html
#
# Environment:
#   BUILD_DIR         build tree (default: a temp dir; never inside the repo)
#   FAIL_UNDER_LINE   gate percentage (default 80)
#   GCOVR / GCOV      tool overrides
#   CTEST_ARGS        extra ctest arguments

set -euo pipefail

here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
fw_root=$(cd -- "${here}/.." && pwd)
src_dir="${fw_root}/tests/host"

BUILD_DIR=${BUILD_DIR:-${TMPDIR:-/tmp}/sts1000-host-coverage}
FAIL_UNDER_LINE=${FAIL_UNDER_LINE:-80}
GCOV=${GCOV:-gcov}

if [[ -z ${GCOVR:-} ]]; then
	if command -v gcovr >/dev/null 2>&1; then
		GCOVR=gcovr
	elif [[ -x /opt/zvenv/bin/gcovr ]]; then
		GCOVR=/opt/zvenv/bin/gcovr
	else
		echo "coverage.sh: gcovr not found; set GCOVR=/path/to/gcovr" >&2
		exit 127
	fi
fi

cov_dir="${BUILD_DIR}/coverage"

cmake_args=(-S "${src_dir}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Debug -DCOVERAGE=ON)
if command -v ninja >/dev/null 2>&1; then
	cmake_args+=(-G Ninja)
fi

ctest_args=(--test-dir "${BUILD_DIR}" --output-on-failure)
if [[ -n ${CTEST_ARGS:-} ]]; then
	read -r -a extra_ctest <<<"${CTEST_ARGS}"
	ctest_args+=("${extra_ctest[@]}")
fi

echo "== configure  (${src_dir} -> ${BUILD_DIR}, COVERAGE=ON)"
cmake "${cmake_args[@]}" "$@"

echo "== build"
cmake --build "${BUILD_DIR}" --parallel

# Counters accumulate across runs and a stale .gcda whose .gcno has since
# changed makes gcc's runtime abort the merge. Start every run from zero.
echo "== reset counters"
find "${BUILD_DIR}" -name '*.gcda' -delete

echo "== test"
ctest "${ctest_args[@]}"

echo "== coverage  (gate: lines >= ${FAIL_UNDER_LINE}%)"
mkdir -p "${cov_dir}"

set +e
"${GCOVR}" \
	--root "${fw_root}" \
	--gcov-executable "${GCOV}" \
	--filter 'app/src/core/' \
	--exclude-unreachable-branches \
	--sort uncovered-percent \
	--txt "${cov_dir}/coverage.txt" \
	--cobertura "${cov_dir}/coverage.xml" --cobertura-pretty \
	--html-details "${cov_dir}/coverage.html" \
	--fail-under-line "${FAIL_UNDER_LINE}" \
	"${BUILD_DIR}"
rc=$?
set -e

if [[ -f ${cov_dir}/coverage.txt ]]; then
	echo
	cat "${cov_dir}/coverage.txt"
	echo
fi

echo "reports: ${cov_dir}/coverage.{txt,xml,html}"

if [[ ${rc} -ne 0 ]]; then
	echo "coverage.sh: FAILED — line coverage of app/src/core is below ${FAIL_UNDER_LINE}%" >&2
	exit "${rc}"
fi

echo "coverage.sh: OK — line coverage of app/src/core meets the ${FAIL_UNDER_LINE}% gate"
