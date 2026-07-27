#!/bin/sh
# Report application functions that were compiled but dropped from the image.
#
# SPDX-License-Identifier: Apache-2.0
#
# Usage:
#   scripts/reachability.sh [-d BUILD_DIR] [-a ALLOWFILE] [--update]
#
#   -d BUILD_DIR  build tree to inspect (default: <repo>/firmware/build)
#   -a ALLOWFILE  allow-list (default: <repo>/firmware/scripts/reachability.allow)
#   --update      rewrite ALLOWFILE with the current absences, marking every
#                 newly-added symbol `TODO:` so unjustified entries are obvious
#
# WHY THIS EXISTS
#
# The application links with -ffunction-sections/--gc-sections. A function whose
# only reference is its own prototype is compiled, archived into libapp.a, and
# then discarded by the linker. It costs no flash, raises no warning, and — this
# is the part that bites — passes every host test, because the host suites link
# core/ directly and never see the target image at all.
#
# That failure mode has hit this project repeatedly: a whole multi-IC firmware
# update orchestrator, the polar skyplot renderer, the PTP Annex-P integrity
# path and a dozen operator recovery actions were all fully written, fully
# tested, documented as delivered, and absent from the shipped binary. Nothing
# in the build said so. This script is the thing that says so.
#
# WHAT IT CHECKS
#
# Every global text symbol defined in libapp.a but missing from zephyr.elf must
# appear in ALLOWFILE. The allow-list is for symbols that are *deliberately*
# unreachable — public API a host suite exercises but no production caller
# needs. It is not a suppression list: every entry carries a reason, and an
# entry that no longer matches an absent symbol is itself an error, so the file
# cannot rot into a rubber stamp.
#
# Absence is judged against the *linked* image, so an inline-only or
# always-inlined helper legitimately shows up and belongs in the allow-list.

set -eu

script_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
firmware_dir=$(dirname -- "${script_dir}")

build_dir="${firmware_dir}/build"
allow_file="${script_dir}/reachability.allow"
update=

usage() {
	sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'
	exit "${1:-0}"
}

while [ $# -gt 0 ]; do
	case "$1" in
	-d|--build-dir)
		[ $# -ge 2 ] || { echo "$0: -d needs an argument" >&2; exit 2; }
		build_dir="$2"
		shift 2
		;;
	-a|--allow)
		[ $# -ge 2 ] || { echo "$0: -a needs an argument" >&2; exit 2; }
		allow_file="$2"
		shift 2
		;;
	--update)
		update=1
		shift
		;;
	-h|--help)
		usage 0
		;;
	*)
		echo "$0: unknown option '$1'" >&2
		usage 2
		;;
	esac
done

elf="${build_dir}/app/zephyr/zephyr.elf"
lib="${build_dir}/app/app/libapp.a"

for f in "${elf}" "${lib}"; do
	if [ ! -f "${f}" ]; then
		echo "$0: ${f} not found — build first (scripts/build.sh)" >&2
		exit 2
	fi
done

# The SDK's nm, not the host's: the host binutils may not know arm-zephyr-eabi
# objects, and a silent empty symbol table here would report perfect
# reachability, which is the one wrong answer this script must never give.
nm=${NM:-}
if [ -z "${nm}" ]; then
	for cand in /opt/zephyr-sdk-*/arm-zephyr-eabi/bin/arm-zephyr-eabi-nm; do
		[ -x "${cand}" ] && nm="${cand}" && break
	done
fi
if [ -z "${nm}" ] || [ ! -x "${nm}" ]; then
	echo "$0: arm-zephyr-eabi-nm not found; set NM=<path>" >&2
	exit 127
fi

tmp=$(mktemp -d)
trap 'rm -rf "${tmp}"' EXIT INT TERM

# Text symbols the linker kept. Local (t) as well as global (T): a symbol that
# survived as a local is still in the image.
"${nm}" --defined-only "${elf}" |
	awk '$2 == "T" || $2 == "t" { print $3 }' |
	sort -u > "${tmp}/kept"

# Global text the application defines. Local symbols are excluded on purpose —
# a file-static with no caller is a compiler warning's job, not this script's,
# and including them would bury the cross-module gaps this exists to find.
"${nm}" --defined-only "${lib}" |
	awk '$2 == "T" { print $3 }' |
	sort -u > "${tmp}/defined"

comm -23 "${tmp}/defined" "${tmp}/kept" > "${tmp}/absent"

if [ ! -s "${tmp}/absent" ] && [ -z "${update}" ]; then
	echo "reachability: every global the application defines is in the image."
	[ -f "${allow_file}" ] || exit 0
fi

# Allow-list format: one symbol per line, optional trailing "# reason".
# Blank lines and full-line comments are ignored.
if [ -f "${allow_file}" ]; then
	sed 's/#.*//' "${allow_file}" |
		awk 'NF { print $1 }' |
		sort -u > "${tmp}/allowed"
else
	: > "${tmp}/allowed"
fi

if [ -n "${update}" ]; then
	new=$(comm -23 "${tmp}/absent" "${tmp}/allowed" | wc -l | tr -d ' ')
	{
		echo "# Symbols the application defines that are deliberately absent from"
		echo "# the linked image. See scripts/reachability.sh for the rules."
		echo "#"
		echo "# Every entry needs a reason. An entry marked TODO: has not been"
		echo "# justified yet and is a standing action, not a decision."
		echo
		comm -12 "${tmp}/absent" "${tmp}/allowed" | while read -r sym; do
			reason=$(awk -v s="${sym}" '$1 == s { sub(/^[^#]*/, ""); print; exit }' \
				"${allow_file}" 2>/dev/null || true)
			if [ -n "${reason}" ]; then
				printf '%-40s %s\n' "${sym}" "${reason}"
			else
				printf '%-40s # TODO: justify or wire up\n' "${sym}"
			fi
		done
		comm -23 "${tmp}/absent" "${tmp}/allowed" | while read -r sym; do
			printf '%-40s # TODO: justify or wire up\n' "${sym}"
		done
	} > "${tmp}/allow.new"
	mv "${tmp}/allow.new" "${allow_file}"
	echo "reachability: wrote ${allow_file} (${new} newly absent, marked TODO)"
	exit 0
fi

comm -23 "${tmp}/absent" "${tmp}/allowed" > "${tmp}/unexplained"
comm -13 "${tmp}/absent" "${tmp}/allowed" > "${tmp}/stale"

status=0

if [ -s "${tmp}/unexplained" ]; then
	echo "reachability: FAIL — compiled but not in the image, and not allow-listed:" >&2
	sed 's/^/    /' "${tmp}/unexplained" >&2
	echo >&2
	echo "    Either give each one a caller, or add it to" >&2
	echo "    ${allow_file} with the reason it is unreachable on purpose." >&2
	status=1
fi

if [ -s "${tmp}/stale" ]; then
	echo "reachability: FAIL — allow-listed but no longer absent (or no longer defined):" >&2
	sed 's/^/    /' "${tmp}/stale" >&2
	echo >&2
	echo "    Drop these from ${allow_file}. A list that keeps entries it no" >&2
	echo "    longer needs stops being read." >&2
	status=1
fi

if [ "${status}" -eq 0 ]; then
	printf 'reachability: OK — %s defined, %s absent, all %s accounted for.\n' \
		"$(wc -l < "${tmp}/defined" | tr -d ' ')" \
		"$(wc -l < "${tmp}/absent" | tr -d ' ')" \
		"$(wc -l < "${tmp}/allowed" | tr -d ' ')"
fi

exit "${status}"
