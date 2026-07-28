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
#
# WHAT IT DOES NOT CHECK (coverage boundary)
#
# Exactly one archive is inspected: ${BUILD_DIR}/app/app/libapp.a, the archive
# Zephyr builds from the application's own `zephyr_library()`. Application code
# that lands in any *other* archive — a second zephyr_library(), a library in a
# subdirectory of app/ — is outside this gate. Nothing does that today, so the
# script scans the application's CMake binary tree for stray archives and warns
# rather than silently covering less than it claims. Zephyr's own libraries and
# the module archives are deliberately out of scope: they are upstream code, not
# ours to keep reachable.
#
# WHICH SYMBOL TYPES ARE GATED
#
#   T  global text — the case this exists for.
#   W  weak text (`__weak`). Gated *identically* to T, and that is not a
#      false-positive risk: when a strong override exists the linker keeps the
#      override under the same name, so the name is present in the image and the
#      symbol is not absent. A weak name absent from the image means neither the
#      default nor any override shipped, which is exactly the gap worth failing
#      on. The tree relies on this idiom in two places — the glue-area no-ops in
#      platform/sts_area_weak.c and the five USART3 seam hooks in
#      console/fwupd_glue.c — and an ungated `W` would let either be dropped
#      silently. Folding W in changed the absent set by zero symbols when it was
#      introduced, which is the expected result on a healthy tree.
#   t  file-local text is excluded on purpose: a file-static with no caller is a
#      compiler warning's job, not this script's, and including them would bury
#      the cross-module gaps this exists to find. (Local symbols *are* read from
#      zephyr.elf, on the other side of the comparison — a symbol that survived
#      into the image as a local is still in the image.)
#
# THE SYMBOL READ IS CHECKED, NOT ASSUMED
#
# A silent empty or truncated symbol table would report perfect reachability,
# which is the one wrong answer this script must never give. A *total* failure
# is loud by accident (every allow-list entry goes stale at once); a *partial*
# read is not, so each nm read is run into a file with its exit status checked,
# and each resulting set is asserted to be above a plausible floor. Failing
# either is exit 3, distinct from a genuine gate failure.

set -eu

# Floors for the symbol reads. Real figures at the time of writing are 1272
# application globals across 103 archive members and ~4500 image text symbols;
# these are set an order of magnitude below that, low enough to survive ordinary
# growth and shrinkage of the tree and high enough that a truncated read cannot
# pass.
#
# What this does NOT catch, stated plainly rather than left to be discovered: a
# read that loses only a *few* members still clears every floor. The floors turn
# the catastrophic-but-quiet case into a loud one; they are not a completeness
# proof. The stale-entry check is the other half — on a populated allow-list any
# truncation that drops an allow-listed symbol fails there too — but that half
# is worth nothing on an empty or freshly-generated list, which is exactly when
# these floors carry the whole load.
min_defined=100
min_kept=500
min_members=20

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
app_bindir="${build_dir}/app/app"
lib="${app_bindir}/libapp.a"

for f in "${elf}" "${lib}"; do
	if [ ! -f "${f}" ]; then
		echo "$0: ${f} not found — build first (scripts/build.sh)" >&2
		exit 2
	fi
done

# Coverage boundary (see the header): libapp.a is the only archive inspected.
# If the application's CMake tree ever grows a second archive, say so rather
# than quietly checking less than the summary line claims.
extra_archives=$(find "${app_bindir}" -name '*.a' ! -name 'libapp.a' 2>/dev/null | sort)
if [ -n "${extra_archives}" ]; then
	echo "reachability: WARNING — application archives this gate does not inspect:" >&2
	printf '%s\n' "${extra_archives}" | sed 's/^/    /' >&2
	echo "    Only $(basename "${lib}") is checked. Extend this script, or fold" >&2
	echo "    those sources back into the app library, before trusting the result." >&2
	echo >&2
fi

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

# Read each symbol table into a file of its own, with the exit status checked.
# This is deliberately not `nm ... | awk ... | sort`: `set -eu` does not catch a
# failure part-way down a pipeline, /bin/sh here is dash and cannot be relied on
# for `pipefail`, and the failure mode that survives — nm producing *some* of
# its output and then dying — is the silent one.
read_symbols() {
	# read_symbols <object> <outfile> <nm-type-regex>; raw output kept at <outfile>.raw
	if ! "${nm}" --defined-only "$1" > "$2.raw" 2>"${tmp}/nm.err"; then
		echo "reachability: FAIL — could not read symbols from $1" >&2
		sed 's/^/    /' "${tmp}/nm.err" >&2
		echo "    Refusing to report reachability from an incomplete symbol table." >&2
		exit 3
	fi
	awk -v types="$3" '$2 ~ types { print $3 }' "$2.raw" | sort -u > "$2"
}

# Text symbols the linker kept. Local (t) as well as global (T): a symbol that
# survived as a local is still in the image.
read_symbols "${elf}" "${tmp}/kept" '^[Tt]$'

# Text the application defines: global (T) and weak (W). Weak is gated because a
# strong override keeps the *name* in the image, so an override never registers
# as an absence — see the symbol-type note in the header. File-local (t) is
# excluded on purpose, also documented there.
read_symbols "${lib}" "${tmp}/defined" '^[TW]$'

# A partial read is the dangerous one: it yields a short `defined` set, no
# absences, and a confident "everything is in the image". Assert both sets are
# above a floor that no real build can fall under, and — because a partial read
# of an archive loses whole members — that nm reported on every member it found.
# nm prints one `member.obj:` header per member, so counting them costs nothing
# and catches the loss of a member outright rather than by symbol count alone.
n_kept=$(wc -l < "${tmp}/kept" | tr -d ' ')
n_defined=$(wc -l < "${tmp}/defined" | tr -d ' ')
n_members=$(grep -c '^[^[:space:]].*:$' "${tmp}/defined.raw" || true)
if [ "${n_kept}" -lt "${min_kept}" ] || [ "${n_defined}" -lt "${min_defined}" ] ||
	[ "${n_members}" -lt "${min_members}" ]; then
	echo "reachability: FAIL — implausible symbol counts; the read was truncated." >&2
	printf '    %s: %s text symbols (floor %s)\n' \
		"${elf}" "${n_kept}" "${min_kept}" >&2
	printf '    %s: %s global/weak text symbols in %s members (floors %s / %s)\n' \
		"${lib}" "${n_defined}" "${n_members}" "${min_defined}" "${min_members}" >&2
	echo >&2
	echo "    A short symbol table reports perfect reachability, which is the" >&2
	echo "    one wrong answer this script must never give. Check ${nm} and the" >&2
	echo "    build tree; do not lower the floor to make this pass." >&2
	exit 3
fi

comm -23 "${tmp}/defined" "${tmp}/kept" > "${tmp}/absent"

# The allow-list is the policy. A missing one is a broken invocation, not an
# empty policy: silently substituting "nothing is allowed" turns a typo in -a
# into either a flood of failures or, on a clean tree, a pass that proves
# nothing. --update is the one mode that may legitimately create it.
if [ ! -f "${allow_file}" ]; then
	if [ -z "${update}" ]; then
		echo "$0: allow-list ${allow_file} not found." >&2
		echo "    Pass -a with a real path, or run --update to create one." >&2
		exit 2
	fi
	: > "${tmp}/allowed"
else
	# Allow-list format: one symbol per line, optional trailing "# reason".
	# Blank lines and full-line comments are ignored.
	sed 's/#.*//' "${allow_file}" |
		awk 'NF { print $1 }' |
		sort -u > "${tmp}/allowed"
fi

if [ ! -s "${tmp}/absent" ] && [ -z "${update}" ]; then
	echo "reachability: every global the application defines is in the image."
	# Fall through anyway when the allow-list still has entries: they are all
	# stale now, and a list that outlives what it explains is the rot this
	# gate exists to prevent.
	[ -s "${tmp}/allowed" ] || exit 0
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
