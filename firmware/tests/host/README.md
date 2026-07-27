# Host unit tests

CMake + CTest + vendored [Unity](https://github.com/ThrowTheSwitch/Unity) v2.6.0 (MIT,
`unity/LICENSE.txt`). These tests exercise `firmware/app/src/core/` — the platform-neutral
half of the firmware (ARCHITECTURE.md §4). Zephyr glue is not built here; it is
compile-verified by the target build and exercised on hardware.

## Running

```sh
firmware/scripts/test.sh            # configure + build + ctest, all suites
firmware/scripts/coverage.sh        # the same, instrumented, + the gcovr gate
```

While other modules are mid-flight, build and run only your own suites. This is the point
of the per-module dependency model below: a neighbour that does not currently compile
cannot fail your run.

```sh
TESTS='ntp ptp'      firmware/scripts/test.sh   # space- or comma-separated
BUILD_DIR=/tmp/mine  firmware/scripts/test.sh   # out of tree; override freely
CTEST_ARGS='-V'      firmware/scripts/test.sh   # extra ctest flags
FAIL_UNDER_LINE=90   firmware/scripts/coverage.sh
```

Both scripts build outside the repository; nothing is written into the tree.

## Adding a test

Registration is zero-touch: drop `test_<module>.c` in this directory and it is discovered,
built into its own executable, and registered with CTest as `<module>`.

| Rule | Detail |
|---|---|
| Filename | `test_<module>.c`, in this directory (not a subdirectory) |
| CTest name | `<module>` — the `test_` prefix is stripped |
| Sources | `test_<module>.c` + `core/<module>/*.c` + every dependency module's `*.c` + `support/*.c` + Unity. Nothing else. |
| Includes | `-I app/src/core`, `-I app/src`, `-I tests/host/support` — so `#include "util/crc.h"`, `#include "port/port_time.h"` and `#include "test_support.h"` all work |
| Flags | `-std=c11 -Wall -Wextra -Werror`; `COVERAGE=ON` adds `--coverage -O0 -g` |
| `main()` | Each file provides its own, using the pattern below |
| Fixtures | `setUp`/`tearDown` default to weak no-ops from `support/test_support.c`; define strong ones in your file only if you need them |

The globs use `CONFIGURE_DEPENDS`, so a plain `cmake --build` notices a new test file, a
new core source, or a whole new module directory landing for the first time.

### Declaring dependencies

`CMakeLists.txt` holds a `module_deps()` map, already populated for every module in
ARCHITECTURE.md §5. **A test binary compiles only its own module plus the transitive
closure of that module's declared dependencies** — so `gnssmgr` declares `ubx`, and `util`
arrives through it. A module directory that does not exist yet contributes nothing and is
skipped silently, which is why the map can be filled in ahead of the code.

You only touch the map when your module's dependency set is not already correct there:

```cmake
module_deps(gnssmgr  ubx util)   # direct deps only; util also arrives via ubx
```

Two cases worth knowing:

* **A module with several suites.** Each suite name needs its own entry naming the module
  it belongs to. `util` is split across `test_crc.c`, `test_cobs.c`, `test_ring.c` and
  `test_bytes.c`, so the map carries `module_deps(crc util)` and friends; there is no
  `core/crc/` directory and the glob for it is simply empty.
* **A suite with no map entry** falls back to compiling all of core and CMake warns. That
  escape hatch keeps a typo or a brand-new module from blocking a run — it is not a
  supported configuration, because it re-couples your suite to every other module.

Configure prints what each suite resolved to, which is the fastest way to see why a symbol
is or is not being linked:

```
--   gnssmgr <- gnssmgr ubx util
--   ntp <- ntp util quality
```

### Harness pattern

```c
#include "unity.h"
#include "util/crc.h"

static void test_crc32_known_answer(void)
{
	TEST_ASSERT_EQUAL_HEX32(0xCBF43926U, sts_crc32_ieee("123456789", 9));
}

int main(void)
{
	UNITY_BEGIN();
	RUN_TEST(test_crc32_known_answer);
	return UNITY_END();
}
```

If your module calls through a `port/` interface, supply the fake in your test file — the
ports are function-pointer structs precisely so a test can fill them in without linking any
glue.

## What a suite owes the reader

ARCHITECTURE.md §9 asks for known-good byte vectors, not self-round-trips. A round-trip
test proves the encoder and decoder agree with each other, which they will even when both
are wrong. Anchor each codec to values derived independently of this code — the published
check value from a CRC catalogue, the example table in the protocol's own specification,
the byte layout in the RFC — and put the provenance in a comment next to the vector.
Round-trip and fuzz-ish tests are then useful on top, for the cases no published vector
covers.

Core takes time and entropy through ports, so tests inject both; nothing here may depend on
the wall clock, the host byte order, or an address.

## Coverage

`scripts/coverage.sh` runs `gcovr` filtered to `firmware/app/src/core/` and fails under
80 % lines (ARCHITECTURE.md §9). It writes `coverage.txt`, `coverage.xml` (Cobertura) and
`coverage.html` into `${BUILD_DIR}/coverage/`, and echoes the per-file table to stdout.

The measurement is the union across every test binary, so a module pulled in only as
somebody else's dependency still appears — at 0 % if nothing exercises it. **A module that
no test binary compiles at all is a different matter: it produces no coverage records and
the gate cannot see it, so it is not counted as 0 % — it is simply absent.** Configure
warns by name when that happens:

```
CMake Warning: tests/host: core module(s) compiled into no test binary: orphan.
```

Treat that warning as a gate failure in the making, not as noise: the percentage stops
describing the whole tree the moment it appears.
