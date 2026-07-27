# Host unit tests

CMake + CTest + vendored [Unity](https://github.com/ThrowTheSwitch/Unity) v2.6.0 (MIT,
`unity/LICENSE.txt`). These tests exercise `firmware/app/src/core/` — the platform-neutral
half of the firmware (ARCHITECTURE.md §4). Zephyr glue is not built here; it is
compile-verified by the target build and exercised on hardware.

## Running

```sh
firmware/scripts/test.sh            # configure + build + ctest
firmware/scripts/coverage.sh        # the same, instrumented, + the gcovr gate
```

Both build out of tree. Override the location with `BUILD_DIR=...`; nothing is written
into the repository.

```sh
BUILD_DIR=/tmp/mybuild firmware/scripts/test.sh
CTEST_ARGS='-R cobs' firmware/scripts/test.sh      # one suite
FAIL_UNDER_LINE=90 firmware/scripts/coverage.sh    # tighter gate locally
```

## Adding a test — do not edit `CMakeLists.txt`

Registration is zero-touch so that module work can proceed in parallel without edit
conflicts. Drop a file in this directory and it is picked up:

| Rule | Detail |
|---|---|
| Filename | `test_<module>.c`, one per `core/` module, in this directory (not a subdirectory) |
| CTest name | `<module>` — the `test_` prefix is stripped |
| Build | The file is compiled with every `core/**/*.c`, every `support/*.c`, and Unity |
| Includes | `-I firmware/app/src/core` and `-I firmware/tests/host/support`, so `#include "util/crc.h"` and `#include "test_support.h"` both work |
| Flags | `-std=c11 -Wall -Wextra -Werror`; `COVERAGE=ON` adds `--coverage -O0 -g` |
| `main()` | Each file provides its own, using the Unity harness pattern below |
| Fixtures | `setUp`/`tearDown` default to weak no-ops from `support/test_support.c`; define strong ones in your file only if you need them |

The globs use `CONFIGURE_DEPENDS`, so a plain `cmake --build` notices a new file. If your
editor created the file while a configure was in flight, re-run `scripts/test.sh`.

Every core source is linked into every test binary. That is deliberate: a module with no
test still emits zero-count coverage records, so it pulls the gate down instead of being
invisible to it.

### Harness pattern

```c
#include "unity.h"
#include "util/crc.h"

static void test_crc32_known_answer(void)
{
	TEST_ASSERT_EQUAL_HEX32(0xCBF43926U, crc32_ieee("123456789", 9));
}

int main(void)
{
	UNITY_BEGIN();
	RUN_TEST(test_crc32_known_answer);
	return UNITY_END();
}
```

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
