# `src/core/` — platform-neutral application logic

Everything here is **plain C11 with no Zephyr, HAL or vendor headers.** This is
the rule that makes the ≥80 % host line-coverage gate in `ARCHITECTURE.md §9`
achievable: `tests/host/` compiles these files with the host toolchain.

Constraints (`ARCHITECTURE.md §4`):

* No Zephyr headers, no HAL, no globals holding hardware state.
* A module reaches the outside world only through the interfaces in
  `../port/` or through caller-supplied function pointers / context structs.
* One directory per module, one prefix per directory (`ntp_`, `mcp_`, `disc_`,
  …). All state lives in a caller-owned `*_ctx` struct.
* No dynamic allocation — fixed pools are supplied by the Zephyr glue.
* Return `int`: 0, or a negative errno-style code. No assertion that would kill
  the host test runner.
* `core/<mod>/<mod>.h` is the module's public API; internal headers stay
  private. Cross-module includes are allowed only for `util`, `cfg` (read) and
  `quality`.

Module inventory and ownership: `ARCHITECTURE.md §5`.

Every `.c` file under this directory is picked up automatically by
`../../CMakeLists.txt` (recursive glob), so a new module needs no CMake edit —
but it does need `tests/host/test_<mod>.c`.
