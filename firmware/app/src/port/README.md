# `src/port/` — the boundary between `core/` and the world

Headers only. Each file declares the interface that a `core/` module uses to
reach hardware, time, storage or crypto, so that `core/` never includes a Zephyr
or HAL header (`ARCHITECTURE.md §4`).

Two implementations exist for every interface:

| Consumer | Implementation |
|---|---|
| Target firmware | `../zephyr/portz/` — real drivers, DMA, PSA/mbedTLS, sockets |
| Host unit tests | `../../../tests/host/support/` — deterministic fakes the tests drive |

Rules:

* Interfaces are function-pointer tables or plain function declarations taking
  an opaque context — never a global.
* Anything non-deterministic (time, entropy, I/O completion) must come through
  a port, otherwise the corresponding `core/` module cannot be tested.
* No `.c` files here. Adding one would link platform code into `core/`'s
  dependency set.
