/*
 * STS1000 Meridian — port interfaces (core ↔ platform boundary).
 *
 * Core modules (app/src/core/**) are platform-neutral: they never include
 * Zephyr/HAL headers. Anything that touches hardware, time, crypto, flash,
 * or the network is reached through the typedefs in port/*.h. On target the
 * implementations live in app/src/zephyr/portz/; host unit tests supply
 * fakes/fixtures.
 *
 * Conventions (binding, see ARCHITECTURE.md §4):
 *  - ops structs of function pointers + opaque void *ctx, passed at *_init().
 *  - int returns: 0 on success, negative errno-style on failure.
 *  - no dynamic allocation inside core; buffers are caller-owned.
 */
#ifndef STS1000_PORT_H_
#define STS1000_PORT_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Shared error codes used across the port boundary (negative). */
#define PORT_EIO        (-5)
#define PORT_EINVAL     (-22)
#define PORT_ENOSPC     (-28)
#define PORT_ETIMEDOUT  (-110)
#define PORT_ENOTSUP    (-95)
#define PORT_EBUSY      (-16)

#endif /* STS1000_PORT_H_ */
