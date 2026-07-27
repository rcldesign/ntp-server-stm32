/*
 * STS1000 "Meridian" — core/mp: internals shared inside the module.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to core/mp/. Nothing outside the module may include this.
 */

#ifndef STS1000_CORE_MP_MP_INTERNAL_H_
#define STS1000_CORE_MP_MP_INTERNAL_H_

#include "mp/mp.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Monotonic milliseconds from the wiring's clock; 0 when unwired. */
uint32_t mp_now(const mp_ctx_t *c);

/**
 * Latch error detail for the reply currently being built.
 *
 * @param code  One of the MP_E_* codes.
 * @param what  Short machine-readable detail placed in `error.data.reason`, or
 *              NULL. Truncated to MP_ERR_DATA_MAX.
 */
void mp_fail(mp_ctx_t *c, int code, const char *what);

/** Transmit one framed message on @p ch; counts a failure. */
int mp_send(mp_ctx_t *c, uint8_t ch, const uint8_t *msg, size_t len);

/** Map an errno-style return from a subsystem onto an MP error code. */
int mp_map_errno(int rc);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_MP_MP_INTERNAL_H_ */
