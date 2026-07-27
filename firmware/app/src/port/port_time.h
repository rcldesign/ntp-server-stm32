/* Time sources crossing the core boundary. See port.h for conventions. */
#ifndef STS1000_PORT_TIME_H_
#define STS1000_PORT_TIME_H_

#include <stdint.h>

/*
 * Timescales:
 *  - "mono_ms": monotonic milliseconds since boot (k_uptime_get on target).
 *    Used for debounce, timeouts, schedulers. Never jumps.
 *  - "tai_ns": TAI nanoseconds since PTP epoch (1970-01-01 TAI). The canonical
 *    system time, sourced from the ETH PTP clock on target. UTC = TAI - leap.
 *  - NTP timestamps (era-0 seconds since 1900-01-01 UTC) are produced by
 *    core/ntp helpers from tai_ns + leap offset; core never asks the port
 *    for NTP time directly.
 */
typedef struct {
	uint64_t (*mono_ms)(void *ctx);
	/* Current TAI time. Returns 0 on success. */
	int (*tai_ns)(void *ctx, uint64_t *out_ns);
	void *ctx;
} port_clock_t;

#endif /* STS1000_PORT_TIME_H_ */
