/*
 * STS1000 "Meridian" — console area: host->device passthrough routing policy.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr-free by construction, for the same reason `sts_ppscorr.h` and
 * `sts_secops_policy.h` are: every way this decision can be wrong is silent.
 * Routing channel 0x08 to USART3 would put FE-5680A commands into a GNSS
 * receiver; accepting a burst while the tunnel is shut would make firmware and
 * the maintenance host two writers on the same UART, which is the exact
 * condition FMT §5.5 exists to prevent; and letting an unbounded burst through
 * turns a `uart_poll_out()` loop under the engine lock into a host-controlled
 * stall of the service tick. None of the three fails loudly at runtime.
 *
 * The decision is a pure function of (channel, which tunnels are open, length),
 * so mp_tunnel.c's implementation is a switch over the verdict and this header
 * is what the host suite exercises.
 */

#ifndef STS1000_ZEPHYR_CONSOLE_STS_MP_TUNNEL_POLICY_H_
#define STS1000_ZEPHYR_CONSOLE_STS_MP_TUNNEL_POLICY_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mp/mp_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

/** What the glue must do with an inbound passthrough burst. */
typedef enum {
	/** Hand it to sts_gnss_uart_raw_tx() — channel 0x07, USART3. */
	STS_MP_TX_GNSS = 0,
	/** Hand it to rb_serial_tunnel_write() — channel 0x08, UART7. */
	STS_MP_TX_RB,
	/** Nothing to write; not an error. */
	STS_MP_TX_NOTHING,
	/** Not a channel this device can write to (0x02/0x03 tees, 0x06 SMP). */
	STS_MP_TX_REFUSE_CHANNEL,
	/** The channel is right but its tunnel is not open (FMT §5.5). */
	STS_MP_TX_REFUSE_CLOSED,
	/** Longer than MP_TUNNEL_TX_MAX; refused whole, never truncated. */
	STS_MP_TX_REFUSE_OVERSIZE,
} sts_mp_tx_act_t;

/**
 * Decide what to do with @p len octets the host sent on channel @p ch.
 *
 * Order matters and is chosen so the refusal names the real obstacle:
 *
 *   1. the channel, because a host writing to the NMEA tee has misunderstood
 *      the protocol and telling it "closed" would send it looking for an
 *      override that does not exist;
 *   2. the tunnel, because an open port is the authorisation and it outranks
 *      the size of what is being written;
 *   3. emptiness, so a zero-length keepalive frame on an open tunnel is a
 *      no-op rather than an error;
 *   4. the burst bound, last, because it is the only refusal a well-behaved
 *      host can fix by chunking.
 *
 * @param ch        MP_CH_* from the received frame.
 * @param gnss_open sts_mp_tunnel_gnss_open().
 * @param rb_open   sts_mp_tunnel_rb_open().
 * @param len       Octets in the frame.
 * @param max       MP_TUNNEL_TX_MAX; passed in so the test can vary it.
 */
static inline sts_mp_tx_act_t sts_mp_tunnel_tx_decide(uint8_t ch, bool gnss_open,
						      bool rb_open, size_t len,
						      size_t max)
{
	bool open;

	switch (ch) {
	case MP_CH_GNSS_PASS:
		open = gnss_open;
		break;
	case MP_CH_RB_PASS:
		open = rb_open;
		break;
	default:
		/*
		 * 0x06 SMP included. `sys.smp.tunnel` is in the manifest and its
		 * override has no apply binding, so nothing can open it; routing
		 * bytes for it would be inventing a path.
		 */
		return STS_MP_TX_REFUSE_CHANNEL;
	}

	if (!open) {
		return STS_MP_TX_REFUSE_CLOSED;
	}
	if (len == 0U) {
		return STS_MP_TX_NOTHING;
	}
	if ((max == 0U) || (len > max)) {
		return STS_MP_TX_REFUSE_OVERSIZE;
	}
	return (ch == MP_CH_GNSS_PASS) ? STS_MP_TX_GNSS : STS_MP_TX_RB;
}

/** Short name for a verdict, for logs and test failures. Never NULL. */
static inline const char *sts_mp_tx_act_name(sts_mp_tx_act_t a)
{
	switch (a) {
	case STS_MP_TX_GNSS:
		return "gnss";
	case STS_MP_TX_RB:
		return "rb";
	case STS_MP_TX_NOTHING:
		return "nothing";
	case STS_MP_TX_REFUSE_CHANNEL:
		return "bad-channel";
	case STS_MP_TX_REFUSE_CLOSED:
		return "closed";
	case STS_MP_TX_REFUSE_OVERSIZE:
		return "oversize";
	default:
		return "?";
	}
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_CONSOLE_STS_MP_TUNNEL_POLICY_H_ */
