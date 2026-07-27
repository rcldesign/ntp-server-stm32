/*
 * STS1000 "Meridian" — the Meridian Console Protocol channel on CDC-ACM #1.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * ARCHITECTURE.md §6/§7: ACM1 is the binary channel tools/meridian_ctl.py
 * speaks. core/mcp is the protocol; this file is the wire underneath it and
 * nothing else — a byte pump, a tick, and the wiring table that hands the
 * engine its ports.
 *
 * The transmit contract is the part that has to be got right (mcp.h): a
 * response must never be dropped, an event always may be. Both fall out of one
 * rule here — the tx callback either takes a whole frame into the TX ring or
 * refuses it — because core/mcp already knows which class it is holding. A
 * refused RSP is retained and re-offered by mcp_poll_tx(); a refused EVT is
 * counted and forgotten.
 *
 * Frames are never split across the ring boundary check: `ring_buf_space_get()`
 * is tested before `ring_buf_put()`, and only the mcp thread ever puts, so the
 * space can only grow between the two. A half-written frame would desynchronise
 * the COBS stream until the next delimiter, which is precisely the failure the
 * all-or-nothing rule exists to prevent.
 *
 * Link loss is a session event, not just a transport one. DTR dropping means
 * the host closed the port: mcp_reset_session() then de-authenticates, cancels
 * subscriptions, drops any staged config and — importantly — clears a response
 * that is waiting for a link that no longer exists, which would otherwise stall
 * the engine's input path forever. The DFU session deliberately survives.
 */

#include <zephyr/kernel.h>

#ifdef CONFIG_STS1000_CONSOLE

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

#include "console/sts_console.h"
#include "mcp/mcp.h"
#include "port/port_time.h"
#include "storage/sts_store.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_mcp, CONFIG_STS1000_LOG_LEVEL);

/* --------------------------------------------------------------- tunables */

#if defined(CONFIG_STS1000_MCP_THREAD_STACK)
#define MCP_STACK CONFIG_STS1000_MCP_THREAD_STACK
#else
#define MCP_STACK 4096
#endif

#if defined(CONFIG_STS1000_MCP_RX_RING)
#define MCP_RX_RING CONFIG_STS1000_MCP_RX_RING
#else
#define MCP_RX_RING 1024
#endif

#if defined(CONFIG_STS1000_MCP_TX_RING)
#define MCP_TX_RING CONFIG_STS1000_MCP_TX_RING
#else
#define MCP_TX_RING 2048
#endif

/** ARCHITECTURE.md §6: the `mcp` thread runs at priority 14. */
#define MCP_PRIO 14

/** Longest the loop sleeps with nothing to do; also the mcp_tick() cadence. */
#define MCP_TICK_MS 20

/** Brief in-callback retry before a frame is declared back-pressured. */
#define MCP_TX_RETRY_SLICES 4
#define MCP_TX_RETRY_MS     2

/* The TX ring must hold one whole encoded frame, or a maximum-size response
 * could never be transmitted at all and the engine would stall permanently. */
BUILD_ASSERT(MCP_TX_RING >= (int)(COBS_ENCODE_MAX(MCP_MAX_FRAME) + 1U),
	     "CONFIG_STS1000_MCP_TX_RING cannot hold a maximum MCP frame");

/* ------------------------------------------------------------------ state */

static const struct device *const mcp_uart =
	DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart1));

static mcp_ctx_t mcp;
static struct k_thread mcp_thread;
static K_THREAD_STACK_DEFINE(mcp_stack, MCP_STACK);
static K_SEM_DEFINE(mcp_evt, 0, 1);

static uint8_t rx_ring_buf[MCP_RX_RING];
static uint8_t tx_ring_buf[MCP_TX_RING];
static struct ring_buf rx_rb;
static struct ring_buf tx_rb;

/* Counters shared with the ISR. Reads are unsynchronised on purpose: these are
 * diagnostics, and a torn 32-bit counter is cheaper than a lock on the RX path. */
static sts_mcp_link_stats_t link_stats;

static bool mcp_started;
static bool link_up;          /* DTR seen asserted */
static int  mcp_liveness_id = -1;
static atomic_t link_down_pending = ATOMIC_INIT(0);

/* ------------------------------------------------------------------ ports */

static uint64_t clk_mono_ms(void *ctx)
{
	ARG_UNUSED(ctx);
	return (uint64_t)k_uptime_get();
}

static int clk_tai_ns(void *ctx, uint64_t *out_ns)
{
	ARG_UNUSED(ctx);
	return sts_time_tai_ns(out_ns);
}

static const port_clock_t mcp_clock = {
	.mono_ms = clk_mono_ms,
	.tai_ns = clk_tai_ns,
	.ctx = NULL,
};

static int mcp_status_cb(void *user, uint8_t group, uint8_t *buf, size_t cap)
{
	ARG_UNUSED(user);
	return sts_status_encode(group, buf, cap);
}

/* ------------------------------------------------------------------ config */

/*
 * The engine reaches the ONE live cfg_ctx_t, which the Zephyr shell backend and
 * the ui_local thread also write. cfg.h is explicit that the context is not
 * internally locked, so the engine borrows this area's mutual exclusion for the
 * whole of any request that touches config (mcp.h "Config locking").
 */
static void mcp_cfg_lock(void *user)
{
	ARG_UNUSED(user);
	sts_cfg_lock();
}

static void mcp_cfg_unlock(void *user)
{
	ARG_UNUSED(user);
	sts_cfg_unlock();
}

/*
 * Commits go through sts_cfg_commit(), not cfg_commit(): it is the only thing
 * that runs the per-group config appliers, and it is the same call the local
 * shell and the panel UI make. Without it `meridian_ctl.py cfg-set && cfg-commit`
 * wrote the tree and NVS, answered "applied=N, reboot_groups=0", and left the
 * log ring, the syslog sender, the network services and the display exactly as
 * they were.
 */
static int mcp_cfg_commit(void *user, cfg_commit_res_t *res)
{
	ARG_UNUSED(user);
	return sts_cfg_commit(res);
}

/*
 * And the same for FACTORY_RESET, for the same reason with a wider blast radius:
 * cfg_factory_reset() alone rewrote every key and erased NVS while the log ring,
 * the syslog sender, the network services, the timing loop and the display all
 * carried on with the configuration they had been handed before the reset — until
 * somebody rebooted the unit. sts_cfg_factory_reset() runs every registered
 * group's appliers.
 */
static int mcp_cfg_factory_reset(void *user)
{
	ARG_UNUSED(user);
	return sts_cfg_factory_reset();
}

/* ------------------------------------------------------------------ ISR */

static void mcp_uart_isr(const struct device *dev, void *user)
{
	ARG_UNUSED(user);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t tmp[64];
			int n = uart_fifo_read(dev, tmp, sizeof(tmp));

			if (n > 0) {
				uint32_t put = ring_buf_put(&rx_rb, tmp,
							    (uint32_t)n);

				link_stats.rx_bytes += put;
				if (put < (uint32_t)n) {
					link_stats.rx_overruns +=
						(uint32_t)n - put;
				}
				k_sem_give(&mcp_evt);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t *p = NULL;
			uint32_t n = ring_buf_get_claim(&tx_rb, &p, 64U);

			if (n == 0U) {
				(void)ring_buf_get_finish(&tx_rb, 0U);
				uart_irq_tx_disable(dev);
			} else {
				int sent = uart_fifo_fill(dev, p, (int)n);

				if (sent < 0) {
					sent = 0;
				}
				(void)ring_buf_get_finish(&tx_rb,
							  (uint32_t)sent);
				link_stats.tx_bytes += (uint32_t)sent;
				if (sent > 0) {
					k_sem_give(&mcp_evt);
				}
			}
		}
	}
}

/* ------------------------------------------------------------- transmit */

static int mcp_tx(void *user, const uint8_t *buf, size_t len)
{
	int slice;

	ARG_UNUSED(user);

	if ((buf == NULL) || (len == 0U)) {
		return -EINVAL;
	}
	if (len > ring_buf_capacity_get(&tx_rb)) {
		/* Cannot ever be sent; refusing forever would wedge the engine. */
		LOG_ERR("frame of %zu B exceeds the %u B TX ring", len,
			(unsigned int)ring_buf_capacity_get(&tx_rb));
		return -EMSGSIZE;
	}

	for (slice = 0; slice < MCP_TX_RETRY_SLICES; slice++) {
		if (ring_buf_space_get(&tx_rb) >= (uint32_t)len) {
			(void)ring_buf_put(&tx_rb, buf, (uint32_t)len);
			uart_irq_tx_enable(mcp_uart);
			return 0;
		}

		/* Make sure the drain is actually running before waiting on it. */
		uart_irq_tx_enable(mcp_uart);

		/* No host attached means no drain is coming; do not spend the
		 * engine's time on a link that is not there. */
		if (!link_up) {
			break;
		}
		k_sleep(K_MSEC(MCP_TX_RETRY_MS));
	}

	link_stats.tx_backpressure++;
	return -EAGAIN;
}

/* ------------------------------------------------------------- identity */

static void fill_ident(mcp_ident_t *id)
{
	uint8_t uid[16];
	uint8_t digest[32];
	ssize_t n;

	memset(id, 0, sizeof(*id));
	(void)memcpy(id->model, "STS1000", 7);

	n = hwinfo_get_device_id(uid, sizeof(uid));
	if (n <= 0) {
		LOG_WRN("no hardware device ID (%d); board_id left zero", (int)n);
		return;
	}

	/*
	 * Publish a digest rather than the raw STM32 UID: it is stable, it is
	 * the same length for every part, and the die coordinates in the raw
	 * value are not something to broadcast on a management channel.
	 */
	if (sts_port_crypto()->sha256(NULL, uid, (size_t)n, digest) == 0) {
		memcpy(id->board_id, digest, sizeof(id->board_id));
	} else {
		memcpy(id->board_id, uid,
		       MIN(sizeof(id->board_id), (size_t)n));
	}
}

/* ------------------------------------------------------------------ link */

void sts_mcp_notify_link_down(void)
{
	atomic_set(&link_down_pending, 1);
	k_sem_give(&mcp_evt);
}

static bool read_dtr(void)
{
	uint32_t dtr = 0U;

	if (uart_line_ctrl_get(mcp_uart, UART_LINE_CTRL_DTR, &dtr) != 0) {
		return false;
	}
	return dtr != 0U;
}

static void track_link(void)
{
	bool now = read_dtr();

	if (atomic_cas(&link_down_pending, 1, 0)) {
		now = false;
	}

	if (now == link_up) {
		link_stats.dtr = now;
		return;
	}

	link_up = now;
	link_stats.dtr = now;

	if (now) {
		LOG_INF("MCP host attached");
	} else {
		link_stats.link_drops++;
		/*
		 * Quiesce the ISR before touching its rings: a DTR drop (the
		 * host merely closing the port) leaves USB enumerated and the
		 * RX interrupt live, so resetting the rings underneath it would
		 * race. TX is re-enabled lazily by the next mcp_tx(); RX is
		 * restored here so the next host can talk.
		 */
		uart_irq_rx_disable(mcp_uart);
		uart_irq_tx_disable(mcp_uart);
		mcp_reset_session(&mcp);
		ring_buf_reset(&tx_rb);
		ring_buf_reset(&rx_rb);
		uart_irq_rx_enable(mcp_uart);
		LOG_INF("MCP host detached; session reset");
	}
}

/* ------------------------------------------------------------------ drain */

static void drain_rx(void)
{
	/*
	 * Bound the work per pass. A host that keeps the pipe full must not be
	 * able to hold this thread inside one drain indefinitely; the tick and
	 * the liveness feed have to keep happening.
	 */
	for (unsigned int pass = 0U; pass < 8U; pass++) {
		uint8_t *p = NULL;
		uint32_t avail;
		size_t consumed = 0U;

		avail = ring_buf_get_claim(&rx_rb, &p, MCP_RX_RING);
		if (avail == 0U) {
			(void)ring_buf_get_finish(&rx_rb, 0U);
			return;
		}

		(void)mcp_input(&mcp, p, (size_t)avail, &consumed);
		(void)ring_buf_get_finish(&rx_rb, (uint32_t)consumed);

		if (consumed == 0U) {
			/* Back-pressured on a held response. mcp_tick() will
			 * retry the transmit; come back next pass. */
			return;
		}
	}
}

/* ----------------------------------------------------------------- thread */

static void mcp_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("MCP channel up on %s (prio %d)", mcp_uart->name, MCP_PRIO);

	for (;;) {
		(void)k_sem_take(&mcp_evt, K_MSEC(MCP_TICK_MS));

		track_link();
		drain_rx();
		(void)mcp_tick(&mcp, (uint64_t)k_uptime_get());

		if (mcp_liveness_id >= 0) {
			sts_liveness_feed(mcp_liveness_id);
		}
	}
}

/* ------------------------------------------------------------------- API */

const mcp_stats_t *sts_mcp_stats(void)
{
	return mcp_started ? mcp_stats(&mcp) : NULL;
}

void sts_mcp_link_stats(sts_mcp_link_stats_t *out)
{
	if (out != NULL) {
		*out = link_stats;
	}
}

int sts_mcp_start(void)
{
	mcp_wiring_t w;
	k_tid_t tid;
	int rc;

	if (mcp_started) {
		return 0;
	}

	if (!device_is_ready(mcp_uart)) {
		LOG_ERR("%s not ready: the MCP channel is unavailable",
			mcp_uart->name);
		return -ENODEV;
	}

	ring_buf_init(&rx_rb, sizeof(rx_ring_buf), rx_ring_buf);
	ring_buf_init(&tx_rb, sizeof(tx_ring_buf), tx_ring_buf);

	memset(&w, 0, sizeof(w));
	w.clock = &mcp_clock;
	w.crypto = sts_port_crypto();
	w.sha = sts_port_sha256_stream();
	w.img = sts_dfu_port();
	w.cfg = sts_cfg();
	w.cfg_lock = mcp_cfg_lock;
	w.cfg_unlock = mcp_cfg_unlock;
	w.cfg_commit_cb = mcp_cfg_commit;
	w.cfg_factory_cb = mcp_cfg_factory_reset;
	w.log = sts_logring();
	w.status_cb = mcp_status_cb;
	w.diag_cb = sts_diag_encode;
	w.tx = mcp_tx;
	w.dfu_erase_gran = sts_dfu_erase_granularity();
	/* Report the true flash write-block (16 B on STM32H5). Core keeps every
	 * non-final FW_DATA chunk a multiple of it, so the DFU port only ever
	 * buffers the final short block. */
	w.dfu_write_block = sts_dfu_write_block();
	fill_ident(&w.ident);

	rc = mcp_init(&mcp, &w);
	if (rc != 0) {
		LOG_ERR("mcp_init failed (%d)", rc);
		return rc;
	}

	uart_irq_rx_disable(mcp_uart);
	uart_irq_tx_disable(mcp_uart);
	uart_irq_callback_user_data_set(mcp_uart, mcp_uart_isr, NULL);
	uart_irq_rx_enable(mcp_uart);

	mcp_liveness_id = sts_liveness_register("mcp");
	if (mcp_liveness_id < 0) {
		LOG_WRN("no liveness slot for the MCP thread (%d)",
			mcp_liveness_id);
	}

	mcp_started = true;

	tid = k_thread_create(&mcp_thread, mcp_stack,
			      K_THREAD_STACK_SIZEOF(mcp_stack),
			      mcp_thread_entry, NULL, NULL, NULL, MCP_PRIO, 0,
			      K_NO_WAIT);
	k_thread_name_set(tid, "mcp");

	return 0;
}

#endif /* CONFIG_STS1000_CONSOLE */
