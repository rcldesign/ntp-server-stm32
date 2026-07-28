/*
 * STS1000 "Meridian" — multi-IC firmware-update glue: binds core/fwupd's
 * targets to the board.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * core/fwupd owns the state machine, the guards and the progress model; this
 * file owns the wires. One fwupd_target_ops_t per component:
 *
 *   STM32_APP        -> sts_dfu_port() (flash_map + MCUboot), i.e. exactly the
 *                       engine the MCP FW_* commands already use. Not
 *                       reimplemented. It is also the only target that can be
 *                       read back, so it is the only one that carries
 *                       fwupd_target_ops_t::readback — see stm_readback().
 *   RB_FE5680A       -> core/fwupd/rb_fwupd over rb_serial_ops() (UART7).
 *   GNSS_ZED_F9T     -> core/fwupd/ubx_fwupd over USART3. **See the seam note
 *                       below: the transport is wired, the opcodes are not
 *                       verified, and the allow bitmap is shut.**
 *   read-only parts  -> identity reads, where a bus is reachable from here.
 *
 * ---------------------------------------------------------------------------
 * The USART3 seam, as built
 * ---------------------------------------------------------------------------
 *
 * ubx_fwupd needs exclusive raw byte access to USART3 plus GPS_SAFEBOOT_N and
 * GPS_RST_N. USART3 is owned by src/zephyr/platform/gnss.c: it holds the ISR,
 * the parser and the gnssmgr instance. Handing the port to a firmware update
 * therefore needs a suspend/resume pair *in that file*, so the seam is declared
 * here as five __weak hooks that default to -ENOTSUP and platform/gnss.c
 * overrides them:
 *
 *   int sts_gnss_uart_suspend(void)      stop the ISR/parser, drop gnssmgr into
 *                                        GNSSMGR_ST_FW_UPDATE via
 *                                        gnssmgr_fw_enter()
 *   int sts_gnss_uart_resume(void)       restart, and gnssmgr_fw_exit() to
 *                                        re-run the configuration walk
 *   int sts_gnss_uart_raw_tx(buf, len)   poll-out bytes; returns the COUNT
 *   int sts_gnss_uart_raw_rx(buf, cap)   drain received bytes, non-blocking
 *   int sts_gnss_uart_set_baud(baud)     reconfigure USART3
 *
 * **All five are implemented** (platform/gnss.c, "USART3 seam" section). The
 * weak defaults survive only for builds without the platform area, and
 * gnss_transport() below is what tells the two cases apart at runtime.
 *
 * What is still shut is one layer up: ubx_fwupd_cfg_t::opcodes_verified is
 * false (the loader protocol is a reconstruction, see ubx_fwupd.h) AND
 * FWUPD_COMP_GNSS_ZED_F9T is absent from the allow bitmap. Either gate alone
 * refuses fwupd_begin(); see sts_fwupd_init() for why neither is opened here
 * and why there is no runtime path that opens them.
 *
 * ---------------------------------------------------------------------------
 * Two transmit conventions meet here, and this file is where they are reconciled
 * ---------------------------------------------------------------------------
 *
 * core/fwupd's ubx_fwupd_ops_t::tx is specified "0 on success". The platform
 * sink is sts_gnss_uart_raw_tx(), which returns the OCTET COUNT — and has to
 * keep returning it, because mp_tunnel.c's host->device path reports that count
 * to the maintenance host (mp_tunnel.c already documents the disagreement from
 * its own side). The conversion therefore belongs at the adapter, and it is
 * sts_fwupd_seam_policy.h's sts_fwupd_tx_from_count().
 *
 * The rubidium side does NOT need it: rb_serial.c's op_tx() already answers 0
 * on success, which is what rb_fwupd_ops_t::tx is specified to do. Stated so
 * that the asymmetry between ubx_op_tx() and rb_serial_ops() below reads as a
 * decision rather than an oversight.
 *
 * ---------------------------------------------------------------------------
 * Threading: one mutex, and why it has to be here
 * ---------------------------------------------------------------------------
 *
 * The orchestrator has two drivers on two threads, exactly as the MP engine
 * does:
 *
 *   RPC    console RX thread (the shell bypass) -> mp_rpc.c's `fw.*` handlers
 *          -> the mp_fwupd_t port below, inside mp_glue.c's engine lock;
 *   step   console supervisor, prio 14 -> sts_fwupd_step() every 250 ms, which
 *          is what enforces core/fwupd's step and transfer-idle timeouts;
 *   shell  `sts fw ...`, console RX thread.
 *
 * `fwupd_ctx_t` is no more internally synchronised than `mp_ctx_t` is, and core
 * never takes a lock, so `g.lock` is taken by **every** entry point in this file
 * that touches `g.fw`. The RPC path therefore nests engine lock -> `g.lock`, and
 * nothing ever takes them the other way round: sts_fwupd_step() takes only
 * `g.lock`, and nothing under it calls back into the MP engine. That is the
 * whole reason mp_wiring_t carries an mp_fwupd_t port rather than a bare
 * `fwupd_ctx_t *` — the pointer would put core's calls outside any lock.
 *
 * sts_fwupd_step() tries the lock with K_NO_WAIT and skips the pass on
 * contention. Not a preference: sts_console.c's BUILD_ASSERT budgets exactly one
 * lock wait per supervisor pass (sts_mp_tick()'s), and adding a second would
 * push the override dead-man's worst-case revert past MP_TICK_MAX_MS. A skipped
 * step costs 250 ms against timeouts measured in tens of seconds.
 *
 * Spending no lock wait is only half of what that assert needs, though: a pass
 * can also run long without waiting for anything, because fwupd_step() fires the
 * step and transfer-idle timeouts and those reach finish() -> t->restore(). The
 * second half of the premise is therefore a bounded-work budget,
 * STS_FWUPD_STEP_BUDGET_MS, which sts_fwupd_step() measures against and reports.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/toolchain.h>

#include "console/mp_glue.h"
#include "console/sts_console.h"
#include "console/sts_fwupd_seam_policy.h"
#include "console/sts_rollback.h"
#include "fwupd/fwupd.h"
#include "mp/mp.h"
#include "fwupd/rb_fwupd.h"
#include "fwupd/ubx_fwupd.h"
#include "port/port_crypto.h"
#include "zephyr/platform/platform.h"
#include "zephyr/storage/sts_store.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_fwupd, CONFIG_STS1000_LOG_LEVEL);

#define ZEPHYR_USER DT_PATH(zephyr_user)

/* ===================================================================== *
 *  The USART3 seam — weak defaults, see the file header
 * ===================================================================== */

__weak int sts_gnss_uart_suspend(void)
{
	return -ENOTSUP;
}

__weak int sts_gnss_uart_resume(void)
{
	return -ENOTSUP;
}

__weak int sts_gnss_uart_raw_tx(const uint8_t *buf, size_t len)
{
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
	return -ENOTSUP;
}

__weak int sts_gnss_uart_raw_rx(uint8_t *buf, size_t cap)
{
	ARG_UNUSED(buf);
	ARG_UNUSED(cap);
	return -ENOTSUP;
}

__weak int sts_gnss_uart_set_baud(uint32_t baud)
{
	ARG_UNUSED(baud);
	return -ENOTSUP;
}

/**
 * What the USART3 raw transport can do right now.
 *
 * Probing rather than a compile-time test: the strong symbols may be linked in
 * without this file knowing, and a zero-length read is harmless to ask.
 *
 * The three-way classification matters. `!= -ENOTSUP` — which is what this used
 * to be — folds "the hooks are absent" together with "the hooks are there and
 * the GNSS thread has not started", and the second answers -ENODEV. That made
 * the probe report a usable transport before platform/gnss.c had run, which
 * turned gnss_prepare()'s documented fail-safe into a refusal that could never
 * fire and made the boot log claim a wiring state it had not established.
 */
static sts_fwupd_xport_t gnss_transport(void)
{
	return sts_fwupd_xport_classify(sts_gnss_uart_raw_rx(NULL, 0U));
}

/** True only when raw access to USART3 is possible in this pass. */
static bool gnss_transport_ready(void)
{
	return gnss_transport() == STS_FWUPD_XPORT_READY;
}

/* ===================================================================== *
 *  Shared state
 * ===================================================================== */

static struct {
	bool ready;
	fwupd_ctx_t fw;
	rb_ctx_t rb;
	ubx_fwupd_t ubx;
	struct k_mutex lock;
	/*
	 * The STM32 staging erase window. See stm_ensure_erased().
	 *
	 * `stm_erased_to` is the cursor: slot 1 is blank below it and unknown
	 * above. `stm_erase_gran` and `stm_erase_ceiling` are sampled once, by
	 * stm_prepare(), so a chunk costs one comparison rather than two port
	 * calls. All three belong to g.lock like everything else in here, and
	 * are only ever touched from the STM32 target's ops.
	 */
	uint32_t stm_erased_to;
	uint32_t stm_erase_gran;
	uint32_t stm_erase_ceiling;
	/*
	 * The inventory snapshot the paged MP reply walks.
	 *
	 * Filled by a `from: 0` page and served from here afterwards, so paging
	 * eleven components probes the board once instead of three times — the
	 * rows behind it are identity reads on real buses. `inv_n` is 0 until the
	 * first page, which is why a mid-list page with no snapshot refills it
	 * rather than serving zeroes.
	 */
	fwupd_inv_row_t inv[FWUPD_COMP__COUNT];
	size_t inv_n;
} g;

/**
 * Take the orchestrator lock, or report that it is busy.
 *
 * K_FOREVER on the RPC path is correct — the caller is a console request and has
 * nothing better to do — but the supervisor's step must never wait, so the
 * timeout is a parameter rather than a policy baked in here.
 */
static int fw_lock(k_timeout_t t)
{
	if (!g.ready) {
		return -ENODEV;
	}
	if (k_is_in_isr()) {
		return -EBUSY;
	}
	return (k_mutex_lock(&g.lock, t) == 0) ? 0 : -EBUSY;
}

static void fw_unlock(void)
{
	(void)k_mutex_unlock(&g.lock);
}

/* ===================================================================== *
 *  STM32_APP — delegate to the existing MCUboot slot engine
 * ===================================================================== */

static int stm_query(void *user, char *out, size_t cap)
{
	port_image_info_t info;
	const port_image_t *img = sts_dfu_port();
	int rc;

	ARG_UNUSED(user);

	if ((img == NULL) || (img->image_info == NULL)) {
		return -ENODEV;
	}
	rc = img->image_info(img->ctx, 0U, &info);
	if (rc != 0) {
		return rc;
	}
	(void)snprintk(out, cap, "%u.%u.%u+%u", (unsigned int)info.version[0],
		       (unsigned int)info.version[1], (unsigned int)info.version[2],
		       (unsigned int)info.version[3]);
	return 0;
}

/**
 * Blank slot 1 up to one granule past @p need_end, and no further.
 *
 * NOTHING under port_image_t::staging_write() erases. sts_dfu.c's write path is
 * a write-block aligner over flash_area_write(); its only erases are the
 * trailer page that mark_pending()/request_revert() blank. So the staging erase
 * is this seam's job, and core/mcp's FW_* path has always done it here too
 * (mcp_dfu.c ensure_erased(), same port, same window, same look-ahead). Skipping
 * it does not corrupt anything — the STM32H5 flash controller raises a
 * programming error on a non-blank quad-word, so the write fails at the first
 * chunk — but it makes the MP `fw.*` path work exactly once per board, on a slot
 * that has never been staged.
 *
 * WINDOW SIZE, since this runs on the console RX thread inside the Maintenance
 * Protocol's engine lock, which sts_console.c's dead-man BUILD_ASSERT budgets —
 * the same call path as the read-back in fwupd.c, and the same budget:
 *
 *   one granule of look-ahead means AT MOST ONE 8 KiB sector erase per
 *   fwupd_data(), because stm_chunk_max() caps a chunk at 1024 octets — an
 *   eighth of a sector — and the cursor already stands a granule beyond the
 *   previous chunk. sts_fwupd_erase_target() carries the general bound;
 *
 *   an H5 sector erase is a few milliseconds, so a `fw.data` call costs that
 *   plus the 1024-octet program and the 1024-octet read-back it already had.
 *   Against STS_MP_TICK_LOCK_MS/STS_MP_TICK_MISS_MAX, which give the override
 *   dead-man roughly 1.5 s of engine-lock hold before its revert deadline is at
 *   risk, that is two orders of magnitude of margin;
 *
 *   the alternative — blanking the whole 896 KiB slot at prepare() — is 110
 *   sector erases inside a single `fw.begin`, i.e. hundreds of milliseconds to
 *   seconds in one uninterruptible hold. That is what this window exists to
 *   avoid, and stm_prepare() pays only the FIRST sector so the first chunk does
 *   not have to.
 *
 * The cursor is also what makes a rewind cheap and safe: a retransmit, or a
 * session restarted at the same size, computes a target at or below where the
 * cursor already stands and erases nothing, so accepted data is never blanked
 * out from under the tool.
 *
 * Caller holds g.lock (every entry point in this file does).
 */
static int stm_ensure_erased(const port_image_t *img, uint32_t need_end)
{
	uint32_t target = sts_fwupd_erase_target(need_end, g.stm_erase_gran,
						 g.stm_erase_ceiling);
	int rc;

	if (target <= g.stm_erased_to) {
		return 0;
	}
	rc = img->staging_erase(img->ctx, g.stm_erased_to,
				target - g.stm_erased_to);
	if (rc != 0) {
		LOG_ERR("stm fw: staging erase [0x%x, 0x%x) failed: %d",
			(unsigned int)g.stm_erased_to, (unsigned int)target, rc);
		return rc;
	}
	g.stm_erased_to = target;
	return 0;
}

static int stm_prepare(void *user, uint32_t size)
{
	const port_image_t *img = sts_dfu_port();
	uint32_t cap;

	ARG_UNUSED(user);

	if ((img == NULL) || (img->staging_erase == NULL) ||
	    (img->staging_size == NULL)) {
		return -ENODEV;
	}
	/*
	 * A zero granularity would make every erase length a non-multiple and
	 * sts_dfu.c would refuse all of them. It cannot happen — sts_dfu.c falls
	 * back to a fixed page size when the driver cannot describe the slot —
	 * but the window arithmetic below is only meaningful with a real one, so
	 * it is stated as a precondition rather than assumed.
	 */
	g.stm_erase_gran = sts_dfu_erase_granularity();
	if (g.stm_erase_gran == 0U) {
		return -ENODEV;
	}
	cap = img->staging_size(img->ctx);
	if (size > cap) {
		return -ENOSPC;
	}

	/*
	 * Erase incrementally, in transfer(), one granule ahead of the write —
	 * see stm_ensure_erased() for why the window is that size and what it
	 * costs. The ceiling is fixed here because neither the image size nor the
	 * slot capacity can change while a session is open.
	 */
	g.stm_erase_ceiling = sts_fwupd_erase_ceiling(size, cap,
						      g.stm_erase_gran);
	g.stm_erased_to = 0U;

	/*
	 * Clear the first window now, so the first chunk does not arrive to find
	 * an unerased sector — exactly what core/mcp's FW_BEGIN does, and the
	 * reason the per-chunk worst case is one sector rather than two.
	 */
	return stm_ensure_erased(img, 0U);
}

static int stm_transfer(void *user, uint32_t off, const uint8_t *data, size_t len,
			bool last)
{
	const port_image_t *img = sts_dfu_port();
	int rc;

	ARG_UNUSED(user);

	if ((img == NULL) || (img->staging_write == NULL) ||
	    (img->staging_erase == NULL)) {
		return -ENODEV;
	}
	/*
	 * Ahead of the write, never after it: staging_write() programs, and a
	 * program into flash this seam has not blanked is a PGSERR on the H5.
	 * core/fwupd guarantees `off` is the frontier and `off + len` is within
	 * the declared size, so the sum cannot overflow and cannot exceed the
	 * ceiling stm_prepare() fixed.
	 */
	rc = stm_ensure_erased(img, off + (uint32_t)len);
	if (rc != 0) {
		return rc;
	}
	return img->staging_write(img->ctx, off, data, len, last);
}

/**
 * The read-back half of core/fwupd's integrity check (fwupd.h, "Two hashes").
 *
 * The SAME port call core/mcp's verify_hash() uses on the SAME slot — not a
 * second reader written for this path. sts_dfu.c's dfu_staging_read() takes
 * dfu_lock for the transfer and overlays any sub-block tail its write aligner
 * is still holding, so a chunk that is not a multiple of the 16-octet write
 * block reads back correct rather than erased; the final chunk is flushed by
 * transfer(..., last=true) before core asks for it, so the end of the image is
 * always compared against committed flash.
 *
 * This is the only target that has it. The GNSS loader and the FE-5680A expose
 * no read path, so their ops tables leave `readback` NULL and core skips the
 * whole check for them — which is the designed answer, not a gap (fwupd.h).
 */
static int stm_readback(void *user, uint32_t off, uint8_t *out, size_t len)
{
	const port_image_t *img = sts_dfu_port();

	ARG_UNUSED(user);

	if ((img == NULL) || (img->staging_read == NULL)) {
		return -ENODEV;
	}
	return img->staging_read(img->ctx, off, out, len);
}

static int stm_verify(void *user, char *out, size_t cap)
{
	port_image_info_t info;
	const port_image_t *img = sts_dfu_port();
	int rc;

	ARG_UNUSED(user);

	if ((img == NULL) || (img->image_info == NULL) ||
	    (img->mark_pending == NULL)) {
		return -ENODEV;
	}

	/* Slot 1 must now parse as an MCUboot image before it is marked. */
	rc = img->image_info(img->ctx, 1U, &info);
	if (rc != 0) {
		return rc;
	}
	if (!info.valid) {
		return -EBADMSG;
	}

	/*
	 * Anti-rollback, checked HERE rather than left to the bootloader.
	 *
	 * MCUboot compares the staged image's IMAGE_TLV_SEC_CNT against the
	 * running one and, when the staged value is lower or missing, erases
	 * slot 1 and boots the primary unchanged
	 * (loader.c check_downgrade_prevention()). That happens at the next
	 * boot, in a bootloader built with CONFIG_MCUBOOT_LOG_LEVEL_OFF, so
	 * without this the operator gets "verified, pending", reboots, and finds
	 * the same version running with nothing anywhere saying why.
	 *
	 * Refusing before mark_pending() also leaves slot 1 unmarked, so the
	 * restore path below is not needed to unwind a doomed swap.
	 */
	rc = sts_rollback_check_staged();
	if (rc != 0) {
		return rc;
	}

	rc = img->mark_pending(img->ctx);
	if (rc != 0) {
		return rc;
	}

	/*
	 * The staged version, not the running one: the swap happens on the next
	 * boot, and MCUboot's own signature check is what decides whether it
	 * sticks — the anti-rollback half of that decision has already been
	 * reproduced above. Reporting the running version here would tell the
	 * operator the update had not taken.
	 */
	(void)snprintk(out, cap, "%u.%u.%u+%u (pending)",
		       (unsigned int)info.version[0],
		       (unsigned int)info.version[1],
		       (unsigned int)info.version[2],
		       (unsigned int)info.version[3]);
	return 0;
}

static int stm_restore(void *user, bool after_failure)
{
	const port_image_t *img = sts_dfu_port();

	ARG_UNUSED(user);

	if (!after_failure) {
		return 0;
	}
	/*
	 * A failed staging attempt must not leave slot 1 marked pending, or the
	 * next reboot tries to swap a partial image. request_revert() clears the
	 * trailer; MCUboot would refuse the image anyway, but leaving it armed
	 * costs a boot cycle and an operator's afternoon.
	 */
	if ((img != NULL) && (img->request_revert != NULL)) {
		(void)img->request_revert(img->ctx);
	}
	return 0;
}

static uint32_t stm_chunk_max(void *user)
{
	ARG_UNUSED(user);
	return 1024U;
}

/* ===================================================================== *
 *  GNSS_ZED_F9T — ubx_fwupd over USART3
 * ===================================================================== */

static int ubx_op_tx(void *user, const uint8_t *d, size_t len)
{
	ARG_UNUSED(user);

	/*
	 * The one place the two transmit conventions meet (see the file header).
	 * ubx_fwupd_ops_t::tx is "0 on success"; sts_gnss_uart_raw_tx() returns
	 * the octet count and must keep doing so for mp_tunnel.c's sake. Passing
	 * the count straight through made ubx_fwupd.c read every successful frame
	 * as a failure — and then propagate the byte count as if it were an
	 * errno, so a caller checking for -EPERM saw 8.
	 */
	return sts_fwupd_tx_from_count(sts_gnss_uart_raw_tx(d, len), len);
}

static int ubx_op_rx(void *user, uint8_t *d, size_t cap)
{
	ARG_UNUSED(user);
	return sts_gnss_uart_raw_rx(d, cap);
}

static int ubx_op_set_baud(void *user, uint32_t baud)
{
	ARG_UNUSED(user);
	return sts_gnss_uart_set_baud(baud);
}

static int ubx_op_safeboot(void *user, bool assert_low)
{
	static const struct gpio_dt_spec sb =
		GPIO_DT_SPEC_GET(ZEPHYR_USER, gps_safeboot_gpios);

	ARG_UNUSED(user);

	/*
	 * The devicetree marks GPS_SAFEBOOT_N GPIO_ACTIVE_LOW, so a logical 1 is
	 * an electrical low — which is what "assert safeboot" means.
	 */
	return gpio_pin_set_dt(&sb, assert_low ? 1 : 0);
}

static int ubx_op_reset(void *user, bool assert_low)
{
	static const struct gpio_dt_spec rst =
		GPIO_DT_SPEC_GET(ZEPHYR_USER, gps_rst_gpios);

	ARG_UNUSED(user);

	/* GPS_RST_N is GPIO_ACTIVE_LOW in the devicetree, as above. */
	return gpio_pin_set_dt(&rst, assert_low ? 1 : 0);
}

static uint64_t ubx_op_now(void *user)
{
	ARG_UNUSED(user);
	return (uint64_t)k_uptime_get();
}

static void ubx_op_delay(void *user, uint32_t ms)
{
	ARG_UNUSED(user);
	k_msleep((int32_t)ms);
}

static const ubx_fwupd_ops_t ubx_ops = {
	.tx = ubx_op_tx,
	.rx = ubx_op_rx,
	.set_baud = ubx_op_set_baud,
	.set_safeboot = ubx_op_safeboot,
	.set_reset = ubx_op_reset,
	.now_ms = ubx_op_now,
	.delay_ms = ubx_op_delay,
	.user = NULL,
};

static int gnss_query(void *user, char *out, size_t cap)
{
	sts_fwupd_query_act_t act;

	ARG_UNUSED(user);

	if (cap > 0U) {
		out[0] = '\0';
	}

	/*
	 * A MON-VER poll needs the raw byte path, because the gnss thread's own
	 * parser feeds gnssmgr and not us. That raw path is shared, and this
	 * function is reachable at G0 through `fw.inventory` — FMT §5.1 makes
	 * observation free, correctly, but observation must not mean "writes to
	 * a port another session owns".
	 *
	 * So the poll is gated on both halves of "may I drive USART3 right now":
	 * the transport has to be usable, and no passthrough tunnel may hold the
	 * port. Without the second test an unauthenticated `fw.inventory` injects
	 * an 8-octet UBX-MON-VER poll into the middle of a technician's session —
	 * possibly into a receiver's bootloader — and then spends its 2 s reply
	 * budget draining that session's bytes out of the tee ring.
	 *
	 * Either refusal reports an empty version with rc 0 — "present, nothing
	 * to read" — which the maintenance tool renders differently from
	 * "absent". Inventing a version string would be worse than admitting the
	 * gap.
	 *
	 * A GNSS update session needs no test here: ubx_fwupd_query_version()
	 * answers -EBUSY while one is open, which lands on the same empty row.
	 *
	 * TODO: publish the decoded UBX-MON-VER in sts_gnss_snap_t so this is
	 * readable without touching the UART at all; that field belongs to
	 * platform/gnss.c, and it is what would finally give the row a value —
	 * the poll below can only succeed while the port is suspended, which is
	 * exactly the state this function refuses to work in.
	 */
	act = sts_fwupd_query_decide(gnss_transport_ready(),
				     sts_mp_tunnel_gnss_open());
	if (act != STS_FWUPD_QUERY_POLL) {
		LOG_DBG("gnss fw: inventory read skipped (%s)",
			sts_fwupd_query_act_name(act));
		return 0;
	}
	if (ubx_fwupd_query_version(&g.ubx, out, cap) != 0) {
		if (cap > 0U) {
			out[0] = '\0';
		}
	}
	return 0;
}

static int gnss_prepare(void *user, uint32_t size)
{
	int rc;

	ARG_UNUSED(user);

	if (!gnss_transport_ready()) {
		LOG_ERR("gnss fw: USART3 raw transport not usable (%s) — refusing",
			sts_fwupd_xport_name(gnss_transport()));
		return -ENOTSUP;
	}
	if (sts_mp_tunnel_gnss_open()) {
		/*
		 * A passthrough tunnel already stood the receiver down and the
		 * host owns the byte path. Suspending again is a no-op, but
		 * resuming at the end of the session would hand USART3 back to
		 * gnssmgr underneath a tunnel that still believes it holds it —
		 * two writers on one UART, which is the condition FMT §5.5
		 * exists to prevent.
		 */
		LOG_ERR("gnss fw: a passthrough tunnel holds USART3 — refusing");
		return -EBUSY;
	}

	/*
	 * Take the port away from the gnss thread first. Only then may safeboot be
	 * asserted: a reset pulse with the ISR still running would feed loader
	 * bytes into the UBX parser and gnssmgr's retry logic into a flash loader.
	 */
	rc = sts_gnss_uart_suspend();
	if (rc != 0) {
		return rc;
	}

	rc = ubx_fwupd_begin(&g.ubx, size);
	if (rc != 0) {
		(void)sts_gnss_uart_resume();
		return rc;
	}
	return 0;
}

static int gnss_transfer(void *user, uint32_t off, const uint8_t *data, size_t len,
			 bool last)
{
	ARG_UNUSED(user);
	ARG_UNUSED(last);
	return ubx_fwupd_write(&g.ubx, off, data, len);
}

static int gnss_verify(void *user, char *out, size_t cap)
{
	const ubx_mon_ver_t *v;
	int rc;

	ARG_UNUSED(user);

	rc = ubx_fwupd_finish(&g.ubx);
	if (rc != 0) {
		return rc;
	}
	v = ubx_fwupd_version(&g.ubx);
	if (v == NULL) {
		return -EIO;
	}
	{
		const char *fw = ubx_mon_ver_ext(v, "FWVER");

		(void)snprintk(out, cap, "%s%s%s", v->sw_version,
			       (fw != NULL) ? " " : "", (fw != NULL) ? fw : "");
	}
	return 0;
}

static int gnss_restore(void *user, bool after_failure)
{
	int rc;
	int first = 0;

	ARG_UNUSED(user);

	/*
	 * Belt and braces. ubx_fwupd's own failure paths already recover, but this
	 * runs on EVERY exit including an abort from a state ubx_fwupd never
	 * entered, and leaving GPS_SAFEBOOT_N asserted is the one outcome that
	 * makes the unit look dead.
	 */
	if (after_failure || ubx_fwupd_in_safeboot(&g.ubx)) {
		rc = ubx_fwupd_recover(&g.ubx);
		if (rc != 0) {
			LOG_ERR("gnss fw: recover failed: %d — receiver may be "
				"held in safeboot", rc);
			first = rc;
		}
	}
	if (ubx_fwupd_in_safeboot(&g.ubx)) {
		LOG_ERR("gnss fw: still in safeboot after recovery");
		if (first == 0) {
			first = -EIO;
		}
	}

	/* Give the port back; gnssmgr_fw_exit() re-runs the config walk (§8.5). */
	rc = sts_gnss_uart_resume();
	if ((rc != 0) && (first == 0)) {
		first = rc;
	}
	return first;
}

static uint32_t gnss_chunk_max(void *user)
{
	ARG_UNUSED(user);
	return UBX_FWUPD_CHUNK;
}

/* ===================================================================== *
 *  RB_FE5680A — rb_fwupd over UART7
 * ===================================================================== */

static int rb_query(void *user, char *out, size_t cap)
{
	sts_fwupd_query_act_t act;

	ARG_UNUSED(user);

	if (cap > 0U) {
		out[0] = '\0';
	}

	/*
	 * The same G0 hazard as gnss_query(), on the other UART, and it bites
	 * even though rb_serial.c's op_tx() already answers -EBUSY under a
	 * tunnel: rb_fwupd_identify() probes when the capability class is still
	 * UNKNOWN, and rb_fwupd_probe() LATCHES RB_CAP_NONE when nothing answers.
	 * A single unauthenticated `fw.inventory` taken while a technician holds
	 * UART7 would therefore pin the rubidium at "nothing answered — check the
	 * cable" for the rest of the uptime, with no re-probe path.
	 *
	 * `transport_ready` is true unconditionally because there is no probe
	 * hook on this side: rb_serial.c owns UART7 directly and its ops answer
	 * -ENODEV for themselves when the port or the Rb rail is down. What this
	 * seam knows, and rb_serial.c deliberately does not (see its header),
	 * is that a tunnel is a reason not to ask at all.
	 */
	act = sts_fwupd_query_decide(true, sts_mp_tunnel_rb_open());
	if (act != STS_FWUPD_QUERY_POLL) {
		LOG_DBG("rb fw: inventory read skipped (%s)",
			sts_fwupd_query_act_name(act));
		return 0;
	}
	return rb_fwupd_identify(&g.rb, out, cap);
}

static int rb_prepare(void *user, uint32_t size)
{
	ARG_UNUSED(user);
	/*
	 * Always -ENOTSUP for every variant this project has documentation for.
	 * rb_fwupd_begin() is the single place that decides, and it says no unless
	 * a loader has been verified AND the probe found one.
	 */
	return rb_fwupd_begin(&g.rb, size);
}

static int rb_transfer(void *user, uint32_t off, const uint8_t *data, size_t len,
		       bool last)
{
	ARG_UNUSED(user);
	ARG_UNUSED(off);
	ARG_UNUSED(data);
	ARG_UNUSED(len);
	ARG_UNUSED(last);
	/* Unreachable: prepare() refuses first. Present so the orchestrator sees a
	 * complete target and reports -ENOTSUP rather than "read-only". */
	return -ENOTSUP;
}

static int rb_verify(void *user, char *out, size_t cap)
{
	ARG_UNUSED(user);
	ARG_UNUSED(out);
	ARG_UNUSED(cap);
	return -ENOTSUP;
}

static int rb_restore(void *user, bool after_failure)
{
	ARG_UNUSED(user);
	ARG_UNUSED(after_failure);
	return 0;
}

/* ===================================================================== *
 *  Read-only components
 * ===================================================================== */

/*
 * These rows exist so the maintenance tool can render a COMPLETE board
 * inventory: "updatable: no" is only a useful answer next to what can be read
 * instead (fwupd.h). Each returns an empty string rather than a fabricated one
 * when its bus is not reachable from this thread — an empty version with rc 0
 * means "present, nothing to read", which the tool renders differently from
 * "absent".
 *
 * The I²C and SPI identity reads are deliberately NOT performed here: the
 * housekeeping thread owns the I²C1 bus sweep and SPI4 has a per-CS
 * reconfiguration protocol (ARCHITECTURE.md §10.4), so reading them from the
 * console thread would need bus arbitration that belongs to those areas. What is
 * wired is the row, the descriptor and the designator; the value arrives when
 * the owning area publishes a snapshot accessor. TODO in the report.
 */
static int ro_query(void *user, char *out, size_t cap)
{
	ARG_UNUSED(user);

	if (cap > 0U) {
		out[0] = '\0';
	}
	return 0;
}

/* ===================================================================== *
 *  Callbacks
 * ===================================================================== */

static void cb_event(void *user, const fwupd_event_t *e)
{
	ARG_UNUSED(user);

	LOG_INF("fwupd %s: %s %u/%u (%u.%u%%)", fwupd_comp_name(e->comp),
		fwupd_state_name(e->state), (unsigned int)e->done,
		(unsigned int)e->total, (unsigned int)(e->permille / 10U),
		(unsigned int)(e->permille % 10U));
}

static void cb_audit(void *user, uint8_t comp, const char *what, int rc)
{
	ARG_UNUSED(user);

	/*
	 * Every transition and every guarded refusal, at notice level or above.
	 * This is the trail that answers "who reflashed the GNSS receiver".
	 */
	if (rc == 0) {
		LOG_INF("fwupd audit: %s %s", fwupd_comp_name(comp), what);
	} else {
		LOG_WRN("fwupd audit: %s %s rc=%d", fwupd_comp_name(comp), what, rc);
	}
}

static void cb_degraded(void *user, uint8_t comp, bool on)
{
	ARG_UNUSED(user);

	if (on) {
		LOG_WRN("fwupd: timing DEGRADED — %s is being reprogrammed",
			fwupd_comp_name(comp));
	} else {
		LOG_INF("fwupd: timing degradation cleared (%s)",
			fwupd_comp_name(comp));
	}
	/*
	 * TODO(integration): also publish this into the §3.8 quality block so NTP
	 * and PTP advertise the degradation rather than only logging it. The
	 * quality publisher is owned by the discipline area; the hook belongs
	 * there, not here.
	 */
}

/* ===================================================================== *
 *  Bring-up
 * ===================================================================== */

int sts_fwupd_init(void)
{
	fwupd_cfg_t cfg;
	fwupd_cb_t cb;
	rb_fwupd_cfg_t rbcfg;
	ubx_fwupd_cfg_t ubxcfg;
	int rc;

	if (g.ready) {
		return -EALREADY;
	}
	k_mutex_init(&g.lock);

	/* --- the FE-5680A client ---------------------------------------- */
	rb_fwupd_cfg_defaults(&rbcfg);
	rc = rb_fwupd_init(&g.rb, &rbcfg, rb_serial_ops());
	if (rc != 0) {
		LOG_ERR("rb_fwupd_init: %d", rc);
		return rc;
	}

	/* --- the u-blox loader client ----------------------------------- */
	ubx_fwupd_cfg_defaults(&ubxcfg);
	/*
	 * opcodes_verified stays FALSE. The safeboot loader protocol is an
	 * informed reconstruction, not a specification (ubx_fwupd.h), and arming
	 * it is a deliberate act by whoever verifies it against real u-blox
	 * tooling. While it is false ubx_fwupd_begin() refuses without touching a
	 * pin, so the receiver cannot be stranded in safeboot.
	 */
	rc = ubx_fwupd_init(&g.ubx, &ubxcfg, &ubx_ops);
	if (rc != 0) {
		LOG_ERR("ubx_fwupd_init: %d", rc);
		return rc;
	}

	/* --- the orchestrator ------------------------------------------- */
	fwupd_cfg_defaults(&cfg);
	/*
	 * Only the STM32 application image is permitted, and — stated plainly,
	 * because it is a product decision and not an unfinished edge —
	 * **there is no runtime path that widens this.**
	 *
	 * fwupd_set_cfg() is the only way to replace the bitmap after
	 * fwupd_init(), and nothing calls it: not the shell, not `fw.*`, not the
	 * config registry. That is deliberate. The STM32 path is the one with a
	 * signature check and an automatic revert behind it (MCUboot). The GNSS
	 * and Rb paths destroy a peripheral with no way back — and on the GNSS
	 * side the loader opcodes are an informed reconstruction rather than a
	 * specification (ubx_fwupd.h) — so widening the bitmap must be a
	 * recompile by somebody who has read both files, not an operator action
	 * reachable from a maintenance session. The FMT advertises the G3 guard
	 * those components would need if the bitmap were ever opened; today the
	 * bitmap is the outer gate and it is shut.
	 *
	 * If that is ever revisited, the widening path is a config key plus a
	 * fwupd_set_cfg() call under g.lock — and it needs the GNSS opcode table
	 * verified first, or fwupd_begin() will refuse anyway.
	 */
	(void)fwupd_cfg_allow(&cfg, (uint8_t)FWUPD_COMP_STM32_APP, true);

	(void)memset(&cb, 0, sizeof(cb));
	cb.event = cb_event;
	cb.audit = cb_audit;
	cb.degraded = cb_degraded;

	rc = fwupd_init(&g.fw, &cfg, &cb, sts_port_sha256_stream());
	if (rc != 0) {
		LOG_ERR("fwupd_init: %d", rc);
		return rc;
	}

	/* --- targets ---------------------------------------------------- */
	{
		static const fwupd_target_ops_t stm = {
			.query_version = stm_query,
			.prepare = stm_prepare,
			.transfer = stm_transfer,
			.readback = stm_readback,
			.verify = stm_verify,
			.restore = stm_restore,
			.chunk_max = stm_chunk_max,
		};
		static const fwupd_target_ops_t gnss = {
			.query_version = gnss_query,
			.prepare = gnss_prepare,
			.transfer = gnss_transfer,
			.verify = gnss_verify,
			.restore = gnss_restore,
			.chunk_max = gnss_chunk_max,
		};
		static const fwupd_target_ops_t rb = {
			.query_version = rb_query,
			.prepare = rb_prepare,
			.transfer = rb_transfer,
			.verify = rb_verify,
			.restore = rb_restore,
		};
		static const fwupd_target_ops_t ro = {
			.query_version = ro_query,
		};
		static const uint8_t ro_comps[] = {
			(uint8_t)FWUPD_COMP_PHY_LAN8742,
			(uint8_t)FWUPD_COMP_SE_ATECC608B,
			(uint8_t)FWUPD_COMP_INA228,
			(uint8_t)FWUPD_COMP_TMP117,
			(uint8_t)FWUPD_COMP_DISPLAY_ST7796,
			(uint8_t)FWUPD_COMP_TOUCH_FT6336U,
			(uint8_t)FWUPD_COMP_DIGIPOT_MCP41U83,
			(uint8_t)FWUPD_COMP_PD_NCP1095,
		};
		size_t i;

		(void)fwupd_set_target(&g.fw, (uint8_t)FWUPD_COMP_STM32_APP, &stm);
		(void)fwupd_set_target(&g.fw, (uint8_t)FWUPD_COMP_GNSS_ZED_F9T,
				       &gnss);
		(void)fwupd_set_target(&g.fw, (uint8_t)FWUPD_COMP_RB_FE5680A, &rb);
		for (i = 0U; i < ARRAY_SIZE(ro_comps); i++) {
			(void)fwupd_set_target(&g.fw, ro_comps[i], &ro);
		}
	}

	g.ready = true;
	/*
	 * The transport state is reported as the probe found it, not as a
	 * boolean. main() starts the platform area before the console one, so
	 * "wired" is the expected answer here and "wired, area not started"
	 * means sts_gnss_start() failed — which is worth seeing in the boot log
	 * rather than being rounded to the same word as "absent".
	 */
	LOG_INF("fwupd ready: %d components, STM32 app updatable, "
		"GNSS transport %s (opcodes unverified, allow bit shut), "
		"Rb not field-updatable",
		(int)FWUPD_COMP__COUNT,
		sts_fwupd_xport_name(gnss_transport()));
	return 0;
}

fwupd_ctx_t *sts_fwupd_ctx(void)
{
	return g.ready ? &g.fw : NULL;
}

rb_ctx_t *sts_fwupd_rb_ctx(void)
{
	return g.ready ? &g.rb : NULL;
}

/**
 * Longest sts_fwupd_step() seen so far, milliseconds.
 *
 * Written and read only by the console supervisor, which is the only caller, so
 * it needs no lock — and it is deliberately outside `g`, whose members all
 * belong to g.lock.
 */
static uint32_t step_worst_ms;

int sts_fwupd_step(void)
{
	uint64_t t0;
	uint32_t spent;
	int rc;

	/*
	 * K_NO_WAIT. See the threading block: sts_console.c's BUILD_ASSERT
	 * budgets one lock wait per supervisor pass and sts_mp_tick() already
	 * spends it. Contention here means a console request is inside the
	 * orchestrator right now, which is the one situation in which nothing
	 * needs pumping.
	 */
	rc = fw_lock(K_NO_WAIT);
	if (rc != 0) {
		return rc;
	}
	t0 = (uint64_t)k_uptime_get();
	rc = fwupd_step(&g.fw, t0);
	spent = (uint32_t)((uint64_t)k_uptime_get() - t0);
	fw_unlock();

	/*
	 * The other half of the same BUILD_ASSERT's premise, made falsifiable.
	 *
	 * Not spending a lock wait does not make a pass short: fwupd_step()'s
	 * timeout paths call finish() -> t->restore(), which is an internal-flash
	 * trailer write for the STM32 target (reachable today: `fw.begin` then
	 * silence for the 60 s transfer-idle timeout) and, once the GNSS allow bit
	 * is ever opened, 20 ms of k_msleep inside ubx_fwupd_recover().
	 *
	 * Reported at error level on each new high-water above the budget, so it
	 * is bounded and so the first bench run that exceeds it says so by name
	 * rather than showing up as an unexplained dead-man revert.
	 */
	if (spent > step_worst_ms) {
		step_worst_ms = spent;
		if (spent > STS_FWUPD_STEP_BUDGET_MS) {
			LOG_ERR("fwupd step ran %u ms, over the %u ms supervisor "
				"budget (sts_console.c BUILD_ASSERT)",
				(unsigned int)spent,
				(unsigned int)STS_FWUPD_STEP_BUDGET_MS);
		}
	}
	return rc;
}

/* ===================================================================== *
 *  The MP control plane's port (mp.h mp_fwupd_t)
 *
 *  One thin locked wrapper per orchestrator call. Nothing here decides
 *  anything: the guards, the allow-list and the state machine are core's, and
 *  duplicating any of them at the seam would give the device two answers.
 * ===================================================================== */

static int mpfw_inventory(void *ctx, size_t first, fwupd_inv_row_t *out,
			  size_t max, size_t *n, size_t *total)
{
	size_t avail;
	size_t take;
	int rc;

	ARG_UNUSED(ctx);

	if ((out == NULL) || (n == NULL) || (total == NULL)) {
		return -EINVAL;
	}
	*n = 0U;
	*total = (size_t)FWUPD_COMP__COUNT;

	rc = fw_lock(K_FOREVER);
	if (rc != 0) {
		return rc;
	}

	if ((first == 0U) || (g.inv_n == 0U)) {
		size_t got = 0U;

		/*
		 * -ENOSPC only means the caller's array was short; the array here
		 * is FWUPD_COMP__COUNT long, so it cannot happen and a non-zero
		 * return is a real failure.
		 */
		rc = fwupd_inventory(&g.fw, g.inv, ARRAY_SIZE(g.inv), &got);
		if (rc != 0) {
			g.inv_n = 0U;
			fw_unlock();
			return rc;
		}
		g.inv_n = got;
	}

	avail = (first < g.inv_n) ? (g.inv_n - first) : 0U;
	take = (avail < max) ? avail : max;
	if (take != 0U) {
		(void)memcpy(out, &g.inv[first], take * sizeof(out[0]));
	}
	*n = take;
	*total = g.inv_n;
	fw_unlock();
	return 0;
}

static int mpfw_begin(void *ctx, uint8_t comp, const fwupd_req_t *req)
{
	int rc;

	ARG_UNUSED(ctx);

	rc = fw_lock(K_FOREVER);
	if (rc != 0) {
		return rc;
	}
	rc = fwupd_begin(&g.fw, comp, req, (uint64_t)k_uptime_get());
	fw_unlock();
	return rc;
}

static int mpfw_data(void *ctx, uint32_t off, const uint8_t *d, size_t len,
		     uint32_t *next_off, bool *duplicate)
{
	fwupd_event_t before;
	uint32_t local_next = 0U;
	int rc;

	ARG_UNUSED(ctx);

	if (next_off == NULL) {
		next_off = &local_next;
	}
	if (duplicate != NULL) {
		*duplicate = false;
	}

	rc = fw_lock(K_FOREVER);
	if (rc != 0) {
		return rc;
	}

	/*
	 * Read `done` and write it in ONE acquisition of the orchestrator's
	 * mutex. That is the whole reason the duplicate flag is decided here and
	 * not by the caller: fwupd_data() returns 0 both for a chunk it wrote
	 * and for a retransmit it discarded, and `next_off` is identical in the
	 * two cases whenever the retransmit ends exactly where the write offset
	 * already was. Only "did `done` move" separates them, and asking through
	 * a second locked status() call would be asking about a different
	 * instant.
	 */
	(void)fwupd_progress(&g.fw, &before);
	rc = fwupd_data(&g.fw, off, d, len, next_off, (uint64_t)k_uptime_get());
	if ((rc == 0) && (duplicate != NULL)) {
		*duplicate = (*next_off == before.done);
	}
	fw_unlock();
	return rc;
}

static int mpfw_end(void *ctx)
{
	int rc;

	ARG_UNUSED(ctx);

	rc = fw_lock(K_FOREVER);
	if (rc != 0) {
		return rc;
	}
	rc = fwupd_end(&g.fw, (uint64_t)k_uptime_get());
	fw_unlock();
	return rc;
}

static int mpfw_abort(void *ctx)
{
	int rc;

	ARG_UNUSED(ctx);

	rc = fw_lock(K_FOREVER);
	if (rc != 0) {
		return rc;
	}
	rc = fwupd_abort(&g.fw, (uint64_t)k_uptime_get());
	/* fwupd_abort() leaves the session in DONE/FAILED so the reason is
	 * readable; the caller asked to be rid of it, so return to IDLE too. */
	if (rc == 0) {
		(void)fwupd_reset(&g.fw);
	}
	fw_unlock();
	return rc;
}

static int mpfw_reset(void *ctx)
{
	int rc;

	ARG_UNUSED(ctx);

	rc = fw_lock(K_FOREVER);
	if (rc != 0) {
		return rc;
	}
	rc = fwupd_reset(&g.fw);
	fw_unlock();
	return rc;
}

static int mpfw_status(void *ctx, mp_fw_status_t *out)
{
	int rc;

	ARG_UNUSED(ctx);

	if (out == NULL) {
		return -EINVAL;
	}
	(void)memset(out, 0, sizeof(*out));

	rc = fw_lock(K_FOREVER);
	if (rc != 0) {
		return rc;
	}
	(void)fwupd_progress(&g.fw, &out->progress);
	(void)snprintk(out->before, sizeof(out->before), "%s",
		       fwupd_version_before(&g.fw));
	(void)snprintk(out->after, sizeof(out->after), "%s",
		       fwupd_version_after(&g.fw));
	out->allow = g.fw.cfg.allow;
	out->chunk_max = FWUPD_CHUNK_MAX;
	{
		/*
		 * The active target's own ceiling when there is one, so `fw.data`
		 * advertises the limit that will actually be enforced rather than
		 * the orchestrator's outer bound.
		 */
		uint8_t comp = out->progress.comp;

		if ((comp < (uint8_t)FWUPD_COMP__COUNT) && g.fw.target_set[comp] &&
		    (g.fw.target[comp].chunk_max != NULL)) {
			uint32_t m =
				g.fw.target[comp].chunk_max(g.fw.target[comp].user);

			if ((m != 0U) && (m < out->chunk_max)) {
				out->chunk_max = m;
			}
		}
	}
	out->chunks_duplicate = g.fw.chunks_duplicate;
	out->chunks_rejected = g.fw.chunks_rejected;
	out->sessions = g.fw.sessions;
	out->sessions_ok = g.fw.sessions_ok;
	out->sessions_failed = g.fw.sessions_failed;
	fw_unlock();
	return 0;
}

const mp_fwupd_t *sts_fwupd_mp_port(void)
{
	static const mp_fwupd_t port = {
		.inventory = mpfw_inventory,
		.begin = mpfw_begin,
		.data = mpfw_data,
		.end = mpfw_end,
		.abort = mpfw_abort,
		.reset = mpfw_reset,
		.status = mpfw_status,
		.ctx = NULL,
	};

	return g.ready ? &port : NULL;
}

/* ===================================================================== *
 *  Locked read accessors for the shell
 * ===================================================================== */

int sts_fwupd_inventory(fwupd_inv_row_t *out, size_t max, size_t *n)
{
	int rc;

	if ((out == NULL) || (n == NULL)) {
		return -EINVAL;
	}
	*n = 0U;

	rc = fw_lock(K_FOREVER);
	if (rc != 0) {
		return rc;
	}
	rc = fwupd_inventory(&g.fw, out, max, n);
	fw_unlock();
	return rc;
}

int sts_fwupd_query(uint8_t comp, char *out, size_t cap)
{
	int rc;

	if ((out == NULL) || (cap == 0U)) {
		return -EINVAL;
	}
	out[0] = '\0';

	rc = fw_lock(K_FOREVER);
	if (rc != 0) {
		return rc;
	}
	rc = fwupd_query(&g.fw, comp, out, cap);
	fw_unlock();
	return rc;
}

int sts_fwupd_status(mp_fw_status_t *out)
{
	return mpfw_status(NULL, out);
}
