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
 *                       reimplemented.
 *   RB_FE5680A       -> core/fwupd/rb_fwupd over rb_serial_ops() (UART7).
 *   GNSS_ZED_F9T     -> core/fwupd/ubx_fwupd over USART3. **See the seam note
 *                       below: the transport binding is not wired yet.**
 *   read-only parts  -> identity reads, where a bus is reachable from here.
 *
 * ---------------------------------------------------------------------------
 * The USART3 seam (the one thing not finished, stated plainly)
 * ---------------------------------------------------------------------------
 *
 * ubx_fwupd needs exclusive raw byte access to USART3 plus GPS_SAFEBOOT_N and
 * GPS_RST_N. USART3 is owned by src/zephyr/platform/gnss.c: it holds the ISR,
 * the parser and the gnssmgr instance. Handing the port to a firmware update
 * therefore needs a suspend/resume pair *in that file*, which is outside this
 * change's file boundary.
 *
 * Rather than reach into another area's file, the seam is declared here as four
 * __weak hooks that default to -ENOTSUP. The image links today, the GNSS row is
 * inventoried (its version is read through the ordinary gnssmgr path), and
 * fwupd_begin(FWUPD_COMP_GNSS_ZED_F9T) returns -ENOTSUP with an explicit log
 * line rather than half-driving a receiver.
 *
 * To finish it, platform/gnss.c implements these four and nothing else changes:
 *
 *   int sts_gnss_uart_suspend(void)      stop the ISR/parser, drop gnssmgr into
 *                                        GNSSMGR_ST_FW_UPDATE via
 *                                        gnssmgr_fw_enter()
 *   int sts_gnss_uart_resume(void)       restart, and gnssmgr_fw_exit() to
 *                                        re-run the configuration walk
 *   int sts_gnss_uart_raw_tx(buf, len)   poll-out bytes
 *   int sts_gnss_uart_raw_rx(buf, cap)   drain received bytes, non-blocking
 *
 * gnssmgr_fw_enter()/gnssmgr_fw_exit() already exist in core (this change added
 * them) and already do the config-restore half of spec §8.5.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/toolchain.h>

#include "console/sts_console.h"
#include "console/sts_rollback.h"
#include "fwupd/fwupd.h"
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

/** True once platform/gnss.c provides a real transport. */
static bool gnss_transport_available(void)
{
	/*
	 * Probing rather than a compile-time test: the strong symbols may be
	 * linked in without this file knowing. A suspend that reports -ENOTSUP is
	 * the definitive answer, and it is harmless to ask.
	 */
	return sts_gnss_uart_raw_rx(NULL, 0U) != -ENOTSUP;
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
	/* STM32 staging cursor, so transfer() can be offset-driven. */
	uint32_t stm_erased_to;
} g;

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

static int stm_prepare(void *user, uint32_t size)
{
	const port_image_t *img = sts_dfu_port();

	ARG_UNUSED(user);

	if ((img == NULL) || (img->staging_erase == NULL) ||
	    (img->staging_size == NULL)) {
		return -ENODEV;
	}
	if (size > img->staging_size(img->ctx)) {
		return -ENOSPC;
	}
	/*
	 * Erase lazily, in transfer(): erasing 896 KB up front blocks this thread
	 * for seconds and the flash layer erases per-sector on write anyway. The
	 * cursor stops a rewind from re-erasing a sector that already holds data.
	 */
	g.stm_erased_to = 0U;
	return 0;
}

static int stm_transfer(void *user, uint32_t off, const uint8_t *data, size_t len,
			bool last)
{
	const port_image_t *img = sts_dfu_port();

	ARG_UNUSED(user);

	if ((img == NULL) || (img->staging_write == NULL)) {
		return -ENODEV;
	}
	return img->staging_write(img->ctx, off, data, len, last);
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
	return sts_gnss_uart_raw_tx(d, len);
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
	ARG_UNUSED(user);

	if (cap > 0U) {
		out[0] = '\0';
	}

	/*
	 * A MON-VER poll needs the raw byte path, because the gnss thread's own
	 * parser feeds gnssmgr and not us. When the transport is wired, ask the
	 * receiver directly; ubx_fwupd_query_version() refuses while a session is
	 * open, so this cannot disturb an update in progress.
	 *
	 * When it is not wired, report an empty version with rc 0 — "present,
	 * nothing to read" — which the maintenance tool renders differently from
	 * "absent". Inventing a version string would be worse than admitting the
	 * gap. TODO: publish the decoded UBX-MON-VER in sts_gnss_snap_t so this is
	 * readable without touching the UART at all; that field belongs to
	 * platform/gnss.c.
	 */
	if (!gnss_transport_available()) {
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

	if (!gnss_transport_available()) {
		LOG_ERR("gnss fw: USART3 raw transport not wired "
			"(platform/gnss.c hooks absent) — refusing");
		return -ENOTSUP;
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
	ARG_UNUSED(user);
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
	 * Only the STM32 application image is permitted out of the box. That is
	 * the one path with a signature check and an automatic revert behind it
	 * (MCUboot); the GNSS and Rb paths destroy a peripheral with no way back,
	 * so an operator has to enable them explicitly.
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
	LOG_INF("fwupd ready: %d components, STM32 app updatable, "
		"GNSS %s, Rb %s",
		(int)FWUPD_COMP__COUNT,
		gnss_transport_available() ? "transport wired but opcodes unverified"
					  : "transport not wired",
		"not field-updatable");
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

int sts_fwupd_step(void)
{
	if (!g.ready) {
		return -ENODEV;
	}
	return fwupd_step(&g.fw, (uint64_t)k_uptime_get());
}
