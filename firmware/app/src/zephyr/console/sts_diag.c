/*
 * STS1000 "Meridian" — MCP DIAG sub-functions (ARCHITECTURE.md §7, cmd 0x50).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Sub 0 (health snapshot) never reaches this file: core/mcp answers it from
 * the summary status group, because spec §10.5 defines device health once and
 * DIAG does not get a second definition of it.
 *
 * Implemented here:
 *
 *   1  I²C bus scan          u8 ver, u8 bus, u8 found, u8 map[16]
 *                            map is a 128-bit set of responding 7-bit
 *                            addresses, LSB of map[0] = address 0.
 *
 *   3  thread / CPU stats    u8 ver, u8 n,
 *                            n × { u8 prio(int8), u32 stack_size,
 *                                  u32 stack_unused, u8 name_len,
 *                                  char name[name_len] }
 *
 * Deferred to the areas that own the data, and answered -ENOTSUP until they
 * land: 2 (PPS residual histogram, timing area) and 4 (INA228 register dump,
 * platform area's housekeeping cache).
 *
 * tools/meridian_ctl.py prints DIAG payloads as hex, so these layouts are
 * documentation rather than a tool contract; nothing there needs changing.
 */

#include <zephyr/kernel.h>

#ifdef CONFIG_STS1000_CONSOLE

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay)
#include <zephyr/drivers/i2c.h>
#define HAVE_I2C1 1
#endif

#include "console/sts_console.h"

LOG_MODULE_REGISTER(sts_diag, CONFIG_STS1000_LOG_LEVEL);

#define DIAG_LAYOUT_VER 1U

/* Lowest and highest 7-bit addresses worth probing (I²C reserves the rest). */
#define I2C_SCAN_FIRST 0x08U
#define I2C_SCAN_LAST  0x77U

/* ------------------------------------------------------------------------- */
/* Sub 1 — I²C scan                                                          */
/* ------------------------------------------------------------------------- */

int sts_diag_i2c_scan(uint8_t found[16])
{
#if defined(HAVE_I2C1)
	const struct device *bus = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	uint8_t probe = 0U;
	int n = 0;

	if (found == NULL) {
		return -EINVAL;
	}
	memset(found, 0, 16);

	if (!device_is_ready(bus)) {
		return -ENODEV;
	}

	/*
	 * A zero-length write is the standard presence probe: the controller
	 * emits START, the address byte, and STOP, and nothing else. The
	 * housekeeping thread shares this bus, but the Zephyr I²C driver
	 * serialises transfers internally, so the scan interleaves with its
	 * sweep rather than corrupting it. At 400 kHz the whole sweep is well
	 * under a millisecond of bus time.
	 */
	for (uint16_t addr = I2C_SCAN_FIRST; addr <= I2C_SCAN_LAST; addr++) {
		if (i2c_write(bus, &probe, 0, addr) == 0) {
			found[addr / 8U] |= (uint8_t)(1U << (addr % 8U));
			n++;
		}
	}

	return n;
#else
	ARG_UNUSED(found);
	return -ENOTSUP;
#endif
}

static int diag_i2c(uint8_t *buf, size_t cap)
{
	uint8_t map[16];
	int n;

	if (cap < (3U + sizeof(map))) {
		return -ENOSPC;
	}

	n = sts_diag_i2c_scan(map);
	if (n < 0) {
		return n;
	}

	buf[0] = (uint8_t)DIAG_LAYOUT_VER;
	buf[1] = 1U; /* I²C1 */
	buf[2] = (uint8_t)n;
	memcpy(&buf[3], map, sizeof(map));
	return (int)(3U + sizeof(map));
}

/* ------------------------------------------------------------------------- */
/* Sub 3 — thread / CPU stats                                                */
/* ------------------------------------------------------------------------- */

#if defined(CONFIG_THREAD_MONITOR)

struct thread_pack {
	uint8_t *buf;
	size_t   cap;
	size_t   len;
	uint8_t  n;
};

static void thread_cb(const struct k_thread *thread, void *user)
{
	struct thread_pack *p = user;
	const char *name;
	size_t name_len;
	size_t need;
	size_t unused = 0U;
	uint32_t stack_size = 0U;

	if (p->n == 0xFFU) {
		return;
	}

	name = k_thread_name_get((k_tid_t)thread);
	if (name == NULL) {
		name = "?";
	}
	name_len = MIN(strlen(name), (size_t)15U);

	need = 1U + 4U + 4U + 1U + name_len;
	if ((p->cap - p->len) < need) {
		return; /* out of room; report what fits */
	}

#if defined(CONFIG_THREAD_STACK_INFO)
	stack_size = (uint32_t)thread->stack_info.size;
#endif
	if (k_thread_stack_space_get(thread, &unused) != 0) {
		unused = 0U;
	}

	p->buf[p->len++] = (uint8_t)(int8_t)k_thread_priority_get(
		(k_tid_t)thread);
	sys_put_le32(stack_size, &p->buf[p->len]);
	p->len += 4U;
	sys_put_le32((uint32_t)unused, &p->buf[p->len]);
	p->len += 4U;
	p->buf[p->len++] = (uint8_t)name_len;
	memcpy(&p->buf[p->len], name, name_len);
	p->len += name_len;
	p->n++;
}

static int diag_threads(uint8_t *buf, size_t cap)
{
	struct thread_pack p = {
		.buf = buf,
		.cap = cap,
		.len = 2U,
		.n = 0U,
	};

	if (cap < 2U) {
		return -ENOSPC;
	}

	/*
	 * The unlocked walk is deliberate: k_thread_foreach() holds the
	 * scheduler lock for the whole iteration, and this runs on the console
	 * thread while the discipline loop is servicing PPS. Every thread in
	 * this application is statically created and never terminates
	 * (ARCHITECTURE.md §6), so there is no list mutation to race with.
	 */
	k_thread_foreach_unlocked(thread_cb, &p);

	buf[0] = (uint8_t)DIAG_LAYOUT_VER;
	buf[1] = p.n;
	return (int)p.len;
}

#else /* !CONFIG_THREAD_MONITOR */

static int diag_threads(uint8_t *buf, size_t cap)
{
	ARG_UNUSED(buf);
	ARG_UNUSED(cap);
	return -ENOTSUP;
}

#endif /* CONFIG_THREAD_MONITOR */

/* ------------------------------------------------------------------------- */

int sts_diag_encode(void *user, uint8_t sub, uint8_t *buf, size_t cap)
{
	ARG_UNUSED(user);

	if ((buf == NULL) || (cap == 0U)) {
		return -EINVAL;
	}

	switch (sub) {
	case 1U:
		return diag_i2c(buf, cap);
	case 3U:
		return diag_threads(buf, cap);
	case 2U: /* PPS residual histogram — timing area */
	case 4U: /* INA228 register dump — platform housekeeping cache */
	default:
		/* TODO(wave-4): route 2 and 4 to the areas that own the data
		 * once sts_app.h grows an accessor for them. */
		return -ENOTSUP;
	}
}

#endif /* CONFIG_STS1000_CONSOLE */
