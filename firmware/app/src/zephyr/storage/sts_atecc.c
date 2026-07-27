/*
 * STS1000 "Meridian" — ATECC608B glue: I²C1 transport + device services.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * core/atecc is the protocol (packet framing, the Microchip CRC-16, response
 * decode, the wake/idle/sleep tokens). This file is the transport it drives —
 * I²C1 at 0x60 (interface ref §4) — plus the small set of services the rest of
 * the firmware actually wants from a secure element:
 *
 *   sts_atecc_serial()   the factory serial number, for the SNMP engine id and
 *                        the USB serial string
 *   sts_atecc_sign()     ECDSA P-256 over a SHA-256 digest, for TLS / NTS-KE
 *   sts_atecc_pubkey()   the matching public key
 *   sts_atecc_random()   hardware TRNG output
 *   sts_atecc_counter_*  the two monotonic counters, for anti-rollback
 *   sts_atecc_attest()   the `sec.attest` report
 *
 * ---------------------------------------------------------------------------
 * Absence is normal, and it must never cost the boot anything
 * ---------------------------------------------------------------------------
 *
 * A board whose part is unpopulated, unprovisioned or dead still has to serve
 * time. So:
 *
 *   * The probe is lazy. Nothing here touches I²C until the first caller asks
 *     for something, and core/atecc latches "absent" after one failed wake, so
 *     an empty socket costs exactly one bounded handshake for the whole boot.
 *   * The absence is logged once, at warning level, and then never again.
 *   * Every entry point returns -ENODEV when the part is not there. Callers
 *     fall back to the software key path (port_crypto over PSA/mbedTLS), which
 *     is what keeps TLS and NTS working on a board with no secure element —
 *     see the port_crypto note at the bottom of this file.
 *   * `sec.atecc.en` = 0 disables the part outright without a rebuild.
 *
 * ---------------------------------------------------------------------------
 * The wake token and the bus speed
 * ---------------------------------------------------------------------------
 *
 * Waking the part needs SDA held low for longer than a byte takes at the
 * board's running 400 kHz. The standard trick is a write to the I²C general-call
 * address 0x00 with the bus temporarily reconfigured to 100 kHz; the NAK that
 * comes back is expected and ignored, because it is the electrical event that
 * matters, not the transfer. core/atecc asks for the speed change through the
 * transport's set_speed callback and restores 400 kHz immediately afterwards.
 *
 * I²C1 is shared with fifteen other devices (interface ref §4), so the whole
 * wake-command-read sequence runs under a mutex. That is also what makes the
 * shared context safe: core/atecc keeps its session state (awake/absent) in
 * one atecc_ctx_t, which two threads must not drive concurrently.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "atecc/atecc.h"
#include "cfg/cfg.h"
#include "storage/sts_atecc.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_atecc, CONFIG_STS1000_LOG_LEVEL);

/* Fallbacks so this file compiles even if its Kconfig fragment is dropped. */
#ifndef CONFIG_STS1000_ATECC_KEY_SLOT
#define CONFIG_STS1000_ATECC_KEY_SLOT 0
#endif

#define ATECC_I2C_ADDR ATECC_ADDR_DEFAULT

static const struct device *const i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));

static struct k_mutex lock;
static atecc_ctx_t ctx;
static bool inited;
static bool enabled = true;
static bool warned_absent;
static uint16_t key_slot = CONFIG_STS1000_ATECC_KEY_SLOT;

/* ------------------------------------------------------------------------- */
/* transport                                                                 */
/* ------------------------------------------------------------------------- */

static int bus_write(void *c, uint8_t addr, const uint8_t *buf, size_t len)
{
	ARG_UNUSED(c);

	if (!device_is_ready(i2c1)) {
		return -ENODEV;
	}
	return i2c_write(i2c1, buf, len, addr);
}

static int bus_read(void *c, uint8_t addr, uint8_t *buf, size_t len)
{
	ARG_UNUSED(c);

	if (!device_is_ready(i2c1)) {
		return -ENODEV;
	}
	return i2c_read(i2c1, buf, len, addr);
}

static int bus_delay(void *c, uint32_t us)
{
	ARG_UNUSED(c);

	/*
	 * Anything at or under a tick is busy-waited; longer waits sleep so the
	 * 700 ms SelfTest does not block a cooperative thread for that long.
	 * k_busy_wait() below the threshold avoids handing the CPU away for the
	 * 5 µs an Info command needs.
	 */
	if (us <= 1000U) {
		k_busy_wait(us);
	} else {
		k_sleep(K_USEC(us));
	}
	return 0;
}

static int bus_speed(void *c, uint32_t hz)
{
	uint32_t cfg;

	ARG_UNUSED(c);

	if (!device_is_ready(i2c1)) {
		return -ENODEV;
	}
	cfg = (hz <= 100000U) ? I2C_SPEED_SET(I2C_SPEED_STANDARD)
			      : I2C_SPEED_SET(I2C_SPEED_FAST);
	return i2c_configure(i2c1, cfg | I2C_MODE_CONTROLLER);
}

/* ------------------------------------------------------------------------- */
/* lifecycle                                                                 */
/* ------------------------------------------------------------------------- */

/** Bind the context on first use. Performs no I/O. */
static int ensure_init(void)
{
	atecc_bus_t bus;
	int rc;

	if (inited) {
		return enabled ? 0 : -ENODEV;
	}

	memset(&bus, 0, sizeof(bus));
	bus.write = bus_write;
	bus.read = bus_read;
	bus.delay_us = bus_delay;
	bus.set_speed = bus_speed;
	bus.ctx = NULL;

	rc = atecc_init(&ctx, &bus, ATECC_I2C_ADDR);
	if (rc != 0) {
		LOG_ERR("atecc_init: %d", rc);
		enabled = false;
		inited = true;
		return -ENODEV;
	}
	inited = true;
	return enabled ? 0 : -ENODEV;
}

/** Log an absent part exactly once, whatever asks for it. */
static void note_absent(void)
{
	if (warned_absent) {
		return;
	}
	warned_absent = true;
	LOG_WRN("ATECC608B at 0x%02x did not answer; "
		"falling back to software keys",
		ATECC_I2C_ADDR);
}

/**
 * Take the bus and make sure the part is awake.
 *
 * @retval 0        Locked and awake; the caller must call release().
 * @retval -ENODEV  Disabled or absent; the lock is already released.
 */
static int acquire(void)
{
	int rc;

	rc = ensure_init();
	if (rc != 0) {
		return rc;
	}

	k_mutex_lock(&lock, K_FOREVER);
	rc = atecc_wake(&ctx);
	if (rc != 0) {
		k_mutex_unlock(&lock);
		note_absent();
		return -ENODEV;
	}
	return 0;
}

static void release(void)
{
	/*
	 * Idle rather than Sleep: Idle keeps the RNG seed, and the part is woken
	 * again for the very next command. Either way TempKey is cleared, so no
	 * loaded digest survives between callers.
	 */
	(void)atecc_idle(&ctx);
	k_mutex_unlock(&lock);
}

void sts_atecc_configure(bool enable, uint8_t slot)
{
	k_mutex_lock(&lock, K_FOREVER);
	enabled = enable;
	key_slot = slot;
	if (enable) {
		/* Allow one more probe after an operator re-enables the part or
		 * reseats a board. */
		if (inited) {
			atecc_reprobe(&ctx);
		}
		warned_absent = false;
	}
	k_mutex_unlock(&lock);
}

void sts_atecc_reapply(void)
{
	cfg_ctx_t *c = sts_cfg();
	bool en = true;
	uint64_t slot = CONFIG_STS1000_ATECC_KEY_SLOT;

	if (c == NULL) {
		return;
	}
	(void)cfg_get_bool(c, (uint16_t)CFG_ID_SEC_ATECC_EN, &en);
	(void)cfg_get_u64(c, (uint16_t)CFG_ID_SEC_ATECC_SLOT, &slot);
	sts_atecc_configure(en, (uint8_t)slot);
}

bool sts_atecc_present(void)
{
	bool present;

	if (acquire() != 0) {
		return false;
	}
	present = atecc_present(&ctx);
	release();
	return present;
}

/* ------------------------------------------------------------------------- */
/* services                                                                  */
/* ------------------------------------------------------------------------- */

int sts_atecc_serial(uint8_t out[ATECC_SERIAL_LEN])
{
	int rc;

	if (out == NULL) {
		return -EINVAL;
	}
	rc = acquire();
	if (rc != 0) {
		return rc;
	}
	rc = atecc_serial(&ctx, out);
	release();
	return rc;
}

int sts_atecc_serial_string(char *out, size_t cap)
{
	uint8_t sn[ATECC_SERIAL_LEN];
	static const char hex[] = "0123456789ABCDEF";
	size_t i;
	int rc;

	if (out == NULL || cap < ((ATECC_SERIAL_LEN * 2U) + 1U)) {
		return -EINVAL;
	}
	rc = sts_atecc_serial(sn);
	if (rc != 0) {
		return rc;
	}
	for (i = 0U; i < ATECC_SERIAL_LEN; i++) {
		out[i * 2U] = hex[(sn[i] >> 4) & 0x0FU];
		out[(i * 2U) + 1U] = hex[sn[i] & 0x0FU];
	}
	out[ATECC_SERIAL_LEN * 2U] = '\0';
	return (int)(ATECC_SERIAL_LEN * 2U);
}

int sts_atecc_random(uint8_t *out, size_t len)
{
	uint8_t blk[32];
	size_t off = 0U;
	int rc;

	if (out == NULL || len == 0U) {
		return -EINVAL;
	}
	rc = acquire();
	if (rc != 0) {
		return rc;
	}
	while (off < len) {
		size_t n = len - off;

		rc = atecc_random(&ctx, blk);
		if (rc != 0) {
			break;
		}
		if (n > sizeof(blk)) {
			n = sizeof(blk);
		}
		memcpy(&out[off], blk, n);
		off += n;
	}
	release();
	/* Never leave a partial fill behind for a caller that ignores rc. */
	if (rc != 0) {
		memset(out, 0, len);
	}
	return rc;
}

int sts_atecc_sign(const uint8_t digest[32], uint8_t sig[64])
{
	int rc;

	if (digest == NULL || sig == NULL) {
		return -EINVAL;
	}
	rc = acquire();
	if (rc != 0) {
		return rc;
	}
	rc = atecc_sign(&ctx, key_slot, digest, sig);
	release();
	return rc;
}

int sts_atecc_pubkey(uint8_t pub[64])
{
	int rc;

	if (pub == NULL) {
		return -EINVAL;
	}
	rc = acquire();
	if (rc != 0) {
		return rc;
	}
	rc = atecc_pubkey(&ctx, key_slot, pub);
	release();
	return rc;
}

int sts_atecc_verify(const uint8_t pub[64], const uint8_t digest[32],
		     const uint8_t sig[64])
{
	bool ok = false;
	int rc;

	if (pub == NULL || digest == NULL || sig == NULL) {
		return -EINVAL;
	}
	rc = acquire();
	if (rc != 0) {
		return rc;
	}
	rc = atecc_verify_extern(&ctx, pub, digest, sig, &ok);
	release();
	if (rc == 0 && !ok) {
		return -EBADE;
	}
	return rc;
}

int sts_atecc_ecdh(const uint8_t peer[64], uint8_t secret[32])
{
	int rc;

	if (peer == NULL || secret == NULL) {
		return -EINVAL;
	}
	rc = acquire();
	if (rc != 0) {
		return rc;
	}
	rc = atecc_ecdh(&ctx, key_slot, peer, secret);
	release();
	return rc;
}

int sts_atecc_counter_read(uint8_t idx, uint32_t *out)
{
	int rc;

	if (out == NULL) {
		return -EINVAL;
	}
	rc = acquire();
	if (rc != 0) {
		return rc;
	}
	rc = atecc_counter_read(&ctx, idx, out);
	release();
	return rc;
}

int sts_atecc_counter_increment(uint8_t idx, uint32_t *out)
{
	int rc;

	if (out == NULL) {
		return -EINVAL;
	}
	rc = acquire();
	if (rc != 0) {
		return rc;
	}
	/*
	 * The counters are one-way and finite (the datasheet's monotonic
	 * counters top out); an accidental increment is unrecoverable, so the
	 * only caller should be the anti-rollback path at a confirmed image
	 * upgrade — not anything that runs per boot.
	 */
	rc = atecc_counter_increment(&ctx, idx, out);
	release();
	return rc;
}

int sts_atecc_attest(atecc_attest_t *out)
{
	int rc;

	if (out == NULL) {
		return -EINVAL;
	}
	rc = ensure_init();
	if (rc != 0) {
		memset(out, 0, sizeof(*out));
		return -ENODEV;
	}

	k_mutex_lock(&lock, K_FOREVER);
	rc = atecc_attest(&ctx, key_slot, out);
	if (rc == -ENODEV) {
		note_absent();
	}
	(void)atecc_idle(&ctx);
	k_mutex_unlock(&lock);
	return rc;
}

int sts_atecc_stats(atecc_stats_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}
	if (!inited) {
		memset(out, 0, sizeof(*out));
		return -ENODEV;
	}
	k_mutex_lock(&lock, K_FOREVER);
	(void)atecc_stats_get(&ctx, out);
	k_mutex_unlock(&lock);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* device-unique value for the SNMP engine id                                */
/* ------------------------------------------------------------------------- */

int sts_atecc_device_unique(uint8_t *out, size_t cap, size_t *out_len,
			    bool *out_from_atecc)
{
	uint8_t sn[ATECC_SERIAL_LEN];

	if (out == NULL || out_len == NULL || cap == 0U) {
		return -EINVAL;
	}
	if (out_from_atecc != NULL) {
		*out_from_atecc = false;
	}

	if (cap >= ATECC_SERIAL_LEN && sts_atecc_serial(sn) == 0) {
		memcpy(out, sn, ATECC_SERIAL_LEN);
		*out_len = ATECC_SERIAL_LEN;
		if (out_from_atecc != NULL) {
			*out_from_atecc = true;
		}
		return 0;
	}

	/*
	 * No secure element: fall back to the SoC's 96-bit unique device id.
	 * It is per-die and immutable, which is all RFC 3411 §5 needs of the
	 * variable part of an engine id — it just is not tamper-resistant.
	 */
	{
		ssize_t n = hwinfo_get_device_id(out, cap);

		if (n > 0) {
			*out_len = (size_t)n;
			return 0;
		}
	}
	return -ENODEV;
}

/* ------------------------------------------------------------------------- */
/* init                                                                      */
/* ------------------------------------------------------------------------- */

static int sts_atecc_sys_init(void)
{
	k_mutex_init(&lock);
	/*
	 * No probe here. The part sits on always-on 3V3_STM so it is powered
	 * from reset, but I²C1 is shared and the bring-up sequencer owns the bus
	 * during stages 2-3; a probe at init would race it for no benefit. The
	 * first caller that wants a key pays for the handshake.
	 */
	return 0;
}

SYS_INIT(sts_atecc_sys_init, APPLICATION, 90);
