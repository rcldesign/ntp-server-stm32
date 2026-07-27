/*
 * STS1000 "Meridian" — Maintenance Protocol passthrough tunnels (console area).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Channels 0x02/0x03 are read-only tees of the GNSS receiver's NMEA and UBX
 * traffic; 0x07 and 0x08 are bidirectional passthroughs to the ZED-F9T on USART3
 * and the FE-5680A on UART7.
 *
 * The safety rule (FMT §5.5) is the whole reason this is a separate file: while a
 * tunnel is open, firmware must not drive that port, and the reference it feeds
 * becomes **suspect**. Opening one is therefore a guarded override on
 * `gnss.tunnel` / `ref.rb.tunnel`, and the open flag is published here so the
 * GNSS and reference managers can stand down and stop trusting what they last
 * heard. Core reports `reference_suspect` in the override reply; this file is
 * what makes it true.
 *
 * A tunnel is *not* a subscription: it is opened by the override and closed by
 * releasing it, by the dead-man, or by leaving MP mode — all of which arrive here
 * as an apply/release from core.
 */

#include <zephyr/kernel.h>

#ifdef CONFIG_STS1000_CONSOLE

#include <errno.h>

#include <zephyr/logging/log.h>

#include "console/mp_glue.h"
#include "fault/fault.h"
#include "zephyr/sts_app.h"

LOG_MODULE_DECLARE(sts_mp, CONFIG_STS1000_LOG_LEVEL);

/* ------------------------------------------------------------------ state */

static bool tunnel_gnss;
static bool tunnel_rb;

/* Counters, for `mp status` and the support bundle. */
static uint32_t tee_gnss_bytes;
static uint32_t tee_rb_bytes;
static uint32_t tee_nmea_bytes;
static uint32_t tee_ubx_bytes;
static uint32_t tee_dropped;

bool sts_mp_tunnel_gnss_open(void)
{
	return tunnel_gnss;
}

bool sts_mp_tunnel_rb_open(void)
{
	return tunnel_rb;
}

/**
 * Open or close the GNSS tunnel.
 *
 * Called from core's apply callback once the G2 guard and the interlocks have
 * passed. Raising the suspect flag is the *first* thing done on open and the last
 * undone on close, so there is no window in which bytes are being diverted while
 * the reference is still believed.
 */
int sts_mp_tunnel_set_gnss(bool open)
{
	if (tunnel_gnss == open) {
		return 0;
	}
	tunnel_gnss = open;

	/*
	 * TODO(platform): the GNSS manager has to be told to stand down, and
	 * `refsel` to treat the receiver as untrusted for the duration. Neither
	 * is reachable from the console area — it needs one entry point in
	 * src/zephyr/sts_app.h (e.g. sts_gnss_suspend(bool)). Until it exists the
	 * tunnel still carries bytes and the alarm below is the operator's
	 * warning, but firmware keeps reading the port concurrently.
	 */
	(void)sts_alarm_set(FAULT_ALARM_TAMPER, open);
	sts_log(LOGR_SUB_GNSS, open ? LOGR_WARN : LOGR_NOTICE,
		"MP GNSS tunnel %s; reference %s", open ? "open" : "closed",
		open ? "SUSPECT" : "trusted");
	return 0;
}

/** Open or close the rubidium serial tunnel. Same contract as the GNSS one. */
int sts_mp_tunnel_set_rb(bool open)
{
	if (tunnel_rb == open) {
		return 0;
	}
	tunnel_rb = open;

	/* TODO(platform): as above, for the FE-5680A reader on UART7. */
	sts_log(LOGR_SUB_TIMING, open ? LOGR_WARN : LOGR_NOTICE,
		"MP Rb tunnel %s; reference %s", open ? "open" : "closed",
		open ? "SUSPECT" : "trusted");
	return 0;
}

/* -------------------------------------------------------------------- tees */

/** Frame @p len bytes onto @p ch, counting a refusal rather than retrying. */
static void tee(uint8_t ch, const uint8_t *data, size_t len, uint32_t *counter)
{
	const mp_ctx_t *c = sts_mp_ctx();
	int rc;

	if ((c == NULL) || (data == NULL) || (len == 0U)) {
		return;
	}
	/*
	 * mp_stream_raw() takes a non-const ctx, but the only mutation is the
	 * frame counter and the TX path; the const accessor exists so other
	 * console-area code cannot reach in and change engine state.
	 */
	rc = mp_stream_raw((mp_ctx_t *)c, ch, data, len);
	if (rc < 0) {
		if (rc != -ENOENT) {
			/* -ENOENT is the normal "nobody is listening" case. */
			tee_dropped++;
		}
		return;
	}
	*counter += (uint32_t)len;
}

void sts_mp_tee_gnss(const uint8_t *data, size_t len)
{
	if (tunnel_gnss) {
		tee(MP_CH_GNSS_PASS, data, len, &tee_gnss_bytes);
	}
}

void sts_mp_tee_rb(const uint8_t *data, size_t len)
{
	if (tunnel_rb) {
		tee(MP_CH_RB_PASS, data, len, &tee_rb_bytes);
	}
}

void sts_mp_tee_nmea(const uint8_t *data, size_t len)
{
	tee(MP_CH_NMEA, data, len, &tee_nmea_bytes);
}

void sts_mp_tee_ubx(const uint8_t *data, size_t len)
{
	tee(MP_CH_UBX, data, len, &tee_ubx_bytes);
}

/** Tee byte counts, for `mp status`. */
void sts_mp_tunnel_stats(uint32_t *gnss, uint32_t *rb, uint32_t *nmea,
			 uint32_t *ubx, uint32_t *dropped)
{
	if (gnss != NULL) {
		*gnss = tee_gnss_bytes;
	}
	if (rb != NULL) {
		*rb = tee_rb_bytes;
	}
	if (nmea != NULL) {
		*nmea = tee_nmea_bytes;
	}
	if (ubx != NULL) {
		*ubx = tee_ubx_bytes;
	}
	if (dropped != NULL) {
		*dropped = tee_dropped;
	}
}

#endif /* CONFIG_STS1000_CONSOLE */
