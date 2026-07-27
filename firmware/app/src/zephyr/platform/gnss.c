/*
 * STS1000 "Meridian" — the GNSS thread (ARCHITECTURE.md §6, priority 6).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * The UBX link to the ZED-F9T-00B on USART3 (PD8 TX / PD9 RX, 460800 8N1) and the
 * host for core/gnssmgr. Three jobs, nothing else:
 *
 *   1. Move bytes. The receive ISR pushes into a ring; this thread drains it
 *      through core/ubx's streaming parser and hands each validated frame to
 *      gnssmgr_on_msg(). Transmission is gnssmgr's send_ubx callback, called only
 *      from this thread.
 *   2. Pump gnssmgr. gnssmgr_step() at the thread's own wake rate for ACK
 *      timeouts and configuration retries; gnssmgr_tick_1hz() once a second for
 *      the antenna supervisor and NAV-PVT staleness.
 *   3. Publish. One mutex-protected snapshot that the discipline loop reads for
 *      gnss_time_locked / qErr / leap, and that pwrseq reads for its stage-5
 *      configuration gate.
 *
 * ---------------------------------------------------------------------------
 * Why the discipline loop cannot work without this file
 * ---------------------------------------------------------------------------
 * disc_tick_pps() takes the `!gnss_time_locked` branch and processes *no* PPS
 * sample while that flag is false (core/disc §3.6: a pulse with no valid
 * timescale behind it is not a reference). With the flag hard-zeroed the loop sat
 * in ACQUIRING at stratum 16 with the DAC at its centre code forever, no matter
 * how good the pulses were — the timing engine, which is the entire product, was
 * inert. Everything here exists to make that flag, the sawtooth qErr and the leap
 * record real.
 *
 * ---------------------------------------------------------------------------
 * Pin ownership
 * ---------------------------------------------------------------------------
 * This file owns no GPIO. GPS_PWR_EN, GPS_RST_N and ANT_BIAS_EN belong to
 * pwrseq_exec.c (ARCHITECTURE.md §10 — one writer per pin), so the antenna
 * supervisor's decision to cut the bias is routed through
 * sts_pwrseq_ant_bias_request() rather than driven here, and the reset/power
 * actions call *into* this file (sts_gnss_notify_reset, sts_gnss_configure)
 * rather than the other way round.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "zephyr/platform/platform.h"
#include "zephyr/sts_app.h"

#include "cfg/cfg.h"
#include "gnssmgr/gnssmgr.h"
#include "ina228/ina228.h"
#include "ubx/ubx.h"
#include "util/ring.h"

LOG_MODULE_REGISTER(sts_gnss, CONFIG_STS1000_LOG_LEVEL);

#define GNSS_STACK_SIZE 3072
#define GNSS_PRIO       6   /* ARCHITECTURE.md §6 */
/*
 * Wake rate. gnssmgr.h asks for gnssmgr_step() at least every ~100 ms so an ACK
 * timeout is not reported late; the same rate keeps the receive ring drained well
 * inside its own depth at 460800 baud.
 */
#define GNSS_TICK_MS    50

#define GNSS_RX_RING_SZ CONFIG_STS1000_GNSS_UART_RX_RING

/* GNSS-group cfg keys (cfg_schema.h, group 0x05). */
#define CFG_KEY_GNSS_CONSTEL      0x0501U
#define CFG_KEY_GNSS_ELEV_MASK    0x0502U
#define CFG_KEY_GNSS_SURVEY_DUR_S 0x0503U
#define CFG_KEY_GNSS_SURVEY_ACC   0x0504U

K_THREAD_STACK_DEFINE(gnss_stack, GNSS_STACK_SIZE);
static struct k_thread gnss_tcb;

static const struct device *const gnss_uart = DEVICE_DT_GET(DT_NODELABEL(usart3));
/* GPS_ANT_OFF_MON (PD4): the LNA's own "I am switched off" indication, one of the
 * antenna supervisor's three evidence sources (bias-supervisor doc §9.3). */
static const struct gpio_dt_spec ant_off_mon = STS_USER_GPIO(gps_ant_off_gpios);

/* ---- module state -------------------------------------------------------- */

static gnssmgr_t mgr;
static ubx_parser_t parser;
static uint8_t parser_buf[UBX_PAYLOAD_CAP_DEFAULT];

static uint8_t rx_buf[GNSS_RX_RING_SZ];
static ring_t rx_ring;

static sts_gnss_snap_t snap;
static K_MUTEX_DEFINE(snap_mutex);

static struct {
	int liveness_id;
	bool started;
	bool ant_supervisor_on;
	atomic_t rx_overruns;
	uint32_t tx_errors;
	uint32_t last_1hz_ms;
	/* Latest NAV-PVT iTOW and the monotonic instant it was decoded, which is
	 * what lets the discipline glue name the ToW of a captured pulse. */
	uint32_t pvt_itow_ms;
	uint64_t pvt_rx_mono_ms;
	bool have_pvt;
	/* A receiver flash session owns USART3: the RX ISR still fills the ring so
	 * the loader transport can drain it, but nothing feeds the UBX parser or
	 * gnssmgr while this is set. See the USART3 seam at the end of this file. */
	bool fw_mode;
} gs;

/* ========================================================================= */
/* UART                                                                      */
/* ========================================================================= */

static void gnss_uart_isr(const struct device *dev, void *user)
{
	uint8_t buf[32];
	int n;

	ARG_UNUSED(user);

	if (!uart_irq_update(dev)) {
		return;
	}

	while (uart_irq_rx_ready(dev)) {
		n = uart_fifo_read(dev, buf, sizeof(buf));
		if (n <= 0) {
			break;
		}
		/*
		 * A short put means the thread has not drained in a whole ring's
		 * worth of traffic. Count it and drop the tail rather than
		 * blocking here: the parser resynchronises on the next B5 62, so
		 * one lost frame costs one second of that message type, while
		 * stalling this ISR would cost UART overruns on every stream.
		 */
		if (ring_put(&rx_ring, buf, (size_t)n) != (size_t)n) {
			atomic_inc(&gs.rx_overruns);
		}
	}
}

static int gnss_send_ubx(void *user, const uint8_t *frame, size_t len)
{
	ARG_UNUSED(user);

	if ((frame == NULL) || (len == 0U)) {
		return -EINVAL;
	}
	if (!device_is_ready(gnss_uart)) {
		return -ENODEV;
	}

	/*
	 * Polled output. Called only from this thread, and only for configuration
	 * frames (a few hundred bytes per boot), so the ~5 ms a 256-byte VALSET
	 * takes at 460800 baud is spent in a thread that has nothing else to do —
	 * an interrupt-driven TX path here would buy nothing and add a queue whose
	 * failure modes gnssmgr's retry logic already covers.
	 */
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(gnss_uart, frame[i]);
	}

	return 0;
}

/* ========================================================================= */
/* gnssmgr callbacks                                                         */
/* ========================================================================= */

static void gnss_set_ant_bias(void *user, bool on)
{
	ARG_UNUSED(user);

	/* pwrseq_exec.c is the single writer of ANT_BIAS_EN (PC9). */
	sts_pwrseq_ant_bias_request(on);

	sts_log(LOGR_SUB_GNSS, on ? LOGR_NOTICE : LOGR_CRIT,
		"antenna bias %s by the supervisor", on ? "restored" : "cut");
}

static void gnss_alarm(void *user, gnssmgr_alarm_t id, bool active)
{
	ARG_UNUSED(user);

	switch (id) {
	case GNSSMGR_ALARM_ANT_OPEN:
		(void)sts_alarm_set(FAULT_ALARM_ANTENNA_OPEN, active);
		break;
	case GNSSMGR_ALARM_ANT_SHORT:
		(void)sts_alarm_set(FAULT_ALARM_ANTENNA_SHORT, active);
		break;
	case GNSSMGR_ALARM_TIME_UNLOCKED:
		(void)sts_alarm_set(FAULT_ALARM_GNSS_LOST, active);
		break;
	case GNSSMGR_ALARM_CONFIG_FAILED:
		/*
		 * An unconfigured receiver is not a time reference: without the
		 * TIMEPULSE and message-rate configuration there is no PPS to
		 * discipline against, so this belongs in the same alarm the loss
		 * of a fix does (and therefore in the relay-disqualify set).
		 */
		(void)sts_alarm_set(FAULT_ALARM_GNSS_LOST, active);
		if (active) {
			sts_log(LOGR_SUB_GNSS, LOGR_CRIT,
				"GNSS configuration failed at step %s",
				gnssmgr_step_name(gnssmgr_failed_step(&mgr)));
		}
		break;
	case GNSSMGR_ALARM_CONFIG_DEGRADED:
		sts_log(LOGR_SUB_GNSS, LOGR_WARN,
			"GNSS configuration degraded at step %s (advisory)",
			gnssmgr_step_name(gnssmgr_failed_step(&mgr)));
		break;
	case GNSSMGR_ALARM_SURVEY_REJECTED:
		sts_log(LOGR_SUB_GNSS, LOGR_WARN,
			"survey-in completed outside its accuracy limit");
		break;
	default:
		break;
	}
}

static const gnssmgr_cb_t gnss_cb = {
	.send_ubx = gnss_send_ubx,
	.set_ant_bias = gnss_set_ant_bias,
	/*
	 * store_ecef is deliberately absent. Persisting the surveyed position
	 * needs a calibration key this build's cfg schema does not define
	 * (group 0x0C holds only the nine INA228 trims), and inventing one here
	 * would put a schema decision in a glue file. Consequence, stated rather
	 * than hidden: a cold boot re-runs the survey instead of going straight to
	 * fixed-position timing mode. Position error is a fixed delay error, so the
	 * cost is survey time, not accuracy.
	 */
	.alarm = gnss_alarm,
};

/* ========================================================================= */
/* publication                                                               */
/* ========================================================================= */

static void gnss_publish(uint32_t now_ms)
{
	gnssmgr_status_t st;
	gnssmgr_leap_t leap;
	gnssmgr_qerr_t qerr;
	gnssmgr_state_t state = gnssmgr_get_state(&mgr);
	sts_gnss_snap_t s;

	ARG_UNUSED(now_ms);

	memset(&s, 0, sizeof(s));

	if (gnssmgr_status(&mgr, &st) == 0) {
		s.have_status = st.valid;
		s.time_locked = st.time_locked;
		s.utc_valid = st.utc_valid;
		s.fix_type = st.fix_type;
		s.sv_used = st.num_sv;
		s.tacc_ns = st.tacc_ns;
	}

	{
		gnssmgr_sats_t sats;

		if ((gnssmgr_sats(&mgr, &sats) == 0) && sats.valid) {
			s.sv_visible = sats.tracked;
			if (sats.used > 0U) {
				s.sv_used = sats.used;
			}
		}
	}

	if ((gnssmgr_leap(&mgr, &leap) == 0) && leap.valid && leap.curr_ls_valid) {
		s.leap_valid = true;
		/*
		 * gnssmgr_leap_t::current_ls is the receiver's GPS-UTC offset (18 s
		 * today). quality_block_t::leap_current_s and every consumer of it
		 * — the PTP-clock reference computation, the UI's TAI->UTC display
		 * conversion, SNMP/web/shell/MCP telemetry and the PFI fast-save —
		 * are all specified as TAI-UTC. TAI-GPS has been fixed at 19 s since
		 * the GPS epoch, so the conversion is GPS-UTC + 19; publishing
		 * current_ls raw understated the offset by 19 s everywhere.
		 */
		s.leap_current_s = (int16_t)leap.current_ls + STS_TAI_MINUS_GPS_S;
		s.leap_pending = leap.event_pending ? leap.ls_change : 0;
	}

	if (gnssmgr_qerr_for_pps(&mgr, &qerr) == 0) {
		s.qerr = qerr;
	}

	/*
	 * The configuration gate pwrseq's stage 5.5 waits on. SURVEY_IN and FIXED
	 * both mean the essential VALSETs were ACKed — a survey that has not
	 * finished is not a configuration failure, and blocking bring-up for an
	 * hour of surveying would leave the watchdog unarmed the whole time.
	 */
	s.cfg_ack = (state == GNSSMGR_ST_SURVEY_IN) || (state == GNSSMGR_ST_FIXED);
	s.cfg_failed = (state == GNSSMGR_ST_CONFIG_FAILED);
	s.ant_state = (uint8_t)gnssmgr_ant_get_state(&mgr);

	s.pvt_itow_ms = gs.pvt_itow_ms;
	s.pvt_rx_mono_ms = gs.pvt_rx_mono_ms;
	if (!gs.have_pvt) {
		s.have_status = false;
	}

	(void)k_mutex_lock(&snap_mutex, K_FOREVER);
	snap = s;
	(void)k_mutex_unlock(&snap_mutex);
}

int sts_gnss_snapshot(sts_gnss_snap_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}
	if (!gs.started) {
		memset(out, 0, sizeof(*out));
		return -ENODEV;
	}

	(void)k_mutex_lock(&snap_mutex, K_FOREVER);
	*out = snap;
	(void)k_mutex_unlock(&snap_mutex);

	return 0;
}

bool sts_gnss_cfg_ack(void)
{
	bool ack;

	if (!gs.started) {
		/*
		 * No GNSS thread means nothing will ever ACK. Reporting "yes" would
		 * let stage 5 walk past its own gate on a board whose receiver is
		 * unreachable; reporting "no" makes the step time out and raise
		 * PWRSEQ_ALARM_GNSS_CFG, which is the truth.
		 */
		return false;
	}

	(void)k_mutex_lock(&snap_mutex, K_FOREVER);
	ack = snap.cfg_ack;
	(void)k_mutex_unlock(&snap_mutex);

	return ack;
}

/* ========================================================================= */
/* receive path                                                              */
/* ========================================================================= */

static void gnss_drain_rx(uint32_t now_ms)
{
	uint8_t b;

	/* A flash session owns the ring; sts_gnss_uart_raw_rx() drains it. Feeding
	 * loader bytes to the UBX parser would at best desync the loader and at
	 * worst hand gnssmgr a frame from a receiver that is not running firmware. */
	if (gs.fw_mode) {
		return;
	}

	while (ring_getc(&rx_ring, &b) == 0) {
		ubx_msg_t m;

		if (ubx_parse_byte(&parser, b) != 1) {
			continue;
		}
		if (ubx_parser_msg(&parser, &m) != 0) {
			continue;
		}

		/*
		 * Note NAV-PVT arrival before gnssmgr consumes it: pairing a
		 * captured pulse with its qErr needs the iTOW *and* the instant it
		 * landed, and gnssmgr's status only keeps the former.
		 */
		if (ubx_msg_is(&m, UBX_CLASS_NAV, UBX_ID_NAV_PVT)) {
			ubx_nav_pvt_t pvt;

			if (ubx_parse_nav_pvt(&m, &pvt) == 0) {
				gs.pvt_itow_ms = pvt.itow_ms;
				gs.pvt_rx_mono_ms = sts_mono_ms();
				gs.have_pvt = true;
			}
		}

		(void)gnssmgr_on_msg(&mgr, &m, now_ms);
	}
}

/* ========================================================================= */
/* the thread                                                                */
/* ========================================================================= */

static void gnss_ant_sample(uint32_t now_ms)
{
	gnssmgr_ant_input_t ant;
	sts_hk_snapshot_t hk;
	int32_t ua;

	if (!gs.ant_supervisor_on) {
		/* Staleness only: without the antenna evidence the supervisor must
		 * not debounce toward a verdict it has no basis for. */
		(void)gnssmgr_tick_1hz(&mgr, NULL, now_ms);
		return;
	}

	memset(&ant, 0, sizeof(ant));

	if (gpio_is_ready_dt(&ant_off_mon)) {
		ant.ant_off_mon = gpio_pin_get_dt(&ant_off_mon) == 1;
	}

	(void)sts_hk_read(&hk);
	ua = hk.ina[INA228_RAIL_V_ANT].current_ua;
	ant.current_valid = hk.ina[INA228_RAIL_V_ANT].valid &&
			    hk.ina[INA228_RAIL_V_ANT].cal_ok && (ua >= 0);
	ant.current_ua = ant.current_valid ? (uint32_t)ua : 0U;
	/* The bias rail is on exactly when pwrseq says it is; the supervisor uses
	 * this to tell "commanded off" from "open circuit". */
	ant.bias_en = ant.current_valid && (ua > 0);

	(void)gnssmgr_tick_1hz(&mgr, &ant, now_ms);
}

static void gnss_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	gs.last_1hz_ms = k_uptime_get_32();

	for (;;) {
		uint32_t now_ms;

		k_msleep(GNSS_TICK_MS);
		now_ms = k_uptime_get_32();

		gnss_drain_rx(now_ms);
		(void)gnssmgr_step(&mgr, now_ms);

		if ((now_ms - gs.last_1hz_ms) >= 1000U) {
			gs.last_1hz_ms = now_ms;
			gnss_ant_sample(now_ms);
		}

		gnss_publish(now_ms);
		sts_liveness_feed(gs.liveness_id);
	}
}

/* ========================================================================= */
/* API                                                                       */
/* ========================================================================= */

static void gnss_load_cfg(gnssmgr_cfg_t *cfg)
{
	uint64_t v;

	(void)gnssmgr_cfg_default(cfg);

	if (cfg_get_u64(sts_cfg(), CFG_KEY_GNSS_ELEV_MASK, &v) == 0) {
		cfg->min_elev_deg = (int8_t)v;
	}
	if (cfg_get_u64(sts_cfg(), CFG_KEY_GNSS_SURVEY_DUR_S, &v) == 0) {
		cfg->survey_min_dur_s = (uint32_t)v;
	}
	if (cfg_get_u64(sts_cfg(), CFG_KEY_GNSS_SURVEY_ACC, &v) == 0) {
		/* Schema is millimetres; gnssmgr wants 0.1 mm units. */
		cfg->survey_acc_limit_0p1mm = (uint32_t)(v * 10U);
	}
}

int sts_gnss_start(void)
{
	gnssmgr_cfg_t cfg;
	k_tid_t tid;
	int rc;

	if (gs.started) {
		return -EALREADY;
	}

	if (!device_is_ready(gnss_uart)) {
		LOG_ERR("USART3 (GNSS) not ready");
		return -ENODEV;
	}

	rc = ring_init(&rx_ring, rx_buf, sizeof(rx_buf));
	if (rc != 0) {
		return rc;
	}

	rc = ubx_parser_init(&parser, parser_buf, sizeof(parser_buf));
	if (rc != 0) {
		return rc;
	}

	if (gpio_is_ready_dt(&ant_off_mon)) {
		(void)gpio_pin_configure_dt(&ant_off_mon, GPIO_INPUT);
	}

	gnss_load_cfg(&cfg);

	rc = gnssmgr_init(&mgr, &cfg, &gnss_cb);
	if (rc != 0) {
		LOG_ERR("gnssmgr_init failed (%d)", rc);
		return rc;
	}

	uart_irq_rx_disable(gnss_uart);
	uart_irq_tx_disable(gnss_uart);
	rc = uart_irq_callback_user_data_set(gnss_uart, gnss_uart_isr, NULL);
	if (rc != 0) {
		LOG_ERR("USART3 IRQ callback failed (%d)", rc);
		return rc;
	}
	uart_irq_rx_enable(gnss_uart);

	gs.liveness_id = sts_liveness_register("gnss");

	tid = k_thread_create(&gnss_tcb, gnss_stack, GNSS_STACK_SIZE, gnss_entry,
			      NULL, NULL, NULL, GNSS_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(tid, "gnss");

	gs.started = true;

	LOG_INF("gnss up on USART3: %u ms tick, %u B rx ring, elev %d deg",
		GNSS_TICK_MS, (unsigned int)GNSS_RX_RING_SZ, cfg.min_elev_deg);

	return 0;
}

int sts_gnss_configure(uint32_t now_ms)
{
	if (!gs.started) {
		return -ENODEV;
	}

	return gnssmgr_start(&mgr, now_ms);
}

int sts_gnss_notify_reset(uint32_t now_ms)
{
	if (!gs.started) {
		return -ENODEV;
	}

	/* Everything the receiver told us describes a session that no longer
	 * exists; the parser may also be mid-frame across the reset. */
	ubx_parser_reset(&parser);
	ring_reset(&rx_ring);
	gs.have_pvt = false;

	return gnssmgr_notify_reset(&mgr, now_ms);
}

void sts_gnss_ant_supervisor_start(void)
{
	gs.ant_supervisor_on = true;
}

/* ========================================================================= */
/* absolute epoch — the only thing that makes a served timestamp traceable    */
/* ========================================================================= */

/*
 * Days from 1970-01-01 to a proleptic-Gregorian civil date, after Howard
 * Hinnant's days_from_civil. Exact for every date the receiver can report and
 * branch-free apart from the leap-cycle shift; no libc time functions, which
 * would drag in a locale-aware mktime and a 64-bit division we do not want in
 * this path.
 */
static int64_t days_from_civil(int32_t y, uint32_t m, uint32_t d)
{
	int64_t era;
	uint32_t yoe, doy, doe;

	y -= (m <= 2U) ? 1 : 0;
	era = ((y >= 0) ? y : (y - 399)) / 400;
	yoe = (uint32_t)(y - (int32_t)(era * 400));            /* 0..399 */
	doy = (153U * ((m > 2U) ? (m - 3U) : (m + 9U)) + 2U) / 5U + d - 1U;
	doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;         /* 0..146096 */

	return era * 146097 + (int64_t)doe - 719468;
}

int sts_gnss_wallclock(sts_gnss_wallclock_t *out)
{
	gnssmgr_status_t st;
	gnssmgr_leap_t leap;
	uint64_t decoded_ms;

	if (out == NULL) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));

	if (!gs.started) {
		return -ENODATA;
	}

	if ((gnssmgr_status(&mgr, &st) != 0) || !st.valid) {
		return -ENODATA;
	}
	if (gnssmgr_leap(&mgr, &leap) != 0) {
		return -ENODATA;
	}

	/*
	 * The age reference is when this NAV-PVT was DECODED, not now — the
	 * consumer subtracts (now - mono_ms) before it compares, so handing it a
	 * fresh timestamp would make a stale reading look current and let a
	 * receiver that stopped talking place the epoch anyway.
	 */
	(void)k_mutex_lock(&snap_mutex, K_FOREVER);
	decoded_ms = snap.pvt_rx_mono_ms;
	(void)k_mutex_unlock(&snap_mutex);

	out->utc_valid = st.utc_valid;
	out->time_locked = st.time_locked;
	out->leap_valid = leap.valid && leap.curr_ls_valid;
	out->tai_minus_utc = out->leap_valid
			     ? ((int16_t)leap.current_ls + STS_TAI_MINUS_GPS_S)
			     : 0;
	out->tacc_ns = st.tacc_ns;
	out->mono_ms = decoded_ms;
	out->utc_nano_ns = st.nano_ns;
	out->utc_unix_s = days_from_civil((int32_t)st.year, st.month, st.day) *
			  86400 +
			  (int64_t)st.hour * 3600 + (int64_t)st.min * 60 +
			  (int64_t)st.sec;

	/*
	 * Deliberately NOT refused here on !utc_valid / !leap_valid / !locked:
	 * the flags are reported and sts_ptpclk's reference table enforces them
	 * (along with its own >= 2020 and staleness checks). One enforcement
	 * point, and the caller can log precisely which condition failed.
	 */
	return 0;
}

/* ========================================================================= */
/* USART3 seam for the GNSS firmware-update transport (core/fwupd)            */
/* ========================================================================= */
/*
 * Strong definitions overriding the __weak stubs in console/fwupd_glue.c.
 * Suspend/resume bracket a receiver flash session: while suspended the RX ISR
 * still fills the ring (so raw_rx can drain it) but nothing feeds the UBX
 * parser or gnssmgr, and gnssmgr sits in GNSSMGR_ST_FW_UPDATE so a config
 * retry can never be aimed into a flash loader.
 */

int sts_gnss_uart_suspend(void)
{
	if (!gs.started) {
		return -ENODEV;
	}
	if (gs.fw_mode) {
		return 0;
	}

	gs.fw_mode = true;
	ubx_parser_reset(&parser);
	ring_reset(&rx_ring);
	gs.have_pvt = false;

	return gnssmgr_fw_enter(&mgr);
}

int sts_gnss_uart_resume(void)
{
	if (!gs.started) {
		return -ENODEV;
	}
	if (!gs.fw_mode) {
		return 0;
	}

	/* Anything in flight belongs to the loader session, not to UBX. */
	ubx_parser_reset(&parser);
	ring_reset(&rx_ring);
	gs.have_pvt = false;
	gs.fw_mode = false;

	return gnssmgr_fw_exit(&mgr, (uint32_t)sts_mono_ms());
}

int sts_gnss_uart_raw_tx(const uint8_t *buf, size_t len)
{
	if ((buf == NULL) && (len != 0U)) {
		return -EINVAL;
	}
	if (!gs.started) {
		return -ENODEV;
	}
	if (!gs.fw_mode) {
		return -EPERM;   /* raw access only inside a suspended session */
	}

	for (size_t i = 0U; i < len; i++) {
		uart_poll_out(gnss_uart, buf[i]);
	}

	return (int)len;
}

int sts_gnss_uart_raw_rx(uint8_t *buf, size_t cap)
{
	size_t n = 0U;
	uint8_t b;

	if (!gs.started) {
		return -ENODEV;
	}
	/* cap == 0 with a NULL buffer is the capability probe fwupd_glue uses. */
	if ((buf == NULL) && (cap != 0U)) {
		return -EINVAL;
	}
	if (buf == NULL) {
		return 0;
	}
	if (!gs.fw_mode) {
		return -EPERM;
	}

	while ((n < cap) && (ring_getc(&rx_ring, &b) == 0)) {
		buf[n] = b;
		n++;
	}

	return (int)n;
}

int sts_gnss_uart_set_baud(uint32_t baud)
{
	struct uart_config cfg;
	int rc;

	if (!gs.started) {
		return -ENODEV;
	}
	if (!gs.fw_mode) {
		return -EPERM;
	}

	rc = uart_config_get(gnss_uart, &cfg);
	if (rc != 0) {
		return rc;
	}
	if (cfg.baudrate == baud) {
		return 0;
	}
	cfg.baudrate = baud;

	/* Re-configuring drops anything mid-shift; the loader protocol
	 * resynchronises after a rate change, but a stale byte would desync it. */
	rc = uart_configure(gnss_uart, &cfg);
	if (rc == 0) {
		ring_reset(&rx_ring);
	}

	return rc;
}
