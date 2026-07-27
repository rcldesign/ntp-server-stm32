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
 *      from this thread. The same drain feeds the maintenance tool's raw views
 *      when one is subscribed — see "maintenance-tool byte tees" below; it is a
 *      copy into a ring somebody else drains, never a call that can block this
 *      thread.
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

/*
 * console/mp_glue.h is a cross-area include, and a deliberate one. The
 * maintenance tool's NMEA/UBX views and its USART3 passthrough are tees of this
 * file's byte stream, and there is no second place the bytes exist. mp_glue.h
 * states the contract on its side; what matters here is that every function it
 * offers this file is a bounded copy or a single atomic read — nothing on the
 * receive path takes the MP engine mutex or touches the console UART — and that
 * src/zephyr/platform/sts_area_weak.c carries a __weak no-op for each of them,
 * so CONFIG_STS1000_MP=n and CONFIG_STS1000_CONSOLE=n both still link.
 */
#include "console/mp_glue.h"
#include "zephyr/platform/gnss_tee.h"
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
/*
 * Published beside `snap` and under the same mutex, but a separate structure:
 * the discipline thread reads the snapshot on the PPS edge and needs one record
 * of each kind, while sts_pps_epoch_get() runs on whatever thread asks and needs
 * an epoch of history to name a pulse at all. sts_app.h states the case.
 */
static sts_gnss_pulse_evidence_t evidence;
static K_MUTEX_DEFINE(snap_mutex);

/*
 * The sky view: the per-satellite records core/gnssmgr deliberately discards.
 *
 * gnssmgr.h is explicit that it reduces NAV-SAT to a counted summary and that
 * "the skyplot iterates the frame itself". This is where that iteration lands.
 * Its own mutex rather than snap_mutex: the discipline thread takes snap_mutex
 * on the PPS edge, and a 10 Hz render tick from priority 15 has no business
 * being anywhere near that lock.
 */
static sts_gnss_sky_t sky;
static K_MUTEX_DEFINE(sky_mutex);

/*
 * Operator-request mailbox (sts_app.h "GNSS operator requests").
 *
 * A spinlock, not a mutex: the producers are management threads at priority 12
 * and the consumer is this thread at priority 6, so a lock that can be held
 * across a preemption would let the web plane park the GNSS link. The critical
 * section is four stores on both sides.
 *
 * One slot. Survey-in and fixed-position are mutually exclusive settings of the
 * same TMODE step, so a second request arriving before the first is applied is
 * refused rather than queued.
 */
enum gnss_req_kind {
	GNSS_REQ_NONE = 0,
	GNSS_REQ_SURVEY,
	GNSS_REQ_FIXED,
};

static struct k_spinlock req_lock;
static struct {
	uint8_t kind; /* enum gnss_req_kind */
	gnssmgr_ecef_t pos;
} gnss_req;

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
	/*
	 * The observation the pair above replaced. The discipline thread never
	 * needs it — it asks on the PPS edge, when the latest NAV-PVT still
	 * describes the previous epoch — but the PTP servo asks on an unrelated
	 * timer, and for most of every second the latest observation is newer
	 * than the capture it would have to name. See the sts_app.h
	 * sts_gnss_pulse_evidence_t comment.
	 */
	uint32_t pvt_prev_itow_ms;
	uint64_t pvt_prev_rx_mono_ms;
	bool have_pvt_prev;
	/*
	 * The configured survey accuracy limit, in gnssmgr's 0.1 mm units, kept
	 * here so an operator-supplied fixed position can declare an accuracy
	 * without reading cfg from either the requesting thread (which would put
	 * the cfg mutex on the web path) or this one (which would put it on a
	 * priority-6 timing thread). It is the same value gnssmgr was handed at
	 * start, so the two cannot disagree.
	 */
	uint32_t survey_acc_0p1mm;
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

	/*
	 * The UBX view (channel 0x03) carries BOTH directions, after the fact.
	 * A capture that shows only what the receiver said cannot answer the
	 * question the channel exists for — "which VALSET did configuration fail
	 * on" — and every frame this function sends is a well-formed UBX message,
	 * so a decoder reading the capture handles it exactly as u-center handles
	 * a two-way log. The passthrough channel (0x07) deliberately does not get
	 * this: there, host->device bytes came FROM the host, and echoing them
	 * back up the same channel would corrupt its view of the receiver.
	 *
	 * Cost is nil in the steady state: this runs at boot and on configuration
	 * retries, not per epoch, and the tee returns after one atomic read when
	 * nobody is subscribed.
	 */
	sts_mp_tee_ubx(frame, len);

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
	 * store_ecef is deliberately absent, and so — for the same reason — is
	 * any persistence behind sts_gnss_set_fixed_ecef(). The site position is
	 * a stored calibration constant (gnssmgr.h) and this build's cfg schema
	 * defines no key for one: group 0x05 stops at gnss.ant.bias (0x0507) and
	 * group 0x0C holds only the INA trims, the PPS offsets, the tempco and the
	 * DAC centre. Adding four rows to cfg_schema.h is the fix; inventing a
	 * private NVS record here would be a second persistence path for a
	 * constant core/gnssmgr already knows how to hand over.
	 *
	 * Consequence, stated rather than hidden: a cold boot re-runs the survey
	 * instead of going straight to fixed-position timing mode, whether the
	 * position came from a survey or from an operator. Position error is a
	 * fixed delay error, so the cost is survey time, not accuracy.
	 */
	.alarm = gnss_alarm,
};

/* ========================================================================= */
/* publication                                                               */
/* ========================================================================= */

/** Copy one gnssmgr TIM-TP record into the naming-evidence form. */
static void gnss_fill_qerr_obs(sts_gnss_qerr_obs_t *o, const gnssmgr_qerr_t *q)
{
	o->target_tow_ms = q->target_tow_ms;
	o->rx_mono_ms = q->rx_mono_ms;
	o->week = q->week;
	o->qerr_ps = q->qerr_ps;
	o->qerr_valid = q->qerr_valid;
	o->valid = true;
}

/*
 * Assemble the pulse-naming evidence: the two most recent NAV-PVT observations
 * and the two most recent UBX-TIM-TP records, newest first.
 *
 * Why two of each rather than one is the whole point of the structure and is
 * argued at sts_app.h's sts_gnss_pulse_evidence_t; in short, for most of every
 * second the newest record of either kind is newer than the capture it would
 * have to describe, and a consumer that is not phase-locked to the PPS lands
 * there most of the time.
 *
 * @p pvt_usable carries the same gate the discipline snapshot applies — a
 * NAV-PVT iTOW from a receiver with no valid status is not evidence of
 * anything — so the two views cannot disagree about whether the receiver is
 * worth believing.
 */
static void gnss_build_evidence(sts_gnss_pulse_evidence_t *e, bool pvt_usable)
{
	gnssmgr_qerr_t q;

	memset(e, 0, sizeof(*e));

	if (pvt_usable && gs.have_pvt) {
		e->pvt[0].itow_ms = gs.pvt_itow_ms;
		e->pvt[0].rx_mono_ms = gs.pvt_rx_mono_ms;
		e->pvt[0].valid = true;

		if (gs.have_pvt_prev) {
			e->pvt[1].itow_ms = gs.pvt_prev_itow_ms;
			e->pvt[1].rx_mono_ms = gs.pvt_prev_rx_mono_ms;
			e->pvt[1].valid = true;
		}
	}

	if (gnssmgr_qerr_for_pps(&mgr, &q) == 0) {
		gnss_fill_qerr_obs(&e->qerr[0], &q);
	}
	if (gnssmgr_qerr_prev_for_pps(&mgr, &q) == 0) {
		gnss_fill_qerr_obs(&e->qerr[1], &q);
	}
}

static void gnss_publish(uint32_t now_ms)
{
	gnssmgr_status_t st;
	gnssmgr_leap_t leap;
	gnssmgr_qerr_t qerr;
	gnssmgr_state_t state = gnssmgr_get_state(&mgr);
	sts_gnss_snap_t s;
	sts_gnss_pulse_evidence_t e;

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

	gnss_build_evidence(&e, s.have_status);

	(void)k_mutex_lock(&snap_mutex, K_FOREVER);
	snap = s;
	evidence = e;
	(void)k_mutex_unlock(&snap_mutex);
}

int sts_gnss_pulse_evidence(sts_gnss_pulse_evidence_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}
	if (!gs.started) {
		memset(out, 0, sizeof(*out));
		return -ENODEV;
	}

	(void)k_mutex_lock(&snap_mutex, K_FOREVER);
	*out = evidence;
	(void)k_mutex_unlock(&snap_mutex);

	return 0;
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

/* ---- maintenance-tool byte tees ----------------------------------------- */
/*
 * Splitting the raw USART3 stream onto the FMT's two read-only views (§7.3/7.4,
 * "raw tees, capturable to .nmea / .ubx") without paying for a second parser.
 *
 * The UBX parser this file already runs over every byte publishes exactly the
 * one fact the split needs: `parser.state == UBX_PS_SYNC1` means it is hunting
 * for B5 62, i.e. the byte about to be consumed is NOT inside a frame. A '$'
 * there starts an NMEA sentence and everything up to and including the LF is
 * NMEA; everything else is UBX. Sampling the state BEFORE ubx_parse_byte()
 * consumes the byte is what makes that true, so the tee call sits above it.
 *
 * The rule is honest about its one blind spot: a '$' that arrives while the
 * parser is one byte into a false sync (state SYNC2, after a stray 0xB5) is
 * classified UBX. That is a byte of line noise landing in the raw UBX capture,
 * which is where line noise belongs.
 *
 * The as-built receiver is configured UBX-only (spec §3.7), so channel 0x02
 * normally stays silent — but a receiver that has just been reset to factory
 * defaults, which is precisely when a technician opens the NMEA view, talks
 * NMEA until the configuration walk lands.
 *
 * ---------------------------------------------------------------------------
 * What this costs the receive path, counted from the generated code
 * ---------------------------------------------------------------------------
 * This loop feeds gnssmgr, which feeds the discipline loop, so the number is
 * stated rather than reassured about. Counts are from `objdump -d` of the
 * as-built image at 250 MHz, taken branches charged 3 cycles.
 *
 *   NOT armed (the normal state — no technician attached)
 *       one `cbz` per byte, taken.               <= 3 cycles  ~12 ns/byte
 *       plus one hoisted sts_mp_gnss_tee_armed() (8 instructions) per 50 ms
 *       drain pass. At the as-built ~1 kB/s UBX rate: ~12 us per second.
 *
 *   ARMED, per byte in this loop
 *       23 instructions, 6 of them taken branches. ~35 cycles ~140 ns/byte
 *
 *   ARMED, per 64-byte run handed to sts_mp_tee_*()
 *       ring_free + 2x ring_putc + ring_put's 64-byte memcpy, inside
 *       k_spin_lock.                            ~191 cycles  ~0.8 us
 *       Amortised that is ~3 cycles (~12 ns) per byte.
 *
 *   ARMED, all in                                ~38 cycles  <=160 ns/byte
 *       ~1 kB/s as-built UBX set:  ~164 us/s, ~8 us per 50 ms pass
 *       USART3 saturated (460800 8N1, 46 080 B/s):
 *                                  ~7.4 ms/s (0.74 % of one core),
 *                                  ~369 us per 50 ms pass — against ~1.4 ms
 *                                  this loop already spends in ubx_parse_byte()
 *                                  for the same bytes.
 *
 * The figure that actually bounds the timing path is not throughput but the
 * longest window with interrupts masked, because k_spin_lock() raises BASEPRI:
 * **<= ~0.8 us, once per 64 bytes staged**. PA0/TIM2 is a hardware input
 * capture, so a PPS edge inside that window is still timestamped by the timer
 * at the edge — the delay shifts when the capture register is read, not what it
 * recorded. Nothing on this path takes a mutex, allocates, or waits.
 *
 * The classifier itself is platform/gnss_tee.h, Zephyr-free so tests/host can
 * drive it (suite `gnss_tee`) — the arrangement net/sts_ppscorr.h uses, for the
 * same reason: a misclassified run is invisible until someone cannot parse a
 * capture. This file supplies only the sink and the parser-state evidence.
 */
static gnss_tee_t gtee;

/** Route a classified run onto its MP channel. */
static void gnss_tee_sink(void *user, bool nmea, const uint8_t *data, size_t len)
{
	ARG_UNUSED(user);

	if (nmea) {
		sts_mp_tee_nmea(data, len);
	} else {
		sts_mp_tee_ubx(data, len);
	}
}

/**
 * Drain the receive ring straight onto the passthrough channel.
 *
 * The counterpart of gnss_drain_rx() for a suspended port. Only the MP tunnel
 * gets this: a receiver *firmware* session also sets gs.fw_mode, and its bytes
 * belong to sts_gnss_uart_raw_rx(), so draining them here would starve the
 * loader transport. Nothing classifies or parses — the host asked for the port,
 * not for an interpretation of it.
 *
 * Bounded by the ring's own capacity rather than by "until empty", so a receiver
 * babbling at line rate cannot hold this thread past its liveness feed.
 */
static void gnss_drain_tunnel(void)
{
	uint8_t buf[GNSS_TEE_STAGE];
	unsigned int pass;

	for (pass = 0U; pass <= (GNSS_RX_RING_SZ / GNSS_TEE_STAGE); pass++) {
		size_t n = ring_get(&rx_ring, buf, sizeof(buf));

		if (n == 0U) {
			return;
		}
		sts_mp_tee_gnss(buf, n);
	}
}

/* ========================================================================= */
/* sky view (UBX-NAV-SAT -> the §6.3 skyplot)                                */
/* ========================================================================= */

/**
 * Cache the receiver's geodetic position for the skyplot's declination model.
 *
 * Only from a fix the receiver itself vouches for: `gnss_fix_ok` is the
 * DOP/accuracy mask having passed, and `invalid_llh` marks a longitude/latitude
 * the receiver knows is meaningless. A declination computed from a bad fix
 * rotates the plot confidently and wrongly, which is the outcome §6.3 exists to
 * prevent — so a rejected fix leaves `pos_valid` as it was rather than clearing
 * it. A fixed-site grandmaster does not move, so the last good position stays
 * true even while the receiver is struggling.
 */
static void gnss_note_position(const ubx_nav_pvt_t *pvt)
{
	if (!pvt->gnss_fix_ok || pvt->invalid_llh) {
		return;
	}

	k_mutex_lock(&sky_mutex, K_FOREVER);
	sky.lat_1e7 = pvt->lat_1e7;
	sky.lon_1e7 = pvt->lon_1e7;
	sky.pos_valid = true;
	k_mutex_unlock(&sky_mutex);
}

/**
 * Cache one UBX-NAV-SAT frame's per-satellite records.
 *
 * Every record is kept, unfiltered: deciding which satellites are worth
 * plotting is display policy and lives in the UI area
 * (ui/sts_sky_policy.h sts_sky_sv_from_ubx()), which is where it can be
 * host-tested. This function's only job is to get the frame across the area
 * seam without tearing.
 *
 * A frame carrying more than STS_GNSS_SKY_MAX_SV records is TRUNCATED, not
 * dropped: the F9T can report well over 32 with four constellations enabled,
 * and 32 markers already saturates a 160-pixel plot. The truncation is by the
 * receiver's own ordering, which is not signal strength — but the alternative
 * is a sort on the priority-6 GNSS thread, and the UI already ranks by C/N0 for
 * the table view.
 */
static void gnss_note_sky(const ubx_msg_t *m, uint64_t rx_mono_ms)
{
	ubx_nav_sat_iter_t it;
	ubx_nav_sat_sv_t sv;
	sts_gnss_sv_t staged[STS_GNSS_SKY_MAX_SV];
	uint8_t n = 0U;

	if (ubx_nav_sat_begin(m, &it) != 0) {
		return;
	}
	while ((n < (uint8_t)STS_GNSS_SKY_MAX_SV) &&
	       (ubx_nav_sat_next(&it, &sv) == 1)) {
		staged[n].gnss_id = sv.gnss_id;
		staged[n].sv_id = sv.sv_id;
		staged[n].cno_dbhz = sv.cno_dbhz;
		staged[n].elev_deg = sv.elev_deg;
		staged[n].azim_deg = sv.azim_deg;
		staged[n].used = sv.used;
		n++;
	}

	/* Staged outside the lock: the iteration is bounded but it is still a
	 * loop, and this mutex is taken by the 10 Hz render tick. */
	k_mutex_lock(&sky_mutex, K_FOREVER);
	memcpy(sky.sv, staged, (size_t)n * sizeof(staged[0]));
	if (n < (uint8_t)STS_GNSS_SKY_MAX_SV) {
		memset(&sky.sv[n], 0,
		       ((size_t)STS_GNSS_SKY_MAX_SV - n) * sizeof(sky.sv[0]));
	}
	sky.count = n;
	sky.itow_ms = it.itow_ms;
	sky.mono_ms = rx_mono_ms;
	k_mutex_unlock(&sky_mutex);
}

int sts_gnss_sky(sts_gnss_sky_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}
	/*
	 * Bounded, unlike the writer's K_FOREVER: the reader is the
	 * lowest-priority thread in the system and a render tick that cannot
	 * have the lock should skip the frame rather than wait behind a GNSS
	 * thread that is mid-parse. Zeroed on failure so a caller that ignores
	 * the return code draws an empty sky rather than a stack frame.
	 */
	if (k_mutex_lock(&sky_mutex, K_MSEC(5)) != 0) {
		memset(out, 0, sizeof(*out));
		return -EBUSY;
	}
	*out = sky;
	k_mutex_unlock(&sky_mutex);
	return 0;
}

static void gnss_drain_rx(void)
{
	bool tee_on;
	uint8_t b;

	/* A flash session owns the ring; sts_gnss_uart_raw_rx() drains it. Feeding
	 * loader bytes to the UBX parser would at best desync the loader and at
	 * worst hand gnssmgr a frame from a receiver that is not running firmware.
	 *
	 * The one exception is the maintenance tunnel, which suspends the receiver
	 * through the same flag but has no other drain of its own. */
	if (gs.fw_mode) {
		if (sts_mp_tunnel_gnss_open()) {
			gnss_drain_tunnel();
		}
		return;
	}

	/* Hoisted: one atomic read per pass, not per byte. */
	tee_on = sts_mp_gnss_tee_armed();

	while (ring_getc(&rx_ring, &b) == 0) {
		ubx_msg_t m;
		uint64_t rx_mono_ms;

		if (tee_on) {
			/* Before the parser consumes it — the classifier needs the
			 * state the byte is about to change. */
			gnss_tee_byte(&gtee, b,
				      parser.state == (uint8_t)UBX_PS_SYNC1);
		}

		if (ubx_parse_byte(&parser, b) != 1) {
			continue;
		}
		if (ubx_parser_msg(&parser, &m) != 0) {
			continue;
		}

		/*
		 * One arrival stamp per message, on the 64-bit monotonic clock.
		 * gnssmgr_on_msg() takes the full width because the UBX-TIM-TP
		 * record it produces is compared against a PPS capture timestamp
		 * that is also 64-bit; a 32-bit stamp stops being comparable with
		 * it after 49.71 days of uptime, silently and permanently.
		 */
		rx_mono_ms = sts_mono_ms();

		/*
		 * Note NAV-PVT arrival before gnssmgr consumes it: pairing a
		 * captured pulse with its qErr needs the iTOW *and* the instant it
		 * landed, and gnssmgr's status only keeps the former.
		 */
		if (ubx_msg_is(&m, UBX_CLASS_NAV, UBX_ID_NAV_PVT)) {
			ubx_nav_pvt_t pvt;

			if (ubx_parse_nav_pvt(&m, &pvt) == 0) {
				/* Retain the observation being replaced; see the
				 * gs.pvt_prev_* comment. */
				if (gs.have_pvt) {
					gs.pvt_prev_itow_ms = gs.pvt_itow_ms;
					gs.pvt_prev_rx_mono_ms = gs.pvt_rx_mono_ms;
					gs.have_pvt_prev = true;
				}
				gs.pvt_itow_ms = pvt.itow_ms;
				gs.pvt_rx_mono_ms = rx_mono_ms;
				gs.have_pvt = true;
				gnss_note_position(&pvt);
			}
		} else if (ubx_msg_is(&m, UBX_CLASS_NAV, UBX_ID_NAV_SAT)) {
			gnss_note_sky(&m, rx_mono_ms);
		}

		(void)gnssmgr_on_msg(&mgr, &m, rx_mono_ms);
	}

	/*
	 * Unconditional, not behind tee_on: a channel that was un-armed part way
	 * through the pass must not leave a run staged for the next one to prefix.
	 * It costs a compare when nothing is staged, and when something is the
	 * tee's own arming test drops it.
	 */
	gnss_tee_flush(&gtee);
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

/*
 * Apply at most one operator request. Runs on this thread, so every gnssmgr_*
 * call below is on the thread that owns the manager.
 */
static void gnss_drain_requests(uint32_t now_ms)
{
	k_spinlock_key_t key;
	gnssmgr_ecef_t pos;
	uint8_t kind;
	int rc;

	key = k_spin_lock(&req_lock);
	kind = gnss_req.kind;
	pos = gnss_req.pos;
	gnss_req.kind = (uint8_t)GNSS_REQ_NONE;
	k_spin_unlock(&req_lock, key);

	if (kind == (uint8_t)GNSS_REQ_NONE) {
		return;
	}

	/*
	 * A flash session owns USART3. gnssmgr is parked in GNSSMGR_ST_FW_UPDATE
	 * and would refuse the request anyway, but drop it here and say so: the
	 * operator asked for something that is not going to happen, and a
	 * -EPERM logged from three layers down names the wrong cause.
	 */
	if (gs.fw_mode) {
		sts_log(LOGR_SUB_GNSS, LOGR_WARN,
			"GNSS operator request ignored: a receiver firmware "
			"session owns the port");
		return;
	}

	if (kind == (uint8_t)GNSS_REQ_SURVEY) {
		rc = gnssmgr_request_survey(&mgr, now_ms);
		sts_log(LOGR_SUB_GNSS, (rc == 0) ? LOGR_NOTICE : LOGR_ERR,
			"operator requested survey-in (rc %d)", rc);
		return;
	}

	/*
	 * Fixed position. gnssmgr_set_stored_ecef() only records the constant —
	 * it does not re-issue TMODE, and the receiver stays in whatever timing
	 * mode it was last given. gnssmgr_start() re-runs the walk, whose TMODE
	 * step then builds its fixed-position form because a position is now
	 * held; it is the only public route to that, since the one other caller
	 * of the TMODE walk (gnssmgr_request_survey) clears the position first.
	 *
	 * gnssmgr_start() rather than gnssmgr_notify_reset(): the receiver has
	 * NOT restarted, so its measurements are still valid, and invalidating
	 * them would drop gnss_time_locked and stall the discipline loop for a
	 * navigation epoch over a configuration change.
	 */
	rc = gnssmgr_set_stored_ecef(&mgr, &pos);
	if (rc == 0) {
		rc = gnssmgr_start(&mgr, now_ms);
	}

	sts_log(LOGR_SUB_GNSS, (rc == 0) ? LOGR_NOTICE : LOGR_ERR,
		"operator set fixed ECEF %d,%d,%d cm (rc %d)", (int)pos.x_cm,
		(int)pos.y_cm, (int)pos.z_cm, rc);
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

		gnss_drain_rx();
		gnss_drain_requests(now_ms);
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

	/* Bind the maintenance tees' run classifier. It stays inert until a
	 * channel is armed, so this costs nothing on a board nobody is servicing. */
	gnss_tee_init(&gtee, gnss_tee_sink, NULL);

	if (gpio_is_ready_dt(&ant_off_mon)) {
		(void)gpio_pin_configure_dt(&ant_off_mon, GPIO_INPUT);
	}

	gnss_load_cfg(&cfg);
	gs.survey_acc_0p1mm = cfg.survey_acc_limit_0p1mm;

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
	 * exists; the parser may also be mid-frame across the reset, and so may
	 * the tee's sentence/frame classifier. */
	ubx_parser_reset(&parser);
	ring_reset(&rx_ring);
	gnss_tee_reset(&gtee);
	gs.have_pvt = false;
	/* Retained history goes with the live observation: an epoch from before
	 * this discontinuity must never name a pulse after it. */
	gs.have_pvt_prev = false;

	return gnssmgr_notify_reset(&mgr, now_ms);
}

void sts_gnss_ant_supervisor_start(void)
{
	gs.ant_supervisor_on = true;
}

/* ========================================================================= */
/* operator requests (sts_app.h)                                             */
/* ========================================================================= */

/*
 * Plausibility bound on an operator-supplied ECEF triple.
 *
 * The REST layer already boxes each axis at 1.5 earth radii, which stops an
 * absurd coordinate but passes (0,0,0) — the centre of the Earth — and every
 * other point inside the planet. A fixed position the receiver can never
 * reconcile with its observations does not fail loudly: the F9T accepts it,
 * reports fixed mode, and produces a time solution biased by whatever the
 * position error is. So bound the RADIUS, which is the quantity that actually
 * has to be terrestrial.
 *
 * WGS-84 radii run 6357 km (polar) to 6378 km (equatorial). 6000/6800 km leaves
 * roughly 350 km of margin on each side — far more than any site altitude — and
 * still rejects the whole interior of the Earth and anything in orbit.
 */
#define GNSS_ECEF_MIN_CM 600000000LL /* 6000 km */
#define GNSS_ECEF_MAX_CM 680000000LL /* 6800 km */

static bool gnss_ecef_plausible(int64_t x_cm, int64_t y_cm, int64_t z_cm)
{
	int64_t r2;

	/* Per-axis first: it is the int32 representability check gnssmgr_ecef_t
	 * needs, and it also keeps the three squares below inside int64. */
	if ((x_cm > GNSS_ECEF_MAX_CM) || (x_cm < -GNSS_ECEF_MAX_CM) ||
	    (y_cm > GNSS_ECEF_MAX_CM) || (y_cm < -GNSS_ECEF_MAX_CM) ||
	    (z_cm > GNSS_ECEF_MAX_CM) || (z_cm < -GNSS_ECEF_MAX_CM)) {
		return false;
	}

	r2 = (x_cm * x_cm) + (y_cm * y_cm) + (z_cm * z_cm);

	return (r2 >= (GNSS_ECEF_MIN_CM * GNSS_ECEF_MIN_CM)) &&
	       (r2 <= (GNSS_ECEF_MAX_CM * GNSS_ECEF_MAX_CM));
}

/** Claim the single request slot. -EBUSY when one is already pending. */
static int gnss_req_post(uint8_t kind, const gnssmgr_ecef_t *pos)
{
	k_spinlock_key_t key;
	int rc = 0;

	key = k_spin_lock(&req_lock);
	if (gnss_req.kind != (uint8_t)GNSS_REQ_NONE) {
		rc = -EBUSY;
	} else {
		gnss_req.kind = kind;
		if (pos != NULL) {
			gnss_req.pos = *pos;
		}
	}
	k_spin_unlock(&req_lock, key);

	return rc;
}

int sts_gnss_request_survey(bool start)
{
	if (!start) {
		/*
		 * core/gnssmgr has no survey abort and this file will not invent
		 * one. The three things a local "stop" could mean are all wrong:
		 * adopting the partial survey stores a position that has by
		 * definition not met its accuracy limit; restoring the previous
		 * position is impossible because gnssmgr_request_survey() cleared
		 * it; and leaving TMODE altogether turns a grandmaster back into
		 * a navigation receiver. Report the gap.
		 */
		return -ENOTSUP;
	}
	if (!gs.started) {
		return -ENODEV;
	}

	return gnss_req_post((uint8_t)GNSS_REQ_SURVEY, NULL);
}

int sts_gnss_set_fixed_ecef(int64_t x_cm, int64_t y_cm, int64_t z_cm)
{
	gnssmgr_ecef_t pos;

	if (!gnss_ecef_plausible(x_cm, y_cm, z_cm)) {
		return -EINVAL;
	}
	if (!gs.started) {
		return -ENODEV;
	}

	memset(&pos, 0, sizeof(pos));
	pos.x_cm = (int32_t)x_cm;
	pos.y_cm = (int32_t)y_cm;
	pos.z_cm = (int32_t)z_cm;
	/*
	 * The high-precision residuals stay zero: the contract is whole
	 * centimetres, so there is no 0.1 mm term to carry, and inventing one
	 * would claim precision the operator did not supply.
	 *
	 * The declared accuracy is the configured survey acceptance limit. It is
	 * the site's stated position-accuracy target and the only number on the
	 * board that describes how well this position is meant to be known;
	 * gnssmgr's own default (cfg.fixed_pos_acc_0p1mm == 0) would otherwise
	 * hand the receiver a claimed accuracy of zero.
	 */
	pos.acc_0p1mm = gs.survey_acc_0p1mm;
	pos.valid = true;

	return gnss_req_post((uint8_t)GNSS_REQ_FIXED, &pos);
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
	gnss_tee_reset(&gtee);
	gs.have_pvt = false;
	/* Retained history goes with the live observation: an epoch from before
	 * this discontinuity must never name a pulse after it. */
	gs.have_pvt_prev = false;

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
	gnss_tee_reset(&gtee);
	gs.have_pvt = false;
	/* Retained history goes with the live observation: an epoch from before
	 * this discontinuity must never name a pulse after it. */
	gs.have_pvt_prev = false;
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
