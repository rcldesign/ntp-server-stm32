/*
 * STS1000 "Meridian" — core/gnssmgr: ZED-F9T receiver lifecycle manager.
 *
 * See gnssmgr.h for the contract. Layout of this file:
 *
 *   1. small helpers        wrap-safe time, alarm edges
 *   2. config-step builders one UBX-CFG-VALSET per step
 *   3. step sequencer       emit / ACK / timeout / retry / advance
 *   4. message handlers     NAV-PVT, NAV-SAT, NAV-TIMELS, NAV-SVIN, TIM-TP,
 *                           MON-RF, ACK
 *   5. antenna supervisor   three-source fusion with debounce
 *   6. public API
 *
 * The config walk sends exactly one VALSET at a time and waits for its ACK.
 * That is slower than one big frame but it is the only way to know *which*
 * group a receiver rejected — with everything in one frame a single unsupported
 * key NAKs the lot and the log says nothing useful.
 */

#include "gnssmgr/gnssmgr.h"

#include <errno.h>
#include <string.h>

#include "ubx/ubx.h"

/* GPS week in milliseconds; iTOW wraps here. */
#define GNSSMGR_WEEK_MS 604800000UL

/* ---------------------------------------------------------------- helpers -- */

/** Wrap-safe "now is at or past deadline" over a uint32_t millisecond clock. */
static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
	return (int32_t)(now_ms - deadline_ms) >= 0;
}

/** Wrap-safe elapsed milliseconds. */
static uint32_t time_since(uint32_t now_ms, uint32_t then_ms)
{
	return now_ms - then_ms;
}

/** Raise or clear an alarm, calling back only on the edge. */
static void alarm_set(gnssmgr_t *g, gnssmgr_alarm_t id, bool active)
{
	uint32_t bit = (uint32_t)1U << (unsigned int)id;
	bool cur = (g->alarms & bit) != 0U;

	if (cur == active) {
		return;
	}
	if (active) {
		g->alarms |= bit;
	} else {
		g->alarms &= ~bit;
	}
	if (g->cb.alarm != NULL) {
		g->cb.alarm(g->cb.user, id, active);
	}
}

/* -------------------------------------------------------- config builders -- */

/*
 * Step order. Protocols first so the receiver is speaking UBX-only before
 * anything else is asked of it; TMODE last because it is the one step whose
 * content depends on runtime state (surveyed or stored position).
 */
enum {
	GNSSMGR_STEP_PORT = 0,
	GNSSMGR_STEP_MSGOUT,
	GNSSMGR_STEP_RATE,
	GNSSMGR_STEP_SIGNAL,
	GNSSMGR_STEP_TP1,
	GNSSMGR_STEP_TP2,
	GNSSMGR_STEP_TXREADY,
	GNSSMGR_STEP_TMODE,
	GNSSMGR_STEP_COUNT,
};

typedef void (*gnssmgr_build_fn)(const gnssmgr_t *g, ubx_valset_t *v);

/*
 * Builders ignore per-key return codes on purpose: ubx_valset_t latches the
 * first failure and ubx_valset_end() reports it, so one check at the end covers
 * a fourteen-key group.
 */

static void build_port(const gnssmgr_t *g, ubx_valset_t *v)
{
	(void)g;
	/* UBX in and out, NMEA off both ways (spec §3.7). RTCM3 corrections
	 * arrive on UART4, not here, so UART1 does not accept them. */
	(void)ubx_valset_add_bool(v, UBX_CFG_UART1INPROT_UBX, true);
	(void)ubx_valset_add_bool(v, UBX_CFG_UART1INPROT_NMEA, false);
	(void)ubx_valset_add_bool(v, UBX_CFG_UART1INPROT_RTCM3X, false);
	(void)ubx_valset_add_bool(v, UBX_CFG_UART1OUTPROT_UBX, true);
	(void)ubx_valset_add_bool(v, UBX_CFG_UART1OUTPROT_NMEA, false);
	(void)ubx_valset_add_bool(v, UBX_CFG_UART1OUTPROT_RTCM3X, false);
}

static void build_msgout(const gnssmgr_t *g, ubx_valset_t *v)
{
	(void)g;
	/*
	 * Rate is in navigation epochs, so 1 = every epoch = 1 Hz at the default
	 * measurement rate. Only the messages this firmware decodes are enabled;
	 * NAV-SIG and MON-HW are named in spec §3.7 but have no consumer until
	 * the skyplot/diagnostics wave, and an enabled message nobody reads is
	 * just UART bandwidth and parser work.
	 */
	(void)ubx_valset_add_u1(v, UBX_CFG_MSGOUT_NAV_PVT_UART1, 1U);
	(void)ubx_valset_add_u1(v, UBX_CFG_MSGOUT_NAV_SAT_UART1, 1U);
	(void)ubx_valset_add_u1(v, UBX_CFG_MSGOUT_NAV_TIMELS_UART1, 1U);
	(void)ubx_valset_add_u1(v, UBX_CFG_MSGOUT_NAV_SVIN_UART1, 1U);
	(void)ubx_valset_add_u1(v, UBX_CFG_MSGOUT_TIM_TP_UART1, 1U);
	(void)ubx_valset_add_u1(v, UBX_CFG_MSGOUT_MON_RF_UART1, 1U);
}

static void build_rate(const gnssmgr_t *g, ubx_valset_t *v)
{
	(void)ubx_valset_add_u2(v, UBX_CFG_RATE_MEAS, g->cfg.meas_rate_ms);
	(void)ubx_valset_add_u2(v, UBX_CFG_RATE_NAV, g->cfg.nav_rate_cycles);
	/* Stationary dynamic model: this is a bolted-down grandmaster. */
	(void)ubx_valset_add_u1(v, UBX_CFG_NAVSPG_DYNMODEL, UBX_DYNMODEL_STATIONARY);
	(void)ubx_valset_add_i1(v, UBX_CFG_NAVSPG_INFIL_MINELEV, g->cfg.min_elev_deg);
}

static void build_signal(const gnssmgr_t *g, ubx_valset_t *v)
{
	(void)ubx_valset_add_bool(v, UBX_CFG_SIGNAL_GPS_ENA, g->cfg.enable_gps);
	(void)ubx_valset_add_bool(v, UBX_CFG_SIGNAL_GAL_ENA, g->cfg.enable_galileo);
	(void)ubx_valset_add_bool(v, UBX_CFG_SIGNAL_GLO_ENA, g->cfg.enable_glonass);
	(void)ubx_valset_add_bool(v, UBX_CFG_SIGNAL_BDS_ENA, g->cfg.enable_beidou);
}

static void build_tp1(const gnssmgr_t *g, ubx_valset_t *v)
{
	uint8_t grid = g->cfg.tp_utc_timegrid ? (uint8_t)UBX_TP_TIMEGRID_UTC
					      : (uint8_t)UBX_TP_TIMEGRID_GPS;

	(void)ubx_valset_add_u1(v, UBX_CFG_TP_PULSE_DEF, UBX_TP_PULSE_DEF_PERIOD);
	(void)ubx_valset_add_u1(v, UBX_CFG_TP_PULSE_LENGTH_DEF, UBX_TP_LENGTH_DEF_LENGTH);
	(void)ubx_valset_add_i2(v, UBX_CFG_TP_ANT_CABLEDELAY, g->cfg.tp_ant_cable_delay_ns);
	(void)ubx_valset_add_u4(v, UBX_CFG_TP_PERIOD_TP1, g->cfg.tp1_period_us);
	(void)ubx_valset_add_u4(v, UBX_CFG_TP_PERIOD_LOCK_TP1, g->cfg.tp1_period_us);
	(void)ubx_valset_add_u4(v, UBX_CFG_TP_LEN_TP1, g->cfg.tp1_len_us);
	(void)ubx_valset_add_u4(v, UBX_CFG_TP_LEN_LOCK_TP1, g->cfg.tp1_len_us);
	/* Board and cable delay are removed in software (spec §3.2 step 3), so
	 * the receiver applies none of its own. */
	(void)ubx_valset_add_i4(v, UBX_CFG_TP_USER_DELAY_TP1, 0);
	(void)ubx_valset_add_bool(v, UBX_CFG_TP_TP1_ENA, true);
	(void)ubx_valset_add_bool(v, UBX_CFG_TP_SYNC_GNSS_TP1, true);
	/* "Locked only": no pulse until the receiver has GNSS time. */
	(void)ubx_valset_add_bool(v, UBX_CFG_TP_USE_LOCKED_TP1, true);
	(void)ubx_valset_add_bool(v, UBX_CFG_TP_ALIGN_TO_TOW_TP1, true);
	(void)ubx_valset_add_bool(v, UBX_CFG_TP_POL_TP1, true); /* rising edge */
	(void)ubx_valset_add_u1(v, UBX_CFG_TP_TIMEGRID_TP1, grid);
}

static void build_tp2(const gnssmgr_t *g, ubx_valset_t *v)
{
	uint8_t grid = g->cfg.tp_utc_timegrid ? (uint8_t)UBX_TP_TIMEGRID_UTC
					      : (uint8_t)UBX_TP_TIMEGRID_GPS;

	/* TIMEPULSE2 → PC6/TIM3 is the cross-check for PA0/TIM2 (interface ref
	 * §3), so it defaults to the same rate and edge as TP1. */
	(void)ubx_valset_add_u4(v, UBX_CFG_TP_PERIOD_TP2, g->cfg.tp2_period_us);
	(void)ubx_valset_add_u4(v, UBX_CFG_TP_PERIOD_LOCK_TP2, g->cfg.tp2_period_us);
	(void)ubx_valset_add_u4(v, UBX_CFG_TP_LEN_TP2, g->cfg.tp2_len_us);
	(void)ubx_valset_add_u4(v, UBX_CFG_TP_LEN_LOCK_TP2, g->cfg.tp2_len_us);
	(void)ubx_valset_add_i4(v, UBX_CFG_TP_USER_DELAY_TP2, 0);
	(void)ubx_valset_add_bool(v, UBX_CFG_TP_TP2_ENA, true);
	(void)ubx_valset_add_bool(v, UBX_CFG_TP_SYNC_GNSS_TP2, true);
	(void)ubx_valset_add_bool(v, UBX_CFG_TP_USE_LOCKED_TP2, true);
	(void)ubx_valset_add_bool(v, UBX_CFG_TP_ALIGN_TO_TOW_TP2, true);
	(void)ubx_valset_add_bool(v, UBX_CFG_TP_POL_TP2, true);
	(void)ubx_valset_add_u1(v, UBX_CFG_TP_TIMEGRID_TP2, grid);
}

static void build_txready(const gnssmgr_t *g, ubx_valset_t *v)
{
	/*
	 * F9T pin 19 is GEOFENCE_STAT out of reset; PD5/EXTI5 means nothing
	 * until this remap is accepted (interface ref §10 caution 2, and
	 * ARCHITECTURE.md §10 invariant 8 — hence gnssmgr_txready_trusted()).
	 */
	(void)ubx_valset_add_bool(v, UBX_CFG_TXREADY_ENABLED, g->cfg.txready_enable);
	(void)ubx_valset_add_bool(v, UBX_CFG_TXREADY_POLARITY, g->cfg.txready_active_low);
	(void)ubx_valset_add_u1(v, UBX_CFG_TXREADY_PIN, g->cfg.txready_pio);
	(void)ubx_valset_add_u2(v, UBX_CFG_TXREADY_THRESHOLD, g->cfg.txready_threshold);
	(void)ubx_valset_add_u1(v, UBX_CFG_TXREADY_INTERFACE, UBX_TXREADY_IF_UART1);
}

static void build_tmode(const gnssmgr_t *g, ubx_valset_t *v)
{
	if (g->have_position) {
		uint32_t acc = (g->cfg.fixed_pos_acc_0p1mm != 0U)
				       ? g->cfg.fixed_pos_acc_0p1mm
				       : g->position.acc_0p1mm;

		(void)ubx_valset_add_u1(v, UBX_CFG_TMODE_MODE, UBX_TMODE_FIXED);
		(void)ubx_valset_add_u1(v, UBX_CFG_TMODE_POS_TYPE, UBX_TMODE_POS_ECEF);
		(void)ubx_valset_add_i4(v, UBX_CFG_TMODE_ECEF_X, g->position.x_cm);
		(void)ubx_valset_add_i4(v, UBX_CFG_TMODE_ECEF_Y, g->position.y_cm);
		(void)ubx_valset_add_i4(v, UBX_CFG_TMODE_ECEF_Z, g->position.z_cm);
		(void)ubx_valset_add_i1(v, UBX_CFG_TMODE_ECEF_X_HP, g->position.x_hp);
		(void)ubx_valset_add_i1(v, UBX_CFG_TMODE_ECEF_Y_HP, g->position.y_hp);
		(void)ubx_valset_add_i1(v, UBX_CFG_TMODE_ECEF_Z_HP, g->position.z_hp);
		(void)ubx_valset_add_u4(v, UBX_CFG_TMODE_FIXED_POS_ACC, acc);
	} else {
		(void)ubx_valset_add_u1(v, UBX_CFG_TMODE_MODE, UBX_TMODE_SURVEY_IN);
		(void)ubx_valset_add_u4(v, UBX_CFG_TMODE_SVIN_MIN_DUR,
					g->cfg.survey_min_dur_s);
		(void)ubx_valset_add_u4(v, UBX_CFG_TMODE_SVIN_ACC_LIMIT,
					g->cfg.survey_acc_limit_0p1mm);
	}
}

static const gnssmgr_build_fn k_steps[GNSSMGR_STEP_TMODE] = {
	[GNSSMGR_STEP_PORT] = build_port,
	[GNSSMGR_STEP_MSGOUT] = build_msgout,
	[GNSSMGR_STEP_RATE] = build_rate,
	[GNSSMGR_STEP_SIGNAL] = build_signal,
	[GNSSMGR_STEP_TP1] = build_tp1,
	[GNSSMGR_STEP_TP2] = build_tp2,
	[GNSSMGR_STEP_TXREADY] = build_txready,
};

/* --------------------------------------------------------- step sequencer -- */

static void config_failed(gnssmgr_t *g)
{
	g->awaiting_ack = false;
	g->state = (uint8_t)GNSSMGR_ST_CONFIG_FAILED;
	alarm_set(g, GNSSMGR_ALARM_CONFIG_FAILED, true);
}

/**
 * Build and send the current step, arming the ACK deadline.
 *
 * A build failure is a programming error (buffer too small, key/setter width
 * mismatch), not a receiver problem, so it goes straight to CONFIG_FAILED
 * rather than burning retries on a frame that will never be right.
 */
static int emit_step(gnssmgr_t *g, uint32_t now_ms)
{
	ubx_valset_t v;
	int rc;

	rc = ubx_valset_begin(&v, g->txbuf, sizeof(g->txbuf), g->cfg.cfg_layers);
	if (rc == 0) {
		if (g->step < (uint8_t)GNSSMGR_STEP_TMODE) {
			k_steps[g->step](g, &v);
		} else {
			build_tmode(g, &v);
		}
		rc = ubx_valset_end(&v);
	}
	if (rc < 0) {
		config_failed(g);
		return rc;
	}

	g->attempt++;
	g->awaiting_ack = true;
	g->deadline_ms = now_ms + g->cfg.ack_timeout_ms;

	if (g->cb.send_ubx(g->cb.user, g->txbuf, (size_t)rc) != 0) {
		/*
		 * The transport refused it. The deadline is already armed, so
		 * the retry path handles it at the normal cadence instead of
		 * spinning on a UART that is not ready.
		 */
		return -EIO;
	}
	return 0;
}

/** Emit the current step again, or give up if the attempt budget is spent. */
static void retry_or_fail(gnssmgr_t *g, uint32_t now_ms)
{
	if (g->attempt > g->cfg.ack_retries) {
		config_failed(g);
		return;
	}
	(void)emit_step(g, now_ms);
}

/** Move past the acknowledged step. */
static void advance_step(gnssmgr_t *g, uint32_t now_ms)
{
	if (g->step == (uint8_t)GNSSMGR_STEP_TXREADY) {
		/* Only a remap we actually asked for makes PD5 meaningful. */
		g->txready_acked = g->cfg.txready_enable;
	}

	if (g->step >= (uint8_t)GNSSMGR_STEP_TMODE) {
		g->awaiting_ack = false;
		g->state = g->have_position ? (uint8_t)GNSSMGR_ST_FIXED
					    : (uint8_t)GNSSMGR_ST_SURVEY_IN;
		return;
	}

	g->step++;
	g->attempt = 0U;
	(void)emit_step(g, now_ms);
}

/** Restart the walk at @p step. */
static int begin_walk(gnssmgr_t *g, uint8_t step, uint32_t now_ms)
{
	g->state = (uint8_t)GNSSMGR_ST_CONFIG;
	g->step = step;
	g->attempt = 0U;
	g->awaiting_ack = false;
	if (step <= (uint8_t)GNSSMGR_STEP_TXREADY) {
		/* The remap is about to be re-sent, so PD5 is untrustworthy
		 * again until its ACK lands (ARCHITECTURE.md §10 invariant 8). */
		g->txready_acked = false;
	}
	alarm_set(g, GNSSMGR_ALARM_CONFIG_FAILED, false);
	return emit_step(g, now_ms);
}

/* ------------------------------------------------------- message handlers -- */

/**
 * Did the receiver's time-of-week jump backwards far enough to mean a restart?
 *
 * A week rollover is a legitimate backwards jump of nearly a full week and must
 * not be mistaken for one. This is only a safety net: a receiver that restarts
 * with its backup domain intact keeps counting and never trips it, which is why
 * gnssmgr_notify_reset() exists and is the primary signal.
 */
static bool itow_backstep(const gnssmgr_t *g, uint32_t itow_ms)
{
	uint32_t prev = g->last_itow_ms;

	if (itow_ms >= prev) {
		return false;
	}
	if ((prev > (GNSSMGR_WEEK_MS - g->cfg.itow_backstep_ms)) &&
	    (itow_ms < g->cfg.itow_backstep_ms)) {
		return false; /* week rollover */
	}
	return (prev - itow_ms) > g->cfg.itow_backstep_ms;
}

static void on_nav_pvt(gnssmgr_t *g, const ubx_nav_pvt_t *p, uint32_t now_ms)
{
	bool restarted = g->itow_seen && itow_backstep(g, p->itow_ms);
	bool usable;
	bool utc_ok;
	bool locked;

	g->itow_seen = true;
	g->last_itow_ms = p->itow_ms;

	usable = p->gnss_fix_ok && ((p->fix_type == (uint8_t)UBX_FIX_3D) ||
				    (p->fix_type == (uint8_t)UBX_FIX_TIME_ONLY));
	utc_ok = p->valid_date && p->valid_time && p->fully_resolved;

	if (!usable || !utc_ok) {
		locked = false;
	} else if (g->status.time_locked) {
		/* Hysteresis: a receiver sitting on the threshold must not
		 * chatter the lock flag once per second. */
		locked = (p->tacc_ns <= g->cfg.tacc_unlock_ns);
	} else {
		locked = (p->tacc_ns <= g->cfg.tacc_lock_ns);
	}

	g->status.valid = true;
	g->status.itow_ms = p->itow_ms;
	g->status.fix_type = p->fix_type;
	g->status.gnss_fix_ok = p->gnss_fix_ok;
	g->status.carr_soln = p->carr_soln;
	g->status.num_sv = p->num_sv;
	g->status.tacc_ns = p->tacc_ns;
	g->status.utc_valid = utc_ok;
	g->status.year = p->year;
	g->status.month = p->month;
	g->status.day = p->day;
	g->status.hour = p->hour;
	g->status.min = p->min;
	g->status.sec = p->sec;
	g->status.nano_ns = p->nano_ns;
	g->status.pdop = p->pdop;
	g->status.lon_1e7 = p->lon_1e7;
	g->status.lat_1e7 = p->lat_1e7;
	g->status.height_mm = p->height_mm;
	g->status.hacc_mm = p->hacc_mm;
	g->status.vacc_mm = p->vacc_mm;
	g->status.age_ms = 0U;

	/*
	 * Edge-only alarm. time_locked starts false, so the first false->true
	 * transition clears an alarm that was never raised (alarm_set() drops
	 * it) and only a genuine loss of lock raises one.
	 */
	if (locked != g->status.time_locked) {
		alarm_set(g, GNSSMGR_ALARM_TIME_UNLOCKED, !locked);
	}
	g->status.time_locked = locked;

	g->pvt_seen = true;
	g->pvt_mono_ms = now_ms;

	if (restarted && (g->state != (uint8_t)GNSSMGR_ST_IDLE)) {
		(void)begin_walk(g, (uint8_t)GNSSMGR_STEP_PORT, now_ms);
	}
}

static int on_nav_sat(gnssmgr_t *g, const ubx_msg_t *m)
{
	ubx_nav_sat_iter_t it;
	ubx_nav_sat_sv_t sv;
	uint32_t cno_sum = 0U;
	uint8_t tracked = 0U;
	uint8_t used = 0U;
	uint8_t cno_max = 0U;

	if (ubx_nav_sat_begin(m, &it) != 0) {
		return -EBADMSG;
	}
	while (ubx_nav_sat_next(&it, &sv) == 1) {
		if (sv.cno_dbhz > 0) {
			tracked++;
		}
		if (sv.cno_dbhz > cno_max) {
			cno_max = sv.cno_dbhz;
		}
		if (sv.used) {
			used++;
			cno_sum += sv.cno_dbhz;
		}
	}

	g->sats.valid = true;
	g->sats.itow_ms = it.itow_ms;
	g->sats.tracked = tracked;
	g->sats.used = used;
	g->sats.cno_max = cno_max;
	g->sats.cno_avg_used = (used > 0U) ? (uint8_t)(cno_sum / used) : 0U;
	return 0;
}

static void on_nav_timels(gnssmgr_t *g, const ubx_nav_timels_t *t)
{
	g->leap.valid = true;
	g->leap.curr_ls_valid = t->valid_curr_ls;
	g->leap.current_ls = t->curr_ls;
	g->leap.event_valid = t->valid_time_to_ls_event;
	g->leap.ls_change = t->ls_change;
	g->leap.time_to_event_s = t->time_to_ls_event_s;
	g->leap.gps_wn = t->date_of_ls_gps_wn;
	g->leap.gps_dn = t->date_of_ls_gps_dn;
	g->leap.src_curr = t->src_of_curr_ls;
	g->leap.src_change = t->src_of_ls_change;
	g->leap.event_pending = t->valid_time_to_ls_event && (t->ls_change != 0);
}

static void on_tim_tp(gnssmgr_t *g, const ubx_tim_tp_t *t, uint32_t now_ms)
{
	g->qerr.valid = true;
	/* towMS already names the NEXT pulse — see gnssmgr_qerr_t. */
	g->qerr.target_tow_ms = t->tow_ms;
	g->qerr.target_tow_sub_ms = t->tow_sub_ms;
	g->qerr.qerr_ps = t->qerr_ps;
	g->qerr.qerr_valid = !t->qerr_invalid;
	g->qerr.week = t->week;
	g->qerr.time_base_utc = t->time_base_utc;
	g->qerr.utc_available = t->utc_available;
	g->qerr.raim = t->raim;
	g->qerr.rx_mono_ms = now_ms;
}

static void on_nav_svin(gnssmgr_t *g, const ubx_nav_svin_t *s, uint32_t now_ms)
{
	g->svin.valid_msg = true;
	g->svin.active = s->active;
	g->svin.valid = s->valid;
	g->svin.dur_s = s->dur_s;
	g->svin.mean_acc_0p1mm = s->mean_acc_0p1mm;
	g->svin.obs = s->obs;
	g->svin.pos.x_cm = s->mean_x_cm;
	g->svin.pos.y_cm = s->mean_y_cm;
	g->svin.pos.z_cm = s->mean_z_cm;
	g->svin.pos.x_hp = s->mean_x_hp;
	g->svin.pos.y_hp = s->mean_y_hp;
	g->svin.pos.z_hp = s->mean_z_hp;
	g->svin.pos.acc_0p1mm = s->mean_acc_0p1mm;
	g->svin.pos.valid = s->valid;

	if (g->state != (uint8_t)GNSSMGR_ST_SURVEY_IN) {
		return;
	}
	/* Complete = the receiver says the mean position is valid and it has
	 * stopped accumulating. Either alone is not a finished survey. */
	if (!s->valid || s->active) {
		return;
	}

	g->position = g->svin.pos;
	g->position.valid = true;
	g->have_position = true;
	if (g->cb.store_ecef != NULL) {
		g->cb.store_ecef(g->cb.user, &g->position);
	}

	/* Re-run the TMODE step, which now builds its fixed-position form. */
	(void)begin_walk(g, (uint8_t)GNSSMGR_STEP_TMODE, now_ms);
}

static int on_mon_rf(gnssmgr_t *g, const ubx_msg_t *m)
{
	ubx_mon_rf_iter_t it;
	ubx_mon_rf_block_t blk;
	uint8_t worst_ant = (uint8_t)UBX_ANT_STATUS_INIT;
	uint8_t power = (uint8_t)UBX_ANT_POWER_DONTKNOW;
	uint8_t jam = (uint8_t)UBX_JAMMING_UNKNOWN;
	bool any = false;

	if (ubx_mon_rf_begin(m, &it) != 0) {
		return -EBADMSG;
	}
	while (ubx_mon_rf_next(&it, &blk) == 1) {
		/*
		 * antStatus/antPower are receiver-global and repeat per block,
		 * but take the most alarming value rather than assuming that:
		 * the enum happens to be ordered INIT < DONTKNOW < OK < SHORT <
		 * OPEN, so a plain max picks a fault over an "ok".
		 */
		if (!any || (blk.ant_status > worst_ant)) {
			worst_ant = blk.ant_status;
		}
		if (!any || (blk.jamming_state > jam)) {
			jam = blk.jamming_state;
		}
		if (!any) {
			power = blk.ant_power;
		}
		any = true;
	}
	if (!any) {
		return 0; /* a block-less MON-RF tells us nothing */
	}

	g->mon_rf_valid = true;
	g->mon_rf_ant_status = worst_ant;
	g->mon_rf_ant_power = power;
	g->mon_rf_jamming = jam;
	return 0;
}

static void on_ack(gnssmgr_t *g, const ubx_ack_t *a, uint32_t now_ms)
{
	if (!g->awaiting_ack || (g->state != (uint8_t)GNSSMGR_ST_CONFIG)) {
		return;
	}
	/* Only VALSET acknowledgements drive the walk. */
	if ((a->cls_id != (uint8_t)UBX_CLASS_CFG) ||
	    (a->msg_id != (uint8_t)UBX_ID_CFG_VALSET)) {
		return;
	}

	if (a->ack) {
		g->awaiting_ack = false;
		advance_step(g, now_ms);
	} else {
		retry_or_fail(g, now_ms);
	}
}

/* ------------------------------------------------------ antenna supervisor -- */

/**
 * Fuse the three evidence sources into one verdict.
 *
 * Rules, from gnss_antenna_bias_supervisor.md §9.2/§9.3:
 *   - Commanded off wins outright. With Q11 off the feed node collapses and the
 *     analog supervisor reports SHORT and "absent" even though nothing is
 *     wrong, so those flags must be masked, not debounced.
 *   - Otherwise a fault claimed by *either* source counts, and SHORT beats
 *     OPEN. Debouncing, not source arbitration, is what stops a transient.
 */
static uint8_t ant_classify(const gnssmgr_t *g, const gnssmgr_ant_input_t *in)
{
	bool sh = false;
	bool op = false;

	if (!in->bias_en || in->ant_off_mon) {
		return (uint8_t)GNSSMGR_ANT_OFF;
	}

	if (g->mon_rf_valid) {
		if (g->mon_rf_ant_status == (uint8_t)UBX_ANT_STATUS_SHORT) {
			sh = true;
		} else if (g->mon_rf_ant_status == (uint8_t)UBX_ANT_STATUS_OPEN) {
			op = true;
		} else {
			/* INIT / DONTKNOW / OK carry no fault claim. */
		}
	}
	if (in->current_valid) {
		if (in->current_ua >= g->cfg.ant_short_ua) {
			sh = true;
		} else if (in->current_ua < g->cfg.ant_open_ua) {
			op = true;
		} else {
			/* In band. */
		}
	}

	if (sh) {
		return (uint8_t)GNSSMGR_ANT_SHORT;
	}
	if (op) {
		return (uint8_t)GNSSMGR_ANT_OPEN;
	}
	if (g->mon_rf_valid || in->current_valid) {
		return (uint8_t)GNSSMGR_ANT_OK;
	}
	return (uint8_t)GNSSMGR_ANT_UNKNOWN;
}

static void ant_enter(gnssmgr_t *g, uint8_t st)
{
	if (g->ant_state == st) {
		return;
	}
	g->ant_state = st;

	switch ((gnssmgr_ant_state_t)st) {
	case GNSSMGR_ANT_SHORT:
		alarm_set(g, GNSSMGR_ALARM_ANT_OPEN, false);
		alarm_set(g, GNSSMGR_ALARM_ANT_SHORT, true);
		if (!g->ant_short_latched) {
			g->ant_short_latched = true;
			if (g->cb.set_ant_bias != NULL) {
				g->cb.set_ant_bias(g->cb.user, false);
			}
		}
		break;
	case GNSSMGR_ANT_OPEN:
		alarm_set(g, GNSSMGR_ALARM_ANT_OPEN, true);
		break;
	case GNSSMGR_ANT_OK:
		alarm_set(g, GNSSMGR_ALARM_ANT_OPEN, false);
		/* ANT_SHORT is deliberately not cleared here: recovery from a
		 * short is gnssmgr_ant_reenable() and nothing else. */
		break;
	case GNSSMGR_ANT_OFF:
	case GNSSMGR_ANT_UNKNOWN:
	default:
		/* Masked or indeterminate — leave the alarms where they are. */
		break;
	}
}

static void ant_sample(gnssmgr_t *g, const gnssmgr_ant_input_t *in)
{
	uint8_t cls = ant_classify(g, in);

	if (cls == (uint8_t)GNSSMGR_ANT_OFF) {
		/* A commanded state, not a measurement: adopt it at once. */
		g->ant_candidate = cls;
		g->ant_count = 0U;
		ant_enter(g, cls);
		return;
	}

	if (cls == g->ant_state) {
		g->ant_candidate = cls;
		g->ant_count = 0U;
		return;
	}

	if (cls != g->ant_candidate) {
		g->ant_candidate = cls;
		g->ant_count = 1U;
	} else if (g->ant_count < 0xFF) {
		g->ant_count++;
	} else {
		/* Saturated; the latch below has long since fired. */
	}

	if (g->ant_count >= g->cfg.ant_debounce) {
		g->ant_count = 0U;
		ant_enter(g, cls);
	}
}

/* -------------------------------------------------------------- public API -- */

int gnssmgr_cfg_default(gnssmgr_cfg_t *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}
	(void)memset(cfg, 0, sizeof(*cfg));

	cfg->ack_timeout_ms = 1500U;
	cfg->ack_retries = 3U;
	cfg->cfg_layers = (uint8_t)UBX_CFG_LAYER_ALL;

	cfg->tp1_period_us = 1000000U;
	cfg->tp1_len_us = 100000U;
	cfg->tp2_period_us = 1000000U;
	cfg->tp2_len_us = 100000U;
	cfg->tp_ant_cable_delay_ns = 0;
	cfg->tp_utc_timegrid = true;

	cfg->meas_rate_ms = 1000U;
	cfg->nav_rate_cycles = 1U;
	cfg->min_elev_deg = 10;
	cfg->enable_gps = true;
	cfg->enable_galileo = true;
	cfg->enable_glonass = true;
	cfg->enable_beidou = true;

	cfg->txready_enable = true;
	cfg->txready_pio = 6U; /* VERIFY vs u-blox ICD — see gnssmgr.h */
	cfg->txready_threshold = 8U;
	cfg->txready_active_low = false;

	cfg->survey_min_dur_s = 3600U;
	cfg->survey_acc_limit_0p1mm = 10000U; /* 1.0 m */
	cfg->fixed_pos_acc_0p1mm = 0U;        /* use the survey's own meanAcc */

	cfg->tacc_lock_ns = 100U;
	cfg->tacc_unlock_ns = 250U;
	cfg->pvt_stale_ms = 3000U;

	cfg->ant_open_ua = 5000U;
	cfg->ant_short_ua = 170000U;
	cfg->ant_debounce = 3U;

	cfg->leap_announce_s = 86400U;
	cfg->itow_backstep_ms = 60000U;
	return 0;
}

static bool cfg_valid(const gnssmgr_cfg_t *c)
{
	if ((c->ack_timeout_ms == 0U) || (c->pvt_stale_ms == 0U)) {
		return false;
	}
	if ((c->cfg_layers & (uint8_t)UBX_CFG_LAYER_ALL) == 0U) {
		return false;
	}
	if ((c->meas_rate_ms == 0U) || (c->nav_rate_cycles == 0U)) {
		return false;
	}
	if ((c->min_elev_deg < -90) || (c->min_elev_deg > 90)) {
		return false;
	}
	if ((c->tp1_period_us == 0U) || (c->tp2_period_us == 0U)) {
		return false;
	}
	if ((c->tp1_len_us >= c->tp1_period_us) ||
	    (c->tp2_len_us >= c->tp2_period_us)) {
		return false;
	}
	if ((c->survey_min_dur_s == 0U) || (c->survey_acc_limit_0p1mm == 0U)) {
		return false;
	}
	if (c->tacc_unlock_ns < c->tacc_lock_ns) {
		return false;
	}
	if (c->ant_short_ua <= c->ant_open_ua) {
		return false;
	}
	if (c->ant_debounce == 0U) {
		return false;
	}
	if (c->itow_backstep_ms == 0U) {
		return false;
	}
	return true;
}

int gnssmgr_init(gnssmgr_t *g, const gnssmgr_cfg_t *cfg, const gnssmgr_cb_t *cb)
{
	if ((g == NULL) || (cfg == NULL) || (cb == NULL)) {
		return -EINVAL;
	}
	if (cb->send_ubx == NULL) {
		return -EINVAL;
	}
	if (!cfg_valid(cfg)) {
		return -EINVAL;
	}

	(void)memset(g, 0, sizeof(*g));
	g->cfg = *cfg;
	g->cb = *cb;
	g->state = (uint8_t)GNSSMGR_ST_IDLE;
	g->ant_state = (uint8_t)GNSSMGR_ANT_UNKNOWN;
	g->ant_candidate = (uint8_t)GNSSMGR_ANT_UNKNOWN;
	return 0;
}

int gnssmgr_start(gnssmgr_t *g, uint32_t mono_ms)
{
	if (g == NULL) {
		return -EINVAL;
	}

	g->ant_state = (uint8_t)GNSSMGR_ANT_UNKNOWN;
	g->ant_candidate = (uint8_t)GNSSMGR_ANT_UNKNOWN;
	g->ant_count = 0U;
	return begin_walk(g, (uint8_t)GNSSMGR_STEP_PORT, mono_ms);
}

int gnssmgr_notify_reset(gnssmgr_t *g, uint32_t mono_ms)
{
	if (g == NULL) {
		return -EINVAL;
	}

	/* Everything the receiver told us is about its previous life. The stored
	 * position survives — it is a site constant, not receiver state. */
	g->status.time_locked = false;
	g->status.valid = false;
	g->pvt_seen = false;
	g->itow_seen = false;
	g->qerr.valid = false;
	g->svin.valid_msg = false;
	g->sats.valid = false;
	g->mon_rf_valid = false;
	alarm_set(g, GNSSMGR_ALARM_TIME_UNLOCKED, false);

	return gnssmgr_start(g, mono_ms);
}

int gnssmgr_step(gnssmgr_t *g, uint32_t mono_ms)
{
	if (g == NULL) {
		return -EINVAL;
	}
	if (!g->awaiting_ack || (g->state != (uint8_t)GNSSMGR_ST_CONFIG)) {
		return 0;
	}
	if (!time_reached(mono_ms, g->deadline_ms)) {
		return 0;
	}

	retry_or_fail(g, mono_ms);
	return 0;
}

int gnssmgr_on_msg(gnssmgr_t *g, const ubx_msg_t *m, uint32_t mono_ms)
{
	if ((g == NULL) || (m == NULL)) {
		return -EINVAL;
	}

	if (ubx_msg_is(m, UBX_CLASS_NAV, UBX_ID_NAV_PVT)) {
		ubx_nav_pvt_t pvt;

		if (ubx_parse_nav_pvt(m, &pvt) != 0) {
			return -EBADMSG;
		}
		on_nav_pvt(g, &pvt, mono_ms);
		return 0;
	}
	if (ubx_msg_is(m, UBX_CLASS_NAV, UBX_ID_NAV_SAT)) {
		return on_nav_sat(g, m);
	}
	if (ubx_msg_is(m, UBX_CLASS_NAV, UBX_ID_NAV_TIMELS)) {
		ubx_nav_timels_t ls;

		if (ubx_parse_nav_timels(m, &ls) != 0) {
			return -EBADMSG;
		}
		on_nav_timels(g, &ls);
		return 0;
	}
	if (ubx_msg_is(m, UBX_CLASS_NAV, UBX_ID_NAV_SVIN)) {
		ubx_nav_svin_t sv;

		if (ubx_parse_nav_svin(m, &sv) != 0) {
			return -EBADMSG;
		}
		on_nav_svin(g, &sv, mono_ms);
		return 0;
	}
	if (ubx_msg_is(m, UBX_CLASS_TIM, UBX_ID_TIM_TP)) {
		ubx_tim_tp_t tp;

		if (ubx_parse_tim_tp(m, &tp) != 0) {
			return -EBADMSG;
		}
		on_tim_tp(g, &tp, mono_ms);
		return 0;
	}
	if (ubx_msg_is(m, UBX_CLASS_MON, UBX_ID_MON_RF)) {
		return on_mon_rf(g, m);
	}
	if (ubx_msg_is(m, UBX_CLASS_ACK, UBX_ID_ACK_ACK) ||
	    ubx_msg_is(m, UBX_CLASS_ACK, UBX_ID_ACK_NAK)) {
		ubx_ack_t ack;

		if (ubx_parse_ack(m, &ack) != 0) {
			return -EBADMSG;
		}
		on_ack(g, &ack, mono_ms);
		return 0;
	}

	return 0; /* not ours */
}

int gnssmgr_tick_1hz(gnssmgr_t *g, const gnssmgr_ant_input_t *ant, uint32_t mono_ms)
{
	if (g == NULL) {
		return -EINVAL;
	}

	if (g->pvt_seen) {
		uint32_t age = time_since(mono_ms, g->pvt_mono_ms);

		g->status.age_ms = age;
		if ((age > g->cfg.pvt_stale_ms) && g->status.time_locked) {
			g->status.time_locked = false;
			alarm_set(g, GNSSMGR_ALARM_TIME_UNLOCKED, true);
		}
	}

	if (ant != NULL) {
		ant_sample(g, ant);
	}
	return 0;
}

int gnssmgr_set_stored_ecef(gnssmgr_t *g, const gnssmgr_ecef_t *pos)
{
	if ((g == NULL) || (pos == NULL) || !pos->valid) {
		return -EINVAL;
	}

	g->position = *pos;
	g->have_position = true;
	return 0;
}

int gnssmgr_position(const gnssmgr_t *g, gnssmgr_ecef_t *out)
{
	if ((g == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (!g->have_position) {
		return -EAGAIN;
	}
	*out = g->position;
	return 0;
}

int gnssmgr_request_survey(gnssmgr_t *g, uint32_t mono_ms)
{
	if (g == NULL) {
		return -EINVAL;
	}
	if ((g->state == (uint8_t)GNSSMGR_ST_IDLE) ||
	    (g->state == (uint8_t)GNSSMGR_ST_CONFIG_FAILED)) {
		return -EPERM;
	}

	g->have_position = false;
	(void)memset(&g->position, 0, sizeof(g->position));
	(void)memset(&g->svin, 0, sizeof(g->svin));
	return begin_walk(g, (uint8_t)GNSSMGR_STEP_TMODE, mono_ms);
}

int gnssmgr_ant_reenable(gnssmgr_t *g)
{
	if (g == NULL) {
		return -EINVAL;
	}

	g->ant_short_latched = false;
	alarm_set(g, GNSSMGR_ALARM_ANT_SHORT, false);
	g->ant_state = (uint8_t)GNSSMGR_ANT_UNKNOWN;
	g->ant_candidate = (uint8_t)GNSSMGR_ANT_UNKNOWN;
	g->ant_count = 0U;
	if (g->cb.set_ant_bias != NULL) {
		g->cb.set_ant_bias(g->cb.user, true);
	}
	return 0;
}

gnssmgr_state_t gnssmgr_get_state(const gnssmgr_t *g)
{
	return (g != NULL) ? (gnssmgr_state_t)g->state : GNSSMGR_ST_IDLE;
}

uint32_t gnssmgr_alarms(const gnssmgr_t *g)
{
	return (g != NULL) ? g->alarms : 0U;
}

int gnssmgr_status(const gnssmgr_t *g, gnssmgr_status_t *out)
{
	if ((g == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (!g->status.valid) {
		return -EAGAIN;
	}
	*out = g->status;
	return 0;
}

int gnssmgr_leap(const gnssmgr_t *g, gnssmgr_leap_t *out)
{
	if ((g == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (!g->leap.valid) {
		return -EAGAIN;
	}
	*out = g->leap;
	return 0;
}

gnssmgr_li_t gnssmgr_leap_indicator(const gnssmgr_t *g)
{
	if ((g == NULL) || !g->leap.valid || !g->leap.curr_ls_valid) {
		return GNSSMGR_LI_UNSYNC;
	}
	if (!g->leap.event_pending) {
		return GNSSMGR_LI_NONE;
	}
	/* Announce only inside the window, and only while the event is still in
	 * the future — timeToLsEvent counts on past zero. */
	if ((g->leap.time_to_event_s < 0) ||
	    ((uint32_t)g->leap.time_to_event_s > g->cfg.leap_announce_s)) {
		return GNSSMGR_LI_NONE;
	}
	if (g->leap.ls_change > 0) {
		return GNSSMGR_LI_INSERT;
	}
	if (g->leap.ls_change < 0) {
		return GNSSMGR_LI_DELETE;
	}
	return GNSSMGR_LI_NONE;
}

int gnssmgr_qerr_for_pps(const gnssmgr_t *g, gnssmgr_qerr_t *out)
{
	if ((g == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (!g->qerr.valid) {
		return -EAGAIN;
	}
	*out = g->qerr;
	return 0;
}

int gnssmgr_svin(const gnssmgr_t *g, gnssmgr_svin_t *out)
{
	if ((g == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (!g->svin.valid_msg) {
		return -EAGAIN;
	}
	*out = g->svin;
	return 0;
}

int gnssmgr_sats(const gnssmgr_t *g, gnssmgr_sats_t *out)
{
	if ((g == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (!g->sats.valid) {
		return -EAGAIN;
	}
	*out = g->sats;
	return 0;
}

gnssmgr_ant_state_t gnssmgr_ant_get_state(const gnssmgr_t *g)
{
	return (g != NULL) ? (gnssmgr_ant_state_t)g->ant_state
			   : GNSSMGR_ANT_UNKNOWN;
}

bool gnssmgr_ant_short_latched(const gnssmgr_t *g)
{
	return (g != NULL) && g->ant_short_latched;
}

bool gnssmgr_txready_trusted(const gnssmgr_t *g)
{
	return (g != NULL) && g->txready_acked;
}
