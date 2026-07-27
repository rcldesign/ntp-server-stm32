/*
 * STS1000 "Meridian" — core/gnssmgr: ZED-F9T receiver lifecycle manager.
 *
 * Platform-neutral C11 (ARCHITECTURE.md §4). No dynamic allocation, no globals,
 * no platform headers. Dependency edge: gnssmgr -> ubx, util.
 *
 * Behaviour comes from spec §3.7 (GNSS engine), §2.2 (F9T roles),
 * docs/gnss_antenna_bias_supervisor.md §9 (antenna-state fusion) and
 * docs/sts1000_firmware_hardware_interface.md §2 Stage 5 / §10 caution 2.
 *
 * The module is pure state: it performs no I/O. Everything enters through
 *
 *   gnssmgr_on_msg()    one parsed UBX frame from the ubx parser
 *   gnssmgr_tick_1hz()  the 1 Hz housekeeping beat, carrying the antenna inputs
 *                       the caller samples elsewhere (PD4 and INA228 0x45)
 *   gnssmgr_step()      a cheap pump for ACK timeouts and retries
 *
 * and leaves through the caller-supplied gnssmgr_cb_t action callbacks
 * (SEND_UBX, SET_ANT_BIAS, STORE_ECEF, RAISE_ALARM). Time is a uint32_t
 * millisecond monotonic passed in by the caller — never read from a clock here
 * — and every comparison is written to survive its wrap.
 *
 * Threading: a gnssmgr_t is owned by one thread (the `gnss` thread, spec §1.2).
 * Nothing here is internally synchronised.
 *
 * What this module deliberately does NOT do:
 *   - set the UART baud rate. The F9T side and the STM32 side must change
 *     together, which only the glue can sequence, so CFG-UART1-BAUDRATE stays
 *     out of the config walk (the key is in ubx.h for the glue to use).
 *   - drive ANT_BIAS_EN during bring-up. Stage 5 owns the initial enable; this
 *     module only ever *cuts* the bias on a persistent short and restores it
 *     when an operator calls gnssmgr_ant_reenable().
 *   - keep per-satellite records. NAV-SAT is reduced to a counted summary; the
 *     skyplot iterates the frame itself with ubx_nav_sat_next().
 */

#ifndef STS1000_CORE_GNSSMGR_GNSSMGR_H_
#define STS1000_CORE_GNSSMGR_GNSSMGR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ubx/ubx.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Scratch space for the frame being emitted.
 *
 * The largest step is the TIMEPULSE-1 group: 14 keys, 56 key bytes + 30 value
 * bytes + 4 VALSET prefix + 8 frame overhead = 98 B. 256 leaves room to add
 * keys without a silent -ENOSPC (which would surface as CONFIG_FAILED anyway).
 */
#define GNSSMGR_TXBUF_SZ 256U

/** Lifecycle states. */
typedef enum {
	GNSSMGR_ST_IDLE = 0,      /**< initialised; gnssmgr_start() not called */
	GNSSMGR_ST_CONFIG,        /**< walking the config sequence, awaiting ACKs */
	GNSSMGR_ST_SURVEY_IN,     /**< TMODE survey-in running, watching NAV-SVIN */
	GNSSMGR_ST_FIXED,         /**< fixed-position timing mode — steady state */
	GNSSMGR_ST_CONFIG_FAILED, /**< retries exhausted; needs an explicit restart */
} gnssmgr_state_t;

/** Alarms raised through gnssmgr_cb_t::alarm. */
typedef enum {
	GNSSMGR_ALARM_CONFIG_FAILED = 0, /**< receiver would not accept its config */
	GNSSMGR_ALARM_ANT_OPEN,          /**< antenna current below the present threshold */
	GNSSMGR_ALARM_ANT_SHORT,         /**< persistent short; bias has been cut */
	GNSSMGR_ALARM_TIME_UNLOCKED,     /**< time lock lost after having been held */
	GNSSMGR_ALARM_COUNT,
} gnssmgr_alarm_t;

/** Antenna supervisor verdict. */
typedef enum {
	GNSSMGR_ANT_UNKNOWN = 0, /**< not enough evidence yet */
	GNSSMGR_ANT_OFF,         /**< commanded off — fault flags masked (bias doc §9.3) */
	GNSSMGR_ANT_OK,
	GNSSMGR_ANT_OPEN,
	GNSSMGR_ANT_SHORT,
} gnssmgr_ant_state_t;

/** Leap indication in NTP LI encoding (RFC 5905 §7.3). */
typedef enum {
	GNSSMGR_LI_NONE   = 0,
	GNSSMGR_LI_INSERT = 1, /**< last minute of the day has 61 s */
	GNSSMGR_LI_DELETE = 2, /**< last minute of the day has 59 s */
	GNSSMGR_LI_UNSYNC = 3, /**< leap state not known */
} gnssmgr_li_t;

/** An ECEF antenna-phase-centre position, u-blox high-precision form. */
typedef struct {
	int32_t  x_cm;
	int32_t  y_cm;
	int32_t  z_cm;
	int8_t   x_hp;          /**< 0.1 mm, -99..99 */
	int8_t   y_hp;
	int8_t   z_hp;
	uint32_t acc_0p1mm;     /**< 3D accuracy estimate of the fix above */
	bool     valid;
} gnssmgr_ecef_t;

/**
 * Action callbacks. @p user is passed back unchanged.
 *
 * Only @p send_ubx is mandatory. A NULL optional callback is simply not called,
 * which keeps a partial integration (or a focused test) from needing stubs.
 */
typedef struct {
	/**
	 * Transmit a complete UBX frame to the receiver.
	 * @return 0 on success; negative on failure, which gnssmgr treats as a
	 *         failed config attempt and retries.
	 */
	int (*send_ubx)(void *user, const uint8_t *frame, size_t len);

	/** Drive ANT_BIAS_EN (PC9). Called only to cut on a persistent short and
	 *  to restore from gnssmgr_ant_reenable(). */
	void (*set_ant_bias)(void *user, bool on);

	/** Persist a surveyed position (spec §3.7: a stored calibration constant). */
	void (*store_ecef)(void *user, const gnssmgr_ecef_t *pos);

	/** Alarm edge. Called only when the state of @p id actually changes. */
	void (*alarm)(void *user, gnssmgr_alarm_t id, bool active);

	void *user;
} gnssmgr_cb_t;

/**
 * Antenna evidence sampled by the caller, handed to gnssmgr_tick_1hz().
 *
 * The third source, UBX-MON-RF, arrives on its own through gnssmgr_on_msg().
 */
typedef struct {
	bool     ant_off_mon;       /**< GPS_ANT_OFF_MON (PD4): true = LNA commanded off */
	bool     bias_en;           /**< ANT_BIAS_EN (PC9) as currently driven */
	bool     current_valid;     /**< false when the INA228 read failed */
	uint32_t current_ua;        /**< antenna-rail current, INA228 0x45 */
} gnssmgr_ant_input_t;

/**
 * Tunables. Fill with gnssmgr_cfg_default() and override what you mean to.
 *
 * Defaults, and why:
 *
 *   ack_timeout_ms  1500  An F9T ACKs a VALSET in well under 100 ms; 1.5 s is
 *                         slack for a flash-layer write plus UART queueing.
 *   ack_retries        3  Four attempts total before CONFIG_FAILED.
 *   cfg_layers   RAM|BBR|FLASH  Spec §3.7 requires the config to survive a
 *                         power cycle.
 *   tp*_period_us  1e6   Both TIMEPULSEs at 1 Hz: PC6/TIM3 exists to cross-check
 *   tp*_len_us     1e5   PA0/TIM2 (interface ref §3), which needs the same rate.
 *   meas_rate_ms  1000   1 Hz navigation (spec §3.7).
 *   min_elev_deg    10   Conventional timing elevation mask.
 *   txready_pio      6   VERIFY — the PIO index behind F9T pin 19. The pin is
 *                        named in the docs, the PIO number is not; a wrong value
 *                        is NAKed, and PD5 is simply never trusted until the
 *                        remap ACKs (interface ref §10 caution 2, ARCHITECTURE
 *                        §10 invariant 8).
 *   survey_min_dur_s 3600         One hour: the shortest survey that gives a
 *                                 standalone receiver a metre-class position.
 *   survey_acc_limit_0p1mm 10000  1.0 m. A standalone (no-RTCM) survey-in will
 *                                 not converge below roughly this; with RTCM3
 *                                 on UART4 a tighter limit is reachable, hence
 *                                 the knob. Position error is a fixed delay
 *                                 error, not a rate error, so metre-class is
 *                                 a few nanoseconds of bias.
 *   fixed_pos_acc_0p1mm 0         0 = use the survey's own meanAcc.
 *   tacc_lock_ns   100 / unlock 250  Two thresholds, so a receiver sitting on
 *                                 the boundary does not chatter. disc owns the
 *                                 N-consecutive-seconds part of the §3.3 lock
 *                                 criteria; this is only the GNSS half.
 *   pvt_stale_ms  3000   Three missed NAV-PVTs drop the lock.
 *   ant_open_ua     5000 The supervisor's own DETECT threshold (~5 mA) from
 *                        gnss_antenna_bias_supervisor.md §6.1.
 *   ant_short_ua  170000 Between the 150 mA antenna maximum and the ~180 mA
 *                        R77 foldback clamp (bias doc §4). Reading it needs no
 *                        divider: INA228 0x45 across R89 150 mΩ is ±273 mA FS.
 *   ant_debounce       3 Persistence before acting, per the bias doc's
 *                        "a single-source transient must not shut anything down".
 *   leap_announce_s 86400 Assert the NTP LI bits for the last day, the common
 *                        server convention.
 *   itow_backstep_ms 60000  Secondary reset detector; see gnssmgr_notify_reset().
 */
typedef struct {
	uint32_t ack_timeout_ms;
	uint8_t  ack_retries;
	uint8_t  cfg_layers;              /**< UBX_CFG_LAYER_* bitmap */

	uint32_t tp1_period_us;
	uint32_t tp1_len_us;
	uint32_t tp2_period_us;
	uint32_t tp2_len_us;
	int16_t  tp_ant_cable_delay_ns;   /**< receiver-side delay; disc applies the
					    *  calibrated cable delay in software */
	bool     tp_utc_timegrid;         /**< true = align to UTC, false = GPS */

	uint16_t meas_rate_ms;
	uint16_t nav_rate_cycles;
	int8_t   min_elev_deg;
	bool     enable_gps;
	bool     enable_galileo;
	bool     enable_glonass;
	bool     enable_beidou;

	bool     txready_enable;
	uint8_t  txready_pio;
	uint16_t txready_threshold;       /**< units of 8 bytes */
	bool     txready_active_low;

	uint32_t survey_min_dur_s;
	uint32_t survey_acc_limit_0p1mm;
	uint32_t fixed_pos_acc_0p1mm;

	uint32_t tacc_lock_ns;
	uint32_t tacc_unlock_ns;
	uint32_t pvt_stale_ms;

	uint32_t ant_open_ua;
	uint32_t ant_short_ua;
	uint8_t  ant_debounce;

	uint32_t leap_announce_s;
	uint32_t itow_backstep_ms;
} gnssmgr_cfg_t;

/** GNSS fix / time-lock summary, refreshed from every NAV-PVT. */
typedef struct {
	bool     valid;        /**< at least one NAV-PVT has been decoded */
	uint32_t itow_ms;
	uint8_t  fix_type;     /**< UBX_FIX_* */
	bool     gnss_fix_ok;
	uint8_t  carr_soln;
	uint8_t  num_sv;
	uint32_t tacc_ns;
	bool     utc_valid;    /**< validDate && validTime && fullyResolved */
	bool     time_locked;  /**< fix usable for timing, tAcc inside the window */
	uint16_t year;
	uint8_t  month;
	uint8_t  day;
	uint8_t  hour;
	uint8_t  min;
	uint8_t  sec;
	int32_t  nano_ns;
	uint16_t pdop;
	int32_t  lon_1e7;
	int32_t  lat_1e7;
	int32_t  height_mm;
	uint32_t hacc_mm;
	uint32_t vacc_mm;
	uint32_t age_ms;       /**< since the last NAV-PVT, at the last tick */
} gnssmgr_status_t;

/** Leap-second schedule tracked from UBX-NAV-TIMELS. */
typedef struct {
	bool     valid;            /**< a NAV-TIMELS has been decoded */
	bool     curr_ls_valid;
	int8_t   current_ls;       /**< GPS-UTC offset, seconds */
	bool     event_valid;      /**< timeToLsEvent is meaningful */
	bool     event_pending;    /**< a non-zero change is scheduled */
	int8_t   ls_change;        /**< +1 insertion, -1 deletion */
	int32_t  time_to_event_s;
	uint16_t gps_wn;
	uint16_t gps_dn;
	uint8_t  src_curr;
	uint8_t  src_change;
} gnssmgr_leap_t;

/**
 * The latest UBX-TIM-TP, tagged with the pulse it describes.
 *
 * Convention implemented here: the receiver emits TIM-TP in the second *before*
 * the pulse it refers to, so the message's towMS is already the time-of-week of
 * that upcoming pulse. @p target_tow_ms is therefore the message's towMS
 * verbatim, and the discipline glue pairs a captured PA0 edge with the record
 * whose @p target_tow_ms matches the edge's own ToW — never with "the last one
 * received", which would be one second early.
 */
typedef struct {
	bool     valid;          /**< a TIM-TP has been decoded */
	bool     qerr_valid;     /**< false when the receiver flagged qErrInvalid */
	int32_t  qerr_ps;        /**< sawtooth quantisation error to subtract */
	uint32_t target_tow_ms;  /**< ToW of the pulse this qErr belongs to */
	uint32_t target_tow_sub_ms;
	uint16_t week;
	bool     time_base_utc;
	bool     utc_available;
	uint8_t  raim;
	uint32_t rx_mono_ms;     /**< caller clock when the message was decoded */
} gnssmgr_qerr_t;

/** Survey-in progress, from UBX-NAV-SVIN. */
typedef struct {
	bool     valid_msg;      /**< a NAV-SVIN has been decoded */
	bool     active;
	bool     valid;          /**< the surveyed position meets the limits */
	uint32_t dur_s;
	uint32_t mean_acc_0p1mm;
	uint32_t obs;
	gnssmgr_ecef_t pos;
} gnssmgr_svin_t;

/** Counted NAV-SAT summary. Per-SV detail stays in the frame. */
typedef struct {
	bool     valid;
	uint32_t itow_ms;
	uint8_t  tracked;    /**< SVs reported with a non-zero C/N0 */
	uint8_t  used;       /**< SVs used in the navigation solution */
	uint8_t  cno_max;    /**< best C/N0, dBHz */
	uint8_t  cno_avg_used; /**< mean C/N0 over the used SVs, dBHz */
} gnssmgr_sats_t;

/** Manager state. All fields private; use the gnssmgr_* functions. */
typedef struct {
	gnssmgr_cfg_t cfg;
	gnssmgr_cb_t  cb;

	uint8_t  state;        /**< gnssmgr_state_t */
	uint8_t  step;         /**< index into the config sequence */
	uint8_t  attempt;      /**< attempts spent on the current step */
	bool     awaiting_ack;
	bool     txready_acked; /**< CFG-TXREADY step accepted; PD5 may be trusted */
	uint32_t deadline_ms;

	bool           have_position;
	gnssmgr_ecef_t position;

	gnssmgr_status_t status;
	uint32_t         pvt_mono_ms;
	bool             pvt_seen;
	bool             itow_seen;
	uint32_t         last_itow_ms;

	gnssmgr_leap_t leap;
	gnssmgr_qerr_t qerr;
	gnssmgr_svin_t svin;
	gnssmgr_sats_t sats;

	/* Antenna supervisor. */
	uint8_t  ant_state;      /**< gnssmgr_ant_state_t, debounced */
	uint8_t  ant_candidate;  /**< gnssmgr_ant_state_t under test */
	uint8_t  ant_count;      /**< consecutive samples supporting the candidate */
	bool     ant_short_latched;
	bool     mon_rf_valid;
	uint8_t  mon_rf_ant_status;
	uint8_t  mon_rf_ant_power;
	uint8_t  mon_rf_jamming;

	uint32_t alarms;         /**< bitmask, bit n = gnssmgr_alarm_t n active */

	uint8_t  txbuf[GNSSMGR_TXBUF_SZ];
} gnssmgr_t;

/* ----------------------------------------------------------- lifecycle -- */

/** Fill @p cfg with the documented defaults. @retval 0 / -EINVAL. */
int gnssmgr_cfg_default(gnssmgr_cfg_t *cfg);

/**
 * Initialise a manager. Does not emit anything; call gnssmgr_start() when the
 * receiver is powered and out of reset (interface ref §2 Stage 5).
 *
 * @retval 0        Ready, state GNSSMGR_ST_IDLE.
 * @retval -EINVAL  NULL argument, no send_ubx callback, or a config value
 *                  outside its permitted range.
 */
int gnssmgr_init(gnssmgr_t *g, const gnssmgr_cfg_t *cfg, const gnssmgr_cb_t *cb);

/**
 * Begin (or restart) the configuration walk from its first step.
 *
 * Clears a CONFIG_FAILED alarm and the antenna debounce, keeps any stored
 * position, leap record and statistics.
 *
 * @retval 0        First step emitted.
 * @retval -EINVAL  NULL argument.
 * @retval -EIO     send_ubx() refused the frame. The walk is still armed and
 *                  the ACK timeout will retry it; the caller is told only so it
 *                  can log the transport failure.
 * @retval <0       The frame could not be built at all (a programming error);
 *                  the manager is left in GNSSMGR_ST_CONFIG_FAILED.
 */
int gnssmgr_start(gnssmgr_t *g, uint32_t mono_ms);

/**
 * Tell the manager the receiver has restarted and its configuration is gone.
 *
 * The glue calls this whenever it pulses GPS_RST_N or cycles GPS_PWR_EN. It is
 * the primary reset signal; the iTOW-backstep heuristic in gnssmgr_on_msg() is
 * only a net for a restart the glue did not cause. Equivalent to
 * gnssmgr_start() plus discarding volatile receiver-derived state.
 *
 * The stored position is kept: it describes the site, not the receiver.
 * Returns as gnssmgr_start().
 */
int gnssmgr_notify_reset(gnssmgr_t *g, uint32_t mono_ms);

/**
 * Pump ACK timeouts and retries. Cheap; call from the gnss thread whenever it
 * wakes (at least every ~100 ms so a timeout is not reported late).
 *
 * @retval 0        Nothing to do, or a retry/step was emitted.
 * @retval -EINVAL  NULL argument.
 */
int gnssmgr_step(gnssmgr_t *g, uint32_t mono_ms);

/**
 * Consume one parsed UBX frame.
 *
 * Unknown messages are ignored and reported as 0, not as an error: the receiver
 * emits messages this firmware never asked for.
 *
 * @retval 0        Consumed (or ignored).
 * @retval -EINVAL  NULL argument.
 * @retval -EBADMSG A message of a known type failed its typed decode.
 */
int gnssmgr_on_msg(gnssmgr_t *g, const ubx_msg_t *m, uint32_t mono_ms);

/**
 * 1 Hz housekeeping: one antenna-supervisor sample plus NAV-PVT staleness.
 *
 * Each call with a non-NULL @p ant is exactly one debounce sample. Every change
 * of antenna verdict needs @p ant_debounce consecutive samples agreeing — the
 * first one included, so GNSSMGR_ANT_UNKNOWN reaches GNSSMGR_ANT_OK no faster
 * than it would reach GNSSMGR_ANT_SHORT. That symmetry is the point: adopting
 * whatever the first sample says would let one spurious reading during rail
 * settling cut the antenna bias. The single exception is the commanded-off
 * state, which is a fact rather than a measurement and is adopted at once.
 *
 * @param ant  Antenna evidence; may be NULL to run only the staleness check.
 * @retval 0 / -EINVAL.
 */
int gnssmgr_tick_1hz(gnssmgr_t *g, const gnssmgr_ant_input_t *ant, uint32_t mono_ms);

/* --------------------------------------------------------- operations -- */

/**
 * Seed the stored antenna position (from NVS) so the receiver goes straight to
 * fixed mode instead of surveying. Call before gnssmgr_start().
 *
 * @retval 0        Stored.
 * @retval -EINVAL  NULL argument, or @p pos is not marked valid.
 */
int gnssmgr_set_stored_ecef(gnssmgr_t *g, const gnssmgr_ecef_t *pos);

/** Copy the stored position. @retval 0 / -EINVAL / -EAGAIN if none is held. */
int gnssmgr_position(const gnssmgr_t *g, gnssmgr_ecef_t *out);

/**
 * Explicit operator re-survey (spec §3.7: never automatic).
 *
 * Discards the stored position and arranges for the TMODE step to be issued in
 * its survey-in form. Called while the configuration walk is still short of the
 * TMODE step, it only drops the position and lets the walk arrive there
 * normally — it does not restart the walk at TMODE, which would skip the
 * message rates and the time-pulse setup.
 *
 * @retval 0        Survey requested.
 * @retval -EINVAL  NULL argument.
 * @retval -EPERM   The manager has not been started, or configuration failed —
 *                  restart it first.
 * @retval -EIO     As gnssmgr_start().
 */
int gnssmgr_request_survey(gnssmgr_t *g, uint32_t mono_ms);

/**
 * Restore antenna bias after a latched short.
 *
 * There is no automatic recovery from a short by design: a self-restoring bias
 * would flap into a genuine fault at the debounce period forever. Idempotent —
 * safe to call when nothing is latched.
 *
 * @retval 0 / -EINVAL.
 */
int gnssmgr_ant_reenable(gnssmgr_t *g);

/* ------------------------------------------------------------ getters -- */

/** Current lifecycle state, or GNSSMGR_ST_IDLE for a NULL manager. */
gnssmgr_state_t gnssmgr_get_state(const gnssmgr_t *g);

/** Active-alarm bitmask (bit n = gnssmgr_alarm_t n). 0 for a NULL manager. */
uint32_t gnssmgr_alarms(const gnssmgr_t *g);

/** Copy the fix/time-lock block. @retval 0 / -EINVAL / -EAGAIN before the first NAV-PVT. */
int gnssmgr_status(const gnssmgr_t *g, gnssmgr_status_t *out);

/** Copy the leap record. @retval 0 / -EINVAL / -EAGAIN before the first NAV-TIMELS. */
int gnssmgr_leap(const gnssmgr_t *g, gnssmgr_leap_t *out);

/** Leap indication for the NTP LI field and the PTP leap flags. */
gnssmgr_li_t gnssmgr_leap_indicator(const gnssmgr_t *g);

/**
 * Latest sawtooth record for the discipline glue.
 *
 * @retval 0        @p out filled; pair it with the PPS edge whose ToW equals
 *                  @p out->target_tow_ms.
 * @retval -EINVAL  NULL argument.
 * @retval -EAGAIN  No TIM-TP decoded yet.
 */
int gnssmgr_qerr_for_pps(const gnssmgr_t *g, gnssmgr_qerr_t *out);

/** Copy the survey-in progress block. @retval 0 / -EINVAL / -EAGAIN. */
int gnssmgr_svin(const gnssmgr_t *g, gnssmgr_svin_t *out);

/** Copy the NAV-SAT summary. @retval 0 / -EINVAL / -EAGAIN. */
int gnssmgr_sats(const gnssmgr_t *g, gnssmgr_sats_t *out);

/** Debounced antenna verdict. */
gnssmgr_ant_state_t gnssmgr_ant_get_state(const gnssmgr_t *g);

/** True while the antenna bias is cut by a latched short. */
bool gnssmgr_ant_short_latched(const gnssmgr_t *g);

/** True once the CFG-TXREADY remap has been accepted (ARCHITECTURE §10 rule 8). */
bool gnssmgr_txready_trusted(const gnssmgr_t *g);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_GNSSMGR_GNSSMGR_H_ */
