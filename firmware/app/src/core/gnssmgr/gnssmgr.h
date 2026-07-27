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
	GNSSMGR_ST_CONFIG_FAILED, /**< an essential step was refused or went
				    *  unanswered; needs an explicit restart */
	/**
	 * Firmware update in progress: the receiver belongs to core/fwupd.
	 *
	 * Entered by gnssmgr_fw_enter() and left by gnssmgr_fw_exit(). While in
	 * this state the manager sends nothing, ACK timeouts do not run, and
	 * arriving messages are ignored — the receiver is either in safeboot or
	 * rebooting, and a config retry aimed into a flash loader is the one thing
	 * that must not happen. Everything the manager believed about the receiver
	 * is dropped on entry, and gnssmgr_fw_exit() re-runs the whole
	 * configuration walk, which is spec §8.5's "restores timing config
	 * afterward".
	 */
	GNSSMGR_ST_FW_UPDATE,
} gnssmgr_state_t;

/**
 * Steps of the configuration walk, in the order they are sent.
 *
 * Public because gnssmgr_failed_step() names one: an operator staring at a
 * CONFIG_FAILED alarm needs to know *which* group the receiver refused, and
 * "step 5" is not an answer. gnssmgr_step_name() renders it for a log line.
 */
typedef enum {
	GNSSMGR_STEP_PORT = 0,    /**< UART1 protocol filters: UBX in/out, NMEA off */
	GNSSMGR_STEP_MSGOUT,      /**< per-message output rates */
	GNSSMGR_STEP_RATE,        /**< measurement/navigation rate, dynamic model, mask */
	GNSSMGR_STEP_SIGNAL,      /**< constellation enables */
	GNSSMGR_STEP_TP1,         /**< TIMEPULSE 1 (PA0/TIM2) */
	GNSSMGR_STEP_TP2,         /**< TIMEPULSE 2 (PC6/TIM3) */
	GNSSMGR_STEP_TXREADY,     /**< pin-19 TX_READY remap — advisory, see below */
	GNSSMGR_STEP_TMODE,       /**< survey-in or fixed position */
	GNSSMGR_STEP_COUNT,
} gnssmgr_step_id_t;

/** Human-readable name of @p step, "?" if out of range. Never NULL. */
const char *gnssmgr_step_name(gnssmgr_step_id_t step);

/** Alarms raised through gnssmgr_cb_t::alarm. */
typedef enum {
	GNSSMGR_ALARM_CONFIG_FAILED = 0, /**< an essential config step was refused */
	GNSSMGR_ALARM_CONFIG_DEGRADED,   /**< an advisory step was refused; the clock
					   *  runs, but without that feature */
	GNSSMGR_ALARM_ANT_OPEN,          /**< antenna current below the present threshold */
	GNSSMGR_ALARM_ANT_SHORT,         /**< persistent short; bias has been cut */
	GNSSMGR_ALARM_TIME_UNLOCKED,     /**< time lock lost after having been held */
	GNSSMGR_ALARM_SURVEY_REJECTED,   /**< a completed survey missed its own limits */
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
 *                        named in the docs, the PIO number is not. Because that
 *                        value is unverified, the TX_READY step is ADVISORY: if
 *                        the receiver NAKs it the walk carries on, TMODE is
 *                        still configured, gnssmgr_txready_trusted() stays
 *                        false, and CONFIG_DEGRADED is raised naming the step.
 *                        An unverified convenience feature must not be able to
 *                        stop a grandmaster from serving time (interface ref
 *                        §10 caution 2, ARCHITECTURE §10 invariant 8).
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
 * Two conventions matter here, and both exist to make one thing work: the
 * discipline glue pairing a captured PA0 edge with the qErr that belongs to
 * *that* edge.
 *
 * 1. Which pulse. The receiver emits TIM-TP in the second *before* the pulse it
 *    refers to, so the message's towMS already names the upcoming pulse. Pair
 *    on @p target_tow_ms; never on "the last record received", which is one
 *    second early.
 *
 * 2. Which timescale. TIM-TP reports its towMS on the timebase the time pulse
 *    is aligned to — UTC when CFG-TP-TIMEGRID_TPx is UTC, which is this board's
 *    default. UBX-NAV-PVT iTOW is always GPS. Left alone the two differ by the
 *    leap-second offset (18 s today), so a glue layer pairing by ToW would
 *    never match and the sawtooth correction would be silently lost — the loop
 *    would run on uncorrected PPS and nobody would see an error.
 *
 *    So @p target_tow_ms is ALWAYS normalised to the **GPS** time of week,
 *    converting from UTC with the tracked leap offset when needed, and wrapping
 *    the week (with @p week incremented) if the conversion crosses the
 *    boundary. @p raw_tow_ms keeps the value exactly as the receiver sent it.
 *
 *    When the pulse is UTC-aligned but the leap offset is not yet known, the
 *    conversion cannot be done: @p qerr_valid is cleared so the glue does not
 *    apply a correction it cannot place in time. @p valid stays true — the
 *    record is still worth showing in telemetry.
 */
typedef struct {
	bool     valid;          /**< a TIM-TP has been decoded */
	bool     qerr_valid;     /**< false when the receiver flagged qErrInvalid,
				   *  or the UTC->GPS conversion was impossible */
	int32_t  qerr_ps;        /**< sawtooth quantisation error to subtract */
	uint32_t target_tow_ms;  /**< GPS ToW of the pulse this qErr belongs to */
	uint32_t target_tow_sub_ms;
	uint32_t raw_tow_ms;     /**< towMS as received, on @p time_base_utc's scale */
	uint16_t week;           /**< GPS week of the target pulse, after normalisation */
	bool     time_base_utc;  /**< the receiver's raw timeBase flag */
	bool     tow_from_utc;   /**< target_tow_ms was converted from UTC */
	bool     utc_available;
	uint8_t  raim;
	/**
	 * Caller clock when the message was decoded, **64-bit**.
	 *
	 * Wider than the rest of this manager's millisecond bookkeeping, and
	 * deliberately so. Every other timestamp here is only ever used as a
	 * difference against another value from the same 32-bit clock, where the
	 * wrap cancels. This one is different: it is handed out to be compared
	 * against a PPS capture timestamp taken on the caller's 64-bit monotonic
	 * clock (disc_qerr_matches_pulse(), which requires the record to precede
	 * the capture by no more than a second). Truncated to 32 bits, that
	 * comparison holds for 49.71 days of uptime and then fails permanently:
	 * the capture timestamp passes UINT32_MAX and never comes back, so the
	 * computed lead is always ~4.29e9 ms, the sawtooth correction is refused
	 * every second thereafter, and nothing about it is loud. On an appliance
	 * specified to run for years that is not an edge case.
	 */
	uint64_t rx_mono_ms;
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

/** RF-front-end view fused from UBX-MON-RF, for telemetry and the supervisor. */
typedef struct {
	bool    valid;         /**< a MON-RF with at least one block has arrived */
	bool    ant_short;     /**< some block reported antStatus SHORT */
	bool    ant_open;      /**< some block reported antStatus OPEN */
	uint8_t ant_power;     /**< UBX_ANT_POWER_*, from the first block */
	uint8_t jamming_state; /**< worst UBX_JAMMING_* across the blocks */
} gnssmgr_rf_t;

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
	uint8_t  step;         /**< gnssmgr_step_id_t currently being configured */
	uint8_t  attempt;      /**< attempts spent on the current step */
	uint8_t  inflight;     /**< copies of the current step sent but unanswered */
	uint8_t  failed_step;  /**< step named by CONFIG_FAILED / CONFIG_DEGRADED */
	bool     awaiting_ack; /**< the ACK deadline is armed */
	bool     txready_acked; /**< CFG-TXREADY step accepted; PD5 may be trusted */
	uint32_t deadline_ms;

	bool           have_position;
	bool           svin_started; /**< a NAV-SVIN with active=1 seen since the
				       *  current survey was requested */
	gnssmgr_ecef_t position;

	gnssmgr_status_t status;
	uint32_t         pvt_mono_ms;
	bool             pvt_seen;
	bool             itow_seen;
	uint32_t         last_itow_ms;

	gnssmgr_leap_t leap;
	gnssmgr_qerr_t qerr;
	gnssmgr_qerr_t qerr_prev; /**< the record @p qerr replaced; see
				    *  gnssmgr_qerr_prev_for_pps() */
	gnssmgr_svin_t svin;
	gnssmgr_sats_t sats;

	/* Antenna supervisor. */
	uint8_t  ant_state;      /**< gnssmgr_ant_state_t, debounced */
	uint8_t  ant_candidate;  /**< gnssmgr_ant_state_t under test */
	uint8_t  ant_count;      /**< consecutive samples supporting the candidate */
	bool     ant_short_latched;
	gnssmgr_rf_t rf;

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
 * Hand the receiver over to a firmware update (spec §8.5).
 *
 * Suspends this manager: no frames are emitted, no ACK deadlines are evaluated,
 * and gnssmgr_on_msg() drops whatever arrives. Everything measured is
 * invalidated, because after the update it describes a different firmware — the
 * stored position and the leap schedule survive, exactly as they do across
 * gnssmgr_notify_reset(), because they describe the world rather than the
 * receiver.
 *
 * The caller must have suspended its own use of the UART; this call does not
 * touch the transport.
 *
 * @retval 0        Suspended; the state is GNSSMGR_ST_FW_UPDATE.
 * @retval -EINVAL  @p g is NULL.
 * @retval -EBUSY   Already in GNSSMGR_ST_FW_UPDATE.
 */
int gnssmgr_fw_enter(gnssmgr_t *g);

/**
 * Take the receiver back and restore its configuration.
 *
 * Re-runs the full configuration walk from the first step, which is what makes
 * the receiver usable again: a firmware update clears the receiver's
 * configuration, so every VALSET group — port filters, message rates, timepulse
 * setup, timing mode — has to be re-sent. Survey-in is *not* re-run when a
 * position is stored; the site has not moved.
 *
 * @retval 0        Configuration walk restarted.
 * @retval -EINVAL  @p g is NULL.
 * @retval -EPERM   Not in GNSSMGR_ST_FW_UPDATE.
 * @retval other    As gnssmgr_start(): the first frame could not be sent.
 */
int gnssmgr_fw_exit(gnssmgr_t *g, uint32_t mono_ms);

/** True while the receiver is handed over to a firmware update. */
bool gnssmgr_fw_active(const gnssmgr_t *g);

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
 * @p mono_ms is 64-bit here alone among the manager's entry points, because one
 * field it feeds — gnssmgr_qerr_t::rx_mono_ms — leaves this module to be
 * compared against a 64-bit PPS capture timestamp, and a 32-bit millisecond
 * clock stops being comparable with it after 49.71 days. Everything else this
 * value touches is the manager's own interval bookkeeping, which uses it modulo
 * 2^32 exactly as before, so passing the low half of the same clock the other
 * entry points get keeps every deadline consistent.
 *
 * @retval 0        Consumed (or ignored).
 * @retval -EINVAL  NULL argument.
 * @retval -EBADMSG A message of a known type failed its typed decode.
 */
int gnssmgr_on_msg(gnssmgr_t *g, const ubx_msg_t *m, uint64_t mono_ms);

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

/**
 * The record gnssmgr_qerr_for_pps() last replaced — one epoch of history.
 *
 * The latest record is not always the one that describes a given pulse, and for
 * most of every second it is provably not. TIM-TP names the *next* pulse, so
 * from the moment the receiver emits the record for pulse N+1 until pulse N+1
 * actually arrives, the pulse that has been captured is N and the only record
 * describing it is the one just superseded. A consumer running on the PPS edge
 * never sees this; a consumer on an unrelated timer sees it for most of the
 * second, and gets no sawtooth and no epoch for as long as its polling phase
 * stays where it is.
 *
 * So one epoch is kept. Between this and gnssmgr_qerr_for_pps() there is always
 * exactly one record on the correct side of any capture in the current second,
 * whatever phase the consumer polls at. Both still have to prove themselves by
 * ToW against the caller's independently-derived belief — see disc_name_pulse(),
 * which consumes exactly this pair.
 *
 * Cleared with the current record whenever the receiver's measurements are
 * invalidated (reset, firmware session): stale history must not outlive the
 * receiver state that produced it.
 *
 * @retval 0        @p out filled.
 * @retval -EINVAL  NULL argument.
 * @retval -EAGAIN  Only one TIM-TP has been decoded since the last reset, so
 *                  there is no superseded record yet.
 */
int gnssmgr_qerr_prev_for_pps(const gnssmgr_t *g, gnssmgr_qerr_t *out);

/** Copy the survey-in progress block. @retval 0 / -EINVAL / -EAGAIN. */
int gnssmgr_svin(const gnssmgr_t *g, gnssmgr_svin_t *out);

/** Copy the NAV-SAT summary. @retval 0 / -EINVAL / -EAGAIN. */
int gnssmgr_sats(const gnssmgr_t *g, gnssmgr_sats_t *out);

/** Copy the MON-RF view (antenna claims + jamming). @retval 0 / -EINVAL / -EAGAIN. */
int gnssmgr_rf(const gnssmgr_t *g, gnssmgr_rf_t *out);

/**
 * The step named by the CONFIG_FAILED or CONFIG_DEGRADED alarm.
 *
 * Meaningful only while one of those alarms is active; GNSSMGR_STEP_PORT
 * otherwise. Pair with gnssmgr_step_name() for a log line.
 */
gnssmgr_step_id_t gnssmgr_failed_step(const gnssmgr_t *g);

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
