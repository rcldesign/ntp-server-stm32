/*
 * STS1000 "Meridian" — core/fault: 1 kHz GPIO scan, debounce, alarm aggregator.
 *
 * Platform-neutral C11. The `io_scan` thread reads the GPIOF and GPIOG input
 * data registers every millisecond and hands both words here; this module owns
 * the diff, the per-class debounce, the UI event stream, and the alarm latch
 * table that drives the holdover relay and the status RGB.
 *
 * There is no I/O expander on this board (see `docs/sts1000_fault_aggregation.md`
 * §1 for why): all 32 aggregated inputs sit on their own pin across two ports,
 * so every signal keeps its identity and a missed edge self-heals on the next
 * scan. That is what lets this module be a pure function of (f_idr, g_idr, t).
 *
 * Signal numbering is the concatenation of the two ports — GPIOF is bits 0..15
 * of the internal bitmap, GPIOG bits 16..31 — which is exactly `fault_sig_t`.
 *
 * Polarity. Every one of the 32 signals is **active low** in the sense this
 * module calls "asserted": buttons and the encoder switch idle high and pull
 * low when pressed; the three RT9742 nFLG flags and the nine INA228 ALERTs are
 * open-drain active-low; the reed switch closes to ground; and the ten
 * power-good lines read high when good, so low is the interesting state. The
 * normalisation is therefore a single XOR against FAULT_ACTIVE_LOW_MASK, and
 * "asserted" uniformly means *the condition worth reacting to*.
 *
 * Masks and bit assignments are verbatim from
 * `docs/sts1000_firmware_hardware_interface.md` §5.
 */

#ifndef STS1000_CORE_FAULT_FAULT_H_
#define STS1000_CORE_FAULT_FAULT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------- signal space */

/**
 * The 32 scanned signals. Values 0..15 are GPIOF bit positions, 16..31 are
 * GPIOG bit positions, so `fault_sig_t` doubles as the internal bitmap index.
 */
typedef enum {
	/* GPIOF — HMI, load-switch fault flags, backup power-good */
	FAULT_SIG_BUTTON_1 = 0,      /* PF0  low = pressed */
	FAULT_SIG_BUTTON_2 = 1,      /* PF1 */
	FAULT_SIG_BUTTON_3 = 2,      /* PF2 */
	FAULT_SIG_BUTTON_4 = 3,      /* PF3 */
	FAULT_SIG_BUTTON_5 = 4,      /* PF4 */
	FAULT_SIG_BUTTON_6 = 5,      /* PF5 */
	FAULT_SIG_BUTTON_7 = 6,      /* PF6 */
	FAULT_SIG_TOUCH_INT = 7,     /* PF7  low = touch controller wants service */
	FAULT_SIG_V_ANT_EN_FAULT = 8,  /* PF8  U27 RT9742 nFLG, antenna bias */
	FAULT_SIG_V_DISP_EN_FAULT = 9, /* PF9  U33 RT9742 nFLG, display 5 V */
	FAULT_SIG_PROX_WAKE = 10,    /* PF10 low = magnet present (reed switch) */
	FAULT_SIG_ENC_BUTTON = 11,   /* PF11 low = encoder switch pressed */
	FAULT_SIG_PANEL_LED_FAULT = 12, /* PF12 U55 RT9742 nFLG, panel LEDs */
	FAULT_SIG_INA_ALERT_PANEL = 13, /* PF13 U54 0x4C — the one alert on F */
	FAULT_SIG_BKP_STM_PG = 14,   /* PF14 high = STM backup supply good */
	FAULT_SIG_BKP_GPS_PG = 15,   /* PF15 high = GPS V_BCKP supply good */

	/* GPIOG[0:7] — rail power-good, high = good */
	FAULT_SIG_PG_3V3_GPS_LDO = 16, /* PG0 U22 LT3045 */
	FAULT_SIG_PG_OCXO_LDO = 17,    /* PG1 U39 */
	FAULT_SIG_PG_3V0_RF_LDO = 18,  /* PG2 U51 (R266 fitted: no masking) */
	FAULT_SIG_PG_5V_PSU = 19,      /* PG3 5 V buck */
	FAULT_SIG_PG_3V3_PSU = 20,     /* PG4 AP3441 */
	FAULT_SIG_PG_OCXO_PSU = 21,    /* PG5 divided OCXO_PSU_PG */
	FAULT_SIG_PG_RB_PSU = 22,      /* PG6 U40 MIC28516 */
	FAULT_SIG_PG_POE = 23,         /* PG7 U9 NCP1095 PGO, divided */

	/* GPIOG[8:15] — INA228 ALERT, open-drain low = alert */
	FAULT_SIG_INA_ALERT_V_POE = 24,   /* PG8  U10 0x40 */
	FAULT_SIG_INA_ALERT_3V3_STM = 25, /* PG9  U31 0x41 */
	FAULT_SIG_INA_ALERT_5V_DISP = 26, /* PG10 U32 0x42 */
	FAULT_SIG_INA_ALERT_3V3 = 27,     /* PG11 U30 0x43 */
	FAULT_SIG_INA_ALERT_3V3_GPS = 28, /* PG12 U23 0x4A */
	FAULT_SIG_INA_ALERT_V_ANT = 29,   /* PG13 U26 0x45 */
	FAULT_SIG_INA_ALERT_OCXO = 30,    /* PG14 U37 0x46 */
	FAULT_SIG_INA_ALERT_VCC_RB = 31,  /* PG15 U44 0x47 */

	FAULT_SIG_COUNT = 32,
} fault_sig_t;

/* Interface ref §5.1 — GPIOF masks. */
#define FAULT_MASK_F_BUTTONS       0x007FU /* PF0..PF6 */
#define FAULT_MASK_F_TOUCH         0x0080U /* PF7 */
#define FAULT_MASK_F_EN_FAULTS     0x1300U /* PF8, PF9, PF12 */
#define FAULT_MASK_F_PROX          0x0400U /* PF10 */
#define FAULT_MASK_F_ENC_BUTTON    0x0800U /* PF11 */
#define FAULT_MASK_F_INA_ALERT     0x2000U /* PF13 */
#define FAULT_MASK_F_BKP_PG        0xC000U /* PF14, PF15 */

/* Interface ref §5.2 — GPIOG masks. */
#define FAULT_MASK_G_PG_RAILS      0x00FFU /* PG0..PG7 */
#define FAULT_MASK_G_INA_ALERTS    0xFF00U /* PG8..PG15 */

/** Every signal, as a 32-bit combined bitmap. */
#define FAULT_SIG_ALL_MASK 0xFFFFFFFFU

/**
 * Signals whose electrical low means "asserted".
 *
 * All of them, as it happens — see the file header. Kept as a named constant so
 * the normalisation reads as a polarity table rather than a bare complement,
 * and so a future active-high signal has an obvious place to be recorded.
 */
#define FAULT_ACTIVE_LOW_MASK FAULT_SIG_ALL_MASK

/** Bit for @p sig in the combined 32-bit bitmap. */
#define FAULT_SIG_BIT(sig) ((uint32_t)1U << (unsigned int)(sig))

/** Short, stable name for @p sig, e.g. "BUTTON_1", "PG_POE". Never NULL. */
const char *fault_sig_name(fault_sig_t sig);

/** Debounce/dispatch class @p sig belongs to. */
typedef enum {
	FAULT_CLASS_BUTTON = 0, /* PF0..PF6, PF11 */
	FAULT_CLASS_TOUCH,      /* PF7 */
	FAULT_CLASS_PROX,       /* PF10 */
	FAULT_CLASS_EN_FAULT,   /* PF8, PF9, PF12 */
	FAULT_CLASS_PG,         /* PG0..PG7 */
	FAULT_CLASS_BKP_PG,     /* PF14, PF15 */
	FAULT_CLASS_INA_ALERT,  /* PF13, PG8..PG15 */
	FAULT_CLASS_COUNT,
} fault_class_t;

/** Class of @p sig. FAULT_CLASS_COUNT for an out-of-range signal. */
fault_class_t fault_sig_class(fault_sig_t sig);

/* -------------------------------------------------------------------- events */

typedef enum {
	/** Debounced press (edge ASSERT) or release (edge DEASSERT). */
	FAULT_EVT_BUTTON = 0,
	/** Hold passed the long-press threshold. Edge is always ASSERT. */
	FAULT_EVT_BUTTON_LONG,
	/** Auto-repeat tick while held. Edge is always ASSERT. */
	FAULT_EVT_BUTTON_REPEAT,
	/** Touch controller asserted its INT; service it over I²C. */
	FAULT_EVT_TOUCH,
	/** Reed switch: ASSERT = magnet present, DEASSERT = gone. */
	FAULT_EVT_PROX,
	/** RT9742 nFLG changed state. */
	FAULT_EVT_EN_FAULT,
	/** A rail power-good went high->low. Edge is always ASSERT. */
	FAULT_EVT_PG_FAULT,
	/** A rail power-good came back. Edge is always DEASSERT. */
	FAULT_EVT_PG_RECOVER,
	/** INA228 ALERT changed state; on ASSERT read that part's DIAG_ALRT. */
	FAULT_EVT_INA_ALERT,
	/** Supercap-backed backup rail power-good changed state. */
	FAULT_EVT_BKP_PG,
	FAULT_EVT_TYPE_COUNT,
} fault_evt_type_t;

typedef enum {
	FAULT_EDGE_DEASSERT = 0,
	FAULT_EDGE_ASSERT = 1,
} fault_edge_t;

typedef struct {
	uint8_t type;     /* fault_evt_type_t */
	uint8_t id;       /* fault_sig_t */
	uint8_t edge;     /* fault_edge_t */
	uint8_t _pad;
	uint32_t mono_ms; /* scan timestamp the event was committed at */
} fault_evt_t;

/**
 * Event queue depth. One scan can in principle commit all 32 signals at once,
 * but a consumer polling at 10 Hz sees at most a handful; 32 covers a full
 * simultaneous transition without the ctx growing a page.
 */
#ifndef FAULT_EVT_QUEUE_LEN
#define FAULT_EVT_QUEUE_LEN 32
#endif

/* ------------------------------------------------------------------- alarms */

/**
 * Alarm identifiers.
 *
 * Ids 0..31 mirror `fault_sig_t` one-for-one, so a scanned fault and its alarm
 * share a number. Ids 32 and up are raised by other modules (disc, gnssmgr,
 * refsel, thermal, pwrseq) through fault_alarm_set(). The whole space fits in a
 * uint64 mask, which is what the relay policy and the SNMP fault word consume.
 */
typedef enum {
	/* 0..31: the scanned signals, same numbering as fault_sig_t */
	FAULT_ALARM_REFERENCE_LOST = 32, /* refsel: no usable 10 MHz reference */
	FAULT_ALARM_GNSS_LOST,           /* gnssmgr: fix or PPS lost */
	FAULT_ALARM_ANTENNA_OPEN,
	FAULT_ALARM_ANTENNA_SHORT,
	FAULT_ALARM_OCXO_UNHEALTHY, /* disc: oven/DAC fault, Vc cmd != sense */
	FAULT_ALARM_DAC_FAULT,
	FAULT_ALARM_RB_FAULT, /* pwrseq: guarded Rb sequence failed */
	FAULT_ALARM_RB_OV,    /* autonomous 26 V latch tripped */
	FAULT_ALARM_THERMAL_WARN,
	FAULT_ALARM_THERMAL_CRITICAL,
	FAULT_ALARM_FAN_FAULT,
	FAULT_ALARM_POE_BUDGET, /* class cannot cover the requested load */
	FAULT_ALARM_I2C_WEDGE,
	FAULT_ALARM_NOR_FAULT,
	FAULT_ALARM_DISPLAY_FAULT,
	FAULT_ALARM_PFI, /* power-fail early warning asserted */
	FAULT_ALARM_TAMPER,
	FAULT_ALARM_COUNT,
} fault_alarm_id_t;

/** Bit for @p id in a 64-bit alarm mask. */
#define FAULT_ALARM_BIT(id) (UINT64_C(1) << (unsigned int)(id))

typedef struct {
	uint32_t first_ms; /* first assertion since the latch was last cleared */
	uint32_t last_ms;  /* most recent assertion */
	uint32_t count;    /* assertions since the latch was last cleared */
	bool active;       /* the condition is true right now */
	bool latched;      /* it has been true at least once since the clear */
} fault_alarm_t;

/**
 * Alarms that disqualify the holdover relay by default.
 *
 * Interface ref §8: K2 is normally *energized*; firmware drives PA6 high only
 * once service quality is met and drops it on any disqualifying fault. The
 * default set is everything that makes served time untrustworthy:
 *
 *   - power-good loss on a rail the timing chain depends on (the OCXO supply
 *     and LDO, the 3V3 and 5 V bucks, the GNSS LDO, and PoE itself);
 *   - INA228 alerts on those same rails;
 *   - loss of the reference or of GNSS — holdover is precisely what this relay
 *     annunciates;
 *   - an unhealthy OCXO or DAC;
 *   - thermal critical, and power-fail.
 *
 * Deliberately excluded, with reasons:
 *   - PG2 3V0_RF and PG6 RB_PSU, and the VCC_RB alert: losing the external
 *     reference or the rubidium reverts to the OCXO and service continues
 *     (spec §13, "Rb unlock / bad clock -> fail-safe revert to OCXO").
 *   - display, panel-LED and touch faults: spec §13 states these are UI-only
 *     and never block timing.
 *   - antenna open/short and the V_ANT alert: they are the *cause* of
 *     FAULT_ALARM_GNSS_LOST, which is already in the set; counting both would
 *     make the relay state depend on detection order.
 *   - backup-rail power-good: the supercaps back the RTC and GNSS ephemeris,
 *     not the served time.
 *   - I²C wedge, NOR fault, PoE budget, tamper: housekeeping and security, all
 *     of which leave the timing path running.
 *
 * The whole mask is replaceable at runtime via fault_relay_set_disqualify_mask().
 */
#define FAULT_RELAY_DISQUALIFY_DEFAULT                                          \
	(FAULT_ALARM_BIT(FAULT_SIG_PG_3V3_GPS_LDO) |                            \
	 FAULT_ALARM_BIT(FAULT_SIG_PG_OCXO_LDO) |                               \
	 FAULT_ALARM_BIT(FAULT_SIG_PG_5V_PSU) |                                 \
	 FAULT_ALARM_BIT(FAULT_SIG_PG_3V3_PSU) |                                \
	 FAULT_ALARM_BIT(FAULT_SIG_PG_OCXO_PSU) |                               \
	 FAULT_ALARM_BIT(FAULT_SIG_PG_POE) |                                    \
	 FAULT_ALARM_BIT(FAULT_SIG_INA_ALERT_V_POE) |                           \
	 FAULT_ALARM_BIT(FAULT_SIG_INA_ALERT_3V3_STM) |                         \
	 FAULT_ALARM_BIT(FAULT_SIG_INA_ALERT_3V3) |                             \
	 FAULT_ALARM_BIT(FAULT_SIG_INA_ALERT_3V3_GPS) |                         \
	 FAULT_ALARM_BIT(FAULT_SIG_INA_ALERT_OCXO) |                            \
	 FAULT_ALARM_BIT(FAULT_ALARM_REFERENCE_LOST) |                          \
	 FAULT_ALARM_BIT(FAULT_ALARM_GNSS_LOST) |                               \
	 FAULT_ALARM_BIT(FAULT_ALARM_OCXO_UNHEALTHY) |                          \
	 FAULT_ALARM_BIT(FAULT_ALARM_DAC_FAULT) |                               \
	 FAULT_ALARM_BIT(FAULT_ALARM_THERMAL_CRITICAL) |                        \
	 FAULT_ALARM_BIT(FAULT_ALARM_PFI))

/* ------------------------------------------------------------------- config */

/**
 * Debounce and repeat timing.
 *
 * All millisecond fields are wall time, not sample counts, so a scan that
 * jitters or skips a tick still debounces over the intended interval. The one
 * sample-count field is the INA alert gate, which the interface ref specifies
 * in samples: two consecutive scans must agree before an ALERT is believed,
 * which rejects a single-scan glitch on an open-drain line without adding
 * latency to a real fault.
 */
typedef struct {
	uint16_t button_debounce_ms; /* §5.3 says 20-30 ms; default 25 */
	uint16_t en_fault_debounce_ms;
	uint16_t pg_debounce_ms; /* also used for the backup-rail PGs */
	uint16_t prox_debounce_ms;
	uint16_t ina_debounce_samples;
	uint16_t long_press_ms;
	uint16_t repeat_delay_ms;  /* from press to the first repeat */
	uint16_t repeat_period_ms; /* between repeats; 250 ms = 4 Hz */
} fault_cfg_t;

/** Fill @p cfg with the defaults documented on each field. */
void fault_cfg_default(fault_cfg_t *cfg);

/* ---------------------------------------------------------------- RGB policy */

/** Inputs to the status-RGB priority encoder (interface ref §8). */
typedef struct {
	bool locked;    /* disciplined and advertising stratum 1 */
	bool holdover;  /* flywheeling without GNSS */
	bool warming;   /* OCXO not yet warm, or converging */
	bool any_fault; /* any annunciated fault */
	bool identify;  /* operator asked the unit to identify itself */
} fault_rgb_in_t;

typedef enum {
	FAULT_RGB_OFF = 0,
	FAULT_RGB_GREEN,      /* locked stratum-1 */
	FAULT_RGB_AMBER,      /* warming or holdover */
	FAULT_RGB_RED,        /* unlocked / fault */
	FAULT_RGB_BLUE_PULSE, /* identify */
} fault_rgb_state_t;

/**
 * Priority-encode @p in into a single RGB state.
 *
 * Order, highest first: identify, fault, holdover-or-warming, locked, off.
 *
 * Identify outranks even a fault deliberately. It is an operator-commanded
 * locate — someone is standing in the rack looking for *this* box — and it is
 * transient by nature, so a red LED that refuses to blink blue defeats the
 * only purpose the function has. The fault is still annunciated on the relay,
 * the display, SNMP and the log; the RGB is the one channel identify borrows.
 *
 * FAULT_RGB_OFF for a NULL @p in and for an all-false input (pre-service).
 */
fault_rgb_state_t fault_rgb_state(const fault_rgb_in_t *in);

/* ------------------------------------------------------------------ context */

typedef struct {
	fault_cfg_t cfg;

	/* scan state */
	uint32_t stable;   /* committed asserted-bitmap */
	uint32_t pending;  /* signals with a candidate transition in flight */
	uint32_t last_raw; /* last (g_idr << 16) | f_idr, as read */
	uint32_t now_ms;
	uint32_t scans;
	bool started; /* at least one scan has been fed */

	uint32_t cand_first_ms[FAULT_SIG_COUNT];
	uint16_t cand_samples[FAULT_SIG_COUNT];

	/*
	 * Hold state for long-press and auto-repeat. Indexed by signal rather
	 * than by a packed button number: only eight slots are ever used, but
	 * the 216 wasted bytes buy away a signal-to-button lookup that would be
	 * a latent out-of-bounds if the class table and the button list ever
	 * disagreed.
	 */
	uint32_t press_ms[FAULT_SIG_COUNT];
	uint32_t next_repeat_ms[FAULT_SIG_COUNT];
	uint8_t long_sent[FAULT_SIG_COUNT];

	/* event queue (drop-newest, see fault_evt_dropped()) */
	fault_evt_t q[FAULT_EVT_QUEUE_LEN];
	uint16_t q_head;
	uint16_t q_len;
	uint32_t q_dropped;

	/* alarms */
	fault_alarm_t alarm[FAULT_ALARM_COUNT];
	uint64_t relay_disqualify;
} fault_ctx_t;

/**
 * Initialise @p ctx. @p cfg may be NULL for fault_cfg_default().
 *
 * The committed state starts fully de-asserted, so the first scans debounce
 * into whatever the board is actually doing and emit events for anything found
 * asserted. That is deliberate: a rail that is already down at boot must raise
 * its fault, not be adopted silently as the baseline.
 *
 * @retval 0        Initialised.
 * @retval -EINVAL  @p ctx is NULL, or a @p cfg field is zero where a non-zero
 *                  value is required (repeat period).
 */
int fault_init(fault_ctx_t *ctx, const fault_cfg_t *cfg);

/**
 * Feed one 1 kHz scan.
 *
 * @param ctx      Context.
 * @param f_idr    GPIOF input data register, as read.
 * @param g_idr    GPIOG input data register, as read.
 * @param mono_ms  Monotonic milliseconds; wrap-safe (unsigned differences).
 *
 * @retval 0        Scan processed.
 * @retval -EINVAL  @p ctx is NULL.
 */
int fault_scan_input(fault_ctx_t *ctx, uint16_t f_idr, uint16_t g_idr,
		     uint32_t mono_ms);

/** Debounced asserted-bitmap. Bit n is FAULT_SIG_BIT(n). */
uint32_t fault_state(const fault_ctx_t *ctx);

/** True when @p sig is currently asserted (debounced). */
bool fault_asserted(const fault_ctx_t *ctx, fault_sig_t sig);

/* -------------------------------------------------------------- event queue */

/**
 * Pop the oldest queued event.
 *
 * @retval 0        Event written to @p out.
 * @retval -EINVAL  @p ctx or @p out is NULL.
 * @retval -EAGAIN  Queue empty.
 */
int fault_evt_get(fault_ctx_t *ctx, fault_evt_t *out);

/** Number of events waiting. */
size_t fault_evt_count(const fault_ctx_t *ctx);

/**
 * Events discarded because the queue was full.
 *
 * The queue drops the **newest** event and counts it, rather than evicting the
 * oldest. In a cascade — a rail collapsing takes its INA alert, its power-good
 * and two load-switch flags with it — the first events are the root cause and
 * the rest are consequences. Keeping the head preserves the diagnosis; the
 * counter tells the consumer its picture is incomplete.
 */
uint32_t fault_evt_dropped(const fault_ctx_t *ctx);

/** Reset the dropped-event counter (after the consumer has logged it). */
void fault_evt_clear_dropped(fault_ctx_t *ctx);

/* ------------------------------------------------------------ alarm control */

/**
 * Assert or de-assert an alarm.
 *
 * Scanned signals (ids 0..31) are driven automatically by fault_scan_input();
 * this entry point is for the software-raised ids and for tests. A false->true
 * edge bumps the count and, if the alarm was not already latched, records
 * @p mono_ms as the first-seen time. De-asserting clears `active` and leaves
 * `latched` set.
 *
 * @retval 0        Applied.
 * @retval -EINVAL  @p ctx is NULL or @p id is out of range.
 */
int fault_alarm_set(fault_ctx_t *ctx, fault_alarm_id_t id, bool active,
		    uint32_t mono_ms);

/** Latch-table entry for @p id, or NULL if @p ctx is NULL or @p id invalid. */
const fault_alarm_t *fault_alarm_get(const fault_ctx_t *ctx,
				     fault_alarm_id_t id);

/** Mask of alarms whose condition is true right now (ARCHITECTURE.md §5). */
uint64_t fault_alarms(const fault_ctx_t *ctx);

/** Mask of alarms latched since their last clear, whether or not still true. */
uint64_t fault_alarms_latched(const fault_ctx_t *ctx);

/** True when any alarm is currently active. */
bool fault_any_active(const fault_ctx_t *ctx);

/**
 * Clear one latch.
 *
 * Latched-versus-momentary is the whole point of the distinction: a momentary
 * fault that has since gone away can be acknowledged and cleared, while a
 * condition that is *still true* must not be — clearing it would produce a
 * "no alarms" display over a live fault. A still-active alarm is refused.
 *
 * @retval 0        Latch cleared.
 * @retval -EINVAL  @p ctx is NULL or @p id out of range.
 * @retval -ENOENT  Nothing was latched.
 * @retval -EBUSY   The condition is still active; the latch is untouched.
 */
int fault_alarm_clear(fault_ctx_t *ctx, fault_alarm_id_t id);

/**
 * Clear every latch whose condition has gone away. Still-active alarms keep
 * their latch. Always succeeds; inspect fault_alarms_latched() for the result.
 *
 * @retval 0        Done.
 * @retval -EINVAL  @p ctx is NULL.
 */
int fault_alarm_clear_all(fault_ctx_t *ctx);

/* ------------------------------------------------------------ relay policy */

/**
 * Replace the set of alarms that disqualify the holdover relay.
 *
 * @retval 0        Applied.
 * @retval -EINVAL  @p ctx is NULL.
 */
int fault_relay_set_disqualify_mask(fault_ctx_t *ctx, uint64_t mask);

/** The current disqualifying set. 0 when @p ctx is NULL. */
uint64_t fault_relay_disqualify_mask(const fault_ctx_t *ctx);

/**
 * Disqualifying alarms that are active right now — the reason the relay may
 * not be energized, or 0 when nothing blocks it.
 */
uint64_t fault_relay_blocking(const fault_ctx_t *ctx);

/**
 * True when no disqualifying alarm is *active*.
 *
 * Eligibility deliberately ignores latched-but-cleared alarms: a fault that has
 * genuinely gone away must not hold the relay de-energized forever. Hysteresis
 * on re-energizing belongs to the service-quality gate that also consults this
 * (interface ref §8 — PA6 goes high only once quality is actually met).
 *
 * False for a NULL @p ctx: the fail-safe answer is "do not energize".
 */
bool fault_relay_eligible(const fault_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_FAULT_FAULT_H_ */
