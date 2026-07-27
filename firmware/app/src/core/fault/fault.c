/*
 * STS1000 "Meridian" — core/fault: 1 kHz GPIO scan, debounce, alarm aggregator.
 *
 * See fault.h for the contract, the polarity rule and the relay policy.
 *
 * Debounce model. Each signal carries a committed level (`stable`) and, when
 * the raw level disagrees with it, a candidate transition timed from the scan
 * that first saw the disagreement. Any scan where the raw level returns to the
 * committed one cancels the candidate outright, so a bouncing contact restarts
 * the timer on every bounce and only a genuinely settled level commits. A
 * transition therefore emits exactly one event no matter how dirty the edge is,
 * and a glitch shorter than the class interval emits none at all.
 *
 * Timing is wall-clock, taken from the caller's monotonic millisecond stamp
 * rather than counted scans, so a jittered or skipped tick still debounces over
 * the intended interval. The one exception is the INA228 alert gate, which the
 * interface ref specifies in samples; it is enforced as a sample count *in
 * addition to* the (zero) time interval.
 *
 * All time comparisons use unsigned differences, so the 32-bit millisecond
 * counter wrapping at ~49.7 days is a non-event.
 */

#include "fault/fault.h"

#include <errno.h>
#include <string.h>

#define BUTTON_COUNT 8

/* -------------------------------------------------------------- signal data */

/*
 * Class of each of the 32 signals, indexed by fault_sig_t. This is the table
 * interface ref §5.3 describes in prose; keeping it as data means the dispatch
 * has no per-signal branching to get wrong.
 */
static const uint8_t sig_class_tbl[FAULT_SIG_COUNT] = {
	FAULT_CLASS_BUTTON,    /* PF0  BUTTON_1 */
	FAULT_CLASS_BUTTON,    /* PF1  BUTTON_2 */
	FAULT_CLASS_BUTTON,    /* PF2  BUTTON_3 */
	FAULT_CLASS_BUTTON,    /* PF3  BUTTON_4 */
	FAULT_CLASS_BUTTON,    /* PF4  BUTTON_5 */
	FAULT_CLASS_BUTTON,    /* PF5  BUTTON_6 */
	FAULT_CLASS_BUTTON,    /* PF6  BUTTON_7 */
	FAULT_CLASS_TOUCH,     /* PF7  DISP_TOUCH_INT */
	FAULT_CLASS_EN_FAULT,  /* PF8  V_ANT_EN_FAULT */
	FAULT_CLASS_EN_FAULT,  /* PF9  V_DISP_EN_FAULT */
	FAULT_CLASS_PROX,      /* PF10 PROX_WAKE */
	FAULT_CLASS_BUTTON,    /* PF11 ENC_BUTTON */
	FAULT_CLASS_EN_FAULT,  /* PF12 PANEL_LED_FAULT */
	FAULT_CLASS_INA_ALERT, /* PF13 INA_ALERT_5V_PANEL */
	FAULT_CLASS_BKP_PG,    /* PF14 BKP_STM_PG */
	FAULT_CLASS_BKP_PG,    /* PF15 BKP_GPS_PG */
	FAULT_CLASS_PG,        /* PG0  3V3_GPS_LDO_PG */
	FAULT_CLASS_PG,        /* PG1  OCXO_LDO_PG */
	FAULT_CLASS_PG,        /* PG2  3V0_RF_LDO_PG */
	FAULT_CLASS_PG,        /* PG3  5V_PSU_PG */
	FAULT_CLASS_PG,        /* PG4  3V3_PSU_PG */
	FAULT_CLASS_PG,        /* PG5  OCXO_PSU_PG */
	FAULT_CLASS_PG,        /* PG6  RB_PSU_PG */
	FAULT_CLASS_PG,        /* PG7  POE_PG */
	FAULT_CLASS_INA_ALERT, /* PG8  INA_ALERT_V_POE */
	FAULT_CLASS_INA_ALERT, /* PG9  INA_ALERT_3V3_STM */
	FAULT_CLASS_INA_ALERT, /* PG10 INA_ALERT_5V_DISP */
	FAULT_CLASS_INA_ALERT, /* PG11 INA_ALERT_3V3 */
	FAULT_CLASS_INA_ALERT, /* PG12 INA_ALERT_3V3_GPS */
	FAULT_CLASS_INA_ALERT, /* PG13 INA_ALERT_V_ANT */
	FAULT_CLASS_INA_ALERT, /* PG14 INA_ALERT_OCXO */
	FAULT_CLASS_INA_ALERT, /* PG15 INA_ALERT_VCC_RB */
};

static const char *const sig_name_tbl[FAULT_SIG_COUNT] = {
	"BUTTON_1",	    "BUTTON_2",		"BUTTON_3",
	"BUTTON_4",	    "BUTTON_5",		"BUTTON_6",
	"BUTTON_7",	    "DISP_TOUCH_INT",	"V_ANT_EN_FAULT",
	"V_DISP_EN_FAULT",  "PROX_WAKE",	"ENC_BUTTON",
	"PANEL_LED_FAULT",  "INA_ALERT_5V_PANEL", "BKP_STM_PG",
	"BKP_GPS_PG",	    "3V3_GPS_LDO_PG",	"OCXO_LDO_PG",
	"3V0_RF_LDO_PG",    "5V_PSU_PG",	"3V3_PSU_PG",
	"OCXO_PSU_PG",	    "RB_PSU_PG",	"POE_PG",
	"INA_ALERT_V_POE",  "INA_ALERT_3V3_STM", "INA_ALERT_5V_DISP",
	"INA_ALERT_3V3",    "INA_ALERT_3V3_GPS", "INA_ALERT_V_ANT",
	"INA_ALERT_OCXO",   "INA_ALERT_VCC_RB",
};

/* The eight signals that behave as buttons, in hold-state index order. */
static const uint8_t button_sig_tbl[BUTTON_COUNT] = {
	FAULT_SIG_BUTTON_1, FAULT_SIG_BUTTON_2, FAULT_SIG_BUTTON_3,
	FAULT_SIG_BUTTON_4, FAULT_SIG_BUTTON_5, FAULT_SIG_BUTTON_6,
	FAULT_SIG_BUTTON_7, FAULT_SIG_ENC_BUTTON,
};

const char *fault_sig_name(fault_sig_t sig)
{
	if ((unsigned int)sig >= (unsigned int)FAULT_SIG_COUNT) {
		return "invalid";
	}
	return sig_name_tbl[(unsigned int)sig];
}

fault_class_t fault_sig_class(fault_sig_t sig)
{
	if ((unsigned int)sig >= (unsigned int)FAULT_SIG_COUNT) {
		return FAULT_CLASS_COUNT;
	}
	return (fault_class_t)sig_class_tbl[(unsigned int)sig];
}

/* -------------------------------------------------------------------- config */

void fault_cfg_default(fault_cfg_t *cfg)
{
	if (cfg == NULL) {
		return;
	}

	cfg->button_debounce_ms = 25U; /* interface ref §5.3: 20-30 ms */
	cfg->en_fault_debounce_ms = 5U;
	cfg->pg_debounce_ms = 5U;
	cfg->prox_debounce_ms = 50U;
	cfg->ina_debounce_samples = 2U;
	cfg->long_press_ms = 800U;
	cfg->repeat_delay_ms = 1000U;
	cfg->repeat_period_ms = 250U; /* 4 Hz */
}

/* Debounce interval, in milliseconds, for a signal class. */
static uint32_t class_debounce_ms(const fault_ctx_t *ctx, fault_class_t cls)
{
	switch (cls) {
	case FAULT_CLASS_BUTTON:
		return ctx->cfg.button_debounce_ms;
	case FAULT_CLASS_EN_FAULT:
		return ctx->cfg.en_fault_debounce_ms;
	case FAULT_CLASS_PG:
	case FAULT_CLASS_BKP_PG:
		return ctx->cfg.pg_debounce_ms;
	case FAULT_CLASS_PROX:
		return ctx->cfg.prox_debounce_ms;
	case FAULT_CLASS_TOUCH:
	case FAULT_CLASS_INA_ALERT:
	default:
		/* The touch INT has no debounce cap on the board and is a
		 * service request, not a level to filter; the INA alert gate is
		 * expressed in samples instead. */
		return 0U;
	}
}

/* Minimum consecutive agreeing scans for a signal class. */
static uint32_t class_debounce_samples(const fault_ctx_t *ctx, fault_class_t cls)
{
	if (cls == FAULT_CLASS_INA_ALERT) {
		return ctx->cfg.ina_debounce_samples;
	}
	return 1U;
}

int fault_init(fault_ctx_t *ctx, const fault_cfg_t *cfg)
{
	if (ctx == NULL) {
		return -EINVAL;
	}

	memset(ctx, 0, sizeof(*ctx));

	if (cfg != NULL) {
		ctx->cfg = *cfg;
	} else {
		fault_cfg_default(&ctx->cfg);
	}

	/* A zero repeat period would divide the hold time by nothing and fire
	 * every scan; a zero sample gate would defeat the INA glitch filter. */
	if ((ctx->cfg.repeat_period_ms == 0U) ||
	    (ctx->cfg.ina_debounce_samples == 0U)) {
		return -EINVAL;
	}

	ctx->relay_disqualify = FAULT_RELAY_DISQUALIFY_DEFAULT;
	return 0;
}

/* --------------------------------------------------------------- event queue */

static void evt_push(fault_ctx_t *ctx, fault_evt_type_t type, unsigned int sig,
		     fault_edge_t edge, uint32_t mono_ms)
{
	fault_evt_t *e;

	if (ctx->q_len >= (uint16_t)FAULT_EVT_QUEUE_LEN) {
		/* Drop-newest: the head of a cascade is the diagnosis. */
		ctx->q_dropped++;
		return;
	}

	e = &ctx->q[(ctx->q_head + ctx->q_len) % (uint16_t)FAULT_EVT_QUEUE_LEN];
	e->type = (uint8_t)type;
	e->id = (uint8_t)sig;
	e->edge = (uint8_t)edge;
	e->_pad = 0U;
	e->mono_ms = mono_ms;
	ctx->q_len++;
}

int fault_evt_get(fault_ctx_t *ctx, fault_evt_t *out)
{
	if ((ctx == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (ctx->q_len == 0U) {
		return -EAGAIN;
	}

	*out = ctx->q[ctx->q_head];
	ctx->q_head = (uint16_t)((ctx->q_head + 1U) % (uint16_t)FAULT_EVT_QUEUE_LEN);
	ctx->q_len--;
	return 0;
}

size_t fault_evt_count(const fault_ctx_t *ctx)
{
	return (ctx != NULL) ? (size_t)ctx->q_len : 0U;
}

uint32_t fault_evt_dropped(const fault_ctx_t *ctx)
{
	return (ctx != NULL) ? ctx->q_dropped : 0U;
}

void fault_evt_clear_dropped(fault_ctx_t *ctx)
{
	if (ctx != NULL) {
		ctx->q_dropped = 0U;
	}
}

/* -------------------------------------------------------------------- alarms */

static bool alarm_id_valid(fault_alarm_id_t id)
{
	return (unsigned int)id < (unsigned int)FAULT_ALARM_COUNT;
}

int fault_alarm_set(fault_ctx_t *ctx, fault_alarm_id_t id, bool active,
		    uint32_t mono_ms)
{
	fault_alarm_t *a;

	if ((ctx == NULL) || !alarm_id_valid(id)) {
		return -EINVAL;
	}

	a = &ctx->alarm[(unsigned int)id];

	if (active && !a->active) {
		/* Rising edge: count it and, on the first since the last clear,
		 * remember when the condition started. */
		a->count++;
		a->last_ms = mono_ms;
		if (!a->latched) {
			a->latched = true;
			a->first_ms = mono_ms;
		}
	}
	a->active = active;
	return 0;
}

const fault_alarm_t *fault_alarm_get(const fault_ctx_t *ctx, fault_alarm_id_t id)
{
	if ((ctx == NULL) || !alarm_id_valid(id)) {
		return NULL;
	}
	return &ctx->alarm[(unsigned int)id];
}

uint64_t fault_alarms(const fault_ctx_t *ctx)
{
	uint64_t m = 0U;
	unsigned int i;

	if (ctx == NULL) {
		return 0U;
	}
	for (i = 0U; i < (unsigned int)FAULT_ALARM_COUNT; i++) {
		if (ctx->alarm[i].active) {
			m |= FAULT_ALARM_BIT(i);
		}
	}
	return m;
}

uint64_t fault_alarms_latched(const fault_ctx_t *ctx)
{
	uint64_t m = 0U;
	unsigned int i;

	if (ctx == NULL) {
		return 0U;
	}
	for (i = 0U; i < (unsigned int)FAULT_ALARM_COUNT; i++) {
		if (ctx->alarm[i].latched) {
			m |= FAULT_ALARM_BIT(i);
		}
	}
	return m;
}

bool fault_any_active(const fault_ctx_t *ctx)
{
	return fault_alarms(ctx) != 0U;
}

int fault_alarm_clear(fault_ctx_t *ctx, fault_alarm_id_t id)
{
	fault_alarm_t *a;

	if ((ctx == NULL) || !alarm_id_valid(id)) {
		return -EINVAL;
	}

	a = &ctx->alarm[(unsigned int)id];
	if (!a->latched) {
		return -ENOENT;
	}
	if (a->active) {
		/* Refusing here is the safety property: acknowledging a live
		 * fault must not make the alarm page read clean. */
		return -EBUSY;
	}

	a->latched = false;
	a->count = 0U;
	a->first_ms = 0U;
	a->last_ms = 0U;
	return 0;
}

int fault_alarm_clear_all(fault_ctx_t *ctx)
{
	unsigned int i;

	if (ctx == NULL) {
		return -EINVAL;
	}
	for (i = 0U; i < (unsigned int)FAULT_ALARM_COUNT; i++) {
		(void)fault_alarm_clear(ctx, (fault_alarm_id_t)i);
	}
	return 0;
}

/* -------------------------------------------------------------- relay policy */

int fault_relay_set_disqualify_mask(fault_ctx_t *ctx, uint64_t mask)
{
	if (ctx == NULL) {
		return -EINVAL;
	}
	ctx->relay_disqualify = mask;
	return 0;
}

uint64_t fault_relay_disqualify_mask(const fault_ctx_t *ctx)
{
	return (ctx != NULL) ? ctx->relay_disqualify : 0U;
}

uint64_t fault_relay_blocking(const fault_ctx_t *ctx)
{
	if (ctx == NULL) {
		return 0U;
	}
	return fault_alarms(ctx) & ctx->relay_disqualify;
}

bool fault_relay_eligible(const fault_ctx_t *ctx)
{
	if (ctx == NULL) {
		return false; /* fail-safe: do not energize */
	}
	return fault_relay_blocking(ctx) == 0U;
}

/* ----------------------------------------------------------------- RGB policy */

fault_rgb_state_t fault_rgb_state(const fault_rgb_in_t *in)
{
	if (in == NULL) {
		return FAULT_RGB_OFF;
	}
	if (in->identify) {
		return FAULT_RGB_BLUE_PULSE;
	}
	if (in->any_fault) {
		return FAULT_RGB_RED;
	}
	if (in->holdover || in->warming) {
		return FAULT_RGB_AMBER;
	}
	if (in->locked) {
		return FAULT_RGB_GREEN;
	}
	return FAULT_RGB_OFF;
}

/* ----------------------------------------------------------------- scan core */

/* Emit the event(s) a committed transition produces, and update the alarm. */
static void dispatch(fault_ctx_t *ctx, unsigned int sig, bool asserted,
		     uint32_t mono_ms)
{
	fault_class_t cls = (fault_class_t)sig_class_tbl[sig];
	fault_edge_t edge = asserted ? FAULT_EDGE_ASSERT : FAULT_EDGE_DEASSERT;

	switch (cls) {
	case FAULT_CLASS_BUTTON:
		if (asserted) {
			ctx->press_ms[sig] = mono_ms;
			ctx->next_repeat_ms[sig] =
				mono_ms + ctx->cfg.repeat_delay_ms;
			ctx->long_sent[sig] = 0U;
		}
		evt_push(ctx, FAULT_EVT_BUTTON, sig, edge, mono_ms);
		break;

	case FAULT_CLASS_TOUCH:
		/* The FT-series INT is a request to read the controller. Its
		 * release says nothing, so only the assert is reported. */
		if (asserted) {
			evt_push(ctx, FAULT_EVT_TOUCH, sig, edge, mono_ms);
		}
		break;

	case FAULT_CLASS_PROX:
		evt_push(ctx, FAULT_EVT_PROX, sig, edge, mono_ms);
		break;

	case FAULT_CLASS_EN_FAULT:
		evt_push(ctx, FAULT_EVT_EN_FAULT, sig, edge, mono_ms);
		(void)fault_alarm_set(ctx, (fault_alarm_id_t)sig, asserted,
				      mono_ms);
		break;

	case FAULT_CLASS_PG:
		evt_push(ctx,
			 asserted ? FAULT_EVT_PG_FAULT : FAULT_EVT_PG_RECOVER,
			 sig, edge, mono_ms);
		(void)fault_alarm_set(ctx, (fault_alarm_id_t)sig, asserted,
				      mono_ms);
		break;

	case FAULT_CLASS_BKP_PG:
		evt_push(ctx, FAULT_EVT_BKP_PG, sig, edge, mono_ms);
		(void)fault_alarm_set(ctx, (fault_alarm_id_t)sig, asserted,
				      mono_ms);
		break;

	case FAULT_CLASS_INA_ALERT:
	default:
		evt_push(ctx, FAULT_EVT_INA_ALERT, sig, edge, mono_ms);
		(void)fault_alarm_set(ctx, (fault_alarm_id_t)sig, asserted,
				      mono_ms);
		break;
	}
}

/* Long-press and auto-repeat for every currently-held button. */
static void button_timers(fault_ctx_t *ctx, uint32_t mono_ms)
{
	unsigned int i;

	for (i = 0U; i < BUTTON_COUNT; i++) {
		unsigned int sig = button_sig_tbl[i];

		if ((ctx->stable & FAULT_SIG_BIT(sig)) == 0U) {
			continue;
		}

		if ((ctx->long_sent[sig] == 0U) &&
		    ((uint32_t)(mono_ms - ctx->press_ms[sig]) >=
		     ctx->cfg.long_press_ms)) {
			ctx->long_sent[sig] = 1U;
			evt_push(ctx, FAULT_EVT_BUTTON_LONG, sig,
				 FAULT_EDGE_ASSERT, mono_ms);
		}

		/*
		 * Absolute deadline, compared as a signed difference so it is
		 * wrap-safe. The next deadline is re-anchored to *now* plus one
		 * period, not advanced by one period from the old deadline:
		 * after a scan stall the deadline is far in the past, and
		 * advancing it by a single period would leave it still in the
		 * past, so the next thousand 1 kHz scans would each fire one
		 * repeat until the deadline caught up — a 4 Hz control turning
		 * into a 1 kHz burst the instant scanning resumes. Re-anchoring
		 * clamps the catch-up to a single repeat and restores the 4 Hz
		 * cadence immediately.
		 */
		if ((int32_t)(mono_ms - ctx->next_repeat_ms[sig]) >= 0) {
			ctx->next_repeat_ms[sig] =
				mono_ms + ctx->cfg.repeat_period_ms;
			evt_push(ctx, FAULT_EVT_BUTTON_REPEAT, sig,
				 FAULT_EDGE_ASSERT, mono_ms);
		}
	}
}

int fault_scan_input(fault_ctx_t *ctx, uint16_t f_idr, uint16_t g_idr,
		     uint32_t mono_ms)
{
	uint32_t raw;
	uint32_t asserted;
	uint32_t diff;
	unsigned int i;

	if (ctx == NULL) {
		return -EINVAL;
	}

	raw = ((uint32_t)g_idr << 16) | (uint32_t)f_idr;
	asserted = (raw ^ FAULT_ACTIVE_LOW_MASK) & FAULT_SIG_ALL_MASK;

	ctx->last_raw = raw;
	ctx->now_ms = mono_ms;
	ctx->scans++;
	ctx->started = true;

	diff = asserted ^ ctx->stable;

	/*
	 * Hot path: nothing differs from the committed state and no candidate
	 * is being timed, so there is nothing for the per-bit loop to do. This
	 * is the overwhelming majority of the thousand scans a second.
	 */
	if ((diff != 0U) || (ctx->pending != 0U)) {
		for (i = 0U; i < (unsigned int)FAULT_SIG_COUNT; i++) {
			uint32_t bit = FAULT_SIG_BIT(i);
			fault_class_t cls;
			uint32_t need_ms;
			uint32_t need_samples;

			if ((diff & bit) == 0U) {
				/* Back in agreement: abandon any candidate. */
				ctx->pending &= ~bit;
				continue;
			}

			if ((ctx->pending & bit) == 0U) {
				ctx->pending |= bit;
				ctx->cand_first_ms[i] = mono_ms;
				ctx->cand_samples[i] = 1U;
			} else if (ctx->cand_samples[i] < UINT16_MAX) {
				ctx->cand_samples[i]++;
			}

			cls = (fault_class_t)sig_class_tbl[i];
			need_ms = class_debounce_ms(ctx, cls);
			need_samples = class_debounce_samples(ctx, cls);

			if (((uint32_t)(mono_ms - ctx->cand_first_ms[i]) >=
			     need_ms) &&
			    ((uint32_t)ctx->cand_samples[i] >= need_samples)) {
				bool now = (asserted & bit) != 0U;

				ctx->stable ^= bit;
				ctx->pending &= ~bit;
				ctx->cand_samples[i] = 0U;
				dispatch(ctx, i, now, mono_ms);
			}
		}
	}

	button_timers(ctx, mono_ms);
	return 0;
}

uint32_t fault_state(const fault_ctx_t *ctx)
{
	return (ctx != NULL) ? ctx->stable : 0U;
}

bool fault_asserted(const fault_ctx_t *ctx, fault_sig_t sig)
{
	if ((ctx == NULL) ||
	    ((unsigned int)sig >= (unsigned int)FAULT_SIG_COUNT)) {
		return false;
	}
	return (ctx->stable & FAULT_SIG_BIT(sig)) != 0U;
}
