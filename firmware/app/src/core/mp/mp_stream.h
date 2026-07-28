/*
 * STS1000 "Meridian" — core/mp: CBOR writer + stream records (FMT §7, §8.1).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation; every buffer is caller-owned.
 *
 * The streams carry CBOR (RFC 8949) rather than JSON: a telemetry record at
 * 10 Hz is the device's highest-rate output and the JSON form of it is roughly
 * three times the bytes for no added clarity. Only definite-length items are
 * emitted, so a host decoder never has to look ahead for a break marker.
 *
 * Wire shape. Every record is one CBOR map with small integer keys — compact and
 * stable. Key 0 is always the record type and key 1 the record version, so a
 * host can dispatch on two bytes:
 *
 *   0  record type   (MP_REC_*)
 *   1  record version
 *   2  sequence      (per-channel, monotonic)
 *   3  device monotonic milliseconds
 *
 * Per-record keys start at 8. They are listed against each encoder below and
 * are append-only: a new field takes the next free number and never reuses one.
 *
 * All of these encoders are pure functions of their input struct. The glue fills
 * the struct from snapshots (never while holding a timing lock, spec §10) and
 * core turns it into bytes, which is what makes every record round-trippable in
 * a host test.
 */

#ifndef STS1000_CORE_MP_MP_STREAM_H_
#define STS1000_CORE_MP_MP_STREAM_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "logring/logring.h"
#include "mp/mp_frame.h"
#include "quality/quality.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================= CBOR writer === */

/** CBOR builder over a caller buffer, with a sticky error. */
typedef struct {
	uint8_t *buf;
	size_t cap;
	size_t len;
	int err; /**< sticky -ENOSPC */
} mp_cbor_t;

int mp_cbor_init(mp_cbor_t *w, uint8_t *buf, size_t cap);

int mp_cbor_uint(mp_cbor_t *w, uint64_t v);
int mp_cbor_int(mp_cbor_t *w, int64_t v);
int mp_cbor_bytes(mp_cbor_t *w, const uint8_t *b, size_t n);
int mp_cbor_textn(mp_cbor_t *w, const char *s, size_t n);
int mp_cbor_text(mp_cbor_t *w, const char *s);
int mp_cbor_arr(mp_cbor_t *w, size_t n);
int mp_cbor_map(mp_cbor_t *w, size_t n);
int mp_cbor_bool(mp_cbor_t *w, bool v);
int mp_cbor_null(mp_cbor_t *w);
/** IEEE-754 binary32, major type 7 additional info 26. */
int mp_cbor_f32(mp_cbor_t *w, float v);

/** Convenience: integer key + value. */
int mp_cbor_kv_uint(mp_cbor_t *w, uint64_t key, uint64_t v);
int mp_cbor_kv_int(mp_cbor_t *w, uint64_t key, int64_t v);
int mp_cbor_kv_bool(mp_cbor_t *w, uint64_t key, bool v);
int mp_cbor_kv_f32(mp_cbor_t *w, uint64_t key, float v);
int mp_cbor_kv_text(mp_cbor_t *w, uint64_t key, const char *s);
int mp_cbor_kv_bytes(mp_cbor_t *w, uint64_t key, const uint8_t *b, size_t n);

/**
 * Finish. @p out_len receives the encoded length.
 *
 * @retval 0        Complete.
 * @retval -EINVAL  @p w is NULL.
 * @retval -ENOSPC  The buffer overflowed; the contents are unusable.
 */
int mp_cbor_finish(const mp_cbor_t *w, size_t *out_len);

/* ------------------------------------------------------------ CBOR reader */

/*
 * A deliberately tiny reader. It exists so the host tests can round-trip every
 * record without a CBOR library, and so the glue can walk a tunnelled record if
 * it ever needs to. It handles exactly the subset the writer emits.
 */

typedef struct {
	const uint8_t *buf;
	size_t len;
	size_t pos;
} mp_cbor_rd_t;

int mp_cbor_rd_init(mp_cbor_rd_t *r, const uint8_t *buf, size_t len);

/** Major type of the next item, or -EBADMSG / -ENODATA. */
int mp_cbor_peek_major(const mp_cbor_rd_t *r);

/** Read an unsigned integer (major 0). */
int mp_cbor_rd_uint(mp_cbor_rd_t *r, uint64_t *out);
/** Read a signed integer (major 0 or 1). */
int mp_cbor_rd_int(mp_cbor_rd_t *r, int64_t *out);
/** Read a map header; @p out receives the pair count. */
int mp_cbor_rd_map(mp_cbor_rd_t *r, size_t *out);
/** Read an array header; @p out receives the element count. */
int mp_cbor_rd_arr(mp_cbor_rd_t *r, size_t *out);
/** Read a byte string; @p out points into the buffer. */
int mp_cbor_rd_bytes(mp_cbor_rd_t *r, const uint8_t **out, size_t *out_n);
/** Read a text string; @p out points into the buffer (not NUL-terminated). */
int mp_cbor_rd_text(mp_cbor_rd_t *r, const char **out, size_t *out_n);
int mp_cbor_rd_bool(mp_cbor_rd_t *r, bool *out);
int mp_cbor_rd_f32(mp_cbor_rd_t *r, float *out);
/** Skip one complete item, including a container and its contents. */
int mp_cbor_skip(mp_cbor_rd_t *r);

/**
 * Position the reader on the value of integer key @p key inside the map that
 * starts at the reader's current position.
 *
 * @retval 0        Positioned on the value.
 * @retval -ENOENT  No such key.
 * @retval <0       Malformed.
 */
int mp_cbor_map_find(mp_cbor_rd_t *r, uint64_t key);

/* ========================================================= record types == */

#define MP_REC_TELEM 1U
#define MP_REC_PPS 2U
#define MP_REC_LOG 3U
#define MP_REC_EVENT 4U
#define MP_REC_MIRROR 5U
#define MP_REC_BUNDLE 6U
#define MP_REC_RAW 7U /**< NMEA/UBX/tunnel tee: framed, not CBOR-wrapped */

/** Common keys. */
#define MP_K_TYPE 0U
#define MP_K_VER 1U
#define MP_K_SEQ 2U
#define MP_K_MONO 3U

/* ========================================================== telemetry ==== */

/** Rails, in ina228_rail_t order. */
#define MP_RAIL_COUNT 9U

typedef struct {
	int32_t bus_mv;
	int32_t current_ua;
	uint32_t power_uw;
	uint16_t diag; /**< INA228 DIAG_ALRT read-back */
	bool valid;    /**< read succeeded and SHUNT_CAL confirmed */
} mp_rail_t;

/** Environmental / power health, mirroring the glue's housekeeping cache. */
typedef struct {
	mp_rail_t rail[MP_RAIL_COUNT];

	int32_t tmp_osc_mc;
	bool tmp_osc_valid;
	int32_t tmp_amb_mc;
	bool tmp_amb_valid;
	int32_t die_mc;
	bool die_valid;
	int32_t humidity_mpct;
	bool humidity_valid;

	uint32_t fan_rpm;
	uint16_t fan_duty_pct;

	uint8_t poe_class;
	uint32_t poe_draw_mw;
	uint32_t poe_budget_mw;

	bool bkp_stm_pg;
	bool bkp_gps_pg;
} mp_health_t;

/**
 * One telemetry record's input (FMT §7.1).
 *
 * Keys, telemetry record version 1:
 *   8  tai_ns            9  time_fallback     10 uptime_s
 *   11 stratum           12 lock_state        13 active_ref
 *   14 gnss_fix          15 sv_used           16 sv_visible
 *   17 tacc_ns           18 last_pps_off_ns   19 pps_off_mean_ns (f32)
 *   20 pps_off_sigma_ns  21 freq_err_ppb      22 vc_cmd_mv
 *   23 vc_sense_mv       24 dac_code          25 adev [1s,10s,100s]
 *   26 root_delay_q16    27 root_disp_q16     28 holdover_elapsed_s
 *   29 holdover_est_err_ns  30 holdover_t_demote_s
 *   31 quality_flags     32 osc_temp_mc       33 leap [pending,current,at_tai_s]
 *   34 rails (array of [mv,ua,uw,diag,valid])
 *   35 tmp_osc_mc        36 tmp_amb_mc        37 die_mc
 *   38 humidity_mpct     39 valid_flags (bit0 osc,1 amb,2 die,3 rh)
 *   40 fan [rpm,duty]    41 poe [class,draw_mw,budget_mw]
 *   42 bkp [stm,gps]     43 alarms            44 alarms_latched
 *   45 scan_state        46 pwrseq [stage,shed,alarms]
 *   47 gnss [state,ant]  48 survey [dur_s,acc_mm]
 *   49 refs (bitmap: 0 rb_lock, 1 rb_powered, 2 extref_ok, 3 pfi)
 *   50 extref_hz         51 refsel_state
 *
 * Enumerations carried raw, so the host renders one set of names rather than
 * the device maintaining a second wire encoding of the same fact:
 *   46 stage   pwrseq_stage_t        46 shed    pwrseq_shed_level_t
 *   46 alarms  bit n = pwrseq_alarm_t n
 *   47 state   gnssmgr_state_t       47 ant     gnssmgr_ant_state_t
 *   51         refsel_state_t
 * Keys 12/13 (lock_state/active_ref) and 43/44 (alarms) keep core/quality's and
 * core/fault's own encodings for the same reason.
 */
typedef struct {
	uint32_t seq;
	uint64_t mono_ms;
	uint64_t tai_ns;
	bool time_fallback;
	uint32_t uptime_s;

	quality_block_t q;
	mp_health_t h;

	uint64_t alarms;
	uint64_t alarms_latched;
	uint32_t scan_state; /**< core/fault debounced asserted bitmap */

	uint8_t pwrseq_stage;
	uint8_t pwrseq_shed;
	uint32_t pwrseq_alarms;

	uint8_t gnss_state;
	uint8_t gnss_ant;
	uint32_t survey_dur_s;
	uint32_t survey_acc_mm;

	bool rb_lock;
	bool rb_powered;
	bool extref_ok;
	bool pfi;
	uint32_t extref_hz;
	uint8_t refsel_state;
} mp_telem_t;

/** Upper bound on a telemetry record; sized from the key list above. */
#define MP_TELEM_MAX 640U

/**
 * Encode a telemetry record.
 *
 * @retval >=0      Encoded length.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOSPC  @p cap too small.
 */
int mp_enc_telem(const mp_telem_t *t, uint8_t *buf, size_t cap);

/* ================================================================ PPS ==== */

/**
 * PPS / discipline record (FMT §7.2).
 *
 * Both capture channels are carried raw, then the residual before and after the
 * sawtooth correction, so a host can *see* the qErr contribution rather than
 * trust that it was applied.
 *
 * Keys, PPS record version 1:
 *   8  pa0_ns      9  pc6_ns       10 pc6_valid    11 qerr_ps
 *   12 residual_ns 13 residual_corrected_ns        14 cable_delay_ns
 *   15 vc_cmd_mv   16 vc_sense_mv  17 dac_code     18 loop_state
 *   19 active_ref  20 ref_flags    21 accepted     22 reject_reason
 *   23 interval_ns (PA0-to-PC6 skew)
 */
typedef struct {
	uint32_t seq;
	uint64_t mono_ms;

	int64_t pa0_ns; /**< TIM2_CH1 capture, receiver PPS */
	int64_t pc6_ns; /**< TIM3_CH1 capture, TIMEPULSE2 cross-check */
	bool pc6_valid;

	int32_t qerr_ps;            /**< UBX-TIM-TP sawtooth term */
	int32_t residual_ns;        /**< phase error before the qErr correction */
	int32_t residual_corr_ns;   /**< after qErr and cable delay */
	int32_t cable_delay_ns;
	int64_t interval_ns;        /**< PA0 - PC6 */

	int32_t vc_cmd_mv;
	int32_t vc_sense_mv;
	uint16_t dac_code;
	uint8_t loop_state;  /**< quality_lock_state_t */
	uint8_t active_ref;  /**< quality_ref_t */
	uint32_t ref_flags;  /**< QUALITY_FLAG_* */
	bool accepted;       /**< passed the median/MAD gate */
	uint8_t reject_reason;
} mp_pps_t;

/** Upper bound on a PPS record. */
#define MP_PPS_MAX 192U

int mp_enc_pps(const mp_pps_t *p, uint8_t *buf, size_t cap);

/* ================================================================ log ==== */

/**
 * Log record batch.
 *
 * Keys, log record version 1:
 *   8  cursor (the value to pass next)   9 gap (records missed)
 *   10 records: array of [seq, mono_ms, level, subsys, text]
 */
int mp_enc_log(uint32_t seq, uint64_t mono_ms, const logr_rec_t *recs,
	       uint16_t n, uint32_t next_cursor, uint32_t gap, uint8_t *buf,
	       size_t cap);

/* ============================================================== events === */

/** Event kinds on channel 0x09. */
typedef enum {
	MP_EV_FAULT = 0,   /**< core/fault event: PG drop, INA alert, EN fault */
	MP_EV_BUTTON,      /**< panel button / encoder switch */
	MP_EV_PROX,        /**< reed switch (door/magnet) */
	MP_EV_TOUCH,       /**< touch controller INT */
	MP_EV_ALARM,       /**< alarm asserted or cleared */
	MP_EV_OVERRIDE,    /**< mp_ovr_ev_t: grant/release/expire/deadman/veto */
	MP_EV_DIAG,        /**< diagnostic progress or result */
	MP_EV_MODE,        /**< MP mode entered/left, tunnel opened/closed */
	MP_EV_KIND_COUNT,
} mp_ev_kind_t;

/** Stable event-kind name. Never NULL. */
const char *mp_ev_kind_name(uint8_t kind);

/** Longest text an event carries. */
#define MP_EV_TEXT_MAX 32U

typedef struct {
	uint8_t kind;  /**< mp_ev_kind_t */
	uint8_t sub;   /**< kind-specific subtype */
	uint16_t id;   /**< signal, alarm id or manifest object index */
	uint8_t edge;  /**< 1 = assert/enter, 0 = deassert/leave */
	int32_t value;
	uint32_t mono_ms;
	char text[MP_EV_TEXT_MAX];
} mp_ev_t;

/** Event queue depth. Drop-newest, like core/fault and core/ui. */
#ifndef MP_EVQ_LEN
#define MP_EVQ_LEN 24U
#endif

/* ======================================================= subscriptions === */

/** One channel's subscription. */
typedef struct {
	bool on;
	uint8_t rate_hz; /**< 1..10 for paced channels; 0 = event-driven */
	uint32_t next_ms;
	uint32_t cursor; /**< log channel: logring cursor */
	uint32_t seq;    /**< per-channel record sequence */
	uint32_t sent;
	uint32_t dropped;
} mp_sub_t;

/** Lowest and highest subscription rate the device accepts (FMT §7). */
#define MP_RATE_MIN_HZ 1U
#define MP_RATE_MAX_HZ 10U

typedef struct {
	mp_sub_t sub[MP_CH_COUNT];

	mp_ev_t evq[MP_EVQ_LEN];
	uint16_t evq_head;
	uint16_t evq_len;
	uint32_t evq_dropped;
} mp_stream_ctx_t;

int mp_stream_init(mp_stream_ctx_t *c);

/** True when @p ch may be subscribed at all. */
bool mp_stream_subscribable(uint8_t ch);

/** True when @p ch is paced by a rate rather than by events. */
bool mp_stream_paced(uint8_t ch);

/**
 * Subscribe to @p ch.
 *
 * @param rate_hz  Required for a paced channel (1..10); ignored otherwise.
 *
 * @retval 0         Subscribed.
 * @retval -EINVAL   @p c is NULL.
 * @retval -ENOTSUP  @p ch is not subscribable.
 * @retval -ERANGE   @p rate_hz outside 1..10 on a paced channel.
 */
int mp_stream_sub(mp_stream_ctx_t *c, uint8_t ch, uint8_t rate_hz,
		  uint32_t now_ms);

/** Unsubscribe. -ENOENT when the channel was not subscribed. */
int mp_stream_unsub(mp_stream_ctx_t *c, uint8_t ch);

/** Drop every subscription (session close, link loss). */
void mp_stream_unsub_all(mp_stream_ctx_t *c);

/** True when @p ch is subscribed. */
bool mp_stream_is_sub(const mp_stream_ctx_t *c, uint8_t ch);

/**
 * True when a paced channel is due, advancing its schedule by one period.
 *
 * The schedule advances from the previous deadline, not from @p now_ms, so a
 * late poll does not slow the stream down; a poll that is late by more than one
 * period resynchronises to @p now_ms instead of bursting to catch up.
 */
bool mp_stream_due(mp_stream_ctx_t *c, uint8_t ch, uint32_t now_ms);

/** Next per-record sequence for @p ch, incrementing it. 0 when unsubscribed. */
uint32_t mp_stream_next_seq(mp_stream_ctx_t *c, uint8_t ch);

/** Log cursor for the log channel. */
uint32_t mp_stream_cursor(const mp_stream_ctx_t *c, uint8_t ch);
void mp_stream_set_cursor(mp_stream_ctx_t *c, uint8_t ch, uint32_t cursor);

/* ---------------------------------------------------------- event queue */

/**
 * Enqueue an event.
 *
 * @retval 0        Queued.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOSPC  Queue full; the *new* event is dropped and counted, so a
 *                  cascade keeps its root cause (the core/fault argument).
 */
int mp_stream_event(mp_stream_ctx_t *c, const mp_ev_t *ev);

/** Convenience: enqueue with a text reason. */
int mp_stream_eventf(mp_stream_ctx_t *c, uint8_t kind, uint8_t sub, uint16_t id,
		     uint8_t edge, int32_t value, uint32_t mono_ms,
		     const char *text);

size_t mp_stream_event_count(const mp_stream_ctx_t *c);
uint32_t mp_stream_event_dropped(const mp_stream_ctx_t *c);

/**
 * Account for @p n events that were lost *before* they reached this queue.
 *
 * The platform-side producers of MP_EV_FAULT/BUTTON/PROX/TOUCH/ALARM run on
 * threads that may not take the engine lock (a 1 kHz scan, the discipline loop),
 * so the glue stages them in a bounded queue of its own and drains it here. A
 * staging overflow is, from the host's point of view, exactly the same loss as
 * an overflow of this queue — and key 8 of the event record is the only field
 * that says a loss happened at all. Without this the host would see a gap
 * indistinguishable from a quiet board, which is the failure the whole channel
 * exists to prevent.
 *
 * Saturating rather than wrapping: the counter answers "were events lost", and
 * a wrap to a small number (or to zero) would answer it wrongly at exactly the
 * moment it matters most.
 *
 * @retval 0        Recorded.
 * @retval -EINVAL  @p c is NULL.
 */
int mp_stream_event_drop_note(mp_stream_ctx_t *c, uint32_t n);

/**
 * Drain up to @p max queued events into one CBOR record.
 *
 * Events are only removed from the queue once they are encoded, so a buffer too
 * small for the batch keeps the remainder for the next call.
 *
 * Keys, event record version 1:
 *   8  dropped   9 events: array of [kind, sub, id, edge, value, mono_ms, text]
 *
 * @retval >=0      Encoded length; 0 when the queue was empty.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOSPC  @p cap cannot hold even the record header.
 */
int mp_enc_events(mp_stream_ctx_t *c, uint32_t seq, uint64_t mono_ms,
		  uint16_t max, uint8_t *buf, size_t cap);

/* ============================================== §8.1 support bundle ====== */

/**
 * Everything the support bundle carries. Pointers may be NULL, in which case
 * that section is emitted as CBOR null — a bundle from a board whose NOR is
 * dead must still be a valid bundle.
 *
 * Keys, bundle record version 1:
 *   8  telemetry (the §7.1 record, nested)      9  versions [fw, boot, board]
 *   10 serial          11 fault_latched (u64)   12 fault_active (u64)
 *   13 i2c_scan (16-byte bitmap, LSB-first per address)
 *   14 pps_hist (array of u32 bins)             15 pps_hist_bin_ns
 *   16 log (the §log record, nested)            17 navsat (raw UBX-NAV-SAT)
 *   18 cfg (raw cfg TLV export)                 19 manifest_hash
 *   20 notes (free text)
 */
typedef struct {
	const mp_telem_t *telem;

	const char *fw_version;
	const char *boot_version;
	const char *board_id;
	const char *serial;

	uint64_t fault_latched;
	uint64_t fault_active;

	const uint8_t *i2c_scan; /**< 16 bytes, one bit per 7-bit address */

	const uint32_t *pps_hist;
	uint16_t pps_hist_n;
	uint32_t pps_hist_bin_ns;

	const logr_rec_t *log;
	uint16_t log_n;

	const uint8_t *navsat; /**< raw UBX-NAV-SAT frame */
	size_t navsat_n;

	const uint8_t *cfg_tlv; /**< cfg_export_all() output */
	size_t cfg_tlv_n;

	uint32_t manifest_hash;
	const char *notes;
} mp_bundle_t;

/**
 * Encode the §8.1 support bundle.
 *
 * The result is normally several kilobytes and is transmitted fragmented
 * (mp_frame_send()).
 *
 * @retval >=0      Encoded length.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOSPC  @p cap too small.
 */
int mp_enc_bundle(const mp_bundle_t *b, uint32_t seq, uint64_t mono_ms,
		  uint8_t *buf, size_t cap);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_MP_MP_STREAM_H_ */
