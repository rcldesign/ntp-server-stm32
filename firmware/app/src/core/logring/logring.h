/*
 * STS1000 "Meridian" — core/logring: structured log ring + RFC 5424 renderer.
 *
 * Platform-neutral C11. No dynamic allocation: the caller owns the record
 * array, so the ring can live in the logger thread's BSS on target and on the
 * stack in a unit test.
 *
 * Spec §5.4: structured events with a level and a subsystem tag, a fast RAM
 * ring feeding three consumers (MCP `LOG_TAIL`, the NOR spool, remote syslog),
 * and a renderer that turns a record into an RFC 5424 line. Only the ring and
 * the renderer live here — the NOR spool, the syslog socket and the audit log
 * are glue.
 *
 * Producer contract
 * -----------------
 * logr_put() never blocks and never fails for lack of space: when the ring is
 * full the oldest record is overwritten and logr_dropped() counts it. Losing
 * the oldest debug line is always better than stalling the thread that is
 * trying to report a fault.
 *
 * Reader contract
 * ---------------
 * Readers are cursors, not consumers: a record stays until it is overwritten,
 * so several readers (MCP tail, spool, syslog) advance independently. A cursor
 * is a sequence number; sequence numbers never repeat and never reset short of
 * a 2^32 wrap. A reader whose cursor has fallen behind the oldest retained
 * record is told exactly how many records it missed rather than silently
 * resuming — that gap count is what makes a truncated log auditable.
 *
 * Concurrency: a logr_t is not internally locked. On target the producer side
 * is entered under the logger thread's lock (or from a single ISR-free
 * context); readers run in the same thread. Nothing here spins or blocks.
 */

#ifndef STS1000_CORE_LOGRING_LOGRING_H_
#define STS1000_CORE_LOGRING_LOGRING_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Longest message body a record holds, in bytes. Longer input is truncated. */
#ifndef LOGR_MSG_MAX
#define LOGR_MSG_MAX 96U
#endif

/** Severity, numbered as in RFC 5424 §6.2.1 (lower is more severe). */
typedef enum {
	LOGR_EMERG  = 0,
	LOGR_ALERT  = 1,
	LOGR_CRIT   = 2,
	LOGR_ERR    = 3,
	LOGR_WARN   = 4,
	LOGR_NOTICE = 5,
	LOGR_INFO   = 6,
	LOGR_DEBUG  = 7,
} logr_level_t;

/** Number of distinct severities. */
#define LOGR_LEVEL_COUNT 8U

/**
 * Subsystem tag. Rendered as the RFC 5424 MSGID and used as the filter axis in
 * MCP `LOG_TAIL` / `LOG_LEVEL`. Values are wire-visible; append, never reorder.
 */
typedef enum {
	LOGR_SUB_SYS    = 0,  /* boot, watchdog, generic system */
	LOGR_SUB_TIMING = 1,  /* discipline loop, holdover, reference select */
	LOGR_SUB_GNSS   = 2,
	LOGR_SUB_NET    = 3,
	LOGR_SUB_NTP    = 4,
	LOGR_SUB_NTS    = 5,
	LOGR_SUB_PTP    = 6,
	LOGR_SUB_PWR    = 7,  /* rails, PoE, supercaps, Rb sequence */
	LOGR_SUB_THERM  = 8,
	LOGR_SUB_UI     = 9,
	LOGR_SUB_SEC    = 10, /* auth, keys, audit-adjacent events */
	LOGR_SUB_MCP    = 11, /* the console protocol itself */
	LOGR_SUB_SNMP   = 12,
	LOGR_SUB_FAULT  = 13, /* fault aggregator / alarm state */
	LOGR_SUB_COUNT  = 14,
} logr_sub_t;

/** One log record. Fixed size; the ring is an array of these. */
typedef struct {
	uint32_t seq;      /* monotonic, assigned by logr_put() */
	uint64_t mono_ms;  /* port_clock_t::mono_ms at the call site */
	uint8_t  level;    /* logr_level_t */
	uint8_t  subsys;   /* logr_sub_t */
	uint8_t  len;      /* bytes used in msg */
	char     msg[LOGR_MSG_MAX];
} logr_rec_t;

/** Reader filter. A zeroed filter passes nothing; use logr_filter_all(). */
typedef struct {
	uint8_t  max_level; /* records with level <= max_level pass */
	uint16_t sub_mask;  /* bit per logr_sub_t; 0 means "every subsystem" */
} logr_filter_t;

/** Ring state. Caller-owned; populated by logr_init(). */
typedef struct {
	logr_rec_t *slots;
	uint16_t    cap;
	uint16_t    count;    /* records currently retained (<= cap) */
	uint32_t    head_seq; /* sequence number the next record will take */
	uint32_t    dropped;  /* records overwritten before every reader saw them */
	uint32_t    filtered; /* records rejected by the per-subsystem level */
	uint8_t     min_level[LOGR_SUB_COUNT];
} logr_t;

/* -------------------------------------------------------------- lifecycle */

/**
 * Bind a ring to its backing array and empty it. Every subsystem starts at
 * @p LOGR_INFO; raise or lower with logr_level_set().
 *
 * @param slots  Array of @p cap records, caller-owned.
 * @param cap    Number of records; must be non-zero. No power-of-two
 *               requirement — the ring indexes by modulo so an operator can
 *               size it to the RAM actually available.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p l or @p slots is NULL, or @p cap is 0.
 */
int logr_init(logr_t *l, logr_rec_t *slots, uint16_t cap);

/** Drop every retained record and reset the counters. Cursors are invalidated. */
void logr_reset(logr_t *l);

/* --------------------------------------------------------------- producer */

/**
 * Append a record.
 *
 * @param level    logr_level_t.
 * @param subsys   logr_sub_t.
 * @param mono_ms  Monotonic timestamp from the caller's clock port.
 * @param msg      Message bytes; truncated at LOGR_MSG_MAX. May be NULL only
 *                 when @p len is 0.
 * @param len      Message length in bytes.
 *
 * @retval 0        Stored.
 * @retval 1        Rejected by the subsystem's level filter. Not an error.
 * @retval -EINVAL  Bad argument (uninitialised ring, bad level or subsystem).
 */
int logr_put(logr_t *l, uint8_t level, uint8_t subsys, uint64_t mono_ms,
	     const char *msg, size_t len);

/** logr_put() for a NUL-terminated string. */
int logr_puts(logr_t *l, uint8_t level, uint8_t subsys, uint64_t mono_ms,
	      const char *msg);

/* ----------------------------------------------------------------- levels */

/**
 * Set the minimum severity a subsystem records.
 *
 * @param subsys  A logr_sub_t, or 0xFF for "every subsystem".
 * @param level   Records with `level <= this` are kept.
 *
 * @retval 0        Applied.
 * @retval -EINVAL  Bad argument.
 */
int logr_level_set(logr_t *l, uint8_t subsys, uint8_t level);

/** Current threshold for @p subsys, or -EINVAL. */
int logr_level_get(const logr_t *l, uint8_t subsys);

/* ----------------------------------------------------------------- reader */

/** A filter that passes every retained record. */
void logr_filter_all(logr_filter_t *f);

/** Sequence number the next record will take (one past the newest). */
uint32_t logr_head(const logr_t *l);

/** Sequence number of the oldest retained record (== logr_head() if empty). */
uint32_t logr_oldest(const logr_t *l);

/** Records currently retained. */
uint16_t logr_count(const logr_t *l);

/** Records overwritten since logr_init()/logr_reset(). */
uint32_t logr_dropped(const logr_t *l);

/** Records rejected by a level filter since logr_init()/logr_reset(). */
uint32_t logr_filtered(const logr_t *l);

/**
 * Copy retained records from @p cursor forward.
 *
 * Records that fail @p f are skipped but still advance the cursor, so a reader
 * that filters aggressively converges instead of re-scanning. A cursor ahead of
 * the head (a reader that restarted against a rebooted ring) is clamped to the
 * head and reports no gap.
 *
 * @param cursor    Sequence number to resume from; 0 means "from the start".
 * @param f         Filter; NULL means "everything".
 * @param out       Destination array of at least @p max records.
 * @param max       Capacity of @p out in records; 0 is legal and just probes
 *                  the cursor.
 * @param out_n     Receives the number of records copied.
 * @param out_next  Receives the cursor to pass to the next call.
 * @param out_gap   Receives the number of records that were overwritten before
 *                  this reader reached them (0 when the reader kept up).
 *
 * @retval 0        Success.
 * @retval -EINVAL  Bad argument.
 */
int logr_tail(const logr_t *l, uint32_t cursor, const logr_filter_t *f,
	      logr_rec_t *out, uint16_t max, uint16_t *out_n,
	      uint32_t *out_next, uint32_t *out_gap);

/* --------------------------------------------------------------- rendering */

/** Human-readable subsystem tag, e.g. "TIMING". "UNKNOWN" out of range. */
const char *logr_sub_name(uint8_t subsys);

/** Human-readable severity, e.g. "warning". "unknown" out of range. */
const char *logr_level_name(uint8_t level);

/** Default syslog facility for this device: local0. */
#define LOGR_FACILITY_LOCAL0 16U

/**
 * Render @p r as one RFC 5424 line (no trailing newline, NUL-terminated).
 *
 *   <PRI>1 TIMESTAMP HOSTNAME APP-NAME PROCID MSGID SD MSG
 *
 * PRI is `facility * 8 + level`. TIMESTAMP is RFC 3339 with microseconds in
 * UTC, built from the wall clock the caller supplies — the ring itself only
 * knows monotonic time, and inventing a wall clock inside core would make the
 * output untestable. PROCID and STRUCTURED-DATA are the nil value "-";
 * structured data belongs to the audit log, which is a separate stream.
 *
 * @param facility  Syslog facility, 0..23.
 * @param hostname  HOSTNAME field; NULL or "" renders as "-".
 * @param appname   APP-NAME field; NULL or "" renders as "-".
 * @param unix_s    Seconds since 1970-01-01 UTC for this record.
 * @param frac_us   Microseconds within that second, 0..999999.
 * @param out       Destination.
 * @param cap       Capacity of @p out, including room for the NUL.
 *
 * @retval >=0      Characters written, excluding the NUL.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOSPC  @p cap too small; @p out is left NUL-terminated if it can
 *                  hold at least one byte.
 */
int logr_render_5424(const logr_rec_t *r, uint8_t facility,
		     const char *hostname, const char *appname,
		     uint64_t unix_s, uint32_t frac_us, char *out, size_t cap);

/** Longest HOSTNAME logr_render_5424() emits; longer input is truncated. */
#define LOGR_HOSTNAME_MAX 64U
/** Longest APP-NAME logr_render_5424() emits; longer input is truncated. */
#define LOGR_APPNAME_MAX 32U

/** Upper bound on logr_render_5424() output, including the NUL. */
#define LOGR_RENDER_MAX (160U + LOGR_MSG_MAX)

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_LOGRING_LOGRING_H_ */
