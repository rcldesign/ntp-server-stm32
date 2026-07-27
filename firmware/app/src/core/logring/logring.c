/*
 * STS1000 "Meridian" — core/logring: structured log ring + RFC 5424 renderer.
 *
 * See logring.h for the contract.
 */

#include "logring/logring.h"

#include <errno.h>
#include <string.h>

_Static_assert(LOGR_MSG_MAX <= 255U,
	       "logr_rec_t::len is a uint8_t; LOGR_MSG_MAX must fit in it");
_Static_assert(LOGR_SUB_COUNT <= 16U,
	       "logr_filter_t::sub_mask is 16 bits wide");

/* ------------------------------------------------------------------------- */
/* Lifecycle                                                                 */
/* ------------------------------------------------------------------------- */

void logr_reset(logr_t *l)
{
	if (l == NULL) {
		return;
	}
	l->count = 0U;
	l->head_seq = 0U;
	l->dropped = 0U;
	l->filtered = 0U;
}

int logr_init(logr_t *l, logr_rec_t *slots, uint16_t cap)
{
	size_t i;

	if ((l == NULL) || (slots == NULL) || (cap == 0U)) {
		return -EINVAL;
	}

	memset(l, 0, sizeof(*l));
	l->slots = slots;
	l->cap = cap;

	for (i = 0U; i < LOGR_SUB_COUNT; i++) {
		l->min_level[i] = (uint8_t)LOGR_INFO;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/* Producer                                                                  */
/* ------------------------------------------------------------------------- */

int logr_put(logr_t *l, uint8_t level, uint8_t subsys, uint64_t mono_ms,
	     const char *msg, size_t len)
{
	logr_rec_t *r;

	if ((l == NULL) || (l->slots == NULL) || (l->cap == 0U)) {
		return -EINVAL;
	}
	if ((level >= LOGR_LEVEL_COUNT) || (subsys >= LOGR_SUB_COUNT)) {
		return -EINVAL;
	}
	if ((msg == NULL) && (len != 0U)) {
		return -EINVAL;
	}
	if (level > l->min_level[subsys]) {
		l->filtered++;
		return 1;
	}
	if (len > LOGR_MSG_MAX) {
		len = LOGR_MSG_MAX;
	}

	r = &l->slots[l->head_seq % l->cap];
	r->seq = l->head_seq;
	r->mono_ms = mono_ms;
	r->level = level;
	r->subsys = subsys;
	r->len = (uint8_t)len;
	if (len != 0U) {
		memcpy(r->msg, msg, len);
	}
	/* Clear the tail so a shorter message cannot expose the remains of the
	 * longer one it replaced — readers copy the whole fixed-size record. */
	memset(&r->msg[len], 0, LOGR_MSG_MAX - len);

	if (l->count == l->cap) {
		l->dropped++;
	} else {
		l->count++;
	}
	l->head_seq++;
	return 0;
}

int logr_puts(logr_t *l, uint8_t level, uint8_t subsys, uint64_t mono_ms,
	      const char *msg)
{
	return logr_put(l, level, subsys, mono_ms, msg,
			(msg != NULL) ? strlen(msg) : 0U);
}

/* ------------------------------------------------------------------------- */
/* Levels                                                                    */
/* ------------------------------------------------------------------------- */

int logr_level_set(logr_t *l, uint8_t subsys, uint8_t level)
{
	size_t i;

	if ((l == NULL) || (level >= LOGR_LEVEL_COUNT)) {
		return -EINVAL;
	}
	if (subsys == 0xFFU) {
		for (i = 0U; i < LOGR_SUB_COUNT; i++) {
			l->min_level[i] = level;
		}
		return 0;
	}
	if (subsys >= LOGR_SUB_COUNT) {
		return -EINVAL;
	}
	l->min_level[subsys] = level;
	return 0;
}

int logr_level_get(const logr_t *l, uint8_t subsys)
{
	if ((l == NULL) || (subsys >= LOGR_SUB_COUNT)) {
		return -EINVAL;
	}
	return (int)l->min_level[subsys];
}

/* ------------------------------------------------------------------------- */
/* Reader                                                                    */
/* ------------------------------------------------------------------------- */

void logr_filter_all(logr_filter_t *f)
{
	if (f == NULL) {
		return;
	}
	f->max_level = (uint8_t)LOGR_DEBUG;
	f->sub_mask = 0U;
}

uint32_t logr_head(const logr_t *l)
{
	return (l != NULL) ? l->head_seq : 0U;
}

uint32_t logr_oldest(const logr_t *l)
{
	return (l != NULL) ? (l->head_seq - l->count) : 0U;
}

uint16_t logr_count(const logr_t *l)
{
	return (l != NULL) ? l->count : 0U;
}

uint32_t logr_dropped(const logr_t *l)
{
	return (l != NULL) ? l->dropped : 0U;
}

uint32_t logr_filtered(const logr_t *l)
{
	return (l != NULL) ? l->filtered : 0U;
}

static bool rec_passes(const logr_rec_t *r, const logr_filter_t *f)
{
	if (f == NULL) {
		return true;
	}
	if (r->level > f->max_level) {
		return false;
	}
	if (f->sub_mask == 0U) {
		return true;
	}
	return (f->sub_mask & (uint16_t)(1U << r->subsys)) != 0U;
}

int logr_tail(const logr_t *l, uint32_t cursor, const logr_filter_t *f,
	      logr_rec_t *out, uint16_t max, uint16_t *out_n,
	      uint32_t *out_next, uint32_t *out_gap)
{
	uint32_t oldest;
	uint32_t s;
	uint32_t gap = 0U;
	uint16_t n = 0U;

	if ((l == NULL) || (l->slots == NULL) || (out_n == NULL) ||
	    (out_next == NULL) || (out_gap == NULL)) {
		return -EINVAL;
	}
	if ((out == NULL) && (max != 0U)) {
		return -EINVAL;
	}

	oldest = l->head_seq - l->count;
	s = cursor;

	/* Signed differences so the comparisons stay correct across a 2^32
	 * sequence wrap. */
	if ((int32_t)(s - oldest) < 0) {
		gap = oldest - s;
		s = oldest;
	} else if ((int32_t)(s - l->head_seq) > 0) {
		/* Cursor from the future — a reader that outlived the ring. */
		s = l->head_seq;
	}

	while (s != l->head_seq) {
		const logr_rec_t *r = &l->slots[s % l->cap];

		if (n >= max) {
			break;
		}
		s++;
		if (rec_passes(r, f)) {
			out[n] = *r;
			n++;
		}
	}

	*out_n = n;
	*out_next = s;
	*out_gap = gap;
	return 0;
}

/* ------------------------------------------------------------------------- */
/* Names                                                                     */
/* ------------------------------------------------------------------------- */

static const char *const sub_names[LOGR_SUB_COUNT] = {
	"SYS", "TIMING", "GNSS", "NET", "NTP", "NTS", "PTP",
	"PWR", "THERM", "UI", "SEC", "MCP", "SNMP", "FAULT",
};

static const char *const level_names[LOGR_LEVEL_COUNT] = {
	"emerg", "alert", "crit", "err", "warning", "notice", "info", "debug",
};

const char *logr_sub_name(uint8_t subsys)
{
	return (subsys < LOGR_SUB_COUNT) ? sub_names[subsys] : "UNKNOWN";
}

const char *logr_level_name(uint8_t level)
{
	return (level < LOGR_LEVEL_COUNT) ? level_names[level] : "unknown";
}

/* ------------------------------------------------------------------------- */
/* RFC 5424 rendering                                                        */
/* ------------------------------------------------------------------------- */

/* Bounded output cursor. Overflow is latched, not trapped: core code must not
 * kill the host test runner, and a truncated syslog line is a reportable
 * condition rather than a crash. */
typedef struct {
	char  *buf;
	size_t cap;
	size_t n;
	bool   ovf;
} emit_t;

static void emit_ch(emit_t *e, char ch)
{
	if ((e->n + 1U) >= e->cap) {
		e->ovf = true;
		return;
	}
	e->buf[e->n] = ch;
	e->n++;
}

/* Emit at most @p limit characters of @p s; NULL/empty renders as the RFC 5424
 * NILVALUE "-". Characters outside the printable US-ASCII range PRINTUSASCII
 * (§6) are replaced so a corrupt hostname cannot inject a field separator. */
static void emit_field(emit_t *e, const char *s, size_t limit)
{
	size_t i;

	if ((s == NULL) || (s[0] == '\0')) {
		emit_ch(e, '-');
		return;
	}
	for (i = 0U; (i < limit) && (s[i] != '\0'); i++) {
		unsigned char uc = (unsigned char)s[i];

		if ((uc < (unsigned char)'!') || (uc > (unsigned char)'~')) {
			uc = (unsigned char)'_';
		}
		emit_ch(e, (char)uc);
	}
}

/* Emit @p v in decimal, zero-padded to @p width (0 = natural width). */
static void emit_u32(emit_t *e, uint32_t v, unsigned int width)
{
	char tmp[10];
	unsigned int n = 0U;

	do {
		tmp[n] = (char)('0' + (char)(v % 10U));
		n++;
		v /= 10U;
	} while ((v != 0U) && (n < sizeof(tmp)));

	while (n < width) {
		emit_ch(e, '0');
		width--;
	}
	while (n > 0U) {
		n--;
		emit_ch(e, tmp[n]);
	}
}

/*
 * Days since 1970-01-01 → proleptic Gregorian civil date.
 *
 * Howard Hinnant, "chrono-Compatible Low-Level Date Algorithms"
 * (howardhinnant.github.io/date_algorithms.html), placed in the public domain
 * by its author. Reproduced here rather than calling gmtime() so the renderer
 * stays free of libc locale/timezone state and is bit-identical on host and
 * target.
 */
static void civil_from_days(uint32_t z, uint32_t *y, uint32_t *m, uint32_t *d)
{
	uint32_t era;
	uint32_t doe;
	uint32_t yoe;
	uint32_t doy;
	uint32_t mp;

	z += 719468U;            /* shift the epoch to 0000-03-01 */
	era = z / 146097U;       /* 400-year era */
	doe = z - (era * 146097U);
	yoe = (doe - (doe / 1460U) + (doe / 36524U) - (doe / 146096U)) / 365U;
	doy = doe - ((365U * yoe) + (yoe / 4U) - (yoe / 100U));
	mp = ((5U * doy) + 2U) / 153U;

	*d = doy - (((153U * mp) + 2U) / 5U) + 1U;
	*m = (mp < 10U) ? (mp + 3U) : (mp - 9U);
	*y = yoe + (era * 400U) + ((*m <= 2U) ? 1U : 0U);
}

/* 9999-12-31T23:59:59Z — the largest instant RFC 3339's 4-digit year holds. */
#define LOGR_UNIX_MAX 253402300799ULL

int logr_render_5424(const logr_rec_t *r, uint8_t facility,
		     const char *hostname, const char *appname,
		     uint64_t unix_s, uint32_t frac_us, char *out, size_t cap)
{
	emit_t e;
	uint32_t days;
	uint32_t sod;
	uint32_t y;
	uint32_t mo;
	uint32_t d;
	size_t i;

	if ((r == NULL) || (out == NULL) || (cap == 0U)) {
		return -EINVAL;
	}
	if ((facility > 23U) || (frac_us > 999999U) ||
	    (unix_s > LOGR_UNIX_MAX)) {
		return -EINVAL;
	}
	if (r->len > LOGR_MSG_MAX) {
		return -EINVAL;
	}

	e.buf = out;
	e.cap = cap;
	e.n = 0U;
	e.ovf = false;

	/* PRI */
	emit_ch(&e, '<');
	emit_u32(&e, ((uint32_t)facility * 8U) +
			     (uint32_t)(r->level & 0x07U), 0U);
	emit_ch(&e, '>');

	/* VERSION */
	emit_ch(&e, '1');
	emit_ch(&e, ' ');

	/* TIMESTAMP */
	days = (uint32_t)(unix_s / 86400ULL);
	sod = (uint32_t)(unix_s % 86400ULL);
	civil_from_days(days, &y, &mo, &d);

	emit_u32(&e, y, 4U);
	emit_ch(&e, '-');
	emit_u32(&e, mo, 2U);
	emit_ch(&e, '-');
	emit_u32(&e, d, 2U);
	emit_ch(&e, 'T');
	emit_u32(&e, sod / 3600U, 2U);
	emit_ch(&e, ':');
	emit_u32(&e, (sod / 60U) % 60U, 2U);
	emit_ch(&e, ':');
	emit_u32(&e, sod % 60U, 2U);
	emit_ch(&e, '.');
	emit_u32(&e, frac_us, 6U);
	emit_ch(&e, 'Z');
	emit_ch(&e, ' ');

	/* HOSTNAME APP-NAME PROCID MSGID */
	emit_field(&e, hostname, LOGR_HOSTNAME_MAX);
	emit_ch(&e, ' ');
	emit_field(&e, appname, LOGR_APPNAME_MAX);
	emit_ch(&e, ' ');
	emit_ch(&e, '-');
	emit_ch(&e, ' ');
	emit_field(&e, logr_sub_name(r->subsys), 16U);
	emit_ch(&e, ' ');

	/* STRUCTURED-DATA */
	emit_ch(&e, '-');
	emit_ch(&e, ' ');

	/* MSG. Control characters would break line framing at the collector. */
	for (i = 0U; i < r->len; i++) {
		unsigned char uc = (unsigned char)r->msg[i];

		if (uc < 0x20U) {
			uc = (unsigned char)' ';
		}
		emit_ch(&e, (char)uc);
	}

	out[(e.n < cap) ? e.n : (cap - 1U)] = '\0';
	return e.ovf ? -ENOSPC : (int)e.n;
}
