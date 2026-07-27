/*
 * STS1000 "Meridian" — core/ptp: datasets, quality mapping and the port engine.
 *
 * Section references are to IEEE 1588-2019; the behavioural requirements come
 * from ntp_server_software_spec.md §4.4 with the quality inputs from §3.8.
 *
 * The port state machine occupies five of the nine states in Table 20:
 *
 *     INITIALIZING --enable--> LISTENING --+-- announce receipt timeout --> MASTER
 *                                          |
 *                                          +-- better foreign master ------> PASSIVE
 *
 *     MASTER  <-> PASSIVE      by BMCA, re-elected whenever the foreign-master
 *                              table changes or a record expires
 *     any     --> FAULTY       ptp_port_fault();  FAULTY --> LISTENING on
 *                              ptp_port_fault_reset()
 *
 * PRE_MASTER and its qualificationTimeout are absent on purpose. §9.2.6.11 sets
 * that timeout to zero for recommended states M1 and M2, and M3 — the only case
 * with a non-zero timeout — cannot arise on a single-port ordinary clock,
 * because Ebest is then always Erbest. UNCALIBRATED and SLAVE are absent
 * because this is a grandmaster appliance; see the scope note in ptp.h.
 */

#include "ptp/ptp.h"

#include <errno.h>
#include <string.h>

_Static_assert(PTP_MSG_MAX_LEN >= PTP_ANNOUNCE_LEN,
	       "transmit scratch buffer cannot hold an Announce");

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* 18 * log2(10) in Q16.16, the decimal scaling of ptp_quality_view_t.adev_tau1_e18. */
#define LOG2_1E18_Q16 3918706

/* ------------------------------------------------------------------- cfg -- */

void ptp_cfg_defaults(ptp_cfg_t *cfg)
{
	if (cfg == NULL) {
		return;
	}

	memset(cfg, 0, sizeof(*cfg));
	cfg->domain = 0U;
	cfg->major_sdo_id = 0U;
	cfg->minor_sdo_id = 0U;
	cfg->priority1 = 128U;
	cfg->priority2 = 128U;
	cfg->port_number = 1U;
	cfg->log_announce_interval = 1;  /* 2 s */
	cfg->log_sync_interval = 0;      /* 1 s */
	cfg->log_min_delay_req_interval = 0;
	cfg->announce_receipt_timeout = 3U;
	cfg->transport = PTP_TRANSPORT_UDP_IPV4;
	cfg->profile = PTP_PROFILE_DEFAULT;
	cfg->degradation = PTP_DEGRADE_ALT_A;
	cfg->oslv_locked = PTP_OSLV_DEFAULT_LOCKED;
}

static bool log_interval_ok(int8_t v)
{
	return (v >= PTP_LOG_INTERVAL_MIN) && (v <= PTP_LOG_INTERVAL_MAX);
}

int ptp_cfg_validate(const ptp_cfg_t *cfg)
{
	if (cfg == NULL) {
		return -EINVAL;
	}
	if (!log_interval_ok(cfg->log_announce_interval) ||
	    !log_interval_ok(cfg->log_sync_interval) ||
	    !log_interval_ok(cfg->log_min_delay_req_interval)) {
		return -EINVAL;
	}
	/* §7.7.3.1: the announce receipt timeout is N intervals with N >= 2. */
	if (cfg->announce_receipt_timeout < 2U) {
		return -EINVAL;
	}
	/* portNumber 0 is reserved (§7.5.2.3). */
	if (cfg->port_number == 0U) {
		return -EINVAL;
	}
	/*
	 * majorSdoId is the high nibble of octet 0 (§13.3.2.2). A wider value
	 * would be silently truncated by the encoder, putting this clock in a
	 * different SDO from the one that was configured — and it would then
	 * discard the peers it was meant to hear.
	 */
	if (cfg->major_sdo_id > 0x0FU) {
		return -EINVAL;
	}
	/* Unsigned compares: the enum's own range makes a "< 0" test provably
	 * dead (-Wtype-limits), while an out-of-range value cast into the type
	 * still fails the upper bound. */
	if ((unsigned int)cfg->transport >= (unsigned int)PTP_TRANSPORT_COUNT) {
		return -EINVAL;
	}
	if ((unsigned int)cfg->profile >= (unsigned int)PTP_PROFILE_COUNT) {
		return -EINVAL;
	}
	if ((unsigned int)cfg->degradation >= (unsigned int)PTP_DEGRADE_COUNT) {
		return -EINVAL;
	}
	return 0;
}

/* ------------------------------------------------------- quality mapping -- */

/*
 * Base-2 logarithm of a positive integer in Q16.16, without libm.
 *
 * The integer part is the position of the leading set bit; the fraction comes
 * from the classic repeated-squaring extraction — square the mantissa, and each
 * time it crosses 2.0 shift it back and record a one bit. Sixteen rounds give
 * the sixteen fractional bits. Returns 0 for v == 0, which callers filter out.
 */
static uint32_t log2_q16(uint64_t v)
{
	uint32_t whole = 0U;
	uint32_t mant;  /* Q30, normalised into [1.0, 2.0) */
	uint32_t frac = 0U;
	uint64_t x = v;
	unsigned int i;

	if (v == 0U) {
		return 0U;
	}

	while (x >= 2U) {
		x >>= 1;
		whole++;
	}

	if (whole >= 30U) {
		mant = (uint32_t)(v >> (whole - 30U));
	} else {
		mant = (uint32_t)(v << (30U - whole));
	}

	for (i = 0U; i < 16U; i++) {
		uint64_t sq = (uint64_t)mant * (uint64_t)mant; /* Q60, < 4.0 */

		mant = (uint32_t)(sq >> 30);                   /* Q30, [1.0, 4.0) */
		frac <<= 1;
		if (mant >= (1U << 31)) {                      /* crossed 2.0 */
			mant >>= 1;
			frac |= 1U;
		}
	}

	return (whole << 16) | frac;
}

/*
 * offsetScaledLogVariance = 256 * log2(PTPVAR) + 0x8000, with the offset
 * variance estimated as PTPVAR = (ADEV(1 s) * tau)^2 in s². Working in log
 * space that is
 *
 *   512 * ( log2(adev_e18) + log2(tau) - 18*log2(10) ) + 0x8000
 *
 * and log2(tau) is exactly the sync interval exponent, so no second logarithm
 * is needed. See the ptp_clock_quality_from_view() docs for why this estimator
 * is the conservative one.
 */
static uint16_t oslv_from_adev(uint64_t adev_e18, int8_t log_sync_interval)
{
	int64_t lg;
	int64_t val;

	if (adev_e18 == 0U) {
		return PTP_OSLV_UNKNOWN;
	}

	lg = (int64_t)log2_q16(adev_e18);
	lg += (int64_t)log_sync_interval * 65536;
	lg -= (int64_t)LOG2_1E18_Q16;

	val = (512 * lg) + ((int64_t)0x8000 * 65536);
	val = (val + 32768) / 65536;

	if (val < 0) {
		/* Reachable: a picosecond-class ADEV at the shortest interval. */
		return 0U;
	}
	if (val >= (int64_t)PTP_OSLV_UNKNOWN) {
		/*
		 * Not reachable with the current scaling — a uint64 of 1e-18
		 * units at tau <= 2^7 s tops out near 38502 — but a computed
		 * value must never collide with the reserved 0xFFFF "not
		 * computed" code, so the guard stays for whoever rescales this.
		 */
		return (uint16_t)(PTP_OSLV_UNKNOWN - 1U);
	}
	return (uint16_t)val;
}

static uint8_t accuracy_from_ns(uint64_t est_ns)
{
	static const struct {
		uint64_t max_ns;
		uint8_t code;
	} ladder[] = {
		{            1ULL, 0x1DU }, /* 1 ns    */
		{            2ULL, 0x1EU }, /* 2.5 ns  */
		{           10ULL, 0x1FU }, /* 10 ns   */
		{           25ULL, 0x20U }, /* 25 ns   */
		{          100ULL, 0x21U }, /* 100 ns  */
		{          250ULL, 0x22U }, /* 250 ns  */
		{         1000ULL, 0x23U }, /* 1 us    */
		{         2500ULL, 0x24U }, /* 2.5 us  */
		{        10000ULL, 0x25U }, /* 10 us   */
		{        25000ULL, 0x26U }, /* 25 us   */
		{       100000ULL, 0x27U }, /* 100 us  */
		{       250000ULL, 0x28U }, /* 250 us  */
		{      1000000ULL, 0x29U }, /* 1 ms    */
		{      2500000ULL, 0x2AU }, /* 2.5 ms  */
		{     10000000ULL, 0x2BU }, /* 10 ms   */
		{     25000000ULL, 0x2CU }, /* 25 ms   */
		{    100000000ULL, 0x2DU }, /* 100 ms  */
		{    250000000ULL, 0x2EU }, /* 250 ms  */
		{   1000000000ULL, 0x2FU }, /* 1 s     */
		{  10000000000ULL, 0x30U }, /* 10 s    */
	};
	size_t i;

	if (est_ns == 0U) {
		return PTP_CLOCK_ACCURACY_UNKNOWN;
	}
	for (i = 0U; i < ARRAY_LEN(ladder); i++) {
		if (est_ns <= ladder[i].max_ns) {
			return ladder[i].code;
		}
	}
	return 0x31U; /* > 10 s */
}

/*
 * Table 4. Classes 6 and 7 are the primary-reference and in-spec-holdover
 * codes; both sit in the 1..127 "never a slave" band, which is what keeps this
 * appliance out of the SLAVE branch of the state decision while it is healthy.
 *
 * TODO(wave-3): a clock that has never locked since boot should strictly
 * advertise 248 (default), not a degradation class. The quality view has no
 * "has ever locked" bit yet; add one with core/quality and gate FREERUN on it.
 *
 * This is not cosmetic, which is why it should not be deprioritised. A unit
 * cold-booted with no antenna advertises class 52 from its first Announce.
 * Against a genuine peer that has degraded honestly to 187, or a default-class
 * 248 clock, 52 wins the BMCA outright — so the box with no idea what time it
 * is becomes grandmaster for the segment, and stays there until GNSS comes up.
 * The gate turns that first Announce into 248 and lets the better clock win.
 */
static uint8_t clock_class_from_state(const ptp_cfg_t *cfg, ptp_sync_state_t s)
{
	switch (s) {
	case PTP_SYNC_LOCKED:
		return 6U;
	case PTP_SYNC_HOLDOVER:
		return 7U;
	case PTP_SYNC_HOLDOVER_EXCEEDED:
	case PTP_SYNC_FREERUN:
	case PTP_SYNC_COUNT:
	default:
		return (cfg->degradation == PTP_DEGRADE_ALT_B) ? 187U : 52U;
	}
}

int ptp_clock_quality_from_view(const ptp_cfg_t *cfg, const ptp_quality_view_t *q,
				ptp_clock_quality_t *out)
{
	bool disciplined;

	if ((cfg == NULL) || (q == NULL) || (out == NULL)) {
		return -EINVAL;
	}

	out->clock_class = clock_class_from_state(cfg, q->sync_state);
	out->clock_accuracy = accuracy_from_ns(q->est_accuracy_ns);

	disciplined = (q->sync_state == PTP_SYNC_LOCKED) ||
		      (q->sync_state == PTP_SYNC_HOLDOVER);

	if (q->adev_tau1_e18 != 0U) {
		out->offset_scaled_log_variance =
			oslv_from_adev(q->adev_tau1_e18, cfg->log_sync_interval);
	} else if (disciplined) {
		out->offset_scaled_log_variance = cfg->oslv_locked;
	} else {
		out->offset_scaled_log_variance = PTP_OSLV_UNKNOWN;
	}

	return 0;
}

/* --------------------------------------------------- §3.8 block -> view --- */

/*
 * Clamp a double into [0, max] without libm. Every comparison is written so
 * that a NaN takes the "unknown" branch: !(d > 0.0) is true for NaN, and
 * !(d < max) is true for both NaN and +inf.
 */
static uint64_t clamp_to_u64(double d, double max)
{
	if (!(d > 0.0)) {
		return 0U;
	}
	if (!(d < max)) {
		return (uint64_t)max;
	}
	return (uint64_t)d;
}

static ptp_sync_state_t sync_state_from_block(const quality_block_t *b)
{
	bool holdover = b->holdover ||
			(b->lock_state == (uint8_t)QUALITY_LOCK_HOLDOVER) ||
			(b->lock_state == (uint8_t)QUALITY_LOCK_RECOVERING) ||
			((b->flags & QUALITY_FLAG_HOLDOVER) != 0U);

	if ((b->lock_state == (uint8_t)QUALITY_LOCK_LOCKED) && !holdover) {
		return PTP_SYNC_LOCKED;
	}
	if (holdover) {
		/* QUALITY_FLAG_DEMOTED is "holdover past policy" — §3.6's exit. */
		return ((b->flags & QUALITY_FLAG_DEMOTED) != 0U)
			       ? PTP_SYNC_HOLDOVER_EXCEEDED
			       : PTP_SYNC_HOLDOVER;
	}
	return PTP_SYNC_FREERUN;
}

static uint8_t time_source_from_block(const quality_block_t *b)
{
	if ((b->flags & QUALITY_FLAG_GNSS_TIME_LOCKED) != 0U) {
		return (uint8_t)PTP_TIME_SRC_GNSS;
	}
	if (b->active_ref == (uint8_t)QUALITY_REF_RB) {
		return (uint8_t)PTP_TIME_SRC_ATOMIC_CLOCK;
	}
	if (b->active_ref == (uint8_t)QUALITY_REF_EXTREF) {
		/* A house 10 MHz standard: real, but of unknown provenance to us. */
		return (uint8_t)PTP_TIME_SRC_OTHER;
	}
	return (uint8_t)PTP_TIME_SRC_INTERNAL_OSC;
}

static uint64_t est_accuracy_from_block(const quality_block_t *b,
					ptp_sync_state_t s)
{
	if ((s == PTP_SYNC_HOLDOVER) || (s == PTP_SYNC_HOLDOVER_EXCEEDED)) {
		int64_t e = b->holdover_est_err_ns;

		if (e < 0) {
			/*
			 * Negating INT64_MIN overflows, so take the magnitude in
			 * the unsigned domain: -(e + 1) is always representable.
			 */
			return (uint64_t)(-(e + 1)) + 1U;
		}
		return (uint64_t)e;
	}
	if (b->gnss_tacc_ns != 0U) {
		return (uint64_t)b->gnss_tacc_ns;
	}
	/* Nothing from the receiver: fall back to our own dispersion estimate. */
	return clamp_to_u64((double)b->pps_off_sigma_ns, 1e18);
}

static void leap_from_block(const quality_block_t *b, uint64_t now_tai_s,
			    ptp_quality_view_t *out)
{
	out->leap61 = false;
	out->leap59 = false;

	if (b->leap_pending == 0) {
		return;
	}
	/* Only inside the announcement window, and only before the event. */
	if (b->leap_at_tai_s <= now_tai_s) {
		return;
	}
	if ((b->leap_at_tai_s - now_tai_s) > (uint64_t)PTP_LEAP_ANNOUNCE_WINDOW_S) {
		return;
	}

	if (b->leap_pending > 0) {
		out->leap61 = true;
	} else {
		out->leap59 = true;
	}
}

int ptp_quality_view_from_block(const quality_block_t *b, uint64_t now_tai_s,
				ptp_quality_view_t *out)
{
	ptp_sync_state_t s;
	bool traceable;

	if ((b == NULL) || (out == NULL)) {
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));

	s = sync_state_from_block(b);
	traceable = (s == PTP_SYNC_LOCKED) || (s == PTP_SYNC_HOLDOVER);

	out->sync_state = s;
	out->utc_offset = b->leap_current_s;
	out->utc_offset_valid = b->utc_valid;
	if (!b->utc_valid && (b->leap_current_s == 0)) {
		/*
		 * Before the receiver has delivered a leap offset the block
		 * carries 0, which as a currentUtcOffset claims TAI == UTC — 37
		 * seconds wrong and superficially plausible. Advertise the
		 * standing offset instead, with currentUtcOffsetValid clear so a
		 * client knows not to trust it. Once the receiver reports, the
		 * real value takes over.
		 */
		out->utc_offset = PTP_DEFAULT_UTC_OFFSET;
	}
	leap_from_block(b, now_tai_s, out);
	out->time_traceable = traceable;
	out->freq_traceable = traceable ||
			      (b->active_ref == (uint8_t)QUALITY_REF_RB) ||
			      (b->active_ref == (uint8_t)QUALITY_REF_EXTREF);
	out->time_source = time_source_from_block(b);
	out->est_accuracy_ns = est_accuracy_from_block(b, s);
	out->adev_tau1_e18 = clamp_to_u64((double)b->adev_1s * 1e18, 1e19);

	return 0;
}

uint16_t ptp_flags_from_view(const ptp_quality_view_t *q)
{
	/* The appliance always serves the PTP (TAI) timescale, never ARB. */
	uint16_t f = PTP_FLAG_PTP_TIMESCALE;

	if (q == NULL) {
		return f;
	}

	/*
	 * §7.2.4 makes leap61 and leap59 mutually exclusive. Should a quality
	 * view ever assert both, insertion wins and deletion is dropped, so the
	 * wire never carries the invalid pair.
	 */
	if (q->leap61) {
		f |= PTP_FLAG_LEAP61;
	} else if (q->leap59) {
		f |= PTP_FLAG_LEAP59;
	} else {
		/* no pending leap */
	}

	if (q->utc_offset_valid) {
		f |= PTP_FLAG_UTC_OFFSET_VALID;
	}
	if (q->time_traceable) {
		f |= PTP_FLAG_TIME_TRACEABLE;
	}
	if (q->freq_traceable) {
		f |= PTP_FLAG_FREQ_TRACEABLE;
	}
	return f;
}

const char *ptp_port_state_name(ptp_port_state_t s)
{
	switch (s) {
	case PTP_PS_INITIALIZING:
		return "INITIALIZING";
	case PTP_PS_FAULTY:
		return "FAULTY";
	case PTP_PS_DISABLED:
		return "DISABLED";
	case PTP_PS_LISTENING:
		return "LISTENING";
	case PTP_PS_PRE_MASTER:
		return "PRE_MASTER";
	case PTP_PS_MASTER:
		return "MASTER";
	case PTP_PS_PASSIVE:
		return "PASSIVE";
	case PTP_PS_UNCALIBRATED:
		return "UNCALIBRATED";
	case PTP_PS_SLAVE:
		return "SLAVE";
	default:
		return "?";
	}
}

/* -------------------------------------------------------------- datasets -- */

int ptp_port_dataset(const ptp_port_ctx_t *c, ptp_dataset_t *out)
{
	if ((c == NULL) || (out == NULL)) {
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));
	out->priority1 = c->cfg.priority1;
	(void)ptp_clock_quality_from_view(&c->cfg, &c->quality, &out->quality);
	out->priority2 = c->cfg.priority2;
	out->gm_identity = c->clock_id;
	out->steps_removed = 0U;
	/*
	 * D0's sender and receiver are both this port (§9.3.4): that identity is
	 * exactly what makes part 2 report ERROR_1 when a clock hears its own
	 * dataset announced back at it.
	 */
	out->sender = c->port_id;
	out->receiver = c->port_id;
	return 0;
}

/* ------------------------------------------------------------- transmit --- */

static void hdr_init(const ptp_port_ctx_t *c, ptp_hdr_t *h, uint8_t type,
		     uint16_t seq, int8_t log_interval, uint16_t flags,
		     int64_t correction)
{
	memset(h, 0, sizeof(*h));
	h->major_sdo_id = c->cfg.major_sdo_id;
	h->msg_type = type;
	h->minor_version = PTP_MINOR_VERSION_2019;
	h->version = PTP_VERSION;
	h->msg_length = (uint16_t)ptp_msg_min_len(type);
	h->domain = c->cfg.domain;
	h->minor_sdo_id = c->cfg.minor_sdo_id;
	h->flags = flags;
	h->correction = correction;
	h->source_port = c->port_id;
	h->seq_id = seq;
	h->control = ptp_msg_control_field(type);
	h->log_msg_interval = log_interval;
}

/*
 * @p buf is the scratch the message was encoded into. It is a parameter rather
 * than always c->txbuf because a Follow_Up can be released from inside the Sync
 * transmit callback, while the glue still holds a pointer into the Sync's
 * buffer; the two must not be the same storage.
 */
static int emit(ptp_port_ctx_t *c, const uint8_t *buf, size_t len, uint8_t type,
		uint16_t seq, ptp_port_kind_t kind, ptp_addr_hint_t addr,
		const ptp_port_id_t *peer)
{
	ptp_tx_desc_t d;
	int rc;

	memset(&d, 0, sizeof(d));
	d.buf = buf;
	d.len = len;
	d.msg_type = type;
	d.seq = seq;
	d.port_kind = kind;
	d.addr = addr;
	if (peer != NULL) {
		d.peer = *peer;
	}
	d.transport = c->cfg.transport;

	rc = c->ops.tx(c->ops.ctx, &d);
	if (rc != 0) {
		c->counters.tx_errors++;
		c->alarms |= PTP_ALARM_TX_ERROR;
		return rc;
	}

	c->counters.tx[type & 0x0FU]++;
	c->alarms &= ~PTP_ALARM_TX_ERROR;
	return 0;
}

static uint64_t now_tai_ns(const ptp_port_ctx_t *c)
{
	uint64_t ns = 0U;

	if (c->ops.tai_ns == NULL) {
		return 0U;
	}
	if (c->ops.tai_ns(c->ops.ctx, &ns) != 0) {
		/* §11.4.3 permits a zero originTimestamp on a two-step Sync. */
		return 0U;
	}
	return ns;
}

static void tx_announce(ptp_port_ctx_t *c)
{
	ptp_hdr_t h;
	ptp_announce_t a;
	size_t len = 0U;
	int rc;

	memset(&a, 0, sizeof(a));
	/* originTimestamp is reserved in -2019 and goes out as zero. */
	(void)ptp_clock_quality_from_view(&c->cfg, &c->quality, &a.gm_quality);
	a.current_utc_offset = c->quality.utc_offset;
	a.gm_priority1 = c->cfg.priority1;
	a.gm_priority2 = c->cfg.priority2;
	a.gm_identity = c->clock_id;
	a.steps_removed = 0U;
	a.time_source = c->quality.time_source;

	hdr_init(c, &h, (uint8_t)PTP_MSG_ANNOUNCE, c->announce_seq,
		 c->cfg.log_announce_interval, ptp_flags_from_view(&c->quality), 0);

	rc = ptp_announce_encode(c->txbuf, sizeof(c->txbuf), &h, &a, &len);
	if (rc != 0) {
		c->counters.tx_errors++;
		return;
	}

	(void)emit(c, c->txbuf, len, (uint8_t)PTP_MSG_ANNOUNCE, c->announce_seq,
		   PTP_PORT_GENERAL, PTP_ADDR_PRIMARY, NULL);
	c->announce_seq++;
}

static void tx_sync(ptp_port_ctx_t *c)
{
	ptp_hdr_t h;
	ptp_timestamp_t ts;
	size_t len = 0U;
	uint16_t seq = c->sync_seq;
	int rc;

	if (c->sync_pending) {
		/* The previous Sync never got its egress timestamp back. */
		c->counters.followup_missed++;
		c->sync_pending = false;
	}

	ts = ptp_ts_from_ns(now_tai_ns(c));

	hdr_init(c, &h, (uint8_t)PTP_MSG_SYNC, seq, c->cfg.log_sync_interval,
		 PTP_FLAG_TWO_STEP, 0);

	rc = ptp_tsmsg_encode(c->txbuf, sizeof(c->txbuf), &h, &ts, &len);
	if (rc != 0) {
		c->counters.tx_errors++;
		return;
	}

	/*
	 * Arm the Follow_Up before handing the frame to the glue: a driver that
	 * can read its own egress timestamp synchronously will call
	 * ptp_on_sync_txts() from inside the tx callback.
	 */
	c->sync_pending = true;
	c->sync_pending_seq = seq;
	c->sync_seq++;

	if (emit(c, c->txbuf, len, (uint8_t)PTP_MSG_SYNC, seq, PTP_PORT_EVENT,
		 PTP_ADDR_PRIMARY, NULL) != 0) {
		if (c->sync_pending) {
			/*
			 * Nothing left the interface, so no egress timestamp is
			 * coming and there is no Follow_Up to publish. Disarming
			 * here also keeps the next Sync from charging this one
			 * to followup_missed, which would double-count a failure
			 * tx_errors already records.
			 */
			c->sync_pending = false;
		} else {
			/*
			 * The glue released the Follow_Up synchronously from
			 * inside the callback and only then reported the Sync as
			 * failed. The Follow_Up is already on the wire with
			 * nothing to follow; a receiver discards it, but the
			 * operator should see that the glue did this.
			 */
			c->counters.followup_orphaned++;
		}
	}
}

int ptp_on_sync_txts(ptp_port_ctx_t *c, uint16_t seq, uint64_t tai_ns)
{
	ptp_hdr_t h;
	ptp_timestamp_t ts;
	size_t len = 0U;
	int rc;

	if (c == NULL) {
		return -EINVAL;
	}
	if (!c->sync_pending || (c->sync_pending_seq != seq)) {
		c->counters.txts_unmatched++;
		return -ENOENT;
	}
	c->sync_pending = false;

	ts = ptp_ts_from_ns(tai_ns);
	hdr_init(c, &h, (uint8_t)PTP_MSG_FOLLOW_UP, seq, c->cfg.log_sync_interval,
		 0U, 0);

	/* Its own scratch: c->txbuf may still hold the Sync being transmitted. */
	rc = ptp_tsmsg_encode(c->fubuf, sizeof(c->fubuf), &h, &ts, &len);
	if (rc != 0) {
		c->counters.tx_errors++;
		return rc;
	}

	return emit(c, c->fubuf, len, (uint8_t)PTP_MSG_FOLLOW_UP, seq,
		    PTP_PORT_GENERAL, PTP_ADDR_PRIMARY, NULL);
}

/* -------------------------------------------------------------- schedule -- */

/*
 * One-shot due test that advances by whole intervals, so cadence does not drift
 * with the call rate. If the caller has fallen more than a whole interval
 * behind, the schedule is re-anchored to now instead of catching up in a burst:
 * a flood of back-to-back Syncs helps nobody.
 */
static bool due(uint64_t *next, uint64_t now, uint32_t interval)
{
	if (now < *next) {
		return false;
	}
	*next += interval;
	if (*next <= now) {
		*next = now + interval;
	}
	return true;
}

/* ------------------------------------------------------------------ BMCA -- */

static void sync_foreign_counters(ptp_port_ctx_t *c)
{
	c->counters.foreign_added = c->foreign.added;
	c->counters.foreign_evicted = c->foreign.evicted;
	c->counters.foreign_expired = c->foreign.expired;
}

static void enter_state(ptp_port_ctx_t *c, ptp_port_state_t next, uint64_t now_ms)
{
	c->state = next;
	c->counters.state_changes++;

	if (next == PTP_PS_MASTER) {
		/* Announce and Sync at once, then on cadence. */
		c->announce_next_ms = now_ms;
		c->sync_next_ms = now_ms;
	} else {
		/* Any Sync in flight loses its Follow_Up: we are no longer master. */
		c->sync_pending = false;
	}

	if (next == PTP_PS_LISTENING) {
		c->announce_rx_ms = now_ms;
		c->arto_latched = false;
	}
}

/*
 * ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES (§9.2.6.11): no qualified Announce for
 * announceReceiptTimeout announce intervals. Counted once per episode, so a
 * port that sits in MASTER for a week reports one timeout, not millions.
 */
static bool check_announce_timeout(ptp_port_ctx_t *c, uint64_t now_ms)
{
	if (now_ms <= c->announce_rx_ms) {
		return false;
	}
	if ((now_ms - c->announce_rx_ms) < (uint64_t)c->arto_ms) {
		return false;
	}
	if (!c->arto_latched) {
		c->counters.announce_timeouts++;
		c->arto_latched = true;
	}
	return true;
}

static void apply_recommended(ptp_port_ctx_t *c, ptp_recommended_t rec,
			      uint64_t now_ms)
{
	ptp_port_state_t next = c->state;

	switch (rec) {
	case PTP_REC_LISTENING:
		next = PTP_PS_LISTENING;
		break;
	case PTP_REC_M1:
	case PTP_REC_M2:
	case PTP_REC_M3:
		next = PTP_PS_MASTER;
		break;
	case PTP_REC_P1:
	case PTP_REC_P2:
	case PTP_REC_S1:
		/*
		 * S1 would be SLAVE in strict 1588. This appliance never slaves;
		 * it defers and raises PTP_ALARM_NOT_BEST_MASTER instead. See the
		 * scope note in ptp.h.
		 */
		next = PTP_PS_PASSIVE;
		break;
	case PTP_REC_COUNT:
	default:
		break;
	}

	if (next != c->state) {
		enter_state(c, next, now_ms);
	}
}

static void run_bmca(ptp_port_ctx_t *c, uint64_t now_ms)
{
	ptp_dataset_t d0;
	ptp_dataset_t erbest_ds;
	const ptp_dataset_t *erbest = NULL;
	ptp_recommended_t rec;
	bool timed_out;

	(void)ptp_port_dataset(c, &d0);

	if (ptp_foreign_best(&c->foreign, &c->port_id, &erbest_ds) != NULL) {
		erbest = &erbest_ds;
	}

	/*
	 * Evaluated unconditionally, and before it is combined with the port
	 * state: the receipt timeout is an event worth counting in MASTER and
	 * PASSIVE too — it is how an operator sees that the peer we deferred to
	 * has gone quiet. Only the LISTENING hold consumes the result.
	 */
	timed_out = check_announce_timeout(c, now_ms);

	c->counters.bmca_runs++;

	/* Single port: Ebest is Erbest, and it is on this port by construction. */
	rec = ptp_bmca_state_decision(&d0, erbest, erbest, (erbest != NULL),
				      (c->state == PTP_PS_LISTENING) && !timed_out);

	if (rec != c->last_rec) {
		c->counters.bmca_decisions++;
		c->last_rec = rec;
	}

	switch (rec) {
	case PTP_REC_P1:
	case PTP_REC_P2:
	case PTP_REC_S1:
		c->alarms |= PTP_ALARM_NOT_BEST_MASTER;
		break;
	default:
		c->alarms &= ~PTP_ALARM_NOT_BEST_MASTER;
		break;
	}

	apply_recommended(c, rec, now_ms);
}

/* -------------------------------------------------------------- receive --- */

static int rx_announce(ptp_port_ctx_t *c, const ptp_hdr_t *hdr,
		       const uint8_t *buf, size_t len, uint64_t now_ms)
{
	ptp_announce_t a;
	ptp_foreign_t *f;

	if (ptp_announce_decode(buf, len, &a) != 0) {
		c->counters.rx_dropped++;
		return -EBADMSG;
	}

	/* §9.3.2.5 d): an Announce this far from its grandmaster is not a candidate. */
	if ((uint32_t)a.steps_removed >= PTP_STEPS_REMOVED_MAX) {
		c->counters.rx_ignored++;
		return 0;
	}

	f = ptp_foreign_update(&c->foreign, &c->policy, &hdr->source_port, &a,
			       hdr->flags, hdr->log_msg_interval, now_ms);
	sync_foreign_counters(c);
	if (f == NULL) {
		c->counters.rx_dropped++;
		return -EBADMSG;
	}

	if (f->qualified) {
		c->announce_rx_ms = now_ms;
		c->arto_latched = false;
	}

	run_bmca(c, now_ms);
	return 0;
}

static void rx_delay_req(ptp_port_ctx_t *c, const ptp_hdr_t *hdr,
			 uint64_t rx_tai_ns)
{
	ptp_hdr_t h;
	ptp_timestamp_t ts;
	ptp_addr_hint_t addr;
	uint16_t flags = 0U;
	size_t len = 0U;
	int rc;

	/* Only a master answers Delay_Req (§9.5.10). */
	if (c->state != PTP_PS_MASTER) {
		c->counters.rx_ignored++;
		return;
	}

	if ((hdr->flags & PTP_FLAG_UNICAST) != 0U) {
		addr = PTP_ADDR_UNICAST_PEER;
		flags = PTP_FLAG_UNICAST;
	} else {
		addr = PTP_ADDR_PRIMARY;
	}

	ts = ptp_ts_from_ns(rx_tai_ns);

	/*
	 * §11.3: the response carries back the request's correctionField less
	 * the sub-nanosecond residue of the receive timestamp. Hardware hands us
	 * whole nanoseconds, so the residue is zero and this is a passthrough.
	 */
	hdr_init(c, &h, (uint8_t)PTP_MSG_DELAY_RESP, hdr->seq_id,
		 c->cfg.log_min_delay_req_interval, flags, hdr->correction);

	rc = ptp_delay_resp_encode(c->txbuf, sizeof(c->txbuf), &h, &ts,
				   &hdr->source_port, &len);
	if (rc != 0) {
		c->counters.tx_errors++;
		return;
	}

	(void)emit(c, c->txbuf, len, (uint8_t)PTP_MSG_DELAY_RESP, hdr->seq_id,
		   PTP_PORT_GENERAL, addr, &hdr->source_port);
}

int ptp_port_rx(ptp_port_ctx_t *c, const uint8_t *buf, size_t len,
		uint64_t rx_tai_ns, uint64_t now_ms)
{
	ptp_hdr_t hdr;
	int rc;

	if ((c == NULL) || (buf == NULL)) {
		return -EINVAL;
	}
	if ((c->state == PTP_PS_INITIALIZING) || (c->state == PTP_PS_FAULTY)) {
		return -EPERM;
	}

	rc = ptp_hdr_decode(buf, len, &hdr);
	if (rc != 0) {
		if (rc == -EPROTO) {
			c->counters.rx_foreign_domain++;
		} else {
			c->counters.rx_dropped++;
		}
		return rc;
	}

	/* A different domain or SDO shares the wire but not the timebase. */
	if ((hdr.domain != c->cfg.domain) ||
	    (hdr.major_sdo_id != c->cfg.major_sdo_id)) {
		c->counters.rx_foreign_domain++;
		return -EPROTO;
	}

	/* Our own multicast, looped back by the switch or the stack. */
	if (ptp_clock_id_cmp(&hdr.source_port.clock_id, &c->clock_id) == 0) {
		c->counters.rx_self++;
		return 0;
	}

	c->counters.rx[hdr.msg_type & 0x0FU]++;

	/*
	 * An event message is only worth anything with its hardware ingress
	 * timestamp, and 0 is the documented "no timestamp" value the glue
	 * passes for general messages. Answering a Delay_Req with
	 * receiveTimestamp 0 would tell the requester its path delay is some
	 * decades negative, which is far worse than not answering at all.
	 */
	if (ptp_msg_is_event(hdr.msg_type) && (rx_tai_ns == 0U)) {
		c->counters.rx_no_timestamp++;
		return -ENODATA;
	}

	switch (hdr.msg_type) {
	case PTP_MSG_ANNOUNCE:
		rc = rx_announce(c, &hdr, buf, len, now_ms);
		break;
	case PTP_MSG_DELAY_REQ:
		rx_delay_req(c, &hdr, rx_tai_ns);
		break;
	default:
		/*
		 * Sync/Follow_Up/Delay_Resp from a peer master, the Pdelay
		 * family, Signaling and Management: parsed, counted, and of no
		 * consequence to a grandmaster-only ordinary clock running E2E.
		 */
		c->counters.rx_ignored++;
		break;
	}

	return rc;
}

/* ----------------------------------------------------------- lifecycle --- */

int ptp_port_init(ptp_port_ctx_t *c, const ptp_cfg_t *cfg, const uint8_t *mac,
		  const ptp_port_ops_t *ops)
{
	int rc;

	if ((c == NULL) || (cfg == NULL) || (mac == NULL) || (ops == NULL) ||
	    (ops->tx == NULL)) {
		return -EINVAL;
	}

	rc = ptp_cfg_validate(cfg);
	if (rc != 0) {
		return rc;
	}

	memset(c, 0, sizeof(*c));
	c->cfg = *cfg;
	c->ops = *ops;

	/* Both arguments were checked above, so this cannot fail. */
	(void)ptp_clock_id_from_mac(&c->clock_id, mac);
	c->port_id.clock_id = c->clock_id;
	c->port_id.port_number = cfg->port_number;

	c->announce_ms = ptp_log_interval_ms(cfg->log_announce_interval);
	c->sync_ms = ptp_log_interval_ms(cfg->log_sync_interval);
	c->arto_ms = (uint32_t)cfg->announce_receipt_timeout * c->announce_ms;

	c->policy.receipt_timeout = cfg->announce_receipt_timeout;
	c->policy.default_interval_ms = c->announce_ms;

	ptp_foreign_init(&c->foreign);

	c->quality.sync_state = PTP_SYNC_FREERUN;
	c->quality.time_source = (uint8_t)PTP_TIME_SRC_INTERNAL_OSC;

	c->state = PTP_PS_INITIALIZING;
	c->last_rec = PTP_REC_LISTENING;

	if (cfg->profile != PTP_PROFILE_DEFAULT) {
		c->alarms |= PTP_ALARM_PROFILE_UNSUPPORTED;
	}

	return 0;
}

int ptp_port_enable(ptp_port_ctx_t *c, uint64_t now_ms)
{
	if (c == NULL) {
		return -EINVAL;
	}
	if ((c->state != PTP_PS_INITIALIZING) && (c->state != PTP_PS_FAULTY)) {
		return -EPERM;
	}

	c->alarms &= ~PTP_ALARM_FAULTY;
	c->last_rec = PTP_REC_LISTENING;
	enter_state(c, PTP_PS_LISTENING, now_ms);
	return 0;
}

int ptp_port_set_quality(ptp_port_ctx_t *c, const ptp_quality_view_t *q)
{
	if ((c == NULL) || (q == NULL)) {
		return -EINVAL;
	}
	c->quality = *q;
	return 0;
}

int ptp_port_step(ptp_port_ctx_t *c, uint64_t now_ms)
{
	if (c == NULL) {
		return -EINVAL;
	}
	if ((c->state == PTP_PS_INITIALIZING) || (c->state == PTP_PS_FAULTY)) {
		return 0;
	}

	if (ptp_foreign_prune(&c->foreign, &c->policy, now_ms) > 0U) {
		sync_foreign_counters(c);
	}

	run_bmca(c, now_ms);

	if (c->state == PTP_PS_MASTER) {
		if (due(&c->announce_next_ms, now_ms, c->announce_ms)) {
			tx_announce(c);
		}
		if (due(&c->sync_next_ms, now_ms, c->sync_ms)) {
			tx_sync(c);
		}
	}

	return 0;
}

int ptp_port_fault(ptp_port_ctx_t *c, uint64_t now_ms)
{
	if (c == NULL) {
		return -EINVAL;
	}
	if (c->state == PTP_PS_FAULTY) {
		return 0;
	}

	/* The link is gone: everything we believed about the segment is stale. */
	ptp_foreign_clear(&c->foreign);
	c->alarms |= PTP_ALARM_FAULTY;
	c->alarms &= ~PTP_ALARM_NOT_BEST_MASTER;
	c->last_rec = PTP_REC_LISTENING;
	enter_state(c, PTP_PS_FAULTY, now_ms);
	return 0;
}

int ptp_port_fault_reset(ptp_port_ctx_t *c, uint64_t now_ms)
{
	if (c == NULL) {
		return -EINVAL;
	}
	if (c->state != PTP_PS_FAULTY) {
		return -EPERM;
	}
	return ptp_port_enable(c, now_ms);
}

/* ------------------------------------------------------------ accessors --- */

ptp_port_state_t ptp_port_state(const ptp_port_ctx_t *c)
{
	return (c != NULL) ? c->state : PTP_PS_INITIALIZING;
}

uint32_t ptp_port_alarms(const ptp_port_ctx_t *c)
{
	return (c != NULL) ? c->alarms : 0U;
}

const ptp_counters_t *ptp_port_counters(const ptp_port_ctx_t *c)
{
	return (c != NULL) ? &c->counters : NULL;
}
