/*
 * STS1000 "Meridian" — core/ptp: best master clock algorithm.
 *
 * Three pieces, all of them pure or table-local so they can be driven directly
 * from the host tests:
 *
 *   - ptp_bmca_compare()        the dataset comparison algorithm, §9.3.4
 *   - ptp_bmca_state_decision() the state decision algorithm, §9.3.3
 *   - ptp_foreign_*()           the bounded foreign-master table, §9.3.2.4
 *
 * Section references are to IEEE 1588-2019.
 */

#include "ptp/ptp.h"

#include <string.h>

/* ------------------------------------------------------ dataset compare --- */

static ptp_dscmp_t pick(bool a_wins)
{
	return a_wins ? PTP_DSCMP_A_BETTER : PTP_DSCMP_B_BETTER;
}

/*
 * Part 2 of the comparison (Figure 35): the two datasets advertise the same
 * grandmaster, so what is left is topology. The ±1 dead band on stepsRemoved
 * keeps a boundary clock and the master one hop above it from fighting over
 * which is closer to the grandmaster.
 */
static ptp_dscmp_t compare_part2(const ptp_dataset_t *a, const ptp_dataset_t *b)
{
	uint32_t sa = a->steps_removed;
	uint32_t sb = b->steps_removed;
	int d;

	if ((sa + 1U) < sb) {
		return PTP_DSCMP_A_BETTER;
	}
	if ((sb + 1U) < sa) {
		return PTP_DSCMP_B_BETTER;
	}

	if (sa < sb) {
		/* B is the further one; ask whether its receiver is its own sender. */
		d = ptp_port_id_cmp(&b->receiver, &b->sender);
		if (d < 0) {
			return PTP_DSCMP_A_BETTER_TOPO;
		}
		if (d > 0) {
			return PTP_DSCMP_A_BETTER;
		}
		return PTP_DSCMP_ERROR_1;
	}
	if (sa > sb) {
		d = ptp_port_id_cmp(&a->receiver, &a->sender);
		if (d < 0) {
			return PTP_DSCMP_B_BETTER_TOPO;
		}
		if (d > 0) {
			return PTP_DSCMP_B_BETTER;
		}
		return PTP_DSCMP_ERROR_1;
	}

	/* Equally far away: order on the senders, then on the receiving ports. */
	d = ptp_port_id_cmp(&a->sender, &b->sender);
	if (d < 0) {
		return PTP_DSCMP_A_BETTER_TOPO;
	}
	if (d > 0) {
		return PTP_DSCMP_B_BETTER_TOPO;
	}

	if (a->receiver.port_number < b->receiver.port_number) {
		return PTP_DSCMP_A_BETTER_TOPO;
	}
	if (a->receiver.port_number > b->receiver.port_number) {
		return PTP_DSCMP_B_BETTER_TOPO;
	}

	/* Same grandmaster, same distance, same sender, same receiving port. */
	return PTP_DSCMP_ERROR_2;
}

ptp_dscmp_t ptp_bmca_compare_profile(const ptp_dataset_t *a,
				     const ptp_dataset_t *b, uint8_t profile)
{
	const ptp_profile_desc_t *desc = ptp_profile_desc(profile);
	int d;

	if ((a == NULL) || (b == NULL)) {
		return PTP_DSCMP_ERROR_2;
	}

	d = ptp_clock_id_cmp(&a->gm_identity, &b->gm_identity);
	if (d == 0) {
		/*
		 * Part 2 is unchanged by every profile in scope. Two datasets
		 * that reach it describe the same grandmaster over different
		 * paths, and a local preference between paths is what
		 * portDS.localPriority already expresses through part 1.
		 */
		return compare_part2(a, b);
	}

	/* Part 1 (Figure 34): lower wins at every rung. */
	if (desc->use_priority1 && (a->priority1 != b->priority1)) {
		/*
		 * G.8275.1 §6.3 removes this rung: the profile pins priority1 at
		 * 128 for every conforming clock, so a difference here can only
		 * come from a misconfigured peer, and honouring it would let that
		 * peer win an election it should lose on clockClass.
		 */
		return pick(a->priority1 < b->priority1);
	}
	if (a->quality.clock_class != b->quality.clock_class) {
		return pick(a->quality.clock_class < b->quality.clock_class);
	}
	if (a->quality.clock_accuracy != b->quality.clock_accuracy) {
		return pick(a->quality.clock_accuracy < b->quality.clock_accuracy);
	}
	if (a->quality.offset_scaled_log_variance !=
	    b->quality.offset_scaled_log_variance) {
		return pick(a->quality.offset_scaled_log_variance <
			    b->quality.offset_scaled_log_variance);
	}
	if (a->priority2 != b->priority2) {
		return pick(a->priority2 < b->priority2);
	}
	if (desc->use_local_priority && (a->local_priority != b->local_priority)) {
		/* The alternate BMCA's own rung, between priority2 and identity. */
		return pick(a->local_priority < b->local_priority);
	}

	/* The identity tiebreak makes the ordering total. */
	return pick(d < 0);
}

ptp_dscmp_t ptp_bmca_compare(const ptp_dataset_t *a, const ptp_dataset_t *b)
{
	return ptp_bmca_compare_profile(a, b, (uint8_t)PTP_PROFILE_DEFAULT);
}

/* -------------------------------------------------------- state decision -- */

/* "Better" against a missing dataset: anything beats nothing. */
static bool better_than(const ptp_dataset_t *a, const ptp_dataset_t *b,
			uint8_t profile)
{
	if (b == NULL) {
		return true;
	}
	return ptp_dscmp_a_wins(ptp_bmca_compare_profile(a, b, profile));
}

ptp_recommended_t ptp_bmca_state_decision_profile(const ptp_dataset_t *d0,
						 const ptp_dataset_t *erbest,
						 const ptp_dataset_t *ebest,
						 bool ebest_on_this_port,
						 bool listening,
						 uint8_t profile)
{
	if (d0 == NULL) {
		return PTP_REC_LISTENING;
	}

	/* Nothing heard yet and nothing to decide: stay in LISTENING. */
	if ((erbest == NULL) && listening) {
		return PTP_REC_LISTENING;
	}

	/*
	 * clockClass 1..127 is the "shall never be a slave" range (§7.6.2.5), so
	 * the decision is master-or-passive and is taken against this port's own
	 * best, not the clock-wide best.
	 */
	if ((d0->quality.clock_class >= 1U) && (d0->quality.clock_class <= 127U)) {
		return better_than(d0, erbest, profile) ? PTP_REC_M1 : PTP_REC_P1;
	}

	if (better_than(d0, ebest, profile)) {
		return PTP_REC_M2;
	}
	if (ebest_on_this_port) {
		return PTP_REC_S1;
	}
	if (ptp_bmca_compare_profile(ebest, erbest, profile) ==
	    PTP_DSCMP_A_BETTER_TOPO) {
		return PTP_REC_P2;
	}
	return PTP_REC_M3;
}

ptp_recommended_t ptp_bmca_state_decision(const ptp_dataset_t *d0,
					  const ptp_dataset_t *erbest,
					  const ptp_dataset_t *ebest,
					  bool ebest_on_this_port,
					  bool listening)
{
	return ptp_bmca_state_decision_profile(d0, erbest, ebest,
					       ebest_on_this_port, listening,
					       (uint8_t)PTP_PROFILE_DEFAULT);
}

const char *ptp_recommended_name(ptp_recommended_t r)
{
	switch (r) {
	case PTP_REC_LISTENING:
		return "LISTENING";
	case PTP_REC_M1:
		return "M1";
	case PTP_REC_M2:
		return "M2";
	case PTP_REC_M3:
		return "M3";
	case PTP_REC_P1:
		return "P1";
	case PTP_REC_P2:
		return "P2";
	case PTP_REC_S1:
		return "S1";
	case PTP_REC_COUNT:
	default:
		return "?";
	}
}

/* -------------------------------------------------- foreign-master table -- */

/*
 * A peer's own announce interval governs its windows: a master announcing every
 * 8 s must not be pruned on our 2 s cadence. An out-of-range logMessageInterval
 * — 0x7F "unspecified", or an absurd exponent — falls back to our own interval
 * rather than producing a multi-minute timeout.
 */
static uint32_t foreign_interval_ms(const ptp_foreign_policy_t *pol,
				    const ptp_foreign_t *f)
{
	if ((f->log_announce_interval < PTP_LOG_INTERVAL_MIN) ||
	    (f->log_announce_interval > PTP_LOG_INTERVAL_MAX)) {
		return pol->default_interval_ms;
	}
	return ptp_log_interval_ms(f->log_announce_interval);
}

void ptp_foreign_init(ptp_foreign_tbl_t *t)
{
	if (t == NULL) {
		return;
	}
	memset(t, 0, sizeof(*t));
}

void ptp_foreign_clear(ptp_foreign_tbl_t *t)
{
	size_t i;

	if (t == NULL) {
		return;
	}
	for (i = 0U; i < (size_t)PTP_MAX_FOREIGN_MASTERS; i++) {
		t->rec[i].in_use = false;
		t->rec[i].qualified = false;
	}
}

void ptp_foreign_dataset_lp(const ptp_foreign_t *f, const ptp_port_id_t *receiver,
			    uint8_t local_priority, ptp_dataset_t *out)
{
	if ((f == NULL) || (receiver == NULL) || (out == NULL)) {
		return;
	}

	out->priority1 = f->announce.gm_priority1;
	out->quality = f->announce.gm_quality;
	out->priority2 = f->announce.gm_priority2;
	out->gm_identity = f->announce.gm_identity;
	out->steps_removed = f->announce.steps_removed;
	out->sender = f->source_port;
	out->receiver = *receiver;
	/*
	 * localPriority is the *receiving port's* attribute, not anything the
	 * Announce carried: G.8275.1 §6.3 makes it a local expression of which
	 * upstream this clock prefers, so it is the same value for every record
	 * on one port.
	 */
	out->local_priority = local_priority;
}

void ptp_foreign_dataset(const ptp_foreign_t *f, const ptp_port_id_t *receiver,
			 ptp_dataset_t *out)
{
	/* 128 is the profile-neutral default; the Default profile ignores it. */
	ptp_foreign_dataset_lp(f, receiver, 128U, out);
}

ptp_foreign_t *ptp_foreign_update(ptp_foreign_tbl_t *t,
				  const ptp_foreign_policy_t *pol,
				  const ptp_port_id_t *src,
				  const ptp_announce_t *a, uint16_t flags,
				  int8_t log_announce_interval, uint64_t now_ms)
{
	ptp_foreign_t *f = NULL;
	uint64_t window_ms;
	size_t i;

	if ((t == NULL) || (pol == NULL) || (src == NULL) || (a == NULL)) {
		return NULL;
	}

	for (i = 0U; i < (size_t)PTP_MAX_FOREIGN_MASTERS; i++) {
		if (t->rec[i].in_use &&
		    (ptp_port_id_cmp(&t->rec[i].source_port, src) == 0)) {
			f = &t->rec[i];
			break;
		}
	}

	if (f == NULL) {
		for (i = 0U; i < (size_t)PTP_MAX_FOREIGN_MASTERS; i++) {
			if (!t->rec[i].in_use) {
				f = &t->rec[i];
				break;
			}
		}
	}

	if (f == NULL) {
		/*
		 * Full. Evict the least recently heard record: it is the one
		 * closest to expiry anyway, and eviction by dataset rank would
		 * let a burst of good-looking Announces starve the incumbent.
		 */
		f = &t->rec[0];
		for (i = 1U; i < (size_t)PTP_MAX_FOREIGN_MASTERS; i++) {
			if (t->rec[i].last_rx_ms < f->last_rx_ms) {
				f = &t->rec[i];
			}
		}
		f->in_use = false;
		t->evicted++;
	}

	if (!f->in_use) {
		memset(f, 0, sizeof(*f));
		f->in_use = true;
		f->source_port = *src;
		t->added++;
	}

	f->announce = *a;
	f->flags = flags;
	f->log_announce_interval = log_announce_interval;

	window_ms = (uint64_t)PTP_FOREIGN_MASTER_TIME_WINDOW *
		    (uint64_t)foreign_interval_ms(pol, f);

	/*
	 * The window slides against the *previous* Announce, not against a fixed
	 * origin. A tumbling window would drop the count back to 1 every fourth
	 * interval even for a master announcing perfectly on cadence, briefly
	 * emptying Erbest and making this port flap out of PASSIVE and transmit
	 * as a second grandmaster alongside a healthy one.
	 *
	 * A new record has last_rx_ms == 0 and count == 0, so either branch
	 * leaves it at 1, which is what §9.3.2.4.5 wants for a first Announce.
	 */
	if ((now_ms < f->last_rx_ms) || ((now_ms - f->last_rx_ms) > window_ms)) {
		f->count = 1U;
	} else if (f->count < UINT16_MAX) {
		f->count++;
	} else {
		/* saturated; long since qualified */
	}
	f->last_rx_ms = now_ms;

	/*
	 * Qualification latches. It is given up only by losing the record
	 * entirely — ptp_foreign_prune() at the announce receipt timeout, or
	 * ptp_foreign_clear() — never by a momentary dip in the count. The
	 * receipt timeout (announceReceiptTimeout intervals) is shorter than
	 * this window (PTP_FOREIGN_MASTER_TIME_WINDOW intervals) under any sane
	 * policy, so in practice a gap long enough to reset the count has
	 * already dropped the record.
	 */
	if (f->count >= PTP_FOREIGN_MASTER_THRESHOLD) {
		f->qualified = true;
	}
	return f;
}

uint32_t ptp_foreign_prune(ptp_foreign_tbl_t *t, const ptp_foreign_policy_t *pol,
			   uint64_t now_ms)
{
	uint32_t dropped = 0U;
	size_t i;

	if ((t == NULL) || (pol == NULL)) {
		return 0U;
	}

	for (i = 0U; i < (size_t)PTP_MAX_FOREIGN_MASTERS; i++) {
		ptp_foreign_t *f = &t->rec[i];
		uint64_t timeout_ms;

		if (!f->in_use) {
			continue;
		}
		timeout_ms = (uint64_t)pol->receipt_timeout *
			     (uint64_t)foreign_interval_ms(pol, f);

		if ((now_ms > f->last_rx_ms) &&
		    ((now_ms - f->last_rx_ms) >= timeout_ms)) {
			f->in_use = false;
			f->qualified = false;
			dropped++;
		}
	}

	t->expired += dropped;
	return dropped;
}

const ptp_foreign_t *ptp_foreign_best_profile(const ptp_foreign_tbl_t *t,
					      const ptp_port_id_t *receiver,
					      uint8_t local_priority,
					      uint8_t profile,
					      ptp_dataset_t *out)
{
	const ptp_foreign_t *best = NULL;
	ptp_dataset_t best_ds;
	size_t i;

	if ((t == NULL) || (receiver == NULL)) {
		return NULL;
	}
	memset(&best_ds, 0, sizeof(best_ds));

	for (i = 0U; i < (size_t)PTP_MAX_FOREIGN_MASTERS; i++) {
		const ptp_foreign_t *f = &t->rec[i];
		ptp_dataset_t ds;

		if (!f->in_use || !f->qualified) {
			continue;
		}
		ptp_foreign_dataset_lp(f, receiver, local_priority, &ds);

		if ((best == NULL) ||
		    ptp_dscmp_a_wins(
			    ptp_bmca_compare_profile(&ds, &best_ds, profile))) {
			best = f;
			best_ds = ds;
		}
	}

	if ((best != NULL) && (out != NULL)) {
		*out = best_ds;
	}
	return best;
}

const ptp_foreign_t *ptp_foreign_best(const ptp_foreign_tbl_t *t,
				      const ptp_port_id_t *receiver,
				      ptp_dataset_t *out)
{
	return ptp_foreign_best_profile(t, receiver, 128U,
					(uint8_t)PTP_PROFILE_DEFAULT, out);
}
