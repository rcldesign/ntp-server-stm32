/*
 * STS1000 "Meridian" — core/ptp: IEEE 1588-2019 grandmaster engine.
 *
 * Platform-neutral C11, no allocation, all state in a caller-owned context
 * (ARCHITECTURE.md §4). Spec reference: ntp_server_software_spec.md §4.4, with
 * the service-quality inputs from §3.8.
 *
 * ---------------------------------------------------------------------------
 * Scope — deliberate, and narrower than IEEE 1588 as a whole
 * ---------------------------------------------------------------------------
 *
 * 1. **Four profiles: Default, G.8275.1, G.8275.2 and C37.238.** The parameters,
 *    permitted ranges, transport restrictions, clockClass ladders, alternate-BMCA
 *    localPriority tiebreak and mandatory Announce TLVs all live in
 *    ptp_profile.h; the engine reads a descriptor rather than branching on the
 *    profile enum. Two profile features are *not* implemented and are reported
 *    rather than hidden: C37.238's peer-delay mechanism (this engine is E2E
 *    only, note 4 below) and G.8275.2's unicast message negotiation. Selecting
 *    either of those two raises PTP_ALARM_PROFILE_UNSUPPORTED, and
 *    ptp_profile_deviation_text() names the gap. Default and G.8275.1 raise
 *    nothing.
 *
 * 2. **Ordinary clock, grandmaster-only.** This appliance is a GNSS-disciplined
 *    grandmaster; it never slaves to another clock. BMCA still runs in full, but
 *    a recommended state of S1 is executed as PASSIVE and raises
 *    PTP_ALARM_NOT_BEST_MASTER. That is a documented deviation from §9.2.5,
 *    which would have the port go to UNCALIBRATED/SLAVE. Deferring instead of
 *    slaving keeps the box from disciplining its OCXO to a network peer, which
 *    is exactly the failure the hardware exists to avoid.
 *
 * 3. **Two-step clock.** Sync carries an estimate and sets twoStepFlag; the
 *    precise egress timestamp arrives from the glue via ptp_on_sync_txts() and
 *    is published in a Follow_Up. One-step is a MAC capability question left
 *    open in the spec (§15), so two-step is what this engine implements.
 *
 * 4. **E2E delay mechanism only** (Delay_Req/Delay_Resp). Peer-delay messages
 *    are recognised, counted and ignored: a grandmaster on a P2P network still
 *    answers Announce/Sync, and Pdelay is the transparent clocks' business.
 *
 * 5. **PDU layer only.** UDP/IPv4 (Annex C), UDP/IPv6 (Annex D) and IEEE 802.3
 *    (Annex E) encapsulation is byte-level framing done by the glue sockets.
 *    This engine emits PDU bytes plus an addressing hint — event vs general
 *    port, and which multicast group — and consumes PDU bytes plus a hardware
 *    timestamp.
 *
 * 6. **No management/signaling responder.** Both are parsed far enough to be
 *    counted and dropped without error (spec §4.4: "PTP management messages
 *    gated").
 *
 * 7. **Annex-P integrity is available but opt-in.** ptp_port_set_icv() attaches
 *    a ptp_icv_ctx_t; from then on every transmitted PDU carries an
 *    AUTHENTICATION TLV and every received PDU is subject to the configured
 *    policy. Without it the engine behaves exactly as before. See ptp_icv.h.
 *
 * ---------------------------------------------------------------------------
 * Quality input
 * ---------------------------------------------------------------------------
 *
 * clockClass/clockAccuracy/offsetScaledLogVariance and the timePropertiesDS
 * flags come from the §3.8 quality block, never from anything this module
 * measures itself. ptp_quality_view_t below is the projection of that block
 * onto what PTP needs, and ptp_quality_view_from_block() is the one place the
 * two layouts meet — the engine itself never sees a quality_block_t, so the
 * §3.8 block can keep evolving without touching the state machine.
 */

#ifndef STS1000_CORE_PTP_PTP_H_
#define STS1000_CORE_PTP_PTP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ptp/ptp_icv.h"
#include "ptp/ptp_msg.h"
#include "ptp/ptp_profile.h"
#include "quality/quality.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------- constants -- */

/** Foreign-master records held per port (§9.3.2.4.1 leaves the bound open). */
#ifndef PTP_MAX_FOREIGN_MASTERS
#define PTP_MAX_FOREIGN_MASTERS 8
#endif

/** FOREIGN_MASTER_THRESHOLD (§9.3.2.4.5): Announces needed to qualify a master. */
#define PTP_FOREIGN_MASTER_THRESHOLD 2U

/** FOREIGN_MASTER_TIME_WINDOW (§9.3.2.4.5), in announce intervals. */
#define PTP_FOREIGN_MASTER_TIME_WINDOW 4U

/** stepsRemoved at or above which an Announce is ignored (§9.3.2.5 d). */
#define PTP_STEPS_REMOVED_MAX 255U

/* ---------------------------------------------------------------- alarms -- */

/**
 * BMCA has taken this port out of the master role: a better clock is on the
 * segment. Raised for recommended states P1, P2 and S1, cleared otherwise. S1
 * is the case where strict 1588 would slave and this appliance goes PASSIVE
 * instead; P1 and P2 are ordinary 1588 outcomes. Either way the operator wants
 * to know that the grandmaster is not grandmastering.
 */
#define PTP_ALARM_NOT_BEST_MASTER  0x00000001U
/** The port is in FAULTY: the glue reported a transport fault. */
#define PTP_ALARM_FAULTY           0x00000002U
/** The most recent transmit callback failed. */
#define PTP_ALARM_TX_ERROR         0x00000004U
/**
 * The configured profile has a *material* unimplemented feature.
 *
 * Raised at ptp_port_init() when the selected profile's descriptor carries a
 * deviation in PTP_DEV_MATERIAL — currently C37.238's peer-delay requirement and
 * G.8275.2's unicast message negotiation. Default and G.8275.1 do not raise it.
 * ptp_profile_deviation_text() renders the reason for the operator.
 *
 * The two deviations every profile shares here (two-step Sync, grandmaster-only)
 * deliberately do *not* raise it: an alarm that is always on tells nobody
 * anything. They are documented in the scope note at the top of this file.
 */
#define PTP_ALARM_PROFILE_UNSUPPORTED 0x00000008U
/** A received PDU failed the Annex-P integrity policy; see ptp_icv_counters(). */
#define PTP_ALARM_ICV_FAILED       0x00000010U

/* ------------------------------------------------------------ enums, cfg -- */

/*
 * ptp_transport_t lives in ptp_profile.h: a profile descriptor has to name the
 * transports it permits, and ptp_profile.h cannot include this header without a
 * cycle. It is re-exported here by that include.
 */

/**
 * Profile selector. The parameters of each live in ptp_profile.h; index into
 * ptp_profile_desc() with one of these.
 */
typedef enum {
	PTP_PROFILE_DEFAULT = 0,
	PTP_PROFILE_TELECOM_G8275_1,
	PTP_PROFILE_TELECOM_G8275_2,
	PTP_PROFILE_POWER_C37_238,
	PTP_PROFILE_COUNT,
} ptp_profile_t;

/**
 * Deviations that make PTP_ALARM_PROFILE_UNSUPPORTED worth raising.
 *
 * PTP_DEV_TWO_STEP_ONLY and PTP_DEV_GM_ONLY are excluded on purpose: they apply
 * to every profile this firmware offers, Default included, so including them
 * would light the alarm permanently.
 */
#define PTP_DEV_MATERIAL (PTP_DEV_E2E_ONLY | PTP_DEV_NO_UNICAST_NEG)

/**
 * clockClass to advertise once holdover leaves its specified window.
 *
 * IEEE 1588-2019 Table 4 offers two ladders down from class 7: alternative A
 * degrades to 52, alternative B to 187. The choice is not cosmetic. §7.6.2.5
 * reserves clockClass below 128 for clocks that shall never be a slave, so
 * alternative A (52) keeps the port in the M1/P1 branch of the state decision
 * and this appliance stays grandmaster-or-passive however far it degrades.
 * Alternative B (187) is above the line, which is what makes the S1 branch —
 * and therefore the deviation described at the top of this file — reachable at
 * all.
 */
typedef enum {
	PTP_DEGRADE_ALT_A = 0, /* class 52 */
	PTP_DEGRADE_ALT_B,     /* class 187 */
	PTP_DEGRADE_COUNT,
} ptp_degradation_t;

/** timeSource enumeration (§7.6.2.8, Table 6). */
typedef enum {
	PTP_TIME_SRC_ATOMIC_CLOCK    = 0x10,
	PTP_TIME_SRC_GNSS            = 0x20,
	PTP_TIME_SRC_TERRESTRIAL_RADIO = 0x30,
	PTP_TIME_SRC_SERIAL_TIME_CODE = 0x39,
	PTP_TIME_SRC_PTP             = 0x40,
	PTP_TIME_SRC_NTP             = 0x50,
	PTP_TIME_SRC_HAND_SET        = 0x60,
	PTP_TIME_SRC_OTHER           = 0x90,
	PTP_TIME_SRC_INTERNAL_OSC    = 0xA0,
} ptp_time_source_t;

/** clockAccuracy value meaning "unknown" (§7.6.2.6, Table 5). */
#define PTP_CLOCK_ACCURACY_UNKNOWN 0xFEU

/** offsetScaledLogVariance value meaning "not computed" (§7.6.3.5). */
#define PTP_OSLV_UNKNOWN 0xFFFFU

/**
 * Default offsetScaledLogVariance while locked, used when the quality view
 * carries no ADEV.
 *
 * 0x4E5D decodes as 2^((0x4E5D - 0x8000)/256) s² = 1.29e-15 s², i.e. an offset
 * standard deviation of about 36 ns — the conservative published figure for a
 * GNSS-disciplined grandmaster. When the view does carry an ADEV the value is
 * computed instead; see ptp_clock_quality_from_view().
 */
#define PTP_OSLV_DEFAULT_LOCKED 0x4E5DU

/** Runtime configuration. Populate with ptp_cfg_defaults(), then override. */
typedef struct {
	uint8_t domain;                     /* domainNumber, default 0 */
	uint8_t major_sdo_id;               /* octet 0 high nibble, default 0 */
	uint8_t minor_sdo_id;               /* octet 5, default 0 */
	uint8_t priority1;                  /* defaultDS.priority1, default 128 */
	uint8_t priority2;                  /* defaultDS.priority2, default 128 */
	uint16_t port_number;               /* portDS.portIdentity.portNumber, default 1 */
	int8_t log_announce_interval;       /* default 1 (2 s) */
	int8_t log_sync_interval;           /* default 0 (1 s) */
	int8_t log_min_delay_req_interval;  /* default 0 (1 s) */
	uint8_t announce_receipt_timeout;   /* N in N x announceInterval, default 3, min 2 */
	ptp_transport_t transport;          /* default UDP/IPv4 */
	ptp_profile_t profile;              /* default Default profile */
	ptp_degradation_t degradation;      /* default alternative A */
	uint16_t oslv_locked;               /* 0 forces the ADEV-derived value */

	/* ---- profile-specific; see ptp_profile.h ---------------------------- */

	/**
	 * defaultDS.localPriority (G.8275.1 §6.3), 1..255, default 128.
	 *
	 * Our own dataset's alternate-BMCA tiebreak. Read only by the telecom
	 * profiles' comparison.
	 */
	uint8_t local_priority;
	/**
	 * portDS.localPriority, 1..255, default 128.
	 *
	 * Assigned to every Announce *received* on this port. It is what makes
	 * localPriority useful on a single-port clock: an operator raises this
	 * above defaultDS.localPriority to defer to an upstream T-GM, or lowers
	 * it to keep grandmastering against one.
	 */
	uint8_t port_local_priority;
	/**
	 * portDS.notSlave (G.8275.1). Structural here: this appliance never
	 * enters SLAVE at all (scope note 2), which is strictly stronger. Kept
	 * so the profile default is visible in telemetry and configuration.
	 */
	bool not_slave;
	uint8_t l2_mac;      /* ptp_l2_mac_t; read by the glue for PTP_TRANSPORT_L2 */
	ptp_c37238_t c37238; /* Power-profile Announce TLV contents */
} ptp_cfg_t;

/** Fill @p cfg with the defaults named above (the Default profile's). */
void ptp_cfg_defaults(ptp_cfg_t *cfg);

/**
 * Overwrite every profile-derived field of @p cfg with @p profile's defaults.
 *
 * Sets profile, domain, priority1/2, localPriority (both), the three log
 * intervals, announceReceiptTimeout, transport, the L2 destination MAC and
 * notSlave. Leaves portNumber, majorSdoId/minorSdoId, degradation, oslv_locked
 * and the C37.238 payload alone — those are site settings, not profile
 * settings. Call it whenever an operator changes the profile; the resulting
 * configuration always passes ptp_cfg_validate().
 *
 * @retval 0        Applied.
 * @retval -EINVAL  @p cfg is NULL or @p profile is out of range.
 */
int ptp_cfg_apply_profile(ptp_cfg_t *cfg, uint8_t profile);

/**
 * Check @p cfg against the ranges the engine relies on, and against the
 * selected profile's own restrictions.
 *
 * Engine-level checks (all profiles): log intervals inside
 * [PTP_LOG_INTERVAL_MIN, PTP_LOG_INTERVAL_MAX], announceReceiptTimeout >= 2
 * (§7.7.3.1), portNumber != 0 (§7.5.2.3), majorSdoId <= 0x0F, and in-range
 * transport / profile / degradation / l2_mac selectors.
 *
 * Profile-level checks: for every profile *except* Default, domain, priority1,
 * priority2, localPriority, the three log intervals, announceReceiptTimeout and
 * the transport must sit inside the descriptor's ranges. The Default profile's
 * ranges are treated as advisory, because IEEE 1588-2019 Annex I.3 states them
 * as recommendations and the standard permits any value the field can hold —
 * whereas G.8275.1's ranges are normative and a value outside them is simply
 * not that profile.
 *
 * @retval 0        Usable.
 * @retval -EINVAL  An engine-level check failed.
 * @retval -ERANGE  A profile-level restriction was violated;
 *                  ptp_cfg_apply_profile() produces a conforming set.
 */
int ptp_cfg_validate(const ptp_cfg_t *cfg);

/* --------------------------------------------------------- quality input -- */

/** Discipline state, as far as clockClass selection is concerned (§3.6, §3.8). */
typedef enum {
	PTP_SYNC_FREERUN = 0,       /* not disciplined */
	PTP_SYNC_LOCKED,            /* locked to the primary reference */
	PTP_SYNC_HOLDOVER,          /* holdover, still inside the characterised spec */
	PTP_SYNC_HOLDOVER_EXCEEDED, /* holdover, spec exceeded */
	PTP_SYNC_COUNT,
} ptp_sync_state_t;

/**
 * How far ahead of a leap second the flags may be announced.
 *
 * §9.4: leap61/leap59 are asserted no more than 12 hours before the event. The
 * GNSS receiver knows about a leap months in advance, so the window matters —
 * without it every Announce for a whole quarter would carry the flag.
 */
#define PTP_LEAP_ANNOUNCE_WINDOW_S 43200U

/**
 * currentUtcOffset advertised before the receiver has reported a real one.
 *
 * TAI - UTC has stood at 37 s since the 2017-01-01 leap second. A block that
 * has not yet heard from the GNSS receiver carries 0, which on the wire claims
 * TAI == UTC: wrong by 37 s and plausible enough to be believed. Announcing the
 * standing value with currentUtcOffsetValid clear is the honest fallback. Update
 * this constant if IERS ever announces another leap second.
 */
#define PTP_DEFAULT_UTC_OFFSET 37

/** The §3.8 quality block projected onto what PTP needs. */
typedef struct {
	ptp_sync_state_t sync_state;
	int16_t utc_offset;        /* currentUtcOffset: TAI - UTC, seconds */
	bool utc_offset_valid;
	bool leap61;               /* leap second to be inserted at end of day */
	bool leap59;               /* leap second to be deleted at end of day */
	bool time_traceable;
	bool freq_traceable;
	uint8_t time_source;       /* ptp_time_source_t */
	uint64_t est_accuracy_ns;  /* estimated |time error|, ns; 0 = unknown */
	uint64_t adev_tau1_e18;    /* ADEV at tau = 1 s, scaled by 1e18; 0 = unknown */
} ptp_quality_view_t;

/**
 * Project a §3.8 quality block onto the PTP view.
 *
 * This is the only place the two layouts meet. The mapping, with its reasoning:
 *
 * | PTP field        | Source |
 * |---|---|
 * | sync_state       | LOCKED and not holdover -> LOCKED; holdover (flag, state, or the rate-limited RECOVERING pull-in) -> HOLDOVER, or HOLDOVER_EXCEEDED once QUALITY_FLAG_DEMOTED says policy is past; anything else -> FREERUN |
 * | utc_offset       | `leap_current_s` (TAI - UTC) |
 * | leap61/leap59    | `leap_pending` sign, but only inside PTP_LEAP_ANNOUNCE_WINDOW_S of `leap_at_tai_s` |
 * | time_traceable   | locked or in holdover: traceability survives holdover, which is what holdover is for |
 * | freq_traceable   | that, or an atomic/house reference on the mux even while free-running |
 * | time_source      | GNSS while the receiver's time is locked; else atomic clock on the Rb, "other" on the external house reference, internal oscillator otherwise |
 * | est_accuracy_ns  | `holdover_est_err_ns` in holdover, else the receiver's `gnss_tacc_ns`, else the rolling PPS sigma |
 * | adev_tau1_e18    | `adev_1s` scaled by 1e18, clamped; 0 when not yet characterised |
 *
 * No libm: the float fields are handled with comparisons and casts only, and a
 * NaN or infinity in the block degrades to "unknown" rather than to a trap.
 *
 * @param b           The snapshot to project. Not retained.
 * @param now_tai_s   Current TAI seconds since the PTP epoch, for the leap
 *                    announcement window.
 * @param out         Receives the view.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p b or @p out is NULL.
 */
int ptp_quality_view_from_block(const quality_block_t *b, uint64_t now_tai_s,
				ptp_quality_view_t *out);

/**
 * Map a quality view onto grandmasterClockQuality.
 *
 * clockClass (Table 4): locked -> 6, holdover-in-spec -> 7, holdover-exceeded
 * and free-run -> 52 or 187 per cfg->degradation.
 *
 * clockAccuracy (Table 5): the ladder from 1 ns (0x1D) to ">10 s" (0x31), with
 * 0xFE for an unknown estimate. The finer 2019 rungs (0x17..0x1C, 1 ps..250 ps)
 * are unreachable from a nanosecond-resolution estimate and are not in the
 * ladder; the 2.5·10^k rungs are entered at their integer-nanosecond floor.
 *
 * offsetScaledLogVariance (§7.6.3.5): 256·log2(PTPVAR) + 0x8000, where PTPVAR
 * is the offset variance in s². This module estimates it from the Allan
 * deviation as PTPVAR = (ADEV(1 s) · tau)², tau being the sync interval — the
 * white-FM relation x(tau) = y·tau, which is conservative by sqrt(3) against
 * the exact TDEV identity TDEV(tau) = tau·MDEV(tau)/sqrt(3). With no ADEV the
 * value is cfg->oslv_locked while locked or in holdover, and 0xFFFF otherwise.
 *
 * Pure function; @p cfg and @p q are not retained.
 *
 * @retval 0        Success.
 * @retval -EINVAL  A pointer argument is NULL.
 */
int ptp_clock_quality_from_view(const ptp_cfg_t *cfg, const ptp_quality_view_t *q,
				ptp_clock_quality_t *out);

/**
 * Build the Announce octet-1 flagField bits from a quality view.
 *
 * Only Announce carries the timePropertiesDS flags (Table 37 scopes leap61,
 * leap59, currentUtcOffsetValid, ptpTimescale, timeTraceable and
 * frequencyTraceable to Announce), so this is not applied to Sync or
 * Delay_Resp. ptpTimescale is always set: the appliance serves PTP (TAI)
 * timescale, never ARB.
 */
uint16_t ptp_flags_from_view(const ptp_quality_view_t *q);

/* ------------------------------------------------------------------ BMCA -- */

/** Result of the dataset comparison algorithm, §9.3.4. */
typedef enum {
	PTP_DSCMP_ERROR_2       = -4, /* both datasets came from the same port */
	PTP_DSCMP_ERROR_1       = -3, /* a clock is announcing its own dataset back */
	PTP_DSCMP_B_BETTER_TOPO = -2,
	PTP_DSCMP_B_BETTER      = -1,
	PTP_DSCMP_A_BETTER      =  1,
	PTP_DSCMP_A_BETTER_TOPO =  2,
} ptp_dscmp_t;

/** True when the comparison picked A, by either route. */
static inline bool ptp_dscmp_a_wins(ptp_dscmp_t r)
{
	return (r == PTP_DSCMP_A_BETTER) || (r == PTP_DSCMP_A_BETTER_TOPO);
}

/** True when the comparison picked B, by either route. */
static inline bool ptp_dscmp_b_wins(ptp_dscmp_t r)
{
	return (r == PTP_DSCMP_B_BETTER) || (r == PTP_DSCMP_B_BETTER_TOPO);
}

/**
 * A dataset as the comparison algorithm sees it (§9.3.4): the grandmaster
 * attributes an Announce advertises, plus the topology pair used by part 2.
 */
typedef struct {
	uint8_t priority1;
	ptp_clock_quality_t quality;
	uint8_t priority2;
	ptp_clock_id_t gm_identity;
	uint16_t steps_removed;
	ptp_port_id_t sender;   /* sourcePortIdentity of the Announce */
	ptp_port_id_t receiver; /* the port that received it */
	/**
	 * localPriority attached to this dataset, 1..255.
	 *
	 * Read only by the telecom profiles' comparison. For our own dataset it
	 * is cfg.local_priority; for a foreign one it is cfg.port_local_priority
	 * of the port that received the Announce. Ignored entirely by the
	 * Default and Power profiles, which is why a zero here is harmless.
	 */
	uint8_t local_priority;
} ptp_dataset_t;

/**
 * Dataset comparison algorithm, IEEE 1588-2019 §9.3.4 (Figures 34 and 35).
 *
 * Part 1 runs when the two grandmasterIdentities differ and orders on
 * priority1, clockClass, clockAccuracy, offsetScaledLogVariance, priority2 and
 * finally grandmasterIdentity — lower always wins, and the identity tiebreak
 * makes the ordering total. Part 2 runs when the grandmasters are the same and
 * orders on stepsRemoved (with the standard's ±1 dead band), then on the sender
 * and receiver port identities.
 *
 * Errors are reported rather than folded into a winner: ERROR_1 is a receiver
 * hearing its own dataset back, ERROR_2 is two datasets from one port. Use
 * ptp_dscmp_a_wins()/ptp_dscmp_b_wins() rather than the sign.
 *
 * Equivalent to ptp_bmca_compare_profile() with PTP_PROFILE_DEFAULT.
 *
 * @param a  Dataset A; must not be NULL.
 * @param b  Dataset B; must not be NULL.
 */
ptp_dscmp_t ptp_bmca_compare(const ptp_dataset_t *a, const ptp_dataset_t *b);

/**
 * Dataset comparison under @p profile's rules.
 *
 * The telecom profiles (G.8275.1, and G.8275.2 which adopts its alternate BMCA)
 * change part 1 in exactly two ways, per G.8275.1 §6.3:
 *
 *   - **priority1 is not compared.** The profile fixes it at 128 for every
 *     conforming clock, so comparing it can only produce a wrong answer against
 *     a misconfigured peer.
 *   - **localPriority is compared after priority2**, lower winning, before the
 *     grandmasterIdentity tiebreak. It is a *local* attribute — not carried on
 *     the wire — so it expresses this clock's own preference between otherwise
 *     equally good grandmasters.
 *
 * Part 2 (same grandmasterIdentity: the stepsRemoved and topology ordering) is
 * unchanged from IEEE 1588. Two datasets that reach part 2 describe the same
 * grandmaster reached by different paths, and a local preference between paths
 * is what portDS.localPriority already expresses through part 1.
 *
 * The Default and Power profiles use the unmodified §9.3.4 algorithm, so this
 * function is byte-for-byte ptp_bmca_compare() for them.
 *
 * @param a        Dataset A; must not be NULL.
 * @param b        Dataset B; must not be NULL.
 * @param profile  ptp_profile_t. Out-of-range behaves as Default.
 */
ptp_dscmp_t ptp_bmca_compare_profile(const ptp_dataset_t *a,
				     const ptp_dataset_t *b, uint8_t profile);

/** Recommended state, §9.3.3 (Figure 33). */
typedef enum {
	PTP_REC_LISTENING = 0, /* no Announce yet and already LISTENING: stay put */
	PTP_REC_M1,            /* class 1..127, we are best */
	PTP_REC_M2,            /* class >127, we are best */
	PTP_REC_M3,            /* best is on another port and not better by topology */
	PTP_REC_P1,            /* class 1..127, we are not best */
	PTP_REC_P2,            /* best is on another port and better by topology */
	PTP_REC_S1,            /* best arrived on this port; 1588 would say SLAVE */
	PTP_REC_COUNT,
} ptp_recommended_t;

/** Short name of @p r for logs. */
const char *ptp_recommended_name(ptp_recommended_t r);

/**
 * State decision algorithm, IEEE 1588-2019 §9.3.3.
 *
 * @param d0                Our own defaultDS-derived dataset. Required.
 * @param erbest            Best qualified Announce on this port, or NULL.
 * @param ebest             Best across all ports, or NULL. For a single-port
 *                          ordinary clock this is @p erbest.
 * @param ebest_on_this_port True when @p ebest arrived on this port.
 * @param listening         True when the port is in LISTENING and its announce
 *                          receipt timeout has *not* expired. With no @p erbest
 *                          this holds the port in LISTENING (Figure 33's first
 *                          decision); passing false is how the caller reports
 *                          ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES.
 *
 * @return The recommended state; PTP_REC_LISTENING when the port should stay.
 */
ptp_recommended_t ptp_bmca_state_decision(const ptp_dataset_t *d0,
					  const ptp_dataset_t *erbest,
					  const ptp_dataset_t *ebest,
					  bool ebest_on_this_port,
					  bool listening);

/**
 * State decision algorithm using @p profile's comparison rules.
 *
 * Identical to ptp_bmca_state_decision() except that every dataset comparison
 * inside it goes through ptp_bmca_compare_profile(). The state decision itself
 * (§9.3.3 Figure 33) is not profile-specific: G.8275.1 changes what "better"
 * means, not what to do about it.
 */
ptp_recommended_t ptp_bmca_state_decision_profile(const ptp_dataset_t *d0,
						  const ptp_dataset_t *erbest,
						  const ptp_dataset_t *ebest,
						  bool ebest_on_this_port,
						  bool listening,
						  uint8_t profile);

/* -------------------------------------------------- foreign-master table -- */

/** One foreign-master record, §9.3.2.4. */
typedef struct {
	bool in_use;
	/**
	 * Latched once @ref count reaches PTP_FOREIGN_MASTER_THRESHOLD. Given up
	 * only by losing the record — ptp_foreign_prune() or ptp_foreign_clear()
	 * — never by a dip in the count, so a master announcing on cadence stays
	 * a candidate instead of blinking out of Erbest.
	 */
	bool qualified;
	ptp_port_id_t source_port;   /* sourcePortIdentity of its Announces */
	ptp_announce_t announce;     /* the most recent Announce body */
	uint16_t flags;              /* its most recent flagField */
	int8_t log_announce_interval;/* its advertised logMessageInterval */
	/** Consecutive Announces no more than one window apart; saturates. */
	uint16_t count;
	uint64_t last_rx_ms;
} ptp_foreign_t;

/** Bounded per-port foreign-master table. */
typedef struct {
	ptp_foreign_t rec[PTP_MAX_FOREIGN_MASTERS];
	uint32_t added;
	uint32_t evicted;
	uint32_t expired;
} ptp_foreign_tbl_t;

/** How the table derives its per-record windows. */
typedef struct {
	uint8_t receipt_timeout;      /* announceReceiptTimeout, N */
	uint32_t default_interval_ms; /* used when a peer's logInterval is out of range */
} ptp_foreign_policy_t;

/** Empty the table and zero its counters. */
void ptp_foreign_init(ptp_foreign_tbl_t *t);

/**
 * Drop every record but keep the lifetime counters.
 *
 * Used when the segment's state stops being trustworthy — a transport fault —
 * where forgetting the peers is right but rewinding the counters is not.
 */
void ptp_foreign_clear(ptp_foreign_tbl_t *t);

/**
 * Record an Announce, creating or refreshing the sender's entry.
 *
 * Qualification follows §9.3.2.4.5: a record becomes qualified once
 * PTP_FOREIGN_MASTER_THRESHOLD Announces arrive no more than
 * PTP_FOREIGN_MASTER_TIME_WINDOW announce intervals apart. The window slides
 * against the previous Announce, so an on-cadence master accumulates without
 * ever falling back; a gap longer than the window restarts the count, and
 * qualification itself is surrendered only by prune or clear. A full table
 * evicts its least recently heard record, which is also the one closest to
 * expiry.
 *
 * @return The record, or NULL on a NULL argument.
 */
ptp_foreign_t *ptp_foreign_update(ptp_foreign_tbl_t *t,
				  const ptp_foreign_policy_t *pol,
				  const ptp_port_id_t *src,
				  const ptp_announce_t *a, uint16_t flags,
				  int8_t log_announce_interval, uint64_t now_ms);

/**
 * Drop records silent for announceReceiptTimeout of their own announce
 * intervals.
 *
 * @return How many records were dropped.
 */
uint32_t ptp_foreign_prune(ptp_foreign_tbl_t *t, const ptp_foreign_policy_t *pol,
			   uint64_t now_ms);

/**
 * Erbest: the best qualified record, as a comparison dataset.
 *
 * @param receiver  Our port identity, for the part-2 topology tiebreak.
 * @param out       Receives the dataset when a qualified record exists.
 * @return The winning record, or NULL when none is qualified.
 */
const ptp_foreign_t *ptp_foreign_best(const ptp_foreign_tbl_t *t,
				      const ptp_port_id_t *receiver,
				      ptp_dataset_t *out);

/**
 * Erbest under @p profile's comparison rules, with @p local_priority attached to
 * every candidate.
 *
 * @param local_priority  portDS.localPriority of the receiving port. Read only
 *                        by the telecom profiles.
 */
const ptp_foreign_t *ptp_foreign_best_profile(const ptp_foreign_tbl_t *t,
					      const ptp_port_id_t *receiver,
					      uint8_t local_priority,
					      uint8_t profile,
					      ptp_dataset_t *out);

/**
 * Build the comparison dataset of a foreign record.
 *
 * localPriority is set to the profile-neutral default 128; use
 * ptp_foreign_dataset_lp() when it matters.
 */
void ptp_foreign_dataset(const ptp_foreign_t *f, const ptp_port_id_t *receiver,
			 ptp_dataset_t *out);

/** As ptp_foreign_dataset(), with an explicit portDS.localPriority. */
void ptp_foreign_dataset_lp(const ptp_foreign_t *f, const ptp_port_id_t *receiver,
			    uint8_t local_priority, ptp_dataset_t *out);

/* ----------------------------------------------------------- port engine -- */

/** portState, §8.2.15.3.1 / Table 20. Only five are ever occupied here. */
typedef enum {
	PTP_PS_INITIALIZING = 1,
	PTP_PS_FAULTY       = 2,
	PTP_PS_DISABLED     = 3, /* declared for completeness; never entered */
	PTP_PS_LISTENING    = 4,
	PTP_PS_PRE_MASTER   = 5, /* unreachable: see the M3 note in ptp_port.c */
	PTP_PS_MASTER       = 6,
	PTP_PS_PASSIVE      = 7,
	PTP_PS_UNCALIBRATED = 8, /* never entered: grandmaster-only */
	PTP_PS_SLAVE        = 9, /* never entered: grandmaster-only */
} ptp_port_state_t;

/** Short name of @p s for logs. */
const char *ptp_port_state_name(ptp_port_state_t s);

/** Which UDP port (or its L2 equivalent) a PDU belongs on. */
typedef enum {
	PTP_PORT_EVENT = 0,   /* UDP 319: hardware-timestamped */
	PTP_PORT_GENERAL,     /* UDP 320 */
} ptp_port_kind_t;

/** Which destination the glue should use. */
typedef enum {
	PTP_ADDR_PRIMARY = 0,    /* 224.0.1.129 / ff0e::181 / 01-1B-19-00-00-00 */
	PTP_ADDR_PDELAY,         /* 224.0.0.107 / ff02::6b / 01-80-C2-00-00-0E */
	PTP_ADDR_UNICAST_PEER,   /* back to the requester; `peer` is valid */
} ptp_addr_hint_t;

/**
 * One outbound PDU.
 *
 * @p buf points into a scratch buffer owned by the context and is valid for the
 * duration of the callback: the glue must copy it or hand it to the stack
 * synchronously.
 *
 * Re-entrancy: the engine never reuses the buffer backing an in-flight
 * descriptor, so a driver that reads its own egress timestamp synchronously may
 * call ptp_on_sync_txts() from inside the Sync callback — the Follow_Up it
 * emits is built in separate storage and the Sync bytes under @p buf do not
 * move. That nested Follow_Up is transmitted *before* the outer Sync callback
 * returns; a driver that queues rather than sends must account for the
 * ordering, and one that then reports the Sync as failed leaves the Follow_Up
 * orphaned on the wire (counted in @ref ptp_counters_t::followup_orphaned).
 */
typedef struct {
	const uint8_t *buf;
	size_t len;
	uint8_t msg_type;         /* ptp_msg_type_t */
	uint16_t seq;
	ptp_port_kind_t port_kind;
	ptp_addr_hint_t addr;
	ptp_port_id_t peer;       /* valid when addr == PTP_ADDR_UNICAST_PEER */
	ptp_transport_t transport;/* echoed from the configuration */
} ptp_tx_desc_t;

/** The engine's window onto the platform. */
typedef struct {
	/**
	 * Transmit one PDU. Return 0 on success, negative errno on failure; a
	 * failure raises PTP_ALARM_TX_ERROR and bumps the tx_errors counter but
	 * never changes the port state — link loss is the glue's fault input.
	 */
	int (*tx)(void *ctx, const ptp_tx_desc_t *d);
	/**
	 * Current TAI time in nanoseconds since the PTP epoch, for the estimate
	 * a two-step Sync carries. Return 0 on success; on failure the engine
	 * sends a zero originTimestamp, which §11.4.3 explicitly permits.
	 */
	int (*tai_ns)(void *ctx, uint64_t *out_ns);
	void *ctx;
} ptp_port_ops_t;

/**
 * Per-message-type and engine counters.
 *
 * @p tx counts messages the transmit callback accepted; a rejected one lands in
 * @p tx_errors instead, so tx + tx_errors is what was attempted. @p rx counts
 * messages that passed the header, domain and self-address filters — the three
 * rejection counters below account for the rest.
 */
typedef struct {
	uint32_t tx[PTP_MSG_TYPE_COUNT];
	uint32_t rx[PTP_MSG_TYPE_COUNT];
	uint32_t rx_dropped;       /* malformed, or a body that failed to decode */
	uint32_t rx_foreign_domain;/* wrong domainNumber / majorSdoId / versionPTP */
	uint32_t rx_self;          /* our own multicast coming back to us */
	uint32_t rx_ignored;       /* well formed, nothing for a GM to do */
	uint32_t bmca_runs;
	uint32_t bmca_decisions;   /* runs whose recommended state changed */
	uint32_t state_changes;
	uint32_t announce_timeouts;
	uint32_t foreign_added;
	uint32_t foreign_evicted;
	uint32_t foreign_expired;
	uint32_t followup_missed;  /* a Sync egress timestamp never arrived */
	/** A synchronously released Follow_Up whose Sync then failed to send. */
	uint32_t followup_orphaned;
	uint32_t txts_unmatched;   /* ptp_on_sync_txts() for an unknown sequenceId */
	uint32_t tx_errors;
	/** Event message dropped for want of a hardware ingress timestamp. */
	uint32_t rx_no_timestamp;
	/** Message dropped by the Annex-P policy; ptp_icv_counters() says why. */
	uint32_t rx_icv_rejected;
	/** Announce built without its mandatory profile TLV (no room). */
	uint32_t tx_profile_tlv_errors;
} ptp_counters_t;

/** Engine state. Caller-owned; initialise with ptp_port_init(). */
typedef struct {
	ptp_cfg_t cfg;
	ptp_port_ops_t ops;
	ptp_foreign_policy_t policy;

	ptp_clock_id_t clock_id;    /* defaultDS.clockIdentity */
	ptp_port_id_t port_id;      /* portDS.portIdentity */
	ptp_quality_view_t quality;

	ptp_port_state_t state;
	ptp_recommended_t last_rec;
	uint32_t alarms;

	uint32_t announce_ms;       /* announceInterval, ms */
	uint32_t sync_ms;           /* syncInterval, ms */
	uint32_t arto_ms;           /* announceReceiptTimeoutInterval, ms */

	uint64_t announce_next_ms;
	uint64_t sync_next_ms;
	uint64_t announce_rx_ms;    /* last qualified Announce, or LISTENING entry */
	bool arto_latched;          /* the current timeout has already been counted */

	uint16_t announce_seq;
	uint16_t sync_seq;
	bool sync_pending;          /* a Sync is awaiting its egress timestamp */
	uint16_t sync_pending_seq;

	ptp_foreign_tbl_t foreign;

	/**
	 * Annex-P integrity, or NULL.
	 *
	 * A pointer rather than an embedded context: ptp_icv_ctx_t carries a key
	 * table and a hash scratch, and a deployment that does not use Annex P
	 * should not pay for them. Attach with ptp_port_set_icv().
	 */
	ptp_icv_ctx_t *icv;

	ptp_counters_t counters;
	/** Scratch for the message being transmitted. */
	uint8_t txbuf[PTP_MSG_MAX_LEN];
	/**
	 * Separate scratch for Follow_Up.
	 *
	 * ptp_on_sync_txts() may legitimately be called from inside the Sync
	 * transmit callback, while the Sync still occupies @ref txbuf and the
	 * glue still holds a pointer into it. Encoding the Follow_Up anywhere
	 * else keeps that pointer honest.
	 *
	 * Full PTP_MSG_MAX_LEN, not PTP_TSMSG_LEN: a Follow_Up carries the
	 * AUTHENTICATION TLV like every other message, and a 44-octet buffer
	 * would silently refuse to sign it.
	 */
	uint8_t fubuf[PTP_MSG_MAX_LEN];
} ptp_port_ctx_t;

/**
 * Initialise the engine into PTP_PS_INITIALIZING.
 *
 * The clockIdentity is derived from @p mac per §7.5.2.2.2. The quality view
 * starts at free-run; call ptp_port_set_quality() before enabling the port if a
 * better answer is already available.
 *
 * @param mac  Six-octet EUI-48 of the Ethernet interface.
 *
 * @retval 0        Success.
 * @retval -EINVAL  A pointer argument is NULL, or @p cfg fails validation.
 */
int ptp_port_init(ptp_port_ctx_t *c, const ptp_cfg_t *cfg, const uint8_t *mac,
		  const ptp_port_ops_t *ops);

/**
 * Leave INITIALIZING for LISTENING and start the announce receipt timeout.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p c is NULL.
 * @retval -EPERM   The port is not in INITIALIZING or FAULTY.
 */
int ptp_port_enable(ptp_port_ctx_t *c, uint64_t now_ms);

/** Replace the quality view. It takes effect on the next Announce. */
int ptp_port_set_quality(ptp_port_ctx_t *c, const ptp_quality_view_t *q);

/**
 * Attach (or with NULL, detach) Annex-P integrity.
 *
 * While attached, every PDU this engine transmits carries an AUTHENTICATION TLV
 * (when the ICV context has a transmit key) and every PDU it receives is put
 * through ptp_icv_verify() before it is acted on. A rejection increments
 * @ref ptp_counters_t::rx_icv_rejected, raises PTP_ALARM_ICV_FAILED, and drops
 * the message.
 *
 * @param icv  Caller-owned context, initialised with ptp_icv_init(). It must
 *             outlive the port context. NULL detaches and clears the alarm.
 *
 * @retval 0        Attached.
 * @retval -EINVAL  @p c is NULL.
 */
int ptp_port_set_icv(ptp_port_ctx_t *c, ptp_icv_ctx_t *icv);

/**
 * Drive timeouts, the BMCA and the transmit schedulers.
 *
 * Call at least as often as the shortest configured interval; the `ptp` thread
 * calls it on its event loop. At most one Announce and one Sync are emitted per
 * call, and the schedulers advance by whole intervals so cadence does not drift.
 *
 * @retval 0        Success (including "nothing to do").
 * @retval -EINVAL  @p c is NULL.
 */
int ptp_port_step(ptp_port_ctx_t *c, uint64_t now_ms);

/**
 * Feed one received PDU.
 *
 * @param rx_tai_ns  Hardware ingress timestamp, TAI nanoseconds. Mandatory and
 *                   non-zero for the event message types (messageType 0x0..0x3,
 *                   see ptp_msg_is_event()); an event message without one is
 *                   rejected rather than answered with a zero receiveTimestamp.
 *                   Pass 0 for general messages, where it is not read.
 *
 * @retval 0        Consumed, including deliberately ignored messages.
 * @retval -EINVAL  @p c or @p buf is NULL.
 * @retval -EBADMSG Malformed or truncated.
 * @retval -EPROTO  Wrong versionPTP, domainNumber or majorSdoId.
 * @retval -ENODATA An event message arrived without a hardware ingress
 *                  timestamp; counted in rx_no_timestamp.
 * @retval -EPERM   The port is not receiving (INITIALIZING or FAULTY), or an
 *                  attached ICV context has no key for the message's
 *                  association.
 * @retval -EACCES  Annex-P integrity check failed, or was absent under the
 *                  REQUIRE policy. Counted in rx_icv_rejected.
 * @retval -EIO     The crypto port failed while verifying; the message is
 *                  dropped rather than accepted unverified.
 */
int ptp_port_rx(ptp_port_ctx_t *c, const uint8_t *buf, size_t len,
		uint64_t rx_tai_ns, uint64_t now_ms);

/**
 * Report the hardware egress timestamp of a transmitted Sync, which releases
 * its Follow_Up.
 *
 * @param seq     sequenceId of the Sync, as handed to the tx callback.
 * @param tai_ns  Egress timestamp, TAI nanoseconds.
 *
 * @retval 0        Follow_Up emitted.
 * @retval -EINVAL  @p c is NULL.
 * @retval -ENOENT  No Sync is outstanding, or @p seq does not match the one
 *                  that is; the counter txts_unmatched records this.
 */
int ptp_on_sync_txts(ptp_port_ctx_t *c, uint16_t seq, uint64_t tai_ns);

/**
 * Enter FAULTY: stop transmitting, drop received traffic, forget the foreign
 * masters. The glue calls this on a transport fault.
 */
int ptp_port_fault(ptp_port_ctx_t *c, uint64_t now_ms);

/** FAULT_CLEARED: leave FAULTY for LISTENING and re-run the election. */
int ptp_port_fault_reset(ptp_port_ctx_t *c, uint64_t now_ms);

/** Current port state; PTP_PS_INITIALIZING for a NULL context. */
ptp_port_state_t ptp_port_state(const ptp_port_ctx_t *c);

/**
 * Current alarm bits, PTP_ALARM_*; 0 for a NULL context.
 *
 * These are live, not latched: NOT_BEST_MASTER follows the BMCA outcome on every
 * run, FAULTY clears on ptp_port_fault_reset(), and TX_ERROR clears on the next
 * successful transmit. PTP_ALARM_ICV_FAILED is latched until the next PDU passes
 * the policy, so a single forged frame is visible rather than blinking past.
 * Only PTP_ALARM_PROFILE_UNSUPPORTED is sticky — it is set once at
 * ptp_port_init() and lasts the life of the context. A consumer that needs edge
 * semantics (a trap, a log line) must remember the previous value.
 */
uint32_t ptp_port_alarms(const ptp_port_ctx_t *c);

/** Counter block, or NULL for a NULL context. */
const ptp_counters_t *ptp_port_counters(const ptp_port_ctx_t *c);

/** Our own dataset, as the BMCA compares it. Exposed for telemetry and tests. */
int ptp_port_dataset(const ptp_port_ctx_t *c, ptp_dataset_t *out);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_PTP_PTP_H_ */
