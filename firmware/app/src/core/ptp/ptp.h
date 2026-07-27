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
 * 1. **Default profile only** (IEEE 1588-2019 Annex I.3, "Default PTP profile,
 *    E2E"). ptp_cfg_t carries the knobs the Telecom (G.8275.1/.2) and Power
 *    (C37.238) profiles need — domain, priority1/2, log intervals, transport —
 *    but none of their profile-specific *behaviours* (alternate BMCA,
 *    localPriority, forced port states, profile TLVs) are implemented.
 *    Selecting one raises PTP_ALARM_PROFILE_UNSUPPORTED at init and the engine
 *    keeps Default-profile behaviour. ARCHITECTURE.md §5 lists these as
 *    deferred.
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
 *    gated"). Annex-P integrity (ICV TLV) is deferred.
 *
 * ---------------------------------------------------------------------------
 * Quality input
 * ---------------------------------------------------------------------------
 *
 * clockClass/clockAccuracy/offsetScaledLogVariance and the timePropertiesDS
 * flags come from the §3.8 quality block, never from anything this module
 * measures itself. core/quality does not exist yet, so ptp_quality_view_t below
 * is the minimal projection this module needs. TODO(wave-3): once
 * core/quality/quality.h lands, replace this struct with an adapter that
 * projects quality_snapshot_t onto it — the field set was chosen to be a strict
 * subset of the §3.8 block.
 */

#ifndef STS1000_CORE_PTP_PTP_H_
#define STS1000_CORE_PTP_PTP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ptp/ptp_msg.h"

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

/** Widest log message interval accepted, either sign. 2^-7 s .. 2^7 s. */
#define PTP_LOG_INTERVAL_MIN (-7)
#define PTP_LOG_INTERVAL_MAX (7)

/* ---------------------------------------------------------------- alarms -- */

/** BMCA says another clock is the better master; we defer instead of slaving. */
#define PTP_ALARM_NOT_BEST_MASTER  0x00000001U
/** The port is in FAULTY: the glue reported a transport fault. */
#define PTP_ALARM_FAULTY           0x00000002U
/** The most recent transmit callback failed. */
#define PTP_ALARM_TX_ERROR         0x00000004U
/** A non-Default profile is configured; Default-profile behaviour is in force. */
#define PTP_ALARM_PROFILE_UNSUPPORTED 0x00000008U

/* ------------------------------------------------------------ enums, cfg -- */

/** Transport encapsulation selector; the glue owns the framing. */
typedef enum {
	PTP_TRANSPORT_UDP_IPV4 = 0, /* IEEE 1588-2019 Annex C */
	PTP_TRANSPORT_UDP_IPV6,     /* Annex D */
	PTP_TRANSPORT_L2,           /* Annex E, EtherType 0x88F7 */
	PTP_TRANSPORT_COUNT,
} ptp_transport_t;

/** Profile selector. Only PTP_PROFILE_DEFAULT changes behaviour; see the scope note. */
typedef enum {
	PTP_PROFILE_DEFAULT = 0,
	PTP_PROFILE_TELECOM_G8275_1,
	PTP_PROFILE_TELECOM_G8275_2,
	PTP_PROFILE_POWER_C37_238,
	PTP_PROFILE_COUNT,
} ptp_profile_t;

/**
 * clockClass to advertise once holdover leaves its specified window.
 *
 * IEEE 1588-2019 Table 4 offers two ladders down from class 7: alternative A
 * degrades to 52, alternative B to 187. A above 127 is the "never a slave"
 * range, so alternative A keeps this appliance grandmaster-only even while
 * degraded; alternative B lets a healthier peer take over.
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
} ptp_cfg_t;

/** Fill @p cfg with the defaults named above. */
void ptp_cfg_defaults(ptp_cfg_t *cfg);

/**
 * Check @p cfg against the ranges the engine relies on.
 *
 * @retval 0        Usable.
 * @retval -EINVAL  NULL, a log interval outside [-7, 7], announceReceiptTimeout
 *                  below 2 (§7.7.3.1 requires N >= 2), portNumber 0 (reserved),
 *                  or an out-of-range transport/profile/degradation selector.
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
 * The §3.8 quality block projected onto what PTP needs. See the file header for
 * the wave-3 adaptation note.
 */
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

/** Milliseconds in 2^@p log_interval seconds, clamped to [-7, 7] and rounded. */
uint32_t ptp_log_interval_ms(int8_t log_interval);

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
 * @param a  Dataset A; must not be NULL.
 * @param b  Dataset B; must not be NULL.
 */
ptp_dscmp_t ptp_bmca_compare(const ptp_dataset_t *a, const ptp_dataset_t *b);

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

/* -------------------------------------------------- foreign-master table -- */

/** One foreign-master record, §9.3.2.4. */
typedef struct {
	bool in_use;
	bool qualified;              /* >= PTP_FOREIGN_MASTER_THRESHOLD in the window */
	ptp_port_id_t source_port;   /* sourcePortIdentity of its Announces */
	ptp_announce_t announce;     /* the most recent Announce body */
	uint16_t flags;              /* its most recent flagField */
	int8_t log_announce_interval;/* its advertised logMessageInterval */
	uint16_t count;              /* Announces inside the current window */
	uint64_t window_start_ms;
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
 * Record an Announce, creating or refreshing the sender's entry.
 *
 * Qualification follows §9.3.2.4.5: a record becomes qualified once
 * PTP_FOREIGN_MASTER_THRESHOLD Announces land inside a window of
 * PTP_FOREIGN_MASTER_TIME_WINDOW announce intervals. A gap longer than the
 * window restarts the count. A full table evicts its least recently heard
 * record, which is also the one closest to expiry.
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

/** Build the comparison dataset of a foreign record. */
void ptp_foreign_dataset(const ptp_foreign_t *f, const ptp_port_id_t *receiver,
			 ptp_dataset_t *out);

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
 * @p buf points into the context's scratch buffer and is valid only for the
 * duration of the callback: the glue must copy it or hand it to the stack
 * synchronously.
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

/** Per-message-type and engine counters. */
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
	uint32_t txts_unmatched;   /* ptp_on_sync_txts() for an unknown sequenceId */
	uint32_t tx_errors;
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

	ptp_counters_t counters;
	uint8_t txbuf[PTP_MSG_MAX_LEN];
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
 * @param rx_tai_ns  Hardware ingress timestamp, TAI nanoseconds. Only read for
 *                   event messages; pass 0 for general ones.
 *
 * @retval 0        Consumed, including deliberately ignored messages.
 * @retval -EINVAL  @p c or @p buf is NULL.
 * @retval -EBADMSG Malformed or truncated.
 * @retval -EPROTO  Wrong versionPTP, domainNumber or majorSdoId.
 * @retval -EPERM   The port is not receiving (INITIALIZING or FAULTY).
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

/** Latched alarm bits, PTP_ALARM_*; 0 for a NULL context. */
uint32_t ptp_port_alarms(const ptp_port_ctx_t *c);

/** Counter block, or NULL for a NULL context. */
const ptp_counters_t *ptp_port_counters(const ptp_port_ctx_t *c);

/** Our own dataset, as the BMCA compares it. Exposed for telemetry and tests. */
int ptp_port_dataset(const ptp_port_ctx_t *c, ptp_dataset_t *out);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_PTP_PTP_H_ */
