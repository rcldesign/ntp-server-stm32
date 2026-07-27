/*
 * STS1000 "Meridian" — core/snmp: compact SNMPv2c read-only agent + traps.
 *
 * Platform-neutral C11, no dynamic allocation, all state in a caller-owned
 * context (ARCHITECTURE.md §4, dependency edge `snmp→util,quality`). The socket,
 * the community string's storage and the trap destination are the glue's
 * business; this module turns request octets into response octets and builds
 * trap PDUs.
 *
 * ---------------------------------------------------------------------------
 * Scope — deliberate
 * ---------------------------------------------------------------------------
 *
 * 1. **This file is the SNMPv2c envelope; USM lives in snmp_v3.h.** Spec §5.3
 *    makes SNMPv3/USM the primary interface with v2c behind an explicit enable,
 *    so `cfg.v2c_disabled` gates this path and `snmp_peek_version()` routes a
 *    datagram to whichever handler owns it. Both share one PDU layer
 *    (snmp_internal.h) so a v2c and a v3 manager can never get different
 *    answers. A version field this handler does not own is counted and dropped
 *    rather than answered, because answering a v1 manager with a v2c PDU is
 *    worse than silence. SET is answered with errorStatus notWritable(17) — the
 *    agent exposes no writable object, and saying so is more useful to a manager
 *    than dropping.
 *
 * 2. **No SMIv2 file is generated from this table.** The OID catalogue below is
 *    the normative description of the enterprise MIB; the shipped
 *    `STS1000-MIB.txt` and the Zabbix template (spec §5.3) are authored against
 *    it and kept in sync by review, not by generation.
 *
 * 3. **Enterprise PEN is a placeholder.** 99999 is not an IANA-assigned Private
 *    Enterprise Number. TODO(pre-production): apply for a PEN and change
 *    SNMP_PEN in one place; every OID in the table derives from it.
 *
 * 4. **No informs.** Traps are unacknowledged (RFC 3416 SNMPv2-Trap-PDU, tag
 *    0xA7). InformRequest needs retransmission state and an acknowledgement
 *    path; a grandmaster that also drives a local UI and a syslog stream has
 *    cheaper ways to be certain it was heard.
 *
 * ---------------------------------------------------------------------------
 * Values come from the caller
 * ---------------------------------------------------------------------------
 *
 * This module holds no telemetry. Every leaf resolves through
 * @ref snmp_getter_t, which the Zephyr glue binds to the §3.8 quality snapshot,
 * the fault aggregator and the service counters. That is what keeps `snmp` a
 * pure codec plus a walk, testable on the host with a table of canned answers.
 */

#ifndef STS1000_CORE_SNMP_SNMP_H_
#define STS1000_CORE_SNMP_SNMP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ limits */

/**
 * Largest datagram accepted or produced, octets.
 *
 * RFC 3416 §3 requires every SNMPv2 implementation to handle at least 484
 * octets; 1400 keeps a full GETBULK response inside a 1500-octet Ethernet MTU
 * without IP fragmentation, which matters because a fragmented UDP response is
 * the first thing a loaded network drops.
 */
#ifndef SNMP_PKT_MAX
#define SNMP_PKT_MAX 1400U
#endif

/** Longest OID, in arcs, that this agent parses or emits. */
#ifndef SNMP_OID_MAX_LEN
#define SNMP_OID_MAX_LEN 24U
#endif

/** Longest OCTET STRING value a getter may return. */
#ifndef SNMP_OCTET_MAX
#define SNMP_OCTET_MAX 64U
#endif

/** Longest community string compared, excluding the NUL. */
#define SNMP_COMMUNITY_MAX 31U

/**
 * Notification types the rate limiter keeps state for.
 *
 * Must equal SNMP_TRAP__COUNT, which is declared further down (the enum needs
 * SNMP_PEN, which needs the BER tags). snmp.c carries a `_Static_assert` that
 * ties the two together, so the duplication cannot silently drift.
 */
#define SNMP_NOTIFY_TYPES 10U

/** Variable bindings accepted in one request. */
#ifndef SNMP_MAX_VARBINDS
#define SNMP_MAX_VARBINDS 16U
#endif

/** Repetitions honoured in one GETBULK, whatever the request asks for. */
#ifndef SNMP_MAX_REPETITIONS
#define SNMP_MAX_REPETITIONS 64U
#endif

/* --------------------------------------------------------------- BER tags */

#define SNMP_TAG_INTEGER      0x02U
#define SNMP_TAG_OCTET_STRING 0x04U
#define SNMP_TAG_NULL         0x05U
#define SNMP_TAG_OID          0x06U
#define SNMP_TAG_SEQUENCE     0x30U

/* APPLICATION class, primitive (RFC 2578 §7.1). */
#define SNMP_TAG_IPADDRESS    0x40U
#define SNMP_TAG_COUNTER32    0x41U
#define SNMP_TAG_GAUGE32      0x42U
#define SNMP_TAG_TIMETICKS    0x43U
#define SNMP_TAG_OPAQUE       0x44U
#define SNMP_TAG_COUNTER64    0x46U

/* CONTEXT class, primitive: the SNMPv2 varbind exceptions (RFC 3416 §3). */
#define SNMP_TAG_NO_SUCH_OBJECT   0x80U
#define SNMP_TAG_NO_SUCH_INSTANCE 0x81U
#define SNMP_TAG_END_OF_MIB_VIEW  0x82U

/* CONTEXT class, constructed: the PDU tags (RFC 3416 §3). */
#define SNMP_PDU_GET       0xA0U
#define SNMP_PDU_GETNEXT   0xA1U
#define SNMP_PDU_RESPONSE  0xA2U
#define SNMP_PDU_SET       0xA3U
#define SNMP_PDU_GETBULK   0xA5U
#define SNMP_PDU_INFORM    0xA6U
#define SNMP_PDU_TRAP_V2   0xA7U
#define SNMP_PDU_REPORT    0xA8U

/** SNMP version field values (RFC 3416 §4 / RFC 1901). */
#define SNMP_VERSION_1  0
#define SNMP_VERSION_2C 1
#define SNMP_VERSION_3  3

/** error-status values used by this agent (RFC 3416 §3). */
#define SNMP_ERR_NO_ERROR    0
#define SNMP_ERR_TOO_BIG     1
#define SNMP_ERR_NO_SUCH_NAME 2
#define SNMP_ERR_GEN_ERR     5
#define SNMP_ERR_NO_ACCESS   6
#define SNMP_ERR_NOT_WRITABLE 17

/* -------------------------------------------------------------- BER codec */

/** BER reader over a caller-owned buffer. All fields private. */
typedef struct {
	const uint8_t *buf;
	size_t len;
	size_t off;
} snmp_rd_t;

/** BER writer over a caller-owned buffer. All fields private. */
typedef struct {
	uint8_t *buf;
	size_t cap;
	size_t len;
} snmp_wr_t;

/** Bind a reader to @p buf. NULL @p buf with a non-zero @p len is rejected. */
int snmp_rd_init(snmp_rd_t *r, const uint8_t *buf, size_t len);

/** Bind a writer to @p buf and empty it. */
int snmp_wr_init(snmp_wr_t *w, uint8_t *buf, size_t cap);

/** Octets written so far. */
size_t snmp_wr_len(const snmp_wr_t *w);

/**
 * Read one TLV header.
 *
 * @param tag  Receives the identifier octet.
 * @param len  Receives the content length; the reader is left on the content.
 *
 * @retval 0         Read.
 * @retval -EINVAL   NULL argument.
 * @retval -EBADMSG  Truncated, a multi-octet identifier (unused by SNMP), an
 *                   indefinite length (illegal in BER-encoded SNMP), a length
 *                   over four octets, or a content run past the buffer.
 */
int snmp_rd_hdr(snmp_rd_t *r, uint8_t *tag, size_t *len);

/** Skip @p len content octets. */
int snmp_rd_skip(snmp_rd_t *r, size_t len);

/** True once the reader is at or past the end of its buffer. */
bool snmp_rd_done(const snmp_rd_t *r);

/**
 * Read a signed INTEGER (tag 0x02).
 *
 * @retval 0         Read.
 * @retval -EBADMSG  Wrong tag, zero-length, or wider than eight octets.
 */
int snmp_rd_int(snmp_rd_t *r, int64_t *out);

/**
 * Read an unsigned value carried under @p want_tag (Counter32, Gauge32,
 * TimeTicks, Counter64). A leading 0x00 padding octet is accepted.
 */
int snmp_rd_uint(snmp_rd_t *r, uint8_t want_tag, uint64_t *out);

/**
 * Read an OCTET STRING; @p out points into the reader's buffer.
 *
 * @retval -EBADMSG  Wrong tag or truncated.
 */
int snmp_rd_octets(snmp_rd_t *r, const uint8_t **out, size_t *out_len);

/**
 * Read an OBJECT IDENTIFIER into @p arcs.
 *
 * @param cap        Capacity of @p arcs in arcs.
 * @param out_n      Receives the arcs stored (never more than @p cap).
 * @param out_over   Optional; set true when the encoded OID had more arcs than
 *                   @p cap and was truncated.
 *
 * @retval 0         Read.
 * @retval -EBADMSG  Wrong tag, empty, an unterminated base-128 arc, an arc
 *                   wider than 32 bits, or a non-minimal (0x80-prefixed) arc.
 */
int snmp_rd_oid(snmp_rd_t *r, uint32_t *arcs, size_t cap, size_t *out_n,
		bool *out_over);

/** Append a raw TLV. @p val may be NULL only when @p len is 0. */
int snmp_wr_tlv(snmp_wr_t *w, uint8_t tag, const uint8_t *val, size_t len);

/** Append a signed INTEGER under @p tag (minimal two's-complement content). */
int snmp_wr_int(snmp_wr_t *w, uint8_t tag, int64_t v);

/**
 * Append an unsigned value under @p tag, with the leading 0x00 the encoding
 * needs when the most significant bit of the first content octet is set.
 */
int snmp_wr_uint(snmp_wr_t *w, uint8_t tag, uint64_t v);

/** Append a zero-length TLV: NULL, or one of the varbind exceptions. */
int snmp_wr_empty(snmp_wr_t *w, uint8_t tag);

/** Append an OBJECT IDENTIFIER. At least two arcs are required. */
int snmp_wr_oid(snmp_wr_t *w, const uint32_t *arcs, size_t n);

/** Append an IpAddress (four octets, network order). */
int snmp_wr_ip(snmp_wr_t *w, uint32_t addr_be_host_order);

/**
 * Open a constructed TLV. Three octets are reserved for the length and shrunk
 * to the minimal encoding by snmp_wr_end(); @p mark carries the reservation.
 */
int snmp_wr_begin(snmp_wr_t *w, uint8_t tag, size_t *mark);

/** Close the constructed TLV opened at @p mark. */
int snmp_wr_end(snmp_wr_t *w, size_t mark);

/** Truncate the writer back to @p len (used to roll a varbind back). */
int snmp_wr_rewind(snmp_wr_t *w, size_t len);

/* ------------------------------------------------------------------ values */

/**
 * A resolved leaf value.
 *
 * @p type selects which member carries it: INTEGER uses @p n.i; Counter32,
 * Gauge32, TimeTicks and Counter64 use @p n.u; IpAddress uses @p n.u (host
 * order, emitted big-endian); OCTET STRING uses @p octets / @p len; OID uses
 * @p oid / @p len (arcs). The three exception tags carry nothing.
 */
typedef struct {
	uint8_t type;
	uint16_t len;
	union {
		int64_t i;
		uint64_t u;
	} n;
	uint8_t octets[SNMP_OCTET_MAX];
	uint32_t oid[SNMP_OID_MAX_LEN];
} snmp_value_t;

void snmp_val_int(snmp_value_t *v, int64_t x);
void snmp_val_uint(snmp_value_t *v, uint8_t tag, uint64_t x);
void snmp_val_ip(snmp_value_t *v, uint32_t addr);
/** Copy @p n octets (truncated at SNMP_OCTET_MAX). @p s may be NULL if n == 0. */
void snmp_val_octets(snmp_value_t *v, const uint8_t *s, size_t n);
/** Copy a NUL-terminated string, truncated at SNMP_OCTET_MAX. */
void snmp_val_str(snmp_value_t *v, const char *s);
/** Copy @p n arcs (truncated at SNMP_OID_MAX_LEN). */
void snmp_val_oid(snmp_value_t *v, const uint32_t *arcs, size_t n);

/** Append @p v as a varbind value. Unknown types encode as noSuchObject. */
int snmp_wr_value(snmp_wr_t *w, const snmp_value_t *v);

/* ------------------------------------------------------------- the catalog */

/**
 * Private Enterprise Number arc.
 *
 * TODO(pre-production): 99999 is a placeholder — IANA has not assigned this
 * project a PEN. Every enterprise OID in the table is built from this one
 * constant, so the change is a single edit plus a MIB-file and Zabbix-template
 * refresh.
 */
#define SNMP_PEN 99999U

/** Rows in the per-rail table: the nine INA228 monitors (interface ref §4.2). */
#define SNMP_RAIL_COUNT 9U

/**
 * Object identifiers handed to the getter.
 *
 * Wire-visible only through the OID table, but they are the glue's switch
 * labels, so append rather than renumber.
 */
typedef enum {
	/* MIB-II system group (1.3.6.1.2.1.1) */
	SNMP_OBJ_SYS_DESCR = 0,
	SNMP_OBJ_SYS_OBJECT_ID,
	SNMP_OBJ_SYS_UPTIME,
	SNMP_OBJ_SYS_CONTACT,
	SNMP_OBJ_SYS_NAME,
	SNMP_OBJ_SYS_LOCATION,
	SNMP_OBJ_SYS_SERVICES,

	/* timing (enterprise .1.1) */
	SNMP_OBJ_STRATUM,
	SNMP_OBJ_LOCK_STATE,
	SNMP_OBJ_ACTIVE_REF,
	SNMP_OBJ_HOLDOVER,
	SNMP_OBJ_HOLDOVER_EST_ERR_NS,
	SNMP_OBJ_HOLDOVER_ELAPSED_S,
	SNMP_OBJ_HOLDOVER_DEMOTE_S,
	SNMP_OBJ_PPS_OFFSET_NS,
	SNMP_OBJ_PPS_MEAN_NS,
	SNMP_OBJ_PPS_SIGMA_NS,
	SNMP_OBJ_FREQ_ERR_PPT,
	SNMP_OBJ_VC_CMD_MV,
	SNMP_OBJ_VC_SENSE_MV,
	SNMP_OBJ_ADEV_1S_E18,
	SNMP_OBJ_ADEV_10S_E18,
	SNMP_OBJ_ADEV_100S_E18,
	SNMP_OBJ_OSC_TEMP_MC,

	/* gnss (enterprise .1.2) */
	SNMP_OBJ_GNSS_FIX,
	SNMP_OBJ_GNSS_SV_USED,
	SNMP_OBJ_GNSS_SV_VISIBLE,
	SNMP_OBJ_GNSS_TACC_NS,
	SNMP_OBJ_GNSS_ANT_STATE,
	SNMP_OBJ_GNSS_LEAP_PENDING,
	SNMP_OBJ_GNSS_LEAP_CURRENT_S,

	/* power (enterprise .1.3), columns of the rail table */
	SNMP_OBJ_RAIL_INDEX,
	SNMP_OBJ_RAIL_NAME,
	SNMP_OBJ_RAIL_MV,
	SNMP_OBJ_RAIL_UA,
	SNMP_OBJ_RAIL_MW,

	/* thermal / environment (enterprise .1.4) */
	SNMP_OBJ_TEMP_ENCLOSURE_MC,
	SNMP_OBJ_TEMP_OSC_MC,
	SNMP_OBJ_TEMP_DIE_MC,
	SNMP_OBJ_HUMIDITY_MPCT,
	SNMP_OBJ_FAN_RPM,
	SNMP_OBJ_FAN_DUTY_PCT,

	/* time services (enterprise .1.5) */
	SNMP_OBJ_NTP_RX,
	SNMP_OBJ_NTP_SERVED,
	SNMP_OBJ_NTP_DROPPED,
	SNMP_OBJ_NTP_KOD,
	SNMP_OBJ_NTP_AUTH_FAIL,
	SNMP_OBJ_NTP_RATE_LIMITED,
	SNMP_OBJ_NTS_RX,
	SNMP_OBJ_NTS_OK,
	SNMP_OBJ_NTS_NAK,
	SNMP_OBJ_NTS_COOKIES,
	SNMP_OBJ_NTSKE_HANDSHAKES,

	/* ptp (enterprise .1.6) */
	SNMP_OBJ_PTP_PORT_STATE,
	SNMP_OBJ_PTP_CLOCK_CLASS,
	SNMP_OBJ_PTP_CLOCK_ACCURACY,
	SNMP_OBJ_PTP_DOMAIN,
	SNMP_OBJ_PTP_TX,
	SNMP_OBJ_PTP_RX,
	SNMP_OBJ_PTP_ANN_TIMEOUTS,
	SNMP_OBJ_PTP_ALARMS,

	/* health (enterprise .1.7) */
	SNMP_OBJ_ALARMS,
	SNMP_OBJ_FW_VERSION,
	SNMP_OBJ_GNSS_FW_VERSION,
	SNMP_OBJ_UPTIME_S,
	SNMP_OBJ_POE_CLASS,
	SNMP_OBJ_POE_DRAW_MW,
	SNMP_OBJ_SUPERCAP_STM_MV,
	SNMP_OBJ_SUPERCAP_GPS_MV,
	SNMP_OBJ_SERIAL,

	SNMP_OBJ__COUNT
} snmp_obj_t;

/**
 * One entry of the static OID catalogue.
 *
 * The full OID of an instance is @p base concatenated with one instance arc:
 * 0 for a scalar (@p n_inst == 1), or 1..@p n_inst for a table column. That is
 * exactly SMIv2's `.0` scalar instance and its integer-indexed table rows, so a
 * table sorted by @p base with ascending instance arcs is already in
 * lexicographic OID order and the GETNEXT walk needs no table-specific code.
 */
typedef struct {
	const uint32_t *base;
	uint8_t base_len;
	uint8_t type;    /* SNMP_TAG_* the getter is expected to return */
	uint16_t obj;    /* snmp_obj_t */
	uint16_t n_inst; /* 1 for a scalar, SNMP_RAIL_COUNT for a rail column */
} snmp_node_t;

/** Entries in the catalogue. */
size_t snmp_mib_node_count(void);

/** Entry @p i, or NULL when out of range. */
const snmp_node_t *snmp_mib_node(size_t i);

/** Total addressable instances across every entry. */
size_t snmp_mib_instance_count(void);

/**
 * Full OID of the @p idx'th instance in lexicographic order.
 *
 * @param out_obj   Optional; receives the snmp_obj_t.
 * @param out_inst  Optional; receives the instance arc (0 for a scalar).
 * @param out_type  Optional; receives the declared value tag.
 * @param arcs      Receives the OID; must hold SNMP_OID_MAX_LEN arcs.
 *
 * @retval >=0      Arcs written.
 * @retval -EINVAL  NULL @p arcs.
 * @retval -ENOENT  @p idx is past the end.
 */
int snmp_mib_instance(size_t idx, uint16_t *out_obj, uint16_t *out_inst,
		      uint8_t *out_type, uint32_t *arcs);

/** Index of the instance whose OID equals @p arcs, or -ENOENT. */
int snmp_mib_find(const uint32_t *arcs, size_t n);

/**
 * Index of the first instance whose OID is strictly greater than @p arcs,
 * or -ENOENT at the end of the MIB view.
 */
int snmp_mib_find_next(const uint32_t *arcs, size_t n);

/** Build the OID of (@p obj, @p inst). @retval >=0 arcs written, else -ENOENT. */
int snmp_mib_oid_of(uint16_t obj, uint16_t inst, uint32_t *arcs);

/**
 * Verify that the catalogue is sorted and self-consistent — every base strictly
 * ascending, no OID longer than SNMP_OID_MAX_LEN, every object id in range.
 *
 * Called by the unit test rather than by an assertion: a mis-ordered table
 * breaks GETNEXT walks in a way that is invisible to a GET-only manager, so it
 * has to be a checked property, and core has no assert that may kill a host
 * test runner (ARCHITECTURE.md §4).
 *
 * @retval 0   Consistent.
 * @retval <0  The index of the first offending entry, negated and offset by
 *             one (entry 0 reports -1).
 */
int snmp_mib_check(void);

/* ------------------------------------------------------------------ getter */

/**
 * Resolve one leaf.
 *
 * @param obj   snmp_obj_t.
 * @param inst  Instance arc: 0 for a scalar, 1..SNMP_RAIL_COUNT for a rail.
 * @param out   Receives the value; pre-zeroed by the caller.
 *
 * @retval 0   @p out holds the value.
 * @retval <0  No value right now; the varbind is answered with
 *             noSuchInstance, which is how SNMPv2 says "the object exists but
 *             this instance has no value at the moment".
 */
typedef struct {
	int (*get)(void *ctx, uint16_t obj, uint16_t inst, snmp_value_t *out);
	void *ctx;
} snmp_getter_t;

/* ------------------------------------------------------------------ config */

typedef struct {
	/**
	 * Community string accepted on requests and emitted in traps. Borrowed,
	 * not copied, and must outlive the context. NULL or "" refuses every
	 * request: an agent with no community configured is not an agent that
	 * answers "public".
	 */
	const char *community;
	/** Leaf resolver. Required. */
	snmp_getter_t getter;
	/** Repetitions honoured per GETBULK; 0 selects SNMP_MAX_REPETITIONS. */
	uint16_t max_repetitions;
	/**
	 * Refuse SNMPv2c outright (spec §9.5: "v2c only if explicitly
	 * enabled").
	 *
	 * Spelled negatively so a zero-initialised configuration keeps the
	 * historical behaviour — v2c available whenever the agent is enabled —
	 * and every existing caller and test is unaffected. The Zephyr glue
	 * drives it from cfg key `sec.snmp.v2c`, which defaults to 0, so a
	 * shipped box is SNMPv3-only until an operator says otherwise.
	 */
	bool v2c_disabled;
	/**
	 * Minimum interval between two notifications of the *same* type, ms.
	 * 0 disables rate limiting. See @ref snmp_notify_gate.
	 */
	uint32_t notify_min_interval_ms;
} snmp_cfg_t;

/* ------------------------------------------------------------------- stats */

typedef struct {
	uint64_t rx;              /**< Datagrams handed to snmp_handle(). */
	uint64_t responses;       /**< Responses produced. */
	uint64_t get;             /**< GetRequest PDUs. */
	uint64_t getnext;         /**< GetNextRequest PDUs. */
	uint64_t getbulk;         /**< GetBulkRequest PDUs. */
	uint64_t set_refused;     /**< SetRequest answered notWritable. */
	uint64_t bad_version;     /**< Version field other than v2c. */
	uint64_t bad_community;   /**< Community mismatch — an auth failure. */
	uint64_t malformed;       /**< Failed to parse. */
	uint64_t too_big;         /**< Answered with errorStatus tooBig. */
	uint64_t unsupported_pdu; /**< A PDU tag this agent does not serve. */
	uint64_t traps;           /**< Trap PDUs built. */
	uint64_t varbinds;        /**< Varbinds answered across all responses. */
	uint64_t v2c_refused;     /**< v2c datagram dropped because v2c is off. */
	uint64_t notify_suppressed; /**< Notifications the rate limiter dropped. */
} snmp_stats_t;

/* ----------------------------------------------------------------- context */

/** Agent context. Caller-owned; all fields private. */
typedef struct {
	snmp_cfg_t cfg;
	snmp_stats_t stats;
	uint32_t trap_reqid;
	/* Per-notification rate-limiter state (@ref snmp_notify_gate). */
	uint64_t notify_last_ms[SNMP_NOTIFY_TYPES];
	uint32_t notify_suppressed[SNMP_NOTIFY_TYPES];
	bool notify_sent[SNMP_NOTIFY_TYPES];
	bool ready;
} snmp_ctx_t;

/**
 * Initialise an agent.
 *
 * @retval 0        Ready.
 * @retval -EINVAL  NULL argument, or a configuration with no getter.
 */
int snmp_init(snmp_ctx_t *c, const snmp_cfg_t *cfg);

/** Replace the community string (a runtime cfg apply). NULL disables the agent. */
int snmp_set_community(snmp_ctx_t *c, const char *community);

/**
 * Handle one received datagram.
 *
 * @param uptime_cs  sysUpTime in hundredths of a second, for the response's
 *                   own bookkeeping and for traps. Passed in rather than read
 *                   from a port so the whole datapath stays deterministic.
 * @param rsp        Response buffer.
 * @param rsp_len    Receives the response length when the return value is 0.
 *
 * @retval 0         Send @p rsp_len octets.
 * @retval -EACCES   Community mismatch: drop, and the caller should raise an
 *                   authentication-failure trap.
 * @retval -EPROTO   Version not v2c: drop.
 * @retval -EBADMSG  Malformed: drop.
 * @retval -ENOTSUP  A PDU this agent does not serve (Response, Trap, Report,
 *                   Inform): drop.
 * @retval -EINVAL   NULL argument or an uninitialised context.
 * @retval -ENOSPC   @p rsp_cap cannot hold even an empty response.
 */
int snmp_handle(snmp_ctx_t *c, const uint8_t *req, size_t req_len,
		uint32_t uptime_cs, uint8_t *rsp, size_t rsp_cap,
		size_t *rsp_len);

/* ------------------------------------------------------------------- traps */

/**
 * Notifications this agent emits (spec §5.3).
 *
 * SNMP_TRAP_COLD_START carries the standard snmpTraps.coldStart OID
 * (1.3.6.1.6.3.1.1.5.1); every other value carries an enterprise notification
 * OID under `<enterprise>.1.0.<n>`, the SMIv2 convention that keeps
 * notifications out of the object subtree a manager walks.
 */
typedef enum {
	SNMP_TRAP_COLD_START = 0,
	SNMP_TRAP_LOCK_ACQUIRED,
	SNMP_TRAP_LOCK_LOST,
	SNMP_TRAP_HOLDOVER_ENTER,
	SNMP_TRAP_HOLDOVER_EXIT,
	SNMP_TRAP_REF_SWITCH,
	SNMP_TRAP_RAIL_ALARM,
	SNMP_TRAP_THERMAL_ALARM,
	SNMP_TRAP_ANTENNA_FAULT,
	SNMP_TRAP_AUTH_FAILURE,
	SNMP_TRAP__COUNT
} snmp_trap_t;

/** One extra varbind to attach to a trap. */
typedef struct {
	uint16_t obj;  /**< snmp_obj_t */
	uint16_t inst; /**< instance arc; 0 for a scalar */
} snmp_bind_t;

/** Notification OID of @p t. @retval >=0 arcs written into @p arcs, else -EINVAL. */
int snmp_trap_oid(snmp_trap_t t, uint32_t *arcs);

/** Short name of @p t, for logs. Never NULL. */
const char *snmp_trap_name(snmp_trap_t t);

/**
 * Build an SNMPv2-Trap PDU.
 *
 * The first two varbinds are mandatory and generated here: sysUpTime.0 and
 * snmpTrapOID.0 (RFC 3416 §4.2.6). @p binds appends further varbinds, each
 * resolved through the configured getter; one that fails to resolve is emitted
 * as noSuchInstance rather than dropped, so the varbind positions a manager
 * expects stay stable.
 *
 * @retval 0        @p out_len octets built.
 * @retval -EINVAL  NULL argument, uninitialised context, or an unknown @p t.
 * @retval -EACCES  No community is configured.
 * @retval -ENOSPC  @p cap is too small.
 */
int snmp_make_trap(snmp_ctx_t *c, snmp_trap_t t, uint32_t uptime_cs,
		   const snmp_bind_t *binds, size_t n_binds, uint8_t *out,
		   size_t cap, size_t *out_len);

/* ------------------------------------------------- version peek & v2c gate */

/**
 * Read the msgVersion field without validating anything else.
 *
 * A dual-stack agent has to route a datagram to the v2c or the v3 handler before
 * either of them can parse it, and both handlers reject a wrong version — so the
 * routing decision needs this and nothing more.
 *
 * @retval 0         @p out_ver holds the version (0 = v1, 1 = v2c, 3 = v3).
 * @retval -EINVAL   NULL argument.
 * @retval -EBADMSG  Not a BER SEQUENCE with an INTEGER first.
 */
int snmp_peek_version(const uint8_t *msg, size_t len, int32_t *out_ver);

/** Enable or disable the SNMPv2c path at runtime (a cfg apply). */
int snmp_set_v2c_enabled(snmp_ctx_t *c, bool enabled);

/* ----------------------------------------------------- notification gating */

/**
 * Should notification @p t be emitted now?
 *
 * Enforces `cfg.notify_min_interval_ms` per notification *type*. Call it once
 * per candidate event and only build the trap when it returns true.
 *
 * Without this, one unauthenticated peer generates one authentication-failure
 * trap per datagram: the agent then amplifies an attack on this box into an
 * attack on the trap receiver, and the flood buries every genuine event. Gating
 * per type keeps an authFailure burst from starving a lockLost.
 *
 * @param mono_ms         Monotonic milliseconds; the caller owns the clock.
 * @param out_suppressed  Optional; on a `true` return, how many notifications
 *                        of this type were suppressed since the last one got
 *                        through — worth putting in the log line that
 *                        accompanies the trap.
 *
 * @return true when the caller should send; false when it should not.
 */
bool snmp_notify_gate(snmp_ctx_t *c, snmp_trap_t t, uint64_t mono_ms,
		      uint32_t *out_suppressed);

/** Notifications of @p t suppressed since the last one that passed the gate. */
int snmp_notify_suppressed(const snmp_ctx_t *c, snmp_trap_t t, uint32_t *out);

/* ------------------------------------------------------------------- stats */

/** Snapshot the counters. @p out is untouched on -EINVAL. */
int snmp_stats_get(const snmp_ctx_t *c, snmp_stats_t *out);

/** Zero the counters. */
int snmp_stats_reset(snmp_ctx_t *c);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_SNMP_SNMP_H_ */
