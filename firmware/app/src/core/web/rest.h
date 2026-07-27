/*
 * STS1000 "Meridian" — core/web: versioned REST API (spec §5.1, §5.2).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11, allocation-free. Everything the API can report or
 * change arrives through @ref rest_providers_t, so the whole router — routing,
 * authentication, role enforcement, CSRF, audit, JSON encoding and decoding —
 * is exercised on the host against fakes, and the Zephyr glue contains no
 * policy at all.
 *
 * Security invariants the router enforces (and the host suite pins):
 *
 *   1. Every mutating route requires a session whose role is >= the route's
 *      minimum, AND a matching CSRF token. No exceptions, including when
 *      `sec.auth.req` is off — that key relaxes *read* access only, because a
 *      browser that can be made to POST cross-origin does not care whether the
 *      operator disabled login.
 *   2. CFG_F_NOEXPORT values are never emitted, at any role, on any route.
 *      CFG_F_SECRET values are never emitted unless the caller is an admin and
 *      asks for them explicitly (`?secrets=1`), which is audited.
 *   3. Every mutating request that is accepted writes one audit record to the
 *      log ring (LOGR_SUB_SEC), including the route, the outcome and the role.
 *   4. A provider that is NULL yields 501, never a half-answer.
 *
 * Portable errno only
 * -------------------
 * Every code this module returns or interprets exists in picolibc AND in
 * Zephyr's minimal libc. That deliberately excludes several codes that would
 * read better — ENOKEY, EKEYREJECTED, EBADE — because a host build accepts them
 * and the target link does not. If a provider needs a new outcome, pick from the
 * portable set rather than the Linux one.
 */

#ifndef STS1000_CORE_WEB_REST_H_
#define STS1000_CORE_WEB_REST_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cfg/cfg.h"
#include "logring/logring.h"
#include "quality/quality.h"
#include "web/auth_web.h"
#include "web/http_parse.h"
#include "web/web.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ limits */

/** Rails reported on the Power/Health page (the nine INA228s). */
#define REST_RAIL_MAX 9U
/** Satellites carried in a GNSS status document / skyplot. */
#define REST_SAT_MAX 32U
/** Log records returned by one GET /logs page. */
#define REST_LOG_PAGE_MAX 32U
/**
 * Config keys emitted by GET /config when the caller asks for no `group`,
 * `start` or `max`.
 *
 * The whole schema does not fit one response buffer on this part, and a 500 on
 * the most obvious request in the API would be a bad answer, so an unqualified
 * listing pages: it returns this many keys and a non-zero `next_id` for the
 * client to continue from. A caller that names a `group` (which is what a UI
 * page actually wants) gets that group whole.
 */
#define REST_CONFIG_PAGE_DEFAULT 32U
/** Longest extra-header block a response may carry (Set-Cookie etc.). */
#define REST_EXTRA_MAX 256U
/** Longest ETag value, including quotes and NUL. */
#define REST_ETAG_MAX 32U

/* ------------------------------------------------------- provider payloads */

/** One monitored rail. */
typedef struct {
	const char *name;
	const char *designator;
	const char *shunt_ref;
	uint8_t     addr;
	int32_t     bus_mv;
	int32_t     current_ua;
	uint32_t    power_uw;
	uint16_t    diag_alrt;
	int32_t     nominal_mv;
	uint32_t    design_max_ma;
	bool        valid;
} rest_rail_t;

/** Power/health snapshot (mirrors sts_health_t; core cannot see that header). */
typedef struct {
	uint8_t     n_rails;
	rest_rail_t rail[REST_RAIL_MAX];

	int32_t tmp_osc_mc;
	int32_t tmp_amb_mc;
	int32_t die_mc;
	int32_t humidity_mpct;
	bool    tmp_osc_valid;
	bool    tmp_amb_valid;
	bool    die_valid;
	bool    humidity_valid;

	uint32_t fan_rpm;
	uint16_t fan_duty_pct;

	uint8_t  poe_class;
	uint32_t poe_draw_mw;
	uint32_t poe_budget_mw;

	bool bkp_stm_pg;
	bool bkp_gps_pg;

	uint32_t age_ms;
} rest_health_t;

/** One satellite, for the skyplot. */
typedef struct {
	uint8_t gnss_id; /**< 0 GPS, 1 SBAS, 2 Galileo, 3 BeiDou, 5 QZSS, 6 GLONASS */
	uint8_t sv_id;
	uint8_t cno;
	int8_t  elev_deg;
	int16_t azim_deg;
	bool    used;
} rest_sat_t;

/** Survey-in / fixed-position state. */
typedef enum {
	REST_SURVEY_IDLE = 0,
	REST_SURVEY_ACTIVE,
	REST_SURVEY_FIXED,
} rest_survey_state_t;

/** Antenna supervisor verdict. */
typedef enum {
	REST_ANT_UNKNOWN = 0,
	REST_ANT_OK,
	REST_ANT_OPEN,
	REST_ANT_SHORT,
} rest_ant_state_t;

/** GNSS receiver snapshot. */
typedef struct {
	uint8_t  fix_type; /**< quality_gnss_fix_t */
	uint8_t  sv_used;
	uint8_t  sv_visible;
	uint32_t tacc_ns;

	uint8_t  survey_state; /**< rest_survey_state_t */
	uint32_t survey_dur_s;
	uint32_t survey_obs;
	uint32_t survey_acc_mm;

	int64_t ecef_x_cm;
	int64_t ecef_y_cm;
	int64_t ecef_z_cm;
	bool    position_valid;

	uint8_t ant_state; /**< rest_ant_state_t */
	bool    ant_bias_on;

	int8_t  leap_pending;
	int16_t leap_current_s;
	bool    utc_valid;

	uint8_t    n_sats;
	rest_sat_t sat[REST_SAT_MAX];

	char sw_version[24];
	char hw_version[16];

	/** False when no GNSS receiver thread has published yet; the SPA shows
	 *  "no receiver detail" rather than an empty sky. */
	bool detail_available;
} rest_gnss_t;

/** Network snapshot. */
typedef struct {
	bool     link_up;
	bool     ipv4_ok;
	bool     ipv6_ok;
	bool     dhcp_bound;
	uint32_t ipv4_addr; /**< host order */
	uint32_t ipv4_mask;
	uint32_t ipv4_gw;
	uint32_t link_changes;
	uint8_t  mac[6];
	char     hostname[64];

	bool    ptp_clock_ok;
	bool    ptp_clock_synced;
	int64_t ptp_clock_off_ns;
	int32_t ptp_clock_rate_ppb;
} rest_net_t;

/** PTP grandmaster snapshot. */
typedef struct {
	uint8_t  port_state;
	uint8_t  clock_class;
	uint8_t  clock_accuracy;
	uint8_t  domain;
	uint8_t  transport;
	bool     running;
	uint32_t alarms;
	uint32_t tx_total;
	uint32_t rx_total;
	uint32_t announce_timeouts;
	uint32_t followup_missed;
} rest_ptp_t;

/** Service counters for the Dashboard. */
typedef struct {
	uint64_t ntp_rx;
	uint64_t ntp_served;
	uint64_t ntp_kod;
	uint64_t ntp_dropped;
	uint64_t nts_served;
	uint64_t ntske_ok;
	uint64_t ntske_fail;
	uint32_t snmp_gets;
	uint32_t web_requests;
	bool     ntp_running;
	bool     nts_running;
	bool     ptp_running;
	bool     snmp_running;
	uint32_t uptime_s;
	char     fw_version[24];
	char     model[16];
	uint8_t  board_id[8];

	/*
	 * SNMPv3 USM refusal counters (RFC 3414 §5 usmStats).
	 *
	 * The evidence base for "somebody is working on the v3 interface": an
	 * attacker guessing a USM user name, replaying outside the timeliness
	 * window, or failing the digest increments exactly these. They are
	 * exported as counters rather than folded into an alarm because the
	 * useful signal is the *rate*, which only a poller can see.
	 */
	uint64_t snmp_v3_unknown_users;
	uint64_t snmp_v3_wrong_digests;
	uint64_t snmp_v3_time_windows;
	uint64_t snmp_v3_decrypt_errors;
	uint64_t snmp_v3_unknown_engines;
	uint64_t snmp_v3_bad_sec_levels;
	uint64_t snmp_v3_authenticated;
} rest_services_t;

/** One image slot. */
typedef struct {
	uint8_t  slot;
	uint32_t size;
	uint32_t version[4];
	bool     valid;
	bool     active;
	bool     pending;
	bool     confirmed;
} rest_fw_slot_t;

/** Firmware / DFU state. */
typedef struct {
	uint8_t        n_slots;
	rest_fw_slot_t slot[2];
	uint8_t        dfu_state;
	uint32_t       dfu_total;
	uint32_t       dfu_written;
	uint32_t       chunk_max;
	uint32_t       write_block;
	bool           pending_confirm; /**< running an unconfirmed test image */
} rest_fw_t;

/** ACME state, reported even when ACME is compiled out. */
typedef enum {
	REST_ACME_DISABLED = 0,
	REST_ACME_IDLE,
	REST_ACME_ACCOUNT,
	REST_ACME_ORDER,
	REST_ACME_CHALLENGE,
	REST_ACME_FINALIZE,
	REST_ACME_ISSUED,
	REST_ACME_FAILED,
} rest_acme_state_t;

/** TLS certificate description. */
typedef struct {
	bool present;
	bool self_signed;
	bool operator_supplied;
	bool persisted; /**< survives a reboot (matters for pinning) */
	char subject[72];
	char issuer[72];
	char not_before[24];
	char not_after[24];
	char sha256_fp[68]; /**< lowercase hex of the DER SHA-256 */
	char key_type[16];
	uint16_t key_bits;
	uint8_t  acme_state; /**< rest_acme_state_t */
	char     acme_detail[64];
} rest_cert_t;

/**
 * A trust anchor this box uses as a CLIENT, i.e. a CA it verifies somebody
 * else's certificate against. Today that is the LDAPS issuer.
 *
 * Deliberately not rest_cert_t: that type describes the server identity this
 * box PRESENTS, and half its fields (key type, ACME state, self-signed) are
 * meaningless for an anchor. Two directions of TLS, two types.
 */
typedef struct {
	bool present;
	bool persisted; /**< survives a reboot */
	char subject[72];
	char not_after[24];
	char sha256_fp[68]; /**< lowercase hex of the DER SHA-256 */
} rest_trust_t;

/** Reference-override modes for POST /timing/reference. */
typedef enum {
	REST_REF_AUTO = 0,
	REST_REF_OCXO,
	REST_REF_RB,
} rest_ref_mode_t;

/** Services that POST /services/<name> can toggle. */
typedef enum {
	REST_SVC_NTP = 0,
	REST_SVC_NTS,
	REST_SVC_PTP,
	REST_SVC_SNMP,
	REST_SVC_SYSLOG,
	REST_SVC_COUNT,
} rest_service_t;

/** Calibration procedures POST /calibration/run can start. */
typedef enum {
	REST_CAL_HOLDOVER = 0, /**< §10.4 holdover characterisation run */
	REST_CAL_OCXO_TUNE,    /**< OCXO pull-range / tempco sweep */
	REST_CAL_INA_TRIM,     /**< per-board SHUNT_CAL trim against a load */
	REST_CAL_COMPASS,      /**< e-compass hard/soft-iron */
	REST_CAL_COUNT,
} rest_cal_proc_t;

/* --------------------------------------------------------------- providers */

/**
 * Everything the API needs from the platform. A NULL member makes its routes
 * answer 501 Not Implemented; nothing here is faked.
 */
typedef struct {
	int (*quality)(void *u, quality_block_t *out);
	int (*health)(void *u, rest_health_t *out);
	int (*gnss)(void *u, rest_gnss_t *out);
	int (*net)(void *u, rest_net_t *out);
	int (*ptp)(void *u, rest_ptp_t *out);
	int (*services)(void *u, rest_services_t *out);

	uint64_t (*alarms)(void *u);
	uint64_t (*alarms_latched)(void *u);
	/** Human name for alarm bit @p bit, or NULL for "report the number". */
	const char *(*alarm_name)(void *u, uint8_t bit);

	/* control */
	int (*gnss_survey)(void *u, bool start);
	int (*gnss_fixed)(void *u, int64_t x_cm, int64_t y_cm, int64_t z_cm);
	int (*ref_override)(void *u, uint8_t mode);
	int (*service_enable)(void *u, uint8_t svc, bool enable);
	int (*cal_run)(void *u, uint8_t proc, uint32_t arg);
	int (*reboot)(void *u, uint8_t mode);
	int (*factory_reset)(void *u);

	/*
	 * Firmware. These MUST be bound to the existing core/mcp DFU engine,
	 * not to a second staging implementation (see sts_web.c).
	 *
	 * Error contract, so the router can produce the right status code:
	 *   fw_begin  -ENOSPC   image larger than the staging slot    -> 413
	 *   fw_data   -EPROTO   offset gap; @p out_next is the frontier -> 409
	 *   fw_end    -EBADMSG  SHA-256 / MCUboot header mismatch     -> 422
	 * Anything else negative becomes 409 (or 500 for fw_data).
	 */
	int (*fw_info)(void *u, rest_fw_t *out);
	int (*fw_begin)(void *u, uint32_t size, const uint8_t sha256[32],
			uint32_t *out_next);
	int (*fw_data)(void *u, uint32_t off, const uint8_t *data, size_t len,
		       uint32_t *out_next);
	int (*fw_end)(void *u, uint32_t *out_size);
	int (*fw_confirm)(void *u);
	int (*fw_revert)(void *u);

	/*
	 * TLS. cert_install() error contract:
	 *   -EBADMSG  the body is not a PEM cert + key pair  -> 422
	 *   -EPERM    the key does not match the certificate -> 422
	 */
	int (*cert_info)(void *u, rest_cert_t *out);
	int (*cert_install)(void *u, const char *pem, size_t len);
	int (*csr_make)(void *u, const char *subject, char *out, size_t cap,
			size_t *out_len);

	/*
	 * The LDAPS trust anchor (spec §9.4). ldap_ca_install() error contract:
	 *   -EBADMSG  not a PEM certificate                      -> 422
	 *   -EFBIG    larger than the device's anchor buffer      -> 413
	 *   -EPERM    the blob carries private-key material       -> 422
	 *   -EROFS    accepted and live, but not persisted        -> 200 + note
	 *   -ENOTSUP  this build cannot do TLS at all             -> 501
	 */
	int (*ldap_ca_info)(void *u, rest_trust_t *out);
	int (*ldap_ca_install)(void *u, const char *pem, size_t len);

	void *u;
} rest_providers_t;

/* --------------------------------------------------------------- responses */

/** A response the caller serialises onto the socket. */
typedef struct {
	uint16_t    status;
	const char *content_type; /**< NULL means "application/json" */
	char       *body;         /**< caller-owned buffer */
	size_t      body_cap;
	size_t      body_len;
	char        etag[REST_ETAG_MAX];
	char        extra[REST_EXTRA_MAX]; /**< extra headers, each CRLF-ended */
	size_t      extra_len;
	bool        no_store;  /**< emit Cache-Control: no-store */
	bool        close;     /**< close the connection after this response */
	bool        body_is_binary; /**< body holds raw bytes, not text */
} web_resp_t;

/** Reason phrase for @p status, e.g. "Not Found". Never NULL. */
const char *rest_status_text(uint16_t status);

/** Bind a response to its body buffer and reset it to 200/JSON. */
void rest_resp_init(web_resp_t *r, char *body, size_t cap);

/* ---------------------------------------------------------------- context */

/** Router counters. */
typedef struct {
	uint32_t requests;
	uint32_t responses_2xx;
	uint32_t responses_4xx;
	uint32_t responses_5xx;
	uint32_t unauthorized;
	uint32_t forbidden;
	uint32_t csrf_failures;
	uint32_t not_found;
	uint32_t audit_records;
	uint32_t fw_chunks;
} rest_stats_t;

/** Router state. Caller-owned. */
typedef struct {
	cfg_ctx_t              *cfg;
	logr_t                 *log;
	auth_web_ctx_t         *auth;
	const rest_providers_t *pv;

	/** Commit hook: appliers must run, so the router cannot call
	 *  cfg_commit() directly (sts_app.h sts_cfg_commit() does). NULL falls
	 *  back to cfg_commit(). */
	int (*commit)(void *u, cfg_commit_res_t *res);
	void *commit_u;

	uint64_t     now_ms;
	rest_stats_t st;
} rest_ctx_t;

/**
 * Bind a router.
 *
 * @retval 0        Ready.
 * @retval -EINVAL  @p c is NULL.
 */
int rest_init(rest_ctx_t *c, cfg_ctx_t *cfg, logr_t *log, auth_web_ctx_t *auth,
	      const rest_providers_t *pv);

/* --------------------------------------------------------------- dispatch */

/** Per-request authentication outcome, resolved before routing. */
typedef struct {
	bool   have_session;
	size_t sess;   /**< valid only when have_session */
	uint8_t role;  /**< WEB_ROLE_NONE when unauthenticated */
} rest_authctx_t;

/**
 * Resolve the request's session from its cookie or bearer token.
 *
 * Always succeeds: an absent or dead token simply yields role NONE.
 */
void rest_resolve_auth(rest_ctx_t *c, const http_req_t *req,
		       rest_authctx_t *out);

/**
 * Route and answer one request.
 *
 * @param body      Request body (already de-chunked); may be NULL when empty.
 * @param body_len  Body length in bytes.
 * @param resp      Initialised with rest_resp_init(); filled in on return.
 *
 * @retval 0        @p resp holds the answer (which may be a 4xx or 5xx).
 * @retval -EINVAL  Bad argument.
 */
int rest_dispatch(rest_ctx_t *c, const http_req_t *req, const char *body,
		  size_t body_len, web_resp_t *resp);

/** True when @p req's path is inside the REST namespace. */
bool rest_is_api_path(const http_req_t *req);

/* ---------------------------------------------------- status encoders */

/**
 * Encode one status group as a JSON object into @p w.
 *
 * Shared with the WebSocket telemetry stream so the two can never disagree
 * about a field name or a unit.
 *
 * @param group  A single WSS_GRP_* bit.
 * @retval 0        Encoded.
 * @retval -EINVAL  Bad argument or an unknown group.
 * @retval -ENOTSUP No provider for that group.
 */
int rest_encode_group(rest_ctx_t *c, uint8_t group, web_jw_t *w);

/**
 * Encode a telemetry envelope for the WebSocket stream:
 * `{"t":<mono_ms>,"seq":N,"<group>":{...},...}`.
 *
 * @param groups  Mask of WSS_GRP_* bits.
 * @retval >=0      Bytes written.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOSPC  @p cap too small.
 */
int rest_encode_telemetry(rest_ctx_t *c, uint8_t groups, uint32_t seq,
			  char *out, size_t cap);

/**
 * Encode a batch of log records from @p cursor forward, advancing it.
 *
 * @retval >=0      Bytes written.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOSPC  @p cap too small.
 */
int rest_encode_logs(rest_ctx_t *c, uint32_t *cursor, uint16_t max,
		     uint8_t max_level, char *out, size_t cap);

/** Prometheus text-format metrics (spec §10.5 subset). */
int rest_encode_metrics(rest_ctx_t *c, char *out, size_t cap);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_WEB_REST_H_ */
