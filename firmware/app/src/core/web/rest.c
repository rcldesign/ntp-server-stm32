/*
 * STS1000 "Meridian" — core/web: the REST router (see rest.h).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 */

#include "web/rest.h"

#include <errno.h>
#include <string.h>

#include "web/wss.h"

/* ------------------------------------------------------------------------- */
/* status lines                                                              */
/* ------------------------------------------------------------------------- */

const char *rest_status_text(uint16_t status)
{
	switch (status) {
	case 200:
		return "OK";
	case 201:
		return "Created";
	case 202:
		return "Accepted";
	case 204:
		return "No Content";
	case 304:
		return "Not Modified";
	case 400:
		return "Bad Request";
	case 401:
		return "Unauthorized";
	case 403:
		return "Forbidden";
	case 404:
		return "Not Found";
	case 405:
		return "Method Not Allowed";
	case 409:
		return "Conflict";
	case 411:
		return "Length Required";
	case 413:
		return "Content Too Large";
	case 415:
		return "Unsupported Media Type";
	case 422:
		return "Unprocessable Content";
	case 426:
		return "Upgrade Required";
	case 429:
		return "Too Many Requests";
	case 431:
		return "Request Header Fields Too Large";
	case 500:
		return "Internal Server Error";
	case 501:
		return "Not Implemented";
	case 503:
		return "Service Unavailable";
	default:
		return "Error";
	}
}

void rest_resp_init(web_resp_t *r, char *body, size_t cap)
{
	if (r == NULL) {
		return;
	}
	memset(r, 0, sizeof(*r));
	r->status = 200U;
	r->body = body;
	r->body_cap = cap;
	if (body != NULL && cap > 0U) {
		body[0] = '\0';
	}
}

/* ------------------------------------------------------------------------- */
/* small response helpers                                                    */
/* ------------------------------------------------------------------------- */

static void resp_add_header(web_resp_t *r, const char *line)
{
	size_t n = strlen(line);

	if ((r->extra_len + n + 2U) > REST_EXTRA_MAX) {
		return;
	}
	memcpy(&r->extra[r->extra_len], line, n);
	r->extra_len += n;
	r->extra[r->extra_len++] = '\r';
	r->extra[r->extra_len++] = '\n';
}

/* `<name>: <value>` with a bounded value. */
static void resp_add_kv(web_resp_t *r, const char *name, const char *value)
{
	char line[REST_EXTRA_MAX];
	size_t nn = strlen(name);
	size_t vn = strlen(value);

	if ((nn + 2U + vn + 1U) > sizeof(line)) {
		return;
	}
	memcpy(line, name, nn);
	line[nn] = ':';
	line[nn + 1U] = ' ';
	memcpy(&line[nn + 2U], value, vn);
	line[nn + 2U + vn] = '\0';
	resp_add_header(r, line);
}

/* Emit `{"error":"<code>","detail":"<detail>"}` with @p status. */
static int fail(rest_ctx_t *c, web_resp_t *r, uint16_t status, const char *code,
		const char *detail)
{
	web_jw_t w;

	r->status = status;
	r->no_store = true;
	r->content_type = "application/json";
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_kstr(&w, "error", code);
	if (detail != NULL) {
		web_jw_kstr(&w, "detail", detail);
	}
	web_jw_ku64(&w, "status", status);
	web_jw_obj_end(&w);
	(void)web_jw_finish(&w, &r->body_len);

	if (status >= 500U) {
		c->st.responses_5xx++;
	} else if (status >= 400U) {
		c->st.responses_4xx++;
	}
	return 0;
}

static int ok_json(rest_ctx_t *c, web_resp_t *r, web_jw_t *w)
{
	int rc = web_jw_finish(w, &r->body_len);

	if (rc != 0) {
		/* A truncated document is not a partial success: a client that
		 * parsed half an object would act on half the truth. */
		return fail(c, r, 500, "encode_overflow",
			    "response did not fit the response buffer");
	}
	r->status = 200U;
	r->content_type = "application/json";
	c->st.responses_2xx++;
	return 0;
}

/* ------------------------------------------------------------------------- */
/* audit                                                                     */
/* ------------------------------------------------------------------------- */

/*
 * One audit record per accepted mutating request (spec §5.4). The record is
 * intentionally terse — logr_rec_t caps the message at LOGR_MSG_MAX — and
 * carries who (role), what (method + path) and the outcome. The tamper-evident
 * append-only audit file on /lfs is the glue's job; this is the event source.
 */
static void audit_action(rest_ctx_t *c, const rest_authctx_t *a,
			 const http_req_t *req, const char *verb, int result)
{
	char msg[LOGR_MSG_MAX];
	size_t o = 0U;
	const char *role = web_role_name(a->role);
	size_t n;

	if (c->log == NULL) {
		return;
	}

	n = strlen("web ");
	memcpy(&msg[o], "web ", n);
	o += n;

	n = strlen(role);
	if ((o + n + 1U) < sizeof(msg)) {
		memcpy(&msg[o], role, n);
		o += n;
		msg[o++] = ' ';
	}
	n = strlen(web_method_name(req->method));
	if ((o + n + 1U) < sizeof(msg)) {
		memcpy(&msg[o], web_method_name(req->method), n);
		o += n;
		msg[o++] = ' ';
	}
	n = req->path.n;
	if ((o + n + 1U) < sizeof(msg)) {
		memcpy(&msg[o], req->path.p, n);
		o += n;
		msg[o++] = ' ';
	}
	if (verb != NULL) {
		n = strlen(verb);
		if ((o + n + 1U) < sizeof(msg)) {
			memcpy(&msg[o], verb, n);
			o += n;
			msg[o++] = ' ';
		}
	}
	if ((o + 4U) < sizeof(msg)) {
		msg[o++] = '=';
		if (result == 0) {
			msg[o++] = 'o';
			msg[o++] = 'k';
		} else {
			msg[o++] = 'e';
			msg[o++] = 'r';
			msg[o++] = 'r';
		}
	}
	msg[o] = '\0';
	(void)logr_put(c->log, (result == 0) ? (uint8_t)LOGR_NOTICE
					    : (uint8_t)LOGR_WARN,
		       (uint8_t)LOGR_SUB_SEC, c->now_ms, msg, o);
	c->st.audit_records++;
}

/* ------------------------------------------------------------------------- */
/* provider access                                                           */
/* ------------------------------------------------------------------------- */

#define PV(c) ((c)->pv)

static bool have_pv(const rest_ctx_t *c)
{
	return c->pv != NULL;
}

/* ------------------------------------------------------------------------- */
/* status group encoders                                                     */
/* ------------------------------------------------------------------------- */

static const char *lock_name(uint8_t s)
{
	return quality_lock_state_name(s);
}

static const char *ref_name(uint8_t r)
{
	return quality_ref_name(r);
}

static const char *fix_name(uint8_t f)
{
	switch (f) {
	case QUALITY_GNSS_2D:
		return "2d";
	case QUALITY_GNSS_3D:
		return "3d";
	case QUALITY_GNSS_TIME_ONLY:
		return "time";
	default:
		return "none";
	}
}

static const char *gnss_const_name(uint8_t id)
{
	switch (id) {
	case 0U:
		return "gps";
	case 1U:
		return "sbas";
	case 2U:
		return "galileo";
	case 3U:
		return "beidou";
	case 4U:
		return "imes";
	case 5U:
		return "qzss";
	case 6U:
		return "glonass";
	default:
		return "other";
	}
}

static void jw_ipv4(web_jw_t *w, const char *key, uint32_t addr)
{
	char buf[16];
	size_t o = 0U;
	int i;

	for (i = 3; i >= 0; i--) {
		unsigned int b = (addr >> (8 * (unsigned int)i)) & 0xFFU;

		if (b >= 100U) {
			buf[o++] = (char)('0' + (b / 100U));
		}
		if (b >= 10U) {
			buf[o++] = (char)('0' + ((b / 10U) % 10U));
		}
		buf[o++] = (char)('0' + (b % 10U));
		if (i != 0) {
			buf[o++] = '.';
		}
	}
	buf[o] = '\0';
	web_jw_kstr(w, key, buf);
}

static void jw_mac(web_jw_t *w, const char *key, const uint8_t mac[6])
{
	static const char hexd[] = "0123456789abcdef";
	char buf[18];
	size_t o = 0U;
	size_t i;

	for (i = 0U; i < 6U; i++) {
		buf[o++] = hexd[(mac[i] >> 4) & 0x0FU];
		buf[o++] = hexd[mac[i] & 0x0FU];
		if (i != 5U) {
			buf[o++] = ':';
		}
	}
	buf[o] = '\0';
	web_jw_kstr(w, key, buf);
}

/* Root delay/dispersion are NTP short format (16.16 s); report nanoseconds. */
static int64_t q16_ns(uint32_t q16)
{
	return quality_ns_from_ntp_short(q16);
}

static int enc_timing(rest_ctx_t *c, web_jw_t *w)
{
	quality_block_t q;

	if (!have_pv(c) || PV(c)->quality == NULL) {
		return -ENOTSUP;
	}
	if (PV(c)->quality(PV(c)->u, &q) != 0) {
		return -EIO;
	}

	web_jw_obj_begin(w);
	web_jw_ku64(w, "tick", q.tick);
	web_jw_ku64(w, "updated_ms", q.updated_mono_ms);
	web_jw_ku64(w, "stratum", q.stratum);
	web_jw_kstr(w, "lock_state", lock_name(q.lock_state));
	web_jw_kstr(w, "reference", ref_name(q.active_ref));
	web_jw_kbool(w, "holdover", q.holdover);
	web_jw_ku64(w, "flags", q.flags);
	web_jw_ki64(w, "last_pps_offset_ns", q.last_pps_off_ns);
	web_jw_kf32(w, "pps_offset_mean_ns", q.pps_off_mean_ns, 3U);
	web_jw_kf32(w, "pps_offset_sigma_ns", q.pps_off_sigma_ns, 3U);
	web_jw_kf32(w, "freq_err_ppb", q.freq_err_ppb, 4U);
	web_jw_ki64(w, "vc_cmd_mv", q.vc_cmd_mv);
	if ((q.flags & QUALITY_FLAG_VC_SENSE_VALID) != 0U) {
		web_jw_ki64(w, "vc_sense_mv", q.vc_sense_mv);
	} else {
		web_jw_knull(w, "vc_sense_mv");
	}
	web_jw_ku64(w, "dac_code", q.dac_code);
	web_jw_kobj(w, "adev");
	web_jw_kf32(w, "tau_1s", q.adev_1s, 15U);
	web_jw_kf32(w, "tau_10s", q.adev_10s, 15U);
	web_jw_kf32(w, "tau_100s", q.adev_100s, 15U);
	web_jw_obj_end(w);
	web_jw_ki64(w, "root_delay_ns", q16_ns(q.root_delay_q16));
	web_jw_ki64(w, "root_disp_ns", q16_ns(q.root_disp_q16));
	web_jw_kobj(w, "holdover_state");
	web_jw_ki64(w, "est_err_ns", q.holdover_est_err_ns);
	web_jw_ku64(w, "elapsed_s", q.holdover_elapsed_s);
	if (q.holdover_t_demote_s == UINT32_MAX) {
		web_jw_knull(w, "time_to_demote_s");
	} else {
		web_jw_ku64(w, "time_to_demote_s", q.holdover_t_demote_s);
	}
	web_jw_obj_end(w);
	if ((q.flags & QUALITY_FLAG_OSC_TEMP_VALID) != 0U) {
		web_jw_kfixed(w, "osc_temp_c", q.osc_temp_mc, 3U);
	} else {
		web_jw_knull(w, "osc_temp_c");
	}
	web_jw_obj_end(w);
	return 0;
}

static int enc_summary(rest_ctx_t *c, web_jw_t *w)
{
	quality_block_t q;
	rest_services_t sv;
	bool have_q = false;
	bool have_sv = false;

	if (!have_pv(c)) {
		return -ENOTSUP;
	}
	memset(&q, 0, sizeof(q));
	memset(&sv, 0, sizeof(sv));
	if (PV(c)->quality != NULL && PV(c)->quality(PV(c)->u, &q) == 0) {
		have_q = true;
	}
	if (PV(c)->services != NULL && PV(c)->services(PV(c)->u, &sv) == 0) {
		have_sv = true;
	}
	if (!have_q && !have_sv) {
		return -ENOTSUP;
	}

	web_jw_obj_begin(w);
	web_jw_kstr(w, "model", have_sv ? sv.model : "");
	web_jw_kstr(w, "firmware", have_sv ? sv.fw_version : "");
	if (have_sv) {
		web_jw_khex(w, "board_id", sv.board_id, sizeof(sv.board_id));
		web_jw_ku64(w, "uptime_s", sv.uptime_s);
	}
	web_jw_ku64(w, "stratum", have_q ? q.stratum : QUALITY_STRATUM_UNSYNC);
	web_jw_kstr(w, "lock_state", lock_name(have_q ? q.lock_state : 0U));
	web_jw_kstr(w, "reference", ref_name(have_q ? q.active_ref : 0U));
	web_jw_kbool(w, "holdover", have_q ? q.holdover : false);
	web_jw_ki64(w, "holdover_est_err_ns", have_q ? q.holdover_est_err_ns : 0);
	web_jw_kstr(w, "gnss_fix", fix_name(have_q ? q.gnss_fix : 0U));
	web_jw_ku64(w, "gnss_sv_used", have_q ? q.gnss_sv_used : 0U);
	web_jw_ku64(w, "gnss_sv_visible", have_q ? q.gnss_sv_visible : 0U);
	web_jw_ki64(w, "last_pps_offset_ns", have_q ? q.last_pps_off_ns : 0);
	web_jw_kobj(w, "services");
	web_jw_kbool(w, "ntp", have_sv && sv.ntp_running);
	web_jw_kbool(w, "nts", have_sv && sv.nts_running);
	web_jw_kbool(w, "ptp", have_sv && sv.ptp_running);
	web_jw_kbool(w, "snmp", have_sv && sv.snmp_running);
	web_jw_ku64(w, "ntp_rx", have_sv ? sv.ntp_rx : 0U);
	web_jw_ku64(w, "ntp_served", have_sv ? sv.ntp_served : 0U);
	web_jw_ku64(w, "ntp_kod", have_sv ? sv.ntp_kod : 0U);
	web_jw_ku64(w, "nts_served", have_sv ? sv.nts_served : 0U);
	web_jw_obj_end(w);
	if (PV(c)->alarms != NULL) {
		web_jw_ku64(w, "alarms", PV(c)->alarms(PV(c)->u));
	} else {
		web_jw_ku64(w, "alarms", 0U);
	}
	web_jw_obj_end(w);
	return 0;
}

static int enc_gnss(rest_ctx_t *c, web_jw_t *w)
{
	rest_gnss_t g;
	size_t i;

	if (!have_pv(c) || PV(c)->gnss == NULL) {
		return -ENOTSUP;
	}
	memset(&g, 0, sizeof(g));
	if (PV(c)->gnss(PV(c)->u, &g) != 0) {
		return -EIO;
	}

	web_jw_obj_begin(w);
	web_jw_kbool(w, "detail_available", g.detail_available);
	web_jw_kstr(w, "fix", fix_name(g.fix_type));
	web_jw_ku64(w, "sv_used", g.sv_used);
	web_jw_ku64(w, "sv_visible", g.sv_visible);
	web_jw_ku64(w, "time_acc_ns", g.tacc_ns);
	web_jw_kbool(w, "utc_valid", g.utc_valid);
	web_jw_ki64(w, "leap_current_s", g.leap_current_s);
	web_jw_ki64(w, "leap_pending", g.leap_pending);
	web_jw_kstr(w, "sw_version", g.sw_version);
	web_jw_kstr(w, "hw_version", g.hw_version);

	web_jw_kobj(w, "survey");
	switch (g.survey_state) {
	case REST_SURVEY_ACTIVE:
		web_jw_kstr(w, "state", "active");
		break;
	case REST_SURVEY_FIXED:
		web_jw_kstr(w, "state", "fixed");
		break;
	default:
		web_jw_kstr(w, "state", "idle");
		break;
	}
	web_jw_ku64(w, "duration_s", g.survey_dur_s);
	web_jw_ku64(w, "observations", g.survey_obs);
	web_jw_ku64(w, "accuracy_mm", g.survey_acc_mm);
	web_jw_obj_end(w);

	web_jw_kobj(w, "position");
	web_jw_kbool(w, "valid", g.position_valid);
	web_jw_ki64(w, "ecef_x_cm", g.ecef_x_cm);
	web_jw_ki64(w, "ecef_y_cm", g.ecef_y_cm);
	web_jw_ki64(w, "ecef_z_cm", g.ecef_z_cm);
	web_jw_obj_end(w);

	web_jw_kobj(w, "antenna");
	switch (g.ant_state) {
	case REST_ANT_OK:
		web_jw_kstr(w, "state", "ok");
		break;
	case REST_ANT_OPEN:
		web_jw_kstr(w, "state", "open");
		break;
	case REST_ANT_SHORT:
		web_jw_kstr(w, "state", "short");
		break;
	default:
		web_jw_kstr(w, "state", "unknown");
		break;
	}
	web_jw_kbool(w, "bias_on", g.ant_bias_on);
	web_jw_obj_end(w);

	web_jw_karr(w, "satellites");
	for (i = 0U; i < g.n_sats && i < REST_SAT_MAX; i++) {
		web_jw_obj_begin(w);
		web_jw_kstr(w, "constellation", gnss_const_name(g.sat[i].gnss_id));
		web_jw_ku64(w, "sv", g.sat[i].sv_id);
		web_jw_ku64(w, "cno", g.sat[i].cno);
		web_jw_ki64(w, "elev", g.sat[i].elev_deg);
		web_jw_ki64(w, "azim", g.sat[i].azim_deg);
		web_jw_kbool(w, "used", g.sat[i].used);
		web_jw_obj_end(w);
	}
	web_jw_arr_end(w);
	web_jw_obj_end(w);
	return 0;
}

static int enc_power(rest_ctx_t *c, web_jw_t *w)
{
	rest_health_t h;
	size_t i;

	if (!have_pv(c) || PV(c)->health == NULL) {
		return -ENOTSUP;
	}
	memset(&h, 0, sizeof(h));
	if (PV(c)->health(PV(c)->u, &h) != 0) {
		return -EIO;
	}

	web_jw_obj_begin(w);
	web_jw_ku64(w, "age_ms", h.age_ms);
	web_jw_karr(w, "rails");
	for (i = 0U; i < h.n_rails && i < REST_RAIL_MAX; i++) {
		const rest_rail_t *rl = &h.rail[i];

		web_jw_obj_begin(w);
		web_jw_kstr(w, "name", (rl->name != NULL) ? rl->name : "");
		web_jw_kstr(w, "designator",
			    (rl->designator != NULL) ? rl->designator : "");
		web_jw_kstr(w, "shunt", (rl->shunt_ref != NULL) ? rl->shunt_ref : "");
		web_jw_ku64(w, "addr", rl->addr);
		web_jw_kbool(w, "valid", rl->valid);
		if (rl->valid) {
			web_jw_kfixed(w, "bus_v", rl->bus_mv, 3U);
			web_jw_kfixed(w, "current_a", rl->current_ua, 6U);
			web_jw_kfixed(w, "power_w", (int64_t)rl->power_uw, 6U);
		} else {
			web_jw_knull(w, "bus_v");
			web_jw_knull(w, "current_a");
			web_jw_knull(w, "power_w");
		}
		web_jw_ku64(w, "diag_alrt", rl->diag_alrt);
		web_jw_kfixed(w, "nominal_v", rl->nominal_mv, 3U);
		web_jw_ku64(w, "design_max_ma", rl->design_max_ma);
		web_jw_obj_end(w);
	}
	web_jw_arr_end(w);

	web_jw_kobj(w, "temperature");
	if (h.tmp_osc_valid) {
		web_jw_kfixed(w, "oscillator_c", h.tmp_osc_mc, 3U);
	} else {
		web_jw_knull(w, "oscillator_c");
	}
	if (h.tmp_amb_valid) {
		web_jw_kfixed(w, "enclosure_c", h.tmp_amb_mc, 3U);
	} else {
		web_jw_knull(w, "enclosure_c");
	}
	if (h.die_valid) {
		web_jw_kfixed(w, "die_c", h.die_mc, 3U);
	} else {
		web_jw_knull(w, "die_c");
	}
	web_jw_obj_end(w);

	if (h.humidity_valid) {
		web_jw_kfixed(w, "humidity_pct", h.humidity_mpct, 3U);
	} else {
		web_jw_knull(w, "humidity_pct");
	}

	web_jw_kobj(w, "fan");
	web_jw_ku64(w, "rpm", h.fan_rpm);
	web_jw_ku64(w, "duty_pct", h.fan_duty_pct);
	web_jw_obj_end(w);

	web_jw_kobj(w, "poe");
	web_jw_ku64(w, "class", h.poe_class);
	web_jw_kfixed(w, "draw_w", h.poe_draw_mw, 3U);
	web_jw_kfixed(w, "budget_w", h.poe_budget_mw, 3U);
	web_jw_obj_end(w);

	web_jw_kobj(w, "backup");
	web_jw_kbool(w, "stm_pg", h.bkp_stm_pg);
	web_jw_kbool(w, "gps_pg", h.bkp_gps_pg);
	web_jw_obj_end(w);
	web_jw_obj_end(w);
	return 0;
}

static int enc_net(rest_ctx_t *c, web_jw_t *w)
{
	rest_net_t n;

	if (!have_pv(c) || PV(c)->net == NULL) {
		return -ENOTSUP;
	}
	memset(&n, 0, sizeof(n));
	if (PV(c)->net(PV(c)->u, &n) != 0) {
		return -EIO;
	}

	web_jw_obj_begin(w);
	web_jw_kbool(w, "link_up", n.link_up);
	web_jw_kbool(w, "ipv4_ok", n.ipv4_ok);
	web_jw_kbool(w, "ipv6_ok", n.ipv6_ok);
	web_jw_kbool(w, "dhcp_bound", n.dhcp_bound);
	jw_ipv4(w, "ipv4_addr", n.ipv4_addr);
	jw_ipv4(w, "ipv4_mask", n.ipv4_mask);
	jw_ipv4(w, "ipv4_gateway", n.ipv4_gw);
	web_jw_ku64(w, "link_changes", n.link_changes);
	jw_mac(w, "mac", n.mac);
	web_jw_kstr(w, "hostname", n.hostname);
	web_jw_kobj(w, "ptp_clock");
	web_jw_kbool(w, "present", n.ptp_clock_ok);
	web_jw_kbool(w, "synced", n.ptp_clock_synced);
	web_jw_ki64(w, "offset_ns", n.ptp_clock_off_ns);
	web_jw_ki64(w, "rate_ppb", n.ptp_clock_rate_ppb);
	web_jw_obj_end(w);
	web_jw_obj_end(w);
	return 0;
}

static int enc_ptp(rest_ctx_t *c, web_jw_t *w)
{
	rest_ptp_t p;

	if (!have_pv(c) || PV(c)->ptp == NULL) {
		return -ENOTSUP;
	}
	memset(&p, 0, sizeof(p));
	if (PV(c)->ptp(PV(c)->u, &p) != 0) {
		return -EIO;
	}

	web_jw_obj_begin(w);
	web_jw_kbool(w, "running", p.running);
	web_jw_ku64(w, "port_state", p.port_state);
	web_jw_ku64(w, "clock_class", p.clock_class);
	web_jw_ku64(w, "clock_accuracy", p.clock_accuracy);
	web_jw_ku64(w, "domain", p.domain);
	web_jw_ku64(w, "transport", p.transport);
	web_jw_ku64(w, "alarms", p.alarms);
	web_jw_ku64(w, "tx_total", p.tx_total);
	web_jw_ku64(w, "rx_total", p.rx_total);
	web_jw_ku64(w, "announce_timeouts", p.announce_timeouts);
	web_jw_ku64(w, "followup_missed", p.followup_missed);
	web_jw_obj_end(w);
	return 0;
}

static int enc_alarms(rest_ctx_t *c, web_jw_t *w)
{
	uint64_t active;
	uint64_t latched = 0U;
	unsigned int bit;

	if (!have_pv(c) || PV(c)->alarms == NULL) {
		return -ENOTSUP;
	}
	active = PV(c)->alarms(PV(c)->u);
	if (PV(c)->alarms_latched != NULL) {
		latched = PV(c)->alarms_latched(PV(c)->u);
	}

	web_jw_obj_begin(w);
	web_jw_ku64(w, "active_mask", active);
	web_jw_ku64(w, "latched_mask", latched);
	web_jw_karr(w, "alarms");
	for (bit = 0U; bit < 64U; bit++) {
		uint64_t m = (uint64_t)1U << bit;
		const char *nm = NULL;

		if (((active | latched) & m) == 0U) {
			continue;
		}
		if (PV(c)->alarm_name != NULL) {
			nm = PV(c)->alarm_name(PV(c)->u, (uint8_t)bit);
		}
		web_jw_obj_begin(w);
		web_jw_ku64(w, "id", bit);
		if (nm != NULL) {
			web_jw_kstr(w, "name", nm);
		}
		web_jw_kbool(w, "active", (active & m) != 0U);
		web_jw_kbool(w, "latched", (latched & m) != 0U);
		web_jw_obj_end(w);
	}
	web_jw_arr_end(w);
	web_jw_obj_end(w);
	return 0;
}

int rest_encode_group(rest_ctx_t *c, uint8_t group, web_jw_t *w)
{
	if (c == NULL || w == NULL) {
		return -EINVAL;
	}
	switch (group) {
	case WSS_GRP_SUMMARY:
		return enc_summary(c, w);
	case WSS_GRP_TIMING:
		return enc_timing(c, w);
	case WSS_GRP_GNSS:
		return enc_gnss(c, w);
	case WSS_GRP_POWER:
		return enc_power(c, w);
	case WSS_GRP_NET:
		return enc_net(c, w);
	case WSS_GRP_PTP:
		return enc_ptp(c, w);
	case WSS_GRP_ALARMS:
		return enc_alarms(c, w);
	default:
		return -EINVAL;
	}
}

int rest_encode_telemetry(rest_ctx_t *c, uint8_t groups, uint32_t seq, char *out,
			  size_t cap)
{
	web_jw_t w;
	unsigned int i;
	size_t len = 0U;

	if (c == NULL || out == NULL || cap == 0U) {
		return -EINVAL;
	}
	web_jw_init(&w, out, cap);
	web_jw_obj_begin(&w);
	web_jw_kstr(&w, "type", "telemetry");
	web_jw_ku64(&w, "seq", seq);
	web_jw_ku64(&w, "t_ms", c->now_ms);
	for (i = 0U; i < 7U; i++) {
		uint8_t bit = (uint8_t)(1U << i);
		const char *nm;

		if ((groups & bit) == 0U) {
			continue;
		}
		nm = wss_group_name(bit);
		if (nm == NULL) {
			continue;
		}
		web_jw_key(&w, nm);
		if (rest_encode_group(c, bit, &w) != 0) {
			/* No provider for this group: emit null so the SPA can
			 * tell "absent" from "zero". */
			web_jw_null(&w);
		}
	}
	web_jw_obj_end(&w);
	if (web_jw_finish(&w, &len) != 0) {
		return -ENOSPC;
	}
	return (int)len;
}

/* ------------------------------------------------------------------------- */
/* logs                                                                      */
/* ------------------------------------------------------------------------- */

int rest_encode_logs(rest_ctx_t *c, uint32_t *cursor, uint16_t max,
		     uint8_t max_level, char *out, size_t cap)
{
	logr_rec_t recs[REST_LOG_PAGE_MAX];
	logr_filter_t f;
	web_jw_t w;
	uint16_t got = 0U;
	uint32_t next = 0U;
	uint32_t gap = 0U;
	size_t len = 0U;
	uint16_t i;

	if (c == NULL || cursor == NULL || out == NULL || cap == 0U) {
		return -EINVAL;
	}
	if (c->log == NULL) {
		return -ENOTSUP;
	}
	if (max == 0U || max > REST_LOG_PAGE_MAX) {
		max = REST_LOG_PAGE_MAX;
	}
	logr_filter_all(&f);
	if (max_level < LOGR_LEVEL_COUNT) {
		f.max_level = max_level;
	}
	if (logr_tail(c->log, *cursor, &f, recs, max, &got, &next, &gap) != 0) {
		return -EIO;
	}

	web_jw_init(&w, out, cap);
	web_jw_obj_begin(&w);
	web_jw_kstr(&w, "type", "logs");
	web_jw_ku64(&w, "cursor", *cursor);
	web_jw_ku64(&w, "next_cursor", next);
	web_jw_ku64(&w, "gap", gap);
	web_jw_ku64(&w, "head", logr_head(c->log));
	web_jw_ku64(&w, "oldest", logr_oldest(c->log));
	web_jw_ku64(&w, "dropped", logr_dropped(c->log));
	web_jw_karr(&w, "records");
	for (i = 0U; i < got; i++) {
		web_jw_obj_begin(&w);
		web_jw_ku64(&w, "seq", recs[i].seq);
		web_jw_ku64(&w, "mono_ms", recs[i].mono_ms);
		web_jw_ku64(&w, "level", recs[i].level);
		web_jw_kstr(&w, "level_name", logr_level_name(recs[i].level));
		web_jw_kstr(&w, "subsystem", logr_sub_name(recs[i].subsys));
		web_jw_kstrn(&w, "message", recs[i].msg, recs[i].len);
		web_jw_obj_end(&w);
	}
	web_jw_arr_end(&w);
	web_jw_obj_end(&w);
	if (web_jw_finish(&w, &len) != 0) {
		return -ENOSPC;
	}
	*cursor = next;
	return (int)len;
}

/* ------------------------------------------------------------------------- */
/* metrics (Prometheus text format)                                          */
/* ------------------------------------------------------------------------- */

struct mw {
	char  *buf;
	size_t cap;
	size_t len;
	bool   overflow;
};

static void mw_raw(struct mw *m, const char *s, size_t n)
{
	if (m->overflow || m->cap == 0U) {
		m->overflow = true;
		return;
	}
	if (n > ((m->cap - 1U) - m->len)) {
		m->overflow = true;
		return;
	}
	memcpy(&m->buf[m->len], s, n);
	m->len += n;
	m->buf[m->len] = '\0';
}

static void mw_str(struct mw *m, const char *s)
{
	mw_raw(m, s, strlen(s));
}

static void mw_i64(struct mw *m, int64_t v)
{
	char d[21];
	size_t n = 0U;
	uint64_t mag;
	char tmp[20];
	size_t k = 0U;

	if (v < 0) {
		d[n++] = '-';
		mag = ~(uint64_t)v + 1U;
	} else {
		mag = (uint64_t)v;
	}
	do {
		tmp[k++] = (char)('0' + (unsigned int)(mag % 10U));
		mag /= 10U;
	} while (mag != 0U);
	while (k > 0U) {
		d[n++] = tmp[--k];
	}
	mw_raw(m, d, n);
}

/* `name value\n`, integer-valued. */
static void metric(struct mw *m, const char *name, int64_t v)
{
	mw_str(m, name);
	mw_raw(m, " ", 1U);
	mw_i64(m, v);
	mw_raw(m, "\n", 1U);
}

/* `name{label="value"} v\n` */
static void metric_l(struct mw *m, const char *name, const char *label,
		     const char *lval, int64_t v)
{
	mw_str(m, name);
	mw_raw(m, "{", 1U);
	mw_str(m, label);
	mw_raw(m, "=\"", 2U);
	mw_str(m, lval);
	mw_raw(m, "\"} ", 3U);
	mw_i64(m, v);
	mw_raw(m, "\n", 1U);
}

int rest_encode_metrics(rest_ctx_t *c, char *out, size_t cap)
{
	struct mw m = { out, cap, 0U, false };
	quality_block_t q;
	rest_health_t h;
	rest_services_t sv;
	size_t i;

	if (c == NULL || out == NULL || cap == 0U) {
		return -EINVAL;
	}
	memset(&q, 0, sizeof(q));
	memset(&h, 0, sizeof(h));
	memset(&sv, 0, sizeof(sv));

	if (have_pv(c) && PV(c)->quality != NULL &&
	    PV(c)->quality(PV(c)->u, &q) == 0) {
		metric(&m, "sts_stratum", q.stratum);
		metric(&m, "sts_lock_state", q.lock_state);
		metric(&m, "sts_reference", q.active_ref);
		metric(&m, "sts_holdover", q.holdover ? 1 : 0);
		metric(&m, "sts_pps_offset_ns", q.last_pps_off_ns);
		metric(&m, "sts_root_disp_ns", q16_ns(q.root_disp_q16));
		metric(&m, "sts_vc_cmd_mv", q.vc_cmd_mv);
		metric(&m, "sts_dac_code", q.dac_code);
		metric(&m, "sts_gnss_sv_used", q.gnss_sv_used);
		metric(&m, "sts_gnss_sv_visible", q.gnss_sv_visible);
		metric(&m, "sts_gnss_time_acc_ns", q.gnss_tacc_ns);
		metric(&m, "sts_holdover_est_err_ns", q.holdover_est_err_ns);
		metric(&m, "sts_quality_flags", q.flags);
	}
	if (have_pv(c) && PV(c)->health != NULL &&
	    PV(c)->health(PV(c)->u, &h) == 0) {
		for (i = 0U; i < h.n_rails && i < REST_RAIL_MAX; i++) {
			const char *nm = (h.rail[i].name != NULL) ? h.rail[i].name
								 : "unknown";

			if (!h.rail[i].valid) {
				metric_l(&m, "sts_rail_valid", "rail", nm, 0);
				continue;
			}
			metric_l(&m, "sts_rail_valid", "rail", nm, 1);
			metric_l(&m, "sts_rail_bus_mv", "rail", nm,
				 h.rail[i].bus_mv);
			metric_l(&m, "sts_rail_current_ua", "rail", nm,
				 h.rail[i].current_ua);
			metric_l(&m, "sts_rail_power_uw", "rail", nm,
				 (int64_t)h.rail[i].power_uw);
		}
		if (h.tmp_osc_valid) {
			metric(&m, "sts_temp_oscillator_mc", h.tmp_osc_mc);
		}
		if (h.tmp_amb_valid) {
			metric(&m, "sts_temp_enclosure_mc", h.tmp_amb_mc);
		}
		if (h.die_valid) {
			metric(&m, "sts_temp_die_mc", h.die_mc);
		}
		if (h.humidity_valid) {
			metric(&m, "sts_humidity_mpct", h.humidity_mpct);
		}
		metric(&m, "sts_fan_rpm", h.fan_rpm);
		metric(&m, "sts_fan_duty_pct", h.fan_duty_pct);
		metric(&m, "sts_poe_draw_mw", h.poe_draw_mw);
		metric(&m, "sts_poe_budget_mw", h.poe_budget_mw);
	}
	if (have_pv(c) && PV(c)->services != NULL &&
	    PV(c)->services(PV(c)->u, &sv) == 0) {
		metric(&m, "sts_uptime_seconds", sv.uptime_s);
		metric(&m, "sts_ntp_rx_total", (int64_t)sv.ntp_rx);
		metric(&m, "sts_ntp_served_total", (int64_t)sv.ntp_served);
		metric(&m, "sts_ntp_kod_total", (int64_t)sv.ntp_kod);
		metric(&m, "sts_ntp_dropped_total", (int64_t)sv.ntp_dropped);
		metric(&m, "sts_nts_served_total", (int64_t)sv.nts_served);
		metric(&m, "sts_ntske_ok_total", (int64_t)sv.ntske_ok);
		metric(&m, "sts_ntske_fail_total", (int64_t)sv.ntske_fail);
	}
	if (have_pv(c) && PV(c)->alarms != NULL) {
		metric(&m, "sts_alarms_active", (int64_t)PV(c)->alarms(PV(c)->u));
	}
	metric(&m, "sts_web_requests_total", c->st.requests);
	metric(&m, "sts_web_4xx_total", c->st.responses_4xx);
	metric(&m, "sts_web_5xx_total", c->st.responses_5xx);
	if (c->auth != NULL) {
		const auth_web_stats_t *as = auth_web_stats(c->auth);

		metric(&m, "sts_web_logins_ok_total", as->logins_ok);
		metric(&m, "sts_web_logins_failed_total", as->logins_bad_pw);
		metric(&m, "sts_web_sessions", auth_web_session_count(c->auth));
	}

	if (m.overflow) {
		return -ENOSPC;
	}
	return (int)m.len;
}

/* ------------------------------------------------------------------------- */
/* config helpers                                                            */
/* ------------------------------------------------------------------------- */

/* True when a key's value must never leave the box on this route. */
static bool key_is_never_served(const cfg_key_t *k)
{
	return (k->flags & CFG_F_NOEXPORT) != 0U;
}

static bool key_is_secret(const cfg_key_t *k)
{
	return (k->flags & CFG_F_SECRET) != 0U;
}

static void jw_cfg_value(web_jw_t *w, const cfg_key_t *k, const cfg_val_t *v)
{
	switch (k->type) {
	case CFG_T_BOOL:
		web_jw_bool(w, v->v.u != 0U);
		break;
	case CFG_T_U8:
	case CFG_T_U16:
	case CFG_T_U32:
	case CFG_T_U64:
		web_jw_u64(w, v->v.u);
		break;
	case CFG_T_I32:
		web_jw_i64(w, v->v.i);
		break;
	case CFG_T_F32:
		web_jw_f32(w, v->v.f, 6U);
		break;
	case CFG_T_STR:
		web_jw_strn(w, (const char *)v->v.b, v->len);
		break;
	case CFG_T_BLOB:
		/* Lowercase hex; PUT accepts the same spelling back. */
		web_jw_hexn(w, v->v.b, v->len);
		break;
	default:
		web_jw_null(w);
		break;
	}
}

/* Emit one config key as an object. @p secrets allows CFG_F_SECRET values. */
static void jw_cfg_key(web_jw_t *w, const cfg_ctx_t *cfg, const cfg_key_t *k,
		       bool secrets)
{
	cfg_val_t live;
	cfg_val_t staged;
	bool is_staged;
	char idbuf[7];
	static const char hexd[] = "0123456789abcdef";

	idbuf[0] = '0';
	idbuf[1] = 'x';
	idbuf[2] = hexd[(k->id >> 12) & 0x0FU];
	idbuf[3] = hexd[(k->id >> 8) & 0x0FU];
	idbuf[4] = hexd[(k->id >> 4) & 0x0FU];
	idbuf[5] = hexd[k->id & 0x0FU];
	idbuf[6] = '\0';

	web_jw_obj_begin(w);
	web_jw_kstr(w, "id", idbuf);
	web_jw_ku64(w, "id_num", k->id);
	web_jw_kstr(w, "name", k->name);
	web_jw_ku64(w, "type", k->type);
	web_jw_ku64(w, "group", CFG_GROUP(k->id));
	web_jw_ku64(w, "flags", k->flags);
	web_jw_kbool(w, "secret", key_is_secret(k));
	web_jw_kbool(w, "reboot_required",
		     (k->flags & CFG_F_REBOOT_REQUIRED) != 0U);
	web_jw_kbool(w, "calibration", (k->flags & CFG_F_CAL) != 0U);

	if (k->type != CFG_T_STR && k->type != CFG_T_BLOB) {
		switch (k->type) {
		case CFG_T_I32:
			web_jw_ki64(w, "min", k->min.i);
			web_jw_ki64(w, "max", k->max.i);
			break;
		case CFG_T_F32:
			web_jw_kf32(w, "min", k->min.f, 6U);
			web_jw_kf32(w, "max", k->max.f, 6U);
			break;
		default:
			web_jw_ku64(w, "min", k->min.u);
			web_jw_ku64(w, "max", k->max.u);
			break;
		}
	} else {
		web_jw_ku64(w, "maxlen", k->maxlen);
	}

	is_staged = cfg_is_staged(cfg, k->id);
	web_jw_kbool(w, "staged", is_staged);

	if (key_is_never_served(k) || (key_is_secret(k) && !secrets)) {
		/*
		 * Invariant 2 (rest.h): the value is withheld, and the fact that
		 * it exists is reported so the SPA can render "set / not set"
		 * without ever holding the bytes.
		 */
		bool present = false;

		if (cfg_get(cfg, k->id, &live) == 0) {
			present = (k->type == CFG_T_STR || k->type == CFG_T_BLOB)
					  ? (live.len != 0U)
					  : true;
		}
		web_jw_kbool(w, "value_withheld", true);
		web_jw_kbool(w, "value_set", present);
		web_jw_obj_end(w);
		return;
	}

	if (cfg_get(cfg, k->id, &live) == 0) {
		web_jw_key(w, "value");
		jw_cfg_value(w, k, &live);
	} else {
		web_jw_knull(w, "value");
	}
	if (is_staged && cfg_get_effective(cfg, k->id, &staged) == 0) {
		web_jw_key(w, "staged_value");
		jw_cfg_value(w, k, &staged);
	}
	web_jw_obj_end(w);
}

/* Resolve a `/config/<sel>` selector: a hex id, a decimal id, or a name. */
static const cfg_key_t *cfg_key_by_selector(const char *sel, size_t n)
{
	size_t i;
	uint32_t v = 0U;

	if (sel == NULL || n == 0U) {
		return NULL;
	}
	if (n > 2U && sel[0] == '0' && (sel[1] == 'x' || sel[1] == 'X')) {
		for (i = 2U; i < n; i++) {
			int hv;
			char ch = sel[i];

			if (ch >= '0' && ch <= '9') {
				hv = ch - '0';
			} else if (ch >= 'a' && ch <= 'f') {
				hv = (ch - 'a') + 10;
			} else if (ch >= 'A' && ch <= 'F') {
				hv = (ch - 'A') + 10;
			} else {
				return NULL;
			}
			v = (v << 4) | (uint32_t)hv;
			if (v > 0xFFFFU) {
				return NULL;
			}
		}
		return cfg_key_find((uint16_t)v);
	}
	for (i = 0U; i < cfg_key_count(); i++) {
		const cfg_key_t *k = cfg_key_at(i);

		if (k != NULL && web_span_eq(sel, n, k->name)) {
			return k;
		}
	}
	return NULL;
}

/*
 * Stage one JSON value against a schema key. Returns 0, or a negative errno
 * that the caller maps onto a status code.
 */
static int stage_json_value(cfg_ctx_t *cfg, const cfg_key_t *k,
			    const web_json_val_t *v)
{
	cfg_val_t cv;

	memset(&cv, 0, sizeof(cv));
	cv.type = k->type;

	switch (k->type) {
	case CFG_T_BOOL: {
		bool b;

		if (web_json_bool(v, &b) == 0) {
			cv.v.u = b ? 1U : 0U;
			break;
		}
		/* 0/1 are accepted as well as true/false: every JSON encoder in
		 * the wild spells booleans one of those two ways. */
		if (v->type == (uint8_t)WEB_JSON_NUM) {
			uint64_t u;

			if (web_json_u64(v, &u) == 0 && u <= 1U) {
				cv.v.u = u;
				break;
			}
		}
		return -EPROTO;
	}
	case CFG_T_U8:
	case CFG_T_U16:
	case CFG_T_U32:
	case CFG_T_U64: {
		uint64_t u;
		int rc = web_json_u64(v, &u);

		if (rc == -EINVAL) {
			return -EPROTO;
		}
		if (rc != 0) {
			return -ERANGE;
		}
		cv.v.u = u;
		break;
	}
	case CFG_T_I32: {
		int64_t s;
		int rc = web_json_i64(v, &s);

		if (rc == -EINVAL) {
			return -EPROTO;
		}
		if (rc != 0) {
			return -ERANGE;
		}
		if (s < INT32_MIN || s > INT32_MAX) {
			return -ERANGE;
		}
		cv.v.i = (int32_t)s;
		break;
	}
	case CFG_T_F32: {
		/*
		 * Floats arrive as a fixed-point integer of micro-units — the
		 * request encoding for a float field is `{"micro": <int>}` or a
		 * plain integer meaning whole units. Accepting `1.5` would need
		 * a decimal-to-binary float parser in core, which is a lot of
		 * subtle code for four calibration keys.
		 */
		if (v->type == (uint8_t)WEB_JSON_NUM) {
			int64_t s;

			if (web_json_i64(v, &s) != 0) {
				return -EPROTO;
			}
			cv.v.f = (float)s;
			break;
		}
		if (v->type == (uint8_t)WEB_JSON_OBJ) {
			web_json_val_t mu;
			int64_t s;

			if (web_json_obj_get(v->p, v->n, "micro", &mu) != 0 ||
			    web_json_i64(&mu, &s) != 0) {
				return -EPROTO;
			}
			cv.v.f = (float)s / 1000000.0f;
			break;
		}
		return -EPROTO;
	}
	case CFG_T_STR: {
		char buf[CFG_VAL_MAX + 1U];
		int n;

		if (v->type != (uint8_t)WEB_JSON_STR) {
			return -EPROTO;
		}
		n = web_json_str_copy(v, buf, sizeof(buf));
		if (n == -ENOSPC) {
			return -ERANGE;
		}
		if (n < 0) {
			return -EPROTO;
		}
		cv.len = (uint16_t)n;
		memcpy(cv.v.b, buf, (size_t)n);
		break;
	}
	case CFG_T_BLOB: {
		char buf[(CFG_VAL_MAX * 2U) + 1U];
		int n;
		int decoded;

		if (v->type != (uint8_t)WEB_JSON_STR) {
			return -EPROTO;
		}
		n = web_json_str_copy(v, buf, sizeof(buf));
		if (n == -ENOSPC) {
			return -ERANGE;
		}
		if (n < 0) {
			return -EPROTO;
		}
		decoded = web_hex_decode(buf, (size_t)n, cv.v.b, CFG_VAL_MAX);
		if (decoded == -ENOSPC) {
			return -ERANGE;
		}
		if (decoded < 0) {
			return -EPROTO;
		}
		cv.len = (uint16_t)decoded;
		break;
	}
	default:
		return -EPROTO;
	}

	return cfg_set(cfg, k->id, &cv);
}

/* ------------------------------------------------------------------------- */
/* route handlers                                                            */
/* ------------------------------------------------------------------------- */

typedef int (*rest_handler_fn)(rest_ctx_t *c, const http_req_t *req,
			       const rest_authctx_t *a, const char *body,
			       size_t body_len, web_resp_t *r, const char *tail,
			       size_t tail_len);

/*
 * Route flags. Auditing is NOT a flag: every mutating handler calls
 * audit_action() itself, because only the handler knows what the operator
 * actually asked for ("survey-start" is more useful than "POST gnss/survey").
 */
#define RF_PREFIX 0x01U /**< match by prefix; the remainder is the tail */
#define RF_OPEN   0x02U /**< no session required (login / session probe) */

static int h_status(rest_ctx_t *c, const http_req_t *req,
		    const rest_authctx_t *a, const char *body, size_t body_len,
		    web_resp_t *r, const char *tail, size_t tail_len)
{
	web_jw_t w;
	uint8_t group;
	int rc;

	(void)a;
	(void)body;
	(void)body_len;
	(void)req;

	if (tail_len == 0U) {
		/* The whole document: every group with a provider. */
		unsigned int i;

		web_jw_init(&w, r->body, r->body_cap);
		web_jw_obj_begin(&w);
		for (i = 0U; i < 7U; i++) {
			uint8_t bit = (uint8_t)(1U << i);
			const char *nm = wss_group_name(bit);

			if (nm == NULL) {
				continue;
			}
			web_jw_key(&w, nm);
			if (rest_encode_group(c, bit, &w) != 0) {
				web_jw_null(&w);
			}
		}
		web_jw_obj_end(&w);
		r->no_store = true;
		return ok_json(c, r, &w);
	}

	group = wss_group_bit(tail, tail_len);
	if (group == 0U || group == WSS_GRP_LOGS) {
		c->st.not_found++;
		return fail(c, r, 404, "unknown_group", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	rc = rest_encode_group(c, group, &w);
	if (rc == -ENOTSUP) {
		return fail(c, r, 501, "no_provider",
			    "this build has no source for that group");
	}
	if (rc != 0) {
		return fail(c, r, 503, "unavailable", NULL);
	}
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_metrics(rest_ctx_t *c, const http_req_t *req,
		     const rest_authctx_t *a, const char *body, size_t body_len,
		     web_resp_t *r, const char *tail, size_t tail_len)
{
	int n;

	(void)req;
	(void)a;
	(void)body;
	(void)body_len;
	(void)tail;
	(void)tail_len;

	n = rest_encode_metrics(c, r->body, r->body_cap);
	if (n < 0) {
		return fail(c, r, 500, "encode_overflow", NULL);
	}
	r->body_len = (size_t)n;
	r->status = 200U;
	r->content_type = "text/plain; version=0.0.4; charset=utf-8";
	r->no_store = true;
	c->st.responses_2xx++;
	return 0;
}

static int h_logs(rest_ctx_t *c, const http_req_t *req, const rest_authctx_t *a,
		  const char *body, size_t body_len, web_resp_t *r,
		  const char *tail, size_t tail_len)
{
	uint32_t cursor = 0U;
	uint32_t max = REST_LOG_PAGE_MAX;
	uint32_t level = LOGR_DEBUG;
	int n;

	(void)a;
	(void)body;
	(void)body_len;
	(void)tail;
	(void)tail_len;

	(void)http_query_get_u32(req, "cursor", &cursor);
	(void)http_query_get_u32(req, "max", &max);
	(void)http_query_get_u32(req, "level", &level);
	if (max == 0U || max > REST_LOG_PAGE_MAX) {
		max = REST_LOG_PAGE_MAX;
	}
	if (level >= LOGR_LEVEL_COUNT) {
		level = LOGR_DEBUG;
	}

	n = rest_encode_logs(c, &cursor, (uint16_t)max, (uint8_t)level, r->body,
			     r->body_cap);
	if (n == -ENOTSUP) {
		return fail(c, r, 501, "no_log_ring", NULL);
	}
	if (n == -ENOSPC) {
		return fail(c, r, 500, "encode_overflow",
			    "reduce ?max= and retry");
	}
	if (n < 0) {
		return fail(c, r, 503, "unavailable", NULL);
	}
	r->body_len = (size_t)n;
	r->status = 200U;
	r->content_type = "application/json";
	r->no_store = true;
	c->st.responses_2xx++;
	return 0;
}

static int h_config_list(rest_ctx_t *c, const http_req_t *req,
			 const rest_authctx_t *a, const char *body,
			 size_t body_len, web_resp_t *r, const char *tail,
			 size_t tail_len)
{
	web_jw_t w;
	uint32_t group = 0U;
	uint32_t start = 0U;
	uint32_t max = 0U;
	bool want_group;
	bool secrets;
	size_t i;
	size_t emitted = 0U;
	uint16_t next_id = 0U;

	(void)body;
	(void)body_len;
	(void)tail;
	(void)tail_len;

	if (c->cfg == NULL) {
		return fail(c, r, 501, "no_config", NULL);
	}
	want_group = (http_query_get_u32(req, "group", &group) == 0);
	(void)http_query_get_u32(req, "start", &start);
	if (http_query_get_u32(req, "max", &max) != 0 && !want_group) {
		/* An unqualified listing pages rather than overflowing the
		 * response buffer (see REST_CONFIG_PAGE_DEFAULT). */
		max = REST_CONFIG_PAGE_DEFAULT;
	}

	/*
	 * Secrets require BOTH the admin role and an explicit request, and the
	 * ask is audited. CFG_F_NOEXPORT stays withheld regardless.
	 */
	secrets = (a->role >= (uint8_t)WEB_ROLE_ADMIN) &&
		  http_query_has(req, "secrets");
	if (secrets) {
		audit_action(c, a, req, "read-secrets", 0);
	}

	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_ku64(&w, "schema_version", CFG_SCHEMA_VERSION);
	web_jw_ku64(&w, "staged_count", cfg_staged_count(c->cfg));
	web_jw_kbool(&w, "secrets_included", secrets);
	web_jw_karr(&w, "keys");
	for (i = 0U; i < cfg_key_count(); i++) {
		const cfg_key_t *k = cfg_key_at(i);

		if (k == NULL) {
			continue;
		}
		if (want_group && CFG_GROUP(k->id) != (uint8_t)group) {
			continue;
		}
		if (k->id < start) {
			continue;
		}
		if (max != 0U && emitted >= max) {
			next_id = k->id;
			break;
		}
		jw_cfg_key(&w, c->cfg, k, secrets);
		emitted++;
	}
	web_jw_arr_end(&w);
	web_jw_ku64(&w, "count", emitted);
	web_jw_ku64(&w, "next_id", next_id);
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_config_get_one(rest_ctx_t *c, const http_req_t *req,
			    const rest_authctx_t *a, const char *body,
			    size_t body_len, web_resp_t *r, const char *tail,
			    size_t tail_len)
{
	const cfg_key_t *k;
	web_jw_t w;
	bool secrets;

	(void)body;
	(void)body_len;

	if (c->cfg == NULL) {
		return fail(c, r, 501, "no_config", NULL);
	}
	k = cfg_key_by_selector(tail, tail_len);
	if (k == NULL) {
		c->st.not_found++;
		return fail(c, r, 404, "unknown_key", NULL);
	}
	secrets = (a->role >= (uint8_t)WEB_ROLE_ADMIN) &&
		  http_query_has(req, "secrets");
	if (secrets && key_is_secret(k)) {
		audit_action(c, a, req, "read-secret", 0);
	}
	web_jw_init(&w, r->body, r->body_cap);
	jw_cfg_key(&w, c->cfg, k, secrets);
	r->no_store = true;
	return ok_json(c, r, &w);
}

/* PUT /config — stage a set of keys. Body: {"<id-or-name>": <value>, ...}. */
static int h_config_put(rest_ctx_t *c, const http_req_t *req,
			const rest_authctx_t *a, const char *body,
			size_t body_len, web_resp_t *r, const char *tail,
			size_t tail_len)
{
	web_json_val_t root;
	web_json_val_t key;
	web_json_val_t val;
	web_jw_t w;
	size_t cur = 0U;
	unsigned int staged = 0U;
	unsigned int rejected = 0U;
	int step;

	(void)req;
	(void)tail;
	(void)tail_len;

	if (c->cfg == NULL) {
		return fail(c, r, 501, "no_config", NULL);
	}
	if (body == NULL || body_len == 0U) {
		return fail(c, r, 400, "empty_body", NULL);
	}
	root.type = (uint8_t)WEB_JSON_OBJ;
	root.p = body;
	root.n = body_len;
	{
		/* Reject a non-object body before reporting per-key results. */
		size_t probe = 0U;

		while (probe < body_len &&
		       (body[probe] == ' ' || body[probe] == '\t' ||
			body[probe] == '\r' || body[probe] == '\n')) {
			probe++;
		}
		if (probe >= body_len || body[probe] != '{') {
			return fail(c, r, 400, "not_an_object", NULL);
		}
		root.p = &body[probe];
		root.n = body_len - probe;
	}

	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_karr(&w, "results");
	for (;;) {
		const cfg_key_t *k;
		int rc;

		step = web_json_obj_next(&root, &cur, &key, &val);
		if (step <= 0) {
			break;
		}
		k = cfg_key_by_selector(key.p, key.n);
		web_jw_obj_begin(&w);
		web_jw_kstrn(&w, "key", key.p, key.n);
		if (k == NULL) {
			web_jw_kstr(&w, "status", "unknown_key");
			rejected++;
			web_jw_obj_end(&w);
			continue;
		}
		web_jw_kstr(&w, "name", k->name);
		rc = stage_json_value(c->cfg, k, &val);
		if (rc == 0) {
			web_jw_kstr(&w, "status", "staged");
			staged++;
		} else {
			rejected++;
			switch (rc) {
			case -ERANGE:
				web_jw_kstr(&w, "status", "out_of_range");
				break;
			case -EPROTO:
				web_jw_kstr(&w, "status", "type_mismatch");
				break;
			default:
				web_jw_kstr(&w, "status", "rejected");
				break;
			}
		}
		web_jw_obj_end(&w);
	}
	web_jw_arr_end(&w);

	if (step < 0) {
		return fail(c, r, 400, "malformed_json", NULL);
	}

	web_jw_ku64(&w, "staged", staged);
	web_jw_ku64(&w, "rejected", rejected);
	web_jw_ku64(&w, "staged_total", cfg_staged_count(c->cfg));
	web_jw_kbool(&w, "commit_required", cfg_staged_count(c->cfg) != 0U);
	web_jw_obj_end(&w);
	audit_action(c, a, req, "stage", (rejected == 0U) ? 0 : -1);
	r->no_store = true;
	if (rejected != 0U && staged == 0U) {
		int rc = ok_json(c, r, &w);

		if (rc == 0) {
			r->status = 422U;
			c->st.responses_2xx--;
			c->st.responses_4xx++;
		}
		return rc;
	}
	return ok_json(c, r, &w);
}

static int do_commit(rest_ctx_t *c, cfg_commit_res_t *res)
{
	if (c->commit != NULL) {
		return c->commit(c->commit_u, res);
	}
	return cfg_commit(c->cfg, res);
}

static int h_config_commit(rest_ctx_t *c, const http_req_t *req,
			   const rest_authctx_t *a, const char *body,
			   size_t body_len, web_resp_t *r, const char *tail,
			   size_t tail_len)
{
	cfg_commit_res_t res;
	web_jw_t w;
	int rc;

	(void)body;
	(void)body_len;
	(void)tail;
	(void)tail_len;

	if (c->cfg == NULL) {
		return fail(c, r, 501, "no_config", NULL);
	}
	memset(&res, 0, sizeof(res));
	rc = do_commit(c, &res);
	audit_action(c, a, req, "commit", (rc == 0 || rc == -EIO) ? 0 : rc);

	if (rc != 0 && rc != -EIO) {
		web_jw_init(&w, r->body, r->body_cap);
		web_jw_obj_begin(&w);
		web_jw_kstr(&w, "error", "validation_failed");
		web_jw_ki64(&w, "code", rc);
		web_jw_ku64(&w, "staged", res.staged);
		web_jw_obj_end(&w);
		(void)web_jw_finish(&w, &r->body_len);
		r->status = 422U;
		r->no_store = true;
		r->content_type = "application/json";
		c->st.responses_4xx++;
		return 0;
	}

	/* A credential may have been part of the committed set. */
	if (c->auth != NULL) {
		(void)auth_web_reload(c->auth);
	}

	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_ku64(&w, "staged", res.staged);
	web_jw_ku64(&w, "applied", res.applied);
	web_jw_ku64(&w, "reboot_keys", res.reboot_keys);
	web_jw_ku64(&w, "reboot_groups", res.reboot_groups);
	web_jw_ku64(&w, "persist_errors", res.persist_errors);
	web_jw_kbool(&w, "persisted", res.persist_errors == 0U);
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_config_revert(rest_ctx_t *c, const http_req_t *req,
			   const rest_authctx_t *a, const char *body,
			   size_t body_len, web_resp_t *r, const char *tail,
			   size_t tail_len)
{
	web_jw_t w;
	uint16_t dropped;

	(void)body;
	(void)body_len;
	(void)tail;
	(void)tail_len;

	if (c->cfg == NULL) {
		return fail(c, r, 501, "no_config", NULL);
	}
	dropped = cfg_staged_count(c->cfg);
	(void)cfg_revert(c->cfg);
	audit_action(c, a, req, "revert", 0);

	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_ku64(&w, "dropped", dropped);
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_config_export(rest_ctx_t *c, const http_req_t *req,
			   const rest_authctx_t *a, const char *body,
			   size_t body_len, web_resp_t *r, const char *tail,
			   size_t tail_len)
{
	size_t out_len = 0U;
	bool secrets;
	int rc;

	(void)body;
	(void)body_len;
	(void)tail;
	(void)tail_len;

	if (c->cfg == NULL) {
		return fail(c, r, 501, "no_config", NULL);
	}
	secrets = (a->role >= (uint8_t)WEB_ROLE_ADMIN) &&
		  http_query_has(req, "secrets");
	rc = cfg_export_all(c->cfg, (uint8_t *)r->body, r->body_cap, secrets,
			    &out_len);
	if (rc == -ENOSPC) {
		return fail(c, r, 500, "export_too_large", NULL);
	}
	if (rc != 0) {
		return fail(c, r, 500, "export_failed", NULL);
	}
	audit_action(c, a, req, secrets ? "export-secrets" : "export", 0);
	r->body_len = out_len;
	r->status = 200U;
	r->content_type = "application/octet-stream";
	r->body_is_binary = true;
	r->no_store = true;
	resp_add_kv(r, "Content-Disposition",
		    "attachment; filename=\"meridian-config.mcf\"");
	c->st.responses_2xx++;
	return 0;
}

static int h_config_import(rest_ctx_t *c, const http_req_t *req,
			   const rest_authctx_t *a, const char *body,
			   size_t body_len, web_resp_t *r, const char *tail,
			   size_t tail_len)
{
	cfg_commit_res_t res;
	web_jw_t w;
	int rc;

	(void)tail;
	(void)tail_len;

	if (c->cfg == NULL) {
		return fail(c, r, 501, "no_config", NULL);
	}
	if (body == NULL || body_len == 0U) {
		return fail(c, r, 400, "empty_body", NULL);
	}
	memset(&res, 0, sizeof(res));
	rc = cfg_import_all(c->cfg, (const uint8_t *)body, body_len, false, &res);
	audit_action(c, a, req, "import", rc);
	if (rc != 0 && rc != -EIO) {
		const char *code = "import_failed";

		switch (rc) {
		case -EBADMSG:
			code = "bad_magic";
			break;
		case -EILSEQ:
			code = "bad_crc";
			break;
		case -ENOTSUP:
			code = "schema_version";
			break;
		case -ERANGE:
			code = "value_out_of_range";
			break;
		case -EPROTO:
			code = "malformed_record";
			break;
		default:
			break;
		}
		return fail(c, r, 422, code, NULL);
	}
	if (c->auth != NULL) {
		(void)auth_web_reload(c->auth);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_ku64(&w, "applied", res.applied);
	web_jw_ku64(&w, "reboot_groups", res.reboot_groups);
	web_jw_ku64(&w, "persist_errors", res.persist_errors);
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

/* GET /calibration — the CFG_F_CAL keys plus the characterisation inputs. */
static int h_cal_get(rest_ctx_t *c, const http_req_t *req,
		     const rest_authctx_t *a, const char *body, size_t body_len,
		     web_resp_t *r, const char *tail, size_t tail_len)
{
	web_jw_t w;
	size_t i;

	(void)req;
	(void)a;
	(void)body;
	(void)body_len;
	(void)tail;
	(void)tail_len;

	if (c->cfg == NULL) {
		return fail(c, r, 501, "no_config", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_karr(&w, "constants");
	for (i = 0U; i < cfg_key_count(); i++) {
		const cfg_key_t *k = cfg_key_at(i);

		if (k == NULL || (k->flags & CFG_F_CAL) == 0U) {
			continue;
		}
		jw_cfg_key(&w, c->cfg, k, false);
	}
	web_jw_arr_end(&w);
	web_jw_karr(&w, "procedures");
	web_jw_obj_begin(&w);
	web_jw_kstr(&w, "id", "holdover");
	web_jw_ku64(&w, "code", REST_CAL_HOLDOVER);
	web_jw_obj_end(&w);
	web_jw_obj_begin(&w);
	web_jw_kstr(&w, "id", "ocxo_tune");
	web_jw_ku64(&w, "code", REST_CAL_OCXO_TUNE);
	web_jw_obj_end(&w);
	web_jw_obj_begin(&w);
	web_jw_kstr(&w, "id", "ina_trim");
	web_jw_ku64(&w, "code", REST_CAL_INA_TRIM);
	web_jw_obj_end(&w);
	web_jw_obj_begin(&w);
	web_jw_kstr(&w, "id", "compass");
	web_jw_ku64(&w, "code", REST_CAL_COMPASS);
	web_jw_obj_end(&w);
	web_jw_arr_end(&w);
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_cal_run(rest_ctx_t *c, const http_req_t *req,
		     const rest_authctx_t *a, const char *body, size_t body_len,
		     web_resp_t *r, const char *tail, size_t tail_len)
{
	web_json_val_t v;
	web_jw_t w;
	uint8_t proc = (uint8_t)REST_CAL_COUNT;
	uint32_t arg = 0U;
	int rc;

	(void)tail;
	(void)tail_len;

	if (!have_pv(c) || PV(c)->cal_run == NULL) {
		return fail(c, r, 501, "no_provider", NULL);
	}
	if (body == NULL || web_json_obj_get(body, body_len, "procedure", &v) != 0) {
		return fail(c, r, 400, "missing_procedure", NULL);
	}
	if (v.type == (uint8_t)WEB_JSON_STR) {
		if (web_json_str_eq(&v, "holdover")) {
			proc = (uint8_t)REST_CAL_HOLDOVER;
		} else if (web_json_str_eq(&v, "ocxo_tune")) {
			proc = (uint8_t)REST_CAL_OCXO_TUNE;
		} else if (web_json_str_eq(&v, "ina_trim")) {
			proc = (uint8_t)REST_CAL_INA_TRIM;
		} else if (web_json_str_eq(&v, "compass")) {
			proc = (uint8_t)REST_CAL_COMPASS;
		}
	}
	if (proc >= (uint8_t)REST_CAL_COUNT) {
		return fail(c, r, 400, "unknown_procedure", NULL);
	}
	if (web_json_obj_get(body, body_len, "arg", &v) == 0) {
		uint64_t u = 0U;

		if (web_json_u64(&v, &u) == 0 && u <= 0xFFFFFFFFULL) {
			arg = (uint32_t)u;
		}
	}
	rc = PV(c)->cal_run(PV(c)->u, proc, arg);
	audit_action(c, a, req, "cal-run", rc);
	if (rc != 0) {
		return fail(c, r, 409, "rejected", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_kbool(&w, "started", true);
	web_jw_ku64(&w, "procedure", proc);
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_gnss_survey(rest_ctx_t *c, const http_req_t *req,
			 const rest_authctx_t *a, const char *body,
			 size_t body_len, web_resp_t *r, const char *tail,
			 size_t tail_len)
{
	web_json_val_t v;
	web_jw_t w;
	bool start;
	int rc;

	(void)tail;
	(void)tail_len;

	if (!have_pv(c) || PV(c)->gnss_survey == NULL) {
		return fail(c, r, 501, "no_provider", NULL);
	}
	if (body == NULL || web_json_obj_get(body, body_len, "action", &v) != 0 ||
	    v.type != (uint8_t)WEB_JSON_STR) {
		return fail(c, r, 400, "missing_action", NULL);
	}
	if (web_json_str_eq(&v, "start")) {
		start = true;
	} else if (web_json_str_eq(&v, "stop")) {
		start = false;
	} else {
		return fail(c, r, 400, "unknown_action", NULL);
	}
	rc = PV(c)->gnss_survey(PV(c)->u, start);
	audit_action(c, a, req, start ? "survey-start" : "survey-stop", rc);
	if (rc != 0) {
		return fail(c, r, 409, "rejected", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_kbool(&w, "surveying", start);
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_gnss_position(rest_ctx_t *c, const http_req_t *req,
			   const rest_authctx_t *a, const char *body,
			   size_t body_len, web_resp_t *r, const char *tail,
			   size_t tail_len)
{
	web_json_val_t vx;
	web_json_val_t vy;
	web_json_val_t vz;
	web_jw_t w;
	int64_t x;
	int64_t y;
	int64_t z;
	int rc;

	(void)tail;
	(void)tail_len;

	if (!have_pv(c) || PV(c)->gnss_fixed == NULL) {
		return fail(c, r, 501, "no_provider", NULL);
	}
	if (body == NULL ||
	    web_json_obj_get(body, body_len, "ecef_x_cm", &vx) != 0 ||
	    web_json_obj_get(body, body_len, "ecef_y_cm", &vy) != 0 ||
	    web_json_obj_get(body, body_len, "ecef_z_cm", &vz) != 0) {
		return fail(c, r, 400, "missing_coordinates", NULL);
	}
	if (web_json_i64(&vx, &x) != 0 || web_json_i64(&vy, &y) != 0 ||
	    web_json_i64(&vz, &z) != 0) {
		return fail(c, r, 400, "bad_coordinates", NULL);
	}
	/*
	 * Sanity-bound the ECEF triple. A point more than ~1000 km from the
	 * WGS-84 surface is not a survey result, and handing the receiver one
	 * would put it into fixed-position mode at a location that can never
	 * produce a fix. Bound each axis at 1.5 * earth radius in centimetres.
	 */
	{
		const int64_t lim = 956000000LL;

		if (x > lim || x < -lim || y > lim || y < -lim || z > lim ||
		    z < -lim) {
			return fail(c, r, 422, "coordinates_out_of_range", NULL);
		}
	}
	rc = PV(c)->gnss_fixed(PV(c)->u, x, y, z);
	audit_action(c, a, req, "fixed-position", rc);
	if (rc != 0) {
		return fail(c, r, 409, "rejected", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_kbool(&w, "applied", true);
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_reference(rest_ctx_t *c, const http_req_t *req,
		       const rest_authctx_t *a, const char *body,
		       size_t body_len, web_resp_t *r, const char *tail,
		       size_t tail_len)
{
	web_json_val_t v;
	web_jw_t w;
	uint8_t mode;
	int rc;

	(void)tail;
	(void)tail_len;

	if (!have_pv(c) || PV(c)->ref_override == NULL) {
		return fail(c, r, 501, "no_provider", NULL);
	}
	if (body == NULL || web_json_obj_get(body, body_len, "mode", &v) != 0 ||
	    v.type != (uint8_t)WEB_JSON_STR) {
		return fail(c, r, 400, "missing_mode", NULL);
	}
	if (web_json_str_eq(&v, "auto")) {
		mode = (uint8_t)REST_REF_AUTO;
	} else if (web_json_str_eq(&v, "ocxo")) {
		mode = (uint8_t)REST_REF_OCXO;
	} else if (web_json_str_eq(&v, "rb")) {
		mode = (uint8_t)REST_REF_RB;
	} else {
		return fail(c, r, 400, "unknown_mode", NULL);
	}
	rc = PV(c)->ref_override(PV(c)->u, mode);
	audit_action(c, a, req, "reference-override", rc);
	if (rc == -EPERM) {
		/* The reference state machine refused: the guard conditions
		 * (extref in band AND rb_lock) are not satisfied. Never force. */
		return fail(c, r, 409, "guard_not_satisfied",
			    "the requested reference is not usable right now");
	}
	if (rc != 0) {
		return fail(c, r, 409, "rejected", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_ku64(&w, "mode", mode);
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_service(rest_ctx_t *c, const http_req_t *req,
		     const rest_authctx_t *a, const char *body, size_t body_len,
		     web_resp_t *r, const char *tail, size_t tail_len)
{
	static const char *const names[REST_SVC_COUNT] = {
		"ntp", "nts", "ptp", "snmp", "syslog",
	};
	web_json_val_t v;
	web_jw_t w;
	uint8_t svc = (uint8_t)REST_SVC_COUNT;
	size_t i;
	bool enable;
	int rc;

	if (!have_pv(c) || PV(c)->service_enable == NULL) {
		return fail(c, r, 501, "no_provider", NULL);
	}
	for (i = 0U; i < REST_SVC_COUNT; i++) {
		if (web_span_eq(tail, tail_len, names[i])) {
			svc = (uint8_t)i;
			break;
		}
	}
	if (svc >= (uint8_t)REST_SVC_COUNT) {
		c->st.not_found++;
		return fail(c, r, 404, "unknown_service", NULL);
	}
	if (body == NULL || web_json_obj_get(body, body_len, "enable", &v) != 0 ||
	    web_json_bool(&v, &enable) != 0) {
		return fail(c, r, 400, "missing_enable", NULL);
	}
	rc = PV(c)->service_enable(PV(c)->u, svc, enable);
	audit_action(c, a, req, enable ? "service-enable" : "service-disable", rc);
	if (rc != 0) {
		return fail(c, r, 409, "rejected", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_kstr(&w, "service", names[svc]);
	web_jw_kbool(&w, "enabled", enable);
	web_jw_kbool(&w, "commit_required",
		     (c->cfg != NULL) && (cfg_staged_count(c->cfg) != 0U));
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

/* ---------------------------------------------------------------- firmware */

static void jw_fw_slot(web_jw_t *w, const rest_fw_slot_t *s)
{
	char ver[32];
	size_t o = 0U;
	size_t i;

	web_jw_obj_begin(w);
	web_jw_ku64(w, "slot", s->slot);
	web_jw_ku64(w, "size", s->size);
	for (i = 0U; i < 4U; i++) {
		uint32_t v = s->version[i];
		char tmp[12];
		size_t k = 0U;

		do {
			tmp[k++] = (char)('0' + (v % 10U));
			v /= 10U;
		} while (v != 0U);
		while (k > 0U && o < (sizeof(ver) - 2U)) {
			ver[o++] = tmp[--k];
		}
		if (i != 3U && o < (sizeof(ver) - 2U)) {
			ver[o++] = '.';
		}
	}
	ver[o] = '\0';
	web_jw_kstr(w, "version", ver);
	web_jw_kbool(w, "valid", s->valid);
	web_jw_kbool(w, "active", s->active);
	web_jw_kbool(w, "pending", s->pending);
	web_jw_kbool(w, "confirmed", s->confirmed);
	web_jw_obj_end(w);
}

static int h_fw_info(rest_ctx_t *c, const http_req_t *req,
		     const rest_authctx_t *a, const char *body, size_t body_len,
		     web_resp_t *r, const char *tail, size_t tail_len)
{
	rest_fw_t fw;
	web_jw_t w;
	size_t i;

	(void)req;
	(void)a;
	(void)body;
	(void)body_len;
	(void)tail;
	(void)tail_len;

	if (!have_pv(c) || PV(c)->fw_info == NULL) {
		return fail(c, r, 501, "no_provider", NULL);
	}
	memset(&fw, 0, sizeof(fw));
	if (PV(c)->fw_info(PV(c)->u, &fw) != 0) {
		return fail(c, r, 503, "unavailable", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_karr(&w, "slots");
	for (i = 0U; i < fw.n_slots && i < 2U; i++) {
		jw_fw_slot(&w, &fw.slot[i]);
	}
	web_jw_arr_end(&w);
	web_jw_kobj(&w, "upload");
	web_jw_ku64(&w, "state", fw.dfu_state);
	web_jw_ku64(&w, "total", fw.dfu_total);
	web_jw_ku64(&w, "written", fw.dfu_written);
	web_jw_ku64(&w, "chunk_max", fw.chunk_max);
	web_jw_ku64(&w, "write_block", fw.write_block);
	web_jw_obj_end(&w);
	web_jw_kbool(&w, "pending_confirm", fw.pending_confirm);
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_fw_begin(rest_ctx_t *c, const http_req_t *req,
		      const rest_authctx_t *a, const char *body,
		      size_t body_len, web_resp_t *r, const char *tail,
		      size_t tail_len)
{
	web_json_val_t v;
	web_jw_t w;
	char hex[65];
	uint8_t sha[32];
	uint64_t size = 0U;
	uint32_t next = 0U;
	int rc;

	(void)tail;
	(void)tail_len;

	if (!have_pv(c) || PV(c)->fw_begin == NULL) {
		return fail(c, r, 501, "no_provider", NULL);
	}
	if (body == NULL || web_json_obj_get(body, body_len, "size", &v) != 0 ||
	    web_json_u64(&v, &size) != 0 || size == 0U || size > 0xFFFFFFFFULL) {
		return fail(c, r, 400, "bad_size", NULL);
	}
	if (web_json_obj_get(body, body_len, "sha256", &v) != 0 ||
	    v.type != (uint8_t)WEB_JSON_STR || v.n != 64U) {
		return fail(c, r, 400, "bad_sha256", NULL);
	}
	if (web_json_str_copy(&v, hex, sizeof(hex)) != 64 ||
	    web_hex_decode(hex, 64U, sha, sizeof(sha)) != 32) {
		return fail(c, r, 400, "bad_sha256", NULL);
	}

	rc = PV(c)->fw_begin(PV(c)->u, (uint32_t)size, sha, &next);
	audit_action(c, a, req, "fw-begin", rc);
	if (rc == -ENOSPC) {
		return fail(c, r, 413, "image_too_large",
			    "the image exceeds the staging slot");
	}
	if (rc != 0) {
		return fail(c, r, 409, "rejected", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_ku64(&w, "next_offset", next);
	web_jw_ku64(&w, "size", size);
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_fw_data(rest_ctx_t *c, const http_req_t *req,
		     const rest_authctx_t *a, const char *body, size_t body_len,
		     web_resp_t *r, const char *tail, size_t tail_len)
{
	web_jw_t w;
	uint32_t off = 0U;
	uint32_t next = 0U;
	int rc;

	(void)tail;
	(void)tail_len;

	if (!have_pv(c) || PV(c)->fw_data == NULL) {
		return fail(c, r, 501, "no_provider", NULL);
	}
	if (http_query_get_u32(req, "offset", &off) != 0) {
		return fail(c, r, 400, "missing_offset", NULL);
	}
	if (body == NULL || body_len == 0U) {
		return fail(c, r, 400, "empty_chunk", NULL);
	}

	rc = PV(c)->fw_data(PV(c)->u, off, (const uint8_t *)body, body_len,
			    &next);
	c->st.fw_chunks++;
	if (rc == -EBADE) {
		/*
		 * Offset gap: answer 409 with the frontier so the client rewinds,
		 * mirroring MCP's ERR_OFFSET contract exactly.
		 */
		web_jw_init(&w, r->body, r->body_cap);
		web_jw_obj_begin(&w);
		web_jw_kstr(&w, "error", "offset_gap");
		web_jw_ku64(&w, "next_offset", next);
		web_jw_obj_end(&w);
		(void)web_jw_finish(&w, &r->body_len);
		r->status = 409U;
		r->content_type = "application/json";
		r->no_store = true;
		c->st.responses_4xx++;
		return 0;
	}
	if (rc != 0) {
		audit_action(c, a, req, "fw-data", rc);
		return fail(c, r, 500, "write_failed", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_ku64(&w, "next_offset", next);
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_fw_end(rest_ctx_t *c, const http_req_t *req,
		    const rest_authctx_t *a, const char *body, size_t body_len,
		    web_resp_t *r, const char *tail, size_t tail_len)
{
	web_jw_t w;
	uint32_t size = 0U;
	int rc;

	(void)body;
	(void)body_len;
	(void)tail;
	(void)tail_len;

	if (!have_pv(c) || PV(c)->fw_end == NULL) {
		return fail(c, r, 501, "no_provider", NULL);
	}
	rc = PV(c)->fw_end(PV(c)->u, &size);
	audit_action(c, a, req, "fw-end", rc);
	if (rc == -EBADMSG) {
		return fail(c, r, 422, "verify_failed",
			    "SHA-256 or MCUboot header mismatch; re-upload");
	}
	if (rc != 0) {
		return fail(c, r, 409, "rejected", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_ku64(&w, "size", size);
	web_jw_kbool(&w, "pending", true);
	web_jw_kstr(&w, "next", "reboot to test the staged image");
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_fw_confirm(rest_ctx_t *c, const http_req_t *req,
			const rest_authctx_t *a, const char *body,
			size_t body_len, web_resp_t *r, const char *tail,
			size_t tail_len)
{
	web_jw_t w;
	int rc;

	(void)body;
	(void)body_len;
	(void)tail;
	(void)tail_len;

	if (!have_pv(c) || PV(c)->fw_confirm == NULL) {
		return fail(c, r, 501, "no_provider", NULL);
	}
	rc = PV(c)->fw_confirm(PV(c)->u);
	audit_action(c, a, req, "fw-confirm", rc);
	if (rc != 0) {
		return fail(c, r, 409, "rejected", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_kbool(&w, "confirmed", true);
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_fw_revert(rest_ctx_t *c, const http_req_t *req,
		       const rest_authctx_t *a, const char *body,
		       size_t body_len, web_resp_t *r, const char *tail,
		       size_t tail_len)
{
	web_jw_t w;
	int rc;

	(void)body;
	(void)body_len;
	(void)tail;
	(void)tail_len;

	if (!have_pv(c) || PV(c)->fw_revert == NULL) {
		return fail(c, r, 501, "no_provider", NULL);
	}
	rc = PV(c)->fw_revert(PV(c)->u);
	audit_action(c, a, req, "fw-revert", rc);
	if (rc != 0) {
		return fail(c, r, 409, "rejected", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_kbool(&w, "revert_requested", true);
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

/* ---------------------------------------------------------------- security */

static int h_sec_users(rest_ctx_t *c, const http_req_t *req,
		       const rest_authctx_t *a, const char *body,
		       size_t body_len, web_resp_t *r, const char *tail,
		       size_t tail_len)
{
	web_jw_t w;
	size_t i;

	(void)req;
	(void)a;
	(void)body;
	(void)body_len;
	(void)tail;
	(void)tail_len;

	if (c->auth == NULL) {
		return fail(c, r, 501, "no_auth", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_karr(&w, "users");
	for (i = 0U; i < AUTH_WEB_USERS; i++) {
		const auth_web_user_t *u = auth_web_user_at(c->auth, i);

		if (u == NULL) {
			continue;
		}
		web_jw_obj_begin(&w);
		web_jw_ku64(&w, "index", i);
		web_jw_kstr(&w, "name", u->name);
		web_jw_kstr(&w, "role", web_role_name(u->role));
		web_jw_kbool(&w, "credential_set", u->has_blob);
		web_jw_kbool(&w, "persistent",
			     auth_web_user_persistent(c->auth, i));
		web_jw_ku64(&w, "failed_attempts", u->fails);
		web_jw_ku64(&w, "lockout_ms",
			    auth_web_retry_after_ms(c->auth, i, c->now_ms));
		web_jw_obj_end(&w);
	}
	web_jw_arr_end(&w);
	web_jw_kstr(&w, "kdf",
		    (c->auth->kdf != NULL && c->auth->kdf->name != NULL)
			    ? c->auth->kdf->name
			    : "unknown");
	web_jw_kbool(&w, "auth_required", auth_web_required(c->auth));
	web_jw_ku64(&w, "sessions", auth_web_session_count(c->auth));
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_sec_password(rest_ctx_t *c, const http_req_t *req,
			  const rest_authctx_t *a, const char *body,
			  size_t body_len, web_resp_t *r, const char *tail,
			  size_t tail_len)
{
	web_json_val_t vu;
	web_json_val_t vp;
	web_jw_t w;
	char pw[AUTH_WEB_PW_MAX + 1U];
	int uidx;
	int n;
	int rc;

	(void)tail;
	(void)tail_len;

	if (c->auth == NULL) {
		return fail(c, r, 501, "no_auth", NULL);
	}
	if (body == NULL || web_json_obj_get(body, body_len, "user", &vu) != 0 ||
	    vu.type != (uint8_t)WEB_JSON_STR) {
		return fail(c, r, 400, "missing_user", NULL);
	}
	if (web_json_obj_get(body, body_len, "password", &vp) != 0 ||
	    vp.type != (uint8_t)WEB_JSON_STR) {
		return fail(c, r, 400, "missing_password", NULL);
	}
	uidx = auth_web_user_find(c->auth, vu.p, vu.n);
	if (uidx < 0) {
		c->st.not_found++;
		return fail(c, r, 404, "unknown_user", NULL);
	}
	n = web_json_str_copy(&vp, pw, sizeof(pw));
	if (n < 0) {
		return fail(c, r, 400, "bad_password", NULL);
	}
	rc = auth_web_set_password(c->auth, (size_t)uidx, (const uint8_t *)pw,
				   (size_t)n);
	/* Scrub before any early return so the plaintext lifetime is minimal. */
	memset(pw, 0, sizeof(pw));
	audit_action(c, a, req, "set-password", rc);
	if (rc == -EINVAL) {
		return fail(c, r, 422, "password_policy",
			    "8..64 bytes required");
	}
	if (rc != 0) {
		return fail(c, r, 500, "store_failed", NULL);
	}

	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_kbool(&w, "staged", true);
	web_jw_kbool(&w, "commit_required",
		     auth_web_user_persistent(c->auth, (size_t)uidx));
	web_jw_kstr(&w, "note",
		    "credential is staged; POST /api/v1/config/commit to persist");
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_sec_tls_get(rest_ctx_t *c, const http_req_t *req,
			 const rest_authctx_t *a, const char *body,
			 size_t body_len, web_resp_t *r, const char *tail,
			 size_t tail_len)
{
	rest_cert_t ci;
	web_jw_t w;

	(void)req;
	(void)a;
	(void)body;
	(void)body_len;
	(void)tail;
	(void)tail_len;

	if (!have_pv(c) || PV(c)->cert_info == NULL) {
		return fail(c, r, 501, "no_provider", NULL);
	}
	memset(&ci, 0, sizeof(ci));
	if (PV(c)->cert_info(PV(c)->u, &ci) != 0) {
		return fail(c, r, 503, "unavailable", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_kbool(&w, "present", ci.present);
	web_jw_kbool(&w, "self_signed", ci.self_signed);
	web_jw_kbool(&w, "operator_supplied", ci.operator_supplied);
	web_jw_kbool(&w, "persisted", ci.persisted);
	web_jw_kstr(&w, "subject", ci.subject);
	web_jw_kstr(&w, "issuer", ci.issuer);
	web_jw_kstr(&w, "not_before", ci.not_before);
	web_jw_kstr(&w, "not_after", ci.not_after);
	web_jw_kstr(&w, "sha256_fingerprint", ci.sha256_fp);
	web_jw_kstr(&w, "key_type", ci.key_type);
	web_jw_ku64(&w, "key_bits", ci.key_bits);
	web_jw_kobj(&w, "acme");
	web_jw_ku64(&w, "state", ci.acme_state);
	web_jw_kbool(&w, "enabled", ci.acme_state != (uint8_t)REST_ACME_DISABLED);
	web_jw_kstr(&w, "detail", ci.acme_detail);
	web_jw_obj_end(&w);
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_sec_tls_post(rest_ctx_t *c, const http_req_t *req,
			  const rest_authctx_t *a, const char *body,
			  size_t body_len, web_resp_t *r, const char *tail,
			  size_t tail_len)
{
	web_jw_t w;
	int rc;

	(void)tail;
	(void)tail_len;

	if (!have_pv(c) || PV(c)->cert_install == NULL) {
		return fail(c, r, 501, "no_provider", NULL);
	}
	if (body == NULL || body_len == 0U) {
		return fail(c, r, 400, "empty_body", NULL);
	}
	rc = PV(c)->cert_install(PV(c)->u, body, body_len);
	audit_action(c, a, req, "tls-install", rc);
	if (rc == -EBADMSG) {
		return fail(c, r, 422, "bad_pem",
			    "expected a PEM certificate and matching private key");
	}
	if (rc == -EKEYREJECTED) {
		return fail(c, r, 422, "key_mismatch",
			    "the private key does not match the certificate");
	}
	if (rc != 0) {
		return fail(c, r, 500, "install_failed", NULL);
	}
	/* A new server key invalidates every pinned session. */
	if (c->auth != NULL) {
		auth_web_logout_all(c->auth);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_kbool(&w, "installed", true);
	web_jw_kstr(&w, "note",
		    "restart the HTTPS listener (or reboot) to serve the new "
		    "certificate; all sessions were closed");
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_sec_csr(rest_ctx_t *c, const http_req_t *req,
		     const rest_authctx_t *a, const char *body, size_t body_len,
		     web_resp_t *r, const char *tail, size_t tail_len)
{
	web_json_val_t v;
	char subject[96];
	size_t out_len = 0U;
	int rc;

	(void)tail;
	(void)tail_len;

	if (!have_pv(c) || PV(c)->csr_make == NULL) {
		return fail(c, r, 501, "no_provider", NULL);
	}
	subject[0] = '\0';
	if (body != NULL && body_len != 0U &&
	    web_json_obj_get(body, body_len, "subject", &v) == 0 &&
	    v.type == (uint8_t)WEB_JSON_STR) {
		if (web_json_str_copy(&v, subject, sizeof(subject)) < 0) {
			return fail(c, r, 400, "bad_subject", NULL);
		}
	}
	rc = PV(c)->csr_make(PV(c)->u, (subject[0] != '\0') ? subject : NULL,
			     r->body, r->body_cap, &out_len);
	audit_action(c, a, req, "tls-csr", rc);
	if (rc == -ENOSPC) {
		return fail(c, r, 500, "csr_too_large", NULL);
	}
	if (rc != 0) {
		return fail(c, r, 500, "csr_failed", NULL);
	}
	r->body_len = out_len;
	r->status = 200U;
	r->content_type = "application/pkcs10";
	r->no_store = true;
	resp_add_kv(r, "Content-Disposition",
		    "attachment; filename=\"meridian.csr\"");
	c->st.responses_2xx++;
	return 0;
}

/* -------------------------------------------------------------- auth routes */

static int h_auth_login(rest_ctx_t *c, const http_req_t *req,
			const rest_authctx_t *a, const char *body,
			size_t body_len, web_resp_t *r, const char *tail,
			size_t tail_len)
{
	web_json_val_t vu;
	web_json_val_t vp;
	auth_web_grant_t g;
	web_jw_t w;
	char pw[AUTH_WEB_PW_MAX + 1U];
	char cookie[128];
	int n;
	int rc;

	(void)a;
	(void)req;
	(void)tail;
	(void)tail_len;

	if (c->auth == NULL) {
		return fail(c, r, 501, "no_auth", NULL);
	}
	if (body == NULL || web_json_obj_get(body, body_len, "user", &vu) != 0 ||
	    vu.type != (uint8_t)WEB_JSON_STR ||
	    web_json_obj_get(body, body_len, "password", &vp) != 0 ||
	    vp.type != (uint8_t)WEB_JSON_STR) {
		return fail(c, r, 400, "missing_credentials", NULL);
	}
	n = web_json_str_copy(&vp, pw, sizeof(pw));
	if (n < 0) {
		return fail(c, r, 400, "bad_password", NULL);
	}

	memset(&g, 0, sizeof(g));
	rc = auth_web_login(c->auth, vu.p, vu.n, (const uint8_t *)pw, (size_t)n,
			    c->now_ms, &g);
	memset(pw, 0, sizeof(pw));

	if (rc == -EBUSY) {
		int uidx = auth_web_user_find(c->auth, vu.p, vu.n);
		uint32_t ms = (uidx >= 0)
				      ? auth_web_retry_after_ms(c->auth,
								(size_t)uidx,
								c->now_ms)
				      : AUTH_WEB_LOCKOUT_MS;
		char after[8];
		uint32_t secs = (ms + 999U) / 1000U;
		size_t o = 0U;
		char tmp[8];
		size_t k = 0U;

		if (secs == 0U) {
			secs = 1U;
		}
		do {
			tmp[k++] = (char)('0' + (secs % 10U));
			secs /= 10U;
		} while (k < sizeof(tmp) && secs != 0U);
		while (k > 0U) {
			after[o++] = tmp[--k];
		}
		after[o] = '\0';
		(void)fail(c, r, 429, "throttled",
			   "too many failed attempts; retry later");
		resp_add_kv(r, "Retry-After", after);
		c->st.unauthorized++;
		return 0;
	}
	if (rc == -ENOKEY) {
		c->st.unauthorized++;
		return fail(c, r, 503, "no_credential",
			    "no administrator credential is provisioned; set one "
			    "over the local UI or the USB console");
	}
	if (rc != 0) {
		c->st.unauthorized++;
		return fail(c, r, 401, "invalid_credentials", NULL);
	}

	/*
	 * Session cookie: HttpOnly (no script access), Secure (HTTPS only),
	 * SameSite=Strict (a cross-site POST carries no cookie at all, which is
	 * defence in depth behind the CSRF token), Path=/ and no Max-Age so the
	 * cookie dies with the browser session.
	 */
	{
		size_t o = 0U;
		const char *pfx = "sts_session=";
		size_t pn = strlen(pfx);
		const char *sfx = "; Path=/; HttpOnly; Secure; SameSite=Strict";

		memcpy(&cookie[o], pfx, pn);
		o += pn;
		o += web_span_copy(&cookie[o], sizeof(cookie) - o, g.token,
				   strlen(g.token));
		(void)web_span_copy(&cookie[o], sizeof(cookie) - o, sfx,
				    strlen(sfx));
	}
	resp_add_kv(r, "Set-Cookie", cookie);

	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_kstr(&w, "role", web_role_name(g.role));
	web_jw_kstr(&w, "csrf_token", g.csrf);
	web_jw_ku64(&w, "idle_timeout_s", g.idle_s);
	web_jw_ku64(&w, "absolute_timeout_s", g.absolute_s);
	/*
	 * The session token is deliberately NOT in the body: putting it there
	 * invites the SPA to keep it in localStorage, which is exactly what
	 * HttpOnly exists to prevent. Non-browser clients get one from the
	 * Set-Cookie header and send it as a bearer token.
	 */
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_auth_logout(rest_ctx_t *c, const http_req_t *req,
			 const rest_authctx_t *a, const char *body,
			 size_t body_len, web_resp_t *r, const char *tail,
			 size_t tail_len)
{
	web_jw_t w;

	(void)req;
	(void)body;
	(void)body_len;
	(void)tail;
	(void)tail_len;

	if (c->auth == NULL) {
		return fail(c, r, 501, "no_auth", NULL);
	}
	if (a->have_session) {
		(void)auth_web_logout(c->auth, a->sess);
	}
	resp_add_kv(r, "Set-Cookie",
		    "sts_session=; Path=/; HttpOnly; Secure; SameSite=Strict; "
		    "Max-Age=0");
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_kbool(&w, "logged_out", true);
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

static int h_auth_session(rest_ctx_t *c, const http_req_t *req,
			  const rest_authctx_t *a, const char *body,
			  size_t body_len, web_resp_t *r, const char *tail,
			  size_t tail_len)
{
	const auth_web_sess_t *s;
	web_jw_t w;

	(void)req;
	(void)body;
	(void)body_len;
	(void)tail;
	(void)tail_len;

	if (c->auth == NULL) {
		return fail(c, r, 501, "no_auth", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_kbool(&w, "authenticated", a->have_session);
	web_jw_kstr(&w, "role", web_role_name(a->role));
	web_jw_kbool(&w, "auth_required", auth_web_required(c->auth));
	if (a->have_session) {
		s = auth_web_sess_at(c->auth, a->sess);
		if (s != NULL) {
			const auth_web_user_t *u =
				auth_web_user_at(c->auth, s->user);

			web_jw_kstr(&w, "user", (u != NULL) ? u->name : "");
			web_jw_kstr(&w, "csrf_token", s->csrf);
			web_jw_ku64(&w, "age_s",
				    (c->now_ms - s->created_ms) / 1000U);
			web_jw_ku64(&w, "requests", s->requests);
		}
	}
	web_jw_obj_end(&w);
	r->no_store = true;
	return ok_json(c, r, &w);
}

/* ---------------------------------------------------------------- system */

static int h_reboot(rest_ctx_t *c, const http_req_t *req,
		    const rest_authctx_t *a, const char *body, size_t body_len,
		    web_resp_t *r, const char *tail, size_t tail_len)
{
	web_json_val_t v;
	web_jw_t w;
	uint64_t mode = 0U;
	int rc;

	(void)tail;
	(void)tail_len;

	if (!have_pv(c) || PV(c)->reboot == NULL) {
		return fail(c, r, 501, "no_provider", NULL);
	}
	if (body != NULL && body_len != 0U &&
	    web_json_obj_get(body, body_len, "mode", &v) == 0) {
		if (web_json_u64(&v, &mode) != 0 || mode > 2U) {
			return fail(c, r, 400, "bad_mode", NULL);
		}
	}
	rc = PV(c)->reboot(PV(c)->u, (uint8_t)mode);
	audit_action(c, a, req, "reboot", rc);
	if (rc != 0) {
		return fail(c, r, 409, "rejected", NULL);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_kbool(&w, "rebooting", true);
	web_jw_ku64(&w, "mode", mode);
	web_jw_obj_end(&w);
	r->no_store = true;
	r->close = true;
	return ok_json(c, r, &w);
}

static int h_factory_reset(rest_ctx_t *c, const http_req_t *req,
			   const rest_authctx_t *a, const char *body,
			   size_t body_len, web_resp_t *r, const char *tail,
			   size_t tail_len)
{
	web_json_val_t v;
	web_jw_t w;
	int rc;

	(void)tail;
	(void)tail_len;

	if (!have_pv(c) || PV(c)->factory_reset == NULL) {
		return fail(c, r, 501, "no_provider", NULL);
	}
	/* The same explicit magic MCP demands, so a stray POST cannot wipe a
	 * deployed box. */
	if (body == NULL || web_json_obj_get(body, body_len, "confirm", &v) != 0 ||
	    !web_json_str_eq(&v, "FACTORY")) {
		return fail(c, r, 400, "confirmation_required",
			    "body must contain {\"confirm\":\"FACTORY\"}");
	}
	rc = PV(c)->factory_reset(PV(c)->u);
	audit_action(c, a, req, "factory-reset", rc);
	if (rc != 0) {
		return fail(c, r, 409, "rejected", NULL);
	}
	if (c->auth != NULL) {
		auth_web_logout_all(c->auth);
	}
	web_jw_init(&w, r->body, r->body_cap);
	web_jw_obj_begin(&w);
	web_jw_kbool(&w, "reset", true);
	web_jw_obj_end(&w);
	r->no_store = true;
	r->close = true;
	return ok_json(c, r, &w);
}

/* ------------------------------------------------------------------------- */
/* the route table                                                           */
/* ------------------------------------------------------------------------- */

typedef struct {
	const char     *path; /**< relative to WEB_API_PREFIX */
	uint8_t         method;
	uint8_t         min_role;
	uint8_t         flags;
	rest_handler_fn fn;
} rest_route_t;

static const rest_route_t routes[] = {
	/* --- read-only -------------------------------------------------- */
	{ "status", WEB_METHOD_GET, WEB_ROLE_VIEWER, RF_PREFIX, h_status },
	{ "metrics", WEB_METHOD_GET, WEB_ROLE_VIEWER, 0U, h_metrics },
	{ "logs", WEB_METHOD_GET, WEB_ROLE_VIEWER, 0U, h_logs },
	{ "firmware", WEB_METHOD_GET, WEB_ROLE_VIEWER, 0U, h_fw_info },
	{ "calibration", WEB_METHOD_GET, WEB_ROLE_VIEWER, 0U, h_cal_get },
	{ "auth/session", WEB_METHOD_GET, WEB_ROLE_NONE, RF_OPEN,
	  h_auth_session },

	/* --- authentication --------------------------------------------- */
	{ "auth/login", WEB_METHOD_POST, WEB_ROLE_NONE, RF_OPEN, h_auth_login },
	{ "auth/logout", WEB_METHOD_POST, WEB_ROLE_NONE, RF_OPEN,
	  h_auth_logout },

	/* --- config ----------------------------------------------------- */
	{ "config/commit", WEB_METHOD_POST, WEB_ROLE_OPERATOR, 0U,
	  h_config_commit },
	{ "config/revert", WEB_METHOD_POST, WEB_ROLE_OPERATOR, 0U,
	  h_config_revert },
	{ "config/export", WEB_METHOD_GET, WEB_ROLE_ADMIN, 0U, h_config_export },
	{ "config/import", WEB_METHOD_POST, WEB_ROLE_ADMIN, 0U,
	  h_config_import },
	{ "config", WEB_METHOD_GET, WEB_ROLE_VIEWER, 0U, h_config_list },
	{ "config", WEB_METHOD_PUT, WEB_ROLE_OPERATOR, 0U, h_config_put },
	{ "config/", WEB_METHOD_GET, WEB_ROLE_VIEWER, RF_PREFIX,
	  h_config_get_one },

	/* --- control ---------------------------------------------------- */
	{ "calibration/run", WEB_METHOD_POST, WEB_ROLE_OPERATOR, 0U,
	  h_cal_run },
	{ "gnss/survey", WEB_METHOD_POST, WEB_ROLE_OPERATOR, 0U,
	  h_gnss_survey },
	{ "gnss/position", WEB_METHOD_POST, WEB_ROLE_OPERATOR, 0U,
	  h_gnss_position },
	{ "timing/reference", WEB_METHOD_POST, WEB_ROLE_OPERATOR, 0U,
	  h_reference },
	{ "services/", WEB_METHOD_POST, WEB_ROLE_OPERATOR,
	  RF_PREFIX, h_service },

	/* --- firmware (admin only) -------------------------------------- */
	{ "firmware/begin", WEB_METHOD_POST, WEB_ROLE_ADMIN, 0U,
	  h_fw_begin },
	{ "firmware/data", WEB_METHOD_POST, WEB_ROLE_ADMIN, 0U, h_fw_data },
	{ "firmware/end", WEB_METHOD_POST, WEB_ROLE_ADMIN, 0U, h_fw_end },
	{ "firmware/confirm", WEB_METHOD_POST, WEB_ROLE_ADMIN, 0U,
	  h_fw_confirm },
	{ "firmware/revert", WEB_METHOD_POST, WEB_ROLE_ADMIN, 0U,
	  h_fw_revert },

	/* --- security (admin only) -------------------------------------- */
	{ "security/users", WEB_METHOD_GET, WEB_ROLE_ADMIN, 0U, h_sec_users },
	{ "security/password", WEB_METHOD_POST, WEB_ROLE_ADMIN, 0U,
	  h_sec_password },
	{ "security/tls", WEB_METHOD_GET, WEB_ROLE_ADMIN, 0U, h_sec_tls_get },
	{ "security/tls", WEB_METHOD_POST, WEB_ROLE_ADMIN, 0U,
	  h_sec_tls_post },
	{ "security/csr", WEB_METHOD_POST, WEB_ROLE_ADMIN, 0U, h_sec_csr },

	/* --- system (admin only) --------------------------------------- */
	{ "reboot", WEB_METHOD_POST, WEB_ROLE_ADMIN, 0U, h_reboot },
	{ "factory-reset", WEB_METHOD_POST, WEB_ROLE_ADMIN, 0U,
	  h_factory_reset },
};

#define ROUTE_COUNT (sizeof(routes) / sizeof(routes[0]))

/* ------------------------------------------------------------------------- */
/* dispatch                                                                  */
/* ------------------------------------------------------------------------- */

int rest_init(rest_ctx_t *c, cfg_ctx_t *cfg, logr_t *log, auth_web_ctx_t *auth,
	      const rest_providers_t *pv)
{
	if (c == NULL) {
		return -EINVAL;
	}
	memset(c, 0, sizeof(*c));
	c->cfg = cfg;
	c->log = log;
	c->auth = auth;
	c->pv = pv;
	return 0;
}

bool rest_is_api_path(const http_req_t *req)
{
	if (req == NULL || req->path.p == NULL) {
		return false;
	}
	if (req->path.n < WEB_API_PREFIX_LEN) {
		return false;
	}
	return memcmp(req->path.p, WEB_API_PREFIX, WEB_API_PREFIX_LEN) == 0;
}

void rest_resolve_auth(rest_ctx_t *c, const http_req_t *req,
		       rest_authctx_t *out)
{
	char token[AUTH_WEB_TOKEN_LEN + 1U];
	size_t sess = 0U;

	if (out == NULL) {
		return;
	}
	memset(out, 0, sizeof(*out));
	out->role = (uint8_t)WEB_ROLE_NONE;
	if (c == NULL || req == NULL || c->auth == NULL) {
		return;
	}

	if (http_cookie_get(req, "sts_session", token, sizeof(token)) <= 0) {
		if (!http_bearer_token(req, token, sizeof(token))) {
			return;
		}
	}
	if (auth_web_validate(c->auth, token, c->now_ms, &sess) != 0) {
		return;
	}
	{
		const auth_web_sess_t *s = auth_web_sess_at(c->auth, sess);

		if (s == NULL) {
			return;
		}
		out->have_session = true;
		out->sess = sess;
		out->role = s->role;
	}
}

/*
 * Find the route for this request. Sets @p tail and @p tail_len for prefix
 * routes. Returns the route, or NULL. @p method_mismatch is set when some route
 * matched the path but not the method (405 rather than 404).
 */
static const rest_route_t *find_route(const http_req_t *req, const char **tail,
				      size_t *tail_len, bool *method_mismatch)
{
	const char *p = &req->path.p[WEB_API_PREFIX_LEN];
	size_t n = (size_t)req->path.n - WEB_API_PREFIX_LEN;
	size_t i;

	*method_mismatch = false;
	*tail = p;
	*tail_len = 0U;

	/* Exact matches win over prefix matches, so /config/commit is not
	 * swallowed by the /config/ prefix route. */
	for (i = 0U; i < ROUTE_COUNT; i++) {
		if ((routes[i].flags & RF_PREFIX) != 0U) {
			continue;
		}
		if (!web_span_eq(p, n, routes[i].path)) {
			continue;
		}
		if (routes[i].method != req->method) {
			/* HEAD is answered by the GET handler; the caller drops
			 * the body. */
			if (!(req->method == (uint8_t)WEB_METHOD_HEAD &&
			      routes[i].method == (uint8_t)WEB_METHOD_GET)) {
				*method_mismatch = true;
				continue;
			}
		}
		*tail_len = 0U;
		return &routes[i];
	}

	for (i = 0U; i < ROUTE_COUNT; i++) {
		size_t plen;

		if ((routes[i].flags & RF_PREFIX) == 0U) {
			continue;
		}
		plen = strlen(routes[i].path);
		if (n < plen || memcmp(p, routes[i].path, plen) != 0) {
			continue;
		}
		/*
		 * A prefix route whose pattern does not end in '/' must be
		 * followed by '/' or end the path, so "statusfoo" does not
		 * match "status".
		 */
		if (routes[i].path[plen - 1U] != '/' && n > plen) {
			if (p[plen] != '/') {
				continue;
			}
		}
		if (routes[i].method != req->method) {
			if (!(req->method == (uint8_t)WEB_METHOD_HEAD &&
			      routes[i].method == (uint8_t)WEB_METHOD_GET)) {
				*method_mismatch = true;
				continue;
			}
		}
		if (n == plen) {
			*tail = &p[plen];
			*tail_len = 0U;
		} else if (routes[i].path[plen - 1U] == '/') {
			*tail = &p[plen];
			*tail_len = n - plen;
		} else {
			*tail = &p[plen + 1U];
			*tail_len = n - plen - 1U;
		}
		return &routes[i];
	}
	return NULL;
}

int rest_dispatch(rest_ctx_t *c, const http_req_t *req, const char *body,
		  size_t body_len, web_resp_t *resp)
{
	const rest_route_t *rt;
	rest_authctx_t actx;
	const char *tail = NULL;
	size_t tail_len = 0U;
	bool mismatch = false;

	if (c == NULL || req == NULL || resp == NULL || resp->body == NULL) {
		return -EINVAL;
	}
	c->st.requests++;

	if (!rest_is_api_path(req)) {
		c->st.not_found++;
		return fail(c, resp, 404, "not_found", NULL);
	}
	if (req->method == (uint8_t)WEB_METHOD_UNKNOWN) {
		return fail(c, resp, 405, "method_not_allowed", NULL);
	}
	if (req->method == (uint8_t)WEB_METHOD_OPTIONS) {
		/*
		 * No CORS headers, on purpose: this API is same-origin only.
		 * Answering an OPTIONS preflight with permissive headers would
		 * hand a hostile page exactly the capability SameSite=Strict and
		 * the CSRF token exist to deny.
		 */
		resp->status = 204U;
		resp->body_len = 0U;
		resp->no_store = true;
		resp_add_kv(resp, "Allow", "GET, HEAD, POST, PUT, OPTIONS");
		c->st.responses_2xx++;
		return 0;
	}

	rest_resolve_auth(c, req, &actx);

	rt = find_route(req, &tail, &tail_len, &mismatch);
	if (rt == NULL) {
		if (mismatch) {
			(void)fail(c, resp, 405, "method_not_allowed", NULL);
			resp_add_kv(resp, "Allow", "GET, HEAD, POST, PUT");
			return 0;
		}
		c->st.not_found++;
		return fail(c, resp, 404, "no_such_route", NULL);
	}

	/* ---- authorisation ------------------------------------------------ */
	if ((rt->flags & RF_OPEN) == 0U) {
		bool mutating = web_method_is_mutating(req->method);

		if (!actx.have_session) {
			/*
			 * Invariant 1 (rest.h): `sec.auth.req = 0` relaxes READ
			 * access only. A mutating route always demands a session,
			 * because the risk it guards against (a hostile page in
			 * the operator's browser) does not go away when the
			 * operator turns login off.
			 */
			if (mutating || (c->auth != NULL &&
					 auth_web_required(c->auth))) {
				c->st.unauthorized++;
				(void)fail(c, resp, 401, "unauthenticated",
					   NULL);
				resp_add_kv(resp, "WWW-Authenticate",
					    "Bearer realm=\"meridian\"");
				return 0;
			}
			/* Open-read build: treat the caller as a viewer. */
			actx.role = (uint8_t)WEB_ROLE_VIEWER;
		}
		if (actx.role < rt->min_role) {
			c->st.forbidden++;
			return fail(c, resp, 403, "insufficient_role",
				    "this route requires a higher role");
		}
		if (mutating) {
			int rc;

			if (!actx.have_session) {
				c->st.unauthorized++;
				return fail(c, resp, 401, "unauthenticated",
					    NULL);
			}
			rc = auth_web_check_csrf(c->auth, actx.sess,
						 req->csrf.p, req->csrf.n);
			if (rc != 0) {
				c->st.csrf_failures++;
				return fail(c, resp, 403, "csrf_failed",
					    "send the session's csrf_token in "
					    "the X-CSRF-Token header");
			}
		}
	}

	return rt->fn(c, req, &actx, body, body_len, resp, tail, tail_len);
}
