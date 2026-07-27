/*
 * STS1000 "Meridian" — the `sts` Zephyr shell command set on CDC-ACM #0.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Spec §7.1 wants "full command/control parity with the web API for
 * headless/field use" over the driverless CDC-ACM port. This wave lands the
 * read-only observation set plus the mutations an operator needs at a bench:
 * configuration, credential provisioning, firmware confirm/revert and reboot.
 * The `clock`, `gnss`, `net` and `cal` groups belong to the areas that own
 * that state and land with them.
 *
 * Who may mutate
 * --------------
 * Two gates, and the second one matters more than it looks:
 *
 *   CONFIG_STS1000_SHELL_MUTATING  compile-time, default y. A production image
 *                                  can be cut with an observation-only shell.
 *
 *   cfg key sec.console.ro         runtime, default 1 — but only enforced once
 *                                  an admin credential exists.
 *
 * That exemption is deliberate and is what keeps a factory-fresh box usable.
 * MCP refuses every mutating command until sec.admin.pw holds a credential
 * (core/mcp h_auth: "a box with auth required and no password is locked, not
 * wide open"), and the local UI cannot type a password. If the physical
 * console also refused, a new board would have no provisioning path at all.
 * So: an unprovisioned box accepts `sts sec passwd`, and from the moment a
 * credential exists sec.console.ro is honoured exactly as written.
 */

#include <zephyr/kernel.h>

#ifdef CONFIG_STS1000_CONSOLE

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/dfu/mcuboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include "cfg/cfg.h"
#include "console/sts_console.h"
#include "fault/fault.h"
#include "logring/logring.h"
#include "mcp/mcp.h"
#include "quality/quality.h"
#include "storage/sts_store.h"
#include "zephyr/sts_app.h"

/*
 * Self-detecting Kconfig presence. This area's Kconfig fragment lives in
 * src/zephyr/console/Kconfig, which app/Kconfig has to `rsource`. Until it
 * does, CONFIG_STS1000_SHELL_MUTATING is simply undefined — and a plain
 * #ifdef would then read "not set", inverting a symbol whose default is y.
 * The hidden sentinel distinguishes "sourced and turned off" from "not
 * sourced at all".
 */
#if defined(CONFIG_STS1000_CONSOLE_KCONFIG)
#define SHELL_MUTATING IS_ENABLED(CONFIG_STS1000_SHELL_MUTATING)
#else
#define SHELL_MUTATING 1
#endif

/** Records `sts log tail` prints per invocation when no count is given. */
#define LOG_TAIL_DEFAULT 20
#define LOG_TAIL_MAX     200

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */

static bool credential_provisioned(void)
{
	uint8_t blob[MCP_PW_BLOB_LEN];
	size_t len = 0U;

	if (cfg_get_bytes(sts_cfg(), (uint16_t)CFG_ID_SEC_ADMIN_PW, blob,
			  sizeof(blob), &len) != 0) {
		return false;
	}
	return len == MCP_PW_BLOB_LEN;
}

/* Returns true when the command may proceed; prints the reason when not. */
static bool mutating_allowed(const struct shell *sh)
{
	bool ro = true;

	if (!SHELL_MUTATING) {
		shell_error(sh, "mutating shell commands are disabled in this "
				"build (CONFIG_STS1000_SHELL_MUTATING=n)");
		return false;
	}

	if (!credential_provisioned()) {
		return true; /* first-boot provisioning window, see the banner */
	}

	if (cfg_get_bool(sts_cfg(), (uint16_t)CFG_ID_SEC_CONSOLE_RO, &ro) != 0) {
		ro = true; /* fail closed */
	}
	if (ro) {
		shell_error(sh, "console is read-only (sec.console.ro=1); use "
				"the MCP channel with AUTH to change it");
		return false;
	}
	return true;
}

/* Format a float without pulling %f into the shell's printf. */
static const char *fmt_f32(char *buf, size_t cap, float v, int decimals)
{
	int64_t scale = 1;
	int64_t scaled;
	bool neg = false;

	for (int i = 0; i < decimals; i++) {
		scale *= 10;
	}
	if (v < 0.0f) {
		neg = true;
		v = -v;
	}
	scaled = (int64_t)((double)v * (double)scale + 0.5);

	(void)snprintf(buf, cap, "%s%lld.%0*lld", neg ? "-" : "",
		       (long long)(scaled / scale), decimals,
		       (long long)(scaled % scale));
	return buf;
}

static void hexdump(const struct shell *sh, const uint8_t *p, size_t n)
{
	char line[3 * 16 + 1];

	for (size_t i = 0; i < n; i += 16U) {
		size_t chunk = MIN((size_t)16U, n - i);
		size_t o = 0U;

		for (size_t j = 0; j < chunk; j++) {
			o += (size_t)snprintf(&line[o], sizeof(line) - o,
					      "%02x ", p[i + j]);
		}
		shell_print(sh, "  %04zx  %s", i, line);
	}
}

static int parse_u64(const char *s, uint64_t *out)
{
	char *end = NULL;
	unsigned long long v;

	errno = 0;
	v = strtoull(s, &end, 0);
	if ((end == s) || (*end != '\0') || (errno != 0)) {
		return -EINVAL;
	}
	*out = (uint64_t)v;
	return 0;
}

static int parse_i64(const char *s, int64_t *out)
{
	char *end = NULL;
	long long v;

	errno = 0;
	v = strtoll(s, &end, 0);
	if ((end == s) || (*end != '\0') || (errno != 0)) {
		return -EINVAL;
	}
	*out = (int64_t)v;
	return 0;
}

/*
 * Decimal float parser built on integer arithmetic. strtof() would work, but
 * the shell deliberately does not enable CONFIG_CBPRINTF_FP_SUPPORT (it is a
 * global cost paid by every log line), so keeping both directions integer-only
 * avoids half-enabling float formatting for one calibration key.
 */
static int parse_f32(const char *s, float *out)
{
	int64_t whole = 0;
	int64_t frac = 0;
	int64_t div = 1;
	bool neg = false;
	bool any = false;

	if (*s == '-') {
		neg = true;
		s++;
	} else if (*s == '+') {
		s++;
	}

	while ((*s >= '0') && (*s <= '9')) {
		whole = (whole * 10) + (*s - '0');
		if (whole > 1000000000LL) {
			return -ERANGE;
		}
		s++;
		any = true;
	}
	if (*s == '.') {
		s++;
		while ((*s >= '0') && (*s <= '9')) {
			if (div < 1000000LL) {
				frac = (frac * 10) + (*s - '0');
				div *= 10;
			}
			s++;
			any = true;
		}
	}
	if (!any || (*s != '\0')) {
		return -EINVAL;
	}

	*out = (float)whole + ((float)frac / (float)div);
	if (neg) {
		*out = -*out;
	}
	return 0;
}

static int parse_hex(const char *s, uint8_t *out, size_t cap, size_t *out_len)
{
	size_t n = strlen(s);
	size_t i;

	if (((n % 2U) != 0U) || ((n / 2U) > cap)) {
		return -EINVAL;
	}
	for (i = 0; i < (n / 2U); i++) {
		char pair[3] = { s[2 * i], s[(2 * i) + 1], '\0' };
		char *end = NULL;
		unsigned long v = strtoul(pair, &end, 16);

		if (*end != '\0') {
			return -EINVAL;
		}
		out[i] = (uint8_t)v;
	}
	*out_len = n / 2U;
	return 0;
}

static const cfg_key_t *resolve_key(const char *token)
{
	const cfg_key_t *k;
	size_t n = cfg_key_count();
	uint64_t id;

	for (size_t i = 0; i < n; i++) {
		k = cfg_key_at(i);
		if ((k != NULL) && (strcmp(k->name, token) == 0)) {
			return k;
		}
	}

	if ((parse_u64(token, &id) == 0) && (id <= 0xFFFFU)) {
		return cfg_key_find((uint16_t)id);
	}
	return NULL;
}

static const char *type_name(uint8_t t)
{
	switch (t) {
	case CFG_T_BOOL:
		return "bool";
	case CFG_T_U8:
		return "u8";
	case CFG_T_U16:
		return "u16";
	case CFG_T_U32:
		return "u32";
	case CFG_T_U64:
		return "u64";
	case CFG_T_I32:
		return "i32";
	case CFG_T_F32:
		return "f32";
	case CFG_T_STR:
		return "str";
	case CFG_T_BLOB:
		return "blob";
	default:
		return "?";
	}
}

static void format_value(const cfg_key_t *k, const cfg_val_t *v, char *buf,
			 size_t cap)
{
	if ((k->flags & CFG_F_SECRET) != 0U) {
		(void)snprintf(buf, cap, "<secret, %u B>", (unsigned int)v->len);
		return;
	}

	switch (v->type) {
	case CFG_T_BOOL:
		(void)snprintf(buf, cap, "%s", (v->v.u != 0U) ? "true" : "false");
		break;
	case CFG_T_U8:
	case CFG_T_U16:
	case CFG_T_U32:
	case CFG_T_U64:
		(void)snprintf(buf, cap, "%llu", (unsigned long long)v->v.u);
		break;
	case CFG_T_I32:
		(void)snprintf(buf, cap, "%d", (int)v->v.i);
		break;
	case CFG_T_F32:
		(void)fmt_f32(buf, cap, v->v.f, 4);
		break;
	case CFG_T_STR:
		(void)snprintf(buf, cap, "\"%.*s\"", (int)v->len,
			       (const char *)v->v.b);
		break;
	case CFG_T_BLOB: {
		size_t o = 0U;
		size_t show = MIN((size_t)v->len, (cap - 4U) / 2U);

		for (size_t i = 0; i < show; i++) {
			o += (size_t)snprintf(&buf[o], cap - o, "%02x",
					      v->v.b[i]);
		}
		if (show < v->len) {
			(void)snprintf(&buf[o], cap - o, "...");
		} else if (v->len == 0U) {
			(void)snprintf(buf, cap, "<empty>");
		}
		break;
	}
	default:
		(void)snprintf(buf, cap, "?");
		break;
	}
}

/* ------------------------------------------------------------------------- */
/* status / quality / alarms                                                 */
/* ------------------------------------------------------------------------- */

static const char *lock_state_name(uint8_t s)
{
	static const char *const names[] = {
		"unknown", "acquiring", "locking", "locked",
		"holdover", "recovering", "parked",
	};

	return (s < ARRAY_SIZE(names)) ? names[s] : "?";
}

static const char *ref_name(uint8_t r)
{
	static const char *const names[] = { "none", "ocxo", "rb", "extref" };

	return (r < ARRAY_SIZE(names)) ? names[r] : "?";
}

static const char *fix_name(uint8_t f)
{
	static const char *const names[] = { "no-fix", "2d", "3d", "time-only" };

	return (f < ARRAY_SIZE(names)) ? names[f] : "?";
}

static const char *group_name(uint8_t g)
{
	static const char *const names[] = { "summary", "timing", "gnss",
					     "power",   "net",    "ptp",
					     "alarms" };

	return (g < ARRAY_SIZE(names)) ? names[g] : "?";
}

static int cmd_quality(const struct shell *sh, size_t argc, char **argv)
{
	quality_block_t q;
	char f1[24];
	char f2[24];
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	rc = sts_quality_snapshot(&q);
	if (rc != 0) {
		shell_error(sh, "quality snapshot failed (%d)", rc);
		return rc;
	}

	shell_print(sh, "stratum      %u", q.stratum);
	shell_print(sh, "lock         %s", lock_state_name(q.lock_state));
	shell_print(sh, "reference    %s", ref_name(q.active_ref));
	shell_print(sh, "gnss         %s, %u/%u SV, tAcc %u ns",
		    fix_name(q.gnss_fix), q.gnss_sv_used, q.gnss_sv_visible,
		    (unsigned int)q.gnss_tacc_ns);
	shell_print(sh, "pps offset   %d ns (mean %s, sigma %s)",
		    (int)q.last_pps_off_ns,
		    fmt_f32(f1, sizeof(f1), q.pps_off_mean_ns, 1),
		    fmt_f32(f2, sizeof(f2), q.pps_off_sigma_ns, 1));
	shell_print(sh, "freq error   %s ppb",
		    fmt_f32(f1, sizeof(f1), q.freq_err_ppb, 3));
	shell_print(sh, "ocxo Vc      cmd %d mV, sense %d mV, dac %u",
		    (int)q.vc_cmd_mv, (int)q.vc_sense_mv, q.dac_code);
	shell_print(sh, "adev         1s %s  10s %s  100s %s",
		    fmt_f32(f1, sizeof(f1), q.adev_1s * 1e12f, 2),
		    fmt_f32(f2, sizeof(f2), q.adev_10s * 1e12f, 2),
		    fmt_f32(f1, sizeof(f1), q.adev_100s * 1e12f, 2));
	shell_print(sh, "             (units 1e-12)");
	shell_print(sh, "holdover     %s, %u s elapsed, est err %lld ns",
		    q.holdover ? "yes" : "no",
		    (unsigned int)q.holdover_elapsed_s,
		    (long long)q.holdover_est_err_ns);
	shell_print(sh, "leap         current %d s, pending %d, utc %s",
		    (int)q.leap_current_s, (int)q.leap_pending,
		    q.utc_valid ? "valid" : "unknown");
	shell_print(sh, "osc temp     %d.%03d C", (int)(q.osc_temp_mc / 1000),
		    (int)(q.osc_temp_mc % 1000 < 0 ? -(q.osc_temp_mc % 1000)
						   : q.osc_temp_mc % 1000));
	shell_print(sh, "flags        0x%08x  tick %u  age %lld ms",
		    (unsigned int)q.flags, (unsigned int)q.tick,
		    (long long)((int64_t)sts_mono_ms() -
				(int64_t)q.updated_mono_ms));
	return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t buf[256];
	uint8_t group;
	uint64_t g;
	int n;

	if (argc < 2U) {
		/* No group: the human summary, which is the quality block plus
		 * whatever alarms are up. */
		(void)cmd_quality(sh, 1U, argv);
		shell_print(sh, "alarms       0x%016llx",
			    (unsigned long long)sts_alarms_active());
		shell_print(sh, "config       %s",
			    sts_cfg_is_persistent() ? "persistent (NVS)"
						    : "RAM defaults only");
		shell_print(sh, "image        %s",
			    sts_update_pending_confirm() ? "UNCONFIRMED"
							 : "confirmed");
		return 0;
	}

	if ((parse_u64(argv[1], &g) != 0) || (g >= (uint64_t)MCP_GRP_COUNT)) {
		shell_error(sh, "group must be 0..%u", MCP_GRP_COUNT - 1U);
		return -EINVAL;
	}
	group = (uint8_t)g;

	n = sts_status_encode(group, buf, sizeof(buf));
	if (n < 0) {
		shell_error(sh, "status group %u (%s) unavailable (%d)", group,
			    group_name(group), n);
		return n;
	}

	shell_print(sh, "group %u (%s): %d bytes, struct version %u", group,
		    group_name(group), n, buf[0]);
	hexdump(sh, buf, (size_t)n);
	return 0;
}

/* Names for the software-raised alarm ids (fault.h >= 32). Ids 0..31 mirror
 * the scanned signals and are reported by number. */
static const char *alarm_name(unsigned int id)
{
	static const char *const names[] = {
		[FAULT_ALARM_REFERENCE_LOST - 32] = "reference-lost",
		[FAULT_ALARM_GNSS_LOST - 32] = "gnss-lost",
		[FAULT_ALARM_ANTENNA_OPEN - 32] = "antenna-open",
		[FAULT_ALARM_ANTENNA_SHORT - 32] = "antenna-short",
		[FAULT_ALARM_OCXO_UNHEALTHY - 32] = "ocxo-unhealthy",
		[FAULT_ALARM_DAC_FAULT - 32] = "dac-fault",
		[FAULT_ALARM_RB_FAULT - 32] = "rb-fault",
		[FAULT_ALARM_RB_OV - 32] = "rb-overvoltage",
		[FAULT_ALARM_THERMAL_WARN - 32] = "thermal-warn",
		[FAULT_ALARM_THERMAL_CRITICAL - 32] = "thermal-critical",
		[FAULT_ALARM_FAN_FAULT - 32] = "fan-fault",
		[FAULT_ALARM_POE_BUDGET - 32] = "poe-budget",
		[FAULT_ALARM_I2C_WEDGE - 32] = "i2c-wedge",
		[FAULT_ALARM_NOR_FAULT - 32] = "nor-fault",
		[FAULT_ALARM_DISPLAY_FAULT - 32] = "display-fault",
		[FAULT_ALARM_PFI - 32] = "power-fail",
		[FAULT_ALARM_TAMPER - 32] = "tamper",
	};

	if ((id < 32U) || ((id - 32U) >= ARRAY_SIZE(names))) {
		return NULL;
	}
	return names[id - 32U];
}

static int cmd_alarms(const struct shell *sh, size_t argc, char **argv)
{
	uint64_t mask = sts_alarms_active();
	unsigned int shown = 0U;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "active mask 0x%016llx", (unsigned long long)mask);
	if (mask == 0U) {
		shell_print(sh, "no active alarms");
		return 0;
	}

	for (unsigned int id = 0U; id < 64U; id++) {
		const char *name;

		if ((mask & (UINT64_C(1) << id)) == 0U) {
			continue;
		}
		name = alarm_name(id);
		if (name != NULL) {
			shell_print(sh, "  %2u  %s", id, name);
		} else {
			shell_print(sh, "  %2u  scanned-signal-%u", id, id);
		}
		shown++;
	}
	shell_print(sh, "%u active", shown);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* log                                                                       */
/* ------------------------------------------------------------------------- */

static int cmd_log_tail(const struct shell *sh, size_t argc, char **argv)
{
	logr_rec_t rec;
	logr_filter_t filter;
	uint32_t want = LOG_TAIL_DEFAULT;
	uint32_t cursor;
	uint32_t head;
	uint32_t gap = 0U;
	uint16_t got = 0U;

	if (argc >= 2U) {
		uint64_t v;

		if ((parse_u64(argv[1], &v) != 0) || (v == 0U) ||
		    (v > LOG_TAIL_MAX)) {
			shell_error(sh, "count must be 1..%d", LOG_TAIL_MAX);
			return -EINVAL;
		}
		want = (uint32_t)v;
	}

	logr_filter_all(&filter);
	head = logr_head(sts_logring());
	cursor = logr_oldest(sts_logring());
	if ((head - cursor) > want) {
		cursor = head - want;
	}

	/* One record per call: the shell prints as it goes and the caller's
	 * stack does not have to hold LOG_TAIL_MAX records. */
	while (cursor != head) {
		uint32_t next = cursor;

		if (logr_tail(sts_logring(), cursor, &filter, &rec, 1U, &got,
			      &next, &gap) != 0) {
			break;
		}
		if (got == 0U) {
			if (next == cursor) {
				break;
			}
			cursor = next;
			continue;
		}
		shell_print(sh, "%8u %10llu.%03u %-7s %-6s %.*s", rec.seq,
			    (unsigned long long)(rec.mono_ms / 1000U),
			    (unsigned int)(rec.mono_ms % 1000U),
			    logr_sub_name(rec.subsys),
			    logr_level_name(rec.level), (int)rec.len, rec.msg);
		cursor = next;
	}

	if (gap != 0U) {
		shell_warn(sh, "%u record(s) were evicted before this tail",
			   gap);
	}
	shell_print(sh, "ring: %u retained, %u dropped, %u filtered",
		    logr_count(sts_logring()), logr_dropped(sts_logring()),
		    logr_filtered(sts_logring()));
	return 0;
}

static int cmd_log_stats(const struct shell *sh, size_t argc, char **argv)
{
	sts_logspool_stats_t s;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	sts_logspool_stats(&s);
	shell_print(sh, "spool        %s", s.spool_open ? "open" : "closed");
	shell_print(sh, "drained      %u records (cursor %u)", s.records,
		    s.cursor);
	shell_print(sh, "spooled      %u records, %u errors", s.spooled,
		    s.spool_errors);
	shell_print(sh, "zephyr LOG   %u forwarded, %u dropped", s.bridge_in,
		    s.bridge_dropped);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* cfg                                                                       */
/* ------------------------------------------------------------------------- */

static int cmd_cfg_list(const struct shell *sh, size_t argc, char **argv)
{
	size_t n = cfg_key_count();
	uint64_t start = 0U;

	if ((argc >= 2U) && (parse_u64(argv[1], &start) != 0)) {
		shell_error(sh, "bad start id");
		return -EINVAL;
	}

	for (size_t i = 0; i < n; i++) {
		const cfg_key_t *k = cfg_key_at(i);
		cfg_val_t v;
		char text[80];

		if ((k == NULL) || (k->id < start)) {
			continue;
		}
		if (cfg_get_effective(sts_cfg(), k->id, &v) != 0) {
			continue;
		}
		format_value(k, &v, text, sizeof(text));
		shell_print(sh, "0x%04x %-20s %-4s %s%s", k->id, k->name,
			    type_name(k->type), text,
			    cfg_is_staged(sts_cfg(), k->id) ? "  (staged)" : "");
	}
	shell_print(sh, "%zu keys, %u staged, store %s", n,
		    cfg_staged_count(sts_cfg()),
		    sts_cfg_is_persistent() ? "persistent" : "RAM-only");
	return 0;
}

static int cmd_cfg_get(const struct shell *sh, size_t argc, char **argv)
{
	const cfg_key_t *k;
	cfg_val_t live;
	cfg_val_t eff;
	char text[80];

	if (argc < 2U) {
		shell_error(sh, "usage: sts cfg get <name|id>");
		return -EINVAL;
	}

	k = resolve_key(argv[1]);
	if (k == NULL) {
		shell_error(sh, "no such key: %s", argv[1]);
		return -ENOENT;
	}
	if ((cfg_get(sts_cfg(), k->id, &live) != 0) ||
	    (cfg_get_effective(sts_cfg(), k->id, &eff) != 0)) {
		shell_error(sh, "read failed");
		return -EIO;
	}

	format_value(k, &live, text, sizeof(text));
	shell_print(sh, "0x%04x %s (%s) = %s", k->id, k->name,
		    type_name(k->type), text);
	if (cfg_is_staged(sts_cfg(), k->id)) {
		format_value(k, &eff, text, sizeof(text));
		shell_print(sh, "  staged -> %s", text);
	}
	shell_print(sh, "  flags:%s%s%s%s",
		    ((k->flags & CFG_F_RUNTIME_APPLY) != 0U) ? " runtime" : "",
		    ((k->flags & CFG_F_REBOOT_REQUIRED) != 0U) ? " reboot" : "",
		    ((k->flags & CFG_F_SECRET) != 0U) ? " secret" : "",
		    ((k->flags & CFG_F_CAL) != 0U) ? " cal" : "");
	return 0;
}

static int cmd_cfg_set(const struct shell *sh, size_t argc, char **argv)
{
	const cfg_key_t *k;
	cfg_val_t v;
	int rc;

	if (argc < 3U) {
		shell_error(sh, "usage: sts cfg set <name|id> <value>");
		return -EINVAL;
	}
	if (!mutating_allowed(sh)) {
		return -EACCES;
	}

	k = resolve_key(argv[1]);
	if (k == NULL) {
		shell_error(sh, "no such key: %s", argv[1]);
		return -ENOENT;
	}

	memset(&v, 0, sizeof(v));
	v.type = k->type;

	switch (k->type) {
	case CFG_T_BOOL: {
		if ((strcmp(argv[2], "true") == 0) ||
		    (strcmp(argv[2], "on") == 0)) {
			v.v.u = 1U;
		} else if ((strcmp(argv[2], "false") == 0) ||
			   (strcmp(argv[2], "off") == 0)) {
			v.v.u = 0U;
		} else if (parse_u64(argv[2], &v.v.u) != 0) {
			shell_error(sh, "expected true/false");
			return -EINVAL;
		}
		break;
	}
	case CFG_T_U8:
	case CFG_T_U16:
	case CFG_T_U32:
	case CFG_T_U64:
		if (parse_u64(argv[2], &v.v.u) != 0) {
			shell_error(sh, "expected an unsigned integer");
			return -EINVAL;
		}
		break;
	case CFG_T_I32: {
		int64_t i;

		if ((parse_i64(argv[2], &i) != 0) || (i < INT32_MIN) ||
		    (i > INT32_MAX)) {
			shell_error(sh, "expected a 32-bit signed integer");
			return -EINVAL;
		}
		v.v.i = (int32_t)i;
		break;
	}
	case CFG_T_F32:
		if (parse_f32(argv[2], &v.v.f) != 0) {
			shell_error(sh, "expected a decimal number");
			return -EINVAL;
		}
		break;
	case CFG_T_STR: {
		size_t n = strlen(argv[2]);

		if (n > CFG_VAL_MAX) {
			shell_error(sh, "string longer than %u bytes",
				    CFG_VAL_MAX);
			return -EINVAL;
		}
		memcpy(v.v.b, argv[2], n);
		v.len = (uint16_t)n;
		break;
	}
	case CFG_T_BLOB: {
		size_t n = 0U;

		if (parse_hex(argv[2], v.v.b, sizeof(v.v.b), &n) != 0) {
			shell_error(sh, "expected an even-length hex string of "
					"at most %u bytes",
				    CFG_VAL_MAX);
			return -EINVAL;
		}
		v.len = (uint16_t)n;
		break;
	}
	default:
		shell_error(sh, "unsupported type");
		return -ENOTSUP;
	}

	rc = cfg_set(sts_cfg(), k->id, &v);
	if (rc != 0) {
		shell_error(sh, "staging %s rejected (%d)%s", k->name, rc,
			    (rc == -ERANGE) ? " - out of range" : "");
		return rc;
	}

	shell_print(sh, "staged %s; %u pending. Run `sts cfg commit`.", k->name,
		    cfg_staged_count(sts_cfg()));
	return 0;
}

static int cmd_cfg_commit(const struct shell *sh, size_t argc, char **argv)
{
	cfg_commit_res_t res;
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mutating_allowed(sh)) {
		return -EACCES;
	}

	memset(&res, 0, sizeof(res));
	rc = sts_cfg_commit(&res);

	shell_print(sh, "staged %u, applied %u, reboot-required %u, persist "
			"errors %u",
		    res.staged, res.applied, res.reboot_keys,
		    res.persist_errors);

	if (rc == -EIO) {
		shell_warn(sh, "applied in RAM but %u key(s) were not persisted",
			   res.persist_errors);
		return 0;
	}
	if (rc != 0) {
		shell_error(sh, "commit rejected (%d); nothing applied", rc);
		return rc;
	}
	if (res.reboot_keys != 0U) {
		shell_warn(sh, "%u applied key(s) need a reboot to take effect",
			   res.reboot_keys);
	}
	return 0;
}

static int cmd_cfg_revert(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t n;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mutating_allowed(sh)) {
		return -EACCES;
	}

	n = cfg_staged_count(sts_cfg());
	(void)cfg_revert(sts_cfg());
	shell_print(sh, "dropped %u staged value(s)", n);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* sec                                                                       */
/* ------------------------------------------------------------------------- */

static int cmd_sec_passwd(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t blob[MCP_PW_BLOB_LEN];
	cfg_commit_res_t res;
	size_t pw_len;
	int rc;

	if (argc < 2U) {
		shell_error(sh, "usage: sts sec passwd <password>");
		return -EINVAL;
	}
	if (!mutating_allowed(sh)) {
		return -EACCES;
	}

	pw_len = strlen(argv[1]);
	if ((pw_len == 0U) || (pw_len > MCP_PW_MAX)) {
		shell_error(sh, "password must be 1..%u bytes", MCP_PW_MAX);
		return -EINVAL;
	}

	rc = mcp_auth_make_blob(sts_port_crypto(), NULL,
				(const uint8_t *)argv[1], pw_len, blob,
				sizeof(blob));
	if (rc != 0) {
		shell_error(sh, "credential derivation failed (%d)", rc);
		return rc;
	}

	rc = cfg_set_bytes(sts_cfg(), (uint16_t)CFG_ID_SEC_ADMIN_PW, blob,
			   sizeof(blob));
	if (rc != 0) {
		shell_error(sh, "staging the credential failed (%d)", rc);
		return rc;
	}

	memset(&res, 0, sizeof(res));
	rc = sts_cfg_commit(&res);
	if ((rc != 0) && (rc != -EIO)) {
		shell_error(sh, "commit failed (%d)", rc);
		return rc;
	}
	if (res.persist_errors != 0U) {
		shell_warn(sh, "credential set in RAM only; it will not survive "
				"a reboot");
	}

	shell_print(sh, "admin credential set. MCP AUTH now works; "
			"sec.console.ro is enforced from here on.");
	sts_log((uint8_t)LOGR_SUB_SEC, (uint8_t)LOGR_NOTICE,
		"admin credential provisioned from the local console");
	return 0;
}

/* ------------------------------------------------------------------------- */
/* fw                                                                        */
/* ------------------------------------------------------------------------- */

static void print_slot(const struct shell *sh, uint8_t slot)
{
	port_image_info_t info;

	if (sts_dfu_image_info(slot, &info) != 0) {
		shell_print(sh, "slot%u: unreadable", slot);
		return;
	}
	shell_print(sh, "slot%u: %s v%u.%u.%u+%u, %u B%s%s%s", slot,
		    info.valid ? "valid" : "no image", info.version[0],
		    info.version[1], info.version[2], info.version[3],
		    (unsigned int)info.size, info.active ? " [active]" : "",
		    info.pending ? " [pending]" : "",
		    info.confirmed ? " [confirmed]" : "");
}

static int cmd_fw_info(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	print_slot(sh, 0U);
	print_slot(sh, 1U);
	shell_print(sh, "next boot swap: %s", sts_dfu_swap_type_name());
	shell_print(sh, "staging usable: %u B, %u B erase pages",
		    (unsigned int)sts_dfu_port()->staging_size(NULL),
		    (unsigned int)sts_dfu_erase_granularity());

	if (sts_update_pending_confirm()) {
		sts_selfconfirm_status_t st;

		sts_selfconfirm_status(&st);
		shell_warn(sh, "running image is UNCONFIRMED");
		shell_print(sh, "self-confirm gate at %u s: age%s cfg%s nvs%s "
				"link%s%s",
			    st.uptime_s, st.min_age_met ? "+" : "-",
			    st.cfg_loaded ? "+" : "-",
			    st.store_ready ? "+" : "-", st.link_ok ? "+" : "-",
			    st.deadline_passed ? " [ABANDONED]" : "");
	}
	return 0;
}

static int cmd_fw_confirm(const struct shell *sh, size_t argc, char **argv)
{
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mutating_allowed(sh)) {
		return -EACCES;
	}
	if (boot_is_img_confirmed()) {
		shell_print(sh, "already confirmed");
		return 0;
	}

	rc = boot_write_img_confirmed();
	if (rc != 0) {
		shell_error(sh, "confirm failed (%d)", rc);
		return rc;
	}
	shell_print(sh, "running image confirmed");
	return 0;
}

static int cmd_fw_revert(const struct shell *sh, size_t argc, char **argv)
{
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mutating_allowed(sh)) {
		return -EACCES;
	}

	rc = sts_dfu_port()->request_revert(NULL);
	if (rc != 0) {
		shell_error(sh, "revert failed (%d)", rc);
		return rc;
	}
	if (sts_update_pending_confirm()) {
		shell_print(sh, "the running image is unconfirmed: the next "
				"reboot returns to the previous one");
	} else {
		shell_print(sh, "any staged image is no longer pending");
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/* reboot                                                                    */
/* ------------------------------------------------------------------------- */

static int cmd_reboot(const struct shell *sh, size_t argc, char **argv)
{
	bool recovery = (argc >= 2U) && (strcmp(argv[1], "recovery") == 0);

	if ((argc >= 2U) && !recovery) {
		shell_error(sh, "usage: sts reboot [recovery]");
		return -EINVAL;
	}
	if (!mutating_allowed(sh)) {
		return -EACCES;
	}

	if (recovery) {
		/*
		 * Refuse rather than reboot into the application and leave the
		 * operator wondering why recovery did not happen. MCUboot on
		 * this board arms serial recovery from a held GPIO only
		 * (sysbuild/mcuboot.conf).
		 */
		shell_error(sh, "recovery cannot be armed from firmware on this "
				"build.");
		shell_print(sh, "To enter MCUboot serial recovery:");
		shell_print(sh, "  1. hold front-panel BUTTON_1 (PF0)");
		shell_print(sh, "  2. reset or power-cycle the board while "
				"holding it");
		shell_print(sh, "  3. keep holding for at least 1 s after reset");
		shell_print(sh, "  4. the board enumerates as \"STS1000 "
				"Meridian recovery\" (PID 0x1001);");
		shell_print(sh, "     upload with: mcumgr -c <conn> image "
				"upload zephyr.signed.bin");
		return -ENOTSUP;
	}

	shell_print(sh, "rebooting...");
	sts_dfu_port()->reboot(NULL, 0);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* diag                                                                      */
/* ------------------------------------------------------------------------- */

static int cmd_diag_i2c(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t map[16];
	int n;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	n = sts_diag_i2c_scan(map);
	if (n < 0) {
		shell_error(sh, "I2C scan unavailable (%d)", n);
		return n;
	}

	shell_print(sh, "I2C1 scan: %d device(s)", n);
	for (unsigned int addr = 0U; addr < 128U; addr++) {
		if ((map[addr / 8U] & (1U << (addr % 8U))) != 0U) {
			shell_print(sh, "  0x%02x", addr);
		}
	}
	return 0;
}

static void shell_thread_cb(const struct k_thread *thread, void *user)
{
	const struct shell *sh = user;
	const char *name = k_thread_name_get((k_tid_t)thread);
	size_t unused = 0U;
	size_t size = 0U;

#if defined(CONFIG_THREAD_STACK_INFO)
	size = thread->stack_info.size;
#endif
	if (k_thread_stack_space_get(thread, &unused) != 0) {
		unused = 0U;
	}

	shell_print(sh, "  %-16s prio %3d  stack %5zu B, %5zu unused (%zu%% "
			"used)",
		    (name != NULL) ? name : "?",
		    k_thread_priority_get((k_tid_t)thread), size, unused,
		    (size > 0U) ? (((size - unused) * 100U) / size) : 0U);
}

static int cmd_diag_threads(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if defined(CONFIG_THREAD_MONITOR)
	shell_print(sh, "threads:");
	k_thread_foreach_unlocked(shell_thread_cb, (void *)sh);
	return 0;
#else
	shell_error(sh, "CONFIG_THREAD_MONITOR is not enabled");
	return -ENOTSUP;
#endif
}

static int cmd_diag_store(const struct shell *sh, size_t argc, char **argv)
{
	sts_critical_t crit;
	sts_logspool_stats_t ls;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "NVS          %s", sts_store_ready() ? "mounted"
							     : "UNAVAILABLE");
	shell_print(sh, "cfg store    %s", sts_cfg_is_persistent()
						   ? "persistent"
						   : "RAM defaults only");
	shell_print(sh, "/lfs         %s", sts_fs_ready() ? "mounted"
							  : "not mounted");

	sts_logspool_stats(&ls);
	shell_print(sh, "log spool    %s", ls.spool_open ? "open" : "closed");

	if (sts_store_critical_load(&crit) == 0) {
		shell_print(sh, "last fast-save: reason %u at %llu ms uptime",
			    crit.reason, (unsigned long long)crit.mono_ms);
		shell_print(sh, "  dac %u, leap %d (pending %d, %s), log cursor "
				"%u",
			    crit.dac_code, (int)crit.leap_current,
			    (int)crit.leap_pending,
			    crit.leap_valid ? "valid" : "unknown",
			    crit.log_cursor);
	} else {
		shell_print(sh, "last fast-save: none");
	}
	return 0;
}

static int cmd_diag_mcp(const struct shell *sh, size_t argc, char **argv)
{
	const mcp_stats_t *s = sts_mcp_stats();
	sts_mcp_link_stats_t l;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (s == NULL) {
		shell_error(sh, "the MCP channel is not running");
		return -ENODEV;
	}

	sts_mcp_link_stats(&l);
	shell_print(sh, "link         DTR %s, %u drop(s)", l.dtr ? "up" : "down",
		    l.link_drops);
	shell_print(sh, "bytes        rx %u, tx %u, rx overrun %u", l.rx_bytes,
		    l.tx_bytes, l.rx_overruns);
	shell_print(sh, "frames       rx %u (cobs %u, crc %u, hdr %u, ver %u, "
			"ignored %u)",
		    s->rx_frames, s->rx_bad_cobs, s->rx_bad_crc, s->rx_bad_hdr,
		    s->rx_bad_ver, s->rx_ignored);
	shell_print(sh, "tx           rsp %u, evt %u, evt dropped %u, stalls %u "
			"(glue %u)",
		    s->rsp_tx, s->evt_tx, s->evt_dropped, s->tx_stalls,
		    l.tx_backpressure);
	shell_print(sh, "auth         ok %u, fail %u, denied %u, expired %u",
		    s->auth_ok, s->auth_fail, s->auth_denied,
		    s->session_expired);
	shell_print(sh, "usb          VBUS %s, %s",
		    sts_usb_vbus_present() ? "present" : "absent",
		    sts_usb_configured() ? "configured" : "not configured");
	return 0;
}

/* ------------------------------------------------------------------------- */
/* Registration                                                              */
/* ------------------------------------------------------------------------- */

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_sts_log,
	SHELL_CMD_ARG(tail, NULL, "tail [n] - print the last n log records",
		      cmd_log_tail, 1, 1),
	SHELL_CMD_ARG(stats, NULL, "log ring and NOR spool counters",
		      cmd_log_stats, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_sts_cfg,
	SHELL_CMD_ARG(list, NULL, "list [start-id] - every key and its value",
		      cmd_cfg_list, 1, 1),
	SHELL_CMD_ARG(get, NULL, "get <name|id>", cmd_cfg_get, 2, 0),
	SHELL_CMD_ARG(set, NULL, "set <name|id> <value> - stages only",
		      cmd_cfg_set, 3, 0),
	SHELL_CMD_ARG(commit, NULL, "validate and apply the staged set",
		      cmd_cfg_commit, 1, 0),
	SHELL_CMD_ARG(revert, NULL, "drop the staged set", cmd_cfg_revert, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_sts_sec,
	SHELL_CMD_ARG(passwd, NULL, "passwd <password> - set the admin credential",
		      cmd_sec_passwd, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_sts_fw,
	SHELL_CMD_ARG(info, NULL, "slot table and swap state", cmd_fw_info, 1, 0),
	SHELL_CMD_ARG(confirm, NULL, "confirm the running image", cmd_fw_confirm,
		      1, 0),
	SHELL_CMD_ARG(revert, NULL, "cancel a pending swap / revert on reboot",
		      cmd_fw_revert, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_sts_diag,
	SHELL_CMD_ARG(i2c, NULL, "scan the I2C1 housekeeping bus", cmd_diag_i2c,
		      1, 0),
	SHELL_CMD_ARG(threads, NULL, "thread priorities and stack usage",
		      cmd_diag_threads, 1, 0),
	SHELL_CMD_ARG(store, NULL, "NVS, /lfs and fast-save state",
		      cmd_diag_store, 1, 0),
	SHELL_CMD_ARG(mcp, NULL, "MCP channel counters", cmd_diag_mcp, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_sts,
	SHELL_CMD_ARG(status, NULL, "status [group] - summary, or one encoded "
				    "STATUS_GET group",
		      cmd_status, 1, 1),
	SHELL_CMD_ARG(quality, NULL, "the §3.8 clock-quality block", cmd_quality,
		      1, 0),
	SHELL_CMD_ARG(alarms, NULL, "active alarm mask", cmd_alarms, 1, 0),
	SHELL_CMD(log, &sub_sts_log, "structured log", NULL),
	SHELL_CMD(cfg, &sub_sts_cfg, "configuration registry", NULL),
	SHELL_CMD(sec, &sub_sts_sec, "security provisioning", NULL),
	SHELL_CMD(fw, &sub_sts_fw, "firmware slots and DFU state", NULL),
	SHELL_CMD_ARG(reboot, NULL, "reboot [recovery]", cmd_reboot, 1, 1),
	SHELL_CMD(diag, &sub_sts_diag, "diagnostics", NULL),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(sts, &sub_sts, "STS1000 Meridian control", NULL);

void sts_shell_announce(void)
{
	if (!SHELL_MUTATING) {
		return;
	}
	if (!credential_provisioned()) {
		/* Not a shell_print: no shell instance exists yet at start. */
		sts_log((uint8_t)LOGR_SUB_SEC, (uint8_t)LOGR_WARN,
			"no admin credential set; console mutations are open "
			"until `sts sec passwd` runs");
	}
}

#endif /* CONFIG_STS1000_CONSOLE */
