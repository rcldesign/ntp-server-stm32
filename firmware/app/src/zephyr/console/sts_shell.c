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

#include "auth/auth.h"
#include "cfg/cfg.h"
#include "console/sts_console.h"
#include "console/sts_recovery_policy.h"
#include "console/sts_rollback.h"
#include "disc/disc.h"
#include "fault/fault.h"
#include "logring/logring.h"
#include "mcp/mcp.h"
#include "quality/quality.h"
#include "storage/sts_atecc.h"
#include "storage/sts_store.h"
#include "zephyr/sts_app.h"

/*
 * Net-area entry points, reached the way mp_glue.c reaches sts_aaa_check_fed():
 * a weak extern rather than an include. src/zephyr/net/sts_aaa.h and sts_net.h
 * are PRIVATE to that area (ARCHITECTURE.md §2), and with CONFIG_STS1000_NET=n
 * there is no net area at all — the symbols then resolve to NULL and the
 * commands say so, instead of the image failing to link.
 */
extern int sts_aaa_unlock(const char *user) __attribute__((weak));
extern void sts_aaa_flush(void) __attribute__((weak));
extern int sts_aaa_stats(auth_stats_t *out) __attribute__((weak));
extern int sts_nts_rotate_now(void) __attribute__((weak));

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

/*
 * sts_cfg() is mutex-guarded (sts_app.h) — the MCP engine and ui_local write the
 * same tree from other threads. Every read below takes the section briefly, and
 * never with shell output inside it: the console is far slower than the lock is
 * allowed to be held.
 */
static bool credential_provisioned(void)
{
	uint8_t blob[MCP_PW_BLOB_LEN];
	size_t len = 0U;
	int rc;

	sts_cfg_lock();
	rc = cfg_get_bytes(sts_cfg(), (uint16_t)CFG_ID_SEC_ADMIN_PW, blob,
			   sizeof(blob), &len);
	sts_cfg_unlock();

	if (rc != 0) {
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

	sts_cfg_lock();
	if (cfg_get_bool(sts_cfg(), (uint16_t)CFG_ID_SEC_CONSOLE_RO, &ro) != 0) {
		ro = true; /* fail closed */
	}
	sts_cfg_unlock();
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
		[FAULT_ALARM_SYSLOG_DOWN - 32] = "syslog-down",
	};

	if ((id < 32U) || ((id - 32U) >= ARRAY_SIZE(names))) {
		return NULL;
	}
	return names[id - 32U];
}

/** Resolve `all`, a decimal/hex id, or a name from alarm_name(). */
static int alarm_resolve(const char *token, bool *all, uint64_t *id)
{
	*all = (strcmp(token, "all") == 0);
	if (*all) {
		*id = 0U;
		return 0;
	}
	if (parse_u64(token, id) == 0) {
		return 0;
	}
	for (unsigned int i = STS_ALARM_FIRST_SOFTWARE; i < STS_ALARM_ID_COUNT;
	     i++) {
		const char *n = alarm_name(i);

		if ((n != NULL) && (strcmp(n, token) == 0)) {
			*id = i;
			return 0;
		}
	}
	return -ENOENT;
}

/**
 * `sts alarms clear <id|name|all>` — acknowledge latches whose cause has gone.
 *
 * A latch that is still active is refused by core/fault and reported as a
 * failure; a latch that was never set is reported and is NOT a failure, because
 * `clear` is idempotent and the operator's intent is already satisfied. The two
 * readings live in sts_recovery_policy.h so they are asserted rather than
 * re-derived here.
 */
static int cmd_alarms_clear(const struct shell *sh, const char *token)
{
	sts_aclr_outcome_t out;
	uint64_t id = 0U;
	bool all = false;

	if (!mutating_allowed(sh)) {
		return -EACCES;
	}
	if (alarm_resolve(token, &all, &id) != 0) {
		shell_error(sh, "unknown alarm: %s", token);
		return -ENOENT;
	}

	switch (sts_aclr_target(all, id)) {
	case STS_ACLR_TARGET_ALL: {
		uint64_t before = sts_alarms_latched();
		uint64_t after;

		(void)sts_alarm_clear_all();
		after = sts_alarms_latched();
		shell_print(sh, "cleared %u latch(es); 0x%016llx still latched "
				"(condition still active)",
			    (unsigned int)__builtin_popcountll(before & ~after),
			    (unsigned long long)after);
		return 0;
	}
	case STS_ACLR_TARGET_ONE:
		out = sts_aclr_outcome(sts_alarm_clear((uint8_t)id));
		break;
	case STS_ACLR_TARGET_BAD:
	default:
		shell_error(sh, "alarm id must be 0..%u",
			    STS_ALARM_ID_COUNT - 1U);
		return -EINVAL;
	}

	if (sts_aclr_is_failure(out)) {
		shell_error(sh, "alarm %u: %s", (unsigned int)id,
			    sts_aclr_outcome_name(out));
		return (out == STS_ACLR_STILL_ACTIVE) ? -EBUSY : -EINVAL;
	}
	shell_print(sh, "alarm %u: %s", (unsigned int)id,
		    sts_aclr_outcome_name(out));
	return 0;
}

static int cmd_alarms(const struct shell *sh, size_t argc, char **argv)
{
	uint64_t mask;
	uint64_t latched;
	unsigned int shown = 0U;

	if (argc >= 2U) {
		if (strcmp(argv[1], "clear") != 0) {
			shell_error(sh, "usage: sts alarms [clear "
					"<id|name|all>]");
			return -EINVAL;
		}
		if (argc < 3U) {
			shell_error(sh, "usage: sts alarms clear "
					"<id|name|all>");
			return -EINVAL;
		}
		return cmd_alarms_clear(sh, argv[2]);
	}

	/* One pair of reads, so the two masks describe the same instant as
	 * closely as two mutex sections can. */
	mask = sts_alarms_active();
	latched = sts_alarms_latched();

	shell_print(sh, "active  mask 0x%016llx", (unsigned long long)mask);
	shell_print(sh, "latched mask 0x%016llx", (unsigned long long)latched);
	if ((mask | latched) == 0U) {
		shell_print(sh, "no active or latched alarms");
		return 0;
	}

	for (unsigned int id = 0U; id < 64U; id++) {
		uint64_t bit = UINT64_C(1) << id;
		const char *name;

		if (((mask | latched) & bit) == 0U) {
			continue;
		}
		name = alarm_name(id);
		shell_print(sh, "  %2u  %-18s %s%s", id,
			    (name != NULL) ? name : "scanned-signal",
			    ((mask & bit) != 0U) ? "ACTIVE" : "-",
			    ((latched & bit) != 0U) ? " latched" : "");
		shown++;
	}
	shell_print(sh, "%u alarm(s); `sts alarms clear <id|name|all>` "
			"acknowledges those whose cause has gone",
		    shown);
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
	uint16_t total_staged;

	if ((argc >= 2U) && (parse_u64(argv[1], &start) != 0)) {
		shell_error(sh, "bad start id");
		return -EINVAL;
	}

	for (size_t i = 0; i < n; i++) {
		const cfg_key_t *k = cfg_key_at(i);
		cfg_val_t v;
		char text[80];
		bool staged;
		int rc;

		if ((k == NULL) || (k->id < start)) {
			continue;
		}

		/* Per key, not around the loop: the output is the slow part. */
		sts_cfg_lock();
		rc = cfg_get_effective(sts_cfg(), k->id, &v);
		staged = cfg_is_staged(sts_cfg(), k->id);
		sts_cfg_unlock();

		if (rc != 0) {
			continue;
		}
		format_value(k, &v, text, sizeof(text));
		shell_print(sh, "0x%04x %-20s %-4s %s%s", k->id, k->name,
			    type_name(k->type), text,
			    staged ? "  (staged)" : "");
	}

	sts_cfg_lock();
	total_staged = cfg_staged_count(sts_cfg());
	sts_cfg_unlock();

	shell_print(sh, "%zu keys, %u staged, store %s", n, total_staged,
		    sts_cfg_is_persistent() ? "persistent" : "RAM-only");
	return 0;
}

static int cmd_cfg_get(const struct shell *sh, size_t argc, char **argv)
{
	const cfg_key_t *k;
	cfg_val_t live;
	cfg_val_t eff;
	char text[80];
	bool staged;
	int rc;

	if (argc < 2U) {
		shell_error(sh, "usage: sts cfg get <name|id>");
		return -EINVAL;
	}

	k = resolve_key(argv[1]);
	if (k == NULL) {
		shell_error(sh, "no such key: %s", argv[1]);
		return -ENOENT;
	}
	/* One section for both reads, so live and effective describe the same
	 * instant even if another thread commits between commands. */
	sts_cfg_lock();
	rc = cfg_get(sts_cfg(), k->id, &live);
	if (rc == 0) {
		rc = cfg_get_effective(sts_cfg(), k->id, &eff);
	}
	staged = cfg_is_staged(sts_cfg(), k->id);
	sts_cfg_unlock();

	if (rc != 0) {
		shell_error(sh, "read failed");
		return -EIO;
	}

	format_value(k, &live, text, sizeof(text));
	shell_print(sh, "0x%04x %s (%s) = %s", k->id, k->name,
		    type_name(k->type), text);
	if (staged) {
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
	uint16_t staged;
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

	/* sts_cfg() is mutex-guarded (sts_app.h): the MCP engine and ui_local
	 * write the same tree. Stage and read the pending count in one section so
	 * the number reported is the one this command produced. */
	sts_cfg_lock();
	rc = cfg_set(sts_cfg(), k->id, &v);
	staged = cfg_staged_count(sts_cfg());
	sts_cfg_unlock();

	if (rc != 0) {
		shell_error(sh, "staging %s rejected (%d)%s", k->name, rc,
			    (rc == -ERANGE) ? " - out of range" : "");
		return rc;
	}

	shell_print(sh, "staged %s; %u pending. Run `sts cfg commit`.", k->name,
		    staged);
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

	sts_cfg_lock();
	n = cfg_staged_count(sts_cfg());
	(void)cfg_revert(sts_cfg());
	sts_cfg_unlock();

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

	/* sts_cfg() is mutex-guarded (sts_app.h). Released before the commit
	 * below, which takes the mutex itself and then dispatches config appliers
	 * with it dropped — holding it across that would run those appliers, some
	 * of which touch sockets, inside the config critical section. */
	sts_cfg_lock();
	rc = cfg_set_bytes(sts_cfg(), (uint16_t)CFG_ID_SEC_ADMIN_PW, blob,
			   sizeof(blob));
	sts_cfg_unlock();

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

/*
 * `sts sec attest` — the spec §9.1/§9.6 attestation report, plus the
 * anti-rollback state that gives the secure element's monotonic counters their
 * meaning. Read-only, so it is deliberately outside the mutating gate: an
 * operator diagnosing a refused update needs it before they have a credential.
 */
static int cmd_sec_attest(const struct shell *sh, size_t argc, char **argv)
{
	sts_rollback_status_t rb;
	atecc_attest_t at;
	char sn[19];
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	sts_rollback_status(&rb);

	shell_print(sh, "security epoch  %u (compiled in)",
		    (unsigned int)rb.epoch);
	if (rb.signed_epoch_valid) {
		shell_print(sh, "  signed image  %u  %s",
			    (unsigned int)rb.signed_epoch,
			    rb.corroborated ? "(matches)" : "*** MISMATCH ***");
	} else {
		shell_warn(sh, "  signed image  NO IMAGE_TLV_SEC_CNT in slot 0 "
				"- MCUboot is enforcing no downgrade gate");
	}
	if (rb.witness_enabled) {
		if (rb.witness_valid) {
			shell_print(sh, "  witness       ATECC counter %u = %u"
					"%s%s",
				    (unsigned int)rb.witness_index,
				    (unsigned int)rb.witness_counter,
				    (rb.witness_steps > 0U) ? ", stepped this "
							      "boot" : "",
				    rb.witnessed ? "" : " (not yet recorded)");
		} else {
			shell_print(sh, "  witness       unavailable (no secure "
					"element)");
		}
	} else {
		shell_print(sh, "  witness       disabled by configuration");
	}

	/* --- the secure element itself ------------------------------------ */
	rc = sts_atecc_attest(&at);
	if (rc != 0) {
		/* -ENODEV is "use the software path", not a failure (sts_atecc.h). */
		shell_warn(sh, "ATECC608B: %s",
			   (rc == -ENODEV) ? "absent, unprovisioned or disabled "
					     "by sec.atecc.en"
					   : "attestation read failed");
		return (rc == -ENODEV) ? 0 : rc;
	}

	if (sts_atecc_serial_string(sn, sizeof(sn)) > 0) {
		shell_print(sh, "serial          %s", sn);
	}
	shell_print(sh, "revision        %02x %02x %02x %02x", at.revision[0],
		    at.revision[1], at.revision[2], at.revision[3]);
	shell_print(sh, "zones           config %s, data %s%s",
		    at.config_locked ? "LOCKED" : "unlocked",
		    at.data_locked ? "LOCKED" : "unlocked",
		    at.lock_valid ? "" : " (lock bytes unreadable)");
	for (unsigned int i = 0U; i < ATECC_COUNTER_COUNT; i++) {
		if (at.counter_valid[i]) {
			shell_print(sh, "counter %u       %u%s", i,
				    (unsigned int)at.counter[i],
				    (rb.witness_enabled &&
				     (i == rb.witness_index))
					    ? "  <- anti-rollback witness"
					    : "");
		} else {
			shell_print(sh, "counter %u       unreadable", i);
		}
	}
	if (at.selftest_valid) {
		shell_print(sh, "selftest        %s (0x%02x)",
			    (at.selftest_result == 0U) ? "pass" : "FAIL",
			    at.selftest_result);
	}
	shell_print(sh, "device key      %s",
		    at.pubkey_valid ? "readable" : "unreadable");
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
		shell_print(sh, "self-confirm gate at %u/%u s: age%s cfg%s nvs%s "
				"link%s lock%s strat1%s%s",
			    st.uptime_s, st.deadline_s,
			    st.min_age_met ? "+" : "-",
			    st.cfg_loaded ? "+" : "-",
			    st.store_ready ? "+" : "-", st.link_ok ? "+" : "-",
			    st.clock_locked ? "+" : "-",
			    st.serving_primary ? "+" : "-",
			    st.deadline_passed ? " [ABANDONED]" : "");
		shell_print(sh, "  the supervisor confirms when the gate closes; "
				"`sts fw confirm` overrides it by hand");
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

	/*
	 * The second — and only other — path that accepts an image, so it owes
	 * the same one-way anti-rollback witness the §8.3 supervisor gate does
	 * (sts_selfconfirm.c). Not fatal: the image is confirmed either way, and
	 * a board with no secure element answers -ENODEV.
	 */
	if (sts_rollback_witness() != 0) {
		shell_warn(sh, "anti-rollback: the running image's security "
				"epoch could not be corroborated; see the log");
	}
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

/*
 * The §6.3 polar skyplot, as characters.
 *
 * The plot is the one panel element the text mirror cannot carry — it is
 * pixels, and everything else the panel shows is glyphs — so a unit being
 * worked on over the USB console with no display attached has no other way to
 * see which part of its sky is obstructed. The renderer's ASCII accessor exists
 * for exactly this (core/ui/skyplot.h), and the UI area renders it into its own
 * small canvas rather than the panel's, so this runs without racing ui_local.
 *
 * WEAK, and reached without including the UI area's private header: sts_ui.h is
 * private to src/zephyr/ui/ (ARCHITECTURE.md §2), and with CONFIG_STS1000_UI=n
 * there is no UI area at all. Same idiom as sts_web.c's sts_dfu_port() and
 * hk.c's sts_atecc_reapply() — the symbol resolves to NULL and the command says
 * so, rather than the image failing to link.
 */
extern size_t ui_display_sky_ascii(char *out, size_t cap) __attribute__((weak));

static int cmd_diag_sky(const struct shell *sh, size_t argc, char **argv)
{
	/* 31 rows of 31 characters plus a newline each, plus the NUL. Static
	 * because the shell thread's stack is not the place for a kilobyte, and
	 * safe because the Zephyr shell dispatches one command at a time. */
	static char pic[(31 * 32) + 1];
	const char *p = pic;
	size_t n;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (ui_display_sky_ascii == NULL) {
		shell_error(sh, "no local-UI area in this image");
		return -ENOSYS;
	}

	n = ui_display_sky_ascii(pic, sizeof(pic));
	if (n == 0u) {
		shell_error(sh, "no skyplot has been rendered");
		return -ENODATA;
	}

	shell_print(sh, "zenith centre, horizon rim; '=' horizon/north tick, "
			"'-' grid, '#' north-unverified badge,");
	shell_print(sh, "'0'-'6' GPS/GAL/GLO/BDS/SBAS/QZSS/other, filled = used "
			"in the solution");

	/* One shell_print() per row: the shell's own line buffer is far smaller
	 * than the picture, and a single print would be truncated. */
	while (*p != '\0') {
		const char *nl = strchr(p, '\n');

		if (nl == NULL) {
			shell_print(sh, "%s", p);
			break;
		}
		shell_print(sh, "%.*s", (int)(nl - p), p);
		p = nl + 1;
	}
	return 0;
}

static int cmd_diag_identify(const struct shell *sh, size_t argc, char **argv)
{
	uint64_t ms = 30000U;

	if (!mutating_allowed(sh)) {
		return -EACCES;
	}
	if (argc >= 2U) {
		if ((parse_u64(argv[1], &ms) != 0) || (ms > 600000U)) {
			shell_error(sh, "duration must be 0..600000 ms");
			return -EINVAL;
		}
	}

	sts_supervisor_identify((uint32_t)ms);
	if (ms == 0U) {
		shell_print(sh, "identify beacon off");
	} else {
		shell_print(sh, "status RGB pulsing blue for %u ms "
				"(0 stops it)",
			    (unsigned int)ms);
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/* gnss — spec §7.1: survey-in, fixed position, antenna status               */
/* ------------------------------------------------------------------------- */

static const char *ant_state_name(uint8_t s)
{
	static const char *const names[] = { "unknown", "off (commanded)", "ok",
					     "OPEN", "SHORT" };

	return (s < ARRAY_SIZE(names)) ? names[s] : "?";
}

static int cmd_gnss_show(const struct shell *sh, size_t argc, char **argv)
{
	sts_gnss_detail_t d;
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	rc = sts_gnss_detail(&d);
	if (rc != 0) {
		shell_error(sh, "the gnss thread is not running (%d)", rc);
		return rc;
	}

	shell_print(sh, "antenna      %s%s", ant_state_name(d.ant_state),
		    d.ant_short_latched ? "  [BIAS CUT - latched short]" : "");
	if (d.ant_short_latched) {
		shell_warn(sh, "  there is no automatic recovery: run "
				"`sts gnss reenable` once the fault is fixed");
	}
	shell_print(sh, "mgr alarms   0x%08x", (unsigned int)d.alarms);

	if (d.rf_valid) {
		shell_print(sh, "MON-RF       antPower %u, jamming %u%s%s",
			    d.rf_ant_power, d.rf_jamming,
			    d.rf_ant_short ? ", SHORT" : "",
			    d.rf_ant_open ? ", OPEN" : "");
	} else {
		shell_print(sh, "MON-RF       no frame decoded yet");
	}

	if (d.svin_seen) {
		shell_print(sh, "survey-in    %s, %u s, %u obs, mean acc %u.%u mm%s",
			    d.svin_active ? "RUNNING" : "idle",
			    (unsigned int)d.svin_dur_s,
			    (unsigned int)d.svin_obs,
			    (unsigned int)(d.svin_acc_0p1mm / 10U),
			    (unsigned int)(d.svin_acc_0p1mm % 10U),
			    d.svin_ok ? ", limits met" : "");
	} else {
		shell_print(sh, "survey-in    no NAV-SVIN decoded");
	}

	if (d.pos_valid) {
		shell_print(sh, "position     ECEF %d, %d, %d cm (acc %u.%u mm)",
			    (int)d.pos_x_cm, (int)d.pos_y_cm, (int)d.pos_z_cm,
			    (unsigned int)(d.pos_acc_0p1mm / 10U),
			    (unsigned int)(d.pos_acc_0p1mm % 10U));
	} else {
		shell_print(sh, "position     none stored (surveying or "
				"unconfigured)");
	}
	return 0;
}

static int cmd_gnss_survey(const struct shell *sh, size_t argc, char **argv)
{
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mutating_allowed(sh)) {
		return -EACCES;
	}

	rc = sts_gnss_request_survey(true);
	if (rc != 0) {
		shell_error(sh, "survey-in refused (%d)%s", rc,
			    (rc == -EBUSY) ? " - a request is already pending"
					   : "");
		return rc;
	}
	shell_print(sh, "survey-in requested; the stored position is discarded "
			"when it starts");
	shell_print(sh, "watch `sts gnss show` — this can take an hour");
	return 0;
}

static int cmd_gnss_reenable(const struct shell *sh, size_t argc, char **argv)
{
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mutating_allowed(sh)) {
		return -EACCES;
	}

	rc = sts_gnss_ant_reenable();
	if (rc != 0) {
		shell_error(sh, "antenna re-enable refused (%d)%s", rc,
			    (rc == -EBUSY) ? " - a request is already pending"
					   : "");
		return rc;
	}
	shell_print(sh, "antenna bias restore queued; if the short is still "
			"there the supervisor re-latches it");
	return 0;
}

/* ------------------------------------------------------------------------- */
/* clock — reference selection, CSS, the flap latch                          */
/* ------------------------------------------------------------------------- */

static int cmd_clock_show(const struct shell *sh, size_t argc, char **argv)
{
	static const char *const req[] = { "auto", "force-ocxo", "extref" };
	quality_block_t q;
	uint8_t r = sts_ref_override_get();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "requested    %s",
		    (r < ARRAY_SIZE(req)) ? req[r] : "?");
	if (sts_quality_snapshot(&q) == 0) {
		shell_print(sh, "active       %s (lock %s)",
			    ref_name(q.active_ref),
			    lock_state_name(q.lock_state));
	}
	shell_print(sh, "CSS events   %u  (HSE stopped and the clock security "
			"system fired)",
		    (unsigned int)sts_clock_css_events());
	return 0;
}

static int cmd_clock_flapclear(const struct shell *sh, size_t argc, char **argv)
{
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mutating_allowed(sh)) {
		return -EACCES;
	}

	rc = sts_ref_clear_flap();
	if (rc != 0) {
		shell_error(sh, "the discipline thread is not running (%d)", rc);
		return rc;
	}
	shell_print(sh, "reference-flap and switch-failed latches cleared on "
			"the next discipline pass");
	return 0;
}

/* ------------------------------------------------------------------------- */
/* pwr — the rubidium recovery actions                                       */
/* ------------------------------------------------------------------------- */

/** Shared reply for the three deferred pwrseq requests. */
static int pwr_request(const struct shell *sh, int rc, const char *what)
{
	if (rc != 0) {
		shell_error(sh, "%s refused (%d)%s", what, rc,
			    (rc == -EBUSY)
				    ? " - the same request is already pending"
				    : ((rc == -ENODEV)
					       ? " - the sequencer has not started"
					       : ""));
		return rc;
	}
	shell_print(sh, "%s queued for the sequencer; the outcome is in the "
			"log (`sts log tail`)",
		    what);
	return 0;
}

static int cmd_pwr_ovclear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mutating_allowed(sh)) {
		return -EACCES;
	}
	return pwr_request(sh, sts_pwrseq_ov_clear(), "Rb OV-latch clear");
}

static int cmd_pwr_rbretry(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mutating_allowed(sh)) {
		return -EACCES;
	}
	return pwr_request(sh, sts_pwrseq_rb_retry(), "Rb sequence retry");
}

static int cmd_pwr_rb(const struct shell *sh, size_t argc, char **argv)
{
	sts_rb_serial_t s;
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	rc = sts_rb_serial_status(&s);
	if (rc != 0) {
		shell_error(sh, "Rb serial status unavailable (%d)", rc);
		return rc;
	}

	shell_print(sh, "K1 relay     %s", (s.mode == 0U) ? "RS-232 (U46)"
							  : "direct CMOS");
	shell_print(sh, "RB_PWR_EN    %s%s", s.rail_up ? "asserted" : "off",
		    s.rail_up ? "" : "  (U46 has no supply; the link is dead)");
	shell_print(sh, "RB_LOCK      %s", s.locked ? "locked" : "not locked");
	shell_print(sh, "tunnel       %s", s.tunnel_open ? "OPEN (a maintenance "
							   "session owns UART7)"
							 : "closed");
	shell_print(sh, "bytes        tx %u, rx %u, overruns %u", s.tx_bytes,
		    s.rx_bytes, s.overruns);
	return 0;
}

/*
 * `sts pwr rbmode <rs232|cmos>` — commission the K1 position.
 *
 * Which one the fitted FE-5680A needs is a property of the surplus variant, not
 * of the board, and the failure mode of choosing wrong is silence rather than
 * damage. FMT §5.2 puts the K1 serial-mode relay in G1; on this plane the
 * equivalent gate is mutating_allowed().
 */
static int cmd_pwr_rbmode(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t mode;
	int rc;

	ARG_UNUSED(argc);

	if (!mutating_allowed(sh)) {
		return -EACCES;
	}
	if (strcmp(argv[1], "rs232") == 0) {
		mode = 0U;
	} else if (strcmp(argv[1], "cmos") == 0) {
		mode = 1U;
	} else {
		shell_error(sh, "usage: sts pwr rbmode <rs232|cmos>");
		return -EINVAL;
	}

	rc = sts_rb_serial_set_mode(mode);
	if (rc != 0) {
		shell_error(sh, "relay refused (%d)%s", rc,
			    (rc == -EBUSY) ? " - a raw tunnel holds the port"
					   : "");
		return rc;
	}
	shell_print(sh, "K1 now in the %s position",
		    (mode == 0U) ? "RS-232" : "CMOS");
	return 0;
}

/* ------------------------------------------------------------------------- */
/* cal tempco — the §10.4 OCXO characterisation fit                          */
/* ------------------------------------------------------------------------- */

/*
 * Operator-entered samples, fitted on demand.
 *
 * Deliberately NOT self-sampling. disc.h states that tempco learning is offline
 * and that the runtime only ever *applies* the stored coefficient, and it is
 * right: while the loop is disciplined it is actively CANCELLING the tempco, so
 * the residual it publishes is the one quantity that cannot measure it. The
 * measurement is a chamber sweep against a reference counter, and the operator
 * is the only thing on the board that has those numbers.
 *
 * What this buys is the arithmetic and the refusal — a technician at a bench
 * gets the slope, the intercept and R^2 without a spreadsheet, and a data set
 * that cannot support a slope is refused instead of being pasted into
 * cal.tempco, which the discipline loop applies as feed-forward.
 *
 * The buffer is static because the Zephyr shell dispatches one command at a
 * time and 192 bytes has no business on the console thread's stack.
 */
static struct {
	float temp_c[STS_TEMPCO_MAX_SAMPLES];
	float osc_ppb[STS_TEMPCO_MAX_SAMPLES];
	uint8_t n;
} tempco;

static int cmd_cal_tempco_add(const struct shell *sh, size_t argc, char **argv)
{
	float t;
	float y;

	ARG_UNUSED(argc);

	if (tempco.n >= (uint8_t)STS_TEMPCO_MAX_SAMPLES) {
		shell_error(sh, "the sample set is full (%u); `sts cal tempco "
				"clear` first",
			    STS_TEMPCO_MAX_SAMPLES);
		return -ENOSPC;
	}
	if ((parse_f32(argv[1], &t) != 0) || (parse_f32(argv[2], &y) != 0)) {
		shell_error(sh, "usage: sts cal tempco add <temp_c> <osc_ppb>");
		return -EINVAL;
	}

	tempco.temp_c[tempco.n] = t;
	tempco.osc_ppb[tempco.n] = y;
	tempco.n++;
	shell_print(sh, "%u/%u samples", tempco.n, STS_TEMPCO_MAX_SAMPLES);
	return 0;
}

static int cmd_cal_tempco_clear(const struct shell *sh, size_t argc,
				char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	tempco.n = 0U;
	shell_print(sh, "tempco sample set cleared");
	return 0;
}

static int cmd_cal_tempco_list(const struct shell *sh, size_t argc, char **argv)
{
	char f1[24];
	char f2[24];

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (uint8_t i = 0U; i < tempco.n; i++) {
		shell_print(sh, "  %2u  %8s C  %10s ppb", i,
			    fmt_f32(f1, sizeof(f1), tempco.temp_c[i], 2),
			    fmt_f32(f2, sizeof(f2), tempco.osc_ppb[i], 3));
	}
	shell_print(sh, "%u/%u samples, span %s C", tempco.n,
		    STS_TEMPCO_MAX_SAMPLES,
		    fmt_f32(f1, sizeof(f1),
			    sts_tempco_span(tempco.temp_c, tempco.n), 2));
	return 0;
}

static int cmd_cal_tempco_fit(const struct shell *sh, size_t argc, char **argv)
{
	sts_tempco_verdict_t v;
	char f1[24];
	char f2[24];
	float slope = 0.0f;
	float offset = 0.0f;
	float r2 = 0.0f;
	float span;
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	rc = disc_tempco_fit(tempco.temp_c, tempco.osc_ppb, tempco.n, &slope,
			     &offset, &r2);
	if (rc != 0) {
		shell_error(sh, "fit failed (%d)%s", rc,
			    (rc == -EDOM) ? " - the temperature does not vary"
					  : "");
		return rc;
	}

	span = sts_tempco_span(tempco.temp_c, tempco.n);
	shell_print(sh, "n %u, span %s C", tempco.n,
		    fmt_f32(f1, sizeof(f1), span, 2));
	shell_print(sh, "slope  %s ppb/C", fmt_f32(f1, sizeof(f1), slope, 4));
	shell_print(sh, "offset %s ppb   R2 %s",
		    fmt_f32(f1, sizeof(f1), offset, 3),
		    fmt_f32(f2, sizeof(f2), r2, 4));

	v = sts_tempco_check(tempco.n, span, r2, slope);
	if (v != STS_TEMPCO_OK) {
		shell_error(sh, "not usable: %s", sts_tempco_verdict_name(v));
		return -ERANGE;
	}

	shell_print(sh, "accepted. To apply it:");
	shell_print(sh, "  sts cfg set cal.tempco %s",
		    fmt_f32(f1, sizeof(f1), slope, 4));
	shell_print(sh, "  sts cfg commit");
	return 0;
}

/* ------------------------------------------------------------------------- */
/* sec — AAA lockouts and the NTS cookie keyring                             */
/* ------------------------------------------------------------------------- */

/*
 * The ATECC608B's own transport counters.
 *
 * This exists because conf/security.conf declines CONFIG_I2C_STATS on the
 * stated grounds that "core/atecc already counts wakes, retries, CRC errors and
 * bus errors itself (atecc_stats_t), and sts_atecc_stats() surfaces them —
 * which is the same information without the system-wide cost". That trade was
 * false: sts_atecc_stats() had no caller, so it was dropped from the image and
 * the information was available nowhere. Either the justification or this
 * command had to exist; the command is the cheaper of the two, and it is the
 * one an operator debugging a marginal I2C1 pull-up actually wants.
 */
static int cmd_sec_atecc(const struct shell *sh, size_t argc, char **argv)
{
	atecc_stats_t s;
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	rc = sts_atecc_stats(&s);
	if (rc != 0) {
		shell_error(sh, "secure element not available (%d)", rc);
		return rc;
	}

	shell_print(sh, "commands      %u", s.commands);
	shell_print(sh, "retries       %u", s.retries);
	shell_print(sh, "crc errors    %u", s.crc_errors);
	shell_print(sh, "status errors %u", s.status_errors);
	shell_print(sh, "wakes         %u  (failed %u)", s.wakes, s.wake_fails);
	shell_print(sh, "bus errors    %u", s.bus_errors);
	return 0;
}

static int cmd_sec_aaa(const struct shell *sh, size_t argc, char **argv)
{
	auth_stats_t s;
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (sts_aaa_stats == NULL) {
		shell_error(sh, "no net area in this image");
		return -ENOSYS;
	}
	rc = sts_aaa_stats(&s);
	if (rc != 0) {
		shell_error(sh, "AAA is not running (%d)", rc);
		return rc;
	}

	shell_print(sh, "checks       %u", s.checks);
	shell_print(sh, "accepts      %u  (cache hits %u, fills %u)", s.accepts,
		    s.cache_hits, s.cache_fills);
	shell_print(sh, "rejects      %u", s.rejects);
	shell_print(sh, "locked out   %u", s.locked);
	shell_print(sh, "unavailable  %u  (no backend could answer)",
		    s.unavailable);
	for (unsigned int b = 0U; b < (unsigned int)AUTH_BE_COUNT; b++) {
		shell_print(sh, "  %-8s   %u accepts",
			    auth_backend_name((uint8_t)b), s.by_backend[b]);
	}
	return 0;
}

static int cmd_sec_unlock(const struct shell *sh, size_t argc, char **argv)
{
	int rc;

	ARG_UNUSED(argc);

	if (!mutating_allowed(sh)) {
		return -EACCES;
	}
	if (sts_aaa_unlock == NULL) {
		shell_error(sh, "no net area in this image");
		return -ENOSYS;
	}

	rc = sts_aaa_unlock(argv[1]);
	if (rc != 0) {
		shell_error(sh, "unlock failed (%d)%s", rc,
			    (rc == -EHOSTUNREACH) ? " - AAA is not running"
						  : "");
		return rc;
	}
	shell_print(sh, "lockout cleared for '%s'", argv[1]);
	sts_log((uint8_t)LOGR_SUB_SEC, (uint8_t)LOGR_NOTICE,
		"account lockout cleared from the local console");
	return 0;
}

static int cmd_sec_aaaflush(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mutating_allowed(sh)) {
		return -EACCES;
	}
	if (sts_aaa_flush == NULL) {
		shell_error(sh, "no net area in this image");
		return -ENOSYS;
	}

	sts_aaa_flush();
	shell_print(sh, "every cached AAA decision dropped; the next login "
			"re-runs the chain");
	return 0;
}

static int cmd_sec_ntsrotate(const struct shell *sh, size_t argc, char **argv)
{
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!mutating_allowed(sh)) {
		return -EACCES;
	}
	if (sts_nts_rotate_now == NULL) {
		shell_error(sh, "no net area in this image");
		return -ENOSYS;
	}

	rc = sts_nts_rotate_now();
	if (rc != 0) {
		shell_error(sh, "NTS key rotation refused (%d)%s", rc,
			    (rc == -ENOTSUP) ? " - NTS is disabled" : "");
		return rc;
	}
	shell_print(sh, "cookie key rotation queued. Outstanding cookies keep "
			"working until they age out of the ring.");
	sts_log((uint8_t)LOGR_SUB_SEC, (uint8_t)LOGR_NOTICE,
		"NTS cookie key rotation forced from the local console");
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
	SHELL_CMD_ARG(attest, NULL, "secure-element report and anti-rollback state",
		      cmd_sec_attest, 1, 0),
	SHELL_CMD_ARG(atecc, NULL, "ATECC608B transport counters (wakes, retries, CRC)",
		      cmd_sec_atecc, 1, 0),
	SHELL_CMD_ARG(aaa, NULL, "AAA counters (checks, lockouts, per-backend)",
		      cmd_sec_aaa, 1, 0),
	SHELL_CMD_ARG(unlock, NULL, "unlock <user> - clear a brute-force lockout",
		      cmd_sec_unlock, 2, 0),
	SHELL_CMD_ARG(aaaflush, NULL, "drop every cached AAA decision",
		      cmd_sec_aaaflush, 1, 0),
	SHELL_CMD_ARG(ntsrotate, NULL, "force an NTS cookie-key rotation now",
		      cmd_sec_ntsrotate, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_sts_gnss,
	SHELL_CMD_ARG(show, NULL, "antenna, survey-in, position and MON-RF",
		      cmd_gnss_show, 1, 0),
	SHELL_CMD_ARG(survey, NULL, "start survey-in (discards the stored position)",
		      cmd_gnss_survey, 1, 0),
	SHELL_CMD_ARG(reenable, NULL, "restore antenna bias after a latched short",
		      cmd_gnss_reenable, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_sts_clock,
	SHELL_CMD_ARG(show, NULL, "reference request/state and CSS events",
		      cmd_clock_show, 1, 0),
	SHELL_CMD_ARG(flapclear, NULL, "clear the reference-flap latch",
		      cmd_clock_flapclear, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_sts_pwr,
	SHELL_CMD_ARG(ovclear, NULL, "clear the Rb over-voltage latch (RB_OV_RESET)",
		      cmd_pwr_ovclear, 1, 0),
	SHELL_CMD_ARG(rbretry, NULL, "re-run the guarded rubidium sequence",
		      cmd_pwr_rbretry, 1, 0),
	SHELL_CMD_ARG(rb, NULL, "FE-5680A serial link: relay, lock, counters",
		      cmd_pwr_rb, 1, 0),
	SHELL_CMD_ARG(rbmode, NULL, "rbmode <rs232|cmos> - move the K1 relay",
		      cmd_pwr_rbmode, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_sts_cal_tempco,
	SHELL_CMD_ARG(add, NULL, "add <temp_c> <osc_ppb> - one sweep sample",
		      cmd_cal_tempco_add, 3, 0),
	SHELL_CMD_ARG(list, NULL, "the samples held so far", cmd_cal_tempco_list,
		      1, 0),
	SHELL_CMD_ARG(clear, NULL, "discard them", cmd_cal_tempco_clear, 1, 0),
	SHELL_CMD_ARG(fit, NULL, "least-squares df/dT, with an acceptance check",
		      cmd_cal_tempco_fit, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_sts_cal,
	SHELL_CMD(tempco, &sub_sts_cal_tempco,
		  "OCXO tempco characterisation (spec 10.4)", NULL),
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
	SHELL_CMD_ARG(sky, NULL, "the §6.3 polar skyplot, as characters",
		      cmd_diag_sky, 1, 0),
	SHELL_CMD_ARG(identify, NULL, "identify [ms] - pulse the status RGB blue",
		      cmd_diag_identify, 1, 1),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_sts,
	SHELL_CMD_ARG(status, NULL, "status [group] - summary, or one encoded "
				    "STATUS_GET group",
		      cmd_status, 1, 1),
	SHELL_CMD_ARG(quality, NULL, "the §3.8 clock-quality block", cmd_quality,
		      1, 0),
	SHELL_CMD_ARG(alarms, NULL, "alarms [clear <id|name|all>]", cmd_alarms,
		      1, 2),
	SHELL_CMD(log, &sub_sts_log, "structured log", NULL),
	SHELL_CMD(cfg, &sub_sts_cfg, "configuration registry", NULL),
	SHELL_CMD(sec, &sub_sts_sec, "security provisioning", NULL),
	SHELL_CMD(fw, &sub_sts_fw, "firmware slots and DFU state", NULL),
	SHELL_CMD(gnss, &sub_sts_gnss, "GNSS receiver and antenna", NULL),
	SHELL_CMD(clock, &sub_sts_clock, "reference selection and recovery", NULL),
	SHELL_CMD(pwr, &sub_sts_pwr, "power-sequencer recovery actions", NULL),
	SHELL_CMD(cal, &sub_sts_cal, "bench calibration", NULL),
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
