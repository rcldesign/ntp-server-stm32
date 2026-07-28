/*
 * STS1000 "Meridian" — cross-area application state (platform-owned).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Implements everything in sts_app.h that is not an area entry point: the
 * quality seqlock, the TAI time source registry, the single cfg context, the
 * log ring, the liveness registry and the MCP status encoder registry.
 *
 * All state here is static. Nothing allocates (ARCHITECTURE.md §4).
 */

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/toolchain.h>

#include "zephyr/sts_app.h"
#include "zephyr/sts_cfg_applier.h"
#include "zephyr/platform/platform.h"

#include "fault/fault.h"
#include "mcp/mcp_wire.h"

LOG_MODULE_REGISTER(sts_app, CONFIG_STS1000_LOG_LEVEL);

/* ========================================================================= */
/* monotonic time                                                            */
/* ========================================================================= */

uint64_t sts_mono_ms(void)
{
	return (uint64_t)k_uptime_get();
}

/* ========================================================================= */
/* quality seqlock                                                           */
/* ========================================================================= */

/*
 * quality_state_t is the seqlock itself (core/quality). The publisher is the
 * discipline thread and nothing else; readers are every management thread.
 * No mutex is involved on either side — ARCHITECTURE.md §10 invariant 10.
 */
static quality_state_t sts_quality_state;

quality_state_t *sts_app_quality_state(void)
{
	return &sts_quality_state;
}

int sts_quality_snapshot(quality_block_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}

	return quality_snapshot(&sts_quality_state, out);
}

/* ========================================================================= */
/* TAI time source                                                           */
/* ========================================================================= */

/*
 * Until the net area registers the ETH PTP clock, TAI is synthesised from the
 * monotonic clock with an arbitrary epoch. That is deliberately *not* a
 * plausible wall time: a fallback reading must be obviously wrong to anything
 * that leaks it, and sts_time_is_fallback() is the supported way to ask.
 *
 * fn/ctx are published with release ordering and read with acquire ordering so
 * a reader can never observe a new fn against a stale ctx.
 */
static sts_tai_source_fn sts_tai_fn;
static void *sts_tai_ctx;
static atomic_t sts_tai_registered = ATOMIC_INIT(0);

void sts_time_register_source(sts_tai_source_fn fn, void *ctx)
{
	if (fn == NULL) {
		return;
	}

	sts_tai_ctx = ctx;
	sts_tai_fn = fn;
	/* Release: the two stores above must be visible before the flag. */
	atomic_set(&sts_tai_registered, 1);

	LOG_INF("TAI source registered");
}

int sts_time_tai_ns(uint64_t *out_ns)
{
	sts_tai_source_fn fn;
	void *ctx;

	if (out_ns == NULL) {
		return -EINVAL;
	}

	if (atomic_get(&sts_tai_registered) != 0) {
		fn = sts_tai_fn;
		ctx = sts_tai_ctx;
		if (fn != NULL && fn(ctx, out_ns) == 0) {
			return 0;
		}
	}

	*out_ns = (uint64_t)k_uptime_get() * 1000000ULL;
	return 0;
}

bool sts_time_is_fallback(void)
{
	return atomic_get(&sts_tai_registered) == 0;
}

/*
 * Absolute-epoch traceability, kept separate from source registration on purpose.
 * A registered PTP-clock source answers immediately; whether the epoch it answers
 * with is right depends on GNSS having set it and the servo having converged, and
 * only the net area knows that. Defaults to 0 = not traceable, so an area that
 * never reports leaves the clock advertising UNSYNC rather than claiming stratum 1
 * off an unset hardware clock.
 */
static atomic_t sts_time_traceable = ATOMIC_INIT(0);

bool sts_time_is_traceable(void)
{
	return atomic_get(&sts_time_traceable) != 0;
}

void sts_time_set_traceable(bool traceable)
{
	atomic_val_t was = atomic_set(&sts_time_traceable, traceable ? 1 : 0);

	if ((was != 0) == traceable) {
		return;
	}

	sts_log(LOGR_SUB_TIMING, traceable ? LOGR_NOTICE : LOGR_WARN,
		"served timescale %s",
		traceable ? "traceable (absolute epoch set)"
			  : "NOT traceable (no absolute epoch)");
}

/* ========================================================================= */
/* log ring                                                                  */
/* ========================================================================= */

#define STS_LOGRING_SLOTS CONFIG_STS1000_LOGRING_SLOTS

static logr_rec_t sts_logring_slots[STS_LOGRING_SLOTS];
static logr_t sts_logring_ctx;
static struct k_spinlock sts_logring_lock;

logr_t *sts_logring(void)
{
	return &sts_logring_ctx;
}

void sts_log(uint8_t subsys, uint8_t level, const char *fmt, ...)
{
	char msg[LOGR_MSG_MAX];
	k_spinlock_key_t key;
	va_list ap;
	int n;

	if (fmt == NULL) {
		return;
	}

	va_start(ap, fmt);
	n = vsnprintf(msg, sizeof(msg), fmt, ap);
	va_end(ap);

	if (n < 0) {
		return;
	}

	/*
	 * A spinlock rather than a mutex: logr_put() is a bounded memcpy into a
	 * fixed slot, and callers include the io_scan thread, whose 1 ms budget
	 * cannot absorb a priority-inheritance handoff.
	 */
	key = k_spin_lock(&sts_logring_lock);
	(void)logr_puts(&sts_logring_ctx, level, subsys, sts_mono_ms(), msg);
	k_spin_unlock(&sts_logring_lock, key);
}

/* ========================================================================= */
/* config                                                                    */
/* ========================================================================= */

/*
 * cfg needs a port_store_t at cfg_init() time, but persistence belongs to the
 * console area, which starts after the platform. Rather than order the areas
 * around that, the platform initialises cfg against a RAM store: every key
 * reads its schema default, sets are accepted, and nothing survives a reboot.
 * sts_cfg_register_store() then swaps in the real store and re-runs
 * cfg_load_all(), which is when persisted values appear.
 *
 * The RAM store is a plain fixed table. It exists to keep the "no persistence
 * yet" window behaving exactly like the persistent case, so an area's start
 * function never has to care which one it is talking to.
 */
#define STS_RAM_STORE_ENTRIES CONFIG_STS1000_RAM_STORE_ENTRIES
#define STS_RAM_STORE_VAL_MAX 64

struct sts_ram_store_entry {
	uint16_t id;
	uint8_t len;
	bool used;
	uint8_t val[STS_RAM_STORE_VAL_MAX];
};

static struct sts_ram_store_entry sts_ram_store[STS_RAM_STORE_ENTRIES];

static struct sts_ram_store_entry *sts_ram_find(uint16_t id, bool create)
{
	struct sts_ram_store_entry *free_slot = NULL;

	for (size_t i = 0; i < ARRAY_SIZE(sts_ram_store); i++) {
		if (sts_ram_store[i].used && sts_ram_store[i].id == id) {
			return &sts_ram_store[i];
		}
		if (!sts_ram_store[i].used && free_slot == NULL) {
			free_slot = &sts_ram_store[i];
		}
	}

	if (create && free_slot != NULL) {
		free_slot->used = true;
		free_slot->id = id;
		free_slot->len = 0;
		return free_slot;
	}

	return NULL;
}

static int sts_ram_load(void *ctx, uint16_t id, void *buf, size_t cap)
{
	const struct sts_ram_store_entry *e = sts_ram_find(id, false);

	ARG_UNUSED(ctx);

	if (e == NULL) {
		return -ENOENT;
	}
	if (e->len > cap) {
		return -ENOMEM;
	}

	memcpy(buf, e->val, e->len);
	return (int)e->len;
}

static int sts_ram_save(void *ctx, uint16_t id, const void *buf, size_t len)
{
	struct sts_ram_store_entry *e;

	ARG_UNUSED(ctx);

	if (len > STS_RAM_STORE_VAL_MAX) {
		return -ENOMEM;
	}

	e = sts_ram_find(id, true);
	if (e == NULL) {
		return -ENOSPC;
	}

	memcpy(e->val, buf, len);
	e->len = (uint8_t)len;
	return 0;
}

static int sts_ram_erase(void *ctx, uint16_t id)
{
	struct sts_ram_store_entry *e = sts_ram_find(id, false);

	ARG_UNUSED(ctx);

	if (e == NULL) {
		return -ENOENT;
	}

	e->used = false;
	e->len = 0;
	return 0;
}

static const port_store_t sts_ram_store_port = {
	.load = sts_ram_load,
	.save = sts_ram_save,
	.erase = sts_ram_erase,
	.ctx = NULL,
};

static cfg_ctx_t sts_cfg_ctx;
static bool sts_cfg_persistent;

/*
 * Applier registry: a chain per group, not one slot. Rationale, the arithmetic
 * and the -ENOSPC contract are all in sts_cfg_applier.h, which is Zephyr-free so
 * tests/host can pin the behaviour down.
 */
static sts_cfg_applier_tbl_t sts_cfg_applier_tbl;

static struct k_mutex sts_cfg_mutex;

cfg_ctx_t *sts_cfg(void)
{
	return &sts_cfg_ctx;
}

void sts_cfg_lock(void)
{
	k_mutex_lock(&sts_cfg_mutex, K_FOREVER);
}

void sts_cfg_unlock(void)
{
	k_mutex_unlock(&sts_cfg_mutex);
}

bool sts_cfg_is_persistent(void)
{
	return sts_cfg_persistent;
}

int sts_cfg_register_applier(uint8_t group, sts_cfg_apply_fn fn, void *ctx)
{
	return sts_cfg_applier_add(&sts_cfg_applier_tbl, group, fn, ctx);
}

static void sts_cfg_apply_group(uint8_t group)
{
	sts_cfg_applier_dispatch(&sts_cfg_applier_tbl, group);
}

/*
 * cfg_init() memsets the whole context, which drops the cross-field validation
 * hook along with everything else. Anything that re-inits the live context has
 * to come back through here or the schema's joint constraints (cfg.h
 * cfg_schema_xvalidate: snmp.enable needs a non-default community) stop being
 * enforced from that point on. Caller holds sts_cfg_mutex.
 */
static void sts_cfg_bind_hooks(void)
{
	(void)cfg_set_validate_hook(&sts_cfg_ctx, cfg_schema_xvalidate, NULL);
}

int sts_cfg_register_store(const port_store_t *store)
{
	uint32_t corrupt = 0;
	int rc;

	if (store == NULL || store->load == NULL || store->save == NULL) {
		return -EINVAL;
	}
	if (sts_cfg_persistent) {
		return -EALREADY;
	}

	k_mutex_lock(&sts_cfg_mutex, K_FOREVER);

	rc = cfg_init(&sts_cfg_ctx, store);
	if (rc == 0) {
		sts_cfg_bind_hooks();
		rc = cfg_load_all(&sts_cfg_ctx, &corrupt);
	}

	if (rc == 0) {
		sts_cfg_persistent = true;
	}

	k_mutex_unlock(&sts_cfg_mutex);

	if (rc != 0) {
		LOG_ERR("cfg: persistent store rejected (%d); staying on RAM defaults", rc);
		/*
		 * cfg_init() has already re-pointed the context at the failed
		 * store, so put the RAM store back rather than leave cfg
		 * talking to something that does not work.
		 */
		k_mutex_lock(&sts_cfg_mutex, K_FOREVER);
		(void)cfg_init(&sts_cfg_ctx, &sts_ram_store_port);
		sts_cfg_bind_hooks();
		(void)cfg_load_all(&sts_cfg_ctx, NULL);
		k_mutex_unlock(&sts_cfg_mutex);
		return rc;
	}

	if (corrupt != 0) {
		LOG_WRN("cfg: %u stored keys were corrupt and fell back to defaults",
			(unsigned int)corrupt);
	}

	/* Everything that registered an applier gets one call with the loaded
	 * values, so an area never has to special-case "before persistence". */
	for (uint8_t g = 0; g < STS_CFG_GROUP_MAX; g++) {
		sts_cfg_apply_group(g);
	}

	LOG_INF("cfg: persistent store active");
	return 0;
}

int sts_cfg_commit(cfg_commit_res_t *res)
{
	cfg_commit_res_t local;
	uint16_t touched[STS_CFG_GROUP_MAX];
	size_t n_touched = 0;
	int rc;

	if (res == NULL) {
		res = &local;
	}

	k_mutex_lock(&sts_cfg_mutex, K_FOREVER);

	/*
	 * Collect the affected groups BEFORE committing: cfg_commit() clears
	 * the staged set, so cfg_is_staged() answers false afterwards.
	 */
	for (size_t i = 0; i < cfg_key_count(); i++) {
		const cfg_key_t *k = cfg_key_at(i);
		uint8_t group;
		bool seen = false;

		if (k == NULL || !cfg_is_staged(&sts_cfg_ctx, k->id)) {
			continue;
		}

		group = (uint8_t)(k->id >> 8);
		for (size_t j = 0; j < n_touched; j++) {
			if (touched[j] == group) {
				seen = true;
				break;
			}
		}
		if (!seen && n_touched < ARRAY_SIZE(touched)) {
			touched[n_touched++] = group;
		}
	}

	rc = cfg_commit(&sts_cfg_ctx, res);

	k_mutex_unlock(&sts_cfg_mutex);

	/* -EIO means the live tree changed but some keys missed the store, so the
	 * appliers still have to run — see sts_cfg_commit_applied(). */
	if (!sts_cfg_commit_applied(rc)) {
		return rc;
	}

	for (size_t j = 0; j < n_touched; j++) {
		sts_cfg_apply_group((uint8_t)touched[j]);
	}

	return rc;
}

int sts_cfg_factory_reset(void)
{
	int rc;

	k_mutex_lock(&sts_cfg_mutex, K_FOREVER);
	rc = cfg_factory_reset(&sts_cfg_ctx);
	k_mutex_unlock(&sts_cfg_mutex);

	/*
	 * Same rule as sts_cfg_commit(): -EIO is "the live tree was reset but some
	 * store erases failed", and a live tree that changed has to be announced.
	 * Only a hard rejection, where nothing was applied, skips the dispatch.
	 */
	if (!sts_cfg_commit_applied(rc)) {
		LOG_ERR("cfg: factory reset failed (%d); appliers not run", rc);
		return rc;
	}

	/*
	 * A commit dispatches only the groups it staged; a factory reset rewrote
	 * every key, so every registered group is told — the same full sweep
	 * sts_cfg_register_store() does once the real store is in place. Dispatched
	 * with the mutex RELEASED, because an applier may block on sockets, DNS or
	 * display I/O (sts_app.h).
	 */
	for (uint8_t g = 0; g < STS_CFG_GROUP_MAX; g++) {
		sts_cfg_apply_group(g);
	}

	LOG_WRN("cfg: factory reset applied; every group re-applied");
	return rc;
}

/* ========================================================================= */
/* liveness registry                                                         */
/* ========================================================================= */

/*
 * The external TPS3430 is a *windowed* watchdog: a kick that is too early is
 * as fatal as one that is too late (docs/sts1000_external_wdt.md). The
 * supervisor therefore needs to know not only that everyone is alive but that
 * nobody has gone quiet, which is what the per-participant deadline gives it.
 *
 * STS_LIVENESS_MAX must be >= the number of sts_liveness_register() call sites
 * that can run, because overflowing it is SILENT in the direction that matters:
 * register() returns -ENOSPC, the caller stores a negative id, every feed
 * becomes a no-op, and that participant is simply never watched again. Nothing
 * logs it and the board still kicks the watchdog. That inverts the policy in
 * platform/sts_super_policy.h, which exists to make "a single late participant
 * must withhold the kick" true.
 *
 * It was 12 against 15 live registrants, so three were already being dropped;
 * enabling NTS-KE (conf/net.conf) added a 16th and would have dropped a fourth.
 * The registrants today, by area:
 *
 *   platform  io_scan, housekeeping, discipline, gnss, pwrseq          (5)
 *   console   console, mcp, logger                                     (3)
 *   net       ptp, ntp, net_mgmt, ntske, snmp, syslog, web             (7)
 *   ui        ui                                                       (1)
 *
 * 20 leaves four spare. The ceiling is 32, not this constant: the mask returned
 * by sts_liveness_stale_mask() is a uint32_t of BIT(i), so a 33rd participant
 * would be undefined rather than merely unwatched. Do not raise this past 32
 * without widening that mask and sts_super_wdt_liveness() with it.
 */
#define STS_LIVENESS_MAX 20
#define STS_LIVENESS_DEADLINE_MS CONFIG_STS1000_LIVENESS_DEADLINE_MS

BUILD_ASSERT(STS_LIVENESS_MAX <= 32,
	     "sts_liveness_stale_mask() returns BIT(i) in a uint32_t");

struct sts_liveness_entry {
	const char *name;
	atomic_t last_ms;
};

static struct sts_liveness_entry sts_liveness[STS_LIVENESS_MAX];
static atomic_t sts_liveness_n = ATOMIC_INIT(0);

int sts_liveness_register(const char *name)
{
	atomic_val_t id;

	if (name == NULL) {
		return -EINVAL;
	}

	id = atomic_inc(&sts_liveness_n);
	if (id >= STS_LIVENESS_MAX) {
		atomic_dec(&sts_liveness_n);
		return -ENOSPC;
	}

	sts_liveness[id].name = name;
	/* Seed as fed, so a participant registered late does not immediately
	 * count as stale and stall the watchdog kick. */
	atomic_set(&sts_liveness[id].last_ms, (atomic_val_t)k_uptime_get_32());

	return (int)id;
}

void sts_liveness_feed(int id)
{
	if (id < 0 || id >= (int)atomic_get(&sts_liveness_n)) {
		return;
	}

	atomic_set(&sts_liveness[id].last_ms, (atomic_val_t)k_uptime_get_32());
}

uint32_t sts_liveness_count(void)
{
	return (uint32_t)atomic_get(&sts_liveness_n);
}

const char *sts_liveness_name(uint32_t id)
{
	if (id >= sts_liveness_count()) {
		return NULL;
	}

	return sts_liveness[id].name;
}

uint32_t sts_liveness_stale_mask(uint32_t now_ms)
{
	uint32_t n = sts_liveness_count();
	uint32_t mask = 0;

	for (uint32_t i = 0; i < n; i++) {
		uint32_t last = (uint32_t)atomic_get(&sts_liveness[i].last_ms);

		/* Unsigned difference: wrap-safe across the 49.7-day k_uptime_get_32
		 * rollover without any special case. */
		if ((now_ms - last) > STS_LIVENESS_DEADLINE_MS) {
			mask |= BIT(i);
		}
	}

	return mask;
}

/* ========================================================================= */
/* alarms                                                                    */
/* ========================================================================= */

uint64_t sts_alarms_active(void)
{
	uint64_t mask;

	sts_fault_lock();
	mask = fault_alarms(sts_fault());
	sts_fault_unlock();

	return mask;
}

int sts_alarm_set(uint8_t alarm_id, bool active)
{
	int rc;

	/*
	 * Ids 0..31 mirror the scanned signals and are driven by
	 * fault_scan_input(); letting another thread force one would make the
	 * scan and the alarm table disagree on the next tick.
	 */
	if (alarm_id < FAULT_SIG_COUNT || alarm_id >= FAULT_ALARM_COUNT) {
		return -EINVAL;
	}

	sts_fault_lock();
	rc = fault_alarm_set(sts_fault(), (fault_alarm_id_t)alarm_id, active,
			     k_uptime_get_32());
	sts_fault_unlock();

	return rc;
}

uint64_t sts_alarms_latched(void)
{
	uint64_t mask;

	sts_fault_lock();
	mask = fault_alarms_latched(sts_fault());
	sts_fault_unlock();

	return mask;
}

int sts_alarm_clear(uint8_t alarm_id)
{
	int rc;

	/*
	 * Note the deliberate asymmetry with sts_alarm_set() above: ids 0..31
	 * ARE accepted here. That function refuses them because the 1 kHz scan
	 * owns the scanned signals' ACTIVE state and a second writer would make
	 * the two disagree. Clearing touches only the LATCH — a historical
	 * record fault_scan_input() never writes — and a technician who cannot
	 * acknowledge a power-good glitch that has since gone away has no use
	 * for the latch at all. core/fault still refuses to clear a latch whose
	 * condition is live (-EBUSY), which is the safety property.
	 */
	if (alarm_id >= FAULT_ALARM_COUNT) {
		return -EINVAL;
	}

	sts_fault_lock();
	rc = fault_alarm_clear(sts_fault(), (fault_alarm_id_t)alarm_id);
	sts_fault_unlock();

	return rc;
}

int sts_alarm_clear_all(void)
{
	int rc;

	sts_fault_lock();
	rc = fault_alarm_clear_all(sts_fault());
	sts_fault_unlock();

	return rc;
}

/* ========================================================================= */
/* MCP status encoders                                                       */
/* ========================================================================= */

struct sts_status_slot {
	sts_status_encode_fn fn;
	void *ctx;
};

static struct sts_status_slot sts_status_slots[MCP_GRP_COUNT];

int sts_status_register_encoder(uint8_t group, sts_status_encode_fn fn, void *ctx)
{
	if (group >= MCP_GRP_COUNT) {
		return -EINVAL;
	}

	sts_status_slots[group].ctx = ctx;
	sts_status_slots[group].fn = fn;
	return 0;
}

/* --- little-endian packing helpers ---------------------------------------
 *
 * The status structs are packed by hand rather than memcpy'd from a struct:
 * the wire layout must not change when the compiler pads a field, and every
 * multi-byte field is little-endian (mcp_wire.h).
 */
struct sts_pack {
	uint8_t *buf;
	size_t cap;
	size_t len;
	bool overflow;
};

static void pk_u8(struct sts_pack *p, uint8_t v)
{
	if (p->len + 1U > p->cap) {
		p->overflow = true;
		return;
	}
	p->buf[p->len++] = v;
}

static void pk_u16(struct sts_pack *p, uint16_t v)
{
	if (p->len + 2U > p->cap) {
		p->overflow = true;
		return;
	}
	sys_put_le16(v, &p->buf[p->len]);
	p->len += 2U;
}

static void pk_u32(struct sts_pack *p, uint32_t v)
{
	if (p->len + 4U > p->cap) {
		p->overflow = true;
		return;
	}
	sys_put_le32(v, &p->buf[p->len]);
	p->len += 4U;
}

static void pk_u64(struct sts_pack *p, uint64_t v)
{
	if (p->len + 8U > p->cap) {
		p->overflow = true;
		return;
	}
	sys_put_le64(v, &p->buf[p->len]);
	p->len += 8U;
}

static void pk_i8(struct sts_pack *p, int8_t v)
{
	pk_u8(p, (uint8_t)v);
}

static void pk_i16(struct sts_pack *p, int16_t v)
{
	pk_u16(p, (uint16_t)v);
}

static void pk_i32(struct sts_pack *p, int32_t v)
{
	pk_u32(p, (uint32_t)v);
}

static void pk_i64(struct sts_pack *p, int64_t v)
{
	pk_u64(p, (uint64_t)v);
}

/*
 * Floats cross the wire as scaled integers, never as IEEE-754 bytes: the PC
 * tool must not have to agree with the firmware on float representation, and
 * every one of these quantities has a natural fixed-point unit anyway.
 */
static int32_t sat_f_to_i32(float v, float scale)
{
	double s = (double)v * (double)scale;

	if (!(s > -2147483648.0)) {
		return INT32_MIN;
	}
	if (!(s < 2147483647.0)) {
		return INT32_MAX;
	}

	return (int32_t)s;
}

/** Status struct layout version. Bump on any field add/remove/reorder. */
#define STS_STATUS_VER 1u

static int status_encode_summary(uint8_t *buf, size_t cap)
{
	quality_block_t q;
	sts_hk_snapshot_t hk;
	struct sts_pack p = { .buf = buf, .cap = cap, .len = 0, .overflow = false };
	uint64_t alarms;
	bool hk_ok;

	(void)sts_quality_snapshot(&q);
	hk_ok = (sts_hk_read(&hk) == 0);
	alarms = sts_alarms_active();

	pk_u8(&p, STS_STATUS_VER);
	pk_u8(&p, q.stratum);
	pk_u8(&p, q.lock_state);
	pk_u8(&p, q.active_ref);
	pk_u8(&p, q.gnss_fix);
	pk_u8(&p, q.gnss_sv_used);
	pk_u8(&p, (uint8_t)(sts_time_is_fallback() ? 1U : 0U));
	pk_u8(&p, (uint8_t)(sts_update_pending_confirm() ? 1U : 0U));
	pk_u32(&p, q.flags);
	pk_u64(&p, alarms);
	pk_u32(&p, (uint32_t)(k_uptime_get() / 1000));
	pk_i32(&p, q.last_pps_off_ns);
	pk_i32(&p, sat_f_to_i32(q.freq_err_ppb, 1000.0f)); /* ppt */
	pk_i32(&p, hk_ok ? hk.temp_enclosure_mc : 0);
	pk_i32(&p, hk_ok ? hk.temp_osc_mc : 0);
	pk_u32(&p, hk_ok ? hk.fan_rpm : 0U);

	return p.overflow ? -ENOMEM : (int)p.len;
}

static int status_encode_timing(uint8_t *buf, size_t cap)
{
	quality_block_t q;
	struct sts_pack p = { .buf = buf, .cap = cap, .len = 0, .overflow = false };
	uint32_t captures = 0;
	uint32_t lost = 0;
	uint32_t extref_hz = 0;
	bool extref_valid = false;
	bool extref_edges = false;

	(void)sts_quality_snapshot(&q);
	sts_pps_counters(&captures, &lost);
	sts_extref_mon_read(&extref_hz, &extref_valid, &extref_edges);

	pk_u8(&p, STS_STATUS_VER);
	pk_u8(&p, q.lock_state);
	pk_u8(&p, q.active_ref);
	pk_u8(&p, (uint8_t)((extref_valid ? 1U : 0U) | (extref_edges ? 2U : 0U) |
			    ((sts_clkmux_get() > 0) ? 4U : 0U)));
	pk_u32(&p, q.tick);
	pk_u32(&p, q.flags);
	pk_i32(&p, q.last_pps_off_ns);
	pk_i32(&p, sat_f_to_i32(q.pps_off_mean_ns, 1000.0f));  /* picoseconds */
	pk_i32(&p, sat_f_to_i32(q.pps_off_sigma_ns, 1000.0f)); /* picoseconds */
	pk_i32(&p, sat_f_to_i32(q.freq_err_ppb, 1000.0f));     /* ppt */
	pk_i32(&p, q.vc_cmd_mv);
	pk_i32(&p, q.vc_sense_mv);
	pk_u16(&p, q.dac_code);
	pk_u16(&p, 0U); /* reserved (alignment) */
	pk_u32(&p, sat_f_to_i32(q.adev_1s, 1e18f) < 0 ? 0U
		    : (uint32_t)sat_f_to_i32(q.adev_1s, 1e18f)); /* 1e-18 units */
	pk_u32(&p, sat_f_to_i32(q.adev_10s, 1e18f) < 0 ? 0U
		    : (uint32_t)sat_f_to_i32(q.adev_10s, 1e18f));
	pk_u32(&p, sat_f_to_i32(q.adev_100s, 1e18f) < 0 ? 0U
		    : (uint32_t)sat_f_to_i32(q.adev_100s, 1e18f));
	pk_u32(&p, q.root_delay_q16);
	pk_u32(&p, q.root_disp_q16);
	pk_i64(&p, q.holdover_est_err_ns);
	pk_u32(&p, q.holdover_elapsed_s);
	pk_u32(&p, q.holdover_t_demote_s);
	pk_u32(&p, extref_hz);
	pk_u32(&p, captures);
	pk_u32(&p, lost);
	pk_i32(&p, q.osc_temp_mc);

	return p.overflow ? -ENOMEM : (int)p.len;
}

static int status_encode_gnss(uint8_t *buf, size_t cap)
{
	quality_block_t q;
	struct sts_pack p = { .buf = buf, .cap = cap, .len = 0, .overflow = false };

	(void)sts_quality_snapshot(&q);

	pk_u8(&p, STS_STATUS_VER);
	pk_u8(&p, q.gnss_fix);
	pk_u8(&p, q.gnss_sv_used);
	pk_u8(&p, q.gnss_sv_visible);
	pk_u8(&p, (uint8_t)(q.utc_valid ? 1U : 0U));
	pk_i8(&p, q.leap_pending);
	pk_i16(&p, q.leap_current_s);
	pk_u32(&p, q.gnss_tacc_ns);
	pk_u32(&p, q.refid);
	pk_u64(&p, q.leap_at_tai_s);

	return p.overflow ? -ENOMEM : (int)p.len;
}

static int status_encode_power(uint8_t *buf, size_t cap)
{
	sts_health_t h;
	struct sts_pack p = { .buf = buf, .cap = cap, .len = 0, .overflow = false };

	/* Same source as the UI Power/Health page and SNMP: sts_health_t. */
	if (sts_health_snapshot(&h) != 0) {
		memset(&h, 0, sizeof(h));
	}

	pk_u8(&p, STS_STATUS_VER);
	pk_u8(&p, (uint8_t)h.ina_count);
	pk_u8(&p, (uint8_t)(h.tmp_amb_valid ? 1U : 0U));
	pk_u8(&p, (uint8_t)(h.tmp_osc_valid ? 1U : 0U));
	pk_i32(&p, h.tmp_amb_mc);
	pk_i32(&p, h.tmp_osc_mc);
	pk_i32(&p, h.die_mc);
	pk_i32(&p, h.humidity_mpct);
	pk_u32(&p, h.fan_rpm);
	pk_u16(&p, h.fan_duty_pct);
	pk_u8(&p, h.poe_class);
	pk_u8(&p, (uint8_t)((h.bkp_stm_pg ? 1U : 0U) | (h.bkp_gps_pg ? 2U : 0U) |
			    (h.die_valid ? 4U : 0U) | (h.humidity_valid ? 8U : 0U)));
	pk_u32(&p, h.poe_draw_mw);
	pk_u32(&p, h.poe_budget_mw);

	/* Nine fixed-width rail records, in ina228_rail_t order. */
	for (size_t i = 0; i < STS_HEALTH_INA_COUNT && i < INA228_RAIL_COUNT; i++) {
		pk_u8(&p, ina228_rail_tbl[i].addr);
		pk_u8(&p, (uint8_t)(h.ina[i].valid ? 1U : 0U));
		pk_u16(&p, h.ina[i].diag_alrt);
		pk_i32(&p, h.ina[i].bus_mv);
		pk_i32(&p, h.ina[i].current_ua);
		pk_u32(&p, h.ina[i].power_uw / 1000U); /* milliwatts */
	}

	return p.overflow ? -ENOMEM : (int)p.len;
}

static int status_encode_alarms(uint8_t *buf, size_t cap)
{
	struct sts_pack p = { .buf = buf, .cap = cap, .len = 0, .overflow = false };
	fault_ctx_t *f = sts_fault();
	uint64_t active;
	uint64_t latched;
	uint64_t blocking;
	uint32_t scans = 0;
	uint32_t overruns = 0;

	sts_fault_lock();
	active = fault_alarms(f);
	latched = fault_alarms_latched(f);
	blocking = fault_relay_blocking(f);
	sts_fault_unlock();

	sts_io_scan_counters(&scans, &overruns);

	pk_u8(&p, STS_STATUS_VER);
	pk_u8(&p, (uint8_t)FAULT_ALARM_COUNT);
	pk_u16(&p, 0U); /* reserved */
	pk_u64(&p, active);
	pk_u64(&p, latched);
	pk_u64(&p, blocking);
	pk_u32(&p, scans);
	pk_u32(&p, overruns);

	return p.overflow ? -ENOMEM : (int)p.len;
}

int sts_status_encode(uint8_t group, uint8_t *buf, size_t cap)
{
	if (buf == NULL || cap == 0U) {
		return -EINVAL;
	}
	if (group >= MCP_GRP_COUNT) {
		return -EINVAL;
	}

	/* A registered sub-encoder always wins: it is the area that owns the
	 * data, and the platform's own encoder (if any) is only a default. */
	if (sts_status_slots[group].fn != NULL) {
		return sts_status_slots[group].fn(sts_status_slots[group].ctx,
						  group, buf, cap);
	}

	switch (group) {
	case MCP_GRP_SUMMARY:
		return status_encode_summary(buf, cap);
	case MCP_GRP_TIMING:
		return status_encode_timing(buf, cap);
	case MCP_GRP_GNSS:
		return status_encode_gnss(buf, cap);
	case MCP_GRP_POWER:
		return status_encode_power(buf, cap);
	case MCP_GRP_ALARMS:
		return status_encode_alarms(buf, cap);
	case MCP_GRP_NET:
	case MCP_GRP_PTP:
	default:
		/* No platform-side source. The net area registers these. */
		return -ENOTSUP;
	}
}

/* ========================================================================= */
/* panel mirror                                                              */
/* ========================================================================= */

/*
 * __weak no-op for the ui -> console seam. The console area's mp_glue.c carries
 * the strong definition, but it is compiled only under CONFIG_STS1000_CONSOLE
 * while the ui area (the caller) is gated on CONFIG_STS1000_UI, so UI=y with
 * CONSOLE=n must still link. It lives here rather than in
 * platform/sts_area_weak.c with the other area fallbacks only because
 * sts_area_weak.c is outside this change's file boundary; both files are
 * unconditionally compiled (see app/CMakeLists.txt), so the override behaves
 * identically. Dropping the frame is the correct no-op: with no MP engine there
 * is no mirror to feed.
 */
__weak void sts_mp_mirror_publish(const mp_mirror_in_t *frame)
{
	ARG_UNUSED(frame);
}

/* ========================================================================= */
/* early init                                                                */
/* ========================================================================= */

int sts_app_early_init(void)
{
	int rc;

	k_mutex_init(&sts_cfg_mutex);

	rc = logr_init(&sts_logring_ctx, sts_logring_slots,
		       (uint16_t)ARRAY_SIZE(sts_logring_slots));
	if (rc != 0) {
		return rc;
	}

	rc = quality_state_init(&sts_quality_state);
	if (rc != 0) {
		return rc;
	}

	rc = cfg_init(&sts_cfg_ctx, &sts_ram_store_port);
	if (rc != 0) {
		return rc;
	}
	sts_cfg_bind_hooks();

	/* Populates every key with its schema default; the RAM store is empty
	 * on the first call, so nothing is "loaded" but the bounds and type
	 * checks still run and the context becomes usable. */
	rc = cfg_load_all(&sts_cfg_ctx, NULL);
	if (rc != 0) {
		return rc;
	}

	return 0;
}
