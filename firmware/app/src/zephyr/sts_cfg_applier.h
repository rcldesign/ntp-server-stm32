/*
 * STS1000 "Meridian" — the config-applier registry, as pure logic.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Implementation detail of sts_cfg_register_applier() / sts_cfg_commit()
 * (sts_app.h). Split out and made Zephyr-free so tests/host can compile it,
 * because the bug it exists to prevent is invisible at runtime: a registration
 * that reports success and silently deletes somebody else's applier.
 *
 * Why a chain and not one slot per group
 * -------------------------------------
 * Config groups are genuinely shared. Group 0x09 (log) is claimed by BOTH the
 * console area — which pushes log.level into the log ring's severity floor, the
 * only thing that makes runtime debug escalation work — and the net area, which
 * reloads the syslog sender. With one slot per group, main.c's start order
 * (console before net) meant net's registration overwrote the console's, so
 * log.level was honoured exactly once during store registration and was inert
 * for the rest of the boot. Worse, the bare assignment returned 0, so the
 * caller's "applier rejected" warning could never fire and nothing anywhere
 * said so.
 *
 * The chain is a fixed per-group array: registration order is dispatch order,
 * an exact duplicate is idempotent, and an overflow is reported (-ENOSPC) rather
 * than displacing an existing subscriber.
 */

#ifndef STS1000_ZEPHYR_STS_CFG_APPLIER_H_
#define STS1000_ZEPHYR_STS_CFG_APPLIER_H_

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "zephyr/sts_app.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Config groups (ARCHITECTURE.md §8 uses 0x01..0x0C; 0x10 leaves room). */
#define STS_CFG_GROUP_MAX 0x10

/**
 * Subscribers per group.
 *
 * Sized from the current claims — log has two (console + net), every other group
 * has one — plus headroom. A fifth claim on one group returns -ENOSPC loudly
 * instead of quietly displacing whoever got there first.
 */
#define STS_CFG_GROUP_SUBS_MAX 4

typedef struct {
	sts_cfg_apply_fn fn;
	void            *ctx;
} sts_cfg_sub_t;

/** The registry. Zero-initialised static storage is a valid empty table. */
typedef struct {
	sts_cfg_sub_t sub[STS_CFG_GROUP_MAX][STS_CFG_GROUP_SUBS_MAX];
} sts_cfg_applier_tbl_t;

/**
 * Append a subscriber for @p group.
 *
 * @retval 0        Registered, or the identical (fn, ctx) pair was already there.
 * @retval -EINVAL  @p t or @p fn is NULL, or @p group is out of range.
 * @retval -ENOSPC  The group already has STS_CFG_GROUP_SUBS_MAX subscribers.
 */
static inline int sts_cfg_applier_add(sts_cfg_applier_tbl_t *t, uint8_t group,
				      sts_cfg_apply_fn fn, void *ctx)
{
	size_t i;

	if ((t == NULL) || (fn == NULL) || (group >= STS_CFG_GROUP_MAX)) {
		return -EINVAL;
	}

	for (i = 0U; i < STS_CFG_GROUP_SUBS_MAX; i++) {
		sts_cfg_sub_t *s = &t->sub[group][i];

		if ((s->fn == fn) && (s->ctx == ctx)) {
			return 0; /* idempotent re-registration */
		}
		if (s->fn == NULL) {
			s->ctx = ctx;
			s->fn = fn;
			return 0;
		}
	}

	return -ENOSPC;
}

/** Number of subscribers registered for @p group. */
static inline size_t sts_cfg_applier_count(const sts_cfg_applier_tbl_t *t,
					   uint8_t group)
{
	size_t i;

	if ((t == NULL) || (group >= STS_CFG_GROUP_MAX)) {
		return 0U;
	}

	for (i = 0U; i < STS_CFG_GROUP_SUBS_MAX; i++) {
		if (t->sub[group][i].fn == NULL) {
			break;
		}
	}
	return i;
}

/**
 * Call every subscriber for @p group, in registration order.
 *
 * Out-of-range groups are ignored rather than rejected: sts_cfg_commit() walks
 * the group byte of whatever keys were staged, and an unclaimed group is normal.
 */
static inline void sts_cfg_applier_dispatch(const sts_cfg_applier_tbl_t *t,
					    uint8_t group)
{
	size_t i;

	if ((t == NULL) || (group >= STS_CFG_GROUP_MAX)) {
		return;
	}

	for (i = 0U; i < STS_CFG_GROUP_SUBS_MAX; i++) {
		sts_cfg_apply_fn fn = t->sub[group][i].fn;

		if (fn == NULL) {
			break;
		}
		fn(t->sub[group][i].ctx, group);
	}
}

/**
 * True when a cfg_commit() result means the LIVE TREE CHANGED and the appliers
 * therefore have to run.
 *
 * -EIO is "applied to the live tree, but N keys did not reach the store"
 * (cfg.h). Treating it as a plain failure and returning early left the running
 * system holding a new log level, IP address or PTP domain that nothing had been
 * told about — the same silently-ineffective commit as never dispatching at all,
 * reached by a different route. Only a validation or cross-field rejection,
 * where nothing was applied, skips the dispatch.
 */
static inline bool sts_cfg_commit_applied(int rc)
{
	return (rc == 0) || (rc == -EIO);
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_STS_CFG_APPLIER_H_ */
