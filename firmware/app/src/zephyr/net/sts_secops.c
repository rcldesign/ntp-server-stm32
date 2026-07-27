/*
 * STS1000 "Meridian" — the AAA offload worker.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * One thread whose whole job is to be the one allowed to block. See
 * sts_secops.h for why a management plane must not call sts_aaa_check()
 * directly from a thread that feeds the watchdog.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <mbedtls/platform_util.h>

#include "auth/auth.h"
#include "net/sts_aaa.h"
#include "net/sts_secops.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_secops, CONFIG_STS1000_LOG_LEVEL);

/*
 * Stack budget. The deepest path is do_ldap(): a sockaddr_storage, a
 * DN_MAX+AUTH_USER_MAX+2 = 129-byte user DN, a 65-byte bind password and the
 * socket calls under them. The RADIUS/TACACS+ packet buffers are file-scope
 * statics in sts_aaa.c, not stack. 3 KiB leaves better than 2x margin over the
 * measured worst case and is the only RAM this facility adds.
 *
 * NOT a Kconfig symbol: app/Kconfig belongs to another area this change does
 * not touch. Override at the compiler if a build ever needs to.
 */
#ifndef STS_AAA_WORKER_STACK
#define STS_AAA_WORKER_STACK 3072
#endif

/*
 * Priority 14 — the console/housekeeping band (ARCHITECTURE.md §6). Below the
 * web workers (12) so a login lookup never preempts a request that is already
 * being served, and never anywhere near the timing path.
 */
#ifndef STS_AAA_WORKER_PRIO
#define STS_AAA_WORKER_PRIO 14
#endif

/*
 * How long the caller sleeps between liveness feeds while it waits. Must be
 * comfortably under CONFIG_STS1000_LIVENESS_DEADLINE_MS (5 000) with room for
 * the caller to be descheduled by higher-priority work; 250 ms gives a 20x
 * margin and costs four wakeups a second during a lookup that is, by
 * definition, already slow.
 */
#define FEED_SLICE_MS 250

BUILD_ASSERT(FEED_SLICE_MS * 4 < CONFIG_STS1000_LIVENESS_DEADLINE_MS,
	     "the liveness feed slice must leave the watchdog wide margin");

/* ------------------------------------------------------------------ state */

/* The request in flight. Owned by whoever holds g_gate. */
static struct {
	char    user[AUTH_USER_MAX + 1U];
	char    secret[AUTH_SECRET_MAX + 1U];
	uint8_t role;
	int     rc;
} g_req;

/* Serialises submitters: one lookup at a time, board-wide. */
static K_MUTEX_DEFINE(g_gate);
/* Handed to the worker; returned when the answer is in g_req. */
static K_SEM_DEFINE(g_start, 0, 1);
static K_SEM_DEFINE(g_done, 0, 1);

static void aaa_worker(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		(void)k_sem_take(&g_start, K_FOREVER);

		g_req.role = (uint8_t)AUTH_ROLE_NONE;
		g_req.rc = sts_aaa_check(g_req.user, g_req.secret, &g_req.role);

		/* The credential dies with the lookup, not with the next one. */
		mbedtls_platform_zeroize(g_req.secret, sizeof(g_req.secret));

		k_sem_give(&g_done);
	}
}

K_THREAD_DEFINE(sts_aaa_worker_tid, STS_AAA_WORKER_STACK, aaa_worker, NULL,
		NULL, NULL, STS_AAA_WORKER_PRIO, 0, 0);

/* --------------------------------------------------------------- helpers */

/** k_mutex_lock(), but stay visibly alive while waiting. */
static void lock_fed(int live_id)
{
	while (k_mutex_lock(&g_gate, K_MSEC(FEED_SLICE_MS)) != 0) {
		if (live_id >= 0) {
			sts_liveness_feed(live_id);
		}
	}
}

/** k_sem_take(K_FOREVER), but stay visibly alive while waiting. */
static void wait_fed(struct k_sem *sem, int live_id)
{
	while (k_sem_take(sem, K_MSEC(FEED_SLICE_MS)) != 0) {
		if (live_id >= 0) {
			sts_liveness_feed(live_id);
		}
	}
}

/* ------------------------------------------------------------------ API */

int sts_aaa_check_fed(const char *user, const char *secret, uint8_t *out_role,
		      int live_id)
{
	size_t ulen;
	size_t slen;
	int rc;

	if (out_role != NULL) {
		*out_role = (uint8_t)AUTH_ROLE_NONE;
	}
	if (user == NULL || secret == NULL || out_role == NULL) {
		return -EINVAL;
	}
	ulen = strlen(user);
	slen = strlen(secret);
	if (ulen == 0U || ulen > AUTH_USER_MAX || slen == 0U ||
	    slen > AUTH_SECRET_MAX) {
		/* Rejected here rather than truncated into the worker's buffer:
		 * a silently shortened secret would authenticate as something
		 * the caller never sent. */
		return -EINVAL;
	}

	lock_fed(live_id);

	memcpy(g_req.user, user, ulen + 1U);
	memcpy(g_req.secret, secret, slen + 1U);

	k_sem_give(&g_start);
	/*
	 * No timeout, and deliberately so. Bounding the wait does not help: no
	 * bound both fits the watchdog deadline and lets a real RADIUS server
	 * answer, so a cap would either be useless or would deny legitimate
	 * logins on a slow but working chain. The wait is bounded by the
	 * operator's own configured per-backend timeouts, which is the right
	 * authority for it — and the feeding loop is what makes an unbounded
	 * wait safe.
	 */
	wait_fed(&g_done, live_id);

	rc = g_req.rc;
	*out_role = (rc == 0) ? g_req.role : (uint8_t)AUTH_ROLE_NONE;

	mbedtls_platform_zeroize(g_req.secret, sizeof(g_req.secret));
	mbedtls_platform_zeroize(g_req.user, sizeof(g_req.user));
	g_req.role = (uint8_t)AUTH_ROLE_NONE;

	k_mutex_unlock(&g_gate);
	return rc;
}
