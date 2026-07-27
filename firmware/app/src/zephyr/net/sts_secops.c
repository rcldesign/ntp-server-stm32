/*
 * STS1000 "Meridian" — the AAA offload worker.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * One thread whose whole job is to be the one allowed to block. See
 * sts_secops.h for why a management plane must not call sts_aaa_check()
 * directly from a thread that feeds the watchdog, and sts_secops_policy.h for
 * the budgets, the gate hand-off and the trade they record.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <mbedtls/platform_util.h>

#include "auth/auth.h"
#include "net/sts_aaa.h"
#include "net/sts_secops.h"
#include "net/sts_secops_policy.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_secops, CONFIG_STS1000_LOG_LEVEL);

/*
 * ---------------------------------------------------------------------------
 * Stack budget, and the configuration that decides it
 * ---------------------------------------------------------------------------
 *
 * WITHOUT TLS SOCKETS the deepest path is do_ldap() over plain TCP: a
 * sockaddr_storage, a DN_MAX+AUTH_USER_MAX+2 = 129-byte user DN, a 65-byte bind
 * password and the socket calls under them. The RADIUS/TACACS+ packet buffers
 * are file-scope statics in sts_aaa.c, not stack. 3 KiB leaves better than 2x
 * margin on that path.
 *
 * WITH TLS SOCKETS it is not close. Zephyr's TLS is a socket *type*, and
 * sockets_tls.c runs mbedtls_ssl_handshake() INLINE ON THE CALLING THREAD — so
 * `sec.ldap.mode = 2` (LDAPS) puts an ECDHE key exchange and an X.509 chain
 * verification on this stack, several KiB of it, reached from an
 * unauthenticated login attempt.
 *
 * And that is the shipped configuration. app/conf/security.conf still describes
 * LDAPS as "one uncomment from live" and CONFIG_NET_SOCKETS_SOCKOPT_TLS as off;
 * that comment is stale. app/conf/web.conf sets the same symbol =y for HTTPS,
 * Kconfig symbols are global, and sts_aaa.c's `#if defined(...)` blocks compile
 * on it — so mode 2 is reachable at runtime today, on a 3 KiB stack, and the
 * failure would be a stack overflow inside the offload worker. That is
 * precisely the dead-worker case the bounded wait below exists to survive, and
 * this facility must not be the thing that manufactures it.
 *
 * 8 KiB is not a measurement. It is this tree's own precedent for the same
 * handshake, the same mbedTLS configuration and the same board: sts_ntske.c
 * sizes NTSKE_STACK_SIZE at 8192 for an mbedTLS TLS 1.3 handshake. Treat it as
 * the floor that keeps the overflow off the table until somebody measures it
 * (CONFIG_THREAD_ANALYZER against a real LDAPS bind), not as the answer.
 *
 * NOT a Kconfig symbol: app/Kconfig belongs to another area this change does
 * not touch. Override at the compiler if a build ever needs to — the
 * BUILD_ASSERT keeps an override from going the wrong way.
 */
#ifndef STS_AAA_WORKER_STACK
#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
#define STS_AAA_WORKER_STACK 8192
#else
#define STS_AAA_WORKER_STACK 3072
#endif
#endif

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
BUILD_ASSERT(STS_AAA_WORKER_STACK >= 8192,
	     "LDAPS runs the mbedTLS handshake on this thread; raise "
	     "STS_AAA_WORKER_STACK and measure it before shipping TLS AAA");
#endif

/*
 * Priority 14 — the console/housekeeping band (ARCHITECTURE.md §6). Below the
 * web workers (12) so a login lookup never preempts a request that is already
 * being served, and never anywhere near the timing path.
 */
#ifndef STS_AAA_WORKER_PRIO
#define STS_AAA_WORKER_PRIO 14
#endif

BUILD_ASSERT(STS_AAA_FED_SLICE_MS * 4U < CONFIG_STS1000_LIVENESS_DEADLINE_MS,
	     "the liveness feed slice must leave the watchdog wide margin");

/* ------------------------------------------------------------------ state */

/* The request in flight. Owned by whoever holds g_gate, then by the worker. */
static struct {
	char    user[AUTH_USER_MAX + 1U];
	char    secret[AUTH_SECRET_MAX + 1U];
	uint8_t role;
	int     rc;
} g_req;

/*
 * The gate: one lookup at a time, board-wide.
 *
 * A semaphore rather than a mutex, because ownership has to be transferable.
 * When a submitter's budget expires the worker is still inside sts_aaa_check()
 * with g_req live, so the gate cannot be released yet — and the submitter, the
 * only thread that could release a mutex it owns, is leaving. The worker
 * releases it instead (sts_aaa_fed_finish()), which a mutex cannot express.
 *
 * The limit of 1 caps the idle token count, but it is NOT the guarantee: Zephyr
 * hands a k_sem_give() straight to a waiting thread without consulting the
 * limit, so a double-release while somebody waits would still admit two
 * lookups. The guarantee is the state machine in sts_secops_policy.h, which is
 * why it is a tested function and not an inline branch.
 *
 * Nothing is lost by not having priority inheritance here: the gate holder is
 * asleep on a socket, not runnable, so there is no priority to donate.
 */
static K_SEM_DEFINE(g_gate, 1, 1);

/* Handed to the worker; returned when the answer is in g_req. */
static K_SEM_DEFINE(g_start, 0, 1);
static K_SEM_DEFINE(g_done, 0, 1);

/*
 * The hand-off critical section. Short and non-blocking by construction: it
 * covers a zeroize, a policy call and a k_sem_give(). It exists so the worker's
 * "publish" and a submitter's "give up" cannot both believe they won — the two
 * decisions are taken under it, so exactly one of them does.
 */
static K_MUTEX_DEFINE(g_state);
static uint8_t g_slot = (uint8_t)STS_AAA_SLOT_IDLE;

static void aaa_worker(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		sts_aaa_fed_finish_t fin;
		uint8_t role = (uint8_t)AUTH_ROLE_NONE;
		int rc;

		(void)k_sem_take(&g_start, K_FOREVER);

		rc = sts_aaa_check(g_req.user, g_req.secret, &role);

		k_mutex_lock(&g_state, K_FOREVER);

		/* The credential dies with the lookup, not with the next one. */
		mbedtls_platform_zeroize(g_req.secret, sizeof(g_req.secret));

		fin = sts_aaa_fed_finish(g_slot);
		g_slot = fin.next;

		if (fin.discard) {
			/* Nobody is coming for this answer. Leave nothing of
			 * the attempt behind for the next one to read. */
			mbedtls_platform_zeroize(g_req.user, sizeof(g_req.user));
			g_req.role = (uint8_t)AUTH_ROLE_NONE;
			g_req.rc = -EHOSTUNREACH;
		} else {
			g_req.role = role;
			g_req.rc = rc;
		}
		if (fin.publish) {
			k_sem_give(&g_done);
		}
		if (fin.release) {
			k_sem_give(&g_gate);
		}

		k_mutex_unlock(&g_state);

		/* Outside the section on purpose: it is documented as covering
		 * a zeroize, a policy call and two k_sem_give()s, and a log
		 * backend in immediate mode is none of those. */
		if (fin.release) {
			LOG_WRN("aaa: lookup finished after its submitter gave "
				"up; gate released");
		}
	}
}

K_THREAD_DEFINE(sts_aaa_worker_tid, STS_AAA_WORKER_STACK, aaa_worker, NULL,
		NULL, NULL, STS_AAA_WORKER_PRIO, 0, 0);

/* --------------------------------------------------------------- helpers */

/**
 * Has the worker thread terminated?
 *
 * A thread that has died cannot set a flag to say so, so the question has to be
 * put to the kernel: k_thread_join() with K_NO_WAIT returns 0 only for a thread
 * already in the DEAD state, and -EBUSY for one that is still there (including
 * one that has not been scheduled yet). This is what makes sts_secops.h's
 * documented -EHOSTUNREACH for "the worker is not there" a real return rather
 * than a comment — before it, a dead worker meant every submitter looped in the
 * fed wait forever, holding the gate and feeding the very watchdog that exists
 * to recover the board from exactly that.
 */
static bool worker_gone(void)
{
	return k_thread_join(sts_aaa_worker_tid, K_NO_WAIT) == 0;
}

/**
 * Acquire the gate, staying visibly alive, and give up at @p deadline.
 *
 * @retval 0               the gate is held by this thread.
 * @retval -EHOSTUNREACH   the budget ran out, or the current holder has already
 *                         overrun a budget of its own.
 */
static int gate_take_fed(int live_id, int64_t deadline)
{
	for (;;) {
		int32_t slice;
		bool may_wait;

		k_mutex_lock(&g_state, K_FOREVER);
		may_wait = sts_aaa_fed_wait_ok(g_slot);
		k_mutex_unlock(&g_state);
		if (!may_wait) {
			return -EHOSTUNREACH;
		}

		slice = sts_aaa_fed_slice_ms(k_uptime_get(), deadline);
		if (slice <= 0) {
			return -EHOSTUNREACH;
		}
		if (k_sem_take(&g_gate, K_MSEC(slice)) == 0) {
			return 0;
		}
		if (live_id >= 0) {
			sts_liveness_feed(live_id);
		}
	}
}

/**
 * k_sem_take(), staying visibly alive, and giving up at @p deadline.
 *
 * @retval 0        taken.
 * @retval -EAGAIN  the budget ran out first.
 */
static int wait_fed(struct k_sem *sem, int live_id, int64_t deadline)
{
	for (;;) {
		int32_t slice = sts_aaa_fed_slice_ms(k_uptime_get(), deadline);

		if (slice <= 0) {
			return -EAGAIN;
		}
		if (k_sem_take(sem, K_MSEC(slice)) == 0) {
			return 0;
		}
		if (live_id >= 0) {
			sts_liveness_feed(live_id);
		}
	}
}

/* ------------------------------------------------------------------ API */

int sts_aaa_check_fed_budgeted(const char *user, const char *secret,
			       uint8_t *out_role, int live_id,
			       uint32_t budget_ms)
{
	int64_t deadline;
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

	/* Clamped, not rejected: a caller asking for more than the ceiling is
	 * asking for the unbounded wait this facility no longer offers, and 0
	 * would otherwise mean "give up before trying". */
	if (budget_ms == 0U || budget_ms > STS_AAA_FED_CEILING_MS) {
		budget_ms = STS_AAA_FED_CEILING_MS;
	}

	if (worker_gone()) {
		LOG_ERR("aaa: the offload worker is gone; denying every lookup");
		return -EHOSTUNREACH;
	}

	deadline = k_uptime_get() + (int64_t)budget_ms;

	if (gate_take_fed(live_id, deadline) != 0) {
		LOG_WRN("aaa: another lookup still holds the gate after %u ms; "
			"denied", (unsigned int)budget_ms);
		return -EHOSTUNREACH;
	}

	memcpy(g_req.user, user, ulen + 1U);
	memcpy(g_req.secret, secret, slen + 1U);

	k_mutex_lock(&g_state, K_FOREVER);
	g_slot = (uint8_t)STS_AAA_SLOT_RUNNING;
	k_mutex_unlock(&g_state);

	k_sem_give(&g_start);

	if (wait_fed(&g_done, live_id, deadline) != 0) {
		sts_aaa_fed_expiry_t exp;
		bool answered;

		k_mutex_lock(&g_state, K_FOREVER);
		/* One last look, inside the same critical section the worker
		 * publishes from, so "it finished just as I gave up" resolves
		 * one way or the other and never both. */
		answered = (k_sem_take(&g_done, K_NO_WAIT) == 0);
		exp = sts_aaa_fed_expiry(g_slot, answered);
		g_slot = exp.next;
		k_mutex_unlock(&g_state);

		if (!exp.take) {
			if (exp.release) {
				k_sem_give(&g_gate);
			}
			LOG_WRN("aaa: lookup exceeded its %u ms budget; denied",
				(unsigned int)budget_ms);
			return -EHOSTUNREACH;
		}
		/* exp.take: the answer is in g_req after all. Fall through to
		 * the common tail, which is also what exp.release asks for. */
	}

	rc = g_req.rc;
	*out_role = (rc == 0) ? g_req.role : (uint8_t)AUTH_ROLE_NONE;

	mbedtls_platform_zeroize(g_req.secret, sizeof(g_req.secret));
	mbedtls_platform_zeroize(g_req.user, sizeof(g_req.user));
	g_req.role = (uint8_t)AUTH_ROLE_NONE;

	k_sem_give(&g_gate);
	return rc;
}

int sts_aaa_check_fed(const char *user, const char *secret, uint8_t *out_role,
		      int live_id)
{
	return sts_aaa_check_fed_budgeted(user, secret, out_role, live_id,
					  STS_AAA_FED_CEILING_MS);
}
