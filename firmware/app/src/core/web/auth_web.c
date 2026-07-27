/*
 * STS1000 "Meridian" — core/web: management-plane authentication.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See auth_web.h for the contract and for why the credential format is pinned
 * to core/mcp's.
 */

#include "web/auth_web.h"

#include <errno.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/* secret hygiene                                                            */
/* ------------------------------------------------------------------------- */

/**
 * Wipe a buffer that holds key material.
 *
 * Through a volatile pointer, matching core/mp's wipe(): a plain memset() over
 * a buffer that is about to leave scope, or that nothing reads again, is a dead
 * store the compiler is entitled to delete — which is exactly how a password
 * survives in a stack frame. core/ stays platform-neutral, so this cannot be
 * mbedtls_platform_zeroize(); the glue uses that where mbedTLS is in scope.
 */
static void wipe(void *p, size_t n)
{
	volatile uint8_t *q = (volatile uint8_t *)p;

	while (n != 0U) {
		*q = 0U;
		q++;
		n--;
	}
}

/* ------------------------------------------------------------------------- */
/* audit                                                                     */
/* ------------------------------------------------------------------------- */

static void audit(auth_web_ctx_t *c, uint8_t level, uint64_t now_ms,
		  const char *msg)
{
	if (c->log == NULL) {
		return;
	}
	(void)logr_puts(c->log, level, (uint8_t)LOGR_SUB_SEC, now_ms, msg);
}

/* ------------------------------------------------------------------------- */
/* KDF                                                                       */
/* ------------------------------------------------------------------------- */

static int kdf_hmac(void *user, const uint8_t *salt, size_t salt_len,
		    const uint8_t *pw, size_t pw_len, uint8_t *out,
		    size_t out_len)
{
	const port_crypto_t *cr = user;

	if (cr == NULL || cr->hmac_sha256 == NULL || out_len != AUTH_WEB_TAG_LEN) {
		return -ENOTSUP;
	}
	/* Identical to mcp_auth_make_blob(): key = salt, message = password. */
	if (cr->hmac_sha256(cr->ctx, salt, salt_len, pw, pw_len, out) != 0) {
		return -EIO;
	}
	return 0;
}

int auth_kdf_hmac_sha256(auth_kdf_t *out, const port_crypto_t *crypto)
{
	if (out == NULL || crypto == NULL) {
		return -EINVAL;
	}
	if (crypto->hmac_sha256 == NULL) {
		return -ENOTSUP;
	}
	out->id = (uint8_t)AUTH_KDF_HMAC_SHA256;
	out->name = "hmac-sha256";
	out->derive = kdf_hmac;
	out->user = (void *)(uintptr_t)crypto;
	return 0;
}

/* ------------------------------------------------------------------------- */
/* helpers                                                                   */
/* ------------------------------------------------------------------------- */

static int rand_hex(auth_web_ctx_t *c, size_t nbytes, char *out, size_t cap)
{
	uint8_t raw[AUTH_WEB_TOKEN_BYTES];

	if (nbytes > sizeof(raw)) {
		return -EINVAL;
	}
	if (c->crypto == NULL || c->crypto->rand == NULL) {
		return -ENOTSUP;
	}
	if (c->crypto->rand(c->crypto->ctx, raw, nbytes) != 0) {
		return -EIO;
	}
	if (web_hex_encode(raw, nbytes, out, cap) != (nbytes * 2U)) {
		return -ENOSPC;
	}
	return 0;
}

static uint32_t idle_seconds(const auth_web_ctx_t *c)
{
	uint64_t v;

	if (c->cfg != NULL &&
	    cfg_get_u64(c->cfg, (uint16_t)CFG_ID_SEC_SESSION_S, &v) == 0 &&
	    v > 0U) {
		return (uint32_t)v;
	}
	return c->idle_s;
}

bool auth_web_required(const auth_web_ctx_t *c)
{
	bool v;

	if (c == NULL) {
		return true;
	}
	if (c->cfg == NULL) {
		/* No policy store: fail closed. A build with no cfg is a unit
		 * test or a bring-up image, and both are better served by
		 * "authentication required" than by an open management plane. */
		return true;
	}
	if (cfg_get_bool(c->cfg, (uint16_t)CFG_ID_SEC_AUTH_REQUIRED, &v) != 0) {
		return true;
	}
	return v;
}

/* ------------------------------------------------------------------------- */
/* lifecycle                                                                 */
/* ------------------------------------------------------------------------- */

int auth_web_init(auth_web_ctx_t *c, const port_crypto_t *crypto,
		  const auth_kdf_t *kdf, cfg_ctx_t *cfg, logr_t *log)
{
	if (c == NULL || crypto == NULL || kdf == NULL || kdf->derive == NULL) {
		return -EINVAL;
	}
	if (crypto->rand == NULL) {
		return -ENOTSUP;
	}
	memset(c, 0, sizeof(*c));
	c->crypto = crypto;
	c->kdf = kdf;
	c->cfg = cfg;
	c->log = log;
	c->idle_s = AUTH_WEB_IDLE_S_DEFAULT;
	c->absolute_s = AUTH_WEB_ABSOLUTE_S_DEFAULT;
	return 0;
}

int auth_web_set_remote(auth_web_ctx_t *c, auth_web_remote_fn fn, void *user)
{
	if (c == NULL) {
		return -EINVAL;
	}
	c->remote = fn;
	c->remote_user = (fn != NULL) ? user : NULL;
	return 0;
}

int auth_web_user_add(auth_web_ctx_t *c, const char *name, uint8_t role,
		      uint16_t cfg_key)
{
	size_t i;
	size_t nlen;
	int existing;

	if (c == NULL || name == NULL) {
		return -EINVAL;
	}
	nlen = strlen(name);
	if (nlen == 0U || nlen > AUTH_WEB_NAME_MAX) {
		return -EINVAL;
	}
	if (role < (uint8_t)WEB_ROLE_VIEWER || role > (uint8_t)WEB_ROLE_ADMIN) {
		return -EINVAL;
	}

	existing = auth_web_user_find(c, name, nlen);
	if (existing >= 0) {
		c->users[existing].role = role;
		c->users[existing].cfg_key = cfg_key;
		return existing;
	}
	for (i = 0U; i < AUTH_WEB_USERS; i++) {
		if (c->users[i].in_use) {
			continue;
		}
		memset(&c->users[i], 0, sizeof(c->users[i]));
		memcpy(c->users[i].name, name, nlen);
		c->users[i].name[nlen] = '\0';
		c->users[i].role = role;
		c->users[i].cfg_key = cfg_key;
		c->users[i].in_use = true;
		return (int)i;
	}
	return -ENOSPC;
}

int auth_web_user_find(const auth_web_ctx_t *c, const char *name, size_t n)
{
	size_t i;

	if (c == NULL || name == NULL || n == 0U) {
		return -ENOENT;
	}
	for (i = 0U; i < AUTH_WEB_USERS; i++) {
		if (!c->users[i].in_use) {
			continue;
		}
		if (web_span_eq(name, n, c->users[i].name)) {
			return (int)i;
		}
	}
	return -ENOENT;
}

const auth_web_user_t *auth_web_user_at(const auth_web_ctx_t *c, size_t idx)
{
	if (c == NULL || idx >= AUTH_WEB_USERS || !c->users[idx].in_use) {
		return NULL;
	}
	return &c->users[idx];
}

bool auth_web_user_persistent(const auth_web_ctx_t *c, size_t idx)
{
	const auth_web_user_t *u = auth_web_user_at(c, idx);

	return (u != NULL) && (u->cfg_key != 0U) && (c->cfg != NULL);
}

int auth_web_reload(auth_web_ctx_t *c)
{
	size_t i;
	int n = 0;

	if (c == NULL) {
		return -EINVAL;
	}
	for (i = 0U; i < AUTH_WEB_USERS; i++) {
		auth_web_user_t *u = &c->users[i];
		uint8_t buf[AUTH_WEB_BLOB_LEN];
		size_t got = 0U;

		if (!u->in_use) {
			continue;
		}
		if (u->cfg_key == 0U || c->cfg == NULL) {
			if (u->has_blob) {
				n++;
			}
			continue;
		}
		if (cfg_get_bytes(c->cfg, u->cfg_key, buf, sizeof(buf), &got) != 0 ||
		    got != AUTH_WEB_BLOB_LEN) {
			/* Unprovisioned or damaged: fail closed rather than
			 * keeping a stale in-RAM credential that no longer
			 * matches what an operator can see in config. This is
			 * also what erases the credential after a factory reset
			 * — the reset empties the key, the CFG_G_SEC applier
			 * calls this, and the RAM copy goes with it. */
			u->has_blob = false;
			wipe(u->blob, sizeof(u->blob));
			wipe(buf, sizeof(buf));
			continue;
		}
		memcpy(u->blob, buf, AUTH_WEB_BLOB_LEN);
		wipe(buf, sizeof(buf));
		u->has_blob = true;
		n++;
	}
	return n;
}

int auth_web_make_blob(const auth_web_ctx_t *c, const uint8_t *salt,
		       const uint8_t *pw, size_t pw_len, uint8_t *out,
		       size_t cap)
{
	uint8_t s[AUTH_WEB_SALT_LEN];
	int rc;

	if (c == NULL || pw == NULL || out == NULL) {
		return -EINVAL;
	}
	if (cap < AUTH_WEB_BLOB_LEN) {
		return -ENOSPC;
	}
	if (pw_len == 0U || pw_len > AUTH_WEB_PW_MAX) {
		return -EINVAL;
	}
	if (c->kdf == NULL || c->kdf->derive == NULL) {
		return -ENOTSUP;
	}
	if (salt != NULL) {
		memcpy(s, salt, sizeof(s));
	} else {
		if (c->crypto == NULL || c->crypto->rand == NULL) {
			return -ENOTSUP;
		}
		if (c->crypto->rand(c->crypto->ctx, s, sizeof(s)) != 0) {
			return -EIO;
		}
	}
	memcpy(out, s, AUTH_WEB_SALT_LEN);
	rc = c->kdf->derive(c->kdf->user, s, sizeof(s), pw, pw_len,
			    &out[AUTH_WEB_SALT_LEN], AUTH_WEB_TAG_LEN);
	if (rc != 0) {
		wipe(out, AUTH_WEB_BLOB_LEN);
		return rc;
	}
	return 0;
}

int auth_web_set_password(auth_web_ctx_t *c, size_t idx, const uint8_t *pw,
			  size_t pw_len)
{
	auth_web_user_t *u;
	uint8_t blob[AUTH_WEB_BLOB_LEN];
	int rc;

	if (c == NULL || pw == NULL) {
		return -EINVAL;
	}
	if (idx >= AUTH_WEB_USERS || !c->users[idx].in_use) {
		return -ENOENT;
	}
	if (pw_len < AUTH_WEB_PW_MIN || pw_len > AUTH_WEB_PW_MAX) {
		return -EINVAL;
	}
	u = &c->users[idx];

	rc = auth_web_make_blob(c, NULL, pw, pw_len, blob, sizeof(blob));
	if (rc != 0) {
		return (rc == -ENOTSUP) ? rc : -EIO;
	}

	if (u->cfg_key != 0U && c->cfg != NULL) {
		rc = cfg_set_bytes(c->cfg, u->cfg_key, blob, sizeof(blob));
		if (rc != 0) {
			wipe(blob, sizeof(blob));
			return -EIO;
		}
	}
	memcpy(u->blob, blob, sizeof(blob));
	u->has_blob = true;
	/* A password change clears the throttle: the operator has just proved
	 * (through an authenticated session) that they control the account. */
	u->fails = 0U;
	u->lock_until_ms = 0U;
	wipe(blob, sizeof(blob));
	return 0;
}

/* ------------------------------------------------------------------------- */
/* throttle                                                                  */
/* ------------------------------------------------------------------------- */

static uint32_t backoff_ms(uint32_t fails)
{
	uint32_t ms = AUTH_WEB_THROTTLE_MS;
	uint32_t over;

	if (fails >= AUTH_WEB_LOCK_TRIES) {
		return AUTH_WEB_LOCKOUT_MS;
	}
	if (fails <= AUTH_WEB_FREE_TRIES) {
		return 0U;
	}
	for (over = AUTH_WEB_FREE_TRIES + 1U; over < fails; over++) {
		if (ms >= (AUTH_WEB_THROTTLE_MAX_MS / 2U)) {
			return AUTH_WEB_THROTTLE_MAX_MS;
		}
		ms *= 2U;
	}
	return ms;
}

uint32_t auth_web_retry_after_ms(const auth_web_ctx_t *c, size_t idx,
				 uint64_t now_ms)
{
	const auth_web_user_t *u = auth_web_user_at(c, idx);

	if (u == NULL || u->lock_until_ms <= now_ms) {
		return 0U;
	}
	return (uint32_t)(u->lock_until_ms - now_ms);
}

/** Arm the escalating backoff after a refused attempt on a KNOWN account. */
static void penalise(auth_web_ctx_t *c, size_t idx, uint64_t now_ms)
{
	auth_web_user_t *u = &c->users[idx];
	uint32_t back;

	u->fails++;
	back = backoff_ms(u->fails);
	if (back != 0U) {
		u->lock_until_ms = now_ms + (uint64_t)back;
	}
}

/** True when at least one account on this box holds a credential. */
static bool any_credential(const auth_web_ctx_t *c)
{
	size_t i;

	for (i = 0U; i < AUTH_WEB_USERS; i++) {
		if (c->users[i].in_use && c->users[i].has_blob) {
			return true;
		}
	}
	return false;
}

/* ------------------------------------------------------------------------- */
/* sessions                                                                  */
/* ------------------------------------------------------------------------- */

static void close_sess(auth_web_ctx_t *c, size_t i)
{
	/* The session and CSRF tokens are bearer secrets: wiped, not merely
	 * marked free, so a closed slot cannot be read out of RAM later. */
	wipe(&c->sess[i], sizeof(c->sess[i]));
}

static bool sess_expired(const auth_web_ctx_t *c, const auth_web_sess_t *s,
			 uint64_t now_ms, bool *absolute)
{
	uint64_t idle_ms = (uint64_t)idle_seconds(c) * 1000ULL;
	uint64_t abs_ms = (uint64_t)c->absolute_s * 1000ULL;

	*absolute = false;
	if (abs_ms != 0U && (now_ms - s->created_ms) >= abs_ms) {
		*absolute = true;
		return true;
	}
	if (idle_ms != 0U && (now_ms - s->last_ms) >= idle_ms) {
		return true;
	}
	return false;
}

/**
 * Verify the stored credential of account @p idx.
 *
 * @retval 0        Match.
 * @retval -EACCES  Mismatch, or the account holds no credential. The two are
 *                  one answer here so the caller cannot accidentally turn
 *                  "unprovisioned" into a distinguishable reply.
 * @retval -EIO     The KDF failed. A fault, never a verdict.
 */
static int local_verify(auth_web_ctx_t *c, size_t idx, const uint8_t *pw,
			size_t pw_len)
{
	uint8_t tag[AUTH_WEB_TAG_LEN];
	auth_web_user_t *u = &c->users[idx];
	bool ok;
	int rc;

	if (!u->has_blob) {
		return -EACCES;
	}
	rc = c->kdf->derive(c->kdf->user, u->blob, AUTH_WEB_SALT_LEN, pw, pw_len,
			    tag, sizeof(tag));
	if (rc != 0) {
		wipe(tag, sizeof(tag));
		return -EIO;
	}
	ok = web_ct_memcmp(tag, &u->blob[AUTH_WEB_SALT_LEN],
			   AUTH_WEB_TAG_LEN) == 0;
	wipe(tag, sizeof(tag));
	return ok ? 0 : -EACCES;
}

/**
 * Refer a credential the local table could not accept to the remote authority.
 *
 * @retval 0        Accepted; @p out_role holds a clamped web_role_t.
 * @retval -EBUSY   The authority holds this principal in a lockout.
 * @retval -EACCES  Refused. EVERY other code the hook can return lands here —
 *                  including -EHOSTUNREACH, "no authority could answer", which
 *                  is a denial and never an allow-on-failure — so the reply
 *                  carries nothing but "no".
 */
static int remote_consult(auth_web_ctx_t *c, const char *user, size_t user_len,
			  const uint8_t *pw, size_t pw_len, uint8_t *out_role)
{
	char name[AUTH_WEB_NAME_MAX + 1U];
	char secret[AUTH_WEB_PW_MAX + 1U];
	uint8_t role = (uint8_t)WEB_ROLE_NONE;
	int rc;

	/*
	 * The hook speaks C strings, and so does sts_aaa_check() beneath it. A
	 * password carrying an embedded NUL would therefore be truncated at it,
	 * and "pw\0anything" would authenticate as "pw". Refuse rather than
	 * truncate. (A name cannot contain one — auth_web_user_find() matched it
	 * against a C string — but an UNKNOWN name reaches here unmatched, so it
	 * is checked too.)
	 */
	if (memchr(pw, 0, pw_len) != NULL ||
	    memchr(user, 0, user_len) != NULL) {
		c->st.logins_remote_denied++;
		return -EACCES;
	}
	memcpy(name, user, user_len);
	name[user_len] = '\0';
	memcpy(secret, pw, pw_len);
	secret[pw_len] = '\0';

	c->st.remote_consults++;
	rc = c->remote(c->remote_user, name, secret, &role);
	wipe(secret, sizeof(secret));

	if (rc != 0) {
		c->st.logins_remote_denied++;
		return (rc == -EBUSY) ? -EBUSY : -EACCES;
	}
	/*
	 * An authority that accepts without naming a usable role gets the LEAST
	 * privilege, not the most. core/auth already applies this clamp; it is
	 * repeated here because this hook is a public extension point and may be
	 * wired to something that does not.
	 */
	if (role == (uint8_t)WEB_ROLE_NONE || role > (uint8_t)WEB_ROLE_ADMIN) {
		role = (uint8_t)WEB_ROLE_VIEWER;
	}
	*out_role = role;
	c->st.logins_remote_ok++;
	return 0;
}

int auth_web_login(auth_web_ctx_t *c, const char *user, size_t user_len,
		   const uint8_t *pw, size_t pw_len, uint64_t now_ms,
		   auth_web_grant_t *out)
{
	auth_web_user_t *u = NULL;
	uint8_t role = (uint8_t)WEB_ROLE_NONE;
	bool by_remote = false;
	int verdict = -EACCES;
	int uidx;
	size_t slot;
	size_t i;
	int rc;

	if (c == NULL || user == NULL || pw == NULL || out == NULL) {
		return -EINVAL;
	}
	if (user_len == 0U || user_len > AUTH_WEB_NAME_MAX) {
		c->st.logins_bad_user++;
		return -EACCES;
	}
	if (pw_len == 0U || pw_len > AUTH_WEB_PW_MAX) {
		c->st.logins_bad_pw++;
		return -EACCES;
	}

	uidx = auth_web_user_find(c, user, user_len);
	if (uidx >= 0) {
		u = &c->users[uidx];

		if (u->lock_until_ms > now_ms) {
			/*
			 * Throttled: neither the password nor the network is
			 * touched, so the answer says nothing about either — and
			 * a brute-force attempt costs the remote authority no
			 * round trips.
			 */
			c->st.logins_throttled++;
			audit(c, (uint8_t)LOGR_WARN, now_ms,
			      "web auth: throttled, password not tested");
			return -EBUSY;
		}

		rc = local_verify(c, (size_t)uidx, pw, pw_len);
		if (rc == -EIO) {
			/* A broken KDF is a fault, not a verdict: it must not
			 * be laundered into a remote lookup. */
			return -EIO;
		}
		if (rc == 0) {
			role = u->role;
			verdict = 0;
		}
	} else {
		/*
		 * An unknown user is reported exactly like a wrong password so
		 * the response does not enumerate accounts. It is deliberately
		 * NOT throttled per-name (there is no counter to attach it to);
		 * the glue rate-limits the route itself and the remote
		 * authority keeps its own lockout table.
		 */
		c->st.logins_bad_user++;
	}

	/*
	 * The remote authority sees only what the local table could not accept:
	 * an unknown account, an account with no credential, or a wrong
	 * password. A local SUCCESS never leaves the box.
	 */
	if (verdict != 0 && c->remote != NULL) {
		verdict = remote_consult(c, user, user_len, pw, pw_len, &role);
		by_remote = (verdict == 0);
	}

	if (verdict != 0) {
		if (u != NULL) {
			c->st.logins_bad_pw++;
			if (verdict != -EBUSY) {
				penalise(c, (size_t)uidx, now_ms);
			}
		}
		if (verdict == -EBUSY) {
			c->st.logins_throttled++;
			audit(c, (uint8_t)LOGR_WARN, now_ms,
			      "web auth: refused, authority holds a lockout");
			return -EBUSY;
		}
		/*
		 * -ENOENT survives for exactly one situation, and it is a
		 * GLOBAL property of the box, never a per-account one: a login
		 * for a known account on a unit where NO account holds a
		 * credential. That is a virgin unit, and rest.c turns this code
		 * into the 503 that tells the operator to commission it over
		 * the local UI or the USB console. Removing it would leave a
		 * fresh box answering 401 with no hint at all.
		 *
		 * Two deliberate narrowings, both closing an enumeration hole:
		 *
		 *   - it needs a KNOWN account name, so an attacker cannot
		 *     probe with arbitrary names and read the answer;
		 *   - it needs the box to hold NO credential at all. Once any
		 *     account is provisioned, an unprovisioned one answers
		 *     -EACCES like every other refusal. Reporting per-account
		 *     provisioning state to an unauthenticated peer is exactly
		 *     the reconnaissance ("admin is set up, operator is not —
		 *     attack operator") that core/mcp's h_auth already refuses
		 *     to give out, and three accounts with fixed, public names
		 *     make it worth something.
		 *
		 * Deliberately NOT conditioned on whether a remote authority is
		 * wired: the production glue always wires one, so testing that
		 * would silently delete the bootstrap message from every
		 * shipped image. A configured, reachable authority has already
		 * had its say by this point — the chain runs first, above.
		 */
		if (uidx >= 0 && !any_credential(c)) {
			audit(c, (uint8_t)LOGR_WARN, now_ms,
			      "web auth: no credential provisioned on this box");
			/* -ENOENT, not -ENOKEY: picolibc has no ENOKEY. */
			return -ENOENT;
		}
		audit(c, (uint8_t)LOGR_WARN, now_ms, "web auth: refused");
		return -EACCES;
	}

	/* Reap anything stale before hunting for a slot. */
	(void)auth_web_tick(c, now_ms);

	slot = AUTH_WEB_SESSIONS;
	for (i = 0U; i < AUTH_WEB_SESSIONS; i++) {
		if (!c->sess[i].active) {
			slot = i;
			break;
		}
	}
	if (slot == AUTH_WEB_SESSIONS) {
		/*
		 * Evict the least-recently-used session rather than refusing the
		 * login: an operator locked out of their own box because four
		 * abandoned browser tabs hold every slot is a worse failure than
		 * dropping the oldest of them.
		 */
		uint64_t oldest = UINT64_MAX;

		slot = 0U;
		for (i = 0U; i < AUTH_WEB_SESSIONS; i++) {
			if (c->sess[i].last_ms <= oldest) {
				oldest = c->sess[i].last_ms;
				slot = i;
			}
		}
		c->st.logins_no_slot++;
		close_sess(c, slot);
	}

	rc = rand_hex(c, AUTH_WEB_TOKEN_BYTES, c->sess[slot].token,
		      sizeof(c->sess[slot].token));
	if (rc != 0) {
		close_sess(c, slot);
		return -EIO;
	}
	rc = rand_hex(c, AUTH_WEB_CSRF_BYTES, c->sess[slot].csrf,
		      sizeof(c->sess[slot].csrf));
	if (rc != 0) {
		close_sess(c, slot);
		return -EIO;
	}

	/*
	 * A remote principal has no entry in the local table, so the slot carries
	 * AUTH_WEB_NO_USER — deliberately out of range, so auth_web_user_at()
	 * answers NULL for it and every existing caller that maps a session to an
	 * account already fails safe. The name is kept in the session either way,
	 * which is what audit and telemetry actually need.
	 */
	c->sess[slot].user = (uidx >= 0) ? (uint8_t)uidx : AUTH_WEB_NO_USER;
	c->sess[slot].role = role;
	c->sess[slot].remote = by_remote;
	memcpy(c->sess[slot].name, user, user_len);
	c->sess[slot].name[user_len] = '\0';
	c->sess[slot].created_ms = now_ms;
	c->sess[slot].last_ms = now_ms;
	c->sess[slot].requests = 0U;
	c->sess[slot].active = true;

	if (u != NULL) {
		/* Whoever authenticated proved control of this account, so the
		 * throttle clears — including when a remote authority is the one
		 * that vouched for them. */
		u->fails = 0U;
		u->lock_until_ms = 0U;
	}
	c->st.logins_ok++;
	audit(c, (uint8_t)LOGR_NOTICE, now_ms,
	      by_remote ? "web auth: session opened (remote authority)"
			: "web auth: session opened");

	out->token = c->sess[slot].token;
	out->csrf = c->sess[slot].csrf;
	out->role = role;
	out->idle_s = idle_seconds(c);
	out->absolute_s = c->absolute_s;
	return 0;
}

int auth_web_validate(auth_web_ctx_t *c, const char *token, uint64_t now_ms,
		      size_t *out_sess)
{
	size_t i;
	size_t match = AUTH_WEB_SESSIONS;

	if (c == NULL || token == NULL) {
		return -EINVAL;
	}
	if (token[0] == '\0') {
		c->st.token_rejected++;
		return -EACCES;
	}
	/*
	 * Every slot is compared, and always with the constant-time comparator,
	 * so the loop's cost does not depend on which slot matched or on how
	 * much of a token was right.
	 */
	for (i = 0U; i < AUTH_WEB_SESSIONS; i++) {
		if (!c->sess[i].active) {
			continue;
		}
		if (web_ct_streq(token, c->sess[i].token,
				 AUTH_WEB_TOKEN_LEN + 1U) == 0) {
			match = i;
		}
	}
	if (match == AUTH_WEB_SESSIONS) {
		c->st.token_rejected++;
		return -EACCES;
	}
	{
		bool absolute = false;

		if (sess_expired(c, &c->sess[match], now_ms, &absolute)) {
			if (absolute) {
				c->st.expired_absolute++;
			} else {
				c->st.expired_idle++;
			}
			close_sess(c, match);
			audit(c, (uint8_t)LOGR_INFO, now_ms,
			      "web auth: session expired");
			return -ETIME;
		}
	}
	c->sess[match].last_ms = now_ms;
	c->sess[match].requests++;
	if (out_sess != NULL) {
		*out_sess = match;
	}
	return 0;
}

const auth_web_sess_t *auth_web_sess_at(const auth_web_ctx_t *c, size_t idx)
{
	if (c == NULL || idx >= AUTH_WEB_SESSIONS || !c->sess[idx].active) {
		return NULL;
	}
	return &c->sess[idx];
}


int auth_web_check_csrf(auth_web_ctx_t *c, size_t idx, const char *token,
			size_t token_len)
{
	char given[AUTH_WEB_CSRF_LEN + 1U];

	if (c == NULL || idx >= AUTH_WEB_SESSIONS) {
		return -EINVAL;
	}
	if (!c->sess[idx].active) {
		return -EINVAL;
	}
	if (token == NULL || token_len != AUTH_WEB_CSRF_LEN) {
		/* A missing or wrong-length token is refused without touching
		 * the comparator, which cannot leak anything it never saw. */
		c->st.csrf_rejected++;
		return -EACCES;
	}
	memcpy(given, token, AUTH_WEB_CSRF_LEN);
	given[AUTH_WEB_CSRF_LEN] = '\0';
	if (web_ct_memcmp(given, c->sess[idx].csrf, AUTH_WEB_CSRF_LEN) != 0) {
		c->st.csrf_rejected++;
		return -EACCES;
	}
	return 0;
}

int auth_web_logout(auth_web_ctx_t *c, size_t idx)
{
	if (c == NULL || idx >= AUTH_WEB_SESSIONS) {
		return -EINVAL;
	}
	if (!c->sess[idx].active) {
		return -ENOENT;
	}
	close_sess(c, idx);
	c->st.logouts++;
	return 0;
}

void auth_web_logout_all(auth_web_ctx_t *c)
{
	size_t i;

	if (c == NULL) {
		return;
	}
	for (i = 0U; i < AUTH_WEB_SESSIONS; i++) {
		if (c->sess[i].active) {
			close_sess(c, i);
			c->st.logouts++;
		}
	}
}

void auth_web_wipe(auth_web_ctx_t *c)
{
	size_t i;

	if (c == NULL) {
		return;
	}
	auth_web_logout_all(c);
	/* logout_all only walks live slots; a slot closed earlier is already
	 * wiped, but sweeping all of them keeps this a single, checkable claim:
	 * after auth_web_wipe() no session bytes remain anywhere. */
	for (i = 0U; i < AUTH_WEB_SESSIONS; i++) {
		wipe(&c->sess[i], sizeof(c->sess[i]));
	}
	for (i = 0U; i < AUTH_WEB_USERS; i++) {
		auth_web_user_t *u = &c->users[i];

		wipe(u->blob, sizeof(u->blob));
		u->has_blob = false;
		/* The lockout goes too: the credential it was protecting no
		 * longer exists, so keeping the window would only lock an
		 * operator out of a box that has just been handed back to
		 * them. The account name, role and cfg binding survive — the
		 * table must still be the one the box boots with. */
		u->fails = 0U;
		u->lock_until_ms = 0U;
	}
}

int auth_web_tick(auth_web_ctx_t *c, uint64_t now_ms)
{
	size_t i;
	int n = 0;

	if (c == NULL) {
		return -EINVAL;
	}
	for (i = 0U; i < AUTH_WEB_SESSIONS; i++) {
		bool absolute = false;

		if (!c->sess[i].active) {
			continue;
		}
		if (!sess_expired(c, &c->sess[i], now_ms, &absolute)) {
			continue;
		}
		if (absolute) {
			c->st.expired_absolute++;
		} else {
			c->st.expired_idle++;
		}
		close_sess(c, i);
		n++;
	}
	return n;
}

uint32_t auth_web_session_count(const auth_web_ctx_t *c)
{
	uint32_t n = 0U;
	size_t i;

	if (c == NULL) {
		return 0U;
	}
	for (i = 0U; i < AUTH_WEB_SESSIONS; i++) {
		if (c->sess[i].active) {
			n++;
		}
	}
	return n;
}

const auth_web_stats_t *auth_web_stats(const auth_web_ctx_t *c)
{
	if (c == NULL) {
		return NULL;
	}
	return &c->st;
}
