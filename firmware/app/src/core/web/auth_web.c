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
			 * matches what an operator can see in config. */
			u->has_blob = false;
			memset(u->blob, 0, sizeof(u->blob));
			continue;
		}
		memcpy(u->blob, buf, AUTH_WEB_BLOB_LEN);
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
		memset(out, 0, AUTH_WEB_BLOB_LEN);
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
			memset(blob, 0, sizeof(blob));
			return -EIO;
		}
	}
	memcpy(u->blob, blob, sizeof(blob));
	u->has_blob = true;
	/* A password change clears the throttle: the operator has just proved
	 * (through an authenticated session) that they control the account. */
	u->fails = 0U;
	u->lock_until_ms = 0U;
	memset(blob, 0, sizeof(blob));
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

/* ------------------------------------------------------------------------- */
/* sessions                                                                  */
/* ------------------------------------------------------------------------- */

static void close_sess(auth_web_ctx_t *c, size_t i)
{
	memset(&c->sess[i], 0, sizeof(c->sess[i]));
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

int auth_web_login(auth_web_ctx_t *c, const char *user, size_t user_len,
		   const uint8_t *pw, size_t pw_len, uint64_t now_ms,
		   auth_web_grant_t *out)
{
	uint8_t tag[AUTH_WEB_TAG_LEN];
	auth_web_user_t *u;
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
	if (uidx < 0) {
		/*
		 * An unknown user is reported exactly like a wrong password so
		 * the response does not enumerate accounts. It is deliberately
		 * NOT throttled per-name (there is no counter to attach it to);
		 * the glue rate-limits the route itself.
		 */
		c->st.logins_bad_user++;
		audit(c, (uint8_t)LOGR_WARN, now_ms, "web auth: unknown user");
		return -EACCES;
	}
	u = &c->users[uidx];

	if (u->lock_until_ms > now_ms) {
		c->st.logins_throttled++;
		audit(c, (uint8_t)LOGR_WARN, now_ms,
		      "web auth: throttled, password not tested");
		return -EBUSY;
	}
	if (!u->has_blob) {
		audit(c, (uint8_t)LOGR_WARN, now_ms,
		      "web auth: account has no credential");
		/* -ENOENT, not -ENOKEY: picolibc has no ENOKEY. */
		return -ENOENT;
	}

	rc = c->kdf->derive(c->kdf->user, u->blob, AUTH_WEB_SALT_LEN, pw, pw_len,
			    tag, sizeof(tag));
	if (rc != 0) {
		return -EIO;
	}
	if (web_ct_memcmp(tag, &u->blob[AUTH_WEB_SALT_LEN], AUTH_WEB_TAG_LEN) != 0) {
		uint32_t back;

		memset(tag, 0, sizeof(tag));
		u->fails++;
		back = backoff_ms(u->fails);
		if (back != 0U) {
			u->lock_until_ms = now_ms + (uint64_t)back;
		}
		c->st.logins_bad_pw++;
		audit(c, (uint8_t)LOGR_WARN, now_ms, "web auth: bad password");
		return -EACCES;
	}
	memset(tag, 0, sizeof(tag));

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

	c->sess[slot].user = (uint8_t)uidx;
	c->sess[slot].role = u->role;
	c->sess[slot].created_ms = now_ms;
	c->sess[slot].last_ms = now_ms;
	c->sess[slot].requests = 0U;
	c->sess[slot].active = true;

	u->fails = 0U;
	u->lock_until_ms = 0U;
	c->st.logins_ok++;
	audit(c, (uint8_t)LOGR_NOTICE, now_ms, "web auth: session opened");

	out->token = c->sess[slot].token;
	out->csrf = c->sess[slot].csrf;
	out->role = u->role;
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
