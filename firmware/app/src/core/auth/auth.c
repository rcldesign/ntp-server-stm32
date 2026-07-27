/*
 * STS1000 "Meridian" — core/auth: the AAA chain, the cache, the lockout, MD5.
 *
 * See auth.h for the contract and for why MD5 is here.
 */

#include "auth/auth.h"

#include <errno.h>
#include <string.h>

/* ========================================================================= */
/* MD5 (RFC 1321) — a wire-format detail of RADIUS and TACACS+, nothing more */
/* ========================================================================= */

static const uint32_t md5_k[64] = {
	0xd76aa478U, 0xe8c7b756U, 0x242070dbU, 0xc1bdceeeU, 0xf57c0fafU,
	0x4787c62aU, 0xa8304613U, 0xfd469501U, 0x698098d8U, 0x8b44f7afU,
	0xffff5bb1U, 0x895cd7beU, 0x6b901122U, 0xfd987193U, 0xa679438eU,
	0x49b40821U, 0xf61e2562U, 0xc040b340U, 0x265e5a51U, 0xe9b6c7aaU,
	0xd62f105dU, 0x02441453U, 0xd8a1e681U, 0xe7d3fbc8U, 0x21e1cde6U,
	0xc33707d6U, 0xf4d50d87U, 0x455a14edU, 0xa9e3e905U, 0xfcefa3f8U,
	0x676f02d9U, 0x8d2a4c8aU, 0xfffa3942U, 0x8771f681U, 0x6d9d6122U,
	0xfde5380cU, 0xa4beea44U, 0x4bdecfa9U, 0xf6bb4b60U, 0xbebfbc70U,
	0x289b7ec6U, 0xeaa127faU, 0xd4ef3085U, 0x04881d05U, 0xd9d4d039U,
	0xe6db99e5U, 0x1fa27cf8U, 0xc4ac5665U, 0xf4292244U, 0x432aff97U,
	0xab9423a7U, 0xfc93a039U, 0x655b59c3U, 0x8f0ccc92U, 0xffeff47dU,
	0x85845dd1U, 0x6fa87e4fU, 0xfe2ce6e0U, 0xa3014314U, 0x4e0811a1U,
	0xf7537e82U, 0xbd3af235U, 0x2ad7d2bbU, 0xeb86d391U
};

static const uint8_t md5_r[64] = {
	7,  12, 17, 22, 7,  12, 17, 22, 7,  12, 17, 22, 7,  12, 17, 22,
	5,  9,  14, 20, 5,  9,  14, 20, 5,  9,  14, 20, 5,  9,  14, 20,
	4,  11, 16, 23, 4,  11, 16, 23, 4,  11, 16, 23, 4,  11, 16, 23,
	6,  10, 15, 21, 6,  10, 15, 21, 6,  10, 15, 21, 6,  10, 15, 21
};

static uint32_t rotl32(uint32_t v, unsigned int n)
{
	return (uint32_t)((v << n) | (v >> (32U - n)));
}

static uint32_t le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
	       ((uint32_t)p[3] << 24);
}

static void md5_block(uint32_t h[4], const uint8_t block[64])
{
	uint32_t m[16];
	uint32_t a = h[0];
	uint32_t b = h[1];
	uint32_t c = h[2];
	uint32_t d = h[3];
	unsigned int i;

	for (i = 0U; i < 16U; i++) {
		m[i] = le32(&block[i * 4U]);
	}

	for (i = 0U; i < 64U; i++) {
		uint32_t f;
		unsigned int g;
		uint32_t tmp;

		if (i < 16U) {
			f = (b & c) | (~b & d);
			g = i;
		} else if (i < 32U) {
			f = (d & b) | (~d & c);
			g = ((5U * i) + 1U) % 16U;
		} else if (i < 48U) {
			f = b ^ c ^ d;
			g = ((3U * i) + 5U) % 16U;
		} else {
			f = c ^ (b | ~d);
			g = (7U * i) % 16U;
		}

		tmp = d;
		d = c;
		c = b;
		b = b + rotl32(a + f + md5_k[i] + m[g], md5_r[i]);
		a = tmp;
	}

	h[0] += a;
	h[1] += b;
	h[2] += c;
	h[3] += d;
}

void auth_md5_init(auth_md5_t *s)
{
	if (s == NULL) {
		return;
	}
	s->h[0] = 0x67452301U;
	s->h[1] = 0xefcdab89U;
	s->h[2] = 0x98badcfeU;
	s->h[3] = 0x10325476U;
	s->bits = 0U;
	s->n = 0U;
	memset(s->buf, 0, sizeof(s->buf));
}

void auth_md5_update(auth_md5_t *s, const void *data, size_t len)
{
	const uint8_t *p = (const uint8_t *)data;
	size_t off = 0U;

	if (s == NULL || (p == NULL && len != 0U)) {
		return;
	}
	s->bits += (uint64_t)len * 8U;

	if (s->n != 0U) {
		size_t take = 64U - s->n;

		if (take > len) {
			take = len;
		}
		memcpy(&s->buf[s->n], p, take);
		s->n += take;
		off = take;
		if (s->n == 64U) {
			md5_block(s->h, s->buf);
			s->n = 0U;
		}
	}
	while ((len - off) >= 64U) {
		md5_block(s->h, &p[off]);
		off += 64U;
	}
	if (off < len) {
		memcpy(s->buf, &p[off], len - off);
		s->n = len - off;
	}
}

void auth_md5_final(auth_md5_t *s, uint8_t out[AUTH_MD5_LEN])
{
	uint8_t tail[72];
	size_t pad;
	size_t i;
	uint64_t bits;

	if (s == NULL || out == NULL) {
		return;
	}
	bits = s->bits;

	/* 0x80 then zeros to 56 mod 64, then the length little-endian. */
	pad = (s->n < 56U) ? (56U - s->n) : (120U - s->n);
	memset(tail, 0, sizeof(tail));
	tail[0] = 0x80U;
	for (i = 0U; i < 8U; i++) {
		tail[pad + i] = (uint8_t)((bits >> (8U * i)) & 0xFFU);
	}
	auth_md5_update(s, tail, pad + 8U);

	for (i = 0U; i < 4U; i++) {
		out[(i * 4U) + 0U] = (uint8_t)(s->h[i] & 0xFFU);
		out[(i * 4U) + 1U] = (uint8_t)((s->h[i] >> 8) & 0xFFU);
		out[(i * 4U) + 2U] = (uint8_t)((s->h[i] >> 16) & 0xFFU);
		out[(i * 4U) + 3U] = (uint8_t)((s->h[i] >> 24) & 0xFFU);
	}
}

void auth_md5(const void *data, size_t len, uint8_t out[AUTH_MD5_LEN])
{
	auth_md5_t s;

	auth_md5_init(&s);
	auth_md5_update(&s, data, len);
	auth_md5_final(&s, out);
}

void auth_hmac_md5(const uint8_t *key, size_t key_len, const uint8_t *msg,
		   size_t msg_len, uint8_t out[AUTH_MD5_LEN])
{
	uint8_t k[64];
	uint8_t pad[64];
	uint8_t inner[AUTH_MD5_LEN];
	auth_md5_t s;
	size_t i;

	if (out == NULL) {
		return;
	}

	memset(k, 0, sizeof(k));
	if (key_len > sizeof(k)) {
		auth_md5(key, key_len, k);
	} else if (key_len != 0U && key != NULL) {
		memcpy(k, key, key_len);
	}

	for (i = 0U; i < sizeof(pad); i++) {
		pad[i] = (uint8_t)(k[i] ^ 0x36U);
	}
	auth_md5_init(&s);
	auth_md5_update(&s, pad, sizeof(pad));
	auth_md5_update(&s, msg, msg_len);
	auth_md5_final(&s, inner);

	for (i = 0U; i < sizeof(pad); i++) {
		pad[i] = (uint8_t)(k[i] ^ 0x5CU);
	}
	auth_md5_init(&s);
	auth_md5_update(&s, pad, sizeof(pad));
	auth_md5_update(&s, inner, sizeof(inner));
	auth_md5_final(&s, out);
}

bool auth_ct_eq(const uint8_t *a, const uint8_t *b, size_t n)
{
	uint8_t diff = 0U;
	size_t i;

	if (a == NULL || b == NULL) {
		return false;
	}
	for (i = 0U; i < n; i++) {
		diff |= (uint8_t)(a[i] ^ b[i]);
	}
	return diff == 0U;
}

/* ========================================================================= */
/* roles & chain parsing                                                     */
/* ========================================================================= */

const char *auth_role_name(uint8_t r)
{
	switch (r) {
	case AUTH_ROLE_VIEWER:
		return "viewer";
	case AUTH_ROLE_OPERATOR:
		return "operator";
	case AUTH_ROLE_ADMIN:
		return "admin";
	default:
		return "none";
	}
}

/** ASCII lower-case; locale-independent by construction. */
static char lc(char ch)
{
	return (ch >= 'A' && ch <= 'Z') ? (char)(ch - 'A' + 'a') : ch;
}

/** Case-insensitive compare of @p n octets of @p s against lower-case @p lit. */
static bool ieq_n(const char *s, size_t n, const char *lit)
{
	size_t i;

	for (i = 0U; i < n; i++) {
		if (lit[i] == '\0' || lc(s[i]) != lit[i]) {
			return false;
		}
	}
	return lit[n] == '\0';
}

static bool ieq(const char *s, const char *lit)
{
	return (s != NULL) && ieq_n(s, strlen(s), lit);
}

int auth_role_parse(const char *s)
{
	if (s == NULL || s[0] == '\0') {
		return -EINVAL;
	}
	if (ieq(s, "admin") || ieq(s, "administrative") || ieq(s, "root") ||
	    ieq(s, "superuser") || ieq(s, "rw-admin")) {
		return AUTH_ROLE_ADMIN;
	}
	if (ieq(s, "operator") || ieq(s, "rw") || ieq(s, "readwrite") ||
	    ieq(s, "read-write")) {
		return AUTH_ROLE_OPERATOR;
	}
	if (ieq(s, "viewer") || ieq(s, "ro") || ieq(s, "readonly") ||
	    ieq(s, "read-only") || ieq(s, "monitor") || ieq(s, "guest")) {
		return AUTH_ROLE_VIEWER;
	}
	return -EINVAL;
}

const char *auth_backend_name(uint8_t b)
{
	switch (b) {
	case AUTH_BE_LOCAL:
		return "local";
	case AUTH_BE_RADIUS:
		return "radius";
	case AUTH_BE_TACACS:
		return "tacacs";
	case AUTH_BE_LDAP:
		return "ldap";
	default:
		return "?";
	}
}

int auth_order_parse(const char *s, uint8_t *out, size_t cap)
{
	size_t n = 0U;
	size_t i = 0U;

	if (out == NULL || cap == 0U) {
		return -EINVAL;
	}
	if (s == NULL) {
		return 0;
	}

	while (s[i] != '\0' && n < cap) {
		size_t start;
		size_t len;
		int be = -1;
		size_t j;
		bool dup = false;

		while (s[i] == ',' || s[i] == ' ' || s[i] == '\t') {
			i++;
		}
		start = i;
		while (s[i] != '\0' && s[i] != ',' && s[i] != ' ' &&
		       s[i] != '\t') {
			i++;
		}
		len = i - start;
		if (len == 0U) {
			continue;
		}

		if (ieq_n(&s[start], len, "local")) {
			be = AUTH_BE_LOCAL;
		} else if (ieq_n(&s[start], len, "radius")) {
			be = AUTH_BE_RADIUS;
		} else if (ieq_n(&s[start], len, "tacacs") ||
			   ieq_n(&s[start], len, "tacacs+")) {
			be = AUTH_BE_TACACS;
		} else if (ieq_n(&s[start], len, "ldap")) {
			be = AUTH_BE_LDAP;
		}
		if (be < 0) {
			continue;
		}
		for (j = 0U; j < n; j++) {
			if (out[j] == (uint8_t)be) {
				dup = true;
			}
		}
		if (!dup) {
			out[n] = (uint8_t)be;
			n++;
		}
	}
	return (int)n;
}

/* ========================================================================= */
/* the local credential                                                      */
/* ========================================================================= */

int auth_local_verify(const port_crypto_t *crypto, const uint8_t *blob,
		      size_t blob_len, const char *secret)
{
	uint8_t mac[AUTH_PW_MAC_LEN];
	size_t slen;

	if (crypto == NULL || secret == NULL) {
		return -EINVAL;
	}
	slen = strlen(secret);
	if (slen == 0U || slen > AUTH_SECRET_MAX) {
		return -EINVAL;
	}
	if (blob == NULL || blob_len != AUTH_PW_BLOB_LEN) {
		return -ENOENT;
	}
	if (crypto->hmac_sha256 == NULL) {
		return -ENOTSUP;
	}

	if (crypto->hmac_sha256(crypto->ctx, blob, AUTH_PW_SALT_LEN,
				(const uint8_t *)secret, slen, mac) != 0) {
		return -EIO;
	}
	if (!auth_ct_eq(mac, &blob[AUTH_PW_SALT_LEN], AUTH_PW_MAC_LEN)) {
		return -EACCES;
	}
	return 0;
}

/* ========================================================================= */
/* context                                                                   */
/* ========================================================================= */

/** Normalise a config: empty chain becomes local-only, 0 defaults resolved. */
static void normalise(auth_cfg_t *cfg)
{
	size_t i;
	size_t n = 0U;

	if (cfg->n_order > (uint8_t)AUTH_BE_COUNT) {
		cfg->n_order = (uint8_t)AUTH_BE_COUNT;
	}
	/* Drop out-of-range and duplicate entries so the chain walk needs no
	 * validation of its own. */
	for (i = 0U; i < cfg->n_order; i++) {
		uint8_t be = cfg->order[i];
		size_t j;
		bool dup = false;

		if (be >= (uint8_t)AUTH_BE_COUNT) {
			continue;
		}
		for (j = 0U; j < n; j++) {
			if (cfg->order[j] == be) {
				dup = true;
			}
		}
		if (!dup) {
			cfg->order[n] = be;
			n++;
		}
	}
	cfg->n_order = (uint8_t)n;
	if (cfg->n_order == 0U) {
		cfg->order[0] = (uint8_t)AUTH_BE_LOCAL;
		cfg->n_order = 1U;
	}

	if (cfg->local_role == 0U || cfg->local_role > (uint8_t)AUTH_ROLE_ADMIN) {
		cfg->local_role = (uint8_t)AUTH_ROLE_ADMIN;
	}
	if (cfg->lockout_fails != 0U && cfg->lockout_s == 0U) {
		cfg->lockout_s = 60U;
	}
}

static int setup(auth_ctx_t *c, const auth_cfg_t *cfg)
{
	if (c == NULL || cfg == NULL || cfg->crypto == NULL) {
		return -EINVAL;
	}
	if (cfg->crypto->hmac_sha256 == NULL) {
		return -ENOTSUP;
	}

	c->cfg = *cfg;
	normalise(&c->cfg);

	memset(c->cache, 0, sizeof(c->cache));
	c->cache_ready = false;
	if (c->cfg.cache_ttl_s != 0U && c->cfg.crypto->rand != NULL) {
		if (c->cfg.crypto->rand(c->cfg.crypto->ctx, c->cache_salt,
					sizeof(c->cache_salt)) == 0) {
			c->cache_ready = true;
		}
	}
	if (!c->cache_ready) {
		/* No salt, no cache: a predictable cache key would let an
		 * attacker who can observe timing probe for cached users. */
		memset(c->cache_salt, 0, sizeof(c->cache_salt));
	}
	c->ready = true;
	return 0;
}

int auth_init(auth_ctx_t *c, const auth_cfg_t *cfg)
{
	if (c == NULL) {
		return -EINVAL;
	}
	memset(c, 0, sizeof(*c));
	return setup(c, cfg);
}

int auth_reconfigure(auth_ctx_t *c, const auth_cfg_t *cfg)
{
	if (c == NULL || !c->ready) {
		return -EINVAL;
	}
	/* Deliberately keeps c->locks: see auth.h. */
	return setup(c, cfg);
}

int auth_stats_get(const auth_ctx_t *c, auth_stats_t *out)
{
	if (c == NULL || out == NULL) {
		return -EINVAL;
	}
	*out = c->stats;
	return 0;
}

int auth_stats_reset(auth_ctx_t *c)
{
	if (c == NULL) {
		return -EINVAL;
	}
	memset(&c->stats, 0, sizeof(c->stats));
	return 0;
}

/* ------------------------------------------------------------------- cache */

/**
 * Cache key: HMAC-SHA-256(per-boot salt, user ‖ 0x00 ‖ secret), truncated.
 *
 * Keying it means the table holds no recoverable credential, and the 0x00
 * separator stops ("ab", "c") and ("a", "bc") from colliding.
 */
static int cache_key(auth_ctx_t *c, const char *user, const char *secret,
		     uint8_t out[16])
{
	uint8_t msg[AUTH_USER_MAX + 1U + AUTH_SECRET_MAX];
	uint8_t mac[32];
	size_t ulen = strlen(user);
	size_t slen = strlen(secret);

	memcpy(msg, user, ulen);
	msg[ulen] = 0x00U;
	memcpy(&msg[ulen + 1U], secret, slen);

	if (c->cfg.crypto->hmac_sha256(c->cfg.crypto->ctx, c->cache_salt,
				       sizeof(c->cache_salt), msg,
				       ulen + 1U + slen, mac) != 0) {
		return -EIO;
	}
	memcpy(out, mac, 16U);
	return 0;
}

static bool cache_lookup(auth_ctx_t *c, const uint8_t key[16], uint64_t now_ms,
			 uint8_t *out_role, uint8_t *out_be)
{
	size_t i;

	for (i = 0U; i < AUTH_CACHE_SLOTS; i++) {
		if (!c->cache[i].valid) {
			continue;
		}
		if (now_ms >= c->cache[i].expiry_ms) {
			c->cache[i].valid = false;
			continue;
		}
		if (auth_ct_eq(c->cache[i].key, key, 16U)) {
			*out_role = c->cache[i].role;
			*out_be = c->cache[i].backend;
			return true;
		}
	}
	return false;
}

static void cache_store(auth_ctx_t *c, const uint8_t key[16], uint64_t now_ms,
			uint8_t role, uint8_t be)
{
	size_t victim = 0U;
	uint64_t oldest = UINT64_MAX;
	size_t i;

	for (i = 0U; i < AUTH_CACHE_SLOTS; i++) {
		if (!c->cache[i].valid) {
			victim = i;
			break;
		}
		if (c->cache[i].expiry_ms < oldest) {
			oldest = c->cache[i].expiry_ms;
			victim = i;
		}
	}

	memcpy(c->cache[victim].key, key, 16U);
	c->cache[victim].expiry_ms =
		now_ms + ((uint64_t)c->cfg.cache_ttl_s * 1000U);
	c->cache[victim].role = role;
	c->cache[victim].backend = be;
	c->cache[victim].valid = true;
	c->stats.cache_fills++;
}

void auth_cache_flush(auth_ctx_t *c, const char *user)
{
	size_t i;

	if (c == NULL) {
		return;
	}
	/*
	 * The cache is keyed by a MAC over user *and* secret, so a per-user
	 * flush cannot single out one user's entries without knowing the
	 * secret. Rather than pretend otherwise, a named flush drops
	 * everything: over-invalidation costs one round trip, under-
	 * invalidation would leave a revoked credential live for the TTL.
	 */
	(void)user;
	for (i = 0U; i < AUTH_CACHE_SLOTS; i++) {
		c->cache[i].valid = false;
	}
}

/* ----------------------------------------------------------------- lockout */

static auth_lock_ent_t *lock_find(auth_ctx_t *c, const char *user)
{
	size_t i;

	for (i = 0U; i < AUTH_LOCK_SLOTS; i++) {
		if (c->locks[i].valid && strcmp(c->locks[i].user, user) == 0) {
			return &c->locks[i];
		}
	}
	return NULL;
}

/** Find or allocate a lockout slot, evicting the least recently touched. */
static auth_lock_ent_t *lock_get(auth_ctx_t *c, const char *user,
				 uint64_t now_ms)
{
	auth_lock_ent_t *e = lock_find(c, user);
	size_t victim = 0U;
	uint64_t oldest = UINT64_MAX;
	size_t i;

	if (e != NULL) {
		e->last_ms = now_ms;
		return e;
	}

	for (i = 0U; i < AUTH_LOCK_SLOTS; i++) {
		if (!c->locks[i].valid) {
			victim = i;
			break;
		}
		/* Never evict an entry that is still holding a lock: that would
		 * make "authenticate as N other users" a lockout bypass. */
		if (c->locks[i].lock_until_ms > now_ms) {
			continue;
		}
		if (c->locks[i].last_ms < oldest) {
			oldest = c->locks[i].last_ms;
			victim = i;
		}
	}
	if (c->locks[victim].valid && c->locks[victim].lock_until_ms > now_ms) {
		/* Every slot is locked. Refusing to track this user is the
		 * conservative outcome: they get no free attempts credited. */
		return NULL;
	}

	e = &c->locks[victim];
	memset(e, 0, sizeof(*e));
	(void)strncpy(e->user, user, AUTH_USER_MAX);
	e->user[AUTH_USER_MAX] = '\0';
	e->last_ms = now_ms;
	e->valid = true;
	return e;
}

bool auth_is_locked(const auth_ctx_t *c, const char *user, uint64_t mono_ms)
{
	size_t i;

	if (c == NULL || user == NULL) {
		return false;
	}
	for (i = 0U; i < AUTH_LOCK_SLOTS; i++) {
		if (c->locks[i].valid && strcmp(c->locks[i].user, user) == 0) {
			return c->locks[i].lock_until_ms > mono_ms;
		}
	}
	return false;
}

int auth_unlock(auth_ctx_t *c, const char *user)
{
	auth_lock_ent_t *e;

	if (c == NULL || user == NULL) {
		return -EINVAL;
	}
	e = lock_find(c, user);
	if (e == NULL) {
		return -ENOENT;
	}
	e->fails = 0U;
	e->lock_until_ms = 0U;
	return 0;
}

/* ------------------------------------------------------------------- chain */

/** Run the local backend. See auth.h for the accept/reject/unavailable rule. */
static int try_local(auth_ctx_t *c, const char *secret, uint8_t *out_role)
{
	int rc = auth_local_verify(c->cfg.crypto, c->cfg.local_blob,
				   c->cfg.local_blob_len, secret);

	switch (rc) {
	case 0:
		*out_role = c->cfg.local_role;
		return 0;
	case -EACCES:
		return -EACCES;
	case -ENOENT:
		/* No credential provisioned. That is "this authority has
		 * nothing to say", not "denied" — otherwise a box configured
		 * for RADIUS only could never authenticate. */
		return -EHOSTUNREACH;
	default:
		return -EHOSTUNREACH;
	}
}

static int try_remote(auth_ctx_t *c, uint8_t be, const char *user,
		      const char *secret, uint8_t *out_role)
{
	int rc;

	if (c->cfg.remote == NULL) {
		return -EHOSTUNREACH;
	}
	*out_role = (uint8_t)AUTH_ROLE_NONE;
	rc = c->cfg.remote(c->cfg.remote_ctx, be, user, secret, out_role);
	if (rc == 0) {
		if (*out_role == (uint8_t)AUTH_ROLE_NONE ||
		    *out_role > (uint8_t)AUTH_ROLE_ADMIN) {
			/* A backend that accepts without a usable role gets the
			 * least privilege rather than the most. */
			*out_role = (uint8_t)AUTH_ROLE_VIEWER;
		}
		return 0;
	}
	if (rc == -EACCES) {
		return -EACCES;
	}
	return -EHOSTUNREACH;
}

int auth_check(auth_ctx_t *c, const char *user, const char *secret,
	       uint64_t mono_ms, auth_out_t *out)
{
	uint8_t key[16];
	bool have_key = false;
	auth_lock_ent_t *lk;
	size_t i;
	int result = -EHOSTUNREACH;
	uint8_t role = (uint8_t)AUTH_ROLE_NONE;
	uint8_t decided_by = (uint8_t)AUTH_BE_COUNT;

	if (out != NULL) {
		memset(out, 0, sizeof(*out));
	}
	if (c == NULL || !c->ready || user == NULL || secret == NULL) {
		return -EINVAL;
	}
	if (user[0] == '\0' || strlen(user) > AUTH_USER_MAX) {
		return -EINVAL;
	}
	if (secret[0] == '\0' || strlen(secret) > AUTH_SECRET_MAX) {
		return -EINVAL;
	}

	c->stats.checks++;

	/* Lockout first: a locked user must not reach any backend, so a
	 * brute-force attempt costs the attacker no RADIUS round trips and the
	 * server no load. */
	lk = lock_find(c, user);
	if (lk != NULL && lk->lock_until_ms > mono_ms) {
		c->stats.locked++;
		if (out != NULL) {
			uint64_t left = lk->lock_until_ms - mono_ms;

			out->retry_after_s = (uint32_t)((left + 999U) / 1000U);
		}
		return -EBUSY;
	}

	if (c->cache_ready) {
		if (cache_key(c, user, secret, key) == 0) {
			have_key = true;
			if (cache_lookup(c, key, mono_ms, &role, &decided_by)) {
				c->stats.cache_hits++;
				c->stats.accepts++;
				if (out != NULL) {
					out->role = role;
					out->backend = decided_by;
					out->cached = true;
				}
				return 0;
			}
		}
	}

	for (i = 0U; i < c->cfg.n_order; i++) {
		uint8_t be = c->cfg.order[i];
		uint8_t r = (uint8_t)AUTH_ROLE_NONE;
		int rc;

		if (be == (uint8_t)AUTH_BE_LOCAL) {
			rc = try_local(c, secret, &r);
		} else {
			rc = try_remote(c, be, user, secret, &r);
		}

		if (rc == 0) {
			role = r;
			decided_by = be;
			result = 0;
			break;
		}
		if (rc == -EACCES) {
			decided_by = be;
			result = -EACCES;
			break;
		}
		/* unavailable: keep walking */
	}

	if (result == 0) {
		if (lk != NULL) {
			lk->fails = 0U;
			lk->lock_until_ms = 0U;
			lk->last_ms = mono_ms;
		}
		if (have_key && c->cfg.cache_ttl_s != 0U) {
			cache_store(c, key, mono_ms, role, decided_by);
		}
		c->stats.accepts++;
		if (decided_by < (uint8_t)AUTH_BE_COUNT) {
			c->stats.by_backend[decided_by]++;
		}
		if (out != NULL) {
			out->role = role;
			out->backend = decided_by;
		}
		return 0;
	}

	if (result == -EACCES) {
		c->stats.rejects++;
		if (c->cfg.lockout_fails != 0U) {
			lk = lock_get(c, user, mono_ms);
			if (lk != NULL) {
				if (lk->fails < UINT16_MAX) {
					lk->fails++;
				}
				if (lk->fails >= c->cfg.lockout_fails) {
					lk->lock_until_ms =
						mono_ms +
						((uint64_t)c->cfg.lockout_s *
						 1000U);
					lk->fails = 0U;
					if (out != NULL) {
						out->retry_after_s =
							c->cfg.lockout_s;
					}
				}
			}
		}
		if (out != NULL) {
			out->backend = decided_by;
		}
		return -EACCES;
	}

	c->stats.unavailable++;
	return -EHOSTUNREACH;
}
