/*
 * STS1000 "Meridian" — core/ptp: IEEE 1588-2019 Annex P AUTHENTICATION TLV.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See ptp_icv.h for the design, the policy set, the ICV coverage question and
 * the replay-window construction.
 */

#include "ptp/ptp_icv.h"

#include <errno.h>
#include <string.h>

#include "util/bytes.h"

/*
 * The two header fields a transparent clock may rewrite: correctionField
 * (octets 8..15, §13.3.2.7) and messageTypeSpecific (octets 16..19,
 * §13.3.2.9). Masked to zero on both sides when cfg.mask_mutable is set.
 */
#define MUTABLE_OFF 8U
#define MUTABLE_LEN 12U

_Static_assert(PTP_ICV_REPLAY_WINDOW_MAX <= 32U,
	       "the replay bitmap is 32 bits wide");
_Static_assert((MUTABLE_OFF + MUTABLE_LEN) <= PTP_HDR_LEN,
	       "the masked window must lie inside the common header");

/* ------------------------------------------------------------------- cfg -- */

size_t ptp_icv_suite_len(uint8_t suite)
{
	switch (suite) {
	case (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_128:
		return 16U;
	case (uint8_t)PTP_ICV_SUITE_HMAC_SHA256_256:
		return 32U;
	default:
		return 0U;
	}
}

void ptp_icv_cfg_defaults(ptp_icv_cfg_t *cfg)
{
	if (cfg == NULL) {
		return;
	}
	(void)memset(cfg, 0, sizeof(*cfg));
	cfg->policy = (uint8_t)PTP_ICV_POLICY_VERIFY_IF_PRESENT;
	cfg->tlv_type = (uint16_t)PTP_TLV_TYPE_AUTHENTICATION;
	cfg->tx_spp = 0U;
	cfg->tx_key_id = 0U;
	cfg->replay_protect = true;
	cfg->replay_window = 16U;
	cfg->mask_mutable = true;
}

static ptp_icv_key_t *key_find(ptp_icv_cfg_t *cfg, uint8_t spp, uint32_t key_id)
{
	size_t i;

	for (i = 0U; i < (size_t)PTP_ICV_MAX_KEYS; i++) {
		if (cfg->key[i].in_use && (cfg->key[i].spp == spp) &&
		    (cfg->key[i].key_id == key_id)) {
			return &cfg->key[i];
		}
	}
	return NULL;
}

static const ptp_icv_key_t *key_find_const(const ptp_icv_cfg_t *cfg, uint8_t spp,
					   uint32_t key_id)
{
	size_t i;

	for (i = 0U; i < (size_t)PTP_ICV_MAX_KEYS; i++) {
		if (cfg->key[i].in_use && (cfg->key[i].spp == spp) &&
		    (cfg->key[i].key_id == key_id)) {
			return &cfg->key[i];
		}
	}
	return NULL;
}

static bool any_key(const ptp_icv_cfg_t *cfg)
{
	size_t i;

	for (i = 0U; i < (size_t)PTP_ICV_MAX_KEYS; i++) {
		if (cfg->key[i].in_use) {
			return true;
		}
	}
	return false;
}

int ptp_icv_key_set(ptp_icv_cfg_t *cfg, uint8_t spp, uint32_t key_id,
		    uint8_t suite, const uint8_t *key, size_t key_len)
{
	ptp_icv_key_t *k;
	size_t i;

	if ((cfg == NULL) || (key == NULL)) {
		return -EINVAL;
	}
	if (ptp_icv_suite_len(suite) == 0U) {
		return -EINVAL;
	}
	if ((key_len == 0U) || (key_len > (size_t)PTP_ICV_KEY_MAX)) {
		return -EINVAL;
	}

	k = key_find(cfg, spp, key_id);
	if (k == NULL) {
		for (i = 0U; i < (size_t)PTP_ICV_MAX_KEYS; i++) {
			if (!cfg->key[i].in_use) {
				k = &cfg->key[i];
				break;
			}
		}
	}
	if (k == NULL) {
		return -ENOSPC;
	}

	(void)memset(k, 0, sizeof(*k));
	k->in_use = true;
	k->spp = spp;
	k->key_id = key_id;
	k->suite = suite;
	k->key_len = (uint8_t)key_len;
	(void)memcpy(k->key, key, key_len);
	return 0;
}

int ptp_icv_key_clear(ptp_icv_cfg_t *cfg, uint8_t spp, uint32_t key_id)
{
	ptp_icv_key_t *k;

	if (cfg == NULL) {
		return -EINVAL;
	}
	k = key_find(cfg, spp, key_id);
	if (k == NULL) {
		return -ENOENT;
	}
	/* Wipe, not just mark free: a retired key must not linger in RAM. */
	(void)memset(k, 0, sizeof(*k));
	return 0;
}

int ptp_icv_cfg_validate(const ptp_icv_cfg_t *cfg)
{
	size_t i;

	if (cfg == NULL) {
		return -EINVAL;
	}
	if ((unsigned int)cfg->policy >= (unsigned int)PTP_ICV_POLICY_COUNT) {
		return -EINVAL;
	}
	if (cfg->replay_window > (uint8_t)PTP_ICV_REPLAY_WINDOW_MAX) {
		return -EINVAL;
	}
	for (i = 0U; i < (size_t)PTP_ICV_MAX_KEYS; i++) {
		const ptp_icv_key_t *k = &cfg->key[i];

		if (!k->in_use) {
			continue;
		}
		if (ptp_icv_suite_len(k->suite) == 0U) {
			return -EINVAL;
		}
		if ((k->key_len == 0U) || (k->key_len > (uint8_t)PTP_ICV_KEY_MAX)) {
			return -EINVAL;
		}
	}

	/*
	 * An empty table is legal and inert. A *populated* table whose transmit
	 * association is missing is a configuration mistake worth reporting:
	 * the operator provisioned keys and then named the wrong one, and the
	 * symptom would otherwise be silently unsigned Announces.
	 */
	if ((cfg->policy != (uint8_t)PTP_ICV_POLICY_OFF) && any_key(cfg) &&
	    (key_find_const(cfg, cfg->tx_spp, cfg->tx_key_id) == NULL)) {
		return -ENOENT;
	}
	return 0;
}

/* -------------------------------------------------------------- lifecycle -- */

int ptp_icv_init(ptp_icv_ctx_t *c, const ptp_icv_cfg_t *cfg,
		 const port_crypto_t *crypto)
{
	int rc;

	if ((c == NULL) || (cfg == NULL) || (crypto == NULL) ||
	    (crypto->hmac_sha256 == NULL)) {
		return -EINVAL;
	}
	rc = ptp_icv_cfg_validate(cfg);
	if (rc != 0) {
		return rc;
	}

	(void)memset(c, 0, sizeof(*c));
	c->cfg = *cfg;
	c->crypto = *crypto;
	return 0;
}

int ptp_icv_set_cfg(ptp_icv_ctx_t *c, const ptp_icv_cfg_t *cfg)
{
	int rc;

	if ((c == NULL) || (cfg == NULL)) {
		return -EINVAL;
	}
	rc = ptp_icv_cfg_validate(cfg);
	if (rc != 0) {
		return rc;
	}
	c->cfg = *cfg;
	return 0;
}

void ptp_icv_reset_peers(ptp_icv_ctx_t *c)
{
	if (c == NULL) {
		return;
	}
	(void)memset(c->peer, 0, sizeof(c->peer));
}

const ptp_icv_counters_t *ptp_icv_counters(const ptp_icv_ctx_t *c)
{
	return (c != NULL) ? &c->counters : NULL;
}

/* ------------------------------------------------------------ constant time */

bool ptp_icv_ct_equal(const uint8_t *a, const uint8_t *b, size_t n)
{
	uint8_t diff = 0U;
	size_t i;

	if ((a == NULL) || (b == NULL) || (n == 0U)) {
		return false;
	}
	/*
	 * No early exit: every byte of both operands is read and the
	 * differences are OR-accumulated, so the loop's duration is a function
	 * of n alone. An early-exit compare would leak the length of a matching
	 * prefix, which is enough to forge an ICV one byte at a time.
	 */
	for (i = 0U; i < n; i++) {
		diff |= (uint8_t)(a[i] ^ b[i]);
	}
	return diff == 0U;
}

/* --------------------------------------------------------------- hashing --- */

/*
 * Hash buf[0, cover) into out[32] with the module's coverage rules applied.
 *
 * The bytes actually hashed are always assembled in the context scratch, in one
 * code path shared by transmit and receive. That is what makes the two ends
 * provably agree on the covered octets: there is no second implementation to
 * drift.
 */
static int icv_mac(ptp_icv_ctx_t *c, const ptp_icv_key_t *k, const uint8_t *buf,
		   size_t cover, uint8_t out[32])
{
	if (cover > sizeof(c->scratch)) {
		return -EBADMSG;
	}
	(void)memcpy(c->scratch, buf, cover);

	if (c->cfg.mask_mutable && (cover >= (size_t)(MUTABLE_OFF + MUTABLE_LEN))) {
		(void)memset(&c->scratch[MUTABLE_OFF], 0, (size_t)MUTABLE_LEN);
	}

	return c->crypto.hmac_sha256(c->crypto.ctx, k->key, (size_t)k->key_len,
				     c->scratch, cover, out);
}

/* -------------------------------------------------------------- transmit --- */

int ptp_icv_append(ptp_icv_ctx_t *c, uint8_t *buf, size_t cap, size_t *len)
{
	const ptp_icv_key_t *k;
	uint8_t value[PTP_ICV_FIXED_LEN + PTP_ICV_SEQNO_LEN + PTP_ICV_MAX_LEN];
	uint8_t tag[32];
	size_t icv_len;
	size_t value_len;
	size_t val_off = 0U;
	size_t icv_off;
	uint8_t spi = 0U;
	size_t n = 0U;
	int rc;

	if ((c == NULL) || (buf == NULL) || (len == NULL)) {
		return -EINVAL;
	}
	if (c->cfg.policy == (uint8_t)PTP_ICV_POLICY_OFF) {
		return -ENOENT;
	}

	k = key_find_const(&c->cfg, c->cfg.tx_spp, c->cfg.tx_key_id);
	if (k == NULL) {
		/*
		 * No provisioned key. Not an error the caller reacts to: the
		 * feature is simply not armed yet, and an unsigned Announce is
		 * exactly what a peer expects from a clock with no association.
		 */
		return -ENOENT;
	}
	icv_len = ptp_icv_suite_len(k->suite);
	if (icv_len == 0U) {
		c->counters.tx_errors++;
		return -EINVAL;
	}

	if (c->cfg.replay_protect) {
		spi |= (uint8_t)PTP_ICV_SPI_SEQNO;
	}

	value[n++] = k->spp;
	value[n++] = spi;
	bytes_put_be32(&value[n], k->key_id);
	n += 4U;
	if (c->cfg.replay_protect) {
		/*
		 * Pre-increment, so the first message on a fresh context carries
		 * 1 and a peer's window never has to special-case 0. Wrapping
		 * after 2^32 messages is 8.5 years at 16 Sync/s and is handled
		 * by the window as an ordinary large forward jump.
		 */
		c->tx_seq++;
		bytes_put_be32(&value[n], c->tx_seq);
		n += 4U;
	}

	/* The ICV field is appended zeroed and filled in after the MAC. */
	(void)memset(&value[n], 0, icv_len);
	value_len = n + icv_len;

	rc = ptp_tlv_append(buf, cap, len, c->cfg.tlv_type, value, value_len,
			    &val_off);
	if (rc != 0) {
		c->counters.tx_errors++;
		return (rc == -ENOSPC) ? -ENOSPC : rc;
	}

	icv_off = val_off + n;

	rc = icv_mac(c, k, buf, icv_off, tag);
	if (rc != 0) {
		/*
		 * The TLV is already in the buffer with a zero ICV, which no
		 * peer would accept. Roll the message back to what it was so a
		 * caller that ignores the return code transmits an ordinary
		 * unsigned message rather than one carrying a forged-looking
		 * all-zero ICV.
		 */
		size_t base = val_off - (size_t)PTP_TLV_HDR_LEN;

		bytes_put_be16(&buf[2], (uint16_t)base);
		*len = base;
		c->counters.tx_errors++;
		return -EIO;
	}

	(void)memcpy(&buf[icv_off], tag, icv_len);
	c->counters.tx_appended++;
	return 0;
}

/* --------------------------------------------------------------- receive --- */

/* Locate the AUTHENTICATION TLV. -ENOENT when absent. */
static int find_auth_tlv(const ptp_icv_ctx_t *c, const uint8_t *buf, size_t len,
			 const uint8_t **value, size_t *val_len)
{
	ptp_tlv_iter_t it;
	uint16_t type;
	int rc;

	rc = ptp_tlv_iter_begin(&it, buf, len);
	if (rc != 0) {
		return rc;
	}
	for (;;) {
		rc = ptp_tlv_iter_next(&it, &type, value, val_len, NULL);
		if (rc != 0) {
			return rc;
		}
		if (type == c->cfg.tlv_type) {
			return 0;
		}
	}
}

/*
 * Sliding-window replay check, the standard anti-replay construction: the
 * highest accepted sequence number plus a bitmap of the window below it.
 *
 * Called only after the ICV has verified. That ordering is the whole point — a
 * forged message must not be able to advance a peer's window and so lock out
 * the genuine messages that follow it.
 */
static bool replay_ok(ptp_icv_ctx_t *c, ptp_icv_peer_t *p, uint32_t seq)
{
	uint32_t window = (c->cfg.replay_window == 0U)
				  ? 1U
				  : (uint32_t)c->cfg.replay_window;
	uint32_t diff;

	if (!p->in_use) {
		p->in_use = true;
		p->highest = seq;
		p->bitmap = 0U;
		return true;
	}

	if (seq > p->highest) {
		diff = seq - p->highest;
		if (diff >= 32U) {
			p->bitmap = 0U;
		} else {
			p->bitmap = (p->bitmap << diff) |
				    ((uint32_t)1U << (diff - 1U));
		}
		p->highest = seq;
		return true;
	}

	diff = p->highest - seq;
	if (diff == 0U) {
		return false; /* the highest itself, replayed */
	}
	if (diff > window) {
		return false; /* below the window: unverifiable, so refused */
	}
	if ((p->bitmap & ((uint32_t)1U << (diff - 1U))) != 0U) {
		return false; /* already seen */
	}
	p->bitmap |= (uint32_t)1U << (diff - 1U);
	return true;
}

static ptp_icv_peer_t *peer_slot(ptp_icv_ctx_t *c, const ptp_port_id_t *id,
				 uint64_t now_ms)
{
	ptp_icv_peer_t *free_slot = NULL;
	ptp_icv_peer_t *oldest;
	size_t i;

	for (i = 0U; i < (size_t)PTP_ICV_MAX_PEERS; i++) {
		if (c->peer[i].in_use) {
			if (ptp_port_id_cmp(&c->peer[i].peer, id) == 0) {
				c->peer[i].last_ms = now_ms;
				return &c->peer[i];
			}
		} else if (free_slot == NULL) {
			free_slot = &c->peer[i];
		} else {
			/* already have a free slot */
		}
	}

	if (free_slot != NULL) {
		(void)memset(free_slot, 0, sizeof(*free_slot));
		free_slot->peer = *id;
		free_slot->last_ms = now_ms;
		return free_slot;
	}

	/* Full: displace the least recently heard peer. */
	oldest = &c->peer[0];
	for (i = 1U; i < (size_t)PTP_ICV_MAX_PEERS; i++) {
		if (c->peer[i].last_ms < oldest->last_ms) {
			oldest = &c->peer[i];
		}
	}
	(void)memset(oldest, 0, sizeof(*oldest));
	oldest->peer = *id;
	oldest->last_ms = now_ms;
	c->counters.peers_evicted++;
	return oldest;
}

int ptp_icv_verify(ptp_icv_ctx_t *c, const uint8_t *buf, size_t len,
		   const ptp_port_id_t *peer, uint64_t now_ms)
{
	const uint8_t *value = NULL;
	size_t val_len = 0U;
	const ptp_icv_key_t *k;
	uint8_t tag[32];
	uint8_t spp;
	uint8_t spi;
	uint32_t key_id;
	uint32_t seq = 0U;
	bool have_seq = false;
	size_t opt = 0U;
	size_t icv_len;
	size_t icv_off;
	int rc;

	if ((c == NULL) || (buf == NULL)) {
		return -EINVAL;
	}
	if (c->cfg.policy == (uint8_t)PTP_ICV_POLICY_OFF) {
		return 0;
	}

	rc = find_auth_tlv(c, buf, len, &value, &val_len);
	if (rc == -ENOENT) {
		if (c->cfg.policy == (uint8_t)PTP_ICV_POLICY_REQUIRE) {
			c->counters.rx_absent_rej++;
			return -EACCES;
		}
		c->counters.rx_absent_ok++;
		return 0;
	}
	if (rc != 0) {
		/*
		 * A malformed TLV suffix. Under REQUIRE this is a rejection
		 * either way; under VERIFY_IF_PRESENT it is still a rejection,
		 * because "I could not parse the suffix" is not the same claim
		 * as "there was no TLV" and must not be treated as one.
		 */
		c->counters.rx_malformed++;
		return -EBADMSG;
	}

	if (val_len < (size_t)PTP_ICV_FIXED_LEN) {
		c->counters.rx_malformed++;
		return -EBADMSG;
	}

	spp = value[0];
	spi = value[1];
	key_id = bytes_get_be32(&value[2]);

	if ((spi & (uint8_t)PTP_ICV_SPI_DISCLOSED_KEY) != 0U) {
		/*
		 * TESLA delayed key disclosure. Its disclosedKey field is
		 * variable-length and this module cannot locate the ICV without
		 * parsing it, so the message is refused rather than hashed over
		 * a guessed offset.
		 */
		c->counters.rx_malformed++;
		return -EBADMSG;
	}
	if ((spi & (uint8_t)PTP_ICV_SPI_SEQNO) != 0U) {
		if (val_len < ((size_t)PTP_ICV_FIXED_LEN + (size_t)PTP_ICV_SEQNO_LEN)) {
			c->counters.rx_malformed++;
			return -EBADMSG;
		}
		seq = bytes_get_be32(&value[PTP_ICV_FIXED_LEN]);
		have_seq = true;
		opt += (size_t)PTP_ICV_SEQNO_LEN;
	}
	if ((spi & (uint8_t)PTP_ICV_SPI_RES) != 0U) {
		if (val_len < ((size_t)PTP_ICV_FIXED_LEN + opt + (size_t)PTP_ICV_RES_LEN)) {
			c->counters.rx_malformed++;
			return -EBADMSG;
		}
		opt += (size_t)PTP_ICV_RES_LEN;
	}

	k = key_find_const(&c->cfg, spp, key_id);
	if (k == NULL) {
		c->counters.rx_no_key++;
		return -EPERM;
	}
	icv_len = ptp_icv_suite_len(k->suite);

	if (val_len != ((size_t)PTP_ICV_FIXED_LEN + opt + icv_len)) {
		/*
		 * The association's suite fixes the ICV length, so a TLV whose
		 * remaining bytes are a different length is either a different
		 * suite under the same keyID or a truncation. Either way there
		 * is nothing to compare.
		 */
		c->counters.rx_malformed++;
		return -EBADMSG;
	}

	icv_off = (size_t)(value - buf) + (size_t)PTP_ICV_FIXED_LEN + opt;

	rc = icv_mac(c, k, buf, icv_off, tag);
	if (rc == -EBADMSG) {
		c->counters.rx_malformed++;
		return -EBADMSG;
	}
	if (rc != 0) {
		/*
		 * The crypto port failed, so the message is unverified. Failing
		 * closed is the only safe answer: a crypto outage must not turn
		 * into a window in which anything is accepted.
		 */
		return -EIO;
	}

	if (!ptp_icv_ct_equal(tag, &buf[icv_off], icv_len)) {
		c->counters.rx_bad_icv++;
		return -EACCES;
	}

	/* Authenticated. Only now may this message touch the replay window. */
	if (c->cfg.replay_protect && have_seq && (peer != NULL)) {
		ptp_icv_peer_t *p = peer_slot(c, peer, now_ms);

		if (!replay_ok(c, p, seq)) {
			c->counters.rx_replay++;
			return -EPROTO;
		}
	}

	c->counters.rx_ok++;
	return 0;
}
