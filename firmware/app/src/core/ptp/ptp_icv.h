/*
 * STS1000 "Meridian" — core/ptp: IEEE 1588-2019 Annex P integrity
 * (the AUTHENTICATION TLV).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11, no allocation, all state in a caller-owned context
 * (ARCHITECTURE.md §4). Crypto arrives through port_crypto_t; this module owns
 * no primitive of its own. Spec reference: ntp_server_software_spec.md §4.4
 * "Security: IEEE 1588-2019 Annex P integrity (ICV/TLV) where peers support it".
 *
 * ---------------------------------------------------------------------------
 * What Annex P integrity is
 * ---------------------------------------------------------------------------
 *
 * IEEE 1588-2019 adds an AUTHENTICATION TLV carrying an Integrity Check Value
 * over the PTP message. A Security Parameters Pointer (SPP) selects a security
 * association; a keyID selects a key within it; the ICV is a MAC over the
 * message. The default suite here is HMAC-SHA-256 truncated to 128 bits, which
 * is what §16.14 names as the mandatory-to-implement algorithm and what
 * port_crypto_t already provides.
 *
 * Three policies, chosen by configuration:
 *
 *   OFF               no TLV is emitted; a received TLV is ignored entirely.
 *   VERIFY_IF_PRESENT a TLV, if present, must verify; absence is accepted.
 *                     This is the interop-friendly setting and the default,
 *                     because §4.4 says "where peers support it".
 *   REQUIRE           every received message must carry a verifying TLV.
 *
 * TX is independent of the RX policy: a TLV is emitted whenever the policy is
 * not OFF and a transmit key is configured. That asymmetry is deliberate —
 * signing our own Announces costs nothing and helps a peer that does check,
 * while requiring signatures from peers is a deployment decision.
 *
 * ---------------------------------------------------------------------------
 * ICV coverage, and why it is a configuration bit
 * ---------------------------------------------------------------------------
 *
 * The ICV covers the message from octet 0 up to but not including the ICV field
 * itself, with the ICV field's own octets treated as absent (they are written
 * after the MAC is computed, and are zero while it is computed). messageLength
 * is already updated to include the TLV before the MAC is taken, so both ends
 * hash the same bytes.
 *
 * Two header fields are legitimately rewritten in flight by a transparent
 * clock: correctionField (octets 8..15) and messageTypeSpecific (octets 16..19).
 * An ICV that covers them survives only a path with no transparent clocks.
 * @ref ptp_icv_cfg_t::mask_mutable therefore selects between
 *
 *   true  (default)  those twelve octets are treated as zero on both sides, so
 *                    the ICV survives a transparent clock;
 *   false            everything is covered, which is stronger on a path known
 *                    to have no TCs and is what a peer configured that way
 *                    expects.
 *
 * Both ends must agree; a mismatch shows up as a 100 % ICV failure rate rather
 * than as anything subtle, and the counters say which.
 *
 * ---------------------------------------------------------------------------
 * Replay protection
 * ---------------------------------------------------------------------------
 *
 * §16.14 makes the sequenceNo field optional. With
 * @ref ptp_icv_cfg_t::replay_protect set, this module emits a monotonically
 * increasing 32-bit sequence number and keeps a bounded sliding window per
 * peer — highest accepted value plus a 32-bit bitmap of the window below it,
 * the standard anti-replay construction. A repeat inside the window, or
 * anything below it, is rejected. Peers are tracked in a fixed table
 * (PTP_ICV_MAX_PEERS); a full table evicts the least recently heard entry, and
 * an evicted peer's window restarts from its next message. That is a bounded
 * loss of replay history, not of authenticity: the ICV is still checked.
 *
 * ---------------------------------------------------------------------------
 * Constant time
 * ---------------------------------------------------------------------------
 *
 * ptp_icv_ct_equal() is the only comparison used on an ICV. It reads every byte
 * of both operands and accumulates differences, so its timing depends on the
 * length and not on the data. A memcmp() here would leak the length of a
 * matching prefix and let an attacker forge an ICV byte at a time.
 */

#ifndef STS1000_CORE_PTP_PTP_ICV_H_
#define STS1000_CORE_PTP_PTP_ICV_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "port/port_crypto.h"
#include "ptp/ptp_msg.h"
#include "ptp/ptp_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------- constants -- */

/** Longest ICV this module produces or accepts (full SHA-256 output). */
#define PTP_ICV_MAX_LEN 32U

/** Keys held in the association table. */
#ifndef PTP_ICV_MAX_KEYS
#define PTP_ICV_MAX_KEYS 4U
#endif

/** Longest key octet string. HMAC accepts any length; 64 is SHA-256's block. */
#define PTP_ICV_KEY_MAX 64U

/** Peers whose replay windows are tracked. */
#ifndef PTP_ICV_MAX_PEERS
#define PTP_ICV_MAX_PEERS 8U
#endif

/** Widest replay window, set by the bitmap width. */
#define PTP_ICV_REPLAY_WINDOW_MAX 32U

/** Octets of AUTHENTICATION TLV value before the optional fields. */
#define PTP_ICV_FIXED_LEN 6U /* SPP 1 + secParamIndicator 1 + keyID 4 */

/** Octets of the optional sequenceNo field. */
#define PTP_ICV_SEQNO_LEN 4U

/*
 * secParamIndicator bits (§16.14.3).
 *
 * VERIFY vs IEEE 1588-2019 §16.14.3 — the bit assignments are not available to
 * this project. Bit 0 selects the TESLA disclosedKey field, which this module
 * does not implement: a message that sets it is rejected as unsupported rather
 * than misparsed. Bits 1 and 2 select sequenceNo and RES; both are parsed.
 */
#define PTP_ICV_SPI_DISCLOSED_KEY 0x01U /* VERIFY vs IEEE 1588-2019 §16.14.3 */
#define PTP_ICV_SPI_SEQNO         0x02U /* VERIFY vs IEEE 1588-2019 §16.14.3 */
#define PTP_ICV_SPI_RES           0x04U /* VERIFY vs IEEE 1588-2019 §16.14.3 */
/** Octets of the optional RES field when PTP_ICV_SPI_RES is set. */
#define PTP_ICV_RES_LEN 4U

/* ------------------------------------------------------------ enums, cfg -- */

/** What to do about integrity on received messages. */
typedef enum {
	PTP_ICV_POLICY_OFF = 0,
	PTP_ICV_POLICY_VERIFY_IF_PRESENT,
	PTP_ICV_POLICY_REQUIRE,
	PTP_ICV_POLICY_COUNT,
} ptp_icv_policy_t;

/** Integrity algorithm. */
typedef enum {
	/** HMAC-SHA-256 truncated to 16 octets — the default suite. */
	PTP_ICV_SUITE_HMAC_SHA256_128 = 0,
	/** HMAC-SHA-256, full 32-octet tag. */
	PTP_ICV_SUITE_HMAC_SHA256_256,
	PTP_ICV_SUITE_COUNT,
} ptp_icv_suite_t;

/** ICV octet count of @p suite; 0 for an out-of-range suite. */
size_t ptp_icv_suite_len(uint8_t suite);

/** One security association: an (SPP, keyID) pair and its key. */
typedef struct {
	bool in_use;
	uint8_t spp;
	uint32_t key_id;
	uint8_t suite;   /**< ptp_icv_suite_t */
	uint8_t key_len; /**< 1..PTP_ICV_KEY_MAX */
	uint8_t key[PTP_ICV_KEY_MAX];
} ptp_icv_key_t;

/** Runtime configuration. Populate with ptp_icv_cfg_defaults(), then override. */
typedef struct {
	uint8_t policy; /**< ptp_icv_policy_t */
	/**
	 * tlvType to emit and to recognise.
	 *
	 * Defaults to PTP_TLV_TYPE_AUTHENTICATION. Configurable because that
	 * constant is VERIFY-marked; PTP_TLV_TYPE_AUTHENTICATION_2008 is the
	 * other value a peer might use.
	 */
	uint16_t tlv_type;
	/** Association used for transmit. Must exist in @ref key. */
	uint8_t tx_spp;
	uint32_t tx_key_id;
	/** Emit sequenceNo on TX and enforce the window on RX. */
	bool replay_protect;
	/** Window width, 1..PTP_ICV_REPLAY_WINDOW_MAX. 0 means "1" (strict). */
	uint8_t replay_window;
	/** Zero correctionField and messageTypeSpecific before hashing. */
	bool mask_mutable;
	ptp_icv_key_t key[PTP_ICV_MAX_KEYS];
} ptp_icv_cfg_t;

/**
 * Fill @p cfg with the documented defaults.
 *
 * policy = VERIFY_IF_PRESENT, tlv_type = PTP_TLV_TYPE_AUTHENTICATION,
 * replay_protect = true, replay_window = 16, mask_mutable = true, and an empty
 * key table. With no key installed, TX emits nothing and RX accepts unsigned
 * messages — i.e. the defaults are inert until an operator provisions a key,
 * which is the only safe behaviour for a feature whose keys arrive out of band.
 */
void ptp_icv_cfg_defaults(ptp_icv_cfg_t *cfg);

/**
 * Install (or replace) an association.
 *
 * Replaces the entry with the same (spp, key_id); otherwise takes a free slot.
 *
 * @retval 0        Installed.
 * @retval -EINVAL  NULL argument, bad suite, or a key length outside 1..64.
 * @retval -ENOSPC  The table is full.
 */
int ptp_icv_key_set(ptp_icv_cfg_t *cfg, uint8_t spp, uint32_t key_id,
		    uint8_t suite, const uint8_t *key, size_t key_len);

/**
 * Remove the association for (@p spp, @p key_id).
 *
 * The slot is wiped, not merely marked free, so a retired key does not linger
 * in RAM where a memory dump would find it.
 *
 * @retval 0        Removed.
 * @retval -ENOENT  No such association.
 * @retval -EINVAL  @p cfg is NULL.
 */
int ptp_icv_key_clear(ptp_icv_cfg_t *cfg, uint8_t spp, uint32_t key_id);

/**
 * Check @p cfg for internal consistency.
 *
 * @retval 0        Usable.
 * @retval -EINVAL  NULL, a policy or window out of range, or a key entry whose
 *                  suite or length is invalid.
 * @retval -ENOENT  The policy is not OFF and @p tx_spp / @p tx_key_id names an
 *                  association that is not installed *and* at least one key is
 *                  installed. An entirely empty table is legal (inert).
 */
int ptp_icv_cfg_validate(const ptp_icv_cfg_t *cfg);

/* ------------------------------------------------------------- counters --- */

typedef struct {
	uint32_t tx_appended;   /**< TLVs emitted */
	uint32_t tx_errors;     /**< TLV could not be built (no key, no space) */
	uint32_t rx_ok;         /**< verified */
	uint32_t rx_absent_ok;  /**< no TLV, accepted by policy */
	uint32_t rx_absent_rej; /**< no TLV, rejected by REQUIRE */
	uint32_t rx_bad_icv;    /**< ICV mismatch */
	uint32_t rx_no_key;     /**< unknown (SPP, keyID) */
	uint32_t rx_replay;     /**< sequenceNo already seen, or below the window */
	uint32_t rx_malformed;  /**< malformed TLV suffix or unsupported option */
	uint32_t peers_evicted; /**< replay-window records displaced */
} ptp_icv_counters_t;

/* -------------------------------------------------------------- context --- */

/** One peer's replay window. */
typedef struct {
	bool in_use;
	ptp_port_id_t peer;
	uint32_t highest;  /**< highest accepted sequenceNo */
	uint32_t bitmap;   /**< bit n set = (highest - 1 - n) has been seen */
	uint64_t last_ms;
} ptp_icv_peer_t;

/** Engine state. Caller-owned; initialise with ptp_icv_init(). */
typedef struct {
	ptp_icv_cfg_t cfg;
	port_crypto_t crypto;
	uint32_t tx_seq;
	ptp_icv_peer_t peer[PTP_ICV_MAX_PEERS];
	ptp_icv_counters_t counters;
	/**
	 * Scratch for the bytes actually hashed.
	 *
	 * Both directions copy the message here before hashing: TX because the
	 * masked octets must be restored afterwards, RX because the buffer is
	 * const. One code path for both is what makes the two ends provably
	 * agree on the covered bytes.
	 */
	uint8_t scratch[PTP_MSG_MAX_LEN];
} ptp_icv_ctx_t;

/**
 * Initialise @p c from @p cfg and @p crypto.
 *
 * @param crypto  Must provide hmac_sha256. Copied into the context.
 *
 * @retval 0        Ready.
 * @retval -EINVAL  NULL argument, or @p crypto has no hmac_sha256.
 * @retval other    As ptp_icv_cfg_validate().
 */
int ptp_icv_init(ptp_icv_ctx_t *c, const ptp_icv_cfg_t *cfg,
		 const port_crypto_t *crypto);

/** Replace the configuration in place, keeping the counters and windows. */
int ptp_icv_set_cfg(ptp_icv_ctx_t *c, const ptp_icv_cfg_t *cfg);

/** Forget every replay window. Used when the segment stops being trustworthy. */
void ptp_icv_reset_peers(ptp_icv_ctx_t *c);

/**
 * Append an AUTHENTICATION TLV to an encoded message.
 *
 * @param buf  Encoded message, at least @p *len octets, with a valid header.
 * @param cap  Capacity of @p buf.
 * @param len  In: message length. Out: length including the TLV.
 *
 * @retval 0        Appended; messageLength updated.
 * @retval -EINVAL  NULL argument or an implausible @p *len.
 * @retval -ENOENT  Nothing to do: the policy is OFF, or no transmit key is
 *                  installed. @p *len is unchanged and this is not an error the
 *                  caller has to react to.
 * @retval -ENOSPC  @p cap cannot hold the TLV. Counted in tx_errors.
 * @retval -EIO     The crypto port failed. Counted in tx_errors.
 */
int ptp_icv_append(ptp_icv_ctx_t *c, uint8_t *buf, size_t cap, size_t *len);

/**
 * Apply the receive policy to a message.
 *
 * @param buf   The message as received.
 * @param len   Octets available.
 * @param peer  sourcePortIdentity, for the replay window. May be NULL, in which
 *              case replay checking is skipped for this message (and counted as
 *              a plain verification).
 *
 * @retval 0        Accept. Either it verified, or no TLV was present and the
 *                  policy tolerates that.
 * @retval -EINVAL  @p c or @p buf is NULL.
 * @retval -EACCES  ICV mismatch, or absent under PTP_ICV_POLICY_REQUIRE.
 * @retval -EPERM   The TLV names an association that is not installed.
 * @retval -EPROTO  Replay: the sequenceNo has been seen, or is below the window.
 * @retval -EBADMSG Malformed TLV suffix, a truncated AUTHENTICATION TLV, or an
 *                  option this module does not implement (disclosedKey).
 * @retval -EIO     The crypto port failed; the message is *not* accepted.
 */
int ptp_icv_verify(ptp_icv_ctx_t *c, const uint8_t *buf, size_t len,
		   const ptp_port_id_t *peer, uint64_t now_ms);

/** Counter block, or NULL for a NULL context. */
const ptp_icv_counters_t *ptp_icv_counters(const ptp_icv_ctx_t *c);

/**
 * Constant-time equality over @p n octets.
 *
 * Reads all of both operands regardless of where they first differ. Returns
 * false for a NULL operand or @p n == 0 — an empty ICV is never a match.
 */
bool ptp_icv_ct_equal(const uint8_t *a, const uint8_t *b, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_PTP_PTP_ICV_H_ */
