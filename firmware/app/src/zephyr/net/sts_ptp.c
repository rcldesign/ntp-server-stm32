/*
 * STS1000 "Meridian" — PTP grandmaster transport + thread (spec §4.4, prio 5).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * core/ptp is the whole engine: BMCA, port state machine, message codecs,
 * clockQuality mapping. It emits PDU bytes plus an addressing hint and consumes
 * PDU bytes plus a hardware timestamp (ptp.h scope note 5). This file is only
 * the encapsulation and the socket plumbing:
 *
 *   Annex C  UDP/IPv4, 224.0.1.129, event 319 / general 320
 *   Annex D  UDP/IPv6, ff0e::181,   event 319 / general 320
 *   Annex E  IEEE 802.3, EtherType 0x88F7, 01-1B-19-00-00-00
 *
 * One transport is active at a time, chosen by cfg key ptp.transport, whose
 * numbering is ptp_transport_t (0 = UDPv4, 1 = UDPv6, 2 = L2). It is
 * reboot-required config, so this file reads it once at start.
 *
 * ---------------------------------------------------------------------------
 * Timestamps
 * ---------------------------------------------------------------------------
 *
 * RX: the event socket asks for SOF_TIMESTAMPING_RX_HARDWARE and the stamp
 * arrives as a SOL_SOCKET/SO_TIMESTAMPING control message. eth_stm32_hal stamps
 * every received frame (MACTSCR.TSENALL), so this works for the L2 transport as
 * well as the UDP ones. A general-port message (Announce, Follow_Up,
 * Delay_Resp) needs no ingress stamp and is passed to core/ptp with zero, which
 * ptp_port_rx() documents as the correct value for a non-event message.
 *
 * TX: two-step only (ptp.h scope note 3). Sync goes out with an estimate and
 * twoStepFlag set; the MAC's egress stamp comes back asynchronously through
 * sts_txts.c and is handed to ptp_on_sync_txts(), which releases the Follow_Up.
 * If the stamp never arrives the engine counts followup_missed and the Sync is
 * simply not followed up — which is the honest outcome, and visible.
 *
 * ---------------------------------------------------------------------------
 * Peer-delay
 * ---------------------------------------------------------------------------
 *
 * E2E only, so the pdelay multicast groups (224.0.0.107 / ff02::6b /
 * 01-80-C2-00-00-0E) are neither joined nor sent to. ptp_addr_hint_t still
 * carries PTP_ADDR_PDELAY for completeness; a descriptor naming it is dropped
 * here and counted, because emitting a Pdelay frame from an E2E-only engine
 * would be a bug worth seeing rather than a packet worth sending.
 *
 * ---------------------------------------------------------------------------
 * Unicast replies, and the requester table
 * ---------------------------------------------------------------------------
 *
 * The engine answers a unicast Delay_Req with PTP_ADDR_UNICAST_PEER and sets the
 * unicastFlag, exactly as §13.3.1 requires. It cannot address the reply itself:
 * ptp_tx_desc_t::peer is a PortIdentity (a clockIdentity plus a port number),
 * not a network address, because core/ptp is transport-agnostic by design.
 *
 * So this file keeps `peers[]`: the network source address of each
 * sourcePortIdentity that has recently sent us a Delay_Req, and honours the hint
 * against it. Ignoring the hint and multicasting the reply — which is what this
 * file used to do — was wrong twice over (F7): the requester never receives a
 * reply it can use, every other node on the segment receives a unicast-flagged
 * Delay_Resp it must discard, and N unicast requests per second become N
 * multicast frames per second flooding the whole link.
 *
 * The same table enforces logMinDelayReqInterval per requester (§9.5.11 lets the
 * responder ignore requests above the announced rate), so a single source cannot
 * turn its own request rate into our transmit rate.
 *
 * ---------------------------------------------------------------------------
 * Annex-P integrity (spec §4.4)
 * ---------------------------------------------------------------------------
 *
 * core/ptp's AUTHENTICATION-TLV engine is attached HERE and nowhere else: the
 * port calls ptp_icv_append() / ptp_icv_verify() unconditionally but only acts
 * on them once ptp_port_set_icv() has given it a context, and this file is the
 * only place that owns one. Without that binding the whole feature was
 * unreachable — see net/sts_ptp_icv_policy.h, which owns the arming decision.
 *
 * The context is created at start and re-planned on every CFG_G_PTP commit
 * (sts_ptp_reload_icv(), called from sts_net_on_cfg()), so an operator arms,
 * re-keys and disarms without a reboot — and so a factory reset's applier
 * fan-out overwrites the key held in RAM instead of leaving it there until the
 * reboot. Default is OFF: `ptp.icv.policy` ships 0.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/ptp_time.h>
#include <zephyr/net/socket.h>

#include <mbedtls/platform_util.h>

#include "cfg/cfg.h"
#include "net/sts_net.h"
#include "net/sts_ptp_icv_policy.h"
#include "net/sts_ptp_profile_policy.h"
#include "storage/sts_store.h"
#include "util/bytes.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_ptp, CONFIG_STS1000_LOG_LEVEL);

#define PTP_STACK_SIZE 4096
#define PTP_PRIORITY 5

#define PTP_EVENT_PORT 319
#define PTP_GENERAL_PORT 320

/** Engine step cadence. The shortest configurable interval is logSync -7
 *  (7.8 ms); 5 ms keeps the scheduler honest at that rate. */
#define PTP_STEP_MS 5

/**
 * PDUs handled per pass over one ready socket before returning to the loop.
 *
 * zsock_poll() returns immediately while data is queued and Zephyr does not
 * timeslice across priorities, so an uncapped drain lets a PTP flood keep this
 * priority-5 thread permanently runnable and starve `housekeeping` (14) — the
 * only caller of the watchdog kick, whose starvation cold-cycles the board
 * (F12). Small, because this thread is the highest-priority network service and
 * the engine step wants to run every 5 ms regardless.
 */
#define PTP_RX_BUDGET 8U

/** Requesters remembered for unicast replies and per-peer rate limiting. */
#define PTP_PEERS_MAX 16U

/* PTP common header offsets (IEEE 1588-2019 §13.3). */
#define PTP_OFF_MSGTYPE 0U
#define PTP_OFF_SRC_PORT_ID 20U
#define PTP_SRC_PORT_ID_LEN 10U
#define PTP_HDR_MIN_LEN 34U

static ptp_port_ctx_t port;
static ptp_cfg_t port_cfg;
static bool running;
static uint8_t transport;

/*
 * Annex-P integrity context.
 *
 * A file-scope object rather than an embedded ptp_port_ctx_t member because
 * ptp.h is explicit that a port which never uses integrity should not pay for
 * the key table and the scratch buffer. Only this file's engine_lock-holding
 * paths touch it: ptp_port_rx()/ptp_port_step() reach it through port.icv while
 * holding the lock, and sts_ptp_reload_icv() takes the same lock to re-plan it.
 */
static ptp_icv_ctx_t icv;
/** True once ptp_icv_init() has succeeded; the port may hold a pointer to it. */
static bool icv_ready;
/** True while ptp_port_ctx_t::icv points at @ref icv. */
static bool icv_attached;
/** Last planner verdict, for the log line and the published statistics. */
static uint8_t icv_state = (uint8_t)STS_PTP_ICV_OFF;

/**
 * One recent Delay_Req source.
 *
 * `id` is the raw 10 octets of the on-wire sourcePortIdentity, compared as
 * bytes: that avoids depending on ptp_port_id_t's in-memory padding and matches
 * exactly what the engine hands back in ptp_tx_desc_t::peer once encoded.
 */
static struct {
	uint8_t id[PTP_SRC_PORT_ID_LEN];
	struct sockaddr_storage addr; /* UDP transports */
	socklen_t addr_len;
	uint8_t mac[6];               /* L2 transport */
	uint64_t last_req_ms;         /* last request we *accepted* */
	uint64_t last_seen_ms;        /* for eviction */
	bool used;
	bool have_mac;
} peers[PTP_PEERS_MAX];
static struct k_mutex peers_lock;

/** Snapshot published by the PTP thread for the management threads to read. */
static sts_ptp_stats_t pub_stats;
static struct k_spinlock pub_lock;

static int ev_fd = -1;  /* event: 319, hardware-timestamped */
static int gen_fd = -1; /* general: 320 */
static int l2_fd = -1;  /* AF_PACKET, both message classes */

static struct in_addr mc4;
static struct in6_addr mc6;
static const uint8_t mc_l2[6] = { 0x01U, 0x1BU, 0x19U, 0x00U, 0x00U, 0x00U };

static uint8_t rxbuf[PTP_MSG_MAX_LEN + 32U];
static uint8_t l2frame[PTP_MSG_MAX_LEN + 14U];

static K_THREAD_STACK_DEFINE(ptp_stack, PTP_STACK_SIZE);
static struct k_thread ptp_thread;
static int live_id = -1;

static struct k_mutex engine_lock;

/* ------------------------------------------------------------------------- */
/* engine callbacks                                                          */
/* ------------------------------------------------------------------------- */

static int tai_ns_cb(void *ctx, uint64_t *out_ns)
{
	ARG_UNUSED(ctx);
	return sts_time_tai_ns(out_ns);
}

/* ------------------------------------------------------------------------- */
/* Annex-P integrity                                                         */
/* ------------------------------------------------------------------------- */

/**
 * Read the `ptp.icv.*` keys into a planner input.
 *
 * @p w is filled completely, key material included, so the caller MUST zeroize
 * it before it goes out of scope.
 */
static void icv_read_want(sts_ptp_icv_want_t *w)
{
	size_t klen = 0U;

	memset(w, 0, sizeof(*w));
	w->policy = (uint8_t)sts_net_cfg_u64(CFG_ID_PTP_ICV_POLICY, 0U);
	w->suite = (uint8_t)sts_net_cfg_u64(CFG_ID_PTP_ICV_SUITE, 0U);
	w->spp = (uint8_t)sts_net_cfg_u64(CFG_ID_PTP_ICV_SPP, 0U);
	w->key_id = (uint32_t)sts_net_cfg_u64(CFG_ID_PTP_ICV_KEY_ID, 0U);
	w->replay_protect = sts_net_cfg_bool(CFG_ID_PTP_ICV_REPLAY, true);
	w->replay_window = (uint8_t)sts_net_cfg_u64(CFG_ID_PTP_ICV_WINDOW, 16U);
	w->mask_mutable = sts_net_cfg_bool(CFG_ID_PTP_ICV_MASK_MUT, true);

	/*
	 * cfg_get_bytes() refuses any length past the caller's capacity, so an
	 * over-long blob leaves klen 0 (unarmed) rather than truncating the key
	 * to something that would authenticate nothing while looking installed.
	 */
	if (cfg_get_bytes(sts_cfg(), (uint16_t)CFG_ID_PTP_ICV_KEY, w->key,
			  sizeof(w->key), &klen) != 0) {
		klen = 0U;
	}
	w->key_len = klen;
}

/**
 * Re-plan the integrity engine and attach or detach it.
 *
 * Caller MUST hold @ref engine_lock: this rewrites the configuration the RX and
 * TX paths are reading through port.icv, and flips the pointer itself.
 *
 * @return true when the state changed and the caller should log it.
 */
static bool icv_apply_locked(void)
{
	sts_ptp_icv_want_t want;
	ptp_icv_cfg_t cfg;
	sts_ptp_icv_plan_t plan;
	uint8_t prev = icv_state;
	int rc;

	icv_read_want(&want);
	sts_ptp_icv_plan(&want, &cfg, &plan);
	mbedtls_platform_zeroize(&want, sizeof(want));

	if (icv_ready && (plan.attach == icv_attached) &&
	    (memcmp(&icv.cfg, &cfg, sizeof(cfg)) == 0)) {
		/*
		 * Nothing moved. Every CFG_G_PTP commit lands here, and most of
		 * them are about priorities or intervals — resetting the replay
		 * windows for those would make an unrelated edit briefly accept
		 * a replayed sequence number from every peer.
		 *
		 * The comparison is exact: both structures were produced by
		 * ptp_icv_cfg_defaults(), which memsets, so the padding is zero
		 * on both sides and memcmp() cannot report a spurious
		 * difference.
		 */
		mbedtls_platform_zeroize(&cfg, sizeof(cfg));
		icv_state = plan.state;
		return icv_state != prev;
	}

	if (!icv_ready) {
		const port_crypto_t *crypto = sts_port_crypto();

		/*
		 * First arm. ptp_icv_init() copies the configuration and the
		 * crypto vtable and zeroes everything else; it needs
		 * hmac_sha256, which portz_crypto.c always provides.
		 */
		if (crypto == NULL || crypto->hmac_sha256 == NULL) {
			mbedtls_platform_zeroize(&cfg, sizeof(cfg));
			if (plan.attach) {
				icv_state = (uint8_t)STS_PTP_ICV_REFUSED;
				LOG_ERR("Annex-P integrity needs HMAC-SHA-256 "
					"from the crypto port; staying off");
			}
			return icv_state != prev;
		}
		rc = ptp_icv_init(&icv, &cfg, crypto);
		if (rc != 0) {
			mbedtls_platform_zeroize(&cfg, sizeof(cfg));
			icv_state = (uint8_t)STS_PTP_ICV_REFUSED;
			LOG_ERR("ptp_icv_init: %d", rc);
			return icv_state != prev;
		}
		icv_ready = true;
	} else {
		/*
		 * Replace the configuration in place. This is also the
		 * zeroization step: ptp_icv_set_cfg() overwrites the whole
		 * ptp_icv_cfg_t, key table included, so a disarming plan
		 * (factory reset, emptied `ptp.icv.key`, refused edit) removes
		 * the RAM copy rather than leaving it until the reboot.
		 */
		rc = ptp_icv_set_cfg(&icv, &cfg);
		if (rc != 0) {
			mbedtls_platform_zeroize(&cfg, sizeof(cfg));
			/* Fail closed: detach rather than keep serving with a
			 * configuration the operator has replaced. */
			(void)ptp_port_set_icv(&port, NULL);
			icv_attached = false;
			icv_state = (uint8_t)STS_PTP_ICV_REFUSED;
			LOG_ERR("ptp_icv_set_cfg: %d; integrity detached", rc);
			return icv_state != prev;
		}
	}
	mbedtls_platform_zeroize(&cfg, sizeof(cfg));

	/*
	 * The configuration really changed, so every peer's replay history is
	 * stale: a re-key restarts sequence numbers, and an old window would
	 * reject the new association's first messages as replays.
	 */
	ptp_icv_reset_peers(&icv);

	(void)ptp_port_set_icv(&port, plan.attach ? &icv : NULL);
	icv_attached = plan.attach;
	icv_state = plan.state;
	return icv_state != prev;
}

/**
 * Re-read `ptp.icv.*` and apply it to the running engine.
 *
 * Called from the CFG_G_PTP applier (net/sts_net.c), i.e. on the committing
 * thread with the cfg mutex released. Safe before the PTP thread exists and
 * safe when PTP is disabled — both leave @ref running false and there is
 * nothing to reconfigure.
 */
void sts_ptp_reload_icv(void)
{
	bool changed;

	if (!running) {
		return;
	}

	k_mutex_lock(&engine_lock, K_FOREVER);
	changed = icv_apply_locked();
	k_mutex_unlock(&engine_lock);

	if (!changed) {
		return;
	}
	LOG_INF("Annex-P integrity: %s",
		sts_ptp_icv_state_name(icv_state));
	sts_log((uint8_t)LOGR_SUB_PTP, (uint8_t)LOGR_NOTICE,
		"PTP Annex-P integrity %s",
		sts_ptp_icv_state_name(icv_state));
}

/* ------------------------------------------------------------------------- */
/* requester table                                                           */
/* ------------------------------------------------------------------------- */

/** Encode a ptp_port_id_t the way §13.3 puts it on the wire. */
static void port_id_encode(const ptp_port_id_t *pid, uint8_t out[PTP_SRC_PORT_ID_LEN])
{
	memcpy(out, pid->clock_id.id, PTP_CLOCK_ID_LEN);
	bytes_put_be16(&out[PTP_CLOCK_ID_LEN], pid->port_number);
}

/** Minimum spacing between Delay_Reqs we will answer, from logMinDelayReq. */
static uint64_t delay_req_min_gap_ms(void)
{
	int8_t lg = port_cfg.log_min_delay_req_interval;

	if (lg >= 0) {
		if (lg > 10) {
			lg = 10; /* 1024 s; beyond this the shift is pointless */
		}
		return UINT64_C(1000) << (unsigned int)lg;
	}
	if (lg < -10) {
		lg = -10;
	}
	return UINT64_C(1000) >> (unsigned int)(-lg);
}

/** Find @p id, or claim a slot for it (evicting the least recently seen). */
static size_t peer_slot(const uint8_t id[PTP_SRC_PORT_ID_LEN], uint64_t now_ms)
{
	size_t free_slot = PTP_PEERS_MAX;
	size_t oldest = 0U;
	uint64_t oldest_ms = UINT64_MAX;
	size_t i;

	for (i = 0U; i < PTP_PEERS_MAX; i++) {
		if (peers[i].used &&
		    memcmp(peers[i].id, id, PTP_SRC_PORT_ID_LEN) == 0) {
			return i;
		}
		if (!peers[i].used) {
			if (free_slot == PTP_PEERS_MAX) {
				free_slot = i;
			}
		} else if (peers[i].last_seen_ms < oldest_ms) {
			oldest_ms = peers[i].last_seen_ms;
			oldest = i;
		}
	}

	i = (free_slot != PTP_PEERS_MAX) ? free_slot : oldest;
	memset(&peers[i], 0, sizeof(peers[i]));
	memcpy(peers[i].id, id, PTP_SRC_PORT_ID_LEN);
	peers[i].used = true;
	/* Fresh entry: no accepted request yet, so the first one is allowed. */
	peers[i].last_req_ms = 0U;
	peers[i].last_seen_ms = now_ms;
	return i;
}

/**
 * Record a Delay_Req source and decide whether to answer it.
 *
 * @return true when the request is inside the announced rate and the engine
 *         should see it.
 */
static bool peer_admit_delay_req(const uint8_t *pdu, size_t pdu_len,
				 const struct sockaddr *sa, socklen_t sa_len,
				 const uint8_t *src_mac, uint64_t now_ms)
{
	uint64_t gap = delay_req_min_gap_ms();
	bool admit;
	size_t i;

	if (pdu_len < PTP_HDR_MIN_LEN) {
		return false;
	}

	k_mutex_lock(&peers_lock, K_FOREVER);
	i = peer_slot(&pdu[PTP_OFF_SRC_PORT_ID], now_ms);
	peers[i].last_seen_ms = now_ms;

	if (sa != NULL && sa_len > 0U && sa_len <= sizeof(peers[i].addr)) {
		memcpy(&peers[i].addr, sa, sa_len);
		peers[i].addr_len = sa_len;
	}
	if (src_mac != NULL) {
		memcpy(peers[i].mac, src_mac, 6U);
		peers[i].have_mac = true;
	}

	/*
	 * §9.5.11: the responder may ignore a Delay_Req arriving faster than the
	 * interval it announced. Without this a single source's request rate is
	 * our transmit rate, which is a segment-wide amplifier when the reply is
	 * multicast and a CPU amplifier even when it is not (F7).
	 */
	admit = (peers[i].last_req_ms == 0U) ||
		((now_ms - peers[i].last_req_ms) >= gap);
	if (admit) {
		peers[i].last_req_ms = now_ms;
	}
	k_mutex_unlock(&peers_lock);

	return admit;
}

/**
 * Look up the network address last seen for @p pid.
 *
 * @retval 0        Found; @p out / @p out_len (or @p out_mac) are filled.
 * @retval -ENOENT  We have never had a request from that port identity.
 */
static int peer_lookup(const ptp_port_id_t *pid, struct sockaddr_storage *out,
		       socklen_t *out_len, uint8_t out_mac[6], bool *out_have_mac)
{
	uint8_t id[PTP_SRC_PORT_ID_LEN];
	int rc = -ENOENT;
	size_t i;

	port_id_encode(pid, id);

	k_mutex_lock(&peers_lock, K_FOREVER);
	for (i = 0U; i < PTP_PEERS_MAX; i++) {
		if (!peers[i].used ||
		    memcmp(peers[i].id, id, PTP_SRC_PORT_ID_LEN) != 0) {
			continue;
		}
		if (out != NULL && peers[i].addr_len > 0U) {
			memcpy(out, &peers[i].addr, peers[i].addr_len);
			*out_len = peers[i].addr_len;
		}
		if (out_mac != NULL) {
			memcpy(out_mac, peers[i].mac, 6U);
			*out_have_mac = peers[i].have_mac;
		}
		rc = 0;
		break;
	}
	k_mutex_unlock(&peers_lock);
	return rc;
}

/* ------------------------------------------------------------------------- */
/* transmit                                                                  */
/* ------------------------------------------------------------------------- */

static int tx_udp(const ptp_tx_desc_t *d)
{
	struct sockaddr_in a4;
	struct sockaddr_in6 a6;
	struct sockaddr_storage uni;
	socklen_t uni_len = 0U;
	struct sockaddr *sa;
	socklen_t slen;
	uint16_t dport = (d->port_kind == PTP_PORT_EVENT) ? PTP_EVENT_PORT
							  : PTP_GENERAL_PORT;
	int fd = (d->port_kind == PTP_PORT_EVENT) ? ev_fd : gen_fd;

	if (fd < 0) {
		return -ENOTCONN;
	}

	if (d->addr == PTP_ADDR_UNICAST_PEER) {
		/*
		 * The engine has set the unicastFlag; sending this to the
		 * multicast group would hand every node on the segment a reply
		 * addressed to somebody else and never reach the requester (F7).
		 * If the requester is not in the table there is nowhere to send
		 * it, and dropping is the only correct answer.
		 */
		if (peer_lookup(&d->peer, &uni, &uni_len, NULL, NULL) != 0 ||
		    uni_len == 0U) {
			LOG_WRN("no unicast address for the requesting port; "
				"dropping a %s reply", "Delay_Resp");
			return -EHOSTUNREACH;
		}
		if (uni.ss_family == AF_INET) {
			((struct sockaddr_in *)&uni)->sin_port = htons(dport);
		} else {
			((struct sockaddr_in6 *)&uni)->sin6_port = htons(dport);
		}
		if (zsock_sendto(fd, d->buf, d->len, 0, (struct sockaddr *)&uni,
				 uni_len) < 0) {
			return -errno;
		}
		return 0;
	}

	if (transport == (uint8_t)PTP_TRANSPORT_UDP_IPV4) {
		memset(&a4, 0, sizeof(a4));
		a4.sin_family = AF_INET;
		a4.sin_port = htons(dport);
		a4.sin_addr = mc4;
		sa = (struct sockaddr *)&a4;
		slen = sizeof(a4);
	} else {
		memset(&a6, 0, sizeof(a6));
		a6.sin6_family = AF_INET6;
		a6.sin6_port = htons(dport);
		a6.sin6_addr = mc6;
		sa = (struct sockaddr *)&a6;
		slen = sizeof(a6);
	}

	if (zsock_sendto(fd, d->buf, d->len, 0, sa, slen) < 0) {
		return -errno;
	}
	return 0;
}

static int tx_l2(const ptp_tx_desc_t *d)
{
	struct sockaddr_ll dst;
	const uint8_t *src = sts_net_mac();
	uint8_t peer_mac[6];
	bool have_mac = false;
	const uint8_t *dst_mac = mc_l2;

	if (l2_fd < 0) {
		return -ENOTCONN;
	}
	if (d->len + 14U > sizeof(l2frame)) {
		return -EMSGSIZE;
	}

	if (d->addr == PTP_ADDR_UNICAST_PEER) {
		if (peer_lookup(&d->peer, NULL, NULL, peer_mac, &have_mac) != 0 ||
		    !have_mac) {
			LOG_WRN("no unicast MAC for the requesting port; "
				"dropping a %s reply", "Delay_Resp");
			return -EHOSTUNREACH;
		}
		dst_mac = peer_mac;
	}

	memcpy(&l2frame[0], dst_mac, 6U);
	memcpy(&l2frame[6], src, 6U);
	bytes_put_be16(&l2frame[12], NET_ETH_PTYPE_PTP);
	memcpy(&l2frame[14], d->buf, d->len);

	memset(&dst, 0, sizeof(dst));
	dst.sll_family = AF_PACKET;
	dst.sll_protocol = htons(NET_ETH_PTYPE_PTP);
	dst.sll_ifindex = net_if_get_by_iface(sts_net_iface());
	dst.sll_halen = 6U;
	memcpy(dst.sll_addr, dst_mac, 6U);

	if (zsock_sendto(l2_fd, l2frame, d->len + 14U, 0,
			 (struct sockaddr *)&dst, sizeof(dst)) < 0) {
		return -errno;
	}
	return 0;
}

static int tx_cb(void *ctx, const ptp_tx_desc_t *d)
{
	ARG_UNUSED(ctx);

	if (d->addr == PTP_ADDR_PDELAY) {
		/* E2E-only engine: see the peer-delay note at the top. */
		LOG_WRN("dropping a pdelay-addressed PDU (E2E only)");
		return -ENOTSUP;
	}

	if (transport == (uint8_t)PTP_TRANSPORT_L2) {
		return tx_l2(d);
	}
	return tx_udp(d);
}

/** Called from the net_if TX-timestamp thread; releases the Follow_Up. */
static void on_tx_timestamp(uint8_t msg_type, uint16_t seq, uint64_t tai_ns)
{
	if (msg_type != (uint8_t)PTP_MSG_SYNC) {
		return;
	}
	/*
	 * ptp_on_sync_txts() transmits the Follow_Up from inside this call, so
	 * it must hold the same lock the service thread uses around
	 * ptp_port_step()/ptp_port_rx(). This callback runs on the cooperative
	 * tx_tstamp thread, so the wait can only ever be for a preempted
	 * priority-5 thread and is bounded by one step.
	 */
	if (k_mutex_lock(&engine_lock, K_MSEC(50)) != 0) {
		return;
	}
	(void)ptp_on_sync_txts(&port, seq, tai_ns);
	k_mutex_unlock(&engine_lock);
}

/* ------------------------------------------------------------------------- */
/* sockets                                                                   */
/* ------------------------------------------------------------------------- */

static void enable_rx_timestamps(int fd)
{
	uint8_t flags = SOF_TIMESTAMPING_RX_HARDWARE |
			SOF_TIMESTAMPING_TX_HARDWARE;

	if (zsock_setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPING, &flags,
			     sizeof(flags)) < 0) {
		LOG_WRN("SO_TIMESTAMPING on the PTP socket: %d", errno);
	}
}

static int open_udp(uint16_t port_no, bool event)
{
	struct sockaddr_in a4;
	struct sockaddr_in6 a6;
	struct sockaddr *sa;
	socklen_t slen;
	sa_family_t fam = (transport == (uint8_t)PTP_TRANSPORT_UDP_IPV4)
				  ? AF_INET
				  : AF_INET6;
	int fd;

	fd = zsock_socket(fam, SOCK_DGRAM, IPPROTO_UDP);
	if (fd < 0) {
		return -errno;
	}

	if (fam == AF_INET) {
		memset(&a4, 0, sizeof(a4));
		a4.sin_family = AF_INET;
		a4.sin_port = htons(port_no);
		a4.sin_addr.s_addr = INADDR_ANY;
		sa = (struct sockaddr *)&a4;
		slen = sizeof(a4);
	} else {
		memset(&a6, 0, sizeof(a6));
		a6.sin6_family = AF_INET6;
		a6.sin6_port = htons(port_no);
		a6.sin6_addr = in6addr_any;
		sa = (struct sockaddr *)&a6;
		slen = sizeof(a6);
	}

	if (zsock_bind(fd, sa, slen) < 0) {
		int rc = -errno;

		(void)zsock_close(fd);
		return rc;
	}

	if (fam == AF_INET) {
		struct ip_mreqn mreq;
		int ttl = 1; /* §Annex C.5: PTP multicast does not leave the link */

		memset(&mreq, 0, sizeof(mreq));
		mreq.imr_multiaddr = mc4;
		mreq.imr_ifindex = net_if_get_by_iface(sts_net_iface());
		if (zsock_setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq,
				     sizeof(mreq)) < 0) {
			LOG_WRN("IP_ADD_MEMBERSHIP(:%u): %d", port_no, errno);
		}
		(void)zsock_setsockopt(fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl,
				       sizeof(ttl));
	} else {
		struct ipv6_mreq mreq6;
		int hops = 1;

		memset(&mreq6, 0, sizeof(mreq6));
		mreq6.ipv6mr_multiaddr = mc6;
		mreq6.ipv6mr_ifindex = net_if_get_by_iface(sts_net_iface());
		if (zsock_setsockopt(fd, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP,
				     &mreq6, sizeof(mreq6)) < 0) {
			LOG_WRN("IPV6_ADD_MEMBERSHIP(:%u): %d", port_no, errno);
		}
		(void)zsock_setsockopt(fd, IPPROTO_IPV6, IPV6_UNICAST_HOPS,
				       &hops, sizeof(hops));
	}

	if (event) {
		enable_rx_timestamps(fd);
	}
	return fd;
}

static int open_l2(void)
{
	struct sockaddr_ll bind_addr;
	int fd;

	fd = zsock_socket(AF_PACKET, SOCK_RAW, htons(NET_ETH_PTYPE_PTP));
	if (fd < 0) {
		return -errno;
	}

	memset(&bind_addr, 0, sizeof(bind_addr));
	bind_addr.sll_family = AF_PACKET;
	bind_addr.sll_protocol = htons(NET_ETH_PTYPE_PTP);
	bind_addr.sll_ifindex = net_if_get_by_iface(sts_net_iface());

	if (zsock_bind(fd, (struct sockaddr *)&bind_addr,
		       sizeof(bind_addr)) < 0) {
		int rc = -errno;

		(void)zsock_close(fd);
		return rc;
	}

	enable_rx_timestamps(fd);
	return fd;
}

/* ------------------------------------------------------------------------- */
/* receive                                                                   */
/* ------------------------------------------------------------------------- */

static uint64_t rx_stamp_from_msg(const struct msghdr *msg)
{
	struct cmsghdr *cm;

	for (cm = CMSG_FIRSTHDR(msg); cm != NULL;
	     cm = CMSG_NXTHDR((struct msghdr *)msg, cm)) {
		struct net_ptp_time ts;

		if (cm->cmsg_level != SOL_SOCKET ||
		    cm->cmsg_type != SO_TIMESTAMPING) {
			continue;
		}
		if (cm->cmsg_len < CMSG_LEN(sizeof(ts))) {
			continue;
		}
		memcpy(&ts, CMSG_DATA(cm), sizeof(ts));
		return sts_ptpclk_ts_to_tai_ns(ts.second, ts.nanosecond);
	}
	return 0U;
}

/** @return true when a PDU was read, false when the socket ran dry. */
static bool rx_one(int fd, bool event, bool l2)
{
	struct sockaddr_storage peer;
	struct iovec iov;
	struct msghdr msg;
	uint8_t cbuf[CMSG_SPACE(sizeof(struct net_ptp_time))];
	const uint8_t *pdu;
	const uint8_t *src_mac = NULL;
	size_t pdu_len;
	uint64_t rx_ts = 0U;
	uint64_t now_ms;
	ssize_t n;

	memset(&msg, 0, sizeof(msg));
	iov.iov_base = rxbuf;
	iov.iov_len = sizeof(rxbuf);
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_name = &peer;
	msg.msg_namelen = sizeof(peer);
	msg.msg_control = cbuf;
	msg.msg_controllen = sizeof(cbuf);

	n = zsock_recvmsg(fd, &msg, 0);
	if (n <= 0) {
		return false;
	}

	pdu = rxbuf;
	pdu_len = (size_t)n;

	if (l2) {
		/* SOCK_RAW hands back the whole frame, Ethernet header and all;
		 * our own multicast comes back too and core/ptp filters it by
		 * sourcePortIdentity (counters.rx_self). */
		if (pdu_len <= 14U) {
			return true;
		}
		if (bytes_get_be16(&rxbuf[12]) != NET_ETH_PTYPE_PTP) {
			return true;
		}
		src_mac = &rxbuf[6]; /* the frame's source address */
		pdu += 14U;
		pdu_len -= 14U;
		/* On L2 the message class is in the PDU, not in a port number. */
		event = ((pdu[0] & 0x0FU) == (uint8_t)PTP_MSG_SYNC) ||
			((pdu[0] & 0x0FU) == (uint8_t)PTP_MSG_DELAY_REQ) ||
			((pdu[0] & 0x0FU) == (uint8_t)PTP_MSG_PDELAY_REQ) ||
			((pdu[0] & 0x0FU) == (uint8_t)PTP_MSG_PDELAY_RESP);
	}

	now_ms = sts_mono_ms();

	/*
	 * A Delay_Req is the only message that makes us transmit on demand, so it
	 * is the only one that needs an address remembered and a rate enforced.
	 * Both happen before the engine sees the PDU: a request over the announced
	 * rate is simply never presented (F7).
	 */
	if (pdu_len >= PTP_HDR_MIN_LEN &&
	    (pdu[PTP_OFF_MSGTYPE] & 0x0FU) == (uint8_t)PTP_MSG_DELAY_REQ) {
		if (!peer_admit_delay_req(pdu, pdu_len,
					  l2 ? NULL : (struct sockaddr *)&peer,
					  l2 ? 0U : msg.msg_namelen, src_mac,
					  now_ms)) {
			return true;
		}
	}

	if (event) {
		rx_ts = rx_stamp_from_msg(&msg);
		if (rx_ts == 0U) {
			uint64_t now = 0U;

			/* Software fallback: worse, but a Delay_Req answered
			 * with a slightly late receipt timestamp is far better
			 * for the client than no Delay_Resp at all. */
			if (sts_time_tai_ns(&now) == 0) {
				rx_ts = now;
			}
		}
	}

	k_mutex_lock(&engine_lock, K_FOREVER);
	(void)ptp_port_rx(&port, pdu, pdu_len, rx_ts, now_ms);
	k_mutex_unlock(&engine_lock);
	return true;
}

/* ------------------------------------------------------------------------- */
/* quality refresh                                                           */
/* ------------------------------------------------------------------------- */

static void refresh_quality(void)
{
	quality_block_t q;
	ptp_quality_view_t v;
	uint64_t tai_ns = 0U;

	if (sts_quality_snapshot(&q) != 0) {
		return;
	}
	(void)sts_time_tai_ns(&tai_ns);

	if (ptp_quality_view_from_block(&q, tai_ns / UINT64_C(1000000000),
					&v) != 0) {
		return;
	}

	/*
	 * The §3.8 block describes the discipline loop, which locks the
	 * oscillator's *rate* to the PPS. It says nothing about whether the MAC
	 * counter every Sync and Follow_Up timestamp is read from has been placed
	 * on TAI, and those are separate mechanisms on this board. With the
	 * counter unplaced the block still reports LOCKED, so this port announced
	 * clockClass 6 with timeTraceable and currentUtcOffsetValid set while
	 * emitting 1900-era originTimestamps — and, being class 6, won the BMCA
	 * outright against every honest clock on the segment (F1).
	 *
	 * So an untraceable timescale is forced to the free-running rung and every
	 * derived claim withdrawn: no traceability, no accuracy or variance
	 * estimate (both describe a placed clock), and no leap announcement (a
	 * clock that does not know the date cannot know when a leap falls).
	 * currentUtcOffset is left as core computed it — TAI − UTC really is known
	 * from GNSS independently of whether the counter was placed.
	 */
	if (!sts_ptpclk_traceable()) {
		v.sync_state = PTP_SYNC_FREERUN;
		v.time_traceable = false;
		v.est_accuracy_ns = 0U;
		v.adev_tau1_e18 = 0U;
		v.leap61 = false;
		v.leap59 = false;
	}

	k_mutex_lock(&engine_lock, K_FOREVER);
	(void)ptp_port_set_quality(&port, &v);
	k_mutex_unlock(&engine_lock);
}

/* ------------------------------------------------------------------------- */
/* published statistics                                                      */
/* ------------------------------------------------------------------------- */

/**
 * Copy the engine's counters and quality into pub_stats.
 *
 * Called from the PTP thread while it holds @ref engine_lock, which is what
 * makes the copy consistent. The management threads then read the snapshot under
 * a spinlock instead of taking engine_lock themselves — reading `port.quality`
 * and the counter array unlocked was a torn read (F16), and taking the engine
 * mutex from SNMP or MCP would violate the §1.2 rule that no management thread
 * may hold a lock on timing state.
 */
static void publish_stats_locked(void)
{
	const ptp_counters_t *c = ptp_port_counters(&port);
	ptp_clock_quality_t cq;
	sts_ptp_stats_t s;

	memset(&s, 0, sizeof(s));
	s.running = true;
	if (c != NULL) {
		s.counters = *c;
	}
	s.port_state = (uint8_t)ptp_port_state(&port);
	s.alarms = ptp_port_alarms(&port);
	s.domain = port_cfg.domain;
	s.transport = transport;

	/*
	 * Annex-P integrity. The counters are the only way an operator can tell
	 * "no peer signs" from "every peer signs and every ICV fails", and the
	 * two look identical from the port's own counters (rx_icv_rejected
	 * covers all of it). Copied here under engine_lock with everything else.
	 */
	s.icv_state = icv_state;
	if (icv_ready) {
		const ptp_icv_counters_t *ic = ptp_icv_counters(&icv);

		if (ic != NULL) {
			s.icv = *ic;
		}
	}

	memset(&cq, 0, sizeof(cq));
	if (ptp_clock_quality_from_view(&port_cfg, &port.quality, &cq) == 0) {
		s.clock_class = cq.clock_class;
		s.clock_accuracy = cq.clock_accuracy;
	}

	K_SPINLOCK(&pub_lock) {
		pub_stats = s;
	}
}

/* ------------------------------------------------------------------------- */
/* the loop                                                                  */
/* ------------------------------------------------------------------------- */

static void ptp_loop(void *a, void *b, void *c)
{
	uint64_t last_quality_ms = 0U;

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	running = true;

	k_mutex_lock(&engine_lock, K_FOREVER);
	(void)ptp_port_enable(&port, sts_mono_ms());
	k_mutex_unlock(&engine_lock);

	for (;;) {
		struct zsock_pollfd fds[2];
		uint64_t now;
		int nfds = 0;
		int rc;

		if (l2_fd >= 0) {
			fds[nfds].fd = l2_fd;
			fds[nfds].events = ZSOCK_POLLIN;
			fds[nfds].revents = 0;
			nfds++;
		} else {
			if (ev_fd >= 0) {
				fds[nfds].fd = ev_fd;
				fds[nfds].events = ZSOCK_POLLIN;
				fds[nfds].revents = 0;
				nfds++;
			}
			if (gen_fd >= 0) {
				fds[nfds].fd = gen_fd;
				fds[nfds].events = ZSOCK_POLLIN;
				fds[nfds].revents = 0;
				nfds++;
			}
		}

		if (nfds == 0) {
			k_sleep(K_SECONDS(1));
			continue;
		}

		rc = zsock_poll(fds, nfds, PTP_STEP_MS);
		if (rc > 0) {
			int i;

			for (i = 0; i < nfds; i++) {
				unsigned int n;

				if ((fds[i].revents & ZSOCK_POLLIN) == 0) {
					continue;
				}
				/* Bounded drain; see PTP_RX_BUDGET. */
				for (n = 0U; n < PTP_RX_BUDGET; n++) {
					if (!rx_one(fds[i].fd,
						    fds[i].fd == ev_fd,
						    fds[i].fd == l2_fd)) {
						break;
					}
				}
			}
		}

		now = sts_mono_ms();
		if (now - last_quality_ms >= 1000U) {
			last_quality_ms = now;
			refresh_quality();
		}

		k_mutex_lock(&engine_lock, K_FOREVER);
		(void)ptp_port_step(&port, now);
		publish_stats_locked();
		k_mutex_unlock(&engine_lock);

		if (live_id >= 0) {
			sts_liveness_feed(live_id);
		}

		/*
		 * This is the highest-priority network thread; without an explicit
		 * yield a sustained PTP flood keeps it runnable and starves
		 * `housekeeping`, whose starvation withholds the watchdog kick and
		 * cold-cycles the board (F12).
		 */
		k_yield();
	}
}

/* ------------------------------------------------------------------------- */
/* public                                                                    */
/* ------------------------------------------------------------------------- */

void sts_ptp_stats(sts_ptp_stats_t *out)
{
	if (out == NULL) {
		return;
	}
	memset(out, 0, sizeof(*out));
	if (!running) {
		return;
	}
	/* The snapshot the PTP thread publishes under engine_lock; see
	 * publish_stats_locked(). */
	K_SPINLOCK(&pub_lock) {
		*out = pub_stats;
	}
}

/*
 * A key's schema default, so the profile/operator pick below compares against
 * the schema rather than against a literal repeated at the call site. A missing
 * row cannot happen for a compiled-in CFG_ID_*, but returning a value the stored
 * one will differ from is the safe way to be wrong: the operator's value wins,
 * which is what the old code did unconditionally.
 */
static uint64_t schema_def_u64(uint16_t id)
{
	const cfg_key_t *k = cfg_key_find(id);

	return (k != NULL) ? k->def.u : UINT64_MAX;
}

static int32_t schema_def_i32(uint16_t id)
{
	const cfg_key_t *k = cfg_key_find(id);

	return (k != NULL) ? k->def.i : INT32_MIN;
}

int sts_ptp_start(void)
{
	uint8_t requested_profile;
	int rc;

	if (!sts_net_cfg_bool(CFG_ID_PTP_ENABLE, true)) {
		LOG_INF("PTP grandmaster disabled by configuration");
		return 0;
	}

	k_mutex_init(&engine_lock);
	k_mutex_init(&peers_lock);

	ptp_cfg_defaults(&port_cfg);

	/*
	 * The profile BEFORE the individual keys, because a profile is a
	 * parameter set and not a label — see net/sts_ptp_profile_policy.h for
	 * what this used to do instead and why the result was a Default-profile
	 * grandmaster wearing a telecom profile's name.
	 */
	requested_profile = sts_ptp_prof_clamp(
		sts_net_cfg_u64(CFG_ID_PTP_PROFILE, (uint64_t)PTP_PROFILE_DEFAULT),
		(uint8_t)PTP_PROFILE_COUNT, (uint8_t)PTP_PROFILE_DEFAULT);
	(void)ptp_cfg_apply_profile(&port_cfg, requested_profile);

	/*
	 * Then the operator on top. sts_ptp_prof_pick_*() takes the stored value
	 * when it differs from the key's schema default and the profile's
	 * otherwise; the header states what that costs. The schema default is
	 * read from the schema rather than repeated here, so adding a key or
	 * changing a default cannot leave a second copy behind.
	 */
	port_cfg.domain = (uint8_t)sts_ptp_prof_pick_u64(
		sts_net_cfg_u64(CFG_ID_PTP_DOMAIN, port_cfg.domain),
		schema_def_u64(CFG_ID_PTP_DOMAIN), port_cfg.domain);
	port_cfg.priority1 = (uint8_t)sts_ptp_prof_pick_u64(
		sts_net_cfg_u64(CFG_ID_PTP_PRIORITY1, port_cfg.priority1),
		schema_def_u64(CFG_ID_PTP_PRIORITY1), port_cfg.priority1);
	port_cfg.priority2 = (uint8_t)sts_ptp_prof_pick_u64(
		sts_net_cfg_u64(CFG_ID_PTP_PRIORITY2, port_cfg.priority2),
		schema_def_u64(CFG_ID_PTP_PRIORITY2), port_cfg.priority2);
	port_cfg.log_announce_interval = (int8_t)sts_ptp_prof_pick_i32(
		sts_net_cfg_i32(CFG_ID_PTP_LOG_ANNOUNCE,
				port_cfg.log_announce_interval),
		schema_def_i32(CFG_ID_PTP_LOG_ANNOUNCE),
		port_cfg.log_announce_interval);
	port_cfg.log_sync_interval = (int8_t)sts_ptp_prof_pick_i32(
		sts_net_cfg_i32(CFG_ID_PTP_LOG_SYNC, port_cfg.log_sync_interval),
		schema_def_i32(CFG_ID_PTP_LOG_SYNC), port_cfg.log_sync_interval);
	port_cfg.log_min_delay_req_interval = (int8_t)sts_ptp_prof_pick_i32(
		sts_net_cfg_i32(CFG_ID_PTP_LOG_DELAYREQ,
				port_cfg.log_min_delay_req_interval),
		schema_def_i32(CFG_ID_PTP_LOG_DELAYREQ),
		port_cfg.log_min_delay_req_interval);

	transport = (uint8_t)sts_ptp_prof_pick_u64(
		sts_net_cfg_u64(CFG_ID_PTP_TRANSPORT, (uint64_t)port_cfg.transport),
		schema_def_u64(CFG_ID_PTP_TRANSPORT), (uint64_t)port_cfg.transport);
	if (transport >= (uint8_t)PTP_TRANSPORT_COUNT) {
		transport = (uint8_t)PTP_TRANSPORT_UDP_IPV4;
	}
	port_cfg.transport = (ptp_transport_t)transport;

	rc = ptp_cfg_validate(&port_cfg);
	if (rc != 0) {
		/*
		 * Name the profile before ptp_cfg_defaults() erases it. The
		 * fallback is unavoidable — the engine will not start on a
		 * configuration it rejected — but "your G.8275.1 selection was
		 * discarded" is the sentence an operator needs, and the old code
		 * only said "falling back to defaults".
		 */
		if (sts_ptp_prof_report_discard(requested_profile,
						(uint8_t)PTP_PROFILE_DEFAULT)) {
			LOG_ERR("ptp cfg rejected (%d) for profile %s: the "
				"profile selection is DISCARDED and this unit "
				"is running the Default profile",
				rc, ptp_profile_name(requested_profile));
		} else {
			LOG_ERR("ptp cfg rejected (%d); falling back to defaults",
				rc);
		}
		ptp_cfg_defaults(&port_cfg);
		transport = (uint8_t)port_cfg.transport;
	}

	{
		ptp_port_ops_t ops = {
			.tx = tx_cb,
			.tai_ns = tai_ns_cb,
			.ctx = NULL,
		};

		rc = ptp_port_init(&port, &port_cfg, sts_net_mac(), &ops);
		if (rc != 0) {
			LOG_ERR("ptp_port_init: %d", rc);
			return rc;
		}
	}

	/*
	 * Arm (or leave off) Annex-P integrity before the thread exists, so the
	 * very first Announce is signed if the operator asked for that. No lock
	 * is strictly needed here — nothing else can reach the port yet — but
	 * taking it keeps one code path for start and for reload.
	 */
	k_mutex_lock(&engine_lock, K_FOREVER);
	(void)icv_apply_locked();
	k_mutex_unlock(&engine_lock);
	LOG_INF("Annex-P integrity: %s", sts_ptp_icv_state_name(icv_state));

	(void)net_addr_pton(AF_INET, "224.0.1.129", &mc4);
	(void)net_addr_pton(AF_INET6, "ff0e::181", &mc6);

	if (transport == (uint8_t)PTP_TRANSPORT_L2) {
		l2_fd = open_l2();
		if (l2_fd < 0) {
			LOG_ERR("AF_PACKET socket: %d; falling back to UDPv4",
				l2_fd);
			l2_fd = -1;
			transport = (uint8_t)PTP_TRANSPORT_UDP_IPV4;
			port_cfg.transport = PTP_TRANSPORT_UDP_IPV4;
		}
	}

	if (transport != (uint8_t)PTP_TRANSPORT_L2) {
		ev_fd = open_udp(PTP_EVENT_PORT, true);
		gen_fd = open_udp(PTP_GENERAL_PORT, false);
		if (ev_fd < 0 || gen_fd < 0) {
			LOG_ERR("PTP sockets: event=%d general=%d", ev_fd,
				gen_fd);
			if (ev_fd >= 0) {
				(void)zsock_close(ev_fd);
				ev_fd = -1;
			}
			if (gen_fd >= 0) {
				(void)zsock_close(gen_fd);
				gen_fd = -1;
			}
			return -ENOTCONN;
		}
	}

	sts_txts_set_ptp(on_tx_timestamp);
	live_id = sts_liveness_register("ptp");

	k_thread_create(&ptp_thread, ptp_stack, K_THREAD_STACK_SIZEOF(ptp_stack),
			ptp_loop, NULL, NULL, NULL, PTP_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&ptp_thread, "ptp");

	LOG_INF("PTP grandmaster: domain %u, transport %u, two-step",
		port_cfg.domain, transport);
	return 0;
}
