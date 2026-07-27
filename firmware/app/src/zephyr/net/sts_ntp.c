/*
 * STS1000 "Meridian" — NTP/NTS server thread (spec §4.1–§4.3, priority 8).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * ---------------------------------------------------------------------------
 * Shape
 * ---------------------------------------------------------------------------
 *
 * One thread, two UDP sockets (v4 and v6, both on port 123), a zsock_poll()
 * loop, and core/ntp doing all of the protocol. The glue's whole job is:
 *
 *   1. get a hardware receive timestamp onto ntp_rx_t.rx_tai_ns,
 *   2. project the §3.8 quality block onto ntp_quality_view_t,
 *   3. call ntp_handle_request(), send what it produced,
 *   4. feed the hardware *egress* timestamp back with ntp_tx_complete() so the
 *      next interleaved request can be answered precisely.
 *
 * Two sockets rather than one v6 socket with V6ONLY off: a dual-stack socket
 * hands v4 peers back as v4-mapped v6 addresses, which would have to be
 * un-mapped again to derive a stable rate-limit key, and a mapped address is a
 * classic place to get an ACL or a bucket wrong. Two sockets keep each family's
 * address in its own type all the way to the client-id hash.
 *
 * ---------------------------------------------------------------------------
 * Timestamping — what the hardware actually gives us, precisely
 * ---------------------------------------------------------------------------
 *
 * RECEIVE. eth_stm32_hal enables MACTSCR.TSENALL, so the MAC stamps *every*
 * received frame, not just PTP ones, and the driver copies the stamp into
 * net_pkt->timestamp and sets net_pkt_set_rx_timestamping(). Zephyr's socket
 * layer surfaces it as a SOL_SOCKET/SO_TIMESTAMPING control message carrying a
 * `struct net_ptp_time`, provided CONFIG_NET_CONTEXT_TIMESTAMPING is on and the
 * socket asked for SOF_TIMESTAMPING_RX_HARDWARE. So RX is a true MAC-level
 * hardware timestamp, taken in the same counter the responses are built from.
 * The driver marks "no stamp" as second==UINT64_MAX; that case is counted in
 * rx_no_hw_ts and falls back to reading the PTP clock in this thread, which
 * costs the scheduling latency between the packet arriving and this loop
 * running (tens of microseconds under load) — precision the client can see, so
 * the counter is exported rather than hidden.
 *
 * TRANSMIT. SOF_TIMESTAMPING_TX_HARDWARE makes the stack set
 * net_pkt_set_tx_timestamping(), which makes eth_stm32_hal call
 * HAL_ETH_PTP_InsertTxTimestamp() and later net_if_add_tx_timestamp(). But
 * Zephyr has no MSG_ERRQUEUE, so the stamp never comes back through the socket.
 * It arrives asynchronously on the global net_if callback that sts_txts.c owns,
 * some time *after* sendto() has returned. That is why:
 *
 *   - the transmit-timestamp field of a basic-mode response is a *software*
 *     estimate: the PTP clock read immediately before sendto(). Everything
 *     after that read — the socket call, the IP/UDP encapsulation, the DMA and
 *     the PHY — is unmeasured, so a basic-mode response carries roughly the
 *     stack's egress latency of error, tens of microseconds;
 *   - interleaved mode (draft-ietf-ntp-interleaved-modes) is where the hardware
 *     stamp is actually used, and it is exact. tx_pending[] below remembers the
 *     transmit field this server wrote, sts_txts.c matches the transmitted
 *     bytes back to it, and ntp_tx_complete() arms the client's next request to
 *     be answered with the real egress instant.
 *
 * That asymmetry is inherent to the platform, not a shortcut: a one-shot server
 * response cannot contain its own egress timestamp on any hardware. It is the
 * reason interleaved mode exists, and the reason it is on by default here.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/ptp_time.h>
#include <zephyr/net/socket.h>

#include "cfg/cfg.h"
#include "net/sts_net.h"
#include "storage/sts_store.h"
#include "util/crc.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_ntp, CONFIG_STS1000_LOG_LEVEL);

#define NTP_UDP_PORT 123
#define NTP_STACK_SIZE 4096
#define NTP_PRIORITY 8

/**
 * Clock precision advertised in the header, as log2 seconds.
 *
 * -20 is 954 ns. The PTP counter itself resolves 4 ns (250 MHz HCLK), but
 * RFC 5905 §7.3 defines precision as the precision of the *system clock as the
 * server can read it*, and the honest limit here is the RX-stamp-to-response
 * path, not the counter. Claiming -24 would be advertising the counter's
 * resolution as if it were the server's accuracy.
 */
#define NTP_PRECISION_LOG2 (-20)

/**
 * Datagrams handled per pass over one ready socket before yielding.
 *
 * zsock_poll() returns immediately while anything is queued, and Zephyr gives
 * no timeslice across priorities, so an uncapped inner loop turns a UDP flood
 * into a permanently-runnable priority-8 thread that starves `housekeeping`
 * (priority 14) — the only caller of the external watchdog kick. Starving it
 * withholds the kick, the TPS3430 asserts WDO_N, and POE_KILL cold-cycles the
 * board, which the attacker simply repeats (F12). Draining a batch keeps the
 * good case fast; yielding between batches keeps the box alive.
 */
#define NTP_RX_BUDGET 16U

/**
 * Outstanding responses awaiting their hardware egress timestamp.
 *
 * The stamp comes back on the net_if TX-timestamp thread within a frame time or
 * two, so this only has to cover responses in flight. Eight silently
 * overwrote live entries at any real query rate; 32 covers the ~10 k req/s
 * design target with margin, entries expire by age rather than by being
 * clobbered, and the overrun is counted instead of hidden (F5).
 */
#define TX_PENDING_SLOTS 32U

/** A pending entry older than this will never be matched; reclaim it. */
#define TX_PENDING_TTL_MS 250U

/* Static: ntp_ctx_t carries a 256-way client table and is far too large to
 * live on a 4 kB thread stack. */
static ntp_ctx_t ntp;
static nts_ctx_t nts;
static ntp_ext_hook_t nts_hook;
static bool nts_enabled;

static uint8_t rx_buf[NTP_PKT_MAX];
static uint8_t tx_buf[NTP_PKT_MAX];

static int sock4 = -1;
static int sock6 = -1;

static struct {
	uint64_t xmt_field;    /* on-wire transmit field: the demux key */
	uint64_t added_ms;     /* monotonic ms the entry was armed */
	uint32_t client_id;
	uint32_t xl_token;     /* RFC 9769 interleave pairing token (core) */
	uint32_t seq;          /* arming order, for oldest-first eviction */
	int32_t tai_minus_utc; /* the offset the response was built with */
	bool used;
} tx_pending[TX_PENDING_SLOTS];
static uint32_t tx_pending_seq;
static struct k_spinlock tx_lock;

static struct {
	uint32_t rx_no_hw_ts;
	uint32_t tx_errors;
	uint32_t txts_matched;
	uint32_t txts_ambiguous; /* stamps discarded: the key was not unique */
	uint32_t tx_pending_overrun; /* entries evicted before their stamp landed */
	bool running;
} gstat;

static K_THREAD_STACK_DEFINE(ntp_stack, NTP_STACK_SIZE);
static struct k_thread ntp_thread;
static int live_id = -1;

/* ------------------------------------------------------------------------- */
/* configuration                                                             */
/* ------------------------------------------------------------------------- */

static void load_cfg(ntp_cfg_t *c)
{
	uint64_t qps;
	uint64_t burst;

	ntp_cfg_default(c);

	/* ntp.rate.qps == 0 means "no per-client limit" in the schema, which is
	 * exactly what core/ntp's client_rate == 0 means. */
	qps = sts_net_cfg_u64(CFG_ID_NTP_RATE_QPS, 0U);
	burst = sts_net_cfg_u64(CFG_ID_NTP_RATE_BURST, 8U);
	c->client_rate = (uint32_t)qps;
	c->client_burst = (uint32_t)burst;

	c->kod_on_limit = sts_net_cfg_bool(CFG_ID_NTP_KOD_ENABLE, true);
	c->interleave = sts_net_cfg_bool(CFG_ID_NTP_INTERLEAVED, true);
}

/* ------------------------------------------------------------------------- */
/* quality projection                                                        */
/* ------------------------------------------------------------------------- */

/**
 * Project the §3.8 block onto the NTP header view.
 *
 * The mapping itself lives in core/ntp (ntp_quality_view_from_block), so it is
 * under host test rather than hand-rolled here. This function's own job is to
 * supply the three inputs core cannot reach — the snapshot, the current time,
 * and whether the served timescale is actually traceable.
 *
 * That last input is the important one. `disc` publishing stratum 1 says the
 * oscillator is locked to the PPS; it does not say the MAC counter every
 * timestamp is read from has been placed on TAI, and on this board those are
 * separate mechanisms. With the counter unplaced this server previously
 * answered LI=0 / stratum 1 / refid 'GPS' carrying a 1900-era instant (F1).
 * sts_time_is_fallback() did not catch it: it reports only whether a TAI source
 * was ever registered, and one is registered from boot.
 */
static void quality_view(ntp_quality_view_t *v)
{
	quality_block_t q;
	uint64_t now_tai_ns = 0U;

	ntp_quality_view_default(v);
	v->precision = NTP_PRECISION_LOG2;

	if (sts_quality_snapshot(&q) != 0) {
		return;
	}
	(void)sts_time_tai_ns(&now_tai_ns);

	(void)ntp_quality_view_from_block(&q, now_tai_ns, sts_mono_ms(),
					  sts_ptpclk_traceable() &&
						  !sts_time_is_fallback(),
					  NTP_PRECISION_LOG2, v);
}

/* ------------------------------------------------------------------------- */
/* client identity                                                           */
/* ------------------------------------------------------------------------- */

/**
 * Rate-limit and interleave key.
 *
 * core/ntp requires this to come from the source address alone — never from
 * packet contents, or a client could pick its own bucket. Port is deliberately
 * excluded: a client that re-binds between polls must keep its bucket and its
 * interleave state, and including the port would also let one host multiply its
 * allowance by opening sockets.
 *
 * The derivation is ntp_client_id(), a *keyed* hash. It used to be an unkeyed
 * CRC-32, which for a 16-octet IPv6 address made a collision with a chosen
 * victim solvable rather than searchable — anyone with a routable /64 could pick
 * a source sharing a specific customer's token bucket and interleave state and
 * drain it into Kiss-o'-Death (F8). See ntp_client_id() for why salting the
 * table slot alone did not help.
 */
static uint32_t client_id_of(const struct sockaddr *sa)
{
	if (sa->sa_family == AF_INET) {
		const struct sockaddr_in *s4 = (const struct sockaddr_in *)sa;

		return ntp_client_id(&ntp, &s4->sin_addr, sizeof(s4->sin_addr));
	}
	if (sa->sa_family == AF_INET6) {
		const struct sockaddr_in6 *s6 = (const struct sockaddr_in6 *)sa;

		return ntp_client_id(&ntp, &s6->sin6_addr, sizeof(s6->sin6_addr));
	}
	return 0U;
}

/* ------------------------------------------------------------------------- */
/* TX timestamp feedback                                                     */
/* ------------------------------------------------------------------------- */

/**
 * Arm the egress-timestamp feedback for one response.
 *
 * The key is the on-wire transmit field, because that is the only per-response
 * value sts_txts.c can see in the transmitted bytes. core/ntp guarantees the
 * field is distinct across consecutive responses (NTP_XMT_DISTINCT_WINDOW), so
 * a duplicate here means something upstream broke that guarantee. It is
 * *rejected* rather than allowed to shadow the live entry: two entries under one
 * key are indistinguishable, and pairing the wrong one reports one client's
 * transmit instant to another as its own t3 (F5). Losing an interleave
 * opportunity is the correct failure.
 */
static void tx_pending_add(uint64_t xmt_field, uint32_t client_id,
			   uint32_t xl_token, int32_t tai_minus_utc)
{
	uint64_t now_ms = sts_mono_ms();
	bool dup = false;

	if (xmt_field == 0U || xl_token == 0U) {
		return;
	}
	K_SPINLOCK(&tx_lock) {
		size_t slot = TX_PENDING_SLOTS;
		size_t oldest = 0U;
		uint32_t oldest_seq = UINT32_MAX;
		size_t i;

		for (i = 0U; i < TX_PENDING_SLOTS; i++) {
			if (tx_pending[i].used &&
			    (now_ms - tx_pending[i].added_ms) >
				    TX_PENDING_TTL_MS) {
				/* Its stamp is never coming; reclaim quietly. */
				tx_pending[i].used = false;
			}
			if (tx_pending[i].used &&
			    tx_pending[i].xmt_field == xmt_field) {
				tx_pending[i].used = false;
				dup = true;
			}
			if (!tx_pending[i].used) {
				if (slot == TX_PENDING_SLOTS) {
					slot = i;
				}
			} else if (tx_pending[i].seq < oldest_seq) {
				oldest_seq = tx_pending[i].seq;
				oldest = i;
			}
		}

		if (dup) {
			/* Neither response can be identified any more. */
			gstat.txts_ambiguous++;
		} else {
			if (slot == TX_PENDING_SLOTS) {
				slot = oldest;
				gstat.tx_pending_overrun++;
			}
			tx_pending[slot].xmt_field = xmt_field;
			tx_pending[slot].added_ms = now_ms;
			tx_pending[slot].client_id = client_id;
			tx_pending[slot].xl_token = xl_token;
			tx_pending[slot].tai_minus_utc = tai_minus_utc;
			tx_pending[slot].seq = ++tx_pending_seq;
			tx_pending[slot].used = true;
		}
	}
}

/**
 * Called from the net_if TX-timestamp thread (cooperative, above every service
 * thread). Does the minimum: match the key, hand the measured stamp to
 * core/ntp, clear the slot.
 */
static void on_tx_timestamp(uint64_t xmt_field, uint64_t tai_ns)
{
	uint32_t client_id = 0U;
	uint32_t xl_token = 0U;
	int32_t tai_minus_utc = 0;
	unsigned int matches = 0U;
	size_t i;

	K_SPINLOCK(&tx_lock) {
		size_t hit = TX_PENDING_SLOTS;

		/*
		 * Scan the whole table and count, rather than taking the first
		 * hit. The token guard in ntp_tx_complete() cannot detect a
		 * mispair, because both the client id and the token are read out
		 * of whichever slot matched — a wrong slot yields a *consistent*
		 * pair and is accepted. So ambiguity has to be resolved here, and
		 * the only safe resolution is to discard the measurement (F5).
		 */
		for (i = 0U; i < TX_PENDING_SLOTS; i++) {
			if (tx_pending[i].used &&
			    tx_pending[i].xmt_field == xmt_field) {
				hit = i;
				matches++;
			}
		}
		if (matches == 1U) {
			client_id = tx_pending[hit].client_id;
			xl_token = tx_pending[hit].xl_token;
			tai_minus_utc = tx_pending[hit].tai_minus_utc;
			tx_pending[hit].used = false;
		} else if (matches > 1U) {
			for (i = 0U; i < TX_PENDING_SLOTS; i++) {
				if (tx_pending[i].used &&
				    tx_pending[i].xmt_field == xmt_field) {
					tx_pending[i].used = false;
				}
			}
			gstat.txts_ambiguous++;
		}
	}

	if (matches != 1U) {
		return;
	}

	/*
	 * The measured egress instant must be reported on the same UTC-based
	 * NTP timescale the response's transmit field used, i.e. with the same
	 * TAI-UTC offset (ntp_ts_from_tai subtracts it). Passing 0 would shift
	 * the interleaved reply by the whole leap-second offset.
	 *
	 * The RFC 9769 pairing token (res.xl_token) is passed straight through so
	 * core can reject a response a later request already superseded. Note it
	 * is *not* protection against a wrong wire-match — see the scan above.
	 *
	 * ntp_tx_complete() only touches that client's cached interleave state
	 * and takes no lock of its own; the NTP thread is the only other writer
	 * and this callback runs cooperatively above it, so the update cannot
	 * interleave with a request being handled.
	 */
	if (ntp_tx_complete(&ntp, client_id, xl_token,
			    ntp_ts_from_tai((int64_t)tai_ns, tai_minus_utc)) ==
	    0) {
		gstat.txts_matched++;
	}
}

/* ------------------------------------------------------------------------- */
/* sockets                                                                   */
/* ------------------------------------------------------------------------- */

static int open_socket(sa_family_t family)
{
	struct sockaddr_in a4;
	struct sockaddr_in6 a6;
	struct sockaddr *sa;
	socklen_t slen;
	uint8_t ts_flags = SOF_TIMESTAMPING_RX_HARDWARE |
			   SOF_TIMESTAMPING_TX_HARDWARE;
	int fd;

	fd = zsock_socket(family, SOCK_DGRAM, IPPROTO_UDP);
	if (fd < 0) {
		LOG_ERR("socket(%d): %d", family, errno);
		return -errno;
	}

	if (family == AF_INET) {
		memset(&a4, 0, sizeof(a4));
		a4.sin_family = AF_INET;
		a4.sin_port = htons(NTP_UDP_PORT);
		a4.sin_addr.s_addr = INADDR_ANY;
		sa = (struct sockaddr *)&a4;
		slen = sizeof(a4);
	} else {
		memset(&a6, 0, sizeof(a6));
		a6.sin6_family = AF_INET6;
		a6.sin6_port = htons(NTP_UDP_PORT);
		a6.sin6_addr = in6addr_any;
		sa = (struct sockaddr *)&a6;
		slen = sizeof(a6);
	}

	if (zsock_bind(fd, sa, slen) < 0) {
		LOG_ERR("bind(%d): %d", family, errno);
		(void)zsock_close(fd);
		return -errno;
	}

	/* Not fatal: without it the server still answers, with software
	 * timestamps and the precision penalty counted in rx_no_hw_ts. */
	if (zsock_setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPING, &ts_flags,
			     sizeof(ts_flags)) < 0) {
		LOG_WRN("SO_TIMESTAMPING unavailable on the %s socket: %d",
			(family == AF_INET) ? "v4" : "v6", errno);
	}

	return fd;
}

/* ------------------------------------------------------------------------- */
/* the loop                                                                  */
/* ------------------------------------------------------------------------- */

/** Pull the hardware RX stamp out of the control messages, or 0. */
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

/** @return true when a datagram was handled, false when the socket ran dry. */
static bool serve_one(int fd)
{
	struct sockaddr_storage peer;
	struct iovec iov;
	struct msghdr msg;
	uint8_t cbuf[CMSG_SPACE(sizeof(struct net_ptp_time))];
	ntp_quality_view_t qv;
	ntp_result_t res;
	ntp_rx_t rx;
	uint64_t tai_ns = 0U;
	ssize_t n;
	int rc;

	memset(&msg, 0, sizeof(msg));
	iov.iov_base = rx_buf;
	iov.iov_len = sizeof(rx_buf);
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

	memset(&rx, 0, sizeof(rx));
	rx.pkt = rx_buf;
	rx.len = (size_t)n;
	rx.client_id = client_id_of((const struct sockaddr *)&peer);
	rx.now_ms = (int64_t)sts_mono_ms();

	rx.rx_tai_ns = (int64_t)rx_stamp_from_msg(&msg);
	if (rx.rx_tai_ns == 0) {
		gstat.rx_no_hw_ts++;
		if (sts_time_tai_ns(&tai_ns) == 0) {
			rx.rx_tai_ns = (int64_t)tai_ns;
		}
	}

	/* Read the clock as late as possible: everything between here and the
	 * MAC is the unmeasured part of a basic-mode response. */
	if (sts_time_tai_ns(&tai_ns) == 0) {
		rx.tx_tai_ns = (int64_t)tai_ns;
	} else {
		rx.tx_tai_ns = rx.rx_tai_ns;
	}

	quality_view(&qv);

	rc = ntp_handle_request(&ntp, &rx, &qv, tx_buf, sizeof(tx_buf), &res);
	if (rc != 0 || res.action == NTP_ACT_IGNORE) {
		return true;
	}

	if (zsock_sendto(fd, tx_buf, res.len, 0, (struct sockaddr *)&peer,
			 msg.msg_namelen) < 0) {
		gstat.tx_errors++;
		return true;
	}

	/* Arm the interleave feedback only when core armed a pairing (xl_token
	 * != 0): a KoD, or interleave-disabled, carries no token. */
	if (res.action == NTP_ACT_RESPOND) {
		tx_pending_add(res.xmt, rx.client_id, res.xl_token,
			       qv.tai_minus_utc);
	}
	return true;
}

static void ntp_loop(void *a, void *b, void *c)
{
	struct zsock_pollfd fds[2];

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	gstat.running = true;

	for (;;) {
		int nfds = 0;
		int rc;

		if (sock4 >= 0) {
			fds[nfds].fd = sock4;
			fds[nfds].events = ZSOCK_POLLIN;
			fds[nfds].revents = 0;
			nfds++;
		}
		if (sock6 >= 0) {
			fds[nfds].fd = sock6;
			fds[nfds].events = ZSOCK_POLLIN;
			fds[nfds].revents = 0;
			nfds++;
		}
		if (nfds == 0) {
			k_sleep(K_SECONDS(1));
			continue;
		}

		/* The 500 ms cap is the liveness heartbeat, not a timeout the
		 * protocol needs: the supervisor must see this thread even on a
		 * silent network. */
		rc = zsock_poll(fds, nfds, 500);
		if (rc > 0) {
			int i;

			for (i = 0; i < nfds; i++) {
				unsigned int n;

				if ((fds[i].revents & ZSOCK_POLLIN) == 0) {
					continue;
				}
				/* Drain a bounded batch, then fall out to the
				 * liveness feed and the yield below. See
				 * NTP_RX_BUDGET. */
				for (n = 0U; n < NTP_RX_BUDGET; n++) {
					if (!serve_one(fds[i].fd)) {
						break;
					}
				}
			}
		}

		if (nts_enabled) {
			(void)nts_keyring_tick(sts_net_keyring(),
					       (int64_t)sts_mono_ms());
		}

		if (live_id >= 0) {
			sts_liveness_feed(live_id);
		}

		/*
		 * Give every equal- and lower-priority thread a turn, whatever the
		 * ingress rate. Zephyr does not timeslice across priorities, so
		 * without this a sustained flood keeps this thread runnable and
		 * `housekeeping` — the only caller of the watchdog kick — never
		 * runs (F12).
		 */
		k_yield();
	}
}

/* ------------------------------------------------------------------------- */
/* public                                                                    */
/* ------------------------------------------------------------------------- */

void sts_ntp_stats(sts_ntp_stats_t *out)
{
	if (out == NULL) {
		return;
	}
	memset(out, 0, sizeof(*out));
	(void)ntp_stats_get(&ntp, &out->ntp);
	if (nts_enabled) {
		(void)nts_stats_get(&nts, &out->nts);
	}
	out->rx_no_hw_ts = gstat.rx_no_hw_ts;
	out->tx_errors = gstat.tx_errors;
	out->txts_matched = gstat.txts_matched;
	out->txts_ambiguous = gstat.txts_ambiguous;
	out->tx_pending_overrun = gstat.tx_pending_overrun;
	out->nts_enabled = nts_enabled;
	out->running = gstat.running;
}

int sts_ntp_start(void)
{
	ntp_cfg_t cfg;
	int rc;

	if (!sts_net_cfg_bool(CFG_ID_NTP_ENABLE, true)) {
		LOG_INF("NTP server disabled by configuration");
		return 0;
	}

	load_cfg(&cfg);
	rc = ntp_init(&ntp, &cfg, sts_port_crypto(), (int64_t)sts_mono_ms());
	if (rc != 0) {
		LOG_ERR("ntp_init: %d", rc);
		return rc;
	}

	/*
	 * The NTS datapath is only worth arming when a client can actually obtain
	 * a cookie, and the only way to get one is the NTS-KE key exchange. That
	 * is compiled out unless mbedTLS carries the RFC 8446 exporter (see
	 * sts_ntske.c and conf/net.conf), so enabling the datapath regardless left
	 * the appliance accepting and NAKing NTS packets that no client could ever
	 * have been issued a cookie for, while `nts.enable` and the status blob both
	 * claimed NTS was on (F13). Advertise what the build can do.
	 */
	if (sts_net_keyring() != NULL && sts_ntske_supported()) {
		rc = nts_init(&nts, sts_net_keyring(), NTS_COOKIES_MAX);
		if (rc == 0) {
			nts_hook.build = nts_ntp_ext_build;
			nts_hook.ctx = &nts;
			(void)ntp_set_ext_hook(&ntp, &nts_hook);
			nts_enabled = true;
		} else {
			LOG_ERR("nts_init: %d; serving plain NTP only", rc);
		}
	} else if (sts_net_keyring() != NULL) {
		LOG_WRN("NTS datapath off: this build has no NTS-KE key exchange "
			"(mbedTLS lacks MBEDTLS_SSL_KEYING_MATERIAL_EXPORT), so no "
			"client can obtain a cookie; serving plain NTP only");
	}

	sock4 = open_socket(AF_INET);
	sock6 = open_socket(AF_INET6);
	if (sock4 < 0 && sock6 < 0) {
		LOG_ERR("no NTP socket could be opened");
		return -ENOTCONN;
	}

	sts_txts_set_ntp(on_tx_timestamp);

	live_id = sts_liveness_register("ntp");

	k_thread_create(&ntp_thread, ntp_stack, K_THREAD_STACK_SIZEOF(ntp_stack),
			ntp_loop, NULL, NULL, NULL, NTP_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&ntp_thread, "ntp_server");

	LOG_INF("NTP server on :%d (v4=%s v6=%s), NTS %s", NTP_UDP_PORT,
		(sock4 >= 0) ? "up" : "down", (sock6 >= 0) ? "up" : "down",
		nts_enabled ? "on" : "off");
	return 0;
}
