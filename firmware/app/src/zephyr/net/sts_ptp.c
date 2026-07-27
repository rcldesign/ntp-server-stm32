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

#include "cfg/cfg.h"
#include "net/sts_net.h"
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

static ptp_port_ctx_t port;
static ptp_cfg_t port_cfg;
static bool running;
static uint8_t transport;

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

static int tx_udp(const ptp_tx_desc_t *d)
{
	struct sockaddr_in a4;
	struct sockaddr_in6 a6;
	struct sockaddr *sa;
	socklen_t slen;
	uint16_t dport = (d->port_kind == PTP_PORT_EVENT) ? PTP_EVENT_PORT
							  : PTP_GENERAL_PORT;
	int fd = (d->port_kind == PTP_PORT_EVENT) ? ev_fd : gen_fd;

	if (fd < 0) {
		return -ENOTCONN;
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

	if (l2_fd < 0) {
		return -ENOTCONN;
	}
	if (d->len + 14U > sizeof(l2frame)) {
		return -EMSGSIZE;
	}

	memcpy(&l2frame[0], mc_l2, sizeof(mc_l2));
	memcpy(&l2frame[6], src, 6U);
	bytes_put_be16(&l2frame[12], NET_ETH_PTYPE_PTP);
	memcpy(&l2frame[14], d->buf, d->len);

	memset(&dst, 0, sizeof(dst));
	dst.sll_family = AF_PACKET;
	dst.sll_protocol = htons(NET_ETH_PTYPE_PTP);
	dst.sll_ifindex = net_if_get_by_iface(sts_net_iface());
	dst.sll_halen = 6U;
	memcpy(dst.sll_addr, mc_l2, sizeof(mc_l2));

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

static void rx_one(int fd, bool event, bool l2)
{
	struct sockaddr_storage peer;
	struct iovec iov;
	struct msghdr msg;
	uint8_t cbuf[CMSG_SPACE(sizeof(struct net_ptp_time))];
	const uint8_t *pdu;
	size_t pdu_len;
	uint64_t rx_ts = 0U;
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
		return;
	}

	pdu = rxbuf;
	pdu_len = (size_t)n;

	if (l2) {
		/* SOCK_RAW hands back the whole frame, Ethernet header and all;
		 * our own multicast comes back too and core/ptp filters it by
		 * sourcePortIdentity (counters.rx_self). */
		if (pdu_len <= 14U) {
			return;
		}
		if (bytes_get_be16(&rxbuf[12]) != NET_ETH_PTYPE_PTP) {
			return;
		}
		pdu += 14U;
		pdu_len -= 14U;
		/* On L2 the message class is in the PDU, not in a port number. */
		event = ((pdu[0] & 0x0FU) == (uint8_t)PTP_MSG_SYNC) ||
			((pdu[0] & 0x0FU) == (uint8_t)PTP_MSG_DELAY_REQ) ||
			((pdu[0] & 0x0FU) == (uint8_t)PTP_MSG_PDELAY_REQ) ||
			((pdu[0] & 0x0FU) == (uint8_t)PTP_MSG_PDELAY_RESP);
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
	(void)ptp_port_rx(&port, pdu, pdu_len, rx_ts, sts_mono_ms());
	k_mutex_unlock(&engine_lock);
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

	k_mutex_lock(&engine_lock, K_FOREVER);
	(void)ptp_port_set_quality(&port, &v);
	k_mutex_unlock(&engine_lock);
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
				if ((fds[i].revents & ZSOCK_POLLIN) == 0) {
					continue;
				}
				rx_one(fds[i].fd, fds[i].fd == ev_fd,
				       fds[i].fd == l2_fd);
			}
		}

		now = sts_mono_ms();
		if (now - last_quality_ms >= 1000U) {
			last_quality_ms = now;
			refresh_quality();
		}

		k_mutex_lock(&engine_lock, K_FOREVER);
		(void)ptp_port_step(&port, now);
		k_mutex_unlock(&engine_lock);

		if (live_id >= 0) {
			sts_liveness_feed(live_id);
		}
	}
}

/* ------------------------------------------------------------------------- */
/* public                                                                    */
/* ------------------------------------------------------------------------- */

void sts_ptp_stats(sts_ptp_stats_t *out)
{
	const ptp_counters_t *c;
	ptp_clock_quality_t cq;

	if (out == NULL) {
		return;
	}
	memset(out, 0, sizeof(*out));
	out->running = running;
	if (!running) {
		return;
	}

	c = ptp_port_counters(&port);
	if (c != NULL) {
		out->counters = *c;
	}
	out->port_state = (uint8_t)ptp_port_state(&port);
	out->alarms = ptp_port_alarms(&port);
	out->domain = port_cfg.domain;
	out->transport = transport;

	memset(&cq, 0, sizeof(cq));
	if (ptp_clock_quality_from_view(&port_cfg, &port.quality, &cq) == 0) {
		out->clock_class = cq.clock_class;
		out->clock_accuracy = cq.clock_accuracy;
	}
}

int sts_ptp_start(void)
{
	int rc;

	if (!sts_net_cfg_bool(CFG_ID_PTP_ENABLE, true)) {
		LOG_INF("PTP grandmaster disabled by configuration");
		return 0;
	}

	k_mutex_init(&engine_lock);

	ptp_cfg_defaults(&port_cfg);
	port_cfg.domain = (uint8_t)sts_net_cfg_u64(CFG_ID_PTP_DOMAIN, 0U);
	port_cfg.priority1 = (uint8_t)sts_net_cfg_u64(CFG_ID_PTP_PRIORITY1, 128U);
	port_cfg.priority2 = (uint8_t)sts_net_cfg_u64(CFG_ID_PTP_PRIORITY2, 128U);
	port_cfg.log_announce_interval =
		(int8_t)sts_net_cfg_i32(CFG_ID_PTP_LOG_ANNOUNCE, 1);
	port_cfg.log_sync_interval =
		(int8_t)sts_net_cfg_i32(CFG_ID_PTP_LOG_SYNC, 0);
	port_cfg.log_min_delay_req_interval =
		(int8_t)sts_net_cfg_i32(CFG_ID_PTP_LOG_DELAYREQ, 0);

	transport = (uint8_t)sts_net_cfg_u64(CFG_ID_PTP_TRANSPORT,
					     (uint64_t)PTP_TRANSPORT_UDP_IPV6);
	if (transport >= (uint8_t)PTP_TRANSPORT_COUNT) {
		transport = (uint8_t)PTP_TRANSPORT_UDP_IPV4;
	}
	port_cfg.transport = (ptp_transport_t)transport;

	port_cfg.profile =
		(ptp_profile_t)sts_net_cfg_u64(CFG_ID_PTP_PROFILE,
					       (uint64_t)PTP_PROFILE_DEFAULT);
	if ((unsigned int)port_cfg.profile >= (unsigned int)PTP_PROFILE_COUNT) {
		port_cfg.profile = PTP_PROFILE_DEFAULT;
	}

	rc = ptp_cfg_validate(&port_cfg);
	if (rc != 0) {
		LOG_ERR("ptp cfg rejected (%d); falling back to defaults", rc);
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
