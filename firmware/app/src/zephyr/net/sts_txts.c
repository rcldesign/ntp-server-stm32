/*
 * STS1000 "Meridian" — hardware TX-timestamp demultiplexer.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * ---------------------------------------------------------------------------
 * Why this file exists
 * ---------------------------------------------------------------------------
 *
 * Zephyr's socket layer takes SO_TIMESTAMPING and turns SOF_TIMESTAMPING_TX_HARDWARE
 * into net_pkt_set_tx_timestamping() on the outgoing packet
 * (subsys/net/ip/net_context.c). eth_stm32_hal then asks the MAC to capture the
 * egress stamp and hands the finished packet to net_if_add_tx_timestamp(). What
 * Zephyr does *not* have is Linux's MSG_ERRQUEUE: there is no path that returns
 * the stamp to the socket that sent the datagram. The only delivery mechanism is
 * net_if_register_timestamp_cb(), which is per-`struct net_pkt` or global — and
 * a `sendto()` caller never sees the net_pkt the stack allocated for it.
 *
 * So this file registers ONE global callback and demultiplexes by looking at
 * the bytes that were actually transmitted. Each consumer identifies its own
 * frame by something already on the wire:
 *
 *   PTP  messageType + sequenceId from the PTP common header (IEEE 1588-2019
 *        §13.3), which is exactly what ptp_on_sync_txts() matches on.
 *   NTP  the 64-bit transmit-timestamp field of the response. Interleaved mode
 *        (draft-ietf-ntp-interleaved-modes) needs the *measured* egress stamp of
 *        the previous response paired with the client it went to, and the value
 *        this firmware wrote into that field is a unique, already-transmitted
 *        token that maps back to it.
 *
 * Running in the tx_tstamp thread
 * -------------------------------
 * net_if's TX-timestamp thread is cooperative at K_PRIO_COOP(1), i.e. above
 * every application thread including `ptp` (5) and `ntp_server` (8). The
 * consumer callbacks below therefore must not block and must not do real work;
 * both of them only stamp a value into a small lock-protected structure that
 * their own thread picks up.
 *
 * Deliberate limits
 * -----------------
 *   - No 802.1Q. A VLAN tag shifts every offset by four octets. net.vlan.enable
 *     is reboot-required config that nothing in this wave acts on, so a tagged
 *     frame is counted as malformed rather than mis-parsed.
 *     TODO(vlan): fold the tag length into frame_offsets() when VLAN lands.
 *   - No IPv6 extension headers. A PTP or NTP datagram this firmware emits
 *     never carries one; anything that does is not ours.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>

#include "net/sts_net.h"
#include "util/bytes.h"

LOG_MODULE_REGISTER(sts_txts, CONFIG_STS1000_LOG_LEVEL);

/* Enough for Ethernet + IPv6 + UDP + the NTP transmit field (offset 40..47). */
#define SNIFF_MAX 112U

#define ETH_HDR_LEN 14U
#define ETHERTYPE_IPV4 0x0800U
#define ETHERTYPE_VLAN 0x8100U
#define ETHERTYPE_IPV6 0x86DDU
#define ETHERTYPE_PTP 0x88F7U

#define IPPROTO_UDP_V 17U

#define PTP_EVENT_PORT 319U
#define NTP_PORT 123U

/* PTP common header (IEEE 1588-2019 §13.3). */
#define PTP_HDR_MIN 34U
#define PTP_OFF_SEQ 30U

/* NTP header (RFC 5905 §7.3). */
#define NTP_HDR_MIN 48U
#define NTP_OFF_XMT 40U

static struct net_if_timestamp_cb cb_handle;
static sts_txts_ptp_fn ptp_fn;
static sts_txts_ntp_fn ntp_fn;
static struct k_spinlock lock;

static struct {
	uint32_t total;
	uint32_t ptp;
	uint32_t ntp;
	uint32_t unmatched;
	uint32_t malformed;
} st;

void sts_txts_set_ptp(sts_txts_ptp_fn fn)
{
	K_SPINLOCK(&lock) {
		ptp_fn = fn;
	}
}

void sts_txts_set_ntp(sts_txts_ntp_fn fn)
{
	K_SPINLOCK(&lock) {
		ntp_fn = fn;
	}
}

void sts_txts_stats(sts_txts_stats_t *out)
{
	if (out == NULL) {
		return;
	}
	K_SPINLOCK(&lock) {
		out->total = st.total;
		out->ptp = st.ptp;
		out->ntp = st.ntp;
		out->unmatched = st.unmatched;
		out->malformed = st.malformed;
	}
}

/** What the frame turned out to be. */
typedef enum {
	FRAME_OTHER = 0,
	FRAME_PTP_EVENT,
	FRAME_NTP,
	FRAME_BAD,
} frame_kind_t;

/**
 * Classify a transmitted frame and report where its payload starts.
 *
 * @param buf   Leading @p len octets of the frame, from the Ethernet header.
 * @param off   Receives the payload offset (PTP or NTP header start).
 */
static frame_kind_t classify(const uint8_t *buf, size_t len, size_t *off)
{
	uint16_t ethertype;
	size_t p;
	uint16_t sport;
	uint16_t dport;

	if (len < ETH_HDR_LEN + 4U) {
		return FRAME_BAD;
	}

	ethertype = bytes_get_be16(&buf[12]);
	p = ETH_HDR_LEN;

	if (ethertype == ETHERTYPE_VLAN) {
		return FRAME_BAD; /* see the VLAN note in the file comment */
	}

	if (ethertype == ETHERTYPE_PTP) {
		if (len < p + PTP_HDR_MIN) {
			return FRAME_BAD;
		}
		*off = p;
		return FRAME_PTP_EVENT;
	}

	if (ethertype == ETHERTYPE_IPV4) {
		size_t ihl;

		if (len < p + 20U) {
			return FRAME_BAD;
		}
		if ((buf[p] >> 4) != 4U) {
			return FRAME_BAD;
		}
		ihl = (size_t)(buf[p] & 0x0FU) * 4U;
		if (ihl < 20U || len < p + ihl + 8U) {
			return FRAME_BAD;
		}
		if (buf[p + 9U] != IPPROTO_UDP_V) {
			return FRAME_OTHER;
		}
		p += ihl;
	} else if (ethertype == ETHERTYPE_IPV6) {
		if (len < p + 40U + 8U) {
			return FRAME_BAD;
		}
		if (buf[p + 6U] != IPPROTO_UDP_V) {
			return FRAME_OTHER;
		}
		p += 40U;
	} else {
		return FRAME_OTHER;
	}

	sport = bytes_get_be16(&buf[p]);
	dport = bytes_get_be16(&buf[p + 2U]);
	p += 8U;

	if (sport == PTP_EVENT_PORT || dport == PTP_EVENT_PORT) {
		if (len < p + PTP_HDR_MIN) {
			return FRAME_BAD;
		}
		*off = p;
		return FRAME_PTP_EVENT;
	}
	if (sport == NTP_PORT) {
		if (len < p + NTP_HDR_MIN) {
			return FRAME_BAD;
		}
		*off = p;
		return FRAME_NTP;
	}

	return FRAME_OTHER;
}

static void on_tx_timestamp(struct net_pkt *pkt)
{
	uint8_t buf[SNIFF_MAX];
	struct net_pkt_cursor backup;
	struct net_ptp_time *ts;
	sts_txts_ptp_fn pfn;
	sts_txts_ntp_fn nfn;
	size_t len;
	size_t off = 0U;
	uint64_t tai_ns;
	frame_kind_t kind;

	K_SPINLOCK(&lock) {
		st.total++;
	}

	ts = net_pkt_timestamp(pkt);
	if (ts == NULL) {
		K_SPINLOCK(&lock) {
			st.malformed++;
		}
		return;
	}
	tai_ns = sts_ptpclk_ts_to_tai_ns(ts->second, ts->nanosecond);
	if (tai_ns == 0U) {
		/* The MAC did not stamp this frame after all. */
		K_SPINLOCK(&lock) {
			st.malformed++;
		}
		return;
	}

	len = net_pkt_get_len(pkt);
	if (len > sizeof(buf)) {
		len = sizeof(buf);
	}

	net_pkt_cursor_backup(pkt, &backup);
	net_pkt_cursor_init(pkt);
	if (net_pkt_read(pkt, buf, len) != 0) {
		net_pkt_cursor_restore(pkt, &backup);
		K_SPINLOCK(&lock) {
			st.malformed++;
		}
		return;
	}
	net_pkt_cursor_restore(pkt, &backup);

	kind = classify(buf, len, &off);

	K_SPINLOCK(&lock) {
		pfn = ptp_fn;
		nfn = ntp_fn;
	}

	switch (kind) {
	case FRAME_PTP_EVENT:
		if (pfn != NULL) {
			pfn((uint8_t)(buf[off] & 0x0FU),
			    bytes_get_be16(&buf[off + PTP_OFF_SEQ]), tai_ns);
			K_SPINLOCK(&lock) {
				st.ptp++;
			}
		} else {
			K_SPINLOCK(&lock) {
				st.unmatched++;
			}
		}
		break;
	case FRAME_NTP:
		if (nfn != NULL) {
			nfn(bytes_get_be64(&buf[off + NTP_OFF_XMT]), tai_ns);
			K_SPINLOCK(&lock) {
				st.ntp++;
			}
		} else {
			K_SPINLOCK(&lock) {
				st.unmatched++;
			}
		}
		break;
	case FRAME_BAD:
		K_SPINLOCK(&lock) {
			st.malformed++;
		}
		break;
	case FRAME_OTHER:
	default:
		K_SPINLOCK(&lock) {
			st.unmatched++;
		}
		break;
	}
}

int sts_txts_init(void)
{
	struct net_if *iface = sts_net_iface();

	if (iface == NULL) {
		return -ENODEV;
	}

	/* pkt == NULL means "every packet"; iface pins it to our MAC so a
	 * second interface (none today) could not feed this demultiplexer. */
	net_if_register_timestamp_cb(&cb_handle, NULL, iface, on_tx_timestamp);
	LOG_INF("TX-timestamp callback registered");
	return 0;
}
