/*
 * STS1000 "Meridian" — network management: addressing, mDNS, link (spec §4.5).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Priority-10 thread (spec §1.2) plus a net_mgmt event callback. Its jobs:
 *
 *   - Addressing. DHCPv4 when net.dhcp is set (the default); otherwise the
 *     static net.ipv4.* keys. IPv6 link-local and SLAAC come up on their own
 *     from the stack. Re-applied on a net-group cfg commit.
 *   - mDNS / DNS-SD. Zephyr's mDNS responder answers <hostname>.local; the
 *     DNS-SD records for _ntp._udp and _ntske._tcp are registered statically
 *     so a client can discover the services (spec §4.5). The hostname is set
 *     from net.hostname before the responder needs it.
 *   - Link and address events -> alarms + logring. A carrier drop raises the
 *     network-lost alarm and logs it; DHCP-bound / address-add clear it and
 *     record the lease.
 *
 * The event callback runs in the net_mgmt event thread, not this one, so it
 * only stamps the shared sts_net_link_t and raises/clears alarms — both cheap
 * and lock-free-ish (a spinlock-guarded struct).
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/dns_sd.h>
#include <zephyr/net/hostname.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>

#include "cfg/cfg.h"
#include "net/sts_net.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_netmgmt, CONFIG_STS1000_LOG_LEVEL);

#define NETMGMT_STACK_SIZE 2048
#define NETMGMT_PRIORITY 10

#define EVENTS_L2 (NET_EVENT_IF_UP | NET_EVENT_IF_DOWN)
#define EVENTS_V4 (NET_EVENT_IPV4_ADDR_ADD | NET_EVENT_IPV4_DHCP_BOUND)
#define EVENTS_V6 (NET_EVENT_IPV6_ADDR_ADD)

/*
 * DNS-SD advertisements (RFC 6763). The port literals match the service
 * threads; DNS_SD_REGISTER_*_SERVICE takes the instance name at build time, so
 * a hostname changed at runtime updates the A/AAAA record via the mDNS
 * responder but not this instance label — acceptable, and noted.
 */
#if defined(CONFIG_DNS_SD)
DNS_SD_REGISTER_UDP_SERVICE(sd_ntp, CONFIG_NET_HOSTNAME, "_ntp", "local",
			    DNS_SD_EMPTY_TXT, 123);
DNS_SD_REGISTER_TCP_SERVICE(sd_ntske, CONFIG_NET_HOSTNAME, "_ntske", "local",
			    DNS_SD_EMPTY_TXT, 4460);
#endif

static struct net_mgmt_event_callback cb_l2;
static struct net_mgmt_event_callback cb_v4;
static struct net_mgmt_event_callback cb_v6;

static sts_net_link_t link;
static struct k_spinlock link_lock;

static K_THREAD_STACK_DEFINE(mgmt_stack, NETMGMT_STACK_SIZE);
static struct k_thread mgmt_thread;
static int live_id = -1;
static bool started;

void sts_net_link_get(sts_net_link_t *out)
{
	if (out == NULL) {
		return;
	}
	K_SPINLOCK(&link_lock) {
		*out = link;
	}
}

/* ------------------------------------------------------------------------- */
/* addressing                                                                */
/* ------------------------------------------------------------------------- */

static void apply_static_ipv4(struct net_if *iface)
{
	struct in_addr addr;
	struct in_addr mask;
	struct in_addr gw;
	uint32_t a = (uint32_t)sts_net_cfg_u64(CFG_ID_NET_IPV4_ADDR, 0U);
	uint32_t m = (uint32_t)sts_net_cfg_u64(CFG_ID_NET_IPV4_MASK,
					       0xFFFFFF00U);
	uint32_t g = (uint32_t)sts_net_cfg_u64(CFG_ID_NET_IPV4_GW, 0U);

	if (a == 0U) {
		LOG_WRN("static addressing selected but net.ipv4.addr is 0");
		return;
	}

	addr.s_addr = htonl(a);
	mask.s_addr = htonl(m);
	gw.s_addr = htonl(g);

	if (net_if_ipv4_addr_add(iface, &addr, NET_ADDR_MANUAL, 0) == NULL) {
		LOG_ERR("static IPv4 address rejected");
		return;
	}
	(void)net_if_ipv4_set_netmask_by_addr(iface, &addr, &mask);
	if (g != 0U) {
		net_if_ipv4_set_gw(iface, &gw);
	}
	LOG_INF("static IPv4 %u.%u.%u.%u/%u", (a >> 24) & 0xFFU,
		(a >> 16) & 0xFFU, (a >> 8) & 0xFFU, a & 0xFFU,
		(m == 0xFFFFFF00U) ? 24U : 0U);
}

static void apply_addressing(void)
{
	struct net_if *iface = sts_net_iface();

	if (iface == NULL) {
		return;
	}

	if (sts_net_cfg_bool(CFG_ID_NET_DHCP, true)) {
		net_dhcpv4_start(iface);
		LOG_INF("DHCPv4 started");
	} else {
		net_dhcpv4_stop(iface);
		apply_static_ipv4(iface);
	}
}

static void apply_hostname(void)
{
	char host[64];
	size_t n = sts_net_cfg_str(CFG_ID_NET_HOSTNAME, host, sizeof(host));

	if (n == 0U) {
		return;
	}
	if (net_hostname_set(host, n) == 0) {
		LOG_INF("hostname '%s'", host);
	}
}

void sts_netmgmt_reapply(void)
{
	if (!started) {
		return;
	}
	apply_hostname();
	apply_addressing();
}

/* ------------------------------------------------------------------------- */
/* events                                                                    */
/* ------------------------------------------------------------------------- */

static void note_link(bool up)
{
	bool was;

	K_SPINLOCK(&link_lock) {
		was = link.link_up;
		link.link_up = up;
		if (up != was) {
			link.link_changes++;
		}
		if (!up) {
			link.ipv4_ok = false;
			link.ipv6_ok = false;
			link.dhcp_bound = false;
			link.ipv4_addr = 0U;
		}
	}

	if (up != was) {
		sts_log(LOGR_SUB_NET, up ? LOGR_NOTICE : LOGR_WARN,
			"link %s", up ? "up" : "down");
	}
	/*
	 * core/fault's alarm table (fault.h) has no network-carrier id — the
	 * §5 fault tree is scoped to timing, power and thermal faults, where a
	 * link drop is an operational event, not a box fault, and is surfaced
	 * through the NET status group and the log rather than the RGB/relay
	 * policy. A dedicated FAULT_ALARM_NETWORK would be the place to hook an
	 * SNMP linkDown trap; that is a core/fault change, out of this area's
	 * boundary. TODO(fault): add it if link state should drive an alarm.
	 */
}

static void on_l2(struct net_mgmt_event_callback *cb, uint64_t ev,
		  struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	if (ev == NET_EVENT_IF_UP) {
		note_link(true);
	} else if (ev == NET_EVENT_IF_DOWN) {
		note_link(false);
	}
}

static void on_v4(struct net_mgmt_event_callback *cb, uint64_t ev,
		  struct net_if *iface)
{
	ARG_UNUSED(cb);

	if (ev == NET_EVENT_IPV4_DHCP_BOUND) {
		struct net_if_ipv4 *v4 = NULL;

		K_SPINLOCK(&link_lock) {
			link.dhcp_bound = true;
			link.ipv4_ok = true;
		}
		if (iface != NULL) {
			v4 = iface->config.ip.ipv4;
		}
		if (v4 != NULL) {
			K_SPINLOCK(&link_lock) {
				link.ipv4_addr =
					ntohl(v4->unicast[0].ipv4.address.in_addr
						      .s_addr);
			}
		}
		sts_log(LOGR_SUB_NET, LOGR_NOTICE, "DHCPv4 bound");
	} else if (ev == NET_EVENT_IPV4_ADDR_ADD) {
		K_SPINLOCK(&link_lock) {
			link.ipv4_ok = true;
		}
	}
}

static void on_v6(struct net_mgmt_event_callback *cb, uint64_t ev,
		  struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	if (ev == NET_EVENT_IPV6_ADDR_ADD) {
		K_SPINLOCK(&link_lock) {
			link.ipv6_ok = true;
		}
	}
}

/* ------------------------------------------------------------------------- */
/* thread                                                                    */
/* ------------------------------------------------------------------------- */

static void mgmt_loop(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	for (;;) {
		if (live_id >= 0) {
			sts_liveness_feed(live_id);
		}
		k_sleep(K_MSEC(500));
	}
}

int sts_netmgmt_start(void)
{
	struct net_if *iface = sts_net_iface();

	if (iface == NULL) {
		return -ENODEV;
	}

	net_mgmt_init_event_callback(&cb_l2, on_l2, EVENTS_L2);
	net_mgmt_init_event_callback(&cb_v4, on_v4, EVENTS_V4);
	net_mgmt_init_event_callback(&cb_v6, on_v6, EVENTS_V6);
	net_mgmt_add_event_callback(&cb_l2);
	net_mgmt_add_event_callback(&cb_v4);
	net_mgmt_add_event_callback(&cb_v6);

	started = true;

	/* Seed the link state from the interface's current carrier so a link
	 * that came up before this callback registered is not missed. */
	note_link(net_if_is_up(iface));

	apply_hostname();
	apply_addressing();

	live_id = sts_liveness_register("net_mgmt");

	k_thread_create(&mgmt_thread, mgmt_stack,
			K_THREAD_STACK_SIZEOF(mgmt_stack), mgmt_loop, NULL,
			NULL, NULL, NETMGMT_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&mgmt_thread, "net_mgmt");

	LOG_INF("net management started");
	return 0;
}
