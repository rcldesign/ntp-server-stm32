/*
 * STS1000 "Meridian" — network area object and entry point (sts_net_start).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * This is the net area's front door. main.c calls sts_net_start() behind
 * CONFIG_STS1000_NET (ARCHITECTURE.md §6, stage 4). It:
 *
 *   1. finds the Ethernet interface and its MAC;
 *   2. binds the ETH PTP clock and registers it as the sts_app.h TAI source
 *      (so every other area's sts_time_tai_ns() is answered from hardware);
 *   3. brings up the shared NTS master-key ring;
 *   4. registers the NET and PTP status sub-encoders and the cfg appliers for
 *      every group this area owns (net, ntp, nts, ptp, snmp, log, sec);
 *   5. starts the shared AAA authority, before anything can take a login;
 *   6. starts the service threads in priority order and, last, the PTP-clock
 *      servo.
 *
 * The cfg-applier fan-out lives here: this area registers ONE applier per group
 * with the platform and dispatches the group id to whichever service(s) care,
 * so no service has to know it shares group 0x09 (log) with, say, a future
 * audit-log consumer. (sts_web.c predates that rule and registers its own
 * CFG_G_SEC applier; the registry allows several subscribers per group, so the
 * two coexist — see the CFG_G_SEC case for who owns what.)
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>

#include "cfg/cfg.h"
#include "mcp/mcp_wire.h"
#include "net/sts_aaa.h"
#include "net/sts_net.h"
#include "storage/sts_atecc.h"
#include "storage/sts_store.h"
#include "util/bytes.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_net, CONFIG_STS1000_LOG_LEVEL);

/** Version byte of the NET and PTP STATUS_GET group structs (this area owns
 *  their layout; the MCP contract only fixes that data[0] is a version). */
#define STS_NET_STATUS_VER 1U

static struct net_if *iface;
static uint8_t mac[6];

static nts_keyring_t keyring;
static bool keyring_ready;

/* ------------------------------------------------------------------------- */
/* accessors                                                                 */
/* ------------------------------------------------------------------------- */

struct net_if *sts_net_iface(void)
{
	return iface;
}

const uint8_t *sts_net_mac(void)
{
	return mac;
}

nts_keyring_t *sts_net_keyring(void)
{
	return keyring_ready ? &keyring : NULL;
}

uint32_t sts_net_uptime_cs(void)
{
	/* sysUpTime is TimeTicks: a 32-bit unsigned counter of hundredths of a
	 * second (RFC 2578), wrapping ~497 days. */
	return (uint32_t)((uint64_t)(k_uptime_get() / 10) & 0xFFFFFFFFU);
}

/* ------------------------------------------------------------------------- */
/* cfg helpers                                                               */
/* ------------------------------------------------------------------------- */

uint64_t sts_net_cfg_u64(uint16_t id, uint64_t dflt)
{
	uint64_t v;

	if (cfg_get_u64(sts_cfg(), id, &v) != 0) {
		return dflt;
	}
	return v;
}

bool sts_net_cfg_bool(uint16_t id, bool dflt)
{
	bool v;

	if (cfg_get_bool(sts_cfg(), id, &v) != 0) {
		return dflt;
	}
	return v;
}

int32_t sts_net_cfg_i32(uint16_t id, int32_t dflt)
{
	int32_t v;

	if (cfg_get_i32(sts_cfg(), id, &v) != 0) {
		return dflt;
	}
	return v;
}

size_t sts_net_cfg_str(uint16_t id, char *out, size_t cap)
{
	size_t n = 0U;

	if (out == NULL || cap == 0U) {
		return 0U;
	}
	if (cfg_get_bytes(sts_cfg(), id, (uint8_t *)out, cap - 1U, &n) != 0) {
		out[0] = '\0';
		return 0U;
	}
	out[n] = '\0';
	return n;
}

/* ------------------------------------------------------------------------- */
/* cfg-apply fan-out                                                         */
/* ------------------------------------------------------------------------- */

void sts_net_on_cfg(uint8_t group)
{
	switch (group) {
	case CFG_G_NET:
		sts_netmgmt_reapply();
		break;
	case CFG_G_SNMP:
		sts_snmp_reapply();
		break;
	case CFG_G_LOG:
		sts_syslog_reapply();
		break;
	case CFG_G_SEC:
		/*
		 * 0x0A had no applier at all, so all 57 security keys — most of
		 * them flagged CFG_F_RUNTIME_APPLY — applied to nothing until the
		 * next reboot. Four consumers, in the order that makes a partly
		 * applied set least surprising: authority first, then the
		 * surfaces that answer with it.
		 *
		 * sts_web.c and sts_console.c register their own CFG_G_SEC
		 * appliers (the registry supports several subscribers per group);
		 * this one deliberately covers only what they do not — the AAA
		 * chain, the SNMP security keys, the NTP MAC table and the
		 * secure element.
		 *
		 * The CFG_F_REBOOT_REQUIRED keys in this group (the SNMPv3 users
		 * and engine keys, sec.atecc.*) are NOT forced live here: cfg
		 * already reports the group in `reboot_groups`, and
		 * re-initialising a live USM engine would clear every localised
		 * key it holds.
		 */
		sts_aaa_reapply();
		sts_snmp_on_cfg_sec();
		sts_ntp_reload_keys();
		sts_atecc_reapply();
		break;
	case CFG_G_PTP:
		/*
		 * One exception to the "staged; effective on restart" rule
		 * below: the `ptp.icv.*` Annex-P keys apply live.
		 *
		 * They have to. `ptp.icv.key` is CFG_F_SECRET, so a factory
		 * reset clears it in the tree — and the documented contract for
		 * secret zeroization is that each subsystem's RAM copy goes with
		 * it via this applier fan-out, not that it waits for the reboot.
		 * Re-planning here also lets an operator arm, re-key and disarm
		 * integrity without restarting the grandmaster.
		 */
		sts_ptp_reload_icv();
		LOG_INF("cfg group 0x%02x staged; effective on restart", group);
		break;
	case CFG_G_NTP:
	case CFG_G_NTS:
		/*
		 * These are almost all CFG_F_REBOOT_REQUIRED (nts.enable,
		 * ptp.enable/transport/profile, ...) or feed core contexts that
		 * this phase reads only at start. The few runtime-apply keys
		 * (ntp rate/kod, ptp priorities) are re-read on the next start;
		 * live reconfiguration of a running ntp_ctx_t/ptp_port_ctx_t is
		 * a documented follow-up rather than something to fake here.
		 */
		LOG_INF("cfg group 0x%02x staged; effective on restart", group);
		break;
	default:
		break;
	}
}

static void cfg_applier(void *ctx, uint8_t group)
{
	ARG_UNUSED(ctx);
	sts_net_on_cfg(group);
}

/* ------------------------------------------------------------------------- */
/* NET / PTP status encoders                                                 */
/* ------------------------------------------------------------------------- */

struct pk {
	uint8_t *buf;
	size_t cap;
	size_t len;
	bool overflow;
};

static void pk_u8(struct pk *p, uint8_t v)
{
	if (p->len + 1U > p->cap) {
		p->overflow = true;
		return;
	}
	p->buf[p->len++] = v;
}

static void pk_u32(struct pk *p, uint32_t v)
{
	if (p->len + 4U > p->cap) {
		p->overflow = true;
		return;
	}
	bytes_put_le32(&p->buf[p->len], v);
	p->len += 4U;
}

static void pk_i64(struct pk *p, int64_t v)
{
	if (p->len + 8U > p->cap) {
		p->overflow = true;
		return;
	}
	bytes_put_le64(&p->buf[p->len], (uint64_t)v);
	p->len += 8U;
}

static int encode_net(void *ctx, uint8_t group, uint8_t *buf, size_t cap)
{
	struct pk p = { .buf = buf, .cap = cap, .len = 0U, .overflow = false };
	sts_net_link_t link;
	sts_ntp_stats_t ns;
	sts_ptpclk_stats_t cs;

	ARG_UNUSED(ctx);
	ARG_UNUSED(group);

	sts_net_link_get(&link);
	sts_ntp_stats(&ns);
	sts_ptpclk_stats(&cs);

	pk_u8(&p, STS_NET_STATUS_VER);
	pk_u8(&p, (uint8_t)((link.link_up ? 1U : 0U) |
			    (link.ipv4_ok ? 2U : 0U) |
			    (link.ipv6_ok ? 4U : 0U) |
			    (link.dhcp_bound ? 8U : 0U)));
	/* Bit 3 (epoch_set) and bit 4 (traceable) are what an operator actually
	 * needs: `synced` only says the servo had a reference this second, while
	 * traceability is the claim NTP and PTP are gated on (F1). */
	pk_u8(&p, (uint8_t)((cs.clock_ok ? 1U : 0U) |
			    (cs.synced ? 2U : 0U) |
			    (sts_time_is_fallback() ? 4U : 0U) |
			    (cs.epoch_set ? 8U : 0U) |
			    (cs.traceable ? 16U : 0U)));
	pk_u8(&p, (uint8_t)(ns.nts_enabled ? 1U : 0U));
	pk_u32(&p, link.ipv4_addr);
	pk_u32(&p, link.ipv4_mask);
	pk_u32(&p, link.ipv4_gw);
	pk_u32(&p, link.link_changes);
	pk_i64(&p, cs.last_off_ns);
	pk_u32(&p, (uint32_t)cs.rate_ppb);
	pk_u32(&p, cs.steps);
	pk_u32(&p, cs.slews);
	/* NTP service headline counters (full set is in the SNMP MIB / MCP). */
	pk_u32(&p, (uint32_t)ns.ntp.rx);
	pk_u32(&p, (uint32_t)ns.ntp.served);
	pk_u32(&p, (uint32_t)ns.ntp.kod);
	pk_u32(&p, ns.txts_matched);

	return p.overflow ? -ENOMEM : (int)p.len;
}

static int encode_ptp(void *ctx, uint8_t group, uint8_t *buf, size_t cap)
{
	struct pk p = { .buf = buf, .cap = cap, .len = 0U, .overflow = false };
	sts_ptp_stats_t ps;
	uint32_t tx_sum = 0U;
	uint32_t rx_sum = 0U;
	size_t i;

	ARG_UNUSED(ctx);
	ARG_UNUSED(group);

	sts_ptp_stats(&ps);
	for (i = 0U; i < PTP_MSG_TYPE_COUNT; i++) {
		tx_sum += ps.counters.tx[i];
		rx_sum += ps.counters.rx[i];
	}

	pk_u8(&p, STS_NET_STATUS_VER);
	pk_u8(&p, ps.port_state);
	pk_u8(&p, ps.clock_class);
	pk_u8(&p, ps.clock_accuracy);
	pk_u8(&p, ps.domain);
	pk_u8(&p, ps.transport);
	pk_u8(&p, (uint8_t)(ps.running ? 1U : 0U));
	pk_u8(&p, 0U); /* reserved */
	pk_u32(&p, ps.alarms);
	pk_u32(&p, tx_sum);
	pk_u32(&p, rx_sum);
	pk_u32(&p, ps.counters.announce_timeouts);
	pk_u32(&p, ps.counters.followup_missed);

	return p.overflow ? -ENOMEM : (int)p.len;
}

/* ------------------------------------------------------------------------- */
/* keyring                                                                   */
/* ------------------------------------------------------------------------- */

static void keyring_start(void)
{
	uint64_t rotate_h;
	int rc;

	if (!sts_net_cfg_bool(CFG_ID_NTS_ENABLE, true)) {
		return;
	}

	rotate_h = sts_net_cfg_u64(CFG_ID_NTS_ROTATION_H, 24U);
	rc = nts_keyring_init(&keyring, sts_port_crypto(),
			      (int64_t)sts_mono_ms(),
			      (int64_t)(rotate_h * 3600000ULL));
	if (rc != 0) {
		LOG_ERR("nts_keyring_init: %d; NTS unavailable", rc);
		return;
	}
	keyring_ready = true;
	/*
	 * Cookie keys are held only in RAM this phase, so a reboot invalidates
	 * every outstanding cookie and clients re-run NTS-KE. Surviving a reboot
	 * needs the same sealed-blob store the NTS-KE cert wants; see the
	 * persistence TODO in sts_ntske.c. nts_keyring_install() is ready for it.
	 */
}

/* ------------------------------------------------------------------------- */
/* entry point                                                               */
/* ------------------------------------------------------------------------- */

int sts_net_start(void)
{
	struct net_linkaddr *ll;

	iface = net_if_get_first_by_type(&NET_L2_GET_NAME(ETHERNET));
	if (iface == NULL) {
		iface = net_if_get_default();
	}
	if (iface == NULL) {
		LOG_ERR("no network interface; net area cannot start");
		return -ENODEV;
	}

	ll = net_if_get_link_addr(iface);
	if (ll != NULL && ll->len == 6U) {
		memcpy(mac, ll->addr, 6U);
	} else {
		/* A locally-administered fallback so the PTP clockIdentity is
		 * still well-formed; the real MAC is set by the driver from
		 * HWINFO, so this is only reached on a misconfigured board. */
		static const uint8_t fb[6] = { 0x02U, 0x53U, 0x54U,
						0x53U, 0x00U, 0x01U };
		memcpy(mac, fb, 6U);
		LOG_WRN("interface has no 6-octet MAC; using a fallback");
	}

	/* Bind the PTP clock and publish it as the TAI source before any
	 * service reads sts_time_tai_ns(). Non-fatal if absent: the services
	 * fall back to the monotonic-derived time and flag it unsynchronised. */
	if (sts_ptpclk_init() != 0) {
		LOG_WRN("PTP clock unavailable; timestamps are software-only");
	}
	(void)sts_txts_init();

	/* Status sub-encoders for the two groups the platform cannot source. */
	(void)sts_status_register_encoder(MCP_GRP_NET, encode_net, NULL);
	(void)sts_status_register_encoder(MCP_GRP_PTP, encode_ptp, NULL);

	/* One applier per owned group; all route through sts_net_on_cfg(). */
	(void)sts_cfg_register_applier(CFG_G_NET, cfg_applier, NULL);
	(void)sts_cfg_register_applier(CFG_G_NTP, cfg_applier, NULL);
	(void)sts_cfg_register_applier(CFG_G_NTS, cfg_applier, NULL);
	(void)sts_cfg_register_applier(CFG_G_PTP, cfg_applier, NULL);
	(void)sts_cfg_register_applier(CFG_G_SNMP, cfg_applier, NULL);
	(void)sts_cfg_register_applier(CFG_G_LOG, cfg_applier, NULL);
	(void)sts_cfg_register_applier(CFG_G_SEC, cfg_applier, NULL);

	/*
	 * AAA before any management surface can take a login, and before the
	 * service threads exist: sts_aaa_check() is the single authority the web
	 * server, the MCP channel and the shell share, and one that has never been
	 * started cannot answer. Non-fatal — an unavailable chain must fall back to
	 * the local account, never block bring-up; core/auth's own contract makes
	 * "no backend could answer" a denial, so degrading here is safe.
	 */
	if (sts_aaa_start() != 0) {
		LOG_WRN("AAA unavailable; management logins fall back to the local "
			"account only");
	}

	keyring_start();

	/* Services, in the ARCHITECTURE.md §6 priority order. Each is
	 * self-guarding on its enable key; a failure to start one is logged and
	 * does not abort the others. */
	(void)sts_ptp_start();     /* prio 5  */
	(void)sts_ntp_start();     /* prio 8  */
	(void)sts_netmgmt_start(); /* prio 10 */
	(void)sts_ntske_start();   /* prio 12 */
	(void)sts_snmp_start();    /* prio 12 */
	(void)sts_syslog_start();  /* prio 16 */

	/* Servo last: it steers the clock the services read, so it should not
	 * begin correcting until the datapath that observes it is live. */
	(void)sts_ptpclk_start();

	LOG_INF("net area up on %s", iface->if_dev->dev->name);
	return 0;
}
