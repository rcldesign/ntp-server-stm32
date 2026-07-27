/*
 * STS1000 "Meridian" — network-area internal header.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/net/. The platform, console and ui areas talk to this
 * area through src/zephyr/sts_app.h and nothing else (ARCHITECTURE.md §2); the
 * one inbound exception is src/zephyr/storage/sts_store.h, which its own header
 * declares to be a deliberately public façade (that is where this area gets its
 * port_crypto_t from rather than standing up a second mbedTLS binding).
 *
 * Naming: every file and header in this directory carries an `sts_` prefix.
 * app/CMakeLists.txt puts `src/` on the include path, so `<zephyr/net/x.h>`
 * would resolve against THIS directory first — a file called `socket.h` or
 * `ethernet.h` here would silently shadow the Zephyr header of that name.
 */

#ifndef STS1000_ZEPHYR_NET_STS_NET_H_
#define STS1000_ZEPHYR_NET_STS_NET_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/net/net_if.h>

#include "nts/nts.h"
#include "ntp/ntp.h"
#include "ptp/ptp.h"
#include "snmp/snmp.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* sts_net.c — area object: interface, shared state, cfg, status encoders     */
/* ------------------------------------------------------------------------- */

/** The Ethernet interface this area serves on. NULL before sts_net_start(). */
struct net_if *sts_net_iface(void);

/** EUI-48 of @ref sts_net_iface, for the PTP clockIdentity. Never NULL. */
const uint8_t *sts_net_mac(void);

/** Shared NTS master-key ring (cookies + NTS-KE). NULL when NTS is disabled. */
nts_keyring_t *sts_net_keyring(void);

/** sysUpTime in hundredths of a second, saturating at the TimeTicks wrap. */
uint32_t sts_net_uptime_cs(void);

/** Convenience: read a numeric cfg key, returning @p dflt on any error. */
uint64_t sts_net_cfg_u64(uint16_t id, uint64_t dflt);
bool sts_net_cfg_bool(uint16_t id, bool dflt);
int32_t sts_net_cfg_i32(uint16_t id, int32_t dflt);
/** Copy a cfg STR key into @p out as a NUL-terminated string. */
size_t sts_net_cfg_str(uint16_t id, char *out, size_t cap);

/**
 * Per-service cfg-apply notification.
 *
 * sts_net.c owns the single sts_cfg_register_applier() registration for each
 * group it claims and fans the group id out to the services here, so no service
 * has to know whether it shares a group with another.
 */
void sts_net_on_cfg(uint8_t group);

/** Link state as the mgmt thread last observed it. */
typedef struct {
	bool link_up;      /* carrier / NET_EVENT_IF_UP */
	bool ipv4_ok;      /* at least one usable IPv4 address */
	bool ipv6_ok;      /* at least one usable global IPv6 address */
	bool dhcp_bound;
	uint32_t ipv4_addr; /* host order; 0 when none */
	uint32_t ipv4_mask;
	uint32_t ipv4_gw;
	uint32_t link_changes;
} sts_net_link_t;

void sts_net_link_get(sts_net_link_t *out);

/* ------------------------------------------------------------------------- */
/* sts_ptpclk.c — ETH PTP clock: TAI source + discipline servo                */
/* ------------------------------------------------------------------------- */

/**
 * Bind to the MAC's PTP clock and register it as the sts_app.h TAI source.
 * Must run before any service thread starts.
 *
 * @retval 0        Bound and registered.
 * @retval -ENODEV  The MAC exposes no PTP clock (CONFIG_PTP_CLOCK_STM32_HAL off).
 */
int sts_ptpclk_init(void);

/** Start the 1 Hz servo. */
int sts_ptpclk_start(void);

/** The PTP clock device, or NULL. */
const struct device *sts_ptpclk_dev(void);

/** Current PTP-clock reading as TAI nanoseconds since the PTP epoch. */
int sts_ptpclk_tai_ns(uint64_t *out_ns);

/** Convert a driver-supplied packet timestamp to TAI ns; 0 when invalid. */
uint64_t sts_ptpclk_ts_to_tai_ns(uint64_t seconds, uint32_t nanoseconds);

/** Servo telemetry, for the NET/PTP status groups and the SNMP MIB. */
typedef struct {
	bool clock_ok;        /* the device exists and answers */
	bool synced;          /* the servo has an absolute reference */
	int64_t last_off_ns;  /* last measured (reference - ptp_clock) */
	int32_t rate_ppb;     /* commanded fractional-frequency correction */
	uint32_t steps;       /* coarse ptp_clock_set() events */
	uint32_t slews;       /* fine ptp_clock_adjust() events */
	uint32_t updates;     /* servo iterations that had a reference */
} sts_ptpclk_stats_t;

void sts_ptpclk_stats(sts_ptpclk_stats_t *out);

/* ------------------------------------------------------------------------- */
/* sts_txts.c — hardware TX-timestamp demultiplexer                           */
/* ------------------------------------------------------------------------- */

/** Called for a transmitted PTP event message once its egress stamp lands. */
typedef void (*sts_txts_ptp_fn)(uint8_t msg_type, uint16_t seq, uint64_t tai_ns);

/**
 * Called for a transmitted NTP response. @p xmt_field is the 64-bit NTP
 * timestamp this firmware wrote into the response's transmit field, which is
 * what lets the NTP server match the stamp back to the client it answered.
 */
typedef void (*sts_txts_ntp_fn)(uint64_t xmt_field, uint64_t tai_ns);

/** Register the global net_if TX-timestamp callback. */
int sts_txts_init(void);

void sts_txts_set_ptp(sts_txts_ptp_fn fn);
void sts_txts_set_ntp(sts_txts_ntp_fn fn);

typedef struct {
	uint32_t total;     /* stamps delivered by the driver */
	uint32_t ptp;       /* matched to a PTP event message */
	uint32_t ntp;       /* matched to an NTP response */
	uint32_t unmatched; /* parsed but claimed by nobody */
	uint32_t malformed; /* could not be parsed at all */
} sts_txts_stats_t;

void sts_txts_stats(sts_txts_stats_t *out);

/* ------------------------------------------------------------------------- */
/* sts_ntp.c — NTP/NTS server thread (priority 8)                             */
/* ------------------------------------------------------------------------- */

int sts_ntp_start(void);

/** Snapshot the core/ntp and core/nts counters plus the socket-level ones. */
typedef struct {
	ntp_stats_t ntp;
	nts_stats_t nts;
	uint32_t rx_no_hw_ts; /* datagrams that arrived without a hardware stamp */
	uint32_t tx_errors;
	uint32_t txts_matched;
	bool running;
} sts_ntp_stats_t;

void sts_ntp_stats(sts_ntp_stats_t *out);

/* ------------------------------------------------------------------------- */
/* sts_ntske.c — NTS-KE TLS 1.3 server (web/tls priority band, 12)            */
/* ------------------------------------------------------------------------- */

int sts_ntske_start(void);

typedef struct {
	uint32_t accepted;
	uint32_t handshakes_ok;
	uint32_t handshakes_failed;
	uint32_t negotiations_ok;
	uint32_t cookies_issued;
	bool running;
	bool cert_ready;
	bool exporter_available;
} sts_ntske_stats_t;

void sts_ntske_stats(sts_ntske_stats_t *out);

/* ------------------------------------------------------------------------- */
/* sts_ptp.c — PTP grandmaster transport + thread (priority 5)                */
/* ------------------------------------------------------------------------- */

int sts_ptp_start(void);

typedef struct {
	ptp_counters_t counters;
	uint8_t port_state;
	uint32_t alarms;
	uint8_t clock_class;
	uint8_t clock_accuracy;
	uint8_t domain;
	uint8_t transport;
	bool running;
} sts_ptp_stats_t;

void sts_ptp_stats(sts_ptp_stats_t *out);

/* ------------------------------------------------------------------------- */
/* sts_netmgmt.c — DHCP / static addressing / mDNS / link events (priority 10)*/
/* ------------------------------------------------------------------------- */

int sts_netmgmt_start(void);

/** Re-apply the net group's configuration (called from sts_net_on_cfg). */
void sts_netmgmt_reapply(void);

/* ------------------------------------------------------------------------- */
/* sts_syslog.c — RFC 5424 sender over UDP (logger priority band, 16)         */
/* ------------------------------------------------------------------------- */

int sts_syslog_start(void);
void sts_syslog_reapply(void);

typedef struct {
	uint32_t sent;
	uint32_t dropped;   /* send() failures */
	uint32_t gaps;      /* records overwritten before this reader saw them */
	bool enabled;
	bool resolved;
} sts_syslog_stats_t;

void sts_syslog_stats(sts_syslog_stats_t *out);

/* ------------------------------------------------------------------------- */
/* sts_snmp.c — SNMPv2c agent + traps (priority 12)                           */
/* ------------------------------------------------------------------------- */

int sts_snmp_start(void);
void sts_snmp_reapply(void);

/** Queue a notification for the trap sender. Safe from any thread. */
void sts_snmp_notify(snmp_trap_t trap);

void sts_snmp_stats(snmp_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_NET_STS_NET_H_ */
