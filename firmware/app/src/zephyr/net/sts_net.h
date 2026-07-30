/*
 * STS1000 "Meridian" — network-area internal header.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/net/. The platform, console and ui areas talk to this
 * area through src/zephyr/sts_app.h and nothing else (ARCHITECTURE.md §2). Two
 * inbound exceptions, both thread-safe façades over hardware this area cannot
 * reach twice:
 *
 *   storage/sts_store.h  where this area gets its port_crypto_t, rather than
 *                        standing up a second mbedTLS binding.
 *   storage/sts_atecc.h  the device-unique value behind the RFC 3411 SNMP
 *                        engine id (sts_snmp.c). Every entry point there
 *                        degrades to the software path, so consuming it needs
 *                        no ordering discipline.
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
#include "snmp/snmp_v3.h"

#include "zephyr/sts_app.h"

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
/* absolute wall-clock time from the GNSS receiver (platform area supplies)    */
/* ------------------------------------------------------------------------- */

/*
 * The GNSS wall-clock interface (sts_gnss_wallclock_t / sts_gnss_wallclock())
 * now lives in zephyr/sts_app.h, which is where a cross-area contract belongs
 * (ARCHITECTURE.md §2) — the platform area implements it from its GNSS thread
 * and sts_ptpclk.c consumes it to place the PTP counter's absolute epoch. It
 * was declared here only while the platform side did not exist.
 */

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

/**
 * Is the time this clock reports actually traceable to the primary reference?
 *
 * **Every service that stamps or advertises time must gate on this.** It is not
 * the same question as "is the oscillator locked", and conflating the two is
 * what let this appliance serve 1900-era timestamps under LI=0 / stratum 1 /
 * refid 'GPS' and PTP clockClass 6 with timeTraceable set (F1):
 *
 *   - `disc` disciplines the oscillator's *rate* against the GPS PPS and
 *     publishes stratum from that. It says nothing about the absolute offset.
 *   - The MAC's 1588 counter — the only source of NTP and PTP timestamps on
 *     this board — powers up at zero and has to be *placed* on TAI. That is a
 *     separate mechanism (sts_gnss_wallclock() → ptp_clock_set()) which can
 *     fail, or simply never run, while the PPS loop locks perfectly.
 *   - sts_time_is_fallback() does not cover it either: it only reports whether
 *     a TAI source was ever registered, not whether that source knows the date.
 *
 * True requires both halves: the counter has been placed on TAI from an
 * absolute reference, and the servo is still tracking that reference — or the
 * reference is legitimately absent and `disc` has taken over under its §3.6
 * holdover policy, which owns the demotion timetable from there.
 */
bool sts_ptpclk_traceable(void);

/**
 * Which reference the servo is tracking.
 *
 * Not cosmetic: it is the difference between served time that is absolutely
 * accurate to a few hundred nanoseconds and served time that is accurate to
 * tens of milliseconds. Both are equally *stable*, so nothing else the appliance
 * publishes distinguishes them.
 */
typedef enum {
	STS_PTPCLK_REF_NONE = 0, /* no usable absolute reference */
	STS_PTPCLK_REF_PPS,      /* the GPS PPS edge, correlated with the counter */
	STS_PTPCLK_REF_GNSS,     /* the receiver's civil time over USART3 */
	STS_PTPCLK_REF_REALTIME, /* Zephyr's realtime clock + the §3.8 leap offset */
} sts_ptpclk_ref_t;

/** Servo telemetry, for the NET/PTP status groups and the SNMP MIB. */
typedef struct {
	bool clock_ok;        /* the device exists and answers */
	bool synced;          /* the servo has an absolute reference */
	bool epoch_set;       /* the counter has been placed on TAI at least once */
	bool traceable;       /* sts_ptpclk_traceable() at snapshot time */
	int64_t last_off_ns;  /* last measured (reference - ptp_clock) */
	int32_t rate_ppb;     /* commanded fractional-frequency correction */
	uint32_t steps;       /* coarse ptp_clock_set() events */
	uint32_t slews;       /* fine ptp_clock_adjust() events */
	uint32_t updates;     /* servo iterations that had a reference */
	uint32_t no_ref;      /* servo iterations with no usable reference */

	/* The correlated PPS reference (net/sts_ppscorr.h). Everything below is
	 * from the most recent attempt, whether it succeeded or not — a refused
	 * correlation is exactly when these numbers are wanted. */
	uint8_t  ref_src;      /* sts_ptpclk_ref_t: which reference last answered */
	uint32_t pps_ok;       /* correlations accepted */
	uint32_t pps_reject;   /* correlations refused */
	uint8_t  pps_last_rc;  /* sts_ppscorr_rc_t of the last attempt */
	uint32_t pps_skew_ns;  /* measured span of the last paired {TIM2, PTP} read */
	int64_t  pps_age_ns;   /* capture age at that read, on TIM2 */
	int64_t  pps_coarse_resid_ns; /* civil time - correlated reference */
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

/**
 * Re-read the four `sec.ntpkeyN.*` symmetric-key slots into the live server.
 *
 * Called from sts_ntp_start() and from the CFG_G_SEC applier. The work is
 * DEFERRED to the NTP thread — see the implementation for why the applier must
 * not write the key table itself.
 */
void sts_ntp_reload_keys(void);

/**
 * Force an immediate NTS cookie-key rotation.
 *
 * The ring already rotates on the `nts.rotation.h` schedule inside the NTP
 * thread's loop; this is the operator's override for a suspected compromise,
 * where waiting up to a day is the wrong answer. Rotation is graceful — the
 * keys still in the ring keep unsealing outstanding cookies, and a client whose
 * cookie has aged out of it re-runs NTS-KE.
 *
 * DEFERRED to the NTP thread, which is the ring's only reader; the console must
 * not mint a key into it mid-unseal.
 *
 * @retval 0        Queued; the thread applies it within its 500 ms poll.
 * @retval -ENOTSUP NTS is disabled in configuration.
 * @retval -ENODEV  The keyring failed to initialise, so NTS is unavailable.
 */
int sts_nts_rotate_now(void);

/** Snapshot the core/ntp and core/nts counters plus the socket-level ones. */
typedef struct {
	ntp_stats_t ntp;
	nts_stats_t nts;
	uint32_t rx_no_hw_ts; /* datagrams that arrived without a hardware stamp */
	uint32_t tx_errors;
	uint32_t txts_matched;
	uint32_t txts_ambiguous;     /* egress stamps discarded as unattributable */
	uint32_t tx_pending_overrun; /* responses evicted before their stamp landed */
	bool nts_enabled;            /* the NTS datapath is armed in this build */
	bool running;
} sts_ntp_stats_t;

void sts_ntp_stats(sts_ntp_stats_t *out);

/* ------------------------------------------------------------------------- */
/* sts_ntske.c — NTS-KE TLS 1.3 server (web/tls priority band, 12)            */
/* ------------------------------------------------------------------------- */

int sts_ntske_start(void);

/**
 * Can this *build* run the NTS-KE key exchange at all?
 *
 * A compile-time answer, safe to call before sts_ntske_start(). The whole TLS
 * body is behind MBEDTLS_SSL_KEYING_MATERIAL_EXPORT (RFC 8446 §7.5 exporter,
 * which RFC 8915 §4.3 needs to derive C2S/S2C), and Zephyr's mbedTLS config
 * omits it — see the three enablement steps in conf/net.conf. Without the key
 * exchange nothing can issue a cookie, so the NTP datapath must not advertise
 * or arm NTS either.
 */
bool sts_ntske_supported(void);

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
	/** Annex-P integrity engine counters; all zero while it is off. */
	ptp_icv_counters_t icv;
	uint8_t port_state;
	uint32_t alarms;
	uint8_t clock_class;
	uint8_t clock_accuracy;
	uint8_t domain;
	uint8_t transport;
	/**
	 * ptp_profile_t the engine is actually running.
	 *
	 * The *effective* profile, not the requested one: a configuration the
	 * engine rejects falls back to Default (see the fallback in
	 * sts_ptp_start()), and reporting the request would tell an operator the
	 * box is C37.238 while it serves 1588 Default. Published so the REST and
	 * SNMP planes can name the profile and render its documented deviations
	 * — the disclosure PTP_ALARM_PROFILE_UNSUPPORTED only hints at.
	 */
	uint8_t profile;
	/** sts_ptp_icv_state_t: off / armed-no-key / armed / refused. */
	uint8_t icv_state;
	bool running;
} sts_ptp_stats_t;

void sts_ptp_stats(sts_ptp_stats_t *out);

/**
 * Re-plan Annex-P integrity from the current `ptp.icv.*` configuration and
 * apply it to the running engine.
 *
 * Called from the CFG_G_PTP applier. This is what makes the feature arm, re-key
 * and disarm without a reboot — and what makes a factory reset overwrite the
 * key held inside the live ptp_icv_ctx_t rather than leaving it in RAM until
 * the reboot lands. A no-op when PTP is not running.
 */
void sts_ptp_reload_icv(void);

/* ------------------------------------------------------------------------- */
/* sts_netmgmt.c — DHCP / static addressing / mDNS / link events (priority 10)*/
/* ------------------------------------------------------------------------- */

int sts_netmgmt_start(void);

/** Re-apply the net group's configuration (called from sts_net_on_cfg). */
void sts_netmgmt_reapply(void);

/* ------------------------------------------------------------------------- */
/* sts_syslog.c — RFC 5424 over UDP, or RFC 5425 over TCP+TLS (prio band 16)  */
/* ------------------------------------------------------------------------- */

int sts_syslog_start(void);
void sts_syslog_reapply(void);

typedef struct {
	uint32_t sent;
	uint32_t dropped;   /* send() failures */
	uint32_t gaps;      /* records overwritten before this reader saw them */
	uint32_t reconnects;      /* TLS sessions successfully established */
	uint32_t handshake_fails; /* TLS connect/handshake attempts that failed */
	bool enabled;
	bool resolved;
	bool tls;        /* the transport in force is RFC 5425 over TLS */
	bool refused;    /* TLS was asked for and REFUSED — nothing is being
			  * sent, and nothing is being sent in the clear */
	bool connected;  /* a TLS session is up */
	bool ca_present; /* a collector trust anchor is installed */
} sts_syslog_stats_t;

void sts_syslog_stats(sts_syslog_stats_t *out);

/**
 * Install the syslog collector's trust anchor (one or more PEM certificates).
 *
 * Without it `log.syslog.tls` REFUSES rather than downgrading — see
 * net/sts_syslog_tls_policy.h. Screened by sts_ldap_ca_check() before it can
 * displace a working anchor.
 *
 * @retval 0        Installed, live and persisted.
 * @retval -EBADMSG Not a usable PEM certificate.
 * @retval -EFBIG   Larger than STS_SYSLOG_CA_PEM_MAX.
 * @retval -EPERM   The blob carries private-key material.
 * @retval -EROFS   Accepted and live, but not persisted.
 * @retval -ENOTSUP This build has no TLS socket layer.
 */
int sts_syslog_ca_install(const char *pem, size_t len);

/** Retract the anchor, from /lfs and from the credential store. */
int sts_syslog_ca_erase(void);

/** Is an anchor installed and registered? */
bool sts_syslog_ca_present(void);

/* ------------------------------------------------------------------------- */
/* sts_snmp.c — SNMPv2c/v3 agent + traps (priority 12)                        */
/* ------------------------------------------------------------------------- */

int sts_snmp_start(void);

/** Re-apply the 0x0B snmp group (community, trap host, ACL, hostname). */
void sts_snmp_reapply(void);

/**
 * Re-apply the 0x0A security keys this agent owns: `sec.snmp.v2c`,
 * `sec.snmp.notify.ms` and `sec.snmpv3.trap.user/.lvl`. The USM users and the
 * engine keys are CFG_F_REBOOT_REQUIRED and deliberately not touched.
 */
void sts_snmp_on_cfg_sec(void);

/** Queue a notification for the trap sender. Safe from any thread. */
void sts_snmp_notify(snmp_trap_t trap);

void sts_snmp_stats(snmp_stats_t *out);

/**
 * USM counters (RFC 3414 §5 usmStats plus local bookkeeping).
 *
 * Serves the snapshot the SNMP thread publishes once per poll pass, not a live
 * read: `snmp_v3_ctx_t` carries the decrypt scratch and is single-threaded by
 * contract, so nothing outside sts_snmp.c may call snmp_v3_stats_get() on it.
 * Callable from any thread; at most one SNMP poll period stale. Zeroed while
 * SNMPv3 is not running, and before the first publish.
 */
void sts_snmp_v3_stats(snmp_v3_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_NET_STS_NET_H_ */
