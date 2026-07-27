/*
 * STS1000 "Meridian" — core/cfg: the configuration schema.
 *
 * Platform-neutral C11. No dynamic allocation, no platform headers.
 *
 * This header is the single source of truth for every configuration key the
 * firmware owns (ARCHITECTURE.md §8, spec §5.5). The key list lives in the
 * CFG_SCHEMA() X-macro below; the runtime table in cfg.c, the CFG_ID_* enum
 * constants and the CFG_KEY_COUNT array bound are all generated from that one
 * list, so a key cannot exist in one place and be missing from another.
 *
 * Key IDs are 0xGGII — GG = group (CFG_G_*), II = item within the group. The
 * list MUST stay sorted by ID: cfg.c binary-searches it and CFG_LIST pages
 * through it in ID order. tests/host/test_cfg.c enforces both.
 *
 * Adding a key
 * ------------
 *   1. Append a row in the right group, keeping the ID order.
 *   2. Bump CFG_SCHEMA_VERSION and add a migration (cfg_migration_t) if the
 *      change is not purely additive — a stored/exported blob written by an
 *      older firmware must still import.
 * Removing or retyping a key always needs a migration; a plain removal makes
 * every older export fail the strict-mode import.
 *
 * Row macros (one per storage class, so the numeric domain stays typed):
 *
 *   U(sym, id, name, type, flags, def, min, max)   BOOL/U8/U16/U32/U64
 *   I(sym, id, name,       flags, def, min, max)   I32
 *   F(sym, id, name,       flags, def, min, max)   F32
 *   S(sym, id, name,       flags, maxlen, def)     STR, def is a C string
 *   B(sym, id, name,       flags, maxlen)          BLOB, default is empty
 *
 * `name` is the short human-readable label returned by MCP CFG_LIST; keep it
 * under CFG_NAME_MAX characters and in `group.item` form.
 */

#ifndef STS1000_CORE_CFG_CFG_SCHEMA_H_
#define STS1000_CORE_CFG_CFG_SCHEMA_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Schema version. Written into every TLV export and checked on import; an
 * import whose version differs needs a registered migration (see cfg.h).
 */
#define CFG_SCHEMA_VERSION 1U

/** Longest key name, excluding the NUL. CFG_LIST sizing depends on it. */
#define CFG_NAME_MAX 24U

/** Largest STR/BLOB payload any key may hold, in bytes. */
#define CFG_VAL_MAX 64U

/* ------------------------------------------------------------------ types */

/** Value type. Wire-visible: these numbers appear in TLV exports and in MCP. */
typedef enum {
	CFG_T_BOOL = 1, /* 0 or 1, one byte on the wire */
	CFG_T_U8   = 2,
	CFG_T_U16  = 3,
	CFG_T_U32  = 4,
	CFG_T_U64  = 5,
	CFG_T_I32  = 6,
	CFG_T_F32  = 7, /* IEEE-754 binary32, little-endian on the wire */
	CFG_T_STR  = 8, /* UTF-8 bytes, no NUL on the wire, length-delimited */
	CFG_T_BLOB = 9, /* opaque bytes */
} cfg_type_t;

/* ------------------------------------------------------------------ flags */

/** Change takes effect without a reboot. */
#define CFG_F_RUNTIME_APPLY   0x01U
/** Change is staged into the live tree but only acts after a reboot. */
#define CFG_F_REBOOT_REQUIRED 0x02U
/** Never leaves the box in a plain export and never reads back unauthenticated. */
#define CFG_F_SECRET          0x04U
/** Calibration constant: produced by a bench procedure, not an operator guess. */
#define CFG_F_CAL             0x08U
/**
 * Write-only over any external channel: never included in a TLV export (even
 * one that asks for secrets) and never returned by a read-back. Used for the
 * admin credential, which is provisioned out-of-band (local UI / ACM0 shell)
 * and must never appear on the MCP wire — not merely gated behind auth, since
 * a box with auth disabled would otherwise leak it. Implies CFG_F_SECRET.
 */
#define CFG_F_NOEXPORT        0x10U

/* ----------------------------------------------------------------- groups */

#define CFG_G_NET    0x01U
#define CFG_G_NTP    0x02U
#define CFG_G_NTS    0x03U
#define CFG_G_PTP    0x04U
#define CFG_G_GNSS   0x05U
#define CFG_G_TIMING 0x06U
#define CFG_G_POWER  0x07U
#define CFG_G_UI     0x08U
#define CFG_G_LOG    0x09U
#define CFG_G_SEC    0x0AU
#define CFG_G_SNMP   0x0BU
#define CFG_G_CAL    0x0CU

/** Group byte of a key ID. */
#define CFG_GROUP(id) ((uint8_t)((uint16_t)(id) >> 8))
/** Item byte of a key ID. */
#define CFG_ITEM(id)  ((uint8_t)((uint16_t)(id) & 0xFFU))
/** Bit for @p g in a cfg_commit_res_t::reboot_groups mask. */
#define CFG_GROUP_BIT(g) ((uint32_t)1U << (g))

/* ----------------------------------------------------------- the key list */

/*
 * Ranges are inclusive. A numeric key with min == max == 0 for an unsigned
 * type is still bounds-checked (it just pins the value), so every row states a
 * real domain; "no constraint" is spelled 0 .. type-max.
 */
#define CFG_SCHEMA(U, I, F, S, B)                                                              \
	/* -- 0x01 net ------------------------------------------------------------------ */  \
	U(NET_DHCP,          0x0101, "net.dhcp",        BOOL, CFG_F_REBOOT_REQUIRED,           \
	  1, 0, 1)                                                                             \
	U(NET_IPV4_ADDR,     0x0102, "net.ipv4.addr",   U32,  CFG_F_REBOOT_REQUIRED,           \
	  0, 0, 0xFFFFFFFFU)                                                                   \
	U(NET_IPV4_MASK,     0x0103, "net.ipv4.mask",   U32,  CFG_F_REBOOT_REQUIRED,           \
	  0xFFFFFF00U, 0, 0xFFFFFFFFU)                                                         \
	U(NET_IPV4_GW,       0x0104, "net.ipv4.gw",     U32,  CFG_F_REBOOT_REQUIRED,           \
	  0, 0, 0xFFFFFFFFU)                                                                   \
	U(NET_IPV4_DNS,      0x0105, "net.ipv4.dns",    U32,  CFG_F_RUNTIME_APPLY,             \
	  0, 0, 0xFFFFFFFFU)                                                                   \
	S(NET_HOSTNAME,      0x0106, "net.hostname",          CFG_F_RUNTIME_APPLY,             \
	  63, "meridian")                                                                      \
	U(NET_VLAN_EN,       0x0107, "net.vlan.enable", BOOL, CFG_F_REBOOT_REQUIRED,           \
	  0, 0, 1)                                                                             \
	U(NET_VLAN_ID,       0x0108, "net.vlan.id",     U16,  CFG_F_REBOOT_REQUIRED,           \
	  1, 1, 4094)                                                                          \
	B(NET_MGMT_ACL,      0x0109, "net.mgmt.acl",          CFG_F_RUNTIME_APPLY, 32)         \
	                                                                                       \
	/* -- 0x02 ntp ------------------------------------------------------------------ */  \
	U(NTP_ENABLE,        0x0201, "ntp.enable",      BOOL, CFG_F_RUNTIME_APPLY,             \
	  1, 0, 1)                                                                             \
	U(NTP_KOD_ENABLE,    0x0202, "ntp.kod",         BOOL, CFG_F_RUNTIME_APPLY,             \
	  1, 0, 1)                                                                             \
	/* 8 req/s, burst 16 — core/ntp's own documented per-client default        */  \
	/* (ntp.h "Defaults: 8 req/s burst 16 per client"). 0 disables the         */  \
	/* per-client bucket entirely, which lets one source drain the aggregate   */  \
	/* bucket and KoD every other client, so it is deliberately NOT the        */  \
	/* default: an operator who wants that has to ask for it.                  */  \
	U(NTP_RATE_QPS,      0x0203, "ntp.rate.qps",    U16,  CFG_F_RUNTIME_APPLY,             \
	  8, 0, 65535)                                                                         \
	U(NTP_RATE_BURST,    0x0204, "ntp.rate.burst",  U16,  CFG_F_RUNTIME_APPLY,             \
	  16, 1, 4096)                                                                         \
	U(NTP_MIN_POLL,      0x0205, "ntp.minpoll",     U8,   CFG_F_RUNTIME_APPLY,             \
	  4, 3, 17)                                                                            \
	U(NTP_SYMKEY_REF,    0x0206, "ntp.symkey.ref",  U16,  CFG_F_RUNTIME_APPLY,             \
	  0, 0, 65535)                                                                         \
	/* Opt-in, not opt-out. RFC 9769 interleaved mode is the whole reason the   */  \
	/* transmit-timestamp cache exists, but it is also the largest piece of     */  \
	/* per-client state the server keeps, and the glue reads THIS key rather    */  \
	/* than core/ntp's library default — so a `1` here is what actually puts    */  \
	/* the mode (and its attack surface) live on a shipped box. It stays 0      */  \
	/* until the interleave pairing path has been proven in the glue.           */  \
	U(NTP_INTERLEAVED,   0x0207, "ntp.interleaved", BOOL, CFG_F_RUNTIME_APPLY,             \
	  0, 0, 1)                                                                             \
	                                                                                       \
	/* -- 0x03 nts ------------------------------------------------------------------ */  \
	U(NTS_ENABLE,        0x0301, "nts.enable",      BOOL, CFG_F_REBOOT_REQUIRED,           \
	  1, 0, 1)                                                                             \
	U(NTS_KE_PORT,       0x0302, "nts.ke.port",     U16,  CFG_F_REBOOT_REQUIRED,           \
	  4460, 1, 65535)                                                                      \
	U(NTS_ROTATION_H,    0x0303, "nts.rotation.h",  U16,  CFG_F_RUNTIME_APPLY,             \
	  24, 1, 168)                                                                          \
	U(NTS_COOKIE_LIFE_H, 0x0304, "nts.cookie.life", U16,  CFG_F_RUNTIME_APPLY,             \
	  168, 1, 8760)                                                                        \
	                                                                                       \
	/* -- 0x04 ptp ------------------------------------------------------------------ */  \
	U(PTP_ENABLE,        0x0401, "ptp.enable",      BOOL, CFG_F_REBOOT_REQUIRED,           \
	  1, 0, 1)                                                                             \
	U(PTP_DOMAIN,        0x0402, "ptp.domain",      U8,   CFG_F_RUNTIME_APPLY,             \
	  0, 0, 255)                                                                           \
	U(PTP_PRIORITY1,     0x0403, "ptp.priority1",   U8,   CFG_F_RUNTIME_APPLY,             \
	  128, 0, 255)                                                                         \
	U(PTP_PRIORITY2,     0x0404, "ptp.priority2",   U8,   CFG_F_RUNTIME_APPLY,             \
	  128, 0, 255)                                                                         \
	I(PTP_LOG_ANNOUNCE,  0x0405, "ptp.log.announce",      CFG_F_RUNTIME_APPLY,             \
	  1, -3, 4)                                                                            \
	I(PTP_LOG_SYNC,      0x0406, "ptp.log.sync",          CFG_F_RUNTIME_APPLY,             \
	  0, -7, 1)                                                                            \
	I(PTP_LOG_DELAYREQ,  0x0407, "ptp.log.delayreq",      CFG_F_RUNTIME_APPLY,             \
	  0, -7, 5)                                                                            \
	U(PTP_TRANSPORT,     0x0408, "ptp.transport",   U8,   CFG_F_REBOOT_REQUIRED,           \
	  1, 0, 2)                                                                             \
	U(PTP_PROFILE,       0x0409, "ptp.profile",     U8,   CFG_F_REBOOT_REQUIRED,           \
	  0, 0, 2)                                                                             \
	U(PTP_TWO_STEP,      0x040A, "ptp.twostep",     BOOL, CFG_F_RUNTIME_APPLY,             \
	  1, 0, 1)                                                                             \
	                                                                                       \
	/* -- 0x05 gnss ----------------------------------------------------------------- */  \
	U(GNSS_CONSTEL,      0x0501, "gnss.constel",    U8,   CFG_F_RUNTIME_APPLY,             \
	  0x0F, 1, 0x3F)                                                                       \
	U(GNSS_ELEV_MASK,    0x0502, "gnss.elev.mask",  U8,   CFG_F_RUNTIME_APPLY,             \
	  10, 0, 45)                                                                           \
	U(GNSS_SURVEY_DUR_S, 0x0503, "gnss.survey.dur", U32,  CFG_F_RUNTIME_APPLY,             \
	  86400, 60, 604800)                                                                   \
	U(GNSS_SURVEY_ACC_MM, 0x0504, "gnss.survey.acc", U32, CFG_F_RUNTIME_APPLY,             \
	  5000, 100, 100000)                                                                   \
	I(GNSS_CABLE_DELAY_NS, 0x0505, "gnss.cable.ns",        CFG_F_RUNTIME_APPLY|CFG_F_CAL,  \
	  0, -100000, 100000)                                                                  \
	U(GNSS_MIN_CNO,      0x0506, "gnss.min.cno",    U8,   CFG_F_RUNTIME_APPLY,             \
	  20, 0, 50)                                                                           \
	U(GNSS_ANT_BIAS_EN,  0x0507, "gnss.ant.bias",   BOOL, CFG_F_RUNTIME_APPLY,             \
	  1, 0, 1)                                                                             \
	                                                                                       \
	/* -- 0x06 timing --------------------------------------------------------------- */  \
	U(TIM_TAU_S,         0x0601, "tim.tau.s",       U16,  CFG_F_RUNTIME_APPLY,             \
	  200, 10, 1000)                                                                       \
	U(TIM_LOCK_PHASE_NS, 0x0602, "tim.lock.ns",     U32,  CFG_F_RUNTIME_APPLY,             \
	  100, 1, 1000000)                                                                     \
	U(TIM_LOCK_HOLD_S,   0x0603, "tim.lock.hold",   U16,  CFG_F_RUNTIME_APPLY,             \
	  300, 1, 3600)                                                                        \
	U(TIM_HOLDOVER_S,    0x0604, "tim.holdover.s",  U32,  CFG_F_RUNTIME_APPLY,             \
	  86400, 60, 2592000)                                                                  \
	U(TIM_DEMOTE_NS,     0x0605, "tim.demote.ns",   U32,  CFG_F_RUNTIME_APPLY,             \
	  1000000, 100, 1000000000U)                                                           \
	U(TIM_EXTREF_BAND_HZ, 0x0606, "tim.extref.hz",  U32,  CFG_F_RUNTIME_APPLY,             \
	  20, 1, 100000)                                                                       \
	U(TIM_HYSTERESIS_S,  0x0607, "tim.hyst.s",      U16,  CFG_F_RUNTIME_APPLY,             \
	  60, 1, 3600)                                                                         \
	U(TIM_PPS_MAD_GATE,  0x0608, "tim.pps.mad",     U16,  CFG_F_RUNTIME_APPLY,             \
	  6, 2, 100)                                                                           \
	                                                                                       \
	/* -- 0x07 power ---------------------------------------------------------------- */  \
	U(PWR_RB_POLICY,     0x0701, "pwr.rb.policy",   U8,   CFG_F_RUNTIME_APPLY,             \
	  1, 0, 2)                                                                             \
	U(PWR_DIGIPOT_MAX,   0x0702, "pwr.pot.max",     U8,   CFG_F_RUNTIME_APPLY|CFG_F_CAL,   \
	  128, 0, 255)                                                                         \
	U(PWR_RB_VMAX_MV,    0x0703, "pwr.rb.vmax.mv",  U16,  CFG_F_RUNTIME_APPLY,             \
	  15000, 4510, 24450)                                                                  \
	U(PWR_POE_BUDGET_MW, 0x0704, "pwr.poe.mw",      U32,  CFG_F_RUNTIME_APPLY,             \
	  25500, 12950, 90000)                                                                 \
	U(PWR_RB_WARMUP_S,   0x0705, "pwr.rb.warmup",   U16,  CFG_F_RUNTIME_APPLY,             \
	  600, 30, 3600)                                                                       \
	                                                                                       \
	/* -- 0x08 ui ------------------------------------------------------------------- */  \
	U(UI_BRIGHTNESS,     0x0801, "ui.brightness",   U8,   CFG_F_RUNTIME_APPLY,             \
	  70, 0, 100)                                                                          \
	U(UI_TIMEOUT_S,      0x0802, "ui.timeout.s",    U16,  CFG_F_RUNTIME_APPLY,             \
	  120, 0, 3600)                                                                        \
	U(UI_RGB_BRIGHT,     0x0803, "ui.rgb.bright",   U8,   CFG_F_RUNTIME_APPLY,             \
	  60, 0, 100)                                                                          \
	                                                                                       \
	/* -- 0x09 log ------------------------------------------------------------------ */  \
	U(LOG_LEVEL,         0x0901, "log.level",       U8,   CFG_F_RUNTIME_APPLY,             \
	  6, 0, 7)                                                                             \
	U(LOG_SYSLOG_EN,     0x0902, "log.syslog.en",   BOOL, CFG_F_RUNTIME_APPLY,             \
	  0, 0, 1)                                                                             \
	S(LOG_SYSLOG_HOST,   0x0903, "log.syslog.host",       CFG_F_RUNTIME_APPLY,             \
	  63, "")                                                                              \
	U(LOG_SYSLOG_PORT,   0x0904, "log.syslog.port", U16,  CFG_F_RUNTIME_APPLY,             \
	  514, 1, 65535)                                                                       \
	U(LOG_SYSLOG_TLS,    0x0905, "log.syslog.tls",  BOOL, CFG_F_RUNTIME_APPLY,             \
	  0, 0, 1)                                                                             \
	                                                                                       \
	/* -- 0x0A security ------------------------------------------------------------- */  \
	U(SEC_AUTH_REQUIRED, 0x0A01, "sec.auth.req",    BOOL, CFG_F_RUNTIME_APPLY,             \
	  1, 0, 1)                                                                             \
	B(SEC_ADMIN_PW,      0x0A02, "sec.admin.pw",                                          \
	  CFG_F_RUNTIME_APPLY|CFG_F_SECRET|CFG_F_NOEXPORT, 48)                                \
	U(SEC_SESSION_S,     0x0A03, "sec.session.s",   U16,  CFG_F_RUNTIME_APPLY,             \
	  600, 30, 3600)                                                                       \
	U(SEC_CONSOLE_RO,    0x0A04, "sec.console.ro",  BOOL, CFG_F_RUNTIME_APPLY,             \
	  1, 0, 1)                                                                             \
	                                                                                       \
	/* -- 0x0B snmp ----------------------------------------------------------------- */  \
	U(SNMP_ENABLE,       0x0B01, "snmp.enable",     BOOL, CFG_F_REBOOT_REQUIRED,           \
	  0, 0, 1)                                                                             \
	/* Ships EMPTY, not "public". An empty community fails closed twice over:   */  \
	/* the glue passes NULL to snmp_set_community() (which disables the agent)   */  \
	/* and core/snmp rejects a zero-length configured community outright. The    */  \
	/* cfg_schema_xvalidate() hook additionally refuses to commit               */  \
	/* snmp.enable = 1 while this is empty, so the agent cannot be turned on     */  \
	/* with a guessable community by accident.                                   */  \
	S(SNMP_COMMUNITY,    0x0B02, "snmp.community",        CFG_F_RUNTIME_APPLY|CFG_F_SECRET,\
	  31, "")                                                                              \
	S(SNMP_TRAP_HOST,    0x0B03, "snmp.trap.host",        CFG_F_RUNTIME_APPLY,             \
	  63, "")                                                                              \
	U(SNMP_TRAP_PORT,    0x0B04, "snmp.trap.port",  U16,  CFG_F_RUNTIME_APPLY,             \
	  162, 1, 65535)                                                                       \
	                                                                                       \
	/* -- 0x0C cal ------------------------------------------------------------------ */  \
	/* One SHUNT_CAL trim per INA228, in the rail order of the interface ref §4.2:    */  \
	/* 0x40 PoE, 0x41 STM 3V3, 0x42 5V_DISP, 0x43 3V3, 0x45 antenna, 0x46 OCXO,       */  \
	/* 0x47 VCC_RB, 0x4A GPS, 0x4C panel-LED 5V. 4096 is the untrimmed SHUNT_CAL.     */  \
	U(CAL_INA_TRIM_0,    0x0C01, "cal.ina.0",       U16,  CFG_F_RUNTIME_APPLY|CFG_F_CAL,   \
	  4096, 3277, 4915)                                                                    \
	U(CAL_INA_TRIM_1,    0x0C02, "cal.ina.1",       U16,  CFG_F_RUNTIME_APPLY|CFG_F_CAL,   \
	  4096, 3277, 4915)                                                                    \
	U(CAL_INA_TRIM_2,    0x0C03, "cal.ina.2",       U16,  CFG_F_RUNTIME_APPLY|CFG_F_CAL,   \
	  4096, 3277, 4915)                                                                    \
	U(CAL_INA_TRIM_3,    0x0C04, "cal.ina.3",       U16,  CFG_F_RUNTIME_APPLY|CFG_F_CAL,   \
	  4096, 3277, 4915)                                                                    \
	U(CAL_INA_TRIM_4,    0x0C05, "cal.ina.4",       U16,  CFG_F_RUNTIME_APPLY|CFG_F_CAL,   \
	  4096, 3277, 4915)                                                                    \
	U(CAL_INA_TRIM_5,    0x0C06, "cal.ina.5",       U16,  CFG_F_RUNTIME_APPLY|CFG_F_CAL,   \
	  4096, 3277, 4915)                                                                    \
	U(CAL_INA_TRIM_6,    0x0C07, "cal.ina.6",       U16,  CFG_F_RUNTIME_APPLY|CFG_F_CAL,   \
	  4096, 3277, 4915)                                                                    \
	U(CAL_INA_TRIM_7,    0x0C08, "cal.ina.7",       U16,  CFG_F_RUNTIME_APPLY|CFG_F_CAL,   \
	  4096, 3277, 4915)                                                                    \
	U(CAL_INA_TRIM_8,    0x0C09, "cal.ina.8",       U16,  CFG_F_RUNTIME_APPLY|CFG_F_CAL,   \
	  4096, 3277, 4915)                                                                    \
	I(CAL_PPS_OFFSET_NS, 0x0C0A, "cal.pps.ns",            CFG_F_RUNTIME_APPLY|CFG_F_CAL,   \
	  0, -1000000, 1000000)                                                                \
	I(CAL_PPS2_OFFSET_NS, 0x0C0B, "cal.pps2.ns",          CFG_F_RUNTIME_APPLY|CFG_F_CAL,   \
	  0, -1000000, 1000000)                                                                \
	F(CAL_TEMPCO_PPB_C,  0x0C0C, "cal.tempco",            CFG_F_RUNTIME_APPLY|CFG_F_CAL,   \
	  0.0f, -100.0f, 100.0f)                                                               \
	U(CAL_DAC_CENTER,    0x0C0D, "cal.dac.center",  U16,  CFG_F_RUNTIME_APPLY|CFG_F_CAL,   \
	  2048, 0, 4095)

/* ------------------------------------------------- generated declarations */

/* One catch-all expansion serves all five row kinds: only the first two
 * arguments (symbol, ID) matter for the enum and the count. */
#define CFG__ROW_ID(sym, id, ...) CFG_ID_##sym = (id),
#define CFG__ROW_ONE(...) +1

/** Numeric ID of every key, e.g. CFG_ID_NET_DHCP. (Trailing comma: C99+.) */
enum cfg_id {
	CFG_SCHEMA(CFG__ROW_ID, CFG__ROW_ID, CFG__ROW_ID, CFG__ROW_ID, CFG__ROW_ID)
};

/** Number of keys in the schema. Integer constant expression. */
#define CFG_KEY_COUNT                                                          \
	((size_t)(0 CFG_SCHEMA(CFG__ROW_ONE, CFG__ROW_ONE, CFG__ROW_ONE,       \
			       CFG__ROW_ONE, CFG__ROW_ONE)))

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_CFG_CFG_SCHEMA_H_ */
