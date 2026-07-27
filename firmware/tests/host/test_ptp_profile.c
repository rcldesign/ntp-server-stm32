/*
 * STS1000 "Meridian" — host unit tests for core/ptp/ptp_profile.c and the
 * profile-aware parts of the port engine and the BMCA.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * What this suite is for, in order of importance:
 *
 *  1. The Default profile must be *unchanged*. Every assertion in the
 *     "regression" section pins a Default-profile answer to the value the
 *     pre-profile engine produced. If profile support ever leaks into Default,
 *     these fail.
 *  2. G.8275.1's published parameters are asserted individually, not as a blob,
 *     so a wrong one names itself.
 *  3. The localPriority tiebreak is exercised in both directions and shown to be
 *     invisible to the Default and Power profiles.
 *  4. The C37.238 TLV is asserted byte by byte against the layout in
 *     ptp_profile.h, and round-tripped.
 */

#include <errno.h>
#include <string.h>

#include "ptp/ptp.h"
#include "ptp/ptp_profile.h"
#include "unity.h"

/* --------------------------------------------------------------- fixtures -- */

static const uint8_t self_mac[6] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x55 };

typedef struct {
	uint8_t buf[8][PTP_MSG_MAX_LEN];
	size_t len[8];
	uint8_t type[8];
	unsigned int n;
	int rc;
} fake_t;

static int fake_tx(void *ctx, const ptp_tx_desc_t *d)
{
	fake_t *f = ctx;

	if (f->rc != 0) {
		return f->rc;
	}
	if (f->n < 8U) {
		size_t n = (d->len < sizeof(f->buf[0])) ? d->len : sizeof(f->buf[0]);

		(void)memcpy(f->buf[f->n], d->buf, n);
		f->len[f->n] = d->len;
		f->type[f->n] = d->msg_type;
		f->n++;
	}
	return 0;
}

static int fake_tai(void *ctx, uint64_t *out)
{
	(void)ctx;
	*out = 1700000000ULL * 1000000000ULL;
	return 0;
}

static void ops_from(ptp_port_ops_t *ops, fake_t *f)
{
	(void)memset(ops, 0, sizeof(*ops));
	ops->tx = fake_tx;
	ops->tai_ns = fake_tai;
	ops->ctx = f;
}

static void fake_init(fake_t *f)
{
	(void)memset(f, 0, sizeof(*f));
}

/* Find the first transmitted message of @p type; NULL when there is none. */
static const uint8_t *find_tx(const fake_t *f, uint8_t type, size_t *len)
{
	unsigned int i;

	for (i = 0U; i < f->n; i++) {
		if (f->type[i] == type) {
			if (len != NULL) {
				*len = f->len[i];
			}
			return f->buf[i];
		}
	}
	return NULL;
}

/* Drive the engine into MASTER and capture one Announce. */
static void run_to_announce(ptp_port_ctx_t *c, fake_t *f, const ptp_cfg_t *cfg)
{
	ptp_port_ops_t ops;
	uint64_t t = 1000U;
	unsigned int i;

	fake_init(f);
	ops_from(&ops, f);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_init(c, cfg, self_mac, &ops));
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(c, t));

	/* Past the announce receipt timeout, then far enough to emit. */
	for (i = 0U; i < 40U; i++) {
		t += 1000U;
		TEST_ASSERT_EQUAL_INT(0, ptp_port_step(c, t));
		if (find_tx(f, (uint8_t)PTP_MSG_ANNOUNCE, NULL) != NULL) {
			return;
		}
	}
	TEST_FAIL_MESSAGE("no Announce was emitted");
}

static void quality_locked(ptp_quality_view_t *q, uint8_t time_source)
{
	(void)memset(q, 0, sizeof(*q));
	q->sync_state = PTP_SYNC_LOCKED;
	q->time_source = time_source;
	q->utc_offset = 37;
	q->utc_offset_valid = true;
	q->time_traceable = true;
	q->freq_traceable = true;
	q->est_accuracy_ns = 40U;
}

/* ===================================================================== *
 *  1. Default-profile regression
 * ===================================================================== */

static void test_default_descriptor_matches_the_engine_defaults(void)
{
	const ptp_profile_desc_t *d = ptp_profile_desc((uint8_t)PTP_PROFILE_DEFAULT);
	ptp_cfg_t cfg;

	ptp_cfg_defaults(&cfg);

	TEST_ASSERT_EQUAL_STRING("Default", d->name);
	TEST_ASSERT_EQUAL_UINT8(cfg.domain, d->domain_default);
	TEST_ASSERT_EQUAL_UINT8(cfg.priority1, d->priority1_default);
	TEST_ASSERT_EQUAL_UINT8(cfg.priority2, d->priority2_default);
	TEST_ASSERT_EQUAL_INT8(cfg.log_announce_interval, d->log_announce_default);
	TEST_ASSERT_EQUAL_INT8(cfg.log_sync_interval, d->log_sync_default);
	TEST_ASSERT_EQUAL_INT8(cfg.log_min_delay_req_interval,
			       d->log_min_delay_req_default);
	TEST_ASSERT_EQUAL_UINT8(cfg.announce_receipt_timeout,
				d->announce_receipt_timeout_default);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)cfg.transport, d->transport_default);

	/* Default's BMCA is the unmodified §9.3.4 one. */
	TEST_ASSERT_TRUE(d->use_priority1);
	TEST_ASSERT_FALSE(d->use_local_priority);
	TEST_ASSERT_FALSE(d->announce_org_tlv);
	TEST_ASSERT_FALSE(d->unicast);
}

/*
 * The exact clockClass table the pre-profile engine produced. If a profile
 * ladder ever leaks into Default, this is what catches it.
 */
static void test_default_clock_class_ladder_is_unchanged(void)
{
	static const struct {
		ptp_sync_state_t s;
		uint8_t time_source;
		uint8_t expect_alt_a;
		uint8_t expect_alt_b;
	} cases[] = {
		{ PTP_SYNC_LOCKED, (uint8_t)PTP_TIME_SRC_GNSS, 6U, 6U },
		{ PTP_SYNC_LOCKED, (uint8_t)PTP_TIME_SRC_ATOMIC_CLOCK, 6U, 6U },
		{ PTP_SYNC_HOLDOVER, (uint8_t)PTP_TIME_SRC_ATOMIC_CLOCK, 7U, 7U },
		{ PTP_SYNC_HOLDOVER, (uint8_t)PTP_TIME_SRC_INTERNAL_OSC, 7U, 7U },
		{ PTP_SYNC_HOLDOVER, (uint8_t)PTP_TIME_SRC_OTHER, 7U, 7U },
		{ PTP_SYNC_HOLDOVER_EXCEEDED, (uint8_t)PTP_TIME_SRC_INTERNAL_OSC,
		  52U, 187U },
		{ PTP_SYNC_FREERUN, (uint8_t)PTP_TIME_SRC_INTERNAL_OSC, 52U, 187U },
		{ PTP_SYNC_FREERUN, (uint8_t)PTP_TIME_SRC_ATOMIC_CLOCK, 52U, 187U },
	};
	size_t i;

	for (i = 0U; i < (sizeof(cases) / sizeof(cases[0])); i++) {
		ptp_cfg_t cfg;
		ptp_quality_view_t q;
		ptp_clock_quality_t out;

		(void)memset(&q, 0, sizeof(q));
		q.sync_state = cases[i].s;
		q.time_source = cases[i].time_source;

		ptp_cfg_defaults(&cfg);
		cfg.degradation = PTP_DEGRADE_ALT_A;
		TEST_ASSERT_EQUAL_INT(0,
			ptp_clock_quality_from_view(&cfg, &q, &out));
		TEST_ASSERT_EQUAL_UINT8(cases[i].expect_alt_a, out.clock_class);

		cfg.degradation = PTP_DEGRADE_ALT_B;
		TEST_ASSERT_EQUAL_INT(0,
			ptp_clock_quality_from_view(&cfg, &q, &out));
		TEST_ASSERT_EQUAL_UINT8(cases[i].expect_alt_b, out.clock_class);
	}
}

/* A Default-profile Announce is exactly 64 octets: no profile TLV, no ICV. */
static void test_default_announce_has_no_tlv_suffix(void)
{
	ptp_port_ctx_t c;
	ptp_cfg_t cfg;
	fake_t f;
	const uint8_t *a;
	size_t len = 0U;

	ptp_cfg_defaults(&cfg);
	run_to_announce(&c, &f, &cfg);

	a = find_tx(&f, (uint8_t)PTP_MSG_ANNOUNCE, &len);
	TEST_ASSERT_NOT_NULL(a);
	TEST_ASSERT_EQUAL_UINT(PTP_ANNOUNCE_LEN, len);
	/* messageLength on the wire agrees. */
	TEST_ASSERT_EQUAL_UINT16(PTP_ANNOUNCE_LEN,
				 ((uint16_t)a[2] << 8) | (uint16_t)a[3]);
	TEST_ASSERT_EQUAL_UINT32(0U,
		ptp_port_counters(&c)->tx_profile_tlv_errors);
}

/* localPriority must be invisible to the Default and Power comparisons. */
static void test_default_and_power_ignore_local_priority(void)
{
	ptp_dataset_t a;
	ptp_dataset_t b;

	(void)memset(&a, 0, sizeof(a));
	(void)memset(&b, 0, sizeof(b));
	a.priority1 = 128U;
	b.priority1 = 128U;
	a.priority2 = 128U;
	b.priority2 = 128U;
	a.quality.clock_class = 6U;
	b.quality.clock_class = 6U;
	a.gm_identity.id[7] = 1U;
	b.gm_identity.id[7] = 2U;

	/* A is worse on localPriority but wins on identity. */
	a.local_priority = 200U;
	b.local_priority = 10U;

	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_A_BETTER,
		ptp_bmca_compare_profile(&a, &b, (uint8_t)PTP_PROFILE_DEFAULT));
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_A_BETTER,
		ptp_bmca_compare_profile(&a, &b, (uint8_t)PTP_PROFILE_POWER_C37_238));
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_A_BETTER, ptp_bmca_compare(&a, &b));

	/* Under G.8275.1 the localPriority rung settles it the other way. */
	TEST_ASSERT_EQUAL_INT(PTP_DSCMP_B_BETTER,
		ptp_bmca_compare_profile(&a, &b,
					 (uint8_t)PTP_PROFILE_TELECOM_G8275_1));
}

/* ===================================================================== *
 *  2. Published profile parameters
 * ===================================================================== */

static void test_g8275_1_parameters(void)
{
	const ptp_profile_desc_t *d =
		ptp_profile_desc((uint8_t)PTP_PROFILE_TELECOM_G8275_1);

	TEST_ASSERT_EQUAL_STRING("G.8275.1", d->name);

	/* Domain 24, range 24..43. */
	TEST_ASSERT_EQUAL_UINT8(24U, d->domain_default);
	TEST_ASSERT_EQUAL_UINT8(24U, d->domain_range.min);
	TEST_ASSERT_EQUAL_UINT8(43U, d->domain_range.max);

	/* priority1 pinned at 128 and not used by the alternate BMCA. */
	TEST_ASSERT_EQUAL_UINT8(128U, d->priority1_default);
	TEST_ASSERT_EQUAL_UINT8(128U, d->priority1_range.min);
	TEST_ASSERT_EQUAL_UINT8(128U, d->priority1_range.max);
	TEST_ASSERT_FALSE(d->use_priority1);

	/* priority2 free; localPriority 1..255, default 128, and it is used. */
	TEST_ASSERT_EQUAL_UINT8(128U, d->priority2_default);
	TEST_ASSERT_EQUAL_UINT8(0U, d->priority2_range.min);
	TEST_ASSERT_EQUAL_UINT8(255U, d->priority2_range.max);
	TEST_ASSERT_EQUAL_UINT8(128U, d->local_priority_default);
	TEST_ASSERT_EQUAL_UINT8(1U, d->local_priority_range.min);
	TEST_ASSERT_EQUAL_UINT8(255U, d->local_priority_range.max);
	TEST_ASSERT_TRUE(d->use_local_priority);

	/* 8 Announce/s and 16 Sync/s: logAnnounce -3, logSync -4. */
	TEST_ASSERT_EQUAL_INT8(-3, d->log_announce_default);
	TEST_ASSERT_EQUAL_INT8(-4, d->log_sync_default);
	TEST_ASSERT_EQUAL_INT8(-4, d->log_min_delay_req_default);
	/* And the rates those exponents actually mean. */
	TEST_ASSERT_EQUAL_UINT32(125U, ptp_log_interval_ms(d->log_announce_default));
	TEST_ASSERT_EQUAL_UINT32(63U, ptp_log_interval_ms(d->log_sync_default));

	/* L2 only, non-forwardable group MAC by default. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)PTP_TRANSPORT_L2, d->transport_default);
	TEST_ASSERT_EQUAL_UINT16(PTP_XPORT_BIT(PTP_TRANSPORT_L2),
				 d->transport_mask);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)PTP_L2_MAC_NON_FORWARDABLE,
				d->l2_mac_default);
	TEST_ASSERT_FALSE(d->unicast);

	TEST_ASSERT_TRUE(d->not_slave_default);
	TEST_ASSERT_FALSE(d->announce_org_tlv);

	/* Fully implemented: no material deviation, so no alarm. */
	TEST_ASSERT_EQUAL_UINT16(0U, d->deviations & (uint16_t)PTP_DEV_MATERIAL);
}

static void test_g8275_2_parameters(void)
{
	const ptp_profile_desc_t *d =
		ptp_profile_desc((uint8_t)PTP_PROFILE_TELECOM_G8275_2);

	TEST_ASSERT_EQUAL_STRING("G.8275.2", d->name);
	TEST_ASSERT_EQUAL_UINT8(44U, d->domain_default);
	TEST_ASSERT_EQUAL_UINT8(44U, d->domain_range.min);
	TEST_ASSERT_EQUAL_UINT8(63U, d->domain_range.max);

	/* UDP unicast, either address family; never L2. */
	TEST_ASSERT_TRUE(d->unicast);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)PTP_TRANSPORT_UDP_IPV4,
				d->transport_default);
	TEST_ASSERT_TRUE((d->transport_mask &
			  PTP_XPORT_BIT(PTP_TRANSPORT_UDP_IPV6)) != 0U);
	TEST_ASSERT_TRUE((d->transport_mask & PTP_XPORT_BIT(PTP_TRANSPORT_L2)) == 0U);

	/* Shares G.8275.1's alternate BMCA. */
	TEST_ASSERT_FALSE(d->use_priority1);
	TEST_ASSERT_TRUE(d->use_local_priority);

	TEST_ASSERT_EQUAL_INT8(0, d->log_announce_default);
	TEST_ASSERT_EQUAL_INT8(-4, d->log_sync_default);

	/* Unicast negotiation is not implemented and says so. */
	TEST_ASSERT_TRUE((d->deviations & (uint16_t)PTP_DEV_NO_UNICAST_NEG) != 0U);
	TEST_ASSERT_TRUE((d->deviations & (uint16_t)PTP_DEV_MATERIAL) != 0U);
}

static void test_c37238_parameters(void)
{
	const ptp_profile_desc_t *d =
		ptp_profile_desc((uint8_t)PTP_PROFILE_POWER_C37_238);

	TEST_ASSERT_EQUAL_STRING("C37.238", d->name);
	TEST_ASSERT_EQUAL_UINT8(0U, d->domain_default);
	TEST_ASSERT_EQUAL_INT8(0, d->log_announce_default);
	TEST_ASSERT_EQUAL_INT8(0, d->log_sync_default);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)PTP_TRANSPORT_L2, d->transport_default);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)PTP_L2_MAC_FORWARDABLE, d->l2_mac_default);

	/* Power uses the unmodified 1588 BMCA. */
	TEST_ASSERT_TRUE(d->use_priority1);
	TEST_ASSERT_FALSE(d->use_local_priority);

	/* Announce TLV is mandatory. */
	TEST_ASSERT_TRUE(d->announce_org_tlv);

	/* Peer-delay is mandated by the profile and not implemented here. */
	TEST_ASSERT_TRUE((d->deviations & (uint16_t)PTP_DEV_E2E_ONLY) != 0U);
}

static void test_out_of_range_profile_degrades_to_default(void)
{
	TEST_ASSERT_EQUAL_PTR(ptp_profile_desc((uint8_t)PTP_PROFILE_DEFAULT),
			      ptp_profile_desc((uint8_t)PTP_PROFILE_COUNT));
	TEST_ASSERT_EQUAL_PTR(ptp_profile_desc((uint8_t)PTP_PROFILE_DEFAULT),
			      ptp_profile_desc(0xFFU));
	TEST_ASSERT_EQUAL_STRING("Default", ptp_profile_name(0xFFU));
}

static void test_deviation_text(void)
{
	TEST_ASSERT_NOT_NULL(
		ptp_profile_deviation_text((uint8_t)PTP_PROFILE_DEFAULT));
	/* C37.238's text must name the peer-delay gap, not just "deviations". */
	TEST_ASSERT_NOT_NULL(strstr(
		ptp_profile_deviation_text((uint8_t)PTP_PROFILE_POWER_C37_238),
		"peer-delay"));
	TEST_ASSERT_NOT_NULL(strstr(
		ptp_profile_deviation_text((uint8_t)PTP_PROFILE_TELECOM_G8275_2),
		"unicast"));
}

static void test_apply_profile_produces_valid_cfg(void)
{
	unsigned int p;

	for (p = 0U; p < (unsigned int)PTP_PROFILE_COUNT; p++) {
		ptp_cfg_t cfg;

		ptp_cfg_defaults(&cfg);
		TEST_ASSERT_EQUAL_INT(0, ptp_cfg_apply_profile(&cfg, (uint8_t)p));
		TEST_ASSERT_EQUAL_INT(0, ptp_cfg_validate(&cfg));
		TEST_ASSERT_EQUAL_UINT8((uint8_t)p, (uint8_t)cfg.profile);
	}

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_cfg_apply_profile(NULL, 0U));
	{
		ptp_cfg_t cfg;

		ptp_cfg_defaults(&cfg);
		TEST_ASSERT_EQUAL_INT(-EINVAL,
			ptp_cfg_apply_profile(&cfg, (uint8_t)PTP_PROFILE_COUNT));
	}
}

/*
 * Site settings survive a profile change. Resetting an operator's measured
 * inaccuracy budget or their port number because they switched profile would be
 * a data-loss bug that only shows up in the field.
 */
static void test_apply_profile_preserves_site_settings(void)
{
	ptp_cfg_t cfg;

	ptp_cfg_defaults(&cfg);
	cfg.port_number = 7U;
	cfg.minor_sdo_id = 0x5AU;
	cfg.degradation = PTP_DEGRADE_ALT_B;
	cfg.oslv_locked = 0x1234U;
	cfg.c37238.grandmaster_id = 0x0042U;
	cfg.c37238.gm_inaccuracy_ns = 250U;

	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_apply_profile(&cfg,
					(uint8_t)PTP_PROFILE_TELECOM_G8275_1));

	TEST_ASSERT_EQUAL_UINT16(7U, cfg.port_number);
	TEST_ASSERT_EQUAL_HEX8(0x5AU, cfg.minor_sdo_id);
	TEST_ASSERT_EQUAL_INT(PTP_DEGRADE_ALT_B, cfg.degradation);
	TEST_ASSERT_EQUAL_HEX16(0x1234U, cfg.oslv_locked);
	TEST_ASSERT_EQUAL_HEX16(0x0042U, cfg.c37238.grandmaster_id);
	TEST_ASSERT_EQUAL_UINT32(250U, cfg.c37238.gm_inaccuracy_ns);
}

static void test_profile_ranges_are_enforced_only_off_default(void)
{
	ptp_cfg_t cfg;

	/* Default: an interval the codec supports is accepted even though the
	 * descriptor's advisory range is narrower. */
	ptp_cfg_defaults(&cfg);
	cfg.log_sync_interval = 7;
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_validate(&cfg));
	cfg.log_sync_interval = -7;
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_validate(&cfg));

	/* G.8275.1: every restriction bites, one at a time. */
	ptp_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_apply_profile(&cfg,
					(uint8_t)PTP_PROFILE_TELECOM_G8275_1));

	cfg.domain = 23U;
	TEST_ASSERT_EQUAL_INT(-ERANGE, ptp_cfg_validate(&cfg));
	cfg.domain = 44U;
	TEST_ASSERT_EQUAL_INT(-ERANGE, ptp_cfg_validate(&cfg));
	cfg.domain = 43U;
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_validate(&cfg));

	cfg.priority1 = 127U;
	TEST_ASSERT_EQUAL_INT(-ERANGE, ptp_cfg_validate(&cfg));
	cfg.priority1 = 128U;

	cfg.local_priority = 0U; /* 0 is reserved: the range starts at 1 */
	TEST_ASSERT_EQUAL_INT(-ERANGE, ptp_cfg_validate(&cfg));
	cfg.local_priority = 1U;
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_validate(&cfg));

	cfg.port_local_priority = 0U;
	TEST_ASSERT_EQUAL_INT(-ERANGE, ptp_cfg_validate(&cfg));
	cfg.port_local_priority = 255U;

	cfg.log_sync_interval = -3;
	TEST_ASSERT_EQUAL_INT(-ERANGE, ptp_cfg_validate(&cfg));
	cfg.log_sync_interval = -4;

	cfg.log_announce_interval = 0;
	TEST_ASSERT_EQUAL_INT(-ERANGE, ptp_cfg_validate(&cfg));
	cfg.log_announce_interval = -3;

	cfg.log_min_delay_req_interval = 0;
	TEST_ASSERT_EQUAL_INT(-ERANGE, ptp_cfg_validate(&cfg));
	cfg.log_min_delay_req_interval = -4;

	cfg.announce_receipt_timeout = 4U;
	TEST_ASSERT_EQUAL_INT(-ERANGE, ptp_cfg_validate(&cfg));
	cfg.announce_receipt_timeout = 3U;

	/* G.8275.1 is Ethernet-only. */
	cfg.transport = PTP_TRANSPORT_UDP_IPV4;
	TEST_ASSERT_EQUAL_INT(-ERANGE, ptp_cfg_validate(&cfg));
	cfg.transport = PTP_TRANSPORT_L2;
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_validate(&cfg));

	/* G.8275.2 is UDP-only. */
	ptp_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_apply_profile(&cfg,
					(uint8_t)PTP_PROFILE_TELECOM_G8275_2));
	cfg.transport = PTP_TRANSPORT_L2;
	TEST_ASSERT_EQUAL_INT(-ERANGE, ptp_cfg_validate(&cfg));
	cfg.transport = PTP_TRANSPORT_UDP_IPV6;
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_validate(&cfg));
}

/* ===================================================================== *
 *  3. Telecom clockClass ladder and localPriority in the engine
 * ===================================================================== */

static void test_telecom_clock_class_ladder(void)
{
	static const struct {
		ptp_sync_state_t s;
		uint8_t time_source;
		uint8_t expect;
	} cases[] = {
		{ PTP_SYNC_LOCKED, (uint8_t)PTP_TIME_SRC_GNSS, 6U },
		/* In-spec holdover, graded by what is flywheeling. */
		{ PTP_SYNC_HOLDOVER, (uint8_t)PTP_TIME_SRC_ATOMIC_CLOCK, 7U },
		{ PTP_SYNC_HOLDOVER, (uint8_t)PTP_TIME_SRC_INTERNAL_OSC, 140U },
		{ PTP_SYNC_HOLDOVER, (uint8_t)PTP_TIME_SRC_GNSS, 140U },
		{ PTP_SYNC_HOLDOVER, (uint8_t)PTP_TIME_SRC_OTHER, 150U },
		/* Out of spec, and free-run, are their own classes. */
		{ PTP_SYNC_HOLDOVER_EXCEEDED, (uint8_t)PTP_TIME_SRC_ATOMIC_CLOCK,
		  160U },
		{ PTP_SYNC_HOLDOVER_EXCEEDED, (uint8_t)PTP_TIME_SRC_OTHER, 160U },
		{ PTP_SYNC_FREERUN, (uint8_t)PTP_TIME_SRC_INTERNAL_OSC, 248U },
	};
	size_t i;
	unsigned int prof;

	for (prof = (unsigned int)PTP_PROFILE_TELECOM_G8275_1;
	     prof <= (unsigned int)PTP_PROFILE_TELECOM_G8275_2; prof++) {
		for (i = 0U; i < (sizeof(cases) / sizeof(cases[0])); i++) {
			ptp_cfg_t cfg;
			ptp_quality_view_t q;
			ptp_clock_quality_t out;

			ptp_cfg_defaults(&cfg);
			TEST_ASSERT_EQUAL_INT(0,
				ptp_cfg_apply_profile(&cfg, (uint8_t)prof));
			/* The degradation alternative must not reach the ladder. */
			cfg.degradation = PTP_DEGRADE_ALT_B;

			(void)memset(&q, 0, sizeof(q));
			q.sync_state = cases[i].s;
			q.time_source = cases[i].time_source;

			TEST_ASSERT_EQUAL_INT(0,
				ptp_clock_quality_from_view(&cfg, &q, &out));
			TEST_ASSERT_EQUAL_UINT8(cases[i].expect, out.clock_class);
		}
	}
}

static void test_freq_cat_classification(void)
{
	TEST_ASSERT_EQUAL_INT(PTP_FREQ_CAT1, ptp_freq_cat_from_time_source(
		(uint8_t)PTP_TIME_SRC_ATOMIC_CLOCK));
	TEST_ASSERT_EQUAL_INT(PTP_FREQ_CAT2, ptp_freq_cat_from_time_source(
		(uint8_t)PTP_TIME_SRC_GNSS));
	TEST_ASSERT_EQUAL_INT(PTP_FREQ_CAT2, ptp_freq_cat_from_time_source(
		(uint8_t)PTP_TIME_SRC_INTERNAL_OSC));
	TEST_ASSERT_EQUAL_INT(PTP_FREQ_CAT3, ptp_freq_cat_from_time_source(
		(uint8_t)PTP_TIME_SRC_OTHER));
	TEST_ASSERT_EQUAL_INT(PTP_FREQ_CAT3, ptp_freq_cat_from_time_source(0x77U));
}

static void test_ladder_null_and_bounds(void)
{
	ptp_class_ladder_t l = { 6U, 7U, 140U, 150U, 160U, 248U };

	TEST_ASSERT_EQUAL_UINT8(PTP_CLASS_USE_DEGRADATION,
		ptp_class_from_ladder(NULL, true, false, false, PTP_FREQ_CAT1));
	/* Locked wins over any holdover flag. */
	TEST_ASSERT_EQUAL_UINT8(6U,
		ptp_class_from_ladder(&l, true, true, true, PTP_FREQ_CAT3));
	/* An out-of-range category falls to CAT3, not off the end of a table. */
	TEST_ASSERT_EQUAL_UINT8(150U,
		ptp_class_from_ladder(&l, false, true, false,
				      (ptp_freq_cat_t)99));
	TEST_ASSERT_EQUAL_UINT8(248U,
		ptp_class_from_ladder(&l, false, false, false, PTP_FREQ_CAT1));
}

/*
 * The localPriority tiebreak in the engine: two clocks identical in every
 * comparable attribute, distinguished only by defaultDS.localPriority (ours) vs
 * portDS.localPriority (theirs). This is the only mechanism a single-port clock
 * has for expressing "prefer/deprefer that upstream", so both directions matter.
 */
static void run_local_priority_case(uint8_t ours, uint8_t theirs,
				    ptp_port_state_t expect)
{
	ptp_port_ctx_t c;
	ptp_cfg_t cfg;
	ptp_port_ops_t ops;
	fake_t f;
	ptp_quality_view_t q;
	uint8_t peer_mac[6] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x66 };
	ptp_port_id_t src;
	ptp_announce_t a;
	ptp_hdr_t h;
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len = 0U;
	uint64_t t = 1000U;
	unsigned int i;

	fake_init(&f);
	ops_from(&ops, &f);

	ptp_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_apply_profile(&cfg,
					(uint8_t)PTP_PROFILE_TELECOM_G8275_1));
	cfg.local_priority = ours;
	cfg.port_local_priority = theirs;
	TEST_ASSERT_EQUAL_INT(0, ptp_port_init(&c, &cfg, self_mac, &ops));

	/*
	 * clockClass 248 (free-run) on both sides: above the 127 "never a slave"
	 * boundary, so the state decision runs its M2/S1 branch and can actually
	 * defer. A locked class-6 clock would take the M1/P1 branch instead.
	 */
	(void)memset(&q, 0, sizeof(q));
	q.sync_state = PTP_SYNC_FREERUN;
	q.time_source = (uint8_t)PTP_TIME_SRC_INTERNAL_OSC;
	TEST_ASSERT_EQUAL_INT(0, ptp_port_set_quality(&c, &q));
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&c, t));

	(void)memset(&src, 0, sizeof(src));
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_id_from_mac(&src.clock_id, peer_mac));
	src.port_number = 1U;

	(void)memset(&a, 0, sizeof(a));
	a.gm_priority1 = 128U;
	a.gm_priority2 = 128U;
	a.gm_quality.clock_class = 248U;
	a.gm_quality.clock_accuracy = PTP_CLOCK_ACCURACY_UNKNOWN;
	a.gm_quality.offset_scaled_log_variance = PTP_OSLV_UNKNOWN;
	a.gm_identity = src.clock_id;
	a.steps_removed = 0U;
	a.time_source = (uint8_t)PTP_TIME_SRC_INTERNAL_OSC;

	(void)memset(&h, 0, sizeof(h));
	h.msg_type = (uint8_t)PTP_MSG_ANNOUNCE;
	h.version = PTP_VERSION;
	h.minor_version = PTP_MINOR_VERSION_2019;
	h.domain = cfg.domain;
	h.source_port = src;
	h.log_msg_interval = cfg.log_announce_interval;
	h.control = ptp_msg_control_field((uint8_t)PTP_MSG_ANNOUNCE);

	/* Enough Announces to qualify the foreign master, then settle. */
	for (i = 0U; i < 4U; i++) {
		h.seq_id = (uint16_t)i;
		TEST_ASSERT_EQUAL_INT(0, ptp_announce_encode(buf, sizeof(buf), &h,
							     &a, &len));
		TEST_ASSERT_EQUAL_INT(0, ptp_port_rx(&c, buf, len, 0U, t));
		t += 100U;
	}
	TEST_ASSERT_EQUAL_INT(0, ptp_port_step(&c, t));

	TEST_ASSERT_EQUAL_INT(expect, ptp_port_state(&c));
}

static void test_local_priority_decides_the_election(void)
{
	/*
	 * Everything else equal. Lower localPriority wins, so:
	 *   ours 10 vs theirs 128  -> we stay MASTER
	 *   ours 200 vs theirs 128 -> we defer (PASSIVE, this appliance's S1)
	 */
	run_local_priority_case(10U, 128U, PTP_PS_MASTER);
	run_local_priority_case(200U, 128U, PTP_PS_PASSIVE);

	/*
	 * Equal localPriority falls through to the grandmasterIdentity tiebreak.
	 * self_mac ends 0x55 and the peer's 0x66, so our identity is the lower
	 * and we win — which also proves localPriority did not short-circuit the
	 * rung below it.
	 */
	run_local_priority_case(128U, 128U, PTP_PS_MASTER);
}

/* ===================================================================== *
 *  4. C37.238 TLV
 * ===================================================================== */

static void test_c37238_value_layout_2011(void)
{
	ptp_c37238_t p;
	uint8_t v[PTP_C37238_VALUE_LEN];

	ptp_c37238_defaults(&p);
	p.v2017 = false;
	p.grandmaster_id = 0xBEEFU;
	p.gm_inaccuracy_ns = 0x11223344UL;
	p.network_inaccuracy_ns = 0x55667788UL;

	(void)memset(v, 0xAA, sizeof(v));
	TEST_ASSERT_EQUAL_INT(0, ptp_c37238_value_encode(v, sizeof(v), &p));

	/* organizationId 00-1B-19, organizationSubType 00-00-01. */
	TEST_ASSERT_EQUAL_HEX8(0x00U, v[0]);
	TEST_ASSERT_EQUAL_HEX8(0x1BU, v[1]);
	TEST_ASSERT_EQUAL_HEX8(0x19U, v[2]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, v[3]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, v[4]);
	TEST_ASSERT_EQUAL_HEX8(0x01U, v[5]);
	/* grandmasterID, big-endian. */
	TEST_ASSERT_EQUAL_HEX8(0xBEU, v[6]);
	TEST_ASSERT_EQUAL_HEX8(0xEFU, v[7]);
	/* grandmasterTimeInaccuracy. */
	TEST_ASSERT_EQUAL_HEX8(0x11U, v[8]);
	TEST_ASSERT_EQUAL_HEX8(0x22U, v[9]);
	TEST_ASSERT_EQUAL_HEX8(0x33U, v[10]);
	TEST_ASSERT_EQUAL_HEX8(0x44U, v[11]);
	/* networkTimeInaccuracy. */
	TEST_ASSERT_EQUAL_HEX8(0x55U, v[12]);
	TEST_ASSERT_EQUAL_HEX8(0x66U, v[13]);
	TEST_ASSERT_EQUAL_HEX8(0x77U, v[14]);
	TEST_ASSERT_EQUAL_HEX8(0x88U, v[15]);
	/* reserved, zeroed. */
	TEST_ASSERT_EQUAL_HEX8(0x00U, v[16]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, v[17]);
}

static void test_c37238_value_layout_2017(void)
{
	ptp_c37238_t p;
	uint8_t v[PTP_C37238_VALUE_LEN];

	ptp_c37238_defaults(&p);
	p.v2017 = true;
	p.total_inaccuracy_ns = 0x0000FFFEUL;
	/* 2011-only fields must not appear in the 2017 encoding. */
	p.grandmaster_id = 0xBEEFU;
	p.gm_inaccuracy_ns = 0x11223344UL;

	TEST_ASSERT_EQUAL_INT(0, ptp_c37238_value_encode(v, sizeof(v), &p));

	TEST_ASSERT_EQUAL_HEX8(0x1CU, v[0]);
	TEST_ASSERT_EQUAL_HEX8(0x12U, v[1]);
	TEST_ASSERT_EQUAL_HEX8(0x9DU, v[2]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, v[3]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, v[4]);
	TEST_ASSERT_EQUAL_HEX8(0x02U, v[5]);
	/* reserved where 2011 put grandmasterID and gmTimeInaccuracy. */
	TEST_ASSERT_EQUAL_HEX8(0x00U, v[6]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, v[7]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, v[8]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, v[11]);
	/* totalTimeInaccuracy. */
	TEST_ASSERT_EQUAL_HEX8(0x00U, v[12]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, v[13]);
	TEST_ASSERT_EQUAL_HEX8(0xFFU, v[14]);
	TEST_ASSERT_EQUAL_HEX8(0xFEU, v[15]);
}

static void test_c37238_roundtrip_and_rejects(void)
{
	ptp_c37238_t p;
	ptp_c37238_t got;
	uint8_t v[PTP_C37238_VALUE_LEN];

	ptp_c37238_defaults(&p);
	p.grandmaster_id = 0x0102U;
	p.gm_inaccuracy_ns = 1000U;
	p.network_inaccuracy_ns = 2000U;
	TEST_ASSERT_EQUAL_INT(0, ptp_c37238_value_encode(v, sizeof(v), &p));
	TEST_ASSERT_EQUAL_INT(0, ptp_c37238_value_decode(v, sizeof(v), &got));
	TEST_ASSERT_FALSE(got.v2017);
	TEST_ASSERT_EQUAL_HEX16(0x0102U, got.grandmaster_id);
	TEST_ASSERT_EQUAL_UINT32(1000U, got.gm_inaccuracy_ns);
	TEST_ASSERT_EQUAL_UINT32(2000U, got.network_inaccuracy_ns);

	p.v2017 = true;
	p.total_inaccuracy_ns = 12345U;
	TEST_ASSERT_EQUAL_INT(0, ptp_c37238_value_encode(v, sizeof(v), &p));
	TEST_ASSERT_EQUAL_INT(0, ptp_c37238_value_decode(v, sizeof(v), &got));
	TEST_ASSERT_TRUE(got.v2017);
	TEST_ASSERT_EQUAL_UINT32(12345U, got.total_inaccuracy_ns);

	/* Wrong length, wrong organization, and NULLs. */
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
		ptp_c37238_value_decode(v, sizeof(v) - 1U, &got));
	v[0] = 0xDEU;
	TEST_ASSERT_EQUAL_INT(-EPROTO, ptp_c37238_value_decode(v, sizeof(v), &got));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_c37238_value_decode(NULL, 18U, &got));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_c37238_value_decode(v, 18U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_c37238_value_encode(NULL, 18U, &p));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_c37238_value_encode(v, 18U, NULL));
	TEST_ASSERT_EQUAL_INT(-ENOSPC, ptp_c37238_value_encode(v, 17U, &p));
	ptp_c37238_defaults(NULL); /* must not crash */
}

/* The Power profile actually puts the TLV on the wire, and messageLength grows. */
static void test_power_profile_announce_carries_the_tlv(void)
{
	ptp_port_ctx_t c;
	ptp_cfg_t cfg;
	fake_t f;
	const uint8_t *a;
	size_t len = 0U;
	ptp_c37238_t got;
	uint16_t wire_len;

	ptp_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_apply_profile(&cfg,
					(uint8_t)PTP_PROFILE_POWER_C37_238));
	cfg.c37238.grandmaster_id = 0x0007U;
	cfg.c37238.gm_inaccuracy_ns = 100U;
	cfg.c37238.network_inaccuracy_ns = 200U;

	run_to_announce(&c, &f, &cfg);

	a = find_tx(&f, (uint8_t)PTP_MSG_ANNOUNCE, &len);
	TEST_ASSERT_NOT_NULL(a);
	TEST_ASSERT_EQUAL_UINT(PTP_ANNOUNCE_LEN + PTP_C37238_TLV_LEN, len);

	wire_len = (uint16_t)(((uint16_t)a[2] << 8) | (uint16_t)a[3]);
	TEST_ASSERT_EQUAL_UINT16(PTP_ANNOUNCE_LEN + PTP_C37238_TLV_LEN, wire_len);

	/* tlvType ORGANIZATION_EXTENSION, lengthField 18. */
	TEST_ASSERT_EQUAL_HEX8(0x00U, a[PTP_ANNOUNCE_LEN + 0U]);
	TEST_ASSERT_EQUAL_HEX8(0x03U, a[PTP_ANNOUNCE_LEN + 1U]);
	TEST_ASSERT_EQUAL_HEX8(0x00U, a[PTP_ANNOUNCE_LEN + 2U]);
	TEST_ASSERT_EQUAL_HEX8(PTP_C37238_VALUE_LEN, a[PTP_ANNOUNCE_LEN + 3U]);

	TEST_ASSERT_EQUAL_INT(0, ptp_c37238_find(a, len, &got));
	TEST_ASSERT_EQUAL_HEX16(0x0007U, got.grandmaster_id);
	TEST_ASSERT_EQUAL_UINT32(100U, got.gm_inaccuracy_ns);
	TEST_ASSERT_EQUAL_UINT32(200U, got.network_inaccuracy_ns);

	TEST_ASSERT_EQUAL_UINT32(0U,
		ptp_port_counters(&c)->tx_profile_tlv_errors);

	/* And a Default-profile Announce has no such TLV to find. */
	ptp_cfg_defaults(&cfg);
	run_to_announce(&c, &f, &cfg);
	a = find_tx(&f, (uint8_t)PTP_MSG_ANNOUNCE, &len);
	TEST_ASSERT_NOT_NULL(a);
	TEST_ASSERT_EQUAL_INT(-ENOENT, ptp_c37238_find(a, len, &got));
}

/* ===================================================================== *
 *  5. TLV suffix mechanics
 * ===================================================================== */

/* A minimal well-formed Announce with no TLVs, for the iterator tests. */
static size_t make_announce(uint8_t *buf, size_t cap)
{
	ptp_hdr_t h;
	ptp_announce_t a;
	size_t len = 0U;

	(void)memset(&h, 0, sizeof(h));
	(void)memset(&a, 0, sizeof(a));
	h.msg_type = (uint8_t)PTP_MSG_ANNOUNCE;
	h.version = PTP_VERSION;
	h.minor_version = PTP_MINOR_VERSION_2019;
	TEST_ASSERT_EQUAL_INT(0, ptp_announce_encode(buf, cap, &h, &a, &len));
	return len;
}

static void test_tlv_append_and_iterate(void)
{
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	static const uint8_t v1[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
	static const uint8_t v2[2] = { 0x01, 0x02 };
	size_t off1 = 0U;
	size_t off2 = 0U;
	ptp_tlv_iter_t it;
	uint16_t type;
	const uint8_t *val;
	size_t vlen;
	size_t tlv_off;

	len = make_announce(buf, sizeof(buf));
	TEST_ASSERT_EQUAL_UINT(PTP_ANNOUNCE_LEN, len);

	TEST_ASSERT_EQUAL_INT(0, ptp_tlv_append(buf, sizeof(buf), &len, 0x1111U,
						v1, sizeof(v1), &off1));
	TEST_ASSERT_EQUAL_UINT(PTP_ANNOUNCE_LEN + 8U, len);
	TEST_ASSERT_EQUAL_UINT(PTP_ANNOUNCE_LEN + 4U, off1);

	TEST_ASSERT_EQUAL_INT(0, ptp_tlv_append(buf, sizeof(buf), &len, 0x2222U,
						v2, sizeof(v2), &off2));
	TEST_ASSERT_EQUAL_UINT(PTP_ANNOUNCE_LEN + 14U, len);

	/* messageLength tracks both. */
	TEST_ASSERT_EQUAL_UINT16(len, ((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);

	TEST_ASSERT_EQUAL_INT(0, ptp_tlv_iter_begin(&it, buf, len));
	TEST_ASSERT_EQUAL_INT(0,
		ptp_tlv_iter_next(&it, &type, &val, &vlen, &tlv_off));
	TEST_ASSERT_EQUAL_HEX16(0x1111U, type);
	TEST_ASSERT_EQUAL_UINT(4U, vlen);
	TEST_ASSERT_EQUAL_UINT(PTP_ANNOUNCE_LEN, tlv_off);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(v1, val, 4U);

	TEST_ASSERT_EQUAL_INT(0,
		ptp_tlv_iter_next(&it, &type, &val, &vlen, &tlv_off));
	TEST_ASSERT_EQUAL_HEX16(0x2222U, type);
	TEST_ASSERT_EQUAL_UINT(2U, vlen);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(v2, val, 2U);

	TEST_ASSERT_EQUAL_INT(-ENOENT,
		ptp_tlv_iter_next(&it, &type, &val, &vlen, &tlv_off));

	/* Optional outputs may all be NULL. */
	TEST_ASSERT_EQUAL_INT(0, ptp_tlv_iter_begin(&it, buf, len));
	TEST_ASSERT_EQUAL_INT(0,
		ptp_tlv_iter_next(&it, NULL, NULL, NULL, NULL));
}

static void test_tlv_append_rejects(void)
{
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	static const uint8_t v[3] = { 1, 2, 3 };

	len = make_announce(buf, sizeof(buf));

	TEST_ASSERT_EQUAL_INT(-EINVAL,
		ptp_tlv_append(NULL, sizeof(buf), &len, 1U, v, 2U, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		ptp_tlv_append(buf, sizeof(buf), NULL, 1U, v, 2U, NULL));
	/* NULL value with a non-zero length. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		ptp_tlv_append(buf, sizeof(buf), &len, 1U, NULL, 2U, NULL));
	/* §14.1: an odd lengthField is refused rather than silently padded. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		ptp_tlv_append(buf, sizeof(buf), &len, 1U, v, 3U, NULL));

	/* No room. */
	TEST_ASSERT_EQUAL_INT(-ENOSPC,
		ptp_tlv_append(buf, len + 4U, &len, 1U, v, 2U, NULL));

	/* A "message" shorter than a header is not one. */
	{
		size_t bad = PTP_HDR_LEN - 1U;

		TEST_ASSERT_EQUAL_INT(-EINVAL,
			ptp_tlv_append(buf, sizeof(buf), &bad, 1U, v, 2U, NULL));
	}

	/* An empty value is legal. */
	TEST_ASSERT_EQUAL_INT(0,
		ptp_tlv_append(buf, sizeof(buf), &len, 1U, NULL, 0U, NULL));
}

static void test_tlv_iter_rejects_malformed(void)
{
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	ptp_tlv_iter_t it;

	len = make_announce(buf, sizeof(buf));

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_tlv_iter_begin(&it, NULL, len));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_tlv_iter_begin(NULL, buf, len));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
		ptp_tlv_iter_begin(&it, buf, PTP_HDR_LEN - 1U));

	/* messageLength beyond the octets we were handed. */
	buf[2] = 0xFFU;
	buf[3] = 0xFFU;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ptp_tlv_iter_begin(&it, buf, len));

	/* messageLength shorter than the fixed fields of the type. */
	buf[2] = 0x00U;
	buf[3] = 0x10U;
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ptp_tlv_iter_begin(&it, buf, len));

	/* A trailing stub too short to be a TLV header. */
	len = make_announce(buf, sizeof(buf));
	buf[len] = 0x00U;
	buf[len + 1U] = 0x03U;
	len += 2U;
	buf[2] = (uint8_t)(len >> 8);
	buf[3] = (uint8_t)(len & 0xFFU);
	TEST_ASSERT_EQUAL_INT(0, ptp_tlv_iter_begin(&it, buf, len));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
		ptp_tlv_iter_next(&it, NULL, NULL, NULL, NULL));

	/* A lengthField that runs past messageLength. */
	len = make_announce(buf, sizeof(buf));
	buf[len + 0U] = 0x00U;
	buf[len + 1U] = 0x03U;
	buf[len + 2U] = 0x00U;
	buf[len + 3U] = 0x40U; /* 64 octets of value that are not there */
	len += 4U;
	buf[2] = (uint8_t)(len >> 8);
	buf[3] = (uint8_t)(len & 0xFFU);
	TEST_ASSERT_EQUAL_INT(0, ptp_tlv_iter_begin(&it, buf, len));
	TEST_ASSERT_EQUAL_INT(-EBADMSG,
		ptp_tlv_iter_next(&it, NULL, NULL, NULL, NULL));

	/* NULL iterator, and an iterator that was never begun. */
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		ptp_tlv_iter_next(NULL, NULL, NULL, NULL, NULL));
	(void)memset(&it, 0, sizeof(it));
	TEST_ASSERT_EQUAL_INT(-ENOENT,
		ptp_tlv_iter_next(&it, NULL, NULL, NULL, NULL));
}

/*
 * 802.3 pads a short frame to 60 octets. Those pad bytes sit past
 * messageLength and must never be mistaken for a TLV — a pad of zeros would
 * otherwise decode as tlvType 0 with a zero-length value, forever.
 */
static void test_tlv_iter_ignores_ethernet_padding(void)
{
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	ptp_tlv_iter_t it;

	len = make_announce(buf, sizeof(buf));
	(void)memset(&buf[len], 0, 16U);

	TEST_ASSERT_EQUAL_INT(0, ptp_tlv_iter_begin(&it, buf, len + 16U));
	TEST_ASSERT_EQUAL_INT(-ENOENT,
		ptp_tlv_iter_next(&it, NULL, NULL, NULL, NULL));
}

static void test_c37238_find_skips_other_vendors(void)
{
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	uint8_t other[PTP_C37238_VALUE_LEN];
	ptp_c37238_t p;
	ptp_c37238_t got;

	len = make_announce(buf, sizeof(buf));

	/* Somebody else's organization TLV first. */
	(void)memset(other, 0, sizeof(other));
	other[0] = 0xACU;
	other[1] = 0xDEU;
	other[2] = 0x48U;
	TEST_ASSERT_EQUAL_INT(0, ptp_tlv_append(buf, sizeof(buf), &len,
		(uint16_t)PTP_TLV_TYPE_ORGANIZATION_EXTENSION, other,
		sizeof(other), NULL));
	TEST_ASSERT_EQUAL_INT(-ENOENT, ptp_c37238_find(buf, len, &got));

	/* Then ours: found, and the other one did not confuse the search. */
	ptp_c37238_defaults(&p);
	p.grandmaster_id = 0x55AAU;
	TEST_ASSERT_EQUAL_INT(0, ptp_c37238_append(buf, sizeof(buf), &len, &p));
	TEST_ASSERT_EQUAL_INT(0, ptp_c37238_find(buf, len, &got));
	TEST_ASSERT_EQUAL_HEX16(0x55AAU, got.grandmaster_id);

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_c37238_find(NULL, len, &got));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_c37238_find(buf, len, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		ptp_c37238_append(NULL, sizeof(buf), &len, &p));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		ptp_c37238_append(buf, sizeof(buf), NULL, &p));
	TEST_ASSERT_EQUAL_INT(-EINVAL,
		ptp_c37238_append(buf, sizeof(buf), &len, NULL));
}

/*
 * A truncated C37.238 TLV is reported as a protocol error, not as "absent":
 * silently treating a mangled TLV from our own organization as missing would
 * hide a genuinely broken peer.
 */
static void test_c37238_find_reports_a_mangled_own_tlv(void)
{
	uint8_t buf[PTP_MSG_MAX_LEN];
	size_t len;
	uint8_t short_val[4] = { 0x00, 0x1B, 0x19, 0x00 };
	ptp_c37238_t got;

	len = make_announce(buf, sizeof(buf));
	TEST_ASSERT_EQUAL_INT(0, ptp_tlv_append(buf, sizeof(buf), &len,
		(uint16_t)PTP_TLV_TYPE_ORGANIZATION_EXTENSION, short_val,
		sizeof(short_val), NULL));
	TEST_ASSERT_EQUAL_INT(-EPROTO, ptp_c37238_find(buf, len, &got));
}

/* ===================================================================== *
 *  Runner
 * ===================================================================== */

int main(void)
{
	UNITY_BEGIN();

	/* Default-profile regression — these must never change. */
	RUN_TEST(test_default_descriptor_matches_the_engine_defaults);
	RUN_TEST(test_default_clock_class_ladder_is_unchanged);
	RUN_TEST(test_default_announce_has_no_tlv_suffix);
	RUN_TEST(test_default_and_power_ignore_local_priority);

	/* Published parameters. */
	RUN_TEST(test_g8275_1_parameters);
	RUN_TEST(test_g8275_2_parameters);
	RUN_TEST(test_c37238_parameters);
	RUN_TEST(test_out_of_range_profile_degrades_to_default);
	RUN_TEST(test_deviation_text);
	RUN_TEST(test_apply_profile_produces_valid_cfg);
	RUN_TEST(test_apply_profile_preserves_site_settings);
	RUN_TEST(test_profile_ranges_are_enforced_only_off_default);

	/* Telecom ladder + alternate BMCA. */
	RUN_TEST(test_telecom_clock_class_ladder);
	RUN_TEST(test_freq_cat_classification);
	RUN_TEST(test_ladder_null_and_bounds);
	RUN_TEST(test_local_priority_decides_the_election);

	/* C37.238 TLV. */
	RUN_TEST(test_c37238_value_layout_2011);
	RUN_TEST(test_c37238_value_layout_2017);
	RUN_TEST(test_c37238_roundtrip_and_rejects);
	RUN_TEST(test_power_profile_announce_carries_the_tlv);

	/* TLV suffix mechanics. */
	RUN_TEST(test_tlv_append_and_iterate);
	RUN_TEST(test_tlv_append_rejects);
	RUN_TEST(test_tlv_iter_rejects_malformed);
	RUN_TEST(test_tlv_iter_ignores_ethernet_padding);
	RUN_TEST(test_c37238_find_skips_other_vendors);
	RUN_TEST(test_c37238_find_reports_a_mangled_own_tlv);

	return UNITY_END();
}
