/*
 * STS1000 "Meridian" — core/ptp port-engine unit tests.
 *
 * What anchors the expectations:
 *
 *  - Emitted PDUs are inspected at the octet level against the IEEE 1588-2019
 *    §13 field offsets written out here independently of the codec, so a test
 *    passes because the bytes are right, not because the encoder and decoder
 *    agree. The codec's own byte vectors live in test_ptp_msg.c.
 *
 *  - State transitions follow §9.2.5/§9.3.3 as scoped in ptp.h: a grandmaster
 *    ordinary clock occupies INITIALIZING, LISTENING, MASTER, PASSIVE, FAULTY.
 *
 *  - clockClass values are Table 4, clockAccuracy Table 5, timeSource Table 6.
 *
 *  - Cadence is checked over 30 simulated seconds stepped a millisecond at a
 *    time, which is the only honest way to test a scheduler that must not drift.
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include "unity.h"

#include "ptp/ptp.h"
#include "test_support.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* Octet offsets, written out here rather than taken from the codec. */
#define O_TYPE        0x00U
#define O_VERSION     0x01U
#define O_LENGTH      0x02U
#define O_DOMAIN      0x04U
#define O_FLAGS       0x06U
#define O_CORRECTION  0x08U
#define O_SRC_CLOCK   0x14U
#define O_SRC_PORT    0x1CU
#define O_SEQ         0x1EU
#define O_CONTROL     0x20U
#define O_LOG_INT     0x21U
#define O_BODY_TS     0x22U
#define O_ANN_UTC     0x2CU
#define O_ANN_PRIO1   0x2FU
#define O_ANN_CLASS   0x30U
#define O_ANN_ACCUR   0x31U
#define O_ANN_OSLV    0x32U
#define O_ANN_PRIO2   0x34U
#define O_ANN_GMID    0x35U
#define O_ANN_STEPS   0x3DU
#define O_ANN_TSRC    0x3FU
#define O_DRESP_REQ   0x2CU

static const uint8_t self_mac[6] = { 0x00, 0x80, 0xE1, 0x11, 0x22, 0x33 };
static const uint8_t self_eui64[8] = {
	0x00, 0x80, 0xE1, 0xFF, 0xFE, 0x11, 0x22, 0x33
};

/* ------------------------------------------------------------- test glue -- */

#define FAKE_MAX_TX 256U

typedef struct {
	struct {
		uint8_t buf[PTP_MSG_MAX_LEN];
		size_t len;
		uint8_t msg_type;
		uint16_t seq;
		ptp_port_kind_t port_kind;
		ptp_addr_hint_t addr;
		ptp_port_id_t peer;
		ptp_transport_t transport;
	} tx[FAKE_MAX_TX];
	size_t n;
	size_t overflow;
	int tx_rc;
	int sync_rc;              /* overrides tx_rc for Sync only */
	uint64_t tai_ns;
	bool tai_fails;
	bool have_tai;

	/*
	 * Synchronous-egress-timestamp glue: report the Sync's timestamp from
	 * inside its own transmit callback, which is what a MAC that can read
	 * its own TX timestamp does. The Follow_Up is then built and sent while
	 * the Sync descriptor is still live, so this is also where the buffer
	 * aliasing is checked.
	 */
	ptp_port_ctx_t *nested_ctx;
	bool nested_txts;
	uint64_t nested_ts;
	int nested_txts_rc;
	bool nested_saw_mutation;
} fake_t;

static int fake_tx(void *ctx, const ptp_tx_desc_t *d)
{
	fake_t *f = (fake_t *)ctx;
	bool is_sync = (d->msg_type == (uint8_t)PTP_MSG_SYNC);
	int rc = f->tx_rc;

	TEST_ASSERT_NOT_NULL(d->buf);
	TEST_ASSERT_LESS_OR_EQUAL_size_t(PTP_MSG_MAX_LEN, d->len);

	if (is_sync && (f->sync_rc != 0)) {
		rc = f->sync_rc;
	}

	if (is_sync && f->nested_txts && (f->nested_ctx != NULL)) {
		uint8_t snapshot[PTP_MSG_MAX_LEN];

		memcpy(snapshot, d->buf, d->len);
		f->nested_txts_rc =
			ptp_on_sync_txts(f->nested_ctx, d->seq, f->nested_ts);
		/*
		 * The Follow_Up has now been encoded and transmitted. The Sync
		 * bytes the glue is still holding must be exactly as handed over.
		 */
		if (memcmp(snapshot, d->buf, d->len) != 0) {
			f->nested_saw_mutation = true;
		}
	}

	if (rc != 0) {
		return rc;
	}
	if (f->n >= FAKE_MAX_TX) {
		f->overflow++;
		return 0;
	}

	memcpy(f->tx[f->n].buf, d->buf, d->len);
	f->tx[f->n].len = d->len;
	f->tx[f->n].msg_type = d->msg_type;
	f->tx[f->n].seq = d->seq;
	f->tx[f->n].port_kind = d->port_kind;
	f->tx[f->n].addr = d->addr;
	f->tx[f->n].peer = d->peer;
	f->tx[f->n].transport = d->transport;
	f->n++;
	return 0;
}

static int fake_tai(void *ctx, uint64_t *out_ns)
{
	fake_t *f = (fake_t *)ctx;

	if (f->tai_fails) {
		return -EIO;
	}
	*out_ns = f->tai_ns;
	return 0;
}

static void fake_init(fake_t *f)
{
	memset(f, 0, sizeof(*f));
	f->have_tai = true;
}

static void ops_from(ptp_port_ops_t *ops, fake_t *f)
{
	memset(ops, 0, sizeof(*ops));
	ops->tx = fake_tx;
	ops->tai_ns = f->have_tai ? fake_tai : NULL;
	ops->ctx = f;
}

/* Index of the n-th emitted message of a given type, or SIZE_MAX. */
static size_t find_tx(const fake_t *f, uint8_t type, size_t nth)
{
	size_t i;
	size_t hit = 0U;

	for (i = 0U; i < f->n; i++) {
		if (f->tx[i].msg_type != type) {
			continue;
		}
		if (hit == nth) {
			return i;
		}
		hit++;
	}
	return SIZE_MAX;
}

static size_t count_tx(const fake_t *f, uint8_t type)
{
	size_t i;
	size_t n = 0U;

	for (i = 0U; i < f->n; i++) {
		if (f->tx[i].msg_type == type) {
			n++;
		}
	}
	return n;
}

static uint16_t be16(const uint8_t *p)
{
	return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* --------------------------------------------------------- announce maker -- */

/*
 * Build a foreign Announce by hand, so the state machine is driven by wire
 * bytes and not by a struct the engine also produces.
 */
typedef struct {
	uint8_t clock_last;   /* last octet of the sender's clockIdentity */
	bool use_self_id;     /* send it with *our* clockIdentity (loopback case) */
	uint16_t port_number;
	uint8_t domain;
	uint8_t major_sdo_id;
	uint8_t priority1;
	uint8_t clock_class;
	uint8_t accuracy;
	uint16_t oslv;
	uint8_t priority2;
	uint8_t gm_last;      /* last octet of grandmasterIdentity */
	uint16_t steps_removed;
	int8_t log_interval;
	uint16_t seq;
} foreign_spec_t;

static void spec_defaults(foreign_spec_t *s)
{
	memset(s, 0, sizeof(*s));
	s->clock_last = 0xAAU;
	s->port_number = 1U;
	s->priority1 = 128U;
	s->clock_class = 6U;
	s->accuracy = 0x20U;
	s->oslv = 0x4E5DU;
	s->priority2 = 128U;
	s->gm_last = 0xAAU;
	s->steps_removed = 1U;
	s->log_interval = 1;
}

static size_t build_announce(uint8_t *buf, const foreign_spec_t *s)
{
	static const uint8_t base_id[8] = {
		0x00, 0x80, 0xE1, 0xFF, 0xFE, 0x00, 0x00, 0x00
	};

	memset(buf, 0, PTP_ANNOUNCE_LEN);
	buf[O_TYPE] = (uint8_t)((s->major_sdo_id << 4) | (uint8_t)PTP_MSG_ANNOUNCE);
	buf[O_VERSION] = 0x12U;
	buf[O_LENGTH] = 0x00U;
	buf[O_LENGTH + 1U] = 0x40U;      /* 64 */
	buf[O_DOMAIN] = s->domain;
	buf[O_FLAGS] = 0x00U;
	buf[O_FLAGS + 1U] = 0x0CU;       /* utcOffsetValid | ptpTimescale */
	if (s->use_self_id) {
		memcpy(&buf[O_SRC_CLOCK], self_eui64, 8);
	} else {
		memcpy(&buf[O_SRC_CLOCK], base_id, 8);
		buf[O_SRC_CLOCK + 7U] = s->clock_last;
	}
	buf[O_SRC_PORT] = (uint8_t)(s->port_number >> 8);
	buf[O_SRC_PORT + 1U] = (uint8_t)(s->port_number & 0xFFU);
	buf[O_SEQ] = (uint8_t)(s->seq >> 8);
	buf[O_SEQ + 1U] = (uint8_t)(s->seq & 0xFFU);
	buf[O_CONTROL] = 5U;
	buf[O_LOG_INT] = (uint8_t)s->log_interval;
	buf[O_ANN_UTC] = 0x00U;
	buf[O_ANN_UTC + 1U] = 0x25U;     /* 37 */
	buf[O_ANN_PRIO1] = s->priority1;
	buf[O_ANN_CLASS] = s->clock_class;
	buf[O_ANN_ACCUR] = s->accuracy;
	buf[O_ANN_OSLV] = (uint8_t)(s->oslv >> 8);
	buf[O_ANN_OSLV + 1U] = (uint8_t)(s->oslv & 0xFFU);
	buf[O_ANN_PRIO2] = s->priority2;
	memcpy(&buf[O_ANN_GMID], base_id, 8);
	buf[O_ANN_GMID + 7U] = s->gm_last;
	buf[O_ANN_STEPS] = (uint8_t)(s->steps_removed >> 8);
	buf[O_ANN_STEPS + 1U] = (uint8_t)(s->steps_removed & 0xFFU);
	buf[O_ANN_TSRC] = 0x20U;
	return PTP_ANNOUNCE_LEN;
}

static size_t build_delay_req(uint8_t *buf, uint8_t clock_last, uint16_t port,
			      uint16_t seq, int64_t correction, bool unicast)
{
	static const uint8_t base_id[8] = {
		0x00, 0x80, 0xE1, 0xFF, 0xFE, 0x00, 0x00, 0x00
	};
	uint64_t raw = (correction < 0)
			       ? ~(uint64_t)(-(correction + 1))
			       : (uint64_t)correction;
	unsigned int i;

	memset(buf, 0, PTP_TSMSG_LEN);
	buf[O_TYPE] = (uint8_t)PTP_MSG_DELAY_REQ;
	buf[O_VERSION] = 0x12U;
	buf[O_LENGTH] = 0x00U;
	buf[O_LENGTH + 1U] = 0x2CU;      /* 44 */
	buf[O_FLAGS] = unicast ? 0x04U : 0x00U;
	for (i = 0U; i < 8U; i++) {
		buf[O_CORRECTION + i] = (uint8_t)((raw >> (56U - (8U * i))) & 0xFFU);
	}
	memcpy(&buf[O_SRC_CLOCK], base_id, 8);
	buf[O_SRC_CLOCK + 7U] = clock_last;
	buf[O_SRC_PORT] = (uint8_t)(port >> 8);
	buf[O_SRC_PORT + 1U] = (uint8_t)(port & 0xFFU);
	buf[O_SEQ] = (uint8_t)(seq >> 8);
	buf[O_SEQ + 1U] = (uint8_t)(seq & 0xFFU);
	buf[O_CONTROL] = 1U;
	buf[O_LOG_INT] = 0x7FU;
	return PTP_TSMSG_LEN;
}

/* Feed the same Announce twice, far enough apart to qualify the sender. */
static void announce_twice(ptp_port_ctx_t *c, const foreign_spec_t *s,
			   uint64_t t0, uint64_t t1)
{
	uint8_t buf[PTP_ANNOUNCE_LEN];
	foreign_spec_t local = *s;
	size_t len;

	local.seq = 1U;
	len = build_announce(buf, &local);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_rx(c, buf, len, 0U, t0));

	local.seq = 2U;
	len = build_announce(buf, &local);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_rx(c, buf, len, 0U, t1));
}

/* ------------------------------------------------------------------- cfg -- */

static void test_cfg_defaults(void)
{
	ptp_cfg_t cfg;

	memset(&cfg, 0xA5, sizeof(cfg));
	ptp_cfg_defaults(&cfg);

	TEST_ASSERT_EQUAL_HEX8(0U, cfg.domain);
	TEST_ASSERT_EQUAL_HEX8(128U, cfg.priority1);
	TEST_ASSERT_EQUAL_HEX8(128U, cfg.priority2);
	TEST_ASSERT_EQUAL_HEX16(1U, cfg.port_number);
	TEST_ASSERT_EQUAL_INT8(1, cfg.log_announce_interval);
	TEST_ASSERT_EQUAL_INT8(0, cfg.log_sync_interval);
	TEST_ASSERT_EQUAL_INT8(0, cfg.log_min_delay_req_interval);
	TEST_ASSERT_EQUAL_HEX8(3U, cfg.announce_receipt_timeout);
	TEST_ASSERT_EQUAL_INT(PTP_TRANSPORT_UDP_IPV4, cfg.transport);
	TEST_ASSERT_EQUAL_INT(PTP_PROFILE_DEFAULT, cfg.profile);
	TEST_ASSERT_EQUAL_INT(PTP_DEGRADE_ALT_A, cfg.degradation);
	TEST_ASSERT_EQUAL_HEX16(PTP_OSLV_DEFAULT_LOCKED, cfg.oslv_locked);

	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_validate(&cfg));

	ptp_cfg_defaults(NULL); /* must not trap */
}

static void test_cfg_validate_rejects(void)
{
	ptp_cfg_t cfg;

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_cfg_validate(NULL));

	ptp_cfg_defaults(&cfg);
	cfg.log_announce_interval = 8;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_cfg_validate(&cfg));

	ptp_cfg_defaults(&cfg);
	cfg.log_sync_interval = -8;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_cfg_validate(&cfg));

	ptp_cfg_defaults(&cfg);
	cfg.log_min_delay_req_interval = PTP_LOG_INTERVAL_UNSPEC;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_cfg_validate(&cfg));

	/* §7.7.3.1 requires N >= 2. */
	ptp_cfg_defaults(&cfg);
	cfg.announce_receipt_timeout = 1U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_cfg_validate(&cfg));
	cfg.announce_receipt_timeout = 2U;
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_validate(&cfg));

	/* portNumber 0 is reserved. */
	ptp_cfg_defaults(&cfg);
	cfg.port_number = 0U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_cfg_validate(&cfg));

	/*
	 * majorSdoId is a nibble. A wider value would be truncated by the
	 * encoder, silently moving the clock to a different SDO from the one
	 * configured — and it would then ignore the peers it was meant to hear.
	 */
	ptp_cfg_defaults(&cfg);
	cfg.major_sdo_id = 0x0FU;
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_validate(&cfg));
	cfg.major_sdo_id = 0x10U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_cfg_validate(&cfg));
	cfg.major_sdo_id = 0xFFU;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_cfg_validate(&cfg));

	ptp_cfg_defaults(&cfg);
	cfg.transport = (ptp_transport_t)PTP_TRANSPORT_COUNT;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_cfg_validate(&cfg));
	cfg.transport = (ptp_transport_t)-1;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_cfg_validate(&cfg));

	ptp_cfg_defaults(&cfg);
	cfg.profile = (ptp_profile_t)PTP_PROFILE_COUNT;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_cfg_validate(&cfg));
	cfg.profile = (ptp_profile_t)-1;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_cfg_validate(&cfg));

	ptp_cfg_defaults(&cfg);
	cfg.degradation = (ptp_degradation_t)PTP_DEGRADE_COUNT;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_cfg_validate(&cfg));
	cfg.degradation = (ptp_degradation_t)-1;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_cfg_validate(&cfg));

	ptp_cfg_defaults(&cfg);
	cfg.l2_mac = (uint8_t)PTP_L2_MAC_COUNT;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_cfg_validate(&cfg));

	/* Every transport and degradation in range is accepted on Default. */
	ptp_cfg_defaults(&cfg);
	cfg.transport = PTP_TRANSPORT_UDP_IPV6;
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_validate(&cfg));
	cfg.transport = PTP_TRANSPORT_L2;
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_validate(&cfg));
	cfg.degradation = PTP_DEGRADE_ALT_B;
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_validate(&cfg));

	/*
	 * Selecting a non-Default profile is no longer a free-standing label: the
	 * profile's own ranges are normative and are enforced. Default-profile
	 * parameters (domain 0, 2 s Announce, UDP/IPv4) are not G.8275.1
	 * parameters, so merely flipping the selector must be rejected — and
	 * ptp_cfg_apply_profile() must produce a set that validates.
	 */
	ptp_cfg_defaults(&cfg);
	cfg.profile = PTP_PROFILE_TELECOM_G8275_1;
	TEST_ASSERT_EQUAL_INT(-ERANGE, ptp_cfg_validate(&cfg));
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_apply_profile(&cfg,
					(uint8_t)PTP_PROFILE_TELECOM_G8275_1));
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_validate(&cfg));
}

/* -------------------------------------------------------- quality mapping -- */

static void test_clock_class_mapping(void)
{
	ptp_cfg_t cfg;
	ptp_quality_view_t q;
	ptp_clock_quality_t out;

	ptp_cfg_defaults(&cfg);
	memset(&q, 0, sizeof(q));

	/*
	 * The degradation ladder describes a clock that HAS been disciplined.
	 * A never-locked clock advertises 248 instead (the never-locked gate in
	 * ptp_port.c); that case is asserted at the end of this function.
	 */
	q.ever_locked = true;

	q.sync_state = PTP_SYNC_LOCKED;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_EQUAL_HEX8(6U, out.clock_class);

	q.sync_state = PTP_SYNC_HOLDOVER;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_EQUAL_HEX8(7U, out.clock_class);

	/* Table 4 degradation alternative A. */
	q.sync_state = PTP_SYNC_HOLDOVER_EXCEEDED;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_EQUAL_HEX8(52U, out.clock_class);

	q.sync_state = PTP_SYNC_FREERUN;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_EQUAL_HEX8(52U, out.clock_class);

	/* ...and alternative B. */
	cfg.degradation = PTP_DEGRADE_ALT_B;
	q.sync_state = PTP_SYNC_HOLDOVER_EXCEEDED;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_EQUAL_HEX8(187U, out.clock_class);

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_clock_quality_from_view(NULL, &q, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_clock_quality_from_view(&cfg, NULL, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_clock_quality_from_view(&cfg, &q, NULL));
	/*
	 * And the never-locked gate: identical view, ever_locked clear, and the
	 * answer becomes 248 under both degradation alternatives. Only free-run
	 * is gated — holdover implies a prior lock, so it keeps its own class.
	 */
	q.ever_locked = false;
	q.sync_state = PTP_SYNC_FREERUN;
	cfg.degradation = PTP_DEGRADE_ALT_A;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_EQUAL_HEX8(248U, out.clock_class);
	cfg.degradation = PTP_DEGRADE_ALT_B;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_EQUAL_HEX8(248U, out.clock_class);

}

static void test_clock_accuracy_ladder(void)
{
	static const struct {
		uint64_t ns;
		uint8_t code;
	} cases[] = {
		{ 0U, PTP_CLOCK_ACCURACY_UNKNOWN },
		{ 1U, 0x1DU },
		{ 2U, 0x1EU },
		{ 3U, 0x1FU },
		{ 10U, 0x1FU },
		{ 11U, 0x20U },
		{ 25U, 0x20U },
		{ 26U, 0x21U },
		{ 100U, 0x21U },
		{ 250U, 0x22U },
		{ 1000U, 0x23U },
		{ 2500U, 0x24U },
		{ 10000U, 0x25U },
		{ 25000U, 0x26U },
		{ 100000U, 0x27U },
		{ 250000U, 0x28U },
		{ 1000000U, 0x29U },
		{ 2500000U, 0x2AU },
		{ 10000000U, 0x2BU },
		{ 25000000U, 0x2CU },
		{ 100000000U, 0x2DU },
		{ 250000000U, 0x2EU },
		{ 1000000000U, 0x2FU },
		{ 10000000000ULL, 0x30U },
		{ 10000000001ULL, 0x31U },
		{ UINT64_MAX, 0x31U },
	};
	ptp_cfg_t cfg;
	ptp_quality_view_t q;
	ptp_clock_quality_t out;
	size_t i;

	ptp_cfg_defaults(&cfg);
	memset(&q, 0, sizeof(q));
	q.sync_state = PTP_SYNC_LOCKED;

	for (i = 0U; i < ARRAY_LEN(cases); i++) {
		q.est_accuracy_ns = cases[i].ns;
		TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
		TEST_ASSERT_EQUAL_HEX8(cases[i].code, out.clock_accuracy);
	}
}

static void test_offset_scaled_log_variance(void)
{
	ptp_cfg_t cfg;
	ptp_quality_view_t q;
	ptp_clock_quality_t out;

	ptp_cfg_defaults(&cfg);
	memset(&q, 0, sizeof(q));

	/* No ADEV, locked: the configured default. */
	q.sync_state = PTP_SYNC_LOCKED;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_EQUAL_HEX16(PTP_OSLV_DEFAULT_LOCKED, out.offset_scaled_log_variance);

	q.sync_state = PTP_SYNC_HOLDOVER;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_EQUAL_HEX16(PTP_OSLV_DEFAULT_LOCKED, out.offset_scaled_log_variance);

	/* No ADEV, not disciplined: unknown. */
	q.sync_state = PTP_SYNC_FREERUN;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_EQUAL_HEX16(PTP_OSLV_UNKNOWN, out.offset_scaled_log_variance);

	/*
	 * With an ADEV the value is computed. Worked by hand from §7.6.3.5 with
	 * PTPVAR = (ADEV * tau)^2:
	 *   ADEV 1e-12, tau = 1 s -> sigma^2 = 1e-24 s^2
	 *   log2(1e-24) = -24 * 3.3219280949 = -79.7263
	 *   256 * -79.7263 + 32768 = 12358.1  ->  0x3046
	 */
	q.sync_state = PTP_SYNC_LOCKED;
	q.adev_tau1_e18 = 1000000ULL; /* 1e-12 */
	cfg.log_sync_interval = 0;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_UINT16_WITHIN(2U, 12358U, out.offset_scaled_log_variance);

	/*
	 * Doubling tau doubles sigma, which is +2 in log2 of the variance, so
	 * the coded value rises by 512.
	 */
	cfg.log_sync_interval = 1;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_UINT16_WITHIN(2U, 12358U + 512U, out.offset_scaled_log_variance);

	cfg.log_sync_interval = -1;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_UINT16_WITHIN(2U, 12358U - 512U, out.offset_scaled_log_variance);

	/* A 1e-9 ADEV: log2(1e-18) = -59.7947, 256 * that + 32768 = 17460.5. */
	cfg.log_sync_interval = 0;
	q.adev_tau1_e18 = 1000000000ULL;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_UINT16_WITHIN(2U, 17460U, out.offset_scaled_log_variance);

	/* Monotonic in ADEV, and never the reserved 0xFFFF "unknown". */
	{
		uint16_t prev = 0U;
		uint64_t adev;

		for (adev = 1000ULL; adev < 100000000000000000ULL; adev *= 10ULL) {
			q.adev_tau1_e18 = adev;
			TEST_ASSERT_EQUAL_INT(0,
				ptp_clock_quality_from_view(&cfg, &q, &out));
			TEST_ASSERT_GREATER_THAN_UINT16(prev,
				out.offset_scaled_log_variance);
			TEST_ASSERT_NOT_EQUAL_HEX16(PTP_OSLV_UNKNOWN,
				out.offset_scaled_log_variance);
			prev = out.offset_scaled_log_variance;
		}
	}

	/* An implausibly good clock floors at 0 rather than going negative. */
	q.adev_tau1_e18 = 1ULL; /* 1e-18 */
	cfg.log_sync_interval = -7;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_EQUAL_HEX16(0U, out.offset_scaled_log_variance);

	/*
	 * The other end cannot reach the reserved 0xFFFF, and that is the point.
	 * With the ADEV a uint64 of 1e-18 units and tau capped at 2^7 s,
	 * log2(PTPVAR) tops out near 2*log2(1.8e19 * 1e-18 * 128) = 22.4, so
	 * 256*22.4 + 0x8000 = 38502 (0x9666) is the ceiling a computed value can
	 * reach. The clamp in oslv_from_adev() guards a future rescaling; it is
	 * not a live path.
	 */
	q.adev_tau1_e18 = UINT64_MAX;
	cfg.log_sync_interval = 7;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_UINT16_WITHIN(8U, 38502U, out.offset_scaled_log_variance);
	TEST_ASSERT_NOT_EQUAL_HEX16(PTP_OSLV_UNKNOWN, out.offset_scaled_log_variance);

	/* cfg.oslv_locked is honoured verbatim when there is no ADEV. */
	ptp_cfg_defaults(&cfg);
	cfg.oslv_locked = 0x1234U;
	q.adev_tau1_e18 = 0U;
	q.sync_state = PTP_SYNC_LOCKED;
	TEST_ASSERT_EQUAL_INT(0, ptp_clock_quality_from_view(&cfg, &q, &out));
	TEST_ASSERT_EQUAL_HEX16(0x1234U, out.offset_scaled_log_variance);
}

static void test_flags_from_view(void)
{
	ptp_quality_view_t q;
	uint16_t f;

	memset(&q, 0, sizeof(q));

	/* ptpTimescale is unconditional: this box never serves ARB. */
	f = ptp_flags_from_view(&q);
	TEST_ASSERT_EQUAL_HEX16(PTP_FLAG_PTP_TIMESCALE, f);
	TEST_ASSERT_EQUAL_HEX16(PTP_FLAG_PTP_TIMESCALE, ptp_flags_from_view(NULL));

	q.utc_offset_valid = true;
	q.time_traceable = true;
	q.freq_traceable = true;
	f = ptp_flags_from_view(&q);
	TEST_ASSERT_TRUE((f & PTP_FLAG_UTC_OFFSET_VALID) != 0U);
	TEST_ASSERT_TRUE((f & PTP_FLAG_TIME_TRACEABLE) != 0U);
	TEST_ASSERT_TRUE((f & PTP_FLAG_FREQ_TRACEABLE) != 0U);

	q.leap61 = true;
	f = ptp_flags_from_view(&q);
	TEST_ASSERT_TRUE((f & PTP_FLAG_LEAP61) != 0U);
	TEST_ASSERT_TRUE((f & PTP_FLAG_LEAP59) == 0U);

	q.leap61 = false;
	q.leap59 = true;
	f = ptp_flags_from_view(&q);
	TEST_ASSERT_TRUE((f & PTP_FLAG_LEAP59) != 0U);
	TEST_ASSERT_TRUE((f & PTP_FLAG_LEAP61) == 0U);

	/* §7.2.4 makes the pair invalid; insertion wins and the wire stays legal. */
	q.leap61 = true;
	f = ptp_flags_from_view(&q);
	TEST_ASSERT_TRUE((f & PTP_FLAG_LEAP61) != 0U);
	TEST_ASSERT_TRUE((f & PTP_FLAG_LEAP59) == 0U);
}

/* ------------------------------------------------- §3.8 block projection -- */

static void test_view_from_block_sync_state(void)
{
	quality_block_t b;
	ptp_quality_view_t v;

	quality_block_init(&b);
	b.lock_state = (uint8_t)QUALITY_LOCK_LOCKED;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_INT(PTP_SYNC_LOCKED, v.sync_state);
	TEST_ASSERT_TRUE(v.time_traceable);

	b.holdover = true;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_INT(PTP_SYNC_HOLDOVER, v.sync_state);
	TEST_ASSERT_TRUE(v.time_traceable);

	b.flags |= QUALITY_FLAG_DEMOTED;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_INT(PTP_SYNC_HOLDOVER_EXCEEDED, v.sync_state);
	TEST_ASSERT_FALSE(v.time_traceable);

	/* The holdover flag alone is enough, whatever the lock state says. */
	quality_block_init(&b);
	b.lock_state = (uint8_t)QUALITY_LOCK_ACQUIRING;
	b.flags = QUALITY_FLAG_HOLDOVER;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_INT(PTP_SYNC_HOLDOVER, v.sync_state);

	/* RECOVERING is a rate-limited return from holdover, not a fresh lock. */
	quality_block_init(&b);
	b.lock_state = (uint8_t)QUALITY_LOCK_RECOVERING;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_INT(PTP_SYNC_HOLDOVER, v.sync_state);

	quality_block_init(&b);
	b.lock_state = (uint8_t)QUALITY_LOCK_ACQUIRING;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_INT(PTP_SYNC_FREERUN, v.sync_state);
	TEST_ASSERT_FALSE(v.time_traceable);
	TEST_ASSERT_FALSE(v.freq_traceable);

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_quality_view_from_block(NULL, 0U, &v));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_quality_view_from_block(&b, 0U, NULL));
}

static void test_view_from_block_leap_window(void)
{
	quality_block_t b;
	ptp_quality_view_t v;
	const uint64_t event = 1800000000ULL;

	quality_block_init(&b);
	b.lock_state = (uint8_t)QUALITY_LOCK_LOCKED;
	b.utc_valid = true;
	b.leap_current_s = 37;
	b.leap_pending = 1;
	b.leap_at_tai_s = event;

	/*
	 * Before the receiver reports, the block carries leap_current_s 0, which
	 * as a currentUtcOffset would claim TAI == UTC. The standing 37 s goes
	 * out instead, with currentUtcOffsetValid clear so nobody trusts it.
	 */
	{
		quality_block_t fresh;
		ptp_quality_view_t fv;

		quality_block_init(&fresh);
		TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&fresh, 0U, &fv));
		TEST_ASSERT_EQUAL_INT16(PTP_DEFAULT_UTC_OFFSET, fv.utc_offset);
		TEST_ASSERT_FALSE(fv.utc_offset_valid);

		/* A real offset always wins, valid or not. */
		fresh.leap_current_s = 38;
		TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&fresh, 0U, &fv));
		TEST_ASSERT_EQUAL_INT16(38, fv.utc_offset);

		/* And a genuine zero from a validated source is respected. */
		fresh.leap_current_s = 0;
		fresh.utc_valid = true;
		TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&fresh, 0U, &fv));
		TEST_ASSERT_EQUAL_INT16(0, fv.utc_offset);
		TEST_ASSERT_TRUE(fv.utc_offset_valid);
	}

	/* Months out: announced by the receiver, not yet by us. */
	TEST_ASSERT_EQUAL_INT(0,
		ptp_quality_view_from_block(&b, event - 86400ULL, &v));
	TEST_ASSERT_FALSE(v.leap61);
	TEST_ASSERT_FALSE(v.leap59);
	TEST_ASSERT_TRUE(v.utc_offset_valid);
	TEST_ASSERT_EQUAL_INT16(37, v.utc_offset);

	/* Just inside the 12-hour window. */
	TEST_ASSERT_EQUAL_INT(0,
		ptp_quality_view_from_block(&b, event - PTP_LEAP_ANNOUNCE_WINDOW_S, &v));
	TEST_ASSERT_TRUE(v.leap61);

	/* One second outside it. */
	TEST_ASSERT_EQUAL_INT(0,
		ptp_quality_view_from_block(&b,
			event - PTP_LEAP_ANNOUNCE_WINDOW_S - 1ULL, &v));
	TEST_ASSERT_FALSE(v.leap61);

	/* At and after the event the flag drops even if the block still says pending. */
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, event, &v));
	TEST_ASSERT_FALSE(v.leap61);
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, event + 10ULL, &v));
	TEST_ASSERT_FALSE(v.leap61);

	/* A deletion sets leap59 instead. */
	b.leap_pending = -1;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, event - 60ULL, &v));
	TEST_ASSERT_TRUE(v.leap59);
	TEST_ASSERT_FALSE(v.leap61);

	b.leap_pending = 0;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, event - 60ULL, &v));
	TEST_ASSERT_FALSE(v.leap59);
	TEST_ASSERT_FALSE(v.leap61);
}

static void test_view_from_block_source_and_accuracy(void)
{
	quality_block_t b;
	ptp_quality_view_t v;

	quality_block_init(&b);
	b.lock_state = (uint8_t)QUALITY_LOCK_LOCKED;
	b.flags = QUALITY_FLAG_GNSS_TIME_LOCKED;
	b.gnss_tacc_ns = 15U;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_HEX8((uint8_t)PTP_TIME_SRC_GNSS, v.time_source);
	TEST_ASSERT_EQUAL_UINT64(15U, v.est_accuracy_ns);

	/* No GNSS lock: the mux position names the source. */
	b.flags = 0U;
	b.active_ref = (uint8_t)QUALITY_REF_RB;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_HEX8((uint8_t)PTP_TIME_SRC_ATOMIC_CLOCK, v.time_source);

	b.active_ref = (uint8_t)QUALITY_REF_EXTREF;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_HEX8((uint8_t)PTP_TIME_SRC_OTHER, v.time_source);

	b.active_ref = (uint8_t)QUALITY_REF_OCXO;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_HEX8((uint8_t)PTP_TIME_SRC_INTERNAL_OSC, v.time_source);

	/* An atomic reference is frequency-traceable even while free-running. */
	quality_block_init(&b);
	b.lock_state = (uint8_t)QUALITY_LOCK_ACQUIRING;
	b.active_ref = (uint8_t)QUALITY_REF_RB;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_FALSE(v.time_traceable);
	TEST_ASSERT_TRUE(v.freq_traceable);

	/* In holdover the accuracy comes from the §3.6 estimator, sign removed. */
	quality_block_init(&b);
	b.lock_state = (uint8_t)QUALITY_LOCK_HOLDOVER;
	b.holdover_est_err_ns = -4200;
	b.gnss_tacc_ns = 15U;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_UINT64(4200U, v.est_accuracy_ns);

	/*
	 * The magnitude is taken without ever evaluating -INT64_MIN, which is
	 * undefined. Under UBSan this case is the proof, not the comment.
	 */
	b.holdover_est_err_ns = INT64_MIN;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_UINT64(9223372036854775808ULL, v.est_accuracy_ns);

	b.holdover_est_err_ns = INT64_MAX;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_UINT64((uint64_t)INT64_MAX, v.est_accuracy_ns);

	b.holdover_est_err_ns = -1;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_UINT64(1U, v.est_accuracy_ns);

	/* No receiver estimate: fall back to the rolling PPS sigma. */
	quality_block_init(&b);
	b.lock_state = (uint8_t)QUALITY_LOCK_LOCKED;
	b.gnss_tacc_ns = 0U;
	b.pps_off_sigma_ns = 9.7f;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_UINT64(9U, v.est_accuracy_ns);

	/* Nothing at all is "unknown", not a fabricated zero-error claim. */
	b.pps_off_sigma_ns = 0.0f;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_UINT64(0U, v.est_accuracy_ns);
}

static void test_view_from_block_adev_is_clamped(void)
{
	quality_block_t b;
	ptp_quality_view_t v;

	quality_block_init(&b);
	b.lock_state = (uint8_t)QUALITY_LOCK_LOCKED;

	b.adev_1s = 1e-12f;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	/* float rounding of 1e-12 * 1e18; within a part in 1e5 of 1e6. */
	TEST_ASSERT_UINT64_WITHIN(20ULL, 1000000ULL, v.adev_tau1_e18);

	/* Not yet characterised, negative, or nonsense all mean "unknown". */
	b.adev_1s = 0.0f;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_UINT64(0U, v.adev_tau1_e18);

	b.adev_1s = -1.0f;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_UINT64(0U, v.adev_tau1_e18);

	/* An implausibly large ADEV clamps instead of overflowing the uint64. */
	b.adev_1s = 1e30f;
	TEST_ASSERT_EQUAL_INT(0, ptp_quality_view_from_block(&b, 0U, &v));
	TEST_ASSERT_EQUAL_UINT64(10000000000000000000ULL, v.adev_tau1_e18);
}

/* ------------------------------------------------------------ lifecycle --- */

static void test_init_and_identity(void)
{
	ptp_port_ctx_t c;
	ptp_cfg_t cfg;
	ptp_port_ops_t ops;
	ptp_dataset_t d0;
	fake_t f;

	fake_init(&f);
	ops_from(&ops, &f);
	ptp_cfg_defaults(&cfg);

	TEST_ASSERT_EQUAL_INT(0, ptp_port_init(&c, &cfg, self_mac, &ops));
	TEST_ASSERT_EQUAL_INT(PTP_PS_INITIALIZING, ptp_port_state(&c));
	TEST_ASSERT_EQUAL_HEX32(0U, ptp_port_alarms(&c));
	TEST_ASSERT_NOT_NULL(ptp_port_counters(&c));

	TEST_ASSERT_EQUAL_INT(0, ptp_port_dataset(&c, &d0));
	TEST_ASSERT_EQUAL_HEX8_ARRAY(self_eui64, d0.gm_identity.id, 8);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(self_eui64, d0.sender.clock_id.id, 8);
	TEST_ASSERT_EQUAL_HEX16(1U, d0.sender.port_number);
	TEST_ASSERT_EQUAL_HEX16(0U, d0.steps_removed);
	TEST_ASSERT_EQUAL_HEX8(128U, d0.priority1);
	/*
	 * Nothing published yet, so this clock has never been disciplined and
	 * advertises IEEE 1588-2019 Table 4's "not traceable" default class. It
	 * must NOT be a degradation class: 52 would outrank an honest peer at
	 * 187 and take the segment on the strength of an uncalibrated OCXO.
	 */
	TEST_ASSERT_EQUAL_HEX8(248U, d0.quality.clock_class);

	/* D0's sender and receiver are the same port, per §9.3.4. */
	TEST_ASSERT_EQUAL_INT(0, ptp_port_id_cmp(&d0.sender, &d0.receiver));
}

static void test_init_rejects_bad_arguments(void)
{
	ptp_port_ctx_t c;
	ptp_cfg_t cfg;
	ptp_port_ops_t ops;
	fake_t f;

	fake_init(&f);
	ops_from(&ops, &f);
	ptp_cfg_defaults(&cfg);

	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_port_init(NULL, &cfg, self_mac, &ops));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_port_init(&c, NULL, self_mac, &ops));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_port_init(&c, &cfg, NULL, &ops));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_port_init(&c, &cfg, self_mac, NULL));

	/* A transmit callback is mandatory: there is no useful engine without it. */
	ops.tx = NULL;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_port_init(&c, &cfg, self_mac, &ops));

	/* Configuration errors surface at init, not at the first Announce. */
	ops_from(&ops, &f);
	cfg.port_number = 0U;
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_port_init(&c, &cfg, self_mac, &ops));
}

/*
 * PTP_ALARM_PROFILE_UNSUPPORTED now means "the selected profile has a material
 * unimplemented feature", not "the selected profile is not Default". Default and
 * G.8275.1 are implemented and raise nothing; G.8275.2 (no unicast message
 * negotiation) and C37.238 (peer-delay mandated, this engine is E2E) do raise it.
 */
static void test_init_flags_an_unsupported_profile(void)
{
	ptp_port_ctx_t c;
	ptp_cfg_t cfg;
	ptp_port_ops_t ops;
	fake_t f;

	fake_init(&f);
	ops_from(&ops, &f);

	ptp_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_init(&c, &cfg, self_mac, &ops));
	TEST_ASSERT_TRUE((ptp_port_alarms(&c) & PTP_ALARM_PROFILE_UNSUPPORTED) == 0U);

	ptp_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_apply_profile(&cfg,
					(uint8_t)PTP_PROFILE_TELECOM_G8275_1));
	TEST_ASSERT_EQUAL_INT(0, ptp_port_init(&c, &cfg, self_mac, &ops));
	TEST_ASSERT_TRUE((ptp_port_alarms(&c) & PTP_ALARM_PROFILE_UNSUPPORTED) == 0U);

	ptp_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_apply_profile(&cfg,
					(uint8_t)PTP_PROFILE_TELECOM_G8275_2));
	TEST_ASSERT_EQUAL_INT(0, ptp_port_init(&c, &cfg, self_mac, &ops));
	TEST_ASSERT_TRUE((ptp_port_alarms(&c) & PTP_ALARM_PROFILE_UNSUPPORTED) != 0U);

	ptp_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ptp_cfg_apply_profile(&cfg,
					(uint8_t)PTP_PROFILE_POWER_C37_238));
	TEST_ASSERT_EQUAL_INT(0, ptp_port_init(&c, &cfg, self_mac, &ops));
	TEST_ASSERT_TRUE((ptp_port_alarms(&c) & PTP_ALARM_PROFILE_UNSUPPORTED) != 0U);
}

static void test_accessors_tolerate_null(void)
{
	ptp_dataset_t d0;

	TEST_ASSERT_EQUAL_INT(PTP_PS_INITIALIZING, ptp_port_state(NULL));
	TEST_ASSERT_EQUAL_HEX32(0U, ptp_port_alarms(NULL));
	TEST_ASSERT_NULL(ptp_port_counters(NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_port_dataset(NULL, &d0));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_port_step(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_port_enable(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_port_set_quality(NULL, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_on_sync_txts(NULL, 0U, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_port_fault(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_port_fault_reset(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(-EINVAL, ptp_port_rx(NULL, NULL, 0U, 0U, 0U));
}

static void test_port_state_names(void)
{
	TEST_ASSERT_EQUAL_STRING("INITIALIZING", ptp_port_state_name(PTP_PS_INITIALIZING));
	TEST_ASSERT_EQUAL_STRING("FAULTY", ptp_port_state_name(PTP_PS_FAULTY));
	TEST_ASSERT_EQUAL_STRING("DISABLED", ptp_port_state_name(PTP_PS_DISABLED));
	TEST_ASSERT_EQUAL_STRING("LISTENING", ptp_port_state_name(PTP_PS_LISTENING));
	TEST_ASSERT_EQUAL_STRING("PRE_MASTER", ptp_port_state_name(PTP_PS_PRE_MASTER));
	TEST_ASSERT_EQUAL_STRING("MASTER", ptp_port_state_name(PTP_PS_MASTER));
	TEST_ASSERT_EQUAL_STRING("PASSIVE", ptp_port_state_name(PTP_PS_PASSIVE));
	TEST_ASSERT_EQUAL_STRING("UNCALIBRATED", ptp_port_state_name(PTP_PS_UNCALIBRATED));
	TEST_ASSERT_EQUAL_STRING("SLAVE", ptp_port_state_name(PTP_PS_SLAVE));
	TEST_ASSERT_EQUAL_STRING("?", ptp_port_state_name((ptp_port_state_t)99));
}

/* ------------------------------------------------------- shared fixture --- */

typedef struct {
	ptp_port_ctx_t c;
	ptp_cfg_t cfg;
	ptp_port_ops_t ops;
	fake_t f;
} rig_t;

static void rig_init(rig_t *r, const ptp_cfg_t *cfg)
{
	fake_init(&r->f);
	r->f.tai_ns = 1750000000000000000ULL;
	ops_from(&r->ops, &r->f);
	r->cfg = *cfg;
	TEST_ASSERT_EQUAL_INT(0, ptp_port_init(&r->c, &r->cfg, self_mac, &r->ops));
}

static void rig_locked(rig_t *r)
{
	ptp_quality_view_t q;

	memset(&q, 0, sizeof(q));
	q.sync_state = PTP_SYNC_LOCKED;
	q.utc_offset = 37;
	q.utc_offset_valid = true;
	q.time_traceable = true;
	q.freq_traceable = true;
	q.time_source = (uint8_t)PTP_TIME_SRC_GNSS;
	q.est_accuracy_ns = 25U;
	TEST_ASSERT_EQUAL_INT(0, ptp_port_set_quality(&r->c, &q));
}

/* Step from t0 to t1 inclusive in 1 ms increments. */
static void rig_run(rig_t *r, uint64_t t0, uint64_t t1)
{
	uint64_t t;

	for (t = t0; t <= t1; t++) {
		TEST_ASSERT_EQUAL_INT(0, ptp_port_step(&r->c, t));
	}
}

/* --------------------------------------------------------- state machine -- */

static void test_listening_becomes_master_on_timeout(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	const ptp_counters_t *ctr;

	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	rig_locked(&r);

	/* Nothing happens before the port is enabled. */
	TEST_ASSERT_EQUAL_INT(0, ptp_port_step(&r.c, 100U));
	TEST_ASSERT_EQUAL_INT(PTP_PS_INITIALIZING, ptp_port_state(&r.c));
	TEST_ASSERT_EQUAL_size_t(0U, r.f.n);

	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
	TEST_ASSERT_EQUAL_INT(PTP_PS_LISTENING, ptp_port_state(&r.c));

	/* announceReceiptTimeout 3 x announceInterval 2000 ms = 6000 ms. */
	rig_run(&r, 0U, 5999U);
	TEST_ASSERT_EQUAL_INT(PTP_PS_LISTENING, ptp_port_state(&r.c));
	TEST_ASSERT_EQUAL_size_t(0U, r.f.n);

	rig_run(&r, 6000U, 6000U);
	TEST_ASSERT_EQUAL_INT(PTP_PS_MASTER, ptp_port_state(&r.c));

	ctr = ptp_port_counters(&r.c);
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->announce_timeouts);
	TEST_ASSERT_EQUAL_UINT32(2U, ctr->state_changes); /* LISTENING, MASTER */

	/* Announce and Sync go out immediately on entering MASTER. */
	TEST_ASSERT_EQUAL_size_t(1U, count_tx(&r.f, (uint8_t)PTP_MSG_ANNOUNCE));
	TEST_ASSERT_EQUAL_size_t(1U, count_tx(&r.f, (uint8_t)PTP_MSG_SYNC));

	/* The timeout is counted once, not on every subsequent step. */
	rig_run(&r, 6001U, 20000U);
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->announce_timeouts);
	TEST_ASSERT_EQUAL_INT(PTP_PS_MASTER, ptp_port_state(&r.c));

	/* Enabling an already-listening port is a programming error. */
	TEST_ASSERT_EQUAL_INT(-EPERM, ptp_port_enable(&r.c, 20000U));
}

static void test_better_master_forces_passive(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	foreign_spec_t s;
	const ptp_counters_t *ctr;

	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
	rig_run(&r, 0U, 6000U);
	TEST_ASSERT_EQUAL_INT(PTP_PS_MASTER, ptp_port_state(&r.c));

	/* One Announce is not enough: §9.3.2.4.5 needs two inside the window. */
	spec_defaults(&s);
	s.priority1 = 1U;
	{
		uint8_t buf[PTP_ANNOUNCE_LEN];
		size_t len = build_announce(buf, &s);

		TEST_ASSERT_EQUAL_INT(0, ptp_port_rx(&r.c, buf, len, 0U, 7000U));
	}
	TEST_ASSERT_EQUAL_INT(PTP_PS_MASTER, ptp_port_state(&r.c));

	/* The second one qualifies it, and priority1 1 beats our 128. */
	{
		uint8_t buf[PTP_ANNOUNCE_LEN];
		size_t len;

		s.seq = 2U;
		len = build_announce(buf, &s);
		TEST_ASSERT_EQUAL_INT(0, ptp_port_rx(&r.c, buf, len, 0U, 9000U));
	}
	TEST_ASSERT_EQUAL_INT(PTP_PS_PASSIVE, ptp_port_state(&r.c));
	TEST_ASSERT_TRUE((ptp_port_alarms(&r.c) & PTP_ALARM_NOT_BEST_MASTER) != 0U);

	ctr = ptp_port_counters(&r.c);
	TEST_ASSERT_EQUAL_UINT32(2U, ctr->rx[PTP_MSG_ANNOUNCE]);
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->foreign_added);

	/*
	 * A passive port transmits nothing. The window stops short of 15000 ms,
	 * where the peer's record would expire (last heard 9000 + 3 x 2000) and
	 * we would rightly take over again.
	 */
	{
		size_t before = r.f.n;

		rig_run(&r, 9001U, 14999U);
		TEST_ASSERT_EQUAL_size_t(before, r.f.n);
		TEST_ASSERT_EQUAL_INT(PTP_PS_PASSIVE, ptp_port_state(&r.c));
	}
}

static void test_worse_master_leaves_us_in_charge(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	foreign_spec_t s;

	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));

	/* A qualified but worse master short-circuits the wait: straight to MASTER. */
	spec_defaults(&s);
	s.priority1 = 200U;
	announce_twice(&r.c, &s, 500U, 2500U);
	TEST_ASSERT_EQUAL_INT(PTP_PS_MASTER, ptp_port_state(&r.c));
	TEST_ASSERT_TRUE((ptp_port_alarms(&r.c) & PTP_ALARM_NOT_BEST_MASTER) == 0U);

	/* And no announce receipt timeout was needed to get there. */
	TEST_ASSERT_EQUAL_UINT32(0U, ptp_port_counters(&r.c)->announce_timeouts);
}

static void test_foreign_timeout_re_elects_master(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	foreign_spec_t s;
	const ptp_counters_t *ctr;

	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));

	spec_defaults(&s);
	s.priority1 = 1U;
	announce_twice(&r.c, &s, 500U, 2500U);
	TEST_ASSERT_EQUAL_INT(PTP_PS_PASSIVE, ptp_port_state(&r.c));

	/* Still passive while the peer is only recently silent. */
	rig_run(&r, 2501U, 2500U + 5999U);
	TEST_ASSERT_EQUAL_INT(PTP_PS_PASSIVE, ptp_port_state(&r.c));

	/* 3 x 2000 ms of silence expires the record and we take over. */
	rig_run(&r, 2500U + 6000U, 2500U + 6000U);
	TEST_ASSERT_EQUAL_INT(PTP_PS_MASTER, ptp_port_state(&r.c));
	TEST_ASSERT_TRUE((ptp_port_alarms(&r.c) & PTP_ALARM_NOT_BEST_MASTER) == 0U);

	ctr = ptp_port_counters(&r.c);
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->foreign_expired);
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->announce_timeouts);

	/* The peer comes back and takes the role again. */
	announce_twice(&r.c, &s, 20000U, 22000U);
	TEST_ASSERT_EQUAL_INT(PTP_PS_PASSIVE, ptp_port_state(&r.c));
}

static void test_on_cadence_better_master_never_flaps(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	foreign_spec_t s;
	const ptp_counters_t *ctr;
	unsigned int k;

	/*
	 * Regression for the tumbling-window defect. A better grandmaster
	 * announcing exactly on cadence used to de-qualify every fourth
	 * interval; Erbest emptied, this port re-elected itself, and the segment
	 * got a second grandmaster transmitting Announce and Sync for an interval
	 * at a time, over and over. Sixty seconds of a perfectly healthy peer
	 * must produce exactly one transition and not one transmitted octet.
	 */
	ptp_cfg_defaults(&cfg);   /* announce 2 s, receipt timeout 6 s */
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));

	spec_defaults(&s);
	s.priority1 = 1U;         /* strictly better than our 128 */

	for (k = 0U; k < 30U; k++) {
		uint64_t at = 1000U + ((uint64_t)k * 2000U);
		uint8_t buf[PTP_ANNOUNCE_LEN];
		size_t len;
		uint64_t t;

		s.seq = (uint16_t)k;
		len = build_announce(buf, &s);
		TEST_ASSERT_EQUAL_INT(0, ptp_port_rx(&r.c, buf, len, 0U, at));

		for (t = at; t < at + 2000U; t++) {
			TEST_ASSERT_EQUAL_INT(0, ptp_port_step(&r.c, t));
			if (k >= 1U) {
				/* Qualified from the second Announce onward. */
				TEST_ASSERT_EQUAL_INT(PTP_PS_PASSIVE,
						      ptp_port_state(&r.c));
			}
		}
	}

	TEST_ASSERT_EQUAL_INT(PTP_PS_PASSIVE, ptp_port_state(&r.c));
	TEST_ASSERT_EQUAL_size_t(0U, r.f.n);

	ctr = ptp_port_counters(&r.c);
	/* INITIALIZING -> LISTENING -> PASSIVE, and nothing after. */
	TEST_ASSERT_EQUAL_UINT32(2U, ctr->state_changes);
	TEST_ASSERT_EQUAL_UINT32(0U, ctr->foreign_expired);
	TEST_ASSERT_EQUAL_UINT32(0U, ctr->announce_timeouts);
	TEST_ASSERT_EQUAL_UINT32(30U, ctr->rx[PTP_MSG_ANNOUNCE]);
	TEST_ASSERT_TRUE((ptp_port_alarms(&r.c) & PTP_ALARM_NOT_BEST_MASTER) != 0U);
}

static void test_fault_and_recovery(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	foreign_spec_t s;
	uint8_t buf[PTP_ANNOUNCE_LEN];
	size_t len;

	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));

	spec_defaults(&s);
	s.priority1 = 1U;
	announce_twice(&r.c, &s, 500U, 2500U);
	TEST_ASSERT_EQUAL_INT(PTP_PS_PASSIVE, ptp_port_state(&r.c));

	/* Fault while passive: the alarm set changes over cleanly. */
	TEST_ASSERT_EQUAL_INT(0, ptp_port_fault(&r.c, 3000U));
	TEST_ASSERT_EQUAL_INT(PTP_PS_FAULTY, ptp_port_state(&r.c));
	TEST_ASSERT_TRUE((ptp_port_alarms(&r.c) & PTP_ALARM_FAULTY) != 0U);
	TEST_ASSERT_TRUE((ptp_port_alarms(&r.c) & PTP_ALARM_NOT_BEST_MASTER) == 0U);

	/* A faulty port neither steps nor receives. */
	{
		size_t before = r.f.n;

		rig_run(&r, 3001U, 30000U);
		TEST_ASSERT_EQUAL_size_t(before, r.f.n);
		TEST_ASSERT_EQUAL_INT(PTP_PS_FAULTY, ptp_port_state(&r.c));
	}
	len = build_announce(buf, &s);
	TEST_ASSERT_EQUAL_INT(-EPERM, ptp_port_rx(&r.c, buf, len, 0U, 30000U));

	/* Repeated faults are idempotent. */
	TEST_ASSERT_EQUAL_INT(0, ptp_port_fault(&r.c, 31000U));

	/* Reset returns to LISTENING with the foreign table forgotten, so the
	 * full announce receipt timeout has to elapse again. */
	TEST_ASSERT_EQUAL_INT(0, ptp_port_fault_reset(&r.c, 40000U));
	TEST_ASSERT_EQUAL_INT(PTP_PS_LISTENING, ptp_port_state(&r.c));
	TEST_ASSERT_TRUE((ptp_port_alarms(&r.c) & PTP_ALARM_FAULTY) == 0U);

	rig_run(&r, 40000U, 45999U);
	TEST_ASSERT_EQUAL_INT(PTP_PS_LISTENING, ptp_port_state(&r.c));
	rig_run(&r, 46000U, 46000U);
	TEST_ASSERT_EQUAL_INT(PTP_PS_MASTER, ptp_port_state(&r.c));

	/* Resetting a port that is not faulty is a programming error. */
	TEST_ASSERT_EQUAL_INT(-EPERM, ptp_port_fault_reset(&r.c, 46001U));
}

/* --------------------------------------------------------- announce body -- */

static void test_announce_bytes_reflect_the_quality_view(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	size_t idx;
	const uint8_t *m;

	ptp_cfg_defaults(&cfg);
	cfg.priority1 = 90U;
	cfg.priority2 = 110U;
	cfg.domain = 24U;
	cfg.major_sdo_id = 0x2U;
	cfg.port_number = 5U;
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
	rig_run(&r, 0U, 6000U);

	idx = find_tx(&r.f, (uint8_t)PTP_MSG_ANNOUNCE, 0U);
	TEST_ASSERT_NOT_EQUAL_size_t(SIZE_MAX, idx);
	m = r.f.tx[idx].buf;

	TEST_ASSERT_EQUAL_size_t(PTP_ANNOUNCE_LEN, r.f.tx[idx].len);
	TEST_ASSERT_EQUAL_HEX8(0x2BU, m[O_TYPE]);       /* majorSdoId 2, Announce */
	TEST_ASSERT_EQUAL_HEX8(0x12U, m[O_VERSION]);    /* minorVersion 1, version 2 */
	TEST_ASSERT_EQUAL_HEX16(64U, be16(&m[O_LENGTH]));
	TEST_ASSERT_EQUAL_HEX8(24U, m[O_DOMAIN]);
	TEST_ASSERT_EQUAL_HEX8(5U, m[O_CONTROL]);       /* Table 39 "all others" */
	TEST_ASSERT_EQUAL_INT8(1, (int8_t)m[O_LOG_INT]);/* logAnnounceInterval */
	TEST_ASSERT_EQUAL_HEX8_ARRAY(self_eui64, &m[O_SRC_CLOCK], 8);
	TEST_ASSERT_EQUAL_HEX16(5U, be16(&m[O_SRC_PORT]));

	/* timePropertiesDS flags live in octet 1 of flagField; twoStep never here. */
	TEST_ASSERT_TRUE((m[O_FLAGS + 1U] & 0x04U) != 0U); /* currentUtcOffsetValid */
	TEST_ASSERT_TRUE((m[O_FLAGS + 1U] & 0x08U) != 0U); /* ptpTimescale */
	TEST_ASSERT_TRUE((m[O_FLAGS + 1U] & 0x10U) != 0U); /* timeTraceable */
	TEST_ASSERT_TRUE((m[O_FLAGS + 1U] & 0x20U) != 0U); /* frequencyTraceable */
	TEST_ASSERT_TRUE((m[O_FLAGS + 1U] & 0x03U) == 0U); /* no leap pending */
	TEST_ASSERT_EQUAL_HEX8(0x00U, m[O_FLAGS]);         /* twoStep clear */

	/* originTimestamp is reserved in -2019 and goes out as zero. */
	{
		size_t i;

		for (i = 0U; i < PTP_TS_LEN; i++) {
			TEST_ASSERT_EQUAL_HEX8(0x00U, m[O_BODY_TS + i]);
		}
	}

	TEST_ASSERT_EQUAL_HEX16(37U, be16(&m[O_ANN_UTC]));
	TEST_ASSERT_EQUAL_HEX8(90U, m[O_ANN_PRIO1]);
	TEST_ASSERT_EQUAL_HEX8(6U, m[O_ANN_CLASS]);     /* locked -> Table 4 class 6 */
	TEST_ASSERT_EQUAL_HEX8(0x20U, m[O_ANN_ACCUR]);  /* 25 ns -> Table 5 0x20 */
	TEST_ASSERT_EQUAL_HEX16(PTP_OSLV_DEFAULT_LOCKED, be16(&m[O_ANN_OSLV]));
	TEST_ASSERT_EQUAL_HEX8(110U, m[O_ANN_PRIO2]);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(self_eui64, &m[O_ANN_GMID], 8);
	TEST_ASSERT_EQUAL_HEX16(0U, be16(&m[O_ANN_STEPS]));
	TEST_ASSERT_EQUAL_HEX8(0x20U, m[O_ANN_TSRC]);   /* GNSS */

	/* Announce goes on the general port, to the primary multicast group. */
	TEST_ASSERT_EQUAL_INT(PTP_PORT_GENERAL, r.f.tx[idx].port_kind);
	TEST_ASSERT_EQUAL_INT(PTP_ADDR_PRIMARY, r.f.tx[idx].addr);
	TEST_ASSERT_EQUAL_INT(PTP_TRANSPORT_UDP_IPV4, r.f.tx[idx].transport);
}

static void test_announce_tracks_quality_transitions(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	ptp_quality_view_t q;
	size_t idx;

	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
	rig_run(&r, 0U, 6000U);

	idx = find_tx(&r.f, (uint8_t)PTP_MSG_ANNOUNCE, 0U);
	TEST_ASSERT_EQUAL_HEX8(6U, r.f.tx[idx].buf[O_ANN_CLASS]);

	/* locked -> holdover: class 7, and a coarser accuracy. */
	memset(&q, 0, sizeof(q));
	q.sync_state = PTP_SYNC_HOLDOVER;
	q.utc_offset = 37;
	q.utc_offset_valid = true;
	q.time_traceable = true;
	q.time_source = (uint8_t)PTP_TIME_SRC_ATOMIC_CLOCK;
	q.est_accuracy_ns = 900U;
	TEST_ASSERT_EQUAL_INT(0, ptp_port_set_quality(&r.c, &q));
	rig_run(&r, 6001U, 8100U);

	idx = find_tx(&r.f, (uint8_t)PTP_MSG_ANNOUNCE, 1U);
	TEST_ASSERT_NOT_EQUAL_size_t(SIZE_MAX, idx);
	TEST_ASSERT_EQUAL_HEX8(7U, r.f.tx[idx].buf[O_ANN_CLASS]);
	TEST_ASSERT_EQUAL_HEX8(0x23U, r.f.tx[idx].buf[O_ANN_ACCUR]); /* 900 ns -> 1 us */
	TEST_ASSERT_EQUAL_HEX8(0x10U, r.f.tx[idx].buf[O_ANN_TSRC]);  /* atomic clock */

	/* holdover -> exceeded: degradation alternative A, and unknown variance. */
	q.sync_state = PTP_SYNC_HOLDOVER_EXCEEDED;
	q.est_accuracy_ns = 0U;
	q.time_traceable = false;
	TEST_ASSERT_EQUAL_INT(0, ptp_port_set_quality(&r.c, &q));
	rig_run(&r, 8101U, 10200U);

	idx = find_tx(&r.f, (uint8_t)PTP_MSG_ANNOUNCE, 2U);
	TEST_ASSERT_NOT_EQUAL_size_t(SIZE_MAX, idx);
	TEST_ASSERT_EQUAL_HEX8(52U, r.f.tx[idx].buf[O_ANN_CLASS]);
	TEST_ASSERT_EQUAL_HEX8(PTP_CLOCK_ACCURACY_UNKNOWN,
			       r.f.tx[idx].buf[O_ANN_ACCUR]);
	TEST_ASSERT_EQUAL_HEX16(PTP_OSLV_UNKNOWN, be16(&r.f.tx[idx].buf[O_ANN_OSLV]));
	TEST_ASSERT_TRUE((r.f.tx[idx].buf[O_FLAGS + 1U] & 0x10U) == 0U);

	/* A pending leap second shows up in the very next Announce. */
	q.leap61 = true;
	TEST_ASSERT_EQUAL_INT(0, ptp_port_set_quality(&r.c, &q));
	rig_run(&r, 10201U, 12300U);
	idx = find_tx(&r.f, (uint8_t)PTP_MSG_ANNOUNCE, 3U);
	TEST_ASSERT_NOT_EQUAL_size_t(SIZE_MAX, idx);
	TEST_ASSERT_TRUE((r.f.tx[idx].buf[O_FLAGS + 1U] & 0x01U) != 0U);

	/* Sequence numbers advance by one per Announce. */
	{
		size_t i;
		size_t first = find_tx(&r.f, (uint8_t)PTP_MSG_ANNOUNCE, 0U);
		uint16_t base = be16(&r.f.tx[first].buf[O_SEQ]);

		for (i = 0U; i < 4U; i++) {
			size_t k = find_tx(&r.f, (uint8_t)PTP_MSG_ANNOUNCE, i);

			TEST_ASSERT_EQUAL_HEX16((uint16_t)(base + i),
						be16(&r.f.tx[k].buf[O_SEQ]));
			TEST_ASSERT_EQUAL_HEX16((uint16_t)(base + i), r.f.tx[k].seq);
		}
	}
}

/* --------------------------------------------------- sync and follow_up --- */

static void test_sync_followup_sequencing(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	size_t si;
	size_t fi;
	uint16_t seq;
	const ptp_counters_t *ctr;

	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
	rig_run(&r, 0U, 6000U);

	si = find_tx(&r.f, (uint8_t)PTP_MSG_SYNC, 0U);
	TEST_ASSERT_NOT_EQUAL_size_t(SIZE_MAX, si);
	seq = r.f.tx[si].seq;

	/* Two-step Sync: twoStepFlag set, event port, controlField 0. */
	TEST_ASSERT_EQUAL_size_t(PTP_TSMSG_LEN, r.f.tx[si].len);
	TEST_ASSERT_EQUAL_HEX8(0x00U, r.f.tx[si].buf[O_TYPE]);
	TEST_ASSERT_TRUE((r.f.tx[si].buf[O_FLAGS] & 0x02U) != 0U);
	TEST_ASSERT_EQUAL_HEX8(0U, r.f.tx[si].buf[O_CONTROL]);
	TEST_ASSERT_EQUAL_INT8(0, (int8_t)r.f.tx[si].buf[O_LOG_INT]);
	TEST_ASSERT_EQUAL_INT(PTP_PORT_EVENT, r.f.tx[si].port_kind);
	TEST_ASSERT_EQUAL_INT(PTP_ADDR_PRIMARY, r.f.tx[si].addr);

	/* The originTimestamp estimate comes from the port clock. */
	TEST_ASSERT_EQUAL_HEX8(0x00U, r.f.tx[si].buf[O_BODY_TS]);
	TEST_ASSERT_EQUAL_HEX16(0U, be16(&r.f.tx[si].buf[O_BODY_TS]));

	/* No Follow_Up until the hardware reports the egress timestamp. */
	TEST_ASSERT_EQUAL_size_t(0U, count_tx(&r.f, (uint8_t)PTP_MSG_FOLLOW_UP));

	TEST_ASSERT_EQUAL_INT(0,
		ptp_on_sync_txts(&r.c, seq, 1750000000123456789ULL));

	fi = find_tx(&r.f, (uint8_t)PTP_MSG_FOLLOW_UP, 0U);
	TEST_ASSERT_NOT_EQUAL_size_t(SIZE_MAX, fi);
	TEST_ASSERT_TRUE(fi > si); /* strictly after its Sync */

	/* The Follow_Up echoes the Sync's sequenceId... */
	TEST_ASSERT_EQUAL_HEX16(seq, be16(&r.f.tx[fi].buf[O_SEQ]));
	TEST_ASSERT_EQUAL_HEX16(seq, r.f.tx[fi].seq);
	/* ...carries controlField 2, goes on the general port... */
	TEST_ASSERT_EQUAL_HEX8(0x08U, r.f.tx[fi].buf[O_TYPE]);
	TEST_ASSERT_EQUAL_HEX8(2U, r.f.tx[fi].buf[O_CONTROL]);
	TEST_ASSERT_EQUAL_INT(PTP_PORT_GENERAL, r.f.tx[fi].port_kind);
	/* ...never sets twoStepFlag... */
	TEST_ASSERT_EQUAL_HEX8(0x00U, r.f.tx[fi].buf[O_FLAGS]);
	/* ...and publishes the precise egress timestamp: 1750000000.123456789 s. */
	{
		const uint8_t *ts = &r.f.tx[fi].buf[O_BODY_TS];
		uint64_t sec = ((uint64_t)be16(ts) << 32) |
			       ((uint64_t)ts[2] << 24) | ((uint64_t)ts[3] << 16) |
			       ((uint64_t)ts[4] << 8) | (uint64_t)ts[5];
		uint32_t ns = ((uint32_t)ts[6] << 24) | ((uint32_t)ts[7] << 16) |
			      ((uint32_t)ts[8] << 8) | (uint32_t)ts[9];

		TEST_ASSERT_EQUAL_UINT64(1750000000ULL, sec);
		TEST_ASSERT_EQUAL_UINT32(123456789U, ns);
	}

	/* A second report for the same sequence has nothing left to release. */
	TEST_ASSERT_EQUAL_INT(-ENOENT, ptp_on_sync_txts(&r.c, seq, 1ULL));
	ctr = ptp_port_counters(&r.c);
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->txts_unmatched);

	/* So does one for a sequence that was never sent. */
	TEST_ASSERT_EQUAL_INT(-ENOENT, ptp_on_sync_txts(&r.c, (uint16_t)(seq + 99U), 1ULL));
	TEST_ASSERT_EQUAL_UINT32(2U, ctr->txts_unmatched);
	TEST_ASSERT_EQUAL_size_t(1U, count_tx(&r.f, (uint8_t)PTP_MSG_FOLLOW_UP));
}

static void test_missing_txts_is_counted_not_hidden(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	const ptp_counters_t *ctr;

	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));

	/* Three Syncs with no egress timestamp reported for any of them. */
	rig_run(&r, 0U, 8000U);
	ctr = ptp_port_counters(&r.c);
	TEST_ASSERT_EQUAL_size_t(3U, count_tx(&r.f, (uint8_t)PTP_MSG_SYNC));
	TEST_ASSERT_EQUAL_size_t(0U, count_tx(&r.f, (uint8_t)PTP_MSG_FOLLOW_UP));
	/* The first Sync had no predecessor, so two were dropped. */
	TEST_ASSERT_EQUAL_UINT32(2U, ctr->followup_missed);

	/* A stale report for the second Sync is refused: only the last is armed. */
	TEST_ASSERT_EQUAL_INT(-ENOENT, ptp_on_sync_txts(&r.c, r.f.tx[0].seq, 1ULL));
}

static void test_nested_txts_does_not_alias_the_sync_buffer(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	size_t si;
	size_t fi;
	const ptp_counters_t *ctr;

	/*
	 * A MAC that can read its own egress timestamp reports it from inside
	 * the Sync transmit callback. The Follow_Up that releases must not be
	 * encoded over the Sync the glue is still holding — the fake checks the
	 * descriptor's bytes before and after the nested call.
	 */
	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	rig_locked(&r);
	r.f.nested_ctx = &r.c;
	r.f.nested_txts = true;
	r.f.nested_ts = 1750000000123456789ULL;

	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
	rig_run(&r, 0U, 6000U);

	TEST_ASSERT_FALSE(r.f.nested_saw_mutation);
	TEST_ASSERT_EQUAL_INT(0, r.f.nested_txts_rc);

	si = find_tx(&r.f, (uint8_t)PTP_MSG_SYNC, 0U);
	fi = find_tx(&r.f, (uint8_t)PTP_MSG_FOLLOW_UP, 0U);
	TEST_ASSERT_NOT_EQUAL_size_t(SIZE_MAX, si);
	TEST_ASSERT_NOT_EQUAL_size_t(SIZE_MAX, fi);

	/* Documented ordering: the nested Follow_Up goes out first. */
	TEST_ASSERT_TRUE(fi < si);

	/* Both messages are intact and are what they claim to be. */
	TEST_ASSERT_EQUAL_HEX8(0x00U, r.f.tx[si].buf[O_TYPE]);
	TEST_ASSERT_EQUAL_size_t(PTP_TSMSG_LEN, r.f.tx[si].len);
	TEST_ASSERT_TRUE((r.f.tx[si].buf[O_FLAGS] & 0x02U) != 0U); /* twoStep */
	TEST_ASSERT_EQUAL_HEX8(0x08U, r.f.tx[fi].buf[O_TYPE]);
	TEST_ASSERT_EQUAL_HEX8(2U, r.f.tx[fi].buf[O_CONTROL]);
	TEST_ASSERT_EQUAL_HEX16(r.f.tx[si].seq, be16(&r.f.tx[fi].buf[O_SEQ]));

	/* The Follow_Up carries the precise timestamp the callback reported. */
	{
		const uint8_t *ts = &r.f.tx[fi].buf[O_BODY_TS];
		uint64_t sec = ((uint64_t)be16(ts) << 32) |
			       ((uint64_t)ts[2] << 24) | ((uint64_t)ts[3] << 16) |
			       ((uint64_t)ts[4] << 8) | (uint64_t)ts[5];
		uint32_t ns = ((uint32_t)ts[6] << 24) | ((uint32_t)ts[7] << 16) |
			      ((uint32_t)ts[8] << 8) | (uint32_t)ts[9];

		TEST_ASSERT_EQUAL_UINT64(1750000000ULL, sec);
		TEST_ASSERT_EQUAL_UINT32(123456789U, ns);
	}

	ctr = ptp_port_counters(&r.c);
	TEST_ASSERT_EQUAL_UINT32(0U, ctr->followup_missed);
	TEST_ASSERT_EQUAL_UINT32(0U, ctr->followup_orphaned);
	TEST_ASSERT_EQUAL_UINT32(0U, ctr->txts_unmatched);
	TEST_ASSERT_EQUAL_UINT32(ctr->tx[PTP_MSG_SYNC], ctr->tx[PTP_MSG_FOLLOW_UP]);
	TEST_ASSERT_FALSE(r.c.sync_pending);
}

static void test_nested_txts_then_failing_sync_is_counted(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	const ptp_counters_t *ctr;

	/*
	 * The awkward corner of the same path: the glue releases the Follow_Up
	 * synchronously and only then reports the Sync as failed. The Follow_Up
	 * is already on the wire with nothing to follow, so it is counted rather
	 * than silently forgotten.
	 */
	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	rig_locked(&r);
	r.f.nested_ctx = &r.c;
	r.f.nested_txts = true;
	r.f.nested_ts = 1750000000000000001ULL;
	r.f.sync_rc = -EIO;

	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
	rig_run(&r, 0U, 6000U);

	TEST_ASSERT_FALSE(r.f.nested_saw_mutation);
	TEST_ASSERT_EQUAL_INT(0, r.f.nested_txts_rc); /* the Follow_Up itself went */

	ctr = ptp_port_counters(&r.c);
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->followup_orphaned);
	TEST_ASSERT_EQUAL_UINT32(0U, ctr->followup_missed);
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->tx_errors);
	TEST_ASSERT_EQUAL_UINT32(0U, ctr->tx[PTP_MSG_SYNC]);
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->tx[PTP_MSG_FOLLOW_UP]);
	TEST_ASSERT_FALSE(r.c.sync_pending);

	/* The Announce still went out; only the Sync failed. */
	TEST_ASSERT_NOT_EQUAL_size_t(SIZE_MAX,
		find_tx(&r.f, (uint8_t)PTP_MSG_ANNOUNCE, 0U));
	TEST_ASSERT_EQUAL_size_t(SIZE_MAX, find_tx(&r.f, (uint8_t)PTP_MSG_SYNC, 0U));
}

static void test_sync_timestamp_falls_back_to_zero(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	size_t si;
	size_t i;

	/* §11.4.3 allows a zero originTimestamp on a two-step Sync. */
	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	r.f.tai_fails = true;
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
	rig_run(&r, 0U, 6000U);

	si = find_tx(&r.f, (uint8_t)PTP_MSG_SYNC, 0U);
	TEST_ASSERT_NOT_EQUAL_size_t(SIZE_MAX, si);
	for (i = 0U; i < PTP_TS_LEN; i++) {
		TEST_ASSERT_EQUAL_HEX8(0x00U, r.f.tx[si].buf[O_BODY_TS + i]);
	}
}

static void test_sync_without_a_clock_port(void)
{
	ptp_port_ctx_t c;
	ptp_cfg_t cfg;
	ptp_port_ops_t ops;
	fake_t f;
	size_t si;
	size_t i;
	uint64_t t;

	/* tai_ns is optional; the engine must not dereference a NULL callback. */
	fake_init(&f);
	f.have_tai = false;
	ops_from(&ops, &f);
	ptp_cfg_defaults(&cfg);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_init(&c, &cfg, self_mac, &ops));
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&c, 0U));
	for (t = 0U; t <= 6000U; t++) {
		TEST_ASSERT_EQUAL_INT(0, ptp_port_step(&c, t));
	}

	si = find_tx(&f, (uint8_t)PTP_MSG_SYNC, 0U);
	TEST_ASSERT_NOT_EQUAL_size_t(SIZE_MAX, si);
	for (i = 0U; i < PTP_TS_LEN; i++) {
		TEST_ASSERT_EQUAL_HEX8(0x00U, f.tx[si].buf[O_BODY_TS + i]);
	}
}

/* ------------------------------------------------------------ delay_resp -- */

static void test_delay_req_produces_a_delay_resp(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	uint8_t req[PTP_TSMSG_LEN];
	size_t len;
	size_t idx;
	const uint8_t *m;
	static const uint8_t requester_eui64[8] = {
		0x00, 0x80, 0xE1, 0xFF, 0xFE, 0x00, 0x00, 0x77
	};

	ptp_cfg_defaults(&cfg);
	cfg.log_min_delay_req_interval = 2;
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
	rig_run(&r, 0U, 6000U);
	TEST_ASSERT_EQUAL_INT(PTP_PS_MASTER, ptp_port_state(&r.c));

	len = build_delay_req(req, 0x77U, 42U, 0x1234U, 0x0000000000ABCDEFLL, false);
	TEST_ASSERT_EQUAL_INT(0,
		ptp_port_rx(&r.c, req, len, 1750000000987654321ULL, 6100U));

	idx = find_tx(&r.f, (uint8_t)PTP_MSG_DELAY_RESP, 0U);
	TEST_ASSERT_NOT_EQUAL_size_t(SIZE_MAX, idx);
	m = r.f.tx[idx].buf;

	TEST_ASSERT_EQUAL_size_t(PTP_DELAY_RESP_LEN, r.f.tx[idx].len);
	TEST_ASSERT_EQUAL_HEX8(0x09U, m[O_TYPE]);
	TEST_ASSERT_EQUAL_HEX16(54U, be16(&m[O_LENGTH]));
	TEST_ASSERT_EQUAL_HEX8(3U, m[O_CONTROL]);
	TEST_ASSERT_EQUAL_INT8(2, (int8_t)m[O_LOG_INT]); /* logMinDelayReqInterval */

	/* sequenceId is echoed from the request, not from our own pool. */
	TEST_ASSERT_EQUAL_HEX16(0x1234U, be16(&m[O_SEQ]));
	TEST_ASSERT_EQUAL_HEX16(0x1234U, r.f.tx[idx].seq);

	/* sourcePortIdentity is ours; requestingPortIdentity is the requester's. */
	TEST_ASSERT_EQUAL_HEX8_ARRAY(self_eui64, &m[O_SRC_CLOCK], 8);
	TEST_ASSERT_EQUAL_HEX8_ARRAY(requester_eui64, &m[O_DRESP_REQ], 8);
	TEST_ASSERT_EQUAL_HEX16(42U, be16(&m[O_DRESP_REQ + 8U]));

	/* §11.3: the request's correctionField is passed straight back. */
	{
		static const uint8_t want[8] = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0xAB, 0xCD, 0xEF
		};

		TEST_ASSERT_EQUAL_HEX8_ARRAY(want, &m[O_CORRECTION], 8);
	}

	/* receiveTimestamp is the hardware ingress time. */
	{
		const uint8_t *ts = &m[O_BODY_TS];
		uint64_t sec = ((uint64_t)be16(ts) << 32) |
			       ((uint64_t)ts[2] << 24) | ((uint64_t)ts[3] << 16) |
			       ((uint64_t)ts[4] << 8) | (uint64_t)ts[5];
		uint32_t ns = ((uint32_t)ts[6] << 24) | ((uint32_t)ts[7] << 16) |
			      ((uint32_t)ts[8] << 8) | (uint32_t)ts[9];

		TEST_ASSERT_EQUAL_UINT64(1750000000ULL, sec);
		TEST_ASSERT_EQUAL_UINT32(987654321U, ns);
	}

	/* Multicast request, multicast reply, general port. */
	TEST_ASSERT_EQUAL_INT(PTP_ADDR_PRIMARY, r.f.tx[idx].addr);
	TEST_ASSERT_EQUAL_INT(PTP_PORT_GENERAL, r.f.tx[idx].port_kind);

	TEST_ASSERT_EQUAL_UINT32(1U, ptp_port_counters(&r.c)->rx[PTP_MSG_DELAY_REQ]);
	TEST_ASSERT_EQUAL_UINT32(1U, ptp_port_counters(&r.c)->tx[PTP_MSG_DELAY_RESP]);
}

static void test_delay_req_negative_correction_and_unicast(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	uint8_t req[PTP_TSMSG_LEN];
	size_t len;
	size_t idx;

	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
	rig_run(&r, 0U, 6000U);

	len = build_delay_req(req, 0x77U, 42U, 9U, -1LL, true);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_rx(&r.c, req, len, 5ULL, 6100U));

	idx = find_tx(&r.f, (uint8_t)PTP_MSG_DELAY_RESP, 0U);
	TEST_ASSERT_NOT_EQUAL_size_t(SIZE_MAX, idx);

	/* A negative correction survives the round trip bit for bit. */
	{
		static const uint8_t want[8] = {
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
		};

		TEST_ASSERT_EQUAL_HEX8_ARRAY(want, &r.f.tx[idx].buf[O_CORRECTION], 8);
	}

	/* §13.9: a unicast request gets a unicast reply, and the flag is set. */
	TEST_ASSERT_EQUAL_INT(PTP_ADDR_UNICAST_PEER, r.f.tx[idx].addr);
	TEST_ASSERT_TRUE((r.f.tx[idx].buf[O_FLAGS] & 0x04U) != 0U);
	TEST_ASSERT_EQUAL_HEX16(42U, r.f.tx[idx].peer.port_number);
	TEST_ASSERT_EQUAL_HEX8(0x77U, r.f.tx[idx].peer.clock_id.id[7]);
}

static void test_event_message_without_a_timestamp_is_rejected(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	uint8_t req[PTP_TSMSG_LEN];
	size_t len;
	const ptp_counters_t *ctr;

	/*
	 * 0 is the value the contract tells the glue to pass for general
	 * messages, so it is exactly what a miswired event path delivers. A
	 * Delay_Resp built on it would carry receiveTimestamp 0, and the
	 * requester would compute a path delay some decades negative — far worse
	 * than no answer. Reject and count instead.
	 */
	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
	rig_run(&r, 0U, 6000U);
	TEST_ASSERT_EQUAL_INT(PTP_PS_MASTER, ptp_port_state(&r.c));

	len = build_delay_req(req, 0x77U, 42U, 5U, 0LL, false);
	TEST_ASSERT_EQUAL_INT(-ENODATA, ptp_port_rx(&r.c, req, len, 0U, 6100U));

	ctr = ptp_port_counters(&r.c);
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->rx_no_timestamp);
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->rx[PTP_MSG_DELAY_REQ]); /* it did arrive */
	TEST_ASSERT_EQUAL_size_t(0U, count_tx(&r.f, (uint8_t)PTP_MSG_DELAY_RESP));

	/* One nanosecond is enough to be a real timestamp. */
	TEST_ASSERT_EQUAL_INT(0, ptp_port_rx(&r.c, req, len, 1ULL, 6200U));
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->rx_no_timestamp);
	TEST_ASSERT_EQUAL_size_t(1U, count_tx(&r.f, (uint8_t)PTP_MSG_DELAY_RESP));

	/* The rule covers every event type, not just the one we answer. */
	{
		uint8_t buf[PTP_MSG_MAX_LEN];
		static const uint8_t event_types[] = {
			(uint8_t)PTP_MSG_SYNC,
			(uint8_t)PTP_MSG_PDELAY_REQ,
			(uint8_t)PTP_MSG_PDELAY_RESP,
		};
		size_t i;

		for (i = 0U; i < ARRAY_LEN(event_types); i++) {
			size_t n = ptp_msg_min_len(event_types[i]);

			memset(buf, 0, sizeof(buf));
			buf[O_TYPE] = event_types[i];
			buf[O_VERSION] = 0x12U;
			buf[O_LENGTH] = (uint8_t)(n >> 8);
			buf[O_LENGTH + 1U] = (uint8_t)(n & 0xFFU);
			buf[O_SRC_CLOCK + 7U] = 0xBBU;
			buf[O_SRC_PORT + 1U] = 0x01U;

			TEST_ASSERT_EQUAL_INT(-ENODATA,
				ptp_port_rx(&r.c, buf, n, 0U, 6300U));
		}
		TEST_ASSERT_EQUAL_UINT32(1U + (uint32_t)ARRAY_LEN(event_types),
					 ctr->rx_no_timestamp);
	}

	/* General messages are unaffected: 0 is the documented value there. */
	{
		uint8_t buf[PTP_MSG_MAX_LEN];
		size_t n = ptp_msg_min_len((uint8_t)PTP_MSG_FOLLOW_UP);

		memset(buf, 0, sizeof(buf));
		buf[O_TYPE] = (uint8_t)PTP_MSG_FOLLOW_UP;
		buf[O_VERSION] = 0x12U;
		buf[O_LENGTH] = (uint8_t)(n >> 8);
		buf[O_LENGTH + 1U] = (uint8_t)(n & 0xFFU);
		buf[O_SRC_CLOCK + 7U] = 0xBBU;
		buf[O_SRC_PORT + 1U] = 0x01U;

		TEST_ASSERT_EQUAL_INT(0, ptp_port_rx(&r.c, buf, n, 0U, 6400U));
	}
	TEST_ASSERT_EQUAL_UINT32(1U + (uint32_t)3U, ctr->rx_no_timestamp);
}

static void test_delay_req_ignored_unless_master(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	uint8_t req[PTP_TSMSG_LEN];
	size_t len;
	const ptp_counters_t *ctr;

	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));

	/* LISTENING: §9.5.10 says only a master answers. */
	len = build_delay_req(req, 0x77U, 1U, 1U, 0LL, false);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_rx(&r.c, req, len, 4242ULL, 100U));
	TEST_ASSERT_EQUAL_size_t(0U, count_tx(&r.f, (uint8_t)PTP_MSG_DELAY_RESP));

	ctr = ptp_port_counters(&r.c);
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->rx_ignored);

	/* PASSIVE likewise. */
	{
		foreign_spec_t s;

		spec_defaults(&s);
		s.priority1 = 1U;
		announce_twice(&r.c, &s, 200U, 2200U);
		TEST_ASSERT_EQUAL_INT(PTP_PS_PASSIVE, ptp_port_state(&r.c));
	}
	TEST_ASSERT_EQUAL_INT(0, ptp_port_rx(&r.c, req, len, 4242ULL, 2300U));
	TEST_ASSERT_EQUAL_size_t(0U, count_tx(&r.f, (uint8_t)PTP_MSG_DELAY_RESP));
	TEST_ASSERT_EQUAL_UINT32(2U, ctr->rx_ignored);
	TEST_ASSERT_EQUAL_UINT32(0U, ctr->rx_no_timestamp);
}

/* --------------------------------------------------------------- receive -- */

static void test_rx_filters(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	foreign_spec_t s;
	uint8_t buf[PTP_ANNOUNCE_LEN];
	size_t len;
	const ptp_counters_t *ctr;

	ptp_cfg_defaults(&cfg);
	cfg.domain = 7U;
	cfg.major_sdo_id = 1U;
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
	ctr = ptp_port_counters(&r.c);

	/* Truncated. */
	spec_defaults(&s);
	s.domain = 7U;
	s.major_sdo_id = 1U;
	len = build_announce(buf, &s);
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ptp_port_rx(&r.c, buf, 10U, 0U, 100U));
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->rx_dropped);

	/* Wrong versionPTP. */
	buf[O_VERSION] = 0x13U;
	TEST_ASSERT_EQUAL_INT(-EPROTO, ptp_port_rx(&r.c, buf, len, 0U, 100U));
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->rx_foreign_domain);
	buf[O_VERSION] = 0x12U;

	/* Wrong domain. */
	s.domain = 8U;
	len = build_announce(buf, &s);
	TEST_ASSERT_EQUAL_INT(-EPROTO, ptp_port_rx(&r.c, buf, len, 0U, 100U));
	TEST_ASSERT_EQUAL_UINT32(2U, ctr->rx_foreign_domain);

	/* Wrong majorSdoId. */
	s.domain = 7U;
	s.major_sdo_id = 2U;
	len = build_announce(buf, &s);
	TEST_ASSERT_EQUAL_INT(-EPROTO, ptp_port_rx(&r.c, buf, len, 0U, 100U));
	TEST_ASSERT_EQUAL_UINT32(3U, ctr->rx_foreign_domain);

	/* Our own Announce, looped back by the switch: recognised, not recorded. */
	s.major_sdo_id = 1U;
	s.use_self_id = true;
	len = build_announce(buf, &s);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_rx(&r.c, buf, len, 0U, 100U));
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->rx_self);
	TEST_ASSERT_EQUAL_UINT32(0U, ctr->foreign_added);
	TEST_ASSERT_EQUAL_UINT32(0U, ctr->rx[PTP_MSG_ANNOUNCE]);

	/* stepsRemoved >= 255 is not a candidate (§9.3.2.5 d). */
	s.use_self_id = false;
	s.steps_removed = 255U;
	len = build_announce(buf, &s);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_rx(&r.c, buf, len, 0U, 100U));
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->rx[PTP_MSG_ANNOUNCE]);
	TEST_ASSERT_EQUAL_UINT32(0U, ctr->foreign_added);

	s.steps_removed = 254U;
	len = build_announce(buf, &s);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_rx(&r.c, buf, len, 0U, 100U));
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->foreign_added);

	/* A port that has not been enabled does not receive at all. */
	{
		rig_t q;

		rig_init(&q, &cfg);
		TEST_ASSERT_EQUAL_INT(-EPERM, ptp_port_rx(&q.c, buf, len, 0U, 100U));
	}
}

static void test_rx_ignores_what_a_grandmaster_cannot_use(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	uint8_t buf[PTP_MSG_MAX_LEN];
	const ptp_counters_t *ctr;
	static const uint8_t types[] = {
		(uint8_t)PTP_MSG_SYNC,
		(uint8_t)PTP_MSG_PDELAY_REQ,
		(uint8_t)PTP_MSG_PDELAY_RESP,
		(uint8_t)PTP_MSG_FOLLOW_UP,
		(uint8_t)PTP_MSG_DELAY_RESP,
		(uint8_t)PTP_MSG_PDELAY_RESP_FOLLOW_UP,
		(uint8_t)PTP_MSG_SIGNALING,
		(uint8_t)PTP_MSG_MANAGEMENT,
	};
	size_t i;

	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
	rig_run(&r, 0U, 6000U);
	ctr = ptp_port_counters(&r.c);

	for (i = 0U; i < ARRAY_LEN(types); i++) {
		size_t len = ptp_msg_min_len(types[i]);
		size_t before = r.f.n;

		memset(buf, 0, sizeof(buf));
		buf[O_TYPE] = types[i];
		buf[O_VERSION] = 0x12U;
		buf[O_LENGTH] = (uint8_t)(len >> 8);
		buf[O_LENGTH + 1U] = (uint8_t)(len & 0xFFU);
		buf[O_SRC_CLOCK + 7U] = 0xBBU;
		buf[O_SRC_PORT + 1U] = 0x01U;
		buf[O_CONTROL] = ptp_msg_control_field(types[i]);

		/* Event types among these need a real ingress timestamp. */
		TEST_ASSERT_EQUAL_INT(0, ptp_port_rx(&r.c, buf, len, 4242ULL, 7000U));
		TEST_ASSERT_EQUAL_UINT32(1U, ctr->rx[types[i]]);
		/* Nothing is emitted in response, and nothing crashes. */
		TEST_ASSERT_EQUAL_size_t(before, r.f.n);
	}
	TEST_ASSERT_EQUAL_UINT32((uint32_t)ARRAY_LEN(types), ctr->rx_ignored);
	TEST_ASSERT_EQUAL_INT(PTP_PS_MASTER, ptp_port_state(&r.c));
}

static void test_rx_malformed_announce_body(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	uint8_t buf[PTP_ANNOUNCE_LEN];
	const ptp_counters_t *ctr;

	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
	ctr = ptp_port_counters(&r.c);

	/*
	 * messageLength says 64 but only 40 octets arrived: the header decoder
	 * rejects it before any body decoder can read past the end.
	 */
	{
		foreign_spec_t s;

		spec_defaults(&s);
		(void)build_announce(buf, &s);
	}
	TEST_ASSERT_EQUAL_INT(-EBADMSG, ptp_port_rx(&r.c, buf, 40U, 0U, 100U));
	TEST_ASSERT_EQUAL_UINT32(1U, ctr->rx_dropped);
	TEST_ASSERT_EQUAL_UINT32(0U, ctr->foreign_added);
}

static void test_rx_fuzz_does_not_disturb_the_engine(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	test_rng_t rng;
	unsigned int iter;

	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
	rig_run(&r, 0U, 6000U);
	TEST_ASSERT_EQUAL_INT(PTP_PS_MASTER, ptp_port_state(&r.c));

	test_rng_init(&rng, 0x5EED1588U);

	for (iter = 0U; iter < 20000U; iter++) {
		uint8_t buf[80];
		size_t len = test_rng_below(&rng, sizeof(buf) + 1U);
		int rc;

		test_rng_fill(&rng, buf, len);
		if (len >= PTP_HDR_LEN) {
			/* Force our domain and version so the parse gets deep. */
			buf[O_VERSION] = (uint8_t)((buf[O_VERSION] & 0xF0U) | 2U);
			buf[O_TYPE] = (uint8_t)(buf[O_TYPE] & 0x0FU);
			buf[O_DOMAIN] = 0U;
			buf[O_LENGTH] = 0U;
			buf[O_LENGTH + 1U] = (uint8_t)len;
		}

		/*
		 * A plausible ingress timestamp, so event messages reach the
		 * dispatch instead of stopping at the missing-timestamp check;
		 * -ENODATA is still allowed because the timestamp is only
		 * consulted for types the fuzzer picks at random.
		 */
		rc = ptp_port_rx(&r.c, buf, len, 4242ULL + iter, 7000U + iter);
		TEST_ASSERT_TRUE((rc == 0) || (rc == -EBADMSG) ||
				 (rc == -EPROTO) || (rc == -ENODATA));

		/* Random noise must never leave the engine in a nonsense state. */
		switch (ptp_port_state(&r.c)) {
		case PTP_PS_MASTER:
		case PTP_PS_PASSIVE:
		case PTP_PS_LISTENING:
			break;
		default:
			TEST_FAIL_MESSAGE("fuzz drove the port into a bad state");
			break;
		}
	}
	/* The counters must still add up. */
	TEST_ASSERT_EQUAL_INT(0, ptp_port_step(&r.c, 100000U));
}

/* -------------------------------------------------------------- transmit -- */

static void test_tx_failure_is_reported(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	const ptp_counters_t *ctr;

	ptp_cfg_defaults(&cfg);
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));

	r.f.tx_rc = -EIO;
	rig_run(&r, 0U, 6000U);

	ctr = ptp_port_counters(&r.c);
	TEST_ASSERT_EQUAL_size_t(0U, r.f.n);
	TEST_ASSERT_EQUAL_UINT32(2U, ctr->tx_errors); /* the Announce and the Sync */
	TEST_ASSERT_EQUAL_UINT32(0U, ctr->tx[PTP_MSG_ANNOUNCE]);
	TEST_ASSERT_TRUE((ptp_port_alarms(&r.c) & PTP_ALARM_TX_ERROR) != 0U);

	/* The alarm describes the most recent attempt, so success clears it. */
	r.f.tx_rc = 0;
	rig_run(&r, 6001U, 8100U);
	TEST_ASSERT_TRUE((ptp_port_alarms(&r.c) & PTP_ALARM_TX_ERROR) == 0U);
	TEST_ASSERT_GREATER_THAN_UINT32(0U, ctr->tx[PTP_MSG_ANNOUNCE]);

	/*
	 * A Sync that the interface refused arms no Follow_Up: there is no
	 * egress timestamp coming for a frame that never left.
	 */
	r.f.tx_rc = -EIO;
	rig_run(&r, 8101U, 9200U);
	TEST_ASSERT_FALSE(r.c.sync_pending);

	/* But a Sync that went out and a Follow_Up that cannot: error upward. */
	{
		size_t si;
		uint16_t seq;
		uint32_t errs_before;

		r.f.tx_rc = 0;
		rig_run(&r, 9201U, 10300U);
		si = find_tx(&r.f, (uint8_t)PTP_MSG_SYNC,
			     count_tx(&r.f, (uint8_t)PTP_MSG_SYNC) - 1U);
		TEST_ASSERT_NOT_EQUAL_size_t(SIZE_MAX, si);
		seq = r.f.tx[si].seq;
		TEST_ASSERT_TRUE(r.c.sync_pending);

		errs_before = ctr->tx_errors;
		r.f.tx_rc = -EIO;
		TEST_ASSERT_EQUAL_INT(-EIO, ptp_on_sync_txts(&r.c, seq, 1ULL));
		TEST_ASSERT_EQUAL_UINT32(errs_before + 1U, ctr->tx_errors);
		TEST_ASSERT_TRUE((ptp_port_alarms(&r.c) & PTP_ALARM_TX_ERROR) != 0U);
	}
}

static void test_transport_selection_is_echoed(void)
{
	static const ptp_transport_t transports[] = {
		PTP_TRANSPORT_UDP_IPV4, PTP_TRANSPORT_UDP_IPV6, PTP_TRANSPORT_L2,
	};
	size_t i;

	/*
	 * The engine does not frame anything; it tells the glue which
	 * encapsulation the operator configured and lets the socket layer do the
	 * rest (Annexes C, D and E).
	 */
	for (i = 0U; i < ARRAY_LEN(transports); i++) {
		rig_t r;
		ptp_cfg_t cfg;
		size_t k;

		ptp_cfg_defaults(&cfg);
		cfg.transport = transports[i];
		rig_init(&r, &cfg);
		rig_locked(&r);
		TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
		rig_run(&r, 0U, 6000U);

		TEST_ASSERT_GREATER_THAN_size_t(0U, r.f.n);
		for (k = 0U; k < r.f.n; k++) {
			TEST_ASSERT_EQUAL_INT(transports[i], r.f.tx[k].transport);
		}
	}
}

/* ------------------------------------------------------------- schedulers -- */

static void check_cadence(int8_t log_interval, unsigned int expected,
			  const char *label)
{
	rig_t r;
	ptp_cfg_t cfg;
	uint64_t t;
	uint64_t master_at = 0U;
	size_t announces;
	size_t syncs;

	ptp_cfg_defaults(&cfg);
	cfg.log_announce_interval = log_interval;
	cfg.log_sync_interval = log_interval;
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));

	/* Find the millisecond at which the port takes over. */
	for (t = 0U; t < 1000000U; t++) {
		TEST_ASSERT_EQUAL_INT(0, ptp_port_step(&r.c, t));
		if (ptp_port_state(&r.c) == PTP_PS_MASTER) {
			master_at = t;
			break;
		}
	}
	TEST_ASSERT_EQUAL_INT(PTP_PS_MASTER, ptp_port_state(&r.c));

	/* Count what goes out over the next 30 simulated seconds. */
	r.f.n = 0U;
	for (t = master_at + 1U; t <= master_at + 30000U; t++) {
		TEST_ASSERT_EQUAL_INT(0, ptp_port_step(&r.c, t));
	}
	announces = count_tx(&r.f, (uint8_t)PTP_MSG_ANNOUNCE);
	syncs = count_tx(&r.f, (uint8_t)PTP_MSG_SYNC);

	TEST_ASSERT_UINT_WITHIN_MESSAGE(1U, expected, (unsigned int)announces, label);
	TEST_ASSERT_UINT_WITHIN_MESSAGE(1U, expected, (unsigned int)syncs, label);
	TEST_ASSERT_EQUAL_size_t(0U, r.f.overflow);
}

static void test_scheduler_cadence(void)
{
	/* 2 Hz, 1 Hz and 0.5 Hz sustained over 30 simulated seconds. */
	check_cadence(-1, 60U, "logInterval -1 (2 Hz)");
	check_cadence(0, 30U, "logInterval 0 (1 Hz)");
	check_cadence(1, 15U, "logInterval 1 (0.5 Hz)");
}

static void test_scheduler_does_not_drift_or_burst(void)
{
	rig_t r;
	ptp_cfg_t cfg;
	uint64_t t;
	size_t n_before;
	size_t syncs_before;

	ptp_cfg_defaults(&cfg);
	cfg.log_sync_interval = 0;
	rig_init(&r, &cfg);
	rig_locked(&r);
	TEST_ASSERT_EQUAL_INT(0, ptp_port_enable(&r.c, 0U));
	rig_run(&r, 0U, 6000U);
	TEST_ASSERT_EQUAL_INT(PTP_PS_MASTER, ptp_port_state(&r.c));

	/*
	 * A thread starved for a minute must not then discharge sixty Syncs back
	 * to back: at most one Announce and one Sync leave per step.
	 */
	n_before = r.f.n;
	TEST_ASSERT_EQUAL_INT(0, ptp_port_step(&r.c, 66000U));
	TEST_ASSERT_LESS_OR_EQUAL_size_t(2U, r.f.n - n_before);

	/* The cadence then resumes from the new anchor: 5 Syncs in 5 s. */
	syncs_before = count_tx(&r.f, (uint8_t)PTP_MSG_SYNC);
	for (t = 66001U; t <= 71000U; t++) {
		TEST_ASSERT_EQUAL_INT(0, ptp_port_step(&r.c, t));
	}
	TEST_ASSERT_EQUAL_size_t(5U,
		count_tx(&r.f, (uint8_t)PTP_MSG_SYNC) - syncs_before);
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_cfg_defaults);
	RUN_TEST(test_cfg_validate_rejects);

	RUN_TEST(test_clock_class_mapping);
	RUN_TEST(test_clock_accuracy_ladder);
	RUN_TEST(test_offset_scaled_log_variance);
	RUN_TEST(test_flags_from_view);

	RUN_TEST(test_view_from_block_sync_state);
	RUN_TEST(test_view_from_block_leap_window);
	RUN_TEST(test_view_from_block_source_and_accuracy);
	RUN_TEST(test_view_from_block_adev_is_clamped);

	RUN_TEST(test_init_and_identity);
	RUN_TEST(test_init_rejects_bad_arguments);
	RUN_TEST(test_init_flags_an_unsupported_profile);
	RUN_TEST(test_accessors_tolerate_null);
	RUN_TEST(test_port_state_names);

	RUN_TEST(test_listening_becomes_master_on_timeout);
	RUN_TEST(test_better_master_forces_passive);
	RUN_TEST(test_worse_master_leaves_us_in_charge);
	RUN_TEST(test_foreign_timeout_re_elects_master);
	RUN_TEST(test_on_cadence_better_master_never_flaps);
	RUN_TEST(test_fault_and_recovery);

	RUN_TEST(test_announce_bytes_reflect_the_quality_view);
	RUN_TEST(test_announce_tracks_quality_transitions);

	RUN_TEST(test_sync_followup_sequencing);
	RUN_TEST(test_missing_txts_is_counted_not_hidden);
	RUN_TEST(test_nested_txts_does_not_alias_the_sync_buffer);
	RUN_TEST(test_nested_txts_then_failing_sync_is_counted);
	RUN_TEST(test_sync_timestamp_falls_back_to_zero);
	RUN_TEST(test_sync_without_a_clock_port);

	RUN_TEST(test_delay_req_produces_a_delay_resp);
	RUN_TEST(test_delay_req_negative_correction_and_unicast);
	RUN_TEST(test_event_message_without_a_timestamp_is_rejected);
	RUN_TEST(test_delay_req_ignored_unless_master);

	RUN_TEST(test_rx_filters);
	RUN_TEST(test_rx_ignores_what_a_grandmaster_cannot_use);
	RUN_TEST(test_rx_malformed_announce_body);
	RUN_TEST(test_rx_fuzz_does_not_disturb_the_engine);

	RUN_TEST(test_tx_failure_is_reported);
	RUN_TEST(test_transport_selection_is_echoed);

	RUN_TEST(test_scheduler_cadence);
	RUN_TEST(test_scheduler_does_not_drift_or_burst);

	return UNITY_END();
}
