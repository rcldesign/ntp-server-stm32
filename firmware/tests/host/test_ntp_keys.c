/*
 * STS1000 "Meridian" — NTP symmetric-key reconciliation, tested away from Zephyr.
 *
 * sts_ntp_keys.h decides which key ids core/ntp holds after a config change. The
 * direction that matters is REVOCATION, and it is invisible at runtime when it is
 * wrong: a key that should have been withdrawn keeps authenticating clients, and
 * the operator has already been told the slot was rejected.
 *
 * The regression these tests exist for: ntp_key_set() validates its arguments and
 * returns BEFORE it locates or clears a slot (core/ntp/ntp.c), so a rejected call
 * leaves any existing entry for that id completely intact. Rotating a live id to
 * parameters core refuses therefore used to leave the OLD key in service while
 * the glue recorded the slot as empty — which also made the id unwithdrawable for
 * the rest of the boot, because withdrawal walks the recorded ids. A rotation
 * prompted by compromise of the old key left it authenticating until reboot.
 *
 * ntp_key_set() only ever touches ctx->keys[] and NULL-checks its context, so
 * these tests drive a zeroed ntp_ctx_t directly and need no crypto port.
 */

#include <string.h>

#include "unity.h"

#include "ntp/ntp.h"
#include "zephyr/net/sts_ntp_keys.h"

/* ------------------------------------------------------------------ fixture */

static ntp_ctx_t g_ntp;
static sts_ntp_key_want_t g_want[STS_NTP_KEY_SLOTS];
static uint16_t g_installed[STS_NTP_KEY_SLOTS];
static sts_ntp_keys_res_t g_res;

void setUp(void)
{
	memset(&g_ntp, 0, sizeof(g_ntp));
	memset(g_want, 0, sizeof(g_want));
	memset(g_installed, 0, sizeof(g_installed));
	memset(&g_res, 0, sizeof(g_res));
}

void tearDown(void) {}

/** Fill a slot from raw configured values, as the glue's key_slot_read() does. */
static sts_ntp_key_reject_t want(size_t slot, uint16_t keyid, uint8_t alg,
				 uint8_t klen)
{
	uint8_t material[NTP_MAC_KEY_MAX];

	memset(material, 0xA5, sizeof(material));
	return sts_ntp_key_want_set(&g_want[slot], keyid, alg, material, klen);
}

static void reconcile(void)
{
	sts_ntp_keys_reconcile(&g_ntp, g_want, g_installed, &g_res);
}

/** The live entry for @p keyid, or NULL when core/ntp does not hold it. */
static const ntp_key_t *live(uint32_t keyid)
{
	size_t i;

	for (i = 0U; i < NTP_MAC_KEYS; i++) {
		if (g_ntp.keys[i].used && g_ntp.keys[i].keyid == keyid) {
			return &g_ntp.keys[i];
		}
	}
	return NULL;
}

/* ------------------------------------------------------- validation mirror */

/*
 * sts_ntp_key_want_set() must reject everything ntp_key_set() rejects. If it
 * ever accepts more, an unusable slot reads as "wanted" and the withdrawal pass
 * skips the id it should have cleared — which is precisely how the regression
 * below became reachable.
 */
static void test_want_set_mirrors_what_core_refuses(void)
{
	/* RFC 5905 §7.5 reserves key id 0 for the crypto-NAK. */
	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_UNUSED, want(0, 0U, NTP_MAC_HMAC_SHA256_128, 32U));
	TEST_ASSERT_FALSE(g_want[0].valid);

	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_NO_MATERIAL, want(0, 5U, NTP_MAC_HMAC_SHA256_128, 0U));
	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_BAD_ALG, want(0, 5U, 99U, 32U));

	/* RFC 8573: AES-CMAC-128 is keyed by exactly one AES-128 key. */
	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_BAD_LEN,
			      want(0, 5U, NTP_MAC_AES_CMAC_128, 32U));
	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_OK,
			      want(0, 5U, NTP_MAC_AES_CMAC_128, NTP_MAC_AES_KEY_LEN));
	TEST_ASSERT_TRUE(g_want[0].valid);
}

/*
 * A rejected slot still carries its configured id, so the caller can name it in
 * a diagnostic and reconciliation can withdraw it. Losing the id here is what
 * made the old key unreachable.
 */
static void test_a_rejected_slot_keeps_its_id(void)
{
	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_BAD_LEN,
			      want(0, 7U, NTP_MAC_AES_CMAC_128, 32U));
	TEST_ASSERT_FALSE(g_want[0].valid);
	TEST_ASSERT_EQUAL_UINT16(7U, g_want[0].keyid);
}

/* ------------------------------------------------------------- the regression */

/*
 * THE test. Rotate a live id to parameters core/ntp refuses and the old key must
 * be gone. Before the fix it stayed live and authenticating, and the operator had
 * been told "rejected".
 */
static void test_rotating_a_live_id_to_invalid_parameters_revokes_it(void)
{
	const ntp_key_t *k;

	/* Boot: id 5 live as HMAC-SHA256-128 with 32 octets. */
	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_OK,
			      want(0, 5U, NTP_MAC_HMAC_SHA256_128, 32U));
	reconcile();
	k = live(5U);
	TEST_ASSERT_NOT_NULL(k);
	TEST_ASSERT_EQUAL_UINT8(NTP_MAC_HMAC_SHA256_128, k->alg);
	TEST_ASSERT_EQUAL_UINT(1U, g_res.live);

	/* Operator rotates id 5 to AES-CMAC-128 but leaves the 32-octet blob. */
	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_BAD_LEN,
			      want(0, 5U, NTP_MAC_AES_CMAC_128, 32U));
	reconcile();

	/* The old key must NOT still be authenticating clients. */
	TEST_ASSERT_NULL(live(5U));
	TEST_ASSERT_EQUAL_UINT(0U, g_res.live);
}

/*
 * The second half of the same defect: after such a rejection the slot must not
 * become permanently unwithdrawable. Clearing the id in cfg has to take effect,
 * rather than requiring a reboot.
 */
static void test_a_rejected_slot_can_still_be_cleared_afterwards(void)
{
	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_OK,
			      want(0, 5U, NTP_MAC_HMAC_SHA256_128, 32U));
	reconcile();
	TEST_ASSERT_NOT_NULL(live(5U));

	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_BAD_LEN,
			      want(0, 5U, NTP_MAC_AES_CMAC_128, 32U));
	reconcile();
	TEST_ASSERT_NULL(live(5U));

	/* Operator now clears the slot outright: sec.ntpkey0.id = 0. */
	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_UNUSED,
			      want(0, 0U, NTP_MAC_HMAC_SHA256_128, 32U));
	reconcile();
	TEST_ASSERT_NULL(live(5U));
	TEST_ASSERT_EQUAL_UINT16(0U, g_installed[0]);
}

/* -------------------------------------------------------------- withdrawal */

static void test_changing_a_slots_id_withdraws_the_old_one(void)
{
	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_OK, want(0, 5U, NTP_MAC_HMAC_SHA256_128, 32U));
	reconcile();
	TEST_ASSERT_NOT_NULL(live(5U));

	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_OK, want(0, 9U, NTP_MAC_HMAC_SHA256_128, 32U));
	reconcile();
	TEST_ASSERT_NULL(live(5U));
	TEST_ASSERT_NOT_NULL(live(9U));
	TEST_ASSERT_EQUAL_UINT16(5U, g_res.withdrawn[0]);
}

static void test_emptying_the_key_material_withdraws_the_id(void)
{
	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_OK, want(0, 5U, NTP_MAC_HMAC_SHA256_128, 32U));
	reconcile();
	TEST_ASSERT_NOT_NULL(live(5U));

	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_NO_MATERIAL,
			      want(0, 5U, NTP_MAC_HMAC_SHA256_128, 0U));
	reconcile();
	TEST_ASSERT_NULL(live(5U));
}

/*
 * Two slots may legally name one id. Withdrawal is resolved against the WHOLE new
 * set, so clearing one slot must not delete a key another slot still installs —
 * a per-slot decision would let slot 1 revoke what slot 0 just re-installed.
 */
static void test_a_duplicate_id_is_not_withdrawn_by_the_other_slot(void)
{
	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_OK, want(0, 5U, NTP_MAC_HMAC_SHA256_128, 32U));
	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_OK, want(1, 5U, NTP_MAC_HMAC_SHA256_128, 32U));
	reconcile();
	TEST_ASSERT_NOT_NULL(live(5U));

	/* Slot 1 is cleared; slot 0 still wants id 5. */
	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_UNUSED,
			      want(1, 0U, NTP_MAC_HMAC_SHA256_128, 32U));
	reconcile();
	TEST_ASSERT_NOT_NULL(live(5U));
}

/* An unchanged configuration must not churn the table. */
static void test_reapplying_the_same_configuration_is_stable(void)
{
	const ntp_key_t *first;
	unsigned int i;

	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_OK, want(0, 5U, NTP_MAC_HMAC_SHA256_128, 32U));
	TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_OK, want(1, 6U, NTP_MAC_AES_CMAC_128,
						   NTP_MAC_AES_KEY_LEN));
	reconcile();
	first = live(5U);
	TEST_ASSERT_NOT_NULL(first);
	TEST_ASSERT_EQUAL_UINT(2U, g_res.live);

	for (i = 0U; i < 4U; i++) {
		reconcile();
		TEST_ASSERT_EQUAL_UINT(2U, g_res.live);
		TEST_ASSERT_NOT_NULL(live(5U));
		TEST_ASSERT_NOT_NULL(live(6U));
		TEST_ASSERT_EQUAL_UINT16(0U, g_res.withdrawn[0]);
		TEST_ASSERT_EQUAL_UINT16(0U, g_res.withdrawn[1]);
	}
}

/* All four slots, each a different id, all live simultaneously. */
static void test_all_slots_install_independently(void)
{
	size_t i;

	for (i = 0U; i < STS_NTP_KEY_SLOTS; i++) {
		TEST_ASSERT_EQUAL_INT(STS_NTP_KEY_OK,
				      want(i, (uint16_t)(11U + i),
					   NTP_MAC_HMAC_SHA256_128, 32U));
	}
	reconcile();
	TEST_ASSERT_EQUAL_UINT((unsigned int)STS_NTP_KEY_SLOTS, g_res.live);
	for (i = 0U; i < STS_NTP_KEY_SLOTS; i++) {
		TEST_ASSERT_NOT_NULL(live((uint32_t)(11U + i)));
	}
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_want_set_mirrors_what_core_refuses);
	RUN_TEST(test_a_rejected_slot_keeps_its_id);

	RUN_TEST(test_rotating_a_live_id_to_invalid_parameters_revokes_it);
	RUN_TEST(test_a_rejected_slot_can_still_be_cleared_afterwards);

	RUN_TEST(test_changing_a_slots_id_withdraws_the_old_one);
	RUN_TEST(test_emptying_the_key_material_withdraws_the_id);
	RUN_TEST(test_a_duplicate_id_is_not_withdrawn_by_the_other_slot);
	RUN_TEST(test_reapplying_the_same_configuration_is_stable);
	RUN_TEST(test_all_slots_install_independently);

	return UNITY_END();
}
