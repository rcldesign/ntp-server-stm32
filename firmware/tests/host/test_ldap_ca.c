/*
 * STS1000 "Meridian" — the LDAPS transport decision and its trust anchor,
 * tested away from Zephyr.
 *
 * sts_ldap_ca.h exists because `sec.ldap.mode = 2` used to be a configuration
 * that could never work: sts_aaa.c opened a TLS socket and set neither
 * TLS_SEC_TAG_LIST nor TLS_HOSTNAME, Zephyr left mbedTLS at its client default
 * of VERIFY_REQUIRED, and with no CA chain loaded the handshake could not
 * complete. Fail-closed — the bind password stayed off the wire — and
 * completely non-functional, with nothing in the log to say which of half a
 * dozen causes applied.
 *
 * Everything this file pins is invisible at runtime when it is wrong:
 *
 *   1. A TLS MODE NEVER FALLS BACK TO PLAINTEXT. There is exactly one way this
 *      feature can fail dangerously rather than merely fail, and it is a
 *      refusal path that ends up opening a cleartext socket and binding on it.
 *      The sweep below asserts it across every (mode, build, anchor) triple,
 *      not just the ones the code happens to branch on.
 *
 *   2. THE REFUSAL COMES BEFORE THE SOCKET. sts_ldap_go_opens_socket() is what
 *      do_ldap() consults, so a refusal that still returned true would connect
 *      out on behalf of a login that cannot succeed — and, for mode 1, would
 *      probe a directory on an unauthenticated caller's behalf.
 *
 *   3. THE REASON NAMES THE CAUSE. "no LDAP CA installed" is the whole point of
 *      refusing early; an operator who instead sees an mbedTLS error code
 *      cannot tell a missing anchor from a wrong port, an unreachable server or
 *      a clock skew.
 *
 *   4. THE ANCHOR IS SCREENED BEFORE IT IS STORED. The anchor is persisted
 *      verbatim to /lfs, so accepting a blob with a private key in it writes
 *      key material to a plain file that nothing on any normal path erases.
 *
 *   5. THE ANCHOR'S SEC TAG IS NOT THE WEB SERVER'S. They are opposite
 *      directions of TLS. Reusing "WEB1" here would arm the LDAPS client with
 *      this box's own server certificate and private key.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "unity.h"

#include "zephyr/net/sts_ldap_ca.h"
#include "zephyr/net/sts_web.h" /* STS_WEB_SEC_TAG, for the collision test */

/* ===================================================================== */
/* fixtures                                                              */
/* ===================================================================== */

/*
 * Framing only. sts_ldap_ca_check() is the structural gate — it never looks at
 * the base64 — and sts_aaa.c follows it with mbedtls_x509_crt_parse(), which is
 * the layer that would reject this body. Using real DER here would test
 * mbedTLS, not this header.
 */
#define CERT_BODY "MIIBkTCB+wIJAKZ0AAAAAAAAMAoGCCqGSM49BAMCMA0xCzAJBgNVBAMMAmNh\n"

#define ONE_CERT                          \
	"-----BEGIN CERTIFICATE-----\n"   \
	CERT_BODY                         \
	"-----END CERTIFICATE-----\n"

#define TWO_CERTS ONE_CERT ONE_CERT

static char big[STS_LDAP_CA_PEM_MAX + 64U];

/** A well-framed anchor of exactly @p total bytes. */
static size_t make_sized(size_t total)
{
	static const char b[] = STS_LDAP_CA_BEGIN "\n";
	static const char e[] = "\n" STS_LDAP_CA_END "\n";
	size_t bn = sizeof(b) - 1U;
	size_t en = sizeof(e) - 1U;
	size_t fill;

	TEST_ASSERT_TRUE(total >= (bn + en));
	TEST_ASSERT_TRUE(total <= sizeof(big));
	fill = total - bn - en;

	memcpy(big, b, bn);
	memset(&big[bn], 'A', fill);
	memcpy(&big[bn + fill], e, en);
	return total;
}

/* ===================================================================== */
/* 1. the transport decision                                             */
/* ===================================================================== */

/** The defect this whole change exists for: mode 2 with no anchor. */
static void test_ldaps_without_an_anchor_never_opens_a_socket(void)
{
	sts_ldap_go_t go = sts_ldap_transport((uint8_t)STS_LDAP_MODE_LDAPS,
					      true, false);

	TEST_ASSERT_EQUAL_INT(STS_LDAP_STOP_NO_CA, go);
	TEST_ASSERT_FALSE(sts_ldap_go_opens_socket(go));
	TEST_ASSERT_FALSE(sts_ldap_go_is_tls(go));
}

/**
 * And the refusal has to be legible. An operator reading "handshake failed"
 * cannot act; one reading "no LDAP CA installed" can.
 */
static void test_the_missing_anchor_is_named_in_the_reason(void)
{
	const char *why = sts_ldap_go_reason(STS_LDAP_STOP_NO_CA);

	TEST_ASSERT_NOT_NULL(strstr(why, "no LDAP CA"));
	/* And it says what to do about it. */
	TEST_ASSERT_NOT_NULL(strstr(why, "ldap-ca"));
}

static void test_ldaps_with_an_anchor_proceeds_over_tls(void)
{
	sts_ldap_go_t go = sts_ldap_transport((uint8_t)STS_LDAP_MODE_LDAPS,
					      true, true);

	TEST_ASSERT_EQUAL_INT(STS_LDAP_GO_LDAPS, go);
	TEST_ASSERT_TRUE(sts_ldap_go_opens_socket(go));
	TEST_ASSERT_TRUE(sts_ldap_go_is_tls(go));
}

/**
 * StartTLS is refused whatever else is true. Zephyr's TLS is a socket type
 * chosen at socket() time; an anchor does not help, and neither does anything
 * the operator can configure.
 */
static void test_starttls_is_always_refused(void)
{
	int build;
	int ca;

	for (build = 0; build <= 1; build++) {
		for (ca = 0; ca <= 1; ca++) {
			sts_ldap_go_t go = sts_ldap_transport(
				(uint8_t)STS_LDAP_MODE_STARTTLS, build != 0,
				ca != 0);

			TEST_ASSERT_FALSE(sts_ldap_go_opens_socket(go));
			/* A build with no TLS at all reports that first: it is
			 * the one thing the operator could change. */
			if (build != 0) {
				TEST_ASSERT_EQUAL_INT(STS_LDAP_STOP_STARTTLS,
						      go);
			} else {
				TEST_ASSERT_EQUAL_INT(STS_LDAP_STOP_NO_TLS_BUILD,
						      go);
			}
		}
	}
}

/** Plain LDAP is untouched by any of this — no TLS state can gate it. */
static void test_plain_mode_is_unaffected(void)
{
	int build;
	int ca;

	for (build = 0; build <= 1; build++) {
		for (ca = 0; ca <= 1; ca++) {
			sts_ldap_go_t go = sts_ldap_transport(
				(uint8_t)STS_LDAP_MODE_PLAIN, build != 0,
				ca != 0);

			TEST_ASSERT_EQUAL_INT(STS_LDAP_GO_PLAIN, go);
			TEST_ASSERT_TRUE(sts_ldap_go_opens_socket(go));
			TEST_ASSERT_FALSE(sts_ldap_go_is_tls(go));
		}
	}
}

static void test_a_tls_mode_in_a_non_tls_build_is_refused(void)
{
	TEST_ASSERT_EQUAL_INT(
		STS_LDAP_STOP_NO_TLS_BUILD,
		sts_ldap_transport((uint8_t)STS_LDAP_MODE_LDAPS, false, false));
	/* Even if an anchor somehow got loaded, there is no socket to arm. */
	TEST_ASSERT_EQUAL_INT(
		STS_LDAP_STOP_NO_TLS_BUILD,
		sts_ldap_transport((uint8_t)STS_LDAP_MODE_LDAPS, false, true));
}

/**
 * An unrecognised mode is refused, not silently treated as plain.
 *
 * The cfg schema bounds `sec.ldap.mode` to 0..2, so reaching this needs a
 * corrupt store — but the pre-existing chain (`if (mode==2) TLS; else if
 * (mode==1) refuse;`) sent mode 3 over a cleartext socket, and "unreachable"
 * is a different claim from "safe".
 */
static void test_an_out_of_range_mode_is_refused(void)
{
	unsigned int m;

	for (m = 3U; m <= 255U; m++) {
		sts_ldap_go_t go = sts_ldap_transport((uint8_t)m, true, true);

		TEST_ASSERT_EQUAL_INT(STS_LDAP_STOP_BAD_MODE, go);
		TEST_ASSERT_FALSE(sts_ldap_go_opens_socket(go));
	}
}

/**
 * THE invariant: over every (mode, build, anchor) triple, a non-plain mode
 * either runs over TLS or does not open a socket at all. Never a cleartext
 * bind, which is the one outcome that would put the directory password on the
 * wire — the precise failure the operator selected TLS to avoid.
 */
static void test_no_tls_mode_ever_reaches_a_cleartext_socket(void)
{
	unsigned int m;
	int build;
	int ca;

	for (m = 1U; m <= 255U; m++) {
		for (build = 0; build <= 1; build++) {
			for (ca = 0; ca <= 1; ca++) {
				sts_ldap_go_t go = sts_ldap_transport(
					(uint8_t)m, build != 0, ca != 0);

				if (sts_ldap_go_opens_socket(go)) {
					TEST_ASSERT_TRUE(
						sts_ldap_go_is_tls(go));
				}
				TEST_ASSERT_NOT_EQUAL_INT(STS_LDAP_GO_PLAIN, go);
			}
		}
	}
}

/** Exactly one triple may open a TLS socket, and it is the fully-armed one. */
static void test_only_a_fully_armed_ldaps_opens_a_tls_socket(void)
{
	unsigned int m;
	int build;
	int ca;
	unsigned int armed = 0U;

	for (m = 0U; m <= 255U; m++) {
		for (build = 0; build <= 1; build++) {
			for (ca = 0; ca <= 1; ca++) {
				if (sts_ldap_go_is_tls(sts_ldap_transport(
					    (uint8_t)m, build != 0, ca != 0))) {
					TEST_ASSERT_EQUAL_UINT(
						(unsigned int)STS_LDAP_MODE_LDAPS,
						m);
					TEST_ASSERT_TRUE(build != 0);
					TEST_ASSERT_TRUE(ca != 0);
					armed++;
				}
			}
		}
	}
	TEST_ASSERT_EQUAL_UINT(1U, armed);
}

/** Every outcome explains itself, and no two explanations are the same text. */
static void test_every_outcome_has_its_own_reason(void)
{
	static const sts_ldap_go_t all[] = {
		STS_LDAP_GO_PLAIN,	    STS_LDAP_GO_LDAPS,
		STS_LDAP_STOP_NO_TLS_BUILD, STS_LDAP_STOP_STARTTLS,
		STS_LDAP_STOP_NO_CA,	    STS_LDAP_STOP_BAD_MODE,
	};
	size_t i;
	size_t j;

	for (i = 0U; i < (sizeof(all) / sizeof(all[0])); i++) {
		const char *a = sts_ldap_go_reason(all[i]);

		TEST_ASSERT_NOT_NULL(a);
		TEST_ASSERT_TRUE(strlen(a) > 0U);
		for (j = i + 1U; j < (sizeof(all) / sizeof(all[0])); j++) {
			TEST_ASSERT_TRUE(strcmp(a, sts_ldap_go_reason(all[j])) !=
					 0);
		}
	}
}

/** The StartTLS refusal points at the mode that does work. */
static void test_the_starttls_refusal_points_at_ldaps(void)
{
	const char *why = sts_ldap_go_reason(STS_LDAP_STOP_STARTTLS);

	TEST_ASSERT_NOT_NULL(strstr(why, "LDAPS"));
	TEST_ASSERT_NOT_NULL(strstr(why, "sec.ldap.mode=2"));
}

/* ===================================================================== */
/* 2. screening the anchor                                               */
/* ===================================================================== */

static void test_a_good_anchor_is_accepted(void)
{
	TEST_ASSERT_EQUAL_INT(STS_LDAP_CA_OK,
			      sts_ldap_ca_check(ONE_CERT, strlen(ONE_CERT)));
}

/** A root plus a cross-signed root is one anchor set, not two installs. */
static void test_a_two_certificate_anchor_set_is_accepted(void)
{
	TEST_ASSERT_EQUAL_INT(STS_LDAP_CA_OK,
			      sts_ldap_ca_check(TWO_CERTS, strlen(TWO_CERTS)));
}

/** Operators paste out of terminals; surrounding noise is not a defect. */
static void test_surrounding_text_is_tolerated(void)
{
	static const char noisy[] = "subject=CN=corp-root\nissuer=CN=corp-root\n"
				    ONE_CERT "\n";

	TEST_ASSERT_EQUAL_INT(STS_LDAP_CA_OK,
			      sts_ldap_ca_check(noisy, strlen(noisy)));
}

static void test_an_empty_anchor_is_refused(void)
{
	TEST_ASSERT_EQUAL_INT(STS_LDAP_CA_EMPTY, sts_ldap_ca_check(NULL, 0U));
	TEST_ASSERT_EQUAL_INT(STS_LDAP_CA_EMPTY, sts_ldap_ca_check(NULL, 99U));
	TEST_ASSERT_EQUAL_INT(STS_LDAP_CA_EMPTY, sts_ldap_ca_check(ONE_CERT, 0U));
}

/**
 * The size gate is exact on both sides, because it is what stops
 * ldap_ca_adopt()'s memcpy() from running off the end of a fixed buffer.
 */
static void test_the_size_bound_is_exact(void)
{
	size_t n;

	n = make_sized((size_t)STS_LDAP_CA_PEM_MAX);
	TEST_ASSERT_EQUAL_INT(STS_LDAP_CA_OK, sts_ldap_ca_check(big, n));

	n = make_sized((size_t)STS_LDAP_CA_PEM_MAX + 1U);
	TEST_ASSERT_EQUAL_INT(STS_LDAP_CA_TOO_BIG, sts_ldap_ca_check(big, n));

	n = make_sized((size_t)STS_LDAP_CA_PEM_MAX + 63U);
	TEST_ASSERT_EQUAL_INT(STS_LDAP_CA_TOO_BIG, sts_ldap_ca_check(big, n));
}

/** The cap has to clear the largest root an enterprise actually deploys. */
static void test_the_buffer_holds_an_rsa_4096_root(void)
{
	/* 1 500 B DER -> 2 000 base64 chars + ~32 line breaks + 52 framing. */
	TEST_ASSERT_TRUE((size_t)STS_LDAP_CA_PEM_MAX >= 2085U);
}

static void test_a_truncated_or_absent_block_is_refused(void)
{
	static const char no_begin[] = CERT_BODY "-----END CERTIFICATE-----\n";
	static const char no_end[] = "-----BEGIN CERTIFICATE-----\n" CERT_BODY;
	static const char reversed[] = "-----END CERTIFICATE-----\n" CERT_BODY
				       "-----BEGIN CERTIFICATE-----\n";
	static const char junk[] = "not a certificate at all\n";

	TEST_ASSERT_EQUAL_INT(STS_LDAP_CA_MALFORMED,
			      sts_ldap_ca_check(no_begin, strlen(no_begin)));
	TEST_ASSERT_EQUAL_INT(STS_LDAP_CA_MALFORMED,
			      sts_ldap_ca_check(no_end, strlen(no_end)));
	/* An END that precedes the BEGIN closes nothing. */
	TEST_ASSERT_EQUAL_INT(STS_LDAP_CA_MALFORMED,
			      sts_ldap_ca_check(reversed, strlen(reversed)));
	TEST_ASSERT_EQUAL_INT(STS_LDAP_CA_MALFORMED,
			      sts_ldap_ca_check(junk, strlen(junk)));
}

/**
 * An embedded NUL makes the file and the parse disagree: mbedTLS finds the PEM
 * framing with strstr(), so it would see only the prefix while /lfs stored the
 * whole blob.
 */
static void test_an_embedded_nul_is_refused(void)
{
	char buf[sizeof(ONE_CERT) + 8U];
	size_t n = strlen(ONE_CERT);

	memcpy(buf, ONE_CERT, n);
	memcpy(&buf[n], "trail", 5U);
	n += 5U;
	buf[4] = '\0'; /* inside the BEGIN line */

	TEST_ASSERT_EQUAL_INT(STS_LDAP_CA_MALFORMED, sts_ldap_ca_check(buf, n));
}

/**
 * A trust anchor is a public object. The blob is written to /lfs verbatim, so a
 * server bundle pasted into the wrong box would leave a private key in a plain
 * file — and mbedTLS would not complain, because it simply ignores the block.
 */
static void test_a_blob_carrying_a_private_key_is_refused(void)
{
	static const char *const bundles[] = {
		ONE_CERT "-----BEGIN PRIVATE KEY-----\nAAAA\n"
			 "-----END PRIVATE KEY-----\n",
		ONE_CERT "-----BEGIN EC PRIVATE KEY-----\nAAAA\n"
			 "-----END EC PRIVATE KEY-----\n",
		ONE_CERT "-----BEGIN RSA PRIVATE KEY-----\nAAAA\n"
			 "-----END RSA PRIVATE KEY-----\n",
		ONE_CERT "-----BEGIN ENCRYPTED PRIVATE KEY-----\nAAAA\n"
			 "-----END ENCRYPTED PRIVATE KEY-----\n",
		/* Key first, certificate second: order must not matter. */
		"-----BEGIN PRIVATE KEY-----\nAAAA\n-----END PRIVATE KEY-----\n" ONE_CERT,
	};
	size_t i;

	for (i = 0U; i < (sizeof(bundles) / sizeof(bundles[0])); i++) {
		TEST_ASSERT_EQUAL_INT(
			STS_LDAP_CA_HAS_KEY,
			sts_ldap_ca_check(bundles[i], strlen(bundles[i])));
	}
}

/** Every refusal explains itself, and no two explanations are the same text. */
static void test_every_anchor_verdict_has_its_own_reason(void)
{
	static const sts_ldap_ca_verdict_t all[] = {
		STS_LDAP_CA_OK,	       STS_LDAP_CA_EMPTY,
		STS_LDAP_CA_TOO_BIG,   STS_LDAP_CA_MALFORMED,
		STS_LDAP_CA_HAS_KEY,
	};
	size_t i;
	size_t j;

	for (i = 0U; i < (sizeof(all) / sizeof(all[0])); i++) {
		const char *a = sts_ldap_ca_reason(all[i]);

		TEST_ASSERT_NOT_NULL(a);
		TEST_ASSERT_TRUE(strlen(a) > 0U);
		for (j = i + 1U; j < (sizeof(all) / sizeof(all[0])); j++) {
			TEST_ASSERT_TRUE(
				strcmp(a, sts_ldap_ca_reason(all[j])) != 0);
		}
	}
}

/* ===================================================================== */
/* 3. the substring primitive the screening is built on                  */
/* ===================================================================== */

static void test_the_bounded_search_stays_inside_its_span(void)
{
	static const char hay[] = "abcdefabc";
	size_t n = strlen(hay);

	TEST_ASSERT_EQUAL_UINT(0U, sts_ldap_ca_find(hay, n, "abc", 3U, 0U));
	TEST_ASSERT_EQUAL_UINT(6U, sts_ldap_ca_find(hay, n, "abc", 3U, 1U));
	/* Only the first six bytes are in scope, so the tail match is invisible. */
	TEST_ASSERT_EQUAL_UINT((size_t)-1,
			       sts_ldap_ca_find(hay, 6U, "abc", 3U, 1U));
	/*
	 * A match that STRADDLES the end of the span is an out-of-bounds read,
	 * not a hit. `hay` really holds "zzzabc", but only its first four bytes
	 * are in scope, so the "abc" at offset 3 lies past the end: a loop that
	 * bounds `i` by n instead of by (i + nn) reports index 3 here after
	 * memcmp() has read two bytes it was never given. In sts_ldap_ca_check()
	 * that buffer is the operator's PEM body, so the overrun is real.
	 */
	{
		static const char straddle[] = "zzzabc";

		TEST_ASSERT_EQUAL_UINT(
			(size_t)-1,
			sts_ldap_ca_find(straddle, 4U, "abc", 3U, 0U));
	}
	/* A needle longer than the span can never match. */
	TEST_ASSERT_EQUAL_UINT((size_t)-1,
			       sts_ldap_ca_find(hay, 2U, "abc", 3U, 0U));
	/* Degenerate inputs answer "absent" rather than reading anything. */
	TEST_ASSERT_EQUAL_UINT((size_t)-1,
			       sts_ldap_ca_find(NULL, 9U, "abc", 3U, 0U));
	TEST_ASSERT_EQUAL_UINT((size_t)-1, sts_ldap_ca_find(hay, n, NULL, 3U, 0U));
	TEST_ASSERT_EQUAL_UINT((size_t)-1, sts_ldap_ca_find(hay, n, "abc", 0U, 0U));
	/* A start past the end is not an out-of-bounds read. */
	TEST_ASSERT_EQUAL_UINT((size_t)-1,
			       sts_ldap_ca_find(hay, n, "abc", 3U, n + 10U));
}

/* ===================================================================== */
/* 4. the credential tag and the storage path                            */
/* ===================================================================== */

/**
 * The anchor's tag must not be the web server's.
 *
 * STS_WEB_SEC_TAG holds this box's own certificate and PRIVATE KEY. Arming the
 * outbound LDAPS socket with that tag would load them as the client's
 * credential set instead of a trust anchor — and, because
 * tls_mbedtls_set_credentials() only builds a CA chain from a tag that supplied
 * one, would leave verification with nothing to verify against.
 */
static void test_the_anchor_tag_is_not_the_web_server_tag(void)
{
	TEST_ASSERT_NOT_EQUAL_UINT32((uint32_t)STS_WEB_SEC_TAG,
				     (uint32_t)STS_LDAP_CA_SEC_TAG);
}

/**
 * The anchor lives beside the server identity, so the factory-reset sweep and
 * any future sealing layer have one directory to find them in.
 */
static void test_the_anchor_is_stored_under_the_shared_tls_directory(void)
{
	TEST_ASSERT_EQUAL_STRING(STS_LDAP_CA_DIR "/ldap_ca.pem",
				 STS_LDAP_CA_PATH);
	TEST_ASSERT_EQUAL_STRING("/lfs/tls", STS_LDAP_CA_DIR);
	/* sts_web.h owns the same directory for the server credential. */
	TEST_ASSERT_EQUAL_INT(0, strncmp(STS_CERT_DIR, STS_LDAP_CA_DIR,
					 strlen(STS_LDAP_CA_DIR)));
}

/** The mode numbers are the cfg schema's, not this header's opinion. */
static void test_the_mode_numbers_match_the_cfg_schema(void)
{
	TEST_ASSERT_EQUAL_UINT(0U, (unsigned int)STS_LDAP_MODE_PLAIN);
	TEST_ASSERT_EQUAL_UINT(1U, (unsigned int)STS_LDAP_MODE_STARTTLS);
	TEST_ASSERT_EQUAL_UINT(2U, (unsigned int)STS_LDAP_MODE_LDAPS);
}

/* ===================================================================== */

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_ldaps_without_an_anchor_never_opens_a_socket);
	RUN_TEST(test_the_missing_anchor_is_named_in_the_reason);
	RUN_TEST(test_ldaps_with_an_anchor_proceeds_over_tls);
	RUN_TEST(test_starttls_is_always_refused);
	RUN_TEST(test_plain_mode_is_unaffected);
	RUN_TEST(test_a_tls_mode_in_a_non_tls_build_is_refused);
	RUN_TEST(test_an_out_of_range_mode_is_refused);
	RUN_TEST(test_no_tls_mode_ever_reaches_a_cleartext_socket);
	RUN_TEST(test_only_a_fully_armed_ldaps_opens_a_tls_socket);
	RUN_TEST(test_every_outcome_has_its_own_reason);
	RUN_TEST(test_the_starttls_refusal_points_at_ldaps);

	RUN_TEST(test_a_good_anchor_is_accepted);
	RUN_TEST(test_a_two_certificate_anchor_set_is_accepted);
	RUN_TEST(test_surrounding_text_is_tolerated);
	RUN_TEST(test_an_empty_anchor_is_refused);
	RUN_TEST(test_the_size_bound_is_exact);
	RUN_TEST(test_the_buffer_holds_an_rsa_4096_root);
	RUN_TEST(test_a_truncated_or_absent_block_is_refused);
	RUN_TEST(test_an_embedded_nul_is_refused);
	RUN_TEST(test_a_blob_carrying_a_private_key_is_refused);
	RUN_TEST(test_every_anchor_verdict_has_its_own_reason);

	RUN_TEST(test_the_bounded_search_stays_inside_its_span);

	RUN_TEST(test_the_anchor_tag_is_not_the_web_server_tag);
	RUN_TEST(test_the_anchor_is_stored_under_the_shared_tls_directory);
	RUN_TEST(test_the_mode_numbers_match_the_cfg_schema);

	return UNITY_END();
}
