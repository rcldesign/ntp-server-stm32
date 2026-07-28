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
 *
 *   6. THE ERASE REPORTS THE PERSISTED HALF AND NOTHING ELSE. The in-RAM anchor
 *      cannot outlive the reboot that every factory-reset plane performs; the
 *      /lfs file can, and it is the only thing the result may depend on.
 *      Reporting a contended RAM half as -EIO propagates through
 *      sts_factory_result() and brands a correctly-wiped unit as one that must
 *      not be treated as decommissioned — which is how a decommissioning gets
 *      repeated, or abandoned.
 *
 *   7. THE PERSISTED HALF NEVER WAITS FOR AN AUTHENTICATION. This one is
 *      structural, so it is checked by reading sts_aaa.c rather than by calling
 *      anything: the unlink must happen before any mutex is taken and must not
 *      be nested under a condition, and the wait for the RAM half must be
 *      bounded. The erase used to take sts_aaa.c's exchange mutex with
 *      K_FOREVER — a mutex held for the length of a directory bind (240 s for
 *      LDAP, ~540 s for a full chain, unbounded against a server that dribbles)
 *      — while running inline on a liveness participant with a 5 000 ms
 *      deadline. A factory reset issued during a login therefore cold-cycled
 *      the board after the config, the web credentials and the TLS identity had
 *      gone but before the anchor had, and before any plane could annunciate an
 *      incomplete reset. The unit came back looking clean and still trusting
 *      the previous operator's CA. Nothing about that is visible in a passing
 *      unit test of the arithmetic, hence the scan.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "unity.h"

#include "zephyr/net/sts_ldap_ca.h"
#include "zephyr/net/sts_web.h" /* STS_WEB_SEC_TAG, for the collision test */

#ifndef STS_APP_SRC_DIR
#error "STS_APP_SRC_DIR must be defined by tests/host/CMakeLists.txt"
#endif

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
 *
 * WHERE the NUL goes is the whole test. Put it in the BEGIN line and the
 * framing search fails on its own account, so sts_ldap_ca_check() answers
 * MALFORMED down the *framing* path with or without the memchr() guard — the
 * assertion stays green with the guard deleted, which is a tick for the one
 * defect it names. In the base64 BODY the framing is intact, so the same blob
 * is OK without the guard and MALFORMED with it. The OK assertion below is the
 * control that pins that: remove it and this test is back to proving nothing in
 * particular.
 */
static void test_an_embedded_nul_is_refused(void)
{
	char buf[sizeof(ONE_CERT) + 8U];
	size_t n = strlen(ONE_CERT);
	/* ONE_CERT is BEGIN + '\n' + CERT_BODY + END, so the base64 starts one
	 * past the BEGIN line — sizeof() counts the NUL, which stands in for it. */
	size_t body = sizeof(STS_LDAP_CA_BEGIN);

	memcpy(buf, ONE_CERT, n);
	memcpy(&buf[n], "trail", 5U);
	n += 5U;

	/* The premise: `body` really is the first byte of the base64, and the
	 * blob is otherwise a well-framed anchor. */
	TEST_ASSERT_EQUAL_INT((int)CERT_BODY[0], (int)buf[body]);
	TEST_ASSERT_EQUAL_INT(STS_LDAP_CA_OK, sts_ldap_ca_check(buf, n));

	buf[body + 4U] = '\0'; /* inside the base64, not the framing */
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
/* 5. erasing the anchor — the arithmetic                                */
/* ===================================================================== */

/** The eight (fs_ready, unlinked, ram_erased) states, as a bit triple. */
static sts_ldap_ca_erase_t erase_of(unsigned int bits)
{
	sts_ldap_ca_erase_t e;

	e.fs_ready = (bits & 1U) != 0U;
	e.unlinked = (bits & 2U) != 0U;
	e.ram_erased = (bits & 4U) != 0U;
	return e;
}

/** What sts_ldap_ca.h says the result must be, restated independently. */
static int want_erase_result(const sts_ldap_ca_erase_t *e)
{
	if (e->fs_ready && !e->unlinked) {
		return -EIO;
	}
	return 0;
}

/**
 * The whole input domain, not a sample.
 *
 * The dangerous direction is the one where a contended RAM half turns into a
 * failure: sts_sec_factory_wipe() folds this return into its own, which
 * sts_factory_result() folds into the plane's, which is what decides whether
 * the operator is told "factory reset INCOMPLETE; do not treat this unit as
 * decommissioned". An anchor that is already gone from /lfs and has at most one
 * second of RAM left to live is not an incomplete reset.
 */
static void test_the_erase_result_over_the_whole_domain(void)
{
	unsigned int bits;

	for (bits = 0U; bits < 8U; bits++) {
		sts_ldap_ca_erase_t e = erase_of(bits);
		char msg[96];

		(void)snprintf(msg, sizeof(msg),
			       "fs_ready=%d unlinked=%d ram_erased=%d",
			       (int)e.fs_ready, (int)e.unlinked,
			       (int)e.ram_erased);
		TEST_ASSERT_EQUAL_INT_MESSAGE(want_erase_result(&e),
					      sts_ldap_ca_erase_result(&e), msg);
	}

	/* A NULL record is not "nothing happened"; it is an erase that cannot
	 * say what it did, which must never read as a clean one. */
	TEST_ASSERT_EQUAL_INT(-EIO, sts_ldap_ca_erase_result(NULL));
}

/**
 * The RAM half never reaches the result — stated on its own, because it is the
 * single rule the whole split depends on.
 *
 * For every (fs_ready, unlinked) pair, flipping ram_erased must change nothing.
 */
static void test_a_deferred_ram_half_is_not_a_failure(void)
{
	unsigned int bits;

	for (bits = 0U; bits < 4U; bits++) {
		sts_ldap_ca_erase_t took = erase_of(bits | 4U);
		sts_ldap_ca_erase_t left = erase_of(bits);
		char msg[96];

		(void)snprintf(msg, sizeof(msg), "fs_ready=%d unlinked=%d",
			       (int)left.fs_ready, (int)left.unlinked);
		TEST_ASSERT_EQUAL_INT_MESSAGE(sts_ldap_ca_erase_result(&took),
					      sts_ldap_ca_erase_result(&left),
					      msg);
	}

	/* And the erase that mattered still fails when the FILE survives. */
	{
		sts_ldap_ca_erase_t e = erase_of(0U | 4U); /* fs down, RAM gone */

		TEST_ASSERT_EQUAL_INT(0, sts_ldap_ca_erase_result(&e));
		e.fs_ready = true;
		e.unlinked = false;
		TEST_ASSERT_EQUAL_INT(-EIO, sts_ldap_ca_erase_result(&e));
	}
}

/**
 * A /lfs that is not mounted is not a failure — the same convention
 * sts_cert_reset() uses. There is then nothing persisted for the call to
 * remove, and `unlinked` is meaningless rather than false.
 */
static void test_a_dead_filesystem_is_not_a_failed_erase(void)
{
	sts_ldap_ca_erase_t e = { false, false, false };

	TEST_ASSERT_EQUAL_INT(0, sts_ldap_ca_erase_result(&e));
	e.unlinked = true;
	TEST_ASSERT_EQUAL_INT(0, sts_ldap_ca_erase_result(&e));
}

/**
 * -ENOENT is the goal, not an error. A unit that never had an anchor is already
 * in the state the erase is trying to reach; anything else from the filesystem
 * means the file may still be there for the next boot to load.
 */
static void test_an_absent_file_is_a_successful_unlink(void)
{
	TEST_ASSERT_TRUE(sts_ldap_ca_unlink_ok(0));
	TEST_ASSERT_TRUE(sts_ldap_ca_unlink_ok(-ENOENT));

	TEST_ASSERT_FALSE(sts_ldap_ca_unlink_ok(-EIO));
	TEST_ASSERT_FALSE(sts_ldap_ca_unlink_ok(-EROFS));
	TEST_ASSERT_FALSE(sts_ldap_ca_unlink_ok(-EBUSY));
	TEST_ASSERT_FALSE(sts_ldap_ca_unlink_ok(-EACCES));
	TEST_ASSERT_FALSE(sts_ldap_ca_unlink_ok(-EINVAL));
	/* A positive return is not a Zephyr fs error code, so it is not one of
	 * the two states this is allowed to accept. */
	TEST_ASSERT_FALSE(sts_ldap_ca_unlink_ok(ENOENT));
}

/** The log line the caller owes the operator, and only when it is owed. */
static void test_a_deferred_ram_half_is_annunciated(void)
{
	unsigned int bits;

	for (bits = 0U; bits < 8U; bits++) {
		sts_ldap_ca_erase_t e = erase_of(bits);

		TEST_ASSERT_EQUAL_INT((int)!e.ram_erased,
				      (int)sts_ldap_ca_erase_ram_deferred(&e));
	}

	/* An erase that cannot say what it did did not clear RAM either. */
	TEST_ASSERT_FALSE(sts_ldap_ca_erase_ram_deferred(NULL));
}

/* ===================================================================== */
/* 6. erasing the anchor — the shape of sts_aaa.c                        */
/* ===================================================================== */

/*
 * The arithmetic above is only worth anything if the implementation feeds it
 * honestly, and the property that matters here cannot be expressed in C: the
 * persisted unlink must run BEFORE the exchange mutex is taken and must not be
 * nested under anything, and the wait for the mutex must be bounded.
 *
 * Put the unlink back inside the lock and every test above still passes, the
 * image still links, and the defect is invisible until an operator wipes a unit
 * while a directory server is slow — at which point the board cold-cycles with
 * the anchor intact and the reset annunciated as clean. So this half reads
 * sts_aaa.c, in the same spirit as test_factory_policy.c and
 * test_smear_isolation.c.
 *
 * It is written so it cannot pass vacuously: it asserts that the file was
 * found, that the stripper actually stripped something, that BOTH definitions
 * of the erase were located, and that the bodies it matched are a plausible
 * size.
 */

#define SRC_MAX (256U * 1024U)

static char g_raw[SRC_MAX];
static char g_code[SRC_MAX]; /* g_raw with comments and literals blanked */
static size_t g_len;
static size_t g_stripped; /* characters the stripper blanked */

static void read_source(const char *rel)
{
	char path[512];
	FILE *f;
	size_t n;

	(void)snprintf(path, sizeof(path), "%s/%s", STS_APP_SRC_DIR, rel);
	f = fopen(path, "rb");
	TEST_ASSERT_NOT_NULL_MESSAGE(f, path);
	n = fread(g_raw, 1U, sizeof(g_raw), f);
	/* A file that exactly filled the buffer was probably truncated. */
	TEST_ASSERT_TRUE_MESSAGE(n < sizeof(g_raw), "raise SRC_MAX");
	(void)fclose(f);
	g_len = n;
	g_raw[n] = '\0';
}

/**
 * Blank every comment, string literal and character literal, in place, one
 * space per character and newlines preserved.
 *
 * Blanking rather than deleting keeps byte offsets 1:1 with the file, so a
 * failure reports a real line number — and it is what makes the checks honest.
 * sts_aaa.c's own comments name K_FOREVER, fs_unlink and the mutex while
 * explaining why the erase no longer uses them that way; a scanner that counted
 * prose would fail on correct code and, worse, could be satisfied by a comment
 * on incorrect code.
 */
static void strip(void)
{
	enum { CODE, BLOCK, LINE, STR, CHR } st = CODE;
	size_t i;

	g_stripped = 0U;
	for (i = 0U; i < g_len; i++) {
		char c = g_raw[i];
		char nx = (i + 1U < g_len) ? g_raw[i + 1U] : '\0';
		bool blank = (st != CODE);

		switch (st) {
		case CODE:
			if ((c == '/') && (nx == '*')) {
				st = BLOCK;
				blank = true;
			} else if ((c == '/') && (nx == '/')) {
				st = LINE;
				blank = true;
			} else if (c == '"') {
				st = STR;
				blank = true;
			} else if (c == '\'') {
				st = CHR;
				blank = true;
			}
			break;
		case BLOCK:
			if ((c == '*') && (nx == '/')) {
				g_code[i] = ' ';
				g_stripped++;
				i++;
				g_code[i] = ' ';
				g_stripped++;
				st = CODE;
				continue;
			}
			break;
		case LINE:
			if (c == '\n') {
				st = CODE;
				blank = false;
			}
			break;
		case STR:
		case CHR:
			if (c == '\\') {
				g_code[i] = ' ';
				g_stripped++;
				if (i + 1U < g_len) {
					i++;
					g_code[i] = ' ';
					g_stripped++;
				}
				continue;
			}
			if (((st == STR) && (c == '"')) ||
			    ((st == CHR) && (c == '\''))) {
				st = CODE;
			}
			break;
		default:
			break;
		}

		if (blank && (c != '\n')) {
			g_code[i] = ' ';
			g_stripped++;
		} else {
			g_code[i] = c;
		}
	}
	g_code[g_len] = '\0';
}

static bool ident_char(char c)
{
	return ((c >= 'a') && (c <= 'z')) || ((c >= 'A') && (c <= 'Z')) ||
	       ((c >= '0') && (c <= '9')) || (c == '_');
}

/** 1-based line number of byte offset @p off. */
static unsigned int line_of(size_t off)
{
	unsigned int line = 1U;
	size_t i;

	for (i = 0U; (i < off) && (i < g_len); i++) {
		if (g_code[i] == '\n') {
			line++;
		}
	}
	return line;
}

/** Offset of the first whole-token @p tok in g_code[from, to), or SIZE_MAX. */
static size_t find_token(const char *tok, size_t from, size_t to)
{
	size_t n = strlen(tok);
	size_t i;

	if (to > g_len) {
		to = g_len;
	}
	for (i = from; (i + n) <= to; i++) {
		if (memcmp(&g_code[i], tok, n) != 0) {
			continue;
		}
		if ((i > 0U) && ident_char(g_code[i - 1U])) {
			continue;
		}
		if (((i + n) < g_len) && ident_char(g_code[i + n])) {
			continue;
		}
		return i;
	}
	return (size_t)-1;
}

/** As find_token(), but only where the token is immediately applied as a call. */
static size_t find_call(const char *fn, size_t from, size_t to)
{
	size_t at = from;

	for (;;) {
		size_t j;

		at = find_token(fn, at, to);
		if (at == (size_t)-1) {
			return (size_t)-1;
		}
		j = at + strlen(fn);
		while ((j < g_len) && ((g_code[j] == ' ') ||
				       (g_code[j] == '\t') ||
				       (g_code[j] == '\n'))) {
			j++;
		}
		if ((j < g_len) && (g_code[j] == '(')) {
			return at;
		}
		at++;
	}
}

/** A brace-matched body: [begin, end). */
typedef struct {
	size_t begin;
	size_t end;
} body_t;

/**
 * The block that follows the occurrence of @p anchor at or after @p from.
 *
 * Safe on the stripped text: every brace inside a comment or a string literal
 * has already been blanked out.
 *
 * @return false when @p anchor does not occur again.
 */
static bool body_after(const char *anchor, size_t from, body_t *out,
		       size_t *out_at)
{
	size_t at = find_token(anchor, from, g_len);
	size_t i;
	unsigned int depth = 0U;

	if (at == (size_t)-1) {
		return false;
	}
	*out_at = at;

	for (i = at; i < g_len; i++) {
		if (g_code[i] == '{') {
			break;
		}
	}
	TEST_ASSERT_TRUE_MESSAGE(i < g_len, anchor);

	out->begin = i + 1U;
	for (; i < g_len; i++) {
		if (g_code[i] == '{') {
			depth++;
		} else if (g_code[i] == '}') {
			depth--;
			if (depth == 0U) {
				out->end = i;
				return true;
			}
		}
	}
	TEST_FAIL_MESSAGE(anchor);
	return false;
}

/** Brace depth of @p off relative to the start of @p b (1 = directly in it). */
static unsigned int depth_at(const body_t *b, size_t off)
{
	unsigned int depth = 1U;
	size_t i;

	for (i = b->begin; (i < off) && (i < b->end); i++) {
		if (g_code[i] == '{') {
			depth++;
		} else if (g_code[i] == '}') {
			depth--;
		}
	}
	return depth;
}

#define AAA_SRC "zephyr/net/sts_aaa.c"
#define ERASE_FN "sts_aaa_ldap_ca_erase"
#define PERSIST_FN "ca_persist_erase"

/**
 * The erase's own body, out of sts_aaa.c.
 *
 * sts_aaa.c defines the function twice — the real one and, under
 * `#else /· !CONFIG_NET_SOCKETS_SOCKOPT_TLS ·/`, a stub that returns 0 because
 * a build with no TLS socket layer can hold no anchor. Both are located, so a
 * scan that silently matched the stub would fail instead of reporting a clean
 * file, and the one that erases anything is the one returned.
 */
static body_t erase_body(void)
{
	body_t chosen = { 0U, 0U };
	size_t found = 0U;
	size_t erasing = 0U;
	size_t from = 0U;

	for (;;) {
		body_t b;
		size_t at;

		if (!body_after(ERASE_FN, from, &b, &at)) {
			break;
		}
		found++;
		if (find_call(PERSIST_FN, b.begin, b.end) != (size_t)-1) {
			erasing++;
			chosen = b;
		}
		from = b.end;
	}

	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		2U, (unsigned int)found,
		"expected two definitions of " ERASE_FN " in " AAA_SRC
		" (the TLS one and the no-TLS stub)");
	TEST_ASSERT_EQUAL_UINT_MESSAGE(
		1U, (unsigned int)erasing,
		"exactly one definition of " ERASE_FN " should erase anything");
	TEST_ASSERT_TRUE_MESSAGE((chosen.end - chosen.begin) > 200U,
				 "the erase body is implausibly short — the "
				 "scan matched the wrong thing");
	return chosen;
}

static void load_aaa(void)
{
	read_source(AAA_SRC);
	strip();
	TEST_ASSERT_TRUE_MESSAGE(g_len > 10000U, AAA_SRC " is implausibly short");
	TEST_ASSERT_TRUE_MESSAGE(g_stripped > 1000U,
				 "the stripper blanked almost nothing, so the "
				 "checks below are not reading code");
}

/**
 * The persisted erase runs first, before any mutex is taken.
 *
 * This is the whole fix. The /lfs file is the only half a reboot cannot clear,
 * and g_lock is held for the length of a directory bind — so an unlink that
 * queues behind the mutex is an unlink that a hostile directory can hold off
 * until the watchdog cold-cycles the board mid-wipe.
 */
static void test_the_persisted_erase_precedes_the_mutex(void)
{
	body_t b;
	size_t unlink_at;
	size_t lock_at;
	char msg[192];

	load_aaa();
	b = erase_body();

	unlink_at = find_call(PERSIST_FN, b.begin, b.end);
	lock_at = find_call("k_mutex_lock", b.begin, b.end);

	TEST_ASSERT_TRUE_MESSAGE(unlink_at != (size_t)-1,
				 ERASE_FN "() does not erase the /lfs anchor");
	TEST_ASSERT_TRUE_MESSAGE(lock_at != (size_t)-1,
				 ERASE_FN "() no longer takes the exchange "
				 "mutex at all — the RAM half needs it");

	(void)snprintf(msg, sizeof(msg),
		       AAA_SRC ":%u — " PERSIST_FN "() must run before the "
		       "k_mutex_lock() at line %u",
		       line_of(unlink_at), line_of(lock_at));
	TEST_ASSERT_TRUE_MESSAGE(unlink_at < lock_at, msg);
}

/**
 * …and it runs unconditionally.
 *
 * "Before the lock" is not enough on its own: `if (k_mutex_lock(...) == 0)
 * { unlink; }` also puts the call textually first and still makes the load-
 * bearing half contingent on a mutex a bind can hold. The first call must
 * therefore sit at the body's own brace depth, gated by nothing.
 *
 * The SECOND call is expected to be nested — it repeats the unlink inside the
 * lock, so an install that persisted a new anchor in the window cannot survive
 * the reset — which is why only the first is checked.
 */
static void test_the_persisted_erase_is_not_conditional(void)
{
	body_t b;
	size_t at;
	char msg[192];

	load_aaa();
	b = erase_body();

	at = find_call(PERSIST_FN, b.begin, b.end);
	TEST_ASSERT_TRUE(at != (size_t)-1);

	(void)snprintf(msg, sizeof(msg),
		       AAA_SRC ":%u — the first " PERSIST_FN "() is nested at "
		       "depth %u; it must be gated by nothing",
		       line_of(at), depth_at(&b, at));
	TEST_ASSERT_EQUAL_UINT_MESSAGE(1U, depth_at(&b, at), msg);
}

/**
 * The wait for the RAM half is bounded.
 *
 * K_FOREVER here is the original defect verbatim: the caller runs inline on a
 * liveness participant with CONFIG_STS1000_LIVENESS_DEADLINE_MS to spend, and
 * the holder it would be waiting for is an authentication exchange with no
 * ceiling at all.
 */
static void test_the_ram_half_waits_with_a_deadline(void)
{
	body_t b;
	char msg[128];

	load_aaa();
	b = erase_body();

	if (find_token("K_FOREVER", b.begin, b.end) != (size_t)-1) {
		(void)snprintf(msg, sizeof(msg),
			       AAA_SRC ":%u — " ERASE_FN "() waits forever for "
			       "the exchange mutex",
			       line_of(find_token("K_FOREVER", b.begin, b.end)));
		TEST_FAIL_MESSAGE(msg);
	}
	TEST_ASSERT_TRUE_MESSAGE(find_token("K_MSEC", b.begin, b.end) !=
					 (size_t)-1,
				 ERASE_FN "() takes the exchange mutex with no "
				 "millisecond deadline");
}

/**
 * The helper that does the persisted half holds no lock and must not grow one.
 *
 * Moving the mutex one level down would satisfy every check above while
 * restoring the defect exactly.
 */
static void test_the_persisted_erase_helper_takes_no_lock(void)
{
	body_t b;
	size_t at;

	load_aaa();
	if (!body_after("static void " PERSIST_FN, 0U, &b, &at)) {
		TEST_FAIL_MESSAGE("no " PERSIST_FN "() definition in " AAA_SRC);
		return;
	}

	TEST_ASSERT_TRUE_MESSAGE(find_call("fs_unlink", b.begin, b.end) !=
					 (size_t)-1,
				 PERSIST_FN "() does not unlink the anchor");
	TEST_ASSERT_TRUE_MESSAGE(find_call("k_mutex_lock", b.begin, b.end) ==
					 (size_t)-1,
				 PERSIST_FN "() must take no lock: it is the "
				 "half that cannot be allowed to wait");
	TEST_ASSERT_TRUE_MESSAGE(find_token("K_FOREVER", b.begin, b.end) ==
					 (size_t)-1,
				 PERSIST_FN "() waits forever for something");
}

/**
 * The result the shipped code returns is the header's, not a second opinion.
 *
 * A hand-rolled `return rc;` alongside the reducer is how the two drift, and
 * the arithmetic suite above would keep passing while the appliance did
 * something else.
 */
static void test_the_erase_returns_the_header_s_verdict(void)
{
	body_t b;

	load_aaa();
	b = erase_body();

	TEST_ASSERT_TRUE_MESSAGE(
		find_call("sts_ldap_ca_erase_result", b.begin, b.end) !=
			(size_t)-1,
		ERASE_FN "() does not use sts_ldap_ca_erase_result(); the "
		"shipped verdict and the tested one must be one decision");
	TEST_ASSERT_TRUE_MESSAGE(
		find_call("sts_ldap_ca_erase_ram_deferred", b.begin, b.end) !=
			(size_t)-1,
		ERASE_FN "() does not annunciate a deferred RAM half");
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

	RUN_TEST(test_the_erase_result_over_the_whole_domain);
	RUN_TEST(test_a_deferred_ram_half_is_not_a_failure);
	RUN_TEST(test_a_dead_filesystem_is_not_a_failed_erase);
	RUN_TEST(test_an_absent_file_is_a_successful_unlink);
	RUN_TEST(test_a_deferred_ram_half_is_annunciated);

	RUN_TEST(test_the_persisted_erase_precedes_the_mutex);
	RUN_TEST(test_the_persisted_erase_is_not_conditional);
	RUN_TEST(test_the_ram_half_waits_with_a_deadline);
	RUN_TEST(test_the_persisted_erase_helper_takes_no_lock);
	RUN_TEST(test_the_erase_returns_the_header_s_verdict);

	return UNITY_END();
}
