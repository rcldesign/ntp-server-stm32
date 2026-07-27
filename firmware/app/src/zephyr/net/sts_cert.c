/*
 * STS1000 "Meridian" — TLS server credential lifecycle (spec §9.2).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * ---------------------------------------------------------------------------
 * Why the self-signed certificate is PERSISTED
 * ---------------------------------------------------------------------------
 * Generating a fresh self-signed certificate on every boot is the easy thing to
 * do, and it is wrong: spec §9.2 offers "self-signed + downloadable root for
 * pinning", and a key that changes at every restart makes pinning meaningless
 * and re-prompts the operator's browser after every power cycle. So the key and
 * certificate are written to /lfs on first ever boot and reloaded thereafter.
 *
 * NOR is explicitly not required to boot (ARCHITECTURE.md §3), so the no-/lfs
 * case is handled rather than treated as fatal: the pair is generated in RAM,
 * `persisted` is reported false, and a warning names the consequence. HTTPS
 * still comes up — an operator who cannot reach the management plane at all is
 * worse off than one whose browser warns twice.
 *
 * The key is stored as a plain PEM file on the external NOR. That is a real,
 * named limitation: the honest place for it is a sealed blob (PSA ITS, or
 * wrapped by the ATECC608B per spec §9.1/§9.6), and the sealing layer does not
 * exist in the tree yet. See the TODO below and the report.
 *
 * There is exactly ONE place that intent is reversed: sts_cert_reset(), the
 * factory-reset path (spec §9.6 secure-erase). Persisting the key is right for
 * every boot of a deployed unit and wrong at the moment the unit is wiped, so
 * that function deletes both PEM files, the ACME account key, the Zephyr
 * credential entries and the in-RAM copies. Nothing else on any normal path
 * removes them.
 *
 * ---------------------------------------------------------------------------
 * ACME
 * ---------------------------------------------------------------------------
 * The order/authorisation skeleton is present but compiled out behind
 * CONFIG_STS1000_ACME, which no Kconfig defines today — so it cannot affect the
 * build, and the normal certificate path is untouched. What is and is not
 * implemented is enumerated at the ACME section below.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/tls_credentials.h>

#include <mbedtls/ctr_drbg.h>
#include <mbedtls/ecp.h>
#include <mbedtls/entropy.h>
#include <mbedtls/oid.h>
#include <mbedtls/pk.h>
#include <mbedtls/platform_util.h>
#include <mbedtls/x509_crt.h>
#include <mbedtls/x509_csr.h>

#include "cfg/cfg.h"
#include "net/sts_net.h"
#include "net/sts_secops.h"
#include "net/sts_web.h"
#include "storage/sts_store.h"
#include "web/web.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_cert, CONFIG_STS1000_LOG_LEVEL);

/*
 * The whole body needs X.509 *writing*, which is a distinct mbedTLS option from
 * X.509 parsing. web.conf turns it on; the guard keeps the file compiling if
 * someone cuts it back out, reporting "no certificate support" instead of
 * failing the link.
 */
#if defined(MBEDTLS_X509_CRT_WRITE_C) && defined(MBEDTLS_PK_WRITE_C) && \
	defined(MBEDTLS_ECP_C)
#define STS_CERT_HAVE_WRITE 1
#else
#define STS_CERT_HAVE_WRITE 0
#endif

/* ------------------------------------------------------------------------- */
/* state                                                                     */
/* ------------------------------------------------------------------------- */

static struct k_mutex lock;

static char crt_pem[STS_CERT_PEM_MAX];
static char key_pem[STS_CERT_PEM_MAX];
static size_t crt_pem_len;
static size_t key_pem_len;

static struct {
	bool     ready;
	bool     self_signed;
	bool     operator_supplied;
	bool     persisted;
	char     subject[72];
	char     issuer[72];
	char     not_before[24];
	char     not_after[24];
	char     fingerprint[68];
	char     key_type[16];
	uint16_t key_bits;
	uint8_t  acme_state;
	char     acme_detail[64];
} st;

/* ------------------------------------------------------------------------- */
/* file helpers                                                              */
/* ------------------------------------------------------------------------- */

static int file_read_all(const char *path, char *buf, size_t cap, size_t *out_len)
{
	struct fs_file_t f;
	ssize_t n;
	int rc;

	if (!sts_fs_ready()) {
		return -EROFS;
	}
	fs_file_t_init(&f);
	rc = fs_open(&f, path, FS_O_READ);
	if (rc != 0) {
		return rc;
	}
	n = fs_read(&f, buf, cap - 1U);
	(void)fs_close(&f);
	if (n < 0) {
		return (int)n;
	}
	buf[n] = '\0';
	*out_len = (size_t)n;
	return 0;
}

static int file_write_all(const char *path, const char *buf, size_t len)
{
	struct fs_file_t f;
	ssize_t n;
	int rc;

	if (!sts_fs_ready()) {
		return -EROFS;
	}
	rc = fs_mkdir(STS_CERT_DIR);
	if (rc != 0 && rc != -EEXIST) {
		LOG_WRN("mkdir %s: %d", STS_CERT_DIR, rc);
	}
	fs_file_t_init(&f);
	rc = fs_open(&f, path, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	if (rc != 0) {
		return rc;
	}
	n = fs_write(&f, buf, len);
	if (n >= 0) {
		rc = fs_sync(&f);
	}
	(void)fs_close(&f);
	if (n < 0) {
		return (int)n;
	}
	if ((size_t)n != len) {
		return -EIO;
	}
	return rc;
}

/* ------------------------------------------------------------------------- */
/* description                                                               */
/* ------------------------------------------------------------------------- */

/* SHA-256 of the certificate DER, hex, through the existing crypto port. */
static void note_fingerprint(const mbedtls_x509_crt *crt)
{
	const port_crypto_t *cr = sts_port_crypto();
	uint8_t digest[32];

	st.fingerprint[0] = '\0';
	if (cr == NULL || cr->sha256 == NULL) {
		return;
	}
	if (cr->sha256(cr->ctx, crt->raw.p, crt->raw.len, digest) != 0) {
		return;
	}
	(void)web_hex_encode(digest, sizeof(digest), st.fingerprint,
			     sizeof(st.fingerprint));
}

static void note_time(const mbedtls_x509_time *t, char *out, size_t cap)
{
	(void)snprintf(out, cap, "%04d-%02d-%02dT%02d:%02d:%02dZ", t->year,
		       t->mon, t->day, t->hour, t->min, t->sec);
}

static void describe(const mbedtls_x509_crt *crt)
{
	int n;

	n = mbedtls_x509_dn_gets(st.subject, sizeof(st.subject), &crt->subject);
	if (n < 0) {
		st.subject[0] = '\0';
	}
	n = mbedtls_x509_dn_gets(st.issuer, sizeof(st.issuer), &crt->issuer);
	if (n < 0) {
		st.issuer[0] = '\0';
	}
	note_time(&crt->valid_from, st.not_before, sizeof(st.not_before));
	note_time(&crt->valid_to, st.not_after, sizeof(st.not_after));
	note_fingerprint(crt);

	/*
	 * Self-signed is decided by comparing the encoded issuer and subject
	 * DNs, which is what "self-issued" means; a real chain check would need
	 * the issuer's key and there is none to check against here.
	 */
	st.self_signed = (crt->issuer_raw.len == crt->subject_raw.len) &&
			 (memcmp(crt->issuer_raw.p, crt->subject_raw.p,
				 crt->subject_raw.len) == 0);
	st.operator_supplied = !st.self_signed;

	(void)web_span_copy(st.key_type, sizeof(st.key_type),
			    mbedtls_pk_get_name(&((mbedtls_x509_crt *)crt)->pk),
			    strlen(mbedtls_pk_get_name(
				    &((mbedtls_x509_crt *)crt)->pk)));
	st.key_bits = (uint16_t)mbedtls_pk_get_bitlen(
		&((mbedtls_x509_crt *)crt)->pk);
}

/* ------------------------------------------------------------------------- */
/* credential registration                                                   */
/* ------------------------------------------------------------------------- */

static int register_credential(void)
{
	int rc;

	/* Replacing a tag means deleting first; a missing tag is not an error. */
	(void)tls_credential_delete(STS_WEB_SEC_TAG,
				    TLS_CREDENTIAL_SERVER_CERTIFICATE);
	(void)tls_credential_delete(STS_WEB_SEC_TAG, TLS_CREDENTIAL_PRIVATE_KEY);

	rc = tls_credential_add(STS_WEB_SEC_TAG, TLS_CREDENTIAL_SERVER_CERTIFICATE,
				crt_pem, crt_pem_len + 1U);
	if (rc != 0) {
		LOG_ERR("tls_credential_add(cert): %d", rc);
		return rc;
	}
	rc = tls_credential_add(STS_WEB_SEC_TAG, TLS_CREDENTIAL_PRIVATE_KEY,
				key_pem, key_pem_len + 1U);
	if (rc != 0) {
		LOG_ERR("tls_credential_add(key): %d", rc);
		return rc;
	}
	return 0;
}

/*
 * Validate a PEM certificate + key pair and, if it is good, adopt it as the live
 * credential. Does not touch the filesystem.
 */
static int adopt_pair(const char *crt, size_t crt_len, const char *key,
		      size_t key_len)
{
	mbedtls_x509_crt c;
	mbedtls_pk_context k;
	int rc;

	if (crt_len == 0U || key_len == 0U || crt_len >= STS_CERT_PEM_MAX ||
	    key_len >= STS_CERT_PEM_MAX) {
		return -EBADMSG;
	}

	mbedtls_x509_crt_init(&c);
	mbedtls_pk_init(&k);

	/* mbedTLS wants the NUL counted for PEM input. */
	rc = mbedtls_x509_crt_parse(&c, (const unsigned char *)crt, crt_len + 1U);
	if (rc != 0) {
		LOG_WRN("certificate parse: -0x%04x", (unsigned int)-rc);
		rc = -EBADMSG;
		goto out;
	}
	rc = mbedtls_pk_parse_key(&k, (const unsigned char *)key, key_len + 1U,
				  NULL, 0U, mbedtls_ctr_drbg_random, NULL);
	if (rc != 0) {
		LOG_WRN("key parse: -0x%04x", (unsigned int)-rc);
		rc = -EBADMSG;
		goto out;
	}
	rc = mbedtls_pk_check_pair(&c.pk, &k, mbedtls_ctr_drbg_random, NULL);
	if (rc != 0) {
		LOG_WRN("key does not match certificate: -0x%04x",
			(unsigned int)-rc);
		/* -EPERM, not -EKEYREJECTED: the target libc has no such code
		 * (rest.h "Portable errno only"). */
		rc = -EPERM;
		goto out;
	}

	memcpy(crt_pem, crt, crt_len);
	crt_pem[crt_len] = '\0';
	crt_pem_len = crt_len;
	memcpy(key_pem, key, key_len);
	key_pem[key_len] = '\0';
	key_pem_len = key_len;

	describe(&c);
	rc = register_credential();
	if (rc == 0) {
		st.ready = true;
	}

out:
	mbedtls_pk_free(&k);
	mbedtls_x509_crt_free(&c);
	return rc;
}

/* ------------------------------------------------------------------------- */
/* generation                                                                */
/* ------------------------------------------------------------------------- */

#if STS_CERT_HAVE_WRITE

static mbedtls_entropy_context entropy;
static mbedtls_ctr_drbg_context drbg;
static bool drbg_ready;

static int drbg_start(void)
{
	int rc;

	if (drbg_ready) {
		return 0;
	}
	mbedtls_entropy_init(&entropy);
	mbedtls_ctr_drbg_init(&drbg);
	rc = mbedtls_ctr_drbg_seed(&drbg, mbedtls_entropy_func, &entropy,
				   (const unsigned char *)"sts1000-web-cert", 16U);
	if (rc != 0) {
		LOG_ERR("ctr_drbg_seed: -0x%04x", (unsigned int)-rc);
		return -EIO;
	}
	drbg_ready = true;
	return 0;
}

/*
 * extendedKeyUsage = id-kp-serverAuth. mbedTLS only offers the generic setter
 * over an ASN.1 OID sequence, so the one OID this certificate needs is spelled
 * out here rather than hand-rolling DER at the call site.
 */
static int set_eku_server_auth(mbedtls_x509write_cert *w)
{
	static const char oid[] = MBEDTLS_OID_SERVER_AUTH;
	mbedtls_asn1_sequence seq;

	memset(&seq, 0, sizeof(seq));
	seq.buf.tag = MBEDTLS_ASN1_OID;
	seq.buf.p = (unsigned char *)(uintptr_t)oid;
	seq.buf.len = sizeof(oid) - 1U;
	seq.next = NULL;
	return mbedtls_x509write_crt_set_ext_key_usage(w, &seq);
}

/* Subject DN from the configured hostname, so the CN matches what is browsed. */
static void default_subject(char *out, size_t cap)
{
	char host[64];

	if (sts_net_cfg_str(CFG_ID_NET_HOSTNAME, host, sizeof(host)) == 0U) {
		(void)web_span_copy(host, sizeof(host), "meridian", 8U);
	}
	(void)snprintf(out, cap, "CN=%s,O=RCL Design,OU=STS1000", host);
}

/*
 * Generate an ECDSA P-256 key and a long-dated self-signed certificate.
 *
 * The validity window is deliberately wide (2026..2046). The box has no
 * trustworthy wall clock until GNSS locks, so a tight window would make the
 * certificate invalid exactly during first boot — which is when the operator
 * needs the web UI to configure the thing. A self-signed leaf is pinned by
 * fingerprint, not chained to a clock, so a wide window is correct here rather
 * than lax.
 */
static int generate_self_signed(void)
{
	mbedtls_pk_context key;
	mbedtls_x509write_cert w;
	mbedtls_mpi serial;
	char subject[96];
	uint8_t serial_raw[16];
	int rc;

	rc = drbg_start();
	if (rc != 0) {
		return rc;
	}

	mbedtls_pk_init(&key);
	mbedtls_x509write_crt_init(&w);
	mbedtls_mpi_init(&serial);

	rc = mbedtls_pk_setup(&key, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY));
	if (rc != 0) {
		goto out;
	}
	rc = mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, mbedtls_pk_ec(key),
				 mbedtls_ctr_drbg_random, &drbg);
	if (rc != 0) {
		goto out;
	}

	default_subject(subject, sizeof(subject));

	mbedtls_x509write_crt_set_version(&w, MBEDTLS_X509_CRT_VERSION_3);
	mbedtls_x509write_crt_set_md_alg(&w, MBEDTLS_MD_SHA256);
	mbedtls_x509write_crt_set_subject_key(&w, &key);
	mbedtls_x509write_crt_set_issuer_key(&w, &key);

	rc = mbedtls_x509write_crt_set_subject_name(&w, subject);
	if (rc != 0) {
		goto out;
	}
	rc = mbedtls_x509write_crt_set_issuer_name(&w, subject);
	if (rc != 0) {
		goto out;
	}
	/* A random serial, so two boards never present the same one. */
	rc = mbedtls_ctr_drbg_random(&drbg, serial_raw, sizeof(serial_raw));
	if (rc != 0) {
		goto out;
	}
	serial_raw[0] &= 0x7FU; /* keep it positive */
	rc = mbedtls_x509write_crt_set_serial_raw(&w, serial_raw,
						  sizeof(serial_raw));
	if (rc != 0) {
		goto out;
	}
	rc = mbedtls_x509write_crt_set_validity(&w, "20260101000000",
						"20460101000000");
	if (rc != 0) {
		goto out;
	}
	rc = mbedtls_x509write_crt_set_basic_constraints(&w, 0, -1);
	if (rc != 0) {
		goto out;
	}
	/*
	 * No subjectKeyIdentifier / authorityKeyIdentifier: mbedTLS declares both
	 * setters only under MBEDTLS_MD_CAN_SHA1, and adding SHA-1 to the image
	 * for two extensions that a pinned self-signed leaf does not need is the
	 * wrong trade. An operator-supplied certificate carries whatever
	 * extensions its CA issued.
	 */
	rc = mbedtls_x509write_crt_set_key_usage(
		&w, MBEDTLS_X509_KU_DIGITAL_SIGNATURE | MBEDTLS_X509_KU_KEY_AGREEMENT);
	if (rc != 0) {
		goto out;
	}
	rc = set_eku_server_auth(&w);
	if (rc != 0) {
		goto out;
	}

	{
		char tmp_crt[STS_CERT_PEM_MAX];
		char tmp_key[STS_CERT_PEM_MAX];

		rc = mbedtls_x509write_crt_pem(&w, (unsigned char *)tmp_crt,
					       sizeof(tmp_crt),
					       mbedtls_ctr_drbg_random, &drbg);
		if (rc != 0) {
			goto out;
		}
		rc = mbedtls_pk_write_key_pem(&key, (unsigned char *)tmp_key,
					      sizeof(tmp_key));
		if (rc != 0) {
			goto out;
		}
		rc = adopt_pair(tmp_crt, strlen(tmp_crt), tmp_key,
				strlen(tmp_key));
		if (rc != 0) {
			goto out;
		}

		/* Persist so the fingerprint survives a reboot. A dead NOR is a
		 * degraded mode, not a failure. */
		st.persisted = false;
		if (file_write_all(STS_CERT_KEY_PATH, tmp_key,
				   strlen(tmp_key)) == 0 &&
		    file_write_all(STS_CERT_CRT_PATH, tmp_crt,
				   strlen(tmp_crt)) == 0) {
			st.persisted = true;
		} else {
			LOG_WRN("self-signed certificate NOT persisted (/lfs "
				"unavailable): the fingerprint changes on every "
				"reboot, so pinning will not hold");
		}
		memset(tmp_key, 0, sizeof(tmp_key));
	}

out:
	mbedtls_mpi_free(&serial);
	mbedtls_x509write_crt_free(&w);
	mbedtls_pk_free(&key);
	if (rc != 0 && rc > -1000) {
		return rc;
	}
	if (rc != 0) {
		LOG_ERR("self-signed generation: -0x%04x", (unsigned int)-rc);
		return -EIO;
	}
	return 0;
}

#endif /* STS_CERT_HAVE_WRITE */

/* ------------------------------------------------------------------------- */
/* public API                                                                */
/* ------------------------------------------------------------------------- */

int sts_cert_init(void)
{
	static bool inited;
	int rc;

	if (!inited) {
		k_mutex_init(&lock);
		inited = true;
	}
	k_mutex_lock(&lock, K_FOREVER);

	memset(&st, 0, sizeof(st));
	st.acme_state = (uint8_t)REST_ACME_DISABLED;
	(void)web_span_copy(st.acme_detail, sizeof(st.acme_detail),
			    "not compiled in", 15U);

	/* 1 + 2: whatever is already on /lfs, operator-supplied or ours. */
	{
		size_t cl = 0U;
		size_t kl = 0U;

		if (file_read_all(STS_CERT_CRT_PATH, crt_pem, sizeof(crt_pem),
				  &cl) == 0 &&
		    file_read_all(STS_CERT_KEY_PATH, key_pem, sizeof(key_pem),
				  &kl) == 0) {
			char c[STS_CERT_PEM_MAX];
			char k[STS_CERT_PEM_MAX];

			memcpy(c, crt_pem, cl + 1U);
			memcpy(k, key_pem, kl + 1U);
			rc = adopt_pair(c, cl, k, kl);
			memset(k, 0, sizeof(k));
			if (rc == 0) {
				st.persisted = true;
				LOG_INF("TLS credential loaded from %s (%s)",
					STS_CERT_CRT_PATH,
					st.self_signed ? "self-signed"
						       : "operator-supplied");
				k_mutex_unlock(&lock);
				return 0;
			}
			LOG_WRN("stored TLS credential unusable (%d); "
				"regenerating",
				rc);
		}
	}

#if STS_CERT_HAVE_WRITE
	rc = generate_self_signed();
	if (rc == 0) {
		LOG_INF("generated self-signed P-256 certificate (%s), sha256 %s",
			st.persisted ? "persisted" : "RAM only", st.fingerprint);
	}
#else
	LOG_ERR("no X.509 write support in this build; HTTPS cannot start");
	rc = -ENOTSUP;
#endif
	k_mutex_unlock(&lock);
	return rc;
}

int sts_cert_reset(void)
{
	int rc = 0;
	int frc;

	k_mutex_lock(&lock, K_FOREVER);

	/*
	 * Order matters. The credential store goes first, so nothing can pick up
	 * the old key for a new TLS session while the files are being unlinked;
	 * then the RAM copies, so the key is not sitting in .bss waiting to be
	 * read out; then the files.
	 */
	(void)tls_credential_delete(STS_WEB_SEC_TAG,
				    TLS_CREDENTIAL_SERVER_CERTIFICATE);
	(void)tls_credential_delete(STS_WEB_SEC_TAG, TLS_CREDENTIAL_PRIVATE_KEY);

	/* The WHOLE buffer, not just the used prefix: a shorter PEM written over
	 * a longer one would otherwise leave the tail of the old key behind. */
	mbedtls_platform_zeroize(key_pem, sizeof(key_pem));
	mbedtls_platform_zeroize(crt_pem, sizeof(crt_pem));
	key_pem_len = 0U;
	crt_pem_len = 0U;

	memset(&st, 0, sizeof(st));
	st.acme_state = (uint8_t)REST_ACME_DISABLED;

	if (sts_fs_ready()) {
		/* -ENOENT is the desired end state, not a failure. */
		frc = fs_unlink(STS_CERT_KEY_PATH);
		if (frc != 0 && frc != -ENOENT) {
			LOG_ERR("unlink %s: %d", STS_CERT_KEY_PATH, frc);
			rc = -EIO;
		}
		frc = fs_unlink(STS_CERT_CRT_PATH);
		if (frc != 0 && frc != -ENOENT) {
			LOG_ERR("unlink %s: %d", STS_CERT_CRT_PATH, frc);
			rc = -EIO;
		}
		/* The ACME account key identifies this box to a CA, so it is
		 * key material like any other even when ACME is compiled out. */
		frc = fs_unlink(STS_CERT_ACME_KEY_PATH);
		if (frc != 0 && frc != -ENOENT) {
			LOG_ERR("unlink %s: %d", STS_CERT_ACME_KEY_PATH, frc);
			rc = -EIO;
		}
	}
	/*
	 * A dead /lfs is NOT reported as a failure: there is then nothing
	 * persisted to erase, and the RAM copy and the credential store — the
	 * only places the key could still be — have already been cleared.
	 */

	k_mutex_unlock(&lock);

	LOG_WRN("TLS server credential erased; HTTPS has no certificate until "
		"the next boot regenerates one");
	return rc;
}

int sts_cert_info(rest_cert_t *out)
{
	if (out == NULL) {
		return -EINVAL;
	}
	memset(out, 0, sizeof(*out));
	k_mutex_lock(&lock, K_FOREVER);
	out->present = st.ready;
	out->self_signed = st.self_signed;
	out->operator_supplied = st.operator_supplied;
	out->persisted = st.persisted;
	(void)web_span_copy(out->subject, sizeof(out->subject), st.subject,
			    strlen(st.subject));
	(void)web_span_copy(out->issuer, sizeof(out->issuer), st.issuer,
			    strlen(st.issuer));
	(void)web_span_copy(out->not_before, sizeof(out->not_before),
			    st.not_before, strlen(st.not_before));
	(void)web_span_copy(out->not_after, sizeof(out->not_after), st.not_after,
			    strlen(st.not_after));
	(void)web_span_copy(out->sha256_fp, sizeof(out->sha256_fp),
			    st.fingerprint, strlen(st.fingerprint));
	(void)web_span_copy(out->key_type, sizeof(out->key_type), st.key_type,
			    strlen(st.key_type));
	out->key_bits = st.key_bits;
	out->acme_state = st.acme_state;
	(void)web_span_copy(out->acme_detail, sizeof(out->acme_detail),
			    st.acme_detail, strlen(st.acme_detail));
	k_mutex_unlock(&lock);
	return 0;
}

/* Find the first PEM block of @p label in (@p pem, @p len). */
static const char *pem_find(const char *pem, size_t len, const char *label,
			    size_t *out_len)
{
	char begin[48];
	char end[48];
	const char *b;
	const char *e;
	size_t bn;
	size_t en;
	size_t i;

	(void)snprintf(begin, sizeof(begin), "-----BEGIN %s-----", label);
	(void)snprintf(end, sizeof(end), "-----END %s-----", label);
	bn = strlen(begin);
	en = strlen(end);

	b = NULL;
	for (i = 0U; (i + bn) <= len; i++) {
		if (memcmp(&pem[i], begin, bn) == 0) {
			b = &pem[i];
			break;
		}
	}
	if (b == NULL) {
		return NULL;
	}
	e = NULL;
	for (i = (size_t)(b - pem) + bn; (i + en) <= len; i++) {
		if (memcmp(&pem[i], end, en) == 0) {
			e = &pem[i] + en;
			break;
		}
	}
	if (e == NULL) {
		return NULL;
	}
	/* Include a trailing newline if the caller supplied one. */
	if ((size_t)(e - pem) < len && (*e == '\n' || *e == '\r')) {
		e++;
	}
	*out_len = (size_t)(e - b);
	return b;
}

int sts_cert_install(const char *pem, size_t len)
{
	static const char *const key_labels[] = {
		"PRIVATE KEY", "EC PRIVATE KEY", "RSA PRIVATE KEY",
	};
	const char *c;
	const char *k = NULL;
	size_t cl = 0U;
	size_t kl = 0U;
	size_t i;
	int rc;

	if (pem == NULL || len == 0U) {
		return -EBADMSG;
	}

	c = pem_find(pem, len, "CERTIFICATE", &cl);
	if (c == NULL) {
		return -EBADMSG;
	}
	for (i = 0U; i < (sizeof(key_labels) / sizeof(key_labels[0])); i++) {
		k = pem_find(pem, len, key_labels[i], &kl);
		if (k != NULL) {
			break;
		}
	}
	if (k == NULL) {
		return -EBADMSG;
	}

	k_mutex_lock(&lock, K_FOREVER);
	{
		/*
		 * Keep the current credential until the new one has been
		 * validated AND written: a half-applied replacement would leave
		 * the box unreachable over HTTPS with no way back in.
		 */
		char prev_crt[STS_CERT_PEM_MAX];
		char prev_key[STS_CERT_PEM_MAX];
		size_t prev_cl = crt_pem_len;
		size_t prev_kl = key_pem_len;
		bool had = st.ready;

		memcpy(prev_crt, crt_pem, crt_pem_len + 1U);
		memcpy(prev_key, key_pem, key_pem_len + 1U);

		rc = adopt_pair(c, cl, k, kl);
		if (rc != 0) {
			if (had) {
				(void)adopt_pair(prev_crt, prev_cl, prev_key,
						 prev_kl);
			}
			memset(prev_key, 0, sizeof(prev_key));
			k_mutex_unlock(&lock);
			return rc;
		}

		st.persisted = false;
		if (file_write_all(STS_CERT_KEY_PATH, key_pem, key_pem_len) == 0 &&
		    file_write_all(STS_CERT_CRT_PATH, crt_pem, crt_pem_len) == 0) {
			st.persisted = true;
		} else {
			LOG_WRN("operator certificate accepted but NOT persisted");
			rc = -EROFS;
		}
		memset(prev_key, 0, sizeof(prev_key));
	}
	k_mutex_unlock(&lock);

	if (rc == 0) {
		LOG_INF("operator TLS certificate installed, sha256 %s",
			st.fingerprint);
	}
	return rc;
}

int sts_cert_csr(const char *subject, char *out, size_t cap, size_t *out_len)
{
#if STS_CERT_HAVE_WRITE
	mbedtls_pk_context key;
	mbedtls_x509write_csr w;
	char subj[96];
	int rc;

	if (out == NULL || cap == 0U || out_len == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&lock, K_FOREVER);
	if (!st.ready) {
		k_mutex_unlock(&lock);
		return -ENOENT;
	}
	rc = drbg_start();
	if (rc != 0) {
		k_mutex_unlock(&lock);
		return rc;
	}

	mbedtls_pk_init(&key);
	mbedtls_x509write_csr_init(&w);

	rc = mbedtls_pk_parse_key(&key, (const unsigned char *)key_pem,
				  key_pem_len + 1U, NULL, 0U,
				  mbedtls_ctr_drbg_random, &drbg);
	if (rc != 0) {
		rc = -EIO;
		goto out;
	}

	if (subject != NULL && subject[0] != '\0') {
		(void)web_span_copy(subj, sizeof(subj), subject,
				    strlen(subject));
	} else {
		default_subject(subj, sizeof(subj));
	}

	mbedtls_x509write_csr_set_md_alg(&w, MBEDTLS_MD_SHA256);
	mbedtls_x509write_csr_set_key(&w, &key);
	rc = mbedtls_x509write_csr_set_subject_name(&w, subj);
	if (rc != 0) {
		rc = -EINVAL;
		goto out;
	}
	rc = mbedtls_x509write_csr_set_key_usage(
		&w, MBEDTLS_X509_KU_DIGITAL_SIGNATURE |
			    MBEDTLS_X509_KU_KEY_AGREEMENT);
	if (rc != 0) {
		rc = -EIO;
		goto out;
	}

	rc = mbedtls_x509write_csr_pem(&w, (unsigned char *)out, cap,
				       mbedtls_ctr_drbg_random, &drbg);
	if (rc != 0) {
		rc = -ENOSPC;
		goto out;
	}
	*out_len = strlen(out);
	rc = 0;

out:
	mbedtls_x509write_csr_free(&w);
	mbedtls_pk_free(&key);
	k_mutex_unlock(&lock);
	return rc;
#else
	ARG_UNUSED(subject);
	ARG_UNUSED(out);
	ARG_UNUSED(cap);
	ARG_UNUSED(out_len);
	return -ENOTSUP;
#endif
}

/* ========================================================================= */
/* ACME (RFC 8555) — SKELETON, COMPILED OUT                                  */
/* ========================================================================= */
/*
 * CONFIG_STS1000_ACME does not exist in any Kconfig, so none of this is built
 * and it cannot affect the image. It is here so the shape of the work is on
 * record rather than re-invented later.
 *
 * WHAT IS SKETCHED BELOW
 *   - the account-key lifecycle (generate once, persist to
 *     STS_CERT_ACME_KEY_PATH, reuse);
 *   - the order state machine's states and the transitions between them;
 *   - the HTTP-01 challenge hand-off: the token/keyAuthorization pair that the
 *     port-80 listener in sts_web.c would have to serve under
 *     /.well-known/acme-challenge/<token>.
 *
 * WHAT IS NOT IMPLEMENTED, AND WHY EACH ONE IS REAL WORK
 *   1. An HTTPS *client*. Every ACME step is a POST to the CA. Zephyr's TLS
 *      sockets can do it, but it needs a CA trust store on the device, a clock
 *      good enough to validate the CA's certificate (chicken-and-egg with GNSS
 *      lock at first boot), and DNS.
 *   2. JWS signing (RFC 7515): protected header with alg/nonce/url/jwk-or-kid,
 *      flattened JSON, base64url (NOT the base64 in core/web/web.c — the
 *      alphabet and padding differ), and ES256 over SHA-256.
 *   3. A JSON *parser* for CA replies. core/web's reader is a bounded scanner
 *      sized for small request bodies; ACME directory and order objects are
 *      larger and nested deeper than WEB_JSON_READ_DEPTH_MAX.
 *   4. Replay-nonce tracking, Retry-After honouring, and exponential backoff
 *      against the CA's rate limits.
 *   5. The thumbprint calculation for keyAuthorization (RFC 8555 §8.1:
 *      base64url(SHA-256(canonical JWK))).
 *   6. Renewal scheduling and an atomic swap of the live credential, including
 *      what happens to open TLS sessions.
 *   7. Certificate-chain storage: an ACME certificate arrives as a chain, so
 *      TLS_CREDENTIAL_SERVER_CERTIFICATE must hold the leaf plus intermediates,
 *      which the single-file layout above does not model.
 *
 * Until those land, the operator paths that DO work are: upload a PEM pair, or
 * generate a CSR here and install the CA's answer.
 */
#if defined(CONFIG_STS1000_ACME)

typedef enum {
	ACME_IDLE = 0,
	ACME_NEED_ACCOUNT,
	ACME_NEW_ORDER,
	ACME_AUTHZ_PENDING,
	ACME_CHALLENGE_SERVED,
	ACME_FINALIZE,
	ACME_DOWNLOAD,
	ACME_DONE,
	ACME_FAILED,
} acme_state_t;

/** HTTP-01 challenge material the port-80 listener would serve. */
typedef struct {
	char token[64];
	char key_authorization[128];
	bool active;
} acme_challenge_t;

static acme_challenge_t acme_challenge;
static uint8_t acme_state = (uint8_t)ACME_IDLE;

/** Load or create the ACME account key. This part IS implementable today. */
static int acme_account_key(mbedtls_pk_context *out)
{
	char pem[STS_CERT_PEM_MAX];
	size_t n = 0U;
	int rc;

	mbedtls_pk_init(out);
	if (file_read_all(STS_CERT_ACME_KEY_PATH, pem, sizeof(pem), &n) == 0) {
		rc = mbedtls_pk_parse_key(out, (const unsigned char *)pem, n + 1U,
					  NULL, 0U, mbedtls_ctr_drbg_random,
					  &drbg);
		if (rc == 0) {
			return 0;
		}
	}
	rc = drbg_start();
	if (rc != 0) {
		return rc;
	}
	rc = mbedtls_pk_setup(out, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY));
	if (rc != 0) {
		return -EIO;
	}
	rc = mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, mbedtls_pk_ec(*out),
				 mbedtls_ctr_drbg_random, &drbg);
	if (rc != 0) {
		return -EIO;
	}
	if (mbedtls_pk_write_key_pem(out, (unsigned char *)pem,
				     sizeof(pem)) == 0) {
		(void)file_write_all(STS_CERT_ACME_KEY_PATH, pem, strlen(pem));
	}
	memset(pem, 0, sizeof(pem));
	return 0;
}

/**
 * Answer an HTTP-01 challenge lookup from the port-80 listener.
 *
 * This is the one ACME hook sts_web.c needs, and it is complete: everything
 * upstream of it (getting a token from the CA) is the unimplemented part.
 */
bool sts_cert_acme_challenge(const char *token, size_t token_len,
			     const char **out_body, size_t *out_len)
{
	if (!acme_challenge.active || token == NULL) {
		return false;
	}
	if (strlen(acme_challenge.token) != token_len ||
	    memcmp(acme_challenge.token, token, token_len) != 0) {
		return false;
	}
	*out_body = acme_challenge.key_authorization;
	*out_len = strlen(acme_challenge.key_authorization);
	return true;
}

/** One step of the order state machine. TODO: everything marked above. */
int sts_cert_acme_step(void)
{
	switch (acme_state) {
	case ACME_IDLE:
	case ACME_NEED_ACCOUNT:
	case ACME_NEW_ORDER:
	case ACME_AUTHZ_PENDING:
	case ACME_CHALLENGE_SERVED:
	case ACME_FINALIZE:
	case ACME_DOWNLOAD:
		/* Needs the HTTPS client, JWS signing and the JSON parser listed
		 * in the comment block above. Deliberately does nothing rather
		 * than pretending to make progress. */
		return -ENOSYS;
	default:
		return 0;
	}
}

#endif /* CONFIG_STS1000_ACME */
