/*
 * STS1000 "Meridian" — core/fwupd: the multi-IC update state machine.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See fwupd.h for the design and the guard model. The one invariant worth
 * restating at the top of the implementation:
 *
 *   **restore() runs exactly once for every prepare() that succeeded, on every
 *   exit path.**
 *
 * Everything that ends a session goes through finish(), and finish() is the only
 * place FAILED or DONE is entered. That is what keeps a GNSS receiver from being
 * left in safeboot by an error path somebody forgot.
 */

#include "fwupd/fwupd.h"

#include <errno.h>
#include <string.h>

/* ---------------------------------------------------------- descriptors --- */

/*
 * The board's firmware/identity-bearing ICs. Designators are from
 * docs/sts1000_schematic_implementation_checklist.md and the INA228 rail table
 * in the root CLAUDE.md.
 *
 * `version_how` is deliberately prose rather than a code: it goes straight into
 * the maintenance tool's inventory view, and "updatable: no" is only a useful
 * answer when it comes with what *can* be read instead.
 */
static const fwupd_comp_desc_t comp_tbl[FWUPD_COMP__COUNT] = {
	[FWUPD_COMP_STM32_APP] = {
		.comp = (uint8_t)FWUPD_COMP_STM32_APP,
		.name = "STM32H563 application",
		.designator = "U12",
		.version_how = "MCUboot image header of the running slot",
		.updatable = true,
		.xport = (uint8_t)FWUPD_XPORT_INTERNAL,
		.count = 1U,
	},
	[FWUPD_COMP_GNSS_ZED_F9T] = {
		.comp = (uint8_t)FWUPD_COMP_GNSS_ZED_F9T,
		.name = "u-blox ZED-F9T-00B",
		.designator = "U20",
		.version_how = "UBX-MON-VER swVersion + FWVER extension",
		.updatable = true,
		.xport = (uint8_t)FWUPD_XPORT_UART_UBX,
		.count = 1U,
	},
	[FWUPD_COMP_RB_FE5680A] = {
		.comp = (uint8_t)FWUPD_COMP_RB_FE5680A,
		.name = "FE-5680A rubidium",
		.designator = "J6",
		.version_how = "UART7 identity/telemetry probe; varies by variant",
		/*
		 * True at the descriptor level because the *board* provides the
		 * path. Whether the fitted variant has a loader is a runtime
		 * question that rb_fwupd answers, and a variant that has none is
		 * refused at fwupd_begin() with -ENOTSUP.
		 */
		.updatable = true,
		.xport = (uint8_t)FWUPD_XPORT_UART_RB,
		.count = 1U,
	},

	/* ---- read-only from here down ------------------------------------- */
	[FWUPD_COMP_PHY_LAN8742] = {
		.comp = (uint8_t)FWUPD_COMP_PHY_LAN8742,
		.name = "LAN8742AI Ethernet PHY",
		.designator = "U13",
		.version_how = "MDIO PHYID1/PHYID2 (mask-ROM silicon revision)",
		.updatable = false,
		.xport = (uint8_t)FWUPD_XPORT_MDIO,
		.count = 1U,
	},
	[FWUPD_COMP_SE_ATECC608B] = {
		.comp = (uint8_t)FWUPD_COMP_SE_ATECC608B,
		.name = "ATECC608B secure element",
		.designator = "U60",
		.version_how = "Info command RevNum; firmware is mask ROM",
		.updatable = false,
		.xport = (uint8_t)FWUPD_XPORT_I2C,
		.count = 1U,
	},
	[FWUPD_COMP_INA228] = {
		.comp = (uint8_t)FWUPD_COMP_INA228,
		.name = "INA228 power monitors",
		.designator = "U10/U31/U32/U30/U26/U37/U44/U23/U54",
		.version_how = "MANUFACTURER_ID 0x54I + DEVICE_ID die/revision",
		.updatable = false,
		.xport = (uint8_t)FWUPD_XPORT_I2C,
		.count = 9U,
	},
	[FWUPD_COMP_TMP117] = {
		.comp = (uint8_t)FWUPD_COMP_TMP117,
		.name = "TMP117 temperature sensors",
		.designator = "U58/U57",
		.version_how = "DEVICE_ID register (0x0117), revision field",
		.updatable = false,
		.xport = (uint8_t)FWUPD_XPORT_I2C,
		.count = 2U,
	},
	[FWUPD_COMP_DISPLAY_ST7796] = {
		.comp = (uint8_t)FWUPD_COMP_DISPLAY_ST7796,
		.name = "ST7796 display controller",
		.designator = "DS1",
		.version_how = "RDDID (0x04) manufacturer/version/driver ID",
		.updatable = false,
		.xport = (uint8_t)FWUPD_XPORT_SPI,
		.count = 1U,
	},
	[FWUPD_COMP_TOUCH_FT6336U] = {
		.comp = (uint8_t)FWUPD_COMP_TOUCH_FT6336U,
		.name = "FT6336U touch controller",
		.designator = "DS1",
		/*
		 * The FT6x36 family does have a vendor firmware-download mode,
		 * but this board exposes no path to it: the controller sits
		 * behind the PCA9306 translator on the gated display rail, and
		 * u-blox-style bootstrapping is not available. Read-only here.
		 */
		.version_how = "FIRMID (0xA6) + CHIPID (0xA3); no field-update path",
		.updatable = false,
		.xport = (uint8_t)FWUPD_XPORT_I2C,
		.count = 1U,
	},
	[FWUPD_COMP_DIGIPOT_MCP41U83] = {
		.comp = (uint8_t)FWUPD_COMP_DIGIPOT_MCP41U83,
		.name = "MCP41U83 digipot",
		.designator = "U43",
		.version_how = "no ID register; presence via wiper readback",
		.updatable = false,
		.xport = (uint8_t)FWUPD_XPORT_SPI,
		.count = 1U,
	},
	[FWUPD_COMP_PD_NCP1095] = {
		.comp = (uint8_t)FWUPD_COMP_PD_NCP1095,
		.name = "NCP1095 PoE PD controller",
		.designator = "U9",
		/*
		 * Hard-wired analogue part: no digital interface of any kind.
		 * Its "identity" is the classification result the board observes.
		 */
		.version_how = "none: analogue PD, state inferred from PG/class pins",
		.updatable = false,
		.xport = (uint8_t)FWUPD_XPORT_NONE,
		.count = 1U,
	},
};

const fwupd_comp_desc_t *fwupd_comp_desc(uint8_t comp)
{
	if ((unsigned int)comp >= (unsigned int)FWUPD_COMP__COUNT) {
		return NULL;
	}
	return &comp_tbl[comp];
}

const char *fwupd_comp_name(uint8_t comp)
{
	const fwupd_comp_desc_t *d = fwupd_comp_desc(comp);

	return (d != NULL) ? d->name : "?";
}

const char *fwupd_state_name(uint8_t st)
{
	switch (st) {
	case (uint8_t)FWUPD_ST_IDLE:
		return "IDLE";
	case (uint8_t)FWUPD_ST_QUERY:
		return "QUERY";
	case (uint8_t)FWUPD_ST_PREPARE:
		return "PREPARE";
	case (uint8_t)FWUPD_ST_TRANSFER:
		return "TRANSFER";
	case (uint8_t)FWUPD_ST_VERIFY:
		return "VERIFY";
	case (uint8_t)FWUPD_ST_RESTORE:
		return "RESTORE";
	case (uint8_t)FWUPD_ST_DONE:
		return "DONE";
	case (uint8_t)FWUPD_ST_FAILED:
		return "FAILED";
	default:
		return "?";
	}
}

const char *fwupd_end_name(uint8_t reason)
{
	switch (reason) {
	case (uint8_t)FWUPD_END_NONE:
		return "none";
	case (uint8_t)FWUPD_END_OK:
		return "ok";
	case (uint8_t)FWUPD_END_ABORTED:
		return "aborted";
	case (uint8_t)FWUPD_END_TIMEOUT:
		return "timeout";
	case (uint8_t)FWUPD_END_HASH:
		return "hash-mismatch";
	case (uint8_t)FWUPD_END_SHORT:
		return "short-image";
	case (uint8_t)FWUPD_END_TARGET_ERROR:
		return "target-error";
	case (uint8_t)FWUPD_END_VERIFY:
		return "version-not-confirmed";
	case (uint8_t)FWUPD_END_READBACK:
		return "readback-mismatch";
	default:
		return "?";
	}
}

/* ---------------------------------------------------------------- config -- */

void fwupd_cfg_defaults(fwupd_cfg_t *cfg)
{
	if (cfg == NULL) {
		return;
	}
	(void)memset(cfg, 0, sizeof(*cfg));
	/*
	 * Nothing allowed. Every one of these operations can brick a soldered-down
	 * part, so the permissive default is the wrong one even for a development
	 * build: an operator has to name the component.
	 */
	cfg->allow = 0U;
	cfg->step_timeout_ms = 30000U;
	cfg->transfer_idle_timeout_ms = 60000U;
}

int fwupd_cfg_allow(fwupd_cfg_t *cfg, uint8_t comp, bool on)
{
	if ((cfg == NULL) ||
	    ((unsigned int)comp >= (unsigned int)FWUPD_COMP__COUNT)) {
		return -EINVAL;
	}
	if (on) {
		cfg->allow |= ((uint32_t)1U << comp);
	} else {
		cfg->allow &= ~((uint32_t)1U << comp);
	}
	return 0;
}

/* ------------------------------------------------------------- internals -- */

static bool session_running(const fwupd_ctx_t *c)
{
	return (c->state != (uint8_t)FWUPD_ST_IDLE) &&
	       (c->state != (uint8_t)FWUPD_ST_DONE) &&
	       (c->state != (uint8_t)FWUPD_ST_FAILED);
}

static void audit(fwupd_ctx_t *c, const char *what, int rc)
{
	if (c->cb.audit != NULL) {
		c->cb.audit(c->cb.user, c->comp, what, rc);
	}
}

static void emit(fwupd_ctx_t *c)
{
	fwupd_event_t e;

	if (c->cb.event == NULL) {
		return;
	}
	(void)fwupd_progress(c, &e);
	c->cb.event(c->cb.user, &e);
}

static void set_state(fwupd_ctx_t *c, fwupd_state_t st, uint64_t now_ms)
{
	c->state = (uint8_t)st;
	c->step_started_ms = now_ms;
	audit(c, fwupd_state_name((uint8_t)st), c->last_rc);
	emit(c);
}

static void set_degraded(fwupd_ctx_t *c, bool on)
{
	if (c->degraded == on) {
		return;
	}
	c->degraded = on;
	if (c->cb.degraded != NULL) {
		c->cb.degraded(c->cb.user, c->comp, on);
	}
}

static const fwupd_target_ops_t *target(const fwupd_ctx_t *c, uint8_t comp)
{
	if ((unsigned int)comp >= (unsigned int)FWUPD_COMP__COUNT) {
		return NULL;
	}
	return c->target_set[comp] ? &c->target[comp] : NULL;
}

static void copy_str(char *dst, size_t cap, const char *src)
{
	size_t n;

	if (cap == 0U) {
		return;
	}
	if (src == NULL) {
		dst[0] = '\0';
		return;
	}
	n = strlen(src);
	if (n >= cap) {
		n = cap - 1U;
	}
	(void)memcpy(dst, src, n);
	dst[n] = '\0';
}

/*
 * ---------------------------------------------------------------------------
 * The read-back check
 * ---------------------------------------------------------------------------
 *
 * Pull everything written but not yet read back out of the component and fold
 * it into the second SHA-256. A no-op for a target with no readback callback,
 * which is how the GNSS receiver and the rubidium keep their previous
 * behaviour exactly (fwupd.h).
 *
 * COST, since this runs on the console RX thread inside the Maintenance
 * Protocol's engine lock, which sts_console.c's dead-man BUILD_ASSERT budgets:
 *
 *   per fwupd_data() call, at most FWUPD_CHUNK_MAX (1024) octets are read and
 *   hashed, in ceil(1024 / FWUPD_READBACK_CHUNK) = 4 reads. On the STM32 target
 *   a read is a memory-mapped flash_area_read() behind sts_dfu.c's `dfu_lock`,
 *   and the hash is mbedTLS SHA-256 — order 1e5 cycles at 250 MHz, i.e. tens of
 *   microseconds, against a call that already programs the same 1024 octets
 *   into flash. Four extra `dfu_lock` acquisitions per chunk, each held for a
 *   memcpy;
 *
 *   over a full 896 KB slot that totals ~3.6k reads and one SHA-256 pass over
 *   the image — a few hundred milliseconds of CPU on this part — but spread
 *   across the ~900 fw.data calls that carried it, never held in one place.
 *   That is the whole reason it is here and not in a single sweep at
 *   fwupd_end(): a sweep would hold the engine lock for that entire figure at
 *   once, and STS_MP_TICK_LOCK_MS/STS_MP_TICK_MISS_MAX give the override
 *   dead-man only ~1.5 s of lock hold before its revert deadline is at risk.
 *
 * Returns 0, or the component's errno — which ends the session as a target
 * error, NOT as a mismatch: "the flash would not answer" and "the flash
 * answered wrong" are different faults.
 */
static int readback_advance(fwupd_ctx_t *c, uint32_t upto)
{
	const fwupd_target_ops_t *t = target(c, c->comp);
	int rc;

	if (!c->rb_active) {
		return 0;
	}
	if ((t == NULL) || (t->readback == NULL)) {
		/*
		 * Unreachable while a session is open — rb_active is only set
		 * for a target that had the callback, and fwupd_set_target()
		 * refuses to replace one mid-session. Refusing rather than
		 * silently skipping, because the alternative is a session that
		 * reports a read-back it did not perform.
		 */
		return -ENODEV;
	}

	while (c->rb_off < upto) {
		uint32_t n = upto - c->rb_off;

		if (n > (uint32_t)sizeof(c->rb_buf)) {
			n = (uint32_t)sizeof(c->rb_buf);
		}
		rc = t->readback(t->user, c->rb_off, c->rb_buf, (size_t)n);
		if (rc != 0) {
			return rc;
		}
		rc = c->sha.update(c->sha.ctx, c->rb_sha_state, c->rb_buf,
				   (size_t)n);
		if (rc != 0) {
			return rc;
		}
		c->rb_off += n;
	}
	return 0;
}

/*
 * The single exit. Runs RESTORE if it is owed, clears the degraded flag, and
 * enters DONE or FAILED.
 *
 * restore() is called here and nowhere else, which is what makes "restore runs
 * on every exit path" a property of the code rather than a habit. A restore that
 * itself fails cannot be recovered from in software — the component is left in
 * whatever state it is in — so it downgrades a success to a failure and is
 * audited loudly.
 */
static void finish(fwupd_ctx_t *c, fwupd_end_t reason, int rc, uint64_t now_ms)
{
	const fwupd_target_ops_t *t = target(c, c->comp);
	bool ok = (reason == FWUPD_END_OK);

	if (c->prepared && (t != NULL) && (t->restore != NULL)) {
		int rrc;

		c->state = (uint8_t)FWUPD_ST_RESTORE;
		emit(c);
		rrc = t->restore(t->user, !ok);
		audit(c, "restore", rrc);
		if (rrc != 0) {
			/*
			 * The component may still be held in whatever
			 * programming state prepare() put it in. Nothing further
			 * can be done from here, but the session must not report
			 * success: a GNSS receiver that did not come out of
			 * safeboot is not a completed update.
			 */
			ok = false;
			if (reason == FWUPD_END_OK) {
				reason = FWUPD_END_TARGET_ERROR;
			}
			if (rc == 0) {
				rc = rrc;
			}
		}
	}
	c->prepared = false;
	set_degraded(c, false);
	c->sha_active = false;
	c->rb_active = false;

	c->reason = (uint8_t)reason;
	c->last_rc = rc;
	c->state = ok ? (uint8_t)FWUPD_ST_DONE : (uint8_t)FWUPD_ST_FAILED;
	if (ok) {
		c->sessions_ok++;
	} else {
		c->sessions_failed++;
	}
	c->step_started_ms = now_ms;
	audit(c, ok ? "done" : "failed", rc);
	emit(c);
}

/* ------------------------------------------------------------ lifecycle --- */

int fwupd_init(fwupd_ctx_t *c, const fwupd_cfg_t *cfg, const fwupd_cb_t *cb,
	       const port_sha256_stream_t *sha)
{
	if ((c == NULL) || (cfg == NULL) || (sha == NULL) ||
	    (sha->init == NULL) || (sha->update == NULL) || (sha->final == NULL)) {
		return -EINVAL;
	}

	(void)memset(c, 0, sizeof(*c));
	c->cfg = *cfg;
	c->sha = *sha;
	if (cb != NULL) {
		c->cb = *cb;
	}
	c->state = (uint8_t)FWUPD_ST_IDLE;
	return 0;
}

int fwupd_set_target(fwupd_ctx_t *c, uint8_t comp, const fwupd_target_ops_t *ops)
{
	if ((c == NULL) ||
	    ((unsigned int)comp >= (unsigned int)FWUPD_COMP__COUNT)) {
		return -EINVAL;
	}
	if (session_running(c)) {
		return -EBUSY;
	}
	if (ops == NULL) {
		(void)memset(&c->target[comp], 0, sizeof(c->target[comp]));
		c->target_set[comp] = false;
		return 0;
	}
	if (ops->query_version == NULL) {
		/* Without an identity read a component cannot even be inventoried. */
		return -EINVAL;
	}
	c->target[comp] = *ops;
	c->target_set[comp] = true;
	return 0;
}

int fwupd_set_cfg(fwupd_ctx_t *c, const fwupd_cfg_t *cfg)
{
	if ((c == NULL) || (cfg == NULL)) {
		return -EINVAL;
	}
	if (session_running(c)) {
		return -EBUSY;
	}
	c->cfg = *cfg;
	return 0;
}

/* ------------------------------------------------------------ inventory --- */

int fwupd_query(fwupd_ctx_t *c, uint8_t comp, char *out, size_t cap)
{
	const fwupd_target_ops_t *t;
	int rc;

	if ((c == NULL) || (out == NULL) || (cap == 0U)) {
		return -EINVAL;
	}
	if ((unsigned int)comp >= (unsigned int)FWUPD_COMP__COUNT) {
		return -EINVAL;
	}
	out[0] = '\0';

	t = target(c, comp);
	if (t == NULL) {
		return -ENODEV;
	}
	rc = t->query_version(t->user, out, cap);
	if (rc != 0) {
		out[0] = '\0';
	}
	return rc;
}

int fwupd_inventory(fwupd_ctx_t *c, fwupd_inv_row_t *out, size_t max, size_t *n)
{
	size_t written = 0U;
	unsigned int i;

	if ((c == NULL) || (out == NULL) || (n == NULL)) {
		return -EINVAL;
	}
	*n = 0U;

	for (i = 0U; i < (unsigned int)FWUPD_COMP__COUNT; i++) {
		fwupd_inv_row_t *r;

		if (written >= max) {
			break;
		}
		r = &out[written];
		(void)memset(r, 0, sizeof(*r));
		r->desc = &comp_tbl[i];

		r->last_rc = fwupd_query(c, (uint8_t)i, r->version,
					 sizeof(r->version));
		if (r->last_rc == 0) {
			r->version_valid = (r->version[0] != '\0');
			/*
			 * A driver that answered at all is evidence the part is
			 * there. NCP1095 has no interface, so its driver reports
			 * an empty string with rc 0 — present, but nothing to
			 * read, which is a different statement from absent.
			 */
			r->present = true;
		}
		written++;
	}

	*n = written;
	return (written < (size_t)FWUPD_COMP__COUNT) ? -ENOSPC : 0;
}

/* -------------------------------------------------------------- session --- */

int fwupd_begin(fwupd_ctx_t *c, uint8_t comp, const fwupd_req_t *req,
		uint64_t now_ms)
{
	const fwupd_target_ops_t *t;
	int rc;

	if ((c == NULL) || (req == NULL)) {
		return -EINVAL;
	}
	if ((unsigned int)comp >= (unsigned int)FWUPD_COMP__COUNT) {
		return -EINVAL;
	}
	if (session_running(c)) {
		return -EBUSY;
	}

	/* Guard 1: the magic. A single corrupted opcode must not start this. */
	if (req->magic != (uint32_t)FWUPD_MAGIC) {
		c->comp = comp;
		audit(c, "reject-magic", -EINVAL);
		return -EINVAL;
	}
	if ((req->size == 0U) || (req->size > (uint32_t)FWUPD_IMAGE_MAX)) {
		c->comp = comp;
		audit(c, "reject-size", -EINVAL);
		return -EINVAL;
	}

	t = target(c, comp);
	if (t == NULL) {
		c->comp = comp;
		audit(c, "reject-no-driver", -ENODEV);
		return -ENODEV;
	}

	/* Guard 2: policy. */
	if ((c->cfg.allow & ((uint32_t)1U << comp)) == 0U) {
		c->comp = comp;
		audit(c, "reject-not-allowed", -EACCES);
		return -EACCES;
	}

	/* Guard 3: the component must actually have a programming path. */
	if ((t->prepare == NULL) || (t->transfer == NULL) || (t->verify == NULL)) {
		c->comp = comp;
		audit(c, "reject-read-only", -ENOTSUP);
		return -ENOTSUP;
	}

	/* Committed: set the session up before anything can fail destructively. */
	c->comp = comp;
	c->reason = (uint8_t)FWUPD_END_NONE;
	c->last_rc = 0;
	c->total = req->size;
	c->done = 0U;
	c->prepared = false;
	c->degraded = false;
	c->rb_off = 0U;
	c->rb_active = false;
	c->chunks_duplicate = c->chunks_duplicate; /* lifetime, not per-session */
	(void)memcpy(c->want_sha, req->sha256, sizeof(c->want_sha));
	copy_str(c->expect_version, sizeof(c->expect_version),
		 req->expect_version);
	c->version_before[0] = '\0';
	c->version_after[0] = '\0';
	c->sessions++;

	set_state(c, FWUPD_ST_QUERY, now_ms);

	/*
	 * The pre-update version is recorded but a failure to read it is not
	 * fatal: a receiver that has already been left half-programmed by an
	 * earlier attempt cannot answer UBX-MON-VER, and refusing to re-flash it
	 * would be exactly the wrong response.
	 */
	rc = t->query_version(t->user, c->version_before,
			      sizeof(c->version_before));
	audit(c, "query", rc);
	if (rc != 0) {
		c->version_before[0] = '\0';
	}

	set_state(c, FWUPD_ST_PREPARE, now_ms);
	/* Raised before prepare(), not after: prepare() is what takes the
	 * component away, so the rest of the firmware must already know. */
	set_degraded(c, true);

	rc = t->prepare(t->user, c->total);
	audit(c, "prepare", rc);
	if (rc != 0) {
		/* prepare() failed, so restore() is not owed — but the degraded
		 * flag was raised and finish() clears it. */
		finish(c, FWUPD_END_TARGET_ERROR, rc, now_ms);
		return rc;
	}
	c->prepared = true;

	rc = c->sha.init(c->sha.ctx, c->sha_state);
	if (rc != 0) {
		audit(c, "sha-init", rc);
		finish(c, FWUPD_END_TARGET_ERROR, rc, now_ms);
		return rc;
	}
	c->sha_active = true;

	/*
	 * The second hash, over what the component actually keeps. Armed only
	 * when the target can be read back; everything downstream tests
	 * c->rb_active, so a target without the callback takes no new path at
	 * all.
	 */
	if (t->readback != NULL) {
		rc = c->sha.init(c->sha.ctx, c->rb_sha_state);
		if (rc != 0) {
			audit(c, "sha-init-readback", rc);
			finish(c, FWUPD_END_TARGET_ERROR, rc, now_ms);
			return rc;
		}
		c->rb_active = true;
	}

	if (t->poll != NULL) {
		/* Stay in PREPARE; fwupd_step() advances when poll() says done. */
		c->last_chunk_ms = now_ms;
		return 0;
	}

	c->last_chunk_ms = now_ms;
	set_state(c, FWUPD_ST_TRANSFER, now_ms);
	return 0;
}

int fwupd_data(fwupd_ctx_t *c, uint32_t off, const uint8_t *data, size_t len,
	       uint32_t *next_off, uint64_t now_ms)
{
	const fwupd_target_ops_t *t;
	uint32_t limit = FWUPD_CHUNK_MAX;
	bool last;
	int rc;

	if ((c == NULL) || (data == NULL) || (len == 0U)) {
		if ((c != NULL) && (next_off != NULL)) {
			*next_off = c->done;
		}
		return -EINVAL;
	}
	if (next_off != NULL) {
		*next_off = c->done;
	}
	if (c->state != (uint8_t)FWUPD_ST_TRANSFER) {
		return -EPERM;
	}

	t = target(c, c->comp);
	if (t == NULL) {
		return -EPERM;
	}
	if (t->chunk_max != NULL) {
		uint32_t m = t->chunk_max(t->user);

		if ((m != 0U) && (m < limit)) {
			limit = m;
		}
	}
	if (len > (size_t)limit) {
		c->chunks_rejected++;
		return -EINVAL;
	}

	/*
	 * Duplicate retransmit: the whole chunk lies inside what has already been
	 * written. Accepted as a no-op so a tool that did not see our
	 * acknowledgement can simply resend.
	 */
	if (((uint64_t)off + (uint64_t)len) <= (uint64_t)c->done) {
		c->chunks_duplicate++;
		return 0;
	}
	if (off != c->done) {
		/*
		 * Either a gap or a partial overlap. Both are refused rather
		 * than trimmed: writing only the tail of an overlapping chunk
		 * would leave the streaming hash covering octets that were never
		 * handed to the target, and the SHA-256 check at fwupd_end()
		 * would then be verifying a different image from the one on the
		 * device.
		 */
		c->chunks_rejected++;
		return -EPROTO;
	}
	if (((uint64_t)off + (uint64_t)len) > (uint64_t)c->total) {
		c->chunks_rejected++;
		return -ENOSPC;
	}

	last = (((uint64_t)off + (uint64_t)len) == (uint64_t)c->total);

	rc = t->transfer(t->user, off, data, len, last);
	if (rc != 0) {
		audit(c, "transfer", rc);
		finish(c, FWUPD_END_TARGET_ERROR, rc, now_ms);
		return rc;
	}

	rc = c->sha.update(c->sha.ctx, c->sha_state, data, len);
	if (rc != 0) {
		audit(c, "sha-update", rc);
		finish(c, FWUPD_END_TARGET_ERROR, rc, now_ms);
		return rc;
	}

	c->done += (uint32_t)len;

	/*
	 * Read the octets straight back out of the component before this chunk
	 * is acknowledged, so `next_off` means "the component holds everything
	 * below this" and not just "we handed it over". Bounded by the chunk
	 * that was written; see readback_advance()'s cost note.
	 */
	rc = readback_advance(c, c->done);
	if (rc != 0) {
		audit(c, "readback", rc);
		finish(c, FWUPD_END_TARGET_ERROR, rc, now_ms);
		return rc;
	}

	c->last_chunk_ms = now_ms;
	if (next_off != NULL) {
		*next_off = c->done;
	}
	emit(c);
	return 0;
}

/* VERIFY: read the version back and compare it to what was asked for. */
static void run_verify(fwupd_ctx_t *c, uint64_t now_ms)
{
	const fwupd_target_ops_t *t = target(c, c->comp);
	int rc;

	if ((t == NULL) || (t->verify == NULL)) {
		finish(c, FWUPD_END_TARGET_ERROR, -ENODEV, now_ms);
		return;
	}

	rc = t->verify(t->user, c->version_after, sizeof(c->version_after));
	audit(c, "verify", rc);
	if (rc != 0) {
		c->version_after[0] = '\0';
		finish(c, FWUPD_END_TARGET_ERROR, rc, now_ms);
		return;
	}

	if (c->expect_version[0] != '\0') {
		if (strstr(c->version_after, c->expect_version) == NULL) {
			/*
			 * Bytes went in and the component came back, but it is
			 * not running what was asked for. Reporting success here
			 * would be the single most misleading outcome this module
			 * could produce.
			 */
			audit(c, "verify-mismatch", -EIO);
			finish(c, FWUPD_END_VERIFY, -EIO, now_ms);
			return;
		}
	}

	finish(c, FWUPD_END_OK, 0, now_ms);
}

int fwupd_end(fwupd_ctx_t *c, uint64_t now_ms)
{
	uint8_t got[32];
	int rc;

	if (c == NULL) {
		return -EINVAL;
	}
	if (c->state != (uint8_t)FWUPD_ST_TRANSFER) {
		return -EPERM;
	}

	if (c->done != c->total) {
		audit(c, "short-image", -EBADMSG);
		finish(c, FWUPD_END_SHORT, -EBADMSG, now_ms);
		return -EBADMSG;
	}

	rc = c->sha.final(c->sha.ctx, c->sha_state, got);
	c->sha_active = false;
	if (rc != 0) {
		audit(c, "sha-final", rc);
		finish(c, FWUPD_END_TARGET_ERROR, rc, now_ms);
		return rc;
	}
	if (memcmp(got, c->want_sha, sizeof(got)) != 0) {
		/*
		 * Not constant-time on purpose: this is an integrity check on an
		 * image the operator supplied along with its own hash, not an
		 * authentication decision. The signature check that *is* an
		 * authentication decision belongs to MCUboot for the STM32 image
		 * and to the vendor loader for the peripherals.
		 */
		audit(c, "hash-mismatch", -EBADMSG);
		finish(c, FWUPD_END_HASH, -EBADMSG, now_ms);
		return -EBADMSG;
	}

	/*
	 * The transport is proven. Now prove the component kept it — before
	 * verify(), which is what arms the image (mark_pending() on the STM32
	 * target). Getting this order wrong would stage a slot the flash did
	 * not take and leave the operator to discover it at the next boot, in a
	 * bootloader that logs nothing.
	 */
	if (c->rb_active) {
		uint8_t rb[32];

		/*
		 * Normally nothing left to do: fwupd_data() closes the gap on
		 * every chunk, so rb_off is already `total`. Kept because the
		 * loop, not the bookkeeping, is what makes "every octet was read
		 * back" true — and because a target whose chunks are acked some
		 * other way must still be covered end to end.
		 */
		rc = readback_advance(c, c->total);
		if (rc != 0) {
			audit(c, "readback", rc);
			finish(c, FWUPD_END_TARGET_ERROR, rc, now_ms);
			return rc;
		}

		rc = c->sha.final(c->sha.ctx, c->rb_sha_state, rb);
		c->rb_active = false;
		if (rc != 0) {
			audit(c, "readback-final", rc);
			finish(c, FWUPD_END_TARGET_ERROR, rc, now_ms);
			return rc;
		}

		/*
		 * Against want_sha, the same digest the streaming hash was
		 * checked against, and not against the streaming digest: one
		 * question, one answer. Comparing the two hashes with each other
		 * would let a common-mode failure — a sha port that returns a
		 * constant, say — agree with itself.
		 *
		 * Not constant-time, for the reason given above: an integrity
		 * check on an operator-supplied image, not an authentication
		 * decision.
		 */
		if (memcmp(rb, c->want_sha, sizeof(rb)) != 0) {
			audit(c, "readback-mismatch", -EBADMSG);
			finish(c, FWUPD_END_READBACK, -EBADMSG, now_ms);
			return -EBADMSG;
		}
	}

	set_state(c, FWUPD_ST_VERIFY, now_ms);

	{
		const fwupd_target_ops_t *t = target(c, c->comp);

		if ((t != NULL) && (t->poll != NULL)) {
			/* Let fwupd_step() wait for the component to come back. */
			return 0;
		}
	}

	run_verify(c, now_ms);
	return 0;
}

int fwupd_abort(fwupd_ctx_t *c, uint64_t now_ms)
{
	if (c == NULL) {
		return -EINVAL;
	}
	if (!session_running(c)) {
		return 0;
	}
	audit(c, "abort", 0);
	finish(c, FWUPD_END_ABORTED, -ECANCELED, now_ms);
	return 0;
}

int fwupd_step(fwupd_ctx_t *c, uint64_t now_ms)
{
	const fwupd_target_ops_t *t;
	uint64_t elapsed;
	int rc;

	if (c == NULL) {
		return -EINVAL;
	}
	if (!session_running(c)) {
		return 0;
	}
	t = target(c, c->comp);
	if (t == NULL) {
		finish(c, FWUPD_END_TARGET_ERROR, -ENODEV, now_ms);
		return 0;
	}

	/* Timeouts first: a stalled step must not be able to hold the
	 * component in a programming state indefinitely. */
	if (c->state == (uint8_t)FWUPD_ST_TRANSFER) {
		elapsed = (now_ms > c->last_chunk_ms)
				  ? (now_ms - c->last_chunk_ms) : 0U;
		if ((c->cfg.transfer_idle_timeout_ms != 0U) &&
		    (elapsed >= (uint64_t)c->cfg.transfer_idle_timeout_ms)) {
			audit(c, "transfer-idle-timeout", -ETIMEDOUT);
			finish(c, FWUPD_END_TIMEOUT, -ETIMEDOUT, now_ms);
			return 0;
		}
		return 0;
	}

	elapsed = (now_ms > c->step_started_ms) ? (now_ms - c->step_started_ms)
					        : 0U;
	if ((c->cfg.step_timeout_ms != 0U) &&
	    (elapsed >= (uint64_t)c->cfg.step_timeout_ms)) {
		audit(c, "step-timeout", -ETIMEDOUT);
		finish(c, FWUPD_END_TIMEOUT, -ETIMEDOUT, now_ms);
		return 0;
	}

	if (t->poll == NULL) {
		return 0;
	}

	rc = t->poll(t->user);
	if (rc == -EAGAIN) {
		return 0;
	}
	if (rc != 0) {
		audit(c, "poll", rc);
		finish(c, FWUPD_END_TARGET_ERROR, rc, now_ms);
		return 0;
	}

	switch (c->state) {
	case (uint8_t)FWUPD_ST_QUERY:
	case (uint8_t)FWUPD_ST_PREPARE:
		c->last_chunk_ms = now_ms;
		set_state(c, FWUPD_ST_TRANSFER, now_ms);
		break;
	case (uint8_t)FWUPD_ST_VERIFY:
		run_verify(c, now_ms);
		break;
	default:
		break;
	}
	return 0;
}

int fwupd_reset(fwupd_ctx_t *c)
{
	if (c == NULL) {
		return -EINVAL;
	}
	if (session_running(c)) {
		return -EBUSY;
	}
	c->state = (uint8_t)FWUPD_ST_IDLE;
	c->reason = (uint8_t)FWUPD_END_NONE;
	c->done = 0U;
	c->total = 0U;
	c->last_rc = 0;
	return 0;
}

/* ------------------------------------------------------------ accessors --- */

uint8_t fwupd_state(const fwupd_ctx_t *c)
{
	return (c != NULL) ? c->state : (uint8_t)FWUPD_ST_IDLE;
}

uint8_t fwupd_component(const fwupd_ctx_t *c)
{
	return (c != NULL) ? c->comp : 0U;
}

uint8_t fwupd_end_reason(const fwupd_ctx_t *c)
{
	return (c != NULL) ? c->reason : (uint8_t)FWUPD_END_NONE;
}

int fwupd_progress(const fwupd_ctx_t *c, fwupd_event_t *out)
{
	if ((c == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	(void)memset(out, 0, sizeof(*out));
	out->comp = c->comp;
	out->state = c->state;
	out->reason = c->reason;
	out->done = c->done;
	out->total = c->total;
	out->rc = c->last_rc;
	if (c->total != 0U) {
		uint64_t p = ((uint64_t)c->done * 1000U) / (uint64_t)c->total;

		out->permille = (p > 1000U) ? 1000U : (uint16_t)p;
	}
	return 0;
}

const char *fwupd_version_before(const fwupd_ctx_t *c)
{
	return (c != NULL) ? c->version_before : "";
}

const char *fwupd_version_after(const fwupd_ctx_t *c)
{
	return (c != NULL) ? c->version_after : "";
}
