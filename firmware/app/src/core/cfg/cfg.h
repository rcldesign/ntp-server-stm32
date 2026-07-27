/*
 * STS1000 "Meridian" — core/cfg: typed configuration registry.
 *
 * Platform-neutral C11. No dynamic allocation; the caller owns the context.
 * Implements the config model of spec §5.5 and ARCHITECTURE.md §8:
 *
 *   - a static, versioned schema (cfg_schema.h) of numeric keys 0xGGII;
 *   - a RAM mirror of the *live* typed values plus a *staged* overlay;
 *   - validate-then-commit: cfg_set() only stages, cfg_commit() validates the
 *     whole staged set (types, bounds, and a caller-supplied cross-field hook)
 *     and applies it all-or-nothing;
 *   - persistence through port_store_t, keyed by the numeric key ID;
 *   - TLV export/import with a CRC-32 trailer, schema versioning and a
 *     migration hook table.
 *
 * Threading: a cfg_ctx_t is not internally locked, and it is NOT owned by a
 * single thread — the MCP engine, the Zephyr shell and the local UI all reach
 * the same context. Mutual exclusion is therefore the caller's job and is not
 * optional: every call that takes a non-const cfg_ctx_t *, and every read that
 * must not observe a half-applied commit, has to run inside one. On target that
 * critical section is sts_cfg_lock()/sts_cfg_unlock() (zephyr/sts_app.h); core
 * modules take it through a callback supplied by the glue (see mcp.h "Config
 * locking"). Timing state is different and is never read from here: services
 * use the lock-free quality snapshot.
 *
 * Atomicity, precisely
 * --------------------
 * cfg_commit() is atomic with respect to *validation*: if any staged value
 * fails its type/bounds check, or the cross-field hook rejects the candidate
 * tree, nothing is applied and the staged set is left intact for the operator
 * to correct. It is not atomic with respect to *persistence*: the live values
 * are updated first (that cannot fail) and each dirty key is then written
 * through port_store. A store write failure is counted in
 * cfg_commit_res_t::persist_errors and turns the return value into -EIO, but
 * the live tree keeps the new values — RAM is authoritative until the next
 * boot, and reporting a half-written store as "not applied" would be a lie.
 */

#ifndef STS1000_CORE_CFG_CFG_H_
#define STS1000_CORE_CFG_CFG_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cfg/cfg_schema.h"
#include "port/port_store.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ values */

/**
 * A typed configuration value.
 *
 * @p len is meaningful for CFG_T_STR and CFG_T_BLOB only and counts bytes in
 * @p v.b (a STR is *not* NUL-terminated on the wire or in this struct). For
 * numeric types @p len is 0 and the value lives in the matching union member:
 * BOOL/U8/U16/U32/U64 in @p v.u, I32 in @p v.i, F32 in @p v.f.
 */
typedef struct {
	uint8_t  type;   /* cfg_type_t */
	uint16_t len;    /* STR/BLOB byte count */
	union {
		uint64_t u;
		int32_t  i;
		float    f;
		uint8_t  b[CFG_VAL_MAX];
	} v;
} cfg_val_t;

/** Numeric domain of a schema row: default, minimum and maximum. */
typedef union {
	uint64_t u;
	int32_t  i;
	float    f;
} cfg_num_t;

/** One schema row. The table is generated from CFG_SCHEMA() in cfg_schema.h. */
typedef struct {
	uint16_t    id;
	uint8_t     type;   /* cfg_type_t */
	uint8_t     flags;  /* CFG_F_* */
	uint16_t    maxlen; /* STR/BLOB capacity in bytes; 0 for numerics */
	const char *name;
	cfg_num_t   def;
	cfg_num_t   min;
	cfg_num_t   max;
	const char *sdef;   /* STR default; NULL for every other type */
} cfg_key_t;

/* -------------------------------------------------------------- migrations */

/**
 * Per-record import migration.
 *
 * Registered against the schema version that *wrote* the export. When an
 * import carries `schema_ver == from_ver` and that differs from
 * CFG_SCHEMA_VERSION, @p fn is called for every decoded record before it is
 * staged, and may rewrite the ID and/or the value in place.
 *
 * @retval 0   Keep the (possibly rewritten) record.
 * @retval 1   Drop the record silently — the key no longer exists.
 * @retval <0  Abort the import with this error.
 *
 * Only single-step migration is implemented: an export two schema versions old
 * needs its own row. Chained N-step migration is a documented TODO.
 */
typedef struct {
	uint16_t from_ver;
	int (*fn)(uint16_t *id, cfg_val_t *val, void *user);
} cfg_migration_t;

/* ----------------------------------------------------------------- context */

struct cfg_ctx;

/**
 * Cross-field validation hook, run by cfg_commit() against the candidate tree
 * (staged overlaid on live). Read values with cfg_get_effective(); a non-zero
 * return rejects the whole commit and nothing is applied.
 */
typedef int (*cfg_validate_fn)(const struct cfg_ctx *c, void *user);

/** Registry state. Caller-owned; zeroed and populated by cfg_init(). */
typedef struct cfg_ctx {
	const port_store_t *store;         /* may be NULL: RAM-only registry */

	cfg_val_t live[CFG_KEY_COUNT];
	cfg_val_t stage[CFG_KEY_COUNT];
	uint32_t  staged_bits[(CFG_KEY_COUNT + 31U) / 32U];
	uint16_t  staged_n;

	cfg_validate_fn xvalidate;
	void           *xvalidate_user;

	const cfg_migration_t *migrations;
	size_t                 migrations_n;
	void                  *migrations_user;

	uint32_t load_corrupt;  /* entries defaulted by the last cfg_load_all() */
	uint32_t load_missing;  /* entries absent from the store at last load */
} cfg_ctx_t;

/** Outcome of a cfg_commit(). */
typedef struct {
	uint16_t staged;        /* staged entries examined */
	uint16_t applied;       /* live values that actually changed */
	uint16_t reboot_keys;   /* applied keys carrying CFG_F_REBOOT_REQUIRED */
	uint32_t reboot_groups; /* CFG_GROUP_BIT() per group needing a reboot */
	uint16_t persist_errors;/* keys the store refused */
} cfg_commit_res_t;

/* ------------------------------------------------------------ schema access */

/** Number of rows in the schema. */
size_t cfg_key_count(void);

/** Row @p idx, or NULL when out of range. Rows are sorted by ascending ID. */
const cfg_key_t *cfg_key_at(size_t idx);

/** Row for key @p id, or NULL when the ID is not in the schema. */
const cfg_key_t *cfg_key_find(uint16_t id);

/** Index of key @p id, or -ENOENT. */
int cfg_key_index(uint16_t id);

/** Index of the first key with `id >= start_id`, or -ENOENT past the end. */
int cfg_key_lower_bound(uint16_t start_id);

/** Encoded width of a numeric type in bytes, or 0 for STR/BLOB/unknown. */
size_t cfg_type_width(uint8_t type);

/* --------------------------------------------------------------- lifecycle */

/**
 * Initialise @p c, load every key with its schema default and clear staging.
 * Does not touch @p store — call cfg_load_all() for that.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p c is NULL.
 */
int cfg_init(cfg_ctx_t *c, const port_store_t *store);

/**
 * Read every key from the store into the live tree.
 *
 * A key the store does not hold keeps its default and counts as *missing*, not
 * corrupt. A key whose stored record has the wrong type, the wrong length, or
 * a value outside its schema bounds is reset to the default and counted as
 * *corrupt*. Neither case fails the load: a damaged NVS must still boot.
 *
 * @param out_corrupt  Optional; receives the corrupt-entry count.
 * @retval 0        Success (whatever the counts).
 * @retval -EINVAL  @p c is NULL.
 */
int cfg_load_all(cfg_ctx_t *c, uint32_t *out_corrupt);

/**
 * Install the cross-field validation hook (NULL clears it).
 *
 * cfg_init() clears it, so re-install after every cfg_init() — including the
 * one that swaps a RAM store for the persistent one.
 */
int cfg_set_validate_hook(cfg_ctx_t *c, cfg_validate_fn fn, void *user);

/**
 * The schema's own cross-field rules, in cfg_validate_fn form.
 *
 * Install with cfg_set_validate_hook(c, cfg_schema_xvalidate, NULL). Rules are
 * constraints no single row can express, i.e. combinations that are individually
 * in range but jointly unsafe:
 *
 *   snmp.enable requires a non-empty snmp.community — the community IS the
 *   agent's only authentication, and the schema default is empty precisely so
 *   the agent cannot come up guessable. Refusing the combination gives the
 *   operator an error instead of an agent that answers nobody.
 *
 * @param user  Unused; pass NULL.
 * @retval 0        The candidate tree is acceptable.
 * @retval -EPROTO  A cross-field rule was violated; nothing is applied.
 */
int cfg_schema_xvalidate(const struct cfg_ctx *c, void *user);

/** Install the import migration table (NULL/0 clears it). */
int cfg_set_migrations(cfg_ctx_t *c, const cfg_migration_t *tbl, size_t n,
		       void *user);

/**
 * Reset every key to its schema default, clear staging, and erase the stored
 * copy of every key (spec §5.5 factory reset; key material is not this
 * module's business).
 *
 * @retval 0        Success.
 * @retval -EIO     At least one store erase failed; RAM is still reset.
 * @retval -EINVAL  @p c is NULL.
 */
int cfg_factory_reset(cfg_ctx_t *c);

/* ------------------------------------------------------------ get / set */

/** Read the live value of @p id. */
int cfg_get(const cfg_ctx_t *c, uint16_t id, cfg_val_t *out);

/** Read the staged value of @p id if one exists, else the live value. */
int cfg_get_effective(const cfg_ctx_t *c, uint16_t id, cfg_val_t *out);

/** True when @p id currently carries a staged value. */
bool cfg_is_staged(const cfg_ctx_t *c, uint16_t id);

/** Number of staged entries. */
uint16_t cfg_staged_count(const cfg_ctx_t *c);

/**
 * Validate @p v against the schema row for @p id and stage it.
 *
 * Staging a value equal to the live value is allowed and still counts as
 * staged; cfg_commit() reports it as staged-but-not-applied.
 *
 * @retval 0          Staged.
 * @retval -EINVAL    Bad argument.
 * @retval -ENOENT    @p id is not in the schema.
 * @retval -EPROTO    Type mismatch.
 * @retval -ERANGE    Value outside the schema bounds, or STR/BLOB too long.
 */
int cfg_set(cfg_ctx_t *c, uint16_t id, const cfg_val_t *v);

/** Drop the whole staged overlay. */
int cfg_revert(cfg_ctx_t *c);

/**
 * Validate and apply the staged overlay.
 *
 * @param res  Optional; receives the outcome even when the call fails.
 * @retval 0          Applied.
 * @retval -EINVAL    @p c is NULL.
 * @retval -EPROTO    A staged entry failed re-validation; nothing applied.
 * @retval -ERANGE    A staged entry is out of bounds; nothing applied.
 * @retval <0         The cross-field hook's own error; nothing applied.
 * @retval -EIO       Applied, but at least one key could not be persisted.
 */
int cfg_commit(cfg_ctx_t *c, cfg_commit_res_t *res);

/* ------------------------------------------------- typed convenience access */

int cfg_get_u64(const cfg_ctx_t *c, uint16_t id, uint64_t *out);
int cfg_get_bool(const cfg_ctx_t *c, uint16_t id, bool *out);
int cfg_get_i32(const cfg_ctx_t *c, uint16_t id, int32_t *out);
int cfg_get_f32(const cfg_ctx_t *c, uint16_t id, float *out);

/** Copy a STR/BLOB value out. @p out_len receives the byte count. */
int cfg_get_bytes(const cfg_ctx_t *c, uint16_t id, uint8_t *out, size_t cap,
		  size_t *out_len);

int cfg_set_u64(cfg_ctx_t *c, uint16_t id, uint64_t v);
int cfg_set_i32(cfg_ctx_t *c, uint16_t id, int32_t v);
int cfg_set_f32(cfg_ctx_t *c, uint16_t id, float v);
int cfg_set_bytes(cfg_ctx_t *c, uint16_t id, const uint8_t *v, size_t len);

/**
 * Decode a wire TLV body into a cfg_val_t without staging it.
 *
 * @p type is a cfg_type_t; @p data / @p len is the encoded value exactly as it
 * appears in a TLV record or an MCP CFG_SET payload. Numeric types must carry
 * their exact encoded width.
 *
 * @retval 0        Decoded.
 * @retval -EINVAL  Bad argument.
 * @retval -EPROTO  Unknown type or wrong encoded width.
 * @retval -ERANGE  STR/BLOB longer than CFG_VAL_MAX.
 */
int cfg_val_decode(cfg_val_t *out, uint8_t type, const uint8_t *data,
		   size_t len);

/**
 * Encode a cfg_val_t's value bytes (no ID, no type, no length prefix).
 *
 * @retval >=0      Bytes written.
 * @retval -EINVAL  Bad argument.
 * @retval -EPROTO  Unknown type.
 * @retval -ENOSPC  @p cap too small.
 */
int cfg_val_encode(const cfg_val_t *v, uint8_t *out, size_t cap);

/* ------------------------------------------------------------ TLV export */

/*
 * Stream layout (all multibyte fields little-endian):
 *
 *   magic     u8[4]  'M','C','F','1'
 *   schema_ver u16
 *   count      u16    number of records that follow
 *   record[]          u16 id, u8 type, u16 len, u8 value[len]
 *   crc32      u32    CRC-32/ISO-HDLC over every preceding byte
 *
 * Every schema key is exported, defaults included, so an export is a complete
 * restorable image rather than a diff. CFG_F_SECRET keys are omitted unless
 * the caller asks for them (MCP does so only for an authenticated session).
 * CFG_F_NOEXPORT keys (the admin credential) are omitted unconditionally —
 * they are provisioned out-of-band and must never appear on the wire.
 */

/** Bytes of the fixed export header. */
#define CFG_EXPORT_HDR_LEN 8U
/** Bytes of the fixed part of one export record (id, type, len). */
#define CFG_EXPORT_REC_HDR 5U
/** Smallest chunk buffer cfg_export_read() accepts. */
#define CFG_EXPORT_MIN_CHUNK (CFG_EXPORT_REC_HDR + CFG_VAL_MAX)

/** Export cursor. Opaque; initialise with cfg_export_begin(). */
typedef struct {
	uint8_t  phase;   /* 0 header, 1 records, 2 crc, 3 done */
	uint16_t idx;     /* next schema index to emit */
	uint16_t count;   /* records this export will emit */
	uint32_t crc;
	uint32_t offset;  /* bytes emitted so far */
	bool     secrets;
} cfg_export_t;

/** Start an export. @p secrets includes CFG_F_SECRET keys. */
int cfg_export_begin(const cfg_ctx_t *c, cfg_export_t *ex, bool secrets);

/**
 * Emit the next chunk of the export.
 *
 * Records are never split across chunks, so @p cap must be at least
 * CFG_EXPORT_MIN_CHUNK; a chunk may therefore come back shorter than @p cap.
 *
 * @param out_len  Receives the bytes written (may be 0 only when done).
 * @retval 1        This chunk completed the export.
 * @retval 0        More chunks remain.
 * @retval -EINVAL  Bad argument.
 * @retval -ENOSPC  @p cap below CFG_EXPORT_MIN_CHUNK.
 * @retval -EPROTO  A live value failed to encode (schema/table mismatch).
 */
int cfg_export_read(const cfg_ctx_t *c, cfg_export_t *ex, uint8_t *buf,
		    size_t cap, size_t *out_len);

/** One-shot export into a single buffer. */
int cfg_export_all(const cfg_ctx_t *c, uint8_t *buf, size_t cap, bool secrets,
		   size_t *out_len);

/* ------------------------------------------------------------ TLV import */

/** Import cursor. Opaque; initialise with cfg_import_begin(). */
typedef struct {
	uint8_t  phase;      /* 0 hdr, 1 rec hdr, 2 rec value, 3 crc, 4 done, 5 failed */
	uint8_t  hdr[CFG_EXPORT_HDR_LEN];
	uint8_t  hdr_n;
	uint8_t  rh[CFG_EXPORT_REC_HDR];
	uint8_t  rh_n;
	uint8_t  val[CFG_VAL_MAX];
	uint16_t val_n;
	uint16_t val_len;
	uint16_t id;
	uint8_t  type;
	uint8_t  crc_b[4];
	uint8_t  crc_n;
	uint32_t crc;        /* running CRC over everything before the trailer */
	uint16_t ver;
	uint16_t count;
	uint16_t got;
	uint16_t skipped;    /* unknown IDs dropped in lenient mode */
	uint32_t offset;     /* bytes consumed so far */
	bool     strict;
	int      err;
} cfg_import_t;

/**
 * Start an import. Clears the staged overlay: an import is a whole-tree
 * transaction, not an addition to whatever the operator had pending.
 *
 * @param strict  Reject an unknown key ID instead of skipping it.
 */
int cfg_import_begin(cfg_ctx_t *c, cfg_import_t *im, bool strict);

/**
 * Feed the next bytes of a TLV stream. Records are staged as they decode.
 *
 * Any failure aborts the import, drops the staged overlay, and latches the
 * error: further feeds return it unchanged until the next cfg_import_begin().
 *
 * @param consumed  Optional; bytes taken from @p data (all of them unless the
 *                  stream ended inside this buffer or an error latched).
 * @retval 1        The stream is complete and its CRC verified. Call
 *                  cfg_import_finish() to commit.
 * @retval 0        More bytes needed.
 * @retval -EINVAL  Bad argument.
 * @retval -EBADMSG Bad magic.
 * @retval -ENOTSUP Unknown schema version with no registered migration.
 * @retval -ENOENT  Unknown key ID (strict mode only).
 * @retval -EPROTO  Malformed record, or a value that failed type validation.
 * @retval -ERANGE  A value outside its schema bounds.
 * @retval -EILSEQ  CRC-32 trailer mismatch.
 */
int cfg_import_feed(cfg_ctx_t *c, cfg_import_t *im, const uint8_t *data,
		    size_t len, size_t *consumed);

/**
 * Commit a completed import (validate-all-then-commit, see cfg_commit()).
 *
 * @retval 0        Applied.
 * @retval -EINVAL  Bad argument.
 * @retval -EAGAIN  The stream is not complete.
 * @retval <0       Whatever cfg_commit() returned; the overlay is left staged.
 */
int cfg_import_finish(cfg_ctx_t *c, cfg_import_t *im, cfg_commit_res_t *res);

/** One-shot import of a complete TLV buffer, including the commit. */
int cfg_import_all(cfg_ctx_t *c, const uint8_t *buf, size_t len, bool strict,
		   cfg_commit_res_t *res);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_CFG_CFG_H_ */
