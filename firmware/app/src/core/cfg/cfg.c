/*
 * STS1000 "Meridian" — core/cfg: typed configuration registry.
 *
 * See cfg.h for the contract and cfg_schema.h for the key list.
 */

#include "cfg/cfg.h"

#include <errno.h>
#include <string.h>

#include "util/bytes.h"
#include "util/crc.h"

/* ------------------------------------------------------------------------- */
/* Schema table                                                              */
/* ------------------------------------------------------------------------- */

/*
 * The five row expansions turn CFG_SCHEMA() into the runtime table. The `sym`
 * argument is only used by cfg_schema.h to mint the CFG_ID_* enum, so it is
 * deliberately unused here.
 */
#define ROW_U(sym, id_, name_, ty, fl, def_, min_, max_)                       \
	{ .id = (id_), .type = (uint8_t)CFG_T_##ty, .flags = (uint8_t)(fl),    \
	  .maxlen = 0U, .name = (name_), .def = { .u = (uint64_t)(def_) },     \
	  .min = { .u = (uint64_t)(min_) }, .max = { .u = (uint64_t)(max_) },  \
	  .sdef = NULL },

#define ROW_I(sym, id_, name_, fl, def_, min_, max_)                           \
	{ .id = (id_), .type = (uint8_t)CFG_T_I32, .flags = (uint8_t)(fl),     \
	  .maxlen = 0U, .name = (name_), .def = { .i = (int32_t)(def_) },      \
	  .min = { .i = (int32_t)(min_) }, .max = { .i = (int32_t)(max_) },    \
	  .sdef = NULL },

#define ROW_F(sym, id_, name_, fl, def_, min_, max_)                           \
	{ .id = (id_), .type = (uint8_t)CFG_T_F32, .flags = (uint8_t)(fl),     \
	  .maxlen = 0U, .name = (name_), .def = { .f = (def_) },               \
	  .min = { .f = (min_) }, .max = { .f = (max_) }, .sdef = NULL },

#define ROW_S(sym, id_, name_, fl, maxlen_, def_)                              \
	{ .id = (id_), .type = (uint8_t)CFG_T_STR, .flags = (uint8_t)(fl),     \
	  .maxlen = (maxlen_), .name = (name_), .def = { .u = 0U },            \
	  .min = { .u = 0U }, .max = { .u = 0U }, .sdef = (def_) },

#define ROW_B(sym, id_, name_, fl, maxlen_)                                    \
	{ .id = (id_), .type = (uint8_t)CFG_T_BLOB, .flags = (uint8_t)(fl),    \
	  .maxlen = (maxlen_), .name = (name_), .def = { .u = 0U },            \
	  .min = { .u = 0U }, .max = { .u = 0U }, .sdef = NULL },

static const cfg_key_t schema[CFG_KEY_COUNT] = {
	CFG_SCHEMA(ROW_U, ROW_I, ROW_F, ROW_S, ROW_B)
};

/* Largest store/TLV record: one type byte plus the widest value. */
#define CFG_REC_MAX (1U + CFG_VAL_MAX)

/* A count field bigger than this in an import header is not a schema we could
 * ever have written; refuse it rather than stream megabytes of garbage. */
#define CFG_IMPORT_COUNT_MAX 1024U

/* ------------------------------------------------------------------------- */
/* Schema access                                                             */
/* ------------------------------------------------------------------------- */

size_t cfg_key_count(void)
{
	return CFG_KEY_COUNT;
}

const cfg_key_t *cfg_key_at(size_t idx)
{
	return (idx < CFG_KEY_COUNT) ? &schema[idx] : NULL;
}

/* First index whose ID is >= id. Returns CFG_KEY_COUNT when there is none. */
static size_t schema_lower_bound(uint16_t id)
{
	size_t lo = 0U;
	size_t hi = CFG_KEY_COUNT;

	while (lo < hi) {
		size_t mid = lo + ((hi - lo) / 2U);

		if (schema[mid].id < id) {
			lo = mid + 1U;
		} else {
			hi = mid;
		}
	}
	return lo;
}

int cfg_key_index(uint16_t id)
{
	size_t i = schema_lower_bound(id);

	if ((i < CFG_KEY_COUNT) && (schema[i].id == id)) {
		return (int)i;
	}
	return -ENOENT;
}

const cfg_key_t *cfg_key_find(uint16_t id)
{
	int idx = cfg_key_index(id);

	return (idx >= 0) ? &schema[idx] : NULL;
}

int cfg_key_lower_bound(uint16_t start_id)
{
	size_t i = schema_lower_bound(start_id);

	return (i < CFG_KEY_COUNT) ? (int)i : -ENOENT;
}

size_t cfg_type_width(uint8_t type)
{
	switch (type) {
	case CFG_T_BOOL:
	case CFG_T_U8:
		return 1U;
	case CFG_T_U16:
		return 2U;
	case CFG_T_U32:
	case CFG_T_I32:
	case CFG_T_F32:
		return 4U;
	case CFG_T_U64:
		return 8U;
	default:
		return 0U; /* STR / BLOB are length-delimited */
	}
}

static bool type_is_numeric(uint8_t type)
{
	return cfg_type_width(type) != 0U;
}

/* ------------------------------------------------------------------------- */
/* Values                                                                    */
/* ------------------------------------------------------------------------- */

static void val_default(const cfg_key_t *k, cfg_val_t *out)
{
	memset(out, 0, sizeof(*out));
	out->type = k->type;

	switch (k->type) {
	case CFG_T_I32:
		out->v.i = k->def.i;
		break;
	case CFG_T_F32:
		out->v.f = k->def.f;
		break;
	case CFG_T_STR:
		if (k->sdef != NULL) {
			size_t n = strlen(k->sdef);

			if (n > k->maxlen) {
				n = k->maxlen;
			}
			memcpy(out->v.b, k->sdef, n);
			out->len = (uint16_t)n;
		}
		break;
	case CFG_T_BLOB:
		/* Empty by definition: "not provisioned". */
		break;
	default:
		out->v.u = k->def.u;
		break;
	}
}

/* Type/bounds check of @p v against row @p k. */
static int val_validate(const cfg_key_t *k, const cfg_val_t *v)
{
	if (v->type != k->type) {
		return -EPROTO;
	}

	switch (k->type) {
	case CFG_T_I32:
		if ((v->v.i < k->min.i) || (v->v.i > k->max.i)) {
			return -ERANGE;
		}
		break;
	case CFG_T_F32:
		/* Written so that a NaN fails rather than slipping through the
		 * negation of a pair of ordered comparisons. */
		if (!((v->v.f >= k->min.f) && (v->v.f <= k->max.f))) {
			return -ERANGE;
		}
		break;
	case CFG_T_STR:
	case CFG_T_BLOB:
		if (v->len > k->maxlen) {
			return -ERANGE;
		}
		if (v->len > CFG_VAL_MAX) {
			return -ERANGE;
		}
		break;
	default:
		if ((v->v.u < k->min.u) || (v->v.u > k->max.u)) {
			return -ERANGE;
		}
		break;
	}
	return 0;
}

static bool val_equal(const cfg_val_t *a, const cfg_val_t *b)
{
	if (a->type != b->type) {
		return false;
	}

	switch (a->type) {
	case CFG_T_F32: {
		/* Bit comparison, not ==: this decides whether the key is
		 * written back to the store, so -0.0 vs 0.0 and NaN vs NaN must
		 * both answer "same bits, nothing to write". Reading the wider
		 * union member instead would sample bytes the float never
		 * wrote. */
		uint32_t ba;
		uint32_t bb;

		memcpy(&ba, &a->v.f, sizeof(ba));
		memcpy(&bb, &b->v.f, sizeof(bb));
		return ba == bb;
	}
	case CFG_T_I32:
		return a->v.i == b->v.i;
	case CFG_T_STR:
	case CFG_T_BLOB:
		return (a->len == b->len) &&
		       (memcmp(a->v.b, b->v.b, a->len) == 0);
	default:
		return a->v.u == b->v.u;
	}
}

int cfg_val_decode(cfg_val_t *out, uint8_t type, const uint8_t *data,
		   size_t len)
{
	size_t w;

	if ((out == NULL) || ((data == NULL) && (len != 0U))) {
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));
	out->type = type;

	w = cfg_type_width(type);
	if (w != 0U) {
		if (len != w) {
			return -EPROTO;
		}
		switch (type) {
		case CFG_T_BOOL:
		case CFG_T_U8:
			out->v.u = data[0];
			break;
		case CFG_T_U16:
			out->v.u = bytes_get_le16(data);
			break;
		case CFG_T_U32:
			out->v.u = bytes_get_le32(data);
			break;
		case CFG_T_U64:
			out->v.u = bytes_get_le64(data);
			break;
		case CFG_T_I32:
			out->v.i = (int32_t)bytes_get_le32(data);
			break;
		default: { /* CFG_T_F32 */
			uint32_t bits = bytes_get_le32(data);
			float f;

			memcpy(&f, &bits, sizeof(f));
			out->v.f = f;
			break;
		}
		}
		return 0;
	}

	if ((type != CFG_T_STR) && (type != CFG_T_BLOB)) {
		return -EPROTO;
	}
	if (len > CFG_VAL_MAX) {
		return -ERANGE;
	}
	if (len != 0U) {
		memcpy(out->v.b, data, len);
	}
	out->len = (uint16_t)len;
	return 0;
}

int cfg_val_encode(const cfg_val_t *v, uint8_t *out, size_t cap)
{
	size_t w;

	if ((v == NULL) || ((out == NULL) && (cap != 0U))) {
		return -EINVAL;
	}

	w = cfg_type_width(v->type);
	if (w != 0U) {
		if (cap < w) {
			return -ENOSPC;
		}
		switch (v->type) {
		case CFG_T_BOOL:
		case CFG_T_U8:
			out[0] = (uint8_t)(v->v.u & 0xFFU);
			break;
		case CFG_T_U16:
			bytes_put_le16(out, (uint16_t)(v->v.u & 0xFFFFU));
			break;
		case CFG_T_U32:
			bytes_put_le32(out, (uint32_t)(v->v.u & 0xFFFFFFFFU));
			break;
		case CFG_T_U64:
			bytes_put_le64(out, v->v.u);
			break;
		case CFG_T_I32:
			bytes_put_le32(out, (uint32_t)v->v.i);
			break;
		default: { /* CFG_T_F32 */
			uint32_t bits;

			memcpy(&bits, &v->v.f, sizeof(bits));
			bytes_put_le32(out, bits);
			break;
		}
		}
		return (int)w;
	}

	if ((v->type != CFG_T_STR) && (v->type != CFG_T_BLOB)) {
		return -EPROTO;
	}
	if (v->len > CFG_VAL_MAX) {
		return -EPROTO;
	}
	if (cap < v->len) {
		return -ENOSPC;
	}
	if (v->len != 0U) {
		memcpy(out, v->v.b, v->len);
	}
	return (int)v->len;
}

/* ------------------------------------------------------------------------- */
/* Staging bookkeeping                                                       */
/* ------------------------------------------------------------------------- */

static bool staged_test(const cfg_ctx_t *c, size_t idx)
{
	return (c->staged_bits[idx / 32U] & (1U << (idx % 32U))) != 0U;
}

static void staged_set_bit(cfg_ctx_t *c, size_t idx)
{
	if (!staged_test(c, idx)) {
		c->staged_bits[idx / 32U] |= (uint32_t)1U << (idx % 32U);
		c->staged_n++;
	}
}

static void staged_clear_all(cfg_ctx_t *c)
{
	memset(c->staged_bits, 0, sizeof(c->staged_bits));
	c->staged_n = 0U;
}

/* ------------------------------------------------------------------------- */
/* Persistence                                                               */
/* ------------------------------------------------------------------------- */

/* Store record: [u8 type][value bytes]. The type byte makes a schema retype
 * detectable as corruption instead of silently reinterpreting old bytes. */
static int rec_encode(const cfg_val_t *v, uint8_t *buf, size_t cap,
		      size_t *out_len)
{
	int n;

	if (cap < 1U) {
		return -ENOSPC;
	}
	buf[0] = v->type;
	n = cfg_val_encode(v, &buf[1], cap - 1U);
	if (n < 0) {
		return n;
	}
	*out_len = (size_t)n + 1U;
	return 0;
}

static int persist_key(cfg_ctx_t *c, const cfg_key_t *k, const cfg_val_t *v)
{
	uint8_t buf[CFG_REC_MAX];
	size_t n = 0U;
	int rc;

	if ((c->store == NULL) || (c->store->save == NULL)) {
		return 0; /* RAM-only registry */
	}

	rc = rec_encode(v, buf, sizeof(buf), &n);
	if (rc != 0) {
		return rc;
	}
	rc = c->store->save(c->store->ctx, k->id, buf, n);
	return (rc < 0) ? rc : 0;
}

/* ------------------------------------------------------------------------- */
/* Lifecycle                                                                 */
/* ------------------------------------------------------------------------- */

int cfg_init(cfg_ctx_t *c, const port_store_t *store)
{
	size_t i;

	if (c == NULL) {
		return -EINVAL;
	}

	memset(c, 0, sizeof(*c));
	c->store = store;

	for (i = 0U; i < CFG_KEY_COUNT; i++) {
		val_default(&schema[i], &c->live[i]);
	}
	staged_clear_all(c);
	return 0;
}

int cfg_load_all(cfg_ctx_t *c, uint32_t *out_corrupt)
{
	size_t i;

	if (c == NULL) {
		return -EINVAL;
	}

	c->load_corrupt = 0U;
	c->load_missing = 0U;

	for (i = 0U; i < CFG_KEY_COUNT; i++) {
		const cfg_key_t *k = &schema[i];
		uint8_t buf[CFG_REC_MAX];
		cfg_val_t v;
		int rc;

		val_default(k, &c->live[i]);

		if ((c->store == NULL) || (c->store->load == NULL)) {
			c->load_missing++;
			continue;
		}

		rc = c->store->load(c->store->ctx, k->id, buf, sizeof(buf));
		if (rc == -2) {
			c->load_missing++;
			continue;
		}
		if ((rc < 1) || ((size_t)rc > sizeof(buf))) {
			/* Negative error, a zero-length record that cannot even
			 * carry the type byte, or a store that claims to have
			 * written more than it was given. */
			c->load_corrupt++;
			continue;
		}
		if (buf[0] != k->type) {
			c->load_corrupt++;
			continue;
		}
		if (cfg_val_decode(&v, buf[0], &buf[1], (size_t)rc - 1U) != 0) {
			c->load_corrupt++;
			continue;
		}
		if (val_validate(k, &v) != 0) {
			c->load_corrupt++;
			continue;
		}
		c->live[i] = v;
	}

	staged_clear_all(c);

	if (out_corrupt != NULL) {
		*out_corrupt = c->load_corrupt;
	}
	return 0;
}

int cfg_set_validate_hook(cfg_ctx_t *c, cfg_validate_fn fn, void *user)
{
	if (c == NULL) {
		return -EINVAL;
	}
	c->xvalidate = fn;
	c->xvalidate_user = user;
	return 0;
}

int cfg_set_migrations(cfg_ctx_t *c, const cfg_migration_t *tbl, size_t n,
		       void *user)
{
	if (c == NULL) {
		return -EINVAL;
	}
	if ((tbl == NULL) && (n != 0U)) {
		return -EINVAL;
	}
	c->migrations = tbl;
	c->migrations_n = n;
	c->migrations_user = user;
	return 0;
}

int cfg_factory_reset(cfg_ctx_t *c)
{
	size_t i;
	int last = 0;

	if (c == NULL) {
		return -EINVAL;
	}

	for (i = 0U; i < CFG_KEY_COUNT; i++) {
		val_default(&schema[i], &c->live[i]);

		if ((c->store != NULL) && (c->store->erase != NULL)) {
			int rc = c->store->erase(c->store->ctx, schema[i].id);

			/* -2 is "no such key", which is the desired end state. */
			if ((rc < 0) && (rc != -2)) {
				last = -EIO;
			}
		}
	}
	staged_clear_all(c);
	return last;
}

/* ------------------------------------------------------------------------- */
/* Get / set                                                                 */
/* ------------------------------------------------------------------------- */

int cfg_get(const cfg_ctx_t *c, uint16_t id, cfg_val_t *out)
{
	int idx;

	if ((c == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	idx = cfg_key_index(id);
	if (idx < 0) {
		return idx;
	}
	*out = c->live[idx];
	return 0;
}

int cfg_get_effective(const cfg_ctx_t *c, uint16_t id, cfg_val_t *out)
{
	int idx;

	if ((c == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	idx = cfg_key_index(id);
	if (idx < 0) {
		return idx;
	}
	*out = staged_test(c, (size_t)idx) ? c->stage[idx] : c->live[idx];
	return 0;
}

bool cfg_is_staged(const cfg_ctx_t *c, uint16_t id)
{
	int idx;

	if (c == NULL) {
		return false;
	}
	idx = cfg_key_index(id);
	return (idx >= 0) && staged_test(c, (size_t)idx);
}

uint16_t cfg_staged_count(const cfg_ctx_t *c)
{
	return (c != NULL) ? c->staged_n : 0U;
}

int cfg_set(cfg_ctx_t *c, uint16_t id, const cfg_val_t *v)
{
	int idx;
	int rc;

	if ((c == NULL) || (v == NULL)) {
		return -EINVAL;
	}
	idx = cfg_key_index(id);
	if (idx < 0) {
		return idx;
	}
	rc = val_validate(&schema[idx], v);
	if (rc != 0) {
		return rc;
	}

	c->stage[idx] = *v;
	staged_set_bit(c, (size_t)idx);
	return 0;
}

int cfg_revert(cfg_ctx_t *c)
{
	if (c == NULL) {
		return -EINVAL;
	}
	staged_clear_all(c);
	return 0;
}

int cfg_commit(cfg_ctx_t *c, cfg_commit_res_t *res)
{
	cfg_commit_res_t local;
	size_t i;
	int rc;

	if (res == NULL) {
		res = &local;
	}
	memset(res, 0, sizeof(*res));

	if (c == NULL) {
		return -EINVAL;
	}

	/* Pass 1 — re-validate every staged entry. Nothing has been touched
	 * yet, so a failure here leaves the operator's staged set intact. */
	for (i = 0U; i < CFG_KEY_COUNT; i++) {
		if (!staged_test(c, i)) {
			continue;
		}
		res->staged++;
		rc = val_validate(&schema[i], &c->stage[i]);
		if (rc != 0) {
			return rc;
		}
	}

	/* Pass 2 — cross-field validation of the candidate tree. The hook reads
	 * through cfg_get_effective(), which already overlays staging. */
	if (c->xvalidate != NULL) {
		rc = c->xvalidate(c, c->xvalidate_user);
		if (rc != 0) {
			return (rc < 0) ? rc : -EPROTO;
		}
	}

	/* Pass 3 — apply. From here nothing may fail the transaction. */
	for (i = 0U; i < CFG_KEY_COUNT; i++) {
		if (!staged_test(c, i)) {
			continue;
		}
		if (val_equal(&c->live[i], &c->stage[i])) {
			continue;
		}

		c->live[i] = c->stage[i];
		res->applied++;

		if ((schema[i].flags & CFG_F_REBOOT_REQUIRED) != 0U) {
			res->reboot_keys++;
			res->reboot_groups |=
				CFG_GROUP_BIT(CFG_GROUP(schema[i].id));
		}

		if (persist_key(c, &schema[i], &c->live[i]) != 0) {
			res->persist_errors++;
		}
	}

	staged_clear_all(c);
	return (res->persist_errors != 0U) ? -EIO : 0;
}

/* ------------------------------------------------------------------------- */
/* Typed convenience access                                                  */
/* ------------------------------------------------------------------------- */

int cfg_get_u64(const cfg_ctx_t *c, uint16_t id, uint64_t *out)
{
	cfg_val_t v;
	int rc;

	if (out == NULL) {
		return -EINVAL;
	}
	rc = cfg_get(c, id, &v);
	if (rc != 0) {
		return rc;
	}
	switch (v.type) {
	case CFG_T_BOOL:
	case CFG_T_U8:
	case CFG_T_U16:
	case CFG_T_U32:
	case CFG_T_U64:
		*out = v.v.u;
		return 0;
	default:
		return -EPROTO;
	}
}

int cfg_get_bool(const cfg_ctx_t *c, uint16_t id, bool *out)
{
	uint64_t u;
	int rc;

	if (out == NULL) {
		return -EINVAL;
	}
	rc = cfg_get_u64(c, id, &u);
	if (rc != 0) {
		return rc;
	}
	*out = (u != 0U);
	return 0;
}

int cfg_get_i32(const cfg_ctx_t *c, uint16_t id, int32_t *out)
{
	cfg_val_t v;
	int rc;

	if (out == NULL) {
		return -EINVAL;
	}
	rc = cfg_get(c, id, &v);
	if (rc != 0) {
		return rc;
	}
	if (v.type != CFG_T_I32) {
		return -EPROTO;
	}
	*out = v.v.i;
	return 0;
}

int cfg_get_f32(const cfg_ctx_t *c, uint16_t id, float *out)
{
	cfg_val_t v;
	int rc;

	if (out == NULL) {
		return -EINVAL;
	}
	rc = cfg_get(c, id, &v);
	if (rc != 0) {
		return rc;
	}
	if (v.type != CFG_T_F32) {
		return -EPROTO;
	}
	*out = v.v.f;
	return 0;
}

int cfg_get_bytes(const cfg_ctx_t *c, uint16_t id, uint8_t *out, size_t cap,
		  size_t *out_len)
{
	cfg_val_t v;
	int rc;

	if ((out == NULL) && (cap != 0U)) {
		return -EINVAL;
	}
	rc = cfg_get(c, id, &v);
	if (rc != 0) {
		return rc;
	}
	if ((v.type != CFG_T_STR) && (v.type != CFG_T_BLOB)) {
		return -EPROTO;
	}
	if (v.len > cap) {
		return -ENOSPC;
	}
	if (v.len != 0U) {
		memcpy(out, v.v.b, v.len);
	}
	if (out_len != NULL) {
		*out_len = v.len;
	}
	return 0;
}

int cfg_set_u64(cfg_ctx_t *c, uint16_t id, uint64_t v)
{
	const cfg_key_t *k = cfg_key_find(id);
	cfg_val_t val;

	if (k == NULL) {
		return -ENOENT;
	}
	if ((k->type == CFG_T_I32) || (k->type == CFG_T_F32) ||
	    !type_is_numeric(k->type)) {
		return -EPROTO;
	}
	memset(&val, 0, sizeof(val));
	val.type = k->type;
	val.v.u = v;
	return cfg_set(c, id, &val);
}

int cfg_set_i32(cfg_ctx_t *c, uint16_t id, int32_t v)
{
	cfg_val_t val;

	memset(&val, 0, sizeof(val));
	val.type = CFG_T_I32;
	val.v.i = v;
	return cfg_set(c, id, &val);
}

int cfg_set_f32(cfg_ctx_t *c, uint16_t id, float v)
{
	cfg_val_t val;

	memset(&val, 0, sizeof(val));
	val.type = CFG_T_F32;
	val.v.f = v;
	return cfg_set(c, id, &val);
}

int cfg_set_bytes(cfg_ctx_t *c, uint16_t id, const uint8_t *v, size_t len)
{
	const cfg_key_t *k = cfg_key_find(id);
	cfg_val_t val;

	if (k == NULL) {
		return -ENOENT;
	}
	if ((k->type != CFG_T_STR) && (k->type != CFG_T_BLOB)) {
		return -EPROTO;
	}
	if ((v == NULL) && (len != 0U)) {
		return -EINVAL;
	}
	if (len > CFG_VAL_MAX) {
		return -ERANGE;
	}
	memset(&val, 0, sizeof(val));
	val.type = k->type;
	val.len = (uint16_t)len;
	if (len != 0U) {
		memcpy(val.v.b, v, len);
	}
	return cfg_set(c, id, &val);
}

/* ------------------------------------------------------------------------- */
/* TLV export                                                                */
/* ------------------------------------------------------------------------- */

static bool export_includes(const cfg_key_t *k, bool secrets)
{
	return secrets || ((k->flags & CFG_F_SECRET) == 0U);
}

int cfg_export_begin(const cfg_ctx_t *c, cfg_export_t *ex, bool secrets)
{
	size_t i;

	if ((c == NULL) || (ex == NULL)) {
		return -EINVAL;
	}

	memset(ex, 0, sizeof(*ex));
	ex->secrets = secrets;
	ex->crc = STS_CRC32_IEEE_SEED;

	for (i = 0U; i < CFG_KEY_COUNT; i++) {
		if (export_includes(&schema[i], secrets)) {
			ex->count++;
		}
	}
	return 0;
}

/* Append @p n bytes to the chunk and fold them into the running CRC. */
static void export_emit(cfg_export_t *ex, uint8_t *buf, size_t *w,
			const uint8_t *src, size_t n)
{
	memcpy(&buf[*w], src, n);
	/* Seed 0 is the zlib convention documented in util/crc.h: the running
	 * value is always the finalised CRC, so chaining composes. */
	ex->crc = sts_crc32_ieee_update(ex->crc, src, n);
	*w += n;
	ex->offset += (uint32_t)n;
}

int cfg_export_read(const cfg_ctx_t *c, cfg_export_t *ex, uint8_t *buf,
		    size_t cap, size_t *out_len)
{
	size_t w = 0U;

	if ((c == NULL) || (ex == NULL) || (buf == NULL) || (out_len == NULL)) {
		return -EINVAL;
	}
	if (cap < CFG_EXPORT_MIN_CHUNK) {
		return -ENOSPC;
	}

	if (ex->phase == 0U) {
		uint8_t hdr[CFG_EXPORT_HDR_LEN];

		hdr[0] = (uint8_t)'M';
		hdr[1] = (uint8_t)'C';
		hdr[2] = (uint8_t)'F';
		hdr[3] = (uint8_t)'1';
		bytes_put_le16(&hdr[4], (uint16_t)CFG_SCHEMA_VERSION);
		bytes_put_le16(&hdr[6], ex->count);
		export_emit(ex, buf, &w, hdr, sizeof(hdr));
		ex->phase = 1U;
	}

	while ((ex->phase == 1U) && ((cap - w) >= CFG_EXPORT_MIN_CHUNK)) {
		const cfg_key_t *k;
		uint8_t rec[CFG_EXPORT_REC_HDR + CFG_VAL_MAX];
		int n;

		if (ex->idx >= CFG_KEY_COUNT) {
			ex->phase = 2U;
			break;
		}
		k = &schema[ex->idx];
		if (!export_includes(k, ex->secrets)) {
			ex->idx++;
			continue;
		}

		n = cfg_val_encode(&c->live[ex->idx], &rec[CFG_EXPORT_REC_HDR],
				   CFG_VAL_MAX);
		if (n < 0) {
			return -EPROTO;
		}
		bytes_put_le16(&rec[0], k->id);
		rec[2] = k->type;
		bytes_put_le16(&rec[3], (uint16_t)n);
		export_emit(ex, buf, &w, rec,
			    CFG_EXPORT_REC_HDR + (size_t)n);
		ex->idx++;
	}

	if ((ex->phase == 2U) && ((cap - w) >= 4U)) {
		uint8_t tr[4];
		uint32_t crc = ex->crc;

		bytes_put_le32(tr, crc);
		/* The trailer is not itself covered, so emit it directly. */
		memcpy(&buf[w], tr, sizeof(tr));
		w += sizeof(tr);
		ex->offset += 4U;
		ex->phase = 3U;
	}

	*out_len = w;
	return (ex->phase == 3U) ? 1 : 0;
}

int cfg_export_all(const cfg_ctx_t *c, uint8_t *buf, size_t cap, bool secrets,
		   size_t *out_len)
{
	cfg_export_t ex;
	size_t total = 0U;
	int rc;

	if ((buf == NULL) || (out_len == NULL)) {
		return -EINVAL;
	}
	rc = cfg_export_begin(c, &ex, secrets);
	if (rc != 0) {
		return rc;
	}

	for (;;) {
		size_t n = 0U;

		rc = cfg_export_read(c, &ex, &buf[total], cap - total, &n);
		if (rc < 0) {
			return rc;
		}
		total += n;
		if (rc == 1) {
			break;
		}
		if (n == 0U) {
			return -ENOSPC;
		}
	}

	*out_len = total;
	return 0;
}

/* ------------------------------------------------------------------------- */
/* TLV import                                                                */
/* ------------------------------------------------------------------------- */

#define IMP_HDR  0U
#define IMP_RECH 1U
#define IMP_RECV 2U
#define IMP_CRC  3U
#define IMP_DONE 4U
#define IMP_FAIL 5U

int cfg_import_begin(cfg_ctx_t *c, cfg_import_t *im, bool strict)
{
	if ((c == NULL) || (im == NULL)) {
		return -EINVAL;
	}
	memset(im, 0, sizeof(*im));
	im->strict = strict;
	im->crc = STS_CRC32_IEEE_SEED;
	staged_clear_all(c);
	return 0;
}

static int import_fail(cfg_ctx_t *c, cfg_import_t *im, int err)
{
	im->phase = IMP_FAIL;
	im->err = err;
	staged_clear_all(c);
	return err;
}

/* Look up a migration for the schema version that wrote this stream. */
static const cfg_migration_t *import_migration(const cfg_ctx_t *c, uint16_t ver)
{
	size_t i;

	for (i = 0U; i < c->migrations_n; i++) {
		if (c->migrations[i].from_ver == ver) {
			return &c->migrations[i];
		}
	}
	return NULL;
}

static int import_header(cfg_ctx_t *c, cfg_import_t *im)
{
	if ((im->hdr[0] != (uint8_t)'M') || (im->hdr[1] != (uint8_t)'C') ||
	    (im->hdr[2] != (uint8_t)'F') || (im->hdr[3] != (uint8_t)'1')) {
		return import_fail(c, im, -EBADMSG);
	}

	im->ver = bytes_get_le16(&im->hdr[4]);
	im->count = bytes_get_le16(&im->hdr[6]);

	if (im->count > CFG_IMPORT_COUNT_MAX) {
		return import_fail(c, im, -EPROTO);
	}
	if ((im->ver != CFG_SCHEMA_VERSION) &&
	    (import_migration(c, im->ver) == NULL)) {
		return import_fail(c, im, -ENOTSUP);
	}

	im->phase = (im->count == 0U) ? IMP_CRC : IMP_RECH;
	return 0;
}

/* Stage one fully-decoded record. */
static int import_record(cfg_ctx_t *c, cfg_import_t *im)
{
	const cfg_migration_t *mig = NULL;
	const cfg_key_t *k;
	cfg_val_t v;
	uint16_t id = im->id;
	int rc;

	rc = cfg_val_decode(&v, im->type, im->val, im->val_len);
	if (rc != 0) {
		return import_fail(c, im, -EPROTO);
	}

	if (im->ver != CFG_SCHEMA_VERSION) {
		mig = import_migration(c, im->ver);
	}
	if (mig != NULL) {
		rc = mig->fn(&id, &v, c->migrations_user);
		if (rc < 0) {
			return import_fail(c, im, rc);
		}
		if (rc == 1) {
			return 0; /* migration dropped a retired key */
		}
	}

	k = cfg_key_find(id);
	if (k == NULL) {
		if (im->strict) {
			return import_fail(c, im, -ENOENT);
		}
		im->skipped++;
		return 0;
	}

	rc = cfg_set(c, id, &v);
	if (rc != 0) {
		return import_fail(c, im, rc);
	}
	return 0;
}

int cfg_import_feed(cfg_ctx_t *c, cfg_import_t *im, const uint8_t *data,
		    size_t len, size_t *consumed)
{
	size_t r = 0U;
	int rc = 0;

	if ((c == NULL) || (im == NULL) || ((data == NULL) && (len != 0U))) {
		return -EINVAL;
	}
	if (im->phase == IMP_FAIL) {
		if (consumed != NULL) {
			*consumed = 0U;
		}
		return im->err;
	}

	while ((r < len) && (im->phase < IMP_DONE)) {
		size_t take;

		switch (im->phase) {
		case IMP_HDR:
			take = CFG_EXPORT_HDR_LEN - im->hdr_n;
			if (take > (len - r)) {
				take = len - r;
			}
			memcpy(&im->hdr[im->hdr_n], &data[r], take);
			im->hdr_n = (uint8_t)(im->hdr_n + take);
			im->crc = sts_crc32_ieee_update(im->crc, &data[r], take);
			r += take;
			im->offset += (uint32_t)take;
			if (im->hdr_n == CFG_EXPORT_HDR_LEN) {
				rc = import_header(c, im);
				if (rc != 0) {
					goto out;
				}
			}
			break;

		case IMP_RECH:
			take = CFG_EXPORT_REC_HDR - im->rh_n;
			if (take > (len - r)) {
				take = len - r;
			}
			memcpy(&im->rh[im->rh_n], &data[r], take);
			im->rh_n = (uint8_t)(im->rh_n + take);
			im->crc = sts_crc32_ieee_update(im->crc, &data[r], take);
			r += take;
			im->offset += (uint32_t)take;
			if (im->rh_n < CFG_EXPORT_REC_HDR) {
				break;
			}
			im->id = bytes_get_le16(&im->rh[0]);
			im->type = im->rh[2];
			im->val_len = bytes_get_le16(&im->rh[3]);
			im->val_n = 0U;
			if (im->val_len > CFG_VAL_MAX) {
				rc = import_fail(c, im, -EPROTO);
				goto out;
			}
			im->phase = IMP_RECV;
			if (im->val_len == 0U) {
				rc = import_record(c, im);
				if (rc != 0) {
					goto out;
				}
				im->got++;
				im->rh_n = 0U;
				im->phase = (im->got >= im->count) ? IMP_CRC
								   : IMP_RECH;
			}
			break;

		case IMP_RECV:
			take = (size_t)(im->val_len - im->val_n);
			if (take > (len - r)) {
				take = len - r;
			}
			memcpy(&im->val[im->val_n], &data[r], take);
			im->val_n = (uint16_t)(im->val_n + take);
			im->crc = sts_crc32_ieee_update(im->crc, &data[r], take);
			r += take;
			im->offset += (uint32_t)take;
			if (im->val_n < im->val_len) {
				break;
			}
			rc = import_record(c, im);
			if (rc != 0) {
				goto out;
			}
			im->got++;
			im->rh_n = 0U;
			im->phase = (im->got >= im->count) ? IMP_CRC : IMP_RECH;
			break;

		default: /* IMP_CRC */
			take = 4U - im->crc_n;
			if (take > (len - r)) {
				take = len - r;
			}
			memcpy(&im->crc_b[im->crc_n], &data[r], take);
			im->crc_n = (uint8_t)(im->crc_n + take);
			r += take;
			im->offset += (uint32_t)take;
			if (im->crc_n < 4U) {
				break;
			}
			if (bytes_get_le32(im->crc_b) != im->crc) {
				rc = import_fail(c, im, -EILSEQ);
				goto out;
			}
			im->phase = IMP_DONE;
			break;
		}
	}

	rc = (im->phase == IMP_DONE) ? 1 : 0;

out:
	if (consumed != NULL) {
		*consumed = r;
	}
	return rc;
}

int cfg_import_finish(cfg_ctx_t *c, cfg_import_t *im, cfg_commit_res_t *res)
{
	if ((c == NULL) || (im == NULL)) {
		return -EINVAL;
	}
	if (im->phase == IMP_FAIL) {
		return im->err;
	}
	if (im->phase != IMP_DONE) {
		return -EAGAIN;
	}
	return cfg_commit(c, res);
}

int cfg_import_all(cfg_ctx_t *c, const uint8_t *buf, size_t len, bool strict,
		   cfg_commit_res_t *res)
{
	cfg_import_t im;
	int rc;

	rc = cfg_import_begin(c, &im, strict);
	if (rc != 0) {
		return rc;
	}
	rc = cfg_import_feed(c, &im, buf, len, NULL);
	if (rc < 0) {
		return rc;
	}
	if (rc == 0) {
		/* The buffer ran out mid-stream. */
		staged_clear_all(c);
		return -EBADMSG;
	}
	return cfg_import_finish(c, &im, res);
}
