/*
 * STS1000 "Meridian" — core/ubx: u-blox UBX protocol codec.
 *
 * See ubx.h for the contract. This file is organised as:
 *
 *   1. signed-field helpers      well-defined two's-complement reinterpretation
 *   2. checksum + frame builder
 *   3. streaming parser
 *   4. typed decoders            NAV-PVT / NAV-SAT / NAV-TIMELS / NAV-SVIN /
 *                                TIM-TP / MON-RF / ACK
 *   5. CFG-VALSET builder
 *
 * Field offsets in the decoders are written as literals with the ICD field name
 * beside them. That is deliberate: a reader with the interface description open
 * can check a decoder line by line, which a pile of named offset constants makes
 * harder, not easier.
 */

#include "ubx/ubx.h"

#include <errno.h>
#include <string.h>

#include "util/bytes.h"

/* --------------------------------------------------------------- signed -- */
/*
 * Converting a uint32_t above INT32_MAX to int32_t is implementation-defined
 * before C23. These do it with only well-defined arithmetic; GCC folds each to
 * nothing on both the host and Cortex-M33. Every signed UBX field goes through
 * one of them (qErr in particular is signed and routinely negative).
 */

static int8_t ubx_s8(uint8_t v)
{
	return (v <= 0x7FU) ? (int8_t)v : (int8_t)((int)v - 0x100);
}

static int16_t ubx_s16(uint16_t v)
{
	return (v <= 0x7FFFU) ? (int16_t)v : (int16_t)((int32_t)v - 0x10000);
}

static int32_t ubx_s32(uint32_t v)
{
	return (v <= 0x7FFFFFFFUL) ? (int32_t)v
				   : (int32_t)(v - 0x80000000UL) + INT32_MIN;
}

/* ------------------------------------------------------------- checksum -- */

void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b)
{
	uint8_t a = 0U;
	uint8_t b = 0U;
	size_t i;

	if ((ck_a == NULL) || (ck_b == NULL)) {
		return;
	}
	if (data != NULL) {
		for (i = 0U; i < len; i++) {
			a = (uint8_t)(a + data[i]);
			b = (uint8_t)(b + a);
		}
	}

	*ck_a = a;
	*ck_b = b;
}

int ubx_frame(uint8_t *buf, size_t cap, uint8_t cls, uint8_t id,
	      const uint8_t *payload, size_t len)
{
	size_t total;
	uint8_t ck_a;
	uint8_t ck_b;

	if (buf == NULL) {
		return -EINVAL;
	}
	if ((payload == NULL) && (len != 0U)) {
		return -EINVAL;
	}
	if (len > UBX_PAYLOAD_MAX) {
		return -EMSGSIZE;
	}

	total = len + UBX_FRAME_OVERHEAD;
	if (cap < total) {
		return -ENOSPC;
	}

	buf[0] = (uint8_t)UBX_SYNC1;
	buf[1] = (uint8_t)UBX_SYNC2;
	buf[2] = cls;
	buf[3] = id;
	bytes_put_le16(&buf[4], (uint16_t)len);
	if (len != 0U) {
		(void)memcpy(&buf[6], payload, len);
	}

	/* Checksum covers class, id, length and payload — not the sync bytes. */
	ubx_checksum(&buf[2], len + 4U, &ck_a, &ck_b);
	buf[6U + len] = ck_a;
	buf[7U + len] = ck_b;

	return (int)total;
}

int ubx_poll(uint8_t *buf, size_t cap, uint8_t cls, uint8_t id)
{
	return ubx_frame(buf, cap, cls, id, NULL, 0U);
}

/* --------------------------------------------------------------- parser -- */

static void ubx_ck_acc(ubx_parser_t *p, uint8_t b)
{
	p->ck_a = (uint8_t)(p->ck_a + b);
	p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
}

void ubx_parser_reset(ubx_parser_t *p)
{
	if (p == NULL) {
		return;
	}
	p->state = (uint8_t)UBX_PS_SYNC1;
	p->cls = 0U;
	p->id = 0U;
	p->len = 0U;
	p->idx = 0U;
	p->ck_a = 0U;
	p->ck_b = 0U;
	p->rx_ck_a = 0U;
	p->complete = false;
}

int ubx_parser_init(ubx_parser_t *p, uint8_t *buf, uint16_t cap)
{
	if ((p == NULL) || (buf == NULL) || (cap == 0U)) {
		return -EINVAL;
	}

	p->buf = buf;
	p->cap = cap;
	p->stats.frames = 0U;
	p->stats.hunt_bytes = 0U;
	p->stats.len_errors = 0U;
	p->stats.ck_errors = 0U;
	ubx_parser_reset(p);
	return 0;
}

int ubx_parse_byte(ubx_parser_t *p, uint8_t b)
{
	unsigned int pass;
	int rc = 0;

	if ((p == NULL) || (p->buf == NULL)) {
		return -EINVAL;
	}

	if (p->complete) {
		/* The reported frame has been consumed; start the next one. */
		ubx_parser_reset(p);
	}

	/*
	 * At most two passes. Only a state that first rewinds to UBX_PS_SYNC1
	 * asks for the second, so the byte that broke a frame still gets its
	 * chance to be the sync1 of the next one, and the loop cannot spin.
	 */
	for (pass = 0U; pass < 2U; pass++) {
		bool again = false;

		switch ((ubx_pstate_t)p->state) {
		case UBX_PS_SYNC1:
			if (b == (uint8_t)UBX_SYNC1) {
				p->state = (uint8_t)UBX_PS_SYNC2;
			} else {
				p->stats.hunt_bytes++;
			}
			break;

		case UBX_PS_SYNC2:
			if (b == (uint8_t)UBX_SYNC2) {
				p->ck_a = 0U;
				p->ck_b = 0U;
				p->state = (uint8_t)UBX_PS_CLASS;
			} else {
				/* Not the pair; the byte may still open one. */
				p->stats.hunt_bytes++;
				p->state = (uint8_t)UBX_PS_SYNC1;
				again = true;
			}
			break;

		case UBX_PS_CLASS:
			p->cls = b;
			ubx_ck_acc(p, b);
			p->state = (uint8_t)UBX_PS_ID;
			break;

		case UBX_PS_ID:
			p->id = b;
			ubx_ck_acc(p, b);
			p->state = (uint8_t)UBX_PS_LEN_LO;
			break;

		case UBX_PS_LEN_LO:
			p->len = b;
			ubx_ck_acc(p, b);
			p->state = (uint8_t)UBX_PS_LEN_HI;
			break;

		case UBX_PS_LEN_HI:
			p->len = (uint16_t)(p->len | ((uint16_t)b << 8));
			ubx_ck_acc(p, b);
			if (p->len > p->cap) {
				p->stats.len_errors++;
				p->state = (uint8_t)UBX_PS_SYNC1;
				rc = -EMSGSIZE;
				again = true;
				break;
			}
			p->idx = 0U;
			p->state = (p->len == 0U) ? (uint8_t)UBX_PS_CK_A
						  : (uint8_t)UBX_PS_PAYLOAD;
			break;

		case UBX_PS_PAYLOAD:
			p->buf[p->idx] = b;
			p->idx++;
			ubx_ck_acc(p, b);
			if (p->idx >= p->len) {
				p->state = (uint8_t)UBX_PS_CK_A;
			}
			break;

		case UBX_PS_CK_A:
			p->rx_ck_a = b;
			p->state = (uint8_t)UBX_PS_CK_B;
			break;

		case UBX_PS_CK_B:
		default:
			if ((p->rx_ck_a == p->ck_a) && (b == p->ck_b)) {
				p->complete = true;
				p->stats.frames++;
				rc = 1;
			} else {
				p->stats.ck_errors++;
				p->state = (uint8_t)UBX_PS_SYNC1;
				rc = -EBADMSG;
				again = true;
			}
			break;
		}

		if (!again) {
			break;
		}
	}

	return rc;
}

int ubx_parser_msg(const ubx_parser_t *p, ubx_msg_t *out)
{
	if ((p == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	if (!p->complete) {
		return -EAGAIN;
	}

	out->cls = p->cls;
	out->id = p->id;
	out->len = p->len;
	out->payload = p->buf;
	return 0;
}

int ubx_parser_stats(const ubx_parser_t *p, ubx_stats_t *out)
{
	if ((p == NULL) || (out == NULL)) {
		return -EINVAL;
	}
	*out = p->stats;
	return 0;
}

bool ubx_msg_is(const ubx_msg_t *m, uint8_t cls, uint8_t id)
{
	return (m != NULL) && (m->cls == cls) && (m->id == id);
}

/* ------------------------------------------------------- typed decoders -- */

/* Shared preamble: identity and exact length. */
static int ubx_check_fixed(const ubx_msg_t *m, const void *out,
			   uint8_t cls, uint8_t id, uint16_t len)
{
	if ((m == NULL) || (out == NULL) || (m->payload == NULL)) {
		return -EINVAL;
	}
	if ((m->cls != cls) || (m->id != id)) {
		return -ENOMSG;
	}
	if (m->len != len) {
		return -EBADMSG;
	}
	return 0;
}

int ubx_parse_nav_pvt(const ubx_msg_t *m, ubx_nav_pvt_t *out)
{
	const uint8_t *p;
	int rc = ubx_check_fixed(m, out, UBX_CLASS_NAV, UBX_ID_NAV_PVT,
				 UBX_LEN_NAV_PVT);

	if (rc != 0) {
		return rc;
	}
	p = m->payload;

	out->itow_ms = bytes_get_le32(&p[0]);   /* iTOW  U4 */
	out->year = bytes_get_le16(&p[4]);      /* year  U2 */
	out->month = p[6];
	out->day = p[7];
	out->hour = p[8];
	out->min = p[9];
	out->sec = p[10];
	out->valid = p[11];                     /* valid X1 */
	out->valid_date = (out->valid & 0x01U) != 0U;
	out->valid_time = (out->valid & 0x02U) != 0U;
	out->fully_resolved = (out->valid & 0x04U) != 0U;
	out->valid_mag = (out->valid & 0x08U) != 0U;
	out->tacc_ns = bytes_get_le32(&p[12]);  /* tAcc  U4 */
	out->nano_ns = ubx_s32(bytes_get_le32(&p[16]));
	out->fix_type = p[20];
	out->flags = p[21];                     /* flags X1 */
	out->gnss_fix_ok = (out->flags & 0x01U) != 0U;
	out->diff_soln = (out->flags & 0x02U) != 0U;
	out->carr_soln = (uint8_t)((out->flags >> 6) & 0x03U);
	out->flags2 = p[22];
	out->num_sv = p[23];
	out->lon_1e7 = ubx_s32(bytes_get_le32(&p[24]));
	out->lat_1e7 = ubx_s32(bytes_get_le32(&p[28]));
	out->height_mm = ubx_s32(bytes_get_le32(&p[32]));
	out->hmsl_mm = ubx_s32(bytes_get_le32(&p[36]));
	out->hacc_mm = bytes_get_le32(&p[40]);
	out->vacc_mm = bytes_get_le32(&p[44]);
	/* 48..75 velocity/heading/accuracy — not decoded, see ubx.h. */
	out->pdop = bytes_get_le16(&p[76]);
	out->flags3 = bytes_get_le16(&p[78]);
	out->invalid_llh = (out->flags3 & 0x0001U) != 0U;

	return 0;
}

int ubx_nav_sat_begin(const ubx_msg_t *m, ubx_nav_sat_iter_t *it)
{
	const uint8_t *p;

	if ((m == NULL) || (it == NULL) || (m->payload == NULL)) {
		return -EINVAL;
	}
	if ((m->cls != UBX_CLASS_NAV) || (m->id != UBX_ID_NAV_SAT)) {
		return -ENOMSG;
	}
	if (m->len < UBX_NAV_SAT_HDR_LEN) {
		return -EBADMSG;
	}
	p = m->payload;

	/*
	 * Validate before arming. An iterator left half-populated by a failed
	 * begin() is a loaded gun: ubx_nav_sat_next() only checks rec != NULL,
	 * so a caller that ignores the return code would walk records that the
	 * length field says are not there.
	 */
	if (m->len != (uint16_t)(UBX_NAV_SAT_HDR_LEN +
				 ((uint32_t)p[5] * UBX_NAV_SAT_SV_LEN))) {
		return -EBADMSG;
	}

	it->itow_ms = bytes_get_le32(&p[0]);
	it->version = p[4];
	it->num_svs = p[5];
	it->idx = 0U;
	it->rec = &p[UBX_NAV_SAT_HDR_LEN];
	return 0;
}

int ubx_nav_sat_next(ubx_nav_sat_iter_t *it, ubx_nav_sat_sv_t *sv)
{
	const uint8_t *p;

	if ((it == NULL) || (sv == NULL) || (it->rec == NULL)) {
		return -EINVAL;
	}
	if (it->idx >= it->num_svs) {
		return 0;
	}
	p = it->rec;

	sv->gnss_id = p[0];
	sv->sv_id = p[1];
	sv->cno_dbhz = p[2];
	sv->elev_deg = ubx_s8(p[3]);
	sv->azim_deg = ubx_s16(bytes_get_le16(&p[4]));
	sv->pr_res = ubx_s16(bytes_get_le16(&p[6]));
	sv->flags = bytes_get_le32(&p[8]);
	sv->quality = (uint8_t)(sv->flags & 0x07U);
	sv->used = (sv->flags & 0x08U) != 0U;
	sv->health = (uint8_t)((sv->flags >> 4) & 0x03U);

	it->idx++;
	it->rec = &p[UBX_NAV_SAT_SV_LEN];
	return 1;
}

int ubx_parse_nav_timels(const ubx_msg_t *m, ubx_nav_timels_t *out)
{
	const uint8_t *p;
	int rc = ubx_check_fixed(m, out, UBX_CLASS_NAV, UBX_ID_NAV_TIMELS,
				 UBX_LEN_NAV_TIMELS);

	if (rc != 0) {
		return rc;
	}
	p = m->payload;

	out->itow_ms = bytes_get_le32(&p[0]);
	out->version = p[4];
	/* 5..7 reserved0 */
	out->src_of_curr_ls = p[8];
	out->curr_ls = ubx_s8(p[9]);
	out->src_of_ls_change = p[10];
	out->ls_change = ubx_s8(p[11]);
	out->time_to_ls_event_s = ubx_s32(bytes_get_le32(&p[12]));
	out->date_of_ls_gps_wn = bytes_get_le16(&p[16]);
	out->date_of_ls_gps_dn = bytes_get_le16(&p[18]);
	/* 20..22 reserved1 */
	out->valid = p[23];
	out->valid_curr_ls = (out->valid & 0x01U) != 0U;
	out->valid_time_to_ls_event = (out->valid & 0x02U) != 0U;

	return 0;
}

int ubx_parse_nav_svin(const ubx_msg_t *m, ubx_nav_svin_t *out)
{
	const uint8_t *p;
	int rc = ubx_check_fixed(m, out, UBX_CLASS_NAV, UBX_ID_NAV_SVIN,
				 UBX_LEN_NAV_SVIN);

	if (rc != 0) {
		return rc;
	}
	p = m->payload;

	out->version = p[0];
	/* 1..3 reserved0 */
	out->itow_ms = bytes_get_le32(&p[4]);
	out->dur_s = bytes_get_le32(&p[8]);
	out->mean_x_cm = ubx_s32(bytes_get_le32(&p[12]));
	out->mean_y_cm = ubx_s32(bytes_get_le32(&p[16]));
	out->mean_z_cm = ubx_s32(bytes_get_le32(&p[20]));
	out->mean_x_hp = ubx_s8(p[24]);
	out->mean_y_hp = ubx_s8(p[25]);
	out->mean_z_hp = ubx_s8(p[26]);
	/* 27 reserved1 */
	out->mean_acc_0p1mm = bytes_get_le32(&p[28]);
	out->obs = bytes_get_le32(&p[32]);
	out->valid = (p[36] != 0U);
	out->active = (p[37] != 0U);
	/* 38..39 reserved2 */

	return 0;
}

int ubx_parse_tim_tp(const ubx_msg_t *m, ubx_tim_tp_t *out)
{
	const uint8_t *p;
	int rc = ubx_check_fixed(m, out, UBX_CLASS_TIM, UBX_ID_TIM_TP,
				 UBX_LEN_TIM_TP);

	if (rc != 0) {
		return rc;
	}
	p = m->payload;

	out->tow_ms = bytes_get_le32(&p[0]);
	out->tow_sub_ms = bytes_get_le32(&p[4]);
	out->qerr_ps = ubx_s32(bytes_get_le32(&p[8]));
	out->week = bytes_get_le16(&p[12]);
	out->flags = p[14];
	out->time_base_utc = (out->flags & 0x01U) != 0U;
	out->utc_available = (out->flags & 0x02U) != 0U;
	out->raim = (uint8_t)((out->flags >> 2) & 0x03U);
	out->qerr_invalid = (out->flags & 0x10U) != 0U;
	out->ref_info = p[15];
	out->time_ref_gnss = (uint8_t)(out->ref_info & 0x0FU);
	out->utc_standard = (uint8_t)((out->ref_info >> 4) & 0x0FU);

	return 0;
}

int ubx_mon_rf_begin(const ubx_msg_t *m, ubx_mon_rf_iter_t *it)
{
	const uint8_t *p;

	if ((m == NULL) || (it == NULL) || (m->payload == NULL)) {
		return -EINVAL;
	}
	if ((m->cls != UBX_CLASS_MON) || (m->id != UBX_ID_MON_RF)) {
		return -ENOMSG;
	}
	if (m->len < UBX_MON_RF_HDR_LEN) {
		return -EBADMSG;
	}
	p = m->payload;

	/* Validate before arming — see ubx_nav_sat_begin(). */
	if (m->len != (uint16_t)(UBX_MON_RF_HDR_LEN +
				 ((uint32_t)p[1] * UBX_MON_RF_BLK_LEN))) {
		return -EBADMSG;
	}

	it->version = p[0];
	it->n_blocks = p[1];
	it->idx = 0U;
	it->rec = &p[UBX_MON_RF_HDR_LEN];
	return 0;
}

int ubx_mon_rf_next(ubx_mon_rf_iter_t *it, ubx_mon_rf_block_t *blk)
{
	const uint8_t *p;

	if ((it == NULL) || (blk == NULL) || (it->rec == NULL)) {
		return -EINVAL;
	}
	if (it->idx >= it->n_blocks) {
		return 0;
	}
	p = it->rec;

	blk->block_id = p[0];
	blk->flags = p[1];
	blk->jamming_state = (uint8_t)(blk->flags & 0x03U);
	blk->ant_status = p[2];
	blk->ant_power = p[3];
	blk->post_status = bytes_get_le32(&p[4]);
	/* 8..11 reserved1 */
	blk->noise_per_ms = bytes_get_le16(&p[12]);
	blk->agc_cnt = bytes_get_le16(&p[14]);
	blk->jam_ind = p[16];
	blk->ofs_i = ubx_s8(p[17]);
	blk->mag_i = p[18];
	blk->ofs_q = ubx_s8(p[19]);
	blk->mag_q = p[20];
	/* 21..23 reserved2 */

	it->idx++;
	it->rec = &p[UBX_MON_RF_BLK_LEN];
	return 1;
}

int ubx_parse_ack(const ubx_msg_t *m, ubx_ack_t *out)
{
	if ((m == NULL) || (out == NULL) || (m->payload == NULL)) {
		return -EINVAL;
	}
	if (m->cls != UBX_CLASS_ACK) {
		return -ENOMSG;
	}
	if ((m->id != UBX_ID_ACK_ACK) && (m->id != UBX_ID_ACK_NAK)) {
		return -ENOMSG;
	}
	if (m->len != UBX_LEN_ACK) {
		return -EBADMSG;
	}

	out->ack = (m->id == UBX_ID_ACK_ACK);
	out->cls_id = m->payload[0];
	out->msg_id = m->payload[1];
	return 0;
}

/* --------------------------------------------------------- CFG-VALSET -- */

int ubx_cfg_key_size_id(uint32_t key)
{
	uint32_t sz = (key >> 28) & 0x07U;

	if ((sz < 1U) || (sz > 5U)) {
		return -EINVAL;
	}
	return (int)sz;
}

int ubx_cfg_key_bytes(uint32_t key)
{
	static const uint8_t width[6] = {0U, 1U, 1U, 2U, 4U, 8U};
	int sz = ubx_cfg_key_size_id(key);

	if (sz < 0) {
		return sz;
	}
	return (int)width[sz];
}

uint16_t ubx_cfg_key_group(uint32_t key)
{
	return (uint16_t)((key >> 16) & 0x0FFFU);
}

int ubx_valset_begin(ubx_valset_t *v, uint8_t *buf, size_t cap, uint8_t layers)
{
	if ((v == NULL) || (buf == NULL)) {
		return -EINVAL;
	}
	if ((layers & (uint8_t)UBX_CFG_LAYER_ALL) == 0U) {
		return -EINVAL;
	}

	v->buf = buf;
	v->cap = cap;
	v->len = 0U;
	v->items = 0U;
	v->err = 0;
	v->open = false;

	/* Sync + header + VALSET prefix + the two checksum bytes at the end. */
	if (cap < (6U + UBX_VALSET_HDR_LEN + 2U)) {
		v->err = -ENOSPC;
		return -ENOSPC;
	}

	buf[0] = (uint8_t)UBX_SYNC1;
	buf[1] = (uint8_t)UBX_SYNC2;
	buf[2] = (uint8_t)UBX_CLASS_CFG;
	buf[3] = (uint8_t)UBX_ID_CFG_VALSET;
	bytes_put_le16(&buf[4], 0U); /* patched by ubx_valset_end() */
	buf[6] = 0U;                 /* version 0: no transaction */
	buf[7] = layers;
	buf[8] = 0U;                 /* reserved0 */
	buf[9] = 0U;

	v->len = 6U + UBX_VALSET_HDR_LEN;
	v->open = true;
	return 0;
}

/* Append one key/value pair; @p bytes is the value width the caller intends. */
static int valset_add(ubx_valset_t *v, uint32_t key, uint64_t val,
		      int want_bytes, int want_size_id)
{
	int key_bytes;
	size_t need;
	int i;

	if (v == NULL) {
		return -EINVAL;
	}
	if (v->err != 0) {
		return v->err; /* first failure wins and is re-reported */
	}
	if (!v->open) {
		v->err = -EINVAL;
		return -EINVAL;
	}

	key_bytes = ubx_cfg_key_bytes(key);
	if (key_bytes < 0) {
		v->err = -EINVAL;
		return -EINVAL;
	}
	if (key_bytes != want_bytes) {
		/* Key/setter width disagree — a wrong key or the wrong add_*. */
		v->err = -EINVAL;
		return -EINVAL;
	}
	if ((want_size_id > 0) && (ubx_cfg_key_size_id(key) != want_size_id)) {
		v->err = -EINVAL;
		return -EINVAL;
	}

	need = 4U + (size_t)want_bytes;
	if ((v->cap - v->len) < (need + 2U)) { /* +2 keeps room for the checksum */
		v->err = -ENOSPC;
		return -ENOSPC;
	}

	bytes_put_le32(&v->buf[v->len], key);
	v->len += 4U;
	for (i = 0; i < want_bytes; i++) {
		v->buf[v->len] = (uint8_t)((val >> (8 * i)) & 0xFFU);
		v->len++;
	}
	v->items++;
	return 0;
}

int ubx_valset_add_u1(ubx_valset_t *v, uint32_t key, uint8_t val)
{
	return valset_add(v, key, val, 1, 0);
}

int ubx_valset_add_u2(ubx_valset_t *v, uint32_t key, uint16_t val)
{
	return valset_add(v, key, val, 2, 0);
}

int ubx_valset_add_u4(ubx_valset_t *v, uint32_t key, uint32_t val)
{
	return valset_add(v, key, val, 4, 0);
}

int ubx_valset_add_u8(ubx_valset_t *v, uint32_t key, uint64_t val)
{
	return valset_add(v, key, val, 8, 0);
}

int ubx_valset_add_i1(ubx_valset_t *v, uint32_t key, int8_t val)
{
	return valset_add(v, key, (uint64_t)(uint8_t)val, 1, 0);
}

int ubx_valset_add_i2(ubx_valset_t *v, uint32_t key, int16_t val)
{
	return valset_add(v, key, (uint64_t)(uint16_t)val, 2, 0);
}

int ubx_valset_add_i4(ubx_valset_t *v, uint32_t key, int32_t val)
{
	return valset_add(v, key, (uint64_t)(uint32_t)val, 4, 0);
}

int ubx_valset_add_bool(ubx_valset_t *v, uint32_t key, bool val)
{
	return valset_add(v, key, val ? 1U : 0U, 1, 1);
}

int ubx_valset_end(ubx_valset_t *v)
{
	size_t payload;
	uint8_t ck_a;
	uint8_t ck_b;

	if (v == NULL) {
		return -EINVAL;
	}
	if (!v->open && (v->err == 0)) {
		return -EINVAL;
	}
	if (v->err != 0) {
		v->open = false;
		return v->err;
	}

	payload = v->len - 6U;
	bytes_put_le16(&v->buf[4], (uint16_t)payload);
	ubx_checksum(&v->buf[2], payload + 4U, &ck_a, &ck_b);
	v->buf[v->len] = ck_a;
	v->buf[v->len + 1U] = ck_b;
	v->len += 2U;
	v->open = false;

	return (int)v->len;
}
