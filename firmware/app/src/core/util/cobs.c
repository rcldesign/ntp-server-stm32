/*
 * STS1000 "Meridian" — core/util: Consistent Overhead Byte Stuffing.
 *
 * Cheshire & Baker, IEEE/ACM ToN 7(2), 1999. See cobs.h for the contract.
 *
 * Structure of an encoded body: a sequence of groups, each "code, data[code-1]".
 * A group with code < 0xFF stands for its data bytes followed by a zero; a group
 * with code == 0xFF stands for 254 data bytes and no zero. The zero implied by
 * the *last* group is not part of the frame — it is the phantom terminator — so
 * a decoder emits a group's implied zero only when another group follows.
 */

#include "util/cobs.h"

#include <errno.h>

/* Maximum data bytes carried by one group (code 0xFF). */
#define COBS_GROUP_MAX 254U

size_t cobs_encode_max(size_t len)
{
	return len + (len / COBS_GROUP_MAX) + 1U;
}

size_t cobs_decode_max(size_t enc_len)
{
	return (enc_len > 0U) ? (enc_len - 1U) : 0U;
}

int cobs_encode(const uint8_t *src, size_t src_len,
		uint8_t *dst, size_t dst_cap, size_t *out_len)
{
	size_t code_i = 0U; /* dst index reserved for the pending group code */
	size_t w = 1U;      /* next dst data write index */
	uint8_t code = 1U;  /* pending group code = 1 + data bytes in the group */
	bool pending = true;
	size_t i;

	if ((dst == NULL) || (out_len == NULL)) {
		return -EINVAL;
	}
	if ((src == NULL) && (src_len != 0U)) {
		return -EINVAL;
	}
	if (src_len > COBS_MAX_FRAME) {
		return -EMSGSIZE;
	}
	if (dst_cap < 1U) {
		return -ENOSPC;
	}

	for (i = 0U; i < src_len; i++) {
		if (src[i] != 0U) {
			if (w >= dst_cap) {
				return -ENOSPC;
			}
			dst[w++] = src[i];
			code++;
			if (code != 0xFFU) {
				continue;
			}
		}

		/*
		 * Close the group: either src[i] is the zero this code stands for,
		 * or the group just filled to its 254-byte maximum.
		 */
		dst[code_i] = code;
		code = 1U;

		if ((src[i] == 0U) || ((i + 1U) < src_len)) {
			/* Another group follows; reserve its code byte. */
			if (w >= dst_cap) {
				return -ENOSPC;
			}
			code_i = w++;
			pending = true;
		} else {
			/*
			 * A full 254-byte group ended exactly at the end of the
			 * input. Code 0xFF carries no implied zero, so nothing
			 * follows it — this is the case that makes a 254-byte
			 * non-zero run encode without a trailing 0x01 group.
			 */
			pending = false;
		}
	}

	if (pending) {
		dst[code_i] = code;
	}

	*out_len = w;
	return 0;
}

int cobs_decode(const uint8_t *src, size_t src_len,
		uint8_t *dst, size_t dst_cap, size_t *out_len)
{
	size_t r = 0U;
	size_t w = 0U;

	if ((src == NULL) || (out_len == NULL)) {
		return -EINVAL;
	}
	if ((dst == NULL) && (dst_cap != 0U)) {
		return -EINVAL;
	}
	if (src_len > COBS_MAX_FRAME_ENCODED) {
		return -EMSGSIZE;
	}
	if (src_len == 0U) {
		/* The shortest legal encoded body is the single byte 0x01. */
		return -EBADMSG;
	}

	while (r < src_len) {
		uint8_t code = src[r++];
		size_t n;
		size_t k;

		if (code == 0U) {
			/* The delimiter must never appear inside the body. */
			return -EBADMSG;
		}

		n = (size_t)code - 1U;
		if (n > (src_len - r)) {
			return -EBADMSG; /* group runs past the end of the input */
		}

		for (k = 0U; k < n; k++) {
			uint8_t b = src[r++];

			if (b == 0U) {
				return -EBADMSG; /* stuffed data is never zero */
			}
			if (w >= dst_cap) {
				return -ENOSPC;
			}
			dst[w++] = b;
		}

		if ((code != 0xFFU) && (r < src_len)) {
			/* Implied zero, emitted only because another group follows. */
			if (w >= dst_cap) {
				return -ENOSPC;
			}
			dst[w++] = 0U;
		}
	}

	*out_len = w;
	return 0;
}

/* ------------------------------------------------------------------------- */
/* Streaming decoder                                                          */
/* ------------------------------------------------------------------------- */

/* Append one decoded byte; latch -ENOSPC on overflow. */
static int dec_push(cobs_dec_t *d, uint8_t b)
{
	if (d->len >= d->cap) {
		d->err = -ENOSPC;
		return -ENOSPC;
	}
	d->buf[d->len++] = b;
	return 0;
}

void cobs_dec_reset(cobs_dec_t *d)
{
	if (d == NULL) {
		return;
	}
	d->len = 0U;
	d->left = 0U;
	d->zero_pending = false;
	d->started = false;
	d->complete = false;
	d->err = 0;
}

int cobs_dec_init(cobs_dec_t *d, uint8_t *buf, size_t cap)
{
	if (d == NULL) {
		return -EINVAL;
	}
	if ((buf == NULL) && (cap != 0U)) {
		return -EINVAL;
	}

	d->buf = buf;
	d->cap = cap;
	cobs_dec_reset(d);
	return 0;
}

int cobs_dec_byte(cobs_dec_t *d, uint8_t b)
{
	if (d == NULL) {
		return -EINVAL;
	}

	if (d->complete) {
		/* The reported frame has been consumed; begin the next one. */
		cobs_dec_reset(d);
	}

	if (b == 0U) {
		int rc;

		if (d->err != 0) {
			rc = d->err;
		} else if (!d->started) {
			/* Leading or repeated delimiter: a separator, not a frame. */
			return 0;
		} else if (d->left != 0U) {
			rc = -EBADMSG; /* the delimiter truncated a group */
		} else {
			/* The last group's implied zero is dropped by design. */
			d->complete = true;
			return 1;
		}

		cobs_dec_reset(d); /* resynchronise on the frame boundary */
		return rc;
	}

	if (d->err != 0) {
		return 0; /* swallow the remainder of the bad frame */
	}

	if (d->left == 0U) {
		/*
		 * Code byte. The previous group's implied zero is emitted here,
		 * now that another group is known to follow.
		 */
		if (d->zero_pending && (dec_push(d, 0U) != 0)) {
			return 0;
		}
		d->left = (size_t)b - 1U;
		d->zero_pending = (b != 0xFFU);
		d->started = true;
		return 0;
	}

	if (dec_push(d, b) != 0) {
		return 0;
	}
	d->left--;
	return 0;
}

size_t cobs_dec_len(const cobs_dec_t *d)
{
	return (d != NULL) ? d->len : 0U;
}

const uint8_t *cobs_dec_buf(const cobs_dec_t *d)
{
	return (d != NULL) ? d->buf : NULL;
}
