/*
 * STS1000 "Meridian" — NMEA/UBX run classifier for the maintenance tees.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/platform/, and deliberately free of every Zephyr, MP
 * and logging dependency so tests/host can compile it — the same arrangement as
 * net/sts_ppscorr.h and console/sts_confirm_gate.h, for the same reason: what it
 * decides is invisible at runtime when it is wrong. A misclassified run does not
 * crash anything; it silently writes UBX frames into a technician's .nmea
 * capture, or splits a sentence across two channels, and the only symptom is a
 * decoder that will not parse a file nobody looks at until an outage.
 *
 * ---------------------------------------------------------------------------
 * What it classifies, and on what evidence
 * ---------------------------------------------------------------------------
 *
 * The FMT's channel 0x02 and 0x03 are "raw tees, capturable to .nmea / .ubx"
 * (spec §7.3/7.4), so the raw USART3 stream has to be split by which protocol
 * each byte belongs to. Running a second parser to find that out would double
 * the per-byte cost on a thread that feeds the discipline loop.
 *
 * It is not necessary. The UBX parser gnss.c already runs over every byte
 * publishes the one fact the split needs: whether it is hunting for the B5 62
 * sync (ubx_pstate_t UBX_PS_SYNC1) or is inside a frame. The caller passes that
 * in as @p hunting, sampled BEFORE ubx_parse_byte() consumes the byte. So:
 *
 *   - inside a sentence            -> NMEA, until and including the LF
 *   - hunting, and the byte is '$' -> starts an NMEA sentence
 *   - anything else                -> UBX
 *
 * The rule's one blind spot is deliberate and bounded: a '$' arriving one byte
 * into a false sync (state SYNC2, after a stray 0xB5) is classified UBX. That is
 * a byte of line noise landing in the raw UBX capture, which is where line noise
 * belongs.
 *
 * The as-built receiver is configured UBX-only (spec §3.7), so the NMEA side
 * normally stays silent — but a receiver freshly reset to factory defaults, which
 * is precisely when a technician opens the NMEA view, talks NMEA until the
 * configuration walk lands.
 *
 * ---------------------------------------------------------------------------
 * Batching
 * ---------------------------------------------------------------------------
 *
 * Bytes accumulate into a run and the sink is called a run at a time, never a
 * byte at a time: the sink's per-call cost is a lock acquisition, so per-byte
 * calls would multiply the expensive half by 64. A run is closed when the
 * channel changes, when it reaches GNSS_TEE_STAGE, or when the caller flushes at
 * the end of a drain pass. The sink is therefore always handed one contiguous
 * run belonging to exactly one channel.
 */

#ifndef STS1000_ZEPHYR_PLATFORM_GNSS_TEE_H_
#define STS1000_ZEPHYR_PLATFORM_GNSS_TEE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Bytes buffered before a run is handed to the sink.
 *
 * 64 balances the two costs: larger runs amortise the sink's lock better, and
 * smaller ones shorten the interval that lock masks interrupts for. It also
 * bounds this struct, which lives in BSS on the GNSS path.
 */
#define GNSS_TEE_STAGE 64U

/**
 * Receives one complete run.
 *
 * @param user  Caller's context.
 * @param nmea  true when the run is NMEA (channel 0x02), false for UBX (0x03).
 * @param data  Run bytes; valid only for the duration of the call.
 * @param len   1 .. GNSS_TEE_STAGE.
 */
typedef void (*gnss_tee_sink_fn)(void *user, bool nmea, const uint8_t *data,
				 size_t len);

typedef struct {
	gnss_tee_sink_fn sink;
	void *user;
	bool nmea;    /**< channel of the staged run; meaningful while len > 0 */
	bool in_nmea; /**< inside a '$'..LF sentence */
	uint8_t len;  /**< bytes staged, 0 .. GNSS_TEE_STAGE */
	uint8_t buf[GNSS_TEE_STAGE];
} gnss_tee_t;

/** Bind a sink and empty the classifier. */
static inline void gnss_tee_init(gnss_tee_t *t, gnss_tee_sink_fn sink,
				 void *user)
{
	if (t == NULL) {
		return;
	}
	t->sink = sink;
	t->user = user;
	t->nmea = false;
	t->in_nmea = false;
	t->len = 0U;
}

/** Hand any staged run to the sink. A no-op when nothing is staged. */
static inline void gnss_tee_flush(gnss_tee_t *t)
{
	if ((t == NULL) || (t->len == 0U)) {
		return;
	}
	if (t->sink != NULL) {
		t->sink(t->user, t->nmea, t->buf, (size_t)t->len);
	}
	t->len = 0U;
}

/**
 * Drop whatever is staged and forget the sentence state.
 *
 * For wherever the byte stream loses continuity — a receiver reset, a port
 * suspend/resume, a baud change. Deliberately does NOT flush: the staged bytes
 * belong to a session that no longer exists, and emitting them would splice two
 * captures together.
 */
static inline void gnss_tee_reset(gnss_tee_t *t)
{
	if (t == NULL) {
		return;
	}
	t->len = 0U;
	t->in_nmea = false;
}

/**
 * Classify and stage one received byte.
 *
 * @param hunting  The UBX parser's state BEFORE it consumes @p b: true when it
 *                 is hunting for the B5 62 sync, i.e. @p b is not inside a
 *                 frame.
 */
static inline void gnss_tee_byte(gnss_tee_t *t, uint8_t b, bool hunting)
{
	bool nmea;

	if (t == NULL) {
		return;
	}

	if (t->in_nmea) {
		nmea = true;
		if (b == (uint8_t)'\n') {
			t->in_nmea = false;
		}
	} else if (hunting && (b == (uint8_t)'$')) {
		t->in_nmea = true;
		nmea = true;
	} else {
		nmea = false;
	}

	/* A run belongs to one channel: close the old one before switching. */
	if ((t->len != 0U) && (t->nmea != nmea)) {
		gnss_tee_flush(t);
	}
	t->nmea = nmea;
	t->buf[t->len] = b;
	t->len++;
	if (t->len >= (uint8_t)GNSS_TEE_STAGE) {
		gnss_tee_flush(t);
	}
}

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_PLATFORM_GNSS_TEE_H_ */
