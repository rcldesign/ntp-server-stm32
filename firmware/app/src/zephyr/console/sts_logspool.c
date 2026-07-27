/*
 * STS1000 "Meridian" — the `logger` thread: log ring → NOR spool, plus the
 * Zephyr LOG → logring bridge.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * ARCHITECTURE.md §6 gives this thread priority 16, the lowest in the system,
 * and §5 says the ring lives in core/logring while "the NOR spool, the syslog
 * socket and the audit log are glue". Spec §12 sizes the ambition: a bounded,
 * oldest-evicted log ring on LittleFS that survives a reboot.
 *
 * The bridge, and why it exists
 * -----------------------------
 * Two log streams would otherwise never meet. Application code emits through
 * sts_log() into core/logring, which is what MCP LOG_TAIL and this spool read;
 * Zephyr drivers and subsystems emit through LOG_ERR/LOG_WRN into the logging
 * subsystem, which on this board reaches a USB console that is unplugged most
 * of the time. A PHY that never links or an I²C bus that wedges reports itself
 * only on the second stream, so a log backend forwards it into the first. The
 * forward is bounded (one record, truncated to LOGR_MSG_MAX) and drop-safe (a
 * re-entrant or overlapping call is counted, not queued).
 *
 * The spool file
 * --------------
 * /lfs/log/spool.bin is a fixed-size array of fixed-size records — a ring in a
 * file. Oldest-evicted comes free: writing slot (head % capacity) overwrites
 * whatever was there. No header is maintained, deliberately: a head pointer
 * written on every record would triple the flash traffic and would still be
 * stale after a power cut. Instead the head is recovered at mount by scanning
 * the file for the highest record sequence, which costs one sequential read of
 * the file once per boot on the lowest-priority thread.
 */

#include <zephyr/kernel.h>

#ifdef CONFIG_STS1000_CONSOLE

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_core.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "console/sts_console.h"
#include "logring/logring.h"
#include "storage/sts_store.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_logspool, CONFIG_STS1000_LOG_LEVEL);

/* --------------------------------------------------------------- tunables */

#if defined(CONFIG_STS1000_LOGGER_THREAD_STACK)
#define LOGGER_STACK CONFIG_STS1000_LOGGER_THREAD_STACK
#else
#define LOGGER_STACK 3072
#endif

#if defined(CONFIG_STS1000_LOG_SPOOL_RECORDS)
#define SPOOL_RECORDS CONFIG_STS1000_LOG_SPOOL_RECORDS
#else
#define SPOOL_RECORDS 1024
#endif

/** ARCHITECTURE.md §6: the `logger` thread runs at priority 16. */
#define LOGGER_PRIO 16
/** Drain cadence. Log latency is not a timing-path concern. */
#define LOGGER_PERIOD_MS 250
/** Records pulled from the ring per pass. */
#define LOGGER_BATCH 8
/** Passes between /lfs mount retries while the volume is missing. */
#define LOGGER_MOUNT_RETRY_PASSES 20

#define SPOOL_DIR  STS_FS_MOUNT_POINT "/log"
#define SPOOL_PATH SPOOL_DIR "/spool.bin"

/* ---------------------------------------------------------- record format */

/** ASCII "STSL" read as a little-endian u32. */
#define SPOOL_MAGIC 0x4C535453U
#define SPOOL_REC_SIZE 128U

struct spool_rec {
	uint32_t magic;
	uint32_t seq;
	uint64_t mono_ms;
	uint8_t  level;
	uint8_t  subsys;
	uint8_t  len;
	uint8_t  pad;
	char     msg[LOGR_MSG_MAX];
	uint8_t  rsvd[SPOOL_REC_SIZE - 20U - LOGR_MSG_MAX];
};

BUILD_ASSERT(sizeof(struct spool_rec) == SPOOL_REC_SIZE,
	     "spool record must stay exactly SPOOL_REC_SIZE bytes");
BUILD_ASSERT(LOGR_MSG_MAX <= 108U,
	     "LOGR_MSG_MAX no longer fits a 128 B spool record");

/* ------------------------------------------------------------------ state */

static struct k_thread logger_thread;
static K_THREAD_STACK_DEFINE(logger_stack, LOGGER_STACK);

static struct fs_file_t spool_file;
static bool spool_open;
static uint32_t spool_head;      /* slot the next record goes to */
static uint32_t spool_unsynced;

static sts_logspool_stats_t stats;
static uint32_t drain_cursor;
static int logger_liveness_id = -1;
static bool logger_started;

/* ------------------------------------------------------------------------- */
/* Zephyr LOG → logring bridge                                               */
/* ------------------------------------------------------------------------- */

static atomic_t bridge_busy = ATOMIC_INIT(0);
static bool bridge_armed;

static char bridge_text[LOGR_MSG_MAX];
static size_t bridge_text_n;
static uint8_t bridge_scratch[64];

static int bridge_out(uint8_t *data, size_t length, void *ctx)
{
	size_t room;

	ARG_UNUSED(ctx);

	room = sizeof(bridge_text) - bridge_text_n;
	if (room > 0U) {
		size_t n = MIN(room, length);

		memcpy(&bridge_text[bridge_text_n], data, n);
		bridge_text_n += n;
	}

	/* Always claim everything: log_output re-invokes until the buffer is
	 * drained, and refusing here would spin rather than truncate. */
	return (int)length;
}

LOG_OUTPUT_DEFINE(bridge_output, bridge_out, bridge_scratch,
		  sizeof(bridge_scratch));

/** Zephyr severity (LOG_LEVEL_ERR..DBG, 1..4) → RFC 5424 (logr_level_t). */
static uint8_t bridge_level(uint8_t zlevel)
{
	switch (zlevel) {
	case LOG_LEVEL_ERR:
		return (uint8_t)LOGR_ERR;
	case LOG_LEVEL_WRN:
		return (uint8_t)LOGR_WARN;
	case LOG_LEVEL_INF:
		return (uint8_t)LOGR_INFO;
	case LOG_LEVEL_DBG:
	default:
		return (uint8_t)LOGR_DEBUG;
	}
}

static void bridge_process(const struct log_backend *const backend,
			   union log_msg_generic *msg)
{
	uint8_t level;

	ARG_UNUSED(backend);

	if (!bridge_armed) {
		return;
	}

	/*
	 * One shared assembly buffer, so overlapping entries are dropped rather
	 * than interleaved. In LOG_MODE_DEFERRED this only ever runs on the log
	 * processing thread and the guard never trips; it exists so switching
	 * to immediate mode (where any thread, and an ISR, can land here) stays
	 * memory-safe, and so a hypothetical LOG_* call from inside sts_log()
	 * cannot recurse.
	 */
	if (!atomic_cas(&bridge_busy, 0, 1)) {
		stats.bridge_dropped++;
		return;
	}

	bridge_text_n = 0U;
	level = log_msg_get_level(&msg->log);

	/*
	 * flags = 0: no timestamp (the ring stamps its own), no colour, no
	 * level tag (carried structurally). The module name is still prefixed,
	 * which is the whole value of forwarding driver output.
	 */
	log_output_msg_process(&bridge_output, &msg->log, 0U);
	log_output_flush(&bridge_output);

	while ((bridge_text_n > 0U) &&
	       ((bridge_text[bridge_text_n - 1U] == '\n') ||
		(bridge_text[bridge_text_n - 1U] == '\r'))) {
		bridge_text_n--;
	}

	if (bridge_text_n > 0U) {
		sts_log((uint8_t)LOGR_SUB_SYS, bridge_level(level), "%.*s",
			(int)bridge_text_n, bridge_text);
		stats.bridge_in++;
	}

	atomic_set(&bridge_busy, 0);
}

static void bridge_dropped(const struct log_backend *const backend,
			   uint32_t cnt)
{
	ARG_UNUSED(backend);
	stats.bridge_dropped += cnt;
}

static void bridge_panic(const struct log_backend *const backend)
{
	ARG_UNUSED(backend);
	/* Nothing to flush: the ring is RAM and the spool is on a bus that is
	 * not safe to drive from a fault context. */
}

static void bridge_init(const struct log_backend *const backend)
{
	ARG_UNUSED(backend);
}

static const struct log_backend_api bridge_api = {
	.process = bridge_process,
	.dropped = bridge_dropped,
	.panic = bridge_panic,
	.init = bridge_init,
};

/* autostart = false: the ring is created by the platform area's early init,
 * and a backend enabled before that would forward into an empty logr_t. */
LOG_BACKEND_DEFINE(sts_log_bridge, bridge_api, false);

/* ------------------------------------------------------------------------- */
/* NOR spool                                                                 */
/* ------------------------------------------------------------------------- */

static int spool_scan_head(void)
{
	struct spool_rec rec;
	uint32_t best_seq = 0U;
	uint32_t best_slot = 0U;
	bool any = false;
	int rc;

	rc = fs_seek(&spool_file, 0, FS_SEEK_SET);
	if (rc != 0) {
		return rc;
	}

	for (uint32_t slot = 0U; slot < (uint32_t)SPOOL_RECORDS; slot++) {
		ssize_t n = fs_read(&spool_file, &rec, sizeof(rec));

		if (n != (ssize_t)sizeof(rec)) {
			break;
		}
		if (rec.magic != SPOOL_MAGIC) {
			continue;
		}
		/* Unsigned wrap-safe "newer than": sequence numbers are
		 * monotonic and never reset short of a 2^32 wrap. */
		if (!any || ((uint32_t)(rec.seq - best_seq) < 0x80000000U)) {
			best_seq = rec.seq;
			best_slot = slot;
			any = true;
		}
	}

	spool_head = any ? ((best_slot + 1U) % (uint32_t)SPOOL_RECORDS) : 0U;
	if (any) {
		LOG_INF("/lfs log spool resumed at slot %u (last seq %u)",
			spool_head, best_seq);
	}
	return 0;
}

static int spool_open_file(void)
{
	struct fs_dirent ent;
	int rc;

	if (spool_open) {
		return 0;
	}

	rc = fs_mkdir(SPOOL_DIR);
	if ((rc != 0) && (rc != -EEXIST)) {
		LOG_WRN("cannot create %s (%d)", SPOOL_DIR, rc);
		return rc;
	}

	fs_file_t_init(&spool_file);
	rc = fs_open(&spool_file, SPOOL_PATH, FS_O_CREATE | FS_O_RDWR);
	if (rc != 0) {
		LOG_WRN("cannot open %s (%d)", SPOOL_PATH, rc);
		return rc;
	}

	/*
	 * Preallocate the whole ring so a slot write is always an overwrite,
	 * never an extend: LittleFS then reuses the same blocks for the life of
	 * the file instead of growing it one record at a time.
	 */
	rc = fs_stat(SPOOL_PATH, &ent);
	if ((rc == 0) &&
	    (ent.size != (size_t)SPOOL_RECORDS * SPOOL_REC_SIZE)) {
		rc = fs_truncate(&spool_file,
				 (off_t)((size_t)SPOOL_RECORDS *
					 SPOOL_REC_SIZE));
		if (rc != 0) {
			LOG_WRN("cannot size %s to %u B (%d)", SPOOL_PATH,
				(unsigned int)((size_t)SPOOL_RECORDS *
					       SPOOL_REC_SIZE),
				rc);
			(void)fs_close(&spool_file);
			return rc;
		}
	}

	(void)spool_scan_head();

	spool_open = true;
	stats.spool_open = true;
	LOG_INF("%s open: %u records of %u B", SPOOL_PATH,
		(unsigned int)SPOOL_RECORDS, (unsigned int)SPOOL_REC_SIZE);
	return 0;
}

static int spool_write(const logr_rec_t *r)
{
	struct spool_rec rec;
	ssize_t n;
	int rc;

	memset(&rec, 0, sizeof(rec));
	rec.magic = SPOOL_MAGIC;
	rec.seq = r->seq;
	rec.mono_ms = r->mono_ms;
	rec.level = r->level;
	rec.subsys = r->subsys;
	rec.len = (uint8_t)MIN((size_t)r->len, sizeof(rec.msg));
	memcpy(rec.msg, r->msg, rec.len);

	rc = fs_seek(&spool_file, (off_t)(spool_head * SPOOL_REC_SIZE),
		     FS_SEEK_SET);
	if (rc != 0) {
		return rc;
	}

	n = fs_write(&spool_file, &rec, sizeof(rec));
	if (n != (ssize_t)sizeof(rec)) {
		return (n < 0) ? (int)n : -EIO;
	}

	spool_head = (spool_head + 1U) % (uint32_t)SPOOL_RECORDS;
	spool_unsynced++;
	return 0;
}

static void spool_close(void)
{
	if (!spool_open) {
		return;
	}
	(void)fs_close(&spool_file);
	spool_open = false;
	stats.spool_open = false;
}

/* ------------------------------------------------------------------------- */
/* Drain                                                                     */
/* ------------------------------------------------------------------------- */

static void drain_once(void)
{
	logr_rec_t batch[LOGGER_BATCH];
	logr_filter_t filter;
	uint16_t got = 0U;
	uint32_t next = drain_cursor;
	uint32_t gap = 0U;
	bool failed = false;

	logr_filter_all(&filter);

	/*
	 * INTEGRATION NOTE (platform area / Wave 3a): sts_log() serialises
	 * producers on a private spinlock that readers cannot take, so this
	 * tail — like core/mcp's LOG_TAIL handler — reads the ring
	 * unsynchronised. Indices are bounded and immutable after logr_init(),
	 * so the worst case is a torn message body in one record, never a bad
	 * access. Exposing a reader lock (or a locked sts_logring_tail()
	 * wrapper) through sts_app.h would close it; the batch is kept small
	 * to shorten the window meanwhile.
	 */
	if (logr_tail(sts_logring(), drain_cursor, &filter, batch,
		      (uint16_t)ARRAY_SIZE(batch), &got, &next, &gap) != 0) {
		return;
	}

	if (got == 0U) {
		drain_cursor = next;
		return;
	}

	stats.records += got;

	if (spool_open) {
		for (uint16_t i = 0U; i < got; i++) {
			if (spool_write(&batch[i]) != 0) {
				stats.spool_errors++;
				failed = true;
				break;
			}
			stats.spooled++;
		}
	}

	if (failed) {
		/* One bad write is a hiccup; a broken file is not worth
		 * retrying every 250 ms. Close and let the mount retry path
		 * decide whether the volume comes back. */
		LOG_WRN("log spool write failed; closing %s", SPOOL_PATH);
		spool_close();
	} else if (spool_open && (spool_unsynced >= (uint32_t)LOGGER_BATCH)) {
		if (fs_sync(&spool_file) == 0) {
			spool_unsynced = 0U;
		} else {
			stats.spool_errors++;
		}
	}

	drain_cursor = next;
	stats.cursor = next;
	sts_store_note_log_cursor(next);
}

/* ----------------------------------------------------------------- thread */

static void logger_thread_entry(void *p1, void *p2, void *p3)
{
	unsigned int since_mount_try = 0U;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("logger thread up (prio %d)", LOGGER_PRIO);

	for (;;) {
		k_sleep(K_MSEC(LOGGER_PERIOD_MS));

		if (!spool_open) {
			if (since_mount_try == 0U) {
				if (sts_fs_try_mount() == 0) {
					(void)spool_open_file();
				}
				since_mount_try = LOGGER_MOUNT_RETRY_PASSES;
			}
			since_mount_try--;
		}

		drain_once();

		if (logger_liveness_id >= 0) {
			sts_liveness_feed(logger_liveness_id);
		}
	}
}

/* ------------------------------------------------------------------- API */

void sts_logspool_stats(sts_logspool_stats_t *out)
{
	if (out != NULL) {
		*out = stats;
	}
}

int sts_logspool_start(void)
{
	k_tid_t tid;

	if (logger_started) {
		return 0;
	}
	logger_started = true;

	/* Start the drain from the oldest record the ring still holds, not
	 * from zero: a cursor behind the ring only produces a gap report. */
	drain_cursor = logr_oldest(sts_logring());
	stats.cursor = drain_cursor;

	bridge_armed = true;
	log_backend_enable(&sts_log_bridge, NULL, CONFIG_LOG_MAX_LEVEL);

	logger_liveness_id = sts_liveness_register("logger");
	if (logger_liveness_id < 0) {
		LOG_WRN("no liveness slot for the logger thread (%d)",
			logger_liveness_id);
	}

	tid = k_thread_create(&logger_thread, logger_stack,
			      K_THREAD_STACK_SIZEOF(logger_stack),
			      logger_thread_entry, NULL, NULL, NULL,
			      LOGGER_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(tid, "logger");

	return 0;
}

#endif /* CONFIG_STS1000_CONSOLE */
