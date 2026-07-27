/*
 * STS1000 "Meridian" — the shared core/fault context.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * core/fault is a pure function of (f_idr, g_idr, t) plus an alarm table, and
 * it is written by two producers: the io_scan thread (1 kHz, the scan and the
 * scanned alarms) and everything else (software alarms 32+). A mutex — not a
 * spinlock — because a scan is a few microseconds of work over 32 signals and
 * priority inheritance is what keeps the 1 kHz cadence when a lower-priority
 * reader is mid-query.
 *
 * ISRs must not touch this. The PFI handler records a flag and lets the
 * housekeeping thread raise FAULT_ALARM_PFI.
 */

#include <zephyr/kernel.h>

#include "zephyr/platform/platform.h"

static fault_ctx_t sts_fault_ctx;
static K_MUTEX_DEFINE(sts_fault_mutex);

fault_ctx_t *sts_fault(void)
{
	return &sts_fault_ctx;
}

void sts_fault_lock(void)
{
	(void)k_mutex_lock(&sts_fault_mutex, K_FOREVER);
}

void sts_fault_unlock(void)
{
	(void)k_mutex_unlock(&sts_fault_mutex);
}

int sts_fault_init(void)
{
	return fault_init(&sts_fault_ctx, NULL);
}
