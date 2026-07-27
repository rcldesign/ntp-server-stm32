/*
 * STS1000 "Meridian" — static application threads.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * The thread set and its priorities are fixed by ARCHITECTURE.md §6 (which in
 * turn follows software spec §1.2). Lower number = higher Zephyr priority.
 *
 * Wave 1a supplies the wiring only: every body is a paced idle loop that proves
 * the thread was created, runs at its priority and keeps its stack. The real
 * module logic is attached wave by wave, one entry point at a time, so the
 * scheduling shape of the system is testable before any of it exists.
 *
 * Not represented here:
 *   - `pps_capture` is an ISR (TIM2/TIM3 input capture) plus a k_work item, not
 *     a thread; it is created by the timing wave alongside the capture setup.
 *   - the Zephyr shell thread (CDC-ACM #0) and the log-processing thread are
 *     created by their own subsystems; ARCHITECTURE.md §6 lists them next to
 *     `mcp` and `logger` because they share those roles.
 */

#include "threads.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(sts1000_threads, CONFIG_STS1000_LOG_LEVEL);

/*
 * X-macro table: name, priority, stack bytes, loop period (ms), role.
 *
 * Priorities are the ARCHITECTURE.md §6 column verbatim — do not renumber them
 * here. Stack sizes are first estimates sized for the eventual work (protocol
 * buffers on the network threads, the render surface on the UI thread) and are
 * expected to be trimmed against CONFIG_INIT_STACKS measurements on hardware.
 */
#define STS1000_THREAD_TABLE(X)                                                                    \
	X(discipline, 4, 4096, 1000, "PPS-paced FLL/PI loop; sole DAC writer; publishes quality") \
	X(ptp, 5, 4096, 1000, "1588-2019 grandmaster: BMCA, Sync/Follow_Up/Delay")                \
	X(gnss, 6, 3072, 1000, "USART3 UBX parser driving the receiver lifecycle")                \
	X(ntp_server, 8, 4096, 1000, "NTP/NTS service on UDP 123 with MAC RX/TX timestamps")      \
	X(net_mgmt, 10, 2048, 1000, "DHCP, mDNS and link events")                                 \
	X(io_scan, 11, 2048, 1, "1 kHz GPIOF/GPIOG IDR scan feeding fault_scan_input()")          \
	X(snmp, 12, 3072, 1000, "SNMPv2c read-only agent and traps on UDP 161")                   \
	X(mcp, 14, 4096, 1000, "Meridian Console Protocol on CDC-ACM #1")                         \
	X(housekeeping, 14, 4096, 250, "I2C sensor sweep, pwrseq_step, thermal_step, WDT kick")   \
	X(ui_local, 15, 3072, 100, "Local TFT render and navigation state machine")               \
	X(logger, 16, 2048, 1000, "Drains the log ring to NOR spool and syslog")

/** Static description of one application thread. */
struct sts1000_thread_desc {
	const char *name;
	const char *role;
	k_thread_stack_t *stack;
	struct k_thread *tcb;
	size_t stack_size;
	int priority;
	uint32_t period_ms;
};

#define STS1000_THREAD_STORAGE(name_, prio_, stack_, period_, role_)                               \
	K_THREAD_STACK_DEFINE(name_##_thread_stack, stack_);                                       \
	static struct k_thread name_##_thread_tcb;

STS1000_THREAD_TABLE(STS1000_THREAD_STORAGE)

#define STS1000_THREAD_DESC(name_, prio_, stack_, period_, role_)                                  \
	{                                                                                          \
		.name = #name_,                                                                    \
		.role = role_,                                                                     \
		.stack = name_##_thread_stack,                                                     \
		.tcb = &name_##_thread_tcb,                                                        \
		.stack_size = K_THREAD_STACK_SIZEOF(name_##_thread_stack),                         \
		.priority = prio_,                                                                 \
		.period_ms = period_,                                                              \
	},

static const struct sts1000_thread_desc sts1000_threads[] = {
	STS1000_THREAD_TABLE(STS1000_THREAD_DESC)
};

BUILD_ASSERT(ARRAY_SIZE(sts1000_threads) == 11,
	     "ARCHITECTURE.md §6 defines 11 static application threads");

/*
 * Every priority in the table must be a valid preemptive priority, otherwise
 * k_thread_create() would silently clamp and the scheduling shape of the
 * system would differ from the one the spec was reasoned about.
 */
#define STS1000_THREAD_PRIO_ASSERT(name_, prio_, stack_, period_, role_)                           \
	BUILD_ASSERT((prio_) >= 0 && (prio_) < CONFIG_NUM_PREEMPT_PRIORITIES,                      \
		     "thread " #name_ " priority is outside the preemptive range; raise "          \
		     "CONFIG_NUM_PREEMPT_PRIORITIES");

STS1000_THREAD_TABLE(STS1000_THREAD_PRIO_ASSERT)

/** Emit a heartbeat roughly this often, whatever the thread's loop period is. */
#define STS1000_HEARTBEAT_MS 10000U

static void sts1000_placeholder_entry(void *p1, void *p2, void *p3)
{
	const struct sts1000_thread_desc *desc = p1;
	uint32_t ticks_per_heartbeat;
	uint32_t ticks = 0U;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	ticks_per_heartbeat = MAX(1U, STS1000_HEARTBEAT_MS / desc->period_ms);

	LOG_INF("thread '%s' up: prio %d, %u ms period, %zu B stack -- %s", desc->name,
		desc->priority, desc->period_ms, desc->stack_size, desc->role);

	for (;;) {
		k_sleep(K_MSEC(desc->period_ms));

		if (++ticks >= ticks_per_heartbeat) {
			ticks = 0U;
			LOG_DBG("thread '%s' alive", desc->name);
		}
	}
}

size_t sts1000_threads_start(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(sts1000_threads); i++) {
		const struct sts1000_thread_desc *desc = &sts1000_threads[i];
		k_tid_t tid;

		tid = k_thread_create(desc->tcb, desc->stack, desc->stack_size,
				      sts1000_placeholder_entry, (void *)desc, NULL, NULL,
				      desc->priority, 0, K_NO_WAIT);
		k_thread_name_set(tid, desc->name);
	}

	return ARRAY_SIZE(sts1000_threads);
}
