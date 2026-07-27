/*
 * STS1000 "Meridian" — static application threads.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef STS1000_ZEPHYR_THREADS_THREADS_H_
#define STS1000_ZEPHYR_THREADS_THREADS_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create and start every static application thread.
 *
 * The thread set, priorities and roles are fixed by ARCHITECTURE.md §6. All
 * stacks are statically allocated; nothing here allocates at run time.
 *
 * Called once from the bring-up sequencer after the early hardware stages have
 * completed. Safe to call only once.
 *
 * @return Number of threads started.
 */
size_t sts1000_threads_start(void);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_THREADS_THREADS_H_ */
