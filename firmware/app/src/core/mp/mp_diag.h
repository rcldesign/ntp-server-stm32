/*
 * STS1000 "Meridian" — core/mp: diagnostic registry and runner (FMT §8.2).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation; one run at a time.
 *
 * Core owns the registry, the sequencing, the per-step timeout and the progress
 * and result events; the glue supplies one action callback that performs step
 * `s` of test `t` against real hardware. That split is the whole design: a
 * diagnostic sequence is exactly the part worth unit-testing (does an aborted run
 * clean up? does a failing step still finish the sequence and report the worst
 * verdict? does a step that hangs time out?) and the part that cannot be tested
 * on a host is a single function pointer.
 *
 * A test's guard class is enforced by the caller through mp_ovr_guard() before
 * mp_diag_start(); the registry publishes the class so the tool can prompt for
 * the right confirmation first.
 */

#ifndef STS1000_CORE_MP_MP_DIAG_H_
#define STS1000_CORE_MP_MP_DIAG_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mp/mp_manifest.h"

#ifdef __cplusplus
extern "C" {
#endif

/** The registered diagnostics (FMT §8.2). */
typedef enum {
	MP_DIAG_I2C_SCAN = 0,    /**< enumerate the 15 devices on I2C1 */
	MP_DIAG_INA_SELFTEST,    /**< ID + SHUNT_CAL + DIAG_ALRT on all nine */
	MP_DIAG_PPS_SELFCHECK,   /**< PA0/PC6 capture agreement and qErr sanity */
	MP_DIAG_FAN_SWEEP,       /**< duty ramp vs tach response */
	MP_DIAG_UI_EXERCISE,     /**< lamp test + RGB + backlight walk */
	MP_DIAG_TOUCH_TARGET,    /**< prompt and read a touch coordinate */
	MP_DIAG_DISPLAY_PATTERN, /**< ST7796 test pattern */
	MP_DIAG_RELAY_EXERCISE,  /**< K2 de-energize/re-energize (service impact) */
	MP_DIAG_GNSS_ANTENNA,    /**< bias current bands + MON-RF + PD4 fusion */
	MP_DIAG_NOR_VERIFY,      /**< SPI-NOR JEDEC id + read-back of a scratch */
	MP_DIAG_SEC_ATTEST,      /**< ATECC608B attestation (deferred) */
	MP_DIAG_ETH_LOOPBACK,    /**< PHY internal loopback */
	MP_DIAG_WDT_TEST,        /**< deliberately miss a kick: reboots the board */
	MP_DIAG_I2C_RECOVER,     /**< clock-out recovery of a wedged bus */
	MP_DIAG_COUNT,
} mp_diag_id_t;

/** One registered test. */
typedef struct {
	const char *name; /**< dotted name, e.g. "i2c.scan" */
	uint8_t guard;    /**< mp_guard_t required to start it */
	uint8_t steps;    /**< number of steps; the progress denominator */
	uint32_t step_timeout_ms;
	uint32_t ilk;   /**< MP_ILK_* the caller must satisfy */
	bool deferred;  /**< advertised but answers -ENOTSUP on this build */
	bool disruptive;/**< interrupts served time or reboots the board */
	const char *desc;
} mp_diag_test_t;

extern const mp_diag_test_t mp_diag_tests[MP_DIAG_COUNT];

/** Entry for @p id, or NULL when out of range. */
const mp_diag_test_t *mp_diag_test(uint8_t id);

/** Look up a test by name. -ENOENT when unknown, -EINVAL for a NULL name. */
int mp_diag_find(const char *name);

/* ------------------------------------------------------------------ results */

typedef enum {
	MP_DIAG_PASS = 0,
	MP_DIAG_FAIL,
	MP_DIAG_SKIP,  /**< not applicable on this unit / not fitted */
	MP_DIAG_ERROR, /**< the step could not be performed */
	MP_DIAG_VERDICT_COUNT,
} mp_diag_verdict_t;

/** Stable verdict name. Never NULL. */
const char *mp_diag_verdict_name(uint8_t v);

/** Longest text a step result carries. */
#define MP_DIAG_TEXT_MAX 32U

typedef struct {
	uint8_t verdict; /**< mp_diag_verdict_t */
	int32_t value;   /**< step-specific measurement, 0 when meaningless */
	bool done;       /**< the test finished early; skip the remaining steps */
	char text[MP_DIAG_TEXT_MAX];
} mp_diag_step_res_t;

/**
 * Perform one step.
 *
 * @param test  mp_diag_id_t.
 * @param step  0-based step index.
 * @param out   Receives the step's verdict. Pre-zeroed by the runner.
 *
 * @retval 0         The step completed; @p out is valid.
 * @retval -EAGAIN   Still running; call again. The runner enforces the step
 *                   timeout, so a callback that never finishes is bounded.
 * @retval -ENOTSUP  Not wired on this build; the step is recorded as SKIP.
 * @retval <0        Any other error; the step is recorded as ERROR.
 */
typedef int (*mp_diag_action_fn)(void *user, uint8_t test, uint8_t step,
				 mp_diag_step_res_t *out);

/** Event kinds this module reports (carried on channel 0x09). */
typedef enum {
	MP_DIAG_EV_START = 0,
	MP_DIAG_EV_PROGRESS,
	MP_DIAG_EV_RESULT,
	MP_DIAG_EV_ABORT,
	MP_DIAG_EV_COUNT,
} mp_diag_ev_t;

/** Stable name for a diagnostic event. Never NULL. */
const char *mp_diag_ev_name(uint8_t ev);

/**
 * Progress / result sink.
 *
 * @param ev       mp_diag_ev_t.
 * @param test     mp_diag_id_t.
 * @param step     Step index (the completed one for PROGRESS).
 * @param verdict  mp_diag_verdict_t; the worst so far for RESULT.
 * @param value    Step measurement, or the failed-step count for RESULT.
 * @param text     Step text, or NULL.
 */
typedef void (*mp_diag_evt_fn)(void *user, uint8_t ev, uint8_t test,
			       uint8_t step, uint8_t verdict, int32_t value,
			       const char *text);

/* --------------------------------------------------------------------- ctx */

typedef struct {
	mp_diag_action_fn action;
	void *action_user;
	mp_diag_evt_fn evt;
	void *evt_user;

	bool running;
	uint8_t test;
	uint8_t step;
	uint8_t steps;
	uint8_t worst; /**< worst verdict seen in this run */
	uint8_t fails;
	uint32_t run_id;
	uint32_t sid; /**< session that started the run */
	uint32_t started_ms;
	uint32_t step_started_ms;

	/* counters */
	uint32_t runs;
	uint32_t completed;
	uint32_t aborts;
	uint32_t timeouts;
	uint32_t next_run_id;
} mp_diag_ctx_t;

/**
 * Initialise.
 *
 * @retval 0        Initialised.
 * @retval -EINVAL  @p c or @p action is NULL.
 */
int mp_diag_init(mp_diag_ctx_t *c, mp_diag_action_fn action, void *action_user,
		 mp_diag_evt_fn evt, void *evt_user);

/**
 * Start a run.
 *
 * @param out_run_id  Optional; receives the run id (never 0).
 *
 * @retval 0         Started; drive it with mp_diag_step().
 * @retval -EINVAL   Bad argument or unknown test.
 * @retval -EBUSY    A run is already in progress.
 * @retval -ENOTSUP  The test is deferred on this build.
 */
int mp_diag_start(mp_diag_ctx_t *c, uint8_t test, uint32_t sid,
		  uint32_t now_ms, uint32_t *out_run_id);

/**
 * Advance the current run by at most one step.
 *
 * @retval 1        A step completed and the run continues.
 * @retval 0        Nothing to do: either no run, or the step is still working.
 * @retval 2        The run finished (a RESULT event has been emitted).
 * @retval -EINVAL  @p c is NULL.
 */
int mp_diag_step(mp_diag_ctx_t *c, uint32_t now_ms);

/**
 * Abort the current run.
 *
 * @retval 0        Aborted.
 * @retval -EINVAL  @p c is NULL.
 * @retval -ENOENT  No run in progress.
 */
int mp_diag_abort(mp_diag_ctx_t *c, uint32_t now_ms);

/** True while a run is in progress. */
bool mp_diag_busy(const mp_diag_ctx_t *c);

/** Progress permille of the current run, 0..1000. 0 when idle. */
uint16_t mp_diag_progress(const mp_diag_ctx_t *c);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_MP_MP_DIAG_H_ */
