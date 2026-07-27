/*
 * STS1000 "Meridian" — core/mp: diagnostic registry and runner (FMT §8.2).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * See mp_diag.h for the core/glue split.
 */

#include "mp/mp_diag.h"

#include <errno.h>
#include <string.h>

/* ---------------------------------------------------------------- registry */

const mp_diag_test_t mp_diag_tests[MP_DIAG_COUNT] = {
	[MP_DIAG_I2C_SCAN] = { .name = "i2c.scan",
			       .guard = MP_GUARD_G1,
			       .steps = 1U,
			       .step_timeout_ms = 3000U,
			       .desc = "probe every 7-bit address on I2C1" },
	[MP_DIAG_INA_SELFTEST] = {
		.name = "ina.selftest",
		.guard = MP_GUARD_G1,
		.steps = 9U,
		.step_timeout_ms = 1000U,
		.desc = "per-rail ID, SHUNT_CAL and DIAG_ALRT check" },
	[MP_DIAG_PPS_SELFCHECK] = {
		.name = "pps.selfcheck",
		.guard = MP_GUARD_G1,
		.steps = 5U,
		.step_timeout_ms = 2000U,
		.desc = "PA0 vs PC6 agreement and qErr plausibility" },
	[MP_DIAG_FAN_SWEEP] = { .name = "fan.sweep",
				.guard = MP_GUARD_G2,
				.steps = 5U,
				.step_timeout_ms = 4000U,
				.ilk = MP_ILK_FAN_FLOOR,
				.desc = "duty ramp against tach response" },
	[MP_DIAG_UI_EXERCISE] = {
		.name = "ui.exercise",
		.guard = MP_GUARD_G1,
		.steps = 6U,
		.step_timeout_ms = 2000U,
		.desc = "lamp test, status RGB walk, backlight ramp" },
	[MP_DIAG_TOUCH_TARGET] = {
		.name = "touch.target",
		.guard = MP_GUARD_G1,
		.steps = 4U,
		.step_timeout_ms = 15000U,
		.desc = "prompt for four corner touches and report error" },
	[MP_DIAG_DISPLAY_PATTERN] = {
		.name = "display.pattern",
		.guard = MP_GUARD_G1,
		.steps = 4U,
		.step_timeout_ms = 3000U,
		.desc = "ST7796 colour bars, gradient and grid" },
	[MP_DIAG_RELAY_EXERCISE] = {
		.name = "relay.exercise",
		.guard = MP_GUARD_G2,
		.steps = 2U,
		.step_timeout_ms = 2000U,
		.ilk = MP_ILK_RELAY_OK,
		.disruptive = true,
		.desc = "K2 de-energize and re-energize; annunciates an alarm" },
	[MP_DIAG_GNSS_ANTENNA] = {
		.name = "gnss.antenna",
		.guard = MP_GUARD_G1,
		.steps = 3U,
		.step_timeout_ms = 3000U,
		.desc = "bias current band, MON-RF and GPS_ANT_OFF_MON fusion" },
	[MP_DIAG_NOR_VERIFY] = {
		.name = "nor.verify",
		.guard = MP_GUARD_G2,
		.steps = 3U,
		.step_timeout_ms = 5000U,
		.desc = "JEDEC id, scratch-sector write and read-back" },
	[MP_DIAG_SEC_ATTEST] = {
		.name = "sec.attest",
		.guard = MP_GUARD_G1,
		.steps = 1U,
		.step_timeout_ms = 3000U,
		.deferred = true,
		.desc = "ATECC608B device-key attestation (CryptoAuthLib binding deferred)" },
	[MP_DIAG_ETH_LOOPBACK] = {
		.name = "eth.loopback",
		.guard = MP_GUARD_G2,
		.steps = 2U,
		.step_timeout_ms = 5000U,
		.disruptive = true,
		.desc = "LAN8742 internal loopback; drops the link" },
	[MP_DIAG_WDT_TEST] = {
		.name = "wdt.test",
		.guard = MP_GUARD_G3,
		.steps = 1U,
		.step_timeout_ms = 5000U,
		.ilk = MP_ILK_WDT_LIVE,
		.disruptive = true,
		.desc = "withhold WDT_KICK: the board cold-cycles via POE_KILL" },
	[MP_DIAG_I2C_RECOVER] = {
		.name = "i2c.recover",
		.guard = MP_GUARD_G2,
		.steps = 2U,
		.step_timeout_ms = 2000U,
		.desc = "controller clock-out recovery of a wedged bus" },
};

const mp_diag_test_t *mp_diag_test(uint8_t id)
{
	return (id < (uint8_t)MP_DIAG_COUNT) ? &mp_diag_tests[id] : NULL;
}

int mp_diag_find(const char *name)
{
	size_t i;

	if (name == NULL) {
		return -EINVAL;
	}
	for (i = 0U; i < MP_DIAG_COUNT; i++) {
		if (strcmp(mp_diag_tests[i].name, name) == 0) {
			return (int)i;
		}
	}
	return -ENOENT;
}

/* ------------------------------------------------------------------- names */

static const char *const verdict_names[MP_DIAG_VERDICT_COUNT] = {
	"pass", "fail", "skip", "error",
};

const char *mp_diag_verdict_name(uint8_t v)
{
	return (v < (uint8_t)MP_DIAG_VERDICT_COUNT) ? verdict_names[v]
						    : "unknown";
}

static const char *const diag_ev_names[MP_DIAG_EV_COUNT] = {
	"start", "progress", "result", "abort",
};

const char *mp_diag_ev_name(uint8_t ev)
{
	return (ev < (uint8_t)MP_DIAG_EV_COUNT) ? diag_ev_names[ev] : "unknown";
}

/* -------------------------------------------------------------------- util */

static uint32_t since(uint32_t now, uint32_t then)
{
	return now - then;
}

static void emit(mp_diag_ctx_t *c, uint8_t ev, uint8_t step, uint8_t verdict,
		 int32_t value, const char *text)
{
	if (c->evt != NULL) {
		c->evt(c->evt_user, ev, c->test, step, verdict, value, text);
	}
}

/** Verdict ordering for "worst so far": PASS < SKIP < FAIL < ERROR. */
static uint8_t verdict_rank(uint8_t v)
{
	switch (v) {
	case MP_DIAG_PASS:
		return 0U;
	case MP_DIAG_SKIP:
		return 1U;
	case MP_DIAG_FAIL:
		return 2U;
	default:
		return 3U;
	}
}

/* --------------------------------------------------------------- lifecycle */

int mp_diag_init(mp_diag_ctx_t *c, mp_diag_action_fn action, void *action_user,
		 mp_diag_evt_fn evt, void *evt_user)
{
	if ((c == NULL) || (action == NULL)) {
		return -EINVAL;
	}
	(void)memset(c, 0, sizeof(*c));
	c->action = action;
	c->action_user = action_user;
	c->evt = evt;
	c->evt_user = evt_user;
	c->next_run_id = 1U;
	return 0;
}

int mp_diag_start(mp_diag_ctx_t *c, uint8_t test, uint32_t sid, uint32_t now_ms,
		  uint32_t *out_run_id)
{
	const mp_diag_test_t *t;

	if (c == NULL) {
		return -EINVAL;
	}
	t = mp_diag_test(test);
	if (t == NULL) {
		return -EINVAL;
	}
	if (c->running) {
		return -EBUSY;
	}
	if (t->deferred) {
		return -ENOTSUP;
	}

	c->running = true;
	c->test = test;
	c->step = 0U;
	c->steps = (t->steps != 0U) ? t->steps : 1U;
	c->worst = (uint8_t)MP_DIAG_PASS;
	c->fails = 0U;
	c->sid = sid;
	c->started_ms = now_ms;
	c->step_started_ms = now_ms;
	c->run_id = c->next_run_id;
	c->next_run_id++;
	if (c->next_run_id == 0U) {
		c->next_run_id = 1U;
	}
	c->runs++;

	if (out_run_id != NULL) {
		*out_run_id = c->run_id;
	}

	emit(c, (uint8_t)MP_DIAG_EV_START, 0U, (uint8_t)MP_DIAG_PASS,
	     (int32_t)c->steps, t->name);
	return 0;
}

/** Record a step verdict and advance. Returns true when the run is complete. */
static bool step_done(mp_diag_ctx_t *c, const mp_diag_step_res_t *res,
		      uint32_t now_ms)
{
	if (verdict_rank(res->verdict) > verdict_rank(c->worst)) {
		c->worst = res->verdict;
	}
	if ((res->verdict == (uint8_t)MP_DIAG_FAIL) ||
	    (res->verdict == (uint8_t)MP_DIAG_ERROR)) {
		c->fails++;
	}

	emit(c, (uint8_t)MP_DIAG_EV_PROGRESS, c->step, res->verdict, res->value,
	     (res->text[0] != '\0') ? res->text : NULL);

	c->step++;
	c->step_started_ms = now_ms;

	/*
	 * A failing step does not abort the sequence. A technician wants the
	 * whole picture — "the OCXO rail is fine, the Rb rail is not" — and
	 * stopping at the first failure would hide the second.
	 */
	return res->done || (c->step >= c->steps);
}

static void finish(mp_diag_ctx_t *c)
{
	c->running = false;
	c->completed++;
	emit(c, (uint8_t)MP_DIAG_EV_RESULT, c->step, c->worst,
	     (int32_t)c->fails, mp_diag_verdict_name(c->worst));
}

int mp_diag_step(mp_diag_ctx_t *c, uint32_t now_ms)
{
	const mp_diag_test_t *t;
	mp_diag_step_res_t res;
	int rc;

	if (c == NULL) {
		return -EINVAL;
	}
	if (!c->running) {
		return 0;
	}

	t = mp_diag_test(c->test);
	if (t == NULL) {
		/* Cannot happen: the test id was validated at start. Fail safe
		 * by ending the run rather than looping forever. */
		c->worst = (uint8_t)MP_DIAG_ERROR;
		finish(c);
		return 2;
	}

	(void)memset(&res, 0, sizeof(res));
	rc = c->action(c->action_user, c->test, c->step, &res);

	if (rc == -EAGAIN) {
		if (since(now_ms, c->step_started_ms) > t->step_timeout_ms) {
			c->timeouts++;
			res.verdict = (uint8_t)MP_DIAG_ERROR;
			res.value = (int32_t)since(now_ms, c->step_started_ms);
			(void)memcpy(res.text, "step timeout", 13U);
			if (step_done(c, &res, now_ms)) {
				finish(c);
				return 2;
			}
			return 1;
		}
		return 0;
	}

	if (rc == -ENOTSUP) {
		res.verdict = (uint8_t)MP_DIAG_SKIP;
		if (res.text[0] == '\0') {
			(void)memcpy(res.text, "not supported", 14U);
		}
	} else if (rc < 0) {
		res.verdict = (uint8_t)MP_DIAG_ERROR;
		res.value = rc;
		if (res.text[0] == '\0') {
			(void)memcpy(res.text, "action error", 13U);
		}
	} else if (res.verdict >= (uint8_t)MP_DIAG_VERDICT_COUNT) {
		/* A callback that returned 0 with a nonsense verdict is a bug in
		 * the glue; surface it rather than trusting the byte. */
		res.verdict = (uint8_t)MP_DIAG_ERROR;
		(void)memcpy(res.text, "bad verdict", 12U);
	}

	res.text[MP_DIAG_TEXT_MAX - 1U] = '\0';

	if (step_done(c, &res, now_ms)) {
		finish(c);
		return 2;
	}
	return 1;
}

int mp_diag_abort(mp_diag_ctx_t *c, uint32_t now_ms)
{
	(void)now_ms;

	if (c == NULL) {
		return -EINVAL;
	}
	if (!c->running) {
		return -ENOENT;
	}
	c->running = false;
	c->aborts++;
	emit(c, (uint8_t)MP_DIAG_EV_ABORT, c->step, (uint8_t)MP_DIAG_ERROR,
	     (int32_t)c->fails, "aborted");
	return 0;
}

bool mp_diag_busy(const mp_diag_ctx_t *c)
{
	return (c != NULL) && c->running;
}

uint16_t mp_diag_progress(const mp_diag_ctx_t *c)
{
	if ((c == NULL) || !c->running || (c->steps == 0U)) {
		return 0U;
	}
	return (uint16_t)(((uint32_t)c->step * 1000U) / (uint32_t)c->steps);
}
