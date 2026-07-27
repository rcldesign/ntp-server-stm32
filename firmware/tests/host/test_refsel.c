/*
 * STS1000 "Meridian" — core/refsel unit tests.
 *
 * The properties under test are spec §3.5's, stated as behaviour rather than
 * as API shape:
 *
 *   - engaging input B needs BOTH guards, continuously, past the hysteresis
 *     window (the truth table below covers all four guard combinations);
 *   - leaving input B happens on the SAME tick either guard drops, with no
 *     debounce whatsoever — this is the fail-safe direction and a test that
 *     allowed "within a second or two" would not be testing it;
 *   - a revert arms a lockout, so a source that is flapping cannot be
 *     re-engaged into the system clock every few seconds;
 *   - the emitted action list is the glitchless HSI-bridge sequence, in order,
 *     with the right mux target — writing MUX_SEL without bridging is the one
 *     thing docs/sts1000_clock_mux.md §4.2 says never to do.
 *
 * Times are milliseconds of the caller's monotonic clock, exactly as the
 * housekeeping thread supplies them.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "refsel/refsel.h"

/* ---------------------------------------------------------------- helpers */

static void in_good(refsel_in_t *in, uint64_t ms)
{
	memset(in, 0, sizeof(*in));
	in->mono_ms = ms;
	in->extref_edges_advancing = true;
	in->extref_freq_hz = 10000000u;
	in->extref_valid = true;
	in->rb_lock = true;
	in->request = REFSEL_REQ_AUTO;
}

/* Advance n seconds with a fixed input; returns the last output. */
static refsel_out_t run(refsel_ctx_t *ctx, refsel_in_t *in, uint64_t *ms,
			unsigned int n)
{
	refsel_out_t out;
	unsigned int i;

	memset(&out, 0, sizeof(out));
	for (i = 0u; i < n; i++) {
		*ms += 1000u;
		in->mono_ms = *ms;
		TEST_ASSERT_EQUAL_INT(0, refsel_step(ctx, in, &out));
	}
	return out;
}

/*
 * Step until a transition is decided and return that tick's output, so the
 * action list is still the one the transition produced. A plain run() of N
 * ticks would leave the list cleared by the quiet ticks that followed.
 */
static refsel_out_t run_until_transition(refsel_ctx_t *ctx, refsel_in_t *in,
					 uint64_t *ms, unsigned int max_ticks)
{
	refsel_out_t out;
	unsigned int i;

	memset(&out, 0, sizeof(out));
	for (i = 0u; i < max_ticks; i++) {
		*ms += 1000u;
		in->mono_ms = *ms;
		TEST_ASSERT_EQUAL_INT(0, refsel_step(ctx, in, &out));
		if (out.transition) {
			return out;
		}
	}
	TEST_FAIL_MESSAGE("no transition within the allotted ticks");
	return out;
}

/*
 * Assert the emitted list is exactly the glitchless sequence for @p mux, with
 * the discipline PARK/UNPARK bracket (MEDIUM-2). The bracket is what stops the
 * HSI-bridge clock discontinuity from tripping the discipline loop into
 * holdover on a healthy upgrade, so its presence and position are load-bearing.
 */
static void assert_handoff_sequence(const refsel_ctx_t *ctx, uint32_t mux,
				    uint32_t settle_ms)
{
	const refsel_step_t *s = NULL;
	size_t n = 0u;

	TEST_ASSERT_EQUAL_INT(0, refsel_actions(ctx, &s, &n));
	TEST_ASSERT_EQUAL_size_t(8u, n);

	/* Park before anything disturbs the clock; unpark only after the PLL is
	 * verified — never leave the loop steering into a free-wheeling clock. */
	TEST_ASSERT_EQUAL_INT(REFSEL_ACT_PARK_DISCIPLINE, s[0].act);
	TEST_ASSERT_EQUAL_INT(REFSEL_ACT_BRIDGE_TO_HSI, s[1].act);
	TEST_ASSERT_EQUAL_INT(REFSEL_ACT_SET_MUX, s[2].act);
	TEST_ASSERT_EQUAL_UINT32(mux, s[2].arg);
	TEST_ASSERT_EQUAL_INT(REFSEL_ACT_WAIT_SETTLE_MS, s[3].act);
	TEST_ASSERT_EQUAL_UINT32(settle_ms, s[3].arg);
	TEST_ASSERT_EQUAL_INT(REFSEL_ACT_RESELECT_HSE, s[4].act);
	TEST_ASSERT_EQUAL_INT(REFSEL_ACT_VERIFY_PLL, s[5].act);
	TEST_ASSERT_EQUAL_INT(REFSEL_ACT_UNPARK_DISCIPLINE, s[6].act);
	TEST_ASSERT_EQUAL_INT(REFSEL_ACT_DONE, s[7].act);
}

static void assert_no_actions(const refsel_ctx_t *ctx)
{
	const refsel_step_t *s = NULL;
	size_t n = 1u;

	TEST_ASSERT_EQUAL_INT(0, refsel_actions(ctx, &s, &n));
	TEST_ASSERT_EQUAL_size_t(0u, n);
}

/* Drive the machine to RB_ACTIVE, stopping on the transition tick so the
 * action list is still available to the caller. */
static void engage_rb(refsel_ctx_t *ctx, uint64_t *ms)
{
	refsel_in_t in;
	refsel_out_t out;

	in_good(&in, *ms);
	out = run_until_transition(ctx, &in, ms, 40u); /* > 30 s hysteresis */
	TEST_ASSERT_EQUAL_INT(REFSEL_RB_ACTIVE, out.state);
}

/* ------------------------------------------------------------------- setup */

static void test_defaults_and_init(void)
{
	refsel_cfg_t cfg;
	refsel_ctx_t ctx;

	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_cfg_defaults(NULL));
	TEST_ASSERT_EQUAL_INT(0, refsel_cfg_defaults(&cfg));

	TEST_ASSERT_EQUAL_UINT32(10000000u, cfg.extref_nominal_hz);
	TEST_ASSERT_EQUAL_UINT32(20u, cfg.extref_band_hz); /* +-2 ppm */
	TEST_ASSERT_EQUAL_UINT32(30000u, cfg.hysteresis_ms);
	TEST_ASSERT_EQUAL_UINT32(300000u, cfg.lockout_ms);
	TEST_ASSERT_FALSE(cfg.extref_require_rb_lock);

	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_init(NULL, &cfg));
	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, &cfg));
	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));

	/* R188 forces MUX_SEL low before firmware runs; the machine must agree
	 * with the hardware it is about to start commanding. */
	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, refsel_state(&ctx));
	assert_no_actions(&ctx);
}

static void test_init_rejects_bad_config(void)
{
	refsel_cfg_t cfg;
	refsel_ctx_t ctx;

	(void)refsel_cfg_defaults(&cfg);
	cfg.extref_nominal_hz = 0u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_init(&ctx, &cfg));

	(void)refsel_cfg_defaults(&cfg);
	cfg.extref_band_hz = 0u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_init(&ctx, &cfg));

	/* A band wider than the nominal frequency would accept a stopped
	 * clock as "in band". */
	(void)refsel_cfg_defaults(&cfg);
	cfg.extref_band_hz = 20000000u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_init(&ctx, &cfg));

	(void)refsel_cfg_defaults(&cfg);
	cfg.flap_alarm_n = 0u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_init(&ctx, &cfg));

	/* More reverts than the history can hold could never be counted. */
	(void)refsel_cfg_defaults(&cfg);
	cfg.flap_alarm_n = REFSEL_FLAP_HISTORY + 1u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_init(&ctx, &cfg));

	(void)refsel_cfg_defaults(&cfg);
	cfg.flap_window_ms = 0u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_init(&ctx, &cfg));

	/* L10: the timing parameters are bounded. settle_ms blocks the glue
	 * with SYSCLK on HSI, so an absurd value is rejected outright; the
	 * debounce windows are capped at the documented ceiling. */
	(void)refsel_cfg_defaults(&cfg);
	cfg.settle_ms = REFSEL_MAX_SETTLE_MS + 1u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_init(&ctx, &cfg));

	(void)refsel_cfg_defaults(&cfg);
	cfg.hysteresis_ms = REFSEL_MAX_WINDOW_MS + 1u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_init(&ctx, &cfg));

	(void)refsel_cfg_defaults(&cfg);
	cfg.lockout_ms = REFSEL_MAX_WINDOW_MS + 1u;
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_init(&ctx, &cfg));

	/* The maxima themselves are accepted. */
	(void)refsel_cfg_defaults(&cfg);
	cfg.settle_ms = REFSEL_MAX_SETTLE_MS;
	cfg.hysteresis_ms = REFSEL_MAX_WINDOW_MS;
	cfg.lockout_ms = REFSEL_MAX_WINDOW_MS;
	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, &cfg));
}

static void test_handoff_brackets_the_discipline_park(void)
{
	refsel_ctx_t ctx;
	uint64_t ms = 0u;
	const refsel_step_t *s = NULL;
	size_t n = 0u;

	/* MEDIUM-2: the glue must park the discipline loop across the whole
	 * clock disturbance, or a healthy OCXO->Rb upgrade would look like a
	 * multi-ms phase step to the loop and demote the server to holdover.
	 * PARK must be first (before the clock moves) and UNPARK must come
	 * after VERIFY_PLL (never leave the loop steering a free-wheeling
	 * clock). */
	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));
	engage_rb(&ctx, &ms);
	TEST_ASSERT_EQUAL_INT(0, refsel_actions(&ctx, &s, &n));

	TEST_ASSERT_EQUAL_size_t(8u, n);
	TEST_ASSERT_EQUAL_INT(REFSEL_ACT_PARK_DISCIPLINE, s[0].act);
	/* UNPARK sits strictly after VERIFY_PLL and strictly before DONE. */
	TEST_ASSERT_EQUAL_INT(REFSEL_ACT_VERIFY_PLL, s[5].act);
	TEST_ASSERT_EQUAL_INT(REFSEL_ACT_UNPARK_DISCIPLINE, s[6].act);
	TEST_ASSERT_EQUAL_INT(REFSEL_ACT_DONE, s[7].act);
}

static void test_api_rejects_null_and_uninitialised(void)
{
	refsel_ctx_t ctx;
	refsel_in_t in;
	refsel_out_t out;
	const refsel_step_t *s;
	size_t n;

	memset(&ctx, 0, sizeof(ctx)); /* initialised == false */
	in_good(&in, 1000u);
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_step(&ctx, &in, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_actions(&ctx, &s, &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_clear_flap(&ctx));
	TEST_ASSERT_EQUAL_INT(REFSEL_STATE__COUNT, refsel_state(&ctx));
	TEST_ASSERT_EQUAL_INT(REFSEL_STATE__COUNT, refsel_state(NULL));

	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_step(NULL, &in, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_step(&ctx, NULL, &out));
	/* NULL out is legal — a caller that only wants the actions. */
	TEST_ASSERT_EQUAL_INT(0, refsel_step(&ctx, &in, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_actions(NULL, &s, &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_actions(&ctx, NULL, &n));
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_actions(&ctx, &s, NULL));
	TEST_ASSERT_EQUAL_INT(-EINVAL, refsel_clear_flap(NULL));
}

/* --------------------------------------------------------- guard truth table */

/*
 * All four combinations of the two guards, each held well past the hysteresis
 * window. Only both-true may engage input B (§3.5).
 */
static void test_guard_truth_table(void)
{
	struct {
		bool extref;
		bool rb;
		refsel_state_t want;
	} cases[] = {
		{ false, false, REFSEL_OCXO_ACTIVE },
		{ false, true, REFSEL_OCXO_ACTIVE },
		{ true, false, REFSEL_OCXO_ACTIVE },
		{ true, true, REFSEL_RB_ACTIVE },
	};
	unsigned int i;

	for (i = 0u; i < 4u; i++) {
		refsel_ctx_t ctx;
		refsel_in_t in;
		refsel_out_t out;
		uint64_t ms = 0u;

		TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));
		in_good(&in, ms);
		in.extref_valid = cases[i].extref;
		in.extref_edges_advancing = cases[i].extref;
		in.rb_lock = cases[i].rb;

		out = run(&ctx, &in, &ms, 60u);
		TEST_ASSERT_EQUAL_INT(cases[i].want, out.state);
		TEST_ASSERT_EQUAL_INT(cases[i].want, refsel_state(&ctx));
	}
}

static void test_frequency_band_edges(void)
{
	struct {
		uint32_t hz;
		bool in_band;
	} cases[] = {
		{ 10000000u, true },  { 10000020u, true },  { 10000021u, false },
		{ 9999980u, true },   { 9999979u, false },  { 10000000u - 500u, false },
	};
	unsigned int i;

	/* The default band is +-20 Hz of 10 MHz, i.e. +-2 ppm. */
	for (i = 0u; i < 6u; i++) {
		refsel_ctx_t ctx;
		refsel_in_t in;
		refsel_out_t out;
		uint64_t ms = 0u;

		TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));
		in_good(&in, ms);
		in.extref_freq_hz = cases[i].hz;

		out = run(&ctx, &in, &ms, 60u);
		if (cases[i].in_band) {
			TEST_ASSERT_EQUAL_INT(REFSEL_RB_ACTIVE, out.state);
			TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_EXTREF_OK) !=
					 0u);
		} else {
			TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);
			TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_EXTREF_OK) ==
					 0u);
		}
	}
}

static void test_stopped_clock_is_never_in_band(void)
{
	refsel_ctx_t ctx;
	refsel_in_t in;
	refsel_out_t out;
	uint64_t ms = 0u;

	/* A dead source can still report its last measured frequency. Edges
	 * advancing is a separate, necessary condition — this is the exact
	 * Rb-stopped case the plain selector exists to survive. */
	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));
	in_good(&in, ms);
	in.extref_edges_advancing = false;
	out = run(&ctx, &in, &ms, 60u);
	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);

	/* Likewise a stale measurement. */
	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));
	ms = 0u;
	in_good(&in, ms);
	in.extref_valid = false;
	out = run(&ctx, &in, &ms, 60u);
	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);
}

/* ------------------------------------------------------------- hysteresis */

static void test_engage_waits_out_the_hysteresis_window(void)
{
	refsel_ctx_t ctx;
	refsel_in_t in;
	refsel_out_t out;
	uint64_t ms = 0u;
	unsigned int i;

	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));
	in_good(&in, ms);

	/* The window starts at the first tick with both guards good, so the
	 * transition lands on the tick 30 s later — not before. */
	for (i = 0u; i < 30u; i++) {
		ms += 1000u;
		in.mono_ms = ms;
		TEST_ASSERT_EQUAL_INT(0, refsel_step(&ctx, &in, &out));
		TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);
		TEST_ASSERT_FALSE(out.transition);
		assert_no_actions(&ctx);
	}

	ms += 1000u;
	in.mono_ms = ms;
	TEST_ASSERT_EQUAL_INT(0, refsel_step(&ctx, &in, &out));
	TEST_ASSERT_EQUAL_INT(REFSEL_RB_ACTIVE, out.state);
	TEST_ASSERT_TRUE(out.transition);
	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.from);
	assert_handoff_sequence(&ctx, REFSEL_MUX_B, 10u);
}

static void test_a_guard_blip_restarts_the_window(void)
{
	refsel_ctx_t ctx;
	refsel_in_t in;
	refsel_out_t out;
	uint64_t ms = 0u;

	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));
	in_good(&in, ms);

	/* 25 s of good, then one bad second: the window must start again from
	 * scratch, not resume. */
	(void)run(&ctx, &in, &ms, 25u);
	in.rb_lock = false;
	(void)run(&ctx, &in, &ms, 1u);
	in.rb_lock = true;
	out = run(&ctx, &in, &ms, 25u);
	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);

	out = run(&ctx, &in, &ms, 10u);
	TEST_ASSERT_EQUAL_INT(REFSEL_RB_ACTIVE, out.state);
}

/* ---------------------------------------------------------- fail-safe revert */

static void test_rb_lock_loss_reverts_on_the_same_tick(void)
{
	refsel_ctx_t ctx;
	refsel_in_t in;
	refsel_out_t out;
	uint64_t ms = 0u;

	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));
	engage_rb(&ctx, &ms);

	/* No debounce in the fail-safe direction: the very call that observes
	 * the drop must produce the handoff. */
	in_good(&in, ms);
	in.rb_lock = false;
	ms += 1000u;
	in.mono_ms = ms;
	TEST_ASSERT_EQUAL_INT(0, refsel_step(&ctx, &in, &out));

	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);
	TEST_ASSERT_TRUE(out.transition);
	TEST_ASSERT_EQUAL_INT(REFSEL_RB_ACTIVE, out.from);
	TEST_ASSERT_EQUAL_UINT16(1u, out.revert_count);
	TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_LOCKOUT) != 0u);
	assert_handoff_sequence(&ctx, REFSEL_MUX_A, 10u);
}

static void test_extref_loss_reverts_on_the_same_tick(void)
{
	refsel_ctx_t ctx;
	refsel_in_t in;
	refsel_out_t out;
	uint64_t ms = 0u;

	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));
	engage_rb(&ctx, &ms);

	/* Frequency wanders out of band while RB_LOCK is still asserted:
	 * either guard alone is enough to force the revert. */
	in_good(&in, ms);
	in.extref_freq_hz = 10000500u;
	ms += 1000u;
	in.mono_ms = ms;
	TEST_ASSERT_EQUAL_INT(0, refsel_step(&ctx, &in, &out));

	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);
	TEST_ASSERT_TRUE(out.transition);
	assert_handoff_sequence(&ctx, REFSEL_MUX_A, 10u);
}

static void test_lockout_blocks_re_engagement_then_expires(void)
{
	refsel_ctx_t ctx;
	refsel_in_t in;
	refsel_out_t out;
	uint64_t ms = 0u;

	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));
	engage_rb(&ctx, &ms);

	in_good(&in, ms);
	in.rb_lock = false;
	(void)run(&ctx, &in, &ms, 1u);
	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, refsel_state(&ctx));

	/* Guards come back immediately and stay good. The lockout must hold
	 * the machine off input B for the full 300 s even though the
	 * hysteresis window is long satisfied. */
	in.rb_lock = true;
	out = run(&ctx, &in, &ms, 100u);
	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);
	TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_LOCKOUT) != 0u);
	TEST_ASSERT_TRUE(out.lockout_remain_ms > 0u);
	TEST_ASSERT_TRUE(out.lockout_remain_ms <= 300000u);
	/* The guards themselves are reported stable — it is the lockout, not
	 * the guards, that is holding things up. */
	TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_GUARDS_STABLE) != 0u);

	out = run(&ctx, &in, &ms, 250u);
	TEST_ASSERT_EQUAL_INT(REFSEL_RB_ACTIVE, out.state);
	TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_LOCKOUT) == 0u);
}

static void test_flap_alarm_after_repeated_reverts(void)
{
	refsel_ctx_t ctx;
	refsel_cfg_t cfg;
	refsel_in_t in;
	refsel_out_t out;
	uint64_t ms = 0u;
	unsigned int cycle;

	/* Short lockout so three flap cycles fit in a readable test; the flap
	 * window stays at the default hour. */
	(void)refsel_cfg_defaults(&cfg);
	cfg.lockout_ms = 5000u;
	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, &cfg));

	for (cycle = 0u; cycle < 3u; cycle++) {
		in_good(&in, ms);
		out = run(&ctx, &in, &ms, 60u);
		TEST_ASSERT_EQUAL_INT(REFSEL_RB_ACTIVE, out.state);

		in.rb_lock = false;
		out = run(&ctx, &in, &ms, 1u);
		TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);
		TEST_ASSERT_EQUAL_UINT16((uint16_t)(cycle + 1u),
					 out.revert_count);

		if (cycle < 2u) {
			TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_FLAP_ALARM) ==
					 0u);
		}
	}

	TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_FLAP_ALARM) != 0u);

	/* Sticky until an operator acknowledges it. */
	in_good(&in, ms);
	out = run(&ctx, &in, &ms, 1u);
	TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_FLAP_ALARM) != 0u);

	TEST_ASSERT_EQUAL_INT(0, refsel_clear_flap(&ctx));
	out = run(&ctx, &in, &ms, 1u);
	TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_FLAP_ALARM) == 0u);
	TEST_ASSERT_EQUAL_UINT16(0u, out.revert_count);
}

static void test_reverts_outside_the_window_do_not_accumulate(void)
{
	refsel_ctx_t ctx;
	refsel_cfg_t cfg;
	refsel_in_t in;
	refsel_out_t out;
	uint64_t ms = 0u;
	unsigned int cycle;

	/* Same three reverts, but spread beyond the flap window: an
	 * occasional revert over a long deployment is not a flap. */
	(void)refsel_cfg_defaults(&cfg);
	cfg.lockout_ms = 5000u;
	cfg.flap_window_ms = 60000u;
	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, &cfg));

	for (cycle = 0u; cycle < 3u; cycle++) {
		in_good(&in, ms);
		out = run(&ctx, &in, &ms, 60u);
		TEST_ASSERT_EQUAL_INT(REFSEL_RB_ACTIVE, out.state);

		in.rb_lock = false;
		out = run(&ctx, &in, &ms, 1u);
		TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);
		TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_FLAP_ALARM) == 0u);
	}
	TEST_ASSERT_EQUAL_UINT16(3u, out.revert_count);
}

/* ------------------------------------------------------------- operator paths */

static void test_force_ocxo_reverts_without_penalising_the_source(void)
{
	refsel_ctx_t ctx;
	refsel_in_t in;
	refsel_out_t out;
	uint64_t ms = 0u;

	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));
	engage_rb(&ctx, &ms);

	in_good(&in, ms);
	in.request = REFSEL_REQ_FORCE_OCXO;
	ms += 1000u;
	in.mono_ms = ms;
	TEST_ASSERT_EQUAL_INT(0, refsel_step(&ctx, &in, &out));

	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);
	TEST_ASSERT_TRUE(out.transition);
	assert_handoff_sequence(&ctx, REFSEL_MUX_A, 10u);
	/* An operator changing their mind is not a fault: no lockout, no flap
	 * accounting. */
	TEST_ASSERT_EQUAL_UINT16(0u, out.revert_count);
	TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_LOCKOUT) == 0u);
	TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_FORCED_OCXO) != 0u);

	/* And it stays put however good the guards are. */
	out = run(&ctx, &in, &ms, 120u);
	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);

	/* Releasing the pin lets it re-engage after the usual window. */
	in.request = REFSEL_REQ_AUTO;
	out = run(&ctx, &in, &ms, 40u);
	TEST_ASSERT_EQUAL_INT(REFSEL_RB_ACTIVE, out.state);
}

static void test_extref_is_operator_only(void)
{
	refsel_ctx_t ctx;
	refsel_in_t in;
	refsel_out_t out;
	uint64_t ms = 0u;

	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));

	/* AUTO must never land in EXTREF_ACTIVE, however good input B looks. */
	in_good(&in, ms);
	out = run(&ctx, &in, &ms, 120u);
	TEST_ASSERT_EQUAL_INT(REFSEL_RB_ACTIVE, out.state);

	/* Asking for the house standard goes back through the OCXO first —
	 * the guard set is different, so input B has to re-qualify. */
	in.request = REFSEL_REQ_EXTREF;
	out = run(&ctx, &in, &ms, 1u);
	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);
	TEST_ASSERT_TRUE(out.transition);
	TEST_ASSERT_EQUAL_UINT16(0u, out.revert_count);
	assert_handoff_sequence(&ctx, REFSEL_MUX_A, 10u);

	out = run_until_transition(&ctx, &in, &ms, 40u);
	TEST_ASSERT_EQUAL_INT(REFSEL_EXTREF_ACTIVE, out.state);
	assert_handoff_sequence(&ctx, REFSEL_MUX_B, 10u);
}

static void test_extref_does_not_need_rb_lock_by_default(void)
{
	refsel_ctx_t ctx;
	refsel_in_t in;
	refsel_out_t out;
	uint64_t ms = 0u;

	/* A house standard presents no lock line. With the default config the
	 * band and edge checks are the whole guard. */
	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));
	in_good(&in, ms);
	in.request = REFSEL_REQ_EXTREF;
	in.rb_lock = false;

	out = run(&ctx, &in, &ms, 40u);
	TEST_ASSERT_EQUAL_INT(REFSEL_EXTREF_ACTIVE, out.state);

	/* The frequency guard still applies, and still reverts immediately. */
	in.extref_freq_hz = 10000100u;
	out = run(&ctx, &in, &ms, 1u);
	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);
	TEST_ASSERT_EQUAL_UINT16(1u, out.revert_count);
}

static void test_extref_can_be_configured_to_require_rb_lock(void)
{
	refsel_ctx_t ctx;
	refsel_cfg_t cfg;
	refsel_in_t in;
	refsel_out_t out;
	uint64_t ms = 0u;

	/* On a build where input B is always the FE-5680A, the operator path
	 * gets exactly the same guards as AUTO. */
	(void)refsel_cfg_defaults(&cfg);
	cfg.extref_require_rb_lock = true;
	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, &cfg));

	in_good(&in, ms);
	in.request = REFSEL_REQ_EXTREF;
	in.rb_lock = false;
	out = run(&ctx, &in, &ms, 60u);
	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);

	in.rb_lock = true;
	out = run(&ctx, &in, &ms, 40u);
	TEST_ASSERT_EQUAL_INT(REFSEL_EXTREF_ACTIVE, out.state);

	/* And losing RB_LOCK now reverts, where by default it would not. */
	in.rb_lock = false;
	out = run(&ctx, &in, &ms, 1u);
	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);
	TEST_ASSERT_EQUAL_UINT16(1u, out.revert_count);
}

/* ------------------------------------------------------------ failure path */

static void test_failed_handoff_falls_back_and_locks_out(void)
{
	refsel_ctx_t ctx;
	refsel_in_t in;
	refsel_out_t out;
	uint64_t ms = 0u;

	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));
	engage_rb(&ctx, &ms);

	/* The glue could not re-lock the PLL on input B (or CSS fired). Every
	 * guard still reads good — the machine must believe the glue anyway. */
	in_good(&in, ms);
	in.switch_failed = true;
	ms += 1000u;
	in.mono_ms = ms;
	TEST_ASSERT_EQUAL_INT(0, refsel_step(&ctx, &in, &out));

	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);
	TEST_ASSERT_TRUE(out.transition);
	assert_handoff_sequence(&ctx, REFSEL_MUX_A, 10u);
	TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_SWITCH_FAILED) != 0u);
	TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_LOCKOUT) != 0u);
	TEST_ASSERT_EQUAL_UINT16(1u, out.revert_count);

	/* The alarm is sticky past the failing input. */
	in.switch_failed = false;
	out = run(&ctx, &in, &ms, 10u);
	TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_SWITCH_FAILED) != 0u);
	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);

	TEST_ASSERT_EQUAL_INT(0, refsel_clear_flap(&ctx));
	out = run(&ctx, &in, &ms, 1u);
	TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_SWITCH_FAILED) == 0u);
}

static void test_failed_handoff_while_already_on_the_ocxo(void)
{
	refsel_ctx_t ctx;
	refsel_in_t in;
	refsel_out_t out;
	uint64_t ms = 0u;

	/* A failure reported when there is nothing to switch away from: no
	 * handoff to emit, but the retry must still be held off. */
	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));
	in_good(&in, ms);
	in.switch_failed = true;
	out = run(&ctx, &in, &ms, 1u);

	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);
	TEST_ASSERT_FALSE(out.transition);
	assert_no_actions(&ctx);
	TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_LOCKOUT) != 0u);
	TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_SWITCH_FAILED) != 0u);
}

/* -------------------------------------------------------------- misc/odd */

static void test_actions_are_cleared_on_a_quiet_step(void)
{
	refsel_ctx_t ctx;
	refsel_in_t in;
	uint64_t ms = 0u;

	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));
	engage_rb(&ctx, &ms);
	assert_handoff_sequence(&ctx, REFSEL_MUX_B, 10u);

	/* A step that decides nothing must not leave the previous list
	 * standing, or the glue would replay a handoff. */
	in_good(&in, ms);
	(void)run(&ctx, &in, &ms, 1u);
	assert_no_actions(&ctx);
}

static void test_backwards_clock_does_not_engage_early(void)
{
	refsel_ctx_t ctx;
	refsel_in_t in;
	refsel_out_t out;

	TEST_ASSERT_EQUAL_INT(0, refsel_init(&ctx, NULL));

	in_good(&in, 100000u);
	TEST_ASSERT_EQUAL_INT(0, refsel_step(&ctx, &in, &out));
	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);

	/* A timestamp older than the window start must read as "no time has
	 * passed", never as a huge elapsed interval. */
	in.mono_ms = 50000u;
	TEST_ASSERT_EQUAL_INT(0, refsel_step(&ctx, &in, &out));
	TEST_ASSERT_EQUAL_INT(REFSEL_OCXO_ACTIVE, out.state);
	TEST_ASSERT_TRUE((out.flags & REFSEL_FLAG_GUARDS_STABLE) == 0u);
}

static void test_names(void)
{
	TEST_ASSERT_EQUAL_STRING("ocxo", refsel_state_name(REFSEL_OCXO_ACTIVE));
	TEST_ASSERT_EQUAL_STRING("rb", refsel_state_name(REFSEL_RB_ACTIVE));
	TEST_ASSERT_EQUAL_STRING("extref",
				 refsel_state_name(REFSEL_EXTREF_ACTIVE));
	TEST_ASSERT_EQUAL_STRING("invalid", refsel_state_name(REFSEL_STATE__COUNT));

	TEST_ASSERT_EQUAL_STRING("none", refsel_action_name(REFSEL_ACT_NONE));
	TEST_ASSERT_EQUAL_STRING("park_discipline",
				 refsel_action_name(REFSEL_ACT_PARK_DISCIPLINE));
	TEST_ASSERT_EQUAL_STRING("bridge_to_hsi",
				 refsel_action_name(REFSEL_ACT_BRIDGE_TO_HSI));
	TEST_ASSERT_EQUAL_STRING("set_mux",
				 refsel_action_name(REFSEL_ACT_SET_MUX));
	TEST_ASSERT_EQUAL_STRING("wait_settle_ms",
				 refsel_action_name(REFSEL_ACT_WAIT_SETTLE_MS));
	TEST_ASSERT_EQUAL_STRING("reselect_hse",
				 refsel_action_name(REFSEL_ACT_RESELECT_HSE));
	TEST_ASSERT_EQUAL_STRING("verify_pll",
				 refsel_action_name(REFSEL_ACT_VERIFY_PLL));
	TEST_ASSERT_EQUAL_STRING("unpark_discipline",
				 refsel_action_name(REFSEL_ACT_UNPARK_DISCIPLINE));
	TEST_ASSERT_EQUAL_STRING("done", refsel_action_name(REFSEL_ACT_DONE));
	TEST_ASSERT_EQUAL_STRING("invalid",
				 refsel_action_name(REFSEL_ACT__COUNT));
}

int main(void)
{
	UNITY_BEGIN();

	RUN_TEST(test_defaults_and_init);
	RUN_TEST(test_init_rejects_bad_config);
	RUN_TEST(test_handoff_brackets_the_discipline_park);
	RUN_TEST(test_api_rejects_null_and_uninitialised);

	RUN_TEST(test_guard_truth_table);
	RUN_TEST(test_frequency_band_edges);
	RUN_TEST(test_stopped_clock_is_never_in_band);

	RUN_TEST(test_engage_waits_out_the_hysteresis_window);
	RUN_TEST(test_a_guard_blip_restarts_the_window);

	RUN_TEST(test_rb_lock_loss_reverts_on_the_same_tick);
	RUN_TEST(test_extref_loss_reverts_on_the_same_tick);
	RUN_TEST(test_lockout_blocks_re_engagement_then_expires);
	RUN_TEST(test_flap_alarm_after_repeated_reverts);
	RUN_TEST(test_reverts_outside_the_window_do_not_accumulate);

	RUN_TEST(test_force_ocxo_reverts_without_penalising_the_source);
	RUN_TEST(test_extref_is_operator_only);
	RUN_TEST(test_extref_does_not_need_rb_lock_by_default);
	RUN_TEST(test_extref_can_be_configured_to_require_rb_lock);

	RUN_TEST(test_failed_handoff_falls_back_and_locks_out);
	RUN_TEST(test_failed_handoff_while_already_on_the_ocxo);

	RUN_TEST(test_actions_are_cleared_on_a_quiet_step);
	RUN_TEST(test_backwards_clock_does_not_engage_early);
	RUN_TEST(test_names);

	return UNITY_END();
}
