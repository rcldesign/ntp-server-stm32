/*
 * STS1000 "Meridian" — the platform-to-telemetry binding, end to end.
 *
 * WHAT DEFECT THIS SUITE EXISTS FOR
 *
 * mp_telem_t declared core/fault's debounced signal bitmap, the power
 * sequencer's stage, shed level and alarm word, the GNSS manager's state and
 * antenna verdict, and the survey progress. mp_stream.c encoded all of them.
 * core/fault, core/pwrseq and core/gnssmgr computed all of them. Nothing joined
 * the two, so every unit in the field reported an empty scan bitmap, stage 0,
 * shed 0, no alarms, receiver state 0 and no survey — for the whole life of the
 * record. tests/host/test_mp_stream.c passed throughout, because it filled
 * mp_telem_t by hand and round-tripped it: it proved the wire, and the defect
 * was upstream of the wire.
 *
 * So this suite starts from platform state and finishes at decoded CBOR:
 *
 *     real fault_ctx_t, driven by fault_scan_input()  --.
 *     real pwrseq_ctx_t, driven by pwrseq_step()        |
 *       -> pwrseq_status()                              |
 *       -> sts_pwrseq_snap_from_status() / _observe() <-'  (sts_pwrseq_pub.h)
 *       -> seqlock publish + read                       (sts_pwrseq_pub.h)
 *       -> sts_mp_telem_bind_pwrseq() / _bind_gnss()    (sts_mp_telem.h)
 *       -> mp_enc_telem()
 *       -> mp_cbor_rd_*                                 assert the values
 *
 * Every link in that chain is a place a field can be dropped in silence, and
 * the assertions are on the DECODED bytes, so dropping one anywhere fails here.
 *
 * It also pins the three planes' shared survey-accuracy rule against each
 * other. The web plane (sts_web_sky.h), this one and the panel (sts_ui_refs.h)
 * all answer "how accurate is the position in force", an operator compares that
 * number against `gnss.survey.acc`, and three copies of one rule is three
 * chances for it to fork.
 */

#include <errno.h>
#include <string.h>

#include "unity.h"

#include "fault/fault.h"
#include "gnssmgr/gnssmgr.h"
#include "ina228/ina228.h"
#include "mp/mp_stream.h"
#include "pwrseq/pwrseq.h"
#include "quality/quality.h"
#include "refsel/refsel.h"

#include "zephyr/console/sts_mp_telem.h"
#include "zephyr/net/sts_web_sky.h"
#include "zephyr/platform/sts_pwrseq_pub.h"
#include "zephyr/ui/sts_ui_refs.h"

static uint8_t g_buf[4096];

void setUp(void)
{
	memset(g_buf, 0, sizeof(g_buf));
}

/* ------------------------------------------------------------- CBOR reads */

/** Position a reader on key @p k of the record in @p buf. */
static void seek(mp_cbor_rd_t *r, size_t len, uint64_t k)
{
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_init(r, g_buf, len));
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_map_find(r, k));
}

static uint64_t get_u(size_t len, uint64_t k)
{
	mp_cbor_rd_t r;
	uint64_t v = 0U;

	seek(&r, len, k);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &v));
	return v;
}

/** Read the @p idx'th element of the @p n-element array at key @p k. */
static uint64_t get_arr_u(size_t len, uint64_t k, size_t n, size_t idx)
{
	mp_cbor_rd_t r;
	size_t cnt = 0U;
	uint64_t v = 0U;
	size_t i;

	seek(&r, len, k);
	TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_arr(&r, &cnt));
	TEST_ASSERT_EQUAL_size_t(n, cnt);
	for (i = 0U; i <= idx; i++) {
		TEST_ASSERT_EQUAL_INT(0, mp_cbor_rd_uint(&r, &v));
	}
	return v;
}

/* --------------------------------------------------- a real sequencer run */

/* hk.c's HK_PERIOD_MS: the rate pwrseq_step() actually runs at. */
#define HK_TICK_MS 250U

/**
 * A board where every observation is the happy one.
 *
 * Mirrors tests/host/test_pwrseq.c's in_healthy(); the point here is not to
 * model the board faithfully but to get the stage machine off IDLE by running
 * its real code, so the values this suite asserts on were computed rather than
 * poked into the struct.
 */
static void in_healthy(pwrseq_in_t *in)
{
	int i;

	memset(in, 0, sizeof(*in));
	in->nor_ready = true;
	in->i2c_probe_ok = true;
	in->cal_loaded = true;
	in->shunt_trims_applied = true;

	for (i = 0; i < (int)INA228_RAIL_COUNT; i++) {
		in->ina_valid[i] = true;
		in->ina_vbus_mv[i] = ina228_rail_tbl[i].nominal_mv;
		in->ina_current_ma[i] = 10;
		in->ina_age_ms[i] = 0U;
	}
	in->ina_vbus_mv[INA228_RAIL_VCC_RB] = 4515;
	in->ina_current_ma[INA228_RAIL_OCXO] = 300;

	in->pg_mask = 0xFFU;
	in->phy_refclk_stable = true;
	in->phy_id_ok = true;
	in->gnss_cfg_ack = true;
	in->ocxo_warm = true;
	in->ocxo_temp_stable = true;
	in->supercaps_charged = true;
	in->liveness_ok = true;
	in->ui_wanted = true;
	in->poe_granted_mw = 25500U;
	in->poe_measured_mw = 9000U;

	/*
	 * A rubidium IS configured, so stage 8 runs its guarded sequence rather
	 * than being skipped. This model supplies no digipot readback, so the
	 * sequence fails its verify and raises real alarm bits — which is the
	 * point: the alarm word this suite carries to the wire was computed by
	 * core/pwrseq, not assigned by the test.
	 */
	in->rb_wanted = true;
}

/**
 * Drive a sequencer into a state that is emphatically not the zero one.
 *
 * Deliberately built out of the module's own entry points — pwrseq_start(),
 * pwrseq_step(), pwrseq_shed_step(), pwrseq_ov_observe() — rather than by
 * assigning to pwrseq_ctx_t. A test that hand-set ctx->stage would still pass
 * against a pwrseq_status() that had stopped reading ctx->stage.
 */
static void drive_sequencer(pwrseq_ctx_t *ctx, pwrseq_in_t *in)
{
	uint32_t ms = 1000U;
	int i;

	TEST_ASSERT_EQUAL_INT(0, pwrseq_init(ctx, NULL));
	TEST_ASSERT_EQUAL_INT(0, pwrseq_start(ctx, ms));

	in_healthy(in);
	for (i = 0; i < 400; i++) {
		pwrseq_act_t act;

		ms += HK_TICK_MS;
		in->mono_ms = ms;
		(void)pwrseq_step(ctx, in);
		while (pwrseq_action_get(ctx, &act) == 0) {
			/* Drained, not executed: the action semantics are
			 * test_pwrseq.c's subject, not this suite's. */
		}
	}

	/*
	 * The 26 V over-voltage latch, raised the way the board raises it:
	 * RB_OV_DET goes high in the INPUT and pwrseq_step() latches it and
	 * sets PWRSEQ_ALARM_RB_OV. Calling pwrseq_ov_observe() directly latches
	 * but raises no alarm — which is what pwrseq.h says and is why the
	 * alarm word must be driven through the step.
	 */
	in->rb_lock = true;
	in->rb_ov_det = true;
	in->extref_in_band = true;
	for (i = 0; i < 4; i++) {
		pwrseq_act_t act;

		ms += HK_TICK_MS;
		in->mono_ms = ms;
		(void)pwrseq_step(ctx, in);
		while (pwrseq_action_get(ctx, &act) == 0) {
		}
	}
	TEST_ASSERT_TRUE(pwrseq_ov_latched(ctx));

	/*
	 * Two rungs of the PoE shed ladder, so `shed` is neither 0 nor the
	 * ladder's top — a binding that reported a boolean would pass at 0/1.
	 *
	 * Last, and deliberately after the steps above: pwrseq_step() runs
	 * pwrseq_shed_track(), and this model board is drawing 9 W against a
	 * 25.5 W grant, so a further step would see relief and restore a rung.
	 */
	ms += HK_TICK_MS;
	TEST_ASSERT_EQUAL_INT(0, pwrseq_shed_step(ctx, ms));
	ms += HK_TICK_MS;
	TEST_ASSERT_EQUAL_INT(0, pwrseq_shed_step(ctx, ms));
}

/* ------------------------------------------------------- a real 1 kHz scan */

/*
 * Three signals, chosen so the word that reaches the wire cannot be produced by
 * any of the ways this binding can be wrong.
 *
 *   BUTTON_3 (PF2)             GPIOF, low bits, button debounce class
 *   PG_OCXO_LDO (PG1)          GPIOG, PG class — a rail, not an input device
 *   INA_ALERT_VCC_RB (PG15)    GPIOG bit 15 => bitmap bit 31, the sign bit
 *
 * = 0x80020004. Bit 31 set catches a signed narrowing anywhere along the chain
 * (CBOR would emit a negative integer, and the decode asserts an unsigned).
 * Spanning both ports catches a 16-bit truncation, and the value is not
 * palindromic under a halfword swap (0x00048002 != 0x80020004) so a GPIOF/GPIOG
 * transposition fails rather than passing by symmetry.
 */
#define SCAN_SIG_A FAULT_SIG_BUTTON_3
#define SCAN_SIG_B FAULT_SIG_PG_OCXO_LDO
#define SCAN_SIG_C FAULT_SIG_INA_ALERT_VCC_RB

#define SCAN_EXPECT                                                            \
	(FAULT_SIG_BIT(SCAN_SIG_A) | FAULT_SIG_BIT(SCAN_SIG_B) |               \
	 FAULT_SIG_BIT(SCAN_SIG_C))

/**
 * Drive @p ctx with real GPIOF/GPIOG words until the debounce commits.
 *
 * Every one of the 32 signals is active LOW, so idle is all-ones and asserting
 * one CLEARS its bit — building the words the other way round would pass
 * against an inverted implementation. 60 scans at 1 kHz clears the slowest
 * class in play here (button, 25 ms) and stays well under long_press_ms (800),
 * so no repeat machinery runs.
 *
 * @return fault_state(), i.e. exactly what pwrseq_fault_snapshot() hands the
 *         sequencer pass on the board.
 */
static uint32_t drive_scan(fault_ctx_t *ctx)
{
	uint16_t f = 0xFFFFU;
	uint16_t g = 0xFFFFU;
	uint32_t ms;

	TEST_ASSERT_EQUAL_INT(0, fault_init(ctx, NULL));

	f = (uint16_t)(f & ~(uint16_t)(1U << (unsigned int)SCAN_SIG_A));
	g = (uint16_t)(g & ~(uint16_t)(1U << ((unsigned int)SCAN_SIG_B - 16U)));
	g = (uint16_t)(g & ~(uint16_t)(1U << ((unsigned int)SCAN_SIG_C - 16U)));

	for (ms = 0U; ms < 60U; ms++) {
		TEST_ASSERT_EQUAL_INT(0, fault_scan_input(ctx, f, g, ms));
	}

	/* The module really did commit them — otherwise the rest of the suite
	 * could pass against a bitmap that happened to be 0. */
	TEST_ASSERT_EQUAL_HEX32(SCAN_EXPECT, fault_state(ctx));
	return fault_state(ctx);
}

/** The receiver detail the gnss thread publishes, mid-survey. */
static void detail_surveying(sts_gnss_detail_t *d)
{
	memset(d, 0, sizeof(*d));
	d->mgr_state = (uint8_t)GNSSMGR_ST_SURVEY_IN;
	d->ant_state = (uint8_t)GNSSMGR_ANT_OPEN;
	d->svin_seen = true;
	d->svin_active = true;
	d->svin_dur_s = 1832U;
	d->svin_obs = 1832U;
	d->svin_acc_0p1mm = 4149U; /* 414.9 mm -> 415 mm */
	d->pos_valid = true;
	d->pos_acc_0p1mm = 90U;    /* the stored position, NOT in force */
}

/* ===================================================== the binding, e2e === */

/**
 * The regression proper: platform state in, decoded telemetry out.
 *
 * Every assertion below reads the ENCODED RECORD, so it fails if the value is
 * lost in the mapping, in the seqlock, in the binding or in the encoder. Each
 * one also asserts the field is non-zero, because zero is precisely what the
 * defect produced and an assertion that only compared against the source would
 * have passed while both were zero.
 */
static void test_pwrseq_and_gnss_reach_the_wire(void)
{
	pwrseq_ctx_t ctx;
	pwrseq_in_t in;
	pwrseq_status_t st;
	fault_ctx_t fctx;
	uint32_t scan;
	sts_pwrseq_pub_t pub;
	sts_pwrseq_snap_t snap;
	sts_pwrseq_snap_t got;
	sts_gnss_detail_t detail;
	mp_telem_t t;
	int len;
	size_t n;

	drive_sequencer(&ctx, &in);
	detail_surveying(&detail);
	scan = drive_scan(&fctx);

	/* The sequencer really did leave the zero state. Without this the rest
	 * of the suite could pass against a stage that happened to be 0. */
	TEST_ASSERT_EQUAL_INT(0, pwrseq_status(&ctx, &st));
	TEST_ASSERT_NOT_EQUAL(PWRSEQ_STAGE_IDLE, st.stage);
	TEST_ASSERT_EQUAL_UINT(PWRSEQ_SHED_PANEL_LED, st.shed);
	TEST_ASSERT_NOT_EQUAL(0U, st.alarms);
	TEST_ASSERT_TRUE(st.ov_latched);

	/*
	 * Publish exactly as pwrseq_exec.c's pwrseq_publish() does — including
	 * the fault bitmap, which that function receives from the same tick that
	 * built `in` rather than re-reading it. Passing the value the scan
	 * actually produced is what makes the key-45 assertion below a statement
	 * about the binding instead of about a literal.
	 */
	memset(&pub, 0, sizeof(pub));
	sts_pwrseq_snap_from_status(&snap, &st);
	sts_pwrseq_snap_observe(&snap, &in, 9999994U, true, scan);
	sts_pwrseq_pub_publish(&pub, &snap);

	/* ...and read it back exactly as sts_pwrseq_snapshot() does. */
	TEST_ASSERT_EQUAL_INT(0, sts_pwrseq_pub_read(&pub, &got));
	TEST_ASSERT_TRUE(got.started);
	TEST_ASSERT_EQUAL_HEX32(SCAN_EXPECT, got.scan_state);

	/* Bind exactly as prov_telem() does. */
	memset(&t, 0, sizeof(t));
	sts_mp_telem_bind_pwrseq(&t, &got, (uint8_t)QUALITY_REF_RB);
	sts_mp_telem_bind_gnss(&t, &detail);

	len = mp_enc_telem(&t, g_buf, sizeof(g_buf));
	TEST_ASSERT_TRUE(len > 0);
	n = (size_t)len;

	/* --- key 45: scan_state ------------------------------------------- */
	/*
	 * The whole point, asserted on the decoded bytes: the word core/fault
	 * committed from real GPIOF/GPIOG scans is the word on the wire. Bit 31
	 * is set, so this also pins that the value survives as an unsigned 32-bit
	 * quantity all the way through CBOR rather than arriving as -2147352572.
	 */
	TEST_ASSERT_NOT_EQUAL(0U, get_u(n, 45U));
	TEST_ASSERT_EQUAL_UINT64((uint64_t)SCAN_EXPECT, get_u(n, 45U));
	TEST_ASSERT_EQUAL_UINT64((uint64_t)fault_state(&fctx), get_u(n, 45U));

	/* --- key 46: pwrseq [stage, shed, alarms] ------------------------- */
	TEST_ASSERT_NOT_EQUAL(0U, get_arr_u(n, 46U, 3U, 0U));
	TEST_ASSERT_EQUAL_UINT64(st.stage, get_arr_u(n, 46U, 3U, 0U));
	TEST_ASSERT_EQUAL_UINT64(PWRSEQ_SHED_PANEL_LED, get_arr_u(n, 46U, 3U, 1U));
	TEST_ASSERT_NOT_EQUAL(0U, get_arr_u(n, 46U, 3U, 2U));
	TEST_ASSERT_EQUAL_UINT64(st.alarms, get_arr_u(n, 46U, 3U, 2U));

	/* --- key 47: gnss [state, ant] ------------------------------------ */
	TEST_ASSERT_EQUAL_UINT64(GNSSMGR_ST_SURVEY_IN, get_arr_u(n, 47U, 2U, 0U));
	TEST_ASSERT_EQUAL_UINT64(GNSSMGR_ANT_OPEN, get_arr_u(n, 47U, 2U, 1U));
	TEST_ASSERT_NOT_EQUAL(0U, get_arr_u(n, 47U, 2U, 0U));
	TEST_ASSERT_NOT_EQUAL(0U, get_arr_u(n, 47U, 2U, 1U));

	/* --- key 48: survey [dur_s, acc_mm] ------------------------------- */
	TEST_ASSERT_EQUAL_UINT64(1832U, get_arr_u(n, 48U, 2U, 0U));
	TEST_ASSERT_EQUAL_UINT64(415U, get_arr_u(n, 48U, 2U, 1U));

	/* --- key 49: refs bitmap, 50: extref_hz, 51: refsel_state --------- */
	/*
	 * bit0 rb_lock (PB13), bit1 rb_powered (RB_PWR_EN), bit2 extref_ok
	 * (EXTREF_MON in band), bit3 pfi.
	 *
	 * 0x05 is the interesting combination and not an accident of the model:
	 * the opto reports lock and EXTREF_MON is in band, but this board's
	 * guarded stage-8 run failed its digipot verify and so RB_PWR_EN was
	 * never left asserted. Under the OLD binding — rb_lock derived from
	 * active_ref — bit 0 would have tracked the SELECTION and bit 2 would
	 * have been clear because the selection is RB, not EXTREF. Both are now
	 * the hardware's own answer.
	 */
	TEST_ASSERT_EQUAL_UINT64(0x05U, get_u(n, 49U));
	TEST_ASSERT_TRUE(got.rb_lock_pin);
	TEST_ASSERT_FALSE(got.rb_enabled);
	TEST_ASSERT_EQUAL_UINT64(9999994U, get_u(n, 50U));
	TEST_ASSERT_EQUAL_UINT64(REFSEL_RB_ACTIVE, get_u(n, 51U));
}

/**
 * The defect's exact signature, asserted as a failure mode.
 *
 * Nothing published => every one of these fields must be zero, and the record
 * must still encode. This is the OLD behaviour, and pinning it is what makes
 * the test above meaningful: it proves the non-zero values came from the
 * platform state and not from the encoder inventing something.
 */
static void test_nothing_published_is_all_zero(void)
{
	mp_telem_t t;
	size_t n;
	int len;

	memset(&t, 0, sizeof(t));
	sts_mp_telem_bind_pwrseq(&t, NULL, (uint8_t)QUALITY_REF_NONE);
	sts_mp_telem_bind_gnss(&t, NULL);

	len = mp_enc_telem(&t, g_buf, sizeof(g_buf));
	TEST_ASSERT_TRUE(len > 0);
	n = (size_t)len;

	TEST_ASSERT_EQUAL_UINT64(0U, get_u(n, 45U));
	TEST_ASSERT_EQUAL_UINT64(0U, get_arr_u(n, 46U, 3U, 0U));
	TEST_ASSERT_EQUAL_UINT64(0U, get_arr_u(n, 46U, 3U, 2U));
	TEST_ASSERT_EQUAL_UINT64(0U, get_arr_u(n, 47U, 2U, 0U));
	TEST_ASSERT_EQUAL_UINT64(0U, get_arr_u(n, 48U, 2U, 0U));
	TEST_ASSERT_EQUAL_UINT64(0U, get_u(n, 49U));
	TEST_ASSERT_EQUAL_UINT64(0U, get_u(n, 50U));
	/* refsel_state is bound even with no sequencer: it comes from the
	 * quality block, and OCXO is refsel's own initial state. */
	TEST_ASSERT_EQUAL_UINT64(REFSEL_OCXO_ACTIVE, get_u(n, 51U));
}

/** A snapshot whose `started` is false must bind nothing, not bind zeros. */
static void test_unstarted_sequencer_binds_nothing(void)
{
	sts_pwrseq_snap_t snap;
	mp_telem_t t;

	memset(&snap, 0, sizeof(snap));
	snap.started = false;
	snap.stage = 9U; /* stale bytes must not leak through */
	snap.rb_lock_pin = true;
	snap.scan_state = 0xDEADBEEFU;

	memset(&t, 0, sizeof(t));
	sts_mp_telem_bind_pwrseq(&t, &snap, (uint8_t)QUALITY_REF_OCXO);

	TEST_ASSERT_EQUAL_UINT8(0U, t.pwrseq_stage);
	TEST_ASSERT_FALSE(t.rb_lock);
	/* A bitmap with no tick behind it is indistinguishable from "every
	 * signal clear", so it must not be published at all. */
	TEST_ASSERT_EQUAL_HEX32(0U, t.scan_state);
}

/* ============================================== scan_state vs the alarms == */

/**
 * Key 45 is the SIGNAL bitmap; keys 43/44 are the ALARM view. Not the same
 * evidence, and the difference is the diagnosis.
 *
 * fault_alarms() suppresses any scanned signal pwrseq has marked expected-off —
 * a rail firmware deliberately gated is not a fault, which is why an OCXO-only
 * unit does not sit red with its rubidium power-good low. fault_state() does
 * not: it is the level, whatever the reason.
 *
 * So on a unit with a deferred rubidium, the VCC_RB alert and the Rb PSU
 * power-good are ASSERTED in key 45 and ABSENT from key 43. A technician
 * needs both words to tell "off because we switched it off" from "off because
 * it failed" — and if key 45 were bound to the alarm mask instead, or to
 * anything derived from it, that distinction would be gone with nothing saying
 * so. This test fails if the two are ever made the same word.
 */
static void test_scan_state_is_the_signal_not_the_alarm_view(void)
{
	fault_ctx_t fctx;
	sts_pwrseq_snap_t snap;
	mp_telem_t t;
	uint32_t scan;
	uint64_t alarms;
	int len;
	size_t n;

	scan = drive_scan(&fctx);

	/* pwrseq gated the Rb rail: its INA alert is expected, not a fault. */
	TEST_ASSERT_EQUAL_INT(
		0, fault_set_expected_off(&fctx, SCAN_SIG_C, true));
	alarms = fault_alarms(&fctx);

	/* The premise, asserted rather than assumed: the two words differ, and
	 * they differ in exactly the expected-off bit. */
	TEST_ASSERT_TRUE((scan & FAULT_SIG_BIT(SCAN_SIG_C)) != 0U);
	TEST_ASSERT_TRUE((alarms & FAULT_ALARM_BIT(SCAN_SIG_C)) == 0U);
	TEST_ASSERT_TRUE((alarms & FAULT_ALARM_BIT(SCAN_SIG_B)) != 0U);

	/* Bind both halves the way prov_telem() does: key 43 straight from
	 * sts_alarms_active() (= fault_alarms), key 45 through the sequencer. */
	memset(&snap, 0, sizeof(snap));
	snap.started = true;
	snap.scan_state = scan;

	memset(&t, 0, sizeof(t));
	t.alarms = alarms;
	sts_mp_telem_bind_pwrseq(&t, &snap, (uint8_t)QUALITY_REF_OCXO);

	len = mp_enc_telem(&t, g_buf, sizeof(g_buf));
	TEST_ASSERT_TRUE(len > 0);
	n = (size_t)len;

	TEST_ASSERT_EQUAL_UINT64((uint64_t)scan, get_u(n, 45U));
	TEST_ASSERT_EQUAL_UINT64(alarms, get_u(n, 43U));
	TEST_ASSERT_NOT_EQUAL(get_u(n, 43U), get_u(n, 45U));
}

/* =================================================== RB_LOCK vs active_ref */

/**
 * The reading the old binding could not produce.
 *
 * prov_telem() used to report rb_lock = (active_ref == QUALITY_REF_RB). The
 * case that matters is a unit still SELECTING the rubidium whose FE has dropped
 * lock: the old form said "locked" — the one moment the field had to be right.
 * Both fields are asserted, because the pair is the diagnosis: still selected,
 * no longer locked.
 */
static void test_rb_lock_is_the_pin_not_the_selection(void)
{
	sts_pwrseq_snap_t snap;
	mp_telem_t t;

	memset(&snap, 0, sizeof(snap));
	snap.started = true;
	snap.rb_enabled = true;  /* rail still up */
	snap.rb_lock_pin = false; /* PB13 says the FE has lost lock */

	memset(&t, 0, sizeof(t));
	sts_mp_telem_bind_pwrseq(&t, &snap, (uint8_t)QUALITY_REF_RB);

	TEST_ASSERT_FALSE(t.rb_lock);
	TEST_ASSERT_TRUE(t.rb_powered);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)REFSEL_RB_ACTIVE, t.refsel_state);
}

/**
 * And the mirror case: a good house standard connected but not selected.
 *
 * The old binding reported extref_ok = (active_ref == QUALITY_REF_EXTREF),
 * i.e. false on every unit running on anything else, however good the external
 * reference was.
 */
static void test_extref_is_the_measurement_not_the_selection(void)
{
	sts_pwrseq_snap_t snap;
	mp_telem_t t;

	memset(&snap, 0, sizeof(snap));
	snap.started = true;
	snap.extref_valid = true;
	snap.extref_in_band = true;
	snap.extref_hz = 10000000U;

	memset(&t, 0, sizeof(t));
	sts_mp_telem_bind_pwrseq(&t, &snap, (uint8_t)QUALITY_REF_OCXO);

	TEST_ASSERT_TRUE(t.extref_ok);
	TEST_ASSERT_EQUAL_UINT32(10000000U, t.extref_hz);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)REFSEL_OCXO_ACTIVE, t.refsel_state);
}

/** Every active_ref value maps to exactly one refsel state. */
static void test_refsel_recovery_is_total(void)
{
	TEST_ASSERT_EQUAL_UINT8((uint8_t)REFSEL_OCXO_ACTIVE,
				sts_mp_telem_refsel((uint8_t)QUALITY_REF_NONE));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)REFSEL_OCXO_ACTIVE,
				sts_mp_telem_refsel((uint8_t)QUALITY_REF_OCXO));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)REFSEL_RB_ACTIVE,
				sts_mp_telem_refsel((uint8_t)QUALITY_REF_RB));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)REFSEL_EXTREF_ACTIVE,
				sts_mp_telem_refsel((uint8_t)QUALITY_REF_EXTREF));
	/* Anything unrecognised must not select a reference. */
	TEST_ASSERT_EQUAL_UINT8((uint8_t)REFSEL_OCXO_ACTIVE,
				sts_mp_telem_refsel(200U));
}

/* ================================================== the publication slot === */

/** A zeroed slot is a valid empty slot; no init call exists or is needed. */
static void test_pub_slot_starts_empty(void)
{
	sts_pwrseq_pub_t pub;
	sts_pwrseq_snap_t out;

	memset(&pub, 0, sizeof(pub));
	memset(&out, 0xA5, sizeof(out));

	TEST_ASSERT_EQUAL_INT(0, sts_pwrseq_pub_read(&pub, &out));
	TEST_ASSERT_FALSE(out.started);
	TEST_ASSERT_EQUAL_UINT8(0U, out.stage);
}

/** Every field survives publish -> read. A memcpy, but assert it anyway. */
static void test_pub_round_trip(void)
{
	sts_pwrseq_pub_t pub;
	sts_pwrseq_snap_t in;
	sts_pwrseq_snap_t out;

	memset(&pub, 0, sizeof(pub));
	memset(&in, 0, sizeof(in));
	in.started = true;
	in.stage = 8U;
	in.stage_entered_ms = 123456U;
	in.shed = (uint8_t)PWRSEQ_SHED_RB;
	in.alarms = 0xDEADU;
	in.rb_lock_pin = true;
	in.extref_hz = 9999999U;
	in.scan_state = 0x80020004U;
	in.tick_mono_ms = 777U;

	sts_pwrseq_pub_publish(&pub, &in);
	TEST_ASSERT_EQUAL_INT(0, sts_pwrseq_pub_read(&pub, &out));
	TEST_ASSERT_EQUAL_MEMORY(&in, &out, sizeof(in));
	/* EQUAL_MEMORY would also pass if the field were absent from the struct;
	 * naming it is what makes its removal a failure here. */
	TEST_ASSERT_EQUAL_HEX32(0x80020004U, out.scan_state);
}

/**
 * A reader that lands mid-write refuses rather than returning a torn struct.
 *
 * The odd sequence is set directly, which is the state sts_pwrseq_pub_publish()
 * leaves the slot in between its two stores. Reaching it on hardware needs the
 * reader to be preempted inside the publisher's few microseconds, eight ticks
 * running; setting the word is how that gets tested at all.
 */
static void test_pub_read_refuses_a_write_in_progress(void)
{
	sts_pwrseq_pub_t pub;
	sts_pwrseq_snap_t out;

	memset(&pub, 0, sizeof(pub));
	pub.slot.started = true;
	pub.slot.stage = 8U;
	atomic_store_explicit(&pub.seq, 1u, memory_order_relaxed);

	memset(&out, 0xA5, sizeof(out));
	TEST_ASSERT_EQUAL_INT(-EAGAIN, sts_pwrseq_pub_read(&pub, &out));
	/* Zeroed, not left holding the 0xA5 fill and not holding a torn copy. */
	TEST_ASSERT_FALSE(out.started);
	TEST_ASSERT_EQUAL_UINT8(0U, out.stage);
}

static void test_pub_null_arguments(void)
{
	sts_pwrseq_pub_t pub;
	sts_pwrseq_snap_t out;

	memset(&pub, 0, sizeof(pub));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sts_pwrseq_pub_read(NULL, &out));
	TEST_ASSERT_EQUAL_INT(-EINVAL, sts_pwrseq_pub_read(&pub, NULL));
	/* Publish must tolerate them silently; it has no way to report. */
	sts_pwrseq_pub_publish(NULL, &out);
	sts_pwrseq_pub_publish(&pub, NULL);
	sts_pwrseq_snap_from_status(NULL, NULL);
	sts_pwrseq_snap_observe(NULL, NULL, 0U, false, 0U);
}

/** A failed status read must not leave the previous tick's beliefs behind. */
static void test_snap_from_status_clears_first(void)
{
	sts_pwrseq_snap_t s;

	memset(&s, 0xFF, sizeof(s));
	sts_pwrseq_snap_from_status(&s, NULL);
	TEST_ASSERT_FALSE(s.started);
	TEST_ASSERT_EQUAL_UINT8(0U, s.stage);
	TEST_ASSERT_EQUAL_UINT32(0U, s.alarms);
	TEST_ASSERT_EQUAL_HEX32(0U, s.scan_state);
}

/** ...and the observation half must be additive, not clearing. */
static void test_snap_observe_is_additive(void)
{
	pwrseq_status_t st;
	pwrseq_in_t in;
	sts_pwrseq_snap_t s;

	memset(&st, 0, sizeof(st));
	st.stage = (pwrseq_stage_t)PWRSEQ_STAGE_9_ARM;
	st.alarms = 0x40U;

	memset(&in, 0, sizeof(in));
	in.rb_lock = true;
	in.extref_in_band = true;
	in.mono_ms = 4242U;

	sts_pwrseq_snap_from_status(&s, &st);
	sts_pwrseq_snap_observe(&s, &in, 10000001U, true, 0x80020004U);

	TEST_ASSERT_EQUAL_UINT8((uint8_t)PWRSEQ_STAGE_9_ARM, s.stage);
	TEST_ASSERT_EQUAL_UINT32(0x40U, s.alarms);
	TEST_ASSERT_TRUE(s.rb_lock_pin);
	TEST_ASSERT_EQUAL_UINT32(10000001U, s.extref_hz);
	TEST_ASSERT_EQUAL_HEX32(0x80020004U, s.scan_state);
	TEST_ASSERT_EQUAL_UINT32(4242U, s.tick_mono_ms);
}

/**
 * The observed half is timestamped AS A SET, so a NULL input must leave all of
 * it alone — including the bitmap, which is the one member whose value reaches
 * this function from a source other than @p in.
 *
 * This is the contract sts_pwrseq_start()'s pwrseq_publish(NULL, 0, false, 0)
 * relies on: a bitmap published against tick_mono_ms == 0 would be a word a
 * reader cannot date, sitting beside a pg_mask and an EXTREF reading that are
 * genuinely absent.
 */
static void test_snap_observe_ignores_scan_state_without_a_tick(void)
{
	pwrseq_status_t st;
	sts_pwrseq_snap_t s;

	memset(&st, 0, sizeof(st));
	st.stage = (pwrseq_stage_t)PWRSEQ_STAGE_9_ARM;

	sts_pwrseq_snap_from_status(&s, &st);
	sts_pwrseq_snap_observe(&s, NULL, 10000001U, true, 0x80020004U);

	TEST_ASSERT_EQUAL_UINT8((uint8_t)PWRSEQ_STAGE_9_ARM, s.stage);
	TEST_ASSERT_EQUAL_HEX32(0U, s.scan_state);
	TEST_ASSERT_EQUAL_UINT32(0U, s.tick_mono_ms);
	TEST_ASSERT_EQUAL_UINT32(0U, s.extref_hz);
}

/* ============================================ one rule, three planes ====== */

/**
 * The survey-accuracy rule must not fork between the web, MP and panel planes.
 *
 * A technician reads the same number off the web UI, the maintenance tool and
 * the box's own screen and compares it against `gnss.survey.acc`. Three
 * implementations of "which accuracy is in force" is three chances for them to
 * disagree, and the disagreement would look like a receiver problem.
 */
static void test_the_three_planes_report_the_same_survey_accuracy(void)
{
	static const struct {
		bool active;
		uint32_t svin_0p1mm;
		uint32_t pos_0p1mm;
		uint32_t want_mm;
	} vec[] = {
		{ true,  4149U, 90U,        415U },   /* live meanAcc, rounded up */
		{ true,  4144U, 90U,        414U },   /* ...rounded down */
		{ true,  0U,    10000U,     0U },     /* surveying, no meanAcc yet */
		{ false, 4149U, 10000U,     1000U },  /* finished: the stored figure */
		{ false, 0U,    UINT32_MAX, 429496730U }, /* no wrap at the top */
	};
	size_t i;

	for (i = 0U; i < (sizeof(vec) / sizeof(vec[0])); i++) {
		sts_gnss_detail_t d;
		rest_gnss_t web;
		ui_health_t panel;
		mp_telem_t t;

		memset(&d, 0, sizeof(d));
		d.svin_active = vec[i].active;
		d.svin_acc_0p1mm = vec[i].svin_0p1mm;
		d.pos_acc_0p1mm = vec[i].pos_0p1mm;
		d.pos_valid = true;

		memset(&web, 0, sizeof(web));
		memset(&panel, 0, sizeof(panel));
		memset(&t, 0, sizeof(t));

		sts_web_sky_survey(&d, &web);
		sts_ui_refs_from_gnss(&panel, &d);
		sts_mp_telem_bind_gnss(&t, &d);

		TEST_ASSERT_EQUAL_UINT32(vec[i].want_mm, web.survey_acc_mm);
		TEST_ASSERT_EQUAL_UINT32(vec[i].want_mm, panel.survey_acc_mm);
		TEST_ASSERT_EQUAL_UINT32(vec[i].want_mm, t.survey_acc_mm);
	}
}

/* ================================================= the panel's bindings === */

/**
 * The antenna enumerations are reordered between core/gnssmgr and core/ui, so a
 * cast is wrong in three of five cases. This is the table that says so.
 */
static void test_panel_antenna_map_is_not_a_cast(void)
{
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ANT_UNKNOWN,
				sts_ui_ant_state((uint8_t)GNSSMGR_ANT_UNKNOWN));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ANT_OFF,
				sts_ui_ant_state((uint8_t)GNSSMGR_ANT_OFF));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ANT_OK,
				sts_ui_ant_state((uint8_t)GNSSMGR_ANT_OK));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ANT_OPEN,
				sts_ui_ant_state((uint8_t)GNSSMGR_ANT_OPEN));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ANT_SHORT,
				sts_ui_ant_state((uint8_t)GNSSMGR_ANT_SHORT));
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ANT_UNKNOWN, sts_ui_ant_state(200U));

	/* The three a cast would get wrong, stated as a fact about the enums so
	 * that reordering either one fails here rather than on a panel. */
	TEST_ASSERT_NOT_EQUAL((uint8_t)GNSSMGR_ANT_OFF, (uint8_t)UI_ANT_OFF);
	TEST_ASSERT_NOT_EQUAL((uint8_t)GNSSMGR_ANT_OK, (uint8_t)UI_ANT_OK);
	TEST_ASSERT_NOT_EQUAL((uint8_t)GNSSMGR_ANT_OPEN, (uint8_t)UI_ANT_OPEN);
}

/**
 * The panel's Rb/external block, which used to be three literal `true`s keyed
 * off active_ref. Same defect as the MP plane's, same evidence.
 */
static void test_panel_rb_block_is_the_hardware_not_the_selection(void)
{
	sts_pwrseq_snap_t snap;
	ui_health_t h;

	memset(&snap, 0, sizeof(snap));
	snap.started = true;
	snap.rb_wanted = true;   /* an FE is fitted and configured */
	snap.rb_enabled = true;  /* its rail is up */
	snap.rb_lock_pin = false; /* and it has dropped lock */
	snap.extref_in_band = true;
	snap.extref_hz = 9999998U;

	memset(&h, 0, sizeof(h));
	sts_ui_refs_from_pwrseq(&h, &snap);

	TEST_ASSERT_TRUE(h.rb_present);
	TEST_ASSERT_TRUE(h.rb_powered);
	TEST_ASSERT_FALSE(h.rb_locked);
	TEST_ASSERT_TRUE(h.extref_ok);
	TEST_ASSERT_EQUAL_UINT32(9999998U, h.extref_hz);

	/* No sequencer: the block stays at the memset default rather than
	 * claiming an absent rubidium is present. */
	memset(&h, 0, sizeof(h));
	sts_ui_refs_from_pwrseq(&h, NULL);
	TEST_ASSERT_FALSE(h.rb_present);
	TEST_ASSERT_FALSE(h.rb_locked);
}

/** Survey progress reaches the Sky page's status line. */
static void test_panel_survey_progress(void)
{
	sts_gnss_detail_t d;
	ui_health_t h;

	detail_surveying(&d);
	memset(&h, 0, sizeof(h));
	sts_ui_refs_from_gnss(&h, &d);

	TEST_ASSERT_TRUE(h.survey_active);
	TEST_ASSERT_EQUAL_UINT32(1832U, h.survey_dur_s);
	TEST_ASSERT_EQUAL_UINT32(415U, h.survey_acc_mm);
	TEST_ASSERT_EQUAL_UINT8((uint8_t)UI_ANT_OPEN, h.ant_state);
}

int main(void)
{
	UNITY_BEGIN();
	RUN_TEST(test_pwrseq_and_gnss_reach_the_wire);
	RUN_TEST(test_nothing_published_is_all_zero);
	RUN_TEST(test_unstarted_sequencer_binds_nothing);
	RUN_TEST(test_scan_state_is_the_signal_not_the_alarm_view);
	RUN_TEST(test_rb_lock_is_the_pin_not_the_selection);
	RUN_TEST(test_extref_is_the_measurement_not_the_selection);
	RUN_TEST(test_refsel_recovery_is_total);
	RUN_TEST(test_pub_slot_starts_empty);
	RUN_TEST(test_pub_round_trip);
	RUN_TEST(test_pub_read_refuses_a_write_in_progress);
	RUN_TEST(test_pub_null_arguments);
	RUN_TEST(test_snap_from_status_clears_first);
	RUN_TEST(test_snap_observe_is_additive);
	RUN_TEST(test_snap_observe_ignores_scan_state_without_a_tick);
	RUN_TEST(test_the_three_planes_report_the_same_survey_accuracy);
	RUN_TEST(test_panel_antenna_map_is_not_a_cast);
	RUN_TEST(test_panel_rb_block_is_the_hardware_not_the_selection);
	RUN_TEST(test_panel_survey_progress);
	return UNITY_END();
}
