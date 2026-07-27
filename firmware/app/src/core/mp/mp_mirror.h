/*
 * STS1000 "Meridian" — core/mp: live front-panel mirror.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-neutral C11. No dynamic allocation; the previous-frame store is
 * caller-owned.
 *
 * NOT PART OF THE FMT SPEC. This is the new customer requirement: the host must
 * be able to render a live replica of the physical front panel — what the screen
 * says, which indicators are lit, and what the operator is physically doing to
 * the box — so a technician on the phone and an engineer on a laptop are looking
 * at the same panel.
 *
 * Three things are mirrored:
 *
 *   (a) the screen — `core/ui`'s text-tile surface (characters + attributes),
 *       the active page, the stack depth and selection, and any dialog state;
 *   (b) indicators — the panel-LED rail state and PWM duty, the per-button
 *       logical lamp states, the status RGB's commanded pattern and channel
 *       duties, and the display backlight;
 *   (c) input echo — which buttons are physically down (from the 1 kHz scan),
 *       the encoder position, and the last touch coordinate.
 *
 * Transport. The mirror is its own channel, **0x0A**, allocated from the FMT
 * spec's reserved 0x0A-0x1F range, rather than a record type on the telemetry
 * channel. Two reasons: a 10 Hz mirror would otherwise force the telemetry rate
 * to follow it, and a host that does not implement the mirror can drop one
 * channel id instead of having to parse and discard a record type. `mirror.get`
 * returns the same record over the control channel as a full keyframe.
 *
 * Delta encoding. A keyframe carries every row; every other frame carries only
 * the rows that changed, narrowed to the changed column span. A row is recorded
 * in the previous-frame store **only once it has actually been emitted**, so a
 * frame truncated by the buffer cap cannot desynchronise the host: the rows that
 * did not fit stay dirty and go out next frame. `MP_MIRROR_F_PARTIAL` tells the
 * host that happened.
 *
 * As-built note on (b): the seven panel lamps (`PANEL_LED_WHITE_N_1..6` plus
 * `PANEL_LED_RED_N_1`) are one series string on `V_PANEL_LED`, gated by
 * `PANEL_LED_EN` (PC0) and dimmed by one `PANEL_LED_PWM` (PE0/LPTIM2_CH2). They
 * are therefore not individually addressable in hardware, and `led_logical` is
 * the UI's *logical* lamp state (which control the UI considers active), not an
 * electrical readback. The electrical truth is `panel_rail_on` +
 * `panel_duty_pct` + `panel_fault` — the last of which is the *alarm* derived
 * from PF12 rather than the pin itself; see its field comment.
 *
 * As-built note on (c): `buttons_down` is a level bitmap the producer
 * reconstructs from a droppable event stream, not a read of the debounced scan.
 * The producer clears it whenever it detects that an event was lost, so a
 * transient false negative is possible and a latched false positive is not. Do
 * not treat a bit that never sets as proof a button is unwired.
 */

#ifndef STS1000_CORE_MP_MP_MIRROR_H_
#define STS1000_CORE_MP_MP_MIRROR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ui/ui.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Mirror record version. Bump on any wire change. */
#define MP_MIRROR_VER 1U

/** Frame flags (key 8). */
#define MP_MIRROR_F_KEYFRAME (1U << 0) /**< every row is present */
#define MP_MIRROR_F_AWAKE (1U << 1)    /**< the panel is not in idle blank */
#define MP_MIRROR_F_IDENTIFY (1U << 2) /**< operator-commanded locate is running */
#define MP_MIRROR_F_LAMP (1U << 3)     /**< lamp test held */
#define MP_MIRROR_F_DIALOG (1U << 4)   /**< an edit/confirm dialog is on top */
#define MP_MIRROR_F_GEOM (1U << 5)     /**< geometry changed with this frame */
#define MP_MIRROR_F_PARTIAL (1U << 6)  /**< dirty rows were left for next frame */

/** Upper bound on one mirror record: a full keyframe of the as-built 60x20 grid. */
#define MP_MIRROR_MAX 3072U

/**
 * One frame's input. The glue fills this from the UI area's last render; core
 * never retains any pointer from it.
 */
typedef struct {
	/* --- (a) screen ------------------------------------------------- */
	uint8_t rows;
	uint8_t cols;
	uint8_t cell_w;
	uint8_t cell_h;
	const char *ch;      /**< rows*cols characters, row-major */
	const uint8_t *attr; /**< rows*cols UI_ATTR_* bytes, row-major */

	const ui_hint_t *hint; /**< overlay hints from the same render */
	uint8_t hint_count;

	uint8_t page;  /**< ui_page_t on top of the stack */
	uint8_t depth; /**< screen-stack depth */
	uint8_t sel;   /**< selection index on the top frame */
	bool dialog;   /**< the top frame is EDIT or CONFIRM */
	uint8_t dialog_item;  /**< ui_menu_item_t under edit/confirmation */
	uint8_t dialog_stage; /**< CONFIRM: 0 = first ask, 1 = second */

	bool awake;
	bool identify;
	bool lamp_test;

	/* --- (b) indicators --------------------------------------------- */
	bool panel_rail_on;     /**< PANEL_LED_EN (PC0) asserted */
	uint8_t panel_duty_pct; /**< PANEL_LED_PWM (PE0) duty */
	/**
	 * Unmasked panel-LED fault *alarm* — not a raw read of U55's nFLG (PF12).
	 *
	 * The producer takes it from the alarm aggregate, which suppresses any
	 * scanned signal the power sequencer has marked expected-off, and the
	 * panel-LED fault is marked exactly that whenever firmware has gated the
	 * rail. A deliberately-dark panel therefore reports false here with the
	 * pin asserted, which is the answer an operator wants; "the pin" is not
	 * available to a host through this record.
	 */
	bool panel_fault;
	uint16_t led_logical;   /**< UI logical lamp bitmap (see the header) */
	uint16_t bl_permille;   /**< display backlight, 0..1000 */
	uint8_t rgb_state;      /**< fault_rgb_state_t commanded pattern */
	uint8_t rgb_r;          /**< D5 red duty, 0..100 (TIM4_CH1, PD12) */
	uint8_t rgb_g;          /**< D5 green duty (TIM4_CH2, PD13) */
	uint8_t rgb_b;          /**< D5 blue duty (TIM4_CH3, PD14) */

	/* --- (c) input echo -------------------------------------------- */
	/** Buttons the producer believes are held, PF0..PF6 + PF11, bit =
	 *  fault_sig_t. Reconstructed from the scan's press/release events and
	 *  cleared on a detected event loss — see the header note on (c). */
	uint32_t buttons_down;
	int32_t enc_pos;       /**< TIM1 quadrature position */
	uint16_t touch_x;
	uint16_t touch_y;
	uint32_t touch_ms; /**< when the coordinate was captured; 0 = never */
} mp_mirror_in_t;

/**
 * Mirror state. `prev_ch` / `prev_attr` are caller-owned and must each hold at
 * least `cells` bytes; size them to the panel geometry (60*20 = 1200 as built).
 */
typedef struct {
	char *prev_ch;
	uint8_t *prev_attr;
	size_t cells;

	uint8_t rows;
	uint8_t cols;
	uint8_t cell_w;
	uint8_t cell_h;
	bool have_prev;

	/** Force a keyframe after this many delta frames, so a host that joined
	 *  mid-stream converges without asking. A keyframe therefore lands every
	 *  `keyframe_every + 1` frames. 0 disables periodic keyframes. */
	uint16_t keyframe_every;
	uint16_t since_key;

	/* counters */
	uint32_t frames;
	uint32_t keyframes;
	uint32_t rows_sent;
	uint32_t partials;
} mp_mirror_ctx_t;

/** Default periodic keyframe interval: 5 s at 10 Hz. */
#define MP_MIRROR_KEYFRAME_EVERY 50U

/**
 * Bind the previous-frame store.
 *
 * @retval 0        Bound; the next frame is a keyframe.
 * @retval -EINVAL  @p c is NULL, or a buffer is NULL with a non-zero @p cells.
 */
int mp_mirror_init(mp_mirror_ctx_t *c, char *prev_ch, uint8_t *prev_attr,
		   size_t cells);

/** Forget the previous frame, so the next one is a keyframe. */
void mp_mirror_reset(mp_mirror_ctx_t *c);

/**
 * Encode one mirror frame.
 *
 * Keys, mirror record version 1:
 *   8  flags        9  page          10 depth         11 sel
 *   12 dialog [item, stage] or null
 *   13 geom [rows, cols, cell_w, cell_h]
 *   14 rows: array of [row, col0, text(bstr), attrs(bstr)]
 *   15 hints: array of [kind, row, col, len, attr, value]
 *   16 leds [rail_on, duty_pct, fault, logical, bl_permille, rgb_state,
 *            rgb_r, rgb_g, rgb_b]
 *   17 input [buttons_down, enc_pos, touch_x, touch_y, touch_ms]
 *
 * @param force_key  Emit a keyframe regardless of the delta state
 *                   (`mirror.get` sets this).
 *
 * @retval >=0        Encoded length.
 * @retval -EINVAL    Bad argument, or geometry beyond UI_SURF_MAX_ROWS/COLS.
 * @retval -ENOMEM    The previous-frame store is smaller than rows*cols.
 * @retval -ENOSPC    @p cap cannot hold even the frame with no rows.
 */
int mp_mirror_encode(mp_mirror_ctx_t *c, const mp_mirror_in_t *in, uint32_t seq,
		     uint64_t mono_ms, bool force_key, uint8_t *buf,
		     size_t cap);

/**
 * Rows that differ from the previous frame, as a bitmap (bit n = row n).
 *
 * Exposed for the tests and for a glue that wants to skip the encode when
 * nothing moved. Returns all-dirty when no previous frame is held.
 */
uint32_t mp_mirror_dirty(const mp_mirror_ctx_t *c, const mp_mirror_in_t *in);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_CORE_MP_MP_MIRROR_H_ */
