/*
 * STS1000 "Meridian" — local-UI area internal header.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * PRIVATE to src/zephyr/ui/. The platform, net and console areas talk to the
 * UI only through src/zephyr/sts_app.h (ARCHITECTURE.md §2); nothing here is
 * visible to them.
 *
 * The area is three translation units around the platform-neutral core/ui:
 *
 *   sts_ui.c      the entry point, the ui_ctx_t, the input message queue, the
 *                 10 Hz render/nav thread, the cfg applier and the action drain
 *                 (sts_ui_post_input() and the strong overrides live here);
 *   ui_display.c  the ST7796 lifecycle + power sequencing, the 8x16 tile
 *                 rasteriser, the dirty-region diff, and the DISP_BL backlight;
 *   ui_input.c    the TIM1 encoder delta and the raw FT6336U touch read.
 */

#ifndef STS1000_ZEPHYR_UI_STS_UI_H_
#define STS1000_ZEPHYR_UI_STS_UI_H_

#include <stdbool.h>
#include <stdint.h>

#include "ui/ui.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- tunables (rsource src/zephyr/ui/Kconfig from app/Kconfig to override) - */

#if defined(CONFIG_STS1000_UI_RENDER_STACK_SIZE)
#define STS_UI_RENDER_STACK CONFIG_STS1000_UI_RENDER_STACK_SIZE
#else
#define STS_UI_RENDER_STACK 3072
#endif

#if defined(CONFIG_STS1000_UI_RENDER_HZ)
#define STS_UI_RENDER_HZ CONFIG_STS1000_UI_RENDER_HZ
#else
#define STS_UI_RENDER_HZ 10
#endif

/* Panel geometry: 480x320 landscape, 8x16 font -> 60x20 tiles. */
#define STS_UI_COLS 60u
#define STS_UI_ROWS 20u

/* ---- ui_display.c ------------------------------------------------------- */

/**
 * Configure DISP_EN, the DISP_BL PWM and the tile/blit buffers. Does NOT touch
 * the panel controller and never blocks on the display rail — the panel is
 * probed lazily on the first blit (spec §6.5: DISP_EN is deferrable under PoE
 * pressure, so the rail may be off when this runs).
 */
int ui_display_init(void);

/**
 * Push @p surf to the panel, repainting only the tiles that changed since the
 * previous blit. Ensures the panel is powered and initialised first, retrying
 * with backoff; a not-yet-ready panel makes this a no-op that returns -EAGAIN
 * rather than blocking. Returns 0 when the frame (or its dirty subset) was
 * written.
 */
int ui_display_blit(const ui_surface_t *surf);

/** Set the DISP_BL duty, 0..1000 permille. Safe before the panel is up. */
void ui_display_backlight_permille(uint16_t permille);

/** True once the ST7796 has been initialised at least once. */
bool ui_display_ready(void);

/* ---- ui_input.c --------------------------------------------------------- */

/** Take ownership of the TIM1 encoder counter; reset the delta accumulator. */
int ui_input_encoder_init(void);

/**
 * Detents accumulated since the last call, signed (CW positive). Reads TIM1's
 * hardware quadrature counter, so no edge is ever missed between calls.
 */
int32_t ui_input_encoder_delta(void);

/**
 * Service a touch-controller interrupt: raw-read the FT6336U over I2C1 and, if
 * a press is present, write its panel coordinates to @p x / @p y.
 *
 * @retval 1        A touch point was read into @p x / @p y.
 * @retval 0        The controller reported no active touch.
 * @retval <0       I2C error (rail down, NAK, bus busy).
 */
int ui_input_touch_read(uint16_t *x, uint16_t *y);

/** Best-effort FT6336U init after DISP_RST release (interrupt-trigger mode). */
int ui_input_touch_init(void);

#ifdef __cplusplus
}
#endif

#endif /* STS1000_ZEPHYR_UI_STS_UI_H_ */
