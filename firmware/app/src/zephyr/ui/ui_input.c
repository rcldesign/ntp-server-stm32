/*
 * STS1000 "Meridian" — UI input glue: TIM1 quadrature encoder + FT6336U touch.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * ENCODER (TIM1, PA8/PA9)
 * -----------------------
 * The board .dts declares `panel_qdec` on TIM1 in hardware encoder-interface
 * mode (x4 quadrature, input filter level 5). The in-tree qdec_stm32 driver
 * owns *initialisation* — pinctrl, the TIM1 clock gate, LL_TIM_ENCODER_Init and
 * the counter enable — which is why this file does not re-run any of that. It
 * does NOT use the sensor channel API to read the count: SENSOR_CHAN_ROTATION
 * returns degrees-since-index (a Q26.6 fixed-point angle), and converting that
 * back to integer detents loses counts. Instead ui_input_encoder_delta() reads
 * the hardware counter directly with LL_TIM_GetCounter() and returns exact
 * signed detent deltas, x4-decimated, remainder carried — no edge is ever
 * missed between calls. That is the "read CNT via LL" the plan asks for, with
 * the driver kept for the one-time init it already performs from devicetree.
 *
 * TOUCH (FT6336U @ 0x38 on I2C1, behind PCA9306)
 * ----------------------------------------------
 * The in-tree ft5336 input driver needs a dedicated int-gpio, but PF7
 * (DISP_TOUCH_INT) is consumed by the platform's 1 kHz GPIO scan (interface
 * ref §5). So there is no DT node for the controller and no driver binds it;
 * the platform posts an STS_INPUT_TOUCH event when PF7 asserts, and this file
 * does a raw I2C register read of the touch point in response
 * (ui_input_touch_read), called from the UI render thread — never from the
 * scan thread, which must not block on I2C. Bus sharing with the housekeeping
 * sweep is safe: the STM32 I2C controller driver serialises transfers with its
 * own per-controller mutex.
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <stm32_ll_tim.h>

#include "zephyr/ui/sts_ui.h"

LOG_MODULE_REGISTER(sts_ui_input, CONFIG_STS1000_LOG_LEVEL);

/* ------------------------------------------------------------- encoder */

#define QDEC_NODE DT_NODELABEL(panel_qdec)
#define ENC_TIM_NODE DT_PARENT(QDEC_NODE)

/* x4 quadrature: four counts per mechanical detent (docs/sts1000_panel_controls
 * §2.2, placeholder 24-detent encoder = 96 counts/rev). */
#define ENC_COUNTS_PER_DETENT 4

#if DT_NODE_HAS_STATUS(QDEC_NODE, okay)
static const struct device *const qdec = DEVICE_DT_GET(QDEC_NODE);
static TIM_TypeDef *const enc_tim = (TIM_TypeDef *)DT_REG_ADDR(ENC_TIM_NODE);
#else
static const struct device *const qdec;
static TIM_TypeDef *const enc_tim;
#endif

static uint32_t enc_last_cnt;
static int32_t enc_residual; /* raw counts not yet promoted to a detent */
static bool enc_ready;

int ui_input_encoder_init(void)
{
	if (enc_tim == NULL) {
		LOG_WRN("encoder: panel_qdec node disabled");
		return -ENODEV;
	}
	if (qdec == NULL || !device_is_ready(qdec)) {
		/* The qdec driver owns TIM1 setup; without it the counter is
		 * not running and reads would be meaningless. */
		LOG_WRN("encoder: qdec device not ready; rotation disabled");
		return -ENODEV;
	}

	enc_last_cnt = LL_TIM_GetCounter(enc_tim);
	enc_residual = 0;
	enc_ready = true;
	return 0;
}

int32_t ui_input_encoder_delta(void)
{
	uint32_t cnt;
	int32_t raw;
	int32_t detents;

	if (!enc_ready) {
		return 0;
	}

	cnt = LL_TIM_GetCounter(enc_tim);

	/*
	 * TIM1 is a 16-bit counter and the qdec driver sets ARR to a multiple
	 * of counts-per-revolution just below 0xFFFF, so the count wraps. A
	 * signed 16-bit difference recovers the true delta for any movement
	 * short of half a revolution between calls — orders of magnitude more
	 * than a human turns a knob in a 100 ms render tick.
	 */
	raw = (int16_t)((uint16_t)cnt - (uint16_t)enc_last_cnt);
	enc_last_cnt = cnt;

	enc_residual += raw;
	detents = enc_residual / ENC_COUNTS_PER_DETENT;
	enc_residual -= detents * ENC_COUNTS_PER_DETENT;
	return detents;
}

/* --------------------------------------------------------------- touch */

#define TOUCH_I2C_NODE DT_NODELABEL(i2c1)
#define FT6336U_ADDR 0x38u

/* FT6336U register map (subset). */
#define FT_REG_TD_STATUS 0x02u /* low nibble = number of active points */
#define FT_REG_P1_XH 0x03u     /* [7:6] event flag, [3:0] X[11:8] */
#define FT_REG_P1_XL 0x04u     /* X[7:0] */
#define FT_REG_P1_YH 0x05u     /* [3:0] Y[11:8] */
#define FT_REG_P1_YL 0x06u     /* Y[7:0] */
#define FT_REG_G_MODE 0xA4u    /* 0 = polling, 1 = interrupt-trigger */
#define FT_REG_CHIP_ID 0xA3u

#define FT_EVT_MASK 0xC0u
#define FT_EVT_LIFT 0x40u /* touch-up: ignore, it carries stale coords */

/*
 * Native touch panel resolution and the rotation onto the landscape display.
 *
 * The FT6336U reports coordinates in the glass's portrait frame (320 wide x
 * 480 tall on the common modules); the ST7796 is driven landscape 480x320
 * (MADCTL in app/dts/ui.overlay). The 90 deg mapping below turns the portrait
 * touch frame into the landscape display frame. core/ui uses the Y coordinate
 * to pick a row, so getting Y onto the display's vertical axis is what matters.
 *
 * BENCH: confirm the axis directions against the fitted module. A touch that
 * selects the row above/below the finger is an inverted axis (drop the
 * subtraction); a touch whose row tracks horizontal finger motion is a swapped
 * pair (exchange tx/ty). Neither is a functional failure — the panel still
 * renders — so this is a tuning item, not a blocker.
 */
#define TOUCH_NATIVE_W 320u /* portrait X span */
#define TOUCH_NATIVE_H 480u /* portrait Y span */
#define DISP_W (STS_UI_COLS * 8u)
#define DISP_H (STS_UI_ROWS * 16u)

#if DT_NODE_HAS_STATUS(TOUCH_I2C_NODE, okay)
static const struct device *const touch_i2c = DEVICE_DT_GET(TOUCH_I2C_NODE);
#else
static const struct device *const touch_i2c;
#endif

int ui_input_touch_init(void)
{
	uint8_t id = 0u;
	int rc;

	if (touch_i2c == NULL || !device_is_ready(touch_i2c)) {
		return -ENODEV;
	}

	/* Identify (best effort — the rail may still be settling). */
	rc = i2c_reg_read_byte(touch_i2c, FT6336U_ADDR, FT_REG_CHIP_ID, &id);
	if (rc != 0) {
		LOG_WRN("FT6336U not responding (%d); touch deferred", rc);
		return rc;
	}
	LOG_INF("FT6336U chip id 0x%02x", id);

	/* Interrupt-trigger mode: INT pulses on a new touch, which the platform
	 * scan sees on PF7 and turns into an STS_INPUT_TOUCH event. */
	rc = i2c_reg_write_byte(touch_i2c, FT6336U_ADDR, FT_REG_G_MODE, 0x01u);
	if (rc != 0) {
		LOG_WRN("FT6336U mode set failed (%d)", rc);
	}
	return rc;
}

int ui_input_touch_read(uint16_t *x, uint16_t *y)
{
	uint8_t buf[5];
	uint16_t tx;
	uint16_t ty;
	uint16_t dx;
	uint16_t dy;
	int rc;

	if (x == NULL || y == NULL) {
		return -EINVAL;
	}
	if (touch_i2c == NULL || !device_is_ready(touch_i2c)) {
		return -ENODEV;
	}

	/* Block read TD_STATUS..P1_YL in one transaction. */
	rc = i2c_burst_read(touch_i2c, FT6336U_ADDR, FT_REG_TD_STATUS, buf,
			    sizeof(buf));
	if (rc != 0) {
		return rc;
	}

	if ((buf[0] & 0x0Fu) == 0u) {
		return 0; /* no active touch point */
	}
	if ((buf[1] & FT_EVT_MASK) == FT_EVT_LIFT) {
		return 0; /* lift event: coordinates are stale */
	}

	tx = (uint16_t)(((uint16_t)(buf[1] & 0x0Fu) << 8) | buf[2]);
	ty = (uint16_t)(((uint16_t)(buf[3] & 0x0Fu) << 8) | buf[4]);

	if (tx >= TOUCH_NATIVE_W) {
		tx = (uint16_t)(TOUCH_NATIVE_W - 1u);
	}
	if (ty >= TOUCH_NATIVE_H) {
		ty = (uint16_t)(TOUCH_NATIVE_H - 1u);
	}

	/* Portrait -> landscape, 90 deg (see the BENCH note above). */
	dx = ty;
	dy = (uint16_t)(TOUCH_NATIVE_W - 1u - tx);

	if (dx >= DISP_W) {
		dx = (uint16_t)(DISP_W - 1u);
	}
	if (dy >= DISP_H) {
		dy = (uint16_t)(DISP_H - 1u);
	}

	*x = dx;
	*y = dy;
	return 1;
}
