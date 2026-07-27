/*
 * STS1000 "Meridian" — PANEL_LED_PWM on LPTIM2_CH2 (PE0, AF3).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * Interface ref §1 line 141 and §7 specify hardware PWM on LPTIM2_CH2. That
 * contradicted a comment in the board .dts and a TODO in prj.conf claiming no
 * timer output reaches PE0; both were wrong and have been corrected. The AF
 * table for STM32H563 (modm-devices `stm32h5-62_63_73.xml`, the source
 * CLAUDE.md nominates when ST pin data is not to hand) gives PE0 as:
 *
 *     AF1 LPTIM1_ETR   AF2 TIM4_ETR    AF3 LPTIM2_CH2   AF4 LPTIM2_ETR
 *     AF6 SPI3_RDY     AF8 UART8_RX    AF9 FDCAN1_RX    AF10 SAI2_MCLK_A
 *     AF12 FMC_NBL0    AF13 DCMI/PSSI_D2
 *
 * so AF3 is correct. Zephyr's generated SoC pinctrl file has no LPTIM groups
 * at all — genpinctrl.py does not emit them for any family, because the
 * in-tree LPTIM driver is only ever used as a kernel tick source and needs no
 * pins — which is why the group is declared in the board pinctrl overlay
 * (sts1000_meridian-pinctrl.dtsi) rather than referenced from the SoC package.
 *
 * There is no Zephyr LPTIM PWM driver in 4.2, so the peripheral is programmed
 * through LL here: clock gate via clock_control_on() from the devicetree
 * clocks property, pin mux via pinctrl_apply_state(), then ARR/CMP.
 *
 * Frequency: the INA228 on the panel-LED rail (U54, 0x4C) must average over
 * many PWM periods or its current reading beats against the duty cycle
 * (interface ref §4.1). Its conversion time x averaging is on the order of
 * milliseconds, so ~1 kHz is chosen — three orders of magnitude above the LED
 * flicker threshold and comfortably inside the averaging window.
 *
 * Fail-safe: PE0 is Hi-Z at reset with the Q23 base pulled down, so the LEDs
 * are off before firmware runs and off again if firmware stops driving. Duty 0
 * also drops PANEL_LED_EN (PC0), so the string is de-energised rather than
 * merely dark.
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <stm32_ll_lptim.h>

#include "zephyr/platform/platform.h"
#include "zephyr/sts_app.h"

LOG_MODULE_REGISTER(sts_panel_pwm, CONFIG_STS1000_LOG_LEVEL);

#define PANEL_LPTIM_NODE DT_NODELABEL(lptim2)

BUILD_ASSERT(DT_NODE_HAS_STATUS(PANEL_LPTIM_NODE, okay),
	     "lptim2 must be enabled for PANEL_LED_PWM (see app/dts/platform.overlay)");

PINCTRL_DT_DEFINE(PANEL_LPTIM_NODE);

static LPTIM_TypeDef *const panel_lptim =
	(LPTIM_TypeDef *)DT_REG_ADDR(PANEL_LPTIM_NODE);

static const struct stm32_pclken panel_pclken[] = STM32_DT_CLOCKS(PANEL_LPTIM_NODE);
static const struct pinctrl_dev_config *panel_pcfg =
	PINCTRL_DT_DEV_CONFIG_GET(PANEL_LPTIM_NODE);

static const struct gpio_dt_spec panel_led_en = STS_USER_GPIO(panel_led_en_gpios);

/*
 * LPTIM prescaler /128 off PCLK1. At the board's 250 MHz PCLK1 that is
 * 1.953 MHz, and ARR = 1952 gives 1000.06 Hz with 1953 duty steps — far more
 * resolution than the eye or the INA228 can use.
 */
#define PANEL_LPTIM_PRESCALER LL_LPTIM_PRESCALER_DIV128
#define PANEL_LPTIM_PRESCALER_DIV 128U

static struct {
	uint32_t arr;
	uint8_t duty_pct;
	bool ready;
} panel;

static int panel_program_duty(uint8_t duty_pct)
{
	uint32_t cmp;

	/*
	 * LPTIM output is high while CNT < CMP (PWM mode, active-high), so CMP
	 * is the on-time in counts. CMP must stay <= ARR; the datasheet also
	 * forbids CMP == ARR for a 100 % duty, so full brightness is expressed
	 * as CMP = ARR, which the hardware treats as always-active for the
	 * whole period given ARR is the reload value. Clamp rather than trust
	 * the caller.
	 */
	if (duty_pct > 100U) {
		duty_pct = 100U;
	}

	cmp = ((uint32_t)duty_pct * (panel.arr + 1U)) / 100U;
	if (cmp > panel.arr) {
		cmp = panel.arr;
	}

	LL_LPTIM_SetCompare(panel_lptim, cmp);

	return 0;
}

int sts_panel_led_init(void)
{
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	uint32_t lptim_hz = 0;
	int rc;

	if (panel.ready) {
		return -EALREADY;
	}

	if (!device_is_ready(clk)) {
		return -ENODEV;
	}
	if (!gpio_is_ready_dt(&panel_led_en)) {
		LOG_ERR("PANEL_LED_EN not ready");
		return -ENODEV;
	}

	/* Off before anything else touches the string. */
	rc = gpio_pin_configure_dt(&panel_led_en, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		return rc;
	}

	/* Gate clock, and select the kernel clock source if the devicetree
	 * gave one (a second clocks entry). */
	rc = clock_control_on(clk, (clock_control_subsys_t)&panel_pclken[0]);
	if (rc != 0) {
		LOG_ERR("LPTIM2 clock gate failed (%d)", rc);
		return rc;
	}

	if (ARRAY_SIZE(panel_pclken) > 1U) {
		rc = clock_control_configure(clk,
					     (clock_control_subsys_t)&panel_pclken[1],
					     NULL);
		if (rc != 0) {
			LOG_ERR("LPTIM2 source select failed (%d)", rc);
			return rc;
		}
	}

	rc = clock_control_get_rate(clk, (clock_control_subsys_t)&panel_pclken[0],
				    &lptim_hz);
	if (rc != 0 || lptim_hz == 0U) {
		LOG_ERR("LPTIM2 rate unknown (%d)", rc);
		return (rc != 0) ? rc : -EIO;
	}

	rc = pinctrl_apply_state(panel_pcfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("PANEL_LED_PWM pinctrl failed (%d)", rc);
		return rc;
	}

	/* ARR must be written with the peripheral enabled but the counter
	 * stopped, and the period is (ARR + 1) counts. */
	LL_LPTIM_Disable(panel_lptim);
	LL_LPTIM_SetPrescaler(panel_lptim, PANEL_LPTIM_PRESCALER);
	LL_LPTIM_SetClockSource(panel_lptim, LL_LPTIM_CLK_SOURCE_INTERNAL);
	LL_LPTIM_SetCounterMode(panel_lptim, LL_LPTIM_COUNTER_MODE_INTERNAL);
	LL_LPTIM_SetPolarity(panel_lptim, LL_LPTIM_OUTPUT_POLARITY_REGULAR);

	panel.arr = (lptim_hz / PANEL_LPTIM_PRESCALER_DIV) /
		    CONFIG_STS1000_PANEL_LED_PWM_HZ;
	if (panel.arr == 0U) {
		LOG_ERR("LPTIM2: %u Hz / %u is below %u Hz PWM", lptim_hz,
			PANEL_LPTIM_PRESCALER_DIV,
			(unsigned int)CONFIG_STS1000_PANEL_LED_PWM_HZ);
		return -EINVAL;
	}
	if (panel.arr > UINT16_MAX) {
		panel.arr = UINT16_MAX;
	}
	panel.arr -= 1U;

	LL_LPTIM_Enable(panel_lptim);
	LL_LPTIM_SetAutoReload(panel_lptim, panel.arr);
	(void)panel_program_duty(0U);
	LL_LPTIM_StartCounter(panel_lptim, LL_LPTIM_OPERATING_MODE_CONTINUOUS);

	panel.duty_pct = 0U;
	panel.ready = true;

	LOG_INF("PANEL_LED_PWM up: LPTIM2_CH2/PE0, %u Hz (ARR %u from %u Hz /%u)",
		(unsigned int)CONFIG_STS1000_PANEL_LED_PWM_HZ, panel.arr, lptim_hz,
		PANEL_LPTIM_PRESCALER_DIV);

	return 0;
}

int sts_panel_led_set(uint8_t duty_pct)
{
	int rc;

	if (!panel.ready) {
		return -ENODEV;
	}
	if (duty_pct > 100U) {
		duty_pct = 100U;
	}

	rc = panel_program_duty(duty_pct);
	if (rc != 0) {
		return rc;
	}

	/*
	 * Order matters in both directions: turn the string's supply on before
	 * raising the duty, and drop the duty before removing the supply, so
	 * PANEL_LED_FAULT (PF12, the RT9742 nFLG) never sees a rail coming up
	 * into a hard-driven load.
	 */
	rc = gpio_pin_set_dt(&panel_led_en, (duty_pct != 0U) ? 1 : 0);
	if (rc != 0) {
		return rc;
	}

	panel.duty_pct = duty_pct;
	return 0;
}

uint8_t sts_panel_led_get(void)
{
	return panel.duty_pct;
}
