/*
 * STS1000 "Meridian" — composite USB device, gated on USB_VBUS_SENSE (PE2).
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * ARCHITECTURE.md §6: one composite device with two CDC-ACM interfaces —
 * ACM0 carries the Zephyr shell and the log console, ACM1 the binary MCP
 * channel used by tools/meridian_ctl.py.
 *
 * The board is self-powered (PoE), so USB 2.0 §7.1.5 applies: a self-powered
 * device must not present its D+ pull-up while VBUS is absent. PE2 is the
 * VBUS divider (docs/usb_console_interface.md); the board .dts reads it as a
 * plain digital input because STM32H563 has no ADC channel on that pin, and
 * it is polled rather than EXTI-driven because EXTI2 belongs to PC2
 * (interface ref §6). So: poll PE2, attach on a rising edge, detach on a
 * falling one.
 *
 * Nothing here is allowed to block the boot. If VBUS never appears the device
 * simply never attaches, which is the normal state of a rack-mounted
 * grandmaster.
 */

#include <zephyr/kernel.h>

#ifdef CONFIG_STS1000_CONSOLE

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>

#include "console/sts_console.h"

LOG_MODULE_REGISTER(sts_usb, CONFIG_STS1000_LOG_LEVEL);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec vbus_sense =
	GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, usb_vbus_sense_gpios);

static const struct device *const cdc_console =
	DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));
static const struct device *const cdc_mcp =
	DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart1));

static bool vbus_gpio_ok;
static bool vbus_state;
static bool usb_attached;
static bool usb_dc_configured;

/* ------------------------------------------------------------------------- */

static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	ARG_UNUSED(param);

	switch (status) {
	case USB_DC_CONFIGURED:
		usb_dc_configured = true;
		LOG_INF("USB configured by host");
		break;
	case USB_DC_DISCONNECTED:
	case USB_DC_RESET:
		usb_dc_configured = false;
		break;
	default:
		break;
	}
}

static bool dtr_asserted(const struct device *dev)
{
	uint32_t dtr = 0U;

	if ((dev == NULL) || !device_is_ready(dev)) {
		return false;
	}
	if (uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr) != 0) {
		return false;
	}
	return dtr != 0U;
}

bool sts_usb_configured(void)
{
	/*
	 * The status callback is the direct answer, but it only exists when
	 * this file owns usb_enable(). With CONFIG_USB_DEVICE_INITIALIZE_AT_BOOT
	 * left on, Zephyr attaches at boot with no callback and usb_enable()
	 * here returns -EALREADY; DTR on either interface is then the honest
	 * evidence that a host is present and has opened a port.
	 */
	return usb_dc_configured || dtr_asserted(cdc_console) ||
	       dtr_asserted(cdc_mcp);
}

bool sts_usb_vbus_present(void)
{
	return vbus_state;
}

/* ------------------------------------------------------------------------- */

static bool vbus_read(void)
{
	int val;

	if (!vbus_gpio_ok) {
		/*
		 * No usable sense line: assume VBUS is present so the console
		 * still comes up. Losing USB entirely would be a worse failure
		 * than an out-of-spec pull-up on an unplugged port.
		 */
		return true;
	}

	val = gpio_pin_get_dt(&vbus_sense);
	if (val < 0) {
		return vbus_state;
	}
	return val != 0;
}

static void usb_attach(void)
{
	int rc;

	if (usb_attached) {
		return;
	}

	rc = usb_enable(usb_status_cb);
	if (rc == -EALREADY) {
		LOG_DBG("USB was already enabled at boot");
	} else if (rc != 0) {
		LOG_ERR("usb_enable failed (%d)", rc);
		return;
	}

	usb_attached = true;
	LOG_INF("USB attached (VBUS present)");
}

static void usb_detach(void)
{
	int rc;

	if (!usb_attached) {
		return;
	}

	usb_dc_configured = false;

	rc = usb_disable();
	if (rc != 0) {
		LOG_WRN("usb_disable failed (%d)", rc);
		/* Leave usb_attached set: a failed detach has not detached. */
		return;
	}

	usb_attached = false;
	LOG_INF("USB detached (VBUS gone)");
}

void sts_usb_poll(void)
{
	bool now = vbus_read();

	if (now == vbus_state) {
		return;
	}

	vbus_state = now;
	if (now) {
		usb_attach();
	} else {
		/*
		 * Drop the MCP session before tearing the stack down so a
		 * response held for a link that no longer exists does not
		 * wedge the engine's input path on the next attach.
		 */
		sts_mcp_notify_link_down();
		usb_detach();
	}
}

int sts_usb_start(void)
{
	int rc;

	if (gpio_is_ready_dt(&vbus_sense)) {
		rc = gpio_pin_configure_dt(&vbus_sense, GPIO_INPUT);
		if (rc == 0) {
			vbus_gpio_ok = true;
		} else {
			LOG_ERR("USB_VBUS_SENSE configure failed (%d): USB will "
				"attach unconditionally",
				rc);
		}
	} else {
		LOG_ERR("USB_VBUS_SENSE port not ready: USB will attach "
			"unconditionally");
	}

	vbus_state = vbus_read();
	if (vbus_state) {
		usb_attach();
	} else {
		LOG_INF("USB held detached: no VBUS on the console port");
	}

	return 0;
}

#endif /* CONFIG_STS1000_CONSOLE */
