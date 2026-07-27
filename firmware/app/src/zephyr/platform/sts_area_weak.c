/*
 * STS1000 "Meridian" — __weak no-op fallbacks for the glue-area entry points.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * The four glue areas (platform, net, console, ui) are developed in parallel
 * and are individually switchable via CONFIG_STS1000_{NET,CONSOLE,UI}. Every
 * cross-area entry point declared in sts_app.h that is *not* implemented by
 * the platform area gets a weak no-op here, so the image links whether or not
 * the owning area exists yet.
 *
 * Overriding works because Zephyr links every zephyr_library — `app` included
 * — inside --whole-archive (zephyr/cmake/linker/ld/target.cmake), so a strong
 * definition in an area's object file always wins over the weak one here. It
 * does NOT depend on archive member ordering.
 *
 * A fallback must be a *safe* no-op, not a stub that pretends to succeed at
 * something observable. Hence:
 *   - the area start functions return 0 (nothing to start is not an error);
 *   - sts_ui_post_input() drops the event (no consumer);
 *   - sts_update_*() report "confirmed", because with no image manager there
 *     is nothing to confirm and reporting "pending" would make the supervisor
 *     retry forever.
 */

#include <zephyr/kernel.h>
#include <zephyr/toolchain.h>

#include "zephyr/sts_app.h"
#include "storage/sts_store.h"

__weak int sts_net_start(void)
{
	return 0;
}

__weak int sts_console_start(void)
{
	return 0;
}

__weak int sts_ui_start(void)
{
	return 0;
}

__weak void sts_ui_post_input(const sts_input_evt_t *evt)
{
	ARG_UNUSED(evt);
}

__weak bool sts_update_pending_confirm(void)
{
	return false;
}

__weak int sts_update_self_confirm(void)
{
	return 0;
}

/*
 * PFI fast-save hooks. The console area's storage backend (src/zephyr/storage/)
 * owns the strong definitions; these weak no-ops let the platform's PFI ISR,
 * the discipline loop and the gnss path call them unconditionally, so a
 * CONFIG_STS1000_CONSOLE=n image still links (nothing is persisted, which is
 * the correct behaviour with no storage backend).
 */
__weak void sts_store_critical_flush_from_isr(void)
{
}

__weak void sts_store_note_dac_code(uint16_t code)
{
	ARG_UNUSED(code);
}

__weak void sts_store_note_leap(int16_t current, int16_t pending, bool valid)
{
	ARG_UNUSED(current);
	ARG_UNUSED(pending);
	ARG_UNUSED(valid);
}

__weak void sts_store_note_log_cursor(uint32_t cursor)
{
	ARG_UNUSED(cursor);
}
