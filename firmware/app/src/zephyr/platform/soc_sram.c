/*
 * STS1000 Meridian — early SoC preparation hook.
 *
 * Copyright (c) 2026 RCL Design
 * SPDX-License-Identifier: Apache-2.0
 *
 * The board links its full 640 KB of system SRAM (SRAM1+SRAM2+SRAM3, see the
 * board DTS &sram1 widening). On the STM32H563 the SRAM2 and SRAM3 clocks are
 * gated at reset (RCC_AHB2ENR bits SRAM2EN/SRAM3EN), so they MUST be enabled
 * before the C runtime zeroes .bss or copies .data into the upper 384 KB.
 *
 * soc_prep_hook() is called from z_prep_c() *before* z_bss_zero()/z_data_copy()
 * (arch/arm/core/cortex_m/prep_c.c), which is the only correct point for this.
 * Enabling an already-enabled clock is harmless, so this is safe regardless of
 * the exact reset state. A read-back barrier guarantees the enable has taken
 * effect before the first access (STM32 RCC peripheral-enable erratum).
 */

#include <soc.h>

void soc_prep_hook(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_SRAM2EN | RCC_AHB2ENR_SRAM3EN;
	/* Read-back so the clock is live before .bss/.data touch SRAM2/SRAM3. */
	(void)RCC->AHB2ENR;
}
