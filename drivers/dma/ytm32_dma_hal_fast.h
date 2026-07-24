/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Compile-time-resolved DMA helpers for the zero-latency ISR path.
 *
 * These inline wrappers use direct register access via the vendor CMSIS
 * DMA_Type struct.  When the channel argument is a compile-time constant
 * (e.g. the DT-derived dma_channel), the compiler folds each function
 * into a single LDR/STR instruction pair — zero call overhead.
 *
 * Source code reads as clean function calls; compiled code is bare
 * register operations.
 *
 * IMPORTANT: These helpers are intentionally minimal — they only cover
 * the ch8 ZLI hot path.  The full vendor HAL (DMA_DRV_*) remains the
 * authority for init, config, and errata-handled paths.
 */

#ifndef ZEPHYR_DRIVERS_DMA_YTM32_DMA_HAL_FAST_H_
#define ZEPHYR_DRIVERS_DMA_YTM32_DMA_HAL_FAST_H_

#include <stdbool.h>
#include <stdint.h>

#include <YTM32B1MD1.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Disable DMA requests for a channel — minimal inline version.
 *
 * The vendor HAL (DMA_DRV_StopChannel) halts the entire DMA engine via
 * CTRL bit 17, waits for all channels to become idle, then does a
 * read-modify-write on REQEN.  That sequence takes ~260 cycles.
 *
 * This inline helper skips the halt/wait/resume and does a direct
 * read-modify-write on REQEN.  Safe when called from the ZLI ISR after
 * the current channel's transfer has completed (DONE flag set) and
 * depth=1 (no pending multi-transfer state).
 *
 * Compiles to: LDR-AND-ORR-STR on REQEN — ~4 cycles.
 */
static inline __attribute__((always_inline)) void ytm32_dma_hal_stop_ch_inline(uint8_t ch)
{
	DMA0->REQEN &= ~((uint32_t)1U << ch);
}

/*
 * Clear the channel trigger-loop-done interrupt flag.
 * Write-1-to-clear: writing a 1 to the bit clears it.
 * Equivalent to what the vendor DMA_DRV_IRQHandler does internally.
 *
 * Compiles to: LDR r0,[CHTLDIF_addr]; STR (1<<ch),[r0]  — ~2 cycles.
 */
static inline __attribute__((always_inline)) void ytm32_dma_hal_clear_done_flag_inline(uint8_t ch)
{
	DMA0->CHTLDIF = (uint32_t)1U << ch;
}

/*
 * Check if a channel's trigger-loop-done flag is set.
 * Returns true if the flag is set (transfer complete).
 */
static inline __attribute__((always_inline)) bool ytm32_dma_hal_is_done_inline(uint8_t ch)
{
	return (DMA0->CHTLDIF & ((uint32_t)1U << ch)) != 0U;
}

/**
 * @brief Restart a DMA channel after block completion.
 *
 * After one block completes the channel is idle and REQEN is auto-cleared.
 * Re-arm by: clear done flag → set REQEN → trigger START.
 * ~6 cycles total (vs ~267 for vendor HAL halt-wait-RMW-resume).
 */
static inline __attribute__((always_inline)) void ytm32_dma_hal_restart_inline(uint8_t ch)
{
	DMA0->CHTLDIF = (uint32_t)1U << ch;   /* clear done (W1C)      */
	DMA0->REQEN  |= (uint32_t)1U << ch;   /* re-enable HW requests */
	DMA0->START   = (uint32_t)1U << ch;   /* re-arm channel        */
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_DMA_YTM32_DMA_HAL_FAST_H_ */
