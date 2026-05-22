/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Thin HAL wrapper for the YTM32 DMA controller.
 *
 * This header exposes a clean C interface that uses only standard types.
 * No vendor or Zephyr headers are included here, eliminating the
 * dma_callback_t typedef collision between the two ecosystems.
 */

#ifndef YTM32_DMA_HAL_H
#define YTM32_DMA_HAL_H

#include <stdbool.h>
#include <stdint.h>

/* Transfer direction values (independent of both Zephyr and vendor enums). */
#define YTM32_DMA_DIR_PERIPH2MEM 0U
#define YTM32_DMA_DIR_MEM2PERIPH 1U
#define YTM32_DMA_DIR_MEM2MEM    2U

/*
 * Completion/error callback invoked from ISR context.
 *   user_data : value provided to ytm32_dma_hal_channel_config
 *   status    : 0 on success, negative errno on error
 */
typedef void (*ytm32_dma_cb_t)(void *user_data, int status);

/*
 * Initialize the DMA controller. Must be called once before any channel
 * operations. Returns 0 on success, negative errno on failure.
 */
int ytm32_dma_hal_init(void);

/*
 * Configure and arm a DMA channel for a single-block transfer.
 *   ch         : virtual channel number (0 .. FEATURE_DMA_VIRTUAL_CHANNELS-1)
 *   trigsrc    : DMAMUX request source (dma_request_source_t numeric value)
 *   src        : source address
 *   dst        : destination address
 *   dir        : YTM32_DMA_DIR_* constant
 *   width      : transfer width in bytes (1, 2, or 4)
 *   length     : total byte count
 *   cb         : completion callback (may be NULL)
 *   user_data  : opaque value passed to cb
 * Returns 0 on success, negative errno on failure.
 */
int ytm32_dma_hal_channel_config(uint8_t ch, uint8_t trigsrc,
				 uintptr_t src, uintptr_t dst,
				 uint8_t dir, uint8_t width, uint32_t length,
				 ytm32_dma_cb_t cb, void *user_data);

/* Release a previously configured channel. */
void ytm32_dma_hal_channel_release(uint8_t ch);

/* Start a configured channel. Returns 0 on success. */
int ytm32_dma_hal_start(uint8_t ch);

/* Stop (disable request for) a channel. Returns 0 on success. */
int ytm32_dma_hal_stop(uint8_t ch);

/* Remaining trigger-loop iterations for an active channel. */
uint32_t ytm32_dma_hal_remaining(uint8_t ch);

/* True if the channel is in error state. */
bool ytm32_dma_hal_error(uint8_t ch);

/* Per-channel IRQ handler — call from the channel's ISR. */
void ytm32_dma_hal_irq(uint8_t ch);

/* Shared error IRQ handler — call from the DMA_Error ISR. */
void ytm32_dma_hal_error_irq(void);

/*
 * Configure a channel for loop (major/minor) transfer.
 *
 * Intended for hardware-triggered peripherals like ADC where a fixed source
 * address (e.g. FIFO) is read 'total_count' times into a linear destination
 * buffer.  After 'total_count' triggers the callback fires and the destination
 * pointer wraps back to 'dst', ready for the next call to
 * ytm32_dma_hal_start().
 *
 *   ch          : virtual channel (0 .. FEATURE_DMA_VIRTUAL_CHANNELS-1)
 *   trigsrc     : DMAMUX request source (dma_request_source_t numeric value)
 *   src         : fixed source address (e.g. peripheral FIFO register)
 *   dst         : destination buffer start address
 *   width       : transfer element width in bytes (1, 2, or 4)
 *   total_count : total number of DMA trigger cycles before callback; the
 *                 destination buffer must hold total_count * width bytes
 *   cb          : completion callback (NULL allowed); called after total_count
 *                 elements have been transferred
 *   user_data   : opaque value forwarded to cb
 */
int ytm32_dma_hal_channel_config_loop(uint8_t ch, uint8_t trigsrc,
				      uintptr_t src, uintptr_t dst,
				      uint8_t width, uint32_t total_count,
				      ytm32_dma_cb_t cb, void *user_data);

#endif /* YTM32_DMA_HAL_H */
