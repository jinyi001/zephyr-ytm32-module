/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Thin HAL wrapper for the YTM32 SPI master controller.
 *
 * This header exposes a clean C interface that uses only standard types.
 * No vendor or Zephyr headers are included here, eliminating the
 * spi_callback_t typedef collision between the two ecosystems.
 */

#ifndef YTM32_SPI_HAL_H
#define YTM32_SPI_HAL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Completion callback invoked from ISR context when a transfer finishes.
 *   user_data : value provided to ytm32_spi_hal_init
 *   status    : 0 on success, negative errno on error
 */
typedef void (*ytm32_spi_cb_t)(void *user_data, int status);

/*
 * Initialize an SPI master instance.
 *   instance   : SPI hardware instance number (0-3 on YTM32B1MD1)
 *   clock_rate : functional clock in Hz (used to derive default baud rate)
 *   use_dma    : true to enable DMA-based transfers
 *   dma_rx_ch  : DMA RX virtual channel (ignored when use_dma=false)
 *   dma_tx_ch  : DMA TX virtual channel (ignored when use_dma=false)
 *   cb         : transfer completion/error callback
 *   user_data  : opaque value forwarded to cb
 * Returns 0 on success, negative errno on failure.
 */
int ytm32_spi_hal_init(uint8_t instance, uint32_t clock_rate,
		       bool use_dma, uint8_t dma_rx_ch, uint8_t dma_tx_ch,
		       ytm32_spi_cb_t cb, void *user_data);

/*
 * Reconfigure SPI bus parameters for a new transfer.  May be called once
 * per transceive() when the active device changes.
 *   instance     : SPI hardware instance number
 *   freq         : desired baud rate in Hz
 *   cpol         : clock polarity (0 = idle-low, 1 = idle-high)
 *   cpha         : clock phase   (0 = sample on first edge, 1 = second edge)
 *   cs_idx       : hardware PCS index (0-3)
 *   cs_active_hi : true if CS asserts high
 *   gpio_cs      : true when CS is driven by software GPIO (not hardware PCS)
 * Returns 0 on success, negative errno on failure.
 */
int ytm32_spi_hal_configure(uint8_t instance, uint32_t freq,
			    uint8_t cpol, uint8_t cpha,
			    uint8_t cs_idx, bool cs_active_hi, bool gpio_cs);

/*
 * Start a non-blocking transfer.  The callback registered in
 * ytm32_spi_hal_init is invoked from ISR context when the transfer ends.
 *   tx  : transmit buffer (must remain valid until callback fires)
 *   rx  : receive  buffer (must remain valid until callback fires)
 *   len : byte count
 * Returns 0 if the transfer was accepted, negative errno otherwise.
 */
int ytm32_spi_hal_transfer(uint8_t instance,
			   const uint8_t *tx, uint8_t *rx, uint16_t len);

/* Abort an in-progress transfer. */
void ytm32_spi_hal_abort(uint8_t instance);

/* Per-instance IRQ handler — call from the SPI instance's ISR. */
void ytm32_spi_hal_irq(uint8_t instance);

#endif /* YTM32_SPI_HAL_H */
