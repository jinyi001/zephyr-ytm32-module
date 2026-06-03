/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * ISR-safe non-blocking SPI transfer for the YTM32 SPI driver.
 *
 * The standard Zephyr spi_transceive_dt() path acquires a mutex and blocks on
 * a semaphore — both are illegal in ISR context.  This header exposes a thin
 * bypass that calls the HAL directly, enabling pipeline-style SPI reads from
 * within a DMA completion or timer-overflow ISR.
 *
 * Constraints the caller MUST satisfy:
 *   1. The bus must already be configured for the target device.  Call
 *      spi_transceive_dt() from thread context at least once before using the
 *      async path (which leaves the bus configured).
 *   2. No concurrent blocking spi_transceive_dt() may be pending on the same
 *      bus instance.
 *   3. TX and RX buffers must remain valid until the callback fires.
 *   4. CS management is the caller's responsibility (GPIO assert/deassert).
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SPI_SPI_YTM32_H_
#define ZEPHYR_INCLUDE_DRIVERS_SPI_SPI_YTM32_H_

#include <zephyr/device.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Completion callback invoked from ISR context when the transfer ends. */
typedef void (*spi_ytm32_async_cb_t)(void *user_data, int status);

/**
 * @brief Start a non-blocking SPI transfer (ISR-safe).
 *
 * Starts a DMA-driven transfer and returns immediately.  The callback fires
 * from the DMA completion ISR when all bytes have been transferred.
 *
 * @param dev       SPI device (e.g. cfg->spi.bus from a spi_dt_spec)
 * @param tx        Transmit buffer (caller-owned, valid until cb fires)
 * @param rx        Receive buffer  (caller-owned, valid until cb fires)
 * @param len       Transfer length in bytes (must be even for 16-bit words)
 * @param cb        Completion callback — called from ISR, must be brief
 * @param user_data Opaque value forwarded to cb
 * @return 0 on success, negative errno if the HAL rejected the transfer
 */
int spi_ytm32_transceive_async(const struct device *dev,
			       const uint8_t *tx, uint8_t *rx, uint16_t len,
			       spi_ytm32_async_cb_t cb, void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SPI_SPI_YTM32_H_ */
