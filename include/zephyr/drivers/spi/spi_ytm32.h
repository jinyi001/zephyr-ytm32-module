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
 * The *_async_dt() API owns configuration and GPIO CS for one transaction. It
 * can be called from ISR context and returns -EBUSY instead of waiting when a
 * blocking waiter or another transfer owns the bus.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SPI_SPI_YTM32_H_
#define ZEPHYR_INCLUDE_DRIVERS_SPI_SPI_YTM32_H_

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Completion callback invoked from ISR context when the transfer ends. */
typedef void (*spi_ytm32_async_cb_t)(void *user_data, int status);

/**
 * @brief Prepare a device-specific async SPI configuration.
 *
 * Must be called from thread context before
 * spi_ytm32_transceive_async_dt(). The spec, buffers and callback data must
 * remain valid until the async callback returns.
 */
int spi_ytm32_async_prepare_dt(const struct spi_dt_spec *spec);

/**
 * @brief Start one device-specific non-blocking SPI transaction.
 *
 * The controller validates and applies @p spec->config, asserts/deasserts its
 * GPIO CS when present, and holds the hardware transaction gate until the
 * transfer is complete. The callback is invoked from ISR context exactly once
 * for a transfer that was accepted.
 */
int spi_ytm32_transceive_async_dt(
	const struct spi_dt_spec *spec, const uint8_t *tx, uint8_t *rx,
	uint16_t len, spi_ytm32_async_cb_t cb, void *user_data);

/**
 * @brief Start a non-blocking SPI transfer (ISR-safe).
 *
 * Starts a DMA-driven transfer and returns immediately. The callback fires
 * from the DMA completion ISR when all bytes have been transferred. This
 * compatibility API does not apply a device configuration or manage CS.
 * Prefer spi_ytm32_transceive_async_dt().
 *
 * @param dev       SPI device (e.g. cfg->spi.bus from a spi_dt_spec)
 * @param tx        Transmit buffer (caller-owned, valid until cb fires)
 * @param rx        Receive buffer  (caller-owned, valid until cb fires)
 * @param len       Transfer length in bytes (must be even for 16-bit words)
 * @param cb        Completion callback — called from ISR, must be brief
 * @param user_data Opaque value forwarded to cb
 * @return 0 on success, negative errno if the HAL rejected the transfer
 */
__deprecated int spi_ytm32_transceive_async(const struct device *dev,
			       const uint8_t *tx, uint8_t *rx, uint16_t len,
			       spi_ytm32_async_cb_t cb, void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SPI_SPI_YTM32_H_ */
