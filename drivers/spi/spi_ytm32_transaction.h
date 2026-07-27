/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef YTM32_SPI_TRANSACTION_H_
#define YTM32_SPI_TRANSACTION_H_

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>

enum spi_ytm32_transfer_kind {
	SPI_YTM32_TRANSFER_IDLE = 0,
	SPI_YTM32_TRANSFER_BLOCKING,
	SPI_YTM32_TRANSFER_ASYNC,
};

/*
 * Hardware ownership is separate from spi_context.  spi_context serializes
 * callers in thread context, while this gate also covers ISR/DMA transfers.
 */
struct spi_ytm32_transaction {
	struct k_spinlock lock;
	struct k_sem gate;
	atomic_t blocking_waiters;
	atomic_t kind;
};

void spi_ytm32_transaction_init(struct spi_ytm32_transaction *transaction);

int spi_ytm32_transaction_blocking_acquire(
	struct spi_ytm32_transaction *transaction, k_timeout_t timeout);

int spi_ytm32_transaction_async_try_acquire(
	struct spi_ytm32_transaction *transaction);

void spi_ytm32_transaction_release(struct spi_ytm32_transaction *transaction);

bool spi_ytm32_transaction_blocking_waiting(
	const struct spi_ytm32_transaction *transaction);

enum spi_ytm32_transfer_kind spi_ytm32_transaction_kind(
	const struct spi_ytm32_transaction *transaction);

#endif /* YTM32_SPI_TRANSACTION_H_ */
