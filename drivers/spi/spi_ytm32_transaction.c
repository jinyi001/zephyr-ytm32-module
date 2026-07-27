/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include "spi_ytm32_transaction.h"

void spi_ytm32_transaction_init(struct spi_ytm32_transaction *transaction)
{
	k_sem_init(&transaction->gate, 1, 1);
	atomic_set(&transaction->blocking_waiters, 0);
	atomic_set(&transaction->kind, SPI_YTM32_TRANSFER_IDLE);
}

int spi_ytm32_transaction_blocking_acquire(
	struct spi_ytm32_transaction *transaction, k_timeout_t timeout)
{
	int ret;
	k_spinlock_key_t key;

	/* Publish the waiter before checking the gate so async cannot overtake it. */
	key = k_spin_lock(&transaction->lock);
	atomic_inc(&transaction->blocking_waiters);
	k_spin_unlock(&transaction->lock, key);

	ret = k_sem_take(&transaction->gate, timeout);
	atomic_dec(&transaction->blocking_waiters);

	if (ret < 0) {
		return -ETIMEDOUT;
	}

	atomic_set(&transaction->kind, SPI_YTM32_TRANSFER_BLOCKING);
	return 0;
}

int spi_ytm32_transaction_async_try_acquire(
	struct spi_ytm32_transaction *transaction)
{
	k_spinlock_key_t key = k_spin_lock(&transaction->lock);

	if (spi_ytm32_transaction_blocking_waiting(transaction)) {
		k_spin_unlock(&transaction->lock, key);
		return -EBUSY;
	}

	if (k_sem_take(&transaction->gate, K_NO_WAIT) < 0) {
		k_spin_unlock(&transaction->lock, key);
		return -EBUSY;
	}

	atomic_set(&transaction->kind, SPI_YTM32_TRANSFER_ASYNC);
	k_spin_unlock(&transaction->lock, key);
	return 0;
}

void spi_ytm32_transaction_release(struct spi_ytm32_transaction *transaction)
{
	atomic_set(&transaction->kind, SPI_YTM32_TRANSFER_IDLE);
	k_sem_give(&transaction->gate);
}

bool spi_ytm32_transaction_blocking_waiting(
	const struct spi_ytm32_transaction *transaction)
{
	return atomic_get(&transaction->blocking_waiters) > 0;
}

enum spi_ytm32_transfer_kind spi_ytm32_transaction_kind(
	const struct spi_ytm32_transaction *transaction)
{
	return (enum spi_ytm32_transfer_kind)atomic_get(&transaction->kind);
}
