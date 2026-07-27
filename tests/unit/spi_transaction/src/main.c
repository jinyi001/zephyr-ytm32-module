/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "spi_ytm32_transaction.h"

struct blocking_thread_context {
	struct spi_ytm32_transaction *transaction;
	int acquire_result;
	struct k_sem finished;
};

struct fake_spi_config {
	uint32_t frequency;
	uint8_t word_size;
	uint8_t cs;
};

struct fake_async_bus {
	struct spi_ytm32_transaction transaction;
	struct fake_spi_config active_config;
	bool cs_active[2];
	int pending_cs;
	bool pending;
	int callback_count;
	int last_status;
};

K_THREAD_STACK_DEFINE(blocking_thread_stack, 1024);
static struct k_thread blocking_thread;
static struct spi_ytm32_transaction transaction;
static struct fake_async_bus fake_bus;

static void fake_async_callback(void *user_data, int status)
{
	struct fake_async_bus *bus = user_data;

	bus->callback_count++;
	bus->last_status = status;
}

/*
 * This fake models the extracted transaction contract used by the real
 * driver: configuration and CS are applied while the gate is owned, and the
 * completion path releases the gate before invoking the device callback.
 */
static int fake_async_start(struct fake_async_bus *bus,
				    const struct fake_spi_config *config,
				    int start_status)
{
	int ret;

	ret = spi_ytm32_transaction_async_try_acquire(&bus->transaction);
	if (ret < 0) {
		return ret;
	}

	bus->active_config = *config;
	bus->pending_cs = config->cs;
	zassert_true(config->cs < ARRAY_SIZE(bus->cs_active),
		     "fake CS index out of range");
	for (size_t i = 0; i < ARRAY_SIZE(bus->cs_active); i++) {
		zassert_false(bus->cs_active[i], "CS overlap before assert");
	}
	bus->cs_active[config->cs] = true;

	if (start_status < 0) {
		bus->cs_active[config->cs] = false;
		bus->pending_cs = -1;
		spi_ytm32_transaction_release(&bus->transaction);
		return start_status;
	}

	bus->pending = true;
	return 0;
}

static void fake_async_complete(struct fake_async_bus *bus, int status)
{
	if (!bus->pending) {
		return;
	}

	bus->pending = false;
	bus->cs_active[bus->pending_cs] = false;
	bus->pending_cs = -1;
	spi_ytm32_transaction_release(&bus->transaction);
	fake_async_callback(bus, status);
}

static void blocking_thread_entry(void *p1, void *p2, void *p3)
{
	struct blocking_thread_context *context = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	context->acquire_result = spi_ytm32_transaction_blocking_acquire(
		context->transaction, K_MSEC(200));
	if (context->acquire_result == 0) {
		spi_ytm32_transaction_release(context->transaction);
	}
	k_sem_give(&context->finished);
}

static void *spi_transaction_setup(void)
{
	spi_ytm32_transaction_init(&transaction);
	spi_ytm32_transaction_init(&fake_bus.transaction);
	fake_bus.active_config = (struct fake_spi_config){0};
	fake_bus.cs_active[0] = false;
	fake_bus.cs_active[1] = false;
	fake_bus.pending_cs = -1;
	fake_bus.pending = false;
	fake_bus.callback_count = 0;
	fake_bus.last_status = 0;
	return NULL;
}

static void spi_transaction_wait_for_blocking_waiter(void)
{
	for (int i = 0; i < 100; i++) {
		if (spi_ytm32_transaction_blocking_waiting(&transaction)) {
			return;
		}
		k_sleep(K_MSEC(1));
	}

	zassert_true(false, "blocking waiter did not become observable");
}

static void spi_transaction_start_blocking_thread(
	struct blocking_thread_context *context)
{
	k_sem_init(&context->finished, 0, 1);
	context->transaction = &transaction;
	context->acquire_result = -EINPROGRESS;

	k_thread_create(&blocking_thread, blocking_thread_stack,
			K_THREAD_STACK_SIZEOF(blocking_thread_stack),
			blocking_thread_entry, context, NULL, NULL,
			K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
}

ZTEST(spi_transaction, test_async_owns_gate_from_blocking_path)
{
	struct blocking_thread_context context;

	zassert_ok(spi_ytm32_transaction_async_try_acquire(&transaction));
	zassert_equal(spi_ytm32_transaction_kind(&transaction),
		      SPI_YTM32_TRANSFER_ASYNC);

	spi_transaction_start_blocking_thread(&context);
	spi_transaction_wait_for_blocking_waiter();
	zassert_equal(spi_ytm32_transaction_async_try_acquire(&transaction),
		      -EBUSY);

	spi_ytm32_transaction_release(&transaction);
	zassert_ok(k_sem_take(&context.finished, K_MSEC(500)));
	zassert_ok(context.acquire_result);
}

ZTEST(spi_transaction, test_blocking_owner_rejects_async)
{
	zassert_ok(spi_ytm32_transaction_blocking_acquire(&transaction,
							K_NO_WAIT));
	zassert_equal(spi_ytm32_transaction_kind(&transaction),
		      SPI_YTM32_TRANSFER_BLOCKING);
	zassert_equal(spi_ytm32_transaction_async_try_acquire(&transaction),
		      -EBUSY);

	spi_ytm32_transaction_release(&transaction);
	zassert_ok(spi_ytm32_transaction_async_try_acquire(&transaction));
	spi_ytm32_transaction_release(&transaction);
}

ZTEST(spi_transaction, test_blocking_timeout_releases_waiter)
{
	zassert_ok(spi_ytm32_transaction_async_try_acquire(&transaction));
	zassert_equal(spi_ytm32_transaction_blocking_acquire(
			&transaction, K_MSEC(5)), -ETIMEDOUT);
	zassert_false(spi_ytm32_transaction_blocking_waiting(&transaction));

	spi_ytm32_transaction_release(&transaction);
	zassert_ok(spi_ytm32_transaction_blocking_acquire(&transaction,
							K_NO_WAIT));
	spi_ytm32_transaction_release(&transaction);
}

ZTEST(spi_transaction, test_async_config_switch_cs_and_error_recovery)
{
	const struct fake_spi_config encoder = {
		.frequency = 1000000U,
		.word_size = 8U,
		.cs = 0U,
	};
	const struct fake_spi_config gate_driver = {
		.frequency = 10000000U,
		.word_size = 8U,
		.cs = 1U,
	};

	zassert_ok(fake_async_start(&fake_bus, &encoder, 0));
	zassert_equal(fake_bus.active_config.frequency, 1000000U);
	zassert_equal(fake_bus.active_config.word_size, 8U);
	zassert_true(fake_bus.cs_active[0]);
	zassert_false(fake_bus.cs_active[1]);
	fake_async_complete(&fake_bus, 0);
	zassert_equal(fake_bus.callback_count, 1);
	zassert_equal(fake_bus.last_status, 0);

	zassert_ok(fake_async_start(&fake_bus, &gate_driver, 0));
	zassert_equal(fake_bus.active_config.frequency, 10000000U);
	zassert_true(fake_bus.cs_active[1]);
	zassert_false(fake_bus.cs_active[0]);
	fake_async_complete(&fake_bus, -EIO);
	zassert_equal(fake_bus.callback_count, 2);
	zassert_equal(fake_bus.last_status, -EIO);

	/* A duplicate HAL completion must not invoke the callback twice. */
	fake_async_complete(&fake_bus, 0);
	zassert_equal(fake_bus.callback_count, 2);
	zassert_false(fake_bus.cs_active[0]);
	zassert_false(fake_bus.cs_active[1]);

	/* A synchronous HAL start failure must release CS and the gate. */
	zassert_equal(fake_async_start(&fake_bus, &encoder, -EIO), -EIO);
	zassert_false(fake_bus.cs_active[0]);
	zassert_false(fake_bus.cs_active[1]);
	zassert_equal(fake_bus.callback_count, 2);
	zassert_ok(spi_ytm32_transaction_blocking_acquire(
		&fake_bus.transaction, K_NO_WAIT));
	spi_ytm32_transaction_release(&fake_bus.transaction);
}

ZTEST_SUITE(spi_transaction, NULL, spi_transaction_setup, NULL, NULL, NULL);
