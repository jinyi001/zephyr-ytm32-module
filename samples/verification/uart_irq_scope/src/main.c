/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define UART_NODE DT_CHOSEN(zephyr_console)

static const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);

static void dummy_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
}

int main(void)
{
	int ret;

	if (!device_is_ready(uart_dev)) {
		printk("FAIL: console UART is not ready\n");
		return 0;
	}

	printk("YTM32 uart_irq_scope\n");
	ret = uart_irq_callback_user_data_set(uart_dev, dummy_cb, NULL);

	if ((ret == -ENOTSUP) || (ret == -ENOSYS)) {
		printk("PASS: interrupt-driven UART API is outside current MVP scope (%d)\n",
		       ret);
	} else if (ret == 0) {
		printk("NOTE: interrupt-driven UART API is available now; revisit test scope\n");
	} else {
		printk("FAIL: unexpected uart_irq_callback_user_data_set() return %d\n", ret);
	}

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
