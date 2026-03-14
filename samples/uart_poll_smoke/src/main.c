/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define UART_NODE DT_CHOSEN(zephyr_console)
#define RX_BUF_LEN 64

static const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);

static void uart_write_string(const char *str)
{
	while (*str != '\0') {
		uart_poll_out(uart_dev, (unsigned char)*str++);
	}
}

int main(void)
{
	unsigned char c;
	char rx_buf[RX_BUF_LEN];
	size_t pos = 0U;

	if (!device_is_ready(uart_dev)) {
		printk("FAIL: console UART is not ready\n");
		return 0;
	}

	printk("YTM32 uart_poll_smoke\n");
	uart_write_string("TX check: This is a POLL test.\r\n");
	uart_write_string("Type a short line and press Enter.\r\n");
	uart_write_string("Characters will be echoed by uart_poll_out().\r\n");

	while (1) {
		if (uart_poll_in(uart_dev, &c) < 0) {
			k_msleep(1);
			continue;
		}

		if ((c == '\r') || (c == '\n')) {
			uart_write_string("\r\n");
			rx_buf[pos] = '\0';
			printk("PASS: received line '%s'\n", rx_buf);
			break;
		}

		if (pos < (RX_BUF_LEN - 1U)) {
			rx_buf[pos++] = (char)c;
		}

		uart_poll_out(uart_dev, c);
	}

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
