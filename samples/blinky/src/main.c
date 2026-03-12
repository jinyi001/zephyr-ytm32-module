/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Blink LED demo for YTM32B1MC0-EVB-Q64.
 * Toggles the red LED (PTD5) at 500ms interval.
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* Get the LED0 alias from devicetree (mapped to led_red / PTD5) */
#define LED0_NODE DT_ALIAS(led0)

#if !DT_NODE_HAS_STATUS_OKAY(LED0_NODE)
#error "LED0 alias is not defined or not enabled in the devicetree"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	int ret;

	printf("Blinky demo starting on %s\n", CONFIG_BOARD_TARGET);

	if (!gpio_is_ready_dt(&led)) {
		printf("Error: LED GPIO device is not ready\n");
		return -1;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		printf("Error %d: failed to configure LED pin\n", ret);
		return ret;
	}

	printf("Blinking LED at PTD5 (Red LED)...\n");

	while (1) {
		gpio_pin_toggle_dt(&led);
		k_msleep(500);
	}

	return 0;
}
