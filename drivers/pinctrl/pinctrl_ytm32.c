/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ytmicro_ytm32_pinctrl

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/init.h>
#include "pins_driver.h"

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	pin_settings_config_t config[pin_cnt];

	for (uint8_t i = 0; i < pin_cnt; i++) {
		uint32_t pinmux = pins[i].pinmux;
		uint32_t port = (pinmux >> 28) & 0xF;
		uint32_t pin = (pinmux >> 24) & 0x1F;
		uint32_t mux = pinmux & 0xF;

		void *base = NULL;
		void *gpio_base = NULL;
		switch(port) {
			case 0: base = PCTRLA; gpio_base = GPIOA; break;
			case 1: base = PCTRLB; gpio_base = GPIOB; break;
			case 2: base = PCTRLC; gpio_base = GPIOC; break;
			case 3: base = PCTRLD; gpio_base = GPIOD; break;
			case 4: base = PCTRLE; gpio_base = GPIOE; break;
			default: return -EINVAL;
		}

		config[i].base = base;
		config[i].pinPortIdx = pin;
		config[i].pullConfig = PCTRL_INTERNAL_PULL_NOT_ENABLED;
		config[i].passiveFilter = false;
		config[i].driveSelect = PCTRL_LOW_DRIVE_STRENGTH;
		config[i].mux = mux;
		config[i].intConfig = PCTRL_DMA_INT_DISABLED;
		config[i].clearIntFlag = false;
		config[i].digitalFilter = false;
		config[i].gpioBase = gpio_base; /* Required by vendor SDK to clear IRQ status even if not GPIO */
		config[i].direction = GPIO_INPUT_DIRECTION;
		config[i].initValue = 0;
	}

	status_t status = PINS_DRV_Init(pin_cnt, config);
	if (status != STATUS_SUCCESS) {
		return -EIO;
	}

	return 0;
}

static int pinctrl_ytm32_init(void)
{
	const struct device *cgu = DEVICE_DT_GET(DT_NODELABEL(cgu));

	if (!device_is_ready(cgu)) {
		return -ENODEV;
	}

	/* 
	 * Enable clocks for PCTRLA (2U) through PCTRLE (6U) 
	 * The raw IDs are from clock_names_t in vendor HAL.
	 */
	for (uint32_t i = 2U; i <= 6U; i++) {
		clock_control_on(cgu, (clock_control_subsys_t)i);
	}

	/* Enable GPIO clock (1U) */
	clock_control_on(cgu, (clock_control_subsys_t)1U);

	return 0;
}

SYS_INIT(pinctrl_ytm32_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
