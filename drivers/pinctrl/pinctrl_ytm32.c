/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ytmicro_ytm32_pinctrl

#include <errno.h>
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
		config[i].mux = mux;

		/* Pull configuration */
		if (pins[i].pincfg & YTM32_PULL_UP_MSK) {
			config[i].pullConfig = PCTRL_INTERNAL_PULL_UP_ENABLED;
		} else if (pins[i].pincfg & YTM32_PULL_DOWN_MSK) {
			config[i].pullConfig = PCTRL_INTERNAL_PULL_DOWN_ENABLED;
		} else {
			config[i].pullConfig = PCTRL_INTERNAL_PULL_NOT_ENABLED;
		}

		/* Drive strength */
#if FEATURE_PINS_HAS_DRIVE_STRENGTH
		if (pins[i].pincfg & YTM32_DRV_STR_MSK) {
			config[i].driveSelect = PCTRL_HIGH_DRIVE_STRENGTH;
		} else {
			config[i].driveSelect = PCTRL_LOW_DRIVE_STRENGTH;
		}
#endif

		/* Open drain */
#if FEATURE_PINS_HAS_OPEN_DRAIN
		if (pins[i].pincfg & YTM32_OPEN_DRAIN_MSK) {
			config[i].openDrain = PCTRL_OPEN_DRAIN_ENABLED;
		} else {
			config[i].openDrain = PCTRL_OPEN_DRAIN_DISABLED;
		}
#endif

		/* Slew rate */
#if FEATURE_PINS_HAS_SLEW_RATE
		if (pins[i].pincfg & YTM32_SLEW_RATE_MSK) {
			config[i].rateSelect = PCTRL_FAST_SLEW_RATE;
		} else {
			config[i].rateSelect = PCTRL_SLOW_SLEW_RATE;
		}
#endif

		/* Passive filter */
#if FEATURE_PINS_HAS_PASSIVE_FILTER
		config[i].passiveFilter = (pins[i].pincfg & YTM32_PASSIVE_FLT_MSK) ? true : false;
#endif
		
		/* Fixed defaults for parameters not currently needed in pinctrl */
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
	int ret = 0;

	if (!device_is_ready(cgu)) {
		return -ENODEV;
	}

#define PINCTRL_NODE DT_NODELABEL(pinctrl)

	/* Enable clocks specified in devicetree */
#if DT_NODE_HAS_PROP(PINCTRL_NODE, clocks)
#define CLOCK_INIT(node_id, prop, idx) \
	do { \
		if (ret == 0) { \
			ret = clock_control_on(cgu, \
				(clock_control_subsys_t)DT_CLOCKS_CELL_BY_IDX(node_id, idx, id)); \
		} \
	} while (0);
	DT_FOREACH_PROP_ELEM(PINCTRL_NODE, clocks, CLOCK_INIT)
#undef CLOCK_INIT
	if (ret < 0) {
		return ret;
	}
#endif

	return 0;
}

SYS_INIT(pinctrl_ytm32_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
