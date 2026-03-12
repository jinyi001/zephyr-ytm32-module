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
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pinctrl_ytm32, CONFIG_PINCTRL_LOG_LEVEL);

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0; i < pin_cnt; i++) {
		pin_settings_config_t config;
		uint32_t pinmux = pins[i].pinmux;
		uint32_t port = (pinmux >> 28) & 0xF;
		uint32_t pin = (pinmux >> 24) & 0x1F;
		uint32_t mux = pinmux & 0xF;

		void *base = NULL;
		void *gpio_base = NULL;

		/*
		 * Map port index (from DTS pinmux encoding) to vendor SDK
		 * PCTRL/GPIO base pointers.
		 *
		 * These global macros (PCTRLA, GPIOA, etc.) are defined in the
		 * vendor SDK header YTM32B1MC0.h and must not be modified.
		 * Port index encoding: 0=A, 1=B, 2=C, 3=D, 4=E.
		 */
		switch (port) {
		case 0:
			base = PCTRLA;
			gpio_base = GPIOA;
			break;
		case 1:
			base = PCTRLB;
			gpio_base = GPIOB;
			break;
		case 2:
			base = PCTRLC;
			gpio_base = GPIOC;
			break;
		case 3:
			base = PCTRLD;
			gpio_base = GPIOD;
			break;
		case 4:
			base = PCTRLE;
			gpio_base = GPIOE;
			break;
		default:
			return -EINVAL;
		}

		LOG_DBG("pin: port=%u pin=%u mux=%u pull=0x%02x",
			port, pin, mux, pins[i].pincfg);

		config.base = base;
		config.pinPortIdx = pin;
		config.mux = mux;

		/* Pull configuration */
		if (pins[i].pincfg & YTM32_PULL_UP_MSK) {
			config.pullConfig = PCTRL_INTERNAL_PULL_UP_ENABLED;
		} else if (pins[i].pincfg & YTM32_PULL_DOWN_MSK) {
			config.pullConfig = PCTRL_INTERNAL_PULL_DOWN_ENABLED;
		} else {
			config.pullConfig = PCTRL_INTERNAL_PULL_NOT_ENABLED;
		}

#if FEATURE_PINS_HAS_DRIVE_STRENGTH
		config.driveSelect = (pins[i].pincfg & YTM32_DRV_STR_MSK) ?
			PCTRL_HIGH_DRIVE_STRENGTH : PCTRL_LOW_DRIVE_STRENGTH;
#endif

#if FEATURE_PINS_HAS_OPEN_DRAIN
		config.openDrain = (pins[i].pincfg & YTM32_OPEN_DRAIN_MSK) ?
			PCTRL_OPEN_DRAIN_ENABLED : PCTRL_OPEN_DRAIN_DISABLED;
#endif

#if FEATURE_PINS_HAS_SLEW_RATE
		config.rateSelect = (pins[i].pincfg & YTM32_SLEW_RATE_MSK) ?
			PCTRL_FAST_SLEW_RATE : PCTRL_SLOW_SLEW_RATE;
#endif

#if FEATURE_PINS_HAS_PASSIVE_FILTER
		config.passiveFilter = !!(pins[i].pincfg & YTM32_PASSIVE_FLT_MSK);
#endif

		/* Fixed defaults for parameters not currently needed in pinctrl */
		config.intConfig = PCTRL_DMA_INT_DISABLED;
		config.clearIntFlag = false;
		config.digitalFilter = false;
		config.gpioBase = gpio_base;
		config.direction = GPIO_INPUT_DIRECTION;
		config.initValue = 0;

		status_t status = PINS_DRV_Init(1U, &config);
		if (status != STATUS_SUCCESS) {
			return -EIO;
		}
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
