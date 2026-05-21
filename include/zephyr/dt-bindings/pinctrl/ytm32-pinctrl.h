/*
 * Copyright (c) 2026 YTMicro
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_YTM32_PINCTRL_WRAPPER_H_
#define ZEPHYR_YTM32_PINCTRL_WRAPPER_H_

#if defined(CONFIG_SOC_YTM32B1MC0)
#include <zephyr/dt-bindings/pinctrl/ytm32b1mc0-pinctrl.h>
#elif defined(CONFIG_SOC_YTM32B1MD1)
#include <zephyr/dt-bindings/pinctrl/ytm32b1md1-pinctrl.h>
#else
#include <zephyr/dt-bindings/pinctrl/ytm32b1mc0-pinctrl.h>
#endif

#endif /* ZEPHYR_YTM32_PINCTRL_WRAPPER_H_ */
