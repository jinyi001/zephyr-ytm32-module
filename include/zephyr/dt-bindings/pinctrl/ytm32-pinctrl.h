/*
 * Copyright (c) 2026 YTMicro
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_YTM32_PINCTRL_WRAPPER_H_
#define ZEPHYR_YTM32_PINCTRL_WRAPPER_H_

/*
 * YTM32 pinctrl entry header.
 *
 * Currently the only supported SoC series is YTM32B1M (covering YTM32B1MC0).
 * When a second series is added, convert this to an ifdef/elif dispatch keyed
 * on CONFIG_SOC_SERIES_*.
 */
#include <zephyr/dt-bindings/pinctrl/ytm32b1m-pinctrl.h>

#endif /* ZEPHYR_YTM32_PINCTRL_WRAPPER_H_ */
