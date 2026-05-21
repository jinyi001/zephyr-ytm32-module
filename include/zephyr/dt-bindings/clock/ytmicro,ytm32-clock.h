/*
 * Copyright (c) 2026 YTMicro
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_YTM32_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_YTM32_CLOCK_H_

#if defined(CONFIG_SOC_YTM32B1MD1)
#include <zephyr/dt-bindings/clock/ytmicro,ytm32b1md1-clock.h>
#elif defined(CONFIG_SOC_YTM32B1MC0)
#include <zephyr/dt-bindings/clock/ytmicro,ytm32b1mc0-clock.h>
#else
#include <zephyr/dt-bindings/clock/ytmicro,ytm32b1mc0-clock.h>
#endif

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_YTM32_CLOCK_H_ */
