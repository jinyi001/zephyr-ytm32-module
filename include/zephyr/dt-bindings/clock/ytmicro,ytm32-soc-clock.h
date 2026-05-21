/*
 * Copyright (c) 2026 YTMicro
 * SPDX-License-Identifier: Apache-2.0
 *
 * SoC-specific clock binding selection wrapper.
 * C code includes this to get the correct YTM32_CLOCK_* constants
 * for the active SoC. The DTS layer uses per-SoC headers directly.
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_YTM32_SOC_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_YTM32_SOC_CLOCK_H_

#if defined(CONFIG_SOC_YTM32B1MD1)
#include <zephyr/dt-bindings/clock/ytmicro,ytm32b1md1-clock.h>
#elif defined(CONFIG_SOC_YTM32B1MC0)
#include <zephyr/dt-bindings/clock/ytmicro,ytm32b1mc0-clock.h>
#else
#include <zephyr/dt-bindings/clock/ytmicro,ytm32b1mc0-clock.h>
#endif

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_YTM32_SOC_CLOCK_H_ */
