/*
 * Copyright (c) 2026 YTMicro
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_YTM32_PINCTRL_WRAPPER_H_
#define ZEPHYR_YTM32_PINCTRL_WRAPPER_H_

/* 
 * Multi-chip compatibility layer wrapper
 * Includes specific SoC pinmux constraints based on the device tree compatible string.
 */

#if defined(YTM32B1MC0_PINCTRL_H_) || defined(YT_YTM32B1MC0) || __has_include(<ytmicro/ytm32/ytm32b1mc0.dtsi>)
/* We can't rely on Kconfig during DTS compilation reliably across all Zephyr versions */
#include <zephyr/dt-bindings/pinctrl/ytm32b1m-pinctrl.h>
#elif defined(CONFIG_SOC_SERIES_YTM32B1M)
#include <zephyr/dt-bindings/pinctrl/ytm32b1m-pinctrl.h>
#elif defined(CONFIG_SOC_SERIES_YTM32B1L)
/* #include <zephyr/dt-bindings/pinctrl/ytm32b1l-pinctrl.h> */
#else
#warning "Unsupported YTM32 SoC for pinctrl or SoC macro not defined via CMake/Kconfig"
#endif

#endif /* ZEPHYR_YTM32_PINCTRL_WRAPPER_H_ */
