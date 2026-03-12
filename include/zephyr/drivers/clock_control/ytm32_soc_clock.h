/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared definitions between the YTM32 SoC init code and the YTM32 clock
 * control driver.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_YTM32_SOC_CLOCK_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_YTM32_SOC_CLOCK_H_

#include <stdint.h>
#include <zephyr/sys/util.h>

#define YTM32_CMU_ENABLED \
	IS_ENABLED(CONFIG_CLOCK_CONTROL_YTM32_CMU)

#define YTM32_CMU_RESET_ENABLED \
	(YTM32_CMU_ENABLED && IS_ENABLED(CONFIG_CLOCK_CONTROL_YTM32_CMU_RESET))

#define YTM32_SIRC_DEEPSLEEP_ENABLED \
	IS_ENABLED(CONFIG_CLOCK_CONTROL_YTM32_KEEP_SIRC_IN_DEEPSLEEP)

#define YTM32_SIRC_STANDBY_ENABLED \
	IS_ENABLED(CONFIG_CLOCK_CONTROL_YTM32_KEEP_SIRC_IN_STANDBY)

int ytm32_soc_apply_clock_config(uint32_t core_clock,
				 uint32_t core_divider,
				 uint32_t fast_bus_divider,
				 uint32_t slow_bus_divider);

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_YTM32_SOC_CLOCK_H_ */
