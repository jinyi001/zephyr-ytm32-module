/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include "system_YTM32B1MC0.h"
#include "clock.h"
#include <zephyr/init.h>

#define YTM32_FIRC_HZ 80000000U
#define YTM32_FXOSC_HZ 24000000U
#define YTM32_CLOCK_CONFIG_COUNT 1U
#define YTM32_CLOCK_CONFIG_INDEX 0U

#if defined(CONFIG_CLOCK_CONTROL_YTM32_CMU)
#define YTM32_CMU_ENABLED true
#else
#define YTM32_CMU_ENABLED false
#endif

#if defined(CONFIG_CLOCK_CONTROL_YTM32_CMU) && \
	defined(CONFIG_CLOCK_CONTROL_YTM32_CMU_RESET)
#define YTM32_CMU_RESET_ENABLED true
#else
#define YTM32_CMU_RESET_ENABLED false
#endif

#if defined(CONFIG_CLOCK_CONTROL_YTM32_KEEP_SIRC_IN_DEEPSLEEP)
#define YTM32_SIRC_DEEPSLEEP_ENABLED true
#else
#define YTM32_SIRC_DEEPSLEEP_ENABLED false
#endif

#if defined(CONFIG_CLOCK_CONTROL_YTM32_KEEP_SIRC_IN_STANDBY)
#define YTM32_SIRC_STANDBY_ENABLED true
#else
#define YTM32_SIRC_STANDBY_ENABLED false
#endif

static const cmu_config_t ytm32_cmu_config = {
	.fircClockMonitor = {
		.enable = YTM32_CMU_ENABLED,
		.resetEnable = YTM32_CMU_RESET_ENABLED,
		.refClock = CMU_REF_SIRC_CLOCK,
		.compareHigh = (100U * 128U) / 2U,
		.compareLow = (60U * 128U) / 2U,
	},
	.fxoscClockMonitor = {
		.enable = YTM32_CMU_ENABLED,
		.resetEnable = YTM32_CMU_RESET_ENABLED,
		.refClock = CMU_REF_SIRC_CLOCK,
		.compareHigh = (30U * 128U) / 2U,
		.compareLow = (18U * 128U) / 2U,
	},
};

static bool ytm32_divider_to_sys_div(uint32_t divider, uint8_t *sys_div)
{
	if ((divider == 0U) || (divider > 16U) || (sys_div == NULL)) {
		return false;
	}

	*sys_div = (uint8_t)(divider - 1U);
	return true;
}

void soc_early_init_hook(void)
{
	SystemInit();
}

int ytm32_soc_apply_clock_config(uint32_t core_clock,
				  uint32_t core_divider,
				  uint32_t fast_bus_divider,
				  uint32_t slow_bus_divider)
{
	scu_config_t scu_config = {
		.fircEnable = true,
		.fircDeepSleepEnable = false,
		.sircDeepSleepEnable = YTM32_SIRC_DEEPSLEEP_ENABLED,
		.sircStandbyEnable = YTM32_SIRC_STANDBY_ENABLED,
		.sircPowerDownEnable = false,
		.sysClkSrc = SCU_SYSTEM_CLOCK_SRC_FIRC,
		.fxoscConfig = {
			.enable = true,
			.bypassMode = false,
			.gainSelection = 6,
			.frequency = YTM32_FXOSC_HZ,
		},
		.sysDiv = SCU_SYS_CLK_DIV_BY_1,
		.fastBusDiv = SCU_SYS_CLK_DIV_BY_1,
		.slowBusDiv = SCU_SYS_CLK_DIV_BY_1,
		.flashDiv = SCU_SYS_CLK_DIV_BY_4,
		.clockOutConfig = {
			.enable = false,
			.source = SCU_CLKOUT_SEL_FIRC_CLK,
			.divider = 1U,
		},
	};
	clock_manager_user_config_t clock_config = {
		.scuConfigPtr = &scu_config,
		.cmuConfigPtr = &ytm32_cmu_config,
		.ipcConfig = {
			.peripheralClocks = NULL,
			.count = 0U,
		},
	};
	const clock_manager_user_config_t *clock_configs[YTM32_CLOCK_CONFIG_COUNT] = {
		&clock_config,
	};
	status_t status;

	if (core_clock != YTM32_FIRC_HZ) {
		return -EINVAL;
	}

	if (!ytm32_divider_to_sys_div(core_divider, &scu_config.sysDiv)) {
		return -EINVAL;
	}

	if (!ytm32_divider_to_sys_div(fast_bus_divider, &scu_config.fastBusDiv)) {
		return -EINVAL;
	}

	if (!ytm32_divider_to_sys_div(slow_bus_divider, &scu_config.slowBusDiv)) {
		return -EINVAL;
	}

	status = CLOCK_SYS_Init(clock_configs, YTM32_CLOCK_CONFIG_COUNT, NULL, 0U);
	if (status != STATUS_SUCCESS) {
		return -EIO;
	}

	status = CLOCK_SYS_UpdateConfiguration(YTM32_CLOCK_CONFIG_INDEX,
					      CLOCK_MANAGER_POLICY_AGREEMENT);
	if (status != STATUS_SUCCESS) {
		return -EIO;
	}

	return 0;
}
