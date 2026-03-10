/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ytmicro_ytm32_cgu

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_ytm32, CONFIG_CLOCK_CONTROL_LOG_LEVEL);
#include <zephyr/drivers/clock_control.h>
#include <zephyr/device.h>
#include <zephyr/arch/cpu.h>

/* Forward declare vendor API to avoid pulling in conflicting vendor CMSIS headers */
extern void CLOCK_DRV_SetModuleClock(uint32_t clockName, bool clockGate, uint32_t clkSrc, uint32_t divider);

/**
 * @brief Enable a specific clock
 * 
 * @param dev Clock controller device
 * @param sys Clock identifier (cast from uint32_t)
 * @return 0 on success, negative errno on failure
 */
static int clock_control_ytm32_on(const struct device *dev, clock_control_subsys_t sys)
{
	uint32_t clock_id = (uint32_t)(uintptr_t)sys;
	
	/* MVP implementation logic: dynamic enablement */
	LOG_INF("Enabling YTM32 clock id %u", clock_id);

	/* 
	 * Call vendor SDK clock setup for exactly this peripheral.
	 * Using hardcoded 3 (CLK_SRC_FXOSC) and 0 (DIV_BY_1) to avoid including vendor enums.
	 */
	CLOCK_DRV_SetModuleClock((uint32_t)clock_id, true, 3, 0);

	return 0;
}

static const struct clock_control_driver_api clock_control_ytm32_api = {
	.on = clock_control_ytm32_on,
	.off = NULL,
	.get_rate = NULL,
};

static int clock_control_ytm32_init(const struct device *dev)
{
	LOG_DBG("YTM32 CGU Initialized");
	return 0;
}

#define YTM32_CGU_INIT(n) \
	DEVICE_DT_INST_DEFINE(n, clock_control_ytm32_init, NULL, NULL, NULL, \
			      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY, \
			      &clock_control_ytm32_api);

DT_INST_FOREACH_STATUS_OKAY(YTM32_CGU_INIT)
