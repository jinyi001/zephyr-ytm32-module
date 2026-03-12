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

#define YTM32_STATUS_SUCCESS 0
#define YTM32_FIRC_HZ 80000000U

/* Forward declare vendor API to avoid pulling in conflicting vendor CMSIS headers */
extern void CLOCK_DRV_SetModuleClock(uint32_t clockName, bool clockGate, uint32_t clkSrc, uint32_t divider);
extern int CLOCK_SYS_GetFreq(uint32_t clockName, uint32_t *frequency);

struct clock_control_ytm32_config {
	uint32_t core_clock;
	uint32_t core_divider;
	uint32_t bus_divider;
};

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
	uint32_t clk_src = 3; /* Default to FXOSC */
	uint32_t divider = 0; /* Default DIV_BY_1 */
	
	LOG_INF("Enabling YTM32 clock id %u", clock_id);

	/* 
	 * Simple mapping logic for MVP:
	 * UARTs, SPIs usually run on FIRC or FAST_BUS.
	 * Here we just use a basic rule: if it's an IPC clock, we try to assign FIRC (clock src 1 for example)
	 * For now, just a placeholder lookup based on clock_id.
	 */
	if (clock_id >= 7 && clock_id <= 9) {
		/* UART0-2: uses FIRC as clock source */
		clk_src = 1; /* FIRC */
		divider = 0;
	} else if (clock_id >= 2 && clock_id <= 6) {
		/* PCTRL A-E: uses FXOSC */
		clk_src = 3; /* FXOSC */
		divider = 0;
	} else if (clock_id == 1) {
		/* GPIO: uses FIRC (matching vendor demo clock_config) */
		clk_src = 1; /* FIRC */
		divider = 0;
	}

	CLOCK_DRV_SetModuleClock((uint32_t)clock_id, true, clk_src, divider);

	return 0;
}

static int clock_control_ytm32_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate)
{
	uint32_t clock_id = (uint32_t)(uintptr_t)sys;
	int status;

	(void)dev;

	if (rate == NULL) {
		return -EINVAL;
	}

	status = CLOCK_SYS_GetFreq(clock_id, rate);
	if (status != YTM32_STATUS_SUCCESS) {
		LOG_ERR("Failed to query clock id %u (status %d)", clock_id, status);
		return -EIO;
	}

	return 0;
}

static const struct clock_control_driver_api clock_control_ytm32_api = {
	.on = clock_control_ytm32_on,
	.off = NULL,
	.get_rate = clock_control_ytm32_get_rate,
};

static int clock_control_ytm32_init(const struct device *dev)
{
	const struct clock_control_ytm32_config *cfg = dev->config;

	if (cfg->core_clock != YTM32_FIRC_HZ) {
		LOG_ERR("Unsupported core clock %u Hz, MVP supports %u Hz FIRC only",
			cfg->core_clock, YTM32_FIRC_HZ);
		return -EINVAL;
	}

	LOG_INF("YTM32 CGU Initialized, Target Core Clock: %u Hz", cfg->core_clock);
	LOG_INF("Core Divider: %u, Bus Divider: %u", cfg->core_divider, cfg->bus_divider);
	LOG_INF("YTM32 clock MVP locked to FIRC 80MHz baseline");
	return 0;
}

#define YTM32_CGU_INIT(n) \
	static const struct clock_control_ytm32_config clock_control_ytm32_config_##n = { \
		.core_clock = DT_INST_PROP(n, core_clock), \
		.core_divider = DT_INST_PROP(n, core_divider), \
		.bus_divider = DT_INST_PROP(n, bus_divider), \
	}; \
	DEVICE_DT_INST_DEFINE(n, clock_control_ytm32_init, NULL, NULL, &clock_control_ytm32_config_##n, \
			      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY, \
			      &clock_control_ytm32_api);

DT_INST_FOREACH_STATUS_OKAY(YTM32_CGU_INIT)
