/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include "system_YTM32B1MC0.h"
#include "pins_driver.h"
#include "clock.h"

static const peripheral_clock_config_t clock_config0PeripheralClockConfig[1] = {
    {
        .clkName = UART1_CLK,
        .clkGate = true,
        .divider = DIV_BY_1,
        .clkSrc = CLK_SRC_FXOSC,
    },
};

static const scu_config_t clock_config0ScuConfig = {
    .fircEnable = true,
    .fircDeepSleepEnable = false,
    .sircDeepSleepEnable = true,
    .sircStandbyEnable = true,
    .sysClkSrc = SCU_SYSTEM_CLOCK_SRC_FIRC,
    .fxoscConfig =
        {
            .enable = true,
            .bypassMode = false,
            .gainSelection = 6,
            .frequency = 24000000U,
        },
    .sysDiv = SCU_SYS_CLK_DIV_BY_1,
    .fastBusDiv = SCU_SYS_CLK_DIV_BY_1,
    .slowBusDiv = SCU_SYS_CLK_DIV_BY_2,
    .flashDiv = SCU_SYS_CLK_DIV_BY_4,
    .clockOutConfig =
        {
            .enable = false,
            .source = SCU_CLKOUT_SEL_FIRC_CLK,
            .divider = 1
        },
};

static const cmu_config_t clock_config0CmuConfig = {
    .fircClockMonitor={
        .enable = true,                 
        .resetEnable = true,           
        .refClock = CMU_REF_SIRC_CLOCK,  
        .compareHigh = (100 * 128 / 2),        
        .compareLow = (60 * 128 / 2),  
    },
    .fxoscClockMonitor={
        .enable = true,                 
        .resetEnable = true,           
        .refClock = CMU_REF_SIRC_CLOCK,              
        .compareHigh = (30 * 128 / 2),        
        .compareLow = (18 * 128 / 2),         
    },
};

static const clock_manager_user_config_t clock_config0ClockManager = {
    .scuConfigPtr = &clock_config0ScuConfig,
    .cmuConfigPtr = &clock_config0CmuConfig,
    .ipcConfig =
        {
            .peripheralClocks = clock_config0PeripheralClockConfig,
            .count = 1,
        },
};

static const clock_manager_user_config_t *g_clockManConfigsArr[] = {
    &clock_config0ClockManager,
};

static clock_manager_callback_user_config_t *g_clockManCallbacksArr[] = {(void *)0};

#define NUM_OF_CONFIGURED_PINS0_UART (2U)

static const pin_settings_config_t g_pin_mux_InitConfigArr0_uart[NUM_OF_CONFIGURED_PINS0_UART] = {
    /* PTC_8-36-UART1_RX- */
    {
        .base = PCTRLC,
        .pinPortIdx = 8U,
        .pullConfig = PCTRL_INTERNAL_PULL_NOT_ENABLED,
        .passiveFilter = false,
        .driveSelect = PCTRL_LOW_DRIVE_STRENGTH,
        .mux = PCTRL_MUX_ALT2,
        .intConfig = PCTRL_DMA_INT_DISABLED,
        .clearIntFlag = false,
        .digitalFilter = false,
        .gpioBase = GPIOC,
        .direction = GPIO_INPUT_DIRECTION,
        .initValue = 0,
    },
    /* PTC_9-35-UART1_TX- */
    {
        .base = PCTRLC,
        .pinPortIdx = 9U,
        .pullConfig = PCTRL_INTERNAL_PULL_NOT_ENABLED,
        .passiveFilter = false,
        .driveSelect = PCTRL_LOW_DRIVE_STRENGTH,
        .mux = PCTRL_MUX_ALT2,
        .intConfig = PCTRL_DMA_INT_DISABLED,
        .clearIntFlag = false,
        .digitalFilter = false,
        .gpioBase = GPIOC,
        .direction = GPIO_INPUT_DIRECTION,
        .initValue = 0,
    },
};

void soc_early_init_hook(void)
{
	SystemInit();
}

static int ytm32_soc_init(void)
{
	CLOCK_SYS_Init(g_clockManConfigsArr, 1U, g_clockManCallbacksArr, 0U);
	CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

	PINS_DRV_Init(NUM_OF_CONFIGURED_PINS0_UART, g_pin_mux_InitConfigArr0_uart);
	return 0;
}

SYS_INIT(ytm32_soc_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);


