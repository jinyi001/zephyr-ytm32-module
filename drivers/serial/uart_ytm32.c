/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ytmicro_ytm32_uart

#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>
#include <errno.h>

#define YTM32_UART_MAX_FUNCTIONAL_CLOCK_HZ 40000000U

/*
 * Derive the vendor HAL UART instance index from the DTS reg address.
 * The YTM32B1MC0 SDK defines UART0/1/2 at 0x4001B000/0x4001C000/0x4001D000
 * with a fixed 0x1000 stride, so:
 *
 *   instance = (reg_addr - UART0_BASE) / 0x1000
 *
 * Build-time validation ensures the devicetree address matches a known UART.
 */
#define YTM32_UART_INSTANCE_STRIDE 0x1000U

#define YTM32_UART_INSTANCE_FROM_ADDR(addr) \
	((((uint32_t)(addr)) - ((uint32_t)UART0_BASE)) / YTM32_UART_INSTANCE_STRIDE)

#define YTM32_UART_INSTANCE_VALID(addr) \
	BUILD_ASSERT((((uint32_t)(addr)) >= ((uint32_t)UART0_BASE)) && \
		     (((uint32_t)(addr)) <= ((uint32_t)UART2_BASE)) && \
		     (((((uint32_t)(addr)) - ((uint32_t)UART0_BASE)) % \
		       YTM32_UART_INSTANCE_STRIDE) == 0U), \
		     "UART reg address does not match any known UART instance")

/* 规避 Zephyr API 和厂商 HAL API 对 uart_callback_t 命名的冲突 */
#define uart_callback_t hal_uart_callback_t
#include "uart_driver.h" /* YTMicro HAL UART header */
#include "uart_hw_access.h" /* YTMicro HAL UART hw access header for inline functions */
#undef uart_callback_t

struct uart_ytm32_config {
	uint32_t base;
	uint8_t instance;
	uint32_t baud_rate;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pincfg;
};

struct uart_ytm32_data {
	uart_state_t hal_state;
};

static int uart_ytm32_poll_in(const struct device *dev, unsigned char *c)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(c);

	/* MVP 阶段先不管输入 */
	return -1;
}

static void uart_ytm32_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_ytm32_config *config = dev->config;
	UART_Type *base = (UART_Type *)config->base;

	/* 等待发送数据寄存器空中断标志位 */
	while (!UART_GetStatusFlag(base, UART_TX_DATA_REG_EMPTY)) {
	}

	UART_Putchar(base, c);
}

static int uart_ytm32_err_check(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static const struct uart_driver_api uart_ytm32_driver_api = {
	.poll_in = uart_ytm32_poll_in,
	.poll_out = uart_ytm32_poll_out,
	.err_check = uart_ytm32_err_check,
};

static int uart_ytm32_init(const struct device *dev)
{
	const struct uart_ytm32_config *config = dev->config;
	struct uart_ytm32_data *data = dev->data;
	uint32_t instance = config->instance;
	uint32_t uart_clk_rate;

	uart_user_config_t hal_config;

	/* 1. 开启 UART 设备时钟 */
	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	int ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_get_rate(config->clock_dev, config->clock_subsys,
				     &uart_clk_rate);
	if (ret < 0) {
		return ret;
	}

	if ((uart_clk_rate == 0U) ||
	    (uart_clk_rate > YTM32_UART_MAX_FUNCTIONAL_CLOCK_HZ)) {
		return -EINVAL;
	}

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* 2. 调用厂商 HAL 进行 UART 初始化 */
	UART_DRV_GetDefaultConfig(&hal_config);
	hal_config.baudRate = config->baud_rate;

	status_t status = UART_DRV_Init(instance, &data->hal_state, &hal_config);
	if (status != STATUS_SUCCESS) {
		return -EIO;
	}

	UART_Type *base_addr = (UART_Type *)config->base;
	/* 持久开启发送和接收功能，因为 SendDataPolling 会不断开关导致截断 */
	UART_SetTransmitterCmd(base_addr, true);
	UART_SetReceiverCmd(base_addr, true);

	return 0;
}

#define YTM32_UART_INIT(n) \
	YTM32_UART_INSTANCE_VALID(DT_INST_REG_ADDR(n)); \
	PINCTRL_DT_INST_DEFINE(n); \
	static struct uart_ytm32_data uart_ytm32_data_##n; \
	static const struct uart_ytm32_config uart_ytm32_config_##n = { \
		.base = DT_INST_REG_ADDR(n), \
		.instance = YTM32_UART_INSTANCE_FROM_ADDR(DT_INST_REG_ADDR(n)), \
		.baud_rate = DT_INST_PROP(n, current_speed), \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)), \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id), \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n), \
	}; \
	DEVICE_DT_INST_DEFINE(n, &uart_ytm32_init, NULL, &uart_ytm32_data_##n, \
			      &uart_ytm32_config_##n, PRE_KERNEL_1, \
			      CONFIG_SERIAL_INIT_PRIORITY, \
			      &uart_ytm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(YTM32_UART_INIT)
