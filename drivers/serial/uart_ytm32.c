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
#include <zephyr/irq.h>
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
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif
};

struct uart_ytm32_data {
	uart_state_t hal_state;
	int errors;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
};

static int uart_ytm32_latch_errors(const struct device *dev)
{
	const struct uart_ytm32_config *config = dev->config;
	struct uart_ytm32_data *data = dev->data;
	UART_Type *base = (UART_Type *)config->base;
	int errors = 0;

	if (UART_GetStatusFlag(base, UART_RX_OVERRUN)) {
		errors |= UART_ERROR_OVERRUN;
		(void)UART_ClearStatusFlag(base, UART_RX_OVERRUN);
	}

	if (UART_GetStatusFlag(base, UART_FRAME_ERR)) {
		errors |= UART_ERROR_FRAMING;
		(void)UART_ClearStatusFlag(base, UART_FRAME_ERR);
	}

	if (UART_GetStatusFlag(base, UART_PARITY_ERR)) {
		errors |= UART_ERROR_PARITY;
		(void)UART_ClearStatusFlag(base, UART_PARITY_ERR);
	}

	if (UART_GetStatusFlag(base, UART_NOISE_DETECT)) {
		errors |= UART_ERROR_NOISE;
		(void)UART_ClearStatusFlag(base, UART_NOISE_DETECT);
	}

	data->errors |= errors;

	return errors;
}

static void uart_ytm32_recover_rx(UART_Type *base, int errors)
{
	if (errors == 0) {
		return;
	}

#if defined(FEATURE_UART_FIFO_SIZE) && (FEATURE_UART_FIFO_SIZE > 0U)
	if ((errors & UART_ERROR_OVERRUN) != 0) {
		UART_ResetRxFifo(base);
		UART_SetRxFifoWatermark(base, 0U);
		UART_EnableRxFifo(base, true);
	}
#endif

	UART_SetReceiverCmd(base, true);
}

static int uart_ytm32_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_ytm32_config *config = dev->config;
	UART_Type *base = (UART_Type *)config->base;
	int errors;

	errors = uart_ytm32_latch_errors(dev);

	if (!UART_GetStatusFlag(base, UART_RX_DATA_REG_FULL)) {
		uart_ytm32_recover_rx(base, errors);
		return -1;
	}

	UART_Getchar8(base, c);
	errors |= uart_ytm32_latch_errors(dev);
	uart_ytm32_recover_rx(base, errors);

	return 0;
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
	const struct uart_ytm32_config *config = dev->config;
	struct uart_ytm32_data *data = dev->data;
	UART_Type *base = (UART_Type *)config->base;
	int errors;

	errors = data->errors | uart_ytm32_latch_errors(dev);
	data->errors = 0;
	uart_ytm32_recover_rx(base, errors);

	return errors;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_ytm32_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	const struct uart_ytm32_config *config = dev->config;
	UART_Type *base = (UART_Type *)config->base;
	int num_tx = 0;

	while ((len - num_tx > 0) && UART_GetStatusFlag(base, UART_TX_DATA_REG_EMPTY)) {
		UART_Putchar(base, tx_data[num_tx++]);
	}

	return num_tx;
}

static int uart_ytm32_fifo_read(const struct device *dev, uint8_t *rx_data, const int len)
{
	const struct uart_ytm32_config *config = dev->config;
	UART_Type *base = (UART_Type *)config->base;
	int num_rx = 0;

	while ((len - num_rx > 0) && UART_GetStatusFlag(base, UART_RX_DATA_REG_FULL)) {
		UART_Getchar8(base, &rx_data[num_rx++]);
	}

	return num_rx;
}

static void uart_ytm32_irq_tx_enable(const struct device *dev)
{
	const struct uart_ytm32_config *config = dev->config;
	UART_SetIntMode((UART_Type *)config->base, UART_INT_TX_DATA_REG_EMPTY, true);
}

static void uart_ytm32_irq_tx_disable(const struct device *dev)
{
	const struct uart_ytm32_config *config = dev->config;
	UART_SetIntMode((UART_Type *)config->base, UART_INT_TX_DATA_REG_EMPTY, false);
}

static int uart_ytm32_irq_tx_ready(const struct device *dev)
{
	const struct uart_ytm32_config *config = dev->config;
	UART_Type *base = (UART_Type *)config->base;

	return (UART_GetIntMode(base, UART_INT_TX_DATA_REG_EMPTY) &&
		UART_GetStatusFlag(base, UART_TX_DATA_REG_EMPTY));
}

static void uart_ytm32_irq_rx_enable(const struct device *dev)
{
	const struct uart_ytm32_config *config = dev->config;
	UART_SetIntMode((UART_Type *)config->base, UART_INT_RX_DATA_REG_FULL, true);
}

static void uart_ytm32_irq_rx_disable(const struct device *dev)
{
	const struct uart_ytm32_config *config = dev->config;
	UART_SetIntMode((UART_Type *)config->base, UART_INT_RX_DATA_REG_FULL, false);
}

static int uart_ytm32_irq_tx_complete(const struct device *dev)
{
	const struct uart_ytm32_config *config = dev->config;
	UART_Type *base = (UART_Type *)config->base;

	return UART_GetStatusFlag(base, UART_TX_COMPLETE);
}

static int uart_ytm32_irq_rx_ready(const struct device *dev)
{
	const struct uart_ytm32_config *config = dev->config;
	UART_Type *base = (UART_Type *)config->base;

	return (UART_GetIntMode(base, UART_INT_RX_DATA_REG_FULL) &&
		UART_GetStatusFlag(base, UART_RX_DATA_REG_FULL));
}

static void uart_ytm32_irq_err_enable(const struct device *dev)
{
	const struct uart_ytm32_config *config = dev->config;
	UART_Type *base = (UART_Type *)config->base;

	UART_SetIntMode(base, UART_INT_RX_OVERRUN, true);
	UART_SetIntMode(base, UART_INT_FRAME_ERR_FLAG, true);
	UART_SetIntMode(base, UART_INT_PARITY_ERR_FLAG, true);
	UART_SetIntMode(base, UART_INT_NOISE_ERR_FLAG, true);
}

static void uart_ytm32_irq_err_disable(const struct device *dev)
{
	const struct uart_ytm32_config *config = dev->config;
	UART_Type *base = (UART_Type *)config->base;

	UART_SetIntMode(base, UART_INT_RX_OVERRUN, false);
	UART_SetIntMode(base, UART_INT_FRAME_ERR_FLAG, false);
	UART_SetIntMode(base, UART_INT_PARITY_ERR_FLAG, false);
	UART_SetIntMode(base, UART_INT_NOISE_ERR_FLAG, false);
}

static int uart_ytm32_irq_is_pending(const struct device *dev)
{
	const struct uart_ytm32_config *config = dev->config;
	UART_Type *base = (UART_Type *)config->base;

	return (uart_ytm32_irq_tx_ready(dev) || uart_ytm32_irq_rx_ready(dev));
}

static int uart_ytm32_irq_update(const struct device *dev)
{
	return 1;
}

static int uart_ytm32_irq_callback_set(const struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct uart_ytm32_data *data = dev->data;

	data->callback = cb;
	data->cb_data = cb_data;

	return 0;
}

static void uart_ytm32_isr(const struct device *dev)
{
	struct uart_ytm32_data *data = dev->data;

	if (data->callback) {
		data->callback(dev, data->cb_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_ytm32_driver_api = {
	.poll_in = uart_ytm32_poll_in,
	.poll_out = uart_ytm32_poll_out,
	.err_check = uart_ytm32_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_ytm32_fifo_fill,
	.fifo_read = uart_ytm32_fifo_read,
	.irq_tx_enable = uart_ytm32_irq_tx_enable,
	.irq_tx_disable = uart_ytm32_irq_tx_disable,
	.irq_tx_ready = uart_ytm32_irq_tx_ready,
	.irq_rx_enable = uart_ytm32_irq_rx_enable,
	.irq_rx_disable = uart_ytm32_irq_rx_disable,
	.irq_tx_complete = uart_ytm32_irq_tx_complete,
	.irq_rx_ready = uart_ytm32_irq_rx_ready,
	.irq_err_enable = uart_ytm32_irq_err_enable,
	.irq_err_disable = uart_ytm32_irq_err_disable,
	.irq_is_pending = uart_ytm32_irq_is_pending,
	.irq_update = uart_ytm32_irq_update,
	.irq_callback_set = uart_ytm32_irq_callback_set,
#endif
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
	base_addr->INTE = 0U;
	UART_DRV_ClearErrorFlags(base_addr);

	#if defined(FEATURE_UART_FIFO_SIZE) && (FEATURE_UART_FIFO_SIZE > 0U)
	UART_ResetRxFifo(base_addr);
	UART_SetRxFifoWatermark(base_addr, 0U);
	UART_EnableRxFifo(base_addr, true);
	#endif

	/* 持久开启发送和接收功能，因为 SendDataPolling 会不断开关导致截断 */
	UART_SetTransmitterCmd(base_addr, true);
	UART_SetReceiverCmd(base_addr, true);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif

	return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define YTM32_UART_IRQ_CONFIG_FUNC(n) \
	static void uart_ytm32_irq_config_##n(const struct device *dev) \
	{ \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), \
			    uart_ytm32_isr, DEVICE_DT_INST_GET(n), 0); \
		irq_enable(DT_INST_IRQN(n)); \
	}
#define YTM32_UART_IRQ_CONFIG_INIT(n) \
	.irq_config_func = uart_ytm32_irq_config_##n,
#else
#define YTM32_UART_IRQ_CONFIG_FUNC(n)
#define YTM32_UART_IRQ_CONFIG_INIT(n)
#endif

#define YTM32_UART_INIT(n) \
	YTM32_UART_INSTANCE_VALID(DT_INST_REG_ADDR(n)); \
	PINCTRL_DT_INST_DEFINE(n); \
	YTM32_UART_IRQ_CONFIG_FUNC(n) \
	static struct uart_ytm32_data uart_ytm32_data_##n; \
	static const struct uart_ytm32_config uart_ytm32_config_##n = { \
		.base = DT_INST_REG_ADDR(n), \
		.instance = YTM32_UART_INSTANCE_FROM_ADDR(DT_INST_REG_ADDR(n)), \
		.baud_rate = DT_INST_PROP(n, current_speed), \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)), \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id), \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n), \
		YTM32_UART_IRQ_CONFIG_INIT(n) \
	}; \
	DEVICE_DT_INST_DEFINE(n, &uart_ytm32_init, NULL, &uart_ytm32_data_##n, \
			      &uart_ytm32_config_##n, PRE_KERNEL_1, \
			      CONFIG_SERIAL_INIT_PRIORITY, \
			      &uart_ytm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(YTM32_UART_INIT)
