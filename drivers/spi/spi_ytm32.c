/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ytmicro_ytm32_spi

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define spi_callback_t hal_spi_callback_t
#include "spi_master_driver.h"
#undef spi_callback_t

LOG_MODULE_REGISTER(spi_ytm32, CONFIG_SPI_LOG_LEVEL);

#define YTM32_SPI_INSTANCE_STRIDE 0x1000U
#define YTM32_SPI_TRANSFER_CHUNK 32U

#define YTM32_SPI_INSTANCE_FROM_ADDR(addr) \
	((((uint32_t)(addr)) - ((uint32_t)SPI0_BASE)) / YTM32_SPI_INSTANCE_STRIDE)

#define YTM32_SPI_INSTANCE_VALID(addr) \
	BUILD_ASSERT((((uint32_t)(addr)) >= ((uint32_t)SPI0_BASE)) && \
		     (((uint32_t)(addr)) <= ((uint32_t)SPI3_BASE)) && \
		     (((((uint32_t)(addr)) - ((uint32_t)SPI0_BASE)) % \
		       YTM32_SPI_INSTANCE_STRIDE) == 0U), \
		     "SPI reg address does not match any known SPI instance")

struct spi_ytm32_config {
	uint32_t base;
	uint8_t instance;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pincfg;
	void (*irq_config_func)(void);
};

struct spi_ytm32_data {
	struct k_mutex lock;
	struct k_sem done;
	spi_state_t hal_state;
	volatile status_t transfer_status;
};

struct spi_ytm32_buf_cursor {
	const struct spi_buf_set *set;
	size_t index;
	size_t offset;
};

static int spi_ytm32_status_to_errno(status_t status)
{
	switch (status) {
	case STATUS_SUCCESS:
		return 0;
	case STATUS_BUSY:
		return -EBUSY;
	case STATUS_TIMEOUT:
		return -ETIMEDOUT;
	default:
		return -EIO;
	}
}

static size_t spi_ytm32_buf_set_len(const struct spi_buf_set *set)
{
	size_t len = 0;

	if (set == NULL) {
		return 0;
	}

	for (size_t i = 0; i < set->count; i++) {
		len += set->buffers[i].len;
	}

	return len;
}

static uint8_t spi_ytm32_buf_cursor_get(struct spi_ytm32_buf_cursor *cursor)
{
	const struct spi_buf *buf;
	const uint8_t *data;

	while ((cursor->set != NULL) && (cursor->index < cursor->set->count)) {
		buf = &cursor->set->buffers[cursor->index];
		if (cursor->offset < buf->len) {
			data = buf->buf;
			return data != NULL ? data[cursor->offset++] : 0xff;
		}
		cursor->index++;
		cursor->offset = 0;
	}

	return 0xff;
}

static void spi_ytm32_buf_cursor_put(struct spi_ytm32_buf_cursor *cursor, uint8_t value)
{
	const struct spi_buf *buf;
	uint8_t *data;

	while ((cursor->set != NULL) && (cursor->index < cursor->set->count)) {
		buf = &cursor->set->buffers[cursor->index];
		if (cursor->offset < buf->len) {
			data = buf->buf;
			if (data != NULL) {
				data[cursor->offset] = value;
			}
			cursor->offset++;
			return;
		}
		cursor->index++;
		cursor->offset = 0;
	}
}

static void spi_ytm32_hal_callback(void *driver_state, spi_event_t event, void *user_data)
{
	struct spi_ytm32_data *data = user_data;

	ARG_UNUSED(driver_state);

	if (event == SPI_EVENT_END_TRANSFER) {
		data->transfer_status = STATUS_SUCCESS;
		k_sem_give(&data->done);
	}
}

static int spi_ytm32_validate_config(const struct spi_config *config)
{
	spi_operation_t operation = config->operation;
	uint16_t word_size = SPI_WORD_SIZE_GET(operation);

	if (SPI_OP_MODE_GET(operation) != SPI_OP_MODE_MASTER) {
		return -ENOTSUP;
	}

	if (word_size != 8U) {
		return -ENOTSUP;
	}

	if ((operation & SPI_TRANSFER_LSB) != 0U) {
		return -ENOTSUP;
	}

	if ((operation & SPI_HALF_DUPLEX) != 0U) {
		return -ENOTSUP;
	}

#if defined(CONFIG_SPI_EXTENDED_MODES)
	if ((operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		return -ENOTSUP;
	}
#endif

	if (config->frequency == 0U) {
		return -EINVAL;
	}

	return 0;
}

static void spi_ytm32_fill_hal_config(const struct spi_config *config,
					     spi_master_config_t *hal_config,
					     struct spi_ytm32_data *data)
{
	SPI_DRV_MasterGetDefaultConfig(hal_config);

	hal_config->bitsPerSec = config->frequency;
	hal_config->whichPcs = (spi_which_pcs_t)MIN(config->slave, 3U);
	hal_config->pcsPolarity = (config->operation & SPI_CS_ACTIVE_HIGH) ?
		SPI_ACTIVE_HIGH : SPI_ACTIVE_LOW;
	hal_config->isPcsContinuous = false;
	hal_config->bitcount = 8U;
	hal_config->clkPhase = (config->operation & SPI_MODE_CPHA) ?
		SPI_CLOCK_PHASE_2ND_EDGE : SPI_CLOCK_PHASE_1ST_EDGE;
	hal_config->clkPolarity = (config->operation & SPI_MODE_CPOL) ?
		SPI_SCK_ACTIVE_LOW : SPI_SCK_ACTIVE_HIGH;
	hal_config->lsbFirst = false;
	hal_config->transferType = SPI_USING_INTERRUPTS;
	hal_config->callback = spi_ytm32_hal_callback;
	hal_config->callbackParam = data;
	hal_config->width = SPI_SINGLE_BIT_XFER;
}

static int spi_ytm32_configure_bus(const struct device *dev,
					  const struct spi_config *config)
{
	const struct spi_ytm32_config *cfg = dev->config;
	struct spi_ytm32_data *data = dev->data;
	spi_master_config_t hal_config;
	uint32_t calculated_baudrate;
	status_t status;
	int ret;

	ret = spi_ytm32_validate_config(config);
	if (ret < 0) {
		return ret;
	}

	spi_ytm32_fill_hal_config(config, &hal_config, data);

	status = SPI_DRV_MasterConfigureBus(cfg->instance, &hal_config, &calculated_baudrate);
	if (status != STATUS_SUCCESS) {
		return spi_ytm32_status_to_errno(status);
	}

	return 0;
}

static int spi_ytm32_cs_configure(const struct spi_config *config)
{
	if (!spi_cs_is_gpio(config)) {
		return 0;
	}

	if (!gpio_is_ready_dt(&config->cs.gpio)) {
		return -ENODEV;
	}

	return gpio_pin_configure_dt(&config->cs.gpio, GPIO_OUTPUT_INACTIVE);
}

static void spi_ytm32_cs_control(const struct spi_config *config, bool active)
{
	if (!spi_cs_is_gpio(config)) {
		return;
	}

	if (config->cs.delay != 0U) {
		k_busy_wait(config->cs.delay);
	}

	(void)gpio_pin_set_dt(&config->cs.gpio, active ? 1 : 0);

	if (config->cs.delay != 0U) {
		k_busy_wait(config->cs.delay);
	}
}

/* 
 * 此函数负责将 Zephyr 的分段缓冲区 (spi_buf_set) 拆分为固定大小的块，
 * 然后调用底层 YTM32 Vendor HAL 接口来真正发送和接收数据。
 */
static int spi_ytm32_transfer(const struct device *dev,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs)
{
	const struct spi_ytm32_config *cfg = dev->config;
	struct spi_ytm32_data *data = dev->data;
	struct spi_ytm32_buf_cursor tx = { .set = tx_bufs };
	struct spi_ytm32_buf_cursor rx = { .set = rx_bufs };
	size_t tx_len = spi_ytm32_buf_set_len(tx_bufs);
	size_t rx_len = spi_ytm32_buf_set_len(rx_bufs);
	size_t remaining = MAX(tx_len, rx_len);
	uint8_t tx_chunk[YTM32_SPI_TRANSFER_CHUNK];
	uint8_t rx_chunk[YTM32_SPI_TRANSFER_CHUNK];
	status_t status;
	int ret;

	while (remaining > 0U) {
		size_t chunk_len = MIN(remaining, YTM32_SPI_TRANSFER_CHUNK);

		/* 将数据从 Zephyr 的 buffer 提取到连续的 chunk 数组中 */
		for (size_t i = 0; i < chunk_len; i++) {
			tx_chunk[i] = spi_ytm32_buf_cursor_get(&tx);
		}

		k_sem_reset(&data->done);
		data->transfer_status = STATUS_BUSY;

		status = SPI_DRV_MasterTransfer(cfg->instance, tx_chunk, rx_chunk, chunk_len);
		if (status != STATUS_SUCCESS) {
			return spi_ytm32_status_to_errno(status);
		}

		ret = k_sem_take(&data->done, K_MSEC(CONFIG_SPI_YTM32_TRANSFER_TIMEOUT_MS));
		if (ret < 0) {
			(void)SPI_DRV_MasterAbortTransfer(cfg->instance);
			return -ETIMEDOUT;
		}

		status = SPI_DRV_MasterGetTransferStatus(cfg->instance, NULL);
		if (status != STATUS_SUCCESS) {
			return spi_ytm32_status_to_errno(status);
		}

		for (size_t i = 0; i < chunk_len; i++) {
			spi_ytm32_buf_cursor_put(&rx, rx_chunk[i]);
		}

		remaining -= chunk_len;
	}

	return 0;
}

/* 
 * 硬件驱动适配层 (Zephyr Driver Wrapper Layer) 的核心入口点。
 * Zephyr SPI 子系统最终会路由并调用到这个 transceive 函数。
 */
static int spi_ytm32_transceive(const struct device *dev,
				       const struct spi_config *config,
				       const struct spi_buf_set *tx_bufs,
				       const struct spi_buf_set *rx_bufs)
{
	struct spi_ytm32_data *data = dev->data;
	int ret;

	/* 1. 获取互斥锁，确保多线程下 SPI 总线不会产生并发冲突 */
	k_mutex_lock(&data->lock, K_FOREVER);

	/* 2. 将 Zephyr 的 SPI 参数 (spi_config) 转换为底层 YTM32 硬件所需的配置并应用 */
	ret = spi_ytm32_configure_bus(dev, config);
	if (ret < 0) {
		goto out;
	}

	/* 3. 配置片选引脚 (CS) */
	ret = spi_ytm32_cs_configure(config);
	if (ret < 0) {
		goto out;
	}

	/* 4. 真正的数据传输流程: 拉低片选 -> 传输数据 -> 拉高片选释放设备 */
	spi_ytm32_cs_control(config, true);
	ret = spi_ytm32_transfer(dev, tx_bufs, rx_bufs);
	spi_ytm32_cs_control(config, false);

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int spi_ytm32_release(const struct device *dev, const struct spi_config *config)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(config);

	return 0;
}

static int spi_ytm32_init(const struct device *dev)
{
	const struct spi_ytm32_config *cfg = dev->config;
	struct spi_ytm32_data *data = dev->data;
	spi_master_config_t hal_config;
	uint32_t clock_rate;
	status_t status;
	int ret;

	k_mutex_init(&data->lock);
	k_sem_init(&data->done, 0, 1);
	data->transfer_status = STATUS_SUCCESS;

	if (!device_is_ready(cfg->clock_dev)) {
		printk("%s clock device not ready\n", dev->name);
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret < 0) {
		printk("%s clock_control_on failed: %d\n", dev->name, ret);
		return ret;
	}

	ret = clock_control_get_rate(cfg->clock_dev, cfg->clock_subsys, &clock_rate);
	if (ret < 0) {
		printk("%s clock_control_get_rate failed: %d\n", dev->name, ret);
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		printk("%s pinctrl_apply_state failed: %d\n", dev->name, ret);
		return ret;
	}

	SPI_DRV_MasterGetDefaultConfig(&hal_config);
	hal_config.bitsPerSec = MIN(clock_rate / 2U, 1000000U);
	hal_config.bitcount = 8U;
	hal_config.transferType = SPI_USING_INTERRUPTS;
	hal_config.callback = spi_ytm32_hal_callback;
	hal_config.callbackParam = data;
	hal_config.width = SPI_SINGLE_BIT_XFER;

	status = SPI_DRV_MasterInit(cfg->instance, &data->hal_state, &hal_config);
	if (status != STATUS_SUCCESS) {
		printk("%s SPI_DRV_MasterInit failed: %d instance=%u clock=%u\n",
		       dev->name, status, cfg->instance, clock_rate);
		return spi_ytm32_status_to_errno(status);
	}

	cfg->irq_config_func();

	LOG_INF("%s functional clock: %u Hz", dev->name, clock_rate);

	return 0;
}

/* 将我们实现的适配函数注册到 Zephyr SPI 子系统的 API 接口中 */
static DEVICE_API(spi, spi_ytm32_api) = {
	.transceive = spi_ytm32_transceive,
	.release = spi_ytm32_release,
};

#define SPI_YTM32_IRQ_HANDLER(n) \
	static void spi_ytm32_irq_handler_##n(const void *arg) \
	{ \
		const struct device *dev = arg; \
		const struct spi_ytm32_config *cfg = dev->config; \
		SPI_DRV_MasterIRQHandler(cfg->instance); \
	} \
	static void spi_ytm32_irq_config_##n(void) \
	{ \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), \
			    spi_ytm32_irq_handler_##n, DEVICE_DT_INST_GET(n), 0); \
		irq_enable(DT_INST_IRQN(n)); \
	}

#define SPI_YTM32_INIT(n) \
	YTM32_SPI_INSTANCE_VALID(DT_INST_REG_ADDR(n)); \
	PINCTRL_DT_INST_DEFINE(n); \
	SPI_YTM32_IRQ_HANDLER(n) \
	static struct spi_ytm32_data spi_ytm32_data_##n; \
	static const struct spi_ytm32_config spi_ytm32_config_##n = { \
		.base = DT_INST_REG_ADDR(n), \
		.instance = YTM32_SPI_INSTANCE_FROM_ADDR(DT_INST_REG_ADDR(n)), \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)), \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id), \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n), \
		.irq_config_func = spi_ytm32_irq_config_##n, \
	}; \
	DEVICE_DT_INST_DEFINE(n, spi_ytm32_init, NULL, &spi_ytm32_data_##n, \
			      &spi_ytm32_config_##n, POST_KERNEL, \
			      CONFIG_SPI_INIT_PRIORITY, &spi_ytm32_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_YTM32_INIT)
