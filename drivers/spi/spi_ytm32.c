/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ytmicro_ytm32_spi

#include <errno.h>

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_ytm32);

#include "spi_context.h"

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

/* Vendor types are fully hidden behind this wrapper header. */
#include "ytm32_spi_hal.h"

#define YTM32_SPI_INSTANCE_STRIDE 0x1000U

/*
 * NULL spi_buf segments (dummy TX / discard RX) are sub-segmented at this
 * size because the module-level dummy arrays are bounded to this length.
 */
#define YTM32_SPI_DUMMY_LEN 64U

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
#ifdef CONFIG_SPI_YTM32_DMA
	const struct device *dma_dev;
	uint8_t dma_tx_chan;
	uint8_t dma_rx_chan;
#endif
};

struct spi_ytm32_data {
	struct spi_context ctx;      /* bus lock, config cache, CS control */
	volatile int hal_result;     /* transfer outcome set by HAL callback */
	struct k_sem done;           /* signals transfer completion from ISR */
#ifdef CONFIG_SPI_YTM32_DMA
	bool    dma_active;
	uint8_t dma_tx_chan;
	uint8_t dma_rx_chan;
#endif
};

/*
 * 0xFF dummy TX: devices that inspect MOSI during dummy clocks see the
 * de-selected idle pattern rather than 0x00.
 * dummy_rx is write-only discard; shared across SPI instances is safe
 * because its contents are never read back.
 */
static const uint8_t dummy_tx[YTM32_SPI_DUMMY_LEN] = {
	[0 ... (YTM32_SPI_DUMMY_LEN - 1)] = 0xFF
};
static uint8_t dummy_rx[YTM32_SPI_DUMMY_LEN];

/* ─────────────────────────── HAL callback ─────────────────────────── */

static void spi_ytm32_hal_cb(void *user_data, int status)
{
	struct spi_ytm32_data *data = user_data;

	data->hal_result = status;
	k_sem_give(&data->done);
}

/* ─────────────────────────── config validation ─────────────────────────── */

static int spi_ytm32_validate_config(const struct spi_config *config)
{
	spi_operation_t op = config->operation;

	if (SPI_OP_MODE_GET(op) != SPI_OP_MODE_MASTER) {
		return -ENOTSUP;
	}
	if (SPI_WORD_SIZE_GET(op) != 8U) {
		return -ENOTSUP;
	}
	if (op & SPI_TRANSFER_LSB) {
		return -ENOTSUP;
	}
	if (op & SPI_HALF_DUPLEX) {
		return -ENOTSUP;
	}
#if defined(CONFIG_SPI_EXTENDED_MODES)
	if ((op & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		return -ENOTSUP;
	}
#endif
	if (config->frequency == 0U) {
		return -EINVAL;
	}
	return 0;
}

/* ─────────────────────────── bus configure ─────────────────────────── */

static int spi_ytm32_configure_bus(const struct device *dev,
				   const struct spi_config *config)
{
	const struct spi_ytm32_config *cfg = dev->config;
	spi_operation_t op = config->operation;
	int ret;

	ret = spi_ytm32_validate_config(config);
	if (ret < 0) {
		return ret;
	}

	return ytm32_spi_hal_configure(
		cfg->instance,
		config->frequency,
		(op & SPI_MODE_CPOL) ? 1U : 0U,
		(op & SPI_MODE_CPHA) ? 1U : 0U,
		(uint8_t)MIN(config->slave, 3U),
		(op & SPI_CS_ACTIVE_HIGH) ? true : false,
		spi_cs_is_gpio(config));
}

/* Configure the GPIO CS pin direction on first use of a given spi_config. */
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

/* ─────────────────────────── transfer ─────────────────────────── */

/* Issue one non-blocking HAL transfer and block until completion. */
static int spi_ytm32_one_shot(const struct device *dev,
			      const uint8_t *tx, uint8_t *rx, uint16_t len)
{
	const struct spi_ytm32_config *cfg = dev->config;
	struct spi_ytm32_data *data = dev->data;
	int ret;

	k_sem_reset(&data->done);

	ret = ytm32_spi_hal_transfer(cfg->instance, tx, rx, len);
	if (ret < 0) {
		return ret;
	}

	ret = k_sem_take(&data->done, K_MSEC(CONFIG_SPI_YTM32_TRANSFER_TIMEOUT_MS));
	if (ret < 0) {
		ytm32_spi_hal_abort(cfg->instance);
		return -ETIMEDOUT;
	}

	return data->hal_result;   /* 0 or -EIO from spi_ytm32_hal_cb */
}

/*
 * Walk both buf_sets in lock-step, issuing one HAL call per natural segment
 * boundary. Non-NULL buffers are passed directly — no copy.
 * NULL buffers (dummy TX / discard RX) use module-level dummy arrays,
 * sub-segmented at YTM32_SPI_DUMMY_LEN when longer.
 */
static int spi_ytm32_transfer(const struct device *dev,
			      const struct spi_buf_set *tx_bufs,
			      const struct spi_buf_set *rx_bufs)
{
	const size_t tx_count = tx_bufs ? tx_bufs->count : 0;
	const size_t rx_count = rx_bufs ? rx_bufs->count : 0;
	size_t ti = 0, ri = 0;
	size_t tx_off = 0, rx_off = 0;

	while (ti < tx_count || ri < rx_count) {
		size_t tx_left = (ti < tx_count) ?
				 tx_bufs->buffers[ti].len - tx_off : 0;
		size_t rx_left = (ri < rx_count) ?
				 rx_bufs->buffers[ri].len - rx_off : 0;

		const uint8_t *tx_ptr;
		size_t tx_seg;

		if (ti < tx_count) {
			const void *buf = tx_bufs->buffers[ti].buf;

			tx_ptr = buf ? (const uint8_t *)buf + tx_off : dummy_tx;
			tx_seg = buf ? tx_left : MIN(tx_left, YTM32_SPI_DUMMY_LEN);
		} else {
			tx_ptr = dummy_tx;
			tx_seg = YTM32_SPI_DUMMY_LEN;
		}

		uint8_t *rx_ptr;
		size_t rx_seg;

		if (ri < rx_count) {
			void *buf = rx_bufs->buffers[ri].buf;

			rx_ptr = buf ? (uint8_t *)buf + rx_off : dummy_rx;
			rx_seg = buf ? rx_left : MIN(rx_left, YTM32_SPI_DUMMY_LEN);
		} else {
			rx_ptr = dummy_rx;
			rx_seg = YTM32_SPI_DUMMY_LEN;
		}

		size_t seg_len = MIN(MIN(tx_seg, rx_seg), (size_t)UINT16_MAX);

		if (seg_len == 0) {
			break;
		}

		int ret = spi_ytm32_one_shot(dev, tx_ptr, rx_ptr,
					     (uint16_t)seg_len);
		if (ret < 0) {
			return ret;
		}

		if (ti < tx_count) {
			tx_off += seg_len;
			if (tx_off >= tx_bufs->buffers[ti].len) {
				ti++;
				tx_off = 0;
			}
		}
		if (ri < rx_count) {
			rx_off += seg_len;
			if (rx_off >= rx_bufs->buffers[ri].len) {
				ri++;
				rx_off = 0;
			}
		}
	}

	return 0;
}

/* ─────────────────────────── Zephyr SPI API ─────────────────────────── */

static int spi_ytm32_transceive(const struct device *dev,
				const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	struct spi_ytm32_data *data = dev->data;
	int ret;

	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	if (!spi_context_configured(&data->ctx, config)) {
		ret = spi_ytm32_configure_bus(dev, config);
		if (ret < 0) {
			goto out;
		}
		ret = spi_ytm32_cs_configure(config);
		if (ret < 0) {
			goto out;
		}
		data->ctx.config = config;
	}

	spi_context_cs_control(&data->ctx, true);
	ret = spi_ytm32_transfer(dev, tx_bufs, rx_bufs);
	spi_context_cs_control(&data->ctx, false);

out:
	if (ret < 0) {
		data->ctx.config = NULL;
	}
	spi_context_release(&data->ctx, ret);
	return ret;
}

static int spi_ytm32_release(const struct device *dev,
			     const struct spi_config *config)
{
	struct spi_ytm32_data *data = dev->data;

	ARG_UNUSED(config);
	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

/* ─────────────────────────── init ─────────────────────────── */

static int spi_ytm32_init(const struct device *dev)
{
	const struct spi_ytm32_config *cfg = dev->config;
	struct spi_ytm32_data *data = dev->data;
	uint32_t clock_rate;
	bool use_dma = false;
	uint8_t dma_rx = 0, dma_tx = 0;
	int ret;

	k_sem_init(&data->done, 0, 1);

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("clock device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret < 0) {
		LOG_ERR("clock_control_on failed: %d", ret);
		return ret;
	}

	ret = clock_control_get_rate(cfg->clock_dev, cfg->clock_subsys, &clock_rate);
	if (ret < 0) {
		LOG_ERR("clock_control_get_rate failed: %d", ret);
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("pinctrl_apply_state failed: %d", ret);
		return ret;
	}

#ifdef CONFIG_SPI_YTM32_DMA
	if (cfg->dma_dev != NULL && device_is_ready(cfg->dma_dev)) {
		use_dma         = true;
		dma_tx          = cfg->dma_tx_chan;
		dma_rx          = cfg->dma_rx_chan;
		data->dma_active = true;
		data->dma_tx_chan = dma_tx;
		data->dma_rx_chan = dma_rx;
		LOG_INF("DMA mode: tx ch%u, rx ch%u", dma_tx, dma_rx);
	} else {
		LOG_INF("interrupt mode (DMA not configured or not ready)");
	}
#endif

	ret = ytm32_spi_hal_init(cfg->instance, clock_rate,
				 use_dma, dma_rx, dma_tx,
				 spi_ytm32_hal_cb, data);
	if (ret < 0) {
		LOG_ERR("ytm32_spi_hal_init failed: %d (instance=%u, clock=%u Hz)",
			ret, cfg->instance, clock_rate);
		return ret;
	}

	cfg->irq_config_func();

	spi_context_unlock_unconditionally(&data->ctx);

	LOG_INF("functional clock: %u Hz", clock_rate);
	return 0;
}

/* ─────────────────────────── driver API + instance macro ─────────────────────────── */

static DEVICE_API(spi, spi_ytm32_api) = {
	.transceive = spi_ytm32_transceive,
	.release    = spi_ytm32_release,
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
};

#define SPI_YTM32_IRQ_HANDLER(n) \
	static void spi_ytm32_irq_handler_##n(const void *arg) \
	{ \
		const struct device *dev = arg; \
		const struct spi_ytm32_config *cfg = dev->config; \
		ytm32_spi_hal_irq(cfg->instance); \
	} \
	static void spi_ytm32_irq_config_##n(void) \
	{ \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), \
			    spi_ytm32_irq_handler_##n, DEVICE_DT_INST_GET(n), 0); \
		irq_enable(DT_INST_IRQN(n)); \
	}

#ifdef CONFIG_SPI_YTM32_DMA
#define SPI_YTM32_DMA_FIELDS(n) \
	.dma_dev    = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas), \
		(DEVICE_DT_GET_OR_NULL(DT_INST_DMAS_CTLR_BY_NAME(n, tx))), \
		(NULL)), \
	.dma_tx_chan = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas), \
		(DT_INST_DMAS_CELL_BY_NAME(n, tx, channel)), (0U)), \
	.dma_rx_chan = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas), \
		(DT_INST_DMAS_CELL_BY_NAME(n, rx, channel)), (0U)),
#else
#define SPI_YTM32_DMA_FIELDS(n)
#endif

#define SPI_YTM32_INIT(n) \
	YTM32_SPI_INSTANCE_VALID(DT_INST_REG_ADDR(n)); \
	PINCTRL_DT_INST_DEFINE(n); \
	SPI_YTM32_IRQ_HANDLER(n) \
	static struct spi_ytm32_data spi_ytm32_data_##n = { \
		SPI_CONTEXT_INIT_LOCK(spi_ytm32_data_##n, ctx), \
		SPI_CONTEXT_INIT_SYNC(spi_ytm32_data_##n, ctx), \
	}; \
	static const struct spi_ytm32_config spi_ytm32_config_##n = { \
		.base            = DT_INST_REG_ADDR(n), \
		.instance        = YTM32_SPI_INSTANCE_FROM_ADDR(DT_INST_REG_ADDR(n)), \
		.clock_dev       = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)), \
		.clock_subsys    = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id), \
		.pincfg          = PINCTRL_DT_INST_DEV_CONFIG_GET(n), \
		.irq_config_func = spi_ytm32_irq_config_##n, \
		SPI_YTM32_DMA_FIELDS(n) \
	}; \
	DEVICE_DT_INST_DEFINE(n, spi_ytm32_init, NULL, &spi_ytm32_data_##n, \
			      &spi_ytm32_config_##n, POST_KERNEL, \
			      CONFIG_SPI_INIT_PRIORITY, &spi_ytm32_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_YTM32_INIT)
