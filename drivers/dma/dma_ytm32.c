/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ytmicro_ytm32_dma

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/ytm32_dma_zli_timing.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

/* Vendor types are fully hidden behind this wrapper header. */
#include "ytm32_dma_hal.h"

LOG_MODULE_REGISTER(dma_ytm32, CONFIG_DMA_LOG_LEVEL);

#define YTM32_DMA_NUM_CHANNELS 16U   /* FEATURE_DMA_VIRTUAL_CHANNELS */

struct ytm32_dma_chan {
	dma_callback_t  zephyr_cb;
	void           *zephyr_cb_data;
	bool            configured;
	enum dma_channel_direction dir;
};

struct ytm32_dma_data {
	/* dma_context MUST be the first member. */
	struct dma_context  ctx;
	struct ytm32_dma_chan chan[YTM32_DMA_NUM_CHANNELS];
	ATOMIC_DEFINE(chan_atomic, YTM32_DMA_NUM_CHANNELS);
};

struct ytm32_dma_config {
	uint32_t base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	void (*irq_config_func)(void);
};

/* ─────────────────────────── HAL → Zephyr callback bridge ─────────────────────────── */

/*
 * ytm32_dma_cb_t bridge: the HAL wrapper calls us with (user_data, status).
 * user_data carries a pointer to ytm32_dma_chan so we can reach the device
 * and the Zephyr callback without an extra lookup.
 */
struct chan_cb_ctx {
	const struct device *dev;
	uint8_t              idx;
};
/* Statically allocated, one per channel; address never changes after init. */
static struct chan_cb_ctx s_cbs[YTM32_DMA_NUM_CHANNELS];

static void dma_ytm32_hal_cb(void *user_data, int hal_status)
{
	const struct chan_cb_ctx *ctx = user_data;
	const struct device *dev = ctx->dev;
	struct ytm32_dma_data *data = dev->data;
	struct ytm32_dma_chan *chan = &data->chan[ctx->idx];

	if (ctx->idx == 8U) {
		ytm32_dma_zli_timing_mark(YTM32_DMA_ZLI_TIMING_ZEPHYR_DMA_CB_ENTRY);
	}
	if (chan->zephyr_cb) {
		chan->zephyr_cb(dev, chan->zephyr_cb_data, ctx->idx, hal_status);
	}
	if (ctx->idx == 8U) {
		ytm32_dma_zli_timing_mark(YTM32_DMA_ZLI_TIMING_ZEPHYR_DMA_CB_EXIT);
	}
}

/* ─────────────────────────── IRQ handlers ─────────────────────────── */

/*
 * Per-channel ISR wrapper — receives the channel index as arg.
 * Registered via IRQ_CONNECT at the DTS priority through _isr_wrapper.
 */
static void dma_ytm32_chan_irq(const void *arg)
{
	ytm32_dma_hal_irq((uint8_t)(uintptr_t)arg);
}

#if defined(CONFIG_DMA_YTM32_CH8_ZERO_LATENCY)
ISR_DIRECT_DECLARE(dma_ytm32_ch8_zli_isr)
{
	ytm32_dma_zli_timing_mark(YTM32_DMA_ZLI_TIMING_IRQ_ENTRY);
	ytm32_dma_hal_irq(8U);
	ytm32_dma_zli_timing_mark(YTM32_DMA_ZLI_TIMING_IRQ_EXIT);
	return 0;
}
#endif

static void dma_ytm32_error_irq(const void *arg)
{
	ARG_UNUSED(arg);
	ytm32_dma_hal_error_irq();
}

/* ─────────────────────────── Zephyr DMA API ─────────────────────────── */

static int dma_ytm32_config(const struct device *dev, uint32_t channel,
			    struct dma_config *cfg)
{
	struct ytm32_dma_data *data = dev->data;
	struct ytm32_dma_chan *chan = &data->chan[channel];
	uint8_t dir;
	int ret;

	if (channel >= YTM32_DMA_NUM_CHANNELS || cfg->block_count != 1) {
		return -EINVAL;
	}

	switch (cfg->channel_direction) {
	case PERIPHERAL_TO_MEMORY: dir = YTM32_DMA_DIR_PERIPH2MEM; break;
	case MEMORY_TO_PERIPHERAL: dir = YTM32_DMA_DIR_MEM2PERIPH;  break;
	case MEMORY_TO_MEMORY:     dir = YTM32_DMA_DIR_MEM2MEM;     break;
	default:
		LOG_ERR("ch%u: unsupported direction %u", channel,
			cfg->channel_direction);
		return -ENOTSUP;
	}

	if (chan->configured) {
		ytm32_dma_hal_channel_release((uint8_t)channel);
		chan->configured = false;
	}

	chan->zephyr_cb      = cfg->dma_callback;
	chan->zephyr_cb_data = cfg->user_data;
	chan->dir            = cfg->channel_direction;

	ret = ytm32_dma_hal_channel_config(
		(uint8_t)channel,
		(uint8_t)cfg->dma_slot,
		cfg->head_block->source_address,
		cfg->head_block->dest_address,
		dir,
		(uint8_t)MAX(cfg->source_data_size, cfg->dest_data_size),
		cfg->head_block->block_size,
		dma_ytm32_hal_cb,
		&s_cbs[channel]);

	if (ret < 0) {
		LOG_ERR("ch%u: channel_config failed: %d", channel, ret);
		return ret;
	}

	chan->configured = true;
	return 0;
}

static int dma_ytm32_start(const struct device *dev, uint32_t channel)
{
	struct ytm32_dma_data *data = dev->data;

	if (channel >= YTM32_DMA_NUM_CHANNELS || !data->chan[channel].configured) {
		return -EINVAL;
	}
	return ytm32_dma_hal_start((uint8_t)channel);
}

static int dma_ytm32_stop(const struct device *dev, uint32_t channel)
{
	ARG_UNUSED(dev);
	if (channel >= YTM32_DMA_NUM_CHANNELS) {
		return -EINVAL;
	}
	return ytm32_dma_hal_stop((uint8_t)channel);
}

static int dma_ytm32_get_status(const struct device *dev, uint32_t channel,
				struct dma_status *stat)
{
	struct ytm32_dma_data *data = dev->data;

	if (channel >= YTM32_DMA_NUM_CHANNELS) {
		return -EINVAL;
	}

	uint32_t remaining = ytm32_dma_hal_remaining((uint8_t)channel);

	stat->busy           = (remaining > 0U);
	stat->free           = !stat->busy;
	stat->pending_length  = remaining;
	stat->dir             = data->chan[channel].dir;

	return ytm32_dma_hal_error((uint8_t)channel) ? -EIO : 0;
}

/* ─────────────────────────── init ─────────────────────────── */

static int dma_ytm32_init(const struct device *dev)
{
	const struct ytm32_dma_config *cfg = dev->config;
	struct ytm32_dma_data *data = dev->data;
	int ret;

	data->ctx.magic        = DMA_MAGIC;
	data->ctx.dma_channels = YTM32_DMA_NUM_CHANNELS;
	data->ctx.atomic       = data->chan_atomic;

	/* Pre-fill HAL callback contexts — never changes after init. */
	for (uint8_t i = 0; i < YTM32_DMA_NUM_CHANNELS; i++) {
		s_cbs[i].dev = dev;
		s_cbs[i].idx = i;
	}

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("clock device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret < 0) {
		LOG_ERR("clock_control_on failed: %d", ret);
		return ret;
	}

	ret = ytm32_dma_hal_init();
	if (ret < 0) {
		LOG_ERR("ytm32_dma_hal_init failed: %d", ret);
		return ret;
	}

	cfg->irq_config_func();

	LOG_INF("ready (%u channels)", YTM32_DMA_NUM_CHANNELS);
	return 0;
}

/* ─────────────────────────── driver API + instance macro ─────────────────────────── */

static DEVICE_API(dma, dma_ytm32_api) = {
	.config     = dma_ytm32_config,
	.start      = dma_ytm32_start,
	.stop       = dma_ytm32_stop,
	.get_status = dma_ytm32_get_status,
};

/* DMA channel IRQ registration.  When requested, ch8 is connected as a Zephyr
 * zero-latency direct IRQ for the ADC DMA path; all other channels keep the
 * normal _isr_wrapper trampoline so their callbacks may use regular ISR-safe
 * Zephyr APIs.
 */
#define YTM32_DMA_IRQ_CONNECT_NORMAL_CHAN(i, n) \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, i, irq), \
		    DT_INST_IRQ_BY_IDX(n, i, priority), \
		    dma_ytm32_chan_irq, (const void *)(uintptr_t)(i), 0); \
	irq_enable(DT_INST_IRQ_BY_IDX(n, i, irq))

#if defined(CONFIG_DMA_YTM32_CH8_ZERO_LATENCY)
#define YTM32_DMA_IRQ_CONNECT_CHAN(i, n) \
	COND_CODE_1(IS_EQ(i, 8), \
		(IRQ_DIRECT_CONNECT(DT_INST_IRQ_BY_IDX(n, i, irq), \
				    0, dma_ytm32_ch8_zli_isr, IRQ_ZERO_LATENCY); \
		 irq_enable(DT_INST_IRQ_BY_IDX(n, i, irq))), \
		(YTM32_DMA_IRQ_CONNECT_NORMAL_CHAN(i, n)))
#else
#define YTM32_DMA_IRQ_CONNECT_CHAN(i, n) \
	YTM32_DMA_IRQ_CONNECT_NORMAL_CHAN(i, n)
#endif

#define YTM32_DMA_INIT(n) \
	static struct ytm32_dma_data dma_ytm32_data_##n; \
	\
	static void dma_ytm32_irq_config_##n(void) \
	{ \
		LISTIFY(16, YTM32_DMA_IRQ_CONNECT_CHAN, (;), n); \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, 16, irq), \
			    DT_INST_IRQ_BY_IDX(n, 16, priority), \
			    dma_ytm32_error_irq, DEVICE_DT_INST_GET(n), 0); \
		irq_enable(DT_INST_IRQ_BY_IDX(n, 16, irq)); \
	} \
	\
	static const struct ytm32_dma_config dma_ytm32_config_##n = { \
		.base            = DT_INST_REG_ADDR(n), \
		.clock_dev       = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)), \
		.clock_subsys    = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id), \
		.irq_config_func = dma_ytm32_irq_config_##n, \
	}; \
	\
	DEVICE_DT_INST_DEFINE(n, dma_ytm32_init, NULL, \
			      &dma_ytm32_data_##n, &dma_ytm32_config_##n, \
			      PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY, \
			      &dma_ytm32_api);

DT_INST_FOREACH_STATUS_OKAY(YTM32_DMA_INIT)
