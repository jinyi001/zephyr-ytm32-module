/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr ADC driver for YTMicro YTM32 ADC.
 *
 * Phase 1: interrupt-driven, software-triggered (adc_read / adc_read_async).
 * Phase 2: hardware-triggered DMA continuous sampling (adc_ytm32_dma_start).
 *
 * Hardware signal chain for Phase 2:
 *   ETMR0 OTRIG (INITTEN=1, counter-bottom pulse)
 *     → TMU route: eTMR0_INIT_TRIG (22) → ADC0_EXT_TRIG (12)
 *     → ADC0 hardware trigger → sequence conversion → FIFO
 *     → DMA ch8 (DMA_REQ_ADC0=42) → SRAM buffer
 *     → DMA complete ISR → user callback
 */

#define DT_DRV_COMPAT ytmicro_ytm32_adc

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/ytm32_soc_clock.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "adc_driver.h"
#include "tmu_driver.h"
#include "../dma/ytm32_dma_hal.h"
#include <zephyr/drivers/adc/adc_ytm32.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

LOG_MODULE_REGISTER(adc_ytm32, CONFIG_ADC_LOG_LEVEL);

/* Maximum external channel count across all YTM32 variants */
#define YTM32_ADC_MAX_CHANS 38U

/* ADC FIFO register offset within ADC peripheral (from the struct layout) */
#define YTM32_ADC_FIFO_OFFSET 0x4CU

/* TMU instance is always 0 on this SoC */
#define YTM32_TMU_INSTANCE 0U
/* TMU target: ADC0 external trigger input */
#define YTM32_TMU_TARGET_ADC0_EXT_TRIG TMU_TARGET_MODULE_ADC0_EXT_TRIG

/* Sentinel meaning "no hardware trigger configured" */
#define YTM32_ADC_NO_HW_TRIG UINT32_MAX

/* ──────────────────────── config / data structs ──────────────────────── */

struct adc_ytm32_config {
	uint32_t                         base;
	uint8_t                          instance;
	const struct device             *clock_dev;
	clock_control_subsys_t           clock_subsys;
	uint32_t                         clk_div;
	const struct pinctrl_dev_config *pincfg;
	void (*irq_config_func)(void);
	/* Phase 2: DMA and hardware trigger (optional) */
	const struct device             *dma_dev;     /* NULL if DMA not wired */
	uint8_t                          dma_channel;
	uint8_t                          dma_slot;    /* DMAMUX request source */
	uint32_t                         hw_trig_src; /* TMU trigger source enum value */
};

struct adc_ytm32_data {
	/* Phase 1 */
	struct adc_context   ctx;
	const struct device *dev;
	uint16_t            *buffer;
	uint16_t            *repeat_buffer;
	uint8_t              channel_count;
	uint8_t              sample_time[YTM32_ADC_MAX_CHANS];
	/* Phase 2 */
	bool                 dma_active;
	struct adc_ytm32_dma_config dma_cfg; /* copy of user config */
};

/* ──────────────────────── helpers ──────────────────────── */

static adc_resolution_t bits_to_resolution(uint8_t bits)
{
	switch (bits) {
	case 10: return ADC_RESOLUTION_10BIT;
	case  8: return ADC_RESOLUTION_8BIT;
	case  6: return ADC_RESOLUTION_6BIT;
	default: return ADC_RESOLUTION_12BIT;
	}
}

static uint8_t channels_to_sequence(uint32_t channels_mask,
				    adc_inputchannel_t *chsel,
				    uint8_t *sample_times,
				    uint8_t *max_smp_out)
{
	uint8_t slot = 0;
	uint8_t max_smp = ADC_DEFAULT_SAMPLE_TIME;

	for (uint8_t ch = 0;
	     channels_mask != 0U && slot < ADC_CHSEL_COUNT;
	     ch++, channels_mask >>= 1U) {
		if ((channels_mask & 1U) == 0U) {
			continue;
		}
		chsel[slot++] = (adc_inputchannel_t)ch;
		if (sample_times[ch] > max_smp) {
			max_smp = sample_times[ch];
		}
	}

	if (max_smp_out != NULL) {
		*max_smp_out = max_smp;
	}
	return slot;
}

/* ──────────────────────── Phase 1: Zephyr ADC API ──────────────────────── */

static int adc_ytm32_channel_setup(const struct device *dev,
				   const struct adc_channel_cfg *channel_cfg)
{
	struct adc_ytm32_data *data = dev->data;

	if (channel_cfg->channel_id >= YTM32_ADC_MAX_CHANS) {
		LOG_ERR("channel %u out of range", channel_cfg->channel_id);
		return -EINVAL;
	}
	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("only ADC_GAIN_1 supported");
		return -ENOTSUP;
	}
	if (channel_cfg->reference != ADC_REF_VDD_1) {
		LOG_ERR("only ADC_REF_VDD_1 supported");
		return -ENOTSUP;
	}
	if (channel_cfg->differential) {
		LOG_ERR("differential not supported");
		return -ENOTSUP;
	}

	uint32_t acq = channel_cfg->acquisition_time;

	if (acq == ADC_ACQ_TIME_DEFAULT) {
		data->sample_time[channel_cfg->channel_id] = ADC_DEFAULT_SAMPLE_TIME;
	} else if (ADC_ACQ_TIME_UNIT(acq) == ADC_ACQ_TIME_TICKS) {
		data->sample_time[channel_cfg->channel_id] =
			(uint8_t)MIN(ADC_ACQ_TIME_VALUE(acq), 0xFFU);
	} else {
		LOG_ERR("unsupported acquisition time unit");
		return -ENOTSUP;
	}

	return 0;
}

static int adc_ytm32_start_read(const struct device *dev,
				const struct adc_sequence *sequence)
{
	struct adc_ytm32_data *data = dev->data;

	if (data->dma_active) {
		LOG_ERR("DMA mode active; stop it before calling adc_read");
		return -EBUSY;
	}
	if (sequence->channels == 0 || sequence->buffer == NULL) {
		return -EINVAL;
	}

	switch (sequence->resolution) {
	case 6: case 8: case 10: case 12:
		break;
	default:
		LOG_ERR("unsupported resolution %u", sequence->resolution);
		return -ENOTSUP;
	}

	uint8_t ch_count = (uint8_t)POPCOUNT(sequence->channels);
	size_t needed = (size_t)ch_count * sizeof(uint16_t);

	if (sequence->options) {
		needed *= (1U + sequence->options->extra_samplings);
	}
	if (sequence->buffer_size < needed) {
		LOG_ERR("buffer too small: need %zu, got %zu", needed,
			sequence->buffer_size);
		return -ENOMEM;
	}

	data->repeat_buffer = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int adc_ytm32_read(const struct device *dev,
			  const struct adc_sequence *sequence)
{
	struct adc_ytm32_data *data = dev->data;
	int err;

	adc_context_lock(&data->ctx, false, NULL);
	err = adc_ytm32_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_ytm32_read_async(const struct device *dev,
				const struct adc_sequence *sequence,
				struct k_poll_signal *async)
{
	struct adc_ytm32_data *data = dev->data;
	int err;

	adc_context_lock(&data->ctx, true, async);
	err = adc_ytm32_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}
#endif

/* ──────────────────────── adc_context callbacks (Phase 1) ──────────────── */

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_ytm32_data *data =
		CONTAINER_OF(ctx, struct adc_ytm32_data, ctx);
	const struct device *dev = data->dev;
	const struct adc_ytm32_config *config = dev->config;
	uint8_t inst = config->instance;
	adc_converter_config_t conv;

	ADC_DRV_InitConverterStruct(&conv);

	uint8_t max_smp;
	uint8_t slot = channels_to_sequence(ctx->sequence.channels,
					    conv.sequenceConfig.channels,
					    data->sample_time, &max_smp);

	data->channel_count = slot;
	data->buffer = (uint16_t *)ctx->sequence.buffer
		       + (size_t)ctx->sampling_index * slot;

	conv.sequenceConfig.totalChannels     = slot;
	conv.sequenceConfig.sequenceMode      = ADC_CONV_LOOP;
	conv.sequenceConfig.sequenceIntEnable = true;
	conv.sequenceConfig.ovrunIntEnable    = true;
	conv.sampleTime   = max_smp;
	conv.clockDivider = (adc_clk_divide_t)config->clk_div;
	conv.resolution   = bits_to_resolution(ctx->sequence.resolution);
	conv.align        = ADC_ALIGN_RIGHT;

	ADC_DRV_ConfigConverter(inst, &conv);
	ADC_DRV_Start(inst);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct adc_ytm32_data *data =
		CONTAINER_OF(ctx, struct adc_ytm32_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

/* ──────────────────────── ISR (Phase 1) ──────────────────────── */

static void adc_ytm32_isr(const struct device *dev)
{
	struct adc_ytm32_data *data = dev->data;
	const struct adc_ytm32_config *config = dev->config;
	uint8_t inst = config->instance;

	/* DMA mode: ADC interrupts are disabled — this path should not fire */
	if (data->dma_active) {
		return;
	}

	if (ADC_DRV_GetOvrRunOfConversionFlag(inst)) {
		ADC_DRV_ClearOvrFlagCmd(inst);
		ADC_DRV_Stop(inst);
		LOG_ERR("ADC FIFO overrun");
		adc_context_complete(&data->ctx, -EIO);
		return;
	}

	if (ADC_DRV_GetEndOfSequenceFlag(inst)) {
		ADC_DRV_ClearEoseqFlagCmd(inst);

		for (uint8_t i = 0U; i < data->channel_count; i++) {
			data->buffer[i] = ADC_DRV_ReadFIFO(inst);
		}

		ADC_DRV_Stop(inst);
		adc_context_on_sampling_done(&data->ctx, dev);
	}
}

/* ──────────────────────── Phase 2: TMU helper ──────────────────────── */

static int adc_ytm32_tmu_route(uint32_t trig_src)
{
	/*
	 * Enable TMU clock via vendor SDK directly (no Zephyr TMU driver).
	 * TMU_CLK = 25 on MD1.
	 */
	extern void CLOCK_DRV_SetModuleClock(uint32_t clockName, bool gate,
					     uint32_t src, uint32_t div);
	CLOCK_DRV_SetModuleClock(25U /* TMU_CLK */, true, 0U, 0U);

	return (TMU_DRV_SetTrigSourceForTargetModule(
			YTM32_TMU_INSTANCE,
			YTM32_TMU_TARGET_ADC0_EXT_TRIG,
			(tmu_trigger_source_t)trig_src) == STATUS_SUCCESS)
		? 0 : -EIO;
}

/* ──────────────────────── Phase 2: DMA callback ──────────────────────── */

static void adc_ytm32_dma_cb(void *user_data, int hal_status)
{
	const struct device *dev = user_data;
	struct adc_ytm32_data *data = dev->data;
	const struct adc_ytm32_config *config = dev->config;

	if (!data->dma_active) {
		return;
	}

	if (hal_status != 0) {
		LOG_ERR("DMA error %d", hal_status);
		data->dma_active = false;
		ADC_DRV_Stop(config->instance);
		return;
	}

	/* Notify user with the completed buffer */
	if (data->dma_cfg.cb != NULL) {
		data->dma_cfg.cb(dev, data->dma_cfg.buf,
				 data->channel_count,
				 data->dma_cfg.depth,
				 data->dma_cfg.user_data);
	}

	/* Re-arm DMA for next batch; ADC keeps running (hardware trigger) */
	ytm32_dma_hal_start(config->dma_channel);
}

/* ──────────────────────── Phase 2: public API ──────────────────────── */

int adc_ytm32_dma_start(const struct device *dev,
			const struct adc_ytm32_dma_config *cfg)
{
	struct adc_ytm32_data *data = dev->data;
	const struct adc_ytm32_config *config = dev->config;
	adc_converter_config_t conv;
	uint8_t max_smp;
	int ret;

	if (config->dma_dev == NULL) {
		LOG_ERR("DMA not configured in DTS for this ADC instance");
		return -ENOTSUP;
	}
	if (config->hw_trig_src == YTM32_ADC_NO_HW_TRIG) {
		LOG_ERR("hw-trigger-source not set in DTS");
		return -ENOTSUP;
	}
	if (cfg->channels == 0 || cfg->buf == NULL || cfg->depth == 0 ||
	    cfg->cb == NULL) {
		return -EINVAL;
	}
	if (data->dma_active) {
		return -EBUSY;
	}

	switch (cfg->resolution) {
	case 6: case 8: case 10: case 12:
		break;
	default:
		return -EINVAL;
	}

	uint8_t ch_count = (uint8_t)POPCOUNT(cfg->channels);

	if (ch_count > ADC_CHSEL_COUNT) {
		LOG_ERR("too many channels (%u > %u)", ch_count, ADC_CHSEL_COUNT);
		return -EINVAL;
	}

	/* Save config for re-arm in callback */
	data->dma_cfg     = *cfg;
	data->channel_count = ch_count;

	/* 1. Route TMU: ETMR0_INIT_TRIG → ADC0_EXT_TRIG */
	ret = adc_ytm32_tmu_route(config->hw_trig_src);
	if (ret < 0) {
		return ret;
	}

	/* 2. Configure DMA: FIFO → buffer, total_count = ch_count × depth */
	uint32_t total = (uint32_t)ch_count * cfg->depth;
	uintptr_t fifo_addr = (uintptr_t)(config->base + YTM32_ADC_FIFO_OFFSET);

	ret = ytm32_dma_hal_channel_config_loop(
		config->dma_channel,
		config->dma_slot,
		fifo_addr,
		(uintptr_t)cfg->buf,
		sizeof(uint16_t),
		total,
		adc_ytm32_dma_cb,
		(void *)dev);
	if (ret < 0) {
		LOG_ERR("DMA loop config failed: %d", ret);
		return ret;
	}

	/* 3. Configure ADC: hardware trigger, DMA enabled, no interrupts */
	ADC_DRV_InitConverterStruct(&conv);
	channels_to_sequence(cfg->channels, conv.sequenceConfig.channels,
			     data->sample_time, &max_smp);

	conv.sequenceConfig.totalChannels     = ch_count;
	conv.sequenceConfig.sequenceMode      = ADC_CONV_LOOP;
	conv.sequenceConfig.sequenceIntEnable = false;
	conv.sequenceConfig.ovrunIntEnable    = false;
	conv.sampleTime   = max_smp;
	conv.clockDivider = (adc_clk_divide_t)config->clk_div;
	conv.resolution   = bits_to_resolution(cfg->resolution);
	conv.align        = ADC_ALIGN_RIGHT;
	conv.trigger      = ADC_TRIGGER_HARDWARE;
	conv.dmaEnable    = true;
	/* watermark=0: DMA request fires on every FIFO write (one sample at a time) */
	conv.dmaWaterMark = 0U;

	ADC_DRV_ConfigConverter(config->instance, &conv);

	/* 4. Start DMA then enable ADC (order matters: DMA ready before first trigger) */
	ret = ytm32_dma_hal_start(config->dma_channel);
	if (ret < 0) {
		return ret;
	}

	data->dma_active = true;
	ADC_DRV_Enable(config->instance);

	return 0;
}

int adc_ytm32_dma_stop(const struct device *dev)
{
	struct adc_ytm32_data *data = dev->data;
	const struct adc_ytm32_config *config = dev->config;

	if (!data->dma_active) {
		return 0;
	}

	data->dma_active = false;
	ADC_DRV_Stop(config->instance);
	ADC_DRV_Disable(config->instance);
	ytm32_dma_hal_stop(config->dma_channel);
	ytm32_dma_hal_channel_release(config->dma_channel);

	return 0;
}

/* ──────────────────────── driver init ──────────────────────── */

static int adc_ytm32_init(const struct device *dev)
{
	struct adc_ytm32_data *data = dev->data;
	const struct adc_ytm32_config *config = dev->config;
	int ret;

	data->dev = dev;

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0 && ret != -ENOENT) {
		return ret;
	}

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("clock device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret < 0) {
		LOG_ERR("failed to enable ADC clock: %d", ret);
		return ret;
	}

	if (config->dma_dev != NULL && !device_is_ready(config->dma_dev)) {
		LOG_ERR("DMA device not ready");
		return -ENODEV;
	}

	ADC_DRV_Reset(config->instance);
	config->irq_config_func();
	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

/* ──────────────────────── driver API table ──────────────────────── */

static DEVICE_API(adc, adc_ytm32_driver_api) = {
	.channel_setup = adc_ytm32_channel_setup,
	.read          = adc_ytm32_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async    = adc_ytm32_read_async,
#endif
	.ref_internal  = 0,
};

/* ──────────────────────── per-instance instantiation ──────────────────── */

/*
 * Resolve optional DMA properties at compile time.
 * COND_CODE_1 picks the DMA-enabled path only when the 'dmas' property exists
 * AND the dma0 node is enabled.
 */
#define YTM32_ADC_DMA_DEV(inst) \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas), \
		(DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, rx))), \
		(NULL))

#define YTM32_ADC_DMA_CH(inst) \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas), \
		(DT_INST_DMAS_CELL_BY_NAME(inst, rx, channel)), \
		(0U))

#define YTM32_ADC_DMA_SLOT(inst) \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas), \
		(DT_INST_DMAS_CELL_BY_NAME(inst, rx, trigsrc)), \
		(0U))

#define YTM32_ADC_HW_TRIG(inst) \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, ytmicro_hw_trigger_source), \
		(DT_INST_PROP(inst, ytmicro_hw_trigger_source)), \
		(YTM32_ADC_NO_HW_TRIG))

#define YTM32_ADC_INIT(inst)							\
	PINCTRL_DT_INST_DEFINE(inst);						\
										\
	static void adc_ytm32_irq_config_##inst(void)				\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(inst),					\
			    DT_INST_IRQ(inst, priority),			\
			    adc_ytm32_isr,					\
			    DEVICE_DT_INST_GET(inst), 0);			\
		irq_enable(DT_INST_IRQN(inst));					\
	}									\
										\
	static const struct adc_ytm32_config adc_ytm32_cfg_##inst = {		\
		.base         = DT_INST_REG_ADDR(inst),				\
		.instance     = 0U,						\
		.clock_dev    = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),	\
		.clock_subsys = (clock_control_subsys_t)			\
				DT_INST_CLOCKS_CELL(inst, id),			\
		.clk_div      = DT_INST_PROP(inst,				\
				ytmicro_functional_clock_divider),		\
		.pincfg       = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),		\
		.irq_config_func = adc_ytm32_irq_config_##inst,			\
		.dma_dev      = YTM32_ADC_DMA_DEV(inst),			\
		.dma_channel  = YTM32_ADC_DMA_CH(inst),			\
		.dma_slot     = YTM32_ADC_DMA_SLOT(inst),			\
		.hw_trig_src  = YTM32_ADC_HW_TRIG(inst),			\
	};									\
										\
	static struct adc_ytm32_data adc_ytm32_data_##inst = {			\
		ADC_CONTEXT_INIT_TIMER(adc_ytm32_data_##inst, ctx),		\
		ADC_CONTEXT_INIT_LOCK(adc_ytm32_data_##inst, ctx),		\
		ADC_CONTEXT_INIT_SYNC(adc_ytm32_data_##inst, ctx),		\
	};									\
										\
	DEVICE_DT_INST_DEFINE(inst, adc_ytm32_init, NULL,			\
			      &adc_ytm32_data_##inst,				\
			      &adc_ytm32_cfg_##inst,				\
			      POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,		\
			      &adc_ytm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(YTM32_ADC_INIT)
