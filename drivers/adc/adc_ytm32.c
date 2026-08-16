/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr ADC driver for YTMicro YTM32 ADC — Phase 1 core: interrupt-driven,
 * software-triggered conversions (adc_read / adc_read_async).
 *
 * The optional Phase 2 extension — hardware-triggered DMA continuous sampling
 * exposed through <zephyr/drivers/adc/adc_ytm32.h> — lives in adc_ytm32_dma.c.
 * Both translation units share the per-instance config and the
 * adc_ytm32_shared state defined in adc_ytm32_priv.h.  This file owns the full
 * driver data layout (it embeds the adc_context) and exposes the shared slice
 * to the DMA TU via adc_ytm32_shared().
 */

#define DT_DRV_COMPAT ytmicro_ytm32_adc

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "device_registers.h"
#include "adc_driver.h"
#include <zephyr/dt-bindings/adc/ytm32b1md1-adc.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#include "adc_ytm32_priv.h"
#include "adc_ytm32_logic.h"

/*
 * Errata E600001 (vendor numbering: ADC_ERRATA_E0002).
 *
 * After the ADC has been enabled but left idle for a long time (>1 s, or ~5 ms
 * at high temperature), the FIRST channel of the next conversion sequence reads
 * an inaccurate value.  This affects both software- and hardware-triggered
 * conversions; continuous conversion is unaffected.
 *
 * Software-trigger workaround (recommended by the errata): IPC-reset the ADC
 * immediately before each conversion.  The vendor HAL implements exactly this
 * inside ADC_Enable() (adc_hw_access.h), guarded by ADC_ERRATA_E0002, and the
 * Phase 1 path below reaches it on every read via adc_context_start_sampling()
 * -> ADC_DRV_Start() -> ADC_Enable().  This keeps ALL channels (including the
 * first) accurate, so — unlike the hardware-trigger workaround — no channel
 * slot has to be sacrificed.
 *
 * The mitigation therefore depends entirely on the HAL macro staying defined.
 * If a future HAL bump drops it, software reads would silently start returning
 * bad first-channel data, so fail the build loudly instead.
 */
#if !defined(ADC_ERRATA_E0002)
#error "ADC_ERRATA_E0002 not defined by HAL — E600001 first-channel workaround is disabled"
#endif

LOG_MODULE_REGISTER(adc_ytm32, CONFIG_ADC_LOG_LEVEL);

/* CIM trigger-select values come from dt-bindings/adc/ytm32b1md1-adc.h. */

/* ──────────────────────── driver data ──────────────────────── */

struct adc_ytm32_data {
	struct adc_context        ctx;
	const struct device      *dev;
	uint16_t                 *buffer;
	uint16_t                 *repeat_buffer;
	/* State the Phase 2 DMA extension also touches (see adc_ytm32_priv.h). */
	struct adc_ytm32_shared   shared;
};

struct adc_ytm32_shared *adc_ytm32_shared(const struct device *dev)
{
	struct adc_ytm32_data *data = dev->data;

	return &data->shared;
}

int adc_ytm32_get_timing(const struct device *dev, uint8_t sample_time,
				 struct adc_ytm32_timing *out)
{
	struct adc_ytm32_data *data;
	const struct adc_ytm32_config *config;
	int ret;

	if (dev == NULL || out == NULL) {
		return -EINVAL;
	}
	data = dev->data;
	config = dev->config;
	if (data->shared.adc_clock_hz == 0U) {
		return -EAGAIN;
	}

	ret = adc_ytm32_validate_timing(data->shared.adc_clock_hz,
					 sample_time, config->adc_start_time);
	if (ret < 0) {
		return ret;
	}

	out->fadc_hz = data->shared.adc_clock_hz;
	out->sample_time = sample_time;
	out->sample_ns = adc_ytm32_ticks_to_ns(sample_time + 2U,
						 data->shared.adc_clock_hz);
	out->conversion_ns = adc_ytm32_ticks_to_ns(sample_time + 14U,
						     data->shared.adc_clock_hz);
	out->startup_ns = adc_ytm32_ticks_to_ns(config->adc_start_time + 1U,
						   data->shared.adc_clock_hz);
	return 0;
}

int adc_ytm32_get_cim_trigger_select(const struct device *dev,
					     uint32_t *select)
{
	const struct adc_ytm32_config *config;

	if (dev == NULL || select == NULL) {
		return -EINVAL;
	}
	config = dev->config;
	if (config->instance != 0U) {
		return -ENOTSUP;
	}

	*select = (CIM->CTRL & CIM_CTRL_ADC0_TRIG_SEL_MASK) >>
		CIM_CTRL_ADC0_TRIG_SEL_SHIFT;
	return 0;
}

/* ──────────────────────── helpers (shared with Phase 2) ─────────────────── */

adc_resolution_t adc_ytm32_bits_to_resolution(uint8_t bits)
{
	switch (bits) {
	case 10: return ADC_RESOLUTION_10BIT;
	case  8: return ADC_RESOLUTION_8BIT;
	case  6: return ADC_RESOLUTION_6BIT;
	default: return ADC_RESOLUTION_12BIT;
	}
}

uint8_t adc_ytm32_channels_to_sequence(uint64_t channels_mask,
				       adc_inputchannel_t *chsel,
				       uint8_t *sample_times,
				       uint8_t *max_smp_out)
{
	uint8_t sequence[ADC_CHSEL_COUNT] = {0};
	uint8_t channel_count = 0U;
	uint8_t max_smp = 0U;
	int ret;

	ret = adc_ytm32_sequence_expand(channels_mask, NULL, 0U,
					ADC_CHSEL_COUNT, sequence, sample_times,
					YTM32_ADC_MAX_CHANS, &channel_count,
					&max_smp);
	if (ret < 0) {
		if (max_smp_out != NULL) {
			*max_smp_out = 0U;
		}
		return 0U;
	}

	for (uint8_t index = 0U; index < channel_count; index++) {
		chsel[index] = (adc_inputchannel_t)sequence[index];
	}
	if (max_smp_out != NULL) {
		*max_smp_out = max_smp;
	}
	return channel_count;
}

int adc_ytm32_sequence_from_config(const struct adc_ytm32_config *config,
					   uint64_t channels_mask,
					   adc_inputchannel_t *chsel,
					   const uint8_t *sample_times,
					   uint8_t *channel_count_out,
					   uint8_t *max_smp_out)
{
	uint8_t sequence[ADC_CHSEL_COUNT] = {0};
	uint8_t channel_count;
	uint8_t max_smp;
	int ret;

	if (config == NULL || chsel == NULL || sample_times == NULL ||
	    channel_count_out == NULL || max_smp_out == NULL) {
		return -EINVAL;
	}

	ret = adc_ytm32_sequence_expand(
		channels_mask,
		config->sequence_order,
		config->sequence_order_count,
		ADC_CHSEL_COUNT,
		sequence,
		sample_times,
		YTM32_ADC_MAX_CHANS,
		&channel_count,
		&max_smp);
	if (ret < 0) {
		return ret;
	}

	for (uint8_t index = 0U; index < channel_count; index++) {
		chsel[index] = (adc_inputchannel_t)sequence[index];
	}
	*channel_count_out = channel_count;
	*max_smp_out = max_smp;
	return 0;
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
		data->shared.sample_time[channel_cfg->channel_id] =
			YTM32_ADC_DEFAULT_SAMPLE_TIME;
	} else if (ADC_ACQ_TIME_UNIT(acq) == ADC_ACQ_TIME_TICKS) {
		data->shared.sample_time[channel_cfg->channel_id] =
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
	const struct adc_ytm32_config *config = dev->config;
	adc_inputchannel_t sequence_channels[ADC_CHSEL_COUNT];
	uint8_t ch_count;
	uint8_t max_smp;
	int ret;

	if (data->shared.dma_active) {
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

	ret = adc_ytm32_sequence_from_config(config, sequence->channels,
						     sequence_channels,
						     data->shared.sample_time,
						     &ch_count, &max_smp);
	if (ret < 0) {
		LOG_ERR("invalid ADC sequence order: %d", ret);
		return ret;
	}
	ret = adc_ytm32_validate_timing(data->shared.adc_clock_hz, max_smp,
						config->adc_start_time);
	if (ret < 0) {
		LOG_ERR("ADC sequence timing is outside DS v1.9 limits: %d", ret);
		return ret;
	}
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

	uint8_t slot;
	uint8_t max_smp;
	int ret = adc_ytm32_sequence_from_config(config, ctx->sequence.channels,
							conv.sequenceConfig.channels,
							data->shared.sample_time,
							&slot, &max_smp);
	if (ret < 0) {
		LOG_ERR("invalid ADC sequence order during sampling: %d", ret);
		adc_context_complete(ctx, ret);
		return;
	}

	data->shared.channel_count = slot;
	/* sampling_index is only reset by adc_context_start_read when
	 * sequence->options != NULL.  For no-options (single) reads it retains
	 * its value from the previous extra_samplings call, so using it as an
	 * offset would write past the caller's single-element buffer.  Always
	 * use index 0 when there are no options; the framework never increments
	 * sampling_index in that code path anyway.
	 */
	uint16_t idx = ctx->sequence.options ? ctx->sampling_index : 0U;
	data->buffer = (uint16_t *)ctx->sequence.buffer + (size_t)idx * slot;

	conv.sequenceConfig.totalChannels     = slot;
	/*
	 * LOOP (continuous) mode — must NOT be changed to discontinuous: the
	 * E600001 software-trigger workaround (see top-of-file note) is only
	 * valid for single/continuous mode, not discontinuous single-channel
	 * triggering.  The ISR stops the ADC after one end-of-sequence.
	 */
	conv.sequenceConfig.sequenceMode      = ADC_CONV_LOOP;
	conv.sequenceConfig.sequenceIntEnable = true;
	conv.sequenceConfig.ovrunIntEnable    = true;
	conv.sampleTime   = max_smp;
	conv.startTime    = config->adc_start_time;
	/* ADC module internal clock divider (CFG1.PRS = ytmicro,adc-clock-divider).
	 * Separate from the IPC clock tree divider (ytmicro,functional-clock-divider).
	 * Vendor HAL semantics are n+1, not 2^n:
	 *   FADC = func_clk / (CFG1.PRS + 1).
	 * DS v1.9 requires FADC in [4, 32] MHz; init and sequence setup validate
	 * this before reaching the HAL.
	 */
	conv.clockDivider = (adc_clk_divide_t)config->adc_clk_div;
	conv.resolution   = adc_ytm32_bits_to_resolution(ctx->sequence.resolution);
	conv.align        = ADC_ALIGN_RIGHT;

	ADC_DRV_ConfigConverter(inst, &conv);
	/*
	 * ADC_DRV_Start() -> ADC_Enable() performs the E600001 IPC reset before
	 * every conversion, so the first channel stays accurate even after a
	 * long idle period.
	 */
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

/* ──────────────────────── ISR (Phase 1 + DMA notify) ───────────────── */

static void adc_ytm32_isr(const struct device *dev)
{
	struct adc_ytm32_data *data = dev->data;
	const struct adc_ytm32_config *config = dev->config;
	uint8_t inst = config->instance;

	/* DMA mode: ADC peripheral interrupts are disabled.  The DMA top-half pends
	 * this ADC IRQ in software only to run the normal notification callback from
	 * regular Zephyr ISR context (handler lives in adc_ytm32_dma.c).
	 */
	if (data->shared.dma_active || data->shared.dma_error != 0) {
		adc_ytm32_dma_notify_isr(dev);
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
		/* Stop the ADC BEFORE reading the FIFO.  In LOOP mode the hardware
		 * immediately starts a new conversion after EOSEQ.  If we stop here
		 * first, no additional conversion can complete and re-assert EOSEQ
		 * while we are in this ISR.  Without this ordering, the extra EOSEQ
		 * fires a spurious interrupt after adc_read() returns, which calls
		 * adc_context_complete() and increments ctx->sync — causing the
		 * next adc_read() to return immediately without waiting for a real
		 * conversion (buffer stays 0xFFFF).
		 */
		ADC_DRV_Stop(inst);

		for (uint8_t i = 0U; i < data->shared.channel_count; i++) {
			data->buffer[i] = ADC_DRV_ReadFIFO(inst);
		}

		adc_context_on_sampling_done(&data->ctx, dev);
	}
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

	uint32_t functional_clock_hz;
	ret = clock_control_get_rate(config->clock_dev, config->clock_subsys,
					     &functional_clock_hz);
	if (ret < 0 || functional_clock_hz == 0U) {
		LOG_ERR("failed to read ADC functional clock: %d", ret);
		return ret < 0 ? ret : -EINVAL;
	}
	data->shared.adc_clock_hz = functional_clock_hz /
		(config->adc_clk_div + 1U);
	ret = adc_ytm32_validate_timing(data->shared.adc_clock_hz,
					YTM32_ADC_DEFAULT_SAMPLE_TIME,
					config->adc_start_time);
	if (ret < 0) {
		LOG_ERR("ADC timing violates YTM32B1MD1 DS v1.9: FADC=%u Hz ret=%d",
			data->shared.adc_clock_hz, ret);
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

#define YTM32_ADC_SEQUENCE_ORDER_ITEM(inst, idx) \
	COND_CODE_1(DT_INST_PROP_HAS_IDX(inst, ytmicro_sequence_order, idx), \
		(DT_INST_PROP_BY_IDX(inst, ytmicro_sequence_order, idx)), \
		(0U))

#define YTM32_ADC_TMU_DEV(inst) \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, ytmicro_tmu), \
		(DEVICE_DT_GET(DT_INST_PHANDLE(inst, ytmicro_tmu))), \
		(NULL))

#define YTM32_ADC_INIT(inst)							\
	PINCTRL_DT_INST_DEFINE(inst);						\
										\
	static void adc_ytm32_irq_config_##inst(void)				\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(inst),					\
			    DT_INST_IRQ(inst, priority),			\
			    adc_ytm32_isr,					\
			    DEVICE_DT_INST_GET(inst), 0);			\
		irq_enable(DT_INST_IRQN(inst));				\
	}									\
										\
	static const struct adc_ytm32_config adc_ytm32_cfg_##inst = {		\
		.base         = DT_INST_REG_ADDR(inst),				\
		.instance     = 0U,						\
		.irq          = (IRQn_Type)DT_INST_IRQN(inst),			\
		.clock_dev    = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),	\
		.clock_subsys = (clock_control_subsys_t)			\
				DT_INST_CLOCKS_CELL(inst, id),			\
		.adc_clk_div  = DT_INST_PROP(inst,				\
				ytmicro_adc_clock_divider),			\
		.adc_start_time = DT_INST_PROP(inst,				\
				ytmicro_adc_start_time),			\
		.sequence_order = {							\
			YTM32_ADC_SEQUENCE_ORDER_ITEM(inst, 0),				\
			YTM32_ADC_SEQUENCE_ORDER_ITEM(inst, 1),				\
			YTM32_ADC_SEQUENCE_ORDER_ITEM(inst, 2),				\
			YTM32_ADC_SEQUENCE_ORDER_ITEM(inst, 3),				\
			YTM32_ADC_SEQUENCE_ORDER_ITEM(inst, 4),				\
			YTM32_ADC_SEQUENCE_ORDER_ITEM(inst, 5),				\
			YTM32_ADC_SEQUENCE_ORDER_ITEM(inst, 6),				\
			YTM32_ADC_SEQUENCE_ORDER_ITEM(inst, 7),				\
		},							\
		.sequence_order_count = DT_INST_PROP_LEN_OR(inst,			\
				ytmicro_sequence_order, 0),					\
		.pincfg       = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),		\
		.irq_config_func = adc_ytm32_irq_config_##inst,			\
		.dma_dev      = YTM32_ADC_DMA_DEV(inst),			\
		.dma_channel  = YTM32_ADC_DMA_CH(inst),			\
		.dma_slot     = YTM32_ADC_DMA_SLOT(inst),			\
		.hw_trig_src  = YTM32_ADC_HW_TRIG(inst),			\
		.tmu_dev      = YTM32_ADC_TMU_DEV(inst),			\
		.cim_trig_sel = DT_INST_PROP_OR(inst,				\
				ytmicro_cim_trigger_select,			\
				YTM32B1MD1_CIM_ADC0_TRIG_SEL_TMU_ADCCLK),	\
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
