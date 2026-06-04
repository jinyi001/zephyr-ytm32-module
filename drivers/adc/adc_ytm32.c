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
#include "../dma/ytm32_dma_hal.h"
#include <zephyr/drivers/adc/adc_ytm32.h>
#include <zephyr/drivers/misc/ytm32_tmu.h>
#include <zephyr/dt-bindings/tmu/ytm32b1md1-tmu.h>
#include <zephyr/dt-bindings/clock/ytmicro,ytm32b1md1-clock.h>
#include <zephyr/dt-bindings/adc/ytm32b1md1-adc.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

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

/* Maximum external channel count across all YTM32 variants */
#define YTM32_ADC_MAX_CHANS 38U

/* ADC FIFO register offset within ADC peripheral (from the struct layout) */
#define YTM32_ADC_FIFO_OFFSET 0x4CU

/* YTM32_TMU_TARGET_ADC0_EXT_TRIG comes from dt-bindings/tmu/ytm32b1md1-tmu.h.
 * CIM trigger-select values come from dt-bindings/adc/ytm32b1md1-adc.h.
 */

/* Sentinel meaning "no hardware trigger configured" */
#define YTM32_ADC_NO_HW_TRIG UINT32_MAX

/* ──────────────────────── config / data structs ──────────────────────── */

struct adc_ytm32_config {
	uint32_t                         base;
	uint8_t                          instance;
	const struct device             *clock_dev;
	clock_control_subsys_t           clock_subsys;
	uint32_t                         adc_clk_div; /* CFG1.PRS — ADC internal divider */
	const struct pinctrl_dev_config *pincfg;
	void (*irq_config_func)(void);
	/* Phase 2: DMA and hardware trigger (optional) */
	const struct device             *dma_dev;     /* NULL if DMA not wired */
	uint8_t                          dma_channel;
	uint8_t                          dma_slot;    /* DMAMUX request source */
	uint32_t                         hw_trig_src; /* TMU trigger source enum value */
	const struct device             *tmu_dev;    /* NULL if no TMU phandle */
	uint32_t                         cim_trig_sel; /* CIM ADC0_TRIG_SEL field value */
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
	/* ADC module internal clock divider (CFG1.PRS = ytmicro,adc-clock-divider).
	 * Separate from the IPC clock tree divider (ytmicro,functional-clock-divider).
	 * Encoding: 0=÷1, 1=÷2, 2=÷4 (default), 3=÷8.
	 * With IPC = FIRC/6 = 16 MHz and default PRS=2: ADC core = 4 MHz (within 2–32 MHz).
	 */
	conv.clockDivider = (adc_clk_divide_t)config->adc_clk_div;
	conv.resolution   = bits_to_resolution(ctx->sequence.resolution);
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

		for (uint8_t i = 0U; i < data->channel_count; i++) {
			data->buffer[i] = ADC_DRV_ReadFIFO(inst);
		}

		adc_context_on_sampling_done(&data->ctx, dev);
	}
}

/* ──────────────────────── Phase 2: TMU helper ──────────────────────── */

static int adc_ytm32_tmu_route(const struct device *dev, uint32_t trig_src)
{
	const struct adc_ytm32_config *config = dev->config;

	if (config->tmu_dev == NULL) {
		LOG_ERR("ytmicro,tmu phandle not set in DTS");
		return -ENOTSUP;
	}
	if (!device_is_ready(config->tmu_dev)) {
		LOG_ERR("TMU device not ready");
		return -ENODEV;
	}

	/* Route the configured trigger source to the ADC0 external-trigger input.
	 * The TMU driver owns the TMU clock, so no manual clock poke here.
	 */
	return ytm32_tmu_route(config->tmu_dev, trig_src,
				       YTM32_TMU_TARGET_ADC0_EXT_TRIG);
}

#if !defined(CIM_CTRL_ADC0_TRIG_SEL_MASK) || !defined(CIM_CTRL_ADC0_TRIG_SEL_SHIFT)
#error "CIM_CTRL_ADC0_TRIG_SEL_MASK/_SHIFT not defined by HAL — hardware trigger requires CIM support"
#endif

static void adc_ytm32_select_hw_trigger_input(const struct device *dev)
{
	const struct adc_ytm32_config *config = dev->config;

	(void)clock_control_on(config->clock_dev,
			       (clock_control_subsys_t)YTM32_CLOCK_CIM);

	if (config->instance == 0U) {
		uint32_t ctrl = CIM->CTRL;

		ctrl &= ~CIM_CTRL_ADC0_TRIG_SEL_MASK;
		ctrl |= CIM_CTRL_ADC0_TRIG_SEL(config->cim_trig_sel);
		CIM->CTRL = ctrl;
	}
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

	if (data->dma_cfg.cb != NULL) {
		data->dma_cfg.cb(dev, data->dma_cfg.buf,
				 data->channel_count,
				 data->dma_cfg.depth,
				 data->dma_cfg.user_data);
	}

	/* Stop DMA immediately after the batch.  In continuous mode the ADC
	 * keeps generating DMA requests; without an explicit stop the DMA
	 * auto-restarts and the ISR fires in a tight loop (starving PendSV
	 * and the thread waiting on the semaphore).  The caller must call
	 * adc_ytm32_dma_resume() from thread context for the next batch.
	 */
	ytm32_dma_hal_stop(config->dma_channel);
}

/* ──────────────────────── Phase 2: internal helpers ─────────────────── */

/*
 * MD1 workaround: clear stale ADSTART state via ADSTOP→ADSTART.
 * Without this, the hardware-trigger accept gate is not reliably armed after
 * ADC_DRV_Enable() + ADRDY.  Verified: ADSTOP→ADSTART immediately enables
 * trigger acceptance from INIT_TRIG(22).
 * Failure mode without this: k_sem_take timeout in trigger_chain stage5.
 */
static int adc_ytm32_arm_hw_trigger(const struct device *dev)
{
	const struct adc_ytm32_config *config = dev->config;
	ADC_Type *base = (ADC_Type *)config->base;

	/* MD1 workaround: clear stale ADSTART state via ADSTOP→ADSTART.
	 * Without this, the hardware-trigger accept gate is not reliably armed
	 * after ADC_DRV_Enable() + ADRDY.  Use a bare counter loop — k_busy_wait
	 * was found to interfere with the trigger acceptance timing on MD1.
	 * Verified: ADSTOP→ADSTART immediately enables trigger acceptance from
	 * INIT_TRIG(22).  Failure mode: k_sem_take timeout in stage5.
	 */
	base->CTRL |= ADC_CTRL_ADSTOP_MASK;
	for (uint32_t i = 0; i < 10000U; i++) {
		if ((base->CTRL & ADC_CTRL_ADSTOP_MASK) == 0U) {
			break;
		}
	}
	base->CTRL |= ADC_CTRL_ADSTART_MASK;
	return 0;
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

	const bool hw_trig = (cfg->trigger == ADC_YTM32_DMA_TRIGGER_HARDWARE);

	if (config->dma_dev == NULL) {
		LOG_ERR("DMA not configured in DTS for this ADC instance");
		return -ENOTSUP;
	}
	if (hw_trig && config->hw_trig_src == YTM32_ADC_NO_HW_TRIG) {
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

	/* 1. (hardware mode) Route the eTMR trigger via TMU to ADC0_EXT_TRIG and
	 * select the TMU output as ADC0's top-level hardware-trigger input.
	 * Software mode self-clocks the sequence, so no trigger fabric is needed.
	 */
	if (hw_trig) {
		ret = adc_ytm32_tmu_route(dev, config->hw_trig_src);
		if (ret < 0) {
			return ret;
		}
		adc_ytm32_select_hw_trigger_input(dev);
	}

	/* 2. Configure DMA: each ADC DMA request drains one full channel sequence;
	 * after 'depth' sequences the user buffer is full.
	 */
	uintptr_t fifo_addr = (uintptr_t)(config->base + YTM32_ADC_FIFO_OFFSET);

	ret = ytm32_dma_hal_channel_config_loop(
		config->dma_channel,
		config->dma_slot,
		fifo_addr,
		(uintptr_t)cfg->buf,
		sizeof(uint16_t),
		ch_count,
		cfg->depth,
		adc_ytm32_dma_cb,
		(void *)dev);
	if (ret < 0) {
		LOG_ERR("DMA loop config failed: %d", ret);
		return ret;
	}

	/* 3. Configure ADC: DMA enabled, no interrupts.
	 * - hardware mode: external trigger drives one sequence per eTMR period
	 *   (LOOP = one sequence per trigger).
	 * - software mode: free-running continuous conversion self-clocks the
	 *   sequence back-to-back.
	 */
	ADC_DRV_InitConverterStruct(&conv);
	channels_to_sequence(cfg->channels, conv.sequenceConfig.channels,
			     data->sample_time, &max_smp);

	conv.sequenceConfig.totalChannels     = ch_count;
	conv.sequenceConfig.sequenceMode      = hw_trig ? ADC_CONV_LOOP
							: ADC_CONV_CONTINUOUS;
	conv.sequenceConfig.sequenceIntEnable = false;
	conv.sequenceConfig.ovrunIntEnable    = false;
	conv.sampleTime   = max_smp;
	/* ADC module internal clock divider (CFG1.PRS = ytmicro,adc-clock-divider).
	 * Separate from the IPC clock tree divider (ytmicro,functional-clock-divider).
	 * Encoding: 0=÷1, 1=÷2, 2=÷4 (default), 3=÷8.
	 * With IPC = FIRC/6 = 16 MHz and default PRS=2: ADC core = 4 MHz (within 2–32 MHz).
	 */
	conv.clockDivider = (adc_clk_divide_t)config->adc_clk_div;
	conv.resolution   = bits_to_resolution(cfg->resolution);
	conv.align        = ADC_ALIGN_RIGHT;
	conv.trigger      = hw_trig ? ADC_TRIGGER_HARDWARE : ADC_TRIGGER_SOFTWARE;
	conv.dmaEnable    = true;
	/* DMA request fires once one full channel sequence is in the FIFO. */
	conv.dmaWaterMark = ch_count - 1U;

	ADC_DRV_ConfigConverter(config->instance, &conv);

	/* 4. Start DMA before arming the ADC (order matters: DMA must be ready
	 * before the first conversion can fill the FIFO).  Software-triggered
	 * continuous mode can use ADC_DRV_Start() directly.  Hardware-triggered
	 * mode needs a stricter MD1 arm sequence below: enable, wait ADRDY, clear
	 * stale start state, then assert ADSTART so TMU/eTMR trigger edges are
	 * accepted.
	 */
	ret = ytm32_dma_hal_start(config->dma_channel);
	if (ret < 0) {
		return ret;
	}

	data->dma_active = true;
	if (hw_trig) {
		/* On MD1 the hardware-trigger gate only arms reliably when ADSTART
		 * is asserted after ADRDY.  Enable first, poll ready, then arm.
		 */
		ADC_DRV_Enable(config->instance);
		for (uint32_t i = 0; i < 10000U; i++) {
			if (ADC_DRV_GetReadyFlag(config->instance)) {
				break;
			}
		}
		ret = adc_ytm32_arm_hw_trigger(dev);
		if (ret < 0) {
			data->dma_active = false;
			ytm32_dma_hal_stop(config->dma_channel);
			ytm32_dma_hal_channel_release(config->dma_channel);
			return ret;
		}
	} else {
		ADC_DRV_Start(config->instance);
	}

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

int adc_ytm32_dma_resume(const struct device *dev)
{
	struct adc_ytm32_data *data = dev->data;
	const struct adc_ytm32_config *config = dev->config;

	if (!data->dma_active) {
		return -EINVAL;
	}

	/* Stop ADC to prevent FIFO overflow while DMA is being re-armed. */
	ADC_DRV_Stop(config->instance);

	/* Reconfigure DMA for one more batch of the same shape. */
	uintptr_t fifo_addr = (uintptr_t)(config->base + YTM32_ADC_FIFO_OFFSET);

	int ret = ytm32_dma_hal_channel_config_loop(
		config->dma_channel,
		config->dma_slot,
		fifo_addr,
		(uintptr_t)data->dma_cfg.buf,
		sizeof(uint16_t),
		data->channel_count,
		data->dma_cfg.depth,
		adc_ytm32_dma_cb,
		(void *)dev);
	if (ret < 0) {
		LOG_ERR("DMA reconfig failed: %d", ret);
		return ret;
	}

	ytm32_dma_hal_start(config->dma_channel);

	if (data->dma_cfg.trigger == ADC_YTM32_DMA_TRIGGER_HARDWARE) {
		/* Hardware trigger: re-arm the ADC trigger gate. */
		ADC_DRV_Enable(config->instance);
		for (uint32_t i = 0; i < 10000U; i++) {
			if (ADC_DRV_GetReadyFlag(config->instance)) {
				break;
			}
		}
		ret = adc_ytm32_arm_hw_trigger(dev);
		if (ret < 0) {
			return ret;
		}
	} else {
		ADC_DRV_Start(config->instance);
	}

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
		irq_enable(DT_INST_IRQN(inst));					\
	}									\
										\
	static const struct adc_ytm32_config adc_ytm32_cfg_##inst = {		\
		.base         = DT_INST_REG_ADDR(inst),				\
		.instance     = 0U,						\
		.clock_dev    = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),	\
		.clock_subsys = (clock_control_subsys_t)			\
				DT_INST_CLOCKS_CELL(inst, id),			\
		.adc_clk_div  = DT_INST_PROP(inst,				\
				ytmicro_adc_clock_divider),			\
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
