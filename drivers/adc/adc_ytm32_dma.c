/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr ADC driver for YTMicro YTM32 — Phase 2: hardware-triggered DMA
 * continuous sampling.  This is the optional extension behind the public
 * <zephyr/drivers/adc/adc_ytm32.h> API (adc_ytm32_dma_start / _stop / _resume);
 * the standard interrupt-driven ADC API lives in adc_ytm32.c.  The two share
 * the per-instance config and the adc_ytm32_shared state via adc_ytm32_priv.h.
 *
 * Hardware signal chain:
 *   ETMR0 OTRIG (INITTEN=1, counter-bottom pulse)
 *     → TMU route: eTMR0_INIT_TRIG (22) → ADC0_EXT_TRIG (12)
 *     → ADC0 hardware trigger → sequence conversion → FIFO
 *     → DMA ch8 (DMA_REQ_ADC0=42) → SRAM buffer
 *     → DMA complete ISR → user callback
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/ytm32_soc_clock.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "device_registers.h"
#include "adc_driver.h"
#include "../dma/ytm32_dma_hal.h"
#include <zephyr/drivers/misc/ytm32_tmu.h>
#include <zephyr/dt-bindings/tmu/ytm32b1md1-tmu.h>
#include <zephyr/dt-bindings/clock/ytmicro,ytm32b1md1-clock.h>

#include "adc_ytm32_priv.h"

LOG_MODULE_DECLARE(adc_ytm32, CONFIG_ADC_LOG_LEVEL);

/* ──────────────────────── DMA notify half (shared ISR) ──────────────────── */

void adc_ytm32_dma_notify_isr(const struct device *dev)
{
	struct adc_ytm32_shared *st = adc_ytm32_shared(dev);

	if (st->dma_error != 0) {
		LOG_ERR("DMA error %d", st->dma_error);
		st->dma_error = 0;
		st->dma_active = false;
		return;
	}

	if (st->dma_cfg.cb != NULL) {
		st->dma_cfg.cb(dev, st->dma_cfg.buf,
			       st->channel_count,
			       st->dma_cfg.depth,
			       st->dma_cfg.user_data);
	}
}

/* ──────────────────────── TMU helper ──────────────────────── */

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

/* ──────────────────────── DMA callback ──────────────────────── */

static void adc_ytm32_dma_cb(void *user_data, int hal_status)
{
	const struct device *dev = user_data;
	struct adc_ytm32_shared *st = adc_ytm32_shared(dev);
	const struct adc_ytm32_config *config = dev->config;

	if (!st->dma_active) {
		return;
	}

	if (hal_status != 0) {
		/* No logging from a possible zero-latency top-half; report it from the
		 * software-pended ADC notification ISR instead.
		 */
		st->dma_error = hal_status;
		ytm32_dma_hal_stop(config->dma_channel);
		st->dma_active = false;
		ADC_DRV_Stop(config->instance);
		NVIC_SetPendingIRQ(config->irq);
		return;
	}

	/* Stop DMA immediately after the batch, before any user callback runs.  In
	 * continuous mode the ADC keeps generating DMA requests; without this stop the
	 * DMA can auto-restart and re-enter the ISR before the waiting thread resumes.
	 */
	ytm32_dma_hal_stop(config->dma_channel);

	/* Call the zero-latency-safe callback from the DMA top-half.  This callback
	 * must not use Zephyr kernel APIs; it is for register reads/writes or other
	 * bounded latency-sensitive work only.
	 */
	if (st->dma_cfg.zl_cb != NULL) {
		st->dma_cfg.zl_cb(dev, st->dma_cfg.buf,
				  st->channel_count,
				  st->dma_cfg.depth,
				  st->dma_cfg.user_data);
	}

	/* Defer the normal callback to a regular-priority software IRQ so callers may
	 * use ISR-safe kernel APIs such as k_sem_give(), even when the DMA top-half is
	 * later connected as a zero-latency direct interrupt.
	 */
	if (st->dma_cfg.cb != NULL) {
		NVIC_SetPendingIRQ(config->irq);
	}
}

/* ──────────────────────── internal helpers ─────────────────── */

/*
 * MD1 workaround: clear stale ADSTART state via ADSTOP→ADSTART.
 * Without this, the hardware-trigger accept gate is not reliably armed after
 * ADC_DRV_Enable() + ADRDY.  Verified: ADSTOP→ADSTART immediately enables
 * trigger acceptance from EXT_TRIG(23) and INIT_TRIG(22).
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
	 * Verified: ADSTOP→ADSTART immediately enables trigger acceptance.
	 * Failure mode: k_sem_take timeout in stage5.
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

/* ──────────────────────── public API ──────────────────────── */

int adc_ytm32_dma_start(const struct device *dev,
			const struct adc_ytm32_dma_config *cfg)
{
	struct adc_ytm32_shared *st = adc_ytm32_shared(dev);
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
	    (cfg->cb == NULL && cfg->zl_cb == NULL)) {
		return -EINVAL;
	}
	if (st->dma_active) {
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
	st->dma_cfg     = *cfg;
	st->dma_error   = 0;
	st->channel_count = ch_count;

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
	uintptr_t fifo_addr = (uintptr_t)&((ADC_Type *)config->base)->FIFO;

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
	adc_ytm32_channels_to_sequence(cfg->channels, conv.sequenceConfig.channels,
				       st->sample_time, &max_smp);

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
	conv.resolution   = adc_ytm32_bits_to_resolution(cfg->resolution);
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

	st->dma_active = true;
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
			st->dma_active = false;
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
	struct adc_ytm32_shared *st = adc_ytm32_shared(dev);
	const struct adc_ytm32_config *config = dev->config;

	if (!st->dma_active) {
		return 0;
	}

	st->dma_active = false;
	ADC_DRV_Stop(config->instance);
	ADC_DRV_Disable(config->instance);
	ytm32_dma_hal_stop(config->dma_channel);
	ytm32_dma_hal_channel_release(config->dma_channel);

	return 0;
}

int adc_ytm32_dma_resume(const struct device *dev)
{
	struct adc_ytm32_shared *st = adc_ytm32_shared(dev);
	const struct adc_ytm32_config *config = dev->config;

	if (!st->dma_active) {
		return -EINVAL;
	}

	/* Stop ADC to prevent FIFO overflow while DMA is being re-armed. */
	ADC_DRV_Stop(config->instance);

	/* Reconfigure DMA for one more batch of the same shape. */
	uintptr_t fifo_addr = (uintptr_t)&((ADC_Type *)config->base)->FIFO;

	int ret = ytm32_dma_hal_channel_config_loop(
		config->dma_channel,
		config->dma_slot,
		fifo_addr,
		(uintptr_t)st->dma_cfg.buf,
		sizeof(uint16_t),
		st->channel_count,
		st->dma_cfg.depth,
		adc_ytm32_dma_cb,
		(void *)dev);
	if (ret < 0) {
		LOG_ERR("DMA reconfig failed: %d", ret);
		return ret;
	}

	ytm32_dma_hal_start(config->dma_channel);

	if (st->dma_cfg.trigger == ADC_YTM32_DMA_TRIGGER_HARDWARE) {
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
