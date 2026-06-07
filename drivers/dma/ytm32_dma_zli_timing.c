/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/dma/ytm32_dma_zli_timing.h>

#if defined(CONFIG_DMA_YTM32_CH8_ZLI_TIMING)

volatile struct ytm32_dma_zli_timing_snapshot ytm32_dma_zli_timing_current;

void ytm32_dma_zli_timing_reset(uint8_t channel)
{
	static uint32_t seq;
	volatile struct ytm32_dma_zli_timing_snapshot *s = &ytm32_dma_zli_timing_current;

	s->seq = ++seq;
	s->channel = channel;
	s->valid = 0U;
	s->irq_entry = 0U;
	s->irq_exit = 0U;
	s->hal_irq_entry = 0U;
	s->hal_irq_exit = 0U;
	s->vendor_bridge_entry = 0U;
	s->vendor_bridge_exit = 0U;
	s->zephyr_dma_cb_entry = 0U;
	s->zephyr_dma_cb_exit = 0U;
	s->adc_dma_cb_entry = 0U;
	s->adc_dma_cb_exit = 0U;
	s->dma_stop_begin = 0U;
	s->dma_stop_end = 0U;
	s->zl_cb_begin = 0U;
	s->zl_cb_end = 0U;
	s->nvic_pend_begin = 0U;
	s->nvic_pend_end = 0U;
	s->notify_isr_entry = 0U;
	s->notify_cb_begin = 0U;
	s->notify_cb_end = 0U;
}

void ytm32_dma_zli_timing_snapshot(struct ytm32_dma_zli_timing_snapshot *out)
{
	volatile struct ytm32_dma_zli_timing_snapshot *s = &ytm32_dma_zli_timing_current;

	out->seq = s->seq;
	out->channel = s->channel;
	out->valid = s->valid;
	out->irq_entry = s->irq_entry;
	out->irq_exit = s->irq_exit;
	out->hal_irq_entry = s->hal_irq_entry;
	out->hal_irq_exit = s->hal_irq_exit;
	out->vendor_bridge_entry = s->vendor_bridge_entry;
	out->vendor_bridge_exit = s->vendor_bridge_exit;
	out->zephyr_dma_cb_entry = s->zephyr_dma_cb_entry;
	out->zephyr_dma_cb_exit = s->zephyr_dma_cb_exit;
	out->adc_dma_cb_entry = s->adc_dma_cb_entry;
	out->adc_dma_cb_exit = s->adc_dma_cb_exit;
	out->dma_stop_begin = s->dma_stop_begin;
	out->dma_stop_end = s->dma_stop_end;
	out->zl_cb_begin = s->zl_cb_begin;
	out->zl_cb_end = s->zl_cb_end;
	out->nvic_pend_begin = s->nvic_pend_begin;
	out->nvic_pend_end = s->nvic_pend_end;
	out->notify_isr_entry = s->notify_isr_entry;
	out->notify_cb_begin = s->notify_cb_begin;
	out->notify_cb_end = s->notify_cb_end;
}

#endif /* CONFIG_DMA_YTM32_CH8_ZLI_TIMING */
