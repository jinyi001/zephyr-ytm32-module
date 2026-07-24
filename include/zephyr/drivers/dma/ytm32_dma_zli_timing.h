/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Debug-only timing probe for the YTM32 DMA channel 8 zero-latency path.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_YTM32_DMA_ZLI_TIMING_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_YTM32_DMA_ZLI_TIMING_H_

#include <stdint.h>

#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ytm32_dma_zli_timing_snapshot {
	uint32_t seq;
	uint8_t channel;
	uint8_t valid;

	uint32_t irq_entry;
	uint32_t irq_exit;
	uint32_t hal_irq_entry;
	uint32_t hal_irq_exit;
	uint32_t vendor_bridge_entry;
	uint32_t vendor_bridge_exit;
	uint32_t zephyr_dma_cb_entry;
	uint32_t zephyr_dma_cb_exit;
	uint32_t adc_dma_cb_entry;
	uint32_t adc_dma_cb_exit;
	uint32_t dma_stop_begin;
	uint32_t dma_stop_end;
	uint32_t zl_cb_begin;
	uint32_t zl_cb_end;
	uint32_t nvic_pend_begin;
	uint32_t nvic_pend_end;
	uint32_t notify_isr_entry;
	uint32_t notify_cb_begin;
	uint32_t notify_cb_end;
};

enum ytm32_dma_zli_timing_point {
	YTM32_DMA_ZLI_TIMING_IRQ_ENTRY,
	YTM32_DMA_ZLI_TIMING_IRQ_EXIT,
	YTM32_DMA_ZLI_TIMING_HAL_IRQ_ENTRY,
	YTM32_DMA_ZLI_TIMING_HAL_IRQ_EXIT,
	YTM32_DMA_ZLI_TIMING_VENDOR_BRIDGE_ENTRY,
	YTM32_DMA_ZLI_TIMING_VENDOR_BRIDGE_EXIT,
	YTM32_DMA_ZLI_TIMING_ZEPHYR_DMA_CB_ENTRY,
	YTM32_DMA_ZLI_TIMING_ZEPHYR_DMA_CB_EXIT,
	YTM32_DMA_ZLI_TIMING_ADC_DMA_CB_ENTRY,
	YTM32_DMA_ZLI_TIMING_ADC_DMA_CB_EXIT,
	YTM32_DMA_ZLI_TIMING_DMA_STOP_BEGIN,
	YTM32_DMA_ZLI_TIMING_DMA_STOP_END,
	YTM32_DMA_ZLI_TIMING_ZL_CB_BEGIN,
	YTM32_DMA_ZLI_TIMING_ZL_CB_END,
	YTM32_DMA_ZLI_TIMING_NVIC_PEND_BEGIN,
	YTM32_DMA_ZLI_TIMING_NVIC_PEND_END,
	YTM32_DMA_ZLI_TIMING_NOTIFY_ISR_ENTRY,
	YTM32_DMA_ZLI_TIMING_NOTIFY_CB_BEGIN,
	YTM32_DMA_ZLI_TIMING_NOTIFY_CB_END,
};

#if defined(CONFIG_DMA_YTM32_CH8_ZLI_TIMING)

#include <YTM32B1MD1.h>

extern volatile struct ytm32_dma_zli_timing_snapshot ytm32_dma_zli_timing_current;

static inline uint32_t ytm32_dma_zli_timing_cycles(void)
{
	return DWT->CYCCNT;
}

static inline void ytm32_dma_zli_timing_mark(enum ytm32_dma_zli_timing_point point)
{
	uint32_t now = ytm32_dma_zli_timing_cycles();
	volatile struct ytm32_dma_zli_timing_snapshot *s = &ytm32_dma_zli_timing_current;

	switch (point) {
	case YTM32_DMA_ZLI_TIMING_IRQ_ENTRY:
		s->irq_entry = now;
		break;
	case YTM32_DMA_ZLI_TIMING_IRQ_EXIT:
		s->irq_exit = now;
		s->valid = 1U;
		break;
	case YTM32_DMA_ZLI_TIMING_HAL_IRQ_ENTRY:
		s->hal_irq_entry = now;
		break;
	case YTM32_DMA_ZLI_TIMING_HAL_IRQ_EXIT:
		s->hal_irq_exit = now;
		break;
	case YTM32_DMA_ZLI_TIMING_VENDOR_BRIDGE_ENTRY:
		s->vendor_bridge_entry = now;
		break;
	case YTM32_DMA_ZLI_TIMING_VENDOR_BRIDGE_EXIT:
		s->vendor_bridge_exit = now;
		break;
	case YTM32_DMA_ZLI_TIMING_ZEPHYR_DMA_CB_ENTRY:
		s->zephyr_dma_cb_entry = now;
		break;
	case YTM32_DMA_ZLI_TIMING_ZEPHYR_DMA_CB_EXIT:
		s->zephyr_dma_cb_exit = now;
		break;
	case YTM32_DMA_ZLI_TIMING_ADC_DMA_CB_ENTRY:
		s->adc_dma_cb_entry = now;
		break;
	case YTM32_DMA_ZLI_TIMING_ADC_DMA_CB_EXIT:
		s->adc_dma_cb_exit = now;
		break;
	case YTM32_DMA_ZLI_TIMING_DMA_STOP_BEGIN:
		s->dma_stop_begin = now;
		break;
	case YTM32_DMA_ZLI_TIMING_DMA_STOP_END:
		s->dma_stop_end = now;
		break;
	case YTM32_DMA_ZLI_TIMING_ZL_CB_BEGIN:
		s->zl_cb_begin = now;
		break;
	case YTM32_DMA_ZLI_TIMING_ZL_CB_END:
		s->zl_cb_end = now;
		break;
	case YTM32_DMA_ZLI_TIMING_NVIC_PEND_BEGIN:
		s->nvic_pend_begin = now;
		break;
	case YTM32_DMA_ZLI_TIMING_NVIC_PEND_END:
		s->nvic_pend_end = now;
		break;
	case YTM32_DMA_ZLI_TIMING_NOTIFY_ISR_ENTRY:
		s->notify_isr_entry = now;
		break;
	case YTM32_DMA_ZLI_TIMING_NOTIFY_CB_BEGIN:
		s->notify_cb_begin = now;
		break;
	case YTM32_DMA_ZLI_TIMING_NOTIFY_CB_END:
		s->notify_cb_end = now;
		break;
	}
}

void ytm32_dma_zli_timing_reset(uint8_t channel);
void ytm32_dma_zli_timing_snapshot(struct ytm32_dma_zli_timing_snapshot *out);

#else

static inline uint32_t ytm32_dma_zli_timing_cycles(void)
{
	return 0U;
}

static inline void ytm32_dma_zli_timing_mark(enum ytm32_dma_zli_timing_point point)
{
	ARG_UNUSED(point);
}

static inline void ytm32_dma_zli_timing_reset(uint8_t channel)
{
	ARG_UNUSED(channel);
}

static inline void ytm32_dma_zli_timing_snapshot(struct ytm32_dma_zli_timing_snapshot *out)
{
	ARG_UNUSED(out);
}

#endif /* CONFIG_DMA_YTM32_CH8_ZLI_TIMING */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_YTM32_DMA_ZLI_TIMING_H_ */
