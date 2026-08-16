/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef YTM32_ADC_LOGIC_H_
#define YTM32_ADC_LOGIC_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/adc/adc_ytm32.h>

#ifdef __cplusplus
extern "C" {
#endif

/* YTM32B1MD1 exposes selectors 0..37.  Zephyr's standard adc_sequence mask
 * remains 32-bit, while the YTM32 DMA extension also supports selectors
 * 32..37 (temperature/reference/internal rails) through a 64-bit mask. */
#define YTM32_ADC_CHANNEL_MASK_BITS 64U
#define YTM32_ADC_CHANNEL_BIT(channel) (UINT64_C(1) << (channel))

/**
 * @brief Expand an ADC channel mask into the hardware sequence order.
 *
 * With an empty @p order, selected channels are emitted in ascending channel
 * number order for backward compatibility.  With an explicit order, the
 * array must contain every selected channel at least once and may repeat a
 * selected channel, for example to implement ADC E600001's sacrificial first
 * conversion. A partial order or an unselected channel is rejected rather
 * than silently falling back.
 *
 * This function is deliberately independent of the MCU HAL so it can be
 * tested on native_sim and reused by both the interrupt and DMA paths.
 */
int adc_ytm32_sequence_expand(uint64_t channels_mask,
				      const uint8_t *order,
				      uint8_t order_count,
				      uint8_t max_slots,
				      uint8_t *sequence,
				      const uint8_t *sample_times,
				      uint8_t sample_time_count,
				      uint8_t *channel_count_out,
				      uint8_t *max_sample_time_out);

/**
 * @brief Validate the ADC timing limits from YTM32B1MD1 DS v1.9.
 *
 * FADC is 4--32 MHz, TSAMPLE is 100--1000 ns, and startup time is at least
 * 2 us.  Tick comparisons use integer cross multiplication so boundary values
 * are not rounded down before validation.
 */
int adc_ytm32_validate_timing(uint32_t fadc_hz, uint8_t sample_time,
				      uint32_t startup_time);

/**
 * @brief Validate DMA sequence mode and calculate trigger cadence.
 *
 * STEP is meaningful only for an external hardware trigger.  FULL consumes
 * one trigger per complete sequence, while STEP consumes one trigger per slot.
 */
int adc_ytm32_dma_trigger_plan(
		enum adc_ytm32_dma_sequence_mode sequence_mode,
		bool hardware_trigger, uint8_t channel_count, uint16_t depth,
		uint32_t *triggers_per_callback);

/** Return a rounded-up duration in nanoseconds for @p ticks at @p fadc_hz. */
uint32_t adc_ytm32_ticks_to_ns(uint32_t ticks, uint32_t fadc_hz);

#ifdef __cplusplus
}
#endif

#endif /* YTM32_ADC_LOGIC_H_ */
