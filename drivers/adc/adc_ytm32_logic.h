/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef YTM32_ADC_LOGIC_H_
#define YTM32_ADC_LOGIC_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The ADC channel selector is a 32-bit Zephyr channel mask. */
#define YTM32_ADC_CHANNEL_MASK_BITS 32U
#define YTM32_ADC_CHANNEL_BIT(channel) (UINT32_C(1) << (channel))

/**
 * @brief Expand an ADC channel mask into the hardware sequence order.
 *
 * With an empty @p order, selected channels are emitted in ascending channel
 * number order for backward compatibility.  With an explicit order, the
 * array must contain every selected channel exactly once; a partial or
 * duplicate order is rejected rather than silently falling back.
 *
 * This function is deliberately independent of the MCU HAL so it can be
 * tested on native_sim and reused by both the interrupt and DMA paths.
 */
int adc_ytm32_sequence_expand(uint32_t channels_mask,
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

/** Return a rounded-up duration in nanoseconds for @p ticks at @p fadc_hz. */
uint32_t adc_ytm32_ticks_to_ns(uint32_t ticks, uint32_t fadc_hz);

#ifdef __cplusplus
}
#endif

#endif /* YTM32_ADC_LOGIC_H_ */
