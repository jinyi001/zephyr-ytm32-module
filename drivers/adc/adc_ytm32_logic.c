/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "adc_ytm32_logic.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>

int adc_ytm32_sequence_expand(uint32_t channels_mask,
				      const uint8_t *order,
				      uint8_t order_count,
				      uint8_t max_slots,
				      uint8_t *sequence,
				      const uint8_t *sample_times,
				      uint8_t sample_time_count,
				      uint8_t *channel_count_out,
				      uint8_t *max_sample_time_out)
{
	uint32_t seen = 0U;
	uint8_t count = 0U;
	uint8_t max_sample_time = 0U;

	if (channels_mask == 0U || sequence == NULL || sample_times == NULL ||
	    sample_time_count == 0U || max_slots == 0U ||
	    channel_count_out == NULL || max_sample_time_out == NULL) {
		return -EINVAL;
	}

	if (order_count > max_slots) {
		return -EINVAL;
	}

	if (order_count == 0U) {
		for (uint8_t channel = 0U;
		     channel < YTM32_ADC_CHANNEL_MASK_BITS;
		     channel++) {
			if ((channels_mask & YTM32_ADC_CHANNEL_BIT(channel)) == 0U) {
				continue;
			}
			if (count >= max_slots || channel >= sample_time_count) {
				return -EINVAL;
			}

			sequence[count++] = channel;
			if (sample_times[channel] > max_sample_time) {
				max_sample_time = sample_times[channel];
			}
		}
	} else {
		for (uint8_t index = 0U; index < order_count; index++) {
			uint8_t channel = order[index];
			uint32_t channel_bit;

			if (channel >= YTM32_ADC_CHANNEL_MASK_BITS ||
			    channel >= sample_time_count) {
				return -EINVAL;
			}
			channel_bit = YTM32_ADC_CHANNEL_BIT(channel);
			if ((channels_mask & channel_bit) == 0U ||
			    (seen & channel_bit) != 0U) {
				return -EINVAL;
			}

			seen |= channel_bit;
			sequence[count++] = channel;
			if (sample_times[channel] > max_sample_time) {
				max_sample_time = sample_times[channel];
			}
		}

		if (seen != channels_mask) {
			return -EINVAL;
		}
	}

	*channel_count_out = count;
	*max_sample_time_out = max_sample_time;
	return 0;
}

int adc_ytm32_validate_timing(uint32_t fadc_hz, uint8_t sample_time,
				      uint32_t startup_time)
{
	uint64_t sample_ticks;
	uint64_t startup_ticks;

	if (fadc_hz < 4000000U || fadc_hz > 32000000U) {
		return -ERANGE;
	}

	sample_ticks = (uint64_t)sample_time + 2U;
	if (sample_ticks * 1000000000ULL < 100ULL * fadc_hz ||
	    sample_ticks * 1000000000ULL > 1000ULL * fadc_hz) {
		return -ERANGE;
	}

	startup_ticks = (uint64_t)startup_time + 1U;
	if (startup_ticks * 1000000000ULL < 2000ULL * fadc_hz) {
		return -ERANGE;
	}

	return 0;
}

uint32_t adc_ytm32_ticks_to_ns(uint32_t ticks, uint32_t fadc_hz)
{
	if (fadc_hz == 0U) {
		return 0U;
	}

	return (uint32_t)(((uint64_t)ticks * 1000000000ULL + fadc_hz - 1U) /
			   fadc_hz);
}
