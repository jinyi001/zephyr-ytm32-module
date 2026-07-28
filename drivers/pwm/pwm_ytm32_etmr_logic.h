/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_PWM_PWM_YTM32_ETMR_LOGIC_H_
#define ZEPHYR_DRIVERS_PWM_PWM_YTM32_ETMR_LOGIC_H_

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PWM_YTM32_ETMR_PHASE_COUNT 3U
#define PWM_YTM32_ETMR_CHANNEL_COUNT 8U
#define PWM_YTM32_ETMR_FAULT_COUNT 4U
#define PWM_YTM32_ETMR_MAX_DUTY_Q15 0x8000U

struct pwm_ytm32_etmr_phase_map {
	uint8_t phase_channel[PWM_YTM32_ETMR_PHASE_COUNT];
	uint8_t channel_mask;
};

/*
 * Validate the three even channels used by the phase fast path.  The
 * complementary-pair mask contains even channel bits only; each selected
 * phase owns that channel and its following odd channel.
 */
static inline int pwm_ytm32_etmr_phase_map_validate(
	const uint8_t phase_channel[PWM_YTM32_ETMR_PHASE_COUNT],
	uint8_t complementary_mask,
	struct pwm_ytm32_etmr_phase_map *map)
{
	if (phase_channel == NULL || map == NULL) {
		return -EINVAL;
	}

	map->channel_mask = 0U;
	for (size_t i = 0U; i < PWM_YTM32_ETMR_PHASE_COUNT; i++) {
		uint8_t channel = phase_channel[i];

		if (channel >= PWM_YTM32_ETMR_CHANNEL_COUNT ||
		    (channel & 1U) != 0U ||
		    (complementary_mask & (uint8_t)(1U << channel)) == 0U) {
			return -EINVAL;
		}

		for (size_t previous = 0U; previous < i; previous++) {
			if (phase_channel[previous] == channel) {
				return -EINVAL;
			}
		}

		map->phase_channel[i] = channel;
		map->channel_mask |= (uint8_t)(3U << channel);
	}

	return 0;
}

/* Convert complementary-pair even bits to all physical output bits. */
static inline uint8_t pwm_ytm32_etmr_complementary_channel_mask(
	uint8_t complementary_mask)
{
	return (uint8_t)((complementary_mask |
			  (uint8_t)(complementary_mask << 1U)) & 0xFFU);
}

/*
 * Convert raw digital fault-input levels to active fault inputs.  A raw bit
 * of one means the pin is high; active-low inputs therefore assert when the
 * corresponding raw bit is zero.  Keeping this conversion pure makes the
 * polarity contract testable without touching eTMR registers.
 */
static inline uint8_t pwm_ytm32_etmr_fault_active_mask(
	uint8_t raw_input_mask, uint8_t configured_mask, uint8_t active_low_mask)
{
	uint8_t active_high = (uint8_t)(raw_input_mask & (uint8_t)~active_low_mask);
	uint8_t active_low = (uint8_t)((uint8_t)~raw_input_mask & active_low_mask);

	return (uint8_t)((active_high | active_low) & configured_mask & 0x0FU);
}

/*
 * Calculate center-aligned VAL0/VAL1 edges for the MD1 edge convention.
 * The explicit endpoints are intentional: a 0% duty cycle is not a safe
 * output state for an inverted complementary channel.
 */
static inline void pwm_ytm32_etmr_center_edges(uint32_t period,
						 uint16_t duty_q15,
						 uint32_t *val0,
						 uint32_t *val1)
{
	uint32_t first;
	uint32_t second;

	if (val0 == NULL || val1 == NULL || period == 0U) {
		return;
	}

	if (duty_q15 == 0U) {
		*val0 = 0U;
		*val1 = 0U;
		return;
	}

	if (duty_q15 >= PWM_YTM32_ETMR_MAX_DUTY_Q15) {
		*val0 = 0U;
		*val1 = period - 1U;
		return;
	}

	first = PWM_YTM32_ETMR_MAX_DUTY_Q15 / 2U -
		(uint32_t)(duty_q15 >> 1U);
	second = PWM_YTM32_ETMR_MAX_DUTY_Q15 / 2U +
		(uint32_t)(duty_q15 >> 1U);
	*val0 = (uint32_t)(((uint64_t)period * first) >> 15U) - 1U;
	*val1 = (uint32_t)(((uint64_t)period * second) >> 15U) - 1U;
}

#endif /* ZEPHYR_DRIVERS_PWM_PWM_YTM32_ETMR_LOGIC_H_ */
