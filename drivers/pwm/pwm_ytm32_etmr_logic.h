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

struct pwm_ytm32_etmr_output_mask {
	uint8_t enable;
	uint16_t value;
};

/*
 * Derive the complementary-pair mask from the three phase channels.  The
 * phase list is the single source of truth: each even phase channel owns
 * that channel and its following odd channel.  Return zero for an invalid
 * list so callers cannot accidentally initialize a partial map.
 */
static inline uint8_t pwm_ytm32_etmr_phase_complementary_mask(
	const uint8_t phase_channel[PWM_YTM32_ETMR_PHASE_COUNT])
{
	uint8_t mask = 0U;

	if (phase_channel == NULL) {
		return 0U;
	}

	for (size_t i = 0U; i < PWM_YTM32_ETMR_PHASE_COUNT; i++) {
		uint8_t channel = phase_channel[i];
		uint8_t bit;

		if (channel >= PWM_YTM32_ETMR_CHANNEL_COUNT ||
		    (channel & 1U) != 0U) {
			return 0U;
		}

		bit = (uint8_t)(1U << channel);
		if ((mask & bit) != 0U) {
			return 0U;
		}

		mask |= bit;
	}

	return mask;
}

/* Validate the three even channels used by the phase fast path. */
static inline int pwm_ytm32_etmr_phase_map_validate(
	const uint8_t phase_channel[PWM_YTM32_ETMR_PHASE_COUNT],
	struct pwm_ytm32_etmr_phase_map *map)
{
	uint8_t complementary_mask;

	if (phase_channel == NULL || map == NULL) {
		return -EINVAL;
	}

	complementary_mask =
		pwm_ytm32_etmr_phase_complementary_mask(phase_channel);
	if (complementary_mask == 0U) {
		return -EINVAL;
	}

	map->channel_mask = 0U;
	for (size_t i = 0U; i < PWM_YTM32_ETMR_PHASE_COUNT; i++) {
		uint8_t channel = phase_channel[i];

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
 * Apply the E503001 endpoint workaround for one complementary pair.
 *
 * CHMASK has one enable bit and one two-bit forced value per physical
 * channel.  At 0% the even/high-side output must stay low while its
 * complementary odd/low-side output stays high; 100% is the inverse.
 * Interior duties use the normal compare outputs.  Masking both outputs at
 * an endpoint also avoids relying on a VAL0/VAL1 value equal to INIT, which
 * is affected by E503012 during the initial counter cycle.
 */
static inline void pwm_ytm32_etmr_endpoint_mask_update(
	struct pwm_ytm32_etmr_output_mask *mask, uint8_t even_channel,
	uint16_t duty_q15)
{
	uint8_t pair_enable;
	uint16_t pair_value_mask;

	if (mask == NULL || even_channel >= PWM_YTM32_ETMR_CHANNEL_COUNT ||
	    (even_channel & 1U) != 0U) {
		return;
	}

	pair_enable = (uint8_t)(3U << even_channel);
	pair_value_mask = (uint16_t)(0xFU << (2U * even_channel));
	mask->enable &= (uint8_t)~pair_enable;
	mask->value &= (uint16_t)~pair_value_mask;

	if (duty_q15 == 0U) {
		mask->enable |= pair_enable;
		mask->value |= (uint16_t)(1U << (2U * (even_channel + 1U)));
	} else if (duty_q15 >= PWM_YTM32_ETMR_MAX_DUTY_Q15) {
		mask->enable |= pair_enable;
		mask->value |= (uint16_t)(1U << (2U * even_channel));
	}
}

static inline uint32_t pwm_ytm32_etmr_output_mask_pack(
	const struct pwm_ytm32_etmr_output_mask *mask)
{
	if (mask == NULL) {
		return 0U;
	}

	return (uint32_t)mask->enable | ((uint32_t)mask->value << 16U);
}

static inline struct pwm_ytm32_etmr_output_mask
pwm_ytm32_etmr_output_mask_unpack(uint32_t packed)
{
	return (struct pwm_ytm32_etmr_output_mask) {
		.enable = (uint8_t)packed,
		.value = (uint16_t)(packed >> 16U),
	};
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
 * eTMR_DRV_GetFaultInputStatus() returns IOSTS.Fx.  The eTMR hardware has
 * already applied FAULT[FxPOL], so a one means that the input is currently
 * fault-active and a zero means that it is idle.  Do not apply the board
 * polarity a second time here.
 */
static inline uint8_t pwm_ytm32_etmr_fault_status_active_mask(
	uint8_t status_mask, uint8_t configured_mask)
{
	return (uint8_t)(status_mask & configured_mask & 0x0FU);
}

/*
 * Calculate center-aligned VAL0/VAL1 edges for the MD1 edge convention.
 * Endpoint edge values are retained as shadow-state placeholders; physical
 * endpoint levels are generated with CHMASK by the E503001 workaround above.
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
