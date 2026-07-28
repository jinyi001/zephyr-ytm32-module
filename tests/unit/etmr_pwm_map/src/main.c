/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/ztest.h>

#include "pwm_ytm32_etmr_logic.h"

static void *etmr_pwm_map_setup(void)
{
	return NULL;
}

ZTEST_SUITE(etmr_pwm_map, NULL, etmr_pwm_map_setup, NULL, NULL, NULL);

ZTEST(etmr_pwm_map, test_v3_phase_map_and_output_mask)
{
	const uint8_t phase_channels[PWM_YTM32_ETMR_PHASE_COUNT] = {6U, 2U, 4U};
	struct pwm_ytm32_etmr_phase_map map;

	zassert_ok(pwm_ytm32_etmr_phase_map_validate(phase_channels, 0x54U,
							     &map),
			   "V3 phase map should be valid");
	zassert_equal(map.phase_channel[0], 6U, "phase A channel");
	zassert_equal(map.phase_channel[1], 2U, "phase B channel");
	zassert_equal(map.phase_channel[2], 4U, "phase C channel");
	zassert_equal(map.channel_mask, 0xFCU, "physical output mask");
	zassert_equal(pwm_ytm32_etmr_complementary_channel_mask(0x54U), 0xFCU,
		       "complementary mask expansion");
}

ZTEST(etmr_pwm_map, test_rejects_invalid_phase_maps)
{
	static const uint8_t odd_channel[PWM_YTM32_ETMR_PHASE_COUNT] = {7U, 2U,
								 4U};
	static const uint8_t out_of_range[PWM_YTM32_ETMR_PHASE_COUNT] = {8U, 2U,
									4U};
	static const uint8_t not_enabled[PWM_YTM32_ETMR_PHASE_COUNT] = {0U, 2U,
									4U};
	static const uint8_t duplicate[PWM_YTM32_ETMR_PHASE_COUNT] = {6U, 6U,
									4U};
	struct pwm_ytm32_etmr_phase_map map;

	zassert_equal(pwm_ytm32_etmr_phase_map_validate(odd_channel, 0x54U,
								&map), -EINVAL,
				      "odd channel must be rejected");
	zassert_equal(pwm_ytm32_etmr_phase_map_validate(out_of_range, 0x54U,
								&map), -EINVAL,
				      "out-of-range channel must be rejected");
	zassert_equal(pwm_ytm32_etmr_phase_map_validate(not_enabled, 0x54U,
								&map), -EINVAL,
				      "channel outside complementary mask must be rejected");
	zassert_equal(pwm_ytm32_etmr_phase_map_validate(duplicate, 0x54U,
								&map), -EINVAL,
				      "duplicate channel must be rejected");
}

ZTEST(etmr_pwm_map, test_center_aligned_edges_have_safe_endpoints)
{
	uint32_t val0;
	uint32_t val1;

	pwm_ytm32_etmr_center_edges(6000U, 0U, &val0, &val1);
	zassert_equal(val0, 0U, "zero duty VAL0");
	zassert_equal(val1, 0U, "zero duty VAL1");

	pwm_ytm32_etmr_center_edges(6000U, 0x4000U, &val0, &val1);
	zassert_equal(val0, 1499U, "50 percent VAL0");
	zassert_equal(val1, 4499U, "50 percent VAL1");

	pwm_ytm32_etmr_center_edges(6000U, 0x8000U, &val0, &val1);
	zassert_equal(val0, 0U, "full duty VAL0");
	zassert_equal(val1, 5999U, "full duty VAL1");
}

ZTEST(etmr_pwm_map, test_fault_polarity_maps_raw_inputs)
{
	/* V3 FLT3 is active-low: high is idle, low is an asserted fault. */
	zassert_equal(pwm_ytm32_etmr_fault_active_mask(0x00U, 0x08U, 0x08U),
		       0x08U, "active-low low level");
	zassert_equal(pwm_ytm32_etmr_fault_active_mask(0x08U, 0x08U, 0x08U),
		       0x00U, "active-low high idle level");

	/* A high-active input has the opposite truth table. */
	zassert_equal(pwm_ytm32_etmr_fault_active_mask(0x08U, 0x08U, 0x00U),
		       0x08U, "active-high high level");
	zassert_equal(pwm_ytm32_etmr_fault_active_mask(0x00U, 0x08U, 0x00U),
		       0x00U, "active-high low idle level");
}
