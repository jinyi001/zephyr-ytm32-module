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

	zassert_equal(pwm_ytm32_etmr_phase_complementary_mask(phase_channels),
			       0x54U, "phase map derives complementary mask");
	zassert_ok(pwm_ytm32_etmr_phase_map_validate(phase_channels, &map),
			   "V3 phase map should be valid");
	zassert_equal(map.phase_channel[0], 6U, "phase A channel");
	zassert_equal(map.phase_channel[1], 2U, "phase B channel");
	zassert_equal(map.phase_channel[2], 4U, "phase C channel");
	zassert_equal(map.channel_mask, 0xFCU, "physical output mask");
	zassert_equal(pwm_ytm32_etmr_complementary_channel_mask(
				pwm_ytm32_etmr_phase_complementary_mask(phase_channels)), 0xFCU,
		       "complementary mask expansion");
}

ZTEST(etmr_pwm_map, test_rejects_invalid_phase_maps)
{
	static const uint8_t odd_channel[PWM_YTM32_ETMR_PHASE_COUNT] = {7U, 2U,
								 4U};
	static const uint8_t out_of_range[PWM_YTM32_ETMR_PHASE_COUNT] = {8U, 2U,
									4U};
	static const uint8_t duplicate[PWM_YTM32_ETMR_PHASE_COUNT] = {6U, 6U,
									4U};
	struct pwm_ytm32_etmr_phase_map map;

	zassert_equal(pwm_ytm32_etmr_phase_complementary_mask(odd_channel), 0U,
			       "odd channel must not derive a pair");
	zassert_equal(pwm_ytm32_etmr_phase_map_validate(odd_channel, &map), -EINVAL,
				      "odd channel must be rejected");
	zassert_equal(pwm_ytm32_etmr_phase_map_validate(out_of_range, &map), -EINVAL,
				      "out-of-range channel must be rejected");
	zassert_equal(pwm_ytm32_etmr_phase_map_validate(duplicate, &map), -EINVAL,
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

ZTEST(etmr_pwm_map, test_erratum_endpoint_masks_for_complementary_pairs)
{
	const uint8_t phase_channels[PWM_YTM32_ETMR_PHASE_COUNT] = {6U, 2U, 4U};
	struct pwm_ytm32_etmr_output_mask mask = {0};
	struct pwm_ytm32_etmr_output_mask unpacked;
	uint32_t packed;

	for (size_t phase = 0U; phase < PWM_YTM32_ETMR_PHASE_COUNT; phase++) {
		pwm_ytm32_etmr_endpoint_mask_update(&mask,
						     phase_channels[phase], 0U);
	}
	zassert_equal(mask.enable, 0xFCU, "all-zero masks all phase outputs");
	zassert_equal(mask.value, 0x4440U,
		      "all-zero forces even low and complementary odd high");

	for (size_t phase = 0U; phase < PWM_YTM32_ETMR_PHASE_COUNT; phase++) {
		pwm_ytm32_etmr_endpoint_mask_update(
			&mask, phase_channels[phase], PWM_YTM32_ETMR_MAX_DUTY_Q15);
	}
	zassert_equal(mask.enable, 0xFCU, "all-full masks all phase outputs");
	zassert_equal(mask.value, 0x1110U,
		      "all-full forces even high and complementary odd low");

	pwm_ytm32_etmr_endpoint_mask_update(&mask, 4U, 0x4000U);
	zassert_equal(mask.enable, 0xCCU, "interior duty releases its pair");
	zassert_equal(mask.value, 0x1010U, "interior duty clears pair values");

	packed = pwm_ytm32_etmr_output_mask_pack(&mask);
	zassert_equal(packed, 0x101000CCU, "packed CHMASK register value");
	unpacked = pwm_ytm32_etmr_output_mask_unpack(packed);
	zassert_equal(unpacked.enable, mask.enable, "unpacked mask enables");
	zassert_equal(unpacked.value, mask.value, "unpacked mask values");
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

ZTEST(etmr_pwm_map, test_fault_status_from_hal_is_already_polarity_decoded)
{
	/* IOSTS.F3 is zero when an active-low, high-idle FLT3 is healthy. */
	zassert_equal(pwm_ytm32_etmr_fault_status_active_mask(0x00U, 0x08U),
		       0x00U, "polarity-decoded idle status");

	/* IOSTS.F3 becomes one only when eTMR has detected an active fault. */
	zassert_equal(pwm_ytm32_etmr_fault_status_active_mask(0x08U, 0x08U),
		       0x08U, "polarity-decoded active status");

	/* Unconfigured status bits must not escape the configured mask. */
	zassert_equal(pwm_ytm32_etmr_fault_status_active_mask(0x0FU, 0x08U),
		       0x08U, "configured status mask");
}
