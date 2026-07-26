/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>

#include "cmu_threshold.h"

static int expect_thresholds(uint32_t monitored_hz, uint32_t reference_hz,
				     uint16_t expected_high, uint16_t expected_low)
{
	struct ytm32_cmu_thresholds thresholds;

	if (!ytm32_cmu_thresholds_from_hz(monitored_hz, reference_hz,
						 &thresholds)) {
		return 1;
	}

	return (thresholds.compare_high != expected_high) ||
		(thresholds.compare_low != expected_low);
}

static int expect_pll(uint32_t reference_hz, uint32_t reference_divider,
			     uint32_t feedback_divider, uint32_t expected_hz)
{
	uint32_t output_hz;

	if (!ytm32_cmu_pll_output_hz(reference_hz, reference_divider,
					     feedback_divider, &output_hz)) {
		return 1;
	}

	return output_hz != expected_hz;
}

int main(void)
{
	/* YTM32B1MD1 SDK: FIRC=96 MHz, SIRC=12 MHz. */
	if (expect_thresholds(96000000U, 12000000U, 1280U, 768U) != 0) {
		return 1;
	}

	/* YTM32B1MD1 SDK/EVB FXOSC=24 MHz, SIRC=12 MHz. */
	if (expect_thresholds(24000000U, 12000000U, 320U, 192U) != 0) {
		return 1;
	}

	/* YTM32B1MC0 SDK: FIRC=80 MHz, SIRC=2 MHz. */
	if (expect_thresholds(80000000U, 2000000U, 6400U, 3840U) != 0) {
		return 1;
	}

	/* YTM32B1MC0 SDK FXOSC=24 MHz, SIRC=2 MHz. */
	if (expect_thresholds(24000000U, 2000000U, 1920U, 1152U) != 0) {
		return 1;
	}

	/* Match the SDK's MHz truncation before its integer divisions. */
	if (expect_thresholds(20512500U, 12000000U, 266U, 160U) != 0) {
		return 1;
	}

	/* YTM32B1MD1 SDK demo PLL: FXOSC / 1 * 10 / 2 = 120 MHz. */
	if (expect_pll(24000000U, 1U, 10U, 120000000U) != 0) {
		return 1;
	}
	if (expect_thresholds(120000000U, 12000000U, 1600U, 960U) != 0) {
		return 1;
	}

	/* A valid FIRC-referenced configuration with the same output frequency. */
	if (expect_pll(96000000U, 4U, 10U, 120000000U) != 0) {
		return 1;
	}

	{
		struct ytm32_cmu_thresholds thresholds;
		uint32_t output_hz;

		if (ytm32_cmu_thresholds_from_hz(0U, 12000000U,
						 &thresholds) ||
		    ytm32_cmu_thresholds_from_hz(96000000U, 0U,
						 &thresholds) ||
		    ytm32_cmu_pll_output_hz(24000000U, 0U, 10U, &output_hz)) {
			return 1;
		}
	}

	puts("CMU threshold calculations passed");
	return 0;
}
