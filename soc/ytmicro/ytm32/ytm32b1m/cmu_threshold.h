/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef YTM32_CMU_THRESHOLD_H_
#define YTM32_CMU_THRESHOLD_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The vendor CMU counts monitored-clock edges during 128 reference cycles. */
#define YTM32_CMU_REFERENCE_CYCLES 128U

struct ytm32_cmu_thresholds {
	uint16_t compare_high;
	uint16_t compare_low;
};

/*
 * Derive the vendor SDK's +/-25% CMU window from clock frequencies in Hz.
 * Preserve the SDK's integer ordering: it truncates both frequencies to MHz
 * before applying the 25% window and subsequent divisions.
 *
 *   high = (monitored_hz / 1MHz) * 128 * 5 / 4 / (reference_hz / 1MHz)
 *   low  = (monitored_hz / 1MHz) * 128 * 3 / 4 / (reference_hz / 1MHz)
 *
 * The 64-bit intermediates prevent the widened MHz value from overflowing
 * before the HAL-equivalent integer divisions. The result is rejected when
 * it cannot be represented by the HAL's uint16_t compare fields.
 */
static inline bool ytm32_cmu_thresholds_from_hz(
	uint32_t monitored_hz, uint32_t reference_hz,
	struct ytm32_cmu_thresholds *thresholds)
{
	uint64_t monitored_mhz;
	uint64_t reference_mhz;
	uint64_t high;
	uint64_t low;

	if ((monitored_hz == 0U) || (reference_hz == 0U) ||
	    (thresholds == NULL)) {
		return false;
	}

	monitored_mhz = monitored_hz / 1000000U;
	reference_mhz = reference_hz / 1000000U;
	if ((monitored_mhz == 0U) || (reference_mhz == 0U)) {
		return false;
	}

	high = (monitored_mhz * YTM32_CMU_REFERENCE_CYCLES * 5U) /
		4U / reference_mhz;
	low = (monitored_mhz * YTM32_CMU_REFERENCE_CYCLES * 3U) /
		4U / reference_mhz;

	if ((high > UINT16_MAX) || (low > UINT16_MAX) || (high <= low)) {
		return false;
	}

	thresholds->compare_high = (uint16_t)high;
	thresholds->compare_low = (uint16_t)low;
	return true;
}

/*
 * Match CLOCK_DRV_GetPllFreq(): divide the reference first, then multiply by
 * the feedback divider and divide by two.  The multiplication is widened so
 * invalid-but-representable configurations cannot wrap a 32-bit temporary.
 */
static inline bool ytm32_cmu_pll_output_hz(uint32_t reference_hz,
						    uint32_t reference_divider,
						    uint32_t feedback_divider,
						    uint32_t *output_hz)
{
	uint64_t divided_reference;
	uint64_t output;

	if ((reference_hz == 0U) || (reference_divider == 0U) ||
	    (feedback_divider == 0U) || (output_hz == NULL)) {
		return false;
	}

	divided_reference = (uint64_t)reference_hz / reference_divider;
	output = (divided_reference * feedback_divider) / 2U;
	if (output > UINT32_MAX) {
		return false;
	}

	*output_hz = (uint32_t)output;
	return true;
}

#endif /* YTM32_CMU_THRESHOLD_H_ */
