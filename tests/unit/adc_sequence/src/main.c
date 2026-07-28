/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/ztest.h>
#include <zephyr/sys/util.h>

#include "adc_ytm32_logic.h"

#define ADC_SEQUENCE_SLOTS 8U
#define ADC_SAMPLE_TIME_COUNT 32U

static void adc_sequence_expand(uint32_t mask, const uint8_t *order,
				uint8_t order_count, uint8_t *sequence,
				uint8_t *count, uint8_t *max_smp)
{
	static const uint8_t sample_times[ADC_SAMPLE_TIME_COUNT] = {
		[0 ... ADC_SAMPLE_TIME_COUNT - 1] = 8U,
	};

	zassert_ok(adc_ytm32_sequence_expand(mask, order, order_count,
					     ADC_SEQUENCE_SLOTS, sequence,
					     sample_times,
					     ADC_SAMPLE_TIME_COUNT, count,
					     max_smp), NULL);
}

ZTEST(adc_sequence, test_explicit_order_is_preserved)
{
	static const uint8_t order[] = {16U, 1U, 0U, 4U};
	uint8_t sequence[ADC_SEQUENCE_SLOTS] = {0};
	uint8_t count;
	uint8_t max_smp;

	adc_sequence_expand(BIT(16) | BIT(1) | BIT(0) | BIT(4), order,
				    ARRAY_SIZE(order), sequence, &count, &max_smp);
	zassert_equal(count, ARRAY_SIZE(order), NULL);
	zassert_mem_equal(sequence, order, ARRAY_SIZE(order), NULL);
	zassert_equal(max_smp, 8U, NULL);
}

ZTEST(adc_sequence, test_empty_order_keeps_ascending_compatibility)
{
	static const uint8_t expected[] = {0U, 4U, 16U};
	uint8_t sequence[ADC_SEQUENCE_SLOTS] = {0};
	uint8_t count;
	uint8_t max_smp;

	adc_sequence_expand(BIT(16) | BIT(4) | BIT(0), NULL, 0U, sequence,
				    &count, &max_smp);
	zassert_equal(count, ARRAY_SIZE(expected), NULL);
	zassert_mem_equal(sequence, expected, ARRAY_SIZE(expected), NULL);
}

ZTEST(adc_sequence, test_explicit_order_must_match_mask_exactly)
{
	static const uint8_t partial[] = {16U, 1U};
	static const uint8_t duplicate[] = {16U, 1U, 1U, 4U};
	static const uint8_t unselected[] = {16U, 2U, 0U, 4U};
	uint8_t sequence[ADC_SEQUENCE_SLOTS] = {0};
	uint8_t count;
	uint8_t max_smp;
	static const uint8_t sample_times[ADC_SAMPLE_TIME_COUNT] = {
		[0 ... ADC_SAMPLE_TIME_COUNT - 1] = 8U,
	};
	uint32_t mask = BIT(16) | BIT(1) | BIT(0) | BIT(4);

	zassert_equal(adc_ytm32_sequence_expand(mask, partial,
					ARRAY_SIZE(partial), ADC_SEQUENCE_SLOTS,
					sequence, sample_times,
					ADC_SAMPLE_TIME_COUNT, &count, &max_smp), -EINVAL,
				"partial order must be rejected");
	zassert_equal(adc_ytm32_sequence_expand(mask, duplicate,
					ARRAY_SIZE(duplicate), ADC_SEQUENCE_SLOTS,
					sequence, sample_times,
					ADC_SAMPLE_TIME_COUNT, &count, &max_smp), -EINVAL,
				"duplicate order entry must be rejected");
	zassert_equal(adc_ytm32_sequence_expand(mask, unselected,
					ARRAY_SIZE(unselected), ADC_SEQUENCE_SLOTS,
					sequence, sample_times,
					ADC_SAMPLE_TIME_COUNT, &count, &max_smp), -EINVAL,
				"unselected channel must be rejected");
}

ZTEST(adc_sequence, test_ds_v19_timing_boundaries)
{
	zassert_ok(adc_ytm32_validate_timing(32000000U, 8U, 63U), NULL);
	zassert_ok(adc_ytm32_validate_timing(16000000U, 4U, 31U), NULL);
	zassert_equal(adc_ytm32_validate_timing(3999999U, 8U, 63U), -ERANGE,
			      "FADC below DS v1.9 range");
	zassert_equal(adc_ytm32_validate_timing(32000001U, 8U, 63U), -ERANGE,
			      "FADC above DS v1.9 range");
	zassert_equal(adc_ytm32_validate_timing(32000000U, 1U, 63U), -ERANGE,
			      "sample time below 100 ns");
	zassert_equal(adc_ytm32_validate_timing(32000000U, 8U, 62U), -ERANGE,
			      "startup time below 2 us");
	zassert_equal(adc_ytm32_ticks_to_ns(64U, 32000000U), 2000U, NULL);
}

ZTEST_SUITE(adc_sequence, NULL, NULL, NULL, NULL, NULL);
