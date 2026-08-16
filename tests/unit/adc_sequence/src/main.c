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
#define ADC_SAMPLE_TIME_COUNT 38U

static void adc_sequence_expand(uint64_t mask, const uint8_t *order,
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

ZTEST(adc_sequence, test_internal_reference_channels_are_supported)
{
	static const uint8_t order[] = {33U, 34U, 35U, 37U};
	uint8_t sequence[ADC_SEQUENCE_SLOTS] = {0};
	uint8_t count;
	uint8_t max_smp;
	uint64_t mask = BIT64(33) | BIT64(34) | BIT64(35) | BIT64(37);

	adc_sequence_expand(mask, order, ARRAY_SIZE(order), sequence, &count,
			    &max_smp);
	zassert_equal(count, ARRAY_SIZE(order), NULL);
	zassert_mem_equal(sequence, order, ARRAY_SIZE(order), NULL);
}

ZTEST(adc_sequence, test_explicit_order_allows_errata_dummy_duplicate)
{
	static const uint8_t order[] = {16U, 16U, 1U, 0U, 4U};
	uint8_t sequence[ADC_SEQUENCE_SLOTS] = {0};
	uint8_t count;
	uint8_t max_smp;
	uint64_t mask = BIT64(16) | BIT64(1) | BIT64(0) | BIT64(4);

	adc_sequence_expand(mask, order, ARRAY_SIZE(order), sequence, &count,
			    &max_smp);
	zassert_equal(count, ARRAY_SIZE(order), NULL);
	zassert_mem_equal(sequence, order, ARRAY_SIZE(order), NULL);
}

ZTEST(adc_sequence, test_explicit_order_must_cover_mask_without_extras)
{
	static const uint8_t partial[] = {16U, 1U};
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

ZTEST(adc_sequence, test_dma_full_sequence_trigger_plan_is_compatible)
{
	uint32_t triggers;

	zassert_ok(adc_ytm32_dma_trigger_plan(
		ADC_YTM32_DMA_SEQUENCE_FULL, false, 5U, 4U, &triggers), NULL);
	zassert_equal(triggers, 4U, "FULL needs one trigger per sequence");
}

ZTEST(adc_sequence, test_dma_step_trigger_plan_advances_one_slot)
{
	uint32_t triggers;

	zassert_ok(adc_ytm32_dma_trigger_plan(
		ADC_YTM32_DMA_SEQUENCE_STEP, true, 4U, 4U, &triggers), NULL);
	zassert_equal(triggers, 16U,
		"STEP needs one trigger per slot in every buffered sequence");
}

ZTEST(adc_sequence, test_dma_step_requires_hardware_trigger)
{
	uint32_t triggers;

	zassert_equal(adc_ytm32_dma_trigger_plan(
		ADC_YTM32_DMA_SEQUENCE_STEP, false, 4U, 1U, &triggers),
		-ENOTSUP, NULL);
	zassert_equal(adc_ytm32_dma_trigger_plan(
		(enum adc_ytm32_dma_sequence_mode)2, true, 4U, 1U,
		&triggers), -EINVAL, NULL);
}

ZTEST_SUITE(adc_sequence, NULL, NULL, NULL, NULL, NULL);
