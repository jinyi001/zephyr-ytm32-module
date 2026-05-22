/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * YTM32 ADC Phase 2 extension: hardware-triggered DMA continuous sampling.
 *
 * This API supplements the standard Zephyr adc_read() interface for the
 * motor-drive use case where the ADC must be triggered by the PWM timer
 * (ETMR0) and results transferred to memory without CPU involvement.
 *
 * Usage:
 *   1. Configure channels with adc_channel_setup() as normal.
 *   2. Allocate a buffer of channel_count * depth * sizeof(uint16_t) bytes.
 *   3. Call adc_ytm32_dma_start() once.
 *   4. The callback fires every 'depth' PWM periods with a pointer to the
 *      filled buffer.  Re-arm is automatic; call adc_ytm32_dma_stop() to halt.
 *
 * Buffer layout (depth=2, channels={0,2,4}):
 *   [ch0_period0, ch2_period0, ch4_period0,
 *    ch0_period1, ch2_period1, ch4_period1]
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ADC_ADC_YTM32_H_
#define ZEPHYR_INCLUDE_DRIVERS_ADC_ADC_YTM32_H_

#include <stdint.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback invoked from DMA ISR after 'depth' PWM periods of data.
 *
 * @param dev      ADC device
 * @param buf      Pointer to the filled sample buffer (same pointer passed
 *                 to adc_ytm32_dma_config.buf)
 * @param ch_count Number of active channels
 * @param depth    Number of complete sequences in the buffer
 * @param user_data Opaque pointer from adc_ytm32_dma_config
 */
typedef void (*adc_ytm32_dma_cb_t)(const struct device *dev,
				   uint16_t *buf,
				   uint8_t ch_count,
				   uint16_t depth,
				   void *user_data);

/**
 * @brief Configuration for hardware-triggered DMA sampling.
 */
struct adc_ytm32_dma_config {
	/** Channel bitmask — same semantics as adc_sequence.channels. */
	uint32_t channels;

	/** ADC resolution: 6, 8, 10, or 12 bits. */
	uint8_t resolution;

	/**
	 * Pre-allocated sample buffer.
	 * Required size: POPCOUNT(channels) * depth * sizeof(uint16_t).
	 */
	uint16_t *buf;

	/**
	 * Number of complete ADC sequences (PWM periods) per callback.
	 * Higher values reduce callback rate but increase latency.
	 */
	uint16_t depth;

	/** Callback invoked from DMA ISR after 'depth' sequences. */
	adc_ytm32_dma_cb_t cb;

	/** Opaque pointer forwarded to cb. */
	void *user_data;
};

/**
 * @brief Start hardware-triggered DMA continuous sampling.
 *
 * Configures the ADC for hardware trigger mode, routes the TMU trigger
 * source (from DTS ytmicro,hw-trigger-source) to ADC0_EXT_TRIG, and arms
 * the DMA channel to drain the FIFO into buf on every sequence completion.
 *
 * The calling code is responsible for ensuring:
 * - ETMR0 is configured and running in center-aligned PWM mode.
 * - ETMR0 OTRIG register has INITTEN=1 to emit the trigger on counter=0.
 * - adc_channel_setup() has been called for every channel in cfg->channels.
 *
 * @param dev ADC device (e.g. DEVICE_DT_GET(DT_NODELABEL(adc0)))
 * @param cfg DMA sampling configuration
 * @return 0 on success, negative errno on failure
 */
int adc_ytm32_dma_start(const struct device *dev,
			const struct adc_ytm32_dma_config *cfg);

/**
 * @brief Stop hardware-triggered DMA sampling.
 *
 * Stops the ADC and DMA channel.  Safe to call from any context.
 *
 * @param dev ADC device
 * @return 0 on success, negative errno on failure
 */
int adc_ytm32_dma_stop(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_ADC_ADC_YTM32_H_ */
