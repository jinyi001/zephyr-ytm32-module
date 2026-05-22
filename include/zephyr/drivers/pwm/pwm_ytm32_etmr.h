/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PWM_PWM_YTM32_ETMR_H_
#define ZEPHYR_INCLUDE_DRIVERS_PWM_PWM_YTM32_ETMR_H_

#include <zephyr/device.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*pwm_ytm32_ovf_cb_t)(void *user_data);

/**
 * @brief Register a callback invoked from the eTMR overflow ISR.
 *
 * The callback runs at carrier frequency (typically 20 kHz) in ISR context.
 * It must complete before the next overflow — keep it short.
 */
int pwm_ytm32_register_ovf_cb(const struct device *dev,
			       pwm_ytm32_ovf_cb_t cb, void *user_data);

/**
 * @brief ISR-safe 3-phase duty update (complementary pairs 0/1, 2/3, 4/5).
 *
 * Updates shadow registers for all three phases and commits them atomically
 * with a software sync trigger.  Call only from within the OVF callback.
 *
 * @param da_q15  Phase-A duty in Q15 [0, 0x8000]
 * @param db_q15  Phase-B duty in Q15 [0, 0x8000]
 * @param dc_q15  Phase-C duty in Q15 [0, 0x8000]
 */
void pwm_ytm32_update_3phase_isr(const struct device *dev,
				  uint16_t da_q15, uint16_t db_q15,
				  uint16_t dc_q15);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PWM_PWM_YTM32_ETMR_H_ */
