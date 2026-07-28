/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PWM_PWM_YTM32_ETMR_H_
#define ZEPHYR_INCLUDE_DRIVERS_PWM_PWM_YTM32_ETMR_H_

#include <zephyr/device.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*pwm_ytm32_ovf_cb_t)(void *user_data);
typedef void (*pwm_ytm32_fault_cb_t)(const struct device *dev,
					     uint32_t raw_status, void *user_data);

struct pwm_ytm32_fault_status {
	uint32_t raw_status;
	uint8_t fault_flags;
	uint8_t input_active_mask;
	uint32_t count;
	bool latched;
	bool armed;
};

/**
 * @brief Start the eTMR counter while keeping outputs in the current mask.
 *
 * Initialization and start never release the safe output mask.  Call
 * pwm_ytm32_etmr_release_safe() only after an application has deliberately
 * prepared a valid output command.
 */
int pwm_ytm32_etmr_start(const struct device *dev);

/** @brief Force all configured physical channels to a low safe state. */
int pwm_ytm32_etmr_force_safe(const struct device *dev);

/**
 * @brief Stop the counter after forcing all configured outputs safe.
 */
int pwm_ytm32_etmr_stop(const struct device *dev);

/**
 * @brief Release the software safe mask.
 *
 * This does not enable the power-stage EN_DRV signal.
 */
int pwm_ytm32_etmr_release_safe(const struct device *dev);

/** @brief Return whether the driver is holding outputs in a safe state. */
bool pwm_ytm32_etmr_is_safe(const struct device *dev);

/** @brief Return whether the DT phase-channel map was accepted. */
bool pwm_ytm32_etmr_phase_config_valid(const struct device *dev);

/**
 * @brief Register a callback invoked after a hardware fault is latched.
 *
 * The callback runs in eTMR fault ISR context after the output mask has been
 * applied and the fault interrupt has been disabled.  It must not clear the
 * hardware flag or attempt to restart the PWM.
 */
int pwm_ytm32_etmr_register_fault_cb(const struct device *dev,
					      pwm_ytm32_fault_cb_t cb,
					      void *user_data);

/** @brief Get the last fault snapshot and current input/latch state. */
int pwm_ytm32_etmr_fault_status_get(const struct device *dev,
					     struct pwm_ytm32_fault_status *status);

/**
 * @brief Clear a latched fault after all configured inputs are inactive.
 *
 * This re-arms the fault inputs while leaving the PWM output mask asserted;
 * call pwm_ytm32_etmr_release_safe() separately when the application is
 * ready.
 */
int pwm_ytm32_etmr_fault_clear(const struct device *dev);

/**
 * @brief Arm or disarm configured hardware fault inputs.
 *
 * Disarming is fail-safe and asserts the output mask.  Arming never releases
 * the output mask and refuses to proceed while an input is active or a fault
 * remains latched.
 */
int pwm_ytm32_etmr_fault_arm(const struct device *dev, bool arm);

/**
 * @brief Register a callback invoked from the eTMR overflow ISR.
 *
 * The callback runs at carrier frequency (typically 20 kHz) in ISR context.
 * It must complete before the next overflow — keep it short.
 */
int pwm_ytm32_register_ovf_cb(const struct device *dev,
			       pwm_ytm32_ovf_cb_t cb, void *user_data);

/**
 * @brief ISR-safe 3-phase duty update using the configured phase map.
 *
 * Updates shadow registers for all three phases and commits them atomically
 * with a software sync trigger.  Call only from within the OVF callback.
 * An invalid device-tree phase map makes this function a no-op and holds the
 * outputs in the safe state.
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
