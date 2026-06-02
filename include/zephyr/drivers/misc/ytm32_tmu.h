/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_YTM32_TMU_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_YTM32_TMU_H_

#include <zephyr/device.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief YTM32 TMU (Trigger MUX Unit) routing API.
 *
 * The TMU is a combinational routing block: each target module selects exactly
 * one trigger source.  Static routes can be described declaratively in
 * devicetree (applied at driver init); this function performs the same routing
 * dynamically at runtime.
 *
 * The driver owns the TMU peripheral clock (enabled via clock_control at init),
 * so callers must not touch the TMU clock gate themselves.
 *
 * @param dev    TMU device (e.g. DEVICE_DT_GET(DT_NODELABEL(tmu0)))
 * @param source Trigger source id (YTM32_TMU_SRC_* from the SoC dt-bindings)
 * @param target Target module id  (YTM32_TMU_TARGET_* from the SoC dt-bindings)
 * @return 0 on success, negative errno on failure
 */
int ytm32_tmu_route(const struct device *dev, uint32_t source, uint32_t target);

/**
 * @brief Read back the trigger source currently routed to a target module.
 *
 * Useful for bring-up/diagnostics to confirm a route register is programmed.
 *
 * @param dev    TMU device
 * @param target Target module id (YTM32_TMU_TARGET_*)
 * @return Trigger source id (YTM32_TMU_SRC_*) currently selected for @p target
 */
uint32_t ytm32_tmu_get_route(const struct device *dev, uint32_t target);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MISC_YTM32_TMU_H_ */
