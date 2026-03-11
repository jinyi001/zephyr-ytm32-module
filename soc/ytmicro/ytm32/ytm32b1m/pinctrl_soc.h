/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_YTM32_PINCTRL_SOC_H_
#define ZEPHYR_SOC_YTM32_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

/**
 * @brief Type for YTM32 pin.
 *
 * It contains the pinmux config (port, pin, mux).
 */
typedef struct pinctrl_soc_pin {
    uint32_t pinmux;
} pinctrl_soc_pin_t;

/**
 * @brief Utility macro to initialize each pin.
 *
 * @param group Group node identifier.
 * @param prop Property name.
 * @param idx Property entry index.
 */
#define Z_PINCTRL_STATE_PIN_INIT(group, prop, idx) \
    { \
        .pinmux = DT_PROP_BY_IDX(group, prop, idx), \
    },

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop) \
    { DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, pinmux, Z_PINCTRL_STATE_PIN_INIT) }

#endif /* ZEPHYR_SOC_YTM32_PINCTRL_SOC_H_ */
