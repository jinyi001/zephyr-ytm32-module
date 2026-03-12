/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@Foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_YTM32_PINCTRL_SOC_H_
#define ZEPHYR_SOC_YTM32_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

/**
 * @brief Type for YTM32 pin.
 *
 * It contains the pinmux config (port, pin, mux) and pincfg (features).
 */
typedef struct pinctrl_soc_pin {
    uint32_t pinmux;
    uint32_t pincfg;
} pinctrl_soc_pin_t;

/**
 * @name YTM32 pin configuration bit field positions and masks.
 * @{
 */
#define YTM32_PULL_UP_POS       0U
#define YTM32_PULL_DOWN_POS     1U
#define YTM32_PULL_DSB_POS      2U
#define YTM32_DRV_STR_POS       3U
#define YTM32_OPEN_DRAIN_POS    4U
#define YTM32_PUSH_PULL_POS     5U
#define YTM32_SLEW_RATE_POS     6U
#define YTM32_PASSIVE_FLT_POS   7U

#define YTM32_PULL_UP_MSK       (1U << YTM32_PULL_UP_POS)
#define YTM32_PULL_DOWN_MSK     (1U << YTM32_PULL_DOWN_POS)
#define YTM32_DRV_STR_MSK       (1U << YTM32_DRV_STR_POS)
#define YTM32_OPEN_DRAIN_MSK    (1U << YTM32_OPEN_DRAIN_POS)
#define YTM32_SLEW_RATE_MSK     (1U << YTM32_SLEW_RATE_POS)
#define YTM32_PASSIVE_FLT_MSK   (1U << YTM32_PASSIVE_FLT_POS)
/** @} */

/**
 * @brief Utility macro to extract pin configuration from the device tree node.
 * 
 * Packs all boolean and enum properties into a single 32-bit integer.
 */
#define Z_PINCTRL_YTM32_PINCFG(node_id)                                        \
    ((DT_PROP(node_id, bias_pull_up) << YTM32_PULL_UP_POS) |                   \
     (DT_PROP(node_id, bias_pull_down) << YTM32_PULL_DOWN_POS) |               \
     (DT_PROP(node_id, bias_disable) << YTM32_PULL_DSB_POS) |                  \
     ((DT_ENUM_IDX_OR(node_id, drive_strength, 0) == 1) << YTM32_DRV_STR_POS) |\
     (DT_PROP(node_id, drive_open_drain) << YTM32_OPEN_DRAIN_POS) |            \
     (DT_PROP(node_id, drive_push_pull) << YTM32_PUSH_PULL_POS) |              \
     ((DT_ENUM_IDX_OR(node_id, slew_rate, 0) == 1) << YTM32_SLEW_RATE_POS) |   \
     (DT_PROP(node_id, ytmicro_passive_filter) << YTM32_PASSIVE_FLT_POS))

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
        .pincfg = Z_PINCTRL_YTM32_PINCFG(group), \
    },

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop) \
    { DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, pinmux, Z_PINCTRL_STATE_PIN_INIT) }

#endif /* ZEPHYR_SOC_YTM32_PINCTRL_SOC_H_ */
