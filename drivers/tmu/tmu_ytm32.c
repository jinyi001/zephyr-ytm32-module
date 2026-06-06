/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr driver for the YTMicro YTM32 TMU (Trigger MUX Unit).
 *
 * The TMU is a combinational routing block: each target module selects exactly
 * one trigger source.  This driver:
 *   - owns the TMU peripheral clock (enabled via clock_control at init),
 *   - applies the static routes described as devicetree child nodes,
 *   - exposes ytm32_tmu_route() for dynamic routing at runtime.
 *
 * It deliberately does NOT invent a generic Zephyr "trigger-mux" device class
 * (Zephyr has none): it is a small project-internal device with a single
 * routing entry point, consumed e.g. by the ADC driver to wire an eTMR
 * counter-bottom pulse to the ADC external-trigger input.
 */

#define DT_DRV_COMPAT ytmicro_ytm32_tmu

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "tmu_driver.h"
#include <zephyr/drivers/misc/ytm32_tmu.h>

LOG_MODULE_REGISTER(tmu_ytm32, CONFIG_YTM32_TMU_LOG_LEVEL);

struct tmu_ytm32_config {
	uint8_t instance;
	const struct device *clk_dev;
	clock_control_subsys_t clk_sys;
	const tmu_inout_mapping_config_t *routes;
	uint8_t num_routes;
};

int ytm32_tmu_route(const struct device *dev, uint32_t source, uint32_t target)
{
	const struct tmu_ytm32_config *cfg = dev->config;
	status_t s;

	s = TMU_DRV_SetTrigSourceForTargetModule(cfg->instance,
						 (tmu_trigger_source_t)source,
						 (tmu_target_module_t)target);
	if (s != STATUS_SUCCESS) {
		LOG_ERR("route src=%u -> target=%u failed (%d)", source, target, s);
		return -EIO;
	}

	LOG_DBG("routed src=%u -> target=%u", source, target);
	return 0;
}

uint32_t ytm32_tmu_get_route(const struct device *dev, uint32_t target)
{
	const struct tmu_ytm32_config *cfg = dev->config;

	return (uint32_t)TMU_DRV_GetTrigSourceForTargetModule(
			cfg->instance, (tmu_target_module_t)target);
}

static int tmu_ytm32_init(const struct device *dev)
{
	const struct tmu_ytm32_config *cfg = dev->config;
	tmu_user_config_t uc = {
		.numInOutMappingConfigs = cfg->num_routes,
		.inOutMappingConfig     = cfg->routes,
	};
	int ret;

	if (!device_is_ready(cfg->clk_dev)) {
		LOG_ERR("clock device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clk_dev, cfg->clk_sys);
	if (ret < 0) {
		LOG_ERR("clock_control_on failed (%d)", ret);
		return ret;
	}

	/* Reset all target routes to default, then apply the DT static routes. */
	if (TMU_DRV_Init(cfg->instance, &uc) != STATUS_SUCCESS) {
		LOG_ERR("TMU_DRV_Init failed (a target module is locked)");
		return -EIO;
	}

	LOG_DBG("TMU%u ready: %u static route(s)", cfg->instance, cfg->num_routes);
	return 0;
}

/* Build the static-route table from devicetree child nodes. */
#define TMU_ROUTE_ENTRY(child)						\
	{								\
		.triggerSource = (tmu_trigger_source_t)DT_PROP(child, source),	\
		.targetModule  = (tmu_target_module_t)DT_PROP(child, target),	\
	},

#define TMU_DEVICE_INIT(inst)						\
									\
	static const tmu_inout_mapping_config_t tmu_routes_##inst[] = {	\
		DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, TMU_ROUTE_ENTRY)	\
	};								\
									\
	static const struct tmu_ytm32_config tmu_ytm32_config_##inst = {\
		/* Single TMU peripheral on this SoC: HAL instance 0. */\
		.instance   = 0U,					\
		.clk_dev    = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),	\
		.clk_sys    = (clock_control_subsys_t)			\
			      DT_INST_CLOCKS_CELL(inst, id),		\
		.routes     = tmu_routes_##inst,			\
		.num_routes = ARRAY_SIZE(tmu_routes_##inst),		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			      tmu_ytm32_init,				\
			      NULL,					\
			      NULL,					\
			      &tmu_ytm32_config_##inst,			\
			      POST_KERNEL,				\
			      CONFIG_YTM32_TMU_INIT_PRIORITY,		\
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(TMU_DEVICE_INIT)
