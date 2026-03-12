/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ytmicro_ytm32_cgu

#include <errno.h>
#include <stddef.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_ytm32, CONFIG_CLOCK_CONTROL_LOG_LEVEL);
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/ytm32_soc_clock.h>
#include <zephyr/device.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/clock/ytmicro,ytm32-clock.h>

/* Driver-private constants — not part of the dt-binding public API */
#define YTM32_IPC_DIV_MIN 1U
#define YTM32_IPC_DIV_MAX 16U

/*
 * HAL status codes — sourced from vendor SDK status_t conventions.
 * Used only inside this driver to bridge HAL return values to errno.
 *   STATUS_SUCCESS      = 0
 *   STATUS_UNSUPPORTED  = 4
 *   STATUS_MCU_GATED_OFF = 0x100
 */
#define YTM32_HAL_STATUS_SUCCESS       0U
#define YTM32_HAL_STATUS_UNSUPPORTED   4U
#define YTM32_HAL_STATUS_MCU_GATED_OFF 0x100U

/**
 * @brief Convert a vendor HAL status_t value to a Zephyr errno.
 *
 * Mapping derived from the YTM32 SDK status_t enumeration:
 *   STATUS_SUCCESS       (0)     -> 0
 *   STATUS_UNSUPPORTED   (4)     -> -ENOTSUP
 *   STATUS_MCU_GATED_OFF (0x100) -> -EAGAIN  (clock gated, may succeed later)
 *   anything else                -> -EIO
 */
static inline int ytm32_hal_status_to_errno(int hal_status)
{
	switch ((uint32_t)hal_status) {
	case YTM32_HAL_STATUS_SUCCESS:
		return 0;
	case YTM32_HAL_STATUS_UNSUPPORTED:
		return -ENOTSUP;
	case YTM32_HAL_STATUS_MCU_GATED_OFF:
		return -EAGAIN;
	default:
		return -EIO;
	}
}

/* Vendor HAL clock functions (from modules/hal/ytmicro SDK) */
extern void CLOCK_DRV_SetModuleClock(uint32_t clockName, bool clockGate,
				     uint32_t clkSrc, uint32_t divider);
extern int CLOCK_SYS_GetFreq(uint32_t clockName, uint32_t *frequency);

struct clock_control_ytm32_config {
	uint32_t core_clock;
	uint32_t core_divider;
	uint32_t fast_bus_divider;
	uint32_t slow_bus_divider;
};

struct clock_control_ytm32_data {
	uint32_t core_rate;
	uint32_t fast_bus_rate;
	uint32_t slow_bus_rate;
	bool system_rates_valid;
	struct k_spinlock lock;
	uint16_t module_refcnt[YTM32_CLOCK_IPC_END];
};

struct ytm32_module_clock_config {
	uint32_t clock_id;
	uint32_t clk_src;
	uint32_t divider;
};

static const struct ytm32_module_clock_config ytm32_fixed_module_clocks[] = {
	{ YTM32_CLOCK_GPIO, YTM32_CLOCK_SRC_FIRC, 1U },
	{ YTM32_CLOCK_PCTRLA, YTM32_CLOCK_SRC_FIRC, 1U },
	{ YTM32_CLOCK_PCTRLB, YTM32_CLOCK_SRC_FIRC, 1U },
	{ YTM32_CLOCK_PCTRLC, YTM32_CLOCK_SRC_FIRC, 1U },
	{ YTM32_CLOCK_PCTRLD, YTM32_CLOCK_SRC_FIRC, 1U },
	{ YTM32_CLOCK_PCTRLE, YTM32_CLOCK_SRC_FIRC, 1U },
};

#define YTM32_DT_FUNCTIONAL_CLOCK_ENTRY(node_id) \
	{ \
		.clock_id = DT_CLOCKS_CELL(node_id, id), \
		.clk_src = DT_PROP_OR(node_id, ytmicro_functional_clock_source, \
			YTM32_CLOCK_SRC_DISABLED), \
		.divider = DT_PROP_OR(node_id, ytmicro_functional_clock_divider, 1U), \
	},

#define YTM32_APPEND_DTS_UART_CLOCK(node_id) \
	IF_ENABLED(DT_NODE_HAS_COMPAT(node_id, ytmicro_ytm32_uart), \
		(YTM32_DT_FUNCTIONAL_CLOCK_ENTRY(node_id)))

static const struct ytm32_module_clock_config ytm32_dts_module_clocks[] = {
	DT_FOREACH_CHILD_STATUS_OKAY(DT_PATH(soc), YTM32_APPEND_DTS_UART_CLOCK)
};

#undef YTM32_APPEND_DTS_UART_CLOCK
#undef YTM32_DT_FUNCTIONAL_CLOCK_ENTRY

static const struct ytm32_module_clock_config *ytm32_find_module_clock(uint32_t clock_id)
{
	for (size_t i = 0; i < ARRAY_SIZE(ytm32_dts_module_clocks); i++) {
		if (ytm32_dts_module_clocks[i].clock_id == clock_id) {
			return &ytm32_dts_module_clocks[i];
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(ytm32_fixed_module_clocks); i++) {
		if (ytm32_fixed_module_clocks[i].clock_id == clock_id) {
			return &ytm32_fixed_module_clocks[i];
		}
	}

	return NULL;
}


static const char *ytm32_clock_label(uint32_t clock_id)
{
	switch (clock_id) {
	case YTM32_CLOCK_GPIO:
		return "gpio";
	case YTM32_CLOCK_PCTRLA:
		return "pctrla";
	case YTM32_CLOCK_PCTRLB:
		return "pctrlb";
	case YTM32_CLOCK_PCTRLC:
		return "pctrlc";
	case YTM32_CLOCK_PCTRLD:
		return "pctrld";
	case YTM32_CLOCK_PCTRLE:
		return "pctrle";
	case YTM32_CLOCK_UART0:
		return "uart0";
	case YTM32_CLOCK_UART1:
		return "uart1";
	case YTM32_CLOCK_UART2:
		return "uart2";
	case YTM32_CLOCK_CORE:
		return "core";
	case YTM32_CLOCK_FAST_BUS:
		return "fast_bus";
	case YTM32_CLOCK_SLOW_BUS:
		return "slow_bus";
	default:
		return "unknown";
	}
}

static bool ytm32_is_supported_module_source(uint32_t clk_src)
{
	switch (clk_src) {
	case YTM32_CLOCK_SRC_FIRC:
	case YTM32_CLOCK_SRC_SIRC:
	case YTM32_CLOCK_SRC_FXOSC:
	case YTM32_CLOCK_SRC_LPO:
	case YTM32_CLOCK_SRC_FAST_BUS:
		return true;
	default:
		return false;
	}
}

static int ytm32_validate_module_clock(const struct ytm32_module_clock_config *clock_cfg,
					       uint32_t clock_id)
{
	if (clock_cfg == NULL) {
		return -EINVAL;
	}

	if (!ytm32_is_supported_module_source(clock_cfg->clk_src)) {
		LOG_ERR("Unsupported YTM32 clock source %u for %s (%u)",
			clock_cfg->clk_src, ytm32_clock_label(clock_id), clock_id);
		return -EINVAL;
	}

	if ((clock_cfg->divider < YTM32_IPC_DIV_MIN) ||
	    (clock_cfg->divider > YTM32_IPC_DIV_MAX)) {
		LOG_ERR("Unsupported YTM32 divider %u for %s (%u)",
			clock_cfg->divider, ytm32_clock_label(clock_id), clock_id);
		return -EINVAL;
	}

	return 0;
}

static int ytm32_log_system_rate(uint32_t clock_id, const char *label)
{
	uint32_t rate = 0U;
	int hal_status = CLOCK_SYS_GetFreq(clock_id, &rate);
	int ret = ytm32_hal_status_to_errno(hal_status);

	if (ret != 0) {
		LOG_ERR("Failed to query %s rate (HAL status %d)", label, hal_status);
		return ret;
	}

	LOG_INF("%s rate: %u Hz", label, rate);
	return 0;
}

static bool ytm32_is_system_clock_id(uint32_t clock_id)
{
	return (clock_id == YTM32_CLOCK_CORE) ||
	       (clock_id == YTM32_CLOCK_FAST_BUS) ||
	       (clock_id == YTM32_CLOCK_SLOW_BUS);
}

static int ytm32_query_clock_rate(uint32_t clock_id, const char *label,
				       uint32_t *rate)
{
	int hal_status;
	int ret;

	if (rate == NULL) {
		return -EINVAL;
	}

	hal_status = CLOCK_SYS_GetFreq(clock_id, rate);
	ret = ytm32_hal_status_to_errno(hal_status);

	if (ret != 0) {
		LOG_ERR("Failed to query %s rate (HAL status %d)", label, hal_status);
	}

	return ret;
}

static int ytm32_module_clock_request(const struct device *dev, uint32_t clock_id,
				      const struct ytm32_module_clock_config *clock_cfg,
				      bool enable)
{
	struct clock_control_ytm32_data *data = dev->data;
	k_spinlock_key_t key;
	uint16_t *refcnt;

	if (clock_id >= YTM32_CLOCK_IPC_END) {
		return -ENOTSUP;
	}

	refcnt = &data->module_refcnt[clock_id];
	key = k_spin_lock(&data->lock);

	if (enable) {
		if (*refcnt == UINT16_MAX) {
			k_spin_unlock(&data->lock, key);
			return -EOVERFLOW;
		}

		if (*refcnt == 0U) {
			CLOCK_DRV_SetModuleClock(clock_id, true,
					 clock_cfg->clk_src,
					 clock_cfg->divider - 1U);
		}

		(*refcnt)++;
	} else {
		if (*refcnt == 0U) {
			k_spin_unlock(&data->lock, key);
			return -EALREADY;
		}

		(*refcnt)--;
		if (*refcnt == 0U) {
			CLOCK_DRV_SetModuleClock(clock_id, false,
					 clock_cfg->clk_src,
					 clock_cfg->divider - 1U);
		}
	}

	k_spin_unlock(&data->lock, key);
	return 0;
}

static int ytm32_refresh_system_rates(const struct device *dev)
{
	const struct clock_control_ytm32_config *cfg = dev->config;
	struct clock_control_ytm32_data *data = dev->data;
	uint32_t expected_fast_rate = cfg->core_clock / cfg->fast_bus_divider;
	uint32_t expected_slow_rate = expected_fast_rate / cfg->slow_bus_divider;
	int ret;

	ret = ytm32_query_clock_rate(YTM32_CLOCK_CORE, "core",
				       &data->core_rate);
	if (ret < 0) {
		return ret;
	}

	ret = ytm32_query_clock_rate(YTM32_CLOCK_FAST_BUS, "fast bus",
				       &data->fast_bus_rate);
	if (ret < 0) {
		return ret;
	}

	ret = ytm32_query_clock_rate(YTM32_CLOCK_SLOW_BUS, "slow bus",
				       &data->slow_bus_rate);
	if (ret < 0) {
		return ret;
	}

	if (data->core_rate != cfg->core_clock) {
		LOG_ERR("Core clock mismatch: DTS %u Hz, HW %u Hz",
			cfg->core_clock, data->core_rate);
		return -EIO;
	}

	if (data->fast_bus_rate != expected_fast_rate) {
		LOG_ERR("Fast bus clock mismatch: DTS %u Hz, HW %u Hz",
			expected_fast_rate, data->fast_bus_rate);
		return -EIO;
	}

	if (data->slow_bus_rate != expected_slow_rate) {
		LOG_ERR("Slow bus clock mismatch: DTS %u Hz, HW %u Hz",
			expected_slow_rate, data->slow_bus_rate);
		return -EIO;
	}

	data->system_rates_valid = true;

	return 0;
}

/**
 * @brief Enable a specific clock
 * 
 * @param dev Clock controller device
 * @param sys Clock identifier (cast from uint32_t)
 * @return 0 on success, negative errno on failure
 */
static int clock_control_ytm32_on(const struct device *dev, clock_control_subsys_t sys)
{
	uint32_t clock_id = (uint32_t)(uintptr_t)sys;
	const struct ytm32_module_clock_config *clock_cfg;
	int ret;
	
	clock_cfg = ytm32_find_module_clock(clock_id);
	if (clock_cfg == NULL) {
		LOG_ERR("Unsupported YTM32 module clock %s (%u)",
			ytm32_clock_label(clock_id), clock_id);
		return -ENOTSUP;
	}

	if (ytm32_validate_module_clock(clock_cfg, clock_id) < 0) {
		return -EINVAL;
	}

	LOG_INF("Enable YTM32 clock %s (%u) src %u div %u",
		ytm32_clock_label(clock_id), clock_id,
		clock_cfg->clk_src, clock_cfg->divider);

	ret = ytm32_module_clock_request(dev, clock_id, clock_cfg, true);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int clock_control_ytm32_off(const struct device *dev, clock_control_subsys_t sys)
{
	uint32_t clock_id = (uint32_t)(uintptr_t)sys;
	const struct ytm32_module_clock_config *clock_cfg;
	int ret;

	if (ytm32_is_system_clock_id(clock_id)) {
		LOG_ERR("System clock %s (%u) cannot be gated off",
			ytm32_clock_label(clock_id), clock_id);
		return -ENOTSUP;
	}

	clock_cfg = ytm32_find_module_clock(clock_id);
	if (clock_cfg == NULL) {
		LOG_ERR("Unsupported YTM32 module clock %s (%u)",
			ytm32_clock_label(clock_id), clock_id);
		return -ENOTSUP;
	}

	if (ytm32_validate_module_clock(clock_cfg, clock_id) < 0) {
		return -EINVAL;
	}

	LOG_INF("Disable YTM32 clock %s (%u)",
		ytm32_clock_label(clock_id), clock_id);

	ret = ytm32_module_clock_request(dev, clock_id, clock_cfg, false);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int clock_control_ytm32_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate)
{
	struct clock_control_ytm32_data *data = dev->data;
	uint32_t clock_id = (uint32_t)(uintptr_t)sys;

	if (rate == NULL) {
		return -EINVAL;
	}

	if (ytm32_is_system_clock_id(clock_id)) {
		if (!data->system_rates_valid) {
			return -EIO;
		}

		switch (clock_id) {
		case YTM32_CLOCK_CORE:
			*rate = data->core_rate;
			break;
		case YTM32_CLOCK_FAST_BUS:
			*rate = data->fast_bus_rate;
			break;
		case YTM32_CLOCK_SLOW_BUS:
			*rate = data->slow_bus_rate;
			break;
		default:
			return -ENOTSUP;
		}

		return 0;
	}

	if (ytm32_find_module_clock(clock_id) == NULL) {
		LOG_ERR("Unsupported YTM32 clock %s (%u)",
			ytm32_clock_label(clock_id), clock_id);
		return -ENOTSUP;
	}

	return ytm32_query_clock_rate(clock_id,
				      ytm32_clock_label(clock_id), rate);

}

static const struct clock_control_driver_api clock_control_ytm32_api = {
	.on = clock_control_ytm32_on,
	.off = clock_control_ytm32_off,
	.get_rate = clock_control_ytm32_get_rate,
};

#define YTM32_DT_FAST_BUS_DIVIDER(inst) DT_INST_PROP_OR(inst, fast_bus_divider, 1)

#define YTM32_DT_SLOW_BUS_DIVIDER(inst) \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, slow_bus_divider), \
		(DT_INST_PROP(inst, slow_bus_divider)), \
		(COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, bus_divider), \
			(DT_INST_PROP(inst, bus_divider)), \
			(0U))))

static int clock_control_ytm32_init(const struct device *dev)
{
	const struct clock_control_ytm32_config *cfg = dev->config;
	int ret;

	if (cfg->core_clock != YTM32_FIRC_HZ) {
		LOG_ERR("Unsupported core clock %u Hz, MVP supports fixed %u Hz FIRC only",
			cfg->core_clock, YTM32_FIRC_HZ);
		return -EINVAL;
	}

	if (cfg->fast_bus_divider != 1U) {
		LOG_ERR("Unsupported fast bus divider %u, MVP-3 keeps fast bus fixed at 1",
			cfg->fast_bus_divider);
		return -EINVAL;
	}

	if (cfg->core_divider != 1U) {
		LOG_ERR("Unsupported core divider %u, MVP profile keeps core divider fixed at 1",
			cfg->core_divider);
		return -EINVAL;
	}

	if (cfg->slow_bus_divider == 0U) {
		LOG_ERR("Missing YTM32 slow bus divider in DTS");
		return -EINVAL;
	}

	ret = ytm32_soc_apply_clock_config(cfg->core_clock,
				  cfg->core_divider,
				  cfg->fast_bus_divider,
				  cfg->slow_bus_divider);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("YTM32 CGU Initialized, Target Core Clock: %u Hz", cfg->core_clock);
	LOG_INF("Core Divider: %u, Fast Bus Divider: %u, Slow Bus Divider: %u",
		cfg->core_divider, cfg->fast_bus_divider, cfg->slow_bus_divider);
	LOG_INF("YTM32 clock MVP locked to FIRC baseline without PLL");
	LOG_INF("YTM32 low-power policy: SIRC deepsleep=%s standby=%s",
		YTM32_SIRC_DEEPSLEEP_ENABLED ? "on" : "off",
		YTM32_SIRC_STANDBY_ENABLED ? "on" : "off");
	LOG_INF("YTM32 CMU monitor: %s%s",
		YTM32_CMU_ENABLED ? "enabled" : "disabled",
		YTM32_CMU_ENABLED ? (YTM32_CMU_RESET_ENABLED ? " (reset on fault)"
							       : " (no reset on fault)") : "");

	if (YTM32_CMU_ENABLED &&
	    (!YTM32_SIRC_DEEPSLEEP_ENABLED || !YTM32_SIRC_STANDBY_ENABLED)) {
		LOG_WRN("CMU uses SIRC as reference clock; validate low-power transitions on the target board");
	}

	ret = ytm32_refresh_system_rates(dev);
	if (ret < 0) {
		return ret;
	}

	ret = ytm32_log_system_rate(YTM32_CLOCK_CORE, "core");
	if (ret < 0) {
		return ret;
	}

	ret = ytm32_log_system_rate(YTM32_CLOCK_FAST_BUS, "fast bus");
	if (ret < 0) {
		return ret;
	}

	ret = ytm32_log_system_rate(YTM32_CLOCK_SLOW_BUS, "slow bus");
	if (ret < 0) {
		return ret;
	}

	return 0;
}

#define YTM32_CGU_INIT(n) \
	static const struct clock_control_ytm32_config clock_control_ytm32_config_##n = { \
		.core_clock = DT_INST_PROP(n, core_clock), \
		.core_divider = DT_INST_PROP(n, core_divider), \
		.fast_bus_divider = YTM32_DT_FAST_BUS_DIVIDER(n), \
		.slow_bus_divider = YTM32_DT_SLOW_BUS_DIVIDER(n), \
	}; \
	static struct clock_control_ytm32_data clock_control_ytm32_data_##n; \
	DEVICE_DT_INST_DEFINE(n, clock_control_ytm32_init, NULL, &clock_control_ytm32_data_##n, &clock_control_ytm32_config_##n, \
			      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY, \
			      &clock_control_ytm32_api);

DT_INST_FOREACH_STATUS_OKAY(YTM32_CGU_INIT)
