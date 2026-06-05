/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ytmicro_ytm32_wdg

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include "device_registers.h"

/* Register layout (WDG_Type) and all field masks/shifts come from the vendor
 * HAL device header selected by device_registers.h.  Only the service-code
 * protocol values and timing limits below are not part of that header.
 */
#define YTM32_WDG_UNLOCK_VALUE_1    0xB631U
#define YTM32_WDG_UNLOCK_VALUE_2    0xC278U
#define YTM32_WDG_TRIGGER_VALUE_1   0xA518U
#define YTM32_WDG_TRIGGER_VALUE_2   0xD826U
#define YTM32_WDG_RESET_CR          0x82U
#define YTM32_WDG_RESET_TOVR        0x0C00U
#define YTM32_WDG_RESET_WVR         0x0000U
#define YTM32_WDG_MIN_TIMEOUT_TICKS 0x0100U
#define YTM32_WDG_MAX_TIMEOUT_TICKS 0xFFFFU

#define YTM32_WDG_LPO_CLOCK_HZ      32000U
#define YTM32_WDG_SIRC_CLOCK_HZ     2000000U

#define YTM32_WDG_UNLOCK_TIMEOUT 32U

#define YTM32_WDG_SOURCE_LPO  0U
#define YTM32_WDG_SOURCE_SIRC 1U

#define YTM32_WDG_SUPPORTED_OPTIONS \
	(WDT_OPT_PAUSE_IN_SLEEP | WDT_OPT_PAUSE_HALTED_BY_DBG)

#define YTM32_WDG_INSTANCE_VALID(addr) \
	BUILD_ASSERT((uint32_t)(addr) == WDG0_BASE, \
		     "WDG reg address does not match WDG0")

struct ytm32_wdg_config {
	uintptr_t base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	uint32_t timeout_clock_hz;
	uint32_t timeout_clock_source;
};

struct ytm32_wdg_data {
	uint32_t timeout_ticks;
	bool timeout_valid;
	bool enabled;
};

static inline WDG_Type *ytm32_wdg_regs(const struct device *dev)
{
	const struct ytm32_wdg_config *config = dev->config;

	return (WDG_Type *)config->base;
}

static inline void ytm32_wdg_unlock_regs(WDG_Type *wdg)
{
	wdg->SVCR = YTM32_WDG_UNLOCK_VALUE_1;
	wdg->SVCR = YTM32_WDG_UNLOCK_VALUE_2;
}

static inline void ytm32_wdg_trigger(WDG_Type *wdg)
{
	wdg->SVCR = YTM32_WDG_TRIGGER_VALUE_1;
	wdg->SVCR = YTM32_WDG_TRIGGER_VALUE_2;
}

static inline bool ytm32_wdg_is_enabled(WDG_Type *wdg)
{
	return (wdg->CR & WDG_CR_EN_MASK) != 0U;
}

static inline bool ytm32_wdg_is_unlocked(WDG_Type *wdg)
{
	return (wdg->LR & (WDG_LR_HL_MASK | WDG_LR_SL_MASK)) == 0U;
}

static int ytm32_wdg_wait_unlock(WDG_Type *wdg)
{
	uint32_t attempts = YTM32_WDG_UNLOCK_TIMEOUT;

	do {
		ytm32_wdg_unlock_regs(wdg);
		if (ytm32_wdg_is_unlocked(wdg)) {
			return 0;
		}
	} while (--attempts > 0U);

	return -EIO;
}

static int ytm32_wdg_install_timeout(const struct device *dev,
				     const struct wdt_timeout_cfg *cfg)
{
	const struct ytm32_wdg_config *config = dev->config;
	struct ytm32_wdg_data *data = dev->data;
	uint64_t ticks;

	if (data->enabled) {
		return -EBUSY;
	}

	if (data->timeout_valid) {
		return -ENOMEM;
	}

	if (cfg->callback != NULL) {
		return -ENOTSUP;
	}

	if (cfg->flags != WDT_FLAG_RESET_SOC) {
		return -ENOTSUP;
	}

	if ((cfg->window.min != 0U) || (cfg->window.max == 0U)) {
		return -EINVAL;
	}

	ticks = DIV_ROUND_UP((uint64_t)cfg->window.max * config->timeout_clock_hz, 1000U);
	if ((ticks < YTM32_WDG_MIN_TIMEOUT_TICKS) ||
	    (ticks > YTM32_WDG_MAX_TIMEOUT_TICKS)) {
		return -EINVAL;
	}

	data->timeout_ticks = (uint32_t)ticks;
	data->timeout_valid = true;

	return 0;
}

static int ytm32_wdg_setup(const struct device *dev, uint8_t options)
{
	const struct ytm32_wdg_config *config = dev->config;
	WDG_Type *wdg = ytm32_wdg_regs(dev);
	struct ytm32_wdg_data *data = dev->data;
	uint32_t key;
	uint32_t cr = WDG_CR_EN_MASK;
	int ret;

	if ((options & ~YTM32_WDG_SUPPORTED_OPTIONS) != 0U) {
		return -ENOTSUP;
	}

	if (!data->timeout_valid) {
		return -EINVAL;
	}

	if (data->enabled || ytm32_wdg_is_enabled(wdg)) {
		return -EBUSY;
	}

	cr |= FIELD_PREP(WDG_CR_CLKSRC_MASK, config->timeout_clock_source);

	if ((options & WDT_OPT_PAUSE_IN_SLEEP) != 0U) {
		cr |= WDG_CR_DSDIS_MASK;
	}

	if ((options & WDT_OPT_PAUSE_HALTED_BY_DBG) != 0U) {
		cr |= WDG_CR_DBGDIS_MASK;
	}

	key = irq_lock();
	ret = ytm32_wdg_wait_unlock(wdg);
	if (ret == 0) {
		wdg->TOVR = data->timeout_ticks;
		wdg->WVR  = 0U;
		wdg->INTF = WDG_INTF_IF_MASK;
		wdg->CR   = cr;
		wdg->LR   = WDG_LR_SL_MASK;
		data->enabled = true;
	}
	irq_unlock(key);

	return ret;
}

static int ytm32_wdg_feed(const struct device *dev, int channel_id)
{
	WDG_Type *wdg = ytm32_wdg_regs(dev);
	struct ytm32_wdg_data *data = dev->data;
	uint32_t key;

	if ((channel_id != 0) || !data->timeout_valid || !data->enabled) {
		return -EINVAL;
	}

	key = irq_lock();
	ytm32_wdg_trigger(wdg);
	irq_unlock(key);

	return 0;
}

static int ytm32_wdg_disable(const struct device *dev)
{
	WDG_Type *wdg = ytm32_wdg_regs(dev);
	struct ytm32_wdg_data *data = dev->data;
	uint32_t key;
	int ret;

	if (!data->enabled || !ytm32_wdg_is_enabled(wdg)) {
		return -EFAULT;
	}

	key = irq_lock();
	ret = ytm32_wdg_wait_unlock(wdg);
	if (ret == 0) {
		wdg->CR   = YTM32_WDG_RESET_CR;
		wdg->TOVR = YTM32_WDG_RESET_TOVR;
		wdg->WVR  = YTM32_WDG_RESET_WVR;
		ytm32_wdg_trigger(wdg);
		data->enabled = false;
		data->timeout_valid = false;
	}
	irq_unlock(key);

	return ret;
}

static int ytm32_wdg_init(const struct device *dev)
{
	const struct ytm32_wdg_config *config = dev->config;

	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	return clock_control_on(config->clock_dev, config->clock_subsys);
}

static DEVICE_API(wdt, ytm32_wdg_api) = {
	.setup = ytm32_wdg_setup,
	.disable = ytm32_wdg_disable,
	.install_timeout = ytm32_wdg_install_timeout,
	.feed = ytm32_wdg_feed,
};

#define YTM32_WDG_CLOCK_HZ(source) \
	((source) == YTM32_WDG_SOURCE_SIRC ? YTM32_WDG_SIRC_CLOCK_HZ : YTM32_WDG_LPO_CLOCK_HZ)

#define YTM32_WDG_INIT(n) \
	YTM32_WDG_INSTANCE_VALID(DT_INST_REG_ADDR(n)); \
	BUILD_ASSERT((DT_INST_PROP(n, ytmicro_timeout_clock_source) == YTM32_WDG_SOURCE_LPO) || \
		     (DT_INST_PROP(n, ytmicro_timeout_clock_source) == YTM32_WDG_SOURCE_SIRC), \
		     "Unsupported YTM32 watchdog timeout clock source"); \
	static struct ytm32_wdg_data ytm32_wdg_data_##n; \
	static const struct ytm32_wdg_config ytm32_wdg_config_##n = { \
		.base = DT_INST_REG_ADDR(n), \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)), \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id), \
		.timeout_clock_hz = \
			YTM32_WDG_CLOCK_HZ(DT_INST_PROP(n, ytmicro_timeout_clock_source)), \
		.timeout_clock_source = DT_INST_PROP(n, ytmicro_timeout_clock_source), \
	}; \
	DEVICE_DT_INST_DEFINE(n, ytm32_wdg_init, NULL, &ytm32_wdg_data_##n, \
			      &ytm32_wdg_config_##n, POST_KERNEL, \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ytm32_wdg_api);

DT_INST_FOREACH_STATUS_OKAY(YTM32_WDG_INIT)
