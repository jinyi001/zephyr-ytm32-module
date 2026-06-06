/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ytmicro_ytm32_lptmr

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/dt-bindings/clock/ytmicro,ytm32-soc-clock.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#include "device_registers.h"

/*
 * Register layout (lpTMR_Type) and field masks/shifts come from the vendor HAL
 * device header selected by device_registers.h.  Only the clock-source value
 * encodings and counter limits below are not part of that header.
 */
#define YTM32_LPTMR_CLOCK_SEL_FIRC  0U
#define YTM32_LPTMR_CLOCK_SEL_SIRC  1U
#define YTM32_LPTMR_CLOCK_SEL_FXOSC 2U
#define YTM32_LPTMR_CLOCK_SEL_LPO   3U

#define YTM32_LPTMR_COUNTER_CHANNELS 1U
#define YTM32_LPTMR_DEFAULT_TOP      lpTMR_CMP_CMP_MASK

#define YTM32_LPTMR_INSTANCE_VALID(addr) \
	BUILD_ASSERT((uint32_t)(addr) == lpTMR0_BASE, \
		     "LPTMR reg address does not match lpTMR0")

#if defined(YTM32_CLOCK_SRC_LPO)
#define YTM32_LPTMR_CLOCK_SOURCE_VALID(src) \
	(((src) == YTM32_CLOCK_SRC_FIRC) || ((src) == YTM32_CLOCK_SRC_SIRC) || \
	 ((src) == YTM32_CLOCK_SRC_FXOSC) || ((src) == YTM32_CLOCK_SRC_LPO))

#define YTM32_LPTMR_CLOCK_SOURCE_SELECT(src) \
	((src) == YTM32_CLOCK_SRC_FIRC ? YTM32_LPTMR_CLOCK_SEL_FIRC : \
	 ((src) == YTM32_CLOCK_SRC_SIRC ? YTM32_LPTMR_CLOCK_SEL_SIRC : \
	  ((src) == YTM32_CLOCK_SRC_FXOSC ? YTM32_LPTMR_CLOCK_SEL_FXOSC : \
	   YTM32_LPTMR_CLOCK_SEL_LPO)))
#else
#define YTM32_LPTMR_CLOCK_SOURCE_VALID(src) \
	(((src) == YTM32_CLOCK_SRC_FIRC) || ((src) == YTM32_CLOCK_SRC_SIRC) || \
	 ((src) == YTM32_CLOCK_SRC_FXOSC))

#define YTM32_LPTMR_CLOCK_SOURCE_SELECT(src) \
	((src) == YTM32_CLOCK_SRC_FIRC ? YTM32_LPTMR_CLOCK_SEL_FIRC : \
	 ((src) == YTM32_CLOCK_SRC_SIRC ? YTM32_LPTMR_CLOCK_SEL_SIRC : \
	  YTM32_LPTMR_CLOCK_SEL_FXOSC))
#endif

struct counter_ytm32_lptmr_config {
	struct counter_config_info info;
	uintptr_t base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	uint32_t clock_sel;
	uint32_t irqn;
	void (*irq_config_func)(const struct device *dev);
	uint32_t prescaler_val;
	bool bypass_prescaler;
	uint32_t prescaler_div;
};

struct counter_ytm32_lptmr_data {
	struct k_spinlock lock;
	uint32_t freq;
	uint32_t top;
	counter_top_callback_t top_callback;
	void *top_user_data;
	counter_alarm_callback_t alarm_callback;
	void *alarm_user_data;
	uint32_t alarm_target;
	bool alarm_active;
	bool alarm_on_top;
	bool running;
	bool sw_irq_pending;
};

static ALWAYS_INLINE lpTMR_Type *counter_ytm32_lptmr_regs(uintptr_t base)
{
	return (lpTMR_Type *)base;
}

static ALWAYS_INLINE void counter_ytm32_lptmr_reg_modify(volatile uint32_t *reg,
						 uint32_t mask, uint32_t value)
{
	uint32_t r = *reg;

	r &= ~mask;
	r |= (value & mask);
	*reg = r;
}

static ALWAYS_INLINE bool counter_ytm32_lptmr_uses_restart_mode(
	const struct counter_ytm32_lptmr_data *data)
{
	return (data->top != YTM32_LPTMR_DEFAULT_TOP) ||
	       (data->top_callback != NULL);
}

static ALWAYS_INLINE bool counter_ytm32_lptmr_irq_required(
	const struct counter_ytm32_lptmr_data *data)
{
	return (data->top_callback != NULL) || data->alarm_active;
}

static ALWAYS_INLINE void counter_ytm32_lptmr_irq_set_pending(unsigned int irqn)
{
	NVIC_SetPendingIRQ((IRQn_Type)irqn);
}

static ALWAYS_INLINE void counter_ytm32_lptmr_irq_clear_pending(unsigned int irqn)
{
	NVIC_ClearPendingIRQ((IRQn_Type)irqn);
}

static ALWAYS_INLINE bool counter_ytm32_lptmr_irq_is_pending(unsigned int irqn)
{
	return NVIC_GetPendingIRQ((IRQn_Type)irqn) != 0U;
}

static ALWAYS_INLINE void counter_ytm32_lptmr_set_enable(uintptr_t base, bool enable)
{
	counter_ytm32_lptmr_reg_modify(&counter_ytm32_lptmr_regs(base)->CTRL, lpTMR_CTRL_EN_MASK,
			       enable ? lpTMR_CTRL_EN_MASK : 0U);
}

static ALWAYS_INLINE void counter_ytm32_lptmr_set_free_running(uintptr_t base, bool enable)
{
	counter_ytm32_lptmr_reg_modify(&counter_ytm32_lptmr_regs(base)->CTRL, lpTMR_CTRL_TMODE_MASK,
			       enable ? lpTMR_CTRL_TMODE_MASK : 0U);
}

static ALWAYS_INLINE void counter_ytm32_lptmr_set_timer_mode(uintptr_t base)
{
	counter_ytm32_lptmr_reg_modify(&counter_ytm32_lptmr_regs(base)->CTRL, lpTMR_CTRL_MODE_MASK,
			       0U);
}

static ALWAYS_INLINE bool counter_ytm32_lptmr_interrupt_enabled(uintptr_t base)
{
	return (counter_ytm32_lptmr_regs(base)->DIE & lpTMR_DIE_IE_MASK) != 0U;
}

static ALWAYS_INLINE void counter_ytm32_lptmr_set_interrupt(uintptr_t base, bool enable)
{
	counter_ytm32_lptmr_reg_modify(&counter_ytm32_lptmr_regs(base)->DIE, lpTMR_DIE_IE_MASK,
			       enable ? lpTMR_DIE_IE_MASK : 0U);
}

static ALWAYS_INLINE bool counter_ytm32_lptmr_compare_flag_get(uintptr_t base)
{
	return (counter_ytm32_lptmr_regs(base)->STS & lpTMR_STS_CCF_MASK) != 0U;
}

static ALWAYS_INLINE void counter_ytm32_lptmr_compare_flag_clear(uintptr_t base)
{
	lpTMR_Type *regs = counter_ytm32_lptmr_regs(base);

	regs->STS = regs->STS | lpTMR_STS_CCF_MASK;
}

static ALWAYS_INLINE void counter_ytm32_lptmr_set_prescaler(uintptr_t base, uint32_t value)
{
	counter_ytm32_lptmr_reg_modify(&counter_ytm32_lptmr_regs(base)->PRS, lpTMR_PRS_PRES_MASK,
			       FIELD_PREP(lpTMR_PRS_PRES_MASK, value));
}

static ALWAYS_INLINE void counter_ytm32_lptmr_set_bypass(uintptr_t base, bool enable)
{
	counter_ytm32_lptmr_reg_modify(&counter_ytm32_lptmr_regs(base)->PRS, lpTMR_PRS_BYPASS_MASK,
			       enable ? lpTMR_PRS_BYPASS_MASK : 0U);
}

static ALWAYS_INLINE void counter_ytm32_lptmr_set_clock_source(uintptr_t base, uint32_t source)
{
	counter_ytm32_lptmr_reg_modify(&counter_ytm32_lptmr_regs(base)->PRS, lpTMR_PRS_CLKSEL_MASK,
			       FIELD_PREP(lpTMR_PRS_CLKSEL_MASK, source));
}

static ALWAYS_INLINE void counter_ytm32_lptmr_set_compare(uintptr_t base, uint32_t value)
{
	counter_ytm32_lptmr_regs(base)->CMP = value & lpTMR_CMP_CMP_MASK;
}

static ALWAYS_INLINE uint32_t counter_ytm32_lptmr_get_counter(uintptr_t base)
{
	lpTMR_Type *regs = counter_ytm32_lptmr_regs(base);

	regs->LCNT = 0U;

	return regs->CNT & lpTMR_CNT_CVAL_MASK;
}

static void counter_ytm32_lptmr_program(const struct device *dev, bool running_after)
{
	const struct counter_ytm32_lptmr_config *config = dev->config;
	struct counter_ytm32_lptmr_data *data = dev->data;
	uintptr_t base = config->base;
	lpTMR_Type *regs = counter_ytm32_lptmr_regs(base);

	counter_ytm32_lptmr_set_enable(base, false);
	regs->CTRL = 0U;
	regs->STS  = lpTMR_STS_CCF_MASK;
	regs->DIE  = 0U;
	regs->PRS  = 0U;
	regs->CMP  = 0U;

	counter_ytm32_lptmr_set_timer_mode(base);
	counter_ytm32_lptmr_set_free_running(base, !counter_ytm32_lptmr_uses_restart_mode(data));
	counter_ytm32_lptmr_set_prescaler(base, config->prescaler_val);
	counter_ytm32_lptmr_set_bypass(base, config->bypass_prescaler);
	counter_ytm32_lptmr_set_clock_source(base, config->clock_sel);
	counter_ytm32_lptmr_set_compare(base, data->top);
	counter_ytm32_lptmr_set_interrupt(base, counter_ytm32_lptmr_irq_required(data));

	if (running_after) {
		counter_ytm32_lptmr_set_enable(base, true);
	}

	data->running = running_after;
	data->sw_irq_pending = false;
	counter_ytm32_lptmr_irq_clear_pending(config->irqn);
}

static int counter_ytm32_lptmr_start(const struct device *dev)
{
	const struct counter_ytm32_lptmr_config *config = dev->config;
	struct counter_ytm32_lptmr_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if (!data->running) {
		counter_ytm32_lptmr_compare_flag_clear(config->base);
		counter_ytm32_lptmr_set_interrupt(config->base,
					 counter_ytm32_lptmr_irq_required(data));
		counter_ytm32_lptmr_set_enable(config->base, true);
		data->running = true;
	}

	k_spin_unlock(&data->lock, key);

	if (data->alarm_active && data->sw_irq_pending) {
		counter_ytm32_lptmr_irq_set_pending(config->irqn);
	}

	return 0;
}

static int counter_ytm32_lptmr_stop(const struct device *dev)
{
	const struct counter_ytm32_lptmr_config *config = dev->config;
	struct counter_ytm32_lptmr_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	counter_ytm32_lptmr_set_interrupt(config->base, false);
	counter_ytm32_lptmr_set_enable(config->base, false);
	counter_ytm32_lptmr_compare_flag_clear(config->base);
	counter_ytm32_lptmr_irq_clear_pending(config->irqn);

	data->alarm_active = false;
	data->alarm_on_top = false;
	data->alarm_callback = NULL;
	data->alarm_user_data = NULL;
	data->sw_irq_pending = false;
	data->running = false;

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int counter_ytm32_lptmr_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_ytm32_lptmr_config *config = dev->config;

	*ticks = counter_ytm32_lptmr_get_counter(config->base);

	return 0;
}

static int counter_ytm32_lptmr_set_alarm(const struct device *dev, uint8_t chan_id,
					 const struct counter_alarm_cfg *alarm_cfg)
{
	const struct counter_ytm32_lptmr_config *config = dev->config;
	struct counter_ytm32_lptmr_data *data = dev->data;
	uint32_t top = data->top;
	uint32_t now;
	uint32_t target;
	bool absolute;
	bool immediate = false;
	k_spinlock_key_t key;

	ARG_UNUSED(chan_id);

	if ((alarm_cfg == NULL) || (alarm_cfg->callback == NULL)) {
		return -EINVAL;
	}

	if (alarm_cfg->ticks > top) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);

	if (data->alarm_active) {
		k_spin_unlock(&data->lock, key);
		return -EBUSY;
	}

	if (counter_ytm32_lptmr_uses_restart_mode(data)) {
		if (alarm_cfg->ticks != data->top) {
			k_spin_unlock(&data->lock, key);
			return -ENOTSUP;
		}

		data->alarm_callback = alarm_cfg->callback;
		data->alarm_user_data = alarm_cfg->user_data;
		data->alarm_target = data->top;
		data->alarm_active = true;
		data->alarm_on_top = true;
		data->sw_irq_pending = false;

		if (data->running) {
			counter_ytm32_lptmr_set_interrupt(config->base, true);
		}

		k_spin_unlock(&data->lock, key);
		return 0;
	}

	now = counter_ytm32_lptmr_get_counter(config->base);
	absolute = (alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) != 0U;
	target = absolute ? alarm_cfg->ticks :
		((now + alarm_cfg->ticks) % (data->top + 1U));
	immediate = !absolute && (alarm_cfg->ticks == 0U) && data->running;

	data->alarm_callback = alarm_cfg->callback;
	data->alarm_user_data = alarm_cfg->user_data;
	data->alarm_target = target;
	data->alarm_active = true;
	data->alarm_on_top = false;
	data->sw_irq_pending = immediate;

	counter_ytm32_lptmr_compare_flag_clear(config->base);
	counter_ytm32_lptmr_set_compare(config->base, target);
	if (data->running) {
		counter_ytm32_lptmr_set_interrupt(config->base, true);
	}

	k_spin_unlock(&data->lock, key);

	if (immediate) {
		counter_ytm32_lptmr_irq_set_pending(config->irqn);
	}

	return 0;
}

static int counter_ytm32_lptmr_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	const struct counter_ytm32_lptmr_config *config = dev->config;
	struct counter_ytm32_lptmr_data *data = dev->data;
	k_spinlock_key_t key;

	ARG_UNUSED(chan_id);

	key = k_spin_lock(&data->lock);

	if (!data->alarm_active) {
		k_spin_unlock(&data->lock, key);
		return 0;
	}

	data->alarm_active = false;
	data->alarm_on_top = false;
	data->alarm_callback = NULL;
	data->alarm_user_data = NULL;
	data->sw_irq_pending = false;
	counter_ytm32_lptmr_irq_clear_pending(config->irqn);

	if (counter_ytm32_lptmr_uses_restart_mode(data)) {
		counter_ytm32_lptmr_set_interrupt(config->base,
					 data->running &&
					 (data->top_callback != NULL));
	} else {
		counter_ytm32_lptmr_set_interrupt(config->base, false);
		counter_ytm32_lptmr_compare_flag_clear(config->base);
		counter_ytm32_lptmr_set_compare(config->base, data->top);
	}

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int counter_ytm32_lptmr_set_top_value(const struct device *dev,
					 const struct counter_top_cfg *cfg)
{
	struct counter_ytm32_lptmr_data *data = dev->data;
	k_spinlock_key_t key;
	bool running_after;

	if ((cfg == NULL) || (cfg->ticks == 0U)) {
		return -EINVAL;
	}

	if ((cfg->flags & COUNTER_TOP_CFG_DONT_RESET) != 0U) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->lock);

	if (data->alarm_active) {
		k_spin_unlock(&data->lock, key);
		return -EBUSY;
	}

	data->top = cfg->ticks;
	data->top_callback = cfg->callback;
	data->top_user_data = cfg->user_data;
	running_after = data->running;

	counter_ytm32_lptmr_program(dev, running_after);

	k_spin_unlock(&data->lock, key);

	return 0;
}

static uint32_t counter_ytm32_lptmr_get_pending_int(const struct device *dev)
{
	const struct counter_ytm32_lptmr_config *config = dev->config;
	const struct counter_ytm32_lptmr_data *data = dev->data;

	if (counter_ytm32_lptmr_interrupt_enabled(config->base) &&
	    counter_ytm32_lptmr_compare_flag_get(config->base)) {
		return 1U;
	}

	if (data->sw_irq_pending && counter_ytm32_lptmr_irq_is_pending(config->irqn)) {
		return 1U;
	}

	return 0U;
}

static uint32_t counter_ytm32_lptmr_get_top_value(const struct device *dev)
{
	const struct counter_ytm32_lptmr_data *data = dev->data;

	return data->top;
}

static uint32_t counter_ytm32_lptmr_get_freq(const struct device *dev)
{
	const struct counter_ytm32_lptmr_data *data = dev->data;

	return data->freq;
}

static void counter_ytm32_lptmr_isr(const struct device *dev)
{
	const struct counter_ytm32_lptmr_config *config = dev->config;
	struct counter_ytm32_lptmr_data *data = dev->data;
	counter_alarm_callback_t alarm_cb = NULL;
	counter_top_callback_t top_cb = NULL;
	void *alarm_user_data = NULL;
	void *top_user_data = NULL;
	uint32_t alarm_ticks = 0U;
	bool top_event;
	bool compare_flag;
	k_spinlock_key_t key;

	compare_flag = counter_ytm32_lptmr_compare_flag_get(config->base);
	if (compare_flag) {
		counter_ytm32_lptmr_compare_flag_clear(config->base);
	}

	key = k_spin_lock(&data->lock);
	top_event = counter_ytm32_lptmr_uses_restart_mode(data);

	if (data->alarm_active && (!top_event || data->alarm_on_top || data->sw_irq_pending)) {
		alarm_cb = data->alarm_callback;
		alarm_user_data = data->alarm_user_data;
		alarm_ticks = data->alarm_on_top ? data->top : data->alarm_target;
		data->alarm_active = false;
		data->alarm_on_top = false;
		data->alarm_callback = NULL;
		data->alarm_user_data = NULL;
		data->sw_irq_pending = false;
	}

	if (top_event && (data->top_callback != NULL)) {
		top_cb = data->top_callback;
		top_user_data = data->top_user_data;
	}

	counter_ytm32_lptmr_set_interrupt(config->base, counter_ytm32_lptmr_irq_required(data));

	k_spin_unlock(&data->lock, key);

	if (alarm_cb != NULL) {
		alarm_cb(dev, 0, alarm_ticks, alarm_user_data);
	}

	if (top_cb != NULL) {
		top_cb(dev, top_user_data);
	}
}

static int counter_ytm32_lptmr_init(const struct device *dev)
{
	const struct counter_ytm32_lptmr_config *config = dev->config;
	struct counter_ytm32_lptmr_data *data = dev->data;
	int ret;

	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_get_rate(config->clock_dev, config->clock_subsys,
				     &data->freq);
	if ((ret < 0) || (data->freq == 0U)) {
		return ret < 0 ? ret : -EINVAL;
	}

	data->freq /= config->prescaler_div;

	data->top = YTM32_LPTMR_DEFAULT_TOP;
	counter_ytm32_lptmr_program(dev, false);
	config->irq_config_func(dev);

	return 0;
}

static DEVICE_API(counter, counter_ytm32_lptmr_api) = {
	.start = counter_ytm32_lptmr_start,
	.stop = counter_ytm32_lptmr_stop,
	.get_value = counter_ytm32_lptmr_get_value,
	.set_alarm = counter_ytm32_lptmr_set_alarm,
	.cancel_alarm = counter_ytm32_lptmr_cancel_alarm,
	.set_top_value = counter_ytm32_lptmr_set_top_value,
	.get_pending_int = counter_ytm32_lptmr_get_pending_int,
	.get_top_value = counter_ytm32_lptmr_get_top_value,
	.get_freq = counter_ytm32_lptmr_get_freq,
};

#define YTM32_LPTMR_PRESCALER_VAL(p) \
	((p) == 1 ? 0 : \
	 (p) == 2 ? 0 : \
	 (p) == 4 ? 1 : \
	 (p) == 8 ? 2 : \
	 (p) == 16 ? 3 : \
	 (p) == 32 ? 4 : \
	 (p) == 64 ? 5 : \
	 (p) == 128 ? 6 : \
	 (p) == 256 ? 7 : \
	 (p) == 512 ? 8 : \
	 (p) == 1024 ? 9 : \
	 (p) == 2048 ? 10 : \
	 (p) == 4096 ? 11 : \
	 (p) == 8192 ? 12 : \
	 (p) == 16384 ? 13 : \
	 (p) == 32768 ? 14 : 15)

#define YTM32_LPTMR_PRESCALER_BYPASS(p) ((p) == 1)

#define YTM32_LPTMR_INIT(n) \
	YTM32_LPTMR_INSTANCE_VALID(DT_INST_REG_ADDR(n)); \
	BUILD_ASSERT(YTM32_LPTMR_CLOCK_SOURCE_VALID( \
		DT_INST_PROP(n, ytmicro_functional_clock_source)), \
		"Unsupported YTM32 LPTMR functional clock source"); \
	static void counter_ytm32_lptmr_irq_config_##n(const struct device *dev) \
	{ \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), \
			    counter_ytm32_lptmr_isr, DEVICE_DT_INST_GET(n), 0); \
		irq_enable(DT_INST_IRQN(n)); \
	} \
	static struct counter_ytm32_lptmr_data counter_ytm32_lptmr_data_##n; \
	static const struct counter_ytm32_lptmr_config counter_ytm32_lptmr_config_##n = { \
		.info = { \
			.max_top_value = YTM32_LPTMR_DEFAULT_TOP, \
			.flags = COUNTER_CONFIG_INFO_COUNT_UP, \
			.channels = YTM32_LPTMR_COUNTER_CHANNELS, \
		}, \
		.base = DT_INST_REG_ADDR(n), \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)), \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, id), \
		.clock_sel = YTM32_LPTMR_CLOCK_SOURCE_SELECT( \
			DT_INST_PROP(n, ytmicro_functional_clock_source)), \
		.irqn = DT_INST_IRQN(n), \
		.irq_config_func = counter_ytm32_lptmr_irq_config_##n, \
		.prescaler_val = YTM32_LPTMR_PRESCALER_VAL(DT_INST_PROP(n, ytmicro_prescaler)), \
		.bypass_prescaler = YTM32_LPTMR_PRESCALER_BYPASS(DT_INST_PROP(n, ytmicro_prescaler)), \
		.prescaler_div = DT_INST_PROP(n, ytmicro_prescaler), \
	}; \
	DEVICE_DT_INST_DEFINE(n, counter_ytm32_lptmr_init, NULL, &counter_ytm32_lptmr_data_##n, \
			      &counter_ytm32_lptmr_config_##n, POST_KERNEL, \
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_ytm32_lptmr_api);

DT_INST_FOREACH_STATUS_OKAY(YTM32_LPTMR_INIT)
