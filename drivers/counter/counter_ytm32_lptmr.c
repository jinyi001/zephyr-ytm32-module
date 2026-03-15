/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ytmicro_ytm32_lptmr

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/dt-bindings/clock/ytmicro,ytm32-clock.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/sys_io.h>

#define YTM32_LPTMR_BASE 0x4005D000U

#define YTM32_LPTMR_CTRL_OFFSET 0x00U
#define YTM32_LPTMR_PRS_OFFSET  0x04U
#define YTM32_LPTMR_DIE_OFFSET  0x08U
#define YTM32_LPTMR_STS_OFFSET  0x0CU
#define YTM32_LPTMR_CMP_OFFSET  0x10U
#define YTM32_LPTMR_LCNT_OFFSET 0x14U
#define YTM32_LPTMR_CNT_OFFSET  0x18U

#define YTM32_LPTMR_CTRL_TMODE BIT(2)
#define YTM32_LPTMR_CTRL_MODE  BIT(1)
#define YTM32_LPTMR_CTRL_EN    BIT(0)

#define YTM32_LPTMR_PRS_PRES_MASK   GENMASK(6, 3)
#define YTM32_LPTMR_PRS_BYPASS      BIT(2)
#define YTM32_LPTMR_PRS_CLKSEL_MASK GENMASK(1, 0)

#define YTM32_LPTMR_DIE_IE  BIT(0)
#define YTM32_LPTMR_STS_CCF BIT(0)
#define YTM32_LPTMR_CMP_MASK 0xFFFFU
#define YTM32_LPTMR_CNT_MASK 0xFFFFU

#define YTM32_LPTMR_CLOCK_SEL_FIRC  0U
#define YTM32_LPTMR_CLOCK_SEL_SIRC  1U
#define YTM32_LPTMR_CLOCK_SEL_FXOSC 2U
#define YTM32_LPTMR_CLOCK_SEL_LPO   3U

#define YTM32_LPTMR_COUNTER_CHANNELS 1U
#define YTM32_LPTMR_DEFAULT_TOP      YTM32_LPTMR_CMP_MASK

#define YTM32_LPTMR_INSTANCE_VALID(addr) \
	BUILD_ASSERT((uint32_t)(addr) == YTM32_LPTMR_BASE, \
		     "LPTMR reg address does not match LPTMR0")

#define YTM32_LPTMR_CLOCK_SOURCE_VALID(src) \
	(((src) == YTM32_CLOCK_SRC_FIRC) || ((src) == YTM32_CLOCK_SRC_SIRC) || \
	 ((src) == YTM32_CLOCK_SRC_FXOSC) || ((src) == YTM32_CLOCK_SRC_LPO))

#define YTM32_LPTMR_CLOCK_SOURCE_SELECT(src) \
	((src) == YTM32_CLOCK_SRC_FIRC ? YTM32_LPTMR_CLOCK_SEL_FIRC : \
	 ((src) == YTM32_CLOCK_SRC_SIRC ? YTM32_LPTMR_CLOCK_SEL_SIRC : \
	  ((src) == YTM32_CLOCK_SRC_FXOSC ? YTM32_LPTMR_CLOCK_SEL_FXOSC : \
	   YTM32_LPTMR_CLOCK_SEL_LPO)))

struct ytm32_lptmr_config {
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

struct ytm32_lptmr_data {
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

static ALWAYS_INLINE uint32_t ytm32_lptmr_read(uintptr_t base, uint32_t offset)
{
	return sys_read32(base + offset);
}

static ALWAYS_INLINE void ytm32_lptmr_write(uintptr_t base, uint32_t offset,
					    uint32_t value)
{
	sys_write32(value, base + offset);
}

static ALWAYS_INLINE void ytm32_lptmr_write_mask(uintptr_t base, uint32_t offset,
						  uint32_t mask,
						  uint32_t value)
{
	uint32_t reg = ytm32_lptmr_read(base, offset);

	reg &= ~mask;
	reg |= (value & mask);
	ytm32_lptmr_write(base, offset, reg);
}

static ALWAYS_INLINE bool ytm32_lptmr_uses_restart_mode(
	const struct ytm32_lptmr_data *data)
{
	return (data->top != YTM32_LPTMR_DEFAULT_TOP) ||
	       (data->top_callback != NULL);
}

static ALWAYS_INLINE bool ytm32_lptmr_irq_required(
	const struct ytm32_lptmr_data *data)
{
	return (data->top_callback != NULL) || data->alarm_active;
}

static ALWAYS_INLINE void ytm32_lptmr_irq_set_pending(unsigned int irqn)
{
	NVIC_SetPendingIRQ((IRQn_Type)irqn);
}

static ALWAYS_INLINE void ytm32_lptmr_irq_clear_pending(unsigned int irqn)
{
	NVIC_ClearPendingIRQ((IRQn_Type)irqn);
}

static ALWAYS_INLINE bool ytm32_lptmr_irq_is_pending(unsigned int irqn)
{
	return NVIC_GetPendingIRQ((IRQn_Type)irqn) != 0U;
}

static ALWAYS_INLINE void ytm32_lptmr_set_enable(uintptr_t base, bool enable)
{
	ytm32_lptmr_write_mask(base, YTM32_LPTMR_CTRL_OFFSET, YTM32_LPTMR_CTRL_EN,
			       enable ? YTM32_LPTMR_CTRL_EN : 0U);
}

static ALWAYS_INLINE void ytm32_lptmr_set_free_running(uintptr_t base, bool enable)
{
	ytm32_lptmr_write_mask(base, YTM32_LPTMR_CTRL_OFFSET,
			       YTM32_LPTMR_CTRL_TMODE,
			       enable ? YTM32_LPTMR_CTRL_TMODE : 0U);
}

static ALWAYS_INLINE void ytm32_lptmr_set_timer_mode(uintptr_t base)
{
	ytm32_lptmr_write_mask(base, YTM32_LPTMR_CTRL_OFFSET,
			       YTM32_LPTMR_CTRL_MODE, 0U);
}

static ALWAYS_INLINE bool ytm32_lptmr_interrupt_enabled(uintptr_t base)
{
	return (ytm32_lptmr_read(base, YTM32_LPTMR_DIE_OFFSET) &
		YTM32_LPTMR_DIE_IE) != 0U;
}

static ALWAYS_INLINE void ytm32_lptmr_set_interrupt(uintptr_t base, bool enable)
{
	ytm32_lptmr_write_mask(base, YTM32_LPTMR_DIE_OFFSET, YTM32_LPTMR_DIE_IE,
			       enable ? YTM32_LPTMR_DIE_IE : 0U);
}

static ALWAYS_INLINE bool ytm32_lptmr_compare_flag_get(uintptr_t base)
{
	return (ytm32_lptmr_read(base, YTM32_LPTMR_STS_OFFSET) &
		YTM32_LPTMR_STS_CCF) != 0U;
}

static ALWAYS_INLINE void ytm32_lptmr_compare_flag_clear(uintptr_t base)
{
	ytm32_lptmr_write(base, YTM32_LPTMR_STS_OFFSET,
			  ytm32_lptmr_read(base, YTM32_LPTMR_STS_OFFSET) |
			  YTM32_LPTMR_STS_CCF);
}

static ALWAYS_INLINE void ytm32_lptmr_set_prescaler(uintptr_t base, uint32_t value)
{
	ytm32_lptmr_write_mask(base, YTM32_LPTMR_PRS_OFFSET,
			       YTM32_LPTMR_PRS_PRES_MASK,
			       FIELD_PREP(YTM32_LPTMR_PRS_PRES_MASK, value));
}

static ALWAYS_INLINE void ytm32_lptmr_set_bypass(uintptr_t base, bool enable)
{
	ytm32_lptmr_write_mask(base, YTM32_LPTMR_PRS_OFFSET,
			       YTM32_LPTMR_PRS_BYPASS,
			       enable ? YTM32_LPTMR_PRS_BYPASS : 0U);
}

static ALWAYS_INLINE void ytm32_lptmr_set_clock_source(uintptr_t base, uint32_t source)
{
	ytm32_lptmr_write_mask(base, YTM32_LPTMR_PRS_OFFSET,
			       YTM32_LPTMR_PRS_CLKSEL_MASK,
			       FIELD_PREP(YTM32_LPTMR_PRS_CLKSEL_MASK, source));
}

static ALWAYS_INLINE void ytm32_lptmr_set_compare(uintptr_t base, uint32_t value)
{
	ytm32_lptmr_write(base, YTM32_LPTMR_CMP_OFFSET, value & YTM32_LPTMR_CMP_MASK);
}

static ALWAYS_INLINE uint32_t ytm32_lptmr_get_counter(uintptr_t base)
{
	ytm32_lptmr_write(base, YTM32_LPTMR_LCNT_OFFSET, 0U);

	return ytm32_lptmr_read(base, YTM32_LPTMR_CNT_OFFSET) & YTM32_LPTMR_CNT_MASK;
}

static void ytm32_lptmr_program(const struct device *dev, bool running_after)
{
	const struct ytm32_lptmr_config *config = dev->config;
	struct ytm32_lptmr_data *data = dev->data;
	uintptr_t base = config->base;

	ytm32_lptmr_set_enable(base, false);
	ytm32_lptmr_write(base, YTM32_LPTMR_CTRL_OFFSET, 0U);
	ytm32_lptmr_write(base, YTM32_LPTMR_STS_OFFSET, YTM32_LPTMR_STS_CCF);
	ytm32_lptmr_write(base, YTM32_LPTMR_DIE_OFFSET, 0U);
	ytm32_lptmr_write(base, YTM32_LPTMR_PRS_OFFSET, 0U);
	ytm32_lptmr_write(base, YTM32_LPTMR_CMP_OFFSET, 0U);

	ytm32_lptmr_set_timer_mode(base);
	ytm32_lptmr_set_free_running(base, !ytm32_lptmr_uses_restart_mode(data));
	ytm32_lptmr_set_prescaler(base, config->prescaler_val);
	ytm32_lptmr_set_bypass(base, config->bypass_prescaler);
	ytm32_lptmr_set_clock_source(base, config->clock_sel);
	ytm32_lptmr_set_compare(base, data->top);
	ytm32_lptmr_set_interrupt(base, ytm32_lptmr_irq_required(data));

	if (running_after) {
		ytm32_lptmr_set_enable(base, true);
	}

	data->running = running_after;
	data->sw_irq_pending = false;
	ytm32_lptmr_irq_clear_pending(config->irqn);
}

static int ytm32_lptmr_start(const struct device *dev)
{
	const struct ytm32_lptmr_config *config = dev->config;
	struct ytm32_lptmr_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if (!data->running) {
		ytm32_lptmr_compare_flag_clear(config->base);
		ytm32_lptmr_set_interrupt(config->base,
					 ytm32_lptmr_irq_required(data));
		ytm32_lptmr_set_enable(config->base, true);
		data->running = true;
	}

	k_spin_unlock(&data->lock, key);

	if (data->alarm_active && data->sw_irq_pending) {
		ytm32_lptmr_irq_set_pending(config->irqn);
	}

	return 0;
}

static int ytm32_lptmr_stop(const struct device *dev)
{
	const struct ytm32_lptmr_config *config = dev->config;
	struct ytm32_lptmr_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	ytm32_lptmr_set_interrupt(config->base, false);
	ytm32_lptmr_set_enable(config->base, false);
	ytm32_lptmr_compare_flag_clear(config->base);
	ytm32_lptmr_irq_clear_pending(config->irqn);

	data->alarm_active = false;
	data->alarm_on_top = false;
	data->alarm_callback = NULL;
	data->alarm_user_data = NULL;
	data->sw_irq_pending = false;
	data->running = false;

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int ytm32_lptmr_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct ytm32_lptmr_config *config = dev->config;

	*ticks = ytm32_lptmr_get_counter(config->base);

	return 0;
}

static int ytm32_lptmr_set_alarm(const struct device *dev, uint8_t chan_id,
					 const struct counter_alarm_cfg *alarm_cfg)
{
	const struct ytm32_lptmr_config *config = dev->config;
	struct ytm32_lptmr_data *data = dev->data;
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

	if (ytm32_lptmr_uses_restart_mode(data)) {
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
			ytm32_lptmr_set_interrupt(config->base, true);
		}

		k_spin_unlock(&data->lock, key);
		return 0;
	}

	now = ytm32_lptmr_get_counter(config->base);
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

	ytm32_lptmr_compare_flag_clear(config->base);
	ytm32_lptmr_set_compare(config->base, target);
	if (data->running) {
		ytm32_lptmr_set_interrupt(config->base, true);
	}

	k_spin_unlock(&data->lock, key);

	if (immediate) {
		ytm32_lptmr_irq_set_pending(config->irqn);
	}

	return 0;
}

static int ytm32_lptmr_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	const struct ytm32_lptmr_config *config = dev->config;
	struct ytm32_lptmr_data *data = dev->data;
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
	ytm32_lptmr_irq_clear_pending(config->irqn);

	if (ytm32_lptmr_uses_restart_mode(data)) {
		ytm32_lptmr_set_interrupt(config->base,
					 data->running &&
					 (data->top_callback != NULL));
	} else {
		ytm32_lptmr_set_interrupt(config->base, false);
		ytm32_lptmr_compare_flag_clear(config->base);
		ytm32_lptmr_set_compare(config->base, data->top);
	}

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int ytm32_lptmr_set_top_value(const struct device *dev,
					 const struct counter_top_cfg *cfg)
{
	struct ytm32_lptmr_data *data = dev->data;
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

	ytm32_lptmr_program(dev, running_after);

	k_spin_unlock(&data->lock, key);

	return 0;
}

static uint32_t ytm32_lptmr_get_pending_int(const struct device *dev)
{
	const struct ytm32_lptmr_config *config = dev->config;
	const struct ytm32_lptmr_data *data = dev->data;

	if (ytm32_lptmr_interrupt_enabled(config->base) &&
	    ytm32_lptmr_compare_flag_get(config->base)) {
		return 1U;
	}

	if (data->sw_irq_pending && ytm32_lptmr_irq_is_pending(config->irqn)) {
		return 1U;
	}

	return 0U;
}

static uint32_t ytm32_lptmr_get_top_value(const struct device *dev)
{
	const struct ytm32_lptmr_data *data = dev->data;

	return data->top;
}

static uint32_t ytm32_lptmr_get_freq(const struct device *dev)
{
	const struct ytm32_lptmr_data *data = dev->data;

	return data->freq;
}

static void ytm32_lptmr_isr(const struct device *dev)
{
	const struct ytm32_lptmr_config *config = dev->config;
	struct ytm32_lptmr_data *data = dev->data;
	counter_alarm_callback_t alarm_cb = NULL;
	counter_top_callback_t top_cb = NULL;
	void *alarm_user_data = NULL;
	void *top_user_data = NULL;
	uint32_t alarm_ticks = 0U;
	bool top_event;
	bool compare_flag;
	k_spinlock_key_t key;

	compare_flag = ytm32_lptmr_compare_flag_get(config->base);
	if (compare_flag) {
		ytm32_lptmr_compare_flag_clear(config->base);
	}

	key = k_spin_lock(&data->lock);
	top_event = ytm32_lptmr_uses_restart_mode(data);

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

	ytm32_lptmr_set_interrupt(config->base, ytm32_lptmr_irq_required(data));

	k_spin_unlock(&data->lock, key);

	if (alarm_cb != NULL) {
		alarm_cb(dev, 0, alarm_ticks, alarm_user_data);
	}

	if (top_cb != NULL) {
		top_cb(dev, top_user_data);
	}
}

static int ytm32_lptmr_init(const struct device *dev)
{
	const struct ytm32_lptmr_config *config = dev->config;
	struct ytm32_lptmr_data *data = dev->data;
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
	ytm32_lptmr_program(dev, false);
	config->irq_config_func(dev);

	return 0;
}

static DEVICE_API(counter, ytm32_lptmr_api) = {
	.start = ytm32_lptmr_start,
	.stop = ytm32_lptmr_stop,
	.get_value = ytm32_lptmr_get_value,
	.set_alarm = ytm32_lptmr_set_alarm,
	.cancel_alarm = ytm32_lptmr_cancel_alarm,
	.set_top_value = ytm32_lptmr_set_top_value,
	.get_pending_int = ytm32_lptmr_get_pending_int,
	.get_top_value = ytm32_lptmr_get_top_value,
	.get_freq = ytm32_lptmr_get_freq,
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
	static void ytm32_lptmr_irq_config_##n(const struct device *dev) \
	{ \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), \
			    ytm32_lptmr_isr, DEVICE_DT_INST_GET(n), 0); \
		irq_enable(DT_INST_IRQN(n)); \
	} \
	static struct ytm32_lptmr_data ytm32_lptmr_data_##n; \
	static const struct ytm32_lptmr_config ytm32_lptmr_config_##n = { \
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
		.irq_config_func = ytm32_lptmr_irq_config_##n, \
		.prescaler_val = YTM32_LPTMR_PRESCALER_VAL(DT_INST_PROP(n, ytmicro_prescaler)), \
		.bypass_prescaler = YTM32_LPTMR_PRESCALER_BYPASS(DT_INST_PROP(n, ytmicro_prescaler)), \
		.prescaler_div = DT_INST_PROP(n, ytmicro_prescaler), \
	}; \
	DEVICE_DT_INST_DEFINE(n, ytm32_lptmr_init, NULL, &ytm32_lptmr_data_##n, \
			      &ytm32_lptmr_config_##n, POST_KERNEL, \
			      CONFIG_COUNTER_INIT_PRIORITY, &ytm32_lptmr_api);

DT_INST_FOREACH_STATUS_OKAY(YTM32_LPTMR_INIT)
