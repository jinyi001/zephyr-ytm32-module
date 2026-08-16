/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ytmicro_ytm32_etmr_pwm

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>

#include "etmr_pwm_driver.h"
#include "etmr_common.h"
#include "pwm_ytm32_etmr_logic.h"

#include <zephyr/drivers/pwm/pwm_ytm32_etmr.h>
#include <zephyr/dt-bindings/clock/ytmicro,ytm32-soc-clock.h>

LOG_MODULE_REGISTER(pwm_ytm32_etmr, CONFIG_PWM_LOG_LEVEL);

/* Number of even channels per instance (covers both even and odd in comp mode) */
#define ETMR_PAIR_COUNT 4U
/* Total physical channels per instance */
#define ETMR_CH_COUNT   8U

struct pwm_ytm32_config {
	uint8_t instance;
	const struct device *clk_dev;
	clock_control_subsys_t clk_sys;
	uint8_t prescaler;
	const struct pinctrl_dev_config *pincfg;
	/* Dead time in ns applied to all complementary pairs */
	uint32_t deadtime_ns;
	/* Initial PWM frequency used during eTMR_DRV_InitPwm */
	uint32_t pwm_freq_hz;
	/* Emit an INIT-match output trigger (drives the TMU eTMR<n>_INIT_TRIG
	 * source for ADC hardware triggering — see SetOutputTrigger below)
	 */
	bool adc_sync_trigger;
	/* Output trigger pulse width in counter clock cycles (DT ytmicro,adc-trigger-width) */
	uint32_t adc_trigger_width;
	/* Start the eTMR counter at driver init (DT ytmicro,autostart) */
	bool autostart;
	/* Use OTRIG.MIDTEN + MID register for mid-period ADC trigger (DT ytmicro,adc-mid-trigger) */
	bool adc_mid_trigger;
	/* Three even phase channels; each owns its following complementary output. */
	uint8_t phase_channels[PWM_YTM32_ETMR_PHASE_COUNT];
	bool phase_channels_present;
	/* Hardware fault inputs and their recovery/safe-state policy. */
	uint8_t fault_channels_mask;
	uint8_t fault_active_low_mask;
	uint8_t fault_filter_count;
	uint8_t fault_filter_period;
	bool fault_input_stretch;
	bool fault_combinational;
	uint8_t fault_recovery;
	uint8_t fault_auto_mode;
};

struct pwm_ytm32_data {
	struct k_mutex lock;
	uint32_t clk_rate;   /* counter clock Hz = functional_clk >> effective_prescaler */
	uint32_t period_cycles; /* last period passed to set_cycles, in counter ticks */
	etmr_state_t etmr_state;
	atomic_t safe_state;
	/* Packed CHMASK state for the commanded 0%/100% endpoints.  This is
	 * distinct from safe_state: safe always forces all six outputs low. */
	atomic_t endpoint_chmask;
	atomic_t counter_running;
	atomic_t phase_config_error;
	atomic_t fault_latched;
	atomic_t fault_armed;
	atomic_t fault_count;
	atomic_t fault_status;
	pwm_ytm32_fault_cb_t fault_cb;
	void *fault_user_data;
	pwm_ytm32_ovf_cb_t ovf_cb;
	void *ovf_user_data;
};

/* ── helpers ─────────────────────────────────────────────────────────── */

/**
 * counter_freq() - 计算 eTMR 计数器的实际工作频率
 *
 * @cfg:  驱动配置结构体指针
 * @data: 驱动运行时数据结构体指针
 *
 * 将从 CGU 获取的功能时钟（clk_rate）右移 prescaler 位，等效于
 * 除以 2^prescaler，得到计数器的滴答频率（Hz）。
 *
 * 返回值：计数器频率（Hz）。
 */
/* Returns counter clock Hz.  data->clk_rate stores the already-divided
 * counter frequency (functional clock >> effective prescaler), so this is
 * a direct read.  The value is set in pwm_ytm32_init after auto-prescaler
 * adjustment. */
static inline uint32_t counter_freq(const struct pwm_ytm32_config *cfg,
				    const struct pwm_ytm32_data *data)
{
	ARG_UNUSED(cfg);
	return data->clk_rate;
}

static inline uint8_t phase_complementary_mask(
	const struct pwm_ytm32_config *cfg)
{
	return pwm_ytm32_etmr_phase_complementary_mask(cfg->phase_channels);
}

/**
 * channel_is_comp() - 判断通道是否属于互补对
 *
 * @cfg: 驱动配置结构体指针
 * @ch:  待查询的通道号（0–7）
 *
 * 将通道号取偶（ch & ~1U），查询由 phase_channels 派生的 mask。
 * 奇数通道（互补输出侧）与其偶数对伴通道共享同一 bit，因此
 * 奇/偶通道均可正确判断是否位于互补对中。
 *
 * 返回值：true 表示通道属于互补对，false 表示独立边沿对齐输出。
 */
static inline bool channel_is_comp(const struct pwm_ytm32_config *cfg, uint32_t ch)
{
	uint32_t even = ch & ~1U;
	return (phase_complementary_mask(cfg) & BIT(even)) != 0U;
}

/**
 * ns_to_ticks() - 将死区时间（纳秒）转换为计数器滴答数
 *
 * @ns:      死区时间，单位纳秒
 * @freq_hz: 计数器时钟频率（Hz），由 counter_freq() 获取
 *
 * 使用 64 位中间结果避免溢出：
 *   ticks = ns × freq_hz / 1_000_000_000
 *
 * 返回值：对应的计数器滴答数，截断为 uint16_t。
 */
static inline uint16_t ns_to_ticks(uint32_t ns, uint32_t freq_hz)
{
	/* freq_hz ticks per second → ticks per ns = freq_hz / 1e9 */
	return (uint16_t)(((uint64_t)ns * freq_hz) / 1000000000ULL);
}

static int pwm_ytm32_etmr_mask_outputs(const struct device *dev,
					       uint8_t mask_enable,
					       uint16_t mask_value)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	uint8_t comp_mask = phase_complementary_mask(cfg);
	uint8_t physical_mask =
		pwm_ytm32_etmr_complementary_channel_mask(comp_mask);

	/* Only the channels owned by this PWM instance are touched. */
	mask_enable &= physical_mask;
	status_t status = eTMR_DRV_SetChnOutMask(cfg->instance, mask_enable,
							 mask_value, true);

	return status == STATUS_SUCCESS ? 0 : -EIO;
}

int pwm_ytm32_etmr_start(const struct device *dev)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	struct pwm_ytm32_data *data = dev->data;

	if (atomic_get(&data->fault_latched) != 0) {
		return -EPERM;
	}

	if (atomic_get(&data->counter_running) != 0) {
		return 0;
	}

	eTMR_DRV_Enable(cfg->instance);
	atomic_set(&data->counter_running, 1);
	return 0;
}

int pwm_ytm32_etmr_force_safe(const struct device *dev)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	struct pwm_ytm32_data *data = dev->data;
	uint8_t comp_mask = phase_complementary_mask(cfg);
	uint8_t physical_mask =
		pwm_ytm32_etmr_complementary_channel_mask(comp_mask);
	int ret;

	atomic_set(&data->safe_state, 1);
	ret = pwm_ytm32_etmr_mask_outputs(dev, physical_mask, 0U);
	/* Keep the software state fail-safe even if the HAL write is rejected. */
	return ret;
}

int pwm_ytm32_etmr_stop(const struct device *dev)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	struct pwm_ytm32_data *data = dev->data;
	int ret = pwm_ytm32_etmr_force_safe(dev);

	eTMR_DRV_Disable(cfg->instance);
	atomic_set(&data->counter_running, 0);
	return ret;
}

int pwm_ytm32_etmr_release_safe(const struct device *dev)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	struct pwm_ytm32_data *data = dev->data;
	struct pwm_ytm32_etmr_output_mask endpoint_mask;
	int ret;

	if (atomic_get(&data->fault_latched) != 0) {
		return -EPERM;
	}

	if (atomic_get(&data->phase_config_error) != 0 ||
	    !cfg->phase_channels_present) {
		return -EINVAL;
	}

	if (atomic_get(&data->counter_running) == 0) {
		return -EAGAIN;
	}

	/*
	 * Restore the commanded endpoint state instead of blindly clearing all
	 * masks.  A complementary pair at 0% contains a physical 100% output,
	 * which YTM32B1MD1 eTMR cannot generate from VAL0/VAL1 (E503001).
	 */
	endpoint_mask = pwm_ytm32_etmr_output_mask_unpack(
		(uint32_t)atomic_get(&data->endpoint_chmask));
	ret = pwm_ytm32_etmr_mask_outputs(dev, endpoint_mask.enable,
					 endpoint_mask.value);
	if (ret == 0) {
		atomic_set(&data->safe_state, 0);
	}
	return ret;
}

bool pwm_ytm32_etmr_is_safe(const struct device *dev)
{
	const struct pwm_ytm32_data *data = dev->data;

	return atomic_get(&data->safe_state) != 0 ||
	       atomic_get(&data->phase_config_error) != 0 ||
	       atomic_get(&data->fault_latched) != 0;
}

bool pwm_ytm32_etmr_phase_config_valid(const struct device *dev)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	const struct pwm_ytm32_data *data = dev->data;

	return cfg->phase_channels_present &&
	       atomic_get(&data->phase_config_error) == 0;
}

static uint8_t pwm_ytm32_etmr_fault_flags_from_status(uint32_t raw_status)
{
	uint8_t flags = 0U;

	if ((raw_status & eTMR_STS_F0F_MASK) != 0U) {
		flags |= BIT(0);
	}
	if ((raw_status & eTMR_STS_F1F_MASK) != 0U) {
		flags |= BIT(1);
	}
	if ((raw_status & eTMR_STS_F2F_MASK) != 0U) {
		flags |= BIT(2);
	}
	if ((raw_status & eTMR_STS_F3F_MASK) != 0U) {
		flags |= BIT(3);
	}

	return flags;
}

static uint8_t pwm_ytm32_etmr_fault_active_inputs(
		const struct device *dev)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	uint8_t status_mask = 0U;

	for (uint8_t fault = 0U; fault < PWM_YTM32_ETMR_FAULT_COUNT; fault++) {
		if ((cfg->fault_channels_mask & BIT(fault)) != 0U &&
		    eTMR_DRV_GetFaultInputStatus(cfg->instance, fault) != 0U) {
			status_mask |= BIT(fault);
		}
	}

	return pwm_ytm32_etmr_fault_status_active_mask(status_mask,
							cfg->fault_channels_mask);
}

int pwm_ytm32_etmr_register_fault_cb(const struct device *dev,
					      pwm_ytm32_fault_cb_t cb,
					      void *user_data)
{
	struct pwm_ytm32_data *data = dev->data;
	unsigned int key = irq_lock();

	data->fault_cb = cb;
	data->fault_user_data = user_data;
	irq_unlock(key);
	return 0;
}

int pwm_ytm32_etmr_fault_status_get(const struct device *dev,
					     struct pwm_ytm32_fault_status *status)
{
	const struct pwm_ytm32_data *data = dev->data;
	uint32_t raw_status;

	if (status == NULL) {
		return -EINVAL;
	}

	raw_status = (uint32_t)atomic_get(&data->fault_status);
	status->raw_status = raw_status;
	status->fault_flags = pwm_ytm32_etmr_fault_flags_from_status(raw_status);
	status->input_active_mask = pwm_ytm32_etmr_fault_active_inputs(dev);
	status->count = (uint32_t)atomic_get(&data->fault_count);
	status->latched = atomic_get(&data->fault_latched) != 0;
	status->armed = atomic_get(&data->fault_armed) != 0;
	return 0;
}

int pwm_ytm32_etmr_fault_clear(const struct device *dev)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	struct pwm_ytm32_data *data = dev->data;
	int ret;

	if (cfg->fault_channels_mask == 0U) {
		return -ENOTSUP;
	}

	if (pwm_ytm32_etmr_fault_active_inputs(dev) != 0U) {
		return -EBUSY;
	}

	ret = pwm_ytm32_etmr_force_safe(dev);
	if (ret < 0) {
		return ret;
	}

	eTMR_DRV_DisableInterrupts(cfg->instance, eTMR_FAULT_INT_ENABLE);
	for (uint8_t fault = 0U; fault < PWM_YTM32_ETMR_FAULT_COUNT; fault++) {
		if ((cfg->fault_channels_mask & BIT(fault)) == 0U) {
			continue;
		}

		eTMR_DRV_SetFaultChnEnable(cfg->instance, fault, false);
		eTMR_DRV_ClearFaultFlag(cfg->instance, fault);
	}

	atomic_set(&data->fault_latched, 0);
	atomic_set(&data->fault_armed, 0);
	atomic_set(&data->fault_status, 0);

	/* Clearing is an explicit recovery boundary: re-arm the hardware fault
	 * inputs before returning, but keep the software output mask asserted. */
	ret = pwm_ytm32_etmr_fault_arm(dev, true);
	if (ret < 0) {
		/* Do not expose a clear result if the interrupt path could not be
		 * restored.  The output remains safe and the next recovery attempt
		 * must explicitly review this condition. */
		atomic_set(&data->fault_latched, 1);
		return ret;
	}
	return 0;
}

int pwm_ytm32_etmr_fault_arm(const struct device *dev, bool arm)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	struct pwm_ytm32_data *data = dev->data;
	status_t status;
	int ret;

	if (cfg->fault_channels_mask == 0U) {
		return -ENOTSUP;
	}

	if (!arm) {
		ret = pwm_ytm32_etmr_force_safe(dev);
		eTMR_DRV_DisableInterrupts(cfg->instance, eTMR_FAULT_INT_ENABLE);
		for (uint8_t fault = 0U; fault < PWM_YTM32_ETMR_FAULT_COUNT;
		     fault++) {
			if ((cfg->fault_channels_mask & BIT(fault)) != 0U) {
				eTMR_DRV_SetFaultChnEnable(cfg->instance, fault, false);
			}
		}
		atomic_set(&data->fault_armed, 0);
		return ret;
	}

	if (atomic_get(&data->fault_latched) != 0) {
		return -EPERM;
	}

	if (pwm_ytm32_etmr_fault_active_inputs(dev) != 0U) {
		return -EBUSY;
	}

	for (uint8_t fault = 0U; fault < PWM_YTM32_ETMR_FAULT_COUNT; fault++) {
		if ((cfg->fault_channels_mask & BIT(fault)) != 0U) {
			eTMR_DRV_SetFaultChnEnable(cfg->instance, fault, true);
		}
	}

	status = eTMR_DRV_EnableInterrupts(cfg->instance,
						 eTMR_FAULT_INT_ENABLE);
	if (status != STATUS_SUCCESS) {
		for (uint8_t fault = 0U; fault < PWM_YTM32_ETMR_FAULT_COUNT;
		     fault++) {
			if ((cfg->fault_channels_mask & BIT(fault)) != 0U) {
				eTMR_DRV_SetFaultChnEnable(cfg->instance, fault, false);
			}
		}
		return -EIO;
	}

	atomic_set(&data->fault_armed, 1);
	return 0;
}

static void pwm_ytm32_handle_fault(const struct device *dev)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	struct pwm_ytm32_data *data = dev->data;
	pwm_ytm32_fault_cb_t cb;
	void *user_data;
	uint32_t raw_status = g_etmrBase[cfg->instance]->STS;
	unsigned int key;

	/* Snapshot first; the flag is intentionally not cleared in this ISR. */
	atomic_set(&data->fault_status, (atomic_val_t)raw_status);
	atomic_inc(&data->fault_count);
	atomic_set(&data->fault_latched, 1);
	atomic_set(&data->fault_armed, 0);
	eTMR_DRV_DisableInterrupts(cfg->instance, eTMR_FAULT_INT_ENABLE);
	(void)pwm_ytm32_etmr_force_safe(dev);

	key = irq_lock();
	cb = data->fault_cb;
	user_data = data->fault_user_data;
	irq_unlock(key);
	if (cb != NULL) {
		cb(dev, raw_status, user_data);
	}
}

/* ── Zephyr PWM API ──────────────────────────────────────────────────── */

/**
 * pwm_ytm32_set_cycles() - 设置指定通道的 PWM 周期与脉宽（Zephyr PWM API）
 *
 * @dev:           Zephyr PWM 设备指针
 * @channel:       目标通道号（0–7）
 * @period_cycles: PWM 周期，单位：计数器滴答
 * @pulse_cycles:  高电平脉宽，单位：计数器滴答
 * @flags:         PWM 标志位；支持 PWM_POLARITY_INVERTED（占空比反转）
 *
 * 行为说明：
 *   1. 若 period_cycles 与上次不同，调用 eTMR_DRV_UpdatePwmPeriod 更新
 *      MOD 寄存器；对中心对齐模式传入 period/2，因为计数器走 0→MOD→0
 *      整个周期共 2×MOD 个滴答；
 *   2. 将 pulse_cycles/period_cycles 转换为 Q15 格式占空比
 *      （0x0000 = 0%，0x8000 = 100%）；
 *   3. 若置 PWM_POLARITY_INVERTED，对占空比取反（eTMR_MAX_DUTY_CYCLE - duty）；
 *   4. 调用 eTMR_DRV_UpdatePwmChannel 写入影子寄存器，再用
 *      eTMR_DRV_SyncWithSoftTrigger 软件触发一次同步，确保原子生效。
 *
 * 奇数通道若属于互补对，则为只读侧，不可直接设置，应设其偶数对伴通道。
 *
 * 返回值：0 成功；-EINVAL 参数非法；-EIO SDK 调用失败。
 */
static int pwm_ytm32_set_cycles(const struct device *dev, uint32_t channel,
				uint32_t period_cycles, uint32_t pulse_cycles,
				pwm_flags_t flags)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	struct pwm_ytm32_data *data = dev->data;
	bool center = channel_is_comp(cfg, channel);
	struct pwm_ytm32_etmr_output_mask endpoint_mask;
	uint32_t duty_q15;
	uint32_t sdk_period;

	if (channel >= ETMR_CH_COUNT) {
		return -EINVAL;
	}

	/* Odd channel that is the comp output of a pair is read-only from HW */
	if ((channel & 1U) && center) {
		LOG_WRN("ch%u is a complementary output; set ch%u instead",
			channel, channel & ~1U);
		return -EINVAL;
	}

	if (period_cycles == 0U) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (period_cycles != data->period_cycles) {
		/*
		 * eTMR is a pure up-only counter.  Period = MOD+1 ticks
		 * regardless of edge-aligned or center-aligned mode.
		 * Center-aligned is achieved via symmetric VAL0/VAL1.
		 */
		sdk_period = period_cycles;
		status_t s = eTMR_DRV_UpdatePwmPeriod(cfg->instance,
						       eTMR_PWM_PERIOD_IN_TICKS,
						       sdk_period);
		if (s != STATUS_SUCCESS) {
			k_mutex_unlock(&data->lock);
			LOG_ERR("UpdatePwmPeriod failed (%d)", s);
			return -EIO;
		}
		data->period_cycles = period_cycles;

		/* Keep MID register in sync with new MOD for MIDTEN-based trigger. */
		if (cfg->adc_mid_trigger) {
			uint16_t new_mod = (uint16_t)data->etmr_state.etmrModValue;

			g_etmrBase[cfg->instance]->MID = new_mod / 2U;
		}
	}

	/* Q15 duty cycle: 0x0000 = 0 %, 0x8000 = 100 % */
	duty_q15 = (uint32_t)(((uint64_t)pulse_cycles * eTMR_MAX_DUTY_CYCLE)
			       / period_cycles);
	duty_q15 = MIN(duty_q15, (uint32_t)eTMR_MAX_DUTY_CYCLE);

	if (flags & PWM_POLARITY_INVERTED) {
		duty_q15 = eTMR_MAX_DUTY_CYCLE - duty_q15;
	}

	eTMR_DRV_UpdatePwmChannel(cfg->instance, channel, duty_q15, 0U);

	if (center) {
		endpoint_mask = pwm_ytm32_etmr_output_mask_unpack(
			(uint32_t)atomic_get(&data->endpoint_chmask));
		pwm_ytm32_etmr_endpoint_mask_update(&endpoint_mask,
						     (uint8_t)channel,
						     (uint16_t)duty_q15);
		atomic_set(&data->endpoint_chmask,
			   (atomic_val_t)pwm_ytm32_etmr_output_mask_pack(
				   &endpoint_mask));
		if (atomic_get(&data->safe_state) == 0) {
			g_etmrBase[cfg->instance]->CHMASK =
				pwm_ytm32_etmr_output_mask_pack(&endpoint_mask);
			/* A zero-latency safe request may preempt after the check
			 * above.  Never let this update's later software sync restore
			 * an endpoint mask over the all-low safety shadow. */
			if (atomic_get(&data->safe_state) != 0) {
				g_etmrBase[cfg->instance]->CHMASK =
					pwm_ytm32_etmr_complementary_channel_mask(
						phase_complementary_mask(cfg));
			}
		}
	}

	/* Commit all pending shadow register updates atomically */
	eTMR_DRV_SyncWithSoftTrigger(cfg->instance);

	k_mutex_unlock(&data->lock);
	return 0;
}

/**
 * pwm_ytm32_get_cycles_per_sec() - 查询计数器时钟频率（Zephyr PWM API）
 *
 * @dev:     Zephyr PWM 设备指针
 * @channel: 通道号（未使用，所有通道共用同一计数器）
 * @cycles:  输出参数，写入计数器频率（Hz）
 *
 * 频率由 counter_freq() 计算，等于 clk_rate >> prescaler。
 * 调用方可用此值将秒/毫秒换算为 period_cycles / pulse_cycles。
 *
 * 返回值：始终为 0。
 */
static int pwm_ytm32_get_cycles_per_sec(const struct device *dev,
					uint32_t channel, uint64_t *cycles)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	const struct pwm_ytm32_data *data = dev->data;

	ARG_UNUSED(channel);
	*cycles = counter_freq(cfg, data);
	return 0;
}

/* ── overflow ISR (shared handler, per-instance wrapper below) ───────── */

/**
 * pwm_ytm32_handle_ovf() - eTMR 计数器溢出（TOF）中断公共处理函数
 *
 * @dev: Zephyr PWM 设备指针
 *
 * 由各实例专属 ISR（pwm_ytm32_ovf_isr_<inst>）调用。完成两件事：
 *   1. 清除 STS.TOF（位 13）标志——必须写 eTMR_STS_TOF_MASK（0x2000）；
 *      注意：eTMR_TIME_OVER_FLOW_FLAG 枚举值（0x1000）对应不同的位，
 *      若误用将无法清除 TOF，导致 IRQ 风暴（曾触发过此 bug）；
 *   2. 若已注册 ovf_cb，调用用户回调（通常在此更新 FOC 三相占空比）。
 */
static void pwm_ytm32_handle_ovf(const struct device *dev)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	struct pwm_ytm32_data *data = dev->data;

	/*
	 * Clear the timer-overflow flag.  The STS TOF flag is bit 13
	 * (eTMR_STS_TOF_MASK = 0x2000); the eTMR_TIME_OVER_FLOW_FLAG enum
	 * (0x1000) is a different bit and does NOT clear it — writing that
	 * leaves TOF asserted and the OVF interrupt re-fires forever.
	 */
	eTMR_DRV_ClearTofFlag(cfg->instance);

	if (data->ovf_cb) {
		data->ovf_cb(data->ovf_user_data);
	}
}

/**
 * pwm_ytm32_register_ovf_cb() - 注册 PWM 溢出（周期）回调
 *
 * @dev:       Zephyr PWM 设备指针
 * @cb:        回调函数指针；传 NULL 可注销回调
 * @user_data: 透传给回调的用户指针
 *
 * 以关中断方式原子更新 ovf_cb / ovf_user_data，确保 ISR 侧不会看到
 * 半更新状态。回调在溢出 ISR 上下文中执行，须满足 ISR 安全约束
 * （不得睡眠，不得使用非 ISR 安全的 API）。
 *
 * 返回值：始终为 0。
 */
int pwm_ytm32_register_ovf_cb(const struct device *dev,
			       pwm_ytm32_ovf_cb_t cb, void *user_data)
{
	struct pwm_ytm32_data *data = dev->data;

	unsigned int key = irq_lock();
	data->ovf_cb = cb;
	data->ovf_user_data = user_data;
	irq_unlock(key);
	return 0;
}

/* ── 3-phase center-aligned fast duty update (FOC hot path) ──────────── */

BUILD_ASSERT(FEATURE_eTMR_COUNT_FROM_INIT_PLUS_ONE == 0,
	     "3-phase fast path assumes the COUNT_FROM_INIT_PLUS_ONE==0 edge convention; "
	     "revisit pwm_ytm32_etmr_center_edges / endpoints for this SoC");

/* Write one complementary-pair even channel's center-aligned shadow edges. */
static inline void pwm_ytm32_etmr_write_center_duty(eTMR_Type *base,
							   uint8_t ch,
							   uint32_t period,
							   uint16_t duty_q15)
{
	uint32_t val0, val1;

	pwm_ytm32_etmr_center_edges(period, duty_q15, &val0, &val1);
	base->CH[ch].VAL0 = val0;   /* == eTMR_SetChnVal0(base, ch, val0) */
	base->CH[ch].VAL1 = val1;   /* == eTMR_SetChnVal1(base, ch, val1) */
}

/**
 * pwm_ytm32_update_3phase_isr() - ISR 上下文中原子更新三相 PWM 占空比
 *
 * @dev:    Zephyr PWM 设备指针
 * @da_q15: A 相占空比，Q15 格式（0x0000 = 0%，0x8000 = 100%）
 * @db_q15: B 相占空比，Q15 格式
 * @dc_q15: C 相占空比，Q15 格式
 *
 * 仅适用于包含三个不同偶数通道的有效 phase-channels 配置。必须在
 * OVF 回调中调用，此时计数器处于 INIT 事件附近，死区插入
 * 和占空比更新均在下一个 PWM 周期生效，满足电机驱动同步要求。
 *
 * 流程：
 *   1. 依次写配置映射中的三对通道的影子占空比寄存器；
 *   2. 软件触发一次 Sync，将三相变化原子提交到活动寄存器。
 */
void pwm_ytm32_update_3phase_isr(const struct device *dev,
				  uint16_t da_q15, uint16_t db_q15,
				  uint16_t dc_q15)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	struct pwm_ytm32_data *data = dev->data;
	eTMR_Type *base = g_etmrBase[cfg->instance];
	struct pwm_ytm32_etmr_output_mask endpoint_mask = {0};
	uint32_t period = data->etmr_state.etmrPeriod;

	if (!pwm_ytm32_etmr_phase_config_valid(dev) || period == 0U) {
		atomic_set(&data->phase_config_error, 1);
		(void)pwm_ytm32_etmr_force_safe(dev);
		return;
	}

	/* Bypass eTMR_DRV_UpdatePwmChannel in the carrier ISR.  The configured
	 * pairs are complementary and center-aligned, so direct shadow writes are
	 * deterministic and the single software sync commits all three phases. */
	pwm_ytm32_etmr_write_center_duty(base, cfg->phase_channels[0], period,
					 da_q15);
	pwm_ytm32_etmr_write_center_duty(base, cfg->phase_channels[1], period,
					 db_q15);
	pwm_ytm32_etmr_write_center_duty(base, cfg->phase_channels[2], period,
					 dc_q15);

	/*
	 * YTM32B1MD1 erratum E503001: a physical 100% PWM output cannot be
	 * generated with VAL0/VAL1.  Every 0% complementary pair contains such
	 * a 100% odd output, so commit endpoint CHMASK state with the three edge
	 * updates.  While software-safe remains asserted, only remember the
	 * command; release_safe() will restore it after the first complete frame.
	 */
	pwm_ytm32_etmr_endpoint_mask_update(&endpoint_mask,
					     cfg->phase_channels[0], da_q15);
	pwm_ytm32_etmr_endpoint_mask_update(&endpoint_mask,
					     cfg->phase_channels[1], db_q15);
	pwm_ytm32_etmr_endpoint_mask_update(&endpoint_mask,
					     cfg->phase_channels[2], dc_q15);
	atomic_set(&data->endpoint_chmask,
		   (atomic_val_t)pwm_ytm32_etmr_output_mask_pack(&endpoint_mask));
	if (atomic_get(&data->safe_state) == 0) {
		base->CHMASK = pwm_ytm32_etmr_output_mask_pack(&endpoint_mask);
		/* Preserve a concurrent all-low safety request if it preempted
		 * this overflow update between the first state check and write. */
		if (atomic_get(&data->safe_state) != 0) {
			base->CHMASK =
				pwm_ytm32_etmr_complementary_channel_mask(
					phase_complementary_mask(cfg));
		}
	}
	eTMR_DRV_SyncWithSoftTrigger(cfg->instance);
}

/* ── init ────────────────────────────────────────────────────────────── */

/**
 * pwm_ytm32_init() - eTMR PWM 驱动初始化
 *
 * @dev: Zephyr PWM 设备指针
 *
 * 在 POST_KERNEL 阶段由 Zephyr 设备模型调用（通过实例专属包装函数
 * pwm_ytm32_init_<inst>）。执行步骤：
 *   1. 使能功能时钟，获取时钟频率写入 data->clk_rate；
 *   2. 应用 pinctrl 默认状态，复用 eTMR 引脚；
 *   3. 调用 eTMR_DRV_Init：配置计数器时钟源、预分频、调试模式；
 *      etmrPrescaler 传 2^prescaler（而非 DT 原始位移值），否则 SDK
 *      会写入 CLKPRS = 0x7F（÷128）而非期望的分频比；
 *   4. 调用 eTMR_DRV_InitPwm：仅配置由 phase_channels 派生的通道对，互补对使用中心对齐、
 *      插入死区；频率参数直接传 DT 的 pwm-frequency-hz（eTMR 是纯
 *      单向上升计数器，不需×2补偿；中心对齐靠对称 VAL0/VAL1 实现）；
 *   5. 若 adc_sync_trigger = true，调用 eTMR_DRV_SetOutputTrigger，
 *      在 INIT-match 事件输出触发脉冲，经 TMU 路由到 ADC 硬件触发；
 *      注意：必须在 eTMR_DRV_InitPwm 之后配置，否则会被 InitPwm 覆盖；
 *   6. 若 autostart = true，调用 eTMR_DRV_Enable 启动计数器；
 *   7. 初始化 data->lock 互斥锁。
 *
 * 返回值：0 成功；-ENODEV 时钟设备未就绪；-EIO SDK 调用失败；
 *         其他负值来自 clock_control / pinctrl。
 */
/*
 * Read the SCU hardware registers to compute the actual FAST_BUS_CLK.
 * The CGU HAL / clock_control_get_rate may return incorrect values
 * for the fast bus clock.  This helper reads the real state directly.
 *
 * IMPORTANT: Use SCU->STS CLKST bits [1:0] (actual clock source status),
 * NOT SCU->CLKS (configuration).  If the clock switch timed out, CLKS
 * may show FXOSC while the hardware is still running from FIRC.
 *
 * SCU->STS  bits [1:0] = CLKST (actual system clock source)
 *   00=FIRC, 01=PLL, 10=FXOSC, 11=SIRC
 * SCU->DIV  bits [11:8] = FBDIVS (fast bus divider - 1)
 */
static uint32_t etmr_get_fast_bus_clk(void)
{
	uint32_t clkst = (SCU->STS & SCU_STS_CLKST_MASK) >> SCU_STS_CLKST_SHIFT;
	uint32_t sys_clk;

	switch (clkst) {
	case 0U: /* FIRC */
		sys_clk = YTM32_FIRC_HZ;
		break;
	case 1U: /* PLL — read actual Fout from PLL_CTRL */
#if defined(CPU_YTM32B1MD1)
	{
		uint32_t pll_ctrl = SCU->PLL_CTRL;
		uint32_t ref_clk = (pll_ctrl & SCU_PLL_CTRL_REFCLKSRCSEL_MASK)
				   ? YTM32_FIRC_HZ
				   : (uint32_t)DT_PROP(DT_NODELABEL(cgu),
						       fxosc_frequency);
		uint32_t fbdiv = ((pll_ctrl & SCU_PLL_CTRL_FBDIV_MASK)
				  >> SCU_PLL_CTRL_FBDIV_SHIFT) + 1U;
		uint32_t refdiv = ((pll_ctrl & SCU_PLL_CTRL_REFDIV_MASK)
				   >> SCU_PLL_CTRL_REFDIV_SHIFT) + 1U;
		sys_clk = (ref_clk * fbdiv) / (2U * refdiv);
		break;
	}
#else
		/* SoCs without an SCU PLL (e.g. YTM32B1MC0) never report CLKST=PLL. */
		sys_clk = YTM32_FIRC_HZ;
		break;
#endif
	case 2U: /* FXOSC — use DTS value */
		sys_clk = DT_PROP(DT_NODELABEL(cgu), fxosc_frequency);
		break;
	default: /* SIRC */
		sys_clk = YTM32_SIRC_HZ;
		break;
	}

	uint32_t fb_divs = (SCU->DIV & SCU_DIV_FBDIVS_MASK) >> SCU_DIV_FBDIVS_SHIFT;

	return sys_clk / (fb_divs + 1U);
}

static int pwm_ytm32_init(const struct device *dev)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	struct pwm_ytm32_data *data = dev->data;
	struct pwm_ytm32_etmr_phase_map phase_map;
	uint32_t clk_rate;
	uint8_t comp_mask;
	int phase_ret;
	int ret;

	atomic_set(&data->safe_state, 1);
	atomic_set(&data->counter_running, 0);
	atomic_set(&data->phase_config_error, 1);
	atomic_set(&data->fault_latched, 0);
	atomic_set(&data->fault_armed, 0);
	atomic_set(&data->fault_count, 0);
	atomic_set(&data->fault_status, 0);

	comp_mask = phase_complementary_mask(cfg);
	phase_ret = pwm_ytm32_etmr_phase_map_validate(cfg->phase_channels,
							      &phase_map);
	if (cfg->phase_channels_present && comp_mask != 0U && phase_ret == 0) {
		atomic_set(&data->phase_config_error, 0);
	} else {
		LOG_WRN("eTMR%u: invalid or absent three-phase channel map; "
			"fast path remains safe", cfg->instance);
	}

	if (!device_is_ready(cfg->clk_dev)) {
		LOG_ERR("clock device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clk_dev, cfg->clk_sys);
	if (ret < 0) {
		LOG_ERR("clock_control_on failed (%d)", ret);
		return ret;
	}

	/*
	 * The eTMR counter runs from FAST_BUS_CLK, not from the module
	 * gate clock.  Read the actual fast bus rate from SCU registers.
	 */
	clk_rate = etmr_get_fast_bus_clk();
	LOG_INF("eTMR%u: FAST_BUS_CLK=%u Hz", cfg->instance, clk_rate);

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("pinctrl_apply_state failed (%d)", ret);
		return ret;
	}

	/* 1. Initialise the base timer (counter, prescaler, sync).
	 *
	 * The SDK's etmrPrescaler is the divide count and it programs the
	 * CLKPRS field as (etmrPrescaler - 1).  Our DT 'prescaler' is a
	 * power-of-two shift (0→÷1, 1→÷2, …, matching counter_freq()'s
	 * `clk >> prescaler`), so pass 2^prescaler as the divide count.
	 * Passing the raw shift would underflow to CLKPRS=0x7F (÷128).
	 *
	 * Auto-prescaler: if the requested frequency would require MOD > 0xFFFF
	 * at the DT prescaler, increment until MOD fits in 16 bits.
	 */
	/* No freq×2 for center-aligned: eTMR is a pure up-only counter
	 * (DS §26.4.2).  Center-aligned PWM is achieved via symmetric
	 * VAL0/VAL1 placement, NOT via up-down counting.  TOF fires once
	 * per period at MOD→INIT, same as edge-aligned. */
	uint32_t sdk_freq_pre = cfg->pwm_freq_hz;
	uint8_t prescaler = cfg->prescaler;

	while (prescaler < 7U &&
	       (clk_rate >> prescaler) / sdk_freq_pre > 0xFFFFU) {
		prescaler++;
	}
	if (prescaler != cfg->prescaler) {
		LOG_WRN("eTMR%u: prescaler auto-adjusted %u→%u for %u Hz",
			cfg->instance, cfg->prescaler, prescaler,
			cfg->pwm_freq_hz);
	}
	/* Store counter frequency (after prescaler) so counter_freq() is
	 * always correct regardless of whether auto-adjustment fired. */
	data->clk_rate = clk_rate >> prescaler;

	etmr_trig_config_t trig_cfg = {
		.trigSrc                = TRIGGER_FROM_MATCHING_EVENT,
		.pwmOutputChannel       = 0U,
		.outputTrigWidth        = (uint16_t)cfg->adc_trigger_width,
		.outputTrigFreq         = 1U,
		.modMatchTrigEnable     = false,
		.midMatchTrigEnable     = cfg->adc_mid_trigger,
		.initMatchTrigEnable    = cfg->adc_sync_trigger,
		.numOfChannels          = 0U,
		.channelTrigParamConfig = NULL,
	};

	/*
	 * The PWM update path writes CHxVAL0/CHxVAL1 and the overflow ISR then
	 * calls eTMR_DRV_SyncWithSoftTrigger().  The SDK leaves register loading
	 * disabled when syncMethod is NULL, which leaves CHx.CTRL.LDEN clear and
	 * prevents those pending values from reaching the active compare
	 * registers.  Keep the load path as a driver invariant: register loads
	 * are selected by the software trigger, once per period, while counter
	 * and output-mask loading remain tied to register loading.
	 */
	etmr_pwm_sync_t sync_cfg = {
		.regSyncFreq            = 1U,
		.regSyncSel             = REG_SYNC_WITH_TRIG,
		.cntInitSyncSel         = CNT_SYNC_WITH_REG,
		.maskOutputSyncSel      = CHMASK_SYNC_WITH_REG,
		.regSyncTrigSrc         = SW_TRIGGER,
		.cntInitSyncTrigSrc     = DISABLE_TRIGGER,
		.maskOutputSyncTrigSrc  = DISABLE_TRIGGER,
		.hwTrigFromTmuEnable    = false,
		.hwTrigFromCimEnable    = false,
		.hwTrigFromPadEnable    = false,
	};

	etmr_user_config_t user_cfg = {
		.etmrClockSource = eTMR_CLOCK_SOURCE_INTERNALCLK,
		.etmrPrescaler   = (uint8_t)BIT(prescaler),
		.debugMode       = false,
		.syncMethod      = &sync_cfg,
		.outputTrigConfig = NULL,
		.isTofIntEnabled = true,
	};

	status_t s = eTMR_DRV_Init(cfg->instance, &user_cfg, &data->etmr_state);
	if (s != STATUS_SUCCESS) {
		LOG_ERR("eTMR_DRV_Init failed (%d)", s);
		return -EIO;
	}

	/* Patch the SDK's cached clock frequency with the value from
	 * etmr_get_fast_bus_clk() (reads SCU_STS, not SCU_CLKS config).
	 * Needed because eTMR_DRV_InitPwm uses etmrSourceClockFrequency
	 * to compute MOD, and HAL state may lag if a clock switch timed out.
	 */
	data->etmr_state.etmrSourceClockFrequency = data->clk_rate;

	/* 2. Build per-channel configs only for the complementary pairs derived
	 *    from phase_channels.  This prevents unselected channels from being
	 *    initialized as PWM outputs by the vendor HAL. */
	uint16_t dt_ticks = ns_to_ticks(cfg->deadtime_ns,
					counter_freq(cfg, data));
	etmr_pwm_ch_param_t ch_cfgs[ETMR_PAIR_COUNT];
	uint8_t channel_count = 0U;

	for (uint8_t ch = 0U; ch < ETMR_CH_COUNT; ch += 2U) {
		if ((comp_mask & BIT(ch)) == 0U) {
			continue;
		}

		ch_cfgs[channel_count++] = (etmr_pwm_ch_param_t){
			.hwChannelId            = ch,
			.polarity               = eTMR_POLARITY_NORMAL,
			.pwmSrcInvert           = false,
			.align                  = eTMR_PWM_CENTER_ALIGN,
			.channelInitVal         = 0U,
			.typeOfUpdate           = eTMR_PWM_UPDATE_IN_DUTY_CYCLE,
			.dutyCycle              = 0U,
			.offset                 = 0U,
			.enableSecondChannelOutput = true,
			.secondChannelPolarity  = eTMR_POLARITY_NORMAL,
			.enableDoubleSwitch     = false,
			.evenDeadTime            = dt_ticks,
			.oddDeadTime             = dt_ticks,
		};
	}

	etmr_fault_param_t fault_cfg = {0};
	if (cfg->fault_channels_mask != 0U) {
		fault_cfg.pwmFaultInterrupt = false;
		fault_cfg.faultFilterSampleCounter = cfg->fault_filter_count & 0x0FU;
		fault_cfg.faultFilterSamplePeriod = cfg->fault_filter_period & 0x0FU;
		fault_cfg.faultInputStrentch = cfg->fault_input_stretch ? 1U : 0U;
		fault_cfg.pwmRecoveryOpportunity =
			(etmr_pwm_recovery_opportunity_t)cfg->fault_recovery;
		fault_cfg.pwmAutoRecoveryMode =
			(etmr_pwm_recovery_auto_mode_t)cfg->fault_auto_mode;
		fault_cfg.faultMode = cfg->fault_combinational ?
			eTMR_FAULT_WITHOUT_CLK : eTMR_FAULT_WITH_CLK;

		/* Motor-control safety invariant: every physical PWM input is low
		 * after a fault.  This is intentionally fixed in the driver; a board
		 * must not select high or tristate as a fault response. */
		for (uint8_t channel = 0U; channel < ETMR_CH_COUNT; channel++) {
			fault_cfg.safeState[channel] = eTMR_LOW_STATE;
		}

		for (uint8_t fault = 0U;
		     fault < PWM_YTM32_ETMR_FAULT_COUNT; fault++) {
			fault_cfg.etmrFaultChannelParam[fault].faultChannelEnabled =
				(cfg->fault_channels_mask & BIT(fault)) != 0U;
			fault_cfg.etmrFaultChannelParam[fault].faultInputPolarity =
				(cfg->fault_active_low_mask & BIT(fault)) != 0U ?
					eTMR_FAULT_SIGNAL_LOW : eTMR_FAULT_SIGNAL_HIGH;
		}
	}

	/*
	 * 3. Pass the actual PWM frequency to the SDK.
	 *    eTMR is up-only: MOD = clk/freq - 1, period = (MOD+1) ticks.
	 *    Center-aligned PWM uses symmetric VAL0/VAL1, NOT up-down counting.
	 *    Do NOT double the frequency for center-aligned mode.
	 */
	uint32_t sdk_freq = cfg->pwm_freq_hz;

	etmr_pwm_param_t pwm_param = {
		.nNumPwmChannels        = channel_count,
		.mode                   = eTMR_PWM_MODE,
		.uFrequencyHZ           = sdk_freq,
		.counterInitValFromInitReg = true,
		.cntVal                 = 0U,
		.pwmChannelConfig       = ch_cfgs,
		.faultConfig            = cfg->fault_channels_mask != 0U ?
						&fault_cfg : NULL,
	};

	s = eTMR_DRV_InitPwm(cfg->instance, &pwm_param);
	if (s != STATUS_SUCCESS) {
		LOG_ERR("eTMR_DRV_InitPwm failed (%d)", s);
		return -EIO;
	}

	/* InitPwm starts every configured pair at 0%.  Preserve the corresponding
	 * endpoint state beneath the all-low software safety mask. */
	struct pwm_ytm32_etmr_output_mask initial_endpoint_mask = {0};

	for (uint8_t phase = 0U; phase < PWM_YTM32_ETMR_PHASE_COUNT; phase++) {
		pwm_ytm32_etmr_endpoint_mask_update(&initial_endpoint_mask,
						     cfg->phase_channels[phase], 0U);
	}
	atomic_set(&data->endpoint_chmask,
		   (atomic_val_t)pwm_ytm32_etmr_output_mask_pack(
			   &initial_endpoint_mask));

	/* InitPwm configures channel initial values but does not establish the
	 * software safety invariant.  Apply the output mask before any optional
	 * counter start or trigger activity. */
	ret = pwm_ytm32_etmr_force_safe(dev);
	if (ret < 0) {
		LOG_ERR("eTMR%u: failed to apply safe output mask", cfg->instance);
		return ret;
	}

	if (cfg->fault_channels_mask != 0U) {
		/* InitFault enables the configured FxEN bits but deliberately leaves
		 * the interrupt disabled until outputs are already masked. */
		atomic_set(&data->fault_armed, 1);
		s = eTMR_DRV_EnableInterrupts(cfg->instance,
						      eTMR_FAULT_INT_ENABLE);
		if (s != STATUS_SUCCESS) {
			atomic_set(&data->fault_armed, 0);
			LOG_ERR("eTMR%u: failed to enable fault interrupt", cfg->instance);
			return -EIO;
		}
	}

	/* InitPwm resets OTRIG and MID, so configure them after PWM init.
	 * adc_sync_trigger → INITEN fires eTMR<n>_INIT_TRIG (TMU slot 22) at
	 * counter bottom each period.
	 * adc_mid_trigger → MIDTEN fires eTMR<n>_EXT_TRIG (TMU slot 23) at
	 * MID = MOD/2 each period; no channel consumed, no UpdatePwmPeriod clash.
	 * Note: OTRIG.MIDTEN maps to EXT_TRIG (slot 23), not INIT_TRIG (slot 22).
	 */
	if (cfg->adc_sync_trigger || cfg->adc_mid_trigger) {
		s = eTMR_DRV_SetOutputTrigger(cfg->instance, &trig_cfg);
		if (s != STATUS_SUCCESS) {
			LOG_ERR("eTMR_DRV_SetOutputTrigger failed (%d)", s);
			return -EIO;
		}
	}

	data->period_cycles = counter_freq(cfg, data) / cfg->pwm_freq_hz;

	/* 5. Set MID = MOD/2 for MIDTEN-based mid-period ADC trigger.
	 * InitPwm resets MID to 0, so this must come after InitPwm.
	 */
	if (cfg->adc_mid_trigger) {
		uint16_t mod = (uint16_t)data->etmr_state.etmrModValue;

		g_etmrBase[cfg->instance]->MID = mod / 2U;
		LOG_DBG("eTMR%u MIDTEN: MID=%u (MOD=%u)",
			cfg->instance, (unsigned)(mod / 2U), (unsigned)mod);
	}

	/*
	 * 6. Optionally start the counter.  eTMR_DRV_Init/InitPwm only configure
	 *    the module; the counter does not run until CTRL.EN is set.
	 *    Only start if ytmicro,autostart is set in DTS — this prevents
	 *    unintended PWM output on non-motor applications at boot.
	 */
	if (cfg->autostart) {
		ret = pwm_ytm32_etmr_start(dev);
		if (ret < 0) {
			LOG_ERR("eTMR%u: failed to start counter", cfg->instance);
			return ret;
		}
	}

	k_mutex_init(&data->lock);

	LOG_DBG("eTMR%u ready: counter_clk=%u Hz prescaler=%u init_freq=%u Hz",
		cfg->instance, data->clk_rate, prescaler, cfg->pwm_freq_hz);

	return 0;
}

/* ── driver API table ────────────────────────────────────────────────── */

static const struct pwm_driver_api pwm_ytm32_driver_api = {
	.set_cycles        = pwm_ytm32_set_cycles,
	.get_cycles_per_sec = pwm_ytm32_get_cycles_per_sec,
};

/* ── per-instance device instantiation ──────────────────────────────── */

/**
 * ETMR_OVF_ISR_DEFINE(inst) - 为实例 inst 定义专属的溢出 ISR 包装函数
 *
 * @inst: DT 实例编号（0、1、…），由 DT_INST_FOREACH_STATUS_OKAY 展开
 *
 * IRQ_CONNECT 要求处理函数地址在编译期确定，故每个实例必须有独立的
 * ISR 符号 pwm_ytm32_ovf_isr_<inst>。该包装函数忽略传入的 dev 参数，
 * 直接通过 DEVICE_DT_INST_GET(inst) 获取正确的设备指针，再调用
 * pwm_ytm32_handle_ovf() 完成实际处理。
 */
#define ETMR_OVF_ISR_DEFINE(inst)					\
	static void pwm_ytm32_ovf_isr_##inst(const struct device *dev)	\
	{								\
		ARG_UNUSED(dev);					\
		pwm_ytm32_handle_ovf(DEVICE_DT_INST_GET(inst));		\
	}

#define ETMR_FAULT_ISR_DEFINE(inst)					\
	static void pwm_ytm32_fault_isr_##inst(const struct device *dev)	\
	{								\
		ARG_UNUSED(dev);					\
		pwm_ytm32_handle_fault(DEVICE_DT_INST_GET(inst));		\
	}

/**
 * ETMR_IRQ_INIT(inst) - 连接并使能实例 inst 的溢出中断
 *
 * @inst: DT 实例编号
 *
 * 在实例专属的 pwm_ytm32_init_<inst>() 中展开（而非 pwm_ytm32_init()
 * 内部），以便 IRQ_CONNECT 宏在编译期绑定正确的 ISR 符号和 IRQ 号。
 * 中断向量号和优先级从 DTS "ovf" 具名 IRQ 单元读取。
 */
#define ETMR_IRQ_INIT(inst)						\
	do {								\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, fault, irq),	\
			    DT_INST_IRQ_BY_NAME(inst, fault, priority),	\
			    pwm_ytm32_fault_isr_##inst,			\
			    NULL, 0);					\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, fault, irq));	\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, ovf, irq),	\
			    DT_INST_IRQ_BY_NAME(inst, ovf, priority),	\
			    pwm_ytm32_ovf_isr_##inst,			\
			    NULL, 0);					\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, ovf, irq));	\
	} while (false)

/* Expand an optional three-phase channel map without evaluating an absent
 * devicetree array property. */
#define PWM_YTM32_PHASE_CHANNEL_INIT(inst, index) \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, ytmicro_phase_channels), \
		(DT_INST_PROP_BY_IDX(inst, ytmicro_phase_channels, index)), \
		(0U))

/**
 * ETMR_PWM_DEVICE_INIT(inst) - 展开单个 eTMR PWM 实例的全部注册代码
 *
 * @inst: DT 实例编号，由 DT_INST_FOREACH_STATUS_OKAY 遍历展开
 *
 * 依次完成：
 *   1. PINCTRL_DT_INST_DEFINE：声明 pinctrl 配置对象；
 *   2. ETMR_OVF_ISR_DEFINE：生成实例专属 ISR 函数；
 *   3. 定义 pwm_ytm32_config_<inst>：从 DTS 属性填充所有配置字段；
 *   4. 定义 pwm_ytm32_data_<inst>：运行时数据（零初始化）；
 *   5. 定义 pwm_ytm32_init_<inst>：先调用 ETMR_IRQ_INIT 连接中断，
 *      再调用 pwm_ytm32_init() 完成其余初始化；
 *   6. DEVICE_DT_INST_DEFINE：向 Zephyr 设备模型注册设备，
 *      初始化优先级为 CONFIG_PWM_INIT_PRIORITY，
 *      API 表为 pwm_ytm32_driver_api。
 */
#define ETMR_PWM_DEVICE_INIT(inst)					\
									\
	BUILD_ASSERT(							\
		!(DT_INST_PROP(inst, ytmicro_adc_sync_trigger) &&	\
		  DT_INST_PROP(inst, ytmicro_adc_mid_trigger)),		\
		"ytmicro,adc-sync-trigger and "			\
		"ytmicro,adc-mid-trigger are mutually exclusive");\
									\
	PINCTRL_DT_INST_DEFINE(inst);					\
									\
	ETMR_OVF_ISR_DEFINE(inst)					\
									\
	ETMR_FAULT_ISR_DEFINE(inst)					\
									\
	static const struct pwm_ytm32_config pwm_ytm32_config_##inst = {\
		.instance  = DT_INST_PROP(inst, ytmicro_instance),	\
		.clk_dev   = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),	\
		.clk_sys   = (clock_control_subsys_t)			\
			     DT_INST_CLOCKS_CELL(inst, id),		\
		.prescaler = DT_INST_PROP_OR(inst, ytmicro_prescaler, 0),\
		.pincfg    = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),	\
		.deadtime_ns = DT_INST_PROP_OR(inst,			\
				ytmicro_deadtime_ns, 0),		\
		.pwm_freq_hz = DT_INST_PROP_OR(inst,			\
				ytmicro_pwm_frequency_hz, 20000),	\
		.adc_sync_trigger = DT_INST_PROP(inst,			\
				ytmicro_adc_sync_trigger),		\
		.adc_trigger_width = DT_INST_PROP_OR(inst,		\
				ytmicro_adc_trigger_width, 16U),	\
		.autostart = DT_INST_PROP(inst, ytmicro_autostart),	\
		.adc_mid_trigger = DT_INST_PROP(inst,			\
				ytmicro_adc_mid_trigger),		\
		.phase_channels = {					\
			PWM_YTM32_PHASE_CHANNEL_INIT(inst, 0),	\
			PWM_YTM32_PHASE_CHANNEL_INIT(inst, 1),	\
			PWM_YTM32_PHASE_CHANNEL_INIT(inst, 2),	\
		},							\
		.phase_channels_present = DT_INST_NODE_HAS_PROP(inst,\
				ytmicro_phase_channels),			\
		.fault_channels_mask = DT_INST_PROP_OR(inst,		\
				ytmicro_fault_channels_mask, 0),		\
		.fault_active_low_mask = DT_INST_PROP_OR(inst,	\
				ytmicro_fault_active_low_mask, 0),	\
		.fault_filter_count = DT_INST_PROP_OR(inst,		\
				ytmicro_fault_filter_count, 0),		\
		.fault_filter_period = DT_INST_PROP_OR(inst,	\
				ytmicro_fault_filter_period, 0),		\
		.fault_input_stretch = DT_INST_PROP(inst,		\
				ytmicro_fault_input_stretch),			\
		.fault_combinational = DT_INST_PROP(inst,		\
				ytmicro_fault_combinational),			\
		.fault_recovery = DT_INST_PROP_OR(inst,		\
				ytmicro_fault_recovery, 0),			\
		.fault_auto_mode = DT_INST_PROP_OR(inst,		\
				ytmicro_fault_auto_mode, 0),			\
	};								\
									\
	static struct pwm_ytm32_data pwm_ytm32_data_##inst;		\
									\
	static int pwm_ytm32_init_##inst(const struct device *dev)	\
	{								\
		ETMR_IRQ_INIT(inst);					\
		return pwm_ytm32_init(dev);				\
	}								\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			      pwm_ytm32_init_##inst,			\
			      NULL,					\
			      &pwm_ytm32_data_##inst,			\
			      &pwm_ytm32_config_##inst,			\
			      POST_KERNEL,				\
			      CONFIG_PWM_INIT_PRIORITY,			\
			      &pwm_ytm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ETMR_PWM_DEVICE_INIT)
