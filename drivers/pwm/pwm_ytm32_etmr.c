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
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>

#include "etmr_pwm_driver.h"
#include "etmr_common.h"

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
	/* Bitmask: bit N (N even) set → channel N/N+1 form a complementary pair */
	uint8_t comp_mask;
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
};

struct pwm_ytm32_data {
	struct k_mutex lock;
	uint32_t clk_rate;   /* counter clock Hz = functional_clk >> effective_prescaler */
	uint32_t period_cycles; /* last period passed to set_cycles, in counter ticks */
	etmr_state_t etmr_state;
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

/**
 * channel_is_comp() - 判断通道是否属于互补对
 *
 * @cfg: 驱动配置结构体指针
 * @ch:  待查询的通道号（0–7）
 *
 * 将通道号取偶（ch & ~1U），查询 comp_mask 中对应的 bit。
 * 奇数通道（互补输出侧）与其偶数对伴通道共享同一 bit，因此
 * 奇/偶通道均可正确判断是否位于互补对中。
 *
 * 返回值：true 表示通道属于互补对，false 表示独立边沿对齐输出。
 */
static inline bool channel_is_comp(const struct pwm_ytm32_config *cfg, uint32_t ch)
{
	uint32_t even = ch & ~1U;
	return (cfg->comp_mask & BIT(even)) != 0U;
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

/*
 * Center-aligned duty (Q15: 0x8000 = 100%) → shadow VAL0/VAL1 edge ticks.
 * Mirrors eTMR_DRV_CalculateDutyCycle()'s center-aligned / UPDATE_IN_DUTY_CYCLE
 * branch for FEATURE_eTMR_COUNT_FROM_INIT_PLUS_ONE == 0 (guarded below):
 *   VAL0 = period·(0x4000 − duty/2) / 0x8000 − 1
 *   VAL1 = period·(0x4000 + duty/2) / 0x8000 − 1
 * with VAL0=VAL1=0 at 0% and (0, period−1) at 100%.  Named SDK constants keep
 * the formula readable and portable; the BUILD_ASSERT guards the edge convention.
 */
#define ETMR_CENTER_EDGE(period, frac15) \
	((uint32_t)(((uint64_t)(period) * (uint32_t)(frac15)) >> eTMR_DUTY_TO_TICKS_SHIFT) - 1U)

BUILD_ASSERT(FEATURE_eTMR_COUNT_FROM_INIT_PLUS_ONE == 0,
	     "3-phase fast path assumes the COUNT_FROM_INIT_PLUS_ONE==0 edge convention; "
	     "revisit ETMR_CENTER_EDGE / endpoints for this SoC");

/* Write one complementary-pair even channel's center-aligned shadow edges. */
static inline void etmr_write_center_duty(eTMR_Type *base, uint8_t ch,
					  uint32_t period, uint16_t duty_q15)
{
	uint32_t val0, val1;

	if (duty_q15 == 0U) {                          /* 0%  */
		val0 = 0U;
		val1 = 0U;
	} else if (duty_q15 >= eTMR_MAX_DUTY_CYCLE) {  /* 100% (also clamps FP overshoot) */
		val0 = 0U;
		val1 = period - 1U;
	} else {
		uint32_t half = (uint32_t)duty_q15 >> 1;

		val0 = ETMR_CENTER_EDGE(period, (eTMR_MAX_DUTY_CYCLE / 2U) - half);
		val1 = ETMR_CENTER_EDGE(period, (eTMR_MAX_DUTY_CYCLE / 2U) + half);
	}
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
 * 仅适用于以下配置：通道 0/1、2/3、4/5 均为互补对
 * （comp_mask = 0x15，即 DT ytmicro,complementary-channels-mask = <0x15>）。
 * 必须在 OVF 回调中调用，此时计数器处于 INIT 事件附近，死区插入
 * 和占空比更新均在下一个 PWM 周期生效，满足电机驱动同步要求。
 *
 * 流程：
 *   1. 依次写三对通道（ch0/ch2/ch4）的影子占空比寄存器；
 *   2. 软件触发一次 Sync，将三相变化原子提交到活动寄存器。
 */
void pwm_ytm32_update_3phase_isr(const struct device *dev,
				  uint16_t da_q15, uint16_t db_q15,
				  uint16_t dc_q15)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	struct pwm_ytm32_data *data = dev->data;
	eTMR_Type *base = g_etmrBase[cfg->instance];
	uint32_t period = data->etmr_state.etmrPeriod;

	/* Fast path: bypass eTMR_DRV_UpdatePwmChannel (per-call CalculateDutyCycle,
	 * state writes, asserts).  Channels 0/2/4 are the complementary-pair even
	 * channels (comp_mask=0x15), always center-aligned, so the general
	 * align-mode dispatch is unnecessary — write the shadow edges directly. */
	etmr_write_center_duty(base, 0U, period, da_q15);
	etmr_write_center_duty(base, 2U, period, db_q15);
	etmr_write_center_duty(base, 4U, period, dc_q15);
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
 *   4. 调用 eTMR_DRV_InitPwm：配置 4 对通道，互补对使用中心对齐、
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
	uint32_t clk_rate;
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

	etmr_user_config_t user_cfg = {
		.etmrClockSource = eTMR_CLOCK_SOURCE_INTERNALCLK,
		.etmrPrescaler   = (uint8_t)BIT(prescaler),
		.debugMode       = false,
		.syncMethod      = NULL,
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

	/* 2. Build per-channel configs for all four even-odd pairs */
	uint16_t dt_ticks = ns_to_ticks(cfg->deadtime_ns,
					counter_freq(cfg, data));
	etmr_pwm_ch_param_t ch_cfgs[ETMR_PAIR_COUNT];

	for (uint8_t i = 0U; i < ETMR_PAIR_COUNT; i++) {
		uint8_t ch = i * 2U;
		bool comp = (cfg->comp_mask & BIT(ch)) != 0U;

		ch_cfgs[i] = (etmr_pwm_ch_param_t){
			.hwChannelId            = ch,
			.polarity               = eTMR_POLARITY_NORMAL,
			.pwmSrcInvert           = false,
			.align                  = comp ? eTMR_PWM_CENTER_ALIGN
						       : eTMR_PWM_RIGHT_EDGE_ALIGN,
			.channelInitVal         = 0U,
			.typeOfUpdate           = eTMR_PWM_UPDATE_IN_DUTY_CYCLE,
			.dutyCycle              = 0U,
			.offset                 = 0U,
			.enableSecondChannelOutput = comp,
			.secondChannelPolarity  = eTMR_POLARITY_INVERT,
			.enableDoubleSwitch     = false,
			.evenDeadTime           = comp ? dt_ticks : 0U,
			.oddDeadTime            = comp ? dt_ticks : 0U,
		};
	}

	/*
	 * 3. Pass the actual PWM frequency to the SDK.
	 *    eTMR is up-only: MOD = clk/freq - 1, period = (MOD+1) ticks.
	 *    Center-aligned PWM uses symmetric VAL0/VAL1, NOT up-down counting.
	 *    Do NOT double the frequency for center-aligned mode.
	 */
	uint32_t sdk_freq = cfg->pwm_freq_hz;

	etmr_pwm_param_t pwm_param = {
		.nNumPwmChannels        = ETMR_PAIR_COUNT,
		.mode                   = eTMR_PWM_MODE,
		.uFrequencyHZ           = sdk_freq,
		.counterInitValFromInitReg = true,
		.cntVal                 = 0U,
		.pwmChannelConfig       = ch_cfgs,
		.faultConfig            = NULL,
	};

	s = eTMR_DRV_InitPwm(cfg->instance, &pwm_param);
	if (s != STATUS_SUCCESS) {
		LOG_ERR("eTMR_DRV_InitPwm failed (%d)", s);
		return -EIO;
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
		eTMR_DRV_Enable(cfg->instance);
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
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, ovf, irq),	\
			    DT_INST_IRQ_BY_NAME(inst, ovf, priority),	\
			    pwm_ytm32_ovf_isr_##inst,			\
			    NULL, 0);					\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, ovf, irq));	\
	} while (false)

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
	static const struct pwm_ytm32_config pwm_ytm32_config_##inst = {\
		.instance  = DT_INST_PROP(inst, ytmicro_instance),	\
		.clk_dev   = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),	\
		.clk_sys   = (clock_control_subsys_t)			\
			     DT_INST_CLOCKS_CELL(inst, id),		\
		.prescaler = DT_INST_PROP_OR(inst, ytmicro_prescaler, 0),\
		.pincfg    = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),	\
		.comp_mask = DT_INST_PROP_OR(inst,			\
				ytmicro_complementary_channels_mask, 0),\
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
