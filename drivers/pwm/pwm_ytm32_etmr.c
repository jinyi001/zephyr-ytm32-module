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
};

struct pwm_ytm32_data {
	struct k_mutex lock;
	uint32_t clk_rate;   /* functional clock rate after CGU, before eTMR prescaler */
	uint32_t period_cycles; /* last period passed to set_cycles, in counter ticks */
	etmr_state_t etmr_state;
	pwm_ytm32_ovf_cb_t ovf_cb;
	void *ovf_user_data;
};

/* ── helpers ─────────────────────────────────────────────────────────── */

static inline uint32_t counter_freq(const struct pwm_ytm32_config *cfg,
				    const struct pwm_ytm32_data *data)
{
	return data->clk_rate >> cfg->prescaler;
}

static inline bool channel_is_comp(const struct pwm_ytm32_config *cfg, uint32_t ch)
{
	uint32_t even = ch & ~1U;
	return (cfg->comp_mask & BIT(even)) != 0U;
}

static inline uint16_t ns_to_ticks(uint32_t ns, uint32_t freq_hz)
{
	/* freq_hz ticks per second → ticks per ns = freq_hz / 1e9 */
	return (uint16_t)(((uint64_t)ns * freq_hz) / 1000000000ULL);
}

/* ── Zephyr PWM API ──────────────────────────────────────────────────── */

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
		 * For center-aligned PWM the counter counts 0→MOD→0, so the
		 * full period = 2×MOD.  Pass period/2 as the "period ticks"
		 * so the SDK sets MOD = period/2 - 1.
		 */
		sdk_period = center ? (period_cycles / 2U) : period_cycles;
		status_t s = eTMR_DRV_UpdatePwmPeriod(cfg->instance,
						       eTMR_PWM_PERIOD_IN_TICKS,
						       sdk_period);
		if (s != STATUS_SUCCESS) {
			k_mutex_unlock(&data->lock);
			LOG_ERR("UpdatePwmPeriod failed (%d)", s);
			return -EIO;
		}
		data->period_cycles = period_cycles;
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

/*
 * ISR-safe 3-phase duty update for FOC.  Only valid when the device is
 * configured with complementary pairs on channels 0/1, 2/3, 4/5
 * (comp_mask = 0x15).  Call exclusively from within the OVF callback.
 * duty_*_q15 in [0, eTMR_MAX_DUTY_CYCLE].
 */
void pwm_ytm32_update_3phase_isr(const struct device *dev,
				  uint16_t da_q15, uint16_t db_q15,
				  uint16_t dc_q15)
{
	const struct pwm_ytm32_config *cfg = dev->config;
	uint32_t inst = cfg->instance;

	eTMR_DRV_UpdatePwmChannel(inst, 0U, da_q15, 0U);
	eTMR_DRV_UpdatePwmChannel(inst, 2U, db_q15, 0U);
	eTMR_DRV_UpdatePwmChannel(inst, 4U, dc_q15, 0U);
	eTMR_DRV_SyncWithSoftTrigger(inst);
}

/* ── init ────────────────────────────────────────────────────────────── */

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

	ret = clock_control_get_rate(cfg->clk_dev, cfg->clk_sys, &clk_rate);
	if (ret < 0) {
		LOG_ERR("clock_control_get_rate failed (%d)", ret);
		return ret;
	}
	data->clk_rate = clk_rate;

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
	 */
	etmr_trig_config_t trig_cfg = {
		.trigSrc                = TRIGGER_FROM_MATCHING_EVENT,
		.pwmOutputChannel       = 0U,
		.outputTrigWidth        = (uint16_t)cfg->adc_trigger_width,
		.outputTrigFreq         = 1U,
		.modMatchTrigEnable     = false,
		.midMatchTrigEnable     = false,
		.initMatchTrigEnable    = true,
		.numOfChannels          = 0U,
		.channelTrigParamConfig = NULL,
	};

	etmr_user_config_t user_cfg = {
		.etmrClockSource = eTMR_CLOCK_SOURCE_INTERNALCLK,
		.etmrPrescaler   = (uint8_t)BIT(cfg->prescaler),
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

	/* 2. Build per-channel configs for all four even/odd pairs */
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
	 * 3. For center-aligned mode the SDK computes MOD = clk/freq - 1.
	 *    For center-aligned the actual output period = 2×MOD ticks, so
	 *    pass 2×pwm_freq_hz so MOD ends up at clk/(2×freq), giving the
	 *    correct switching frequency.
	 */
	bool any_center = (cfg->comp_mask != 0U);
	uint32_t sdk_freq = any_center ? cfg->pwm_freq_hz * 2U : cfg->pwm_freq_hz;

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

	/* InitPwm rewrites timer registers, so configure OTRIG after PWM init. */
	if (cfg->adc_sync_trigger) {
		s = eTMR_DRV_SetOutputTrigger(cfg->instance, &trig_cfg);
		if (s != STATUS_SUCCESS) {
			LOG_ERR("eTMR_DRV_SetOutputTrigger failed (%d)", s);
			return -EIO;
		}
		LOG_DBG("eTMR%u output trigger enabled for ADC sync", cfg->instance);
	}

	/*
	 * 4. ADC-sync output trigger is configured as part of eTMR_DRV_Init()
	 *    above, matching the SDK/FAE initialization order.  It emits a widened
	 *    INIT-match pulse on eTMR<n>_INIT_TRIG for the TMU route.
	 */

	data->period_cycles = counter_freq(cfg, data) / cfg->pwm_freq_hz;

	/*
	 * 5. Optionally start the counter.  eTMR_DRV_Init/InitPwm only configure
	 *    the module; the counter does not run until CTRL.EN is set.
	 *    Only start if ytmicro,autostart is set in DTS — this prevents
	 *    unintended PWM output on non-motor applications at boot.
	 */
	if (cfg->autostart) {
		eTMR_DRV_Enable(cfg->instance);
	}

	k_mutex_init(&data->lock);

	LOG_DBG("eTMR%u ready: clk=%u Hz prescaler=%u init_freq=%u Hz",
		cfg->instance, clk_rate, cfg->prescaler, cfg->pwm_freq_hz);

	return 0;
}

/* ── driver API table ────────────────────────────────────────────────── */

static const struct pwm_driver_api pwm_ytm32_driver_api = {
	.set_cycles        = pwm_ytm32_set_cycles,
	.get_cycles_per_sec = pwm_ytm32_get_cycles_per_sec,
};

/* ── per-instance device instantiation ──────────────────────────────── */

/*
 * Each eTMR instance needs its own overflow ISR function because
 * IRQ_CONNECT requires a compile-time constant handler address.
 */
#define ETMR_OVF_ISR_DEFINE(inst)					\
	static void pwm_ytm32_ovf_isr_##inst(const struct device *dev)	\
	{								\
		ARG_UNUSED(dev);					\
		pwm_ytm32_handle_ovf(DEVICE_DT_INST_GET(inst));		\
	}

#define ETMR_IRQ_INIT(inst)						\
	do {								\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, ovf, irq),	\
			    DT_INST_IRQ_BY_NAME(inst, ovf, priority),	\
			    pwm_ytm32_ovf_isr_##inst,			\
			    NULL, 0);					\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, ovf, irq));	\
	} while (false)

#define ETMR_PWM_DEVICE_INIT(inst)					\
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
