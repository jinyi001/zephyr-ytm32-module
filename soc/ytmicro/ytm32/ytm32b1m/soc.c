/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#if defined(CONFIG_SOC_YTM32B1MD1)
#include "system_YTM32B1MD1.h"
#else
#include "system_YTM32B1MC0.h"
#endif
#include "device_registers.h"
#include "clock.h"
#include <zephyr/init.h>
#include <zephyr/dt-bindings/clock/ytmicro,ytm32-soc-clock.h>
#include <zephyr/drivers/clock_control/ytm32_soc_clock.h>

/* YTM32_FIRC_HZ and YTM32_FXOSC_HZ are provided by the active SoC clock binding. */

#define YTM32_CLOCK_CONFIG_COUNT 1U
#define YTM32_CLOCK_CONFIG_INDEX 0U

/*
 * CMU comparison thresholds for FIRC/FXOSC monitoring:
 *
 *   Reference clock: SIRC (32 kHz typical)
 *   Monitored clocks: FIRC (80 MHz nominal), FXOSC (24 MHz nominal)
 *
 * CMU counts 128 reference cycles and compares the monitored clock edges
 * seen in that window. The vendor SDK expresses the threshold values as
 * (target_ratio * 128) / 2, so the chosen values map to these windows:
 *
 *   FIRC compareHigh = (100 * 128) / 2 = 6400   -> allow up to 100 MHz
 *   FIRC compareLow  = ( 60 * 128) / 2 = 3840   -> alarm below 60 MHz
 *   FXOSC compareHigh = (30 * 128) / 2 = 1920   -> allow up to 30 MHz
 *   FXOSC compareLow  = (18 * 128) / 2 = 1152   -> alarm below 18 MHz
 *   PLL  compareHigh = (150 * 128) / 2 = 9600   -> allow up to 150 MHz
 *   PLL  compareLow  = ( 90 * 128) / 2 = 5760   -> alarm below  90 MHz
 *
 * The PLL channel only exists when the SoC supports PLL; it is monitored
 * only when the PLL is actually used as the system clock source (otherwise
 * the channel stays disabled and the compare values are inert).
 */
static const cmu_config_t ytm32_cmu_config = {
	.fircClockMonitor = {
		.enable = YTM32_CMU_ENABLED,
		.resetEnable = YTM32_CMU_RESET_ENABLED,
		.refClock = CMU_REF_SIRC_CLOCK,
		.compareHigh = (100U * 128U) / 2U,
		.compareLow = (60U * 128U) / 2U,
	},
#if defined(FEATURE_SCU_SUPPORT_PLL) && FEATURE_SCU_SUPPORT_PLL
	.pllClockMonitor = {
		.enable = YTM32_CMU_ENABLED,
		.resetEnable = YTM32_CMU_RESET_ENABLED,
		.refClock = CMU_REF_SIRC_CLOCK,
		.compareHigh = (150U * 128U) / 2U,
		.compareLow = (90U * 128U) / 2U,
	},
#endif /* FEATURE_SCU_SUPPORT_PLL */
	.fxoscClockMonitor = {
		.enable = YTM32_CMU_ENABLED,
		.resetEnable = YTM32_CMU_RESET_ENABLED,
		.refClock = CMU_REF_SIRC_CLOCK,
		.compareHigh = (30U * 128U) / 2U,
		.compareLow = (18U * 128U) / 2U,
	},
};

static bool ytm32_divider_to_sys_div(uint32_t divider, uint8_t *sys_div)
{
	if ((divider == 0U) || (divider > 16U) || (sys_div == NULL)) {
		return false;
	}

	*sys_div = (uint8_t)(divider - 1U);
	return true;
}

#if defined(FEATURE_SCU_SUPPORT_PLL) && FEATURE_SCU_SUPPORT_PLL
/*
 * PLL output frequency, matching the vendor CLOCK_DRV_GetPllFreq() math:
 *   pll-out = (ref_hz / refdiv) * fbdiv / 2
 * where ref_hz is FXOSC or FIRC depending on pll_reference_clock.
 */
static uint32_t ytm32_pll_output_hz(const struct ytm32_soc_clock_config *cfg)
{
	uint32_t ref_hz = (cfg->pll_reference_clock == YTM32_PLL_REF_FIRC)
				  ? YTM32_FIRC_HZ
				  : cfg->fxosc_frequency;

	return ((ref_hz / cfg->pll_reference_divider) *
		cfg->pll_feedback_divider) / 2U;
}
#endif /* FEATURE_SCU_SUPPORT_PLL */

static int ytm32_system_clock_source_hz(const struct ytm32_soc_clock_config *cfg,
					 uint32_t *source_hz,
					 scu_system_clock_src_t *scu_source)
{
	if ((cfg == NULL) || (source_hz == NULL) || (scu_source == NULL)) {
		return -EINVAL;
	}

	switch (cfg->system_clock_source) {
	case YTM32_SYSTEM_CLOCK_SRC_FIRC:
		*source_hz = YTM32_FIRC_HZ;
		*scu_source = SCU_SYSTEM_CLOCK_SRC_FIRC;
		return 0;
	case YTM32_SYSTEM_CLOCK_SRC_FXOSC:
		if (cfg->fxosc_frequency == 0U) {
			return -EINVAL;
		}
		*source_hz = cfg->fxosc_frequency;
		*scu_source = SCU_SYSTEM_CLOCK_SRC_FXOSC;
		return 0;
#if defined(FEATURE_SCU_SUPPORT_PLL) && FEATURE_SCU_SUPPORT_PLL
	case YTM32_SYSTEM_CLOCK_SRC_PLL:
		if ((cfg->pll_reference_divider == 0U) ||
		    (cfg->pll_feedback_divider == 0U)) {
			return -EINVAL;
		}
		if ((cfg->pll_reference_clock == YTM32_PLL_REF_FXOSC) &&
		    (cfg->fxosc_frequency == 0U)) {
			return -EINVAL;
		}
		*source_hz = ytm32_pll_output_hz(cfg);
		*scu_source = SCU_SYSTEM_CLOCK_SRC_PLL;
		return 0;
#endif /* FEATURE_SCU_SUPPORT_PLL */
	default:
		return -EINVAL;
	}
}

void soc_prep_hook(void)
{
}

void soc_early_init_hook(void)
{
	/*
	 * Replicate the parts of the vendor SystemInit() that Zephyr does not
	 * own, but intentionally DROP EfmInitMpu(): the Cortex-M MPU is now
	 * configured by Zephyr (CONFIG_ARM_MPU + soc arm_mpu_regions.c). The FPU
	 * (CPACR) is enabled earlier by Zephyr's z_arm_floating_point_init() when
	 * CONFIG_FPU=y, so it is not repeated here.
	 *
	 * Ownership: R16 (EFM prefetch/DPD), R18 (CIM lockup), R15 (WDG0); MPU
	 * migration tracked as C-MPU-EFMINIT-01 in BRINGUP_RESOURCE_OWNERSHIP.md.
	 * Not calling SystemInit() lets --gc-sections drop SystemInit/EfmInitMpu.
	 */
	EFM->CTRL |= EFM_CTRL_DPD_EN_MASK | EFM_CTRL_PREFETCH_EN_MASK;
	CIM->CTRL |= CIM_CTRL_LOCKUPEN_MASK;
#if (DISABLE_WDOG)
	WDG0->SVCR = 0xB631U;
	WDG0->SVCR = 0xC278U;
	WDG0->CR &= ~WDG_CR_EN_MASK;
#endif
}

int ytm32_soc_apply_clock_config(const struct ytm32_soc_clock_config *cfg)
{
	scu_config_t scu_config = {
		.fircEnable = true,
		.fircDeepSleepEnable = false,
		.sircDeepSleepEnable = YTM32_SIRC_DEEPSLEEP_ENABLED,
		.sircStandbyEnable = YTM32_SIRC_STANDBY_ENABLED,
		.sircPowerDownEnable = false,
		.sysClkSrc = SCU_SYSTEM_CLOCK_SRC_FIRC,
		.fxoscConfig = {
			.enable = true,
			.bypassMode = false,
			.gainSelection = 6,
			.frequency = YTM32_FXOSC_HZ,
		},
		.sysDiv = SCU_SYS_CLK_DIV_BY_1,
		.fastBusDiv = SCU_SYS_CLK_DIV_BY_1,
		.slowBusDiv = SCU_SYS_CLK_DIV_BY_1,
		.flashDiv = SCU_SYS_CLK_DIV_BY_4,
		.clockOutConfig = {
			.enable = false,
			.source = SCU_CLKOUT_SEL_FIRC_CLK,
			.divider = 1U,
		},
	};
	clock_manager_user_config_t clock_config = {
		.scuConfigPtr = &scu_config,
		.cmuConfigPtr = &ytm32_cmu_config,
		.ipcConfig = {
			.peripheralClocks = NULL,
			.count = 0U,
		},
	};
	const clock_manager_user_config_t *clock_configs[YTM32_CLOCK_CONFIG_COUNT] = {
		&clock_config,
	};
	status_t status;
	uint32_t source_hz;
	scu_system_clock_src_t scu_source;
	int ret;

	ret = ytm32_system_clock_source_hz(cfg, &source_hz, &scu_source);
	if (ret < 0) {
		return ret;
	}

	if (cfg->core_clock != (source_hz / cfg->core_divider)) {
		return -EINVAL;
	}

	scu_config.sysClkSrc = scu_source;
	/*
	 * FXOSC must be enabled when it is the system clock source, or when it
	 * is the PLL reference.  Enabling it spuriously (no crystal) makes the
	 * vendor HAL block on CLOCK_SYS_WaitFXOSCValid() and silently stay on
	 * SIRC, so keep this condition tight.
	 */
	scu_config.fxoscConfig.enable =
		(cfg->system_clock_source == YTM32_SYSTEM_CLOCK_SRC_FXOSC) ||
		((cfg->system_clock_source == YTM32_SYSTEM_CLOCK_SRC_PLL) &&
		 (cfg->pll_reference_clock == YTM32_PLL_REF_FXOSC));
	scu_config.fxoscConfig.bypassMode = cfg->fxosc_bypass;
	scu_config.fxoscConfig.gainSelection = cfg->fxosc_gain_selection;
	scu_config.fxoscConfig.frequency = cfg->fxosc_frequency;

#if defined(FEATURE_SCU_SUPPORT_PLL) && FEATURE_SCU_SUPPORT_PLL
	scu_config.pllConfig.enable =
		(cfg->system_clock_source == YTM32_SYSTEM_CLOCK_SRC_PLL);
	scu_config.pllConfig.pllRefClock =
		(cfg->pll_reference_clock == YTM32_PLL_REF_FIRC)
			? SCU_PLL_REF_FIRC_CLK
			: SCU_PLL_REF_FXOSC_CLK;
	scu_config.pllConfig.pllFeedBackDiv = (uint8_t)cfg->pll_feedback_divider;
	scu_config.pllConfig.pllRefClkDiv = (uint8_t)cfg->pll_reference_divider;
#endif /* FEATURE_SCU_SUPPORT_PLL */

	if (!ytm32_divider_to_sys_div(cfg->core_divider, &scu_config.sysDiv)) {
		return -EINVAL;
	}

	if (!ytm32_divider_to_sys_div(cfg->fast_bus_divider, &scu_config.fastBusDiv)) {
		return -EINVAL;
	}

	if (!ytm32_divider_to_sys_div(cfg->slow_bus_divider, &scu_config.slowBusDiv)) {
		return -EINVAL;
	}

	status = CLOCK_SYS_Init(clock_configs, YTM32_CLOCK_CONFIG_COUNT, NULL, 0U);
	if (status != STATUS_SUCCESS) {
		return -EIO;
	}

	status = CLOCK_SYS_UpdateConfiguration(YTM32_CLOCK_CONFIG_INDEX,
					      CLOCK_MANAGER_POLICY_AGREEMENT);
	if (status != STATUS_SUCCESS) {
		return -EIO;
	}

	return 0;
}
