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
#include <zephyr/dt-bindings/clock/ytmicro,ytm32-soc-clock.h>

/*
 * ── CGU clock-tree compile-time constraints (RM §12.1) ───────────────────────
 *
 * These BUILD_ASSERTs cross-validate the DTS properties against each other and
 * against the C header constants.  They catch:
 *   1. Wrong crystal frequency (out-of-spec FXOSC value)
 *   2. core-clock inconsistent with fxosc-frequency / core-divider
 *   3. CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC diverged from the DTS core-clock
 *   4. YTM32_FIRC_HZ / YTM32_SIRC_HZ in clock.h diverged from the DTSI
 */

/* (1) FIRC / SIRC constants in clock.h must match the DTSI hardware fact. */
BUILD_ASSERT(
	DT_PROP_OR(DT_NODELABEL(cgu), firc_frequency, YTM32_FIRC_HZ) == YTM32_FIRC_HZ,
	"firc-frequency in DTS contradicts YTM32_FIRC_HZ in clock.h");
BUILD_ASSERT(
	DT_PROP_OR(DT_NODELABEL(cgu), sirc_frequency, YTM32_SIRC_HZ) == YTM32_SIRC_HZ,
	"sirc-frequency in DTS contradicts YTM32_SIRC_HZ in clock.h");

/* (2) FXOSC range: RM §12.1 permits 4~40 MHz.  Assertion is skipped when the
 *     system clock source is not FXOSC (logical-OR short-circuit). */
BUILD_ASSERT(
	DT_PROP(DT_NODELABEL(cgu), system_clock_source) != YTM32_SYSTEM_CLOCK_SRC_FXOSC ||
	(DT_PROP(DT_NODELABEL(cgu), fxosc_frequency) >= 4000000U &&
	 DT_PROP(DT_NODELABEL(cgu), fxosc_frequency) <= 40000000U),
	"fxosc-frequency out of spec: RM §12.1 allows 4~40 MHz only");

/* (3) core-clock must equal the derived value (source / core-divider). */
BUILD_ASSERT(
	DT_PROP(DT_NODELABEL(cgu), system_clock_source) != YTM32_SYSTEM_CLOCK_SRC_FXOSC ||
	DT_PROP(DT_NODELABEL(cgu), core_clock) ==
		DT_PROP(DT_NODELABEL(cgu), fxosc_frequency) /
		DT_PROP(DT_NODELABEL(cgu), core_divider),
	"core-clock != fxosc-frequency / core-divider (update core-clock in DTS)");
BUILD_ASSERT(
	DT_PROP(DT_NODELABEL(cgu), system_clock_source) != YTM32_SYSTEM_CLOCK_SRC_FIRC ||
	DT_PROP(DT_NODELABEL(cgu), core_clock) ==
		YTM32_FIRC_HZ / DT_PROP(DT_NODELABEL(cgu), core_divider),
	"core-clock != YTM32_FIRC_HZ / core-divider (update core-clock in DTS)");

/* (4) Kconfig SysTick frequency must match the CGU core-clock in DTS. */
BUILD_ASSERT(
	CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC ==
		DT_PROP(DT_NODELABEL(cgu), core_clock),
	"CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC != CGU core-clock: "
	"check Kconfig.defconfig dt_node_int_prop_int derivation");

/*
 * ── PLL clock-tree compile-time constraints ──────────────────────────────────
 *
 * Derived from the YTM32B1MD1 host configuration tool (clock.ts) and the vendor
 * CLOCK_DRV_GetPllFreq() math.  All PLL asserts short-circuit (logical-OR) when
 * the system clock source is not PLL.
 *
 *   pll-ref-hz = pll-reference-clock == FIRC ? FIRC : fxosc-frequency
 *   finput     = pll-ref-hz / pll-reference-divider          (must be 8–48 MHz)
 *   pll-out    = (finput * pll-feedback-divider) / 2         (must be 100–200 MHz)
 *   core-clock = pll-out / core-divider
 */
#define YTM32_CGU_SRC        DT_PROP(DT_NODELABEL(cgu), system_clock_source)
#define YTM32_CGU_IS_PLL     (YTM32_CGU_SRC == YTM32_SYSTEM_CLOCK_SRC_PLL)
#define YTM32_CGU_PLL_REFSRC DT_PROP_OR(DT_NODELABEL(cgu), pll_reference_clock, YTM32_PLL_REF_FXOSC)
#define YTM32_CGU_PLL_REFDIV DT_PROP_OR(DT_NODELABEL(cgu), pll_reference_divider, 1)
#define YTM32_CGU_PLL_FBDIV  DT_PROP_OR(DT_NODELABEL(cgu), pll_feedback_divider, 1)
#define YTM32_CGU_FXOSC      DT_PROP_OR(DT_NODELABEL(cgu), fxosc_frequency, 0)
#define YTM32_CGU_PLL_REF_HZ \
	((YTM32_CGU_PLL_REFSRC == YTM32_PLL_REF_FIRC) ? YTM32_FIRC_HZ : YTM32_CGU_FXOSC)
#define YTM32_CGU_PLL_FINPUT (YTM32_CGU_PLL_REF_HZ / YTM32_CGU_PLL_REFDIV)
#define YTM32_CGU_PLL_OUT    ((YTM32_CGU_PLL_FINPUT * YTM32_CGU_PLL_FBDIV) / 2U)
#define YTM32_CGU_FASTDIV    DT_PROP_OR(DT_NODELABEL(cgu), fast_bus_divider, 1)
#define YTM32_CGU_SLOWDIV \
	DT_PROP_OR(DT_NODELABEL(cgu), slow_bus_divider, \
		   DT_PROP_OR(DT_NODELABEL(cgu), bus_divider, 1))

/* (5) core-clock must equal the derived PLL output / core-divider. */
BUILD_ASSERT(
	!YTM32_CGU_IS_PLL ||
	DT_PROP(DT_NODELABEL(cgu), core_clock) ==
		YTM32_CGU_PLL_OUT / DT_PROP(DT_NODELABEL(cgu), core_divider),
	"core-clock != PLL-out / core-divider (PLL-out = ref/refdiv * fbdiv / 2)");

#if defined(CONFIG_SOC_YTM32B1MD1)
/* (6) PLL reference divider: 1 or 3–16 (value 2 is unsupported; clock.ts). */
BUILD_ASSERT(
	!YTM32_CGU_IS_PLL ||
	(YTM32_CGU_PLL_REFDIV == 1U) ||
	((YTM32_CGU_PLL_REFDIV >= 3U) && (YTM32_CGU_PLL_REFDIV <= 16U)),
	"pll-reference-divider out of spec: allowed 1 or 3-16 (not 2)");

/* (7) PLL feedback divider: 10–63 (clock.ts). */
BUILD_ASSERT(
	!YTM32_CGU_IS_PLL ||
	((YTM32_CGU_PLL_FBDIV >= 10U) && (YTM32_CGU_PLL_FBDIV <= 63U)),
	"pll-feedback-divider out of spec: allowed 10-63");

/* (8) PLL input finput = ref/refdiv must be 8–48 MHz (clock.ts). */
BUILD_ASSERT(
	!YTM32_CGU_IS_PLL ||
	((YTM32_CGU_PLL_FINPUT >= 8000000U) && (YTM32_CGU_PLL_FINPUT <= 48000000U)),
	"PLL finput (ref/refdiv) out of spec: must be 8-48 MHz");

/* (9) PLL output must be 100–200 MHz (clock.ts). */
BUILD_ASSERT(
	!YTM32_CGU_IS_PLL ||
	((YTM32_CGU_PLL_OUT >= 100000000U) && (YTM32_CGU_PLL_OUT <= 200000000U)),
	"PLL output out of spec: must be 100-200 MHz");

/* (10) When PLL is fed from FXOSC, the crystal must be 4–40 MHz (RM §12.1). */
BUILD_ASSERT(
	!YTM32_CGU_IS_PLL ||
	(YTM32_CGU_PLL_REFSRC != YTM32_PLL_REF_FXOSC) ||
	((YTM32_CGU_FXOSC >= 4000000U) && (YTM32_CGU_FXOSC <= 40000000U)),
	"fxosc-frequency out of spec for PLL reference: RM §12.1 allows 4-40 MHz");

/* (11) Core / fast bus / slow bus frequency ceilings (clock.ts). */
BUILD_ASSERT(
	DT_PROP(DT_NODELABEL(cgu), core_clock) <= 120000000U,
	"core-clock exceeds 120 MHz (YTM32B1MD1 max, clock.ts)");
BUILD_ASSERT(
	(DT_PROP(DT_NODELABEL(cgu), core_clock) / YTM32_CGU_FASTDIV) <= 120000000U,
	"fast bus clock exceeds 120 MHz (YTM32B1MD1 max, clock.ts)");
BUILD_ASSERT(
	((DT_PROP(DT_NODELABEL(cgu), core_clock) / YTM32_CGU_FASTDIV) /
		YTM32_CGU_SLOWDIV) <= 40000000U,
	"slow bus clock exceeds 40 MHz (YTM32B1MD1 max, clock.ts)");
#endif /* CONFIG_SOC_YTM32B1MD1 */

#undef YTM32_CGU_SRC
#undef YTM32_CGU_IS_PLL
#undef YTM32_CGU_PLL_REFSRC
#undef YTM32_CGU_PLL_REFDIV
#undef YTM32_CGU_PLL_FBDIV
#undef YTM32_CGU_FXOSC
#undef YTM32_CGU_PLL_REF_HZ
#undef YTM32_CGU_PLL_FINPUT
#undef YTM32_CGU_PLL_OUT
#undef YTM32_CGU_FASTDIV
#undef YTM32_CGU_SLOWDIV

/* ─────────────────────────────────────────────────────────────────────────── */

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
static inline int clock_control_ytm32_hal_status_to_errno(int hal_status)
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
	struct ytm32_soc_clock_config soc_clock;
};

struct clock_control_ytm32_data {
	uint32_t core_rate;
	uint32_t fast_bus_rate;
	uint32_t slow_bus_rate;
	bool system_rates_valid;
	struct k_spinlock lock;
	uint16_t module_refcnt[YTM32_CLOCK_IPC_END];
};

struct clock_control_ytm32_module_clock_config {
	uint32_t clock_id;
	uint32_t clk_src;
	uint32_t divider;
};

static const struct clock_control_ytm32_module_clock_config ytm32_fixed_module_clocks[] = {
	/*
	 * DMA is bus-clocked infrastructure with no functional clock mux, so the
	 * SoC dtsi node carries no functional-clock-source property.  It still
	 * needs its IPC clock gate enabled before DMA_DRV_Init() touches the
	 * controller registers; register it here like the other gate-only
	 * peripherals (the SRCSEL/DIV fields are ignored for DMA, matching the
	 * vendor clock config's periSrc=0/div=0).
	 */
	{ YTM32_CLOCK_DMA, YTM32_CLOCK_SRC_FIRC, 1U },
	/*
	 * TMU is a gate-only trigger-routing block on the bus clock (no functional
	 * clock mux), so its SoC dtsi node carries no functional-clock-source.
	 * Register the gate here like DMA; SRCSEL/DIV are ignored for TMU.
	 */
	{ YTM32_CLOCK_TMU, YTM32_CLOCK_SRC_FIRC, 1U },
	{ YTM32_CLOCK_CIM, YTM32_CLOCK_SRC_FIRC, 1U },
	{ YTM32_CLOCK_GPIO, YTM32_CLOCK_SRC_FIRC, 1U },
	{ YTM32_CLOCK_PCTRLA, YTM32_CLOCK_SRC_FIRC, 1U },
	{ YTM32_CLOCK_PCTRLB, YTM32_CLOCK_SRC_FIRC, 1U },
	{ YTM32_CLOCK_PCTRLC, YTM32_CLOCK_SRC_FIRC, 1U },
	{ YTM32_CLOCK_PCTRLD, YTM32_CLOCK_SRC_FIRC, 1U },
	{ YTM32_CLOCK_PCTRLE, YTM32_CLOCK_SRC_FIRC, 1U },
	{ YTM32_CLOCK_SPI0, YTM32_CLOCK_SRC_FIRC, 1U },
	{ YTM32_CLOCK_SPI1, YTM32_CLOCK_SRC_FIRC, 1U },
	{ YTM32_CLOCK_SPI2, YTM32_CLOCK_SRC_FIRC, 1U },
#if defined(YTM32_CLOCK_SPI3)
	{ YTM32_CLOCK_SPI3, YTM32_CLOCK_SRC_FIRC, 1U },
#endif
};

#define YTM32_DT_FUNCTIONAL_CLOCK_ENTRY(node_id) \
	{ \
		.clock_id = DT_CLOCKS_CELL(node_id, id), \
		.clk_src = DT_PROP(node_id, ytmicro_functional_clock_source), \
		.divider = DT_PROP_OR(node_id, ytmicro_functional_clock_divider, 1U), \
	},

#define YTM32_APPEND_DTS_CLOCK_IF_HAS_FUNC_SRC(node_id) \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, clocks), \
		(COND_CODE_1(DT_NODE_HAS_PROP(node_id, ytmicro_functional_clock_source), \
			(YTM32_DT_FUNCTIONAL_CLOCK_ENTRY(node_id)), ())), ())

static const struct clock_control_ytm32_module_clock_config ytm32_dts_module_clocks[] = {
	DT_FOREACH_CHILD_STATUS_OKAY(DT_PATH(soc),
				     YTM32_APPEND_DTS_CLOCK_IF_HAS_FUNC_SRC)
};

#undef YTM32_APPEND_DTS_CLOCK_IF_HAS_FUNC_SRC
#undef YTM32_DT_FUNCTIONAL_CLOCK_ENTRY

static const struct clock_control_ytm32_module_clock_config *ytm32_find_module_clock(uint32_t clock_id)
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


static const char *const ytm32_clock_labels[] = {
	[YTM32_CLOCK_DMA] = "dma",
#if defined(YTM32_CLOCK_TRACE)
	[YTM32_CLOCK_TRACE] = "trace",
#endif
#if defined(YTM32_CLOCK_EFM)
	[YTM32_CLOCK_EFM] = "efm",
#endif
	[YTM32_CLOCK_GPIO] = "gpio",
	[YTM32_CLOCK_PCTRLA] = "pctrla",
	[YTM32_CLOCK_PCTRLB] = "pctrlb",
	[YTM32_CLOCK_PCTRLC] = "pctrlc",
	[YTM32_CLOCK_PCTRLD] = "pctrld",
	[YTM32_CLOCK_PCTRLE] = "pctrle",
#if defined(CONFIG_SOC_YTM32B1MD1)
	[YTM32_CLOCK_LINFLEXD0] = "linflexd0",
	[YTM32_CLOCK_LINFLEXD1] = "linflexd1",
	[YTM32_CLOCK_LINFLEXD2] = "linflexd2",
#else
	[YTM32_CLOCK_UART0] = "uart0",
	[YTM32_CLOCK_UART1] = "uart1",
	[YTM32_CLOCK_UART2] = "uart2",
#endif
#if defined(YTM32_CLOCK_SENT0)
	[YTM32_CLOCK_SENT0] = "sent0",
#endif
	[YTM32_CLOCK_I2C0] = "i2c0",
	[YTM32_CLOCK_I2C1] = "i2c1",
	[YTM32_CLOCK_SPI0] = "spi0",
	[YTM32_CLOCK_SPI1] = "spi1",
	[YTM32_CLOCK_SPI2] = "spi2",
#if defined(YTM32_CLOCK_SPI3)
	[YTM32_CLOCK_SPI3] = "spi3",
#endif
	[YTM32_CLOCK_FLEXCAN0] = "flexcan0",
	[YTM32_CLOCK_FLEXCAN1] = "flexcan1",
#if defined(YTM32_CLOCK_FLEXCAN2)
	[YTM32_CLOCK_FLEXCAN2] = "flexcan2",
#endif
	[YTM32_CLOCK_ADC0] = "adc0",
	[YTM32_CLOCK_ACMP0] = "acmp0",
#if defined(YTM32_CLOCK_PTU0)
	[YTM32_CLOCK_PTU0] = "ptu0",
#endif
	[YTM32_CLOCK_TMU] = "tmu",
	[YTM32_CLOCK_ETMR0] = "etmr0",
	[YTM32_CLOCK_ETMR1] = "etmr1",
#if defined(YTM32_CLOCK_ETMR2)
	[YTM32_CLOCK_ETMR2] = "etmr2",
#endif
#if defined(YTM32_CLOCK_ETMR3)
	[YTM32_CLOCK_ETMR3] = "etmr3",
#endif
#if defined(YTM32_CLOCK_MPWM0)
	[YTM32_CLOCK_MPWM0] = "mpwm0",
#endif
#if defined(YTM32_CLOCK_TMR0)
	[YTM32_CLOCK_TMR0] = "tmr0",
#endif
	[YTM32_CLOCK_PTMR0] = "ptmr0",
	[YTM32_CLOCK_LPTMR0] = "lptmr0",
#if defined(YTM32_CLOCK_RTC)
	[YTM32_CLOCK_RTC] = "rtc",
#endif
#if defined(YTM32_CLOCK_REGFILE)
	[YTM32_CLOCK_REGFILE] = "regfile",
#endif
#if defined(YTM32_CLOCK_WKU)
	[YTM32_CLOCK_WKU] = "wku",
#endif
	[YTM32_CLOCK_CRC] = "crc",
	[YTM32_CLOCK_TRNG] = "trng",
	[YTM32_CLOCK_HCU] = "hcu",
	[YTM32_CLOCK_WDG0] = "wdg0",
	[YTM32_CLOCK_EWDG0] = "ewdg0",
	[YTM32_CLOCK_EMU0] = "emu0",
	[YTM32_CLOCK_STU] = "stu",
	[YTM32_CLOCK_CIM] = "cim",
	[YTM32_CLOCK_SCU] = "scu",
#if defined(YTM32_CLOCK_PCU)
	[YTM32_CLOCK_PCU] = "pcu",
#endif
#if defined(YTM32_CLOCK_RCU)
	[YTM32_CLOCK_RCU] = "rcu",
#endif
	[YTM32_CLOCK_CORE] = "core",
	[YTM32_CLOCK_FAST_BUS] = "fast_bus",
	[YTM32_CLOCK_SLOW_BUS] = "slow_bus",
};

static const char *ytm32_clock_label(uint32_t clock_id)
{
	if ((clock_id < ARRAY_SIZE(ytm32_clock_labels)) &&
	    (ytm32_clock_labels[clock_id] != NULL)) {
		return ytm32_clock_labels[clock_id];
	}

	return "unknown";
}

static bool clock_control_ytm32_is_supported_module_source(uint32_t clk_src)
{
	switch (clk_src) {
	case YTM32_CLOCK_SRC_FIRC:
	case YTM32_CLOCK_SRC_SIRC:
	case YTM32_CLOCK_SRC_FXOSC:
#if defined(YTM32_CLOCK_SRC_LPO)
	case YTM32_CLOCK_SRC_LPO:
#endif
#if defined(YTM32_CLOCK_SRC_FAST_BUS)
	case YTM32_CLOCK_SRC_FAST_BUS:
#endif
#if defined(YTM32_CLOCK_SRC_PLL)
	case YTM32_CLOCK_SRC_PLL:
#endif
		return true;
	default:
		return false;
	}
}

static int clock_control_ytm32_validate_module_clock(const struct clock_control_ytm32_module_clock_config *clock_cfg,
					       uint32_t clock_id)
{
	if (clock_cfg == NULL) {
		return -EINVAL;
	}

	if (!clock_control_ytm32_is_supported_module_source(clock_cfg->clk_src)) {
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

static int clock_control_ytm32_log_system_rate(uint32_t clock_id, const char *label)
{
	uint32_t rate = 0U;
	int hal_status = CLOCK_SYS_GetFreq(clock_id, &rate);
	int ret = clock_control_ytm32_hal_status_to_errno(hal_status);

	if (ret != 0) {
		LOG_ERR("Failed to query %s rate (HAL status %d)", label, hal_status);
		return ret;
	}

	LOG_INF("%s rate: %u Hz", label, rate);
	return 0;
}

static bool clock_control_ytm32_is_system_clock_id(uint32_t clock_id)
{
	return (clock_id == YTM32_CLOCK_CORE) ||
	       (clock_id == YTM32_CLOCK_FAST_BUS) ||
	       (clock_id == YTM32_CLOCK_SLOW_BUS);
}

static int clock_control_ytm32_query_clock_rate(uint32_t clock_id, const char *label,
				       uint32_t *rate)
{
	int hal_status;
	int ret;

	if (rate == NULL) {
		return -EINVAL;
	}

	hal_status = CLOCK_SYS_GetFreq(clock_id, rate);
	ret = clock_control_ytm32_hal_status_to_errno(hal_status);

	if (ret != 0) {
		LOG_ERR("Failed to query %s rate (HAL status %d)", label, hal_status);
	}

	return ret;
}

static int clock_control_ytm32_module_clock_request(const struct device *dev, uint32_t clock_id,
				      const struct clock_control_ytm32_module_clock_config *clock_cfg,
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

static int clock_control_ytm32_refresh_system_rates(const struct device *dev)
{
	const struct clock_control_ytm32_config *cfg = dev->config;
	struct clock_control_ytm32_data *data = dev->data;
	const struct ytm32_soc_clock_config *soc_clock = &cfg->soc_clock;
	uint32_t expected_fast_rate = soc_clock->core_clock / soc_clock->fast_bus_divider;
	uint32_t expected_slow_rate = expected_fast_rate / soc_clock->slow_bus_divider;
	int ret;

	ret = clock_control_ytm32_query_clock_rate(YTM32_CLOCK_CORE, "core",
				       &data->core_rate);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_ytm32_query_clock_rate(YTM32_CLOCK_FAST_BUS, "fast bus",
				       &data->fast_bus_rate);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_ytm32_query_clock_rate(YTM32_CLOCK_SLOW_BUS, "slow bus",
				       &data->slow_bus_rate);
	if (ret < 0) {
		return ret;
	}

	if (data->core_rate != soc_clock->core_clock) {
		LOG_ERR("Core clock mismatch: DTS %u Hz, HW %u Hz",
			soc_clock->core_clock, data->core_rate);
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
	const struct clock_control_ytm32_module_clock_config *clock_cfg;
	int ret;
	
	clock_cfg = ytm32_find_module_clock(clock_id);
	if (clock_cfg == NULL) {
		LOG_ERR("Unsupported YTM32 module clock %s (%u)",
			ytm32_clock_label(clock_id), clock_id);
		return -ENOTSUP;
	}

	if (clock_control_ytm32_validate_module_clock(clock_cfg, clock_id) < 0) {
		return -EINVAL;
	}

	LOG_INF("Enable YTM32 clock %s (%u) src %u div %u",
		ytm32_clock_label(clock_id), clock_id,
		clock_cfg->clk_src, clock_cfg->divider);

	ret = clock_control_ytm32_module_clock_request(dev, clock_id, clock_cfg, true);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int clock_control_ytm32_off(const struct device *dev, clock_control_subsys_t sys)
{
	uint32_t clock_id = (uint32_t)(uintptr_t)sys;
	const struct clock_control_ytm32_module_clock_config *clock_cfg;
	int ret;

	if (clock_control_ytm32_is_system_clock_id(clock_id)) {
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

	if (clock_control_ytm32_validate_module_clock(clock_cfg, clock_id) < 0) {
		return -EINVAL;
	}

	LOG_INF("Disable YTM32 clock %s (%u)",
		ytm32_clock_label(clock_id), clock_id);

	ret = clock_control_ytm32_module_clock_request(dev, clock_id, clock_cfg, false);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int clock_control_ytm32_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate)
{
	struct clock_control_ytm32_data *data = dev->data;
	uint32_t clock_id = (uint32_t)(uintptr_t)sys;
	const struct clock_control_ytm32_module_clock_config *clock_cfg;
	int ret;

	if (rate == NULL) {
		return -EINVAL;
	}

	if (clock_control_ytm32_is_system_clock_id(clock_id)) {
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

	clock_cfg = ytm32_find_module_clock(clock_id);
	if (clock_cfg == NULL) {
		LOG_ERR("Unsupported YTM32 clock %s (%u)",
			ytm32_clock_label(clock_id), clock_id);
		return -ENOTSUP;
	}

	ret = clock_control_ytm32_query_clock_rate(clock_id, ytm32_clock_label(clock_id), rate);
	if ((ret == 0) && (*rate == 0U)) {
		/* Fallback for modules lacking IPC clock source selection */
		uint32_t src_clock_id = 0U;
		switch (clock_cfg->clk_src) {
		case YTM32_CLOCK_SRC_FIRC: src_clock_id = YTM32_CLOCK_IPC_FIRC; break;
		case YTM32_CLOCK_SRC_SIRC: src_clock_id = YTM32_CLOCK_IPC_SIRC; break;
		case YTM32_CLOCK_SRC_FXOSC: src_clock_id = YTM32_CLOCK_IPC_FXOSC; break;
#if defined(YTM32_CLOCK_SRC_LPO) && defined(YTM32_CLOCK_IPC_LPO)
		case YTM32_CLOCK_SRC_LPO: src_clock_id = YTM32_CLOCK_IPC_LPO; break;
#endif
#if defined(YTM32_CLOCK_SRC_FAST_BUS)
		case YTM32_CLOCK_SRC_FAST_BUS: src_clock_id = YTM32_CLOCK_FAST_BUS; break;
#endif
#if defined(YTM32_CLOCK_SRC_PLL) && defined(YTM32_CLOCK_IPC_PLL)
		case YTM32_CLOCK_SRC_PLL: src_clock_id = YTM32_CLOCK_IPC_PLL; break;
#endif
		default: src_clock_id = 0U; break;
		}

		if (src_clock_id != 0U) {
			ret = clock_control_ytm32_query_clock_rate(src_clock_id, "source_clock", rate);
			if (ret == 0) {
				*rate /= clock_cfg->divider;
			}
		} else {
			ret = -ENOTSUP;
		}
	}

	return ret;

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
	const struct ytm32_soc_clock_config *soc_clock = &cfg->soc_clock;
	int ret;

	if ((soc_clock->fast_bus_divider == 0U) || (soc_clock->slow_bus_divider == 0U) ||
	    (soc_clock->core_divider == 0U)) {
		LOG_ERR("Missing YTM32 clock divider in DTS");
		return -EINVAL;
	}

	ret = ytm32_soc_apply_clock_config(soc_clock);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("YTM32 CGU Initialized, Target Core Clock: %u Hz", soc_clock->core_clock);
	LOG_INF("Clock Source: %u, Core Divider: %u, Fast Bus Divider: %u, Slow Bus Divider: %u",
		soc_clock->system_clock_source, soc_clock->core_divider,
		soc_clock->fast_bus_divider, soc_clock->slow_bus_divider);
	LOG_INF("YTM32 FXOSC frequency: %u Hz%s", soc_clock->fxosc_frequency,
		soc_clock->fxosc_bypass ? " (bypass)" : "");
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

	ret = clock_control_ytm32_refresh_system_rates(dev);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_ytm32_log_system_rate(YTM32_CLOCK_CORE, "core");
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_ytm32_log_system_rate(YTM32_CLOCK_FAST_BUS, "fast bus");
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_ytm32_log_system_rate(YTM32_CLOCK_SLOW_BUS, "slow bus");
	if (ret < 0) {
		return ret;
	}

	return 0;
}

#define YTM32_CGU_INIT(n) \
	static const struct clock_control_ytm32_config clock_control_ytm32_config_##n = { \
		.soc_clock = { \
			.system_clock_source = DT_INST_PROP_OR(n, system_clock_source, YTM32_SYSTEM_CLOCK_SRC_FIRC), \
			.fxosc_frequency = DT_INST_PROP_OR(n, fxosc_frequency, YTM32_FXOSC_HZ), \
			.fxosc_bypass = DT_INST_PROP(n, fxosc_bypass), \
			.fxosc_gain_selection = DT_INST_PROP_OR(n, fxosc_gain_selection, 6), \
			.pll_reference_clock = DT_INST_PROP_OR(n, pll_reference_clock, YTM32_PLL_REF_FXOSC), \
			.pll_feedback_divider = DT_INST_PROP_OR(n, pll_feedback_divider, 1), \
			.pll_reference_divider = DT_INST_PROP_OR(n, pll_reference_divider, 1), \
			.core_clock = DT_INST_PROP(n, core_clock), \
			.core_divider = DT_INST_PROP(n, core_divider), \
			.fast_bus_divider = YTM32_DT_FAST_BUS_DIVIDER(n), \
			.slow_bus_divider = YTM32_DT_SLOW_BUS_DIVIDER(n), \
		}, \
	}; \
	static struct clock_control_ytm32_data clock_control_ytm32_data_##n; \
	DEVICE_DT_INST_DEFINE(n, clock_control_ytm32_init, NULL, &clock_control_ytm32_data_##n, &clock_control_ytm32_config_##n, \
			      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY, \
			      &clock_control_ytm32_api);

DT_INST_FOREACH_STATUS_OKAY(YTM32_CGU_INIT)
