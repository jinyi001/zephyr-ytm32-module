/*
 * Copyright (c) 2026 YTMicro
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_YTM32B1MD1_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_YTM32B1MD1_CLOCK_H_

/* IPC clocks mapped from YTM32B1MD1 clock_names_t in the vendor SDK */
#define YTM32_CLOCK_DMA                  0U
#define YTM32_CLOCK_TRACE                1U
#define YTM32_CLOCK_EFM                  2U
#define YTM32_CLOCK_GPIO                 3U
#define YTM32_CLOCK_PCTRLA               4U
#define YTM32_CLOCK_PCTRLB               5U
#define YTM32_CLOCK_PCTRLC               6U
#define YTM32_CLOCK_PCTRLD               7U
#define YTM32_CLOCK_PCTRLE               8U
#define YTM32_CLOCK_LINFLEXD0            9U
#define YTM32_CLOCK_LINFLEXD1            10U
#define YTM32_CLOCK_LINFLEXD2            11U
#define YTM32_CLOCK_UART0                YTM32_CLOCK_LINFLEXD0
#define YTM32_CLOCK_UART1                YTM32_CLOCK_LINFLEXD1
#define YTM32_CLOCK_UART2                YTM32_CLOCK_LINFLEXD2
#define YTM32_CLOCK_SENT0                12U
#define YTM32_CLOCK_I2C0                 13U
#define YTM32_CLOCK_I2C1                 14U
#define YTM32_CLOCK_SPI0                 15U
#define YTM32_CLOCK_SPI1                 16U
#define YTM32_CLOCK_SPI2                 17U
#define YTM32_CLOCK_SPI3                 18U
#define YTM32_CLOCK_FLEXCAN0             19U
#define YTM32_CLOCK_FLEXCAN1             20U
#define YTM32_CLOCK_FLEXCAN2             21U
#define YTM32_CLOCK_ADC0                 22U
#define YTM32_CLOCK_ACMP0                23U
#define YTM32_CLOCK_PTU0                 24U
#define YTM32_CLOCK_TMU                  25U
#define YTM32_CLOCK_ETMR0                26U
#define YTM32_CLOCK_ETMR1                27U
#define YTM32_CLOCK_ETMR2                28U
#define YTM32_CLOCK_ETMR3                29U
#define YTM32_CLOCK_TMR0                 30U
#define YTM32_CLOCK_PTMR0                31U
#define YTM32_CLOCK_LPTMR0               32U
#define YTM32_CLOCK_RTC                  33U
#define YTM32_CLOCK_REGFILE              34U
#define YTM32_CLOCK_WKU                  35U
#define YTM32_CLOCK_CRC                  36U
#define YTM32_CLOCK_TRNG                 37U
#define YTM32_CLOCK_HCU                  38U
#define YTM32_CLOCK_WDG0                 39U
#define YTM32_CLOCK_EWDG0                40U
#define YTM32_CLOCK_EMU0                 41U
#define YTM32_CLOCK_STU                  42U
#define YTM32_CLOCK_CIM                  43U
#define YTM32_CLOCK_SCU                  44U
#define YTM32_CLOCK_PCU                  45U
#define YTM32_CLOCK_RCU                  46U

#define YTM32_CLOCK_IPC_PERI_END         47U
#define YTM32_CLOCK_IPC_SIRC             48U
#define YTM32_CLOCK_IPC_FIRC             49U
#define YTM32_CLOCK_IPC_FXOSC            50U
#define YTM32_CLOCK_IPC_PLL              51U

#define YTM32_CLOCK_IPC_END              52U
#define YTM32_CLOCK_CORE                 53U
#define YTM32_CLOCK_FAST_BUS             54U
#define YTM32_CLOCK_SLOW_BUS             55U

/* IPC functional clock source values mapped from YTM32B1MD1 peripheral_clock_source_t */
#define YTM32_CLOCK_SRC_DISABLED         0U
#define YTM32_CLOCK_SRC_FIRC             1U
#define YTM32_CLOCK_SRC_SIRC             2U
#define YTM32_CLOCK_SRC_FXOSC            3U
#define YTM32_CLOCK_SRC_PLL              5U

/* Fixed oscillator frequencies (Hz) — matches DTS firc/sirc-frequency in ytm32b1md1.dtsi */
#define YTM32_FIRC_HZ                    96000000U  /* RM §12.1 FEATURE_SCU_FIRC_FREQ  */
#define YTM32_SIRC_HZ                    12000000U  /* RM §12.1 FEATURE_SCU_SIRC_FREQ  */
#define YTM32_FXOSC_HZ                   24000000U  /* SDK EVB default; board sets own  */

#define YTM32_SYSTEM_CLOCK_SRC_FIRC      0U
#define YTM32_SYSTEM_CLOCK_SRC_FXOSC     1U
#define YTM32_SYSTEM_CLOCK_SRC_PLL       2U

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_YTM32B1MD1_CLOCK_H_ */
