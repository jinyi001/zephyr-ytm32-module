/*
 * Copyright (c) 2026 YTMicro
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_YTM32_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_YTM32_CLOCK_H_

/* IPC clocks mapped from clock_names_t in vendor SDK */
#define YTM32_CLOCK_DMA                  0U
#define YTM32_CLOCK_GPIO                 1U
#define YTM32_CLOCK_PCTRLA               2U
#define YTM32_CLOCK_PCTRLB               3U
#define YTM32_CLOCK_PCTRLC               4U
#define YTM32_CLOCK_PCTRLD               5U
#define YTM32_CLOCK_PCTRLE               6U
#define YTM32_CLOCK_UART0                7U
#define YTM32_CLOCK_UART1                8U
#define YTM32_CLOCK_UART2                9U
#define YTM32_CLOCK_I2C0                 10U
#define YTM32_CLOCK_I2C1                 11U
#define YTM32_CLOCK_SPI0                 12U
#define YTM32_CLOCK_SPI1                 13U
#define YTM32_CLOCK_SPI2                 14U
#define YTM32_CLOCK_FLEXCAN0             15U
#define YTM32_CLOCK_FLEXCAN1             16U
#define YTM32_CLOCK_ADC0                 17U
#define YTM32_CLOCK_ACMP0                18U
#define YTM32_CLOCK_TMU                  19U
#define YTM32_CLOCK_ETMR0                20U
#define YTM32_CLOCK_ETMR1                21U
#define YTM32_CLOCK_MPWM0                22U
#define YTM32_CLOCK_PTMR0                23U
#define YTM32_CLOCK_LPTMR0               24U
#define YTM32_CLOCK_CRC                  25U
#define YTM32_CLOCK_TRNG                 26U
#define YTM32_CLOCK_HCU                  27U
#define YTM32_CLOCK_WDG0                 28U
#define YTM32_CLOCK_EWDG0                29U
#define YTM32_CLOCK_EMU0                 30U
#define YTM32_CLOCK_STU                  31U
#define YTM32_CLOCK_CIM                  32U
#define YTM32_CLOCK_SCU                  33U

#define YTM32_CLOCK_IPC_PERI_END         34U
#define YTM32_CLOCK_IPC_SIRC             35U
#define YTM32_CLOCK_IPC_FIRC             36U
#define YTM32_CLOCK_IPC_FXOSC            37U
#define YTM32_CLOCK_IPC_LPO              38U

#define YTM32_CLOCK_IPC_END              39U
#define YTM32_CLOCK_CORE                 40U
#define YTM32_CLOCK_FAST_BUS             41U
#define YTM32_CLOCK_SLOW_BUS             42U

/* IPC functional clock source values mapped from peripheral_clock_source_t */
#define YTM32_CLOCK_SRC_DISABLED         0U
#define YTM32_CLOCK_SRC_FIRC             1U
#define YTM32_CLOCK_SRC_SIRC             2U
#define YTM32_CLOCK_SRC_FXOSC            3U
#define YTM32_CLOCK_SRC_LPO              4U
#define YTM32_CLOCK_SRC_FAST_BUS         7U

/* Fixed oscillator frequencies (Hz) */
#define YTM32_FIRC_HZ                    80000000U
#define YTM32_FXOSC_HZ                   24000000U

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_YTM32_CLOCK_H_ */
