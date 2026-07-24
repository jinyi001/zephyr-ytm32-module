/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * TMU (Trigger MUX Unit) source-trigger and target-module identifiers for the
 * YTM32B1MD1 SoC.  Values mirror the tmu_trigger_source_t / tmu_target_module_t
 * enumerations in the vendor HAL (YTM32B1MD1_features.h) and are used in the
 * devicetree to describe TMU routes symbolically instead of with raw numbers.
 */

#ifndef ZEPHYR_DT_BINDINGS_TMU_YTM32B1MD1_TMU_H_
#define ZEPHYR_DT_BINDINGS_TMU_YTM32B1MD1_TMU_H_

/* ── Trigger sources (tmu_trigger_source_t) ──────────────────────────── */
#define YTM32_TMU_SRC_DISABLED            0
#define YTM32_TMU_SRC_VDD                 1
#define YTM32_TMU_SRC_TMU_IN0             2
#define YTM32_TMU_SRC_TMU_IN1             3
#define YTM32_TMU_SRC_TMU_IN2             4
#define YTM32_TMU_SRC_TMU_IN3             5
#define YTM32_TMU_SRC_TMU_IN4             6
#define YTM32_TMU_SRC_TMU_IN5             7
#define YTM32_TMU_SRC_TMU_IN6             8
#define YTM32_TMU_SRC_TMU_IN7             9
#define YTM32_TMU_SRC_TMU_IN8             10
#define YTM32_TMU_SRC_TMU_IN9             11
#define YTM32_TMU_SRC_TMU_IN10            12
#define YTM32_TMU_SRC_TMU_IN11            13
#define YTM32_TMU_SRC_ACMP0_OUT           14
#define YTM32_TMU_SRC_PTMR_CH0            17
#define YTM32_TMU_SRC_PTMR_CH1            18
#define YTM32_TMU_SRC_PTMR_CH2            19
#define YTM32_TMU_SRC_PTMR_CH3            20
#define YTM32_TMU_SRC_LPTMR0              21
#define YTM32_TMU_SRC_ETMR0_INIT_TRIG     22
#define YTM32_TMU_SRC_ETMR0_EXT_TRIG      23
#define YTM32_TMU_SRC_ETMR1_INIT_TRIG     24
#define YTM32_TMU_SRC_ETMR1_EXT_TRIG      25
#define YTM32_TMU_SRC_ETMR2_INIT_TRIG     26
#define YTM32_TMU_SRC_ETMR2_EXT_TRIG      27
#define YTM32_TMU_SRC_ETMR3_INIT_TRIG     28
#define YTM32_TMU_SRC_ETMR3_EXT_TRIG      29
#define YTM32_TMU_SRC_ADC0_EOC            30
#define YTM32_TMU_SRC_ADC0_AWD            31
#define YTM32_TMU_SRC_PTU0_ADCH0_TRIG     34
#define YTM32_TMU_SRC_PTU0_PULSE_OUT0     36
#define YTM32_TMU_SRC_PTU1_ADCH0_TRIG     37
#define YTM32_TMU_SRC_PTU1_PULSE_OUT1     39
#define YTM32_TMU_SRC_RTC_ALARM           43
#define YTM32_TMU_SRC_RTC_SECOND          44
#define YTM32_TMU_SRC_SPI2_FRAME          45
#define YTM32_TMU_SRC_SPI2_RX_DATA        46
#define YTM32_TMU_SRC_SPI3_FRAME          47
#define YTM32_TMU_SRC_SPI3_RX_DATA        48
#define YTM32_TMU_SRC_I2C0_MASTER_TRIG    55
#define YTM32_TMU_SRC_I2C0_SLAVE_TRIG     56
#define YTM32_TMU_SRC_I2C1_MASTER_TRIG    57
#define YTM32_TMU_SRC_SPI0_FRAME          59
#define YTM32_TMU_SRC_SPI0_RX_DATA        60
#define YTM32_TMU_SRC_SPI1_FRAME          61
#define YTM32_TMU_SRC_SPI1_RX_DATA        62
#define YTM32_TMU_SRC_SIM_SW_TRIG         63

/* ── Target modules (tmu_target_module_t) ────────────────────────────── */
#define YTM32_TMU_TARGET_TMU_OUT0         4
#define YTM32_TMU_TARGET_TMU_OUT1         5
#define YTM32_TMU_TARGET_TMU_OUT2         6
#define YTM32_TMU_TARGET_TMU_OUT3         7
#define YTM32_TMU_TARGET_TMU_OUT4         8
#define YTM32_TMU_TARGET_TMU_OUT5         9
#define YTM32_TMU_TARGET_TMU_OUT6         10
#define YTM32_TMU_TARGET_TMU_OUT7         11
#define YTM32_TMU_TARGET_ADC0_EXT_TRIG    12
#define YTM32_TMU_TARGET_LPTMR0_ALT0      21
#define YTM32_TMU_TARGET_ACMP0_SAMPLE_IN  28
#define YTM32_TMU_TARGET_LINFLEXD0_TRIG   32
#define YTM32_TMU_TARGET_LINFLEXD1_TRIG   33
#define YTM32_TMU_TARGET_LINFLEXD2_TRIG   34
#define YTM32_TMU_TARGET_SPI0_TRIG        36
#define YTM32_TMU_TARGET_SPI1_TRIG        37
#define YTM32_TMU_TARGET_SPI2_TRIG        38
#define YTM32_TMU_TARGET_SPI3_TRIG        39
#define YTM32_TMU_TARGET_ETMR0_HWTRIG0    40
#define YTM32_TMU_TARGET_ETMR0_FAULT0     41
#define YTM32_TMU_TARGET_ETMR0_FAULT1     42
#define YTM32_TMU_TARGET_ETMR0_FAULT2     43
#define YTM32_TMU_TARGET_ETMR1_HWTRIG0    44
#define YTM32_TMU_TARGET_ETMR1_FAULT0     45
#define YTM32_TMU_TARGET_ETMR1_FAULT1     46
#define YTM32_TMU_TARGET_ETMR1_FAULT2     47
#define YTM32_TMU_TARGET_ETMR2_HWTRIG0    48
#define YTM32_TMU_TARGET_ETMR2_FAULT0     49
#define YTM32_TMU_TARGET_ETMR2_FAULT1     50
#define YTM32_TMU_TARGET_ETMR2_FAULT2     51
#define YTM32_TMU_TARGET_ETMR3_HWTRIG0    52
#define YTM32_TMU_TARGET_ETMR3_FAULT0     53
#define YTM32_TMU_TARGET_ETMR3_FAULT1     54
#define YTM32_TMU_TARGET_ETMR3_FAULT2     55
#define YTM32_TMU_TARGET_PTU0_TRIG_IN0    56
#define YTM32_TMU_TARGET_I2C1_TRIG        57
#define YTM32_TMU_TARGET_I2C2_TRIG        58
#define YTM32_TMU_TARGET_PTU1_TRIG_IN0    60

#endif /* ZEPHYR_DT_BINDINGS_TMU_YTM32B1MD1_TMU_H_ */
