/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * CIM (Cross-trigger Interface Module) ADC trigger-select constants for the
 * YTM32B1MD1 SoC.  These are the ADC0_TRIG_SEL field values in CIM_CTRL.
 * Used in the devicetree ytmicro,cim-trigger-select property.
 */

#ifndef ZEPHYR_DT_BINDINGS_ADC_YTM32B1MD1_ADC_H_
#define ZEPHYR_DT_BINDINGS_ADC_YTM32B1MD1_ADC_H_

/* CIM ADC0_TRIG_SEL field values (YTM32B1MD1 SVD, CIM_CTRL register). */
#define YTM32B1MD1_CIM_ADC0_TRIG_SEL_PTU         0U  /* PTU output */
#define YTM32B1MD1_CIM_ADC0_TRIG_SEL_TMU_BUSCLK  1U  /* TMU output, bus-clock sync */
#define YTM32B1MD1_CIM_ADC0_TRIG_SEL_TMU_ADCCLK  2U  /* TMU output, ADC-clock sync (recommended) */

#endif /* ZEPHYR_DT_BINDINGS_ADC_YTM32B1MD1_ADC_H_ */
