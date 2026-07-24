/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SOC_H_
#define SOC_H_

#if defined(CONFIG_SOC_YTM32B1MD1)
#include <YTM32B1MD1.h>
#else
#include <YTM32B1MC0.h>
#endif

/* CMSIS Compatibility */
#define SVCall_IRQn SVC_IRQn
#define MemoryManagement_IRQn MemManage_IRQn

#include <cmsis_core_m_defaults.h>

#endif /* SOC_H_ */
