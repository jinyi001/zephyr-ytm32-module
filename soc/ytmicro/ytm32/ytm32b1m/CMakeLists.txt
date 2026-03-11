# Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

if(CONFIG_SOC_YTM32B1MC0)
  zephyr_compile_definitions(CPU_YTM32B1MC0)
endif()

zephyr_sources(soc.c)

# Workaround for Zephyr not passing +nofp or -mfloat-abi=soft for Cortex-M33 without FPU
if(NOT CONFIG_CPU_HAS_FPU)
  zephyr_cc_option(-mfloat-abi=soft)
endif()
