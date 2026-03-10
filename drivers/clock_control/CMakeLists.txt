# Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_CLOCK_CONTROL_YTM32 clock_control_ytm32.c)
