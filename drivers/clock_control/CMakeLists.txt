# Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
# SPDX-License-Identifier: Apache-2.0

if(TARGET drivers__clock_control)
    set_property(TARGET drivers__clock_control PROPERTY ALLOW_EMPTY TRUE)
endif()

if(CONFIG_CLOCK_CONTROL_YTM32)
zephyr_library()
zephyr_library_sources(clock_control_ytm32.c)
endif()
