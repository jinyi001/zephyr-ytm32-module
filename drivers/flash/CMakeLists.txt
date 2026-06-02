# Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
# SPDX-License-Identifier: Apache-2.0

if(TARGET drivers__flash)
	set_property(TARGET drivers__flash PROPERTY ALLOW_EMPTY TRUE)
endif()

if(CONFIG_FLASH_YTM32)
zephyr_library()
zephyr_library_sources(flash_ytm32.c)
zephyr_linker_sources(RAMFUNC_SECTION flash_ytm32_ramfunc.ld)
endif()
