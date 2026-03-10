# Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(${SOC_SERIES})

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")

