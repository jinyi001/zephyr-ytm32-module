# Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=YTM32B1MC03")

board_runner_args(jlink "--iface=swd")
board_runner_args(jlink "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
