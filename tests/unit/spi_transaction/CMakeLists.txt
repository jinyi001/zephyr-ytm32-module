# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(spi_transaction)

target_include_directories(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../../../drivers/spi
)
target_sources(app PRIVATE
	src/main.c
	../../../drivers/spi/spi_ytm32_transaction.c
)
