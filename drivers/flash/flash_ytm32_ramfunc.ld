/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Vendor YTM32 flash driver marks its flash command launcher with
 * __attribute__((section(".code_ram"))).  Fold that input section into
 * Zephyr's .ramfunc output section so it is copied from flash to SRAM at
 * boot by z_data_copy() and executes from RAM during program/erase.
 */
KEEP(*(.code_ram))
KEEP(*(.code_ram.*))
