/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * YTM32B1MC0 ECC SRAM initialization.
 *
 * This chip has ECC-protected SRAM. After power-on reset, ECC checkbits are
 * in an undefined state. Any read from an uninitialized RAM address triggers
 * an ECC error (bus fault). ALL 32KB of SRAM must be written before it can
 * be safely read.
 *
 * This routine runs via CONFIG_SOC_EARLY_RESET_HOOK, which is called from
 * Zephyr's reset.S BEFORE the stack pointer is set up and before any C code
 * or RAM access occurs. It uses only registers (no stack) to fill the entire
 * SRAM range with a known pattern.
 *
 * Memory map:
 *   SRAM start: 0x20000000
 *   SRAM end:   0x20008000 (32KB)
 */

.syntax unified
.thumb

.global soc_early_reset_hook
.type soc_early_reset_hook, %function
.section .text.soc_early_reset_hook, "ax"
.align 2

soc_early_reset_hook:
    /* Fill pattern: 0x5A5A5A5A (same as vendor SDK RamInit0) */
    ldr r0, =0x5A5A5A5A
    mov r1, r0

    /* SRAM range: 0x20000000 - 0x20008000 (32 KB) */
    ldr r2, =0x20000000
    ldr r3, =0x20008000

.L_ecc_fill_loop:
    stm r2!, {r0, r1}      /* Store 8 bytes per iteration, auto-increment r2 */
    cmp r2, r3
    blo .L_ecc_fill_loop    /* Loop until r2 >= r3 */

    bx lr                   /* Return to reset.S */

.size soc_early_reset_hook, . - soc_early_reset_hook
