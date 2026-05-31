/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * soc_reset_hook — called from Zephyr reset.S (CONFIG_SOC_RESET_HOOK) after
 * the MSP is set but BEFORE relocate_vector_table() changes VTOR.
 *
 * One responsibility:
 *
 * 1. ECC SRAM initialisation.
 *    YTM32B1M SRAM is ECC-protected.  ECC check-bits are undefined at
 *    power-on reset; any read from an unwritten address generates a bus
 *    fault.  We write the entire SRAM with a known pattern before any C
 *    code (which may read globals) executes.
 *
 *    The SRAM range is taken from Zephyr's generated Kconfig values
 *    (CONFIG_SRAM_BASE_ADDRESS / CONFIG_SRAM_SIZE), which are derived from
 *    the active board/devicetree memory definition.
 *
 * Implementation notes:
 *   • Uses only r0-r3 and LR — no push/pop, so the uninitialised stack is
 *     never read (ECC-safe entry and exit).
 *   • Returns via bx lr (register-only, no stack pop).
 */

.syntax unified
.thumb

.global soc_reset_hook
.type   soc_reset_hook, %function
.section .text.soc_reset_hook, "ax"
.align 2

soc_reset_hook:
    /* ------------------------------------------------------------------ */
    /* 1.  ECC SRAM initialisation — write every word before any read.    */
    /* ------------------------------------------------------------------ */
    ldr  r0, =0x5A5A5A5A        /* fill pattern (matches vendor RamInit0) */
    mov  r1, r0

    ldr  r2, =CONFIG_SRAM_BASE_ADDRESS
    ldr  r3, =(CONFIG_SRAM_BASE_ADDRESS + (CONFIG_SRAM_SIZE * 1024))

.L_ecc_fill_loop:
    stm  r2!, {r0, r1}          /* store 8 bytes, advance r2             */
    cmp  r2, r3
    blo  .L_ecc_fill_loop

    bx   lr

.size soc_reset_hook, . - soc_reset_hook
