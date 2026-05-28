/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * soc_reset_hook — called from Zephyr reset.S (CONFIG_SOC_RESET_HOOK) after
 * the MSP is set but BEFORE relocate_vector_table() changes VTOR.
 *
 * Two responsibilities:
 *
 * 1. Disable SysTick (CTRL = 0).
 *    The YTM32 ROM bootloader may leave SysTick running.  After
 *    relocate_vector_table() our sys_clock_isr() becomes the SysTick
 *    handler.  If a tick fires between that point and z_data_copy()
 *    completing, sys_clock_announce() sees a stale timeout_list (timeout_list
 *    is .data, not .bss — it holds a self-pointer as its initial value, which
 *    is only valid after z_data_copy()).  This produces an MPU fault from
 *    sys_dlist_remove dereferencing a NULL prev pointer, or an assertion
 *    failure "Threads may not be created in ISRs" depending on the timing.
 *    Disabling SysTick here closes the race completely; sys_clock_driver_init
 *    (PRE_KERNEL_2) re-enables it with correct parameters.
 *
 * 2. ECC SRAM initialisation.
 *    YTM32B1M SRAM is ECC-protected.  ECC check-bits are undefined at
 *    power-on reset; any read from an unwritten address generates a bus
 *    fault.  We write the entire SRAM with a known pattern before any C
 *    code (which may read globals) executes.
 *
 *    SRAM map:
 *      YTM32B1MC0 — 0x20000000 .. 0x20008000  (32 KB)
 *      YTM32B1MD1 — 0x1fff8000 .. 0x20008000  (64 KB)
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
    /* 1.  Disable SysTick + clear its NVIC pending bit before VTOR is    */
    /*     redirected to our ISR table.                                   */
    /*                                                                    */
    /* Clearing CTRL alone is not enough: the YTM32 ROM bootloader can    */
    /* leave the SysTick interrupt PENDING in SCB->ICSR.  Once Zephyr     */
    /* releases BASEPRI after z_prep_c, the still-pending IRQ fires       */
    /* through the new VTOR into sys_clock_isr, which then dereferences   */
    /* timeout_list before z_data_copy has initialised it.                */
    /* ------------------------------------------------------------------ */
    ldr  r0, =0xE000E010        /* SysTick->CTRL                          */
    movs r1, #0
    str  r1, [r0]               /* CTRL = 0: disable counter + interrupt  */

    ldr  r0, =0xE000ED04        /* SCB->ICSR                              */
    ldr  r1, =0x02000000        /* PENDSTCLR (bit 25): clear pending SysTick */
    str  r1, [r0]
    dsb                         /* ensure both writes have taken effect    */

    /* ------------------------------------------------------------------ */
    /* 2.  ECC SRAM initialisation — write every word before any read.    */
    /* ------------------------------------------------------------------ */
    ldr  r0, =0x5A5A5A5A        /* fill pattern (matches vendor RamInit0) */
    mov  r1, r0

#if defined(CPU_YTM32B1MD1)
    ldr  r2, =0x1fff8000        /* MD1: 64 KB SRAM starts here           */
#else
    ldr  r2, =0x20000000        /* MC0: 32 KB SRAM starts here           */
#endif
    ldr  r3, =0x20008000        /* both SoCs end at the same address     */

.L_ecc_fill_loop:
    stm  r2!, {r0, r1}          /* store 8 bytes, advance r2             */
    cmp  r2, r3
    blo  .L_ecc_fill_loop

    bx   lr

.size soc_reset_hook, . - soc_reset_hook
