/*
 * Copyright (c) 2026 YI JIN <jinyi_2001@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/init.h>
#include "interrupt_manager.h"

/* --- Vendor HAL Interrupt Manager Overrides --- */

static int32_t g_interruptDisableCount = 0;

void INT_SYS_InstallHandler(IRQn_Type irqNumber, isr_t newHandler, isr_t *oldHandler)
{
    /* 
     * In Zephyr, interrupts are managed via IRQ_CONNECT in drivers.
     * We override this vendor HAL function to prevent it from modifying
     * the vector table (VTOR), which is in Flash and would cause a Bus Fault.
     */
    ARG_UNUSED(irqNumber);
    ARG_UNUSED(newHandler);
    ARG_UNUSED(oldHandler);
}

void INT_SYS_EnableIRQ(IRQn_Type irqNumber)
{
    NVIC_EnableIRQ(irqNumber);
}

void INT_SYS_DisableIRQ(IRQn_Type irqNumber)
{
    NVIC_DisableIRQ(irqNumber);
}

void INT_SYS_EnableIRQGlobal(void)
{
    if (g_interruptDisableCount > 0)
    {
        g_interruptDisableCount--;
        if (g_interruptDisableCount <= 0)
        {
            __enable_irq();
        }
    }
}

void INT_SYS_DisableIRQGlobal(void)
{
    __disable_irq();
    g_interruptDisableCount++;
}

void INT_SYS_SetPriority(IRQn_Type irqNumber, uint8_t priority)
{
    NVIC_SetPriority(irqNumber, priority);
}

uint8_t INT_SYS_GetPriority(IRQn_Type irqNumber)
{
    return (uint8_t)NVIC_GetPriority(irqNumber);
}

#if FEATURE_INTERRUPT_HAS_PENDING_STATE
void INT_SYS_ClearPending(IRQn_Type irqNumber)
{
    NVIC_ClearPendingIRQ(irqNumber);
}

void INT_SYS_SetPending(IRQn_Type irqNumber)
{
    NVIC_SetPendingIRQ(irqNumber);
}

uint32_t INT_SYS_GetPending(IRQn_Type irqNumber)
{
    return NVIC_GetPendingIRQ(irqNumber);
}
#endif /* FEATURE_INTERRUPT_HAS_PENDING_STATE */

#if FEATURE_INTERRUPT_HAS_ACTIVE_STATE
uint32_t INT_SYS_GetActive(IRQn_Type irqNumber)
{
    return NVIC_GetActive(irqNumber);
}
#endif /* FEATURE_INTERRUPT_HAS_ACTIVE_STATE */

static int intc_ytm32_init(void)
{
    /* 
     * The stubs above replace the need for the vendor HAL interrupt_manager.c.
     * Nothing else needs to be initialized here as Zephyr's standard ARM NVIC
     * code handles actual interrupt routing and enablement.
     */
    return 0;
}

SYS_INIT(intc_ytm32_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
