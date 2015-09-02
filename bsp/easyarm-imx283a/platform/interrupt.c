/*
 * File      : interrupt.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2015, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2011-01-13     weety      first version
 * 2015-04-27     ArdaFu     Port bsp from at91sam9260 to asm9260t
 */

#include <rthw.h>
#include <rtthread.h>
#include "board.h"
#include "regs-icoll.h"

#define MAX_HANDLERS ARCH_NR_IRQS

extern rt_uint32_t rt_interrupt_nest;

/* exception and interrupt handler table */
struct rt_irq_desc irq_desc[MAX_HANDLERS];

rt_uint32_t rt_interrupt_from_thread;
rt_uint32_t rt_interrupt_to_thread;
rt_uint32_t rt_thread_switch_interrupt_flag;

/* --------------------------------------------------------------------
 *  Interrupt initialization
 * -------------------------------------------------------------------- */

rt_isr_handler_t rt_hw_interrupt_handle(rt_uint32_t vector, void *param)
{
    rt_kprintf("UN-handled interrupt %d occurred!!!\n", vector);
    return RT_NULL;
}

/**
 * This function will initialize hardware interrupt
 */
void rt_hw_interrupt_init(void)
{
    register rt_uint32_t idx;

	/* Reset icoll */
	REG_WR_ADDR(REGS_ICOL_BASE + HW_ICOLL_CTRL_CLR, BM_ICOLL_CTRL_SFTRST);
	while (REG_RD_ADDR(REGS_ICOL_BASE + HW_ICOLL_CTRL) & BM_ICOLL_CTRL_SFTRST)
		;

	REG_WR_ADDR(REGS_ICOL_BASE + HW_ICOLL_CTRL_CLR, BM_ICOLL_CTRL_CLKGATE);

    /* init exceptions table */
    for(idx=0; idx < MAX_HANDLERS; idx++)
    {
		REG_WR_ADDR(REGS_ICOL_BASE + HW_ICOLL_INTERRUPTn(idx), 0);
        irq_desc[idx].handler = (rt_isr_handler_t)rt_hw_interrupt_handle;
        irq_desc[idx].param = RT_NULL;
#ifdef RT_USING_INTERRUPT_INFO
        rt_snprintf(irq_desc[idx].name, RT_NAME_MAX - 1, "null");
        irq_desc[idx].counter = 0;
#endif
    }

    /* init interrupt nest, and context in thread sp */
    rt_interrupt_nest = 0;
    rt_interrupt_from_thread = 0;
    rt_interrupt_to_thread = 0;
    rt_thread_switch_interrupt_flag = 0;

    REG_WR_ADDR(REGS_ICOL_BASE + HW_ICOLL_LEVELACK,
                 BF_ICOLL_LEVELACK_IRQLEVELACK(BV_ICOLL_LEVELACK_IRQLEVELACK__LEVEL0));
    REG_WR_ADDR(REGS_ICOL_BASE + HW_ICOLL_LEVELACK,
                 BF_ICOLL_LEVELACK_IRQLEVELACK(BV_ICOLL_LEVELACK_IRQLEVELACK__LEVEL1));
    REG_WR_ADDR(REGS_ICOL_BASE + HW_ICOLL_LEVELACK,
                 BF_ICOLL_LEVELACK_IRQLEVELACK(BV_ICOLL_LEVELACK_IRQLEVELACK__LEVEL2));
    REG_WR_ADDR(REGS_ICOL_BASE + HW_ICOLL_LEVELACK,
                 BF_ICOLL_LEVELACK_IRQLEVELACK(BV_ICOLL_LEVELACK_IRQLEVELACK__LEVEL3));
    
    /* isr memcpy */
    rt_memcpy((void *)0, (void *)0x40000000, 256);
    REG_WR_ADDR(REGS_ICOL_BASE + HW_ICOLL_VECTOR, 0);
    /* Barrier */
    (void)REG_RD_ADDR(REGS_ICOL_BASE + HW_ICOLL_STAT);    
}

/**
 * This function will mask a interrupt.
 * @param vector the interrupt number
 */
void rt_hw_interrupt_mask(int irq)
{
    REG_WR_ADDR(REGS_ICOL_BASE + HW_ICOLL_INTERRUPTn_CLR(irq), BM_ICOLL_INTERRUPTn_ENABLE);
}

/**
 * This function will un-mask a interrupt.
 * @param vector the interrupt number
 */
void rt_hw_interrupt_umask(int irq)
{
    REG_WR_ADDR(REGS_ICOL_BASE + HW_ICOLL_INTERRUPTn_SET(irq), BM_ICOLL_INTERRUPTn_ENABLE);
}

/**
 * This function will install a interrupt service routine to a interrupt.
 * @param vector the interrupt number
 * @param handler the interrupt service routine to be installed
 * @param param the interrupt service function parameter
 * @param name the interrupt name
 * @return old handler
 */
rt_isr_handler_t rt_hw_interrupt_install(int vector, rt_isr_handler_t handler,
        void *param, char *name)
{
    rt_isr_handler_t old_handler = RT_NULL;

    if(vector < MAX_HANDLERS)
    {
        old_handler = irq_desc[vector].handler;
        if (handler != RT_NULL)
        {
            irq_desc[vector].handler = (rt_isr_handler_t)handler;
            irq_desc[vector].param = param;
#ifdef RT_USING_INTERRUPT_INFO
            rt_snprintf(irq_desc[vector].name, RT_NAME_MAX - 1, "%s", name);
            irq_desc[vector].counter = 0;
#endif
        }
    }

    return old_handler;
}

rt_uint32_t rt_hw_interrupt_get_active(rt_uint32_t fiq_irq)
{
    rt_uint32_t id;
    /* AIC need this dummy read */
    REG_RD_ADDR(REGS_ICOL_BASE + HW_ICOLL_VECTOR);
    /* get irq number */
    id = REG_RD_ADDR(REGS_ICOL_BASE + HW_ICOLL_STAT);
    return id;
}

void rt_hw_interrupt_ack(rt_uint32_t fiq_irq, rt_uint32_t id)
{
    REG_WR_ADDR(REGS_ICOL_BASE + HW_ICOLL_VECTOR, 0);
    /* ACK current interrupt */
    REG_WR_ADDR(REGS_ICOL_BASE + HW_ICOLL_LEVELACK,
                 BF_ICOLL_LEVELACK_IRQLEVELACK(BV_ICOLL_LEVELACK_IRQLEVELACK__LEVEL0));
    /* Barrier */
    (void)REG_RD_ADDR(REGS_ICOL_BASE + HW_ICOLL_STAT);
}

#ifdef RT_USING_FINSH
void list_irq(void)
{
    int irq;
    rt_kprintf("number\tcount\tname\n");
    for (irq = 0; irq < MAX_HANDLERS; irq++)
    {
        if (rt_strncmp(irq_desc[irq].name, "null", sizeof("null")))
        {
            rt_kprintf("%02ld: %10ld  %s\n",
                       irq, irq_desc[irq].counter, irq_desc[irq].name);
        }
    }
}

#include <finsh.h>
FINSH_FUNCTION_EXPORT(list_irq, list system irq);
MSH_CMD_EXPORT(list_irq, list system irq);
#endif
