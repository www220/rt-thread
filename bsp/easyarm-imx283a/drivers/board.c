/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2009 RT-Thread Develop Team
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
 */

#include <rtthread.h>
#include <rthw.h>

#include "board.h"
#include <mmu.h>

static struct mem_desc hw_mem_desc[] =
{
	{ 0x00000000, 0xFFFFFFFF, 0x00000000, RW_NCNB },     /* None cached for 4G memory */
	{ 0x40000000, 0x44000000, 0x40000000, RW_CB },       /* 64M cached SDRAM memory */
	{ 0x00000000, 0x00001000, 0x40000000, RW_CB },       /* isr vector table */
};

void rt_hw_board_init()
{
    /* initialize mmu */
    rt_hw_mmu_init(hw_mem_desc, sizeof(hw_mem_desc)/sizeof(hw_mem_desc[0]));
    /* initialize hardware interrupt */
    rt_hw_interrupt_init();
    
    /* initialize uart */
    rt_hw_usart_init();
#ifdef RT_USING_CONSOLE
    rt_console_set_device(CONSOLE_DEVICE);
#endif

    /* initialize timer0 */
    rt_hw_timer_init();
}

void inittmppath(void)
{
}

void cpu_usage_idle_hook(void)
{
    wtdog_count = 0;
}

/*@}*/
