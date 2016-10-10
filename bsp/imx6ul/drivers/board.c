/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2016, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-11-20     Bernard    the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include "board.h"
#include <cortex_a.h>

#undef ALIGN
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>

#define CONFIG_WD_PERIOD 4000000
void udelay(unsigned long usec)
{
	ulong kv;

	do {
		kv = usec > CONFIG_WD_PERIOD ? CONFIG_WD_PERIOD : usec;
		__udelay (kv);
		usec -= kv;
	} while(usec);
}
void mdelay(unsigned long msec)
{
	while (msec--)
		udelay(1000);
}

#define EPIT_BASE    EPIT1_BASE_ADDR
#define __REG(x)     (*((volatile unsigned long *)(x)))
static void rt_hw_timer_isr(int vector, void *param)
{
    rt_tick_increase();
    __REG(EPIT_BASE+4) = 0x01;
}

int rt_hw_timer_init(void)
{
    uint32_t clk = mxc_get_clock(MXC_IPG_CLK);

    __REG(EPIT_BASE) = 0x00010000;
    while (__REG(EPIT_BASE) & 0x00010000);

    __REG(EPIT_BASE) = ((clk/1000000-1)<<4)|0x012a000a;

    __REG(EPIT_BASE+8) = 999;
    __REG(EPIT_BASE) = __REG(EPIT_BASE)|0x05;

    rt_hw_interrupt_install(IMX_INT_EPIT1, rt_hw_timer_isr, RT_NULL, "tick");
    rt_hw_interrupt_umask(IMX_INT_EPIT1);

    return 0;
}
INIT_BOARD_EXPORT(rt_hw_timer_init);

/**
 * This function will initialize hardware board
 */
extern unsigned imx_ddr_size(void);
void rt_hw_board_init(void)
{
    timer_init();
    rt_hw_interrupt_init();
    enable_neon_fpu();

    /* initialize uart */
    rt_hw_uart_init();
    rt_console_set_device(CONSOLE_DEVICE);

    /* initialize timer */
    rt_hw_timer_init();

#ifdef RT_USING_MTD_NAND
    /* initialize gpmi */
    rt_hw_mtd_nand_init();
#endif

#ifdef RT_USING_SDIO
    /* initialize ssp */
    rt_hw_ssp_init();
#endif

    rt_kprintf("\n\n");
    print_cpuinfo();
    rt_kprintf("MEM:   %d MiB\n",imx_ddr_size()/2048/1024);
}

/*@}*/
