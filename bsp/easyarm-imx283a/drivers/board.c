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
#include <rtm.h>

#include "board.h"
#include <stdlib.h>
#include <mmu.h>
#include <time.h>

unsigned long loops_per_jiffy = 0x1bb00;
#define jiffies ((REG_RD(REGS_DIGCTL_BASE, HW_DIGCTL_MICROSECONDS))/1000)

void calibrate_delay(void)
{
	unsigned long ticks, loopbit;
	int lps_precision = 8;

	loops_per_jiffy = (1<<12);
	while ((loops_per_jiffy <<= 1) != 0) {
		/* wait for "start of" clock tick */
		ticks = jiffies;
		while (ticks == jiffies)
			/* nothing */;
		/* Go .. */
		ticks = jiffies; 
		__delay(loops_per_jiffy);
		ticks = jiffies - ticks;
		if (ticks)
			break;
	}

	/*
	 * Do a binary approximation to get loops_per_jiffy set to
	 * equal one clock (up to lps_precision bits)
	 */
	loops_per_jiffy >>= 1;
	loopbit = loops_per_jiffy;
	while (lps_precision-- && (loopbit >>= 1)) {
		loops_per_jiffy |= loopbit;
		ticks = jiffies;
		while (ticks == jiffies)
			/* nothing */;
		ticks = jiffies;
		__delay(loops_per_jiffy);
		if (jiffies != ticks)	/* longer than 1 tick */
			loops_per_jiffy &= ~loopbit;
	}
}

static struct mem_desc hw_mem_desc[] =
{
	{ 0x80000000, 0xFFFFFFFF, 0x80000000, RW_NCNB },     /* None cached for io memory */
	{ 0x40000000, HEAP_END-1, 0x40000000, RW_CB },       /* 63M cached SDRAM memory */
	{ HEAP_END,   HEAP_END+0xFFFFF, HEAP_END, RW_NCNB },     /* 1M none-cached SDRAM */
	{ 0x00000000, 0x00020000, 0x00000000, RW_NCNB },     /* isr vector table */
};
/* None-cached RAM DMA */
unsigned char * dma_align_mem = (unsigned char *)0x00000100;
unsigned char * dma_align_max = (unsigned char *)HEAP_END;
unsigned char * mod_align_max = (unsigned char *)0x40400000;

static struct pin_desc led_pins_desc[] = {
	{ PIN_WDT, PIN_GPIO, PAD_8MA, PAD_3V3, 1 },
	{ PIN_NET0, PIN_GPIO, PAD_8MA, PAD_3V3, 1 },
	{ PIN_BEEP, PIN_GPIO, PAD_8MA, PAD_3V3, 1 },
	{ PIN_RUN, PIN_GPIO, PAD_8MA, PAD_3V3, 1 },
	{ PIN_ERR, PIN_GPIO, PAD_8MA, PAD_3V3, 1 },
	{ PIN_UART1, PIN_GPIO, PAD_4MA, PAD_3V3, 1 },
	{ PIN_UART2, PIN_GPIO, PAD_4MA, PAD_3V3, 1 },
	{ PIN_UART3, PIN_GPIO, PAD_4MA, PAD_3V3, 1 },
	{ PIN_UART4, PIN_GPIO, PAD_4MA, PAD_3V3, 1 },
	{ PIN_GPRS, PIN_GPIO, PAD_4MA, PAD_3V3, 1 },
	{ PIN_PZ1, PIN_GPIO, PAD_4MA, PAD_3V3, 1 },
	{ PIN_PZ2, PIN_GPIO, PAD_4MA, PAD_3V3, 1 },
};
static struct pin_group led_pins = {
	.pins		= led_pins_desc,
	.nr_pins	= ARRAY_SIZE(led_pins_desc)
};

void rt_hw_board_init()
{
    u32 init[10];
    init[0] = REG_RD(REGS_DIGCTL_BASE, HW_DIGCTL_MICROSECONDS);

    /* Set up LED pins */
	pin_set_group(&led_pins);
    
	pin_gpio_direction(PIN_WDT, 1);
	pin_gpio_direction(PIN_BEEP, 1);
	pin_gpio_direction(PIN_RUN, 1);
	pin_gpio_direction(PIN_ERR, 1);
	pin_gpio_direction(PIN_UART1, 1);
	pin_gpio_direction(PIN_UART2, 1);
	pin_gpio_direction(PIN_UART3, 1);
	pin_gpio_direction(PIN_UART4, 1);
	pin_gpio_set(PIN_GPRS, 1);
	pin_gpio_direction(PIN_GPRS, 1);
    pin_gpio_set(PIN_WDT, 0);
    pin_gpio_set(PIN_UART1, 0);
    pin_gpio_set(PIN_UART2, 0);
    pin_gpio_set(PIN_UART3, 0);
    pin_gpio_set(PIN_UART4, 0);
    pin_gpio_set(PIN_GPRS, 1);

    PZ[0] = pin_gpio_get(PIN_PZ1);
    PZ[1] = pin_gpio_get(PIN_PZ2);

    /* initialize mmu */
    rt_hw_mmu_init(hw_mem_desc, sizeof(hw_mem_desc)/sizeof(hw_mem_desc[0]));
    init[1] = REG_RD(REGS_DIGCTL_BASE, HW_DIGCTL_MICROSECONDS);
    /* init udelay */
    calibrate_delay();
    init[2] = REG_RD(REGS_DIGCTL_BASE, HW_DIGCTL_MICROSECONDS);
    pin_gpio_set(PIN_WDT, 1);
    /* initialize hardware interrupt */
    rt_hw_interrupt_init();
    init[3] = REG_RD(REGS_DIGCTL_BASE, HW_DIGCTL_MICROSECONDS);

    /* initialize uart */
    rt_hw_usart_init();
#ifdef RT_USING_CONSOLE
    rt_console_set_device(CONSOLE_DEVICE);
#endif
    init[4] = REG_RD(REGS_DIGCTL_BASE, HW_DIGCTL_MICROSECONDS);
    pin_gpio_set(PIN_WDT, 0);

    /* initialize timer0 */
    rt_hw_timer_init();
    init[5] = REG_RD(REGS_DIGCTL_BASE, HW_DIGCTL_MICROSECONDS);

#ifdef RT_USING_MTD_NAND
    /* initialize gpmi */
    rt_hw_mtd_nand_init();
#endif

#ifdef RT_USING_SDIO
    /* initialize ssp */
    rt_hw_ssp_init();
#endif

#ifdef RT_USING_USB_HOSTU
    rt_hw_usbh_init();
#endif

    pin_gpio_set(PIN_WDT, 1);
    rt_kprintf("loops %d, init %d, all %d\n",loops_per_jiffy,init[2]-init[1],init[5]-init[0]);
}

#if defined(FINSH_USING_MSH)
#include <finsh.h>
#include <msh.h>
int cmd_reboot(int argc, char** argv)
{
	int i;
    SysLog_Main(rtt_LogEmerg, "Cmd Reboot!\n");
    g_nSaveSysLog = 1;
    rt_sem_release(&syslog_sem);
    g_nSaveSysLog = 1;
    rt_sem_release(&syslog_sem);
    rt_sem_release(&syslog_sem);
    rt_sem_release(&syslog_sem);
    rt_sem_release(&syslog_sem);
    for (i=0; i<100 && g_nSaveSysLog; i++)
        rt_thread_delay(100);

	//使用内部的看门狗来达到重启的目的
	writel(1, REGS_RTC_BASE + HW_RTC_WATCHDOG);
	writel(0x80000000, REGS_RTC_BASE + HW_RTC_PERSISTENT1_SET);
	writel(BM_RTC_CTRL_WATCHDOGEN, REGS_RTC_BASE + HW_RTC_CTRL_SET);
    while(1);
	return 0;
}
RTM_EXPORT(__delay);
RTM_EXPORT(__udelay);
RTM_EXPORT(__const_udelay);
RTM_EXPORT(cmd_reboot);

int cmd_beep(int argc, char** argv)
{
#if !DIS_BEEP
	int beep = 100,i;
	if (argc > 1)
		beep = atol(argv[1]);
    for (i=0;i<beep;i++)
    {
    	pin_gpio_set(PIN_BEEP, 1);
    	udelay(250);
    	pin_gpio_set(PIN_BEEP, 0);
    	if (((i%100) == 0) && (i != 0))
			rt_thread_delay(50);
		else
    		udelay(250);
    }
#endif
	return 0;
}
RTM_EXPORT(cmd_beep);

FINSH_FUNCTION_EXPORT_ALIAS(cmd_reboot, __cmd_reboot, Reboot With WDT.)
FINSH_FUNCTION_EXPORT_ALIAS(cmd_beep, __cmd_beep, Beep.)
#endif //FINSH_USING_MSH
/*@}*/
