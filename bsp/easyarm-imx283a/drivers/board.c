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
	{ 0x00000000, 0xFFFFFFFF, 0x00000000, RW_NCNB },     /* None cached for 4G memory */
	{ 0x40000000, HEAP_END-1, 0x40000000, RW_CB },       /* 63M cached SDRAM memory */
	{ HEAP_END,   0x43FFFFFF, 0x43F00000, RW_NCNB },     /* 1M none-cached SDRAM */
	{ 0x00000000, 0x00020000, 0x00000000, RW_NCNB },     /* isr vector table */
};
/* None-cached RAM DMA */
unsigned char * dma_align_mem = (unsigned char *)0x00000100;

static struct pin_desc gpmi_pins_desc[] = {
	{ PINID_GPMI_D00, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_D01, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_D02, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_D03, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_D04, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_D05, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_D06, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_D07, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_RDN, PIN_FUN1, PAD_8MA, PAD_3V3, 1 },
	{ PINID_GPMI_WRN, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_ALE, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_CLE, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_RDY0, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_CE0N, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_RESETN, PIN_FUN1, PAD_4MA, PAD_3V3, 0 }
};
static struct pin_desc led_pins_desc[] = {
	{ PINID_GPMI_RDY1, PIN_GPIO, PAD_8MA, PAD_3V3, 1 },
	{ PINID_LCD_D16, PIN_GPIO, PAD_8MA, PAD_3V3, 1 },
	{ PINID_LCD_D21, PIN_GPIO, PAD_8MA, PAD_3V3, 1 },
	{ PINID_LCD_D22, PIN_GPIO, PAD_8MA, PAD_3V3, 1 },
	{ PINID_LCD_D23, PIN_GPIO, PAD_8MA, PAD_3V3, 1 }
};

static struct pin_group gpmi_pins = {
	.pins		= gpmi_pins_desc,
	.nr_pins	= ARRAY_SIZE(gpmi_pins_desc)
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
    
	pin_gpio_direction(PINID_GPMI_RDY1, 1);
	pin_gpio_direction(PINID_LCD_D21, 1);
	pin_gpio_direction(PINID_LCD_D22, 1);
	pin_gpio_direction(PINID_LCD_D23, 1);
    pin_gpio_set(PINID_GPMI_RDY1, 0);

    /* initialize mmu */
    rt_hw_mmu_init(hw_mem_desc, sizeof(hw_mem_desc)/sizeof(hw_mem_desc[0]));
    init[1] = REG_RD(REGS_DIGCTL_BASE, HW_DIGCTL_MICROSECONDS);
    /* init udelay */
    calibrate_delay();
    init[2] = REG_RD(REGS_DIGCTL_BASE, HW_DIGCTL_MICROSECONDS);
    pin_gpio_set(PINID_GPMI_RDY1, 1);
    /* initialize hardware interrupt */
    rt_hw_interrupt_init();
    init[3] = REG_RD(REGS_DIGCTL_BASE, HW_DIGCTL_MICROSECONDS);

    /* initialize uart */
    rt_hw_usart_init();
#ifdef RT_USING_CONSOLE
    rt_console_set_device(CONSOLE_DEVICE);
#endif
    init[4] = REG_RD(REGS_DIGCTL_BASE, HW_DIGCTL_MICROSECONDS);

    /* initialize timer0 */
    rt_hw_timer_init();
    init[5] = REG_RD(REGS_DIGCTL_BASE, HW_DIGCTL_MICROSECONDS);

#ifdef RT_USING_MTD_NAND
    /* Set up GPMI pins */
	pin_set_group(&gpmi_pins);
    rt_hw_mtd_nand_init();
#endif

    pin_gpio_set(PINID_GPMI_RDY1, 0);
    rt_kprintf("loops %d, init %d, all %d\n",loops_per_jiffy,init[2]-init[1],init[5]-init[0]);
}

/*@}*/
