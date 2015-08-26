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
	{ 0x40000000, 0x44000000, 0x40000000, RW_CB },       /* 64M cached SDRAM memory */
	{ 0x00000000, 0x00001000, 0x40000000, RW_CB },       /* isr vector table */
};

/* Gpmi pins */
static struct pin_desc gpmi_pins_desc[] = {
	{ PINID_GPMI_D00, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_D01, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_D02, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_D03, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_D04, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_D05, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_D06, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_D07, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_RDN, PIN_FUN1, PAD_8MA, PAD_1V8, 1 },
	{ PINID_GPMI_WRN, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_ALE, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_CLE, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_RDY0, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_RDY1, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_CE0N, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_CE1N, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_GPMI_RESETN, PIN_FUN1, PAD_4MA, PAD_3V3, 0 }
};

static struct pin_group gpmi_pins = {
	.pins		= gpmi_pins_desc,
	.nr_pins	= ARRAY_SIZE(gpmi_pins_desc)
};

void rt_hw_board_init()
{
    /* initialize mmu */
    rt_hw_mmu_init(hw_mem_desc, sizeof(hw_mem_desc)/sizeof(hw_mem_desc[0]));
    /* init udelay */
    calibrate_delay();
    /* initialize hardware interrupt */
    rt_hw_interrupt_init();
    
    /* initialize uart */
    rt_hw_usart_init();
#ifdef RT_USING_CONSOLE
    rt_console_set_device(CONSOLE_DEVICE);
#endif

    /* initialize timer0 */
    rt_hw_timer_init();
    
    /* Set up GPMI pins */
	pin_set_group(&gpmi_pins);
}

/*@}*/
