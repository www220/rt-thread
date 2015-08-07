/*
 * File      : timer0.c
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
 * 2015-04-29     ArdaFu       first version
 */

#include <rthw.h>
#include "board.h"
 
/*
 * TIMROT gets 4 timer instances
 * Define N as 0..3 to specify
 */
#define N		0

/* Ticks per second */
#define CONFIG_SYS_HZ	1000

/* Maximum fixed count */
#define TIMER_LOAD_VAL	1000

void rt_hw_timer0_init(void)
{
	/*
	 * Reset Timers and Rotary Encoder module
	 */

	/* Clear SFTRST */
	REG_CLR(REGS_TIMROT_BASE, HW_TIMROT_ROTCTRL, 1 << 31);
	while (REG_RD(REGS_TIMROT_BASE, HW_TIMROT_ROTCTRL) & (1 << 31))
		;

	/* Clear CLKGATE */
	REG_CLR(REGS_TIMROT_BASE, HW_TIMROT_ROTCTRL, 1 << 30);

	/* Set SFTRST and wait until CLKGATE is set */
	REG_SET(REGS_TIMROT_BASE, HW_TIMROT_ROTCTRL, 1 << 31);
	while (!(REG_RD(REGS_TIMROT_BASE, HW_TIMROT_ROTCTRL) & (1 << 30)))
		;

	/* Clear SFTRST and CLKGATE */
	REG_CLR(REGS_TIMROT_BASE, HW_TIMROT_ROTCTRL, 1 << 31);
	REG_CLR(REGS_TIMROT_BASE, HW_TIMROT_ROTCTRL, 1 << 30);

	/*
	* Now initialize timer
	*/

	/* Set fixed_count to 0 */
	REG_WR(REGS_TIMROT_BASE, HW_TIMROT_FIXED_COUNTn(N), 0);

	/* set UPDATE bit and 1Khz frequency */
	REG_WR(REGS_TIMROT_BASE, HW_TIMROT_TIMCTRLn(N),
		BM_TIMROT_TIMCTRLn_RELOAD | BM_TIMROT_TIMCTRLn_UPDATE |
		BV_TIMROT_TIMCTRLn_SELECT__1KHZ_XTAL);

	/* Set fixed_count to maximal value */
	REG_WR(REGS_TIMROT_BASE, HW_TIMROT_FIXED_COUNTn(N), TIMER_LOAD_VAL);
    {
        unsigned int *pp1 = (unsigned int *)0,*pp2 = (unsigned int *)0x41008000,i;
        for (i=0; i<16; i++)
            pp1[i] = pp2[i];
    }
	REG_WR(REGS_TIMROT_BASE, HW_TIMROT_TIMCTRLn(N), REG_RD(REGS_TIMROT_BASE, HW_TIMROT_TIMCTRLn(N)) | BM_TIMROT_TIMCTRLn_IRQ_EN);
    {
        while (0)
        {
            unsigned int aa = REG_RD(REGS_TIMROT_BASE, HW_TIMROT_RUNNING_COUNTn(N));
            if (aa <= 1)
                rt_kprintf("elsep ok %d\n",aa);
        }
    }
}
