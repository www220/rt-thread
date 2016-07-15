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

/* Maximum fixed count */
#define TIMER_LOAD_VAL	32000/RT_TICK_PER_SECOND

void rt_hw_systick_handler(int vector, void *param)
{
    register rt_uint32_t ir = REG_RD(REGS_TIMROT_BASE, HW_TIMROT_TIMCTRLn(N));
    if (ir & BM_TIMROT_TIMCTRLn_IRQ)
    {
        rt_tick_increase();
        REG_WR(REGS_TIMROT_BASE, HW_TIMROT_TIMCTRLn_CLR(N), BM_TIMROT_TIMCTRLn_IRQ);
    }
}

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

	/* Set fixed_count to 0xffffffff */
	REG_WR(REGS_TIMROT_BASE, HW_TIMROT_FIXED_COUNTn(N), 0xffffffff);

	/* set UPDATE bit and 1Khz frequency */
	REG_WR(REGS_TIMROT_BASE, HW_TIMROT_TIMCTRLn(N),
		BM_TIMROT_TIMCTRLn_RELOAD | BM_TIMROT_TIMCTRLn_UPDATE |
		BV_TIMROT_TIMCTRLn_SELECT__32KHZ_XTAL | BM_TIMROT_TIMCTRLn_IRQ_EN);

	/* Set fixed_count to maximal value */
	REG_WR(REGS_TIMROT_BASE, HW_TIMROT_FIXED_COUNTn(N), TIMER_LOAD_VAL);
}

void rt_hw_timer_init()
{
    rt_hw_timer0_init();
    /* install interrupt handler */
    rt_hw_interrupt_install(IRQ_TIMER0, rt_hw_systick_handler, RT_NULL, "SysTick");
    rt_hw_interrupt_umask(IRQ_TIMER0);
}

static u32 mx28_get_pclk(void)
{
	const u32 xtal = 24, ref = 480;
	u32 clkfrac, clkseq, clkctrl;
	u32 frac, div;
	u32 pclk;

	clkfrac = REG_RD(REGS_CLKCTRL_BASE, HW_CLKCTRL_FRAC0);
	clkseq = REG_RD(REGS_CLKCTRL_BASE, HW_CLKCTRL_CLKSEQ);
	clkctrl = REG_RD(REGS_CLKCTRL_BASE, HW_CLKCTRL_CPU);

	if (clkctrl & (BM_CLKCTRL_CPU_DIV_XTAL_FRAC_EN |
		BM_CLKCTRL_CPU_DIV_CPU_FRAC_EN)) {
		/* No support of fractional divider calculation */
		pclk = 0;
	} else {
		if (clkseq & BM_CLKCTRL_CLKSEQ_BYPASS_CPU) {
			/* xtal path */
			div = (clkctrl & BM_CLKCTRL_CPU_DIV_XTAL) >>
				BP_CLKCTRL_CPU_DIV_XTAL;
			pclk = xtal / div;
		} else {
			/* ref path */
			frac = (clkfrac & BM_CLKCTRL_FRAC0_CPUFRAC) >>
				BP_CLKCTRL_FRAC0_CPUFRAC;
			div = (clkctrl & BM_CLKCTRL_CPU_DIV_CPU) >>
				BP_CLKCTRL_CPU_DIV_CPU;
			pclk =  (ref * 18 / frac) / div;
		}
	}

	return pclk;
}
u32 mx28_get_hclk(void)
{
	u32 clkctrl, div, hclk;

	clkctrl = REG_RD(REGS_CLKCTRL_BASE, HW_CLKCTRL_HBUS);

	if (clkctrl & BM_CLKCTRL_HBUS_DIV_FRAC_EN) {
		/* No support of fractional divider calculation */
		hclk = 0;
	} else {
		div = (clkctrl & BM_CLKCTRL_HBUS_DIV) >>
			BP_CLKCTRL_HBUS_DIV;
		hclk = mx28_get_pclk() / div;
	}

	return hclk;
}
static u32 mx28_get_emiclk(void)
{
	const u32 xtal = 24, ref = 480;
	u32 clkfrac, clkseq, clkctrl;
	u32 frac, div;
	u32 emiclk;

	clkfrac = REG_RD(REGS_CLKCTRL_BASE, HW_CLKCTRL_FRAC0);
	clkseq = REG_RD(REGS_CLKCTRL_BASE, HW_CLKCTRL_CLKSEQ);
	clkctrl = REG_RD(REGS_CLKCTRL_BASE, HW_CLKCTRL_EMI);

	if (clkseq & BM_CLKCTRL_CLKSEQ_BYPASS_EMI) {
		/* xtal path */
		div = (clkctrl & BM_CLKCTRL_EMI_DIV_XTAL) >>
			BP_CLKCTRL_EMI_DIV_XTAL;
		emiclk = xtal / div;
	} else {
		/* ref path */
		frac = (clkfrac & BM_CLKCTRL_FRAC0_EMIFRAC) >>
			BP_CLKCTRL_FRAC0_EMIFRAC;
		div = (clkctrl & BM_CLKCTRL_EMI_DIV_EMI) >>
			BP_CLKCTRL_EMI_DIV_EMI;
		emiclk =  (ref * 18 / frac) / div;
	}

	return emiclk;
}
static u32 mx28_get_gpmiclk(void)
{
	const u32 xtal = 24, ref = 480;
	u32 clkfrac, clkseq, clkctrl;
	u32 frac, div;
	u32 gpmiclk;

	clkfrac = REG_RD(REGS_CLKCTRL_BASE, HW_CLKCTRL_FRAC1);
	clkseq = REG_RD(REGS_CLKCTRL_BASE, HW_CLKCTRL_CLKSEQ);
	clkctrl = REG_RD(REGS_CLKCTRL_BASE, HW_CLKCTRL_GPMI);

	if (clkseq & BM_CLKCTRL_CLKSEQ_BYPASS_GPMI) {
		/* xtal path */
		div = (clkctrl & BM_CLKCTRL_GPMI_DIV) >>
			BP_CLKCTRL_GPMI_DIV;
		gpmiclk = xtal / div;
	} else {
		/* ref path */
		frac = (clkfrac & BM_CLKCTRL_FRAC1_GPMIFRAC) >>
			BP_CLKCTRL_FRAC1_GPMIFRAC;
		div = (clkctrl & BM_CLKCTRL_GPMI_DIV) >>
			BP_CLKCTRL_GPMI_DIV;
		gpmiclk =  (ref * 18 / frac) / div;
	}

	return gpmiclk;
}

#ifdef RT_USING_FINSH
void list_cpuinfo(void)
{
	rt_kprintf("Freescale i.MX28 family\n");
	rt_kprintf("CPU:   %d MHz\n", mx28_get_pclk());
	rt_kprintf("BUS:   %d MHz\n", mx28_get_hclk());
	rt_kprintf("EMI:   %d MHz\n", mx28_get_emiclk());
	rt_kprintf("GPMI:   %d MHz\n", mx28_get_gpmiclk());
}

#include <finsh.h>
FINSH_FUNCTION_EXPORT(list_cpuinfo, list cpu info);
MSH_CMD_EXPORT(list_cpuinfo, list cpu info);
FINSH_VAR_EXPORT(dma_align_mem, finsh_type_int, dma ram pos for finsh)
FINSH_VAR_EXPORT(dma_align_max, finsh_type_int, dma_max pos for finsh)
#endif
