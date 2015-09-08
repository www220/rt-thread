/*
 * Freescale MXS LCDIF low-level routines
 *
 * Author: Vitaly Wool <vital@embeddedalley.com>
 *
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "lcdif.h"

void mxs_init_lcdif(void)
{
	writel(BM_LCDIF_CTRL_CLKGATE,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	/* Reset controller */
	writel(BM_LCDIF_CTRL_SFTRST, REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
	udelay(10);

	/* Take controller out of reset */
	writel(BM_LCDIF_CTRL_SFTRST | BM_LCDIF_CTRL_CLKGATE,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);

	/* Setup the bus protocol */
	writel(BM_LCDIF_CTRL1_MODE86,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);
	writel(BM_LCDIF_CTRL1_BUSY_ENABLE,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);

	/* Take display out of reset */
	writel(BM_LCDIF_CTRL1_RESET,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET);

	/* VSYNC is an input by default */
	writel(BM_LCDIF_VDCTRL0_VSYNC_OEB,
		     REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0_SET);

	/* Reset display */
	writel(BM_LCDIF_CTRL1_RESET,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);
	udelay(10);
	writel(BM_LCDIF_CTRL1_RESET,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET);
	udelay(10);
}

int mxs_lcdif_dma_init(struct rt_device *dev, dma_addr_t phys, int memsize)
{
	writel(BM_LCDIF_CTRL_LCDIF_MASTER,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);

	writel(phys, REGS_LCDIF_BASE + HW_LCDIF_CUR_BUF);
	writel(phys, REGS_LCDIF_BASE + HW_LCDIF_NEXT_BUF);

	return 0;
}

void mxs_lcdif_dma_release(void)
{
	writel(BM_LCDIF_CTRL_LCDIF_MASTER,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	return;
}

void mxs_lcdif_run(void)
{
	writel(BM_LCDIF_CTRL_LCDIF_MASTER,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
	writel(BM_LCDIF_CTRL_RUN, REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
}

void mxs_lcdif_stop(void)
{
	writel(BM_LCDIF_CTRL_RUN, REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	writel(BM_LCDIF_CTRL_LCDIF_MASTER,
		     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	udelay(100);
}

int mxs_lcdif_pan_display(dma_addr_t addr)
{
	writel(addr, REGS_LCDIF_BASE + HW_LCDIF_NEXT_BUF);

	return 0;
}

