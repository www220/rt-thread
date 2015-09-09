/*
 * Freescale STMP37XX/STMP378X LRADC helper routines
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <common.h>
#include <regs-lradc.h>
#include <lradc.h>
#include "board.h"
#include <errno.h>

static int channels[8];
int hw_lradc_use_channel(int channel)
{
	if (channel < 0 || channel > 7)
		return -EINVAL;
	channels[channel]++;
	return 0;
}

int hw_lradc_unuse_channel(int channel)
{
	if (channel < 0 || channel > 7)
		return -EINVAL;
	channels[channel]--;
	return 0;
}

void hw_lradc_reinit(int enable_ground_ref, unsigned freq)
{
	writel(BM_LRADC_CTRL0_SFTRST,
		     REGS_LRADC_BASE + HW_LRADC_CTRL0_SET);
	udelay(1);
	writel(BM_LRADC_CTRL0_SFTRST,
		     REGS_LRADC_BASE + HW_LRADC_CTRL0_CLR);

	/* Clear the Clock Gate for normal operation */
	writel(BM_LRADC_CTRL0_CLKGATE,
		     REGS_LRADC_BASE + HW_LRADC_CTRL0_CLR);

	if (enable_ground_ref)
		writel(BM_LRADC_CTRL0_ONCHIP_GROUNDREF,
			     REGS_LRADC_BASE + HW_LRADC_CTRL0_SET);
	else
		writel(BM_LRADC_CTRL0_ONCHIP_GROUNDREF,
			    REGS_LRADC_BASE + HW_LRADC_CTRL0_CLR);

	writel(BM_LRADC_CTRL3_CYCLE_TIME,
		     REGS_LRADC_BASE + HW_LRADC_CTRL3_CLR);
	writel(BF_LRADC_CTRL3_CYCLE_TIME(freq),
		     REGS_LRADC_BASE + HW_LRADC_CTRL3_SET);

	writel(BM_LRADC_CTRL4_LRADC6SELECT | BM_LRADC_CTRL4_LRADC7SELECT,
		     REGS_LRADC_BASE + HW_LRADC_CTRL4_CLR);
	writel(BF_LRADC_CTRL4_LRADC6SELECT(BV_LRADC_CTRL4_LRADC6SELECT__CHANNEL10),
		     REGS_LRADC_BASE + HW_LRADC_CTRL4_SET);
	writel(BF_LRADC_CTRL4_LRADC7SELECT(BV_LRADC_CTRL4_LRADC7SELECT__CHANNEL7),
		     REGS_LRADC_BASE + HW_LRADC_CTRL4_SET);
}

int hw_lradc_init_ladder(int channel, int trigger, unsigned sampling)
{
	/*
	 * check if the lradc channel is present in this product
	 */
	if (!hw_lradc_present(channel))
		return -ENODEV;

	hw_lradc_configure_channel(channel, !0 /* div2 */ ,
				   0 /* acc */ ,
				   0 /* num_samples */);

	/* Setup the trigger loop forever */
	hw_lradc_set_delay_trigger(trigger, 1 << channel,
				   1 << trigger, 0, sampling);

	/* Clear the accumulator & NUM_SAMPLES */
	writel(0xFFFFFFFF, REGS_LRADC_BASE + HW_LRADC_CHn_CLR(channel));
	return 0;
}

int hw_lradc_stop_ladder(int channel, int trigger)
{
	/*
	 * check if the lradc channel is present in this product
	 */
	if (!hw_lradc_present(channel))
		return -ENODEV;
	hw_lradc_clear_delay_trigger(trigger, 1 << channel, 1 << trigger);
	return 0;
}

int hw_lradc_present(int channel)
{
	if (channel < 0 || channel > 7)
		return 0;
	return readl(REGS_LRADC_BASE + HW_LRADC_STATUS)
	    & (1 << (16 + channel));
}

void hw_lradc_configure_channel(int channel, int enable_div2,
				int enable_acc, int samples)
{
	if (enable_div2)
		writel(BF_LRADC_CTRL2_DIVIDE_BY_TWO(1 << channel),
			     REGS_LRADC_BASE + HW_LRADC_CTRL2_SET);
	else
		writel(BF_LRADC_CTRL2_DIVIDE_BY_TWO(1 << channel),
			     REGS_LRADC_BASE + HW_LRADC_CTRL2_CLR);

	/* Clear the accumulator & NUM_SAMPLES */
	writel(0xFFFFFFFF, REGS_LRADC_BASE + HW_LRADC_CHn_CLR(channel));

	/* Sets NUM_SAMPLES bitfield of HW_LRADC_CHn register. */
	writel(BM_LRADC_CHn_NUM_SAMPLES,
		     REGS_LRADC_BASE + HW_LRADC_CHn_CLR(channel));
	writel(BF_LRADC_CHn_NUM_SAMPLES(samples),
		     REGS_LRADC_BASE + HW_LRADC_CHn_SET(channel));

	if (enable_acc)
		writel(BM_LRADC_CHn_ACCUMULATE,
			     REGS_LRADC_BASE + HW_LRADC_CHn_SET(channel));
	else
		writel(BM_LRADC_CHn_ACCUMULATE,
			     REGS_LRADC_BASE + HW_LRADC_CHn_CLR(channel));
}

void hw_lradc_set_delay_trigger(int trigger, u32 trigger_lradc,
				u32 delay_triggers, u32 loops, u32 delays)
{
	/* set TRIGGER_LRADCS in HW_LRADC_DELAYn */
	writel(BF_LRADC_DELAYn_TRIGGER_LRADCS(trigger_lradc),
		     REGS_LRADC_BASE + HW_LRADC_DELAYn_SET(trigger));
	writel(BF_LRADC_DELAYn_TRIGGER_DELAYS(delay_triggers),
		     REGS_LRADC_BASE + HW_LRADC_DELAYn_SET(trigger));

	writel(BM_LRADC_DELAYn_LOOP_COUNT | BM_LRADC_DELAYn_DELAY,
		     REGS_LRADC_BASE + HW_LRADC_DELAYn_CLR(trigger));
	writel(BF_LRADC_DELAYn_LOOP_COUNT(loops),
		     REGS_LRADC_BASE  + HW_LRADC_DELAYn_SET(trigger));
	writel(BF_LRADC_DELAYn_DELAY(delays),
		     REGS_LRADC_BASE + HW_LRADC_DELAYn_SET(trigger));
}

void hw_lradc_clear_delay_trigger(int trigger, u32 trigger_lradc,
				  u32 delay_triggers)
{
	writel(BF_LRADC_DELAYn_TRIGGER_LRADCS(trigger_lradc),
		     REGS_LRADC_BASE + HW_LRADC_DELAYn_CLR(trigger));
	writel(BF_LRADC_DELAYn_TRIGGER_DELAYS(delay_triggers),
		     REGS_LRADC_BASE + HW_LRADC_DELAYn_CLR(trigger));
}

void hw_lradc_set_delay_trigger_kick(int trigger, int value)
{
	if (value)
		writel(BM_LRADC_DELAYn_KICK,
			     REGS_LRADC_BASE + HW_LRADC_DELAYn_SET(trigger));
	else
		writel(BM_LRADC_DELAYn_KICK,
			     REGS_LRADC_BASE + HW_LRADC_DELAYn_CLR(trigger));
}
