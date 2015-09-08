#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include "board.h"
#include "lcdif.h"

#define TM043NDH02   0
#define HW480272F    1
#define TM070RDH13   2
#define LCM_TYPE   HW480272F

#if ((LCM_TYPE) == (TM043NDH02))
#define LCM_NAME  "TM043NDH02"
#define DOTCLK_FREQUENCY_HZ   9000000
#define DOTCLK_H_ACTIVE 480 
#define DOTCLK_H_PULSE_WIDTH  41
#define DOTCLK_HF_PORCH  5
#define DOTCLK_HB_PORCH  5
#define DOTCLK_V_ACTIVE  272
#define DOTCLK_V_PULSE_WIDTH 20
#define DOTCLK_VF_PORCH  5
#define DOTCLK_VB_PORCH  5

#elif ((LCM_TYPE) == (HW480272F))
#define LCM_NAME  "HW480272F"
#define DOTCLK_FREQUENCY_HZ   9000000
#define DOTCLK_H_ACTIVE 480 
#define DOTCLK_H_PULSE_WIDTH  41
#define DOTCLK_HF_PORCH  5
#define DOTCLK_HB_PORCH  5
#define DOTCLK_V_ACTIVE  272
#define DOTCLK_V_PULSE_WIDTH 20
#define DOTCLK_VF_PORCH  5
#define DOTCLK_VB_PORCH  5

#elif ((LCM_TYPE) == (TM070RDH13))
#define LCM_NAME  "TM070RDH13"
#define DOTCLK_FREQUENCY_HZ   40000000
#define DOTCLK_H_ACTIVE  800
#define DOTCLK_H_PULSE_WIDTH 6
#define DOTCLK_HF_PORCH  354
#define DOTCLK_HB_PORCH  40
#define DOTCLK_V_ACTIVE  480
#define DOTCLK_V_PULSE_WIDTH  3
#define DOTCLK_VF_PORCH  147
#define DOTCLK_VB_PORCH  20

#else
#define LCM_NAME  "Default_480x272"
#define DOTCLK_FREQUENCY_HZ   9000000
#define DOTCLK_H_ACTIVE 480 
#define DOTCLK_H_PULSE_WIDTH  41
#define DOTCLK_HF_PORCH  5
#define DOTCLK_HB_PORCH  5
#define DOTCLK_V_ACTIVE  272
#define DOTCLK_V_PULSE_WIDTH 20
#define DOTCLK_VF_PORCH  5
#define DOTCLK_VB_PORCH  5
#endif

#define DOTCLK_H_WAIT_CNT  (DOTCLK_H_PULSE_WIDTH + DOTCLK_HB_PORCH)
#define DOTCLK_H_PERIOD (DOTCLK_H_WAIT_CNT + DOTCLK_HF_PORCH + DOTCLK_H_ACTIVE)
#define DOTCLK_V_WAIT_CNT (DOTCLK_V_PULSE_WIDTH + DOTCLK_VB_PORCH)
#define DOTCLK_V_PERIOD (DOTCLK_VF_PORCH + DOTCLK_V_ACTIVE + DOTCLK_V_WAIT_CNT)

static struct mxs_platform_bl_data bl_data;

static void init_timings()
{
	unsigned phase_time;
	unsigned timings;

	/* Just use a phase_time of 1. As optimal as it gets, now. */
	phase_time = 1;

	/* Program all 4 timings the same */
	timings = phase_time;
	timings |= timings << 8;
	timings |= timings << 16;
	writel(timings, REGS_LCDIF_BASE + HW_LCDIF_TIMING);
}

#define CLKCTRL_BASE_ADDR REGS_CLKCTRL_BASE
#define DIGCTRL_BASE_ADDR REGS_DIGCTL_BASE

static unsigned long pll_get_rate(int clk)
{
	unsigned int reg;
	if (clk == 2)
		return 50000000;
	if (clk == 0) {
		reg = readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL0CTRL1);
		reg = (reg & BM_CLKCTRL_PLL0CTRL0_DIV_SEL) >> BP_CLKCTRL_PLL0CTRL0_DIV_SEL;
	} else {
		reg = readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_PLL1CTRL1);
		reg = (reg & BM_CLKCTRL_PLL1CTRL0_DIV_SEL) >> BP_CLKCTRL_PLL1CTRL0_DIV_SEL;
	}
	switch (reg) {
	case 0:
		return 480000000;
	case 1:
		return 384000000;
	case 2:
		return 288000000;
	default:
		return -EINVAL;
	}
}

static int lcdif_set_rate(unsigned long rate)
{
	int ret = 0;

	/*
	 * ref_pix can be between 480e6*18/35=246.9MHz and 480e6*18/18=480MHz,
	 * which is between 18/(18*480e6)=2.084ns and 35/(18*480e6)=4.050ns.
	 *
	 * ns_cycle >= 2*18e3/(18*480) = 25/6
	 * ns_cycle <= 2*35e3/(18*480) = 875/108
	 *
	 * Multiply the ns_cycle by 'div' to lengthen it until it fits the
	 * bounds. This is the divider we'll use after ref_pix.
	 *
	 * 6 * ns_cycle >= 25 * div
	 * 108 * ns_cycle <= 875 * div
	 */
	u32 ns_cycle = 1000000000 / rate;
	u32 div, reg_val;
	u32 lowest_result = (u32) -1;
	u32 lowest_div = 0, lowest_fracdiv = 0;

	ns_cycle *= 2;

	for (div = 1; div < 256; ++div) {
		u32 fracdiv;
		u32 ps_result;
		int lower_bound = 6 * ns_cycle >= 25 * div;
		int upper_bound = 108 * ns_cycle <= 875 * div;
		if (!lower_bound)
			break;
		if (!upper_bound)
			continue;
		/*
		 * Found a matching div. Calculate fractional divider needed,
		 * rounded up.
		 */
		fracdiv = ((pll_get_rate(0) / 1000000 * 18 / 2) *
		           ns_cycle + 1000 * div - 1) /
		          (1000 * div);
		if (fracdiv < 18 || fracdiv > 35) {
			ret = -EINVAL;
			goto out;
		}
		/* Calculate the actual cycle time this results in */
		ps_result = 6250 * div * fracdiv / 27;

		/* Use the fastest result that doesn't break ns_cycle */
		if (ps_result <= lowest_result) {
			lowest_result = ps_result;
			lowest_div = div;
			lowest_fracdiv = fracdiv;
		}
	}

	if (div >= 256 || lowest_result == (u32) -1) {
		ret = -EINVAL;
		goto out;
	}
	/* set to XTAL source first */
	reg_val = readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ);
	reg_val |= BM_CLKCTRL_CLKSEQ_BYPASS_DIS_LCDIF;
	writel(reg_val, CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ);


	/* Program ref_pix phase fractional divider */
	reg_val = readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC1);
	reg_val &= ~BM_CLKCTRL_FRAC1_PIXFRAC;
	reg_val |= BF_CLKCTRL_FRAC1_PIXFRAC(lowest_fracdiv);
	writel(reg_val, CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC1);

	/* Ungate PIX CLK*/
	writel(BM_CLKCTRL_FRAC1_CLKGATEPIX,
	             CLKCTRL_BASE_ADDR + HW_CLKCTRL_FRAC1_CLR);

	/* Program LCDIF divider */
	reg_val = readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_DIS_LCDIF);
	reg_val &= ~(BM_CLKCTRL_DIS_LCDIF_DIV | BM_CLKCTRL_DIS_LCDIF_CLKGATE);
	reg_val |= BF_CLKCTRL_DIS_LCDIF_DIV(lowest_div);
	writel(reg_val, CLKCTRL_BASE_ADDR + HW_CLKCTRL_DIS_LCDIF);

	{
		int i;
		for (i = 100000; i; i--)
			if ((readl(CLKCTRL_BASE_ADDR + HW_CLKCTRL_DIS_LCDIF) & (1 << 29)) == 0)
				break;
		if (!i)
			return -ETIMEDOUT;
	}
	/* Switch to ref_pix source */
	reg_val = BM_CLKCTRL_CLKSEQ_BYPASS_DIS_LCDIF;
	writel(reg_val, CLKCTRL_BASE_ADDR + HW_CLKCTRL_CLKSEQ_CLR);

out:
	rt_kprintf("Programming PFD=%u,DIV=%u ref_pix=%uMHz "
		 "PIXCLK=%uMHz cycle=%u.%03uns\n",
		 lowest_fracdiv, lowest_div,
		 480*18/lowest_fracdiv, 480*18/lowest_fracdiv/lowest_div,
		 lowest_result / 1000, lowest_result % 1000);
	return ret;
}

static int init_panel(struct rt_device *dev, dma_addr_t phys, int memsize,
		      struct mxs_platform_fb_entry *pentry)
{
	int ret = 0;

    mxs_clock_enable(CLKCTRL_BASE_ADDR + HW_CLKCTRL_DIS_LCDIF, BM_CLKCTRL_DIS_LCDIF_CLKGATE);
	ret = lcdif_set_rate(pentry->dclk_f);	/* Hz */
    if (ret) {                
		mxs_clock_disable(CLKCTRL_BASE_ADDR + HW_CLKCTRL_DIS_LCDIF, BM_CLKCTRL_DIS_LCDIF_CLKGATE);            
		goto out;        
	}

	/*
	 * Make sure we do a high-to-low transition to reset the panel.
	 * First make it low for 100 msec, hi for 10 msec, low for 10 msec,
	 * then hi.
	 */
	writel(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);	/* low */
	rt_thread_delay(100);
	writel(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET);	/* high */
	rt_thread_delay(10);
	writel(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);	/* low */

	/* For the Samsung, Reset must be held low at least 30 uSec
	 * Therefore, we'll hold it low for about 10 mSec just to be sure.
	 * Then we'll wait 1 mSec afterwards.
	 */
	rt_thread_delay(10);
	writel(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET);	/* high */
	rt_thread_delay(1);
	
	setup_dotclk_panel(DOTCLK_V_PULSE_WIDTH, DOTCLK_V_PERIOD,
			   DOTCLK_V_WAIT_CNT, DOTCLK_V_ACTIVE,
			   DOTCLK_H_PULSE_WIDTH, DOTCLK_H_PERIOD,
			   DOTCLK_H_WAIT_CNT, DOTCLK_H_ACTIVE, 0);

	ret = mxs_lcdif_dma_init(dev, phys, memsize);
	if (ret)
		goto out;

	return 0;

out:
	return ret;
}

static void release_panel(struct rt_device *dev,
			  struct mxs_platform_fb_entry *pentry)
{
	release_dotclk_panel();
	mxs_lcdif_dma_release();
	mxs_clock_disable(CLKCTRL_BASE_ADDR + HW_CLKCTRL_DIS_LCDIF, BM_CLKCTRL_DIS_LCDIF_CLKGATE);            
}

static int blank_panel(int blank)
{
	int ret = 0, count;

	if (blank) {
		writel(BM_LCDIF_CTRL_BYPASS_COUNT,
			     REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
		for (count = 100000; count; count--) {
			if (readl(REGS_LCDIF_BASE + HW_LCDIF_STAT) &
			    BM_LCDIF_STAT_TXFIFO_EMPTY)
				break;
			udelay(1);
		}
		rt_thread_delay(1);
	} else {
		writel(BM_LCDIF_CTRL_BYPASS_COUNT,
			     REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
	}
	return ret;
}

static int init_bl(struct mxs_platform_bl_data *data)
{
    mxs_clock_enable(CLKCTRL_BASE_ADDR + HW_CLKCTRL_XTAL, BM_CLKCTRL_XTAL_PWM_CLK24M_GATE);
	mxs_reset_clock(REGS_PWM_BASE, 1);

	writel(BF_PWM_ACTIVEn_INACTIVE(0) |
		     BF_PWM_ACTIVEn_ACTIVE(0),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(3));
	writel(BF_PWM_PERIODn_CDIV(6) |	/* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) |	/* low */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) |	/* high */
		     BF_PWM_PERIODn_PERIOD(399),
		     REGS_PWM_BASE + HW_PWM_PERIODn(3));
	writel(BM_PWM_CTRL_PWM3_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL_SET);

	return 0;
}

static void free_bl(struct mxs_platform_bl_data *data)
{
	writel(BF_PWM_ACTIVEn_INACTIVE(0) |
		     BF_PWM_ACTIVEn_ACTIVE(0),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(3));
	writel(BF_PWM_PERIODn_CDIV(6) |	/* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) |	/* low */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) |	/* high */
		     BF_PWM_PERIODn_PERIOD(399),
		     REGS_PWM_BASE + HW_PWM_PERIODn(3));
	writel(BM_PWM_CTRL_PWM3_ENABLE, REGS_PWM_BASE + HW_PWM_CTRL_CLR);

    mxs_clock_disable(CLKCTRL_BASE_ADDR + HW_CLKCTRL_XTAL, BM_CLKCTRL_XTAL_PWM_CLK24M_GATE);
}

static int values[] = { 0, 4, 9, 14, 20, 27, 35, 45, 57, 75, 100 };

static int set_bl_intensity(struct mxs_platform_bl_data *data,
			    int intensity)
{
	int scaled_int;

    if (intensity < 0)
        intensity = 0;
	scaled_int = values[intensity / 10];
	if (scaled_int < 100) {
		int rem = intensity - 10 * (intensity / 10);	// r = i % 10; 
		scaled_int += rem * (values[intensity / 10 + 1] -
				     values[intensity / 10]) / 10;
	}
	
	writel(BF_PWM_ACTIVEn_INACTIVE(scaled_int*399/100) |
		     BF_PWM_ACTIVEn_ACTIVE(0),
		     REGS_PWM_BASE + HW_PWM_ACTIVEn(3));
	writel(BF_PWM_PERIODn_CDIV(6) |	/* divide by 64 */
		     BF_PWM_PERIODn_INACTIVE_STATE(2) |	/* high  */
		     BF_PWM_PERIODn_ACTIVE_STATE(3) |	/* low */
		     BF_PWM_PERIODn_PERIOD(399),
		     REGS_PWM_BASE + HW_PWM_PERIODn(3));

	return 0;
}

static struct mxs_platform_bl_data bl_data = {
    .bl_max_intensity = 100,
    .bl_default_intensity = 80,
    .bl_cons_intensity = 50,
    .init_bl = init_bl,
    .free_bl = free_bl,
    .set_bl_intensity = set_bl_intensity,
};

static struct mxs_platform_fb_entry fb_entry = {
    .name  = (LCM_NAME),
    .x_res = (DOTCLK_V_ACTIVE),
    .y_res = (DOTCLK_H_ACTIVE),
    .bpp   = 16,
    .dclk_f = (DOTCLK_FREQUENCY_HZ),
    .lcd_type = MXS_LCD_PANEL_DOTCLK,
    .init_panel = init_panel,
    .release_panel = release_panel,
    .blank_panel = blank_panel,
    .run_panel = mxs_lcdif_run,
    .stop_panel = mxs_lcdif_stop,
    .pan_display = mxs_lcdif_pan_display,
    .bl_data = &bl_data,
};

static struct pin_desc lcd_pins_desc[] = {
	{ PINID_LCD_D00, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_LCD_D01, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_LCD_D02, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_LCD_D03, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_LCD_D04, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_LCD_D05, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_LCD_D06, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_LCD_D07, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_LCD_D08, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_LCD_D09, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_LCD_D10, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_LCD_D11, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_LCD_D12, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_LCD_D13, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_LCD_D14, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_LCD_D15, PIN_FUN1, PAD_4MA, PAD_3V3, 0 },
	{ PINID_LCD_RESET, PIN_FUN1, PAD_8MA, PAD_3V3, 0 },
	{ PINID_LCD_RD_E, PIN_FUN2, PAD_8MA, PAD_3V3, 0 },
	{ PINID_LCD_WR_RWN, PIN_FUN2, PAD_8MA, PAD_3V3, 0 },
	{ PINID_LCD_CS, PIN_FUN2, PAD_8MA, PAD_3V3, 0 },
	{ PINID_LCD_RS, PIN_FUN2, PAD_8MA, PAD_3V3, 0 },
	{ PINID_PWM3, PIN_FUN1, PAD_8MA, PAD_3V3, 0 }
};
static struct pin_group lcd_pins = {
	.pins		= lcd_pins_desc,
	.nr_pins	= ARRAY_SIZE(lcd_pins_desc)
};

struct sdlfb_device
{
    struct rt_device parent;

    struct mxs_platform_fb_entry* entry;
    rt_uint8_t *phys;
    int memsize;
};
struct sdlfb_device _device;

static rt_err_t  sdlfb_init(rt_device_t dev)
{
    return RT_EOK;
}
static rt_err_t  sdlfb_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}
static rt_err_t  sdlfb_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t sdlfb_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    struct sdlfb_device *device;
    device = (struct sdlfb_device *)dev;
    
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->phys != RT_NULL);

    switch (cmd) 
    {
    case RTGRAPHIC_CTRL_GET_INFO: {
        struct rt_device_graphic_info *info;
        info = (struct rt_device_graphic_info *) args;
        
        info->bits_per_pixel = 16;
        info->pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
        info->framebuffer = device->phys;
        info->width = device->entry->x_res;
        info->height = device->entry->y_res;
    break; }
    
    case RTGRAPHIC_CTRL_RECT_UPDATE: {
        struct rt_device_rect_info *rect;
        rect = (struct rt_device_rect_info *)args;
        
        rt_kprintf("update %d %d %d %d\n",rect->x,rect->y,rect->width,rect->height);
        break; }
    }
    
    return RT_EOK;
}

extern rt_err_t rtgui_graphic_set_device(rt_device_t device);
int lcd_init(void)
{
    int ret = 0;
    
    /* Set up LCD pins */
	pin_set_group(&lcd_pins);

    mxs_init_lcdif();
    ret = bl_data.init_bl(&bl_data);
    if (ret != 0)
    {
		rt_kprintf("cannot initialize LCD backlight\n");
        goto out_panel;
    }
    
    _device.entry = &fb_entry;
    _device.memsize = fb_entry.y_res * fb_entry.x_res * fb_entry.bpp / 8;
    _device.phys = memalign_max(32, _device.memsize);
    ret = fb_entry.init_panel(RT_DEVICE(&_device),(dma_addr_t)_device.phys,_device.memsize,&fb_entry);
    if (ret != 0)
    {
		rt_kprintf("cannot initialize LCD panel\n");
        goto out_panel;
    }
    init_timings();
    fb_entry.run_panel();
    fb_entry.blank_panel(0);
    bl_data.set_bl_intensity(&bl_data,bl_data.bl_default_intensity);
    
    _device.parent.type = RT_Device_Class_Graphic;
    _device.parent.init = sdlfb_init;
    _device.parent.open = sdlfb_open;
    _device.parent.close = sdlfb_close;
    _device.parent.read = RT_NULL;
    _device.parent.write = RT_NULL;
    _device.parent.control = sdlfb_control;

    rt_device_register(RT_DEVICE(&_device), "sdl", RT_DEVICE_FLAG_RDWR);
    rtgui_graphic_set_device(RT_DEVICE(&_device));
    return 0;
    
out_panel:
    return ret;
}

#if defined(RT_USING_FINSH)
#include <finsh.h>
void lcd_intensity(int intensity)
{
    bl_data.set_bl_intensity(&bl_data, intensity);
    rt_kprintf("lcd intensity:%d\n",intensity);
}

void lcd_blank(int blank)
{
    fb_entry.blank_panel(blank);
    rt_kprintf("lcd blank:%d\n",blank);
}

void lcd_test()
{
    static rt_uint8_t tt = 0;
    rt_memset(_device.phys,tt++,_device.memsize);
    _device.entry->pan_display((dma_addr_t)_device.phys+480*2);
}

FINSH_FUNCTION_EXPORT(lcd_intensity, set lcd intensity);
FINSH_FUNCTION_EXPORT(lcd_blank, set lcd blank);
FINSH_FUNCTION_EXPORT(lcd_test, test lcd);

#endif //RT_USING_FINSH
