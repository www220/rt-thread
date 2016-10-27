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

#undef ALIGN
#include <common.h>
#include <malloc.h>
#include <asm/io.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/imx-common/dma.h>
#include <mxsfb.h>
#include <video_fb.h>
#include <pwm.h>

#define LIGHT_PWM	0
#define LIGHT_GPIO	IMX_GPIO_NR(3, 4)

struct lcd_panel_info_t {
	unsigned int lcdif_base_addr;
	int depth;
	struct fb_videomode mode;
};

struct mxs_platform_bl_data {
	int bl_max_intensity;
	int bl_default_intensity;
	int (*init_bl) (struct mxs_platform_bl_data *);
	int (*set_bl_intensity) (struct mxs_platform_bl_data *, int);
	void (*free_bl) (struct mxs_platform_bl_data *);
};

struct mxs_platform_fb_entry {
	struct lcd_panel_info_t info;
	GraphicDevice* display;
	int (*init_panel) (struct rt_device *, dma_addr_t, struct mxs_platform_fb_entry *);
	void (*release_panel) (struct rt_device *, struct mxs_platform_fb_entry *);
	void (*blank_panel) (int, struct mxs_platform_fb_entry *);
	void (*run_panel) (struct mxs_platform_fb_entry *);
	void (*stop_panel) (struct mxs_platform_fb_entry *);
	void (*pan_display) (dma_addr_t, struct mxs_platform_fb_entry *);
};

static int init_panel(struct rt_device *dev, dma_addr_t phys, struct mxs_platform_fb_entry *pentry)
{
	int ret = 0;
	pentry->display = video_hw_init(phys);
	if (pentry->display == 0)
	{
		ret = -1;
		goto out;
	}
	return 0;

out:
	return ret;
}

static void release_panel(struct rt_device *dev, struct mxs_platform_fb_entry *pentry)
{
	lcdif_power_down();
}

static void blank_panel(int blank, struct mxs_platform_fb_entry *pentry)
{
	struct mxs_lcdif_regs *regs = (struct mxs_lcdif_regs *)(pentry->display->isaBase);

	if (blank) {
		writel(LCDIF_CTRL_BYPASS_COUNT, &regs->hw_lcdif_ctrl_clr);
	} else {
		writel(LCDIF_CTRL_BYPASS_COUNT, &regs->hw_lcdif_ctrl_set);
	}
}

static void run_panel(struct mxs_platform_fb_entry *pentry)
{
	struct mxs_lcdif_regs *regs = (struct mxs_lcdif_regs *)(pentry->display->isaBase);

	writel(LCDIF_CTRL_RUN, &regs->hw_lcdif_ctrl_set);
}

static void stop_panel(struct mxs_platform_fb_entry *pentry)
{
	struct mxs_lcdif_regs *regs = (struct mxs_lcdif_regs *)(pentry->display->isaBase);

	writel(LCDIF_CTRL_RUN, &regs->hw_lcdif_ctrl_clr);
}

static void display_panel(dma_addr_t addr, struct mxs_platform_fb_entry *pentry)
{
	struct mxs_lcdif_regs *regs = (struct mxs_lcdif_regs *)(pentry->display->isaBase);

	writel(addr, &regs->hw_lcdif_next_buf);
}

static int init_bl(struct mxs_platform_bl_data *data)
{
#if LIGHT_PWM
	pwm_enable(4);
#endif
	return 0;
}

static void free_bl(struct mxs_platform_bl_data *data)
{
#if LIGHT_PWM
	pwm_disable(4);
#endif
}

static int values[] = { 0, 4, 9, 14, 20, 27, 35, 45, 57, 75, 100 };
static int set_bl_intensity(struct mxs_platform_bl_data *data, int intensity)
{
	int scaled_int = 100;

	if (intensity < 0)
		intensity = 0;
	if (intensity < 100)
		scaled_int = values[intensity / 10];
	if (scaled_int < 100) {
		int rem = intensity - 10 * (intensity / 10);	// r = i % 10;
		scaled_int += rem * (values[intensity / 10 + 1] -
					values[intensity / 10]) / 10;
	}

#if LIGHT_PWM
	pwm_config(4, (scaled_int*400)/100, 400);
#else
	gpio_set_value(LIGHT_GPIO, scaled_int>50);
#endif
	return 0;
}

static struct mxs_platform_bl_data bl_data = {
    .bl_max_intensity = 100,
    .bl_default_intensity = 80,
    .init_bl = init_bl,
    .free_bl = free_bl,
    .set_bl_intensity = set_bl_intensity,
};

static struct mxs_platform_fb_entry fb_entry = {
	.info = {
		.lcdif_base_addr = LCDIF1_BASE_ADDR,
		.depth = 16,
		.mode	= {
			.name           = "SINLINX_LCD4.3",
			.xres           = 480,
			.yres           = 272,
			.pixclock       = 108695,
			.left_margin    = 8,
			.right_margin   = 4,
			.upper_margin   = 2,
			.lower_margin   = 4,
			.hsync_len      = 41,
			.vsync_len      = 10,
			.sync           = 0,
			.vmode          = FB_VMODE_NONINTERLACED
		},
	},
	.init_panel = init_panel,
	.release_panel = release_panel,
	.blank_panel = blank_panel,
	.run_panel = run_panel,
	.stop_panel = stop_panel,
	.pan_display = display_panel,
};

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
	PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

iomux_v3_cfg_t const lcd_pads[] = {
	MX6_PAD_LCD_CLK__LCDIF_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_ENABLE__LCDIF_ENABLE | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_HSYNC__LCDIF_HSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_VSYNC__LCDIF_VSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA00__LCDIF_DATA00 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA01__LCDIF_DATA01 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA02__LCDIF_DATA02 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA03__LCDIF_DATA03 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA04__LCDIF_DATA04 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA05__LCDIF_DATA05 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA06__LCDIF_DATA06 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA07__LCDIF_DATA07 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA08__LCDIF_DATA08 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA09__LCDIF_DATA09 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA10__LCDIF_DATA10 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA11__LCDIF_DATA11 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA12__LCDIF_DATA12 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA13__LCDIF_DATA13 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA14__LCDIF_DATA14 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DATA15__LCDIF_DATA15 | MUX_PAD_CTRL(LCD_PAD_CTRL),

	/*
	 * Use GPIO for Brightness adjustment, duty cycle = period.
	 */
	MX6_PAD_LCD_RESET__GPIO3_IO04   | MUX_PAD_CTRL(NO_PAD_CTRL),
};

struct sdlfb_device
{
    struct rt_device parent;

    struct mxs_platform_fb_entry* entry;
    rt_uint8_t *phys[2];
    rt_uint8_t *draw;
};
static struct sdlfb_device _device;
static volatile rt_uint32_t _frame_flg;
static volatile rt_uint32_t _frame_chg;

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
    RT_ASSERT(device->entry != RT_NULL);

    switch (cmd) 
    {
    case RTGRAPHIC_CTRL_GET_INFO: {
        struct rt_device_graphic_info *info;
        info = (struct rt_device_graphic_info *) args;
        
        info->bits_per_pixel = device->entry->info.depth;
        info->pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
        info->framebuffer = device->draw;
        info->width = device->entry->info.mode.xres;
        info->height = device->entry->info.mode.yres;
    break; }
    
    case RTGRAPHIC_CTRL_RECT_UPDATE: {
        struct rt_device_rect_info *rect;
        rect = (struct rt_device_rect_info *)args;
        struct mxs_lcdif_regs *regs = (struct mxs_lcdif_regs *)(fb_entry.info.lcdif_base_addr);
        //如果缓存的结果没有输出，直接覆盖
        if (_frame_chg)
        {
            if (readl(&regs->hw_lcdif_ctrl1)&LCDIF_CTRL1_CUR_FRAME_DONE_IRQ)
                _frame_chg = 0;
            else
                rt_memcpy(device->phys[_frame_flg],device->draw,device->entry->display->memSize);
        }
        //如果画的太快都没有显示出去，直接跳过
        if (!_frame_chg)
        {
            //不停的交替两个缓存
            if (++_frame_flg >= 2)
                _frame_flg = 0;
            rt_memcpy(device->phys[_frame_flg],device->draw,device->entry->display->memSize);
            device->entry->pan_display((dma_addr_t)device->phys[_frame_flg],device->entry);
            _frame_chg = 1;
            writel(LCDIF_CTRL1_CUR_FRAME_DONE_IRQ, &regs->hw_lcdif_ctrl1_clr);
        }
        break; }
    }
    
    return RT_EOK;
}

#if 0
#include "im.h"
#endif
int lcd_init(void)
{
    int ret = 0,i;
    int memsize;

#if LIGHT_PWM
    pwm_init(4, 0, 0);
    pwm_config(4, 400, 400);
#else
    gpio_direction_output(LIGHT_GPIO , 1);
#endif

    enable_lcdif_clock(fb_entry.info.lcdif_base_addr);
    imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

    mxs_lcd_panel_setup(fb_entry.info.mode, fb_entry.info.depth, fb_entry.info.lcdif_base_addr);
    ret = bl_data.init_bl(&bl_data);
    if (ret != 0)
    {
        rt_kprintf("cannot initialize LCD backlight\n");
        goto out_panel;
    }

	_device.entry = &fb_entry;
	memsize = fb_entry.info.mode.xres * fb_entry.info.mode.yres * (fb_entry.info.depth/8);
	_device.phys[0] = rt_memalign_max(ARCH_DMA_MINALIGN, memsize);
	_device.phys[1] = rt_memalign_max(ARCH_DMA_MINALIGN, memsize);
	_device.draw = rt_malloc(memsize);

    //初始化启动使用第一个缓存区
    _frame_flg = 0;
    _frame_chg = 0;
#if 0
    rt_memcpy((void *)_device.phys[0], gImage_im, memsize);
#else
    FILE *file;
    if ((file = fopen("/resource/splash.logo", "rb")) != NULL)
    {
        fread((void *)_device.phys[0], memsize, 1, file);
        fclose(file);
    }
    else
    {
        rt_memset((void *)_device.phys[0], 0xff, memsize);
    }
#endif
    ret = fb_entry.init_panel(RT_DEVICE(&_device),(dma_addr_t)_device.phys[0],&fb_entry);
    if (ret != 0)
    {
        rt_kprintf("cannot initialize LCD panel\n");
        goto out_panel;
    }
#if LIGHT_PWM
    //延时一下让图片加载完成
    for (i=0; i<100; i+=2)
    {
        bl_data.set_bl_intensity(&bl_data,i);
        rt_thread_delay(10);
    }
    bl_data.set_bl_intensity(&bl_data,bl_data.bl_max_intensity);
#endif

    _device.parent.type = RT_Device_Class_Graphic;
    _device.parent.init = sdlfb_init;
    _device.parent.open = sdlfb_open;
    _device.parent.close = sdlfb_close;
    _device.parent.read = RT_NULL;
    _device.parent.write = RT_NULL;
    _device.parent.control = sdlfb_control;

    rt_device_register(RT_DEVICE(&_device), "lcd", RT_DEVICE_FLAG_RDWR);
    extern rt_err_t rtgui_graphic_set_device(rt_device_t device);
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
    fb_entry.blank_panel(blank, &fb_entry);
    rt_kprintf("lcd blank:%d\n",blank);
}

FINSH_FUNCTION_EXPORT(lcd_intensity, set lcd intensity);
FINSH_FUNCTION_EXPORT(lcd_blank, set lcd blank);

#endif //RT_USING_FINSH
