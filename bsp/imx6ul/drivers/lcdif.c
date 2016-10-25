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

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
	PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

iomux_v3_cfg_t const lcd_pads[23] = {
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

	MX6_PAD_LCD_DATA22__MQS_RIGHT    | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_LCD_DATA23__MQS_LEFT     | MUX_PAD_CTRL(NO_PAD_CTRL),

	/*
	 * Use GPIO for Brightness adjustment, duty cycle = period.
	 */
	MX6_PAD_LCD_RESET__GPIO3_IO04   | MUX_PAD_CTRL(NO_PAD_CTRL),
};

struct lcd_panel_info_t {
	unsigned int lcdif_base_addr;
	int depth;
	struct fb_videomode mode;
};

static struct lcd_panel_info_t const displays = {
	.lcdif_base_addr = LCDIF1_BASE_ADDR,
	.depth = 16,
	.mode	= {
		.name			= "SINLINX_LCD4.3",
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
} };

struct sdlfb_device
{
    struct rt_device parent;

    GraphicDevice* entry;
    rt_uint8_t *phys[3];
    int memsize;
};

static struct sdlfb_device _device;
static volatile rt_uint32_t _frame_flg;

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

        info->bits_per_pixel = 16;
        info->pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
        info->framebuffer = device->phys[2];
        info->width = device->entry->y_res;
        info->height = device->entry->x_res;
    break; }

    case RTGRAPHIC_CTRL_RECT_UPDATE: {
        struct rt_device_rect_info *rect;
        rect = (struct rt_device_rect_info *)args;
        //不停的交替两个缓存
        if (++_frame_flg >= 2)
            _frame_flg = 0;
        rt_memcpy(device->phys[_frame_flg],device->phys[2],device->memsize);
        device->entry->pan_display((dma_addr_t)device->phys[_frame_flg]);
        break; }
    }

    return RT_EOK;
}

int lcd_init(void)
{
    int ret = 0,i;

	enable_lcdif_clock(displays.lcdif_base_addr);
	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

	mxs_lcd_panel_setup(displays.mode, displays.depth, displays.lcdif_base_addr);

    _device.entry = video_hw_init();
    _device.memsize = fb_entry.y_res * fb_entry.x_res * fb_entry.bpp / 8;
    _device.phys[0] = memalign_max(32, _device.memsize);
    _device.phys[1] = memalign_max(32, _device.memsize);
    _device.phys[2] = memalign_max(32, _device.memsize);
    //初始化启动使用第一个缓存区
    _frame_flg = 0;
#ifndef RTGUI_USING_HZ_FILE
    rt_memcpy((void *)_device.phys[0],gImage_im,_device.memsize);
#else
{
    FILE *file;
    if ((file = fopen("/etc/logo.bin", "rb")) != NULL)
    {
        fread((void *)_device.phys[0], _device.memsize, 1, file);
        fclose(file);
    }
    else
    {
        rt_memset((void *)_device.phys[0],0x00,_device.memsize);
    }
}
#endif

    _device.parent.type = RT_Device_Class_Graphic;
    _device.parent.init = sdlfb_init;
    _device.parent.open = sdlfb_open;
    _device.parent.close = sdlfb_close;
    _device.parent.read = RT_NULL;
    _device.parent.write = RT_NULL;
    _device.parent.control = sdlfb_control;

    rt_device_register(RT_DEVICE(&_device), "lcd", RT_DEVICE_FLAG_RDWR);
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

FINSH_FUNCTION_EXPORT(lcd_intensity, set lcd intensity);
FINSH_FUNCTION_EXPORT(lcd_blank, set lcd blank);

#endif //RT_USING_FINSH
